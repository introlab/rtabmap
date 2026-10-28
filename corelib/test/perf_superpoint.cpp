// Comparison of the implementations of SuperPoint and of the devices they run
// on, for the two sets of weights:
//
//   rpautrat/SuperPoint (superpoint_v6_from_tf.pth, MIT)
//   - PyDetector (Kp/DetectorStrategy=15): the python script
//     corelib/src/python/rtabmap_superpoint_rpautrat.py run by the embedded
//     interpreter (torch), CPU and CUDA (PyDetector/Cuda);
//   - SuperPointRpautrat (Kp/DetectorStrategy=16): libtorch running the
//     TorchScript traced from the python model, CPU and CUDA
//     (SuperPointRpautrat/Cuda);
//   - OnnxDetector (Kp/DetectorStrategy=17): ONNX Runtime running the model
//     exported by corelib/src/onnx/export_superpoint_onnx.py, with the CPU,
//     CUDA and TensorRT execution providers (Onnx/Provider).
//
//   magicleap/SuperPointPretrainedNetwork (superpoint_v1.pth, non-commercial)
//   - PyDetector: corelib/src/python/rtabmap_superpoint.py;
//   - SuperPointTorch (Kp/DetectorStrategy=11): libtorch running the traced
//     superpoint_v1.pt, CPU and CUDA (SuperPoint/Cuda);
//   - OnnxDetector: superpoint_magicleap.onnx from the LightGlue-ONNX releases.
//
// over the images of data/samples. For each variant: the model load time, the
// first frame (warm-up, and the TensorRT engine build), then the time per
// frame of generateKeypoints() + generateDescriptors(), the number of
// keypoints, and the agreement with the first variant of the same weights
// (fraction of its keypoints found within a pixel, mean cosine similarity of
// the descriptors of those), which tells whether the implementations compute
// the same thing.
//
// The variants a build or machine doesn't have are skipped: each needs its
// backend compiled in (WITH_PYTHON, WITH_TORCH, WITH_ONNXRUNTIME) and its
// assets under data/tests (scripts/fetch_test_data.sh), the GPU ones a CUDA
// device (and the TensorRT provider for Onnx/Provider=2).
//
// Its own executable, run by ctest under the "performance" label:
//   ctest -L performance
//   bin/test_superpoint_perf
//   RTABMAP_PERF_FRAMES=10 bin/test_superpoint_perf
//   RTABMAP_PERF_PYDETECTOR_SCRIPT=~/SuperPoint/rtabmap_superpoint_rpautrat.py bin/test_superpoint_perf
//
// The times are reported rather than asserted on, as they depend on the
// machine. What is asserted is that every variant that runs detects features
// on every frame.
#include <gtest/gtest.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#ifdef RTABMAP_ONNXRUNTIME
#include "onnx/OnnxSession.h"
#endif
#ifdef RTABMAP_PYTHON
#include <rtabmap/core/PythonInterface.h>
#endif
#include <opencv2/imgcodecs.hpp>
#include <opencv2/flann.hpp>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

using namespace rtabmap;

#ifdef RTABMAP_PYTHON
// The embedded interpreter is initialized once, before the first test, like
// test_features2d does: initializing it lazily from a test body (the python
// script and the libtorch Rpautrat backend, which traces its model through
// python) ends in "take_gil: PyMUTEX_LOCK failed".
class PythonInterfaceEnv : public ::testing::Environment {
public:
	void SetUp() override { PythonInterface::instance("test_superpoint_perf"); }
};
static ::testing::Environment * const kPythonEnv =
		::testing::AddGlobalTestEnvironment(new PythonInterfaceEnv);
#endif

namespace {

std::string testFile(const std::string & name)
{
	return std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/" + name;
}

std::string envOr(const char * name, const std::string & fallback)
{
	const char * value = std::getenv(name);
	return value && *value?std::string(value):fallback;
}

struct Variant
{
	std::string weights;
	std::string name;
	std::string device;
	Feature2D::Type type;
	ParametersMap params;
	bool gpu;
};

struct Frame
{
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
};

struct Result
{
	double loadMs;
	double firstFrameMs;
	std::vector<double> frameMs;
	std::vector<Frame> frames;
	double meanKeypoints;
};

std::vector<cv::Mat> loadFrames(int count)
{
	std::vector<cv::Mat> frames;
	const std::string path = std::string(RTABMAP_TEST_DATA_ROOT) + "/samples";
	UDirectory dir(path, "jpg png");
	for(std::string file = dir.getNextFileName(); !file.empty() && (int)frames.size() < count; file = dir.getNextFileName())
	{
		cv::Mat image = cv::imread(path + "/" + file, cv::IMREAD_GRAYSCALE);
		if(!image.empty())
		{
			frames.push_back(image);
		}
	}
	return frames;
}

// OnnxDetector variants of a model, one per execution provider available.
void addOnnxVariants(std::vector<Variant> & list, const std::string & weights, const std::string & onnx, const ParametersMap & common)
{
#ifdef RTABMAP_ONNXRUNTIME
	if(!Feature2D::isAvailable(Feature2D::kFeatureOnnxDetector) || !UFile::exists(onnx)) return;
	struct {OnnxSession::Provider provider; bool fp16; const char * name;} providers[] = {
		{OnnxSession::kProviderCPU, false, "cpu"}, {OnnxSession::kProviderCUDA, false, "cuda"},
		{OnnxSession::kProviderTensorRT, false, "tensorrt"}, {OnnxSession::kProviderTensorRT, true, "tensorrt-fp16"}};
	for(const auto & p : providers)
	{
		if(!OnnxSession::isProviderAvailable(p.provider)) continue;
		ParametersMap params = common;
		params[Parameters::kOnnxDetectorModelPath()] = onnx;
		params[Parameters::kOnnxProvider()] = uNumber2Str((int)p.provider);
		if(p.provider == OnnxSession::kProviderTensorRT)
		{
			params[Parameters::kOnnxTensorRTCachePath()] = testFile("trt_cache");
			params[Parameters::kOnnxTensorRTFp16()] = p.fp16?"true":"false";
			UDirectory::makeDir(testFile("trt_cache"));
		}
		list.push_back({weights, "OnnxDetector", p.name, Feature2D::kFeatureOnnxDetector, params, p.provider != OnnxSession::kProviderCPU});
	}
#endif
}

// PyDetector variants of a script, CPU and CUDA.
void addPyDetectorVariants(std::vector<Variant> & list, const std::string & weights, const std::string & script, const ParametersMap & common)
{
	if(!Feature2D::isAvailable(Feature2D::kFeaturePyDetector) || !UFile::exists(script)) return;
	for(int cuda=0; cuda<2; ++cuda)
	{
		ParametersMap params = common;
		params[Parameters::kPyDetectorPath()] = script;
		params[Parameters::kPyDetectorCuda()] = cuda?"true":"false";
		list.push_back({weights, "PyDetector (python script)", cuda?"cuda":"cpu", Feature2D::kFeaturePyDetector, params, cuda != 0});
	}
}

// The variants this build and machine can run.
std::vector<Variant> variants()
{
	std::vector<Variant> list;
	ParametersMap common;
	common[Parameters::kKpMaxFeatures()] = "1000";

	// rpautrat weights: every implementation uses the defaults of the python
	// model (threshold 0.005, NMS radius 4)
	const std::string rpautrat = "rpautrat";
	addOnnxVariants(list, rpautrat, testFile("superpoint_rpautrat.onnx"), common);
	const std::string weights = testFile("superpoint_v6_from_tf.pth");
	const std::string model = testFile("superpoint_pytorch.py");
	if(Feature2D::isAvailable(Feature2D::kFeatureSuperPointRpautrat) && UFile::exists(weights) && UFile::exists(model))
	{
		for(int cuda=0; cuda<2; ++cuda)
		{
			ParametersMap params = common;
			params[Parameters::kSuperPointRpautratWeightsPath()] = weights;
			params[Parameters::kSuperPointRpautratModelPath()] = model;
			params[Parameters::kSuperPointRpautratCuda()] = cuda?"true":"false";
			list.push_back({rpautrat, "SuperPointRpautrat (libtorch)", cuda?"cuda":"cpu", Feature2D::kFeatureSuperPointRpautrat, params, cuda != 0});
		}
	}
	addPyDetectorVariants(list, rpautrat, envOr("RTABMAP_PERF_PYDETECTOR_SCRIPT", testFile("rtabmap_superpoint_rpautrat.py")), common);

	// magicleap weights: the python script has a threshold of 0.015 built in,
	// the other implementations are set to it (NMS radius 4 everywhere)
	const std::string magicleap = "magicleap";
	ParametersMap magicleapCommon = common;
	magicleapCommon[Parameters::kOnnxDetectorThreshold()] = "0.015";
	magicleapCommon[Parameters::kSuperPointThreshold()] = "0.015";
	addOnnxVariants(list, magicleap, testFile("superpoint_magicleap.onnx"), magicleapCommon);
	const std::string torchModel = testFile("superpoint_v1.pt");
	if(Feature2D::isAvailable(Feature2D::kFeatureSuperPointTorch) && UFile::exists(torchModel))
	{
		for(int cuda=0; cuda<2; ++cuda)
		{
			ParametersMap params = magicleapCommon;
			params[Parameters::kSuperPointModelPath()] = torchModel;
			params[Parameters::kSuperPointCuda()] = cuda?"true":"false";
			list.push_back({magicleap, "SuperPointTorch (libtorch)", cuda?"cuda":"cpu", Feature2D::kFeatureSuperPointTorch, params, cuda != 0});
		}
	}
	addPyDetectorVariants(list, magicleap, testFile("rtabmap_superpoint.py"), magicleapCommon);
	return list;
}

// Fraction of the keypoints of b with a keypoint of a within a pixel, and the
// mean cosine similarity of their descriptors.
void agreement(const Frame & a, const Frame & b, double & found, double & cosine)
{
	found = 0.0;
	cosine = 0.0;
	if(a.keypoints.empty() || b.keypoints.empty() || a.descriptors.cols != b.descriptors.cols)
	{
		return;
	}
	cv::Mat pointsA((int)a.keypoints.size(), 2, CV_32F);
	for(size_t i=0; i<a.keypoints.size(); ++i)
	{
		pointsA.at<float>((int)i, 0) = a.keypoints[i].pt.x;
		pointsA.at<float>((int)i, 1) = a.keypoints[i].pt.y;
	}
	cv::flann::Index index(pointsA, cv::flann::KDTreeIndexParams(1));
	int matched = 0;
	double cosineSum = 0.0;
	for(size_t i=0; i<b.keypoints.size(); ++i)
	{
		cv::Mat query = (cv::Mat_<float>(1,2) << b.keypoints[i].pt.x, b.keypoints[i].pt.y);
		cv::Mat indices, dists;
		index.knnSearch(query, indices, dists, 1, cv::flann::SearchParams());
		if(dists.at<float>(0,0) <= 1.0f) // squared distance
		{
			++matched;
			const cv::Mat da = a.descriptors.row(indices.at<int>(0,0));
			const cv::Mat db = b.descriptors.row((int)i);
			const double na = cv::norm(da), nb = cv::norm(db);
			cosineSum += na > 0 && nb > 0?da.dot(db) / (na * nb):0.0;
		}
	}
	found = double(matched) / double(b.keypoints.size());
	cosine = matched?cosineSum / matched:0.0;
}

double median(std::vector<double> values)
{
	if(values.empty()) return 0.0;
	std::sort(values.begin(), values.end());
	return values[values.size()/2];
}

}  // namespace

TEST(SuperPointPerf, Implementations)
{
	const int frameCount = atoi(envOr("RTABMAP_PERF_FRAMES", "30").c_str());
	std::vector<cv::Mat> frames = loadFrames(frameCount);
	ASSERT_FALSE(frames.empty()) << "no image under " << RTABMAP_TEST_DATA_ROOT << "/samples";
	std::vector<Variant> list = variants();
	if(list.empty())
	{
		GTEST_SKIP() << "no SuperPoint implementation available: build with WITH_ONNXRUNTIME / WITH_TORCH / WITH_PYTHON and run scripts/fetch_test_data.sh";
	}
	printf("\n%d frames of %dx%d from %s/samples, Kp/MaxFeatures=1000\n\n", (int)frames.size(), frames[0].cols, frames[0].rows, RTABMAP_TEST_DATA_ROOT);

	std::vector<Result> results;
	std::vector<Variant> ran;
	for(size_t v=0; v<list.size(); ++v)
	{
		const Variant & variant = list[v];
		printf("%s: %s [%s]...\n", variant.weights.c_str(), variant.name.c_str(), variant.device.c_str());
		fflush(stdout);
		UTimer timer;
		std::unique_ptr<Feature2D> detector(Feature2D::create(variant.type, variant.params));
		ASSERT_TRUE(detector.get() != NULL);
		if(variant.gpu && !detector->isGpuAvailable())
		{
			printf("  skipped: no GPU for this backend on this machine\n");
			continue;
		}
		Result result;
		result.loadMs = timer.ticks() * 1000.0;

		// warm-up on the first frame (lazy initializations, TensorRT engine build)
		{
			std::vector<cv::KeyPoint> keypoints = detector->generateKeypoints(frames[0]);
			cv::Mat descriptors = detector->generateDescriptors(frames[0], keypoints);
			result.firstFrameMs = timer.ticks() * 1000.0;
			if(keypoints.empty())
			{
				printf("  skipped: no keypoint on the first frame (model not loaded? see the errors above)\n");
				continue;
			}
		}

		bool ok = true;
		double keypointSum = 0.0;
		for(size_t f=0; f<frames.size(); ++f)
		{
			Frame frame;
			timer.restart();
			frame.keypoints = detector->generateKeypoints(frames[f]);
			frame.descriptors = detector->generateDescriptors(frames[f], frame.keypoints);
			result.frameMs.push_back(timer.ticks() * 1000.0);
			EXPECT_FALSE(frame.keypoints.empty()) << variant.name << " [" << variant.device << "] frame " << f;
			EXPECT_EQ((int)frame.keypoints.size(), frame.descriptors.rows) << variant.name << " [" << variant.device << "] frame " << f;
			ok = ok && !frame.keypoints.empty();
			keypointSum += (double)frame.keypoints.size();
			result.frames.push_back(frame);
		}
		result.meanKeypoints = keypointSum / double(frames.size());
		if(ok)
		{
			results.push_back(result);
			ran.push_back(variant);
		}
	}
	if(results.empty())
	{
		GTEST_SKIP() << "no variant could run";
	}

	printf("\n%-10s %-30s %-14s %9s %10s %9s %9s %9s %8s %10s %8s\n",
			"weights", "implementation", "device", "load ms", "first ms", "mean ms", "median ms", "min ms", "kpts", "same kpts", "cosine");
	for(size_t v=0; v<results.size(); ++v)
	{
		const Result & r = results[v];
		// reference: the first variant that ran with the same weights
		size_t ref = 0;
		while(ref < v && ran[ref].weights != ran[v].weights) ++ref;
		double found = 0.0, cosine = 0.0;
		for(size_t f=0; f<r.frames.size(); ++f)
		{
			double fr, co;
			agreement(results[ref].frames[f], r.frames[f], fr, co);
			found += fr;
			cosine += co;
		}
		found /= (double)r.frames.size();
		cosine /= (double)r.frames.size();
		printf("%-10s %-30s %-14s %9.1f %10.1f %9.2f %9.2f %9.2f %8.0f %9.1f%% %8.3f\n",
				ran[v].weights.c_str(), ran[v].name.c_str(), ran[v].device.c_str(), r.loadMs, r.firstFrameMs,
				std::accumulate(r.frameMs.begin(), r.frameMs.end(), 0.0) / (double)r.frameMs.size(),
				median(r.frameMs), *std::min_element(r.frameMs.begin(), r.frameMs.end()),
				r.meanKeypoints, 100.0 * found, cosine);
	}
	printf("\n\"same kpts\" and \"cosine\": keypoints of the row found within a pixel in the first row of the same\n"
			"weights, and the mean cosine similarity of their descriptors (1 = same implementation).\n\n");
}
