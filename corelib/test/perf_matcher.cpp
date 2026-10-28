// Comparison of the feature matchers, on SuperPoint features (magicleap
// weights, the ones SuperGlue and the published LightGlue were trained with):
//
//   - FLANN kd-tree with the nearest neighbor distance ratio test
//     (Vis/CorNNType=1 with Vis/CorNNDR=0.8, rtabmap's default, and 0.6; the
//     index is built on the train descriptors for each pair) and brute force
//     with cross check (Vis/CorNNType=5), as references;
//   - LightGlue, OnnxMatcher (Vis/CorNNType=10) with superpoint_magicleap_lightglue.onnx
//     from the LightGlue-ONNX releases, CPU, CUDA and TensorRT (Onnx/Provider);
//   - SuperGlue exported to ONNX by corelib/src/onnx/export_superglue_onnx.py,
//     OnnxMatcher, CPU, CUDA and TensorRT;
//   - SuperGlue run in Python by PyMatcher (Vis/CorNNType=6) with
//     corelib/src/python/rtabmap_superglue.py, CPU and CUDA (PyMatcher/Cuda);
//
// over pairs of consecutive images of data/samples. The features come from
// superpoint_magicleap.onnx (OnnxDetector) or, without it, from
// SuperPointTorch. For each matcher: the time per pair of generateKeypoints()
// excluded, the number of matches, and the inliers of a RANSAC fundamental
// matrix (3 px), as a proxy for the quality of the matches.
//
// The variants a build or machine doesn't have are skipped (WITH_ONNXRUNTIME,
// WITH_PYTHON, the assets of scripts/fetch_test_data.sh, a GPU).
//
// Its own executable, run by ctest under the "performance" label:
//   ctest -L performance
//   bin/test_matcher_perf
//   RTABMAP_PERF_FRAMES=10 bin/test_matcher_perf
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
#include "onnx/OnnxMatcher.h"
#endif
#ifdef RTABMAP_PYTHON
#include <rtabmap/core/PythonInterface.h>
#include "python/PyMatcher.h"
#endif
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

using namespace rtabmap;

#ifdef RTABMAP_PYTHON
// The embedded interpreter is initialized once, before the first test (see
// test_features2d / perf_superpoint).
class PythonInterfaceEnv : public ::testing::Environment {
public:
	void SetUp() override { PythonInterface::instance("test_matcher_perf"); }
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

struct Frame
{
	cv::Mat image;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
};

typedef std::function<std::vector<cv::DMatch>(const Frame & query, const Frame & train)> MatchFn;

struct Matcher
{
	std::string name;
	std::string device;
	MatchFn match;
};

// Features of the first frames of data/samples.
std::vector<Frame> loadFrames(int count)
{
	std::vector<Frame> frames;
	const std::string path = std::string(RTABMAP_TEST_DATA_ROOT) + "/samples";
	ParametersMap params;
	params[Parameters::kKpMaxFeatures()] = "1000";
	std::unique_ptr<Feature2D> detector;
	std::string detectorName;
	if(Feature2D::isAvailable(Feature2D::kFeatureOnnxDetector) && UFile::exists(testFile("superpoint_magicleap.onnx")))
	{
		params[Parameters::kOnnxDetectorModelPath()] = testFile("superpoint_magicleap.onnx");
		params[Parameters::kOnnxProvider()] = "0";
		detector.reset(Feature2D::create(Feature2D::kFeatureOnnxDetector, params));
		detectorName = "superpoint_magicleap.onnx";
	}
	else if(Feature2D::isAvailable(Feature2D::kFeatureSuperPointTorch) && UFile::exists(testFile("superpoint_v1.pt")))
	{
		params[Parameters::kSuperPointModelPath()] = testFile("superpoint_v1.pt");
		params[Parameters::kSuperPointCuda()] = "false";
		detector.reset(Feature2D::create(Feature2D::kFeatureSuperPointTorch, params));
		detectorName = "SuperPointTorch";
	}
	if(!detector)
	{
		return frames;
	}
	// The images are numbered: in numerical order, consecutive files are
	// consecutive frames (the directory listing would put 2.jpg after 19.jpg).
	UDirectory dir(path, "jpg png");
	std::vector<std::pair<int, std::string> > files;
	for(std::string file = dir.getNextFileName(); !file.empty(); file = dir.getNextFileName())
	{
		files.push_back(std::make_pair(atoi(file.c_str()), file));
	}
	std::sort(files.begin(), files.end());
	for(size_t i=0; i<files.size() && (int)frames.size() < count; ++i)
	{
		Frame frame;
		frame.image = cv::imread(path + "/" + files[i].second, cv::IMREAD_GRAYSCALE);
		if(frame.image.empty()) continue;
		frame.keypoints = detector->generateKeypoints(frame.image);
		frame.descriptors = detector->generateDescriptors(frame.image, frame.keypoints);
		if(!frame.keypoints.empty() && frame.descriptors.rows == (int)frame.keypoints.size())
		{
			frames.push_back(frame);
		}
	}
	printf("features: %s, %d frames\n", detectorName.c_str(), (int)frames.size());
	return frames;
}

#ifdef RTABMAP_ONNXRUNTIME
// OnnxMatcher variants of a model, one per execution provider available.
void addOnnxMatchers(std::vector<Matcher> & list, const std::string & name, const std::string & model, std::vector<std::shared_ptr<OnnxMatcher> > & keep)
{
	if(!UFile::exists(model)) return;
	struct {OnnxSession::Provider provider; bool fp16; const char * name;} providers[] = {
		{OnnxSession::kProviderCPU, false, "cpu"}, {OnnxSession::kProviderCUDA, false, "cuda"},
		{OnnxSession::kProviderTensorRT, false, "tensorrt"}, {OnnxSession::kProviderTensorRT, true, "tensorrt-fp16"}};
	for(const auto & p : providers)
	{
		if(!OnnxSession::isProviderAvailable(p.provider)) continue;
		std::string cache;
		if(p.provider == OnnxSession::kProviderTensorRT)
		{
			cache = testFile("trt_cache");
			UDirectory::makeDir(cache);
		}
		std::shared_ptr<OnnxMatcher> matcher(new OnnxMatcher(model, 0.1f, (int)p.provider, p.fp16, cache));
		if(!matcher->isModelLoaded() || matcher->session()->provider() != p.provider) continue;
		keep.push_back(matcher);
		list.push_back({name, p.name, [matcher](const Frame & q, const Frame & t) {
			return matcher->match(q.descriptors, t.descriptors, q.keypoints, t.keypoints, q.image.size(), t.image.size());
		}});
	}
}
#endif

std::vector<Matcher> matchers(std::vector<std::shared_ptr<void> > & keep)
{
	std::vector<Matcher> list;
	for(float nndr : {0.8f, 0.6f})
	{
		list.push_back({uFormat("FLANN kd-tree, NNDR %.1f", nndr), "cpu", [nndr](const Frame & q, const Frame & t) {
			std::vector<std::vector<cv::DMatch> > knn;
			cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::KDTreeIndexParams>(4), cv::makePtr<cv::flann::SearchParams>(32));
			matcher.knnMatch(q.descriptors, t.descriptors, knn, 2);
			std::vector<cv::DMatch> matches;
			for(size_t i=0; i<knn.size(); ++i)
			{
				if(knn[i].size() == 2 && knn[i][0].distance < nndr * knn[i][1].distance)
				{
					matches.push_back(knn[i][0]);
				}
			}
			return matches;
		}});
	}
	list.push_back({"brute force cross check", "cpu", [](const Frame & q, const Frame & t) {
		std::vector<cv::DMatch> matches;
		cv::BFMatcher(cv::NORM_L2SQR, true).match(q.descriptors, t.descriptors, matches);
		return matches;
	}});
#ifdef RTABMAP_ONNXRUNTIME
	std::vector<std::shared_ptr<OnnxMatcher> > onnx;
	addOnnxMatchers(list, "LightGlue (OnnxMatcher)", testFile("superpoint_magicleap_lightglue.onnx"), onnx);
	addOnnxMatchers(list, "SuperGlue (OnnxMatcher)", testFile("superglue_indoor.onnx"), onnx);
	for(size_t i=0; i<onnx.size(); ++i) keep.push_back(onnx[i]);
#endif
#ifdef RTABMAP_PYTHON
	const std::string script = envOr("RTABMAP_PERF_PYMATCHER_SCRIPT", testFile("rtabmap_superglue.py"));
	if(UFile::exists(script))
	{
		for(int cuda=0; cuda<2; ++cuda)
		{
			std::shared_ptr<PyMatcher> matcher(new PyMatcher(script, 0.2f, 20, cuda != 0, "indoor"));
			keep.push_back(matcher);
			list.push_back({"SuperGlue (PyMatcher)", cuda?"cuda":"cpu", [matcher](const Frame & q, const Frame & t) {
				return matcher->match(q.descriptors, t.descriptors, q.keypoints, t.keypoints, q.image.size());
			}});
		}
	}
#endif
	return list;
}

int ransacInliers(const Frame & q, const Frame & t, const std::vector<cv::DMatch> & matches)
{
	if(matches.size() < 8) return 0;
	std::vector<cv::Point2f> pq, pt;
	for(size_t i=0; i<matches.size(); ++i)
	{
		pq.push_back(q.keypoints[matches[i].queryIdx].pt);
		pt.push_back(t.keypoints[matches[i].trainIdx].pt);
	}
	std::vector<unsigned char> mask;
	cv::findFundamentalMat(pq, pt, cv::FM_RANSAC, 3.0, 0.99, mask);
	return (int)std::count(mask.begin(), mask.end(), 1);
}

double median(std::vector<double> values)
{
	if(values.empty()) return 0.0;
	std::sort(values.begin(), values.end());
	return values[values.size()/2];
}

}  // namespace

TEST(MatcherPerf, Matchers)
{
	const int frameCount = atoi(envOr("RTABMAP_PERF_FRAMES", "20").c_str());
	std::vector<Frame> frames = loadFrames(frameCount);
	if(frames.size() < 2)
	{
		GTEST_SKIP() << "no SuperPoint (magicleap) features: needs superpoint_magicleap.onnx or superpoint_v1.pt under " << testFile("");
	}
	std::vector<std::shared_ptr<void> > keep;
	std::vector<Matcher> list = matchers(keep);
	printf("%d pairs of consecutive frames of %dx%d, up to 1000 keypoints per frame\n\n", (int)frames.size()-1, frames[0].image.cols, frames[0].image.rows);

	printf("%-28s %-14s %10s %9s %9s %9s %9s %9s\n", "matcher", "device", "first ms", "mean ms", "median ms", "matches", "inliers", "inlier %");
	for(size_t m=0; m<list.size(); ++m)
	{
		const Matcher & matcher = list[m];
		UTimer timer;
		std::vector<cv::DMatch> warm = matcher.match(frames[0], frames[1]);
		const double firstMs = timer.ticks() * 1000.0;
		if(warm.empty())
		{
			printf("%-28s %-14s   skipped: no match (model not loaded? see the errors above)\n", matcher.name.c_str(), matcher.device.c_str());
			continue;
		}
		std::vector<double> ms;
		double matches = 0.0, inliers = 0.0;
		for(size_t f=0; f+1<frames.size(); ++f)
		{
			timer.restart();
			std::vector<cv::DMatch> result = matcher.match(frames[f+1], frames[f]);
			ms.push_back(timer.ticks() * 1000.0);
			matches += (double)result.size();
			inliers += (double)ransacInliers(frames[f+1], frames[f], result);
		}
		// a pair of frames may not overlap, the matcher must find matches over the sequence
		EXPECT_GT(inliers, 0.0) << matcher.name << " [" << matcher.device << "]";
		const double pairs = (double)(frames.size()-1);
		printf("%-28s %-14s %10.1f %9.2f %9.2f %9.0f %9.0f %8.1f%%\n", matcher.name.c_str(), matcher.device.c_str(), firstMs,
				std::accumulate(ms.begin(), ms.end(), 0.0) / pairs, median(ms), matches / pairs, inliers / pairs,
				matches > 0?100.0 * inliers / matches:0.0);
	}
	printf("\n\"inliers\": matches consistent with a RANSAC fundamental matrix (3 px) between the two frames.\n\n");
}
