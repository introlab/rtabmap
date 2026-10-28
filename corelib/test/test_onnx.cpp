// Tests for OnnxSession / OnnxDetector / OnnxMatcher: rtabmap's ONNX Runtime
// backends for learned local features (SuperPoint, DISK, XFeat, ...) and
// matchers (LightGlue, ...).
//
// Models, all under permissive licenses, fetched or exported into data/tests/
// by scripts/fetch_test_data.sh (the tests skip when one is absent):
//  - disk.onnx + disk_lightglue.onnx: DISK and its LightGlue from the
//    LightGlue-ONNX releases (Apache-2.0), and superpoint_magicleap.onnx +
//    superpoint_magicleap_lightglue.onnx (the original SuperPoint weights,
//    non-commercial license, the pair LightGlue was trained with): the
//    detector/matcher pairs of the matcher tests, always there in CI;
//  - superpoint_rpautrat.onnx (MIT weights) and xfeat.onnx (Apache-2.0):
//    exported from the original implementations by the scripts of
//    corelib/src/onnx when torch is available;
//  - sift_lightglue.onnx: LightGlue for (Root)SIFT, exported the same way,
//    matched with rtabmap's own SIFT (keypoint scale and orientation inputs).
//
// corelib/test/CMakeLists.txt only registers this test when ONNXRuntime_FOUND,
// and a defensive #ifdef matches that.
#include <gtest/gtest.h>
#include <rtabmap/core/Version.h>

#ifdef RTABMAP_ONNXRUNTIME

#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/RegistrationInfo.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include "onnx/OnnxSession.h"   // private headers (see CMakeLists.txt include dirs)
#include "onnx/OnnxDetector.h"
#include "onnx/OnnxMatcher.h"
#include "onnx/OnnxDescriptor.h"
#include <rtabmap/core/GlobalDescriptorExtractor.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/camera/CameraImages.h>
#include "TestUtils.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cstdlib>
#include <memory>

using namespace rtabmap;

namespace {

std::string testFile(const std::string & name)
{
	return std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/" + name;
}
const std::string kDiskModel = "disk.onnx";
const std::string kDiskLightGlue = "disk_lightglue.onnx";
const std::string kMagicleapModel = "superpoint_magicleap.onnx";
const std::string kMagicleapLightGlue = "superpoint_magicleap_lightglue.onnx";
const std::string kSuperPointModel = "superpoint_rpautrat.onnx";
const std::string kXFeatModel = "xfeat.onnx";
const std::string kSiftLightGlue = "sift_lightglue.onnx";
const std::string kSuperGlue = "superglue_indoor.onnx";
const std::string kNetVLAD = "netvlad.onnx";

#define SKIP_WITHOUT(model) \
	if(!UFile::exists(testFile(model))) GTEST_SKIP() << "missing " << testFile(model) << " (run scripts/fetch_test_data.sh)"

// A detector and the matcher trained on its descriptors.
struct ModelPair
{
	std::string detector;
	std::string matcher;
};
std::ostream & operator<<(std::ostream & os, const ModelPair & pair) {return os << pair.detector << " + " << pair.matcher;}

// The detector/matcher pairs on disk, for the parameterized matcher tests.
std::vector<ModelPair> availableModelPairs()
{
	std::vector<ModelPair> pairs;
	for(const ModelPair & p : {ModelPair{kDiskModel, kDiskLightGlue}, ModelPair{kMagicleapModel, kMagicleapLightGlue}})
	{
		if(UFile::exists(testFile(p.detector)) && UFile::exists(testFile(p.matcher))) pairs.push_back(p);
	}
	if(pairs.empty())
	{
		pairs.push_back(ModelPair()); // one skipped instance rather than no instantiation
	}
	return pairs;
}

// The detector models on disk, for the parameterized detector tests.
std::vector<std::string> availableDetectorModels()
{
	std::vector<std::string> models;
	for(const std::string & m : {kDiskModel, kMagicleapModel, kSuperPointModel, kXFeatModel})
	{
		if(UFile::exists(testFile(m))) models.push_back(m);
	}
	if(models.empty())
	{
		models.push_back(""); // one skipped instance rather than no instantiation
	}
	return models;
}

cv::Mat sampleImage()
{
	cv::Mat image = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/samples/17.jpg", cv::IMREAD_GRAYSCALE);
	EXPECT_FALSE(image.empty());
	return image;
}

// CPU provider: the tests must give the same result on every host. A
// thousand keypoints: enough for the checks, and keeps LightGlue on CPU fast.
ParametersMap detectorParams(const std::string & model, int maxFeatures = 1000)
{
	ParametersMap params;
	params[Parameters::kOnnxDetectorModelPath()] = model.empty()?"":testFile(model);
	params[Parameters::kOnnxProvider()] = "0";
	params[Parameters::kKpMaxFeatures()] = std::to_string(maxFeatures);
	return params;
}

std::unique_ptr<Feature2D> createDetector(const ParametersMap & params)
{
	std::unique_ptr<Feature2D> detector(Feature2D::create(Feature2D::kFeatureOnnxDetector, params));
	EXPECT_TRUE(detector.get() != NULL);
	EXPECT_EQ(Feature2D::kFeatureOnnxDetector, detector->getType());
	return detector;
}

// Detects on the image and extracts the descriptors, like Memory / RegistrationVis do.
void detect(Feature2D & detector, const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors, const cv::Mat & mask = cv::Mat())
{
	keypoints = detector.generateKeypoints(image, mask);
	descriptors = detector.generateDescriptors(image, keypoints);
}

void expectKeypointsInImage(const std::vector<cv::KeyPoint> & keypoints, const cv::Mat & image)
{
	for(size_t i=0; i<keypoints.size(); ++i)
	{
		EXPECT_GE(keypoints[i].pt.x, 0.0f);
		EXPECT_GE(keypoints[i].pt.y, 0.0f);
		EXPECT_LT(keypoints[i].pt.x, (float)image.cols);
		EXPECT_LT(keypoints[i].pt.y, (float)image.rows);
		EXPECT_GT(keypoints[i].response, 0.0f);
	}
}

// Descriptor dimension a detector model declares.
int declaredDescriptorDim(const OnnxSession & session)
{
	int index = session.outputIndex("descriptors");
	return index >= 0?(int)session.outputs()[index].dim(2):-1;
}

// The query image is the train image translated: matched keypoints must be
// offset by the translation.
void expectTranslatedMatches(Feature2D & detector, const OnnxMatcher & matcher, float dx = 37.0f, float dy = 13.0f)
{
	cv::Mat train = sampleImage();
	cv::Mat query;
	cv::Mat affine = (cv::Mat_<double>(2,3) << 1, 0, dx, 0, 1, dy);
	cv::warpAffine(train, query, affine, train.size());

	std::vector<cv::KeyPoint> kptsTrain, kptsQuery;
	cv::Mat descTrain, descQuery;
	detect(detector, train, kptsTrain, descTrain);
	detect(detector, query, kptsQuery, descQuery);
	ASSERT_GT(kptsTrain.size(), 100u);
	ASSERT_GT(kptsQuery.size(), 100u);

	std::vector<cv::DMatch> matches = matcher.match(descQuery, descTrain, kptsQuery, kptsTrain, query.size(), train.size());
	EXPECT_GE(matches.size(), std::min(kptsTrain.size(), kptsQuery.size())/2);
	int consistent = 0;
	for(size_t i=0; i<matches.size(); ++i)
	{
		const cv::Point2f d = kptsQuery[matches[i].queryIdx].pt - kptsTrain[matches[i].trainIdx].pt;
		if(std::fabs(d.x - dx) <= 3.0f && std::fabs(d.y - dy) <= 3.0f) ++consistent;
	}
	EXPECT_GE(consistent, (int)matches.size()*8/10);
}

SensorData loadRgbdSensorData(const std::string & id)
{
	const std::string root = std::string(RTABMAP_TEST_DATA_ROOT);
	const cv::Mat depth = cv::imread(root + "/rgbd/depth/" + id + ".png", cv::IMREAD_UNCHANGED);
	const cv::Mat rgb = cv::imread(root + "/rgbd/rgb/" + id + ".jpg", cv::IMREAD_COLOR);
	CameraModel model;
	if(depth.empty() || rgb.empty() || !model.load(root + "/rgbd/calib/" + id + ".yaml"))
	{
		return SensorData();
	}
	return SensorData(rgb, depth, model);
}

// Registers the same frame with itself and two frames of the same scene.
void expectRgbdRegistration(const ParametersMap & params)
{
	SensorData from = loadRgbdSensorData("17");
	SensorData to = loadRgbdSensorData("154");
	if(from.imageRaw().empty() || to.imageRaw().empty())
	{
		GTEST_SKIP() << "missing RGB-D sample frames under " << RTABMAP_TEST_DATA_ROOT << "/rgbd";
	}
	RegistrationVis reg(params);
	ASSERT_EQ(10, reg.getNNType());

	RegistrationInfo info;
	Signature sameA(from);
	Signature sameB(from);
	Transform t = reg.computeTransformationMod(sameA, sameB, Transform(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_NEAR(0.0f, t.x(), 0.05f);
	EXPECT_NEAR(0.0f, t.y(), 0.05f);
	EXPECT_NEAR(0.0f, t.z(), 0.05f);
	EXPECT_NEAR(0.0f, t.theta(), 0.1f);
	EXPECT_GT(info.inliers, 100);
	EXPECT_GT(info.matches, 100);

	Signature sigFrom(from);
	Signature sigTo(to);
	t = reg.computeTransformationMod(sigFrom, sigTo, Transform(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_GT(info.inliers, 50);
	EXPECT_LT(t.getNorm(), 2.0f);
	EXPECT_LT(t.getAngle(Transform::getIdentity()), 1.0f);
}

ParametersMap registrationParams(const std::string & matcherModel)
{
	ParametersMap params;
	params[Parameters::kOnnxProvider()] = "0";
	params[Parameters::kVisCorNNType()] = "10";
	params[Parameters::kOnnxMatcherModelPath()] = testFile(matcherModel);
	params[Parameters::kVisMaxFeatures()] = "1000";
	params[Parameters::kVisEstimationType()] = "1"; // PnP
	params[Parameters::kVisBundleAdjustment()] = "0";
	return params;
}

}  // namespace

//////////////////////////
// OnnxSession
//////////////////////////

TEST(OnnxSession, RuntimeInfo)
{
	EXPECT_FALSE(OnnxSession::runtimeVersion().empty());
	EXPECT_TRUE(OnnxSession::isProviderAvailable(OnnxSession::kProviderCPU));
	EXPECT_EQ("CPU", OnnxSession::providerName(OnnxSession::kProviderCPU));
	EXPECT_EQ("CUDA", OnnxSession::providerName(OnnxSession::kProviderCUDA));
	EXPECT_EQ("TensorRT", OnnxSession::providerName(OnnxSession::kProviderTensorRT));
}

TEST(OnnxSession, MissingModelReturnsNull)
{
	EXPECT_TRUE(OnnxSession::get("/nonexistent/model.onnx").get() == NULL);
	EXPECT_TRUE(OnnxSession::get("").get() == NULL);
}

TEST(OnnxSession, DescribesInputsAndOutputs)
{
	SKIP_WITHOUT(kDiskModel);
	std::shared_ptr<OnnxSession> session = OnnxSession::get(testFile(kDiskModel));
	ASSERT_TRUE(session.get() != NULL);
	EXPECT_EQ(OnnxSession::kProviderCPU, session->provider());
	EXPECT_EQ(testFile(kDiskModel), session->modelPath());

	// DISK takes an RGB image of any size
	ASSERT_EQ(1u, session->inputs().size());
	EXPECT_EQ("image", session->inputs()[0].name);
	EXPECT_EQ(OnnxTensor::kFloat32, session->inputs()[0].type);
	ASSERT_EQ(4u, session->inputs()[0].shape.size());
	EXPECT_EQ(1, session->inputs()[0].shape[0]);
	EXPECT_EQ(3, session->inputs()[0].shape[1]);
	EXPECT_EQ(-1, session->inputs()[0].shape[2]); // dynamic height
	EXPECT_EQ(-1, session->inputs()[0].shape[3]); // dynamic width
	EXPECT_FALSE(session->inputs()[0].isStatic());
	EXPECT_EQ(0u, session->inputs()[0].count());

	ASSERT_EQ(3u, session->outputs().size());
	EXPECT_EQ(0, session->outputIndex("keypoints"));
	EXPECT_EQ(1, session->outputIndex("scores"));
	EXPECT_EQ(2, session->outputIndex("descriptors"));
	EXPECT_EQ(-1, session->outputIndex("nope"));
	EXPECT_EQ(0, session->inputIndex("image"));
	EXPECT_EQ(-1, session->inputIndex("nope"));
	EXPECT_EQ(128, declaredDescriptorDim(*session));
	EXPECT_NE(std::string::npos, session->inputs()[0].toString().find("image float32 [1,3,-1,-1]"));
}

// Two users of the same model and options share one session; different
// options get another one.
TEST(OnnxSession, SharedBetweenUsers)
{
	SKIP_WITHOUT(kDiskModel);
	std::shared_ptr<OnnxSession> a = OnnxSession::get(testFile(kDiskModel));
	std::shared_ptr<OnnxSession> b = OnnxSession::get(testFile(kDiskModel));
	ASSERT_TRUE(a.get() != NULL);
	EXPECT_EQ(a.get(), b.get());
	EXPECT_EQ(2, a.use_count());

	OnnxSession::Options options;
	options.tensorRTFp16 = true; // ignored on CPU but part of the key
	std::shared_ptr<OnnxSession> c = OnnxSession::get(testFile(kDiskModel), options);
	ASSERT_TRUE(c.get() != NULL);
	EXPECT_NE(a.get(), c.get());

	// Once every user is gone, the next get() loads it again.
	a.reset();
	b.reset();
	std::shared_ptr<OnnxSession> d = OnnxSession::get(testFile(kDiskModel));
	ASSERT_TRUE(d.get() != NULL);
	EXPECT_EQ(1, d.use_count());
}

// A requested provider the library doesn't have falls back to CPU (with a
// warning) instead of failing to load the model.
TEST(OnnxSession, UnavailableProviderFallsBackToCpu)
{
	SKIP_WITHOUT(kDiskModel);
	if(OnnxSession::isProviderAvailable(OnnxSession::kProviderCUDA) && OnnxSession::isProviderAvailable(OnnxSession::kProviderTensorRT))
	{
		GTEST_SKIP() << "every provider is available in this ONNX Runtime library";
	}
	OnnxSession::Options options;
	options.provider = OnnxSession::isProviderAvailable(OnnxSession::kProviderCUDA)?OnnxSession::kProviderTensorRT:OnnxSession::kProviderCUDA;
	std::shared_ptr<OnnxSession> session = OnnxSession::get(testFile(kDiskModel), options);
	ASSERT_TRUE(session.get() != NULL);
	EXPECT_NE(options.provider, session->provider());
	EXPECT_EQ(3u, session->outputs().size());
}

TEST(OnnxSession, RunRejectsBadInputs)
{
	SKIP_WITHOUT(kDiskModel);
	std::shared_ptr<OnnxSession> session = OnnxSession::get(testFile(kDiskModel));
	ASSERT_TRUE(session.get() != NULL);
	std::vector<OnnxTensor> outputs;

	// wrong count
	EXPECT_FALSE(session->run(std::vector<OnnxTensor>(), outputs));
	EXPECT_TRUE(outputs.empty());

	// wrong name
	std::vector<OnnxTensor> inputs(1);
	inputs[0].name = "nope";
	inputs[0].shape = {1,3,16,16};
	inputs[0].f.assign(3*16*16, 0.0f);
	EXPECT_FALSE(session->run(inputs, outputs));

	// wrong type
	inputs[0].name = "image";
	inputs[0].type = OnnxTensor::kUInt8;
	inputs[0].u8.assign(3*16*16, 0);
	EXPECT_FALSE(session->run(inputs, outputs));

	// a blank image gives valid outputs
	inputs[0].type = OnnxTensor::kFloat32;
	inputs[0].shape = {1,3,64,64};
	inputs[0].f.assign(3*64*64, 0.5f);
	ASSERT_TRUE(session->run(inputs, outputs));
	ASSERT_EQ(3u, outputs.size());
	EXPECT_EQ("keypoints", outputs[0].name);
	EXPECT_TRUE(outputs[0].type == OnnxTensor::kInt64 || outputs[0].type == OnnxTensor::kFloat32);
	ASSERT_EQ(3u, outputs[0].shape.size());
	EXPECT_EQ(outputs[0].shape[1], outputs[2].shape[1]);
	EXPECT_EQ(128, outputs[2].shape[2]);
	EXPECT_EQ(outputs[2].count(), outputs[2].f.size());
}

// The CUDA provider (an onnxruntime-*-gpu library with CUDA/cuDNN installed)
// gives the same features and matches as the CPU one, up to floating point
// differences. Skipped when the ONNX Runtime library has no CUDA provider.
TEST(OnnxSession, CudaProviderMatchesCpu)
{
	SKIP_WITHOUT(kDiskModel);
	SKIP_WITHOUT(kDiskLightGlue);
	if(!OnnxSession::isProviderAvailable(OnnxSession::kProviderCUDA))
	{
		GTEST_SKIP() << "CUDA execution provider not available in this ONNX Runtime build";
	}
	cv::Mat image = sampleImage();
	std::unique_ptr<Feature2D> cpu = createDetector(detectorParams(kDiskModel));
	ASSERT_TRUE(cpu->isGpuAvailable());
	ParametersMap params = detectorParams(kDiskModel);
	params[Parameters::kOnnxProvider()] = "1";
	std::unique_ptr<Feature2D> cuda = createDetector(params);
	ASSERT_TRUE(static_cast<OnnxDetector*>(cuda.get())->isModelLoaded());
	if(static_cast<OnnxDetector*>(cuda.get())->session()->provider() != OnnxSession::kProviderCUDA)
	{
		GTEST_SKIP() << "CUDA provider fell back to CPU (no CUDA device or cuDNN?)";
	}

	std::vector<cv::KeyPoint> kptsCpu, kptsCuda;
	cv::Mat descCpu, descCuda;
	detect(*cpu, image, kptsCpu, descCpu);
	detect(*cuda, image, kptsCuda, descCuda);
	ASSERT_GT(kptsCpu.size(), 100u);
	EXPECT_NEAR((double)kptsCpu.size(), (double)kptsCuda.size(), kptsCpu.size()*0.05);
	ASSERT_EQ((int)kptsCuda.size(), descCuda.rows);
	EXPECT_EQ(descCpu.cols, descCuda.cols);

	OnnxMatcher matcherCpu(testFile(kDiskLightGlue), 0.1f, 0);
	OnnxMatcher matcherCuda(testFile(kDiskLightGlue), 0.1f, 1);
	ASSERT_TRUE(matcherCuda.isModelLoaded());
	EXPECT_EQ(OnnxSession::kProviderCUDA, matcherCuda.session()->provider());
	std::vector<cv::DMatch> matchesCpu = matcherCpu.match(descCpu, descCpu, kptsCpu, kptsCpu, image.size());
	std::vector<cv::DMatch> matchesCuda = matcherCuda.match(descCpu, descCpu, kptsCpu, kptsCpu, image.size());
	EXPECT_NEAR((double)matchesCpu.size(), (double)matchesCuda.size(), matchesCpu.size()*0.05);
	int identity = 0;
	for(size_t i=0; i<matchesCuda.size(); ++i)
	{
		if(matchesCuda[i].queryIdx == matchesCuda[i].trainIdx) ++identity;
	}
	EXPECT_GE(identity, (int)matchesCuda.size()*9/10);
}

// The TensorRT provider: opt-in with RTABMAP_TEST_TENSORRT=1 as building the
// engines takes minutes (cached under data/tests/trt_cache for the next run).
TEST(OnnxSession, TensorRTProviderMatchesCpu)
{
	SKIP_WITHOUT(kDiskModel);
	SKIP_WITHOUT(kDiskLightGlue);
	if(!std::getenv("RTABMAP_TEST_TENSORRT"))
	{
		GTEST_SKIP() << "set RTABMAP_TEST_TENSORRT=1 to run (builds TensorRT engines, minutes)";
	}
	if(!OnnxSession::isProviderAvailable(OnnxSession::kProviderTensorRT))
	{
		GTEST_SKIP() << "TensorRT execution provider not available in this ONNX Runtime build";
	}
	// show the engine build warnings of OnnxSession
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);
	cv::Mat image = sampleImage();
	std::unique_ptr<Feature2D> cpu = createDetector(detectorParams(kDiskModel));
	ParametersMap params = detectorParams(kDiskModel);
	params[Parameters::kOnnxProvider()] = "2";
	params[Parameters::kOnnxTensorRTCachePath()] = testFile("trt_cache");
	UDirectory::makeDir(testFile("trt_cache"));
	std::unique_ptr<Feature2D> trt = createDetector(params);
	ASSERT_TRUE(static_cast<OnnxDetector*>(trt.get())->isModelLoaded());
	if(static_cast<OnnxDetector*>(trt.get())->session()->provider() != OnnxSession::kProviderTensorRT)
	{
		GTEST_SKIP() << "TensorRT provider fell back (TensorRT libraries not found?)";
	}

	std::vector<cv::KeyPoint> kptsCpu, kptsTrt;
	cv::Mat descCpu, descTrt;
	detect(*cpu, image, kptsCpu, descCpu);
	detect(*trt, image, kptsTrt, descTrt);
	ASSERT_GT(kptsCpu.size(), 100u);
	EXPECT_NEAR((double)kptsCpu.size(), (double)kptsTrt.size(), kptsCpu.size()*0.1);
	ASSERT_EQ((int)kptsTrt.size(), descTrt.rows);

	OnnxMatcher matcherTrt(testFile(kDiskLightGlue), 0.1f, 2, false, testFile("trt_cache"));
	ASSERT_TRUE(matcherTrt.isModelLoaded());
	EXPECT_EQ(OnnxSession::kProviderTensorRT, matcherTrt.session()->provider());
	std::vector<cv::DMatch> matches = matcherTrt.match(descCpu, descCpu, kptsCpu, kptsCpu, image.size());
	EXPECT_GE(matches.size(), kptsCpu.size()*8/10);
	int identity = 0;
	for(size_t i=0; i<matches.size(); ++i)
	{
		if(matches[i].queryIdx == matches[i].trainIdx) ++identity;
	}
	EXPECT_GE(identity, (int)matches.size()*9/10);

	// Without a cache path, the engines go to the "tensorrt_cache" folder of
	// the working directory given (or the default one).
	ParametersMap defaults = detectorParams(kDiskModel);
	defaults[Parameters::kOnnxProvider()] = "2";
	defaults[Parameters::kRtabmapWorkingDirectory()] = testFile("");
	EXPECT_EQ(testFile("") + "/tensorrt_cache", OnnxSession::defaultTensorRTCachePath(defaults));
	EXPECT_TRUE(UDirectory::exists(testFile("tensorrt_cache")));
	ULogger::setType(ULogger::kTypeNoLog);
}

//////////////////////////
// OnnxDetector
//////////////////////////

TEST(OnnxDetector, IsAvailable)
{
	EXPECT_TRUE(Feature2D::isAvailable(Feature2D::kFeatureOnnxDetector));
	EXPECT_EQ("OnnxDetector", Feature2D::typeName(Feature2D::kFeatureOnnxDetector));
}

// A bad path logs an error and the detector returns no features, it doesn't crash.
TEST(OnnxDetector, MissingModelReturnsEmpty)
{
	ParametersMap params = detectorParams("");
	params[Parameters::kOnnxDetectorModelPath()] = "/nonexistent/model.onnx";
	std::unique_ptr<Feature2D> detector = createDetector(params);
	EXPECT_FALSE(static_cast<OnnxDetector*>(detector.get())->isModelLoaded());
	cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC1);
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	detect(*detector, image, keypoints, descriptors);
	EXPECT_TRUE(keypoints.empty());
	EXPECT_TRUE(descriptors.empty());

	// Also with the parameter not set at all.
	params.erase(Parameters::kOnnxDetectorModelPath());
	std::unique_ptr<Feature2D> detector2 = createDetector(params);
	EXPECT_FALSE(static_cast<OnnxDetector*>(detector2.get())->isModelLoaded());
	EXPECT_TRUE(detector2->generateKeypoints(image).empty());
}

// The detector tests run on every detector model on disk.
class OnnxDetectorModel : public ::testing::TestWithParam<std::string>
{
protected:
	void SetUp() override
	{
		if(GetParam().empty())
		{
			GTEST_SKIP() << "no detector model under " << testFile("") << " (run scripts/fetch_test_data.sh)";
		}
	}
	const std::string & model() const {return GetParam();}
};

std::string modelName(const ::testing::TestParamInfo<std::string> & info)
{
	std::string name = info.param.empty()?"none":info.param.substr(0, info.param.find('.'));
	std::replace(name.begin(), name.end(), '-', '_');
	return name;
}

INSTANTIATE_TEST_SUITE_P(Models, OnnxDetectorModel, ::testing::ValuesIn(availableDetectorModels()), modelName);

TEST_P(OnnxDetectorModel, BasicDetection)
{
	std::unique_ptr<Feature2D> detector = createDetector(detectorParams(model()));
	OnnxDetector * onnxDetector = static_cast<OnnxDetector*>(detector.get());
	ASSERT_TRUE(onnxDetector->isModelLoaded());
	EXPECT_EQ(testFile(model()), onnxDetector->modelPath());
	EXPECT_EQ(0, onnxDetector->provider());
	ASSERT_TRUE(onnxDetector->session().get() != NULL);
	EXPECT_EQ(OnnxSession::kProviderCPU, onnxDetector->session()->provider());
	const int dim = declaredDescriptorDim(*onnxDetector->session()); // -1 when the model doesn't declare it

	cv::Mat image = sampleImage();
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	detect(*detector, image, keypoints, descriptors);

	EXPECT_GT(keypoints.size(), 100u);
	ASSERT_EQ((int)keypoints.size(), descriptors.rows);
	EXPECT_GT(descriptors.cols, 0);
	if(dim > 0) EXPECT_EQ(dim, descriptors.cols);
	EXPECT_EQ(CV_32F, descriptors.type());
	expectKeypointsInImage(keypoints, image);

	// These networks give L2-normalized descriptors.
	for(int i=0; i<descriptors.rows; i+=50)
	{
		EXPECT_NEAR(1.0, cv::norm(descriptors.row(i)), 1e-3);
	}

	// Deterministic on CPU.
	std::vector<cv::KeyPoint> keypoints2;
	cv::Mat descriptors2;
	detect(*detector, image, keypoints2, descriptors2);
	ASSERT_EQ(keypoints.size(), keypoints2.size());
	EXPECT_EQ(keypoints[0].pt, keypoints2[0].pt);
	EXPECT_EQ(0.0, cv::norm(descriptors, descriptors2));
}

TEST_P(OnnxDetectorModel, MaskFiltersKeypoints)
{
	std::unique_ptr<Feature2D> detector = createDetector(detectorParams(model()));
	cv::Mat image = sampleImage();
	cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
	mask(cv::Rect(image.cols/2, 0, image.cols - image.cols/2, image.rows)).setTo(255);

	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	detect(*detector, image, keypoints, descriptors, mask);
	EXPECT_GT(keypoints.size(), 20u);
	EXPECT_EQ((int)keypoints.size(), descriptors.rows);
	for(size_t i=0; i<keypoints.size(); ++i)
	{
		EXPECT_GE(keypoints[i].pt.x, (float)(image.cols/2));
	}

	// Everything masked out: no features, no crash.
	mask.setTo(0);
	detect(*detector, image, keypoints, descriptors, mask);
	EXPECT_TRUE(keypoints.empty());
	EXPECT_TRUE(descriptors.empty());
}

TEST_P(OnnxDetectorModel, MaxFeaturesCap)
{
	cv::Mat image = sampleImage();
	std::unique_ptr<Feature2D> detectorAll = createDetector(detectorParams(model(), 0));
	std::vector<cv::KeyPoint> all = detectorAll->generateKeypoints(image);
	ASSERT_GT(all.size(), 50u);

	std::unique_ptr<Feature2D> detector = createDetector(detectorParams(model(), 50));
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	detect(*detector, image, keypoints, descriptors);
	EXPECT_EQ(50u, keypoints.size());
	EXPECT_EQ(50, descriptors.rows);

	// The strongest keypoints are kept.
	std::vector<float> responses;
	for(size_t i=0; i<all.size(); ++i) responses.push_back(all[i].response);
	std::sort(responses.begin(), responses.end(), std::greater<float>());
	for(size_t i=0; i<keypoints.size(); ++i)
	{
		EXPECT_GE(keypoints[i].response, responses[49]);
	}

	// SSC
	ParametersMap params = detectorParams(model(), 50);
	params[Parameters::kKpSSC()] = "true";
	std::unique_ptr<Feature2D> detectorSsc = createDetector(params);
	detect(*detectorSsc, image, keypoints, descriptors);
	EXPECT_LE(keypoints.size(), 50u);
	EXPECT_EQ((int)keypoints.size(), descriptors.rows);
}

TEST_P(OnnxDetectorModel, ThresholdFiltersScores)
{
	std::unique_ptr<Feature2D> detector = createDetector(detectorParams(model(), 0));
	cv::Mat image = sampleImage();
	std::vector<cv::KeyPoint> all = detector->generateKeypoints(image);
	ASSERT_GT(all.size(), 10u);

	// A threshold at the median response keeps about half of the keypoints.
	std::vector<float> responses;
	for(size_t i=0; i<all.size(); ++i) responses.push_back(all[i].response);
	std::sort(responses.begin(), responses.end());
	const float threshold = responses[responses.size()/2];

	ParametersMap params = detectorParams(model(), 0);
	params[Parameters::kOnnxDetectorThreshold()] = uNumber2Str(threshold, 9);
	detector->parseParameters(params);
	EXPECT_NEAR(threshold, static_cast<OnnxDetector*>(detector.get())->threshold(), 1e-7f);
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	detect(*detector, image, keypoints, descriptors);
	EXPECT_LT(keypoints.size(), all.size());
	EXPECT_GT(keypoints.size(), 0u);
	EXPECT_EQ((int)keypoints.size(), descriptors.rows);
	for(size_t i=0; i<keypoints.size(); ++i)
	{
		EXPECT_GE(keypoints[i].response, threshold);
	}
}

// The base class runs the detector once per grid cell: the descriptors must
// follow the keypoints of all the cells.
TEST_P(OnnxDetectorModel, GridKeepsDescriptorsInSync)
{
	ParametersMap params = detectorParams(model(), 200);
	params[Parameters::kKpGridRows()] = "2";
	params[Parameters::kKpGridCols()] = "2";
	std::unique_ptr<Feature2D> detector = createDetector(params);
	cv::Mat image = sampleImage();
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	detect(*detector, image, keypoints, descriptors);
	EXPECT_GT(keypoints.size(), 100u);
	EXPECT_LE(keypoints.size(), 200u);
	ASSERT_EQ((int)keypoints.size(), descriptors.rows);
	expectKeypointsInImage(keypoints, image);

	// Each cell contributes.
	int quadrants[4] = {0,0,0,0};
	for(size_t i=0; i<keypoints.size(); ++i)
	{
		quadrants[(keypoints[i].pt.y >= image.rows/2 ? 2 : 0) + (keypoints[i].pt.x >= image.cols/2 ? 1 : 0)]++;
	}
	for(int q=0; q<4; ++q)
	{
		EXPECT_GT(quadrants[q], 0) << "quadrant " << q;
		EXPECT_LE(quadrants[q], 50) << "quadrant " << q;
	}

	// The descriptors of a cell are the ones computed on the full image
	// cropped to it: close to the ones without a grid for the same keypoints
	// (not equal: the crop changes the receptive field near the cell borders).
	std::unique_ptr<Feature2D> detectorNoGrid = createDetector(detectorParams(model(), 0));
	std::vector<cv::KeyPoint> keypointsNoGrid;
	cv::Mat descriptorsNoGrid;
	detect(*detectorNoGrid, image, keypointsNoGrid, descriptorsNoGrid);
	int found = 0;
	for(size_t i=0; i<keypoints.size() && found < 10; ++i)
	{
		for(size_t j=0; j<keypointsNoGrid.size(); ++j)
		{
			if(keypoints[i].pt == keypointsNoGrid[j].pt)
			{
				EXPECT_LT(cv::norm(descriptors.row((int)i), descriptorsNoGrid.row((int)j)), 1.0);
				++found;
				break;
			}
		}
	}
	EXPECT_GT(found, 0);
}

TEST_P(OnnxDetectorModel, RoiRatios)
{
	ParametersMap params = detectorParams(model());
	params[Parameters::kKpRoiRatios()] = "0 0 0 0.5"; // bottom half removed
	std::unique_ptr<Feature2D> detector = createDetector(params);
	cv::Mat image = sampleImage();
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	detect(*detector, image, keypoints, descriptors);
	EXPECT_GT(keypoints.size(), 20u);
	ASSERT_EQ((int)keypoints.size(), descriptors.rows);
	for(size_t i=0; i<keypoints.size(); ++i)
	{
		EXPECT_LT(keypoints[i].pt.y, (float)(image.rows/2));
	}
}

// Descriptors requested for other keypoints than the ones detected are an
// error (there is no "compute" pass in these models).
TEST_P(OnnxDetectorModel, MismatchedKeypointsGiveEmptyDescriptors)
{
	std::unique_ptr<Feature2D> detector = createDetector(detectorParams(model()));
	cv::Mat image = sampleImage();
	std::vector<cv::KeyPoint> keypoints = detector->generateKeypoints(image);
	ASSERT_GT(keypoints.size(), 1u);
	keypoints.pop_back();
	EXPECT_TRUE(detector->generateDescriptors(image, keypoints).empty());
}

// Changing a parameter that doesn't affect the model keeps the session;
// changing the model path reloads it.
TEST_P(OnnxDetectorModel, ReparseParameters)
{
	std::unique_ptr<Feature2D> detector = createDetector(detectorParams(model()));
	OnnxDetector * onnxDetector = static_cast<OnnxDetector*>(detector.get());
	const OnnxSession * session = onnxDetector->session().get();
	ASSERT_TRUE(session != NULL);

	ParametersMap params;
	params[Parameters::kOnnxDetectorThreshold()] = "0.01";
	params[Parameters::kKpMaxFeatures()] = "10";
	detector->parseParameters(params);
	EXPECT_EQ(session, onnxDetector->session().get());
	EXPECT_EQ(10, detector->getMaxFeatures());

	params.clear();
	params[Parameters::kOnnxDetectorModelPath()] = "/nonexistent/model.onnx";
	detector->parseParameters(params);
	EXPECT_FALSE(onnxDetector->isModelLoaded());

	params[Parameters::kOnnxDetectorModelPath()] = testFile(model());
	detector->parseParameters(params);
	EXPECT_TRUE(onnxDetector->isModelLoaded());
	EXPECT_EQ(10u, detector->generateKeypoints(sampleImage()).size());
}

//////////////////////////
// OnnxMatcher
//////////////////////////

TEST(OnnxMatcher, NormalizeKeypoints)
{
	std::vector<cv::KeyPoint> keypoints;
	keypoints.push_back(cv::KeyPoint(320, 240, 1));
	keypoints.push_back(cv::KeyPoint(0, 0, 1));
	keypoints.push_back(cv::KeyPoint(640, 480, 1));
	std::vector<float> xy;
	OnnxMatcher::normalizeKeypoints(keypoints, cv::Size(640, 480), xy);
	ASSERT_EQ(6u, xy.size());
	EXPECT_FLOAT_EQ(0.0f, xy[0]);
	EXPECT_FLOAT_EQ(0.0f, xy[1]);
	EXPECT_FLOAT_EQ(-1.0f, xy[2]);
	EXPECT_FLOAT_EQ(-0.75f, xy[3]);
	EXPECT_FLOAT_EQ(1.0f, xy[4]);
	EXPECT_FLOAT_EQ(0.75f, xy[5]);
}

TEST(OnnxMatcher, PackKeypointsWithScaleAndOrientation)
{
	std::vector<cv::KeyPoint> keypoints;
	keypoints.push_back(cv::KeyPoint(320, 240, 3.5f, 90.0f));  // size, angle in degrees
	keypoints.push_back(cv::KeyPoint(0, 0, 12.0f, -1.0f));     // OpenCV: angle not computed
	std::vector<float> data;
	OnnxMatcher::packKeypoints(keypoints, cv::Size(640, 480), 2, data);
	ASSERT_EQ(4u, data.size());
	EXPECT_FLOAT_EQ(0.0f, data[0]);
	EXPECT_FLOAT_EQ(-1.0f, data[2]);
	OnnxMatcher::packKeypoints(keypoints, cv::Size(640, 480), 4, data);
	ASSERT_EQ(8u, data.size());
	EXPECT_FLOAT_EQ(0.0f, data[0]);
	EXPECT_FLOAT_EQ(0.0f, data[1]);
	EXPECT_FLOAT_EQ(3.5f, data[2]);
	EXPECT_NEAR(CV_PI/2.0, data[3], 1e-6);
	EXPECT_FLOAT_EQ(-1.0f, data[4]);
	EXPECT_FLOAT_EQ(-0.75f, data[5]);
	EXPECT_FLOAT_EQ(12.0f, data[6]);
	EXPECT_FLOAT_EQ(0.0f, data[7]);
}

TEST(OnnxMatcher, MissingModelReturnsEmpty)
{
	OnnxMatcher matcher("/nonexistent/model.onnx", 0.1f, 0);
	EXPECT_FALSE(matcher.isModelLoaded());
	EXPECT_EQ("/nonexistent/model.onnx", matcher.modelPath());
	EXPECT_FLOAT_EQ(0.1f, matcher.matchThreshold());
	EXPECT_EQ(0, matcher.provider());
	cv::Mat desc = cv::Mat::ones(3, 128, CV_32F);
	std::vector<cv::KeyPoint> kpts(3, cv::KeyPoint(10, 10, 8));
	EXPECT_TRUE(matcher.match(desc, desc, kpts, kpts, cv::Size(640, 480)).empty());

	OnnxMatcher empty("");
	EXPECT_FALSE(empty.isModelLoaded());
}

// The matcher tests run on every detector/matcher pair on disk.
class OnnxMatcherPair : public ::testing::TestWithParam<ModelPair>
{
protected:
	void SetUp() override
	{
		if(GetParam().detector.empty())
		{
			GTEST_SKIP() << "no detector/matcher pair under " << testFile("") << " (run scripts/fetch_test_data.sh)";
		}
	}
	const std::string & detectorModel() const {return GetParam().detector;}
	const std::string & matcherModel() const {return GetParam().matcher;}
};

std::string pairName(const ::testing::TestParamInfo<ModelPair> & info)
{
	std::string name = info.param.detector.empty()?"none":info.param.detector.substr(0, info.param.detector.find('.'));
	std::replace(name.begin(), name.end(), '-', '_');
	return name;
}

INSTANTIATE_TEST_SUITE_P(Pairs, OnnxMatcherPair, ::testing::ValuesIn(availableModelPairs()), pairName);

TEST_P(OnnxMatcherPair, DescribesInputsAndOutputs)
{
	OnnxMatcher matcher(testFile(matcherModel()), 0.1f, 0);
	ASSERT_TRUE(matcher.isModelLoaded());
	const OnnxSession & session = *matcher.session();
	EXPECT_EQ(0, session.inputIndex("kpts0"));
	EXPECT_EQ(1, session.inputIndex("kpts1"));
	EXPECT_EQ(2, session.inputIndex("desc0"));
	EXPECT_EQ(3, session.inputIndex("desc1"));
	EXPECT_EQ(2, session.inputs()[0].dim(2));
	EXPECT_GT(session.inputs()[2].dim(2), 0);
	EXPECT_GE(session.outputIndex("matches0"), 0);
	EXPECT_GE(session.outputIndex("mscores0"), 0);
}

TEST_P(OnnxMatcherPair, InvalidInputsReturnEmpty)
{
	OnnxMatcher matcher(testFile(matcherModel()), 0.1f, 0);
	ASSERT_TRUE(matcher.isModelLoaded());
	const int dim = (int)matcher.session()->inputs()[2].dim(2);
	ASSERT_GT(dim, 0);
	std::vector<cv::KeyPoint> kpts(3, cv::KeyPoint(10, 10, 8));

	// empty
	EXPECT_TRUE(matcher.match(cv::Mat(), cv::Mat(), std::vector<cv::KeyPoint>(), std::vector<cv::KeyPoint>(), cv::Size(640, 480)).empty());
	// binary descriptors
	cv::Mat binary = cv::Mat::ones(3, 32, CV_8U);
	EXPECT_TRUE(matcher.match(binary, binary, kpts, kpts, cv::Size(640, 480)).empty());
	// wrong dimension for the model
	cv::Mat wrongDim = cv::Mat::ones(3, dim + 64, CV_32F);
	EXPECT_TRUE(matcher.match(wrongDim, wrongDim, kpts, kpts, cv::Size(640, 480)).empty());
	// keypoints/descriptors count mismatch
	cv::Mat desc = cv::Mat::ones(3, dim, CV_32F);
	std::vector<cv::KeyPoint> kpts2(2, cv::KeyPoint(10, 10, 8));
	EXPECT_TRUE(matcher.match(desc, desc, kpts2, kpts, cv::Size(640, 480)).empty());
	// no image size
	EXPECT_TRUE(matcher.match(desc, desc, kpts, kpts, cv::Size()).empty());
}

TEST_P(OnnxMatcherPair, SameFeaturesMatchThemselves)
{
	std::unique_ptr<Feature2D> detector = createDetector(detectorParams(detectorModel()));
	cv::Mat image = sampleImage();
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	detect(*detector, image, keypoints, descriptors);
	ASSERT_GT(keypoints.size(), 100u);

	OnnxMatcher matcher(testFile(matcherModel()), 0.1f, 0);
	ASSERT_TRUE(matcher.isModelLoaded());
	std::vector<cv::DMatch> matches = matcher.match(descriptors, descriptors, keypoints, keypoints, image.size());
	EXPECT_GE(matches.size(), keypoints.size()*9/10);
	int identity = 0;
	for(size_t i=0; i<matches.size(); ++i)
	{
		EXPECT_GE(matches[i].queryIdx, 0);
		EXPECT_LT(matches[i].queryIdx, (int)keypoints.size());
		EXPECT_GE(matches[i].trainIdx, 0);
		EXPECT_LT(matches[i].trainIdx, (int)keypoints.size());
		EXPECT_GE(matches[i].distance, 0.0f);
		EXPECT_LE(matches[i].distance, 0.9f); // score >= threshold
		if(matches[i].queryIdx == matches[i].trainIdx) ++identity;
	}
	EXPECT_GE(identity, (int)matches.size()*9/10);
}

TEST_P(OnnxMatcherPair, TranslatedImageMatches)
{
	std::unique_ptr<Feature2D> detector = createDetector(detectorParams(detectorModel()));
	OnnxMatcher matcher(testFile(matcherModel()), 0.1f, 0);
	ASSERT_TRUE(matcher.isModelLoaded());
	expectTranslatedMatches(*detector, matcher);

	// A stricter threshold keeps fewer matches, all with a higher score.
	cv::Mat image = sampleImage();
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	detect(*detector, image, keypoints, descriptors);
	OnnxMatcher strict(testFile(matcherModel()), 0.99f, 0);
	std::vector<cv::DMatch> matches = matcher.match(descriptors, descriptors, keypoints, keypoints, image.size());
	std::vector<cv::DMatch> strictMatches = strict.match(descriptors, descriptors, keypoints, keypoints, image.size());
	EXPECT_LE(strictMatches.size(), matches.size());
	for(size_t i=0; i<strictMatches.size(); ++i)
	{
		EXPECT_LE(strictMatches[i].distance, 0.01f + 1e-5f);
	}
}

// LightGlue for SIFT takes the keypoint scale and orientation too (kpts with
// 4 columns), filled from rtabmap's own SIFT keypoints; it was trained on
// RootSIFT descriptors.
TEST(OnnxMatcher, SiftLightGlueWithScaleAndOrientation)
{
	SKIP_WITHOUT(kSiftLightGlue);
	OnnxMatcher matcher(testFile(kSiftLightGlue), 0.1f, 0);
	ASSERT_TRUE(matcher.isModelLoaded());
	EXPECT_EQ(4, matcher.session()->inputs()[0].dim(2));
	EXPECT_EQ(128, matcher.session()->inputs()[2].dim(2));

	ParametersMap params;
	params[Parameters::kSIFTRootSIFT()] = "true";
	params[Parameters::kSIFTContrastThreshold()] = "0.01"; // the sample frame has little texture
	params[Parameters::kKpMaxFeatures()] = "1000";
	std::unique_ptr<Feature2D> sift(Feature2D::create(Feature2D::kFeatureSift, params));
	ASSERT_TRUE(sift.get() != NULL);
	expectTranslatedMatches(*sift, matcher);
}

// SuperGlue takes the keypoint scores too (scores0/scores1 inputs, filled
// from the keypoint responses) and was trained with the magicleap SuperPoint.
TEST(OnnxMatcher, SuperGlueWithKeypointScores)
{
	SKIP_WITHOUT(kMagicleapModel);
	SKIP_WITHOUT(kSuperGlue);
	OnnxMatcher matcher(testFile(kSuperGlue), 0.2f, 0);
	ASSERT_TRUE(matcher.isModelLoaded());
	EXPECT_EQ(6u, matcher.session()->inputs().size());
	EXPECT_GE(matcher.session()->inputIndex("scores0"), 0);
	std::unique_ptr<Feature2D> detector = createDetector(detectorParams(kMagicleapModel));
	expectTranslatedMatches(*detector, matcher);
}

//////////////////////////
// RegistrationVis with OnnxDetector / OnnxMatcher
//////////////////////////

TEST(OnnxMatcher, RegistrationVisParseParameters)
{
	SKIP_WITHOUT(kDiskLightGlue);
	ParametersMap params = registrationParams(kDiskLightGlue);
	RegistrationVis reg(params);
	EXPECT_EQ(10, reg.getNNType());
	EXPECT_EQ("ONNX MATCHER", reg.getNNTypeName());
	EXPECT_EQ("ONNX MATCHER", RegistrationVis::getNNTypeName(10));

	// Without a model path, RegistrationVis falls back to the default matcher.
	params.erase(Parameters::kOnnxMatcherModelPath());
	RegistrationVis fallback(params);
	EXPECT_EQ(Parameters::defaultVisCorNNType(), fallback.getNNType());
}

TEST_P(OnnxMatcherPair, RegistrationVisRgbd)
{
	ParametersMap params = registrationParams(matcherModel());
	params[Parameters::kVisFeatureType()] = std::to_string((int)Feature2D::kFeatureOnnxDetector);
	params[Parameters::kOnnxDetectorModelPath()] = testFile(detectorModel());
	expectRgbdRegistration(params);
}

TEST(OnnxMatcher, RegistrationVisRgbdSiftLightGlue)
{
	SKIP_WITHOUT(kSiftLightGlue);
	ParametersMap params = registrationParams(kSiftLightGlue);
	params[Parameters::kVisFeatureType()] = std::to_string((int)Feature2D::kFeatureSift);
	params[Parameters::kSIFTRootSIFT()] = "true";
	params[Parameters::kSIFTContrastThreshold()] = "0.01";
	expectRgbdRegistration(params);
}

//////////////////////////
// OnnxDescriptor
//////////////////////////

namespace {
ParametersMap descriptorParams(const std::string & model, int dim = 0)
{
	ParametersMap params;
	params[Parameters::kMemGlobalDescriptorStrategy()] = std::to_string((int)GlobalDescriptorExtractor::kOnnxDescriptor);
	params[Parameters::kOnnxDescriptorModelPath()] = model.empty()?"":testFile(model);
	params[Parameters::kOnnxDescriptorDim()] = std::to_string(dim);
	params[Parameters::kOnnxProvider()] = "0";
	return params;
}
SensorData colorSensorData(const std::string & id)
{
	cv::Mat rgb = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/rgb/" + id + ".jpg", cv::IMREAD_COLOR);
	EXPECT_FALSE(rgb.empty());
	return SensorData(rgb, CameraModel());
}
}

TEST(OnnxDescriptor, TypeAndMissingModel)
{
	EXPECT_EQ("OnnxDescriptor", GlobalDescriptorExtractor::typeName(GlobalDescriptorExtractor::kOnnxDescriptor));
	ParametersMap params = descriptorParams("");
	params[Parameters::kOnnxDescriptorModelPath()] = "/nonexistent/model.onnx";
	std::unique_ptr<GlobalDescriptorExtractor> extractor(GlobalDescriptorExtractor::create(params));
	ASSERT_TRUE(extractor.get() != NULL);
	EXPECT_EQ(GlobalDescriptorExtractor::kOnnxDescriptor, extractor->getType());
	EXPECT_FALSE(static_cast<OnnxDescriptor*>(extractor.get())->isModelLoaded());
	EXPECT_TRUE(extractor->extract(SensorData(cv::Mat::zeros(120, 160, CV_8UC3), CameraModel())).data().empty());
}

TEST(OnnxDescriptor, NetVLAD)
{
	SKIP_WITHOUT(kNetVLAD);
	std::unique_ptr<GlobalDescriptorExtractor> extractor(GlobalDescriptorExtractor::create(descriptorParams(kNetVLAD)));
	ASSERT_TRUE(extractor.get() != NULL);
	OnnxDescriptor * onnx = static_cast<OnnxDescriptor*>(extractor.get());
	ASSERT_TRUE(onnx->isModelLoaded());
	EXPECT_EQ("image", onnx->session()->inputs()[0].name);
	EXPECT_EQ(3, onnx->session()->inputs()[0].dim(1));

	GlobalDescriptor a = extractor->extract(colorSensorData("17"));
	ASSERT_FALSE(a.data().empty());
	EXPECT_EQ((int)GlobalDescriptorExtractor::kOnnxDescriptor, a.type());
	EXPECT_EQ(1, a.data().rows);
	EXPECT_EQ(CV_32F, a.data().type());
	EXPECT_GT(a.data().cols, 100);
	EXPECT_NEAR(1.0, cv::norm(a.data()), 1e-3); // L2-normalized

	// deterministic, and another view of the same scene is closer than a
	// gray image
	GlobalDescriptor a2 = extractor->extract(colorSensorData("17"));
	EXPECT_EQ(0.0, cv::norm(a.data(), a2.data()));
	GlobalDescriptor b = extractor->extract(colorSensorData("154"));
	ASSERT_EQ(a.data().cols, b.data().cols);
	SensorData gray(cv::Mat(480, 640, CV_8UC1, cv::Scalar(128)), CameraModel());
	GlobalDescriptor g = extractor->extract(gray); // grayscale input is accepted too
	ASSERT_EQ(a.data().cols, g.data().cols);
	EXPECT_GT(a.data().dot(b.data()), a.data().dot(g.data()));

	// Dim keeps the leading values, re-normalized
	std::unique_ptr<GlobalDescriptorExtractor> truncated(GlobalDescriptorExtractor::create(descriptorParams(kNetVLAD, 128)));
	GlobalDescriptor t = truncated->extract(colorSensorData("17"));
	ASSERT_EQ(128, t.data().cols);
	EXPECT_NEAR(1.0, cv::norm(t.data()), 1e-3);
	cv::Mat lead = a.data().colRange(0, 128).clone();
	lead /= cv::norm(lead);
	EXPECT_NEAR(0.0, cv::norm(lead, t.data()), 1e-4);
}

// Appearance-only loop closure detection on data/samples (84 frames of a
// corridor with the samples_GT.bmp loop closure ground truth), the
// likelihood coming from the NetVLAD descriptors (Kp/TfIdfLikelihoodUsed=false)
// instead of the bag-of-words.
//
// Asserted: every node gets a NetVLAD descriptor, and the most similar node
// older than the STM (what Signature::compareTo() ranks first) is a true loop
// closure for most queries that have one.
//
// Reported only: the loop closure hypotheses of the Bayes filter. The
// likelihood adjustment (Rtabmap::adjustLikelihood()) is designed for sparse
// likelihoods where most nodes get 0 (bag-of-words): with global descriptors
// every node has a similarity, so the virtual place (mean/stddev) dominates
// and the hypothesis values stay far below Rtabmap/LoopThr, with
// Rtabmap/VirtualPlaceLikelihoodRatio=1 they are spread over 0.1-0.3 for
// loops and non-loops alike. This is printed as a reference for tuning.
TEST(OnnxDescriptor, NetVLADAppearanceOnlyLoopClosure)
{
	SKIP_WITHOUT(kNetVLAD);
	const std::string samplesDir = std::string(RTABMAP_TEST_DATA_ROOT) + "/samples";
	const std::string gtPath = std::string(RTABMAP_TEST_DATA_ROOT) + "/samples_GT.bmp";
	if(!UDirectory::exists(samplesDir) || !UFile::exists(gtPath)) GTEST_SKIP() << "missing " << samplesDir << " or " << gtPath;
	cv::Mat gt = cv::imread(gtPath, cv::IMREAD_GRAYSCALE);
	ASSERT_EQ(84, gt.rows);
	ASSERT_EQ(84, gt.cols);
	int gtPositives = 0;
	for(int i=0; i<gt.rows; ++i)
	{
		gtPositives += cv::countNonZero(gt.row(i)) > 0 ? 1 : 0;
	}
	ASSERT_GT(gtPositives, 0);

	const int stmSize = 20;
	ParametersMap params = descriptorParams(kNetVLAD);
	params[Parameters::kRGBDEnabled()] = "false";
	params[Parameters::kKpTfIdfLikelihoodUsed()] = "false";
	params[Parameters::kMemRehearsalSimilarity()] = "1";   // no merges, as recommended with global descriptors
	params[Parameters::kMemSTMSize()] = uNumber2Str(stmSize);
	params[Parameters::kMemBadSignaturesIgnored()] = "false";
	const std::string workDb = test::tempPath(uFormat("test_onnx_netvlad_loops_%d.db", test::getPid()));
	UFile::erase(workDb);
	Rtabmap rtabmap;
	rtabmap.init(params, workDb);

	CameraImages camera(samplesDir);
	ASSERT_TRUE(camera.init());
	struct Hyp {int query; int id; double value; bool correct; bool accepted;};
	std::vector<Hyp> hyps;
	std::vector<cv::Mat> descriptors; // one per frame, in order
	int frames = 0;
	UTimer timer;
	for(SensorData data = camera.takeImage(); !data.imageRaw().empty(); data = camera.takeImage())
	{
		++frames;
		data.setId(frames);
		data.setStamp(frames);
		ASSERT_TRUE(rtabmap.process(data, Transform()));
		const Signature * last = rtabmap.getMemory()->getLastWorkingSignature(false);
		ASSERT_TRUE(last != NULL);
		ASSERT_EQ(frames, last->id());
		ASSERT_EQ(1u, last->sensorData().globalDescriptors().size()) << "frame " << frames;
		EXPECT_EQ((int)GlobalDescriptorExtractor::kOnnxDescriptor, last->sensorData().globalDescriptors()[0].type());
		descriptors.push_back(last->sensorData().globalDescriptors()[0].data().clone());
		Hyp h;
		h.query = frames - 1;
		h.id = rtabmap.getHighestHypothesisId();
		h.value = rtabmap.getHighestHypothesisValue();
		h.correct = h.id > 0 && h.id - 1 < gt.cols && gt.at<unsigned char>(h.query, h.id - 1) == 255;
		h.accepted = rtabmap.getLoopClosureId() > 0;
		hyps.push_back(h);
	}
	ASSERT_EQ(84, frames);
	const double secondsPerFrame = timer.elapsed() / frames;
	rtabmap.close();
	UFile::erase(workDb);

	// Retrieval by descriptor similarity: the best node older than the STM
	int queriesWithLoop = 0, top1Correct = 0, top1OnNonLoop = 0;
	for(int q=stmSize; q<frames; ++q)
	{
		int best = -1;
		double bestSim = -2.0;
		for(int j=0; j<q-stmSize+1; ++j)
		{
			const double sim = descriptors[q].dot(descriptors[j]);
			if(sim > bestSim) {bestSim = sim; best = j;}
		}
		ASSERT_GE(best, 0);
		if(cv::countNonZero(gt.row(q)) > 0)
		{
			++queriesWithLoop;
			top1Correct += gt.at<unsigned char>(q, best) == 255 ? 1 : 0;
		}
		else if(bestSim > 0.5)
		{
			++top1OnNonLoop;
		}
	}
	std::cerr << "[NetVLAD] top-1 retrieval: " << top1Correct << "/" << queriesWithLoop
			<< " queries with a loop closure have a true loop as most similar old node, "
			<< top1OnNonLoop << " queries without one have a similarity > 0.5; "
			<< secondsPerFrame << " s/frame\n";
	ASSERT_GT(queriesWithLoop, 0);
	EXPECT_GE(float(top1Correct) / float(queriesWithLoop), 0.8f);

	// Bayes filter hypotheses, for reference (see the comment above)
	std::sort(hyps.begin(), hyps.end(), [](const Hyp & a, const Hyp & b){return a.value > b.value;});
	int tp = 0, acceptedTp = 0, acceptedFp = 0;
	float recallAt100p = 0.0f;
	bool seenFp = false;
	for(const Hyp & h : hyps)
	{
		if(h.accepted) (h.correct?acceptedTp:acceptedFp)++;
		if(h.id <= 0 || h.value <= 0.0 || seenFp) continue;
		if(!h.correct)
		{
			seenFp = true;
			continue;
		}
		recallAt100p = float(++tp) / float(gtPositives);
	}
	std::cerr << "[NetVLAD] Bayes filter: recall at 100% precision = " << recallAt100p * 100.0f << "% (" << tp << "/" << gtPositives
			<< "), accepted loop closures tp=" << acceptedTp << " fp=" << acceptedFp << "\n";
}

#endif  // RTABMAP_ONNXRUNTIME
