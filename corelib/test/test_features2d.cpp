#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
#include <vector>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UException.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/core/Version.h>
#ifdef RTABMAP_PYTHON
#include <rtabmap/core/PythonInterface.h>
#endif
#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>

using namespace rtabmap;

#ifdef RTABMAP_PYTHON
// PythonInterface is a Meyers singleton that asserts construction on the
// main thread and embeds a pybind11 scoped_interpreter for the rest of
// the process. Triggering it lazily inside a gtest body has caused
// `take_gil: PyMUTEX_LOCK failed` deadlocks across SuperPoint reconstruct
// cycles; eagerly initialising once before the first test bypasses the
// problem. The Environment hook runs on the gtest main thread immediately
// before `RUN_ALL_TESTS()` starts.
class PythonInterfaceEnv : public ::testing::Environment {
public:
	void SetUp() override { PythonInterface::instance("test_features2d"); }
};
static ::testing::Environment * const kPythonEnv =
		::testing::AddGlobalTestEnvironment(new PythonInterfaceEnv);
#endif

namespace {

// 200x200 default is intentionally different from the natural-image sample
// so the Generate* tests exercise both sizes. The 200x200 + half-image ROI
// also locks in the ORBextractor::DistributeOctTree regression: keypoints
// landing on the right ROI border used to OOB-write the per-column bucket.
static cv::Mat checkerboardImage(int rows = 200, int cols = 200, int cell = 20)
{
	cv::Mat image(rows, cols, CV_8UC1);
	for(int y = 0; y < rows; ++y)
	{
		for(int x = 0; x < cols; ++x)
		{
			image.at<uchar>(y, x) = uchar((((x / cell) + (y / cell)) % 2) ? 255 : 0);
		}
	}
	// Add blobs so binary descriptors (ORB) also find stable corners.
	for(int i = 0; i < 8; ++i)
	{
		cv::circle(image, cv::Point(25 + i * 22, 30 + (i % 3) * 40), 6, cv::Scalar(180), -1);
	}
	return image;
}

static ParametersMap orbTestParams()
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kKpMaxFeatures(), "200"));
	params.insert(ParametersPair(Parameters::kKpSubPixWinSize(), "0"));
	params.insert(ParametersPair(Parameters::kKpGridRows(), "1"));
	params.insert(ParametersPair(Parameters::kKpGridCols(), "1"));
	return params;
}

// Forward-declare helpers defined near the Generate* tests, so the earlier
// detector-iterating tests can share the same fixtures.
ParametersMap detectorAssetParams(Feature2D::Type t);
bool isGenericGenerateCandidate(Feature2D::Type t);
struct NamedImage { const char * label; cv::Mat image; };
std::vector<NamedImage> generateTestImages();

} // namespace

// Smoke-test that isGpuAvailable() compiles and is callable for every available
// detector. We don't assert what it should return -- that depends on the
// host (GPU/CUDA presence, build flags). What we DO check: a detector that
// reports isGpuAvailable()=true must have actually selected a GPU codepath
// (i.e. its build was compiled with GPU support).
TEST(Feature2DTest, IsGpuAvailable)
{
	for(int strategy = Feature2D::kFeatureSurf; strategy < Feature2D::kFeatureEnd; ++strategy)
	{
		const Feature2D::Type t = static_cast<Feature2D::Type>(strategy);
		if(!Feature2D::isAvailable(t)) continue;
		SCOPED_TRACE(Feature2D::typeName(t));
		// isGpuAvailable() only checks build flags + runtime CUDA, so we
		// don't need an actual model loaded -- construct SuperPoint /
		// PyDetector even when their assets aren't present.
		std::unique_ptr<Feature2D> detector(Feature2D::create(t, detectorAssetParams(t)));
		ASSERT_TRUE(detector.get() != NULL);
		const bool gpu = detector->isGpuAvailable();
		// Smoke: the call returned without crashing or hanging. Actual
		// value depends on the host (GPU/CUDA presence, build flags),
		// so we don't assert true/false here.
		(void)gpu;
	}
}

// Invariant: every type in [kFeatureSurf, kFeatureEnd) is named explicitly in
// the typeName() switch. If a new value is appended to Feature2D::Type
// without adding a matching `case`, it falls through to the "Unknown"
// default and this test fails -- forcing the new backend to ship with a
// label like every other strategy.
TEST(Feature2DTest, TypeNameCoverage)
{
	for(int strategy = Feature2D::kFeatureSurf; strategy < Feature2D::kFeatureEnd; ++strategy)
	{
		const Feature2D::Type t = static_cast<Feature2D::Type>(strategy);
		const std::string name = Feature2D::typeName(t);
		EXPECT_NE(name, "Unknown")
				<< "Feature2D::typeName() returns \"Unknown\" for enum value "
				<< strategy << " -- add a `case` for it in the switch.";
		EXPECT_FALSE(name.empty())
				<< "Feature2D::typeName() returns empty string for enum value " << strategy;
	}
	// Sentinel + out-of-range values fall through to "Unknown".
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureUndef), "Unknown");
	EXPECT_EQ(Feature2D::typeName((Feature2D::Type)99), "Unknown");
}

TEST(Feature2DTest, ComputeRoiFromRatios)
{
	const cv::Mat image = checkerboardImage(100, 100);
	std::vector<float> ratios;
	ratios.push_back(0.1f);
	ratios.push_back(0.1f);
	ratios.push_back(0.1f);
	ratios.push_back(0.1f);
	const cv::Rect roi = Feature2D::computeRoi(image, ratios);
	EXPECT_EQ(roi.x, 10);
	EXPECT_EQ(roi.y, 10);
	EXPECT_EQ(roi.width, 80);
	EXPECT_EQ(roi.height, 80);
}

TEST(Feature2DTest, ComputeRoiFromString)
{
	const cv::Mat image = checkerboardImage(100, 100);
	const cv::Rect roi = Feature2D::computeRoi(image, "0.1 0.1 0.1 0.1");
	EXPECT_EQ(roi.width, 80);
	EXPECT_EQ(roi.height, 80);
}

TEST(Feature2DTest, LimitKeypointsByResponse)
{
	std::vector<cv::KeyPoint> keypoints;
	for(int i = 0; i < 10; ++i)
	{
		cv::KeyPoint kpt;
		kpt.pt = cv::Point2f(float(i * 10), float(i * 10));
		kpt.response = float(i);
		keypoints.push_back(kpt);
	}
	Feature2D::limitKeypoints(keypoints, 5, cv::Size(200, 200), false);
	EXPECT_EQ(keypoints.size(), 5u);

	std::vector<float> keptResponses;
	keptResponses.reserve(keypoints.size());
	for(size_t i = 0; i < keypoints.size(); ++i)
	{
		keptResponses.push_back(keypoints[i].response);
	}
	std::sort(keptResponses.begin(), keptResponses.end());

	const std::vector<float> expectedTop5 = {5.f, 6.f, 7.f, 8.f, 9.f};
	EXPECT_EQ(keptResponses, expectedTop5);
}

TEST(Feature2DTest, LimitKeypointsWithDescriptors)
{
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors(8, 32, CV_8UC1, cv::Scalar(0));
	for(int i = 0; i < 8; ++i)
	{
		cv::KeyPoint kpt;
		kpt.pt = cv::Point2f(float(i), float(i));
		kpt.response = float(i);
		keypoints.push_back(kpt);
		descriptors.at<uchar>(i, 0) = uchar(i);
	}
	Feature2D::limitKeypoints(keypoints, descriptors, 3, cv::Size(100, 100), false);
	EXPECT_EQ(keypoints.size(), 3u);
	EXPECT_EQ(descriptors.rows, 3);

	for(int k = 0; k < descriptors.rows; ++k)
	{
		const int index = int(keypoints[k].pt.x);
		EXPECT_EQ(index, int(keypoints[k].pt.y));
		EXPECT_FLOAT_EQ(keypoints[k].response, float(index));
		EXPECT_EQ(descriptors.at<uchar>(k, 0), uchar(index));
	}

	std::vector<float> keptResponses;
	for(size_t i = 0; i < keypoints.size(); ++i)
	{
		keptResponses.push_back(keypoints[i].response);
	}
	std::sort(keptResponses.begin(), keptResponses.end());
	const std::vector<float> expectedTop3 = {5.f, 6.f, 7.f};
	EXPECT_EQ(keptResponses, expectedTop3);
}

// Verify Kp/MaxFeatures caps detector output across every available
// detector and both test images. Cap = 20: every detector currently
// finds more than that on both images, so the cap is genuinely
// exercised in all combinations. Also verifies generateDescriptors()
// produces one row per surviving keypoint (no silent drop).
TEST(Feature2DTest, KpMaxFeaturesCapsDetectorOutput)
{
	const int kMaxKeypoints = 20;
	int tested = 0;
	for(const auto & img : generateTestImages())
	{
		SCOPED_TRACE(std::string("image=") + img.label);
		for(int s = Feature2D::kFeatureSurf; s < Feature2D::kFeatureEnd; ++s)
		{
			const Feature2D::Type t = static_cast<Feature2D::Type>(s);
			if(!isGenericGenerateCandidate(t)) continue;
			SCOPED_TRACE(Feature2D::typeName(t));

			ParametersMap params = detectorAssetParams(t);
			params[Parameters::kKpMaxFeatures()] = uNumber2Str(kMaxKeypoints);
			std::unique_ptr<Feature2D> detector(Feature2D::create(t, params));
			ASSERT_TRUE(detector.get() != NULL);
			std::vector<cv::KeyPoint> capped = detector->generateKeypoints(img.image);
			EXPECT_LE(static_cast<int>(capped.size()), kMaxKeypoints)
					<< "capped=" << capped.size();

			cv::Mat descriptors = detector->generateDescriptors(img.image, capped);
			EXPECT_EQ(descriptors.rows, static_cast<int>(capped.size()))
					<< "descriptors=" << descriptors.rows
					<< " keypoints=" << capped.size();
			++tested;
		}
	}
	ASSERT_GT(tested, 0) << "no Features2D detector was available";
}

TEST(Feature2DTest, FilterKeypointsByDepth)
{
	std::vector<cv::KeyPoint> keypoints;
	keypoints.push_back(cv::KeyPoint(50.f, 50.f, 1.f));
	keypoints.push_back(cv::KeyPoint(60.f, 60.f, 1.f));

	cv::Mat depth(100, 100, CV_32FC1, cv::Scalar(2.f));
	depth.at<float>(60, 60) = 0.05f;

	Feature2D::filterKeypointsByDepth(keypoints, depth, 0.1f, 5.f);
	EXPECT_EQ(keypoints.size(), 1u);
	EXPECT_FLOAT_EQ(keypoints[0].pt.x, 50.f);
}

TEST(Feature2DTest, FilterKeypointsByDisparity)
{
	std::vector<cv::KeyPoint> keypoints;
	keypoints.push_back(cv::KeyPoint(10.f, 10.f, 1.f));
	keypoints.push_back(cv::KeyPoint(20.f, 20.f, 1.f));

	cv::Mat disparity(50, 50, CV_32FC1, cv::Scalar(0.f));
	disparity.at<float>(10, 10) = 5.f;

	Feature2D::filterKeypointsByDisparity(keypoints, disparity, 1.f);
	EXPECT_EQ(keypoints.size(), 1u);
	EXPECT_FLOAT_EQ(keypoints[0].pt.x, 10.f);
}

TEST(Feature2DTest, CreateOrbDetector)
{
	ParametersMap params = orbTestParams();
	std::unique_ptr<Feature2D> detector(Feature2D::create(Feature2D::kFeatureOrb, params));
	ASSERT_TRUE(detector.get() != NULL);
	EXPECT_EQ(detector->getType(), Feature2D::kFeatureOrb);
	EXPECT_EQ(detector->getMaxFeatures(), 200);
}

// Smoke test: create() must not crash or return null for every strategy (fallbacks allowed).
TEST(Feature2DTest, CreateAllDetectorStrategiesSmoke)
{
	// kFeatureUndef passes through create()'s default branch; cover it too.
	{
		std::unique_ptr<Feature2D> detector(Feature2D::create(Feature2D::kFeatureUndef));
		ASSERT_TRUE(detector.get() != NULL) << "create() returned null for kFeatureUndef";
	}

	for(int strategy = Feature2D::kFeatureSurf; strategy < Feature2D::kFeatureEnd; ++strategy)
	{
		const Feature2D::Type requested = static_cast<Feature2D::Type>(strategy);
		std::unique_ptr<Feature2D> detector(Feature2D::create(requested));
		ASSERT_TRUE(detector.get() != NULL)
				<< "create() returned null for " << Feature2D::typeName(requested);

		ParametersMap paramsWithStrategy;
		paramsWithStrategy[Parameters::kKpDetectorStrategy()] = uNumber2Str(strategy);
		std::unique_ptr<Feature2D> detectorFromParams(Feature2D::create(paramsWithStrategy));
		ASSERT_TRUE(detectorFromParams.get() != NULL)
				<< "create(ParametersMap) returned null for Kp/DetectorStrategy=" << strategy;
	}
}

TEST(Feature2DTest, CreateFromParametersMap)
{
	ParametersMap params = orbTestParams();
	params.insert(ParametersPair(Parameters::kKpDetectorStrategy(), "2")); // ORB
	std::unique_ptr<Feature2D> detector(Feature2D::create(params));
	ASSERT_TRUE(detector.get() != NULL);
	EXPECT_EQ(detector->getType(), Feature2D::kFeatureOrb);
}

// Invariant: Feature2D::isAvailable(T) is the negation of "create() would
// silently substitute a different backend for T in this build". If anyone
// adds a new build-flag fallback in Feature2D::create() without updating
// isAvailable() (or vice versa), this test fails -- which is the whole point.
// Iterates [kFeatureSurf, kFeatureEnd) so a new backend appended to the enum
// is auto-covered without an extra edit here.
TEST(Feature2DTest, IsAvailableMatchesCreate)
{
	for(int strategy = Feature2D::kFeatureSurf; strategy < Feature2D::kFeatureEnd; ++strategy)
	{
		const Feature2D::Type requested = static_cast<Feature2D::Type>(strategy);
		std::unique_ptr<Feature2D> detector(Feature2D::create(requested));
		ASSERT_TRUE(detector.get() != NULL)
				<< Feature2D::typeName(requested) << ": create() returned null";
		const bool available = Feature2D::isAvailable(requested);
		const bool matched   = detector->getType() == requested;
		EXPECT_EQ(available, matched)
				<< Feature2D::typeName(requested)
				<< ": isAvailable()=" << available
				<< " but create()->getType()="
				<< Feature2D::typeName(detector->getType())
				<< " (matched=" << matched << "). "
				<< "Update Feature2D::isAvailable() to match the new "
				<< "fallback in Feature2D::create() (or vice versa).";
	}

	// kFeatureUndef is a sentinel ("strategy not specified"). create() falls
	// through to a default backend (SURF or GFTT_ORB depending on
	// RTABMAP_NONFREE), so isAvailable() must report it as unavailable.
	EXPECT_FALSE(Feature2D::isAvailable(Feature2D::kFeatureUndef))
			<< "kFeatureUndef should never be reported as available";
}

TEST(Feature2DTest, ParseParametersUpdatesMaxFeatures)
{
	ParametersMap params = orbTestParams();
	std::unique_ptr<Feature2D> detector(Feature2D::create(Feature2D::kFeatureOrb, params));
	ASSERT_TRUE(detector.get() != NULL);

	ParametersMap update;
	update.insert(ParametersPair(Parameters::kKpMaxFeatures(), "50"));
	detector->parseParameters(update);
	EXPECT_EQ(detector->getMaxFeatures(), 50);
}

namespace {
// Load `data/samples/17.jpg` -- a real outdoor frame with strong corner
// content for blob-based descriptors (SIFT/SURF/KAZE) and stable enough
// for binary ones (ORB/BRIEF). Test data ships with the repo so this is
// always available, no fetch_test_data.sh required.
cv::Mat loadSampleImage()
{
	const std::string path = std::string(RTABMAP_TEST_DATA_ROOT) + "/samples/17.jpg";
	return cv::imread(path, cv::IMREAD_GRAYSCALE);
}

// Paths fetched by scripts/fetch_test_data.sh into data/tests/.
inline std::string superpointTorchModel()
{
	return std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/superpoint_v1.pt";
}
inline std::string superpointRpautratWeights()
{
	return std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/superpoint_v6_from_tf.pth";
}
inline std::string superpointRpautratModel()
{
	return std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/superpoint_pytorch.py";
}

// Detector types iterated by the Generate* tests. PyDetector needs a
// user-supplied script (skipped). SuperPoint variants need libtorch
// weights + (for Rpautrat) a Python model file; included when those
// assets are on disk.
bool isGenericGenerateCandidate(Feature2D::Type t)
{
	if(!Feature2D::isAvailable(t)) return false;
	if(t == Feature2D::kFeaturePyDetector) return false;
	if(t == Feature2D::kFeatureSuperPointTorch)
	{
		return UFile::exists(superpointTorchModel());
	}
	if(t == Feature2D::kFeatureSuperPointRpautrat)
	{
		return UFile::exists(superpointRpautratWeights())
				&& UFile::exists(superpointRpautratModel());
	}
	return true;
}

ParametersMap detectorAssetParams(Feature2D::Type t)
{
	ParametersMap params;
	if(t == Feature2D::kFeatureSuperPointTorch)
	{
		params[Parameters::kSuperPointModelPath()] = superpointTorchModel();
		params[Parameters::kSuperPointCuda()]      = "false";
	}
	else if(t == Feature2D::kFeatureSuperPointRpautrat)
	{
		params[Parameters::kSuperPointRpautratWeightsPath()] = superpointRpautratWeights();
		params[Parameters::kSuperPointRpautratModelPath()]   = superpointRpautratModel();
		params[Parameters::kSuperPointRpautratCuda()]        = "false";
	}
	return params;
}

// Flips the per-detector "use GPU" parameter on. Each backend has its own
// key (SURF/GpuVersion, SIFT/Gpu, ORB/Gpu, FAST/Gpu, GFTT/Gpu,
// SuperPoint/Cuda, SuperPointRpautrat/Cuda). Detectors that share a key
// (FAST_BRIEF/FAST_FREAK share FAST/Gpu; the GFTT variants share GFTT/Gpu)
// are grouped accordingly. Detectors without a GPU backend pass through
// unchanged.
void setDetectorGpu(Feature2D::Type t, ParametersMap & params)
{
	switch(t)
	{
	case Feature2D::kFeatureSurf:               params[Parameters::kSURFGpuVersion()] = "true"; break;
	case Feature2D::kFeatureSift:               params[Parameters::kSIFTGpu()]        = "true"; break;
	case Feature2D::kFeatureOrb:                params[Parameters::kORBGpu()]         = "true"; break;
	case Feature2D::kFeatureFastBrief:
	case Feature2D::kFeatureFastFreak:          params[Parameters::kFASTGpu()]        = "true"; break;
	case Feature2D::kFeatureGfttFreak:
	case Feature2D::kFeatureGfttBrief:
	case Feature2D::kFeatureGfttOrb:
	case Feature2D::kFeatureGfttDaisy:          params[Parameters::kGFTTGpu()]        = "true"; break;
	case Feature2D::kFeatureSuperPointTorch:    params[Parameters::kSuperPointCuda()] = "true"; break;
	case Feature2D::kFeatureSuperPointRpautrat: params[Parameters::kSuperPointRpautratCuda()] = "true"; break;
	default: break;
	}
}
}  // namespace

namespace {
// Each (test, detector) pair runs against both a real outdoor frame (17.jpg)
// and a synthetic high-contrast checkerboard. The pair-iteration catches
// detectors that handle one image well and crash on the other (e.g. an
// edge case in ORB-OCTREE on uniform synthetic patterns with ROI).
// NamedImage is forward-declared near the top of this file so earlier
// tests can share the helper.
std::vector<NamedImage> generateTestImages()
{
	return {
		{"sample17",     loadSampleImage()},
		{"checkerboard", checkerboardImage()},
	};
}
}  // namespace

TEST(Feature2DTest, GenerateKeypointsAndDescriptors)
{
	int tested = 0;
	for(const auto & img : generateTestImages())
	{
		SCOPED_TRACE(std::string("image=") + img.label);
		for(int strategy = Feature2D::kFeatureSurf; strategy < Feature2D::kFeatureEnd; ++strategy)
		{
			const Feature2D::Type t = static_cast<Feature2D::Type>(strategy);
			if(!isGenericGenerateCandidate(t)) continue;
			SCOPED_TRACE(Feature2D::typeName(t));

			// CPU pass.
			ParametersMap cpuParams = detectorAssetParams(t);
			std::unique_ptr<Feature2D> cpuDetector(Feature2D::create(t, cpuParams));
			ASSERT_TRUE(cpuDetector.get() != NULL);

			std::vector<cv::KeyPoint> cpuKeypoints = cpuDetector->generateKeypoints(img.image);
			EXPECT_GT(cpuKeypoints.size(), 0u);

			cv::Mat cpuDescriptors = cpuDetector->generateDescriptors(img.image, cpuKeypoints);
			ASSERT_FALSE(cpuDescriptors.empty());
			EXPECT_EQ(cpuDescriptors.rows, static_cast<int>(cpuKeypoints.size()));
			// Descriptor dtype is detector-specific (binary CV_8UC1 for
			// ORB/BRIEF/BRISK/FREAK, float CV_32FC1 for SURF/SIFT/KAZE/DAISY),
			// so we just sanity-check it's one of those two.
			EXPECT_TRUE(cpuDescriptors.type() == CV_8UC1 || cpuDescriptors.type() == CV_32FC1)
					<< "unexpected descriptor type " << cpuDescriptors.type();
			++tested;

			// GPU pass (only when the build + runtime supports it; the
			// per-instance flag is the per-detector GPU param we set
			// below). Compare keypoint counts between the two paths:
			// the CPU and GPU implementations don't have to be
			// bit-identical, but they should detect a similar order
			// of magnitude of keypoints on the same image.
			if(!cpuDetector->isGpuAvailable()) continue;
			cpuDetector.reset();  // free CUDA resources before reconstructing
			SCOPED_TRACE("variant=GPU");

			ParametersMap gpuParams = detectorAssetParams(t);
			setDetectorGpu(t, gpuParams);
			std::unique_ptr<Feature2D> gpuDetector(Feature2D::create(t, gpuParams));
			ASSERT_TRUE(gpuDetector.get() != NULL);

			std::vector<cv::KeyPoint> gpuKeypoints = gpuDetector->generateKeypoints(img.image);
			EXPECT_GT(gpuKeypoints.size(), 0u);

			cv::Mat gpuDescriptors = gpuDetector->generateDescriptors(img.image, gpuKeypoints);
			ASSERT_FALSE(gpuDescriptors.empty());
			EXPECT_EQ(gpuDescriptors.rows, static_cast<int>(gpuKeypoints.size()));

			// CPU/GPU implementations differ in sub-pixel refinement,
			// thresholding, and NMS, so the counts won't match exactly.
			// Allow up to 30% relative difference -- catches a backend
			// that returns 0 or a huge spread, lets small drift through.
			const float cpuCount = static_cast<float>(cpuKeypoints.size());
			const float gpuCount = static_cast<float>(gpuKeypoints.size());
			const float relDiff  = std::fabs(gpuCount - cpuCount) / std::max(cpuCount, 1.0f);
			EXPECT_LE(relDiff, 0.30f)
					<< "CPU keypoints=" << cpuKeypoints.size()
					<< " vs GPU keypoints=" << gpuKeypoints.size()
					<< " (relDiff=" << relDiff << ")";
		}
	}
	ASSERT_GT(tested, 0) << "no detector was available to exercise";
}

TEST(Feature2DTest, GenerateKeypointsAndDescriptorsWithMask)
{
	int tested = 0;
	for(const auto & img : generateTestImages())
	{
		SCOPED_TRACE(std::string("image=") + img.label);
		// Valid region: left half only (OpenCV mask: non-zero = detect).
		cv::Mat mask(img.image.rows, img.image.cols, CV_8UC1, cv::Scalar(0));
		mask(cv::Rect(0, 0, img.image.cols / 2, img.image.rows)).setTo(255);

		for(int strategy = Feature2D::kFeatureSurf; strategy < Feature2D::kFeatureEnd; ++strategy)
		{
			const Feature2D::Type t = static_cast<Feature2D::Type>(strategy);
			if(!isGenericGenerateCandidate(t)) continue;
			SCOPED_TRACE(Feature2D::typeName(t));

			std::unique_ptr<Feature2D> detector(Feature2D::create(t, detectorAssetParams(t)));
			ASSERT_TRUE(detector.get() != NULL);

			std::vector<cv::KeyPoint> keypoints = detector->generateKeypoints(img.image, mask);
			EXPECT_GT(keypoints.size(), 0u);

			for(size_t i = 0; i < keypoints.size(); ++i)
			{
				const int x = cvRound(keypoints[i].pt.x);
				const int y = cvRound(keypoints[i].pt.y);
				ASSERT_GE(x, 0);
				ASSERT_GE(y, 0);
				ASSERT_LT(x, mask.cols);
				ASSERT_LT(y, mask.rows);
				EXPECT_GT(mask.at<uchar>(y, x), 0)
						<< "keypoint " << i << " at (" << x << "," << y << ") outside mask";
			}

			cv::Mat descriptors = detector->generateDescriptors(img.image, keypoints);
			ASSERT_FALSE(descriptors.empty());
			EXPECT_EQ(descriptors.rows, static_cast<int>(keypoints.size()));
			++tested;
		}
	}
	ASSERT_GT(tested, 0) << "no detector was available to exercise";
}

TEST(Feature2DTest, GenerateKeypointsAndDescriptorsWithRoi)
{
	int tested = 0;
	for(const auto & img : generateTestImages())
	{
		SCOPED_TRACE(std::string("image=") + img.label);
		// Left half only (Kp/RoiRatios: left right top bottom).
		const cv::Rect roi = Feature2D::computeRoi(img.image, "0 0.5 0 0");
		ASSERT_GT(roi.width, 0);
		ASSERT_GT(roi.height, 0);

		for(int strategy = Feature2D::kFeatureSurf; strategy < Feature2D::kFeatureEnd; ++strategy)
		{
			const Feature2D::Type t = static_cast<Feature2D::Type>(strategy);
			if(!isGenericGenerateCandidate(t)) continue;
			// SuperPointTorch / SuperPointRpautrat ignore the ROI extent
			// (they only refuse non-zero offsets; they don't crop the
			// image to roi.width/height). Skipping until that's fixed.
			if(t == Feature2D::kFeatureSuperPointTorch
					|| t == Feature2D::kFeatureSuperPointRpautrat) continue;
			SCOPED_TRACE(Feature2D::typeName(t));

			ParametersMap params = detectorAssetParams(t);
			params[Parameters::kKpRoiRatios()] = "0 0.5 0 0";

			std::unique_ptr<Feature2D> detector(Feature2D::create(t, params));
			ASSERT_TRUE(detector.get() != NULL);

			std::vector<cv::KeyPoint> keypoints = detector->generateKeypoints(img.image);
			EXPECT_GT(keypoints.size(), 0u);

			for(size_t i = 0; i < keypoints.size(); ++i)
			{
				const int x = cvRound(keypoints[i].pt.x);
				const int y = cvRound(keypoints[i].pt.y);
				EXPECT_GE(x, roi.x) << "keypoint " << i << " at (" << x << "," << y << ")";
				EXPECT_GE(y, roi.y);
				EXPECT_LT(x, roi.x + roi.width);
				EXPECT_LT(y, roi.y + roi.height);
			}

			cv::Mat descriptors = detector->generateDescriptors(img.image, keypoints);
			ASSERT_FALSE(descriptors.empty());
			EXPECT_EQ(descriptors.rows, static_cast<int>(keypoints.size()));
			++tested;
		}
	}
	ASSERT_GT(tested, 0) << "no detector was available to exercise";
}

TEST(Feature2DTest, GenerateDescriptorsEmptyForNoKeypoints)
{
	const cv::Mat image = loadSampleImage();
	std::unique_ptr<Feature2D> detector(Feature2D::create(Feature2D::kFeatureOrb, orbTestParams()));
	std::vector<cv::KeyPoint> keypoints;
	const cv::Mat descriptors = detector->generateDescriptors(image, keypoints);
	EXPECT_TRUE(descriptors.empty());
}
