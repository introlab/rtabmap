#include <gtest/gtest.h>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/core/stereo/StereoBM.h>
#include <rtabmap/core/util2d.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <set>
#include <limits>
#include <algorithm>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>

using namespace rtabmap;

namespace {

// Shared by RGB-D and stereo-as-RGB-D (stereo depth from StereoBM).
static constexpr int kVisMaxFeatures = 3000;
static constexpr int kGfttMinDistance = 5;
static constexpr double kGfttQualityLevel = 0.01;
static const char kRoiRatios[] = "0 0 0 0.3";

static ParametersMap registrationVisTestParams(int estimationType = 1, int corType = 0)
{
	ParametersMap params;
	params[Parameters::kVisFeatureType()] = "8"; // GFTT/ORB
	params[Parameters::kVisMaxFeatures()] = std::to_string(kVisMaxFeatures);
	params[Parameters::kGFTTMinDistance()] = std::to_string(kGfttMinDistance);
	params[Parameters::kGFTTQualityLevel()] = std::to_string(kGfttQualityLevel);
	params[Parameters::kVisEstimationType()] = std::to_string(estimationType);
	params[Parameters::kVisCorType()] = std::to_string(corType); // 0=feature matching, 1=optical flow
	params[Parameters::kVisBundleAdjustment()] = "0";
	params[Parameters::kVisRoiRatios()] = kRoiRatios;
	if(corType == 1)
	{
		// Lucas-Kanade tracker defaults (win=16, levels=3) leave enough
		// sub-pixel drift to push most correspondences past the 2 px
		// reprojection bound on some OpenCV builds (SIMD/IPP differences in
		// cv::calcOpticalFlowPyrLK). A larger window + more pyramid levels
		// converge tighter and keep the OF test robust across platforms
		// (~90% inlier ratio instead of ~5% on Ubuntu's DFSG OpenCV).
		params[Parameters::kVisCorFlowWinSize()] = "21";
		params[Parameters::kVisCorFlowMaxLevel()] = "5";
	}
	return params;
}

#if defined(RTABMAP_G2O)
static ParametersMap registrationVisTestParamsWithG2oBundleAdjustment(int estimationType, int corType = 0)
{
	ParametersMap params = registrationVisTestParams(estimationType, corType);
	params[Parameters::kVisBundleAdjustment()] = "1";
	params[Parameters::kVisPnPRefineIterations()] = "0";
	return params;
}
#endif

static void expectNearIdentity(const Transform & t)
{
	ASSERT_FALSE(t.isNull());
	if(t.isIdentity())
	{
		return;
	}
	EXPECT_NEAR(t.x(), 0.f, 0.05f);
	EXPECT_NEAR(t.y(), 0.f, 0.05f);
	EXPECT_NEAR(t.z(), 0.f, 0.05f);
	EXPECT_NEAR(t.theta(), 0.f, 0.1f);
}

// Epipolar (Vis/EstimationType=2) is degenerate for identical views (no parallax).
static void expectSameFrameResult(int estimationType, const Transform & result, const RegistrationInfo & info, int minInliers)
{
	if(estimationType == 2 && result.isNull())
	{
		EXPECT_EQ(info.inliers, 0);
		return;
	}
	expectNearIdentity(result);
	EXPECT_GE(info.inliers, minInliers);
}

static SensorData loadRgbdSensorData(const std::string & id)
{
	const std::string root = std::string(RTABMAP_TEST_DATA_ROOT);
	const cv::Mat depth = cv::imread(root + "/rgbd/depth/" + id + ".png", cv::IMREAD_UNCHANGED);
	const cv::Mat rgb = cv::imread(root + "/rgbd/rgb/" + id + ".jpg", cv::IMREAD_COLOR);
	if(depth.empty() || rgb.empty())
	{
		return SensorData();
	}
	CameraModel model;
	if(!model.load(root + "/rgbd/calib/" + id + ".yaml"))
	{
		return SensorData();
	}
	return SensorData(rgb, depth, model);
}

// Stereo rectified pair -> RGB-D via StereoBM (same path as SensorCaptureThread stereo-to-depth).
static SensorData loadStereoSensorData(const std::string & id)
{
	const std::string root = std::string(RTABMAP_TEST_DATA_ROOT);
	const cv::Mat left = cv::imread(root + "/stereo_rect/left/" + id + ".jpg", cv::IMREAD_UNCHANGED);
	const cv::Mat right = cv::imread(root + "/stereo_rect/right/" + id + ".jpg", cv::IMREAD_GRAYSCALE);
	if(left.empty() || right.empty())
	{
		return SensorData();
	}
	StereoCameraModel stereoModel;
	if(!stereoModel.load(root + "/stereo_rect", "stereo") || !stereoModel.isValidForProjection())
	{
		return SensorData();
	}

	return SensorData(left, right, stereoModel);
}

static std::string estimationTypeName(int type)
{
	switch(type)
	{
	case 0: return "3DTo3D";
	case 1: return "PnP";
	case 2: return "Epipolar";
	default: return "Unknown";
	}
}

static Transform computeRegistration(
		const SensorData & fromData,
		const SensorData & toData,
		const ParametersMap & params,
		RegistrationInfo * infoOut = nullptr,
		const Transform & guess = Transform())
{
	RegistrationVis reg(params);
	Signature from(fromData);
	Signature to(toData);
	RegistrationInfo info;
	return reg.computeTransformation(from, to, guess, infoOut ? infoOut : &info);
}

// Golden transforms (GFTT/ORB, MinDistance=3, QualityLevel=0.01, MaxFeatures=3000, RoiRatios=0 0 0 0.3).
// Captured with Vis/CorType=0 (feature matching); also used for optical flow (CorType=1) within tolerance.
// Shared by FM/OF, Vis/BundleAdjustment=0 and g2o BA=1.
static constexpr float kGoldenTransTolM = 0.20f;
static constexpr float kGoldenTransTolEpipolarM = 0.70f; // CI can give up to 0.61 depending on platform and opencv version used
static constexpr float kGoldenAngleTolRad = 0.28f;

static float goldenTransTolForEstimationType(int estimationType)
{
	return estimationType == 2 ? kGoldenTransTolEpipolarM : kGoldenTransTolM;
}

// rgbd/17 -> rgbd/154
static const Transform kRgbd17To154Expected[3] = {
		Transform(0.99870515f, 0.04859976f, 0.01503834f, 0.35593706f,
				-0.04851507f, 0.99880475f, -0.00594683f, -0.21210882f,
				-0.01530938f, 0.00520954f, 0.99986923f, 0.04961354f),
		Transform(0.99626255f, 0.08612057f, 0.00666280f, 0.40622258f,
				-0.08607896f, 0.99626857f, -0.00629793f, -0.15886482f,
				-0.00718031f, 0.00570087f, 0.99995804f, 0.02304785f),
		Transform(0.99893183f, 0.03752048f, -0.02697127f, 0.51900554f,
				-0.03745415f, 0.99929398f, 0.00296024f, -0.29134610f,
				0.02706329f, -0.00194689f, 0.99963182f, -0.10698023f)};

// stereo_rect/50 -> stereo_rect/60 (native stereo rectified)
static const Transform kStereo50To60Expected[3] = {
		Transform(0.99237216f, -0.11357912f, 0.04792929f, 0.13281979f,
				0.11265049f, 0.99339861f, 0.02165955f, 0.14165790f,
				-0.05007296f, -0.01609508f, 0.99861586f, 0.02466334f),
		Transform(0.99054426f, -0.12871768f, 0.04747626f, 0.13877144f,
				0.12772329f, 0.99153322f, 0.02342802f, 0.02014065f,
				-0.05008988f, -0.01714267f, 0.99859768f, 0.02536142f),
		Transform(0.99255836f, -0.11204252f, 0.04769079f, 0.15163948f,
				0.11127493f, 0.99361807f, 0.01846521f, 0.16984999f,
				-0.04945532f, -0.01302101f, 0.99869144f, 0.02214202f)};

static void expectTransformNearExpected(
		const Transform & result,
		const Transform & expected,
		float transTolM,
		float angleTolRad,
		const std::string & label)
{
	ASSERT_FALSE(result.isNull()) << label;
	EXPECT_LT(result.getDistance(expected), transTolM) << label;
	EXPECT_LT(result.getAngle(expected), angleTolRad) << label;
}

static Transform computeRegistrationRobust(
		const SensorData & fromData,
		const SensorData & toData,
		const ParametersMap & params,
		RegistrationInfo * infoOut = nullptr,
		const Transform & guess = Transform())
{
	const int estimationType = std::atoi(params.at(Parameters::kVisEstimationType()).c_str());
	// Epipolar (type=2) is the most RANSAC-sensitive so it gets the biggest
	// attempt budget; PnP (type=1) also uses RANSAC internally via
	// cv::solvePnPRansac and has been observed to fail one-shot on CI's
	// Ubuntu OpenCV build, so retry it too. F2F (type=0) is largely
	// deterministic but cheap, so we still seed and retry a few times for
	// uniformity.
	const int maxAttempts = estimationType == 2 ? 50 : 10;
	Transform result;
	RegistrationInfo info;
	for(int attempt = 0; attempt < maxAttempts; ++attempt)
	{
		cv::theRNG() = cv::RNG(static_cast<uint64_t>(0x9e3779b97f4a7c15ULL) ^
				static_cast<uint64_t>(attempt + 1));
		result = computeRegistration(fromData, toData, params, &info, guess);
		if(!result.isNull())
		{
			break;
		}
	}
	if(infoOut)
	{
		*infoOut = info;
	}
	return result;
}

static void expectAllEstimationTypesMatchExpected(
		const SensorData & fromData,
		const SensorData & toData,
		const Transform expectedByType[3],
		float transTolM,
		float angleTolRad,
		const std::function<ParametersMap(int)> & paramsForEstimationType =
				[](int estimationType) { return registrationVisTestParams(estimationType); })
{
	for(int estimationType = 0; estimationType <= 2; ++estimationType)
	{
		RegistrationInfo info;
		const Transform result = computeRegistrationRobust(
				fromData, toData, paramsForEstimationType(estimationType), &info);
		const std::string label = estimationTypeName(estimationType);
		const float transTol = goldenTransTolForEstimationType(estimationType);
		expectTransformNearExpected(result, expectedByType[estimationType], transTol, angleTolRad, label);
		EXPECT_GE(info.inliers, 6) << label;
		EXPECT_FALSE(info.covariance.empty()) << label;
	}
}

class RegistrationVisEstimationTypeTest : public ::testing::TestWithParam<int>
{
protected:
	ParametersMap params() const
	{
		return registrationVisTestParams(GetParam());
	}
};

} // namespace

TEST(RegistrationVisTest, ConstructorAndParseParameters)
{
	RegistrationVis reg(registrationVisTestParams());
	EXPECT_TRUE(reg.isImageRequired());
	EXPECT_FALSE(reg.isScanRequired());
	EXPECT_EQ(reg.getMinInliers(), 20);
	EXPECT_EQ(reg.getEstimationType(), 1);
	EXPECT_NE(reg.getDetector(), nullptr);

	ParametersMap update;
	update[Parameters::kVisMinInliers()] = "10";
	update[Parameters::kVisInlierDistance()] = "0.2";
	update[Parameters::kVisEstimationType()] = "2";
	reg.parseParameters(update);
	EXPECT_EQ(reg.getMinInliers(), 10);
	EXPECT_FLOAT_EQ(reg.getInlierDistance(), 0.2f);
	EXPECT_EQ(reg.getEstimationType(), 2);
}

TEST(RegistrationVisTest, ParseParametersWithGetters)
{
	struct Case
	{
		ParametersMap update;
		std::function<void(const RegistrationVis &)> check;
	};

	const std::vector<Case> cases = {
			{{{Parameters::kVisMinInliers(), "12"}},
					[](const RegistrationVis & reg) { EXPECT_EQ(reg.getMinInliers(), 12); }},
			{{{Parameters::kVisMinInliers(), "2"}},
					[](const RegistrationVis & reg) {
						EXPECT_EQ(reg.getMinInliers(), 6); // clamped to minimum 6
					}},
			{{{Parameters::kVisInlierDistance(), "0.25"}},
					[](const RegistrationVis & reg) { EXPECT_FLOAT_EQ(reg.getInlierDistance(), 0.25f); }},
			{{{Parameters::kVisIterations(), "150"}},
					[](const RegistrationVis & reg) { EXPECT_EQ(reg.getIterations(), 150); }},
			{{{Parameters::kVisEstimationType(), "0"}},
					[](const RegistrationVis & reg) { EXPECT_EQ(reg.getEstimationType(), 0); }},
			{{{Parameters::kVisEstimationType(), "2"}},
					[](const RegistrationVis & reg) { EXPECT_EQ(reg.getEstimationType(), 2); }},
			{{{Parameters::kVisCorNNDR(), "0.55"}},
					[](const RegistrationVis & reg) { EXPECT_FLOAT_EQ(reg.getNNDR(), 0.55f); }},
			{{{Parameters::kVisCorNNType(), "3"}},
					[](const RegistrationVis & reg) { EXPECT_EQ(reg.getNNType(), 3); }},
			{{{Parameters::kVisMaxFeatures(), "120"}},
					[](const RegistrationVis & reg) {
						ASSERT_NE(reg.getDetector(), nullptr);
						EXPECT_EQ(reg.getDetector()->getMaxFeatures(), 120);
					}},
			{{{Parameters::kVisFeatureType(), "2"}},
					[](const RegistrationVis & reg) {
						ASSERT_NE(reg.getDetector(), nullptr);
						EXPECT_EQ(reg.getDetector()->getType(), Feature2D::kFeatureOrb);
					}},
			{{{Parameters::kVisCorType(), "1"}},
					[](const RegistrationVis & reg) { EXPECT_TRUE(reg.canUseGuess()); }},
			{{{Parameters::kVisCorGuessWinSize(), "30"}},
					[](const RegistrationVis & reg) { EXPECT_TRUE(reg.canUseGuess()); }},
	};

	for(size_t i = 0; i < cases.size(); ++i)
	{
		RegistrationVis reg(registrationVisTestParams());
		reg.parseParameters(cases[i].update);
		cases[i].check(reg);
	}
}

TEST(RegistrationVisTest, ParseMaxFeaturesCapsExtractedKeypoints)
{
	const SensorData data = loadRgbdSensorData("17");
	ASSERT_FALSE(data.imageRaw().empty());

	ParametersMap params = registrationVisTestParams();
	params[Parameters::kVisMaxFeatures()] = "80";
	RegistrationVis reg(params);

	ASSERT_NE(reg.getDetector(), nullptr);
	EXPECT_EQ(reg.getDetector()->getMaxFeatures(), 80);

	Signature from(data);
	Signature to;
	RegistrationInfo info;
	Transform nullGuess;
	reg.computeTransformationMod(from, to, nullGuess, &info);

	EXPECT_LE(from.sensorData().keypoints().size(), 80u);
	EXPECT_GT(from.sensorData().keypoints().size(), 0u);
	EXPECT_FALSE(from.sensorData().descriptors().empty());
}

TEST(RegistrationVisTest, HighMinInliersRejectsRegistration)
{
	const SensorData fromData = loadRgbdSensorData("17");
	const SensorData toData = loadRgbdSensorData("154");
	ASSERT_FALSE(fromData.imageRaw().empty());
	ASSERT_FALSE(toData.imageRaw().empty());

	ParametersMap params = registrationVisTestParams(1);
	params[Parameters::kVisMinInliers()] = "10000";

	RegistrationInfo info;
	const Transform result = computeRegistration(fromData, toData, params, &info);
	EXPECT_TRUE(result.isNull());
}

TEST(RegistrationVisTest, StrictCorNndrReducesInliers)
{
	const SensorData fromData = loadRgbdSensorData("17");
	const SensorData toData = loadRgbdSensorData("154");
	ASSERT_FALSE(fromData.imageRaw().empty());
	ASSERT_FALSE(toData.imageRaw().empty());

	ParametersMap looseParams = registrationVisTestParams(1);
	looseParams[Parameters::kVisMinInliers()] = "6";
	looseParams[Parameters::kVisCorNNDR()] = "0.99";
	ParametersMap strictParams = registrationVisTestParams(1);
	strictParams[Parameters::kVisMinInliers()] = "6";
	strictParams[Parameters::kVisCorNNDR()] = "0.01";

	RegistrationInfo looseInfo;
	RegistrationInfo strictInfo;
	const Transform loose = computeRegistration(fromData, toData, looseParams, &looseInfo);
	const Transform strict = computeRegistration(fromData, toData, strictParams, &strictInfo);

	ASSERT_FALSE(loose.isNull());
	EXPECT_GE(looseInfo.inliers, strictInfo.inliers);
}

TEST(RegistrationVisTest, RgbdTwoFramesMatchExpectedTransformOpticalFlow)
{
	const SensorData fromData = loadRgbdSensorData("17");
	const SensorData toData = loadRgbdSensorData("154");
	ASSERT_FALSE(fromData.imageRaw().empty());
	ASSERT_FALSE(toData.imageRaw().empty());

	expectAllEstimationTypesMatchExpected(
			fromData,
			toData,
			kRgbd17To154Expected,
			kGoldenTransTolM,
			kGoldenAngleTolRad,
			[](int estimationType) { return registrationVisTestParams(estimationType, 1); });
}

TEST(RegistrationVisTest, StereoTwoFramesMatchExpectedTransformOpticalFlow)
{
	const SensorData fromData = loadStereoSensorData("50");
	const SensorData toData = loadStereoSensorData("60");
	ASSERT_FALSE(fromData.imageRaw().empty());
	ASSERT_FALSE(toData.imageRaw().empty());

	expectAllEstimationTypesMatchExpected(
			fromData,
			toData,
			kStereo50To60Expected,
			kGoldenTransTolM,
			kGoldenAngleTolRad,
			[](int estimationType) { return registrationVisTestParams(estimationType, 1); });
}

TEST_P(RegistrationVisEstimationTypeTest, RgbdSameFrameNearIdentity)
{
	const SensorData data = loadRgbdSensorData("17");
	ASSERT_FALSE(data.imageRaw().empty());
	ASSERT_FALSE(data.depthRaw().empty());

	RegistrationVis reg(registrationVisTestParams(GetParam()));
	EXPECT_EQ(reg.getEstimationType(), GetParam());
	EXPECT_EQ(reg.getDetector()->getMaxFeatures(), kVisMaxFeatures);

	const Signature from(data);
	const Signature to(data);

	RegistrationInfo info;
	Transform nullGuess;
	const Transform result = reg.computeTransformation(from, to, nullGuess, &info);

	expectSameFrameResult(GetParam(), result, info, reg.getMinInliers());
	if(!result.isNull())
	{
		EXPECT_GE(info.matches, info.inliers);
		EXPECT_FALSE(info.covariance.empty());
	}
}

TEST(RegistrationVisTest, RgbdTwoFramesMatchExpectedTransform)
{
	const SensorData fromData = loadRgbdSensorData("17");
	const SensorData toData = loadRgbdSensorData("154");
	ASSERT_FALSE(fromData.imageRaw().empty());
	ASSERT_FALSE(toData.imageRaw().empty());

	expectAllEstimationTypesMatchExpected(
			fromData,
			toData,
			kRgbd17To154Expected,
			kGoldenTransTolM,
			kGoldenAngleTolRad,
			[](int estimationType) { return registrationVisTestParams(estimationType, 0); });
}

TEST_P(RegistrationVisEstimationTypeTest, StereoSameFrameNearIdentity)
{
	const SensorData data = loadStereoSensorData("50");
	ASSERT_FALSE(data.imageRaw().empty());
	ASSERT_FALSE(data.rightRaw().empty());
	ASSERT_FALSE(data.stereoCameraModels().empty());

	RegistrationVis reg(params());
	EXPECT_EQ(reg.getEstimationType(), GetParam());

	const Signature from(data);
	const Signature to(data);

	RegistrationInfo info;
	Transform nullGuess;
	const Transform result = reg.computeTransformation(from, to, nullGuess, &info);

	expectSameFrameResult(GetParam(), result, info, reg.getMinInliers());
	if(!result.isNull())
	{
		EXPECT_FALSE(info.covariance.empty());
	}
}

TEST(RegistrationVisTest, StereoTwoFramesMatchExpectedTransform)
{
	const SensorData fromData = loadStereoSensorData("50");
	const SensorData toData = loadStereoSensorData("60");
	ASSERT_FALSE(fromData.imageRaw().empty());
	ASSERT_FALSE(toData.imageRaw().empty());

	expectAllEstimationTypesMatchExpected(
			fromData, toData, kStereo50To60Expected, kGoldenTransTolM, kGoldenAngleTolRad);
}

TEST(RegistrationVisTest, StereoFeatureMatchingAndOpticalFlowSucceed3DTo3D)
{
	const SensorData fromData = loadStereoSensorData("50");
	const SensorData toData = loadStereoSensorData("60");
	ASSERT_FALSE(fromData.imageRaw().empty());
	ASSERT_FALSE(toData.imageRaw().empty());

	RegistrationInfo info;
	const Transform fm = computeRegistration(fromData, toData, registrationVisTestParams(0, 0), &info);
	const Transform of = computeRegistration(fromData, toData, registrationVisTestParams(0, 1), &info);
	ASSERT_FALSE(fm.isNull());
	ASSERT_FALSE(of.isNull());
	EXPECT_LT(fm.getDistance(of), kGoldenTransTolM);
}

#if defined(RTABMAP_G2O)
// Guess based matching (Vis/CorGuessWinSize): the words of "from" are projected
// into "to" with the guess, and only the features within a radius of their
// projection are compared. Vis/CorGuessMatchToProjection picks which of the two
// sets is indexed and which one searches it, so both directions index 2D points
// and search them by radius.
TEST(RegistrationVisTest, RgbdGuessMatchingBothDirections)
{
	const SensorData fromData = loadRgbdSensorData("17");
	const SensorData toData = loadRgbdSensorData("154");
	ASSERT_FALSE(fromData.imageRaw().empty());
	ASSERT_FALSE(toData.imageRaw().empty());

	// A guess a few centimeters and a degree away from the answer, the way
	// odometry gives one. Far enough to matter, close enough for the projected
	// words to land inside the search radius.
	const Transform expected = kRgbd17To154Expected[1]; // PnP
	const Transform guess = expected * Transform(0.03f, -0.02f, 0.01f, 0.0f, 0.0f, 0.02f);

	for(bool matchToProjection: {false, true})
	{
		ParametersMap params = registrationVisTestParams(1 /* PnP */);
		params[Parameters::kVisCorGuessMatchToProjection()] = matchToProjection?"true":"false";

		RegistrationInfo info;
		const Transform result = computeRegistrationRobust(fromData, toData, params, &info, guess);

		const std::string label = std::string(Parameters::kVisCorGuessMatchToProjection()) +
				(matchToProjection?"=true":"=false");
		expectTransformNearExpected(result, expected, kGoldenTransTolM, kGoldenAngleTolRad, label);
		EXPECT_GE(info.inliers, 6) << label;
		// Only the guess based matching fills projectedIDs, whichever of its two
		// directions is taken, so this is what tells that it produced the
		// result rather than the plain descriptor matching. OdometryF2M relies
		// on them being filled to know which words of its map are still seen.
		EXPECT_FALSE(info.projectedIDs.empty()) << label;
	}
}

TEST(RegistrationVisTest, RgbdTwoFramesMatchExpectedTransformWithG2oBundleAdjustment)
{
	const SensorData fromData = loadRgbdSensorData("17");
	const SensorData toData = loadRgbdSensorData("154");
	ASSERT_FALSE(fromData.imageRaw().empty());
	ASSERT_FALSE(toData.imageRaw().empty());

	expectAllEstimationTypesMatchExpected(
			fromData,
			toData,
			kRgbd17To154Expected,
			kGoldenTransTolM,
			kGoldenAngleTolRad,
			[](int estimationType) {
				return registrationVisTestParamsWithG2oBundleAdjustment(estimationType, 0);
			});
}

TEST(RegistrationVisTest, StereoTwoFramesMatchExpectedTransformWithG2oBundleAdjustment)
{
	const SensorData fromData = loadStereoSensorData("50");
	const SensorData toData = loadStereoSensorData("60");
	ASSERT_FALSE(fromData.imageRaw().empty());
	ASSERT_FALSE(toData.imageRaw().empty());

	expectAllEstimationTypesMatchExpected(
			fromData,
			toData,
			kStereo50To60Expected,
			kGoldenTransTolM,
			kGoldenAngleTolRad,
			[](int estimationType) {
				return registrationVisTestParamsWithG2oBundleAdjustment(estimationType);
			});
}
#endif

INSTANTIATE_TEST_SUITE_P(
		AllEstimationTypes,
		RegistrationVisEstimationTypeTest,
		::testing::Values(0, 1, 2),
		[](const ::testing::TestParamInfo<int> & info) {
			return estimationTypeName(info.param);
		});

// Regenerate golden Transform(...) lines: build and run
//   ./bin/test_registrationvis --gtest_also_run_disabled_tests --gtest_filter='*CaptureGoldenTransforms*'
TEST(RegistrationVisTest, DISABLED_CaptureGoldenTransforms)
{
	auto printTransform = [](const char * label, const Transform & t) {
		printf("\t\t// %s xyz=(%.8f,%.8f,%.8f)\n", label, t.x(), t.y(), t.z());
		printf("\t\tTransform(%.8ff, %.8ff, %.8ff, %.8ff,\n", t.r11(), t.r12(), t.r13(), t.x());
		printf("\t\t\t\t%.8ff, %.8ff, %.8ff, %.8ff,\n", t.r21(), t.r22(), t.r23(), t.y());
		printf("\t\t\t\t%.8ff, %.8ff, %.8ff, %.8ff),\n", t.r31(), t.r32(), t.r33(), t.z());
	};
	const SensorData rgbdFrom = loadRgbdSensorData("17");
	const SensorData rgbdTo = loadRgbdSensorData("154");
	const SensorData stereoFrom = loadStereoSensorData("50");
	const SensorData stereoTo = loadStereoSensorData("60");
	ASSERT_FALSE(rgbdFrom.imageRaw().empty());
	ASSERT_FALSE(stereoFrom.imageRaw().empty());

	auto capture = [&](const char * setName, const SensorData & from, const SensorData & to) {
		printf("// %s (Vis/CorType=0; shared golden for FM and OF)\n", setName);
		printf("static const Transform kExpected[3] = {\n");
		for(int estimationType = 0; estimationType <= 2; ++estimationType)
		{
			RegistrationInfo info;
			const Transform result = computeRegistrationRobust(
					from, to, registrationVisTestParams(estimationType, 0), &info);
			printTransform(estimationTypeName(estimationType).c_str(), result);
		}
		printf("};\n");
	};
	capture("rgbd/17 -> rgbd/154", rgbdFrom, rgbdTo);
	capture("stereo_rect/50 -> stereo_rect/60", stereoFrom, stereoTo);

	auto printFmOfDist = [&](const char * label, const SensorData & from, const SensorData & to) {
		for(int estimationType = 0; estimationType <= 2; ++estimationType)
		{
			const Transform fm = computeRegistrationRobust(
					from, to, registrationVisTestParams(estimationType, 0));
			const Transform of = computeRegistrationRobust(
					from, to, registrationVisTestParams(estimationType, 1));
			printf("// %s %s FM-OF distance: %.6f m\n",
					label, estimationTypeName(estimationType).c_str(), fm.getDistance(of));
		}
	};
	printFmOfDist("rgbd", rgbdFrom, rgbdTo);
	printFmOfDist("stereo", stereoFrom, stereoTo);
}

// Guess based matching on a two-camera rig: a projected map point whose window in
// the second camera holds a single keypoint that doesn't look like the feature
// (another descriptor) is not matched to it: the keypoint fails the distance
// ratio (Vis/CorNNDR) against the other keypoints of that camera, as a match
// among several candidates would. A ratio accepting anything restores the old
// behavior, where the lone keypoint was taken.
TEST(RegistrationVisTest, GuessMatchingRejectsALoneKeypointWithAnotherDescriptor)
{
#ifndef RTABMAP_OPENGV
	GTEST_SKIP() << "multi-camera PnP needs OpenGV";
#endif
	// Two cameras fanned out in yaw, 116 deg of field of view each
	std::vector<CameraModel> rig;
	rig.push_back(CameraModel(200.0, 200.0, 320.0, 240.0, Transform(0.0f, -0.15f, 0.0f, 0.0f, 0.0f, -0.4f)*CameraModel::opticalRotation(), 0.0, cv::Size(640, 480)));
	rig.push_back(CameraModel(200.0, 200.0, 320.0, 240.0, Transform(0.0f, 0.15f, 0.0f, 0.0f, 0.0f, 0.4f)*CameraModel::opticalRotation(), 0.0, cv::Size(640, 480)));

	// Landmarks on a 90 px grid of each camera's image, 2 to 3 m away, with their
	// own descriptors: far enough apart for a projection to have a single keypoint
	// in its window when its decoy is the only one around
	cv::RNG rng(5);
	std::vector<cv::Point3f> landmarks;
	for(size_t camera=0; camera<rig.size(); ++camera)
	{
		for(int v=70; v<=430; v+=90)
		{
			for(int u=70; u<=580; u+=90)
			{
				float x, y, z;
				rig[camera].project((float)u, (float)v, 2.0f + 0.1f*float((u/90 + v/90) % 10), x, y, z);
				const cv::Point3f landmark = util3d::transformPoint(cv::Point3f(x, y, z), rig[camera].localTransform());
				// the second camera's grid only where the first one doesn't see:
				// no interleaving of the two grids in the overlap
				const cv::Point3f inFirst = util3d::transformPoint(landmark, rig[0].localTransform().inverse());
				float uFirst, vFirst;
				rig[0].reproject(inFirst.x, inFirst.y, inFirst.z, uFirst, vFirst);
				if(camera == 0 || inFirst.z <= 0.0f || uFirst < 0.0f || uFirst >= rig[0].imageWidth() || vFirst < 0.0f || vFirst >= rig[0].imageHeight())
				{
					landmarks.push_back(landmark);
				}
			}
		}
	}
	// A few close pairs (12 cm apart, about 10 px), so that some windows hold
	// several candidates and the ratio test has matches to calibrate the check
	const size_t gridCount = landmarks.size();
	for(size_t i=1; i<gridCount; i+=5)
	{
		landmarks.push_back(landmarks[i] + cv::Point3f(0.0f, 0.12f, 0.0f));
	}
	const int count = (int)landmarks.size();
	cv::Mat descriptors(count, 32, CV_8UC1);
	rng.fill(descriptors, cv::RNG::UNIFORM, 0, 256);
	// The keypoint of a point of the frame's base frame in a camera, in the stitched image
	auto observe = [&rig](const cv::Point3f & inBase, int camera, float & u, float & v)
	{
		const cv::Point3f inCamera = util3d::transformPoint(inBase, rig[camera].localTransform().inverse());
		if(inCamera.z <= 0.3f)
		{
			return false;
		}
		rig[camera].reproject(inCamera.x, inCamera.y, inCamera.z, u, v);
		if(u < 30.0f || u > rig[camera].imageWidth()-30.0f || v < 30.0f || v > rig[camera].imageHeight()-30.0f)
		{
			return false;
		}
		u += float(camera)*rig[camera].imageWidth();
		return true;
	};

	// The map ("from"): one observation per landmark, from the origin
	std::multimap<int, int> mapWords;
	std::vector<cv::KeyPoint> mapKpts;
	std::vector<cv::Point3f> map3D;
	cv::Mat mapDesc;
	for(int i=0; i<count; ++i)
	{
		float u, v;
		if(observe(landmarks[i], 0, u, v) || observe(landmarks[i], 1, u, v))
		{
			mapWords.insert(std::make_pair(i+1, (int)mapKpts.size()));
			mapKpts.push_back(cv::KeyPoint(u, v, 3.0f));
			map3D.push_back(landmarks[i]);
			mapDesc.push_back(descriptors.row(i));
		}
	}

	// The frame ("to") a little further: every landmark seen with its own keypoint
	// and, 15 px away, a decoy with another descriptor (the ratio test picks the
	// right one). For the trap landmarks, the second camera only has the decoy, 10
	// px from the projection: a lone keypoint that is not the feature.
	const Transform pose(0.05f, 0.02f, 0.0f, 0.0f, 0.0f, 0.01f);
	const float nan = std::numeric_limits<float>::quiet_NaN();
	std::vector<cv::KeyPoint> frameKpts;
	std::vector<cv::Point3f> frame3D;
	cv::Mat frameDesc;
	std::set<int> trapWords;
	for(int i=0; i<count; ++i)
	{
		const cv::Point3f inBase = util3d::transformPoint(landmarks[i], pose.inverse());
		float u0, v0, u1, v1;
		const bool in0 = observe(inBase, 0, u0, v0);
		const bool in1 = observe(inBase, 1, u1, v1);
		const bool trap = in0 && in1 && i % 4 == 0 && i < (int)gridCount && i % 5 != 1;
		if(trap)
		{
			trapWords.insert(i+1);
		}
		for(int camera=0; camera<2; ++camera)
		{
			if(!(camera == 0 ? in0 : in1))
			{
				continue;
			}
			const float u = camera == 0 ? u0 : u1;
			const float v = camera == 0 ? v0 : v1;
			cv::Mat decoy(1, 32, CV_8UC1);
			rng.fill(decoy, cv::RNG::UNIFORM, 0, 256);
			if(trap && camera == 1)
			{
				frameKpts.push_back(cv::KeyPoint(u + 10.0f, v, 3.0f));
				frame3D.push_back(cv::Point3f(nan, nan, nan));
				frameDesc.push_back(decoy);
				continue;
			}
			frameKpts.push_back(cv::KeyPoint(u, v, 3.0f));
			frame3D.push_back(inBase);
			frameDesc.push_back(descriptors.row(i));
			frameKpts.push_back(cv::KeyPoint(u + 15.0f, v, 3.0f));
			frame3D.push_back(cv::Point3f(nan, nan, nan));
			frameDesc.push_back(decoy);
		}
	}
	ASSERT_GE(trapWords.size(), 4u);

	// With the default distance ratio, in both matching directions, and with a
	// ratio accepting anything
	struct Case { float nndr; bool matchToProjection; };
	for(const Case & c : {Case{0.8f, false}, Case{0.8f, true}, Case{10.0f, false}})
	{
		ParametersMap params;
		params.insert(ParametersPair(Parameters::kVisEstimationType(), "1"));
		params.insert(ParametersPair(Parameters::kVisCorGuessWinSize(), "25"));
		params.insert(ParametersPair(Parameters::kVisCorNNDR(), uNumber2Str(c.nndr)));
		params.insert(ParametersPair(Parameters::kVisCorGuessMatchToProjection(), c.matchToProjection?"true":"false"));
		RegistrationVis reg(params);

		Signature from(1);
		from.setWords(mapWords, mapKpts, map3D, mapDesc);
		Signature to(2);
		to.sensorData() = SensorData(cv::Mat(), cv::Mat(), rig, 2);
		to.sensorData().setFeatures(frameKpts, frame3D, frameDesc);

		RegistrationInfo info;
		const Transform result = reg.computeTransformationMod(from, to, pose, &info);
		const std::string label = uFormat("nndr=%.1f matchToProjection=%d", c.nndr, c.matchToProjection?1:0);
		ASSERT_FALSE(result.isNull()) << label << ": " << info.rejectedMsg;
		EXPECT_NEAR(result.x(), pose.x(), 0.01f) << label;
		EXPECT_NEAR(result.y(), pose.y(), 0.01f) << label;
		ASSERT_EQ(info.inliersIDsPerCam.size(), 2u) << label;

		// The lone decoys matched to the trap words in the second camera
		int trapsMatchedInSecondCamera = 0;
		int trapsInlierInSecondCamera = 0;
		for(int id : trapWords)
		{
			for(std::multimap<int, int>::const_iterator iter=to.getWords().lower_bound(id); iter!=to.getWords().end() && iter->first == id; ++iter)
			{
				trapsMatchedInSecondCamera += to.getWordsKpts()[iter->second].pt.x >= rig[0].imageWidth() ? 1 : 0;
			}
			trapsInlierInSecondCamera += std::count(info.inliersIDsPerCam[1].begin(), info.inliersIDsPerCam[1].end(), id);
		}
		if(c.nndr < 1.0f)
		{
			EXPECT_EQ(trapsMatchedInSecondCamera, 0) << label << ": a lone keypoint with another descriptor is not the feature";
		}
		else
		{
			EXPECT_EQ(trapsMatchedInSecondCamera, (int)trapWords.size()) << label << ": without the check the lone keypoint is taken";
		}
		// Either way, those wrong observations are not inliers of the motion estimation
		EXPECT_EQ(trapsInlierInSecondCamera, 0) << label;
		// The per-camera inlier ids are consistent with the per-camera counts
		for(size_t cam=0; cam<2; ++cam)
		{
			EXPECT_EQ((int)info.inliersIDsPerCam[cam].size(), info.inliersPerCam[cam]) << label;
		}
		EXPECT_GE(info.inliers, 30) << label;
	}
}
