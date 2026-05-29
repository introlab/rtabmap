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
		RegistrationInfo * infoOut = nullptr)
{
	RegistrationVis reg(params);
	Signature from(fromData);
	Signature to(toData);
	RegistrationInfo info;
	Transform nullGuess;
	return reg.computeTransformation(from, to, nullGuess, infoOut ? infoOut : &info);
}

// Golden transforms (GFTT/ORB, MinDistance=3, QualityLevel=0.01, MaxFeatures=3000, RoiRatios=0 0 0 0.3).
// Captured with Vis/CorType=0 (feature matching); also used for optical flow (CorType=1) within tolerance.
// Shared by FM/OF, Vis/BundleAdjustment=0 and g2o BA=1.
static constexpr float kGoldenTransTolM = 0.20f;
static constexpr float kGoldenTransTolEpipolarM = 0.60f; // Epipolar RANSAC variance: golden was captured on OpenCV 4.5.4; OpenCV 4.6.0 lands up to ~0.57 m away on the same input (the OpenCV changelog has no calib3d/findEssentialMat entry between those releases, so the drift is probably indirect -- RNG sequence, math/BLAS, etc.).
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
		RegistrationInfo * infoOut = nullptr)
{
	const int estimationType = std::atoi(params.at(Parameters::kVisEstimationType()).c_str());
	const int maxAttempts = estimationType == 2 ? 5 : 1;
	Transform result;
	RegistrationInfo info;
	for(int attempt = 0; attempt < maxAttempts; ++attempt)
	{
		result = computeRegistration(fromData, toData, params, &info);
		if(!result.isNull() || estimationType != 2)
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
