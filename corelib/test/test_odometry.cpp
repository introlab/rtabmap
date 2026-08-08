#include <gtest/gtest.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <memory>
#include <string>

using namespace rtabmap;

namespace {

class MockOdometry : public Odometry
{
public:
	explicit MockOdometry(const ParametersMap & parameters = ParametersMap())
		: Odometry(parameters)
	{
	}

	Type getType() override
	{
		return kTypeUndef;
	}

	void setNextTransform(const Transform & transform)
	{
		nextTransform_ = transform;
	}

	const Transform & lastGuess() const
	{
		return lastGuess_;
	}

protected:
	Transform computeTransform(SensorData &, const Transform & guess, OdometryInfo *) override
	{
		lastGuess_ = guess;
		return nextTransform_;
	}

private:
	Transform nextTransform_;
	Transform lastGuess_;
};

SensorData makeImageData(int id, double stamp)
{
	const cv::Mat image = cv::Mat::zeros(32, 32, CV_8UC1);
	const CameraModel model(100.0, 100.0, 16.0, 16.0);
	SensorData data(image, model, id);
	data.setStamp(stamp);
	return data;
}

ParametersMap baseTestParameters()
{
	ParametersMap parameters;
	parameters.insert(ParametersPair(Parameters::kOdomGuessMotion(), "false"));
	parameters.insert(ParametersPair(Parameters::kOdomFilteringStrategy(), "0"));
	parameters.insert(ParametersPair(Parameters::kOdomFillInfoData(), "true"));
	parameters.insert(ParametersPair(Parameters::kRtabmapImagesAlreadyRectified(), "true"));
	return parameters;
}

void expectTransformNear(const Transform & a, const Transform & b, float tol = 1e-4f)
{
	EXPECT_NEAR(a.x(), b.x(), tol);
	EXPECT_NEAR(a.y(), b.y(), tol);
	EXPECT_NEAR(a.z(), b.z(), tol);
}

} // namespace

TEST(OdometryTest, CreateUsesF2MByDefault)
{
	ParametersMap parameters;
	parameters.insert(ParametersPair(Parameters::kOdomStrategy(), "0"));

	Odometry * odometry = Odometry::create(parameters);
	ASSERT_NE(odometry, nullptr);
	EXPECT_EQ(odometry->getType(), Odometry::kTypeF2M);
	delete odometry;
}

TEST(OdometryTest, CreateUnknownTypeFallsBackToF2M)
{
	Odometry::Type type = static_cast<Odometry::Type>(999);
	Odometry * odometry = Odometry::create(type);
	ASSERT_NE(odometry, nullptr);
	EXPECT_EQ(type, Odometry::kTypeF2M);
	EXPECT_EQ(odometry->getType(), Odometry::kTypeF2M);
	delete odometry;
}

TEST(OdometryTest, ResetSetsInitialPoseAndCounters)
{
	MockOdometry odometry(baseTestParameters());
	const Transform initialPose(1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f);
	odometry.reset(initialPose);

	EXPECT_EQ(odometry.framesProcessed(), 0u);
	EXPECT_DOUBLE_EQ(odometry.previousStamp(), 0.0);
	expectTransformNear(odometry.getPose(), initialPose);
	EXPECT_TRUE(odometry.getVelocityGuess().isNull());
}

TEST(OdometryTest, ProcessAccumulatesPose)
{
	MockOdometry odometry(baseTestParameters());
	odometry.reset(Transform::getIdentity());

	const Transform step(0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	odometry.setNextTransform(step);

	SensorData frame0 = makeImageData(0, 0.0);
	const Transform pose0 = odometry.process(frame0);
	expectTransformNear(pose0, step);
	expectTransformNear(odometry.getPose(), pose0);
	EXPECT_EQ(odometry.framesProcessed(), 1u);

	SensorData frame1 = makeImageData(1, 1.0);
	const Transform pose1 = odometry.process(frame1);
	const Transform expectedPose1(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	expectTransformNear(pose1, expectedPose1);
	expectTransformNear(odometry.getPose(), expectedPose1);
	EXPECT_EQ(odometry.framesProcessed(), 2u);
}

TEST(OdometryTest, ProcessWithExternalGuess)
{
	MockOdometry odometry(baseTestParameters());
	odometry.reset(Transform::getIdentity());
	odometry.setNextTransform(Transform(0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));

	SensorData data = makeImageData(0, 0.0);
	const Transform guess(0.2f, 0.3f, 0.0f, 0.0f, 0.0f, 0.0f);
	odometry.process(data, guess);

	expectTransformNear(odometry.lastGuess(), guess);
}

TEST(OdometryTest, ProcessFillsOdometryInfo)
{
	MockOdometry odometry(baseTestParameters());
	odometry.reset(Transform::getIdentity());
	odometry.setNextTransform(Transform(0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));

	SensorData data = makeImageData(0, 1.0);
	OdometryInfo info;
	const Transform pose = odometry.process(data, &info);

	EXPECT_FALSE(pose.isNull());
	EXPECT_FALSE(info.lost);
	EXPECT_DOUBLE_EQ(info.stamp, 1.0);
	expectTransformNear(info.transform, Transform(0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	expectTransformNear(pose, Transform(0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	EXPECT_TRUE(odometry.isInfoDataFilled());
}

TEST(OdometryTest, ProcessLostReturnsNullTransform)
{
	MockOdometry odometry(baseTestParameters());
	odometry.reset(Transform::getIdentity());
	odometry.setNextTransform(Transform());

	SensorData data = makeImageData(0, 0.0);
	OdometryInfo info;
	const Transform t = odometry.process(data, &info);

	EXPECT_TRUE(t.isNull());
	EXPECT_TRUE(info.lost);
	EXPECT_EQ(odometry.framesProcessed(), 0u);
}

TEST(OdometryTest, DefaultCapabilityFlags)
{
	MockOdometry odometry(baseTestParameters());
	EXPECT_FALSE(odometry.canProcessRawImages());
	EXPECT_FALSE(odometry.canProcessAsyncIMU());
}

// ---------------------------------------------------------------------------
// Strategies driven through the public API on real frames committed under
// data/. The tests above use MockOdometry, which overrides computeTransform(),
// so they exercise the base class but never enter a backend implementation.
// These run the real strategies end to end -- feature extraction,
// correspondence and motion estimation all execute. Parameterized on
// Odom/Strategy, so covering another backend is one list entry.
// ---------------------------------------------------------------------------

namespace {

const char * strategyName(Odometry::Type type)
{
	switch(type)
	{
	case Odometry::kTypeF2M: return "F2M";
	case Odometry::kTypeF2F: return "F2F";
	default:                 return "other";
	}
}

// data/stereo_rect holds two rectified pairs (50 and 60) plus the calibration:
// enough for one initialisation frame and one motion estimate.
SensorData loadStereoFrame(const std::string & name, int id, double stamp)
{
	const std::string root(RTABMAP_TEST_DATA_ROOT);
	const cv::Mat left  = cv::imread(root + "/stereo_rect/left/"  + name + ".jpg", cv::IMREAD_GRAYSCALE);
	const cv::Mat right = cv::imread(root + "/stereo_rect/right/" + name + ".jpg", cv::IMREAD_GRAYSCALE);
	StereoCameraModel model;
	if(left.empty() || right.empty() || !model.load(root + "/stereo_rect", "stereo"))
	{
		return SensorData();
	}
	return SensorData(left, right, model, id, stamp);
}

// data/rgbd holds two RGB-D frames (17 and 154) with per-frame calibration.
SensorData loadRgbdFrame(const std::string & name, int id, double stamp)
{
	const std::string root(RTABMAP_TEST_DATA_ROOT);
	const cv::Mat rgb   = cv::imread(root + "/rgbd/rgb/"   + name + ".jpg", cv::IMREAD_COLOR);
	const cv::Mat depth = cv::imread(root + "/rgbd/depth/" + name + ".png", cv::IMREAD_UNCHANGED);
	CameraModel model;
	if(rgb.empty() || depth.empty() || !model.load(root + "/rgbd/calib", name))
	{
		return SensorData();
	}
	return SensorData(rgb, depth, model, id, stamp);
}

class OdometryStrategyTest : public ::testing::TestWithParam<Odometry::Type>
{
protected:
	// Deliberately minimal: exercise each strategy's default path rather than
	// a tuned configuration.
	std::unique_ptr<Odometry> createOdometry() const
	{
		ParametersMap parameters;
		parameters.insert(ParametersPair(Parameters::kOdomStrategy(),
				uNumber2Str(static_cast<int>(GetParam()))));
		std::unique_ptr<Odometry> odometry(Odometry::create(parameters));
		if(odometry && odometry->getType() != GetParam())
		{
			// create() falls back to F2M for a backend that is not compiled
			// in, which would otherwise silently test F2M twice.
			return nullptr;
		}
		return odometry;
	}
};

}  // namespace

// A first frame has nothing to register against: every strategy should accept
// it, report the identity pose, and initialise its map / previous frame.
TEST_P(OdometryStrategyTest, FirstStereoFrameInitialisesAtOrigin)
{
	std::unique_ptr<Odometry> odometry = createOdometry();
	if(!odometry)
	{
		GTEST_SKIP() << strategyName(GetParam()) << " not built in";
	}

	SensorData first = loadStereoFrame("50", 1, 0.0);
	ASSERT_FALSE(first.imageRaw().empty()) << "data/stereo_rect/left/50.jpg missing";

	OdometryInfo info;
	const Transform pose = odometry->process(first, &info);

	EXPECT_FALSE(pose.isNull());
	EXPECT_TRUE(pose.isIdentity()) << "first pose should be the origin, got " << pose.prettyPrint();
	EXPECT_EQ(0, info.reg.inliers) << "nothing to register against on the first frame";
}

// Second frame: the strategy must recover a real motion between the two
// rectified pairs. The bounds are loose on purpose -- the claim is "a
// plausible, non-degenerate transform from real correspondences", not a
// specific value, which differs per backend.
TEST_P(OdometryStrategyTest, StereoPairRecoversMotion)
{
	std::unique_ptr<Odometry> odometry = createOdometry();
	if(!odometry)
	{
		GTEST_SKIP() << strategyName(GetParam()) << " not built in";
	}

	SensorData first  = loadStereoFrame("50", 1, 0.0);
	SensorData second = loadStereoFrame("60", 2, 0.1);
	ASSERT_FALSE(first.imageRaw().empty());
	ASSERT_FALSE(second.imageRaw().empty());

	OdometryInfo firstInfo;
	ASSERT_FALSE(odometry->process(first, &firstInfo).isNull());

	OdometryInfo info;
	const Transform pose = odometry->process(second, &info);

	ASSERT_FALSE(pose.isNull())
			<< strategyName(GetParam()) << " lost tracking between frames 50 and 60";
	EXPECT_GT(info.reg.inliers, 20)
			<< strategyName(GetParam()) << " too few inliers (matches=" << info.reg.matches << ")";
	EXPECT_LE(info.reg.inliers, info.reg.matches);

	// The frames are ~15 cm apart; beyond a metre the estimate diverged.
	const float distance = pose.getNorm();
	EXPECT_GT(distance, 0.01f) << strategyName(GetParam()) << " reported no motion at all";
	EXPECT_LT(distance, 1.0f)  << strategyName(GetParam()) << " implausible motion "
			<< pose.prettyPrint();
}

// Same entry point with RGB-D input, so the depth-to-3D path runs instead of
// stereo correspondence. Frames 17 and 154 are far apart in the sequence, so
// losing tracking is a legitimate outcome; what must hold is that the reported
// info agrees with the returned transform.
TEST_P(OdometryStrategyTest, HandlesRgbdFrames)
{
	std::unique_ptr<Odometry> odometry = createOdometry();
	if(!odometry)
	{
		GTEST_SKIP() << strategyName(GetParam()) << " not built in";
	}

	SensorData first  = loadRgbdFrame("17", 1, 0.0);
	SensorData second = loadRgbdFrame("154", 2, 0.1);
	ASSERT_FALSE(first.imageRaw().empty())  << "data/rgbd/rgb/17.jpg missing";
	ASSERT_FALSE(second.imageRaw().empty()) << "data/rgbd/rgb/154.jpg missing";

	OdometryInfo firstInfo;
	EXPECT_FALSE(odometry->process(first, &firstInfo).isNull());

	OdometryInfo info;
	const Transform pose = odometry->process(second, &info);
	if(pose.isNull())
	{
		EXPECT_LT(info.reg.inliers, 20) << "null transform but plenty of inliers";
	}
	else
	{
		EXPECT_GT(info.reg.inliers, 0);
		EXPECT_LT(pose.getNorm(), 20.0f) << "implausible jump " << pose.prettyPrint();
	}
}

INSTANTIATE_TEST_SUITE_P(
		Strategies,
		OdometryStrategyTest,
		::testing::Values(Odometry::kTypeF2M, Odometry::kTypeF2F),
		[](const ::testing::TestParamInfo<Odometry::Type> & info) {
			return strategyName(info.param);
		});

// ---------------------------------------------------------------------------
// Scan-matching odometry (Reg/Strategy=1). The visual tests above depend on
// real imagery; these use synthetic geometry instead, which lets the expected
// motion be known exactly rather than merely plausible.
//
// A corner is used on purpose: two perpendicular surfaces constrain every
// translational DoF, so ICP has a unique optimum. A single plane or a corridor
// would leave a direction unobservable and the test would pass or fail on the
// initial guess rather than on the registration.
// ---------------------------------------------------------------------------

namespace {

// 2D corner: floor at y=-half and wall at x=-half, expressed in the sensor
// frame of the first observation. Jitter keeps the per-point normals
// well-defined (see the same construction in test_registrationicp.cpp).
LaserScan makeCorner2D(float length = 4.0f, int pointsPerLine = 400, uint64_t seed = 0xC0FFEE)
{
	cv::RNG rng(seed);
	cv::Mat data(1, 2 * pointsPerLine, CV_32FC2);
	const float half = 0.5f * length;
	int idx = 0;
	for(int i = 0; i < pointsPerLine; ++i, ++idx)
	{
		data.at<cv::Vec2f>(0, idx) = cv::Vec2f(
				rng.uniform(-half, half), -half + rng.uniform(-0.005f, 0.005f));
	}
	for(int i = 0; i < pointsPerLine; ++i, ++idx)
	{
		data.at<cv::Vec2f>(0, idx) = cv::Vec2f(
				-half + rng.uniform(-0.005f, 0.005f), rng.uniform(-half, half));
	}
	return LaserScan(data, /*maxPoints=*/0, /*maxRange=*/0.0f, LaserScan::kXY);
}

// 3D corner: floor plus two walls, so all 6 DoF are constrained.
LaserScan makeCorner3D(float length = 4.0f, int pointsPerSurface = 400, uint64_t seed = 0xC0FFEE)
{
	cv::RNG rng(seed);
	cv::Mat data(1, 3 * pointsPerSurface, CV_32FC3);
	const float half = 0.5f * length;
	int idx = 0;
	for(int i = 0; i < pointsPerSurface; ++i, ++idx)  // floor z=-half
	{
		data.at<cv::Vec3f>(0, idx) = cv::Vec3f(rng.uniform(-half, half), rng.uniform(-half, half),
				-half + static_cast<float>(rng.gaussian(0.005)));
	}
	for(int i = 0; i < pointsPerSurface; ++i, ++idx)  // wall x=-half
	{
		data.at<cv::Vec3f>(0, idx) = cv::Vec3f(-half + static_cast<float>(rng.gaussian(0.005)),
				rng.uniform(-half, half), rng.uniform(-half, half));
	}
	for(int i = 0; i < pointsPerSurface; ++i, ++idx)  // wall y=-half
	{
		data.at<cv::Vec3f>(0, idx) = cv::Vec3f(rng.uniform(-half, half),
				-half + static_cast<float>(rng.gaussian(0.005)), rng.uniform(-half, half));
	}
	return LaserScan(data, /*maxPoints=*/0, /*maxRange=*/0.0f, LaserScan::kXYZ);
}

SensorData makeScanData(const LaserScan & scan, int id, double stamp)
{
	SensorData data;
	data.setId(id);
	data.setStamp(stamp);
	data.setLaserScan(scan);
	return data;
}

ParametersMap icpOdometryParameters(Odometry::Type type, bool force3DoF, bool guessMotion)
{
	ParametersMap parameters;
	parameters.insert(ParametersPair(Parameters::kOdomStrategy(), uNumber2Str(static_cast<int>(type))));
	parameters.insert(ParametersPair(Parameters::kRegStrategy(), "1"));  // ICP
	parameters.insert(ParametersPair(Parameters::kRegForce3DoF(), force3DoF ? "true" : "false"));
	parameters.insert(ParametersPair(Parameters::kOdomGuessMotion(), guessMotion ? "true" : "false"));
	parameters.insert(ParametersPair(Parameters::kIcpPointToPlane(), "false"));
	parameters.insert(ParametersPair(Parameters::kIcpVoxelSize(), "0.0"));
	parameters.insert(ParametersPair(Parameters::kIcpCorrespondenceRatio(), "0.1"));
	return parameters;
}

// Distance between two transforms, reported as translation and angle so a
// failure says which part diverged.
void expectPoseNear(const Transform & actual, const Transform & expected,
		float transTol, float angTolDeg, const std::string & what)
{
	ASSERT_FALSE(actual.isNull()) << what << ": null transform";
	EXPECT_LT(actual.getDistance(expected), transTol)
			<< what << ": translation off -- got " << actual.prettyPrint()
			<< " expected " << expected.prettyPrint();
	const float angDeg = actual.getAngle(expected) * 180.0f / static_cast<float>(CV_PI);
	EXPECT_LT(angDeg, angTolDeg)
			<< what << ": rotation off by " << angDeg << " deg -- got " << actual.prettyPrint()
			<< " expected " << expected.prettyPrint();
}

// Observe one corner from two viewpoints and return what odometry reports for
// the second frame. `guess` may be null (no external guess).
Transform runTwoScanOdometry(
		const ParametersMap & parameters,
		const LaserScan & corner,
		const Transform & motion,
		const Transform & guess,
		OdometryInfo * info)
{
	std::unique_ptr<Odometry> odometry(Odometry::create(parameters));
	if(!odometry)
	{
		return Transform();
	}
	// The corner is fixed in the world; the second scan is the same points
	// expressed in the moved sensor frame.
	SensorData first  = makeScanData(corner, 1, 0.0);
	SensorData second = makeScanData(util3d::transformLaserScan(corner, motion.inverse()), 2, 0.1);

	OdometryInfo firstInfo;
	const Transform firstPose = odometry->process(first, &firstInfo);
	if(firstPose.isNull())
	{
		return Transform();
	}
	return guess.isNull() ? odometry->process(second, info)
	                      : odometry->process(second, guess, info);
}

}  // namespace

// 2D corner, no guess: ICP starts from identity and must find the motion from
// the geometry alone.
TEST_P(OdometryStrategyTest, Icp2DCornerRecoversMotionWithoutGuess)
{
	const Transform motion(0.10f, 0.06f, 0.0f, 0.0f, 0.0f, 0.05f);  // 12 cm / ~3 deg
	OdometryInfo info;
	const Transform pose = runTwoScanOdometry(
			icpOdometryParameters(GetParam(), /*force3DoF=*/true, /*guessMotion=*/false),
			makeCorner2D(), motion, Transform(), &info);
	if(pose.isNull() && info.reg.icpInliersRatio == 0.0f)
	{
		GTEST_SKIP() << "scan-matching odometry unavailable for this strategy";
	}
	expectPoseNear(pose, motion, 0.002f, 0.2f, "2D corner, no guess");
}

// 3D corner, no guess: same in 6DoF, so the two walls plus floor have to pin
// all three rotations as well.
TEST_P(OdometryStrategyTest, Icp3DCornerRecoversMotionWithoutGuess)
{
	const Transform motion(0.10f, 0.06f, 0.04f, 0.02f, 0.03f, 0.05f);
	OdometryInfo info;
	const Transform pose = runTwoScanOdometry(
			icpOdometryParameters(GetParam(), /*force3DoF=*/false, /*guessMotion=*/false),
			makeCorner3D(), motion, Transform(), &info);
	if(pose.isNull() && info.reg.icpInliersRatio == 0.0f)
	{
		GTEST_SKIP() << "scan-matching odometry unavailable for this strategy";
	}
	expectPoseNear(pose, motion, 0.002f, 0.2f, "3D corner, no guess");
}

// With a guess that is deliberately off, the result must be closer to the truth
// than the guess was -- i.e. ICP actually converged instead of returning the
// guess unchanged, which is the failure mode this asserts against.
TEST_P(OdometryStrategyTest, IcpConvergesFromOffsetGuess)
{
	const Transform motion(0.10f, 0.06f, 0.0f, 0.0f, 0.0f, 0.05f);
	// Guess is in the right neighbourhood but ~4 cm and ~1.7 deg away.
	const Transform guess(0.14f, 0.03f, 0.0f, 0.0f, 0.0f, 0.02f);
	const float guessError = guess.getDistance(motion);

	OdometryInfo info;
	const Transform pose = runTwoScanOdometry(
			icpOdometryParameters(GetParam(), /*force3DoF=*/true, /*guessMotion=*/false),
			makeCorner2D(), motion, guess, &info);
	if(pose.isNull() && info.reg.icpInliersRatio == 0.0f)
	{
		GTEST_SKIP() << "scan-matching odometry unavailable for this strategy";
	}
	ASSERT_FALSE(pose.isNull()) << "lost tracking despite a close guess";
	EXPECT_LT(pose.getDistance(motion), guessError)
			<< "result (" << pose.prettyPrint() << ") is no closer to the truth than the guess ("
			<< guess.prettyPrint() << "); ICP did not converge";
	expectPoseNear(pose, motion, 0.002f, 0.2f, "2D corner, offset guess");
}
