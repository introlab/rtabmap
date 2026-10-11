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
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/utilite/UStl.h>
#include <cmath>
#include <limits>
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

// A sweep that arrives far short of the points it should have cannot be
// registered against: Icp/CorrespondenceRatio is measured against a full sweep,
// so even matching every point it does have would leave it under the ratio.
// F2M refuses such a scan as the first keyframe rather than starting a map that
// nothing can be matched to. maxPoints is what says how big a full sweep is, so
// the check is only possible when a driver reported it.
TEST(OdometryTest, RefusesAFirstScanTooSmallForTheCorrespondenceRatio)
{
	const ParametersMap parameters =
			icpOdometryParameters(Odometry::kTypeF2M, /*force3DoF=*/false, /*guessMotion=*/false);
	const LaserScan corner = makeCorner3D();   // 1200 points

	// The same points three ways. First as a twentieth of the sweep they should be,
	// which no registration could reach the 0.1 ratio against.
	const LaserScan tooFewPoints(
			corner.data(), 20*(int)corner.size(), /*maxRange=*/0.0f, corner.format());
	std::unique_ptr<Odometry> refusing(Odometry::create(parameters));
	ASSERT_TRUE(refusing.get() != 0);
	SensorData refusingData = makeScanData(tooFewPoints, 1, 0.0);
	EXPECT_TRUE(refusing->process(refusingData).isNull())
			<< "a map was started on a scan no later one could be registered to";

	// Then as everything a sweep has, which is what a full one looks like.
	const LaserScan wholeSweep(
			corner.data(), (int)corner.size(), /*maxRange=*/0.0f, corner.format());
	std::unique_ptr<Odometry> accepting(Odometry::create(parameters));
	ASSERT_TRUE(accepting.get() != 0);
	SensorData acceptingData = makeScanData(wholeSweep, 1, 0.0);
	EXPECT_FALSE(accepting->process(acceptingData).isNull())
			<< "a full sweep was refused as the first keyframe";

	// And last as makeCorner3D() builds it, with maxPoints left at 0: a driver that
	// never said how big a sweep is, so there is no ratio to fall under and the check
	// does not apply.
	std::unique_ptr<Odometry> unchecked(Odometry::create(parameters));
	ASSERT_TRUE(unchecked.get() != 0);
	SensorData uncheckedData = makeScanData(corner, 1, 0.0);
	EXPECT_FALSE(unchecked->process(uncheckedData).isNull())
			<< "a scan of unknown sweep size was refused";
}

// ---------------------------------------------------------------------------
// Lidar deskewing with an IMU (Odom/Deskewing): a sensor moving in a box-shaped room,
// whose scans are generated point by point from where the sensor was at each point's
// time, as a spinning lidar measures them.
// ---------------------------------------------------------------------------

namespace {

const Eigen::Vector3d kRoomMin(-6.0, -4.0, -1.5);
const Eigen::Vector3d kRoomMax(7.0, 5.0, 2.5);
const double kSweep = 0.1;     // s, first column to last
const int kRings = 16;
const int kColumns = 512;
const double kGravity = 9.80665;

/// Pose of the sensor at time t, and its acceleration (world frame).
struct Trajectory
{
	double yaw = 0.0;              // rad, heading at t=0
	double yawRate = 0.0;          // rad/s
	double acceleration = 0.0;     // m/s^2 along x, from rest at t=0
	Transform pose(double t) const
	{
		return Transform(float(0.5*acceleration*t*t), 0, 0, 0, 0, float(yaw + yawRate*t));
	}
	Eigen::Vector3d linearAcceleration() const { return Eigen::Vector3d(acceleration, 0, 0); }
};

/// The scan of a sweep starting at @p stamp: organized (rings x columns), time on columns.
LaserScan makeSweep(const Trajectory & trajectory, double stamp)
{
	cv::Mat data(kRings, kColumns, CV_32FC(5));
	for(int u=0; u<kColumns; ++u)
	{
		const double dt = kSweep * double(u) / double(kColumns-1);
		const Transform pose = trajectory.pose(stamp + dt);
		const Eigen::Matrix3d rotation = pose.toEigen3d().linear();
		const Eigen::Vector3d origin(pose.x(), pose.y(), pose.z());
		const double azimuth = 2.0*M_PI*double(u)/double(kColumns);
		for(int v=0; v<kRings; ++v)
		{
			const double elevation = (-15.0 + 30.0*double(v)/double(kRings-1)) * M_PI / 180.0;
			const Eigen::Vector3d direction(std::cos(elevation)*std::cos(azimuth), std::cos(elevation)*std::sin(azimuth), std::sin(elevation));
			const Eigen::Vector3d world = rotation * direction;
			// Distance to the wall the ray hits
			double range = std::numeric_limits<double>::max();
			for(int i=0; i<3; ++i)
			{
				if(world[i] > 1e-9) range = std::min(range, (kRoomMax[i] - origin[i]) / world[i]);
				else if(world[i] < -1e-9) range = std::min(range, (kRoomMin[i] - origin[i]) / world[i]);
			}
			float * p = data.ptr<float>(v, u);
			p[0] = float(range*direction.x());
			p[1] = float(range*direction.y());
			p[2] = float(range*direction.z());
			p[3] = 1.0f;
			p[4] = float(dt);
		}
	}
	return LaserScan(data, kRings*kColumns, 0.0f, LaserScan::kXYZIT, Transform::getIdentity());
}

/// What the IMU, at the sensor's origin, measures at time t.
IMU makeImu(const Trajectory & trajectory, double t)
{
	const Eigen::Quaterniond q = trajectory.pose(t).getQuaterniond();
	const Eigen::Vector3d f = q.inverse() * (trajectory.linearAcceleration() + Eigen::Vector3d(0, 0, kGravity));
	return IMU(cv::Vec4d(q.x(), q.y(), q.z(), q.w()), cv::Mat::eye(3,3,CV_64FC1),
			cv::Vec3d(0, 0, trajectory.yawRate), cv::Mat::eye(3,3,CV_64FC1),
			cv::Vec3d(f.x(), f.y(), f.z()), cv::Mat::eye(3,3,CV_64FC1),
			Transform::getIdentity());
}

/// RMS distance (m) of the scan's points to the nearest wall, with the sensor at @p pose.
double wallDistance(const LaserScan & scan, const Transform & pose)
{
	double sum = 0.0;
	int n = 0;
	for(int i=0; i<scan.size(); ++i)
	{
		const float * p = scan.data().ptr<float>(0, i);
		const Transform world = pose * Transform(p[0], p[1], p[2], 0, 0, 0);
		const Eigen::Vector3d w(world.x(), world.y(), world.z());
		double d = std::numeric_limits<double>::max();
		for(int k=0; k<3; ++k)
		{
			d = std::min(d, std::fabs(w[k] - kRoomMin[k]));
			d = std::min(d, std::fabs(w[k] - kRoomMax[k]));
		}
		sum += d*d;
		++n;
	}
	return n?std::sqrt(sum/n):0.0;
}

/**
 * Feeds @p frames sweeps, one every 0.1 s from t=0.1, with the IMU at 200 Hz up to the end
 * of each sweep (as OdometryThread does), and returns the RMS wall distance of the last
 * sweep as odometry left it in the frame (deskewed or not).
 */
double deskewedWallDistance(const Trajectory & trajectory, int frames, const ParametersMap & extra)
{
	ParametersMap parameters;
	parameters.insert(ParametersPair(Parameters::kRegStrategy(), "1"));
	parameters.insert(ParametersPair(Parameters::kIcpVoxelSize(), "0"));
	parameters.insert(ParametersPair(Parameters::kIcpPointToPlane(), "true"));
	parameters.insert(ParametersPair(Parameters::kIcpPointToPlaneK(), "10"));
	parameters.insert(ParametersPair(Parameters::kIcpMaxTranslation(), "0"));
	parameters.insert(ParametersPair(Parameters::kIcpMaxCorrespondenceDistance(), "0.5"));
	for(const ParametersPair & p : extra) uInsert(parameters, p);
	std::unique_ptr<Odometry> odometry(Odometry::create(parameters));

	double imuStamp = 0.0;
	double distance = -1.0;
	for(int i=1; i<=frames; ++i)
	{
		const double stamp = 0.1*i;
		for(; imuStamp <= stamp + kSweep + 0.005; imuStamp += 0.005)
		{
			SensorData imu(makeImu(trajectory, imuStamp), 0, imuStamp);
			odometry->process(imu);
		}
		SensorData data(makeSweep(trajectory, stamp), cv::Mat(), cv::Mat(), CameraModel(), i, stamp);
		OdometryInfo info;
		const Transform pose = odometry->process(data, &info);
		EXPECT_FALSE(pose.isNull()) << "frame " << i << " not registered";
		distance = wallDistance(data.laserScanRaw(), trajectory.pose(stamp));
	}
	return distance;
}

}  // namespace

TEST(OdometryTest, DeskewsWithTheImuOrientation)
{
	// Turning at 2 rad/s: 11 degrees over a sweep, so the far walls are smeared by about a
	// metre. On the first frame, before any pose, only the IMU orientation is used.
	Trajectory turning;
	turning.yawRate = 2.0;
	ParametersMap off;
	off.insert(ParametersPair(Parameters::kOdomDeskewing(), "false"));
	const double skewed = deskewedWallDistance(turning, 1, off);
	const double deskewed = deskewedWallDistance(turning, 1, ParametersMap());
	EXPECT_GT(skewed, 0.1) << "the scan should be smeared without deskewing";
	EXPECT_LT(deskewed, 0.01) << "the points should be back on the walls";
}

TEST(OdometryTest, DeskewsWithTheImuAccelerationOverTheSmoothingDelay)
{
	// Accelerating at 2 m/s^2 from rest. With Odom/GuessSmoothingDelay, the velocity is
	// carried to each frame with the IMU acceleration, and the prediction integrates it
	// over the sweep. At 0, the acceleration is not used: the velocity of the last
	// interval lags behind, and the sweep is still skewed.
	Trajectory accelerating;
	accelerating.acceleration = 2.0;
	// Not facing exactly along x: an IMU orientation whose x, y and z are all zero (the
	// identity) is read by RTAB-Map's odometry as "not set", and the IMU ignored.
	accelerating.yaw = 0.3;
	ParametersMap noDelay;
	noDelay.insert(ParametersPair(Parameters::kOdomGuessSmoothingDelay(), "0"));
	ParametersMap delay;
	delay.insert(ParametersPair(Parameters::kOdomGuessSmoothingDelay(), "0.5"));
	const double withoutAcceleration = deskewedWallDistance(accelerating, 12, noDelay);
	const double withAcceleration = deskewedWallDistance(accelerating, 12, delay);
	EXPECT_LT(withAcceleration, 0.005) << "the points should be back on the walls";
	EXPECT_GT(withoutAcceleration, withAcceleration * 3.0);
}

TEST(OdometryTest, InitialOrientationIsTheImuOrientationAtTheFirstFrame)
{
	// IMU received for 2 s before the first frame, while the sensor turns at 1 rad/s: the
	// first frame must start from the orientation at its own stamp, not at the first IMU
	// sample (2 rad earlier).
	Trajectory turning;
	turning.yaw = 0.3;
	turning.yawRate = 1.0;
	ParametersMap parameters;
	parameters.insert(ParametersPair(Parameters::kRegStrategy(), "1"));
	parameters.insert(ParametersPair(Parameters::kIcpVoxelSize(), "0"));
	std::unique_ptr<Odometry> odometry(Odometry::create(parameters));
	const double stamp = 2.0;
	for(double t = 0.0; t <= stamp + kSweep + 0.005; t += 0.005)
	{
		SensorData imu(makeImu(turning, t), 0, t);
		odometry->process(imu);
	}
	SensorData data(makeSweep(turning, stamp), cv::Mat(), cv::Mat(), CameraModel(), 1, stamp);
	OdometryInfo info;
	const Transform pose = odometry->process(data, &info);
	ASSERT_FALSE(pose.isNull());
	// The IMU was fed up to the end of the sweep (as OdometryThread does), but the orientation
	// is interpolated at the frame's stamp
	EXPECT_NEAR(pose.theta(), turning.yaw + turning.yawRate*stamp, 0.002);

	// An initial pose given with a rotation is kept
	std::unique_ptr<Odometry> given(Odometry::create(parameters));
	given->reset(Transform(0, 0, 0, 0, 0, 1.0f));
	for(double t = 0.0; t <= stamp; t += 0.005)
	{
		SensorData imu(makeImu(turning, t), 0, t);
		given->process(imu);
	}
	EXPECT_NEAR(given->getPose().theta(), 1.0, 1e-6);
}

TEST(OdometryTest, ScanMapRealignsWithGravity)
{
	// Odometry starts with a roll 5 degrees off (an initial pose given with a rotation is
	// not overridden by the IMU), while the IMU says the sensor is level. Without gravity in
	// the local scan map, the error stays; with it, the map's keyframes are pulled toward the
	// IMU's gravity and the odometry's roll converges.
	if(!Optimizer::isAvailable(Optimizer::kTypeG2O) && !Optimizer::isAvailable(Optimizer::kTypeGTSAM))
	{
		GTEST_SKIP() << "Requires g2o or GTSAM";
	}
	Trajectory moving;
	moving.yaw = 0.3;
	moving.acceleration = 1.0;
	const double initialRoll = 5.0 * M_PI / 180.0;
	auto finalRoll = [&](bool scanGravity)
	{
		ParametersMap parameters;
		parameters.insert(ParametersPair(Parameters::kRegStrategy(), "1"));
		parameters.insert(ParametersPair(Parameters::kIcpVoxelSize(), "0.1"));
		parameters.insert(ParametersPair(Parameters::kIcpPointToPlane(), "true"));
		parameters.insert(ParametersPair(Parameters::kIcpPointToPlaneK(), "10"));
		parameters.insert(ParametersPair(Parameters::kIcpMaxTranslation(), "0"));
		parameters.insert(ParametersPair(Parameters::kIcpMaxCorrespondenceDistance(), "0.5"));
		parameters.insert(ParametersPair(Parameters::kOdomScanKeyFrameThr(), "0.95"));
		parameters.insert(ParametersPair(Parameters::kOdomF2MScanGravity(), scanGravity?"true":"false"));
		// Gravity links with Optimizer/GravitySigma's default (0.3 rad)
		std::unique_ptr<Odometry> odometry(Odometry::create(parameters));
		odometry->reset(Transform(0, 0, 0, float(initialRoll), 0, float(moving.yaw)));
		double imuStamp = 0.0;
		Transform pose;
		for(int i=1; i<=30; ++i)
		{
			const double stamp = 0.1*i;
			for(; imuStamp <= stamp + kSweep + 0.005; imuStamp += 0.005)
			{
				SensorData imu(makeImu(moving, imuStamp), 0, imuStamp);
				odometry->process(imu);
			}
			SensorData data(makeSweep(moving, stamp), cv::Mat(), cv::Mat(), CameraModel(), i, stamp);
			OdometryInfo info;
			pose = odometry->process(data, &info);
			EXPECT_FALSE(pose.isNull()) << "frame " << i << " not registered (scan gravity " << scanGravity << ")";
		}
		float roll, pitch, yaw;
		pose.getEulerAngles(roll, pitch, yaw);
		return std::fabs(roll);
	};
	const double withoutGravity = finalRoll(false);
	const double withGravity = finalRoll(true);
	EXPECT_NEAR(withoutGravity, initialRoll, 0.01) << "without gravity, the initial error should stay";
	EXPECT_LT(withGravity, initialRoll * 0.3) << "with gravity, the roll should converge to the IMU's";
}
