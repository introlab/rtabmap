#include <gtest/gtest.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Parameters.h>
#include <opencv2/core.hpp>

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
