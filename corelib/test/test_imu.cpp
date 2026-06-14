#include <gtest/gtest.h>
#include <rtabmap/core/IMU.h>
#include <opencv2/core.hpp>
#include <cmath>

using namespace rtabmap;

namespace {

static cv::Mat covariance3x3Diagonal(double d0, double d1, double d2)
{
	cv::Mat cov = cv::Mat::zeros(3, 3, CV_64FC1);
	cov.at<double>(0, 0) = d0;
	cov.at<double>(1, 1) = d1;
	cov.at<double>(2, 2) = d2;
	return cov;
}

static void expectVec3Near(const cv::Vec3d & a, const cv::Vec3d & b, double tol = 1e-5)
{
	EXPECT_NEAR(a[0], b[0], tol);
	EXPECT_NEAR(a[1], b[1], tol);
	EXPECT_NEAR(a[2], b[2], tol);
}

} // namespace

TEST(IMUTest, DefaultConstructorIsEmpty)
{
	const IMU imu;
	EXPECT_TRUE(imu.empty());
	EXPECT_TRUE(imu.localTransform().isNull());
	EXPECT_TRUE(imu.orientationCovariance().empty());
	EXPECT_TRUE(imu.angularVelocityCovariance().empty());
	EXPECT_TRUE(imu.linearAccelerationCovariance().empty());
}

TEST(IMUTest, IdentityLocalTransformIsNotEmpty)
{
	const IMU imu(
			cv::Vec3d(0.1, 0.2, 0.3),
			cv::Mat(),
			cv::Vec3d(1.0, 0.0, 0.0),
			cv::Mat(),
			Transform::getIdentity());
	EXPECT_FALSE(imu.empty());
	EXPECT_TRUE(imu.localTransform().isIdentity());
}

TEST(IMUTest, FullConstructorStoresFields)
{
	const cv::Vec4d orientation(0.1, 0.2, 0.3, 0.9);
	const cv::Vec3d angularVelocity(0.01, 0.02, 0.03);
	const cv::Vec3d linearAcceleration(9.8, 0.1, 0.2);
	const cv::Mat orientationCov = covariance3x3Diagonal(1.0, 1.0, 1.0);
	const cv::Mat angularCov = covariance3x3Diagonal(2.0, 2.0, 2.0);
	const cv::Mat linearCov = covariance3x3Diagonal(3.0, 3.0, 3.0);
	const Transform local(0.1f, 0.2f, 0.3f, 0.f, 0.f, 0.5f);

	const IMU imu(orientation, orientationCov, angularVelocity, angularCov, linearAcceleration, linearCov, local);

	EXPECT_FALSE(imu.empty());
	EXPECT_EQ(imu.orientation(), orientation);
	EXPECT_EQ(imu.angularVelocity(), angularVelocity);
	EXPECT_EQ(imu.linearAcceleration(), linearAcceleration);
	EXPECT_EQ(cv::norm(imu.orientationCovariance(), orientationCov, cv::NORM_INF), 0);
	EXPECT_EQ(cv::norm(imu.angularVelocityCovariance(), angularCov, cv::NORM_INF), 0);
	EXPECT_EQ(cv::norm(imu.linearAccelerationCovariance(), linearCov, cv::NORM_INF), 0);
	EXPECT_FLOAT_EQ(imu.localTransform().x(), 0.1f);
	EXPECT_FLOAT_EQ(imu.localTransform().theta(), 0.5f);
}

TEST(IMUTest, VelocityOnlyConstructorLeavesOrientationUnset)
{
	const IMU imu(cv::Vec3d(1, 2, 3), cv::Mat(), cv::Vec3d(4, 5, 6), cv::Mat(), Transform::getIdentity());

	EXPECT_EQ(imu.orientation(), cv::Vec4d());
	EXPECT_TRUE(imu.orientationCovariance().empty());
	EXPECT_EQ(imu.angularVelocity(), cv::Vec3d(1, 2, 3));
	EXPECT_EQ(imu.linearAcceleration(), cv::Vec3d(4, 5, 6));
}

TEST(IMUTest, ConvertToBaseFrameNoOpWhenLocalTransformNull)
{
	IMU imu;
	imu.convertToBaseFrame();
	EXPECT_TRUE(imu.empty());
}

TEST(IMUTest, ConvertToBaseFrameNoOpWhenRotationIdentity)
{
	IMU imu(cv::Vec3d(1, 0, 0), cv::Mat(), cv::Vec3d(0, 0, 9.8), cv::Mat(), Transform::getIdentity());
	const cv::Vec3d accelBefore = imu.linearAcceleration();
	const cv::Vec3d gyroBefore = imu.angularVelocity();

	imu.convertToBaseFrame();

	expectVec3Near(imu.linearAcceleration(), accelBefore);
	expectVec3Near(imu.angularVelocity(), gyroBefore);
	EXPECT_TRUE(imu.localTransform().isIdentity());
}

TEST(IMUTest, ConvertToBaseFrameRotatesLinearAcceleration)
{
	const double halfPi = CV_PI / 2.0;
	const Transform local(0.f, 0.f, 0.f, 0.f, 0.f, static_cast<float>(halfPi));
	IMU imu(cv::Vec3d(), cv::Mat(), cv::Vec3d(1.0, 0.0, 0.0), cv::Mat(), local);

	imu.convertToBaseFrame();

	expectVec3Near(imu.linearAcceleration(), cv::Vec3d(0.0, 1.0, 0.0), 1e-4);
	EXPECT_NEAR(imu.localTransform().theta(), 0.0f, 1e-5f);
	EXPECT_FLOAT_EQ(imu.localTransform().x(), 0.f);
}

TEST(IMUTest, ConvertToBaseFrameRotatesCovariance)
{
	const double halfPi = CV_PI / 2.0;
	const Transform local(0.f, 0.f, 0.f, 0.f, 0.f, static_cast<float>(halfPi));
	// Distinct diagonal: a 90° yaw swaps x/y variance (1 and 2), z (4) unchanged.
	const cv::Mat cov = covariance3x3Diagonal(1.0, 2.0, 4.0);
	IMU imu(cv::Vec3d(1, 0, 0), cov, cv::Vec3d(), cv::Mat(), local);

	imu.convertToBaseFrame();

	const cv::Mat & rotated = imu.angularVelocityCovariance();
	ASSERT_EQ(rotated.rows, 3);
	EXPECT_NE(rotated.at<double>(0, 0), 1.0);
	EXPECT_NE(rotated.at<double>(1, 1), 2.0);
	EXPECT_NEAR(rotated.at<double>(0, 0), 2.0, 1e-5);
	EXPECT_NEAR(rotated.at<double>(1, 1), 1.0, 1e-5);
	EXPECT_NEAR(rotated.at<double>(2, 2), 4.0, 1e-5);
	EXPECT_NEAR(rotated.at<double>(0, 1), 0.0, 1e-5);
	EXPECT_NEAR(rotated.at<double>(1, 0), 0.0, 1e-5);
}

TEST(IMUEventTest, DefaultConstructor)
{
	const IMUEvent event;
	EXPECT_EQ(event.getClassName(), "IMUEvent");
	EXPECT_DOUBLE_EQ(event.getStamp(), 0.0);
	EXPECT_TRUE(event.getData().empty());
}

TEST(IMUEventTest, StoresDataAndStamp)
{
	const IMU imu(cv::Vec3d(0.1, 0.2, 0.3), cv::Mat(), cv::Vec3d(1, 0, 0), cv::Mat(), Transform::getIdentity());
	const double stamp = 123.456;
	const IMUEvent event(imu, stamp);

	EXPECT_EQ(event.getClassName(), "IMUEvent");
	EXPECT_DOUBLE_EQ(event.getStamp(), stamp);
	EXPECT_FALSE(event.getData().empty());
	EXPECT_EQ(event.getData().angularVelocity(), imu.angularVelocity());
	EXPECT_EQ(event.getData().linearAcceleration(), imu.linearAcceleration());
}
