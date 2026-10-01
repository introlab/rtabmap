#include <gtest/gtest.h>
#include <rtabmap/core/Landmark.h>
#include <opencv2/core.hpp>

using namespace rtabmap;

namespace {

static cv::Mat covarianceDiagonal(
		double x,
		double y,
		double z,
		double roll,
		double pitch,
		double yaw)
{
	cv::Mat cov = cv::Mat::zeros(6, 6, CV_64FC1);
	cov.at<double>(0, 0) = x;
	cov.at<double>(1, 1) = y;
	cov.at<double>(2, 2) = z;
	cov.at<double>(3, 3) = roll;
	cov.at<double>(4, 4) = pitch;
	cov.at<double>(5, 5) = yaw;
	return cov;
}

} // namespace

TEST(LandmarkTest, DefaultConstructor)
{
	const Landmark landmark;
	EXPECT_EQ(landmark.id(), 0);
	EXPECT_FLOAT_EQ(landmark.size(), 0.0f);
	EXPECT_TRUE(landmark.pose().isNull());
	EXPECT_TRUE(landmark.covariance().empty());
}

TEST(LandmarkTest, ConstructorStoresFields)
{
	const cv::Mat covariance = covarianceDiagonal(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
	const Transform pose(1.0f, 2.0f, 3.0f, 0.f, 0.f, 0.5f);
	const Landmark landmark(10, 0.25f, pose, covariance);

	EXPECT_EQ(landmark.id(), 10);
	EXPECT_FLOAT_EQ(landmark.size(), 0.25f);
	EXPECT_FLOAT_EQ(landmark.pose().x(), 1.0f);
	EXPECT_FLOAT_EQ(landmark.pose().y(), 2.0f);
	EXPECT_FLOAT_EQ(landmark.pose().z(), 3.0f);
	EXPECT_FLOAT_EQ(landmark.pose().theta(), 0.5f);
	EXPECT_EQ(landmark.covariance().rows, 6);
	EXPECT_EQ(landmark.covariance().cols, 6);
	EXPECT_EQ(landmark.covariance().type(), CV_64FC1);
	EXPECT_DOUBLE_EQ(landmark.covariance().at<double>(0, 0), 0.1);
	EXPECT_DOUBLE_EQ(landmark.covariance().at<double>(5, 5), 0.6);
}

TEST(LandmarkTest, DeprecatedConstructorUsesZeroSize)
{
	const cv::Mat covariance = covarianceDiagonal(1, 1, 1, 9999, 9999, 9999);
	const Transform pose(0.5f, 0.0f, 0.0f, 0.f, 0.f, 0.f);
	const Landmark landmark(5, pose, covariance);

	EXPECT_EQ(landmark.id(), 5);
	EXPECT_FLOAT_EQ(landmark.size(), 0.0f);
	EXPECT_FLOAT_EQ(landmark.pose().x(), 0.5f);
}

TEST(LandmarkTest, AngularCovarianceCanUseLargeValueForUnknown)
{
	const cv::Mat covariance = covarianceDiagonal(0.01, 0.01, 0.01, 9999, 9999, 9999);
	const Landmark landmark(
			1,
			0.0f,
			Transform(0, 0, 0, 0, 0, 0),
			covariance);

	EXPECT_DOUBLE_EQ(landmark.covariance().at<double>(3, 3), 9999.0);
	EXPECT_DOUBLE_EQ(landmark.covariance().at<double>(5, 5), 9999.0);
}

TEST(LandmarkTest, LandmarksMapKeyedById)
{
	Landmarks landmarks;
	landmarks.insert(std::make_pair(
			1,
			Landmark(1, 0.1f, Transform(1, 0, 0, 0, 0, 0), covarianceDiagonal(1, 1, 1, 1, 1, 1))));
	landmarks.insert(std::make_pair(
			2,
			Landmark(2, 0.2f, Transform(2, 0, 0, 0, 0, 0), covarianceDiagonal(2, 2, 2, 2, 2, 2))));

	ASSERT_EQ(landmarks.size(), 2u);
	EXPECT_EQ(landmarks.at(1).id(), 1);
	EXPECT_FLOAT_EQ(landmarks.at(2).size(), 0.2f);
	EXPECT_DOUBLE_EQ(landmarks.at(2).covariance().at<double>(0, 0), 2.0);
}
