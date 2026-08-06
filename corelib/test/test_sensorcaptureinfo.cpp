#include <gtest/gtest.h>
#include <rtabmap/core/SensorCaptureInfo.h>
#include <rtabmap/core/Transform.h>
#include <opencv2/core.hpp>

using namespace rtabmap;

// Constructor Tests

TEST(SensorCaptureInfoTest, DefaultConstructor)
{
    SensorCaptureInfo info;
    
    // Basic fields
    EXPECT_TRUE(info.cameraName.empty());
    EXPECT_EQ(info.id, 0);
    EXPECT_DOUBLE_EQ(info.stamp, 0.0);
    
    // Timing fields (all should be 0.0f)
    EXPECT_FLOAT_EQ(info.timeCapture, 0.0f);
    EXPECT_FLOAT_EQ(info.timeDeskewing, 0.0f);
    EXPECT_FLOAT_EQ(info.timeDisparity, 0.0f);
    EXPECT_FLOAT_EQ(info.timeMirroring, 0.0f);
    EXPECT_FLOAT_EQ(info.timeStereoExposureCompensation, 0.0f);
    EXPECT_FLOAT_EQ(info.timeImageDecimation, 0.0f);
    EXPECT_FLOAT_EQ(info.timeHistogramEqualization, 0.0f);
    EXPECT_FLOAT_EQ(info.timeScanFromDepth, 0.0f);
    EXPECT_FLOAT_EQ(info.timeUndistortDepth, 0.0f);
    EXPECT_FLOAT_EQ(info.timeBilateralFiltering, 0.0f);
    EXPECT_FLOAT_EQ(info.timeTotal, 0.0f);
    
    // Odometry fields
    EXPECT_TRUE(info.odomPose.isNull());
    EXPECT_FALSE(info.odomCovariance.empty());
    EXPECT_EQ(info.odomCovariance.rows, 6);
    EXPECT_EQ(info.odomCovariance.cols, 6);
    EXPECT_EQ(info.odomCovariance.type(), CV_64FC1);
    // Should be identity matrix
    cv::Mat identity = cv::Mat::eye(6, 6, CV_64FC1);
    EXPECT_TRUE(cv::countNonZero(info.odomCovariance != identity) == 0);
    EXPECT_TRUE(info.odomVelocity.empty());
}

// Comprehensive Tests

TEST(SensorCaptureInfoTest, ComprehensiveUsage)
{
    SensorCaptureInfo info;
    
    // Set all fields
    info.cameraName = "comprehensive_camera";
    info.id = 1000;
    info.stamp = 123456.789;
    
    info.timeCapture = 0.010f;
    info.timeDeskewing = 0.005f;
    info.timeDisparity = 0.020f;
    info.timeMirroring = 0.001f;
    info.timeStereoExposureCompensation = 0.003f;
    info.timeImageDecimation = 0.002f;
    info.timeHistogramEqualization = 0.004f;
    info.timeScanFromDepth = 0.015f;
    info.timeUndistortDepth = 0.008f;
    info.timeBilateralFiltering = 0.012f;
    info.timeTotal = 0.080f;
    
    Transform pose(10.0f, 20.0f, 30.0f, 0.1f, 0.2f, 0.3f);
    info.odomPose = pose;
    
    cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.1;
    info.odomCovariance = covariance;
    
    info.odomVelocity = {1.5f, 2.5f, 3.5f, 0.15f, 0.25f, 0.35f};
    
    // Verify all fields
    EXPECT_EQ(info.cameraName, "comprehensive_camera");
    EXPECT_EQ(info.id, 1000);
    EXPECT_DOUBLE_EQ(info.stamp, 123456.789);
    
    EXPECT_FLOAT_EQ(info.timeTotal, 0.080f);
    
    EXPECT_FLOAT_EQ(info.odomPose.x(), 10.0f);
    EXPECT_FALSE(info.odomCovariance.empty());
    EXPECT_EQ(info.odomVelocity.size(), 6u);
    EXPECT_FLOAT_EQ(info.odomVelocity[0], 1.5f);
}

TEST(SensorCaptureInfoTest, CopyAssignment)
{
    SensorCaptureInfo info1;
    info1.cameraName = "camera1";
    info1.id = 100;
    info1.stamp = 123.456;
    info1.timeCapture = 0.01f;
    info1.timeTotal = 0.05f;
    info1.odomPose = Transform(1.0f, 2.0f, 3.0f, 0, 0, 0);
    info1.odomVelocity = {1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f};
    
    SensorCaptureInfo info2;
    info2 = info1;
    
    EXPECT_EQ(info2.cameraName, info1.cameraName);
    EXPECT_EQ(info2.id, info1.id);
    EXPECT_DOUBLE_EQ(info2.stamp, info1.stamp);
    EXPECT_FLOAT_EQ(info2.timeCapture, info1.timeCapture);
    EXPECT_FLOAT_EQ(info2.timeTotal, info1.timeTotal);
    EXPECT_FLOAT_EQ(info2.odomPose.x(), info1.odomPose.x());
    EXPECT_EQ(info2.odomVelocity.size(), info1.odomVelocity.size());
}

TEST(SensorCaptureInfoTest, CopyConstructor)
{
    SensorCaptureInfo info1;
    info1.cameraName = "camera1";
    info1.id = 200;
    info1.stamp = 456.789;
    info1.timeDisparity = 0.02f;
    info1.odomPose = Transform(5.0f, 6.0f, 7.0f, 0, 0, 0);
    
    SensorCaptureInfo info2(info1);
    
    EXPECT_EQ(info2.cameraName, info1.cameraName);
    EXPECT_EQ(info2.id, info1.id);
    EXPECT_DOUBLE_EQ(info2.stamp, info1.stamp);
    EXPECT_FLOAT_EQ(info2.timeDisparity, info1.timeDisparity);
    EXPECT_FLOAT_EQ(info2.odomPose.x(), info1.odomPose.x());
}

// Edge Cases

TEST(SensorCaptureInfoTest, NegativeTiming)
{
    SensorCaptureInfo info;
    
    // Timing can technically be negative (though unusual)
    info.timeCapture = -0.001f;
    EXPECT_FLOAT_EQ(info.timeCapture, -0.001f);
}

TEST(SensorCaptureInfoTest, VeryLargeTiming)
{
    SensorCaptureInfo info;
    
    info.timeTotal = 10.0f;
    EXPECT_FLOAT_EQ(info.timeTotal, 10.0f);
}

TEST(SensorCaptureInfoTest, EmptyCameraName)
{
    SensorCaptureInfo info;
    
    info.cameraName = "";
    EXPECT_TRUE(info.cameraName.empty());
    
    info.cameraName = "camera";
    EXPECT_FALSE(info.cameraName.empty());
    
    info.cameraName = "";
    EXPECT_TRUE(info.cameraName.empty());
}

TEST(SensorCaptureInfoTest, ZeroOdometryCovariance)
{
    SensorCaptureInfo info;
    
    cv::Mat zeroCov = cv::Mat::zeros(6, 6, CV_64FC1);
    info.odomCovariance = zeroCov;
    
    EXPECT_EQ(info.odomCovariance.rows, 6);
    EXPECT_EQ(info.odomCovariance.cols, 6);
    EXPECT_DOUBLE_EQ(info.odomCovariance.at<double>(0, 0), 0.0);
}

TEST(SensorCaptureInfoTest, EmptyOdometryVelocity)
{
    SensorCaptureInfo info;
    
    EXPECT_TRUE(info.odomVelocity.empty());
    
    info.odomVelocity = {1.0f};
    EXPECT_EQ(info.odomVelocity.size(), 1u);
    
    info.odomVelocity.clear();
    EXPECT_TRUE(info.odomVelocity.empty());
}

// Real-world Usage Scenarios

TEST(SensorCaptureInfoTest, StereoCaptureScenario)
{
    SensorCaptureInfo info;
    
    info.cameraName = "stereo_camera";
    info.id = 500;
    info.stamp = 1000.0;
    
    // Typical stereo processing times
    info.timeCapture = 0.010f;
    info.timeDisparity = 0.030f;
    info.timeBilateralFiltering = 0.015f;
    info.timeTotal = 0.055f;
    
    info.odomPose = Transform(0.5f, 0.0f, 0.0f, 0, 0, 0);
    info.odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
    
    EXPECT_EQ(info.cameraName, "stereo_camera");
    EXPECT_GE(info.timeTotal, info.timeCapture + info.timeDisparity);
}

TEST(SensorCaptureInfoTest, RGBDCaptureScenario)
{
    SensorCaptureInfo info;
    
    info.cameraName = "rgbd_camera";
    info.id = 600;
    info.stamp = 2000.0;
    
    // Typical RGB-D processing times
    info.timeCapture = 0.008f;
    info.timeUndistortDepth = 0.005f;
    info.timeScanFromDepth = 0.010f;
    info.timeTotal = 0.025f;
    
    info.odomPose = Transform(1.0f, 1.0f, 1.0f, 0, 0, 0.1f);
    info.odomVelocity = {0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.05f};
    
    EXPECT_EQ(info.cameraName, "rgbd_camera");
    EXPECT_FALSE(info.odomPose.isNull());
    EXPECT_EQ(info.odomVelocity.size(), 6u);
}

