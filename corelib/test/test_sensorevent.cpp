#include <gtest/gtest.h>
#include <rtabmap/core/SensorEvent.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/SensorCaptureInfo.h>
#include <opencv2/core.hpp>

using namespace rtabmap;

// Constructor Tests

TEST(SensorEventTest, ConstructorFromImage)
{
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    int seq = 42;
    double stamp = 12345.678;
    std::string cameraName = "camera1";
    
    SensorEvent event(image, seq, stamp, cameraName);
    
    EXPECT_EQ(event.getCode(), SensorEvent::kCodeData);
    EXPECT_EQ(event.getClassName(), "SensorEvent");
    EXPECT_EQ(event.cameraName(), cameraName);
    EXPECT_EQ(event.data().id(), seq);
    EXPECT_DOUBLE_EQ(event.data().stamp(), stamp);
    EXPECT_FALSE(event.data().imageRaw().empty());
}

TEST(SensorEventTest, ConstructorFromImageDefaults)
{
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC1);
    
    SensorEvent event(image);
    
    EXPECT_EQ(event.getCode(), SensorEvent::kCodeData);
    EXPECT_EQ(event.data().id(), 0);
    EXPECT_DOUBLE_EQ(event.data().stamp(), 0.0);
    EXPECT_TRUE(event.cameraName().empty());
}

TEST(SensorEventTest, ConstructorEndOfStream)
{
    SensorEvent event;
    
    EXPECT_EQ(event.getCode(), SensorEvent::kCodeNoMoreImages);
    EXPECT_EQ(event.getClassName(), "SensorEvent");
    // End-of-stream event may have empty or default sensor data
}

TEST(SensorEventTest, ConstructorFromSensorData)
{
    SensorData data;
    data.setId(100);
    data.setStamp(54321.0);
    cv::Mat image = cv::Mat::ones(240, 320, CV_8UC3) * 128;
    data.setRGBDImage(image, cv::Mat(), CameraModel());
    
    SensorEvent event(data);
    
    EXPECT_EQ(event.getCode(), SensorEvent::kCodeData);
    EXPECT_EQ(event.data().id(), 100);
    EXPECT_DOUBLE_EQ(event.data().stamp(), 54321.0);
    EXPECT_TRUE(event.cameraName().empty());
}

TEST(SensorEventTest, ConstructorFromSensorDataWithCameraName)
{
    SensorData data;
    data.setId(200);
    std::string cameraName = "stereo_camera";
    
    SensorEvent event(data, cameraName);
    
    EXPECT_EQ(event.getCode(), SensorEvent::kCodeData);
    EXPECT_EQ(event.data().id(), 200);
    EXPECT_EQ(event.cameraName(), cameraName);
}

TEST(SensorEventTest, ConstructorFromSensorDataWithCaptureInfo)
{
    SensorData data;
    data.setId(300);
    data.setStamp(98765.432);
    
    SensorCaptureInfo captureInfo;
    captureInfo.cameraName = "rgbd_camera";
    captureInfo.id = 300;
    captureInfo.stamp = 98765.432;
    captureInfo.timeCapture = 0.01f;
    captureInfo.timeTotal = 0.05f;
    captureInfo.odomPose = Transform(1.0f, 2.0f, 3.0f, 0, 0, 0);
    
    SensorEvent event(data, captureInfo);
    
    EXPECT_EQ(event.getCode(), SensorEvent::kCodeData);
    EXPECT_EQ(event.data().id(), 300);
    EXPECT_EQ(event.cameraName(), "rgbd_camera");
    
    const auto& info = event.info();
    EXPECT_EQ(info.id, 300);
    EXPECT_DOUBLE_EQ(info.stamp, 98765.432);
    EXPECT_FLOAT_EQ(info.timeCapture, 0.01f);
    EXPECT_FLOAT_EQ(info.timeTotal, 0.05f);
    EXPECT_FLOAT_EQ(info.odomPose.x(), 1.0f);
}

// Event Code Tests

TEST(SensorEventTest, EventCodeData)
{
    cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC1);
    SensorEvent event(image);
    
    EXPECT_EQ(event.getCode(), SensorEvent::kCodeData);
}

TEST(SensorEventTest, EventCodeNoMoreImages)
{
    SensorEvent event;
    
    EXPECT_EQ(event.getCode(), SensorEvent::kCodeNoMoreImages);
}

// Class Name Tests

TEST(SensorEventTest, GetClassName)
{
    cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC1);
    SensorEvent event1(image);
    SensorEvent event2; // End of stream
    
    EXPECT_EQ(event1.getClassName(), "SensorEvent");
    EXPECT_EQ(event2.getClassName(), "SensorEvent");
}

// Data Access Tests

TEST(SensorEventTest, DataAccess)
{
    SensorData data;
    data.setId(500);
    data.setStamp(11111.0);
    cv::Mat image = cv::Mat::ones(100, 100, CV_8UC3) * 255;
    data.setRGBDImage(image, cv::Mat(), CameraModel());
    
    SensorEvent event(data);
    
    const auto& retrievedData = event.data();
    EXPECT_EQ(retrievedData.id(), 500);
    EXPECT_DOUBLE_EQ(retrievedData.stamp(), 11111.0);
    EXPECT_FALSE(retrievedData.imageRaw().empty());
    EXPECT_EQ(retrievedData.imageRaw().rows, 100);
    EXPECT_EQ(retrievedData.imageRaw().cols, 100);
}

// Camera Name Tests

TEST(SensorEventTest, CameraName)
{
    cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC1);
    
    // Without camera name
    SensorEvent event1(image);
    EXPECT_TRUE(event1.cameraName().empty());
    
    // With camera name
    SensorEvent event2(image, 0, 0.0, "my_camera");
    EXPECT_EQ(event2.cameraName(), "my_camera");
    
    // With SensorData and camera name
    SensorData data;
    SensorEvent event3(data, "another_camera");
    EXPECT_EQ(event3.cameraName(), "another_camera");
}

// Sensor Capture Info Tests

TEST(SensorEventTest, SensorCaptureInfo)
{
    SensorData data;
    data.setId(600);
    
    SensorCaptureInfo captureInfo;
    captureInfo.cameraName = "test_camera";
    captureInfo.id = 600;
    captureInfo.stamp = 22222.0;
    captureInfo.timeCapture = 0.005f;
    captureInfo.timeDeskewing = 0.001f;
    captureInfo.timeDisparity = 0.010f;
    captureInfo.timeTotal = 0.020f;
    captureInfo.odomPose = Transform(10.0f, 20.0f, 30.0f, 0, 0, 0);
    captureInfo.odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.1;
    
    SensorEvent event(data, captureInfo);
    
    const auto& info = event.info();
    EXPECT_EQ(info.cameraName, "test_camera");
    EXPECT_EQ(info.id, 600);
    EXPECT_DOUBLE_EQ(info.stamp, 22222.0);
    EXPECT_FLOAT_EQ(info.timeCapture, 0.005f);
    EXPECT_FLOAT_EQ(info.timeDeskewing, 0.001f);
    EXPECT_FLOAT_EQ(info.timeDisparity, 0.010f);
    EXPECT_FLOAT_EQ(info.timeTotal, 0.020f);
    EXPECT_FLOAT_EQ(info.odomPose.x(), 10.0f);
    EXPECT_FLOAT_EQ(info.odomPose.y(), 20.0f);
    EXPECT_FLOAT_EQ(info.odomPose.z(), 30.0f);
    EXPECT_FALSE(info.odomCovariance.empty());
    EXPECT_EQ(info.odomCovariance.rows, 6);
    EXPECT_EQ(info.odomCovariance.cols, 6);
}

TEST(SensorEventTest, SensorCaptureInfoDefaultValues)
{
    cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC1);
    SensorEvent event(image, 0, 0.0, "camera");
    
    const auto& info = event.info();
    EXPECT_EQ(info.id, 0);
    EXPECT_DOUBLE_EQ(info.stamp, 0.0);
    EXPECT_FLOAT_EQ(info.timeCapture, 0.0f);
    EXPECT_FLOAT_EQ(info.timeTotal, 0.0f);
    EXPECT_TRUE(info.odomPose.isNull());
    EXPECT_FALSE(info.odomCovariance.empty()); // Should be initialized to identity
}

// Comprehensive Tests

TEST(SensorEventTest, ComprehensiveUsage)
{
    // Create sensor data with multiple components
    SensorData data;
    data.setId(1000);
    data.setStamp(99999.999);
    
    cv::Mat rgb = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 640, CV_16UC1) * 1000;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    data.setRGBDImage(rgb, depth, model);
    
    // Create capture info with timing and odometry
    SensorCaptureInfo captureInfo;
    captureInfo.cameraName = "comprehensive_camera";
    captureInfo.id = 1000;
    captureInfo.stamp = 99999.999;
    captureInfo.timeCapture = 0.010f;
    captureInfo.timeDisparity = 0.020f;
    captureInfo.timeTotal = 0.050f;
    captureInfo.odomPose = Transform(1.5f, 2.5f, 3.5f, 0, 0, 0);
    
    SensorEvent event(data, captureInfo);
    
    // Verify event properties
    EXPECT_EQ(event.getCode(), SensorEvent::kCodeData);
    EXPECT_EQ(event.getClassName(), "SensorEvent");
    
    // Verify sensor data
    EXPECT_EQ(event.data().id(), 1000);
    EXPECT_DOUBLE_EQ(event.data().stamp(), 99999.999);
    EXPECT_FALSE(event.data().imageRaw().empty());
    EXPECT_FALSE(event.data().depthOrRightRaw().empty());
    
    // Verify capture info
    EXPECT_EQ(event.cameraName(), "comprehensive_camera");
    const auto& info = event.info();
    EXPECT_FLOAT_EQ(info.timeCapture, 0.010f);
    EXPECT_FLOAT_EQ(info.timeDisparity, 0.020f);
    EXPECT_FLOAT_EQ(info.timeTotal, 0.050f);
    EXPECT_FLOAT_EQ(info.odomPose.x(), 1.5f);
}

// Edge Cases

TEST(SensorEventTest, EmptyImage)
{
    cv::Mat emptyImage;
    SensorEvent event(emptyImage);
    
    EXPECT_EQ(event.getCode(), SensorEvent::kCodeData);
    EXPECT_TRUE(event.data().imageRaw().empty());
}

TEST(SensorEventTest, DifferentImageTypes)
{
    // Grayscale
    cv::Mat gray = cv::Mat::zeros(100, 100, CV_8UC1);
    SensorEvent event1(gray);
    EXPECT_FALSE(event1.data().imageRaw().empty());
    
    // Color
    cv::Mat color = cv::Mat::zeros(100, 100, CV_8UC3);
    SensorEvent event2(color);
    EXPECT_FALSE(event2.data().imageRaw().empty());
    
    // Float
    cv::Mat floatImg = cv::Mat::zeros(100, 100, CV_32FC1);
    SensorEvent event;
    EXPECT_THROW(event = SensorEvent(floatImg), UException);
}

// UEvent Inheritance Tests

TEST(SensorEventTest, UEventInheritance)
{
    cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC1);
    SensorEvent event(image);
    
    // Should be usable as UEvent
    UEvent* baseEvent = &event;
    EXPECT_EQ(baseEvent->getCode(), SensorEvent::kCodeData);
    EXPECT_EQ(baseEvent->getClassName(), "SensorEvent");
}

TEST(SensorEventTest, PolymorphicBehavior)
{
    cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC1);
    SensorEvent* event = new SensorEvent(image);
    
    UEvent* baseEvent = event;
    EXPECT_EQ(baseEvent->getCode(), SensorEvent::kCodeData);
    EXPECT_EQ(baseEvent->getClassName(), "SensorEvent");
    
    delete event;
}

