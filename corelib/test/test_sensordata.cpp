#include <gtest/gtest.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/GPS.h>
#include <rtabmap/core/EnvSensor.h>
#include <rtabmap/core/Landmark.h>
#include <rtabmap/core/GlobalDescriptor.h>
#include <rtabmap/utilite/UException.h>
#include <opencv2/core.hpp>

using namespace rtabmap;

// Constructor Tests

TEST(SensorDataTest, DefaultConstructor)
{
    SensorData data;
    
    EXPECT_FALSE(data.isValid());
    EXPECT_EQ(data.id(), 0);
    EXPECT_DOUBLE_EQ(data.stamp(), 0.0);
    EXPECT_TRUE(data.imageRaw().empty());
    EXPECT_TRUE(data.imageCompressed().empty());
    EXPECT_TRUE(data.depthOrRightRaw().empty());
    EXPECT_TRUE(data.depthOrRightCompressed().empty());
    EXPECT_TRUE(data.cameraModels().empty());
    EXPECT_TRUE(data.stereoCameraModels().empty());
}

TEST(SensorDataTest, ConstructorAppearanceOnly)
{
    cv::Mat image = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    int id = 100;
    double stamp = 12345.678;
    
    SensorData data(image, id, stamp);
    
    EXPECT_TRUE(data.isValid());
    EXPECT_EQ(data.id(), id);
    EXPECT_DOUBLE_EQ(data.stamp(), stamp);
    EXPECT_FALSE(data.imageRaw().empty());
    EXPECT_EQ(data.imageRaw().rows, 480);
    EXPECT_EQ(data.imageRaw().cols, 640);
}

TEST(SensorDataTest, ConstructorMono)
{
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC1);
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    int id = 200;
    
    SensorData data(image, model, id);
    
    EXPECT_TRUE(data.isValid());
    EXPECT_EQ(data.id(), id);
    EXPECT_FALSE(data.cameraModels().empty());
    EXPECT_EQ(data.cameraModels().size(), 1u);
}

TEST(SensorDataTest, ConstructorRGBD)
{
    cv::Mat rgb = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 640, CV_16UC1) * 1000;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    int id = 300;
    
    SensorData data(rgb, depth, model, id);
    
    EXPECT_TRUE(data.isValid());
    EXPECT_EQ(data.id(), id);
    EXPECT_FALSE(data.imageRaw().empty());
    EXPECT_FALSE(data.depthOrRightRaw().empty());
    EXPECT_FALSE(data.cameraModels().empty());
}

TEST(SensorDataTest, ConstructorRGBDWithConfidence)
{
    cv::Mat rgb = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 640, CV_32FC1) * 1.5f;
    cv::Mat confidence = cv::Mat::ones(480, 640, CV_8UC1) * 80;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    
    SensorData data(rgb, depth, confidence, model);
    
    EXPECT_TRUE(data.isValid());
    EXPECT_FALSE(data.depthConfidenceRaw().empty());
    EXPECT_EQ(data.depthConfidenceRaw().type(), CV_8UC1);
}

TEST(SensorDataTest, ConstructorRGBDWithLaserScan)
{
    cv::Mat rgb = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 640, CV_16UC1) * 1000;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    LaserScan scan = LaserScan::backwardCompatibility(cv::Mat(1,100,CV_32FC2));
    
    SensorData data(scan, rgb, depth, model);
    
    EXPECT_TRUE(data.isValid());
    EXPECT_FALSE(data.laserScanRaw().isEmpty());
}

TEST(SensorDataTest, ConstructorMultiCameraRGBD)
{
    cv::Mat rgb = cv::Mat::ones(480, 1280, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 1280, CV_16UC1) * 1000;
    std::vector<CameraModel> models;
    models.push_back(CameraModel(525.0, 525.0, 320.0, 240.0));
    models.push_back(CameraModel(525.0, 525.0, 960.0, 240.0));
    
    SensorData data(rgb, depth, models);
    
    EXPECT_TRUE(data.isValid());
    EXPECT_EQ(data.cameraModels().size(), models.size());
}

TEST(SensorDataTest, ConstructorStereo)
{
    cv::Mat left = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(480, 640, CV_8UC1);
    StereoCameraModel model(525.0, 525.0, 320.0, 240.0, 0.12);
    
    SensorData data(left, right, model);
    
    EXPECT_TRUE(data.isValid());
    EXPECT_FALSE(data.imageRaw().empty());
    EXPECT_FALSE(data.depthOrRightRaw().empty());
    EXPECT_FALSE(data.stereoCameraModels().empty());
}

TEST(SensorDataTest, ConstructorMultiStereo)
{
    cv::Mat left = cv::Mat::zeros(480, 1280, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(480, 1280, CV_8UC1);
    std::vector<StereoCameraModel> models;
    models.push_back(StereoCameraModel(525.0, 525.0, 320.0, 240.0, 0.12));
    models.push_back(StereoCameraModel(525.0, 525.0, 320.0, 240.0, 0.12));
    SensorData data(left, right, models);
    
    EXPECT_TRUE(data.isValid());
    EXPECT_FALSE(data.imageRaw().empty());
    EXPECT_FALSE(data.depthOrRightRaw().empty());
    EXPECT_EQ(data.stereoCameraModels().size(), models.size());
}

TEST(SensorDataTest, ConstructorStereoWithLaserScan)
{
    cv::Mat left = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(480, 640, CV_8UC1);
    StereoCameraModel model(525.0, 525.0, 320.0, 240.0, 0.12);
    LaserScan scan = LaserScan::backwardCompatibility(cv::Mat(1,100,CV_32FC2));
    
    SensorData data(scan, left, right, model);
    
    EXPECT_TRUE(data.isValid());
    EXPECT_FALSE(data.laserScanRaw().isEmpty());
}

TEST(SensorDataTest, ConstructorIMUOnly)
{
    IMU imu(cv::Vec3d(), cv::Mat(), cv::Vec3d(), cv::Mat(), Transform::getIdentity());
    int id = 500;
    double stamp = 99999.0;
    
    SensorData data(imu, id, stamp);
    
    EXPECT_TRUE(data.isValid());
    EXPECT_EQ(data.id(), id);
    EXPECT_DOUBLE_EQ(data.stamp(), stamp);
    EXPECT_FALSE(data.imu().empty());
}

// isValid() Tests

TEST(SensorDataTest, IsValidWithId)
{
    SensorData data;
    EXPECT_FALSE(data.isValid());
    
    data.setId(100);
    EXPECT_TRUE(data.isValid());
}

TEST(SensorDataTest, IsValidWithStamp)
{
    SensorData data;
    EXPECT_FALSE(data.isValid());
    
    data.setStamp(12345.0);
    EXPECT_TRUE(data.isValid());
}

TEST(SensorDataTest, IsValidWithImage)
{
    SensorData data;
    cv::Mat image = cv::Mat::ones(100, 100, CV_8UC3);
    data.setRGBDImage(image, cv::Mat(), CameraModel());
    EXPECT_TRUE(data.isValid());
}

TEST(SensorDataTest, IsValidWithLaserScan)
{
    SensorData data;
    LaserScan scan = LaserScan::backwardCompatibility(cv::Mat(1,100,CV_32FC2));
    data.setLaserScan(scan);
    EXPECT_TRUE(data.isValid());
}

TEST(SensorDataTest, IsValidWithIMU)
{
    SensorData data;
    IMU imu(cv::Vec3d(), cv::Mat(), cv::Vec3d(), cv::Mat(), Transform::getIdentity());
    data.setIMU(imu);
    EXPECT_TRUE(data.isValid());
}

// ID and Stamp Tests

TEST(SensorDataTest, IdGetterSetter)
{
    SensorData data;
    EXPECT_EQ(data.id(), 0);
    
    data.setId(123);
    EXPECT_EQ(data.id(), 123);
    
    data.setId(0);
    EXPECT_EQ(data.id(), 0);
}

TEST(SensorDataTest, StampGetterSetter)
{
    SensorData data;
    EXPECT_DOUBLE_EQ(data.stamp(), 0.0);
    
    data.setStamp(12345.678);
    EXPECT_DOUBLE_EQ(data.stamp(), 12345.678);
    
    data.setStamp(0.0);
    EXPECT_DOUBLE_EQ(data.stamp(), 0.0);
}

// setRGBDImage Tests

TEST(SensorDataTest, SetRGBDImage)
{
    SensorData data;
    cv::Mat rgb = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 640, CV_16UC1) * 1000;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    
    data.setRGBDImage(rgb, depth, model);
    
    EXPECT_FALSE(data.imageRaw().empty());
    EXPECT_FALSE(data.depthOrRightRaw().empty());
    EXPECT_EQ(data.imageRaw().rows, 480);
    EXPECT_EQ(data.imageRaw().cols, 640);
    EXPECT_FALSE(data.cameraModels().empty());
}

TEST(SensorDataTest, SetRGBDImageWithConfidence)
{
    SensorData data;
    cv::Mat rgb = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 640, CV_32FC1) * 1.5f;
    cv::Mat confidence = cv::Mat::ones(480, 640, CV_8UC1) * 90;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    
    data.setRGBDImage(rgb, depth, confidence, model);
    
    EXPECT_FALSE(data.depthConfidenceRaw().empty());
    EXPECT_EQ(data.depthConfidenceRaw().type(), CV_8UC1);
}

TEST(SensorDataTest, SetRGBDImageMultiCamera)
{
    SensorData data;
    cv::Mat rgb = cv::Mat::ones(480, 1280, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 1280, CV_16UC1) * 1000;
    std::vector<CameraModel> models;
    models.push_back(CameraModel(525.0, 525.0, 320.0, 240.0));
    models.push_back(CameraModel(525.0, 525.0, 960.0, 240.0));
    
    data.setRGBDImage(rgb, depth, models);
    
    EXPECT_EQ(data.cameraModels().size(), 2u);
}

TEST(SensorDataTest, SetRGBDImageClearPreviousData)
{
    SensorData data;
    cv::Mat rgb1 = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    cv::Mat depth1 = cv::Mat::ones(480, 640, CV_16UC1) * 1000;
    CameraModel model1(525.0, 525.0, 320.0, 240.0);
    data.setRGBDImage(rgb1, depth1, model1);
    
    cv::Mat rgb2 = cv::Mat::ones(240, 320, CV_8UC3) * 255;
    cv::Mat depth2 = cv::Mat::ones(240, 320, CV_16UC1) * 2000;
    CameraModel model2(525.0, 525.0, 160.0, 120.0);
    data.setRGBDImage(rgb2, depth2, model2, true);
    
    EXPECT_EQ(data.imageRaw().rows, 240);
    EXPECT_EQ(data.imageRaw().cols, 320);
    EXPECT_EQ(data.cameraModels().size(), 1u);
}

// setStereoImage Tests

TEST(SensorDataTest, SetStereoImage)
{
    SensorData data;
    cv::Mat left = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(480, 640, CV_8UC1);
    StereoCameraModel model(525.0, 525.0, 320.0, 240.0, 0.12);
    
    data.setStereoImage(left, right, model);
    
    EXPECT_FALSE(data.imageRaw().empty());
    EXPECT_FALSE(data.depthOrRightRaw().empty());
    EXPECT_FALSE(data.stereoCameraModels().empty());
    EXPECT_TRUE(data.cameraModels().empty());
}

TEST(SensorDataTest, SetStereoImageMultiCamera)
{
    SensorData data;
    cv::Mat left = cv::Mat::zeros(480, 1280, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(480, 1280, CV_8UC1);
    std::vector<StereoCameraModel> models;
    models.push_back(StereoCameraModel(525.0, 525.0, 320.0, 240.0, 0.12));
    models.push_back(StereoCameraModel(525.0, 525.0, 960.0, 240.0, 0.12));
    
    data.setStereoImage(left, right, models);
    
    EXPECT_EQ(data.stereoCameraModels().size(), 2u);
}

// setLaserScan Tests

TEST(SensorDataTest, SetLaserScan)
{
    SensorData data;
    LaserScan scan = LaserScan::backwardCompatibility(cv::Mat(1,100,CV_32FC2));
    
    data.setLaserScan(scan);
    
    EXPECT_FALSE(data.laserScanRaw().isEmpty());
    EXPECT_EQ(data.laserScanRaw().size(), 100u);
}

// Camera Model Tests

TEST(SensorDataTest, SetCameraModel)
{
    SensorData data;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    
    data.setCameraModel(model);
    
    EXPECT_EQ(data.cameraModels().size(), 1u);
    EXPECT_DOUBLE_EQ(data.cameraModels()[0].fx(), 525.0);
}

TEST(SensorDataTest, SetCameraModels)
{
    SensorData data;
    std::vector<CameraModel> models;
    models.push_back(CameraModel(525.0, 525.0, 320.0, 240.0));
    models.push_back(CameraModel(600.0, 600.0, 400.0, 300.0));
    
    data.setCameraModels(models);
    
    EXPECT_EQ(data.cameraModels().size(), 2u);
}

TEST(SensorDataTest, SetStereoCameraModel)
{
    SensorData data;
    StereoCameraModel model(525.0, 525.0, 320.0, 240.0, 0.12);
    
    data.setStereoCameraModel(model);
    
    EXPECT_EQ(data.stereoCameraModels().size(), 1u);
    EXPECT_DOUBLE_EQ(data.stereoCameraModels()[0].baseline(), 0.12);
}

TEST(SensorDataTest, SetStereoCameraModels)
{
    SensorData data;
    std::vector<StereoCameraModel> models;
    models.push_back(StereoCameraModel(525.0, 525.0, 320.0, 240.0, 0.12));
    models.push_back(StereoCameraModel(600.0, 600.0, 400.0, 300.0, 0.15));
    
    data.setStereoCameraModels(models);
    
    EXPECT_EQ(data.stereoCameraModels().size(), 2u);
}

// Convenience Methods Tests

TEST(SensorDataTest, DepthRaw)
{
    SensorData data;
    cv::Mat depth = cv::Mat::ones(480, 640, CV_16UC1) * 1000;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    data.setRGBDImage(cv::Mat(), depth, model);
    
    EXPECT_TRUE(data.rightRaw().empty());
    cv::Mat extractedDepth = data.depthRaw();
    EXPECT_FALSE(extractedDepth.empty());
    EXPECT_EQ(extractedDepth.type(), CV_16UC1);
}

TEST(SensorDataTest, RightRaw)
{
    SensorData data;
    cv::Mat left = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat right = cv::Mat::ones(480, 640, CV_8UC1) * 128;
    StereoCameraModel model(525.0, 525.0, 320.0, 240.0, 0.12);
    data.setStereoImage(left, right, model);
    
    EXPECT_TRUE(data.depthRaw().empty());
    cv::Mat extractedRight = data.rightRaw();
    EXPECT_FALSE(extractedRight.empty());
    EXPECT_EQ(extractedRight.type(), CV_8UC1);
}

// Features Tests

TEST(SensorDataTest, SetFeatures)
{
    SensorData data;
    std::vector<cv::KeyPoint> keypoints;
    keypoints.push_back(cv::KeyPoint(100, 200, 5.0f));
    keypoints.push_back(cv::KeyPoint(300, 400, 5.0f));
    
    std::vector<cv::Point3f> keypoints3D;
    keypoints3D.push_back(cv::Point3f(1.0f, 2.0f, 3.0f));
    keypoints3D.push_back(cv::Point3f(4.0f, 5.0f, 6.0f));
    
    cv::Mat descriptors = cv::Mat::ones(2, 128, CV_32F);
    
    data.setFeatures(keypoints, keypoints3D, descriptors);
    
    EXPECT_EQ(data.keypoints().size(), 2u);
    EXPECT_EQ(data.keypoints3D().size(), 2u);
    EXPECT_FALSE(data.descriptors().empty());
    EXPECT_EQ(data.descriptors().rows, 2);
}

// Global Descriptors Tests

TEST(SensorDataTest, AddGlobalDescriptor)
{
    SensorData data;
    GlobalDescriptor desc(0, cv::Mat::ones(1, 100, CV_32F));
    
    data.addGlobalDescriptor(desc);
    
    EXPECT_EQ(data.globalDescriptors().size(), 1u);
}

TEST(SensorDataTest, SetGlobalDescriptors)
{
    SensorData data;
    std::vector<GlobalDescriptor> descriptors;
    descriptors.push_back(GlobalDescriptor(0, cv::Mat::ones(1, 100, CV_32F)));
    descriptors.push_back(GlobalDescriptor(0, cv::Mat::ones(1, 200, CV_32F)));
    
    data.setGlobalDescriptors(descriptors);
    
    EXPECT_EQ(data.globalDescriptors().size(), 2u);
}

TEST(SensorDataTest, ClearGlobalDescriptors)
{
    SensorData data;
    GlobalDescriptor desc(0, cv::Mat::ones(1, 100, CV_32F));
    data.addGlobalDescriptor(desc);
    EXPECT_EQ(data.globalDescriptors().size(), 1u);
    
    data.clearGlobalDescriptors();
    EXPECT_TRUE(data.globalDescriptors().empty());
}

// Pose Tests

TEST(SensorDataTest, SetGroundTruth)
{
    SensorData data;
    Transform pose(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f);
    
    data.setGroundTruth(pose);
    
    EXPECT_FALSE(data.groundTruth().isNull());
    EXPECT_FLOAT_EQ(data.groundTruth().x(), 1.0f);
}

TEST(SensorDataTest, SetGlobalPose)
{
    SensorData data;
    Transform pose(10.0f, 20.0f, 30.0f, 0.1f, 0.2f, 0.3f);
    cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.1;
    
    data.setGlobalPose(pose, covariance);
    
    EXPECT_FALSE(data.globalPose().isNull());
    EXPECT_FALSE(data.globalPoseCovariance().empty());
    EXPECT_EQ(data.globalPoseCovariance().rows, 6);
    EXPECT_EQ(data.globalPoseCovariance().cols, 6);
}

// GPS Tests

TEST(SensorDataTest, SetGPS)
{
    double stamp = 1111.11;
    double latitude = -73.0;
    GPS gps(stamp, 45.0, latitude, 100.0, 5.0, 0.0);
    
    SensorData data;
    data.setGPS(gps);
    
    EXPECT_DOUBLE_EQ(data.gps().stamp(), stamp);
    EXPECT_DOUBLE_EQ(data.gps().latitude(), latitude);
}

// IMU Tests

TEST(SensorDataTest, SetIMU)
{
    SensorData data;
    IMU imu;
    
    data.setIMU(imu);
    EXPECT_TRUE(data.imu().empty());

    imu = IMU(cv::Vec3d(), cv::Mat(), cv::Vec3d(), cv::Mat(), Transform::getIdentity());
    data.setIMU(imu);
    EXPECT_FALSE(data.imu().empty());
}

// Environmental Sensors Tests

TEST(SensorDataTest, SetEnvSensors)
{
    SensorData data;
    EnvSensors sensors;
    sensors.insert(std::make_pair(EnvSensor::kAmbientTemperature, EnvSensor(EnvSensor::kAmbientTemperature, 25.0f, 0.0)));
    
    data.setEnvSensors(sensors);
    
    EXPECT_FALSE(data.envSensors().empty());
}

TEST(SensorDataTest, AddEnvSensor)
{
    SensorData data;
    EnvSensor sensor(EnvSensor::kAmbientRelativeHumidity, 60.0f, 0.0);
    
    data.addEnvSensor(sensor);
    
    EXPECT_FALSE(data.envSensors().empty());
    EXPECT_EQ(data.envSensors().count(EnvSensor::kAmbientRelativeHumidity), 1u);
}

// Landmarks Tests

TEST(SensorDataTest, SetLandmarks)
{
    SensorData data;
    Landmarks landmarks;
    landmarks.insert(std::make_pair(1, Landmark(1, 0, Transform(1.0f, 2.0f, 3.0f, 0, 0, 0), cv::Mat::eye(6,6,CV_64FC1))));
    
    data.setLandmarks(landmarks);
    
    EXPECT_FALSE(data.landmarks().empty());
    EXPECT_EQ(data.landmarks().count(1), 1u);
}

// User Data Tests

TEST(SensorDataTest, SetUserData)
{
    SensorData data;
    cv::Mat userData = cv::Mat::ones(100, 100, CV_8UC1) * 128;
    
    data.setUserData(userData);
    
    EXPECT_FALSE(data.userDataRaw().empty());
    EXPECT_EQ(data.userDataRaw().rows, 100);
    EXPECT_EQ(data.userDataRaw().cols, 100);
}

// Occupancy Grid Tests

TEST(SensorDataTest, SetOccupancyGrid)
{
    SensorData data;
    // Use supported format: CV_32FC2 (2 channels of float)
    cv::Mat ground = cv::Mat::ones(1, 100, CV_32FC2);
    cv::Mat obstacles = cv::Mat::zeros(1, 100, CV_32FC2);
    cv::Mat empty = cv::Mat::zeros(1, 100, CV_32FC2);
    float cellSize = 0.05f;
    cv::Point3f viewPoint(0.0f, 0.0f, 0.0f);
    
    data.setOccupancyGrid(ground, obstacles, empty, cellSize, viewPoint);
    
    EXPECT_FALSE(data.gridGroundCellsRaw().empty());
    EXPECT_FALSE(data.gridObstacleCellsRaw().empty());
    EXPECT_FALSE(data.gridEmptyCellsRaw().empty());
    EXPECT_FLOAT_EQ(data.gridCellSize(), cellSize);
    EXPECT_FLOAT_EQ(data.gridViewPoint().x, viewPoint.x);
    // It should auto compress
    EXPECT_FALSE(data.gridGroundCellsCompressed().empty());
    EXPECT_FALSE(data.gridObstacleCellsCompressed().empty());
    EXPECT_FALSE(data.gridEmptyCellsCompressed().empty());
}

TEST(SensorDataTest, SetOccupancyGridCompressed)
{
    SensorData data;
    // Use supported compressed format: CV_8UC1 with 1 row
    cv::Mat ground = cv::Mat::ones(1, 100, CV_8UC1);
    cv::Mat obstacles = cv::Mat::ones(1, 100, CV_8UC1);
    cv::Mat empty = cv::Mat::ones(1, 100, CV_8UC1);
    float cellSize = 0.05f;
    cv::Point3f viewPoint(0.0f, 0.0f, 0.0f);
    
    data.setOccupancyGrid(ground, obstacles, empty, cellSize, viewPoint);
    
    EXPECT_FALSE(data.gridGroundCellsCompressed().empty());
    EXPECT_FALSE(data.gridObstacleCellsCompressed().empty());
    EXPECT_FALSE(data.gridEmptyCellsCompressed().empty());
    EXPECT_FLOAT_EQ(data.gridCellSize(), cellSize);
}

TEST(SensorDataTest, SetOccupancyGridUnsupportedGroundFormat)
{
    SensorData data;
    // Unsupported format: CV_8UC1 with multiple rows (not compressed format)
    cv::Mat ground = cv::Mat::ones(100, 100, CV_8UC1);
    cv::Mat obstacles = cv::Mat::zeros(1, 100, CV_32FC2);
    cv::Mat empty = cv::Mat::zeros(1, 100, CV_32FC2);
    float cellSize = 0.05f;
    cv::Point3f viewPoint(0.0f, 0.0f, 0.0f);
    
    EXPECT_THROW(data.setOccupancyGrid(ground, obstacles, empty, cellSize, viewPoint), UException);
}

TEST(SensorDataTest, SetOccupancyGridUnsupportedObstacleFormat)
{
    SensorData data;
    cv::Mat ground = cv::Mat::zeros(1, 100, CV_32FC2);
    // Unsupported format: CV_16UC1
    cv::Mat obstacles = cv::Mat::ones(1, 100, CV_16UC1);
    cv::Mat empty = cv::Mat::zeros(1, 100, CV_32FC2);
    float cellSize = 0.05f;
    cv::Point3f viewPoint(0.0f, 0.0f, 0.0f);
    
    EXPECT_THROW(data.setOccupancyGrid(ground, obstacles, empty, cellSize, viewPoint), UException);
}

TEST(SensorDataTest, SetOccupancyGridUnsupportedEmptyFormat)
{
    SensorData data;
    cv::Mat ground = cv::Mat::zeros(1, 100, CV_32FC2);
    cv::Mat obstacles = cv::Mat::zeros(1, 100, CV_32FC2);
    // Unsupported format: CV_32FC1 (only CV_32FC2 through CV_32FC7 are supported)
    cv::Mat empty = cv::Mat::ones(1, 100, CV_32FC1);
    float cellSize = 0.05f;
    cv::Point3f viewPoint(0.0f, 0.0f, 0.0f);
    
    EXPECT_THROW(data.setOccupancyGrid(ground, obstacles, empty, cellSize, viewPoint), UException);
}

TEST(SensorDataTest, SetOccupancyGridSupportedFormats)
{
    float cellSize = 0.05f;
    cv::Point3f viewPoint(0.0f, 0.0f, 0.0f);
    
    // Test all supported formats: CV_32FC2 through CV_32FC7
    for (int channels = 2; channels <= 7; ++channels)
    {
        SensorData data;
        cv::Mat ground = cv::Mat(1, 100, CV_32FC(channels));
        cv::Mat obstacles = cv::Mat(1, 100, CV_32FC(channels));
        cv::Mat empty = cv::Mat(1, 100, CV_32FC(channels));
        
        EXPECT_NO_THROW(data.setOccupancyGrid(ground, obstacles, empty, cellSize, viewPoint))
            << "Failed for CV_32FC(" << channels << ")";
    }
}

TEST(SensorDataTest, ClearOccupancyGridRaw)
{
    SensorData data;
    cv::Mat ground = cv::Mat::ones(1, 100, CV_32FC2);
    cv::Mat obstacles = cv::Mat::zeros(1, 100, CV_32FC2);
    cv::Mat empty = cv::Mat::zeros(1, 100, CV_32FC2);
    data.setOccupancyGrid(ground, obstacles, empty, 0.05f, cv::Point3f(0, 0, 0));
    
    EXPECT_FALSE(data.gridGroundCellsRaw().empty());
    
    data.clearOccupancyGridRaw();
    
    EXPECT_TRUE(data.gridGroundCellsRaw().empty());
    EXPECT_TRUE(data.gridObstacleCellsRaw().empty());
    // Should not remove compressed
    EXPECT_FALSE(data.gridGroundCellsCompressed().empty());
    EXPECT_FALSE(data.gridObstacleCellsCompressed().empty());
}

// Data Clearing Tests

TEST(SensorDataTest, ClearCompressedData)
{
    cv::Mat rgb = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 640, CV_16UC1) * 1000;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    // Construct with compressed data (simulated compressed data)
    SensorData data(cv::Mat::ones(1, 100, CV_8UC1), cv::Mat::ones(1, 100, CV_8UC1), model);
    // Add raw data
    data.setRGBDImage(rgb, depth, model, false);
    
    data.clearCompressedData(true, false, false);
    
    // Raw data should still be present
    EXPECT_FALSE(data.imageRaw().empty());
    EXPECT_FALSE(data.depthRaw().empty());
}

TEST(SensorDataTest, ClearRawData)
{
    SensorData data;
    cv::Mat rgb = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 640, CV_16UC1) * 1000;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    data.setRGBDImage(rgb, depth, model);
    
    EXPECT_FALSE(data.imageRaw().empty());
    
    data.clearRawData(true, false, false);
    
    EXPECT_TRUE(data.imageRaw().empty());
}

// Visibility Tests

TEST(SensorDataTest, IsPointVisibleFromCameras)
{
    SensorData data;
    CameraModel model(525.0, 525.0, 320.0, 240.0, CameraModel::opticalRotation(), 0.0, cv::Size(640, 480));
    data.setCameraModel(model);
    cv::Mat rgb = cv::Mat::ones(480, 640, CV_8UC3);
    data.setRGBDImage(rgb, cv::Mat(), model);

    EXPECT_EQ(data.cameraModels().size(), 1);
    EXPECT_TRUE(data.cameraModels()[0].isValidForProjection());
    
    // Point in front of camera (should be visible, returns camera index 0)
    cv::Point3f pt(1.0f, 0.0f, 0.0f);
    int cameraIndex = data.isPointVisibleFromCameras(pt);
    EXPECT_EQ(cameraIndex, 0);
    
    // Point behind camera (should not be visible, returns -1)
    cv::Point3f ptBehind(-1.0f, 0.0f, 0.0f);
    cameraIndex = data.isPointVisibleFromCameras(ptBehind);
    EXPECT_EQ(cameraIndex, -1);
}

TEST(SensorDataTest, IsPointVisibleFromCamerasMultiCamera)
{
    SensorData data;
    std::vector<CameraModel> models;
    models.push_back(CameraModel(525.0, 525.0, 320.0, 240.0, CameraModel::opticalRotation(), 0.0, cv::Size(640, 480)));
    models.push_back(CameraModel(525.0, 525.0, 320.0, 240.0, CameraModel::opticalRotation(), 0.0, cv::Size(640, 480)));
    data.setCameraModels(models);
    cv::Mat rgb = cv::Mat::ones(480, 1280, CV_8UC3);
    data.setRGBDImage(rgb, cv::Mat(), models);

    EXPECT_EQ(data.cameraModels().size(), 2);
    
    // Point visible from first camera (should return index 0)
    cv::Point3f pt(1.0f, 0.0f, 0.0f);
    int cameraIndex = data.isPointVisibleFromCameras(pt);
    EXPECT_GE(cameraIndex, 0);
    EXPECT_LE(cameraIndex, 1);
    
    // Point not visible from any camera (should return -1)
    cv::Point3f ptBehind(-1.0f, 0.0f, 0.0f);
    cameraIndex = data.isPointVisibleFromCameras(ptBehind);
    EXPECT_EQ(cameraIndex, -1);
}

TEST(SensorDataTest, IsPointVisibleFromCamerasNoCameras)
{
    SensorData data;
    // No camera models set
    
    cv::Point3f pt(1.0f, 0.0f, 0.0f);
    int cameraIndex = data.isPointVisibleFromCameras(pt);
    EXPECT_EQ(cameraIndex, -1);
}

// Memory Usage Tests

TEST(SensorDataTest, GetMemoryUsed)
{
    SensorData data;
    unsigned long memEmpty = data.getMemoryUsed();
    
    cv::Mat rgb = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 640, CV_16UC1) * 1000;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    data.setRGBDImage(rgb, depth, model);
    
    unsigned long memWithData = data.getMemoryUsed();
    
    EXPECT_GT(memWithData, memEmpty);
}

// Comprehensive Tests

TEST(SensorDataTest, ComprehensiveUsage)
{
    SensorData data;
    
    // Set ID and stamp
    data.setId(1000);
    data.setStamp(123456.789);
    
    // Set RGB-D images
    cv::Mat rgb = cv::Mat::ones(480, 640, CV_8UC3) * 128;
    cv::Mat depth = cv::Mat::ones(480, 640, CV_16UC1) * 1000;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    data.setRGBDImage(rgb, depth, model);
    
    // Set laser scan
    LaserScan scan = LaserScan::backwardCompatibility(cv::Mat::ones(1, 640, CV_32FC3));
    data.setLaserScan(scan);
    
    // Set features
    std::vector<cv::KeyPoint> keypoints;
    keypoints.push_back(cv::KeyPoint(100, 200, 5.0f));
    std::vector<cv::Point3f> keypoints3D;
    keypoints3D.push_back(cv::Point3f(1.0f, 2.0f, 3.0f));
    cv::Mat descriptors = cv::Mat::ones(1, 128, CV_32F);
    data.setFeatures(keypoints, keypoints3D, descriptors);
    
    // Set global pose
    Transform pose(10.0f, 20.0f, 30.0f, 0, 0, 0);
    cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.1;
    data.setGlobalPose(pose, covariance);
    
    // Verify all data
    EXPECT_TRUE(data.isValid());
    EXPECT_EQ(data.id(), 1000);
    EXPECT_DOUBLE_EQ(data.stamp(), 123456.789);
    EXPECT_FALSE(data.imageRaw().empty());
    EXPECT_FALSE(data.depthOrRightRaw().empty());
    EXPECT_FALSE(data.laserScanRaw().isEmpty());
    EXPECT_EQ(data.keypoints().size(), 1u);
    EXPECT_FALSE(data.globalPose().isNull());
}

// Edge Cases

TEST(SensorDataTest, EmptyImages)
{
    SensorData data;
    cv::Mat emptyRgb;
    cv::Mat emptyDepth;
    CameraModel model(525.0, 525.0, 320.0, 240.0);
    
    data.setRGBDImage(emptyRgb, emptyDepth, model);
    
    EXPECT_TRUE(data.imageRaw().empty());
    EXPECT_TRUE(data.depthOrRightRaw().empty());
    EXPECT_FALSE(data.cameraModels().empty());
}

TEST(SensorDataTest, DifferentImageTypes)
{
    SensorData data;
    
    // Grayscale
    cv::Mat gray = cv::Mat::zeros(100, 100, CV_8UC1);
    data.setRGBDImage(gray, cv::Mat(), CameraModel());
    EXPECT_EQ(data.imageRaw().type(), CV_8UC1);
    
    // RGB
    cv::Mat color = cv::Mat::zeros(100, 100, CV_8UC3);
    data.setRGBDImage(color, cv::Mat(), CameraModel());
    EXPECT_EQ(data.imageRaw().type(), CV_8UC3);
}

TEST(SensorDataTest, DifferentDepthTypes)
{
    SensorData data;
    cv::Mat rgb = cv::Mat::ones(100, 100, CV_8UC3);
    CameraModel model(525.0, 525.0, 50.0, 50.0);
    
    // 16-bit depth (millimeters)
    cv::Mat depth16 = cv::Mat::ones(100, 100, CV_16UC1) * 1000;
    data.setRGBDImage(rgb, depth16, model);
    EXPECT_EQ(data.depthOrRightRaw().type(), CV_16UC1);
    
    // 32-bit depth (meters)
    cv::Mat depth32 = cv::Mat::ones(100, 100, CV_32FC1) * 1.0f;
    data.setRGBDImage(rgb, depth32, model);
    EXPECT_EQ(data.depthOrRightRaw().type(), CV_32FC1);
}

