#include "gtest/gtest.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/UException.h"
#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace rtabmap;

TEST(Util3dTest, rgbFromCloudGeneratesCorrectRGB)
{
    // Create a 2x2 organized point cloud with known colors
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    cloud.width = 2;
    cloud.height = 2;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);

    // Fill in the cloud with distinct colors
    cloud.at(0).r = 255; cloud.at(0).g = 0; cloud.at(0).b = 0;   // Red
    cloud.at(1).r = 0; cloud.at(1).g = 255; cloud.at(1).b = 0;   // Green
    cloud.at(2).r = 0; cloud.at(2).g = 0; cloud.at(2).b = 255;   // Blue
    cloud.at(3).r = 255; cloud.at(3).g = 255; cloud.at(3).b = 255; // White

    // Test with bgrOrder = true
    cv::Mat bgr = util3d::rgbFromCloud(cloud, true);
    ASSERT_EQ(bgr.rows, 2);
    ASSERT_EQ(bgr.cols, 2);
    EXPECT_EQ(bgr.at<cv::Vec3b>(0,0), cv::Vec3b(0,0,255));   // BGR for red
    EXPECT_EQ(bgr.at<cv::Vec3b>(0,1), cv::Vec3b(0,255,0));   // BGR for green
    EXPECT_EQ(bgr.at<cv::Vec3b>(1,0), cv::Vec3b(255,0,0));   // BGR for blue
    EXPECT_EQ(bgr.at<cv::Vec3b>(1,1), cv::Vec3b(255,255,255)); // White

    // Test with bgrOrder = false
    cv::Mat rgb = util3d::rgbFromCloud(cloud, false);
    EXPECT_EQ(rgb.at<cv::Vec3b>(0,0), cv::Vec3b(255,0,0));   // RGB for red
    EXPECT_EQ(rgb.at<cv::Vec3b>(0,1), cv::Vec3b(0,255,0));   // RGB for green
    EXPECT_EQ(rgb.at<cv::Vec3b>(1,0), cv::Vec3b(0,0,255));   // RGB for blue
    EXPECT_EQ(rgb.at<cv::Vec3b>(1,1), cv::Vec3b(255,255,255)); // White
}

TEST(Util3dTest, depthFromCloudBasic32F)
{
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    cloud.width = 3;
    cloud.height = 2;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    // Fill the cloud with predictable Z values
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud.points[i].x = float(i);
        cloud.points[i].y = float(i);
        cloud.points[i].z = 1.0f + float(i) * 0.1f;
    }

    cv::Mat depth = util3d::depthFromCloud(cloud, false);

    std::cout << depth << std::endl;

    ASSERT_EQ(depth.type(), CV_32FC1);
    ASSERT_EQ(depth.rows, 2);
    ASSERT_EQ(depth.cols, 3);

    // Check specific depth values
    for (int r = 0; r < depth.rows; ++r)
    {
        for (int c = 0; c < depth.cols; ++c)
        {
            float expected = 1.0f + float(r * depth.cols + c) * 0.1f;
            EXPECT_FLOAT_EQ(depth.at<float>(r, c), expected);
        }
    }
}

TEST(Util3dTest, depthFromCloudBasic16U)
{
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    cloud.width = 2;
    cloud.height = 2;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud.points[i].x = float(i);
        cloud.points[i].y = float(i);
        cloud.points[i].z = 1.0f; // 1 meter
    }

    cv::Mat depth = util3d::depthFromCloud(cloud, true);

    ASSERT_EQ(depth.type(), CV_16UC1);
    ASSERT_EQ(depth.rows, 2);
    ASSERT_EQ(depth.cols, 2);

    for (int r = 0; r < depth.rows; ++r)
    {
        for (int c = 0; c < depth.cols; ++c)
        {
            EXPECT_EQ(depth.at<uint16_t>(r, c), 1000); // 1000 mm
        }
    }
}

TEST(Util3dTest, rgbdFromCloudBasicConversion)
{
    // Create a 2x2 point cloud with synthetic values
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    cloud.width = 2;
    cloud.height = 2;
    cloud.is_dense = true;
    cloud.points.resize(4);

    for (size_t i = 0; i < 4; ++i)
    {
        cloud.points[i].x = static_cast<float>(i);
        cloud.points[i].y = static_cast<float>(i) * 0.5f;
        cloud.points[i].z = 1.0f + 0.1f * i;
        cloud.points[i].r = 10 * i;
        cloud.points[i].g = 20 * i;
        cloud.points[i].b = 30 * i;
    }

    cv::Mat bgr, depth;

    util3d::rgbdFromCloud(cloud, bgr, depth, true, false);

    // Check image sizes and types
    ASSERT_EQ(bgr.rows, 2);
    ASSERT_EQ(bgr.cols, 2);
    ASSERT_EQ(bgr.type(), CV_8UC3);
    ASSERT_EQ(depth.rows, 2);
    ASSERT_EQ(depth.cols, 2);
    ASSERT_EQ(depth.type(), CV_32FC1);

    // Check a pixel's values
    cv::Vec3b color = bgr.at<cv::Vec3b>(0,1);
    EXPECT_EQ(color[0], 30);  // B
    EXPECT_EQ(color[1], 20);  // G
    EXPECT_EQ(color[2], 10);  // R

    float d = depth.at<float>(0,0);
    EXPECT_NEAR(d, 1.0f, 1e-5);
}

TEST(Util3dTest, projectDepthTo3D)
{
    // Create a synthetic 5x5 depth image with all values set to 1.0 meter
    cv::Mat depthImage = cv::Mat::ones(5, 5, CV_32FC1);

    float fx = 525.0f;
    float fy = 525.0f;
    float cx = 2.0f;
    float cy = 2.0f;

    // Project the center pixel (2, 2)
    pcl::PointXYZ pt = util3d::projectDepthTo3D(depthImage, 2, 2, cx, cy, fx, fy, false, 0.0f);

    // Since we project the principal point, x and y should be zero
    EXPECT_FLOAT_EQ(pt.x, 0.0f);
    EXPECT_FLOAT_EQ(pt.y, 0.0f);
    EXPECT_FLOAT_EQ(pt.z, 1.0f);

    // Project a pixel to the right of center (3, 2)
    pt = util3d::projectDepthTo3D(depthImage, 3, 2, cx, cy, fx, fy, false, 0.0f);
    EXPECT_NEAR(pt.x, (3.0f - cx) * 1.0f / fx, 1e-5);
    EXPECT_FLOAT_EQ(pt.y, 0.0f);
    EXPECT_FLOAT_EQ(pt.z, 1.0f);
}

TEST(Util3dTest, projectDepthTo3DRayBasicRayProjection)
{
    cv::Size imageSize(640, 480);
    float fx = 525.0f;
    float fy = 525.0f;
    float cx = 319.5f;
    float cy = 239.5f;

    // Center pixel should return (0,0,1)
    Eigen::Vector3f ray = util3d::projectDepthTo3DRay(imageSize, cx, cy, cx, cy, fx, fy);
    EXPECT_NEAR(ray[0], 0.0f, 1e-5);
    EXPECT_NEAR(ray[1], 0.0f, 1e-5);
    EXPECT_NEAR(ray[2], 1.0f, 1e-5);

    // A pixel 1 unit right should return a small positive x component
    ray = util3d::projectDepthTo3DRay(imageSize, cx + 1, cy, cx, cy, fx, fy);
    EXPECT_NEAR(ray[0], 1.0f / fx, 1e-5);
    EXPECT_NEAR(ray[1], 0.0f, 1e-5);
    EXPECT_NEAR(ray[2], 1.0f, 1e-5);
}

TEST(Util3dTest, cloudFromDepthWithCameraModel) {
    // Create a sample depth image (CV_32FC1 format)
    cv::Mat depthImage(4, 4, CV_32FC1);
    for(int j=0;j<depthImage.rows; ++j)
    {
        for(int i=0;i<depthImage.cols; ++i)
        {
            depthImage.at<float>(j, i) = j*depthImage.cols + i + 1;
        }
    }

    // Camera model with arbitrary parameters
    CameraModel model(10.0f, 10.0f, 1.5f, 1.5f);

    // Create test cloud from depth image
    std::vector<int> validIndices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::cloudFromDepth(depthImage, model, 1, 0.0f, 0.0f, &validIndices);

    // Check if cloud is created
    ASSERT_NE(cloud, nullptr);
    ASSERT_EQ(cloud->height, 4);
    ASSERT_EQ(cloud->width, 4);

    ASSERT_EQ(cloud->size(), validIndices.size());

    // Check if the first point in the cloud has reasonable values
    pcl::PointXYZ & pt = cloud->at(0);
    ASSERT_LT(pt.x, 0.0f); // Sample assert, check actual expected values
    ASSERT_LT(pt.y, 0.0f); // Sample assert, check actual expected values
    ASSERT_FLOAT_EQ(pt.z, 1.0f); // Check depth
}

TEST(Util3dTest, cloudFromDepthRGBValidInputCreatesPointCloud) {
    // Create a simple test depth image (e.g., 5x5)
    cv::Mat imageRgb = cv::Mat::zeros(5, 5, CV_8UC3); // Black RGB image
    cv::Mat imageDepth = cv::Mat::ones(5, 5, CV_32FC1); // Depth image with all values 1.0

    CameraModel model(500, 500, 2.5, 2.5); // Example CameraModel parameters
    std::vector<int> validIndices;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(imageRgb, imageDepth, model, 1, 10.0f, 0.1f, &validIndices);

    // Check if the cloud contains points
    ASSERT_FALSE(cloud->empty());
    ASSERT_EQ(cloud->width, 5);
    ASSERT_EQ(cloud->height, 5);

    // Check if some valid indices exist
    ASSERT_EQ(validIndices.size(), cloud->size());
}

TEST(Util3dTest, cloudFromDepthRGBEmptyInput) {
    cv::Mat imageRgb = cv::Mat(); // Empty RGB image
    cv::Mat imageDepth = cv::Mat(); // Empty Depth image

    CameraModel model(500, 500, 2.5, 2.5);
    std::vector<int> validIndices;

    ASSERT_THROW(util3d::cloudFromDepthRGB(imageRgb, imageDepth, model, 1, 10.0f, 0.1f, &validIndices), UException);
}

TEST(Util3dTest, cloudFromDepthRGBDecimationReducesPointCloudSize) {
    // Create a simple test depth image (e.g., 6x6) and RGB image
    cv::Mat imageRgb = cv::Mat::zeros(6, 6, CV_8UC3); 
    cv::Mat imageDepth = cv::Mat::ones(6, 6, CV_32FC1);

    CameraModel model(500, 500, 3.0, 3.0);
    std::vector<int> validIndices;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(imageRgb, imageDepth, model, 2, 10.0f, 0.1f, &validIndices);

    // Decimation by 2 should result in 3x3 cloud
    ASSERT_EQ(cloud->width, 3);
    ASSERT_EQ(cloud->height, 3);
}

// Test depth constraints: points outside the range should not be included in the cloud
TEST(Util3dTest, cloudFromDepthRGBDepthConstraintsLimitPointCloud) {
    cv::Mat imageRgb = cv::Mat::zeros(5, 5, CV_8UC3);
    cv::Mat imageDepth = cv::Mat::ones(5, 5, CV_32FC1) * 0.05f; // Depth values smaller than the minimum

    CameraModel model(500, 500, 2.5, 2.5);
    std::vector<int> validIndices;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(imageRgb, imageDepth, model, 1, 10.0f, 0.1f, &validIndices);

    // Check that no points are valid (because the depth is below the minimum)
    ASSERT_EQ(cloud->size(), 25);
    ASSERT_EQ(validIndices.size(), 0);
}

TEST(Util3dTest, cloudFromDepthRGBImageSizeMismatchThrowsError) {
    cv::Mat imageRgb = cv::Mat::zeros(6, 6, CV_8UC3);  // 6x6 RGB image
    cv::Mat imageDepth = cv::Mat::ones(6, 6, CV_32FC1); // 6x6 Depth image

    CameraModel model(500, 500, 2.5, 2.5, CameraModel::opticalRotation(), 0, cv::Size(5,5)); // should mismatch

    // Test should fail since image size and calibration model don't match
    ASSERT_THROW(util3d::cloudFromDepthRGB(imageRgb, imageDepth, model, 1, 10.0f, 0.1f, nullptr), UException);
}

TEST(Util3dTest, cloudFromDisparityValidInputs) {
    cv::Mat imageDisparity = cv::Mat::ones(10, 10, CV_32FC1) * 50.0f;  // 50.0 as the disparity value
    StereoCameraModel model(10, 10, 4.5f, 4.5f, 0.05f, CameraModel::opticalRotation(), imageDisparity.size());

    // Decimation, maxDepth, and minDepth values
    int decimation = 1;
    float maxDepth = 0.0f;
    float minDepth = 0.0f;

    std::vector<int> validIndices;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::cloudFromDisparity(imageDisparity, model, decimation, maxDepth, minDepth, &validIndices);

    // Verify the size of the cloud and valid indices
    ASSERT_NE(cloud, nullptr);
    ASSERT_EQ(cloud->size(), imageDisparity.total());
    ASSERT_EQ(validIndices.size(), imageDisparity.total());

    // try with 
    imageDisparity = cv::Mat::ones(10, 10, CV_16SC1) * 50 * 16;
    cloud = util3d::cloudFromDisparity(imageDisparity, model, decimation, maxDepth, minDepth, &validIndices);

    // Verify the size of the cloud and valid indices
    ASSERT_NE(cloud, nullptr);
    ASSERT_EQ(cloud->size(), imageDisparity.total());
    ASSERT_EQ(validIndices.size(), imageDisparity.total());
}

TEST(Util3dTest, cloudFromDisparityRGBValidInputs) {
    cv::Mat imageRgb = cv::Mat(10, 10, CV_8UC3, cv::Scalar(255, 200, 150));  // BGR
    cv::Mat imageDisparity = cv::Mat::ones(10, 10, CV_32FC1) * 50.0f;  // 50.0 as the disparity value
    StereoCameraModel model(10, 10, 4.5f, 4.5f, 0.05f, CameraModel::opticalRotation(), imageDisparity.size());

    // Decimation, maxDepth, and minDepth values
    int decimation = 1;
    float maxDepth = 0.0f;
    float minDepth = 0.0f;

    std::vector<int> validIndices;

    // Call the function
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDisparityRGB(imageRgb, imageDisparity, model, decimation, maxDepth, minDepth, &validIndices);

    // Verify the size of the cloud and valid indices
    ASSERT_NE(cloud, nullptr);
    ASSERT_EQ(cloud->size(), imageDisparity.total());
    ASSERT_EQ(validIndices.size(), imageDisparity.total());

    // Test if the values are within the expected depth range and color assignment
    for (const auto& pt : *cloud) {
        ASSERT_GE(pt.z, minDepth);
        ASSERT_EQ(pt.r, 150);  // Check if the color is correctly assigned to red
        ASSERT_EQ(pt.g, 200);  // Check if the color is correctly assigned to green
        ASSERT_EQ(pt.b, 255);  // Check if the color is correctly assigned to blue
    }
}

TEST(Util3dTest, cloudFromDisparityInvalidDecimation) {
    // Create a mock disparity image (CV_32FC1)
    cv::Mat imageDisparity = cv::Mat::ones(10, 10, CV_32FC1) * 50.0f;
    StereoCameraModel model(10, 10, 4.5f, 4.5f, 0.05f, CameraModel::opticalRotation(), imageDisparity.size());

    // Set invalid decimation factor
    int decimation = 3;  // Should be divisible by image size
    float maxDepth = 0.0f;
    float minDepth = 0.0f;

    std::vector<int> validIndices;

    // Call the function
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::cloudFromDisparity(imageDisparity, model, decimation, maxDepth, minDepth, &validIndices);

    // Verify that the cloud is still generated and decimation is adjusted
    ASSERT_NE(cloud, nullptr);
    ASSERT_GT(cloud->size(), 0);
}

TEST(Util3dTest, cloudFromDisparityEmptyImages) {
    // Call the function with empty images
    ASSERT_THROW(util3d::cloudFromDisparity(cv::Mat(), StereoCameraModel()), UException);
    ASSERT_THROW(util3d::cloudFromDisparityRGB(cv::Mat(), cv::Mat(), StereoCameraModel()), UException);
}

TEST(Util3dTest, cloudFromStereoImagesBasicTest) {
    cv::Mat imageLeft = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/stereo_rect/left/50.jpg", cv::IMREAD_UNCHANGED);
    cv::Mat imageRight = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/stereo_rect/right/50.jpg", cv::IMREAD_GRAYSCALE);

    ASSERT_FALSE(imageLeft.empty());
    ASSERT_FALSE(imageRight.empty());

    // Stereo camera model
    StereoCameraModel model;
    ASSERT_TRUE(model.load(std::string(RTABMAP_TEST_DATA_ROOT) + "/stereo_rect", "stereo"));

    // Decimation factor (reduce resolution by half)
    int decimation = 2;

    // Depth range settings
    float maxDepth = 10.0f;  // Max depth in meters
    float minDepth = 0.1f;   // Min depth in meters

    // Vector to hold valid indices (optional)
    std::vector<int> validIndices;

    ParametersMap parameters;
    parameters[Parameters::kStereoBMNumDisparities()] = "64"; 

    // Call the function
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromStereoImages(
        imageLeft, imageRight, model, decimation, maxDepth, minDepth, &validIndices, parameters);

    // Verify the cloud is not empty and has the expected properties
    ASSERT_NE(cloud, nullptr);
    ASSERT_EQ(cloud->width, imageLeft.cols/decimation);
    ASSERT_EQ(cloud->height, imageLeft.rows/decimation);

    ASSERT_GT(validIndices.size(), 0);

    std::set<int> validSet(validIndices.begin(), validIndices.end());

    // Check that the cloud points are within expected bounds (e.g., not NaN)
    for (size_t i=0; i<cloud->size(); ++i)
    {
        const pcl::PointXYZRGB & point = cloud->at(i);
        if(validSet.find(i) != validSet.end())
        {
            ASSERT_TRUE(std::isfinite(point.x));
            ASSERT_TRUE(std::isfinite(point.y));
            ASSERT_TRUE(std::isfinite(point.z));
            ASSERT_GT(point.z, minDepth);
            ASSERT_LT(point.z, maxDepth);

            int v = i / cloud->width;
            int u = i - v*cloud->width;
            ASSERT_EQ(point.b, imageLeft.at<cv::Vec3b>(v*decimation, u*decimation)[0]);
            ASSERT_EQ(point.g, imageLeft.at<cv::Vec3b>(v*decimation, u*decimation)[1]);
            ASSERT_EQ(point.r, imageLeft.at<cv::Vec3b>(v*decimation, u*decimation)[2]);
        }
        else
        {
            ASSERT_FALSE(std::isfinite(point.x));
            ASSERT_FALSE(std::isfinite(point.y));
            ASSERT_FALSE(std::isfinite(point.z));
        }
    }
}

TEST(Util3dTest, cloudsFromSensorDataBasicCloudGeneration) {
    cv::Mat depthImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/depth/17.png", cv::IMREAD_UNCHANGED);
    cv::Mat rgbImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/rgb/17.jpg", cv::IMREAD_COLOR);
    
    ASSERT_FALSE(depthImage.empty());
    ASSERT_FALSE(rgbImage.empty());

    CameraModel model;
    ASSERT_TRUE(model.load(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/calib/17.yaml"));
    SensorData sensorData(rgbImage, depthImage, model);

    // Test Parameters
    int decimation = 1;  // No decimation
    float maxDepth = 10.0f;
    float minDepth = 0.1f;
    std::vector<float> roiRatios = {0.0f, 0.0f, 0.0f, 0.0f};  // Full image

    // Call the function to generate clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds = util3d::cloudsFromSensorData(
        sensorData,
        decimation,
        maxDepth,
        minDepth,
        nullptr,  // No validIndices in this test
        ParametersMap(),
        roiRatios
    );

    // Check that we got clouds
    ASSERT_GT(clouds.size(), 0) << "Expected at least one point cloud";
    
    // Check the size of the first cloud to be non-zero
    ASSERT_GT(clouds[0]->size(), 0) << "First point cloud is empty";
}

TEST(Util3dTest, cloudsFromSensorDataCloudGenerationWithDecimation) {
    cv::Mat depthImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/depth/17.png", cv::IMREAD_UNCHANGED);
    cv::Mat rgbImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/rgb/17.jpg", cv::IMREAD_COLOR);
    
    CameraModel model;
    model.load(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/calib/17.yaml");
    SensorData sensorData(rgbImage, depthImage, model);

    // Test Parameters
    int decimation = 2;  // Apply decimation
    float maxDepth = 10.0f;
    float minDepth = 0.1f;
    std::vector<float> roiRatios = {0.0f, 0.0f, 0.0f, 0.0f};

    // Generate clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds = util3d::cloudsFromSensorData(
        sensorData,
        decimation,
        maxDepth,
        minDepth,
        nullptr,
        ParametersMap(),
        roiRatios
    );

    // Check that we got clouds
    ASSERT_GT(clouds.size(), 0) << "Expected at least one point cloud";

    // Check that the cloud size after decimation is smaller than the original
    ASSERT_GT(clouds[0]->size(), 0) << "First point cloud is empty after decimation";
    ASSERT_LT(clouds[0]->size(), 640 * 480) << "Cloud size should be smaller due to decimation";
}

TEST(Util3dTest, cloudsFromSensorDataCloudGenerationWithEmptyImages) {
    // Create empty sensor data
    SensorData sensorData;

    // Test Parameters
    int decimation = 1;
    float maxDepth = 10.0f;
    float minDepth = 0.1f;
    std::vector<float> roiRatios = {0.0f, 0.0f, 0.0f, 0.0f};

    // Call the function with empty images
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds = util3d::cloudsFromSensorData(
        sensorData,
        decimation,
        maxDepth,
        minDepth,
        nullptr,
        ParametersMap(),
        roiRatios
    );

    // Assert that no clouds are generated from empty images
    ASSERT_EQ(clouds.size(), 0) << "Expected no point clouds for empty input images";
}

TEST(Util3dTest, cloudFromSensorDataGeneratesPointCloud) {
    cv::Mat depthImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/depth/17.png", cv::IMREAD_UNCHANGED);
    cv::Mat rgbImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/rgb/17.jpg", cv::IMREAD_COLOR);
    
    ASSERT_FALSE(depthImage.empty());
    ASSERT_FALSE(rgbImage.empty());

    CameraModel model;
    ASSERT_TRUE(model.load(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/calib/17.yaml"));
    SensorData sensorData(rgbImage, depthImage, model);

    int decimation = 1;  // No decimation
    float maxDepth = 10.0f;
    float minDepth = 0.1f;
    std::vector<int> validIndices;
    std::vector<float> roiRatios = {0.0f, 0.0f, 0.0f, 0.0f};  // Full image

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::cloudFromSensorData(
        sensorData,
        decimation,
        maxDepth,
        minDepth,
        &validIndices,
        ParametersMap(),
        roiRatios
    );

    // Assert: Check that the cloud is not null and has points
    ASSERT_NE(cloud, nullptr);  // The cloud should not be null
    ASSERT_EQ(cloud->size(), rgbImage.total()); 

    // Check if valid indices are filled correctly
    ASSERT_GT(validIndices.size(), 0);
}

TEST(Util3dTest, cloudsRGBFromSensorDataTestSingleCamera) {
    cv::Mat depthImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/depth/17.png", cv::IMREAD_UNCHANGED);
    cv::Mat rgbImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/rgb/17.jpg", cv::IMREAD_COLOR);
    
    ASSERT_FALSE(depthImage.empty());
    ASSERT_FALSE(rgbImage.empty());

    CameraModel model;
    ASSERT_TRUE(model.load(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/calib/17.yaml"));
    SensorData sensorData(rgbImage, depthImage, model);

    int decimation = 1;
    float maxDepth = 10.0f;
    float minDepth = 0.1f;
    std::vector<pcl::IndicesPtr> validIndices;
    std::vector<float> roiRatios = {0.0f, 0.0f, 0.0f, 0.0f};  // ROI ratios for the test

    // Call the function
    auto clouds = util3d::cloudsRGBFromSensorData(
        sensorData, 
        decimation, 
        maxDepth, 
        minDepth, 
        &validIndices, 
        ParametersMap(), 
        roiRatios
    );

    // Check if the returned clouds are not empty
    ASSERT_FALSE(clouds.empty());

    // Check if the first cloud has a reasonable size (not empty)
    ASSERT_GT(clouds[0]->size(), 0);
    ASSERT_GT(validIndices[0]->size(), 0);
}

TEST(Util3dTest, cloudsRGBFromSensorDataTestStereoCameras) {
    cv::Mat imageLeft = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/stereo_rect/left/50.jpg", cv::IMREAD_UNCHANGED);
    cv::Mat imageRight = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/stereo_rect/right/50.jpg", cv::IMREAD_GRAYSCALE);

    ASSERT_FALSE(imageLeft.empty());
    ASSERT_FALSE(imageRight.empty());

    // Stereo camera model
    StereoCameraModel model;
    ASSERT_TRUE(model.load(std::string(RTABMAP_TEST_DATA_ROOT) + "/stereo_rect", "stereo"));
    SensorData sensorData(imageLeft, imageRight, model);
    
    int decimation = 2;
    float maxDepth = 10.0f;
    float minDepth = 0.1f;
    std::vector<pcl::IndicesPtr> validIndices;
    ParametersMap stereoParameters;  // Fill with necessary stereo parameters
    std::vector<float> roiRatios = {0.1f, 0.1f, 0.9f, 0.9f};  // ROI ratios for the test

    // Call the function for stereo cameras
    auto clouds = util3d::cloudsRGBFromSensorData(
        sensorData, 
        decimation, 
        maxDepth, 
        minDepth, 
        &validIndices, 
        stereoParameters, 
        roiRatios
    );

    // Check if the returned clouds are not empty
    ASSERT_FALSE(clouds.empty());

    // Check if the number of clouds matches the number of stereo camera models
    ASSERT_EQ(clouds.size(), sensorData.stereoCameraModels().size());

    // Check if the first cloud has a reasonable size (not empty)
    ASSERT_GT(clouds[0]->size(), 0);
    ASSERT_GT(validIndices[0]->size(), 0);
}

TEST(Util3dTest, cloudsRGBFromSensorDataTestEmptySensorData) {
    SensorData sensorData;  // Create empty sensor data (or no data at all)
    int decimation = 1;
    float maxDepth = 10.0f;
    float minDepth = 0.1f;
    std::vector<pcl::IndicesPtr> validIndices;
    ParametersMap stereoParameters;
    std::vector<float> roiRatios = {0.0f, 0.0f, 0.0f, 0.0f};

    // Call the function with empty sensor data
    auto clouds = util3d::cloudsRGBFromSensorData(
        sensorData, 
        decimation, 
        maxDepth, 
        minDepth, 
        &validIndices, 
        stereoParameters, 
        roiRatios
    );

    // The function should return an empty vector if no valid data is available
    ASSERT_TRUE(clouds.empty());
}

TEST(Util3dTest, cloudsRGBFromSensorDataTestInvalidROIRatios) {
    cv::Mat depthImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/depth/17.png", cv::IMREAD_UNCHANGED);
    cv::Mat rgbImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/rgb/17.jpg", cv::IMREAD_COLOR);
    
    ASSERT_FALSE(depthImage.empty());
    ASSERT_FALSE(rgbImage.empty());

    CameraModel model;
    ASSERT_TRUE(model.load(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/calib/17.yaml"));
    SensorData sensorData(rgbImage, depthImage, model);

    int decimation = 1;
    float maxDepth = 10.0f;
    float minDepth = 0.1f;
    std::vector<pcl::IndicesPtr> validIndices;
    ParametersMap stereoParameters;
    std::vector<float> roiRatios = {0.5f, 0.5f, 0.5f, 1.5f};  // Invalid ROI (this can be an edge case to test)

    // Call the function
    auto clouds = util3d::cloudsRGBFromSensorData(
        sensorData, 
        decimation, 
        maxDepth, 
        minDepth, 
        &validIndices, 
        stereoParameters, 
        roiRatios
    );

    // Check if clouds are still returned, but possibly with invalid results
    ASSERT_FALSE(clouds.empty());
    ASSERT_EQ(clouds[0]->size(), rgbImage.total());
    ASSERT_GT(validIndices[0]->size(), 0);
}

TEST(Util3dTest, cloudRGBFromSensorDataGeneratesCloudWithValidData) {
    cv::Mat depthImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/depth/17.png", cv::IMREAD_UNCHANGED);
    cv::Mat rgbImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/rgb/17.jpg", cv::IMREAD_COLOR);
    
    ASSERT_FALSE(depthImage.empty());
    ASSERT_FALSE(rgbImage.empty());

    CameraModel model;
    ASSERT_TRUE(model.load(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/calib/17.yaml"));
    SensorData sensorData(rgbImage, depthImage, model);

    int decimation = 1;   // No decimation
    float maxDepth = 10.0f;
    float minDepth = 0.1f;
    std::vector<int> validIndices;
    ParametersMap stereoParameters;  // Assuming stereoParameters are empty or predefined
    std::vector<float> roiRatios = {0.0f, 0.0f, 0.0f, 0.0f};  // Some ROI ratio

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
        sensorData,
        decimation,
        maxDepth,
        minDepth,
        &validIndices,
        stereoParameters,
        roiRatios
    );

    // Verify results
    ASSERT_NE(cloud, nullptr);  // The cloud should not be null
    EXPECT_EQ(cloud->size(), rgbImage.total()); 
    EXPECT_GT(validIndices.size(), 0);

    std::set<int> validSet(validIndices.begin(), validIndices.end());

    // Optionally verify point data (for example, checking if points have RGB values within expected ranges)
    for (size_t i=0; i<cloud->size(); ++i)
    {
        const pcl::PointXYZRGB & point = cloud->at(i);
        if(validSet.find(i) != validSet.end())
        {
            ASSERT_TRUE(std::isfinite(point.x));
            ASSERT_TRUE(std::isfinite(point.y));
            ASSERT_TRUE(std::isfinite(point.z));
            ASSERT_GT(point.x, minDepth); // in base frame
            ASSERT_LT(point.x, maxDepth); // in base frame

            int v = i / cloud->width;
            int u = i - v*cloud->width;
            ASSERT_EQ(point.b, rgbImage.at<cv::Vec3b>(v*decimation, u*decimation)[0]);
            ASSERT_EQ(point.g, rgbImage.at<cv::Vec3b>(v*decimation, u*decimation)[1]);
            ASSERT_EQ(point.r, rgbImage.at<cv::Vec3b>(v*decimation, u*decimation)[2]);
        }
        else
        {
            ASSERT_FALSE(std::isfinite(point.x));
            ASSERT_FALSE(std::isfinite(point.y));
            ASSERT_FALSE(std::isfinite(point.z));
        }
    }
}

TEST(Util3dTest, cloudRGBFromSensorDataHandlesEmptyData) {
    SensorData sensorData;
    int decimation = 1;
    float maxDepth = 10.0f;
    float minDepth = 0.1f;
    std::vector<int> validIndices;
    ParametersMap stereoParameters;
    std::vector<float> roiRatios;

    // Call the function under test
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
        sensorData,
        decimation,
        maxDepth,
        minDepth,
        &validIndices,
        stereoParameters,
        roiRatios
    );

    // Verify the cloud is generated, even with empty input data
    ASSERT_NE(cloud, nullptr);
    EXPECT_EQ(cloud->size(), 0);  // The cloud should be empty due to lack of data
    EXPECT_EQ(validIndices.size(), 0);  // No valid indices due to empty cloud
}

TEST(Util3dTest, cloudRGBFromSensorDataHandlesInvalidROI) {
    cv::Mat depthImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/depth/17.png", cv::IMREAD_UNCHANGED);
    cv::Mat rgbImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/rgb/17.jpg", cv::IMREAD_COLOR);
    
    ASSERT_FALSE(depthImage.empty());
    ASSERT_FALSE(rgbImage.empty());

    CameraModel model;
    ASSERT_TRUE(model.load(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/calib/17.yaml"));
    SensorData sensorData(rgbImage, depthImage, model);

    int decimation = 1;
    float maxDepth = 10.0f;
    float minDepth = 0.1f;
    std::vector<int> validIndices;
    ParametersMap stereoParameters;
    std::vector<float> roiRatios = {1.2f, 0.1f, 0.9f, 1.5f};  // Invalid ROI ratios

    // Call the function under test
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
        sensorData,
        decimation,
        maxDepth,
        minDepth,
        &validIndices,
        stereoParameters,
        roiRatios
    );

    // Verify the cloud is generated even if ROI ratios are invalid
    ASSERT_NE(cloud, nullptr);
    EXPECT_EQ(cloud->size(), rgbImage.total());
    EXPECT_GT(validIndices.size(), 0);
}

TEST(Util3dTest, laserScanFromDepthImage) {
    cv::Mat depthImage = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/depth/17.png", cv::IMREAD_UNCHANGED);
    ASSERT_FALSE(depthImage.empty());

    CameraModel model;
    ASSERT_TRUE(model.load(std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/calib/17.yaml"));

    // Depth range settings
    float maxDepth = 10.0f;
    float minDepth = 0.5f;

    // Call the function to get the laser scan
    pcl::PointCloud<pcl::PointXYZ> scan = util3d::laserScanFromDepthImage(
        depthImage, model.fx(), model.fy(), model.cx(), model.cy(), maxDepth, minDepth);

    // Check that the resulting point cloud has points
    ASSERT_GT(scan.size(), 0);
    for (const auto& pt : scan) {
        if(pcl::isFinite(pt)) {
            ASSERT_GE(pt.z, minDepth);   // Depth must be >= minDepth
            ASSERT_LT(pt.z, maxDepth);   // Depth must be <= maxDepth
        }
    }

    // empty image
    ASSERT_THROW(util3d::laserScanFromDepthImage(
        cv::Mat(), 0, 0, 0, 0, 0, 0, CameraModel::opticalRotation()), UException);
}

TEST(Util3dTest, laserScanFromDepthImageEmptyDepthImage) {
    
}

TEST(Util3dTest, laserScanFromDepthImages) {
    int width = 640;
    int height = 480;
    cv::Mat depthImage(height, width*2, CV_32FC1, cv::Scalar(1.0)); // Depth set to 1.0 for all points

    // Create camera models (assuming two cameras in a stereo setup)
    std::vector<CameraModel> cameraModels;
    cameraModels.push_back(CameraModel(
        525.0f, 525.0f, 319.5f, 239.5f, 
        CameraModel::opticalRotation(), 0, cv::Size(640,480)));
    cameraModels.push_back(CameraModel(
        525.0f, 525.0f, 319.5f, 239.5f, 
        Transform(0,0,-M_PI/2.0)*CameraModel::opticalRotation(), 0, cv::Size(640,480)));

    // Depth range settings
    float maxDepth = 10.0f;
    float minDepth = 0.5f;

    // Call the function to get the laser scan
    pcl::PointCloud<pcl::PointXYZ> scan = util3d::laserScanFromDepthImages(depthImage, cameraModels, maxDepth, minDepth);

    // Check that the resulting point cloud has points
    ASSERT_GT(scan.size(), 0);
    for (const auto& pt : scan) {
        ASSERT_TRUE(pcl::isFinite(pt));
        float d = uNorm(pt.x, pt.y, pt.z);
        ASSERT_GE(d, minDepth);   // Depth must be >= minDepth
        ASSERT_LT(d, maxDepth);   // Depth must be <= maxDepth
    }

    // invalid model(s)
    ASSERT_THROW(util3d::laserScanFromDepthImages(depthImage, std::vector<CameraModel>(1, cameraModels.front()), 0, 0), UException);
    ASSERT_THROW(util3d::laserScanFromDepthImages(depthImage, std::vector<CameraModel>(1), 0, 0), UException);
    ASSERT_THROW(util3d::laserScanFromDepthImages(depthImage, std::vector<CameraModel>(), 0, 0), UException);

    // invalid/empty images
    ASSERT_THROW(util3d::laserScanFromDepthImages(depthImage.colRange(0,depthImage.cols/2), cameraModels,0,0), UException);
    ASSERT_THROW(util3d::laserScanFromDepthImages(cv::Mat(), cameraModels,0,0), UException);
    ASSERT_THROW(util3d::laserScanFromDepthImages(cv::Mat(), std::vector<CameraModel>(2),0,0), UException);
}

TEST(Util3dTest, laserScanFromPointCloudXYZ) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1.0f, 2.0f, 3.0f));

    LaserScan scan = util3d::laserScanFromPointCloud(cloud);

    EXPECT_EQ(scan.size(), 1);
    pcl::PointXYZ pt = util3d::laserScanToPoint(scan, 0);
    EXPECT_FLOAT_EQ(pt.x, 1.0f);
    EXPECT_FLOAT_EQ(pt.y, 2.0f);
    EXPECT_FLOAT_EQ(pt.z, 3.0f);
}

// Test for pcl::PointXYZRGB
TEST(Util3dTest, laserScanFromPointCloudXYZRGB) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointXYZRGB point;
    point.x = 1.0f; point.y = 2.0f; point.z = 3.0f;
    point.r = 255; point.g = 75; point.b = 120;
    cloud.push_back(point);

    LaserScan scan = util3d::laserScanFromPointCloud(cloud);

    EXPECT_EQ(scan.size(), 1);
    EXPECT_TRUE(scan.hasRGB());
    pcl::PointXYZRGB pt = util3d::laserScanToPointRGB(scan, 0);
    EXPECT_FLOAT_EQ(pt.x, 1.0f);
    EXPECT_FLOAT_EQ(pt.y, 2.0f);
    EXPECT_FLOAT_EQ(pt.z, 3.0f);
    EXPECT_EQ(pt.r, 255);
    EXPECT_EQ(pt.g, 75);
    EXPECT_EQ(pt.b, 120);
}

// Test for pcl::PointNormal
TEST(Util3dTest, laserScanFromPointCloudNormal) {
    pcl::PointCloud<pcl::PointNormal> cloud;
    pcl::PointNormal point;
    point.x = 1.0f; point.y = 2.0f; point.z = 3.0f;
    point.normal_x = 0.0f; point.normal_y = 0.0f; point.normal_z = 1.0f;
    cloud.push_back(point);

    LaserScan scan = util3d::laserScanFromPointCloud(cloud);

    EXPECT_EQ(scan.size(), 1);
    EXPECT_TRUE(scan.hasNormals());
    pcl::PointNormal pt = util3d::laserScanToPointNormal(scan, 0);
    EXPECT_FLOAT_EQ(pt.x, 1.0f);
    EXPECT_FLOAT_EQ(pt.y, 2.0f);
    EXPECT_FLOAT_EQ(pt.z, 3.0f);
    EXPECT_FLOAT_EQ(pt.normal_x, 0.0f);
    EXPECT_FLOAT_EQ(pt.normal_y, 0.0f);
    EXPECT_FLOAT_EQ(pt.normal_z, 1.0f);
}

// Test for pcl::PointXYZI
TEST(Util3dTest, laserScanFromPointCloudXYZI) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI point;
    point.x = 1.0f; point.y = 2.0f; point.z = 3.0f; point.intensity = 100.0f;
    cloud.push_back(point);

    LaserScan scan = util3d::laserScanFromPointCloud(cloud);

    EXPECT_EQ(scan.size(), 1);
    EXPECT_TRUE(scan.hasIntensity());
    pcl::PointXYZI pt = util3d::laserScanToPointI(scan, 0, 0);
    EXPECT_FLOAT_EQ(pt.x, 1.0f);
    EXPECT_FLOAT_EQ(pt.y, 2.0f);
    EXPECT_FLOAT_EQ(pt.z, 3.0f);
    EXPECT_FLOAT_EQ(pt.intensity, 100.0f);
}

// Test for pcl::PointXYZRGBNormal
TEST(Util3dTest, laserScanFromPointCloudXYZRGBNormal) {
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
    pcl::PointXYZRGBNormal point;
    point.x = 1.0f; point.y = 2.0f; point.z = 3.0f;
    point.r = 255; point.g = 75; point.b = 120;
    point.normal_x = 0.0f; point.normal_y = 0.0f; point.normal_z = 1.0f;
    cloud.push_back(point);

    LaserScan scan = util3d::laserScanFromPointCloud(cloud);

    EXPECT_EQ(scan.size(), 1);
    EXPECT_TRUE(scan.hasRGB());
    EXPECT_TRUE(scan.hasNormals());
    pcl::PointXYZRGBNormal pt = util3d::laserScanToPointRGBNormal(scan, 0, 0, 0, 0);
    EXPECT_FLOAT_EQ(pt.x, 1.0f);
    EXPECT_FLOAT_EQ(pt.y, 2.0f);
    EXPECT_FLOAT_EQ(pt.z, 3.0f);
    EXPECT_EQ(pt.r, 255);
    EXPECT_EQ(pt.g, 75);
    EXPECT_EQ(pt.b, 120);
    EXPECT_FLOAT_EQ(pt.normal_x, 0.0f);
    EXPECT_FLOAT_EQ(pt.normal_y, 0.0f);
    EXPECT_FLOAT_EQ(pt.normal_z, 1.0f);
}

// Test for pcl::PointXYZINormal
TEST(Util3dTest, laserScanFromPointCloudXYZINormal) {
    pcl::PointCloud<pcl::PointXYZINormal> cloud;
    pcl::PointXYZINormal point;
    point.x = 1.0f; point.y = 2.0f; point.z = 3.0f;
    point.intensity = 100.0f;
    point.normal_x = 0.0f; point.normal_y = 0.0f; point.normal_z = 1.0f;
    cloud.push_back(point);

    LaserScan scan = util3d::laserScanFromPointCloud(cloud);

    EXPECT_EQ(scan.size(), 1);
    EXPECT_TRUE(scan.hasIntensity());
    EXPECT_TRUE(scan.hasNormals());
    pcl::PointXYZINormal pt = util3d::laserScanToPointINormal(scan, 0, 0);
    EXPECT_FLOAT_EQ(pt.x, 1.0f);
    EXPECT_FLOAT_EQ(pt.y, 2.0f);
    EXPECT_FLOAT_EQ(pt.z, 3.0f);
    EXPECT_FLOAT_EQ(pt.intensity, 100.0f);
    EXPECT_FLOAT_EQ(pt.normal_x, 0.0f);
    EXPECT_FLOAT_EQ(pt.normal_y, 0.0f);
    EXPECT_FLOAT_EQ(pt.normal_z, 1.0f);
}

TEST(Util3dTest, laserScan2dFromPointCloudXYZ) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1.0f, 2.0f, 3.0f));

    LaserScan scan = util3d::laserScan2dFromPointCloud(cloud);

    EXPECT_EQ(scan.size(), 1);
    EXPECT_TRUE(scan.is2d());
    pcl::PointXYZ pt = util3d::laserScanToPoint(scan, 0);
    EXPECT_FLOAT_EQ(pt.x, 1.0f);
    EXPECT_FLOAT_EQ(pt.y, 2.0f);
    EXPECT_FLOAT_EQ(pt.z, 0.0f);
}

// Test for pcl::PointNormal
TEST(Util3dTest, laserScan2dFromPointCloudNormal) {
    pcl::PointCloud<pcl::PointNormal> cloud;
    pcl::PointNormal point;
    point.x = 1.0f; point.y = 2.0f; point.z = 3.0f;
    point.normal_x = 0.0f; point.normal_y = 1.0f; point.normal_z = 0.0f;
    cloud.push_back(point);

    LaserScan scan = util3d::laserScan2dFromPointCloud(cloud);

    EXPECT_EQ(scan.size(), 1);
    EXPECT_TRUE(scan.is2d());
    EXPECT_TRUE(scan.hasNormals());
    pcl::PointNormal pt = util3d::laserScanToPointNormal(scan, 0);
    EXPECT_FLOAT_EQ(pt.x, 1.0f);
    EXPECT_FLOAT_EQ(pt.y, 2.0f);
    EXPECT_FLOAT_EQ(pt.z, 0.0f);
    EXPECT_FLOAT_EQ(pt.normal_x, 0.0f);
    EXPECT_FLOAT_EQ(pt.normal_y, 1.0f);
    EXPECT_FLOAT_EQ(pt.normal_z, 0.0f);
}

// Test for pcl::PointXYZI
TEST(Util3dTest, laserScan2dFromPointCloudXYZI) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI point;
    point.x = 1.0f; point.y = 2.0f; point.z = 3.0f; point.intensity = 100.0f;
    cloud.push_back(point);

    LaserScan scan = util3d::laserScan2dFromPointCloud(cloud);

    EXPECT_EQ(scan.size(), 1);
    EXPECT_TRUE(scan.is2d());
    EXPECT_TRUE(scan.hasIntensity());
    pcl::PointXYZI pt = util3d::laserScanToPointI(scan, 0, 0);
    EXPECT_FLOAT_EQ(pt.x, 1.0f);
    EXPECT_FLOAT_EQ(pt.y, 2.0f);
    EXPECT_FLOAT_EQ(pt.z, 0.0f);
    EXPECT_FLOAT_EQ(pt.intensity, 100.0f);
}

// Test for pcl::PointXYZINormal
TEST(Util3dTest, laserScan2dFromPointCloudXYZINormal) {
    pcl::PointCloud<pcl::PointXYZINormal> cloud;
    pcl::PointXYZINormal point;
    point.x = 1.0f; point.y = 2.0f; point.z = 3.0f;
    point.intensity = 100.0f;
    point.normal_x = 0.0f; point.normal_y = 1.0f; point.normal_z = 0.0f;
    cloud.push_back(point);

    LaserScan scan = util3d::laserScan2dFromPointCloud(cloud);

    EXPECT_EQ(scan.size(), 1);
    EXPECT_TRUE(scan.is2d());
    EXPECT_TRUE(scan.hasIntensity());
    EXPECT_TRUE(scan.hasNormals());
    pcl::PointXYZINormal pt = util3d::laserScanToPointINormal(scan, 0, 0);
    EXPECT_FLOAT_EQ(pt.x, 1.0f);
    EXPECT_FLOAT_EQ(pt.y, 2.0f);
    EXPECT_FLOAT_EQ(pt.z, 0.0f);
    EXPECT_FLOAT_EQ(pt.intensity, 100.0f);
    EXPECT_FLOAT_EQ(pt.normal_x, 0.0f);
    EXPECT_FLOAT_EQ(pt.normal_y, 1.0f);
    EXPECT_FLOAT_EQ(pt.normal_z, 0.0f);
}






TEST(Util3dTest, laserScanToPointCloud)
{
    // XYZ point
    cv::Mat dataXYZ = (cv::Mat_<float>(1, 3) << 1.0f, 2.0f, 3.0f);
    LaserScan scanXYZ = LaserScan(dataXYZ.reshape(3, 1), 1, 0.0f, LaserScan::kXYZ, Transform::getIdentity());

    // XYZ + Normals
    cv::Mat dataNormal = (cv::Mat_<float>(1, 6) << 1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 1.0f);
    LaserScan scanNormal = LaserScan(dataNormal.reshape(6, 1), 1, 0.0f, LaserScan::kXYZNormal, Transform::getIdentity());

    // XYZ + RGB
    cv::Mat dataRGB = (cv::Mat_<float>(1, 4) << 1.0f, 2.0f, 3.0f, LaserScan::packRGB(255,128,64));
    LaserScan scanRGB = LaserScan(dataRGB.reshape(4, 1), 1, 0.0f, LaserScan::kXYZRGB, Transform::getIdentity());

    // XYZ + Intensity
    cv::Mat dataI = (cv::Mat_<float>(1, 4) << 1.0f, 2.0f, 3.0f, 0.5f);
    LaserScan scanI = LaserScan(dataI.reshape(4, 1), 1, 0.0f, LaserScan::kXYZI, Transform::getIdentity());

    // XYZ + RGB + Normals
    cv::Mat dataRGBNormal = (cv::Mat_<float>(1, 7) << 1.0f, 2.0f, 3.0f, LaserScan::packRGB(255,128,64), 0.0f, 1.0f, 0.0f);
    LaserScan scanRGBNormal = LaserScan(dataRGBNormal.reshape(7, 1), 1, 0.0f, LaserScan::kXYZRGBNormal, Transform::getIdentity());

    // XYZ + Intensity + Normals
    cv::Mat dataINormal = (cv::Mat_<float>(1, 7) << 1.0f, 2.0f, 3.0f, 0.75f, 1.0f, 0.0f, 0.0f);
    LaserScan scanINormal = LaserScan(dataINormal.reshape(7, 1), 1, 0.0f, LaserScan::kXYZINormal, Transform::getIdentity());

    {
        pcl::PointXYZ pt = util3d::laserScanToPoint(scanXYZ, 0);
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
        auto cloud = util3d::laserScanToPointCloud(scanXYZ);
        ASSERT_EQ(cloud->size(), 1u);
        pt = cloud->points[0];
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
    }
    {
        pcl::PointNormal pt = util3d::laserScanToPointNormal(scanNormal, 0);
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
        EXPECT_FLOAT_EQ(pt.normal_x, 0.0f);
        EXPECT_FLOAT_EQ(pt.normal_y, 0.0f);
        EXPECT_FLOAT_EQ(pt.normal_z, 1.0f);
        auto cloud = util3d::laserScanToPointCloudNormal(scanNormal);
        ASSERT_EQ(cloud->size(), 1u);
        pt = cloud->points[0];
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
        EXPECT_FLOAT_EQ(pt.normal_x, 0.0f);
        EXPECT_FLOAT_EQ(pt.normal_y, 0.0f);
        EXPECT_FLOAT_EQ(pt.normal_z, 1.0f);
    }
    {
        pcl::PointXYZRGB pt = util3d::laserScanToPointRGB(scanRGB, 0);
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
        EXPECT_EQ(pt.r, 255);
        EXPECT_EQ(pt.g, 128);
        EXPECT_EQ(pt.b, 64);
        auto cloud = util3d::laserScanToPointCloudRGB(scanRGB);
        ASSERT_EQ(cloud->size(), 1u);
        pt = cloud->points[0];
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
        EXPECT_EQ(pt.r, 255);
        EXPECT_EQ(pt.g, 128);
        EXPECT_EQ(pt.b, 64);
    }
    {
        pcl::PointXYZI pt = util3d::laserScanToPointI(scanI, 0, 0.5f);
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
        EXPECT_FLOAT_EQ(pt.intensity, 0.5f);
        auto cloud = util3d::laserScanToPointCloudI(scanI);
        ASSERT_EQ(cloud->size(), 1u);
        pt = cloud->points[0];
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
        EXPECT_FLOAT_EQ(pt.intensity, 0.5f);
    }
    {
        pcl::PointXYZRGBNormal pt = util3d::laserScanToPointRGBNormal(scanRGBNormal, 0, 100, 100, 100);
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
        EXPECT_EQ(pt.r, 255);
        EXPECT_EQ(pt.g, 128);
        EXPECT_EQ(pt.b, 64);
        EXPECT_FLOAT_EQ(pt.normal_x, 0.0f);
        EXPECT_FLOAT_EQ(pt.normal_y, 1.0f);
        EXPECT_FLOAT_EQ(pt.normal_z, 0.0f);
        auto cloud = util3d::laserScanToPointCloudRGBNormal(scanRGBNormal);
        ASSERT_EQ(cloud->size(), 1u);
        pt = cloud->points[0];
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
        EXPECT_EQ(pt.r, 255);
        EXPECT_EQ(pt.g, 128);
        EXPECT_EQ(pt.b, 64);
        EXPECT_FLOAT_EQ(pt.normal_x, 0.0f);
        EXPECT_FLOAT_EQ(pt.normal_y, 1.0f);
        EXPECT_FLOAT_EQ(pt.normal_z, 0.0f);
    }
    {
        pcl::PointXYZINormal pt = util3d::laserScanToPointINormal(scanINormal, 0, 0.75f);
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
        EXPECT_FLOAT_EQ(pt.intensity, 0.75f);
        EXPECT_FLOAT_EQ(pt.normal_x, 1.0f);
        EXPECT_FLOAT_EQ(pt.normal_y, 0.0f);
        EXPECT_FLOAT_EQ(pt.normal_z, 0.0f);
        auto cloud = util3d::laserScanToPointCloudINormal(scanINormal);
        ASSERT_EQ(cloud->size(), 1u);
        pt = cloud->points[0];
        EXPECT_FLOAT_EQ(pt.x, 1.0f);
        EXPECT_FLOAT_EQ(pt.y, 2.0f);
        EXPECT_FLOAT_EQ(pt.z, 3.0f);
        EXPECT_FLOAT_EQ(pt.intensity, 0.75f);
        EXPECT_FLOAT_EQ(pt.normal_x, 1.0f);
        EXPECT_FLOAT_EQ(pt.normal_y, 0.0f);
        EXPECT_FLOAT_EQ(pt.normal_z, 0.0f);
    }
}

TEST(Util3dTest, getMinMax3D) {
    // Create a 3x3 CV matrix of type CV_32FC3 (3D points)
    cv::Mat laserScan(1, 3, CV_32FC3);
    float data[9] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, -1.0f, -2.0f, -3.0f};
    memcpy(laserScan.data, data, sizeof(data));

    {
        // cv::Point3f
        cv::Point3f min, max;
        util3d::getMinMax3D(laserScan, min, max);

        // Test the expected min and max values
        EXPECT_FLOAT_EQ(min.x, -1.0f);
        EXPECT_FLOAT_EQ(min.y, -2.0f);
        EXPECT_FLOAT_EQ(min.z, -3.0f);
        EXPECT_FLOAT_EQ(max.x, 4.0f);
        EXPECT_FLOAT_EQ(max.y, 5.0f);
        EXPECT_FLOAT_EQ(max.z, 6.0f);
    }

    // pcl::PointXYZ
    {
        pcl::PointXYZ min, max;
        util3d::getMinMax3D(laserScan, min, max);

        // Test the expected min and max values
        EXPECT_FLOAT_EQ(min.x, -1.0f);
        EXPECT_FLOAT_EQ(min.y, -2.0f);
        EXPECT_FLOAT_EQ(min.z, -3.0f);
        EXPECT_FLOAT_EQ(max.x, 4.0f);
        EXPECT_FLOAT_EQ(max.y, 5.0f);
        EXPECT_FLOAT_EQ(max.z, 6.0f);
    }
}

// Test with a laser scan that contains only 2D data (no Z values)
TEST(Util3dTest, getMinMax3DCVPoint3f_2DData) {
    // Create a 2x3 CV matrix of type CV_32FC2 (2D points)
    cv::Mat laserScan(1, 2, CV_32FC2);
    float data[6] = {1.0f, 2.0f, 4.0f, 5.0f};  // No Z values
    memcpy(laserScan.data, data, sizeof(data));

    cv::Point3f min, max;

    // Call the function
    util3d::getMinMax3D(laserScan, min, max);

    // Test the expected min and max values
    EXPECT_FLOAT_EQ(min.x, 1.0f);
    EXPECT_FLOAT_EQ(min.y, 2.0f);
    EXPECT_FLOAT_EQ(min.z, 0.0f);  // Z is set to 0
    EXPECT_FLOAT_EQ(max.x, 4.0f);
    EXPECT_FLOAT_EQ(max.y, 5.0f);
    EXPECT_FLOAT_EQ(max.z, 0.0f);  // Z is set to 0
}

// Test with a matrix where all points are the same (edge case)
TEST(Util3dTest, getMinMax3DSamePoints) {
    // Create a 1x3 CV matrix of type CV_32FC3 where all points are the same
    cv::Mat laserScan(1, 3, CV_32FC3);
    float data[9] = {2.0f, 3.0f, 4.0f, 2.0f, 3.0f, 4.0f, 2.0f, 3.0f, 4.0f};
    memcpy(laserScan.data, data, sizeof(data));

    cv::Point3f min, max;

    // Call the function
    util3d::getMinMax3D(laserScan, min, max);

    // Test that min and max are the same for all points
    EXPECT_FLOAT_EQ(min.x, 2.0f);
    EXPECT_FLOAT_EQ(min.y, 3.0f);
    EXPECT_FLOAT_EQ(min.z, 4.0f);
    EXPECT_FLOAT_EQ(max.x, 2.0f);
    EXPECT_FLOAT_EQ(max.y, 3.0f);
    EXPECT_FLOAT_EQ(max.z, 4.0f);
}

// Test with an empty laser scan (should trigger an error or assertion)
TEST(Util3dTest, getMinMax3DEmptyLaserScan) {
    // Create an empty CV matrix
    cv::Mat laserScan;

    cv::Point3f min, max;

    // Expect the function to assert or throw an exception
    EXPECT_THROW(util3d::getMinMax3D(laserScan, min, max), UException);
}

// Test for projectDisparityTo3D with valid disparity
TEST(Util3dTest, projectDisparityTo3DValidDisparity) {
    StereoCameraModel model(525, 525, 319.5f, 219.5f, 0.05f, CameraModel::opticalRotation(), cv::Size(640,480));
    
    // Define a 2D point (u, v) in the left image
    cv::Point2f pt(100.0f, 150.0f);
    float disparity = 10.0f; // Example disparity value in pixels

    // Call the function
    cv::Point3f result = util3d::projectDisparityTo3D(pt, disparity, model);

    // Check that the result is not NaN and the z value is greater than 0 (since disparity > 0)
    ASSERT_FALSE(std::isnan(result.x));
    ASSERT_FALSE(std::isnan(result.y));
    ASSERT_FALSE(std::isnan(result.z));
    ASSERT_GT(result.z, 0.0f);
}

// Test for projectDisparityTo3D with invalid disparity (0.0f)
TEST(Util3dTest, projectDisparityTo3DInvalidDisparityZero) {
    StereoCameraModel model(525, 525, 319.5f, 219.5f, 0.05f, CameraModel::opticalRotation(), cv::Size(640,480));
    
    // Define a 2D point (u, v) in the left image
    cv::Point2f pt(100.0f, 150.0f);
    float disparity = 0.0f; // Invalid disparity value (disparity can't be 0)

    // Call the function
    cv::Point3f result = util3d::projectDisparityTo3D(pt, disparity, model);

    // Check that the result is NaN
    ASSERT_TRUE(std::isnan(result.x));
    ASSERT_TRUE(std::isnan(result.y));
    ASSERT_TRUE(std::isnan(result.z));
}

// Test for projectDisparityTo3D with disparity map (CV_32FC1 type)
TEST(Util3dTest, projectDisparityTo3DDisparityMapValid) {
    StereoCameraModel model(525, 525, 319.5f, 219.5f, 0.05f, CameraModel::opticalRotation(), cv::Size(640,480));

    // Create a sample disparity map (1 channel, float type)
    cv::Mat disparity_map(480, 640, CV_32FC1, cv::Scalar(2.0f)); // All pixels have a disparity of 2

    // Define a point in the left image (u, v)
    cv::Point2f pt(320, 240);  // Test center of the disparity map

    // Call the function
    cv::Point3f result = util3d::projectDisparityTo3D(pt, disparity_map, model);

    // Check that the result is not NaN and has a valid z value
    ASSERT_FALSE(std::isnan(result.x));
    ASSERT_FALSE(std::isnan(result.y));
    ASSERT_FALSE(std::isnan(result.z));
    ASSERT_GT(result.z, 0.0f);
}

// Test for projectDisparityTo3D with disparity map out of bounds (invalid pixel)
TEST(Util3dTest, projectDisparityTo3DDisparityMapOutOfBounds) {
    StereoCameraModel model(525, 525, 319.5f, 219.5f, 0.05f, CameraModel::opticalRotation(), cv::Size(640,480));

    // Create a sample disparity map (1 channel, float type)
    cv::Mat disparity_map(480, 640, CV_32FC1, cv::Scalar(2.f)); // All pixels have a disparity of 2

    // Define a point outside the disparity map (invalid u, v)
    cv::Point2f pt(720.0f, 20.0f);  // Point is out of bounds

    // Call the function
    cv::Point3f result = util3d::projectDisparityTo3D(pt, disparity_map, model);

    // Check that the result is NaN since the point is out of bounds
    ASSERT_TRUE(std::isnan(result.x));
    ASSERT_TRUE(std::isnan(result.y));
    ASSERT_TRUE(std::isnan(result.z));
}

// Test for projectDisparityTo3D with invalid disparity map type
TEST(Util3dTest, projectDisparityTo3DInvalidDisparityMapType) {
    StereoCameraModel model(525, 525, 319.5f, 219.5f, 0.05f, CameraModel::opticalRotation(), cv::Size(640,480));

    // Create a sample disparity map with invalid type (CV_8UC1)
    cv::Mat disparity_map(480, 640, CV_8UC1, cv::Scalar(2)); // Invalid type (should be CV_32FC1 or CV_16SC1)

    // Define a point in the left image (u, v)
    cv::Point2f pt(5.0f, 5.0f);  // Test center of the disparity map

    // Call the function
    ASSERT_THROW(util3d::projectDisparityTo3D(pt, disparity_map, model), UException);
}

TEST(Util3dTest, projectCloudToCameraCvMatLaserScan) {
    // Mock image size
    cv::Size imageSize(640, 480);

    // Mock camera matrix (fx, fy, cx, cy)
    cv::Mat cameraMatrixK = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);

    // Create mock laser scan (CV_32FC2)
    cv::Mat laserScan = cv::Mat::zeros(1, 640, CV_32FC2);  // 2D points
    for (int i = 0; i < 640; ++i) {
        laserScan.at<cv::Vec2f>(0, i) = cv::Vec2f(10, (i-640/2)*0.1);  // Mock some points
    }

    Transform cameraTransform = CameraModel::opticalRotation();

    // Call the function, normally we should retrieve a depth image with depth values only on the middle row
    cv::Mat result = util3d::projectCloudToCamera(imageSize, cameraMatrixK, laserScan, cameraTransform);

    // Test the result (checking the type and size of the result)
    EXPECT_EQ(result.type(), CV_32FC1);
    EXPECT_EQ(result.size(), imageSize);
    EXPECT_EQ(cv::countNonZero(result.rowRange(0,480/2-1)), 0);
    EXPECT_EQ(cv::countNonZero(result.rowRange(480/2+1,480)), 0);
    EXPECT_GT(cv::countNonZero(result.rowRange(480/2-1, 480/2+1)), 0);
    cv::Mat mask = result != 0;
    EXPECT_EQ(cv::mean(result, mask), cv::Scalar(10));
}

// Test for projectCloudToCamera with pcl::PointCloud<pcl::PointXYZ>
TEST(Util3dTest, projectCloudToCameraPclPointCloud) {
    // Mock image size
    cv::Size imageSize(640, 480);

    // Mock camera matrix (fx, fy, cx, cy)
    cv::Mat cameraMatrixK = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);

    // Create mock pcl::PointCloud<pcl::PointXYZ> 
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserScan(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < 640; ++i) {
        pcl::PointXYZ point(10, (i-640/2)*0.1, 0);  // horizontal line
        laserScan->points.push_back(point);
        point = pcl::PointXYZ(10, 0, (i-640/2)*0.1);  // vertical line
        laserScan->points.push_back(point);
    }

    Transform cameraTransform = CameraModel::opticalRotation();

    // Call the function
    cv::Mat result = util3d::projectCloudToCamera(imageSize, cameraMatrixK, laserScan, cameraTransform);

    // Test the result (checking the type and size of the result)
    EXPECT_EQ(result.type(), CV_32FC1);
    EXPECT_EQ(result.size(), imageSize);
    EXPECT_EQ(cv::countNonZero(result(cv::Range(0,480/2-1), cv::Range(0,640/2-1))), 0); // top-left quadrant
    EXPECT_EQ(cv::countNonZero(result(cv::Range(0,480/2-1), cv::Range(640/2+1, 640))), 0); // top-right quadrant
    EXPECT_EQ(cv::countNonZero(result(cv::Range(480/2+1, 480), cv::Range(0,640/2-1))), 0); // bottom-left quadrant
    EXPECT_EQ(cv::countNonZero(result(cv::Range(480/2+1, 480), cv::Range(640/2+1, 640))), 0); // bottom-right quadrant
    EXPECT_GT(cv::countNonZero(result.colRange(640/2-1,640/2+1)), 0); // vertical line
    EXPECT_GT(cv::countNonZero(result.rowRange(480/2-1, 480/2+1)), 0); // horizontal line
    cv::Mat mask = result != 0;
    EXPECT_EQ(cv::mean(result, mask), cv::Scalar(10));
}

// Test for projectCloudToCamera with pcl::PCLPointCloud2
TEST(Util3dTest, projectCloudToCameraPCLPointCloud2) {
    // Mock image size
    cv::Size imageSize(640, 480);

    // Mock camera matrix (fx, fy, cx, cy)
    cv::Mat cameraMatrixK = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);

    // Create mock pcl::PCLPointCloud2 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < 640; ++i) {
        pcl::PointXYZ point(10, (i-640/2)*0.1, 0);  // horizontal line
        cloud->points.push_back(point);
        point = pcl::PointXYZ(10, 0, (i-640/2)*0.1);  // vertical line
        cloud->points.push_back(point);
    }
    pcl::PCLPointCloud2::Ptr laserScan(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud, *laserScan);

    Transform cameraTransform = CameraModel::opticalRotation();

    // Call the function
    cv::Mat result = util3d::projectCloudToCamera(imageSize, cameraMatrixK, laserScan, cameraTransform);

    // Test the result (checking the type and size of the result)
    EXPECT_EQ(result.type(), CV_32FC1);
    EXPECT_EQ(result.size(), imageSize);
    EXPECT_EQ(cv::countNonZero(result(cv::Range(0,480/2-1), cv::Range(0,640/2-1))), 0); // top-left quadrant
    EXPECT_EQ(cv::countNonZero(result(cv::Range(0,480/2-1), cv::Range(640/2+1, 640))), 0); // top-right quadrant
    EXPECT_EQ(cv::countNonZero(result(cv::Range(480/2+1, 480), cv::Range(0,640/2-1))), 0); // bottom-left quadrant
    EXPECT_EQ(cv::countNonZero(result(cv::Range(480/2+1, 480), cv::Range(640/2+1, 640))), 0); // bottom-right quadrant
    EXPECT_GT(cv::countNonZero(result.colRange(640/2-1,640/2+1)), 0); // vertical line
    EXPECT_GT(cv::countNonZero(result.rowRange(480/2-1, 480/2+1)), 0); // horizontal line
    cv::Mat mask = result != 0;
    EXPECT_EQ(cv::mean(result, mask), cv::Scalar(10));
}

// Helper function to compare matrices
bool compareMatrices(const cv::Mat& mat1, const cv::Mat& mat2, float tolerance = 1e-5) {
    if (mat1.size() != mat2.size() || mat1.type() != mat2.type()) {
        return false;
    }
    for (int i = 0; i < mat1.rows; ++i) {
        for (int j = 0; j < mat1.cols; ++j) {
            if (std::abs(mat1.at<float>(i, j) - mat2.at<float>(i, j)) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

// Helper function to create a sample registered depth image (for testing)
cv::Mat createTestDepthImage(int rows, int cols) {
    cv::Mat depthImage = cv::Mat::zeros(rows, cols, CV_32FC1);
    // Manually set some non-zero values to simulate depth points
    depthImage.at<float>(2, 3) = 1.0f;
    depthImage.at<float>(5, 4) = 2.0f;
    depthImage.at<float>(7, 7) = 1.5f;
    return depthImage;
}

TEST(Util3dTest, fillProjectedCloudHolesFillTest) {
    cv::Mat depthImage = cv::Mat::zeros(10, 10, CV_32FC1);
    depthImage.at<float>(2, 3) = 10.0f;
    depthImage.at<float>(4, 3) = 10.0f;
    depthImage.at<float>(2, 5) = 10.0f;

    cv::Mat original = depthImage.clone();

    // Call the function to fill the holes vertically
    util3d::fillProjectedCloudHoles(depthImage, true, false);

    EXPECT_FLOAT_EQ(depthImage.at<float>(3, 3), 10.0f);
    EXPECT_FLOAT_EQ(depthImage.at<float>(2, 4), 0.0f);

    // Call the function to fill the holes horizontally
    depthImage = original.clone();
    util3d::fillProjectedCloudHoles(depthImage, false, false); 
    EXPECT_FLOAT_EQ(depthImage.at<float>(3, 3), 0.0f);
    EXPECT_FLOAT_EQ(depthImage.at<float>(2, 4), 10.0f);

    // Call the function to fill to border vertically
    depthImage = original.clone();
    util3d::fillProjectedCloudHoles(depthImage, true, true);
    EXPECT_EQ(cv::countNonZero(depthImage.colRange(0, 3)), 0);
    EXPECT_EQ(cv::countNonZero(depthImage.colRange(3, 4)), 8); // current implementation only fills up to last pixel
    EXPECT_EQ(cv::countNonZero(depthImage.colRange(4, 5)), 0);
    EXPECT_EQ(cv::countNonZero(depthImage.colRange(5, 6)), 8);
    EXPECT_EQ(cv::countNonZero(depthImage.colRange(6, 10)), 0);

    // Call the function to fill to border horizontally
    depthImage = original.clone();
    util3d::fillProjectedCloudHoles(depthImage, false, true);
    EXPECT_EQ(cv::countNonZero(depthImage.rowRange(0, 2)), 0);
    EXPECT_EQ(cv::countNonZero(depthImage.rowRange(2, 3)), 8);
    EXPECT_EQ(cv::countNonZero(depthImage.rowRange(3, 4)), 0);
    EXPECT_EQ(cv::countNonZero(depthImage.rowRange(4, 5)), 8);
    EXPECT_EQ(cv::countNonZero(depthImage.rowRange(5, 10)), 0);

    //Test case with no holes in the depth image (nothing should change).
    depthImage = cv::Mat::ones(10, 10, CV_32FC1);  // No holes, all values are 1.0f

    // Call the function to fill the holes (shouldn't change anything)
    util3d::fillProjectedCloudHoles(depthImage, true, true);  // Fill to border

    // Assert that the image hasn't changed
    ASSERT_EQ(cv::mean(depthImage), cv::Scalar(1.0f));
}

TEST(Util3dTest, filterFloorThreshold) {
    // Setup: Create a synthetic depth image and CameraModel
    cv::Mat depth = cv::Mat::zeros(4, 4, CV_16UC1);
    depth.at<unsigned short>(1, 1) = 1000;
    depth.at<unsigned short>(2, 2) = 500;
    depth.at<unsigned short>(3, 3) = 1500;

    // Camera Model: camera is 75 cm over the ground, looking directly down
    std::vector<CameraModel> cameraModels;
    cameraModels.push_back(CameraModel(500.0f, 500.0f, 320.0f, 240.0f, 
        Transform(0,0,0.75, 0,M_PI/2,0)*CameraModel::opticalRotation(), 0, cv::Size(640, 480)));

    // Test case where threshold is 10 cm
    float threshold = 0.1f;  // In meters

    cv::Mat depthBelow;

    // Call the filterFloor function
    cv::Mat filteredDepth = util3d::filterFloor(depth, cameraModels, threshold, &depthBelow);

    // Check that depthBelow contains the points below the threshold
    EXPECT_EQ(depthBelow.at<unsigned short>(1, 1), 1000);
    EXPECT_EQ(depthBelow.at<unsigned short>(2, 2), 0);
    EXPECT_EQ(depthBelow.at<unsigned short>(3, 3), 1500);

    // Check that depth kept points over the threshold
    EXPECT_EQ(filteredDepth.at<unsigned short>(1, 1), 0);
    EXPECT_EQ(filteredDepth.at<unsigned short>(2, 2), 500);
    EXPECT_EQ(filteredDepth.at<unsigned short>(3, 3), 0);
}

TEST(Util3dTest, filterFloorEmptyDepthImage) {
    // Test case with empty depth image
    cv::Mat emptyDepth = cv::Mat::zeros(0, 0, CV_16UC1);
    std::vector<CameraModel> cameraModels;
    cameraModels.push_back(CameraModel(500.0f, 500.0f, 320.0f, 240.0f, CameraModel::opticalRotation(), 0, cv::Size(640, 480)));

    cv::Mat depthBelow;
    cv::Mat filteredDepth = util3d::filterFloor(emptyDepth, cameraModels, 1.0f, &depthBelow);

    // Expect empty matrices for both filteredDepth and depthBelow
    EXPECT_TRUE(filteredDepth.empty());
    EXPECT_TRUE(depthBelow.empty());
}

TEST(Util3dTest, projectCloudToCameras) {

    // Create mock point cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
    pcl::PointXYZRGBNormal pt1;
    pt1.x = 1.0f;
    pt1.y = 0.0f;
    pt1.z = 0.0f;
    pt1.normal_x = 0.0f;
    pt1.normal_y = 0.0f;
    pt1.normal_z = 1.0f;  // Normal pointing upwards
    cloud.push_back(pt1);

    pcl::PointXYZRGBNormal pt2;
    pt2.x = 1.0f;
    pt2.y = 0.05f;
    pt2.z = 0.0f;
    pt2.normal_x = -1.0f; // Normal pointing backwards
    pt2.normal_y = 0.0f;
    pt2.normal_z = 0.0f;
    cloud.push_back(pt2);

    pcl::PointXYZRGBNormal pt3;
    pt3.x = 0.5f;
    pt3.y = 1.0f;
    pt3.z = 0.0f;
    pt3.normal_x = 0.0f; 
    pt3.normal_y = -1.0f; // Normal pointing right
    pt3.normal_z = 0.0f;
    cloud.push_back(pt3);

    pcl::PointXYZRGBNormal pt4;
    pt4.x = 1.0f;
    pt4.y = 0.0f;
    pt4.z = -0.025f; // below pt1 by 2.5 cm
    pt4.normal_x = 0.0f;
    pt4.normal_y = 0.0f;
    pt4.normal_z = 1.0f;  // Normal pointing upwards
    cloud.push_back(pt4);

    pcl::PointXYZRGBNormal pt5;
    pt5.x = 1.0f;
    pt5.y = 0.0f;
    pt5.z = -0.1f;   // below pt1 by 10 cm
    pt5.normal_x = 0.0f;
    pt5.normal_y = 0.0f;
    pt5.normal_z = 1.0f;  // Normal pointing upwards
    cloud.push_back(pt5);

    std::map<int, Transform> cameraPoses;
    cameraPoses[1] = Transform(0.5f, 0.0f, 0.0f,0,0,0); // Camera 1 looking at pt2, but closer to pt1 than camera 2
    cameraPoses[2] = Transform(1.0f, 0.0f, 1.0f,0,M_PI/2,0); // Camera 2 looking at pt1 (looking down)
    
    std::map<int, std::vector<CameraModel>> cameraModels;
    CameraModel model(500, 500, 319.5f, 239.5f, CameraModel::opticalRotation(), 0, cv::Size(640,480));
    cameraModels[1].push_back(model);
    cameraModels[2].push_back(model);

    model.setLocalTransform(Transform(0,0,0,0,0,M_PI/2)*CameraModel::opticalRotation()); // this view from camera 1 position is looking left (only pt3 in FOV)
    cameraModels[1].push_back(model);

    // Set parameters for projection
    float maxDistance = 10.0f;
    float maxAngle = 45.0f * M_PI/ 180.0f;
    float maxDepthError = 0.05f; // For camera 1, it should see pt1 and pt4, but not pt5
    std::vector<float> roiRatios = {0.0f, 0.0f, 0.0f, 0.0f};  // Full image ROI
    cv::Mat projMask = cv::Mat::ones(480, 640, CV_8UC1);  // Projection mask (all valid)
    bool distanceToCamPolicy = true;
    ProgressState* state = nullptr;  // Not using progress state in this test

    ULogger::setLevel(ULogger::kDebug);
    ULogger::setType(ULogger::kTypeConsole);

    // Call the function to test
    auto result = util3d::projectCloudToCameras(cloud, cameraPoses, cameraModels, maxDistance, maxAngle, maxDepthError, roiRatios, projMask, distanceToCamPolicy, state);

    // Validate the result
    ASSERT_EQ(result.size(), cloud.size());  // The result should have the same size as the input point cloud

    EXPECT_EQ(result[0].first.first, 2);  // Camera node ID
    EXPECT_EQ(result[0].first.second, 0);  // Camera index
    EXPECT_NEAR(result[0].second.x, 0.5f, 0.1f);  // UV x-coordinate, close to the center
    EXPECT_NEAR(result[0].second.y, 0.5f, 0.1f);  // UV y-coordinate, close to the center

    EXPECT_EQ(result[1].first.first, 1);  // Camera node ID
    EXPECT_EQ(result[1].first.second, 0);  // Camera index
    EXPECT_NEAR(result[1].second.x, 0.5f, 0.1f);  // UV x-coordinate, close to the center
    EXPECT_NEAR(result[1].second.y, 0.5f, 0.1f);  // UV y-coordinate, close to the center

    EXPECT_EQ(result[2].first.first, 1);  // Camera node ID
    EXPECT_EQ(result[2].first.second, 1);  // Camera index
    EXPECT_NEAR(result[2].second.x, 0.5f, 0.1f);  // UV x-coordinate, close to the center
    EXPECT_NEAR(result[2].second.y, 0.5f, 0.1f);  // UV y-coordinate, close to the center

    EXPECT_EQ(result[3].first.first, 2);  // Camera node ID
    EXPECT_EQ(result[3].first.second, 0);  // Camera index
    EXPECT_NEAR(result[3].second.x, 0.5f, 0.1f);  // UV x-coordinate, close to the center
    EXPECT_NEAR(result[3].second.y, 0.5f, 0.1f);  // UV y-coordinate, close to the center

    EXPECT_EQ(result[4].first.first, 0);  // Camera node ID (not found = 0)
}

TEST(Util3dTest, isFinite) {
    cv::Point3f pt1(1.0f, 2.0f, 3.0f);
    EXPECT_TRUE(util3d::isFinite(pt1));

    cv::Point3f pt2(NAN, 2.0f, 3.0f);
    EXPECT_FALSE(util3d::isFinite(pt2));

    cv::Point3f pt3(1.0f, INFINITY, 3.0f);
    EXPECT_FALSE(util3d::isFinite(pt3));

    cv::Point3f pt4(NAN, -INFINITY, INFINITY);
    EXPECT_FALSE(util3d::isFinite(pt4));
}

TEST(Util3dTest, concatenateCloudsXYZ)
{
    std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudList;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
    cloud1->push_back(pcl::PointXYZ(1, 2, 3));
    cloud1->push_back(pcl::PointXYZ(4, 5, 6));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    cloud2->push_back(pcl::PointXYZ(7, 8, 9));

    cloudList.push_back(cloud1);
    cloudList.push_back(cloud2);

    auto result = util3d::concatenateClouds(cloudList);

    ASSERT_EQ(result->size(), 3);
    EXPECT_EQ(result->at(0).x, 1);
    EXPECT_EQ(result->at(2).z, 9);
}

TEST(Util3dTest, concatenateCloudsXYZRGB)
{
    std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudList;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointXYZRGB pt1; pt1.x = 1; pt1.y = 1; pt1.z = 1; pt1.r = 255; pt1.g = 0; pt1.b = 0;
    pcl::PointXYZRGB pt2; pt2.x = 2; pt2.y = 2; pt2.z = 2; pt2.r = 0; pt2.g = 255; pt2.b = 0;
    cloud1->push_back(pt1);
    cloud1->push_back(pt2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointXYZRGB pt3; pt3.x = 3; pt3.y = 3; pt3.z = 3; pt3.r = 0; pt3.g = 0; pt3.b = 255;
    cloud2->push_back(pt3);

    cloudList.push_back(cloud1);
    cloudList.push_back(cloud2);

    auto result = util3d::concatenateClouds(cloudList);

    ASSERT_EQ(result->size(), 3);
    EXPECT_EQ(result->at(0).r, 255);
    EXPECT_EQ(result->at(1).g, 255);
    EXPECT_EQ(result->at(2).b, 255);
}


TEST(Util3dTest, concatenateIndices)
{
    pcl::IndicesPtr indices1(new std::vector<int>({1, 2}));
    pcl::IndicesPtr indices2(new std::vector<int>({3, 4}));
    pcl::IndicesPtr indices3(new std::vector<int>({5}));

    std::vector<pcl::IndicesPtr> inputs = {indices1, indices2, indices3};
    pcl::IndicesPtr result = util3d::concatenate(inputs);

    ASSERT_EQ(result->size(), 5u);
    EXPECT_EQ((*result)[0], 1);
    EXPECT_EQ((*result)[1], 2);
    EXPECT_EQ((*result)[2], 3);
    EXPECT_EQ((*result)[3], 4);
    EXPECT_EQ((*result)[4], 5);

    pcl::IndicesPtr indicesA(new std::vector<int>({10, 20}));
    pcl::IndicesPtr indicesB(new std::vector<int>({30}));

    result = util3d::concatenate(indicesA, indicesB);

    ASSERT_EQ(result->size(), 3u);
    EXPECT_EQ((*result)[0], 10);
    EXPECT_EQ((*result)[1], 20);
    EXPECT_EQ((*result)[2], 30);
}

TEST(Util3dTest, savePCDWordsFromPCLPoints) {
	std::multimap<int, pcl::PointXYZ> words;
	words.insert({0, pcl::PointXYZ(1, 2, 3)});
	words.insert({1, pcl::PointXYZ(4, 5, 6)});
	Transform t = Transform::getIdentity();

	std::string file = "test_pcd_pcl.pcd";
	util3d::savePCDWords(file, words, t);

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::io::loadPCDFile(file, cloud);

	EXPECT_EQ(cloud.size(), 2);
	EXPECT_FLOAT_EQ(cloud[0].x, 1);
	EXPECT_FLOAT_EQ(cloud[1].z, 6);
	std::remove(file.c_str());
}

TEST(Util3dTest, savePCDWordsFromCVPoints) {
	std::multimap<int, cv::Point3f> words;
	words.insert({0, cv::Point3f(1, 0, 0)});
	words.insert({1, cv::Point3f(0, 1, 0)});
	Transform t = Transform(1, 0, 0, 1,
	                        0, 1, 0, 2,
	                        0, 0, 1, 3); // Translate

	std::string file = "test_pcd_cv.pcd";
	util3d::savePCDWords(file, words, t);

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::io::loadPCDFile(file, cloud);

	EXPECT_EQ(cloud.size(), 2);
	EXPECT_FLOAT_EQ(cloud[0].x, 2); // 1 + tx
	EXPECT_FLOAT_EQ(cloud[1].y, 3); // 1 + ty
	std::remove(file.c_str());
}

TEST(Util3dTest, loadBINScanValid) {
    std::string tmp_file = "test.bin";
    std::ofstream file(tmp_file, std::ios::binary);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.resize(5);
    for (int i = 0; i < 5; ++i) {
        float x = static_cast<float>(rand()) / RAND_MAX;
        float y = static_cast<float>(rand()) / RAND_MAX;
        float z = static_cast<float>(rand()) / RAND_MAX;
        float intensity = static_cast<float>(rand()) / RAND_MAX;
        file.write(reinterpret_cast<char*>(&x), sizeof(float));
        file.write(reinterpret_cast<char*>(&y), sizeof(float));
        file.write(reinterpret_cast<char*>(&z), sizeof(float));
        file.write(reinterpret_cast<char*>(&intensity), sizeof(float));
        cloud[i].x = x;
        cloud[i].y = y;
        cloud[i].z = z;
        cloud[i].intensity = intensity;
    }
    file.close();

    cv::Mat result = util3d::loadBINScan(tmp_file);

    // Check if the matrix dimensions are correct (1 x 5 x 4)
    EXPECT_EQ(result.rows, 1);
    EXPECT_EQ(result.cols, cloud.size());
    EXPECT_EQ(result.channels(), 4);
    EXPECT_EQ(result.type(), CV_32FC4);

    for(size_t i=0; i<cloud.size(); ++i)
    {
        EXPECT_EQ(cloud[i].x, result.at<cv::Vec4f>(0, i)[0]);
        EXPECT_EQ(cloud[i].y, result.at<cv::Vec4f>(0, i)[1]);
        EXPECT_EQ(cloud[i].z, result.at<cv::Vec4f>(0, i)[2]);
        EXPECT_EQ(cloud[i].intensity, result.at<cv::Vec4f>(0, i)[3]);
    }

    std::remove(tmp_file.c_str());  // Clean up the test file
}

TEST(Util3dTest, loadBINScanEmpty) {
    std::string tmp_file = "empty_test.bin";
    std::ofstream(tmp_file, std::ios::binary).close();  // Create empty file

    cv::Mat result = util3d::loadBINScan(tmp_file);

    // Ensure the output is empty
    EXPECT_TRUE(result.empty());

    std::remove(tmp_file.c_str());  // Clean up the test file
}

// Test case to check loading a binary scan
TEST(Util3dTest, loadScanBINFile) {
    std::string tmp_file = "test.bin";
    std::ofstream file(tmp_file, std::ios::binary);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.resize(5);
    for (int i = 0; i < 5; ++i) {
        float x = static_cast<float>(rand()) / RAND_MAX;
        float y = static_cast<float>(rand()) / RAND_MAX;
        float z = static_cast<float>(rand()) / RAND_MAX;
        float intensity = static_cast<float>(rand()) / RAND_MAX;
        file.write(reinterpret_cast<char*>(&x), sizeof(float));
        file.write(reinterpret_cast<char*>(&y), sizeof(float));
        file.write(reinterpret_cast<char*>(&z), sizeof(float));
        file.write(reinterpret_cast<char*>(&intensity), sizeof(float));
        cloud[i].x = x;
        cloud[i].y = y;
        cloud[i].z = z;
        cloud[i].intensity = intensity;
    }
    file.close();

    LaserScan result = util3d::loadScan(tmp_file);

    // Check that result is valid and has expected properties
    EXPECT_TRUE(!result.empty());  
    EXPECT_EQ(result.data().rows, 1);
    EXPECT_EQ(result.data().cols, cloud.size());
    EXPECT_EQ(result.data().channels(), 4);
    EXPECT_EQ(result.data().type(), CV_32FC4);
    EXPECT_EQ(result.format(), LaserScan::kXYZI);

    for(size_t i=0; i<cloud.size(); ++i)
    {
        EXPECT_EQ(cloud[i].x, result.field(i, 0));
        EXPECT_EQ(cloud[i].y, result.field(i, 1));
        EXPECT_EQ(cloud[i].z, result.field(i, 2));
        EXPECT_EQ(cloud[i].intensity, result.field(i, result.getIntensityOffset()));
    }

    std::remove(tmp_file.c_str());  // Clean up the test file
}

// Test case to check loading a PCD file
TEST(Util3dTest, loadScanPCDFile) {
    std::string tmp_file = "test.pcd";

    // Create a dummy PCD file
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloud.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));

    pcl::io::savePCDFile(tmp_file, cloud);

    LaserScan result = util3d::loadScan(tmp_file);

    // Check that result is valid and contains points
    EXPECT_TRUE(!result.empty());  
    EXPECT_EQ(result.data().rows, 1);
    EXPECT_EQ(result.data().cols, 2);
    EXPECT_EQ(result.data().channels(), 3);
    EXPECT_EQ(result.data().type(), CV_32FC3);
    EXPECT_EQ(result.format(), LaserScan::kXYZ);

    for(size_t i=0; i<cloud.size(); ++i)
    {
        EXPECT_EQ(cloud[i].x, result.field(i, 0));
        EXPECT_EQ(cloud[i].y, result.field(i, 1));
        EXPECT_EQ(cloud[i].z, result.field(i, 2));
    }

    std::remove(tmp_file.c_str());  // Clean up the test file
}

// Test case to check loading a PLY file
TEST(Util3dTest, loadScanPLYFile) {
    std::string tmp_file = "test.ply";

    // Create a dummy PLY file
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(7.0, 8.0, 9.0));

    pcl::io::savePLYFile(tmp_file, cloud);

    LaserScan result = util3d::loadScan(tmp_file);

    // Check that result is valid and contains points
    EXPECT_TRUE(!result.empty());  
    EXPECT_EQ(result.data().rows, 1);
    EXPECT_EQ(result.data().cols, 1);
    EXPECT_EQ(result.data().channels(), 3);
    EXPECT_EQ(result.data().type(), CV_32FC3);
    EXPECT_EQ(result.format(), LaserScan::kXYZ);

    for(size_t i=0; i<cloud.size(); ++i)
    {
        EXPECT_EQ(cloud[i].x, result.field(i, 0));
        EXPECT_EQ(cloud[i].y, result.field(i, 1));
        EXPECT_EQ(cloud[i].z, result.field(i, 2));
    }

    std::remove(tmp_file.c_str());  // Clean up the test file
}

TEST(Util3dTest, deskewValidScan) {
    // Prepare mock LaserScan data (simulating a 3x3 point cloud with time info)
    cv::Mat mockData = cv::Mat::zeros(1, 3, CV_32FC(5));
    float * dataPtr = (float*)mockData.data;
    dataPtr[0] = 1;  // x1
    dataPtr[1] = 0; // y1, deskewed it should be -1
    dataPtr[4] = -1; // t1, 1 sec in past
    dataPtr[5] = 1;  // x2
    dataPtr[6] = 0; // y2
    dataPtr[9] = 0; // t2
    dataPtr[10] = 1;  // x3
    dataPtr[11] = 0; // y3, deskewed it should be 1
    dataPtr[14] = 1; // t3, 1 sec in future

    // Create a mock LaserScan object
    LaserScan scan(mockData, 3, 10.0f, LaserScan::kXYZIT);

    // Create a mock Transform for velocity
    Transform velocity(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);  // moving on Y at 1 meter per second

    double inputStamp = 1000.0;  // Example timestamp

    // Perform the deskew operation
    LaserScan result = util3d::deskew(scan, inputStamp, velocity);

    // Verify the result (in a real test, we would assert against the expected values)
    EXPECT_EQ(result.size(), 3);  // Ensure result is not empty
    EXPECT_EQ(result.format(), LaserScan::kXYZI);  // Ensure the format is as expected
    float * resultPtr = (float*)result.data().data;

    EXPECT_EQ(resultPtr[0], dataPtr[0]); // x1: should stay the same
    EXPECT_EQ(resultPtr[1], -1);         // y1: should have moved
    EXPECT_EQ(resultPtr[2], dataPtr[2]); // z1: should stay the same
    EXPECT_EQ(resultPtr[3], dataPtr[3]); // i1: should stay the same

    EXPECT_EQ(resultPtr[4], dataPtr[5]); // x2: should stay the same
    EXPECT_EQ(resultPtr[5], dataPtr[6]); // y2: should stay the same
    EXPECT_EQ(resultPtr[6], dataPtr[7]); // z2: should stay the same
    EXPECT_EQ(resultPtr[7], dataPtr[8]); // i2: should stay the same

    EXPECT_EQ(resultPtr[8], dataPtr[10]);  // x3: should stay the same
    EXPECT_EQ(resultPtr[9], 1);            // y3: should have moved
    EXPECT_EQ(resultPtr[10], dataPtr[12]); // z3: should stay the same
    EXPECT_EQ(resultPtr[11], dataPtr[13]); // i3: should stay the same

    // invalid velocity
    EXPECT_TRUE(util3d::deskew(scan, inputStamp, Transform()).empty());

    // invalid format
    EXPECT_TRUE(util3d::deskew(LaserScan(cv::Mat::zeros(1, 3, CV_32FC(4)), 3, 10.0f, LaserScan::kXYZI), inputStamp, velocity).empty());

    // empty scan
    EXPECT_TRUE(util3d::deskew(LaserScan(), inputStamp, velocity).empty());
}
