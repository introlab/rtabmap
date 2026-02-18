#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Transform.h"
#include "rtabmap/utilite/UException.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UFile.h"
#include <cmath>

using namespace rtabmap;

class CameraModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test camera parameters
        fx_ = 525.0;
        fy_ = 525.0;
        cx_ = 320.0;
        cy_ = 240.0;
        imageWidth_ = 640;
        imageHeight_ = 480;
        imageSize_ = cv::Size(imageWidth_, imageHeight_);
        
        // Create intrinsic matrix K
        K_ = (cv::Mat_<double>(3, 3) <<
            fx_, 0.0, cx_,
            0.0, fy_, cy_,
            0.0, 0.0, 1.0);
        
        // Create distortion coefficients (4 parameters: k1, k2, p1, p2)
        D_ = (cv::Mat_<double>(1, 4) << -0.1, 0.05, 0.001, -0.001);
        
        // Create rectification matrix (identity for simplicity)
        R_ = cv::Mat::eye(3, 3, CV_64FC1);
        
        // Create projection matrix P
        P_ = (cv::Mat_<double>(3, 4) <<
            fx_, 0.0, cx_, 0.0,
            0.0, fy_, cy_, 0.0,
            0.0, 0.0, 1.0, 0.0);
    }

    void TearDown() override {
    }

    double fx_, fy_, cx_, cy_;
    int imageWidth_, imageHeight_;
    cv::Size imageSize_;
    cv::Mat K_, D_, R_, P_;
};

// Constructor Tests

TEST_F(CameraModelTest, DefaultConstructor)
{
    CameraModel model;
    EXPECT_FALSE(model.isValidForProjection());
    EXPECT_FALSE(model.isValidForReprojection());
    EXPECT_FALSE(model.isValidForRectification());
    EXPECT_EQ(model.fx(), 0.0);
    EXPECT_EQ(model.fy(), 0.0);
    EXPECT_EQ(model.cx(), 0.0);
    EXPECT_EQ(model.cy(), 0.0);
}

TEST_F(CameraModelTest, MinimalConstructor)
{
    CameraModel model(fx_, fy_, cx_, cy_);
    EXPECT_TRUE(model.isValidForProjection());
    EXPECT_FALSE(model.isValidForReprojection()); // No image size
    EXPECT_DOUBLE_EQ(model.fx(), fx_);
    EXPECT_DOUBLE_EQ(model.fy(), fy_);
    EXPECT_DOUBLE_EQ(model.cx(), cx_);
    EXPECT_DOUBLE_EQ(model.cy(), cy_);
}

TEST_F(CameraModelTest, MinimalConstructorWithImageSize)
{
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    EXPECT_TRUE(model.isValidForProjection());
    EXPECT_TRUE(model.isValidForReprojection());
    EXPECT_EQ(model.imageWidth(), imageWidth_);
    EXPECT_EQ(model.imageHeight(), imageHeight_);
}

TEST_F(CameraModelTest, MinimalConstructorWithName)
{
    std::string name = "test_camera";
    CameraModel model(name, fx_, fy_, cx_, cy_);
    EXPECT_EQ(model.name(), name);
    EXPECT_TRUE(model.isValidForProjection());
}

TEST_F(CameraModelTest, FullConstructor)
{
    std::string name = "test_camera";
    CameraModel model(name, imageSize_, K_, D_, R_, P_);
    EXPECT_EQ(model.name(), name);
    EXPECT_TRUE(model.isValidForProjection());
    EXPECT_TRUE(model.isValidForReprojection());
    EXPECT_TRUE(model.isValidForRectification());
    EXPECT_DOUBLE_EQ(model.fx(), fx_);
    EXPECT_DOUBLE_EQ(model.fy(), fy_);
    EXPECT_DOUBLE_EQ(model.cx(), cx_);
    EXPECT_DOUBLE_EQ(model.cy(), cy_);
}

// Getter Tests

TEST_F(CameraModelTest, Getters)
{
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    
    EXPECT_DOUBLE_EQ(model.fx(), fx_);
    EXPECT_DOUBLE_EQ(model.fy(), fy_);
    EXPECT_DOUBLE_EQ(model.cx(), cx_);
    EXPECT_DOUBLE_EQ(model.cy(), cy_);
    EXPECT_EQ(model.Tx(), 0.0);
    EXPECT_EQ(model.imageSize(), imageSize_);
    EXPECT_EQ(model.imageWidth(), imageWidth_);
    EXPECT_EQ(model.imageHeight(), imageHeight_);
}

TEST_F(CameraModelTest, MatrixGetters)
{
    CameraModel model("test", imageSize_, K_, D_, R_, P_);
    
    cv::Mat K = model.K();
    EXPECT_FALSE(K.empty());
    EXPECT_DOUBLE_EQ(K.at<double>(0, 0), fx_);
    
    cv::Mat D = model.D();
    EXPECT_FALSE(D.empty());
    
    cv::Mat R = model.R();
    EXPECT_FALSE(R.empty());
    
    cv::Mat P = model.P();
    EXPECT_FALSE(P.empty());
}

// Validation Tests

TEST_F(CameraModelTest, IsValidForProjection)
{
    CameraModel invalid;
    EXPECT_FALSE(invalid.isValidForProjection());
    
    CameraModel valid(fx_, fy_, cx_, cy_);
    EXPECT_TRUE(valid.isValidForProjection());
}

TEST_F(CameraModelTest, IsValidForReprojection)
{
    CameraModel noSize(fx_, fy_, cx_, cy_);
    EXPECT_FALSE(noSize.isValidForReprojection());
    
    CameraModel withSize(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    EXPECT_TRUE(withSize.isValidForReprojection());
}

TEST_F(CameraModelTest, IsValidForRectification)
{
    CameraModel minimal(fx_, fy_, cx_, cy_);
    EXPECT_FALSE(minimal.isValidForRectification());
    
    CameraModel full("test", imageSize_, K_, D_, R_, P_);
    EXPECT_TRUE(full.isValidForRectification());
}

// Rectification Tests

TEST_F(CameraModelTest, InitRectificationMap)
{
    CameraModel model("test", imageSize_, K_, D_, R_, P_);
    
    EXPECT_FALSE(model.isRectificationMapInitialized());
    bool result = model.initRectificationMap();
    EXPECT_TRUE(result);
    EXPECT_TRUE(model.isRectificationMapInitialized());
}

TEST_F(CameraModelTest, InitRectificationMapInvalid)
{
    CameraModel model(fx_, fy_, cx_, cy_); // No distortion, no rectification matrices
    EXPECT_THROW(model.initRectificationMap(), UException);
}

TEST_F(CameraModelTest, RectifyImage)
{
    CameraModel model("test", imageSize_, K_, D_, R_, P_);
    model.initRectificationMap();
    
    // Create a test image
    cv::Mat testImage = cv::Mat::zeros(imageHeight_, imageWidth_, CV_8UC1);
    cv::circle(testImage, cv::Point(imageWidth_/2, imageHeight_/2), 50, cv::Scalar(255), -1);
    
    cv::Mat rectified = model.rectifyImage(testImage);
    EXPECT_FALSE(rectified.empty());
    EXPECT_EQ(rectified.rows, imageHeight_);
    EXPECT_EQ(rectified.cols, imageWidth_);
}

TEST_F(CameraModelTest, RectifyImageWithoutMap)
{
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    
    cv::Mat testImage = cv::Mat::zeros(imageHeight_, imageWidth_, CV_8UC1);
    cv::Mat rectified = model.rectifyImage(testImage);
    
    // Should return a clone of the original if maps are not initialized
    EXPECT_FALSE(rectified.empty());
    EXPECT_EQ(rectified.rows, imageHeight_);
    EXPECT_EQ(rectified.cols, imageWidth_);
}

TEST_F(CameraModelTest, RectifyDepth)
{
    CameraModel model("test", imageSize_, K_, D_, R_, P_);
    model.initRectificationMap();
    
    // Create a test depth image
    cv::Mat depthImage = cv::Mat::zeros(imageHeight_, imageWidth_, CV_16UC1);
    depthImage.at<unsigned short>(imageHeight_/2, imageWidth_/2) = 1000; // 1 meter in mm
    
    cv::Mat rectified = model.rectifyDepth(depthImage);
    EXPECT_FALSE(rectified.empty());
    EXPECT_EQ(rectified.rows, imageHeight_);
    EXPECT_EQ(rectified.cols, imageWidth_);
    EXPECT_EQ(rectified.type(), CV_16UC1);
}

// Projection/Reprojection Tests

TEST_F(CameraModelTest, Project)
{
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    
    float u = cx_;
    float v = cy_;
    float depth = 1.0f; // 1 meter
    
    float x, y, z;
    model.project(u, v, depth, x, y, z);
    
    // At principal point with depth 1.0, x and y should be approximately 0
    EXPECT_NEAR(x, 0.0f, 0.01f);
    EXPECT_NEAR(y, 0.0f, 0.01f);
    EXPECT_FLOAT_EQ(z, depth);
}

TEST_F(CameraModelTest, ProjectInvalidDepth)
{
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    
    float u = cx_;
    float v = cy_;
    float depth = 0.0f; // Invalid depth
    
    float x, y, z;
    model.project(u, v, depth, x, y, z);
    
    // Should return NaN for invalid depth
    EXPECT_TRUE(std::isnan(x));
    EXPECT_TRUE(std::isnan(y));
    EXPECT_TRUE(std::isnan(z));
}

TEST_F(CameraModelTest, ReprojectFloat)
{
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    
    float x = 0.0f;
    float y = 0.0f;
    float z = 1.0f; // 1 meter
    
    float u, v;
    model.reproject(x, y, z, u, v);
    
    // At origin with z=1.0, should project to principal point
    EXPECT_NEAR(u, cx_, 0.01f);
    EXPECT_NEAR(v, cy_, 0.01f);
}

TEST_F(CameraModelTest, ReprojectInt)
{
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    
    float x = 0.0f;
    float y = 0.0f;
    float z = 1.0f;
    
    int u, v;
    model.reproject(x, y, z, u, v);
    
    EXPECT_NEAR(u, static_cast<int>(cx_), 1);
    EXPECT_NEAR(v, static_cast<int>(cy_), 1);
}

// Field of View Tests

TEST_F(CameraModelTest, FieldOfView)
{
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    
    double fovX = model.fovX();
    double fovY = model.fovY();
    double hFOV = model.horizontalFOV();
    double vFOV = model.verticalFOV();
    
    EXPECT_GT(fovX, 0.0);
    EXPECT_GT(fovY, 0.0);
    EXPECT_GT(hFOV, 0.0);
    EXPECT_GT(vFOV, 0.0);
    
    // Horizontal FOV should be larger than vertical for typical cameras
    EXPECT_GT(fovX, fovY);
    
    // Degrees should be approximately radians * 180 / PI
    EXPECT_NEAR(hFOV, fovX * 180.0 / M_PI, 0.1);
    EXPECT_NEAR(vFOV, fovY * 180.0 / M_PI, 0.1);
}

// InFrame Tests

TEST_F(CameraModelTest, InFrame)
{
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    
    EXPECT_TRUE(model.inFrame(0, 0));
    EXPECT_TRUE(model.inFrame(imageWidth_ - 1, imageHeight_ - 1));
    EXPECT_FALSE(model.inFrame(-1, 0));
    EXPECT_FALSE(model.inFrame(0, -1));
    EXPECT_FALSE(model.inFrame(imageWidth_, 0));
    EXPECT_FALSE(model.inFrame(0, imageHeight_));
}

// Scaling and ROI Tests

TEST_F(CameraModelTest, Scaled)
{
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    
    double scale = 0.5;
    CameraModel scaled = model.scaled(scale);
    
    EXPECT_NEAR(scaled.fx(), fx_ * scale, 0.01);
    EXPECT_NEAR(scaled.fy(), fy_ * scale, 0.01);
    EXPECT_NEAR(scaled.cx(), cx_ * scale, 0.01);
    EXPECT_NEAR(scaled.cy(), cy_ * scale, 0.01);
    EXPECT_EQ(scaled.imageWidth(), static_cast<int>(imageWidth_ * scale));
    EXPECT_EQ(scaled.imageHeight(), static_cast<int>(imageHeight_ * scale));
}

TEST_F(CameraModelTest, ROI)
{
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    
    cv::Rect roi(100, 100, 200, 200);
    CameraModel roiModel = model.roi(roi);
    
    EXPECT_NEAR(roiModel.cx(), cx_ - roi.x, 0.01);
    EXPECT_NEAR(roiModel.cy(), cy_ - roi.y, 0.01);
    EXPECT_EQ(roiModel.imageWidth(), roi.width);
    EXPECT_EQ(roiModel.imageHeight(), roi.height);
}

// Serialization Tests

TEST_F(CameraModelTest, SerializeDeserialize)
{
    CameraModel original("test_camera", imageSize_, K_, D_, R_, P_);
    original.setName("original");
    
    std::vector<unsigned char> data = original.serialize();
    EXPECT_FALSE(data.empty());
    
    CameraModel restored;
    unsigned int bytesRead = restored.deserialize(data);
    EXPECT_GT(bytesRead, 0u);
    
    EXPECT_DOUBLE_EQ(restored.fx(), original.fx());
    EXPECT_DOUBLE_EQ(restored.fy(), original.fy());
    EXPECT_DOUBLE_EQ(restored.cx(), original.cx());
    EXPECT_DOUBLE_EQ(restored.cy(), original.cy());
    EXPECT_EQ(restored.imageSize(), original.imageSize());
}

// Name and Transform Tests

TEST_F(CameraModelTest, SetName)
{
    CameraModel model;
    std::string name = "my_camera";
    model.setName(name);
    EXPECT_EQ(model.name(), name);
}

TEST_F(CameraModelTest, LocalTransform)
{
    Transform transform = Transform(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    CameraModel model(fx_, fy_, cx_, cy_, transform);
    
    EXPECT_FALSE(model.localTransform().isNull());
    model.setLocalTransform(CameraModel::opticalRotation());
    EXPECT_FALSE(model.localTransform().isNull());
}

// Fisheye Tests

TEST_F(CameraModelTest, IsFisheye)
{
    // Standard distortion (4 parameters)
    CameraModel standard("test", imageSize_, K_, D_, R_, P_);
    EXPECT_FALSE(standard.isFisheye());
    
    // Fisheye distortion (6 parameters: k1, k2, 0, 0, k3, k4)
    cv::Mat D_fisheye = (cv::Mat_<double>(1, 6) << 0.1, 0.05, 0.0, 0.0, 0.01, 0.005);
    CameraModel fisheye("test", imageSize_, K_, D_fisheye, R_, P_);
    EXPECT_TRUE(fisheye.isFisheye());
}

// SetImageSize Tests

TEST_F(CameraModelTest, SetImageSize)
{
    CameraModel model(fx_, fy_, 0.0, 0.0); // cx, cy = 0
    
    cv::Size newSize(320, 240);
    model.setImageSize(newSize);
    
    EXPECT_EQ(model.imageSize(), newSize);
    // Principal point should be set to center
    EXPECT_NEAR(model.cx(), newSize.width / 2.0 - 0.5, 0.01);
    EXPECT_NEAR(model.cy(), newSize.height / 2.0 - 0.5, 0.01);
}

// Tx (Baseline) Tests

TEST_F(CameraModelTest, Tx)
{
    double Tx = fx_ * 0.12; // Baseline * fx (e.g., 12cm baseline)
    CameraModel model(fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), Tx, imageSize_);
    
    EXPECT_NEAR(model.Tx(), Tx, 0.01);
}

// Save/Load Tests

TEST_F(CameraModelTest, SaveLoadRoundTrip)
{
    // Create a temporary directory for testing
    std::string testDir = "test_camera_calibration";
    UDirectory::makeDir(testDir);
    
    // Create original camera model with all parameters
    std::string cameraName = "test_camera";
    Transform localTransform = Transform(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    CameraModel original(cameraName, imageSize_, K_, D_, R_, P_, localTransform);
    
    // Save the model
    bool saveResult = original.save(testDir);
    EXPECT_TRUE(saveResult);
    
    // Verify file was created
    std::string expectedFile = testDir + "/" + cameraName + ".yaml";
    EXPECT_TRUE(UFile::exists(expectedFile));
    
    // Load the model back
    CameraModel loaded;
    bool loadResult = loaded.load(testDir, cameraName);
    EXPECT_TRUE(loadResult);
    
    // Verify all parameters match
    
    // Basic parameters
    EXPECT_EQ(loaded.name(), original.name());
    EXPECT_EQ(loaded.imageSize(), original.imageSize());
    EXPECT_EQ(loaded.imageWidth(), original.imageWidth());
    EXPECT_EQ(loaded.imageHeight(), original.imageHeight());
    
    // Intrinsic parameters
    EXPECT_DOUBLE_EQ(loaded.fx(), original.fx());
    EXPECT_DOUBLE_EQ(loaded.fy(), original.fy());
    EXPECT_DOUBLE_EQ(loaded.cx(), original.cx());
    EXPECT_DOUBLE_EQ(loaded.cy(), original.cy());
    EXPECT_DOUBLE_EQ(loaded.Tx(), original.Tx());
    
    // Raw intrinsic matrix K
    cv::Mat K_raw_loaded = loaded.K_raw();
    cv::Mat K_raw_original = original.K_raw();
    if(!K_raw_loaded.empty() && !K_raw_original.empty())
    {
        EXPECT_EQ(K_raw_loaded.rows, K_raw_original.rows);
        EXPECT_EQ(K_raw_loaded.cols, K_raw_original.cols);
        for(int i = 0; i < K_raw_loaded.rows; ++i)
        {
            for(int j = 0; j < K_raw_loaded.cols; ++j)
            {
                EXPECT_NEAR(K_raw_loaded.at<double>(i, j), K_raw_original.at<double>(i, j), 0.001);
            }
        }
    }
    
    // Intrinsic matrix K (may be rectified)
    cv::Mat K_loaded = loaded.K();
    cv::Mat K_original = original.K();
    EXPECT_EQ(K_loaded.rows, K_original.rows);
    EXPECT_EQ(K_loaded.cols, K_original.cols);
    for(int i = 0; i < K_loaded.rows; ++i)
    {
        for(int j = 0; j < K_loaded.cols; ++j)
        {
            EXPECT_NEAR(K_loaded.at<double>(i, j), K_original.at<double>(i, j), 0.001);
        }
    }
    
    // Raw distortion coefficients
    cv::Mat D_raw_loaded = loaded.D_raw();
    cv::Mat D_raw_original = original.D_raw();
    if(!D_raw_loaded.empty() && !D_raw_original.empty())
    {
        EXPECT_EQ(D_raw_loaded.rows, D_raw_original.rows);
        EXPECT_EQ(D_raw_loaded.cols, D_raw_original.cols);
        for(int i = 0; i < D_raw_loaded.cols; ++i)
        {
            EXPECT_NEAR(D_raw_loaded.at<double>(0, i), D_raw_original.at<double>(0, i), 0.001);
        }
    }
    
    // Distortion coefficients (may be rectified)
    cv::Mat D_loaded = loaded.D();
    cv::Mat D_original = original.D();
    if(!D_loaded.empty() && !D_original.empty())
    {
        EXPECT_EQ(D_loaded.rows, D_original.rows);
        EXPECT_EQ(D_loaded.cols, D_original.cols);
        for(int i = 0; i < D_loaded.cols; ++i)
        {
            EXPECT_NEAR(D_loaded.at<double>(0, i), D_original.at<double>(0, i), 0.001);
        }
    }
    
    // Rectification matrix R
    cv::Mat R_loaded = loaded.R();
    cv::Mat R_original = original.R();
    if(!R_loaded.empty() && !R_original.empty())
    {
        EXPECT_EQ(R_loaded.rows, R_original.rows);
        EXPECT_EQ(R_loaded.cols, R_original.cols);
        for(int i = 0; i < R_loaded.rows; ++i)
        {
            for(int j = 0; j < R_loaded.cols; ++j)
            {
                EXPECT_NEAR(R_loaded.at<double>(i, j), R_original.at<double>(i, j), 0.001);
            }
        }
    }
    
    // Projection matrix P
    cv::Mat P_loaded = loaded.P();
    cv::Mat P_original = original.P();
    if(!P_loaded.empty() && !P_original.empty())
    {
        EXPECT_EQ(P_loaded.rows, P_original.rows);
        EXPECT_EQ(P_loaded.cols, P_original.cols);
        for(int i = 0; i < P_loaded.rows; ++i)
        {
            for(int j = 0; j < P_loaded.cols; ++j)
            {
                EXPECT_NEAR(P_loaded.at<double>(i, j), P_original.at<double>(i, j), 0.001);
            }
        }
    }
    
    // Local transform
    Transform localTransform_loaded = loaded.localTransform();
    Transform localTransform_original = original.localTransform();
    if(!localTransform_loaded.isNull() && !localTransform_original.isNull())
    {
        // Compare transform matrices element by element
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 4; ++j)
            {
                EXPECT_NEAR(localTransform_loaded.data()[i*4+j], localTransform_original.data()[i*4+j], 0.001);
            }
        }
    }
    
    // Fisheye detection
    EXPECT_EQ(loaded.isFisheye(), original.isFisheye());
    
    // Validation states
    EXPECT_EQ(loaded.isValidForProjection(), original.isValidForProjection());
    EXPECT_EQ(loaded.isValidForReprojection(), original.isValidForReprojection());
    EXPECT_EQ(loaded.isValidForRectification(), original.isValidForRectification());
    
    // Field of view (if image size is set)
    if(loaded.imageWidth() > 0 && loaded.imageHeight() > 0 && original.imageWidth() > 0 && original.imageHeight() > 0)
    {
        EXPECT_NEAR(loaded.fovX(), original.fovX(), 0.001);
        EXPECT_NEAR(loaded.fovY(), original.fovY(), 0.001);
        EXPECT_NEAR(loaded.horizontalFOV(), original.horizontalFOV(), 0.001);
        EXPECT_NEAR(loaded.verticalFOV(), original.verticalFOV(), 0.001);
    }
}

TEST_F(CameraModelTest, SaveLoadRoundTripFullPath)
{
    // Create a temporary directory for testing
    std::string testDir = "test_camera_calibration2";
    UDirectory::makeDir(testDir);
    
    // Create original camera model
    std::string cameraName = "test_camera2";
    CameraModel original(cameraName, imageSize_, K_, D_, R_, P_);
    
    // Save the model
    bool saveResult = original.save(testDir);
    EXPECT_TRUE(saveResult);
    
    // Load using full file path
    std::string filePath = testDir + "/" + cameraName + ".yaml";
    CameraModel loaded;
    bool loadResult = loaded.load(filePath);
    EXPECT_TRUE(loadResult);
    
    // Verify all parameters match
    EXPECT_EQ(loaded.name(), original.name());
    EXPECT_EQ(loaded.imageSize(), original.imageSize());
    EXPECT_DOUBLE_EQ(loaded.fx(), original.fx());
    EXPECT_DOUBLE_EQ(loaded.fy(), original.fy());
    EXPECT_DOUBLE_EQ(loaded.cx(), original.cx());
    EXPECT_DOUBLE_EQ(loaded.cy(), original.cy());
    EXPECT_DOUBLE_EQ(loaded.Tx(), original.Tx());
    
    // Verify matrices
    cv::Mat K_loaded = loaded.K();
    cv::Mat K_original = original.K();
    if(!K_loaded.empty() && !K_original.empty())
    {
        EXPECT_EQ(K_loaded.rows, K_original.rows);
        EXPECT_EQ(K_loaded.cols, K_original.cols);
        for(int i = 0; i < K_loaded.rows; ++i)
        {
            for(int j = 0; j < K_loaded.cols; ++j)
            {
                EXPECT_NEAR(K_loaded.at<double>(i, j), K_original.at<double>(i, j), 0.001);
            }
        }
    }
    
    cv::Mat D_loaded = loaded.D();
    cv::Mat D_original = original.D();
    if(!D_loaded.empty() && !D_original.empty())
    {
        EXPECT_EQ(D_loaded.cols, D_original.cols);
        for(int i = 0; i < D_loaded.cols; ++i)
        {
            EXPECT_NEAR(D_loaded.at<double>(0, i), D_original.at<double>(0, i), 0.001);
        }
    }
    
    // Verify validation states
    EXPECT_EQ(loaded.isValidForProjection(), original.isValidForProjection());
    EXPECT_EQ(loaded.isValidForReprojection(), original.isValidForReprojection());
    EXPECT_EQ(loaded.isValidForRectification(), original.isValidForRectification());
}

TEST_F(CameraModelTest, SaveLoadRoundTripMinimal)
{
    // Create a temporary directory for testing
    std::string testDir = "test_camera_calibration3";
    UDirectory::makeDir(testDir);
    
    // Create minimal camera model (no distortion, no rectification)
    std::string cameraName = "minimal_camera";
    CameraModel original(cameraName, fx_, fy_, cx_, cy_, CameraModel::opticalRotation(), 0.0, imageSize_);
    
    // Save the model
    bool saveResult = original.save(testDir);
    EXPECT_TRUE(saveResult);
    
    // Load the model back
    CameraModel loaded;
    bool loadResult = loaded.load(testDir, cameraName);
    EXPECT_TRUE(loadResult);
    
    // Verify all parameters match
    EXPECT_EQ(loaded.name(), original.name());
    EXPECT_EQ(loaded.imageSize(), original.imageSize());
    EXPECT_EQ(loaded.imageWidth(), original.imageWidth());
    EXPECT_EQ(loaded.imageHeight(), original.imageHeight());
    EXPECT_DOUBLE_EQ(loaded.fx(), original.fx());
    EXPECT_DOUBLE_EQ(loaded.fy(), original.fy());
    EXPECT_DOUBLE_EQ(loaded.cx(), original.cx());
    EXPECT_DOUBLE_EQ(loaded.cy(), original.cy());
    EXPECT_DOUBLE_EQ(loaded.Tx(), original.Tx());
    
    // Verify matrices
    cv::Mat K_loaded = loaded.K();
    cv::Mat K_original = original.K();
    if(!K_loaded.empty() && !K_original.empty())
    {
        EXPECT_EQ(K_loaded.rows, K_original.rows);
        EXPECT_EQ(K_loaded.cols, K_original.cols);
        for(int i = 0; i < K_loaded.rows; ++i)
        {
            for(int j = 0; j < K_loaded.cols; ++j)
            {
                EXPECT_NEAR(K_loaded.at<double>(i, j), K_original.at<double>(i, j), 0.001);
            }
        }
    }
    
    // Verify validation states
    EXPECT_EQ(loaded.isValidForProjection(), original.isValidForProjection());
    EXPECT_EQ(loaded.isValidForReprojection(), original.isValidForReprojection());
}

TEST_F(CameraModelTest, SaveLoadRoundTripFisheye)
{
    // Create a temporary directory for testing
    std::string testDir = "test_camera_calibration4";
    UDirectory::makeDir(testDir);
    
    // Create fisheye camera model (6 distortion parameters: k1, k2, 0, 0, k3, k4)
    // Format: [k1, k2, p1, p2, k3, k4] where p1=p2=0 for fisheye
    double k1 = 0.1;
    double k2 = 0.05;
    double k3 = 0.01;
    double k4 = 0.005;
    cv::Mat D_fisheye = (cv::Mat_<double>(1, 6) << k1, k2, 0.0, 0.0, k3, k4);
    std::string cameraName = "fisheye_camera";
    CameraModel original(cameraName, imageSize_, K_, D_fisheye, R_, P_);
    
    EXPECT_TRUE(original.isFisheye());
    EXPECT_EQ(original.D_raw().cols, 6);
    
    // Verify original distortion coefficients
    cv::Mat D_original = original.D_raw();
    EXPECT_DOUBLE_EQ(D_original.at<double>(0, 0), k1);
    EXPECT_DOUBLE_EQ(D_original.at<double>(0, 1), k2);
    EXPECT_DOUBLE_EQ(D_original.at<double>(0, 2), 0.0); // p1
    EXPECT_DOUBLE_EQ(D_original.at<double>(0, 3), 0.0); // p2
    EXPECT_DOUBLE_EQ(D_original.at<double>(0, 4), k3);
    EXPECT_DOUBLE_EQ(D_original.at<double>(0, 5), k4);
    
    // Save the model (converts 6 params to 4 params for ROS compatibility)
    bool saveResult = original.save(testDir);
    EXPECT_TRUE(saveResult);
    
    // Load the model back (converts 4 params back to 6 params)
    CameraModel loaded;
    bool loadResult = loaded.load(testDir, cameraName);
    EXPECT_TRUE(loadResult);
    
    // Verify basic parameters match
    EXPECT_EQ(loaded.name(), original.name());
    EXPECT_DOUBLE_EQ(loaded.fx(), original.fx());
    EXPECT_DOUBLE_EQ(loaded.fy(), original.fy());
    EXPECT_DOUBLE_EQ(loaded.cx(), original.cx());
    EXPECT_DOUBLE_EQ(loaded.cy(), original.cy());
    EXPECT_EQ(loaded.imageSize(), original.imageSize());
    
    // Verify fisheye model is preserved after save/load conversion
    EXPECT_TRUE(loaded.isFisheye());
    EXPECT_EQ(loaded.D_raw().cols, 6);
    
    // Verify distortion coefficients are correctly converted back
    // Save converts: [k1, k2, 0, 0, k3, k4] -> [k1, k2, k3, k4]
    // Load converts: [k1, k2, k3, k4] -> [k1, k2, 0, 0, k3, k4]
    cv::Mat D_loaded = loaded.D_raw();
    EXPECT_DOUBLE_EQ(D_loaded.at<double>(0, 0), k1); // k1 preserved
    EXPECT_DOUBLE_EQ(D_loaded.at<double>(0, 1), k2); // k2 preserved
    EXPECT_DOUBLE_EQ(D_loaded.at<double>(0, 2), 0.0); // p1 should be 0
    EXPECT_DOUBLE_EQ(D_loaded.at<double>(0, 3), 0.0); // p2 should be 0
    EXPECT_DOUBLE_EQ(D_loaded.at<double>(0, 4), k3); // k3 preserved
    EXPECT_DOUBLE_EQ(D_loaded.at<double>(0, 5), k4); // k4 preserved
    
    // Verify the model is still valid for rectification
    EXPECT_TRUE(loaded.isValidForRectification());
    EXPECT_EQ(loaded.isValidForProjection(), original.isValidForProjection());
    
    // Verify rectification maps can be initialized
    bool mapInitResult = loaded.initRectificationMap();
    EXPECT_TRUE(mapInitResult);
    EXPECT_TRUE(loaded.isRectificationMapInitialized());
}

