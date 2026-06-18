#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Transform.h"
#include "rtabmap/utilite/UException.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UFile.h"
#include <cmath>

using namespace rtabmap;

class StereoCameraModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test camera parameters
        fx_ = 525.0;
        fy_ = 525.0;
        cx_ = 320.0;
        cy_ = 240.0;
        baseline_ = 0.12; // 12 cm baseline
        imageWidth_ = 640;
        imageHeight_ = 480;
        imageSize_ = cv::Size(imageWidth_, imageHeight_);
        
        // Create intrinsic matrix K
        K_ = (cv::Mat_<double>(3, 3) <<
            fx_, 0.0, cx_,
            0.0, fy_, cy_,
            0.0, 0.0, 1.0);
        
        // Create distortion coefficients
        D_ = (cv::Mat_<double>(1, 4) << -0.1, 0.05, 0.001, -0.001);
        
        // Create rectification matrix
        R_ = cv::Mat::eye(3, 3, CV_64FC1);
        
        // Create projection matrix P (with Tx = baseline * fx for left camera)
        double Tx = baseline_ * fx_;
        P_left_ = (cv::Mat_<double>(3, 4) <<
            fx_, 0.0, cx_, Tx,
            0.0, fy_, cy_, 0.0,
            0.0, 0.0, 1.0, 0.0);
        
        // Right camera projection matrix (Tx = 0 typically)
        P_right_ = (cv::Mat_<double>(3, 4) <<
            fx_, 0.0, cx_, 0.0,
            0.0, fy_, cy_, 0.0,
            0.0, 0.0, 1.0, 0.0);
        
        // Create stereo extrinsic parameters
        // R and T represent the left camera relative to the right camera coordinate system
        // For parallel cameras with baseline along x-axis, T is negative baseline
        // Translation of left camera relative to right camera coordinate system
        T_ = (cv::Mat_<double>(3, 1) << -baseline_, 0.0, 0.0);
        
        // Rotation matrix of left camera relative to right camera coordinate system
        // (identity for parallel cameras)
        R_stereo_ = cv::Mat::eye(3, 3, CV_64FC1);
    }

    void TearDown() override {
    }

    double fx_, fy_, cx_, cy_, baseline_;
    int imageWidth_, imageHeight_;
    cv::Size imageSize_;
    cv::Mat K_, D_, R_, P_left_, P_right_, R_stereo_, T_;
};

// Constructor Tests

TEST_F(StereoCameraModelTest, DefaultConstructor)
{
    StereoCameraModel model;
    EXPECT_FALSE(model.isValidForProjection());
    EXPECT_FALSE(model.isValidForRectification());
    EXPECT_EQ(model.baseline(), 0.0);
}

TEST_F(StereoCameraModelTest, MinimalConstructor)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    EXPECT_TRUE(model.isValidForProjection());
    EXPECT_NEAR(model.baseline(), baseline_, 0.001);
    EXPECT_DOUBLE_EQ(model.left().fx(), fx_);
    EXPECT_DOUBLE_EQ(model.right().fx(), fx_);
}

TEST_F(StereoCameraModelTest, MinimalConstructorWithName)
{
    std::string name = "stereo_camera";
    StereoCameraModel model(name, fx_, fy_, cx_, cy_, baseline_);
    EXPECT_EQ(model.name(), name);
    EXPECT_TRUE(model.isValidForProjection());
    EXPECT_NEAR(model.baseline(), baseline_, 0.001);
}

TEST_F(StereoCameraModelTest, ConstructorFromCameraModels)
{
    CameraModel left("left", imageSize_, K_, D_, R_, P_left_);
    CameraModel right("right", imageSize_, K_, D_, R_, P_right_);
    
    StereoCameraModel model("stereo", left, right, R_stereo_, T_);
    
    EXPECT_EQ(model.name(), "stereo");
    EXPECT_TRUE(model.isValidForProjection());
    EXPECT_NEAR(model.baseline(), baseline_, 0.01);
    EXPECT_EQ(model.left().name(), "stereo_left");
    EXPECT_EQ(model.right().name(), "stereo_right");
}

TEST_F(StereoCameraModelTest, ConstructorFromCameraModelsWithTransform)
{
    CameraModel left("left", imageSize_, K_, D_, R_, P_left_);
    CameraModel right("right", imageSize_, K_, D_, R_, P_right_);
    
    // Transform represents left camera relative to right camera coordinate system
    // For baseline along x-axis, x should be negative
    Transform extrinsics = Transform(-baseline_, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    StereoCameraModel model("stereo", left, right, extrinsics);
    
    EXPECT_TRUE(model.isValidForProjection());
    EXPECT_NEAR(model.baseline(), baseline_, 0.01);
    
    // Verify stereo transform matches (left camera relative to right camera coordinate system)
    Transform stereoTransform = model.stereoTransform();
    EXPECT_NEAR(stereoTransform.x(), -baseline_, 0.001);
}

TEST_F(StereoCameraModelTest, FullConstructor)
{
    std::string name = "stereo_camera";
    StereoCameraModel model(
        name,
        imageSize_, K_, D_, R_, P_left_,
        imageSize_, K_, D_, R_, P_right_,
        R_stereo_, T_, cv::Mat(), cv::Mat()
    );
    
    EXPECT_EQ(model.name(), name);
    EXPECT_TRUE(model.isValidForProjection());
    EXPECT_TRUE(model.isValidForRectification());
    EXPECT_NEAR(model.baseline(), baseline_, 0.01);
}

// Validation Tests

TEST_F(StereoCameraModelTest, IsValidForProjection)
{
    StereoCameraModel invalid;
    EXPECT_FALSE(invalid.isValidForProjection());
    
    StereoCameraModel valid(fx_, fy_, cx_, cy_, baseline_);
    EXPECT_TRUE(valid.isValidForProjection());
}

TEST_F(StereoCameraModelTest, IsValidForRectification)
{
    StereoCameraModel minimal(fx_, fy_, cx_, cy_, baseline_);
    EXPECT_FALSE(minimal.isValidForRectification());
    
    CameraModel left("left", imageSize_, K_, D_, R_, P_left_);
    CameraModel right("right", imageSize_, K_, D_, R_, P_right_);
    StereoCameraModel full("stereo", left, right, R_stereo_, T_);
    EXPECT_TRUE(full.isValidForRectification());
}

// Rectification Tests

TEST_F(StereoCameraModelTest, InitRectificationMap)
{
    CameraModel left("left", imageSize_, K_, D_, R_, P_left_);
    CameraModel right("right", imageSize_, K_, D_, R_, P_right_);
    StereoCameraModel model("stereo", left, right, R_stereo_, T_);
    
    EXPECT_FALSE(model.isRectificationMapInitialized());
    model.initRectificationMap();
    EXPECT_TRUE(model.isRectificationMapInitialized());
}

// Depth/Disparity Conversion Tests

TEST_F(StereoCameraModelTest, ComputeDepth)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    
    // Test with known disparity
    // disparity = baseline * fx / depth
    float depth = 1.0f; // 1 meter
    float expectedDisparity = static_cast<float>(baseline_ * fx_ / depth);
    
    float computedDepth = model.computeDepth(expectedDisparity);
    EXPECT_NEAR(computedDepth, depth, 0.01f);
}

TEST_F(StereoCameraModelTest, ComputeDepthZeroDisparity)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    float depth = model.computeDepth(0.0f);
    EXPECT_EQ(depth, 0.0f);
}

TEST_F(StereoCameraModelTest, ComputeDisparityFromDepth)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    
    float depth = 1.0f; // 1 meter
    float disparity = model.computeDisparity(depth);
    
    // Verify round-trip
    float computedDepth = model.computeDepth(disparity);
    EXPECT_NEAR(computedDepth, depth, 0.01f);
}

TEST_F(StereoCameraModelTest, ComputeDisparityFromDepthMM)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    
    unsigned short depthMM = 1000; // 1 meter in millimeters
    float disparity = model.computeDisparity(depthMM);
    
    // Should be same as computing from meters
    float disparityFromMeters = model.computeDisparity(1.0f);
    EXPECT_NEAR(disparity, disparityFromMeters, 0.1f);
}

TEST_F(StereoCameraModelTest, ComputeDisparityZeroDepth)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    float disparity = model.computeDisparity(0.0f);
    EXPECT_EQ(disparity, 0.0f);
    
    unsigned short depthMM = 0;
    float disparityMM = model.computeDisparity(depthMM);
    EXPECT_EQ(disparityMM, 0.0f);
}

// Getter Tests

TEST_F(StereoCameraModelTest, Baseline)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    EXPECT_NEAR(model.baseline(), baseline_, 0.001);
}

TEST_F(StereoCameraModelTest, LeftRightModels)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    
    const CameraModel& left = model.left();
    const CameraModel& right = model.right();
    
    EXPECT_DOUBLE_EQ(left.fx(), fx_);
    EXPECT_DOUBLE_EQ(right.fx(), fx_);
    EXPECT_DOUBLE_EQ(left.fy(), fy_);
    EXPECT_DOUBLE_EQ(right.fy(), fy_);
}

TEST_F(StereoCameraModelTest, ExtrinsicMatrices)
{
    CameraModel left("left", imageSize_, K_, D_, R_, P_left_);
    CameraModel right("right", imageSize_, K_, D_, R_, P_right_);
    StereoCameraModel model("stereo", left, right, R_stereo_, T_);
    
    // R and T represent left camera relative to right camera coordinate system
    const cv::Mat& R = model.R();
    const cv::Mat& T = model.T();
    
    EXPECT_FALSE(R.empty());
    EXPECT_FALSE(T.empty());
    EXPECT_EQ(R.rows, 3);
    EXPECT_EQ(R.cols, 3);
    EXPECT_EQ(T.rows, 3);
    EXPECT_EQ(T.cols, 1);
    
    // Verify T matches expected value (negative baseline for left relative to right)
    EXPECT_NEAR(T.at<double>(0, 0), -baseline_, 0.001);
    EXPECT_NEAR(T.at<double>(1, 0), 0.0, 0.001);
    EXPECT_NEAR(T.at<double>(2, 0), 0.0, 0.001);
}

// Name and Suffix Tests

TEST_F(StereoCameraModelTest, SetName)
{
    StereoCameraModel model;
    std::string name = "my_stereo";
    model.setName(name);
    EXPECT_EQ(model.name(), name);
}

TEST_F(StereoCameraModelTest, SetNameWithSuffixes)
{
    StereoCameraModel model;
    model.setName("stereo", "cam1", "cam2");
    EXPECT_EQ(model.name(), "stereo");
    EXPECT_EQ(model.getLeftSuffix(), "cam1");
    EXPECT_EQ(model.getRightSuffix(), "cam2");
}

TEST_F(StereoCameraModelTest, GetSuffixes)
{
    StereoCameraModel model;
    EXPECT_EQ(model.getLeftSuffix(), "left");
    EXPECT_EQ(model.getRightSuffix(), "right");
}

// Transform Tests

TEST_F(StereoCameraModelTest, LocalTransform)
{
    Transform transform = Transform(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_, transform);
    
    EXPECT_FALSE(model.localTransform().isNull());
    model.setLocalTransform(Transform());
    EXPECT_TRUE(model.localTransform().isNull());
}

TEST_F(StereoCameraModelTest, StereoTransform)
{
    CameraModel left("left", imageSize_, K_, D_, R_, P_left_);
    CameraModel right("right", imageSize_, K_, D_, R_, P_right_);
    StereoCameraModel model("stereo", left, right, R_stereo_, T_);
    
    Transform stereoTransform = model.stereoTransform();
    EXPECT_FALSE(stereoTransform.isNull());
    
    // Stereo transform represents left camera relative to right camera coordinate system
    // For a baseline of 0.12 m, the x value should be -0.12 (negative)
    EXPECT_NEAR(stereoTransform.x(), -baseline_, 0.001);
}

TEST_F(StereoCameraModelTest, StereoTransformBaselineExample)
{
    // Test the specific example from documentation: 15 cm baseline -> -0.15 x value
    double testBaseline = 0.15; // 15 cm
    
    CameraModel left("left", imageSize_, K_, D_, R_, P_left_);
    CameraModel right("right", imageSize_, K_, D_, R_, P_right_);
    
    // Create T with negative baseline (left camera relative to right camera coordinate system)
    cv::Mat T_test = (cv::Mat_<double>(3, 1) << -testBaseline, 0.0, 0.0);
    cv::Mat R_test = cv::Mat::eye(3, 3, CV_64FC1);
    
    StereoCameraModel model("stereo", left, right, R_test, T_test);
    
    Transform stereoTransform = model.stereoTransform();
    EXPECT_FALSE(stereoTransform.isNull());
    
    // Verify the x value is -0.15 as documented
    EXPECT_NEAR(stereoTransform.x(), -0.15, 0.001);
    
    // Verify baseline matches
    EXPECT_NEAR(model.baseline(), testBaseline, 0.001);
}

// Scaling and ROI Tests

TEST_F(StereoCameraModelTest, Scale)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    model.setImageSize(imageSize_);
    
    double scale = 0.5;
    model.scale(scale);
    
    EXPECT_NEAR(model.left().fx(), fx_ * scale, 0.01);
    EXPECT_NEAR(model.right().fx(), fx_ * scale, 0.01);
    EXPECT_EQ(model.left().imageWidth(), static_cast<int>(imageWidth_ * scale));
    // Baseline should not be scaled (it's a physical distance)
    EXPECT_NEAR(model.baseline(), baseline_, 0.001);
}

TEST_F(StereoCameraModelTest, ROI)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    model.setImageSize(imageSize_);
    
    cv::Rect roi(100, 100, 200, 200);
    model.roi(roi);
    
    EXPECT_EQ(model.left().imageWidth(), roi.width);
    EXPECT_EQ(model.left().imageHeight(), roi.height);
    EXPECT_EQ(model.right().imageWidth(), roi.width);
    EXPECT_EQ(model.right().imageHeight(), roi.height);
}

// SetImageSize Tests

TEST_F(StereoCameraModelTest, SetImageSize)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    
    cv::Size newSize(320, 240);
    model.setImageSize(newSize);
    
    EXPECT_EQ(model.left().imageSize(), newSize);
    EXPECT_EQ(model.right().imageSize(), newSize);
}

// Serialization Tests

TEST_F(StereoCameraModelTest, SerializeDeserialize)
{
    StereoCameraModel original(fx_, fy_, cx_, cy_, baseline_);
    original.setName("test_stereo");
    original.setImageSize(imageSize_);
    
    std::vector<unsigned char> data = original.serialize();
    EXPECT_FALSE(data.empty());
    
    StereoCameraModel restored;
    unsigned int bytesRead = restored.deserialize(data);
    EXPECT_GT(bytesRead, 0u);
    
    EXPECT_NEAR(restored.baseline(), original.baseline(), 0.001);
    EXPECT_DOUBLE_EQ(restored.left().fx(), original.left().fx());
    EXPECT_DOUBLE_EQ(restored.right().fx(), original.right().fx());
}

TEST_F(StereoCameraModelTest, SerializeDeserializeFromPointer)
{
    StereoCameraModel original(fx_, fy_, cx_, cy_, baseline_);
    original.setName("test_stereo");
    
    std::vector<unsigned char> data = original.serialize();
    
    StereoCameraModel restored;
    unsigned int bytesRead = restored.deserialize(data.data(), data.size());
    EXPECT_GT(bytesRead, 0u);
    EXPECT_EQ(bytesRead, data.size());
    
    EXPECT_NEAR(restored.baseline(), original.baseline(), 0.001);
}

// Round-trip Depth/Disparity Tests

TEST_F(StereoCameraModelTest, DepthDisparityRoundTrip)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, baseline_);
    
    // Test multiple depths
    float testDepths[] = {0.5f, 1.0f, 2.0f, 5.0f, 10.0f};
    
    for(float depth : testDepths)
    {
        float disparity = model.computeDisparity(depth);
        float computedDepth = model.computeDepth(disparity);
        EXPECT_NEAR(computedDepth, depth, 0.01f) << "Depth: " << depth;
    }
}

// Edge Cases

TEST_F(StereoCameraModelTest, InvalidBaseline)
{
    StereoCameraModel model(fx_, fy_, cx_, cy_, 0.0); // Zero baseline
    EXPECT_FALSE(model.isValidForProjection());
    EXPECT_EQ(model.baseline(), 0.0);
}

TEST_F(StereoCameraModelTest, NegativeBaseline)
{
    // Negative baseline should still compute, but may not be physically meaningful
    StereoCameraModel model(fx_, fy_, cx_, cy_, -0.12);
    EXPECT_DOUBLE_EQ(model.baseline(),-0.12);
}

// Save/Load Tests

TEST_F(StereoCameraModelTest, SaveLoadRoundTrip)
{
    // Create a temporary directory for testing
    std::string testDir = "test_stereo_calibration";
    UDirectory::makeDir(testDir);
    
    // Create original stereo camera model with full parameters
    std::string cameraName = "test_stereo";
    CameraModel left("left", imageSize_, K_, D_, R_, P_left_);
    CameraModel right("right", imageSize_, K_, D_, R_, P_right_);
    StereoCameraModel original(cameraName, left, right, R_stereo_, T_);
    
    // Save the model (with stereo transform)
    bool saveResult = original.save(testDir, false);
    EXPECT_TRUE(saveResult);
    
    // Verify files were created
    std::string leftFile = testDir + "/" + cameraName + "_left.yaml";
    std::string rightFile = testDir + "/" + cameraName + "_right.yaml";
    std::string poseFile = testDir + "/" + cameraName + "_pose.yaml";
    EXPECT_TRUE(UFile::exists(leftFile));
    EXPECT_TRUE(UFile::exists(rightFile));
    EXPECT_TRUE(UFile::exists(poseFile));
    
    // Load the model back
    StereoCameraModel loaded;
    bool loadResult = loaded.load(testDir, cameraName, false);
    EXPECT_TRUE(loadResult);
    
    // Verify all parameters match
    EXPECT_STREQ(loaded.name().c_str(), original.name().c_str());
    EXPECT_NEAR(loaded.baseline(), original.baseline(), 0.001);
    
    // Verify left camera parameters
    EXPECT_DOUBLE_EQ(loaded.left().fx(), original.left().fx());
    EXPECT_DOUBLE_EQ(loaded.left().fy(), original.left().fy());
    EXPECT_DOUBLE_EQ(loaded.left().cx(), original.left().cx());
    EXPECT_DOUBLE_EQ(loaded.left().cy(), original.left().cy());
    EXPECT_DOUBLE_EQ(loaded.left().Tx(), original.left().Tx());
    EXPECT_EQ(loaded.left().imageSize(), original.left().imageSize());
    
    // Verify right camera parameters
    EXPECT_DOUBLE_EQ(loaded.right().fx(), original.right().fx());
    EXPECT_DOUBLE_EQ(loaded.right().fy(), original.right().fy());
    EXPECT_DOUBLE_EQ(loaded.right().cx(), original.right().cx());
    EXPECT_DOUBLE_EQ(loaded.right().cy(), original.right().cy());
    EXPECT_DOUBLE_EQ(loaded.right().Tx(), original.right().Tx());
    EXPECT_EQ(loaded.right().imageSize(), original.right().imageSize());
    
    // Verify stereo extrinsic matrices
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
    
    cv::Mat T_loaded = loaded.T();
    cv::Mat T_original = original.T();
    if(!T_loaded.empty() && !T_original.empty())
    {
        EXPECT_EQ(T_loaded.rows, T_original.rows);
        EXPECT_EQ(T_loaded.cols, T_original.cols);
        for(int i = 0; i < T_loaded.rows; ++i)
        {
            for(int j = 0; j < T_loaded.cols; ++j)
            {
                EXPECT_NEAR(T_loaded.at<double>(i, j), T_original.at<double>(i, j), 0.001);
            }
        }
    }
    
    // Verify stereo transform (left camera relative to right camera coordinate system)
    Transform stereoTransform_loaded = loaded.stereoTransform();
    Transform stereoTransform_original = original.stereoTransform();
    if(!stereoTransform_loaded.isNull() && !stereoTransform_original.isNull())
    {
        // Compare transform matrices element by element
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 4; ++j)
            {
                EXPECT_NEAR(stereoTransform_loaded.data()[i*4+j], stereoTransform_original.data()[i*4+j], 0.001);
            }
        }
        
        // Verify x value is negative baseline (left camera relative to right camera coordinate system)
        EXPECT_NEAR(stereoTransform_loaded.x(), -original.baseline(), 0.001);
        EXPECT_NEAR(stereoTransform_original.x(), -original.baseline(), 0.001);
    }
    
    // Verify local transform
    Transform localTransform_loaded = loaded.localTransform();
    Transform localTransform_original = original.localTransform();
    if(!localTransform_loaded.isNull() && !localTransform_original.isNull())
    {
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 4; ++j)
            {
                EXPECT_NEAR(localTransform_loaded.data()[i*4+j], localTransform_original.data()[i*4+j], 0.001);
            }
        }
    }
    
    // Verify validation states
    EXPECT_EQ(loaded.isValidForProjection(), original.isValidForProjection());
    EXPECT_EQ(loaded.isValidForRectification(), original.isValidForRectification());
}

TEST_F(StereoCameraModelTest, SaveLoadRoundTripIgnoreTransform)
{
    // Create a temporary directory for testing
    std::string testDir = "test_stereo_calibration2";
    UDirectory::makeDir(testDir);
    
    // Create original stereo camera model
    std::string cameraName = "test_stereo2";
    StereoCameraModel original(cameraName, fx_, fy_, cx_, cy_, baseline_);
    original.setImageSize(imageSize_);
    
    // Save the model (without stereo transform)
    bool saveResult = original.save(testDir, true);
    EXPECT_TRUE(saveResult);
    
    // Verify camera files were created (but not pose file)
    std::string leftFile = testDir + "/" + cameraName + "_left.yaml";
    std::string rightFile = testDir + "/" + cameraName + "_right.yaml";
    EXPECT_TRUE(UFile::exists(leftFile));
    EXPECT_TRUE(UFile::exists(rightFile));
    
    // Load the model back (ignoring stereo transform)
    StereoCameraModel loaded;
    bool loadResult = loaded.load(testDir, cameraName, true);
    EXPECT_TRUE(loadResult);
    
    // Verify parameters match
    EXPECT_EQ(loaded.name(), original.name());
    EXPECT_NEAR(loaded.baseline(), original.baseline(), 0.001);
    EXPECT_DOUBLE_EQ(loaded.left().fx(), original.left().fx());
    EXPECT_DOUBLE_EQ(loaded.right().fx(), original.right().fx());
}

TEST_F(StereoCameraModelTest, SaveLoadRoundTripMinimal)
{
    // Create a temporary directory for testing
    std::string testDir = "test_stereo_calibration3";
    UDirectory::makeDir(testDir);
    
    // Create minimal stereo camera model
    std::string cameraName = "minimal_stereo";
    StereoCameraModel original(cameraName, fx_, fy_, cx_, cy_, baseline_);
    original.setImageSize(imageSize_);
    
    // Save the model
    bool saveResult = original.save(testDir);
    EXPECT_TRUE(saveResult);
    
    // Load the model back
    StereoCameraModel loaded;
    bool loadResult = loaded.load(testDir, cameraName);
    EXPECT_TRUE(loadResult);
    
    // Verify parameters match
    EXPECT_EQ(loaded.name(), original.name());
    EXPECT_NEAR(loaded.baseline(), original.baseline(), 0.001);
    EXPECT_DOUBLE_EQ(loaded.left().fx(), original.left().fx());
    EXPECT_DOUBLE_EQ(loaded.right().fx(), original.right().fx());
    EXPECT_EQ(loaded.left().imageSize(), original.left().imageSize());
    EXPECT_EQ(loaded.right().imageSize(), original.right().imageSize());
}

TEST_F(StereoCameraModelTest, SaveStereoTransform)
{
    // Create a temporary directory for testing
    std::string testDir = "test_stereo_calibration4";
    UDirectory::makeDir(testDir);
    
    // Create stereo camera model with extrinsics
    std::string cameraName = "stereo_with_extrinsics";
    CameraModel left("left", imageSize_, K_, D_, R_, P_left_);
    CameraModel right("right", imageSize_, K_, D_, R_, P_right_);
    StereoCameraModel model(cameraName, left, right, R_stereo_, T_);
    
    // Save stereo transform separately
    bool saveResult = model.saveStereoTransform(testDir);
    EXPECT_TRUE(saveResult);
    
    // Verify pose file was created
    std::string poseFile = testDir + "/" + cameraName + "_pose.yaml";
    EXPECT_TRUE(UFile::exists(poseFile));
}

