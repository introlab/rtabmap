#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "rtabmap/core/StereoDense.h"
#include "rtabmap/core/stereo/StereoBM.h"
#include "rtabmap/core/stereo/StereoSGBM.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UException.h"
#include <memory>

using namespace rtabmap;

class StereoDenseTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create synthetic stereo images for testing
        // Generate random pattern for left image
        int imageWidth = 160;
        int imageHeight = 120;
        leftImage_ = cv::Mat(imageHeight, imageWidth, CV_8UC1);
        cv::randu(leftImage_, cv::Scalar(0), cv::Scalar(256));
        
        // Shift the entire left image left by shiftPixels_ to create the right image
        // Right image will have zeros on the right edge where there's no correspondence
        shiftPixels_ = 5;
        rightImage_ = cv::Mat::zeros(leftImage_.size(), CV_8UC1);
        cv::Rect leftROI(shiftPixels_, 0, leftImage_.cols - shiftPixels_, leftImage_.rows);
        cv::Rect rightROI(0, 0, leftImage_.cols - shiftPixels_, leftImage_.rows);
        leftImage_(leftROI).copyTo(rightImage_(rightROI));
        
        // Create color versions
#if CV_MAJOR_VERSION >= 3
        cv::cvtColor(leftImage_, leftImageColor_, cv::COLOR_GRAY2BGR);
        cv::cvtColor(rightImage_, rightImageColor_, cv::COLOR_GRAY2BGR);
#else
        cv::cvtColor(leftImage_, leftImageColor_, CV_GRAY2BGR);
        cv::cvtColor(rightImage_, rightImageColor_, CV_GRAY2BGR);
#endif
    }

    void TearDown() override {
    }

    cv::Mat leftImage_;
    cv::Mat rightImage_;
    cv::Mat leftImageColor_;
    cv::Mat rightImageColor_;
    int shiftPixels_;  ///< Number of pixels the right image is shifted left (expected disparity)
};

// Helper function to get strategy name for error messages
const char* getStrategyName(StereoDense::Type type) {
    switch(type) {
        case StereoDense::kTypeBM: return "BM";
        case StereoDense::kTypeSGBM: return "SGBM";
        default: return "Unknown";
    }
}

// StereoDense Tests

TEST_F(StereoDenseTest, Constructor)
{
    // Test with all stereo strategies
    StereoDense::Type strategies[] = {
        StereoDense::kTypeBM,
        StereoDense::kTypeSGBM
    };

    for(StereoDense::Type strategy : strategies)
    {
        ParametersMap params;
        
        // Set strategy-specific parameters
        if(strategy == StereoDense::kTypeBM)
        {
            params.insert(ParametersPair(Parameters::kStereoBMBlockSize(), "15"));
            params.insert(ParametersPair(Parameters::kStereoBMNumDisparities(), "64"));
        }
        else // kTypeSGBM
        {
            params.insert(ParametersPair(Parameters::kStereoSGBMBlockSize(), "5"));
            params.insert(ParametersPair(Parameters::kStereoSGBMNumDisparities(), "128"));
        }
        
        std::unique_ptr<StereoDense> stereo(StereoDense::create(strategy, params));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(strategy);
        
        // Test default constructor
        ParametersMap params2;
        std::unique_ptr<StereoDense> stereo2(StereoDense::create(strategy, params2));
        EXPECT_NE(stereo2.get(), nullptr) << "Strategy: " << getStrategyName(strategy);
    }
}

TEST_F(StereoDenseTest, ParseParameters)
{
    // Test with all stereo strategies
    StereoDense::Type strategies[] = {
        StereoDense::kTypeBM,
        StereoDense::kTypeSGBM
    };

    for(StereoDense::Type strategy : strategies)
    {
        std::unique_ptr<StereoDense> stereo(StereoDense::create(strategy));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(strategy);
        
        ParametersMap params;
        
        if(strategy == StereoDense::kTypeBM)
        {
            params.insert(ParametersPair(Parameters::kStereoBMBlockSize(), "21"));
            params.insert(ParametersPair(Parameters::kStereoBMMinDisparity(), "5"));
            params.insert(ParametersPair(Parameters::kStereoBMNumDisparities(), "128"));
            params.insert(ParametersPair(Parameters::kStereoBMPreFilterSize(), "11"));
            params.insert(ParametersPair(Parameters::kStereoBMPreFilterCap(), "63"));
            params.insert(ParametersPair(Parameters::kStereoBMUniquenessRatio(), "20"));
            params.insert(ParametersPair(Parameters::kStereoBMTextureThreshold(), "15"));
            params.insert(ParametersPair(Parameters::kStereoBMSpeckleWindowSize(), "200"));
            params.insert(ParametersPair(Parameters::kStereoBMSpeckleRange(), "8"));
            params.insert(ParametersPair(Parameters::kStereoBMDisp12MaxDiff(), "1"));
        }
        else // kTypeSGBM
        {
            params.insert(ParametersPair(Parameters::kStereoSGBMBlockSize(), "5"));
            params.insert(ParametersPair(Parameters::kStereoSGBMMinDisparity(), "5"));
            params.insert(ParametersPair(Parameters::kStereoSGBMNumDisparities(), "128"));
            params.insert(ParametersPair(Parameters::kStereoSGBMPreFilterCap(), "63"));
            params.insert(ParametersPair(Parameters::kStereoSGBMUniquenessRatio(), "20"));
            params.insert(ParametersPair(Parameters::kStereoSGBMSpeckleWindowSize(), "200"));
            params.insert(ParametersPair(Parameters::kStereoSGBMSpeckleRange(), "8"));
            params.insert(ParametersPair(Parameters::kStereoSGBMP1(), "100"));
            params.insert(ParametersPair(Parameters::kStereoSGBMP2(), "1000"));
            params.insert(ParametersPair(Parameters::kStereoSGBMDisp12MaxDiff(), "1"));
            params.insert(ParametersPair(Parameters::kStereoSGBMMode(), "0"));
        }
        
        stereo->parseParameters(params);
        // Should not throw
        EXPECT_TRUE(true) << "Strategy: " << getStrategyName(strategy);
    }
}

TEST_F(StereoDenseTest, ComputeDisparityGrayscale)
{
    // Test with all stereo strategies
    StereoDense::Type strategies[] = {
        StereoDense::kTypeBM,
        StereoDense::kTypeSGBM
    };

    StereoBM stereoBM;
    cv::Mat disparity = stereoBM.computeDisparity(leftImage_, rightImage_);

    for(StereoDense::Type strategy : strategies)
    {
        std::unique_ptr<StereoDense> stereo(StereoDense::create(strategy));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(strategy);
        
        cv::Mat disparity = stereo->computeDisparity(leftImage_, rightImage_);
        
        EXPECT_FALSE(disparity.empty()) << "Strategy: " << getStrategyName(strategy);
        EXPECT_EQ(disparity.rows, leftImage_.rows) << "Strategy: " << getStrategyName(strategy);
        EXPECT_EQ(disparity.cols, leftImage_.cols) << "Strategy: " << getStrategyName(strategy);
        EXPECT_EQ(disparity.type(), CV_16SC1) << "Strategy: " << getStrategyName(strategy);
        
        // Check that disparity is around shiftPixels_ (5 pixels) across the image
        // The entire left image was shifted left by shiftPixels_ to create the right image
        // In fixed-point format (4 fractional bits): shiftPixels_ * 16
        // Exclude the right edge where there's no correspondence (disparity = 0 or negative)
        cv::Rect validRegion(0, 0, disparity.cols - shiftPixels_, disparity.rows);
        cv::Mat validDisparity = disparity(validRegion);
        
        // Count valid disparities (positive values) in the valid region
        cv::Mat validMask = validDisparity > 0;
        int validCount = cv::countNonZero(validMask);
        
        // Should have many valid disparities in the valid region
        EXPECT_GT(validCount, 0) << "Strategy: " << getStrategyName(strategy);
        
        // Check that the mean disparity is approximately shiftPixels_ (5 pixels)
        // In fixed-point format: shiftPixels_ * 16 = 80
        // Allow some tolerance for stereo matching imperfections
        cv::Scalar meanDisparity = cv::mean(validDisparity, validMask);
        double expectedDisparity = shiftPixels_ * 16.0; // shiftPixels_ in fixed-point format
        EXPECT_NEAR(meanDisparity[0], expectedDisparity, 20.0) << "Strategy: " << getStrategyName(strategy); // Allow ±20 units tolerance (±1.25 pixels)
    }
}

TEST_F(StereoDenseTest, ComputeDisparityColor)
{
    // Test with all stereo strategies
    StereoDense::Type strategies[] = {
        StereoDense::kTypeBM,
        StereoDense::kTypeSGBM
    };

    for(StereoDense::Type strategy : strategies)
    {
        std::unique_ptr<StereoDense> stereo(StereoDense::create(strategy));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(strategy);
        
        cv::Mat disparity = stereo->computeDisparity(leftImageColor_, rightImageColor_);
        
        EXPECT_FALSE(disparity.empty()) << "Strategy: " << getStrategyName(strategy);
        EXPECT_EQ(disparity.rows, leftImageColor_.rows) << "Strategy: " << getStrategyName(strategy);
        EXPECT_EQ(disparity.cols, leftImageColor_.cols) << "Strategy: " << getStrategyName(strategy);
        EXPECT_EQ(disparity.type(), CV_16SC1) << "Strategy: " << getStrategyName(strategy);
        
        // Check that disparity is around shiftPixels_ (5 pixels) across the image
        // The entire left image was shifted left by shiftPixels_ to create the right image
        // In fixed-point format (4 fractional bits): shiftPixels_ * 16
        // Exclude the right edge where there's no correspondence (disparity = 0 or negative)
        cv::Rect validRegion(0, 0, disparity.cols - shiftPixels_, disparity.rows);
        cv::Mat validDisparity = disparity(validRegion);
        
        // Count valid disparities (positive values) in the valid region
        cv::Mat validMask = validDisparity > 0;
        int validCount = cv::countNonZero(validMask);
        
        // Should have many valid disparities in the valid region
        EXPECT_GT(validCount, 0) << "Strategy: " << getStrategyName(strategy);
        
        // Check that the mean disparity is approximately shiftPixels_ (5 pixels)
        // In fixed-point format: shiftPixels_ * 16 = 80
        // Allow some tolerance for stereo matching imperfections
        cv::Scalar meanDisparity = cv::mean(validDisparity, validMask);
        double expectedDisparity = shiftPixels_ * 16.0; // shiftPixels_ in fixed-point format
        EXPECT_NEAR(meanDisparity[0], expectedDisparity, 20.0) << "Strategy: " << getStrategyName(strategy); // Allow ±20 units tolerance (±1.25 pixels)
    }
}

TEST_F(StereoDenseTest, ComputeDisparityDifferentSizes)
{
    // Test with all stereo strategies
    StereoDense::Type strategies[] = {
        StereoDense::kTypeBM,
        StereoDense::kTypeSGBM
    };

    for(StereoDense::Type strategy : strategies)
    {
        std::unique_ptr<StereoDense> stereo(StereoDense::create(strategy));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(strategy);
        
        cv::Mat smallLeft = cv::Mat::zeros(75, 75, CV_8UC1);
        cv::Mat smallRight = cv::Mat::zeros(50, 50, CV_8UC1);
        
        // Should throw
        EXPECT_THROW(stereo->computeDisparity(smallLeft, smallRight), UException) << "Strategy: " << getStrategyName(strategy);
    }
}

TEST_F(StereoDenseTest, ComputeDisparityEmptyImages)
{
    // Test with all stereo strategies
    StereoDense::Type strategies[] = {
        StereoDense::kTypeBM,
        StereoDense::kTypeSGBM
    };

    for(StereoDense::Type strategy : strategies)
    {
        std::unique_ptr<StereoDense> stereo(StereoDense::create(strategy));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(strategy);
        
        cv::Mat emptyLeft, emptyRight;
        
        // Should throw 
        EXPECT_THROW(stereo->computeDisparity(emptyLeft, emptyRight), UException) << "Strategy: " << getStrategyName(strategy);
    }
}


