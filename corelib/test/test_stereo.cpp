#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "rtabmap/core/Stereo.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UException.h"
#include <memory>
#include <vector>

using namespace rtabmap;

class StereoTest : public ::testing::Test {
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
        
        // Detect feature points in the left image
        // Use a simple corner detector to get test points
        cv::Mat corners;
        cv::goodFeaturesToTrack(leftImage_, corners, 50, 0.01, 10);
        leftCorners_.clear();
        for(int i = 0; i < corners.rows; ++i)
        {
            leftCorners_.push_back(cv::Point2f(corners.at<float>(i, 0), corners.at<float>(i, 1)));
        }
        
        // Filter corners to be in valid region (not too close to edges and within valid disparity range)
        std::vector<cv::Point2f> filteredCorners;
        for(const cv::Point2f& pt : leftCorners_)
        {
            // Ensure corner is not too close to edges and will have valid correspondence
            if(pt.x >= shiftPixels_ + 10 && pt.x < leftImage_.cols - 10 &&
               pt.y >= 10 && pt.y < leftImage_.rows - 10)
            {
                filteredCorners.push_back(pt);
            }
        }
        leftCorners_ = filteredCorners;
        
        // Ensure we have some corners to test with
        if(leftCorners_.empty())
        {
            // Add some manual test points if no corners detected
            leftCorners_.push_back(cv::Point2f(50, 50));
            leftCorners_.push_back(cv::Point2f(80, 60));
            leftCorners_.push_back(cv::Point2f(100, 40));
        }
    }

    void TearDown() override {
    }

    cv::Mat leftImage_;
    cv::Mat rightImage_;
    std::vector<cv::Point2f> leftCorners_;
    int shiftPixels_;  ///< Number of pixels the right image is shifted left (expected disparity)
};

// Helper function to get strategy name for error messages
const char* getStrategyName(bool opticalFlow) {
    return opticalFlow ? "OpticalFlow" : "BlockMatching";
}

// Stereo Tests

TEST_F(StereoTest, Constructor)
{
    // Test with both strategies
    bool strategies[] = {false, true}; // false = BlockMatching, true = OpticalFlow

    for(bool opticalFlow : strategies)
    {
        ParametersMap params;
        params.insert(ParametersPair(Parameters::kStereoOpticalFlow(), opticalFlow ? "true" : "false"));
        
        std::unique_ptr<Stereo> stereo(Stereo::create(params));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(opticalFlow);
    }
}

TEST_F(StereoTest, ParseParameters)
{
    // Test with both strategies
    bool strategies[] = {false, true};

    for(bool opticalFlow : strategies)
    {
        ParametersMap params;
        params.insert(ParametersPair(Parameters::kStereoOpticalFlow(), opticalFlow ? "true" : "false"));
        params.insert(ParametersPair(Parameters::kStereoWinWidth(), "10"));
        params.insert(ParametersPair(Parameters::kStereoWinHeight(), "10"));
        params.insert(ParametersPair(Parameters::kStereoIterations(), "10"));
        params.insert(ParametersPair(Parameters::kStereoMaxLevel(), "2"));
        params.insert(ParametersPair(Parameters::kStereoMinDisparity(), "0.0"));
        params.insert(ParametersPair(Parameters::kStereoMaxDisparity(), "64.0"));
        params.insert(ParametersPair(Parameters::kStereoSSD(), "true"));
        params.insert(ParametersPair(Parameters::kStereoEps(), "0.01"));
        params.insert(ParametersPair(Parameters::kStereoGpu(), "false"));
        
        std::unique_ptr<Stereo> stereo(Stereo::create(params));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(opticalFlow);
        
        stereo->parseParameters(params);
        // Should not throw
        EXPECT_TRUE(true) << "Strategy: " << getStrategyName(opticalFlow);
    }
}

TEST_F(StereoTest, ComputeCorrespondences)
{
    // Test with both strategies
    bool strategies[] = {false, true};

    for(bool opticalFlow : strategies)
    {
        ParametersMap params;
        params.insert(ParametersPair(Parameters::kStereoOpticalFlow(), opticalFlow ? "true" : "false"));
        params.insert(ParametersPair(Parameters::kStereoMinDisparity(), "0.0"));
        params.insert(ParametersPair(Parameters::kStereoMaxDisparity(), "20.0")); // Should cover shiftPixels_ (5)
        
        std::unique_ptr<Stereo> stereo(Stereo::create(params));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(opticalFlow);
        
        std::vector<unsigned char> status;
        std::vector<cv::Point2f> rightCorners = stereo->computeCorrespondences(
            leftImage_, rightImage_, leftCorners_, status);
        
        EXPECT_EQ(rightCorners.size(), leftCorners_.size()) << "Strategy: " << getStrategyName(opticalFlow);
        EXPECT_EQ(status.size(), leftCorners_.size()) << "Strategy: " << getStrategyName(opticalFlow);
        
        // Count valid correspondences
        int validCount = 0;
        int correctDisparityCount = 0;
        for(size_t i = 0; i < status.size(); ++i)
        {
            if(status[i] != 0)
            {
                validCount++;
                // Check that disparity is approximately shiftPixels_
                float disparity = leftCorners_[i].x - rightCorners[i].x;
                if(std::abs(disparity - shiftPixels_) < 2.0f) // Allow 2 pixels tolerance
                {
                    correctDisparityCount++;
                }
            }
        }
        
        // Should have some valid correspondences
        EXPECT_GT(validCount, 0) << "Strategy: " << getStrategyName(opticalFlow);
        
        // Most valid correspondences should have correct disparity
        if(validCount > 0)
        {
            float correctRatio = static_cast<float>(correctDisparityCount) / validCount;
            EXPECT_GT(correctRatio, 0.5f) << "Strategy: " << getStrategyName(opticalFlow); // At least 50% should be correct
        }
    }
}

TEST_F(StereoTest, ComputeCorrespondencesEmptyCorners)
{
    // Test with both strategies
    bool strategies[] = {false, true};

    for(bool opticalFlow : strategies)
    {
        ParametersMap params;
        params.insert(ParametersPair(Parameters::kStereoOpticalFlow(), opticalFlow ? "true" : "false"));
        
        std::unique_ptr<Stereo> stereo(Stereo::create(params));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(opticalFlow);
        
        std::vector<cv::Point2f> emptyCorners;
        std::vector<unsigned char> status;
        
        std::vector<cv::Point2f> rightCorners;
        rightCorners = stereo->computeCorrespondences(
            leftImage_, rightImage_, emptyCorners, status);

        EXPECT_EQ(rightCorners.size(), 0u) << "Strategy: " << getStrategyName(opticalFlow);
        EXPECT_EQ(status.size(), 0u) << "Strategy: " << getStrategyName(opticalFlow);
    }
}

TEST_F(StereoTest, ComputeCorrespondencesDifferentSizes)
{
    // Test with both strategies
    bool strategies[] = {false, true};

    for(bool opticalFlow : strategies)
    {
        ParametersMap params;
        params.insert(ParametersPair(Parameters::kStereoOpticalFlow(), opticalFlow ? "true" : "false"));
        
        std::unique_ptr<Stereo> stereo(Stereo::create(params));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(opticalFlow);
        
        cv::Mat smallRight = cv::Mat::zeros(50, 50, CV_8UC1);
        std::vector<unsigned char> status;
        
        // Should throw
        EXPECT_THROW(stereo->computeCorrespondences(leftImage_, smallRight, leftCorners_, status), UException) 
        << "Strategy: " << getStrategyName(opticalFlow);
    }
}

TEST_F(StereoTest, ComputeCorrespondencesEmptyImages)
{
    // Test with both strategies
    bool strategies[] = {false, true};

    for(bool opticalFlow : strategies)
    {
        ParametersMap params;
        params.insert(ParametersPair(Parameters::kStereoOpticalFlow(), opticalFlow ? "true" : "false"));
        
        std::unique_ptr<Stereo> stereo(Stereo::create(params));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(opticalFlow);
        
        cv::Mat emptyLeft, emptyRight;
        std::vector<unsigned char> status;
        
        // Should throw
        EXPECT_THROW(stereo->computeCorrespondences(emptyLeft, emptyRight, leftCorners_, status), UException)
            << "Strategy: " << getStrategyName(opticalFlow);
    }
}

TEST_F(StereoTest, GetterMethods)
{
    // Test with both strategies
    bool strategies[] = {false, true};

    for(bool opticalFlow : strategies)
    {
        ParametersMap params;
        params.insert(ParametersPair(Parameters::kStereoOpticalFlow(), opticalFlow ? "true" : "false"));
        params.insert(ParametersPair(Parameters::kStereoWinWidth(), "12"));
        params.insert(ParametersPair(Parameters::kStereoWinHeight(), "8"));
        params.insert(ParametersPair(Parameters::kStereoIterations(), "15"));
        params.insert(ParametersPair(Parameters::kStereoMaxLevel(), "3"));
        params.insert(ParametersPair(Parameters::kStereoMinDisparity(), "1.0"));
        params.insert(ParametersPair(Parameters::kStereoMaxDisparity(), "50.0"));
        params.insert(ParametersPair(Parameters::kStereoSSD(), "false"));
        
        std::unique_ptr<Stereo> stereo(Stereo::create(params));
        EXPECT_NE(stereo.get(), nullptr) << "Strategy: " << getStrategyName(opticalFlow);
        
        cv::Size winSize = stereo->winSize();
        EXPECT_EQ(winSize.width, 12) << "Strategy: " << getStrategyName(opticalFlow);
        EXPECT_EQ(winSize.height, 8) << "Strategy: " << getStrategyName(opticalFlow);
        EXPECT_EQ(stereo->iterations(), 15) << "Strategy: " << getStrategyName(opticalFlow);
        EXPECT_EQ(stereo->maxLevel(), 3) << "Strategy: " << getStrategyName(opticalFlow);
        EXPECT_FLOAT_EQ(stereo->minDisparity(), 1.0f) << "Strategy: " << getStrategyName(opticalFlow);
        EXPECT_FLOAT_EQ(stereo->maxDisparity(), 50.0f) << "Strategy: " << getStrategyName(opticalFlow);
        EXPECT_FALSE(stereo->winSSD()) << "Strategy: " << getStrategyName(opticalFlow);
        EXPECT_FALSE(stereo->isGpuEnabled()) << "Strategy: " << getStrategyName(opticalFlow);
    }
}

TEST_F(StereoTest, StereoOpticalFlow_IsGpuEnabled)
{
    // Test GPU enable/disable for optical flow
    ParametersMap params;
    params.insert(ParametersPair(Parameters::kStereoOpticalFlow(), "true"));
    params.insert(ParametersPair(Parameters::kStereoGpu(), "false"));
    
    std::unique_ptr<Stereo> stereo(Stereo::create(params));
    EXPECT_NE(stereo.get(), nullptr);
    EXPECT_FALSE(stereo->isGpuEnabled());
    
    // Test with GPU enabled (may not be available)
    ParametersMap params2;
    params2.insert(ParametersPair(Parameters::kStereoOpticalFlow(), "true"));
    params2.insert(ParametersPair(Parameters::kStereoGpu(), "true"));
    
    std::unique_ptr<Stereo> stereo2(Stereo::create(params2));
    EXPECT_NE(stereo2.get(), nullptr);
    // GPU may or may not be enabled depending on build configuration
    // Just verify the method doesn't crash
    bool gpuEnabled = stereo2->isGpuEnabled();
    (void)gpuEnabled; // Suppress unused variable warning
}

TEST_F(StereoTest, StereoOpticalFlow_Epsilon)
{
    // Test epsilon parameter for optical flow
    ParametersMap params;
    params.insert(ParametersPair(Parameters::kStereoOpticalFlow(), "true"));
    params.insert(ParametersPair(Parameters::kStereoEps(), "0.005"));
    
    std::unique_ptr<Stereo> stereo(Stereo::create(params));
    EXPECT_NE(stereo.get(), nullptr);
    
    // Cast to StereoOpticalFlow to access epsilon()
    StereoOpticalFlow* opticalFlow = dynamic_cast<StereoOpticalFlow*>(stereo.get());
    EXPECT_NE(opticalFlow, nullptr);
    EXPECT_FLOAT_EQ(opticalFlow->epsilon(), 0.005f);
}

