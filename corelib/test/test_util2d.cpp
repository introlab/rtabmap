#include "gtest/gtest.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/UException.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "pcl/io/pcd_io.h"

using namespace rtabmap;

TEST(Util2dTest, ssdIdenticalImages)
{
    cv::Mat img1 = (cv::Mat_<uint8_t>(3,3) << 10, 20, 30, 40, 50, 60, 70, 80, 90);
    cv::Mat img2 = img1.clone();
    float score = util2d::ssd(img1, img2);
    EXPECT_FLOAT_EQ(score, 0.0f);
}

TEST(Util2dTest, ssdDifferentImages)
{
    cv::Mat img1 = (cv::Mat_<uint8_t>(2,2) << 10, 20, 30, 40);
    cv::Mat img2 = (cv::Mat_<uint8_t>(2,2) << 11, 19, 31, 39);
    float score = util2d::ssd(img1, img2);
    float expected = pow(10 - 11, 2) + pow(20 - 19, 2) + pow(30 - 31, 2) + pow(40 - 39, 2);
    EXPECT_FLOAT_EQ(score, expected);
}

TEST(Util2dTest, ssdStereoLikeInput)
{
    cv::Mat left = (cv::Mat_<cv::Vec2s>(1,1) << cv::Vec2s(10, 20));
    cv::Mat right = (cv::Mat_<cv::Vec2s>(1,1) << cv::Vec2s(11, 19));
    float avgL = (10 + 20) / 2.0f;
    float avgR = (11 + 19) / 2.0f;
    float expected = (avgL - avgR) * (avgL - avgR);
    EXPECT_FLOAT_EQ(util2d::ssd(left, right), expected);
}

TEST(Util2dTest, sadIdenticalImages)
{
    cv::Mat img1 = (cv::Mat_<float>(2,2) << 1.0, 2.0, 3.0, 4.0);
    cv::Mat img2 = img1.clone();
    float score = util2d::sad(img1, img2);
    EXPECT_FLOAT_EQ(score, 0.0f);
}

TEST(Util2dTest, sadDifferentImages)
{
    cv::Mat img1 = (cv::Mat_<float>(2,2) << 1.0, 2.0, 3.0, 4.0);
    cv::Mat img2 = (cv::Mat_<float>(2,2) << 0.5, 2.5, 2.5, 5.0);
    float score = fabs(1.0 - 0.5) + fabs(2.0 - 2.5) + fabs(3.0 - 2.5) + fabs(4.0 - 5.0);
    EXPECT_FLOAT_EQ(util2d::sad(img1, img2), score);
}

TEST(Util2dTest, sadStereoLikeInput)
{
    cv::Mat left = (cv::Mat_<cv::Vec2s>(1,1) << cv::Vec2s(5, 15));
    cv::Mat right = (cv::Mat_<cv::Vec2s>(1,1) << cv::Vec2s(10, 10));
    float avgL = (5 + 15) / 2.0f;
    float avgR = (10 + 10) / 2.0f;
    float expected = fabs(avgL - avgR);
    EXPECT_FLOAT_EQ(util2d::sad(left, right), expected);
}

TEST(Util2dTest, calcStereoCorrespondencesCheckInputs) {

    // Create synthetic images
    cv::Mat left = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(100, 100, CV_8UC1);

    std::vector<cv::Point2f> emptyCorners;
    std::vector<unsigned char> status;
    
    // Check inputs
    ASSERT_THROW(util2d::calcStereoCorrespondences(left, cv::Mat(), emptyCorners, status), UException);
    ASSERT_THROW(util2d::calcStereoCorrespondences(cv::Mat(), right, emptyCorners, status), UException);
    ASSERT_THROW(util2d::calcStereoCorrespondences(left, cv::Mat(2,2,CV_8UC1), emptyCorners, status), UException);
    ASSERT_THROW(util2d::calcStereoCorrespondences(left, cv::Mat(left.size(),CV_8UC3), emptyCorners, status), UException);
    ASSERT_THROW(util2d::calcStereoCorrespondences(left, right, emptyCorners, status, cv::Size(1,1)), cv::Exception);
    ASSERT_NO_THROW(util2d::calcStereoCorrespondences(left, right, emptyCorners, status, cv::Size(3,3)));
    ASSERT_THROW(util2d::calcStereoCorrespondences(left, cv::Mat(2,2,CV_8UC1), emptyCorners, status, cv::Size(6,3), 3, 5, -1.0f), UException);
    ASSERT_THROW(util2d::calcStereoCorrespondences(left, cv::Mat(2,2,CV_8UC1), emptyCorners, status, cv::Size(6,3), 3, 5, 10, 1), UException);

    // Check results
    ASSERT_TRUE(util2d::calcStereoCorrespondences(left, right, emptyCorners, status).empty());
}

TEST(Util2dTest, calcStereoCorrespondencesSSD) {

    // Create synthetic images
    cv::Mat left = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(100, 100, CV_8UC1);

    std::vector<unsigned char> status;

    // Draw white dots in both images
    cv::Point2f leftPoint(50, 50);
    cv::Point2f subPixelLeftPoint(25, 25);
    cv::Point2f wrongPoint(75, 75);

    left.at<uchar>((int)leftPoint.y, (int)leftPoint.x) = 255;
    right.at<uchar>((int)leftPoint.y, (int)leftPoint.x-5) = 255;

    left.at<uchar>((int)subPixelLeftPoint.y, (int)subPixelLeftPoint.x) = 64;
    left.at<uchar>((int)subPixelLeftPoint.y, (int)subPixelLeftPoint.x+1) = 191;
    right.at<uchar>((int)subPixelLeftPoint.y, (int)subPixelLeftPoint.x-6) = 255;

    left.at<uchar>((int)wrongPoint.y, (int)wrongPoint.x) = 75;

    std::vector<cv::Point2f> leftCorners = { leftPoint, subPixelLeftPoint, wrongPoint };

    // SSD
    std::vector<cv::Point2f> rightCorners = util2d::calcStereoCorrespondences(left, right, leftCorners, status, cv::Size(6,3), 3, 5, 0.0f, 64.0f, true);
    ASSERT_EQ(rightCorners.size(), 3);
    ASSERT_EQ(status.size(), 3);
    ASSERT_TRUE(status[0]);
    ASSERT_TRUE(status[1]);
    ASSERT_FALSE(status[2]);
    ASSERT_NEAR(leftCorners[0].x - rightCorners[0].x, 5.0f, 0.1f);
    ASSERT_NEAR(leftCorners[1].x - rightCorners[1].x, 6.75f, 0.1f);
}

TEST(Util2dTest, calcStereoCorrespondencesSAD) {

    // Create synthetic images
    cv::Mat left = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(100, 100, CV_8UC1);

    std::vector<unsigned char> status;

    // Draw white dots in both images
    cv::Point2f leftPoint(50, 50);
    cv::Point2f subPixelLeftPoint(25, 25);
    cv::Point2f wrongPoint(75, 75);

    left.at<uchar>((int)leftPoint.y, (int)leftPoint.x) = 255;
    right.at<uchar>((int)leftPoint.y, (int)leftPoint.x-5) = 255;

    left.at<uchar>((int)subPixelLeftPoint.y, (int)subPixelLeftPoint.x) = 64;
    left.at<uchar>((int)subPixelLeftPoint.y, (int)subPixelLeftPoint.x+1) = 191;
    right.at<uchar>((int)subPixelLeftPoint.y, (int)subPixelLeftPoint.x-6) = 255;

    left.at<uchar>((int)wrongPoint.y, (int)wrongPoint.x) = 75;

    std::vector<cv::Point2f> leftCorners = { leftPoint, subPixelLeftPoint, wrongPoint };

    // SAD
    std::vector<cv::Point2f> rightCorners = util2d::calcStereoCorrespondences(left, right, leftCorners, status, cv::Size(6,3), 3, 5, 0.0f, 64.0f, false);
    ASSERT_EQ(rightCorners.size(), 3);
    ASSERT_EQ(status.size(), 3);
    ASSERT_TRUE(status[0]);
    ASSERT_TRUE(status[1]);
    ASSERT_FALSE(status[2]);
    ASSERT_NEAR(leftCorners[0].x - rightCorners[0].x, 5.0f, 0.1f);
    ASSERT_NEAR(leftCorners[1].x - rightCorners[1].x, 6.75f, 0.1f);
}

TEST(Util2dTest, calcOpticalFlowPyrLKStereo)
{
    // Create synthetic images
    cv::Mat left = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(100, 100, CV_8UC1);

    // Draw a white dot in both images, shifted by 5 pixels in x-direction and 0.5 pixel in y-direction
    cv::Point2f leftPoint(50, 50);
    cv::Point2f rightPoint = leftPoint - cv::Point2f(5.0f, 0.5f);

    left.at<uchar>((int)leftPoint.y, (int)leftPoint.x) = 255;
    right.at<uchar>((int)rightPoint.y, (int)rightPoint.x) = 255;

    std::vector<cv::Point2f> prevPts = { leftPoint };
    std::vector<cv::Point2f> nextPts;

    cv::Mat status, err;

    // Run the stereo optical flow function
    util2d::calcOpticalFlowPyrLKStereo(
        left, right,
        prevPts, nextPts,
        status, err,
        cv::Size(21, 21),  // window size
        3,                 // max level
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        0,                 // no flags
        1e-4               // minEigThreshold
    );

    ASSERT_EQ(status.at<uchar>(0), 1) << "Optical flow failed.";
    ASSERT_NEAR(nextPts[0].y, leftPoint.y, 0.1f) << "Y position should remain constant.";
    ASSERT_NEAR(leftPoint.x - nextPts[0].x, 5.0f, 0.1f) << "X displacement should be close to 5.";
}

TEST(Util2dTest, disparityFromStereoImages)
{
    // Check inputs
    ASSERT_THROW(util2d::disparityFromStereoImages(cv::Mat(), cv::Mat(2,2,CV_8UC1)), UException);
    ASSERT_THROW(util2d::disparityFromStereoImages(cv::Mat(2,2,CV_8UC1), cv::Mat()), UException);
    ASSERT_THROW(util2d::disparityFromStereoImages(cv::Mat(2,2,CV_8UC1), cv::Mat(3,3,CV_8UC1)), UException);
    ASSERT_THROW(util2d::disparityFromStereoImages(cv::Mat(2,2,CV_8UC3), cv::Mat(2,2,CV_8UC3)), UException);

    cv::Mat left = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT)+"/stereo_rect/left/50.jpg");
    cv::Mat right = cv::imread(std::string(RTABMAP_TEST_DATA_ROOT)+"/stereo_rect/right/50.jpg", cv::IMREAD_GRAYSCALE);

    cv::Mat disparity = util2d::disparityFromStereoImages(left, right);

    EXPECT_FALSE(disparity.empty());
    EXPECT_EQ(disparity.size(), left.size());
    EXPECT_EQ(disparity.type(), CV_16SC1);

    cv::Rect rect(300, 400, 40, 40);
    cv::Mat roi = disparity(rect);

    // Check if all values in ROI are close to 10 (float comparison)
    double minVal, maxVal;
    cv::minMaxLoc(roi, &minVal, &maxVal);

    EXPECT_NEAR(minVal, 313, 0.5);
    EXPECT_NEAR(maxVal, 388, 0.5);
}

TEST(Util2dTest, depthFromDisparityFloat32) {
    const int rows = 2;
    const int cols = 3;
    const float fx = 500.0f;         // focal length in pixels
    const float baseline = 0.1f;     // 10 cm baseline
    const float disparityValue = 25.0f;

    cv::Mat disparity(rows, cols, CV_32FC1, cv::Scalar(disparityValue));
    cv::Mat depth = util2d::depthFromDisparity(disparity, fx, baseline, CV_32FC1);

    ASSERT_EQ(depth.rows, rows);
    ASSERT_EQ(depth.cols, cols);
    ASSERT_EQ(depth.type(), CV_32FC1);

    float expectedDepth = baseline * fx / disparityValue; // = 0.1 * 500 / 25 = 2.0
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            EXPECT_NEAR(depth.at<float>(i, j), expectedDepth, 1e-5);
        }
    }
}

TEST(Util2dTest, depthFromDisparityUInt16) {
    const int rows = 2;
    const int cols = 3;
    const float fx = 500.0f;
    const float baseline = 0.1f;
    const float disparityValue = 25.0f;

    cv::Mat disparity(rows, cols, CV_32FC1, cv::Scalar(disparityValue));
    cv::Mat depth = util2d::depthFromDisparity(disparity, fx, baseline, CV_16UC1);

    ASSERT_EQ(depth.rows, rows);
    ASSERT_EQ(depth.cols, cols);
    ASSERT_EQ(depth.type(), CV_16UC1);

    unsigned short expectedDepth = static_cast<unsigned short>((baseline * fx / disparityValue) * 1000); // in mm
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            EXPECT_EQ(depth.at<unsigned short>(i, j), expectedDepth);
        }
    }
}

TEST(Util2dTest, depthFromStereoImages)
{
    // Parameters
    const int width = 20;
    const int height = 20;
    float fx = 100.0f;          // focal length in pixels
    float baseline = 0.1f;      // 10 cm
    int flowWinSize = 5;
    int flowMaxLevel = 3;
    int flowIterations = 10;
    double flowEps = 0.01;

    // Create synthetic stereo images
    cv::Mat leftImage = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat rightImage = cv::Mat::zeros(height, width, CV_8UC1);

    // Place a dot at (10,10) in the left image and simulate a disparity of 5 pixels
    leftImage.at<uchar>(10, 10) = 255;
    rightImage.at<uchar>(10, 5) = 255;

    // Known point to track
    std::vector<cv::Point2f> leftCorners = {cv::Point2f(10.0f, 10.0f)};

    // Call the function under test
    cv::Mat depth = util2d::depthFromStereoImages(
        leftImage,
        rightImage,
        leftCorners,
        fx,
        baseline,
        flowWinSize,
        flowMaxLevel,
        flowIterations,
        flowEps
    );

    // Check result
    float expectedDisparity = 5.0f;
    float expectedDepth = (fx * baseline) / expectedDisparity;
    float actualDepth = depth.at<float>(10, 10);

    ASSERT_NEAR(actualDepth, expectedDepth, 1e-3);
}

TEST(Util2dTest, disparityFromStereoCorrespondencesDisparityComputation)
{
    // Test setup
    cv::Size disparitySize(5, 5);
    std::vector<cv::Point2f> leftCorners = {
        cv::Point2f(1.0f, 1.0f), 
        cv::Point2f(2.0f, 1.0f), 
        cv::Point2f(3.0f, 1.0f), 
        cv::Point2f(4.0f, 1.0f)
    };
    std::vector<cv::Point2f> rightCorners = {
        cv::Point2f(0.5f, 1.0f), 
        cv::Point2f(1.5f, 1.0f), 
        cv::Point2f(2.5f, 1.0f), 
        cv::Point2f(3.5f, 1.0f)
    };
    std::vector<unsigned char> mask = {1, 1, 1, 1};

    // Call the function to test
    cv::Mat disparity = util2d::disparityFromStereoCorrespondences(disparitySize, leftCorners, rightCorners, mask);

    // Verify the disparity values
    EXPECT_EQ(disparity.at<float>(1, 1), 0.5f);  // 1.0f - 0.5f = 0.5f
    EXPECT_EQ(disparity.at<float>(2, 1), 0.5f);  // 2.0f - 1.5f = 0.5f
    EXPECT_EQ(disparity.at<float>(3, 1), 0.5f);  // 3.0f - 2.5f = 0.5f
    EXPECT_EQ(disparity.at<float>(4, 1), 0.5f);  // 4.0f - 3.5f = 0.5f

    // Check that locations with no disparity (e.g., the first col) remain 0
    EXPECT_EQ(disparity.at<float>(0, 0), 0.0f);
    EXPECT_EQ(disparity.at<float>(1, 0), 0.0f);
    EXPECT_EQ(disparity.at<float>(2, 0), 0.0f);
    EXPECT_EQ(disparity.at<float>(3, 0), 0.0f);
    EXPECT_EQ(disparity.at<float>(4, 0), 0.0f);
}

TEST(Util2dTest, disparityFromStereoCorrespondencesEmptyMask)
{
    // Test with empty mask (all points should be included)
    cv::Size disparitySize(4, 4);
    std::vector<cv::Point2f> leftCorners = {
        cv::Point2f(1.0f, 1.0f),
        cv::Point2f(2.0f, 1.0f),
        cv::Point2f(3.0f, 1.0f)
    };
    std::vector<cv::Point2f> rightCorners = {
        cv::Point2f(0.5f, 1.0f),
        cv::Point2f(1.5f, 1.0f),
        cv::Point2f(2.5f, 1.0f)
    };
    std::vector<unsigned char> mask;

    // Call the function
    cv::Mat disparity = util2d::disparityFromStereoCorrespondences(disparitySize, leftCorners, rightCorners, mask);

    // Verify the disparity values for all points
    EXPECT_EQ(disparity.at<float>(1, 1), 0.5f);  // 1.0f - 0.5f = 0.5f
    EXPECT_EQ(disparity.at<float>(2, 1), 0.5f);  // 2.0f - 1.5f = 0.5f
    EXPECT_EQ(disparity.at<float>(3, 1), 0.5f);  // 3.0f - 2.5f = 0.5f
}

TEST(Util2dTest, disparityFromStereoCorrespondencesInconsistentCornersSize)
{
    // Test with inconsistent corners size (should fail)
    cv::Size disparitySize(5, 5);
    std::vector<cv::Point2f> leftCorners = {
        cv::Point2f(1.0f, 1.0f),
        cv::Point2f(2.0f, 1.0f)
    };
    std::vector<cv::Point2f> rightCorners = {
        cv::Point2f(0.5f, 1.0f)
    };
    std::vector<unsigned char> mask = {1};

    ASSERT_THROW(util2d::disparityFromStereoCorrespondences(disparitySize, leftCorners, rightCorners, mask), UException);
    rightCorners.push_back(cv::Point2f(1.5f,1.0f));
    ASSERT_THROW(util2d::disparityFromStereoCorrespondences(disparitySize, leftCorners, rightCorners, mask), UException);
    mask.push_back(1);
    leftCorners.back().x = -2;
    ASSERT_THROW(util2d::disparityFromStereoCorrespondences(disparitySize, leftCorners, rightCorners, mask), UException);
    leftCorners.back().x = 6;
    ASSERT_THROW(util2d::disparityFromStereoCorrespondences(disparitySize, leftCorners, rightCorners, mask), UException);
}

TEST(Util2dTest, depthFromStereoCorrespondences)
{
    // Create a dummy left image (only used for size)
    cv::Mat leftImage = cv::Mat::zeros(10, 10, CV_8UC1);

    // Define synthetic stereo correspondences
    std::vector<cv::Point2f> leftCorners = {
        cv::Point2f(5.0f, 5.0f)
    };
    std::vector<cv::Point2f> rightCorners = {
        cv::Point2f(2.0f, 5.0f) // disparity = 3.0
    };
    std::vector<unsigned char> mask = {1};

    // Intrinsics
    float fx = 100.0f;        // focal length in pixels
    float baseline = 0.1f;    // 10 cm

    // Expected depth = fx * baseline / disparity = 100 * 0.1 / 3 = 3.333...
    float expectedDepth = (fx * baseline) / (leftCorners[0].x - rightCorners[0].x);

    // Call function
    cv::Mat depth = util2d::depthFromStereoCorrespondences(leftImage, leftCorners, rightCorners, mask, fx, baseline);

    // Check result
    ASSERT_EQ(depth.type(), CV_32FC1);
    float actualDepth = depth.at<float>(5, 5); // rounded position from input
    ASSERT_NEAR(actualDepth, expectedDepth, 1e-4);
}

TEST(Util2dTest, cvtDepthFromFloat) {
    // Create a simple 3x3 depth image in meters
    cv::Mat depth32F = (cv::Mat_<float>(3, 3) <<
        0.5f, 1.0f, 1.5f,
        2.0f, 3.0f, 4.0f,
        5.0f, 6.0f, 7.0f); // in meters

    // Convert to 16-bit unsigned short (mm)
    cv::Mat depth16U = util2d::cvtDepthFromFloat(depth32F);

    ASSERT_EQ(depth16U.type(), CV_16UC1);
    ASSERT_EQ(depth16U.rows, 3);
    ASSERT_EQ(depth16U.cols, 3);

    EXPECT_EQ(depth16U.at<unsigned short>(0, 0), 500);   // 0.5m -> 500mm
    EXPECT_EQ(depth16U.at<unsigned short>(0, 1), 1000);  // 1.0m -> 1000mm
    EXPECT_EQ(depth16U.at<unsigned short>(2, 2), 7000);  // 7.0m -> 7000mm
}

TEST(Util2dTest, cvtDepthToFloat) {
    // Create a simple 3x3 depth image in millimeters
    cv::Mat depth16U = (cv::Mat_<unsigned short>(3, 3) <<
        500, 1000, 1500,
        2000, 3000, 4000,
        5000, 6000, 7000); // in mm

    // Convert to 32-bit float (meters)
    cv::Mat depth32F = util2d::cvtDepthToFloat(depth16U);

    ASSERT_EQ(depth32F.type(), CV_32FC1);
    ASSERT_EQ(depth32F.rows, 3);
    ASSERT_EQ(depth32F.cols, 3);

    EXPECT_FLOAT_EQ(depth32F.at<float>(0, 0), 0.5f);   // 500mm -> 0.5m
    EXPECT_FLOAT_EQ(depth32F.at<float>(0, 1), 1.0f);   // 1000mm -> 1.0m
    EXPECT_FLOAT_EQ(depth32F.at<float>(2, 2), 7.0f);   // 7000mm -> 7.0m
}

TEST(Util2dTest, cvtDepthRoundTripConversion) {
    // Test round-trip conversion
    cv::Mat original = (cv::Mat_<float>(2, 2) << 0.25f, 1.0f, 2.5f, 3.3f);

    cv::Mat depth16U = util2d::cvtDepthFromFloat(original);
    cv::Mat depth32F = util2d::cvtDepthToFloat(depth16U);

    for (int i = 0; i < original.rows; ++i) {
        for (int j = 0; j < original.cols; ++j) {
            float expected = std::floor(original.at<float>(i, j) * 1000.0f + 0.5f) / 1000.0f;
            EXPECT_NEAR(depth32F.at<float>(i, j), expected, 0.001);
        }
    }
}

TEST(Util2dTest, getDepthCenterDepthValue32FNoSmoothingNoEstimation) {
    cv::Mat depth = cv::Mat::zeros(5, 5, CV_32FC1);
    depth.at<float>(2, 2) = 1.5f;

    float result = util2d::getDepth(depth, 2.0f, 2.0f, false, 0.1f, false);
    EXPECT_FLOAT_EQ(result, 1.5f);
}

TEST(Util2dTest, getDepthCenterDepthValue16UNoSmoothingNoEstimation) {
    cv::Mat depth = cv::Mat::zeros(5, 5, CV_16UC1);
    depth.at<unsigned short>(2, 2) = 1500; // 1.5 meters

    float result = util2d::getDepth(depth, 2.0f, 2.0f, false, 0.1f, false);
    EXPECT_FLOAT_EQ(result, 1.5f);
}

TEST(Util2dTest, getDepthSmoothing32F) {
    cv::Mat depth = cv::Mat::zeros(5, 5, CV_32FC1);
    depth.at<float>(2, 2) = 1.0f;
    depth.at<float>(2, 1) = 1.0f;
    depth.at<float>(1, 2) = 1.0f;
    depth.at<float>(2, 3) = 1.0f;
    depth.at<float>(3, 2) = 1.0f;

    float result = util2d::getDepth(depth, 2.0f, 2.0f, true, 0.1f, false);
    EXPECT_NEAR(result, 1.0f, 1e-5f);
}

TEST(Util2dTest, getDepthEstimationFromNeighbors16U) {
    cv::Mat depth = cv::Mat::zeros(5, 5, CV_16UC1);
    depth.at<unsigned short>(2, 1) = 1500; // 1.5m
    depth.at<unsigned short>(1, 2) = 1500;

    float result = util2d::getDepth(depth, 2.0f, 2.0f, false, 0.1f, true);
    EXPECT_NEAR(result, 1.5f, 1e-3f);
}

TEST(Util2dTest, getDepthOutOfBounds) {
    cv::Mat depth = cv::Mat::ones(5, 5, CV_32FC1);

    float result = util2d::getDepth(depth, 5.5f, 5.5f, false, 0.1f, false);
    EXPECT_FLOAT_EQ(result, 0.0f);
}

TEST(Util2dTest, computeRoiValidStringInput)
{
    cv::Size imageSize(200, 100);
    cv::Mat image = cv::Mat::zeros(imageSize, CV_8UC1);
    cv::Rect roi = util2d::computeRoi(image, "0.1 0.1 0.2 0.2");
    EXPECT_EQ(roi.x, 20);
    EXPECT_EQ(roi.width, 160);
    EXPECT_EQ(roi.y, 20);
    EXPECT_EQ(roi.height, 60);
    roi = util2d::computeRoi(imageSize, "0.1 0.1 0.2 0.2");
    EXPECT_EQ(roi.x, 20);
    EXPECT_EQ(roi.width, 160);
    EXPECT_EQ(roi.y, 20);
    EXPECT_EQ(roi.height, 60);
}

TEST(Util2dTest, computeRoiInvalidStringInput)
{
    cv::Mat image = cv::Mat::zeros(100, 200, CV_8UC1);
    cv::Rect roi = util2d::computeRoi(image, "0.5 0.6"); // Invalid format
    EXPECT_EQ(roi, cv::Rect()); // Expect empty ROI
}

TEST(Util2dTest, computeRoiValidVectorInput)
{
    cv::Size imageSize(300, 150);
    cv::Mat image = cv::Mat::zeros(imageSize, CV_8UC1);
    std::vector<float> ratios = {0.1f, 0.2f, 0.1f, 0.1f};
    cv::Rect roi = util2d::computeRoi(image, ratios);
    EXPECT_EQ(roi.x, 30);
    EXPECT_EQ(roi.width, 210);
    EXPECT_EQ(roi.y, 15);
    EXPECT_EQ(roi.height, 120);
    roi = util2d::computeRoi(imageSize, ratios);
    EXPECT_EQ(roi.x, 30);
    EXPECT_EQ(roi.width, 210);
    EXPECT_EQ(roi.y, 15);
    EXPECT_EQ(roi.height, 120);
}

TEST(Util2dTest, computeRoiInvalidVectorSize)
{
    cv::Size imageSize(300, 150);
    std::vector<float> invalidRatios = {0.1f, 0.2f}; // Invalid size
    cv::Rect roi = util2d::computeRoi(imageSize, invalidRatios);
    EXPECT_EQ(roi, cv::Rect());
}

TEST(Util2dTest, computeRoiZeroImageSize)
{
    cv::Size imageSize(0, 0);
    cv::Mat image = cv::Mat::zeros(imageSize, CV_8UC1);
    std::vector<float> ratios = {0.1f, 0.1f, 0.1f, 0.1f};
    cv::Rect roi = util2d::computeRoi(image, ratios);
    EXPECT_EQ(roi, cv::Rect());
    roi = util2d::computeRoi(imageSize, ratios);
    EXPECT_EQ(roi, cv::Rect());
}

TEST(Util2dTest, decimateFloatDepthImage)
{
    // Create a 4x4 depth image with increasing values
    cv::Mat depth = (cv::Mat_<float>(4, 4) <<
        1, 2, 3, 4,
        5, 6, 7, 8,
        9,10,11,12,
       13,14,15,16);

    // Decimate by 2
    cv::Mat decimated = util2d::decimate(depth, 2);

    ASSERT_EQ(decimated.rows, 2);
    ASSERT_EQ(decimated.cols, 2);
    EXPECT_FLOAT_EQ(decimated.at<float>(0,0), 1);
    EXPECT_FLOAT_EQ(decimated.at<float>(0,1), 3);
    EXPECT_FLOAT_EQ(decimated.at<float>(1,0), 9);
    EXPECT_FLOAT_EQ(decimated.at<float>(1,1), 11);
}

TEST(Util2dTest, decimate16UDepthImage)
{
    cv::Mat depth = (cv::Mat_<uint16_t>(4, 4) <<
        100, 200, 300, 400,
        500, 600, 700, 800,
        900,1000,1100,1200,
       1300,1400,1500,1600);

    cv::Mat decimated = util2d::decimate(depth, 2);

    ASSERT_EQ(decimated.rows, 2);
    ASSERT_EQ(decimated.cols, 2);
    EXPECT_EQ(decimated.at<uint16_t>(0,0), 100);
    EXPECT_EQ(decimated.at<uint16_t>(0,1), 300);
    EXPECT_EQ(decimated.at<uint16_t>(1,0), 900);
    EXPECT_EQ(decimated.at<uint16_t>(1,1), 1100);
}

TEST(Util2dTest, interpolateFloatDepthImage)
{
    // Create a simple 2x2 image to interpolate into 4x4
    cv::Mat input = (cv::Mat_<float>(2,2) <<
        1.0f, 1.0f,
        1.0f, 1.0f);

    int factor = 2;
    float depthErrorRatio = 0.1f;

    cv::Mat interpolated = util2d::interpolate(input, factor, depthErrorRatio);

    ASSERT_EQ(interpolated.rows, 4);
    ASSERT_EQ(interpolated.cols, 4);

    // All interpolated values should still be 1.0f
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            EXPECT_FLOAT_EQ(interpolated.at<float>(r, c), 1.0f);
        }
    }
}

TEST(Util2dTest, interpolate16UDepthImage)
{
    cv::Mat input = (cv::Mat_<uint16_t>(2,2) <<
        1000, 1000,
        1000, 1000);

    int factor = 2;
    float depthErrorRatio = 0.1f;

    cv::Mat interpolated = util2d::interpolate(input, factor, depthErrorRatio);

    ASSERT_EQ(interpolated.rows, 4);
    ASSERT_EQ(interpolated.cols, 4);

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            EXPECT_EQ(interpolated.at<uint16_t>(r, c), 1000);
        }
    }
}

TEST(Util2dTest, registerDepth)
{
    // Create a 2x2 synthetic depth image in meters
    cv::Mat depth = (cv::Mat_<float>(2,2) << 1.0f, 1.0f,
                                             1.0f, 1.0f);
    cv::Mat depthK = (cv::Mat_<double>(3,3) <<
                      1.0, 0, 0.5,
                      0, 1.0, 0.5,
                      0, 0, 1.0);

    // RGB image size (same size for simplicity)
    cv::Size colorSize = depth.size();
    cv::Mat colorK = depthK.clone();

    // Identity transform (depth and color cameras are perfectly aligned)
    rtabmap::Transform transform = rtabmap::Transform::getIdentity(); // default is identity

    // Run registration
    cv::Mat registered = util2d::registerDepth(depth, depthK, colorSize, colorK, transform);

    // Check that the output is the same as input since transform is identity and intrinsics match
    ASSERT_EQ(registered.type(), depth.type());
    ASSERT_EQ(registered.size(), colorSize);

    for (int y = 0; y < registered.rows; ++y)
    {
        for (int x = 0; x < registered.cols; ++x)
        {
            EXPECT_FLOAT_EQ(registered.at<float>(y, x), 1.0f);
        }
    }
}

TEST(Util2dTest, registerDepthWithOverlap)
{
    cv::Mat depth = cv::Mat::ones(11,11,CV_32FC1);
    cv::Mat depthK = (cv::Mat_<double>(3,3) <<
                      10.0, 0, 5,
                      0, 10.0, 5,
                      0, 0, 1.0);

    cv::Size colorSize = depth.size();
    cv::Mat colorK = depthK;

    // move the color camera right and rotate to left (optical frame) 
    rtabmap::Transform transform = rtabmap::Transform(1.5,0.0,1.0,0,-M_PI/2,0);

    // Run registration
    cv::Mat registered = util2d::registerDepth(depth, depthK, colorSize, colorK, transform.inverse());

    ASSERT_EQ(registered.type(), depth.type());
    ASSERT_EQ(registered.size(), colorSize);

    // last column from original should match the middle column of registered
    // all other columns should be 0
    for (int y = 0; y < registered.rows; ++y)
    {
        for (int x = 0; x < registered.cols; ++x)
        {
            EXPECT_FLOAT_EQ(registered.at<float>(y, x), x != registered.cols/2 ? 0.0f : 1.0f);
        }
    }
}

TEST(Util2dTest, fillDepthHoles) {
    cv::Mat depth = (cv::Mat_<float>(3,3) <<
                      1, 0, 2,
                      0, 3, 0,
                      2, 0, 4);

    int maximumHoleSize = 2;
    float errorRatio = 1.0f;

    cv::Mat filledDepth = util2d::fillDepthHoles(depth, maximumHoleSize, errorRatio);

    EXPECT_EQ(filledDepth.size(), depth.size());
    EXPECT_EQ(filledDepth.type(), depth.type());

    cv::Mat expected = (cv::Mat_<float>(3,3) <<
            1, 1.5, 2,
            1.5, 3, 0,
            2, 0, 4);

    for (int i = 0; i < filledDepth.rows; i++) {
        for (int j = 0; j < filledDepth.cols; j++) {
            EXPECT_EQ(filledDepth.at<float>(i, j), expected.at<float>(i,j));
        }
    }
}

TEST(Util2dTest, fillDepthHolesLargeHoleTest) {

    cv::Mat depth = (cv::Mat_<float>(5,5) <<
                        1, 0, 2, 3, 5,
                        0, 3, 0, 0, 4,
                        2, 0, 4, 2, 3,
                        1, 0, 0, 0, 2,
                        0, 3, 0, 9, 7);
    cv::Mat expected = (cv::Mat_<float>(5,5) <<
                        1, 1.5, 2, 3, 5,
                        1.5, 3, 3.16, 3.08, 4,
                        2, 3, 4, 2, 3,
                        1, 3, 0, 0, 2,
                        0, 3, 0, 9, 7);

    int maximumHoleSize = 2;
    float errorRatio = 1.0f;

    cv::Mat filledDepth = util2d::fillDepthHoles(depth, maximumHoleSize, errorRatio);

    EXPECT_EQ(filledDepth.size(), depth.size());
    EXPECT_EQ(filledDepth.type(), depth.type());

    for (int i = 0; i < filledDepth.rows; i++) {
        for (int j = 0; j < filledDepth.cols; j++) {
            EXPECT_NEAR(filledDepth.at<float>(i, j), expected.at<float>(i,j), 0.1f);
        }
    }
}

TEST(Util2dTest, fillDepthHolesInvalidInputTest) {
    cv::Mat depth(5, 5, CV_32FC1);

    // Test with invalid maximumHoleSize (<= 0)
    int maximumHoleSize = 0;
    float errorRatio = 0.1f;
    EXPECT_THROW(util2d::fillDepthHoles(depth, maximumHoleSize, errorRatio), UException);
}

cv::Mat createTestDepthImage()
{
    cv::Mat img = (cv::Mat_<unsigned short>(5, 5) <<
        1000,    0, 1010,    0, 1020,
           0,    0,    0,    0,    0,
        1005,    0, 1015,    0, 1025,
           0,    0,    0,    0,    0,
        1010,    0, 1020,    0, 1030);
    return img;
}

TEST(Util2dTest, fillRegisteredDepthHolesVerticalFilling)
{
    cv::Mat input = createTestDepthImage();
    cv::Mat original = input.clone();

    util2d::fillRegisteredDepthHoles(input, true, false, false); // Vertical only

    for (int i = 0; i < input.rows; i++) {
        for (int j = 0; j < input.cols; j++) {
            if((i==1 && j==2) || (i==3 && j==2)) {
                // only middle column should have now all valid values
                EXPECT_GT(input.at<unsigned short>(i, j), 0);
            }
            else {
                EXPECT_EQ(input.at<unsigned short>(i, j), original.at<unsigned short>(i,j));
            }
        }
    }
}

TEST(Util2dTest, fillRegisteredDepthHolesHorizontalFilling)
{
    cv::Mat input = createTestDepthImage();
    cv::Mat original = input.clone();

    util2d::fillRegisteredDepthHoles(input, false, true, false); // Horizontal only

    for (int i = 0; i < input.rows; i++) {
        for (int j = 0; j < input.cols; j++) {
            if((i==2 && j==1) || (i==2 && j==3)) {
                // only middle row should have now all valid values
                EXPECT_GT(input.at<unsigned short>(i, j), 0);
            }
            else {
                EXPECT_EQ(input.at<unsigned short>(i, j), original.at<unsigned short>(i,j));
            }
        }
    }
}

TEST(Util2dTest, fillRegisteredDepthHolesDoubleHoleFilling)
{
    cv::Mat input = (cv::Mat_<unsigned short>(5, 5) <<
        1000, 1005,    0, 1020, 1030,
           0, 1010,    0,    0, 1015,
        1010,    0,    0,    0, 1020,
        1015,    0,    0, 1015, 1025,
        1010, 1015, 1020, 1025, 1030);
    
    cv::Mat expected = (cv::Mat_<unsigned short>(5, 5) <<
        1000, 1005,    0, 1020, 1030,
           0, 1010, 1011, 1013, 1015,
        1010, 1011, 1013,    0, 1020, // <- Note that when double filling is on, there is a double contour
        1015, 1013, 1017, 1015, 1025,
        1010, 1015, 1020, 1025, 1030);

    util2d::fillRegisteredDepthHoles(input, true, true, true); // Fill double holes too

    for (int i = 0; i < input.rows; i++) {
        for (int j = 0; j < input.cols; j++) {
            EXPECT_EQ(input.at<unsigned short>(i, j), expected.at<unsigned short>(i,j));
        }
    }
}

TEST(Util2dTest, fastBilateralFilteringEmptyInput) {
    cv::Mat empty;
    EXPECT_THROW(util2d::fastBilateralFiltering(empty, 2.0f, 0.1f, false), UException);
}

TEST(Util2dTest, fastBilateralFilteringAllZeroInput) {
    cv::Mat depth = cv::Mat::zeros(10, 10, CV_32FC1);
    cv::Mat result = util2d::fastBilateralFiltering(depth, 2.0f, 0.1f, false);
    // All values should still be 0
    for (int i = 0; i < result.rows; ++i) {
        for (int j = 0; j < result.cols; ++j) {
            EXPECT_EQ(result.at<float>(i, j), 0.0f);
        }
    }
}

TEST(Util2dTest, fastBilateralFilteringSimpleFloatInput) {
    cv::Mat depth = cv::Mat::ones(5, 5, CV_32FC1) * 1.0f;
    depth.at<float>(2,2) = 1.05f;
    cv::Mat result = util2d::fastBilateralFiltering(depth, 3.0f, 0.05f, false);

    ASSERT_FALSE(result.empty());
    EXPECT_EQ(result.type(), CV_32FC1);
    EXPECT_EQ(result.rows, depth.rows);
    EXPECT_EQ(result.cols, depth.cols);

    // All values should still be close to 1.0 (since no variation)
    for (int i = 0; i < result.rows; ++i) {
        for (int j = 0; j < result.cols; ++j) {
            EXPECT_NEAR(result.at<float>(i, j), 1.0f, 1e-2);
        }
    }
}

TEST(Util2dTest, fastBilateralFilteringSimpleUShortInput) {
    cv::Mat depth = cv::Mat::ones(5, 5, CV_16UC1) * 1000; // 1 meter
    depth.at<unsigned short>(2,2) = 1050;
    cv::Mat result = util2d::fastBilateralFiltering(depth, 2.0f, 0.05f, true);

    ASSERT_FALSE(result.empty());
    EXPECT_EQ(result.type(), CV_32FC1);
    EXPECT_EQ(result.rows, depth.rows);
    EXPECT_EQ(result.cols, depth.cols);

    for (int i = 0; i < result.rows; ++i) {
        for (int j = 0; j < result.cols; ++j) {
            EXPECT_NEAR(result.at<float>(i, j), 1.0f, 1e-2);
        }
    }
}

// High Depth Variation (should preserve edge due to sigmaR)
TEST(Util2dTest, fastBilateralFilteringPreservesEdgesWithLowSigmaR) {
    // Create a step edge: left side = 1.0f, right side = 3.0f
    cv::Mat depth = cv::Mat::ones(5, 10, CV_32FC1);
    depth.colRange(5, 10).setTo(3.0f);
    depth.at<float>(2,7) = 3.1f;
    depth.at<float>(2,2) = 1.1f;

    float sigmaS = 2.0f;
    float sigmaR = 0.1f; // Low sigmaR to preserve edge

    cv::Mat result = util2d::fastBilateralFiltering(depth, sigmaS, sigmaR, false);

    ASSERT_FALSE(result.empty());

    for (int y = 0; y < depth.rows; ++y) {
        EXPECT_LT(result.at<float>(y, 4), 2.0f); // Should stay close to 1.0
        EXPECT_GT(result.at<float>(y, 5), 2.0f); // Should stay close to 3.0
    }
}

// High sigmaR (should blur across the edge)
TEST(Util2dTest, fastBilateralFilteringBlursEdgesWithHighSigmaR) {
    cv::Mat depth = cv::Mat::ones(5, 10, CV_32FC1);
    depth.colRange(5, 10).setTo(3.0f);

    float sigmaS = 2.0f;
    float sigmaR = 5.0f; // High sigmaR allows more blurring

    cv::Mat result = util2d::fastBilateralFiltering(depth, sigmaS, sigmaR, false);

    ASSERT_FALSE(result.empty());

    // Middle column should be somewhere between 1.0 and 3.0
    for (int y = 0; y < depth.rows; ++y) {
        float val = result.at<float>(y, 5);
        EXPECT_GT(val, 1.1f);
        EXPECT_LT(val, 2.9f);
    }
}

// Random Depth Values with NaNs or Invalids
TEST(Util2dTest, fastBilateralFilteringHandlesInvalidDepthValues) {
    cv::Mat depth = cv::Mat::ones(5, 5, CV_32FC1);
    depth.at<float>(2, 2) = std::numeric_limits<float>::quiet_NaN();
    depth.at<float>(1, 3) = -1.0f;
    depth.at<float>(1, 2) = 1.1f;

    cv::Mat result = util2d::fastBilateralFiltering(depth, 2.0f, 0.1f, false);
   
    ASSERT_FALSE(result.empty());

    // Make sure the invalid pixels are skipped, and output is still valid
    for (int y = 0; y < result.rows; ++y) {
        for (int x = 0; x < result.cols; ++x) {
            float val = result.at<float>(y, x);
            EXPECT_TRUE((val!=0.0f && val-1.0f<0.02f) || val==0.0f);
        }
    }
}

// Early Division Toggle
TEST(Util2dTest, fastBilateralFilteringEarlyDivisionOptionConsistency) {
    cv::Mat depth = cv::Mat::ones(5, 5, CV_32FC1) * 2.0f;
    depth.at<float>(2,2) = 2.1f;
    cv::Mat result1 = util2d::fastBilateralFiltering(depth, 2.0f, 0.1f, true);
    cv::Mat result2 = util2d::fastBilateralFiltering(depth, 2.0f, 0.1f, false);

    ASSERT_FALSE(result1.empty());
    ASSERT_FALSE(result2.empty());

    for (int y = 0; y < depth.rows; ++y) {
        for (int x = 0; x < depth.cols; ++x) {
            EXPECT_NEAR(result1.at<float>(y, x), result2.at<float>(y, x), 1e-3);
        }
    }
}

// Test for empty input
TEST(Util2dTest, depthBleedingFilteringHandlesEmptyInput)
{
    cv::Mat empty;
    EXPECT_NO_THROW(util2d::depthBleedingFiltering(empty, 0.1f));
}

// Test that borders are zeroed out
TEST(Util2dTest, depthBleedingFilteringBordersAreZeroed)
{
    cv::Mat depth = cv::Mat::ones(5, 5, CV_32FC1);
    util2d::depthBleedingFiltering(depth, 0.1f);

    for(int i = 0; i < 5; ++i)
    {
        EXPECT_EQ(depth.at<float>(0, i), 0.0f);
        EXPECT_EQ(depth.at<float>(4, i), 0.0f);
        EXPECT_EQ(depth.at<float>(i, 0), 0.0f);
        EXPECT_EQ(depth.at<float>(i, 4), 0.0f);
    }
}

// Test that valid depths are not removed
TEST(Util2dTest, depthBleedingFilteringKeepsValidDepths)
{
    cv::Mat depth = cv::Mat::ones(5, 5, CV_32FC1);
    depth.at<float>(2,2) = 1.01f; // Within threshold of 0.1
    util2d::depthBleedingFiltering(depth, 0.1f);
    EXPECT_GT(depth.at<float>(2,2), 0.0f);
}

// Test that invalid depth is removed
TEST(Util2dTest, depthBleedingFilteringFiltersInvalidDepths)
{
    cv::Mat depth = cv::Mat::ones(5, 5, CV_32FC1);
    depth.at<float>(2,2) = 5.0f; // Large depth jump
    util2d::depthBleedingFiltering(depth, 0.1f);
    EXPECT_EQ(depth.at<float>(2,2), 0.0f);
}

// Repeat the above for CV_16UC1
TEST(Util2dTest, depthBleedingFilteringFiltersInvalidDepths16U)
{
    cv::Mat depth = cv::Mat::ones(5, 5, CV_16UC1) * 1000; // 1.0m in mm
    depth.at<uint16_t>(2,2) = 5000; // 5.0m
    util2d::depthBleedingFiltering(depth, 0.1f);
    EXPECT_EQ(depth.at<uint16_t>(2,2), 0);
}

TEST(Util2dTest, depthBleedingFilteringKeepsValidDepths16U)
{
    cv::Mat depth = cv::Mat::ones(5, 5, CV_16UC1) * 1000;
    depth.at<uint16_t>(2,2) = 1090; // 0.09m difference, within 0.1m
    util2d::depthBleedingFiltering(depth, 0.1f);
    EXPECT_GT(depth.at<uint16_t>(2,2), 0);
}

TEST(Util2dTest, HSVtoRGBPureRed) {
    float r, g, b;
    util2d::HSVtoRGB(&r, &g, &b, 0.0f, 1.0f, 1.0f);
    EXPECT_NEAR(r, 1.0f, 1e-4f);
    EXPECT_NEAR(g, 0.0f, 1e-4f);
    EXPECT_NEAR(b, 0.0f, 1e-4f);
}

TEST(Util2dTest, HSVtoRGBPureGreen) {
    float r, g, b;
    util2d::HSVtoRGB(&r, &g, &b, 120.0f, 1.0f, 1.0f);
    EXPECT_NEAR(r, 0.0f, 1e-4f);
    EXPECT_NEAR(g, 1.0f, 1e-4f);
    EXPECT_NEAR(b, 0.0f, 1e-4f);
}

TEST(Util2dTest, HSVtoRGBPureBlue) {
    float r, g, b;
    util2d::HSVtoRGB(&r, &g, &b, 240.0f, 1.0f, 1.0f);
    EXPECT_NEAR(r, 0.0f, 1e-4f);
    EXPECT_NEAR(g, 0.0f, 1e-4f);
    EXPECT_NEAR(b, 1.0f, 1e-4f);
}

TEST(Util2dTest, HSVtoRGBGrayscaleFromZeroSaturation) {
    float r, g, b;
    util2d::HSVtoRGB(&r, &g, &b, 0.0f, 0.0f, 0.5f);
    EXPECT_NEAR(r, 0.5f, 1e-4f);
    EXPECT_NEAR(g, 0.5f, 1e-4f);
    EXPECT_NEAR(b, 0.5f, 1e-4f);
}

TEST(Util2dTest, HSVtoRGBHueWrapAround360) {
    float r, g, b;
    util2d::HSVtoRGB(&r, &g, &b, 360.0f, 1.0f, 1.0f); // Hue 360 = Hue 0
    EXPECT_NEAR(r, 1.0f, 1e-4f);
    EXPECT_NEAR(g, 0.0f, 1e-4f);
    EXPECT_NEAR(b, 0.0f, 1e-4f);
}

TEST(Util2dTest, HSVtoRGBHalfSaturationHalfBrightness) {
    float r, g, b;
    util2d::HSVtoRGB(&r, &g, &b, 60.0f, 0.5f, 0.5f); // Yellowish
    EXPECT_NEAR(r, 0.5f, 1e-4f);
    EXPECT_NEAR(g, 0.5f, 1e-4f);
    EXPECT_NEAR(b, 0.25f, 1e-4f);
}

TEST(Util2dTest, NMSKeepsStrongestKeypointOnly) {
    std::vector<cv::KeyPoint> inputKeypoints = {
        cv::KeyPoint(cv::Point2f(50, 50), 1.f, -1, 0.8f), // weaker
        cv::KeyPoint(cv::Point2f(52, 52), 1.f, -1, 0.9f), // stronger, within dist_thresh
        cv::KeyPoint(cv::Point2f(100, 100), 1.f, -1, 0.7f) // far enough
    };

    cv::Mat descriptorsIn = cv::Mat::eye(3, 256, CV_32F); // mock descriptors
    std::vector<cv::KeyPoint> outputKeypoints;
    cv::Mat descriptorsOut;

    int dist_thresh = 5;
    int width = 200;
    int height = 200;

    util2d::NMS(inputKeypoints, descriptorsIn, outputKeypoints, descriptorsOut, dist_thresh, width, height);

    ASSERT_EQ(outputKeypoints.size(), 2);
    EXPECT_EQ(outputKeypoints[0].pt, cv::Point2f(52, 52)); // stronger one in the cluster
    EXPECT_EQ(outputKeypoints[1].pt, cv::Point2f(100, 100));

    ASSERT_EQ(descriptorsOut.rows, 2);
    EXPECT_TRUE(cv::countNonZero(descriptorsOut.row(0) != descriptorsIn.row(1)) == 0);
}

TEST(Util2dTest, NMSNoDescriptorsHandledGracefully) {
    std::vector<cv::KeyPoint> inputKeypoints = {
        cv::KeyPoint(cv::Point2f(10, 10), 1.f, -1, 1.0f)
    };
    std::vector<cv::KeyPoint> outputKeypoints;
    cv::Mat descriptorsOut;

    util2d::NMS(inputKeypoints, cv::Mat(), outputKeypoints, descriptorsOut, 3, 50, 50);

    ASSERT_EQ(outputKeypoints.size(), 1);
    EXPECT_TRUE(descriptorsOut.empty());
}

TEST(Util2dTest, NMSAllSuppressedDueToProximity) {
    std::vector<cv::KeyPoint> inputKeypoints = {
        cv::KeyPoint(cv::Point2f(30, 30), 1.f, -1, 0.5f),
        cv::KeyPoint(cv::Point2f(31, 31), 1.f, -1, 0.4f),
        cv::KeyPoint(cv::Point2f(32, 32), 1.f, -1, 0.3f)
    };

    cv::Mat descriptorsIn = cv::Mat::ones(3, 256, CV_32F);
    std::vector<cv::KeyPoint> outputKeypoints;
    cv::Mat descriptorsOut;

    util2d::NMS(inputKeypoints, descriptorsIn, outputKeypoints, descriptorsOut, 3, 100, 100);

    ASSERT_EQ(outputKeypoints.size(), 1); // only strongest survives
    EXPECT_NEAR(outputKeypoints[0].response, 0.5f, 1e-6);
}

TEST(Util2dTest, NMSImageBoundsRespected) {
    std::vector<cv::KeyPoint> inputKeypoints = {
        cv::KeyPoint(cv::Point2f(0, 0), 1.f, -1, 1.0f),
        cv::KeyPoint(cv::Point2f(199, 199), 1.f, -1, 1.0f),
        cv::KeyPoint(cv::Point2f(300, 300), 1.f, -1, 1.0f) // out of bounds
    };

    cv::Mat descriptorsIn = cv::Mat::eye(3, 256, CV_32F);
    std::vector<cv::KeyPoint> outputKeypoints;
    cv::Mat descriptorsOut;

    util2d::NMS(inputKeypoints, descriptorsIn, outputKeypoints, descriptorsOut, 2, 200, 200);

    ASSERT_EQ(outputKeypoints.size(), 2);
    for (const auto& kp : outputKeypoints) {
        EXPECT_LT(kp.pt.x, 200);
        EXPECT_LT(kp.pt.y, 200);
    }
}

std::vector<cv::KeyPoint> generateGridKeypoints(int cols, int rows, int spacing) {
    std::vector<cv::KeyPoint> keypoints;
    for (int y = 0; y < rows; y += spacing) {
        for (int x = 0; x < cols; x += spacing) {
            keypoints.emplace_back(cv::Point2f(x + 0.5f, y + 0.5f), 1.0f, -1, float(rand() % 100) / 100.0f);
        }
    }
    return keypoints;
}

TEST(Util2dTest, SSCSelectsRoughlyCorrectNumber) {
    int imgWidth = 300;
    int imgHeight = 300;
    auto keypoints = generateGridKeypoints(imgWidth, imgHeight, 5); // creates many keypoints

    int maxKeypoints = 100;
    float tolerance = 0.1f;

    std::vector<int> selected = util2d::SSC(keypoints, maxKeypoints, tolerance, imgWidth, imgHeight, {});

    EXPECT_GE(selected.size(), int(maxKeypoints * (1 - tolerance)));
    EXPECT_LE(selected.size(), int(maxKeypoints * (1 + tolerance)));
}

TEST(Util2dTest, SSCReturnsEmptyOnEmptyInput) {
    std::vector<cv::KeyPoint> keypoints;
    std::vector<int> selected = util2d::SSC(keypoints, 100, 0.1f, 100, 100, {});

    EXPECT_TRUE(selected.empty());
}

TEST(Util2dTest, SSCRespectsProvidedIndices) {
    int imgWidth = 200;
    int imgHeight = 200;
    auto keypoints = generateGridKeypoints(imgWidth, imgHeight, 10);

    // Sort keypoints manually (e.g., by response) and use the first N indices
    std::vector<int> topIndices(keypoints.size());
    for(size_t i=0; i<keypoints.size(); ++i)
    {
        topIndices[i] = i;
    }
    int maxKeypoints = 5;

    std::vector<int> selected = util2d::SSC(keypoints, maxKeypoints, 0.1f, imgWidth, imgHeight, topIndices);

    for (int idx : selected) {
        EXPECT_TRUE(std::find(topIndices.begin(), topIndices.end(), idx) != topIndices.end());
    }

    EXPECT_GE(selected.size(), 4);
    EXPECT_LE(selected.size(), 6);
}

TEST(Util2dTest, SSCSpatialDistributionCheck) {
    int imgWidth = 100;
    int imgHeight = 100;
    auto keypoints = generateGridKeypoints(imgWidth, imgHeight, 1); // dense points

    int maxKeypoints = 10;
    float tolerance = 0.2f;

    std::vector<int> selected = util2d::SSC(keypoints, maxKeypoints, tolerance, imgWidth, imgHeight, {});

    // Check that no two selected keypoints are too close
    float minDistance = float(imgWidth + imgHeight) / float(maxKeypoints); // loose estimate
    for (size_t i = 0; i < selected.size(); ++i) {
        for (size_t j = i + 1; j < selected.size(); ++j) {
            auto& pt1 = keypoints[selected[i]].pt;
            auto& pt2 = keypoints[selected[j]].pt;
            float dist = cv::norm(pt1 - pt2);
            EXPECT_GT(dist, minDistance * 0.2f); // 20% of estimated spacing
        }
    }
}

cv::Mat createTestImage(int width, int height, uchar value = 100) {
    return cv::Mat(height, width, CV_8UC3, cv::Scalar(value, value, value));
}

TEST(Util2dTest, rotateImagesUpsideUpIfNecessaryNoRotation) {
    CameraModel model(500, 500, 320, 240, CameraModel::opticalRotation(), 0, cv::Size(640, 480));
    cv::Mat rgb = createTestImage(640, 480);
    cv::Mat depth = createTestImage(640, 480);

    bool rotated = util2d::rotateImagesUpsideUpIfNecessary(model, rgb, depth);

    EXPECT_FALSE(rotated);
    EXPECT_EQ(rgb.cols, 640);
    EXPECT_EQ(rgb.rows, 480);
    EXPECT_EQ(depth.cols, 640);
    EXPECT_EQ(depth.rows, 480);
    float roll,pitch,yaw;
    (model.localTransform() * CameraModel::opticalRotation().inverse()).getEulerAngles(roll, pitch, yaw);
    EXPECT_EQ(roll, 0.0f);
    EXPECT_EQ(pitch, 0.0f);
    EXPECT_EQ(yaw, 0.0f);
}

TEST(Util2dTest, rotateImagesUpsideUpIfNecessaryRotation90Degrees) {
    // Simulate 90° roll
    Transform rot = Transform(0,0,0, M_PI / 2, 0, 0);
    CameraModel model(500, 500, 320, 240, rot*CameraModel::opticalRotation(), 0, cv::Size(640, 480));
    cv::Mat rgb = createTestImage(640, 480, 150);
    cv::Mat depth = createTestImage(640, 480, 200);

    bool rotated = util2d::rotateImagesUpsideUpIfNecessary(model, rgb, depth);

    EXPECT_TRUE(rotated);
    EXPECT_EQ(rgb.cols, 480);  // Transposed
    EXPECT_EQ(rgb.rows, 640);
    EXPECT_EQ(rgb.at<cv::Vec3b>(0, 0)[0], 150);  // Same pixel values
    EXPECT_EQ(depth.cols, 480);  // Transposed
    EXPECT_EQ(depth.rows, 640);
    EXPECT_EQ(depth.at<cv::Vec3b>(0, 0)[0], 200);
    float roll,pitch,yaw;
    (model.localTransform() * CameraModel::opticalRotation().inverse()).getEulerAngles(roll, pitch, yaw);
    EXPECT_NEAR(roll, M_PI, 1e-5);
    EXPECT_NEAR(pitch, 0.0f, 1e-5);
    EXPECT_NEAR(yaw, 0.0f, 1e-5);
}

TEST(Util2dTest, rotateImagesUpsideUpIfNecessaryRotation180Degrees) {
    Transform rot = Transform(0,0,0, M_PI, 0, 0);
    CameraModel model(500, 500, 320, 240, rot*CameraModel::opticalRotation(), 0, cv::Size(640, 480));
    cv::Mat rgb = createTestImage(640, 480, 123);
    cv::Mat depth = createTestImage(640, 480, 77);

    bool rotated = util2d::rotateImagesUpsideUpIfNecessary(model, rgb, depth);

    EXPECT_TRUE(rotated);
    EXPECT_EQ(rgb.cols, 640);  // Same size
    EXPECT_EQ(rgb.rows, 480);
    EXPECT_EQ(rgb.at<cv::Vec3b>(0, 0)[0], 123);
    EXPECT_EQ(depth.cols, 640);  // Same size
    EXPECT_EQ(depth.rows, 480);
    EXPECT_EQ(depth.at<cv::Vec3b>(0, 0)[0], 77);
    float roll,pitch,yaw;
    (model.localTransform() * CameraModel::opticalRotation().inverse()).getEulerAngles(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1e-5);
    EXPECT_NEAR(pitch, 0.0f, 1e-5);
    EXPECT_NEAR(yaw, 0.0f, 1e-5);
}

TEST(Util2dTest, rotateImagesUpsideUpIfNecessaryRotation270Degrees) {
    Transform rot = Transform(0,0,0, 3*M_PI/2, 0, 0);
    CameraModel model(500, 500, 320, 240, rot*CameraModel::opticalRotation(), 0, cv::Size(640, 480));
    cv::Mat rgb = createTestImage(640, 480, 90);
    cv::Mat depth = createTestImage(640, 480, 60);

    bool rotated = util2d::rotateImagesUpsideUpIfNecessary(model, rgb, depth);

    EXPECT_TRUE(rotated);
    EXPECT_EQ(rgb.cols, 480);
    EXPECT_EQ(rgb.rows, 640);
    EXPECT_EQ(rgb.at<cv::Vec3b>(0, 0)[0], 90);
    EXPECT_EQ(depth.cols, 480);
    EXPECT_EQ(depth.rows, 640);
    EXPECT_EQ(depth.at<cv::Vec3b>(0, 0)[0], 60);
    float roll,pitch,yaw;
    (model.localTransform() * CameraModel::opticalRotation().inverse()).getEulerAngles(roll, pitch, yaw);
    EXPECT_NEAR(roll, M_PI, 1e-5);
    EXPECT_NEAR(pitch, 0.0f, 1e-5);
    EXPECT_NEAR(yaw, 0.0f, 1e-5);
}

TEST(Util2dTest, rotateImagesUpsideUpIfNecessaryPitchTooHighShouldSkip) {
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kDebug);
    // Simulate roll = 90°, but pitch = 90° too (invalid)
    Transform rot = Transform(0,0,0, M_PI/2, M_PI/2, 0);
    CameraModel model(500, 500, 320, 240, rot*CameraModel::opticalRotation(), 0, cv::Size(640, 480));
    Transform orgTransform = model.localTransform();
    cv::Mat rgb = createTestImage(640, 480);
    cv::Mat depth = createTestImage(640, 480);

    bool rotated = util2d::rotateImagesUpsideUpIfNecessary(model, rgb, depth);

    EXPECT_FALSE(rotated);
    EXPECT_EQ(rgb.cols, 640);
    EXPECT_EQ(rgb.rows, 480);
    float roll,pitch,yaw;
    (model.localTransform().inverse() * orgTransform).getEulerAngles(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1e-5);
    EXPECT_NEAR(pitch, 0.0f, 1e-5);
    EXPECT_NEAR(yaw, 0.0f, 1e-5);
}