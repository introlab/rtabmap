#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
#include <vector>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UException.h>
#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#include <cmath>

using namespace rtabmap;

namespace {

static cv::Mat checkerboardImage(int rows = 200, int cols = 200, int cell = 20)
{
	cv::Mat image(rows, cols, CV_8UC1);
	for(int y = 0; y < rows; ++y)
	{
		for(int x = 0; x < cols; ++x)
		{
			image.at<uchar>(y, x) = uchar((((x / cell) + (y / cell)) % 2) ? 255 : 0);
		}
	}
	// Add blobs so binary descriptors (ORB) also find stable corners.
	for(int i = 0; i < 8; ++i)
	{
		cv::circle(image, cv::Point(25 + i * 22, 30 + (i % 3) * 40), 6, cv::Scalar(180), -1);
	}
	return image;
}

static ParametersMap orbTestParams()
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kKpMaxFeatures(), "200"));
	params.insert(ParametersPair(Parameters::kKpSubPixWinSize(), "0"));
	params.insert(ParametersPair(Parameters::kKpGridRows(), "1"));
	params.insert(ParametersPair(Parameters::kKpGridCols(), "1"));
	return params;
}

} // namespace

TEST(Feature2DTest, TypeNameKnownTypes)
{
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureUndef), "Unknown");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureSurf), "SURF");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureSift), "SIFT");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureOrb), "ORB");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureFastFreak), "FAST+FREAK");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureFastBrief), "FAST+BRIEF");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureGfttFreak), "GFTT+Freak");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureGfttBrief), "GFTT+Brief");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureBrisk), "BRISK");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureGfttOrb), "GFTT+ORB");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureKaze), "KAZE");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureOrbOctree), "ORB-OCTREE");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureSuperPointTorch), "SUPERPOINT");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureSurfFreak), "SURF+Freak");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureGfttDaisy), "GFTT+Daisy");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureSurfDaisy), "SURF+Daisy");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeaturePyDetector), "Unknown");
	EXPECT_EQ(Feature2D::typeName(Feature2D::kFeatureSuperPointRpautrat), "SUPERPOINT-RPAUTRAT");
	EXPECT_EQ(Feature2D::typeName((Feature2D::Type)99), "Unknown");
}

TEST(Feature2DTest, ComputeRoiFromRatios)
{
	const cv::Mat image = checkerboardImage(100, 100);
	std::vector<float> ratios;
	ratios.push_back(0.1f);
	ratios.push_back(0.1f);
	ratios.push_back(0.1f);
	ratios.push_back(0.1f);
	const cv::Rect roi = Feature2D::computeRoi(image, ratios);
	EXPECT_EQ(roi.x, 10);
	EXPECT_EQ(roi.y, 10);
	EXPECT_EQ(roi.width, 80);
	EXPECT_EQ(roi.height, 80);
}

TEST(Feature2DTest, ComputeRoiFromString)
{
	const cv::Mat image = checkerboardImage(100, 100);
	const cv::Rect roi = Feature2D::computeRoi(image, "0.1 0.1 0.1 0.1");
	EXPECT_EQ(roi.width, 80);
	EXPECT_EQ(roi.height, 80);
}

TEST(Feature2DTest, LimitKeypointsByResponse)
{
	std::vector<cv::KeyPoint> keypoints;
	for(int i = 0; i < 10; ++i)
	{
		cv::KeyPoint kpt;
		kpt.pt = cv::Point2f(float(i * 10), float(i * 10));
		kpt.response = float(i);
		keypoints.push_back(kpt);
	}
	Feature2D::limitKeypoints(keypoints, 5, cv::Size(200, 200), false);
	EXPECT_EQ(keypoints.size(), 5u);

	std::vector<float> keptResponses;
	keptResponses.reserve(keypoints.size());
	for(size_t i = 0; i < keypoints.size(); ++i)
	{
		keptResponses.push_back(keypoints[i].response);
	}
	std::sort(keptResponses.begin(), keptResponses.end());

	const std::vector<float> expectedTop5 = {5.f, 6.f, 7.f, 8.f, 9.f};
	EXPECT_EQ(keptResponses, expectedTop5);
}

TEST(Feature2DTest, LimitKeypointsWithDescriptors)
{
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors(8, 32, CV_8UC1, cv::Scalar(0));
	for(int i = 0; i < 8; ++i)
	{
		cv::KeyPoint kpt;
		kpt.pt = cv::Point2f(float(i), float(i));
		kpt.response = float(i);
		keypoints.push_back(kpt);
		descriptors.at<uchar>(i, 0) = uchar(i);
	}
	Feature2D::limitKeypoints(keypoints, descriptors, 3, cv::Size(100, 100), false);
	EXPECT_EQ(keypoints.size(), 3u);
	EXPECT_EQ(descriptors.rows, 3);

	for(int k = 0; k < descriptors.rows; ++k)
	{
		const int index = int(keypoints[k].pt.x);
		EXPECT_EQ(index, int(keypoints[k].pt.y));
		EXPECT_FLOAT_EQ(keypoints[k].response, float(index));
		EXPECT_EQ(descriptors.at<uchar>(k, 0), uchar(index));
	}

	std::vector<float> keptResponses;
	for(size_t i = 0; i < keypoints.size(); ++i)
	{
		keptResponses.push_back(keypoints[i].response);
	}
	std::sort(keptResponses.begin(), keptResponses.end());
	const std::vector<float> expectedTop3 = {5.f, 6.f, 7.f};
	EXPECT_EQ(keptResponses, expectedTop3);
}

TEST(Feature2DTest, FilterKeypointsByDepth)
{
	std::vector<cv::KeyPoint> keypoints;
	keypoints.push_back(cv::KeyPoint(50.f, 50.f, 1.f));
	keypoints.push_back(cv::KeyPoint(60.f, 60.f, 1.f));

	cv::Mat depth(100, 100, CV_32FC1, cv::Scalar(2.f));
	depth.at<float>(60, 60) = 0.05f;

	Feature2D::filterKeypointsByDepth(keypoints, depth, 0.1f, 5.f);
	EXPECT_EQ(keypoints.size(), 1u);
	EXPECT_FLOAT_EQ(keypoints[0].pt.x, 50.f);
}

TEST(Feature2DTest, FilterKeypointsByDisparity)
{
	std::vector<cv::KeyPoint> keypoints;
	keypoints.push_back(cv::KeyPoint(10.f, 10.f, 1.f));
	keypoints.push_back(cv::KeyPoint(20.f, 20.f, 1.f));

	cv::Mat disparity(50, 50, CV_32FC1, cv::Scalar(0.f));
	disparity.at<float>(10, 10) = 5.f;

	Feature2D::filterKeypointsByDisparity(keypoints, disparity, 1.f);
	EXPECT_EQ(keypoints.size(), 1u);
	EXPECT_FLOAT_EQ(keypoints[0].pt.x, 10.f);
}

TEST(Feature2DTest, CreateOrbDetector)
{
	ParametersMap params = orbTestParams();
	std::unique_ptr<Feature2D> detector(Feature2D::create(Feature2D::kFeatureOrb, params));
	ASSERT_TRUE(detector.get() != NULL);
	EXPECT_EQ(detector->getType(), Feature2D::kFeatureOrb);
	EXPECT_EQ(detector->getMaxFeatures(), 200);
}

// Smoke test: create() must not crash or return null for every strategy (fallbacks allowed).
TEST(Feature2DTest, CreateAllDetectorStrategiesSmoke)
{
	const Feature2D::Type strategies[] = {
		Feature2D::kFeatureUndef,
		Feature2D::kFeatureSurf,
		Feature2D::kFeatureSift,
		Feature2D::kFeatureOrb,
		Feature2D::kFeatureFastFreak,
		Feature2D::kFeatureFastBrief,
		Feature2D::kFeatureGfttFreak,
		Feature2D::kFeatureGfttBrief,
		Feature2D::kFeatureBrisk,
		Feature2D::kFeatureGfttOrb,
		Feature2D::kFeatureKaze,
		Feature2D::kFeatureOrbOctree,
		Feature2D::kFeatureSuperPointTorch,
		Feature2D::kFeatureSurfFreak,
		Feature2D::kFeatureGfttDaisy,
		Feature2D::kFeatureSurfDaisy,
		Feature2D::kFeaturePyDetector,
		Feature2D::kFeatureSuperPointRpautrat,
	};
	const ParametersMap params = orbTestParams();

	for(size_t i = 0; i < sizeof(strategies) / sizeof(strategies[0]); ++i)
	{
		const Feature2D::Type requested = strategies[i];
		std::unique_ptr<Feature2D> detector(Feature2D::create(requested, params));
		ASSERT_TRUE(detector.get() != NULL)
				<< "create() returned null for " << Feature2D::typeName(requested);
	}

	for(int strategy = Feature2D::kFeatureSurf; strategy <= Feature2D::kFeatureSuperPointRpautrat; ++strategy)
	{
		ParametersMap paramsWithStrategy = params;
		paramsWithStrategy[Parameters::kKpDetectorStrategy()] = uNumber2Str(strategy);
		std::unique_ptr<Feature2D> detector(Feature2D::create(paramsWithStrategy));
		ASSERT_TRUE(detector.get() != NULL)
				<< "create(ParametersMap) returned null for Kp/DetectorStrategy=" << strategy;
	}
}

TEST(Feature2DTest, CreateFromParametersMap)
{
	ParametersMap params = orbTestParams();
	params.insert(ParametersPair(Parameters::kKpDetectorStrategy(), "2")); // ORB
	std::unique_ptr<Feature2D> detector(Feature2D::create(params));
	ASSERT_TRUE(detector.get() != NULL);
	EXPECT_EQ(detector->getType(), Feature2D::kFeatureOrb);
}

TEST(Feature2DTest, ParseParametersUpdatesMaxFeatures)
{
	ParametersMap params = orbTestParams();
	std::unique_ptr<Feature2D> detector(Feature2D::create(Feature2D::kFeatureOrb, params));
	ASSERT_TRUE(detector.get() != NULL);

	ParametersMap update;
	update.insert(ParametersPair(Parameters::kKpMaxFeatures(), "50"));
	detector->parseParameters(update);
	EXPECT_EQ(detector->getMaxFeatures(), 50);
}

TEST(Feature2DTest, GenerateKeypointsAndDescriptors)
{
	const cv::Mat image = checkerboardImage();
	std::unique_ptr<Feature2D> detector(Feature2D::create(Feature2D::kFeatureOrb, orbTestParams()));
	ASSERT_TRUE(detector.get() != NULL);

	std::vector<cv::KeyPoint> keypoints = detector->generateKeypoints(image);
	EXPECT_GT(keypoints.size(), 0u);

	cv::Mat descriptors = detector->generateDescriptors(image, keypoints);
	ASSERT_FALSE(descriptors.empty());
	EXPECT_EQ(descriptors.rows, static_cast<int>(keypoints.size()));
	EXPECT_EQ(descriptors.type(), CV_8UC1);
}

TEST(Feature2DTest, GenerateKeypointsAndDescriptorsWithMask)
{
	const cv::Mat image = checkerboardImage();
	// Valid region: left half only (OpenCV mask: non-zero = detect).
	cv::Mat mask(image.rows, image.cols, CV_8UC1, cv::Scalar(0));
	mask(cv::Rect(0, 0, image.cols / 2, image.rows)).setTo(255);

	std::unique_ptr<Feature2D> detector(Feature2D::create(Feature2D::kFeatureOrb, orbTestParams()));
	ASSERT_TRUE(detector.get() != NULL);

	std::vector<cv::KeyPoint> keypoints = detector->generateKeypoints(image, mask);
	EXPECT_GT(keypoints.size(), 0u);

	for(size_t i = 0; i < keypoints.size(); ++i)
	{
		const int x = cvRound(keypoints[i].pt.x);
		const int y = cvRound(keypoints[i].pt.y);
		ASSERT_GE(x, 0);
		ASSERT_GE(y, 0);
		ASSERT_LT(x, mask.cols);
		ASSERT_LT(y, mask.rows);
		EXPECT_GT(mask.at<uchar>(y, x), 0)
				<< "keypoint " << i << " at (" << x << "," << y << ") outside mask";
	}

	cv::Mat descriptors = detector->generateDescriptors(image, keypoints);
	ASSERT_FALSE(descriptors.empty());
	EXPECT_EQ(descriptors.rows, static_cast<int>(keypoints.size()));
	EXPECT_EQ(descriptors.type(), CV_8UC1);
}

TEST(Feature2DTest, GenerateKeypointsAndDescriptorsWithRoi)
{
	const cv::Mat image = checkerboardImage();
	// Left half only (Kp/RoiRatios: left right top bottom).
	const cv::Rect roi = Feature2D::computeRoi(image, "0 0.5 0 0");
	ASSERT_GT(roi.width, 0);
	ASSERT_GT(roi.height, 0);

	ParametersMap params = orbTestParams();
	params.insert(ParametersPair(Parameters::kKpRoiRatios(), "0 0.5 0 0"));

	std::unique_ptr<Feature2D> detector(Feature2D::create(Feature2D::kFeatureOrb, params));
	ASSERT_TRUE(detector.get() != NULL);

	std::vector<cv::KeyPoint> keypoints = detector->generateKeypoints(image);
	EXPECT_GT(keypoints.size(), 0u);

	for(size_t i = 0; i < keypoints.size(); ++i)
	{
		const int x = cvRound(keypoints[i].pt.x);
		const int y = cvRound(keypoints[i].pt.y);
		EXPECT_GE(x, roi.x) << "keypoint " << i << " at (" << x << "," << y << ")";
		EXPECT_GE(y, roi.y);
		EXPECT_LT(x, roi.x + roi.width);
		EXPECT_LT(y, roi.y + roi.height);
	}

	cv::Mat descriptors = detector->generateDescriptors(image, keypoints);
	ASSERT_FALSE(descriptors.empty());
	EXPECT_EQ(descriptors.rows, static_cast<int>(keypoints.size()));
	EXPECT_EQ(descriptors.type(), CV_8UC1);
}

TEST(Feature2DTest, GenerateDescriptorsEmptyForNoKeypoints)
{
	const cv::Mat image = checkerboardImage();
	std::unique_ptr<Feature2D> detector(Feature2D::create(Feature2D::kFeatureOrb, orbTestParams()));
	std::vector<cv::KeyPoint> keypoints;
	const cv::Mat descriptors = detector->generateDescriptors(image, keypoints);
	EXPECT_TRUE(descriptors.empty());
}
