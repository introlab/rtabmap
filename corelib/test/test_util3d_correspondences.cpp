#include "gtest/gtest.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_correspondences.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/utilite/UException.h"
#include "rtabmap/utilite/UConversion.h"
#include <pcl/io/pcd_io.h>

using namespace rtabmap;

TEST(Util3dCorrespondences, extractXYZCorrespondencesValidOneToOneMatch) {
    std::multimap<int, pcl::PointXYZ> words1 = {
        {1, pcl::PointXYZ(1, 2, 3)},
        {2, pcl::PointXYZ(4, 5, 6)}
    };
    std::multimap<int, pcl::PointXYZ> words2 = {
        {1, pcl::PointXYZ(1.1f, 2.1f, 3.1f)},
        {2, pcl::PointXYZ(4.1f, 5.1f, 6.1f)}
    };

    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    util3d::extractXYZCorrespondences(words1, words2, cloud1, cloud2);

    ASSERT_EQ(cloud1.size(), 2);
    ASSERT_EQ(cloud2.size(), 2);
    EXPECT_EQ(cloud1.points[0].x, 1);
    EXPECT_EQ(cloud2.points[1].z, 6.1f);
}

TEST(Util3dCorrespondences, extractXYZCorrespondencesDuplicateKeysIgnored) {
    std::multimap<int, pcl::PointXYZ> words1 = {
        {1, pcl::PointXYZ(0, 0, 0)},
        {1, pcl::PointXYZ(1, 1, 1)}, // duplicate
        {2, pcl::PointXYZ(2, 2, 2)}
    };
    std::multimap<int, pcl::PointXYZ> words2 = {
        {1, pcl::PointXYZ(1, 1, 1)},
        {2, pcl::PointXYZ(2.1f, 2.1f, 2.1f)}
    };

    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    util3d::extractXYZCorrespondences(words1, words2, cloud1, cloud2);

    ASSERT_EQ(cloud1.size(), 1);
    ASSERT_EQ(cloud2.size(), 1);
    EXPECT_EQ(cloud1[0].x, 2);
    EXPECT_EQ(cloud2[0].z, 2.1f);
}

TEST(Util3dCorrespondences, extractXYZCorrespondencesInvalidPointsIgnored) {
    pcl::PointXYZ nanPt(std::numeric_limits<float>::quiet_NaN(), 0, 0);
    std::multimap<int, pcl::PointXYZ> words1 = {
        {1, pcl::PointXYZ(1, 2, 3)},
        {2, nanPt}
    };
    std::multimap<int, pcl::PointXYZ> words2 = {
        {1, pcl::PointXYZ(1.5f, 2.5f, 3.5f)},
        {2, pcl::PointXYZ(4, 5, 6)}
    };

    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    util3d::extractXYZCorrespondences(words1, words2, cloud1, cloud2);

    ASSERT_EQ(cloud1.size(), 1);
    ASSERT_EQ(cloud2.size(), 1);
    EXPECT_EQ(cloud1[0].x, 1);
    EXPECT_EQ(cloud2[0].y, 2.5f);
}

TEST(Util3dCorrespondences, extractXYZCorrespondencesNoCommonIDs) {
    std::multimap<int, pcl::PointXYZ> words1 = {
        {10, pcl::PointXYZ(1, 2, 3)}
    };
    std::multimap<int, pcl::PointXYZ> words2 = {
        {20, pcl::PointXYZ(4, 5, 6)}
    };

    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    util3d::extractXYZCorrespondences(words1, words2, cloud1, cloud2);

    EXPECT_TRUE(cloud1.empty());
    EXPECT_TRUE(cloud2.empty());
}

TEST(Util3dCorrespondences, extractXYZCorrespondencesRANSACAcceptsCleanMatches) {
	std::multimap<int, pcl::PointXYZ> words1;
	std::multimap<int, pcl::PointXYZ> words2;

	// 10 consistent matches
	for (int i = 0; i < 10; ++i) {
		words1.insert({i, pcl::PointXYZ(i * 1.0f, exp2(i)/10.0f, 0.0f)});
		words2.insert({i, pcl::PointXYZ(i * 1.0f + 1.1f, exp2(i)/10.0f + 1.1f, 0.0f)});  // Slight noise
	}

	pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
	util3d::extractXYZCorrespondencesRANSAC(words1, words2, cloud1, cloud2);

	EXPECT_EQ(cloud1.size(), cloud2.size());
	EXPECT_GE(cloud1.size(), 8); // At least 8 inliers from 10 consistent matches
}

TEST(Util3dCorrespondences, extractXYZCorrespondencesRANSACRejectsOutliers) {
	std::multimap<int, pcl::PointXYZ> words1;
	std::multimap<int, pcl::PointXYZ> words2;

	// 8 inliers
	for (int i = 0; i < 8; ++i) {
		words1.insert({i, pcl::PointXYZ(i * 1.0f, exp2(i)/10.0f, 0.0f)});
		words2.insert({i, pcl::PointXYZ(i * 1.0f + 1.1f, exp2(i)/10.0f + 1.1f, 0.0f)});  // Slight noise
	}

	// 2 outliers
	words1.insert({100, pcl::PointXYZ(0.0f, 0.0f, 0.0f)});
	words2.insert({100, pcl::PointXYZ(100.0f, 100.0f, 0.0f)});
	words1.insert({101, pcl::PointXYZ(1.0f, 1.0f, 0.0f)});
	words2.insert({101, pcl::PointXYZ(200.0f, -50.0f, 0.0f)});

	pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
	util3d::extractXYZCorrespondencesRANSAC(words1, words2, cloud1, cloud2);

	EXPECT_EQ(cloud1.size(), cloud2.size());
	EXPECT_EQ(cloud1.size(), 8);  // RANSAC should reject 2 outliers
}

TEST(Util3dCorrespondences, extractXYZCorrespondencesRANSACFailsGracefullyOnTooFewMatches) {
	std::multimap<int, pcl::PointXYZ> words1 = {
		{1, pcl::PointXYZ(0, 0, 0)},
		{2, pcl::PointXYZ(1, 1, 1)},
		{3, pcl::PointXYZ(2, 2, 2)}
	};

	std::multimap<int, pcl::PointXYZ> words2 = {
		{1, pcl::PointXYZ(0.1f, 0.1f, 0)},
		{2, pcl::PointXYZ(1.1f, 1.1f, 1)},
		{3, pcl::PointXYZ(2.1f, 2.1f, 2)}
	};

	pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
	util3d::extractXYZCorrespondencesRANSAC(words1, words2, cloud1, cloud2);

	EXPECT_TRUE(cloud1.empty());
	EXPECT_TRUE(cloud2.empty());
}

TEST(Util3dCorrespondences, extractXYZCorrespondencesValidCorrespondencesAreExtracted) {
    // Create simple 5x5 depth images with valid depth
    cv::Mat depth1 = cv::Mat::ones(5, 5, CV_32FC1) * 1.0f;
    cv::Mat depth2 = cv::Mat::ones(5, 5, CV_32FC1) * 1.5f;

    std::list<std::pair<cv::Point2f, cv::Point2f>> matches = {
        {cv::Point2f(2, 2), cv::Point2f(2, 2)},
        {cv::Point2f(1, 1), cv::Point2f(1, 1)},
        {cv::Point2f(3, 3), cv::Point2f(3, 3)}
    };

    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;

    float fx = 1.0f, fy = 1.0f, cx = 2.0f, cy = 2.0f;
    util3d::extractXYZCorrespondences(matches, depth1, depth2, cx, cy, fx, fy, 2.0f, cloud1, cloud2);

    ASSERT_EQ(cloud1.size(), 3);
    ASSERT_EQ(cloud2.size(), 3);

    // Check one known point
    EXPECT_FLOAT_EQ(cloud1[0].z, 1.0f);
    EXPECT_FLOAT_EQ(cloud2[0].z, 1.5f);
}

TEST(Util3dCorrespondences, extractXYZCorrespondencesFiltersInvalidDepth) {
    cv::Mat depth1 = cv::Mat::ones(5, 5, CV_32FC1) * 1.0f;
    cv::Mat depth2 = cv::Mat::ones(5, 5, CV_32FC1) * 1.5f;
    depth1.at<float>(2, 2) = 0.0f;  // Invalid
    depth2.at<float>(1, 1) = std::numeric_limits<float>::quiet_NaN();  // Invalid

    std::list<std::pair<cv::Point2f, cv::Point2f>> matches = {
        {cv::Point2f(2, 2), cv::Point2f(2, 2)},
        {cv::Point2f(1, 1), cv::Point2f(1, 1)},
        {cv::Point2f(3, 3), cv::Point2f(3, 3)}
    };

    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;

    util3d::extractXYZCorrespondences(matches, depth1, depth2, 2.0f, 2.0f, 1.0f, 1.0f, 2.0f, cloud1, cloud2);

    // Only the third match should remain
    ASSERT_EQ(cloud1.size(), 1);
    ASSERT_EQ(cloud2.size(), 1);
    EXPECT_FLOAT_EQ(cloud1[0].z, 1.0f);
    EXPECT_FLOAT_EQ(cloud2[0].z, 1.5f);
}

TEST(Util3dCorrespondences, extractXYZCorrespondencesRespectsMaxDepthConstraint) {
    cv::Mat depth1 = cv::Mat::ones(5, 5, CV_32FC1) * 3.0f;  // Exceeds maxDepth
    cv::Mat depth2 = cv::Mat::ones(5, 5, CV_32FC1) * 1.0f;

    std::list<std::pair<cv::Point2f, cv::Point2f>> matches = {
        {cv::Point2f(2, 2), cv::Point2f(2, 2)},
        {cv::Point2f(1, 1), cv::Point2f(1, 1)}
    };

    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;

    util3d::extractXYZCorrespondences(matches, depth1, depth2, 2.0f, 2.0f, 1.0f, 1.0f, 2.5f, cloud1, cloud2);

    // All points should be rejected due to depth1 being too large
    EXPECT_TRUE(cloud1.empty());
    EXPECT_TRUE(cloud2.empty());
}

TEST(Util3dCorrespondences, extractXYZCorrespondencesOrgCloudsValidCorrespondencesAreExtracted) {
    int width = 5, height = 5;

    // Create two organized point clouds
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    cloud1.width = cloud2.width = width;
    cloud1.height = cloud2.height = height;
    cloud1.is_dense = cloud2.is_dense = false;
    cloud1.points.resize(width * height);
    cloud2.points.resize(width * height);

    // Fill the clouds with some values
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            int idx = v * width + u;
            cloud1.at(idx).x = u;
            cloud1.at(idx).y = v;
            cloud1.at(idx).z = 1.0f;
            cloud2.at(idx).x = u + 0.5f;
            cloud2.at(idx).y = v + 0.5f;
            cloud2.at(idx).z = 1.5f;
        }
    }

    // Set correspondences to valid pixel positions
    std::list<std::pair<cv::Point2f, cv::Point2f>> correspondences = {
        {cv::Point2f(1, 1), cv::Point2f(1, 1)},
        {cv::Point2f(2, 2), cv::Point2f(2, 2)},
        {cv::Point2f(3, 3), cv::Point2f(3, 3)}
    };

    pcl::PointCloud<pcl::PointXYZ> inliers1, inliers2;

    util3d::extractXYZCorrespondences(correspondences, cloud1, cloud2, inliers1, inliers2);

    ASSERT_EQ(inliers1.size(), 3);
    ASSERT_EQ(inliers2.size(), 3);

    EXPECT_FLOAT_EQ(inliers1[0].x, 1);
    EXPECT_FLOAT_EQ(inliers1[0].z, 1.0f);
    EXPECT_FLOAT_EQ(inliers2[0].x, 1.5f);
    EXPECT_FLOAT_EQ(inliers2[0].z, 1.5f);
}

TEST(Util3dCorrespondences, extractXYZCorrespondencesOrgCloudsInvalidPointsAreFilteredOut) {
    int width = 3, height = 3;

    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    cloud1.width = cloud2.width = width;
    cloud1.height = cloud2.height = height;
    cloud1.is_dense = cloud2.is_dense = false;
    cloud1.points.resize(width * height);
    cloud2.points.resize(width * height);

    // Set all points to NaN
    for (size_t i = 0; i < cloud1.size(); ++i) {
        cloud1[i].x = cloud1[i].y = cloud1[i].z = std::numeric_limits<float>::quiet_NaN();
        cloud2[i].x = cloud2[i].y = cloud2[i].z = std::numeric_limits<float>::quiet_NaN();
    }

    // Set one valid point at (1,1)
    int idx = 1 * width + 1;
    cloud1[idx].x = 1.0f;
    cloud1[idx].y = 1.0f;
    cloud1[idx].z = 1.0f;
    cloud2[idx].x = 2.0f;
    cloud2[idx].y = 2.0f;
    cloud2[idx].z = 2.0f;

    std::list<std::pair<cv::Point2f, cv::Point2f>> correspondences = {
        {cv::Point2f(0, 0), cv::Point2f(0, 0)}, // Invalid
        {cv::Point2f(1, 1), cv::Point2f(1, 1)}, // Valid
        {cv::Point2f(2, 2), cv::Point2f(2, 2)}  // Invalid
    };

    pcl::PointCloud<pcl::PointXYZ> inliers1, inliers2;

    util3d::extractXYZCorrespondences(correspondences, cloud1, cloud2, inliers1, inliers2);

    ASSERT_EQ(inliers1.size(), 1);
    ASSERT_EQ(inliers2.size(), 1);
    EXPECT_FLOAT_EQ(inliers1[0].x, 1.0f);
    EXPECT_FLOAT_EQ(inliers2[0].x, 2.0f);
}

TEST(Util3dCorrespondences, countUniquePairsNoPairs) {
    std::multimap<int, pcl::PointXYZ> wordsA, wordsB;

    wordsA.insert({1, pcl::PointXYZ(1, 2, 3)});
    wordsB.insert({2, pcl::PointXYZ(1, 2, 3)}); // No overlapping key

    EXPECT_EQ(util3d::countUniquePairs(wordsA, wordsB), 0);
}

TEST(Util3dCorrespondences, countUniquePairsOneUniquePair) {
    std::multimap<int, pcl::PointXYZ> wordsA, wordsB;

    wordsA.insert({1, pcl::PointXYZ(1, 1, 1)});
    wordsB.insert({1, pcl::PointXYZ(2, 2, 2)}); // One unique pair

    EXPECT_EQ(util3d::countUniquePairs(wordsA, wordsB), 1);
}

TEST(Util3dCorrespondences, countUniquePairsMultipleUniquePairs) {
    std::multimap<int, pcl::PointXYZ> wordsA, wordsB;

    wordsA.insert({1, pcl::PointXYZ(1, 1, 1)});
    wordsA.insert({2, pcl::PointXYZ(2, 2, 2)});
    wordsA.insert({3, pcl::PointXYZ(3, 3, 3)});
    
    wordsB.insert({1, pcl::PointXYZ(1, 1, 1)});
    wordsB.insert({2, pcl::PointXYZ(2, 2, 2)});
    wordsB.insert({3, pcl::PointXYZ(3, 3, 3)});

    EXPECT_EQ(util3d::countUniquePairs(wordsA, wordsB), 3);
}

TEST(Util3dCorrespondences, countUniquePairsDuplicatedPointsNotCounted) {
    std::multimap<int, pcl::PointXYZ> wordsA, wordsB;

    wordsA.insert({1, pcl::PointXYZ(1, 1, 1)});
    wordsA.insert({1, pcl::PointXYZ(1.1f, 1.1f, 1.1f)}); // duplicate in A
    wordsB.insert({1, pcl::PointXYZ(2, 2, 2)});

    EXPECT_EQ(util3d::countUniquePairs(wordsA, wordsB), 0);

    wordsA.clear();
    wordsB.clear();

    wordsA.insert({2, pcl::PointXYZ(1, 1, 1)});
    wordsB.insert({2, pcl::PointXYZ(2, 2, 2)});
    wordsB.insert({2, pcl::PointXYZ(3, 3, 3)}); // duplicate in B

    EXPECT_EQ(util3d::countUniquePairs(wordsA, wordsB), 0);
}

TEST(Util3dCorrespondences, filterMaxDepthFiltersByMaxDepthZ) {
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    pcl::PointCloud<pcl::PointXYZ> cloud2;

    // Add points (some above and some below maxDepth = 5.0)
    cloud1.push_back(pcl::PointXYZ(1.0f, 1.0f, 4.0f));
    cloud2.push_back(pcl::PointXYZ(1.1f, 1.0f, 4.0f));
    cloud1.push_back(pcl::PointXYZ(2.0f, 2.0f, 6.0f));  // exceeds maxDepth
    cloud2.push_back(pcl::PointXYZ(2.1f, 2.0f, 6.0f));
    cloud1.push_back(pcl::PointXYZ(3.0f, 3.0f, 3.0f));
    cloud2.push_back(pcl::PointXYZ(3.1f, 3.0f, 3.0f));

    // Filter by maxDepth=5.0 on 'z' axis, no duplicates removal
    util3d::filterMaxDepth(cloud1, cloud2, 5.0f, 'z', false);

    EXPECT_EQ(cloud1.size(), 2);
    EXPECT_EQ(cloud2.size(), 2);

    for (const auto& pt : cloud1) {
        EXPECT_LT(pt.z, 5.0f);
    }
}

TEST(Util3dCorrespondences, filterMaxDepthRemovesDuplicates) {
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    pcl::PointCloud<pcl::PointXYZ> cloud2;

    // Duplicate points in cloud1, but different points in cloud2
    cloud1.push_back(pcl::PointXYZ(1.0f, 1.0f, 1.0f));
    cloud2.push_back(pcl::PointXYZ(1.1f, 1.0f, 1.0f));
    cloud1.push_back(pcl::PointXYZ(1.0f, 1.0f, 1.0f));  // duplicate
    cloud2.push_back(pcl::PointXYZ(1.2f, 1.0f, 1.0f));
    cloud1.push_back(pcl::PointXYZ(2.0f, 2.0f, 2.0f));
    cloud2.push_back(pcl::PointXYZ(2.1f, 2.0f, 2.0f));

    // maxDepth large enough to keep all points, removeDuplicates = true
    util3d::filterMaxDepth(cloud1, cloud2, 10.0f, 'z', true);

    EXPECT_EQ(cloud1.size(), 2);
    EXPECT_EQ(cloud2.size(), 2);

    // Check that duplicate is removed (only one point with 1.0,1.0,1.0)
    int countPoint = 0;
    for (const auto& pt : cloud1) {
        if (pt.x == 1.0f && pt.y == 1.0f && pt.z == 1.0f) {
            countPoint++;
        }
    }
    EXPECT_EQ(countPoint, 1);
}

TEST(Util3dCorrespondences, findCorrespondencesBasicMatching)
{
    std::multimap<int, cv::KeyPoint> wordsA, wordsB;
    std::list<std::pair<cv::Point2f, cv::Point2f>> pairs;

    // Setup wordsA: IDs 1, 2, 3 (only 2 is unique)
    wordsA.insert({1, cv::KeyPoint(10.0f, 10.0f, 1)});
    wordsA.insert({2, cv::KeyPoint(20.0f, 20.0f, 1)});
    wordsA.insert({3, cv::KeyPoint(30.0f, 30.0f, 1)});
    wordsA.insert({3, cv::KeyPoint(31.0f, 31.0f, 1)});

    // Setup wordsB: IDs 2, 3 (only 2 is unique in both)
    wordsB.insert({2, cv::KeyPoint(20.5f, 20.5f, 1)});
    wordsB.insert({3, cv::KeyPoint(30.5f, 30.5f, 1)});
    wordsB.insert({3, cv::KeyPoint(32.0f, 32.0f, 1)});

    util3d::findCorrespondences(wordsA, wordsB, pairs);

    ASSERT_EQ(pairs.size(), 1);
    EXPECT_EQ(pairs.front().first, cv::Point2f(20.0f, 20.0f));
    EXPECT_EQ(pairs.front().second, cv::Point2f(20.5f, 20.5f));
}

TEST(Util3dCorrespondences, findCorrespondencesMatchesWithDepthCheck)
{
    std::multimap<int, cv::Point3f> words1, words2;
    std::vector<cv::Point3f> inliers1, inliers2;
    std::vector<int> correspondences;

    // Insert matching and non-matching entries
    words1.insert({1, cv::Point3f(1, 1, 1)});
    words1.insert({2, cv::Point3f(2, 2, 2)});
    words1.insert({3, cv::Point3f(100, 100, 100)}); // out of depth

    words2.insert({1, cv::Point3f(1.1f, 1.1f, 1.1f)});
    words2.insert({2, cv::Point3f(2.1f, 2.1f, 2.1f)});
    words2.insert({3, cv::Point3f(101, 101, 101)}); // out of depth

    float maxDepth = 10.0f;

    util3d::findCorrespondences(words1, words2, inliers1, inliers2, maxDepth, &correspondences);

    ASSERT_EQ(inliers1.size(), 2);
    ASSERT_EQ(inliers2.size(), 2);
    ASSERT_EQ(correspondences.size(), 2);

    EXPECT_EQ(correspondences[0], 1);
    EXPECT_EQ(correspondences[1], 2);
}

TEST(Util3dCorrespondences, findCorrespondencesMatchesWithMaxDepth)
{
    std::map<int, cv::Point3f> words1, words2;
    std::vector<cv::Point3f> inliers1, inliers2;
    std::vector<int> correspondences;

    words1[1] = cv::Point3f(1, 1, 1);
    words1[2] = cv::Point3f(2, 2, 2);
    words1[3] = cv::Point3f(100, 100, 100); // Exceeds maxDepth

    words2[1] = cv::Point3f(1.2f, 1.2f, 1.2f);
    words2[2] = cv::Point3f(2.2f, 2.2f, 2.2f);
    words2[3] = cv::Point3f(101, 101, 101); // Exceeds maxDepth

    util3d::findCorrespondences(words1, words2, inliers1, inliers2, 10.0f, &correspondences);

    ASSERT_EQ(inliers1.size(), 2);
    ASSERT_EQ(inliers2.size(), 2);
    ASSERT_EQ(correspondences.size(), 2);

    EXPECT_EQ(correspondences[0], 1);
    EXPECT_EQ(correspondences[1], 2);
}