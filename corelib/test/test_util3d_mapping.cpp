#include "gtest/gtest.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_mapping.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/utilite/UException.h"
#include "rtabmap/utilite/UConversion.h"
#include <pcl/io/pcd_io.h>

using namespace rtabmap;

TEST(Util3dMapping, rayTraceClearsFreePathWithoutObstacle) {
    cv::Mat grid = cv::Mat::ones(10, 10, CV_8SC1) * 50; // Initial grid (non-zero for testing)
    cv::Point2i start(2, 2);
    cv::Point2i end(7, 7);

    util3d::rayTrace(start, end, grid, false);

    // Check that the path has been cleared (set to 0)
    for (int i = 2; i < 7; ++i) {
        ASSERT_EQ(grid.at<signed char>(i, i), 0);
    }
}

TEST(Util3dMapping, rayTraceStopsOnObstacleWhenFlagTrue) {
    cv::Mat grid = cv::Mat::ones(10, 10, CV_8SC1) * 50;
    grid.at<signed char>(5, 5) = 100; // Add obstacle
    cv::Point2i start(2, 2);
    cv::Point2i end(7, 7);

    util3d::rayTrace(start, end, grid, true);

    // Ensure points before the obstacle are cleared, and after are not
    for (int i = 2; i < 5; ++i) {
        EXPECT_EQ(grid.at<signed char>(i, i), 0);
    }
    EXPECT_EQ(grid.at<signed char>(5, 5), 100); // Obstacle should remain
    EXPECT_NE(grid.at<signed char>(6, 6), 0);   // Not cleared after obstacle
}

TEST(Util3dMapping, rayTraceIgnoresObstacleWhenFlagFalse) {
    cv::Mat grid = cv::Mat::ones(10, 10, CV_8SC1) * 50;
    grid.at<signed char>(5, 5) = 100; // Add obstacle
    cv::Point2i start(2, 2);
    cv::Point2i end(7, 7);

    util3d::rayTrace(start, end, grid, false);

    // All cells in the line should be cleared regardless of obstacle
    for (int i = 2; i < 7; ++i) {
        EXPECT_EQ(grid.at<signed char>(i, i), 0);
    }
}

TEST(Util3dMapping, rayTraceHandlesSteepSlopeCorrectly) {
    // Create a 10x10 grid filled with 50
    cv::Mat grid = cv::Mat::ones(10, 10, CV_8SC1) * 50;

    // Set a steep slope: vertical-like line from bottom to top
    cv::Point2i start(5, 1);  // near the bottom
    cv::Point2i end(6, 8);    // almost vertical but not perfectly, so slope > 1

    util3d::rayTrace(start, end, grid, false);

    // Check that some pixels along that steep line are cleared
    // We expect the values along the approximate path to be set to 0
    int clearedCount = 0;
    for (int y = 1; y <= 8; ++y) {
        for (int x = 5; x <= 6; ++x) {
            if (grid.at<signed char>(y, x) == 0) {
                clearedCount++;
            }
        }
    }

    // At least 5 pixels should have been cleared along the steep slope
    EXPECT_GE(clearedCount, 5) << "Steep slope did not clear expected cells.";

    grid = cv::Mat::ones(10, 10, CV_8SC1) * 50;

    // Same thing, but inverted
    util3d::rayTrace(end, start, grid, false);

    // Check that some pixels along that steep line are cleared
    // We expect the values along the approximate path to be set to 0
    clearedCount = 0;
    for (int y = 1; y <= 8; ++y) {
        for (int x = 5; x <= 6; ++x) {
            if (grid.at<signed char>(y, x) == 0) {
                clearedCount++;
            }
        }
    }

    // At least 5 pixels should have been cleared along the steep slope
    EXPECT_GE(clearedCount, 5) << "Steep slope did not clear expected cells.";
}

TEST(Util3dMapping, rayTraceHandlesHorizontalVerticalSlopesCorrectly) {
    // Create a 10x10 grid filled with 50
    cv::Mat grid = cv::Mat::ones(10, 10, CV_8SC1) * 50;

    // Set horizontal slope
    cv::Point2i start(1, 1);
    cv::Point2i end(8, 1);

    util3d::rayTrace(start, end, grid, false);

    for (int x = 1; x < 8; ++x) {
        EXPECT_EQ(grid.at<signed char>(1, x), 0);
    }

    start = cv::Point2i(4, 2);
    end = cv::Point2i(4, 8);
    util3d::rayTrace(start, end, grid, false);

    for (int y = 2; y < 8; ++y) {
        EXPECT_EQ(grid.at<signed char>(y, 4), 0);
    }
}

TEST(Util3dMapping, rayTraceClipsEndPointToBoundary) {
    cv::Mat grid = cv::Mat::ones(10, 10, CV_8SC1) * 50;
    cv::Point2i start(3, 3);
    cv::Point2i end(20, 20); // Way outside bounds

    util3d::rayTrace(start, end, grid, false);

    // All cells along the diagonal from (3,3) to (8,8) should be cleared
    for (int i = 3; i < grid.rows-1 && i < grid.cols-1; ++i) {
        EXPECT_EQ(grid.at<signed char>(i, i), 0);
    }
}

TEST(Util3dMapping, create2DMapBasicMapGeneration)
{
    std::map<int, Transform> poses;
    std::map<int, std::pair<cv::Mat, cv::Mat>> scans;
    std::map<int, cv::Point3f> viewpoints;

    int id = 1;
    float cellSize = 0.05f;
    float xMin, yMin;
    float minMapSize = 1.0f;
    float scanMaxRange = 0.5f;
    bool unknownSpaceFilled = false;

    // Fake pose (origin)
    poses[id] = Transform::getIdentity();

    // Fake viewpoint
    viewpoints[id] = cv::Point3f(-0.1f, -0.0f, 0);

    cv::Mat hit(1, 2, CV_32FC2);
    // Create a hit point under max scan range
    hit.at<cv::Vec2f>(0, 0) = cv::Vec2f(0.2f, 0.2f);
    // Create a hit point over max scan range
    hit.at<cv::Vec2f>(0, 1) = cv::Vec2f(3.0f, 0.0f);

    // No miss points
    cv::Mat noHit(1, 1, CV_32FC2);
    noHit.at<cv::Vec2f>(0, 0) = cv::Vec2f(0.0f, 0.2f);

    scans[id] = std::make_pair(hit, noHit);

    // Call the function
    cv::Mat map = util3d::create2DMap(poses, scans, viewpoints, cellSize, unknownSpaceFilled, xMin, yMin, minMapSize, scanMaxRange);

    // Verify map size
    ASSERT_FALSE(map.empty());
    EXPECT_EQ(map.type(), CV_8S);
    EXPECT_EQ(map.cols, minMapSize/cellSize+20);
    EXPECT_EQ(map.rows, minMapSize/cellSize+20);

    // Verify that the viewpoint is cleared
    EXPECT_EQ(map.at<signed char>(
        (viewpoints[id].y - yMin) / cellSize,
        (viewpoints[id].x - xMin) / cellSize),
        0);

    // Verify that the hit cell is marked as obstacle (100)
    EXPECT_EQ(map.at<signed char>(
        (hit.at<cv::Vec2f>(0, 0)[1] - yMin) / cellSize,
        (hit.at<cv::Vec2f>(0, 0)[0] - xMin) / cellSize),
        100);

    // Verify that ray tracing worked only up to max scan range (from offset viewpoint)
    EXPECT_EQ(map.at<signed char>(
        (hit.at<cv::Vec2f>(0, 1)[1] + viewpoints[id].y - yMin) / cellSize,
        (scanMaxRange + viewpoints[id].x - xMin) / cellSize -1),
        0);

    EXPECT_EQ(map.at<signed char>(
        (hit.at<cv::Vec2f>(0, 1)[1] + viewpoints[id].y - yMin) / cellSize,
        (scanMaxRange + viewpoints[id].x - xMin) / cellSize),
        -1);

    // Verify that the nohit cell is marked as empty (0)
    EXPECT_EQ(map.at<signed char>(
        (noHit.at<cv::Vec2f>(0, 0)[1] - yMin) / cellSize -1,  // ray tracing only up to the nohit cell
        (noHit.at<cv::Vec2f>(0, 0)[0] - xMin) / cellSize),
        0);
    EXPECT_EQ(map.at<signed char>(
        (noHit.at<cv::Vec2f>(0, 0)[1] - yMin) / cellSize,
        (noHit.at<cv::Vec2f>(0, 0)[0] - xMin) / cellSize),
        -1);
}

TEST(Util3dMapping, occupancy2DFromLaserScanBasicTest)
{
    // Synthetic scan with 3 hits and 2 no-hits (in 2D)
    cv::Mat scanHit(1, 3, CV_32FC2);
    scanHit.at<cv::Vec2f>(0, 0) = cv::Vec2f(1.0f, 0.0f);
    scanHit.at<cv::Vec2f>(0, 1) = cv::Vec2f(0.0f, 1.0f);
    scanHit.at<cv::Vec2f>(0, 2) = cv::Vec2f(1.0f, 1.0f);

    cv::Mat scanNoHit(1, 2, CV_32FC2);
    scanNoHit.at<cv::Vec2f>(0, 0) = cv::Vec2f(0.5f, 0.5f);
    scanNoHit.at<cv::Vec2f>(0, 1) = cv::Vec2f(-1.0f, -1.0f);

    cv::Point3f viewpoint(0.0f, 0.0f, 0.0f);

    cv::Mat empty, occupied;
    float cellSize = 0.1f;
    float scanMaxRange = 2.0f;

    util3d::occupancy2DFromLaserScan(scanHit, scanNoHit, viewpoint, empty, occupied, cellSize, true, scanMaxRange);

    // Validate sizes
    ASSERT_FALSE(occupied.empty()) << "Occupied matrix should not be empty.";
    ASSERT_EQ(occupied.cols, 3) << "Expected 3 occupied points.";
    ASSERT_EQ(occupied.type(), CV_32FC2) << "Occupied should be CV_32FC2.";

    ASSERT_FALSE(empty.empty()) << "Empty matrix should not be empty.";
    ASSERT_EQ(empty.type(), CV_32FC2) << "Empty should be CV_32FC2.";
    ASSERT_GT(empty.cols, 20) << "There should be some empty cells.";    
}

TEST(Util3dMapping, create2DMapFromOccupancyLocalMapsBasic)
{
    // Setup poses
    std::map<int, Transform> poses;
    poses[1] = Transform::getIdentity();  // Assume (0, 0)

    // Setup occupancy data
    std::map<int, std::pair<cv::Mat, cv::Mat>> occupancy;

    // Create dummy empty cells
    cv::Mat empty(1, 2, CV_32FC2);
    empty.at<cv::Vec2f>(0, 0) = cv::Vec2f(1.0f, 1.0f);
    empty.at<cv::Vec2f>(0, 1) = cv::Vec2f(2.0f, 1.0f);

    // Create dummy occupied cells
    cv::Mat occupied(1, 1, CV_32FC2);
    occupied.at<cv::Vec2f>(0, 0) = cv::Vec2f(1.5f, 2.0f);

    occupancy[1] = std::make_pair(empty, occupied);

    // Output parameters
    float xMin = 0.0f;
    float yMin = 0.0f;
    float cellSize = 0.1f;

    // Run function
    cv::Mat map = util3d::create2DMapFromOccupancyLocalMaps(
        poses,
        occupancy,
        cellSize,
        xMin,
        yMin,
        0.0f,       // minMapSize
        false,      // erode
        0.0f        // footprintRadius
    );

    // Verify output
    ASSERT_FALSE(map.empty());
    ASSERT_EQ(map.type(), CV_8S);

    // Check expected occupancy value at occupied cell
    int col = static_cast<int>((occupied.at<cv::Vec2f>(0, 0)[0] - xMin) / cellSize);
    int row = static_cast<int>((occupied.at<cv::Vec2f>(0, 0)[1] - yMin) / cellSize);
    ASSERT_GE(row, 0);
    ASSERT_GE(col, 0);
    ASSERT_LT(row, map.rows);
    ASSERT_LT(col, map.cols);
    EXPECT_EQ(map.at<signed char>(row, col), 100); // Obstacle

    // Check empty cells
    int rowEmpty = static_cast<int>((empty.at<cv::Vec2f>(0, 0)[1] - yMin) / cellSize);
    int colEmpty1 = static_cast<int>((empty.at<cv::Vec2f>(0, 0)[0] - xMin) / cellSize);
    int colEmpty2 = static_cast<int>((empty.at<cv::Vec2f>(0, 1)[0] - xMin) / cellSize);
    EXPECT_EQ(map.at<signed char>(rowEmpty, colEmpty1), 0); // Free
    EXPECT_EQ(map.at<signed char>(rowEmpty, colEmpty2), 0); // Free
}

TEST(Util3dMapping, convertMap2Image8UBasicConversionNormalFormat)
{
    // Create a simple 3x3 CV_8S occupancy grid
    cv::Mat map8S = (cv::Mat_<signed char>(3, 3) <<
        -1,  0, 100,
        -2, 50, 75,
        25, -1, 0);

    // Call the function in normal format
    cv::Mat result = util3d::convertMap2Image8U(map8S, false);

    ASSERT_EQ(result.type(), CV_8U);
    ASSERT_EQ(result.rows, 3);
    ASSERT_EQ(result.cols, 3);

    // Expected grayscale values for normal format
    EXPECT_EQ(result.at<uchar>(0, 0), 89);   // -1 (unknown)
    EXPECT_EQ(result.at<uchar>(0, 1), 178);  // 0 (free)
    EXPECT_EQ(result.at<uchar>(0, 2), 0);    // 100 (obstacle)
    EXPECT_EQ(result.at<uchar>(1, 0), 200);  // -2 (footprint)
    EXPECT_EQ(result.at<uchar>(1, 1), 89);   // 50
    EXPECT_LT(result.at<uchar>(1, 2), 89);   // 75 → scaled toward obstacle (0–89)
    EXPECT_GT(result.at<uchar>(2, 0), 89);   // 25 → scaled toward free (89–178)
    EXPECT_EQ(result.at<uchar>(2, 1), 89);   // -1
    EXPECT_EQ(result.at<uchar>(2, 2), 178);  // 0
}

TEST(Util3dMapping, convertMap2Image8UBasicConversionPGMFormat)
{
    cv::Mat map8S = (cv::Mat_<signed char>(2, 2) <<
        -1, 0,
        -2, 100);

    // Call with pgmFormat = true
    cv::Mat result = util3d::convertMap2Image8U(map8S, true);

    ASSERT_EQ(result.type(), CV_8U);
    ASSERT_EQ(result.rows, 2);
    ASSERT_EQ(result.cols, 2);

    // Because pgmFormat flips vertically, test accordingly
    EXPECT_EQ(result.at<uchar>(0, 0), 254);  // 0 (was at [1,0])
    EXPECT_EQ(result.at<uchar>(0, 1), 0);    // 100
    EXPECT_EQ(result.at<uchar>(1, 0), 205);  // -1
    EXPECT_EQ(result.at<uchar>(1, 1), 254);  // -2
}

TEST(Util3dMapping, convertImage8U2MapNonPGMFormat)
{
    // Create a 2x2 grayscale image using non-PGM expected values
    cv::Mat input = (cv::Mat_<uchar>(2, 2) << 178, 0, 200, 89);

    // Call conversion
    cv::Mat map = util3d::convertImage8U2Map(input, false);

    ASSERT_EQ(map.type(), CV_8S);
    ASSERT_EQ(map.rows, 2);
    ASSERT_EQ(map.cols, 2);

    // Expected values: 0 (free), 100 (occupied), -2 (footprint), -1 (unknown)
    EXPECT_EQ(map.at<signed char>(0, 0), 0);
    EXPECT_EQ(map.at<signed char>(0, 1), 100);
    EXPECT_EQ(map.at<signed char>(1, 0), -2);
    EXPECT_EQ(map.at<signed char>(1, 1), -1);
}

TEST(Util3dMapping, convertImage8U2MapPGMFormat)
{
    // Create a 2x2 grayscale image using PGM expected values
    cv::Mat input = (cv::Mat_<uchar>(2, 2) << 254, 0, 205, 205);

    // Call conversion
    cv::Mat map = util3d::convertImage8U2Map(input, true);

    ASSERT_EQ(map.type(), CV_8S);
    ASSERT_EQ(map.rows, 2);
    ASSERT_EQ(map.cols, 2);

    // Because PGM inverts rows vertically, we need to check accordingly
    // map.at<signed char>(i, j) corresponds to input.at<uchar>((rows-1)-i, j)
    EXPECT_EQ(map.at<signed char>(0, 0), -1);  // input(1, 0) = 205
    EXPECT_EQ(map.at<signed char>(0, 1), -1);  // input(1, 1) = 205
    EXPECT_EQ(map.at<signed char>(1, 0), 0);   // input(0, 0) = 254
    EXPECT_EQ(map.at<signed char>(1, 1), 100); // input(0, 1) = 0
}

TEST(Util3dMapping, erodeMapBasicErosion)
{
    // Create a 5x5 map with CV_8SC1 type
    // -1 = unknown, 0 = free, 100 = obstacle
    cv::Mat map = (cv::Mat_<signed char>(5,5) <<
        0,   0,   0,   0,  0,
        0, 100, 100, 100,  0,
        0,   0, 100, 100,  100,
        0, 100, 100, 100,  100,
        0,   0, 100, 100,  100);

    cv::Mat erodedMap = util3d::erodeMap(map);

    cv::Mat expected = (cv::Mat_<signed char>(5,5) <<
        0,   0,   0,   0,  0,
        0,   0, 100, 100,  0,
        0,   0, 100, 100,  100,
        0,   0, 100, 100,  100,
        0,   0, 100, 100,  100);

    cv::Mat diff;
    cv::compare(erodedMap, expected, diff, cv::CMP_NE);
    // Count non-zero elements in diff (means different)
    EXPECT_EQ(cv::countNonZero(diff), 0);
    
}

TEST(Util3dMapping, erodeMapNoErosionWithUnknown)
{
    // Create a 3x3 map with obstacle touching unknown cell
    cv::Mat map = (cv::Mat_<signed char>(3,3) <<
        0,  0,  0,
        0, 100, -1,
        0,  0,  0);

    cv::Mat erodedMap = util3d::erodeMap(map);

    // Obstacle should NOT be eroded because adjacent unknown cell (-1)
    EXPECT_EQ(erodedMap.at<signed char>(1,1), 100);
}

TEST(Util3dMapping, projectCloudOnXYPlaneZCoordinatesAreZero)
{
    // Create a test point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    input_cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    input_cloud->push_back(pcl::PointXYZ(4.0, 5.0, -1.0));
    input_cloud->push_back(pcl::PointXYZ(0.0, 0.0, 10.0));

    // Project the cloud
    auto projected_cloud = util3d::projectCloudOnXYPlane<pcl::PointXYZ>(*input_cloud);

    // Check that the size remains the same
    ASSERT_EQ(projected_cloud->size(), input_cloud->size());

    // Check that x and y remain the same, and z is set to zero
    for (size_t i = 0; i < projected_cloud->size(); ++i)
    {
        EXPECT_FLOAT_EQ(projected_cloud->at(i).x, input_cloud->at(i).x);
        EXPECT_FLOAT_EQ(projected_cloud->at(i).y, input_cloud->at(i).y);
        EXPECT_FLOAT_EQ(projected_cloud->at(i).z, 0.0f);
    }
}

TEST(Util3dMapping, segmentObstaclesFromGround)
{
    // Create a cloud of a floor, then elevate some part of it to make a flat obstacle
    pcl::IndicesPtr expected_ground(new std::vector<int>);
    pcl::IndicesPtr expected_big_obstacles(new std::vector<int>);
    pcl::IndicesPtr expected_small_obstacles(new std::vector<int>);
    pcl::IndicesPtr expected_flat_obstacles(new std::vector<int>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<10; ++i) {
        for(int j=0; j<10; ++j) {
            if(i>5 && j>5) {
                expected_flat_obstacles->push_back(cloud->size());
            }
            else {
                expected_ground->push_back(cloud->size());
            }
            cloud->push_back(pcl::PointXYZ(0.05f*i, 0.05f*j, 0.01f*i+(i>5 && j>5 ? 0.25f:0.0f)));
        }
    }
    // Add a wall and a small obstacle
    for(int i=0; i<10; ++i) {
        for(int k=0; k<10; ++k) {
            if(i>5 && k>5) {
                expected_small_obstacles->push_back(cloud->size());
                cloud->push_back(pcl::PointXYZ(0.05f*i, -0.35f, 0.05f*k));
            }
            expected_big_obstacles->push_back(cloud->size());
            cloud->push_back(pcl::PointXYZ(0.05f*i, -0.15f, 0.05f*k));
        }
    }

    pcl::IndicesPtr ground, obstacles, flatObs;

    // test basic
    float normalKSearch = 5;
    float angleMax = 20.0f*M_PI/180.0f;  // Allow 20 degrees of deviation
    float clusterRadius = 0.1f;
    util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
        cloud,
        ground,
        obstacles,
        normalKSearch,       
        angleMax,     
        clusterRadius,
        1,        // minClusterSize
        false,    // segmentFlatObstacles
        0.0f,     // maxGroundHeight
        &flatObs, // flatObstacles
        Eigen::Vector4f(0,0,2,0), // viewpoint
        0.8f      // groundNormalsUp
    );

    ASSERT_EQ(ground->size(), expected_ground->size() + expected_flat_obstacles->size());
    ASSERT_EQ(obstacles->size(), expected_big_obstacles->size() + expected_small_obstacles->size());

    // test indices
    util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
        cloud,
        expected_ground,
        ground,
        obstacles,
        normalKSearch,
        angleMax,
        clusterRadius,
        1,        // minClusterSize
        false,    // segmentFlatObstacles
        0.0f,     // maxGroundHeight
        &flatObs, // flatObstacles
        Eigen::Vector4f(0,0,2,0), // viewpoint
        0.8f      // groundNormalsUp
    );

    ASSERT_EQ(ground->size(), expected_ground->size());
    ASSERT_EQ(obstacles->size(), 0);

    // test flat obstacles
    util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
        cloud,
        ground,
        obstacles,
        normalKSearch,
        angleMax,
        clusterRadius,
        1,        // minClusterSize
        true,    // segmentFlatObstacles
        0.0f,     // maxGroundHeight
        &flatObs,  // flatObstacles
        Eigen::Vector4f(0,0,2,0), // viewpoint
        0.8f      // groundNormalsUp
    );

    ASSERT_EQ(flatObs->size(), expected_flat_obstacles->size());
    ASSERT_EQ(ground->size(), expected_ground->size());
    ASSERT_EQ(obstacles->size(), expected_big_obstacles->size() + expected_small_obstacles->size() + expected_flat_obstacles->size());

    // test flat obstacles with maxGroundHeight
    for(int i=0; i<2; ++i) {
        util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
            cloud,
            ground,
            obstacles,
            normalKSearch,
            angleMax,
            clusterRadius,
            1,        // minClusterSize
            i==0,    // segmentFlatObstacles
            0.1f,     // maxGroundHeight
            &flatObs,  // flatObstacles
            Eigen::Vector4f(0,0,2,0), // viewpoint
            0.8f      // groundNormalsUp
        );

        ASSERT_EQ(flatObs->size(), expected_flat_obstacles->size());
        ASSERT_EQ(ground->size(), expected_ground->size());
        // all obstacles under maxGroundHeight are ignored
        ASSERT_EQ(obstacles->size(), expected_big_obstacles->size() + expected_small_obstacles->size() + expected_flat_obstacles->size() - 20);
    }

    // test viewpoint (ceiling segmentation)
    util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
        cloud,
        ground,
        obstacles,
        normalKSearch,
        angleMax,
        clusterRadius,
        1,        // minClusterSize
        false,    // segmentFlatObstacles
        0.0f,     // maxGroundHeight
        &flatObs,  // flatObstacles
        Eigen::Vector4f(0,0,0.15f,0), // viewpoint under the top flat obstacle
        0.8f      // groundNormalsUp
    );

    ASSERT_EQ(ground->size(), expected_ground->size());
    ASSERT_EQ(obstacles->size(), expected_big_obstacles->size() + expected_small_obstacles->size() + expected_flat_obstacles->size());

    // test min cluster radius 
    util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
        cloud,
        ground,
        obstacles,
        normalKSearch,
        angleMax,
        clusterRadius,
        17,        // minClusterSize
        false,    // segmentFlatObstacles
        0.0f,     // maxGroundHeight
        &flatObs,  // flatObstacles
        Eigen::Vector4f(0,0,2,0),
        0.8f      // groundNormalsUp
    );

    ASSERT_EQ(ground->size(), expected_ground->size());
    ASSERT_EQ(obstacles->size(), expected_big_obstacles->size());

    // Everything obstacles
    util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
        cloud,
        ground,
        obstacles,
        normalKSearch,
        angleMax,
        clusterRadius,
        1,        // minClusterSize
        false,    // segmentFlatObstacles
        -0.1f,     // maxGroundHeight
        &flatObs,  // flatObstacles
        Eigen::Vector4f(0,0,2,0),
        0.8f      // groundNormalsUp
    );

    ASSERT_EQ(ground->size(), 0);
    ASSERT_EQ(obstacles->size(), expected_ground->size() + expected_big_obstacles->size() + expected_small_obstacles->size() + expected_flat_obstacles->size());

    // Everything ground or ignored
    for(int i=0; i<2; ++i) {
        util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
            cloud,
            ground,
            obstacles,
            normalKSearch,
            angleMax,
            clusterRadius,
            1,        // minClusterSize
            i==0,    // segmentFlatObstacles
            10,     // maxGroundHeight
            &flatObs,  // flatObstacles
            Eigen::Vector4f(0,0,2,0),
            0.8f      // groundNormalsUp
        );
        // wether we segment or not, all flat surfaces are under 10 meters
        ASSERT_EQ(ground->size(), expected_ground->size() + expected_flat_obstacles->size());
        ASSERT_EQ(obstacles->size(), 0);
    }
}

TEST(Util3dMapping, occupancy2DFromGroundObstaclesBasic)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Ground points (clustered near 0,0)
    groundCloud->push_back(pcl::PointXYZ(0.05f, 0.05f, -0.2f));
    groundCloud->push_back(pcl::PointXYZ(0.06f, 0.04f, -0.1f));
    groundCloud->push_back(pcl::PointXYZ(0.5f, 0.5f, -0.2f)); // Separate voxel

    // Obstacle points
    obstaclesCloud->push_back(pcl::PointXYZ(1.0f, 1.0f, 1.0f));
    obstaclesCloud->push_back(pcl::PointXYZ(1.02f, 1.01f, 1.2f)); // Same voxel
    obstaclesCloud->push_back(pcl::PointXYZ(2.0f, 2.0f, 1.5f));

    cv::Mat groundMat, obstaclesMat;
    float cellSize = 0.1f;

    util3d::occupancy2DFromGroundObstacles<pcl::PointXYZ>(groundCloud, obstaclesCloud, groundMat, obstaclesMat, cellSize);

    // Check that points were voxelized and projected
    EXPECT_EQ(groundMat.rows, 1);
    EXPECT_EQ(groundMat.cols, 2); // Expect 2 distinct voxels
    EXPECT_EQ(groundMat.type(), CV_32FC2);

    EXPECT_NEAR(groundMat.at<cv::Vec2f>(0)[0], 0.055, 0.001);
    EXPECT_NEAR(groundMat.at<cv::Vec2f>(0)[1], 0.045, 0.001);
    EXPECT_NEAR(groundMat.at<cv::Vec2f>(1)[0], 0.5, 0.001);
    EXPECT_NEAR(groundMat.at<cv::Vec2f>(1)[1], 0.5, 0.001);

    EXPECT_EQ(obstaclesMat.rows, 1);
    EXPECT_EQ(obstaclesMat.cols, 2); // Expect 2 voxels (1.0,1.0) and (2.0,2.0)
    EXPECT_EQ(obstaclesMat.type(), CV_32FC2);

    EXPECT_NEAR(obstaclesMat.at<cv::Vec2f>(0)[0], 1.01, 0.001);
    EXPECT_NEAR(obstaclesMat.at<cv::Vec2f>(0)[1], 1.005, 0.001);
    EXPECT_NEAR(obstaclesMat.at<cv::Vec2f>(1)[0], 2, 0.001);
    EXPECT_NEAR(obstaclesMat.at<cv::Vec2f>(1)[1], 2, 0.001);
}