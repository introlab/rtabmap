#include "gtest/gtest.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/utilite/UException.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/core/Version.h"
#include <pcl/io/pcd_io.h>

using namespace rtabmap;

// Utility to generate a flat plane of normals pointing up
pcl::PointCloud<pcl::PointNormal> createFlatNormalCloud(int count, const cv::Point3f& normal)
{
    pcl::PointCloud<pcl::PointNormal> cloud;
    cloud.resize(count);
    for (int i = 0; i < count; ++i)
    {
        cloud[i].normal_x = normal.x;
        cloud[i].normal_y = normal.y;
        cloud[i].normal_z = normal.z;
    }
    return cloud;
}

TEST(Util3dSurface, computeNormalsComplexityVaryingNormals3D)
{
    auto floor = createFlatNormalCloud(100, cv::Point3f(0.0f, 0.0f, 1.0f));
    auto wallA = createFlatNormalCloud(100, cv::Point3f(0.0f, 1.0f, 0.0f));
    auto wallB = createFlatNormalCloud(100, cv::Point3f(1.0f, 0.0f, 0.0f));
    auto smallWallB = createFlatNormalCloud(10, cv::Point3f(1.0f, 0.0f, 0.0f));

    // One flat surface
    float complexity = util3d::computeNormalsComplexity(floor);
    EXPECT_NEAR(complexity, 0.0f, 1e-3);

    pcl::PointCloud<pcl::PointNormal> cloudA;
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
    pcl::concatenate(floor, wallA, cloudA);
#else
    pcl::concatenatePointCloud(floor, wallA, cloudA);
#endif
    // Two perpendicular surfaces
    complexity = util3d::computeNormalsComplexity(cloudA);
    EXPECT_NEAR(complexity, 0.0f, 1e-3);

    // Three perpendicular surfaces
    pcl::PointCloud<pcl::PointNormal> cloudB;
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
    pcl::concatenate(cloudA, wallB, cloudB);
#else
    pcl::concatenatePointCloud(cloudA, wallB, cloudB);
#endif
    
    complexity = util3d::computeNormalsComplexity(cloudB);
    EXPECT_NEAR(complexity, 0.25f, 1e-3);

    // Three perpendicular surfaces (one small)
    pcl::PointCloud<pcl::PointNormal> smallCloudB;
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
    pcl::concatenate(cloudA, smallWallB, smallCloudB);
#else
    pcl::concatenatePointCloud(cloudA, smallWallB, smallCloudB);
#endif
    
    complexity = util3d::computeNormalsComplexity(smallCloudB);
    EXPECT_LT(complexity, 0.25f);
    EXPECT_GT(complexity, 0.01f);
}

TEST(Util3dSurface, computeNormalsComplexityIdentityVsRotated)
{
    auto cloud = createFlatNormalCloud(50, cv::Point3f(0.0f, 1.0f, 0.0f));
    Transform identity = Transform::getIdentity();
    Transform rotated = Transform(0,0,0,0,M_PI / 4,0);

    float c1 = util3d::computeNormalsComplexity(cloud, identity, false, nullptr, nullptr);
    float c2 = util3d::computeNormalsComplexity(cloud, rotated, false, nullptr, nullptr);

    EXPECT_NEAR(c1, c2, 1e-5); // rotation should not affect complexity
}

TEST(Util3dSurface, computeNormalsComplexityEmptyOrInvalidNormals)
{
    pcl::PointCloud<pcl::PointNormal> cloud;
    pcl::PointNormal pt;
    pt.normal_x = std::numeric_limits<float>::quiet_NaN();
    pt.normal_y = 0.0f;
    pt.normal_z = 0.0f;
    cloud.push_back(pt);

    float complexity = util3d::computeNormalsComplexity(cloud, Transform(), false, nullptr, nullptr);
    EXPECT_EQ(complexity, 0.0f); // Should return 0 when all normals are invalid
}

TEST(Util3dSurface, computeNormalsComplexityVaryingNormals2D)
{
    auto wallA = createFlatNormalCloud(100, cv::Point3f(0.0f, 1.0f, 0.0f));
    auto negWallA = createFlatNormalCloud(100, cv::Point3f(0.0f, -1.0f, 0.0f));
    auto wallB = createFlatNormalCloud(100, cv::Point3f(1.0f, 0.0f, 0.0f));
    auto smalllWallB = createFlatNormalCloud(10, cv::Point3f(1.0f, 0.0f, 0.0f));

    // One flat surface
    float complexity = util3d::computeNormalsComplexity(wallA, Transform(), true);
    EXPECT_NEAR(complexity, 0.0f, 1e-3);

    complexity = util3d::computeNormalsComplexity(wallB, Transform(), true);
    EXPECT_NEAR(complexity, 0.0f, 1e-3);

    pcl::PointCloud<pcl::PointNormal> cloud;
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
    pcl::concatenate(wallA, wallB, cloud);
#else
    pcl::concatenatePointCloud(wallA, wallB, cloud);
#endif
    // Two perpendicular surfaces
    complexity = util3d::computeNormalsComplexity(cloud, Transform(), true);
    EXPECT_NEAR(complexity, 0.25f, 1e-3);

    pcl::PointCloud<pcl::PointNormal> cloudB;
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
    pcl::concatenate(wallA, smalllWallB, cloudB);
#else
    pcl::concatenatePointCloud(wallA, smalllWallB, cloudB);
#endif
    // Two perpendicular surfaces (one small)
    complexity = util3d::computeNormalsComplexity(cloudB, Transform(), true);
    EXPECT_LT(complexity, 0.25f);
    EXPECT_GT(complexity, 0.01f);

    pcl::PointCloud<pcl::PointNormal> corridorLikeCloud;
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
    pcl::concatenate(wallA, negWallA, corridorLikeCloud);
#else
    pcl::concatenatePointCloud(wallA, negWallA, corridorLikeCloud);
#endif
    // Two parallel surfaces simulating a corridor
    cv::Mat vector,values;
    complexity = util3d::computeNormalsComplexity(corridorLikeCloud, Transform(), true, &vector, &values);
    EXPECT_NEAR(complexity, 0.0f, 1e-3);
    EXPECT_NEAR(vector.at<float>(0,0), 0, 1e-3);
    EXPECT_NEAR(vector.at<float>(0,1), 1, 1e-3); // first eigen vector should be aligned with the normals
}