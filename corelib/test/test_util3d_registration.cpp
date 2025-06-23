#include "gtest/gtest.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/utilite/UException.h"
#include <pcl/common/impl/angles.hpp>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

using namespace rtabmap;

TEST(Util3DRegistration, transformFromXYZCorrespondencesSVDIdentityTransform)
{
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    cloud1.push_back(pcl::PointXYZ(1, 2, 3));
    cloud1.push_back(pcl::PointXYZ(4, 5, 6));
    cloud1.push_back(pcl::PointXYZ(7, 8, 9));
    cloud1.push_back(pcl::PointXYZ(10, 11, 12));
    cloud1.push_back(pcl::PointXYZ(1, 6, 12));
    cloud2 = cloud1; // Exact same points

    Transform result = util3d::transformFromXYZCorrespondencesSVD(cloud1, cloud2);
    Transform identity = Transform::getIdentity();

    EXPECT_LT(result.getDistance(identity), 0.001f);
    EXPECT_LT(result.getAngle(identity), 0.001f);
}

TEST(Util3DRegistration, transformFromXYZCorrespondencesSVDTranslationOnly)
{
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    cloud1.push_back(pcl::PointXYZ(0, 0, 0));
    cloud1.push_back(pcl::PointXYZ(1, 0, 0));
    cloud1.push_back(pcl::PointXYZ(0, 1, 0));

    cloud2.push_back(pcl::PointXYZ(1, 2, 3));
    cloud2.push_back(pcl::PointXYZ(2, 2, 3));
    cloud2.push_back(pcl::PointXYZ(1, 3, 3));

    Transform result = util3d::transformFromXYZCorrespondencesSVD(cloud1, cloud2);

    EXPECT_LT(result.getAngle(Transform::getIdentity()), 0.001f);
    EXPECT_NEAR(result.x(), 1.0f, 1e-4f);
    EXPECT_NEAR(result.y(), 2.0f, 1e-4f);
    EXPECT_NEAR(result.z(), 3.0f, 1e-4f);
}

TEST(Util3DRegistration, transformFromXYZCorrespondencesSVDRotationAndTranslation)
{
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;

    // Original triangle
    cloud1.push_back(pcl::PointXYZ(1, 0, 0));
    cloud1.push_back(pcl::PointXYZ(0, 1, 0));
    cloud1.push_back(pcl::PointXYZ(0, 0, 1));

    // Apply known rotation (90° about Z) and translation (1, 2, 3)
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
    Eigen::Vector3f t(1, 2, 3);

    for (const auto & pt : cloud1.points)
    {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        p = R * p + t;
        cloud2.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
    }

    Transform result = util3d::transformFromXYZCorrespondencesSVD(cloud1, cloud2);

    Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
    expected.block<3,3>(0,0) = R;
    expected.block<3,1>(0,3) = t;
    Transform expected_t = Transform::fromEigen4f(expected);

    EXPECT_LT(result.getAngle(expected_t), 0.001f);
    EXPECT_NEAR(result.x(), expected_t.x(), 1e-4f);
    EXPECT_NEAR(result.y(), expected_t.y(), 1e-4f);
    EXPECT_NEAR(result.z(), expected_t.z(), 1e-4f);
}

TEST(Util3DRegistration, transformFromXYZCorrespondencesSVDMismatchedSizes)
{
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    cloud1.push_back(pcl::PointXYZ(0, 0, 0));
    cloud1.push_back(pcl::PointXYZ(1, 1, 1));
    cloud2.push_back(pcl::PointXYZ(0, 0, 0)); // Only 1 point

    EXPECT_THROW(util3d::transformFromXYZCorrespondencesSVD(cloud1, cloud2), UException);
}

TEST(Util3DRegistration, transformFromXYZCorrespondencesIdentityTransform)
{
    auto cloud1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud1->push_back(pcl::PointXYZ(1, 0, 0));
    cloud1->push_back(pcl::PointXYZ(0, 1, 0));
    cloud1->push_back(pcl::PointXYZ(0, 0, 1));

    auto cloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud1);

    std::vector<int> inliers;
    cv::Mat covariance;

    Transform result = util3d::transformFromXYZCorrespondences(
        cloud1, cloud2, 0.01, 100, 0, 1.0, &inliers, &covariance);

    Transform identity = Transform::getIdentity();
    EXPECT_LT(result.getDistance(identity), 0.001f);
    EXPECT_LT(result.getAngle(identity), 0.001f);
    EXPECT_EQ(inliers.size(), 3);
    EXPECT_EQ(covariance.rows, 6);
    EXPECT_EQ(covariance.cols, 6);
}

TEST(Util3DRegistration, transformFromXYZCorrespondencesTranslatedCloud)
{
    auto cloud1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Eigen::Vector3f t(1.0f, 2.0f, 3.0f);

    for (int i = 0; i < 3; ++i)
    {
        pcl::PointXYZ p(i, i * 2, i * 3);
        cloud1->push_back(p);
        cloud2->push_back(pcl::PointXYZ(p.x + t.x(), p.y + t.y(), p.z + t.z()));
    }

    std::vector<int> inliers;
    cv::Mat covariance;

    Transform result = util3d::transformFromXYZCorrespondences(
        cloud1, cloud2, 0.1, 100, 0, 1.0, &inliers, &covariance);

    EXPECT_LT(result.getAngle(Transform::getIdentity()), 0.001f);
    EXPECT_NEAR(result.x(), t.x(), 1e-4f);
    EXPECT_NEAR(result.y(), t.y(), 1e-4f);
    EXPECT_NEAR(result.z(), t.z(), 1e-4f);
    EXPECT_EQ(inliers.size(), 3);
}

TEST(Util3DRegistration, transformFromXYZCorrespondencesTooFewPoints)
{
    auto cloud1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    cloud1->push_back(pcl::PointXYZ(0, 0, 0));
    cloud2->push_back(pcl::PointXYZ(1, 1, 1));

    Transform result = util3d::transformFromXYZCorrespondences(
        cloud1, cloud2, 0.1, 100, 0, 1.0, nullptr, nullptr);

    EXPECT_TRUE(result.isNull());
}

TEST(Util3DRegistration, transformFromXYZCorrespondencesMismatchedPointCounts)
{
    auto cloud1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    cloud1->push_back(pcl::PointXYZ(0, 0, 0));
    cloud1->push_back(pcl::PointXYZ(1, 0, 0));
    cloud1->push_back(pcl::PointXYZ(0, 1, 0));

    cloud2->push_back(pcl::PointXYZ(0, 0, 0)); // Only one point

    Transform result = util3d::transformFromXYZCorrespondences(
        cloud1, cloud2, 0.1, 100, 0, 1.0, nullptr, nullptr);

    EXPECT_TRUE(result.isNull());
}

TEST(Util3DRegistration, computeVarianceAndCorrespondencesPerfectMatchNoAngleCheck)
{
    auto cloud1 = std::make_shared<pcl::PointCloud<pcl::PointNormal> >();
    auto cloud2 = std::make_shared<pcl::PointCloud<pcl::PointNormal> >();

    for (int i = 0; i < 5; ++i)
    {
        pcl::PointNormal pt;
        pt.x = i; pt.y = i; pt.z = i;
        pt.normal_x = 1; pt.normal_y = 0; pt.normal_z = 0;
        cloud1->push_back(pt);
        cloud2->push_back(pt); // identical
    }

    double variance = -1.0;
    int correspondences = -1;
    util3d::computeVarianceAndCorrespondences(
        cloud1, cloud2, 0.1, -1.0, variance, correspondences, true);

    EXPECT_EQ(correspondences, 5);
    EXPECT_DOUBLE_EQ(variance, 0.0);
}

TEST(Util3DRegistration, computeVarianceAndCorrespondencesNormalMismatchFilteredByAngle)
{
    auto cloud1 = std::make_shared<pcl::PointCloud<pcl::PointNormal> >();
    auto cloud2 = std::make_shared<pcl::PointCloud<pcl::PointNormal> >();

    for (int i = 0; i < 5; ++i)
    {
        pcl::PointNormal a, b;
        a.x = b.x = i; a.y = b.y = i; a.z = b.z = i;

        a.normal_x = 1; a.normal_y = 0; a.normal_z = 0;
        b.normal_x = 0; b.normal_y = 1; b.normal_z = 0; // orthogonal normals

        cloud1->push_back(a);
        cloud2->push_back(b);
    }

    double variance = -1.0;
    int correspondences = -1;

    util3d::computeVarianceAndCorrespondences(
        cloud1, cloud2, 0.1, pcl::deg2rad(45.0), variance, correspondences, true);

    EXPECT_EQ(correspondences, 0);
    EXPECT_DOUBLE_EQ(variance, 1.0); // untouched default value
}

TEST(Util3DRegistration, computeVarianceAndCorrespondencesAnglePassWithLargeThreshold)
{
    auto cloud1 = std::make_shared<pcl::PointCloud<pcl::PointNormal> >();
    auto cloud2 = std::make_shared<pcl::PointCloud<pcl::PointNormal> >();

    for (int i = 0; i < 5; ++i)
    {
        pcl::PointNormal a, b;
        a.x = b.x = i; a.y = b.y = i; a.z = b.z = i;

        a.normal_x = 1; a.normal_y = 0; a.normal_z = 0;
        b.normal_x = 0.7f; b.normal_y = 0.7f; b.normal_z = 0;

        cloud1->push_back(a);
        cloud2->push_back(b);
    }

    double variance = -1.0;
    int correspondences = -1;

    util3d::computeVarianceAndCorrespondences(
        cloud1, cloud2, 0.1, pcl::deg2rad(90.0), variance, correspondences, true);

    EXPECT_EQ(correspondences, 5);
    EXPECT_GE(variance, 0.0);
}

TEST(Util3DRegistration, computeVarianceAndCorrespondencesNoCorrespondencesDueToDistance)
{
    auto cloud1 = std::make_shared<pcl::PointCloud<pcl::PointNormal> >();
    auto cloud2 = std::make_shared<pcl::PointCloud<pcl::PointNormal> >();

    for (int i = 0; i < 5; ++i)
    {
        pcl::PointNormal pt1, pt2;
        pt1.x = pt2.x = i;
        pt1.y = pt2.y = i;
        pt1.z = pt2.z = i;

        pt1.normal_x = pt2.normal_x = 1;
        pt1.normal_y = pt2.normal_y = 0;
        pt1.normal_z = pt2.normal_z = 0;

        pt2.x += 100; // make them too far apart

        cloud1->push_back(pt1);
        cloud2->push_back(pt2);
    }

    double variance = -1.0;
    int correspondences = -1;

    util3d::computeVarianceAndCorrespondences(
        cloud1, cloud2, 0.5, 0.0, variance, correspondences, true);

    EXPECT_EQ(correspondences, 0);
    EXPECT_DOUBLE_EQ(variance, 1.0); // default untouched
}

TEST(Util3DRegistration, icpIdentityTransformConverges)
{
    auto cloud_source = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    for (float i = 0; i < 5; ++i)
    {
        cloud_source->emplace_back(i, i * 2.0f, 0.0f);
    }

    auto cloud_target = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >(*cloud_source); // identical

    bool hasConverged = false;
    pcl::PointCloud<pcl::PointXYZ> aligned;
    Transform result = util3d::icp(
        cloud_source, cloud_target,
        0.1,             // max correspondence distance
        50,              // max iterations
        hasConverged,
        aligned,
        1e-6f,           // epsilon
        false            // 3D ICP
    );

    EXPECT_TRUE(hasConverged);
    // ICP should return identity transform for identical clouds
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    EXPECT_TRUE(result.toEigen4f().isApprox(identity, 1e-4));
}

TEST(Util3DRegistration, icpTranslatedTransformConverges)
{
    auto cloud_source = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    for (float i = 0; i < 5; ++i)
    {
        cloud_source->emplace_back(i, i * 2.0f, 0);
    }

    // Translate the cloud
    Transform transformGT(0.025, 0, 0.0f ,0,0,0);
    auto cloud_target = util3d::transformPointCloud(cloud_source, transformGT);

    bool hasConverged = false;
    pcl::PointCloud<pcl::PointXYZ> aligned;
    Transform result = util3d::icp(
        cloud_source, cloud_target,
        0.05, 100, hasConverged, aligned,
        1e-6f,
        false
    );

    EXPECT_TRUE(hasConverged);

    std::cout << result << std::endl;
    std::cout << transformGT << std::endl;

    Eigen::Matrix4f estimated = result.toEigen4f();
    Eigen::Matrix4f expected = transformGT.toEigen4f();
    EXPECT_TRUE(estimated.isApprox(expected, 1e-2));
}

TEST(Util3DRegistration, icp2DAlignsFlatClouds)
{
    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    auto cloud_source = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (float x = 0; x < 5; ++x)
    {
        for (float y = 0; y < 5; ++y)
        {
            if(y == 0 || y == 2 || y == 4 || x==0 || x==2 || x== 4){
                cloud_source->emplace_back(x*0.05, y*0.05, (int)x%2==0?0.01:-0.01);
            }
        }
    }

    Transform transformGT(0.075, 0.05, 0.01f, 0,0, M_PI / 8);

    auto cloud_target = util3d::transformPointCloud(cloud_source, transformGT);

    bool hasConverged = false;
    pcl::PointCloud<pcl::PointXYZ> aligned;
    Transform result = util3d::icp(
        cloud_source, cloud_target,
        0.15, 100, hasConverged, aligned,
        1e-6f,
        true // use ICP 2D
    );

    transformGT.z() = 0;

    EXPECT_TRUE(hasConverged);
    Eigen::Matrix4f estimated = result.toEigen4f();
    Eigen::Matrix4f expected = transformGT.toEigen4f();
    EXPECT_TRUE(estimated.isApprox(expected, 1e-4));
}

TEST(Util3DRegistration, icpPointToPlaneAlignsTranslatedPlane)
{
    // Create a plane point cloud
    auto cloud_source_raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    for (float x = -0.5f; x <= 0.5f; x += 0.1f)
    {
        for (float y = -0.5f; y <= 0.5f; y += 0.1f)
        {
            cloud_source_raw->emplace_back(x, y, int(x*10)%2==0&&int(y*10)%2==0?0.01:-0.01);
        }
    }

    // Compute normals
    auto normals = util3d::computeNormals(cloud_source_raw, 20, 0, Eigen::Vector3f(0,0,1));
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_source_raw, *normals, *cloud_source);

    // Apply known transformation
    Transform gtTransform(0.2f, -0.1f, 0.05f, 0.1f, 0, 0.1f);

    auto cloud_target = util3d::transformPointCloud(cloud_source, gtTransform);

    bool hasConverged = false;
    pcl::PointCloud<pcl::PointNormal> aligned;
    Transform estimated = util3d::icpPointToPlane(
        cloud_source,
        cloud_target,
        0.2,   // maxCorrespondenceDistance
        50,    // iterations
        hasConverged,
        aligned,
        1e-6f, // epsilon
        false  // icp2D
    );

    std::cout << gtTransform << std::endl;
    std::cout << estimated << std::endl;

    EXPECT_TRUE(hasConverged);
    Eigen::Matrix4f estimatedMatrix = estimated.toEigen4f();
    Eigen::Matrix4f expectedMatrix = gtTransform.toEigen4f();
    EXPECT_TRUE(estimatedMatrix.isApprox(expectedMatrix, 1e-4));

    hasConverged = false;
    estimated = util3d::icpPointToPlane(
        cloud_source,
        cloud_target,
        0.2,   // maxCorrespondenceDistance
        50,    // iterations
        hasConverged,
        aligned,
        1e-6f, // epsilon
        true  // icp2D
    );

    // remove z and roll
    gtTransform = Transform(0.2f, -0.1f, 0, 0, 0, 0.1f);

    EXPECT_TRUE(hasConverged);
    estimatedMatrix = estimated.toEigen4f();
    expectedMatrix = gtTransform.toEigen4f();
    EXPECT_TRUE(estimatedMatrix.isApprox(expectedMatrix, 1e-4));
}