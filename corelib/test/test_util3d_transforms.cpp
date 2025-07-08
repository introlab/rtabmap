#include "gtest/gtest.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/UException.h"

using namespace rtabmap;

TEST(Util3DTransforms, transformLaserScanBasicXYZTransform)
{
    // Create a simple 3D laser scan with 3 points
    cv::Mat scanData(1, 3, CV_32FC3); // 3D points (X,Y,Z)
    scanData.at<cv::Vec3f>(0,0) = cv::Vec3f(1.0f, 0.0f, 0.0f);
    scanData.at<cv::Vec3f>(0,1) = cv::Vec3f(0.0f, 1.0f, 0.0f);
    scanData.at<cv::Vec3f>(0,2) = cv::Vec3f(0.0f, 0.0f, 1.0f);

    LaserScan scan(scanData, 0, 10.0f, LaserScan::kXYZ, Transform::getIdentity());

    // Create a transform: translate (1, 2, 3) and rotate 90 degrees about Z
    Transform transform(0, -1, 0, 1,
                        1,  0, 0, 2,
                        0,  0, 1, 3);

    // Apply transform
    LaserScan result = util3d::transformLaserScan(scan, transform);

    // Expected points after transformation
    std::vector<Eigen::Vector3f> expectedPoints = {
        Eigen::Vector3f(1.0f, 3.0f, 3.0f), // Rotated (1,0,0) + translated
        Eigen::Vector3f(0.0f, 2.0f, 3.0f), // Rotated (0,1,0) + translated
        Eigen::Vector3f(1.0f, 2.0f, 4.0f)  // Rotated (0,0,1) + translated
    };

    ASSERT_EQ(result.size(), 3);
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(result.field(i, 0), expectedPoints[i].x(), 1e-5f);
        EXPECT_NEAR(result.field(i, 1), expectedPoints[i].y(), 1e-5f);
        EXPECT_NEAR(result.field(i, 2), expectedPoints[i].z(), 1e-5f);
    }
}

TEST(Util3DTransforms, transformLaserScanXYZNormalTransform)
{
    // One point: position (1, 0, 0), normal (0, 1, 0)
    cv::Mat scanData(1, 1, CV_32FC(6)); // XYZ + normal_x, normal_y, normal_z
    float* ptr = scanData.ptr<float>(0, 0);
    ptr[0] = 1.0f; // x
    ptr[1] = 0.0f; // y
    ptr[2] = 0.0f; // z
    ptr[3] = 0.0f; // normal_x
    ptr[4] = 1.0f; // normal_y
    ptr[5] = 0.0f; // normal_z

    LaserScan scan(scanData, 0, 10.0f, LaserScan::kXYZNormal, Transform::getIdentity());

    // Transform: rotate 90 deg around Z axis + translate (0, 1, 0)
    Transform transform(0, -1, 0, 1,
                        1,  0, 0, 2,
                        0,  0, 1, 3);

    // Apply transform
    LaserScan result = util3d::transformLaserScan(scan, transform);

    ASSERT_EQ(result.size(), 1);
    const float* out = result.data().ptr<float>(0, 0);

    // Expected:
    // Point (1,0,0) → rotated to (0,1,0) + translated → (1,2,3)
    // Normal (0,1,0) → rotated to (-1,0,0) (normals are rotated, not translated)
    EXPECT_NEAR(out[0], 1.0f, 1e-5);  // x
    EXPECT_NEAR(out[1], 3.0f, 1e-5);  // y
    EXPECT_NEAR(out[2], 3.0f, 1e-5);  // z

    EXPECT_NEAR(out[3], -1.0f, 1e-5); // normal_x
    EXPECT_NEAR(out[4],  0.0f, 1e-5); // normal_y
    EXPECT_NEAR(out[5],  0.0f, 1e-5); // normal_z
}

TEST(Util3DTransforms, transformPointCloudXYZ)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ pt(1.0f, 2.0f, 3.0f);
    cloud->push_back(pt);

    Transform tf(1, 0, 0, 1,
                 0, 1, 0, 2,
                 0, 0, 1, 3); // Translate (1,2,3)

    auto result = util3d::transformPointCloud(cloud, tf);

    ASSERT_EQ(result->size(), 1);
    EXPECT_FLOAT_EQ(result->at(0).x, 2.0f);
    EXPECT_FLOAT_EQ(result->at(0).y, 4.0f);
    EXPECT_FLOAT_EQ(result->at(0).z, 6.0f);

    // test indices
    pcl::PointXYZ pt2(4.0f, 5.0f, 6.0f);
    cloud->push_back(pt2);
    pcl::IndicesPtr indices(new pcl::Indices(1,1)); // will transform only second point
    result = util3d::transformPointCloud(cloud, indices, tf);
    ASSERT_EQ(result->size(), 1);
    EXPECT_FLOAT_EQ(result->at(0).x, 5.0f);
    EXPECT_FLOAT_EQ(result->at(0).y, 7.0f);
    EXPECT_FLOAT_EQ(result->at(0).z, 9.0f);
}

TEST(Util3DTransforms, transformPointCloudXYZI)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI pt;
    pt.x = 1; pt.y = 1; pt.z = 1; pt.intensity = 42.0f;
    cloud->push_back(pt);

    Transform tf(1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 1); // Translate (0,0,1)

    auto result = util3d::transformPointCloud(cloud, tf);
    ASSERT_EQ(result->size(), 1);
    EXPECT_FLOAT_EQ(result->at(0).z, 2.0f);
    EXPECT_FLOAT_EQ(result->at(0).intensity, 42.0f);
    
    // test indices
    pt.intensity = 52.0f;
    cloud->push_back(pt);
    pcl::IndicesPtr indices(new pcl::Indices(1,1)); // will transform only second point
    result = util3d::transformPointCloud(cloud, indices, tf);
    ASSERT_EQ(result->size(), 1);
    EXPECT_FLOAT_EQ(result->at(0).intensity, 52.0f);
}

TEST(Util3DTransforms, transformPointCloudXYZRGB)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB pt;
    pt.x = 0; pt.y = 0; pt.z = 0;
    pt.r = 255; pt.g = 100; pt.b = 50;
    cloud->push_back(pt);

    Transform tf(1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0);

    auto result = util3d::transformPointCloud(cloud, tf);
    ASSERT_EQ(result->size(), 1);
    EXPECT_EQ(result->at(0).r, 255);
    EXPECT_EQ(result->at(0).g, 100);
    EXPECT_EQ(result->at(0).b, 50);

    // test indices
    pt.r = 250; pt.g = 105; pt.b = 55;
    cloud->push_back(pt);
    pcl::IndicesPtr indices(new pcl::Indices(1,1)); // will transform only second point
    result = util3d::transformPointCloud(cloud, indices, tf);
    ASSERT_EQ(result->size(), 1);
    EXPECT_EQ(result->at(0).r, 250);
    EXPECT_EQ(result->at(0).g, 105);
    EXPECT_EQ(result->at(0).b, 55);
}

TEST(Util3DTransforms, transformPointCloudPointNormal)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointNormal pt;
    pt.x = 1; pt.y = 0; pt.z = 0;
    pt.normal_x = 0; pt.normal_y = 1; pt.normal_z = 0;
    cloud->push_back(pt);

    Transform tf(0, -1, 0, 0,
                 1,  0, 0, 0,
                 0,  0, 1, 0); // 90 deg rotation around Z

    auto result = util3d::transformPointCloud(cloud, tf);
    ASSERT_EQ(result->size(), 1);
    EXPECT_NEAR(result->at(0).x, 0.0f, 1e-6);
    EXPECT_NEAR(result->at(0).y, 1.0f, 1e-6);
    EXPECT_NEAR(result->at(0).normal_x, -1.0f, 1e-6);
    EXPECT_NEAR(result->at(0).normal_y, 0.0f, 1e-6);

    // test indices
    pt.x = 0; pt.y = 1; pt.z = 0;
    pt.normal_x = 1; pt.normal_y = 0; pt.normal_z = 0;
    cloud->push_back(pt);
    pcl::IndicesPtr indices(new pcl::Indices(1,1)); // will transform only second point
    result = util3d::transformPointCloud(cloud, indices, tf);
    ASSERT_EQ(result->size(), 1);
    EXPECT_NEAR(result->at(0).x, -1.0f, 1e-6);
    EXPECT_NEAR(result->at(0).y, 0.0f, 1e-6);
    EXPECT_NEAR(result->at(0).normal_x, 0.0f, 1e-6);
    EXPECT_NEAR(result->at(0).normal_y, 1.0f, 1e-6);
}

TEST(Util3DTransforms, transformPointCloudXYZINormal)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointXYZINormal pt;
    pt.x = 0; pt.y = 1; pt.z = 0; pt.intensity = 42.0f;
    pt.normal_x = 1; pt.normal_y = 0; pt.normal_z = 0;
    cloud->push_back(pt);

    Transform tf(0, -1, 0, 0,
                 1,  0, 0, 0,
                 0,  0, 1, 0);

    auto result = util3d::transformPointCloud(cloud, tf);
    ASSERT_EQ(result->size(), 1);
    EXPECT_FLOAT_EQ(result->at(0).intensity, 42.0f);
    EXPECT_NEAR(result->at(0).normal_x, 0.0f, 1e-6);
    EXPECT_NEAR(result->at(0).normal_y, 1.0f, 1e-6);

    // test indices
    pt.intensity = 52.0f;
    cloud->push_back(pt);
    pcl::IndicesPtr indices(new pcl::Indices(1,1)); // will transform only second point
    result = util3d::transformPointCloud(cloud, indices, tf);
    ASSERT_EQ(result->size(), 1);
    EXPECT_FLOAT_EQ(result->at(0).intensity, 52.0f);
}

TEST(Util3DTransforms, transformPointCloudXYZRGBNormal)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointXYZRGBNormal pt;
    pt.x = 0; pt.y = 1; pt.z = 0;
    pt.r = 10; pt.g = 20; pt.b = 30;
    pt.normal_x = 1; pt.normal_y = 0; pt.normal_z = 0;
    cloud->push_back(pt);

    Transform tf(0, -1, 0, 0,
                 1,  0, 0, 0,
                 0,  0, 1, 0);

    auto result = util3d::transformPointCloud(cloud, tf);
    ASSERT_EQ(result->size(), 1);
    EXPECT_EQ(result->at(0).r, 10);
    EXPECT_EQ(result->at(0).g, 20);
    EXPECT_EQ(result->at(0).b, 30);
    EXPECT_NEAR(result->at(0).normal_x, 0.0f, 1e-6);
    EXPECT_NEAR(result->at(0).normal_y, 1.0f, 1e-6);

    // test indices
    pt.r = 250; pt.g = 105; pt.b = 55;
    cloud->push_back(pt);
    pcl::IndicesPtr indices(new pcl::Indices(1,1)); // will transform only second point
    result = util3d::transformPointCloud(cloud, indices, tf);
    ASSERT_EQ(result->size(), 1);
    EXPECT_EQ(result->at(0).r, 250);
    EXPECT_EQ(result->at(0).g, 105);
    EXPECT_EQ(result->at(0).b, 55);
}

TEST(Util3DTransforms, transformCVPointF)
{
    cv::Point3f pt(1.0f, 2.0f, 3.0f);

    Transform tf(1, 0, 0, 1,
                 0, 1, 0, 2,
                 0, 0, 1, 3); // Translate (1,2,3)

    auto result = util3d::transformPoint(pt, tf);

    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_FLOAT_EQ(result.z, 6.0f);
}

TEST(Util3DTransforms, transformCVPointD)
{
    cv::Point3d pt(1.0f, 2.0f, 3.0f);

    Transform tf(1, 0, 0, 1,
                 0, 1, 0, 2,
                 0, 0, 1, 3); // Translate (1,2,3)

    auto result = util3d::transformPoint(pt, tf);

    EXPECT_FLOAT_EQ(result.x, 2.0);
    EXPECT_FLOAT_EQ(result.y, 4.0);
    EXPECT_FLOAT_EQ(result.z, 6.0);
}

TEST(Util3DTransforms, transformPointXYZ)
{
    pcl::PointXYZ pt(1.0f, 2.0f, 3.0f);

    Transform tf(1, 0, 0, 1,
                 0, 1, 0, 2,
                 0, 0, 1, 3); // Translate (1,2,3)

    auto result = util3d::transformPoint(pt, tf);

    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_FLOAT_EQ(result.z, 6.0f);
}

TEST(Util3DTransforms, transformPointXYZI)
{
    pcl::PointXYZI pt;
    pt.x = 1; pt.y = 1; pt.z = 1; pt.intensity = 42.0f;

    Transform tf(1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 1); // Translate (0,0,1)

    auto result = util3d::transformPoint(pt, tf);
    EXPECT_FLOAT_EQ(result.z, 2.0f);
    EXPECT_FLOAT_EQ(result.intensity, 42.0f);
}

TEST(Util3DTransforms, transformPointXYZRGB)
{
    pcl::PointXYZRGB pt;
    pt.x = 0; pt.y = 0; pt.z = 0;
    pt.r = 255; pt.g = 100; pt.b = 50;

    Transform tf(1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0);

    auto result = util3d::transformPoint(pt, tf);
    EXPECT_EQ(result.r, 255);
    EXPECT_EQ(result.g, 100);
    EXPECT_EQ(result.b, 50);
}

TEST(Util3DTransforms, transformPointPointNormal)
{
    pcl::PointNormal pt;
    pt.x = 1; pt.y = 0; pt.z = 0;
    pt.normal_x = 0; pt.normal_y = 1; pt.normal_z = 0;

    Transform tf(0, -1, 0, 0,
                 1,  0, 0, 0,
                 0,  0, 1, 0); // 90 deg rotation around Z

    auto result = util3d::transformPoint(pt, tf);
    EXPECT_NEAR(result.x, 0.0f, 1e-6);
    EXPECT_NEAR(result.y, 1.0f, 1e-6);
    EXPECT_NEAR(result.normal_x, -1.0f, 1e-6);
    EXPECT_NEAR(result.normal_y, 0.0f, 1e-6);
}

TEST(Util3DTransforms, transformPointXYZINormal)
{
    pcl::PointXYZINormal pt;
    pt.x = 0; pt.y = 1; pt.z = 0; pt.intensity = 42.0f;
    pt.normal_x = 1; pt.normal_y = 0; pt.normal_z = 0;

    Transform tf(0, -1, 0, 0,
                 1,  0, 0, 0,
                 0,  0, 1, 0);

    auto result = util3d::transformPoint(pt, tf);
    EXPECT_FLOAT_EQ(result.intensity, 42.0f);
    EXPECT_NEAR(result.normal_x, 0.0f, 1e-6);
    EXPECT_NEAR(result.normal_y, 1.0f, 1e-6);
}

TEST(Util3DTransforms, transformPointXYZRGBNormal)
{
    pcl::PointXYZRGBNormal pt;
    pt.x = 0; pt.y = 1; pt.z = 0;
    pt.r = 10; pt.g = 20; pt.b = 30;
    pt.normal_x = 1; pt.normal_y = 0; pt.normal_z = 0;

    Transform tf(0, -1, 0, 0,
                 1,  0, 0, 0,
                 0,  0, 1, 0);

    auto result = util3d::transformPoint(pt, tf);
    EXPECT_EQ(result.r, 10);
    EXPECT_EQ(result.g, 20);
    EXPECT_EQ(result.b, 30);
    EXPECT_NEAR(result.normal_x, 0.0f, 1e-6);
    EXPECT_NEAR(result.normal_y, 1.0f, 1e-6);
}