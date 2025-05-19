#include "gtest/gtest.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/utilite/UException.h"
#include "rtabmap/utilite/UConversion.h"

using namespace rtabmap;

TEST(Util3dFiltering, commonFilteringEmpty) {
    ASSERT_TRUE(util3d::commonFiltering(LaserScan(), 1).isEmpty()); 
}

TEST(Util3dFiltering, commonFilteringDownsample) {
    for(int i=0; i<6; ++i)
    {
        // Dense
        cv::Mat data = cv::Mat::zeros(1, i+1, CV_32FC3); 
        LaserScan scan(data, data.total(), 0, LaserScan::kXYZ);
        ASSERT_EQ(util3d::commonFiltering(scan, 0).size(), i+1);
        ASSERT_EQ(util3d::commonFiltering(scan, 1).size(), i+1);
        ASSERT_EQ(util3d::commonFiltering(scan, 2).size(), (i+1) <= 2 ? (i+1) : (i+1)/2);
        ASSERT_EQ(util3d::commonFiltering(scan, 3).size(), (i+1) <= 3 ? (i+1) : (i+1)/3);

        // Organized (cols>rows)
        data = cv::Mat::zeros(i+1, (i+1)*2+i, CV_32FC3); 
        SCOPED_TRACE(uFormat("Testing Organized dim %dx%d (i=%d)", i+1, (i+1)*2+i, i));
        scan = LaserScan(data, data.total(), 0, LaserScan::kXYZ);
        ASSERT_EQ(util3d::commonFiltering(scan, 0).size(), (i+1)*((i+1)*2+i));
        ASSERT_EQ(util3d::commonFiltering(scan, 1).size(), (i+1)*((i+1)*2+i));
        ASSERT_EQ(util3d::commonFiltering(scan, 2).size(), (i+1)*2+i <= 2 ? (i+1)*((i+1)*2+i) : (i+1)*(((i+1)*2+i)/2));
        ASSERT_EQ(util3d::commonFiltering(scan, 3).size(), (i+1)*2+i <= 3 ? (i+1)*((i+1)*2+i) : (i+1)*(((i+1)*2+i)/3));
        ASSERT_EQ(util3d::commonFiltering(scan, 2).data().rows, 1);
        ASSERT_EQ(util3d::commonFiltering(scan, 3).data().rows, 1);
        ASSERT_EQ(util3d::commonFiltering(scan, 2).data().cols, (i+1)*2+i <= 2 ? (i+1)*((i+1)*2+i) : (i+1)*(((i+1)*2+i)/2));
        ASSERT_EQ(util3d::commonFiltering(scan, 3).data().cols, (i+1)*2+i <= 3 ? (i+1)*((i+1)*2+i) : (i+1)*(((i+1)*2+i)/3));

        // Organized (cols<rows)
        data = cv::Mat::zeros((i+1)*2+i, (i+1), CV_32FC3); 
        SCOPED_TRACE(uFormat("Testing Organized dim %dx%d (i=%d)", (i+1)*2+i, (i+1), i));
        scan = LaserScan(data, data.total(), 0, LaserScan::kXYZ);
        ASSERT_EQ(util3d::commonFiltering(scan, 0).size(), (i+1)*((i+1)*2+i));
        ASSERT_EQ(util3d::commonFiltering(scan, 1).size(), (i+1)*((i+1)*2+i));
        ASSERT_EQ(util3d::commonFiltering(scan, 2).size(), (i+1)*2+i <= 2 ? (i+1)*((i+1)*2+i) : (i+1)*(((i+1)*2+i)/2));
        ASSERT_EQ(util3d::commonFiltering(scan, 3).size(), (i+1)*2+i <= 3 ? (i+1)*((i+1)*2+i) : (i+1)*(((i+1)*2+i)/3));
        ASSERT_EQ(util3d::commonFiltering(scan, 2).data().rows, (i+1)*2+i <= 2 ? (i+1)*2+i : 1);
        ASSERT_EQ(util3d::commonFiltering(scan, 3).data().rows, (i+1)*2+i <= 3 ? (i+1)*2+i : 1);
        ASSERT_EQ(util3d::commonFiltering(scan, 2).data().cols, (i+1)*2+i <= 2 ? (i+1) : (i+1)*(((i+1)*2+i)/2));
        ASSERT_EQ(util3d::commonFiltering(scan, 3).data().cols, (i+1)*2+i <= 3 ? (i+1) : (i+1)*(((i+1)*2+i)/3));
    }
}

TEST(Util3dFiltering, commonFilteringRange) {
    cv::Mat data = cv::Mat::zeros(1, 5, CV_32FC3);
    for(int i=0; i<(int)data.total(); ++i)
    {
        data.at<cv::Vec3f>(0, i)[0] = i-1; // -1, 0, 1, 2, 3
    }
    LaserScan scan(data, data.total(), 0, LaserScan::kXYZ);
    ASSERT_EQ(util3d::commonFiltering(scan, 1, 0.0f, 0.0f).size(), 5);  // rangeMin<=0 ignored
    ASSERT_EQ(util3d::commonFiltering(scan, 1, -0.1f, 0.0f).size(), 5); // rangeMin<=0 ignored
    ASSERT_EQ(util3d::commonFiltering(scan, 1, 0.5f, 0.0f).size(), 4);  // rangeMax<=0 ignored, -1 is 1 meter away
    ASSERT_EQ(util3d::commonFiltering(scan, 1, 0.5f, 4.0f).size(), 4);  // -1 is 1 meter away
    ASSERT_EQ(util3d::commonFiltering(scan, 1, 0.0f, 4.0f).size(), 5);  // rangeMin<=0 ignored
    ASSERT_EQ(util3d::commonFiltering(scan, 1, 0.0f, 2.5f).size(), 4);  // rangeMin<=0 ignored
    ASSERT_EQ(util3d::commonFiltering(scan, 1, 2.5f, 0.5f).size(), 0);
}

TEST(Util3dFiltering, commonFilteringVoxel) {
    cv::Mat data = cv::Mat::zeros(1, 5, CV_32FC3);
    data.at<cv::Vec3f>(0, 0)[0] = 0.1;
    data.at<cv::Vec3f>(0, 1)[0] = 0.11;
    data.at<cv::Vec3f>(0, 2)[0] = 0.5;
    data.at<cv::Vec3f>(0, 3)[0] = 0.51;
    data.at<cv::Vec3f>(0, 4)[0] = 3;
    LaserScan scan(data, data.total(), 0, LaserScan::kXYZ);
    ASSERT_EQ(util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.001).size(), 5);  // voxel very small
    ASSERT_EQ(util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.05).size(), 3);
    ASSERT_EQ(util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 1).size(), 2);
}

TEST(Util3dFiltering, commonFilteringNormal) {
    cv::Mat data = cv::Mat::zeros(1, 100, CV_32FC3);
    // make points all on same plane (0,1,0);
    for(int i=0; i<10; ++i) {
        for(int j=0; j<10; ++j) {
            data.at<cv::Vec3f>(0, i*10+j)[0] = i*0.05; // x
            data.at<cv::Vec3f>(0, i*10+j)[1] = i<5 ? -1 : 1; // y
            data.at<cv::Vec3f>(0, i*10+j)[2] = j*0.05; // z
        }
    }
    LaserScan scan(data, data.total(), 0, LaserScan::kXYZ);

    // NormalK
    LaserScan result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 10);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    int normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset+1), i<50 ? 1.0f : -1.0f, 1e-5); // ny
    }

    // Radius
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 0, 0.5f);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset+1), i<50 ? 1.0f : -1.0f, 1e-5); // ny
    }
}

TEST(Util3dFiltering, commonFilteringNormalRGB) {
    cv::Mat data = cv::Mat::zeros(1, 100, CV_32FC4);
    // make points all on same plane (0,1,0);
    for(int i=0; i<10; ++i) {
        for(int j=0; j<10; ++j) {
            data.at<cv::Vec4f>(0, i*10+j)[0] = i*0.05; // x
            data.at<cv::Vec4f>(0, i*10+j)[1] = i<5 ? -1 : 1; // y
            data.at<cv::Vec4f>(0, i*10+j)[2] = j*0.05; // z
            data.at<cv::Vec4f>(0, i*10+j)[3] = LaserScan::packRGB(255, 128, 64);
        }
    }
    LaserScan scan(data, data.total(), 0, LaserScan::kXYZRGB);

    // NormalK
    LaserScan result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 10);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasRGB());
    int normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset+1), i<50 ? 1.0f : -1.0f, 1e-5); // ny
        EXPECT_EQ(result.field(i, result.getRGBOffset()), LaserScan::packRGB(255, 128, 64));
    }

    // Radius
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 0, 0.5f);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasRGB());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset+1), i<50 ? 1.0f : -1.0f, 1e-5); // ny
        EXPECT_EQ(result.field(i, result.getRGBOffset()), LaserScan::packRGB(255, 128, 64));
    }
}

TEST(Util3dFiltering, commonFilteringNormalI) {
    cv::Mat data = cv::Mat::zeros(1, 100, CV_32FC4);
    // make points all on same plane (0,1,0);
    for(int i=0; i<10; ++i) {
        for(int j=0; j<10; ++j) {
            data.at<cv::Vec4f>(0, i*10+j)[0] = i*0.05; // x
            data.at<cv::Vec4f>(0, i*10+j)[1] = i<5 ? -1 : 1; // y
            data.at<cv::Vec4f>(0, i*10+j)[2] = j*0.05; // z
            data.at<cv::Vec4f>(0, i*10+j)[3] = 420;
        }
    }
    LaserScan scan(data, data.total(), 0, LaserScan::kXYZI);

    // NormalK
    LaserScan result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 10);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasIntensity());
    int normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset+1), i<50 ? 1.0f : -1.0f, 1e-5); // ny
        EXPECT_EQ(result.field(i, result.getIntensityOffset()), 420);
    }

    // Radius
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 0, 0.5f);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasIntensity());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset+1), i<50 ? 1.0f : -1.0f, 1e-5); // ny
        EXPECT_EQ(result.field(i, result.getIntensityOffset()), 420);
    }
}

TEST(Util3dFiltering, commonFilteringNormalVoxel) {
    cv::Mat data = cv::Mat::zeros(1, 100, CV_32FC(6));
    // make points all on same plane (0,1,0);
    for(int i=0; i<10; ++i) {
        for(int j=0; j<10; ++j) {
            data.at<cv::Vec6f>(0, i*10+j)[0] = i*0.05; // x
            data.at<cv::Vec6f>(0, i*10+j)[1] = i<5 ? -1 : 1; // y
            data.at<cv::Vec6f>(0, i*10+j)[2] = j*0.05; // z
            data.at<cv::Vec6f>(0, i*10+j)[3] = 1; // nx
        }
    }
    LaserScan scan(data, data.total(), 0, LaserScan::kXYZNormal);

    // NormalK
    LaserScan result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 10);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    int normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset), 1.0f, 1e-5); // nx
    }

    // Radius
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 0, 0.5f);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset), 1.0f, 1e-5); // nx
    }

    // combined with voxel filter, the normals should be recomputed
    // NormalK
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.1f, 5);
    EXPECT_EQ(result.size(), 30);
    EXPECT_TRUE(result.hasNormals());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        if(result.field(i, 1) < 0) { // y
            EXPECT_NEAR(result.field(i, normalOffset+1), 1, 1e-5); // ny
        }
        else {
            EXPECT_NEAR(result.field(i, normalOffset+1), -1, 1e-5); // ny
        }
    }

    // Radius
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.1f, 0, 0.25f);
    EXPECT_EQ(result.size(), 30);
    EXPECT_TRUE(result.hasNormals());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        if(result.field(i, 1) < 0) { // y
            EXPECT_NEAR(result.field(i, normalOffset+1), 1, 1e-5); // ny
        }
        else {
            EXPECT_NEAR(result.field(i, normalOffset+1), -1, 1e-5); // ny
        }
    }
}

TEST(Util3dFiltering, commonFilteringNormalVoxelRGB) {
    cv::Mat data = cv::Mat::zeros(1, 100, CV_32FC(7));
    // make points all on same plane (0,1,0);
    for(int i=0; i<10; ++i) {
        for(int j=0; j<10; ++j) {
            float * ptr = data.ptr<float>(0, i*10+j);
            ptr[0] = i*0.05; // x
            ptr[1] = i<5 ? -1 : 1; // y
            ptr[2] = j*0.05; // z
            ptr[3] = LaserScan::packRGB(255,128,64); // rgb
            ptr[4] = 1; // nx
        }
    }
    LaserScan scan(data, data.total(), 0, LaserScan::kXYZRGBNormal);

    // NormalK
    LaserScan result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 10);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasRGB());
    int normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset), 1.0f, 1e-5); // nx
        EXPECT_EQ(result.field(i, result.getRGBOffset()), LaserScan::packRGB(255,128,64)); //rgb
    }

    // Radius
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 0, 0.5f);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasRGB());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset), 1.0f, 1e-5); // nx
        EXPECT_EQ(result.field(i, result.getRGBOffset()), LaserScan::packRGB(255,128,64)); //rgb
    }

    // combined with voxel filter, the normals should be recomputed
    // NormalK
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.1f, 5);
    EXPECT_EQ(result.size(), 30);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasRGB());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        if(result.field(i, 1) < 0) { // y
            EXPECT_NEAR(result.field(i, normalOffset+1), 1, 1e-5); // ny
        }
        else {
            EXPECT_NEAR(result.field(i, normalOffset+1), -1, 1e-5); // ny
        }
        EXPECT_EQ(result.field(i, result.getRGBOffset()), LaserScan::packRGB(255,128,64)); //rgb
    }

    // Radius
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.1f, 0, 0.25f);
    EXPECT_EQ(result.size(), 30);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasRGB());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        if(result.field(i, 1) < 0) { // y
            EXPECT_NEAR(result.field(i, normalOffset+1), 1, 1e-5); // ny
        }
        else {
            EXPECT_NEAR(result.field(i, normalOffset+1), -1, 1e-5); // ny
        }
        EXPECT_EQ(result.field(i, result.getRGBOffset()), LaserScan::packRGB(255,128,64)); //rgb
    }
}

TEST(Util3dFiltering, commonFilteringNormalVoxelI) {
    cv::Mat data = cv::Mat::zeros(1, 100, CV_32FC(7));
    // make points all on same plane (0,1,0);
    for(int i=0; i<10; ++i) {
        for(int j=0; j<10; ++j) {
            float * ptr = data.ptr<float>(0, i*10+j);
            ptr[0] = i*0.05; // x
            ptr[1] = i<5 ? -1 : 1; // y
            ptr[2] = j*0.05; // z
            ptr[3] = 420; // intensity
            ptr[4] = 1; // nx
        }
    }
    LaserScan scan(data, data.total(), 0, LaserScan::kXYZINormal);

    // NormalK
    LaserScan result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 10);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasIntensity());
    int normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset), 1.0f, 1e-5); // nx
        EXPECT_EQ(result.field(i, result.getIntensityOffset()), 420); //intensity
    }

    // Radius
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 0, 0.5f);
    EXPECT_EQ(result.size(), 100);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasIntensity());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        EXPECT_NEAR(result.field(i, normalOffset), 1.0f, 1e-5); // nx
        EXPECT_EQ(result.field(i, result.getIntensityOffset()), 420); //intensity
    }

    // combined with voxel filter, the normals should be recomputed
    // NormalK
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.1f, 5);
    EXPECT_EQ(result.size(), 30);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasIntensity());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        if(result.field(i, 1) < 0) { // y
            EXPECT_NEAR(result.field(i, normalOffset+1), 1, 1e-5); // ny
        }
        else {
            EXPECT_NEAR(result.field(i, normalOffset+1), -1, 1e-5); // ny
        }
        EXPECT_EQ(result.field(i, result.getIntensityOffset()), 420); //intensity
    }

    // Radius
    result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.1f, 0, 0.25f);
    EXPECT_EQ(result.size(), 30);
    EXPECT_TRUE(result.hasNormals());
    EXPECT_TRUE(result.hasIntensity());
    normalOffset = result.getNormalsOffset();
    for(int i=0; i<result.size(); ++i) {
        if(result.field(i, 1) < 0) { // y
            EXPECT_NEAR(result.field(i, normalOffset+1), 1, 1e-5); // ny
        }
        else {
            EXPECT_NEAR(result.field(i, normalOffset+1), -1, 1e-5); // ny
        }
        EXPECT_EQ(result.field(i, result.getIntensityOffset()), 420); //intensity
    }
}

TEST(Util3dFiltering, commonFilteringGroundNormalsUp) {
    cv::Mat data = cv::Mat::zeros(1, 6, CV_32FC(6));
    data.at<cv::Vec6f>(0,0)[5] = 1; // nz
    
    data.at<cv::Vec6f>(0,1)[5] = -1; // nz

    data.at<cv::Vec6f>(0,2)[2] = 15; // z
    data.at<cv::Vec6f>(0,2)[5] = 1; // nz

    data.at<cv::Vec6f>(0,3)[2] = 15; // z
    data.at<cv::Vec6f>(0,3)[5] = -1; // nz

    data.at<cv::Vec6f>(0,4)[0] = 5; // x
    data.at<cv::Vec6f>(0,4)[2] = 5; // z
    data.at<cv::Vec6f>(0,4)[3] = -cos(M_PI/4); // nx
    data.at<cv::Vec6f>(0,4)[5] = sin(M_PI/4); // nz (~0.707)

    data.at<cv::Vec6f>(0,5)[0] = 5; // x
    data.at<cv::Vec6f>(0,5)[2] = 5; // z
    data.at<cv::Vec6f>(0,5)[3] = -cos(M_PI/4); // nx
    data.at<cv::Vec6f>(0,5)[5] = -sin(M_PI/4); // nz (-0.707)
    LaserScan scan(data, data.total(), 0, LaserScan::kXYZNormal, Transform(0,0,10,0,0,0));

    LaserScan result = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.8f);
    EXPECT_EQ(result.size(), 6);
    EXPECT_TRUE(result.hasNormals());
    int nz = result.getNormalsOffset()+2;
    EXPECT_EQ(result.field(0, nz), 1.0f);
    EXPECT_EQ(result.field(1, nz), 1.0f);
    EXPECT_EQ(result.field(2, nz), -1.0f);
    EXPECT_EQ(result.field(3, nz), -1.0f);
    EXPECT_FLOAT_EQ(result.field(4, nz), sin(M_PI/4));
    EXPECT_FLOAT_EQ(result.field(5, nz), -sin(M_PI/4));
}

TEST(Util3dFiltering, rangeFilteringNoFilteringAppliedWhenEmpty)
{
    LaserScan emptyScan; // Assuming default constructor gives empty scan
    LaserScan result = util3d::rangeFiltering(emptyScan, 1.0f, 5.0f);
    EXPECT_TRUE(result.isEmpty());
}

TEST(Util3dFiltering, rangeFilteringNoFilteringWhenMinMaxZero)
{
    // Simulate a simple 2D scan with 3 points: (1,0), (3,0), (6,0)
    cv::Mat data = (cv::Mat_<float>(1, 3*2) << 1.0f, 0.0f, 3.0f, 0.0f, 6.0f, 0.0f);
    data = data.reshape(2, 1); // Reshape to 1 row, 3 columns with 2 floats per point

    LaserScan scan(data, 0, 0, LaserScan::kXY); // assuming kXY means 2D points
    LaserScan result = util3d::rangeFiltering(scan, 0.0f, 0.0f);

    EXPECT_EQ(result.size(), scan.size());
    EXPECT_EQ(result.data().at<cv::Vec2f>(0,0), scan.data().at<cv::Vec2f>(0,0));
    EXPECT_EQ(result.data().at<cv::Vec2f>(0,1), scan.data().at<cv::Vec2f>(0,1));
    EXPECT_EQ(result.data().at<cv::Vec2f>(0,2), scan.data().at<cv::Vec2f>(0,2));
}

TEST(Util3dFiltering, rangeFilteringFilteringRemovesOutOfRangePoints)
{
    // 2D points: (1,0) [1m], (3,0) [3m], (6,0) [6m]
    cv::Mat data = (cv::Mat_<float>(1, 3*2) << 1.0f, 0.0f, 3.0f, 0.0f, 6.0f, 0.0f);
    data = data.reshape(2, 1);

    LaserScan scan(data, 0, 0, LaserScan::kXY);
    LaserScan result = util3d::rangeFiltering(scan, 2.0f, 5.0f);

    ASSERT_EQ(result.size(), 1); // Only the (3,0) point should remain
    EXPECT_FLOAT_EQ(result.data().at<cv::Vec3f>(0)[0], 3.0f); // x
    EXPECT_FLOAT_EQ(result.data().at<cv::Vec3f>(0)[1], 0.0f); // y

    // 3D points: (1,0) [1m], (3,0) [3m], (6,0) [6m]
    data = (cv::Mat_<float>(1, 3*3) << 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 6.0f);
    data = data.reshape(3, 1);

    scan = LaserScan(data, 0, 0, LaserScan::kXYZ);
    result = util3d::rangeFiltering(scan, 2.0f, 5.0f);

    ASSERT_EQ(result.size(), 1); // Only the (0,0,3) point should remain
    EXPECT_FLOAT_EQ(result.data().at<cv::Vec3f>(0)[0], 0.0f); // x
    EXPECT_FLOAT_EQ(result.data().at<cv::Vec3f>(0)[1], 0.0f); // y
    EXPECT_FLOAT_EQ(result.data().at<cv::Vec3f>(0)[2], 3.0f); // z
}

TEST(Util3dFiltering, rangeFilteringFilteringWithOnlyMinOrMax)
{
    // 2D points: (1,0) [1m], (4,0) [4m]
    cv::Mat data = (cv::Mat_<float>(1, 2*2) << 1.0f, 0.0f, 4.0f, 0.0f);
    data = data.reshape(2, 1);

    LaserScan scan(data, 0, 0, LaserScan::kXY);

    // Only apply minimum range
    LaserScan resultMin = util3d::rangeFiltering(scan, 2.0f, 0.0f);
    ASSERT_EQ(resultMin.size(), 1);
    EXPECT_FLOAT_EQ(resultMin.data().at<float>(0, 0), 4.0f);

    // Only apply maximum range
    LaserScan resultMax = util3d::rangeFiltering(scan, 0.0f, 2.5f);
    ASSERT_EQ(resultMax.size(), 1);
    EXPECT_FLOAT_EQ(resultMax.data().at<float>(0, 0), 1.0f);
}

TEST(Util3dFiltering, rangeFilteringPointXYZ) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(1, 0, 0));   // Distance: 1
    cloud->push_back(pcl::PointXYZ(3, 0, 0));   // Distance: 3
    cloud->push_back(pcl::PointXYZ(6, 0, 0));   // Distance: 6

    pcl::IndicesPtr indices(new std::vector<int>); // Empty means use whole cloud

    auto filtered = util3d::rangeFiltering(cloud, indices, 2.0f, 5.0f);
    ASSERT_EQ(filtered->size(), 1);
    EXPECT_EQ(filtered->at(0), 1); // Only point at index 1 is within range
}

TEST(Util3dFiltering, rangeFilteringPointXYZRGB) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->push_back(pcl::PointXYZRGB()); cloud->back().x = 2; cloud->back().y = 0; cloud->back().z = 0; // Distance: 2
    cloud->push_back(pcl::PointXYZRGB()); cloud->back().x = 4; cloud->back().y = 0; cloud->back().z = 0; // Distance: 4

    pcl::IndicesPtr indices(new std::vector<int>);
    indices->push_back(0);
    indices->push_back(1);

    auto filtered = util3d::rangeFiltering(cloud, indices, 0.0f, 3.0f);
    ASSERT_EQ(filtered->size(), 1);
    EXPECT_EQ(filtered->at(0), 0);
}

TEST(Util3dFiltering, rangeFilteringPointNormal) {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    cloud->push_back(pcl::PointNormal()); cloud->back().x = 1.0f; // Distance: 1
    cloud->push_back(pcl::PointNormal()); cloud->back().x = 10.0f; // Distance: 10

    pcl::IndicesPtr indices(new std::vector<int>);
    auto filtered = util3d::rangeFiltering(cloud, indices, 0.0f, 5.0f);
    ASSERT_EQ(filtered->size(), 1);
    EXPECT_EQ(filtered->at(0), 0);
}

TEST(Util3dFiltering, rangeFilteringPointXYZRGBNormal) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointXYZRGBNormal pt1; pt1.x = 0.5f; pt1.y = 0; pt1.z = 0;  // Distance: 0.5
    pcl::PointXYZRGBNormal pt2; pt2.x = 2.5f; pt2.y = 0; pt2.z = 0;  // Distance: 2.5
    cloud->push_back(pt1);
    cloud->push_back(pt2);

    pcl::IndicesPtr indices(new std::vector<int>); // All points
    auto filtered = util3d::rangeFiltering(cloud, indices, 1.0f, 3.0f);
    ASSERT_EQ(filtered->size(), 1);
    EXPECT_EQ(filtered->at(0), 1);
}

TEST(Util3dFiltering, rangeSplitFilteringPointXYZ)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(1, 0, 0)); // dist = 1
    cloud->push_back(pcl::PointXYZ(4, 0, 0)); // dist = 4
    cloud->push_back(pcl::PointXYZ(2, 0, 0)); // dist = 2

    pcl::IndicesPtr indices(new std::vector<int>); // Empty = use all
    pcl::IndicesPtr closeIndices, farIndices;

    util3d::rangeSplitFiltering(cloud, indices, 3.0f, closeIndices, farIndices);

    EXPECT_EQ(closeIndices->size(), 2);
    EXPECT_EQ(closeIndices->at(0), 0);
    EXPECT_EQ(closeIndices->at(1), 2);
    EXPECT_EQ(farIndices->size(), 1);
    EXPECT_EQ(farIndices->at(0), 1);

    indices->push_back(1);
    indices->push_back(2);

    util3d::rangeSplitFiltering(cloud, indices, 3.0f, closeIndices, farIndices);

    EXPECT_EQ(closeIndices->size(), 1);
    EXPECT_EQ(closeIndices->at(0), 2);
    EXPECT_EQ(farIndices->size(), 1);
    EXPECT_EQ(farIndices->at(0), 1);
}

TEST(Util3dFiltering, rangeSplitFilteringPointXYZRGB)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB pt1, pt2;
    pt1.x = 1; pt1.y = 0; pt1.z = 0; pt1.r = 255;
    pt2.x = 5; pt2.y = 0; pt2.z = 0; pt2.g = 255;
    cloud->push_back(pt1);
    cloud->push_back(pt2);

    pcl::IndicesPtr indices(new std::vector<int>); // All points
    pcl::IndicesPtr closeIndices, farIndices;

    util3d::rangeSplitFiltering(cloud, indices, 3.0f, closeIndices, farIndices);

    EXPECT_EQ(closeIndices->size(), 1);
    EXPECT_EQ(closeIndices->at(0), 0);
    EXPECT_EQ(farIndices->size(), 1);
    EXPECT_EQ(farIndices->at(0), 1);

    indices->push_back(1);

    util3d::rangeSplitFiltering(cloud, indices, 3.0f, closeIndices, farIndices);

    EXPECT_EQ(closeIndices->size(), 0);
    EXPECT_EQ(farIndices->size(), 1);
    EXPECT_EQ(farIndices->at(0), 1);
}

TEST(Util3dFiltering, rangeSplitFilteringPointNormal)
{
    auto cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointNormal pt1, pt2;
    pt1.x = 0.5f; pt1.y = 0; pt1.z = 0;
    pt2.x = 3.5f; pt2.y = 0; pt2.z = 0;
    cloud->push_back(pt1);
    cloud->push_back(pt2);

    pcl::IndicesPtr indices(new std::vector<int>); // Use full cloud
    pcl::IndicesPtr closeIndices, farIndices;

    util3d::rangeSplitFiltering(cloud, indices, 2.0f, closeIndices, farIndices);

    EXPECT_EQ(closeIndices->size(), 1);
    EXPECT_EQ(closeIndices->at(0), 0);
    EXPECT_EQ(farIndices->size(), 1);
    EXPECT_EQ(farIndices->at(0), 1);

    indices->push_back(1);

    util3d::rangeSplitFiltering(cloud, indices, 2.0f, closeIndices, farIndices);

    EXPECT_EQ(closeIndices->size(), 0);
    EXPECT_EQ(farIndices->size(), 1);
    EXPECT_EQ(farIndices->at(0), 1);
}

TEST(Util3dFiltering, rangeSplitFilteringPointXYZRGBNormal)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointXYZRGBNormal pt1, pt2;
    pt1.x = 1.0f; pt1.y = 0; pt1.z = 0;
    pt2.x = 4.0f; pt2.y = 0; pt2.z = 0;
    cloud->push_back(pt1);
    cloud->push_back(pt2);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::IndicesPtr closeIndices, farIndices;

    util3d::rangeSplitFiltering(cloud, indices, 3.0f, closeIndices, farIndices);

    EXPECT_EQ(closeIndices->size(), 1);
    EXPECT_EQ(closeIndices->at(0), 0);
    EXPECT_EQ(farIndices->size(), 1);
    EXPECT_EQ(farIndices->at(0), 1);

    indices->push_back(1);

    util3d::rangeSplitFiltering(cloud, indices, 3.0f, closeIndices, farIndices);

    EXPECT_EQ(closeIndices->size(), 0);
    EXPECT_EQ(farIndices->size(), 1);
    EXPECT_EQ(farIndices->at(0), 1);
}

TEST(Util3dFiltering, downsampleImplUnorganizedCloud) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // Create 10 points along X axis
    for (int i = 0; i < 10; ++i) {
        cloud->push_back(pcl::PointXYZ(float(i), 0, 0));
    }

    auto result = util3d::downsample(cloud, 3);

    ASSERT_EQ(result->size(), 3);
    EXPECT_FLOAT_EQ(result->at(0).x, 0);
    EXPECT_FLOAT_EQ(result->at(1).x, 3);
    EXPECT_FLOAT_EQ(result->at(2).x, 6);

    //LaserScan version
    {
        // 2D scan
        cv::Mat data(1, 10, CV_32FC2);

        // Create 10 points along X axis
        for (int i = 0; i < 10; ++i) {
            data.at<cv::Vec2f>(i)[0] = float(i); // x
        }
        auto scan = LaserScan(data, LaserScan::kXY, 0.0f, 40.0f, -M_PI/2.0f, M_PI/2.0f, M_PI/10.0f);
        auto result = util3d::downsample(scan, 3);

        ASSERT_EQ(result.data().cols, 3);
        ASSERT_EQ(result.angleIncrement(), scan.angleIncrement()*3);
        EXPECT_FLOAT_EQ(result.field(0, 0), 0);
        EXPECT_FLOAT_EQ(result.field(1, 0), 3);
        EXPECT_FLOAT_EQ(result.field(2, 0), 6);
    }
}

TEST(Util3dFiltering, downsampleImplStepLargerThanCloudSize) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < 5; ++i)
        cloud->push_back(pcl::PointXYZ(float(i), 0, 0));

    auto result = util3d::downsample(cloud, 10);
    ASSERT_EQ(result->size(), cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i)
        EXPECT_EQ(result->at(i).x, cloud->at(i).x);

    //LaserScan version
    {
        cv::Mat data(1, 10, CV_32FC3);
        for (int i = 0; i < 5; ++i)
            data.at<cv::Vec3f>(i)[0] = float(i); // x
        
        auto scan = LaserScan(data, 0, 0, LaserScan::kXYZ);
        auto result = util3d::downsample(scan, 10);
        ASSERT_EQ(result.size(), scan.size());
        for (int i = 0; i < scan.size(); ++i)
            EXPECT_EQ(result.field(i,0), scan.field(i,0));
    }
}

TEST(Util3dFiltering, downsampleImplStepOne) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(1, 2, 3));
    auto result = util3d::downsample(cloud, 1);
    ASSERT_EQ(result->size(), cloud->size());
    EXPECT_EQ(result->at(0).x, 1);
    EXPECT_EQ(result->at(0).y, 2);
    EXPECT_EQ(result->at(0).z, 3);

    //LaserScan version
    {
        cv::Mat data = (cv::Mat_<float>(1, 3) << 1.0f, 2.0f, 3.0f);
        data = data.reshape(3, 1);
        auto scan = LaserScan(data, 0, 0, LaserScan::kXYZ);
        auto result = util3d::downsample(scan, 1);
        ASSERT_EQ(result.size(), scan.size());
        EXPECT_EQ(result.field(0,0), 1);
        EXPECT_EQ(result.field(0,1), 2);
        EXPECT_EQ(result.field(0,2), 3);
    }
}

TEST(Util3dFiltering, downsampleImplOrganizedDepthImage) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 4;
    cloud->height = 4;
    cloud->is_dense = false;
    cloud->resize(cloud->width * cloud->height);

    // Fill organized cloud with row-major increasing values
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            cloud->at(j * 4 + i) = pcl::PointXYZ(float(i + j * 4), 0, 0);
        }
    }

    auto result = util3d::downsample(cloud, 2);

    ASSERT_EQ(result->width, 2);
    ASSERT_EQ(result->height, 2);
    ASSERT_EQ(result->size(), 4);

    EXPECT_EQ(result->at(0).x, cloud->at(0).x);   // (0,0)
    EXPECT_EQ(result->at(1).x, cloud->at(2).x);   // (0,2)
    EXPECT_EQ(result->at(2).x, cloud->at(8).x);   // (2,0)
    EXPECT_EQ(result->at(3).x, cloud->at(10).x);  // (2,2)

    //LaserScan version
    {
        cv::Mat data(4, 4, CV_32FC3);

        // Fill organized cloud with row-major increasing values
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 4; ++i) {
                data.at<cv::Vec3f>(j * 4 + i)[0] = float(i + j * 4);
            }
        }

        auto scan = LaserScan(data, data.total(), 0, LaserScan::kXYZ);
        auto result = util3d::downsample(scan, 2);

        ASSERT_EQ(result.data().cols, 2);
        ASSERT_EQ(result.data().rows, 2);
        ASSERT_EQ(result.size(), 4);

        EXPECT_EQ(result.field(0,0), scan.field(0,0));   // (0,0)
        EXPECT_EQ(result.field(1,0), scan.field(2,0));   // (0,2)
        EXPECT_EQ(result.field(2,0), scan.field(8,0));   // (2,0)
        EXPECT_EQ(result.field(3,0), scan.field(10,0));  // (2,2)
    }
}

TEST(Util3dFiltering, downsampleImplInvalidStep) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(0, 0, 0));
    EXPECT_THROW(util3d::downsample(cloud, 0), UException);

    // just checking if other point type functions are there, 
    // note that they all call the exact same implemented function 
    // under the hood, so we son't duplicate all tests already 
    // done with pcl::PointXYZ type.
    EXPECT_THROW(util3d::downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(), 0), UException);
    EXPECT_THROW(util3d::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr(), 0), UException);
    EXPECT_THROW(util3d::downsample(pcl::PointCloud<pcl::PointNormal>::Ptr(), 0), UException);
    EXPECT_THROW(util3d::downsample(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(), 0), UException);
    EXPECT_THROW(util3d::downsample(pcl::PointCloud<pcl::PointXYZINormal>::Ptr(), 0), UException);

    // LaserScan version
    EXPECT_THROW(util3d::downsample(LaserScan(), 0), UException);
}

TEST(Util3dFiltering, downsampleImplLiDARStyleRingsInRows) {
    const int width = 8;
    const int height = 2; // 2 rings, 8 points per ring
    const int step = 2;

    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = true;
    cloud->resize(width * height);

    // Fill with values (row-major): pt(i,j) = i + j*width
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            cloud->at(j * width + i) = pcl::PointXYZ(float(i + j * width), 0, 0);
        }
    }

    auto result = util3d::downsample(cloud, step);

    EXPECT_EQ(result->width, width / step);
    EXPECT_EQ(result->height, height);
    EXPECT_EQ(result->size(), (width / step) * height);

    // Expect values at (0, 2, 4, 6) for each row
    EXPECT_FLOAT_EQ(result->at(0).x, 0);  // row 0, col 0
    EXPECT_FLOAT_EQ(result->at(1).x, 2);  // row 0, col 2
    EXPECT_FLOAT_EQ(result->at(2).x, 4);  // row 0, col 4
    EXPECT_FLOAT_EQ(result->at(3).x, 6);  // row 0, col 6
    EXPECT_FLOAT_EQ(result->at(4).x, 8);  // row 1, col 0
    EXPECT_FLOAT_EQ(result->at(5).x, 10); // row 1, col 2
    EXPECT_FLOAT_EQ(result->at(6).x, 12); // row 1, col 4
    EXPECT_FLOAT_EQ(result->at(7).x, 14); // row 1, col 6

    // LaserScan version
    {
        cv::Mat data(height, width, CV_32FC3);

        // Fill with values (row-major): pt(i,j) = i + j*width
        for (int j = 0; j < height; ++j) {
            for (int i = 0; i < width; ++i) {
                data.at<cv::Vec3f>(j * width + i)[0] = float(i + j * width);
            }
        }

        auto scan = LaserScan(data, data.total(), 0, LaserScan::kXYZ);
        auto result = util3d::downsample(scan, step);

        EXPECT_EQ(result.data().cols, width / step);
        EXPECT_EQ(result.data().rows, height);
        EXPECT_EQ(result.size(), (width / step) * height);

        // Expect values at (0, 2, 4, 6) for each row
        EXPECT_FLOAT_EQ(result.field(0,0), 0);  // row 0, col 0
        EXPECT_FLOAT_EQ(result.field(1,0), 2);  // row 0, col 2
        EXPECT_FLOAT_EQ(result.field(2,0), 4);  // row 0, col 4
        EXPECT_FLOAT_EQ(result.field(3,0), 6);  // row 0, col 6
        EXPECT_FLOAT_EQ(result.field(4,0), 8);  // row 1, col 0
        EXPECT_FLOAT_EQ(result.field(5,0), 10); // row 1, col 2
        EXPECT_FLOAT_EQ(result.field(6,0), 12); // row 1, col 4
        EXPECT_FLOAT_EQ(result.field(7,0), 14); // row 1, col 6
    }
}

TEST(Util3dFiltering, downsampleImplLiDARStyleRingsInColumns) {
    const int width = 2;  // 2 columns = 2 rings
    const int height = 8; // 8 points per ring
    const int step = 2;

    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = true;
    cloud->resize(width * height);

    // Fill with values (row-major): pt(i,j) = i + j*width
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            cloud->at(j * width + i) = pcl::PointXYZ(float(i + j * width), 0, 0);
        }
    }

    auto result = util3d::downsample(cloud, step);

    EXPECT_EQ(result->width, width);
    EXPECT_EQ(result->height, height / step);
    EXPECT_EQ(result->size(), (height / step) * width);

    // Expect values at rows 0,2,4,6 for each column
    EXPECT_FLOAT_EQ(result->at(0).x, 0);  // row 0, col 0
    EXPECT_FLOAT_EQ(result->at(1).x, 1);  // row 0, col 1
    EXPECT_FLOAT_EQ(result->at(2).x, 4);  // row 2, col 0
    EXPECT_FLOAT_EQ(result->at(3).x, 5);  // row 2, col 1
    EXPECT_FLOAT_EQ(result->at(4).x, 8);  // row 4, col 0
    EXPECT_FLOAT_EQ(result->at(5).x, 9);  // row 4, col 1
    EXPECT_FLOAT_EQ(result->at(6).x, 12); // row 6, col 0
    EXPECT_FLOAT_EQ(result->at(7).x, 13); // row 6, col 1

    // LaserScan version
    {
        cv::Mat data(height, width, CV_32FC3);

        // Fill with values (row-major): pt(i,j) = i + j*width
        for (int j = 0; j < height; ++j) {
            for (int i = 0; i < width; ++i) {
                data.at<cv::Vec3f>(j * width + i)[0] = float(i + j * width);
            }
        }

        auto scan = LaserScan(data, data.total(), 0, LaserScan::kXYZ);
        auto result = util3d::downsample(scan, step);

        EXPECT_EQ(result.data().cols, width);
        EXPECT_EQ(result.data().rows, height/step);
        EXPECT_EQ(result.size(), (width / step) * height);

        // Expect values at rows 0,2,4,6 for each column
        EXPECT_FLOAT_EQ(result.field(0,0), 0);  // row 0, col 0
        EXPECT_FLOAT_EQ(result.field(1,0), 1);  // row 0, col 1
        EXPECT_FLOAT_EQ(result.field(2,0), 4);  // row 2, col 0
        EXPECT_FLOAT_EQ(result.field(3,0), 5);  // row 2, col 1
        EXPECT_FLOAT_EQ(result.field(4,0), 8);  // row 4, col 0
        EXPECT_FLOAT_EQ(result.field(5,0), 9);  // row 4, col 1
        EXPECT_FLOAT_EQ(result.field(6,0), 12); // row 6, col 0
        EXPECT_FLOAT_EQ(result.field(7,0), 13); // row 6, col 1
    }
}

TEST(Util3dFiltering, voxelizeFullCloud) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // Create a 3D grid of points (e.g., 10x10x1)
    for (int x = 0; x < 10; ++x) {
        for (int y = 0; y < 10; ++y) {
            cloud->push_back(pcl::PointXYZ(float(x), float(y), 0.0f));
        }
    }

    float voxelSize = 2.0f;
    auto result = util3d::voxelize(cloud, voxelSize);

    EXPECT_EQ(result->size(), 25);
}

TEST(Util3dFiltering, voxelizeWithIndices) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::IndicesPtr indices(new std::vector<int>);

    // Add 100 points, only even-indexed ones will be used
    for (int i = 0; i < 100; ++i) {
        cloud->push_back(pcl::PointXYZ(float(i % 10), float(i / 10), 0.0f));
        if (i % 2 == 0) {
            indices->push_back(i);
        }
    }

    float voxelSize = 2.0f;
    auto result = util3d::voxelize(cloud, indices, voxelSize);

    EXPECT_EQ(result->size(), 25);
}

TEST(Util3dFiltering, voxelizeSmallCloud) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < 3; ++i) {
        cloud->push_back(pcl::PointXYZ(float(i), 0, 0));
    }

    float voxelSize = 0.0001f;  // Very small voxel size (no points get merged)
    auto result = util3d::voxelize(cloud, voxelSize);

    EXPECT_EQ(result->size(), cloud->size());
}

TEST(Util3dFiltering, voxelizeEmptyInput) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    float voxelSize = 1.0f;

    auto result = util3d::voxelize(cloud, voxelSize);
    EXPECT_TRUE(result->empty());

    pcl::IndicesPtr indices(new std::vector<int>);
    auto result2 = util3d::voxelize(cloud, indices, voxelSize);
    EXPECT_TRUE(result2->empty());
}

TEST(Util3dFiltering, voxelizeInvalidVoxelSize) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(0, 0, 0));
    pcl::IndicesPtr indices(new std::vector<int>{0});

    EXPECT_THROW(util3d::voxelize(cloud, 0.0f), UException);
    EXPECT_THROW(util3d::voxelize(cloud, indices, 0.0f), UException);

    // just checking if other point type functions are there, 
    // note that they all call the exact same implemented function 
    // under the hood, so we son't duplicate all tests already 
    // done with pcl::PointXYZ type.
    EXPECT_THROW(util3d::voxelize(pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>()), 0.0f), UException);
    EXPECT_THROW(util3d::voxelize(pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>()), indices, 0.0f), UException);
    EXPECT_THROW(util3d::voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>()), 0.0f), UException);
    EXPECT_THROW(util3d::voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>()), indices, 0.0f), UException);
    EXPECT_THROW(util3d::voxelize(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>()), 0.0f), UException);
    EXPECT_THROW(util3d::voxelize(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>()), indices, 0.0f), UException);
    EXPECT_THROW(util3d::voxelize(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()), 0.0f), UException);
    EXPECT_THROW(util3d::voxelize(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()), indices, 0.0f), UException);
    EXPECT_THROW(util3d::voxelize(pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>()), 0.0f), UException);
    EXPECT_THROW(util3d::voxelize(pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>()), indices, 0.0f), UException);
}

TEST(Util3dFiltering, randomSamplingSamplesCorrectNumberOfPoints)
{
    constexpr int total_points = 100;
    constexpr int sample_size = 10;

    // Create a point cloud with 100 points
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < total_points; ++i) {
        input_cloud->points.emplace_back(static_cast<float>(i), static_cast<float>(i), static_cast<float>(i));
    }
    input_cloud->width = total_points;
    input_cloud->height = 1;
    input_cloud->is_dense = true;

    // Call the function under test
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud = util3d::randomSampling(input_cloud, sample_size);

    // Validate output
    ASSERT_EQ(sampled_cloud->size(), sample_size);
    // Check that the sampled points are from the input set
    for (const auto& pt : *sampled_cloud) {
        bool found = false;
        for (const auto& orig_pt : *input_cloud) {
            if (pt.x == orig_pt.x && pt.y == orig_pt.y && pt.z == orig_pt.z) {
                found = true;
                break;
            }
        }
        EXPECT_TRUE(found) << "Sampled point not found in input cloud: (" << pt.x << ", " << pt.y << ", " << pt.z << ")";
    }
}

TEST(Util3dFiltering, randomSamplingThrowsAssertionForInvalidSampleSize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    input_cloud->push_back(pcl::PointXYZ(1.0f, 2.0f, 3.0f));
    EXPECT_THROW(util3d::randomSampling(input_cloud, 0), UException);
}

TEST(Util3dFiltering, passThroughFiltersCorrectZRange)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Create 10 points along z-axis from 0.0 to 9.0
    for (int i = 0; i < 10; ++i)
        cloud->points.emplace_back(0.0f, 0.0f, static_cast<float>(i));

    cloud->width = 10;
    cloud->height = 1;

    pcl::IndicesPtr indices = nullptr;  // Use full cloud
    float min = 3.0f;
    float max = 6.0f;
    bool negative = false;

    // Run filter
    pcl::IndicesPtr output = util3d::passThrough(cloud, indices, "z", min, max, negative);

    // Should include points with z = 3, 4, 5, 6
    ASSERT_EQ(output->size(), 4);
    for (int idx : *output) {
        float z = cloud->points[idx].z;
        EXPECT_GE(z, min);
        EXPECT_LE(z, max);
    }
}

TEST(Util3dFiltering, passThroughFiltersOutsideZRangeWithNegative)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < 10; ++i)
        cloud->points.emplace_back(0.0f, 0.0f, static_cast<float>(i));

    pcl::IndicesPtr indices = nullptr;
    float min = 3.0f;
    float max = 6.0f;
    bool negative = true;

    pcl::IndicesPtr output = util3d::passThrough(cloud, indices, "z", min, max, negative);

    // Should include all except z = 3,4,5,6 => 6 points
    ASSERT_EQ(output->size(), 6);
    for (int idx : *output) {
        float z = cloud->points[idx].z;
        EXPECT_TRUE(z < min || z > max);
    }
}

TEST(Util3dFiltering, passThroughFiltersWithIndicesSubset)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < 10; ++i)
        cloud->points.emplace_back(0.0f, 0.0f, static_cast<float>(i));

    // Only consider even-indexed points
    pcl::IndicesPtr indices(new std::vector<int>);
    for (int i = 0; i < 10; i += 2)
        indices->push_back(i);

    float min = 2.0f;
    float max = 6.0f;
    bool negative = false;

    pcl::IndicesPtr output = util3d::passThrough(cloud, indices, "z", min, max, negative);

    // From even indices: 2, 4, 6 match => 3 points
    ASSERT_EQ(output->size(), 3);
    for (int idx : *output) {
        float z = cloud->points[idx].z;
        EXPECT_TRUE(z == 2.0f || z == 4.0f || z == 6.0f);
    }
}

TEST(Util3dFiltering, passThroughInvalidAxisTriggersAssertion)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 1.0f));
    pcl::IndicesPtr indices = nullptr;

    EXPECT_THROW(util3d::passThrough(cloud, indices, "invalid_axis", 0.0f, 1.0f, false), UException);
}

TEST(Util3dFiltering, passThroughInvalidRangeTriggersAssertion)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 1.0f));
    pcl::IndicesPtr indices = nullptr;

    EXPECT_THROW(util3d::passThrough(cloud, indices, "z", 5.0f, 2.0f, false), UException);
}

TEST(Util3dFiltering, cropBoxIncludesPointsInBox)
{
    using PointT = pcl::PointXYZ;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    // Points along x-axis from 0 to 9
    for (int i = 0; i < 10; ++i)
        cloud->points.emplace_back(static_cast<float>(i), 0.0f, 0.0f);

    Eigen::Vector4f min(3.0f, -1.0f, -1.0f, 1.0f);
    Eigen::Vector4f max(6.0f, 1.0f, 1.0f, 1.0f);
    pcl::IndicesPtr indices = nullptr;
    Transform transform = Transform::getIdentity();
    bool negative = false;

    // Point included in the box
    auto output = util3d::cropBox(cloud, indices, min, max, transform, negative);

    ASSERT_EQ(output->size(), 4);
    for (int idx : *output) {
        float x = cloud->points[idx].x;
        EXPECT_GE(x, 3.0f);
        EXPECT_LE(x, 6.0f);
    }

    // Point excluded from the box
    negative = true;
    output = util3d::cropBox(cloud, indices, min, max, transform, negative);

    ASSERT_EQ(output->size(), 6);
    for (int idx : *output) {
        float x = cloud->points[idx].x;
        EXPECT_TRUE(x < 3.0f || x > 6.0f);
    }

    // Apply translation
    transform = Transform(3.0f, 0.0f, 0.0f, 0,0,0);  // Shift box forward by 3
    negative = false;
    output = util3d::cropBox(cloud, indices, min, max, transform, negative);

    // Transformed box covers x = [6, 9]
    ASSERT_EQ(output->size(), 4);
    for (int idx : *output) {
        float x = cloud->points[idx].x;
        EXPECT_GE(x, 6.0f);
        EXPECT_LE(x, 9.0f);
    }
}

TEST(Util3dFiltering, cropBoxInvalidBoundsTriggerAssertion)
{
    using PointT = pcl::PointXYZ;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    cloud->points.emplace_back(0.0f, 0.0f, 0.0f);
    pcl::IndicesPtr indices = nullptr;

    Eigen::Vector4f min(5.0f, 0.0f, 0.0f, 1.0f);
    Eigen::Vector4f max(1.0f, 1.0f, 1.0f, 1.0f); // Invalid: min[0] > max[0]

    Transform transform = Transform::getIdentity();

    EXPECT_THROW(util3d::cropBox(cloud, indices, min, max, transform, false), UException);
}

// Main test: filtering points inside a frustum
TEST(Util3dFiltering, frustumFilteringIncludesPointsInFrustum)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Add points along the X axis (frustum forward direction)
    pcl::IndicesPtr indicesOnXAxisOnly(new std::vector<int>{0});
    pcl::IndicesPtr indicesOnYAxisOnly(new std::vector<int>);
    pcl::IndicesPtr indicesOnZAxisOnly(new std::vector<int>);
    pcl::IndicesPtr allIndices(new std::vector<int>{0});
    for (int i = 0; i <= 10; ++i) {
        cloud->points.emplace_back(static_cast<float>(i), 0.0f, 0.0f); // x goes from 0 to 10
        if(i>0) {
            indicesOnXAxisOnly->emplace_back(3*(i-1)+1);
            allIndices->emplace_back(3*(i-1)+1);
            cloud->points.emplace_back(2.0f, static_cast<float>(i)/2.0f, 0.0f); // y goes from 1 to 10 at x=2
            indicesOnYAxisOnly->emplace_back(3*(i-1)+2);
            allIndices->emplace_back(3*(i-1)+2);
            cloud->points.emplace_back(2.0f, 0.0f, static_cast<float>(i)/2.0f); // z goes from 1 to 10 at x=2
            indicesOnZAxisOnly->emplace_back(3*(i-1)+3);
            allIndices->emplace_back(3*(i-1)+3);
        }
    }

    float hFOV = 90.0f;   // wide field of view
    float vFOV = 70.0f;
    float nearClip = 1.5f;
    float farClip = 7.5f;
    Transform cameraPose = Transform(-1,0,0,0,0,0) * CameraModel::opticalRotation(); // camera looking forward on x-axis, 1 meter back

    pcl::IndicesPtr result = util3d::frustumFiltering(
        cloud, nullptr, cameraPose, hFOV, vFOV, nearClip, farClip, false);

    float halfhFOV = tan(hFOV*M_PI/180.0f/2.0f)*3; // at 3 meters from the camera
    float halfvFOV = tan(vFOV*M_PI/180.0f/2.0f)*3; // at 3 meters from the camera

    // Should include points with x in [1,7]
    ASSERT_EQ(result->size(), 16);
    for (int idx : *result) {
        float x = cloud->points[idx].x;
        float y = cloud->points[idx].y;
        float z = cloud->points[idx].z;
        EXPECT_GE(x, nearClip-1.0f);
        EXPECT_LE(x, farClip-1.0f);
        EXPECT_GE(y, -halfhFOV);
        EXPECT_LE(y, halfhFOV);
        EXPECT_GE(z, -halfvFOV);
        EXPECT_LE(z, halfvFOV);
    }
    // Check with all indices, should give same result
    result = util3d::frustumFiltering(
        cloud, allIndices, cameraPose, hFOV, vFOV, nearClip, farClip, false);

    // Should include points with x in [1,7]
    ASSERT_EQ(result->size(), 16);
    for (int idx : *result) {
        float x = cloud->points[idx].x;
        float y = cloud->points[idx].y;
        float z = cloud->points[idx].z;
        EXPECT_GE(x, nearClip-1.0f);
        EXPECT_LE(x, farClip-1.0f);
        EXPECT_GE(y, -halfhFOV);
        EXPECT_LE(y, halfhFOV);
        EXPECT_GE(z, -halfvFOV);
        EXPECT_LE(z, halfvFOV);
    }

    // test negative flag
    result = util3d::frustumFiltering(
        cloud, nullptr, cameraPose, hFOV, vFOV, nearClip, farClip, true);

    ASSERT_EQ(result->size(), 15);
    for (int idx : *result) {
        float x = cloud->points[idx].x;
        float y = cloud->points[idx].y;
        float z = cloud->points[idx].z;
        EXPECT_TRUE(x < nearClip-1.0f || x > farClip-1.0f || y < -halfhFOV || y > halfhFOV || z < -halfvFOV || z > halfvFOV);
    }

    // test with sub-indices
    result = util3d::frustumFiltering(
        cloud, indicesOnXAxisOnly, cameraPose, hFOV, vFOV, nearClip, farClip, false);
    
    // Should return indices in [1,4,7,10,13,16] (1m, 2m, 3m, 4m, 5m, 6m, 7m) from the indicesOnXAxisOnly list
    std::vector<int> expected = {1,4,7,10,13,16};
    ASSERT_EQ(result->size(), expected.size());
    for (int idx : *result) {
        EXPECT_NE(std::find(expected.begin(), expected.end(), idx), expected.end());
    }

    // Check pitch camera rotation
    // Should return indices in [9,12,15,18,21,24] z=(1.5m, 2m, 2.5m, 3m, 3.5m, 4m) from the indicesOnXAxisOnly list
    result = util3d::frustumFiltering(
        cloud, indicesOnZAxisOnly, Transform(2,0,5,0,M_PI/2,0)*CameraModel::opticalRotation(), 1, 1, 0.75, 3.75, false);
    std::vector<int> expectedZ = {9,12,15,18,21,24};
    ASSERT_EQ(result->size(), expectedZ.size());
    for (int idx : *result) {
        EXPECT_NE(std::find(expectedZ.begin(), expectedZ.end(), idx), expectedZ.end());
    }

    // Check yaw camera rotation
    // Should return indices in [8,11,14,17,20,23] y=(1.5m, 2m, 2.5m, 3m, 3.5m, 4m) from the indicesOnYAxisOnly list
    result = util3d::frustumFiltering(
        cloud, indicesOnYAxisOnly, Transform(2,5,0,0,0,-M_PI/2)*CameraModel::opticalRotation(), 1, 1, 0.75, 3.75, false);
    std::vector<int> expectedY = {8,11,14,17,20,23};
    ASSERT_EQ(result->size(), expectedY.size());
    for (int idx : *result) {
        EXPECT_NE(std::find(expectedY.begin(), expectedY.end(), idx), expectedY.end());
    }
}


// Test assertion failure on invalid FOV
TEST(Util3dFiltering, frustumFilteringInvalidFOVTriggersAssertion)
{
    using PointT = pcl::PointXYZ;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    cloud->push_back(PointT(0, 0, 0));
    pcl::IndicesPtr indices = nullptr;
    Transform cameraPose = Transform::getIdentity();

    EXPECT_THROW(util3d::frustumFiltering(cloud, indices, cameraPose, 0.0f, 45.0f, 1.0f, 5.0f, false), UException);
    EXPECT_THROW(util3d::frustumFiltering(cloud, indices, cameraPose, 45.0f, 0.0f, 1.0f, 5.0f, false), UException);
}

// Test assertion failure on invalid clip plane distances
TEST(Util3dFiltering, frustumFilteringInvalidClipPlaneTriggersAssertion)
{
    using PointT = pcl::PointXYZ;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    cloud->push_back(PointT(0, 0, 0));
    pcl::IndicesPtr indices = nullptr;
    Transform cameraPose = Transform::getIdentity();

    EXPECT_THROW(util3d::frustumFiltering(cloud, indices, cameraPose, 60.0f, 45.0f, 5.0f, 1.0f, false), UException);
}