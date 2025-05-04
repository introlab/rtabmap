#include "gtest/gtest.h"
#include "rtabmap/core/util3d_filtering.h"
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