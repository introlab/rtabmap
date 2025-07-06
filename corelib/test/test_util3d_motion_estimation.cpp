#include "gtest/gtest.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_motion_estimation.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/utilite/UException.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/core/Version.h"
#include <pcl/io/pcd_io.h>

using namespace rtabmap;

float randomNoise(float max) {
    return ((static_cast<float>(rand()) / RAND_MAX) * 2.0f - 1.0f) * max;
}

TEST(Util3dMotionEstimation, estimateMotion3DTo2DBasic) {

    // Two triangles in front of the camera at two different depths, centered with the middle of the image frame
    std::map<int, cv::Point3f> words3A = {
        {0, cv::Point3f(1,0,1)},
        {1, cv::Point3f(1,1,-1)},
        {2, cv::Point3f(1,-1,-1)},
        {3, cv::Point3f(2,0,0)},
        {4, cv::Point3f(2,0.5,0)},
        {5, cv::Point3f(3,-0.5,0)},
        {6, cv::Point3f(2,0,10)} // outlier
    };

    CameraModel cam(200, 200, 320, 240, CameraModel::opticalRotation(), 0, cv::Size(640, 480));
    std::map<int, cv::KeyPoint> words2B;
    for(auto & pt: words3A) {
        cv::Point3f ptt = util3d::transformPoint(pt.second, cam.localTransform().inverse());
        float u,v;
        cam.reproject(ptt.x,ptt.y,ptt.z, u, v);
        if(cam.inFrame(u,v)) {
            words2B.insert(std::make_pair(pt.first, cv::KeyPoint(u, v, 3)));
        }
        else {
            words2B.insert(std::make_pair(pt.first, cv::KeyPoint(10, 10, 3)));
        }
    }
    std::map<int, cv::Point3f> words3B; // leave empty

    Transform guess = Transform::getIdentity(); // non-null identity
    cv::Mat covariance;
    std::vector<int> matchesOut, inliersOut;

    // Test without image size set, so covariance is computed completely by 
    // reproj errors, which is expected to be zero here
    Transform result = util3d::estimateMotion3DTo2D(
        words3A, words2B, CameraModel(200, 200, 320, 240), 
        /*minInliers=*/4,
        /*iterations=*/100,
        /*reprojError=*/2.0,
        /*flagsPnP=*/0,
        /*refineIterations=*/1,
        /*varianceMedianRatio=*/4,
        /*maxVariance=*/0.0f,
        guess,
        words3B,
        &covariance,
        &matchesOut,
        &inliersOut,
        /*splitLinearCovarianceComponents=*/false
    );

    EXPECT_FALSE(result.isNull());
    float x,y,z,roll,pitch,yaw;
    result.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
    EXPECT_NEAR(x, 0, 1e-6);
    EXPECT_NEAR(y, 0, 1e-6);
    EXPECT_NEAR(z, 0, 1e-6);
    EXPECT_NEAR(roll, 0, 1e-6);
    EXPECT_NEAR(pitch, 0, 1e-6);
    EXPECT_NEAR(yaw, 0, 1e-6);
    EXPECT_EQ(matchesOut.size(), 7u);
    EXPECT_EQ(inliersOut.size(), 6u);

    // covariance must be 6x6
    EXPECT_EQ(covariance.rows, 6);
    EXPECT_EQ(covariance.cols, 6);
    EXPECT_NEAR(covariance.at<double>(0,0), 1e-6, 1e-6);
    EXPECT_NEAR(covariance.at<double>(3,3), 1e-6, 1e-6);

    // Test with image size set to compute covariance differently: 
    // 3D points of A reprojected in B frame with 10 % error. For the angle, 
    // it should still be close to zero (10% error is added to the ray, so if 
    // there was no error in angle, then result doesn't change).
    result = util3d::estimateMotion3DTo2D(
        words3A, words2B, cam,
        /*minInliers=*/4,
        /*iterations=*/100,
        /*reprojError=*/2.0,
        /*flagsPnP=*/0,
        /*refineIterations=*/1,
        /*varianceMedianRatio=*/4,
        /*maxVariance=*/0.0f,
        guess,
        words3B,
        &covariance,
        &matchesOut,
        &inliersOut,
        /*splitLinearCovarianceComponents=*/false
    );

    EXPECT_FALSE(result.isNull());
    result.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
    EXPECT_NEAR(x, 0, 1e-6);
    EXPECT_NEAR(y, 0, 1e-6);
    EXPECT_NEAR(z, 0, 1e-6);
    EXPECT_NEAR(roll, 0, 1e-6);
    EXPECT_NEAR(pitch, 0, 1e-6);
    EXPECT_NEAR(yaw, 0, 1e-6);
    EXPECT_EQ(matchesOut.size(), 7u);
    EXPECT_EQ(inliersOut.size(), 6u);

    // covariance must be 6x6
    EXPECT_EQ(covariance.rows, 6);
    EXPECT_EQ(covariance.cols, 6);
    EXPECT_NEAR(covariance.at<double>(0,0), 0.066, 1e-3);
    EXPECT_NEAR(covariance.at<double>(3,3), 1e-6, 1e-6);

    // Test with exact same 3D points, covariance in xyz expected to be close to 0 (or epsilon 1e-6)
    result = util3d::estimateMotion3DTo2D(
        words3A, words2B, cam,
        /*minInliers=*/4,
        /*iterations=*/100,
        /*reprojError=*/2.0,
        /*flagsPnP=*/0,
        /*refineIterations=*/1,
        /*varianceMedianRatio=*/4,
        /*maxVariance=*/0.0f,
        guess,
        words3A,
        &covariance,
        &matchesOut,
        &inliersOut,
        /*splitLinearCovarianceComponents=*/false
    );

    EXPECT_FALSE(result.isNull());
    result.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
    EXPECT_NEAR(x, 0, 1e-6);
    EXPECT_NEAR(y, 0, 1e-6);
    EXPECT_NEAR(z, 0, 1e-6);
    EXPECT_NEAR(roll, 0, 1e-6);
    EXPECT_NEAR(pitch, 0, 1e-6);
    EXPECT_NEAR(yaw, 0, 1e-6);
    EXPECT_EQ(matchesOut.size(), 7u);
    EXPECT_EQ(inliersOut.size(), 6u);

    // covariance must be 6x6
    EXPECT_EQ(covariance.rows, 6);
    EXPECT_EQ(covariance.cols, 6);
    EXPECT_NEAR(covariance.at<double>(0,0), 1e-6, 1e-6);
    EXPECT_NEAR(covariance.at<double>(3,3), 1e-6, 1e-6);
}

// Same test than above, but with added noise on the points and pixels
TEST(Util3dMotionEstimation, estimateMotion3DTo2DWithNoise) {

    // Two triangles in front of the camera at two different depths, centered with the middle of the image frame
    std::map<int, cv::Point3f> words3A = {
        {0, cv::Point3f(1,0,1)},
        {1, cv::Point3f(1,1,-1)},
        {2, cv::Point3f(1,-1,-1)},
        {3, cv::Point3f(2,0,0)},
        {4, cv::Point3f(2,0.5,0)},
        {5, cv::Point3f(3,-0.5,0)},
        {6, cv::Point3f(2,0,10)} // outlier
    };

    CameraModel cam(200, 200, 320, 240, CameraModel::opticalRotation(), 0, cv::Size(640, 480));
    std::map<int, cv::KeyPoint> words2B;
    std::map<int, cv::Point3f> words3B;
    for(auto & pt: words3A) {
        cv::Point3f ptt = util3d::transformPoint(pt.second, cam.localTransform().inverse());
        float u,v;
        cam.reproject(ptt.x,ptt.y,ptt.z, u, v);
        if(cam.inFrame(u,v)) {
            // Add +-5 pixels noise to 2D keypoints
            words2B.insert(std::make_pair(pt.first, cv::KeyPoint(u+randomNoise(5.0f), v+randomNoise(5.0f), 3)));
        }
        else {
            words2B.insert(std::make_pair(pt.first, cv::KeyPoint(10, 10, 3)));
        }
        // Add +-2 cm noise to 3D points
        words3B.insert(std::make_pair(pt.first, 
            cv::Point3f(pt.second.x+randomNoise(0.02f), pt.second.y+randomNoise(0.02f), pt.second.z+randomNoise(0.02f))));
        pt.second.x += randomNoise(0.02f);
        pt.second.y += randomNoise(0.02f);
        pt.second.z += randomNoise(0.02f);
    }    

    Transform guess = Transform::getIdentity(); // non-null identity
    cv::Mat covariance;
    std::vector<int> matchesOut, inliersOut;

    // Test without image size set, so covariance is computed completely by 
    // reproj errors, which is expected to be zero here
    Transform result = util3d::estimateMotion3DTo2D(
        words3A, words2B, cam, 
        /*minInliers=*/4,
        /*iterations=*/100,
        /*reprojError=*/5.0,
        /*flagsPnP=*/0,
        /*refineIterations=*/1,
        /*varianceMedianRatio=*/4,
        /*maxVariance=*/0.0f,
        guess,
        words3B,
        &covariance,
        &matchesOut,
        &inliersOut,
        /*splitLinearCovarianceComponents=*/true
    );

    EXPECT_FALSE(result.isNull());
    float x,y,z,roll,pitch,yaw;
    result.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
    EXPECT_NEAR(x, 0, 3e-2);
    EXPECT_NEAR(y, 0, 3e-2);
    EXPECT_NEAR(z, 0, 3e-2);
    EXPECT_NEAR(roll, 0, 1e-2);
    EXPECT_NEAR(pitch, 0, 1e-2);
    EXPECT_NEAR(yaw, 0, 1e-2);
    EXPECT_EQ(matchesOut.size(), 7u);
    EXPECT_EQ(inliersOut.size(), 6u);

    // covariance must be 6x6
    EXPECT_EQ(covariance.rows, 6);
    EXPECT_EQ(covariance.cols, 6);
    EXPECT_NEAR(covariance.at<double>(0,0), 5e-3, 1e-2);
    EXPECT_NEAR(covariance.at<double>(1,1), 5e-3, 1e-2);
    EXPECT_NEAR(covariance.at<double>(2,2), 5e-3, 1e-2);
    EXPECT_NE(covariance.at<double>(0,0), covariance.at<double>(1,1));
    EXPECT_NE(covariance.at<double>(1,1), covariance.at<double>(2,2));
    EXPECT_NE(covariance.at<double>(0,0), covariance.at<double>(2,2));
    EXPECT_NEAR(covariance.at<double>(3,3), 1e-2, 1e-2);

}

TEST(Util3dMotionEstimation, estimateMotion3DTo2DMultiCamBasic) {

    // Two triangles in front of the camera at two different depths, centered with the middle of the image frame
    std::map<int, cv::Point3f> words3A = {
        {0, cv::Point3f(1,0,0.5)},
        {1, cv::Point3f(1,0.5,-0.5)},
        {2, cv::Point3f(1,-0.5,-0.5)},
        {3, cv::Point3f(2,0,0)},
        {4, cv::Point3f(2,0.25,0)},
        {5, cv::Point3f(3,-0.25,0)},
        {6, cv::Point3f(2,0,10)} // outlier
    };
    // Transform that point cloud for the left and right cameras
    std::map<int, cv::Point3f> words3ALeftRight;
    Transform leftT(0,0,M_PI/2);
    Transform rightT(0,0,-M_PI/2);

    int index = 7;
    for(auto & pt: words3A) {
        cv::Point3f ptT = util3d::transformPoint(pt.second, leftT);
        words3ALeftRight.insert(std::make_pair(index++, ptT));
        ptT = util3d::transformPoint(pt.second, rightT);
        words3ALeftRight.insert(std::make_pair(index++, ptT));
    }
    words3A.insert(words3ALeftRight.begin(), words3ALeftRight.end());

    float imageWidth = 640;
    CameraModel camFront(200, 200, 320, 240, CameraModel::opticalRotation(), 0, cv::Size(imageWidth, 480));
    CameraModel camLeft(200, 200, 320, 240, leftT*CameraModel::opticalRotation(), 0, cv::Size(imageWidth, 480));
    CameraModel camRight(200, 200, 320, 240, rightT*CameraModel::opticalRotation(), 0, cv::Size(imageWidth, 480));
    std::vector<CameraModel> models = {camFront, camLeft, camRight};
    std::map<int, cv::KeyPoint> words2B;

    for(auto & pt: words3A) {
        for(size_t i=0; i<models.size(); ++i) {
            cv::Point3f ptt = util3d::transformPoint(pt.second, models[i].localTransform().inverse());
            float u,v;
            if(ptt.z>0) {
                models[i].reproject(ptt.x,ptt.y,ptt.z, u, v);
                if(models[i].inFrame(u,v)) {
                    words2B.insert(std::make_pair(pt.first, cv::KeyPoint((i*imageWidth)+u, v, 3)));
                    break;
                }
                else if(pt.second.z > 9) {
                    words2B.insert(std::make_pair(pt.first, cv::KeyPoint((i*imageWidth)+10, 10, 3)));
                    break;
                }
            }
        }
    }
    EXPECT_EQ(words3A.size(), words2B.size());
    std::map<int, cv::Point3f> words3B; // leave empty

    Transform guess = Transform::getIdentity(); // non-null identity
    cv::Mat covariance;
    std::vector<std::vector<int> > matchesOut, inliersOut;

    // For the three approaches, the results should be the same
    Transform result;
    for(int i=0; i<3; ++i) {
        result = util3d::estimateMotion3DTo2D(
            words3A, words2B, models, 
            /*samplingPolicy*/i,
            /*minInliers=*/4,
            /*iterations=*/100,
            /*reprojError=*/2.0,
            /*flagsPnP=*/0,
            /*refineIterations=*/1,
            /*varianceMedianRatio=*/4,
            /*maxVariance=*/0.0f,
            guess,
            words3B,
            &covariance,
            &matchesOut,
            &inliersOut,
            /*splitLinearCovarianceComponents=*/false
        );

#ifdef RTABMAP_OPENGV
        EXPECT_FALSE(result.isNull());
        float x,y,z,roll,pitch,yaw;
        result.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
        EXPECT_NEAR(x, 0, 1e-2);
        EXPECT_NEAR(y, 0, 1e-2);
        EXPECT_NEAR(z, 0, 1e-2);
        EXPECT_NEAR(roll, 0, 5e-3);
        EXPECT_NEAR(pitch, 0, 5e-3);
        EXPECT_NEAR(yaw, 0, 5e-3);
        EXPECT_EQ(matchesOut.size(), 3u);
        EXPECT_EQ(inliersOut.size(), 3u);
        for(size_t i=0; i<matchesOut.size(); ++i) {
            EXPECT_EQ(matchesOut[i].size(), 7u);
        }
        for(size_t i=0; i<inliersOut.size(); ++i) {
            EXPECT_EQ(inliersOut[i].size(), 6u);
        }

        // covariance must be 6x6
        EXPECT_EQ(covariance.rows, 6);
        EXPECT_EQ(covariance.cols, 6);
        EXPECT_NEAR(covariance.at<double>(0,0), 0.03, 1e-2);
        EXPECT_NEAR(covariance.at<double>(3,3), 1e-3, 1e-3);
#else
        EXPECT_TRUE(result.isNull());
#endif
    }
}

// Same thing than above, but with noise
TEST(Util3dMotionEstimation, estimateMotion3DTo2DMultiCamWithNoise) {

    // Two triangles in front of the camera at two different depths, centered with the middle of the image frame
    std::map<int, cv::Point3f> words3A = {
        {0, cv::Point3f(1,0,0.5)},
        {1, cv::Point3f(1,0.5,-0.5)},
        {2, cv::Point3f(1,-0.5,-0.5)},
        {3, cv::Point3f(2,0,0)},
        {4, cv::Point3f(2,0.25,0)},
        {5, cv::Point3f(3,-0.25,0)},
        {6, cv::Point3f(2,0,10)} // outlier
    };
    // Transform that point cloud for the left and right cameras
    std::map<int, cv::Point3f> words3ALeftRight;
    Transform leftT(0,0,M_PI/2);
    Transform rightT(0,0,-M_PI/2);

    int index = 7;
    for(auto & pt: words3A) {
        cv::Point3f ptT = util3d::transformPoint(pt.second, leftT);
        words3ALeftRight.insert(std::make_pair(index++, ptT));
        ptT = util3d::transformPoint(pt.second, rightT);
        words3ALeftRight.insert(std::make_pair(index++, ptT));
    }
    words3A.insert(words3ALeftRight.begin(), words3ALeftRight.end());

    float imageWidth = 640;
    CameraModel camFront(200, 200, 320, 240, CameraModel::opticalRotation(), 0, cv::Size(imageWidth, 480));
    CameraModel camLeft(200, 200, 320, 240, leftT*CameraModel::opticalRotation(), 0, cv::Size(imageWidth, 480));
    CameraModel camRight(200, 200, 320, 240, rightT*CameraModel::opticalRotation(), 0, cv::Size(imageWidth, 480));
    std::vector<CameraModel> models = {camFront, camLeft, camRight};
    std::map<int, cv::KeyPoint> words2B;
    std::map<int, cv::Point3f> words3B;

    for(auto & pt: words3A) {
        for(size_t i=0; i<models.size(); ++i) {
            cv::Point3f ptt = util3d::transformPoint(pt.second, models[i].localTransform().inverse());
            float u,v;
            if(ptt.z>0) {
                models[i].reproject(ptt.x,ptt.y,ptt.z, u, v);
                if(models[i].inFrame(u,v)) {
                    // Add +-5 pixels noise to 2D keypoints
                    words2B.insert(std::make_pair(pt.first, cv::KeyPoint((i*imageWidth)+u+randomNoise(5.0f), v+randomNoise(5.0f), 3)));
                    break;
                }
                else if(pt.second.z > 9) {
                    words2B.insert(std::make_pair(pt.first, cv::KeyPoint((i*imageWidth)+10, 10, 3)));
                    break;
                }
            }
        }
        // Add +-2 cm noise to 3D points
        words3B.insert(std::make_pair(pt.first, 
            cv::Point3f(pt.second.x+randomNoise(0.02f), pt.second.y+randomNoise(0.02f), pt.second.z+randomNoise(0.02f))));
        pt.second.x += randomNoise(0.02f);
        pt.second.y += randomNoise(0.02f);
        pt.second.z += randomNoise(0.02f);
    }
    EXPECT_EQ(words3A.size(), words2B.size());

    Transform guess = Transform::getIdentity(); // non-null identity
    cv::Mat covariance;
    std::vector<std::vector<int> > matchesOut, inliersOut;

    // For the three approaches, the results should be the same
    Transform result;
    for(int i=0; i<3; ++i) {
        result = util3d::estimateMotion3DTo2D(
            words3A, words2B, models, 
            /*samplingPolicy*/i,
            /*minInliers=*/4,
            /*iterations=*/100,
            /*reprojError=*/6.0,
            /*flagsPnP=*/0,
            /*refineIterations=*/1,
            /*varianceMedianRatio=*/4,
            /*maxVariance=*/0.0f,
            guess,
            words3B,
            &covariance,
            &matchesOut,
            &inliersOut,
            /*splitLinearCovarianceComponents=*/false
        );

#ifdef RTABMAP_OPENGV
        EXPECT_FALSE(result.isNull());
        float x,y,z,roll,pitch,yaw;
        result.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
        EXPECT_NEAR(x, 0, 6e-2);
        EXPECT_NEAR(y, 0, 6e-2);
        EXPECT_NEAR(z, 0, 6e-2);
        EXPECT_NEAR(roll, 0, 5e-2);
        EXPECT_NEAR(pitch, 0, 5e-2);
        EXPECT_NEAR(yaw, 0, 5e-2);
        EXPECT_EQ(matchesOut.size(), 3u);
        EXPECT_EQ(inliersOut.size(), 3u);
        for(size_t i=0; i<matchesOut.size(); ++i) {
            EXPECT_EQ(matchesOut[i].size(), 7u);
        }
        for(size_t i=0; i<inliersOut.size(); ++i) {
            EXPECT_GE(inliersOut[i].size(), 2u);
        }

        // covariance must be 6x6
        EXPECT_EQ(covariance.rows, 6);
        EXPECT_EQ(covariance.cols, 6);
        EXPECT_LT(covariance.at<double>(0,0), 0.008);
        EXPECT_GT(covariance.at<double>(0,0), 1e-5);
        EXPECT_LT(covariance.at<double>(3,3), 0.06);
        EXPECT_GT(covariance.at<double>(3,3), 1e-5);
#else
        EXPECT_TRUE(result.isNull());
#endif
    }
}