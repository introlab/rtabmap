#include "gtest/gtest.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_features.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/utilite/UException.h"
#include "rtabmap/utilite/UConversion.h"

using namespace rtabmap;

TEST(Util3dFeaturesTest, GenerateKeypoints3DDepthBasicProjection) {
    // Arrange
    std::vector<cv::KeyPoint> keypoints = {
        cv::KeyPoint(10.0f, 10.0f, 1.0f),
        cv::KeyPoint(20.0f, 20.0f, 1.0f)
    };

    // Create a 30x30 depth image with constant depth of 2.0
    cv::Mat depth = cv::Mat::ones(30, 30, CV_32FC1) * 2.0f;

    CameraModel model(100, 100, 15, 15, Transform::getIdentity(), 0, cv::Size(30,30));

    auto keypoints3d = util3d::generateKeypoints3DDepth(
        keypoints,
        depth,
        model,
        0.5f,
        5.0f
    );

    // Assert
    ASSERT_EQ(keypoints3d.size(), keypoints.size());
    for (const auto& pt : keypoints3d) {
        EXPECT_TRUE(util3d::isFinite(pt));  // not NaN or Inf
        EXPECT_NEAR(pt.z, 2.0f, 1e-5);
    }

    // invalid depth
    keypoints3d = util3d::generateKeypoints3DDepth(
        keypoints,
        cv::Mat::zeros(30, 30, CV_32FC1),
        model,
        0.5f,
        5.0f
    );

    // Assert
    ASSERT_EQ(keypoints3d.size(), keypoints.size());
    for (const auto& pt : keypoints3d) {
        EXPECT_TRUE(!util3d::isFinite(pt));  // not NaN or Inf
    }
}

TEST(Util3dFeaturesTest, GenerateKeypoints3DDepthMultiCameras) {
    std::vector<cv::KeyPoint> keypoints = {
        cv::KeyPoint(15.0f, 15.0f, 1.0f),
        cv::KeyPoint(45.0f, 15.0f, 1.0f),
        cv::KeyPoint(75.0f, 15.0f, 1.0f),
        cv::KeyPoint(105.0f, 15.0f, 1.0f)
    };

    cv::Mat depth = cv::Mat::ones(30, 120, CV_32FC1) * 2.0f;

   
    std::vector<CameraModel> cameraModels = {
        CameraModel(100, 100, 15, 15, Transform(0,0,M_PI/2)*CameraModel::opticalRotation(), 0, cv::Size(30,30)), // left
        CameraModel(100, 100, 15, 15, CameraModel::opticalRotation(), 0, cv::Size(30,30)), // forward
        CameraModel(100, 100, 15, 15, Transform(0,0,-M_PI/2)*CameraModel::opticalRotation(), 0, cv::Size(30,30)), // right
        CameraModel(100, 100, 15, 15, Transform(0,0,M_PI)*CameraModel::opticalRotation(), 0, cv::Size(30,30)) // back
    };

    std::vector<cv::Point3f> keypoints3d = util3d::generateKeypoints3DDepth(
        keypoints,
        depth,
        cameraModels,
        0.5f,
        5.0f
    );

    ASSERT_EQ(keypoints3d.size(), keypoints.size());
    for (const auto& pt : keypoints3d) {
        EXPECT_TRUE(util3d::isFinite(pt));
    }
    EXPECT_NEAR(keypoints3d[0].y, 2.0f, 1e-5);
    EXPECT_NEAR(keypoints3d[1].x, 2.0f, 1e-5);
    EXPECT_NEAR(keypoints3d[2].y, -2.0f, 1e-5);
    EXPECT_NEAR(keypoints3d[3].x, -2.0f, 1e-5);
}

TEST(Util3dFeaturesTest, GenerateKeypoints3DDisparityValidDisparity) {
    std::vector<cv::KeyPoint> keypoints = {
        cv::KeyPoint(5.0f, 5.0f, 1.0f),
        cv::KeyPoint(10.0f, 10.0f, 1.0f)
    };

    StereoCameraModel stereoModel(100.0, 100.0, 10.0, 10.0, 0.075, Transform::getIdentity(), cv::Size(20,20));

    // d = (baseline * f)/Z
    cv::Mat disparity = cv::Mat::zeros(20, 20, CV_32F);
    disparity.at<float>(5, 5) = stereoModel.baseline()*stereoModel.left().fx() / 2.0f; // Depth = 2.0
    disparity.at<float>(10, 10) = stereoModel.baseline()*stereoModel.left().fx(); // Depth = 1.0

    std::vector<cv::Point3f> keypoints3D = util3d::generateKeypoints3DDisparity(
        keypoints, disparity, stereoModel, 0.1f, 5.0f
    );

    ASSERT_EQ(keypoints3D.size(), keypoints.size());

    EXPECT_NEAR(keypoints3D[0].z, 2.0f, 1e-4);
    EXPECT_NEAR(keypoints3D[1].z, 1.0f, 1e-4);
}

TEST(Util3dFeaturesTest, GenerateKeypoints3DDisparityInvalidDisparityReturnsNaN) {
    std::vector<cv::KeyPoint> keypoints = {
        cv::KeyPoint(5.0f, 5.0f, 1.0f),
        cv::KeyPoint(10.0f, 10.0f, 1.0f)
    };

    cv::Mat disparity = cv::Mat::zeros(20, 20, CV_32F);
    disparity.at<float>(5, 5) = 0.0f;  // Invalid disparity
    disparity.at<float>(10, 10) = -1.0f; // Invalid disparity

    StereoCameraModel stereoModel(100.0, 100.0, 10.0, 10.0, 0.075, Transform::getIdentity(), cv::Size(20,20));

    std::vector<cv::Point3f> keypoints3D = util3d::generateKeypoints3DDisparity(
        keypoints, disparity, stereoModel, 0.1f, 5.0f
    );

    for (const auto &pt : keypoints3D) {
        EXPECT_FALSE(util3d::isFinite(pt));
    }
}

TEST(Util3dFeaturesTest, GenerateKeypoints3DDisparityDepthClipping) {
    std::vector<cv::KeyPoint> keypoints = {
        cv::KeyPoint(5.0f, 5.0f, 1.0f),  // z = 2.0
        cv::KeyPoint(10.0f, 10.0f, 1.0f) // z = 0.5
    };

    StereoCameraModel stereoModel(100.0, 100.0, 10.0, 10.0, 0.075, Transform::getIdentity(), cv::Size(20,20));

    // d = (baseline * f)/Z
    cv::Mat disparity = cv::Mat::zeros(20, 20, CV_32F);
    disparity.at<float>(5, 5) = stereoModel.baseline()*stereoModel.left().fx() / 2.0f;   // Depth = 2.0
    disparity.at<float>(10, 10) = stereoModel.baseline()*stereoModel.left().fx() / 0.5f; // Depth = 0.5

    // Clip to minDepth = 1.0
    std::vector<cv::Point3f> keypoints3D = util3d::generateKeypoints3DDisparity(
        keypoints, disparity, stereoModel, 1.0f, 3.0f
    );

    EXPECT_FALSE(util3d::isFinite(keypoints3D[1]));  // z = 0.5, should be NaN
    EXPECT_NEAR(keypoints3D[0].z, 2.0f, 1e-4);      // z = 2.0, should be valid
}

TEST(Util3dFeaturesTest, GenerateKeypoints3DStereoValidPointsWithDepthFilter) {
    std::vector<cv::Point2f> leftCorners = {
        {100.0f, 100.0f},
        {120.0f, 120.0f}
    };
    std::vector<cv::Point2f> rightCorners = {
        {90.0f, 100.0f},   // disparity = 10
        {110.0f, 120.0f}   // disparity = 10
    };

    StereoCameraModel model(500.0, 500.0, 100.0, 100.0, 0.075, Transform::getIdentity());
    std::vector<unsigned char> mask; // Empty mask = all valid

    float minDepth = 2.0f;
    float maxDepth = 6.0f;

    auto result = util3d::generateKeypoints3DStereo(leftCorners, rightCorners, model, mask, minDepth, maxDepth);

    ASSERT_EQ(result.size(), leftCorners.size());

    for (const auto& pt : result) {
        EXPECT_TRUE(util3d::isFinite(pt));
        EXPECT_NEAR(pt.z, 3.75f, 0.001);
    }
}

TEST(Util3dFeaturesTest, GenerateKeypoints3DStereoInvalidDisparityResultsInNaN) {
    std::vector<cv::Point2f> leftCorners = {
        {100.0f, 100.0f}
    };
    std::vector<cv::Point2f> rightCorners = {
        {100.0f, 100.0f} // disparity = 0.0 (invalid)
    };

    StereoCameraModel model(500.0, 500.0, 100.0, 100.0, 0.075, Transform::getIdentity());
    std::vector<unsigned char> mask;

    auto result = util3d::generateKeypoints3DStereo(leftCorners, rightCorners, model, mask, 0.1f, 10.0f);

    ASSERT_EQ(result.size(), 1);
    EXPECT_TRUE(std::isnan(result[0].x));
    EXPECT_TRUE(std::isnan(result[0].y));
    EXPECT_TRUE(std::isnan(result[0].z));
}

TEST(Util3dFeaturesTest, GenerateKeypoints3DStereoAppliesMaskCorrectly) {
    std::vector<cv::Point2f> leftCorners = {
        {100.0f, 100.0f},
        {120.0f, 120.0f}
    };
    std::vector<cv::Point2f> rightCorners = {
        {90.0f, 100.0f},   // disparity = 10
        {110.0f, 120.0f}   // disparity = 10
    };
    std::vector<unsigned char> mask = {0, 1};  // Only second is valid

    StereoCameraModel model(500.0, 500.0, 100.0, 100.0, 0.075, Transform::getIdentity());

    auto result = util3d::generateKeypoints3DStereo(leftCorners, rightCorners, model, mask, 0.1f, 10.0f);

    ASSERT_EQ(result.size(), 2);
    EXPECT_TRUE(std::isnan(result[0].x));
    EXPECT_TRUE(util3d::isFinite(result[1]));
}

TEST(Util3dFeaturesTest, GenerateKeypoints3DStereoOutOfRangeDepthResultsInNaN) {
    std::vector<cv::Point2f> leftCorners = {
        {100.0f, 100.0f}
    };
    std::vector<cv::Point2f> rightCorners = {
        {99.5f, 100.0f} // very small disparity → large depth
    };

    StereoCameraModel model(500.0, 500.0, 100.0, 100.0, 0.075, Transform::getIdentity());
    std::vector<unsigned char> mask;

    auto result = util3d::generateKeypoints3DStereo(leftCorners, rightCorners, model, mask, 0.1f, 2.0f);

    ASSERT_EQ(result.size(), 1);
    EXPECT_TRUE(std::isnan(result[0].x));
}

TEST(Util3dFeaturesTest, AggregateBasicAggregation) {
    std::list<int> wordIds = {101, 102, 103};
    std::vector<cv::KeyPoint> keypoints = {
        cv::KeyPoint(10.0f, 20.0f, 1.0f),
        cv::KeyPoint(30.0f, 40.0f, 1.0f),
        cv::KeyPoint(50.0f, 60.0f, 1.0f)
    };

    auto result = util3d::aggregate(wordIds, keypoints);

    ASSERT_EQ(result.size(), 3);
    
    auto it = result.begin();
    EXPECT_EQ(it->first, 101);
    EXPECT_FLOAT_EQ(it->second.pt.x, 10.0f); ++it;
    EXPECT_EQ(it->first, 102);
    EXPECT_FLOAT_EQ(it->second.pt.y, 40.0f); ++it;
    EXPECT_EQ(it->first, 103);
    EXPECT_FLOAT_EQ(it->second.pt.x, 50.0f);
}

TEST(Util3dFeaturesTest, AggregateHandlesDuplicateWordIds) {
    std::list<int> wordIds = {200, 201, 200};
    std::vector<cv::KeyPoint> keypoints = {
        cv::KeyPoint(1.0f, 2.0f, 1.0f),
        cv::KeyPoint(3.0f, 4.0f, 1.0f),
        cv::KeyPoint(5.0f, 6.0f, 1.0f)
    };

    auto result = util3d::aggregate(wordIds, keypoints);

    EXPECT_EQ(result.count(200), 2);
    EXPECT_EQ(result.count(201), 1);

    auto range = result.equal_range(200);
    std::vector<float> x_values;
    for (auto it = range.first; it != range.second; ++it) {
        x_values.push_back(it->second.pt.x);
    }
    EXPECT_EQ(x_values[0], 1.0f);
    EXPECT_EQ(x_values[1], 5.0f);
}

TEST(Util3dFeaturesTest, AggregateEmptyInputReturnsEmptyMapOrInvalid) {
    std::list<int> wordIds;
    std::vector<cv::KeyPoint> keypoints;

    auto result = util3d::aggregate(wordIds, keypoints);

    EXPECT_TRUE(result.empty());

    wordIds.push_back(1);
    EXPECT_THROW(util3d::aggregate(wordIds, keypoints), UException);
}
namespace {

// Synthetic 2-view setup for generateWords3DMono(): random 3D points seen by a
// camera with a significant lever arm relative to the base frame, and a base
// motion with rotation (so the lever arm also contributes to the camera
// translation).
struct MonoScene
{
    CameraModel model;
    Transform baseMotion; // pose of next base frame in ref base frame
    std::map<int, cv::KeyPoint> refWords;
    std::map<int, cv::KeyPoint> nextWords;
    std::map<int, cv::Point3f> points3D; // ground truth, in ref base frame
};

MonoScene makeMonoScene()
{
    MonoScene scene;
    const Transform localTransform = Transform(0.3f, -0.1f, 0.5f, 0, 0, 0) * CameraModel::opticalRotation();
    scene.model = CameraModel(300.0, 300.0, 160.0, 120.0, localTransform, 0, cv::Size(320, 240));
    scene.baseMotion = Transform(0.15f, 0.05f, 0.0f, 0.0f, 0.0f, 0.15f);
    const Transform cameraMotion = localTransform.inverse() * scene.baseMotion * localTransform;

    cv::RNG rng(42);
    for(int id=1; id<=200; ++id)
    {
        // in ref camera frame
        cv::Point3f ptA(rng.uniform(-1.5f, 1.5f), rng.uniform(-1.0f, 1.0f), rng.uniform(2.0f, 5.0f));
        cv::Point3f ptB = util3d::transformPoint(ptA, cameraMotion.inverse());
        if(ptB.z <= 0)
        {
            continue;
        }
        float uA, vA, uB, vB;
        scene.model.reproject(ptA.x, ptA.y, ptA.z, uA, vA);
        scene.model.reproject(ptB.x, ptB.y, ptB.z, uB, vB);
        if(scene.model.inFrame(uA, vA) && scene.model.inFrame(uB, vB))
        {
            scene.refWords.insert(std::make_pair(id, cv::KeyPoint(uA, vA, 3)));
            scene.nextWords.insert(std::make_pair(id, cv::KeyPoint(uB, vB, 3)));
            scene.points3D.insert(std::make_pair(id, util3d::transformPoint(ptA, localTransform)));
        }
    }
    return scene;
}

void expectWordsInBaseFrame(const MonoScene & scene, const std::map<int, cv::Point3f> & words3D)
{
    ASSERT_GT(words3D.size(), scene.points3D.size()/2);
    for(std::map<int, cv::Point3f>::const_iterator iter=words3D.begin(); iter!=words3D.end(); ++iter)
    {
        const cv::Point3f & gt = scene.points3D.at(iter->first);
        EXPECT_NEAR(iter->second.x, gt.x, 0.02f) << "word " << iter->first;
        EXPECT_NEAR(iter->second.y, gt.y, 0.02f) << "word " << iter->first;
        EXPECT_NEAR(iter->second.z, gt.z, 0.02f) << "word " << iter->first;
    }
}

} // namespace

// Scale taken from the motion guess (Mem/StereoFromMotion, Vis/EstimationType=2):
// the camera local transform translation must not be scaled.
TEST(Util3dFeaturesTest, GenerateWords3DMonoWithMotionGuessAndLeverArm) {
    MonoScene scene = makeMonoScene();
    ASSERT_GT(scene.points3D.size(), 50u);

    Transform cameraTransform = scene.baseMotion;
    std::map<int, cv::Point3f> words3D = util3d::generateWords3DMono(
        scene.refWords, scene.nextWords, scene.model, cameraTransform, 1.0f);

    ASSERT_FALSE(cameraTransform.isNull());
    EXPECT_NEAR(cameraTransform.x(), scene.baseMotion.x(), 0.005f);
    EXPECT_NEAR(cameraTransform.y(), scene.baseMotion.y(), 0.005f);
    EXPECT_NEAR(cameraTransform.z(), scene.baseMotion.z(), 0.005f);
    EXPECT_NEAR(cameraTransform.theta(), scene.baseMotion.theta(), 0.005f);
    expectWordsInBaseFrame(scene, words3D);
}

// Scale estimated from 3D guess points without motion guess (OdometryMono initialization).
TEST(Util3dFeaturesTest, GenerateWords3DMonoWithPointsGuessAndLeverArm) {
    MonoScene scene = makeMonoScene();
    ASSERT_GT(scene.points3D.size(), 50u);

    Transform cameraTransform;
    double variance = -1.0;
    std::map<int, cv::Point3f> words3D = util3d::generateWords3DMono(
        scene.refWords, scene.nextWords, scene.model, cameraTransform, 1.0f, 0.99f, 4,
        scene.points3D, &variance);

    ASSERT_FALSE(cameraTransform.isNull());
    EXPECT_NEAR(cameraTransform.x(), scene.baseMotion.x(), 0.005f);
    EXPECT_NEAR(cameraTransform.y(), scene.baseMotion.y(), 0.005f);
    EXPECT_NEAR(cameraTransform.z(), scene.baseMotion.z(), 0.005f);
    EXPECT_LT(variance, 1e-4);
    expectWordsInBaseFrame(scene, words3D);
}

// Variance against 3D guess points with motion guess (RegistrationVis
// epipolar estimation, compared to Vis/EpipolarGeometryVar).
TEST(Util3dFeaturesTest, GenerateWords3DMonoVarianceWithLeverArm) {
    MonoScene scene = makeMonoScene();
    ASSERT_GT(scene.points3D.size(), 50u);

    Transform cameraTransform = scene.baseMotion;
    double variance = -1.0;
    std::map<int, cv::Point3f> words3D = util3d::generateWords3DMono(
        scene.refWords, scene.nextWords, scene.model, cameraTransform, 1.0f, 0.99f, 4,
        scene.points3D, &variance);

    EXPECT_GE(variance, 0.0);
    EXPECT_LT(variance, 1e-4);
    expectWordsInBaseFrame(scene, words3D);
}
