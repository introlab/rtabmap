#include <gtest/gtest.h>
#include <rtabmap/core/Transform.h>
#include <cmath>

using namespace rtabmap;

// Helper for floating point comparison
static bool approxEqual(float a, float b, float epsilon = 1e-5f)
{
    return std::fabs(a - b) < epsilon;
}

// ---------- TEST CASES ----------

TEST(TransformTests, DefaultConstructor)
{
    Transform t;
    EXPECT_TRUE(t.isNull());
    EXPECT_FALSE(t.isIdentity());
}

TEST(TransformTests, Identity)
{
    Transform t = Transform::getIdentity();
    EXPECT_TRUE(t.isIdentity());
    EXPECT_FALSE(t.isNull());

    // Identity * Identity = Identity
    Transform result = t * Transform::getIdentity();
    EXPECT_TRUE(result.isIdentity());
}

TEST(TransformTests, EqualityOperators)
{
    Transform t1 = Transform::getIdentity();
    Transform t2 = Transform::getIdentity();

    EXPECT_TRUE(t1 == t2);
    EXPECT_FALSE(t1 != t2);

    t2.x() = 1.0f;
    EXPECT_FALSE(t1 == t2);
    EXPECT_TRUE(t1 != t2);
}

TEST(TransformTests, TranslationConstructor)
{
    Transform t(1.0f, 2.0f, 3.0f, 0, 0, 0);
    EXPECT_TRUE(approxEqual(t.x(), 1.0f));
    EXPECT_TRUE(approxEqual(t.y(), 2.0f));
    EXPECT_TRUE(approxEqual(t.z(), 3.0f));
}

TEST(TransformTests, MatrixConstructor)
{
    cv::Mat mat = (cv::Mat_<float>(3, 4) <<
        1, 0, 0, 1,
        0, 1, 0, 2,
        0, 0, 1, 3);
    Transform t(mat);
    EXPECT_TRUE(approxEqual(t.x(), 1.0f));
    EXPECT_TRUE(approxEqual(t.y(), 2.0f));
    EXPECT_TRUE(approxEqual(t.z(), 3.0f));
}

TEST(TransformTests, Inverse)
{
    Transform t(1, 0, 0, 1,
                0, 1, 0, 2,
                0, 0, 1, 3);
    Transform inv = t.inverse();
    Transform id = t * inv;
    EXPECT_TRUE(id.isIdentity());
}

TEST(TransformTests, Interpolation)
{
    Transform t1(0, 0, 0, 0, 0, 0);
    Transform t2(0, 0, 1, 0, 0, 0);
    Transform mid = t1.interpolate(0.5f, t2);
    EXPECT_TRUE(approxEqual(mid.z(), 0.5f));
}

TEST(TransformTests, NormAndDistance)
{
    Transform t1(0, 0, 0, 0, 0, 0);
    Transform t2(1, 2, 2, 0, 0, 0);

    EXPECT_TRUE(approxEqual(t2.getNorm(), 3.0f));
    EXPECT_TRUE(approxEqual(t1.getDistance(t2), 3.0f));
    EXPECT_TRUE(approxEqual(t1.getDistanceSquared(t2), 9.0f));
}

TEST(TransformTests, EulerAndQuaternionConversion)
{
    Transform t(1, 2, 3, 0.1f, 0.2f, 0.3f);
    float roll, pitch, yaw;
    t.getEulerAngles(roll, pitch, yaw);
    EXPECT_TRUE(std::isfinite(roll));
    EXPECT_TRUE(std::isfinite(pitch));
    EXPECT_TRUE(std::isfinite(yaw));

    Eigen::Quaternionf q = t.getQuaternionf();
    EXPECT_TRUE(std::isfinite(q.x()));
    EXPECT_TRUE(std::isfinite(q.y()));
    EXPECT_TRUE(std::isfinite(q.z()));
    EXPECT_TRUE(std::isfinite(q.w()));
}

TEST(TransformTests, StringConversion)
{
    Transform original(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f);
    Transform parsed = Transform::fromString("1.0 2.0 3.0 0.1 0.2 0.3");
    EXPECT_TRUE(original.getDistance(parsed) < 1e-4f);
}

TEST(TransformTests, DoFConversion)
{
    // 6DoF transform
    Transform t6(1, 2, 3, 0.1f, 0.2f, 0.3f);

    Transform t3dof = t6.to3DoF();
    EXPECT_TRUE(t3dof.is3DoF());
    EXPECT_TRUE(t3dof.is4DoF());
    EXPECT_TRUE(approxEqual(t3dof.z(), 0.0f)); // Z is removed
    EXPECT_TRUE(approxEqual(t3dof.r31(), 0.0f));
    EXPECT_TRUE(approxEqual(t3dof.r32(), 0.0f));
    EXPECT_TRUE(approxEqual(t3dof.r33(), 1.0f));

    Transform t4dof = t6.to4DoF();
    EXPECT_TRUE(t4dof.is4DoF());
    EXPECT_FALSE(t4dof.is3DoF());
    EXPECT_TRUE(approxEqual(t4dof.r31(), 0.0f));
    EXPECT_TRUE(approxEqual(t4dof.r32(), 0.0f));
    EXPECT_TRUE(approxEqual(t4dof.r33(), 1.0f));
}

TEST(TransformTests, OpenGLToRTABMap)
{
    // Multiplying should return identity
    Transform result = Transform::opengl_T_rtabmap() * Transform::rtabmap_T_opengl();
    EXPECT_TRUE(result.isIdentity());

    // Test conversion from opengl world to rtabmap world
    EXPECT_NEAR((Transform::rtabmap_T_opengl() * Transform(1,0,0,0,0,0)).y(), -1, 1e-5);
    EXPECT_NEAR((Transform::rtabmap_T_opengl() * Transform(0,1,0,0,0,0)).z(),  1, 1e-5);
    EXPECT_NEAR((Transform::rtabmap_T_opengl() * Transform(0,0,1,0,0,0)).x(), -1, 1e-5);

    // Test conversion from rtabmap world to opengl world
    EXPECT_NEAR((Transform::opengl_T_rtabmap() * Transform(1,0,0,0,0,0)).z(), -1, 1e-5);
    EXPECT_NEAR((Transform::opengl_T_rtabmap() * Transform(0,1,0,0,0,0)).x(), -1, 1e-5);
    EXPECT_NEAR((Transform::opengl_T_rtabmap() * Transform(0,0,1,0,0,0)).y(),  1, 1e-5);
}

TEST(TransformTests, EigenConversions)
{
    Transform t(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f);

    // Eigen 4x4 float
    Eigen::Matrix4f e4f = t.toEigen4f();
    Transform fromE4f = Transform::fromEigen4f(e4f);
    EXPECT_LT(t.getDistance(fromE4f), 1e-4f);

    // Eigen 4x4 double
    Eigen::Matrix4d e4d = t.toEigen4d();
    Transform fromE4d = Transform::fromEigen4d(e4d);
    EXPECT_LT(t.getDistance(fromE4d), 1e-4f);

    // Eigen Affine3f
    Eigen::Affine3f aff3f = t.toEigen3f();
    Transform fromAff3f = Transform::fromEigen3f(aff3f);
    EXPECT_LT(t.getDistance(fromAff3f), 1e-4f);

    // Eigen Affine3d
    Eigen::Affine3d aff3d = t.toEigen3d();
    Transform fromAff3d = Transform::fromEigen3d(aff3d);
    EXPECT_LT(t.getDistance(fromAff3d), 1e-4f);

    // Quaternion check
    Eigen::Quaternionf qf = t.getQuaternionf();
    EXPECT_NEAR(qf.norm(), 1.0f, 1e-5f);

    Eigen::Quaterniond qd = t.getQuaterniond();
    EXPECT_NEAR(qd.norm(), 1.0, 1e-5);
}

TEST(TransformTests, GetTransformFromBuffer)
{
    std::map<double, Transform> tfBuffer;

    tfBuffer[1.0] = Transform(0, 0, 0, 0, 0, 0);         // t=1.0
    tfBuffer[2.0] = Transform(1, 0, 0, 0, 0, M_PI/2.0);  // t=2.0
    tfBuffer[3.0] = Transform(2, 0, 0, 0, 0, M_PI);      // t=3.0

    // Test exact timestamp
    Transform tExact = Transform::getTransform(tfBuffer, 2.0);
    EXPECT_NEAR(tExact.x(), 1.0f, 1e-5);

    // Test below
    Transform tBelow = Transform::getTransform(tfBuffer, 1.5);
    EXPECT_NEAR(tBelow.x(), 0.5f, 1e-5);
    EXPECT_NEAR(tBelow.theta(), M_PI/4.0, 1e-5);

    // Test above
    Transform tAbove = Transform::getTransform(tfBuffer, 2.5);
    EXPECT_NEAR(tAbove.x(), 1.5f, 1e-5);
    EXPECT_NEAR(tAbove.theta(), 3.0*M_PI/4.0, 1e-5);

    // Test very close to a border
    Transform tNear = Transform::getTransform(tfBuffer, 2.0001);
    EXPECT_NEAR(tNear.x(), 1.0001f, 1e-4);

    // Requesting transform in the past
    Transform tPast = Transform::getTransform(tfBuffer, 0.5);
    EXPECT_TRUE(tPast.isNull());

    // Requesting transform in the future
    Transform tFuture = Transform::getTransform(tfBuffer, 3.5);
    EXPECT_TRUE(tFuture.isNull());
}