#include <gtest/gtest.h>

#include <cmath>
#include <algorithm>
#include <functional>

#include <rtabmap/core/ImuMotionPredictor.h>
#include <rtabmap/core/IMU.h>

using rtabmap::ImuMotionPredictor;

namespace {

Eigen::Quaterniond yaw(double angle)
{
	return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
}

rtabmap::Transform pose(const Eigen::Vector3d & position, double angle)
{
	return rtabmap::Transform(position.x(), position.y(), position.z(), 0, 0, angle);
}

/// An IMU at rest or accelerating, as measured: orientation of the IMU in its world
/// frame, and the specific force (acceleration minus gravity) in the IMU frame.
rtabmap::IMU measuredImu(const Eigen::Quaterniond & worldToImu,
		const Eigen::Vector3d & accelerationInWorld, double gravity,
		const rtabmap::Transform & baseToImu)
{
	const Eigen::Vector3d f = worldToImu.inverse() * (accelerationInWorld + Eigen::Vector3d(0, 0, gravity));
	const Eigen::Quaterniond q = worldToImu.normalized();
	return rtabmap::IMU(cv::Vec4d(q.x(), q.y(), q.z(), q.w()), cv::Mat::eye(3,3,CV_64FC1),
			cv::Vec3d(0,0,0), cv::Mat::eye(3,3,CV_64FC1),
			cv::Vec3d(f.x(), f.y(), f.z()), cv::Mat::eye(3,3,CV_64FC1),
			baseToImu);
}

/// IMU measurements at 200 Hz over [from, to] of an IMU at the base origin, oriented and
/// accelerating (in its world frame) as given. Without accelerometer, it measures no
/// linear acceleration at all.
void addSamples(ImuMotionPredictor & predictor, double from, double to,
		const std::function<Eigen::Quaterniond(double)> & orientation,
		const std::function<Eigen::Vector3d(double)> & acceleration,
		bool withAccelerometer = true)
{
	for(int i=0; from + i*0.005 <= to + 1e-9; ++i)
	{
		const double t = from + i*0.005;
		rtabmap::IMU imu = measuredImu(orientation(t), acceleration(t), predictor.gravity(), rtabmap::Transform::getIdentity());
		if(!withAccelerometer)
		{
			imu = rtabmap::IMU(imu.orientation(), imu.orientationCovariance(),
					imu.angularVelocity(), imu.angularVelocityCovariance(),
					cv::Vec3d(0,0,0), imu.linearAccelerationCovariance(),
					imu.localTransform());
		}
		predictor.addImu(t, imu);
	}
}

}  // namespace

TEST(ImuMotionPredictor, predicts_only_the_orientation_without_a_pose)
{
	ImuMotionPredictor predictor;
	EXPECT_TRUE(predictor.predict(1.0).isNull());

	predictor.addImu(1.0, measuredImu(yaw(0.3), Eigen::Vector3d(1, 0, 0), 9.80665, rtabmap::Transform::getIdentity()));
	predictor.addImu(1.1, measuredImu(yaw(0.5), Eigen::Vector3d(1, 0, 0), 9.80665, rtabmap::Transform::getIdentity()));
	const rtabmap::Transform predicted = predictor.predict(1.05);
	ASSERT_FALSE(predicted.isNull()) << "the orientation is known without a pose";
	EXPECT_NEAR(predicted.theta(), 0.4, 1e-5);
	EXPECT_NEAR(predicted.x(), 0.0, 1e-9) << "no position without a pose";

	ImuMotionPredictor withoutImu;
	withoutImu.addPose(1.0, rtabmap::Transform::getIdentity());
	EXPECT_TRUE(withoutImu.predict(1.0).isNull()) << "no imu yet";
}

TEST(ImuMotionPredictor, follows_a_constant_velocity)
{
	ImuMotionPredictor predictor;
	const Eigen::Vector3d velocity(1.0, -0.5, 0.2);
	addSamples(predictor, 0.0, 0.3,
			[](double) { return Eigen::Quaterniond::Identity(); },
			[](double) { return Eigen::Vector3d::Zero(); });
	predictor.addPose(0.0, pose(Eigen::Vector3d::Zero(), 0));
	EXPECT_TRUE(predictor.velocity().isZero()) << "a single pose has no velocity";
	predictor.addPose(0.1, pose(velocity * 0.1, 0));

	EXPECT_TRUE(predictor.velocity().isApprox(velocity, 1e-6));
	const rtabmap::Transform predicted = predictor.predict(0.25);
	ASSERT_FALSE(predicted.isNull());
	EXPECT_NEAR(predicted.x(), velocity.x() * 0.25, 1e-6);
	EXPECT_NEAR(predicted.y(), velocity.y() * 0.25, 1e-6);
	EXPECT_NEAR(predicted.z(), velocity.z() * 0.25, 1e-6);
}

TEST(ImuMotionPredictor, integrates_the_acceleration)
{
	// From rest at t=0 with a constant 2 m/s^2: p = t^2, v = 2t. The velocity at the
	// second pose is the instantaneous one, not the average over the interval, and the
	// prediction keeps accelerating. An IMU without accelerometer gives a constant
	// velocity instead.
	const double a = 2.0;
	for(bool withAccelerometer : {true, false})
	{
		ImuMotionPredictor predictor;
		addSamples(predictor, -0.05, 0.3,
				[](double) { return Eigen::Quaterniond::Identity(); },
				[&](double t) { return Eigen::Vector3d(t < 0.0 ? 0.0 : a, 0, 0); },
				withAccelerometer);
		predictor.addPose(0.0, pose(Eigen::Vector3d::Zero(), 0));
		predictor.addPose(0.1, pose(Eigen::Vector3d(0.5*a*0.01, 0, 0), 0));

		const double t = 0.2;
		const rtabmap::Transform predicted = predictor.predict(t);
		ASSERT_FALSE(predicted.isNull());
		if(withAccelerometer)
		{
			EXPECT_NEAR(predictor.velocity().x(), a*0.1, 1e-6);
			EXPECT_NEAR(predicted.x(), 0.5*a*t*t, 1e-6);
		}
		else
		{
			// Constant velocity model: the average velocity over the last interval.
			EXPECT_NEAR(predictor.velocity().x(), 0.5*a*0.1, 1e-6);
			EXPECT_NEAR(predicted.x(), 0.5*a*0.01 + 0.5*a*0.1*(t-0.1), 1e-6);
		}
	}
}

TEST(ImuMotionPredictor, expresses_the_imu_in_the_odometry_frame)
{
	// The IMU's world frame and the odometry frame differ by 90 degrees of yaw. The base
	// turns at 1 rad/s and accelerates along the IMU world's x, which is the odometry's y.
	const double rate = 1.0;
	const double offset = M_PI/2.0;
	const double a = 3.0;
	ImuMotionPredictor predictor;
	addSamples(predictor, 0.0, 0.3,
			[&](double t) { return yaw(rate*t); },
			[&](double) { return Eigen::Vector3d(a, 0, 0); });
	predictor.addPose(0.0, pose(Eigen::Vector3d::Zero(), offset));
	predictor.addPose(0.1, pose(Eigen::Vector3d(0, 0.5*a*0.01, 0), offset + rate*0.1));

	const double t = 0.25;
	const rtabmap::Transform predicted = predictor.predict(t);
	ASSERT_FALSE(predicted.isNull());
	EXPECT_NEAR(predicted.x(), 0.0, 1e-6);
	EXPECT_NEAR(predicted.y(), 0.5*a*t*t, 1e-6);
	EXPECT_NEAR(predicted.theta(), offset + rate*t, 1e-5);
}

TEST(ImuMotionPredictor, a_lost_pose_resets_the_prediction)
{
	ImuMotionPredictor predictor;
	addSamples(predictor, 0.0, 0.5,
			[](double) { return Eigen::Quaterniond::Identity(); },
			[](double) { return Eigen::Vector3d::Zero(); });
	predictor.addPose(0.0, pose(Eigen::Vector3d::Zero(), 0));
	predictor.addPose(0.1, pose(Eigen::Vector3d(0.1, 0, 0), 0));
	ASSERT_FALSE(predictor.velocity().isZero());

	predictor.addPose(0.2, rtabmap::Transform());
	EXPECT_TRUE(predictor.predict(0.25).isIdentity()) << "orientation only, which is constant here";

	// After a reset of the odometry, the pose jumps: no velocity across it.
	predictor.addPose(0.3, pose(Eigen::Vector3d(10, 0, 0), 0));
	EXPECT_TRUE(predictor.velocity().isZero());
	EXPECT_NEAR(predictor.predict(0.4).x(), 10.0, 1e-6);
}

TEST(ImuMotionPredictor, poses_too_far_apart_give_no_velocity)
{
	ImuMotionPredictor predictor(0.5);
	addSamples(predictor, 0.0, 1.5,
			[](double) { return Eigen::Quaterniond::Identity(); },
			[](double) { return Eigen::Vector3d::Zero(); });
	predictor.addPose(0.0, pose(Eigen::Vector3d::Zero(), 0));
	predictor.addPose(1.0, pose(Eigen::Vector3d(1, 0, 0), 0));
	EXPECT_TRUE(predictor.velocity().isZero());
}

TEST(ImuMotionPredictor, a_longer_window_averages_out_the_pose_noise)
{
	// 1 m/s along x, with odometry poses alternating 1 cm on each side of the truth: the
	// worst case for a velocity differenced over one frame, which sees 0.2 m/s of noise.
	for(double window : {0.0, 0.5})
	{
		ImuMotionPredictor predictor(1.0, window);
		addSamples(predictor, 0.0, 2.0,
				[](double) { return Eigen::Quaterniond::Identity(); },
				[](double) { return Eigen::Vector3d::Zero(); });
		double maxError = 0.0;
		for(int i=0; i<=15; ++i)
		{
			const double t = i*0.1;
			predictor.addPose(t, pose(Eigen::Vector3d(t + (i%2?0.01:-0.01), 0, 0), 0));
			if(i >= 10)
			{
				maxError = std::max(maxError, std::fabs(predictor.velocity().x() - 1.0));
			}
		}
		if(window == 0.0)
		{
			EXPECT_NEAR(maxError, 0.2, 1e-6) << "differenced over one frame";
		}
		else
		{
			EXPECT_LT(maxError, 0.05) << "differenced over half a second";
		}
	}
}

TEST(ImuMotionPredictor, keeps_only_the_samples_since_the_last_pose)
{
	ImuMotionPredictor predictor;
	addSamples(predictor, 0.0, 0.2,
			[](double) { return Eigen::Quaterniond::Identity(); },
			[](double) { return Eigen::Vector3d::Zero(); });
	ASSERT_EQ(predictor.samples(), 41u);
	predictor.addPose(0.1025, pose(Eigen::Vector3d::Zero(), 0));
	// 0.100 (the last one before the pose, to interpolate at its stamp) .. 0.200
	EXPECT_EQ(predictor.samples(), 21u);

	predictor.reset();
	EXPECT_EQ(predictor.samples(), 0u);
	EXPECT_TRUE(predictor.predict(0.2).isNull());
}


TEST(ImuMotionPredictor, removes_gravity_from_what_the_imu_measures)
{
	// The IMU is mounted rolled by 90 degrees on a level base at rest: it measures gravity
	// along its own y. Once removed, nothing moves, and the base stays level.
	const rtabmap::Transform baseToImu(0, 0, 0, M_PI/2.0, 0, 0);
	const Eigen::Quaterniond worldToImu = baseToImu.getQuaterniond();
	ImuMotionPredictor predictor;
	for(int i=0; i<=60; ++i)
	{
		predictor.addImu(i*0.005, measuredImu(worldToImu, Eigen::Vector3d::Zero(), 9.80665, baseToImu));
	}
	predictor.addPose(0.0, rtabmap::Transform::getIdentity());
	const rtabmap::Transform predicted = predictor.predict(0.3);
	ASSERT_FALSE(predicted.isNull());
	EXPECT_NEAR(predicted.getNorm(), 0.0, 1e-6) << "gravity was not removed";
	EXPECT_TRUE(predicted.getQuaterniond().isApprox(Eigen::Quaterniond::Identity(), 1e-6)) << "the base orientation is the imu's, unmounted";
}

TEST(ImuMotionPredictor, removes_the_gravity_it_is_given)
{
	// On the Moon, at rest: the IMU measures 1.62 m/s^2 up.
	ImuMotionPredictor predictor(1.0, 0.5, 1.62);
	for(int i=0; i<=60; ++i)
	{
		predictor.addImu(i*0.005, measuredImu(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 1.62,
				rtabmap::Transform::getIdentity()));
	}
	predictor.addPose(0.0, rtabmap::Transform::getIdentity());
	EXPECT_NEAR(predictor.predict(0.3).z(), 0.0, 1e-6);

	// Earth's gravity removed from the same measurement: it looks like falling.
	ImuMotionPredictor earth;
	for(int i=0; i<=60; ++i)
	{
		earth.addImu(i*0.005, measuredImu(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 1.62,
				rtabmap::Transform::getIdentity()));
	}
	earth.addPose(0.0, rtabmap::Transform::getIdentity());
	EXPECT_NEAR(earth.predict(0.2).z(), 0.5*(1.62-9.80665)*0.04, 1e-6);
}

TEST(ImuMotionPredictor, an_imu_without_acceleration_is_not_a_free_fall)
{
	ImuMotionPredictor predictor;
	const Eigen::Quaterniond q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
	for(int i=0; i<=60; ++i)
	{
		predictor.addImu(i*0.005, rtabmap::IMU(cv::Vec4d(q.x(), q.y(), q.z(), q.w()), cv::Mat::eye(3,3,CV_64FC1),
				cv::Vec3d(0,0,0), cv::Mat::eye(3,3,CV_64FC1),
				cv::Vec3d(0,0,0), cv::Mat::eye(3,3,CV_64FC1),
				rtabmap::Transform::getIdentity()));
	}
	predictor.addPose(0.0, rtabmap::Transform::getIdentity());
	EXPECT_NEAR(predictor.predict(0.3).getNorm(), 0.0, 1e-6);

	// And one without orientation is ignored altogether.
	ImuMotionPredictor noOrientation;
	noOrientation.addImu(0.0, rtabmap::IMU(cv::Vec4d(0,0,0,0), cv::Mat(),
			cv::Vec3d(0,0,0), cv::Mat(), cv::Vec3d(0,0,9.8), cv::Mat(), rtabmap::Transform::getIdentity()));
	EXPECT_EQ(noOrientation.samples(), 0u);
}

TEST(ImuMotionPredictor, predicts_the_same_whatever_the_order_samples_and_predictions_come_in)
{
	// The integration since the last pose is kept between predictions: it must not go
	// stale when samples arrive after a prediction, or out of order. Compared with a
	// predictor given every sample before predicting anything.
	auto acceleration = [](double t) { return Eigen::Vector3d(std::sin(20*t), std::cos(15*t), 0.3*t); };
	auto orientation = [](double t) { return yaw(0.5*t); };
	auto imu = [&](double t) { return measuredImu(orientation(t), acceleration(t), 9.80665, rtabmap::Transform::getIdentity()); };

	ImuMotionPredictor reference;
	for(int i=0; i<=80; ++i) reference.addImu(i*0.005, imu(i*0.005));
	reference.addPose(0.0, rtabmap::Transform::getIdentity());
	reference.addPose(0.1, pose(Eigen::Vector3d(0.05, 0, 0), 0.05));

	ImuMotionPredictor incremental;
	for(int i=0; i<=40; ++i) if(i != 30) incremental.addImu(i*0.005, imu(i*0.005));
	incremental.addPose(0.0, rtabmap::Transform::getIdentity());
	incremental.addPose(0.1, pose(Eigen::Vector3d(0.05, 0, 0), 0.05)); // velocity needs up to 0.1: covered
	incremental.predict(0.12);   // integrates without the sample at 0.15
	incremental.predict(0.3);    // beyond the newest sample (0.2)
	incremental.addImu(0.15, imu(0.15));   // out of order
	for(int i=41; i<=80; ++i)
	{
		incremental.addImu(i*0.005, imu(i*0.005));
		if(i % 7 == 0) incremental.predict(i*0.005 - 0.0012);
	}

	for(double t : {0.1, 0.1013, 0.15, 0.2337, 0.4, 0.45})
	{
		const rtabmap::Transform a = reference.predict(t);
		const rtabmap::Transform b = incremental.predict(t);
		ASSERT_FALSE(a.isNull());
		ASSERT_FALSE(b.isNull());
		EXPECT_NEAR(a.x(), b.x(), 1e-6) << "t=" << t;
		EXPECT_NEAR(a.y(), b.y(), 1e-6) << "t=" << t;
		EXPECT_NEAR(a.z(), b.z(), 1e-6) << "t=" << t;
	}
}

TEST(ImuMotionPredictor, holds_the_acceleration_at_the_pose_until_a_newer_sample)
{
	// The pose comes after the newest sample: the acceleration there is held from that
	// sample, until a newer one says otherwise.
	ImuMotionPredictor reference;
	ImuMotionPredictor incremental;
	auto imu = [](double a) { return measuredImu(Eigen::Quaterniond::Identity(), Eigen::Vector3d(a, 0, 0), 9.80665, rtabmap::Transform::getIdentity()); };
	for(ImuMotionPredictor * p : {&reference, &incremental})
	{
		p->addImu(0.0, imu(0.0));
		p->addImu(0.1, imu(0.0));
		p->addPose(0.15, rtabmap::Transform::getIdentity());
	}
	incremental.predict(0.2);
	for(ImuMotionPredictor * p : {&reference, &incremental})
	{
		p->addImu(0.2, imu(4.0));
		p->addImu(0.3, imu(4.0));
	}
	EXPECT_NEAR(reference.predict(0.3).x(), incremental.predict(0.3).x(), 1e-9);
}
