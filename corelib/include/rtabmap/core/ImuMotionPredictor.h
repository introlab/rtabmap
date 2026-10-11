/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef RTABMAP_CORE_IMUMOTIONPREDICTOR_H_
#define RTABMAP_CORE_IMUMOTIONPREDICTOR_H_

#include <rtabmap/core/rtabmap_core_export.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/IMU.h>

#include <Eigen/Geometry>

#include <map>

namespace rtabmap {

/**
 * @brief Predicts the pose of the base frame from the last odometry pose and the IMU.
 *
 * Between two odometry updates, the pose is propagated with the IMU: the orientation is
 * the IMU's own, re-expressed in the odometry frame, and the position is integrated from
 * the velocity at the last odometry update and the gravity-compensated acceleration. That
 * velocity is re-estimated at every odometry update, from the displacement since an
 * odometry pose a short window back (see velocityWindow) corrected by the acceleration
 * measured in between, so the position never drifts for long: only the motion since the
 * last update is predicted.
 *
 * This is what lidar deskewing needs: the pose at every point's time, during a sweep that
 * started after the last pose odometry estimated.
 *
 * The IMU samples are given as they are measured (see addImu()):
 * the orientation of the IMU in its world frame and its specific force, from which gravity
 * is removed here. That world frame must be gravity aligned with +z up, as in ROS (REP-103,
 * e.g. ENU); its yaw doesn't matter. An orientation given in a frame with z down (NED)
 * must be converted first, otherwise gravity is added instead of removed. The lever arm between the IMU and the base origin is
 * ignored: its centripetal and tangential accelerations are small over the fraction of a
 * second this predicts.
 *
 * Not thread-safe: a caller sharing it between threads must lock around every call.
 */
class RTABMAP_CORE_EXPORT ImuMotionPredictor
{
public:
	/**
	 * The IMU acceleration is used only with a velocity window (> 0): over a single frame,
	 * the velocity is too noisy to be carried forward with it. Without it (window of 0, or
	 * no acceleration in the IMU samples), the position follows the last velocity
	 * (constant velocity model).
	 *
	 * @param maxPoseInterval odometry poses older (s) than this are not used to estimate
	 *                        the velocity, which is null without one
	 * @param velocityWindow  the velocity is estimated from the displacement since the
	 *                        newest pose at least this old (s), corrected by the
	 *                        acceleration measured since, so that it is the velocity at the
	 *                        last pose, not an average. Over a single frame interval, the
	 *                        noise of the odometry poses would be of the order of the
	 *                        velocity itself. 0: the displacement since the previous pose,
	 *                        without acceleration.
	 * @param gravity         magnitude (m/s^2) of the gravity removed from the specific force
	 *                        given to addImu(), standard gravity by default
	 *
	 * These are fixed for the life of the predictor (there is no setter): changing them
	 * while it estimates would mix poses and samples taken under different settings.
	 */
	explicit ImuMotionPredictor(
			double maxPoseInterval = 1.0,
			double velocityWindow = 0.5,
			double gravity = 9.80665);

	double maxPoseInterval() const {return maxPoseInterval_;}
	double velocityWindow() const {return velocityWindow_;}
	double gravity() const {return gravity_;}

	/**
	 * @brief Adds an IMU measurement.
	 * @param stamp time of the measurement (s)
	 * @param imu   orientation of the IMU in its world frame (gravity aligned, +z up), linear
	 *              acceleration as measured (the specific force, which includes the
	 *              reaction to gravity) and the transform from the base frame to the IMU.
	 *              Without orientation, the measurement is ignored; without linear
	 *              acceleration (all zeros, or a covariance of -1), only its orientation
	 *              is used.
	 */
	void addImu(double stamp, const IMU & imu);

	/**
	 * @brief Adds an odometry pose, from which the next poses are predicted.
	 * @param stamp time of the pose (s)
	 * @param pose  pose of the base frame in the odometry frame; a null pose (odometry
	 *              lost) resets the prediction until the next valid pose
	 */
	void addPose(double stamp, const rtabmap::Transform & pose);

	/// Forgets the poses and the IMU samples.
	void reset();

	/**
	 * @brief Predicts the pose of the base frame in the odometry frame.
	 * @param stamp time (s) of the prediction, normally after the last pose
	 * @return the predicted pose, null if there is no IMU sample yet. Without a pose (none
	 *         yet, or the last one was null), the orientation alone is predicted, in the
	 *         IMU's world frame and at the origin: still the relative rotation between two
	 *         stamps, which is what deskewing needs most.
	 */
	rtabmap::Transform predict(double stamp) const;

	/// Whether there is a pose to predict from (none yet, or the last one was null).
	bool hasPose() const;
	/// Stamp of the last pose added, 0 if there is none.
	double lastPoseStamp() const;
	/// Velocity (m/s) of the base in the odometry frame at the last pose.
	Eigen::Vector3d velocity() const;
	/// Number of IMU samples kept.
	size_t samples() const;

private:
	// orientation: of the base frame in the IMU's world frame; acceleration: of the base in
	// that frame, gravity removed
	void addSample(double stamp, const Eigen::Quaterniond & orientation, const Eigen::Vector3d & acceleration);

	struct Sample
	{
		Eigen::Quaterniond orientation;
		Eigen::Vector3d acceleration;
	};
	Eigen::Quaterniond orientationAt(double stamp) const;
	Eigen::Vector3d accelerationAt(double stamp) const;
	void integrate(double from, double to, const Eigen::Quaterniond & rotation,
			Eigen::Vector3d & velocity, Eigen::Vector3d & position) const;
	void updateIntegration() const;

	// The acceleration integrated from the last pose up to a sample's stamp, in the
	// odometry frame: predicting a stamp then only integrates from the sample before it.
	struct Integrated
	{
		Eigen::Vector3d acceleration; // at that stamp
		Eigen::Vector3d velocity;     // change since the last pose
		Eigen::Vector3d position;     // change since the last pose (without its velocity)
	};

private:
	double maxPoseInterval_;
	double velocityWindow_;
	double gravity_;
	std::map<double, Sample> samples_;
	// Recent odometry poses, to estimate the velocity from (see velocityWindow)
	std::map<double, rtabmap::Transform> poses_;

	// The last odometry pose and the state predictions start from.
	double poseStamp_;
	rtabmap::Transform pose_;
	Eigen::Vector3d velocity_;
	// Rotation from the IMU's world frame to the odometry frame, at the last pose: the two
	// are both gravity aligned, but their yaw differ.
	Eigen::Quaterniond worldToOdom_;

	// Built lazily by predict(), from the last pose to the newest sample; cleared when the
	// pose or the samples it was built from change.
	mutable std::map<double, Integrated> integrated_;
	// Whether the acceleration at the last pose's stamp (the first entry) is final: it is
	// held constant from the newest sample until a sample after that stamp is received.
	mutable bool integratedStartIsFinal_;
};

}

#endif /* RTABMAP_CORE_IMUMOTIONPREDICTOR_H_ */
