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

#include <rtabmap/core/ImuMotionPredictor.h>

#include <vector>

namespace rtabmap {

// Samples older than this (s) behind the newest one are dropped, so the buffer stays
// bounded while no odometry pose comes to trim it.
static const double kMaxBufferDuration = 10.0;

ImuMotionPredictor::ImuMotionPredictor(double maxPoseInterval, double velocityWindow, double gravity) :
	maxPoseInterval_(maxPoseInterval),
	velocityWindow_(velocityWindow),
	gravity_(gravity),
	integratedStartIsFinal_(false),
	poseStamp_(0.0),
	velocity_(Eigen::Vector3d::Zero()),
	worldToOdom_(Eigen::Quaterniond::Identity())
{
}

void ImuMotionPredictor::addImu(double stamp, const IMU & imu)
{
	const cv::Vec4d & o = imu.orientation();
	const Eigen::Quaterniond imuOrientation(o[3], o[0], o[1], o[2]);
	if(imu.empty() ||
	   imuOrientation.norm() < 0.5 ||
	   (!imu.orientationCovariance().empty() && imu.orientationCovariance().at<double>(0,0) == -1.0))
	{
		// No orientation
		return;
	}
	const Eigen::Quaterniond worldToImu = imuOrientation.normalized();
	const Eigen::Quaterniond baseToImu = imu.localTransform().isNull()?
			Eigen::Quaterniond::Identity():
			imu.localTransform().getQuaterniond();

	const cv::Vec3d & f = imu.linearAcceleration();
	Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
	if((f[0] != 0.0 || f[1] != 0.0 || f[2] != 0.0) &&
	   (imu.linearAccelerationCovariance().empty() || imu.linearAccelerationCovariance().at<double>(0,0) != -1.0))
	{
		// The specific force includes the reaction to gravity, up in the world frame
		acceleration = worldToImu * Eigen::Vector3d(f[0], f[1], f[2]) - Eigen::Vector3d(0, 0, gravity_);
	}
	addSample(stamp, worldToImu * baseToImu.inverse(), acceleration);
}

void ImuMotionPredictor::addSample(double stamp, const Eigen::Quaterniond & orientation, const Eigen::Vector3d & acceleration)
{
	Sample sample;
	sample.orientation = orientation.normalized();
	sample.acceleration = acceleration;
	samples_[stamp] = sample;
	if(!integrated_.empty() && stamp <= integrated_.rbegin()->first)
	{
		// Out of order: what was integrated after it changes
		integrated_.clear();
	}
	while(samples_.size() > 2 && samples_.begin()->first < stamp - kMaxBufferDuration)
	{
		samples_.erase(samples_.begin());
		integrated_.clear();
	}
}

void ImuMotionPredictor::addPose(double stamp, const rtabmap::Transform & pose)
{
	integrated_.clear();
	if(pose.isNull())
	{
		pose_.setNull();
		poseStamp_ = 0.0;
		velocity_.setZero();
		poses_.clear();
		return;
	}

	Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
	Eigen::Quaterniond worldToOdom = Eigen::Quaterniond::Identity();
	if(!samples_.empty())
	{
		// The odometry and the IMU agree on the orientation of the base at that time,
		// whatever the yaw of their frames.
		worldToOdom = (pose.getQuaterniond() * orientationAt(stamp).inverse()).normalized();
	}

	// The velocity is estimated from the displacement since a previous pose. Not the
	// last one: odometry's own noise, divided by a frame interval, would be of the order of
	// the velocity itself, and since the prediction deskews the next scans, that noise would
	// feed back into the next poses. The newest pose at least velocityWindow_ old is used
	// instead (or the oldest kept if there is none yet), not older than maxPoseInterval_.
	poses_.erase(poses_.lower_bound(stamp), poses_.end());
	poses_.erase(poses_.begin(), poses_.lower_bound(stamp - maxPoseInterval_));
	std::map<double, rtabmap::Transform>::const_iterator reference = poses_.begin();
	for(std::map<double, rtabmap::Transform>::const_iterator iter = poses_.begin();
		iter != poses_.end() && iter->first <= stamp - velocityWindow_; ++iter)
	{
		reference = iter;
	}
	if(reference != poses_.end())
	{
		const double interval = stamp - reference->first;
		const Eigen::Vector3d displacement(
				pose.x() - reference->second.x(),
				pose.y() - reference->second.y(),
				pose.z() - reference->second.z());

		if(velocityWindow_ > 0.0 &&
		   !samples_.empty() &&
		   samples_.begin()->first <= reference->first &&
		   samples_.rbegin()->first >= stamp)
		{
			// displacement = v0*T + D, with D the double integral of the acceleration
			// over the interval: solve for v0, the velocity at the reference pose, then
			// carry it to this pose with the single integral V.
			Eigen::Vector3d deltaVelocity;
			Eigen::Vector3d deltaPosition;
			integrate(reference->first, stamp, worldToOdom, deltaVelocity, deltaPosition);
			velocity = (displacement - deltaPosition) / interval + deltaVelocity;
		}
		else
		{
			// Average velocity over the interval (no window, or the samples don't cover it)
			velocity = displacement / interval;
		}
	}

	pose_ = pose;
	poseStamp_ = stamp;
	velocity_ = velocity;
	worldToOdom_ = worldToOdom;
	poses_[stamp] = pose;

	// The next velocities are estimated from the poses kept: keep the samples from the
	// oldest one, including the last sample before it to interpolate at its stamp.
	std::map<double, Sample>::iterator iter = samples_.upper_bound(poses_.begin()->first);
	if(iter != samples_.begin())
	{
		--iter;
		samples_.erase(samples_.begin(), iter);
	}
}

void ImuMotionPredictor::reset()
{
	samples_.clear();
	poses_.clear();
	integrated_.clear();
	pose_.setNull();
	poseStamp_ = 0.0;
	velocity_.setZero();
	worldToOdom_.setIdentity();
}

rtabmap::Transform ImuMotionPredictor::predict(double stamp) const
{
	if(samples_.empty())
	{
		return rtabmap::Transform();
	}
	if(pose_.isNull())
	{
		// No pose yet (or lost): only the orientation is known, in the IMU's world frame.
		const Eigen::Quaterniond orientation = orientationAt(stamp);
		return rtabmap::Transform(0, 0, 0, orientation.x(), orientation.y(), orientation.z(), orientation.w());
	}

	const Eigen::Quaterniond orientation = (worldToOdom_ * orientationAt(stamp)).normalized();

	// With t0 the stamp of the last pose, p0 its position and v0 the velocity there, and
	// a(u) the acceleration (gravity removed, in the odometry frame), the position at t is:
	//
	//   p(t) = p0 + v0*(t-t0) + D(t),  with D(t) = integral_t0^t integral_t0^s a(u) du ds
	//
	// D(t) is the displacement due to the change of velocity since t0, V(s) = integral_t0^s a(u) du.
	Eigen::Vector3d position(pose_.x(), pose_.y(), pose_.z()); // p0
	position += velocity_ * (stamp - poseStamp_);               // + v0*(t-t0)
	if(velocityWindow_ <= 0.0)
	{
		// No acceleration without a velocity window: constant velocity
	}
	else if(stamp >= poseStamp_)
	{
		// + D(t). D and V are kept at every sample stamp since t0 (integrated_), so only
		// the part from the last sample ti <= t is integrated here, with dt = t-ti:
		//
		//   D(t) = D(ti) + V(ti)*dt + integral_ti^t integral_ti^s a(u) du ds
		//
		// Between samples the acceleration is linear, from a(ti) to a(t), for which that
		// last double integral is exactly dt^2*(2*a(ti) + a(t))/6. Those are the same
		// segments integrate() would go through, without redoing all those before ti.
		updateIntegration();
		std::map<double, Integrated>::const_iterator from = integrated_.upper_bound(stamp);
		--from; // ti: the first entry is at t0, so there is one
		const double dt = stamp - from->first;
		const Eigen::Vector3d accelerationB = worldToOdom_ * accelerationAt(stamp); // a(t)
		position += from->second.position +                       // D(ti)
				from->second.velocity * dt +                          // V(ti)*dt
				dt * dt * (2.0 * from->second.acceleration + accelerationB) / 6.0; // a(ti) to a(t)
	}
	else
	{
		// + D(t), backward from t0: only for points stamped before the last pose, rare
		Eigen::Vector3d deltaVelocity;
		Eigen::Vector3d deltaPosition;
		integrate(poseStamp_, stamp, worldToOdom_, deltaVelocity, deltaPosition);
		position += deltaPosition;
	}

	return rtabmap::Transform(position.x(), position.y(), position.z(),
			orientation.x(), orientation.y(), orientation.z(), orientation.w());
}

bool ImuMotionPredictor::hasPose() const
{
	return !pose_.isNull();
}

double ImuMotionPredictor::lastPoseStamp() const
{
	return poseStamp_;
}

Eigen::Vector3d ImuMotionPredictor::velocity() const
{
	return velocity_;
}

size_t ImuMotionPredictor::samples() const
{
	return samples_.size();
}

// samples_ must not be empty for the four below.

void ImuMotionPredictor::updateIntegration() const
{
	if(!integrated_.empty() && !integratedStartIsFinal_ && samples_.rbegin()->first >= poseStamp_)
	{
		// The acceleration at the pose was held from the newest sample, which is no
		// longer the newest
		integrated_.clear();
	}
	if(integrated_.empty())
	{
		Integrated start;
		start.acceleration = worldToOdom_ * accelerationAt(poseStamp_);
		start.velocity.setZero();
		start.position.setZero();
		integrated_[poseStamp_] = start;
		integratedStartIsFinal_ = samples_.rbegin()->first >= poseStamp_;
	}
	// Extend to the samples received since
	for(std::map<double, Sample>::const_iterator iter = samples_.upper_bound(integrated_.rbegin()->first);
		iter != samples_.end(); ++iter)
	{
		const std::pair<const double, Integrated> & previous = *integrated_.rbegin();
		const double dt = iter->first - previous.first;
		Integrated next;
		// Same segment as in integrate(), from the previous sample ti to this one:
		// D(ti+1) = D(ti) + V(ti)*dt + dt^2*(2*a(ti) + a(ti+1))/6, V(ti+1) = V(ti) + dt*(a(ti) + a(ti+1))/2
		next.acceleration = worldToOdom_ * iter->second.acceleration;
		next.position = previous.second.position + previous.second.velocity * dt +
				dt * dt * (2.0 * previous.second.acceleration + next.acceleration) / 6.0;
		next.velocity = previous.second.velocity + dt * (previous.second.acceleration + next.acceleration) / 2.0;
		integrated_.insert(integrated_.end(), std::make_pair(iter->first, next));
	}
}

Eigen::Quaterniond ImuMotionPredictor::orientationAt(double stamp) const
{
	std::map<double, Sample>::const_iterator after = samples_.lower_bound(stamp);
	if(after == samples_.end())
	{
		return samples_.rbegin()->second.orientation;
	}
	if(after == samples_.begin() || after->first == stamp)
	{
		return after->second.orientation;
	}
	std::map<double, Sample>::const_iterator before = std::prev(after);
	const double ratio = (stamp - before->first) / (after->first - before->first);
	return before->second.orientation.slerp(ratio, after->second.orientation);
}

Eigen::Vector3d ImuMotionPredictor::accelerationAt(double stamp) const
{
	std::map<double, Sample>::const_iterator after = samples_.lower_bound(stamp);
	if(after == samples_.end())
	{
		return samples_.rbegin()->second.acceleration;
	}
	if(after == samples_.begin() || after->first == stamp)
	{
		return after->second.acceleration;
	}
	std::map<double, Sample>::const_iterator before = std::prev(after);
	const double ratio = (stamp - before->first) / (after->first - before->first);
	return before->second.acceleration + ratio * (after->second.acceleration - before->second.acceleration);
}

void ImuMotionPredictor::integrate(double from, double to, const Eigen::Quaterniond & rotation,
		Eigen::Vector3d & velocity, Eigen::Vector3d & position) const
{
	velocity.setZero();
	position.setZero();
	if(from == to)
	{
		return;
	}

	// Breakpoints: the bounds and every sample in between, in the direction of
	// integration (backward if "to" is before "from"). Between two of them the
	// acceleration is linear, which the segment update below integrates exactly; before
	// the first sample and after the last one, it is held constant.
	std::vector<double> stamps;
	stamps.push_back(from);
	if(from < to)
	{
		for(std::map<double, Sample>::const_iterator iter = samples_.upper_bound(from);
			iter != samples_.end() && iter->first < to; ++iter)
		{
			stamps.push_back(iter->first);
		}
	}
	else
	{
		std::map<double, Sample>::const_iterator iter = samples_.lower_bound(from);
		while(iter != samples_.begin())
		{
			--iter;
			if(iter->first <= to)
			{
				break;
			}
			stamps.push_back(iter->first);
		}
	}
	stamps.push_back(to);

	// Returned: with a(u) the acceleration rotated by "rotation", from "from" (t0) to "to" (t),
	//
	//   velocity = V(t) = integral_t0^t a(u) du
	//   position = D(t) = integral_t0^t integral_t0^s a(u) du ds
	//
	// that is, the change of velocity and the displacement it causes (from a velocity null
	// at t0: the caller adds v0*(t-t0)). They are accumulated breakpoint by breakpoint: from
	// ti to the next one ti+1, with dt = ti+1 - ti and a(u) linear from a(ti) to a(ti+1),
	//
	//   D(ti+1) = D(ti) + V(ti)*dt + dt^2*(2*a(ti) + a(ti+1))/6
	//   V(ti+1) = V(ti) + dt*(a(ti) + a(ti+1))/2
	//
	// both exact for a linear acceleration (dt is negative backward, the same formulas hold).
	Eigen::Vector3d accelerationA = rotation * accelerationAt(stamps[0]); // a(t0)
	for(size_t i=1; i<stamps.size(); ++i)
	{
		const double dt = stamps[i] - stamps[i-1];
		const Eigen::Vector3d accelerationB = rotation * accelerationAt(stamps[i]); // a(ti+1)
		position += velocity * dt +                                     // V(ti)*dt
				dt * dt * (2.0 * accelerationA + accelerationB) / 6.0;  // a(ti) to a(ti+1)
		velocity += dt * (accelerationA + accelerationB) / 2.0;         // trapezoid of a
		accelerationA = accelerationB;
	}
}

}
