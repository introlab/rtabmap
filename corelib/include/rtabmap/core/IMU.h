/*
Copyright (c) 2010-2018, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef IMU_H_
#define IMU_H_

#include <opencv2/core/core.hpp>
#include <rtabmap/utilite/UEvent.h>
#include <rtabmap/core/Transform.h>

namespace rtabmap {

/**
 * @class IMU
 * @brief Inertial measurement sample (ROS @c sensor_msgs/Imu-like fields).
 *
 * Holds orientation (quaternion), angular velocity, linear acceleration, optional
 * 3×3 row-major covariance matrices (double), and an optional @ref Transform
 * expressing the IMU frame relative to the robot base.
 *
 * @ref empty() is true when @ref localTransform() is null (default constructor).
 * A sample constructed with @ref Transform::getIdentity() is not empty.
 *
 * @ref convertToBaseFrame() rotates linear/angular velocity (and orientation when
 * quaternion x/y/z are non-zero) into the base frame, then clears rotation in
 * @ref localTransform() while keeping translation.
 *
 * @see SensorData::imu()
 * @see IMUEvent
 */
class IMU
{
public:
	/** @brief Default-constructs an empty sample (null @ref localTransform()). */
	IMU() {}

	/**
	 * @brief Constructs a sample with orientation and motion data.
	 * @param orientation Unit quaternion (qx, qy, qz, qw).
	 * @param orientationCovariance 3×3 row-major covariance about x, y, z (empty if unused).
	 * @param angularVelocity Rad/s about x, y, z.
	 * @param angularVelocityCovariance 3×3 row-major covariance (empty if unused).
	 * @param linearAcceleration m/s² about x, y, z.
	 * @param linearAccelerationCovariance 3×3 row-major covariance (empty if unused).
	 * @param localTransform IMU frame in base coordinates (default identity).
	 */
	IMU(const cv::Vec4d & orientation, // qx qy qz qw
		const cv::Mat & orientationCovariance,
		const cv::Vec3d & angularVelocity,
		const cv::Mat & angularVelocityCovariance,
		const cv::Vec3d & linearAcceleration,
		const cv::Mat & linearAccelerationCovariance,
		const Transform & localTransform = Transform::getIdentity()) :
			orientation_(orientation),
			orientationCovariance_(orientationCovariance),
			angularVelocity_(angularVelocity),
			angularVelocityCovariance_(angularVelocityCovariance),
			linearAcceleration_(linearAcceleration),
			linearAccelerationCovariance_(linearAccelerationCovariance),
			localTransform_(localTransform)
	{
	}

	/**
	 * @brief Constructs a sample without orientation (e.g. no magnetometer / no attitude).
	 * @param angularVelocity Rad/s about x, y, z.
	 * @param angularVelocityCovariance 3×3 row-major covariance (empty if unused).
	 * @param linearAcceleration m/s² about x, y, z.
	 * @param linearAccelerationCovariance 3×3 row-major covariance (empty if unused).
	 * @param localTransform IMU frame in base coordinates (default identity).
	 */
	IMU(const cv::Vec3d & angularVelocity,
		const cv::Mat & angularVelocityCovariance,
		const cv::Vec3d & linearAcceleration,
		const cv::Mat & linearAccelerationCovariance,
		const Transform & localTransform = Transform::getIdentity()) :
			angularVelocity_(angularVelocity),
			angularVelocityCovariance_(angularVelocityCovariance),
			linearAcceleration_(linearAcceleration),
			linearAccelerationCovariance_(linearAccelerationCovariance),
			localTransform_(localTransform)
	{
	}

	/** @return Orientation quaternion (qx, qy, qz, qw). */
	const cv::Vec4d & orientation() const {return orientation_;}
	/** @return 3×3 orientation covariance (row-major, empty if orientation unset). */
	const cv::Mat & orientationCovariance() const {return orientationCovariance_;}

	/** @return Angular velocity (rad/s). */
	const cv::Vec3d & angularVelocity() const {return angularVelocity_;}
	/** @return 3×3 angular velocity covariance (row-major, empty if unused). */
	const cv::Mat & angularVelocityCovariance() const {return angularVelocityCovariance_;}

	/** @return Linear acceleration (m/s²). */
	const cv::Vec3d & linearAcceleration() const {return linearAcceleration_;}
	/** @return 3×3 linear acceleration covariance (row-major, empty if unused). */
	const cv::Mat & linearAccelerationCovariance() const {return linearAccelerationCovariance_;}

	/** @return Transform from IMU frame to base frame. */
	const Transform & localTransform() const {return localTransform_;}

	/**
	 * @brief Rotate motion (and optionally orientation) into the base frame.
	 *
	 * Applies @ref localTransform() rotation to vectors and covariances, then sets
	 * rotational part of @ref localTransform() to identity (translation unchanged).
	 * No-op if @ref localTransform() is null or rotation is identity.
	 * Orientation is updated only when quaternion x, y, z are not all zero.
	 */
	void convertToBaseFrame();

	/**
	 * @brief True when @ref localTransform() is null (placeholder / unset sample).
	 */
	bool empty() const
	{
		return localTransform_.isNull();
	}

private:
	cv::Vec4d orientation_;
	cv::Mat orientationCovariance_;

	cv::Vec3d angularVelocity_;
	cv::Mat angularVelocityCovariance_;

	cv::Vec3d linearAcceleration_;
	cv::Mat linearAccelerationCovariance_;

	Transform localTransform_;
};

/**
 * @class IMUEvent
 * @brief @ref UEvent carrying an @ref IMU sample and timestamp.
 */
class IMUEvent : public UEvent
{
public:
	/** @brief Default-constructs an event with zero stamp. */
	IMUEvent() :
		stamp_(0.0)
	{}
	/**
	 * @brief Constructs an event with IMU data and stamp.
	 * @param data IMU sample.
	 * @param stamp Timestamp in seconds.
	 */
	IMUEvent(const IMU & data, double stamp) :
		data_(data),
		stamp_(stamp)
	{
	}
	/** @return Event type name for the utilite event system. */
	virtual std::string getClassName() const {return "IMUEvent";}
	/** @return IMU payload. */
	const IMU & getData() const {return data_;}
	/** @return Timestamp in seconds. */
	double getStamp() const {return stamp_;}

private:
	IMU data_;
	double stamp_;
};

}

#endif /* IMU_H_ */
