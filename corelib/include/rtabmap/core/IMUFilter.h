/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_IMUFILTER_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_IMUFILTER_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines
#include <rtabmap/core/Parameters.h>
#include <Eigen/Geometry>

namespace rtabmap {

/**
 * @class IMUFilter
 * @brief Fuses gyroscope and accelerometer samples into an orientation quaternion.
 *
 * Implementations integrate angular rates and correct drift using the measured
 * gravity vector. The public @ref update() API takes timestamps and computes the
 * time step between consecutive calls.
 *
 * Factory methods @ref create() return heap-allocated instances; the caller owns
 * the pointer (e.g. @ref SensorCaptureThread, @ref IMUThread, @ref Camera).
 *
 * Output orientation from @ref getOrientation() is a unit quaternion
 * `(qx, qy, qz, qw)` in the same convention as @ref IMU (body frame).
 *
 * Filter tuning parameters are read from @ref ParametersMap (ImuFilter/... keys);
 * see @ref ComplementaryFilter and @ref MadgwickFilter (when RTAB-Map is built
 * with Madgwick support).
 *
 * @see IMU
 * @see SensorCaptureThread::enableIMUFiltering()
 */
class RTABMAP_CORE_EXPORT IMUFilter
{
public:
	/**
	 * @brief Orientation fusion algorithm.
	 */
	enum Type {
		kMadgwick = 0,              /**< Madgwick AHRS (attitude and heading reference system). RTAB-Map must be built with Madgwick support. */
		kComplementaryFilter = 1};  /**< Complementary filter (always available). */

	/**
	 * @brief Creates a filter using type parsed from @p parameters.
	 * @param parameters Optional ImuFilter/... tuning parameters.
	 * @return New filter instance (caller owns the pointer).
	 */
	static IMUFilter * create(const ParametersMap & parameters = ParametersMap());

	/**
	 * @brief Creates a filter of the given @p type.
	 * @param type Fusion algorithm; falls back to @ref kComplementaryFilter if
	 *             @ref kMadgwick is requested but not compiled in.
	 * @param parameters Optional ImuFilter/... tuning parameters.
	 * @return New filter instance (caller owns the pointer).
	 */
	static IMUFilter * create(IMUFilter::Type type, const ParametersMap & parameters = ParametersMap());

	virtual ~IMUFilter() {}

	/**
	 * @brief Re-reads filter parameters from @p parameters.
	 * @param parameters ImuFilter/... keys (implementation-specific).
	 */
	virtual void parseParameters(const ParametersMap & parameters) {}

	/**
	 * @brief Integrates one IMU sample and updates the internal orientation estimate.
	 * @param gx Gyroscope x angular rate (rad/s).
	 * @param gy Gyroscope y angular rate (rad/s).
	 * @param gz Gyroscope z angular rate (rad/s).
	 * @param ax Accelerometer x (m/s²; magnitude ~9.81 when stationary).
	 * @param ay Accelerometer y (m/s²).
	 * @param az Accelerometer z (m/s²).
	 * @param stamp Sample time (seconds); used with the previous stamp to compute `dt`.
	 */
	void update(
			double gx, double gy, double gz,
			double ax, double ay, double az,
			double stamp);

	/** @return Active fusion algorithm type. */
	virtual IMUFilter::Type type() const = 0;

	/**
	 * @brief Current orientation estimate.
	 * @param qx Quaternion x
	 * @param qy Quaternion y
	 * @param qz Quaternion z
	 * @param qw Quaternion w (scalar)
	 */
	virtual void getOrientation(double & qx, double & qy, double & qz, double & qw) const = 0;

	/**
	 * @brief Resets internal state to the given orientation.
	 * @param qx Quaternion x (default 0)
	 * @param qy Quaternion y (default 0)
	 * @param qz Quaternion z (default 0)
	 * @param qw Quaternion w (default 1, identity)
	 */
	virtual void reset(double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0) = 0;

protected:
	IMUFilter(const ParametersMap & parameters = ParametersMap()) : previousStamp_(0) {}

private:
	virtual void updateImpl(
			double gx, double gy, double gz,
			double ax, double ay, double az,
			double dt) = 0;

	double previousStamp_;
};

} // namespace rtabmap

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_IMUFILTER_H_ */
