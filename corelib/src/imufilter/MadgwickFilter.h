
/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CORELIB_SRC_IMUFILTER_MADGWICKFILTER_H_
#define CORELIB_SRC_IMUFILTER_MADGWICKFILTER_H_

#include <rtabmap/core/IMUFilter.h>
#include <cmath>

namespace rtabmap {

class MadgwickFilter : public IMUFilter
{
  public:

	MadgwickFilter(const ParametersMap & parameters = ParametersMap());
    virtual ~MadgwickFilter(){}

  private:
    // **** state variables
    double q0, q1, q2, q3;  // quaternion
    float w_bx_, w_by_, w_bz_; //
    bool initialized_;

    // **** paramaters
	double gain_;    // algorithm gain
	double zeta_;    // gyro drift bias gain

public:
	/**
	 * Gain of the filter. Higher values lead to faster convergence but
	 * more noise. Lower values lead to slower convergence but smoother signal. [0.0, 1.0]
	 */
    void setAlgorithmGain(double gain);

    /**
     * Gyro drift gain (approx. rad/s). [-1.0, 1.0]
     */
    void setDriftBiasGain(double zeta);

    virtual void parseParameters(const ParametersMap & parameters);
    virtual IMUFilter::Type type() const {return IMUFilter::kMadgwick;}
    virtual void getOrientation(double & qx, double & qy, double & qz, double & qw) const;
	virtual void reset(double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0);
private:

	// Update from accelerometer and gyroscope data.
	// [gx, gy, gz]: Angular veloctiy, in rad / s.
	// [ax, ay, az]: Normalized gravity vector.
	// dt: time delta, in seconds.
    void updateImpl(
    		double gx, double gy, double gz,
			double ax, double ay, double az,
    	    double dt);
};

}


#endif /* CORELIB_SRC_IMUFILTER_MADGWICKFILTER_H_ */
