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

#include <rtabmap/core/Parameters.h>
#include <Eigen/Geometry>

namespace rtabmap {

class IMUFilter
{
public:
	enum Type {
		kMadgwick=0,
		kComplementaryFilter=1};
public:
	static IMUFilter * create(const ParametersMap & parameters = ParametersMap());
	static IMUFilter * create(IMUFilter::Type type, const ParametersMap & parameters = ParametersMap());

public:
	virtual void parseParameters(const ParametersMap & parameters) {}
	virtual ~IMUFilter(){}

	void update(
			double gx, double gy, double gz,
			double ax, double ay, double az,
			double stamp);

	virtual IMUFilter::Type type() const = 0;
	virtual void getOrientation(double & qx, double & qy, double & qz, double & qw) const = 0;
	virtual void reset(double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0) = 0;

protected:
	IMUFilter(const ParametersMap & parameters = ParametersMap()) : previousStamp_(0) {}

private:
	// Update from accelerometer and gyroscope data.
	// [gx, gy, gz]: Angular veloctiy, in rad / s.
	// [ax, ay, az]: Normalized gravity vector.
	// dt: time delta, in seconds.
	virtual void updateImpl(
			double gx, double gy, double gz,
			double ax, double ay, double az,
			double dt) = 0;

private:
	double previousStamp_;
};

}


#endif /* CORELIB_INCLUDE_RTABMAP_CORE_IMUFILTER_H_ */
