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


#include <rtabmap/core/IMUFilter.h>
#include <rtabmap/utilite/ULogger.h>
#include "imufilter/ComplementaryFilter.h"
#ifdef RTABMAP_MADGWICK
#include "imufilter/MadgwickFilter.h"
#endif

namespace rtabmap {

IMUFilter * IMUFilter::create(const ParametersMap & parameters)
{
	int type = Parameters::defaultKpDetectorStrategy();
	Parameters::parse(parameters, Parameters::kKpDetectorStrategy(), type);
	return create((IMUFilter::Type)type, parameters);
}

IMUFilter * IMUFilter::create(IMUFilter::Type type, const ParametersMap & parameters)
{
#ifndef RTABMAP_MADGWICK
	if(type == IMUFilter::kMadgwick)
	{
		UWARN("Madgwick filter cannot be used as RTAB-Map is not built with the option enabled. Complementary filter is used instead.");
		type = IMUFilter::kComplementaryFilter;
	}
#endif

	IMUFilter * filter = 0;
	switch(type)
	{
#ifdef RTABMAP_MADGWICK
	case IMUFilter::kMadgwick:
		filter = new MadgwickFilter(parameters);
		break;
#endif
	default:
		filter = new ComplementaryFilter(parameters);
		type = IMUFilter::kComplementaryFilter;
		break;

	}
	return filter;
}

void IMUFilter::update(
			double gx, double gy, double gz,
			double ax, double ay, double az,
			double stamp)
	{
		if(previousStamp_ == 0.0)
		{
			previousStamp_ = stamp;
		}
		double dt = stamp - previousStamp_;

		updateImpl(gx, gy, gz, ax, ay, az, dt);

		previousStamp_ = stamp;
	}

}
