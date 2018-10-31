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

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_GPS_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_GPS_H_

#include <rtabmap/core/GeodeticCoords.h>

namespace rtabmap {

class GPS
{
public:
	GPS():
		stamp_(0.0),
		longitude_(0.0),
		latitude_(0.0),
		altitude_(0.0),
		error_(0.0),
		bearing_(0.0)
	{}
	GPS(const double & stamp,
		const double & longitude,
		const double & latitude,
		const double & altitude,
		const double & error,
		const double & bearing):
			stamp_(stamp),
			longitude_(longitude),
			latitude_(latitude),
			altitude_(altitude),
			error_(error),
			bearing_(bearing)
	{}
	const double & stamp() const {return stamp_;}
	const double & longitude() const {return longitude_;}
	const double & latitude() const {return latitude_;}
	const double & altitude() const {return altitude_;}
	const double & error() const {return error_;}
	const double & bearing() const {return bearing_;}

	GeodeticCoords toGeodeticCoords() const {return GeodeticCoords(latitude_, longitude_, altitude_);}
private:
	double stamp_;     // in sec
	double longitude_; // DD
	double latitude_;  // DD
	double altitude_;  // m
	double error_;     // m
	double bearing_;   // deg (North 0->360 clockwise)
};

}

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_GPS_H_ */
