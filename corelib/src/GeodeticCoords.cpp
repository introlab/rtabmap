/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

/*
 * The methods in this file were modified from the originals of the MRPT toolkit (see notice below):
 * https://github.com/MRPT/mrpt/blob/master/libs/topography/src/conversions.cpp
 */

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include "rtabmap/core/GeodeticCoords.h"

#include <cmath>
#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

namespace rtabmap {


inline double DEG2RAD(const double x) { return x*M_PI/180.0;}
inline double RAD2DEG(const double x) { return x*180.0/M_PI;}
inline double square(const double & value) {return value*value;}

GeodeticCoords::GeodeticCoords() :
		latitude_(0.0),
		longitude_(0.0),
		altitude_(0.0)
{

}
GeodeticCoords::GeodeticCoords(double latitude, double longitude, double altitude) :
		latitude_(latitude),
		longitude_(longitude),
		altitude_(altitude)
{

}

//*---------------------------------------------------------------
//			geodeticToGeocentric_WGS84
// ---------------------------------------------------------------*/
cv::Point3d GeodeticCoords::toGeocentric_WGS84() const
{
	// --------------------------------------------------------------------
	// See: http://en.wikipedia.org/wiki/Reference_ellipsoid
	//  Constants are for WGS84
	// --------------------------------------------------------------------

	static const double a = 6378137;		// Semi-major axis of the Earth (meters)
	static const double b = 6356752.3142;	// Semi-minor axis:

	static const double ae = acos(b/a);  	// eccentricity:
	static const double cos2_ae_earth =  square(cos(ae)); // The cos^2 of the angular eccentricity of the Earth: // 0.993305619995739L;
	static const double sin2_ae_earth = square(sin(ae));  // The sin^2 of the angular eccentricity of the Earth: // 0.006694380004261L;

	const double lon  = DEG2RAD( double(this->longitude()) );
	const double lat  = DEG2RAD( double(this->latitude()) );

	// The radius of curvature in the prime vertical:
	const double N = a / std::sqrt( 1.0 - sin2_ae_earth*square( sin(lat) ) );

	// Generate 3D point:
	cv::Point3d out;
	out.x = (N+this->altitude())*cos(lat)*cos(lon);
	out.y = (N+this->altitude())*cos(lat)*sin(lon);
	out.z = (cos2_ae_earth*N+this->altitude())*sin(lat);

	return out;
}


/*---------------------------------------------------------------
				geodeticToENU_WGS84
 ---------------------------------------------------------------*/
cv::Point3d GeodeticCoords::toENU_WGS84(const GeodeticCoords &origin) const
{
	// Generate 3D point:
	cv::Point3d	P_geocentric = this->toGeocentric_WGS84();

	// Generate reference 3D point:
	cv::Point3d P_geocentric_ref = origin.toGeocentric_WGS84();

	return Geocentric_WGS84ToENU_WGS84(P_geocentric, P_geocentric_ref, origin);
}

cv::Point3d GeodeticCoords::Geocentric_WGS84ToENU_WGS84(
		const cv::Point3d & geocentric_WGS84,
		const cv::Point3d & origin_geocentric_WGS84,
		const GeodeticCoords & origin)
{
	// --------------------------------------------------------------------
	//  Explanation: We compute the earth-centric coordinates first,
	//    then make a system transformation to local XYZ coordinates
	//    using a system of three orthogonal vectors as local reference.
	//
	// See: http://en.wikipedia.org/wiki/Reference_ellipsoid
	// (JLBC 21/DEC/2006)  (Fixed: JLBC 9/JUL/2008)
	// - Oct/2013, Emilio Sanjurjo: Fixed UP vector pointing exactly normal to ellipsoid surface.
	// --------------------------------------------------------------------

	const double clat = cos(DEG2RAD(origin.latitude())), slat = sin(DEG2RAD(origin.latitude()));
	const double clon = cos(DEG2RAD(origin.longitude())), slon = sin(DEG2RAD(origin.longitude()));

	// Compute the resulting relative coordinates:
	// For using smaller numbers:
	cv::Point3d geocentric_WGS84_rel = geocentric_WGS84-origin_geocentric_WGS84;

	// Optimized calculation: Local transformed coordinates of P_geo(x,y,z)
	//   after rotation given by the transposed rotation matrix from ENU -> ECEF.
	cv::Point3d out;
	out.x = -slon*geocentric_WGS84_rel.x + clon*geocentric_WGS84_rel.y;
	out.y = -clon*slat*geocentric_WGS84_rel.x -slon*slat*geocentric_WGS84_rel.y + clat*geocentric_WGS84_rel.z;
	out.z = clon*clat*geocentric_WGS84_rel.x + slon*clat*geocentric_WGS84_rel.y +slat*geocentric_WGS84_rel.z;

	return out;
}

void GeodeticCoords::fromGeocentric_WGS84(const cv::Point3d& geocentric)
{
	static const double a = 6378137;		// Semi-major axis of the Earth (meters)
	static const double b = 6356752.3142;	// Semi-minor axis:

	const double sa2 = a*a;
	const double sb2 = b*b;

	const double e2 = (sa2 - sb2) / sa2;
	const double ep2 = (sa2 - sb2) / sb2;
	const double p = std::sqrt(geocentric.x * geocentric.x + geocentric.y * geocentric.y);
	const double theta = atan2(geocentric.z * a, p * b);

	longitude_ = atan2(geocentric.y, geocentric.x);
	latitude_ = atan2(
			geocentric.z + ep2 * b * sin(theta) * sin(theta) * sin(theta),
		p - e2 * a * cos(theta) * cos(theta) * cos(theta));

	const double clat = cos(latitude_);
	const double slat = sin(latitude_);
	const double N = sa2 / std::sqrt(sa2 * clat * clat + sb2 * slat * slat);

	altitude_ = p / clat - N;
	longitude_ = RAD2DEG(longitude_);
	latitude_ = RAD2DEG(latitude_);
}

void GeodeticCoords::fromENU_WGS84(const cv::Point3d& enu, const GeodeticCoords& origin)
{
	fromGeocentric_WGS84(ENU_WGS84ToGeocentric_WGS84(enu, origin));
}

cv::Point3d GeodeticCoords::ENU_WGS84ToGeocentric_WGS84(const cv::Point3d& enu, const GeodeticCoords& origin)
{
	// Generate reference 3D point:
	cv::Point3f originGeocentric;
	originGeocentric = origin.toGeocentric_WGS84();

	cv::Vec3d P_ref(originGeocentric.x, originGeocentric.y, originGeocentric.z);

	// Z axis -> In direction out-ward the center of the Earth:
	cv::Vec3d REF_X, REF_Y, REF_Z;
	REF_Z = cv::normalize(P_ref);

	// 1st column: Starting at the reference point, move in the tangent
	// direction
	//   east-ward: I compute this as the derivative of P_ref wrt "longitude":
	//      A_east[0] =-(N+in_height_meters)*cos(lat)*sin(lon);  --> -Z[1]
	//      A_east[1] = (N+in_height_meters)*cos(lat)*cos(lon);  -->  Z[0]
	//      A_east[2] = 0;                                       -->  0
	// ---------------------------------------------------------------------------
	cv::Vec3d AUX_X(-REF_Z[1], REF_Z[0], 0);
	REF_X = cv::normalize(AUX_X);

	// 2nd column: The cross product:
	REF_Y = REF_Z.cross(REF_X);

	cv::Point3d out_coords;
	out_coords.x =
		REF_X[0] * enu.x + REF_Y[0] * enu.y + REF_Z[0] * enu.z + originGeocentric.x;
	out_coords.y =
		REF_X[1] * enu.x + REF_Y[1] * enu.y + REF_Z[1] * enu.z + originGeocentric.y;
	out_coords.z =
		REF_X[2] * enu.x + REF_Y[2] * enu.y + REF_Z[2] * enu.z + originGeocentric.z;

	return out_coords;
}

}
