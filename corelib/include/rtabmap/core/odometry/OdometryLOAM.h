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

#ifndef ODOMETRYLOAM_H_
#define ODOMETRYLOAM_H_

#include <rtabmap/core/Odometry.h>

#ifdef RTABMAP_LOAM
#include <loam_velodyne/BasicScanRegistration.h>
#include <loam_velodyne/BasicLaserOdometry.h>
#include <loam_velodyne/BasicLaserMapping.h>
#include <loam_velodyne/BasicTransformMaintenance.h>
#include <loam_velodyne/MultiScanRegistration.h>
#endif

namespace rtabmap {

class RTABMAP_CORE_EXPORT OdometryLOAM : public Odometry
{
public:
	OdometryLOAM(const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap());
	virtual ~OdometryLOAM();

	virtual void reset(const Transform & initialPose = Transform::getIdentity());
	virtual Odometry::Type getType() {return Odometry::kTypeLOAM;}

private:
	virtual Transform computeTransform(SensorData & image, const Transform & guess = Transform(), OdometryInfo * info = 0);

private:
#ifdef RTABMAP_LOAM
	std::vector<pcl::PointCloud<pcl::PointXYZI> > segmentScanRings(const pcl::PointCloud<pcl::PointXYZ> & laserCloudIn);

	loam::BasicScanRegistration scanRegistration_;
	loam::MultiScanMapper scanMapper_;
	loam::BasicLaserOdometry * laserOdometry_;
	loam::BasicLaserMapping * laserMapping_;
	loam::BasicTransformMaintenance transformMaintenance_;
	Transform lastPose_;
	float scanPeriod_;
	float linVar_;
	float angVar_;
	bool localMapping_;
	bool lost_;
#endif
};

}

#endif /* ODOMETRYLOAM_H_ */
