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

#include "rtabmap/core/odometry/OdometryLOAM.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/core/util3d.h"
#include <pcl/common/transforms.h>

float SCAN_PERIOD = 0.1f;

namespace rtabmap {

/**
 * https://github.com/laboshinl/loam_velodyne/pull/66
 */

OdometryLOAM::OdometryLOAM(const ParametersMap & parameters) :
	Odometry(parameters)
#ifdef RTABMAP_LOAM
	,lastPose_(Transform::getIdentity())
	,scanPeriod_(Parameters::defaultOdomLOAMScanPeriod())
	,linVar_(Parameters::defaultOdomLOAMLinVar())
	,angVar_(Parameters::defaultOdomLOAMAngVar())
	,localMapping_(Parameters::defaultOdomLOAMLocalMapping())
	,lost_(false)
#endif
{
#ifdef RTABMAP_LOAM
	int velodyneType = 0;
	Parameters::parse(parameters, Parameters::kOdomLOAMSensor(), velodyneType);
	Parameters::parse(parameters, Parameters::kOdomLOAMScanPeriod(), scanPeriod_);
	UASSERT(scanPeriod_>0.0f);
	Parameters::parse(parameters, Parameters::kOdomLOAMLinVar(), linVar_);
	UASSERT(linVar_>0.0f);
	Parameters::parse(parameters, Parameters::kOdomLOAMAngVar(), angVar_);
	UASSERT(angVar_>0.0f);
	Parameters::parse(parameters, Parameters::kOdomLOAMLocalMapping(), localMapping_);
	if(velodyneType == 1)
	{
		scanMapper_ = loam::MultiScanMapper::Velodyne_HDL_32();
	}
	else if(velodyneType == 2)
	{
		scanMapper_ = loam::MultiScanMapper::Velodyne_HDL_64E();
	}
	else
	{
		scanMapper_ = loam::MultiScanMapper::Velodyne_VLP_16();
	}
	laserOdometry_ =  new loam::BasicLaserOdometry(scanPeriod_);
	laserMapping_ = new loam::BasicLaserMapping(scanPeriod_);
#endif
}

OdometryLOAM::~OdometryLOAM()
{
#ifdef RTABMAP_LOAM
	delete laserOdometry_;
	delete laserMapping_;
#endif
}

void OdometryLOAM::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_LOAM
	lastPose_.setIdentity();
	scanRegistration_ = loam::BasicScanRegistration();
	loam::RegistrationParams regParams;
	regParams.scanPeriod = scanPeriod_;
	scanRegistration_.configure(regParams);
	delete laserOdometry_;
	laserOdometry_ =  new loam::BasicLaserOdometry(scanPeriod_);
	delete laserMapping_;
	laserMapping_ = new loam::BasicLaserMapping(scanPeriod_);
	transformMaintenance_ = loam::BasicTransformMaintenance();
	lost_ = false;
#endif
}

#ifdef RTABMAP_LOAM
std::vector<pcl::PointCloud<pcl::PointXYZI> > OdometryLOAM::segmentScanRings(const pcl::PointCloud<pcl::PointXYZ> & laserCloudIn)
{
	std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans;

	size_t cloudSize = laserCloudIn.size();

	// determine scan start and end orientations
	float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
	float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
			laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
	if (endOri - startOri > 3 * M_PI) {
		endOri -= 2 * M_PI;
	} else if (endOri - startOri < M_PI) {
		endOri += 2 * M_PI;
	}

	bool halfPassed = false;
	pcl::PointXYZI point;
	laserCloudScans.resize(scanMapper_.getNumberOfScanRings());
	// clear all scanline points
	std::for_each(laserCloudScans.begin(), laserCloudScans.end(), [](auto&&v) {v.clear(); });

	// extract valid points from input cloud
	for (size_t i = 0; i < cloudSize; i++) {
		point.x = laserCloudIn[i].y;
		point.y = laserCloudIn[i].z;
		point.z = laserCloudIn[i].x;

		// skip NaN and INF valued points
		if (!pcl_isfinite(point.x) ||
				!pcl_isfinite(point.y) ||
				!pcl_isfinite(point.z)) {
			continue;
		}

		// skip zero valued points
		if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
			continue;
		}

		// calculate vertical point angle and scan ID
		float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
		int scanID = scanMapper_.getRingForAngle(angle);
		if (scanID >= scanMapper_.getNumberOfScanRings() || scanID < 0 ){
			continue;
		}

		// calculate horizontal point angle
		float ori = -std::atan2(point.x, point.z);
		if (!halfPassed) {
			if (ori < startOri - M_PI / 2) {
				ori += 2 * M_PI;
			} else if (ori > startOri + M_PI * 3 / 2) {
				ori -= 2 * M_PI;
			}

			if (ori - startOri > M_PI) {
				halfPassed = true;
			}
		} else {
			ori += 2 * M_PI;

			if (ori < endOri - M_PI * 3 / 2) {
				ori += 2 * M_PI;
			} else if (ori > endOri + M_PI / 2) {
				ori -= 2 * M_PI;
			}
		}

		// calculate relative scan time based on point orientation
		float relTime = SCAN_PERIOD * (ori - startOri) / (endOri - startOri);
		point.intensity = scanID + relTime;

		// imu not used...
		//scanRegistration_.projectPointToStartOfSweep(point, relTime);

		laserCloudScans[scanID].push_back(point);
	}

	return laserCloudScans;
}
#endif

// return not null transform if odometry is correctly computed
Transform OdometryLOAM::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	Transform t;
#ifdef RTABMAP_LOAM
	UTimer timer;

	if(data.laserScanRaw().isEmpty())
	{
		UERROR("LOAM works only with laser scans and the current input is empty. Aborting odometry update...");
		return t;
	}
	else if(data.laserScanRaw().is2d())
	{
		UERROR("LOAM version used works only with 3D laser scans from Velodyne. Aborting odometry update...");
		return t;
	}

	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1)*9999;
	if(!lost_)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudInPtr = util3d::laserScanToPointCloud(data.laserScanRaw());
		std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans = segmentScanRings(*laserCloudInPtr);

		ros::Time stampT;
		stampT.fromSec(data.stamp());
		loam::Time scanTime = loam::fromROSTime(stampT);
		scanRegistration_.processScanlines(scanTime, laserCloudScans);

		*laserOdometry_->cornerPointsSharp() = scanRegistration_.cornerPointsSharp();
		*laserOdometry_->cornerPointsLessSharp() = scanRegistration_.cornerPointsLessSharp();
		*laserOdometry_->surfPointsFlat() = scanRegistration_.surfacePointsFlat();
		*laserOdometry_->surfPointsLessFlat() = scanRegistration_.surfacePointsLessFlat();
		*laserOdometry_->laserCloud() = scanRegistration_.laserCloud();
		pcl::PointCloud<pcl::PointXYZ> imuTrans;
		imuTrans.resize(4);
		laserOdometry_->updateIMU(imuTrans);
		laserOdometry_->process();

		if(localMapping_)
		{
			laserMapping_->laserCloudCornerLast() = *laserOdometry_->lastCornerCloud();
			laserMapping_->laserCloudSurfLast() = *laserOdometry_->lastSurfaceCloud();
			laserMapping_->laserCloud() = *laserOdometry_->laserCloud();
			laserMapping_->updateOdometry(laserOdometry_->transformSum());
			laserMapping_->process(scanTime);
		}

		transformMaintenance_.updateOdometry(
				laserOdometry_->transformSum().rot_x.rad(),
				laserOdometry_->transformSum().rot_y.rad(),
				laserOdometry_->transformSum().rot_z.rad(),
				laserOdometry_->transformSum().pos.x(),
				laserOdometry_->transformSum().pos.y(),
				laserOdometry_->transformSum().pos.z());
		transformMaintenance_.updateMappingTransform(laserMapping_->transformAftMapped(), laserMapping_->transformBefMapped());
		transformMaintenance_.transformAssociateToMap();
		const float * tm = transformMaintenance_.transformMapped();
		Transform pose = Transform(tm[5], tm[3], tm[4], tm[2], tm[0], tm[1]);

		if(!pose.isNull())
		{
			covariance = cv::Mat::eye(6,6,CV_64FC1);
			covariance(cv::Range(0,3), cv::Range(0,3)) *= linVar_;
			covariance(cv::Range(3,6), cv::Range(3,6)) *= angVar_;

			t = lastPose_.inverse() * pose; // incremental
			lastPose_ = pose;

			const Transform & localTransform = data.laserScanRaw().localTransform();
			if(!t.isNull() && !t.isIdentity() && !localTransform.isIdentity() && !localTransform.isNull())
			{
				// from laser frame to base frame
				t = localTransform * t * localTransform.inverse();
			}

			if(info)
			{
				info->type = (int)kTypeLOAM;
				info->localScanMapSize = laserMapping_->laserCloudSurroundDS().size();
				if(covariance.cols == 6 && covariance.rows == 6 && covariance.type() == CV_64FC1)
				{
					info->reg.covariance = covariance;
				}

				if(this->isInfoDataFilled())
				{
					Transform rot(0,0,1,0,1,0,0,0,0,1,0,0);
					pcl::PointCloud<pcl::PointXYZI> out;
					pcl::transformPointCloud(laserMapping_->laserCloudSurroundDS(), out, rot.toEigen3f());
					info->localScanMap = LaserScan::backwardCompatibility(util3d::laserScanFromPointCloud(out), 0, data.laserScanRaw().rangeMax(), data.laserScanRaw().localTransform());
				}
			}
		}
		else
		{
			lost_ = true;
			UWARN("LOAM failed to register the latest scan, odometry should be reset.");
		}
	}
	UINFO("Odom update time = %fs, lost=%s", timer.elapsed(), lost_?"true":"false");

#else
	UERROR("RTAB-Map is not built with LOAM support! Select another odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
