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

#include "rtabmap/core/odometry/OdometryFLOAM.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/core/util3d.h"
#include <pcl/common/transforms.h>

#ifdef RTABMAP_FLOAM
#include <laserProcessingClass.h>
#include <odomEstimationClass.h>
#endif

namespace rtabmap {

/**
 * https://github.com/wh200720041/floam
 */

OdometryFLOAM::OdometryFLOAM(const ParametersMap & parameters) :
	Odometry(parameters)
#ifdef RTABMAP_FLOAM
	,laserProcessing_(new LaserProcessingClass())
	,odomEstimation_(new OdomEstimationClass())
	,lastPose_(Transform::getIdentity())
	,lost_(false)
#endif
{
#ifdef RTABMAP_FLOAM
	int sensor = Parameters::defaultOdomLOAMSensor();
	double vertical_angle = 2.0; // seems not used by floam (https://github.com/wh200720041/floam/issues/31)
	float scan_period= Parameters::defaultOdomLOAMScanPeriod();
	float max_dis = Parameters::defaultIcpRangeMax();
	float min_dis = Parameters::defaultIcpRangeMin();
	float map_resolution = Parameters::defaultOdomLOAMResolution();
	linVar_ = Parameters::defaultOdomLOAMLinVar();
	angVar_ = Parameters::defaultOdomLOAMAngVar();

	Parameters::parse(parameters, Parameters::kOdomLOAMSensor(), sensor);
	Parameters::parse(parameters, Parameters::kOdomLOAMScanPeriod(), scan_period);
	Parameters::parse(parameters, Parameters::kIcpRangeMax(), max_dis);
	Parameters::parse(parameters, Parameters::kIcpRangeMin(), min_dis);
	Parameters::parse(parameters, Parameters::kOdomLOAMResolution(), map_resolution);

	UASSERT(scan_period>0.0f);
	Parameters::parse(parameters, Parameters::kOdomLOAMLinVar(), linVar_);
	UASSERT(linVar_>0.0f);
	Parameters::parse(parameters, Parameters::kOdomLOAMAngVar(), angVar_);
	UASSERT(angVar_>0.0f);

	lidar::Lidar lidar_param;
	lidar_param.setScanPeriod(scan_period);
	lidar_param.setVerticalAngle(vertical_angle);
	lidar_param.setLines(sensor==2?64:sensor==1?32:16);
	lidar_param.setMaxDistance(max_dis<=0?200:max_dis);
	lidar_param.setMinDistance(min_dis);

	laserProcessing_->init(lidar_param);
	odomEstimation_->init(lidar_param, map_resolution);
#endif
}

OdometryFLOAM::~OdometryFLOAM()
{
#ifdef RTABMAP_FLOAM
	delete laserProcessing_;
	delete odomEstimation_;
#endif
}

void OdometryFLOAM::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_FLOAM
	lastPose_.setIdentity();
	lost_ = false;
#endif
}

// return not null transform if odometry is correctly computed
Transform OdometryFLOAM::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	Transform t;
#ifdef RTABMAP_FLOAM
	UTimer timer;
	UTimer timerTotal;

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
		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudInPtr = util3d::laserScanToPointCloudI(data.laserScanRaw());

		UDEBUG("Scan conversion: %fs", timer.ticks());

		pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

		laserProcessing_->featureExtraction(laserCloudInPtr,pointcloud_edge,pointcloud_surf);
		UDEBUG("Feature extraction: %fs", timer.ticks());

		if(this->framesProcessed() == 0){
			odomEstimation_->initMapWithPoints(pointcloud_edge, pointcloud_surf);
		}else{
			odomEstimation_->updatePointsToMap(pointcloud_edge, pointcloud_surf);
		}
		UDEBUG("Update: %fs", timer.ticks());

		Transform pose = Transform::fromEigen3d(odomEstimation_->odom);

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
				if(covariance.cols == 6 && covariance.rows == 6 && covariance.type() == CV_64FC1)
				{
					info->reg.covariance = covariance;
				}

				if(this->isInfoDataFilled())
				{
					pcl::PointCloud<pcl::PointXYZI>::Ptr localMap(new pcl::PointCloud<pcl::PointXYZI>());
					odomEstimation_->getMap(localMap);
					info->localScanMapSize = localMap->size();
					info->localScanMap = LaserScan(util3d::laserScanFromPointCloud(*localMap), 0, data.laserScanRaw().rangeMax(), data.laserScanRaw().localTransform());
					UDEBUG("Fill info data: %fs", timer.ticks());
				}
			}
		}
		else
		{
			lost_ = true;
			UWARN("FLOAM failed to register the latest scan, odometry should be reset.");
		}
	}
	UINFO("Odom update time = %fs, lost=%s", timerTotal.elapsed(), lost_?"true":"false");

#else
	UERROR("RTAB-Map is not built with FLOAM support! Select another odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
