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

#include "rtabmap/core/odometry/OdometryLIOSAM.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"

#ifdef RTABMAP_LIOSAM
#include <LioSamCore.h>
#include <pcl/common/transforms.h>
#endif

namespace rtabmap {

OdometryLIOSAM::OdometryLIOSAM(const ParametersMap & parameters) :
	Odometry(parameters)
#ifdef RTABMAP_LIOSAM
	,lioSam_(0)
	,lastPose_(Transform::getIdentity())
	,lost_(false)
	,linVar_(Parameters::defaultOdomLIOSAMLinVar())
	,angVar_(Parameters::defaultOdomLIOSAMAngVar())
#endif
{
#ifdef RTABMAP_LIOSAM
	Parameters::parse(parameters, Parameters::kOdomLIOSAMLinVar(), linVar_);
	UASSERT(linVar_ > 0.0f);
	Parameters::parse(parameters, Parameters::kOdomLIOSAMAngVar(), angVar_);
	UASSERT(angVar_ > 0.0f);

	// Build ParamServer from rtabmap parameters
	ParamServer config;

	// Sensor type
	int sensorType = Parameters::defaultOdomLIOSAMSensor();
	Parameters::parse(parameters, Parameters::kOdomLIOSAMSensor(), sensorType);
	if(sensorType == 1)
		config.sensor = SensorType::OUSTER;
	else if(sensorType == 2)
		config.sensor = SensorType::LIVOX;
	else
		config.sensor = SensorType::VELODYNE;

	config.N_SCAN = Parameters::defaultOdomLIOSAMNScan();
	Parameters::parse(parameters, Parameters::kOdomLIOSAMNScan(), config.N_SCAN);

	config.Horizon_SCAN = Parameters::defaultOdomLIOSAMHorizonScan();
	Parameters::parse(parameters, Parameters::kOdomLIOSAMHorizonScan(), config.Horizon_SCAN);

	config.imuAccNoise = Parameters::defaultOdomLIOSAMImuAccNoise();
	Parameters::parse(parameters, Parameters::kOdomLIOSAMImuAccNoise(), config.imuAccNoise);

	config.imuGyrNoise = Parameters::defaultOdomLIOSAMImuGyrNoise();
	Parameters::parse(parameters, Parameters::kOdomLIOSAMImuGyrNoise(), config.imuGyrNoise);

	config.imuAccBiasN = Parameters::defaultOdomLIOSAMImuAccBiasN();
	Parameters::parse(parameters, Parameters::kOdomLIOSAMImuAccBiasN(), config.imuAccBiasN);

	config.imuGyrBiasN = Parameters::defaultOdomLIOSAMImuGyrBiasN();
	Parameters::parse(parameters, Parameters::kOdomLIOSAMImuGyrBiasN(), config.imuGyrBiasN);

	config.imuGravity = Parameters::defaultOdomLIOSAMImuGravity();
	Parameters::parse(parameters, Parameters::kOdomLIOSAMImuGravity(), config.imuGravity);

	config.edgeThreshold = Parameters::defaultOdomLIOSAMEdgeThreshold();
	Parameters::parse(parameters, Parameters::kOdomLIOSAMEdgeThreshold(), config.edgeThreshold);

	config.surfThreshold = Parameters::defaultOdomLIOSAMSurfThreshold();
	Parameters::parse(parameters, Parameters::kOdomLIOSAMSurfThreshold(), config.surfThreshold);

	// Set reasonable defaults for params not exposed via rtabmap
	config.downsampleRate = 1;
	config.lidarMinRange = 1.0f;
	config.lidarMaxRange = 1000.0f;
	config.imuRPYWeight = 0.01f;
	config.odometrySurfLeafSize = 0.2f;
	config.mappingCornerLeafSize = 0.2f;
	config.mappingSurfLeafSize = 0.4f;
	config.z_tollerance = FLT_MAX;
	config.rotation_tollerance = FLT_MAX;
	config.numberOfCores = 4;
	config.mappingProcessInterval = 0.01;
	config.surroundingkeyframeAddingDistThreshold = 1.0f;
	config.surroundingkeyframeAddingAngleThreshold = 0.2f;
	config.surroundingKeyframeDensity = 1.0f;
	config.surroundingKeyframeSearchRadius = 50.0f;
	config.loopClosureEnableFlag = false;  // rtabmap handles loop closures
	config.loopClosureFrequency = 1.0f;
	config.surroundingKeyframeSize = 50;
	config.historyKeyframeSearchRadius = 10.0f;
	config.historyKeyframeSearchTimeDiff = 30.0f;
	config.historyKeyframeSearchNum = 25;
	config.historyKeyframeFitnessScore = 0.3f;
	config.globalMapVisualizationSearchRadius = 1e3f;
	config.globalMapVisualizationPoseDensity = 10.0f;
	config.globalMapVisualizationLeafSize = 1.0f;
	config.edgeFeatureMinValidNum = 10;
	config.surfFeatureMinValidNum = 100;
	config.savePCD = false;
	config.useImuHeadingInitialization = false;
	config.useGpsElevation = false;
	config.gpsCovThreshold = 2.0f;
	config.poseCovThreshold = 25.0f;

	// Identity extrinsics (IMU data should already be in lidar frame)
	config.extRotV = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	config.extRPYV = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	config.extTransV = {0, 0, 0};

	// Set the global extrinsics used by imuConverter
	extRot = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(config.extRotV.data());
	extRPY = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(config.extRPYV.data());
	extTrans = Eigen::Map<const Eigen::Matrix<double, 3, 1> >(config.extTransV.data());
	extQRPY = Eigen::Quaterniond(extRPY).inverse();

	lioSam_ = new lio_sam::LioSamCore(config);
#endif
}

OdometryLIOSAM::~OdometryLIOSAM()
{
#ifdef RTABMAP_LIOSAM
	delete lioSam_;
#endif
}

void OdometryLIOSAM::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_LIOSAM
	if(lioSam_)
	{
		lioSam_->reset();
	}
	lastPose_ = Transform::getIdentity();
	lost_ = false;
#endif
}

Transform OdometryLIOSAM::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	Transform t;
#ifdef RTABMAP_LIOSAM
	UTimer timer;
	UTimer timerTotal;

	// Handle async IMU data (canProcessAsyncIMU() == true means
	// the base class sends IMU-only data directly to computeTransform)
	if(!data.imu().empty())
	{
		Eigen::Quaterniond qd(
			data.imu().orientation()[3],  // w
			data.imu().orientation()[0],  // x
			data.imu().orientation()[1],  // y
			data.imu().orientation()[2]); // z
		Eigen::Vector3d acc(
			data.imu().linearAcceleration()[0],
			data.imu().linearAcceleration()[1],
			data.imu().linearAcceleration()[2]);
		Eigen::Vector3d gyro(
			data.imu().angularVelocity()[0],
			data.imu().angularVelocity()[1],
			data.imu().angularVelocity()[2]);
		lioSam_->addImu(data.stamp(), acc, gyro, qd);

		// IMU-only: no pose to return
		if(data.laserScanRaw().isEmpty())
		{
			return t;
		}
	}

	if(data.laserScanRaw().isEmpty())
	{
		UERROR("LIO-SAM requires laser scans and the current input is empty. Aborting odometry update...");
		return t;
	}
	else if(data.laserScanRaw().is2d())
	{
		UERROR("LIO-SAM requires 3D laser scans. Aborting odometry update...");
		return t;
	}

	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999;
	if(!lost_)
	{
		// Convert laser scan to PCL point cloud
		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn = util3d::laserScanToPointCloudI(data.laserScanRaw());
		UDEBUG("Scan conversion: %fs, points=%d", timer.ticks(), (int)laserCloudIn->size());

		// Extract ring and time channels if available
		std::vector<int> rings;
		std::vector<float> times;
		const LaserScan & scan = data.laserScanRaw();

		// Check if scan has ring channel
		if(scan.hasRGB() || scan.channels() >= 6)
		{
			// Try to extract ring from the scan data
			// Ring is typically channel 4 in XYZI+ring+time format
		}

		// Process scan
		Eigen::Affine3f poseOut;
		Eigen::MatrixXd covOut;
		bool ok = lioSam_->processScan(data.stamp(), laserCloudIn, rings, times, poseOut, covOut);
		UDEBUG("LIO-SAM process: %fs", timer.ticks());

		if(ok)
		{
			Transform pose = Transform::fromEigen3f(poseOut);

			if(!pose.isNull())
			{
				covariance = cv::Mat::eye(6, 6, CV_64FC1);
				covariance(cv::Range(0, 3), cv::Range(0, 3)) *= linVar_;
				covariance(cv::Range(3, 6), cv::Range(3, 6)) *= angVar_;

				t = lastPose_.inverse() * pose;  // incremental
				lastPose_ = pose;

				const Transform & localTransform = data.laserScanRaw().localTransform();
				if(!t.isNull() && !t.isIdentity() && !localTransform.isIdentity() && !localTransform.isNull())
				{
					// from laser frame to base frame
					t = localTransform * t * localTransform.inverse();
				}

				if(info)
				{
					info->type = (int)kTypeLIOSAM;
					if(covariance.cols == 6 && covariance.rows == 6 && covariance.type() == CV_64FC1)
					{
						info->reg.covariance = covariance;
					}

					if(this->isInfoDataFilled())
					{
						pcl::PointCloud<pcl::PointXYZI>::Ptr localMap = lioSam_->getLocalMap();
						if(localMap && !localMap->empty())
						{
							info->localScanMapSize = localMap->size();
							info->localScanMap = LaserScan(util3d::laserScanFromPointCloud(*localMap), 0, data.laserScanRaw().rangeMax(), data.laserScanRaw().localTransform());
						}
						UDEBUG("Fill info data: %fs", timer.ticks());
					}
				}
			}
			else
			{
				lost_ = true;
				UWARN("LIO-SAM failed to register the latest scan, odometry should be reset.");
			}
		}
		else
		{
			UDEBUG("LIO-SAM processScan returned false (may be initializing)");
		}
	}
	UINFO("LIO-SAM odom update time = %fs, lost=%s", timerTotal.elapsed(), lost_ ? "true" : "false");

#else
	UERROR("RTAB-Map is not built with LIO-SAM support! Select another odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
