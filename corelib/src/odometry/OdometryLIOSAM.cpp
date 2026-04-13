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
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UFile.h"

#ifdef RTABMAP_LIOSAM
#include <LioSamCore.h>
#include <pcl/common/transforms.h>
#endif

namespace rtabmap {

static ParametersMap disableDeskewing(ParametersMap params) {
	// LIO-SAM performs its own internal deskewing via imageProjection.
	// The base-class deskew must be disabled so that the original per-point
	// timestamps reach LIO-SAM intact.
	params[Parameters::kOdomDeskewing()] = "false";
	return params;
}

OdometryLIOSAM::OdometryLIOSAM(const ParametersMap & parameters) :
	Odometry(disableDeskewing(parameters))
#ifdef RTABMAP_LIOSAM
	,lioSam_(0)
	,lastPose_(Transform::getIdentity())
	,lost_(false)
	,linVar_(Parameters::defaultOdomLIOSAMLinVar())
	,angVar_(Parameters::defaultOdomLIOSAMAngVar())
	,parameters_(parameters)
#endif
{
#ifdef RTABMAP_LIOSAM
	Parameters::parse(parameters, Parameters::kOdomLIOSAMLinVar(), linVar_);
	UASSERT(linVar_ > 0.0f);
	Parameters::parse(parameters, Parameters::kOdomLIOSAMAngVar(), angVar_);
	UASSERT(angVar_ > 0.0f);
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
	imuLocalTransform_ = Transform();
	imuBuffer_.clear();
#endif
}

#ifdef RTABMAP_LIOSAM
bool OdometryLIOSAM::init(const Transform & imuLocalTransform, const Transform & lidarLocalTransform)
{
	ParamServer config;

	// Check if a config file path was provided
	std::string configPath;
	Parameters::parse(parameters_, Parameters::kOdomLIOSAMConfigPath(), configPath);
	if(!configPath.empty())
	{
		configPath = uReplaceChar(configPath, '~', UDirectory::homeDir());
		if(!UFile::exists(configPath))
		{
			UERROR("LIO-SAM config file not found: %s", configPath.c_str());
			return false;
		}
		UINFO("Loading LIO-SAM parameters from config file: %s", configPath.c_str());
		config = loadParamsFromYaml(configPath);
	}
	else
	{
		UINFO("No LIO-SAM config file provided, using rtabmap parameters");

		// Build ParamServer from individual rtabmap parameters
		int sensorType = Parameters::defaultOdomLIOSAMSensor();
		Parameters::parse(parameters_, Parameters::kOdomLIOSAMSensor(), sensorType);
		if(sensorType == 1)
			config.sensor = SensorType::OUSTER;
		else if(sensorType == 2)
			config.sensor = SensorType::LIVOX;
		else
			config.sensor = SensorType::VELODYNE;

		config.N_SCAN = Parameters::defaultOdomLIOSAMNScan();
		Parameters::parse(parameters_, Parameters::kOdomLIOSAMNScan(), config.N_SCAN);

		config.Horizon_SCAN = Parameters::defaultOdomLIOSAMHorizonScan();
		Parameters::parse(parameters_, Parameters::kOdomLIOSAMHorizonScan(), config.Horizon_SCAN);

		config.imuAccNoise = Parameters::defaultOdomLIOSAMImuAccNoise();
		Parameters::parse(parameters_, Parameters::kOdomLIOSAMImuAccNoise(), config.imuAccNoise);

		config.imuGyrNoise = Parameters::defaultOdomLIOSAMImuGyrNoise();
		Parameters::parse(parameters_, Parameters::kOdomLIOSAMImuGyrNoise(), config.imuGyrNoise);

		config.imuAccBiasN = Parameters::defaultOdomLIOSAMImuAccBiasN();
		Parameters::parse(parameters_, Parameters::kOdomLIOSAMImuAccBiasN(), config.imuAccBiasN);

		config.imuGyrBiasN = Parameters::defaultOdomLIOSAMImuGyrBiasN();
		Parameters::parse(parameters_, Parameters::kOdomLIOSAMImuGyrBiasN(), config.imuGyrBiasN);

		config.imuGravity = Parameters::defaultOdomLIOSAMImuGravity();
		Parameters::parse(parameters_, Parameters::kOdomLIOSAMImuGravity(), config.imuGravity);

		config.edgeThreshold = Parameters::defaultOdomLIOSAMEdgeThreshold();
		Parameters::parse(parameters_, Parameters::kOdomLIOSAMEdgeThreshold(), config.edgeThreshold);

		config.surfThreshold = Parameters::defaultOdomLIOSAMSurfThreshold();
		Parameters::parse(parameters_, Parameters::kOdomLIOSAMSurfThreshold(), config.surfThreshold);

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
	}

	// Always override extrinsics from sensor local transforms when available.
	// This ensures the IMU-to-lidar transform matches the actual sensor setup
	// regardless of what the config file says.
	//   imuLocalTransform   = T_base_imu   (base_link -> imu_link)
	//   lidarLocalTransform = T_base_lidar  (base_link -> lidar_link)
	// LIO-SAM's imuConverter() expects T_lidar_imu:
	//   T_lidar_imu = T_base_lidar^{-1} * T_base_imu
	if(!imuLocalTransform.isNull() && !lidarLocalTransform.isNull())
	{
		Transform T_lidar_imu = lidarLocalTransform.inverse() * imuLocalTransform;
		Eigen::Matrix4d T = T_lidar_imu.toEigen4d();
		Eigen::Matrix3d rot = T.block<3,3>(0,0);
		Eigen::Vector3d trans = T.block<3,1>(0,3);
		config.extRotV = {rot(0,0), rot(0,1), rot(0,2),
		                  rot(1,0), rot(1,1), rot(1,2),
		                  rot(2,0), rot(2,1), rot(2,2)};
		config.extRPYV = config.extRotV;
		config.extTransV = {trans(0), trans(1), trans(2)};
		UINFO("LIO-SAM extrinsics (T_lidar_imu) computed from sensor local transforms: %s", T_lidar_imu.prettyPrint().c_str());
	}
	else if(config.extRotV.size() != 9 || config.extTransV.size() != 3)
	{
		// No valid extrinsics from sensor data or config file
		UERROR("Cannot compute IMU-to-lidar extrinsics: IMU local transform %s, lidar local transform %s. "
		       "Both must be valid, or the config file must contain valid extrinsics.",
		       imuLocalTransform.isNull() ? "is null" : "is valid",
		       lidarLocalTransform.isNull() ? "is null" : "is valid");
		return false;
	}
	else
	{
		UINFO("Using extrinsics from config file (sensor local transforms not available)");
	}

	// Set the global extrinsics used by imuConverter
	extRot = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(config.extRotV.data());
	extRPY = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(config.extRPYV.data());
	extTrans = Eigen::Map<const Eigen::Matrix<double, 3, 1> >(config.extTransV.data());
	extQRPY = Eigen::Quaterniond(extRPY).inverse();

	lioSam_ = new lio_sam::LioSamCore(config);

	// Replay buffered IMU samples
	UINFO("Replaying %d buffered IMU samples into LIO-SAM", (int)imuBuffer_.size());
	for(const ImuSample & s : imuBuffer_)
	{
		lioSam_->addImu(s.stamp, s.acc, s.gyro, s.orientation);
	}
	imuBuffer_.clear();

	return true;
}
#endif

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

		// Deferred initialization: need both IMU and lidar local transforms
		// to compute T_lidar_imu extrinsics for LIO-SAM.
		if(!lioSam_)
		{
			// Cache IMU local transform when first available
			if(imuLocalTransform_.isNull() && !data.imu().localTransform().isNull())
			{
				imuLocalTransform_ = data.imu().localTransform();
			}

			// Try to initialize if we have both transforms
			if(!imuLocalTransform_.isNull() && !data.laserScanRaw().isEmpty() &&
			   !data.laserScanRaw().localTransform().isNull())
			{
				if(!init(imuLocalTransform_, data.laserScanRaw().localTransform()))
				{
					UERROR("Failed to initialize LIO-SAM");
					return t;
				}
			}
			else
			{
				// Buffer IMU until we can initialize
				ImuSample s;
				s.stamp = data.stamp();
				s.acc = acc;
				s.gyro = gyro;
				s.orientation = qd;
				imuBuffer_.push_back(s);

				if(data.laserScanRaw().isEmpty())
				{
					return t;
				}
			}
		}

		if(lioSam_)
		{
			lioSam_->addImu(data.stamp(), acc, gyro, qd);
		}

		// IMU-only: no pose to return
		if(data.laserScanRaw().isEmpty())
		{
			return t;
		}
	}

	if(!lioSam_)
	{
		// A scan arrived without IMU in the same message.
		// Try to init if the IMU local transform was already cached.
		if(!imuLocalTransform_.isNull() && !data.laserScanRaw().isEmpty() &&
		   !data.laserScanRaw().localTransform().isNull())
		{
			if(!init(imuLocalTransform_, data.laserScanRaw().localTransform()))
			{
				UERROR("Failed to initialize LIO-SAM");
				return t;
			}
		}
		else
		{
			UDEBUG("LIO-SAM not yet initialized, waiting for IMU (have=%s) and lidar (need scan) local transforms...",
				imuLocalTransform_.isNull() ? "no" : "yes");
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
		const LaserScan & scan = data.laserScanRaw();
		if(scan.format() != LaserScan::kXYZIRT)
		{
			UERROR("LIO-SAM requires a scan in format %s (got %s). "
			       "Populate the scan via util3d::laserScanFromPointCloud<PointXYZIRT>() "
			       "so that per-point ring and time fields are available.",
			       LaserScan::formatName(LaserScan::kXYZIRT).c_str(),
			       scan.formatName().c_str());
			return t;
		}

		// Split the kXYZIRT scan into the three parallel buffers LIO-SAM expects.
		const int numPoints = scan.size();
		const int ringOffset = scan.getRingOffset();
		const int timeOffset = scan.getTimeOffset();
		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>);
		laserCloudIn->reserve(numPoints);
		std::vector<int> rings;
		std::vector<float> times;
		rings.reserve(numPoints);
		times.reserve(numPoints);
		for(int i=0; i<numPoints; ++i)
		{
			const int row = i / scan.data().cols;
			const int col = i - row * scan.data().cols;
			const float * ptr = scan.data().ptr<float>(row, col);
			pcl::PointXYZI pt;
			pt.x = ptr[0];
			pt.y = ptr[1];
			pt.z = ptr[2];
			pt.intensity = ptr[3];
			laserCloudIn->push_back(pt);
			rings.push_back(static_cast<int>(ptr[ringOffset]));
			times.push_back(ptr[timeOffset]);
		}
		UDEBUG("Scan split: %fs, points=%d", timer.ticks(), (int)laserCloudIn->size());

		// Process scan. Retrieve the deskewed (motion-compensated) cloud
		// produced by LIO-SAM's image projection stage so we can propagate
		// it back into SensorData: otherwise downstream consumers such as
		// loop closure registration would still see the raw pre-deskew scan.
		Eigen::Affine3f poseOut;
		Eigen::MatrixXd covOut;
		pcl::PointCloud<pcl::PointXYZI>::Ptr deskewedCloud(new pcl::PointCloud<pcl::PointXYZI>);
		bool ok = lioSam_->processScan(data.stamp(), laserCloudIn, rings, times, poseOut, covOut, deskewedCloud);
		UDEBUG("LIO-SAM process: %fs", timer.ticks());

		if(ok)
		{
			// Replace the raw scan on SensorData with LIO-SAM's deskewed
			// cloud so downstream stages (loop closure registration in
			// particular) use the motion-compensated points instead of
			// the raw pre-deskew scan. The deskewed cloud is still in the
			// lidar frame, so the existing localTransform/rangeMax apply.
			if(deskewedCloud && !deskewedCloud->empty())
			{
				const LaserScan & rawScan = data.laserScanRaw();
				LaserScan deskewedScan(
					util3d::laserScanFromPointCloud(*deskewedCloud),
					rawScan.maxPoints(),
					rawScan.rangeMax(),
					rawScan.localTransform());
				data.setLaserScan(deskewedScan);
				UDEBUG("Replaced raw scan with deskewed cloud (%d -> %d points)",
					(int)laserCloudIn->size(), (int)deskewedCloud->size());
			}

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
