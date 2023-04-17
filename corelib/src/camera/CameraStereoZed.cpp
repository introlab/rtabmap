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

#include <rtabmap/core/camera/CameraStereoZed.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>

#ifdef RTABMAP_ZED
#include <sl/Camera.hpp>
#endif

namespace rtabmap
{

#ifdef RTABMAP_ZED

static cv::Mat slMat2cvMat(sl::Mat& input) {
#if ZED_SDK_MAJOR_VERSION < 3
	//convert MAT_TYPE to CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}
	// cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	//cv::Mat and sl::Mat will share the same memory pointer
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
#else
    //convert MAT_TYPE to CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
    case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
    case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
    case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
    case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
    case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
    case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
    case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
    case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
    default: break;
    }
    // cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    //cv::Mat and sl::Mat will share the same memory pointer
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
#endif
}

Transform zedPoseToTransform(const sl::Pose & pose)
{
	return Transform(
			pose.pose_data.m[0], pose.pose_data.m[1], pose.pose_data.m[2], pose.pose_data.m[3],
			pose.pose_data.m[4], pose.pose_data.m[5], pose.pose_data.m[6], pose.pose_data.m[7],
			pose.pose_data.m[8], pose.pose_data.m[9], pose.pose_data.m[10], pose.pose_data.m[11]);
}

#if ZED_SDK_MAJOR_VERSION < 3
IMU zedIMUtoIMU(const sl::IMUData & imuData, const Transform & imuLocalTransform)
{
	sl::Orientation orientation = imuData.pose_data.getOrientation();

	//Convert zed imu orientation from camera frame to world frame ENU!
	Transform opticalTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
	Transform orientationT(0,0,0, orientation.ox, orientation.oy, orientation.oz, orientation.ow);
	orientationT = opticalTransform * orientationT;

	static double deg2rad = 0.017453293;
	Eigen::Vector4d accT = Eigen::Vector4d(imuData.linear_acceleration.v[0], imuData.linear_acceleration.v[1], imuData.linear_acceleration.v[2], 1);
	Eigen::Vector4d gyrT = Eigen::Vector4d(imuData.angular_velocity.v[0]*deg2rad, imuData.angular_velocity.v[1]*deg2rad, imuData.angular_velocity.v[2]*deg2rad, 1);

	cv::Mat orientationCov = (cv::Mat_<double>(3,3)<<
			imuData.pose_covariance[21], imuData.pose_covariance[22], imuData.pose_covariance[23],
			imuData.pose_covariance[27], imuData.pose_covariance[28], imuData.pose_covariance[29],
			imuData.pose_covariance[33], imuData.pose_covariance[34], imuData.pose_covariance[35]);
	cv::Mat angCov = (cv::Mat_<double>(3,3)<<
			imuData.angular_velocity_convariance.r[0], imuData.angular_velocity_convariance.r[1], imuData.angular_velocity_convariance.r[2],
			imuData.angular_velocity_convariance.r[3], imuData.angular_velocity_convariance.r[4], imuData.angular_velocity_convariance.r[5],
			imuData.angular_velocity_convariance.r[6], imuData.angular_velocity_convariance.r[7], imuData.angular_velocity_convariance.r[8]);
	cv::Mat accCov = (cv::Mat_<double>(3,3)<<
			imuData.linear_acceleration_convariance.r[0], imuData.linear_acceleration_convariance.r[1], imuData.linear_acceleration_convariance.r[2],
			imuData.linear_acceleration_convariance.r[3], imuData.linear_acceleration_convariance.r[4], imuData.linear_acceleration_convariance.r[5],
			imuData.linear_acceleration_convariance.r[6], imuData.linear_acceleration_convariance.r[7], imuData.linear_acceleration_convariance.r[8]);

	Eigen::Quaternionf quat = orientationT.getQuaternionf();

	return IMU(
			cv::Vec4d(quat.x(), quat.y(), quat.z(), quat.w()),
			orientationCov,
			cv::Vec3d(gyrT[0], gyrT[1], gyrT[2]),
			angCov,
			cv::Vec3d(accT[0], accT[1], accT[2]),
			accCov,
			imuLocalTransform);
}
#else
IMU zedIMUtoIMU(const sl::SensorsData & sensorData, const Transform & imuLocalTransform)
{
    sl::Orientation orientation = sensorData.imu.pose.getOrientation();

    //Convert zed imu orientation from camera frame to world frame ENU!
    Transform opticalTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
    Transform orientationT(0,0,0, orientation.ox, orientation.oy, orientation.oz, orientation.ow);
    orientationT = opticalTransform * orientationT;

    static double deg2rad = 0.017453293;
    Eigen::Vector4d accT = Eigen::Vector4d(sensorData.imu.linear_acceleration.v[0], sensorData.imu.linear_acceleration.v[1], sensorData.imu.linear_acceleration.v[2], 1);
    Eigen::Vector4d gyrT = Eigen::Vector4d(sensorData.imu.angular_velocity.v[0]*deg2rad, sensorData.imu.angular_velocity.v[1]*deg2rad, sensorData.imu.angular_velocity.v[2]*deg2rad, 1);

    cv::Mat orientationCov = (cv::Mat_<double>(3,3)<<
            sensorData.imu.pose_covariance.r[0], sensorData.imu.pose_covariance.r[1], sensorData.imu.pose_covariance.r[2],
            sensorData.imu.pose_covariance.r[3], sensorData.imu.pose_covariance.r[4], sensorData.imu.pose_covariance.r[5],
            sensorData.imu.pose_covariance.r[6], sensorData.imu.pose_covariance.r[7], sensorData.imu.pose_covariance.r[8]);
    cv::Mat angCov = (cv::Mat_<double>(3,3)<<
            sensorData.imu.angular_velocity_covariance.r[0], sensorData.imu.angular_velocity_covariance.r[1], sensorData.imu.angular_velocity_covariance.r[2],
            sensorData.imu.angular_velocity_covariance.r[3], sensorData.imu.angular_velocity_covariance.r[4], sensorData.imu.angular_velocity_covariance.r[5],
            sensorData.imu.angular_velocity_covariance.r[6], sensorData.imu.angular_velocity_covariance.r[7], sensorData.imu.angular_velocity_covariance.r[8]);
    cv::Mat accCov = (cv::Mat_<double>(3,3)<<
            sensorData.imu.linear_acceleration_covariance.r[0], sensorData.imu.linear_acceleration_covariance.r[1], sensorData.imu.linear_acceleration_covariance.r[2],
            sensorData.imu.linear_acceleration_covariance.r[3], sensorData.imu.linear_acceleration_covariance.r[4], sensorData.imu.linear_acceleration_covariance.r[5],
            sensorData.imu.linear_acceleration_covariance.r[6], sensorData.imu.linear_acceleration_covariance.r[7], sensorData.imu.linear_acceleration_covariance.r[8]);

    Eigen::Quaternionf quat = orientationT.getQuaternionf();

    return IMU(
            cv::Vec4d(quat.x(), quat.y(), quat.z(), quat.w()),
            orientationCov,
            cv::Vec3d(gyrT[0], gyrT[1], gyrT[2]),
            angCov,
            cv::Vec3d(accT[0], accT[1], accT[2]),
            accCov,
            imuLocalTransform);
}
#endif

class ZedIMUThread: public UThread
{
public:
	ZedIMUThread(float rate, sl::Camera * zed, const Transform & imuLocalTransform, bool accurate)
	{
		UASSERT(rate > 0.0f);
		UASSERT(zed != 0);
		rate_ = rate;
		zed_= zed;
		accurate_ = accurate;
		imuLocalTransform_ = imuLocalTransform;
	}
private:
	virtual void mainLoopBegin()
	{
		frameRateTimer_.start();
	}
	virtual void mainLoop()
	{
		double delay = 1000.0/double(rate_);
		int sleepTime = delay - 1000.0f*frameRateTimer_.getElapsedTime();
		if(sleepTime > 0)
		{
			if(accurate_)
			{
				if(sleepTime > 1)
				{
					uSleep(sleepTime-1);
				}
				// Add precision at the cost of a small overhead
				delay/=1000.0;
				while(frameRateTimer_.getElapsedTime() < delay-0.000001)
				{
					//
				}
			}
			else
			{
				uSleep(sleepTime);
			}
		}
		frameRateTimer_.start();

#if ZED_SDK_MAJOR_VERSION < 3
		sl::IMUData imudata;
		bool res = zed_->getIMUData(imudata, sl::TIME_REFERENCE_IMAGE);
		if(res == sl::SUCCESS && imudata.valid)
		{
			UEventsManager::post(new IMUEvent(zedIMUtoIMU(imudata, imuLocalTransform_), UTimer::now()));
		}
#else
        sl::SensorsData sensordata;
        sl::ERROR_CODE res = zed_->getSensorsData(sensordata, sl::TIME_REFERENCE::IMAGE);
        if(res == sl::ERROR_CODE::SUCCESS && sensordata.imu.is_available)
        {
            UEventsManager::post(new IMUEvent(zedIMUtoIMU(sensordata, imuLocalTransform_), UTimer::now()));
        }
#endif
	}
	float rate_;
	sl::Camera * zed_;
	bool accurate_;
	Transform imuLocalTransform_;
	UTimer frameRateTimer_;
};
#endif

bool CameraStereoZed::available()
{
#ifdef RTABMAP_ZED
	return true;
#else
	return false;
#endif
}


int CameraStereoZed::sdkVersion()
{
#ifdef RTABMAP_ZED
	return ZED_SDK_MAJOR_VERSION;
#else
	return -1;
#endif
}


CameraStereoZed::CameraStereoZed(
		int deviceId,
		int resolution,
		int quality,
		int sensingMode,
		int confidenceThr,
		bool computeOdometry,
		float imageRate,
		const Transform & localTransform,
		bool selfCalibration,
		bool odomForce3DoF,
		int texturenessConfidenceThr) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_ZED
    ,
	zed_(0),
	src_(CameraVideo::kUsbDevice),
	usbDevice_(deviceId),
	svoFilePath_(""),
	resolution_(resolution),
	quality_(quality),
	selfCalibration_(selfCalibration),
	sensingMode_(sensingMode),
	confidenceThr_(confidenceThr),
	texturenessConfidenceThr_(texturenessConfidenceThr),
	computeOdometry_(computeOdometry),
	lost_(true),
	force3DoF_(odomForce3DoF),
	publishInterIMU_(false),
	imuPublishingThread_(0)
#endif
{
	UDEBUG("");
#ifdef RTABMAP_ZED
#if ZED_SDK_MAJOR_VERSION < 4
	if(resolution_ == 3)
	{
		resolution_ = 2;
	}
	else if(resolution_ == 5)
	{
		resolution_ = 3;
	}
#endif
#if ZED_SDK_MAJOR_VERSION < 3
	UASSERT(resolution_ >= sl::RESOLUTION_HD2K && resolution_ <sl::RESOLUTION_LAST);
	UASSERT(quality_ >= sl::DEPTH_MODE_NONE && quality_ <sl::DEPTH_MODE_LAST);
	UASSERT(sensingMode_ >= sl::SENSING_MODE_STANDARD && sensingMode_ <sl::SENSING_MODE_LAST);
	UASSERT(confidenceThr_ >= 0 && confidenceThr_ <=100);
#else
    sl::RESOLUTION res = static_cast<sl::RESOLUTION>(resolution_);
    sl::DEPTH_MODE qual = static_cast<sl::DEPTH_MODE>(quality_);

    UASSERT(res >= sl::RESOLUTION::HD2K && res < sl::RESOLUTION::LAST);
    UASSERT(qual >= sl::DEPTH_MODE::NONE && qual < sl::DEPTH_MODE::LAST);
#if ZED_SDK_MAJOR_VERSION < 4
    sl::SENSING_MODE sens = static_cast<sl::SENSING_MODE>(sensingMode_);
    UASSERT(sens >= sl::SENSING_MODE::STANDARD && sens < sl::SENSING_MODE::LAST);
#else
    UASSERT(sensingMode_ >= 0 && sensingMode_ < 2);
#endif
    UASSERT(confidenceThr_ >= 0 && confidenceThr_ <=100);
    UASSERT(texturenessConfidenceThr_ >= 0 && texturenessConfidenceThr_ <=100);
#endif
#endif
}

CameraStereoZed::CameraStereoZed(
		const std::string & filePath,
		int quality,
		int sensingMode,
		int confidenceThr,
		bool computeOdometry,
		float imageRate,
		const Transform & localTransform,
		bool selfCalibration,
		bool odomForce3DoF,
		int texturenessConfidenceThr) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_ZED
    ,
	zed_(0),
	src_(CameraVideo::kVideoFile),
	usbDevice_(0),
	svoFilePath_(filePath),
	resolution_(2),
	quality_(quality),
	selfCalibration_(selfCalibration),
	sensingMode_(sensingMode),
	confidenceThr_(confidenceThr),
	texturenessConfidenceThr_(texturenessConfidenceThr),
	computeOdometry_(computeOdometry),
	lost_(true),
	force3DoF_(odomForce3DoF),
	publishInterIMU_(false),
	imuPublishingThread_(0)
#endif
{
	UDEBUG("");
#ifdef RTABMAP_ZED
#if ZED_SDK_MAJOR_VERSION < 3
	UASSERT(resolution_ >= sl::RESOLUTION_HD2K && resolution_ <sl::RESOLUTION_LAST);
	UASSERT(quality_ >= sl::DEPTH_MODE_NONE && quality_ <sl::DEPTH_MODE_LAST);
	UASSERT(sensingMode_ >= sl::SENSING_MODE_STANDARD && sensingMode_ <sl::SENSING_MODE_LAST);
	UASSERT(confidenceThr_ >= 0 && confidenceThr_ <=100);
#else
    sl::RESOLUTION res = static_cast<sl::RESOLUTION>(resolution_);
    sl::DEPTH_MODE qual = static_cast<sl::DEPTH_MODE>(quality_);

    UASSERT(res >= sl::RESOLUTION::HD2K && res < sl::RESOLUTION::LAST);
    UASSERT(qual >= sl::DEPTH_MODE::NONE && qual < sl::DEPTH_MODE::LAST);
#if ZED_SDK_MAJOR_VERSION < 4
    sl::SENSING_MODE sens = static_cast<sl::SENSING_MODE>(sensingMode_);
    UASSERT(sens >= sl::SENSING_MODE::STANDARD && sens < sl::SENSING_MODE::LAST);
#else
    UASSERT(sensingMode_ >= 0 && sensingMode_ < 2);
#endif
    UASSERT(confidenceThr_ >= 0 && confidenceThr_ <=100);
    UASSERT(texturenessConfidenceThr_ >= 0 && texturenessConfidenceThr_ <=100);
#endif
#endif
}

CameraStereoZed::~CameraStereoZed()
{
#ifdef RTABMAP_ZED
	if(imuPublishingThread_)
	{
		imuPublishingThread_->join(true);
	}
	delete imuPublishingThread_;
	delete zed_;
#endif
}

void CameraStereoZed::publishInterIMU(bool enabled)
{
#ifdef RTABMAP_ZED
	publishInterIMU_ = enabled;
#endif
}

bool CameraStereoZed::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_ZED
	if(imuPublishingThread_)
	{
		imuPublishingThread_->join(true);
		delete imuPublishingThread_;
		imuPublishingThread_=0;
	}
	if(zed_)
	{
		delete zed_;
		zed_ = 0;
	}
	
	lost_ = true;

	sl::InitParameters param;
	param.camera_resolution=static_cast<sl::RESOLUTION>(resolution_);
	param.camera_fps=getImageRate();	
	param.depth_mode=(sl::DEPTH_MODE)quality_;
#if ZED_SDK_MAJOR_VERSION < 3
    param.camera_linux_id=usbDevice_;
	param.coordinate_units=sl::UNIT_METER;
	param.coordinate_system=(sl::COORDINATE_SYSTEM)sl::COORDINATE_SYSTEM_IMAGE ;
#else
    param.coordinate_units=sl::UNIT::METER;
    param.coordinate_system=sl::COORDINATE_SYSTEM::IMAGE ;
#endif
	param.sdk_verbose=true;
	param.sdk_gpu_id=-1;
	param.depth_minimum_distance=-1;
	param.camera_disable_self_calib=!selfCalibration_;

	sl::ERROR_CODE r = sl::ERROR_CODE::SUCCESS;
	if(src_ == CameraVideo::kVideoFile)
	{
		UINFO("svo file = %s", svoFilePath_.c_str());
		zed_ = new sl::Camera(); // Use in SVO playback mode
        #if ZED_SDK_MAJOR_VERSION < 3
		param.svo_input_filename=svoFilePath_.c_str();
#else
        param.input.setFromSVOFile(svoFilePath_.c_str());
#endif
		r = zed_->open(param);
	}
	else
	{
#if ZED_SDK_MAJOR_VERSION >= 3
        param.input.setFromCameraID(usbDevice_);
#endif
		UINFO("Resolution=%d imagerate=%f device=%d", resolution_, getImageRate(), usbDevice_);
		zed_ = new sl::Camera(); // Use in Live Mode
		r = zed_->open(param);
	}

	if(r!=sl::ERROR_CODE::SUCCESS)
	{
		UERROR("Camera initialization failed: \"%s\"", toString(r).c_str());
		delete zed_;
		zed_ = 0;
		return false;
	}

#if ZED_SDK_MAJOR_VERSION < 3
	UINFO("Init ZED: Mode=%d Unit=%d CoordinateSystem=%d Verbose=false device=-1 minDist=-1 self-calibration=%s vflip=false",
	      quality_, sl::UNIT_METER, sl::COORDINATE_SYSTEM_IMAGE , selfCalibration_?"true":"false");

    if(quality_!=sl::DEPTH_MODE_NONE)
    {
        zed_->setConfidenceThreshold(confidenceThr_);
    }
#else
    UINFO("Init ZED: Mode=%d Unit=%d CoordinateSystem=%d Verbose=false device=-1 minDist=-1 self-calibration=%s vflip=false",
          quality_, sl::UNIT::METER, sl::COORDINATE_SYSTEM::IMAGE , selfCalibration_?"true":"false");
#endif




    UDEBUG("");

	if (computeOdometry_)
	{
#if ZED_SDK_MAJOR_VERSION < 3
		sl::TrackingParameters tparam;
        tparam.enable_spatial_memory=false;
        r = zed_->enableTracking(tparam);
#else
        sl::PositionalTrackingParameters tparam;
        tparam.enable_area_memory=false;
        r = zed_->enablePositionalTracking(tparam);
#endif
        if(r!=sl::ERROR_CODE::SUCCESS)
		{
			UERROR("Camera tracking initialization failed: \"%s\"", toString(r).c_str());
		}
	}

	sl::CameraInformation infos = zed_->getCameraInformation();
#if ZED_SDK_MAJOR_VERSION < 4
	sl::CalibrationParameters *stereoParams = &(infos.calibration_parameters );
#else
	sl::CalibrationParameters *stereoParams = &(infos.camera_configuration.calibration_parameters );
#endif
	sl::Resolution res = stereoParams->left_cam.image_size;

	stereoModel_ = StereoCameraModel(
		stereoParams->left_cam.fx, 
		stereoParams->left_cam.fy, 
		stereoParams->left_cam.cx, 
		stereoParams->left_cam.cy, 
#if ZED_SDK_MAJOR_VERSION < 4
		stereoParams->T[0],//baseline
#else
		stereoParams->getCameraBaseline(),
#endif
		this->getLocalTransform(),
		cv::Size(res.width, res.height));

	UINFO("Calibration: fx=%f, fy=%f, cx=%f, cy=%f, baseline=%f, width=%d, height=%d, transform=%s",
			stereoParams->left_cam.fx,
			stereoParams->left_cam.fy,
			stereoParams->left_cam.cx,
			stereoParams->left_cam.cy,
#if ZED_SDK_MAJOR_VERSION < 4
			stereoParams->T[0],//baseline
#else
			stereoParams->getCameraBaseline(),
#endif
			(int)res.width,
			(int)res.height,
			this->getLocalTransform().prettyPrint().c_str());

#if ZED_SDK_MAJOR_VERSION < 3
	if(infos.camera_model == sl::MODEL_ZED_M)
#else
    if(infos.camera_model != sl::MODEL::ZED)
#endif
	{
#if ZED_SDK_MAJOR_VERSION < 4
		imuLocalTransform_ = this->getLocalTransform() * zedPoseToTransform(infos.camera_imu_transform).inverse();
#else
		imuLocalTransform_ = this->getLocalTransform() * zedPoseToTransform(infos.sensors_configuration.camera_imu_transform).inverse();
#endif
		UINFO("IMU local transform: %s (imu2cam=%s))",
		imuLocalTransform_.prettyPrint().c_str(),
#if ZED_SDK_MAJOR_VERSION < 4
		zedPoseToTransform(infos.camera_imu_transform).prettyPrint().c_str());
#else
		zedPoseToTransform(infos.sensors_configuration.camera_imu_transform).prettyPrint().c_str());
#endif
		if(publishInterIMU_)
		{
			imuPublishingThread_ = new ZedIMUThread(200, zed_, imuLocalTransform_, true);
			imuPublishingThread_->start();
		}
	}

	return true;
#else
	UERROR("CameraStereoZED: RTAB-Map is not built with ZED sdk support!");
#endif
	return false;
}

bool CameraStereoZed::isCalibrated() const
{
#ifdef RTABMAP_ZED
	return stereoModel_.isValidForProjection();
#else
	return false;
#endif
}

std::string CameraStereoZed::getSerial() const
{
#ifdef RTABMAP_ZED
	if(zed_)
	{
		return uFormat("%x", zed_->getCameraInformation ().serial_number);
	}
#endif
	return "";
}

bool CameraStereoZed::odomProvided() const
{
#ifdef RTABMAP_ZED
	return computeOdometry_;
#else
	return false;
#endif
}

bool CameraStereoZed::getPose(double stamp, Transform & pose, cv::Mat & covariance)
{
#ifdef RTABMAP_ZED

	if (computeOdometry_ && zed_)
	{
		sl::Pose p;

#if ZED_SDK_MAJOR_VERSION < 3
		if(!zed_->grab())
		{
			return false;
		}
		sl::TRACKING_STATE tracking_state = zed_->getPosition(p);
		if (tracking_state == sl::TRACKING_STATE_OK)
#else
		if(zed_->grab()!=sl::ERROR_CODE::SUCCESS)
		{
			return false;
		}
		 sl::POSITIONAL_TRACKING_STATE tracking_state = zed_->getPosition(p);
		if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK)
#endif
		{
			int trackingConfidence = p.pose_confidence;
			// FIXME What does pose_confidence == -1 mean?
			if (trackingConfidence>0)
			{
				pose = zedPoseToTransform(p);
				if (!pose.isNull())
				{
					//transform from:
					// x->right, y->down, z->forward
					//to:
					// x->forward, y->left, z->up
					pose = this->getLocalTransform() * pose * this->getLocalTransform().inverse();
					if(force3DoF_)
					{
						pose = pose.to3DoF();
					}
					if (lost_)
					{
						covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // don't know transform with previous pose
						lost_ = false;
						UDEBUG("Init %s (var=%f)", pose.prettyPrint().c_str(), 9999.0f);
					}
					else
					{
						covariance = cv::Mat::eye(6, 6, CV_64FC1) * 1.0f / float(trackingConfidence);
						UDEBUG("Run %s (var=%f)", pose.prettyPrint().c_str(), 1.0f / float(trackingConfidence));
					}
					return true;
				}
				else
				{
					covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // lost
					lost_ = true;
					UWARN("ZED lost! (trackingConfidence=%d)", trackingConfidence);
				}
			}
			else
			{
				covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // lost
				lost_ = true;
				UWARN("ZED lost! (trackingConfidence=%d)", trackingConfidence);
			}
		}
		else
		{
			UWARN("Tracking not ok: state=\"%s\"", toString(tracking_state).c_str());
		}
	}
#endif
	return false;
}

SensorData CameraStereoZed::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_ZED
#if ZED_SDK_MAJOR_VERSION < 3
	sl::RuntimeParameters rparam((sl::SENSING_MODE)sensingMode_, quality_ > 0, quality_ > 0, sl::REFERENCE_FRAME_CAMERA);
#elif ZED_SDK_MAJOR_VERSION < 4
    sl::RuntimeParameters rparam((sl::SENSING_MODE)sensingMode_, quality_ > 0, confidenceThr_, texturenessConfidenceThr_, sl::REFERENCE_FRAME::CAMERA);
#else
    sl::RuntimeParameters rparam(quality_ > 0, sensingMode_ == 1, confidenceThr_, texturenessConfidenceThr_, sl::REFERENCE_FRAME::CAMERA);
#endif

	if(zed_)
	{
		UTimer timer;
#if ZED_SDK_MAJOR_VERSION < 3
		bool res = zed_->grab(rparam);
		while (src_ == CameraVideo::kUsbDevice && res!=sl::SUCCESS && timer.elapsed() < 2.0)
		{
			// maybe there is a latency with the USB, try again in 10 ms (for the next 2 seconds)
			uSleep(10);
			res = zed_->grab(rparam);
		}

        if(res==sl::SUCCESS)
#else

		sl::ERROR_CODE res;
		bool imuReceived = true;
		do
		{
			res = zed_->grab(rparam);

			// If the sensor supports IMU, wait IMU to be available before sending data.
			if(imuPublishingThread_ == 0 && !imuLocalTransform_.isNull())
			{
				 sl::SensorsData imudatatmp;
				res = zed_->getSensorsData(imudatatmp, sl::TIME_REFERENCE::IMAGE);
				imuReceived = res == sl::ERROR_CODE::SUCCESS && imudatatmp.imu.is_available && imudatatmp.imu.timestamp.data_ns != 0;
			}
		}
		while(src_ == CameraVideo::kUsbDevice && (res!=sl::ERROR_CODE::SUCCESS || !imuReceived) && timer.elapsed() < 2.0);

        if(res==sl::ERROR_CODE::SUCCESS)
#endif
		{
			// get left image
			sl::Mat tmp;
#if ZED_SDK_MAJOR_VERSION < 3
			zed_->retrieveImage(tmp,sl::VIEW_LEFT);
#else
            zed_->retrieveImage(tmp,sl::VIEW::LEFT);
#endif
			cv::Mat rgbaLeft = slMat2cvMat(tmp);

			cv::Mat left;
			cv::cvtColor(rgbaLeft, left, cv::COLOR_BGRA2BGR);

			if(quality_ > 0)
			{
				// get depth image
				cv::Mat depth;
				sl::Mat tmp;
#if ZED_SDK_MAJOR_VERSION < 3
				zed_->retrieveMeasure(tmp,sl::MEASURE_DEPTH);
#else
                zed_->retrieveMeasure(tmp,sl::MEASURE::DEPTH);
#endif
				slMat2cvMat(tmp).copyTo(depth);

				data = SensorData(left, depth, stereoModel_.left(), this->getNextSeqID(), UTimer::now());
			}
			else
			{
				// get right image
#if ZED_SDK_MAJOR_VERSION < 3
				sl::Mat tmp;zed_->retrieveImage(tmp,sl::VIEW_RIGHT );
#else
                sl::Mat tmp;zed_->retrieveImage(tmp,sl::VIEW::RIGHT );
#endif
				cv::Mat rgbaRight = slMat2cvMat(tmp);
				cv::Mat right;
				cv::cvtColor(rgbaRight, right, cv::COLOR_BGRA2GRAY);
			
				data = SensorData(left, right, stereoModel_, this->getNextSeqID(), UTimer::now());
			}

			if(imuPublishingThread_ == 0)
			{
#if ZED_SDK_MAJOR_VERSION < 3
				sl::IMUData imudata;
				res = zed_->getIMUData(imudata, sl::TIME_REFERENCE_IMAGE);
				if(res == sl::SUCCESS && imudata.valid)
#else
                sl::SensorsData imudata;
                res = zed_->getSensorsData(imudata, sl::TIME_REFERENCE::IMAGE);
                if(res == sl::ERROR_CODE::SUCCESS && imudata.imu.is_available)
#endif
				{
					//ZED-Mini
					data.setIMU(zedIMUtoIMU(imudata, imuLocalTransform_));
				}
			}

			if (computeOdometry_ && info)
			{
				sl::Pose pose;
#if ZED_SDK_MAJOR_VERSION < 3
				sl::TRACKING_STATE tracking_state = zed_->getPosition(pose);
				if (tracking_state == sl::TRACKING_STATE_OK)
#else
                sl::POSITIONAL_TRACKING_STATE tracking_state = zed_->getPosition(pose);
                if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK)
#endif
				{
					int trackingConfidence = pose.pose_confidence;
					// FIXME What does pose_confidence == -1 mean?
					info->odomPose = zedPoseToTransform(pose);
					if (!info->odomPose.isNull())
					{
						//transform from:
						// x->right, y->down, z->forward
						//to:
						// x->forward, y->left, z->up
						info->odomPose = this->getLocalTransform() * info->odomPose * this->getLocalTransform().inverse();
						if(force3DoF_)
						{
							info->odomPose = info->odomPose.to3DoF();
						}
						if (lost_)
						{
							info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // don't know transform with previous pose
							lost_ = false;
							UDEBUG("Init %s (var=%f)", info->odomPose.prettyPrint().c_str(), 9999.0f);
						}
						else if(trackingConfidence==0)
						{
							info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // lost
							lost_ = true;
							UWARN("ZED lost! (trackingConfidence=%d)", trackingConfidence);
						}
						else
						{
							info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 1.0f / float(trackingConfidence);
							UDEBUG("Run %s (var=%f)", info->odomPose.prettyPrint().c_str(), 1.0f / float(trackingConfidence));
						}
					}
					else
					{
						info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // lost
						lost_ = true;
						UWARN("ZED lost! (trackingConfidence=%d)", trackingConfidence);
					}
				}
				else
				{
					UWARN("Tracking not ok: state=\"%s\"", toString(tracking_state).c_str());
				}
			}
		}
		else if(src_ == CameraVideo::kUsbDevice)
		{
			UERROR("CameraStereoZed: Failed to grab images after 2 seconds!");
		}
		else
		{
			UWARN("CameraStereoZed: end of stream is reached!");
		}
	}
#else
	UERROR("CameraStereoZED: RTAB-Map is not built with ZED sdk support!");
#endif
	return data;
}

} // namespace rtabmap
