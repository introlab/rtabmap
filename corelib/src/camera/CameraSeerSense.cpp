#include <rtabmap/core/camera/CameraSeerSense.h>
#include <rtabmap/utilite/UEventsManager.h>

namespace rtabmap {

bool CameraSeerSense::available()
{
#ifdef RTABMAP_XVSDK
	return true;
#else
	return false;
#endif
}

CameraSeerSense::CameraSeerSense(float imageRate, const Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_XVSDK
	,
	imuPublished_(true),
	publishInterIMU_(false)
#endif
{
#ifdef RTABMAP_XVSDK
	xv::setLogLevel(xv::LogLevel(ULogger::level()+1));
#endif
}

CameraSeerSense::~CameraSeerSense()
{
}

void CameraSeerSense::setIMU(bool imuPublished, bool publishInterIMU)
{
#ifdef RTABMAP_XVSDK
	imuPublished_ = imuPublished;
	publishInterIMU_ = publishInterIMU;
#else
	UERROR("CameraSeerSense: RTAB-Map is not built with XVisio SDK support!");
#endif
}

bool CameraSeerSense::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_XVSDK
	auto devices = xv::getDevices(3.);
	if(devices.empty())
	{
		UERROR("Timeout for SeerSense device detection.");
		return false;
	}
	device_ = devices.begin()->second;

	if(imuPublished_ && device_->imuSensor())
	{
		imuLocalTransform_ = this->getLocalTransform() * imuLocalTransform_;
		UINFO("IMU local transform = %s", imuLocalTransform_.prettyPrint().c_str());
		device_->imuSensor()->registerCallback([this](const xv::Imu & xvImu) {
			if(publishInterIMU_)
			{
				IMU imu(cv::Vec3d(xvImu.gyro[0], xvImu.gyro[1], xvImu.gyro[2]), cv::Mat::eye(3,3,CV_64FC1),
						cv::Vec3d(xvImu.accel[0], xvImu.accel[1], xvImu.accel[2]), cv::Mat::eye(3,3,CV_64FC1),
						imuLocalTransform_);
				UEventsManager::post(new IMUEvent(imu, xvImu.hostTimestamp));
			}
			else
			{
				UScopeMutex lock(imuMutex_);
				accBuffer_.emplace_hint(accBuffer_.end(), xvImu.hostTimestamp, cv::Vec3d(xvImu.accel[0], xvImu.accel[1], xvImu.accel[2]));
				gyroBuffer_.emplace_hint(gyroBuffer_.end(), xvImu.hostTimestamp, cv::Vec3d(xvImu.gyro[0], xvImu.gyro[1], xvImu.gyro[2]));
			}
		});
	}

	return true;
#else
	UERROR("CameraSeerSense: RTAB-Map is not built with XVisio SDK support!");
#endif
	return false;
}

} // namespace rtabmap
