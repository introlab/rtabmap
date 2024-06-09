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

CameraSeerSense::CameraSeerSense(bool computeOdometry, float imageRate, const Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_XVSDK
	,
	computeOdometry_(computeOdometry),
	imuId_(0),
	tofId_(0)
#endif
{
#ifdef RTABMAP_XVSDK
	xv::setLogLevel(xv::LogLevel(ULogger::level()+1));
#endif
}

CameraSeerSense::~CameraSeerSense()
{
#ifdef RTABMAP_XVSDK
	if(imuId_)
		device_->imuSensor()->unregisterCallback(imuId_);
	
	if(tofId_)
		device_->tofCamera()->unregisterColorDepthImageCallback(tofId_);

	if(device_->slam())
        device_->slam()->stop();

    if(device_->imuSensor())
		device_->imuSensor()->stop();

    if(device_->colorCamera())
		device_->colorCamera()->stop();

    if(device_->tofCamera())
		device_->tofCamera()->stop();

	dataReady_.release();
#endif
}

bool CameraSeerSense::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_XVSDK
	auto devices = xv::getDevices(3);
	if(devices.empty())
	{
		UERROR("Timeout for SeerSense device detection.");
		return false;
	}
	device_ = devices.begin()->second;

	UASSERT(device_->imuSensor());
	device_->imuSensor()->start();
	imuId_ = device_->imuSensor()->registerCallback([this](const xv::Imu & xvImu) {
		if(xvImu.hostTimestamp > 0)
		{
			if(isInterIMUPublishing())
			{
				IMU imu(cv::Vec3d(xvImu.gyro[0], xvImu.gyro[1], xvImu.gyro[2]), cv::Mat::eye(3,3,CV_64FC1),
						cv::Vec3d(xvImu.accel[0], xvImu.accel[1], xvImu.accel[2]), cv::Mat::eye(3,3,CV_64FC1),
						this->getLocalTransform());
				this->postInterIMU(imu, xvImu.hostTimestamp);
			}
			else
			{
				UScopeMutex lock(imuMutex_);
				imuBuffer_.emplace_hint(imuBuffer_.end(), xvImu.hostTimestamp,
					std::make_pair(cv::Vec3d(xvImu.gyro[0], xvImu.gyro[1], xvImu.gyro[2]), cv::Vec3d(xvImu.accel[0], xvImu.accel[1], xvImu.accel[2])));
			}
		}
	});

	if(computeOdometry_)
	{
		UASSERT(device_->slam());
		device_->slam()->start(xv::Slam::Mode::Mixed);
	}

	auto frameRate = xv::TofCamera::Framerate::FPS_30;
	if(this->getImageRate() > 25)
		frameRate = xv::TofCamera::Framerate::FPS_30;
	else if(this->getImageRate() > 20)
		frameRate = xv::TofCamera::Framerate::FPS_25;
	else if(this->getImageRate() > 15)
		frameRate = xv::TofCamera::Framerate::FPS_20;
	else if(this->getImageRate() > 10)
		frameRate = xv::TofCamera::Framerate::FPS_15;
	else if(this->getImageRate() > 5)
		frameRate = xv::TofCamera::Framerate::FPS_10;
	else if(this->getImageRate() > 0)
		frameRate = xv::TofCamera::Framerate::FPS_5;

	UASSERT(device_->colorCamera());
	device_->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_640x480);
	device_->colorCamera()->start();
	UASSERT(device_->tofCamera());
	device_->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::IQMIX_DF, xv::TofCamera::Resolution::QVGA, frameRate);
	device_->tofCamera()->start();

	UASSERT(!device_->tofCamera()->calibration().empty());
	auto xvTofCalib = device_->tofCamera()->calibration()[0];
	UASSERT(!xvTofCalib.pdcm.empty());
	cv::Mat D(1, xvTofCalib.pdcm[0].distor.size(), CV_64FC1, xvTofCalib.pdcm[0].distor.begin());
	cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat P = cv::Mat::eye(3, 4, CV_64FC1);
	P.at<double>(0,0) = xvTofCalib.pdcm[0].fx;
	P.at<double>(1,1) = xvTofCalib.pdcm[0].fy;
	P.at<double>(0,2) = xvTofCalib.pdcm[0].u0;
	P.at<double>(1,2) = xvTofCalib.pdcm[0].v0;
	cv::Mat K = P.colRange(0, 3);
	cameraModel_ = CameraModel(device_->id(), cv::Size(xvTofCalib.pdcm[0].w, xvTofCalib.pdcm[0].h), K, D, R, P,
		this->getLocalTransform() * Transform(
			xvTofCalib.pose.rotation()[0], xvTofCalib.pose.rotation()[1], xvTofCalib.pose.rotation()[2], xvTofCalib.pose.translation()[0],
			xvTofCalib.pose.rotation()[3], xvTofCalib.pose.rotation()[4], xvTofCalib.pose.rotation()[5], xvTofCalib.pose.translation()[1],
			xvTofCalib.pose.rotation()[6], xvTofCalib.pose.rotation()[7], xvTofCalib.pose.rotation()[8], xvTofCalib.pose.translation()[2]
		)).scaled(0.5);
	UASSERT(cameraModel_.isValidForRectification());
	cameraModel_.initRectificationMap();

	lastData_ = std::pair<double, std::pair<cv::Mat, cv::Mat>>();
	tofId_ = device_->tofCamera()->registerColorDepthImageCallback([this](const xv::DepthColorImage & xvDepthColorImage) {
		if(xvDepthColorImage.hostTimestamp > 0)
		{
			cv::Mat color = cv::Mat::zeros(cameraModel_.imageSize(), CV_8UC3);
			color.forEach<cv::Vec3b>([&](cv::Vec3b& pixel, const int position[]) -> void {
				const auto rgb = reinterpret_cast<std::uint8_t const *>(xvDepthColorImage.data.get() + (position[0]*cameraModel_.imageWidth()+position[1]) * 7);
				pixel = cv::Vec3b(rgb[2], rgb[1], rgb[0]);
			});
			cv::Mat depth = cv::Mat::zeros(cameraModel_.imageSize(), CV_32FC1);
			depth.forEach<float>([&](float &pixel, const int position[]) -> void {
				pixel = *reinterpret_cast<float const *>(xvDepthColorImage.data.get() + (position[0]*cameraModel_.imageWidth()+position[1]) * 7 + 3);
			});
			UScopeMutex lock(dataMutex_);
			bool notify = !lastData_.first;
			lastData_ = std::make_pair(xvDepthColorImage.hostTimestamp, std::make_pair(color, depth));
			if(notify)
				dataReady_.release();
		}
	});

	return true;
#else
	UERROR("CameraSeerSense: RTAB-Map is not built with XVisio SDK support!");
#endif
	return false;
}

bool CameraSeerSense::isCalibrated() const
{
	return true;
}

std::string CameraSeerSense::getSerial() const
{
#ifdef RTABMAP_XVSDK
	return device_->id();
#endif
	return "";
}

bool CameraSeerSense::odomProvided() const
{
#ifdef RTABMAP_XVSDK
	return computeOdometry_;
#else
	return false;
#endif
}

bool CameraSeerSense::getPose(double stamp, Transform & pose, cv::Mat & covariance, double maxWaitTime)
{
#ifdef RTABMAP_XVSDK
	xv::Pose xvPose;
	if(computeOdometry_ && device_->slam()->getPoseAt(xvPose, stamp))
	{
		pose = this->getLocalTransform() *
		Transform(
			xvPose.transform().rotation()[0], xvPose.transform().rotation()[1], xvPose.transform().rotation()[2], xvPose.transform().translation()[0],
			xvPose.transform().rotation()[3], xvPose.transform().rotation()[4], xvPose.transform().rotation()[5], xvPose.transform().translation()[1],
			xvPose.transform().rotation()[6], xvPose.transform().rotation()[7], xvPose.transform().rotation()[8], xvPose.transform().translation()[2]) *
		this->getLocalTransform().inverse();
		covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.0005;
		return true;
	}
#endif
	return false;
}

SensorData CameraSeerSense::captureImage(SensorCaptureInfo * info)
{
	SensorData data;
#ifdef RTABMAP_XVSDK
	if(!dataReady_.acquire(1, 3000))
	{
		UERROR("Did not receive frame since 3 seconds...");
		return data;
	}

	dataMutex_.lock();
	data = SensorData(
		cameraModel_.rectifyImage(lastData_.second.first, cv::INTER_CUBIC),
		cameraModel_.rectifyImage(lastData_.second.second, cv::INTER_NEAREST),
		cameraModel_, this->getNextSeqID(), lastData_.first);
	lastData_ = std::pair<double, std::pair<cv::Mat, cv::Mat>>();
	dataMutex_.unlock();

	if(!isInterIMUPublishing())
	{
		cv::Vec3d gyro, acc;
		std::map<double, std::pair<cv::Vec3d, cv::Vec3d>>::const_iterator iterA, iterB;

		imuMutex_.lock();
		while(imuBuffer_.empty() || imuBuffer_.rbegin()->first < data.stamp())
		{
			imuMutex_.unlock();
			uSleep(1);
			imuMutex_.lock();
		}

		iterB = imuBuffer_.lower_bound(data.stamp());
		iterA = iterB;
		if(iterA != imuBuffer_.begin())
			iterA = --iterA;
		if(iterA == iterB || data.stamp() == iterB->first)
		{
			gyro = iterB->second.first;
			acc = iterB->second.second;
		}
		else if(data.stamp() > iterA->first && data.stamp() < iterB->first)
		{
			float t = (data.stamp()-iterA->first) / (iterB->first-iterA->first);
			gyro = iterA->second.first + t*(iterB->second.first - iterA->second.first);
			acc = iterA->second.second + t*(iterB->second.second - iterA->second.second);
		}
		imuBuffer_.erase(imuBuffer_.begin(), iterB);

		imuMutex_.unlock();
		data.setIMU(IMU(gyro, cv::Mat::eye(3, 3, CV_64FC1), acc, cv::Mat::eye(3, 3, CV_64FC1), this->getLocalTransform()));
	}

	xv::Pose xvPose;
	if(computeOdometry_ && device_->slam()->getPoseAt(xvPose, data.stamp()))
	{
		info->odomPose = this->getLocalTransform() *
		Transform(
			xvPose.transform().rotation()[0], xvPose.transform().rotation()[1], xvPose.transform().rotation()[2], xvPose.transform().translation()[0],
			xvPose.transform().rotation()[3], xvPose.transform().rotation()[4], xvPose.transform().rotation()[5], xvPose.transform().translation()[1],
			xvPose.transform().rotation()[6], xvPose.transform().rotation()[7], xvPose.transform().rotation()[8], xvPose.transform().translation()[2]) *
		this->getLocalTransform().inverse();
		info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.0005;
	}
#else
	UERROR("CameraSeerSense: RTAB-Map is not built with XVisio SDK support!");
#endif
	return data;
}

} // namespace rtabmap
