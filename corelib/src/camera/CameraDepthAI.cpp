/*
Copyright (c) 2010-2021, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap/core/camera/CameraDepthAI.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>


namespace rtabmap {

bool CameraDepthAI::available()
{
#ifdef RTABMAP_DEPTHAI
	return true;
#else
	return false;
#endif
}

CameraDepthAI::CameraDepthAI(
		const std::string & mxidOrName,
		int resolution,
		float imageRate,
		const Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_DEPTHAI
	,
	mxidOrName_(mxidOrName),
	outputDepth_(false),
	depthConfidence_(200),
	resolution_(resolution),
	alphaScaling_(0.0),
	imuFirmwareUpdate_(false),
	imuPublished_(true),
	publishInterIMU_(false),
	dotProjectormA_(0.0),
	floodLightmA_(200.0),
	detectFeatures_(0),
	useHarrisDetector_(false),
	minDistance_(7.0),
	numTargetFeatures_(1000),
	threshold_(0.01),
	nms_(true),
	nmsRadius_(4)
#endif
{
#ifdef RTABMAP_DEPTHAI
	UASSERT(resolution_>=(int)dai::MonoCameraProperties::SensorResolution::THE_720_P && resolution_<=(int)dai::MonoCameraProperties::SensorResolution::THE_1200_P);
#endif
}

CameraDepthAI::~CameraDepthAI()
{
#ifdef RTABMAP_DEPTHAI
	if(device_.get())
	{
		device_->close();
	}
#endif
}

void CameraDepthAI::setOutputDepth(bool enabled, int confidence)
{
#ifdef RTABMAP_DEPTHAI
	outputDepth_ = enabled;
	if(outputDepth_)
	{
		depthConfidence_ = confidence;
	}
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setAlphaScaling(float alphaScaling)
{
#ifdef RTABMAP_DEPTHAI
	alphaScaling_ = alphaScaling;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setIMUFirmwareUpdate(bool enabled)
{
#ifdef RTABMAP_DEPTHAI
	imuFirmwareUpdate_ = enabled;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setIMUPublished(bool published)
{
#ifdef RTABMAP_DEPTHAI
	imuPublished_ = published;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::publishInterIMU(bool enabled)
{
#ifdef RTABMAP_DEPTHAI
	publishInterIMU_ = enabled;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setLaserDotBrightness(float dotProjectormA)
{
#ifdef RTABMAP_DEPTHAI
	dotProjectormA_ = dotProjectormA;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setFloodLightBrightness(float floodLightmA)
{
#ifdef RTABMAP_DEPTHAI
	floodLightmA_ = floodLightmA;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setDetectFeatures(int detectFeatures)
{
#ifdef RTABMAP_DEPTHAI
	detectFeatures_ = detectFeatures;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setBlobPath(const std::string & blobPath)
{
#ifdef RTABMAP_DEPTHAI
	blobPath_ = blobPath;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setGFTTDetector(bool useHarrisDetector, float minDistance, int numTargetFeatures)
{
#ifdef RTABMAP_DEPTHAI
	useHarrisDetector_ = useHarrisDetector;
	minDistance_ = minDistance;
	numTargetFeatures_ = numTargetFeatures;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setSuperPointDetector(float threshold, bool nms, int nmsRadius)
{
#ifdef RTABMAP_DEPTHAI
	threshold_ = threshold;
	nms_ = nms;
	nmsRadius_ = nmsRadius;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

bool CameraDepthAI::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_DEPTHAI

	std::vector<dai::DeviceInfo> devices = dai::Device::getAllAvailableDevices();
	if(devices.empty())
	{
		UERROR("No DepthAI device found");
		return false;
	}

	if(device_.get())
		device_->close();

	accBuffer_.clear();
	gyroBuffer_.clear();

	bool deviceFound = false;
	dai::DeviceInfo deviceToUse(mxidOrName_);
	if(mxidOrName_.empty())
	{
		std::tie(deviceFound, deviceToUse) = dai::Device::getFirstAvailableDevice();
	}
	else if(!deviceToUse.mxid.empty())
	{
		std::tie(deviceFound, deviceToUse) = dai::Device::getDeviceByMxId(deviceToUse.mxid);
	}
	else
	{
		for(auto& device : devices)
		{
			if(deviceToUse.name == device.name)
			{
				deviceFound = true;
				deviceToUse = device;
			}
		}
	}

	if(!deviceFound)
	{
		UERROR("Could not find DepthAI device with MXID or IP/USB name \"%s\", found devices:", mxidOrName_.c_str());
		for(auto& device : devices)
			UERROR("%s", device.toString().c_str());
		return false;
	}

	// look for calibration files
	stereoModel_ = StereoCameraModel();
	targetSize_ = cv::Size(resolution_<2?1280:resolution_==4?1920:640, resolution_==0?720:resolution_==1?800:resolution_==2?400:resolution_==3?480:1200);

	dai::Pipeline p;
	auto monoLeft  = p.create<dai::node::MonoCamera>();
	auto monoRight = p.create<dai::node::MonoCamera>();
	auto stereo    = p.create<dai::node::StereoDepth>();
	std::shared_ptr<dai::node::IMU> imu;
	if(imuPublished_)
		imu = p.create<dai::node::IMU>();
	std::shared_ptr<dai::node::FeatureTracker> gfttDetector;
	std::shared_ptr<dai::node::ImageManip> manip;
	std::shared_ptr<dai::node::NeuralNetwork> superPointNetwork;
	if(detectFeatures_ == 1)
	{
		gfttDetector = p.create<dai::node::FeatureTracker>();
	}
	else if(detectFeatures_ == 2)
	{
		if(!blobPath_.empty())
		{
			manip = p.create<dai::node::ImageManip>();
			superPointNetwork = p.create<dai::node::NeuralNetwork>();
		}
		else
		{
			UWARN("Missing SuperPoint blob file!");
			detectFeatures_ = 0;
		}
	}

	auto xoutLeft = p.create<dai::node::XLinkOut>();
	auto xoutDepthOrRight = p.create<dai::node::XLinkOut>();
	std::shared_ptr<dai::node::XLinkOut> xoutIMU;
	if(imuPublished_)
		xoutIMU = p.create<dai::node::XLinkOut>();
	std::shared_ptr<dai::node::XLinkOut> xoutFeatures;
	if(detectFeatures_)
		xoutFeatures = p.create<dai::node::XLinkOut>();

	// XLinkOut
	xoutLeft->setStreamName("rectified_left");
	xoutDepthOrRight->setStreamName(outputDepth_?"depth":"rectified_right");
	if(imuPublished_)
		xoutIMU->setStreamName("imu");
	if(detectFeatures_)
		xoutFeatures->setStreamName("features");

	// MonoCamera
	monoLeft->setResolution((dai::MonoCameraProperties::SensorResolution)resolution_);
	monoLeft->setCamera("left");
	monoRight->setResolution((dai::MonoCameraProperties::SensorResolution)resolution_);
	monoRight->setCamera("right");
	if(detectFeatures_ == 2)
	{
		if(this->getImageRate() <= 0 || this->getImageRate() > 15)
		{
			UWARN("On-device SuperPoint enabled, image rate is limited to 15 FPS!");
			monoLeft->setFps(15);
			monoRight->setFps(15);
		}
	}
	else if(this->getImageRate() > 0)
	{
		monoLeft->setFps(this->getImageRate());
		monoRight->setFps(this->getImageRate());
	}

	// StereoDepth
	stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);
	stereo->setExtendedDisparity(false);
	stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
	if(alphaScaling_>-1.0f)
		stereo->setAlphaScaling(alphaScaling_);
	stereo->initialConfig.setConfidenceThreshold(depthConfidence_);
	stereo->initialConfig.setLeftRightCheck(true);
	stereo->initialConfig.setLeftRightCheckThreshold(5);
	stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
	auto config = stereo->initialConfig.get();
	config.censusTransform.kernelSize = dai::StereoDepthConfig::CensusTransform::KernelSize::KERNEL_7x9;
	config.censusTransform.kernelMask = 0X2AA00AA805540155;
	config.postProcessing.brightnessFilter.maxBrightness = 255;
	stereo->initialConfig.set(config);

	// Link plugins CAM -> STEREO -> XLINK
	monoLeft->out.link(stereo->left);
	monoRight->out.link(stereo->right);

	// Using VideoEncoder on PoE devices, Subpixel is not supported
	if(deviceToUse.protocol == X_LINK_TCP_IP)
	{
		auto leftEnc  = p.create<dai::node::VideoEncoder>();
		auto depthOrRightEnc  = p.create<dai::node::VideoEncoder>();
		leftEnc->setDefaultProfilePreset(monoLeft->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
		depthOrRightEnc->setDefaultProfilePreset(monoRight->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
		stereo->rectifiedLeft.link(leftEnc->input);
		if(outputDepth_)
			stereo->disparity.link(depthOrRightEnc->input);
		else
			stereo->rectifiedRight.link(depthOrRightEnc->input);
		leftEnc->bitstream.link(xoutLeft->input);
		depthOrRightEnc->bitstream.link(xoutDepthOrRight->input);
	}
	else
	{
		stereo->setSubpixel(true);
		stereo->setSubpixelFractionalBits(4);
		config = stereo->initialConfig.get();
		config.costMatching.disparityWidth = dai::StereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_64;
		config.costMatching.enableCompanding = true;
		stereo->initialConfig.set(config);
		stereo->rectifiedLeft.link(xoutLeft->input);
		if(outputDepth_)
			stereo->depth.link(xoutDepthOrRight->input);
		else
			stereo->rectifiedRight.link(xoutDepthOrRight->input);
	}

	if(imuPublished_)
	{
		// enable ACCELEROMETER_RAW and GYROSCOPE_RAW at 100 hz rate
		imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 100);
		// above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
		imu->setBatchReportThreshold(1);
		// maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
		// if lower or equal to batchReportThreshold then the sending is always blocking on device
		// useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
		imu->setMaxBatchReports(10);

		// Link plugins IMU -> XLINK
		imu->out.link(xoutIMU->input);

		imu->enableFirmwareUpdate(imuFirmwareUpdate_);
	}

	if(detectFeatures_ == 1)
	{
		gfttDetector->setHardwareResources(1, 2);
		gfttDetector->initialConfig.setCornerDetector(
			useHarrisDetector_?dai::FeatureTrackerConfig::CornerDetector::Type::HARRIS:dai::FeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI);
		gfttDetector->initialConfig.setNumTargetFeatures(numTargetFeatures_);
		gfttDetector->initialConfig.setMotionEstimator(false);
		auto cfg = gfttDetector->initialConfig.get();
		cfg.featureMaintainer.minimumDistanceBetweenFeatures = minDistance_ * minDistance_;
		gfttDetector->initialConfig.set(cfg);
		stereo->rectifiedLeft.link(gfttDetector->inputImage);
		gfttDetector->outputFeatures.link(xoutFeatures->input);
	}
	else if(detectFeatures_ == 2)
	{
		manip->setKeepAspectRatio(false);
		manip->setMaxOutputFrameSize(320 * 200);
		manip->initialConfig.setResize(320, 200);
		superPointNetwork->setBlobPath(blobPath_);
		superPointNetwork->setNumInferenceThreads(2);
		superPointNetwork->setNumNCEPerInferenceThread(1);
		superPointNetwork->input.setBlocking(false);
		stereo->rectifiedLeft.link(manip->inputImage);
		manip->out.link(superPointNetwork->input);
		superPointNetwork->out.link(xoutFeatures->input);
	}

	device_.reset(new dai::Device(p, deviceToUse));

	UINFO("Loading eeprom calibration data");
	dai::CalibrationHandler calibHandler = device_->readCalibration();

	cv::Mat cameraMatrix, distCoeffs, new_camera_matrix;

	std::vector<std::vector<float> > matrix = calibHandler.getCameraIntrinsics(dai::CameraBoardSocket::CAM_B, dai::Size2f(targetSize_.width, targetSize_.height));
	cameraMatrix = (cv::Mat_<double>(3,3) <<
		matrix[0][0], matrix[0][1], matrix[0][2],
		matrix[1][0], matrix[1][1], matrix[1][2],
		matrix[2][0], matrix[2][1], matrix[2][2]);

	std::vector<float> coeffs = calibHandler.getDistortionCoefficients(dai::CameraBoardSocket::CAM_B);
	if(calibHandler.getDistortionModel(dai::CameraBoardSocket::CAM_B) == dai::CameraModel::Perspective)
		distCoeffs = (cv::Mat_<double>(1,8) << coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7]);

	if(alphaScaling_>-1.0f)
		new_camera_matrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, targetSize_, alphaScaling_);
	else
		new_camera_matrix = cameraMatrix;

	double fx = new_camera_matrix.at<double>(0, 0);
	double fy = new_camera_matrix.at<double>(1, 1);
	double cx = new_camera_matrix.at<double>(0, 2);
	double cy = new_camera_matrix.at<double>(1, 2);
	double baseline = calibHandler.getBaselineDistance()/100.0;
	UINFO("left: fx=%f fy=%f cx=%f cy=%f baseline=%f", fx, fy, cx, cy, baseline);
	stereoModel_ = StereoCameraModel(device_->getDeviceName(), fx, fy, cx, cy, baseline, this->getLocalTransform(), targetSize_);

	if(imuPublished_)
	{
		// Cannot test the following, I get "IMU calibration data is not available on device yet." with my camera
		// Update: now (as March 6, 2022) it crashes in "dai::CalibrationHandler::getImuToCameraExtrinsics(dai::CameraBoardSocket, bool)"
		//matrix = calibHandler.getImuToCameraExtrinsics(dai::CameraBoardSocket::CAM_B);
		//imuLocalTransform_ = Transform(
		//		matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3],
		//		matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3],
		//		matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3]);
		auto eeprom = calibHandler.getEepromData();
		if(eeprom.boardName == "OAK-D" ||
		   eeprom.boardName == "BW1098OBC")
		{
			imuLocalTransform_ = Transform(
				 0, -1,  0,  0.0525,
				 1,  0,  0,  0.0137,
				 0,  0,  1,  0);
		}
		else if(eeprom.boardName == "DM9098")
		{
			imuLocalTransform_ = Transform(
				 0,  1,  0,  0.075445,
				 1,  0,  0,  0.00079,
				 0,  0, -1, -0.007);
		}
		else if(eeprom.boardName == "NG9097")
		{
			imuLocalTransform_ = Transform(
				 0,  1,  0,  0.0775,
				 1,  0,  0,  0.020265,
				 0,  0, -1, -0.007);
		}
		else
		{
			UWARN("Unknown boardName (%s)! Disabling IMU!", eeprom.boardName.c_str());
			imuPublished_ = false;
		}
	}
	else
	{
		UINFO("IMU disabled");
	}

	if(imuPublished_)
	{
		imuLocalTransform_ = this->getLocalTransform() * imuLocalTransform_;
		UINFO("IMU local transform = %s", imuLocalTransform_.prettyPrint().c_str());
		device_->getOutputQueue("imu", 50, false)->addCallback([this](const std::shared_ptr<dai::ADatatype> data) {
			auto imuData = std::dynamic_pointer_cast<dai::IMUData>(data);
			auto imuPackets = imuData->packets;

			for(auto& imuPacket : imuPackets)
			{
				auto& acceleroValues = imuPacket.acceleroMeter;
				auto& gyroValues = imuPacket.gyroscope;
				double accStamp = std::chrono::duration<double>(acceleroValues.getTimestampDevice().time_since_epoch()).count();
				double gyroStamp = std::chrono::duration<double>(gyroValues.getTimestampDevice().time_since_epoch()).count();

				if(publishInterIMU_)
				{
					IMU imu(cv::Vec3f(gyroValues.x, gyroValues.y, gyroValues.z), cv::Mat::eye(3,3,CV_64FC1),
							cv::Vec3f(acceleroValues.x, acceleroValues.y, acceleroValues.z), cv::Mat::eye(3,3,CV_64FC1),
							imuLocalTransform_);
					UEventsManager::post(new IMUEvent(imu, (accStamp+gyroStamp)/2));
				}
				else
				{
					UScopeMutex lock(imuMutex_);
					accBuffer_.emplace_hint(accBuffer_.end(), std::make_pair(accStamp, cv::Vec3f(acceleroValues.x, acceleroValues.y, acceleroValues.z)));
					gyroBuffer_.emplace_hint(gyroBuffer_.end(), std::make_pair(gyroStamp, cv::Vec3f(gyroValues.x, gyroValues.y, gyroValues.z)));
				}
			}
		});
	}
	leftQueue_ = device_->getOutputQueue("rectified_left", 8, false);
	rightOrDepthQueue_ = device_->getOutputQueue(outputDepth_?"depth":"rectified_right", 8, false);
	if(detectFeatures_)
		featuresQueue_ = device_->getOutputQueue("features", 8, false);

	std::vector<std::tuple<std::string, int, int>> irDrivers = device_->getIrDrivers();
	if(!irDrivers.empty())
	{
		device_->setIrLaserDotProjectorBrightness(dotProjectormA_);
		device_->setIrFloodLightBrightness(floodLightmA_);
	}

	uSleep(2000); // avoid bad frames on start

	return true;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
	return false;
}

bool CameraDepthAI::isCalibrated() const
{
#ifdef RTABMAP_DEPTHAI
	return stereoModel_.isValidForProjection();
#else
	return false;
#endif
}

std::string CameraDepthAI::getSerial() const
{
#ifdef RTABMAP_DEPTHAI
	return device_->getMxId();
#endif
	return "";
}

SensorData CameraDepthAI::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_DEPTHAI

	cv::Mat left, depthOrRight;
	auto rectifL = leftQueue_->get<dai::ImgFrame>();
	auto rectifRightOrDepth = rightOrDepthQueue_->get<dai::ImgFrame>();

	while(rectifL->getSequenceNum() < rectifRightOrDepth->getSequenceNum())
		rectifL = leftQueue_->get<dai::ImgFrame>();
	while(rectifL->getSequenceNum() > rectifRightOrDepth->getSequenceNum())
		rectifRightOrDepth = rightOrDepthQueue_->get<dai::ImgFrame>();

	double stamp = std::chrono::duration<double>(rectifL->getTimestampDevice(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();
	if(device_->getDeviceInfo().protocol == X_LINK_TCP_IP)
	{
		left = cv::imdecode(rectifL->getData(), cv::IMREAD_GRAYSCALE);
		depthOrRight = cv::imdecode(rectifRightOrDepth->getData(), cv::IMREAD_GRAYSCALE);
		if(outputDepth_)
		{
			cv::Mat depth(targetSize_, CV_16UC1);
			depth.forEach<uint16_t>([&](uint16_t& pixel, const int * position) -> void {
				pixel = stereoModel_.computeDepth(depthOrRight.at<uint8_t>(position))*1000;
			});
			depthOrRight = depth;
		}
	}
	else
	{
		left = rectifL->getFrame(true);
		depthOrRight = rectifRightOrDepth->getFrame(true);
	}

	if(outputDepth_)
		data = SensorData(left, depthOrRight, stereoModel_.left(), this->getNextSeqID(), stamp);
	else
		data = SensorData(left, depthOrRight, stereoModel_, this->getNextSeqID(), stamp);

	if(imuPublished_ && !publishInterIMU_)
	{
		cv::Vec3d acc, gyro;
		std::map<double, cv::Vec3f>::const_iterator iterA, iterB;
	
		imuMutex_.lock();
		while(accBuffer_.empty() || gyroBuffer_.empty() || accBuffer_.rbegin()->first < stamp || gyroBuffer_.rbegin()->first < stamp)
		{
			imuMutex_.unlock();
			uSleep(1);
			imuMutex_.lock();
		}

		//acc
		iterB = accBuffer_.lower_bound(stamp);
		iterA = iterB;
		if(iterA != accBuffer_.begin())
			iterA = --iterA;
		if(iterA == iterB || stamp == iterB->first)
		{
			acc = iterB->second;
		}
		else if(stamp > iterA->first && stamp < iterB->first)
		{
			float t = (stamp-iterA->first) / (iterB->first-iterA->first);
			acc = iterA->second + t*(iterB->second - iterA->second);
		}
		accBuffer_.erase(accBuffer_.begin(), iterB);

		//gyro
		iterB = gyroBuffer_.lower_bound(stamp);
		iterA = iterB;
		if(iterA != gyroBuffer_.begin())
			iterA = --iterA;
		if(iterA == iterB || stamp == iterB->first)
		{
			gyro = iterB->second;
		}
		else if(stamp > iterA->first && stamp < iterB->first)
		{
			float t = (stamp-iterA->first) / (iterB->first-iterA->first);
			gyro = iterA->second + t*(iterB->second - iterA->second);
		}
		gyroBuffer_.erase(gyroBuffer_.begin(), iterB);

		imuMutex_.unlock();
		data.setIMU(IMU(gyro, cv::Mat::eye(3, 3, CV_64FC1), acc, cv::Mat::eye(3, 3, CV_64FC1), imuLocalTransform_));
	}

	if(detectFeatures_ == 1)
	{
		auto features = featuresQueue_->get<dai::TrackedFeatures>();
		while(features->getSequenceNum() < rectifL->getSequenceNum())
			features = featuresQueue_->get<dai::TrackedFeatures>();
		auto detectedFeatures = features->trackedFeatures;

		std::vector<cv::KeyPoint> keypoints;
		for(auto& feature : detectedFeatures)
			keypoints.emplace_back(cv::KeyPoint(feature.position.x, feature.position.y, 3));
		data.setFeatures(keypoints, std::vector<cv::Point3f>(), cv::Mat());
	}
	else if(detectFeatures_ == 2)
	{
		auto features = featuresQueue_->get<dai::NNData>();
		while(features->getSequenceNum() < rectifL->getSequenceNum())
			features = featuresQueue_->get<dai::NNData>();

		auto heatmap = features->getLayerFp16("heatmap");
		auto desc = features->getLayerFp16("desc");

		cv::Mat prob(200, 320, CV_32FC1, heatmap.data());
		cv::resize(prob, prob, targetSize_, 0, 0, cv::INTER_CUBIC);
		std::vector<cv::Point> kpts;
		cv::findNonZero(prob > threshold_, kpts);
		std::vector<cv::KeyPoint> keypoints_no_nms, keypoints;
		for(auto& kpt : kpts)
		{
			float response = prob.at<float>(kpt);
			keypoints_no_nms.emplace_back(cv::KeyPoint(kpt, 8, -1, response));
		}

		if(nms_ && !keypoints_no_nms.empty())
		{
			cv::Mat descEmpty;
			util2d::NMS(keypoints_no_nms, descEmpty, keypoints, descEmpty, 0, nmsRadius_, targetSize_.width, targetSize_.height);
		}
		else if(!keypoints_no_nms.empty())
		{
			keypoints = keypoints_no_nms;
		}

		cv::Mat coarse_desc(25, 40, CV_32FC(256), desc.data());
		coarse_desc.forEach<cv::Vec<float, 256>>([&](cv::Vec<float, 256>& descriptor, const int position[]) -> void {
			cv::normalize(descriptor, descriptor);
		});
		cv::Mat mapX(keypoints.size(), 1, CV_32FC1);
		cv::Mat mapY(keypoints.size(), 1, CV_32FC1);
		for(size_t i=0; i<keypoints.size(); ++i)
		{
			mapX.at<float>(i) = (keypoints[i].pt.x - (targetSize_.width-1)/2) * 40/targetSize_.width + (40-1)/2;
			mapY.at<float>(i) = (keypoints[i].pt.y - (targetSize_.height-1)/2) * 25/targetSize_.height + (25-1)/2;
		}
		cv::Mat map1, map2, descriptors;
		cv::convertMaps(mapX, mapY, map1, map2, CV_16SC2);
		cv::remap(coarse_desc, descriptors, map1, map2, cv::INTER_LINEAR);
		descriptors.forEach<cv::Vec<float, 256>>([&](cv::Vec<float, 256>& descriptor, const int position[]) -> void {
			cv::normalize(descriptor, descriptor);
		});
		descriptors = descriptors.reshape(1);

		data.setFeatures(keypoints, std::vector<cv::Point3f>(), descriptors);
	}

#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
	return data;
}

} // namespace rtabmap
