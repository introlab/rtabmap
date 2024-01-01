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
	outputMode_(0),
	confThreshold_(200),
	lrcThreshold_(5),
	resolution_(resolution),
	useSpecTranslation_(false),
	alphaScaling_(0.0),
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

void CameraDepthAI::setOutputMode(int outputMode)
{
#ifdef RTABMAP_DEPTHAI
	outputMode_ = outputMode;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setDepthProfile(int confThreshold, int lrcThreshold)
{
#ifdef RTABMAP_DEPTHAI
	confThreshold_ = confThreshold;
	lrcThreshold_ = lrcThreshold;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setRectification(bool useSpecTranslation, float alphaScaling)
{
#ifdef RTABMAP_DEPTHAI
	useSpecTranslation_ = useSpecTranslation;
	alphaScaling_ = alphaScaling;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setIMU(bool imuPublished, bool publishInterIMU)
{
#ifdef RTABMAP_DEPTHAI
	imuPublished_ = imuPublished;
	publishInterIMU_ = publishInterIMU;
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setIrBrightness(float dotProjectormA, float floodLightmA)
{
#ifdef RTABMAP_DEPTHAI
	dotProjectormA_ = dotProjectormA;
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
	if(devices.empty() && mxidOrName_.empty())
	{
		UERROR("No DepthAI device found or specified");
		return false;
	}

	if(device_.get())
		device_->close();

	accBuffer_.clear();
	gyroBuffer_.clear();

	bool deviceFound = false;
	dai::DeviceInfo deviceToUse(mxidOrName_);
	if(mxidOrName_.empty())
		std::tie(deviceFound, deviceToUse) = dai::Device::getFirstAvailableDevice();
	else if(!deviceToUse.mxid.empty())
		std::tie(deviceFound, deviceToUse) = dai::Device::getDeviceByMxId(deviceToUse.mxid);
	else
		deviceFound = true;

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
	std::shared_ptr<dai::node::Camera> colorCam;
	if(outputMode_==2)
	{
		colorCam = p.create<dai::node::Camera>();
		if(detectFeatures_)
		{
			UWARN("On-device feature detectors cannot be enabled on color camera input!");
			detectFeatures_ = 0;
		}
	}
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

	auto xoutLeftOrColor = p.create<dai::node::XLinkOut>();
	auto xoutDepthOrRight = p.create<dai::node::XLinkOut>();
	std::shared_ptr<dai::node::XLinkOut> xoutIMU;
	if(imuPublished_)
		xoutIMU = p.create<dai::node::XLinkOut>();
	std::shared_ptr<dai::node::XLinkOut> xoutFeatures;
	if(detectFeatures_)
		xoutFeatures = p.create<dai::node::XLinkOut>();

	// XLinkOut
	xoutLeftOrColor->setStreamName(outputMode_<2?"rectified_left":"rectified_color");
	xoutDepthOrRight->setStreamName(outputMode_?"depth":"rectified_right");
	if(imuPublished_)
		xoutIMU->setStreamName("imu");
	if(detectFeatures_)
		xoutFeatures->setStreamName("features");

	monoLeft->setResolution((dai::MonoCameraProperties::SensorResolution)resolution_);
	monoRight->setResolution((dai::MonoCameraProperties::SensorResolution)resolution_);
	monoLeft->setCamera("left");
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
	if(outputMode_ == 2)
		stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
	else
		stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);
	stereo->setExtendedDisparity(false);
	stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
	stereo->enableDistortionCorrection(true);
	stereo->setDisparityToDepthUseSpecTranslation(useSpecTranslation_);
	stereo->setDepthAlignmentUseSpecTranslation(useSpecTranslation_);
	if(alphaScaling_ > -1.0f)
		stereo->setAlphaScaling(alphaScaling_);
	stereo->initialConfig.setConfidenceThreshold(confThreshold_);
	stereo->initialConfig.setLeftRightCheck(true);
	stereo->initialConfig.setLeftRightCheckThreshold(lrcThreshold_);
	stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
	auto config = stereo->initialConfig.get();
	config.censusTransform.kernelSize = dai::StereoDepthConfig::CensusTransform::KernelSize::KERNEL_7x9;
	config.censusTransform.kernelMask = 0X2AA00AA805540155;
	config.postProcessing.brightnessFilter.maxBrightness = 255;
	stereo->initialConfig.set(config);

	// Link plugins CAM -> STEREO -> XLINK
	monoLeft->out.link(stereo->left);
	monoRight->out.link(stereo->right);

	if(outputMode_ == 2)
	{
		colorCam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
		colorCam->setSize(targetSize_.width, targetSize_.height);
		if(this->getImageRate() > 0)
			colorCam->setFps(this->getImageRate());
		if(alphaScaling_ > -1.0f)
			colorCam->setCalibrationAlpha(alphaScaling_);
	}

	// Using VideoEncoder on PoE devices, Subpixel is not supported
	if(deviceToUse.protocol == X_LINK_TCP_IP || mxidOrName_.find(".") != std::string::npos)
	{
		auto leftOrColorEnc  = p.create<dai::node::VideoEncoder>();
		auto depthOrRightEnc  = p.create<dai::node::VideoEncoder>();
		leftOrColorEnc->setDefaultProfilePreset(monoLeft->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
		depthOrRightEnc->setDefaultProfilePreset(monoRight->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
		if(outputMode_ < 2)
		{
			stereo->rectifiedLeft.link(leftOrColorEnc->input);
		}
		else
		{
			colorCam->video.link(leftOrColorEnc->input);
		}
		if(outputMode_)
		{
			depthOrRightEnc->setQuality(100);
			stereo->disparity.link(depthOrRightEnc->input);
		}
		else
		{
			stereo->rectifiedRight.link(depthOrRightEnc->input);
		}
		leftOrColorEnc->bitstream.link(xoutLeftOrColor->input);
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
		if(outputMode_ < 2)
		{
			stereo->rectifiedLeft.link(xoutLeftOrColor->input);
		}
		else
		{
			monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
			monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
			colorCam->video.link(xoutLeftOrColor->input);
		}
		if(outputMode_)
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

	auto cameraId = outputMode_<2?dai::CameraBoardSocket::CAM_B:dai::CameraBoardSocket::CAM_A;
	cv::Mat cameraMatrix, distCoeffs, newCameraMatrix;

	std::vector<std::vector<float> > matrix = calibHandler.getCameraIntrinsics(cameraId, targetSize_.width, targetSize_.height);
	cameraMatrix = (cv::Mat_<double>(3,3) <<
		matrix[0][0], matrix[0][1], matrix[0][2],
		matrix[1][0], matrix[1][1], matrix[1][2],
		matrix[2][0], matrix[2][1], matrix[2][2]);

	std::vector<float> coeffs = calibHandler.getDistortionCoefficients(cameraId);
	if(calibHandler.getDistortionModel(cameraId) == dai::CameraModel::Perspective)
		distCoeffs = (cv::Mat_<double>(1,8) << coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7]);

	if(alphaScaling_>-1.0f)
		newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, targetSize_, alphaScaling_);
	else
		newCameraMatrix = cameraMatrix;

	double fx = newCameraMatrix.at<double>(0, 0);
	double fy = newCameraMatrix.at<double>(1, 1);
	double cx = newCameraMatrix.at<double>(0, 2);
	double cy = newCameraMatrix.at<double>(1, 2);
	double baseline = calibHandler.getBaselineDistance(dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_B, useSpecTranslation_)/100.0;
	UINFO("fx=%f fy=%f cx=%f cy=%f baseline=%f", fx, fy, cx, cy, baseline);
	if(outputMode_ == 2)
		stereoModel_ = StereoCameraModel(device_->getDeviceName(), fx, fy, cx, cy, baseline, this->getLocalTransform(), targetSize_);
	else
		stereoModel_ = StereoCameraModel(device_->getDeviceName(), fx, fy, cx, cy, baseline, this->getLocalTransform()*Transform(-calibHandler.getBaselineDistance(dai::CameraBoardSocket::CAM_A)/100.0, 0, 0), targetSize_);

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
				 1,  0,  0,  0.013662,
				 0,  0,  1,  0);
		}
		else if(eeprom.boardName == "DM9098")
		{
			imuLocalTransform_ = Transform(
				 0,  1,  0,  0.037945,
				 1,  0,  0,  0.00079,
				 0,  0, -1,  0);
		}
		else if(eeprom.boardName == "NG2094")
		{
			imuLocalTransform_ = Transform(
				 0,  1,  0,  0.0374,
				 1,  0,  0,  0.00176,
				 0,  0, -1,  0);
		}
		else if(eeprom.boardName == "NG9097")
		{
			imuLocalTransform_ = Transform(
				 0,  1,  0,  0.04,
				 1,  0,  0,  0.020265,
				 0,  0, -1,  0);
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
					accBuffer_.emplace_hint(accBuffer_.end(), accStamp, cv::Vec3f(acceleroValues.x, acceleroValues.y, acceleroValues.z));
					gyroBuffer_.emplace_hint(gyroBuffer_.end(), gyroStamp, cv::Vec3f(gyroValues.x, gyroValues.y, gyroValues.z));
				}
			}
		});
	}
	leftOrColorQueue_ = device_->getOutputQueue(outputMode_<2?"rectified_left":"rectified_color", 8, false);
	rightOrDepthQueue_ = device_->getOutputQueue(outputMode_?"depth":"rectified_right", 8, false);
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

	cv::Mat leftOrColor, depthOrRight;
	auto rectifLeftOrColor = leftOrColorQueue_->get<dai::ImgFrame>();
	auto rectifRightOrDepth = rightOrDepthQueue_->get<dai::ImgFrame>();

	while(rectifLeftOrColor->getSequenceNum() < rectifRightOrDepth->getSequenceNum())
		rectifLeftOrColor = leftOrColorQueue_->get<dai::ImgFrame>();
	while(rectifLeftOrColor->getSequenceNum() > rectifRightOrDepth->getSequenceNum())
		rectifRightOrDepth = rightOrDepthQueue_->get<dai::ImgFrame>();

	double stamp = std::chrono::duration<double>(rectifLeftOrColor->getTimestampDevice(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();
	if(device_->getDeviceInfo().protocol == X_LINK_TCP_IP || mxidOrName_.find(".") != std::string::npos)
	{
		leftOrColor = cv::imdecode(rectifLeftOrColor->getData(), cv::IMREAD_ANYCOLOR);
		depthOrRight = cv::imdecode(rectifRightOrDepth->getData(), cv::IMREAD_GRAYSCALE);
		if(outputMode_)
		{
			cv::Mat disp;
			depthOrRight.convertTo(disp, CV_16UC1);
			cv::divide(-stereoModel_.right().Tx() * 1000, disp, depthOrRight);
		}
	}
	else
	{
		leftOrColor = rectifLeftOrColor->getCvFrame();
		depthOrRight = rectifRightOrDepth->getCvFrame();
	}

	if(outputMode_)
		data = SensorData(leftOrColor, depthOrRight, stereoModel_.left(), this->getNextSeqID(), stamp);
	else
		data = SensorData(leftOrColor, depthOrRight, stereoModel_, this->getNextSeqID(), stamp);

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
		while(features->getSequenceNum() < rectifLeftOrColor->getSequenceNum())
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
		while(features->getSequenceNum() < rectifLeftOrColor->getSequenceNum())
			features = featuresQueue_->get<dai::NNData>();

		auto heatmap = features->getLayerFp16("heatmap");
		auto desc = features->getLayerFp16("desc");

		cv::Mat scores(200, 320, CV_32FC1, heatmap.data());
		cv::resize(scores, scores, targetSize_, 0, 0, cv::INTER_CUBIC);

		if(nms_)
		{
			cv::Mat dilated_scores(targetSize_, CV_32FC1);
			cv::dilate(scores, dilated_scores, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(nmsRadius_*2+1, nmsRadius_*2+1)));
			cv::Mat max_mask = scores == dilated_scores;
			cv::dilate(scores, dilated_scores, cv::Mat());
			cv::Mat max_mask_r1 = scores == dilated_scores;
			cv::Mat supp_mask(targetSize_, CV_8UC1);
			for(size_t i=0; i<2; i++)
			{
				cv::dilate(max_mask, supp_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(nmsRadius_*2+1, nmsRadius_*2+1)));
				cv::Mat supp_scores = scores.clone();
				supp_scores.setTo(0, supp_mask);
				cv::dilate(supp_scores, dilated_scores, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(nmsRadius_*2+1, nmsRadius_*2+1)));
				cv::Mat new_max_mask = cv::Mat::zeros(targetSize_, CV_8UC1);
				cv::bitwise_not(supp_mask, supp_mask);
				cv::bitwise_and(supp_scores == dilated_scores, supp_mask, new_max_mask, max_mask_r1);
				cv::bitwise_or(max_mask, new_max_mask, max_mask);
			}
			cv::bitwise_not(max_mask, supp_mask);
			scores.setTo(0, supp_mask);
		}

		std::vector<cv::Point> kpts;
		cv::findNonZero(scores > threshold_, kpts);
		std::vector<cv::KeyPoint> keypoints;
		for(auto& kpt : kpts)
		{
			float response = scores.at<float>(kpt);
			keypoints.emplace_back(cv::KeyPoint(kpt, 8, -1, response));
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
