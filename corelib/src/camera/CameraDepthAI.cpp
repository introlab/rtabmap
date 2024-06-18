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
	extendedDisparity_(false),
	subpixelFractionalBits_(0),
	compandingWidth_(0),
	useSpecTranslation_(false),
	alphaScaling_(0.0),
	imuPublished_(true),
	publishInterIMU_(false),
	dotIntensity_(0.0),
	floodIntensity_(0.0),
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

void CameraDepthAI::setExtendedDisparity(bool extendedDisparity)
{
#ifdef RTABMAP_DEPTHAI
	extendedDisparity_ = extendedDisparity;
	if(extendedDisparity_)
	{
		if(subpixelFractionalBits_>0)
		{
			UWARN("Extended disparity has been enabled while subpixel being also enabled, disabling subpixel...");
			subpixelFractionalBits_ = 0;
		}
		if(compandingWidth_>0)
		{
			UWARN("Extended disparity has been enabled while companding being also enabled, disabling companding...");
			compandingWidth_ = 0;
		}
	}
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setSubpixelMode(bool enabled, int fractionalBits)
{
#ifdef RTABMAP_DEPTHAI
	UASSERT(fractionalBits>=3 && fractionalBits<=5);
	subpixelFractionalBits_ = enabled?fractionalBits:0;
	if(subpixelFractionalBits_ != 0 && extendedDisparity_)
	{
		UWARN("Subpixel has been enabled while extended disparity being also enabled, disabling extended disparity...");
		extendedDisparity_ = false;
	}
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setCompanding(bool enabled, int width)
{
#ifdef RTABMAP_DEPTHAI
	UASSERT(width == 64 || width == 96);
	compandingWidth_ = enabled?width:0;
	if(compandingWidth_ != 0 && extendedDisparity_)
	{
		UWARN("Companding has been enabled while extended disparity being also enabled, disabling extended disparity...");
		extendedDisparity_ = false;
	}
#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
}

void CameraDepthAI::setRectification(bool useSpecTranslation, float alphaScaling, bool enabled)
{
#ifdef RTABMAP_DEPTHAI
	useSpecTranslation_ = useSpecTranslation;
	alphaScaling_ = alphaScaling;
	imagesRectified_ = enabled;
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

void CameraDepthAI::setIrIntensity(float dotIntensity, float floodIntensity)
{
#ifdef RTABMAP_DEPTHAI
	dotIntensity_ = dotIntensity;
	floodIntensity_ = floodIntensity;
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
	std::shared_ptr<dai::node::StereoDepth> stereo;
	if(imagesRectified_)
		stereo = p.create<dai::node::StereoDepth>();
	std::shared_ptr<dai::node::Camera> colorCam;
	if(outputMode_==2)
	{
		colorCam = p.create<dai::node::Camera>();
		if(!imagesRectified_)
			colorCam->setMeshSource(dai::CameraProperties::WarpMeshSource::NONE);
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
	std::shared_ptr<dai::node::NeuralNetwork> neuralNetwork;
	if(detectFeatures_ == 1)
	{
		gfttDetector = p.create<dai::node::FeatureTracker>();
	}
	else if(detectFeatures_ >= 2)
	{
		if(!blobPath_.empty())
		{
			manip = p.create<dai::node::ImageManip>();
			neuralNetwork = p.create<dai::node::NeuralNetwork>();
		}
		else
		{
			UWARN("Missing MyriadX blob file!");
			detectFeatures_ = 0;
		}
	}

	auto sync = p.create<dai::node::Sync>();
	auto xoutCamera = p.create<dai::node::XLinkOut>();
	std::shared_ptr<dai::node::XLinkOut> xoutIMU;
	if(imuPublished_)
		xoutIMU = p.create<dai::node::XLinkOut>();

	// XLinkOut
	xoutCamera->setStreamName("camera");
	if(imuPublished_)
		xoutIMU->setStreamName("imu");

	monoLeft->setResolution((dai::MonoCameraProperties::SensorResolution)resolution_);
	monoRight->setResolution((dai::MonoCameraProperties::SensorResolution)resolution_);
	monoLeft->setCamera("left");
	monoRight->setCamera("right");
	if(detectFeatures_ >= 2)
	{
		if(this->getImageRate() <= 0 || this->getImageRate() > 15)
		{
			UWARN("On-device SuperPoint or HF-Net enabled, image rate is limited to 15 FPS!");
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
	if(stereo.get())
	{
		if(outputMode_ == 2)
			stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
		else
			stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);
		stereo->setExtendedDisparity(extendedDisparity_);
		stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
		stereo->enableDistortionCorrection(true);
		stereo->setDisparityToDepthUseSpecTranslation(useSpecTranslation_);
		stereo->setDepthAlignmentUseSpecTranslation(useSpecTranslation_);
		if(alphaScaling_ > -1.0f)
			stereo->setAlphaScaling(alphaScaling_);
		stereo->initialConfig.setConfidenceThreshold(confThreshold_);
		stereo->initialConfig.setLeftRightCheck(lrcThreshold_>=0);
		if(lrcThreshold_>=0)
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
	}

	if(outputMode_ == 2)
	{
		colorCam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
		colorCam->setSize(targetSize_.width, targetSize_.height);
		if(this->getImageRate() > 0)
			colorCam->setFps(this->getImageRate());
		if(alphaScaling_ > -1.0f)
			colorCam->setCalibrationAlpha(alphaScaling_);
	}
	this->setImageRate(0);

	// Using VideoEncoder on PoE devices, Subpixel is not supported
	if(deviceToUse.protocol == X_LINK_TCP_IP || mxidOrName_.find(".") != std::string::npos)
	{
		auto leftOrColorEnc  = p.create<dai::node::VideoEncoder>();
		auto depthOrRightEnc  = p.create<dai::node::VideoEncoder>();
		leftOrColorEnc->setDefaultProfilePreset(monoLeft->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
		depthOrRightEnc->setDefaultProfilePreset(monoRight->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
		if(outputMode_ < 2)
		{
			if(imagesRectified_) {
				stereo->rectifiedLeft.link(leftOrColorEnc->input);
			}
			else {
				monoLeft->out.link(leftOrColorEnc->input);
			}
			leftOrColorEnc->bitstream.link(sync->inputs["left"]);
		}
		else
		{
			colorCam->video.link(leftOrColorEnc->input);
			leftOrColorEnc->bitstream.link(sync->inputs["color"]);
		}
		if(imagesRectified_ && outputMode_)
		{
			depthOrRightEnc->setQuality(100);
			stereo->disparity.link(depthOrRightEnc->input);
			depthOrRightEnc->bitstream.link(sync->inputs["depth"]);
		}
		else
		{
			if(imagesRectified_) {
				stereo->rectifiedRight.link(depthOrRightEnc->input);
			}
			else {
				monoRight->out.link(depthOrRightEnc->input);
			}
			depthOrRightEnc->bitstream.link(sync->inputs["right"]);
		}
	}
	else
	{
		if(stereo.get()) {
			stereo->setSubpixel(subpixelFractionalBits_>=3 && subpixelFractionalBits_<=5);
			if(subpixelFractionalBits_>=3 && subpixelFractionalBits_<=5)
				stereo->setSubpixelFractionalBits(subpixelFractionalBits_);
			auto config = stereo->initialConfig.get();
			config.costMatching.enableCompanding = compandingWidth_>0;
			if(compandingWidth_>0)
				config.costMatching.disparityWidth = compandingWidth_==64?dai::StereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_64:dai::StereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_96;
			stereo->initialConfig.set(config);
		}
		if(outputMode_ < 2)
		{
			if(imagesRectified_)
				stereo->rectifiedLeft.link(sync->inputs["left"]);
			else
				monoLeft->out.link(sync->inputs["left"]);
		}
		else
		{
			monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
			monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
			colorCam->video.link(sync->inputs["color"]);
		}
		if(imagesRectified_) {
			if(outputMode_)
				stereo->depth.link(sync->inputs["depth"]);
			else
				stereo->rectifiedRight.link(sync->inputs["right"]);
		}
		else {
			monoRight->out.link(sync->inputs["right"]);
		}
	}

	sync->setSyncThreshold(std::chrono::milliseconds(int(500 / monoLeft->getFps())));
	sync->out.link(xoutCamera->input);

	if(imuPublished_)
	{
		// enable ACCELEROMETER_RAW and GYROSCOPE_RAW at 200 hz rate
		imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
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
		if(stereo.get())
			stereo->rectifiedLeft.link(gfttDetector->inputImage);
		else
			monoLeft->out.link(gfttDetector->inputImage);
		gfttDetector->outputFeatures.link(sync->inputs["feat"]);
	}
	else if(detectFeatures_ >= 2)
	{
		manip->setKeepAspectRatio(false);
		manip->setMaxOutputFrameSize(320 * 200);
		manip->initialConfig.setResize(320, 200);
		neuralNetwork->setBlobPath(blobPath_);
		neuralNetwork->setNumInferenceThreads(2);
		neuralNetwork->setNumNCEPerInferenceThread(1);
		neuralNetwork->input.setBlocking(false);
		if(stereo.get())
			stereo->rectifiedLeft.link(manip->inputImage);
		else
			monoLeft->out.link(manip->inputImage);
		manip->out.link(neuralNetwork->input);
		neuralNetwork->out.link(sync->inputs["feat"]);
	}

	device_.reset(new dai::Device(p, deviceToUse));

	UINFO("Device serial: %s", device_->getMxId().c_str());
	UINFO("Available camera sensors: ");
	for(auto& sensor : device_->getCameraSensorNames()) {
		UINFO("Socket: CAM_%c - %s", 'A'+(unsigned char)sensor.first, sensor.second.c_str());
	}

	UINFO("Loading eeprom calibration data");
	dai::CalibrationHandler calibHandler = device_->readCalibration();

	if(!calibrationFolder.empty() && !cameraName.empty() && imagesRectified_)
	{
		UINFO("Flashing camera...");
		if(outputMode_ == 2)
		{
			stereoModel_.setName(cameraName, "rgb", "depth");
		}
		if(stereoModel_.load(calibrationFolder, cameraName, false))
		{
			std::vector<std::vector<float> > intrinsicsLeft(3);
			std::vector<std::vector<float> > intrinsicsRight(3);
			for(int row = 0; row<3; ++row)
			{
				intrinsicsLeft[row].resize(3);
				intrinsicsRight[row].resize(3);
				for(int col = 0; col<3; ++col)
				{
					intrinsicsLeft[row][col] = stereoModel_.left().K_raw().at<double>(row,col);
					intrinsicsRight[row][col] = stereoModel_.right().K_raw().at<double>(row,col);
				}
			}

			std::vector<float> distortionsLeft = stereoModel_.left().D_raw();
			std::vector<float> distortionsRight = stereoModel_.right().D_raw();
			std::vector<std::vector<float> > rotationMatrix(3);
			for(int row = 0; row<3; ++row)
			{
				rotationMatrix[row].resize(3);
				for(int col = 0; col<3; ++col)
				{
					rotationMatrix[row][col] = stereoModel_.stereoTransform()(row,col);
				}
			}

			std::vector<float> translation(3);
			translation[0] = stereoModel_.stereoTransform().x()*100.0f;
			translation[1] = stereoModel_.stereoTransform().y()*100.0f;
			translation[2] = stereoModel_.stereoTransform().z()*100.0f;

			if(outputMode_ == 2)
			{
				// Only set RGB intrinsics
				calibHandler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_A, intrinsicsLeft, stereoModel_.left().imageWidth(), stereoModel_.left().imageHeight());
				calibHandler.setDistortionCoefficients(dai::CameraBoardSocket::CAM_A, distortionsLeft);
				std::vector<float> specTranslation = calibHandler.getCameraTranslationVector(dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_C, true);
				calibHandler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_C, rotationMatrix, translation, specTranslation);
			}
			else
			{
				calibHandler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_B, intrinsicsLeft, stereoModel_.left().imageWidth(), stereoModel_.left().imageHeight());
				calibHandler.setDistortionCoefficients(dai::CameraBoardSocket::CAM_B, distortionsLeft);
				calibHandler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_C, intrinsicsRight, stereoModel_.right().imageWidth(), stereoModel_.right().imageHeight());
				calibHandler.setDistortionCoefficients(dai::CameraBoardSocket::CAM_C, distortionsRight);
				std::vector<float> specTranslation = calibHandler.getCameraTranslationVector(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, true);
				calibHandler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, rotationMatrix, translation, specTranslation);
			}

			try {
				UINFO("Flashing camera with calibration from %s with camera name %s", calibrationFolder.c_str(), cameraName.c_str());
				if(ULogger::level() <= ULogger::kInfo)
				{
					std::cout << "K left: " << stereoModel_.left().K_raw() << std::endl;
					std::cout << "K right: " << stereoModel_.right().K_raw() << std::endl;
					std::cout << "D left: " << stereoModel_.left().D_raw() << std::endl;
					std::cout << "D right: " << stereoModel_.right().D_raw() << std::endl;
					std::cout << "Extrinsics: " << stereoModel_.stereoTransform() << std::endl;
					std::cout << "Expected K with rectification_alpha=0: " << stereoModel_.left().K()*(double(targetSize_.width)/double(stereoModel_.left().imageWidth())) << std::endl;
				}
				device_->flashCalibration2(calibHandler);
				UINFO("Closing device...");
				device_->close();
				UINFO("Restarting pipeline...");
				device_.reset(new dai::Device(p, deviceToUse));
			}
			catch(const std::runtime_error & e) {
				UERROR("Failed flashing calibration: %s", e.what());
			}
		}
		else
		{
			UERROR("Failed loading calibration from %s with camera name %s", calibrationFolder.c_str(), cameraName.c_str());
		}

		//Reload calibration
		calibHandler = device_->readCalibration();
	}

	auto eeprom = calibHandler.getEepromData();
	UINFO("Product name: %s, board name: %s", eeprom.productName.c_str(), eeprom.boardName.c_str());

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
	UINFO("fx=%f fy=%f cx=%f cy=%f (target size = %dx%d)", fx, fy, cx, cy, targetSize_.width, targetSize_.height);
	if(outputMode_ == 2) {
		stereoModel_ = StereoCameraModel(device_->getDeviceName(), fx, fy, cx, cy, 0, this->getLocalTransform(), targetSize_);
	}
	else {
		double baseline = calibHandler.getBaselineDistance(dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_B, false)/100.0;
		UINFO("baseline=%f", baseline);
		stereoModel_ = StereoCameraModel(device_->getDeviceName(), fx, fy, cx, cy, outputMode_==0?baseline:0, this->getLocalTransform()*Transform(-calibHandler.getBaselineDistance(dai::CameraBoardSocket::CAM_A)/100.0, 0, 0), targetSize_);
	}

	if(imuPublished_)
	{
		// Cannot test the following, I get "IMU calibration data is not available on device yet." with my camera
		// Update: now (as March 6, 2022) it crashes in "dai::CalibrationHandler::getImuToCameraExtrinsics(dai::CameraBoardSocket, bool)"
		//matrix = calibHandler.getImuToCameraExtrinsics(dai::CameraBoardSocket::CAM_B);
		//imuLocalTransform_ = Transform(
		//		matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3],
		//		matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3],
		//		matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3]);
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

	cameraQueue_ = device_->getOutputQueue("camera", 8, false);
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

	if(!device_->getIrDrivers().empty())
	{
		UINFO("Setting IR intensity");
		device_->setIrLaserDotProjectorIntensity(dotIntensity_);
		device_->setIrFloodLightIntensity(floodIntensity_);
	}
	else if(dotIntensity_ > 0 || floodIntensity_ > 0)
	{
		UWARN("No IR drivers were detected! IR intensity cannot be set.");
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
	return outputMode_ == 0?stereoModel_.isValidForProjection():stereoModel_.left().isValidForProjection();
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

SensorData CameraDepthAI::captureImage(SensorCaptureInfo * info)
{
	SensorData data;
#ifdef RTABMAP_DEPTHAI

	auto messageGroup = cameraQueue_->get<dai::MessageGroup>();
	auto rectifLeftOrColor = messageGroup->get<dai::ImgFrame>(outputMode_<2?"left":"color");
	auto rectifRightOrDepth = messageGroup->get<dai::ImgFrame>(imagesRectified_ && outputMode_?"depth":"right");

	cv::Mat leftOrColor, depthOrRight;
	if(device_->getDeviceInfo().protocol == X_LINK_TCP_IP || mxidOrName_.find(".") != std::string::npos)
	{
		leftOrColor = cv::imdecode(rectifLeftOrColor->getData(), cv::IMREAD_ANYCOLOR);
		depthOrRight = cv::imdecode(rectifRightOrDepth->getData(), cv::IMREAD_GRAYSCALE);
		if(imagesRectified_ && outputMode_)
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

	double stamp = std::chrono::duration<double>(rectifLeftOrColor->getTimestampDevice(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();
	if(imagesRectified_ && outputMode_)
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
		auto features = messageGroup->get<dai::TrackedFeatures>("feat")->trackedFeatures;
		std::vector<cv::KeyPoint> keypoints;
		for(auto& feature : features)
			keypoints.emplace_back(cv::KeyPoint(feature.position.x, feature.position.y, 3));
		data.setFeatures(keypoints, std::vector<cv::Point3f>(), cv::Mat());
	}
	else if(detectFeatures_ >= 2)
	{
		auto features = messageGroup->get<dai::NNData>("feat");
		std::vector<float> scores_dense, local_descriptor_map, global_descriptor;
		if(detectFeatures_ == 2)
		{
			scores_dense = features->getLayerFp16("heatmap");
			local_descriptor_map = features->getLayerFp16("desc");
		}
		else if(detectFeatures_ == 3)
		{
			scores_dense = features->getLayerFp16("pred/local_head/detector/Squeeze");
			local_descriptor_map = features->getLayerFp16("pred/local_head/descriptor/transpose");
			global_descriptor = features->getLayerFp16("pred/global_head/l2_normalize_1");
		}

		cv::Mat scores(200, 320, CV_32FC1, scores_dense.data());
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

		cv::Mat coarse_desc(25, 40, CV_32FC(256), local_descriptor_map.data());
		if(detectFeatures_ == 2)
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
		if(detectFeatures_ == 3)
			data.addGlobalDescriptor(GlobalDescriptor(1, cv::Mat(1, global_descriptor.size(), CV_32FC1, global_descriptor.data()).clone()));
	}

#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
	return data;
}

} // namespace rtabmap
