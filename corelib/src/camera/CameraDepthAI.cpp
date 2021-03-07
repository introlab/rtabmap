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
		const std::string & deviceSerial,
		int resolution,
		float imageRate,
		const Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_DEPTHAI
	,
	deviceSerial_(deviceSerial),
	outputDepth_(false),
	depthConfidence_(200),
	resolution_(resolution)
#endif
{
#ifdef RTABMAP_DEPTHAI
	UASSERT(resolution_>=0 && resolution_<=2);
#endif
}

CameraDepthAI::~CameraDepthAI()
{
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

std::vector<unsigned char> convertCalibration(const StereoCameraModel & stereoModel)
{
	UDEBUG("");
	// Calibration
	// https://github.com/luxonis/depthai/blob/39852dcb9fe349476c30d0ed90d3750bb2a53e26/depthai_helpers/calibration_utils.py#L97-L109
	std::vector<unsigned char> data;
	cv::Mat tmp;
	int ptr;
	// R1_fp32
	stereoModel.left().R().convertTo(tmp, CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());

	// R2_fp32
	stereoModel.right().R().convertTo(tmp, CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());

	// M1_fp32
	stereoModel.left().K_raw().convertTo(tmp, CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());

	// M2_fp32
	stereoModel.right().K_raw().convertTo(tmp, CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());

	// R_fp32
	stereoModel.R().convertTo(tmp, CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());

	// T_fp32
	stereoModel.T().convertTo(tmp, CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());

	// M3_fp32
	tmp = cv::Mat::zeros(3,3,CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());

	// R_rgb_fp32
	tmp = cv::Mat::eye(3,3,CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());

	// T_rgb_fp32
	tmp = cv::Mat::zeros(1,3,CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());

	// d1_coeff_fp32
	stereoModel.left().D_raw().convertTo(tmp, CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());
	data.resize(data.size() + (14-tmp.total())*sizeof(float), 0); // padding

	// d2_coeff_fp32
	stereoModel.right().D_raw().convertTo(tmp, CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());
	data.resize(data.size() + (14-tmp.total())*sizeof(float), 0); // padding

	// d3_coeff_fp32
	tmp = cv::Mat::zeros(1,14,CV_32FC1);
	ptr = data.size();
	data.resize(data.size() + tmp.total()*tmp.elemSize());
	memcpy(data.data()+ptr, tmp.data, tmp.total()*tmp.elemSize());

	return data;
}

bool CameraDepthAI::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_DEPTHAI

	std::vector<dai::DeviceInfo> devices = dai::Device::getAllAvailableDevices();
	if(devices.empty())
	{
		return false;
	}

	dai::DeviceInfo deviceToUse;
	if(deviceSerial_.empty())
		deviceToUse = devices[0];
	for(size_t i=0; i<devices.size(); ++i)
	{
		UINFO("DepthAI device found: %s", devices[i].getMxId().c_str());
		if(!deviceSerial_.empty() && deviceSerial_.compare(devices[i].getMxId()) == 0)
		{
			deviceToUse = devices[i];
		}
	}

	if(deviceToUse.getMxId().empty())
	{
		UERROR("Could not find device with serial \"%s\", found devices:", deviceSerial_.c_str());
		for(size_t i=0; i<devices.size(); ++i)
		{
			UERROR("DepthAI device found: %s", devices[i].getMxId().c_str());
		}
		return false;
	}
	deviceSerial_ = deviceToUse.getMxId();

	// look for calibration files
	stereoModel_ = StereoCameraModel();
	if(!calibrationFolder.empty())
	{
		std::string name = cameraName.empty()?deviceSerial_:cameraName;
		if(!stereoModel_.load(calibrationFolder, name, false))
		{
			UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
					name.c_str(), calibrationFolder.c_str());
			outputDepth_ = false;
		}
		else
		{
			UINFO("Stereo parameters: fx=%f cx=%f cy=%f baseline=%f",
					stereoModel_.left().fx(),
					stereoModel_.left().cx(),
					stereoModel_.left().cy(),
					stereoModel_.baseline());
			stereoModel_.setLocalTransform(this->getLocalTransform());

			cv::Size target(resolution_<2?1280:640, resolution_==0?720:resolution_==1?800:400);

			if(stereoModel_.left().imageWidth() != target.width)
			{
				//adjust scale if resolution is not the same used than in calibration
				UWARN("Loaded calibration has different resolution (%dx%d) than "
					  "the selected device resolution (%dx%d). We will scale the calibration "
					  "for convenience.",
					  stereoModel_.left().imageWidth(), stereoModel_.left().imageHeight(),
					  target.width, target.height);
				stereoModel_.scale(double(target.width)/double(stereoModel_.left().imageWidth()));
			}

			if(stereoModel_.left().imageHeight() != target.height)
			{
				// Ratio not the same, adjust cy
				cv::Rect roi(0, (stereoModel_.left().imageHeight()-target.height)/2, target.width, target.height);
				UWARN("Loaded calibration has different height (%dx%d) than "
					  "the selected device resolution (%dx%d). We will crop the calibration "
					  "for convenience.",
					  stereoModel_.left().imageWidth(), stereoModel_.left().imageHeight(),
					  target.width, target.height);
				stereoModel_.roi(roi);
			}

			if(ULogger::level() <= ULogger::kInfo)
			{
				UINFO("Calibration:");
				std::cout << stereoModel_ << std::endl;
			}
		}
	}

	if(!stereoModel_.isValidForRectification())
	{
		UINFO("Disabling outputDepth as no valid calibration has been loaded.");
		outputDepth_ = false;
	}
	else
	{
		stereoModel_.initRectificationMap();
	}

	dai::Pipeline p;
	auto monoLeft  = p.create<dai::node::MonoCamera>();
	auto monoRight = p.create<dai::node::MonoCamera>();
	auto stereo    = p.create<dai::node::StereoDepth>();
	auto xoutLeft = p.create<dai::node::XLinkOut>();
	auto xoutDepthOrRight = p.create<dai::node::XLinkOut>();

	 // XLinkOut
	xoutLeft->setStreamName(outputDepth_/*stereoModel_.isValidForRectification()*/?"rectified_left":"left");
	xoutDepthOrRight->setStreamName(outputDepth_?"depth"/*:stereoModel_.isValidForRectification()?"rectified_right"*/:"right");

	// MonoCamera
	monoLeft->setResolution((dai::MonoCameraProperties::SensorResolution)resolution_);
	monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
	monoRight->setResolution((dai::MonoCameraProperties::SensorResolution)resolution_);
	monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
	if(this->getImageRate()>0)
	{
		monoLeft->setFps(this->getImageRate());
		monoRight->setFps(this->getImageRate());
	}

	// StereoDepth
	stereo->setOutputDepth(outputDepth_);
	stereo->setOutputRectified(stereoModel_.isValidForRectification());
	stereo->setConfidenceThreshold(depthConfidence_);
	stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
	stereo->setRectifyMirrorFrame(false);
	stereo->setLeftRightCheck(false);
	stereo->setSubpixel(false);
	stereo->setExtendedDisparity(false);

	// Link plugins CAM -> STEREO -> XLINK
	monoLeft->out.link(stereo->left);
	monoRight->out.link(stereo->right);

	if(outputDepth_)
	{
		stereo->rectifiedLeft.link(xoutLeft->input);
		stereo->depth.link(xoutDepthOrRight->input);
	}
	/*else if(stereoModel_.isValidForRectification())
	{
		stereo->rectifiedLeft.link(xoutLeft->input);
		stereo->rectifiedRight.link(xoutDepthOrRight->input);
	}*/
	else
	{
		stereo->syncedLeft.link(xoutLeft->input);
		stereo->syncedRight.link(xoutDepthOrRight->input);
	}


	if(stereoModel_.isValidForRectification())
	{
		// FIXME: What is the exact format for the calibration stream?
		//std::vector<unsigned char> data = convertCalibration(stereoModel_);
		//stereo->loadCalibrationData(data);
	}
	device_.reset(new dai::Device(p, deviceToUse));

	UDEBUG("");
	if(outputDepth_)
	{
		leftQueue_ = device_->getOutputQueue("rectified_left", 8, false);
		rightOrDepthQueue_ = device_->getOutputQueue("depth", 8, false);
	}
	else
	{
		UDEBUG("");
		leftQueue_ = device_->getOutputQueue(/*stereoModel_.isValidForRectification()?"rectified_left":*/"left", 8, false);
		UDEBUG("");
		rightOrDepthQueue_ = device_->getOutputQueue(/*stereoModel_.isValidForRectification()?"rectified_right":*/"right", 8, false);
		UDEBUG("");
	}

	device_->startPipeline();

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
	return deviceSerial_;
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
	if(rectifL.get() && rectifRightOrDepth.get())
	{
		auto stampLeft = rectifL->getTimestamp().time_since_epoch().count();
		auto stampRight = rectifRightOrDepth->getTimestamp().time_since_epoch().count();
		double stamp = double(stampLeft)/10e8;
		left = rectifL->getCvFrame();
		depthOrRight = rectifRightOrDepth->getCvFrame();

		if(!left.empty() && !depthOrRight.empty())
		{
			if(depthOrRight.type() == CV_8UC1)
			{
				if(stereoModel_.isValidForRectification())
				{
					left = stereoModel_.left().rectifyImage(left);
					depthOrRight = stereoModel_.right().rectifyImage(depthOrRight);
				}
				data = SensorData(left, depthOrRight, stereoModel_, this->getNextSeqID(), stamp);
			}
			else
			{
				cv::flip(depthOrRight, depthOrRight, 1);
				data = SensorData(left, depthOrRight, stereoModel_.left(), this->getNextSeqID(), stamp);
			}

			if(stampLeft != stampRight)
			{
				UWARN("Frames are not synchronized! %f vs %f", double(stampLeft)/10e8, double(stampRight)/10e8);
			}
		}
	}
	else
	{
		UWARN("Null images received!?");
	}

#else
	UERROR("CameraDepthAI: RTAB-Map is not built with depthai-core support!");
#endif
	return data;
}

} // namespace rtabmap
