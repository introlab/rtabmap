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
	resolution_(resolution),
	imuFirmwareUpdate_(false)
#endif
{
#ifdef RTABMAP_DEPTHAI
	UASSERT(resolution_>=0 && resolution_<=2);
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

void CameraDepthAI::setIMUFirmwareUpdate(bool enabled)
{
#ifdef RTABMAP_DEPTHAI
	imuFirmwareUpdate_ = enabled;
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
		return false;
	}

	if(device_.get())
	{
		device_->close();
	}
	accBuffer_.clear();
	gyroBuffer_.clear();

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
	cv::Size targetSize(resolution_<2?1280:640, resolution_==0?720:resolution_==1?800:400);

	dai::Pipeline p;
	auto monoLeft  = p.create<dai::node::MonoCamera>();
	auto monoRight = p.create<dai::node::MonoCamera>();
	auto stereo    = p.create<dai::node::StereoDepth>();
	auto imu       = p.create<dai::node::IMU>();
	auto xoutLeft = p.create<dai::node::XLinkOut>();
	auto xoutDepthOrRight = p.create<dai::node::XLinkOut>();
	auto xoutIMU = p.create<dai::node::XLinkOut>();

	// XLinkOut
	xoutLeft->setStreamName("rectified_left");
	xoutDepthOrRight->setStreamName(outputDepth_?"depth":"rectified_right");
	xoutIMU->setStreamName("imu");

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
	stereo->initialConfig.setConfidenceThreshold(depthConfidence_);
	stereo->initialConfig.setLeftRightCheckThreshold(5);
	stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
	stereo->setLeftRightCheck(true);
	stereo->setSubpixel(false);
	stereo->setExtendedDisparity(false);

	// Link plugins CAM -> STEREO -> XLINK
	monoLeft->out.link(stereo->left);
	monoRight->out.link(stereo->right);

	if(outputDepth_)
	{
		// Depth is registered to right image by default, so subscribe to right image when depth is used
		if(outputDepth_)
			stereo->rectifiedRight.link(xoutLeft->input);
		else
			stereo->rectifiedLeft.link(xoutLeft->input);
		stereo->depth.link(xoutDepthOrRight->input);
	}
	else
	{
		stereo->rectifiedLeft.link(xoutLeft->input);
		stereo->rectifiedRight.link(xoutDepthOrRight->input);
	}

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

	imu->enableFirmwareUpdate(imuFirmwareUpdate_);

	device_.reset(new dai::Device(p, deviceToUse));

	UINFO("Loading eeprom calibration data");
	dai::CalibrationHandler calibHandler = device_->readCalibration();
	std::vector<std::vector<float> > matrix = calibHandler.getCameraIntrinsics(dai::CameraBoardSocket::LEFT, dai::Size2f(targetSize.width, targetSize.height));
	double fx = matrix[0][0];
	double fy = matrix[1][1];
	double cx = matrix[0][2];
	double cy = matrix[1][2];
	matrix = calibHandler.getCameraExtrinsics(dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::LEFT);
	double baseline = matrix[0][3]/100.0;
	UINFO("left: fx=%f fy=%f cx=%f cy=%f baseline=%f", fx, fy, cx, cy, baseline);
	stereoModel_ = StereoCameraModel(device_->getMxId(), fx, fy, cx, cy, baseline, this->getLocalTransform(), targetSize);

	// Cannot test the following, I get "IMU calibration data is not available on device yet." with my camera
	// Update: now (as March 6, 2022) it crashes in "dai::CalibrationHandler::getImuToCameraExtrinsics(dai::CameraBoardSocket, bool)"
	//matrix = calibHandler.getImuToCameraExtrinsics(dai::CameraBoardSocket::LEFT);
	//imuLocalTransform_ = Transform(
	//		matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3],
	//		matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3],
	//		matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3]);
	// Hard-coded: x->down, y->left, z->forward
	imuLocalTransform_ = Transform(
			 0, 0, 1, 0,
			 0, 1, 0, 0,
			-1 ,0, 0, 0);
	UINFO("IMU local transform = %s", imuLocalTransform_.prettyPrint().c_str());

	leftQueue_ = device_->getOutputQueue("rectified_left", 8, false);
	rightOrDepthQueue_ = device_->getOutputQueue(outputDepth_?"depth":"rectified_right", 8, false);
	imuQueue_ = device_->getOutputQueue("imu", 50, false);

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
				data = SensorData(left, depthOrRight, stereoModel_.left(), this->getNextSeqID(), stamp);
			}

			if(fabs(double(stampLeft)/10e8 - double(stampRight)/10e8) >= 0.0001) //0.1 ms
			{
				UWARN("Frames are not synchronized! %f vs %f", double(stampLeft)/10e8, double(stampRight)/10e8);
			}

			//get imu
			int added=  0;
			while(1)
			{
				auto imuData = imuQueue_->get<dai::IMUData>();

				auto imuPackets = imuData->packets;
				double accStamp = 0.0;
				double gyroStamp = 0.0;
				for(auto& imuPacket : imuPackets) {
					auto& acceleroValues = imuPacket.acceleroMeter;
					auto& gyroValues = imuPacket.gyroscope;

					accStamp = double(acceleroValues.timestamp.get().time_since_epoch().count())/10e8;
					gyroStamp = double(gyroValues.timestamp.get().time_since_epoch().count())/10e8;
					accBuffer_.insert(accBuffer_.end(), std::make_pair(accStamp, cv::Vec3f(acceleroValues.x, acceleroValues.y, acceleroValues.z)));
					gyroBuffer_.insert(gyroBuffer_.end(), std::make_pair(gyroStamp, cv::Vec3f(gyroValues.x, gyroValues.y, gyroValues.z)));
					if(accBuffer_.size() > 1000)
					{
						accBuffer_.erase(accBuffer_.begin());
					}
					if(gyroBuffer_.size() > 1000)
					{
						gyroBuffer_.erase(gyroBuffer_.begin());
					}
					++added;
				}
				if(accStamp >= stamp && gyroStamp >= stamp)
				{
					break;
				}
			}

			cv::Vec3d acc, gyro;
			bool valid = !accBuffer_.empty() && !gyroBuffer_.empty();
			//acc
			if(!accBuffer_.empty())
			{
				std::map<double, cv::Vec3f>::const_iterator iterB = accBuffer_.lower_bound(stamp);
				std::map<double, cv::Vec3f>::const_iterator iterA = iterB;
				if(iterA != accBuffer_.begin())
				{
					iterA = --iterA;
				}
				if(iterB == accBuffer_.end())
				{
					iterB = --iterB;
				}
				if(iterA == iterB && stamp == iterA->first)
				{
					acc[0] = iterA->second[0];
					acc[1] = iterA->second[1];
					acc[2] = iterA->second[2];
				}
				else if(stamp >= iterA->first && stamp <= iterB->first)
				{
					float t = (stamp-iterA->first) / (iterB->first-iterA->first);
					acc[0] = iterA->second[0] + t*(iterB->second[0] - iterA->second[0]);
					acc[1] = iterA->second[1] + t*(iterB->second[1] - iterA->second[1]);
					acc[2] = iterA->second[2] + t*(iterB->second[2] - iterA->second[2]);
				}
				else
				{
					valid = false;
					if(stamp < iterA->first)
					{
						UWARN("Could not find acc data to interpolate at image time %f (earliest is %f). Are sensors synchronized?", stamp, iterA->first);
					}
					else if(stamp > iterB->first)
					{
						UWARN("Could not find acc data to interpolate at image time %f (latest is %f). Are sensors synchronized?", stamp, iterB->first);
					}
					else
					{
						UWARN("Could not find acc data to interpolate at image time %f (between %f and %f). Are sensors synchronized?", stamp, iterA->first, iterB->first);
					}
				}
			}
			//gyro
			if(!gyroBuffer_.empty())
			{
				std::map<double, cv::Vec3f>::const_iterator iterB = gyroBuffer_.lower_bound(stamp);
				std::map<double, cv::Vec3f>::const_iterator iterA = iterB;
				if(iterA != gyroBuffer_.begin())
				{
					iterA = --iterA;
				}
				if(iterB == gyroBuffer_.end())
				{
					iterB = --iterB;
				}
				if(iterA == iterB && stamp == iterA->first)
				{
					gyro[0] = iterA->second[0];
					gyro[1] = iterA->second[1];
					gyro[2] = iterA->second[2];
				}
				else if(stamp >= iterA->first && stamp <= iterB->first)
				{
					float t = (stamp-iterA->first) / (iterB->first-iterA->first);
					gyro[0] = iterA->second[0] + t*(iterB->second[0] - iterA->second[0]);
					gyro[1] = iterA->second[1] + t*(iterB->second[1] - iterA->second[1]);
					gyro[2] = iterA->second[2] + t*(iterB->second[2] - iterA->second[2]);
				}
				else
				{
					valid = false;
					if(stamp < iterA->first)
					{
						UWARN("Could not find gyro data to interpolate at image time %f (earliest is %f). Are sensors synchronized?", stamp, iterA->first);
					}
					else if(stamp > iterB->first)
					{
						UWARN("Could not find gyro data to interpolate at image time %f (latest is %f). Are sensors synchronized?", stamp, iterB->first);
					}
					else
					{
						UWARN("Could not find gyro data to interpolate at image time %f (between %f and %f). Are sensors synchronized?", stamp, iterA->first, iterB->first);
					}
				}
			}

			if(valid)
			{
				data.setIMU(IMU(gyro, cv::Mat::eye(3, 3, CV_64FC1), acc, cv::Mat::eye(3, 3, CV_64FC1), imuLocalTransform_));
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
