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

#pragma once

#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"
#include "rtabmap/utilite/USemaphore.h"
#include <memory>

namespace mynteye {
class Device;
class API;
}

namespace rtabmap
{

/**
 * Class CameraMyntEye
 *
 */
class RTABMAP_CORE_EXPORT CameraMyntEye : public Camera
{
public:
	static bool available();

public:
	CameraMyntEye(const std::string & device = "", bool apiRectification = false, bool apiDepth = false, float imageRate = 0, const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraMyntEye();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;
	virtual bool odomProvided() const { return false; }

	void publishInterIMU(bool enabled);
	void setAutoExposure();
	void setManualExposure(int gain=24, int brightness=120, int constrast=116);
	void setIrControl(int value);

protected:
	/**
	 * returned rgb and depth images should be already rectified if calibration was loaded
	 */
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_MYNTEYE
	double hardTimeToSoftTime(std::uint64_t hardTime);
	void getPoseAndIMU(const double & stamp, IMU & imu, int maxWaitTimeMs = 35) const;
	double checkUpTimeStamp(std::uint64_t _hard_time, std::uint8_t stream);

	std::shared_ptr<mynteye::Device> device_;
	std::shared_ptr<mynteye::API> api_;
	StereoCameraModel stereoModel_;
	std::string deviceName_;
	bool apiRectification_;
	bool apiDepth_;
	bool autoExposure_;
	int gain_;
	int brightness_;
	int contrast_;
	int irControl_;
	USemaphore dataReady_;
	UMutex dataMutex_;
	cv::Mat leftFrameBuffer_;
	cv::Mat rightFrameBuffer_;
	std::pair<cv::Mat, cv::Mat> lastFrames_;
	double lastFramesStamp_;
	std::uint64_t stamp_;
	bool publishInterIMU_;
	Transform imuLocalTransform_;
	std::map<double, std::pair<cv::Vec3f, cv::Vec3f> > imuBuffer_;
	UMutex imuMutex_;

	double softTimeBegin_;
	std::uint64_t hardTimeBegin_;
	std::uint64_t unitHardTime_;
	std::vector<std::uint64_t> lastHardTimes_;
	std::vector<std::uint64_t> acc_;
#endif
};


} // namespace rtabmap
