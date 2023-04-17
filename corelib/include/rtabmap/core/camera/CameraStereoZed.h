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

#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/camera/CameraVideo.h"
#include "rtabmap/core/Version.h"

namespace sl
{
class Camera;
}

namespace rtabmap
{
class ZedIMUThread;

class RTABMAP_CORE_EXPORT CameraStereoZed :
	public Camera
{
public:
	static bool available();
	static int sdkVersion();
public:
	CameraStereoZed(
			int deviceId,
			int resolution = 6, // 0=HD2K, 1=HD1080, 2=HD1200, 3=HD720, 4=SVGA, 5=VGA, 6=AUTO
			int quality = 1,    // 0=NONE, 1=PERFORMANCE, 2=QUALITY
			int sensingMode = 0,// 0=STANDARD, 1=FILL
			int confidenceThr = 100,
			bool computeOdometry = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity(),
			bool selfCalibration = true,
			bool odomForce3DoF = false,
			int texturenessConfidenceThr = 90); // introduced with ZED SDK 3
	CameraStereoZed(
			const std::string & svoFilePath,
			int quality = 1,    // 0=NONE, 1=PERFORMANCE, 2=QUALITY, 3=NEURAL
			int sensingMode = 0,// 0=STANDARD, 1=FILL
			int confidenceThr = 100,
			bool computeOdometry = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity(),
			bool selfCalibration = true,
			bool odomForce3DoF = false,
			int texturenessConfidenceThr = 90); // introduced with ZED SDK 3
	virtual ~CameraStereoZed();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;
	virtual bool odomProvided() const;
	virtual bool getPose(double stamp, Transform & pose, cv::Mat & covariance);

	void publishInterIMU(bool enabled);

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_ZED
	sl::Camera * zed_;
	StereoCameraModel stereoModel_;
	Transform imuLocalTransform_;
	CameraVideo::Source src_;
	int usbDevice_;
    std::string svoFilePath_;
	int resolution_;
	int quality_;
	bool selfCalibration_;
	int sensingMode_;
	int confidenceThr_;
	int texturenessConfidenceThr_; // introduced with ZED SDK 3
	bool computeOdometry_;
	bool lost_;
	bool force3DoF_;
	bool publishInterIMU_;
	ZedIMUThread * imuPublishingThread_;
#endif
};


} // namespace rtabmap
