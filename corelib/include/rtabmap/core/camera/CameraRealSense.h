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

#include "rtabmap/utilite/UMutex.h"
#include "rtabmap/utilite/USemaphore.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"

namespace rs
{
	class context;
	class device;
	namespace slam {
	class slam;
	}
}

namespace rtabmap
{

class slam_event_handler;
class RTABMAP_CORE_EXPORT CameraRealSense :
	public Camera
{
public:
	static bool available();
	enum RGBSource {kColor, kInfrared, kFishEye};

public:
	// default local transform z in, x right, y down));
	CameraRealSense(
		int deviceId = 0,
		int presetRGB = 0, // 0=best quality, 1=largest image, 2=highest framerate
		int presetDepth = 0, // 0=best quality, 1=largest image, 2=highest framerate
		bool computeOdometry = false,
		float imageRate = 0,
		const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraRealSense();

	void setDepthScaledToRGBSize(bool enabled);
	void setRGBSource(RGBSource source);
	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;
	virtual bool odomProvided() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_REALSENSE
	rs::context * ctx_;
	rs::device * dev_;
	int deviceId_;
	int presetRGB_;
	int presetDepth_;
	bool computeOdometry_;
	bool depthScaledToRGBSize_;
	RGBSource rgbSource_;
	CameraModel cameraModel_;
	std::vector<int> rsRectificationTable_;

	int motionSeq_[2];
	rs::slam::slam * slam_;
	UMutex slamLock_;

	std::map<double, std::pair<cv::Mat, cv::Mat> > bufferedFrames_;
	std::pair<cv::Mat, cv::Mat> lastSyncFrames_;
	UMutex dataMutex_;
	USemaphore dataReady_;
#endif
};

} // namespace rtabmap
