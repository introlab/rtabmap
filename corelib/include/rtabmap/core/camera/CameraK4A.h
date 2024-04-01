/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"
#include "rtabmap/utilite/UTimer.h"

#ifdef RTABMAP_K4A
#include <k4a/k4atypes.h>
#include <k4arecord/playback.h>
#endif

namespace rtabmap
{

class RTABMAP_CORE_EXPORT CameraK4A :
	public Camera
{
public:
	static bool available();

public:
	CameraK4A(int deviceId = 0,
		float imageRate = 0.0f,
		const Transform & localTransform = Transform::getIdentity());
	CameraK4A(const std::string & fileName,
		float imageRate = 0.0f,
		const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraK4A();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

	void setIRDepthFormat(bool enabled);
	void setPreferences(int rgb_resolution, int framerate, int depth_resolution);

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	void close();

private:

#ifdef RTABMAP_K4A
	k4a_device_t deviceHandle_;
	k4a_device_configuration_t config_;
	k4a_calibration_t calibration_;
	k4a_transformation_t transformationHandle_;
	k4a_capture_t captureHandle_;
	k4a_playback_t playbackHandle_;
	std::string serial_number_;

	CameraModel model_;
	int deviceId_;
	std::string fileName_;
	int rgb_resolution_;
	int framerate_;
	int depth_resolution_;
	bool ir_;
	double previousStamp_;
	double timestampOffset_;
	UTimer timer_;
	Transform imuLocalTransform_;
#endif

};


} // namespace rtabmap
