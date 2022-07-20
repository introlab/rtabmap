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

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"

#ifdef RTABMAP_DEPTHAI
#ifndef DEPTHAI_OPENCV_SUPPORT
#define DEPTHAI_OPENCV_SUPPORT
#endif
#include <depthai/depthai.hpp>
#endif

namespace rtabmap
{

class RTABMAP_EXP CameraDepthAI :
	public Camera
{
public:
	static bool available();

public:
	CameraDepthAI(
			const std::string & deviceSerial = "",
			int resolution = 1, // 0=720p, 1=800p, 2=400p
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraDepthAI();

	void setOutputDepth(bool enabled, int confidence = 200);
	void setIMUFirmwareUpdate(bool enabled);

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_DEPTHAI
	StereoCameraModel stereoModel_;
	Transform imuLocalTransform_;
	std::string deviceSerial_;
	bool outputDepth_;
	int depthConfidence_;
	int resolution_;
	bool imuFirmwareUpdate_;
	std::shared_ptr<dai::Device> device_;
	std::shared_ptr<dai::DataOutputQueue> leftQueue_;
	std::shared_ptr<dai::DataOutputQueue> rightOrDepthQueue_;
	std::shared_ptr<dai::DataOutputQueue> imuQueue_;
	std::map<double, cv::Vec3f> accBuffer_;
	std::map<double, cv::Vec3f> gyroBuffer_;
#endif
};


} // namespace rtabmap
