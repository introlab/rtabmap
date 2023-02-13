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

namespace rtabmap
{

class RTABMAP_CORE_EXPORT CameraStereoVideo :
	public Camera
{
public:
	static bool available();

public:
	CameraStereoVideo(
			const std::string & pathSideBySide,
			bool rectifyImages = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity());
	CameraStereoVideo(
			const std::string & pathLeft,
			const std::string & pathRight,
			bool rectifyImages = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity());
	CameraStereoVideo(
			int device,
			bool rectifyImages = false,
			float imageRate = 0.0f,
			const Transform & localTransform = Transform::getIdentity());
	CameraStereoVideo(
			int deviceLeft,
			int deviceRight,
			bool rectifyImages = false,
			float imageRate = 0.0f,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraStereoVideo();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

	void setResolution(int width, int height) {_width=width, _height=height;}

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	cv::VideoCapture capture_;
	cv::VideoCapture capture2_;
	std::string path_;
	std::string path2_;
	bool rectifyImages_;
	StereoCameraModel stereoModel_;
	std::string cameraName_;
	CameraVideo::Source src_;
	int usbDevice_;
	int usbDevice2_;
	int _width;
	int _height;
};

} // namespace rtabmap
