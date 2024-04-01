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

#include <opencv2/highgui/highgui.hpp>
#include "rtabmap/core/Camera.h"

namespace rtabmap
{

class RTABMAP_CORE_EXPORT CameraVideo :
	public Camera
{
public:
	enum Source{kVideoFile, kUsbDevice};

public:
	CameraVideo(int usbDevice = 0,
			bool rectifyImages = false,
			float imageRate = 0,
			const Transform & localTransform = Transform::getIdentity());
	CameraVideo(const std::string & filePath,
			bool rectifyImages = false,
			float imageRate = 0,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraVideo();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;
	int getUsbDevice() const {return _usbDevice;}
	const std::string & getFilePath() const {return _filePath;}

	/**
	 * Set wanted usb resolution, should be set before initialization. 0 means
	 * default resolution. It won't be applied if a valid camera calibration
	 * has been loaded, thus resolution from calibration is used.
	 * */
	void setResolution(int width, int height) {_width=width, _height=height;}

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	// File type
	std::string _filePath;
	bool _rectifyImages;

	cv::VideoCapture _capture;
	Source _src;

	// Usb camera
	int _usbDevice;
	std::string _guid;
	int _width;
	int _height;

	CameraModel _model;
};


} // namespace rtabmap
