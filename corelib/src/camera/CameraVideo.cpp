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

#include <rtabmap/core/camera/CameraVideo.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#if CV_MAJOR_VERSION > 3
#include <opencv2/videoio/videoio_c.h>
#endif

namespace rtabmap
{

CameraVideo::CameraVideo(
		int usbDevice,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_rectifyImages(rectifyImages),
	_src(kUsbDevice),
	_usbDevice(usbDevice)
{

}

CameraVideo::CameraVideo(
		const std::string & filePath,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_filePath(filePath),
	_rectifyImages(rectifyImages),
	_src(kVideoFile),
	_usbDevice(0)
{
}

CameraVideo::~CameraVideo()
{
	_capture.release();
}

bool CameraVideo::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	_guid = cameraName;
	if(_capture.isOpened())
	{
		_capture.release();
	}

	if(_src == kUsbDevice)
	{
		ULOGGER_DEBUG("CameraVideo::init() Usb device initialization on device %d", _usbDevice);
		_capture.open(_usbDevice);
	}
	else if(_src == kVideoFile)
	{
		ULOGGER_DEBUG("Camera: filename=\"%s\"", _filePath.c_str());
		_capture.open(_filePath.c_str());
	}
	else
	{
		ULOGGER_ERROR("Camera: Unknown source...");
	}
	if(!_capture.isOpened())
	{
		ULOGGER_ERROR("Camera: Failed to create a capture object!");
		_capture.release();
		return false;
	}
	else
	{
		if (_guid.empty())
		{
			unsigned int guid = (unsigned int)_capture.get(CV_CAP_PROP_GUID);
			if (guid != 0 && guid != 0xffffffff)
			{
				_guid = uFormat("%08x", guid);
			}
		}

		// look for calibration files
		if(!calibrationFolder.empty() && !_guid.empty())
		{
			if(!_model.load(calibrationFolder, _guid))
			{
				UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
						_guid.c_str(), calibrationFolder.c_str());
			}
			else
			{
				UINFO("Camera parameters: fx=%f fy=%f cx=%f cy=%f",
						_model.fx(),
						_model.fy(),
						_model.cx(),
						_model.cy());
			}
		}
		_model.setLocalTransform(this->getLocalTransform());
		if(_rectifyImages && !_model.isValidForRectification())
		{
			UERROR("Parameter \"rectifyImages\" is set, but no camera model is loaded or valid.");
			return false;
		}
	}
	return true;
}

bool CameraVideo::isCalibrated() const
{
	return _model.isValidForProjection();
}

std::string CameraVideo::getSerial() const
{
	return _guid;
}

SensorData CameraVideo::captureImage(CameraInfo * info)
{
	cv::Mat img;
	if(_capture.isOpened())
	{
		if(_capture.read(img))
		{
			if(_model.imageHeight() == 0 || _model.imageWidth() == 0)
			{
				_model.setImageSize(img.size());
			}

			if(_model.isValidForRectification() && _rectifyImages)
			{
				img = _model.rectifyImage(img);
			}
			else
			{
				// clone required
				img = img.clone();
			}
		}
		else if(_usbDevice)
		{
			UERROR("Camera has been disconnected!");
		}
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}

	return SensorData(img, _model, this->getNextSeqID(), UTimer::now());
}

} // namespace rtabmap
