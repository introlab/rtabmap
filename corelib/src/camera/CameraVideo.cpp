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
#if CV_MAJOR_VERSION > 4
#include <opencv2/videoio/legacy/constants_c.h>
#endif
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
	_usbDevice(usbDevice),
	_width(0),
	_height(0)
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
	_usbDevice(0),
	_width(0),
	_height(0)
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
		if(_src == kUsbDevice)
		{
			if(_model.isValidForProjection())
			{
				if(_width > 0 && _height > 0 && (_width!=_model.imageWidth() || _height != _model.imageHeight()))
				{
					UWARN("Desired resolution of %dx%d is set but calibration has "
						"been loaded with resolution %dx%d, using calibration resolution.",
						_width, _height,
						_model.imageWidth(), _model.imageHeight());
				}

				bool resolutionSet = false;
				resolutionSet = _capture.set(CV_CAP_PROP_FRAME_WIDTH, _model.imageWidth());
				resolutionSet = resolutionSet && _capture.set(CV_CAP_PROP_FRAME_HEIGHT, _model.imageHeight());

				// Check if the resolution was set successfully
				int actualWidth = int(_capture.get(CV_CAP_PROP_FRAME_WIDTH));
				int actualHeight = int(_capture.get(CV_CAP_PROP_FRAME_HEIGHT));
				if(!resolutionSet ||
				   actualWidth != _model.imageWidth() ||
				   actualHeight != _model.imageHeight())
				{
					UERROR("Calibration resolution (%dx%d) cannot be set to camera driver, "
					      "actual resolution is %dx%d. You would have to re-calibrate with one "
						  "supported format by your camera. "
						  "Do \"v4l2-ctl --list-formats-ext\" to list all supported "
						  "formats by your camera.",
						  _model.imageWidth(), _model.imageHeight(),
						  actualWidth, actualHeight);
				}
			}
			else if(_width > 0 && _height > 0)
			{
				int resolutionSet = false;
				resolutionSet = _capture.set(CV_CAP_PROP_FRAME_WIDTH, _width);
				resolutionSet = resolutionSet && _capture.set(CV_CAP_PROP_FRAME_HEIGHT, _height);

				// Check if the resolution was set successfully
				int actualWidth = int(_capture.get(CV_CAP_PROP_FRAME_WIDTH));
				int actualHeight = int(_capture.get(CV_CAP_PROP_FRAME_HEIGHT));
				if(!resolutionSet || actualWidth != _width || actualHeight != _height)
				{
					UWARN("Desired resolution (%dx%d) cannot be set to camera driver, "
					      "actual resolution is %dx%d. "
						  "Do \"v4l2-ctl --list-formats-ext\" to list all supported "
						  "formats by your camera.",
						  _width, _height, actualWidth, actualHeight);
				}
			}

			// Set FPS
			if (this->getFrameRate() > 0 && _capture.set(CV_CAP_PROP_FPS, this->getFrameRate()))
			{
				// Check if the FPS was set successfully
				double actualFPS = _capture.get(cv::CAP_PROP_FPS);
				
				if(fabs(actualFPS - this->getFrameRate()) < 0.01)
				{
					this->setFrameRate(0);
				}
				else
				{
					UWARN("Desired FPS (%f Hz) cannot be set to camera driver, "
						"actual FPS is %f Hz. We will throttle to lowest FPS. "
						"Do \"v4l2-ctl --list-formats-ext\" to list all supported "
						"formats by your camera.",
						this->getFrameRate(), actualFPS);
					if(this->getFrameRate() > actualFPS)
					{
						this->setFrameRate(0);
					}
				}
			}

			// Set FOURCC
			if (!_fourcc.empty())
			{
				if(_fourcc.size() == 4)
				{
					std::string fourccUpperCase = uToUpperCase(_fourcc);
					int fourcc = cv::VideoWriter::fourcc(fourccUpperCase.at(0), fourccUpperCase.at(1), fourccUpperCase.at(2), fourccUpperCase.at(3));
					
					bool fourccSupported = _capture.set(CV_CAP_PROP_FOURCC, fourcc);

					// Check if the FOURCC was set successfully
					int actualFourcc = int(_capture.get(CV_CAP_PROP_FOURCC));

					if(!fourccSupported || actualFourcc != fourcc)
					{
						UWARN("Camera doesn't support provided FOURCC \"%s\". "
							"Do \"v4l2-ctl --list-formats-ext\" to list all supported "
							"formats by your camera.", fourccUpperCase.c_str());
					}
				}
				else
				{
					UERROR("FOURCC parameter should be 4 characters, current value is \"%s\"", _fourcc.c_str());
				}
			}
		}
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

SensorData CameraVideo::captureImage(SensorCaptureInfo * info)
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
