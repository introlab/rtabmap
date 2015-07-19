/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/CameraRGB.h"
#include "rtabmap/core/DBDriver.h"

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <cmath>

namespace rtabmap
{

/////////////////////////
// CameraImages
/////////////////////////
CameraImages::CameraImages(const std::string & path,
					 int startAt,
					 bool refreshDir,
					 bool rectifyImages,
					 float imageRate,
					 const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_path(path),
	_startAt(startAt),
	_refreshDir(refreshDir),
	_rectifyImages(rectifyImages),
	_count(0),
	_dir(0)
{

}

CameraImages::~CameraImages(void)
{
	if(_dir)
	{
		delete _dir;
	}
}

bool CameraImages::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	_cameraName = cameraName;

	UDEBUG("");
	if(_dir)
	{
		_dir->setPath(_path, "jpg ppm png bmp pnm tiff");
	}
	else
	{
		_dir = new UDirectory(_path, "jpg ppm png bmp pnm tiff");
	}
	_count = 0;
	if(_path[_path.size()-1] != '\\' && _path[_path.size()-1] != '/')
	{
		_path.append("/");
	}
	if(!_dir->isValid())
	{
		ULOGGER_ERROR("Directory path is not valid \"%s\"", _path.c_str());
	}
	else if(_dir->getFileNames().size() == 0)
	{
		UWARN("Directory is empty \"%s\"", _path.c_str());
	}
	else
	{
		UINFO("path=%s images=%d", _path.c_str(), (int)this->imagesCount());
	}

	// look for calibration files
	if(!calibrationFolder.empty() && !cameraName.empty())
	{
		if(!_model.load(calibrationFolder + "/" + cameraName + ".yaml"))
		{
			UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
					cameraName.c_str(), calibrationFolder.c_str());
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
	if(_rectifyImages && !_model.isValid())
	{
		UERROR("Parameter \"rectifyImages\" is set, but no camera model is loaded or valid.");
		return false;
	}

	return _dir->isValid();
}

bool CameraImages::isCalibrated() const
{
	return _model.isValid();
}

std::string CameraImages::getSerial() const
{
	return _cameraName;
}

unsigned int CameraImages::imagesCount() const
{
	if(_dir)
	{
		return (unsigned int)_dir->getFileNames().size();
	}
	return 0;
}

SensorData CameraImages::captureImage()
{
	cv::Mat img;
	UDEBUG("");
	if(_dir->isValid())
	{
		if(_refreshDir)
		{
			_dir->update();
		}
		if(_startAt == 0)
		{
			const std::list<std::string> & fileNames = _dir->getFileNames();
			if(fileNames.size())
			{
				if(_lastFileName.empty() || uStrNumCmp(_lastFileName,*fileNames.rbegin()) < 0)
				{
					_lastFileName = *fileNames.rbegin();
					std::string fullPath = _path + _lastFileName;
					img = cv::imread(fullPath.c_str());
				}
			}
		}
		else
		{
			std::string fileName;
			std::string fullPath;
			fileName = _dir->getNextFileName();
			if(fileName.size())
			{
				fullPath = _path + fileName;
				while(++_count < _startAt && (fileName = _dir->getNextFileName()).size())
				{
					fullPath = _path + fileName;
				}
				if(fileName.size())
				{
					ULOGGER_DEBUG("Loading image : %s", fullPath.c_str());

#if CV_MAJOR_VERSION >2 || (CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >=4)
					img = cv::imread(fullPath.c_str(), cv::IMREAD_UNCHANGED);
#else
					img = cv::imread(fullPath.c_str(), -1);
#endif
					UDEBUG("width=%d, height=%d, channels=%d, elementSize=%d, total=%d",
							img.cols, img.rows, img.channels(), img.elemSize(), img.total());

#if CV_MAJOR_VERSION < 3
					// FIXME : it seems that some png are incorrectly loaded with opencv c++ interface, where c interface works...
					if(img.depth() != CV_8U)
					{
						// The depth should be 8U
						UWARN("Cannot read the image correctly, falling back to old OpenCV C interface...");
						IplImage * i = cvLoadImage(fullPath.c_str());
						img = cv::Mat(i, true);
						cvReleaseImage(&i);
					}
#endif

					if(img.channels()>3)
					{
						UWARN("Conversion from 4 channels to 3 channels (file=%s)", fullPath.c_str());
						cv::Mat out;
						cv::cvtColor(img, out, CV_BGRA2BGR);
						img = out;
					}
				}
			}
		}

		if(!img.empty() && _model.isValid() && _rectifyImages)
		{
			img = _model.rectifyImage(img);
		}
	}
	else
	{
		UWARN("Directory is not set, camera must be initialized.");
	}

	return SensorData(img, _model, this->getNextSeqID(), UTimer::now());
}



/////////////////////////
// CameraVideo
/////////////////////////
CameraVideo::CameraVideo(
		int usbDevice,
		float imageRate,
		const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_rectifyImages(false),
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
	_guid.clear();
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
		unsigned int guid = (unsigned int)_capture.get(CV_CAP_PROP_GUID);
		if(guid != 0 && guid != 0xffffffff)
		{
			_guid = uFormat("%08x", guid);
		}

		// look for calibration files
		if(!calibrationFolder.empty() && (!_guid.empty() || !cameraName.empty()))
		{
			if(!_model.load(calibrationFolder + "/" + (cameraName.empty()?_guid:cameraName) + ".yaml"))
			{
				UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
						cameraName.empty()?_guid.c_str():cameraName.c_str(), calibrationFolder.c_str());
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
		if(_rectifyImages && !_model.isValid())
		{
			UERROR("Parameter \"rectifyImages\" is set, but no camera model is loaded or valid.");
			return false;
		}
	}
	return true;
}

bool CameraVideo::isCalibrated() const
{
	return _model.isValid();
}

std::string CameraVideo::getSerial() const
{
	return _guid;
}

SensorData CameraVideo::captureImage()
{
	cv::Mat img;
	if(_capture.isOpened())
	{
		if(_capture.read(img))
		{
			if(_model.isValid() && (_src != kVideoFile || _rectifyImages))
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
