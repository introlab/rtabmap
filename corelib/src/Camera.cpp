/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rtabmap/core/Camera.h"
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

namespace rtabmap
{

Camera::Camera(float imageRate,
		unsigned int imageWidth,
		unsigned int imageHeight,
		unsigned int framesDropped) :
	_imageRate(imageRate),
	_imageWidth(imageWidth),
	_imageHeight(imageHeight),
	_framesDropped(framesDropped),
	_localTransform(Transform::getIdentity()),
	_frameRateTimer(new UTimer())
{
}

Camera::~Camera()
{
	if(_frameRateTimer)
	{
		delete _frameRateTimer;
	}
}

void Camera::setImageSize(unsigned int width, unsigned int height)
{
	_imageWidth = width;
	_imageHeight = height;
}

void Camera::getImageSize(unsigned int & width, unsigned int & height)
{
	width = _imageWidth;
	height = _imageHeight;
}

cv::Mat Camera::takeImage()
{
	cv::Mat rgb, depth;
	float depthConstant = 0.0f;
	takeImage(rgb, depth, depthConstant);
	return rgb;
}

void Camera::takeImage(cv::Mat & rgb)
{
	cv::Mat depth;
	float depthConstant = 0.0f;
	takeImage(rgb, depth, depthConstant);
}

void Camera::takeImage(cv::Mat & rgb, cv::Mat & depth, float & depthConstant)
{
	float imageRate = _imageRate==0.0f?33.0f:_imageRate; // limit to 33Hz if infinity
	if(imageRate>0)
	{
		int sleepTime = (1000.0f/imageRate - 1000.0f*_frameRateTimer->getElapsedTime());
		if(sleepTime > 2)
		{
			uSleep(sleepTime-2);
		}

		// Add precision at the cost of a small overhead
		while(_frameRateTimer->getElapsedTime() < 1.0/double(imageRate)-0.000001)
		{
			//
		}

		double slept = _frameRateTimer->getElapsedTime();
		_frameRateTimer->start();
		UDEBUG("slept=%fs vs target=%fs", slept, 1.0/double(imageRate));
	}

	UTimer timer;
	this->captureImage(rgb, depth, depthConstant);
	UDEBUG("Time capturing image = %fs", timer.ticks());

	if(!rgb.empty())
	{
		UASSERT(rgb.depth() == CV_8U);

		if(_framesDropped)
		{
			unsigned int count = 0;
			while(count++ < _framesDropped)
			{
				cv::Mat tmp,tmp2;
				float tmpf;
				this->captureImage(tmp, tmp2, tmpf);
				if(!tmp.empty())
				{
					UDEBUG("frame dropped (%d/%d)", (int)count, (int)_framesDropped);
				}
				else
				{
					break;
				}
			}
			UDEBUG("Frames dropped time = %fs", timer.ticks());
		}
	}
}

/////////////////////////
// CameraImages
/////////////////////////
CameraImages::CameraImages(const std::string & path,
					 int startAt,
					 bool refreshDir,
					 float imageRate,
					 unsigned int imageWidth,
					 unsigned int imageHeight,
					 unsigned int framesDropped) :
	Camera(imageRate, imageWidth, imageHeight, framesDropped),
	_path(path),
	_startAt(startAt),
	_refreshDir(refreshDir),
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

bool CameraImages::init()
{
	UDEBUG("");
	if(_dir)
	{
		_dir->setPath(_path, "jpg ppm png bmp pnm");
	}
	else
	{
		_dir = new UDirectory(_path, "jpg ppm png bmp pnm");
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
	return _dir->isValid();
}

void CameraImages::captureImage(cv::Mat & rgb, cv::Mat & depth, float & depthConstant)
{
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
					rgb = cv::imread(fullPath.c_str());
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
#if CV_MAJOR_VERSION >=2 and CV_MINOR_VERSION >=4
					rgb = cv::imread(fullPath.c_str(), cv::IMREAD_UNCHANGED);
#else
					rgb = cv::imread(fullPath.c_str(), -1);
#endif
					UDEBUG("width=%d, height=%d, channels=%d, elementSize=%d, total=%d", rgb.cols, rgb.rows, rgb.channels(), rgb.elemSize(), rgb.total());

					// FIXME : it seems that some png are incorrectly loaded with opencv c++ interface, where c interface works...
					if(rgb.depth() != CV_8U)
					{
						// The depth should be 8U
						UWARN("Cannot read the image correctly, falling back to old OpenCV C interface...");
						IplImage * i = cvLoadImage(fullPath.c_str());
						rgb = cv::Mat(i, true);
						cvReleaseImage(&i);
					}
				}
			}
		}
	}
	else
	{
		UWARN("Directory is not set, camera must be initialized.");
	}

	unsigned int w;
	unsigned int h;
	this->getImageSize(w, h);

	if(!rgb.empty() &&
	   w &&
	   h &&
	   w != (unsigned int)rgb.cols &&
	   h != (unsigned int)rgb.rows)
	{
		cv::Mat resampled;
		cv::resize(rgb, resampled, cv::Size(w, h));
		rgb = resampled;
	}
}



/////////////////////////
// CameraVideo
/////////////////////////
CameraVideo::CameraVideo(int usbDevice,
						 float imageRate,
						 unsigned int imageWidth,
						 unsigned int imageHeight,
						 unsigned int framesDropped) :
	Camera(imageRate, imageWidth, imageHeight, framesDropped),
	_src(kUsbDevice),
	_usbDevice(usbDevice)
{

}

CameraVideo::CameraVideo(const std::string & filePath,
						   float imageRate,
						   unsigned int imageWidth,
						   unsigned int imageHeight,
						   unsigned int framesDropped) :
	Camera(imageRate, imageWidth, imageHeight, framesDropped),
	_filePath(filePath),
	_src(kVideoFile),
	_usbDevice(0)
{
}

CameraVideo::~CameraVideo()
{
	_capture.release();
}

bool CameraVideo::init()
{
	if(_capture.isOpened())
	{
		_capture.release();
	}

	if(_src == kUsbDevice)
	{
		unsigned int w;
		unsigned int h;
		this->getImageSize(w, h);

		ULOGGER_DEBUG("CameraVideo::init() Usb device initialization on device %d with imgSize=[%d,%d]", _usbDevice, w, h);
		_capture.open(_usbDevice);

		if(w && h)
		{
			_capture.set(CV_CAP_PROP_FRAME_WIDTH, double(w));
			_capture.set(CV_CAP_PROP_FRAME_HEIGHT, double(h));
		}
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
	return true;
}

void CameraVideo::captureImage(cv::Mat & rgb, cv::Mat & depth, float & depthConstant)
{
	if(_capture.isOpened())
	{
		if(_capture.read(rgb))
		{
			unsigned int w;
			unsigned int h;
			this->getImageSize(w, h);

			if(!rgb.empty() &&
			   w &&
			   h &&
			   w != (unsigned int)rgb.cols &&
			   h != (unsigned int)rgb.rows)
			{
				cv::Mat resampled;
				cv::resize(rgb, resampled, cv::Size(w, h));
				rgb = resampled;
			}
			else
			{
				// clone required
				rgb = rgb.clone();
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
}

/////////////////////////
// CameraRGBD
/////////////////////////
CameraRGBD::CameraRGBD(float imageRate, bool asus) :
	Camera(imageRate),
	_asus(asus)
{

}

CameraRGBD::~CameraRGBD()
{
	_capture.release();
}

bool CameraRGBD::init()
{
	if(_capture.isOpened())
	{
		_capture.release();
	}

	ULOGGER_DEBUG("CameraRGBD::init()");
	_capture.open( _asus?CV_CAP_OPENNI_ASUS:CV_CAP_OPENNI );
	if(_capture.isOpened())
	{
		_capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
		// Print some avalible device settings.
		UINFO("Depth generator output mode:");
		UINFO("FRAME_WIDTH        %f", _capture.get( CV_CAP_PROP_FRAME_WIDTH ));
		UINFO("FRAME_HEIGHT       %f", _capture.get( CV_CAP_PROP_FRAME_HEIGHT ));
		UINFO("FRAME_MAX_DEPTH    %f mm", _capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ));
		UINFO("FPS                %f", _capture.get( CV_CAP_PROP_FPS ));
		UINFO("REGISTRATION       %f", _capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ));
		if(_capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ) == 0.0)
		{
			UERROR("Depth registration is not activated on this device!");
		}
		if( _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) )
		{
			UINFO("Image generator output mode:");
			UINFO("FRAME_WIDTH    %f", _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ));
			UINFO("FRAME_HEIGHT   %f", _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ));
			UINFO("FPS            %f", _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ));
		}
		else
		{
			UERROR("CameraRGBD: Device doesn't contain image generator.");
			_capture.release();
			return false;
		}
	}
	else
	{
		ULOGGER_ERROR("CameraRGBD: Failed to create a capture object!");
		_capture.release();
		return false;
	}
	return true;
}

void CameraRGBD::captureImage(cv::Mat & rgb, cv::Mat & depth, float & depthConstant)
{
	if(_capture.isOpened())
	{
		_capture.grab();
		_capture.retrieve( depth, CV_CAP_OPENNI_DEPTH_MAP );
		_capture.retrieve( rgb, CV_CAP_OPENNI_BGR_IMAGE );

		depth = depth.clone();
		rgb = rgb.clone();
		depthConstant = 0.001905f;
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}
}

} // namespace rtabmap
