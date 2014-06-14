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
#include <cmath>

#ifdef WITH_OPENNI2

#endif
#include <OpenNI.h>

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
bool CameraRGBD::available()
{
	return cv::getBuildInformation().find("OpenNI:                      YES") != std::string::npos;
}

CameraRGBD::CameraRGBD(float imageRate, bool asus) :
	Camera(imageRate),
	_asus(asus),
	_depthFocal(0.0f)
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
		_depthFocal = _capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH );
		// Print some avalible device settings.
		UINFO("Depth generator output mode:");
		UINFO("FRAME_WIDTH        %f", _capture.get( CV_CAP_PROP_FRAME_WIDTH ));
		UINFO("FRAME_HEIGHT       %f", _capture.get( CV_CAP_PROP_FRAME_HEIGHT ));
		UINFO("FRAME_MAX_DEPTH    %f mm", _capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ));
		UINFO("BASELINE           %f mm", _capture.get( CV_CAP_PROP_OPENNI_BASELINE ));
		UINFO("FPS                %f", _capture.get( CV_CAP_PROP_FPS ));
		UINFO("Focal              %f", _capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH ));
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
		UASSERT(_depthFocal > 0.0f);
		depthConstant = 1.0f/_depthFocal;
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}
}


/////////////////////////
// CameraOpenNI2
/////////////////////////
bool CameraOpenNI2::available()
{
#ifdef WITH_OPENNI2
	return true;
#else
	return false;
#endif
}

CameraOpenNI2::CameraOpenNI2(float imageRate) :
	Camera(imageRate),
	_device(new openni::Device()),
	_color(new openni::VideoStream()),
	_depth(new openni::VideoStream()),
	_depthFocal(0.0f)
{
}

CameraOpenNI2::~CameraOpenNI2()
{
	_color->stop();
	_color->destroy();
	_depth->stop();
	_depth->destroy();
	_device->close();
	openni::OpenNI::shutdown();

	delete _device;
	delete _color;
	delete _depth;
}

bool CameraOpenNI2::init()
{
	openni::OpenNI::initialize();

	if(_device->open(openni::ANY_DEVICE) != openni::STATUS_OK)
	{
		UERROR("CameraOpenNI2: Cannot open device.");
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(!_device->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		UERROR("CameraOpenNI2: Device doesn't support depth/color registration.");
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_device->getSensorInfo(openni::SENSOR_DEPTH) == NULL ||
	  _device->getSensorInfo(openni::SENSOR_COLOR) == NULL)
	{
		UERROR("CameraOpenNI2: Cannot get sensor info for depth and color.");
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_depth->create(*_device, openni::SENSOR_DEPTH) != openni::STATUS_OK)
	{
		UERROR("CameraOpenNI2: Cannot create depth stream.");
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_color->create(*_device, openni::SENSOR_COLOR) != openni::STATUS_OK)
	{
		UERROR("CameraOpenNI2: Cannot create color stream.");
		_depth->destroy();
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_device->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) != openni::STATUS_OK)
	{
		UERROR("CameraOpenNI2: Failed to set depth/color registration.");
	}

	_depth->setMirroringEnabled(false);
	_color->setMirroringEnabled(false);

	const openni::Array<openni::VideoMode>& depthVideoModes = _depth->getSensorInfo().getSupportedVideoModes();
	for(int i=0; i<depthVideoModes.getSize(); ++i)
	{
		UINFO("CameraOpenNI2: Depth video mode %d: fps=%d, pixel=%d, w=%d, h=%d",
				i,
				depthVideoModes[i].getFps(),
				depthVideoModes[i].getPixelFormat(),
				depthVideoModes[i].getResolutionX(),
				depthVideoModes[i].getResolutionY());
	}

	const openni::Array<openni::VideoMode>& colorVideoModes = _color->getSensorInfo().getSupportedVideoModes();
	for(int i=0; i<colorVideoModes.getSize(); ++i)
	{
		UINFO("CameraOpenNI2: Color video mode %d: fps=%d, pixel=%d, w=%d, h=%d",
				i,
				colorVideoModes[i].getFps(),
				colorVideoModes[i].getPixelFormat(),
				colorVideoModes[i].getResolutionX(),
				colorVideoModes[i].getResolutionY());
	}

	openni::VideoMode mMode;
	mMode.setFps(30);
	mMode.setResolution(640,480);
	mMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	_depth->setVideoMode(mMode);

	openni::VideoMode mModeColor;
	mModeColor.setFps(30);
	mModeColor.setResolution(640,480);
	mModeColor.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	_color->setVideoMode(mModeColor);

	UINFO("CameraOpenNI2: Using depth video mode: fps=%d, pixel=%d, w=%d, h=%d, H-FOV=%f rad, V-FOV=%f rad",
			_depth->getVideoMode().getFps(),
			_depth->getVideoMode().getPixelFormat(),
			_depth->getVideoMode().getResolutionX(),
			_depth->getVideoMode().getResolutionY(),
			_depth->getHorizontalFieldOfView(),
			_depth->getVerticalFieldOfView());

	bool registered = true;
	if(registered)
	{
		_depthFocal = float(_color->getVideoMode().getResolutionX()/2) / std::tan(_color->getHorizontalFieldOfView()/2.0f);
	}
	else
	{
		_depthFocal = float(_depth->getVideoMode().getResolutionX()/2) / std::tan(_depth->getHorizontalFieldOfView()/2.0f);
	}
	UINFO("depth focal = %f", _depthFocal);

	UINFO("CameraOpenNI2: Using color video mode: fps=%d, pixel=%d, w=%d, h=%d, H-FOV=%f rad, V-FOV=%f rad",
			_color->getVideoMode().getFps(),
			_color->getVideoMode().getPixelFormat(),
			_color->getVideoMode().getResolutionX(),
			_color->getVideoMode().getResolutionY(),
			_color->getHorizontalFieldOfView(),
			_color->getVerticalFieldOfView());

	if(_depth->start() != openni::STATUS_OK ||
	   _color->start() != openni::STATUS_OK)
	{
		UERROR("CameraOpenNI2: Cannot start depth and/or color streams.");
		_depth->stop();
		_color->stop();
		_depth->destroy();
		_color->destroy();
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	uSleep(1000); // just to make sure the sensor is correctly initialized

	return true;
}

void CameraOpenNI2::captureImage(cv::Mat & rgb, cv::Mat & depth, float & depthConstant)
{
	if(_device->isValid() &&
		_depth->isValid() &&
		_color->isValid() &&
		_device->getSensorInfo(openni::SENSOR_DEPTH) != NULL &&
		_device->getSensorInfo(openni::SENSOR_COLOR) != NULL)
	{
		openni::VideoFrameRef depthFrame, colorFrame;

		_depth->readFrame(&depthFrame);
		_color->readFrame(&colorFrame);

		if(depthFrame.isValid() && colorFrame.isValid())
		{
			int h=depthFrame.getHeight();
			int w=depthFrame.getWidth();
			depth = cv::Mat(h, w, CV_16U, (void*)depthFrame.getData()).clone();

			h=colorFrame.getHeight();
			w=colorFrame.getWidth();
			cv::Mat tmp(h, w, CV_8UC3, (void *)colorFrame.getData());
			cv::cvtColor(tmp, rgb, CV_RGB2BGR);
		}
		UASSERT(_depthFocal != 0.0f);
		depthConstant = 1.0f/_depthFocal;
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}
}

} // namespace rtabmap
