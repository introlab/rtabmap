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
#include "rtabmap/core/Features2d.h"

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>

#include <opencv2/imgproc/imgproc.hpp>

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
	_frameRateTimer(new UTimer()),
	_featuresExtracted(false),
	_keypointDetector(0),
	_keypointDescriptor(0)
{
}

Camera::~Camera()
{
	if(_keypointDetector)
	{
		delete _keypointDetector;
	}
	if(_keypointDescriptor)
	{
		delete _keypointDescriptor;
	}
	if(_frameRateTimer)
	{
		delete _frameRateTimer;
	}
}

void Camera::setFeaturesExtracted(bool featuresExtracted, KeypointDetector::DetectorType detector, KeypointDescriptor::DescriptorType descriptor)
{
	_featuresExtracted = featuresExtracted;
	if(detector != KeypointDetector::kDetectorUndef || descriptor != KeypointDescriptor::kDescriptorUndef)
	{
		ParametersMap pm;
		pm.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str((int)detector)));
		this->parseParameters(pm);
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

void Camera::parseParameters(const ParametersMap & parameters)
{
	UDEBUG("");
	ParametersMap::const_iterator iter;
	//Keypoint detector
	KeypointDetector::DetectorType detector = KeypointDetector::kDetectorUndef;
	if((iter=parameters.find(Parameters::kKpDetectorStrategy())) != parameters.end())
	{
		detector = (KeypointDetector::DetectorType)std::atoi((*iter).second.c_str());
	}

	if(detector!=KeypointDetector::kDetectorUndef)
	{
		ULOGGER_DEBUG("new detector strategy %d", int(detector));
		if(_keypointDetector)
		{
			delete _keypointDetector;
			_keypointDetector = 0;
		}
		if(_keypointDescriptor)
		{
			delete _keypointDescriptor;
			_keypointDescriptor = 0;
		}
		switch(detector)
		{
		case KeypointDetector::kDetectorSift:
			_keypointDetector = new SIFTDetector(parameters);
			_keypointDescriptor = new SIFTDescriptor(parameters);
			break;
		case KeypointDetector::kDetectorSurf:
		default:
			_keypointDetector = new SURFDetector(parameters);
			_keypointDescriptor = new SURFDescriptor(parameters);
			break;
		}
	}
	else
	{
		if(_keypointDetector)
		{
			_keypointDetector->parseParameters(parameters);
		}
		if(_keypointDescriptor)
		{
			_keypointDescriptor->parseParameters(parameters);
		}
	}
}

cv::Mat Camera::takeImage()
{
	cv::Mat descriptors;
	std::vector<cv::KeyPoint> keypoints;
	bool tmp = _featuresExtracted;
	_featuresExtracted = false; // no need to extract descriptors/keypoints in this function
	cv::Mat img = takeImage(descriptors, keypoints);
	_featuresExtracted = tmp;
	return img;
}

cv::Mat Camera::takeImage(cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
{
	descriptors = cv::Mat();
	keypoints.clear();
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

	cv::Mat img;
	UTimer timer;
	img = this->captureImage();
	UDEBUG("Time capturing image = %fs", timer.ticks());

	if(!img.empty())
	{
		if(img.depth() != CV_8U)
		{
			UWARN("Images should have already 8U depth !?");
			cv::Mat tmp = img;
			img = cv::Mat();
			tmp.convertTo(img, CV_8U);
			UDEBUG("Time converting image to 8U = %fs", timer.ticks());
		}

		if(_featuresExtracted && _keypointDetector && _keypointDescriptor)
		{
			keypoints = _keypointDetector->generateKeypoints(img);
			descriptors = _keypointDescriptor->generateDescriptors(img, keypoints);
			UDEBUG("Post treatment time = %fs", timer.ticks());
		}

		if(_framesDropped)
		{
			unsigned int count = 0;
			while(count++ < _framesDropped)
			{
				cv::Mat tmp = this->captureImage();
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
	return img;
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

cv::Mat CameraImages::captureImage()
{
	UDEBUG("");
	cv::Mat img;
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
#if CV_MAJOR_VERSION >=2 and CV_MINOR_VERSION >=4
					img = cv::imread(fullPath.c_str(), cv::IMREAD_UNCHANGED);
#else
					img = cv::imread(fullPath.c_str(), -1);
#endif
					UDEBUG("width=%d, height=%d, channels=%d, elementSize=%d, total=%d", img.cols, img.rows, img.channels(), img.elemSize(), img.total());

					// FIXME : it seems that some png are incorrectly loaded with opencv c++ interface, where c interface works...
					if(img.depth() != CV_8U)
					{
						// The depth should be 8U
						UWARN("Cannot read the image correctly, falling back to old OpenCV C interface...");
						IplImage * i = cvLoadImage(fullPath.c_str());
						img = cv::Mat(i, true);
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

	if(!img.empty() &&
	   w &&
	   h &&
	   w != (unsigned int)img.cols &&
	   h != (unsigned int)img.rows)
	{
		cv::Mat resampled;
		cv::resize(img, resampled, cv::Size(w, h));
		img = resampled;
	}

	return img;
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

cv::Mat CameraVideo::captureImage()
{
	cv::Mat img;  // Null image
	if(_capture.isOpened())
	{
		if(!_capture.read(img))
		{
			if(_usbDevice)
			{
				UERROR("Camera has been disconnected!");
			}
		}
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}

	unsigned int w;
	unsigned int h;
	this->getImageSize(w, h);

	if(!img.empty() &&
	   w &&
	   h &&
	   w != (unsigned int)img.cols &&
	   h != (unsigned int)img.rows)
	{
		cv::Mat resampled;
		cv::resize(img, resampled, cv::Size(w, h));
		return resampled;
	}
	else
	{
		// clone required
		return img.clone();
	}
}

} // namespace rtabmap
