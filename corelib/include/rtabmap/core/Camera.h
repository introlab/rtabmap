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

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include "rtabmap/core/Image.h"
#include <set>
#include <stack>
#include <list>
#include <vector>

class UDirectory;
class UTimer;

namespace openni
{
class Device;
class VideoStream;
}

namespace rtabmap
{

/**
 * Class Camera
 *
 */
class RTABMAP_EXP Camera
{
public:
	virtual ~Camera();
	cv::Mat takeImage(); // backward compatibility
	void takeImage(cv::Mat & rgb);
	void takeImage(cv::Mat & rgb, cv::Mat & depth, float & depthConstant);
	virtual bool init() = 0;

	//getters
	void getImageSize(unsigned int & width, unsigned int & height);
	float getImageRate() const {return _imageRate;}
	const Transform & getLocalTransform() const {return _localTransform;}

	//setters
	void setImageRate(float imageRate) {_imageRate = imageRate;}
	void setImageSize(unsigned int width, unsigned int height);
	void setLocalTransform(const Transform & localTransform) {_localTransform= localTransform;}

protected:
	/**
	 * Constructor
	 *
	 * @param imageRate : image/second , 0 for fast as the camera can
	 */
	Camera(float imageRate = 0,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0,
			unsigned int framesDropped = 0);

	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & depthConstant) = 0;

private:
	float _imageRate;
	unsigned int _imageWidth;
	unsigned int _imageHeight;
	unsigned int _framesDropped;
	Transform _localTransform;
	UTimer * _frameRateTimer;
};


/////////////////////////
// CameraImages
/////////////////////////
class RTABMAP_EXP CameraImages :
	public Camera
{
public:
	CameraImages(const std::string & path,
			int startAt = 1,
			bool refreshDir = false,
			float imageRate = 0,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0,
			unsigned int framesDropped = 0);
	virtual ~CameraImages();

	virtual bool init();
	std::string getPath() const {return _path;}

protected:
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & depthConstant);

private:
	std::string _path;
	int _startAt;
	// If the list of files in the directory is refreshed
	// on each call of takeImage()
	bool _refreshDir;
	int _count;
	UDirectory * _dir;
	std::string _lastFileName;
};




/////////////////////////
// CameraVideo
/////////////////////////
class RTABMAP_EXP CameraVideo :
	public Camera
{
public:
	enum Source{kVideoFile, kUsbDevice};

public:
	CameraVideo(int usbDevice = 0,
			float imageRate = 0,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0,
			unsigned int framesDropped = 0);
	CameraVideo(const std::string & filePath,
			float imageRate = 0,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0,
			unsigned int framesDropped = 0);
	virtual ~CameraVideo();

	virtual bool init();
	int getUsbDevice() const {return _usbDevice;}
	const std::string & getFilePath() const {return _filePath;}

protected:
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & depthConstant);

private:
	// File type
	std::string _filePath;

	cv::VideoCapture _capture;
	Source _src;

	// Usb camera
	int _usbDevice;
};



/////////////////////////
// CameraRGBD
/////////////////////////
class RTABMAP_EXP CameraRGBD :
	public Camera
{

public:
	static bool available();

public:
	CameraRGBD(float imageRate = 0,
				bool asus = false);
	virtual ~CameraRGBD();

	virtual bool init();

protected:
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & depthConstant);

private:
	bool _asus;
	cv::VideoCapture _capture;
	float _depthFocal;
};

/////////////////////////
// CameraOpenNI2
/////////////////////////
class RTABMAP_EXP CameraOpenNI2 :
	public Camera
{

public:
	static bool available();

public:
	CameraOpenNI2(float imageRate = 0);
	virtual ~CameraOpenNI2();

	virtual bool init();

protected:
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & depthConstant);

private:
	openni::Device * _device;
	openni::VideoStream * _color;
	openni::VideoStream * _depth;
	float _depthFocal;
};


} // namespace rtabmap
