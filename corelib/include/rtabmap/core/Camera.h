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

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include "rtabmap/core/SensorData.h"
#include <set>
#include <stack>
#include <list>
#include <vector>

class UDirectory;
class UTimer;

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
	cv::Mat takeImage();
	virtual bool init() = 0;

	//getters
	void getImageSize(unsigned int & width, unsigned int & height);
	float getImageRate() const {return _imageRate;}

	//setters
	void setImageRate(float imageRate) {_imageRate = imageRate;}
	void setImageSize(unsigned int width, unsigned int height);

protected:
	/**
	 * Constructor
	 *
	 * @param imageRate : image/second , 0 for fast as the camera can
	 */
	Camera(float imageRate = 0,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0);

	virtual cv::Mat captureImage() = 0;

private:
	float _imageRate;
	unsigned int _imageWidth;
	unsigned int _imageHeight;
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
			unsigned int imageHeight = 0);
	virtual ~CameraImages();

	virtual bool init();
	std::string getPath() const {return _path;}

protected:
	virtual cv::Mat captureImage();

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
			unsigned int imageHeight = 0);
	CameraVideo(const std::string & filePath,
			float imageRate = 0,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0);
	virtual ~CameraVideo();

	virtual bool init();
	int getUsbDevice() const {return _usbDevice;}
	const std::string & getFilePath() const {return _filePath;}

protected:
	virtual cv::Mat captureImage();

private:
	// File type
	std::string _filePath;

	cv::VideoCapture _capture;
	Source _src;

	// Usb camera
	int _usbDevice;
};


} // namespace rtabmap
