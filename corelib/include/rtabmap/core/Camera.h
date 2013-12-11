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
#include <opencv2/features2d/features2d.hpp>
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/Image.h"
#include "rtabmap/core/Features2d.h"
#include <set>
#include <stack>
#include <list>
#include <vector>

class UDirectory;
class UTimer;

namespace rtabmap
{

class KeypointDetector;
class KeypointDescriptor;

/**
 * Class Camera
 *
 */
class RTABMAP_EXP Camera
{
public:
	virtual ~Camera();
	cv::Mat takeImage();
	cv::Mat takeImage(cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);
	virtual bool init() = 0;

	//getters
	void getImageSize(unsigned int & width, unsigned int & height);
	float getImageRate() const {return _imageRate;}
	bool isFeaturesExtracted() const {return _featuresExtracted;}

	//setters
	void setFeaturesExtracted(bool featuresExtracted,
			KeypointDetector::DetectorType detector = KeypointDetector::kDetectorUndef,
			KeypointDescriptor::DescriptorType descriptor = KeypointDescriptor::kDescriptorUndef);
	void setImageRate(float imageRate) {_imageRate = imageRate;}
	void setImageSize(unsigned int width, unsigned int height);


	virtual void parseParameters(const ParametersMap & parameters);

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

	virtual cv::Mat captureImage() = 0;

private:
	float _imageRate;
	unsigned int _imageWidth;
	unsigned int _imageHeight;
	unsigned int _framesDropped;
	UTimer * _frameRateTimer;

	bool _featuresExtracted;
	KeypointDetector * _keypointDetector;
	KeypointDescriptor * _keypointDescriptor;
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
