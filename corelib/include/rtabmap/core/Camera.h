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
#include <utilite/UThreadNode.h>
#include <utilite/UEventsHandler.h>
#include <utilite/UEvent.h>
#include <utilite/UDirectory.h>
#include <utilite/UTimer.h>
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/Image.h"
#include <set>
#include <stack>
#include <list>
#include <vector>

namespace rtabmap
{

class CameraEvent :
	public UEvent
{
public:
	enum Code {
		kCodeFeatures,
		kCodeImage,
		kCodeNoMoreImages
	};

public:
	CameraEvent(const cv::Mat & image, int cameraId = 0) :
		UEvent(kCodeImage),
		_cameraId(cameraId),
		_image(image)
	{
	}
	CameraEvent(const cv::Mat & descriptors, const std::vector<cv::KeyPoint> & keypoints, const cv::Mat & image = cv::Mat(), int cameraId = 0) :
		UEvent(kCodeFeatures),
		_cameraId(cameraId),
		_image(image, 0, descriptors, keypoints)
	{
	}
	CameraEvent(int cameraId = 0) :
		UEvent(kCodeNoMoreImages),
		_cameraId(cameraId)
	{
	}

	int cameraId() const {return _cameraId;}

	// Image or descriptors
	const Image & image() const {return _image;}

	virtual ~CameraEvent() {}
	virtual std::string getClassName() const {return std::string("CameraEvent");}

private:
	int _cameraId;
	Image _image;
};

/**
 * Class Camera
 *
 */
class RTABMAP_EXP Camera :
	public UThreadNode,
	public UEventsHandler
{
public:
	enum State {kStateCapturing, kStateChangingParameters};

public:
	virtual ~Camera();
	cv::Mat takeImage();
	cv::Mat takeImage(cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);
	virtual bool init() = 0;

	//getters
	bool isPaused() const {return !this->isRunning();}
	bool isCapturing() const {return this->isRunning();}
	void getImageSize(unsigned int & width, unsigned int & height);
	float getImageRate() const {return _imageRate;}
	bool isFeaturesExtracted() const {return _featuresExtracted;}

	//setters
	void setFeaturesExtracted(bool featuresExtracted,
			KeypointDetector::DetectorType detector = KeypointDetector::kDetectorUndef,
			KeypointDescriptor::DescriptorType descriptor = KeypointDescriptor::kDescriptorUndef);
	void setImageRate(float imageRate) {_imageRate = imageRate;}
	void setAutoRestart(bool autoRestart) {_autoRestart = autoRestart;}
	void setImageSize(unsigned int width, unsigned int height);


	virtual void parseParameters(const ParametersMap & parameters);
	int id() const {return _id;}

protected:
	/**
	 * Constructor
	 *
	 * @param imageRate : image/second , 0 for fast as the camera can
	 */
	Camera(float imageRate = 0, bool autoRestart = false, unsigned int imageWidth = 0, unsigned int imageHeight = 0, unsigned int framesDropped = 0, int id = 0);

	virtual void handleEvent(UEvent* anEvent);
	virtual cv::Mat captureImage() = 0;

private:
	virtual void mainLoopBegin();
	virtual void mainLoop();
	void process();
	void pushNewState(State newState, const ParametersMap & parameters = ParametersMap());

private:
	float _imageRate;
	int _id;
	bool _autoRestart;
	unsigned int _imageWidth;
	unsigned int _imageHeight;
	unsigned int _framesDropped;
	UTimer _frameRateTimer;
	UMutex _imageSizeMutex;

	UMutex _stateMutex;
	std::stack<State> _state;
	std::stack<ParametersMap> _stateParam;

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
			bool autoRestart = false,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0,
			unsigned int framesDropped = 0,
			int id = 0);
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
	UDirectory _dir;
	int _count;
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
			bool autoRestart = false,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0,
			unsigned int framesDropped = 0,
			int id = 0);
	CameraVideo(const std::string & filePath,
			float imageRate = 0,
			bool autoRestart = false,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0,
			unsigned int framesDropped = 0,
			int id = 0);
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
