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
#include "utilite/UThreadNode.h"
#include "utilite/UEventsHandler.h"
#include "rtabmap/core/Parameters.h"
#include <set>
#include <stack>
#include <list>
#include <vector>

class UDirectory;

namespace rtabmap
{

class KeypointDetector;
class KeypointDescriptor;
class SMState;

/**
 * No treatment
 */
class RTABMAP_EXP CamPostTreatment
{
public:
	CamPostTreatment(const ParametersMap & parameters = ParametersMap()) {
		this->parseParameters(parameters);
	}
	virtual ~CamPostTreatment() {}
	virtual void process(SMState * smState) const;
	virtual void parseParameters(const ParametersMap & parameters) {}
};

/**
 * Extract keypoints from the image
 */
class RTABMAP_EXP CamKeypointTreatment : public CamPostTreatment
{
public:
	enum DetectorStrategy {kDetectorSurf, kDetectorStar, kDetectorSift, kDetectorFast, kDetectorUndef};
	enum DescriptorStrategy {kDescriptorSurf, kDescriptorSift, kDescriptorBrief, kDescriptorColor, kDescriptorHue, kDescriptorUndef};

public:
	CamKeypointTreatment(const ParametersMap & parameters = ParametersMap()) :
		_keypointDetector(0),
		_keypointDescriptor(0)
	{
		this->parseParameters(parameters);
	}
	virtual ~CamKeypointTreatment();
	virtual void process(SMState * smState) const;
	virtual void parseParameters(const ParametersMap & parameters);
	DetectorStrategy detectorStrategy() const;
private:
	KeypointDetector * _keypointDetector;
	KeypointDescriptor * _keypointDescriptor;
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

	virtual IplImage * takeImage(std::list<std::vector<float> > * actions = 0) = 0;
	SMState * takeSMState();
	virtual bool init() = 0;
	bool isPaused() const {return !this->isRunning();}
	bool isCapturing() const {return this->isRunning();}
	unsigned int getImageWidth() const {return _imageWidth;}
	unsigned int getImageHeight() const {return _imageHeight;}
	void setPostThreatement(CamPostTreatment * strategy); // ownership is transferred
	void setImageRate(float imageRate) {_imageRate = imageRate;}
	void setAutoRestart(bool autoRestart) {_autoRestart = autoRestart;}

protected:
	/**
	 * Constructor
	 *
	 * @param imageRate : image/second , 0 for fast as the camera can
	 */
	Camera(float imageRate = 0, bool autoRestart = false, unsigned int imageWidth = 0, unsigned int imageHeight = 0);

	virtual void handleEvent(UEvent* anEvent);

private:
	virtual void mainLoop();
	void process();
	void pushNewState(State newState, const ParametersMap & parameters = ParametersMap());
	virtual void parseParameters(const ParametersMap & parameters) {_postThreatement->parseParameters(parameters);}

private:
	float _imageRate;
	int _id;
	bool _autoRestart;
	unsigned int _imageWidth;
	unsigned int _imageHeight;
	CamPostTreatment * _postThreatement;

	UMutex _stateMutex;
	std::stack<State> _state;
	std::stack<ParametersMap> _stateParam;
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
			unsigned int imageHeight = 0);
	virtual ~CameraImages();

	virtual IplImage * takeImage(std::list<std::vector<float> > * actions = 0);
	virtual bool init();

private:
	std::string _path;
	int _startAt;
	// If the list of files in the directory is refreshed
	// on each call of takeImage()
	bool _refreshDir;
	UDirectory * _dir;
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
			unsigned int imageHeight = 0);
	CameraVideo(const std::string & fileName,
			float imageRate = 0,
			bool autoRestart = false,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0);
	virtual ~CameraVideo();

	virtual IplImage * takeImage(std::list<std::vector<float> > * actions = 0);
	virtual bool init();

private:
	// File type
	std::string _fileName;

	CvCapture* _capture;
	Source _src;

	// Usb camera
	int _usbDevice;
};






/////////////////////////
// CameraDatabase
/////////////////////////
class DBDriver;
class RTABMAP_EXP CameraDatabase :
	public Camera
{
public:
	CameraDatabase(const std::string & path,
			bool loadActions,
			float imageRate = 0,
			bool autoRestart = false,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0);
	virtual ~CameraDatabase();

	virtual IplImage * takeImage(std::list<std::vector<float> > * actions = 0);
	virtual bool init();

private:
	std::string _path;
	bool _loadActions;
	std::set<int>::iterator _indexIter;
	DBDriver * _dbDriver;
	std::set<int> _ids;
};


} // namespace rtabmap
