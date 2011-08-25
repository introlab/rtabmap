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
#include "rtabmap/core/CameraEvent.h"
#include "utilite/UEventsManager.h"
#include "utilite/UConversion.h"
#include "rtabmap/core/DBDriver.h"
#include "rtabmap/core/DBDriverFactory.h"
#include "rtabmap/core/KeypointDescriptor.h"
#include "rtabmap/core/KeypointDetector.h"
#include "rtabmap/core/SMState.h"
#include "utilite/UStl.h"
#include "utilite/UConversion.h"
#include "utilite/UFile.h"
#include "utilite/UDirectory.h"
#include "utilite/UTimer.h"
#include <opencv2/imgproc/imgproc_c.h>

namespace rtabmap
{

void CamPostTreatment::process(SMState * smState) const
{
	//no threatment...
}

CamKeypointTreatment::~CamKeypointTreatment()
{
	if(_keypointDetector)
	{
		delete _keypointDetector;
	}
	if(_keypointDescriptor)
	{
		delete _keypointDescriptor;
	}
}
void CamKeypointTreatment::process(SMState * smState) const
{
	if(smState && smState->getImage() && smState->getKeypoints().size() == 0 && smState->getSensors().size() == 0)
	{
		std::list<cv::KeyPoint> keypoints = _keypointDetector->generateKeypoints(smState->getImage());
		std::list<std::vector<float> > descriptors = _keypointDescriptor->generateDescriptors(smState->getImage(), keypoints);
		smState->setSensors(descriptors);
		smState->setKeypoints(keypoints);
	}
}

void CamKeypointTreatment::parseParameters(const ParametersMap & parameters)
{
	UDEBUG("");
	ParametersMap::const_iterator iter;
	//Keypoint detector
	DetectorStrategy detectorStrategy = kDetectorUndef;
	if((iter=parameters.find(Parameters::kKpDetectorStrategy())) != parameters.end())
	{
		detectorStrategy = (DetectorStrategy)std::atoi((*iter).second.c_str());
	}
	DetectorStrategy currentDetectorStrategy = this->detectorStrategy();
	if(!_keypointDetector || ( detectorStrategy!=kDetectorUndef && (detectorStrategy != currentDetectorStrategy) ) )
	{
		ULOGGER_DEBUG("new detector strategy %d", int(detectorStrategy));
		if(_keypointDetector)
		{
			delete _keypointDetector;
			_keypointDetector = 0;
		}
		switch(detectorStrategy)
		{
		case kDetectorStar:
			_keypointDetector = new StarDetector(parameters);
			break;
		case kDetectorSift:
			_keypointDetector = new SIFTDetector(parameters);
			break;
		case kDetectorSurf:
		default:
			_keypointDetector = new SURFDetector(parameters);
			break;
		}
	}
	else if(_keypointDetector)
	{
		_keypointDetector->parseParameters(parameters);
	}

	//Keypoint descriptor
	DescriptorStrategy descriptorStrategy = kDescriptorUndef;
	if((iter=parameters.find(Parameters::kKpDescriptorStrategy())) != parameters.end())
	{
		descriptorStrategy = (DescriptorStrategy)std::atoi((*iter).second.c_str());
	}
	if(!_keypointDescriptor || descriptorStrategy!=kDescriptorUndef)
	{
		ULOGGER_DEBUG("new descriptor strategy %d", int(descriptorStrategy));
		if(_keypointDescriptor)
		{
			delete _keypointDescriptor;
			_keypointDescriptor = 0;
		}
		switch(descriptorStrategy)
		{
		case kDescriptorColorSurf:
			// see decorator pattern...
			_keypointDescriptor = new ColorDescriptor(parameters, new SURFDescriptor(parameters));
			break;
		case kDescriptorLaplacianSurf:
			// see decorator pattern...
			_keypointDescriptor = new LaplacianDescriptor(parameters, new SURFDescriptor(parameters));
			break;
		case kDescriptorSift:
			_keypointDescriptor = new SIFTDescriptor(parameters);
			break;
		case kDescriptorHueSurf:
			// see decorator pattern...
			_keypointDescriptor = new HueDescriptor(parameters, new SURFDescriptor(parameters));
			break;
		case kDescriptorSurf:
		default:
			_keypointDescriptor = new SURFDescriptor(parameters);
			break;
		}
	}
	else if(_keypointDescriptor)
	{
		_keypointDescriptor->parseParameters(parameters);
	}
	CamPostTreatment::parseParameters(parameters);
}

CamKeypointTreatment::DetectorStrategy CamKeypointTreatment::detectorStrategy() const
{
	DetectorStrategy strategy = kDetectorUndef;
	StarDetector * star = dynamic_cast<StarDetector*>(_keypointDetector);
	SURFDetector * surf = dynamic_cast<SURFDetector*>(_keypointDetector);
	if(star)
	{
		strategy = kDetectorStar;
	}
	else if(surf)
	{
		strategy = kDetectorSurf;
	}
	return strategy;
}

Camera::Camera(float imageRate,
		bool autoRestart,
		unsigned int imageWidth,
		unsigned int imageHeight) :
	_imageRate(imageRate),
	_autoRestart(autoRestart),
	_imageWidth(imageWidth),
	_imageHeight(imageHeight)
{
	_postThreatement = new CamPostTreatment();
	UEventsManager::addHandler(this);
}

Camera::~Camera(void)
{
	this->kill();
	delete _postThreatement;
}

void Camera::mainLoop()
{
	State state = kStateCapturing;
	ParametersMap parameters;

	_stateMutex.lock();
	{
		if(!_state.empty() && !_stateParam.empty())
		{
			state = _state.top();
			_state.pop();
			parameters = _stateParam.top();
			_stateParam.pop();
		}
	}
	_stateMutex.unlock();

	if(state == kStateCapturing)
	{
		process();
	}
	else if(state == kStateChangingParameters)
	{
		this->parseParameters(parameters);
	}
}

// ownership is transferred
void Camera::setPostThreatement(CamPostTreatment * strategy)
{
	if(strategy)
	{
		delete _postThreatement;
		_postThreatement = strategy;
	}
}

void Camera::pushNewState(State newState, const ParametersMap & parameters)
{
	ULOGGER_DEBUG("to %d", newState);

	_stateMutex.lock();
	{
		_state.push(newState);
		_stateParam.push(parameters);
	}
	_stateMutex.unlock();
}

void Camera::handleEvent(UEvent* anEvent)
{
	if(anEvent->getClassName().compare("CameraEvent") == 0)
	{
		CameraEvent * cameraEvent = (CameraEvent*)anEvent;
		if(cameraEvent->getCode() == CameraEvent::kCodeCtrl)
		{
			CameraEvent::Cmd cmd = cameraEvent->getCommand();

			if(cmd == CameraEvent::kCmdPause)
			{
				if(this->isRunning())
				{
					this->kill();
				}
				else
				{
					this->start();
				}
			}
			else if(cmd == CameraEvent::kCmdChangeParam)
			{
				// TODO : Put in global Parameters ?
				_imageRate = cameraEvent->getImageRate();
				_autoRestart = cameraEvent->getAutoRestart();
			}
			else
			{
				ULOGGER_DEBUG("Camera::handleEvent(Util::Event* anEvent) : command undefined...");
			}
		}
	}
	if(anEvent->getClassName().compare("ParamEvent") == 0)
	{
		if(this->isIdle())
		{
			_stateMutex.lock();
			this->parseParameters(((ParamEvent*)anEvent)->getParameters());
			_stateMutex.unlock();
		}
		else
		{
			ULOGGER_DEBUG("changing parameters");
			pushNewState(kStateChangingParameters, ((ParamEvent*)anEvent)->getParameters());
		}
	}
}

void Camera::process()
{
	UTimer timer;
	ULOGGER_DEBUG("Camera::process()");
	SMState * smState = this->takeImage();
	if(smState)
	{
		_postThreatement->process(smState);
		this->post(new SMStateEvent(smState));

		double elapsed = timer.ticks();
		UDEBUG("Post treatment time = %fs", elapsed);
		if(_imageRate>0)
		{
			float sleepTime = 1000.0f/_imageRate - 1000.0f*elapsed;
			if(sleepTime > 0)
			{
				UDEBUG("Now sleeping for = %fms", sleepTime);
				uSleep(sleepTime);
			}
		}
	}
	else
	{
		if(_autoRestart)
		{
			this->init();
		}
		else
		{
			ULOGGER_DEBUG("Camera::process() : no more images...");
			this->kill();
			this->post(new CameraEvent());
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
					 bool autoRestart,
					 unsigned int imageWidth,
					 unsigned int imageHeight) :
	Camera(imageRate, autoRestart, imageWidth, imageHeight),
	_path(path),
	_startAt(startAt),
	_refreshDir(refreshDir),
	_dir(0),
	_count(0)
{
}

CameraImages::~CameraImages(void)
{
	this->kill();
	if(_dir)
	{
		delete _dir;
		_dir = 0;
	}
}

bool CameraImages::init()
{
	if(_dir)
	{
		delete _dir;
		_dir = 0;
	}
	_dir = new UDirectory(_path, "jpg ppm png bmp pnm");
	_count = 0;
	if(_path[_path.size()-1] != '\\' && _path[_path.size()-1] != '/')
	{
		_path.append("/");
	}
	if(!_dir)
	{
		ULOGGER_ERROR("Directory path not valid \"%s\"", _path.c_str());
	}
	return _dir != 0;
}

SMState * CameraImages::takeImage()
{
	IplImage * img = 0;
	if(_dir)
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
					img = cvLoadImage(fullPath.c_str(), CV_LOAD_IMAGE_UNCHANGED);
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
					ULOGGER_DEBUG("Loading image : %s\n", fullPath.c_str());
					img = cvLoadImage(fullPath.c_str(), CV_LOAD_IMAGE_UNCHANGED);
				}
			}
		}
	}
	if(img &&
	   getImageWidth() &&
	   getImageHeight() &&
	   getImageWidth() != (unsigned int)img->width &&
	   getImageHeight() != (unsigned int)img->height)
	{
		// declare a destination IplImage object with correct size, depth and channels
		IplImage * resampledImg = cvCreateImage( cvSize((int)(getImageWidth()) ,
											   (int)(getImageHeight()) ),
											   img->depth, img->nChannels );

		//use cvResize to resize source to a destination image (linear interpolation)
		cvResize(img, resampledImg);
		cvReleaseImage(&img);
		img = resampledImg;
	}
	if(img)
	{
		return new SMState(img);
	}
	return 0;
}



/////////////////////////
// CameraVideo
/////////////////////////
CameraVideo::CameraVideo(int usbDevice,
						 float imageRate,
						 bool autoRestart,
						 unsigned int imageWidth,
						 unsigned int imageHeight) :
	Camera(imageRate, autoRestart, imageWidth, imageHeight),
	_capture(0),
	_src(kUsbDevice),
	_usbDevice(usbDevice)
{

}

CameraVideo::CameraVideo(const std::string & fileName,
						   float imageRate,
						   bool autoRestart,
						   unsigned int imageWidth,
						   unsigned int imageHeight) :
	Camera(imageRate, autoRestart, imageWidth, imageHeight),
	_fileName(fileName),
	_capture(0),
	_src(kVideoFile)
{
}

CameraVideo::~CameraVideo()
{
	this->kill();
	if(_capture)
	{
		cvReleaseCapture(&_capture);
	}
}

bool CameraVideo::init()
{
	if(_capture)
	{
		cvReleaseCapture(&_capture);
		_capture = 0;
	}

	if(_src == kUsbDevice)
	{
		ULOGGER_DEBUG("CameraVideo::init() Usb device initialization on device %d with imgSize=[%d,%d]", _usbDevice, getImageWidth(), getImageHeight());
		_capture = cvCaptureFromCAM(_usbDevice);
		if(_capture && getImageWidth() && getImageHeight())
		{
			cvSetCaptureProperty(_capture, CV_CAP_PROP_FRAME_WIDTH, double(getImageWidth()));
			cvSetCaptureProperty(_capture, CV_CAP_PROP_FRAME_HEIGHT, double(getImageHeight()));
		}
	}
	else if(_src == kVideoFile)
	{
		ULOGGER_DEBUG("CameraVideo::init() filename=\"%s\"", _fileName.c_str());
		_capture = cvCaptureFromAVI(_fileName.c_str());
	}
	else
	{
		ULOGGER_ERROR("CameraVideo::init() Unknown source...");
	}
	if(!_capture)
	{
		ULOGGER_ERROR("CameraVideo::init() Failed to create a capture object!");
		return false;
	}
	return true;
}

SMState * CameraVideo::takeImage()
{
	IplImage * img = 0;  // Null image
	if(_capture)
	{
		if(!cvGrabFrame(_capture)){              // capture a frame
			ULOGGER_WARN("CameraVideo: Could not grab a frame, the end of the feed may be reached...");
		}
		else
		{
			img=cvRetrieveFrame(_capture);           // retrieve the captured frame
		}
	}
	else
	{
		ULOGGER_WARN("CameraVideo::takeImage() The camera must be initialized before requesting an image.");
	}

	if(img &&
	   getImageWidth() &&
	   getImageHeight() &&
	   getImageWidth() != (unsigned int)img->width &&
	   getImageHeight() != (unsigned int)img->height)
	{
		// declare a destination IplImage object with correct size, depth and channels
		IplImage * resampledImg = cvCreateImage( cvSize((int)(getImageWidth()) ,
											   (int)(getImageHeight()) ),
											   img->depth, img->nChannels );

		//use cvResize to resize source to a destination image (linear interpolation)
		cvResize(img, resampledImg);
		img = resampledImg;
	}
	else if(img)
	{
		img = cvCloneImage(img);
	}

	if(img)
	{
		return new SMState(img);
	}
	return 0;
}







/////////////////////////
// CameraDatabase
/////////////////////////
CameraDatabase::CameraDatabase(const std::string & path,
								bool ignoreChildren,
								float imageRate,
								bool autoRestart,
								unsigned int imageWidth,
								unsigned int imageHeight) :
	Camera(imageRate, autoRestart, imageWidth, imageHeight),
	_path(path),
	_ignoreChildren(ignoreChildren),
	_indexIter(_ids.begin()),
	_dbDriver(0)
{
}

CameraDatabase::~CameraDatabase(void)
{
	this->kill();
	if(_dbDriver)
	{
		_dbDriver->closeConnection();
		delete _dbDriver;
	}
}

bool CameraDatabase::init()
{
	if(_dbDriver)
	{
		_dbDriver->closeConnection();
		delete _dbDriver;
		_dbDriver = 0;
	}
	_ids.clear();
	_indexIter = _ids.begin();

	std::string driverType = "sqlite3";
	ParametersMap parameters;
	parameters.insert(ParametersPair(Parameters::kDbSqlite3InMemory(), "false"));
	_dbDriver = rtabmap::DBDriverFactory::createDBDriver(driverType, parameters);
	if(!_dbDriver)
	{
		ULOGGER_ERROR("CameraDatabase::init() can't create \"%s\" driver",driverType.c_str());
		return false;
	}
	else if(!_dbDriver->openConnection(_path.c_str()))
	{
		ULOGGER_ERROR("CameraDatabase::init() Can't read database \"%s\"",_path.c_str());
		return false;
	}
	else
	{
		_dbDriver->getAllSignatureIds(_ids);
		_indexIter = _ids.begin();
	}
	return true;
}

SMState * CameraDatabase::takeImage()
{
	IplImage * img = 0;
	if(_dbDriver && _indexIter != _ids.end())
	{
		if(_ignoreChildren)
		{
			bool ignore = true;
			while(img == 0 && _indexIter != _ids.end() && ignore)
			{
				ignore = false;
				int loopId = 0;
				if(_dbDriver->getLoopClosureId(*_indexIter, loopId))
				{
					if(loopId == *_indexIter+1)
					{
						ignore = true;
					}
					else
					{
						_dbDriver->getImage(*_indexIter, &img);
					}
				}
				++_indexIter;
			}
		}
		else
		{
			_dbDriver->getImage(*_indexIter, &img);
			++_indexIter;
		}
	}
	else if(!_dbDriver)
	{
		ULOGGER_WARN("The camera must be initialized first...");
	}
	else if(_ids.size() == 0)
	{
		ULOGGER_WARN("The database \"%s\" is empty...", _path.c_str());
	}

	if(img &&
	   getImageWidth() &&
	   getImageHeight() &&
	   getImageWidth() != (unsigned int)img->width &&
	   getImageHeight() != (unsigned int)img->height)
	{
		// declare a destination IplImage object with correct size, depth and channels
		IplImage * resampledImg = cvCreateImage( cvSize((int)(getImageWidth()) ,
											   (int)(getImageHeight()) ),
											   img->depth, img->nChannels );

		//use cvResize to resize source to a destination image (linear interpolation)
		cvResize(img, resampledImg);
		cvReleaseImage(&img);
		img = resampledImg;
	}

	if(img)
	{
		SMState * smState = new SMState(img);
		if(_dbDriver && _indexIter!=_ids.begin())
		{
			std::set<int>::iterator iter = _indexIter;
			--iter;
			std::map<int, std::list<std::vector<float> > > neighbors;
			_dbDriver->loadNeighbors(*iter, neighbors);
			for(std::map<int, std::list<std::vector<float> > >::iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
			{
				if(i->first > *iter && i->second.size())
				{
					smState->setActuators(i->second);
					break;
				}
			}
		}
		return smState;
	}
	return 0;
}

} // namespace rtabmap
