/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap/core/SensorEvent.h>
#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/DBDriver.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UEventsManager.h>

#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/Compression.h"

namespace rtabmap {

DBReader::DBReader(const std::string & databasePath,
				   float frameRate,
				   bool odometryIgnored,
				   bool ignoreGoalDelay,
				   bool goalsIgnored,
				   int startId,
				   const std::vector<unsigned int> & cameraIndices,
				   int stopId,
				   bool intermediateNodesIgnored,
				   bool landmarksIgnored,
				   bool featuresIgnored,
				   int startMapId,
				   int stopMapId,
				   bool priorsIgnored,
				   const std::vector<Transform> & cameraLocalTransformOverrides) :
	Camera(frameRate),
	_paths(uSplit(databasePath, ';')),
	_odometryIgnored(odometryIgnored),
	_ignoreGoalDelay(ignoreGoalDelay),
	_goalsIgnored(goalsIgnored),
	_startId(startId),
	_stopId(stopId),
	_cameraIndices(cameraIndices),
	_intermediateNodesIgnored(intermediateNodesIgnored),
	_landmarksIgnored(landmarksIgnored),
	_featuresIgnored(featuresIgnored),
	_priorsIgnored(priorsIgnored),
	_startMapId(startMapId),
	_stopMapId(stopMapId),
	_cameraLocalTransformOverrides(cameraLocalTransformOverrides),
	_dbDriver(0),
	_currentId(_ids.end()),
	_previousMapId(-1),
	_previousStamp(0),
	_previousMapID(0),
	_calibrated(false)
{
	checkArguments();
}

DBReader::DBReader(const std::list<std::string> & databasePaths,
				   float frameRate,
				   bool odometryIgnored,
				   bool ignoreGoalDelay,
				   bool goalsIgnored,
				   int startId,
				   const std::vector<unsigned int> & cameraIndices,
				   int stopId,
				   bool intermediateNodesIgnored,
				   bool landmarksIgnored,
				   bool featuresIgnored,
				   int startMapId,
				   int stopMapId,
				   bool priorsIgnored,
				   const std::vector<Transform> & cameraLocalTransformOverrides) :
	Camera(frameRate),
   _paths(databasePaths),
	_odometryIgnored(odometryIgnored),
	_ignoreGoalDelay(ignoreGoalDelay),
	_goalsIgnored(goalsIgnored),
	_startId(startId),
	_stopId(stopId),
	_cameraIndices(cameraIndices),
	_intermediateNodesIgnored(intermediateNodesIgnored),
	_landmarksIgnored(landmarksIgnored),
	_featuresIgnored(featuresIgnored),
	_priorsIgnored(priorsIgnored),
	_startMapId(startMapId),
	_stopMapId(stopMapId),
	_cameraLocalTransformOverrides(cameraLocalTransformOverrides),
	_dbDriver(0),
	_currentId(_ids.end()),
	_previousMapId(-1),
	_previousStamp(0),
	_previousMapID(0),
	_calibrated(false)
{
	checkArguments();
}

void DBReader::checkArguments()
{
	if(_stopId>0 && _stopId<_startId)
	{
		_stopId = _startId;
	}

	if(_stopMapId>-1 && _stopMapId<_startMapId)
	{
		_stopMapId = _startMapId;
	}

	if(!_cameraLocalTransformOverrides.empty())
	{
		if(!_cameraIndices.empty() &&
			_cameraIndices.size() != _cameraLocalTransformOverrides.size())
		{
			UERROR("Camera local transform overrides (%d) are not the same size than the camera indices (%d). The overrides are ignored.",
				_cameraLocalTransformOverrides.size(),
				_cameraIndices.size()
			);
			_cameraLocalTransformOverrides.clear();
		}
		for(size_t i=0; i<_cameraLocalTransformOverrides.size(); ++i)
		{
			if(_cameraLocalTransformOverrides[i].isNull())
			{
				UERROR("Camera local transform overrides vector cannot contains null transforms! Clearing overrides.");
				_cameraLocalTransformOverrides.clear();
				break;
			}
		}
	}
}

DBReader::~DBReader()
{
	if(_dbDriver)
	{
		_dbDriver->closeConnection();
		delete _dbDriver;
	}
}

bool DBReader::init(
		const std::string &,
		const std::string &)
{
	if(_dbDriver)
	{
		_dbDriver->closeConnection();
		delete _dbDriver;
		_dbDriver = 0;
	}
	_ids.clear();
	_currentId=_ids.end();
	_previousMapId = -1;
	_previousInfMatrix = cv::Mat();
	_previousStamp = 0;
	_previousMapID = 0;
	_calibrated = false;

	if(_paths.size() == 0)
	{
		UERROR("No database path set...");
		return false;
	}

	std::string path = _paths.front();
	if(!UFile::exists(path))
	{
		UERROR("Database path does not exist (%s)", path.c_str());
		return false;
	}

	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kDbSqlite3InMemory(), "false"));
	_dbDriver = DBDriver::create(parameters);
	if(!_dbDriver)
	{
		UERROR("Driver doesn't exist.");
		return false;
	}
	if(!_dbDriver->openConnection(path))
	{
		UERROR("Can't open database %s", path.c_str());
		delete _dbDriver;
		_dbDriver = 0;
		return false;
	}

	_dbDriver->getAllNodeIds(_ids);
	_currentId = _ids.begin();
	if(_startId>0 && _ids.size())
	{
		std::set<int>::iterator iter = _ids.find(_startId);
		if(iter == _ids.end())
		{
			UWARN("Start index is too high (%d), the last ID in database is %d. Starting from beginning...", _startId, *_ids.rbegin());
		}
		else
		{
			_currentId = iter;
		}
	}

	if(_ids.size())
	{
		std::vector<CameraModel> models;
		std::vector<StereoCameraModel> stereoModels;
		if(_dbDriver->getCalibration(*_ids.begin(), models, stereoModels))
		{
			if(models.size())
			{
				if(models.at(0).isValidForProjection())
				{
					_calibrated = true;
				}
				else if(models.at(0).fx() && models.at(0).fy() && models.at(0).imageWidth() == 0)
				{
					// backward compatibility for databases not saving cx,cy and imageSize
					SensorData data;
					_dbDriver->getNodeData(*_ids.begin(), data, true, false, false, false);
					cv::Mat rgb;
					data.uncompressData(&rgb, 0); // this will update camera models if old format
					if(data.cameraModels().size() && data.cameraModels().at(0).isValidForProjection())
					{
						_calibrated = true;
					}
				}
			}
			else if(stereoModels.size() && stereoModels.at(0).isValidForProjection())
			{
				_calibrated = true;
			}
			else
			{
				Signature * s = _dbDriver->loadSignature(*_ids.begin());
				_dbDriver->loadNodeData(s);
				if( s->sensorData().imageCompressed().empty() &&
					s->getWords().empty() &&
					!s->sensorData().laserScanCompressed().empty())
				{
					_calibrated = true; // only scans
				}
				delete s;
			}
		}
	}
	else
	{
		_calibrated = true; // database is empty, make sure calibration warning is not shown.
	}

	_timer.start();

	return true;
}

bool DBReader::isCalibrated() const
{
	return _calibrated;
}

std::string DBReader::getSerial() const
{
	return "DBReader";
}

bool DBReader::getPose(double stamp, Transform & pose, cv::Mat & covariance, double maxWaitTime)
{
	UERROR("DBReader only provides pose when capturing data, it cannot provide asynchronous pose.");
	return false;
}

SensorData DBReader::captureImage(SensorCaptureInfo * info)
{
	SensorData data = this->getNextData(info);
	if(data.id()>0 && _stopId>0 && data.id() > _stopId)
	{
		UINFO("Last ID %d has been reached! Ignoring", _stopId);
		return SensorData();
	}
	if(data.id() == 0)
	{
		UINFO("no more images...");
		while(_paths.size() > 1 && data.id() == 0)
		{
			_paths.pop_front();
			UWARN("Loading next database \"%s\"...", _paths.front().c_str());
			if(!this->init())
			{
				UERROR("Failed to initialize the next database \"%s\"", _paths.front().c_str());
				return data;
			}
			else
			{
				data = this->getNextData(info);
			}
		}
	}

	if(data.id())
	{
		std::string goalId;
		double previousStamp = data.stamp();
		if(previousStamp == 0)
		{
			data.setStamp(UTimer::now());
		}

		if(!_goalsIgnored &&
		   data.userDataRaw().type() == CV_8SC1 &&
		   data.userDataRaw().cols >= 7 && // including null str ending
		   data.userDataRaw().rows == 1 &&
		   memcmp(data.userDataRaw().data, "GOAL:", 5) == 0)
		{
			//GOAL format detected, remove it from the user data and send it as goal event
			std::string goalStr = (const char *)data.userDataRaw().data;
			if(!goalStr.empty())
			{
				std::list<std::string> strs = uSplit(goalStr, ':');
				if(strs.size() == 2)
				{
					goalId = *strs.rbegin();
					data.setUserData(cv::Mat());

					double delay = 0.0;
					if(!_ignoreGoalDelay && _currentId != _ids.end())
					{
						// get stamp for the next signature to compute the delay
						// that was used originally for planning
						int weight;
						std::string label;
						double stamp;
						int mapId;
						Transform localTransform, pose, groundTruth;
						std::vector<float> velocity;
						GPS gps;
						EnvSensors sensors;
						_dbDriver->getNodeInfo(*_currentId, pose, mapId, weight, label, stamp, groundTruth, velocity, gps, sensors);
						if(previousStamp && stamp && stamp > previousStamp)
						{
							delay = stamp - previousStamp;
						}
					}

					if(delay > 0.0)
					{
						UWARN("Goal \"%s\" detected, posting it! Waiting %f seconds before sending next data...",
							   goalId.c_str(), delay);
					}
					else
					{
						UWARN("Goal \"%s\" detected, posting it!", goalId.c_str());
					}

					if(uIsInteger(goalId))
					{
						UEventsManager::post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGoal, atoi(goalId.c_str())));
					}
					else
					{
						UEventsManager::post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGoal, goalId));
					}

					if(delay > 0.0)
					{
						uSleep(delay*1000);
					}
				}
			}
		}
	}
	return data;
}

SensorData DBReader::getNextData(SensorCaptureInfo * info)
{
	SensorData data;
	if(_dbDriver)
	{
		while(_currentId != _ids.end())
		{
			std::list<int> signIds;
			signIds.push_back(*_currentId);
			std::list<Signature *> signatures;
			_dbDriver->loadSignatures(signIds, signatures);
			if(signatures.empty())
			{
				return data;
			}
			_dbDriver->loadNodeData(signatures);
			Signature * s  = signatures.front();

			if(_intermediateNodesIgnored && s->getWeight() == -1)
			{
				UDEBUG("Ignoring node %d (intermediate nodes ignored)", s->id());
				++_currentId;
				delete s;
				continue;
			}

			if(s->mapId() < _startMapId || (_stopMapId>=0 && s->mapId() > _stopMapId))
			{
				UDEBUG("Ignoring node %d (map id=%d, min=%d max=%d)", s->id(), s->mapId(), _startMapId, _stopMapId);
				++_currentId;
				delete s;
				continue;
			}

			data = s->sensorData();

			// info
			Transform pose = s->getPose();
			Transform globalPose;
			cv::Mat globalPoseCov;

			std::multimap<int, Link> priorLinks;
			if(!_priorsIgnored)
			{
				_dbDriver->loadLinks(*_currentId, priorLinks, Link::kPosePrior);
				if( priorLinks.size() &&
					!priorLinks.begin()->second.transform().isNull() &&
					priorLinks.begin()->second.infMatrix().cols == 6 &&
					priorLinks.begin()->second.infMatrix().rows == 6)
				{
					globalPose = priorLinks.begin()->second.transform();
					globalPoseCov = priorLinks.begin()->second.infMatrix().inv();
					if(data.gps().stamp() != 0.0 &&
							globalPoseCov.at<double>(3,3)>=9999 &&
							globalPoseCov.at<double>(4,4)>=9999 &&
							globalPoseCov.at<double>(5,5)>=9999)
					{
						// clear global pose as GPS was used for prior
						globalPose.setNull();
					}
				}
			}

			Transform gravityTransform;
			std::multimap<int, Link> gravityLinks;
			_dbDriver->loadLinks(*_currentId, gravityLinks, Link::kGravity);
			if( gravityLinks.size() &&
				!gravityLinks.begin()->second.transform().isNull() &&
				gravityLinks.begin()->second.infMatrix().cols == 6 &&
				gravityLinks.begin()->second.infMatrix().rows == 6)
			{
				gravityTransform = gravityLinks.begin()->second.transform();
			}

			Landmarks landmarks;
			if(!_landmarksIgnored)
			{
				std::multimap<int, Link> landmarkLinks;
				_dbDriver->loadLinks(*_currentId, landmarkLinks, Link::kLandmark);
				for(std::multimap<int, Link>::iterator iter=landmarkLinks.begin(); iter!=landmarkLinks.end(); ++iter)
				{
					 cv::Mat landmarkSize = iter->second.uncompressUserDataConst();
					landmarks.insert(std::make_pair(-iter->first,
							Landmark(-iter->first,
									!landmarkSize.empty() && landmarkSize.type() == CV_32FC1 && landmarkSize.total()==1?landmarkSize.at<float>(0,0):0.0f,
									iter->second.transform(),
									iter->second.infMatrix().inv())));
				}
			}

			cv::Mat infMatrix = cv::Mat::eye(6,6,CV_64FC1);
			if(!_odometryIgnored)
			{
				std::multimap<int, Link> links;
				_dbDriver->loadLinks(*_currentId, links, Link::kNeighbor);
				if(links.size() && links.begin()->first < *_currentId)
				{
					// assume the first is the backward neighbor, take its variance
					infMatrix = links.begin()->second.infMatrix();
					_previousInfMatrix = infMatrix;
				}
				else
				{
					if(_previousMapId != s->mapId())
					{
						// first node, set high variance to make rtabmap trigger a new map
						infMatrix /= 9999.0;
						UDEBUG("First node of map %d, variance set to 9999", s->mapId());
					}
					else
					{
						// if localization data saved in database, covariance will be set in a prior link
						_dbDriver->loadLinks(*_currentId, links, Link::kPosePrior);
						if(links.size())
						{
							// assume the first is the backward neighbor, take its variance
							infMatrix = links.begin()->second.infMatrix();
							_previousInfMatrix = infMatrix;
						}
						else
						{
							if(_previousInfMatrix.empty())
							{
								_previousInfMatrix = cv::Mat::eye(6,6,CV_64FC1);
							}
							// we have a node not linked to map, use last variance
							infMatrix = _previousInfMatrix;
						}
					}
				}
				_previousMapId = s->mapId();
			}
			else
			{
				pose.setNull();
			}

			int seq = *_currentId;
			++_currentId;

			// Frame rate
			if(this->getImageRate() < 0.0f)
			{
				if(s->getStamp() == 0)
				{
					UERROR("The option to use database stamps is set (framerate<0), but there are no stamps saved in the database! Aborting...");
					delete s;
					return data;
				}
				else if(_previousMapID == s->mapId() && _previousStamp > 0)
				{
					float ratio = -this->getImageRate();
					int sleepTime = 1000.0*(s->getStamp()-_previousStamp)/ratio - 1000.0*_timer.getElapsedTime();
					double stamp = s->getStamp();
					if(sleepTime > 10000)
					{
						UWARN("Detected long delay (%d sec, stamps = %f vs %f). Waiting a maximum of 10 seconds.",
								sleepTime/1000, _previousStamp, s->getStamp());
						sleepTime = 10000;
						stamp = _previousStamp+10;
					}
					if(sleepTime > 2)
					{
						uSleep(sleepTime-2);
					}

					// Add precision at the cost of a small overhead
					while(_timer.getElapsedTime() < (stamp-_previousStamp)/ratio-0.000001)
					{
						//
					}

					double slept = _timer.getElapsedTime();
					_timer.start();
					UDEBUG("slept=%fs vs target=%fs (ratio=%f)", slept, (stamp-_previousStamp)/ratio, ratio);
				}
				_previousStamp = s->getStamp();
				_previousMapID = s->mapId();
			}

			data.uncompressData();
			std::map<int, int> cameraOldNewIndices;
			std::vector<CameraModel> dbModels = data.cameraModels();
			if(dbModels.empty() && !data.stereoCameraModels().empty())
			{
				for(size_t i=0; i<data.stereoCameraModels().size(); ++i)
				{
					dbModels.push_back(data.stereoCameraModels()[i].left());
				}
			}

			if(!_cameraLocalTransformOverrides.empty() && 
				!_cameraIndices.empty() &&
				_cameraIndices.size() != _cameraLocalTransformOverrides.size())
			{
				UERROR("Camera local transform overrides (%d) are not the same size than the camera indices (%d). The overrides are ignored.",
					_cameraLocalTransformOverrides.size(),
					_cameraIndices.size()
				);
				_cameraLocalTransformOverrides.clear();
			}

			std::vector<Transform> combinedLocalTransforms;
			if(dbModels.size() > 1 && !_cameraIndices.empty())
			{
				// update images and local transforms
				cv::Mat combinedImages;
				cv::Mat combinedDepthImages;
				std::vector<CameraModel> combinedModels;
				std::vector<StereoCameraModel> combinedStereoModels;
				for(size_t i=0; i<_cameraIndices.size(); ++i)
				{
					UASSERT_MSG(_cameraIndices[i] < dbModels.size(), uFormat("DBReader: camera index %ld is not valid (should be between 0 and %ld)",
						_cameraIndices[i], dbModels.size()-1).c_str());

					int addedCameras = std::max(combinedModels.size(), combinedStereoModels.size());

					int subImageWidth = data.imageRaw().cols/dbModels.size();
					UASSERT(!data.imageRaw().empty() &&
							data.imageRaw().cols % dbModels.size() == 0 &&
							(int)_cameraIndices[i]*subImageWidth < data.imageRaw().cols);
					if(combinedImages.empty())
					{
						// initialize with first camera
						combinedImages = cv::Mat(data.imageRaw().rows, subImageWidth*(_cameraIndices.size()-i), data.imageRaw().type());
					}

					cv::Mat fromROI = cv::Mat(data.imageRaw(), cv::Rect(_cameraIndices[i]*subImageWidth, 0, subImageWidth, data.imageRaw().rows));
					cv::Mat toROI = cv::Mat(combinedImages, cv::Rect(addedCameras*subImageWidth, 0, subImageWidth, combinedImages.rows));
					fromROI.copyTo(toROI);

					cv::Mat depth;
					if(!data.depthOrRightRaw().empty())
					{
						subImageWidth = data.depthOrRightRaw().cols/dbModels.size();
						UASSERT(data.depthOrRightRaw().cols % dbModels.size() == 0 &&
								subImageWidth == data.depthOrRightRaw().cols/(int)dbModels.size() &&
								(int)_cameraIndices[i]*subImageWidth < data.depthOrRightRaw().cols);
						if(combinedDepthImages.empty())
						{
							// initialize with first camera
							combinedDepthImages = cv::Mat(data.depthOrRightRaw().rows, subImageWidth*(_cameraIndices.size()-i), data.depthOrRightRaw().type());
						}
						fromROI = cv::Mat(data.depthOrRightRaw(), cv::Rect(_cameraIndices[i]*subImageWidth, 0, subImageWidth, data.depthOrRightRaw().rows));
						toROI = cv::Mat(combinedDepthImages, cv::Rect(addedCameras*subImageWidth, 0, subImageWidth, combinedDepthImages.rows));
						fromROI.copyTo(toROI);
					}

					if(!data.cameraModels().empty())
					{
						CameraModel model = data.cameraModels()[_cameraIndices[i]];
						if(!_cameraLocalTransformOverrides.empty())
						{
							model.setLocalTransform(_cameraLocalTransformOverrides[i] * CameraModel::opticalRotation());
						}
						combinedModels.push_back(model);
						combinedLocalTransforms.push_back(model.localTransform());
					}
					else
					{
						StereoCameraModel stereoModel = data.stereoCameraModels()[_cameraIndices[i]];
						if(!_cameraLocalTransformOverrides.empty())
						{
							stereoModel.setLocalTransform(_cameraLocalTransformOverrides[i] * CameraModel::opticalRotation());
						}
						combinedStereoModels.push_back(stereoModel);
						combinedLocalTransforms.push_back(stereoModel.localTransform());
					}
					cameraOldNewIndices.insert(std::make_pair(_cameraIndices[i], i));
				}
				if(!combinedModels.empty())
				{
					data.setRGBDImage(combinedImages, combinedDepthImages, combinedModels);
				}
				else
				{
					data.setStereoImage(combinedImages, combinedDepthImages, combinedStereoModels);
				}
			}
			else if(!_cameraLocalTransformOverrides.empty() &&
					_cameraLocalTransformOverrides.size() == dbModels.size())
			{
				// just update local transforms
				std::vector<CameraModel> combinedModels;
				std::vector<StereoCameraModel> combinedStereoModels;
				for(size_t i=0; i<dbModels.size(); ++i)
				{
					if(!data.cameraModels().empty())
					{
						CameraModel model = data.cameraModels()[i];
						model.setLocalTransform(_cameraLocalTransformOverrides[i] * CameraModel::opticalRotation());
						combinedModels.push_back(model);
						combinedLocalTransforms.push_back(model.localTransform());
					}
					else
					{
						StereoCameraModel stereoModel = data.stereoCameraModels()[i];
						stereoModel.setLocalTransform(_cameraLocalTransformOverrides[i] * CameraModel::opticalRotation());
						combinedStereoModels.push_back(stereoModel);
						combinedLocalTransforms.push_back(stereoModel.localTransform());
					}
				}
				if(!combinedModels.empty())
				{
					data.setCameraModels(combinedModels);
				}
				else
				{
					data.setStereoCameraModels(combinedStereoModels);
				}
			}
			data.setId(seq);
			data.setStamp(s->getStamp());
			data.setGroundTruth(s->getGroundTruthPose());
			if(!globalPose.isNull())
			{
				data.setGlobalPose(globalPose, globalPoseCov);
			}
			if(!gravityTransform.isNull())
			{
				Eigen::Quaterniond q = gravityTransform.getQuaterniond();
				data.setIMU(IMU(
						cv::Vec4d(q.x(), q.y(), q.z(), q.w()), cv::Mat::eye(3,3,CV_64FC1),
						cv::Vec3d(), cv::Mat(),
						cv::Vec3d(), cv::Mat(),
						Transform::getIdentity())); // we assume that gravity links are already transformed in base_link
			}
			data.setLandmarks(landmarks);

			UDEBUG("Laser=%d RGB/Left=%d Depth/Right=%d, Grid=%d, UserData=%d, GlobalPose=%d, GPS=%d, IMU=%d",
					data.laserScanRaw().isEmpty()?0:1,
					data.imageRaw().empty()?0:1,
					data.depthOrRightRaw().empty()?0:1,
					data.gridCellSize()==0.0f?0:1,
					data.userDataRaw().empty()?0:1,
					globalPose.isNull()?0:1,
					data.gps().stamp()!=0.0?1:0,
					gravityTransform.isNull()?0:1);

			cv::Mat descriptors = s->getWordsDescriptors().clone();
			const std::vector<cv::KeyPoint> & keypoints = s->getWordsKpts();
			const std::vector<cv::Point3f> & keypoints3D = s->getWords3();
			if(!_featuresIgnored &&
				!keypoints.empty() &&
			   (keypoints3D.empty() || keypoints.size() == keypoints3D.size()) &&
			   (descriptors.empty() || (int)keypoints.size() == descriptors.rows))
			{
				if(!cameraOldNewIndices.empty())
				{
					cv::Mat newDescriptors;
					std::vector<cv::KeyPoint> newKeypoints;
					std::vector<cv::Point3f> newKeypoints3D;
					UASSERT(!dbModels.empty() && dbModels[0].imageWidth()>0);
					int subImageWidth = dbModels[0].imageWidth();
					for(size_t i = 0; i<keypoints.size(); ++i)
					{
						int cameraIndex = int(keypoints.at(i).pt.x / subImageWidth);
						UASSERT_MSG(cameraIndex >= 0 && cameraIndex < (int)dbModels.size(),
								uFormat("cameraIndex=%d, db models=%d, kpt.x=%f, image width=%d",
										cameraIndex, (int)dbModels.size(), keypoints[i].pt.x, subImageWidth).c_str());
						if(cameraOldNewIndices.find(cameraIndex) != cameraOldNewIndices.end())
						{
							int newCameraIndex = cameraOldNewIndices.at(cameraIndex);
							newKeypoints.push_back(keypoints[i]);
							newKeypoints.back().pt.x += (newCameraIndex-cameraIndex)*subImageWidth;
							if(!keypoints3D.empty())
							{
								cv::Point3f pt = util3d::transformPoint(keypoints3D.at(i), dbModels[cameraIndex].localTransform().inverse());
								pt = util3d::transformPoint(pt, combinedLocalTransforms[cameraIndex]);
								newKeypoints3D.push_back(pt);
							}
							if(!descriptors.empty())
							{
								newDescriptors.push_back(descriptors.row(i));
							}
						}
					}
					data.setFeatures(newKeypoints, newKeypoints3D, newDescriptors);
				}
				else if(!combinedLocalTransforms.empty())
				{
					// We are overriding the camera local transforms, let's move 3D words accordingly
					UASSERT(dbModels.size() == combinedLocalTransforms.size());
					std::vector<cv::Point3f> newKeypoints3D;
					UASSERT(dbModels[0].imageWidth()>0);
					int subImageWidth = dbModels[0].imageWidth();
					for(size_t i = 0; i<keypoints3D.size(); ++i)
					{
						int cameraIndex = int(keypoints.at(i).pt.x / subImageWidth);
						UASSERT_MSG(cameraIndex >= 0 && cameraIndex < (int)dbModels.size(),
								uFormat("cameraIndex=%d, db models=%d, kpt.x=%f, image width=%d",
										cameraIndex, (int)dbModels.size(), keypoints[i].pt.x, subImageWidth).c_str());
						cv::Point3f pt = util3d::transformPoint(keypoints3D.at(i), dbModels[cameraIndex].localTransform().inverse());
						pt = util3d::transformPoint(pt, combinedLocalTransforms[cameraIndex]);
						newKeypoints3D.push_back(pt);
					}
					data.setFeatures(keypoints, newKeypoints3D, descriptors);
				}
				else
				{
					data.setFeatures(keypoints, keypoints3D, descriptors);
				}
			}
			else if(!_featuresIgnored && !keypoints.empty() && (!keypoints3D.empty() || !descriptors.empty()))
			{
				UERROR("Missing feature data, features won't be published.");
			}

			if(data.imageRaw().empty() && data.imageCompressed().empty() && s->getWeight()>=0 && keypoints.empty())
			{
				UWARN("No image loaded from the database for id=%d!", seq);
			}

			if(!_odometryIgnored)
			{
				if(pose.isNull())
				{
					UWARN("Reading the database: odometry is null! "
						  "Please set \"Ignore odometry = true\" if there is "
						  "no odometry in the database.");
				}
				if(info)
				{
					info->odomPose = pose;
					info->odomCovariance = infMatrix.inv();
					info->odomVelocity = s->getVelocity();
					UDEBUG("odom variance = %f/%f", info->odomCovariance.at<double>(0,0), info->odomCovariance.at<double>(5,5));
				}
			}
			delete s;
			break;
		}
	}
	else
	{
		UERROR("Not initialized...");
	}
	return data;
}

} /* namespace rtabmap */
