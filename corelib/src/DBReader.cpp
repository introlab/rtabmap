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

#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/DBDriver.h"
#include "DBDriverSqlite3.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UEventsManager.h>

#include "rtabmap/core/CameraEvent.h"
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
				   int startIndex,
				   int cameraIndex) :
	Camera(frameRate),
	_paths(uSplit(databasePath, ';')),
	_odometryIgnored(odometryIgnored),
	_ignoreGoalDelay(ignoreGoalDelay),
	_goalsIgnored(goalsIgnored),
	_startIndex(startIndex),
	_cameraIndex(cameraIndex),
	_dbDriver(0),
	_currentId(_ids.end()),
	_previousMapId(-1),
	_previousStamp(0),
	_previousMapID(0),
	_calibrated(false)
{
}

DBReader::DBReader(const std::list<std::string> & databasePaths,
				   float frameRate,
				   bool odometryIgnored,
				   bool ignoreGoalDelay,
				   bool goalsIgnored,
				   int startIndex,
				   int cameraIndex) :
	Camera(frameRate),
   _paths(databasePaths),
	_odometryIgnored(odometryIgnored),
	_ignoreGoalDelay(ignoreGoalDelay),
	_goalsIgnored(goalsIgnored),
	_startIndex(startIndex),
	_cameraIndex(cameraIndex),
	_dbDriver(0),
	_currentId(_ids.end()),
	_previousMapId(-1),
	_previousStamp(0),
	_previousMapID(0),
	_calibrated(false)
{
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
		const std::string & calibrationFolder,
		const std::string & cameraName)
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
	_dbDriver = new DBDriverSqlite3(parameters);
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
	if(_startIndex>0 && _ids.size())
	{
		std::set<int>::iterator iter = uIteratorAt(_ids, _startIndex);
		if(iter == _ids.end())
		{
			UWARN("Start index is too high (%d), the last in database is %d. Starting from beginning...", _startIndex, _ids.size()-1);
		}
		else
		{
			_currentId = iter;
		}
	}

	if(_ids.size())
	{
		std::vector<CameraModel> models;
		StereoCameraModel stereoModel;
		if(_dbDriver->getCalibration(*_ids.begin(), models, stereoModel))
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
			else if(stereoModel.isValidForProjection())
			{
				_calibrated = true;
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

SensorData DBReader::captureImage(CameraInfo * info)
{
	SensorData data = this->getNextData(info);
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
						_dbDriver->getNodeInfo(*_currentId, pose, mapId, weight, label, stamp, groundTruth, velocity, gps);
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

SensorData DBReader::getNextData(CameraInfo * info)
{
	SensorData data;
	if(_dbDriver)
	{
		if(_currentId != _ids.end())
		{
			int mapId;
			_dbDriver->getNodeData(*_currentId, data);

			// info
			Transform pose;
			int weight;
			std::string label;
			double stamp;
			Transform groundTruth;
			std::vector<float> velocity;
			GPS gps;
			_dbDriver->getNodeInfo(*_currentId, pose, mapId, weight, label, stamp, groundTruth, velocity, gps);

			cv::Mat infMatrix = cv::Mat::eye(6,6,CV_64FC1);
			if(!_odometryIgnored)
			{
				std::map<int, Link> links;
				_dbDriver->loadLinks(*_currentId, links, Link::kNeighbor);
				if(links.size() && links.begin()->first < *_currentId)
				{
					// assume the first is the backward neighbor, take its variance
					infMatrix = links.begin()->second.infMatrix();
					_previousInfMatrix = infMatrix;
				}
				else if(_previousMapId != mapId)
				{
					// first node, set high variance to make rtabmap trigger a new map
					infMatrix /= 9999.0;
					UDEBUG("First node of map %d, variance set to 9999", mapId);
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
				_previousMapId = mapId;
			}
			else
			{
				pose.setNull();
			}

			int seq = *_currentId;
			++_currentId;
			if(data.imageCompressed().empty() && weight>=0)
			{
				UWARN("No image loaded from the database for id=%d!", *_currentId);
			}

			// Frame rate
			if(this->getImageRate() < 0.0f)
			{
				if(stamp == 0)
				{
					UERROR("The option to use database stamps is set (framerate<0), but there are no stamps saved in the database! Aborting...");
					return data;
				}
				else if(_previousMapID == mapId && _previousStamp > 0)
				{
					int sleepTime = 1000.0*(stamp-_previousStamp) - 1000.0*_timer.getElapsedTime();
					if(sleepTime > 10000)
					{
						UWARN("Detected long delay (%d sec, stamps = %f vs %f). Waiting a maximum of 10 seconds.",
								sleepTime/1000, _previousStamp, stamp);
						sleepTime = 10000;
					}
					if(sleepTime > 2)
					{
						uSleep(sleepTime-2);
					}

					// Add precision at the cost of a small overhead
					while(_timer.getElapsedTime() < (stamp-_previousStamp)-0.000001)
					{
						//
					}

					double slept = _timer.getElapsedTime();
					_timer.start();
					UDEBUG("slept=%fs vs target=%fs", slept, stamp-_previousStamp);
				}
				_previousStamp = stamp;
				_previousMapID = mapId;
			}
			else
			{
				stamp = 0;
			}

			data.uncompressData();
			if(data.cameraModels().size() > 1 &&
				_cameraIndex >= 0)
			{
				if(_cameraIndex < (int)data.cameraModels().size())
				{
					// select one camera
					int subImageWidth = data.imageRaw().cols/data.cameraModels().size();
					UASSERT(!data.imageRaw().empty() &&
							data.imageRaw().cols % data.cameraModels().size() == 0 &&
							_cameraIndex*subImageWidth < data.imageRaw().cols);
					data.setImageRaw(
							cv::Mat(data.imageRaw(),
							cv::Rect(_cameraIndex*subImageWidth, 0, subImageWidth, data.imageRaw().rows)).clone());

					if(!data.depthOrRightRaw().empty())
					{
						UASSERT(data.depthOrRightRaw().cols % data.cameraModels().size() == 0 &&
								subImageWidth == data.depthOrRightRaw().cols/(int)data.cameraModels().size() &&
								_cameraIndex*subImageWidth < data.depthOrRightRaw().cols);
						data.setDepthOrRightRaw(
								cv::Mat(data.depthOrRightRaw(),
								cv::Rect(_cameraIndex*subImageWidth, 0, subImageWidth, data.depthOrRightRaw().rows)).clone());
					}
					CameraModel model = data.cameraModels().at(_cameraIndex);
					data.setCameraModel(model);
				}
				else
				{
					UWARN("DBReader: Camera index %d doesn't exist! Camera models = %d.", _cameraIndex, (int)data.cameraModels().size());
				}
			}
			data.setId(seq);
			data.setStamp(stamp);
			data.setGroundTruth(groundTruth);
			data.setGPS(gps);
			UDEBUG("Laser=%d RGB/Left=%d Depth/Right=%d, UserData=%d",
					data.laserScanRaw().empty()?0:1,
					data.imageRaw().empty()?0:1,
					data.depthOrRightRaw().empty()?0:1,
					data.userDataRaw().empty()?0:1);

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
					info->odomVelocity = velocity;
					UDEBUG("odom variance = %f/%f", info->odomCovariance.at<double>(0,0), info->odomCovariance.at<double>(5,5));
				}
			}
		}
	}
	else
	{
		UERROR("Not initialized...");
	}
	return data;
}

} /* namespace rtabmap */
