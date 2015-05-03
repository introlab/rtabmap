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

#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/DBDriver.h"
#include "DBDriverSqlite3.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>

#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/Compression.h"

namespace rtabmap {

DBReader::DBReader(const std::string & databasePath,
				   float frameRate,
				   bool odometryIgnored,
				   bool ignoreGoalDelay) :
	_paths(uSplit(databasePath, ';')),
	_frameRate(frameRate),
	_odometryIgnored(odometryIgnored),
	_ignoreGoalDelay(ignoreGoalDelay),
	_dbDriver(0),
	_currentId(_ids.end())
{
}

DBReader::DBReader(const std::list<std::string> & databasePaths,
				   float frameRate,
				   bool odometryIgnored,
				   bool ignoreGoalDelay) :
   _paths(databasePaths),
   _frameRate(frameRate),
	_odometryIgnored(odometryIgnored),
	_ignoreGoalDelay(ignoreGoalDelay),
	_dbDriver(0),
	_currentId(_ids.end())
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

bool DBReader::init(int startIndex)
{
	if(_dbDriver)
	{
		_dbDriver->closeConnection();
		delete _dbDriver;
		_dbDriver = 0;
	}
	_ids.clear();
	_currentId=_ids.end();
	_previousStamp = 0;

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
	if(startIndex>0 && _ids.size())
	{
		std::set<int>::iterator iter = uIteratorAt(_ids, startIndex);
		if(iter == _ids.end())
		{
			UWARN("Start index is too high (%d), the last in database is %d. Starting from beginning...", startIndex, _ids.size()-1);
		}
		else
		{
			_currentId = iter;
		}
	}

	return true;
}

void DBReader::setFrameRate(float frameRate)
{
	_frameRate = frameRate;
}

void DBReader::mainLoopBegin()
{
	_timer.start();
}

void DBReader::mainLoop()
{
	SensorData data = this->getNextData();
	if(data.isValid())
	{
		int goalId = 0;
		double previousStamp = data.stamp();
		data.setStamp(UTimer::now());
		if(data.userData().size() >= 6 && memcmp(data.userData().data(), "GOAL:", 5) == 0)
		{
			//GOAL format detected, remove it from the user data and send it as goal event
			std::string goalStr = uBytes2Str(data.userData());
			if(!goalStr.empty())
			{
				std::list<std::string> strs = uSplit(goalStr, ':');
				if(strs.size() == 2)
				{
					goalId = atoi(strs.rbegin()->c_str());
					data.setUserData(std::vector<unsigned char>());
				}
			}
		}
		if(!_odometryIgnored)
		{
			if(data.pose().isNull())
			{
				UWARN("Reading the database: odometry is null! "
					  "Please set \"Ignore odometry = true\" if there is "
					  "no odometry in the database.");
			}
			this->post(new OdometryEvent(data));
		}
		else
		{
			this->post(new CameraEvent(data));
		}

		if(goalId > 0)
		{
			this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGoal, "", goalId));

			if(!_ignoreGoalDelay && _currentId != _ids.end())
			{
				// get stamp for the next signature to compute the delay
				// that was used originally for planning
				int weight;
				std::string label;
				double stamp;
				int mapId;
				Transform localTransform, pose;
				std::vector<unsigned char> userData;
				_dbDriver->getNodeInfo(*_currentId, pose, mapId, weight, label, stamp, userData);
				if(previousStamp && stamp && stamp > previousStamp)
				{
					double delay = stamp - previousStamp;
					UWARN("Goal %d detected, posting it! Waiting %f seconds before sending next data...",
							goalId, delay);
					uSleep(delay*1000);
				}
			}
			else
			{
				UWARN("Goal %d detected, posting it!", goalId);
			}
		}

	}
	else if(!this->isKilled())
	{
		UINFO("no more images...");
		if(_paths.size() > 1)
		{
			_paths.pop_front();
			UWARN("Loading next database \"%s\"...", _paths.front().c_str());
			if(!this->init())
			{
				UERROR("Failed to initialize the next database \"%s\"", _paths.front().c_str());
				this->kill();
				this->post(new CameraEvent());
			}
		}
		else
		{
			this->kill();
			this->post(new CameraEvent());
		}

	}

}

SensorData DBReader::getNextData()
{
	SensorData data;
	if(_dbDriver)
	{
		if(!this->isKilled() && _currentId != _ids.end())
		{
			cv::Mat imageBytes;
			cv::Mat depthBytes;
			cv::Mat laserScanBytes;
			int mapId;
			float fx,fy,cx,cy;
			Transform localTransform, pose;
			float rotVariance = 1.0f;
			float transVariance = 1.0f;
			std::vector<unsigned char> userData;
			int laserScanMaxPts = 0;
			_dbDriver->getNodeData(*_currentId, imageBytes, depthBytes, laserScanBytes, fx, fy, cx, cy, localTransform, laserScanMaxPts);

			// info
			int weight;
			std::string label;
			double stamp;
			_dbDriver->getNodeInfo(*_currentId, pose, mapId, weight, label, stamp, userData);

			if(!_odometryIgnored)
			{
				std::map<int, Link> links;
				_dbDriver->loadLinks(*_currentId, links, Link::kNeighbor);
				if(links.size())
				{
					// assume the first is the backward neighbor, take its variance
					rotVariance = links.begin()->second.rotVariance();
					transVariance = links.begin()->second.transVariance();
				}
			}
			else
			{
				pose.setNull();
			}

			int seq = *_currentId;
			++_currentId;
			if(imageBytes.empty())
			{
				UWARN("No image loaded from the database for id=%d!", *_currentId);
			}

			// Frame rate
			if(_frameRate < 0.0f)
			{
				if(stamp == 0)
				{
					UERROR("The option to use database stamps is set (framerate<0), but there are no stamps saved in the database! Aborting...");
					this->kill();
				}
				else if(_previousStamp > 0)
				{
					int sleepTime = 1000.0*(stamp-_previousStamp) - 1000.0*_timer.getElapsedTime();
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
			}
			else if(_frameRate>0.0f)
			{
				int sleepTime = (1000.0f/_frameRate - 1000.0f*_timer.getElapsedTime());
				if(sleepTime > 2)
				{
					uSleep(sleepTime-2);
				}

				// Add precision at the cost of a small overhead
				while(_timer.getElapsedTime() < 1.0/double(_frameRate)-0.000001)
				{
					//
				}

				double slept = _timer.getElapsedTime();
				_timer.start();
				UDEBUG("slept=%fs vs target=%fs", slept, 1.0/double(_frameRate));
			}

			if(!this->isKilled())
			{
				rtabmap::CompressionThread ctImage(imageBytes, true);
				rtabmap::CompressionThread ctDepth(depthBytes, true);
				rtabmap::CompressionThread ctLaserScan(laserScanBytes, false);
				ctImage.start();
				ctDepth.start();
				ctLaserScan.start();
				ctImage.join();
				ctDepth.join();
				ctLaserScan.join();
				data = SensorData(
						ctLaserScan.getUncompressedData(),
						laserScanMaxPts,
						ctImage.getUncompressedData(),
						ctDepth.getUncompressedData(),
						fx,fy,cx,cy,
						localTransform,
						pose,
						rotVariance,
						transVariance,
						seq,
						stamp,
						userData);
				UDEBUG("Laser=%d RGB/Left=%d Depth=%d Right=%d",
						data.laserScan().empty()?0:1,
						data.image().empty()?0:1,
						data.depth().empty()?0:1,
						data.rightImage().empty()?0:1);
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
