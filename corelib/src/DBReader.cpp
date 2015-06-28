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
	_currentId(_ids.end()),
	_previousStamp(0)
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
	_currentId(_ids.end()),
	_previousStamp(0)
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
	OdometryEvent odom = this->getNextData();
	if(odom.data().id())
	{
		int goalId = 0;
		double previousStamp = odom.data().stamp();
		odom.data().setStamp(UTimer::now());
		if(odom.data().userDataRaw().type() == CV_8SC1 &&
		   odom.data().userDataRaw().cols >= 7 && // including null str ending
		   odom.data().userDataRaw().rows == 1 &&
		   memcmp(odom.data().userDataRaw().data, "GOAL:", 5) == 0)
		{
			//GOAL format detected, remove it from the user data and send it as goal event
			std::string goalStr = (const char *)odom.data().userDataRaw().data;
			if(!goalStr.empty())
			{
				std::list<std::string> strs = uSplit(goalStr, ':');
				if(strs.size() == 2)
				{
					goalId = atoi(strs.rbegin()->c_str());
					odom.data().setUserData(cv::Mat());
				}
			}
		}
		if(!_odometryIgnored)
		{
			if(odom.pose().isNull())
			{
				UWARN("Reading the database: odometry is null! "
					  "Please set \"Ignore odometry = true\" if there is "
					  "no odometry in the database.");
			}
			this->post(new OdometryEvent(odom));
		}
		else
		{
			this->post(new CameraEvent(odom.data()));
		}

		if(goalId > 0)
		{
			if(!_ignoreGoalDelay && _currentId != _ids.end())
			{
				// get stamp for the next signature to compute the delay
				// that was used originally for planning
				int weight;
				std::string label;
				double stamp;
				int mapId;
				Transform localTransform, pose;
				_dbDriver->getNodeInfo(*_currentId, pose, mapId, weight, label, stamp);
				if(previousStamp && stamp && stamp > previousStamp)
				{
					double delay = stamp - previousStamp;
					UWARN("Goal %d detected, posting it! Waiting %f seconds before sending next data...",
							goalId, delay);
					this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGoal, "", goalId));
					uSleep(delay*1000);
				}
				else
				{
					UWARN("Goal %d detected, posting it!", goalId);
					this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGoal, "", goalId));
				}
			}
			else
			{
				UWARN("Goal %d detected, posting it!", goalId);
				this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGoal, "", goalId));
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

OdometryEvent DBReader::getNextData()
{
	OdometryEvent odom;
	if(_dbDriver)
	{
		if(!this->isKilled() && _currentId != _ids.end())
		{
			int mapId;
			SensorData data;
			_dbDriver->getNodeData(*_currentId, data);

			// info
			Transform pose;
			int weight;
			std::string label;
			double stamp;
			_dbDriver->getNodeInfo(*_currentId, pose, mapId, weight, label, stamp);

			cv::Mat infMatrix = cv::Mat::eye(6,6,CV_64FC1);
			if(!_odometryIgnored)
			{
				std::map<int, Link> links;
				_dbDriver->loadLinks(*_currentId, links, Link::kNeighbor);
				if(links.size())
				{
					// assume the first is the backward neighbor, take its variance
					infMatrix = links.begin()->second.infMatrix();
				}
			}
			else
			{
				pose.setNull();
			}

			int seq = *_currentId;
			++_currentId;
			if(data.imageCompressed().empty())
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
				data.uncompressData();
				data.setId(seq);
				data.setStamp(stamp);
				UDEBUG("Laser=%d RGB/Left=%d Depth/Right=%d, UserData=%d",
						data.laserScanRaw().empty()?0:1,
						data.imageRaw().empty()?0:1,
						data.depthOrRightRaw().empty()?0:1,
						data.userDataRaw().empty()?0:1);

				odom = OdometryEvent(data, pose, infMatrix.inv());
			}
		}
	}
	else
	{
		UERROR("Not initialized...");
	}
	return odom;
}

} /* namespace rtabmap */
