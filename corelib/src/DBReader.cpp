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

#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/util3d.h"

namespace rtabmap {

DBReader::DBReader(const std::string & databasePath,
				   float frameRate,
				   bool odometryIgnored,
				   float delayToStartSec) :
	_path(databasePath),
	_frameRate(frameRate),
	_odometryIgnored(odometryIgnored),
	_delayToStartSec(delayToStartSec),
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

	if(!UFile::exists(_path))
	{
		UERROR("Database path does not exist (%s)", _path.c_str());
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
	if(!_dbDriver->openConnection(_path))
	{
		UERROR("Can't open database %s", _path.c_str());
		delete _dbDriver;
		_dbDriver = 0;
		return false;
	}

	_dbDriver->getAllNodeIds(_ids);
	_currentId = _ids.begin();
	if(startIndex>0 && _ids.size())
	{
		std::set<int>::iterator iter = _ids.lower_bound(startIndex);
		if(iter == _ids.end())
		{
			UWARN("Start index is too high (%d), the last in database is %d. Starting from beginning...", startIndex, *_ids.rbegin());
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
	if(frameRate >= 0.0f)
	{
		_frameRate = frameRate;
	}
}

void DBReader::mainLoopBegin()
{
	if(_delayToStartSec > 0.0f)
	{
		uSleep(_delayToStartSec*1000.0f);
	}
	_timer.start();
}

void DBReader::mainLoop()
{
	cv::Mat image, depth, depth2d;
	float fx,fy,cx,cy;
	Transform localTransform, pose;
	int seq = 0;
	this->getNextImage(image, depth, depth2d, fx, fy, cx, cy, localTransform, pose, seq);
	if(!image.empty())
	{
		if(depth.empty())
		{
			this->post(new CameraEvent(image));
		}
		else
		{
			if(!_odometryIgnored)
			{
				SensorData data(image, depth, depth2d, fx, fy, cx, cy, pose, localTransform, seq);
				this->post(new OdometryEvent(data));
				if(pose.isNull())
				{
					UWARN("Reading the database: odometry is null! "
						  "Please set \"Ignore odometry = true\" if there is "
						  "no odometry in the database.");
				}
			}
			else
			{
				// without odometry
				this->post(new CameraEvent(image, depth, depth2d, fx, fy, cx, cy, localTransform, seq));
			}
		}
	}
	else if(!this->isKilled())
	{
		UINFO("no more images...");
		this->kill();
		this->post(new CameraEvent());
	}

}

void DBReader::getNextImage(
		cv::Mat & image,
		cv::Mat & depth,
		cv::Mat & depth2d,
		float & fx,
		float & fy,
		float & cx,
		float & cy,
		Transform & localTransform,
		Transform & pose,
		int & seq)
{
	if(_dbDriver)
	{
		float frameRate = _frameRate;
		if(frameRate>0.0f)
		{
			int sleepTime = (1000.0f/frameRate - 1000.0f*_timer.getElapsedTime());
			if(sleepTime > 2)
			{
				uSleep(sleepTime-2);
			}

			// Add precision at the cost of a small overhead
			while(_timer.getElapsedTime() < 1.0/double(frameRate)-0.000001)
			{
				//
			}

			double slept = _timer.getElapsedTime();
			_timer.start();
			UDEBUG("slept=%fs vs target=%fs", slept, 1.0/double(frameRate));
		}

		if(!this->isKilled() && _currentId != _ids.end())
		{
			cv::Mat imageBytes;
			cv::Mat depthBytes;
			cv::Mat depth2dBytes;
			int mapId;
			_dbDriver->getNodeData(*_currentId, imageBytes, depthBytes, depth2dBytes, fx, fy, cx, cy, localTransform);
			_dbDriver->getPose(*_currentId, pose, mapId);
			seq = *_currentId;
			++_currentId;
			if(imageBytes.empty())
			{
				UWARN("No image loaded from the database for id=%d!", *_currentId);
			}

			util3d::CompressionThread ctImage(imageBytes, true);
			util3d::CompressionThread ctDepth(depthBytes, true);
			util3d::CompressionThread ctDepth2D(depth2dBytes, false);
			ctImage.start();
			ctDepth.start();
			ctDepth2D.start();
			ctImage.join();
			ctDepth.join();
			ctDepth2D.join();
			image = ctImage.getUncompressedData();
			depth = ctDepth.getUncompressedData();
			depth2d = ctDepth2D.getUncompressedData();
		}
	}
	else
	{
		UERROR("Not initialized...");
	}
}

} /* namespace rtabmap */
