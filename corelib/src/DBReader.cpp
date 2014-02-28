/*
 * DBReader.cpp
 *
 *  Created on: 2012-06-13
 *      Author: mathieu
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
	float depthConstant;
	Transform localTransform, pose;
	this->getNextImage(image, depth, depth2d, depthConstant, localTransform, pose);
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
				Image data(image, depth, depth2d, depthConstant, pose, localTransform);
				this->post(new OdometryEvent(data));
			}
			else
			{
				// without odometry
				this->post(new CameraEvent(image, depth, depth2d, depthConstant, localTransform));
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
		float & depthConstant,
		Transform & localTransform,
		Transform & pose)
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
			std::vector<unsigned char> imageBytes;
			std::vector<unsigned char> depthBytes;
			std::vector<unsigned char> depth2dBytes;
			int mapId;
			_dbDriver->getNodeData(*_currentId, imageBytes, depthBytes, depth2dBytes, depthConstant, localTransform);
			_dbDriver->getPose(*_currentId, pose, mapId);
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
