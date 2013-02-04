/*
 * DBReader.cpp
 *
 *  Created on: 2012-06-13
 *      Author: mathieu
 */

#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/DBDriver.h"
#include "DBDriverSqlite3.h"

#include <utilite/ULogger.h>
#include <utilite/UEventsManager.h>
#include <utilite/UFile.h>

#include "rtabmap/core/CameraEvent.h"

namespace rtabmap {

DBReader::DBReader(const std::string & databasePath,
				   float frameRate) :
	_path(databasePath),
	_frameRate(frameRate),
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
	_timer.start();
}

void DBReader::mainLoop()
{
	cv::Mat image;
	this->getNextImage(image);
	if(!image.empty())
	{
		UEventsManager::post(new CameraEvent(image));
	}
	else if(!this->isKilled())
	{
		UDEBUG("no more images...");
		this->kill();
		UEventsManager::post(new CameraEvent());
	}

}

void DBReader::getNextImage(cv::Mat & image)
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
			//sensors
			_dbDriver->getImage(*_currentId, image);
			++_currentId;
			if(image.empty())
			{
				UWARN("No image loaded from the database!");
			}
		}
	}
	else
	{
		UERROR("Not initialized...");
	}
}

} /* namespace rtabmap */
