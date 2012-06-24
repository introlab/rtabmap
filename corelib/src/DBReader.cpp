/*
 * DBReader.cpp
 *
 *  Created on: 2012-06-13
 *      Author: mathieu
 */

#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/DBDriver.h"

#include "rtabmap/core/SensorimotorEvent.h"
#include "rtabmap/core/DBDriverFactory.h"

#include <utilite/ULogger.h>
#include <utilite/UEventsManager.h>
#include <utilite/UFile.h>

namespace rtabmap {

DBReader::DBReader(const std::string & databasePath,
				   float frameRate,
		           const std::set<Sensor::Type> & sensorTypes,
		           const std::set<Actuator::Type> & actuatorTypes) :
	_path(databasePath),
	_frameRate(frameRate),
	_sensorTypes(sensorTypes),
	_actuatorTypes(actuatorTypes),
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

bool DBReader::init()
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
	_dbDriver = DBDriverFactory::createDBDriver("sqlite3", parameters);
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
	std::list<Sensor> sensors;
	std::list<Actuator> actuators;
	this->getNextSensorimotorState(sensors, actuators);
	if(!sensors.empty() || !actuators.empty())
	{
		UEventsManager::post(new SensorimotorEvent(sensors, actuators));
	}
	else if(!this->isKilled())
	{
		UDEBUG("no more sensorimotor states...");
		this->kill();
		UEventsManager::post(new SensorimotorEvent());
	}

}

void DBReader::getNextSensorimotorState(std::list<Sensor> & sensors, std::list<Actuator> & actuators)
{
	sensors.clear();
	actuators.clear();

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
			_dbDriver->getRawData(*_currentId, sensors);

			//actuators
			NeighborsMultiMap neighbors;
			++_currentId;
			if(_currentId != _ids.end())
			{
				_dbDriver->getActuatorData(*_currentId, actuators);
			}

			UDEBUG("sensors.size=%d actuators.size=%d", sensors.size(), actuators.size());

			//filtering for types wanted
			if(_sensorTypes.size())
			{
				for(std::list<Sensor>::iterator jter=sensors.begin(); jter!=sensors.end();)
				{
					if(_sensorTypes.find((Sensor::Type)jter->type()) == _sensorTypes.end())
					{
						jter = sensors.erase(jter);
					}
					else
					{
						++jter;
					}
				}
			}
			if(_actuatorTypes.size())
			{
				for(std::list<Actuator>::iterator jter=actuators.begin(); jter!=actuators.end();)
				{
					if(_actuatorTypes.find((Actuator::Type)jter->type()) == _actuatorTypes.end())
					{
						jter = actuators.erase(jter);
					}
					else
					{
						++jter;
					}
				}
			}

			UDEBUG("after filtering sensors.size=%d actuators.size=%d", sensors.size(), actuators.size());
		}
	}
	else
	{
		UERROR("Not initialized...");
	}
}

} /* namespace rtabmap */
