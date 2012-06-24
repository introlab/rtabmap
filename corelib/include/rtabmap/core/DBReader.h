/*
 * DBReader.h
 *
 *  Created on: 2012-06-13
 *      Author: mathieu
 */

#ifndef DBREADER_H_
#define DBREADER_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "rtabmap/core/Sensor.h"
#include "rtabmap/core/Actuator.h"

#include <utilite/UThreadNode.h>
#include <utilite/UTimer.h>

#include <set>

namespace rtabmap {

class DBDriver;

class RTABMAP_EXP DBReader : public UThreadNode {
public:
	DBReader(const std::string & databasePath,
			 float frameRate = 0.0f,
			 const std::set<Sensor::Type> & sensorTypes = std::set<Sensor::Type>(),
			 const std::set<Actuator::Type> & actuatorTypes = std::set<Actuator::Type>());
	virtual ~DBReader();

	bool init();
	void setFrameRate(float frameRate);
	void getNextSensorimotorState(std::list<Sensor> & sensors, std::list<Actuator> & actuators);

protected:
	virtual void mainLoopBegin();
	virtual void mainLoop();

private:
	std::string _path;
	float _frameRate;
	std::set<Sensor::Type> _sensorTypes;
	std::set<Actuator::Type> _actuatorTypes;

	DBDriver * _dbDriver;
	UTimer _timer;
	std::set<int> _ids;
	std::set<int>::iterator _currentId;
};

} /* namespace rtabmap */
#endif /* DBREADER_H_ */
