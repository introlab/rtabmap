/*
 * SensorimotorEvent.h
 *
 *  Created on: 2012-05-27
 *      Author: mathieu
 */

#ifndef SENSORIMOTOREVENT_H_
#define SENSORIMOTOREVENT_H_

#include "rtabmap/core/Sensor.h"
#include "rtabmap/core/Actuator.h"
#include <utilite/UEvent.h>

namespace rtabmap
{

class SensorimotorEvent : public UEvent
{
public:
	enum Type {
			kTypeData,
			kTypeNoMoreData
		};

public:
	SensorimotorEvent() :
		UEvent(kTypeNoMoreData) {}
	SensorimotorEvent(const std::list<Sensor> & sensors,
					  const std::list<Actuator> & actuators) :
        UEvent(kTypeData),
		sensors_(sensors),
		actuators_(actuators) {}
	virtual ~SensorimotorEvent() {}
	int type() const {return this->getCode();}
	virtual std::string getClassName() const {return "SensorimotorEvent";}
	const std::list<Sensor> & sensors() const {return sensors_;}
	const std::list<Actuator> & actuators() const {return actuators_;}
private:
	std::list<Sensor> sensors_;
	std::list<Actuator> actuators_;
};

}


#endif /* SENSORIMOTOREVENT_H_ */
