/*
 * OdometryEvent.h
 *
 *  Created on: 2013-10-15
 *      Author: Mathieu
 */

#ifndef ODOMETRYEVENT_H_
#define ODOMETRYEVENT_H_

#include "rtabmap/utilite/UEvent.h"
#include "rtabmap/core/Image.h"

namespace rtabmap {

class OdometryEvent : public UEvent
{
public:
	OdometryEvent(
		const Image & data) :
			_data(data) {}
	virtual ~OdometryEvent() {}
	virtual std::string getClassName() const {return "OdometryEvent";}

	bool isValid() const {return !_data.pose().isNull();}
	const Image & data() const {return _data;}

private:
	Image _data;
};

class OdometryResetEvent : public UEvent
{
public:
	OdometryResetEvent(){}
	virtual ~OdometryResetEvent() {}
	virtual std::string getClassName() const {return "OdometryResetEvent";}
};

}


#endif /* ODOMETRYEVENT_H_ */
