/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SMPAIRVARIANT_H_
#define SMPAIRVARIANT_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include <vector>
#include <utilite/ULogger.h>

namespace rtabmap {

// SensoriMotor state
class SMState
{
public:
	// Constructor 0 : ownership is transferred
	SMState(IplImage * image = 0) :
		_image(image)
	{}
	// Constructor 1
	SMState(const std::list<std::vector<float> > & sensors, const std::list<std::vector<float> > & actuators) :
		_sensors(sensors),
		_actuators(actuators),
		_image(0)
	{}
	// Constructor 2 : for convenience with ROS conversion...
	// Sensors and actuators vectors will be split into a list with smaller vectors of length sensorStep and actuatorStep respectively.
	SMState(const std::vector<float> & sensors, int sensorStep, const std::vector<float> & actuators, int actuatorStep) :
		_image(0)
	{
		if(sensorStep && sensors.size() % sensorStep != 0)
		{
			UERROR("Sensors must all have the same length.");
		}
		if(actuatorStep && actuators.size() % actuatorStep != 0)
		{
			UERROR("Actuators must all have the same length.");
		}
		for(unsigned int i=0; i<sensors.size() && i<i+sensorStep; i+=sensorStep)
		{
			_sensors.push_back(std::vector<float>(sensors.data()+i, sensors.data()+i+sensorStep));
		}
		for(unsigned int i=0; i<actuators.size() && i<i+actuatorStep; i+=actuatorStep)
		{
			_actuators.push_back(std::vector<float>(actuators.data()+i, actuators.data()+i+actuatorStep));
		}
	}

	virtual ~SMState()
	{
		if(_image)
		{
			cvReleaseImage(&_image);
		}
	}

	const IplImage * getImage() const {return _image;}
	const std::list<cv::KeyPoint> & getKeypoints() const {return _keypoints;}
	const std::list<std::vector<float> > & getSensors() const {return _sensors;}
	const std::list<std::vector<float> > & getActuators() const {return _actuators;}
	void setSensors(const std::list<std::vector<float> > & sensors) {_sensors=sensors;}
	void setActuators(const std::list<std::vector<float> > & actuators) {_actuators=actuators;}
	void setKeypoints(const std::list<cv::KeyPoint> & keypoints) {_keypoints = keypoints;}

	//ownership is transferred
	void setImage(IplImage * image)
	{
		if(_image)
		{
			cvReleaseImage(&_image);
		}
		_image = image;
	}

	void getSensorsMerged(std::vector<float> & sensors, int & step) const
	{
		sensors.clear();
		step = 0;
		if(_sensors.size())
		{
			// here we assume that all sensors have the same length
			step = _sensors.front().size();
			for(std::list<std::vector<float> >::const_iterator iter = _sensors.begin();
					iter != _sensors.end();
					++iter)
			{
				sensors.insert(sensors.end(), iter->begin(), iter->end());
			}
		}
	}
	void getActuatorsMerged(std::vector<float> & actuators, int & step) const
	{
		actuators.clear();
		step = 0;
		if(_actuators.size())
		{
			// here we assume that all sensors have the same length
			step = _actuators.front().size();
			for(std::list<std::vector<float> >::const_iterator iter = _actuators.begin();
					iter != _actuators.end();
					++iter)
			{
				actuators.insert(actuators.end(), iter->begin(), iter->end());
			}
		}
	}

private:
	std::list<std::vector<float> > _sensors; // descriptors
	std::list<std::vector<float> > _actuators;
	IplImage * _image;
	std::list<cv::KeyPoint> _keypoints;
};

// Sensorimotor state event
// Take ownership of the state
class SMStateEvent : public UEvent
{
public:
	SMStateEvent(SMState * state) :
		UEvent(0),
		_state(state) {}

	virtual ~SMStateEvent() {if(_state) delete _state;}
	const SMState * getSMState() const {return _state;}
	SMState * getSMStateOwnership() {SMState * state = _state; _state=0; return state;}
	virtual std::string getClassName() const {return "SMStateEvent";} // TODO : macro?

private:
	SMState * _state;
};

}

#endif /* SMPAIRVARIANT_H_ */
