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

namespace rtabmap {

// SensoriMotor state
class SMState
{
public:
	// Constructor 1
	// image and/or keypoints can be passed for debugging (rtabmap will not re-extract keypoints/descriptors from the image if not null, only for debug/visualization)
	// take image ownership
	SMState(const std::list<std::vector<float> > & sensorStates, const std::list<std::vector<float> > & actuatorStates, IplImage * image = 0, const std::list<cv::KeyPoint> & keypoints = std::list<cv::KeyPoint>()) :
		_sensorStates(sensorStates),
		_actuatorStates(actuatorStates),
		_image(image),
		_keypoints(keypoints),
		_descriptorsProvided(true)
	{}
	// Constructor 2 :
	// rtabmap will automatically extract keypoints and descriptors from the image...
	// take image ownership
	SMState(IplImage * image, const std::list<std::vector<float> > & actuatorStates = std::list<std::vector<float> >()) :
		_actuatorStates(actuatorStates),
		_image(image),
		_descriptorsProvided(false)
	{}
	virtual ~SMState()
	{
		if(_image)
			cvReleaseImage(&_image);
	}

	bool isDescriptorsProvided() const {return _descriptorsProvided;}
	const IplImage * getImage() const {return _image;}
	const std::list<cv::KeyPoint> & getKeypoints() const {return _keypoints;}
	const std::list<std::vector<float> > & getSensorStates() const {return _sensorStates;}
	const std::list<std::vector<float> > & getActuatorStates() const {return _actuatorStates;}

	void setSensorStates(const std::list<std::vector<float> > & sensorStates) {_sensorStates=sensorStates;}
	void setActuatorStates(const std::list<std::vector<float> > & actuatorStates) {_actuatorStates=actuatorStates;}

private:
	std::list<std::vector<float> > _sensorStates; // descriptors
	std::list<std::vector<float> > _actuatorStates;
	IplImage * _image;
	std::list<cv::KeyPoint> _keypoints;
	bool _descriptorsProvided;
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
	const SMState * getData() const {return _state;}
	SMState * getDataOwnership() {SMState * state = _state; _state=0; return state;}
	virtual std::string getClassName() const {return "SMStateEvent";} // TODO : macro?

private:
	SMState * _state;
};

}

#endif /* SMPAIRVARIANT_H_ */
