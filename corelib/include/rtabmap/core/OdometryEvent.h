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

#ifndef ODOMETRYEVENT_H_
#define ODOMETRYEVENT_H_

#include "rtabmap/utilite/UEvent.h"
#include "rtabmap/core/SensorData.h"

namespace rtabmap {

class OdometryEvent : public UEvent
{
public:
	OdometryEvent(
		const SensorData & data, int quality = -1, float time = 0.0f, int features = 0, int localMapSize = 0) :
			_data(data),
			_quality(quality),
			_time(time),
			_features(features),
			_localMapSize(localMapSize)
	{}
	virtual ~OdometryEvent() {}
	virtual std::string getClassName() const {return "OdometryEvent";}

	bool isValid() const {return !_data.pose().isNull();}
	const SensorData & data() const {return _data;}
	int quality() const {return _quality;}
	float time() const {return _time;} // seconds
	int features() const {return _features;}
	int localMapSize() const {return _localMapSize;}

private:
	SensorData _data;
	int _quality;
	float _time; // seconds
	int _features;
	int _localMapSize;
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
