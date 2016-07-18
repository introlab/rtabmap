/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap/core/OdometryF2M.h>
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryMono.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/utilite/ULogger.h"

namespace rtabmap {

OdometryThread::OdometryThread(Odometry * odometry, unsigned int dataBufferMaxSize) :
	_odometry(odometry),
	_dataBufferMaxSize(dataBufferMaxSize),
	_resetOdometry(false)
{
	UASSERT(_odometry != 0);
}

OdometryThread::~OdometryThread()
{
	this->unregisterFromEventsManager();
	this->join(true);
	if(_odometry)
	{
		delete _odometry;
	}
	UDEBUG("");
}

void OdometryThread::handleEvent(UEvent * event)
{
	if(this->isRunning())
	{
		if(event->getClassName().compare("CameraEvent") == 0)
		{
			CameraEvent * cameraEvent = (CameraEvent*)event;
			if(cameraEvent->getCode() == CameraEvent::kCodeData)
			{
				this->addData(cameraEvent->data());
			}
		}
		else if(event->getClassName().compare("OdometryResetEvent") == 0)
		{
			_resetOdometry = true;
		}
	}
}

void OdometryThread::mainLoopKill()
{
	_dataAdded.release();
}

//============================================================
// MAIN LOOP
//============================================================
void OdometryThread::mainLoop()
{
	if(_resetOdometry)
	{
		_odometry->reset();
		_resetOdometry = false;
	}

	SensorData data;
	if(getData(data))
	{
		OdometryInfo info;
		Transform pose = _odometry->process(data, &info);
		// a null pose notify that odometry could not be computed
		double variance = info.variance>0?info.variance:1;
		this->post(new OdometryEvent(data, pose, variance, variance, info));
	}
}

void OdometryThread::addData(const SensorData & data)
{
	if(dynamic_cast<OdometryMono*>(_odometry) == 0 && dynamic_cast<OdometryF2M*>(_odometry) == 0)
	{
		if(data.imageRaw().empty() || data.depthOrRightRaw().empty() || (data.cameraModels().size()==0 && !data.stereoCameraModel().isValidForProjection()))
		{
			ULOGGER_ERROR("Missing some information (images empty or missing calibration)!?");
			return;
		}
	}
	else
	{
		// Mono and BOW can accept RGB only
		if(data.imageRaw().empty() || (data.cameraModels().size()==0 && !data.stereoCameraModel().isValidForProjection()))
		{
			ULOGGER_ERROR("Missing some information (image empty or missing calibration)!?");
			return;
		}
	}

	bool notify = true;
	_dataMutex.lock();
	{
		_dataBuffer.push_back(data);
		while(_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
		{
			UDEBUG("Data buffer is full, the oldest data is removed to add the new one.");
			_dataBuffer.pop_front();
			notify = false;
		}
	}
	_dataMutex.unlock();

	if(notify)
	{
		_dataAdded.release();
	}
}

bool OdometryThread::getData(SensorData & data)
{
	bool dataFilled = false;
	_dataAdded.acquire();
	_dataMutex.lock();
	{
		if(!_dataBuffer.empty())
		{
			data = _dataBuffer.front();
			_dataBuffer.pop_front();
			dataFilled = true;
		}
	}
	_dataMutex.unlock();
	return dataFilled;
}

} // namespace rtabmap
