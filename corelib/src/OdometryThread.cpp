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

#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/utilite/ULogger.h"

namespace rtabmap {

OdometryThread::OdometryThread(Odometry * odometry) :
	_odometry(odometry),
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
			if(cameraEvent->getCode() == CameraEvent::kCodeImageDepth)
			{
				this->addData(cameraEvent->data());
			}
			else if(cameraEvent->getCode() == CameraEvent::kCodeNoMoreImages)
			{
				this->post(new CameraEvent()); // forward the event
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
	getData(data);
	if(data.isValid())
	{
		OdometryInfo info;
		Transform pose = _odometry->process(data, &info);
		data.setPose(pose, info.variance, info.variance); // a null pose notify that odometry could not be computed
		this->post(new OdometryEvent(data, info));
	}
}

void OdometryThread::addData(const SensorData & data)
{
	if(dynamic_cast<OdometryMono*>(_odometry) == 0)
	{
		if(data.image().empty() || data.depthOrRightImage().empty() || data.fx() == 0.0f || data.fyOrBaseline() == 0.0f)
		{
			ULOGGER_ERROR("Missing some information (images empty or missing calibration)!?");
			return;
		}
	}
	else
	{
		if(data.image().empty() || data.fx() == 0.0f || data.fyOrBaseline() == 0.0f)
		{
			ULOGGER_ERROR("Missing some information (image empty or missing calibration)!?");
			return;
		}
	}

	bool notify = true;
	_dataMutex.lock();
	{
		notify = !_dataBuffer.isValid();
		_dataBuffer = data;
	}
	_dataMutex.unlock();

	if(notify)
	{
		_dataAdded.release();
	}
}

void OdometryThread::getData(SensorData & data)
{
	_dataAdded.acquire();
	_dataMutex.lock();
	{
		if(_dataBuffer.isValid())
		{
			data = _dataBuffer;
			_dataBuffer = SensorData();
		}
	}
	_dataMutex.unlock();
}

} // namespace rtabmap
