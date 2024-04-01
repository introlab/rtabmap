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

#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/odometry/OdometryMono.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/utilite/ULogger.h"

namespace rtabmap {

OdometryThread::OdometryThread(Odometry * odometry, unsigned int dataBufferMaxSize) :
	_odometry(odometry),
	_dataBufferMaxSize(dataBufferMaxSize),
	_resetOdometry(false),
	_resetPose(Transform::getIdentity()),
	_lastImuStamp(0.0),
	_imuEstimatedDelay(0.0)
{
	UASSERT(_odometry != 0);
}

OdometryThread::~OdometryThread()
{
	this->unregisterFromEventsManager();
	this->join(true);
	delete _odometry;
	UDEBUG("");
}

bool OdometryThread::handleEvent(UEvent * event)
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
		else if(event->getClassName().compare("IMUEvent") == 0)
		{
			IMUEvent * imuEvent = (IMUEvent*)event;
			if(!imuEvent->getData().empty())
			{
				this->addData(SensorData(imuEvent->getData(), 0, imuEvent->getStamp()));
			}
		}
	}
	if(event->getClassName().compare("OdometryResetEvent") == 0)
	{
		OdometryResetEvent * odomEvent = (OdometryResetEvent*)event;
		_resetPose.setIdentity();
		if(!odomEvent->getPose().isNull())
		{
			_resetPose = odomEvent->getPose();
		}
		_resetOdometry = true;
	}
	return false;
}

void OdometryThread::mainLoopBegin()
{
	ULogger::registerCurrentThread("Odometry");
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
		_odometry->reset(_resetPose);
		_resetOdometry = false;
		UScopeMutex lock(_dataMutex);
		_dataBuffer.clear();
		_imuBuffer.clear();
		_lastImuStamp = 0.0f;
	}

	SensorData data;
	if(getData(data))
	{
		OdometryInfo info;
		UDEBUG("Processing data...");
		Transform pose = _odometry->process(data, &info);
		if(!data.imageRaw().empty() || !data.laserScanRaw().empty() || (pose.isNull() && data.imu().empty()))
		{
			UDEBUG("Odom pose = %s", pose.prettyPrint().c_str());
			// a null pose notify that odometry could not be computed
			this->post(new OdometryEvent(data, pose, info));
		}
	}
}

void OdometryThread::addData(const SensorData & data)
{
	if(data.imu().empty())
	{
		if(dynamic_cast<OdometryMono*>(_odometry) == 0)
		{
			if((data.imageRaw().empty() || data.depthOrRightRaw().empty() || (data.cameraModels().empty() && data.stereoCameraModels().empty())) &&
					data.laserScanRaw().empty())
			{
				ULOGGER_ERROR("Missing some information (images/scans empty or missing calibration)!?");
				return;
			}
		}
		else
		{
			// Mono can accept RGB only
			if(data.imageRaw().empty() || (data.cameraModels().empty() && data.stereoCameraModels().empty()))
			{
				ULOGGER_ERROR("Missing some information (image empty or missing calibration)!?");
				return;
			}
		}
	}

	bool notify = true;
	_dataMutex.lock();
	{
		if(!data.imageRaw().empty() || !data.laserScanRaw().isEmpty() || data.imu().empty())
		{
			_dataBuffer.push_back(data);
			while(_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
			{
				UDEBUG("Data buffer is full, the oldest data is removed to add the new one.");
				_dataBuffer.erase(_dataBuffer.begin());
				notify = false;
			}
		}
		else
		{
			_imuBuffer.push_back(data);
			if(_lastImuStamp != 0.0 && data.stamp() > _lastImuStamp)
			{
				_imuEstimatedDelay = data.stamp() - _lastImuStamp;
			}
			_lastImuStamp = data.stamp();
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
			if(!_imuBuffer.empty())
			{
				// Send IMU up to stamp greater than image (OpenVINS needs this).
				while(!_imuBuffer.empty())
				{
					_odometry->process(_imuBuffer.front());
					double stamp = _imuBuffer.front().stamp();
					_imuBuffer.pop_front();
					if(stamp > _dataBuffer.front().stamp())
					{
						break;
					}
				}
			}

			data = _dataBuffer.front();
			_dataBuffer.pop_front();
			dataFilled = true;
		}
	}
	_dataMutex.unlock();
	return dataFilled;
}

} // namespace rtabmap
