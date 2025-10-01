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

#include <rtabmap/core/SensorEvent.h>
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/odometry/OdometryMono.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/utilite/ULogger.h"

namespace rtabmap {

OdometryThread::OdometryThread(Odometry * odometry, unsigned int dataBufferMaxSize) :
	_odometry(odometry),
	_dataBufferMaxSize(dataBufferMaxSize),
	_resetOdometry(false),
	_resetPose(Transform::getIdentity()),
	_oldestAsyncImuStamp(0.0),
	_newestAsyncImuStamp(0.0)
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
		if(event->getClassName().compare("SensorEvent") == 0)
		{
			SensorEvent * sensorEvent = (SensorEvent*)event;
			if(sensorEvent->getCode() == SensorEvent::kCodeData)
			{
				this->addData(*sensorEvent);
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
		_oldestAsyncImuStamp = 0.0;
		_newestAsyncImuStamp = 0.0;
		_previousGuessPose.setNull();
	}

	SensorEvent event;
	if(getData(event))
	{
		OdometryInfo info;
		UDEBUG("Processing data...");
		Transform guess;
		UDEBUG("event.info().odomPose=%s", event.info().odomPose.prettyPrint().c_str());
		if(!_previousGuessPose.isNull() && !event.info().odomPose.isNull()) {
			guess = _previousGuessPose.inverse() * event.info().odomPose;
		}

		SensorData data = event.data();
		Transform pose = _odometry->process(data, guess , &info);
		if(!data.imageRaw().empty() || !data.laserScanRaw().empty() || (pose.isNull() && data.imu().empty()))
		{
			UDEBUG("Odom pose = %s", pose.prettyPrint().c_str());
			if(!pose.isNull()) {
				_previousGuessPose = event.info().odomPose;
				if(!event.info().odomPose.isNull() && info.reg.covariance.at<double>(0,0) >= 9999 &&
					(pose.x() != 0.0f || pose.y() != 0.0f || pose.z() != 0.0f)) // not the first frame
				{
					// In case of external guess and auto reset, keep reporting lost till we
					// process the second frame with valid covariance. This way it
					// won't trigger a new map.
					pose = Transform();
				}
			}
			// a null pose notify that odometry could not be computed
			this->post(new OdometryEvent(data, pose, info));
		}
	}
}

void OdometryThread::addData(const SensorEvent & event)
{
	if(event.data().imu().empty())
	{
		if(dynamic_cast<OdometryMono*>(_odometry) == 0)
		{
			if((event.data().imageRaw().empty() || event.data().depthOrRightRaw().empty() || (event.data().cameraModels().empty() && event.data().stereoCameraModels().empty())) &&
					event.data().laserScanRaw().empty())
			{
				ULOGGER_ERROR("Missing some information (images/scans empty or missing calibration)!?");
				return;
			}
		}
		else
		{
			// Mono can accept RGB only
			if(event.data().imageRaw().empty() || (event.data().cameraModels().empty() && event.data().stereoCameraModels().empty()))
			{
				ULOGGER_ERROR("Missing some information (image empty or missing calibration)!?");
				return;
			}
		}
	}

	bool notify = true;
	_dataMutex.lock();
	{
		if( !event.data().imageRaw().empty() ||
			!event.data().imageCompressed().empty() ||
			!event.data().laserScanRaw().isEmpty() ||
			!event.data().laserScanCompressed().empty() || 
			event.data().imu().empty())
		{
			if(_oldestAsyncImuStamp > 0.0 && event.data().stamp() < _oldestAsyncImuStamp) {
				UWARN("Received image/lidar with stamp (%f) older than oldest received imu "
					"(%f), skipping that frame (imu buffer size=%ld). "
					"When using async IMU, make sure IMU is published faster "
					"than camera/lidar (assuming IMU latency is very small compared to camera/lidar).",
					event.data().stamp(), _oldestAsyncImuStamp, _imuBuffer.size());
				notify = false;
			}
			else if(_newestAsyncImuStamp > 0.0 && event.data().stamp()>=_newestAsyncImuStamp) {
				UWARN("Received image/lidar with stamp (%f) newer than latest received imu "
					"(%f), skipping that frame (imu buffer size=%ld). "
					"When using async IMU, make sure IMU is published faster "
					"than camera/lidar (assuming IMU latency is very small compared to camera/lidar).",
					event.data().stamp(), _newestAsyncImuStamp, _imuBuffer.size());
				notify = false;
			}
			else {
				_dataBuffer.push_back(event);
				while(_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
				{
					UDEBUG("Data buffer is full, the oldest data is removed to add the new one.");
					_dataBuffer.erase(_dataBuffer.begin());
					notify = false;
				}
			}
		}
		else
		{
			_imuBuffer.push_back(event.data());
			if(_oldestAsyncImuStamp == 0) {
				_oldestAsyncImuStamp = event.data().stamp();
			}
			_newestAsyncImuStamp = event.data().stamp();
		}
	}
	_dataMutex.unlock();

	if(notify)
	{
		_dataAdded.release();
	}
}

bool OdometryThread::getData(SensorEvent & event)
{
	bool dataFilled = false;
	_dataAdded.acquire();
	_dataMutex.lock();
	{
		if(!_dataBuffer.empty())
		{
			// Send IMU up to stamp greater than image (OpenVINS needs this).
			while(!_imuBuffer.empty())
			{
				_odometry->process(_imuBuffer.front());
				double stamp =_imuBuffer.front().stamp();
				_imuBuffer.pop_front();
				if(stamp > _dataBuffer.front().data().stamp()) {
					break;
				}
			}

			event = _dataBuffer.front();
			_dataBuffer.pop_front();
			dataFilled = true;
		}
	}
	_dataMutex.unlock();
	return dataFilled;
}

} // namespace rtabmap
