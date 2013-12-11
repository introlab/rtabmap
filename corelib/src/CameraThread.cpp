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

#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/ParamEvent.h"

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UTimer.h>

namespace rtabmap
{

// ownership transferred
CameraThread::CameraThread(Camera * camera, bool autoRestart) :
		_camera(camera),
		_autoRestart(autoRestart),
		_seq(0)
{
	UASSERT(_camera != 0);
}

CameraThread::~CameraThread()
{
	UEventsManager::removeHandler(this);
	join(true);
	delete _camera;
}

bool CameraThread::init()
{
	if(!this->isRunning())
	{
		if(_camera)
		{
			_seq = 0;
			return _camera->init();
		}
		else
		{
			UERROR("Cannot initialize the camera because the camera object is null...");
		}
	}
	else
	{
		UERROR("Cannot initialize the camera because it is already running...");
	}
	return false;
}

void CameraThread::mainLoop()
{
	State state = kStateCapturing;
	ParametersMap parameters;

	_stateMutex.lock();
	{
		if(!_state.empty() && !_stateParam.empty())
		{
			state = _state.top();
			_state.pop();
			parameters = _stateParam.top();
			_stateParam.pop();
		}
	}
	_stateMutex.unlock();

	if(state == kStateCapturing)
	{
		process();
	}
	else if(state == kStateChangingParameters)
	{
		_camera->parseParameters(parameters);
	}
}

void CameraThread::pushNewState(State newState, const ParametersMap & parameters)
{
	ULOGGER_DEBUG("to %d", newState);

	_stateMutex.lock();
	{
		_state.push(newState);
		_stateParam.push(parameters);
	}
	_stateMutex.unlock();
}

void CameraThread::handleEvent(UEvent* anEvent)
{
	if(anEvent->getClassName().compare("ParamEvent") == 0)
	{
		if(this->isIdle())
		{
			_stateMutex.lock();
			_camera->parseParameters(((ParamEvent*)anEvent)->getParameters());
			_stateMutex.unlock();
		}
		else
		{
			ULOGGER_DEBUG("changing parameters");
			pushNewState(kStateChangingParameters, ((ParamEvent*)anEvent)->getParameters());
		}
	}
}

void CameraThread::process()
{
	UTimer timer;
	ULOGGER_DEBUG("Camera::process()");
	cv::Mat descriptors;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat img = _camera->takeImage(descriptors, keypoints);
	if(!img.empty() && !this->isKilled())
	{
		if(_camera->isFeaturesExtracted())
		{
			this->post(new CameraEvent(descriptors, keypoints, img, ++_seq));
		}
		else
		{
			this->post(new CameraEvent(img, ++_seq));
		}
	}
	else if(!this->isKilled())
	{
		if(_autoRestart)
		{
			_camera->init();
		}
		else
		{
			ULOGGER_DEBUG("Camera::process() : no more images...");
			this->kill();
			this->post(new CameraEvent());
		}
	}
}

} // namespace rtabmap
