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

#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraEvent.h"

#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap
{

// ownership transferred
CameraThread::CameraThread(Camera * camera) :
		_camera(camera),
		_cameraRGBD(0),
		_seq(0)
{
	UASSERT(_camera != 0);
}

// ownership transferred
CameraThread::CameraThread(CameraRGBD * camera) :
		_camera(0),
		_cameraRGBD(camera),
		_seq(0)
{
	UASSERT(_cameraRGBD != 0);
}

CameraThread::~CameraThread()
{
	join(true);
	if(_camera)
	{
		delete _camera;
	}
	if(_cameraRGBD)
	{
		delete _cameraRGBD;
	}
}

void CameraThread::setImageRate(float imageRate)
{
	if(_camera)
	{
		_camera->setImageRate(imageRate);
	}
	if(_cameraRGBD)
	{
		_cameraRGBD->setImageRate(imageRate);
	}
}

bool CameraThread::init()
{
	if(!this->isRunning())
	{
		_seq = 0;
		if(_cameraRGBD)
		{
			return _cameraRGBD->init();
		}
		else
		{
			return _camera->init();
		}

		// Added sleep time to ignore first frames (which are darker)
		uSleep(1000);
	}
	else
	{
		UERROR("Cannot initialize the camera because it is already running...");
	}
	return false;
}

void CameraThread::mainLoop()
{
	UTimer timer;
	UDEBUG("");
	cv::Mat rgb, depth;
	float fx = 0.0f;
	float fy = 0.0f;
	float cx = 0.0f;
	float cy = 0.0f;
	if(_cameraRGBD)
	{
		_cameraRGBD->takeImage(rgb, depth, fx, fy, cx, cy);
	}
	else
	{
		rgb = _camera->takeImage();
	}

	if(!rgb.empty() && !this->isKilled())
	{
		if(_cameraRGBD)
		{
			this->post(new CameraEvent(rgb, depth, fx, fy, cx, cy, _cameraRGBD->getLocalTransform(), ++_seq));
		}
		else
		{
			this->post(new CameraEvent(rgb, ++_seq));
		}
	}
	else if(!this->isKilled())
	{
		if(_cameraRGBD)
		{
			UERROR("Retrieved data is empty! Stopping the camera...");
		}
		else
		{
			UWARN("no more images...");
		}
		this->kill();
		this->post(new CameraEvent());
	}
}

} // namespace rtabmap
