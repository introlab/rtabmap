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
	cv::Mat descriptors;
	std::vector<cv::KeyPoint> keypoints;
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
