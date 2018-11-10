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

#include "rtabmap/core/Camera.h"

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <cmath>

namespace rtabmap
{

Camera::Camera(float imageRate, const Transform & localTransform) :
	_imageRate(imageRate),
	_localTransform(localTransform),
	_targetImageSize(0,0),
	_frameRateTimer(new UTimer()),
	_seq(0)
{
}

Camera::~Camera()
{
	UDEBUG("");
	delete _frameRateTimer;
	UDEBUG("");
}

void Camera::resetTimer()
{
	_frameRateTimer->start();
}

SensorData Camera::takeImage(CameraInfo * info)
{
	bool warnFrameRateTooHigh = false;
	float actualFrameRate = 0;
	float imageRate = _imageRate;
	if(imageRate>0)
	{
		int sleepTime = (1000.0f/imageRate - 1000.0f*_frameRateTimer->getElapsedTime());
		if(sleepTime > 2)
		{
			uSleep(sleepTime-2);
		}
		else if(sleepTime < 0)
		{
			warnFrameRateTooHigh = true;
			actualFrameRate = 1.0/(_frameRateTimer->getElapsedTime());
		}

		// Add precision at the cost of a small overhead
		while(_frameRateTimer->getElapsedTime() < 1.0/double(imageRate)-0.000001)
		{
			//
		}

		double slept = _frameRateTimer->getElapsedTime();
		_frameRateTimer->start();
		UDEBUG("slept=%fs vs target=%fs", slept, 1.0/double(imageRate));
	}

	UTimer timer;
	SensorData data  = this->captureImage(info);
	double captureTime = timer.ticks();
	if(warnFrameRateTooHigh)
	{
		UWARN("Camera: Cannot reach target image rate %f Hz, current rate is %f Hz and capture time = %f s.",
				imageRate, actualFrameRate, captureTime);
	}
	else
	{
		UDEBUG("Time capturing image = %fs", captureTime);
	}
	if(info)
	{
		info->id = data.id();
		info->stamp = data.stamp();
		info->timeCapture = captureTime;
	}
	return data;
}

} // namespace rtabmap
