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
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/CameraRGBD.h"

#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap
{

// ownership transferred
CameraThread::CameraThread(Camera * camera) :
		_camera(camera),
		_mirroring(false),
		_colorOnly(false)
{
	UASSERT(_camera != 0);
}

CameraThread::~CameraThread()
{
	join(true);
	if(_camera)
	{
		delete _camera;
	}
}

void CameraThread::setImageRate(float imageRate)
{
	if(_camera)
	{
		_camera->setImageRate(imageRate);
	}
}

void CameraThread::mainLoop()
{
	UTimer timer;
	UDEBUG("");
	SensorData data = _camera->takeImage();

	if(!data.imageRaw().empty())
	{
		if(_colorOnly && !data.depthRaw().empty())
		{
			data.setDepthOrRightRaw(cv::Mat());
		}
		if(_mirroring && data.cameraModels().size() == 1)
		{
			cv::Mat tmpRgb;
			cv::flip(data.imageRaw(), tmpRgb, 1);
			data.setImageRaw(tmpRgb);
			if(data.cameraModels()[0].cx())
			{
				CameraModel tmpModel(
						data.cameraModels()[0].fx(),
						data.cameraModels()[0].fy(),
						float(data.imageRaw().cols) - data.cameraModels()[0].cx(),
						data.cameraModels()[0].cy(),
						data.cameraModels()[0].localTransform());
				data.setCameraModel(tmpModel);
			}
			if(!data.depthRaw().empty())
			{
				cv::Mat tmpDepth;
				cv::flip(data.depthRaw(), tmpDepth, 1);
				data.setDepthOrRightRaw(tmpDepth);
			}
		}

		this->post(new CameraEvent(data, _camera->getSerial()));
	}
	else if(!this->isKilled())
	{
		UWARN("no more images...");
		this->kill();
		this->post(new CameraEvent());
	}
}

void CameraThread::mainLoopKill()
{
	if(dynamic_cast<CameraFreenect2*>(_camera) != 0)
	{
		int i=20;
		while(i-->0)
		{
			uSleep(100);
			if(!this->isKilled())
			{
				break;
			}
		}
		if(this->isKilled())
		{
			//still in killed state, maybe a deadlock
			UERROR("CameraFreenect2: Failed to kill normally the Freenect2 driver! The thread is locked "
				   "on waitForNewFrame() method of libfreenect2. This maybe caused by not linking on the right libusb. "
				   "Note that rtabmap should link on libusb of libfreenect2. "
				   "Tip before starting rtabmap: \"$ export LD_LIBRARY_PATH=~/libfreenect2/depends/libusb/lib:$LD_LIBRARY_PATH\"");
		}

	}
}

} // namespace rtabmap
