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

#ifndef CAMERAARENGINE_H_
#define CAMERAARENGINE_H_

#include "CameraMobile.h"
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/GeodeticCoords.h>
#include <rtabmap/utilite/UMutex.h>
#include <rtabmap/utilite/USemaphore.h>
#include <rtabmap/utilite/UEventsSender.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEvent.h>
#include <rtabmap/utilite/UTimer.h>
#include <boost/thread/mutex.hpp>

#include <huawei_arengine_interface.h>

namespace rtabmap {

class CameraAREngine : public CameraMobile {
public:
	CameraAREngine(void* env, void* context, void* activity, bool smoothing = false);
	virtual ~CameraAREngine();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual void close(); // close Tango connection
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);
	virtual void capturePoseOnly();

private:
	rtabmap::Transform getPoseAtTimestamp(double timestamp);

private:
	void * env_;
	void * context_;
	void * activity_;
	HwArSession* arSession_ = nullptr;
	HwArConfig* arConfig_ = nullptr;
	HwArFrame* arFrame_ = nullptr;
	HwArCameraIntrinsics *arCameraIntrinsics_ = nullptr;
	HwArPose * arPose_ = nullptr;
	bool arInstallRequested_;
	GLuint textureId_;
	UMutex arSessionMutex_;

};

} /* namespace rtabmap */
#endif /* CAMERAARENGINE_H_ */
