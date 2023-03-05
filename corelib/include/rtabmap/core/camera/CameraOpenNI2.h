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

#pragma once

#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"

namespace openni
{
class Device;
class VideoStream;
}

namespace rtabmap
{
class RTABMAP_CORE_EXPORT CameraOpenNI2 :
	public Camera
{

public:
	static bool available();
	static bool exposureGainAvailable();
	enum Type {kTypeColorDepth, kTypeIRDepth, kTypeIR};

public:
	CameraOpenNI2(const std::string & deviceId = "",
					Type type = kTypeColorDepth,
					float imageRate = 0,
					const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraOpenNI2();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

	bool setAutoWhiteBalance(bool enabled);
	bool setAutoExposure(bool enabled);
	bool setExposure(int value);
	bool setGain(int value);
	bool setMirroring(bool enabled);
	void setOpenNI2StampsAndIDsUsed(bool used);
	void setIRDepthShift(int horizontal, int vertical);
	void setDepthDecimation(int decimation);

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_OPENNI2
	Type _type;
	openni::Device * _device;
	openni::VideoStream * _color;
	openni::VideoStream * _depth;
	float _depthFx;
	float _depthFy;
	std::string _deviceId;
	bool _openNI2StampsAndIDsUsed;
	StereoCameraModel _stereoModel;
	int _depthHShift;
	int _depthVShift;
	int _depthDecimation;
#endif
};



} // namespace rtabmap
