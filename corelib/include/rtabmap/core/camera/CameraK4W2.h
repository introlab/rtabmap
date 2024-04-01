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

#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"

typedef struct IKinectSensor IKinectSensor;
typedef struct ICoordinateMapper ICoordinateMapper;
typedef struct _DepthSpacePoint DepthSpacePoint;
typedef struct _ColorSpacePoint ColorSpacePoint;
typedef struct tagRGBQUAD RGBQUAD;
typedef struct IMultiSourceFrameReader IMultiSourceFrameReader;

namespace rtabmap
{

class RTABMAP_CORE_EXPORT CameraK4W2 :
	public Camera
{
public:
	static bool available();

	enum Type {
		kTypeColor2DepthSD,
		kTypeDepth2ColorSD,
		kTypeDepth2ColorHD
	};

public:
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;

public:
	// default local transform z in, x right, y down));
	CameraK4W2(int deviceId = 0, // not used
		Type type = kTypeDepth2ColorSD,
		float imageRate = 0.0f,
		const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraK4W2();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	void close();

private:
#ifdef RTABMAP_K4W2
	Type type_;
	IKinectSensor*          pKinectSensor_;
	ICoordinateMapper*      pCoordinateMapper_;
	DepthSpacePoint*        pDepthCoordinates_;
	ColorSpacePoint*        pColorCoordinates_;
	IMultiSourceFrameReader* pMultiSourceFrameReader_;
	RGBQUAD * pColorRGBX_;
	INT_PTR hMSEvent;
	CameraModel colorCameraModel_;
#endif
};


} // namespace rtabmap
