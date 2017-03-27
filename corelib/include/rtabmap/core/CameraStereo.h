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

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/CameraRGB.h"
#include "rtabmap/core/Version.h"
#include <list>

namespace FlyCapture2
{
class Camera;
}

namespace sl
{
namespace zed
{
class Camera;
}
}

namespace rtabmap
{

/////////////////////////
// CameraStereoDC1394
/////////////////////////
class DC1394Device;

class RTABMAP_EXP CameraStereoDC1394 :
	public Camera
{
public:
	static bool available();

public:
	CameraStereoDC1394( float imageRate=0.0f, const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraStereoDC1394();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_DC1394
	DC1394Device *device_;
	StereoCameraModel stereoModel_;
#endif
};

/////////////////////////
// CameraStereoFlyCapture2
/////////////////////////
class RTABMAP_EXP CameraStereoFlyCapture2 :
	public Camera
{
public:
	static bool available();

public:
	CameraStereoFlyCapture2( float imageRate=0.0f, const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraStereoFlyCapture2();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_FLYCAPTURE2
	FlyCapture2::Camera * camera_;
	void * triclopsCtx_; // TriclopsContext
#endif
};

/////////////////////////
// CameraStereoZED
/////////////////////////
class RTABMAP_EXP CameraStereoZed :
	public Camera
{
public:
	static bool available();

public:
	CameraStereoZed(
			int deviceId,
			int resolution = 2, // 0=HD2K, 1=HD1080, 2=HD720, 3=VGA
			int quality = 1,    // 0=NONE, 1=PERFORMANCE, 2=QUALITY
			int sensingMode = 1,// 0=FULL, 1=RAW
			int confidenceThr = 100,
			bool computeOdometry = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity(),
			bool selfCalibration = false);
	CameraStereoZed(
			const std::string & svoFilePath,
			int quality = 1,    // 0=NONE, 1=PERFORMANCE, 2=QUALITY
			int sensingMode = 1,// 0=FULL, 1=RAW
			int confidenceThr = 100,
			bool computeOdometry = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity(),
			bool selfCalibration = false);
	virtual ~CameraStereoZed();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;
	virtual bool odomProvided() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_ZED
	sl::zed::Camera * zed_;
	StereoCameraModel stereoModel_;
	CameraVideo::Source src_;
	int usbDevice_;
	std::string svoFilePath_;
	int resolution_;
	int quality_;
	bool selfCalibration_;
	int sensingMode_;
	int confidenceThr_;
	bool computeOdometry_;
	bool lost_;
#endif
};

/////////////////////////
// CameraStereoImages
/////////////////////////
class CameraImages;
class RTABMAP_EXP CameraStereoImages :
	public CameraImages
{
public:
	static bool available();

public:
	CameraStereoImages(
			const std::string & pathLeftImages,
			const std::string & pathRightImages,
			bool rectifyImages = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity());
	CameraStereoImages(
			const std::string & pathLeftRightImages,
			bool rectifyImages = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraStereoImages();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	CameraImages * camera2_;
	StereoCameraModel stereoModel_;
};


/////////////////////////
// CameraStereoVideo
/////////////////////////
class CameraImages;
class RTABMAP_EXP CameraStereoVideo :
	public Camera
{
public:
	static bool available();

public:
	CameraStereoVideo(
			const std::string & pathSideBySide,
			bool rectifyImages = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity());
	CameraStereoVideo(
			const std::string & pathLeft,
			const std::string & pathRight,
			bool rectifyImages = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity());
	CameraStereoVideo(
			int device,
			bool rectifyImages = false,
			float imageRate = 0.0f,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraStereoVideo();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	cv::VideoCapture capture_;
	cv::VideoCapture capture2_;
	std::string path_;
	std::string path2_;
	bool rectifyImages_;
	StereoCameraModel stereoModel_;
	std::string cameraName_;
	CameraVideo::Source src_;
	int usbDevice_;
};

} // namespace rtabmap
