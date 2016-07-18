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

#include "rtabmap/utilite/UMutex.h"
#include "rtabmap/utilite/USemaphore.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/CameraRGB.h"

#include <pcl/pcl_config.h>

#ifdef HAVE_OPENNI
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_image.h>
#endif

#include <boost/signals2/connection.hpp>

namespace openni
{
class Device;
class VideoStream;
}

namespace pcl
{
class Grabber;
}

namespace libfreenect2
{
class Freenect2;
class Freenect2Device;
class SyncMultiFrameListener;
class Registration;
class PacketPipeline;
}

typedef struct _freenect_context freenect_context;
typedef struct _freenect_device freenect_device;

namespace rtabmap
{

/////////////////////////
// CameraOpenNIPCL
/////////////////////////
class RTABMAP_EXP CameraOpenni :
	public Camera
{
public:
	static bool available();

public:
	// default local transform z in, x right, y down));
	CameraOpenni(const std::string & deviceId="",
			float imageRate = 0,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraOpenni();
#ifdef HAVE_OPENNI
    void image_cb (
    		const boost::shared_ptr<openni_wrapper::Image>& rgb,
			const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
			float constant);
#endif

    virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
    pcl::Grabber* interface_;
    std::string deviceId_;
    boost::signals2::connection connection_;
    cv::Mat depth_;
    cv::Mat rgb_;
    float depthConstant_;
    UMutex dataMutex_;
    USemaphore dataReady_;
};

/////////////////////////
// CameraOpenNICV
/////////////////////////
class RTABMAP_EXP CameraOpenNICV :
	public Camera
{

public:
	static bool available();

public:
	CameraOpenNICV(bool asus = false,
					float imageRate = 0,
					const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraOpenNICV();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const {return "";} // unknown with OpenCV

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	bool _asus;
	cv::VideoCapture _capture;
	float _depthFocal;
};

/////////////////////////
// CameraOpenNI2
/////////////////////////
class RTABMAP_EXP CameraOpenNI2 :
	public Camera
{

public:
	static bool available();
	static bool exposureGainAvailable();

public:
	CameraOpenNI2(const std::string & deviceId = "",
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
	void setOpenNI2StampsAndIDsUsed(bool used) {_openNI2StampsAndIDsUsed = used;}

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	openni::Device * _device;
	openni::VideoStream * _color;
	openni::VideoStream * _depth;
	float _depthFx;
	float _depthFy;
	std::string _deviceId;
	bool _openNI2StampsAndIDsUsed;
};


/////////////////////////
// CameraFreenect
/////////////////////////
class FreenectDevice;

class RTABMAP_EXP CameraFreenect :
	public Camera
{
public:
	static bool available();

public:
	// default local transform z in, x right, y down));
	CameraFreenect(int deviceId= 0,
					float imageRate=0.0f,
					const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraFreenect();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	int deviceId_;
	freenect_context * ctx_;
	FreenectDevice * freenectDevice_;
};

/////////////////////////
// CameraFreenect2
/////////////////////////

class RTABMAP_EXP CameraFreenect2 :
	public Camera
{
public:
	static bool available();

	enum Type{
		kTypeColor2DepthSD,
		kTypeDepth2ColorSD,
		kTypeDepth2ColorHD,
		kTypeDepth2ColorHD2,
		kTypeIRDepth,
		kTypeColorIR
	};

public:
	// default local transform z in, x right, y down));
	CameraFreenect2(int deviceId= 0,
					Type type = kTypeColor2DepthSD,
					float imageRate=0.0f,
					const Transform & localTransform = Transform::getIdentity(),
					float minDepth = 0.3f,
					float maxDepth = 12.0f,
					bool bilateralFiltering = true,
					bool edgeAwareFiltering = true,
					bool noiseFiltering = true);
	virtual ~CameraFreenect2();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	int deviceId_;
	Type type_;
	StereoCameraModel stereoModel_;
	libfreenect2::Freenect2 * freenect2_;
	libfreenect2::Freenect2Device *dev_;
	libfreenect2::SyncMultiFrameListener * listener_;
	libfreenect2::Registration * reg_;
	float minKinect2Depth_;
	float maxKinect2Depth_;
	bool bilateralFiltering_;
	bool edgeAwareFiltering_;
	bool noiseFiltering_;
};


/////////////////////////
// CameraRGBDImages
/////////////////////////
class CameraImages;
class RTABMAP_EXP CameraRGBDImages :
	public CameraImages
{
public:
	static bool available();

public:
	CameraRGBDImages(
			const std::string & pathRGBImages,
			const std::string & pathDepthImages,
			float depthScaleFactor = 1.0f,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraRGBDImages();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	CameraImages cameraDepth_;
};

} // namespace rtabmap
