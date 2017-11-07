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
#include "rtabmap/core/Version.h"

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

namespace rs
{
	class context;
	class device;
	namespace slam {
	class slam;
	}
}

typedef struct _freenect_context freenect_context;
typedef struct _freenect_device freenect_device;

typedef struct IKinectSensor IKinectSensor;
typedef struct ICoordinateMapper ICoordinateMapper;
typedef struct _DepthSpacePoint DepthSpacePoint;
typedef struct _ColorSpacePoint ColorSpacePoint;
typedef struct tagRGBQUAD RGBQUAD;
typedef struct IMultiSourceFrameReader IMultiSourceFrameReader;

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
#endif
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
	enum Type {kTypeColorDepth, kTypeIRDepth};

public:
	// default local transform z in, x right, y down));
	CameraFreenect(int deviceId= 0,
					Type type = kTypeColorDepth,
					float imageRate=0.0f,
					const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraFreenect();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_FREENECT
	int deviceId_;
	Type type_;
	freenect_context * ctx_;
	FreenectDevice * freenectDevice_;
	StereoCameraModel stereoModel_;
#endif
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
					Type type = kTypeDepth2ColorSD,
					float imageRate=0.0f,
					const Transform & localTransform = Transform::getIdentity(),
					float minDepth = 0.3f,
					float maxDepth = 12.0f,
					bool bilateralFiltering = true,
					bool edgeAwareFiltering = true,
					bool noiseFiltering = true,
					const std::string & pipelineName = "");
	virtual ~CameraFreenect2();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_FREENECT2
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
	std::string pipelineName_;
#endif
};

/////////////////////////
// CameraK4W2
/////////////////////////

class RTABMAP_EXP CameraK4W2 :
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

/////////////////////////
// CameraRealSense
/////////////////////////
class slam_event_handler;
class RTABMAP_EXP CameraRealSense :
	public Camera
{
public:
	static bool available();

public:
	// default local transform z in, x right, y down));
	CameraRealSense(
		int deviceId = 0,
		int presetRGB = 0, // 0=best quality, 1=largest image, 2=highest framerate
		int presetDepth = 0, // 0=best quality, 1=largest image, 2=highest framerate
		bool computeOdometry = false,
		float imageRate = 0,
		const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraRealSense();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;
	virtual bool odomProvided() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_REALSENSE
	rs::context * ctx_;
	rs::device * dev_;
	int deviceId_;
	int presetRGB_;
	int presetDepth_;
	bool computeOdometry_;

	int motionSeq_[2];
	rs::slam::slam * slam_;
	UMutex slamLock_;

	std::map<double, std::pair<cv::Mat, cv::Mat> > bufferedFrames_;
	std::pair<cv::Mat, cv::Mat> lastSyncFrames_;
	UMutex dataMutex_;
	USemaphore dataReady_;
#endif
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
