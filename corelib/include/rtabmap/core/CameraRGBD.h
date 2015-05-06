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

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include "rtabmap/core/SensorData.h"
#include "rtabmap/utilite/UMutex.h"
#include "rtabmap/utilite/USemaphore.h"
#include "rtabmap/core/CameraModel.h"
#include <set>
#include <stack>
#include <list>
#include <vector>

#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_image.h>

#include <boost/signals2/connection.hpp>

class UDirectory;
class UTimer;

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

namespace FlyCapture2
{
class Camera;
}

typedef struct _freenect_context freenect_context;
typedef struct _freenect_device freenect_device;

namespace rtabmap
{

/**
 * Class CameraRGBD
 *
 */
class RTABMAP_EXP CameraRGBD
{
public:
	virtual ~CameraRGBD();
	void takeImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy);

	virtual bool init(const std::string & calibrationFolder = ".") = 0;
	virtual bool isCalibrated() const = 0;
	virtual std::string getSerial() const = 0;

	//getters
	float getImageRate() const {return _imageRate;}
	const Transform & getLocalTransform() const {return _localTransform;}
	bool isMirroringEnabled() const {return _mirroring;}
	bool isColorOnly() const {return _colorOnly;}

	//setters
	void setImageRate(float imageRate) {_imageRate = imageRate;}
	void setLocalTransform(const Transform & localTransform) {_localTransform= localTransform;}
	void setMirroringEnabled(bool mirroring) {_mirroring = mirroring;}
	void setColorOnly(bool colorOnly) {_colorOnly = colorOnly;}

protected:
	/**
	 * Constructor
	 *
	 * @param imageRate : image/second , 0 for fast as the camera can
	 */
	CameraRGBD(float imageRate = 0,
				const Transform & localTransform = Transform::getIdentity());

	/**
	 * returned rgb and depth images should be already rectified
	 */
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy) = 0;

private:
	float _imageRate;
	Transform _localTransform;
	bool _mirroring;
	bool _colorOnly;
	UTimer * _frameRateTimer;
};

/////////////////////////
// CameraOpenNIPCL
/////////////////////////
class RTABMAP_EXP CameraOpenni :
	public CameraRGBD
{
public:
	static bool available() {return true;}

public:
	// default local transform z in, x right, y down));
	CameraOpenni(const std::string & deviceId="",
			float imageRate = 0,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraOpenni();

    void image_cb (
    		const boost::shared_ptr<openni_wrapper::Image>& rgb,
			const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
			float constant);

    virtual bool init(const std::string & calibrationFolder = ".");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const;

protected:
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy);

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
	public CameraRGBD
{

public:
	static bool available();

public:
	CameraOpenNICV(bool asus = false,
					float imageRate = 0,
					const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraOpenNICV();

	virtual bool init(const std::string & calibrationFolder = ".");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const {return "";} // unknown with OpenCV

protected:
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy);

private:
	bool _asus;
	cv::VideoCapture _capture;
	float _depthFocal;
};

/////////////////////////
// CameraOpenNI2
/////////////////////////
class RTABMAP_EXP CameraOpenNI2 :
	public CameraRGBD
{

public:
	static bool available();
	static bool exposureGainAvailable();

public:
	CameraOpenNI2(const std::string & deviceId = "",
					float imageRate = 0,
					const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraOpenNI2();

	virtual bool init(const std::string & calibrationFolder = ".");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

	bool setAutoWhiteBalance(bool enabled);
	bool setAutoExposure(bool enabled);
	bool setExposure(int value);
	bool setGain(int value);
	bool setMirroring(bool enabled);

protected:
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy);

private:
	openni::Device * _device;
	openni::VideoStream * _color;
	openni::VideoStream * _depth;
	float _depthFx;
	float _depthFy;
	std::string _deviceId;
};


/////////////////////////
// CameraFreenect
/////////////////////////
class FreenectDevice;

class RTABMAP_EXP CameraFreenect :
	public CameraRGBD
{
public:
	static bool available();

public:
	// default local transform z in, x right, y down));
	CameraFreenect(int deviceId= 0,
					float imageRate=0.0f,
					const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraFreenect();

	virtual bool init(const std::string & calibrationFolder = ".");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy);

private:
	int deviceId_;
	freenect_context * ctx_;
	FreenectDevice * freenectDevice_;
};

/////////////////////////
// CameraFreenect2
/////////////////////////

class RTABMAP_EXP CameraFreenect2 :
	public CameraRGBD
{
public:
	static bool available();

	enum Type{
		kTypeRGBDepthSD,
		kTypeRGBDepthHD,
		kTypeIRDepth,
		kTypeRGBIR
	};

public:
	// default local transform z in, x right, y down));
	CameraFreenect2(int deviceId= 0,
					Type type = kTypeRGBDepthSD,
					float imageRate=0.0f,
					const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraFreenect2();

	virtual bool init(const std::string & calibrationFolder = ".");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy);

private:
	int deviceId_;
	Type type_;
	StereoCameraModel stereoModel_;
	libfreenect2::Freenect2 * freenect2_;
	libfreenect2::Freenect2Device *dev_;
	libfreenect2::PacketPipeline * pipeline_;
	libfreenect2::SyncMultiFrameListener * listener_;
	libfreenect2::Registration * reg_;
};

/////////////////////////
// CameraStereoDC1394
/////////////////////////
class DC1394Device;

class RTABMAP_EXP CameraStereoDC1394 :
	public CameraRGBD
{
public:
	static bool available();

public:
	CameraStereoDC1394( float imageRate=0.0f, const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraStereoDC1394();

	virtual bool init(const std::string & calibrationFolder = ".");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual void captureImage(cv::Mat & left, cv::Mat & right, float & fx, float & baseline, float & cx, float & cy);

private:
	DC1394Device *device_;
	StereoCameraModel stereoModel_;
};

/////////////////////////
// CameraStereoFlyCapture2
/////////////////////////
class RTABMAP_EXP CameraStereoFlyCapture2 :
	public CameraRGBD
{
public:
	static bool available();

public:
	CameraStereoFlyCapture2( float imageRate=0.0f, const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraStereoFlyCapture2();

	virtual bool init(const std::string & calibrationFolder = ".");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual void captureImage(cv::Mat & left, cv::Mat & right, float & fx, float & baseline, float & cx, float & cy);

private:
	FlyCapture2::Camera * camera_;
	void * triclopsCtx_; // TriclopsContext
};

} // namespace rtabmap
