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
	virtual bool init() = 0;

	//getters
	float getImageRate() const {return _imageRate;}
	const Transform & getLocalTransform() const {return _localTransform;}
	float getFx() const {return _fx;}
	float getFy() const {return _fy;}
	float getCx() const {return _cx;}
	float getCy() const {return _cy;}

	//setters
	void setImageRate(float imageRate) {_imageRate = imageRate;}
	void setLocalTransform(const Transform & localTransform) {_localTransform= localTransform;}
	void setFx(float fx) {_fx = fx;}
	void setFy(float fy) {_fy = fy;}
	void setCx(float cx) {_cx = cx;}
	void setCy(float cy) {_cy = cy;}

protected:
	/**
	 * Constructor
	 *
	 * @param imageRate : image/second , 0 for fast as the camera can
	 */
	CameraRGBD(float imageRate = 0,
				const Transform & localTransform = Transform::getIdentity(),
				float fx = 0.0f,
				float fy = 0.0f,
				float cx = 0.0f,
				float cy = 0.0f);

	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy) = 0;

private:
	float _imageRate;
	Transform _localTransform;
	UTimer * _frameRateTimer;
	float _fx;
	float _fy;
	float _cx;
	float _cy;
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
			const Transform & localTransform = Transform::getIdentity(),
			float fx = 0.0f,
			float fy = 0.0f,
			float cx = 0.0f,
			float cy = 0.0f);
	virtual ~CameraOpenni();

    void image_cb (
    		const boost::shared_ptr<openni_wrapper::Image>& rgb,
			const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
			float constant);

    bool init();

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
					const Transform & localTransform = Transform::getIdentity(),
					float fx = 0.0f,
					float fy = 0.0f,
					float cx = 0.0f,
					float cy = 0.0f);
	virtual ~CameraOpenNICV();

	virtual bool init();

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
	CameraOpenNI2(float imageRate = 0,
					const Transform & localTransform = Transform::getIdentity(),
					float fx = 0.0f,
					float fy = 0.0f,
					float cx = 0.0f,
					float cy = 0.0f);
	virtual ~CameraOpenNI2();

	virtual bool init();

	bool setAutoWhiteBalance(bool enabled);
	bool setAutoExposure(bool enabled);
	bool setExposure(int value);
	bool setGain(int value);

protected:
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy);

private:
	openni::Device * _device;
	openni::VideoStream * _color;
	openni::VideoStream * _depth;
	float _depthFx;
	float _depthFy;
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
					const Transform & localTransform = Transform::getIdentity(),
					float fx = 0.0f,
					float fy = 0.0f,
					float cx = 0.0f,
					float cy = 0.0f);
	virtual ~CameraFreenect();

	bool init();

protected:
	virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy);

private:
	int deviceId_;
	freenect_context * ctx_;
	FreenectDevice * freenectDevice_;
};

} // namespace rtabmap
