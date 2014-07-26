/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include "rtabmap/core/Image.h"
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

public:
	CameraOpenNI2(float imageRate = 0,
					const Transform & localTransform = Transform::getIdentity(),
					float fx = 0.0f,
					float fy = 0.0f,
					float cx = 0.0f,
					float cy = 0.0f);
	virtual ~CameraOpenNI2();

	virtual bool init();

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
