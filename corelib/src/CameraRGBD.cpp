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

#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/CameraRGB.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/Version.h"

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UMath.h>

#ifdef HAVE_OPENNI
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_image.h>
#endif

#ifdef RTABMAP_FREENECT
#include <libfreenect.h>
#ifdef FREENECT_DASH_INCLUDES
#include <libfreenect-registration.h>
#else
#include <libfreenect_registration.h>
#endif
#endif

#ifdef RTABMAP_FREENECT2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#endif

#ifdef RTABMAP_OPENNI2
#include <OniVersion.h>
#include <OpenNI.h>
#endif

namespace rtabmap
{

/////////////////////////
// CameraOpenNIPCL
/////////////////////////
CameraOpenni::CameraOpenni(const std::string & deviceId, float imageRate, const Transform & localTransform) :
		Camera(imageRate, localTransform),
		interface_(0),
		deviceId_(deviceId),
		depthConstant_(0.0f)
{
}

bool CameraOpenni::available() 
{
#ifdef HAVE_OPENNI
	return true;
#else
	return false;
#endif
}

CameraOpenni::~CameraOpenni()
{
#ifdef HAVE_OPENNI
	UDEBUG("");
	if(connection_.connected())
	{
		connection_.disconnect();
	}

	if(interface_)
	{
		interface_->stop();
		uSleep(1000); // make sure it is stopped
		delete interface_;
		interface_ = 0;
	}
#endif
}
#ifdef HAVE_OPENNI
void CameraOpenni::image_cb (
		const boost::shared_ptr<openni_wrapper::Image>& rgb,
		const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
		float constant)
{
	UScopeMutex s(dataMutex_);

	bool notify = rgb_.empty();

	cv::Mat rgbFrame(rgb->getHeight(), rgb->getWidth(), CV_8UC3);
	rgb->fillRGB(rgb->getWidth(), rgb->getHeight(), rgbFrame.data);
	cv::cvtColor(rgbFrame, rgb_, CV_RGB2BGR);

	depth_ = cv::Mat(rgb->getHeight(), rgb->getWidth(), CV_16UC1);
	depth->fillDepthImageRaw(rgb->getWidth(), rgb->getHeight(), (unsigned short*)depth_.data);

	depthConstant_ = constant;

	if(notify)
	{
		dataReady_.release();
	}
}
#endif

bool CameraOpenni::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef HAVE_OPENNI
	if(interface_)
	{
		interface_->stop();
		uSleep(100); // make sure it is stopped
		delete interface_;
		interface_ = 0;
	}

	try
	{
		if(UFile::getExtension(deviceId_).compare("oni") == 0)
		{
			interface_ = new pcl::ONIGrabber(deviceId_, false, true);
		}
		else
		{
			interface_ = new pcl::OpenNIGrabber(deviceId_);
		}

		boost::function<void (
				const boost::shared_ptr<openni_wrapper::Image>&,
				const boost::shared_ptr<openni_wrapper::DepthImage>&,
				float)> f = boost::bind (&CameraOpenni::image_cb, this, _1, _2, _3);
		connection_ = interface_->registerCallback (f);

		interface_->start ();
	}
	catch(const pcl::IOException& ex)
	{
		UERROR("OpenNI exception: %s", ex.what());
		if(interface_)
		{
			delete interface_;
			interface_ = 0;
		}
		return false;
	}
	return true;
#else
	UERROR("PCL not built with OpenNI! Cannot initialize CameraOpenNI");
	return false;
#endif
}

bool CameraOpenni::isCalibrated() const
{
#ifdef HAVE_OPENNI
	return true;
#else
	return false;
#endif
}

std::string CameraOpenni::getSerial() const
{
#ifdef HAVE_OPENNI
	if(interface_)
	{
		return interface_->getName();
	}
#endif
	return "";
}

SensorData CameraOpenni::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef HAVE_OPENNI
	if(interface_ && interface_->isRunning())
	{
		if(!dataReady_.acquire(1, 2000))
		{
			UWARN("Not received new frames since 2 seconds, end of stream reached!");
		}
		else
		{
			UScopeMutex s(dataMutex_);
			if(depthConstant_ && !rgb_.empty() && !depth_.empty())
			{
				CameraModel model(
						1.0f/depthConstant_, //fx
						1.0f/depthConstant_, //fy
						float(rgb_.cols/2) - 0.5f,  //cx
						float(rgb_.rows/2) - 0.5f,  //cy
						this->getLocalTransform(),
						0,
						rgb_.size());
				data = SensorData(rgb_, depth_, model, this->getNextSeqID(), UTimer::now());
			}

			depth_ = cv::Mat();
			rgb_ = cv::Mat();
			depthConstant_ = 0.0f;
		}
	}
#else
	UERROR("CameraOpenNI: RTAB-Map is not built with PCL having OpenNI support!");
#endif
	return data;
}



/////////////////////////
// CameraOpenNICV
/////////////////////////
bool CameraOpenNICV::available()
{
	return cv::getBuildInformation().find("OpenNI:                      YES") != std::string::npos;
}

CameraOpenNICV::CameraOpenNICV(bool asus, float imageRate, const rtabmap::Transform & localTransform) :
	Camera(imageRate, localTransform),
	_asus(asus),
	_depthFocal(0.0f)
{

}

CameraOpenNICV::~CameraOpenNICV()
{
	_capture.release();
}

bool CameraOpenNICV::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	if(_capture.isOpened())
	{
		_capture.release();
	}

	ULOGGER_DEBUG("Camera::init()");
	_capture.open( _asus?CV_CAP_OPENNI_ASUS:CV_CAP_OPENNI );
	if(_capture.isOpened())
	{
		_capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
		_depthFocal = _capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH );
		// Print some avalible device settings.
		UINFO("Depth generator output mode:");
		UINFO("FRAME_WIDTH        %f", _capture.get( CV_CAP_PROP_FRAME_WIDTH ));
		UINFO("FRAME_HEIGHT       %f", _capture.get( CV_CAP_PROP_FRAME_HEIGHT ));
		UINFO("FRAME_MAX_DEPTH    %f mm", _capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ));
		UINFO("BASELINE           %f mm", _capture.get( CV_CAP_PROP_OPENNI_BASELINE ));
		UINFO("FPS                %f", _capture.get( CV_CAP_PROP_FPS ));
		UINFO("Focal              %f", _capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH ));
		UINFO("REGISTRATION       %f", _capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ));
		if(_capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ) == 0.0)
		{
			UERROR("Depth registration is not activated on this device!");
		}
		if( _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) )
		{
			UINFO("Image generator output mode:");
			UINFO("FRAME_WIDTH    %f", _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ));
			UINFO("FRAME_HEIGHT   %f", _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ));
			UINFO("FPS            %f", _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ));
		}
		else
		{
			UERROR("Camera: Device doesn't contain image generator.");
			_capture.release();
			return false;
		}
	}
	else
	{
		ULOGGER_ERROR("Camera: Failed to create a capture object!");
		_capture.release();
		return false;
	}
	return true;
}

bool CameraOpenNICV::isCalibrated() const
{
	return true;
}

SensorData CameraOpenNICV::captureImage(CameraInfo * info)
{
	SensorData data;
	if(_capture.isOpened())
	{
		_capture.grab();
		cv::Mat depth, rgb;
		_capture.retrieve(depth, CV_CAP_OPENNI_DEPTH_MAP );
		_capture.retrieve(rgb, CV_CAP_OPENNI_BGR_IMAGE );

		depth = depth.clone();
		rgb = rgb.clone();

		UASSERT(_depthFocal>0.0f);
		if(!rgb.empty() && !depth.empty())
		{
			CameraModel model(
					_depthFocal, //fx
					_depthFocal, //fy
					float(rgb.cols/2) - 0.5f,  //cx
					float(rgb.rows/2) - 0.5f,  //cy
					this->getLocalTransform(),
					0,
					rgb.size());
			data = SensorData(rgb, depth, model, this->getNextSeqID(), UTimer::now());
		}
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}
	return data;
}


/////////////////////////
// CameraOpenNI2
/////////////////////////
bool CameraOpenNI2::available()
{
#ifdef RTABMAP_OPENNI2
	return true;
#else
	return false;
#endif
}

bool CameraOpenNI2::exposureGainAvailable()
{
#if ONI_VERSION_MAJOR > 2 || (ONI_VERSION_MAJOR==2 && ONI_VERSION_MINOR >= 2)
	return true;
#else
	return false;
#endif
}

CameraOpenNI2::CameraOpenNI2(
		const std::string & deviceId,
		float imageRate,
		const rtabmap::Transform & localTransform) :
	Camera(imageRate, localTransform),
#ifdef RTABMAP_OPENNI2
	_device(new openni::Device()),
	_color(new openni::VideoStream()),
	_depth(new openni::VideoStream()),
#else
	_device(0),
	_color(0),
	_depth(0),
#endif
	_depthFx(0.0f),
	_depthFy(0.0f),
	_deviceId(deviceId),
	_openNI2StampsAndIDsUsed(false)
{
}

CameraOpenNI2::~CameraOpenNI2()
{
#ifdef RTABMAP_OPENNI2
	_color->stop();
	_color->destroy();
	_depth->stop();
	_depth->destroy();
	_device->close();
	openni::OpenNI::shutdown();

	delete _device;
	delete _color;
	delete _depth;
#endif
}

bool CameraOpenNI2::setAutoWhiteBalance(bool enabled)
{
#ifdef RTABMAP_OPENNI2
	if(_color && _color->getCameraSettings())
	{
		return _color->getCameraSettings()->setAutoWhiteBalanceEnabled(enabled) == openni::STATUS_OK;
	}
#else
	UERROR("CameraOpenNI2: RTAB-Map is not built with OpenNI2 support!");
#endif
	return false;
}

bool CameraOpenNI2::setAutoExposure(bool enabled)
{
#ifdef RTABMAP_OPENNI2
	if(_color && _color->getCameraSettings())
	{
		return _color->getCameraSettings()->setAutoExposureEnabled(enabled) == openni::STATUS_OK;
	}
#else
	UERROR("CameraOpenNI2: RTAB-Map is not built with OpenNI2 support!");
#endif
	return false;
}

bool CameraOpenNI2::setExposure(int value)
{
#ifdef RTABMAP_OPENNI2
#if ONI_VERSION_MAJOR > 2 || (ONI_VERSION_MAJOR==2 && ONI_VERSION_MINOR >= 2)
	if(_color && _color->getCameraSettings())
	{
		return _color->getCameraSettings()->setExposure(value) == openni::STATUS_OK;
	}
#else
	UERROR("CameraOpenNI2: OpenNI >= 2.2 required to use this method.");
#endif
#else
	UERROR("CameraOpenNI2: RTAB-Map is not built with OpenNI2 support!");
#endif
	return false;
}

bool CameraOpenNI2::setGain(int value)
{
#ifdef RTABMAP_OPENNI2
#if ONI_VERSION_MAJOR > 2 || (ONI_VERSION_MAJOR==2 && ONI_VERSION_MINOR >= 2)
	if(_color && _color->getCameraSettings())
	{
		return _color->getCameraSettings()->setGain(value) == openni::STATUS_OK;
	}
#else
	UERROR("CameraOpenNI2: OpenNI >= 2.2 required to use this method.");
#endif
#else
	UERROR("CameraOpenNI2: RTAB-Map is not built with OpenNI2 support!");
#endif
	return false;
}

bool CameraOpenNI2::setMirroring(bool enabled)
{
#ifdef RTABMAP_OPENNI2
	if(_color->isValid() && _depth->isValid())
	{
		return _depth->setMirroringEnabled(enabled) == openni::STATUS_OK &&
				_color->setMirroringEnabled(enabled) == openni::STATUS_OK;
	}
#endif
	return false;
}

bool CameraOpenNI2::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_OPENNI2
	openni::OpenNI::initialize();

	if(_device->open(_deviceId.empty()?openni::ANY_DEVICE:_deviceId.c_str()) != openni::STATUS_OK)
	{
		if(!_deviceId.empty())
		{
			UERROR("CameraOpenNI2: Cannot open device \"%s\".", _deviceId.c_str());
		}
		else
		{
			UERROR("CameraOpenNI2: Cannot open device.");
		}
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(UFile::getExtension(_deviceId).compare("oni")==0)
	{
		if(_device->getPlaybackControl() &&
		   _device->getPlaybackControl()->setRepeatEnabled(false) != openni::STATUS_OK)
		{
			UERROR("CameraOpenNI2: Cannot set repeat mode to false.");
			_device->close();
			openni::OpenNI::shutdown();
			return false;
		}
	}
	else if(!_device->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		UERROR("CameraOpenNI2: Device doesn't support depth/color registration.");
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_device->getSensorInfo(openni::SENSOR_DEPTH) == NULL ||
	  _device->getSensorInfo(openni::SENSOR_COLOR) == NULL)
	{
		UERROR("CameraOpenNI2: Cannot get sensor info for depth and color.");
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_depth->create(*_device, openni::SENSOR_DEPTH) != openni::STATUS_OK)
	{
		UERROR("CameraOpenNI2: Cannot create depth stream.");
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_color->create(*_device, openni::SENSOR_COLOR) != openni::STATUS_OK)
	{
		UERROR("CameraOpenNI2: Cannot create color stream.");
		_depth->destroy();
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_device->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) != openni::STATUS_OK)
	{
		UERROR("CameraOpenNI2: Failed to set depth/color registration.");
	}

	if (_device->setDepthColorSyncEnabled(true) != openni::STATUS_OK)
	{
		UERROR("CameraOpenNI2: Failed to set depth color sync");
	}

	_depth->setMirroringEnabled(false);
	_color->setMirroringEnabled(false);

	const openni::Array<openni::VideoMode>& depthVideoModes = _depth->getSensorInfo().getSupportedVideoModes();
	for(int i=0; i<depthVideoModes.getSize(); ++i)
	{
		UINFO("CameraOpenNI2: Depth video mode %d: fps=%d, pixel=%d, w=%d, h=%d",
				i,
				depthVideoModes[i].getFps(),
				depthVideoModes[i].getPixelFormat(),
				depthVideoModes[i].getResolutionX(),
				depthVideoModes[i].getResolutionY());
	}

	const openni::Array<openni::VideoMode>& colorVideoModes = _color->getSensorInfo().getSupportedVideoModes();
	for(int i=0; i<colorVideoModes.getSize(); ++i)
	{
		UINFO("CameraOpenNI2: Color video mode %d: fps=%d, pixel=%d, w=%d, h=%d",
				i,
				colorVideoModes[i].getFps(),
				colorVideoModes[i].getPixelFormat(),
				colorVideoModes[i].getResolutionX(),
				colorVideoModes[i].getResolutionY());
	}

	openni::VideoMode mMode;
	mMode.setFps(30);
	mMode.setResolution(640,480);
	mMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	_depth->setVideoMode(mMode);

	openni::VideoMode mModeColor;
	mModeColor.setFps(30);
	mModeColor.setResolution(640,480);
	mModeColor.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	_color->setVideoMode(mModeColor);

	UINFO("CameraOpenNI2: Using depth video mode: fps=%d, pixel=%d, w=%d, h=%d, H-FOV=%f rad, V-FOV=%f rad",
			_depth->getVideoMode().getFps(),
			_depth->getVideoMode().getPixelFormat(),
			_depth->getVideoMode().getResolutionX(),
			_depth->getVideoMode().getResolutionY(),
			_depth->getHorizontalFieldOfView(),
			_depth->getVerticalFieldOfView());

	if(_color->getCameraSettings())
	{
		UINFO("CameraOpenNI2: AutoWhiteBalanceEnabled = %d", _color->getCameraSettings()->getAutoWhiteBalanceEnabled()?1:0);
		UINFO("CameraOpenNI2: AutoExposureEnabled = %d", _color->getCameraSettings()->getAutoExposureEnabled()?1:0);
#if ONI_VERSION_MAJOR > 2 || (ONI_VERSION_MAJOR==2 && ONI_VERSION_MINOR >= 2)
		UINFO("CameraOpenNI2: Exposure = %d", _color->getCameraSettings()->getExposure());
		UINFO("CameraOpenNI2: GAIN = %d", _color->getCameraSettings()->getGain());
#endif
	}

	bool registered = true;
	if(registered)
	{
		_depthFx = float(_color->getVideoMode().getResolutionX()/2) / std::tan(_color->getHorizontalFieldOfView()/2.0f);
		_depthFy = float(_color->getVideoMode().getResolutionY()/2) / std::tan(_color->getVerticalFieldOfView()/2.0f);
	}
	else
	{
		_depthFx = float(_depth->getVideoMode().getResolutionX()/2) / std::tan(_depth->getHorizontalFieldOfView()/2.0f);
		_depthFy = float(_depth->getVideoMode().getResolutionY()/2) / std::tan(_depth->getVerticalFieldOfView()/2.0f);
	}
	UINFO("depth fx=%f fy=%f", _depthFx, _depthFy);

	UINFO("CameraOpenNI2: Using color video mode: fps=%d, pixel=%d, w=%d, h=%d, H-FOV=%f rad, V-FOV=%f rad",
			_color->getVideoMode().getFps(),
			_color->getVideoMode().getPixelFormat(),
			_color->getVideoMode().getResolutionX(),
			_color->getVideoMode().getResolutionY(),
			_color->getHorizontalFieldOfView(),
			_color->getVerticalFieldOfView());

	if(_depth->start() != openni::STATUS_OK ||
	   _color->start() != openni::STATUS_OK)
	{
		UERROR("CameraOpenNI2: Cannot start depth and/or color streams.");
		_depth->stop();
		_color->stop();
		_depth->destroy();
		_color->destroy();
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	uSleep(1000); // just to make sure the sensor is correctly initialized

	return true;
#else
	UERROR("CameraOpenNI2: RTAB-Map is not built with OpenNI2 support!");
	return false;
#endif
}

bool CameraOpenNI2::isCalibrated() const
{
	return true;
}

std::string CameraOpenNI2::getSerial() const
{
#ifdef RTABMAP_OPENNI2
	if(_device)
	{
		return _device->getDeviceInfo().getName();
	}
#endif
	return "";
}

SensorData CameraOpenNI2::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_OPENNI2
	int readyStream = -1;
	if(_device->isValid() &&
		_depth->isValid() &&
		_color->isValid() &&
		_device->getSensorInfo(openni::SENSOR_DEPTH) != NULL &&
		_device->getSensorInfo(openni::SENSOR_COLOR) != NULL)
	{
		openni::VideoStream* depthStream[] = {_depth};
		openni::VideoStream* colorStream[] = {_color};
		if(openni::OpenNI::waitForAnyStream(depthStream, 1, &readyStream, 2000) != openni::STATUS_OK ||
		   openni::OpenNI::waitForAnyStream(colorStream, 1, &readyStream, 2000) != openni::STATUS_OK)
		{
			UWARN("No frames received since the last 2 seconds, end of stream is reached!");
		}
		else
		{
			openni::VideoFrameRef depthFrame, colorFrame;
			_depth->readFrame(&depthFrame);
			_color->readFrame(&colorFrame);
			cv::Mat depth, rgb;
			if(depthFrame.isValid() && colorFrame.isValid())
			{
				int h=depthFrame.getHeight();
				int w=depthFrame.getWidth();
				depth = cv::Mat(h, w, CV_16U, (void*)depthFrame.getData()).clone();

				h=colorFrame.getHeight();
				w=colorFrame.getWidth();
				cv::Mat tmp(h, w, CV_8UC3, (void *)colorFrame.getData());
				cv::cvtColor(tmp, rgb, CV_RGB2BGR);
			}
			UASSERT(_depthFx != 0.0f && _depthFy != 0.0f);
			if(!rgb.empty() && !depth.empty())
			{
				CameraModel model(
						_depthFx, //fx
						_depthFy, //fy
						float(rgb.cols/2) - 0.5f,  //cx
						float(rgb.rows/2) - 0.5f,  //cy
						this->getLocalTransform(),
						0,
						rgb.size());
				if(_openNI2StampsAndIDsUsed)
				{
					data = SensorData(rgb, depth, model, depthFrame.getFrameIndex(), double(depthFrame.getTimestamp()) / 1000000.0);
				}
				else
				{
					data = SensorData(rgb, depth, model, this->getNextSeqID(), UTimer::now());
				}
			}
		}
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}
#else
	UERROR("CameraOpenNI2: RTAB-Map is not built with OpenNI2 support!");
#endif
	return data;
}

#ifdef RTABMAP_FREENECT
//
// FreenectDevice
//
class FreenectDevice : public UThread {
  public:
	FreenectDevice(freenect_context * ctx, int index) :
		index_(index),
		ctx_(ctx),
		device_(0),
		depthFocal_(0.0f)
	{
		UASSERT(ctx_ != 0);
	}

	~FreenectDevice()
	{
		this->join(true);
		if(device_ && freenect_close_device(device_) < 0){} //FN_WARNING("Device did not shutdown in a clean fashion");
	}

	const std::string & getSerial() const {return serial_;}

	bool init()
	{
		if(device_)
		{
			this->join(true);
			freenect_close_device(device_);
			device_ = 0;
		}
		serial_.clear();
		std::vector<std::string> deviceSerials;
		freenect_device_attributes* attr_list;
		freenect_device_attributes* item;
		freenect_list_device_attributes(ctx_, &attr_list);
		for (item = attr_list; item != NULL; item = item->next) {
			deviceSerials.push_back(std::string(item->camera_serial));
		}
		freenect_free_device_attributes(attr_list);

		if(freenect_open_device(ctx_, &device_, index_) < 0)
		{
			UERROR("FreenectDevice: Cannot open Kinect");
			return false;
		}

		if(index_ >= 0 && index_ < (int)deviceSerials.size())
		{
			serial_ = deviceSerials[index_];
		}
		else
		{
			UERROR("Could not get serial for index %d", index_);
		}

		freenect_set_user(device_, this);
		freenect_set_video_mode(device_, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
		freenect_set_depth_mode(device_, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED));
		depthBuffer_ = cv::Mat(cv::Size(640,480),CV_16UC1);
		rgbBuffer_ = cv::Mat(cv::Size(640,480), CV_8UC3);
		freenect_set_depth_buffer(device_, depthBuffer_.data);
		freenect_set_video_buffer(device_, rgbBuffer_.data);
		freenect_set_depth_callback(device_, freenect_depth_callback);
		freenect_set_video_callback(device_, freenect_video_callback);

		bool registered = true;
		float rgb_focal_length_sxga = 1050.0f;
		float width_sxga = 1280.0f;
		float width = freenect_get_current_depth_mode(device_).width;
		float scale = width / width_sxga;
		if(registered)
		{
			depthFocal_ =  rgb_focal_length_sxga * scale;
		}
		else
		{
			freenect_registration reg = freenect_copy_registration(device_);
			float depth_focal_length_sxga = reg.zero_plane_info.reference_distance / reg.zero_plane_info.reference_pixel_size;
			freenect_destroy_registration(&reg);

			depthFocal_ =  depth_focal_length_sxga * scale;
		}

		UINFO("FreenectDevice: Depth focal = %f", depthFocal_);
		return true;
	}

	float getDepthFocal() const {return depthFocal_;}

	void getData(cv::Mat & rgb, cv::Mat & depth)
	{
		if(this->isRunning())
		{
			if(!dataReady_.acquire(1, 2000))
			{
				UERROR("Not received any frames since 2 seconds, try to restart the camera again.");
			}
			else
			{
				UScopeMutex s(dataMutex_);
				rgb = rgbLastFrame_;
				depth = depthLastFrame_;
				rgbLastFrame_ = cv::Mat();
				depthLastFrame_= cv::Mat();
			}
		}
	}

private:
	// Do not call directly even in child
	void VideoCallback(void* rgb)
	{
		UASSERT(rgbBuffer_.data == rgb);
		UScopeMutex s(dataMutex_);
		bool notify = rgbLastFrame_.empty();
		cv::cvtColor(rgbBuffer_, rgbLastFrame_, CV_RGB2BGR);
		if(!depthLastFrame_.empty() && notify)
		{
			dataReady_.release();
		}
	}

	// Do not call directly even in child
	void DepthCallback(void* depth)
	{
		UASSERT(depthBuffer_.data == depth);
		UScopeMutex s(dataMutex_);
		bool notify = depthLastFrame_.empty();
		depthLastFrame_ = depthBuffer_.clone();
		if(!rgbLastFrame_.empty() && notify)
		{
			dataReady_.release();
		}
	}

	void startVideo() {
		if(device_ && freenect_start_video(device_) < 0) UERROR("Cannot start RGB callback");
	}
	void stopVideo() {
		if(device_ && freenect_stop_video(device_) < 0) UERROR("Cannot stop RGB callback");
	}
	void startDepth() {
		if(device_ && freenect_start_depth(device_) < 0) UERROR("Cannot start depth callback");
	}
	void stopDepth() {
		if(device_ && freenect_stop_depth(device_) < 0) UERROR("Cannot stop depth callback");
	}

	virtual void mainLoopBegin()
	{
		this->startDepth();
		this->startVideo();
	}

	virtual void mainLoop()
	{
		timeval t;
		t.tv_sec = 0;
		t.tv_usec = 10000;
		if(freenect_process_events_timeout(ctx_, &t) < 0)
		{
			UERROR("FreenectDevice: Cannot process freenect events");
			this->kill();
		}
	}

	virtual void mainLoopEnd()
	{
		this->stopDepth();
		this->stopVideo();
		dataReady_.release();
	}

	static void freenect_depth_callback(freenect_device *dev, void *depth, uint32_t timestamp) {
		FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
		device->DepthCallback(depth);
	}
	static void freenect_video_callback(freenect_device *dev, void *video, uint32_t timestamp) {
		FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
		device->VideoCallback(video);
	}

	//noncopyable
	FreenectDevice( const FreenectDevice& );
	const FreenectDevice& operator=( const FreenectDevice& );

  private:
	int index_;
	std::string serial_;
	freenect_context * ctx_;
	freenect_device * device_;
	cv::Mat depthBuffer_;
	cv::Mat rgbBuffer_;
	UMutex dataMutex_;
	cv::Mat depthLastFrame_;
	cv::Mat rgbLastFrame_;
	float depthFocal_;
	USemaphore dataReady_;
};
#endif

//
// CameraFreenect
//
bool CameraFreenect::available()
{
#ifdef RTABMAP_FREENECT
	return true;
#else
	return false;
#endif
}

CameraFreenect::CameraFreenect(int deviceId, float imageRate, const Transform & localTransform) :
		Camera(imageRate, localTransform),
		deviceId_(deviceId),
		ctx_(0),
		freenectDevice_(0)
{
#ifdef RTABMAP_FREENECT
	if(freenect_init(&ctx_, NULL) < 0) UERROR("Cannot initialize freenect library");
	// claim camera
	freenect_select_subdevices(ctx_, static_cast<freenect_device_flags>(FREENECT_DEVICE_CAMERA));
#endif
}

CameraFreenect::~CameraFreenect()
{
#ifdef RTABMAP_FREENECT
	if(freenectDevice_)
	{
		freenectDevice_->join(true);
		delete freenectDevice_;
		freenectDevice_ = 0;
	}
	if(ctx_)
	{
		if(freenect_shutdown(ctx_) < 0){} //FN_WARNING("Freenect did not shutdown in a clean fashion");
	}
#endif
}

bool CameraFreenect::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_FREENECT
	if(freenectDevice_)
	{
		freenectDevice_->join(true);
		delete freenectDevice_;
		freenectDevice_ = 0;
	}

	if(ctx_ && freenect_num_devices(ctx_) > 0)
	{
		freenectDevice_ = new FreenectDevice(ctx_, deviceId_);
		if(freenectDevice_->init())
		{
			freenectDevice_->start();
			uSleep(3000);
			return true;
		}
		else
		{
			UERROR("CameraFreenect: Init failed!");
		}
		delete freenectDevice_;
		freenectDevice_ = 0;
	}
	else
	{
		UERROR("CameraFreenect: No devices connected!");
	}
#else
	UERROR("CameraFreenect: RTAB-Map is not built with Freenect support!");
#endif
	return false;
}

bool CameraFreenect::isCalibrated() const
{
	return true;
}

std::string CameraFreenect::getSerial() const
{
#ifdef RTABMAP_FREENECT
	if(freenectDevice_)
	{
		return freenectDevice_->getSerial();
	}
#endif
	return "";
}

SensorData CameraFreenect::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_FREENECT
	if(ctx_ && freenectDevice_)
	{
		if(freenectDevice_->isRunning())
		{
			cv::Mat depth,rgb;
			freenectDevice_->getData(rgb, depth);
			if(!rgb.empty() && !depth.empty())
			{
				UASSERT(freenectDevice_->getDepthFocal() != 0.0f);
				if(!rgb.empty() && !depth.empty())
				{
					CameraModel model(
							freenectDevice_->getDepthFocal(), //fx
							freenectDevice_->getDepthFocal(), //fy
							float(rgb.cols/2) - 0.5f,  //cx
							float(rgb.rows/2) - 0.5f,  //cy
							this->getLocalTransform(),
							0,
							rgb.size());
					data = SensorData(rgb, depth, model, this->getNextSeqID(), UTimer::now());
				}
			}
		}
		else
		{
			UERROR("CameraFreenect: Re-initialization needed!");
			delete freenectDevice_;
			freenectDevice_ = 0;
		}
	}
#else
	UERROR("CameraFreenect: RTAB-Map is not built with Freenect support!");
#endif
	return data;
}

//
// CameraFreenect2
//
bool CameraFreenect2::available()
{
#ifdef RTABMAP_FREENECT2
	return true;
#else
	return false;
#endif
}

CameraFreenect2::CameraFreenect2(
	int deviceId, 
	Type type, 
	float imageRate, 
	const Transform & localTransform, 
	float minDepth,
	float maxDepth,
	bool bilateralFiltering,
	bool edgeAwareFiltering,
	bool noiseFiltering) :
		Camera(imageRate, localTransform),
		deviceId_(deviceId),
		type_(type),
		freenect2_(0),
		dev_(0),
		listener_(0),
		reg_(0),
		minKinect2Depth_(minDepth),
		maxKinect2Depth_(maxDepth),
		bilateralFiltering_(bilateralFiltering),
		edgeAwareFiltering_(edgeAwareFiltering),
		noiseFiltering_(noiseFiltering)
{
#ifdef RTABMAP_FREENECT2
	UASSERT(minKinect2Depth_ < maxKinect2Depth_ && minKinect2Depth_>0 && maxKinect2Depth_>0 && maxKinect2Depth_<=65.535f);
	freenect2_ = new libfreenect2::Freenect2();
	switch(type_)
	{
	case kTypeColorIR:
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir);
		break;
	case kTypeIRDepth:
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
		break;
	case kTypeColor2DepthSD:
	case kTypeDepth2ColorHD:
	case kTypeDepth2ColorSD:
	default:
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color  | libfreenect2::Frame::Depth);
		break;
	}
#endif
}

CameraFreenect2::~CameraFreenect2()
{
#ifdef RTABMAP_FREENECT2
	UDEBUG("");
	if(dev_)
	{
		dev_->stop();
		dev_->close();
		//deleted in freenect2_ destructor (Freeenect2Impl::clearDevices())
	}
	if(listener_)
	{
		delete listener_;
	}

	if(reg_)
	{
		delete reg_;
		reg_ = 0;
	}

	if(freenect2_)
	{
		delete freenect2_;
	}
	UDEBUG("");
#endif
}

bool CameraFreenect2::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_FREENECT2
	if(dev_)
	{
		dev_->stop();
		dev_->close();
		dev_ = 0; //deleted in freenect2_ destructor (Freeenect2Impl::clearDevices())
	}

	if(reg_)
	{
		delete reg_;
		reg_ = 0;
	}

	libfreenect2::PacketPipeline * pipeline;
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
	pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
	pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
	pipeline = new libfreenect2::CpuPacketPipeline();
#endif
#endif

	if(deviceId_ <= 0)
	{
		UDEBUG("Opening default device...");
		dev_ = freenect2_->openDefaultDevice(pipeline);
		pipeline = 0;// pipeline deleted in dev_ (Freenect2DeviceImpl::~Freenect2DeviceImpl())
	}
	else
	{
		UDEBUG("Opening device ID=%d...", deviceId_);
		dev_ = freenect2_->openDevice(deviceId_, pipeline);
		pipeline = 0;// pipeline deleted in dev_ (Freenect2DeviceImpl::~Freenect2DeviceImpl())
	}

	if(dev_)
	{
		//default
		//MinDepth(0.5f),
		//MaxDepth(4.5f),
		//EnableBilateralFilter(true),
		//EnableEdgeAwareFilter(true)
		libfreenect2::Freenect2Device::Config config;
		config.EnableBilateralFilter = bilateralFiltering_;
		config.EnableEdgeAwareFilter = edgeAwareFiltering_;
		config.MinDepth = minKinect2Depth_;
		config.MaxDepth = maxKinect2Depth_;
		dev_->setConfiguration(config);

		dev_->setColorFrameListener(listener_);
		dev_->setIrAndDepthFrameListener(listener_);

		dev_->start();

		UINFO("CameraFreenect2: device serial: %s", dev_->getSerialNumber().c_str());
		UINFO("CameraFreenect2: device firmware: %s", dev_->getFirmwareVersion().c_str());

		//default registration params
		libfreenect2::Freenect2Device::IrCameraParams depthParams = dev_->getIrCameraParams();
		libfreenect2::Freenect2Device::ColorCameraParams colorParams = dev_->getColorCameraParams();
		reg_ = new libfreenect2::Registration(depthParams, colorParams);

		// look for calibration files
		if(!calibrationFolder.empty())
		{
			std::string calibrationName = dev_->getSerialNumber();
			if(!cameraName.empty())
			{
				calibrationName = cameraName;
			}
			if(!stereoModel_.load(calibrationFolder, calibrationName, false))
			{
				UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, default calibration used.",
						calibrationName.c_str(), calibrationFolder.c_str());
			}
			else
			{
				if(type_==kTypeColor2DepthSD)
				{
					UWARN("Freenect2: When using custom calibration file, type "
						  "kTypeColor2DepthSD is not supported. kTypeDepth2ColorSD is used instead...");
					type_ = kTypeDepth2ColorSD;
				}

				// downscale color image by 2
				cv::Mat colorP = stereoModel_.right().P();
				cv::Size colorSize = stereoModel_.right().imageSize();
				if(type_ == kTypeDepth2ColorSD)
				{
					colorP.at<double>(0,0)/=2.0f; //fx
					colorP.at<double>(1,1)/=2.0f; //fy
					colorP.at<double>(0,2)/=2.0f; //cx
					colorP.at<double>(1,2)/=2.0f; //cy
					colorSize.width/=2;
					colorSize.height/=2;
				}
				cv::Mat depthP = stereoModel_.left().P();
				cv::Size depthSize = stereoModel_.left().imageSize();
				float ratioY = float(colorSize.height)/float(depthSize.height);
				float ratioX = float(colorSize.width)/float(depthSize.width);
				depthP.at<double>(0,0)*=ratioX; //fx
				depthP.at<double>(1,1)*=ratioY; //fy
				depthP.at<double>(0,2)*=ratioX; //cx
				depthP.at<double>(1,2)*=ratioY; //cy
				depthSize.width*=ratioX;
				depthSize.height*=ratioY;
				const CameraModel & l = stereoModel_.left();
				const CameraModel & r = stereoModel_.right();
				stereoModel_ = StereoCameraModel(stereoModel_.name(),
						depthSize, l.K_raw(), l.D_raw(), l.R(), depthP,
						colorSize, r.K_raw(), r.D_raw(), r.R(), colorP,
						stereoModel_.R(), stereoModel_.T(), stereoModel_.E(), stereoModel_.F());
				stereoModel_.initRectificationMap();
			}
		}

		return true;
	}
	else
	{
		UERROR("CameraFreenect2: no device connected or failure opening the default one! Note that rtabmap should link on libusb of libfreenect2. "
					"Tip, before starting rtabmap: \"$ export LD_LIBRARY_PATH=~/libfreenect2/depends/libusb/lib:$LD_LIBRARY_PATH\"");
	}
#else
	UERROR("CameraFreenect2: RTAB-Map is not built with Freenect2 support!");
#endif
	return false;
}

bool CameraFreenect2::isCalibrated() const
{
	return true;
}

std::string CameraFreenect2::getSerial() const
{
#ifdef RTABMAP_FREENECT2
	if(dev_)
	{
		return dev_->getSerialNumber();
	}
#endif
	return "";
}

SensorData CameraFreenect2::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_FREENECT2
	if(dev_ && listener_)
	{
		libfreenect2::FrameMap frames;
#ifndef LIBFREENECT2_THREADING_STDLIB
		UDEBUG("Waiting for new frames... If it is stalled here, rtabmap should link on libusb of libfreenect2. "
				"Tip, before starting rtabmap: \"$ export LD_LIBRARY_PATH=~/libfreenect2/depends/libusb/lib:$LD_LIBRARY_PATH\"");
		listener_->waitForNewFrame(frames);
#else
		if(!listener_->waitForNewFrame(frames, 1000))
		{
			UWARN("CameraFreenect2: Failed to get frames! rtabmap should link on libusb of libfreenect2. "
					"Tip, before starting rtabmap: \"$ export LD_LIBRARY_PATH=~/libfreenect2/depends/libusb/lib:$LD_LIBRARY_PATH\"");
		}
		else
#endif
		{
			double stamp = UTimer::now();
			libfreenect2::Frame *rgbFrame = 0;
			libfreenect2::Frame *irFrame = 0;
			libfreenect2::Frame *depthFrame = 0;

			switch(type_)
			{
			case kTypeColorIR: //used for calibration
				rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
				irFrame = uValue(frames, libfreenect2::Frame::Ir, (libfreenect2::Frame*)0);
				break;
			case kTypeIRDepth:
				irFrame = uValue(frames, libfreenect2::Frame::Ir, (libfreenect2::Frame*)0);
				depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
				break;
			case kTypeColor2DepthSD:
			case kTypeDepth2ColorSD:
			case kTypeDepth2ColorHD:
			case kTypeDepth2ColorHD2:
			default:
				rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
				depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
				break;
			}

			cv::Mat rgb, depth;
			float fx=0,fy=0,cx=0,cy=0;
			if(irFrame && depthFrame)
			{
				cv::Mat irMat((int)irFrame->height, (int)irFrame->width, CV_32FC1, irFrame->data);
				//convert to gray scaled
				float maxIr_ = 0x7FFF;
				float minIr_ = 0x0;
				const float factor = 255.0f / float((maxIr_ - minIr_));
				rgb = cv::Mat(irMat.rows, irMat.cols, CV_8UC1);
				for(int i=0; i<irMat.rows; ++i)
				{
					for(int j=0; j<irMat.cols; ++j)
					{
						rgb.at<unsigned char>(i, j) = (unsigned char)std::min(float(std::max(irMat.at<float>(i,j) - minIr_, 0.0f)) * factor, 255.0f);
					}
				}

				cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data).convertTo(depth, CV_16U, 1);

				cv::flip(rgb, rgb, 1);
				cv::flip(depth, depth, 1);

				if(stereoModel_.isValidForRectification())
				{
					//rectify
					rgb = stereoModel_.left().rectifyImage(rgb);
					depth = stereoModel_.left().rectifyImage(depth);
					fx = stereoModel_.left().fx();
					fy = stereoModel_.left().fy();
					cx = stereoModel_.left().cx();
					cy = stereoModel_.left().cy();
				}
				else
				{
					libfreenect2::Freenect2Device::IrCameraParams params = dev_->getIrCameraParams();
					fx = params.fx;
					fy = params.fy;
					cx = params.cx;
					cy = params.cy;
				}
			}
			else
			{
				//rgb + ir or rgb + depth
				if(stereoModel_.isValidForRectification())
				{
					cv::Mat rgbMatC4((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data);
					cv::Mat rgbMat; // rtabmap uses 3 channels RGB
					cv::cvtColor(rgbMatC4, rgbMat, CV_BGRA2BGR);
					cv::flip(rgbMat, rgb, 1);

					//rectify color
					rgb = stereoModel_.right().rectifyImage(rgb);
					if(irFrame)
					{
						//rectify IR
						cv::Mat((int)irFrame->height, (int)irFrame->width, CV_32FC1, irFrame->data).convertTo(depth, CV_16U, 1);
						cv::flip(depth, depth, 1);
						depth = stereoModel_.left().rectifyImage(depth);
					}
					else
					{
						//rectify depth
						cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data).convertTo(depth, CV_16U, 1);
						cv::flip(depth, depth, 1);
						depth = stereoModel_.left().rectifyDepth(depth);

						bool registered = true;
						if(registered)
						{
							depth = util2d::registerDepth(
									depth,
									stereoModel_.left().P().colRange(0,3).rowRange(0,3), //scaled depth K
									stereoModel_.right().P().colRange(0,3).rowRange(0,3), //scaled color K
									stereoModel_.stereoTransform());
							util2d::fillRegisteredDepthHoles(depth, true, false);
							fx = stereoModel_.right().fx();
							fy = stereoModel_.right().fy();
							cx = stereoModel_.right().cx();
							cy = stereoModel_.right().cy();
						}
						else
						{
							fx = stereoModel_.left().fx();
							fy = stereoModel_.left().fy();
							cx = stereoModel_.left().cx();
							cy = stereoModel_.left().cy();
						}
					}
				}
				else
				{
					//use data from libfreenect2
					if(irFrame)
					{
						cv::Mat rgbMatC4((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data);
						cv::Mat rgbMat; // rtabmap uses 3 channels RGB
						cv::cvtColor(rgbMatC4, rgbMat, CV_BGRA2BGR);
						cv::flip(rgbMat, rgb, 1);

						cv::Mat((int)irFrame->height, (int)irFrame->width, CV_32FC1, irFrame->data).convertTo(depth, CV_16U, 1);
						cv::flip(depth, depth, 1);
					}
					else
					{
						//registration of the depth
						UASSERT(reg_!=0);

						float maxDepth = maxKinect2Depth_*1000.0f;
						float minDepth =  minKinect2Depth_*1000.0f;
						if(type_ == kTypeColor2DepthSD || type_ == kTypeDepth2ColorHD)
						{
							cv::Mat rgbMatBGRA;
							libfreenect2::Frame depthUndistorted(512, 424, 4);
							libfreenect2::Frame rgbRegistered(512, 424, 4);

							// do it before registration
							if(noiseFiltering_)
							{
								cv::Mat depthMat = cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data);
								for(int dx=0; dx<depthMat.cols; ++dx)
								{
									bool onEdgeX = dx==depthMat.cols-1;
									for(int dy=0; dy<depthMat.rows; ++dy)
									{
										bool onEdge = onEdgeX || dy==depthMat.rows-1;
										float z = 0.0f;
										float & dz = depthMat.at<float>(dy,dx);
										if(dz>=minDepth && dz <= maxDepth)
										{
											z = dz;
											if(noiseFiltering_ && !onEdge)
											{
												z=0;
												const float & dz1 = depthMat.at<float>(dy,dx+1);
												const float & dz2 = depthMat.at<float>(dy+1,dx);
												const float & dz3 = depthMat.at<float>(dy+1,dx+1);
												if( dz1>=minDepth && dz1 <= maxDepth &&
													dz2>=minDepth && dz2 <= maxDepth &&
													dz3>=minDepth && dz3 <= maxDepth)
												{
													float avg = (dz + dz1 + dz2 + dz3) / 4.0f;
													float thres = 0.01f*avg;
											
													if( fabs(dz-avg) < thres &&
														fabs(dz1-avg) < thres &&
														fabs(dz2-avg) < thres &&
														fabs(dz3-avg) < thres)
													{
														z = dz;
													}
												}
											}
										}
										dz = z; 
									}
								}
							}
	
							libfreenect2::Frame bidDepth(1920, 1082, 4); // HD
							reg_->apply(rgbFrame, depthFrame, &depthUndistorted, &rgbRegistered, true, &bidDepth);
	
							cv::Mat depthMat;
							if(type_ == kTypeColor2DepthSD)
							{						
								rgbMatBGRA = cv::Mat((int)rgbRegistered.height, (int)rgbRegistered.width, CV_8UC4, rgbRegistered.data);
								depthMat = cv::Mat((int)depthUndistorted.height, (int)depthUndistorted.width, CV_32FC1, depthUndistorted.data);
						
								//use IR params
								libfreenect2::Freenect2Device::IrCameraParams params = dev_->getIrCameraParams();
								fx = params.fx;
								fy = params.fy;
								cx = params.cx;
								cy = params.cy;
							}
							else
							{
								rgbMatBGRA = cv::Mat((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data);
								depthMat = cv::Mat((int)bidDepth.height, (int)bidDepth.width, CV_32FC1, bidDepth.data);
								depthMat = depthMat(cv::Range(1, 1081), cv::Range::all());
								
								//use color params
								libfreenect2::Freenect2Device::ColorCameraParams params = dev_->getColorCameraParams();
								fx = params.fx;
								fy = params.fy;
								cx = params.cx;
								cy = params.cy;
							}

							//filter max depth and flip
							depth = cv::Mat(depthMat.size(), CV_16UC1);
							for(int dx=0; dx<depthMat.cols; ++dx)
							{
								for(int dy=0; dy<depthMat.rows; ++dy)
								{
									unsigned short z = 0;
									const float & dz = depthMat.at<float>(dy,dx);
									if(dz>=minDepth && dz <= maxDepth)
									{
										z = (unsigned short)dz;
									}
									depth.at<unsigned short>(dy,(depthMat.cols-1)-dx) = z;  //flip
								}
							}
	
							// rtabmap uses 3 channels RGB
							cv::cvtColor(rgbMatBGRA, rgb, CV_BGRA2BGR);
							cv::flip(rgb, rgb, 1);
						}
						else //register depth to color (OLD WAY)
						{
							UASSERT(type_ == kTypeDepth2ColorSD || type_ == kTypeDepth2ColorHD2);
							cv::Mat rgbMatBGRA = cv::Mat((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data);
							if(type_ == kTypeDepth2ColorSD)
							{
								cv::Mat tmp;
								cv::resize(rgbMatBGRA, tmp, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
								rgbMatBGRA = tmp;
							}
							// rtabmap uses 3 channels RGB
							cv::cvtColor(rgbMatBGRA, rgb, CV_BGRA2BGR);
							cv::flip(rgb, rgb, 1);	

							cv::Mat depthFrameMat = cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data);
							depth = cv::Mat::zeros(rgbMatBGRA.rows, rgbMatBGRA.cols, CV_16U);
							for(int dx=0; dx<depthFrameMat.cols-1; ++dx)
							{
								for(int dy=0; dy<depthFrameMat.rows-1; ++dy)
								{
									float dz = depthFrameMat.at<float>(dy,dx);
									if(dz>=minDepth && dz<=maxDepth)
									{
										bool goodDepth = true;
										if(noiseFiltering_)
										{
											goodDepth = false;
											float dz1 = depthFrameMat.at<float>(dy,dx+1);
											float dz2 = depthFrameMat.at<float>(dy+1,dx);
											float dz3 = depthFrameMat.at<float>(dy+1,dx+1);
											if(dz1>=minDepth && dz1 <= maxDepth &&
											   dz2>=minDepth && dz2 <= maxDepth &&
											   dz3>=minDepth && dz3 <= maxDepth)
											{
												float avg = (dz + dz1 + dz2 + dz3) / 4.0f;
												float thres = 0.01 * avg;
												if( fabs(dz-avg) < thres &&
													fabs(dz1-avg) < thres &&
													fabs(dz2-avg) < thres &&
													fabs(dz3-avg) < thres)
												{
													goodDepth = true;
												}
											}
										}
										if(goodDepth)
										{
											float cx=-1,cy=-1;
											reg_->apply(dx, dy, dz, cx, cy);
											if(type_ == kTypeDepth2ColorSD)
											{
												cx/=2.0f;
												cy/=2.0f;
											}
											int rcx = cvRound(cx);
											int rcy = cvRound(cy);
											if(uIsInBounds(rcx, 0, depth.cols) && uIsInBounds(rcy, 0, depth.rows))
											{
												unsigned short & zReg = depth.at<unsigned short>(rcy, rcx);
												if(zReg == 0 || zReg > (unsigned short)dz)
												{
													zReg = (unsigned short)dz;
												}
											}
										}
									}
								}
							}
							util2d::fillRegisteredDepthHoles(depth, true, true, type_==kTypeDepth2ColorHD2);
							util2d::fillRegisteredDepthHoles(depth, type_==kTypeDepth2ColorSD, type_==kTypeDepth2ColorHD2);//second pass
							cv::flip(depth, depth, 1);
							libfreenect2::Freenect2Device::ColorCameraParams params = dev_->getColorCameraParams();
							fx = params.fx*(type_==kTypeDepth2ColorSD?0.5:1.0f);
							fy = params.fy*(type_==kTypeDepth2ColorSD?0.5:1.0f);
							cx = params.cx*(type_==kTypeDepth2ColorSD?0.5:1.0f);
							cy = params.cy*(type_==kTypeDepth2ColorSD?0.5:1.0f);
						}
					}
				}
			}

			CameraModel model;
			if(fx && fy)
			{
				model=CameraModel(
						fx, //fx
						fy, //fy
						cx,  //cx
						cy, // cy
						this->getLocalTransform(),
						0,
						rgb.size());
			}
			data = SensorData(rgb, depth, model, this->getNextSeqID(), stamp);

			listener_->release(frames);
		}
	}
#else
	UERROR("CameraFreenect2: RTAB-Map is not built with Freenect2 support!");
#endif
	return data;
}

//
// CameraRGBDImages
//
bool CameraRGBDImages::available()
{
	return true;
}

CameraRGBDImages::CameraRGBDImages(
		const std::string & pathRGBImages,
		const std::string & pathDepthImages,
		float depthScaleFactor,
		float imageRate,
		const Transform & localTransform) :
		CameraImages(pathRGBImages, imageRate, localTransform)
{
	UASSERT(depthScaleFactor >= 1.0);
	cameraDepth_.setPath(pathDepthImages);
	cameraDepth_.setDepth(true, depthScaleFactor);
}

CameraRGBDImages::~CameraRGBDImages()
{
}

bool CameraRGBDImages::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	bool success = false;
	if(CameraImages::init(calibrationFolder, cameraName) && cameraDepth_.init())
	{
		if(this->imagesCount() == cameraDepth_.imagesCount())
		{
			success = true;
		}
		else
		{
			UERROR("Cameras don't have the same number of images (%d vs %d)",
					this->imagesCount(), cameraDepth_.imagesCount());
		}
	}

	return success;
}

bool CameraRGBDImages::isCalibrated() const
{
	return this->cameraModel().isValidForProjection();
}

std::string CameraRGBDImages::getSerial() const
{
	return this->cameraModel().name();
}

SensorData CameraRGBDImages::captureImage(CameraInfo * info)
{
	SensorData data;

	SensorData rgb, depth;
	rgb = CameraImages::captureImage(info);
	if(!rgb.imageRaw().empty())
	{
		depth = cameraDepth_.takeImage();
		if(!depth.depthRaw().empty())
		{
			data = SensorData(rgb.imageRaw(), depth.depthRaw(), rgb.cameraModels(), rgb.id(), rgb.stamp());
			data.setGroundTruth(rgb.groundTruth());
		}
	}
	return data;
}


} // namespace rtabmap
