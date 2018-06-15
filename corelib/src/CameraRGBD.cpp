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

#ifdef RTABMAP_OPENNI
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

#ifdef RTABMAP_K4W2
#include <Kinect.h>
#endif

#ifdef RTABMAP_REALSENSE
#include <librealsense/rs.hpp>
#ifdef RTABMAP_REALSENSE_SLAM
#include <rs_core.h>
#include <rs_utils.h>
#include <librealsense/slam/slam.h>
#endif
#endif

#ifdef RTABMAP_REALSENSE2
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
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
#ifdef RTABMAP_OPENNI
	return true;
#else
	return false;
#endif
}

CameraOpenni::~CameraOpenni()
{
#ifdef RTABMAP_OPENNI
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
#ifdef RTABMAP_OPENNI
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
#ifdef RTABMAP_OPENNI
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
#ifdef RTABMAP_OPENNI
	return true;
#else
	return false;
#endif
}

std::string CameraOpenni::getSerial() const
{
#ifdef RTABMAP_OPENNI
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
#ifdef RTABMAP_OPENNI
	if(interface_ && interface_->isRunning())
	{
		if(!dataReady_.acquire(1, 5000))
		{
			UWARN("Not received new frames since 5 seconds, end of stream reached!");
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
		Type type,
		float imageRate,
		const rtabmap::Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_OPENNI2
    ,
	_type(type),
	_device(new openni::Device()),
	_color(new openni::VideoStream()),
	_depth(new openni::VideoStream()),
	_depthFx(0.0f),
	_depthFy(0.0f),
	_deviceId(deviceId),
	_openNI2StampsAndIDsUsed(false),
	_depthHShift(0),
	_depthVShift(0)
#endif
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

void CameraOpenNI2::setOpenNI2StampsAndIDsUsed(bool used)
{
#ifdef RTABMAP_OPENNI2
	_openNI2StampsAndIDsUsed = used;
#endif
}

void CameraOpenNI2::setIRDepthShift(int horizontal, int vertical)
{
#ifdef RTABMAP_OPENNI2
	UASSERT(horizontal >= 0);
	UASSERT(vertical >= 0);
	_depthHShift = horizontal;
	_depthVShift = vertical;
#endif
}

bool CameraOpenNI2::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_OPENNI2
	openni::OpenNI::initialize();

	openni::Array<openni::DeviceInfo> devices;
	openni::OpenNI::enumerateDevices(&devices);
	for(int i=0; i<devices.getSize(); ++i)
	{
		UINFO("Device %d: Name=%s URI=%s Vendor=%s",
				i,
				devices[i].getName(),
				devices[i].getUri(),
				devices[i].getVendor());
	}
	if(_deviceId.empty() && devices.getSize() == 0)
	{
		UERROR("CameraOpenNI2: No device detected!");
		return false;
	}

	openni::Status error = _device->open(_deviceId.empty()?openni::ANY_DEVICE:_deviceId.c_str());
	if(error != openni::STATUS_OK)
	{
		if(!_deviceId.empty())
		{
			UERROR("CameraOpenNI2: Cannot open device \"%s\" (error=%d).", _deviceId.c_str(), error);
		}
		else
		{
#ifdef _WIN32
			UERROR("CameraOpenNI2: Cannot open device \"%s\" (error=%d).", devices[0].getName(), error);
#else
			UERROR("CameraOpenNI2: Cannot open device \"%s\" (error=%d). Verify if \"%s\" is in udev rules: \"/lib/udev/rules.d/40-libopenni2-0.rules\". If not, add it and reboot.", devices[0].getName(), error, devices[0].getUri());
#endif
		}

		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	// look for calibration files
	_stereoModel = StereoCameraModel();
	bool hardwareRegistration = true;
	if(!calibrationFolder.empty())
	{
		// we need the serial
		std::string calibrationName = _device->getDeviceInfo().getName();
		if(!cameraName.empty())
		{
			calibrationName = cameraName;
		}
		_stereoModel.setName(calibrationName, "depth", "rgb");
		hardwareRegistration = !_stereoModel.load(calibrationFolder, calibrationName, false);

		if(_type != kTypeColorDepth)
		{
			hardwareRegistration = false;
		}


		if((_type != kTypeColorDepth && !_stereoModel.left().isValidForRectification()) ||
		   (_type == kTypeColorDepth && !_stereoModel.right().isValidForRectification()))
		{
			UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, default calibration used.",
					calibrationName.c_str(), calibrationFolder.c_str());
		}
		else if(_type == kTypeColorDepth && _stereoModel.right().isValidForRectification() && hardwareRegistration)
		{
			UWARN("Missing extrinsic calibration file for camera \"%s\" in \"%s\" folder, default registration is used even if rgb is rectified!",
					calibrationName.c_str(), calibrationFolder.c_str());
		}
		else if(_type == kTypeColorDepth && _stereoModel.right().isValidForRectification() && !hardwareRegistration)
		{
			UINFO("Custom calibration files for \"%s\" were found in \"%s\" folder. To use "
				  "factory calibration, remove the corresponding files from that directory.", calibrationName.c_str(), calibrationFolder.c_str());
		}
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
	else if(_type==kTypeColorDepth && hardwareRegistration &&
			!_device->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		UERROR("CameraOpenNI2: Device doesn't support depth/color registration.");
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_device->getSensorInfo(openni::SENSOR_DEPTH) == NULL ||
	  _device->getSensorInfo(_type==kTypeColorDepth?openni::SENSOR_COLOR:openni::SENSOR_IR) == NULL)
	{
		UERROR("CameraOpenNI2: Cannot get sensor info for depth and %s.", _type==kTypeColorDepth?"color":"ir");
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

	if(_color->create(*_device, _type==kTypeColorDepth?openni::SENSOR_COLOR:openni::SENSOR_IR) != openni::STATUS_OK)
	{
		UERROR("CameraOpenNI2: Cannot create %s stream.", _type==kTypeColorDepth?"color":"ir");
		_depth->destroy();
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_type==kTypeColorDepth && hardwareRegistration &&
	   _device->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) != openni::STATUS_OK)
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
		UINFO("CameraOpenNI2: %s video mode %d: fps=%d, pixel=%d, w=%d, h=%d",
				_type==kTypeColorDepth?"color":"ir",
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
	UINFO("CameraOpenNI2: Using %s video mode: fps=%d, pixel=%d, w=%d, h=%d, H-FOV=%f rad, V-FOV=%f rad",
			_type==kTypeColorDepth?"color":"ir",
			_color->getVideoMode().getFps(),
			_color->getVideoMode().getPixelFormat(),
			_color->getVideoMode().getResolutionX(),
			_color->getVideoMode().getResolutionY(),
			_color->getHorizontalFieldOfView(),
			_color->getVerticalFieldOfView());

	if(_depth->getVideoMode().getResolutionX() != 640 ||
		_depth->getVideoMode().getResolutionY() != 480 ||
		_depth->getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM)
	{
		UERROR("Could not set depth format to 640x480 pixel=%d(mm)!",
				openni::PIXEL_FORMAT_DEPTH_1_MM);
		_depth->destroy();
		_color->destroy();
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}
	if(_color->getVideoMode().getResolutionX() != 640 ||
		_color->getVideoMode().getResolutionY() != 480 ||
		_color->getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_RGB888)
	{
		UERROR("Could not set %s format to 640x480 pixel=%d!",
				_type==kTypeColorDepth?"color":"ir",
				openni::PIXEL_FORMAT_RGB888);
		_depth->destroy();
		_color->destroy();
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_color->getCameraSettings())
	{
		UINFO("CameraOpenNI2: AutoWhiteBalanceEnabled = %d", _color->getCameraSettings()->getAutoWhiteBalanceEnabled()?1:0);
		UINFO("CameraOpenNI2: AutoExposureEnabled = %d", _color->getCameraSettings()->getAutoExposureEnabled()?1:0);
#if ONI_VERSION_MAJOR > 2 || (ONI_VERSION_MAJOR==2 && ONI_VERSION_MINOR >= 2)
		UINFO("CameraOpenNI2: Exposure = %d", _color->getCameraSettings()->getExposure());
		UINFO("CameraOpenNI2: GAIN = %d", _color->getCameraSettings()->getGain());
#endif
	}

	if(_type==kTypeColorDepth && hardwareRegistration)
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

	if(_type == kTypeIR)
	{
		UWARN("With type IR-only, depth stream will not be started");
	}

	if((_type != kTypeIR && _depth->start() != openni::STATUS_OK) ||
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

	uSleep(3000); // just to make sure the sensor is correctly initialized and exposure is set

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
		_device->getSensorInfo(_type==kTypeColorDepth?openni::SENSOR_COLOR:openni::SENSOR_IR) != NULL)
	{
		openni::VideoStream* depthStream[] = {_depth};
		openni::VideoStream* colorStream[] = {_color};
		if((_type != kTypeIR && openni::OpenNI::waitForAnyStream(depthStream, 1, &readyStream, 5000) != openni::STATUS_OK) ||
		   openni::OpenNI::waitForAnyStream(colorStream, 1, &readyStream, 5000) != openni::STATUS_OK)
		{
			UWARN("No frames received since the last 5 seconds, end of stream is reached!");
		}
		else
		{
			openni::VideoFrameRef depthFrame, colorFrame;
			if(_type != kTypeIR)
			{
				_depth->readFrame(&depthFrame);
			}
			_color->readFrame(&colorFrame);
			cv::Mat depth, rgb;
			if((_type == kTypeIR || depthFrame.isValid()) && colorFrame.isValid())
			{
				int h,w;
				if(_type != kTypeIR)
				{
					h=depthFrame.getHeight();
					w=depthFrame.getWidth();
					depth = cv::Mat(h, w, CV_16U, (void*)depthFrame.getData()).clone();
				}
				h=colorFrame.getHeight();
				w=colorFrame.getWidth();
				cv::Mat tmp(h, w, CV_8UC3, (void *)colorFrame.getData());
				if(_type==kTypeColorDepth)
				{
					cv::cvtColor(tmp, rgb, CV_RGB2BGR);
				}
				else // IR
				{
					rgb = tmp.clone();
				}
			}
			UASSERT(_depthFx != 0.0f && _depthFy != 0.0f);
			if(!rgb.empty() && (_type == kTypeIR || !depth.empty()))
			{
				// default calibration
				CameraModel model(
						_depthFx, //fx
						_depthFy, //fy
						float(rgb.cols/2) - 0.5f,  //cx
						float(rgb.rows/2) - 0.5f,  //cy
						this->getLocalTransform(),
						0,
						rgb.size());

				if(_type==kTypeColorDepth)
				{
					if(_stereoModel.right().isValidForRectification())
					{
						rgb = _stereoModel.right().rectifyImage(rgb);
						model = _stereoModel.right();

						if(_stereoModel.left().isValidForRectification() && !_stereoModel.stereoTransform().isNull())
						{
							if (_depthHShift > 0 || _depthVShift > 0)
							{
								cv::Mat out = cv::Mat::zeros(depth.size(), depth.type());
								depth(cv::Rect(_depthHShift, _depthVShift, depth.cols - _depthHShift, depth.rows - _depthVShift)).copyTo(out(cv::Rect(0, 0, depth.cols - _depthHShift, depth.rows - _depthVShift)));
								depth = out;
							}
							depth = _stereoModel.left().rectifyImage(depth, 0);
							depth = util2d::registerDepth(depth, _stereoModel.left().K(), rgb.size(), _stereoModel.right().K(), _stereoModel.stereoTransform());
						}
					}
				}
				else // IR
				{
					if(_stereoModel.left().isValidForRectification())
					{
						rgb = _stereoModel.left().rectifyImage(rgb);
						if(_type!=kTypeIR)
						{
							depth = _stereoModel.left().rectifyImage(depth, 0);
						}
						model = _stereoModel.left();
					}
				}
				model.setLocalTransform(this->getLocalTransform());

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
	FreenectDevice(freenect_context * ctx, int index, bool color = true, bool registered = true) :
		index_(index),
		color_(color),
		registered_(registered),
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

		UINFO("color=%d registered=%d", color_?1:0, registered_?1:0);

		freenect_set_user(device_, this);
		freenect_frame_mode videoMode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, color_?FREENECT_VIDEO_RGB:FREENECT_VIDEO_IR_8BIT);
		freenect_frame_mode depthMode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, color_ && registered_?FREENECT_DEPTH_REGISTERED:FREENECT_DEPTH_MM);
		if(!videoMode.is_valid)
		{
			UERROR("Freenect: video mode selected not valid!");
			return false;
		}
		if(!depthMode.is_valid)
		{
			UERROR("Freenect: depth mode selected not valid!");
			return false;
		}
		UASSERT(videoMode.data_bits_per_pixel == 8 || videoMode.data_bits_per_pixel == 24);
		UASSERT(depthMode.data_bits_per_pixel == 16);
		freenect_set_video_mode(device_, videoMode);
		freenect_set_depth_mode(device_, depthMode);
		rgbIrBuffer_ = cv::Mat(cv::Size(videoMode.width,videoMode.height), color_?CV_8UC3:CV_8UC1);
		depthBuffer_ = cv::Mat(cv::Size(depthMode.width,depthMode.height), CV_16UC1);
		freenect_set_depth_buffer(device_, depthBuffer_.data);
		freenect_set_video_buffer(device_, rgbIrBuffer_.data);
		freenect_set_depth_callback(device_, freenect_depth_callback);
		freenect_set_video_callback(device_, freenect_video_callback);

		float rgb_focal_length_sxga = 1050.0f;
		float width_sxga = 1280.0f;
		float width = freenect_get_current_depth_mode(device_).width;
		float scale = width / width_sxga;
		if(color_ && registered_)
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
			if(!dataReady_.acquire(1, 5000))
			{
				UERROR("Not received any frames since 5 seconds, try to restart the camera again.");
			}
			else
			{
				UScopeMutex s(dataMutex_);
				rgb = rgbIrLastFrame_;
				depth = depthLastFrame_;
				rgbIrLastFrame_ = cv::Mat();
				depthLastFrame_= cv::Mat();
			}
		}
	}

private:
	// Do not call directly even in child
	void VideoCallback(void* rgb)
	{
		UASSERT(rgbIrBuffer_.data == rgb);
		UScopeMutex s(dataMutex_);
		bool notify = rgbIrLastFrame_.empty();

		if(color_)
		{
			cv::cvtColor(rgbIrBuffer_, rgbIrLastFrame_, CV_RGB2BGR);
		}
		else // IrDepth
		{
			rgbIrLastFrame_ = rgbIrBuffer_.clone();
		}
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
		if(!rgbIrLastFrame_.empty() && notify)
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
	bool color_;
	bool registered_;
	std::string serial_;
	freenect_context * ctx_;
	freenect_device * device_;
	cv::Mat depthBuffer_;
	cv::Mat rgbIrBuffer_;
	UMutex dataMutex_;
	cv::Mat depthLastFrame_;
	cv::Mat rgbIrLastFrame_;
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

CameraFreenect::CameraFreenect(int deviceId, Type type, float imageRate, const Transform & localTransform) :
		Camera(imageRate, localTransform)
#ifdef RTABMAP_FREENECT
        ,
		deviceId_(deviceId),
		type_(type),
		ctx_(0),
		freenectDevice_(0)
#endif
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
		// look for calibration files
		bool hardwareRegistration = true;
		stereoModel_ = StereoCameraModel();
		if(!calibrationFolder.empty())
		{
			// we need the serial, HACK: init a temp device to get it
			FreenectDevice dev(ctx_, deviceId_);
			if(!dev.init())
			{
				UERROR("CameraFreenect: Init failed!");
			}
			std::string calibrationName = dev.getSerial();
			if(!cameraName.empty())
			{
				calibrationName = cameraName;
			}
			stereoModel_.setName(calibrationName, "depth", "rgb");
			hardwareRegistration = !stereoModel_.load(calibrationFolder, calibrationName, false);

			if(type_ == kTypeIRDepth)
			{
				hardwareRegistration = false;
			}


			if((type_ == kTypeIRDepth && !stereoModel_.left().isValidForRectification()) ||
			   (type_ == kTypeColorDepth && !stereoModel_.right().isValidForRectification()))
			{
				UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, default calibration used.",
						calibrationName.c_str(), calibrationFolder.c_str());
			}
			else if(type_ == kTypeColorDepth && stereoModel_.right().isValidForRectification() && hardwareRegistration)
			{
				UWARN("Missing extrinsic calibration file for camera \"%s\" in \"%s\" folder, default registration is used even if rgb is rectified!",
						calibrationName.c_str(), calibrationFolder.c_str());
			}
			else if(type_ == kTypeColorDepth && stereoModel_.right().isValidForRectification() && !hardwareRegistration)
			{
				UINFO("Custom calibration files for \"%s\" were found in \"%s\" folder. To use "
					  "factory calibration, remove the corresponding files from that directory.", calibrationName.c_str(), calibrationFolder.c_str());
			}
		}

		freenectDevice_ = new FreenectDevice(ctx_, deviceId_, type_==kTypeColorDepth, hardwareRegistration);
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

				// default calibration
				CameraModel model(
						freenectDevice_->getDepthFocal(), //fx
						freenectDevice_->getDepthFocal(), //fy
						float(rgb.cols/2) - 0.5f,  //cx
						float(rgb.rows/2) - 0.5f,  //cy
						this->getLocalTransform(),
						0,
						rgb.size());

				if(type_==kTypeIRDepth)
				{
					if(stereoModel_.left().isValidForRectification())
					{
						rgb = stereoModel_.left().rectifyImage(rgb);
						depth = stereoModel_.left().rectifyImage(depth, 0);
						model = stereoModel_.left();
					}
				}
				else
				{
					if(stereoModel_.right().isValidForRectification())
					{
						rgb = stereoModel_.right().rectifyImage(rgb);
						model = stereoModel_.right();

						if(stereoModel_.left().isValidForRectification() && !stereoModel_.stereoTransform().isNull())
						{
							depth = stereoModel_.left().rectifyImage(depth, 0);
							depth = util2d::registerDepth(depth, stereoModel_.left().K(), rgb.size(), stereoModel_.right().K(), stereoModel_.stereoTransform());
						}
					}
				}
				model.setLocalTransform(this->getLocalTransform());

				data = SensorData(rgb, depth, model, this->getNextSeqID(), UTimer::now());
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
	bool noiseFiltering,
	const std::string & pipelineName) :
		Camera(imageRate, localTransform)
#ifdef RTABMAP_FREENECT2
        ,
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
		noiseFiltering_(noiseFiltering),
		pipelineName_(pipelineName)
#endif
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

#ifdef RTABMAP_FREENECT2
libfreenect2::PacketPipeline *createPacketPipelineByName(const std::string & name)
{
	std::string availablePipelines;
#if defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
	availablePipelines += "gl ";
	if (name == "gl")
	{
		UINFO("Using 'gl' pipeline.");
		return new libfreenect2::OpenGLPacketPipeline();
	}
#endif
#if defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
	availablePipelines += "cuda cudakde ";
	if (name == "cuda")
	{
		UINFO("Using 'cuda' pipeline.");
		return new libfreenect2::CudaPacketPipeline();
	}
	if (name == "cudakde")
	{
		UINFO("Using 'cudakde' pipeline.");
		return new libfreenect2::CudaKdePacketPipeline();
	}
#endif
#if defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
	availablePipelines += "cl clkde ";
	if (name == "cl")
	{
		UINFO("Using 'cl' pipeline.");
		return new libfreenect2::OpenCLPacketPipeline();
	}
	if (name == "clkde")
	{
		UINFO("Using 'clkde' pipeline.");
		return new libfreenect2::OpenCLKdePacketPipeline();
	}
#endif
	availablePipelines += "cpu";
	if (name == "cpu")
	{
		UINFO("Using 'cpu' pipeline.");
		return new libfreenect2::CpuPacketPipeline();
	}

	if (!name.empty())
	{
		UERROR("'%s' pipeline is not available. Available pipelines are: \"%s\". Default one is used instead (first one in the list).", 
			name.c_str(), availablePipelines.c_str());
	}

	// create default pipeline
#if defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
	UINFO("Using 'gl' pipeline.");
	return new libfreenect2::OpenGLPacketPipeline();
#elif defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
	UINFO("Using 'cuda' pipeline.");
	return new libfreenect2::CudaPacketPipeline();
#elif defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
	UINFO("Using 'cl' pipeline.");
	return new libfreenect2::OpenCLPacketPipeline();
#else
	UINFO("Using 'cpu' pipeline.");
	return new libfreenect2::CpuPacketPipeline();
#endif
}
#endif

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

	libfreenect2::PacketPipeline * pipeline = createPacketPipelineByName(pipelineName_);

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
		stereoModel_ = StereoCameraModel();
		if(!calibrationFolder.empty())
		{
			std::string calibrationName = dev_->getSerialNumber();
			if(!cameraName.empty())
			{
				calibrationName = cameraName;
			}
			stereoModel_.setName(calibrationName, "depth", "rgb");
			if(!stereoModel_.load(calibrationFolder, calibrationName, false))
			{
				UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, default calibration "
					"is used. Note that from version 0.11.10, calibration suffixes for Freenect2 driver have "
					"changed from \"_left\"->\"_depth\" and \"_right\"->\"_rgb\". You can safely rename "
					"the calibration files to avoid recalibrating.",
						calibrationName.c_str(), calibrationFolder.c_str());
			}
			else
			{
				UINFO("Custom calibration files for \"%s\" were found in \"%s\" folder. To use "
					  "factory calibration, remove the corresponding files from that directory.", calibrationName.c_str(), calibrationFolder.c_str());

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
					depth = stereoModel_.left().rectifyDepth(depth);
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
					#ifdef LIBFREENECT2_WITH_TEGRAJPEG_SUPPORT 

					 cv::cvtColor(rgbMatC4, rgbMat, CV_RGBA2BGR); 

					#else 

					cv::cvtColor(rgbMatC4, rgbMat, CV_BGRA2BGR); 

					#endif 
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

						//depth = stereoModel_.left().rectifyImage(depth, 0); // ~0.5/4 ms but is more noisy
						depth = stereoModel_.left().rectifyDepth(depth);      // ~16/25 ms

						bool registered = true;
						if(registered)
						{
							depth = util2d::registerDepth(
									depth,
									stereoModel_.left().P().colRange(0,3).rowRange(0,3), //scaled depth K
									depth.size(),
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
						#ifdef LIBFREENECT2_WITH_TEGRAJPEG_SUPPORT 

						 cv::cvtColor(rgbMatC4, rgbMat, CV_RGB2BGR); 

						#else 

						cv::cvtColor(rgbMatC4, rgbMat, CV_BGRA2BGR); 

						#endif 
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
							#ifdef LIBFREENECT2_WITH_TEGRAJPEG_SUPPORT 

							 cv::cvtColor(rgbMatBGRA, rgb, CV_RGBA2BGR); 

							#else 

							cv::cvtColor(rgbMatBGRA, rgb, CV_BGRA2BGR); 

							#endif 
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
							#ifdef LIBFREENECT2_WITH_TEGRAJPEG_SUPPORT 

							 cv::cvtColor(rgbMatBGRA, rgb, CV_RGBA2BGR); 

							#else 

							cv::cvtColor(rgbMatBGRA, rgb, CV_BGRA2BGR); 

							#endif 
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
// CameraK4W2
//

#ifdef RTABMAP_K4W2
// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
#endif

bool CameraK4W2::available()
{
#ifdef RTABMAP_K4W2
	return true;
#else
	return false;
#endif
}

CameraK4W2::CameraK4W2(
	int deviceId,
	Type type,
	float imageRate,
	const Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_K4W2
	,
	type_(type),
	pKinectSensor_(NULL),
	pCoordinateMapper_(NULL),
	pDepthCoordinates_(new DepthSpacePoint[cColorWidth * cColorHeight]),
	pColorCoordinates_(new ColorSpacePoint[cDepthWidth * cDepthHeight]),
	pMultiSourceFrameReader_(NULL),
	pColorRGBX_(new RGBQUAD[cColorWidth * cColorHeight]),
	hMSEvent(NULL)
#endif
{
}

CameraK4W2::~CameraK4W2()
{
#ifdef RTABMAP_K4W2
	if (pDepthCoordinates_)
	{
		delete[] pDepthCoordinates_;
		pDepthCoordinates_ = NULL;
	}

	if (pColorCoordinates_)
	{
		delete[] pColorCoordinates_;
		pColorCoordinates_ = NULL;
	}

	if (pColorRGBX_)
	{
		delete[] pColorRGBX_;
		pColorRGBX_ = NULL;
	}

	close();
#endif
}

void CameraK4W2::close()
{
#ifdef RTABMAP_K4W2
	if (pMultiSourceFrameReader_)
	{
		pMultiSourceFrameReader_->UnsubscribeMultiSourceFrameArrived(hMSEvent);
		CloseHandle((HANDLE)hMSEvent);
		hMSEvent = NULL;
	}

	// done with frame reader
	SafeRelease(pMultiSourceFrameReader_);

	// done with coordinate mapper
	SafeRelease(pCoordinateMapper_);

	// close the Kinect Sensor
	if (pKinectSensor_)
	{
		pKinectSensor_->Close();
	}

	SafeRelease(pKinectSensor_);

	colorCameraModel_ = CameraModel();
#endif
}

bool CameraK4W2::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_K4W2
	HRESULT hr;

	close();

	hr = GetDefaultKinectSensor(&pKinectSensor_);
	if (FAILED(hr))
	{
		return false;
	}

	if (pKinectSensor_)
	{
		// Initialize the Kinect and get coordinate mapper and the frame reader

		hr = pKinectSensor_->Open();

		if (SUCCEEDED(hr))
		{
			hr = pKinectSensor_->get_CoordinateMapper(&pCoordinateMapper_);

			if (SUCCEEDED(hr))
			{
				hr = pKinectSensor_->OpenMultiSourceFrameReader(
					FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
					&pMultiSourceFrameReader_);

				if (SUCCEEDED(hr))
				{
					hr = pMultiSourceFrameReader_->SubscribeMultiSourceFrameArrived(&hMSEvent);
				}
			}
		}
	}

	if (!pKinectSensor_ || FAILED(hr))
	{
		UERROR("No ready Kinect found!");
		close();
		return false;
	}

	// to query camera parameters, we should wait a little
	uSleep(3000);

	// initialize color calibration if not set yet
	CameraIntrinsics intrinsics;
	hr = pCoordinateMapper_->GetDepthCameraIntrinsics(&intrinsics);
	if (SUCCEEDED(hr) && intrinsics.FocalLengthX > 0.0f)
	{
		// guess color intrinsics by comparing two reprojections
		CameraModel depthModel(
			intrinsics.FocalLengthX,
			intrinsics.FocalLengthY,
			intrinsics.PrincipalPointX,
			intrinsics.PrincipalPointY);

		cv::Mat fakeDepth = cv::Mat::ones(cDepthHeight, cDepthWidth, CV_16UC1) * 1000;
		hr = pCoordinateMapper_->MapDepthFrameToColorSpace(cDepthWidth * cDepthHeight, (UINT16*)fakeDepth.data, cDepthWidth * cDepthHeight, pColorCoordinates_);
		if (SUCCEEDED(hr))
		{
			int firstIndex = -1;
			int lastIndex = -1;
			for (int depthIndex = 0; depthIndex < (cDepthWidth*cDepthHeight); ++depthIndex)
			{
				ColorSpacePoint p = pColorCoordinates_[depthIndex];
				// Values that are negative infinity means it is an invalid color to depth mapping so we
				// skip processing for this pixel
				if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
				{
					if (firstIndex == -1)
					{
						firstIndex = depthIndex;
					}
					lastIndex = depthIndex;
				}
			}

			UASSERT(firstIndex >= 0 && lastIndex >= 0);
			float fx, fy, cx, cy;
			float x1, y1, z1, x2, y2, z2;
			depthModel.project(firstIndex - (firstIndex / cDepthWidth)*cDepthWidth, firstIndex / cDepthWidth, 1.0f, x1, y1, z1);
			depthModel.project(lastIndex - (lastIndex / cDepthWidth)*cDepthWidth, lastIndex / cDepthWidth, 1.0f, x2, y2, z2);
			ColorSpacePoint uv1 = pColorCoordinates_[firstIndex];
			ColorSpacePoint uv2 = pColorCoordinates_[lastIndex];
			fx = ((uv1.X - uv2.X)*z1*z2) / (x1*z2 - x2*z1);
			cx = uv1.X - (x1 / z1) * fx;
			fy = ((uv1.Y - uv2.Y)*z1*z2) / (y1*z2 - y2*z1);
			cy = uv1.Y - (y1 / z1) * fy;

			colorCameraModel_ = CameraModel(
				fx,
				fy,
				float(cColorWidth) - cx,
				cy,
				this->getLocalTransform(),
				0,
				cv::Size(cColorWidth, cColorHeight));
		}
	}

	if (!colorCameraModel_.isValidForProjection())
	{
		UERROR("Failed to get camera parameters! Is the camera connected? Try restarting the camera again or use kTypeColor2DepthSD.");
		close();
		return false;
	}

	std::string serial = getSerial();
	if (!serial.empty())
	{
		UINFO("Running kinect device \"%s\"", serial.c_str());
	}

	return true;
#else
	UERROR("CameraK4W2: RTAB-Map is not built with Kinect for Windows 2 SDK support!");
	return false;
#endif
}

bool CameraK4W2::isCalibrated() const
{
	return true;
}

std::string CameraK4W2::getSerial() const
{
#ifdef RTABMAP_K4W2
	if (pKinectSensor_)
	{
		wchar_t uid[255] = { 0 };
		// It seems to fail every time!?
		HRESULT hr = pKinectSensor_->get_UniqueKinectId(255, uid);
		if (SUCCEEDED(hr))
		{
			std::wstring ws(uid);
			return std::string(ws.begin(), ws.end());
		}
	}
#endif
	return "";
}

SensorData CameraK4W2::captureImage(CameraInfo * info)
{
	SensorData data;

#ifdef RTABMAP_K4W2

	if (!pMultiSourceFrameReader_)
	{
		return data;
	}

	HRESULT hr;

	//now check for frame events
	HANDLE handles[] = { reinterpret_cast<HANDLE>(hMSEvent) };

	double t = UTimer::now();
	while((UTimer::now()-t < 5.0) && WaitForMultipleObjects(_countof(handles), handles, false, 5000) == WAIT_OBJECT_0)
	{
		IMultiSourceFrameArrivedEventArgs* pArgs = NULL;

		hr = pMultiSourceFrameReader_->GetMultiSourceFrameArrivedEventData(hMSEvent, &pArgs);
		if (SUCCEEDED(hr))
		{
			IMultiSourceFrameReference * pFrameRef = NULL;
			hr = pArgs->get_FrameReference(&pFrameRef);
			if (SUCCEEDED(hr))
			{
				IMultiSourceFrame* pMultiSourceFrame = NULL;
				IDepthFrame* pDepthFrame = NULL;
				IColorFrame* pColorFrame = NULL;

				hr = pFrameRef->AcquireFrame(&pMultiSourceFrame);
				if (FAILED(hr))
				{
					UERROR("Failed getting latest frame.");
				}

				IDepthFrameReference* pDepthFrameReference = NULL;
				hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
				if (SUCCEEDED(hr))
				{
					hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
				}
				SafeRelease(pDepthFrameReference);

				IColorFrameReference* pColorFrameReference = NULL;
				hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
				if (SUCCEEDED(hr))
				{
					hr = pColorFrameReference->AcquireFrame(&pColorFrame);
				}
				SafeRelease(pColorFrameReference);

				if (pDepthFrame && pColorFrame)
				{
					IFrameDescription* pDepthFrameDescription = NULL;
					int nDepthWidth = 0;
					int nDepthHeight = 0;
					UINT nDepthBufferSize = 0;
					UINT16 *pDepthBuffer = NULL;

					IFrameDescription* pColorFrameDescription = NULL;
					int nColorWidth = 0;
					int nColorHeight = 0;
					ColorImageFormat imageFormat = ColorImageFormat_None;
					UINT nColorBufferSize = 0;
					RGBQUAD *pColorBuffer = NULL;

					// get depth frame data
					if (SUCCEEDED(hr))
						hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
					if (SUCCEEDED(hr))
						hr = pDepthFrameDescription->get_Width(&nDepthWidth);
					if (SUCCEEDED(hr))
						hr = pDepthFrameDescription->get_Height(&nDepthHeight);
					if (SUCCEEDED(hr))
						hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);

					// get color frame data
					if (SUCCEEDED(hr))
						hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
					if (SUCCEEDED(hr))
						hr = pColorFrameDescription->get_Width(&nColorWidth);
					if (SUCCEEDED(hr))
						hr = pColorFrameDescription->get_Height(&nColorHeight);
					if (SUCCEEDED(hr))
						hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
					if (SUCCEEDED(hr))
					{
						if (imageFormat == ColorImageFormat_Bgra)
						{
							hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
						}
						else if (pColorRGBX_)
						{
							pColorBuffer = pColorRGBX_;
							nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
							hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
						}
						else
						{
							hr = E_FAIL;
						}
					}

					if(SUCCEEDED(hr))
					{
						//ProcessFrame(nDepthTime, pDepthBuffer, nDepthWidth, nDepthHeight,
						//	pColorBuffer, nColorWidth, nColorHeight,
						//	pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight);

						// Make sure we've received valid data
						if (pCoordinateMapper_ &&
							pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) &&
							pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight))
						{
							if (type_ == kTypeColor2DepthSD)
							{
								HRESULT hr = pCoordinateMapper_->MapColorFrameToDepthSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, nColorWidth * nColorHeight, pDepthCoordinates_);
								if (SUCCEEDED(hr))
								{
									cv::Mat depth = cv::Mat::zeros(nDepthHeight, nDepthWidth, CV_16UC1);
									cv::Mat imageColorRegistered = cv::Mat::zeros(nDepthHeight, nDepthWidth, CV_8UC3);
									// loop over output pixels
									for (int colorIndex = 0; colorIndex < (nColorWidth*nColorHeight); ++colorIndex)
									{
										DepthSpacePoint p = pDepthCoordinates_[colorIndex];
										// Values that are negative infinity means it is an invalid color to depth mapping so we
										// skip processing for this pixel
										if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
										{
											// To avoid black lines caused by rounding pixel values, we should set 4 pixels
											// At the same do mirror
											int pixel_x_l, pixel_y_l, pixel_x_h, pixel_y_h;
											pixel_x_l = nDepthWidth - static_cast<int>(p.X);
											pixel_y_l = static_cast<int>(p.Y);
											pixel_x_h = pixel_x_l - 1;
											pixel_y_h = pixel_y_l + 1;

											const RGBQUAD* pSrc = pColorBuffer + colorIndex;
											if ((pixel_x_l >= 0 && pixel_x_l < nDepthWidth) && (pixel_y_l >= 0 && pixel_y_l < nDepthHeight))
											{
												unsigned char *  ptr = imageColorRegistered.ptr<unsigned char>(pixel_y_l, pixel_x_l);
												ptr[0] = pSrc->rgbBlue;
												ptr[1] = pSrc->rgbGreen;
												ptr[2] = pSrc->rgbRed;
												depth.at<unsigned short>(pixel_y_l, pixel_x_l) = *(pDepthBuffer + nDepthWidth - pixel_x_l + pixel_y_l*nDepthWidth);
											}
											if ((pixel_x_l >= 0 && pixel_x_l < nDepthWidth) && (pixel_y_h >= 0 && pixel_y_h < nDepthHeight))
											{
												unsigned char *  ptr = imageColorRegistered.ptr<unsigned char>(pixel_y_h, pixel_x_l);
												ptr[0] = pSrc->rgbBlue;
												ptr[1] = pSrc->rgbGreen;
												ptr[2] = pSrc->rgbRed;
												depth.at<unsigned short>(pixel_y_h, pixel_x_l) = *(pDepthBuffer + nDepthWidth - pixel_x_l + pixel_y_h*nDepthWidth);
											}
											if ((pixel_x_h >= 0 && pixel_x_h < nDepthWidth) && (pixel_y_l >= 0 && pixel_y_l < nDepthHeight))
											{
												unsigned char *  ptr = imageColorRegistered.ptr<unsigned char>(pixel_y_l, pixel_x_h);
												ptr[0] = pSrc->rgbBlue;
												ptr[1] = pSrc->rgbGreen;
												ptr[2] = pSrc->rgbRed;
												depth.at<unsigned short>(pixel_y_l, pixel_x_h) = *(pDepthBuffer + nDepthWidth - pixel_x_h + pixel_y_l*nDepthWidth);
											}
											if ((pixel_x_h >= 0 && pixel_x_h < nDepthWidth) && (pixel_y_h >= 0 && pixel_y_h < nDepthHeight))
											{
												unsigned char *  ptr = imageColorRegistered.ptr<unsigned char>(pixel_y_h, pixel_x_h);
												ptr[0] = pSrc->rgbBlue;
												ptr[1] = pSrc->rgbGreen;
												ptr[2] = pSrc->rgbRed;
												depth.at<unsigned short>(pixel_y_h, pixel_x_h) = *(pDepthBuffer + nDepthWidth - pixel_x_h + pixel_y_h*nDepthWidth);
											}
										}
									}

									CameraIntrinsics intrinsics;
									pCoordinateMapper_->GetDepthCameraIntrinsics(&intrinsics);
									CameraModel model(
										intrinsics.FocalLengthX,
										intrinsics.FocalLengthY,
										intrinsics.PrincipalPointX,
										intrinsics.PrincipalPointY,
										this->getLocalTransform(),
										0,
										depth.size());
									data = SensorData(imageColorRegistered, depth, model, this->getNextSeqID(), UTimer::now());
								}
								else
								{
									UERROR("Failed color to depth registration!");
								}
							}
							else //depthToColor
							{
								HRESULT hr = pCoordinateMapper_->MapDepthFrameToColorSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth * nDepthHeight, pColorCoordinates_);
								if (SUCCEEDED(hr))
								{
									cv::Mat depthSource(nDepthHeight, nDepthWidth, CV_16UC1, pDepthBuffer);
									cv::Mat depthRegistered = cv::Mat::zeros(
										type_ == kTypeDepth2ColorSD ? nColorHeight/2 : nColorHeight, 
										type_ == kTypeDepth2ColorSD ? nColorWidth/2 : nColorWidth,
										CV_16UC1);
									cv::Mat imageColor;
									if(type_ == kTypeDepth2ColorSD)
									{
										cv::Mat tmp;
										cv::resize(cv::Mat(nColorHeight, nColorWidth, CV_8UC4, pColorBuffer), tmp, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
										cv::cvtColor(tmp, imageColor, CV_BGRA2BGR);
									}
									else
									{
										cv::cvtColor(cv::Mat(nColorHeight, nColorWidth, CV_8UC4, pColorBuffer), imageColor, CV_BGRA2BGR);
									}
									// loop over output pixels
									for (int depthIndex = 0; depthIndex < (nDepthWidth*nDepthHeight); ++depthIndex)
									{
										ColorSpacePoint p = pColorCoordinates_[depthIndex];
										// Values that are negative infinity means it is an invalid color to depth mapping so we
										// skip processing for this pixel
										if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
										{
											if (type_ == kTypeDepth2ColorSD)
											{
												p.X /= 2.0f;
												p.Y /= 2.0f;
											}
											const unsigned short & depth_value = depthSource.at<unsigned short>(0, depthIndex);
											int pixel_x_l, pixel_y_l, pixel_x_h, pixel_y_h;
											// get the coordinate on image plane.
											pixel_x_l = depthRegistered.cols - p.X; // flip depth
											pixel_y_l = p.Y;
											pixel_x_h = pixel_x_l - 1;
											pixel_y_h = pixel_y_l + 1;

											if (pixel_x_l >= 0 && pixel_x_l < depthRegistered.cols &&
												pixel_y_l>0 && pixel_y_l < depthRegistered.rows && // ignore first line
												depth_value)
											{
												unsigned short & depthPixel = depthRegistered.at<unsigned short>(pixel_y_l, pixel_x_l);
												if (depthPixel == 0 || depthPixel > depth_value)
												{
													depthPixel = depth_value;
												}
											}
											if (pixel_x_h >= 0 && pixel_x_h < depthRegistered.cols &&
												pixel_y_h>0 && pixel_y_h < depthRegistered.rows && // ignore first line
												depth_value)
											{
												unsigned short & depthPixel = depthRegistered.at<unsigned short>(pixel_y_h, pixel_x_h);
												if (depthPixel == 0 || depthPixel > depth_value)
												{
													depthPixel = depth_value;
												}
											}
										}
									}

									CameraModel model = colorCameraModel_;
									if (type_ == kTypeDepth2ColorSD)
									{
										model = model.scaled(0.5);
									}
									util2d::fillRegisteredDepthHoles(depthRegistered, true, true, type_ == kTypeDepth2ColorHD);
									depthRegistered = rtabmap::util2d::fillDepthHoles(depthRegistered, 1);
									cv::flip(imageColor, imageColor, 1);
									data = SensorData(imageColor, depthRegistered, model, this->getNextSeqID(), UTimer::now());
								}
								else
								{
									UERROR("Failed depth to color registration!");
								}
							}
						}
					}

					SafeRelease(pDepthFrameDescription);
					SafeRelease(pColorFrameDescription);
				}

				pFrameRef->Release();

				SafeRelease(pDepthFrame);
				SafeRelease(pColorFrame);
				SafeRelease(pMultiSourceFrame);
			}	
			pArgs->Release();
		}
		if (!data.imageRaw().empty())
		{
			break;
		}
	}
#else
	UERROR("CameraK4W2: RTAB-Map is not built with Kinect for Windows 2 SDK support!");
#endif
	return data;
}

/////////////////////////
// CameraRealSense
/////////////////////////
bool CameraRealSense::available()
{
#ifdef RTABMAP_REALSENSE
	return true;
#else
	return false;
#endif
}

CameraRealSense::CameraRealSense(
		int device,
		int presetRGB,
		int presetDepth,
		bool computeOdometry,
		float imageRate,
		const rtabmap::Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_REALSENSE
    ,
	ctx_(0),
	dev_(0),
	deviceId_(device),
	presetRGB_(presetRGB),
	presetDepth_(presetDepth),
	computeOdometry_(computeOdometry),
	slam_(0)
#endif
{
	UDEBUG("");
}

CameraRealSense::~CameraRealSense()
{
	UDEBUG("");
#ifdef RTABMAP_REALSENSE
	UDEBUG("");
	if(dev_)
	{
		if(slam_!=0)
		{
			dev_->stop(rs::source::all_sources);
		}
		else
		{
			dev_->stop();
		}
		dev_ = 0;
	}
	UDEBUG("");
	if (ctx_)
	{
		delete ctx_;
	}
#ifdef RTABMAP_REALSENSE_SLAM
	UDEBUG("");
	if(slam_)
	{
		UScopeMutex lock(slamLock_);
		slam_->flush_resources();
		delete slam_;
		slam_ = 0;
	}
#endif
#endif
}

#ifdef RTABMAP_REALSENSE_SLAM
bool setStreamConfigIntrin(
		rs::core::stream_type stream,
		std::map< rs::core::stream_type, rs::core::intrinsics > intrinsics,
		rs::core::video_module_interface::supported_module_config & supported_config,
		rs::core::video_module_interface::actual_module_config & actual_config)
{
  auto & supported_stream_config = supported_config[stream];
  if (!supported_stream_config.is_enabled || supported_stream_config.size.width != intrinsics[stream].width || supported_stream_config.size.height != intrinsics[stream].height)
  {
	  UERROR("size of stream is not supported by slam");
	  			UERROR("  supported: stream %d, width: %d height: %d", (uint32_t) stream, supported_stream_config.size.width, supported_stream_config.size.height);
	  			UERROR("  received: stream %d, width: %d height: %d", (uint32_t) stream, intrinsics[stream].width, intrinsics[stream].height);

    return false;
  }
  rs::core::video_module_interface::actual_image_stream_config &actual_stream_config = actual_config[stream];
  actual_config[stream].size.width = intrinsics[stream].width;
  actual_config[stream].size.height = intrinsics[stream].height;
  actual_stream_config.frame_rate = supported_stream_config.frame_rate;
  actual_stream_config.intrinsics = intrinsics[stream];
  actual_stream_config.is_enabled = true;
  return true;
}
#endif

bool CameraRealSense::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_REALSENSE

	if(dev_)
	{
		dev_->stop(rs::source::all_sources);
		dev_ = 0;
	}
	bufferedFrames_.clear();

#ifdef RTABMAP_REALSENSE_SLAM
	motionSeq_[0] = motionSeq_[1] = 0;
	if(slam_)
	{
		UScopeMutex lock(slamLock_);
		UDEBUG("Flush slam");
		slam_->flush_resources();
		delete slam_;
		slam_ = 0;
	}
#endif

	if (ctx_ == 0)
	{
		ctx_ = new rs::context();
	}

	UDEBUG("");
	if (ctx_->get_device_count() == 0)
	{
		UERROR("No RealSense device detected!");
		return false;
	}

	UDEBUG("");
	dev_ = ctx_->get_device(deviceId_);
	if (dev_ == 0)
	{
		UERROR("Cannot connect to device %d", deviceId_);
		return false;
	}
	std::string name = dev_->get_name();
	UINFO("Using device %d, an %s", deviceId_, name.c_str());
	UINFO("    Serial number: %s", dev_->get_serial());
	UINFO("    Firmware version: %s", dev_->get_firmware_version());
	UINFO("    Preset RGB: %d", presetRGB_);
	UINFO("    Preset Depth: %d", presetDepth_);

	bool computeOdometry = false;
#ifdef RTABMAP_REALSENSE_SLAM
	if (name.find("ZR300") != std::string::npos && computeOdometry_)
	{
		// Only enable ZR300 functionality if fisheye stream is enabled.
		// Accel/Gyro automatically enabled when fisheye requested
		computeOdometry = true;
	}
#endif

	// Configure depth and color to run with the device's preferred settings
	UINFO("Enabling streams...");
	// R200: 
	//    0=640x480   vs 480x360
	//    1=1920x1080 vs 640x480
	//    2=640x480   vs 320x240
	dev_->enable_stream(rs::stream::depth, (rs::preset)presetDepth_);
	dev_->enable_stream(rs::stream::color, (rs::preset)presetRGB_);

	rs::intrinsics depth_intrin = dev_->get_stream_intrinsics(rs::stream::depth);
	rs::intrinsics color_intrin = dev_->get_stream_intrinsics(rs::stream::color);
	UINFO("    RGB:   %dx%d", color_intrin.width, color_intrin.height);
	UINFO("    Depth: %dx%d", depth_intrin.width, depth_intrin.height);

#ifdef RTABMAP_REALSENSE_SLAM
	UDEBUG("Setup frame callback");
	// Define lambda callback for receiving stream data
	std::function<void(rs::frame)> frameCallback = [this](rs::frame frame)
	{
		if(slam_ != 0)
		{
			const auto timestampDomain = frame.get_frame_timestamp_domain();
			if (rs::timestamp_domain::microcontroller != timestampDomain)
			{
				UERROR("error: Junk time stamp in stream: %d\twith frame counter: %d",
						(int)(frame.get_stream_type()), frame.get_frame_number());
				return ;
			}
		}

		int width = frame.get_width();
		int height = frame.get_height();
		rs::core::correlated_sample_set sample_set = {};

		rs::core::image_info info =
		{
		  width,
		  height,
		  rs::utils::convert_pixel_format(frame.get_format()),
		  frame.get_stride()
		};
		cv::Mat image;
		if(frame.get_format() == rs::format::raw8)
		{
			image = cv::Mat(height, width, CV_8UC1, (unsigned char*)frame.get_data());
		}
		else if(frame.get_format() == rs::format::z16)
		{
			image = cv::Mat(height, width, CV_16UC1, (unsigned char*)frame.get_data());
			if(bufferedFrames_.find(frame.get_timestamp()) != bufferedFrames_.end())
			{
				bufferedFrames_.find(frame.get_timestamp())->second.second = image.clone();
				UScopeMutex lock(dataMutex_);
				bool notify = lastSyncFrames_.first.empty();
				lastSyncFrames_ = bufferedFrames_.find(frame.get_timestamp())->second;
				if(notify)
				{
					dataReady_.release();
				}
				bufferedFrames_.erase(frame.get_timestamp());
			}
			else
			{
				bufferedFrames_.insert(std::make_pair(frame.get_timestamp(), std::make_pair(cv::Mat(), image.clone())));
			}
			if(bufferedFrames_.size()>5)
			{
				UWARN("Frames cannot be synchronized!");
				bufferedFrames_.clear();
			}
		}
		else if(frame.get_format() == rs::format::rgb8)
		{
			image = cv::Mat(height, width, CV_8UC3, (unsigned char*)frame.get_data());
			if(bufferedFrames_.find(frame.get_timestamp()) != bufferedFrames_.end())
			{
				bufferedFrames_.find(frame.get_timestamp())->second.first = image.clone();
				UScopeMutex lock(dataMutex_);
				bool notify = lastSyncFrames_.first.empty();
				lastSyncFrames_ = bufferedFrames_.find(frame.get_timestamp())->second;
				if(notify)
				{
					dataReady_.release();
				}
				bufferedFrames_.erase(frame.get_timestamp());
			}
			else
			{
				bufferedFrames_.insert(std::make_pair(frame.get_timestamp(), std::make_pair(image.clone(), cv::Mat())));
			}
			if(bufferedFrames_.size()>5)
			{
				UWARN("Frames cannot be synchronized!");
				bufferedFrames_.clear();
			}
			return;
		}
		else
		{
			return;
		}

		if(slam_ != 0)
		{
			rs::core::stream_type stream = rs::utils::convert_stream_type(frame.get_stream_type());
			sample_set[stream] = rs::core::image_interface::create_instance_from_raw_data(
								   & info,
								   image.data,
								   stream,
								   rs::core::image_interface::flag::any,
								   frame.get_timestamp(),
								   (uint64_t)frame.get_frame_number(),
								   rs::core::timestamp_domain::microcontroller);

			UScopeMutex lock(slamLock_);
			if (slam_->process_sample_set(sample_set) < rs::core::status_no_error)
			{
				UERROR("error: failed to process sample");
			}
			sample_set[stream]->release();
		}
	};

	// Setup stream callback for stream
	if(computeOdometry)
	{
		dev_->set_frame_callback(rs::stream::fisheye, frameCallback);
	}
	dev_->set_frame_callback(rs::stream::depth, frameCallback);
	dev_->set_frame_callback(rs::stream::color, frameCallback);

	if (computeOdometry)
	{
		dev_->enable_stream(rs::stream::fisheye, 640, 480, rs::format::raw8, 30);
		rs::intrinsics fisheye_intrin = dev_->get_stream_intrinsics(rs::stream::fisheye);
		UINFO("    Fish: %dx%d", fisheye_intrin.width, fisheye_intrin.height);

		// Needed to align image timestamps to common clock-domain with the motion events
		dev_->set_option(rs::option::fisheye_strobe, 1);
		// This option causes the fisheye image to be aquired in-sync with the depth image.
		dev_->set_option(rs::option::fisheye_external_trigger, 1);
		dev_->set_option(rs::option::fisheye_color_auto_exposure, 1);

		UDEBUG("Setup motion callback");
		//define callback to the motion events and set it.
		std::function<void(rs::motion_data)> motion_callback;
		motion_callback = [this](rs::motion_data entry)
		{
			if ((entry.timestamp_data.source_id != RS_EVENT_IMU_GYRO) &&
					(entry.timestamp_data.source_id != RS_EVENT_IMU_ACCEL))
				return;

			rs_event_source motionType = entry.timestamp_data.source_id;

			rs::core::correlated_sample_set sample_set = {};
			if (motionType == RS_EVENT_IMU_ACCEL)
			{
				sample_set[rs::core::motion_type::accel].timestamp = entry.timestamp_data.timestamp;
				sample_set[rs::core::motion_type::accel].data[0] = (float)entry.axes[0];
				sample_set[rs::core::motion_type::accel].data[1] = (float)entry.axes[1];
				sample_set[rs::core::motion_type::accel].data[2] = (float)entry.axes[2];
				sample_set[rs::core::motion_type::accel].type = rs::core::motion_type::accel;
				++motionSeq_[0];
				sample_set[rs::core::motion_type::accel].frame_number = motionSeq_[0];
			}
			else if (motionType == RS_EVENT_IMU_GYRO)
			{
				sample_set[rs::core::motion_type::gyro].timestamp = entry.timestamp_data.timestamp;
				sample_set[rs::core::motion_type::gyro].data[0] = (float)entry.axes[0];
				sample_set[rs::core::motion_type::gyro].data[1] = (float)entry.axes[1];
				sample_set[rs::core::motion_type::gyro].data[2] = (float)entry.axes[2];
				sample_set[rs::core::motion_type::gyro].type = rs::core::motion_type::gyro;
				++motionSeq_[1];
				sample_set[rs::core::motion_type::gyro].frame_number = motionSeq_[1];
			}

			UScopeMutex lock(slamLock_);
			if (slam_->process_sample_set(sample_set) < rs::core::status_no_error)
			{
				UERROR("error: failed to process sample");
			}
		};

		std::function<void(rs::timestamp_data)> timestamp_callback;
		timestamp_callback = [](rs::timestamp_data entry) {};

		dev_->enable_motion_tracking(motion_callback, timestamp_callback);
		UINFO("  enabled accel and gyro stream");

		rs::motion_intrinsics imuIntrinsics;
		rs::extrinsics fisheye2ImuExtrinsics;
		rs::extrinsics fisheye2DepthExtrinsics;
		try
		{
			imuIntrinsics = dev_->get_motion_intrinsics();
			fisheye2ImuExtrinsics = dev_->get_motion_extrinsics_from(rs::stream::fisheye);
			fisheye2DepthExtrinsics = dev_->get_extrinsics(rs::stream::depth, rs::stream::fisheye);
		}
		catch (const rs::error & e) {
			UERROR("Exception: %s (try to unplug/plug the camera)", e.what());
			return false;
		}

		UDEBUG("Setup SLAM");
		UScopeMutex lock(slamLock_);
		slam_ = new rs::slam::slam();
		slam_->set_auto_occupancy_map_building(false);
		slam_->force_relocalization_pose(false);

		rs::core::video_module_interface::supported_module_config supported_config = {};
		if (slam_->query_supported_module_config(0, supported_config) < rs::core::status_no_error)
		{
			UERROR("Failed to query the first supported module configuration");
			return false;
		}

		rs::core::video_module_interface::actual_module_config actual_config = {};

		// Set camera intrinsics
		std::map< rs::core::stream_type, rs::core::intrinsics > intrinsics;
		intrinsics[rs::core::stream_type::fisheye] = rs::utils::convert_intrinsics(fisheye_intrin);
		intrinsics[rs::core::stream_type::depth] = rs::utils::convert_intrinsics(depth_intrin);

		if(!setStreamConfigIntrin(rs::core::stream_type::fisheye, intrinsics, supported_config, actual_config))
		{
			return false;
		}
		if(!setStreamConfigIntrin(rs::core::stream_type::depth, intrinsics, supported_config, actual_config))
		{
			return false;
		}

		// Set IMU intrinsics
		actual_config[rs::core::motion_type::accel].is_enabled = true;
		actual_config[rs::core::motion_type::gyro].is_enabled = true;
		actual_config[rs::core::motion_type::gyro].intrinsics = rs::utils::convert_motion_device_intrinsics(imuIntrinsics.gyro);
		actual_config[rs::core::motion_type::accel].intrinsics = rs::utils::convert_motion_device_intrinsics(imuIntrinsics.acc);

		// Set extrinsics
		actual_config[rs::core::stream_type::fisheye].extrinsics_motion = rs::utils::convert_extrinsics(fisheye2ImuExtrinsics);
		actual_config[rs::core::stream_type::fisheye].extrinsics = rs::utils::convert_extrinsics(fisheye2DepthExtrinsics);

		UDEBUG("Set SLAM config");
		// Set actual config
		if (slam_->set_module_config(actual_config) < rs::core::status_no_error)
		{
			UERROR("error : failed to set the enabled module configuration");
			return false;
		}

		dev_->start(rs::source::all_sources);
	}
	else
	{
		dev_->start();
	}
#else
	dev_->start();
	try {
		dev_->wait_for_frames();
	}
	catch (const rs::error & e)
	{
		UERROR("Exception: %s", e.what());
	}
#endif

	uSleep(1000); // ignore the first frames
	UINFO("Enabling streams...done!");

	return true;

#else
	UERROR("CameraRealSense: RTAB-Map is not built with RealSense support!");
	return false;
#endif
}

bool CameraRealSense::isCalibrated() const
{
	return true;
}

std::string CameraRealSense::getSerial() const
{
#ifdef RTABMAP_REALSENSE
	if (dev_)
	{
		return dev_->get_serial();
	}
	else
	{
		UERROR("Cannot get a serial before initialization. Call init() before.");
	}
#endif
	return "NA";
}

bool CameraRealSense::odomProvided() const
{
#ifdef RTABMAP_REALSENSE_SLAM
	return slam_!=0;
#else
	return false;
#endif
}

#ifdef RTABMAP_REALSENSE_SLAM
Transform rsPoseToTransform(const rs::slam::PoseMatrix4f & pose)
{
	return Transform(
			pose.m_data[0], pose.m_data[1], pose.m_data[2], pose.m_data[3],
			pose.m_data[4], pose.m_data[5], pose.m_data[6], pose.m_data[7],
			pose.m_data[8], pose.m_data[9], pose.m_data[10], pose.m_data[11]);
}
#endif

SensorData CameraRealSense::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_REALSENSE
	if (dev_)
	{
		cv::Mat rgb;
		cv::Mat depthIn;

		// Retrieve camera parameters for mapping between depth and color
		rs::intrinsics depth_intrin = dev_->get_stream_intrinsics(rs::stream::depth);
		rs::extrinsics depth_to_color = dev_->get_extrinsics(rs::stream::depth, rs::stream::color);
		rs::intrinsics color_intrin = dev_->get_stream_intrinsics(rs::stream::color);

#ifdef RTABMAP_REALSENSE_SLAM
		if(!dataReady_.acquire(1, 5000))
		{
			UWARN("Not received new frames since 5 seconds, end of stream reached!");
			return data;
		}
		{
			UScopeMutex lock(dataMutex_);
			rgb = lastSyncFrames_.first;
			depthIn = lastSyncFrames_.second;
			lastSyncFrames_.first = cv::Mat();
			lastSyncFrames_.second = cv::Mat();
		}

		if(rgb.empty() || depthIn.empty())
		{
			return data;
		}
#else
		try {
			dev_->wait_for_frames();
		}
		catch (const rs::error & e)
		{
			UERROR("Exception: %s", e.what());
			return data;
		}

		// Retrieve our images
		depthIn = cv::Mat(depth_intrin.height, depth_intrin.width, CV_16UC1, (unsigned char*)dev_->get_frame_data(rs::stream::depth));
		rgb = cv::Mat(color_intrin.height, color_intrin.width, CV_8UC3, (unsigned char*)dev_->get_frame_data(rs::stream::color));
#endif

		float scale = dev_->get_depth_scale();

		// factory registration...
		cv::Mat bgr;
		cv::cvtColor(rgb, bgr, CV_RGB2BGR);

		CameraModel model(
			color_intrin.fx, //fx
			color_intrin.fy, //fy
			color_intrin.ppx,  //cx
			color_intrin.ppy,  //cy
			this->getLocalTransform(),
			0,
			bgr.size());

		cv::Mat depth;
		if (color_intrin.width % depth_intrin.width == 0 && color_intrin.height % depth_intrin.height == 0 &&
			depth_intrin.width < color_intrin.width &&
			depth_intrin.height < color_intrin.height)
		{
			//we can keep the depth image size as is
			depth = cv::Mat::zeros(cv::Size(depth_intrin.width, depth_intrin.height), CV_16UC1);
			float scaleX = float(depth_intrin.width) / float(color_intrin.width);
			float scaleY = float(depth_intrin.height) / float(color_intrin.height);
			color_intrin.fx *= scaleX;
			color_intrin.fy *= scaleY;
			color_intrin.ppx *= scaleX;
			color_intrin.ppy *= scaleY;
			color_intrin.height = depth_intrin.height;
			color_intrin.width = depth_intrin.width;
		}
		else
		{
			//depth to color
			depth = cv::Mat::zeros(bgr.size(), CV_16UC1);
		}
		for (int dy = 0; dy < depth_intrin.height; ++dy)
		{
			for (int dx = 0; dx < depth_intrin.width; ++dx)
			{
				// Retrieve the 16-bit depth value and map it into a depth in meters
				uint16_t depth_value = depthIn.at<unsigned short>(dy,dx);
				float depth_in_meters = depth_value * scale;

				// Skip over pixels with a depth value of zero, which is used to indicate no data
				if (depth_value == 0 || depth_in_meters>10.0f) continue;

				// Map from pixel coordinates in the depth image to pixel coordinates in the color image
				rs::float2 depth_pixel = { (float)dx, (float)dy };
				rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
				rs::float3 color_point = depth_to_color.transform(depth_point);
				rs::float2 color_pixel = color_intrin.project(color_point);

				int pdx = color_pixel.x;
				int pdy = color_pixel.y;
				if (uIsInBounds(pdx, 0, depth.cols) && uIsInBounds(pdy, 0, depth.rows))
				{
					depth.at<unsigned short>(pdy, pdx) = (unsigned short)(depth_in_meters*1000.0f); // convert to mm
				}
			}
		}

		if (color_intrin.width > depth_intrin.width)
		{
			// Fill holes
			UTimer time;
			util2d::fillRegisteredDepthHoles(depth, true, true, color_intrin.width > depth_intrin.width * 2);
			util2d::fillRegisteredDepthHoles(depth, true, true, color_intrin.width > depth_intrin.width * 2);//second pass
			UDEBUG("Filling depth holes: %fs", time.ticks());
		}
		
		if (!bgr.empty() && !depth.empty())
		{
			data = SensorData(bgr, depth, model, this->getNextSeqID(), UTimer::now());
#ifdef RTABMAP_REALSENSE_SLAM
			if(info && slam_)
			{
				UScopeMutex lock(slamLock_);
				rs::slam::PoseMatrix4f pose;
				if(slam_->get_camera_pose(pose) == rs::core::status_no_error)
				{
					Transform opticalRotation(0,0,1,0, -1,0,0,0, 0,-1,0,0);
					info->odomPose = opticalRotation * rsPoseToTransform(pose) * opticalRotation.inverse();
				}
				else
				{
					UERROR("Failed getting odometry pose");
				}
			}
#endif
		}
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}
#else
	UERROR("CameraRealSense: RTAB-Map is not built with RealSense support!");
#endif
	return data;
}

/////////////////////////
// CameraRealSense2
/////////////////////////
bool CameraRealSense2::available()
{
#ifdef RTABMAP_REALSENSE2
	return true;
#else
	return false;
#endif
}

CameraRealSense2::CameraRealSense2(
		const std::string & device,
		float imageRate,
		const rtabmap::Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_REALSENSE2
    ,
	deviceId_(device),
	depth_scale_meters_(1.0f),
	emitterEnabled_(true),
	irDepth_(false)
#endif
{
	UDEBUG("");
}

CameraRealSense2::~CameraRealSense2()
{
	UDEBUG("");
}

#ifdef RTABMAP_REALSENSE2
void CameraRealSense2::alignFrame(const rs2_intrinsics& from_intrin,
                                   const rs2_intrinsics& other_intrin,
                                   rs2::frame from_image,
                                   uint32_t output_image_bytes_per_pixel,
                                   const rs2_extrinsics& from_to_other,
								   cv::Mat & registeredDepth)
{
    static const auto meter_to_mm = 0.001f;
    uint8_t* p_out_frame = registeredDepth.data;
    auto from_vid_frame = from_image.as<rs2::video_frame>();
    auto from_bytes_per_pixel = from_vid_frame.get_bytes_per_pixel();

    static const auto blank_color = 0x00;
    UASSERT(registeredDepth.total()*registeredDepth.channels()*registeredDepth.depth() == other_intrin.height * other_intrin.width * output_image_bytes_per_pixel);
    memset(p_out_frame, blank_color, other_intrin.height * other_intrin.width * output_image_bytes_per_pixel);

    auto p_from_frame = reinterpret_cast<const uint8_t*>(from_image.get_data());
    auto from_stream_type = from_image.get_profile().stream_type();
    float depth_units = ((from_stream_type == RS2_STREAM_DEPTH)?depth_scale_meters_:1.f);
    UASSERT(from_stream_type == RS2_STREAM_DEPTH);
    UASSERT_MSG(depth_units > 0.0f, uFormat("depth_scale_meters_=%f", depth_scale_meters_).c_str());
#pragma omp parallel for schedule(dynamic)
    for (int from_y = 0; from_y < from_intrin.height; ++from_y)
    {
        int from_pixel_index = from_y * from_intrin.width;
        for (int from_x = 0; from_x < from_intrin.width; ++from_x, ++from_pixel_index)
        {
            // Skip over depth pixels with the value of zero
            float depth = (from_stream_type == RS2_STREAM_DEPTH)?(depth_units * ((const uint16_t*)p_from_frame)[from_pixel_index]): 1.f;
            if (depth)
            {
                // Map the top-left corner of the depth pixel onto the other image
                float from_pixel[2] = { from_x - 0.5f, from_y - 0.5f }, from_point[3], other_point[3], other_pixel[2];
                rs2_deproject_pixel_to_point(from_point, &from_intrin, from_pixel, depth);
                rs2_transform_point_to_point(other_point, &from_to_other, from_point);
                rs2_project_point_to_pixel(other_pixel, &other_intrin, other_point);
                const int other_x0 = static_cast<int>(other_pixel[0] + 0.5f);
                const int other_y0 = static_cast<int>(other_pixel[1] + 0.5f);

                // Map the bottom-right corner of the depth pixel onto the other image
                from_pixel[0] = from_x + 0.5f; from_pixel[1] = from_y + 0.5f;
                rs2_deproject_pixel_to_point(from_point, &from_intrin, from_pixel, depth);
                rs2_transform_point_to_point(other_point, &from_to_other, from_point);
                rs2_project_point_to_pixel(other_pixel, &other_intrin, other_point);
                const int other_x1 = static_cast<int>(other_pixel[0] + 0.5f);
                const int other_y1 = static_cast<int>(other_pixel[1] + 0.5f);

                if (other_x0 < 0 || other_y0 < 0 || other_x1 >= other_intrin.width || other_y1 >= other_intrin.height)
                    continue;

                for (int y = other_y0; y <= other_y1; ++y)
                {
                    for (int x = other_x0; x <= other_x1; ++x)
                    {
                        int out_pixel_index = y * other_intrin.width + x;
                        //Tranfer n-bit pixel to n-bit pixel
                        for (int i = 0; i < from_bytes_per_pixel; i++)
                        {
                            const auto out_offset = out_pixel_index * output_image_bytes_per_pixel + i;
                            const auto from_offset = from_pixel_index * output_image_bytes_per_pixel + i;
                            p_out_frame[out_offset] = p_from_frame[from_offset] * (depth_units / meter_to_mm);
                        }
                    }
                }
            }
        }
    }
}
#endif

bool CameraRealSense2::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_REALSENSE2

	UINFO("setupDevice...");

	auto list = ctx_.query_devices();
	if (0 == list.size())
	{
		UERROR("No RealSense2 devices were found!");
		return false;
	}

	bool found=false;
	for (auto&& dev : list)
	{
		auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		auto pid_str = dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
		uint16_t pid;
		std::stringstream ss;
		ss << std::hex << pid_str;
		ss >> pid;
		UINFO("Device with serial number %s was found with product ID=%d.", sn, (int)pid);
		if (deviceId_.empty() || deviceId_ == sn)
		{
			dev_ = dev;
			found=true;
			break;
		}
	}

	if (!found)
	{
		UERROR("The requested device %s is NOT found!", deviceId_.c_str());
		return false;
	}

	ctx_.set_devices_changed_callback([this](rs2::event_information& info)
	{
		if (info.was_removed(dev_))
		{
			UERROR("The device has been disconnected!");
		}
	});


	auto camera_name = dev_.get_info(RS2_CAMERA_INFO_NAME);
	UINFO("Device Name: %s", camera_name);

	auto sn = dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	UINFO("Device Serial No: %s", sn);

	auto fw_ver = dev_.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
	UINFO("Device FW version: %s", fw_ver);

	auto pid = dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
	UINFO("Device Product ID: 0x%s", pid);

	auto dev_sensors = dev_.query_sensors();

	UINFO("Device Sensors: ");
	std::vector<rs2::sensor> sensors(2); //0=rgb 1=depth
	for(auto&& elem : dev_sensors)
	{
		std::string module_name = elem.get_info(RS2_CAMERA_INFO_NAME);
		if ("Stereo Module" == module_name)
		{
			sensors[1] = elem;
			sensors[1].set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, emitterEnabled_);
			if(irDepth_)
			{
				sensors[0] = elem;
			}
		}
		else if ("Coded-Light Depth Sensor" == module_name)
		{
		}
		else if ("RGB Camera" == module_name)
		{
			if(!irDepth_)
			{
				sensors[0] = elem;
			}
		}
		else if ("Wide FOV Camera" == module_name)
		{
		}
		else if ("Motion Module" == module_name)
		{
		}
		else
		{
			UERROR("Module Name \"%s\" isn't supported by LibRealSense!", module_name.c_str());
			return false;
		}
		UINFO("%s was found.", elem.get_info(RS2_CAMERA_INFO_NAME));
	}

	UDEBUG("");

	model_ = CameraModel();
	rs2::stream_profile depthStreamProfile;
	rs2::stream_profile rgbStreamProfile;
	std::vector<std::vector<rs2::stream_profile> > profilesPerSensor(2);
	 for (unsigned int i=0; i<sensors.size(); ++i)
	 {
		 UDEBUG("i=%d", (int)i);
		auto profiles = sensors[i].get_stream_profiles();
		bool added = false;
		UDEBUG("profiles=%d", (int)profiles.size());
		for (auto& profile : profiles)
		{
			auto video_profile = profile.as<rs2::video_stream_profile>();
			if (video_profile.format() == (i==1?RS2_FORMAT_Z16:irDepth_?RS2_FORMAT_Y8:RS2_FORMAT_RGB8) &&
				video_profile.width()  == 640 &&
				video_profile.height() == 480 &&
				video_profile.fps()    == 30)
			{
				profilesPerSensor[irDepth_?1:i].push_back(profile);
				auto intrinsic = video_profile.get_intrinsics();
				if(i==1)
				{
					depthBuffer_ = cv::Mat(cv::Size(640, 480), CV_16UC1, cv::Scalar(0));
					depthStreamProfile = profile;
					depthIntrinsics_ = intrinsic;
				}
				else
				{
					rgbBuffer_ = cv::Mat(cv::Size(640, 480), irDepth_?CV_8UC1:CV_8UC3, irDepth_?cv::Scalar(0):cv::Scalar(0, 0, 0));
					model_ = CameraModel(camera_name, intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy, this->getLocalTransform(), 0, cv::Size(intrinsic.width, intrinsic.height));
					rgbStreamProfile = profile;
					rgbIntrinsics_ = intrinsic;
				}
				added = true;
				break;
			}
		}
		if (!added)
		{
			UERROR("Given stream configuration is not supported by the device! "
					"Stream Index: %d, Width: %d, Height: %d, FPS: %d", i, 640, 480, 30);
			return false;
		}
	 }

	 if(!model_.isValidForProjection())
	 {
		 UERROR("Calibration info not valid!");
		 return false;
	 }
	 depthToRGBExtrinsics_ = depthStreamProfile.get_extrinsics_to(rgbStreamProfile);

	 for (unsigned int i=0; i<sensors.size(); ++i)
	 {
		 if(profilesPerSensor[i].size())
		 {
			 UINFO("Starting sensor %d with %d profiles", (int)i, (int)profilesPerSensor[i].size());
			 sensors[i].open(profilesPerSensor[i]);
			 if(i ==1)
			 {
				 auto depth_sensor = sensors[i].as<rs2::depth_sensor>();
				 depth_scale_meters_ = depth_sensor.get_depth_scale();
			 }
			 sensors[i].start(syncer_);
		 }
	 }

	uSleep(1000); // ignore the first frames
	UINFO("Enabling streams...done!");

	return true;

#else
	UERROR("CameraRealSense: RTAB-Map is not built with RealSense2 support!");
	return false;
#endif
}

bool CameraRealSense2::isCalibrated() const
{
	return true;
}

std::string CameraRealSense2::getSerial() const
{
#ifdef RTABMAP_REALSENSE2
	return dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
#endif
	return "NA";
}

void CameraRealSense2::setEmitterEnabled(bool enabled)
{
#ifdef RTABMAP_REALSENSE2
	emitterEnabled_ = enabled;
#endif
}

void CameraRealSense2::setIRDepthFormat(bool enabled)
{
#ifdef RTABMAP_REALSENSE2
	irDepth_ = enabled;
#endif
}

SensorData CameraRealSense2::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_REALSENSE2

	try{
		auto frameset = syncer_.wait_for_frames(5000);
		UTimer timer;
		while (frameset.size() != 2 && timer.elapsed() < 2.0)
		{
			// maybe there is a latency with the USB, try again in 10 ms (for the next 2 seconds)
			frameset = syncer_.wait_for_frames(10);
		}
		if (frameset.size() == 2)
		{
			double stamp = UTimer::now();
			UDEBUG("Frameset arrived.");
			bool is_rgb_arrived = false;
			bool is_depth_arrived = false;
			rs2::frame rgb_frame;
			rs2::frame depth_frame;
			for (auto it = frameset.begin(); it != frameset.end(); ++it)
			{
				auto f = (*it);
				auto stream_type = f.get_profile().stream_type();
				if (stream_type == RS2_STREAM_COLOR || stream_type == RS2_STREAM_INFRARED)
				{
					rgb_frame = f;
					is_rgb_arrived = true;
				}
				else if (stream_type == RS2_STREAM_DEPTH)
				{
					depth_frame = f;
					is_depth_arrived = true;
				}
			}

			if(is_rgb_arrived && is_depth_arrived)
			{
				auto from_image_frame = depth_frame.as<rs2::video_frame>();
				cv::Mat depth;
				if(irDepth_)
				{
					depth = cv::Mat(depthBuffer_.size(), depthBuffer_.type(), (void*)depth_frame.get_data()).clone();
				}
				else
				{
					depth = cv::Mat(depthBuffer_.size(), depthBuffer_.type());
					alignFrame(depthIntrinsics_, rgbIntrinsics_,
							depth_frame, from_image_frame.get_bytes_per_pixel(),
							depthToRGBExtrinsics_, depth);
				}

				cv::Mat rgb = cv::Mat(rgbBuffer_.size(), rgbBuffer_.type(), (void*)rgb_frame.get_data());
				cv::Mat bgr;
				if(rgb.channels() == 3)
				{
					cv::cvtColor(rgb, bgr, CV_RGB2BGR);
				}
				else
				{
					bgr = rgb.clone();
				}

				data = SensorData(bgr, depth, model_, this->getNextSeqID(), stamp);
			}
			else
			{
				UERROR("Not received depth and rgb");
			}
		}
		else
		{
			UERROR("Missing frames (received %d)", (int)frameset.size());
		}
	}
	catch(const std::exception& ex)
	{
		UERROR("An error has occurred during frame callback: %s", ex.what());
	}
#else
	UERROR("CameraRealSense2: RTAB-Map is not built with RealSense2 support!");
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
