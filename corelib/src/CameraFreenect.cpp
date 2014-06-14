/*
 * CameraFreenect.cpp
 *
 *  Created on: 2014-06-02
 *      Author: Mathieu
 */

#include "rtabmap/core/CameraFreenect.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/utilite/ULogger.h"
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef WITH_FREENECT
#include <libfreenect.h>
#include <libfreenect-registration.h>
#endif

namespace rtabmap {

//
// FreenectDevice
//
FreenectDevice::FreenectDevice(freenect_context * ctx, int index) :
	index_(index),
	ctx_(ctx),
	device_(0),
	depthMat_(cv::Size(640,480),CV_16UC1),
	rgbMat_(cv::Size(640,480), CV_8UC3, cv::Scalar(0)),
	depthReady_(false),
	rgbReady_(false),
	depthFocal_(0.0f)
{
	UASSERT(ctx_ != 0);
}

#ifdef WITH_FREENECT
FreenectDevice::~FreenectDevice() {
	if(device_ && freenect_close_device(device_) < 0){} //FN_WARNING("Device did not shutdown in a clean fashion");
}
void FreenectDevice::startVideo() {
	if(device_ && freenect_start_video(device_) < 0) UERROR("Cannot start RGB callback");
}
void FreenectDevice::stopVideo() {
	if(device_ && freenect_stop_video(device_) < 0) UERROR("Cannot stop RGB callback");
}
void FreenectDevice::startDepth() {
	if(device_ && freenect_start_depth(device_) < 0) UERROR("Cannot start depth callback");
}
void FreenectDevice::stopDepth() {
	if(device_ && freenect_stop_depth(device_) < 0) UERROR("Cannot stop depth callback");
}

bool FreenectDevice::init()
{
	if(freenect_open_device(ctx_, &device_, index_) < 0)
	{
		UERROR("FreenectDevice: Cannot open Kinect");
		return false;
	}
	freenect_set_user(device_, this);
	freenect_set_video_mode(device_, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
	freenect_set_depth_mode(device_, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED));
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

void FreenectDevice::freenect_depth_callback(freenect_device *dev, void *depth, uint32_t timestamp) {
	FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
	device->DepthCallback(depth, timestamp);
}
void FreenectDevice::freenect_video_callback(freenect_device *dev, void *video, uint32_t timestamp) {
	FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
	device->VideoCallback(video, timestamp);
}
#else
FreenectDevice::~FreenectDevice() {}
void FreenectDevice::startVideo() {}
void FreenectDevice::stopVideo() {}
void FreenectDevice::startDepth() {}
void FreenectDevice::stopDepth() {}

bool FreenectDevice::init()
{
	UERROR("RTAB-Map is not built with Freenect support!");
	return false;
}
void FreenectDevice::freenect_depth_callback(freenect_device *dev, void *depth, uint32_t timestamp) {}
void FreenectDevice::freenect_video_callback(freenect_device *dev, void *video, uint32_t timestamp) {}
#endif

// Do not call directly even in child
void FreenectDevice::VideoCallback(void* _rgb, uint32_t timestamp)
{
	rgbMutex_.lock();
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);
	rgbMat_.data = rgb;
	rgbReady_ = true;
	rgbMutex_.unlock();
}

// Do not call directly even in child
void FreenectDevice::DepthCallback(void* _depth, uint32_t timestamp)
{
	depthMutex_.lock();
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	depthMat_.data = (uchar*) depth;
	depthReady_ = true;
	depthMutex_.unlock();
}

cv::Mat FreenectDevice::getRgb()
{
	cv::Mat out;
	rgbMutex_.lock();
	if(rgbReady_)
	{
		cv::cvtColor(rgbMat_, out, CV_RGB2BGR);
		rgbReady_ = false;
	}
	rgbMutex_.unlock();
	return out;
}

cv::Mat FreenectDevice::getDepth()
{
	cv::Mat out;
	depthMutex_.lock();
	if(depthReady_)
	{
		depthMat_.copyTo(out);
		depthReady_ = false;
	}
	depthMutex_.unlock();
	return out;
}


//
// CameraFreenect
//
bool CameraFreenect::available()
{
#ifdef WITH_FREENECT
	return true;
#else
	return false;
#endif
}

CameraFreenect::CameraFreenect(int deviceId, float inputRate, const Transform & localTransform) :
		deviceId_(deviceId),
		rate_(inputRate),
		frameRateTimer_(new UTimer()),
		localTransform_(localTransform),
		seq_(0),
		ctx_(0),
		freenectDevice_(0)
{
#ifdef WITH_FREENECT
	if(freenect_init(&ctx_, NULL) < 0) UERROR("Cannot initialize freenect library");
	// We claim both the motor and camera devices, since this class exposes both.
	// It does not support audio, so we do not claim it.
	freenect_select_subdevices(ctx_, static_cast<freenect_device_flags>(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
#endif
}

CameraFreenect::~CameraFreenect()
{
	UDEBUG("");
	join(true);
	if(freenectDevice_)
	{
		delete freenectDevice_;
		freenectDevice_ = 0;
	}
#ifdef WITH_FREENECT
	if(freenect_shutdown(ctx_) < 0){} //FN_WARNING("Freenect did not shutdown in a clean fashion");
#endif
	delete frameRateTimer_;
}

bool CameraFreenect::init()
{
#ifdef WITH_FREENECT
	if(!this->isRunning())
	{
		if(freenectDevice_)
		{
			delete freenectDevice_;
			freenectDevice_ = 0;
		}

		seq_ = 0;

		if(freenect_num_devices(ctx_) > 0)
		{
			freenectDevice_ = new FreenectDevice(ctx_, deviceId_);
			if(freenectDevice_->init())
			{
				return true;
			}
			delete freenectDevice_;
			freenectDevice_ = 0;
		}
		else
		{
			UERROR("CameraFreenect: No devices connected!");
		}
	}
	else
	{
		UERROR("CameraFreenect: Cannot initialize the camera because it is already running...");
	}
#else
	UERROR("CameraFreenect: RTAB-Map is not built with Freenect support!");
#endif
	return false;
}


void CameraFreenect::setFrameRate(float rate)
{
	rate_ = rate;
}

void CameraFreenect::mainLoopBegin()
{
	if(freenectDevice_)
	{
		freenectDevice_->startDepth();
		freenectDevice_->startVideo();
		frameRateTimer_->start();
	}
	else
	{
		UERROR("CameraFreenect: init should be called before starting the camera.");
		this->kill();
	}
}

void CameraFreenect::mainLoop()
{
#ifdef WITH_FREENECT
	timeval t;
	t.tv_sec = 0;
	t.tv_usec = 10000;
	if(freenect_process_events_timeout(ctx_, &t) < 0) UERROR("Cannot process freenect events");

	if(freenectDevice_ && !this->isKilled())
	{
		float imageRate = rate_==0.0f?33.0f:rate_; // limit to 33Hz if infinity

		if(frameRateTimer_->getElapsedTime() >= 1.0/double(imageRate)-0.000001)
		{
			double slept = frameRateTimer_->getElapsedTime();
			frameRateTimer_->start();
			UDEBUG("slept=%fs vs target=%fs", slept, 1.0/double(imageRate));

			cv::Mat depth = freenectDevice_->getDepth();
			cv::Mat rgb = freenectDevice_->getRgb();

			if(depth.empty())
			{
				UWARN("CameraFreenect: Depth not ready! Try to reduce the image rate to avoid this warning...");
				return;
			}

			if(rgb.empty())
			{
				UWARN("CameraFreenect: Rgb not ready! Try to reduce the image rate to avoid this warning...");
				return;
			}
			UASSERT(freenectDevice_->getDepthFocal() != 0.0f);
			float constant = 1.0f/freenectDevice_->getDepthFocal();
			this->post(new CameraEvent(rgb, depth, constant, localTransform_, ++seq_));
		}
	}
#endif
}

void CameraFreenect::mainLoopEnd()
{
	if(freenectDevice_)
	{
		freenectDevice_->stopDepth();
		freenectDevice_->stopVideo();
	}
}

} /* namespace rtabmap */
