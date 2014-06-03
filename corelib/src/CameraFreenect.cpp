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
	rgbReady_(false)
{
	UASSERT(ctx_ != 0);
}

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
		UERROR("Cannot open Kinect");
		return false;
	}
	freenect_set_user(device_, this);
	freenect_set_video_mode(device_, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
	freenect_set_depth_mode(device_, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED));
	freenect_set_depth_callback(device_, freenect_depth_callback);
	freenect_set_video_callback(device_, freenect_video_callback);
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
// CameraOpenKinect
//
CameraFreenect::CameraFreenect(int deviceId, float inputRate, const Transform & localTransform) :
		deviceId_(deviceId),
		rate_(inputRate),
		frameRateTimer_(new UTimer()),
		localTransform_(localTransform),
		seq_(0),
		ctx_(0),
		freenectDevice_(0)
{
	if(freenect_init(&ctx_, NULL) < 0) UERROR("Cannot initialize freenect library");
	// We claim both the motor and camera devices, since this class exposes both.
	// It does not support audio, so we do not claim it.
	freenect_select_subdevices(ctx_, static_cast<freenect_device_flags>(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
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
	if(freenect_shutdown(ctx_) < 0){} //FN_WARNING("Freenect did not shutdown in a clean fashion");
	delete frameRateTimer_;
}

bool CameraFreenect::init()
{
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
			UERROR("CameraOpenKinect: No devices connected!");
		}
	}
	else
	{
		UERROR("CameraOpenKinect: Cannot initialize the camera because it is already running...");
	}
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
		UERROR("CameraOpenKinect: init should be called before starting the camera.");
	}
}

void CameraFreenect::mainLoop()
{
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
				UWARN("CameraOpenKinect: Depth not ready! Try to reduce the image rate to avoid this warning...");
				return;
			}

			if(rgb.empty())
			{
				UWARN("CameraOpenKinect: Rgb not ready! Try to reduce the image rate to avoid this warning...");
				return;
			}

			float constant = 0.001905f;
			this->post(new CameraEvent(rgb, depth, constant, localTransform_, ++seq_));
		}
	}
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
