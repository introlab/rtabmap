/*
 * CameraOpenni.cpp
 *
 *  Created on: 2013-08-22
 *      Author: Mathieu
 */

#include "rtabmap/core/CameraOpenni.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/utilite/ULogger.h"
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/openni_grabber.h>

namespace rtabmap {

CameraOpenni::CameraOpenni(const std::string & deviceId, float inputRate, const Transform & localTransform) :
		interface_(0),
		deviceId_(deviceId),
		rate_(inputRate),
		frameRateTimer_(new UTimer()),
		localTransform_(localTransform),
		seq_(0)
{
}

CameraOpenni::~CameraOpenni()
{
	UDEBUG("");
	kill();
	delete frameRateTimer_;
	if(interface_)
	{
		uSleep(100); // make sure it is stopped
		delete interface_;
		interface_ = 0;
	}
}

void CameraOpenni::image_cb (
		const boost::shared_ptr<openni_wrapper::Image>& rgb,
		const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
		float constant)
{
	if(rate_>0.0f)
	{
		if(frameRateTimer_->getElapsedTime() < 1.0f/rate_)
		{
			return;
		}
	}
	frameRateTimer_->start();

	UTimer t;

	cv::Mat rgbFrame(rgb->getHeight(), rgb->getWidth(), CV_8UC3);
	rgb->fillRGB(rgb->getWidth(), rgb->getHeight(), rgbFrame.data);
	cv::Mat bgrFrame;
	cv::cvtColor(rgbFrame, bgrFrame, CV_RGB2BGR);

	cv::Mat depthFrame(rgb->getHeight(), rgb->getWidth(), CV_16UC1);
	depth->fillDepthImageRaw(rgb->getWidth(), rgb->getHeight(), (unsigned short*)depthFrame.data);

	this->post(new CameraEvent(bgrFrame, depthFrame, constant, localTransform_, ++seq_));
}

bool CameraOpenni::init()
{
	if(interface_ && interface_->isRunning())
	{
		UERROR("Already started!!!\n");
		return false;
	}
	else if(interface_)
	{
		delete interface_;
		interface_ = 0;
	}

	seq_ = 0;
	try
	{
		interface_ = new pcl::OpenNIGrabber(deviceId_);
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

	frameRateTimer_->start();
	return true;
}

void CameraOpenni::start()
{
	if(interface_)
	{
		if(!connection_.connected())
		{
			boost::function<void (
					const boost::shared_ptr<openni_wrapper::Image>&,
					const boost::shared_ptr<openni_wrapper::DepthImage>&,
					float)> f = boost::bind (&CameraOpenni::image_cb, this, _1, _2, _3);
			connection_ = interface_->registerCallback (f);
		}

		if(!interface_->isRunning())
		{
			interface_->start ();
		}
	}
}

void CameraOpenni::pause()
{
	if(connection_.connected())
	{
		connection_.disconnect();
	}
}

void CameraOpenni::kill()
{
	UDEBUG("");
	if(interface_)
	{
		interface_->stop();
	}
}

bool CameraOpenni::isRunning()
{
	return (interface_ && interface_->isRunning());
}

void CameraOpenni::setFrameRate(float rate)
{
	rate_ = rate;
}

} /* namespace rtabmap */
