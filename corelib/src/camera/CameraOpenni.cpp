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

#include <rtabmap/core/camera/CameraOpenni.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UThreadC.h>
#include <rtabmap/utilite/UTimer.h>
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_OPENNI
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_image.h>
#endif

namespace rtabmap
{

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

} // namespace rtabmap
