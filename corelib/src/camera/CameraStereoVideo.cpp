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

#include <rtabmap/core/camera/CameraStereoVideo.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/imgproc/types_c.h>
#if CV_MAJOR_VERSION > 3
#include <opencv2/videoio/videoio_c.h>
#endif

namespace rtabmap
{

bool CameraStereoVideo::available()
{
	return true;
}

CameraStereoVideo::CameraStereoVideo(
		const std::string & path,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
		Camera(imageRate, localTransform),
		path_(path),
		rectifyImages_(rectifyImages),
		src_(CameraVideo::kVideoFile),
		usbDevice_(0),
		usbDevice2_(-1)
{
}

CameraStereoVideo::CameraStereoVideo(
		const std::string & pathLeft,
		const std::string & pathRight,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
		Camera(imageRate, localTransform),
		path_(pathLeft),
		path2_(pathRight),
		rectifyImages_(rectifyImages),
		src_(CameraVideo::kVideoFile),
		usbDevice_(0),
		usbDevice2_(-1)
{
}

CameraStereoVideo::CameraStereoVideo(
	int device,
	bool rectifyImages,
	float imageRate,
	const Transform & localTransform) :
	Camera(imageRate, localTransform),
	rectifyImages_(rectifyImages),
	src_(CameraVideo::kUsbDevice),
	usbDevice_(device),
	usbDevice2_(-1)
{
}

CameraStereoVideo::CameraStereoVideo(
	int deviceLeft,
	int deviceRight,
	bool rectifyImages,
	float imageRate,
	const Transform & localTransform) :
	Camera(imageRate, localTransform),
	rectifyImages_(rectifyImages),
	src_(CameraVideo::kUsbDevice),
	usbDevice_(deviceLeft),
	usbDevice2_(deviceRight)
{
}

CameraStereoVideo::~CameraStereoVideo()
{
	capture_.release();
	capture2_.release();
}

bool CameraStereoVideo::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	cameraName_ = cameraName;
	if(capture_.isOpened())
	{
		capture_.release();
	}
	if(capture2_.isOpened())
	{
		capture2_.release();
	}

	if (src_ == CameraVideo::kUsbDevice)
	{
		capture_.open(usbDevice_);
		if(usbDevice2_ < 0)
		{
			ULOGGER_DEBUG("CameraStereoVideo: Usb device initialization on device %d", usbDevice_);
		}
		else
		{
			ULOGGER_DEBUG("CameraStereoVideo: Usb device initialization on devices %d and %d", usbDevice_, usbDevice2_);
			capture2_.open(usbDevice2_);
		}
	}
	else if (src_ == CameraVideo::kVideoFile)
	{
		capture_.open(path_.c_str());
		if(path2_.empty())
		{
			ULOGGER_DEBUG("CameraStereoVideo: filename=\"%s\"", path_.c_str());
		}
		else
		{
			ULOGGER_DEBUG("CameraStereoVideo: filenames=\"%s\" and \"%s\"", path_.c_str(), path2_.c_str());
			capture2_.open(path2_.c_str());
		}
	}
	else
	{
		ULOGGER_ERROR("CameraStereoVideo: Unknown source...");
	}

	if(!capture_.isOpened() || ((!path2_.empty() || usbDevice2_>=0) && !capture2_.isOpened()))
	{
		ULOGGER_ERROR("CameraStereoVideo: Failed to create a capture object!");
		capture_.release();
		capture2_.release();
		return false;
	}

	if (cameraName_.empty())
	{
		unsigned int guid = (unsigned int)capture_.get(CV_CAP_PROP_GUID);
		if (guid != 0 && guid != 0xffffffff)
		{
			cameraName_ = uFormat("%08x", guid);
		}
	}

	// look for calibration files
	if(!calibrationFolder.empty() && !cameraName_.empty())
	{
		if(!stereoModel_.load(calibrationFolder, cameraName_, false))
		{
			UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
				cameraName_.c_str(), calibrationFolder.c_str());
		}
		else
		{
			UINFO("Stereo parameters: fx=%f cx=%f cy=%f baseline=%f",
					stereoModel_.left().fx(),
					stereoModel_.left().cx(),
					stereoModel_.left().cy(),
					stereoModel_.baseline());
		}
	}

	stereoModel_.setLocalTransform(this->getLocalTransform());
	if(rectifyImages_ && !stereoModel_.isValidForRectification())
	{
		UERROR("Parameter \"rectifyImages\" is set, but no stereo model is loaded or valid.");
		return false;
	}
	return true;
}

bool CameraStereoVideo::isCalibrated() const
{
	return stereoModel_.isValidForProjection();
}

std::string CameraStereoVideo::getSerial() const
{
	return cameraName_;
}

SensorData CameraStereoVideo::captureImage(CameraInfo * info)
{
	SensorData data;

	cv::Mat img;
	if(capture_.isOpened() && ((path2_.empty() && usbDevice2_ < 0) || capture2_.isOpened()))
	{
		cv::Mat leftImage;
		cv::Mat rightImage;
		if(path2_.empty() && usbDevice2_ < 0)
		{
			if(!capture_.read(img))
			{
				return data;
			}
			// Side by side stream
			leftImage = cv::Mat(img, cv::Rect( 0, 0, img.size().width/2, img.size().height ));
			rightImage = cv::Mat(img, cv::Rect( img.size().width/2, 0, img.size().width/2, img.size().height ));
		}
		else if(!capture_.read(leftImage) || !capture2_.read(rightImage))
		{
			return data;
		}
		else if(leftImage.cols != rightImage.cols || leftImage.rows != rightImage.rows)
		{
			UERROR("Left and right streams don't have image of the same size: left=%dx%d right=%dx%d",
					leftImage.cols, leftImage.rows, rightImage.cols, rightImage.rows);
			return data;
		}

		// Rectification
		bool rightCvt = false;
		if(rightImage.type() != CV_8UC1)
		{
			cv::Mat tmp;
			cv::cvtColor(rightImage, tmp, CV_BGR2GRAY);
			rightImage = tmp;
			rightCvt = true;
		}

		if(rectifyImages_ && stereoModel_.left().isValidForRectification() && stereoModel_.right().isValidForRectification())
		{
			leftImage = stereoModel_.left().rectifyImage(leftImage);
			rightImage = stereoModel_.right().rectifyImage(rightImage);
		}
		else
		{
			leftImage = leftImage.clone();
			if(!rightCvt)
			{
				rightImage = rightImage.clone();
			}
		}

		if(stereoModel_.left().imageHeight() == 0 || stereoModel_.left().imageWidth() == 0)
		{
			stereoModel_.setImageSize(leftImage.size());
		}

		data = SensorData(leftImage, rightImage, stereoModel_, this->getNextSeqID(), UTimer::now());
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}

	return data;
}


} // namespace rtabmap
