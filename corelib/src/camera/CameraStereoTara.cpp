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

/**
 * Contributed by e-consystemgit
 * https://www.e-consystems.com/opensource-linux-webcam-software-application.asp
 */

#include <rtabmap/core/camera/CameraStereoTara.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThreadC.h>
#include <rtabmap/utilite/UConversion.h>
#if CV_MAJOR_VERSION > 3
#include <opencv2/videoio/videoio_c.h>
#endif

namespace rtabmap
{


bool CameraStereoTara::available()
{
	return true;
}


CameraStereoTara::CameraStereoTara(
	int device,
	bool rectifyImages,
	float imageRate,
	const Transform & localTransform) :
	Camera(imageRate, localTransform),
	rectifyImages_(rectifyImages),
	usbDevice_(device)
{

	if ( CV_MAJOR_VERSION < 3 || ( CV_MAJOR_VERSION > 3 && CV_MINOR_VERSION < 1 ))
	{
		UERROR(" OpenCV %d.%d detected, Minimum OpenCV 3.2 Required ", CV_MAJOR_VERSION,CV_MINOR_VERSION);
		exit(0);
	}
}

CameraStereoTara::~CameraStereoTara()
{
	capture_.release();
}

bool CameraStereoTara::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	cameraName_ = cameraName;
	if(capture_.isOpened())
	{
		capture_.release();
	}


		capture_.open(usbDevice_);

		capture_.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '1', '6', ' '));
		capture_.set(CV_CAP_PROP_FPS, 60);
		capture_.set(CV_CAP_PROP_FRAME_WIDTH, 752);
		capture_.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
		capture_.set(CV_CAP_PROP_CONVERT_RGB,false);

		ULOGGER_DEBUG("CameraStereoTara: Usb device initialization on device %d", usbDevice_);


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

bool CameraStereoTara::isCalibrated() const
{
	return stereoModel_.isValidForProjection();
}

std::string CameraStereoTara::getSerial() const
{
	return cameraName_;
}

SensorData CameraStereoTara::captureImage(CameraInfo * info)
{
	SensorData data;

	if(capture_.isOpened() )
	{
		cv::Mat img, leftImage, interLeavedFrame, rightImage;
        std::vector<cv::Mat> StereoFrames;
		interLeavedFrame.create(480, 752, CV_8UC2);

		if(!capture_.read(img))
		{
			return data;
		}

	  interLeavedFrame.data = img.data;
	  split(interLeavedFrame, StereoFrames);
    leftImage=StereoFrames[0].clone();
    rightImage=StereoFrames[1].clone();

		// Rectification
		if(rectifyImages_ && stereoModel_.left().isValidForRectification() && stereoModel_.right().isValidForRectification())
		{
			leftImage = stereoModel_.left().rectifyImage(leftImage);
			rightImage = stereoModel_.right().rectifyImage(rightImage);
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
