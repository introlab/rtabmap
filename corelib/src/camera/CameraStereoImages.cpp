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

#include <rtabmap/core/camera/CameraStereoImages.h>
#include <rtabmap/utilite/UStl.h>
#include <opencv2/imgproc/types_c.h>

namespace rtabmap
{

bool CameraStereoImages::available()
{
	return true;
}

CameraStereoImages::CameraStereoImages(
		const std::string & pathLeftImages,
		const std::string & pathRightImages,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
		CameraImages(pathLeftImages, imageRate, localTransform),
		camera2_(new CameraImages(pathRightImages))
{
	this->setImagesRectified(rectifyImages);
}

CameraStereoImages::CameraStereoImages(
		const std::string & pathLeftRightImages,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
		CameraImages("", imageRate, localTransform),
		camera2_(0)
{
	std::vector<std::string> paths = uListToVector(uSplit(pathLeftRightImages, uStrContains(pathLeftRightImages, ":")?':':';'));
	if(paths.size() >= 1)
	{
		this->setPath(paths[0]);
		this->setImagesRectified(rectifyImages);

		if(paths.size() >= 2)
		{
			camera2_ = new CameraImages(paths[1]);
		}
	}
	else
	{
		UERROR("The path is empty!");
	}
}

CameraStereoImages::~CameraStereoImages()
{
	UDEBUG("");
	delete camera2_;
	UDEBUG("");
}

bool CameraStereoImages::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	// look for calibration files
	if(!calibrationFolder.empty() && !cameraName.empty())
	{
		if(!stereoModel_.load(calibrationFolder, cameraName, false) && !stereoModel_.isValidForProjection())
		{
			UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
					cameraName.c_str(), calibrationFolder.c_str());
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
	stereoModel_.setName(cameraName);
	if(this->isImagesRectified() && !stereoModel_.isValidForRectification())
	{
		UERROR("Parameter \"rectifyImages\" is set, but no stereo model is loaded or valid.");
		return false;
	}

	//desactivate before init as we will do it in this class instead for convenience
	bool rectify = this->isImagesRectified();
	this->setImagesRectified(false);

	bool success = false;
	if(CameraImages::init())
	{
		if(camera2_)
		{
			camera2_->setBayerMode(this->getBayerMode());
			if(camera2_->init())
			{
				if(this->imagesCount() == camera2_->imagesCount())
				{
					success = true;
				}
				else
				{
					UERROR("Cameras don't have the same number of images (%d vs %d)",
							this->imagesCount(), camera2_->imagesCount());
				}
			}
			else
			{
				UERROR("Cannot initialize the second camera.");
			}
		}
		else
		{
			success = true;
		}
	}
	this->setImagesRectified(rectify); // reset the flag
	return success;
}

bool CameraStereoImages::isCalibrated() const
{
	return stereoModel_.isValidForProjection();
}

std::string CameraStereoImages::getSerial() const
{
	return stereoModel_.name();
}

SensorData CameraStereoImages::captureImage(CameraInfo * info)
{
	SensorData data;

	SensorData left, right;
	left = CameraImages::captureImage(info);
	if(!left.imageRaw().empty())
	{
		if(camera2_)
		{
			right = camera2_->takeImage(info);
		}
		else
		{
			right = this->takeImage(info);
		}

		if(!right.imageRaw().empty())
		{
			// Rectification
			cv::Mat leftImage = left.imageRaw();
			cv::Mat rightImage = right.imageRaw();
			if(rightImage.type() != CV_8UC1)
			{
				cv::Mat tmp;
				cv::cvtColor(rightImage, tmp, CV_BGR2GRAY);
				rightImage = tmp;
			}
			if(this->isImagesRectified() && stereoModel_.isValidForRectification())
			{
				leftImage = stereoModel_.left().rectifyImage(leftImage);
				rightImage = stereoModel_.right().rectifyImage(rightImage);
			}

			if(stereoModel_.left().imageHeight() == 0 || stereoModel_.left().imageWidth() == 0)
			{
				stereoModel_.setImageSize(leftImage.size());
			}

			data = SensorData(left.laserScanRaw(), leftImage, rightImage, stereoModel_, left.id()/(camera2_?1:2), left.stamp());
			data.setGroundTruth(left.groundTruth());
		}
	}
	return data;
}

} // namespace rtabmap
