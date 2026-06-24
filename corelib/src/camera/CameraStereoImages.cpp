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
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
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
		camera2_(new CameraImages(pathRightImages)),
		rightGrayScale_(true)
{
	this->setImagesRectified(rectifyImages);
}

CameraStereoImages::CameraStereoImages(
		const std::string & pathLeftRightImages,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
		CameraImages("", imageRate, localTransform),
		camera2_(0),
		rightGrayScale_(true)
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
	UINFO("Calibration folder: \"%s\", name=\"%s\"", calibrationFolder.c_str(), cameraName.c_str());

	multiStereoModels_.clear();

	// look for calibration files
	if(!_multiCameraCalib && !calibrationFolder.empty() && !cameraName.empty())
	{
		if(!stereoModel_.load(calibrationFolder, cameraName, false, this->isImagesRectified()) && !stereoModel_.isValidForProjection())
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

	if(!_multiCameraCalib)
	{
		stereoModel_.setLocalTransform(this->getLocalTransform());
		stereoModel_.setName(cameraName);
		if(this->isImagesRectified() && !stereoModel_.isValidForRectification())
		{
			UWARN("Parameter \"rectifyImages\" is set, but no stereo model is loaded or valid for rectification. This can be ignored if input images are already rectified.");
		}
	}

	//desactivate before init as we will do it in this class instead for convenience
	bool rectify = this->isImagesRectified();
	this->setImagesRectified(false);
	// The base reader is used here only to enumerate/read the left images; in
	// multi-camera mode this class loads the calibration and splits the images
	// itself, so prevent the base from doing its own multi-camera handling and
	// per-frame config loading (which would look for config files inside the image
	// directory). Both flags are restored below: _multiCameraCalib still drives the
	// per-frame vs. shared calibration loading in this class.
	bool configForEachFrame = this->isConfigForEachFrame();
	bool multiCameraCalib = _multiCameraCalib;
	if(multiCameraCalib)
	{
		this->setConfigForEachFrame(false);
	}
	_multiCameraCalib = false;

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

	// restore the flags: _multiCameraCalib drives the per-frame vs. shared calibration loading below
	this->setConfigForEachFrame(configForEachFrame);
	_multiCameraCalib = multiCameraCalib;

	if(success && _multiCameraCalib)
	{
		// Multi-camera stereo mode: each left/right image is a horizontal stack of N
		// sub-camera images. Load one StereoCameraModel per sub-camera from calibration
		// files named "<imageBaseName>_<index>_left.yaml" and "<imageBaseName>_<index>_right.yaml"
		// (index starting at 0).
		if(calibrationFolder.empty())
		{
			UERROR("Multi-camera calibration is enabled but no calibration folder was provided.");
			success = false;
		}
		else
		{
			const std::vector<std::string> imageFiles = this->filenames();
			std::string firstBase = imageFiles.front().substr(0, imageFiles.front().find_last_of('.'));

			// In config-for-each-frame mode the prefix is each image's base name (one
			// calibration set per frame). Otherwise a single calibration set is shared by all
			// frames, using cameraName as the prefix when provided. cameraName may point to a
			// specific sub-camera calibration "<rig>_<index>" (e.g. when a single calibration
			// file is selected); in that case strip the trailing "_<index>" to recover the rig
			// prefix shared by all sub-cameras. Fall back to the first image's base name if empty.
			std::string leftSuffix = "_0_" + stereoModel_.getLeftSuffix() + ".yaml";
			std::string sharedBase = cameraName;
			if(!sharedBase.empty() && !UFile::exists(calibrationFolder + "/" + sharedBase + leftSuffix))
			{
				std::size_t us = sharedBase.find_last_of('_');
				if(us != std::string::npos && us+1 < sharedBase.size() &&
					sharedBase.find_first_not_of("0123456789", us+1) == std::string::npos &&
					UFile::exists(calibrationFolder + "/" + sharedBase.substr(0, us) + leftSuffix))
				{
					sharedBase = sharedBase.substr(0, us);
				}
			}
			if(sharedBase.empty())
			{
				sharedBase = firstBase;
			}

			// auto-detect the number of cameras
			int numCameras = 0;
			std::string detectBase = this->isConfigForEachFrame() ? firstBase : sharedBase;
			while(UFile::exists(calibrationFolder + "/" + detectBase + "_" + uNumber2Str(numCameras) + "_" + stereoModel_.getLeftSuffix() + ".yaml"))
			{
				++numCameras;
			}

			if(numCameras == 0)
			{
				UERROR("Multi-camera calibration is enabled but no calibration file matching "
						"\"%s/%s_<index>_%s.yaml\" was found.",
						calibrationFolder.c_str(), detectBase.c_str(), stereoModel_.getLeftSuffix().c_str());
				success = false;
			}
			else
			{
				UINFO("Multi-camera stereo mode: %d sub-cameras detected from \"%s\" (%s).", numCameras, calibrationFolder.c_str(),
						this->isConfigForEachFrame()?"one calibration per frame, loaded on demand":
								uFormat("calibration \"%s_<index>\" reused for all frames", sharedBase.c_str()).c_str());
				_calibrationFolder = calibrationFolder;
				_multiCameraCount = numCameras;
				if(this->isConfigForEachFrame())
				{
					// Validate the first frame now; the per-frame sub-camera stereo models are
					// loaded on demand in captureImage() (keyed by each image's base name) so we
					// don't keep every frame's models - and their rectification maps - in memory.
					if(loadStereoCameraModels(firstBase, rectify).empty())
					{
						success = false;
					}
				}
				else
				{
					// A single calibration set is shared by all frames: load it once.
					std::vector<StereoCameraModel> models = loadStereoCameraModels(sharedBase, rectify);
					if(models.empty())
					{
						success = false;
					}
					else
					{
						multiStereoModels_ = models;
					}
				}
			}
		}
	}

	this->setImagesRectified(rectify); // reset the flag
	return success;
}

bool CameraStereoImages::isCalibrated() const
{
	return stereoModel_.isValidForProjection() ||
			(!multiStereoModels_.empty() && multiStereoModels_.front().isValidForProjection()) ||
			(_multiCameraCalib && this->isConfigForEachFrame() && _multiCameraCount > 0); // per-frame models loaded on demand
}

std::vector<StereoCameraModel> CameraStereoImages::loadStereoCameraModels(const std::string & baseName, bool rectify) const
{
	std::vector<StereoCameraModel> models(_multiCameraCount);
	for(int i=0; i<_multiCameraCount; ++i)
	{
		std::string name = baseName + "_" + uNumber2Str(i);
		if(!models[i].load(_calibrationFolder, name, true /*ignoreStereoTransform*/, rectify) || !models[i].isValidForProjection())
		{
			UERROR("Failed to load a valid stereo calibration \"%s/%s_{%s,%s}.yaml\" for multi-camera frame base \"%s\".",
					_calibrationFolder.c_str(), name.c_str(),
					models[i].getLeftSuffix().c_str(), models[i].getRightSuffix().c_str(), baseName.c_str());
			return std::vector<StereoCameraModel>();
		}
		if(models[i].localTransform().isNull())
		{
			// In multi-camera mode the rig extrinsics are required: each
			// sub-camera calibration must provide a "local_transform".
			UERROR("Stereo calibration \"%s/%s_%s.yaml\" has no \"local_transform\"; it is "
					"required in multi-camera mode (rig extrinsics).",
					_calibrationFolder.c_str(), name.c_str(), models[i].getLeftSuffix().c_str());
			return std::vector<StereoCameraModel>();
		}
		if(rectify && !models[i].isValidForRectification())
		{
			UERROR("Parameter \"rectifyImages\" is set, but stereo calibration \"%s/%s_{%s,%s}.yaml\" is not valid for rectification.",
					_calibrationFolder.c_str(), name.c_str(),
					models[i].getLeftSuffix().c_str(), models[i].getRightSuffix().c_str());
			return std::vector<StereoCameraModel>();
		}
	}
	return models;
}

std::string CameraStereoImages::getSerial() const
{
	return stereoModel_.name();
}

SensorData CameraStereoImages::captureImage(SensorCaptureInfo * info)
{
	SensorData data;

	SensorData left, right;
	// Disable the base reader's own multi-camera handling while reading the stacked
	// left/right images: this class splits and rectifies them itself below. Restored
	// before the multi-camera branch, which relies on the real flag value.
	bool multiCameraCalib = _multiCameraCalib;
	_multiCameraCalib = false;
	left = CameraImages::captureImage(info);
	if(!left.imageRaw().empty())
	{
		if(camera2_)
		{
			camera2_->setBayerMode(this->getBayerMode());
			right = camera2_->takeImage(info);
		}
		else
		{
			right = this->takeImage(info);
		}
	}
	_multiCameraCalib = multiCameraCalib;

	if(!left.imageRaw().empty() && !right.imageRaw().empty())
	{
		// Rectification
		cv::Mat leftImage = left.imageRaw();
		cv::Mat rightImage = right.imageRaw();
		if(rightImage.type() != CV_8UC1 && rightGrayScale_)
		{
			cv::Mat tmp;
			cv::cvtColor(rightImage, tmp, CV_BGR2GRAY);
			rightImage = tmp;
		}

		if(multiCameraCalib)
		{
			// Multi-camera stereo mode: left and right images are horizontal stacks
			// of N sub-images (one stereo pair per sub-camera). Each pair is rectified
			// independently with its own model and written back into a stacked image of
			// the same layout. Sub-images are split using a uniform width (cols / N),
			// matching the convention used downstream to de-stack the images.
			std::vector<StereoCameraModel> models;
			if(this->isConfigForEachFrame())
			{
				// Per-frame calibration loaded on demand (not kept in memory),
				// keyed by the current image's base name.
				std::string base = this->lastImageFileName();
				base = base.substr(0, base.find_last_of('.'));
				models = loadStereoCameraModels(base, this->isImagesRectified());
			}
			else
			{
				// a single calibration set is shared by all frames
				UASSERT(!multiStereoModels_.empty());
				models = multiStereoModels_;
			}
			if(models.empty())
			{
				return data;
			}
			int n = (int)models.size();

			if(leftImage.cols % n != 0 || rightImage.cols % n != 0)
			{
				UERROR("Multi-camera stereo: stacked image width (left=%d, right=%d) is not "
						"divisible by the number of cameras (%d).", leftImage.cols, rightImage.cols, n);
				return data;
			}
			int subWidthLeft = leftImage.cols/n;
			int subWidthRight = rightImage.cols/n;

			if(this->isImagesRectified())
			{
				cv::Mat leftRect(leftImage.rows, leftImage.cols, leftImage.type());
				cv::Mat rightRect(rightImage.rows, rightImage.cols, rightImage.type());
				for(int i=0; i<n; ++i)
				{
					cv::Rect roiL(subWidthLeft*i, 0, subWidthLeft, leftImage.rows);
					cv::Rect roiR(subWidthRight*i, 0, subWidthRight, rightImage.rows);
					models[i].left().rectifyImage(leftImage(roiL)).copyTo(leftRect(roiL));
					models[i].right().rectifyImage(rightImage(roiR)).copyTo(rightRect(roiR));
				}
				leftImage = leftRect;
				rightImage = rightRect;
			}

			for(int i=0; i<n; ++i)
			{
				if(models[i].left().imageHeight() == 0 || models[i].left().imageWidth() == 0)
				{
					models[i].setImageSize(cv::Size(subWidthLeft, leftImage.rows));
				}
			}

			data = SensorData(left.laserScanRaw(), leftImage, rightImage, models, left.id()/(camera2_?1:2), left.stamp());
			data.setGroundTruth(left.groundTruth());
		}
		else
		{
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
