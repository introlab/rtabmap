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

#include <rtabmap/core/camera/CameraStereoFlyCapture2.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>

#ifdef RTABMAP_FLYCAPTURE2
#include <triclops.h>
#include <fc2triclops.h>
#endif

namespace rtabmap
{

CameraStereoFlyCapture2::CameraStereoFlyCapture2(float imageRate, const Transform & localTransform) :
		Camera(imageRate, localTransform)
#ifdef RTABMAP_FLYCAPTURE2
        ,
		camera_(0),
		triclopsCtx_(0)
#endif
{
#ifdef RTABMAP_FLYCAPTURE2
	camera_ = new FlyCapture2::Camera();
#endif
}

CameraStereoFlyCapture2::~CameraStereoFlyCapture2()
{
#ifdef RTABMAP_FLYCAPTURE2
	// Close the camera
	camera_->StopCapture();
	camera_->Disconnect();

	// Destroy the Triclops context
	triclopsDestroyContext( triclopsCtx_ ) ;

	delete camera_;
#endif
}

bool CameraStereoFlyCapture2::available()
{
#ifdef RTABMAP_FLYCAPTURE2
	return true;
#else
	return false;
#endif
}

bool CameraStereoFlyCapture2::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_FLYCAPTURE2
	if(camera_)
	{
		// Close the camera
		camera_->StopCapture();
		camera_->Disconnect();
	}
	if(triclopsCtx_)
	{
		triclopsDestroyContext(triclopsCtx_);
		triclopsCtx_ = 0;
	}

	// connect camera
	FlyCapture2::Error fc2Error = camera_->Connect();
	if(fc2Error != FlyCapture2::PGRERROR_OK)
	{
		UERROR("Failed to connect the camera.");
		return false;
	}

	// configure camera
	Fc2Triclops::StereoCameraMode mode = Fc2Triclops::TWO_CAMERA_NARROW;
	if(Fc2Triclops::setStereoMode(*camera_, mode ))
	{
		UERROR("Failed to set stereo mode.");
		return false;
	}

	// generate the Triclops context
	FlyCapture2::CameraInfo camInfo;
	if(camera_->GetCameraInfo(&camInfo) != FlyCapture2::PGRERROR_OK)
	{
		UERROR("Failed to get camera info.");
		return false;
	}

	float dummy;
	unsigned packetSz;
	FlyCapture2::Format7ImageSettings imageSettings;
	int maxWidth = 640;
	int maxHeight = 480;
	if(camera_->GetFormat7Configuration(&imageSettings, &packetSz, &dummy) == FlyCapture2::PGRERROR_OK)
	{
		maxHeight = imageSettings.height;
		maxWidth = imageSettings.width;
	}

	// Get calibration from th camera
	if(Fc2Triclops::getContextFromCamera(camInfo.serialNumber, &triclopsCtx_))
	{
		UERROR("Failed to get calibration from the camera.");
		return false;
	}

	float fx, cx, cy, baseline;
	triclopsGetFocalLength(triclopsCtx_, &fx);
	triclopsGetImageCenter(triclopsCtx_, &cy, &cx);
	triclopsGetBaseline(triclopsCtx_, &baseline);
	UINFO("Stereo parameters: fx=%f cx=%f cy=%f baseline=%f", fx, cx, cy, baseline);

	triclopsSetCameraConfiguration(triclopsCtx_, TriCfg_2CAM_HORIZONTAL_NARROW );
	UASSERT(triclopsSetResolutionAndPrepare(triclopsCtx_, maxHeight, maxWidth, maxHeight, maxWidth) == Fc2Triclops::ERRORTYPE_OK);

	if(camera_->StartCapture() != FlyCapture2::PGRERROR_OK)
	{
		UERROR("Failed to start capture.");
		return false;
	}

	return true;
#else
	UERROR("CameraStereoFlyCapture2: RTAB-Map is not built with Triclops support!");
#endif
	return false;
}

bool CameraStereoFlyCapture2::isCalibrated() const
{
#ifdef RTABMAP_FLYCAPTURE2
	if(triclopsCtx_)
	{
		float fx, cx, cy, baseline;
		triclopsGetFocalLength(triclopsCtx_, &fx);
		triclopsGetImageCenter(triclopsCtx_, &cy, &cx);
		triclopsGetBaseline(triclopsCtx_, &baseline);
		return fx > 0.0f && cx > 0.0f && cy > 0.0f && baseline > 0.0f;
	}
#endif
	return false;
}

std::string CameraStereoFlyCapture2::getSerial() const
{
#ifdef RTABMAP_FLYCAPTURE2
	if(camera_ && camera_->IsConnected())
	{
		FlyCapture2::CameraInfo camInfo;
		if(camera_->GetCameraInfo(&camInfo) == FlyCapture2::PGRERROR_OK)
		{
			return uNumber2Str(camInfo.serialNumber);
		}
	}
#endif
	return "";
}

// struct containing image needed for processing
#ifdef RTABMAP_FLYCAPTURE2
struct ImageContainer
{
	FlyCapture2::Image tmp[2];
    FlyCapture2::Image unprocessed[2];
} ;
#endif

SensorData CameraStereoFlyCapture2::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_FLYCAPTURE2
	if(camera_ && triclopsCtx_ && camera_->IsConnected())
	{
		// grab image from camera.
		// this image contains both right and left imagesCount
		FlyCapture2::Image grabbedImage;
		if(camera_->RetrieveBuffer(&grabbedImage) == FlyCapture2::PGRERROR_OK)
		{
			// right and left image extracted from grabbed image
			ImageContainer imageCont;

			// generate triclops input from grabbed image
			FlyCapture2::Image imageRawRight;
			FlyCapture2::Image imageRawLeft;
			FlyCapture2::Image * unprocessedImage = imageCont.unprocessed;

			// Convert the pixel interleaved raw data to de-interleaved and color processed data
			if(Fc2Triclops::unpackUnprocessedRawOrMono16Image(
										   grabbedImage,
										   true /*assume little endian*/,
										   imageRawLeft /* right */,
										   imageRawRight /* left */) == Fc2Triclops::ERRORTYPE_OK)
			{
				// convert to color
				FlyCapture2::Image srcImgRightRef(imageRawRight);
				FlyCapture2::Image srcImgLeftRef(imageRawLeft);

				bool ok = true;;
				if ( srcImgRightRef.SetColorProcessing(FlyCapture2::HQ_LINEAR) != FlyCapture2::PGRERROR_OK ||
				     srcImgLeftRef.SetColorProcessing(FlyCapture2::HQ_LINEAR) != FlyCapture2::PGRERROR_OK)
				{
					ok = false;
				}

				if(ok)
				{
					FlyCapture2::Image imageColorRight;
					FlyCapture2::Image imageColorLeft;
					if ( srcImgRightRef.Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &imageColorRight) != FlyCapture2::PGRERROR_OK ||
						 srcImgLeftRef.Convert(FlyCapture2::PIXEL_FORMAT_BGRU, &imageColorLeft) != FlyCapture2::PGRERROR_OK)
					{
						ok = false;
					}

					if(ok)
					{
						//RECTIFY RIGHT
						TriclopsInput triclopsColorInputs;
						triclopsBuildRGBTriclopsInput(
							grabbedImage.GetCols(),
							grabbedImage.GetRows(),
							imageColorRight.GetStride(),
							(unsigned long)grabbedImage.GetTimeStamp().seconds,
							(unsigned long)grabbedImage.GetTimeStamp().microSeconds,
							imageColorRight.GetData(),
							imageColorRight.GetData(),
							imageColorRight.GetData(),
							&triclopsColorInputs);

						triclopsRectify(triclopsCtx_, const_cast<TriclopsInput *>(&triclopsColorInputs) );
						// Retrieve the rectified image from the triclops context
						TriclopsImage rectifiedImage;
						triclopsGetImage( triclopsCtx_,
							TriImg_RECTIFIED,
							TriCam_REFERENCE,
							&rectifiedImage );

						cv::Mat left,right;
						right = cv::Mat(rectifiedImage.nrows, rectifiedImage.ncols, CV_8UC1, rectifiedImage.data).clone();

						//RECTIFY LEFT COLOR
						triclopsBuildPackedTriclopsInput(
							grabbedImage.GetCols(),
							grabbedImage.GetRows(),
							imageColorLeft.GetStride(),
							(unsigned long)grabbedImage.GetTimeStamp().seconds,
							(unsigned long)grabbedImage.GetTimeStamp().microSeconds,
							imageColorLeft.GetData(),
							&triclopsColorInputs );

						cv::Mat pixelsLeftBuffer( grabbedImage.GetRows(), grabbedImage.GetCols(), CV_8UC4);
						TriclopsPackedColorImage colorImage;
						triclopsSetPackedColorImageBuffer(
							triclopsCtx_,
							TriCam_LEFT,
							(TriclopsPackedColorPixel*)pixelsLeftBuffer.data );

						triclopsRectifyPackedColorImage(
							triclopsCtx_,
							TriCam_LEFT,
							&triclopsColorInputs,
							&colorImage );

						cv::cvtColor(pixelsLeftBuffer, left, CV_RGBA2RGB);

						// Set calibration stuff
						float fx, cy, cx, baseline;
						triclopsGetFocalLength(triclopsCtx_, &fx);
						triclopsGetImageCenter(triclopsCtx_, &cy, &cx);
						triclopsGetBaseline(triclopsCtx_, &baseline);

						StereoCameraModel model(
								fx,
								fx,
								cx,
								cy,
								baseline,
								this->getLocalTransform(),
								left.size());
						data = SensorData(left, right, model, this->getNextSeqID(), UTimer::now());
					}
				}
			}
		}
	}

#else
	UERROR("CameraStereoFlyCapture2: RTAB-Map is not built with Triclops support!");
#endif
	return data;
}

} // namespace rtabmap
