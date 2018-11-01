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
#include <triclopsrectify.h>
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

#ifdef RTABMAP_FLYCAPTURE2

// struct containing image needed for processing
struct ImageContainer
{
	FlyCapture2::Image tmp[2];
	FlyCapture2::Image unprocessed[2];
};

// generate color stereo input
void generateColorStereoInput(TriclopsContext const &context,
	FlyCapture2::Image const &grabbedImage,
	ImageContainer &imageCont,
	TriclopsColorStereoPair &stereoPair)
{
	Fc2Triclops::ErrorType fc2TriclopsError;
	TriclopsError te;

	TriclopsColorImage triclopsImageContainer[2];
	FlyCapture2::Image *tmpImage = imageCont.tmp;
	FlyCapture2::Image *unprocessedImage = imageCont.unprocessed;

	// Convert the pixel interleaved raw data to de-interleaved and color processed data
	fc2TriclopsError = Fc2Triclops::unpackUnprocessedRawOrMono16Image(
		grabbedImage,
		true /*assume little endian*/,
		tmpImage[0],
		tmpImage[1]);

	UASSERT_MSG(fc2TriclopsError == Fc2Triclops::ERRORTYPE_OK, uFormat("Error: %d", (int)fc2TriclopsError).c_str());

	// preprocess color image
	for (int i = 0; i < 2; ++i) {
		FlyCapture2::Error fc2Error;
		fc2Error = tmpImage[i].SetColorProcessing(FlyCapture2::HQ_LINEAR);
		UASSERT_MSG(fc2Error == FlyCapture2::PGRERROR_OK, fc2Error.GetDescription());

		// convert preprocessed color image to BGRU format
		fc2Error = tmpImage[i].Convert(FlyCapture2::PIXEL_FORMAT_BGRU,
			&unprocessedImage[i]);
		UASSERT_MSG(fc2Error == FlyCapture2::PGRERROR_OK, fc2Error.GetDescription());
	}

	// create triclops image for right and left lens
	for (size_t i = 0; i < 2; ++i) {
		TriclopsColorImage *image = &triclopsImageContainer[i];
		te = triclopsLoadColorImageFromBuffer(
			reinterpret_cast<TriclopsColorPixel *>(unprocessedImage[i].GetData()),
			unprocessedImage[i].GetRows(),
			unprocessedImage[i].GetCols(),
			unprocessedImage[i].GetStride(),
			image);
		UASSERT_MSG(te == Fc2Triclops::ERRORTYPE_OK, uFormat("Error: %d", (int)te).c_str());
	}

	// create stereo input from the triclops images constructed above
	// pack image data into a TriclopsColorStereoPair structure
	te = triclopsBuildColorStereoPairFromBuffers(
		context,
		&triclopsImageContainer[1],
		&triclopsImageContainer[0],
		&stereoPair);
	UASSERT_MSG(te == Fc2Triclops::ERRORTYPE_OK, uFormat("Error: %d", (int)te).c_str());
}
#endif

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

	triclopsSetResolution(triclopsCtx_, maxHeight, maxWidth);
	if (triclopsPrepareRectificationData(triclopsCtx_,
		maxHeight,
		maxWidth,
		maxHeight,
		maxWidth))
	{
		UERROR("Failed to prepare rectification matrices.");
		return false;
	}
	triclopsSetCameraConfiguration(triclopsCtx_, TriCfg_2CAM_HORIZONTAL_NARROW);

	float fx, cx, cy, baseline;
	triclopsGetFocalLength(triclopsCtx_, &fx);
	triclopsGetImageCenter(triclopsCtx_, &cy, &cx);
	cx *= maxWidth;
	cy *= maxHeight;
	triclopsGetBaseline(triclopsCtx_, &baseline);
	UINFO("Stereo parameters: fx=%f cx=%f cy=%f baseline=%f %dx%d", fx, cx, cy, baseline, maxWidth, maxHeight);
		
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

			TriclopsColorStereoPair colorStereoInput;
			generateColorStereoInput(triclopsCtx_, grabbedImage, imageCont, colorStereoInput);

			// rectify images
			TriclopsError triclops_status = triclopsColorRectify(triclopsCtx_, &colorStereoInput);
			UASSERT_MSG(triclops_status == Fc2Triclops::ERRORTYPE_OK, uFormat("Error: %d", (int)triclops_status).c_str());

			// get images
			cv::Mat left,right;
			TriclopsColorImage color_image;
			triclops_status = triclopsGetColorImage(triclopsCtx_, TriImg_RECTIFIED_COLOR, TriCam_LEFT, &color_image);
			UASSERT_MSG(triclops_status == Fc2Triclops::ERRORTYPE_OK, uFormat("Error: %d", (int)triclops_status).c_str());
			cv::cvtColor(cv::Mat(color_image.nrows, color_image.ncols, CV_8UC4, color_image.data), left, CV_RGBA2RGB);
			triclops_status = triclopsGetColorImage(triclopsCtx_, TriImg_RECTIFIED_COLOR, TriCam_RIGHT, &color_image);
			UASSERT_MSG(triclops_status == Fc2Triclops::ERRORTYPE_OK, uFormat("Error: %d", (int)triclops_status).c_str());
			cv::cvtColor(cv::Mat(color_image.nrows, color_image.ncols, CV_8UC4, color_image.data), right, CV_RGBA2GRAY);

			// Set calibration stuff
			float fx, cy, cx, baseline;
			triclopsGetFocalLength(triclopsCtx_, &fx);
			triclopsGetImageCenter(triclopsCtx_, &cy, &cx);
			triclopsGetBaseline(triclopsCtx_, &baseline);
			cx *= left.cols;
			cy *= left.rows;

			StereoCameraModel model(
				fx,
				fx,
				cx,
				cy,
				baseline,
				this->getLocalTransform(),
				left.size());
			data = SensorData(left, right, model, this->getNextSeqID(), UTimer::now());

			// Compute disparity
			/*triclops_status = triclopsStereo(triclopsCtx_);
			UASSERT_MSG(triclops_status == Fc2Triclops::ERRORTYPE_OK, uFormat("Error: %d", (int)triclops_status).c_str());

			TriclopsImage16 disparity_image;
			triclops_status = triclopsGetImage16(triclopsCtx_, TriImg16_DISPARITY, TriCam_REFERENCE, &disparity_image);
			UASSERT_MSG(triclops_status == Fc2Triclops::ERRORTYPE_OK, uFormat("Error: %d", (int)triclops_status).c_str());
			cv::Mat depth(disparity_image.nrows, disparity_image.ncols, CV_32FC1);
			int pixelinc = disparity_image.rowinc / 2;
			float x, y;
			for (int i = 0, k = 0; i < disparity_image.nrows; i++) {
				unsigned short *row = disparity_image.data + i * pixelinc;
				float *rowOut = (float *)depth.row(i).data;
				for (int j = 0; j < disparity_image.ncols; j++, k++) {
					unsigned short disparity = row[j];

					// do not save invalid points
					if (disparity < 0xFFF0) {
						// convert the 16 bit disparity value to floating point x,y,z
						triclopsRCD16ToXYZ(triclopsCtx_, i, j, disparity, &x, &y, &rowOut[j]);
					}
				}
			}
			CameraModel model(fx, fx, cx, cy, this->getLocalTransform(), 0, left.size());
			data = SensorData(left, depth, model, this->getNextSeqID(), UTimer::now());
			*/
		}
	}

#else
	UERROR("CameraStereoFlyCapture2: RTAB-Map is not built with Triclops support!");
#endif
	return data;
}

} // namespace rtabmap
