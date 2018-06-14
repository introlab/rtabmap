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

#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/CameraRGB.h"
#include "rtabmap/core/Version.h"

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UMath.h>

#ifdef RTABMAP_DC1394
#include <dc1394/dc1394.h>
#endif

#ifdef RTABMAP_FLYCAPTURE2
#include <triclops.h>
#include <fc2triclops.h>
#endif

#ifdef RTABMAP_ZED
#include <sl/Camera.hpp>
#endif

namespace rtabmap
{

//
// CameraStereoDC1394
// Inspired from ROS camera1394stereo package
//

#ifdef RTABMAP_DC1394
class DC1394Device
{
public:
	DC1394Device() :
		camera_(0),
		context_(0)
	{

	}
	~DC1394Device()
	{
		if (camera_)
		{
			if (DC1394_SUCCESS != dc1394_video_set_transmission(camera_, DC1394_OFF) ||
				DC1394_SUCCESS != dc1394_capture_stop(camera_))
			{
				UWARN("unable to stop camera");
			}

			// Free resources
			dc1394_capture_stop(camera_);
			dc1394_camera_free(camera_);
			camera_ = NULL;
		}
		if(context_)
		{
			dc1394_free(context_);
			context_ = NULL;
		}
	}

	const std::string & guid() const {return guid_;}

	bool init()
	{
		if(camera_)
		{
			// Free resources
			dc1394_capture_stop(camera_);
			dc1394_camera_free(camera_);
			camera_ = NULL;
		}

		// look for a camera
		int err;
		if(context_ == NULL)
		{
			context_ = dc1394_new ();
			if (context_ == NULL)
			{
				UERROR(    "Could not initialize dc1394_context.\n"
				"Make sure /dev/raw1394 exists, you have access permission,\n"
				"and libraw1394 development package is installed.");
				return false;
			}
		}

		dc1394camera_list_t *list;
		err = dc1394_camera_enumerate(context_, &list);
		if (err != DC1394_SUCCESS)
		{
			UERROR("Could not get camera list");
			return false;
		}

		if (list->num == 0)
		{
			UERROR("No cameras found");
			dc1394_camera_free_list (list);
			return false;
		}
		uint64_t guid = list->ids[0].guid;
		dc1394_camera_free_list (list);

		// Create a camera
		camera_ = dc1394_camera_new (context_, guid);
		if (!camera_)
		{
			UERROR("Failed to initialize camera with GUID [%016lx]", guid);
			return false;
		}

		uint32_t value[3];
		value[0]= camera_->guid & 0xffffffff;
		value[1]= (camera_->guid >>32) & 0x000000ff;
		value[2]= (camera_->guid >>40) & 0xfffff;
		guid_ = uFormat("%06x%02x%08x", value[2], value[1], value[0]);

		UINFO("camera model: %s %s", camera_->vendor, camera_->model);

		// initialize camera
		// Enable IEEE1394b mode if the camera and bus support it
		bool bmode = camera_->bmode_capable;
		if (bmode
			&& (DC1394_SUCCESS !=
			dc1394_video_set_operation_mode(camera_,
									DC1394_OPERATION_MODE_1394B)))
		{
			bmode = false;
			UWARN("failed to set IEEE1394b mode");
		}

		// start with highest speed supported
		dc1394speed_t request = DC1394_ISO_SPEED_3200;
		int rate = 3200;
		if (!bmode)
		{
			// not IEEE1394b capable: so 400Mb/s is the limit
			request = DC1394_ISO_SPEED_400;
			rate = 400;
		}

		// round requested speed down to next-lower defined value
		while (rate > 400)
		{
			if (request <= DC1394_ISO_SPEED_MIN)
			{
				// get current ISO speed of the device
				dc1394speed_t curSpeed;
				if (DC1394_SUCCESS == dc1394_video_get_iso_speed(camera_, &curSpeed) && curSpeed <= DC1394_ISO_SPEED_MAX)
				{
					// Translate curSpeed back to an int for the parameter
					// update, works as long as any new higher speeds keep
					// doubling.
					request = curSpeed;
					rate = 100 << (curSpeed - DC1394_ISO_SPEED_MIN);
				}
				else
				{
					UWARN("Unable to get ISO speed; assuming 400Mb/s");
					rate = 400;
					request = DC1394_ISO_SPEED_400;
				}
				break;
			}
			// continue with next-lower possible value
			request = (dc1394speed_t) ((int) request - 1);
			rate = rate / 2;
		}

		// set the requested speed
		if (DC1394_SUCCESS != dc1394_video_set_iso_speed(camera_, request))
		{
			UERROR("Failed to set iso speed");
			return false;
		}

		// set video mode
		dc1394video_modes_t vmodes;
		err = dc1394_video_get_supported_modes(camera_, &vmodes);
		if (err != DC1394_SUCCESS)
		{
			UERROR("unable to get supported video modes");
			return (dc1394video_mode_t) 0;
		}

		// see if requested mode is available
		bool found = false;
		dc1394video_mode_t videoMode = DC1394_VIDEO_MODE_FORMAT7_3; // bumblebee
		for (uint32_t i = 0; i < vmodes.num; ++i)
		{
			if (vmodes.modes[i] == videoMode)
			{
				found = true;
			}
		}
		if(!found)
		{
			UERROR("unable to get video mode %d", videoMode);
			return false;
		}

		if (DC1394_SUCCESS != dc1394_video_set_mode(camera_, videoMode))
		{
			UERROR("Failed to set video mode %d", videoMode);
			return false;
		}

		// special handling for Format7 modes
		if (dc1394_is_video_mode_scalable(videoMode) == DC1394_TRUE)
		{
			if (DC1394_SUCCESS != dc1394_format7_set_color_coding(camera_, videoMode, DC1394_COLOR_CODING_RAW16))
			{
				UERROR("Could not set color coding");
				return false;
			}
			uint32_t packetSize;
			if (DC1394_SUCCESS != dc1394_format7_get_recommended_packet_size(camera_, videoMode, &packetSize))
			{
				UERROR("Could not get default packet size");
				return false;
			}

			if (DC1394_SUCCESS != dc1394_format7_set_packet_size(camera_, videoMode, packetSize))
			{
				UERROR("Could not set packet size");
				return false;
			}
		}
		else
		{
			UERROR("Video is not in mode scalable");
		}

		// start the device streaming data
		// Set camera to use DMA, improves performance.
		if (DC1394_SUCCESS != dc1394_capture_setup(camera_, 4, DC1394_CAPTURE_FLAGS_DEFAULT))
		{
			UERROR("Failed to open device!");
			return false;
		}

		// Start transmitting camera data
		if (DC1394_SUCCESS != dc1394_video_set_transmission(camera_, DC1394_ON))
		{
			UERROR("Failed to start device!");
			return false;
		}

		return true;
	}

	bool getImages(cv::Mat & left, cv::Mat & right)
	{
		if(camera_)
		{
			dc1394video_frame_t * frame = NULL;
			UDEBUG("[%016lx] waiting camera", camera_->guid);
			dc1394_capture_dequeue (camera_, DC1394_CAPTURE_POLICY_WAIT, &frame);
			if (!frame)
			{
				UERROR("Unable to capture frame");
				return false;
			}
			dc1394video_frame_t frame1 = *frame;
			// deinterlace frame into two imagesCount one on top the other
			size_t frame1_size = frame->total_bytes;
			frame1.image = (unsigned char *) malloc(frame1_size);
			frame1.allocated_image_bytes = frame1_size;
			frame1.color_coding = DC1394_COLOR_CODING_RAW8;
			int err = dc1394_deinterlace_stereo_frames(frame, &frame1, DC1394_STEREO_METHOD_INTERLACED);
			if (err != DC1394_SUCCESS)
			{
				free(frame1.image);
				dc1394_capture_enqueue(camera_, frame);
				UERROR("Could not extract stereo frames");
				return false;
			}

			uint8_t* capture_buffer = reinterpret_cast<uint8_t *>(frame1.image);
			UASSERT(capture_buffer);

			cv::Mat image(frame->size[1], frame->size[0], CV_8UC3);
			cv::Mat image2 = image.clone();

			//DC1394_COLOR_CODING_RAW16:
			//DC1394_COLOR_FILTER_BGGR
			cv::cvtColor(cv::Mat(frame->size[1], frame->size[0], CV_8UC1, capture_buffer), left, CV_BayerRG2BGR);
			cv::cvtColor(cv::Mat(frame->size[1], frame->size[0], CV_8UC1, capture_buffer+image.total()), right, CV_BayerRG2GRAY);

			dc1394_capture_enqueue(camera_, frame);

			free(frame1.image);

			return true;
		}
		return false;
	}

private:
	dc1394camera_t *camera_;
	dc1394_t *context_;
	std::string guid_;
};
#endif

bool CameraStereoDC1394::available()
{
#ifdef RTABMAP_DC1394
	return true;
#else
	return false;
#endif
}

CameraStereoDC1394::CameraStereoDC1394(float imageRate, const Transform & localTransform) :
		Camera(imageRate, localTransform)
#ifdef RTABMAP_DC1394
        ,
		device_(0)
#endif
{
#ifdef RTABMAP_DC1394
	device_ = new DC1394Device();
#endif
}

CameraStereoDC1394::~CameraStereoDC1394()
{
#ifdef RTABMAP_DC1394
	if(device_)
	{
		delete device_;
	}
#endif
}

bool CameraStereoDC1394::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_DC1394
	if(device_)
	{
		bool ok = device_->init();
		if(ok)
		{
			// look for calibration files
			if(!calibrationFolder.empty())
			{
				if(!stereoModel_.load(calibrationFolder, cameraName.empty()?device_->guid():cameraName, false))
				{
					UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
							cameraName.empty()?device_->guid().c_str():cameraName.c_str(), calibrationFolder.c_str());
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
		}
		return ok;
	}
#else
	UERROR("CameraDC1394: RTAB-Map is not built with dc1394 support!");
#endif
	return false;
}

bool CameraStereoDC1394::isCalibrated() const
{
#ifdef RTABMAP_DC1394
	return stereoModel_.isValidForProjection();
#else
	return false;
#endif
}

std::string CameraStereoDC1394::getSerial() const
{
#ifdef RTABMAP_DC1394
	if(device_)
	{
		return device_->guid();
	}
#endif
	return "";
}

SensorData CameraStereoDC1394::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_DC1394
	if(device_)
	{
		cv::Mat left, right;
		device_->getImages(left, right);

		if(!left.empty() && !right.empty())
		{
			// Rectification
			if(stereoModel_.left().isValidForRectification())
			{
				left = stereoModel_.left().rectifyImage(left);
			}
			if(stereoModel_.right().isValidForRectification())
			{
				right = stereoModel_.right().rectifyImage(right);
			}
			StereoCameraModel model;
			if(stereoModel_.isValidForProjection())
			{
				model = StereoCameraModel(
						stereoModel_.left().fx(), //fx
						stereoModel_.left().fy(), //fy
						stereoModel_.left().cx(), //cx
						stereoModel_.left().cy(), //cy
						stereoModel_.baseline(),
						this->getLocalTransform(),
						left.size());
			}
			data = SensorData(left, right, model, this->getNextSeqID(), UTimer::now());
		}
	}
#else
	UERROR("CameraDC1394: RTAB-Map is not built with dc1394 support!");
#endif
	return data;
}

//
// CameraTriclops
//
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

//
// CameraStereoZED
//
bool CameraStereoZed::available()
{
#ifdef RTABMAP_ZED
	return true;
#else
	return false;
#endif
}

CameraStereoZed::CameraStereoZed(
		int deviceId,
		int resolution,
		int quality,
		int sensingMode,
		int confidenceThr,
		bool computeOdometry,
		float imageRate,
		const Transform & localTransform,
		bool selfCalibration) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_ZED
    ,
	zed_(0),
	src_(CameraVideo::kUsbDevice),
	usbDevice_(deviceId),
	svoFilePath_(""),
	resolution_(resolution),
	quality_(quality),
	selfCalibration_(selfCalibration),
	sensingMode_(sensingMode),
	confidenceThr_(confidenceThr),
	computeOdometry_(computeOdometry),
	lost_(true)
#endif
{
	UDEBUG("");
#ifdef RTABMAP_ZED
	UASSERT(resolution_ >= sl::RESOLUTION_HD2K && resolution_ <sl::RESOLUTION_LAST);
	UASSERT(quality_ >= sl::DEPTH_MODE_NONE && quality_ <sl::DEPTH_MODE_LAST);
	UASSERT(sensingMode_ >= sl::SENSING_MODE_STANDARD && sensingMode_ <sl::SENSING_MODE_LAST);
	UASSERT(confidenceThr_ >= 0 && confidenceThr_ <=100);
#endif
}

CameraStereoZed::CameraStereoZed(
		const std::string & filePath,
		int quality,
		int sensingMode,
		int confidenceThr,
		bool computeOdometry,
		float imageRate,
		const Transform & localTransform,
		bool selfCalibration) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_ZED
    ,
	zed_(0),
	src_(CameraVideo::kVideoFile),
	usbDevice_(0),
	svoFilePath_(filePath),
	resolution_(2),
	quality_(quality),
	selfCalibration_(selfCalibration),
	sensingMode_(sensingMode),
	confidenceThr_(confidenceThr),
	computeOdometry_(computeOdometry),
	lost_(true)
#endif
{
	UDEBUG("");
#ifdef RTABMAP_ZED
	UASSERT(resolution_ >= sl::RESOLUTION_HD2K && resolution_ <sl::RESOLUTION_LAST);
	UASSERT(quality_ >= sl::DEPTH_MODE_NONE && quality_ <sl::DEPTH_MODE_LAST);
	UASSERT(sensingMode_ >= sl::SENSING_MODE_STANDARD && sensingMode_ <sl::SENSING_MODE_LAST);
	UASSERT(confidenceThr_ >= 0 && confidenceThr_ <=100);
#endif
}

CameraStereoZed::~CameraStereoZed()
{
#ifdef RTABMAP_ZED
	if(zed_)
	{
		delete zed_;
	}
#endif
}

bool CameraStereoZed::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_ZED
	if(zed_)
	{
		delete zed_;
		zed_ = 0;
	}
	
	lost_ = true;

	sl::InitParameters param;
	param.camera_resolution=static_cast<sl::RESOLUTION>(resolution_);
	param.camera_fps=getImageRate();
	param.camera_linux_id=usbDevice_;
	param.depth_mode=(sl::DEPTH_MODE)quality_;
	param.coordinate_units=sl::UNIT_METER;
	param.coordinate_system=(sl::COORDINATE_SYSTEM)sl::COORDINATE_SYSTEM_IMAGE ;
	param.sdk_verbose=true;
	param.sdk_gpu_id=-1;
	param.depth_minimum_distance=-1;
	param.camera_disable_self_calib=!selfCalibration_;

	sl::ERROR_CODE r = sl::ERROR_CODE::SUCCESS;
	if(src_ == CameraVideo::kVideoFile)
	{
		UINFO("svo file = %s", svoFilePath_.c_str());
		zed_ = new sl::Camera(); // Use in SVO playback mode
		param.svo_input_filename=svoFilePath_.c_str();
		r = zed_->open(param);
	}
	else
	{
		UINFO("Resolution=%d imagerate=%f device=%d", resolution_, getImageRate(), usbDevice_);
		zed_ = new sl::Camera(); // Use in Live Mode
		r = zed_->open(param);
	}

	if(r!=sl::ERROR_CODE::SUCCESS)
	{
		UERROR("Camera initialization failed: \"%s\"", toString(r).c_str());
		delete zed_;
		zed_ = 0;
		return false;
	}


	UINFO("Init ZED: Mode=%d Unit=%d CoordinateSystem=%d Verbose=false device=-1 minDist=-1 self-calibration=%s vflip=false",
	      quality_, sl::UNIT_METER, sl::COORDINATE_SYSTEM_IMAGE , selfCalibration_?"true":"false");
	UDEBUG("");

	if(quality_!=sl::DEPTH_MODE_NONE)
	{
		zed_->setConfidenceThreshold(confidenceThr_);
	}

	if (computeOdometry_)
	{
		sl::TrackingParameters tparam;
		tparam.enable_spatial_memory=false;
		zed_->enableTracking(tparam);
		if(r!=sl::ERROR_CODE::SUCCESS)
		{
			UERROR("Camera tracking initialization failed: \"%s\"", toString(r).c_str());
		}
	}

	sl::CameraInformation infos = zed_->getCameraInformation();
	sl::CalibrationParameters *stereoParams = &(infos.calibration_parameters );
	sl::Resolution res = stereoParams->left_cam.image_size;

	stereoModel_ = StereoCameraModel(
		stereoParams->left_cam.fx, 
		stereoParams->left_cam.fy, 
		stereoParams->left_cam.cx, 
		stereoParams->left_cam.cy, 
		stereoParams->T[0],//baseline
		this->getLocalTransform(),
		cv::Size(res.width, res.height));

	UINFO("Calibration: fx=%f, fy=%f, cx=%f, cy=%f, baseline=%f, width=%d, height=%d, transform=%s",
			stereoParams->left_cam.fx,
			stereoParams->left_cam.fy,
			stereoParams->left_cam.cx,
			stereoParams->left_cam.cy,
			stereoParams->T[0],//baseline
			(int)res.width,
			(int)res.height,
			this->getLocalTransform().prettyPrint().c_str());

	return true;
#else
	UERROR("CameraStereoZED: RTAB-Map is not built with ZED sdk support!");
#endif
	return false;
}

bool CameraStereoZed::isCalibrated() const
{
#ifdef RTABMAP_ZED
	return stereoModel_.isValidForProjection();
#else
	return false;
#endif
}

std::string CameraStereoZed::getSerial() const
{
#ifdef RTABMAP_ZED
	if(zed_)
	{
		return uFormat("%x", zed_->getCameraInformation ().serial_number);
	}
#endif
	return "";
}

bool CameraStereoZed::odomProvided() const
{
#ifdef RTABMAP_ZED
	return computeOdometry_;
#else
	return false;
#endif
}
#ifdef RTABMAP_ZED
static cv::Mat slMat2cvMat(sl::Mat& input) {
	//convert MAT_TYPE to CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}	
	// cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	//cv::Mat and sl::Mat will share the same memory pointer
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

Transform zedPoseToTransform(const sl::Pose & pose)
{
	return Transform(
			pose.pose_data.m[0], pose.pose_data.m[1], pose.pose_data.m[2], pose.pose_data.m[3],
			pose.pose_data.m[4], pose.pose_data.m[5], pose.pose_data.m[6], pose.pose_data.m[7],
			pose.pose_data.m[8], pose.pose_data.m[9], pose.pose_data.m[10], pose.pose_data.m[11]);
}
#endif

SensorData CameraStereoZed::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_ZED
	sl::RuntimeParameters rparam((sl::SENSING_MODE)sensingMode_, quality_ > 0, quality_ > 0, sl::REFERENCE_FRAME_CAMERA);
	if(zed_)
	{
		UTimer timer;
		bool res = zed_->grab(rparam);
		while (src_ == CameraVideo::kUsbDevice && res!=sl::SUCCESS && timer.elapsed() < 2.0)
		{
			// maybe there is a latency with the USB, try again in 10 ms (for the next 2 seconds)
			uSleep(10);
			res = zed_->grab(rparam);
		}
		if(res==sl::SUCCESS)
		{
			// get left image
			sl::Mat tmp;
			zed_->retrieveImage(tmp,sl::VIEW_LEFT);
			cv::Mat rgbaLeft = slMat2cvMat(tmp);

			cv::Mat left;
			cv::cvtColor(rgbaLeft, left, cv::COLOR_BGRA2BGR);

			if(quality_ > 0)
			{
				// get depth image
				cv::Mat depth;
				sl::Mat tmp;
				zed_->retrieveMeasure(tmp,sl::MEASURE_DEPTH);
				slMat2cvMat(tmp).copyTo(depth);

				data = SensorData(left, depth, stereoModel_.left(), this->getNextSeqID(), UTimer::now());
			}
			else
			{
				// get right image
				sl::Mat tmp;zed_->retrieveImage(tmp,sl::VIEW_RIGHT );
				cv::Mat rgbaRight = slMat2cvMat(tmp);
				cv::Mat right;
				cv::cvtColor(rgbaRight, right, cv::COLOR_BGRA2GRAY);
			
				data = SensorData(left, right, stereoModel_, this->getNextSeqID(), UTimer::now());
			}

			if (computeOdometry_ && info)
			{
				sl::Pose pose;
				sl::TRACKING_STATE tracking_state = zed_->getPosition(pose);
				if (tracking_state == sl::TRACKING_STATE_OK)
				{
					int trackingConfidence = pose.pose_confidence;
					// FIXME What does pose_confidence == -1 mean?
					if (trackingConfidence>0)
					{
						info->odomPose = zedPoseToTransform(pose);
						if (!info->odomPose.isNull())
						{
							//transform x->forward, y->left, z->up
							Transform opticalTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
							info->odomPose = opticalTransform * info->odomPose * opticalTransform.inverse();

							if (lost_)
							{
								info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // don't know transform with previous pose
								lost_ = false;
								UDEBUG("Init %s (var=%f)", info->odomPose.prettyPrint().c_str(), 9999.0f);
							}
							else
							{
								info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 1.0f / float(trackingConfidence);
								UDEBUG("Run %s (var=%f)", info->odomPose.prettyPrint().c_str(), 1.0f / float(trackingConfidence));
							}
						}
						else
						{
							info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // lost
							lost_ = true;
							UWARN("ZED lost! (trackingConfidence=%d)", trackingConfidence);
						}
					}
					else
					{
						info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // lost
						lost_ = true;
						UWARN("ZED lost! (trackingConfidence=%d)", trackingConfidence);
					}
				}
				else
				{
					UWARN("Tracking not ok: state=\"%s\"", toString(tracking_state).c_str());
				}
			}
		}
		else if(src_ == CameraVideo::kUsbDevice)
		{
			UERROR("CameraStereoZed: Failed to grab images after 2 seconds!");
		}
		else
		{
			UWARN("CameraStereoZed: end of stream is reached!");
		}
	}
#else
	UERROR("CameraStereoZED: RTAB-Map is not built with ZED sdk support!");
#endif
	return data;
}


//
// CameraStereoImages
//
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
	if(camera2_)
	{
		delete camera2_;
	}
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

//
// CameraStereoVideo
//
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
