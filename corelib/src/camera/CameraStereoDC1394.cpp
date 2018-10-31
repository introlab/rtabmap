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

#include <rtabmap/core/camera/CameraStereoDC1394.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_DC1394
#include <dc1394/dc1394.h>
#endif

namespace rtabmap
{

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
	delete device_;
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

} // namespace rtabmap
