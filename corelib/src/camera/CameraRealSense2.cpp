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

#include <rtabmap/core/camera/CameraRealSense2.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThreadC.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_REALSENSE2
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#endif

namespace rtabmap
{

bool CameraRealSense2::available()
{
#ifdef RTABMAP_REALSENSE2
	return true;
#else
	return false;
#endif
}

CameraRealSense2::CameraRealSense2(
		const std::string & device,
		float imageRate,
		const rtabmap::Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_REALSENSE2
    ,
	ctx_(new rs2::context),
	dev_(new rs2::device),
	deviceId_(device),
	syncer_(new rs2::syncer),
	depth_scale_meters_(1.0f),
	depthIntrinsics_(new rs2_intrinsics),
	rgbIntrinsics_(new rs2_intrinsics),
	depthToRGBExtrinsics_(new rs2_extrinsics),
	emitterEnabled_(true),
	irDepth_(false)
#endif
{
	UDEBUG("");
}

CameraRealSense2::~CameraRealSense2()
{
#ifdef RTABMAP_REALSENSE2
  for(rs2::sensor _sensor : dev_->query_sensors())
  {
      _sensor.stop();
      _sensor.close();
  }
	delete ctx_;
	delete dev_;
	delete syncer_;
	delete depthIntrinsics_;
	delete rgbIntrinsics_;
	delete depthToRGBExtrinsics_;
#endif
	UDEBUG("");
}

#ifdef RTABMAP_REALSENSE2
void alignFrame(const rs2_intrinsics& from_intrin,
                                   const rs2_intrinsics& other_intrin,
                                   rs2::frame from_image,
                                   uint32_t output_image_bytes_per_pixel,
                                   const rs2_extrinsics& from_to_other,
								   cv::Mat & registeredDepth,
								   float depth_scale_meters)
{
    static const auto meter_to_mm = 0.001f;
    uint8_t* p_out_frame = registeredDepth.data;
    auto from_vid_frame = from_image.as<rs2::video_frame>();
    auto from_bytes_per_pixel = from_vid_frame.get_bytes_per_pixel();

    static const auto blank_color = 0x00;
    UASSERT(registeredDepth.total()*registeredDepth.channels()*registeredDepth.depth() == other_intrin.height * other_intrin.width * output_image_bytes_per_pixel);
    memset(p_out_frame, blank_color, other_intrin.height * other_intrin.width * output_image_bytes_per_pixel);

    auto p_from_frame = reinterpret_cast<const uint8_t*>(from_image.get_data());
    auto from_stream_type = from_image.get_profile().stream_type();
    float depth_units = ((from_stream_type == RS2_STREAM_DEPTH)? depth_scale_meters:1.f);
    UASSERT(from_stream_type == RS2_STREAM_DEPTH);
    UASSERT_MSG(depth_units > 0.0f, uFormat("depth_scale_meters=%f", depth_scale_meters).c_str());
#pragma omp parallel for schedule(dynamic)
    for (int from_y = 0; from_y < from_intrin.height; ++from_y)
    {
        int from_pixel_index = from_y * from_intrin.width;
        for (int from_x = 0; from_x < from_intrin.width; ++from_x, ++from_pixel_index)
        {
            // Skip over depth pixels with the value of zero
            float depth = (from_stream_type == RS2_STREAM_DEPTH)?(depth_units * ((const uint16_t*)p_from_frame)[from_pixel_index]): 1.f;
            if (depth)
            {
                // Map the top-left corner of the depth pixel onto the other image
                float from_pixel[2] = { from_x - 0.5f, from_y - 0.5f }, from_point[3], other_point[3], other_pixel[2];
                rs2_deproject_pixel_to_point(from_point, &from_intrin, from_pixel, depth);
                rs2_transform_point_to_point(other_point, &from_to_other, from_point);
                rs2_project_point_to_pixel(other_pixel, &other_intrin, other_point);
                const int other_x0 = static_cast<int>(other_pixel[0] + 0.5f);
                const int other_y0 = static_cast<int>(other_pixel[1] + 0.5f);

                // Map the bottom-right corner of the depth pixel onto the other image
                from_pixel[0] = from_x + 0.5f; from_pixel[1] = from_y + 0.5f;
                rs2_deproject_pixel_to_point(from_point, &from_intrin, from_pixel, depth);
                rs2_transform_point_to_point(other_point, &from_to_other, from_point);
                rs2_project_point_to_pixel(other_pixel, &other_intrin, other_point);
                const int other_x1 = static_cast<int>(other_pixel[0] + 0.5f);
                const int other_y1 = static_cast<int>(other_pixel[1] + 0.5f);

                if (other_x0 < 0 || other_y0 < 0 || other_x1 >= other_intrin.width || other_y1 >= other_intrin.height)
                    continue;

                for (int y = other_y0; y <= other_y1; ++y)
                {
                    for (int x = other_x0; x <= other_x1; ++x)
                    {
                        int out_pixel_index = y * other_intrin.width + x;
                        //Tranfer n-bit pixel to n-bit pixel
                        for (int i = 0; i < from_bytes_per_pixel; i++)
                        {
                            const auto out_offset = out_pixel_index * output_image_bytes_per_pixel + i;
                            const auto from_offset = from_pixel_index * output_image_bytes_per_pixel + i;
                            p_out_frame[out_offset] = p_from_frame[from_offset] * (depth_units / meter_to_mm);
                        }
                    }
                }
            }
        }
    }
}
#endif

bool CameraRealSense2::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_REALSENSE2

	UINFO("setupDevice...");

	auto list = ctx_->query_devices();
	if (0 == list.size())
	{
		UERROR("No RealSense2 devices were found!");
		return false;
	}

	bool found=false;
	for (auto&& dev : list)
	{
		auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		auto pid_str = dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
		uint16_t pid;
		std::stringstream ss;
		ss << std::hex << pid_str;
		ss >> pid;
		UINFO("Device with serial number %s was found with product ID=%d.", sn, (int)pid);
		if (deviceId_.empty() || deviceId_ == sn)
		{
			*dev_ = dev;
			found=true;
			break;
		}
	}

	if (!found)
	{
		UERROR("The requested device %s is NOT found!", deviceId_.c_str());
		return false;
	}

	ctx_->set_devices_changed_callback([this](rs2::event_information& info)
	{
		if (info.was_removed(*dev_))
		{
			UERROR("The device has been disconnected!");
		}
	});


	auto camera_name = dev_->get_info(RS2_CAMERA_INFO_NAME);
	UINFO("Device Name: %s", camera_name);

	auto sn = dev_->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	UINFO("Device Serial No: %s", sn);

	auto fw_ver = dev_->get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
	UINFO("Device FW version: %s", fw_ver);

	auto pid = dev_->get_info(RS2_CAMERA_INFO_PRODUCT_ID);
	UINFO("Device Product ID: 0x%s", pid);

	auto dev_sensors = dev_->query_sensors();

	UINFO("Device Sensors: ");
	std::vector<rs2::sensor> sensors(2); //0=rgb 1=depth
	for(auto&& elem : dev_sensors)
	{
		std::string module_name = elem.get_info(RS2_CAMERA_INFO_NAME);
		if ("Stereo Module" == module_name)
		{
			sensors[1] = elem;
			sensors[1].set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, emitterEnabled_);
			if(irDepth_)
			{
				sensors[0] = elem;
			}
		}
		else if ("Coded-Light Depth Sensor" == module_name)
		{
		}
		else if ("RGB Camera" == module_name)
		{
			if(!irDepth_)
			{
				sensors[0] = elem;
			}
		}
		else if ("Wide FOV Camera" == module_name)
		{
		}
		else if ("Motion Module" == module_name)
		{
		}
		else
		{
			UERROR("Module Name \"%s\" isn't supported by LibRealSense!", module_name.c_str());
			return false;
		}
		UINFO("%s was found.", elem.get_info(RS2_CAMERA_INFO_NAME));
	}

	UDEBUG("");

	model_ = CameraModel();
	rs2::stream_profile depthStreamProfile;
	rs2::stream_profile rgbStreamProfile;
	std::vector<std::vector<rs2::stream_profile> > profilesPerSensor(2);
	 for (unsigned int i=0; i<sensors.size(); ++i)
	 {
		 UDEBUG("i=%d", (int)i);
		auto profiles = sensors[i].get_stream_profiles();
		bool added = false;
		UDEBUG("profiles=%d", (int)profiles.size());
		for (auto& profile : profiles)
		{
			auto video_profile = profile.as<rs2::video_stream_profile>();
			if (video_profile.format() == (i==1?RS2_FORMAT_Z16:irDepth_?RS2_FORMAT_Y8:RS2_FORMAT_RGB8) &&
				video_profile.width()  == 640 &&
				video_profile.height() == 480 &&
				video_profile.fps()    == 30)
			{
				profilesPerSensor[irDepth_?1:i].push_back(profile);
				auto intrinsic = video_profile.get_intrinsics();
				if(i==1)
				{
					depthBuffer_ = cv::Mat(cv::Size(640, 480), CV_16UC1, cv::Scalar(0));
					depthStreamProfile = profile;
					*depthIntrinsics_ = intrinsic;
				}
				else
				{
					rgbBuffer_ = cv::Mat(cv::Size(640, 480), irDepth_?CV_8UC1:CV_8UC3, irDepth_?cv::Scalar(0):cv::Scalar(0, 0, 0));
					model_ = CameraModel(camera_name, intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy, this->getLocalTransform(), 0, cv::Size(intrinsic.width, intrinsic.height));
					rgbStreamProfile = profile;
					*rgbIntrinsics_ = intrinsic;
				}
				added = true;
				break;
			}
		}
		if (!added)
		{
			UERROR("Given stream configuration is not supported by the device! "
					"Stream Index: %d, Width: %d, Height: %d, FPS: %d", i, 640, 480, 30);
			return false;
		}
	 }

	 if(!model_.isValidForProjection())
	 {
		 UERROR("Calibration info not valid!");
		 return false;
	 }
	 *depthToRGBExtrinsics_ = depthStreamProfile.get_extrinsics_to(rgbStreamProfile);

	 for (unsigned int i=0; i<sensors.size(); ++i)
	 {
		 if(profilesPerSensor[i].size())
		 {
			 UINFO("Starting sensor %d with %d profiles", (int)i, (int)profilesPerSensor[i].size());
			 sensors[i].open(profilesPerSensor[i]);
			 if(i ==1)
			 {
				 auto depth_sensor = sensors[i].as<rs2::depth_sensor>();
				 depth_scale_meters_ = depth_sensor.get_depth_scale();
			 }
			 sensors[i].start(*syncer_);
		 }
	 }

	uSleep(1000); // ignore the first frames
	UINFO("Enabling streams...done!");

	return true;

#else
	UERROR("CameraRealSense: RTAB-Map is not built with RealSense2 support!");
	return false;
#endif
}

bool CameraRealSense2::isCalibrated() const
{
	return true;
}

std::string CameraRealSense2::getSerial() const
{
#ifdef RTABMAP_REALSENSE2
	return dev_->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
#endif
	return "NA";
}

void CameraRealSense2::setEmitterEnabled(bool enabled)
{
#ifdef RTABMAP_REALSENSE2
	emitterEnabled_ = enabled;
#endif
}

void CameraRealSense2::setIRDepthFormat(bool enabled)
{
#ifdef RTABMAP_REALSENSE2
	irDepth_ = enabled;
#endif
}

SensorData CameraRealSense2::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_REALSENSE2

	try{
		auto frameset = syncer_->wait_for_frames(5000);
		UTimer timer;
		while (frameset.size() != 2 && timer.elapsed() < 2.0)
		{
			// maybe there is a latency with the USB, try again in 100 ms (for the next 2 seconds)
			frameset = syncer_->wait_for_frames(100);
		}
		if (frameset.size() == 2)
		{
			double stamp = UTimer::now();
			UDEBUG("Frameset arrived.");
			bool is_rgb_arrived = false;
			bool is_depth_arrived = false;
			rs2::frame rgb_frame;
			rs2::frame depth_frame;
			for (auto it = frameset.begin(); it != frameset.end(); ++it)
			{
				auto f = (*it);
				auto stream_type = f.get_profile().stream_type();
				if (stream_type == RS2_STREAM_COLOR || stream_type == RS2_STREAM_INFRARED)
				{
					rgb_frame = f;
					is_rgb_arrived = true;
				}
				else if (stream_type == RS2_STREAM_DEPTH)
				{
					depth_frame = f;
					is_depth_arrived = true;
				}
			}

			if(is_rgb_arrived && is_depth_arrived)
			{
				auto from_image_frame = depth_frame.as<rs2::video_frame>();
				cv::Mat depth;
				if(irDepth_)
				{
					depth = cv::Mat(depthBuffer_.size(), depthBuffer_.type(), (void*)depth_frame.get_data()).clone();
				}
				else
				{
					depth = cv::Mat(depthBuffer_.size(), depthBuffer_.type());
					alignFrame(*depthIntrinsics_, *rgbIntrinsics_,
							depth_frame, from_image_frame.get_bytes_per_pixel(),
							*depthToRGBExtrinsics_, depth, depth_scale_meters_);
				}

				cv::Mat rgb = cv::Mat(rgbBuffer_.size(), rgbBuffer_.type(), (void*)rgb_frame.get_data());
				cv::Mat bgr;
				if(rgb.channels() == 3)
				{
					cv::cvtColor(rgb, bgr, CV_RGB2BGR);
				}
				else
				{
					bgr = rgb.clone();
				}

				data = SensorData(bgr, depth, model_, this->getNextSeqID(), stamp);
			}
			else
			{
				UERROR("Not received depth and rgb");
			}
		}
		else
		{
			UERROR("Missing frames (received %d)", (int)frameset.size());
		}
	}
	catch(const std::exception& ex)
	{
		UERROR("An error has occurred during frame callback: %s", ex.what());
	}
#else
	UERROR("CameraRealSense2: RTAB-Map is not built with RealSense2 support!");
#endif
	return data;
}

} // namespace rtabmap
