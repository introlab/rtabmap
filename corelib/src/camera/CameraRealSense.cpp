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

#include <rtabmap/core/camera/CameraRealSense.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UThreadC.h>
#include <rtabmap/core/util2d.h>
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_REALSENSE
#include <librealsense/rs.hpp>
#ifdef RTABMAP_REALSENSE_SLAM
#include <rs_core.h>
#include <rs_utils.h>
#include <librealsense/slam/slam.h>
#endif
#endif


namespace rtabmap
{

bool CameraRealSense::available()
{
#ifdef RTABMAP_REALSENSE
	return true;
#else
	return false;
#endif
}

CameraRealSense::CameraRealSense(
		int device,
		int presetRGB,
		int presetDepth,
		bool computeOdometry,
		float imageRate,
		const rtabmap::Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_REALSENSE
    ,
	ctx_(0),
	dev_(0),
	deviceId_(device),
	presetRGB_(presetRGB),
	presetDepth_(presetDepth),
	computeOdometry_(computeOdometry),
	depthScaledToRGBSize_(false),
	rgbSource_(kColor),
	slam_(0)
#endif
{
	UDEBUG("");
}

CameraRealSense::~CameraRealSense()
{
	UDEBUG("");
#ifdef RTABMAP_REALSENSE
	UDEBUG("");
	if(dev_)
	{
		try
		{
			if(slam_!=0)
			{
				dev_->stop(rs::source::all_sources);
			}
			else
			{
				dev_->stop();
			}
		}
		catch(const rs::error & error){UWARN("%s", error.what());}
		dev_ = 0;
	}
	UDEBUG("");
	if (ctx_)
	{
		delete ctx_;
	}
#ifdef RTABMAP_REALSENSE_SLAM
	UDEBUG("");
	if(slam_)
	{
		UScopeMutex lock(slamLock_);
		slam_->flush_resources();
		delete slam_;
		slam_ = 0;
	}
#endif
#endif
}

#ifdef RTABMAP_REALSENSE_SLAM
bool setStreamConfigIntrin(
		rs::core::stream_type stream,
		std::map< rs::core::stream_type, rs::core::intrinsics > intrinsics,
		rs::core::video_module_interface::supported_module_config & supported_config,
		rs::core::video_module_interface::actual_module_config & actual_config)
{
  auto & supported_stream_config = supported_config[stream];
  if (!supported_stream_config.is_enabled || supported_stream_config.size.width != intrinsics[stream].width || supported_stream_config.size.height != intrinsics[stream].height)
  {
	  UERROR("size of stream is not supported by slam");
	  			UERROR("  supported: stream %d, width: %d height: %d", (uint32_t) stream, supported_stream_config.size.width, supported_stream_config.size.height);
	  			UERROR("  received: stream %d, width: %d height: %d", (uint32_t) stream, intrinsics[stream].width, intrinsics[stream].height);

    return false;
  }
  rs::core::video_module_interface::actual_image_stream_config &actual_stream_config = actual_config[stream];
  actual_config[stream].size.width = intrinsics[stream].width;
  actual_config[stream].size.height = intrinsics[stream].height;
  actual_stream_config.frame_rate = supported_stream_config.frame_rate;
  actual_stream_config.intrinsics = intrinsics[stream];
  actual_stream_config.is_enabled = true;
  return true;
}
#endif

void CameraRealSense::setDepthScaledToRGBSize(bool enabled) {
#ifdef RTABMAP_REALSENSE
	depthScaledToRGBSize_ = enabled;
#endif
}
void CameraRealSense::setRGBSource(RGBSource source)
{
#ifdef RTABMAP_REALSENSE
	rgbSource_ = source;
#endif
}

#ifdef RTABMAP_REALSENSE
template<class GET_DEPTH, class TRANSFER_PIXEL> void align_images(const rs_intrinsics & depth_intrin, const rs_extrinsics & depth_to_other, const rs_intrinsics & other_intrin, GET_DEPTH get_depth, TRANSFER_PIXEL transfer_pixel)
{
	// Iterate over the pixels of the depth image
#pragma omp parallel for schedule(dynamic)
	for(int depth_y = 0; depth_y < depth_intrin.height; ++depth_y)
	{
		int depth_pixel_index = depth_y * depth_intrin.width;
		for(int depth_x = 0; depth_x < depth_intrin.width; ++depth_x, ++depth_pixel_index)
		{
			// Skip over depth pixels with the value of zero, we have no depth data so we will not write anything into our aligned images
			if(float depth = get_depth(depth_pixel_index))
			{
				// Map the top-left corner of the depth pixel onto the other image
				float depth_pixel[2] = {depth_x-0.5f, depth_y-0.5f}, depth_point[3], other_point[3], other_pixel[2];
				rs_deproject_pixel_to_point(depth_point, &depth_intrin, depth_pixel, depth);
				rs_transform_point_to_point(other_point, &depth_to_other, depth_point);
				rs_project_point_to_pixel(other_pixel, &other_intrin, other_point);
				const int other_x0 = static_cast<int>(other_pixel[0] + 0.5f);
				const int other_y0 = static_cast<int>(other_pixel[1] + 0.5f);

				// Map the bottom-right corner of the depth pixel onto the other image
				depth_pixel[0] = depth_x+0.5f; depth_pixel[1] = depth_y+0.5f;
				rs_deproject_pixel_to_point(depth_point, &depth_intrin, depth_pixel, depth);
				rs_transform_point_to_point(other_point, &depth_to_other, depth_point);
				rs_project_point_to_pixel(other_pixel, &other_intrin, other_point);
				const int other_x1 = static_cast<int>(other_pixel[0] + 0.5f);
				const int other_y1 = static_cast<int>(other_pixel[1] + 0.5f);

				if(other_x0 < 0 || other_y0 < 0 || other_x1 >= other_intrin.width || other_y1 >= other_intrin.height) continue;

				// Transfer between the depth pixels and the pixels inside the rectangle on the other image
				for(int y=other_y0; y<=other_y1; ++y) for(int x=other_x0; x<=other_x1; ++x) transfer_pixel(depth_pixel_index, y * other_intrin.width + x);
			}
		}
	}
}
typedef uint8_t byte;

void align_z_to_other(byte * z_aligned_to_other, const uint16_t * z_pixels, float z_scale, const rs_intrinsics & z_intrin, const rs_extrinsics & z_to_other, const rs_intrinsics & other_intrin)
{
	auto out_z = (uint16_t *)(z_aligned_to_other);
	align_images(z_intrin, z_to_other, other_intrin,
		[z_pixels, z_scale](int z_pixel_index) { return z_scale * z_pixels[z_pixel_index]; },
		[out_z, z_pixels](int z_pixel_index, int other_pixel_index) { out_z[other_pixel_index] = out_z[other_pixel_index] ? std::min(out_z[other_pixel_index],z_pixels[z_pixel_index]) : z_pixels[z_pixel_index]; });
}

void align_disparity_to_other(byte * disparity_aligned_to_other, const uint16_t * disparity_pixels, float disparity_scale, const rs_intrinsics & disparity_intrin, const rs_extrinsics & disparity_to_other, const rs_intrinsics & other_intrin)
{
	auto out_disparity = (uint16_t *)(disparity_aligned_to_other);
	align_images(disparity_intrin, disparity_to_other, other_intrin,
		[disparity_pixels, disparity_scale](int disparity_pixel_index) { return disparity_scale / disparity_pixels[disparity_pixel_index]; },
		[out_disparity, disparity_pixels](int disparity_pixel_index, int other_pixel_index) { out_disparity[other_pixel_index] = disparity_pixels[disparity_pixel_index]; });
}

template<int N> struct bytes { char b[N]; };
template<int N, class GET_DEPTH> void align_other_to_depth_bytes(byte * other_aligned_to_depth, GET_DEPTH get_depth, const rs_intrinsics & depth_intrin, const rs_extrinsics & depth_to_other, const rs_intrinsics & other_intrin, const byte * other_pixels)
{
	auto in_other = (const bytes<N> *)(other_pixels);
	auto out_other = (bytes<N> *)(other_aligned_to_depth);
	align_images(depth_intrin, depth_to_other, other_intrin, get_depth,
		[out_other, in_other](int depth_pixel_index, int other_pixel_index) { out_other[depth_pixel_index] = in_other[other_pixel_index]; });
}

/////////////////////////
// Image rectification //
/////////////////////////

std::vector<int> compute_rectification_table(const rs_intrinsics & rect_intrin, const rs_extrinsics & rect_to_unrect, const rs_intrinsics & unrect_intrin)
{
	std::vector<int> rectification_table;
	rectification_table.resize(rect_intrin.width * rect_intrin.height);
	align_images(rect_intrin, rect_to_unrect, unrect_intrin, [](int) { return 1.0f; },
		[&rectification_table](int rect_pixel_index, int unrect_pixel_index) { rectification_table[rect_pixel_index] = unrect_pixel_index; });
	return rectification_table;
}

template<class T> void rectify_image_pixels(T * rect_pixels, const std::vector<int> & rectification_table, const T * unrect_pixels)
{
	for(auto entry : rectification_table) *rect_pixels++ = unrect_pixels[entry];
}

void rectify_image(uint8_t * rect_pixels, const std::vector<int> & rectification_table, const uint8_t * unrect_pixels, rs_format format)
{
	switch(format)
	{
	case RS_FORMAT_Y8:
		return rectify_image_pixels((bytes<1> *)rect_pixels, rectification_table, (const bytes<1> *)unrect_pixels);
	case RS_FORMAT_Y16: case RS_FORMAT_Z16:
		return rectify_image_pixels((bytes<2> *)rect_pixels, rectification_table, (const bytes<2> *)unrect_pixels);
	case RS_FORMAT_RGB8: case RS_FORMAT_BGR8:
		return rectify_image_pixels((bytes<3> *)rect_pixels, rectification_table, (const bytes<3> *)unrect_pixels);
	case RS_FORMAT_RGBA8: case RS_FORMAT_BGRA8:
		return rectify_image_pixels((bytes<4> *)rect_pixels, rectification_table, (const bytes<4> *)unrect_pixels);
	default:
		assert(false); // NOTE: rectify_image_pixels(...) is not appropriate for RS_FORMAT_YUYV images, no logic prevents U/V channels from being written to one another
	}
}
#endif

bool CameraRealSense::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_REALSENSE

	if(dev_)
	{
		try
		{
			if(slam_!=0)
			{
				dev_->stop(rs::source::all_sources);
			}
			else
			{
				dev_->stop();
			}
		}
		catch(const rs::error & error){UWARN("%s", error.what());}
		dev_ = 0;
	}
	bufferedFrames_.clear();
	rsRectificationTable_.clear();

#ifdef RTABMAP_REALSENSE_SLAM
	motionSeq_[0] = motionSeq_[1] = 0;
	if(slam_)
	{
		UScopeMutex lock(slamLock_);
		UDEBUG("Flush slam");
		slam_->flush_resources();
		delete slam_;
		slam_ = 0;
	}
#endif

	if (ctx_ == 0)
	{
		ctx_ = new rs::context();
	}

	UDEBUG("");
	if (ctx_->get_device_count() == 0)
	{
		UERROR("No RealSense device detected!");
		return false;
	}

	UDEBUG("");
	dev_ = ctx_->get_device(deviceId_);
	if (dev_ == 0)
	{
		UERROR("Cannot connect to device %d", deviceId_);
		return false;
	}
	std::string name = dev_->get_name();
	UINFO("Using device %d, an %s", deviceId_, name.c_str());
	UINFO("    Serial number: %s", dev_->get_serial());
	UINFO("    Firmware version: %s", dev_->get_firmware_version());
	UINFO("    Preset RGB: %d", presetRGB_);
	UINFO("    Preset Depth: %d", presetDepth_);

#ifndef RTABMAP_REALSENSE_SLAM
	computeOdometry_ = false;
#endif

	if (name.find("ZR300") == std::string::npos)
	{
		// Only enable ZR300 functionality odometry if fisheye stream is enabled.
		// Accel/Gyro automatically enabled when fisheye requested
		computeOdometry_ = false;
		// Only ZR300 has fisheye
		if(rgbSource_ == kFishEye)
		{
			UWARN("Fisheye cannot be used with %s camera, using color instead...", name.c_str());
			rgbSource_ = kColor;
		}
	}

	rs::intrinsics depth_intrin;
	rs::intrinsics fisheye_intrin;
	rs::intrinsics color_intrin;
	// Configure depth and color to run with the device's preferred settings
	UINFO("Enabling streams...");
	// R200: 
	//    0=640x480   vs 480x360
	//    1=1920x1080 vs 640x480
	//    2=640x480   vs 320x240
	try {

		// left/rgb stream
		if(rgbSource_==kFishEye || computeOdometry_)
		{
			dev_->enable_stream(rs::stream::fisheye, 640, 480, rs::format::raw8, 30);
			if(computeOdometry_)
			{
				// Needed to align image timestamps to common clock-domain with the motion events
				dev_->set_option(rs::option::fisheye_strobe, 1);
			}
			// This option causes the fisheye image to be acquired in-sync with the depth image.
			dev_->set_option(rs::option::fisheye_external_trigger, 1);
			dev_->set_option(rs::option::fisheye_color_auto_exposure, 1);
			fisheye_intrin = dev_->get_stream_intrinsics(rs::stream::fisheye);
			UINFO("    Fisheye:   %dx%d", fisheye_intrin.width, fisheye_intrin.height);
			if(rgbSource_==kFishEye)
			{
				color_intrin = fisheye_intrin; // not rectified
			}
		}
		if(rgbSource_!=kFishEye)
		{
			dev_->enable_stream(rs::stream::color, (rs::preset)presetRGB_);
			color_intrin = dev_->get_stream_intrinsics(rs::stream::rectified_color); // rectified
			UINFO("    RGB:   %dx%d", color_intrin.width, color_intrin.height);

			if(rgbSource_==kInfrared)
			{
				dev_->enable_stream(rs::stream::infrared, (rs::preset)presetRGB_);
				color_intrin = dev_->get_stream_intrinsics(rs::stream::infrared); // rectified
				UINFO("    IR left:   %dx%d", color_intrin.width, color_intrin.height);
			}
		}

		dev_->enable_stream(rs::stream::depth, (rs::preset)presetDepth_);
		depth_intrin = dev_->get_stream_intrinsics(rs::stream::depth); // rectified
		UINFO("    Depth: %dx%d", depth_intrin.width, depth_intrin.height);
	}
	catch(const rs::error & error)
	{
		UERROR("Failed starting the streams: %s", error.what());
		return false;
	}

	Transform imu2Camera = Transform::getIdentity();

#ifdef RTABMAP_REALSENSE_SLAM
	UDEBUG("Setup frame callback");
	// Define lambda callback for receiving stream data
	std::function<void(rs::frame)> frameCallback = [this](rs::frame frame)
	{
		if(slam_ != 0)
		{
			const auto timestampDomain = frame.get_frame_timestamp_domain();
			if (rs::timestamp_domain::microcontroller != timestampDomain)
			{
				UERROR("error: Junk time stamp in stream: %d\twith frame counter: %d",
						(int)(frame.get_stream_type()), frame.get_frame_number());
				return ;
			}
		}

		int width = frame.get_width();
		int height = frame.get_height();
		rs::core::correlated_sample_set sample_set = {};

		rs::core::image_info info =
		{
		  width,
		  height,
		  rs::utils::convert_pixel_format(frame.get_format()),
		  frame.get_stride()
		};
		cv::Mat image;
		if(frame.get_format() == rs::format::raw8 || frame.get_format() == rs::format::y8)
		{
			image = cv::Mat(height, width, CV_8UC1, (unsigned char*)frame.get_data());
			if(frame.get_stream_type() == rs::stream::fisheye)
			{
				// fisheye always received just after the depth image (doesn't have exact timestamp with depth)
				if(bufferedFrames_.size())
				{
					bufferedFrames_.rbegin()->second.first = image.clone();
					UScopeMutex lock(dataMutex_);
					bool notify = lastSyncFrames_.first.empty();
					lastSyncFrames_ = bufferedFrames_.rbegin()->second;
					if(notify)
					{
						dataReady_.release();
					}
					bufferedFrames_.clear();
				}
			}
			else if(frame.get_stream_type() == rs::stream::infrared) // infrared (does have exact timestamp with depth)
			{
				if(bufferedFrames_.find(frame.get_timestamp()) != bufferedFrames_.end())
				{
					bufferedFrames_.find(frame.get_timestamp())->second.first = image.clone();
					UScopeMutex lock(dataMutex_);
					bool notify = lastSyncFrames_.first.empty();
					lastSyncFrames_ = bufferedFrames_.find(frame.get_timestamp())->second;
					if(notify)
					{
						dataReady_.release();
					}
					bufferedFrames_.erase(frame.get_timestamp());
				}
				else
				{
					bufferedFrames_.insert(std::make_pair(frame.get_timestamp(), std::make_pair(image.clone(), cv::Mat())));
				}
				if(bufferedFrames_.size()>5)
				{
					UWARN("Frames cannot be synchronized!");
					bufferedFrames_.clear();
				}
				return;
			}
		}
		else if(frame.get_format() == rs::format::z16)
		{
			image = cv::Mat(height, width, CV_16UC1, (unsigned char*)frame.get_data());
			if(bufferedFrames_.find(frame.get_timestamp()) != bufferedFrames_.end())
			{
				bufferedFrames_.find(frame.get_timestamp())->second.second = image.clone();
				UScopeMutex lock(dataMutex_);
				bool notify = lastSyncFrames_.first.empty();
				lastSyncFrames_ = bufferedFrames_.find(frame.get_timestamp())->second;
				if(notify)
				{
					dataReady_.release();
				}
				bufferedFrames_.erase(frame.get_timestamp());
			}
			else
			{
				bufferedFrames_.insert(std::make_pair(frame.get_timestamp(), std::make_pair(cv::Mat(), image.clone())));
			}
			if(bufferedFrames_.size()>5)
			{
				UWARN("Frames cannot be synchronized!");
				bufferedFrames_.clear();
			}
		}
		else if(frame.get_format() == rs::format::rgb8)
		{
			if(rsRectificationTable_.size())
			{
				image = cv::Mat(height, width, CV_8UC3);
				rectify_image(image.data, rsRectificationTable_, (unsigned char*)frame.get_data(), (rs_format)frame.get_format());
			}
			else
			{
				image = cv::Mat(height, width, CV_8UC3, (unsigned char*)frame.get_data());
			}
			if(bufferedFrames_.find(frame.get_timestamp()) != bufferedFrames_.end())
			{
				bufferedFrames_.find(frame.get_timestamp())->second.first = image.clone();
				UScopeMutex lock(dataMutex_);
				bool notify = lastSyncFrames_.first.empty();
				lastSyncFrames_ = bufferedFrames_.find(frame.get_timestamp())->second;
				if(notify)
				{
					dataReady_.release();
				}
				bufferedFrames_.erase(frame.get_timestamp());
			}
			else
			{
				bufferedFrames_.insert(std::make_pair(frame.get_timestamp(), std::make_pair(image.clone(), cv::Mat())));
			}
			if(bufferedFrames_.size()>5)
			{
				UWARN("Frames cannot be synchronized!");
				bufferedFrames_.clear();
			}
			return;
		}
		else
		{
			return;
		}

		if(slam_ != 0)
		{
			rs::core::stream_type stream = rs::utils::convert_stream_type(frame.get_stream_type());
			sample_set[stream] = rs::core::image_interface::create_instance_from_raw_data(
								   & info,
								   image.data,
								   stream,
								   rs::core::image_interface::flag::any,
								   frame.get_timestamp(),
								   (uint64_t)frame.get_frame_number(),
								   rs::core::timestamp_domain::microcontroller);

			UScopeMutex lock(slamLock_);
			if (slam_->process_sample_set(sample_set) < rs::core::status_no_error)
			{
				UERROR("error: failed to process sample");
			}
			sample_set[stream]->release();
		}
	};

	UDEBUG("");
	// Setup stream callback for stream
	if(computeOdometry_ || rgbSource_ == kFishEye)
	{
		dev_->set_frame_callback(rs::stream::fisheye, frameCallback);
	}
	if(rgbSource_ == kInfrared)
	{
		dev_->set_frame_callback(rs::stream::infrared, frameCallback);
	}
	else if(rgbSource_ == kColor)
	{
		dev_->set_frame_callback(rs::stream::color, frameCallback);
	}

	dev_->set_frame_callback(rs::stream::depth, frameCallback);

	UDEBUG("");

	if (computeOdometry_)
	{
		UDEBUG("Setup motion callback");
		//define callback to the motion events and set it.
		std::function<void(rs::motion_data)> motion_callback;
		motion_callback = [this](rs::motion_data entry)
		{
			if ((entry.timestamp_data.source_id != RS_EVENT_IMU_GYRO) &&
					(entry.timestamp_data.source_id != RS_EVENT_IMU_ACCEL))
				return;

			rs_event_source motionType = entry.timestamp_data.source_id;

			rs::core::correlated_sample_set sample_set = {};
			if (motionType == RS_EVENT_IMU_ACCEL)
			{
				sample_set[rs::core::motion_type::accel].timestamp = entry.timestamp_data.timestamp;
				sample_set[rs::core::motion_type::accel].data[0] = (float)entry.axes[0];
				sample_set[rs::core::motion_type::accel].data[1] = (float)entry.axes[1];
				sample_set[rs::core::motion_type::accel].data[2] = (float)entry.axes[2];
				sample_set[rs::core::motion_type::accel].type = rs::core::motion_type::accel;
				++motionSeq_[0];
				sample_set[rs::core::motion_type::accel].frame_number = motionSeq_[0];
			}
			else if (motionType == RS_EVENT_IMU_GYRO)
			{
				sample_set[rs::core::motion_type::gyro].timestamp = entry.timestamp_data.timestamp;
				sample_set[rs::core::motion_type::gyro].data[0] = (float)entry.axes[0];
				sample_set[rs::core::motion_type::gyro].data[1] = (float)entry.axes[1];
				sample_set[rs::core::motion_type::gyro].data[2] = (float)entry.axes[2];
				sample_set[rs::core::motion_type::gyro].type = rs::core::motion_type::gyro;
				++motionSeq_[1];
				sample_set[rs::core::motion_type::gyro].frame_number = motionSeq_[1];
			}

			UScopeMutex lock(slamLock_);
			if (slam_->process_sample_set(sample_set) < rs::core::status_no_error)
			{
				UERROR("error: failed to process sample");
			}
		};

		std::function<void(rs::timestamp_data)> timestamp_callback;
		timestamp_callback = [](rs::timestamp_data entry) {};

		dev_->enable_motion_tracking(motion_callback, timestamp_callback);
		UINFO("  enabled accel and gyro stream");

		rs::motion_intrinsics imuIntrinsics;
		rs::extrinsics fisheye2ImuExtrinsics;
		rs::extrinsics fisheye2DepthExtrinsics;
		try
		{
			imuIntrinsics = dev_->get_motion_intrinsics();
			fisheye2ImuExtrinsics = dev_->get_motion_extrinsics_from(rs::stream::fisheye);
			fisheye2DepthExtrinsics = dev_->get_extrinsics(rs::stream::depth, rs::stream::fisheye);
		}
		catch (const rs::error & e) {
			UERROR("Exception: %s (try to unplug/plug the camera)", e.what());
			return false;
		}

		UDEBUG("Setup SLAM");
		UScopeMutex lock(slamLock_);
		slam_ = new rs::slam::slam();
		slam_->set_auto_occupancy_map_building(false);
		slam_->force_relocalization_pose(false);

		rs::core::video_module_interface::supported_module_config supported_config = {};
		if (slam_->query_supported_module_config(0, supported_config) < rs::core::status_no_error)
		{
			UERROR("Failed to query the first supported module configuration");
			return false;
		}

		rs::core::video_module_interface::actual_module_config actual_config = {};

		// Set camera intrinsics
		std::map< rs::core::stream_type, rs::core::intrinsics > intrinsics;
		intrinsics[rs::core::stream_type::fisheye] = rs::utils::convert_intrinsics(fisheye_intrin);
		intrinsics[rs::core::stream_type::depth] = rs::utils::convert_intrinsics(depth_intrin);

		if(!setStreamConfigIntrin(rs::core::stream_type::fisheye, intrinsics, supported_config, actual_config))
		{
			return false;
		}
		if(!setStreamConfigIntrin(rs::core::stream_type::depth, intrinsics, supported_config, actual_config))
		{
			return false;
		}

		// Set IMU intrinsics
		actual_config[rs::core::motion_type::accel].is_enabled = true;
		actual_config[rs::core::motion_type::gyro].is_enabled = true;
		actual_config[rs::core::motion_type::gyro].intrinsics = rs::utils::convert_motion_device_intrinsics(imuIntrinsics.gyro);
		actual_config[rs::core::motion_type::accel].intrinsics = rs::utils::convert_motion_device_intrinsics(imuIntrinsics.acc);

		// Set extrinsics
		actual_config[rs::core::stream_type::fisheye].extrinsics_motion = rs::utils::convert_extrinsics(fisheye2ImuExtrinsics);
		actual_config[rs::core::stream_type::fisheye].extrinsics = rs::utils::convert_extrinsics(fisheye2DepthExtrinsics);

		UDEBUG("Set SLAM config");
		// Set actual config
		if (slam_->set_module_config(actual_config) < rs::core::status_no_error)
		{
			UERROR("error : failed to set the enabled module configuration");
			return false;
		}

		rs::extrinsics fisheye2imu = dev_->get_motion_extrinsics_from(rs::stream::fisheye);
		imu2Camera = Transform(
				fisheye2imu.rotation[0], fisheye2imu.rotation[1], fisheye2imu.rotation[2], fisheye2imu.translation[0],
				fisheye2imu.rotation[3], fisheye2imu.rotation[4], fisheye2imu.rotation[5], fisheye2imu.translation[1],
				fisheye2imu.rotation[6], fisheye2imu.rotation[7], fisheye2imu.rotation[8], fisheye2imu.translation[2]).inverse();

		if(rgbSource_ == kInfrared)
		{
			rs::extrinsics color2Fisheye = dev_->get_extrinsics(rs::stream::fisheye, rs::stream::infrared);
			Transform fisheye2Color = Transform(
					color2Fisheye.rotation[0], color2Fisheye.rotation[1], color2Fisheye.rotation[2], color2Fisheye.translation[0],
					color2Fisheye.rotation[3], color2Fisheye.rotation[4], color2Fisheye.rotation[5], color2Fisheye.translation[1],
					color2Fisheye.rotation[6], color2Fisheye.rotation[7], color2Fisheye.rotation[8], color2Fisheye.translation[2]).inverse();
			imu2Camera *= fisheye2Color;
		}
		else if(rgbSource_ == kColor)
		{
			rs::extrinsics color2Fisheye = dev_->get_extrinsics(rs::stream::fisheye, rs::stream::rectified_color);
			Transform fisheye2Color = Transform(
					color2Fisheye.rotation[0], color2Fisheye.rotation[1], color2Fisheye.rotation[2], color2Fisheye.translation[0],
					color2Fisheye.rotation[3], color2Fisheye.rotation[4], color2Fisheye.rotation[5], color2Fisheye.translation[1],
					color2Fisheye.rotation[6], color2Fisheye.rotation[7], color2Fisheye.rotation[8], color2Fisheye.translation[2]).inverse();
			imu2Camera *= fisheye2Color;
		}

		UDEBUG("start device!");
		try
		{
			dev_->start(rs::source::all_sources);
		}
		catch(const rs::error & error)
		{
			UERROR("Failed starting the device: %s (try to unplug/plug the camera)", error.what());
			return false;
		}
	}
	else
	{
		UDEBUG("start device!");
		try
		{
			dev_->start();
		}
		catch(const rs::error & error)
		{
			UERROR("Failed starting the device: %s (try to unplug/plug the camera)", error.what());
			return false;
		}
	}
#else
	try {
		dev_->start();
		dev_->wait_for_frames();
	}
	catch (const rs::error & e)
	{
		UERROR("Exception: %s (try to unplug/plug the camera)", e.what());
	}
#endif

	cv::Mat D;
	if(rgbSource_ == kFishEye)
	{
		// ftheta/equidistant model
		D = cv::Mat::zeros(1,6,CV_64FC1);
		D.at<double>(0,0) = color_intrin.coeffs[0];
		D.at<double>(0,1) = color_intrin.coeffs[1];
		D.at<double>(0,4) = color_intrin.coeffs[2];
		D.at<double>(0,5) = color_intrin.coeffs[3];
	}
	else
	{
		// Brown-Conrady / radtan
		D = cv::Mat::zeros(1,5,CV_64FC1);
		D.at<double>(0,0) = color_intrin.coeffs[0];
		D.at<double>(0,1) = color_intrin.coeffs[1];
		D.at<double>(0,2) = color_intrin.coeffs[2];
		D.at<double>(0,3) = color_intrin.coeffs[3];
		D.at<double>(0,4) = color_intrin.coeffs[4];
	}
	cv::Mat K = cv::Mat::eye(3, 3, CV_64FC1);
	K.at<double>(0,0) = color_intrin.fx;
	K.at<double>(1,1) = color_intrin.fy;
	K.at<double>(0,2) = color_intrin.ppx;
	K.at<double>(1,2) = color_intrin.ppy;
	cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat P = cv::Mat::eye(3, 4, CV_64FC1);
	K(cv::Range(0,2), cv::Range(0,3)).copyTo(P(cv::Range(0,2), cv::Range(0,3)));
	cameraModel_ = CameraModel(
			dev_->get_name(),
			cv::Size(color_intrin.width, color_intrin.height),
			K,
			D,
			R,
			P,
			this->getLocalTransform()*imu2Camera);

	UDEBUG("");

	if(rgbSource_ == kColor)
	{
		rs::extrinsics rect_to_unrect = dev_->get_extrinsics(rs::stream::rectified_color, rs::stream::color);
		rs::intrinsics unrect_intrin = dev_->get_stream_intrinsics(rs::stream::color);
		rsRectificationTable_ = compute_rectification_table(color_intrin, rect_to_unrect, unrect_intrin);
	}
	else if(rgbSource_ == kFishEye)
	{
		UINFO("calibration folder=%s name=%s", calibrationFolder.c_str(), cameraName.c_str());
		if(!calibrationFolder.empty() && !cameraName.empty())
		{
			CameraModel model;
			if(!model.load(calibrationFolder, cameraName))
			{
				UWARN("Failed to load calibration \"%s\" in \"%s\" folder, you should calibrate the camera!",
						cameraName.c_str(), calibrationFolder.c_str());
			}
			else
			{
				UINFO("Camera parameters: fx=%f fy=%f cx=%f cy=%f",
						model.fx(),
						model.fy(),
						model.cx(),
						model.cy());
				cameraModel_ = model;
				cameraModel_.setName(cameraName);
				cameraModel_.initRectificationMap();
				cameraModel_.setLocalTransform(this->getLocalTransform()*imu2Camera);
			}
		}
	}

	uSleep(1000); // ignore the first frames
	UINFO("Enabling streams...done!");

	return true;

#else
	UERROR("CameraRealSense: RTAB-Map is not built with RealSense support!");
	return false;
#endif
}

bool CameraRealSense::isCalibrated() const
{
	return true;
}

std::string CameraRealSense::getSerial() const
{
#ifdef RTABMAP_REALSENSE
	if (dev_)
	{
		return dev_->get_serial();
	}
	else
	{
		UERROR("Cannot get a serial before initialization. Call init() before.");
	}
#endif
	return "NA";
}

bool CameraRealSense::odomProvided() const
{
#ifdef RTABMAP_REALSENSE_SLAM
	return slam_!=0;
#else
	return false;
#endif
}

#ifdef RTABMAP_REALSENSE_SLAM
Transform rsPoseToTransform(const rs::slam::PoseMatrix4f & pose)
{
	return Transform(
			pose.m_data[0], pose.m_data[1], pose.m_data[2], pose.m_data[3],
			pose.m_data[4], pose.m_data[5], pose.m_data[6], pose.m_data[7],
			pose.m_data[8], pose.m_data[9], pose.m_data[10], pose.m_data[11]);
}
#endif

SensorData CameraRealSense::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_REALSENSE
	if (dev_)
	{
		cv::Mat rgb;
		cv::Mat depthIn;

		// Retrieve camera parameters for mapping between depth and color
		rs::intrinsics depth_intrin = dev_->get_stream_intrinsics(rs::stream::depth);
		rs::extrinsics depth_to_color;
		rs::intrinsics color_intrin;
		if(rgbSource_ == kFishEye)
		{
			depth_to_color = dev_->get_extrinsics(rs::stream::depth, rs::stream::fisheye);
			color_intrin = dev_->get_stream_intrinsics(rs::stream::fisheye);
		}
		else if(rgbSource_ == kInfrared)
		{
			depth_to_color = dev_->get_extrinsics(rs::stream::depth, rs::stream::infrared);
			color_intrin = dev_->get_stream_intrinsics(rs::stream::infrared);
		}
		else // color
		{
			depth_to_color = dev_->get_extrinsics(rs::stream::depth, rs::stream::rectified_color);
			color_intrin = dev_->get_stream_intrinsics(rs::stream::rectified_color);
		}

#ifdef RTABMAP_REALSENSE_SLAM
		if(!dataReady_.acquire(1, 5000))
		{
			UWARN("Not received new frames since 5 seconds, end of stream reached!");
			return data;
		}

		{
			UScopeMutex lock(dataMutex_);
			rgb = lastSyncFrames_.first;
			depthIn = lastSyncFrames_.second;
			lastSyncFrames_.first = cv::Mat();
			lastSyncFrames_.second = cv::Mat();
		}

		if(rgb.empty() || depthIn.empty())
		{
			return data;
		}
#else
		try {
			dev_->wait_for_frames();
		}
		catch (const rs::error & e)
		{
			UERROR("Exception: %s", e.what());
			return data;
		}

		// Retrieve our images
		depthIn = cv::Mat(depth_intrin.height, depth_intrin.width, CV_16UC1, (unsigned char*)dev_->get_frame_data(rs::stream::depth));
		if(rgbSource_ == kFishEye)
		{
			rgb = cv::Mat(color_intrin.height, color_intrin.width, CV_8UC1, (unsigned char*)dev_->get_frame_data(rs::stream::fisheye));
		}
		else if(rgbSource_ == kInfrared)
		{
			rgb = cv::Mat(color_intrin.height, color_intrin.width, CV_8UC1, (unsigned char*)dev_->get_frame_data(rs::stream::infrared));
		}
		else
		{
			rgb = cv::Mat(color_intrin.height, color_intrin.width, CV_8UC3, (unsigned char*)dev_->get_frame_data(rs::stream::color));
		}
#endif

		// factory registration...
		cv::Mat bgr;
		if(rgbSource_ != kColor)
		{
			bgr = rgb;
		}
		else
		{
			cv::cvtColor(rgb, bgr, CV_RGB2BGR);
		}

		bool rectified = false;
		if(rgbSource_ == kFishEye && cameraModel_.isRectificationMapInitialized())
		{
			bgr = cameraModel_.rectifyImage(bgr);
			rectified = true;
			color_intrin.fx = cameraModel_.fx();
			color_intrin.fy = cameraModel_.fy();
			color_intrin.ppx = cameraModel_.cx();
			color_intrin.ppy = cameraModel_.cy();
			UASSERT_MSG(color_intrin.width == cameraModel_.imageWidth() && color_intrin.height == cameraModel_.imageHeight(),
					uFormat("color_intrin=%dx%d cameraModel_=%dx%d",
							color_intrin.width, color_intrin.height, cameraModel_.imageWidth(), cameraModel_.imageHeight()).c_str());
			((rs_intrinsics*)&color_intrin)->model = RS_DISTORTION_NONE;
		}
#ifndef RTABMAP_REALSENSE_SLAM
		else if(rgbSource_ != kColor)
		{
			bgr = bgr.clone();
		}
#endif

		cv::Mat depth;
		if(rgbSource_ != kFishEye || rectified)
		{
			if (color_intrin.width % depth_intrin.width == 0 && color_intrin.height % depth_intrin.height == 0 &&
				depth_intrin.width < color_intrin.width &&
				depth_intrin.height < color_intrin.height &&
				!depthScaledToRGBSize_)
			{
				//we can keep the depth image size as is
				depth = cv::Mat::zeros(cv::Size(depth_intrin.width, depth_intrin.height), CV_16UC1);
				float scaleX = float(depth_intrin.width) / float(color_intrin.width);
				float scaleY = float(depth_intrin.height) / float(color_intrin.height);
				color_intrin.fx *= scaleX;
				color_intrin.fy *= scaleY;
				color_intrin.ppx *= scaleX;
				color_intrin.ppy *= scaleY;
				color_intrin.height = depth_intrin.height;
				color_intrin.width = depth_intrin.width;
			}
			else
			{
				//depth to color
				depth = cv::Mat::zeros(bgr.size(), CV_16UC1);
			}

			float scale = dev_->get_depth_scale();
			for (int dy = 0; dy < depth_intrin.height; ++dy)
			{
				for (int dx = 0; dx < depth_intrin.width; ++dx)
				{
					// Retrieve the 16-bit depth value and map it into a depth in meters
					uint16_t depth_value = depthIn.at<unsigned short>(dy,dx);
					float depth_in_meters = depth_value * scale;

					// Skip over pixels with a depth value of zero, which is used to indicate no data
					if (depth_value == 0 || depth_in_meters>10.0f) continue;

					// Map from pixel coordinates in the depth image to pixel coordinates in the color image
					int pdx = dx;
					int pdy = dy;
					if(rgbSource_ == kColor || rgbSource_ == kFishEye)
					{
						rs::float2 depth_pixel = { (float)dx, (float)dy };
						rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
						rs::float3 color_point = depth_to_color.transform(depth_point);
						rs::float2 color_pixel = color_intrin.project(color_point);

						pdx = color_pixel.x;
						pdy = color_pixel.y;
					}
					//else infrared is already registered

					if (uIsInBounds(pdx, 0, depth.cols) && uIsInBounds(pdy, 0, depth.rows))
					{
						depth.at<unsigned short>(pdy, pdx) = (unsigned short)(depth_in_meters*1000.0f); // convert to mm
					}
				}
			}

			if (color_intrin.width > depth_intrin.width)
			{
				// Fill holes
				UTimer time;
				util2d::fillRegisteredDepthHoles(depth, true, true, color_intrin.width > depth_intrin.width * 2);
				util2d::fillRegisteredDepthHoles(depth, true, true, color_intrin.width > depth_intrin.width * 2);//second pass
				UDEBUG("Filling depth holes: %fs", time.ticks());
			}
		}

		if (!bgr.empty() && ((rgbSource_==kFishEye && !rectified) || !depth.empty()))
		{
			data = SensorData(bgr, depth, cameraModel_, this->getNextSeqID(), UTimer::now());
#ifdef RTABMAP_REALSENSE_SLAM
			if(info && slam_)
			{
				UScopeMutex lock(slamLock_);
				rs::slam::PoseMatrix4f pose;
				if(slam_->get_camera_pose(pose) == rs::core::status_no_error)
				{
					/*rs::slam::tracking_accuracy accuracy = slam_->get_tracking_accuracy();
					if( accuracy == rs::slam::tracking_accuracy::low ||
						accuracy == rs::slam::tracking_accuracy::medium ||
						accuracy == rs::slam::tracking_accuracy::high)*/
					{
						// the pose is in camera link or IMU frame, get pose of the color camera
						Transform opticalRotation(0,0,1,0, -1,0,0,0, 0,-1,0,0);
						info->odomPose = opticalRotation * rsPoseToTransform(pose) * opticalRotation.inverse();
						info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.0005;
					}
					/*else
					{
						UERROR("Odometry failed: accuracy=%d", accuracy);
					}*/
				}
				else
				{
					UERROR("Failed getting odometry pose");
				}
			}
#endif
		}
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}
#else
	UERROR("CameraRealSense: RTAB-Map is not built with RealSense support!");
#endif
	return data;
}

} // namespace rtabmap
