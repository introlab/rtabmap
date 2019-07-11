/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap/core/camera/CameraK4A.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThreadC.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/Compression.h>
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_K4A
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#endif

namespace rtabmap
{

bool CameraK4A::available()
{
#ifdef RTABMAP_K4A
	return true;
#else
	return false;
#endif
}

CameraK4A::CameraK4A(
	int deviceId,
	float imageRate,
	const Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_K4A
	,playbackHandle_(NULL),
	transformationHandle_(NULL),
	deviceId_(deviceId),
	ir_(false),
	previousStamp_(0.0)
#endif
{
	UERROR("CameraK4A: Live camera stream is not yet supported, only recorded mkv files are.");
}

CameraK4A::CameraK4A(
	const std::string & fileName,
	float imageRate,
	const Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_K4A
	,playbackHandle_(NULL),
	transformationHandle_(NULL),
	deviceId_(-1),
	fileName_(fileName),
	ir_(false),
	previousStamp_(0.0)
#endif
{
}

CameraK4A::~CameraK4A()
{
	close();
}

void CameraK4A::close()
{
#ifdef RTABMAP_K4A
	if (playbackHandle_ != NULL)
	{
		k4a_playback_close((k4a_playback_t)playbackHandle_);
	}
	if (transformationHandle_ != NULL)
	{
		k4a_transformation_destroy((k4a_transformation_t)transformationHandle_);
	}
	/*
	// Shut down the camera when finished with application logic
	k4a_device_stop_cameras(device);
	k4a_device_close(device);
	*/
#endif
}

void CameraK4A::setIRDepthFormat(bool enabled)
{
#ifdef RTABMAP_K4A
	ir_ = enabled;
#endif
}

bool CameraK4A::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_K4A
	
	close();

	if (!fileName_.empty())
	{
		if (k4a_playback_open(fileName_.c_str(), (k4a_playback_t*)&playbackHandle_) != K4A_RESULT_SUCCEEDED)
		{
			UERROR("Failed to open recording \"%s\"", fileName_.c_str());
			return false;
		}

		uint64_t recording_length = k4a_playback_get_last_timestamp_usec((k4a_playback_t)playbackHandle_);
		UINFO("Recording is %lld seconds long", recording_length / 1000000);

		k4a_calibration_t calibration;
		if (k4a_playback_get_calibration((k4a_playback_t)playbackHandle_, &calibration))
		{
			UERROR("Failed to get calibration");
			close();
			return false;
		}
		if (ir_)
		{
			model_ = CameraModel(
				calibration.depth_camera_calibration.intrinsics.parameters.param.fx,
				calibration.depth_camera_calibration.intrinsics.parameters.param.fy,
				calibration.depth_camera_calibration.intrinsics.parameters.param.cx,
				calibration.depth_camera_calibration.intrinsics.parameters.param.cy,
				this->getLocalTransform(),
				0,
				cv::Size(calibration.depth_camera_calibration.resolution_width, calibration.depth_camera_calibration.resolution_height));
		}
		else
		{
			model_ = CameraModel(
				calibration.color_camera_calibration.intrinsics.parameters.param.fx,
				calibration.color_camera_calibration.intrinsics.parameters.param.fy,
				calibration.color_camera_calibration.intrinsics.parameters.param.cx,
				calibration.color_camera_calibration.intrinsics.parameters.param.cy,
				this->getLocalTransform(),
				0,
				cv::Size(calibration.color_camera_calibration.resolution_width, calibration.color_camera_calibration.resolution_height));

			transformationHandle_ = k4a_transformation_create(&calibration);
		}

		k4a_record_configuration_t config;
		if (k4a_playback_get_record_configuration((k4a_playback_t)playbackHandle_, &config))
		{
			UERROR("Failed to getting recording configuration");
			close();
			return false;
		}
	}
	else if (deviceId_ >= 0)
	{
		UERROR("CameraK4A: Live camera stream is not yet supported, only recorded mkv files are.");
		return false;

		/*uint32_t count = k4a_device_get_installed_count();
		if (count == 0)
		{
			UERROR("No k4a devices attached!");
			return false;
		}

		// Open the first plugged in Kinect device
		k4a_device_t device = NULL;
		if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device)))
		{
			UERROR("Failed to open k4a device!");
			return false;
		}

		// Get the size of the serial number
		size_t serial_size = 0;
		k4a_device_get_serialnum(device, NULL, &serial_size);

		// Allocate memory for the serial, then acquire it
		char *serial = (char*)(malloc(serial_size));
		k4a_device_get_serialnum(device, serial, &serial_size);
		UINFO("Opened device: %s", serial);
		free(serial);

		// Configure a stream of 4096x3072 BRGA color data at 15 frames per second
		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config.camera_fps = K4A_FRAMES_PER_SECOND_15;
		config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		config.color_resolution = K4A_COLOR_RESOLUTION_3072P;

		// Start the camera with the given configuration
		if (K4A_FAILED(k4a_device_start_cameras(device, &config)))
		{
			UERROR("Failed to start cameras!");
			k4a_device_close(device);
			return false;
		}*/
	}
	
	return true;
#else
	UERROR("CameraK4A: RTAB-Map is not built with Kinect for Azure SDK support!");
	return false;
#endif
}

bool CameraK4A::isCalibrated() const
{
	return true;
}

std::string CameraK4A::getSerial() const
{
#ifdef RTABMAP_K4A
	return fileName_.empty()?"":fileName_;
#else
	return "";
#endif
}

SensorData CameraK4A::captureImage(CameraInfo * info)
{
	SensorData data;

#ifdef RTABMAP_K4A

	if (playbackHandle_ != NULL)
	{
		k4a_capture_t capture = NULL;
		k4a_stream_result_t result = K4A_STREAM_RESULT_SUCCEEDED;

		// wait to get all frames
		UTimer time;
		while (result == K4A_STREAM_RESULT_SUCCEEDED && time.elapsed() < 5.0)
		{
			result = k4a_playback_get_next_capture((k4a_playback_t)playbackHandle_, &capture);

			if (result == K4A_STREAM_RESULT_SUCCEEDED)
			{
				cv::Mat bgrCV;
				cv::Mat depthCV;
				double stamp = 0;

				// Process capture here
				if (ir_)
				{
					k4a_image_t ir = k4a_capture_get_ir_image(capture);
					if (ir != NULL)
					{
						/*UDEBUG("ir res:%4dx%4d stride:%5d format:%d stamp=%f",
							k4a_image_get_height_pixels(ir),
							k4a_image_get_width_pixels(ir),
							k4a_image_get_stride_bytes(ir),
							k4a_image_get_format(ir),
							double(k4a_image_get_timestamp_usec(ir)) / 1000000.0);*/

						UASSERT(k4a_image_get_format(ir) == K4A_IMAGE_FORMAT_IR16);

						cv::Mat bgrCV16(k4a_image_get_height_pixels(ir), k4a_image_get_width_pixels(ir), CV_16UC1, (void*)k4a_image_get_buffer(ir));
						bgrCV16.convertTo(bgrCV, CV_8U);

						// Release the image
						k4a_image_release(ir);
					}
				}
				else
				{
					k4a_image_t color = k4a_capture_get_color_image(capture);
					if (color != NULL)
					{
						/*UDEBUG("Color res:%4dx%4d stride:%5d format:%d stamp=%f",
							k4a_image_get_height_pixels(color),
							k4a_image_get_width_pixels(color),
							k4a_image_get_stride_bytes(color),
							k4a_image_get_format(color),
							double(k4a_image_get_timestamp_usec(color)) / 1000000.0);*/

						UASSERT(k4a_image_get_format(color) == K4A_IMAGE_FORMAT_COLOR_MJPG || k4a_image_get_format(color) == K4A_IMAGE_FORMAT_COLOR_BGRA32);

						if (k4a_image_get_format(color) == K4A_IMAGE_FORMAT_COLOR_MJPG)
						{
							bgrCV = uncompressImage(cv::Mat(1, (int)k4a_image_get_size(color), CV_8UC1, (void*)k4a_image_get_buffer(color)));
							//UDEBUG("Uncompressed = %d %d %d", bgrCV.rows, bgrCV.cols, bgrCV.channels());
						}
						else
						{
							cv::Mat bgra(k4a_image_get_height_pixels(color), k4a_image_get_width_pixels(color), CV_8UC4, (void*)k4a_image_get_buffer(color));
							cv::cvtColor(bgra, bgrCV, CV_BGRA2BGR);
						}

						// Release the image
						k4a_image_release(color);
					}
				}

				if (!bgrCV.empty())
				{
					k4a_image_t depth = k4a_capture_get_depth_image(capture);
					if (depth != NULL)
					{
						/*UDEBUG("Depth16 res:%4dx%4d stride:%5d format:%d stamp=%f",
							k4a_image_get_height_pixels(depth),
							k4a_image_get_width_pixels(depth),
							k4a_image_get_stride_bytes(depth),
							k4a_image_get_format(depth),
							double(k4a_image_get_timestamp_usec(depth)) / 1000000.0);*/

						UASSERT(k4a_image_get_format(depth) == K4A_IMAGE_FORMAT_DEPTH16);

						stamp = ((double)k4a_image_get_timestamp_usec(depth)) / 1000000;

						if (ir_)
						{
							depthCV = cv::Mat(k4a_image_get_height_pixels(depth), k4a_image_get_width_pixels(depth), CV_16UC1, (void*)k4a_image_get_buffer(depth)).clone();
						}
						else
						{
							k4a_image_t transformedDepth;
							if (k4a_image_create(k4a_image_get_format(depth), bgrCV.cols, bgrCV.rows, bgrCV.cols * 2, &transformedDepth) == K4A_RESULT_SUCCEEDED)
							{
								if (k4a_transformation_depth_image_to_color_camera((k4a_transformation_t)transformationHandle_, depth, transformedDepth) == K4A_RESULT_SUCCEEDED)
								{
									depthCV = cv::Mat(k4a_image_get_height_pixels(transformedDepth), k4a_image_get_width_pixels(transformedDepth), CV_16UC1, (void*)k4a_image_get_buffer(transformedDepth)).clone();
								}
								else
								{
									UERROR("Failed registration!");
								}
								k4a_image_release(transformedDepth);
							}
							else
							{
								UERROR("Failed allocating depth registered! (%d %d %d)", bgrCV.cols, bgrCV.rows, bgrCV.cols * 2);
							}
							
						}

						// Release the image
						k4a_image_release(depth);
					}
				}

				k4a_capture_release(capture);

				IMU imu;
				// FIXME: local imu transform missing
				/*k4a_imu_sample_t imuSample;
				if (k4a_playback_get_next_imu_sample((k4a_playback_t)playbackHandle_, &imuSample) == K4A_STREAM_RESULT_SUCCEEDED)
				{
					// K4A IMU Co-ordinates
					//  x+ = "backwards"
					//  y+ = "left"
					//  z+ = "down"
					//
					// ROS Standard co-ordinates:
					//  x+ = "forward"
					//  y+ = "left"
					//  z+ = "up"
					//
					// Remap K4A IMU to ROS co-ordinate system:
					// ROS_X+ = K4A_X-
					// ROS_Y+ = K4A_Y+
					// ROS_Z+ = K4A_Z-

					imu = IMU(
						cv::Vec3d(-1*imuSample.gyro_sample.xyz.x, imuSample.gyro_sample.xyz.y, -1 * imuSample.gyro_sample.xyz.z),
						cv::Mat::eye(3, 3, CV_64FC1),
						cv::Vec3d(-1 * imuSample.acc_sample.xyz.x, imuSample.acc_sample.xyz.y, -1 * imuSample.acc_sample.xyz.z),
						cv::Mat::eye(3, 3, CV_64FC1),
						Transform::getIdentity());
				}*/

				if (!bgrCV.empty() && !depthCV.empty())
				{
					data = SensorData(bgrCV, depthCV, model_, this->getNextSeqID(), stamp);
					data.setIMU(imu);

					// Frame rate
					if (this->getImageRate() < 0.0f)
					{
						if (stamp == 0)
						{
							UWARN("The option to use mkv stamps is set (framerate<0), but there are no stamps saved in the file! Aborting...");
						}
						else if (previousStamp_ > 0)
						{
							float ratio = -this->getImageRate();
							int sleepTime = 1000.0*(stamp - previousStamp_) / ratio - 1000.0*timer_.getElapsedTime();
							if (sleepTime > 10000)
							{
								UWARN("Detected long delay (%d sec, stamps = %f vs %f). Waiting a maximum of 10 seconds.",
									sleepTime / 1000, previousStamp_, stamp);
								sleepTime = 10000;
							}
							if (sleepTime > 2)
							{
								uSleep(sleepTime - 2);
							}

							// Add precision at the cost of a small overhead
							while (timer_.getElapsedTime() < (stamp - previousStamp_) / ratio - 0.000001)
							{
								//
							}

							double slept = timer_.getElapsedTime();
							timer_.start();
							UDEBUG("slept=%fs vs target=%fs (ratio=%f)", slept, (stamp - previousStamp_) / ratio, ratio);
						}
						previousStamp_ = stamp;
					}

					break;
				}
			}
		}

		if (result == K4A_STREAM_RESULT_EOF)
		{
			// End of file reached
			UINFO("End of file reached");
		}
		else if (result == K4A_STREAM_RESULT_FAILED)
		{
			UERROR("Failed to read entire recording");
		}
	}
	else
	{
		UERROR("CameraK4A: Live camera stream is not yet supported, only recorded mkv files are.");
	}

#else
	UERROR("CameraK4A: RTAB-Map is not built with Kinect for Azure SDK support!");
#endif
	return data;
}

} // namespace rtabmap
