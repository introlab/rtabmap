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
			,
			deviceHandle_(NULL),
			config_(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL),
			transformationHandle_(NULL),
			captureHandle_(NULL),
			playbackHandle_(NULL),
			deviceId_(deviceId),
			rgb_resolution_(0),
			framerate_(2),
			depth_resolution_(2),
			ir_(false),
			previousStamp_(0.0),
			timestampOffset_(0.0)
#endif
{
}

CameraK4A::CameraK4A(
		const std::string & fileName,
		float imageRate,
		const Transform & localTransform) :
			Camera(imageRate, localTransform)
#ifdef RTABMAP_K4A
			,
			deviceHandle_(NULL),
			transformationHandle_(NULL),
			captureHandle_(NULL),
			playbackHandle_(NULL),
			deviceId_(-1),
			fileName_(fileName),
			rgb_resolution_(0),
			framerate_(2),
			depth_resolution_(2),
			ir_(false),
			previousStamp_(0.0),
			timestampOffset_(0.0)
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
		playbackHandle_ = NULL;
	}
	else if (deviceHandle_ != NULL)
	{
		k4a_device_stop_imu(deviceHandle_);

		k4a_device_stop_cameras(deviceHandle_);
		k4a_device_close(deviceHandle_);
		deviceHandle_ = NULL;
		config_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	}

	if (transformationHandle_ != NULL)
	{
		k4a_transformation_destroy((k4a_transformation_t)transformationHandle_);
		transformationHandle_ = NULL;
	}
	previousStamp_ = 0.0;
	timestampOffset_ = 0.0;
#endif
}

void CameraK4A::setPreferences(int rgb_resolution, int framerate, int depth_resolution)
{
#ifdef RTABMAP_K4A
	rgb_resolution_ = rgb_resolution;
	framerate_ = framerate;
	depth_resolution_ = depth_resolution;
	UINFO("setPreferences(): %i %i %i", rgb_resolution, framerate, depth_resolution);
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
	
	if (!fileName_.empty())
	{
		close();

		if (k4a_playback_open(fileName_.c_str(), (k4a_playback_t*)&playbackHandle_) != K4A_RESULT_SUCCEEDED)
		{
			UERROR("Failed to open recording \"%s\"", fileName_.c_str());
			return false;
		}

		uint64_t recording_length = k4a_playback_get_last_timestamp_usec((k4a_playback_t)playbackHandle_);
		UINFO("Recording is %lld seconds long", recording_length / 1000000);

		if (k4a_playback_get_calibration((k4a_playback_t)playbackHandle_, &calibration_))
		{
			UERROR("Failed to get calibration");
			close();
			return false;
		}

	}
	else if (deviceId_ >= 0)
	{
		if(deviceHandle_!=NULL)
		{
			this->close();
		}

		switch(rgb_resolution_)
		{
		case 0: config_.color_resolution = K4A_COLOR_RESOLUTION_720P; break;
		case 1: config_.color_resolution = K4A_COLOR_RESOLUTION_1080P; break;
		case 2: config_.color_resolution = K4A_COLOR_RESOLUTION_1440P; break;
		case 3: config_.color_resolution = K4A_COLOR_RESOLUTION_1536P; break;
		case 4: config_.color_resolution = K4A_COLOR_RESOLUTION_2160P; break;
		case 5:
		default: config_.color_resolution = K4A_COLOR_RESOLUTION_3072P; break;
		}

		switch(framerate_)
		{
		case 0: config_.camera_fps = K4A_FRAMES_PER_SECOND_5; break;
		case 1: config_.camera_fps = K4A_FRAMES_PER_SECOND_15; break;
		case 2:
		default: config_.camera_fps = K4A_FRAMES_PER_SECOND_30; break;
		}

		switch(depth_resolution_)
		{
		case 0: config_.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED; break;
		case 1: config_.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; break;
		case 2: config_.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED; break;
		case 3:
		default: config_.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED; break;
		}

		// This is fixed for now
		config_.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;

		int device_count = k4a_device_get_installed_count();

		if (device_count == 0)
		{
			UERROR("No k4a devices attached!");
			return false;
		}
		else if(deviceId_ > device_count)
		{
			UERROR("Cannot select device %d, only %d devices detected.", deviceId_, device_count);
		}

		UINFO("CameraK4A found %d k4a device(s) attached", device_count);

		// Open the first plugged in Kinect device
		if (K4A_FAILED(k4a_device_open(deviceId_, &deviceHandle_)))
		{
			UERROR("Failed to open k4a device!");
			return false;
		}

		// Get the size of the serial number
		size_t serial_size = 0;
		k4a_device_get_serialnum(deviceHandle_, NULL, &serial_size);

		// Allocate memory for the serial, then acquire it
		char *serial = (char*)(malloc(serial_size));
		k4a_device_get_serialnum(deviceHandle_, serial, &serial_size);
		serial_number_.assign(serial, serial_size);
		free(serial);

		UINFO("Opened K4A device: %s", serial_number_.c_str());

		// Start the camera with the given configuration
		if (K4A_FAILED(k4a_device_start_cameras(deviceHandle_, &config_)))
		{
			UERROR("Failed to start cameras!");
			close();
			return false;
		}

		UINFO("K4A camera started successfully");

		if (K4A_FAILED(k4a_device_get_calibration(deviceHandle_, config_.depth_mode, config_.color_resolution, &calibration_)))
		{
			UERROR("k4a_device_get_calibration() failed!");
			close();
			return false;
		}
	}
	else
	{
		UERROR("k4a_device_get_calibration() no file and no valid device id!");
		return false;
	}

	if (ir_)
	{
		cv::Mat K = cv::Mat::eye(3, 3, CV_64FC1);
		K.at<double>(0,0) = calibration_.depth_camera_calibration.intrinsics.parameters.param.fx;
		K.at<double>(1,1) = calibration_.depth_camera_calibration.intrinsics.parameters.param.fy;
		K.at<double>(0,2) = calibration_.depth_camera_calibration.intrinsics.parameters.param.cx;
		K.at<double>(1,2) = calibration_.depth_camera_calibration.intrinsics.parameters.param.cy;
		cv::Mat D = cv::Mat::eye(1, 8, CV_64FC1);
		D.at<double>(0,0) = calibration_.depth_camera_calibration.intrinsics.parameters.param.k1;
		D.at<double>(0,1) = calibration_.depth_camera_calibration.intrinsics.parameters.param.k2;
		D.at<double>(0,2) = calibration_.depth_camera_calibration.intrinsics.parameters.param.p1;
		D.at<double>(0,3) = calibration_.depth_camera_calibration.intrinsics.parameters.param.p2;
		D.at<double>(0,4) = calibration_.depth_camera_calibration.intrinsics.parameters.param.k3;
		D.at<double>(0,5) = calibration_.depth_camera_calibration.intrinsics.parameters.param.k4;
		D.at<double>(0,6) = calibration_.depth_camera_calibration.intrinsics.parameters.param.k5;
		D.at<double>(0,7) = calibration_.depth_camera_calibration.intrinsics.parameters.param.k6;
		cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
		cv::Mat P = cv::Mat::eye(3, 4, CV_64FC1);
		P.at<double>(0,0) = K.at<double>(0,0);
		P.at<double>(1,1) = K.at<double>(1,1);
		P.at<double>(0,2) = K.at<double>(0,2);
		P.at<double>(1,2) = K.at<double>(1,2);
		model_ = CameraModel(
				"k4a_ir",
				cv::Size(calibration_.depth_camera_calibration.resolution_width, calibration_.depth_camera_calibration.resolution_height),
				K,D,R,P,
				this->getLocalTransform());
	}
	else
	{
		cv::Mat K = cv::Mat::eye(3, 3, CV_64FC1);
		K.at<double>(0,0) = calibration_.color_camera_calibration.intrinsics.parameters.param.fx;
		K.at<double>(1,1) = calibration_.color_camera_calibration.intrinsics.parameters.param.fy;
		K.at<double>(0,2) = calibration_.color_camera_calibration.intrinsics.parameters.param.cx;
		K.at<double>(1,2) = calibration_.color_camera_calibration.intrinsics.parameters.param.cy;
		cv::Mat D = cv::Mat::eye(1, 8, CV_64FC1);
		D.at<double>(0,0) = calibration_.color_camera_calibration.intrinsics.parameters.param.k1;
		D.at<double>(0,1) = calibration_.color_camera_calibration.intrinsics.parameters.param.k2;
		D.at<double>(0,2) = calibration_.color_camera_calibration.intrinsics.parameters.param.p1;
		D.at<double>(0,3) = calibration_.color_camera_calibration.intrinsics.parameters.param.p2;
		D.at<double>(0,4) = calibration_.color_camera_calibration.intrinsics.parameters.param.k3;
		D.at<double>(0,5) = calibration_.color_camera_calibration.intrinsics.parameters.param.k4;
		D.at<double>(0,6) = calibration_.color_camera_calibration.intrinsics.parameters.param.k5;
		D.at<double>(0,7) = calibration_.color_camera_calibration.intrinsics.parameters.param.k6;
		cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
		cv::Mat P = cv::Mat::eye(3, 4, CV_64FC1);
		P.at<double>(0,0) = K.at<double>(0,0);
		P.at<double>(1,1) = K.at<double>(1,1);
		P.at<double>(0,2) = K.at<double>(0,2);
		P.at<double>(1,2) = K.at<double>(1,2);
		model_ = CameraModel(
				"k4a_color",
				cv::Size(calibration_.color_camera_calibration.resolution_width, calibration_.color_camera_calibration.resolution_height),
				K,D,R,P,
				this->getLocalTransform());
	}
	UASSERT(model_.isValidForRectification());
	model_.initRectificationMap();

	if (ULogger::level() <= ULogger::kInfo)
	{
		UINFO("K4A calibration:");
		std::cout << model_ << std::endl;
	}

	transformationHandle_ = k4a_transformation_create(&calibration_);

	// Get imu transform
	k4a_calibration_extrinsics_t* imu_extrinsics;
	if(ir_)
	{
		imu_extrinsics = &calibration_.extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_DEPTH];
	}
	else
	{
		imu_extrinsics = &calibration_.extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_COLOR];
	}
	imuLocalTransform_ = Transform(
		imu_extrinsics->rotation[0], imu_extrinsics->rotation[1], imu_extrinsics->rotation[2], imu_extrinsics->translation[0] / 1000.0f,
		imu_extrinsics->rotation[3], imu_extrinsics->rotation[4], imu_extrinsics->rotation[5], imu_extrinsics->translation[1] / 1000.0f,
		imu_extrinsics->rotation[6], imu_extrinsics->rotation[7], imu_extrinsics->rotation[8], imu_extrinsics->translation[2] / 1000.0f);

	UINFO("camera to imu=%s", imuLocalTransform_.prettyPrint().c_str());
	UINFO("base to camera=%s", this->getLocalTransform().prettyPrint().c_str());
	imuLocalTransform_ = this->getLocalTransform()*imuLocalTransform_;
	UINFO("base to imu=%s", imuLocalTransform_.prettyPrint().c_str());


	// Start playback or camera
	if (!fileName_.empty())
	{
		k4a_record_configuration_t config;
		if (k4a_playback_get_record_configuration((k4a_playback_t)playbackHandle_, &config))
		{
			UERROR("Failed to getting recording configuration");
			close();
			return false;
		}
	}
	else
	{
		if (K4A_FAILED(k4a_device_start_imu(deviceHandle_)))
		{
			UERROR("Failed to start K4A IMU");
			close();
			return false;
		}

		UINFO("K4a IMU started successfully");

		// Get an initial capture to put the camera in the right state
		if (K4A_WAIT_RESULT_SUCCEEDED == k4a_device_get_capture(deviceHandle_, &captureHandle_, K4A_WAIT_INFINITE))
		{
			k4a_capture_release(captureHandle_);
			return true;
		}

		close();
		return false;
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
	if(!fileName_.empty())
	{
		return fileName_;
	}
	return(serial_number_);
#else
	return "";
#endif
}

SensorData CameraK4A::captureImage(CameraInfo * info)
{
	SensorData data;

#ifdef RTABMAP_K4A

	k4a_image_t ir_image_ = NULL;
	k4a_image_t rgb_image_ = NULL;
	k4a_imu_sample_t imu_sample_;

	double t = UTimer::now();

	bool captured = false;
	if(playbackHandle_)
	{
		k4a_stream_result_t result = K4A_STREAM_RESULT_FAILED;
		while((UTimer::now()-t < 0.1) &&
				(K4A_STREAM_RESULT_SUCCEEDED != (result=k4a_playback_get_next_capture(playbackHandle_, &captureHandle_)) ||
				((ir_ && (ir_image_=k4a_capture_get_ir_image(captureHandle_)) == NULL) || (!ir_ && (rgb_image_=k4a_capture_get_color_image(captureHandle_)) == NULL))))
		{
			k4a_capture_release(captureHandle_);
			// the first frame may be null, just retry for 1 second
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
		captured = result == K4A_STREAM_RESULT_SUCCEEDED;
	}
	else // device
	{
		k4a_wait_result_t result = K4A_WAIT_RESULT_FAILED;
		while((UTimer::now()-t < 5.0) &&
				(K4A_WAIT_RESULT_SUCCEEDED != (result=k4a_device_get_capture(deviceHandle_, &captureHandle_, K4A_WAIT_INFINITE)) ||
				((ir_ && (ir_image_=k4a_capture_get_ir_image(captureHandle_)) == NULL) || (!ir_ && (rgb_image_=k4a_capture_get_color_image(captureHandle_)) == NULL))))
		{
			k4a_capture_release(captureHandle_);
			// the first frame may be null, just retry for 5 seconds
		}
		captured = result == K4A_WAIT_RESULT_SUCCEEDED;
	}

	if (captured && (rgb_image_!=NULL || ir_image_!=NULL))
	{
		cv::Mat bgrCV;
		cv::Mat depthCV;
		IMU imu;

		if (ir_image_ != NULL)
		{
			// Convert IR image
			cv::Mat bgrCV16(k4a_image_get_height_pixels(ir_image_),
					k4a_image_get_width_pixels(ir_image_),
					CV_16UC1,
					(void*)k4a_image_get_buffer(ir_image_));

			bgrCV16.convertTo(bgrCV, CV_8U);

			bgrCV = model_.rectifyImage(bgrCV);

			// Release the image
			k4a_image_release(ir_image_);
		}
		else
		{
			// Convert RGB image
			if (k4a_image_get_format(rgb_image_) == K4A_IMAGE_FORMAT_COLOR_MJPG)
			{
				bgrCV = uncompressImage(cv::Mat(1, (int)k4a_image_get_size(rgb_image_),
						CV_8UC1,
						(void*)k4a_image_get_buffer(rgb_image_)));
			}
			else
			{
				cv::Mat bgra(k4a_image_get_height_pixels(rgb_image_),
						k4a_image_get_width_pixels(rgb_image_),
						CV_8UC4,
						(void*)k4a_image_get_buffer(rgb_image_));

				cv::cvtColor(bgra, bgrCV, CV_BGRA2BGR);
			}
			bgrCV = model_.rectifyImage(bgrCV);

			// Release the image
			k4a_image_release(rgb_image_);
		}

		double stamp = UTimer::now();
		if(!bgrCV.empty())
		{
			// Retrieve depth image from capture
			k4a_image_t depth_image_ = k4a_capture_get_depth_image(captureHandle_);

			if (depth_image_ != NULL)
			{
				double stampDevice = ((double)k4a_image_get_device_timestamp_usec(depth_image_)) / 1000000.0;

				if(timestampOffset_ == 0.0)
				{
					timestampOffset_ = stamp - stampDevice;
				}
				if(!playbackHandle_)
					stamp = stampDevice + timestampOffset_;

				if (ir_)
				{
					depthCV = cv::Mat(k4a_image_get_height_pixels(depth_image_),
							k4a_image_get_width_pixels(depth_image_),
							CV_16UC1,
							(void*)k4a_image_get_buffer(depth_image_));

					depthCV = model_.rectifyDepth(depthCV);
				}
				else
				{
					k4a_image_t transformedDepth = NULL;
					if (k4a_image_create(k4a_image_get_format(depth_image_),
							bgrCV.cols, bgrCV.rows, bgrCV.cols * 2, &transformedDepth) == K4A_RESULT_SUCCEEDED)
					{
						if(k4a_transformation_depth_image_to_color_camera(transformationHandle_, depth_image_, transformedDepth) == K4A_RESULT_SUCCEEDED)
						{
							depthCV = cv::Mat(k4a_image_get_height_pixels(transformedDepth),
									k4a_image_get_width_pixels(transformedDepth),
									CV_16UC1,
									(void*)k4a_image_get_buffer(transformedDepth)).clone();
						}
						else
						{
							UERROR("K4A failed to register depth image");
						}

						k4a_image_release(transformedDepth);
					}
					else
					{
						UERROR("K4A failed to allocate registered depth image");
					}
				}
				k4a_image_release(depth_image_);
			}
		}

		k4a_capture_release(captureHandle_);

		if(playbackHandle_)
		{
			// Get IMU sample, clear buffer
			// FIXME: not tested, uncomment when tested.
			k4a_playback_seek_timestamp(playbackHandle_, stamp* 1000000+1, K4A_PLAYBACK_SEEK_BEGIN);
			if(K4A_STREAM_RESULT_SUCCEEDED == k4a_playback_get_previous_imu_sample(playbackHandle_, &imu_sample_))
			{
				imu = IMU(cv::Vec3d(imu_sample_.gyro_sample.xyz.x, imu_sample_.gyro_sample.xyz.y, imu_sample_.gyro_sample.xyz.z),
						cv::Mat::eye(3, 3, CV_64FC1),
						cv::Vec3d(imu_sample_.acc_sample.xyz.x, imu_sample_.acc_sample.xyz.y, imu_sample_.acc_sample.xyz.z),
						cv::Mat::eye(3, 3, CV_64FC1),
						imuLocalTransform_);
			}
			else
			{
				UWARN("IMU data NULL");
			}
		}
		else
		{
			// Get IMU sample, clear buffer
			if(K4A_WAIT_RESULT_SUCCEEDED == k4a_device_get_imu_sample(deviceHandle_, &imu_sample_, 60))
			{
				imu = IMU(cv::Vec3d(imu_sample_.gyro_sample.xyz.x, imu_sample_.gyro_sample.xyz.y, imu_sample_.gyro_sample.xyz.z),
						cv::Mat::eye(3, 3, CV_64FC1),
						cv::Vec3d(imu_sample_.acc_sample.xyz.x, imu_sample_.acc_sample.xyz.y, imu_sample_.acc_sample.xyz.z),
						cv::Mat::eye(3, 3, CV_64FC1),
						imuLocalTransform_);
			}
			else
			{
				UERROR("IMU data NULL");
			}
		}

		// Relay the data to rtabmap
		if (!bgrCV.empty() && !depthCV.empty())
		{
			data = SensorData(bgrCV, depthCV, model_, this->getNextSeqID(), stamp);
			if(!imu.empty())
			{
				data.setIMU(imu);
			}

			// Frame rate
			if (playbackHandle_ && this->getImageRate() < 0.0f)
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
		}
	}
#else
	UERROR("CameraK4A: RTAB-Map is not built with Kinect for Azure SDK support!");
#endif
	return data;
}

} // namespace rtabmap
