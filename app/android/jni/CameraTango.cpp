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

#include "CameraTango.h"
#include "util.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/util2d.h"
#include <tango_client_api.h>
#include <tango_support_api.h>

namespace rtabmap {

#define nullptr 0
const int kVersionStringLength = 128;
const int holeSize = 5;
const float maxDepthError = 0.10;
const int scanDownsampling = 1;

// Callbacks
void onPointCloudAvailableRouter(void* context, const TangoPointCloud* point_cloud)
{
	CameraTango* app = static_cast<CameraTango*>(context);
	if(app->isRunning() && point_cloud->num_points>0)
	{
		app->cloudReceived(cv::Mat(1, point_cloud->num_points, CV_32FC4, point_cloud->points[0]), point_cloud->timestamp);
	}
}

void onFrameAvailableRouter(void* context, TangoCameraId id, const TangoImageBuffer* color)
{
	CameraTango* app = static_cast<CameraTango*>(context);
	if(app->isRunning())
	{
		cv::Mat tangoImage;
		if(color->format == TANGO_HAL_PIXEL_FORMAT_RGBA_8888)
		{
			tangoImage = cv::Mat(color->height, color->width, CV_8UC4, color->data);
		}
		else if(color->format == TANGO_HAL_PIXEL_FORMAT_YV12)
		{
			tangoImage = cv::Mat(color->height+color->height/2, color->width, CV_8UC1, color->data);
		}
		else if(color->format == TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP)
		{
			tangoImage = cv::Mat(color->height+color->height/2, color->width, CV_8UC1, color->data);
		}
		else if(color->format == 35)
		{
			tangoImage = cv::Mat(color->height+color->height/2, color->width, CV_8UC1, color->data);
		}
		else
		{
			LOGE("Not supported color format : %d.", color->format);
		}

		if(!tangoImage.empty())
		{
			app->rgbReceived(tangoImage, (unsigned int)color->format, color->timestamp);
		}
	}
}

void onPoseAvailableRouter(void* context, const TangoPoseData* pose)
{
	if(pose->status_code == TANGO_POSE_VALID)
	{
		CameraTango* app = static_cast<CameraTango*>(context);
		app->poseReceived(app->tangoPoseToTransform(pose));
	}
}

void onTangoEventAvailableRouter(void* context, const TangoEvent* event)
{
	CameraTango* app = static_cast<CameraTango*>(context);
	app->tangoEventReceived(event->type, event->event_key, event->event_value);
}

//////////////////////////////
// CameraTango
//////////////////////////////
const float CameraTango::bilateralFilteringSigmaS = 2.0f;
const float CameraTango::bilateralFilteringSigmaR = 0.075f;

CameraTango::CameraTango(bool colorCamera, int decimation, bool publishRawScan, bool smoothing) :
		Camera(0),
		tango_config_(0),
		previousStamp_(0.0),
		stampEpochOffset_(0.0),
		colorCamera_(colorCamera),
		decimation_(decimation),
		rawScanPublished_(publishRawScan),
		smoothing_(smoothing),
		cloudStamp_(0),
		tangoColorType_(0),
		tangoColorStamp_(0),
		colorCameraToDisplayRotation_(ROTATION_0),
		originUpdate_(false)
{
	UASSERT(decimation >= 1);
}

CameraTango::~CameraTango() {
	// Disconnect Tango service
	close();
}

// Compute fisheye distorted coordinates from undistorted coordinates.
// The distortion model used by the Tango fisheye camera is called FOV and is
// described in 'Straight lines have to be straight' by Frederic Devernay and
// Olivier Faugeras. See https://hal.inria.fr/inria-00267247/document.
// Tango ROS Streamer: https://github.com/Intermodalics/tango_ros/blob/master/tango_ros_common/tango_ros_native/src/tango_ros_node.cpp
void applyFovModel(
    double xu, double yu, double w, double w_inverse, double two_tan_w_div_two,
    double* xd, double* yd) {
  double ru = sqrt(xu * xu + yu * yu);
  constexpr double epsilon = 1e-7;
  if (w < epsilon || ru < epsilon) {
    *xd = xu;
    *yd = yu ;
  } else {
    double rd_div_ru = std::atan(ru * two_tan_w_div_two) * w_inverse / ru;
    *xd = xu * rd_div_ru;
    *yd = yu * rd_div_ru;
  }
}
// Compute the warp maps to undistort the Tango fisheye image using the FOV
// model. See OpenCV documentation for more information on warp maps:
// http://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
// Tango ROS Streamer: https://github.com/Intermodalics/tango_ros/blob/master/tango_ros_common/tango_ros_native/src/tango_ros_node.cpp
// @param fisheyeModel the fisheye camera intrinsics.
// @param mapX the output map for the x direction.
// @param mapY the output map for the y direction.
void initFisheyeRectificationMap(
    const CameraModel& fisheyeModel,
    cv::Mat & mapX, cv::Mat & mapY) {
  const double & fx = fisheyeModel.K().at<double>(0,0);
  const double & fy = fisheyeModel.K().at<double>(1,1);
  const double & cx = fisheyeModel.K().at<double>(0,2);
  const double & cy = fisheyeModel.K().at<double>(1,2);
  const double & w = fisheyeModel.D().at<double>(0,0);
  mapX.create(fisheyeModel.imageSize(), CV_32FC1);
  mapY.create(fisheyeModel.imageSize(), CV_32FC1);
  LOGD("initFisheyeRectificationMap: fx=%f fy=%f, cx=%f, cy=%f, w=%f", fx, fy, cx, cy, w);
  // Pre-computed variables for more efficiency.
  const double fy_inverse = 1.0 / fy;
  const double fx_inverse = 1.0 / fx;
  const double w_inverse = 1 / w;
  const double two_tan_w_div_two = 2.0 * std::tan(w * 0.5);
  // Compute warp maps in x and y directions.
  // OpenCV expects maps from dest to src, i.e. from undistorted to distorted
  // pixel coordinates.
  for(int iu = 0; iu < fisheyeModel.imageHeight(); ++iu) {
    for (int ju = 0; ju < fisheyeModel.imageWidth(); ++ju) {
      double xu = (ju - cx) * fx_inverse;
      double yu = (iu - cy) * fy_inverse;
      double xd, yd;
      applyFovModel(xu, yu, w, w_inverse, two_tan_w_div_two, &xd, &yd);
      double jd = cx + xd * fx;
      double id = cy + yd * fy;
      mapX.at<float>(iu, ju) = jd;
      mapY.at<float>(iu, ju) = id;
    }
  }
}

bool CameraTango::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	close();

	TangoSupport_initialize(TangoService_getPoseAtTime, TangoService_getCameraIntrinsics);

	// Connect to Tango
	LOGI("NativeRTABMap: Setup tango config");
	tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
	if (tango_config_ == nullptr)
	{
		LOGE("NativeRTABMap: Failed to get default config form");
		return false;
	}

	// Set auto-recovery for motion tracking as requested by the user.
	bool is_atuo_recovery = true;
	int ret = TangoConfig_setBool(tango_config_, "config_enable_auto_recovery", is_atuo_recovery);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: config_enable_auto_recovery() failed with error code: %d", ret);
		return false;
	}

	if(colorCamera_)
	{
		// Enable color.
		ret = TangoConfig_setBool(tango_config_, "config_enable_color_camera", true);
		if (ret != TANGO_SUCCESS)
		{
			LOGE("NativeRTABMap: config_enable_color_camera() failed with error code: %d", ret);
			return false;
		}
	}

	// Enable depth.
	ret = TangoConfig_setBool(tango_config_, "config_enable_depth", true);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: config_enable_depth() failed with error code: %d", ret);
		return false;
	}

	// Need to specify the depth_mode as XYZC.
	ret = TangoConfig_setInt32(tango_config_, "config_depth_mode", TANGO_POINTCLOUD_XYZC);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("Failed to set 'depth_mode' configuration flag with error code: %d", ret);
		return false;
	}

	// Note that it's super important for AR applications that we enable low
	// latency imu integration so that we have pose information available as
	// quickly as possible. Without setting this flag, you'll often receive
	// invalid poses when calling GetPoseAtTime for an image.
	ret = TangoConfig_setBool(tango_config_, "config_enable_low_latency_imu_integration", true);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to enable low latency imu integration.");
		return false;
	}

	// Drift correction allows motion tracking to recover after it loses tracking.
	//
	// The drift corrected pose is is available through the frame pair with
	// base frame AREA_DESCRIPTION and target frame DEVICE.
	/*ret = TangoConfig_setBool(tango_config_, "config_enable_drift_correction", true);
	if (ret != TANGO_SUCCESS) {
		LOGE(
			"NativeRTABMap: enabling config_enable_drift_correction "
			"failed with error code: %d",
			ret);
		return false;
	}*/

	// Get TangoCore version string from service.
	char tango_core_version[kVersionStringLength];
	ret = TangoConfig_getString(tango_config_, "tango_service_library_version", tango_core_version, kVersionStringLength);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: get tango core version failed with error code: %d", ret);
		return false;
	}
	LOGI("NativeRTABMap: Tango version : %s", tango_core_version);


	// Callbacks
	LOGI("NativeRTABMap: Setup callbacks");
	// Attach the OnXYZijAvailable callback.
	// The callback will be called after the service is connected.
	ret = TangoService_connectOnPointCloudAvailable(onPointCloudAvailableRouter);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to connect to point cloud callback with error code: %d", ret);
		return false;
	}

	ret = TangoService_connectOnFrameAvailable(colorCamera_?TANGO_CAMERA_COLOR:TANGO_CAMERA_FISHEYE, this, onFrameAvailableRouter);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to connect to color callback with error code: %d", ret);
		return false;
	}

	// Attach the onPoseAvailable callback.
	// The callback will be called after the service is connected.
	TangoCoordinateFramePair pair;
	//pair.base = TANGO_COORDINATE_FRAME_AREA_DESCRIPTION; // drift correction is enabled
	pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
	pair.target = TANGO_COORDINATE_FRAME_DEVICE;
	ret = TangoService_connectOnPoseAvailable(1, &pair, onPoseAvailableRouter);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to connect to pose callback with error code: %d", ret);
		return false;
	}

	// Attach the onEventAvailable callback.
	// The callback will be called after the service is connected.
	ret = TangoService_connectOnTangoEvent(onTangoEventAvailableRouter);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("PointCloudApp: Failed to connect to event callback with error code: %d", ret);
		return false;
	}

	// Now connect service so the callbacks above will be called
	LOGI("NativeRTABMap: Connect to tango service");
	ret = TangoService_connect(this, tango_config_);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to connect to the Tango service with error code: %d", ret);
		return false;
	}

	// update extrinsics
	LOGI("NativeRTABMap: Update extrinsics");
	TangoPoseData pose_data;
	TangoCoordinateFramePair frame_pair;

	// TangoService_getPoseAtTime function is used for query device extrinsics
	// as well. We use timestamp 0.0 and the target frame pair to get the
	// extrinsics from the sensors.
	//
	// Get color camera with respect to device transformation matrix.
	frame_pair.base = TANGO_COORDINATE_FRAME_DEVICE;
	frame_pair.target = colorCamera_?TANGO_COORDINATE_FRAME_CAMERA_COLOR:TANGO_COORDINATE_FRAME_CAMERA_FISHEYE;
	ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_data);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to get transform between the color camera frame and device frames");
		return false;
	}
	deviceTColorCamera_ = rtabmap::Transform(
			pose_data.translation[0],
			pose_data.translation[1],
			pose_data.translation[2],
			pose_data.orientation[0],
			pose_data.orientation[1],
			pose_data.orientation[2],
			pose_data.orientation[3]);

	// camera intrinsic
	TangoCameraIntrinsics color_camera_intrinsics;
	ret = TangoService_getCameraIntrinsics(colorCamera_?TANGO_CAMERA_COLOR:TANGO_CAMERA_FISHEYE, &color_camera_intrinsics);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to get the intrinsics for the color camera with error code: %d.", ret);
		return false;
	}

	LOGD("Calibration: fx=%f fy=%f cx=%f cy=%f width=%d height=%d",
			color_camera_intrinsics.fx,
			color_camera_intrinsics.fy,
			color_camera_intrinsics.cx,
			color_camera_intrinsics.cy,
			color_camera_intrinsics.width,
			color_camera_intrinsics.height);

	cv::Mat K = cv::Mat::eye(3, 3, CV_64FC1);
	K.at<double>(0,0) = color_camera_intrinsics.fx;
	K.at<double>(1,1) = color_camera_intrinsics.fy;
	K.at<double>(0,2) = color_camera_intrinsics.cx;
	K.at<double>(1,2) = color_camera_intrinsics.cy;
	cv::Mat D = cv::Mat::zeros(1, 5, CV_64FC1);
	LOGD("Calibration type = %d", color_camera_intrinsics.calibration_type);
	if(color_camera_intrinsics.calibration_type == TANGO_CALIBRATION_POLYNOMIAL_5_PARAMETERS ||
			color_camera_intrinsics.calibration_type == TANGO_CALIBRATION_EQUIDISTANT)
	{
		D.at<double>(0,0) = color_camera_intrinsics.distortion[0];
		D.at<double>(0,1) = color_camera_intrinsics.distortion[1];
		D.at<double>(0,2) = color_camera_intrinsics.distortion[2];
		D.at<double>(0,3) = color_camera_intrinsics.distortion[3];
		D.at<double>(0,4) = color_camera_intrinsics.distortion[4];
	}
	else if(color_camera_intrinsics.calibration_type == TANGO_CALIBRATION_POLYNOMIAL_3_PARAMETERS)
	{
		D.at<double>(0,0) = color_camera_intrinsics.distortion[0];
		D.at<double>(0,1) = color_camera_intrinsics.distortion[1];
		D.at<double>(0,2) = 0.;
		D.at<double>(0,3) = 0.;
		D.at<double>(0,4) = color_camera_intrinsics.distortion[2];
	}
	else if(color_camera_intrinsics.calibration_type == TANGO_CALIBRATION_POLYNOMIAL_2_PARAMETERS)
	{
		D.at<double>(0,0) = color_camera_intrinsics.distortion[0];
		D.at<double>(0,1) = color_camera_intrinsics.distortion[1];
		D.at<double>(0,2) = 0.;
		D.at<double>(0,3) = 0.;
		D.at<double>(0,4) = 0.;
	}

	cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat P;

	LOGD("Distortion params: %f, %f, %f, %f, %f", D.at<double>(0,0), D.at<double>(0,1), D.at<double>(0,2), D.at<double>(0,3), D.at<double>(0,4));
	model_ = CameraModel(colorCamera_?"color":"fisheye",
			cv::Size(color_camera_intrinsics.width, color_camera_intrinsics.height),
			K, D, R, P,
			tango_device_T_rtabmap_device.inverse()*deviceTColorCamera_); // device to camera optical rotation in rtabmap frame

	if(!colorCamera_)
	{
		initFisheyeRectificationMap(model_, fisheyeRectifyMapX_, fisheyeRectifyMapY_);
	}

	LOGI("deviceTColorCameraTango  =%s", deviceTColorCamera_.prettyPrint().c_str());
	LOGI("deviceTColorCameraRtabmap=%s", (tango_device_T_rtabmap_device.inverse()*deviceTColorCamera_).prettyPrint().c_str());

	cameraStartedTime_.restart();

	return true;
}

void CameraTango::close()
{
	if(tango_config_)
	{
		TangoConfig_free(tango_config_);
		tango_config_ = nullptr;
		LOGI("TangoService_disconnect()");
		TangoService_disconnect();
		LOGI("TangoService_disconnect() done.");
	}
	previousPose_.setNull();
	previousStamp_ = 0.0;
	fisheyeRectifyMapX_ = cv::Mat();
	fisheyeRectifyMapY_ = cv::Mat();
	lastKnownGPS_ = GPS();
	lastEnvSensors_.clear();
	originOffset_ = Transform();
	originUpdate_ = false;
}

void CameraTango::resetOrigin()
{
	originUpdate_ = true;
}

void CameraTango::cloudReceived(const cv::Mat & cloud, double timestamp)
{
	if(this->isRunning() && !cloud.empty())
	{
		//LOGD("Depth received! %fs (%d points)", timestamp, cloud.cols);

		UASSERT(cloud.type() == CV_32FC4);
		boost::mutex::scoped_lock  lock(dataMutex_);

		// From post: http://stackoverflow.com/questions/29236110/timing-issues-with-tango-image-frames
		// "In the current version of Project Tango Tablet RGB IR camera
		//  is used for both depth and color images and it can only do one
		//  or the other for each frame. So in the stream we get 4 RGB frames
		//  followed by 1 Depth frame resulting in the pattern you observed. This
		//  is more of a hardware limitation."
		//
		//  So, synchronize with the last RGB frame before the Depth is acquired
		if(!tangoColor_.empty())
		{
			double dt = fabs(timestamp - tangoColorStamp_);

			//LOGD("Depth: %f vs %f = %f", tangoColorStamp_, timestamp, dt);

			if(dt >= 0.0 && dt < 0.5)
			{
				bool notify = cloud_.empty();
				cloud_ = cloud.clone();
				cloudStamp_ = timestamp;
				if(notify)
				{
					//LOGD("Cloud: Release semaphore");
					dataReady_.release();
				}
			}
		}
	}
}

void CameraTango::rgbReceived(const cv::Mat & tangoImage, int type, double timestamp)
{
	if(this->isRunning() && !tangoImage.empty())
	{
		//LOGD("RGB received! %fs", timestamp);

		boost::mutex::scoped_lock  lock(dataMutex_);

		tangoColor_ = tangoImage.clone();
		tangoColorStamp_ = timestamp;
		tangoColorType_ = type;
	}
}

static rtabmap::Transform opticalRotation(
								1.0f,  0.0f,  0.0f, 0.0f,
							    0.0f, -1.0f,  0.0f, 0.0f,
								0.0f,  0.0f, -1.0f, 0.0f);
void CameraTango::poseReceived(const Transform & pose)
{
	if(!pose.isNull())
	{
		// send pose of the camera (without optical rotation), not the device
		Transform p = pose*deviceTColorCamera_*opticalRotation;
		if(originUpdate_)
		{
			originOffset_ = p.translation().inverse();
			originUpdate_ = false;
		}
		if(!originOffset_.isNull())
		{
			this->post(new PoseEvent(originOffset_*p));
		}
		else
		{
			this->post(new PoseEvent(p));
		}
	}
}

void CameraTango::tangoEventReceived(int type, const char * key, const char * value)
{
	this->post(new CameraTangoEvent(type, key, value));
}

bool CameraTango::isCalibrated() const
{
	return model_.isValidForProjection();
}

std::string CameraTango::getSerial() const
{
	return "Tango";
}

void CameraTango::setGPS(const GPS & gps)
{
	lastKnownGPS_ = gps;
}

void CameraTango::addEnvSensor(int type, float value)
{
	lastEnvSensors_.insert(std::make_pair((EnvSensor::Type)type, EnvSensor((EnvSensor::Type)type, value)));
}

rtabmap::Transform CameraTango::tangoPoseToTransform(const TangoPoseData * tangoPose) const
{
	UASSERT(tangoPose);
	rtabmap::Transform pose;

	pose = rtabmap::Transform(
			tangoPose->translation[0],
			tangoPose->translation[1],
			tangoPose->translation[2],
			tangoPose->orientation[0],
			tangoPose->orientation[1],
			tangoPose->orientation[2],
			tangoPose->orientation[3]);

	return pose;
}

rtabmap::Transform CameraTango::getPoseAtTimestamp(double timestamp)
{
	rtabmap::Transform pose;

	TangoPoseData pose_start_service_T_device;
	TangoCoordinateFramePair frame_pair;
	frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
	frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
	TangoErrorType status = TangoService_getPoseAtTime(timestamp, frame_pair, &pose_start_service_T_device);
	if (status != TANGO_SUCCESS)
	{
		LOGE(
			"PoseData: Failed to get transform between the Start of service and "
			"device frames at timestamp %lf",
			timestamp);
	}
	if (pose_start_service_T_device.status_code != TANGO_POSE_VALID)
	{
		LOGW(
			"PoseData: Failed to get transform between the Start of service and "
			"device frames at timestamp %lf",
			timestamp);
	}
	else
	{

		pose = tangoPoseToTransform(&pose_start_service_T_device);
	}

	return pose;
}

SensorData CameraTango::captureImage(CameraInfo * info)
{
	//LOGI("Capturing image...");

	SensorData data;
	if(!dataReady_.acquire(1, 2000))
	{
		if(this->isRunning())
		{
			LOGE("Not received any frames since 2 seconds, try to restart the camera again.");
			this->post(new CameraTangoEvent(0, "CameraTango", "No frames received since 2 seconds."));

			boost::mutex::scoped_lock  lock(dataMutex_);
			if(!cloud_.empty() && !tangoColor_.empty())
			{
				UERROR("cloud and image were set!?");
			}
		}
		cloud_ = cv::Mat();
		cloudStamp_ = 0.0;
		tangoColor_ = cv::Mat();
		tangoColorStamp_ = 0.0;
		tangoColorType_ = 0;
	}
	else
	{
		cv::Mat cloud;
		cv::Mat tangoImage;
		cv::Mat rgb;
		double cloudStamp = 0.0;
		double rgbStamp = 0.0;
		int tangoColorType = 0;

		{
			boost::mutex::scoped_lock  lock(dataMutex_);
			cloud = cloud_;
			cloudStamp = cloudStamp_;
			cloud_ = cv::Mat();
			cloudStamp_ = 0.0;
			tangoImage = tangoColor_;
			rgbStamp = tangoColorStamp_;
			tangoColorType = tangoColorType_;
			tangoColor_ = cv::Mat();
			tangoColorStamp_ = 0.0;
			tangoColorType_ = 0;
		}

		LOGD("tangoColorType=%d", tangoColorType);
		if(tangoColorType == TANGO_HAL_PIXEL_FORMAT_RGBA_8888)
		{
			cv::cvtColor(tangoImage, rgb, CV_RGBA2BGR);
		}
		else if(tangoColorType == TANGO_HAL_PIXEL_FORMAT_YV12)
		{
			cv::cvtColor(tangoImage, rgb, CV_YUV2BGR_YV12);
		}
		else if(tangoColorType == TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP)
		{
			cv::cvtColor(tangoImage, rgb, CV_YUV2BGR_NV21);
		}
		else if(tangoColorType == 35)
		{
			cv::cvtColor(tangoImage, rgb, cv::COLOR_YUV420sp2GRAY);
		}
		else
		{
			LOGE("Not supported color format : %d.", tangoColorType);
			return data;
		}

		//for(int i=0; i<rgb.cols; ++i)
		//{
		//	UERROR("%d,%d,%d", (int)rgb.at<cv::Vec3b>(i)[0], (int)rgb.at<cv::Vec3b>(i)[1], (int)rgb.at<cv::Vec3b>(i)[2]);
		//}

		CameraModel model = model_;

		if(colorCamera_)
		{
			if(decimation_ > 1)
			{
				rgb = util2d::decimate(rgb, decimation_);
				model = model.scaled(1.0/double(decimation_));
			}
		}
		else
		{
			//UTimer t;
			cv::Mat rgbRect;
			cv::remap(rgb, rgbRect, fisheyeRectifyMapX_, fisheyeRectifyMapY_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
			rgb = rgbRect;
			//LOGD("Rectification time=%fs", t.ticks());
		}

		// Querying the depth image's frame transformation based on the depth image's
		// timestamp.
		cv::Mat depth;

		// Calculate the relative pose from color camera frame at timestamp
		// color_timestamp t1 and depth
		// camera frame at depth_timestamp t0.
		Transform colorToDepth;
		TangoPoseData pose_color_image_t1_T_depth_image_t0;
		if (TangoSupport_calculateRelativePose(
				rgbStamp, colorCamera_?TANGO_COORDINATE_FRAME_CAMERA_COLOR:TANGO_COORDINATE_FRAME_CAMERA_FISHEYE, cloudStamp,
			  TANGO_COORDINATE_FRAME_CAMERA_DEPTH,
			  &pose_color_image_t1_T_depth_image_t0) == TANGO_SUCCESS)
		{
			colorToDepth = tangoPoseToTransform(&pose_color_image_t1_T_depth_image_t0);
		}
		else
		{
			LOGE(
				"SynchronizationApplication: Could not find a valid relative pose at "
				"time for color and "
				" depth cameras.");
		}

		if(colorToDepth.getNormSquared() > 100000)
		{
			LOGE("Very large color to depth error detected (%s)! Ignoring this frame!", colorToDepth.prettyPrint().c_str());
			colorToDepth.setNull();
		}
		cv::Mat scan;
		if(!colorToDepth.isNull())
		{
			// The Color Camera frame at timestamp t0 with respect to Depth
			// Camera frame at timestamp t1.
			//LOGD("colorToDepth=%s", colorToDepth.prettyPrint().c_str());
			LOGD("rgb=%dx%d cloud size=%d", rgb.cols, rgb.rows, (int)cloud.total());

			int pixelsSet = 0;
			int depthSizeDec = colorCamera_?8:1;
			depth = cv::Mat::zeros(model_.imageHeight()/depthSizeDec, model_.imageWidth()/depthSizeDec, CV_16UC1); // mm
			CameraModel depthModel = model_.scaled(1.0f/float(depthSizeDec));
			std::vector<cv::Point3f> scanData(rawScanPublished_?cloud.total():0);
			int oi=0;
			int closePoints = 0;
			float closeROI[4];
			closeROI[0] = depth.cols/4;
			closeROI[1] = 3*(depth.cols/4);
			closeROI[2] = depth.rows/4;
			closeROI[3] = 3*(depth.rows/4);
			unsigned short minDepthValue=10000;
			for(unsigned int i=0; i<cloud.total(); ++i)
			{
				float * p = cloud.ptr<float>(0,i);
				cv::Point3f pt = util3d::transformPoint(cv::Point3f(p[0], p[1], p[2]), colorToDepth);

				if(pt.z > 0.0f && i%scanDownsampling == 0 && rawScanPublished_)
				{
					scanData.at(oi++) = pt;
				}

				int pixel_x_l, pixel_y_l, pixel_x_h, pixel_y_h;
				// get the coordinate on image plane.
				pixel_x_l = static_cast<int>((depthModel.fx()) * (pt.x / pt.z) + depthModel.cx());
				pixel_y_l = static_cast<int>((depthModel.fy()) * (pt.y / pt.z) + depthModel.cy());
				pixel_x_h = static_cast<int>((depthModel.fx()) * (pt.x / pt.z) + depthModel.cx() + 0.5f);
				pixel_y_h = static_cast<int>((depthModel.fy()) * (pt.y / pt.z) + depthModel.cy() + 0.5f);
				unsigned short depth_value(pt.z * 1000.0f);

				if(pixel_x_l>=closeROI[0] && pixel_x_l<closeROI[1] &&
				   pixel_y_l>closeROI[2] && pixel_y_l<closeROI[3] &&
				   depth_value < 600)
				{
					++closePoints;
					if(depth_value < minDepthValue)
					{
						minDepthValue = depth_value;
					}
				}

				bool pixelSet = false;
				if(pixel_x_l>=0 && pixel_x_l<depth.cols &&
				   pixel_y_l>0 && pixel_y_l<depth.rows && // ignore first line
				   depth_value)
				{
					unsigned short & depthPixel = depth.at<unsigned short>(pixel_y_l, pixel_x_l);
					if(depthPixel == 0 || depthPixel > depth_value)
					{
						depthPixel = depth_value;
						pixelSet = true;
					}
				}
				if(pixel_x_h>=0 && pixel_x_h<depth.cols &&
				   pixel_y_h>0 && pixel_y_h<depth.rows && // ignore first line
				   depth_value)
				{
					unsigned short & depthPixel = depth.at<unsigned short>(pixel_y_h, pixel_x_h);
					if(depthPixel == 0 || depthPixel > depth_value)
					{
						depthPixel = depth_value;
						pixelSet = true;
					}
				}
				if(pixelSet)
				{
					pixelsSet += 1;
				}
			}

			if(closePoints > 100)
			{
				this->post(new CameraTangoEvent(0, "TooClose", ""));
			}

			if(oi)
			{
				scan = cv::Mat(1, oi, CV_32FC3, scanData.data()).clone();
			}
			//LOGD("pixels depth set= %d", pixelsSet);
		}
		else
		{
			LOGE("color to depth pose is null?!? (rgb stamp=%f) (depth stamp=%f)", rgbStamp, cloudStamp);
		}

		if(!rgb.empty() && !depth.empty())
		{
			depth = rtabmap::util2d::fillDepthHoles(depth, holeSize, maxDepthError);

			Transform poseDevice = getPoseAtTimestamp(rgbStamp);

			// adjust origin
			if(!originOffset_.isNull())
			{
				poseDevice = originOffset_ * poseDevice;
			}

			//LOGD("Local    = %s", model.localTransform().prettyPrint().c_str());
			//LOGD("tango    = %s", poseDevice.prettyPrint().c_str());
			//LOGD("opengl(t)= %s", (opengl_world_T_tango_world * poseDevice).prettyPrint().c_str());

			//Rotate in RTAB-Map's coordinate
			Transform odom = rtabmap_world_T_tango_world * poseDevice * tango_device_T_rtabmap_device;

			//LOGD("rtabmap  = %s", odom.prettyPrint().c_str());
			//LOGD("opengl(r)= %s", (opengl_world_T_rtabmap_world * odom * rtabmap_device_T_opengl_device).prettyPrint().c_str());

			Transform scanLocalTransform = model.localTransform();

			// Rotate image depending on the camera orientation
			if(colorCameraToDisplayRotation_ == ROTATION_90)
			{
				cv::Mat rgbt(rgb.cols, rgb.rows, rgb.type());
				cv::flip(rgb,rgb,1);
				cv::transpose(rgb,rgbt);
				rgb = rgbt;
				cv::Mat deptht(depth.cols, depth.rows, depth.type());
				cv::flip(depth,depth,1);
				cv::transpose(depth,deptht);
				depth = deptht;
				cv::Size sizet(model.imageHeight(), model.imageWidth());
				model = CameraModel(model.fy(), model.fx(), model.cy(), model.cx()>0?model.imageWidth()-model.cx():0, model.localTransform()*rtabmap::Transform(0,0,0,0,0,1.57079632679489661923132169163975144));
				model.setImageSize(sizet);
			}
			else if(colorCameraToDisplayRotation_ == ROTATION_180)
			{
				cv::flip(rgb,rgb,1);
				cv::flip(rgb,rgb,0);
				cv::flip(depth,depth,1);
				cv::flip(depth,depth,0);
				cv::Size sizet(model.imageWidth(), model.imageHeight());
				model = CameraModel(
						model.fx(),
						model.fy(),
						model.cx()>0?model.imageWidth()-model.cx():0,
						model.cy()>0?model.imageHeight()-model.cy():0,
						model.localTransform()*rtabmap::Transform(0,0,0,0,0,1.57079632679489661923132169163975144*2.0));
				model.setImageSize(sizet);
			}
			else if(colorCameraToDisplayRotation_ == ROTATION_270)
			{
				cv::Mat rgbt(rgb.cols, rgb.rows, rgb.type());
				cv::transpose(rgb,rgbt);
				cv::flip(rgbt,rgbt,1);
				rgb = rgbt;
				cv::Mat deptht(depth.cols, depth.rows, depth.type());
				cv::transpose(depth,deptht);
				cv::flip(deptht,deptht,1);
				depth = deptht;
				cv::Size sizet(model.imageHeight(), model.imageWidth());
				model = CameraModel(model.fy(), model.fx(), model.cy()>0?model.imageHeight()-model.cy():0, model.cx(), model.localTransform()*rtabmap::Transform(0,0,0,0,0,-1.57079632679489661923132169163975144));
				model.setImageSize(sizet);
			}

			if(smoothing_)
			{
				//UTimer t;
				depth = rtabmap::util2d::fastBilateralFiltering(depth, bilateralFilteringSigmaS, bilateralFilteringSigmaR);
				data.setDepthOrRightRaw(depth);
				//LOGD("Bilateral filtering, time=%fs", t.ticks());
			}

			if(rawScanPublished_)
			{
				data = SensorData(LaserScan::backwardCompatibility(scan, cloud.total()/scanDownsampling, 0, scanLocalTransform), rgb, depth, model, this->getNextSeqID(), rgbStamp);
			}
			else
			{
				data = SensorData(rgb, depth, model, this->getNextSeqID(), rgbStamp);
			}
			data.setGroundTruth(odom);

			if(lastKnownGPS_.stamp() > 0.0 && rgbStamp-lastKnownGPS_.stamp()<1.0)
			{
				data.setGPS(lastKnownGPS_);
			}
			else if(lastKnownGPS_.stamp()>0.0)
			{
				LOGD("GPS too old (current time=%f, gps time = %f)", rgbStamp, lastKnownGPS_.stamp());
			}

			if(lastEnvSensors_.size())
			{
				data.setEnvSensors(lastEnvSensors_);
				lastEnvSensors_.clear();
			}
		}
		else
		{
			LOGE("Could not get depth and rgb images!?!");
		}
	}
	return data;

}

void CameraTango::mainLoopBegin()
{
	double t = cameraStartedTime_.elapsed();
	if(t < 5.0)
	{
		uSleep((5.0-t)*1000); // just to make sure that the camera is started
	}
}

void CameraTango::mainLoop()
{
	if(tango_config_)
	{
		SensorData data = this->captureImage();

		if(!data.groundTruth().isNull())
		{
			rtabmap::Transform pose = data.groundTruth();
			data.setGroundTruth(Transform());

			// convert stamp to epoch
			bool firstFrame = previousPose_.isNull();
			if(firstFrame)
			{
				stampEpochOffset_ = UTimer::now()-data.stamp();
			}
			data.setStamp(stampEpochOffset_ + data.stamp());
			OdometryInfo info;
			if(!firstFrame)
			{
				info.interval = data.stamp()-previousStamp_;
				info.transform = previousPose_.inverse() * pose;
			}
			// linear cov = 0.0001
			info.reg.covariance = cv::Mat::eye(6,6,CV_64FC1) * (firstFrame?9999.0:0.0001);
			if(!firstFrame)
			{
				// angular cov = 0.000001
				info.reg.covariance.at<double>(3,3) *= 0.01;
				info.reg.covariance.at<double>(4,4) *= 0.01;
				info.reg.covariance.at<double>(5,5) *= 0.01;
			}
			LOGI("Publish odometry message (variance=%f)", firstFrame?9999:0.0001);
			this->post(new OdometryEvent(data, pose, info));
			previousPose_ = pose;
			previousStamp_ = data.stamp();
		}
		else if(!this->isKilled())
		{
			LOGW("Odometry lost");
			this->post(new OdometryEvent());
		}
	}
	else
	{
		UERROR("Camera not initialized, cannot start thread.");
		this->kill();
	}
}

} /* namespace rtabmap */
