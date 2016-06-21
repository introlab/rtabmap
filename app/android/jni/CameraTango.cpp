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
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/OdometryEvent.h"
#include <tango_client_api.h>

namespace rtabmap {

#define nullptr 0
const int kVersionStringLength = 128;
const int holeSize = 10;
const float maxDepthError = 0.10;

// Callbacks
void onPointCloudAvailableRouter(void* context, const TangoXYZij* xyz_ij)
{
	CameraTango* app = static_cast<CameraTango*>(context);
	app->cloudReceived(cv::Mat(1, xyz_ij->xyz_count, CV_32FC3, xyz_ij->xyz[0]), xyz_ij->timestamp);
}

void onFrameAvailableRouter(void* context, TangoCameraId id, const TangoImageBuffer* color)
{
	CameraTango* app = static_cast<CameraTango*>(context);
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
	else
	{
		LOGE("Not supported color format : %d.", color->format);
	}

	if(!tangoImage.empty())
	{
		app->rgbReceived(tangoImage, (unsigned int)color->format, color->timestamp);
	}
}

void onPoseAvailableRouter(void* context, const TangoPoseData* pose)
{
	CameraTango* app = static_cast<CameraTango*>(context);
	app->poseReceived(app->tangoPoseToTransform(pose, true));
}

void onTangoEventAvailableRouter(void* context, const TangoEvent* event)
{
	CameraTango* app = static_cast<CameraTango*>(context);
	app->tangoEventReceived(event->type, event->event_key, event->event_value);
}

// In OpenGL, axes are x->right, y->up and z->outScreen
// Image is x->right, y->down and z->inScreen
static rtabmap::Transform opticalRotation(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, -1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, -1.0f, 0.0f);

//////////////////////////////
// CameraTango
//////////////////////////////
CameraTango::CameraTango(int decimation, bool autoExposure) :
		Camera(0, opticalRotation),
		tango_config_(0),
		firstFrame_(true),
		decimation_(decimation),
		autoExposure_(autoExposure),
		cloudStamp_(0),
		tangoColorType_(0),
		tangoColorStamp_(0)
{
	UASSERT(decimation >= 1);
}

CameraTango::~CameraTango() {
	// Disconnect Tango service
	close();
}

bool CameraTango::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	close();

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

	// Enable color.
	ret = TangoConfig_setBool(tango_config_, "config_enable_color_camera", true);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: config_enable_color_camera() failed with error code: %d", ret);
		return false;
	}

	// disable auto exposure (disabled, seems broken on latest Tango releases)
	ret = TangoConfig_setBool(tango_config_, "config_color_mode_auto", autoExposure_);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: config_color_mode_auto() failed with error code: %d", ret);
		//return false;
	}
	else
	{
		if(!autoExposure_)
		{
			ret = TangoConfig_setInt32(tango_config_, "config_color_iso", 800);
			if (ret != TANGO_SUCCESS)
			{
				LOGE("NativeRTABMap: config_color_iso() failed with error code: %d", ret);
				return false;
			}
		}
		bool verifyAutoExposureState;
		int32_t verifyIso, verifyExp;
		TangoConfig_getBool( tango_config_, "config_color_mode_auto", &verifyAutoExposureState );
		TangoConfig_getInt32( tango_config_, "config_color_iso", &verifyIso );
		TangoConfig_getInt32( tango_config_, "config_color_exp", &verifyExp );
		LOGI( "NativeRTABMap: config_color autoExposure=%s %d %d", verifyAutoExposureState?"On" : "Off", verifyIso, verifyExp );
	}

	// Enable depth.
	ret = TangoConfig_setBool(tango_config_, "config_enable_depth", true);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: config_enable_depth() failed with error code: %d", ret);
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
	ret = TangoService_connectOnXYZijAvailable(onPointCloudAvailableRouter);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to connect to point cloud callback with error code: %d", ret);
		return false;
	}

	ret = TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, this, onFrameAvailableRouter);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to connect to color callback with error code: %d", ret);
		return false;
	}

	// Attach the onPoseAvailable callback.
	// The callback will be called after the service is connected.
	TangoCoordinateFramePair pair;
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
	// Get device with respect to imu transformation matrix.
	frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
	frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
	ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_data);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to get transform between the IMU frame and device frames");
		return false;
	}
	imuTDevice_ = rtabmap::Transform(
			pose_data.translation[0],
			pose_data.translation[1],
			pose_data.translation[2],
			pose_data.orientation[0],
			pose_data.orientation[1],
			pose_data.orientation[2],
			pose_data.orientation[3]);

	// Get color camera with respect to imu transformation matrix.
	frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
	frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
	ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_data);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to get transform between the color camera frame and device frames");
		return false;
	}
	imuTDepthCamera_ = rtabmap::Transform(
			pose_data.translation[0],
			pose_data.translation[1],
			pose_data.translation[2],
			pose_data.orientation[0],
			pose_data.orientation[1],
			pose_data.orientation[2],
			pose_data.orientation[3]);

	deviceTDepth_ = imuTDevice_.inverse() * imuTDepthCamera_;

	// camera intrinsic
	TangoCameraIntrinsics color_camera_intrinsics;
	ret = TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &color_camera_intrinsics);
	if (ret != TANGO_SUCCESS)
	{
		LOGE("NativeRTABMap: Failed to get the intrinsics for the color camera with error code: %d.", ret);
		return false;
	}
	model_ = CameraModel(
			color_camera_intrinsics.fx,
			color_camera_intrinsics.fy,
			color_camera_intrinsics.cx,
			color_camera_intrinsics.cy,
			this->getLocalTransform());
	model_.setImageSize(cv::Size(color_camera_intrinsics.width, color_camera_intrinsics.height));

	// optical rotation
	model_.setLocalTransform(Transform(
			0.0f, 0.0f, 1.0f, 0.0f,
			-1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, -1.0f, 0.0f, 0.0f));

	return true;
}

void CameraTango::close()
{
	if(tango_config_)
	{
		TangoConfig_free(tango_config_);
		tango_config_ = nullptr;
		TangoService_disconnect();
	}
	firstFrame_ = true;
}

void CameraTango::cloudReceived(const cv::Mat & cloud, double timestamp)
{
	if(this->isRunning())
	{
		UASSERT(cloud.type() == CV_32FC3);
		boost::mutex::scoped_lock  lock(dataMutex_);

		bool notify = cloud_.empty();
		cloud_ = cloud.clone();
		cloudStamp_ = timestamp;
		LOGD("Depth received! (%d points)", cloud.cols);
		if(!tangoColor_.empty() && notify)
		{
			LOGD("Cloud: Release semaphore");
			dataReady_.release();
		}
	}
}

void CameraTango::rgbReceived(const cv::Mat & tangoImage, int type, double timestamp)
{
	if(this->isRunning())
	{
		boost::mutex::scoped_lock  lock(dataMutex_);

		if(!cloud_.empty())
		{
			if(!tangoImage.empty())
			{
				bool notify = tangoColor_.empty();
				tangoColor_ = tangoImage.clone();
				tangoColorStamp_ = timestamp;
				tangoColorType_ = type;
				LOGD("RGB received!");
				if(!cloud_.empty() && notify)
				{
					LOGD("RGB: Release semaphore");
					dataReady_.release();
				}
			}
		}
	}
}

void CameraTango::poseReceived(const Transform & pose)
{
	if(!pose.isNull() && pose.getNormSquared() < 100000)
	{
		this->post(new PoseEvent(pose));
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

rtabmap::Transform CameraTango::tangoPoseToTransform(const TangoPoseData * tangoPose, bool inOpenGLFrame) const
{
	UASSERT(tangoPose);
	rtabmap::Transform pose;
	if(!deviceTDepth_.isNull())
	{
		pose = rtabmap::Transform(
				tangoPose->translation[0],
				tangoPose->translation[1],
				tangoPose->translation[2],
				tangoPose->orientation[0],
				tangoPose->orientation[1],
				tangoPose->orientation[2],
				tangoPose->orientation[3]);

		// transform in OpenGL + extrinsics
		// opengl_world_T_opengl_camera =
		//      opengl_world_T_start_service *
		//      start_service_T_device *
		//      device_T_imu *
		//      imu_T_depth_camera *
		//      depth_camera_T_opengl_camera;
		if(inOpenGLFrame)
		{
			pose = opengl_world_T_tango_world * pose * deviceTDepth_ * depth_camera_T_opengl_camera;
		}
	}

	return pose;
}

rtabmap::Transform CameraTango::getPoseAtTimestamp(double timestamp, bool inOpenGLFrame)
{
	rtabmap::Transform pose;
	if(!deviceTDepth_.isNull())
	{
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

			pose = tangoPoseToTransform(&pose_start_service_T_device, inOpenGLFrame);
		}


	}
	return pose;
}

SensorData CameraTango::captureImage(CameraInfo * info)
{
	LOGI("Capturing image...");

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
		if(decimation_ > 1)
		{
			rgb = util2d::decimate(rgb, decimation_);
			model = model.scaled(1.0/double(decimation_));
		}

		// Querying the depth image's frame transformation based on the depth image's
		// timestamp.
		cv::Mat depth;
		Transform poseDepth = getPoseAtTimestamp(cloudStamp, false);
		Transform poseColor = getPoseAtTimestamp(rgbStamp, false);

		if(poseColor.getNormSquared() > 100000)
		{
			LOGE("Very large odometry color pose detected (%s)! Ignoring this frame!", poseColor.prettyPrint().c_str());
			poseColor.setNull();
		}
		if(poseDepth.getNormSquared() > 100000)
		{
			LOGE("Very large odometry depth pose detected (%s)! Ignoring this frame!", poseDepth.prettyPrint().c_str());
			poseDepth.setNull();
		}

		if(!poseDepth.isNull() && !poseColor.isNull())
		{
			// The Color Camera frame at timestamp t0 with respect to Depth
			// Camera frame at timestamp t1.
			Transform colorToDepth = deviceTDepth_.inverse() * poseColor.inverse() * poseDepth * deviceTDepth_;
			LOGI("colorToDepth=%s", colorToDepth.prettyPrint().c_str());

			int pixelsSet = 0;
			depth = cv::Mat::zeros(model_.imageHeight()/8, model_.imageWidth()/8, CV_16UC1); // mm
			CameraModel depthModel = model_.scaled(1.0f/8.0f);
			for(unsigned int i=0; i<cloud.total(); ++i)
			{
				cv::Vec3f & p = cloud.at<cv::Vec3f>(i);
				cv::Point3f pt = util3d::transformPoint(cv::Point3f(p[0], p[1], p[2]), colorToDepth);

				int pixel_x, pixel_y;
				// get the coordinate on image plane.
				pixel_x = static_cast<int>((depthModel.fx()) * (pt.x / pt.z) + depthModel.cx());
				pixel_y = static_cast<int>((depthModel.fy()) * (pt.y / pt.z) + depthModel.cy());
				unsigned short depth_value(pt.z * 1000.0f);

				if(pixel_x>=0 && pixel_x<depth.cols &&
				   pixel_y>0 && pixel_y<depth.rows &&
				   depth_value)
				{
					unsigned short & depthPixel = depth.at<unsigned short>(pixel_y, pixel_x);
					if(depthPixel == 0 || depthPixel > depth_value)
					{
						depthPixel = depth_value;
						pixelsSet += 1;
					}
				}
			}
			LOGI("pixels depth set= %d", pixelsSet);
		}
		else
		{
			LOGE("Poses are null?!? color=%d (stamp=%f) depth=%d (stamp=%f)", poseColor.isNull()?0:1, rgbStamp, poseDepth.isNull()?0:1, cloudStamp);
		}

		if(!rgb.empty() && !depth.empty())
		{
			depth = rtabmap::util2d::fillDepthHoles(depth, holeSize, maxDepthError);

			Transform poseColorOpenGL = getPoseAtTimestamp(rgbStamp, true);

			//Rotate in RTAB-Map's coordinate
			Transform odom = rtabmap_world_T_opengl_world * poseColorOpenGL * depth_camera_T_opengl_camera * model.localTransform().inverse();

			data = SensorData(rgb, depth, model, this->getNextSeqID(), rgbStamp);
			data.setGroundTruth(odom);
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
	uSleep(2000); // just to make sure that the camera is started
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
			LOGI("Publish odometry message (variance=%f)", firstFrame_?9999:0.0001);
			this->post(new OdometryEvent(data, pose, firstFrame_?9999:0.0001, firstFrame_?9999:0.0001));
			firstFrame_ = false;
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
