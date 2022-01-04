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

#include "CameraARCore.h"
#include "util.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/util2d.h"

namespace rtabmap {

//////////////////////////////
// CameraARCore
//////////////////////////////
CameraARCore::CameraARCore(void* env, void* context, void* activity, bool depthFromMotion, bool smoothing):
	CameraMobile(smoothing),
	env_(env),
	context_(context),
	activity_(activity),
	arInstallRequested_(false),
	depthFromMotion_(depthFromMotion)
{
}

CameraARCore::~CameraARCore() {
	// Disconnect ARCore service
	close();
}


struct CameraConfig {
  int32_t width = 0;
  int32_t height = 0;
  std::string config_label;
  ArCameraConfig* config = nullptr;
};

void getCameraConfigLowestAndHighestResolutions(
		std::vector<CameraConfig> & camera_configs,
    CameraConfig** lowest_resolution_config,
    CameraConfig** highest_resolution_config) {
  if (camera_configs.empty()) {
    return;
  }

  int low_resolution_config_idx = 0;
  int high_resolution_config_idx = 0;
  int32_t smallest_height = camera_configs[0].height;
  int32_t largest_height = camera_configs[0].height;

  for (int i = 1; i < camera_configs.size(); ++i) {
    int32_t image_height = camera_configs[i].height;
    if (image_height < smallest_height) {
      smallest_height = image_height;
      low_resolution_config_idx = i;
    } else if (image_height > largest_height) {
      largest_height = image_height;
      high_resolution_config_idx = i;
    }
  }

  if (low_resolution_config_idx == high_resolution_config_idx) {
    *lowest_resolution_config = &camera_configs[low_resolution_config_idx];
  } else {
    *lowest_resolution_config = &camera_configs[low_resolution_config_idx];
    *highest_resolution_config = &camera_configs[high_resolution_config_idx];
  }
}

void copyCameraConfig(
    const ArSession* ar_session, const ArCameraConfigList* all_configs,
    int index, int num_configs, CameraConfig* camera_config) {
  if (camera_config != nullptr && index >= 0 && index < num_configs) {
    ArCameraConfig_create(ar_session, &camera_config->config);
    ArCameraConfigList_getItem(ar_session, all_configs, index,
                               camera_config->config);
    ArCameraConfig_getImageDimensions(ar_session, camera_config->config,
                                      &camera_config->width,
                                      &camera_config->height);
    camera_config->config_label = "(" + std::to_string(camera_config->width) +
                                  "x" + std::to_string(camera_config->height) +
                                  ")";
  }
}

void destroyCameraConfigs(std::vector<CameraConfig> & camera_configs) {
  for (int i = 0; i < camera_configs.size(); ++i) {
    if (camera_configs[i].config != nullptr) {
      ArCameraConfig_destroy(camera_configs[i].config);
    }
  }
}

std::string CameraARCore::getSerial() const
{
	return "ARCore";
}

bool CameraARCore::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	close();

	UScopeMutex lock(arSessionMutex_);

	ArInstallStatus install_status;
	// If install was not yet requested, that means that we are resuming the
	// activity first time because of explicit user interaction (such as
	// launching the application)
	bool user_requested_install = !arInstallRequested_;

	// === ATTENTION!  ATTENTION!  ATTENTION! ===
	// This method can and will fail in user-facing situations.  Your
	// application must handle these cases at least somewhat gracefully.  See
	// HelloAR Java sample code for reasonable behavior.
	ArCoreApk_requestInstall(env_, activity_, user_requested_install, &install_status);

	switch (install_status)
	{
		case AR_INSTALL_STATUS_INSTALLED:
			break;
		case AR_INSTALL_STATUS_INSTALL_REQUESTED:
			arInstallRequested_ = true;
			return false;
	}

	// === ATTENTION!  ATTENTION!  ATTENTION! ===
	// This method can and will fail in user-facing situations.  Your
	// application must handle these cases at least somewhat gracefully.  See
	// HelloAR Java sample code for reasonable behavior.
	UASSERT(ArSession_create(env_, context_, &arSession_) == AR_SUCCESS);
	UASSERT(arSession_);

	int32_t is_depth_supported = 0;
	ArSession_isDepthModeSupported(arSession_, AR_DEPTH_MODE_AUTOMATIC, &is_depth_supported);

	ArConfig_create(arSession_, &arConfig_);
	UASSERT(arConfig_);

	if (is_depth_supported!=0) {
	  ArConfig_setDepthMode(arSession_, arConfig_, AR_DEPTH_MODE_AUTOMATIC);
	} else {
	  ArConfig_setDepthMode(arSession_, arConfig_, AR_DEPTH_MODE_DISABLED);
	}

	ArConfig_setFocusMode(arSession_, arConfig_, AR_FOCUS_MODE_FIXED);
	UASSERT(ArSession_configure(arSession_, arConfig_) == AR_SUCCESS);

	ArFrame_create(arSession_, &arFrame_);
	UASSERT(arFrame_);

	ArCameraIntrinsics_create(arSession_, &arCameraIntrinsics_);
	UASSERT(arCameraIntrinsics_);

	ArPose_create(arSession_, nullptr, &arPose_);
	UASSERT(arPose_);

	ArCameraConfigList* all_camera_configs = nullptr;
	int32_t num_configs = 0;
	ArCameraConfigList_create(arSession_, &all_camera_configs);
	// Create filter first to get both 30 and 60 fps.
	ArCameraConfigFilter* camera_config_filter = nullptr;
	ArCameraConfigFilter_create(arSession_, &camera_config_filter);
	ArCameraConfigFilter_setTargetFps(arSession_, camera_config_filter, AR_CAMERA_CONFIG_TARGET_FPS_30 | AR_CAMERA_CONFIG_TARGET_FPS_60);
	ArSession_getSupportedCameraConfigsWithFilter(arSession_, camera_config_filter, all_camera_configs);
	ArCameraConfigList_getSize(arSession_, all_camera_configs, &num_configs);

	if (num_configs < 1) {
		UERROR("No camera config found");
		close();
		return false;
	}

	std::vector<CameraConfig> camera_configs;
	CameraConfig* cpu_low_resolution_camera_config_ptr = nullptr;
	CameraConfig* cpu_high_resolution_camera_config_ptr = nullptr;
	camera_configs.resize(num_configs);
	for (int i = 0; i < num_configs; ++i) {
		copyCameraConfig(arSession_, all_camera_configs, i, num_configs,
						 &camera_configs[i]);
	}
	// Determine the highest and lowest CPU resolutions.
	cpu_low_resolution_camera_config_ptr = nullptr;
	cpu_high_resolution_camera_config_ptr = nullptr;
	getCameraConfigLowestAndHighestResolutions(
			camera_configs,
			&cpu_low_resolution_camera_config_ptr,
			&cpu_high_resolution_camera_config_ptr);

	// Cleanup the list obtained as it is safe to destroy the list as camera
	// config instances were explicitly created and copied. Refer to the
	// previous comment.
	ArCameraConfigList_destroy(all_camera_configs);
	ArSession_setCameraConfig(arSession_, cpu_low_resolution_camera_config_ptr->config);

	/// Sets the behavior of @ref ArSession_update(). See
	/// ::ArUpdateMode for available options.
	ArConfig_setUpdateMode(arSession_, arConfig_, AR_UPDATE_MODE_BLOCKING);

	deviceTColorCamera_ = opticalRotation;

	if (ArSession_resume(arSession_) != ArStatus::AR_SUCCESS)
	{
		UERROR("Cannot resume camera!");
		// In a rare case (such as another camera app launching) the camera may be
		// given to a different app and so may not be available to this app. Handle
		// this properly and recreate the session at the next iteration.
		close();
		return false;
	}

	return true;
}

void CameraARCore::close()
{
	UScopeMutex lock(arSessionMutex_);
	if(arSession_!= nullptr)
	{
		ArSession_destroy(arSession_);
	}
	arSession_ = nullptr;

	if(arConfig_!= nullptr)
	{
		ArConfig_destroy(arConfig_);
	}
	arConfig_ = nullptr;

	if (arFrame_ != nullptr)
	{
		ArFrame_destroy(arFrame_);
	}
	arFrame_ = nullptr;

	if (arCameraIntrinsics_ != nullptr)
	{
		ArCameraIntrinsics_destroy(arCameraIntrinsics_);
	}
	arCameraIntrinsics_ = nullptr;

	if (arPose_ != nullptr)
	{
		ArPose_destroy(arPose_);
	}
	arPose_ = nullptr;

	CameraMobile::close();
}

LaserScan CameraARCore::scanFromPointCloudData(
		const float * pointCloudData,
		int points,
		const Transform & pose,
		const CameraModel & model,
		const cv::Mat & rgb,
		std::vector<cv::KeyPoint> * kpts,
		std::vector<cv::Point3f> * kpts3D)
{
	if(pointCloudData && points>0)
	{
		cv::Mat scanData(1, points, CV_32FC4);
		float * ptr = scanData.ptr<float>();
		for(unsigned int i=0;i<points; ++i)
		{
			cv::Point3f pt(pointCloudData[i*4], pointCloudData[i*4 + 1], pointCloudData[i*4 + 2]);
			pt = util3d::transformPoint(pt, pose.inverse()*rtabmap_world_T_opengl_world);
			ptr[i*4] = pt.x;
			ptr[i*4 + 1] = pt.y;
			ptr[i*4 + 2] = pt.z;

			//get color from rgb image
			cv::Point3f org= pt;
			pt = util3d::transformPoint(pt, opticalRotationInv);
			int u,v;
			model.reproject(pt.x, pt.y, pt.z, u, v);
			unsigned char r=255,g=255,b=255;
			if(model.inFrame(u, v))
			{
				b=rgb.at<cv::Vec3b>(v,u).val[0];
				g=rgb.at<cv::Vec3b>(v,u).val[1];
				r=rgb.at<cv::Vec3b>(v,u).val[2];
				if(kpts)
					kpts->push_back(cv::KeyPoint(u,v,3));
				if(kpts3D)
					kpts3D->push_back(org);
			}
			*(int*)&ptr[i*4 + 3] = int(b) | (int(g) << 8) | (int(r) << 16);

			//confidence
			//*(int*)&ptr[i*4 + 3] = (int(pointCloudData[i*4 + 3] * 255.0f) << 8) | (int(255) << 16);

		}
		return LaserScan::backwardCompatibility(scanData, 0, 10, rtabmap::Transform::getIdentity());
	}
	return LaserScan();
}

void CameraARCore::setScreenRotationAndSize(ScreenRotation colorCameraToDisplayRotation, int width, int height)
{
	CameraMobile::setScreenRotationAndSize(colorCameraToDisplayRotation, width, height);
	if(arSession_)
	{
		int ret = static_cast<int>(colorCameraToDisplayRotation) + 1; // remove 90deg camera rotation
		  if (ret > 3) {
		    ret -= 4;
		  }

		ArSession_setDisplayGeometry(arSession_, ret, width, height);
	}
}

SensorData CameraARCore::captureImage(CameraInfo * info)
{
	UScopeMutex lock(arSessionMutex_);
	//LOGI("Capturing image...");

	SensorData data;
	if(!arSession_)
	{
		return data;
	}

	if(textureId_ == 0)
	{
		glGenTextures(1, &textureId_);
		glBindTexture(GL_TEXTURE_EXTERNAL_OES, textureId_);
		glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}
	if(textureId_!=0)
		ArSession_setCameraTextureName(arSession_, textureId_);

	// Update session to get current frame and render camera background.
	if (ArSession_update(arSession_, arFrame_) != AR_SUCCESS) {
		LOGE("CameraARCore::captureImage() ArSession_update error");
		return data;
	}

	// If display rotation changed (also includes view size change), we need to
	// re-query the uv coordinates for the on-screen portion of the camera image.
	int32_t geometry_changed = 0;
	ArFrame_getDisplayGeometryChanged(arSession_, arFrame_, &geometry_changed);
	if (geometry_changed != 0 || !uvs_initialized_) {
		ArFrame_transformCoordinates2d(
				arSession_, arFrame_, AR_COORDINATES_2D_OPENGL_NORMALIZED_DEVICE_COORDINATES,
				BackgroundRenderer::kNumVertices, BackgroundRenderer_kVertices, AR_COORDINATES_2D_TEXTURE_NORMALIZED,
				transformed_uvs_);
		UASSERT(transformed_uvs_);
		uvs_initialized_ = true;
	}

	ArCamera* ar_camera;
	ArFrame_acquireCamera(arSession_, arFrame_, &ar_camera);

	ArCamera_getViewMatrix(arSession_, ar_camera, glm::value_ptr(viewMatrix_));
	ArCamera_getProjectionMatrix(arSession_, ar_camera,
							   /*near=*/0.1f, /*far=*/100.f,
							   glm::value_ptr(projectionMatrix_));

	// adjust origin
	if(!getOriginOffset().isNull())
	{
		viewMatrix_ = glm::inverse(rtabmap::glmFromTransform(rtabmap::opengl_world_T_rtabmap_world * getOriginOffset() *rtabmap::rtabmap_world_T_opengl_world)*glm::inverse(viewMatrix_));
	}

	ArTrackingState camera_tracking_state;
	ArCamera_getTrackingState(arSession_, ar_camera, &camera_tracking_state);

	Transform pose;
	CameraModel model;
	if(camera_tracking_state == AR_TRACKING_STATE_TRACKING)
	{
		// pose in OpenGL coordinates
		float pose_raw[7];
		ArCamera_getPose(arSession_, ar_camera, arPose_);
		ArPose_getPoseRaw(arSession_, arPose_, pose_raw);
		pose = Transform(pose_raw[4], pose_raw[5], pose_raw[6], pose_raw[0], pose_raw[1], pose_raw[2], pose_raw[3]);
		pose = rtabmap::rtabmap_world_T_opengl_world * pose * rtabmap::opengl_world_T_rtabmap_world;

		Transform poseArCore = pose;
		if(pose.isNull())
		{
			LOGE("CameraARCore: Pose is null");
		}
		else
		{
			this->poseReceived(pose);
			// adjust origin
			if(!getOriginOffset().isNull())
			{
				pose = getOriginOffset() * pose;
			}
			info->odomPose = pose;
		}

		// Get calibration parameters
		float fx,fy, cx, cy;
		int32_t width, height;
		ArCamera_getImageIntrinsics(arSession_, ar_camera, arCameraIntrinsics_);
		ArCameraIntrinsics_getFocalLength(arSession_, arCameraIntrinsics_, &fx, &fy);
		ArCameraIntrinsics_getPrincipalPoint(arSession_, arCameraIntrinsics_, &cx, &cy);
		ArCameraIntrinsics_getImageDimensions(arSession_, arCameraIntrinsics_, &width, &height);
#ifndef DISABLE_LOG
		LOGI("%f %f %f %f %d %d", fx, fy, cx, cy, width, height);
#endif

		if(fx > 0 && fy > 0 && width > 0 && height > 0 && cx > 0 && cy > 0)
		{
			model = CameraModel(fx, fy, cx, cy, deviceTColorCamera_, 0, cv::Size(width, height));

			ArPointCloud * pointCloud = nullptr;
			ArFrame_acquirePointCloud(arSession_, arFrame_, &pointCloud);

			int32_t is_depth_supported = 0;
			ArSession_isDepthModeSupported(arSession_, AR_DEPTH_MODE_AUTOMATIC, &is_depth_supported);

			ArImage * image = nullptr;
			ArStatus status = ArFrame_acquireCameraImage(arSession_, arFrame_, &image);
			if(status == AR_SUCCESS)
			{
				if(is_depth_supported)
				{
					LOGD("Acquire depth image!");
					ArImage * depthImage = nullptr;
					ArFrame_acquireDepthImage(arSession_, arFrame_, &depthImage);

					ArImageFormat format;
					ArImage_getFormat(arSession_, depthImage, &format);
					if(format == AR_IMAGE_FORMAT_DEPTH16)
					{
						LOGD("Depth format detected!");
						int planeCount;
						ArImage_getNumberOfPlanes(arSession_, depthImage, &planeCount);
						LOGD("planeCount=%d", planeCount);
						UASSERT_MSG(planeCount == 1, uFormat("Error: getNumberOfPlanes() planceCount = %d", planeCount).c_str());
						const uint8_t *data = nullptr;
						int len = 0;
						int stride;
						int depth_width;
						int depth_height;
						ArImage_getWidth(arSession_, depthImage, &depth_width);
						ArImage_getHeight(arSession_, depthImage, &depth_height);
						ArImage_getPlaneRowStride(arSession_, depthImage, 0, &stride);
						ArImage_getPlaneData(arSession_, depthImage, 0, &data, &len);

						LOGD("width=%d, height=%d, bytes=%d stride=%d", depth_width, depth_height, len, stride);

						cv::Mat occlusionImage = cv::Mat(depth_height, depth_width, CV_16UC1, (void*)data).clone();

						float scaleX = (float)depth_width / (float)width;
						float scaleY = (float)depth_height / (float)height;
						CameraModel occlusionModel(fx*scaleX, fy*scaleY, cx*scaleX, cy*scaleY, pose*deviceTColorCamera_, 0, cv::Size(depth_width, depth_height));
						this->setOcclusionImage(occlusionImage, occlusionModel);
					}
					ArImage_release(depthImage);
				}

				int64_t timestamp_ns;
				ArImageFormat format;
				ArImage_getTimestamp(arSession_, image, &timestamp_ns);
				ArImage_getFormat(arSession_, image, &format);
				if(format == AR_IMAGE_FORMAT_YUV_420_888)
				{
#ifndef DISABLE_LOG
					int32_t num_planes;
					ArImage_getNumberOfPlanes(arSession_, image, &num_planes);
					for(int i=0;i<num_planes; ++i)
					{
						int32_t pixel_stride;
						int32_t row_stride;
						ArImage_getPlanePixelStride(arSession_, image, i, &pixel_stride);
						ArImage_getPlaneRowStride(arSession_, image, i, &row_stride);
						LOGI("Plane %d/%d: pixel stride=%d, row stride=%d", i+1, num_planes, pixel_stride, row_stride);
					}
#endif
					const uint8_t * plane_data;
					const uint8_t * plane_uv_data;
					int32_t data_length;
					ArImage_getPlaneData(arSession_, image, 0, &plane_data, &data_length);
					int32_t uv_data_length;
					ArImage_getPlaneData(arSession_, image, 2, &plane_uv_data, &uv_data_length);

					if(plane_data != nullptr && data_length == height*width)
					{
						double stamp = double(timestamp_ns)/10e8;
#ifndef DISABLE_LOG
						LOGI("data_length=%d stamp=%f", data_length, stamp);
#endif
						cv::Mat rgb;
						if((long)plane_uv_data-(long)plane_data != data_length)
						{
							// The uv-plane is not concatenated to y plane in memory, so concatenate them
							cv::Mat yuv(height+height/2, width, CV_8UC1);
							memcpy(yuv.data, plane_data, data_length);
							memcpy(yuv.data+data_length, plane_uv_data, height/2*width);
							cv::cvtColor(yuv, rgb, cv::COLOR_YUV2BGR_NV21);
						}
						else
						{
							cv::cvtColor(cv::Mat(height+height/2, width, CV_8UC1, (void*)plane_data), rgb, cv::COLOR_YUV2BGR_NV21);
						}

						std::vector<cv::KeyPoint> kpts;
						std::vector<cv::Point3f> kpts3;
						LaserScan scan;
						if(pointCloud)
						{
							int32_t points = 0;
							ArPointCloud_getNumberOfPoints(arSession_, pointCloud, &points);
							const float * pointCloudData = 0;
							ArPointCloud_getData(arSession_, pointCloud, &pointCloudData);
#ifndef DISABLE_LOG
							LOGI("pointCloudData=%d size=%d", pointCloudData?1:0, points);
#endif
							if(pointCloudData && points>0)
							{
								scan = scanFromPointCloudData(pointCloudData, points, poseArCore, model, rgb, &kpts, &kpts3);
							}
						}
						else
						{
							LOGI("pointCloud empty");
						}

						data = SensorData(scan, rgb, depthFromMotion_?getOcclusionImage():cv::Mat(), model, 0, stamp);
						data.setFeatures(kpts,  kpts3, cv::Mat());
					}
				}
				else
				{
					LOGE("CameraARCore: cannot convert image format %d", format);
				}
			}
			else
			{
				LOGE("CameraARCore: failed to get rgb image (status=%d)", (int)status);
			}

			ArImage_release(image);
			ArPointCloud_release(pointCloud);
		}
	}

	ArCamera_release(ar_camera);

	return data;

}

void CameraARCore::capturePoseOnly()
{
	UScopeMutex lock(arSessionMutex_);
	//LOGI("Capturing image...");

	if(!arSession_)
	{
		return;
	}

	if(textureId_ != 0)
	{
		glBindTexture(GL_TEXTURE_EXTERNAL_OES, textureId_);
		glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		ArSession_setCameraTextureName(arSession_, textureId_);
	}

	// Update session to get current frame and render camera background.
	if (ArSession_update(arSession_, arFrame_) != AR_SUCCESS) {
		LOGE("CameraARCore::capturePoseOnly() ArSession_update error");
		return;
	}

	// If display rotation changed (also includes view size change), we need to
	// re-query the uv coordinates for the on-screen portion of the camera image.
	int32_t geometry_changed = 0;
	ArFrame_getDisplayGeometryChanged(arSession_, arFrame_, &geometry_changed);
	if (geometry_changed != 0 || !uvs_initialized_) {
		ArFrame_transformCoordinates2d(
				arSession_, arFrame_, AR_COORDINATES_2D_OPENGL_NORMALIZED_DEVICE_COORDINATES,
				BackgroundRenderer::kNumVertices, BackgroundRenderer_kVertices, AR_COORDINATES_2D_TEXTURE_NORMALIZED,
				transformed_uvs_);
		UASSERT(transformed_uvs_);
		uvs_initialized_ = true;
	}

	ArCamera* ar_camera;
	ArFrame_acquireCamera(arSession_, arFrame_, &ar_camera);

	ArCamera_getViewMatrix(arSession_, ar_camera, glm::value_ptr(viewMatrix_));
	ArCamera_getProjectionMatrix(arSession_, ar_camera,
							   /*near=*/0.1f, /*far=*/100.f,
							   glm::value_ptr(projectionMatrix_));

	// adjust origin
	if(!getOriginOffset().isNull())
	{
		viewMatrix_ = glm::inverse(rtabmap::glmFromTransform(rtabmap::opengl_world_T_rtabmap_world * getOriginOffset() *rtabmap::rtabmap_world_T_opengl_world)*glm::inverse(viewMatrix_));
	}

	ArTrackingState camera_tracking_state;
	ArCamera_getTrackingState(arSession_, ar_camera, &camera_tracking_state);

	Transform pose;
	CameraModel model;
	if(camera_tracking_state == AR_TRACKING_STATE_TRACKING)
	{
		// pose in OpenGL coordinates
		float pose_raw[7];
		ArCamera_getPose(arSession_, ar_camera, arPose_);
		ArPose_getPoseRaw(arSession_, arPose_, pose_raw);
		pose = Transform(pose_raw[4], pose_raw[5], pose_raw[6], pose_raw[0], pose_raw[1], pose_raw[2], pose_raw[3]);
		if(!pose.isNull())
		{
			pose = rtabmap::rtabmap_world_T_opengl_world * pose * rtabmap::opengl_world_T_rtabmap_world;
			this->poseReceived(pose);

			if(!getOriginOffset().isNull())
			{
				pose = getOriginOffset() * pose;
			}
		}

		int32_t is_depth_supported = 0;
		ArSession_isDepthModeSupported(arSession_, AR_DEPTH_MODE_AUTOMATIC, &is_depth_supported);

		if(is_depth_supported)
		{
			LOGD("Acquire depth image!");
			ArImage * depthImage = nullptr;
			ArFrame_acquireDepthImage(arSession_, arFrame_, &depthImage);

			ArImageFormat format;
			ArImage_getFormat(arSession_, depthImage, &format);
			if(format == AR_IMAGE_FORMAT_DEPTH16)
			{
				LOGD("Depth format detected!");
				int planeCount;
				ArImage_getNumberOfPlanes(arSession_, depthImage, &planeCount);
				LOGD("planeCount=%d", planeCount);
				UASSERT_MSG(planeCount == 1, uFormat("Error: getNumberOfPlanes() planceCount = %d", planeCount).c_str());
				const uint8_t *data = nullptr;
				int len = 0;
				int stride;
				int width;
				int height;
				ArImage_getWidth(arSession_, depthImage, &width);
				ArImage_getHeight(arSession_, depthImage, &height);
				ArImage_getPlaneRowStride(arSession_, depthImage, 0, &stride);
				ArImage_getPlaneData(arSession_, depthImage, 0, &data, &len);

				LOGD("width=%d, height=%d, bytes=%d stride=%d", width, height, len, stride);

				cv::Mat occlusionImage = cv::Mat(height, width, CV_16UC1, (void*)data).clone();

				float fx,fy, cx, cy;
				int32_t rgb_width, rgb_height;
				ArCamera_getImageIntrinsics(arSession_, ar_camera, arCameraIntrinsics_);
				ArCameraIntrinsics_getFocalLength(arSession_, arCameraIntrinsics_, &fx, &fy);
				ArCameraIntrinsics_getPrincipalPoint(arSession_, arCameraIntrinsics_, &cx, &cy);
				ArCameraIntrinsics_getImageDimensions(arSession_, arCameraIntrinsics_, &rgb_width, &rgb_height);

				float scaleX = (float)width / (float)rgb_width;
				float scaleY = (float)height / (float)rgb_height;
				CameraModel occlusionModel(fx*scaleX, fy*scaleY, cx*scaleX, cy*scaleY, pose*deviceTColorCamera_, 0, cv::Size(width, height));
				this->setOcclusionImage(occlusionImage, occlusionModel);
			}
			ArImage_release(depthImage);
		}
	}

	ArCamera_release(ar_camera);
}

} /* namespace rtabmap */
