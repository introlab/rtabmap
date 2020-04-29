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

#ifdef DEPTH_TEST
// Camera Callbacks
static void CameraDeviceOnDisconnected(void* context, ACameraDevice* device) {
  LOGE("Camera(id: %s) is disconnected.\n", ACameraDevice_getId(device));
}
static void CameraDeviceOnError(void* context, ACameraDevice* device,
                                int error) {
  LOGE("Error(code: %d) on Camera(id: %s).\n", error,
       ACameraDevice_getId(device));
}
// Capture Callbacks
bool g_captureSessionReady = false;
static void CaptureSessionOnReady(void* context,
                                  ACameraCaptureSession* session) {
  LOGI("Session is ready.\n");
  g_captureSessionReady = true;
}
static void CaptureSessionOnActive(void* context,
                                   ACameraCaptureSession* session) {
  LOGI("Session is activated.\n");
}
#endif // DEPTH_TEST

//////////////////////////////
// CameraARCore
//////////////////////////////
CameraARCore::CameraARCore(void* env, void* context, void* activity, bool smoothing):
	CameraMobile(smoothing),
	env_(env),
	context_(context),
	activity_(activity),
	arInstallRequested_(false)
{
	glGenTextures(1, &textureId_);
}

CameraARCore::~CameraARCore() {
	// Disconnect ARCore service
	close();

	glDeleteTextures(1, &textureId_);
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

#ifdef DEPTH_TEST
void OnImageCallback(void *ctx, AImageReader *reader) {
  reinterpret_cast<CameraARCore *>(ctx)->imageCallback(reader);
}
void CameraARCore::imageCallback(AImageReader *reader) {
	int32_t format;
	media_status_t status = AImageReader_getFormat(reader, &format);
	UWARN("format=%d", format);
	UASSERT_MSG(status == AMEDIA_OK, "Failed to get the media format");

	if (format == AIMAGE_FORMAT_DEPTH16) {
		// Create a thread and write out the jpeg files
		AImage *image = nullptr;
		media_status_t status = AImageReader_acquireNextImage(reader, &image);
		UASSERT_MSG(status == AMEDIA_OK && image, "Image is not available");

		int planeCount;
		status = AImage_getNumberOfPlanes(image, &planeCount);
		UASSERT_MSG(status == AMEDIA_OK && planeCount == 1,
				uFormat("Error: getNumberOfPlanes() planceCount = %d", planeCount).c_str());
		uint8_t *data = nullptr;
		int len = 0;
		int stride;
		int width;
		int height;
		AImage_getWidth(image, &width);
		AImage_getHeight(image, &height);
		AImage_getPlaneRowStride(image, 0, &stride);
		AImage_getPlaneData(image, 0, &data, &len);

		cv::Mat output(height, width, CV_16UC1);
		uint16_t *dataShort = (uint16_t *)data;
		uint16_t max=0x0;
		for (int y = 0; y < output.rows; ++y)
		{
			for (int x = 0; x < output.cols; ++x)
			{
				uint16_t depthSample = dataShort[y*output.cols + x];
				uint16_t depthRange = (depthSample & 0x1FFF); // first 3 bits are confidence
				output.at<uint16_t>(y,x) = depthRange;
				if(depthRange > max)
				{
					max = depthRange;
				}
			}
		}
		UWARN("width=%d, height=%d, bytes=%d stride=%d max=%dmm",
				width, height, len, stride, (int)max);

		std::string path = "/storage/emulated/0/RTAB-Map/depth.png";
		cv::imwrite(path, output);
		UWARN("depth image saved to %s", path.c_str());

		AImage_delete(image);
	}
}
#endif // DEPTH_TEST

bool CameraARCore::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	close();

#ifdef DEPTH_TEST
	///////////////////////////
	// Depth image using camera2 API
	/////////////////////////////
	camera_status_t cameraStatus = ACAMERA_OK;

	cameraManager_ = ACameraManager_create();

	deviceStateCallbacks_.onDisconnected = CameraDeviceOnDisconnected;
	deviceStateCallbacks_.onError = CameraDeviceOnError;

	const char * cameraId = "0";
	cameraStatus = ACameraManager_openCamera(cameraManager_, cameraId, &deviceStateCallbacks_, &cameraDevice_);
	UASSERT_MSG(cameraStatus == ACAMERA_OK, uFormat("Failed to open camera device (id: %s)",
			cameraId).c_str());

	// Currently only working resolution on Huawei P30 Pro
	cv::Size size(240, 180);
	int format = AIMAGE_FORMAT_DEPTH16;

	media_status_t mediaStatus = AImageReader_new(size.width, size.height, format, 2, &imageReader_);
	UASSERT_MSG(imageReader_ && mediaStatus == AMEDIA_OK, uFormat("Failed to create AImageReader %dx%d format=%d",
			size.width, size.height, format).c_str());

	AImageReader_ImageListener listener{
	  .context = this,
	  .onImageAvailable = OnImageCallback,
	};
	AImageReader_setImageListener(imageReader_, &listener);

	//
	ANativeWindow *nativeWindow;
	mediaStatus = AImageReader_getWindow(imageReader_, &nativeWindow);
	UASSERT_MSG(mediaStatus == AMEDIA_OK, "Could not get ANativeWindow");

	outputNativeWindow_ = nativeWindow;
	ACaptureSessionOutputContainer_create(&captureSessionOutputContainer_);
	ANativeWindow_acquire(outputNativeWindow_);
	ACaptureSessionOutput_create(outputNativeWindow_, &sessionOutput_);
	ACaptureSessionOutputContainer_add(captureSessionOutputContainer_, sessionOutput_);
	ACameraOutputTarget_create(outputNativeWindow_, &cameraOutputTarget_);

	cameraStatus = ACameraDevice_createCaptureRequest(cameraDevice_, TEMPLATE_RECORD, &captureRequest_);
	UASSERT_MSG(cameraStatus == ACAMERA_OK,
			uFormat("Failed to create preview capture request (id: %s, status=%d)",
					cameraId, cameraStatus).c_str());

	ACaptureRequest_addTarget(captureRequest_, cameraOutputTarget_);

	captureSessionStateCallbacks_.onReady = CaptureSessionOnReady;
	captureSessionStateCallbacks_.onActive = CaptureSessionOnActive;
	ACameraDevice_createCaptureSession(
			cameraDevice_,
			captureSessionOutputContainer_, // outputs
			&captureSessionStateCallbacks_, // callbacks
			&captureSession_);

	ACameraCaptureSession_setRepeatingRequest(captureSession_, nullptr, 1,
			&captureRequest_, nullptr);

	// Don't start ARCore as we cannot use both at the same time
	return true;
#endif // DEPTH_TEST

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

	ArConfig_create(arSession_, &arConfig_);
	UASSERT(arConfig_);

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

	// Required as ArSession_update does some off-screen OpenGL stuff...
	ArSession_setCameraTextureName(arSession_, textureId_);

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

#ifdef DEPTH_TEST

	if(captureSession_!=nullptr)
	{
		g_captureSessionReady = false;
		ACameraCaptureSession_stopRepeating(captureSession_);
		double start = UTimer::now();
		while(g_captureSessionReady != true && UTimer::now()-start < 2.0){
			uSleep(100);
			UWARN("Waiting session to close.... max 2 seconds");
		}
		//ACameraCaptureSession_close(captureSession_); // FIXME: this crashes?!
		captureSession_ = nullptr;

		ACaptureRequest_removeTarget(captureRequest_, cameraOutputTarget_);
		ACaptureRequest_free(captureRequest_);
		ACameraOutputTarget_free(cameraOutputTarget_);
		captureRequest_ = nullptr;
		cameraOutputTarget_ = nullptr;

		ACaptureSessionOutputContainer_remove(captureSessionOutputContainer_, sessionOutput_);
		ANativeWindow_release(outputNativeWindow_);
		ACaptureSessionOutputContainer_free(captureSessionOutputContainer_);
		ACaptureSessionOutput_free(sessionOutput_);
		captureSessionOutputContainer_ = nullptr;
		sessionOutput_ = nullptr;

		ACameraDevice_close(cameraDevice_);
		cameraDevice_ = nullptr;

		ACameraManager_delete(cameraManager_);
		cameraManager_ = nullptr;

		AImageReader_delete(imageReader_);
		imageReader_ = nullptr;
	}
#endif

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

SensorData CameraARCore::captureImage(CameraInfo * info)
{
	UScopeMutex lock(arSessionMutex_);
	//LOGI("Capturing image...");

	SensorData data;
	if(!arSession_)
	{
		return data;
	}

	// Update session to get current frame and render camera background.
	if (ArSession_update(arSession_, arFrame_) != AR_SUCCESS) {
		LOGE("CameraARCore::captureImage() ArSession_update error");
		return data;
	}

	ArCamera* ar_camera;
	ArFrame_acquireCamera(arSession_, arFrame_, &ar_camera);

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

			ArImage * image = nullptr;
			ArStatus status = ArFrame_acquireCameraImage(arSession_, arFrame_, &image);
			if(status == AR_SUCCESS)
			{
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
							cv::cvtColor(yuv, rgb, CV_YUV2BGR_NV21);
						}
						else
						{
							cv::cvtColor(cv::Mat(height+height/2, width, CV_8UC1, (void*)plane_data), rgb, CV_YUV2BGR_NV21);
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
								scan = scanFromPointCloudData(pointCloudData, points, pose, model, rgb, &kpts, &kpts3);
							}
						}
						else
						{
							LOGI("pointCloud empty");
						}

						data = SensorData(scan, rgb, cv::Mat(), model, 0, stamp);
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

	if(pose.isNull())
	{
		LOGE("CameraARCore: Pose is null");
	}
	else
	{
		this->poseReceived(pose);
		info->odomPose = pose;
	}
	return data;

}

void CameraARCore::capturePoseOnly()
{
	UScopeMutex lock(arSessionMutex_);
	//LOGI("Capturing image...");

	SensorData data;
	if(!arSession_)
	{
		return;
	}

	// Update session to get current frame and render camera background.
	if (ArSession_update(arSession_, arFrame_) != AR_SUCCESS) {
		LOGE("CameraARCore::captureImage() ArSession_update error");
		return;
	}

	ArCamera* ar_camera;
	ArFrame_acquireCamera(arSession_, arFrame_, &ar_camera);

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
		}
	}

	ArCamera_release(ar_camera);
}

} /* namespace rtabmap */
