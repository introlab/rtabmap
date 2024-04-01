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

#include "CameraAREngine.h"
#include "util.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/util2d.h"

#include <media/NdkImage.h>

namespace rtabmap {


//////////////////////////////
// CameraAREngine
//////////////////////////////
CameraAREngine::CameraAREngine(void* env, void* context, void* activity, bool smoothing):
	CameraMobile(smoothing),
	env_(env),
	context_(context),
	activity_(activity),
	arInstallRequested_(false)
{
	glGenTextures(1, &textureId_);
}

CameraAREngine::~CameraAREngine() {
	// Disconnect ARCore service
	close();

	glDeleteTextures(1, &textureId_);
}

std::string CameraAREngine::getSerial() const
{
	return "AREngine";
}

bool CameraAREngine::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	close();

	UScopeMutex lock(arSessionMutex_);

	HwArInstallStatus install_status;
	// If install was not yet requested, that means that we are resuming the
	// activity first time because of explicit user interaction (such as
	// launching the application)
	bool user_requested_install = !arInstallRequested_;

	// === ATTENTION!  ATTENTION!  ATTENTION! ===
	// This method can and will fail in user-facing situations.  Your
	// application must handle these cases at least somewhat gracefully.  See
	// HelloAR Java sample code for reasonable behavior.
	HwArEnginesApk_requestInstall(env_, activity_, user_requested_install, &install_status);

	switch (install_status)
	{
		case HWAR_INSTALL_STATUS_INSTALLED:
			break;
		case HWAR_INSTALL_STATUS_INSTALL_REQUESTED:
			arInstallRequested_ = true;
			return false;
	}

	// === ATTENTION!  ATTENTION!  ATTENTION! ===
	// This method can and will fail in user-facing situations.  Your
	// application must handle these cases at least somewhat gracefully.  See
	// HelloAR Java sample code for reasonable behavior.
	UASSERT(HwArSession_create(env_, context_, &arSession_) == HWAR_SUCCESS);
	UASSERT(arSession_);

	HwArConfig_create(arSession_, &arConfig_);
	UASSERT(arConfig_);

	HwArConfig_setFocusMode(arSession_, arConfig_, HWAR_FOCUS_MODE_FIXED);
	UASSERT(HwArSession_configure(arSession_, arConfig_) == HWAR_SUCCESS);

	HwArFrame_create(arSession_, &arFrame_);
	UASSERT(arFrame_);

	HwArCameraIntrinsics_create(arSession_, &arCameraIntrinsics_); // May fail?!
	//UASSERT(arCameraIntrinsics_);

	HwArPose_create(arSession_, nullptr, &arPose_);
	UASSERT(arPose_);

	/// Sets the behavior of @ref ArSession_update(). See
	/// ::ArUpdateMode for available options.
	HwArConfig_setUpdateMode(arSession_, arConfig_, HWAR_UPDATE_MODE_BLOCKING);

	deviceTColorCamera_ = opticalRotation;

	// Required as ArSession_update does some off-screen OpenGL stuff...
	HwArSession_setCameraTextureName(arSession_, textureId_);

	if (HwArSession_resume(arSession_) != HWAR_SUCCESS)
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

void CameraAREngine::close()
{
	UScopeMutex lock(arSessionMutex_);
	if (arCameraIntrinsics_ != nullptr)
	{
		HwArCameraIntrinsics_destroy(arSession_, arCameraIntrinsics_);
	}
	arCameraIntrinsics_ = nullptr;

	if(arSession_!= nullptr)
	{
		HwArSession_destroy(arSession_);
	}
	arSession_ = nullptr;

	if(arConfig_!= nullptr)
	{
		HwArConfig_destroy(arConfig_);
	}
	arConfig_ = nullptr;

	if (arFrame_ != nullptr)
	{
		HwArFrame_destroy(arFrame_);
	}
	arFrame_ = nullptr;

	if (arPose_ != nullptr)
	{
		HwArPose_destroy(arPose_);
	}
	arPose_ = nullptr;

	CameraMobile::close();
}

SensorData CameraAREngine::captureImage(CameraInfo * info)
{
	UScopeMutex lock(arSessionMutex_);
	//LOGI("Capturing image...");

	SensorData data;
	if(!arSession_)
	{
		return data;
	}

	// Update session to get current frame and render camera background.
	if (HwArSession_update(arSession_, arFrame_) != HWAR_SUCCESS) {
		LOGE("CameraAREngine::captureImage() ArSession_update error");
		return data;
	}

	HwArCamera* ar_camera;
	HwArFrame_acquireCamera(arSession_, arFrame_, &ar_camera);

	HwArTrackingState camera_tracking_state;
	HwArCamera_getTrackingState(arSession_, ar_camera, &camera_tracking_state);

	Transform pose;
	if(camera_tracking_state == HWAR_TRACKING_STATE_TRACKING)
	{
		// pose in OpenGL coordinates
		float pose_raw[7];
		HwArCamera_getPose(arSession_, ar_camera, arPose_);
		HwArPose_getPoseRaw(arSession_, arPose_, pose_raw);
		pose = Transform(pose_raw[4], pose_raw[5], pose_raw[6], pose_raw[0], pose_raw[1], pose_raw[2], pose_raw[3]);

		// Get calibration parameters
		// FIXME: Hard-coded as getting intrinsics with the api fails
		float fx=492.689667,fy=492.606201, cx=323.594849, cy=234.659744;
		int32_t camWidth=640, camHeight=480;
		//HwArCamera_getImageIntrinsics(arSession_, ar_camera, arCameraIntrinsics_);
		//HwArCameraIntrinsics_getFocalLength(arSession_, arCameraIntrinsics_, &fx, &fy);
		//HwArCameraIntrinsics_getPrincipalPoint(arSession_, arCameraIntrinsics_, &cx, &cy);
		//HwArCameraIntrinsics_getImageDimensions(arSession_, arCameraIntrinsics_, &camWidth, &camHeight);
		LOGI("%f %f %f %f %d %d", fx, fy, cx, cy, camWidth, camHeight);

		if(fx > 0 && fy > 0 && camWidth > 0 && camHeight > 0 && cx > 0 && cy > 0)
		{
			//ArPointCloud * point_cloud;
			//ArFrame_acquirePointCloud(ar_session_, ar_frame_, &point_cloud);

			HwArImage * image = nullptr;
			HwArImage * depthImage = nullptr;
			HwArStatus statusRgb = HwArFrame_acquireCameraImage(arSession_, arFrame_, &image);
			HwArStatus statusDepth = HwArFrame_acquireDepthImage(arSession_, arFrame_, &depthImage);
			if(statusRgb == HWAR_SUCCESS && statusDepth == HWAR_SUCCESS)
			{
				int64_t timestamp_ns;
				HwArFrame_getTimestamp(arSession_, arFrame_, &timestamp_ns);

				int planeCount;
				uint8_t *imageData = nullptr;
				int len = 0;
				int stride;
				int width;
				int height;
				const AImage* ndkImageRGB;
				HwArImage_getNdkImage(image, &ndkImageRGB);

				AImage_getNumberOfPlanes(ndkImageRGB, &planeCount);
				AImage_getWidth(ndkImageRGB, &width);
				AImage_getHeight(ndkImageRGB, &height);
				AImage_getPlaneRowStride(ndkImageRGB, 0, &stride);
				AImage_getPlaneData(ndkImageRGB, 0, &imageData, &len);
				LOGI("RGB: width=%d, height=%d, bytes=%d stride=%d planeCount=%d", width, height, len, stride, planeCount);

				cv::Mat outputRGB;
				if(imageData != nullptr && len>0)
				{
					cv::cvtColor(cv::Mat(height+height/2, width, CV_8UC1, (void*)imageData), outputRGB, cv::COLOR_YUV2BGR_NV21);
				}

				//Depth
				const AImage* ndkImageDepth;
				HwArImage_getNdkImage(depthImage, &ndkImageDepth);
				AImage_getNumberOfPlanes(ndkImageDepth, &planeCount);
				AImage_getWidth(ndkImageDepth, &width);
				AImage_getHeight(ndkImageDepth, &height);
				AImage_getPlaneRowStride(ndkImageDepth, 0, &stride);
				AImage_getPlaneData(ndkImageDepth, 0, &imageData, &len);
				LOGI("Depth: width=%d, height=%d, bytes=%d stride=%d planeCount=%d", width, height, len, stride, planeCount);

				cv::Mat outputDepth(height, width, CV_16UC1);
				uint16_t *dataShort = (uint16_t *)imageData;
				for (int y = 0; y < outputDepth.rows; ++y)
				{
					for (int x = 0; x < outputDepth.cols; ++x)
					{
						uint16_t depthSample = dataShort[y*outputDepth.cols + x];
						uint16_t depthRange = (depthSample & 0x1FFF); // first 3 bits are confidence
						outputDepth.at<uint16_t>(y,x) = depthRange;
					}
				}

				if(!outputRGB.empty() && !outputDepth.empty())
				{
					double stamp = double(timestamp_ns)/10e8;
					CameraModel model = CameraModel(fx, fy, cx, cy, deviceTColorCamera_, 0, cv::Size(camWidth, camHeight));
					data = SensorData(outputRGB, outputDepth, model, 0, stamp);
				}
			}
			else
			{
				LOGE("CameraAREngine: failed to get rgb image (status=%d %d)", (int)statusRgb, (int)statusDepth);
			}

			HwArImage_release(image);
			HwArImage_release(depthImage);
		}
		else
		{
			LOGE("Invalid intrinsics!");
		}
	}

	HwArCamera_release(ar_camera);

	if(pose.isNull())
	{
		LOGE("CameraAREngine: Pose is null");
	}
	else
	{
		pose = rtabmap::rtabmap_world_T_opengl_world * pose * rtabmap::opengl_world_T_rtabmap_world;
		this->poseReceived(pose);
        // adjust origin
        if(!getOriginOffset().isNull())
        {
            pose = getOriginOffset() * pose;
        }
		info->odomPose = pose;
	}
	return data;

}

void CameraAREngine::capturePoseOnly()
{
	UScopeMutex lock(arSessionMutex_);
	//LOGI("Capturing image...");

	SensorData data;
	if(!arSession_)
	{
		return;
	}

	// Update session to get current frame and render camera background.
	if (HwArSession_update(arSession_, arFrame_) != HWAR_SUCCESS) {
		LOGE("CameraARCore::captureImage() ArSession_update error");
		return;
	}

	HwArCamera* ar_camera;
	HwArFrame_acquireCamera(arSession_, arFrame_, &ar_camera);

	HwArTrackingState camera_tracking_state;
	HwArCamera_getTrackingState(arSession_, ar_camera, &camera_tracking_state);

	Transform pose;
	CameraModel model;
	if(camera_tracking_state == HWAR_TRACKING_STATE_TRACKING)
	{
		// pose in OpenGL coordinates
		float pose_raw[7];
		HwArCamera_getPose(arSession_, ar_camera, arPose_);
		HwArPose_getPoseRaw(arSession_, arPose_, pose_raw);
		pose = Transform(pose_raw[4], pose_raw[5], pose_raw[6], pose_raw[0], pose_raw[1], pose_raw[2], pose_raw[3]);
		if(!pose.isNull())
		{
			pose = rtabmap::rtabmap_world_T_opengl_world * pose * rtabmap::opengl_world_T_rtabmap_world;
			this->poseReceived(pose);
		}
	}

	HwArCamera_release(ar_camera);
}

} /* namespace rtabmap */
