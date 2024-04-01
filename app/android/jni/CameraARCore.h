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

#ifndef CAMERAARCORE_H_
#define CAMERAARCORE_H_

#include "CameraMobile.h"
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/GeodeticCoords.h>
#include <rtabmap/utilite/UMutex.h>
#include <rtabmap/utilite/USemaphore.h>
#include <rtabmap/utilite/UEventsSender.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEvent.h>
#include <rtabmap/utilite/UTimer.h>
#include <boost/thread/mutex.hpp>
#include <background_renderer.h>

#include <arcore_c_api.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <media/NdkImageReader.h>
#include <android/native_window.h>

namespace rtabmap {

class CameraARCore : public CameraMobile {
public:
	static LaserScan scanFromPointCloudData(
			const float * pointCloudData,
			int points,
			const Transform & pose,
			const CameraModel & model,
			const cv::Mat & rgb,
			std::vector<cv::KeyPoint> * kpts = 0,
			std::vector<cv::Point3f> * kpts3D = 0);

public:
	CameraARCore(void* env, void* context, void* activity, bool depthFromMotion = false, bool smoothing = false);
	virtual ~CameraARCore();

	bool uvsInitialized() const {return uvs_initialized_;}
	const float* uvsTransformed() const {return transformed_uvs_;}
	void getVPMatrices(glm::mat4 & view, glm::mat4 & projection) const {view=viewMatrix_; projection=projectionMatrix_;}

	virtual void setScreenRotationAndSize(ScreenRotation colorCameraToDisplayRotation, int width, int height);

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	void setupGL();
	virtual void close(); // close Tango connection
	virtual std::string getSerial() const;
	GLuint getTextureId() const {return textureId_;}

	void imageCallback(AImageReader *reader);

protected:
	virtual SensorData captureImage(CameraInfo * info = 0); // should be called in opengl thread
	virtual void capturePoseOnly();

private:
	rtabmap::Transform getPoseAtTimestamp(double timestamp);

private:
	void * env_;
	void * context_;
	void * activity_;
	ArSession* arSession_ = nullptr;
	ArConfig* arConfig_ = nullptr;
	ArFrame* arFrame_ = nullptr;
	ArCameraIntrinsics *arCameraIntrinsics_ = nullptr;
	ArPose * arPose_ = nullptr;
	bool arInstallRequested_;
	UMutex arSessionMutex_;

	bool depthFromMotion_;
};

} /* namespace rtabmap */
#endif /* CAMERAARCORE_H_ */
