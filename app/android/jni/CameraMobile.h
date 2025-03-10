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

#ifndef CAMERAMOBILE_H_
#define CAMERAMOBILE_H_

#include <rtabmap/core/Camera.h>
#include <rtabmap/core/GeodeticCoords.h>
#include <rtabmap/utilite/UMutex.h>
#include <rtabmap/utilite/USemaphore.h>
#include <rtabmap/utilite/UEventsSender.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEvent.h>
#include <rtabmap/utilite/UTimer.h>
#include <boost/thread/mutex.hpp>
#include "util.h"

namespace rtabmap {

class CameraInfoEvent: public UEvent
{
public:
	CameraInfoEvent(int type, const std::string & key, const std::string & value) : type_(type), key_(key), value_(value) {}
	virtual std::string getClassName() const {return "CameraInfoEvent";}
	int type() const {return type_;}
	const std::string & key() const {return key_;}
	const std::string & value() const {return value_;}

private:
	int type_;
	std::string key_;
	std::string value_;

};

class PoseEvent: public UEvent
{
public:
	PoseEvent(const Transform & pose) : pose_(pose) {}
	virtual std::string getClassName() const {return "PoseEvent";}
	const Transform & pose() const {return pose_;}

private:
	Transform pose_;
};

class CameraMobile : public Camera, public UEventsSender {
public:
	static const float bilateralFilteringSigmaS;
	static const float bilateralFilteringSigmaR;

	static const rtabmap::Transform opticalRotation;
	static const rtabmap::Transform opticalRotationInv;
    
public:
    static LaserScan scanFromPointCloudData(
            const cv::Mat & pointCloudData,
            const Transform & pose,
            const CameraModel & model,
            const cv::Mat & rgb,
            std::vector<cv::KeyPoint> * kpts = 0,
            std::vector<cv::Point3f> * kpts3D = 0,
            int kptsSize = 3);

public:
	CameraMobile(bool smoothing = false, float upstreamRelocalizationAccThr = 0.0f);
	virtual ~CameraMobile();

	// abstract functions
	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual void close(); // inherited classes should call its parent at the end of their close().
	virtual std::string getSerial() const {return "CameraMobile";}

	// original pose of device in rtabmap frame (without origin offset), stamp of the device (may be not epoch), viewMatrix in opengl frame (without origin offset)
	void update(const SensorData & data, const Transform & pose, const glm::mat4 & viewMatrix, const glm::mat4 & projectionMatrix, const float * texCoord);
	void updateOnRender();

	void resetOrigin(const rtabmap::Transform & offset = rtabmap::Transform());
	virtual bool isCalibrated() const;

	virtual bool odomProvided() const { return true; }
	virtual bool getPose(double epochStamp, Transform & pose, cv::Mat & covariance, double maxWaitTime = 0.06); // Return pose of device in rtabmap frame (with origin offset), stamp should be epoch time
	// original pose of device in rtabmap frame (without origin offset), stamp of the device (may be not epoch)
	void poseReceived(const Transform & pose, double deviceStamp);
	double getStampEpochOffset() const {return stampEpochOffset_;}

	const CameraModel & getCameraModel() const {return model_;}
	const Transform & getDeviceTColorCamera() const {return deviceTColorCamera_;}
	void setSmoothing(bool enabled) {smoothing_ = enabled;}
	virtual void setScreenRotationAndSize(ScreenRotation colorCameraToDisplayRotation, int width, int height) {colorCameraToDisplayRotation_ = colorCameraToDisplayRotation;}
	void setGPS(const GPS & gps);
	void addEnvSensor(int type, float value);

    GLuint getTextureId() {return textureId_;}
    bool uvsInitialized() const {return uvs_initialized_;}
    const float* uvsTransformed() const {return transformed_uvs_;}
    void getVPMatrices(glm::mat4 & view, glm::mat4 & projection) const {view=viewMatrix_; projection=projectionMatrix_;}
    ScreenRotation getScreenRotation() const {return colorCameraToDisplayRotation_;}
    
    void setOcclusionImage(const cv::Mat & image, const CameraModel & model) {occlusionModel_ = model; occlusionImage_ = image;}
    const cv::Mat & getOcclusionImage(CameraModel * model=0) const {if(model)*model=occlusionModel_; return occlusionImage_; }

protected:
	virtual SensorData updateDataOnRender(Transform & pose);

private:
	virtual SensorData captureImage(SensorCaptureInfo * info = 0);
	void postUpdate(); // Should be called while being protected by dataMutex_

protected:
	CameraModel model_; // local transform is the device to camera optical rotation in rtabmap frame
	Transform deviceTColorCamera_; // device to camera optical rotation in rtabmap frame
    
    GLuint textureId_;
    glm::mat4 viewMatrix_;
    glm::mat4 projectionMatrix_;
    float transformed_uvs_[8];
    bool uvs_initialized_ = false;

private:
	bool firstFrame_;
	double stampEpochOffset_;
	bool smoothing_;
	ScreenRotation colorCameraToDisplayRotation_;
	GPS lastKnownGPS_;
	EnvSensors lastEnvSensors_;
	Transform originOffset_;
	bool originUpdate_;
    rtabmap::Transform manualOriginOffset_;
	float upstreamRelocalizationAccThr_;
	rtabmap::Transform previousAnchorPose_;
	std::vector<float> previousAnchorLinearVelocity_;
	double previousAnchorStamp_;
    std::map<double, std::pair<rtabmap::Transform, rtabmap::Transform> > relocalizationDebugBuffer_;

	USemaphore dataReady_;
	UMutex dataMutex_;
	SensorData data_;
	Transform dataPose_;
    bool dataGoodTracking_;

	UMutex poseMutex_;
	std::map<double, Transform> poseBuffer_; // <stamp, Pose>
    
    cv::Mat occlusionImage_;
    CameraModel occlusionModel_;
};

} /* namespace rtabmap */
#endif /* CAMERATANGO_H_ */
