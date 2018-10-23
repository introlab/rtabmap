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

#ifndef CAMERATANGO_H_
#define CAMERATANGO_H_

#include <rtabmap/core/Camera.h>
#include <rtabmap/core/GeodeticCoords.h>
#include <rtabmap/utilite/UMutex.h>
#include <rtabmap/utilite/USemaphore.h>
#include <rtabmap/utilite/UEventsSender.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEvent.h>
#include <rtabmap/utilite/UTimer.h>
#include <boost/thread/mutex.hpp>
#include <tango_support_api.h>

class TangoPoseData;

namespace rtabmap {

class PoseEvent: public UEvent
{
public:
	PoseEvent(const Transform & pose) : pose_(pose) {}
	virtual std::string getClassName() const {return "PoseEvent";}
	const Transform & pose() const {return pose_;}

private:
	Transform pose_;
};

class CameraTangoEvent: public UEvent
{
public:
	CameraTangoEvent(int type, const std::string & key, const std::string & value) : type_(type), key_(key), value_(value) {}
	virtual std::string getClassName() const {return "CameraTangoEvent";}
	int type() const {return type_;}
	const std::string & key() const {return key_;}
	const std::string & value() const {return value_;}

private:
	int type_;
	std::string key_;
	std::string value_;

};

class CameraTango : public Camera, public UThread, public UEventsSender {
public:
	static const float bilateralFilteringSigmaS;
	static const float bilateralFilteringSigmaR;

public:
	CameraTango(bool colorCamera, int decimation, bool publishRawScan, bool smoothing);
	virtual ~CameraTango();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	void close(); // close Tango connection
	void resetOrigin();
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;
	const CameraModel & getCameraModel() const {return model_;}
	rtabmap::Transform tangoPoseToTransform(const TangoPoseData * tangoPose) const;
	void setColorCamera(bool enabled) {if(!this->isRunning()) colorCamera_ = enabled;}
	void setDecimation(int value) {decimation_ = value;}
	void setSmoothing(bool enabled) {smoothing_ = enabled;}
	void setRawScanPublished(bool enabled) {rawScanPublished_ = enabled;}
	void setScreenRotation(TangoSupportRotation colorCameraToDisplayRotation) {colorCameraToDisplayRotation_ = colorCameraToDisplayRotation;}
	void setGPS(const GPS & gps);
	void addEnvSensor(int type, float value);

	void cloudReceived(const cv::Mat & cloud, double timestamp);
	void rgbReceived(const cv::Mat & tangoImage, int type, double timestamp);
	void poseReceived(const Transform & pose);
	void tangoEventReceived(int type, const char * key, const char * value);

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	rtabmap::Transform getPoseAtTimestamp(double timestamp);

	virtual void mainLoopBegin();
	virtual void mainLoop();

private:
	void * tango_config_;
	Transform previousPose_;
	double previousStamp_;
	UTimer cameraStartedTime_;
	double stampEpochOffset_;
	bool colorCamera_;
	int decimation_;
	bool rawScanPublished_;
	bool smoothing_;
	cv::Mat cloud_;
	double cloudStamp_;
	cv::Mat tangoColor_;
	int tangoColorType_;
	double tangoColorStamp_;
	boost::mutex dataMutex_;
	USemaphore dataReady_;
	CameraModel model_;
	Transform deviceTColorCamera_;
	TangoSupportRotation colorCameraToDisplayRotation_;
	cv::Mat fisheyeRectifyMapX_;
	cv::Mat fisheyeRectifyMapY_;
	GPS lastKnownGPS_;
	EnvSensors lastEnvSensors_;
	Transform originOffset_;
	bool originUpdate_;
};

} /* namespace rtabmap */
#endif /* CAMERATANGO_H_ */
