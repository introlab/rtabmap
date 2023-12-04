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

#pragma once

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsSender.h>

namespace clams
{
class DiscreteDepthDistortionModel;
}

namespace rtabmap
{

class Camera;
class Lidar;
class SensorCapture;
class SensorCaptureInfo;
class SensorData;
class StereoDense;
class IMUFilter;
class Feature2D;

/**
 * Class CameraThread
 *
 */
class RTABMAP_CORE_EXPORT SensorCaptureThread :
	public UThread,
	public UEventsSender
{
public:
	// ownership transferred
	SensorCaptureThread(
			Camera * camera,
			const ParametersMap & parameters = ParametersMap());
	/**
	 * @param camera the camera to take images from
	 * @param odomSensor an odometry sensor to get a pose (can be again the camera)
	 * @param odomAsGt set odometry sensor pose as ground truth instead of odometry
	 * @param extrinsics the static transform between odometry sensor's left lens frame to camera's left lens frame  (without optical rotation)
	 */
	SensorCaptureThread(
			Camera * camera,
			SensorCapture * odomSensor,
			const Transform & extrinsics,
			double poseTimeOffset = 0.0,
			float poseScaleFactor = 1.0f,
			double poseWaitTime = 0.1,
			const ParametersMap & parameters = ParametersMap());
	/**
	 * @param lidar the lidar to take scans from
	 */
	SensorCaptureThread(
			Lidar * lidar,
			const ParametersMap & parameters = ParametersMap());
	/**
	 * @param lidar the lidar to take scans from
	 * @param camera the camera to take images from. If the camera is providing a pose, it can be used for deskewing
	 */
	SensorCaptureThread(
			Lidar * lidar,
			Camera * camera,
			const ParametersMap & parameters = ParametersMap());
	/**
	 * @param lidar the lidar to take scans from
	 * @param odomSensor an odometry sensor to get a pose and used for deskewing (can be again the lidar)
	 */
	SensorCaptureThread(
			Lidar * lidar,
			SensorCapture * odomSensor,
			double poseTimeOffset = 0.0,
			float poseScaleFactor = 1.0f,
			double poseWaitTime = 0.1,
			const ParametersMap & parameters = ParametersMap());
	/**
	 * @param lidar the lidar to take scans from
	 * @param camera the camera to take images from
	 * @param odomSensor an odometry sensor to get a pose and used for deskewing (can be again the camera or lidar)
	 * @param extrinsics the static transform between odometry frame to camera frame (without optical rotation)
	 */
	SensorCaptureThread(
			Lidar * lidar,
			Camera * camera,
			SensorCapture * odomSensor,
			const Transform & extrinsics,
			double poseTimeOffset = 0.0,
			float poseScaleFactor = 1.0f,
			double poseWaitTime = 0.1,
			const ParametersMap & parameters = ParametersMap());
	virtual ~SensorCaptureThread();

	void setMirroringEnabled(bool enabled) {_mirroring = enabled;}
	void setStereoExposureCompensation(bool enabled) {_stereoExposureCompensation = enabled;}
	void setColorOnly(bool colorOnly) {_colorOnly = colorOnly;}
	void setImageDecimation(int decimation) {_imageDecimation = decimation;}
	void setHistogramMethod(int histogramMethod) {_histogramMethod = histogramMethod;}
	void setStereoToDepth(bool enabled) {_stereoToDepth = enabled;}
	void setFrameRate(float frameRate);
	RTABMAP_DEPRECATED void setImageRate(float frameRate) {setFrameRate(frameRate);}
	void setDistortionModel(const std::string & path);
	void setOdomAsGroundTruth(bool enabled) {_odomAsGt = enabled;}
	void enableBilateralFiltering(float sigmaS, float sigmaR);
	void disableBilateralFiltering() {_bilateralFiltering = false;}
	void enableIMUFiltering(int filteringStrategy=1, const ParametersMap & parameters = ParametersMap(), bool baseFrameConversion = false);
	void disableIMUFiltering();
	void enableFeatureDetection(const ParametersMap & parameters = ParametersMap());
	void disableFeatureDetection();

	// Use new version of this function with groundNormalsUp=0.8 for forceGroundNormalsUp=True and groundNormalsUp=0.0 for forceGroundNormalsUp=False.
	RTABMAP_DEPRECATED void setScanParameters(
			bool fromDepth,
			int downsampleStep, // decimation of the depth image in case the scan is from depth image
			float rangeMin,
			float rangeMax,
			float voxelSize,
			int normalsK,
			float normalsRadius,
			bool forceGroundNormalsUp,
			bool deskewing);
	void setScanParameters(
			bool fromDepth,
			int downsampleStep=1, // decimation of the depth image in case the scan is from depth image
			float rangeMin=0.0f,
			float rangeMax=0.0f,
			float voxelSize = 0.0f,
			int normalsK = 0,
			float normalsRadius = 0.0f,
			float groundNormalsUp = 0.0f,
			bool deskewing = false);

	void postUpdate(SensorData * data, SensorCaptureInfo * info = 0) const;

	//getters
	bool isPaused() const {return !this->isRunning();}
	bool isCapturing() const {return this->isRunning();}
	bool odomProvided() const;

	Camera * camera() {return _camera;} // return null if not set, valid until CameraThread is deleted
	SensorCapture * odomSensor() {return _odomSensor;} // return null if not set, valid until CameraThread is deleted
	Lidar * lidar() {return _lidar;} // return null if not set, valid until CameraThread is deleted

private:
	virtual void mainLoopBegin();
	virtual void mainLoop();
	virtual void mainLoopKill();

private:
	Camera * _camera;
	SensorCapture * _odomSensor;
	Lidar * _lidar;
	Transform _extrinsicsOdomToCamera;
	bool _odomAsGt;
	double _poseTimeOffset;
	float _poseScaleFactor;
	double _poseWaitTime;
	bool _mirroring;
	bool _stereoExposureCompensation;
	bool _colorOnly;
	int _imageDecimation;
	int _histogramMethod;
	bool _stereoToDepth;
	bool _scanDeskewing;
	bool _scanFromDepth;
	int _scanDownsampleStep;
	float _scanRangeMin;
	float _scanRangeMax;
	float _scanVoxelSize;
	int _scanNormalsK;
	float _scanNormalsRadius;
	float _scanForceGroundNormalsUp;
	StereoDense * _stereoDense;
	clams::DiscreteDepthDistortionModel * _distortionModel;
	bool _bilateralFiltering;
	float _bilateralSigmaS;
	float _bilateralSigmaR;
	IMUFilter * _imuFilter;
	bool _imuBaseFrameConversion;
	Feature2D * _featureDetector;
	bool _depthAsMask;
};

//backward compatibility
RTABMAP_DEPRECATED typedef SensorCaptureThread CameraThread;

} // namespace rtabmap
