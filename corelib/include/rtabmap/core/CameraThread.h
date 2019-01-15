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

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsSender.h>

namespace clams
{
class DiscreteDepthDistortionModel;
}

namespace rtabmap
{

class Camera;
class CameraInfo;
class SensorData;
class StereoDense;

/**
 * Class CameraThread
 *
 */
class RTABMAP_EXP CameraThread :
	public UThread,
	public UEventsSender
{
public:
	// ownership transferred
	CameraThread(Camera * camera, const ParametersMap & parameters = ParametersMap());
	virtual ~CameraThread();

	void setMirroringEnabled(bool enabled) {_mirroring = enabled;}
	void setStereoExposureCompensation(bool enabled) {_stereoExposureCompensation = enabled;}
	void setColorOnly(bool colorOnly) {_colorOnly = colorOnly;}
	void setImageDecimation(int decimation) {_imageDecimation = decimation;}
	void setStereoToDepth(bool enabled) {_stereoToDepth = enabled;}
	void setImageRate(float imageRate);
	void setDistortionModel(const std::string & path);
	void enableBilateralFiltering(float sigmaS, float sigmaR);
	void disableBilateralFiltering() {_bilateralFiltering = false;}

	void setScanParameters(
			bool fromDepth,
			int downsampleStep=1, // decimation of the depth image in case the scan is from depth image
			float rangeMin=0.0f,
			float rangeMax=0.0f,
			float voxelSize = 0.0f,
			int normalsK = 0,
			int normalsRadius = 0.0f,
			bool forceGroundNormalsUp = false)
	{
		_scanFromDepth = fromDepth;
		_scanDownsampleStep=downsampleStep;
		_scanRangeMin = rangeMin;
		_scanRangeMax = rangeMax;
		_scanVoxelSize = voxelSize;
		_scanNormalsK = normalsK;
		_scanNormalsRadius = normalsRadius;
		_scanForceGroundNormalsUp = forceGroundNormalsUp;
	}

	void postUpdate(SensorData * data, CameraInfo * info = 0) const;

	//getters
	bool isPaused() const {return !this->isRunning();}
	bool isCapturing() const {return this->isRunning();}

	Camera * camera() {return _camera;} // return null if not set, valid until CameraThread is deleted

private:
	virtual void mainLoopBegin();
	virtual void mainLoop();
	virtual void mainLoopKill();

private:
	Camera * _camera;
	bool _mirroring;
	bool _stereoExposureCompensation;
	bool _colorOnly;
	int _imageDecimation;
	bool _stereoToDepth;
	bool _scanFromDepth;
	int _scanDownsampleStep;
	float _scanRangeMin;
	float _scanRangeMax;
	float _scanVoxelSize;
	int _scanNormalsK;
	float _scanNormalsRadius;
	bool _scanForceGroundNormalsUp;
	StereoDense * _stereoDense;
	clams::DiscreteDepthDistortionModel * _distortionModel;
	bool _bilateralFiltering;
	float _bilateralSigmaS;
	float _bilateralSigmaR;
};

} // namespace rtabmap
