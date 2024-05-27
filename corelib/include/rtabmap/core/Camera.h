/*
Copyright (c) 2010-2022, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include <rtabmap/core/SensorCapture.h>
#include <rtabmap/core/IMU.h>

namespace rtabmap
{

class IMUFilter;

/**
 * Class Camera
 *
 */
class RTABMAP_CORE_EXPORT Camera : public SensorCapture
{
public:
	virtual ~Camera();

	SensorData takeImage(SensorCaptureInfo * info = 0) {return takeData(info);}
	float getImageRate() const {return getFrameRate();}
	void setImageRate(float imageRate) {setFrameRate(imageRate);}
	void setInterIMUPublishing(bool enabled, IMUFilter * filter = 0); // Take ownership of filter
	bool isInterIMUPublishing() const {return publishInterIMU_;}

	bool initFromFile(const std::string & calibrationPath);
	virtual bool isCalibrated() const = 0;

protected:
	/**
	 * Constructor
	 *
	 * @param imageRate the frame rate (Hz), 0 for fast as the camera can
	 * @param localTransform the transform from base frame to camera frame (without optical rotation)
	 */
	Camera(float imageRate = 0, const Transform & localTransform = Transform::getIdentity());

	virtual SensorData captureImage(SensorCaptureInfo * info = 0) = 0;

	void postInterIMU(const IMU & imu, double stamp);

private:
	virtual SensorData captureData(SensorCaptureInfo * info = 0) {return captureImage(info);}

private:
	IMUFilter * imuFilter_;
	bool publishInterIMU_;
};


} // namespace rtabmap
