/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"

#include <memory>
#include <string>

namespace einstar
{
class Device;
class StereoDepth;
}

namespace rtabmap
{

/**
 * SHINING 3D Einstar handheld scanner, through the open-source libeinstar
 * userspace driver.
 *
 * Produces RGB-D frames: depth is computed from the IR stereo pair (the
 * projected speckle moves with the scanner, so it is used for depth only)
 * and registered into the undistorted colour image, which drives odometry.
 *
 * Calibration comes from the scanner itself when calibrationPath is empty,
 * otherwise from an EXStar project camera model (*.data_cm), EXStar's
 * calibration cache directory (TexCCF.txt, LeftCCF.txt, RightCCF.txt) or a
 * custom-info dump from einstar-probe. A replay directory of libeinstar captures
 * (frame_<id>_cam<n>.pgm) can be used instead of a live device.
 */
class RTABMAP_CORE_EXPORT CameraEinstar :
	public Camera
{
public:
	static bool available();

public:
	CameraEinstar(
		const std::string & calibrationPath = "",
		const std::string & replayDir = "",
		float imageRate = 0.0f,
		const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraEinstar();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

	// Working range used for stereo matching (mm).
	void setDepthRange(float minDepthMm, float maxDepthMm);

protected:
	virtual SensorData captureImage(SensorCaptureInfo * info = 0);

private:
	struct Impl;
	std::string calibrationPath_;
	std::string replayDir_;
	float minDepthMm_;
	float maxDepthMm_;
#ifdef RTABMAP_EINSTAR
	std::unique_ptr<Impl> impl_;
#endif
};

} // namespace rtabmap
