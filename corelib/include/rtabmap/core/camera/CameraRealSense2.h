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

#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"

#include <pcl/pcl_config.h>


namespace rs2
{
	class context;
	class device;
	class syncer;
}
struct rs2_intrinsics;
struct rs2_extrinsics;

namespace rtabmap
{

class RTABMAP_EXP CameraRealSense2 :
	public Camera
{
public:
	static bool available();

public:
	// default local transform z in, x right, y down));
	CameraRealSense2(
		const std::string & deviceId = "",
		float imageRate = 0,
		const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraRealSense2();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

	// parameters are set during initialization
	void setEmitterEnabled(bool enabled);
	void setIRDepthFormat(bool enabled);

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_REALSENSE2
	rs2::context * ctx_;
	rs2::device * dev_;
	std::string deviceId_;
	rs2::syncer * syncer_;
	float depth_scale_meters_;
	rs2_intrinsics * depthIntrinsics_;
	rs2_intrinsics * rgbIntrinsics_;
	rs2_extrinsics * depthToRGBExtrinsics_;
	cv::Mat depthBuffer_;
	cv::Mat rgbBuffer_;
	CameraModel model_;

	bool emitterEnabled_;
	bool irDepth_;
#endif
};


} // namespace rtabmap
