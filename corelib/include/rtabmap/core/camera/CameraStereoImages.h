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

#include <rtabmap/core/camera/CameraImages.h>

#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/Version.h"

namespace rtabmap
{

class CameraImages;
class RTABMAP_CORE_EXPORT CameraStereoImages :
	public CameraImages
{
public:
	static bool available();

public:
	CameraStereoImages(
			const std::string & pathLeftImages,
			const std::string & pathRightImages,
			bool rectifyImages = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity());
	CameraStereoImages(
			const std::string & pathLeftRightImages,
			bool rectifyImages = false,
			float imageRate=0.0f,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraStereoImages();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

	virtual void setStartIndex(int index) {CameraImages::setStartIndex(index);camera2_->setStartIndex(index);} // negative means last
	virtual void setMaxFrames(int value) {CameraImages::setMaxFrames(value);camera2_->setMaxFrames(value);}

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
	CameraImages * camera2_;
	StereoCameraModel stereoModel_;
};


} // namespace rtabmap
