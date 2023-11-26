/*
Copyright (c) 2010-2022, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither names of its contributors may be used to endorse or promote products
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

#include <opencv2/highgui/highgui.hpp>
#include <rtabmap/core/SensorCaptureInfo.h>
#include "rtabmap/core/SensorData.h"
#include <set>
#include <stack>
#include <list>
#include <vector>

class UDirectory;
class UTimer;

namespace rtabmap
{

/**
 * Class Camera
 *
 */
class RTABMAP_CORE_EXPORT SensorCapture
{
public:
	virtual ~SensorCapture();
	SensorData takeData(SensorCaptureInfo * info = 0);

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "") = 0;
	virtual std::string getSerial() const = 0;
	virtual bool odomProvided() const { return false; }
	virtual bool getPose(double stamp, Transform & pose, cv::Mat & covariance, double maxWaitTime = 0.06) { return false; }

	//getters
	float getFrameRate() const {return _frameRate;}
	const Transform & getLocalTransform() const {return _localTransform;}

	//setters
	void setFrameRate(float frameRate) {_frameRate = frameRate;}
	void setLocalTransform(const Transform & localTransform) {_localTransform= localTransform;}

	void resetTimer();
protected:
	/**
	 * Constructor
	 *
	 * @param frameRate the frame rate (Hz), 0 for fast as the sensor can
	 * @param localTransform the transform from base frame to sensor frame
	 */
	SensorCapture(float frameRate = 0, const Transform & localTransform = Transform::getIdentity());

	/**
	 * returned rgb and depth images should be already rectified if calibration was loaded
	 */
	virtual SensorData captureData(SensorCaptureInfo * info = 0) = 0;

	int getNextSeqID() {return ++_seq;}

private:
	float _frameRate;
	Transform _localTransform;
	UTimer * _frameRateTimer;
	int _seq;
};


} // namespace rtabmap
