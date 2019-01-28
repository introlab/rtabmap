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

#include <string>

namespace rtabmap
{

class CameraInfo
{

public:
	CameraInfo() :
		cameraName(""),
		id(0),
		stamp(0.0),
		timeCapture(0.0f),
		timeDisparity(0.0f),
		timeMirroring(0.0f),
		timeStereoExposureCompensation(0.0f),
		timeImageDecimation(0.0f),
		timeScanFromDepth(0.0f),
		timeUndistortDepth(0.0f),
		timeBilateralFiltering(0.0f),
		timeTotal(0.0f),
		odomCovariance(cv::Mat::eye(6,6,CV_64FC1))
	{
	}
	virtual ~CameraInfo() {}

	std::string cameraName;
	int id;
	double stamp;
	float timeCapture;
	float timeDisparity;
	float timeMirroring;
	float timeStereoExposureCompensation;
	float timeImageDecimation;
	float timeScanFromDepth;
	float timeUndistortDepth;
	float timeBilateralFiltering;
	float timeTotal;
	Transform odomPose;
	cv::Mat odomCovariance;
	std::vector<float> odomVelocity;
};

} // namespace rtabmap
