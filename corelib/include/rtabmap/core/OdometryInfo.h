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

#ifndef ODOMETRYINFO_H_
#define ODOMETRYINFO_H_

#include <map>
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/RegistrationInfo.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/LaserScan.h"
#include <opencv2/features2d/features2d.hpp>

namespace rtabmap {

class OdometryInfo
{
public:
	OdometryInfo() :
		lost(true),
		features(0),
		localMapSize(0),
		localScanMapSize(0),
		localKeyFrames(0),
		localBundleOutliers(0),
		localBundleConstraints(0),
		localBundleTime(0),
		keyFrameAdded(false),
		timeEstimation(0.0f),
		timeParticleFiltering(0.0f),
		stamp(0),
		interval(0),
		distanceTravelled(0.0f),
		memoryUsage(0),
		gravityRollError(0.0),
		gravityPitchError(0.0),
		type(0)
	{}

	OdometryInfo copyWithoutData() const
	{
		OdometryInfo output;
		output.lost = lost;
		output.reg = reg.copyWithoutData();
		output.features = features;
		output.localMapSize = localMapSize;
		output.localScanMapSize = localScanMapSize;
		output.localKeyFrames = localKeyFrames;
		output.localBundleOutliers = localBundleOutliers;
		output.localBundleConstraints = localBundleConstraints;
		output.localBundleTime = localBundleTime;
		output.localBundlePoses = localBundlePoses;
		output.localBundleModels = localBundleModels;
		output.keyFrameAdded = keyFrameAdded;
		output.timeEstimation = timeEstimation;
		output.timeParticleFiltering = timeParticleFiltering;
		output.stamp = stamp;
		output.interval = interval;
		output.transform = transform;
		output.transformFiltered = transformFiltered;
		output.transformGroundTruth = transformGroundTruth;
		output.guessVelocity = guessVelocity;
		output.guess = guess;
		output.distanceTravelled = distanceTravelled;
		output.memoryUsage = memoryUsage;
		output.gravityRollError = gravityRollError;
		output.gravityPitchError = gravityPitchError;
		output.type = type;
		return output;
	}

	bool lost;
	RegistrationInfo reg;
	int features;
	int localMapSize;
	int localScanMapSize;
	int localKeyFrames;
	int localBundleOutliers;
	int localBundleConstraints;
	float localBundleTime;
	std::map<int, Transform> localBundlePoses;
	std::map<int, std::vector<CameraModel> > localBundleModels;
	bool keyFrameAdded;
	float timeEstimation;
	float timeParticleFiltering;
	double stamp;
	double interval;
	Transform transform;
	Transform transformFiltered;
	Transform transformGroundTruth;
	Transform guessVelocity; // deprecated, will be removed. Use guess and interval instead.
	Transform guess;
	float distanceTravelled;
	int memoryUsage; //MB
	double gravityRollError;
	double gravityPitchError;

	int type;

	// F2M
	std::multimap<int, cv::KeyPoint> words;
	std::map<int, cv::Point3f> localMap;
	LaserScan localScanMap;

	// F2F
	std::vector<cv::Point2f> refCorners;
	std::vector<cv::Point2f> newCorners;
	std::vector<int> cornerInliers;
};

}

#endif /* ODOMETRYINFO_H_ */
