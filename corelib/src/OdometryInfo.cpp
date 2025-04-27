/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/OdometryInfo.h"
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap {

OdometryInfo::OdometryInfo() :
	lost(true),
	features(0),
	localMapSize(0),
	localScanMapSize(0),
	localKeyFrames(0),
	localBundleOutliers(0),
	localBundleConstraints(0),
	localBundleTime(0),
	localBundleAvgInlierDistance(0.0f),
	localBundleMaxKeyFramesForInlier(0),
	keyFrameAdded(false),
	timeDeskewing(0.0f),
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

OdometryInfo OdometryInfo::copyWithoutData() const
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
	output.localBundleAvgInlierDistance = localBundleAvgInlierDistance;
	output.localBundleMaxKeyFramesForInlier = localBundleMaxKeyFramesForInlier;
	output.localBundleOutliersPerCam = localBundleOutliersPerCam;
	output.keyFrameAdded = keyFrameAdded;
	output.timeDeskewing = timeDeskewing;
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

std::map<std::string, float> OdometryInfo::statistics(const Transform & pose)
{
	std::map<std::string, float> stats;

	stats.insert(std::make_pair("Odometry/TimeRegistration/ms", reg.totalTime*1000.0f));
	stats.insert(std::make_pair("Odometry/RAM_usage/MB", memoryUsage));

	// Based on rtabmap/MainWindow.cpp
	stats.insert(std::make_pair("Odometry/Features/", features));
	stats.insert(std::make_pair("Odometry/Matches/", reg.matches));
	stats.insert(std::make_pair("Odometry/MatchesRatio/", features<=0?0.0f:float(reg.inliers)/float(features)));
	stats.insert(std::make_pair("Odometry/Inliers/", reg.inliers));
	stats.insert(std::make_pair("Odometry/InliersMeanDistance/m", reg.inliersMeanDistance));
	stats.insert(std::make_pair("Odometry/InliersDistribution/", reg.inliersDistribution));
	stats.insert(std::make_pair("Odometry/InliersRatio/", reg.inliers));
	for(size_t i=0; i<reg.matchesPerCam.size(); ++i)
	{
		stats.insert(std::make_pair(uFormat("Odometry/matchesCam%ld/", i), reg.matchesPerCam[i]));
	}
	for(size_t i=0; i<reg.inliersPerCam.size(); ++i)
	{
		stats.insert(std::make_pair(uFormat("Odometry/inliersCam%ld/", i), reg.inliersPerCam[i]));
	}
	if(reg.matchesPerCam.size() == reg.inliersPerCam.size())
	{
		for(size_t i=0; i<reg.matchesPerCam.size(); ++i)
		{
			stats.insert(std::make_pair(uFormat("Odometry/inliersRatioCam%ld/", i), reg.matchesPerCam[i]>0 ? (float)reg.inliersPerCam[i] / (float)reg.matchesPerCam[i] : 0.0f));
		}
	}
	stats.insert(std::make_pair("Odometry/ICPInliersRatio/", reg.icpInliersRatio));
	stats.insert(std::make_pair("Odometry/ICPRotation/rad", reg.icpRotation));
	stats.insert(std::make_pair("Odometry/ICPTranslation/m", reg.icpTranslation));
	stats.insert(std::make_pair("Odometry/ICPStructuralComplexity/", reg.icpStructuralComplexity));
	stats.insert(std::make_pair("Odometry/ICPStructuralDistribution/", reg.icpStructuralDistribution));
	stats.insert(std::make_pair("Odometry/ICPCorrespondences/", reg.icpCorrespondences));
	stats.insert(std::make_pair("Odometry/StdDevLin/", sqrt((float)reg.covariance.at<double>(0,0))));
	stats.insert(std::make_pair("Odometry/StdDevAng/", sqrt((float)reg.covariance.at<double>(5,5))));
	stats.insert(std::make_pair("Odometry/VarianceLin/", (float)reg.covariance.at<double>(0,0)));
	stats.insert(std::make_pair("Odometry/VarianceAng/", (float)reg.covariance.at<double>(5,5)));
	stats.insert(std::make_pair("Odometry/TimeEstimation/ms", timeEstimation*1000.0f));
	stats.insert(std::make_pair("Odometry/TimeFiltering/ms", timeParticleFiltering*1000.0f));
	stats.insert(std::make_pair("Odometry/LocalMapSize/", localMapSize));
	stats.insert(std::make_pair("Odometry/LocalScanMapSize/", localScanMapSize));
	stats.insert(std::make_pair("Odometry/LocalKeyFrames/", localKeyFrames));
	stats.insert(std::make_pair("Odometry/LocalBundleOutliers/", localBundleOutliers));
	stats.insert(std::make_pair("Odometry/LocalBundleConstraints/", localBundleConstraints));
	stats.insert(std::make_pair("Odometry/LocalBundleTime/ms", localBundleTime*1000.0f));
	stats.insert(std::make_pair("Odometry/localBundleAvgInlierDistance/pix", localBundleAvgInlierDistance));
	stats.insert(std::make_pair("Odometry/localBundleMaxKeyFramesForInlier/", localBundleMaxKeyFramesForInlier));
	for(size_t i=0; i<localBundleOutliersPerCam.size(); ++i)
	{
		stats.insert(std::make_pair(uFormat("Odometry/localBundleOutliersCam%ld/", i), localBundleOutliersPerCam[i]));
	}
	stats.insert(std::make_pair("Odometry/KeyFrameAdded/", keyFrameAdded?1.0f:0.0f));
	stats.insert(std::make_pair("Odometry/Interval/ms", (float)interval));
	stats.insert(std::make_pair("Odometry/Distance/m", distanceTravelled));

	float x,y,z,roll,pitch,yaw;
	if(!pose.isNull())
	{
		pose.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		stats.insert(std::make_pair("Odometry/Px/m", x));
		stats.insert(std::make_pair("Odometry/Py/m", y));
		stats.insert(std::make_pair("Odometry/Pz/m", z));
		stats.insert(std::make_pair("Odometry/Proll/deg", roll*180.0/CV_PI));
		stats.insert(std::make_pair("Odometry/Ppitch/deg", pitch*180.0/CV_PI));
		stats.insert(std::make_pair("Odometry/Pyaw/deg", yaw*180.0/CV_PI));
	}

	float dist = 0.0f, speed=0.0f;
	if(!transform.isNull())
	{
		transform.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		dist = transform.getNorm();
		stats.insert(std::make_pair("Odometry/T/m", dist));
		stats.insert(std::make_pair("Odometry/Tx/m", x));
		stats.insert(std::make_pair("Odometry/Ty/m", y));
		stats.insert(std::make_pair("Odometry/Tz/m", z));
		stats.insert(std::make_pair("Odometry/Troll/deg", roll*180.0/CV_PI));
		stats.insert(std::make_pair("Odometry/Tpitch/deg", pitch*180.0/CV_PI));
		stats.insert(std::make_pair("Odometry/Tyaw/deg", yaw*180.0/CV_PI));

		if(interval>0.0)
		{
			speed = dist/interval;
			stats.insert(std::make_pair("Odometry/Speed/kph", speed*3.6));
			stats.insert(std::make_pair("Odometry/Speed/mph", speed*2.237));
			stats.insert(std::make_pair("Odometry/Speed/mps", speed));
			stats.insert(std::make_pair("Odometry/Vx/mps", x/interval));
			stats.insert(std::make_pair("Odometry/Vy/mps", y/interval));
			stats.insert(std::make_pair("Odometry/Vz/mps", z/interval));
			stats.insert(std::make_pair("Odometry/Vroll/degps", (roll*180.0/CV_PI)/interval));
			stats.insert(std::make_pair("Odometry/Vpitch/degps", (pitch*180.0/CV_PI)/interval));
			stats.insert(std::make_pair("Odometry/Vyaw/degps", (yaw*180.0/CV_PI)/interval));
		}
	}
	if(!transformGroundTruth.isNull())
	{
		if(!transform.isNull())
		{
			rtabmap::Transform diff = transformGroundTruth.inverse()*transform;
			stats.insert(std::make_pair("Odometry/TG_error_lin/m", diff.getNorm()));
			stats.insert(std::make_pair("Odometry/TG_error_ang/deg", diff.getAngle()*180.0/CV_PI));
		}

		transformGroundTruth.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		dist = transformGroundTruth.getNorm();
		stats.insert(std::make_pair("Odometry/TG/m", dist));
		stats.insert(std::make_pair("Odometry/TGx/m", x));
		stats.insert(std::make_pair("Odometry/TGy/m", y));
		stats.insert(std::make_pair("Odometry/TGz/m", z));
		stats.insert(std::make_pair("Odometry/TGroll/deg", roll*180.0/CV_PI));
		stats.insert(std::make_pair("Odometry/TGpitch/deg", pitch*180.0/CV_PI));
		stats.insert(std::make_pair("Odometry/TGyaw/deg", yaw*180.0/CV_PI));

		if(interval>0.0)
		{
			speed = dist/interval;
			stats.insert(std::make_pair("Odometry/SpeedG/kph", speed*3.6));
			stats.insert(std::make_pair("Odometry/SpeedG/mph", speed*2.237));
			stats.insert(std::make_pair("Odometry/SpeedG/mps", speed));
		}
	}
	return stats;
}

} // namespace rtabmap
