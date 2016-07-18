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

#include "rtabmap/core/OdometryF2F.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Registration.h"
#include "rtabmap/core/EpipolarGeometry.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"

namespace rtabmap {

OdometryF2F::OdometryF2F(const ParametersMap & parameters) :
	Odometry(parameters),
	keyFrameThr_(Parameters::defaultOdomKeyFrameThr()),
	scanKeyFrameThr_(Parameters::defaultOdomScanKeyFrameThr())
{
	registrationPipeline_ = Registration::create(parameters);
	Parameters::parse(parameters, Parameters::kOdomKeyFrameThr(), keyFrameThr_);
	Parameters::parse(parameters, Parameters::kOdomScanKeyFrameThr(), scanKeyFrameThr_);
	UASSERT(keyFrameThr_>=0.0f && keyFrameThr_<=1.0f);
	UASSERT(scanKeyFrameThr_>=0.0f && scanKeyFrameThr_<=1.0f);
}

OdometryF2F::~OdometryF2F()
{
	delete registrationPipeline_;
}

void OdometryF2F::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	refFrame_ = Signature();
	lastKeyFramePose_.setNull();
}

// return not null transform if odometry is correctly computed
Transform OdometryF2F::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	UTimer timer;
	Transform output;
	if(!data.rightRaw().empty() && !data.stereoCameraModel().isValidForProjection())
	{
		UERROR("Calibrated stereo camera required");
		return output;
	}
	if(!data.depthRaw().empty() &&
		(data.cameraModels().size() != 1 || !data.cameraModels()[0].isValidForProjection()))
	{
		UERROR("Calibrated camera required (multi-cameras not supported).");
		return output;
	}

	RegistrationInfo regInfo;

	UASSERT(!this->getPose().isNull());
	if(lastKeyFramePose_.isNull())
	{
		lastKeyFramePose_ = this->getPose(); // reset to current pose
	}
	Transform motionSinceLastKeyFrame = lastKeyFramePose_.inverse()*this->getPose();

	Signature newFrame(data);
	if(refFrame_.sensorData().isValid())
	{
		Signature tmpRefFrame = refFrame_;
		output = registrationPipeline_->computeTransformationMod(
				tmpRefFrame,
				newFrame,
				!guess.isNull()?motionSinceLastKeyFrame*guess:Transform(),
				&regInfo);

		if(info && this->isInfoDataFilled())
		{
			std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
			EpipolarGeometry::findPairsUnique(tmpRefFrame.getWords(), newFrame.getWords(), pairs);
			info->refCorners.resize(pairs.size());
			info->newCorners.resize(pairs.size());
			std::map<int, int> idToIndex;
			int i=0;
			for(std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > >::iterator iter=pairs.begin();
				iter!=pairs.end();
				++iter)
			{
				info->refCorners[i] = iter->second.first.pt;
				info->newCorners[i] = iter->second.second.pt;
				idToIndex.insert(std::make_pair(iter->first, i));
				++i;
			}
			info->cornerInliers.resize(regInfo.inliersIDs.size(), 1);
			i=0;
			for(; i<(int)regInfo.inliersIDs.size(); ++i)
			{
				info->cornerInliers[i] = idToIndex.at(regInfo.inliersIDs[i]);
			}

			Transform t = this->getPose()*motionSinceLastKeyFrame.inverse();
			for(std::multimap<int, cv::Point3f>::const_iterator iter=tmpRefFrame.getWords3().begin(); iter!=tmpRefFrame.getWords3().end(); ++iter)
			{
				info->localMap.insert(std::make_pair(iter->first, util3d::transformPoint(iter->second, t)));
			}
			info->words = newFrame.getWords();
		}
	}
	else
	{
		//return Identity
		output = Transform::getIdentity();
		// a very high variance tells that the new pose is not linked with the previous one
		regInfo.variance = 9999;
	}

	if(!output.isNull())
	{
		output = motionSinceLastKeyFrame.inverse() * output;

		// new key-frame?
		if( (registrationPipeline_->isImageRequired() && (keyFrameThr_ == 0 || float(regInfo.inliers) <= keyFrameThr_*float(refFrame_.sensorData().keypoints().size()))) ||
			(registrationPipeline_->isScanRequired() && (scanKeyFrameThr_ == 0 || regInfo.icpInliersRatio <= scanKeyFrameThr_)))
		{
			UDEBUG("Update key frame");
			int features = newFrame.getWordsDescriptors().size();
			if(features == 0)
			{
				newFrame = Signature(data);
				// this will generate features only for the first frame or if optical flow was used (no 3d words)
				Signature dummy;
				registrationPipeline_->computeTransformationMod(
						newFrame,
						dummy);
				features = (int)newFrame.sensorData().keypoints().size();
			}

			if((features >= registrationPipeline_->getMinVisualCorrespondences()) &&
			   (registrationPipeline_->getMinGeometryCorrespondencesRatio()==0.0f ||
					   (newFrame.sensorData().laserScanRaw().cols &&
					   (newFrame.sensorData().laserScanMaxPts() == 0 || float(newFrame.sensorData().laserScanRaw().cols)/float(newFrame.sensorData().laserScanMaxPts())>=registrationPipeline_->getMinGeometryCorrespondencesRatio()))))
			{
				refFrame_ = newFrame;

				refFrame_.setWords(std::multimap<int, cv::KeyPoint>());
				refFrame_.setWords3(std::multimap<int, cv::Point3f>());
				refFrame_.setWordsDescriptors(std::multimap<int, cv::Mat>());

				//reset motion
				lastKeyFramePose_.setNull();
			}
			else
			{
				if (!refFrame_.sensorData().isValid())
				{
					// Don't send odometry if we don't have a keyframe yet
					output.setNull();
				}

				if(features < registrationPipeline_->getMinVisualCorrespondences())
				{
					UWARN("Too low 2D features (%d), keeping last key frame...", features);
				}

				if(registrationPipeline_->getMinGeometryCorrespondencesRatio()>0.0f && newFrame.sensorData().laserScanRaw().cols==0)
				{
					UWARN("Too low scan points (%d), keeping last key frame...", newFrame.sensorData().laserScanRaw().cols);
				}
				else if(registrationPipeline_->getMinGeometryCorrespondencesRatio()>0.0f && newFrame.sensorData().laserScanMaxPts() != 0 && float(newFrame.sensorData().laserScanRaw().cols)/float(newFrame.sensorData().laserScanMaxPts())<registrationPipeline_->getMinGeometryCorrespondencesRatio())
				{
					UWARN("Too low scan points ratio (%d < %d), keeping last key frame...", float(newFrame.sensorData().laserScanRaw().cols)/float(newFrame.sensorData().laserScanMaxPts()), registrationPipeline_->getMinGeometryCorrespondencesRatio());
				}
			}
		}
	}
	else if(!regInfo.rejectedMsg.empty())
	{
		UWARN("Registration failed: \"%s\"", regInfo.rejectedMsg.c_str());
	}

	data.setFeatures(newFrame.sensorData().keypoints(), newFrame.sensorData().descriptors());

	if(info)
	{
		info->type = 1;
		info->variance = regInfo.variance;
		info->inliers = regInfo.inliers;
		info->icpInliersRatio = regInfo.icpInliersRatio;
		info->matches = regInfo.matches;
		info->features = newFrame.sensorData().keypoints().size();
	}

	UINFO("Odom update time = %fs lost=%s inliers=%d, ref frame corners=%d, transform accepted=%s",
			timer.elapsed(),
			output.isNull()?"true":"false",
			(int)regInfo.inliers,
			(int)newFrame.sensorData().keypoints().size(),
			!output.isNull()?"true":"false");

	return output;
}

} // namespace rtabmap
