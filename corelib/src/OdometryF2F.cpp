/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"

namespace rtabmap {

OdometryF2F::OdometryF2F(const ParametersMap & parameters) :
	Odometry(parameters),
	keyFrameThr_(Parameters::defaultOdomF2FKeyFrameThr()),
	guessFromMotion_(Parameters::defaultOdomF2FGuessMotion()),
	motionSinceLastKeyFrame_(Transform::getIdentity())
{
	registrationPipeline_ = Registration::create(parameters);
	Parameters::parse(parameters, Parameters::kOdomF2FKeyFrameThr(), keyFrameThr_);
	Parameters::parse(parameters, Parameters::kOdomF2FGuessMotion(), guessFromMotion_);
}

OdometryF2F::~OdometryF2F()
{
	delete registrationPipeline_;
}

void OdometryF2F::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	refFrame_ = Signature();
	motionSinceLastKeyFrame_.setIdentity();
}

// return not null transform if odometry is correctly computed
Transform OdometryF2F::computeTransform(
		const SensorData & data,
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

	Signature newFrame(data);
	if(refFrame_.sensorData().isValid())
	{
		output = registrationPipeline_->computeTransformationMod(
				refFrame_,
				newFrame,
				guessFromMotion_?motionSinceLastKeyFrame_*this->previousTransform():Transform(),
				&regInfo);

		if(info && this->isInfoDataFilled())
		{
			std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
			EpipolarGeometry::findPairsUnique(refFrame_.getWords(), newFrame.getWords(), pairs);
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

		}
	}
	else
	{
		//return Identity
		output = Transform::getIdentity();
	}

	if(!output.isNull())
	{
		output = motionSinceLastKeyFrame_.inverse() * output;
		motionSinceLastKeyFrame_ *= output;

		// new key-frame?
		if(keyFrameThr_ <= 0 || (int)regInfo.inliers <= keyFrameThr_)
		{
			UDEBUG("Update key frame");
			Signature newRefFrame(data);

			int features = 0;
			if(registrationPipeline_->getMinVisualCorrespondences()>0)
			{
				// this will generate features only for the first frame
				Signature dummy;
				registrationPipeline_->computeTransformationMod(
						newRefFrame,
						dummy);
				features = (int)newRefFrame.getWords().size();
			}

			if((features >= registrationPipeline_->getMinVisualCorrespondences()) &&
			   (registrationPipeline_->getMinGeometryCorrespondencesRatio()==0.0f ||
					   (newRefFrame.sensorData().laserScanRaw().cols &&
					   (newRefFrame.sensorData().laserScanMaxPts() == 0 || float(newRefFrame.sensorData().laserScanRaw().cols)/float(newRefFrame.sensorData().laserScanMaxPts())>=registrationPipeline_->getMinGeometryCorrespondencesRatio()))))
			{
				refFrame_ = newRefFrame;

				//reset motion
				motionSinceLastKeyFrame_.setIdentity();
			}
			else
			{
				if(features < registrationPipeline_->getMinVisualCorrespondences())
				{
					UWARN("Too low 2D features (%d), keeping last key frame...", features);
				}

				if(registrationPipeline_->getMinGeometryCorrespondencesRatio()>0.0f && newRefFrame.sensorData().laserScanRaw().cols==0)
				{
					UWARN("Too low scan points (%d), keeping last key frame...", newRefFrame.sensorData().laserScanRaw().cols);
				}
				else if(registrationPipeline_->getMinGeometryCorrespondencesRatio()>0.0f && newRefFrame.sensorData().laserScanMaxPts() != 0 && float(newRefFrame.sensorData().laserScanRaw().cols)/float(newRefFrame.sensorData().laserScanMaxPts())<registrationPipeline_->getMinGeometryCorrespondencesRatio())
				{
					UWARN("Too low scan points ratio (%d < %d), keeping last key frame...", float(newRefFrame.sensorData().laserScanRaw().cols)/float(newRefFrame.sensorData().laserScanMaxPts()), registrationPipeline_->getMinGeometryCorrespondencesRatio());
				}
			}
		}
	}
	else if(!regInfo.rejectedMsg.empty())
	{
		UWARN("Registration failed: \"%s\"", regInfo.rejectedMsg.c_str());
	}

	if(info)
	{
		info->type = 1;
		info->variance = regInfo.variance;
		info->inliers = regInfo.inliers;
		info->icpInliersRatio = regInfo.icpInliersRatio;
		info->matches = regInfo.matches;
	}

	UINFO("Odom update time = %fs lost=%s inliers=%d, ref frame corners=%d, transform accepted=%s",
			timer.elapsed(),
			output.isNull()?"true":"false",
			(int)regInfo.inliers,
			(int)refFrame_.getWords().size(),
			!output.isNull()?"true":"false");

	return output;
}

} // namespace rtabmap
