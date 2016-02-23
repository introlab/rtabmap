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

#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/VisualWord.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/RegistrationVis.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_correspondences.h"
#include "rtabmap/core/util3d_motion_estimation.h"
#include "rtabmap/core/Optimizer.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <rtabmap/core/OdometryF2M.h>

#if _MSC_VER
	#define ISFINITE(value) _finite(value)
#else
	#define ISFINITE(value) std::isfinite(value)
#endif

namespace rtabmap {

OdometryF2M::OdometryF2M(const ParametersMap & parameters) :
	Odometry(parameters),
	maximumMapSize_(Parameters::defaultOdomF2MMaxSize()),
	fixedMapPath_(Parameters::defaultOdomF2MFixedMapPath()),
	regVis_(new RegistrationVis(parameters)),
	map_(new Signature(-1)),
	lastFrame_(new Signature(1))
{
	UDEBUG("");
	Parameters::parse(parameters, Parameters::kOdomF2MMaxSize(), maximumMapSize_);
	Parameters::parse(parameters, Parameters::kOdomF2MFixedMapPath(), fixedMapPath_);

	if(!fixedMapPath_.empty())
	{
		UINFO("Init odometry from a fixed database: \"%s\"", fixedMapPath_.c_str());
		// init the local map with a all 3D features contained in the database
		ParametersMap customParameters;
		customParameters.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
		customParameters.insert(ParametersPair(Parameters::kMemInitWMWithAllNodes(), "true"));
		customParameters.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
		Memory memory(customParameters);
		if(!memory.init(fixedMapPath_, false, ParametersMap()))
		{
			UERROR("Error initializing the memory for BOW Odometry.");
		}
		else
		{
			// get the graph
			std::map<int, int> ids = memory.getNeighborsId(memory.getLastSignatureId(), 0, -1);
			std::map<int, Transform> poses;
			std::multimap<int, Link> links;
			memory.getMetricConstraints(uKeysSet(ids), poses, links, true);

			if(poses.size())
			{
				//optimize the graph
				Optimizer * optimizer = Optimizer::create(parameters);
				std::map<int, Transform> optimizedPoses = optimizer->optimize(poses.begin()->first, poses, links);
				delete optimizer;

				std::multimap<int, cv::Point3f> words3D;
				std::multimap<int, cv::Mat> wordsDescriptors;

				// fill the local map
				for(std::map<int, Transform>::iterator posesIter=optimizedPoses.begin();
					posesIter!=optimizedPoses.end();
					++posesIter)
				{
					const Signature * s = memory.getSignature(posesIter->first);
					if(s)
					{
						// Transform 3D points accordingly to pose and add them to local map
						for(std::multimap<int, cv::Point3f>::const_iterator pointsIter=s->getWords3().begin();
							pointsIter!=s->getWords3().end();
							++pointsIter)
						{
							if(!uContains(words3D, pointsIter->first))
							{
								words3D.insert(std::make_pair(pointsIter->first, util3d::transformPoint(pointsIter->second, posesIter->second)));

								if(s->getWordsDescriptors().size() == s->getWords3().size())
								{
									UASSERT(uContains(s->getWordsDescriptors(), pointsIter->first));
									wordsDescriptors.insert(std::make_pair(pointsIter->first, s->getWordsDescriptors().find(pointsIter->first)->second));
								}
								else // load descriptor from dictionary
								{
									UASSERT(memory.getVWDictionary()->getWord(pointsIter->first) != 0);
									wordsDescriptors.insert(std::make_pair(pointsIter->first, memory.getVWDictionary()->getWord(pointsIter->first)->getDescriptor()));
								}
							}
						}
					}
				}
				UASSERT(words3D.size() == wordsDescriptors.size());
				map_->setWords3(words3D);
				map_->setWordsDescriptors(wordsDescriptors);
			}
			else
			{
				UERROR("No pose loaded from database \"%s\"", fixedMapPath_.c_str());
			}
		}
		if((int)map_->getWords3().size() < regVis_->getMinInliers() || map_->getWords3().size() == 0)
		{
			UERROR("The loaded fixed map from \"%s\" is too small! Only %d unique features loaded. Odometry won't be computed!",
					fixedMapPath_.c_str(), (int)map_->getWords3().size());
		}
	}
}

OdometryF2M::~OdometryF2M()
{
	delete map_;
	delete lastFrame_;
	UDEBUG("");
}


void OdometryF2M::reset(const Transform & initialPose)
{
	if(fixedMapPath_.empty())
	{
		Odometry::reset(initialPose);
		map_->sensorData() = SensorData();
	}
	else
	{
		UWARN("Odometry cannot be reset when a fixed local map is set.");
	}
}

// return not null transform if odometry is correctly computed
Transform OdometryF2M::computeTransform(
		SensorData & data,
		OdometryInfo * info)
{
	UTimer timer;
	Transform output;

	if(info)
	{
		info->type = 0;
	}

	RegistrationInfo regInfo;
	int nFeatures = 0;

	delete lastFrame_;
	lastFrame_ = new Signature(data);

	// Generate keypoints from the new data
	if(lastFrame_->sensorData().isValid())
	{
		if(map_->getWords3().size() && lastFrame_->sensorData().isValid())
		{
			Transform guess = this->previousTransform().isIdentity()||this->previousTransform().isNull()?Transform():this->getPose()*this->previousTransform();
			Transform transform = regVis_->computeTransformationMod(*map_, *lastFrame_, guess, &regInfo);

			data.setFeatures(lastFrame_->sensorData().keypoints(), lastFrame_->sensorData().descriptors());

			if(!transform.isNull())
			{
				// make it incremental
				transform = this->getPose().inverse() * transform;
			}
			else if(!regInfo.rejectedMsg.empty())
			{
				UWARN("Registration failed: \"%s\"", regInfo.rejectedMsg.c_str());
			}
			else
			{
				UWARN("Unknown registration error");
			}

			if(fixedMapPath_.empty())
			{
				output = transform;

				int added = 0;
				int removed = 0;

				// update local map
				std::multimap<int, cv::Point3f> mapPoints = map_->getWords3();
				std::multimap<int, cv::Mat> mapDescriptors = map_->getWordsDescriptors();
				Transform t = this->getPose()*output;
				UASSERT(mapPoints.size() == mapDescriptors.size());
				UASSERT(lastFrame_->getWordsDescriptors().size() == lastFrame_->getWords3().size());
				std::list<int> newIds = uUniqueKeys(lastFrame_->getWordsDescriptors());
				for(std::list<int>::iterator iter=newIds.begin(); iter!=newIds.end(); ++iter)
				{
					if(mapPoints.find(*iter) == mapPoints.end())
					{
						mapPoints.insert(std::make_pair(*iter, util3d::transformPoint(lastFrame_->getWords3().find(*iter)->second, t)));
						mapDescriptors.insert(std::make_pair(*iter, lastFrame_->getWordsDescriptors().find(*iter)->second));
						++added;
					}
				}

				// remove words in map if max size is reached
				if((int)mapPoints.size() > maximumMapSize_)
				{
					// remove oldest first, keep matched features
					std::set<int> matches(regInfo.matchesIDs.begin(), regInfo.matchesIDs.end());
					std::multimap<int, cv::Mat>::iterator iterMapWords = mapDescriptors.begin();
					for(std::multimap<int, cv::Point3f>::iterator iter = mapPoints.begin();
						iter!=mapPoints.end() && (int)mapPoints.size() > maximumMapSize_ && mapPoints.size() >= newIds.size();)
					{
						if(matches.find(iter->first) == matches.end())
						{
							iter = mapPoints.erase(iter);
							iterMapWords = mapDescriptors.erase(iterMapWords);
							++removed;
						}
						else
						{
							++iter;
							++iterMapWords;
						}
					}
				}

				map_->setWords3(mapPoints);
				map_->setWordsDescriptors(mapDescriptors);

				UINFO("Updated map: %d added %d removed (new map size=%d)", added, removed, (int)mapPoints.size());
			}
			else
			{
				// fixed local map, don't update with the new signature
				output = transform;
			}
		}
		else
		{
			// just generate keypoints for the new signature
			Signature dummy;
			regVis_->computeTransformationMod(
					*lastFrame_,
					dummy);

			data.setFeatures(lastFrame_->sensorData().keypoints(), lastFrame_->sensorData().descriptors());

			if(fixedMapPath_.empty() && (int)lastFrame_->getWords3().size() >= regVis_->getMinInliers())
			{
				output.setIdentity();
				// a very high variance tells that the new pose is not linked with the previous one
				regInfo.variance = 9999;

				Transform t = this->getPose(); // initial pose may be not identity...
				std::multimap<int, cv::Point3f> transformedPoints;
				for(std::multimap<int, cv::Point3f>::const_iterator iter = lastFrame_->getWords3().begin(); iter!=lastFrame_->getWords3().end(); ++iter)
				{
					transformedPoints.insert(std::make_pair(iter->first, util3d::transformPoint(iter->second, t)));
				}

				map_->setWords3(transformedPoints);
				map_->setWordsDescriptors(lastFrame_->getWordsDescriptors());
				map_->sensorData().setCameraModels(lastFrame_->sensorData().cameraModels());
				map_->sensorData().setStereoCameraModel(lastFrame_->sensorData().stereoCameraModel());
			}
		}

		map_->sensorData().setFeatures(std::vector<cv::KeyPoint>(), cv::Mat()); // clear sensorData features

		nFeatures = lastFrame_->getWords().size();
		if(this->isInfoDataFilled() && info)
		{
			info->words = lastFrame_->getWords();
		}
	}

	if(info)
	{
		info->variance = regInfo.variance;
		info->inliers = regInfo.inliers;
		info->matches = regInfo.matches;
		info->features = nFeatures;
		info->localMapSize = (int)map_->getWords3().size();

		if(this->isInfoDataFilled())
		{
			info->wordMatches = regInfo.matchesIDs;
			info->wordInliers = regInfo.inliersIDs;
		}
	}

	UINFO("Odom update time = %fs lost=%s features=%d inliers=%d/%d variance=%f local_map=%d",
			timer.elapsed(),
			output.isNull()?"true":"false",
			nFeatures,
			regInfo.inliers,
			regInfo.matches,
			regInfo.variance,
			(int)map_->getWords3().size());
	return output;
}

} // namespace rtabmap
