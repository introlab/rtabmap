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

#include "rtabmap/core/OdometryLocalMap.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Memory.h"
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

#if _MSC_VER
	#define ISFINITE(value) _finite(value)
#else
	#define ISFINITE(value) std::isfinite(value)
#endif

namespace rtabmap {

OdometryLocalMap::OdometryLocalMap(const ParametersMap & parameters) :
	Odometry(parameters),
	localHistoryMaxSize_(Parameters::defaultOdomBowLocalHistorySize()),
	fixedLocalMapPath_(Parameters::defaultOdomBowFixedLocalMapPath()),
	memory_(0),
	regVis_(new RegistrationVis(parameters))
{
	UDEBUG("");
	Parameters::parse(parameters, Parameters::kOdomBowLocalHistorySize(), localHistoryMaxSize_);
	Parameters::parse(parameters, Parameters::kOdomBowFixedLocalMapPath(), fixedLocalMapPath_);

	ParametersMap customParameters;
	float minDepth = Parameters::defaultVisMinDepth();
	float maxDepth = Parameters::defaultVisMaxDepth();
	std::string roi = Parameters::defaultVisRoiRatios();
	Parameters::parse(parameters, Parameters::kVisMinDepth(), minDepth);
	Parameters::parse(parameters, Parameters::kVisMaxDepth(), maxDepth);
	Parameters::parse(parameters, Parameters::kVisRoiRatios(), roi);
	customParameters.insert(ParametersPair(Parameters::kKpMinDepth(), uNumber2Str(minDepth)));
	customParameters.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(maxDepth)));
	customParameters.insert(ParametersPair(Parameters::kKpRoiRatios(), roi));
	customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
	customParameters.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
	customParameters.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
	customParameters.insert(ParametersPair(Parameters::kMemNotLinkedNodesKept(), "false"));
	customParameters.insert(ParametersPair(Parameters::kMemSaveDepth16Format(), "false"));
	int nn = Parameters::defaultVisCorNNType();
	float nndr = Parameters::defaultVisCorNNDR();
	int featureType = Parameters::defaultVisFeatureType();
	int maxFeatures = Parameters::defaultVisMaxFeatures();
	Parameters::parse(parameters, Parameters::kVisCorNNType(), nn);
	Parameters::parse(parameters, Parameters::kVisCorNNDR(), nndr);
	Parameters::parse(parameters, Parameters::kVisFeatureType(), featureType);
	Parameters::parse(parameters, Parameters::kVisMaxFeatures(), maxFeatures);
	customParameters.insert(ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(nn)));
	customParameters.insert(ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(nndr)));
	customParameters.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(featureType)));
	customParameters.insert(ParametersPair(Parameters::kKpMaxFeatures(), uNumber2Str(maxFeatures)));

	// Memory's stereo parameters, copy from Odometry
	int subPixWinSize = Parameters::defaultVisSubPixWinSize();
	int subPixIterations = Parameters::defaultVisSubPixIterations();
	double subPixEps = Parameters::defaultVisSubPixEps();
	Parameters::parse(parameters, Parameters::kVisSubPixWinSize(), subPixWinSize);
	Parameters::parse(parameters, Parameters::kVisSubPixIterations(), subPixIterations);
	Parameters::parse(parameters, Parameters::kVisSubPixEps(), subPixEps);
	customParameters.insert(ParametersPair(Parameters::kKpSubPixWinSize(), uNumber2Str(subPixWinSize)));
	customParameters.insert(ParametersPair(Parameters::kKpSubPixIterations(), uNumber2Str(subPixIterations)));
	customParameters.insert(ParametersPair(Parameters::kKpSubPixEps(), uNumber2Str(subPixEps)));

	// add only feature stuff
	for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string group = uSplit(iter->first, '/').front();
		if(Parameters::isFeatureParameter(iter->first) ||
		   group.compare("Stereo") == 0)
		{
			customParameters.insert(*iter);
		}
	}

	if(fixedLocalMapPath_.empty())
	{
		memory_ = new Memory(customParameters);
		if(!memory_->init("", false, ParametersMap()))
		{
			UERROR("Error initializing the memory for BOW Odometry.");
		}
	}
	else
	{
		UINFO("Init odometry from a fixed database: \"%s\"", fixedLocalMapPath_.c_str());
		// init the local map with a all 3D features contained in the database
		customParameters.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
		customParameters.insert(ParametersPair(Parameters::kMemInitWMWithAllNodes(), "true"));
		memory_ = new Memory(customParameters);
		if(!memory_->init(fixedLocalMapPath_, false, ParametersMap()))
		{
			UERROR("Error initializing the memory for BOW Odometry.");
		}
		else
		{
			// get the graph
			std::map<int, int> ids = memory_->getNeighborsId(memory_->getLastSignatureId(), 0, -1);
			std::map<int, Transform> poses;
			std::multimap<int, Link> links;
			memory_->getMetricConstraints(uKeysSet(ids), poses, links, true);

			if(poses.size())
			{
				//optimize the graph
				Optimizer * optimizer = Optimizer::create(parameters);
				std::map<int, Transform> optimizedPoses = optimizer->optimize(poses.begin()->first, poses, links);
				delete optimizer;

				// fill the local map
				for(std::map<int, Transform>::iterator posesIter=optimizedPoses.begin();
					posesIter!=optimizedPoses.end();
					++posesIter)
				{
					const Signature * s = memory_->getSignature(posesIter->first);
					if(s)
					{
						// Transform 3D points accordingly to pose and add them to local map
						const std::multimap<int, cv::Point3f> & words3D = s->getWords3();
						for(std::multimap<int, cv::Point3f>::const_iterator pointsIter=words3D.begin();
							pointsIter!=words3D.end();
							++pointsIter)
						{
							if(!uContains(localMap_, pointsIter->first))
							{
								localMap_.insert(std::make_pair(pointsIter->first, util3d::transformPoint(pointsIter->second, posesIter->second)));
							}
						}
					}
				}
			}
			else
			{
				UERROR("No pose loaded from database \"%s\"", fixedLocalMapPath_.c_str());
			}
		}
		if((int)localMap_.size() < regVis_->getMinInliers() || localMap_.size() == 0)
		{
			UERROR("The loaded fixed map from \"%s\" is too small! Only %d unique features loaded. Odometry won't be computed!",
					fixedLocalMapPath_.c_str(), (int)localMap_.size());
		}
	}
}

OdometryLocalMap::~OdometryLocalMap()
{
	delete memory_;
	UDEBUG("");
}


void OdometryLocalMap::reset(const Transform & initialPose)
{
	if(fixedLocalMapPath_.empty())
	{
		Odometry::reset(initialPose);
		memory_->init("", false, ParametersMap());
		localMap_.clear();
	}
	else
	{
		UWARN("Odometry cannot be reset when a fixed local map is set.");
	}
}

// return not null transform if odometry is correctly computed
Transform OdometryLocalMap::computeTransform(
		const SensorData & data,
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

	if(memory_->update(data))
	{
		const Signature * newSignature = memory_->getLastWorkingSignature();
		if(newSignature)
		{
			nFeatures = (int)newSignature->getWords().size();
			if(this->isInfoDataFilled() && info)
			{
				info->words = newSignature->getWords();
			}
		}

		if(localMap_.size() && newSignature)
		{
			Transform transform;
			if((int)localMap_.size() >= regVis_->getMinInliers() &&
			   (int)newSignature->getWords().size()>=regVis_->getMinInliers())
			{
				Transform t;
				Signature tmpLocalMap(-1);
				tmpLocalMap.setWords3(localMap_);
				t = regVis_->computeTransformation(tmpLocalMap, *newSignature, this->getPose(), &regInfo);

				if(!t.isNull())
				{
					// make it incremental
					transform = this->getPose().inverse() * t;
				}
				else if(!regInfo.rejectedMsg.empty())
				{
					UWARN("Registration failed: \"%s\"", regInfo.rejectedMsg.c_str());
				}
				else
				{
					UWARN("Unknown registration error");
				}
			}
			else if((int)newSignature->getWords().size()<regVis_->getMinInliers())
			{
				UWARN("New signature has too low extracted features (%d < %d)", (int)newSignature->getWords().size(), regVis_->getMinInliers());
			}
			else
			{
				UWARN("Local map too small!? (%d < %d)", (int)localMap_.size(), regVis_->getMinInliers());
			}

			if(transform.isNull())
			{
				memory_->deleteLocation(newSignature->id());
			}
			else if(fixedLocalMapPath_.empty())
			{
				output = transform;

				// remove words if history max size is reached
				while(localMap_.size() && (int)localMap_.size() > localHistoryMaxSize_ && memory_->getStMem().size()>1)
				{
					int nodeId = *memory_->getStMem().begin();
					std::list<int> removedPts;
					memory_->deleteLocation(nodeId, &removedPts);
					for(std::list<int>::iterator iter = removedPts.begin(); iter!=removedPts.end(); ++iter)
					{
						localMap_.erase(*iter);
					}
				}

				if(localHistoryMaxSize_ == 0 && localMap_.size() > 0 && localMap_.size() > newSignature->getWords3().size())
				{
					UERROR("Local map should have only words of the last added signature here! (size=%d, max history size=%d, newWords=%d)",
							(int)localMap_.size(), localHistoryMaxSize_, (int)newSignature->getWords3().size());
				}

				// update local map
				std::list<int> uniques = uUniqueKeys(newSignature->getWords3());
				Transform t = this->getPose()*output;
				for(std::list<int>::iterator iter = uniques.begin(); iter!=uniques.end(); ++iter)
				{
					// Only add unique words not in local map
					if(newSignature->getWords3().count(*iter) == 1)
					{
						// keep old word
						if(localMap_.find(*iter) == localMap_.end())
						{
							const cv::Point3f & pt = newSignature->getWords3().find(*iter)->second;
							if(util3d::isFinite(pt))
							{
								cv::Point3f pt2 = util3d::transformPoint(pt, t);
								localMap_.insert(std::make_pair(*iter, pt2));
							}
						}
					}
					else
					{
						localMap_.erase(*iter);
					}
				}
			}
			else
			{
				// fixed local map, just delete the new signature
				output = transform;
				memory_->deleteLocation(newSignature->id());
			}
		}
		else if(newSignature)
		{
			int count = 0;
			std::list<int> uniques = uUniqueKeys(newSignature->getWords3());
			if(fixedLocalMapPath_.empty() && (int)uniques.size() >= regVis_->getMinInliers())
			{
				output.setIdentity();

				Transform t = this->getPose(); // initial pose maybe not identity...
				for(std::list<int>::iterator iter = uniques.begin(); iter!=uniques.end(); ++iter)
				{
					// Only add unique words
					if(newSignature->getWords3().count(*iter) == 1)
					{
						const cv::Point3f & pt = newSignature->getWords3().find(*iter)->second;
						if(util3d::isFinite(pt))
						{
							cv::Point3f pt2 = util3d::transformPoint(pt, t);
							localMap_.insert(std::make_pair(*iter, pt2));
						}
						else
						{
							++count;
						}
					}
				}
			}
			else
			{
				// not enough features, just delete it
				memory_->deleteLocation(newSignature->id());
			}
			UDEBUG("uniques=%d, pt not finite = %d", (int)uniques.size(),count);
		}

		memory_->emptyTrash();
	}

	if(info)
	{
		info->variance = regInfo.variance;
		info->inliers = regInfo.inliers;
		info->matches = regInfo.matches;
		info->features = nFeatures;
		info->localMapSize = (int)localMap_.size();

		if(this->isInfoDataFilled())
		{
			info->wordMatches = regInfo.matchesIDs;
			info->wordInliers = regInfo.inliersIDs;
		}
	}

	UINFO("Odom update time = %fs lost=%s features=%d inliers=%d/%d variance=%f local_map=%d dict=%d nodes=%d",
			timer.elapsed(),
			output.isNull()?"true":"false",
			nFeatures,
			regInfo.inliers,
			regInfo.matches,
			regInfo.variance,
			(int)localMap_.size(),
			(int)memory_->getVWDictionary()->getVisualWords().size(),
			(int)memory_->getStMem().size());
	return output;
}

} // namespace rtabmap
