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

#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_correspondences.h"
#include "rtabmap/core/util3d_motion_estimation.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/VWDictionary.h"
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

OdometryBOW::OdometryBOW(const ParametersMap & parameters) :
	Odometry(parameters),
	_localHistoryMaxSize(Parameters::defaultOdomBowLocalHistorySize()),
	_fixedLocalMapPath(Parameters::defaultOdomBowFixedLocalMapPath()),
	_memory(0)
{
	UDEBUG("");
	Parameters::parse(parameters, Parameters::kOdomBowLocalHistorySize(), _localHistoryMaxSize);
	Parameters::parse(parameters, Parameters::kOdomBowFixedLocalMapPath(), _fixedLocalMapPath);

	ParametersMap customParameters;
	customParameters.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(this->getMaxDepth())));
	customParameters.insert(ParametersPair(Parameters::kKpRoiRatios(), this->getRoiRatios()));
	customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
	customParameters.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
	customParameters.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
	customParameters.insert(ParametersPair(Parameters::kMemNotLinkedNodesKept(), "false"));
	int nn = Parameters::defaultOdomBowNNType();
	float nndr = Parameters::defaultOdomBowNNDR();
	int featureType = Parameters::defaultOdomFeatureType();
	int maxFeatures = Parameters::defaultOdomMaxFeatures();
	Parameters::parse(parameters, Parameters::kOdomBowNNType(), nn);
	Parameters::parse(parameters, Parameters::kOdomBowNNDR(), nndr);
	Parameters::parse(parameters, Parameters::kOdomFeatureType(), featureType);
	Parameters::parse(parameters, Parameters::kOdomMaxFeatures(), maxFeatures);
	customParameters.insert(ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(nn)));
	customParameters.insert(ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(nndr)));
	customParameters.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(featureType)));
	customParameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), uNumber2Str(maxFeatures)));

	// Memory's stereo parameters, copy from Odometry
	int subPixWinSize = Parameters::defaultOdomSubPixWinSize();
	int subPixIterations = Parameters::defaultOdomSubPixIterations();
	double subPixEps = Parameters::defaultOdomSubPixEps();
	Parameters::parse(parameters, Parameters::kOdomSubPixWinSize(), subPixWinSize);
	Parameters::parse(parameters, Parameters::kOdomSubPixIterations(), subPixIterations);
	Parameters::parse(parameters, Parameters::kOdomSubPixEps(), subPixEps);
	customParameters.insert(ParametersPair(Parameters::kKpSubPixWinSize(), uNumber2Str(subPixWinSize)));
	customParameters.insert(ParametersPair(Parameters::kKpSubPixIterations(), uNumber2Str(subPixIterations)));
	customParameters.insert(ParametersPair(Parameters::kKpSubPixEps(), uNumber2Str(subPixEps)));

	// add only feature stuff
	for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string group = uSplit(iter->first, '/').front();
		if(group.compare("SURF") == 0 ||
			group.compare("SIFT") == 0 ||
			group.compare("BRIEF") == 0 ||
			group.compare("FAST") == 0 ||
			group.compare("ORB") == 0 ||
			group.compare("FREAK") == 0 ||
			group.compare("GFTT") == 0 ||
			group.compare("BRISK") == 0)
		{
			customParameters.insert(*iter);
		}
	}

	if(_fixedLocalMapPath.empty())
	{
		_memory = new Memory(customParameters);
		if(!_memory->init("", false, ParametersMap()))
		{
			UERROR("Error initializing the memory for BOW Odometry.");
		}
	}
	else
	{
		UINFO("Init odometry from a fixed database: \"%s\"", _fixedLocalMapPath.c_str());
		// init the local map with a all 3D features contained in the database
		customParameters.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
		customParameters.insert(ParametersPair(Parameters::kMemInitWMWithAllNodes(), "true"));
		_memory = new Memory(customParameters);
		if(!_memory->init(_fixedLocalMapPath, false, ParametersMap()))
		{
			UERROR("Error initializing the memory for BOW Odometry.");
		}
		else
		{
			// get the graph
			std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastSignatureId(), 0, -1);
			std::map<int, Transform> poses;
			std::multimap<int, Link> links;
			_memory->getMetricConstraints(uKeysSet(ids), poses, links, true);

			if(poses.size())
			{
				//optimize the graph
				graph::TOROOptimizer optimizer;
				std::map<int, Transform> optimizedPoses = optimizer.optimize(poses.begin()->first, poses, links);

				// fill the local map
				for(std::map<int, Transform>::iterator posesIter=optimizedPoses.begin();
					posesIter!=optimizedPoses.end();
					++posesIter)
				{
					const Signature * s = _memory->getSignature(posesIter->first);
					if(s)
					{
						// Transform 3D points accordingly to pose and add them to local map
						const std::multimap<int, pcl::PointXYZ> & words3D = s->getWords3();
						for(std::multimap<int, pcl::PointXYZ>::const_iterator pointsIter=words3D.begin();
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
				UERROR("No pose loaded from database \"%s\"", _fixedLocalMapPath.c_str());
			}
		}
		if((int)localMap_.size() < this->getMinInliers() || localMap_.size() == 0)
		{
			UERROR("The loaded fixed map from \"%s\" is too small! Only %d unique features loaded. Odometry won't be computed!",
					_fixedLocalMapPath.c_str(), (int)localMap_.size());
		}
	}
}

OdometryBOW::~OdometryBOW()
{
	delete _memory;
	UDEBUG("");
}


void OdometryBOW::reset(const Transform & initialPose)
{
	if(_fixedLocalMapPath.empty())
	{
		Odometry::reset(initialPose);
		_memory->init("", false, ParametersMap());
		localMap_.clear();
	}
	else
	{
		UWARN("Odometry cannot be reset when a fixed local map is set.");
	}
}

// return not null transform if odometry is correctly computed
Transform OdometryBOW::computeTransform(
		const SensorData & data,
		OdometryInfo * info)
{
	UTimer timer;
	Transform output;

	if(info)
	{
		info->type = 0;
	}

	double variance = 0;
	int inliersCount = 0;
	int correspondences = 0;
	int nFeatures = 0;

	if(_memory->update(data))
	{
		const Signature * newSignature = _memory->getLastWorkingSignature();
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
			if((int)localMap_.size() >= this->getMinInliers())
			{
				std::vector<int> matches, inliers;
				Transform t;
				if(this->getEstimationType() == 1) // PnP
				{
					// 3D to 2D
					if(data.cameraModels().size() > 1)
					{
						UERROR("PnP cannot be used on multi-cameras setup.");
					}
					else if((int)newSignature->getWords().size() >= this->getMinInliers())
					{
						UASSERT(data.stereoCameraModel().isValid() || (data.cameraModels().size() == 1 && data.cameraModels()[0].isValid()));
						const CameraModel & cameraModel = data.stereoCameraModel().isValid()?data.stereoCameraModel().left():data.cameraModels()[0];

						t = util3d::estimateMotion3DTo2D(
								localMap_,
								newSignature->getWords(),
								cameraModel,
								this->getMinInliers(),
								this->getIterations(),
								this->getPnPReprojError(),
								this->getPnPFlags(),
								this->getPose(),
								newSignature->getWords3(),
								&variance,
								&matches,
								&inliers);
					}
					else
					{
						UWARN("Not enough features in the new image (%d < %d)", (int)newSignature->getWords().size(), this->getMinInliers());
					}
				}
				else
				{
					// 3D to 3D
					if((int)newSignature->getWords3().size() >= this->getMinInliers())
					{
						t = util3d::estimateMotion3DTo3D(
								localMap_,
								newSignature->getWords3(),
								this->getMinInliers(),
								this->getInlierDistance(),
								this->getIterations(),
								this->getRefineIterations(),
								&variance,
								&matches,
								&inliers);
					}
					else
					{
						UWARN("Not enough 3D features in the new image (%d < %d)", (int)newSignature->getWords3().size(), this->getMinInliers());
					}
				}

				correspondences = matches.size();
				inliersCount = inliers.size();
				if(this->isInfoDataFilled() && info)
				{
					info->wordMatches = matches;
					info->wordInliers = inliers;
				}

				if(!t.isNull())
				{
					// make it incremental
					transform = this->getPose().inverse() * t;
				}
				else if(correspondences < this->getMinInliers())
				{
					UWARN("Not enough correspondences (%d < %d)", correspondences, this->getMinInliers());
				}
				else if(inliersCount < this->getMinInliers())
				{
					UWARN("Not enough inliers (%d < %d)", inliersCount, this->getMinInliers());
				}
				else
				{
					UWARN("Unknown estimation error");
				}
			}
			else
			{
				UWARN("Local map too small!? (%d < %d)", (int)localMap_.size(), this->getMinInliers());
			}

			if(transform.isNull())
			{
				_memory->deleteLocation(newSignature->id());
			}
			else if(_fixedLocalMapPath.empty())
			{
				output = transform;

				// remove words if history max size is reached
				while(localMap_.size() && (int)localMap_.size() > _localHistoryMaxSize && _memory->getStMem().size()>1)
				{
					int nodeId = *_memory->getStMem().begin();
					std::list<int> removedPts;
					_memory->deleteLocation(nodeId, &removedPts);
					for(std::list<int>::iterator iter = removedPts.begin(); iter!=removedPts.end(); ++iter)
					{
						localMap_.erase(*iter);
					}
				}

				if(_localHistoryMaxSize == 0 && localMap_.size() > 0 && localMap_.size() > newSignature->getWords3().size())
				{
					UERROR("Local map should have only words of the last added signature here! (size=%d, max history size=%d, newWords=%d)",
							(int)localMap_.size(), _localHistoryMaxSize, (int)newSignature->getWords3().size());
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
							const pcl::PointXYZ & pt = newSignature->getWords3().find(*iter)->second;
							if(pcl::isFinite(pt))
							{
								pcl::PointXYZ pt2 = util3d::transformPoint(pt, t);
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
				_memory->deleteLocation(newSignature->id());
			}
		}
		else if(newSignature)
		{
			int count = 0;
			std::list<int> uniques = uUniqueKeys(newSignature->getWords3());
			if(_fixedLocalMapPath.empty() && (int)uniques.size() >= this->getMinInliers())
			{
				output.setIdentity();

				Transform t = this->getPose(); // initial pose maybe not identity...
				for(std::list<int>::iterator iter = uniques.begin(); iter!=uniques.end(); ++iter)
				{
					// Only add unique words
					if(newSignature->getWords3().count(*iter) == 1)
					{
						const pcl::PointXYZ & pt = newSignature->getWords3().find(*iter)->second;
						if(pcl::isFinite(pt))
						{
							pcl::PointXYZ pt2 = util3d::transformPoint(pt, t);
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
				_memory->deleteLocation(newSignature->id());
			}
			UDEBUG("uniques=%d, pt not finite = %d", (int)uniques.size(),count);
		}

		_memory->emptyTrash();
	}

	if(info)
	{
		info->variance = variance;
		info->inliers = inliersCount;
		info->matches = correspondences;
		info->features = nFeatures;
		info->localMapSize = (int)localMap_.size();
	}

	UINFO("Odom update time = %fs lost=%s features=%d inliers=%d/%d variance=%f local_map=%d dict=%d nodes=%d",
			timer.elapsed(),
			output.isNull()?"true":"false",
			nFeatures,
			inliersCount,
			correspondences,
			variance,
			(int)localMap_.size(),
			(int)_memory->getVWDictionary()->getVisualWords().size(),
			(int)_memory->getStMem().size());
	return output;
}

} // namespace rtabmap
