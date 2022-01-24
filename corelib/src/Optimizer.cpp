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

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/RegistrationVis.h>
#include <set>
#include <queue>

#include <rtabmap/core/optimizer/OptimizerTORO.h>
#include <rtabmap/core/optimizer/OptimizerG2O.h>
#include <rtabmap/core/optimizer/OptimizerGTSAM.h>
#include <rtabmap/core/optimizer/OptimizerCVSBA.h>
#include <rtabmap/core/optimizer/OptimizerCeres.h>

namespace rtabmap {

bool Optimizer::isAvailable(Optimizer::Type type)
{
	if(type == Optimizer::kTypeG2O)
	{
		return OptimizerG2O::available();
	}
	else if(type == Optimizer::kTypeGTSAM)
	{
		return OptimizerGTSAM::available();
	}
	else if(type == Optimizer::kTypeCVSBA)
	{
		return OptimizerCVSBA::available();
	}
	else if(type == Optimizer::kTypeTORO)
	{
		return OptimizerTORO::available();
	}
	else if(type == Optimizer::kTypeCeres)
	{
		return OptimizerCeres::available();
	}
	return false;
}

Optimizer * Optimizer::create(const ParametersMap & parameters)
{
	int optimizerTypeInt = Parameters::defaultOptimizerStrategy();
	Parameters::parse(parameters, Parameters::kOptimizerStrategy(), optimizerTypeInt);
	return create((Optimizer::Type)optimizerTypeInt, parameters);
}

Optimizer * Optimizer::create(Optimizer::Type type, const ParametersMap & parameters)
{
	UASSERT_MSG(OptimizerG2O::available() || OptimizerGTSAM::available() || OptimizerTORO::available() || OptimizerCeres::available(),
			"RTAB-Map is not built with any graph optimization approach!");

	if(!OptimizerTORO::available() && type == Optimizer::kTypeTORO)
	{
		if(OptimizerGTSAM::available())
		{
			UWARN("TORO optimizer not available. GTSAM will be used instead.");
					type = Optimizer::kTypeGTSAM;
		}
		else if(OptimizerG2O::available())
		{
			UWARN("TORO optimizer not available. g2o will be used instead.");
					type = Optimizer::kTypeG2O;
		}
		else if(OptimizerCeres::available())
		{
			UWARN("TORO optimizer not available. ceres will be used instead.");
					type = Optimizer::kTypeCeres;
		}
	}
	if(!OptimizerG2O::available() && type == Optimizer::kTypeG2O)
	{
		if(OptimizerGTSAM::available())
		{
			UWARN("g2o optimizer not available. GTSAM will be used instead.");
					type = Optimizer::kTypeGTSAM;
		}
		else if(OptimizerTORO::available())
		{
			UWARN("g2o optimizer not available. TORO will be used instead.");
					type = Optimizer::kTypeTORO;
		}
		else if(OptimizerCeres::available())
		{
			UWARN("g2o optimizer not available. ceres will be used instead.");
					type = Optimizer::kTypeCeres;
		}
	}
	if(!OptimizerGTSAM::available() && type == Optimizer::kTypeGTSAM)
	{
		if(OptimizerG2O::available())
		{
			UWARN("GTSAM optimizer not available. g2o will be used instead.");
					type = Optimizer::kTypeG2O;
		}
		else if(OptimizerTORO::available())
		{
			UWARN("GTSAM optimizer not available. TORO will be used instead.");
					type = Optimizer::kTypeTORO;
		}
		else if(OptimizerCeres::available())
		{
			UWARN("GTSAM optimizer not available. ceres will be used instead.");
					type = Optimizer::kTypeCeres;
		}
	}
	if(!OptimizerCVSBA::available() && type == Optimizer::kTypeCVSBA)
	{
		if(OptimizerG2O::available())
		{
			UWARN("CVSBA optimizer not available. g2o will be used instead.");
					type = Optimizer::kTypeG2O;
		}
	}
	if(!OptimizerCeres::available() && type == Optimizer::kTypeCeres)
	{
		if(OptimizerGTSAM::available())
		{
			UWARN("Ceres optimizer not available. gtsam will be used instead.");
					type = Optimizer::kTypeGTSAM;
		}
		else if(OptimizerG2O::available())
		{
			UWARN("Ceres optimizer not available. g2o will be used instead.");
					type = Optimizer::kTypeG2O;
		}
		else if(OptimizerTORO::available())
		{
			UWARN("Ceres optimizer not available. TORO will be used instead.");
					type = Optimizer::kTypeTORO;
		}
	}
	Optimizer * optimizer = 0;
	switch(type)
	{
	case Optimizer::kTypeGTSAM:
		optimizer = new OptimizerGTSAM(parameters);
		break;
	case Optimizer::kTypeG2O:
		optimizer = new OptimizerG2O(parameters);
		break;
	case Optimizer::kTypeCVSBA:
		optimizer = new OptimizerCVSBA(parameters);
		break;
	case Optimizer::kTypeCeres:
		optimizer = new OptimizerCeres(parameters);
		break;
	case Optimizer::kTypeTORO:
	default:
		optimizer = new OptimizerTORO(parameters);
		break;

	}
	return optimizer;
}

void Optimizer::getConnectedGraph(
		int fromId,
		const std::map<int, Transform> & posesIn,
		const std::multimap<int, Link> & linksIn,
		std::map<int, Transform> & posesOut,
		std::multimap<int, Link> & linksOut,
		bool adjustPosesWithConstraints) const
{
	UDEBUG("IN: fromId=%d poses=%d links=%d priorsIgnored=%d landmarksIgnored=%d", fromId, (int)posesIn.size(), (int)linksIn.size(), priorsIgnored()?1:0, landmarksIgnored()?1:0);
	UASSERT(fromId>0);
	UASSERT(uContains(posesIn, fromId));

	posesOut.clear();
	linksOut.clear();

	std::set<int> nextPoses;
	nextPoses.insert(fromId);
	std::multimap<int, int> biLinks;
	for(std::multimap<int, Link>::const_iterator iter=linksIn.begin(); iter!=linksIn.end(); ++iter)
	{
		if(iter->second.from() != iter->second.to())
		{
			if(graph::findLink(biLinks, iter->second.from(), iter->second.to()) == biLinks.end())
			{
				biLinks.insert(std::make_pair(iter->second.from(), iter->second.to()));
				biLinks.insert(std::make_pair(iter->second.to(), iter->second.from()));
			}
		}
	}

	while(nextPoses.size())
	{
		int currentId = *nextPoses.rbegin(); // fill up all nodes before landmarks
		nextPoses.erase(*nextPoses.rbegin());

		if(posesOut.empty())
		{
			posesOut.insert(std::make_pair(currentId, posesIn.find(currentId)->second));

			// add prior links
			for(std::multimap<int, Link>::const_iterator pter=linksIn.find(currentId); pter!=linksIn.end() && pter->first==currentId; ++pter)
			{
				if(pter->second.from() == pter->second.to() && (!priorsIgnored() || pter->second.type() != Link::kPosePrior))
				{
					linksOut.insert(*pter);
				}
			}
		}

		for(std::multimap<int, int>::const_iterator iter=biLinks.find(currentId); iter!=biLinks.end() && iter->first==currentId; ++iter)
		{
			int toId = iter->second;
			if(posesIn.find(toId) != posesIn.end() && (!landmarksIgnored() || toId>0))
			{
				std::multimap<int, Link>::const_iterator kter = graph::findLink(linksIn, currentId, toId);
				if(nextPoses.find(toId) == nextPoses.end())
				{
					if(!uContains(posesOut, toId))
					{
						if(adjustPosesWithConstraints)
						{
							if(isSlam2d() && kter->second.type() == Link::kLandmark && toId>0)
							{
								Transform t;
								if(kter->second.from()==currentId)
								{
									t = kter->second.transform();
								}
								else
								{
									t = kter->second.transform().inverse();
								}
								posesOut.insert(std::make_pair(toId, (posesOut.at(currentId) * t).to3DoF()));
							}
							else
							{
								Transform t = posesOut.at(currentId) * (kter->second.from()==currentId?kter->second.transform():kter->second.transform().inverse());
								posesOut.insert(std::make_pair(toId, t));
							}
						}
						else
						{
							posesOut.insert(*posesIn.find(toId));
						}
						// add prior links
						for(std::multimap<int, Link>::const_iterator pter=linksIn.find(toId); pter!=linksIn.end() && pter->first==toId; ++pter)
						{
							if(pter->second.from() == pter->second.to() && (!priorsIgnored() || pter->second.type() != Link::kPosePrior))
							{
								linksOut.insert(*pter);
							}
						}

						nextPoses.insert(toId);
					}

					// only add unique links
					if(graph::findLink(linksOut, currentId, toId) == linksOut.end())
					{
						if(kter->second.to() < 0)
						{
							// For landmarks, make sure fromId is the landmark
							linksOut.insert(std::make_pair(kter->second.to(), kter->second.inverse()));
						}
						else
						{
							linksOut.insert(*kter);
						}
					}
				}
			}
		}
	}
	UDEBUG("OUT: poses=%d links=%d", (int)posesOut.size(), (int)linksOut.size());
}

Optimizer::Optimizer(int iterations, bool slam2d, bool covarianceIgnored, double epsilon, bool robust, bool priorsIgnored, bool landmarksIgnored, float gravitySigma) :
		iterations_(iterations),
		slam2d_(slam2d),
		covarianceIgnored_(covarianceIgnored),
		epsilon_(epsilon),
		robust_(robust),
		priorsIgnored_(priorsIgnored),
		landmarksIgnored_(landmarksIgnored),
		gravitySigma_(gravitySigma)
{
}

Optimizer::Optimizer(const ParametersMap & parameters) :
		iterations_(Parameters::defaultOptimizerIterations()),
		slam2d_(Parameters::defaultRegForce3DoF()),
		covarianceIgnored_(Parameters::defaultOptimizerVarianceIgnored()),
		epsilon_(Parameters::defaultOptimizerEpsilon()),
		robust_(Parameters::defaultOptimizerRobust()),
		priorsIgnored_(Parameters::defaultOptimizerPriorsIgnored()),
		landmarksIgnored_(Parameters::defaultOptimizerLandmarksIgnored()),
		gravitySigma_(Parameters::defaultOptimizerGravitySigma())
{
	parseParameters(parameters);
}

void Optimizer::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kOptimizerIterations(), iterations_);
	Parameters::parse(parameters, Parameters::kOptimizerVarianceIgnored(), covarianceIgnored_);
	Parameters::parse(parameters, Parameters::kRegForce3DoF(), slam2d_);
	Parameters::parse(parameters, Parameters::kOptimizerEpsilon(), epsilon_);
	Parameters::parse(parameters, Parameters::kOptimizerRobust(), robust_);
	Parameters::parse(parameters, Parameters::kOptimizerPriorsIgnored(), priorsIgnored_);
	Parameters::parse(parameters, Parameters::kOptimizerLandmarksIgnored(), landmarksIgnored_);
	Parameters::parse(parameters, Parameters::kOptimizerGravitySigma(), gravitySigma_);
}

std::map<int, Transform> Optimizer::optimizeIncremental(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & constraints,
		std::list<std::map<int, Transform> > * intermediateGraphes,
		double * finalError,
		int * iterationsDone)
{
	std::map<int, Transform> incGraph;
	std::multimap<int, Link> incGraphLinks;
	incGraph.insert(*poses.begin());
	int i=0;
	std::multimap<int, Link> constraintsCpy = constraints;
	UDEBUG("Incremental optimization... poses=%d constraints=%d", (int)poses.size(), (int)constraints.size());
	for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		incGraph.insert(*iter);
		bool hasLoopClosure = false;
		for(std::multimap<int, Link>::iterator jter=constraintsCpy.lower_bound(iter->first); jter!=constraintsCpy.end() && jter->first==iter->first; ++jter)
		{
			UDEBUG("%d: %d -> %d type=%d", iter->first, jter->second.from(), jter->second.to(), jter->second.type());
			if(jter->second.type() == Link::kNeighbor || jter->second.type() == Link::kNeighborMerged)
			{
				UASSERT(uContains(incGraph, iter->first));
				incGraph.insert(std::make_pair(jter->second.to(), incGraph.at(iter->first) * jter->second.transform()));
				incGraphLinks.insert(*jter);
			}
			else
			{
				if(!uContains(incGraph, jter->second.to()) && jter->second.to() > iter->first)
				{
					// node not yet in graph, switch link direction
					constraintsCpy.insert(std::make_pair(jter->second.to(), jter->second.inverse()));
				}
				else
				{
					UASSERT(uContains(incGraph, jter->second.to()));
					incGraphLinks.insert(*jter);
					hasLoopClosure = true;
				}
			}
		}
		if(hasLoopClosure)
		{
			incGraph = this->optimize(incGraph.begin()->first, incGraph, incGraphLinks);
			if(incGraph.empty())
			{
				UWARN("Failed incremental optimization... last pose added is %d", iter->first);
				break;
			}
		}
		UDEBUG("Iteration %d/%d %s", ++i, (int)poses.size(), hasLoopClosure?"*":"");
	}
	if(!incGraph.empty() && incGraph.size() == poses.size())
	{
		UASSERT(incGraphLinks.size() == constraints.size());
		UASSERT(uContains(poses, rootId) && uContains(incGraph, rootId));
		incGraph.at(rootId) = poses.at(rootId);
		return this->optimize(rootId, incGraph, incGraphLinks, intermediateGraphes, finalError, iterationsDone);
	}

	UDEBUG("Failed incremental optimization");
	return std::map<int, Transform>();
}

std::map<int, Transform> Optimizer::optimize(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		std::list<std::map<int, Transform> > * intermediateGraphes,
		double * finalError,
		int * iterationsDone)
{
	cv::Mat covariance;
	return optimize(rootId,
			poses,
			edgeConstraints,
			covariance,
			intermediateGraphes,
			finalError,
			iterationsDone);
}

std::map<int, Transform> Optimizer::optimize(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & constraints,
		cv::Mat & outputCovariance,
		std::list<std::map<int, Transform> > * intermediateGraphes,
		double * finalError,
		int * iterationsDone)
{
	UERROR("Optimizer %d doesn't implement optimize() method.", (int)this->type());
	return std::map<int, Transform>();
}

std::map<int, Transform> Optimizer::optimizeBA(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, CameraModel> & models,
		std::map<int, cv::Point3f> & points3DMap,
		const std::map<int, std::map<int, FeatureBA> > & wordReferences,
		std::set<int> * outliers)
{
	UERROR("Optimizer %d doesn't implement optimizeBA() method.", (int)this->type());
	return std::map<int, Transform>();
}

std::map<int, Transform> Optimizer::optimizeBA(
		int rootId,
		const std::map<int, Transform> & posesIn,
		const std::multimap<int, Link> & links,
		const std::map<int, Signature> & signatures,
		std::map<int, cv::Point3f> & points3DMap,
		std::map<int, std::map<int, FeatureBA> > & wordReferences,
		bool rematchFeatures)
{
	UDEBUG("");
	std::map<int, CameraModel> models;
	std::map<int, Transform> poses;
	for(std::map<int, Transform>::const_iterator iter=posesIn.lower_bound(1); iter!=posesIn.end(); ++iter)
	{
		// Get camera model
		CameraModel model;
		if(uContains(signatures, iter->first))
		{
			if(signatures.at(iter->first).sensorData().cameraModels().size() == 1 && signatures.at(iter->first).sensorData().cameraModels().at(0).isValidForProjection())
			{
				model = signatures.at(iter->first).sensorData().cameraModels()[0];
			}
			else if(signatures.at(iter->first).sensorData().stereoCameraModel().isValidForProjection())
			{
				model = signatures.at(iter->first).sensorData().stereoCameraModel().left();

				// Set Tx = -baseline*fx for stereo BA
				model = CameraModel(
						model.fx(),
						model.fy(),
						model.cx(),
						model.cy(),
						model.localTransform(),
						-signatures.at(iter->first).sensorData().stereoCameraModel().baseline()*model.fx());
			}
			else if(signatures.at(iter->first).sensorData().cameraModels().size() > 1)
			{
				UERROR("Multi-cameras (%d) is not supported (id=%d).",
						signatures.at(iter->first).sensorData().cameraModels().size(),
						iter->first);
				return std::map<int, Transform>();
			}
			else
			{
				UERROR("Missing calibration for node %d", iter->first);
				return std::map<int, Transform>();
			}
		}
		else
		{
			UERROR("Did not find node %d in cache", iter->first);
			return std::map<int, Transform>();
		}

		UASSERT(model.isValidForProjection());

		models.insert(std::make_pair(iter->first, model));
		poses.insert(*iter);
	}

	// compute correspondences
	this->computeBACorrespondences(poses, links, signatures, points3DMap, wordReferences, rematchFeatures);

	return optimizeBA(rootId, poses, links, models, points3DMap, wordReferences);
}

std::map<int, Transform> Optimizer::optimizeBA(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, Signature> & signatures,
		bool rematchFeatures)
{
	std::map<int, cv::Point3f> points3DMap;
	std::map<int, std::map<int, FeatureBA> > wordReferences;
	return optimizeBA(rootId, poses, links, signatures, points3DMap, wordReferences, rematchFeatures);
}

Transform Optimizer::optimizeBA(
		const Link & link,
		const CameraModel & model,
		std::map<int, cv::Point3f> & points3DMap,
		const std::map<int, std::map<int, FeatureBA> > & wordReferences,
		std::set<int> * outliers)
{
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(link.from(), Transform::getIdentity()));
	poses.insert(std::make_pair(link.to(), link.transform()));
	std::multimap<int, Link> links;
	links.insert(std::make_pair(link.from(), link));
	std::map<int, CameraModel> models;
	models.insert(std::make_pair(link.from(), model));
	models.insert(std::make_pair(link.to(), model));
	poses = optimizeBA(link.from(), poses, links, models, points3DMap, wordReferences, outliers);
	if(poses.size() == 2)
	{
		return poses.rbegin()->second;
	}
	else
	{
		return link.transform();
	}
}

struct KeyPointCompare
{
   bool operator() (const cv::KeyPoint& lhs, const cv::KeyPoint& rhs) const
   {
       return lhs.pt.x < rhs.pt.x || (lhs.pt.x == rhs.pt.x && lhs.pt.y < rhs.pt.y);
   }
};

void Optimizer::computeBACorrespondences(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, Signature> & signatures,
		std::map<int, cv::Point3f> & points3DMap,
		std::map<int, std::map<int, FeatureBA> > & wordReferences,
		bool rematchFeatures)
{
	UDEBUG("");
	int wordCount = 0;
	int edgeWithWordsAdded = 0;
	std::map<int, std::map<cv::KeyPoint, int, KeyPointCompare> > frameToWordMap; // <FrameId, <Keypoint, wordId> >
	for(std::multimap<int, Link>::const_iterator iter=links.lower_bound(1); iter!=links.end(); ++iter)
	{
		Link link = iter->second;
		if(link.to() < link.from())
		{
			link = link.inverse();
		}
		if(link.to() != link.from() &&
		   uContains(signatures, link.from()) &&
		   uContains(signatures, link.to()) &&
		   uContains(poses, link.from()))
		{
			Signature sFrom = signatures.at(link.from());
			if(sFrom.getWeight() >= 0) // ignore intermediate links
			{
				Signature sTo = signatures.at(link.to());
				if(sTo.getWeight() < 0)
				{
					for(std::multimap<int, Link>::const_iterator jter=links.find(sTo.id());
						sTo.getWeight() < 0 && jter!=links.end() &&  uContains(signatures, jter->second.to());
						++jter)
					{
						sTo = signatures.at(jter->second.to());
					}
				}

				if(sFrom.getWords().size() &&
					sTo.getWords().size() &&
					sFrom.getWords3().size())
				{
					ParametersMap regParam;
					regParam.insert(ParametersPair(Parameters::kVisEstimationType(), "1"));
					regParam.insert(ParametersPair(Parameters::kVisPnPReprojError(), "5"));
					regParam.insert(ParametersPair(Parameters::kVisMinInliers(), "6"));
					regParam.insert(ParametersPair(Parameters::kVisCorNNDR(), "0.6"));
					RegistrationVis reg(regParam);

					if(!rematchFeatures)
					{
						sFrom.setWordsDescriptors(cv::Mat());
						sTo.setWordsDescriptors(cv::Mat());
					}

					RegistrationInfo info;
					Transform t = reg.computeTransformationMod(sFrom, sTo, Transform(), &info);
					//Transform t = reg.computeTransformationMod(sFrom, sTo, iter->second.transform(), &info);
					UDEBUG("%d->%d, inliers=%d",sFrom.id(), sTo.id(), (int)info.inliersIDs.size());

					if(!t.isNull())
					{
						if(!rematchFeatures)
						{
							// set descriptors for the output
							if(sFrom.getWords().size() &&
							   sFrom.getWordsDescriptors().empty() &&
							   (int)sFrom.getWords().size() == signatures.at(link.from()).getWordsDescriptors().rows)
							{
								sFrom.setWordsDescriptors(signatures.at(link.from()).getWordsDescriptors());
							}
							if(sTo.getWords().size() &&
							   sTo.getWordsDescriptors().empty() &&
							   (int)sTo.getWords().size() == signatures.at(link.to()).getWordsDescriptors().rows)
							{
								sTo.setWordsDescriptors(signatures.at(link.to()).getWordsDescriptors());
							}
						}

						Transform pose = poses.at(sFrom.id());
						UASSERT(!pose.isNull());
						for(unsigned int i=0; i<info.inliersIDs.size(); ++i)
						{
							int indexFrom = sFrom.getWords().lower_bound(info.inliersIDs[i])->second;
							cv::Point3f p = sFrom.getWords3()[indexFrom];
							if(p.x > 0.0f) // make sure the point is valid
							{
								cv::KeyPoint ptFrom = sFrom.getWordsKpts()[indexFrom];
								int indexTo = sTo.getWords().lower_bound(info.inliersIDs[i])->second;
								cv::KeyPoint ptTo = sTo.getWordsKpts()[indexTo];

								int wordId = -1;

								// find if the word is already added
								std::map<int, std::map<cv::KeyPoint, int, KeyPointCompare> >::iterator fromIter = frameToWordMap.find(sFrom.id());
								std::map<int, std::map<cv::KeyPoint, int, KeyPointCompare> >::iterator toIter = frameToWordMap.find(sTo.id());
								bool fromAlreadyAdded = false;
								bool toAlreadyAdded = false;
								if( fromIter != frameToWordMap.end() &&
									fromIter->second.find(ptFrom) != fromIter->second.end())
								{
									wordId = fromIter->second.at(ptFrom);
									fromAlreadyAdded = true;
								}
								if( toIter != frameToWordMap.end() &&
									toIter->second.find(ptTo) != toIter->second.end())
								{
									wordId = toIter->second.at(ptTo);
									toAlreadyAdded = true;
								}

								if(wordId == -1)
								{
									wordId = ++wordCount;
									wordReferences.insert(std::make_pair(wordId, std::map<int, FeatureBA>()));

									p = util3d::transformPoint(p, pose);
									points3DMap.insert(std::make_pair(wordId, p));
								}
								else
								{
									UASSERT(wordReferences.find(wordId) != wordReferences.end());
									UASSERT(points3DMap.find(wordId) != points3DMap.end());
								}

								if(!fromAlreadyAdded)
								{
									cv::Mat descriptorFrom;
									if(!sFrom.getWordsDescriptors().empty())
									{
										UASSERT(indexFrom < sFrom.getWordsDescriptors().rows);
										descriptorFrom = sFrom.getWordsDescriptors().row(indexFrom);
									}
									wordReferences.at(wordId).insert(std::make_pair(sFrom.id(), FeatureBA(ptFrom, p.x, descriptorFrom)));
									frameToWordMap.insert(std::make_pair(sFrom.id(), std::map<cv::KeyPoint, int, KeyPointCompare>()));
									frameToWordMap.at(sFrom.id()).insert(std::make_pair(ptFrom, wordId));
								}

								if(!toAlreadyAdded)
								{
									cv::Mat descriptorTo;
									if(!sTo.getWordsDescriptors().empty())
									{
										UASSERT(indexTo < sTo.getWordsDescriptors().rows);
										descriptorTo = sTo.getWordsDescriptors().row(indexTo);
									}
									float depth = 0.0f;
									if(!sTo.getWords3().empty())
									{
										UASSERT(indexTo < (int)sTo.getWords3().size());
										const cv::Point3f & pt = sTo.getWords3()[indexTo];
										if( pt.x > 0)
										{
											depth = pt.x;
										}
									}
									wordReferences.at(wordId).insert(std::make_pair(sTo.id(), FeatureBA(ptTo, depth, descriptorTo)));
									frameToWordMap.insert(std::make_pair(sTo.id(), std::map<cv::KeyPoint, int, KeyPointCompare>()));
									frameToWordMap.at(sTo.id()).insert(std::make_pair(ptTo, wordId));
								}
							}
						}
						++edgeWithWordsAdded;
					}
					else
					{
						UWARN("Not enough inliers (%d) between %d and %d", info.inliersIDs.size(), sFrom.id(), sTo.id());
					}
				}
			}
		}
	}
	UDEBUG("Added %d words (edges with words=%d/%d)", wordCount, edgeWithWordsAdded, links.size());
}

} /* namespace rtabmap */
