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
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_features.h>
#include <limits>
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

class LinkIdKey
{
public:
LinkIdKey(int id, Link::Type type) :
		id_(id),
		type_(type) {}
	bool operator<(const LinkIdKey & k) const
	{
		// landmark, sort by smallest to largest landmark id, after normal links
		if(id_ < 0 && k.id_ < 0)
		{
			return id_ > k.id_;
		}
		else if(id_ < 0) {
			return false;
		}
		else if(k.id_ < 0) {
			return true;
		}

		if(type_ == Link::kNeighbor && k.type_ != Link::kNeighbor)
		{
			return true;
		}
		else if(type_ != Link::kNeighbor && k.type_ == Link::kNeighbor)
		{
			return false;
		}
		else if(type_ == Link::kNeighborMerged && k.type_ != Link::kNeighbor && k.type_ != Link::kNeighborMerged)
		{
			return true;
		}
		else if(k.type_ == Link::kNeighborMerged && type_ != Link::kNeighbor && type_ != Link::kNeighborMerged)
		{
			return false;
		}
		else
		{
			// normal link, sort by smallest to largest id
			return id_ < k.id_;
		}
	}
	int id_;
	Link::Type type_;
};

void Optimizer::getConnectedGraph(
		int fromId,
		const std::map<int, Transform> & posesIn,
		const std::multimap<int, Link> & linksIn,
		std::map<int, Transform> & posesOut,
		std::multimap<int, Link> & linksOut) const
{
	UDEBUG("IN: fromId=%d poses=%d links=%d priorsIgnored=%d landmarksIgnored=%d", fromId, (int)posesIn.size(), (int)linksIn.size(), priorsIgnored()?1:0, landmarksIgnored()?1:0);
	UASSERT(fromId>0);
	UASSERT_MSG(uContains(posesIn, fromId), uFormat("poses=%ld (first=%d last=%d) fromId=%d",
		posesIn.size(), posesIn.empty()?0:posesIn.begin()->first, posesIn.empty()?0:posesIn.rbegin()->first, fromId).c_str());

	posesOut.clear();
	linksOut.clear();

	std::map<LinkIdKey, Transform> nextPoses;
	nextPoses.insert(std::make_pair(LinkIdKey(fromId, Link::kUndef), posesIn.find(fromId)->second));
	std::multimap<int, std::pair<int, Link::Type> > biLinks;
	for(std::multimap<int, Link>::const_iterator iter=linksIn.begin(); iter!=linksIn.end(); ++iter)
	{
		if(iter->second.from() != iter->second.to())
		{
			if(graph::findLink(biLinks, iter->second.from(), iter->second.to(), true, iter->second.type()) == biLinks.end())
			{
				biLinks.insert(std::make_pair(iter->second.from(), std::make_pair(iter->second.to(), iter->second.type())));
				biLinks.insert(std::make_pair(iter->second.to(), std::make_pair(iter->second.from(), iter->second.type())));
			}
		}
	}

	while(!nextPoses.empty())
	{
		// Fill up all nodes before landmarks
		// For nodes, fill up all neightbor nodes before loop closure ones
		int currentId = nextPoses.begin()->first.id_; 
		Transform currentPose = nextPoses.begin()->second;
		nextPoses.erase(nextPoses.begin());

		if(posesOut.find(currentId) != posesOut.end()) {
			// Already added from priority list
			continue;
		}

		posesOut.insert(std::make_pair(currentId, currentPose));

		// add prior links
		for(std::multimap<int, Link>::const_iterator pter=linksIn.lower_bound(currentId); pter!=linksIn.end() && pter->first==currentId; ++pter)
		{
			if(pter->second.from() == pter->second.to() && (!priorsIgnored() || pter->second.type() != Link::kPosePrior))
			{
				linksOut.insert(*pter);
			}
		}

		for(std::multimap<int, std::pair<int, Link::Type> >::const_iterator iter=biLinks.lower_bound(currentId); iter!=biLinks.end() && iter->first==currentId; ++iter)
		{
			int toId = iter->second.first;
			Link::Type type = iter->second.second;
			if(posesIn.find(toId) != posesIn.end() && (!landmarksIgnored() || toId>0))
			{
				std::multimap<int, Link>::const_iterator kter = graph::findLink(linksIn, currentId, toId, true, type);
				UASSERT(kter!=linksIn.end());
				if(!uContains(posesOut, toId))
				{
					const Transform & poseToIn = posesIn.at(toId);
					Transform t = kter->second.from()==currentId?kter->second.transform():kter->second.transform().inverse();
					Transform pose;
					if(isSlam2d() && kter->second.type() == Link::kLandmark && toId>0 && (poseToIn.is3DoF() || poseToIn.is4DoF()))
					{
						if(poseToIn.is3DoF())
						{
							pose = (posesOut.at(currentId) * t).to3DoF();
						}
						else
						{
							pose = (posesOut.at(currentId) * t).to4DoF();
						}
					}
					else
					{
						pose = posesOut.at(currentId)* t;
					}

					nextPoses.insert(std::make_pair(LinkIdKey(toId, type), pose));
				}

				// only add unique links
				if(graph::findLink(linksOut, currentId, toId, true, kter->second.type()) == linksOut.end())
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
		const std::map<int, std::vector<CameraModel> > & models,
		std::map<int, cv::Point3f> & points3DMap,
		const std::map<int, std::multimap<int, FeatureBA> > & wordReferences,
		BAOutliers * outliers)
{
	if(outliers)
	{
		outliers->clear();
	}
	UERROR("Optimizer %d doesn't implement optimizeBA() method.", (int)this->type());
	return std::map<int, Transform>();
}
std::map<int, Transform> Optimizer::optimizeBA(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, std::vector<CameraModel> > & models,
		std::map<int, cv::Point3f> & points3DMap,
		const std::map<int, std::map<int, FeatureBA> > & wordReferences,
		BAOutliers * outliers)
{
	std::map<int, std::multimap<int, FeatureBA> > references;
	for(std::map<int, std::map<int, FeatureBA> >::const_iterator iter=wordReferences.begin(); iter!=wordReferences.end(); ++iter)
	{
		references.insert(references.end(), std::make_pair(iter->first, std::multimap<int, FeatureBA>(iter->second.begin(), iter->second.end())));
	}
	return optimizeBA(rootId, poses, links, models, points3DMap, references, outliers);
}


std::map<int, Transform> Optimizer::optimizeBA(
		int rootId,
		const std::map<int, Transform> & posesIn,
		const std::multimap<int, Link> & links,
		const std::map<int, Signature> & signatures,
		std::map<int, cv::Point3f> & points3DMap,
		std::map<int, std::multimap<int, FeatureBA> > & wordReferences,
		bool rematchFeatures,
		const ParametersMap & registrationParameters)
{
	UDEBUG("");
	std::map<int, std::vector<CameraModel> > multiModels;
	std::map<int, Transform> poses;
	for(std::map<int, Transform>::const_iterator iter=posesIn.lower_bound(1); iter!=posesIn.end(); ++iter)
	{
		// Get camera model
		std::vector<CameraModel> models;
		if(uContains(signatures, iter->first))
		{
			const SensorData & s = signatures.at(iter->first).sensorData();
			if(s.cameraModels().size() >= 1 && s.cameraModels().at(0).isValidForProjection())
			{
				models = s.cameraModels();
			}
			else if(!s.stereoCameraModels().empty() && s.stereoCameraModels()[0].isValidForProjection())
			{
				for(size_t i=0; i<s.stereoCameraModels().size(); ++i)
				{
					CameraModel model = s.stereoCameraModels()[i].left();

					// Set Tx = -baseline*fx for stereo BA
					models.push_back(CameraModel(
							model.fx(),
							model.fy(),
							model.cx(),
							model.cy(),
							model.localTransform(),
							-s.stereoCameraModels()[i].baseline()*model.fx(),
							model.imageSize()));
				}
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

		multiModels.insert(std::make_pair(iter->first, models));
		poses.insert(*iter);
	}

	// compute correspondences
	this->computeBACorrespondences(poses, links, signatures, points3DMap, wordReferences, rematchFeatures, false, registrationParameters);

	return optimizeBA(rootId, poses, links, multiModels, points3DMap, wordReferences);
}

std::map<int, Transform> Optimizer::optimizeBA(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, Signature> & signatures,
		bool rematchFeatures,
		const ParametersMap & registrationParameters)
{
	std::map<int, cv::Point3f> points3DMap;
	std::map<int, std::multimap<int, FeatureBA> > wordReferences;
	return optimizeBA(rootId, poses, links, signatures, points3DMap, wordReferences, rematchFeatures, registrationParameters);
}

Transform Optimizer::optimizeBA(
		const Link & link,
		const CameraModel & model,
		std::map<int, cv::Point3f> & points3DMap,
		const std::map<int, std::multimap<int, FeatureBA> > & wordReferences,
		BAOutliers * outliers)
{
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(link.from(), Transform::getIdentity()));
	poses.insert(std::make_pair(link.to(), link.transform()));
	std::multimap<int, Link> links;
	links.insert(std::make_pair(link.from(), link));
	std::map<int, std::vector<CameraModel> > models;
	std::vector<CameraModel> tmp;
	tmp.push_back(model);
	models.insert(std::make_pair(link.from(), tmp));
	models.insert(std::make_pair(link.to(), tmp));
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

// The width of the image of one camera of a multi-camera signature, 0 for a single camera
static int subImageWidthOf(const Signature & signature)
{
	const SensorData & data = signature.sensorData();
	if(data.cameraModels().size() > 1)
	{
		return data.cameraModels()[0].imageWidth();
	}
	if(data.stereoCameraModels().size() > 1)
	{
		return data.stereoCameraModels()[0].left().imageWidth();
	}
	return 0;
}

void Optimizer::computeBACorrespondences(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, Signature> & signatures,
		std::map<int, cv::Point3f> & points3DMap,
		std::map<int, std::multimap<int, FeatureBA> > & wordReferences,
		bool rematchFeatures,
		bool useLinkTransformAsGuess,
		ParametersMap registrationParameters)
{
	UDEBUG("rematchFeatures=%d", rematchFeatures?1:0);
	int wordCount = 0;
	int edgeWithWordsAdded = 0;

	// Some defaults if not provided
	registrationParameters.insert(ParametersPair(Parameters::kVisEstimationType(), "1"));
	registrationParameters.insert(ParametersPair(Parameters::kVisPnPReprojError(), "5"));
	registrationParameters.insert(ParametersPair(Parameters::kVisMinInliers(), "6"));
	registrationParameters.insert(ParametersPair(Parameters::kVisCorNNDR(), "0.6"));
	RegistrationVis reg(registrationParameters);

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

				if((sFrom.sensorData().cameraModels().empty() && sFrom.sensorData().stereoCameraModels().empty()) ||
				   (sTo.sensorData().cameraModels().empty() && sTo.sensorData().stereoCameraModels().empty()))
				{
					UERROR("No camera models found");
					continue;
				}

				if(sTo.getWeight() < 0)
				{
					for(std::multimap<int, Link>::const_iterator jter=links.find(sTo.id());
						sTo.getWeight() < 0 && jter!=links.end() &&  uContains(signatures, jter->second.to());
						++jter)
					{
						sTo = signatures.at(jter->second.to());
					}
				}

				if(sFrom.getWordsKpts().size() &&
					sTo.getWordsKpts().size() &&
					sFrom.getWords3().size())
				{
					if(!rematchFeatures)
					{
						sFrom.setWordsDescriptors(cv::Mat());
						sTo.setWordsDescriptors(cv::Mat());
					}
					else if(sFrom.getWordsDescriptors().empty() && sTo.getWordsDescriptors().empty())
					{
						UWARN("Rematching features is enabled but signatures (%d and %d) don't have word descriptors!? "
							  "Features won't be rematched. If it is an old database, do rtabmap-reprocess "
							  "so that signatures contain word desriptors.",
							  sFrom.id(), sTo.id());
					}

					RegistrationInfo info;
					Transform t = reg.computeTransformationMod(sFrom, sTo, useLinkTransformAsGuess?iter->second.transform():Transform(), &info);
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

						// The observations of each word, one per camera of the rig seeing it
						std::multimap<int, int> observationsFrom;
						std::map<int, int> bestFrom = util3d::uniqueWordObservations(sFrom.getWords(), sFrom.getWordsKpts(), sFrom.getWords3(), subImageWidthOf(sFrom), &observationsFrom);
						std::multimap<int, int> observationsTo;
						std::map<int, int> bestTo = util3d::uniqueWordObservations(sTo.getWords(), sTo.getWordsKpts(), sTo.getWords3(), subImageWidthOf(sTo), &observationsTo);

						// The word of a keypoint already added, if any
						auto wordOf = [&frameToWordMap](const Signature & s, const std::multimap<int, int> & observations, int id)
						{
							std::map<int, std::map<cv::KeyPoint, int, KeyPointCompare> >::const_iterator frameIter = frameToWordMap.find(s.id());
							if(frameIter != frameToWordMap.end())
							{
								for(std::multimap<int, int>::const_iterator jter=observations.lower_bound(id); jter!=observations.end() && jter->first == id; ++jter)
								{
									std::map<cv::KeyPoint, int, KeyPointCompare>::const_iterator kter = frameIter->second.find(s.getWordsKpts()[jter->second]);
									if(kter != frameIter->second.end())
									{
										return kter->second;
									}
								}
							}
							return -1;
						};
						// Adds the observations of a word not added yet, one per camera.
						// The depth is the one of the point along the camera's optical axis.
						auto addObservations = [&frameToWordMap, &wordReferences](const Signature & s, const std::multimap<int, int> & observations, int id, int wordId, const cv::Point3f & pointInBase)
						{
							std::map<cv::KeyPoint, int, KeyPointCompare> & frameWords = frameToWordMap[s.id()];
							for(std::multimap<int, int>::const_iterator jter=observations.lower_bound(id); jter!=observations.end() && jter->first == id; ++jter)
							{
								cv::KeyPoint kpt = s.getWordsKpts()[jter->second];
								if(frameWords.find(kpt) != frameWords.end())
								{
									continue;
								}
								frameWords.insert(std::make_pair(kpt, wordId));

								cv::Mat descriptor;
								if(!s.getWordsDescriptors().empty())
								{
									UASSERT(jter->second < s.getWordsDescriptors().rows);
									descriptor = s.getWordsDescriptors().row(jter->second);
								}
								int cameraIndex = 0;
								int subImageWidth = subImageWidthOf(s);
								if(subImageWidth > 0)
								{
									cameraIndex = int(kpt.pt.x / subImageWidth);
									kpt.pt.x = kpt.pt.x - (subImageWidth*float(cameraIndex));
								}
								float depth = 0.0f;
								if(util3d::isFinite(pointInBase))
								{
									const Transform & localTransform = !s.sensorData().cameraModels().empty()?
											s.sensorData().cameraModels()[cameraIndex].localTransform():
											s.sensorData().stereoCameraModels()[cameraIndex].localTransform();
									depth = util3d::transformPoint(pointInBase, localTransform.inverse()).z;
								}
								wordReferences.at(wordId).insert(std::make_pair(s.id(), FeatureBA(kpt, depth, descriptor, cameraIndex)));
							}
						};

						for(unsigned int i=0; i<info.inliersIDs.size(); ++i)
						{
							const int id = info.inliersIDs[i];
							UASSERT(bestFrom.find(id) != bestFrom.end());
							const cv::Point3f & p = sFrom.getWords3()[bestFrom.at(id)];
							if(p.x > 0.0f) // make sure the point is valid
							{
								// find if the word is already added
								int wordId = wordOf(sFrom, observationsFrom, id);
								if(wordId == -1)
								{
									wordId = wordOf(sTo, observationsTo, id);
								}
								if(wordId == -1)
								{
									wordId = ++wordCount;
									wordReferences.insert(std::make_pair(wordId, std::multimap<int, FeatureBA>()));
									points3DMap.insert(std::make_pair(wordId, util3d::transformPoint(p, pose)));
								}
								else
								{
									UASSERT(wordReferences.find(wordId) != wordReferences.end());
									UASSERT(points3DMap.find(wordId) != points3DMap.end());
								}

								addObservations(sFrom, observationsFrom, id, wordId, p);
								// the point as the "to" frame sees it, for the depth of its observations
								cv::Point3f pTo(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
								if(!sTo.getWords3().empty() && bestTo.find(id) != bestTo.end())
								{
									pTo = sTo.getWords3()[bestTo.at(id)];
								}
								addObservations(sTo, observationsTo, id, wordId, pTo);
							}
						}
						++edgeWithWordsAdded;
					}
					else
					{
						UWARN("Not enough inliers (%d) between %d and %d", (int)info.inliersIDs.size(), sFrom.id(), sTo.id());
					}
				}
			}
		}
	}
	UDEBUG("Added %d words (edges with words=%d/%d)", wordCount, edgeWithWordsAdded, (int)links.size());
	if(links.empty())
	{
		UERROR("No links found for BA?!");
	}
	else if(wordCount == 0)
	{
		UERROR("No words added for BA?!");
	}
}

} /* namespace rtabmap */
