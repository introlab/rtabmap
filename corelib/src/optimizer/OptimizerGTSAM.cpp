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
#include "rtabmap/core/Graph.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/core/util3d.h>
#include <set>

#include <rtabmap/core/optimizer/OptimizerGTSAM.h>

#ifdef RTABMAP_GTSAM
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/sam/BearingFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <optimizer/gtsam/XYFactor.h>
#include <optimizer/gtsam/XYZFactor.h>
#include <optimizer/gtsam/PlanarBodyZFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

#ifdef RTABMAP_VERTIGO
#include "vertigo/gtsam/betweenFactorSwitchable.h"
#include "vertigo/gtsam/switchVariableLinear.h"
#endif
#endif // end RTABMAP_GTSAM

namespace rtabmap {

OptimizerGTSAM::OptimizerGTSAM(const ParametersMap & parameters) :
	Optimizer(parameters),
	internalOptimizerType_(Parameters::defaultGTSAMOptimizer()),
	pixelVariance_(Parameters::defaultOptimizerPixelVariance()),
	disparityVariance_(Parameters::defaultOptimizerDisparityVariance()),
	robustKernelDelta_(Parameters::defaultOptimizerRobustKernelDelta()),
	baseline_(Parameters::defaultOptimizerBaseline()),
	isam2_(0),
	lastSwitchId_(1000000000)
{
	lastRootFactorIndex_.first = 0;
	parseParameters(parameters);
}

OptimizerGTSAM::~OptimizerGTSAM()
{
#ifdef RTABMAP_GTSAM
	delete isam2_;
#endif
}

bool OptimizerGTSAM::available()
{
#ifdef RTABMAP_GTSAM
	return true;
#else
	return false;
#endif
}

void OptimizerGTSAM::parseParameters(const ParametersMap & parameters)
{
	Optimizer::parseParameters(parameters);
#ifdef RTABMAP_GTSAM
	Parameters::parse(parameters, Parameters::kGTSAMOptimizer(), internalOptimizerType_);
	Parameters::parse(parameters, Parameters::kOptimizerPixelVariance(), pixelVariance_);
	Parameters::parse(parameters, Parameters::kOptimizerDisparityVariance(), disparityVariance_);
	Parameters::parse(parameters, Parameters::kOptimizerRobustKernelDelta(), robustKernelDelta_);
	Parameters::parse(parameters, Parameters::kOptimizerBaseline(), baseline_);
	UASSERT(pixelVariance_ > 0.0);
	UASSERT(disparityVariance_ > 0.0);
	UASSERT(baseline_ >= 0.0);

	bool incremental = isam2_;
	double threshold = Parameters::defaultGTSAMIncRelinearizeThreshold();
	int skip = Parameters::defaultGTSAMIncRelinearizeSkip();
	Parameters::parse(parameters, Parameters::kGTSAMIncremental(), incremental);
	Parameters::parse(parameters, Parameters::kGTSAMIncRelinearizeThreshold(), threshold);
	Parameters::parse(parameters, Parameters::kGTSAMIncRelinearizeSkip(), skip);
	UDEBUG("GTSAM %s=%d", Parameters::kGTSAMOptimizer().c_str(), internalOptimizerType_);
	UDEBUG("GTSAM %s=%d", Parameters::kGTSAMIncremental().c_str(), incremental?1:0);
	UDEBUG("GTSAM %s=%f", Parameters::kGTSAMIncRelinearizeThreshold().c_str(), threshold);
	UDEBUG("GTSAM %s=%d", Parameters::kGTSAMIncRelinearizeSkip().c_str(), skip);
	if(incremental && !isam2_)
	{
		gtsam::ISAM2Params::OptimizationParams optParams;
		if(internalOptimizerType_==2)
		{
			optParams = gtsam::ISAM2DoglegParams();
		}
		else
		{
			optParams = gtsam::ISAM2GaussNewtonParams();
		}
		gtsam::ISAM2Params params(optParams);
		params.relinearizeThreshold = threshold;
		params.relinearizeSkip = skip;
		params.evaluateNonlinearError = true;
		isam2_ = new gtsam::ISAM2(params);

		addedPoses_.clear();
		lastAddedConstraints_.clear();
		lastRootFactorIndex_.first = 0;
		lastSwitchId_ = 1000000000;
	}
	else if(!incremental && isam2_)
	{
		delete isam2_;
		isam2_ = 0;
	}
#endif
}

std::map<int, Transform> OptimizerGTSAM::optimize(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		cv::Mat & outputCovariance,
		std::list<std::map<int, Transform> > * intermediateGraphes,
		double * finalError,
		int * iterationsDone)
{
	outputCovariance = cv::Mat::eye(6,6,CV_64FC1);
	std::map<int, Transform> optimizedPoses;
#ifdef RTABMAP_GTSAM

#ifndef RTABMAP_VERTIGO
	if(this->isRobust())
	{
		UWARN("Vertigo robust optimization is not available! Robust optimization is now disabled.");
		setRobust(false);
	}
#endif

	UDEBUG("Optimizing graph...");
	if(edgeConstraints.size()>=1 && poses.size()>=2 && iterations() > 0)
	{
		gtsam::NonlinearFactorGraph graph;

		// detect if there is a global pose prior set, if so remove rootId
		bool hasGPSPrior = false;
		bool hasGravityConstraints = false;
		if(!priorsIgnored() || (!isSlam2d() && gravitySigma() > 0))
		{
			for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
			{
				if(iter->second.from() == iter->second.to())
				{
					if(!priorsIgnored() && iter->second.type() == Link::kPosePrior)
					{
						hasGPSPrior = true;
						if ((isSlam2d() && 1 / static_cast<double>(iter->second.infMatrix().at<double>(5,5)) < 9999) ||
							(1 / static_cast<double>(iter->second.infMatrix().at<double>(3,3)) < 9999.0 &&
							 1 / static_cast<double>(iter->second.infMatrix().at<double>(4,4)) < 9999.0 &&
							 1 / static_cast<double>(iter->second.infMatrix().at<double>(5,5)) < 9999.0))
						{
							// orientation is set, don't set root prior (it is no GPS)
							rootId = 0;
							hasGPSPrior = false;
							break;
						}
					}
					if(iter->second.type() == Link::kGravity)
					{
						hasGravityConstraints = true;
						if(priorsIgnored())
						{
							break;
						}
					}
				}
			}
		}

		std::vector<ConstraintToFactor> addedPrior;
		gtsam::FactorIndices removeFactorIndices;

		//prior first pose
		if(rootId != 0 && (!isam2_ || lastRootFactorIndex_.first != rootId))
		{
			UDEBUG("Setting prior for rootId=%d", rootId);
			UASSERT(uContains(poses, rootId));
			const Transform & initialPose = poses.at(rootId);
			UDEBUG("hasGPSPrior=%s", hasGPSPrior?"true":"false");
			if(isSlam2d())
			{
				gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector3(0.01, 0.01, hasGPSPrior?1e-2:1e-9));
				graph.add(gtsam::PriorFactor<gtsam::Pose2>(rootId, gtsam::Pose2(initialPose.x(), initialPose.y(), initialPose.theta()), priorNoise));
				addedPrior.push_back(ConstraintToFactor(rootId, rootId, -1));
			}
			else
			{
				gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances(
						(gtsam::Vector(6) <<
								(hasGravityConstraints?2:1e-2), (hasGravityConstraints?2:1e-2), (hasGPSPrior?1e-2:1e-9), // roll, pitch, fixed yaw if there are no priors
								(hasGPSPrior?2:1e-2), hasGPSPrior?2:1e-2, hasGPSPrior?2:1e-2 // xyz
								).finished());
				graph.add(gtsam::PriorFactor<gtsam::Pose3>(rootId, gtsam::Pose3(initialPose.toEigen4d()), priorNoise));
				addedPrior.push_back(ConstraintToFactor(rootId, rootId, -1));
			}
			if(isam2_ && lastRootFactorIndex_.first!=0)
			{
				if(uContains(poses, lastRootFactorIndex_.first))
				{
					UDEBUG("isam2: switching rootid from %d to %d", lastRootFactorIndex_.first, rootId);
					removeFactorIndices.push_back(lastRootFactorIndex_.second);
				}
				else
				{
					UDEBUG("isam2: reset iSAM2, disjoint mapping sessions between previous root %d and new root %d", lastRootFactorIndex_.first, rootId);
					// reset iSAM2, disjoint mapping session
					gtsam::ISAM2Params params = isam2_->params();
					delete isam2_;
					isam2_ = new gtsam::ISAM2(params);
					addedPoses_.clear();
					lastAddedConstraints_.clear();
					isLandmarkWithRotation_.clear();
					lastRootFactorIndex_.first = 0;
					lastSwitchId_ = 1000000000;
				}
				lastRootFactorIndex_.first = 0;
			}
		}

		std::map<int, Transform> newPoses;
		std::multimap<int, Link> newEdgeConstraints;

		if(isam2_)
		{
			UDEBUG("Add new poses...");
			// new poses?
			for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				if(addedPoses_.find(iter->first) == addedPoses_.end())
				{
					newPoses.insert(*iter);
					UDEBUG("Adding pose %d to factor graph", iter->first);
				}
			}
			UDEBUG("Add new links...");
			// new links?
			for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
			{
				if(addedPoses_.find(iter->second.from()) == addedPoses_.end() ||
				   addedPoses_.find(iter->second.to()) == addedPoses_.end())
				{
					newEdgeConstraints.insert(*iter);
					UDEBUG("Adding constraint %d (%d->%d) to factor graph", iter->first, iter->second.from(), iter->second.to());
				}
			}

			if(!this->isRobust())
			{
				UDEBUG("Remove links...");
				// Remove constraints not there anymore in case the last loop closures were rejected.
				// As we don't track "switch" constraints, we don't support this if vertigo is used.
				for(size_t i=0; i<lastAddedConstraints_.size(); ++i)
				{
					if(lastAddedConstraints_[i].from != lastAddedConstraints_[i].to &&
					   graph::findLink(edgeConstraints, lastAddedConstraints_[i].from, lastAddedConstraints_[i].to) == edgeConstraints.end())
					{
						removeFactorIndices.push_back(lastAddedConstraints_[i].factorIndice);
						UDEBUG("Removing constraint %d->%d (factor indice=%ld)",
								lastAddedConstraints_[i].from,
								lastAddedConstraints_[i].to,
								lastAddedConstraints_[i].factorIndice);
					}
				}
			}
			else if(poses.rbegin()->first >= 1000000000)
			{
				UERROR("Lastest pose id (%d) is too huge for robust optimization (over switch offset of 1000000000)", poses.rbegin()->first);
				return optimizedPoses;
			}

			lastAddedConstraints_ = addedPrior;
		}
		else
		{
			newPoses = poses;
			newEdgeConstraints = edgeConstraints;
		}

		UDEBUG("fill poses to gtsam... rootId=%d (priorsIgnored=%d landmarksIgnored=%d)",
				rootId, priorsIgnored()?1:0, landmarksIgnored()?1:0);
		gtsam::Values initialEstimate;
		// In batch (non-iSAM2) mode each optimize() call is independent.
		// In iSAM2 mode the map persists so we can resolve landmarks added
		// in a previous incremental call but referenced by a new edge.
		if(!isam2_)
		{
			isLandmarkWithRotation_.clear();
		}
		for(std::map<int, Transform>::const_iterator iter = newPoses.begin(); iter!=newPoses.end(); ++iter)
		{
			UASSERT(!iter->second.isNull());
			if(isSlam2d())
			{
				if(iter->first > 0)
				{
					initialEstimate.insert(iter->first, gtsam::Pose2(iter->second.x(), iter->second.y(), iter->second.theta()));
					addedPoses_.insert(iter->first);
				}
				else if(!landmarksIgnored())
				{
					// check if it is SE2 or only PointXY
					std::multimap<int, Link>::const_iterator jter=newEdgeConstraints.find(iter->first);
					UASSERT_MSG(jter != newEdgeConstraints.end(), uFormat("Not found landmark %d in edges!", iter->first).c_str());

					if (1 / static_cast<double>(jter->second.infMatrix().at<double>(5,5)) >= 9999.0)
					{
						initialEstimate.insert(iter->first, gtsam::Point2(iter->second.x(), iter->second.y()));
						isLandmarkWithRotation_.insert(std::make_pair(iter->first, false));
					}
					else
					{
						initialEstimate.insert(iter->first, gtsam::Pose2(iter->second.x(), iter->second.y(), iter->second.theta()));
						isLandmarkWithRotation_.insert(std::make_pair(iter->first, true));
					}
					addedPoses_.insert(iter->first);
				}

			}
			else
			{
				if(iter->first > 0)
				{
					initialEstimate.insert(iter->first, gtsam::Pose3(iter->second.toEigen4d()));
					addedPoses_.insert(iter->first);
				}
				else if(!landmarksIgnored())
				{
					// check if it is SE3 or only PointXYZ
					std::multimap<int, Link>::const_iterator jter=newEdgeConstraints.find(iter->first);
					UASSERT_MSG(jter != newEdgeConstraints.end(), uFormat("Not found landmark %d in edges!", iter->first).c_str());

					if (1 / static_cast<double>(jter->second.infMatrix().at<double>(3,3)) >= 9999.0 ||
						1 / static_cast<double>(jter->second.infMatrix().at<double>(4,4)) >= 9999.0 ||
						1 / static_cast<double>(jter->second.infMatrix().at<double>(5,5)) >= 9999.0)
					{
						initialEstimate.insert(iter->first, gtsam::Point3(iter->second.x(), iter->second.y(), iter->second.z()));
						isLandmarkWithRotation_.insert(std::make_pair(iter->first, false));
					}
					else
					{
						initialEstimate.insert(iter->first, gtsam::Pose3(iter->second.toEigen4d()));
						isLandmarkWithRotation_.insert(std::make_pair(iter->first, true));
					}
					addedPoses_.insert(iter->first);
				}
			}
		}

		UDEBUG("fill edges to gtsam...");
		if(!isam2_)
		{
			lastSwitchId_ = newPoses.rbegin()->first+1;
		}
		for(std::multimap<int, Link>::const_iterator iter=newEdgeConstraints.begin(); iter!=newEdgeConstraints.end(); ++iter)
		{
			int id1 = iter->second.from();
			int id2 = iter->second.to();

            UASSERT_MSG(poses.find(id1)!=poses.end(), uFormat("id1=%d for constraint %d->%d (type=%d)", id1, id1, id2, iter->second.type()).c_str());
            UASSERT_MSG(poses.find(id2)!=poses.end(), uFormat("id2=%d for constraint %d->%d (type=%d)", id2, id1, id2, iter->second.type()).c_str());

			UASSERT(!iter->second.transform().isNull());
			if(id1 == id2)
			{
				if(iter->second.type() == Link::kPosePrior && !priorsIgnored() &&
				  (!landmarksIgnored() || id1>0))
				{
					if(isSlam2d())
					{
						if(id1 < 0 && !isLandmarkWithRotation_.at(id1))
						{
							gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector2(
									1/iter->second.infMatrix().at<double>(0,0),
									1/iter->second.infMatrix().at<double>(1,1)));
							graph.add(XYFactor<gtsam::Point2>(id1, gtsam::Point2(iter->second.transform().x(), iter->second.transform().y()), model));
							lastAddedConstraints_.push_back(ConstraintToFactor(id1, id1, -1));
						}
						else if (1 / static_cast<double>(iter->second.infMatrix().at<double>(5,5)) >= 9999.0)
						{
							gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector2(
									1/iter->second.infMatrix().at<double>(0,0),
									1/iter->second.infMatrix().at<double>(1,1)));
							graph.add(XYFactor<gtsam::Pose2>(id1, gtsam::Point2(iter->second.transform().x(), iter->second.transform().y()), model));
							lastAddedConstraints_.push_back(ConstraintToFactor(id1, id1, -1));
						}
						else
						{
							Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
							if(!isCovarianceIgnored())
							{
								information(0,0) = iter->second.infMatrix().at<double>(0,0); // x-x
								information(0,1) = iter->second.infMatrix().at<double>(0,1); // x-y
								information(0,2) = iter->second.infMatrix().at<double>(0,5); // x-theta
								information(1,0) = iter->second.infMatrix().at<double>(1,0); // y-x
								information(1,1) = iter->second.infMatrix().at<double>(1,1); // y-y
								information(1,2) = iter->second.infMatrix().at<double>(1,5); // y-theta
								information(2,0) = iter->second.infMatrix().at<double>(5,0); // theta-x
								information(2,1) = iter->second.infMatrix().at<double>(5,1); // theta-y
								information(2,2) = iter->second.infMatrix().at<double>(5,5); // theta-theta
							}

							gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Gaussian::Information(information);
							graph.add(gtsam::PriorFactor<gtsam::Pose2>(id1, gtsam::Pose2(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().theta()), model));
							lastAddedConstraints_.push_back(ConstraintToFactor(id1, id1, -1));
						}
					}
					else
					{
						if(id1 < 0 && !isLandmarkWithRotation_.at(id1))
						{
							gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Precisions(gtsam::Vector3(
										iter->second.infMatrix().at<double>(0,0),
										iter->second.infMatrix().at<double>(1,1),
										iter->second.infMatrix().at<double>(2,2)));
							graph.add(XYZFactor<gtsam::Point3>(id1, gtsam::Point3(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().z()), model));
							lastAddedConstraints_.push_back(ConstraintToFactor(id1, id1, -1));
						}
						else if (1 / static_cast<double>(iter->second.infMatrix().at<double>(3,3)) >= 9999.0 ||
							1 / static_cast<double>(iter->second.infMatrix().at<double>(4,4)) >= 9999.0 ||
							1 / static_cast<double>(iter->second.infMatrix().at<double>(5,5)) >= 9999.0)
						{
							gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Precisions(gtsam::Vector3(
										iter->second.infMatrix().at<double>(0,0),
										iter->second.infMatrix().at<double>(1,1),
										iter->second.infMatrix().at<double>(2,2)));
							graph.add(XYZFactor<gtsam::Pose3>(id1, gtsam::Point3(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().z()), model));
							lastAddedConstraints_.push_back(ConstraintToFactor(id1, id1, -1));
						}
						else
						{
							Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
							if(!isCovarianceIgnored())
							{
								memcpy(information.data(), iter->second.infMatrix().data, iter->second.infMatrix().total()*sizeof(double));
							}

							Eigen::Matrix<double, 6, 6> mgtsam = Eigen::Matrix<double, 6, 6>::Identity();
							mgtsam.block(0,0,3,3) = information.block(3,3,3,3); // cov rotation
							mgtsam.block(3,3,3,3) = information.block(0,0,3,3); // cov translation
							mgtsam.block(0,3,3,3) = information.block(3,0,3,3); // off diagonal
							mgtsam.block(3,0,3,3) = information.block(0,3,3,3); // off diagonal
							gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(mgtsam);

							graph.add(gtsam::PriorFactor<gtsam::Pose3>(id1, gtsam::Pose3(iter->second.transform().toEigen4d()), model));
							lastAddedConstraints_.push_back(ConstraintToFactor(id1, id1, -1));
						}
					}
				}
				else if(!isSlam2d() && gravitySigma() > 0 && iter->second.type() == Link::kGravity && newPoses.find(iter->first) != newPoses.end())
				{
					gtsam::Rot3 nRbMeas = gtsam::Pose3(iter->second.transform().toEigen4d()).rotation();
					gtsam::Unit3 nZ(0,0,1);
					gtsam::Unit3 bGMeas = nRbMeas.unrotate(nZ);
					gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(2, gravitySigma());
#if GTSAM_VERSION_NUMERIC <= 40300
					// Note: till 40301 is officially released, version 40300 with "4.3a1" would fail here.
					//       Just replace "<=" above by "<" to use AttitudeFactor<Pose3> below.
					graph.add(gtsam::Pose3AttitudeFactor(iter->first, nZ, model, bGMeas));
#else
					graph.add(gtsam::AttitudeFactor<gtsam::Pose3>(iter->first, nZ, model, bGMeas));
#endif
					lastAddedConstraints_.push_back(ConstraintToFactor(iter->first, iter->first, -1));
				}
			}
			else if(id1<0 || id2 < 0)
			{
				if(!landmarksIgnored())
				{
					//landmarks
					UASSERT((id1 < 0 && id2 > 0) || (id1 > 0 && id2 < 0));
					Transform t;
					if(id2 < 0)
					{
						t = iter->second.transform();
					}
					else
					{
						t = iter->second.transform().inverse();
						std::swap(id1, id2); // should be node -> landmark
					}
					UASSERT(isLandmarkWithRotation_.find(id2) != isLandmarkWithRotation_.end());
#ifdef RTABMAP_VERTIGO
					if(this->isRobust() && isLandmarkWithRotation_.at(id2))
					{
						// create new switch variable
						// Sunderhauf IROS 2012:
						// "Since it is reasonable to initially accept all loop closure constraints,
						//  a proper and convenient initial value for all switch variables would be
						//  sij = 1 when using the linear switch function"
						double prior = 1.0;
						initialEstimate.insert(gtsam::Symbol('s',lastSwitchId_), vertigo::SwitchVariableLinear(prior));

						// create switch prior factor
						// "If the front-end is not able to assign sound individual values
						//  for Ξij , it is save to set all Ξij = 1, since this value is close
						//  to the individual optimal choice of Ξij for a large range of
						//  outliers."
						gtsam::noiseModel::Diagonal::shared_ptr switchPriorModel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(1.0));
						graph.add(gtsam::PriorFactor<vertigo::SwitchVariableLinear> (gtsam::Symbol('s',lastSwitchId_), vertigo::SwitchVariableLinear(prior), switchPriorModel));
					}
					else if(this->isRobust() && !isLandmarkWithRotation_.at(id2))
					{
						UWARN("%s cannot be used for landmark constraints without orientation.", Parameters::kOptimizerRobust().c_str());
					}
#endif

					if(isSlam2d())
					{
						if(isLandmarkWithRotation_.at(id2))
						{
							Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
							if(!isCovarianceIgnored())
							{
								information(0,0) = iter->second.infMatrix().at<double>(0,0); // x-x
								information(0,1) = iter->second.infMatrix().at<double>(0,1); // x-y
								information(0,2) = iter->second.infMatrix().at<double>(0,5); // x-theta
								information(1,0) = iter->second.infMatrix().at<double>(1,0); // y-x
								information(1,1) = iter->second.infMatrix().at<double>(1,1); // y-y
								information(1,2) = iter->second.infMatrix().at<double>(1,5); // y-theta
								information(2,0) = iter->second.infMatrix().at<double>(5,0); // theta-x
								information(2,1) = iter->second.infMatrix().at<double>(5,1); // theta-y
								information(2,2) = iter->second.infMatrix().at<double>(5,5); // theta-theta
							}
							gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Gaussian::Information(information);

#ifdef RTABMAP_VERTIGO
							if(this->isRobust())
							{
								// create switchable edge factor
								graph.add(vertigo::BetweenFactorSwitchableLinear<gtsam::Pose2>(id1, id2, gtsam::Symbol('s', lastSwitchId_++), gtsam::Pose2(t.x(), t.y(), t.theta()), model));
							}
							else
#endif
							{
								graph.add(gtsam::BetweenFactor<gtsam::Pose2>(id1, id2, gtsam::Pose2(t.x(), t.y(), t.theta()), model));
								lastAddedConstraints_.push_back(ConstraintToFactor(id1, id2, -1));
							}
						}
						else if(1 / static_cast<double>(iter->second.infMatrix().at<double>(1,1)) < 9999)
						{
							Eigen::Matrix<double, 2, 2> information = Eigen::Matrix<double, 2, 2>::Identity();
							if(!isCovarianceIgnored())
							{
								cv::Mat linearCov = cv::Mat(iter->second.infMatrix(), cv::Range(0,2), cv::Range(0,2)).clone();;
								memcpy(information.data(), linearCov.data, linearCov.total()*sizeof(double));
							}
							gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(information);

							gtsam::Point2 landmark(t.x(), t.y());
							gtsam::Pose2 p;
							graph.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(id1, id2, p.bearing(landmark), p.range(landmark), model));
							lastAddedConstraints_.push_back(ConstraintToFactor(id1, id2, -1));
						}
						else
						{
							Eigen::Matrix<double, 1, 1> information = Eigen::Matrix<double, 1, 1>::Identity();
							if(!isCovarianceIgnored())
							{
								cv::Mat linearCov = cv::Mat(iter->second.infMatrix(), cv::Range(0,1), cv::Range(0,1)).clone();;
								memcpy(information.data(), linearCov.data, linearCov.total()*sizeof(double));
							}
							gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(information);

							gtsam::Point2 landmark(t.x(), t.y());
							gtsam::Pose2 p;
							graph.add(gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2>(id1, id2, p.bearing(landmark), model));
							lastAddedConstraints_.push_back(ConstraintToFactor(id1, id2, -1));
						}
					}
					else
					{
						if(isLandmarkWithRotation_.at(id2))
						{
							Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
							if(!isCovarianceIgnored())
							{
								memcpy(information.data(), iter->second.infMatrix().data, iter->second.infMatrix().total()*sizeof(double));
							}

							Eigen::Matrix<double, 6, 6> mgtsam = Eigen::Matrix<double, 6, 6>::Identity();
							mgtsam.block(0,0,3,3) = information.block(3,3,3,3); // cov rotation
							mgtsam.block(3,3,3,3) = information.block(0,0,3,3); // cov translation
							mgtsam.block(0,3,3,3) = information.block(3,0,3,3); // off diagonal
							mgtsam.block(3,0,3,3) = information.block(0,3,3,3); // off diagonal
							gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(mgtsam);

#ifdef RTABMAP_VERTIGO
							if(this->isRobust() &&
							   iter->second.type() != Link::kNeighbor &&
							   iter->second.type() != Link::kNeighborMerged)
							{
								// create switchable edge factor
								graph.add(vertigo::BetweenFactorSwitchableLinear<gtsam::Pose3>(id1, id2, gtsam::Symbol('s', lastSwitchId_++), gtsam::Pose3(t.toEigen4d()), model));
							}
							else
#endif
							{
								graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id1, id2, gtsam::Pose3(t.toEigen4d()), model));
								lastAddedConstraints_.push_back(ConstraintToFactor(id1, id2, -1));
							}
						}
						else if(1 / static_cast<double>(iter->second.infMatrix().at<double>(2,2)) < 9999)
						{
							Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
							if(!isCovarianceIgnored())
							{
								cv::Mat linearCov = cv::Mat(iter->second.infMatrix(), cv::Range(0,3), cv::Range(0,3)).clone();
								memcpy(information.data(), linearCov.data, linearCov.total()*sizeof(double));
							}

							gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(information);

							gtsam::Point3 landmark(t.x(), t.y(), t.z());
							gtsam::Pose3 p;
							graph.add(gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>(id1, id2, p.bearing(landmark), p.range(landmark), model));
							lastAddedConstraints_.push_back(ConstraintToFactor(id1, id2, -1));
						}
						else
						{
							Eigen::Matrix<double, 2, 2> information = Eigen::Matrix<double, 2, 2>::Identity();
							if(!isCovarianceIgnored())
							{
								cv::Mat linearCov = cv::Mat(iter->second.infMatrix(), cv::Range(0,2), cv::Range(0,2)).clone();
								memcpy(information.data(), linearCov.data, linearCov.total()*sizeof(double));
							}

							gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(information);

							gtsam::Point3 landmark(t.x(), t.y(), t.z());
							gtsam::Pose3 p;
							graph.add(gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3>(id1, id2, p.bearing(landmark), model));
							lastAddedConstraints_.push_back(ConstraintToFactor(id1, id2, -1));
						}
					}
				}
			}
			else // id1 != id2
			{
#ifdef RTABMAP_VERTIGO
				if(this->isRobust() &&
				   iter->second.type() != Link::kNeighbor &&
				   iter->second.type() != Link::kNeighborMerged)
				{
					// create new switch variable
					// Sunderhauf IROS 2012:
					// "Since it is reasonable to initially accept all loop closure constraints,
					//  a proper and convenient initial value for all switch variables would be
					//  sij = 1 when using the linear switch function"
					double prior = 1.0;
					initialEstimate.insert(gtsam::Symbol('s',lastSwitchId_), vertigo::SwitchVariableLinear(prior));

					// create switch prior factor
					// "If the front-end is not able to assign sound individual values
					//  for Ξij , it is save to set all Ξij = 1, since this value is close
					//  to the individual optimal choice of Ξij for a large range of
					//  outliers."
					gtsam::noiseModel::Diagonal::shared_ptr switchPriorModel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(1.0));
					graph.add(gtsam::PriorFactor<vertigo::SwitchVariableLinear> (gtsam::Symbol('s',lastSwitchId_), vertigo::SwitchVariableLinear(prior), switchPriorModel));
				}
#endif
				if(isSlam2d())
				{
					Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
					if(!isCovarianceIgnored())
					{
						information(0,0) = iter->second.infMatrix().at<double>(0,0); // x-x
						information(0,1) = iter->second.infMatrix().at<double>(0,1); // x-y
						information(0,2) = iter->second.infMatrix().at<double>(0,5); // x-theta
						information(1,0) = iter->second.infMatrix().at<double>(1,0); // y-x
						information(1,1) = iter->second.infMatrix().at<double>(1,1); // y-y
						information(1,2) = iter->second.infMatrix().at<double>(1,5); // y-theta
						information(2,0) = iter->second.infMatrix().at<double>(5,0); // theta-x
						information(2,1) = iter->second.infMatrix().at<double>(5,1); // theta-y
						information(2,2) = iter->second.infMatrix().at<double>(5,5); // theta-theta
					}
					gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Gaussian::Information(information);

#ifdef RTABMAP_VERTIGO
					if(this->isRobust() &&
					   iter->second.type()!=Link::kNeighbor &&
					   iter->second.type() != Link::kNeighborMerged)
					{
						// create switchable edge factor
						graph.add(vertigo::BetweenFactorSwitchableLinear<gtsam::Pose2>(id1, id2, gtsam::Symbol('s', lastSwitchId_++), gtsam::Pose2(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().theta()), model));
					}
					else
#endif
					{
						graph.add(gtsam::BetweenFactor<gtsam::Pose2>(id1, id2, gtsam::Pose2(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().theta()), model));
						lastAddedConstraints_.push_back(ConstraintToFactor(id1, id2, -1));
					}
				}
				else
				{
					Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
					if(!isCovarianceIgnored())
					{
						memcpy(information.data(), iter->second.infMatrix().data, iter->second.infMatrix().total()*sizeof(double));
					}

					Eigen::Matrix<double, 6, 6> mgtsam = Eigen::Matrix<double, 6, 6>::Identity();
					mgtsam.block(0,0,3,3) = information.block(3,3,3,3); // cov rotation
					mgtsam.block(3,3,3,3) = information.block(0,0,3,3); // cov translation
					mgtsam.block(0,3,3,3) = information.block(3,0,3,3); // off diagonal
					mgtsam.block(3,0,3,3) = information.block(0,3,3,3); // off diagonal
					gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(mgtsam);

#ifdef RTABMAP_VERTIGO
					if(this->isRobust() &&
					   iter->second.type() != Link::kNeighbor &&
					   iter->second.type() != Link::kNeighborMerged)
					{
						// create switchable edge factor
						graph.add(vertigo::BetweenFactorSwitchableLinear<gtsam::Pose3>(id1, id2, gtsam::Symbol('s', lastSwitchId_++), gtsam::Pose3(iter->second.transform().toEigen4d()), model));
					}
					else
#endif
					{
						graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id1, id2, gtsam::Pose3(iter->second.transform().toEigen4d()), model));
						lastAddedConstraints_.push_back(ConstraintToFactor(id1, id2, -1));
					}
				}
			}
		}

		UDEBUG("create optimizer");
		gtsam::NonlinearOptimizer * optimizer = 0;

		if(!isam2_) // Batch optimization
		{
			UDEBUG("Batch optimization...");
			if(internalOptimizerType_ == 2)
			{
				gtsam::DoglegParams parameters;
				parameters.relativeErrorTol = epsilon();
				parameters.maxIterations = iterations();
				optimizer = new gtsam::DoglegOptimizer(graph, initialEstimate, parameters);
			}
			else if(internalOptimizerType_ == 1)
			{
				gtsam::GaussNewtonParams parameters;
				parameters.relativeErrorTol = epsilon();
				parameters.maxIterations = iterations();
				optimizer = new gtsam::GaussNewtonOptimizer(graph, initialEstimate, parameters);
			}
			else
			{
				gtsam::LevenbergMarquardtParams parameters;
				parameters.relativeErrorTol = epsilon();
				parameters.maxIterations = iterations();
				optimizer = new gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate, parameters);
			}
		}
		else
		{
			UDEBUG("iSAM2 optimization...");
		}

		UDEBUG("GTSAM optimizing begin (max iterations=%d, robust=%d)", iterations(), isRobust()?1:0);
		UTimer timer;
		int it = 0;
		double initialError = optimizer?graph.error(initialEstimate):0;
		double lastError = optimizer?optimizer->error():0;
		for(int i=0; i<iterations(); ++i)
		{
			if(intermediateGraphes && i > 0)
			{
				float x,y,z,roll,pitch,yaw;
				std::map<int, Transform> tmpPoses;
				const gtsam::Values values = isam2_?isam2_->calculateEstimate():optimizer->values();
#if GTSAM_VERSION_NUMERIC >= 40200
				for(gtsam::Values::deref_iterator iter=values.begin(); iter!=values.end(); ++iter)
#else
				for(gtsam::Values::const_iterator iter=values.begin(); iter!=values.end(); ++iter)
#endif
				{
					int key = (int)iter->key;
					if(iter->value.dim() > 1 && uContains(newPoses, key))
					{
						if(isSlam2d())
						{
							if(key > 0)
							{
								gtsam::Pose2 p = iter->value.cast<gtsam::Pose2>();
								tmpPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), p.theta())));
							}
							else if(!landmarksIgnored() && isLandmarkWithRotation_.find(key)!=isLandmarkWithRotation_.end())
							{
								if(isLandmarkWithRotation_.at(key))
								{
									newPoses.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
									gtsam::Pose2 p = iter->value.cast<gtsam::Pose2>();
									tmpPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), z, roll, pitch, p.theta())));
								}
								else
								{
									newPoses.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
									gtsam::Point2 p = iter->value.cast<gtsam::Point2>();
									tmpPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), z, roll,pitch,yaw)));
								}
							}
						}
						else
						{
							if(key > 0)
							{
								gtsam::Pose3 p = iter->value.cast<gtsam::Pose3>();
								tmpPoses.insert(std::make_pair(key, Transform::fromEigen4d(p.matrix())));
							}
							else if(!landmarksIgnored() && isLandmarkWithRotation_.find(key)!=isLandmarkWithRotation_.end())
							{
								if(isLandmarkWithRotation_.at(key))
								{
									gtsam::Pose3 p = iter->value.cast<gtsam::Pose3>();
									tmpPoses.insert(std::make_pair(key, Transform::fromEigen4d(p.matrix())));
								}
								else
								{
									newPoses.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
									gtsam::Point3 p = iter->value.cast<gtsam::Point3>();
									tmpPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), p.z(), roll,pitch,yaw)));
								}
							}
						}
					}
				}
				intermediateGraphes->push_back(tmpPoses);
			}

			gtsam::ISAM2Result result;
			double error = 0;
			try
			{
				if(optimizer) // Batch optimization
				{
					optimizer->iterate();
					error = optimizer->error();
				}
				else if(i==0) // iSAM2 (add factors)
				{
					UDEBUG("Update iSAM with the new factors");
					result = isam2_->update(graph, initialEstimate, removeFactorIndices);
#if BOOST_VERSION >= 106800
					UASSERT(result.errorBefore.has_value());
					UASSERT(result.errorAfter.has_value());
#else
					UASSERT(result.errorBefore.is_initialized());
					UASSERT(result.errorAfter.is_initialized());
#endif
					UDEBUG("error before = %f after=%f", result.errorBefore.value(), result.errorAfter.value());
					initialError = lastError = result.errorBefore.value();
					error = result.errorAfter.value();
					if(!this->isRobust())
					{
						UASSERT_MSG(lastAddedConstraints_.size() == result.newFactorsIndices.size(),
								uFormat("%ld versus %ld", lastAddedConstraints_.size(), result.newFactorsIndices.size()).c_str());
						for(size_t j=0; j<result.newFactorsIndices.size(); ++j)
						{
							UDEBUG("New factor indice: %ld", result.newFactorsIndices[j]);
							lastAddedConstraints_[j].factorIndice = result.newFactorsIndices[j];
						}
					}
					if(rootId != 0 && lastRootFactorIndex_.first == 0)
					{
						UASSERT(result.newFactorsIndices.size()>=1);
						lastRootFactorIndex_.first = rootId;
						lastRootFactorIndex_.second = result.newFactorsIndices[0]; // first one should be always the root prior
					}
				}
				else // iSAM2 (more iterations)
				{
					result = isam2_->update();
#if BOOST_VERSION >= 106800
					UASSERT(result.errorBefore.has_value());
					UASSERT(result.errorAfter.has_value());
#else
					UASSERT(result.errorBefore.is_initialized());
					UASSERT(result.errorAfter.is_initialized());
#endif
					UDEBUG("error before = %f after=%f", result.errorBefore.value(), result.errorAfter.value());

					lastError = result.errorBefore.value();
					error = result.errorAfter.value();
				}
				++it;
			}
			catch(gtsam::IndeterminantLinearSystemException & e)
			{
				UWARN("GTSAM exception caught: %s\n Graph has %d edges and %d vertices", e.what(),
						(int)newEdgeConstraints.size(),
						(int)newPoses.size());
				delete optimizer;
				if(isam2_)
				{
					// We are in bad state, cleanup
					UDEBUG("Reset iSAM2!");
					gtsam::ISAM2Params params = isam2_->params();
					delete isam2_;
					isam2_ = new gtsam::ISAM2(params);
					addedPoses_.clear();
					lastAddedConstraints_.clear();
					lastRootFactorIndex_.first = 0;
					lastSwitchId_ = 1000000000;
				}
				return optimizedPoses;
			}

			// early stop condition
			UDEBUG("iteration %d error =%f", i+1, error);
			double errorDelta = lastError - error;
			if(this->epsilon() > 0.0 && fabs(error) > 1000000000000.0)
			{
				UWARN("Error computed (%e) is very huge and/or diverging! Aborting! "
					   "Set %s to 0 to ignore that check and keep iterating up to %s (%d).",
					   error,
					   Parameters::kOptimizerEpsilon().c_str(),
					   Parameters::kOptimizerIterations().c_str(),
					   this->iterations());
				return optimizedPoses;
			}
			else
			{
				if((isam2_ || i>0) && errorDelta < this->epsilon())
				{
					if(errorDelta < 0)
					{
						UDEBUG("Negative improvement?! Ignore and continue optimizing... (%f < %f)", errorDelta, this->epsilon());
					}
					else
					{
						UDEBUG("Stop optimizing, not enough improvement (%f < %f)", errorDelta, this->epsilon());
						break;
					}
				}
				else if(i==0)
				{ 
					if(error < 0)
					{
						UDEBUG("Negative error?! Ignore and continue optimizing... (%f)", error);
					}
					else if(error < this->epsilon())
					{
						UINFO("Stop optimizing, error is already under epsilon (%f < %f)", error, this->epsilon());
						break;
					}
				}
			}
			lastError = error;
		}
		if(finalError)
		{
			*finalError = lastError;
		}
		if(iterationsDone)
		{
			*iterationsDone = it;
		}
		UDEBUG("GTSAM optimizing end (%d iterations done (error initial=%f final=%f), time=%f s)",
				it, initialError, lastError, timer.ticks());

		float x,y,z,roll,pitch,yaw;
		const gtsam::Values values = isam2_?isam2_->calculateEstimate():optimizer->values();
#if GTSAM_VERSION_NUMERIC >= 40200
		for(gtsam::Values::deref_iterator iter=values.begin(); iter!=values.end(); ++iter)
#else
		for(gtsam::Values::const_iterator iter=values.begin(); iter!=values.end(); ++iter)
#endif
		{
			int key = (int)iter->key;
			if(iter->value.dim() > 1 && uContains(poses, key))
			{
				if(isSlam2d())
				{
					if(key > 0)
					{
						poses.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
						gtsam::Pose2 p = iter->value.cast<gtsam::Pose2>();
						optimizedPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), z, roll, pitch, p.theta())));
					}
					else if(!landmarksIgnored() && isLandmarkWithRotation_.find(key)!=isLandmarkWithRotation_.end())
					{
						if(isLandmarkWithRotation_.at(key))
						{
							poses.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
							gtsam::Pose2 p = iter->value.cast<gtsam::Pose2>();
							optimizedPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), z, roll, pitch, p.theta())));
						}
						else
						{
							poses.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
							gtsam::Point2 p = iter->value.cast<gtsam::Point2>();
							optimizedPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), z,roll,pitch,yaw)));
						}
					}
				}
				else
				{
					if(key > 0)
					{
						gtsam::Pose3 p = iter->value.cast<gtsam::Pose3>();
						optimizedPoses.insert(std::make_pair(key, Transform::fromEigen4d(p.matrix())));
					}
					else if(!landmarksIgnored() && isLandmarkWithRotation_.find(key)!=isLandmarkWithRotation_.end())
					{
						if(isLandmarkWithRotation_.at(key))
						{
							gtsam::Pose3 p = iter->value.cast<gtsam::Pose3>();
							optimizedPoses.insert(std::make_pair(key, Transform::fromEigen4d(p.matrix())));
						}
						else
						{
							poses.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
							gtsam::Point3 p = iter->value.cast<gtsam::Point3>();
							optimizedPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), p.z(), roll,pitch,yaw)));
						}
					}
				}
			}
		}

		// compute marginals
		try {
			UDEBUG("Computing marginals for node %d...", poses.rbegin()->first);
			UTimer t;
			gtsam::Matrix info;
			if(optimizer)
			{
				gtsam::Marginals marginals(graph, optimizer->values());
			 	info = marginals.marginalCovariance(poses.rbegin()->first);
			}
			else //iSAM2
			{
				info = isam2_->marginalCovariance(poses.rbegin()->first);
			}
			UDEBUG("Computed marginals = %fs (key=%d)", t.ticks(), poses.rbegin()->first);
			if(isSlam2d() && info.cols() == 3 && info.cols() == 3)
			{
				outputCovariance.at<double>(0,0) = info(0,0); // x-x
				outputCovariance.at<double>(0,1) = info(0,1); // x-y
				outputCovariance.at<double>(0,5) = info(0,2); // x-theta
				outputCovariance.at<double>(1,0) = info(1,0); // y-x
				outputCovariance.at<double>(1,1) = info(1,1); // y-y
				outputCovariance.at<double>(1,5) = info(1,2); // y-theta
				outputCovariance.at<double>(5,0) = info(2,0); // theta-x
				outputCovariance.at<double>(5,1) = info(2,1); // theta-y
				outputCovariance.at<double>(5,5) = info(2,2); // theta-theta
			}
			else if(!isSlam2d() && info.cols() == 6 && info.cols() == 6)
			{
				Eigen::Matrix<double, 6, 6> mgtsam = Eigen::Matrix<double, 6, 6>::Identity();
				mgtsam.block(3,3,3,3) = info.block(0,0,3,3); // cov rotation
				mgtsam.block(0,0,3,3) = info.block(3,3,3,3); // cov translation
				mgtsam.block(0,3,3,3) = info.block(3,0,3,3); // off diagonal
				mgtsam.block(3,0,3,3) = info.block(0,3,3,3); // off diagonal
				memcpy(outputCovariance.data, mgtsam.data(), outputCovariance.total()*sizeof(double));
			}
			else
			{
				UERROR("GTSAM: Could not compute marginal covariance!");
				optimizedPoses.clear(); // Failed optimization
			}
		}
		catch(gtsam::IndeterminantLinearSystemException & e)
		{
			UERROR("GTSAM exception caught: %s", e.what());
			optimizedPoses.clear(); // Failed optimization
		}
		catch(std::exception& e)
		{
			UERROR("GTSAM exception caught: %s", e.what());
			optimizedPoses.clear(); // Failed optimization
		}

		delete optimizer;
	}
	else if(poses.size() == 1 || iterations() <= 0)
	{
		optimizedPoses = poses;
	}
	else
	{
		UWARN("This method should be called at least with 1 pose!");
	}
	UDEBUG("Optimizing graph...end!");
#else
	UERROR("Not built with GTSAM support!");
#endif
	return optimizedPoses;
}

// Multi-camera offset: same convention as OptimizerG2O.cpp so per-rig camera
// vertex keys stay disjoint from pose keys (max 10 cameras per pose).
#define GTSAM_BA_MULTICAM_OFFSET 10

// Build a gtsam::Symbol for a 3D point (BA "landmark" in the optimization
// sense — NOT the same as Link::kLandmark used in pose-graph land).
// wordReferences can mix positive ids (features quantized to a visual
// vocabulary word) with negative ids (features that weren't matched to
// any vocabulary entry — see Memory.cpp's "Set ID -1 to features not
// used for quantization"). gtsam::Symbol packs the index into 56
// unsigned bits, so a signed int with the high bit set trips
// "Symbol index is too large". Use distinct prefix characters for the
// two cases so the numeric index is always non-negative AND
// positive/negative ids cannot collide on the same Symbol.
static inline gtsam::Symbol point3dSymbol(int id)
{
    return id < 0
        ? gtsam::Symbol('L', static_cast<std::uint64_t>(-id))
        : gtsam::Symbol('l', static_cast<std::uint64_t>(id));
}

std::map<int, Transform> OptimizerGTSAM::optimizeBA(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, std::vector<CameraModel> > & models,
		std::map<int, cv::Point3f> & points3DMap,
		const std::map<int, std::map<int, FeatureBA> > & wordReferences,
		std::set<int> * outliers)
{
	std::map<int, Transform> optimizedPoses;
#ifdef RTABMAP_GTSAM
	UDEBUG("Optimizing BA graph...");

	if(!(poses.size() >= 2 && iterations() > 0 && (models.size() == poses.size() || poses.begin()->first < 0)))
	{
		UWARN("GTSAM BA: nothing to optimize (poses=%d models=%d iterations=%d)",
				(int)poses.size(), (int)models.size(), iterations());
		return optimizedPoses;
	}

	gtsam::NonlinearFactorGraph graph;
	gtsam::Values initial;

	// Cache per-frame, per-camera intrinsics. Note that GTSAM's
	// GenericProjectionFactor/GenericStereoFactor hold a shared_ptr to the
	// calibration -- we have to keep these alive for the lifetime of the
	// graph, hence storing them by map.
	std::map<std::pair<int,int>, gtsam::Cal3_S2::shared_ptr>       calMono;
	std::map<std::pair<int,int>, gtsam::Cal3_S2Stereo::shared_ptr> calStereo;
	std::map<std::pair<int,int>, double>                           baselineByCam;

	// 1) Add pose variables (in CAMERA frame: pose * localTransform).
	UDEBUG("GTSAM BA: adding %d poses... (rootId=%d)", (int)poses.size(), rootId);
	for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		if(iter->first <= 0)
		{
			continue;
		}
		std::map<int, std::vector<CameraModel> >::const_iterator iterModel = models.find(iter->first);
		if(iterModel == models.end() || iterModel->second.empty())
		{
			UERROR("GTSAM BA: missing camera model for pose %d", iter->first);
			return optimizedPoses;
		}
		for(size_t i=0; i<iterModel->second.size(); ++i)
		{
			const CameraModel & m = iterModel->second[i];
			if(!m.isValidForProjection())
			{
				UERROR("GTSAM BA: model %d.%d is invalid for projection", iter->first, (int)i);
				return optimizedPoses;
			}
			const Transform camPose = iter->second * m.localTransform();
			if(camPose.isNull())
			{
				UERROR("GTSAM BA: null camera pose for %d.%d", iter->first, (int)i);
				return optimizedPoses;
			}
			const gtsam::Symbol xkey('x', iter->first * GTSAM_BA_MULTICAM_OFFSET + (int)i);
			initial.insert(xkey, gtsam::Pose3(camPose.toEigen4d()));

			// Intrinsics: skew=0 (no shear in any CameraModel rtabmap supports).
			gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(m.fx(), m.fy(), 0.0, m.cx(), m.cy()));
			calMono[std::make_pair(iter->first, (int)i)] = K;
			const double baseline = m.Tx() < 0.0 ? (-m.Tx() / m.fx()) : baseline_;
			if(baseline > 0.0)
			{
				gtsam::Cal3_S2Stereo::shared_ptr Ks(new gtsam::Cal3_S2Stereo(m.fx(), m.fy(), 0.0, m.cx(), m.cy(), baseline));
				calStereo[std::make_pair(iter->first, (int)i)]     = Ks;
				baselineByCam[std::make_pair(iter->first, (int)i)] = baseline;
			}

			// Fix the root pose (or fix everyone else if rootId<0). GTSAM has
			// no equivalent of g2o's setFixed(); the standard idiom is a
			// near-zero-sigma prior on each axis. We add this only to the
			// primary camera (i==0) of a multi-cam rig -- the others are
			// rigidly linked via the multi-cam BetweenFactors below.
			const bool fixNode = (rootId >= 0 && iter->first == rootId) ||
			                     (rootId <  0 && iter->first != -rootId);
			if(fixNode && i == 0)
			{
				gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
						gtsam::noiseModel::Diagonal::Sigmas(
								(gtsam::Vector(6) << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9).finished());
				graph.add(gtsam::PriorFactor<gtsam::Pose3>(xkey, gtsam::Pose3(camPose.toEigen4d()), priorNoise));
			}
			else if(isSlam2d() && i == 0)
			{
				// 2D / planar BA: lock the body-frame z of each non-root
				// camera to its initial value (mirrors g2o's EdgeSBACamPrior
				// with pinfo(2,2) = 1e9). Lateral motion and yaw stay free.
				const gtsam::Pose3 cam_to_body(m.localTransform().inverse().toEigen4d());
				gtsam::SharedNoiseModel planarNoise =
						gtsam::noiseModel::Isotropic::Sigma(1, std::sqrt(1.0 / 1e9));
				graph.add(PlanarBodyZFactor(xkey, cam_to_body, iter->second.z(), planarNoise));
			}
		}
	}

	// 2) Pose-graph BetweenFactors (same role as the g2o EdgeSBACam edges).
	//    Expressed in camera frame: cam_from^{-1} * world * cam_to where
	//    cam = body * localTransform.
	UDEBUG("GTSAM BA: adding %d links...", (int)links.size());
	for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
	{
		const Link & link = iter->second;
		if(link.from() <= 0 || link.to() <= 0)
		{
			continue;
		}
		if(link.from() == link.to())
		{
			continue;
		}
		if(!uContains(poses, link.from()) || !uContains(poses, link.to()))
		{
			continue;
		}
		UASSERT(!link.transform().isNull());

		const Transform camLink = models.at(link.from())[0].localTransform().inverse() *
		                          link.transform() *
		                          models.at(link.to())[0].localTransform();

		Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
		if(!isCovarianceIgnored())
		{
			memcpy(information.data(), link.infMatrix().data, link.infMatrix().total()*sizeof(double));
		}
		// rtabmap's covariance/information convention is [linear|angular];
		// GTSAM expects [angular|linear]. Swap the blocks.
		Eigen::Matrix<double, 6, 6> mgtsam;
		mgtsam.block<3,3>(0,0) = information.block<3,3>(3,3); // rotation
		mgtsam.block<3,3>(3,3) = information.block<3,3>(0,0); // translation
		mgtsam.block<3,3>(0,3) = information.block<3,3>(3,0);
		mgtsam.block<3,3>(3,0) = information.block<3,3>(0,3);
		gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(mgtsam);

		graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
				gtsam::Symbol('x', link.from() * GTSAM_BA_MULTICAM_OFFSET),
				gtsam::Symbol('x', link.to()   * GTSAM_BA_MULTICAM_OFFSET),
				gtsam::Pose3(camLink.toEigen4d()),
				noise));
	}

	// 3) Hard rigid edges between camera 0 and the other cameras of a
	//    multi-cam rig (g2o uses Identity*1e7; we mirror that here).
	for(std::map<int, std::vector<CameraModel> >::const_iterator iter=models.begin(); iter!=models.end(); ++iter)
	{
		if(!uContains(poses, iter->first))
		{
			continue;
		}
		for(size_t i=1; i<iter->second.size(); ++i)
		{
			const Transform camLink = iter->second[0].localTransform().inverse() * iter->second[i].localTransform();
			Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity() * 9999999.0;
			gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(information);
			graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
					gtsam::Symbol('x',  iter->first * GTSAM_BA_MULTICAM_OFFSET),
					gtsam::Symbol('x',  iter->first * GTSAM_BA_MULTICAM_OFFSET + (int)i),
					gtsam::Pose3(camLink.toEigen4d()),
					noise));
		}
	}

	// 4) 3D points + reprojection observations.
	UDEBUG("GTSAM BA: adding %d 3D points and observations...", (int)points3DMap.size());
	std::set<gtsam::Key> insertedPoints;
	// Track factor->word mapping so the post-optimization residual sweep can
	// report which observations went over the robust-kernel threshold.
	std::vector<std::pair<size_t /*factorIndex*/, int /*wordId*/> > obsFactors;

	// Build the per-axis noise models once (loop-invariant). Stereo: per-axis
	// sigmas matching the g2o stereo path. StereoPoint2 is (uL, uR, v); uR =
	// uL - disparity. uL and v carry pixel-detector noise, uR carries
	// disparity-channel noise (matches the g2o stereo edge's (u, v, u-disp)
	// interpretation up to a covariance rotation that is fine for typical
	// small sigmas). The robust-Huber wrapping is also invariant.
	const double sigmaPixel     = std::sqrt(pixelVariance_);
	const double sigmaDisparity = std::sqrt(disparityVariance_);
	gtsam::SharedNoiseModel stereoNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
			(gtsam::Vector(3) << sigmaPixel, sigmaDisparity, sigmaPixel).finished());
	gtsam::SharedNoiseModel monoNoiseModel = gtsam::noiseModel::Isotropic::Sigma(2, sigmaPixel);
	if(robustKernelDelta_ > 0.0)
	{
		gtsam::noiseModel::mEstimator::Base::shared_ptr huber =
				gtsam::noiseModel::mEstimator::Huber::Create(robustKernelDelta_);
		stereoNoiseModel = gtsam::noiseModel::Robust::Create(huber, stereoNoiseModel);
		monoNoiseModel   = gtsam::noiseModel::Robust::Create(huber, monoNoiseModel);
	}
	for(std::map<int, std::map<int, FeatureBA> >::const_iterator iter = wordReferences.begin(); iter!=wordReferences.end(); ++iter)
	{
		const int wordId = iter->first;
		if(points3DMap.find(wordId) == points3DMap.end())
		{
			continue;
		}
		const cv::Point3f pt3d = points3DMap.at(wordId);
		if(!util3d::isFinite(pt3d))
		{
			UWARN("Ignoring 3D point %d because it has nan value(s)!", wordId);
			continue;
		}
		const gtsam::Symbol pkey = point3dSymbol(wordId);
		initial.insert(pkey, gtsam::Point3(pt3d.x, pt3d.y, pt3d.z));
		insertedPoints.insert(pkey);

		for(std::map<int, FeatureBA>::const_iterator jter = iter->second.begin(); jter != iter->second.end(); ++jter)
		{
			const int poseId  = jter->first;
			const int camIdx  = jter->second.cameraIndex;
			const FeatureBA & f = jter->second;
			if(poses.find(poseId) == poses.end())
			{
				continue;
			}
			const std::pair<int,int> camKey(poseId, camIdx);
			if(calMono.find(camKey) == calMono.end())
			{
				continue;
			}
			const gtsam::Symbol xkey('x', poseId * GTSAM_BA_MULTICAM_OFFSET + camIdx);
			if(!initial.exists(xkey))
			{
				continue;
			}

			const double depth    = f.depth;
			const double baseline = baselineByCam.count(camKey) ? baselineByCam.at(camKey) : 0.0;
			const bool isStereo   = (uIsFinite(depth) && depth > 0.0 && baseline > 0.0 && calStereo.count(camKey));

			size_t factorIdx = graph.size();
			if(isStereo)
			{
				const gtsam::Cal3_S2Stereo::shared_ptr & Ks = calStereo.at(camKey);
				const double disparity = baseline * Ks->fx() / depth;
				const gtsam::StereoPoint2 obs(f.kpt.pt.x, f.kpt.pt.x - disparity, f.kpt.pt.y);
				graph.add(gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(
						obs, stereoNoiseModel, xkey, pkey, Ks));
			}
			else
			{
				if(baseline > 0.0)
				{
					UDEBUG("Stereo cam detected but observation (word=%d cam=%d.%d) has null depth (%f m), adding mono observation instead.",
							wordId, poseId, camIdx, depth);
				}
				const gtsam::Cal3_S2::shared_ptr & K = calMono.at(camKey);
				const gtsam::Point2 obs(f.kpt.pt.x, f.kpt.pt.y);
				graph.add(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(
						obs, monoNoiseModel, xkey, pkey, K));
			}
			obsFactors.push_back(std::make_pair(factorIdx, wordId));
		}
	}

	// 5) Optimize.
	UTimer timer;
	gtsam::Values result;
	double finalError = std::numeric_limits<double>::quiet_NaN();
	try
	{
		// Always use Levenberg-Marquardt for BA, ignoring GTSAM/Optimizer.
		// Same rationale as the g2o BA path: BA's Hessian is often
		// near-singular (points near infinity, near-parallel rays), so
		// Gauss-Newton's unbounded step can blow up. Dogleg works but
		// offers no advantage over LM on BA. LM is what every major BA
		// library (Ceres, g2o, COLMAP) defaults to.
		// Use a tight tolerance so the optimizer runs to convergence
		// instead of stopping early on GTSAM's default absoluteErrorTol
		// (1e-5), which leaves the longest-range points off truth on
		// mono BA.
		const double tol = epsilon() > 0.0 ? epsilon() : 1e-12;
		gtsam::LevenbergMarquardtParams params;
		params.relativeErrorTol = tol;
		params.absoluteErrorTol = tol;
		params.maxIterations    = iterations();
		gtsam::NonlinearOptimizer * optimizer = new gtsam::LevenbergMarquardtOptimizer(graph, initial, params);
		UDEBUG("GTSAM BA optimizing (max iterations=%d, robustKernel=%f)...", iterations(), robustKernelDelta_);
		result = optimizer->optimize();
		finalError = optimizer->error();
		UDEBUG("GTSAM BA done (initialError=%f finalError=%f time=%fs)", graph.error(initial), finalError, timer.ticks());
		delete optimizer;
	}
	catch(const gtsam::IndeterminantLinearSystemException & e)
	{
		UERROR("GTSAM BA: indeterminant linear system: %s", e.what());
		return optimizedPoses;
	}
	catch(const std::exception & e)
	{
		UERROR("GTSAM BA failed: %s", e.what());
		return optimizedPoses;
	}

	if(uIsNan(finalError))
	{
		UERROR("GTSAM BA produced a NaN error.");
		return optimizedPoses;
	}

	// 6) Report observations whose per-factor residual exceeded the robust
	//    kernel delta. Unlike g2o we don't re-optimize without them -- the
	//    Huber kernel has already down-weighted them in the solve.
	if(outliers && robustKernelDelta_ > 0.0)
	{
		const double thresholdSq = robustKernelDelta_ * robustKernelDelta_;
		for(std::vector<std::pair<size_t, int> >::const_iterator iter = obsFactors.begin(); iter != obsFactors.end(); ++iter)
		{
			if(iter->first >= graph.size()) continue;
			const double e = graph.at(iter->first)->error(result);
			// GTSAM returns 0.5 * r^T * Σ^{-1} * r; multiply by 2 to get chi^2.
			if(2.0 * e > thresholdSq)
			{
				outliers->insert(iter->second);
			}
		}
		UDEBUG("GTSAM BA: %d outlier observations flagged.", (int)outliers->size());
	}

	// 7) Read back poses (camera frame -> body frame via localTransform^-1).
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(iter->first <= 0)
		{
			continue;
		}
		const gtsam::Symbol xkey('x', iter->first * GTSAM_BA_MULTICAM_OFFSET);
		if(!result.exists(xkey))
		{
			continue;
		}
		Transform t = Transform::fromEigen4d(result.at<gtsam::Pose3>(xkey).matrix());
		t *= models.at(iter->first)[0].localTransform().inverse();
		if(t.isNull())
		{
			UERROR("GTSAM BA: optimized pose %d is null", iter->first);
			optimizedPoses.clear();
			return optimizedPoses;
		}
		if(isSlam2d())
		{
			// Same snap-back idiom as g2o / Ceres: PlanarBodyZFactor locks
			// each non-root body z to its initial value, but tiny LM-residual
			// slack can still leave a sub-mm drift. Snap z back exactly when
			// within tolerance; fall back to a 2D-projected delta otherwise.
			if(std::fabs(t.z() - iter->second.z()) < 0.001f)
			{
				t.z() = iter->second.z();
			}
			else
			{
				UWARN("Planar constraints didn't work!? original pose (%d), pose %s -> %s. Falling back to old approach.",
						iter->first,
						iter->second.prettyPrint().c_str(),
						t.prettyPrint().c_str());
				const Transform delta = iter->second.inverse() * t;
				t = iter->second * delta.to3DoF();
			}
		}
		optimizedPoses.insert(std::make_pair(iter->first, t));
	}

	// 8) Read back 3D points.
	for(std::map<int, cv::Point3f>::iterator iter = points3DMap.begin(); iter != points3DMap.end(); ++iter)
	{
		const gtsam::Symbol pkey = point3dSymbol(iter->first);
		if(insertedPoints.count(pkey) && result.exists(pkey))
		{
			const gtsam::Point3 p = result.at<gtsam::Point3>(pkey);
			iter->second = cv::Point3f(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
		}
	}

#else
	UERROR("Not built with GTSAM support!");
	(void)rootId;
	(void)poses;
	(void)links;
	(void)models;
	(void)points3DMap;
	(void)wordReferences;
	(void)outliers;
#endif
	return optimizedPoses;
}

} /* namespace rtabmap */
