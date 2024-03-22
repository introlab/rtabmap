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
#include <set>

#include <rtabmap/core/optimizer/OptimizerISAM2.h>

#ifdef RTABMAP_GTSAM
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include "gtsam/GravityFactor.h"
#include <optimizer/gtsam/XYFactor.h>
#include <optimizer/gtsam/XYZFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

#ifdef RTABMAP_VERTIGO
#include "vertigo/gtsam/betweenFactorSwitchable.h"
#include "vertigo/gtsam/switchVariableLinear.h"
#endif
#endif // end RTABMAP_VERTIGO

namespace rtabmap {

OptimizerISAM2::OptimizerISAM2(const ParametersMap & parameters) :
	Optimizer(parameters),
	isam2_(0)
{
	parseParameters(parameters);
#ifdef RTABMAP_GTSAM
	gtsam::ISAM2Params::OptimizationParams optParams;
	if(optimizer_==2)
	{
		optParams = gtsam::ISAM2DoglegParams();
	}
	else{
		optParams = gtsam::ISAM2GaussNewtonParams();
	}
	gtsam::ISAM2Params params(optParams);
	params.relinearizeThreshold = 0.01;
	params.relinearizeSkip = 1;
	params.evaluateNonlinearError = true;
	isam2_ = new ISAM2(params);
#endif
}

OptimizerISAM2::~OptimizerISAM2()
{
#ifdef RTABMAP_GTSAM
	delete isam2_;
#endif
}

bool OptimizerISAM2::available()
{
#ifdef RTABMAP_GTSAM
	return true;
#else
	return false;
#endif
}

void OptimizerISAM2::parseParameters(const ParametersMap & parameters)
{
	Optimizer::parseParameters(parameters);
	Parameters::parse(parameters, Parameters::kGTSAMOptimizer(), optimizer_);
}

std::map<int, Transform> OptimizerISAM2::optimize(
		int rootId,
		const std::map<int, Transform> & allPoses,
		const std::multimap<int, Link> & allEdgeConstraints,
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
	if(allEdgeConstraints.size()>=1 && allPoses.size()>=2 && iterations() > 0)
	{
		std::map<int, Transform> newPoses;
		std::multimap<int, Link> newEdgeConstraints;

		for(std::map<int, Transform>::const_reverse_iterator iter=allPoses.rbegin(); iter!=allPoses.rend(); ++iter)
		{
			auto inserted = addedPoses_.insert(*iter);
			if(inserted.second)
			{
				newPoses.insert(*iter);
				UDEBUG("Adding pose %d to factor graph", iter->first);
			}
			else
			{
				break;
			}
		}
		for(std::multimap<int, Link>::const_reverse_iterator iter=allEdgeConstraints.rbegin(); iter!=allEdgeConstraints.rend(); ++iter)
		{
			int largestId = iter->second.from()>iter->second.to()?iter->second.from():iter->second.to();
			if(addedEdgeConstraints_.find(largestId) == addedEdgeConstraints_.end())
			{
				newEdgeConstraints.insert(*iter);
				UDEBUG("Adding constraint %d (%d->%d) to factor graph", iter->first, iter->second.from(), iter->second.to());
			}
			else
			{
				break;
			}
		}
		addedEdgeConstraints_.insert(newEdgeConstraints.begin(), newEdgeConstraints.end());

		gtsam::NonlinearFactorGraph graph;

		// detect if there is a global pose prior set, if so remove rootId
		bool hasGPSPrior = false;
		bool hasGravityConstraints = false;
		if(!priorsIgnored() || (!isSlam2d() && gravitySigma() > 0))
		{
			for(std::multimap<int, Link>::const_iterator iter=newEdgeConstraints.begin(); iter!=newEdgeConstraints.end(); ++iter)
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

		//prior first pose
		if(rootId != 0 && uContains(newPoses, rootId))
		{
			const Transform & initialPose = newPoses.at(rootId);
			UDEBUG("hasGPSPrior=%s", hasGPSPrior?"true":"false");
			if(isSlam2d())
			{
				gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector3(0.01, 0.01, hasGPSPrior?1e-2:std::numeric_limits<double>::min()));
				graph.add(gtsam::PriorFactor<gtsam::Pose2>(rootId, gtsam::Pose2(initialPose.x(), initialPose.y(), initialPose.theta()), priorNoise));
			}
			else
			{
				gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances(
						(gtsam::Vector(6) <<
								(hasGravityConstraints?2:1e-2), (hasGravityConstraints?2:1e-2), hasGPSPrior?1e-2:std::numeric_limits<double>::min(), // roll, pitch, fixed yaw if there are no priors
								(hasGPSPrior?2:1e-2), hasGPSPrior?2:1e-2, hasGPSPrior?2:1e-2 // xyz
								).finished());
				graph.add(gtsam::PriorFactor<gtsam::Pose3>(rootId, gtsam::Pose3(initialPose.toEigen4d()), priorNoise));
			}
		}

		UDEBUG("fill poses to gtsam... rootId=%d (priorsIgnored=%d landmarksIgnored=%d)",
				rootId, priorsIgnored()?1:0, landmarksIgnored()?1:0);
		gtsam::Values initialEstimate;
		std::map<int, bool> isLandmarkWithRotation;
		for(std::map<int, Transform>::const_iterator iter = newPoses.begin(); iter!=newPoses.end(); ++iter)
		{
			UASSERT(!iter->second.isNull());
			if(isSlam2d())
			{
				if(iter->first > 0)
				{
					initialEstimate.insert(iter->first, gtsam::Pose2(iter->second.x(), iter->second.y(), iter->second.theta()));
				}
				else if(!landmarksIgnored())
				{
					// check if it is SE2 or only PointXY
					std::multimap<int, Link>::const_iterator jter=newEdgeConstraints.find(iter->first);
					UASSERT_MSG(jter != newEdgeConstraints.end(), uFormat("Not found landmark %d in edges!", iter->first).c_str());

					if (1 / static_cast<double>(jter->second.infMatrix().at<double>(5,5)) >= 9999.0)
					{
						initialEstimate.insert(iter->first, gtsam::Point2(iter->second.x(), iter->second.y()));
						isLandmarkWithRotation.insert(std::make_pair(iter->first, false));
					}
					else
					{
						initialEstimate.insert(iter->first, gtsam::Pose2(iter->second.x(), iter->second.y(), iter->second.theta()));
						isLandmarkWithRotation.insert(std::make_pair(iter->first, true));
					}
				}

			}
			else
			{
				if(iter->first > 0)
				{
					initialEstimate.insert(iter->first, gtsam::Pose3(iter->second.toEigen4d()));
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
						isLandmarkWithRotation.insert(std::make_pair(iter->first, false));
					}
					else
					{
						initialEstimate.insert(iter->first, gtsam::Pose3(iter->second.toEigen4d()));
						isLandmarkWithRotation.insert(std::make_pair(iter->first, true));
					}
				}
			}
		}

		UDEBUG("fill edges to gtsam...");
		int switchCounter = newPoses.rbegin()->first+1;
		for(std::multimap<int, Link>::const_iterator iter=newEdgeConstraints.begin(); iter!=newEdgeConstraints.end(); ++iter)
		{
			int id1 = iter->second.from();
			int id2 = iter->second.to();

            UASSERT_MSG(addedPoses_.find(id1)!=addedPoses_.end(), uFormat("id1=%d", id1).c_str());
            UASSERT_MSG(addedPoses_.find(id2)!=addedPoses_.end(), uFormat("id2=%d", id2).c_str());

			UASSERT(!iter->second.transform().isNull());
			if(id1 == id2)
			{
				if(iter->second.type() == Link::kPosePrior && !priorsIgnored() &&
				  (!landmarksIgnored() || id1>0))
				{
					if(isSlam2d())
					{
						if(id1 < 0 && !isLandmarkWithRotation.at(id1))
						{
							noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Variances(Vector2(
									1/iter->second.infMatrix().at<double>(0,0),
									1/iter->second.infMatrix().at<double>(1,1)));
							graph.add(XYFactor<gtsam::Point2>(id1, gtsam::Point2(iter->second.transform().x(), iter->second.transform().y()), model));
						}
						else if (1 / static_cast<double>(iter->second.infMatrix().at<double>(5,5)) >= 9999.0)
						{
							noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Variances(Vector2(
									1/iter->second.infMatrix().at<double>(0,0),
									1/iter->second.infMatrix().at<double>(1,1)));
							graph.add(XYFactor<gtsam::Pose2>(id1, gtsam::Point2(iter->second.transform().x(), iter->second.transform().y()), model));
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
						}
					}
					else
					{
						if(id1 < 0 && !isLandmarkWithRotation.at(id1))
						{
							noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Precisions(Vector3(
										iter->second.infMatrix().at<double>(0,0),
										iter->second.infMatrix().at<double>(1,1),
										iter->second.infMatrix().at<double>(2,2)));
							graph.add(XYZFactor<gtsam::Point3>(id1, gtsam::Point3(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().z()), model));
						}
						else if (1 / static_cast<double>(iter->second.infMatrix().at<double>(3,3)) >= 9999.0 ||
							1 / static_cast<double>(iter->second.infMatrix().at<double>(4,4)) >= 9999.0 ||
							1 / static_cast<double>(iter->second.infMatrix().at<double>(5,5)) >= 9999.0)
						{
							noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Precisions(Vector3(
										iter->second.infMatrix().at<double>(0,0),
										iter->second.infMatrix().at<double>(1,1),
										iter->second.infMatrix().at<double>(2,2)));
							graph.add(XYZFactor<gtsam::Pose3>(id1, gtsam::Point3(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().z()), model));
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
						}
					}
				}
				else if(!isSlam2d() && gravitySigma() > 0 && iter->second.type() == Link::kGravity && newPoses.find(iter->first) != newPoses.end())
				{
					Vector3 r = gtsam::Pose3(iter->second.transform().toEigen4d()).rotation().xyz();
					gtsam::Unit3 nG = gtsam::Rot3::RzRyRx(r.x(), r.y(), 0).rotate(gtsam::Unit3(0,0,-1));
					gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector2(gravitySigma(), 10));
					graph.add(Pose3GravityFactor(iter->first, nG, model, Unit3(0,0,1)));
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
					if(isSlam2d())
					{
						if(isLandmarkWithRotation.at(id2))
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
							graph.add(gtsam::BetweenFactor<gtsam::Pose2>(id1, id2, gtsam::Pose2(t.x(), t.y(), t.theta()), model));
						}
						else
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
						}
					}
					else
					{
						if(isLandmarkWithRotation.at(id2))
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
							graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id1, id2, gtsam::Pose3(t.toEigen4d()), model));
						}
						else
						{
							Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
							if(!isCovarianceIgnored())
							{
								cv::Mat linearCov = cv::Mat(iter->second.infMatrix(), cv::Range(0,3), cv::Range(0,3)).clone();;
								memcpy(information.data(), linearCov.data, linearCov.total()*sizeof(double));
							}
							gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(information);

							gtsam::Point3 landmark(t.x(), t.y(), t.z());
							gtsam::Pose3 p;
							graph.add(gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>(id1, id2, p.bearing(landmark), p.range(landmark), model));
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
					initialEstimate.insert(gtsam::Symbol('s',switchCounter), vertigo::SwitchVariableLinear(prior));

					// create switch prior factor
					// "If the front-end is not able to assign sound individual values
					//  for Ξij , it is save to set all Ξij = 1, since this value is close
					//  to the individual optimal choice of Ξij for a large range of
					//  outliers."
					gtsam::noiseModel::Diagonal::shared_ptr switchPriorModel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(1.0));
					graph.add(gtsam::PriorFactor<vertigo::SwitchVariableLinear> (gtsam::Symbol('s',switchCounter), vertigo::SwitchVariableLinear(prior), switchPriorModel));
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
						graph.add(vertigo::BetweenFactorSwitchableLinear<gtsam::Pose2>(id1, id2, gtsam::Symbol('s', switchCounter++), gtsam::Pose2(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().theta()), model));
					}
					else
#endif
					{
						graph.add(gtsam::BetweenFactor<gtsam::Pose2>(id1, id2, gtsam::Pose2(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().theta()), model));
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
						graph.add(vertigo::BetweenFactorSwitchableLinear<gtsam::Pose3>(id1, id2, gtsam::Symbol('s', switchCounter++), gtsam::Pose3(iter->second.transform().toEigen4d()), model));
					}
					else
#endif
					{
						graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id1, id2, gtsam::Pose3(iter->second.transform().toEigen4d()), model));
					}
				}
			}
		}

		gtsam::FactorIndices removeFactorIndices;
		
		UDEBUG("GTSAM optimizing begin (max iterations=%d, robust=%d)", iterations(), isRobust()?1:0);
		UTimer timer;
		int it = 0;
		double initialError = 0;
		double lastError = 0;
		for(int i=0; i<iterations(); ++i)
		{
			if(intermediateGraphes && i > 0)
			{
				float x,y,z,roll,pitch,yaw;
				std::map<int, Transform> tmpPoses;
				Values values = isam2_->calculateEstimate();
#if GTSAM_VERSION_NUMERIC >= 40200
				for(gtsam::Values::deref_iterator iter=values.begin(); iter!=values.end(); ++iter)
#else
				for(gtsam::Values::iterator iter=values.begin(); iter!=values.end(); ++iter)
#endif
				{
					if(iter->value.dim() > 1)
					{
						int key = (int)iter->key;
						UASSERT(addedPoses_.find(key) != addedPoses_.end());
						if(isSlam2d())
						{
							if(key > 0)
							{
								gtsam::Pose2 p = iter->value.cast<gtsam::Pose2>();
								tmpPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), p.theta())));
							}
							else if(!landmarksIgnored() && isLandmarkWithRotation.find(key)!=isLandmarkWithRotation.end())
							{
								if(isLandmarkWithRotation.at(key))
								{
									addedPoses_.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
									gtsam::Pose2 p = iter->value.cast<gtsam::Pose2>();
									tmpPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), z, roll, pitch, p.theta())));
								}
								else
								{
									addedPoses_.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
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
							else if(!landmarksIgnored() && isLandmarkWithRotation.find(key)!=isLandmarkWithRotation.end())
							{
								if(isLandmarkWithRotation.at(key))
								{
									gtsam::Pose3 p = iter->value.cast<gtsam::Pose3>();
									tmpPoses.insert(std::make_pair(key, Transform::fromEigen4d(p.matrix())));
								}
								else
								{
									addedPoses_.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
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
			try
			{
				if(i==0)
				{
					UDEBUG("Update iSAM with the new factors");
					result = isam2_->update(graph, initialEstimate, removeFactorIndices);
					UDEBUG("error before = %f after=%f", result.errorBefore.value_or(-1), result.errorAfter.value_or(-1));
					for(size_t j=result.newFactorsIndices.size(); j<result.newFactorsIndices.size(); ++j)
					{
						UDEBUG("New factor indice: %d", j);
					}
					result.newFactorsIndices;
				}
				else
				{
					result = isam2_->update();
					UDEBUG("error before = %f after=%f", result.errorBefore.value_or(-1), result.errorAfter.value_or(-1));

					// early stop condition
					UASSERT(result.errorBefore.has_value());
					UASSERT(result.errorAfter.has_value());
					double previousError = result.errorBefore.value();
					double error = result.errorAfter.value();
					UDEBUG("iteration %d error =%f", i+1, error);
					double errorDelta = previousError - error;
					if(i>0 && errorDelta < this->epsilon())
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
					else if(i==0 && error < this->epsilon())
					{
						UINFO("Stop optimizing, error is already under epsilon (%f < %f)", error, this->epsilon());
						break;
					}
					lastError = error;
				}
				++it;
			}
			catch(gtsam::IndeterminantLinearSystemException & e)
			{
				UWARN("GTSAM exception caught: %s\n Graph has %d edges and %d vertices", e.what(),
						(int)addedEdgeConstraints_.size(),
						(int)addedPoses_.size());
				return optimizedPoses;
			}
		}
		if(finalError)
		{
			*finalError = lastError;
		}
		if(iterationsDone)
		{
			*iterationsDone = it;
		}
		UDEBUG("GTSAM optimizing end (%d iterations done (initial=%f final=%f), time=%f s)",
				it, initialError, lastError, timer.ticks());

		float x,y,z,roll,pitch,yaw;
		Values values = isam2_->calculateEstimate();
#if GTSAM_VERSION_NUMERIC >= 40200
		for(gtsam::Values::deref_iterator iter=values.begin(); iter!=values.end(); ++iter)
#else
		for(gtsam::Values::iterator iter=values.begin(); iter!=values.end(); ++iter)
#endif
		{
			if(iter->value.dim() > 1)
			{
				int key = (int)iter->key;
				UASSERT(addedPoses_.find(key) != addedPoses_.end());
				if(isSlam2d())
				{
					if(key > 0)
					{
						addedPoses_.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
						gtsam::Pose2 p = iter->value.cast<gtsam::Pose2>();
						optimizedPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), z, roll, pitch, p.theta())));
					}
					else if(!landmarksIgnored() && isLandmarkWithRotation.find(key)!=isLandmarkWithRotation.end())
					{
						if(isLandmarkWithRotation.at(key))
						{
							addedPoses_.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
							gtsam::Pose2 p = iter->value.cast<gtsam::Pose2>();
							optimizedPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), z, roll, pitch, p.theta())));
						}
						else
						{
							addedPoses_.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
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
					else if(!landmarksIgnored() && isLandmarkWithRotation.find(key)!=isLandmarkWithRotation.end())
					{
						if(isLandmarkWithRotation.at(key))
						{
							gtsam::Pose3 p = iter->value.cast<gtsam::Pose3>();
							optimizedPoses.insert(std::make_pair(key, Transform::fromEigen4d(p.matrix())));
						}
						else
						{
							addedPoses_.at(key).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
							gtsam::Point3 p = iter->value.cast<gtsam::Point3>();
							optimizedPoses.insert(std::make_pair(key, Transform(p.x(), p.y(), p.z(), roll,pitch,yaw)));
						}
					}
				}
			}
		}

		// compute marginals
		try {
			UDEBUG("Computing marginals...");
			UTimer t;
			gtsam::Matrix info = isam2_->marginalCovariance(addedPoses_.rbegin()->first);
			UDEBUG("Computed marginals = %fs (key=%d)", t.ticks(), addedPoses_.rbegin()->first);
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
	}
	else if(allPoses.size() == 1 || iterations() <= 0)
	{
		optimizedPoses = allPoses;
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

} /* namespace rtabmap */
