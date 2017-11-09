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
#include <rtabmap/utilite/UTimer.h>
#include <set>

#include <rtabmap/core/OptimizerG2O.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_motion_estimation.h>
#include <rtabmap/core/util3d.h>

#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM2)
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/linear_solver.h"

#ifdef RTABMAP_G2O
#include "g2o/types/sba/types_sba.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/config.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"
#ifdef G2O_HAVE_CSPARSE
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#endif
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#ifdef G2O_HAVE_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#endif
enum {
    PARAM_OFFSET=0,
};
#endif // RTABMAP_G2O

#ifdef RTABMAP_ORB_SLAM2
#include "g2o/types/types_sba.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/solvers/linear_solver_eigen.h"
#endif

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearEigenSolver;
#ifdef RTABMAP_G2O
typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
#ifdef G2O_HAVE_CSPARSE
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
#endif
#ifdef G2O_HAVE_CHOLMOD
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
#endif

#ifdef RTABMAP_VERTIGO
#include "vertigo/g2o/edge_switchPrior.h"
#include "vertigo/g2o/edge_se2Switchable.h"
#include "vertigo/g2o/edge_se3Switchable.h"
#include "vertigo/g2o/vertex_switchLinear.h"
#endif
#endif

#endif // end defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM2)

namespace rtabmap {

bool OptimizerG2O::available()
{
#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM2)
	return true;
#else
	return false;
#endif
}

bool OptimizerG2O::isCSparseAvailable()
{
#ifdef G2O_HAVE_CSPARSE
	return true;
#else
	return false;
#endif
}

bool OptimizerG2O::isCholmodAvailable()
{
#ifdef G2O_HAVE_CHOLMOD
	return true;
#else
	return false;
#endif
}

void OptimizerG2O::parseParameters(const ParametersMap & parameters)
{
	Optimizer::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kg2oSolver(), solver_);
	Parameters::parse(parameters, Parameters::kg2oOptimizer(), optimizer_);
	Parameters::parse(parameters, Parameters::kg2oPixelVariance(), pixelVariance_);
	Parameters::parse(parameters, Parameters::kg2oRobustKernelDelta(), robustKernelDelta_);
	Parameters::parse(parameters, Parameters::kg2oBaseline(), baseline_);
	UASSERT(pixelVariance_ > 0.0);
	UASSERT(baseline_ >= 0.0);

#ifdef RTABMAP_ORB_SLAM2
	if(solver_ != 3)
	{
		UWARN("g2o built with ORB_SLAM2 has only Eigen solver available, using Eigen=3 instead of %d.", solver_);
		solver_ = 3;
	}
#else
#ifndef G2O_HAVE_CHOLMOD
	if(solver_ == 2)
	{
		UWARN("g2o is not built with chmold, so it cannot be used as solver. Using CSparse instead.");
		solver_ = 0;
	}
#endif

#ifndef G2O_HAVE_CSPARSE
	if(solver_ == 0)
	{
		UWARN("g2o is not built with csparse, so it cannot be used as solver. Using PCG instead.");
		solver_ = 1;
	}
#endif
#endif

}

std::map<int, Transform> OptimizerG2O::optimize(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		std::list<std::map<int, Transform> > * intermediateGraphes,
		double * finalError,
		int * iterationsDone)
{
	std::map<int, Transform> optimizedPoses;
#ifdef RTABMAP_G2O
	UDEBUG("Optimizing graph...");

#ifndef RTABMAP_VERTIGO
	if(this->isRobust())
	{
		UWARN("Vertigo robust optimization is not available! Robust optimization is now disabled.");
		setRobust(false);
	}
#endif

	optimizedPoses.clear();
	if(edgeConstraints.size()>=1 && poses.size()>=2 && iterations() > 0)
	{
		// Apply g2o optimization

		g2o::SparseOptimizer optimizer;
		optimizer.setVerbose(ULogger::level()==ULogger::kDebug);
		g2o::ParameterSE3Offset* odomOffset = new g2o::ParameterSE3Offset();
		odomOffset->setId(PARAM_OFFSET);
		optimizer.addParameter(odomOffset);

		SlamBlockSolver * blockSolver = 0;

		if(solver_ == 3)
		{
			//eigen
			SlamLinearEigenSolver * linearSolver = new SlamLinearEigenSolver();
			linearSolver->setBlockOrdering(false);
			blockSolver = new SlamBlockSolver(linearSolver);
		}
		else if(solver_ == 2)
		{
#ifdef G2O_HAVE_CHOLMOD
			//chmold
			SlamLinearCholmodSolver * linearSolver = new SlamLinearCholmodSolver();
			linearSolver->setBlockOrdering(false);
			blockSolver = new SlamBlockSolver(linearSolver);
#endif
		}
		else if(solver_ == 0)
		{
#ifdef G2O_HAVE_CSPARSE
			//csparse
			SlamLinearCSparseSolver* linearSolver = new SlamLinearCSparseSolver();
			linearSolver->setBlockOrdering(false);
			blockSolver = new SlamBlockSolver(linearSolver);
#endif
		}

		if(blockSolver == 0)
		{
			//pcg
			SlamLinearPCGSolver * linearSolver = new SlamLinearPCGSolver();
			blockSolver = new SlamBlockSolver(linearSolver);
		}

		if(optimizer_ == 1)
		{
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(blockSolver));
		}
		else
		{
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(blockSolver));
		}

		// detect if there is a global pose prior set, if so remove rootId
		if(!priorsIgnored())
		{
			for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
			{
				if(iter->second.from() == iter->second.to())
				{
					rootId = 0;
					break;
				}
			}
		}

		UDEBUG("fill poses to g2o...");
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			UASSERT(!iter->second.isNull());
			g2o::HyperGraph::Vertex * vertex = 0;
			if(isSlam2d())
			{
				g2o::VertexSE2 * v2 = new g2o::VertexSE2();
				v2->setEstimate(g2o::SE2(iter->second.x(), iter->second.y(), iter->second.theta()));
				if(iter->first == rootId)
				{
					v2->setFixed(true);
				}
				vertex = v2;
			}
			else
			{
				g2o::VertexSE3 * v3 = new g2o::VertexSE3();

				Eigen::Affine3d a = iter->second.toEigen3d();
				Eigen::Isometry3d pose;
				pose = a.linear();
				pose.translation() = a.translation();
				v3->setEstimate(pose);
				if(iter->first == rootId)
				{
					v3->setFixed(true);
				}
				vertex = v3;
			}
			vertex->setId(iter->first);
			UASSERT_MSG(optimizer.addVertex(vertex), uFormat("cannot insert vertex %d!?", iter->first).c_str());
		}

		UDEBUG("fill edges to g2o...");
		int vertigoVertexId = poses.rbegin()->first+1;
		for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
		{
			int id1 = iter->second.from();
			int id2 = iter->second.to();

			UASSERT(!iter->second.transform().isNull());

			g2o::HyperGraph::Edge * edge = 0;

			if(id1 == id2)
			{
				if(!priorsIgnored())
				{
					if(isSlam2d())
					{
						g2o::EdgeSE2Prior * priorEdge = new g2o::EdgeSE2Prior();
						g2o::VertexSE2* v1 = (g2o::VertexSE2*)optimizer.vertex(id1);
						priorEdge->setVertex(0, v1);
						priorEdge->setMeasurement(g2o::SE2(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().theta()));
						priorEdge->setParameterId(0, PARAM_OFFSET);
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
						priorEdge->setInformation(information);
						edge = priorEdge;
					}
					else
					{
						g2o::EdgeSE3Prior * priorEdge = new g2o::EdgeSE3Prior();
						g2o::VertexSE3* v1 = (g2o::VertexSE3*)optimizer.vertex(id1);
						priorEdge->setVertex(0, v1);
						Eigen::Affine3d a = iter->second.transform().toEigen3d();
						Eigen::Isometry3d pose;
						pose = a.linear();
						pose.translation() = a.translation();
						priorEdge->setMeasurement(pose);
						priorEdge->setParameterId(0, PARAM_OFFSET);
						Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
						if(!isCovarianceIgnored())
						{
							memcpy(information.data(), iter->second.infMatrix().data, iter->second.infMatrix().total()*sizeof(double));
						}
						priorEdge->setInformation(information);
						edge = priorEdge;
					}
				}
			}
			else
			{
#ifdef RTABMAP_VERTIGO
				VertexSwitchLinear * v = 0;
				if(this->isRobust() &&
				   iter->second.type() != Link::kNeighbor &&
				   iter->second.type() != Link::kNeighborMerged)
				{
					// For loop closure links, add switchable edges

					// create new switch variable
					// Sunderhauf IROS 2012:
					// "Since it is reasonable to initially accept all loop closure constraints,
					//  a proper and convenient initial value for all switch variables would be
					//  sij = 1 when using the linear switch function"
					v = new VertexSwitchLinear();
					v->setEstimate(1.0);
					v->setId(vertigoVertexId++);
					UASSERT_MSG(optimizer.addVertex(v), uFormat("cannot insert switchable vertex %d!?", v->id()).c_str());

					// create switch prior factor
					// "If the front-end is not able to assign sound individual values
					//  for Ξij , it is save to set all Ξij = 1, since this value is close
					//  to the individual optimal choice of Ξij for a large range of
					//  outliers."
					EdgeSwitchPrior * prior = new EdgeSwitchPrior();
					prior->setMeasurement(1.0);
					prior->setVertex(0, v);
					UASSERT_MSG(optimizer.addEdge(prior), uFormat("cannot insert switchable prior edge %d!?", v->id()).c_str());
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

#ifdef RTABMAP_VERTIGO
					if(this->isRobust() &&
					   iter->second.type() != Link::kNeighbor  &&
					   iter->second.type() != Link::kNeighborMerged)
					{
						EdgeSE2Switchable * e = new EdgeSE2Switchable();
						g2o::VertexSE2* v1 = (g2o::VertexSE2*)optimizer.vertex(id1);
						g2o::VertexSE2* v2 = (g2o::VertexSE2*)optimizer.vertex(id2);
						UASSERT(v1 != 0);
						UASSERT(v2 != 0);
						e->setVertex(0, v1);
						e->setVertex(1, v2);
						e->setVertex(2, v);
						e->setMeasurement(g2o::SE2(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().theta()));
						e->setInformation(information);
						edge = e;
					}
					else
#endif
					{
						g2o::EdgeSE2 * e = new g2o::EdgeSE2();
						g2o::VertexSE2* v1 = (g2o::VertexSE2*)optimizer.vertex(id1);
						g2o::VertexSE2* v2 = (g2o::VertexSE2*)optimizer.vertex(id2);
						UASSERT(v1 != 0);
						UASSERT(v2 != 0);
						e->setVertex(0, v1);
						e->setVertex(1, v2);
						e->setMeasurement(g2o::SE2(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().theta()));
						e->setInformation(information);
						edge = e;
					}
				}
				else
				{
					Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
					if(!isCovarianceIgnored())
					{
						memcpy(information.data(), iter->second.infMatrix().data, iter->second.infMatrix().total()*sizeof(double));
					}

					Eigen::Affine3d a = iter->second.transform().toEigen3d();
					Eigen::Isometry3d constraint;
					constraint = a.linear();
					constraint.translation() = a.translation();

#ifdef RTABMAP_VERTIGO
					if(this->isRobust() &&
					   iter->second.type() != Link::kNeighbor &&
					   iter->second.type() != Link::kNeighborMerged)
					{
						EdgeSE3Switchable * e = new EdgeSE3Switchable();
						g2o::VertexSE3* v1 = (g2o::VertexSE3*)optimizer.vertex(id1);
						g2o::VertexSE3* v2 = (g2o::VertexSE3*)optimizer.vertex(id2);
						UASSERT(v1 != 0);
						UASSERT(v2 != 0);
						e->setVertex(0, v1);
						e->setVertex(1, v2);
						e->setVertex(2, v);
						e->setMeasurement(constraint);
						e->setInformation(information);
						edge = e;
					}
					else
#endif
					{
						g2o::EdgeSE3 * e = new g2o::EdgeSE3();
						g2o::VertexSE3* v1 = (g2o::VertexSE3*)optimizer.vertex(id1);
						g2o::VertexSE3* v2 = (g2o::VertexSE3*)optimizer.vertex(id2);
						UASSERT(v1 != 0);
						UASSERT(v2 != 0);
						e->setVertex(0, v1);
						e->setVertex(1, v2);
						e->setMeasurement(constraint);
						e->setInformation(information);
						edge = e;
					}
				}
			}

			if (edge && !optimizer.addEdge(edge))
			{
				delete edge;
				UERROR("Map: Failed adding constraint between %d and %d, skipping", id1, id2);
			}


		}

		UDEBUG("Initial optimization...");
		optimizer.initializeOptimization();

		UASSERT(optimizer.verifyInformationMatrices());

		UINFO("g2o optimizing begin (max iterations=%d, robust=%d)", iterations(), isRobust()?1:0);
		int it = 0;
		UTimer timer;
		double lastError = 0.0;
		if(intermediateGraphes || this->epsilon() > 0.0)
		{
			for(int i=0; i<iterations(); ++i)
			{
				if(intermediateGraphes)
				{
					if(i > 0)
					{
						std::map<int, Transform> tmpPoses;
						if(isSlam2d())
						{
							for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
							{
								const g2o::VertexSE2* v = (const g2o::VertexSE2*)optimizer.vertex(iter->first);
								if(v)
								{
									float roll, pitch, yaw;
									iter->second.getEulerAngles(roll, pitch, yaw);
									Transform t(v->estimate().translation()[0], v->estimate().translation()[1], iter->second.z(), roll, pitch, v->estimate().rotation().angle());
									tmpPoses.insert(std::pair<int, Transform>(iter->first, t));
									UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
								}
								else
								{
									UERROR("Vertex %d not found!?", iter->first);
								}
							}
						}
						else
						{
							for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
							{
								const g2o::VertexSE3* v = (const g2o::VertexSE3*)optimizer.vertex(iter->first);
								if(v)
								{
									Transform t = Transform::fromEigen3d(v->estimate());
									tmpPoses.insert(std::pair<int, Transform>(iter->first, t));
									UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
								}
								else
								{
									UERROR("Vertex %d not found!?", iter->first);
								}
							}
						}
						intermediateGraphes->push_back(tmpPoses);
					}
				}

				it += optimizer.optimize(1);

				// early stop condition
				optimizer.computeActiveErrors();
				double chi2 = optimizer.activeRobustChi2();
				UDEBUG("iteration %d: %d nodes, %d edges, chi2: %f", i, (int)optimizer.vertices().size(), (int)optimizer.edges().size(), chi2);

				if(i>0 && optimizer.activeRobustChi2() > 1000000000000.0)
				{
					UWARN("g2o: Large optimimzation error detected (%f), aborting optimization!");
					return optimizedPoses;
				}

				double errorDelta = lastError - chi2;
				if(i>0 && errorDelta < this->epsilon())
				{
					if(errorDelta < 0)
					{
						UDEBUG("Negative improvement?! Ignore and continue optimizing... (%f < %f)", errorDelta, this->epsilon());
					}
					else
					{
						UINFO("Stop optimizing, not enough improvement (%f < %f)", errorDelta, this->epsilon());
						break;
					}
				}
				else if(i==0 && chi2 < this->epsilon())
				{
					UINFO("Stop optimizing, error is already under epsilon (%f < %f)", chi2, this->epsilon());
					break;
				}
				lastError = chi2;
			}
		}
		else
		{
			it = optimizer.optimize(iterations());
			optimizer.computeActiveErrors();
			UDEBUG("%d nodes, %d edges, chi2: %f", (int)optimizer.vertices().size(), (int)optimizer.edges().size(), optimizer.activeRobustChi2());
		}
		if(finalError)
		{
			*finalError = lastError;
		}
		if(iterationsDone)
		{
			*iterationsDone = it;
		}
		UINFO("g2o optimizing end (%d iterations done, error=%f, time = %f s)", it, optimizer.activeRobustChi2(), timer.ticks());

		if(optimizer.activeRobustChi2() > 1000000000000.0)
		{
			UWARN("g2o: Large optimimzation error detected (%f), aborting optimization!");
			return optimizedPoses;
		}

		if(isSlam2d())
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				const g2o::VertexSE2* v = (const g2o::VertexSE2*)optimizer.vertex(iter->first);
				if(v)
				{
					float roll, pitch, yaw;
					iter->second.getEulerAngles(roll, pitch, yaw);
					Transform t(v->estimate().translation()[0], v->estimate().translation()[1], iter->second.z(), roll, pitch, v->estimate().rotation().angle());
					optimizedPoses.insert(std::pair<int, Transform>(iter->first, t));
					UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
				}
				else
				{
					UERROR("Vertex %d not found!?", iter->first);
				}
			}
		}
		else
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				const g2o::VertexSE3* v = (const g2o::VertexSE3*)optimizer.vertex(iter->first);
				if(v)
				{
					Transform t = Transform::fromEigen3d(v->estimate());
					optimizedPoses.insert(std::pair<int, Transform>(iter->first, t));
					UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
				}
				else
				{
					UERROR("Vertex %d not found!?", iter->first);
				}
			}
		}
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
#ifdef RTABMAP_ORB_SLAM2
	UERROR("G2O graph optimization cannot be used with g2o built from ORB_SLAM2, only SBA is available.");
#else
	UERROR("Not built with G2O support!");
#endif
#endif
	return optimizedPoses;
}

std::map<int, Transform> OptimizerG2O::optimizeBA(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, CameraModel> & models,
		std::map<int, cv::Point3f> & points3DMap,
		const std::map<int, std::map<int, cv::Point3f> > & wordReferences,
		std::set<int> * outliers)
{
	std::map<int, Transform> optimizedPoses;
#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM2)
	UDEBUG("Optimizing graph...");

	optimizedPoses.clear();
	if(poses.size()>=2 && iterations() > 0 && models.size() == poses.size())
	{
		g2o::SparseOptimizer optimizer;
		optimizer.setVerbose(ULogger::level()==ULogger::kDebug);
		g2o::BlockSolver_6_3::LinearSolverType * linearSolver = 0;

#ifdef RTABMAP_ORB_SLAM2
		linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
#else
		if(solver_ == 3)
		{
			//eigen
			linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
		}
		else if(solver_ == 2)
		{
#ifdef G2O_HAVE_CHOLMOD
			//chmold
			linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
#endif
		}
		else if(solver_ == 0)
		{
#ifdef G2O_HAVE_CSPARSE
			//csparse
			linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();
#endif
		}

		if(linearSolver == 0)
		{
			//pcg
			linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
		}
#endif

		g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

#ifndef RTABMAP_ORB_SLAM2
		if(optimizer_ == 1)
		{
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(solver_ptr));
		}
		else
#endif
		{
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(solver_ptr));
		}

		UDEBUG("fill poses to g2o...");
		for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); )
		{
			// Get camera model
			std::map<int, CameraModel>::const_iterator iterModel = models.find(iter->first);
			UASSERT(iterModel != models.end() && iterModel->second.isValidForProjection());

			Transform camPose = iter->second * iterModel->second.localTransform();

			// Add node's pose
			UASSERT(!camPose.isNull());
#ifdef RTABMAP_ORB_SLAM2
			g2o::VertexSE3Expmap * vCam = new g2o::VertexSE3Expmap();
#else
			g2o::VertexCam * vCam = new g2o::VertexCam();
#endif

			Eigen::Affine3d a = camPose.toEigen3d();
#ifdef RTABMAP_ORB_SLAM2
			a = a.inverse();
			vCam->setEstimate(g2o::SE3Quat(a.linear(), a.translation()));
#else
			g2o::SBACam cam(Eigen::Quaterniond(a.linear()), a.translation());
			cam.setKcam(
					iterModel->second.fx(),
					iterModel->second.fy(),
					iterModel->second.cx(),
					iterModel->second.cy(),
					iterModel->second.Tx()<0.0?-iterModel->second.Tx()/iterModel->second.fx():baseline_); // baseline in meters
			vCam->setEstimate(cam);
#endif
			vCam->setId(iter->first);

			// negative root means that all other poses should be fixed instead of the root
			vCam->setFixed((rootId >= 0 && iter->first == rootId) || (rootId < 0 && iter->first != -rootId));

			UDEBUG("cam %d (fixed=%d) fx=%f fy=%f cx=%f cy=%f Tx=%f baseline=%f t=%s",
					iter->first,
					vCam->fixed()?1:0,
					iterModel->second.fx(),
					iterModel->second.fy(),
					iterModel->second.cx(),
					iterModel->second.cy(),
					iterModel->second.Tx(),
					iterModel->second.Tx()<0.0?-iterModel->second.Tx()/iterModel->second.fx():baseline_,
					camPose.prettyPrint().c_str());

			UASSERT_MSG(optimizer.addVertex(vCam), uFormat("cannot insert vertex %d!?", iter->first).c_str());

			++iter;
		}

#ifndef RTABMAP_ORB_SLAM2
		UDEBUG("fill edges to g2o...");
		for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
		{
			if(uContains(poses, iter->second.from()) &&
			   uContains(poses, iter->second.to()))
			{
				// add edge
				int id1 = iter->second.from();
				int id2 = iter->second.to();

				if(id1 != id2) // not supporting prior
				{
					UASSERT(!iter->second.transform().isNull());

					Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
					memcpy(information.data(), iter->second.infMatrix().data, iter->second.infMatrix().total()*sizeof(double));

					// between cameras, not base_link
					Transform camLink = models.at(id1).localTransform().inverse()*iter->second.transform()*models.at(id2).localTransform();
					UDEBUG("added edge %d->%d (in cam frame=%s)",
							id1,
							id2,
							camLink.prettyPrint().c_str());
					Eigen::Affine3d a = camLink.toEigen3d();

					g2o::EdgeSBACam * e = new g2o::EdgeSBACam();
					g2o::VertexCam* v1 = (g2o::VertexCam*)optimizer.vertex(id1);
					g2o::VertexCam* v2 = (g2o::VertexCam*)optimizer.vertex(id2);
					UASSERT(v1 != 0);
					UASSERT(v2 != 0);
					e->setVertex(0, v1);
					e->setVertex(1, v2);
					e->setMeasurement(g2o::SE3Quat(a.linear(), a.translation()));
					e->setInformation(information);

					if (!optimizer.addEdge(e))
					{
						delete e;
						UERROR("Map: Failed adding constraint between %d and %d, skipping", id1, id2);
						return optimizedPoses;
					}
				}
			}
		}
#endif

		UDEBUG("fill 3D points to g2o...");
		const int stepVertexId = poses.rbegin()->first+1;
		std::list<g2o::OptimizableGraph::Edge*> edges;
		for(std::map<int, std::map<int, cv::Point3f> >::const_iterator iter = wordReferences.begin(); iter!=wordReferences.end(); ++iter)
		{
			if(points3DMap.find(iter->first) != points3DMap.end())
			{
				const cv::Point3f & pt3d = points3DMap.at(iter->first);
				g2o::VertexSBAPointXYZ* vpt3d = new g2o::VertexSBAPointXYZ();

				vpt3d->setEstimate(Eigen::Vector3d(pt3d.x, pt3d.y, pt3d.z));
				vpt3d->setId(stepVertexId + iter->first);
				vpt3d->setMarginalized(true);
				optimizer.addVertex(vpt3d);

				UDEBUG("Added 3D point %d (%f,%f,%f)", vpt3d->id()-stepVertexId, pt3d.x, pt3d.y, pt3d.z);

				// set observations
				for(std::map<int, cv::Point3f>::const_iterator jter=iter->second.begin(); jter!=iter->second.end(); ++jter)
				{
					int camId = jter->first;
					if(poses.find(camId) != poses.end() && optimizer.vertex(camId) != 0)
					{
						const cv::Point3f & pt = jter->second;
						double depth = pt.z;

						UDEBUG("Added observation pt=%d to cam=%d (%f,%f) d=%f", vpt3d->id()-stepVertexId, camId, pt.x, pt.y, depth);

						g2o::OptimizableGraph::Edge * e;
						double baseline = 0.0;
#ifdef RTABMAP_ORB_SLAM2
						g2o::VertexSE3Expmap* vcam = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(camId));
						std::map<int, CameraModel>::const_iterator iterModel = models.find(camId);

						cv::Point3f t = util3d::transformPoint(pt3d, Transform::fromEigen3d(vcam->estimate()).inverse());
						UDEBUG("in cam %d frame=(%f,%f,%f)", camId, t.x, t.y, t.z);

						cv::Point3f t2 = util3d::transformPoint(pt3d, (poses.at(camId)*iterModel->second.localTransform()).inverse());
						UDEBUG("in cam2 %d frame=(%f,%f,%f)",camId, t2.x, t2.y, t2.z);

						g2o::Vector3d t3 = vcam->estimate().map(g2o::Vector3d(pt3d.x, pt3d.y, pt3d.z));
						UDEBUG("in cam3 %d frame=(%f,%f,%f)",camId, t3[0], t3[1], t3[2]);

						cv::Point3f t4 = util3d::transformPoint(pt3d, (poses.at(camId)*iterModel->second.localTransform()));
						UDEBUG("in cam4 %d frame=(%f,%f,%f)",camId, t4.x, t4.y, t4.z);

						UASSERT(iterModel != models.end() && iterModel->second.isValidForProjection());
						baseline = iterModel->second.Tx()<0.0?-iterModel->second.Tx()/iterModel->second.fx():baseline_;
#else
						g2o::VertexCam* vcam = dynamic_cast<g2o::VertexCam*>(optimizer.vertex(camId));
						baseline = vcam->estimate().baseline;
#endif
						double variance = pixelVariance_;
						if(uIsFinite(depth) && depth > 0.0 && baseline > 0.0)
						{
							// stereo edge
#ifdef RTABMAP_ORB_SLAM2
							g2o::EdgeStereoSE3ProjectXYZ* es = new g2o::EdgeStereoSE3ProjectXYZ();
							float disparity = baseline * iterModel->second.fx() / depth;
							Eigen::Vector3d obs( pt.x, pt.y, pt.x-disparity);
							es->setMeasurement(obs);
							//variance *= log(exp(1)+disparity);
							es->setInformation(Eigen::Matrix3d::Identity() / variance);
							es->fx = iterModel->second.fx();
							es->fy = iterModel->second.fy();
							es->cx = iterModel->second.cx();
							es->cy = iterModel->second.cy();
							es->bf = baseline*es->fx;
							e = es;
#else
							g2o::EdgeProjectP2SC* es = new g2o::EdgeProjectP2SC();
							float disparity = baseline * vcam->estimate().Kcam(0,0) / depth;
							Eigen::Vector3d obs( pt.x, pt.y, pt.x-disparity);
							es->setMeasurement(obs);
							//variance *= log(exp(1)+disparity);
							es->setInformation(Eigen::Matrix3d::Identity() / variance);
							e = es;
#endif
						}
						else
						{
							if(baseline > 0.0)
							{
								UWARN("Stereo camera model detected but current "
										"observation (pt=%d to cam=%d) has null depth (%f m), adding "
										"mono observation instead.",
										vpt3d->id()-stepVertexId, camId, depth);
							}
							// mono edge
#ifdef RTABMAP_ORB_SLAM2
							g2o::EdgeSE3ProjectXYZ* em = new g2o::EdgeSE3ProjectXYZ();
							Eigen::Vector2d obs( pt.x, pt.y);
							em->setMeasurement(obs);
							em->setInformation(Eigen::Matrix2d::Identity() / variance);
							em->fx = iterModel->second.fx();
							em->fy = iterModel->second.fy();
							em->cx = iterModel->second.cx();
							em->cy = iterModel->second.cy();
							e = em;

#else
							g2o::EdgeProjectP2MC* em = new g2o::EdgeProjectP2MC();
							Eigen::Vector2d obs( pt.x, pt.y);
							em->setMeasurement(obs);
							em->setInformation(Eigen::Matrix2d::Identity() / variance);
							e = em;
#endif
						}
						e->setVertex(0, vpt3d);
						e->setVertex(1, vcam);
						UDEBUG("");
						if(robustKernelDelta_ > 0.0)
						{
							g2o::RobustKernelHuber* kernel = new g2o::RobustKernelHuber;
							kernel->setDelta(robustKernelDelta_);
							e->setRobustKernel(kernel);
						}

						optimizer.addEdge(e);
						edges.push_back(e);
					}
				}
			}
		}

		UDEBUG("Initial optimization...");
		optimizer.initializeOptimization();

		UASSERT(optimizer.verifyInformationMatrices());

		UINFO("g2o optimizing begin (max iterations=%d, robustKernel=%f)", iterations(), robustKernelDelta_);

		int it = 0;
		UTimer timer;
		int outliersCount = 0;
		int outliersCountFar = 0;

		for(int i=0; i<(robustKernelDelta_>0.0?2:1); ++i)
		{
			it += optimizer.optimize(i==0&&robustKernelDelta_>0.0?3:iterations());

			// early stop condition
			optimizer.computeActiveErrors();
			double chi2 = optimizer.activeRobustChi2();
			if(uIsNan(chi2))
			{
				UERROR("Optimization generated NANs, aborting optimization! Try another g2o's optimizer (current=%d).", optimizer_);
				return optimizedPoses;
			}
			UDEBUG("iteration %d: %d nodes, %d edges, chi2: %f", i, (int)optimizer.vertices().size(), (int)optimizer.edges().size(), chi2);

			if(i>0 && (optimizer.activeRobustChi2() > 1000000000000.0 || !uIsFinite(optimizer.activeRobustChi2())))
			{
				UWARN("g2o: Large optimization error detected (%f), aborting optimization!");
				return optimizedPoses;
			}

			if(robustKernelDelta_>0.0)
			{
				for(std::list<g2o::OptimizableGraph::Edge*>::iterator iter=edges.begin(); iter!=edges.end();++iter)
				{
					if((*iter)->level() == 0 && (*iter)->chi2() > (*iter)->robustKernel()->delta())
					{
						(*iter)->setLevel(1);
						++outliersCount;
						double d = 0.0;
#ifdef RTABMAP_ORB_SLAM2
						if(dynamic_cast<g2o::EdgeStereoSE3ProjectXYZ*>(*iter) != 0)
						{
							d = ((g2o::EdgeStereoSE3ProjectXYZ*)(*iter))->measurement()[0]-((g2o::EdgeStereoSE3ProjectXYZ*)(*iter))->measurement()[2];
						}
						UDEBUG("Ignoring edge (%d<->%d) d=%f var=%f kernel=%f chi2=%f", (*iter)->vertex(0)->id()-stepVertexId, (*iter)->vertex(1)->id(), d, 1.0/((g2o::EdgeStereoSE3ProjectXYZ*)(*iter))->information()(0,0), (*iter)->robustKernel()->delta(), (*iter)->chi2());
#else
						if(dynamic_cast<g2o::EdgeProjectP2SC*>(*iter) != 0)
						{
							d = ((g2o::EdgeProjectP2SC*)(*iter))->measurement()[0]-((g2o::EdgeProjectP2SC*)(*iter))->measurement()[2];
						}
						UDEBUG("Ignoring edge (%d<->%d) d=%f var=%f kernel=%f chi2=%f", (*iter)->vertex(0)->id()-stepVertexId, (*iter)->vertex(1)->id(), d, 1.0/((g2o::EdgeProjectP2SC*)(*iter))->information()(0,0), (*iter)->robustKernel()->delta(), (*iter)->chi2());
#endif

						const cv::Point3f & pt3d = points3DMap.at((*iter)->vertex(0)->id()-stepVertexId);
						((g2o::VertexSBAPointXYZ*)(*iter)->vertex(0))->setEstimate(Eigen::Vector3d(pt3d.x, pt3d.y, pt3d.z));

						if(outliers)
						{
							outliers->insert((*iter)->vertex(0)->id()-stepVertexId);
						}
						if(d < 5.0)
						{
							outliersCountFar++;
						}
					}
					//(*iter)->setRobustKernel(0);
				}
				if(i==0)
					optimizer.initializeOptimization(0);
				UDEBUG("outliers=%d outliersCountFar=%d", outliersCount, outliersCountFar);
			}
		}

		UINFO("g2o optimizing end (%d iterations done, error=%f, outliers=%d/%d (delta=%f) time = %f s)", it, optimizer.activeRobustChi2(), outliersCount, (int)edges.size(), robustKernelDelta_, timer.ticks());

		if(optimizer.activeRobustChi2() > 1000000000000.0)
		{
			UWARN("g2o: Large optimimzation error detected (%f), aborting optimization!");
			return optimizedPoses;
		}

		// update poses
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
#ifdef RTABMAP_ORB_SLAM2
			const g2o::VertexSE3Expmap* v = (const g2o::VertexSE3Expmap*)optimizer.vertex(iter->first);
#else
			const g2o::VertexCam* v = (const g2o::VertexCam*)optimizer.vertex(iter->first);
#endif
			if(v)
			{
				Transform t = Transform::fromEigen3d(v->estimate());

#ifdef RTABMAP_ORB_SLAM2
				t=t.inverse();
#endif

				// remove model local transform
				t *= models.at(iter->first).localTransform().inverse();
				UDEBUG("%d from=%s to=%s", iter->first, iter->second.prettyPrint().c_str(), t.prettyPrint().c_str());
				if(t.isNull())
				{
					UERROR("Optimized pose %d is null!?!?", iter->first);
					optimizedPoses.clear();
					return optimizedPoses;
				}

				// FIXME: is there a way that we can add the 2D constraint directly in SBA?
				if(this->isSlam2d())
				{
					// get transform between old and new pose
					t = iter->second.inverse() * t;
					optimizedPoses.insert(std::pair<int, Transform>(iter->first, iter->second * t.to3DoF()));
				}
				else
				{
					optimizedPoses.insert(std::pair<int, Transform>(iter->first, t));
				}
			}
			else
			{
				UERROR("Vertex (pose) %d not found!?", iter->first);
			}
		}

		//update points3D

		for(std::map<int, cv::Point3f>::iterator iter = points3DMap.begin(); iter!=points3DMap.end(); ++iter)
		{
			const g2o::VertexSBAPointXYZ* v = (const g2o::VertexSBAPointXYZ*)optimizer.vertex(stepVertexId + iter->first);
			if(v)
			{
				cv::Point3f p(v->estimate()[0], v->estimate()[1], v->estimate()[2]);
				//UDEBUG("%d from=%f,%f,%f to=%f,%f,%f", iter->first, iter->second.x, iter->second.y, iter->second.z, p.x, p.y, p.z);
				iter->second = p;
			}
			else
			{
				iter->second.x = iter->second.y = iter->second.z = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}
	else if(poses.size() > 1 && poses.size() != models.size())
	{
		UERROR("This method should be called with size of poses = size camera models!");
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
	UERROR("Not built with G2O support!");
#endif
	return optimizedPoses;
}

bool OptimizerG2O::saveGraph(
		const std::string & fileName,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		bool useRobustConstraints)
{
	FILE * file = 0;

#ifdef _MSC_VER
	fopen_s(&file, fileName.c_str(), "w");
#else
	file = fopen(fileName.c_str(), "w");
#endif

	if(file)
	{
		// VERTEX_SE3 id x y z qw qx qy qz
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			Eigen::Quaternionf q = iter->second.getQuaternionf();
			fprintf(file, "VERTEX_SE3:QUAT %d %f %f %f %f %f %f %f\n",
					iter->first,
					iter->second.x(),
					iter->second.y(),
					iter->second.z(),
					q.x(),
					q.y(),
					q.z(),
					q.w());
		}

		//EDGE_SE3 observed_vertex_id observing_vertex_id x y z qx qy qz qw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
		int virtualVertexId = poses.size()?poses.rbegin()->first+1:0;
		for(std::multimap<int, Link>::const_iterator iter = edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
		{
			std::string prefix = "EDGE_SE3:QUAT";
			std::string suffix = "";

			if(useRobustConstraints &&
			   iter->second.type() != Link::kNeighbor &&
			   iter->second.type() != Link::kNeighborMerged)
			{
				prefix = "EDGE_SE3_SWITCHABLE";
				fprintf(file, "VERTEX_SWITCH %d 1\n", virtualVertexId);
				fprintf(file, "EDGE_SWITCH_PRIOR %d 1 1.0\n", virtualVertexId);
				suffix = uFormat(" %d", virtualVertexId++);
			}

			Eigen::Quaternionf q = iter->second.transform().getQuaternionf();
			fprintf(file, "%s %d %d%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
					prefix.c_str(),
					iter->second.from(),
					iter->second.to(),
					suffix.c_str(),
					iter->second.transform().x(),
					iter->second.transform().y(),
					iter->second.transform().z(),
					q.x(),
					q.y(),
					q.z(),
					q.w(),
					iter->second.infMatrix().at<double>(0,0),
					iter->second.infMatrix().at<double>(0,1),
					iter->second.infMatrix().at<double>(0,2),
					iter->second.infMatrix().at<double>(0,3),
					iter->second.infMatrix().at<double>(0,4),
					iter->second.infMatrix().at<double>(0,5),
					iter->second.infMatrix().at<double>(1,1),
					iter->second.infMatrix().at<double>(1,2),
					iter->second.infMatrix().at<double>(1,3),
					iter->second.infMatrix().at<double>(1,4),
					iter->second.infMatrix().at<double>(1,5),
					iter->second.infMatrix().at<double>(2,2),
					iter->second.infMatrix().at<double>(2,3),
					iter->second.infMatrix().at<double>(2,4),
					iter->second.infMatrix().at<double>(2,5),
					iter->second.infMatrix().at<double>(3,3),
					iter->second.infMatrix().at<double>(3,4),
					iter->second.infMatrix().at<double>(3,5),
					iter->second.infMatrix().at<double>(4,4),
					iter->second.infMatrix().at<double>(4,5),
					iter->second.infMatrix().at<double>(5,5));
		}
		UINFO("Graph saved to %s", fileName.c_str());
		fclose(file);
	}
	else
	{
		UERROR("Cannot save to file %s", fileName.c_str());
		return false;
	}
	return true;
}

} /* namespace rtabmap */
