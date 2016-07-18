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

#ifdef RTABMAP_G2O
#include "g2o/config.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/linear_solver.h"
#include "g2o/types/sba/types_sba.h"
#include "g2o/core/robust_kernel_impl.h"
#ifdef G2O_HAVE_CSPARSE
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#endif
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#ifdef G2O_HAVE_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#endif
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
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

#endif // end RTABMAP_G2O

namespace rtabmap {

bool OptimizerG2O::available()
{
#ifdef RTABMAP_G2O
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
	UASSERT(pixelVariance_ > 0.0);

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

		SlamBlockSolver * blockSolver = 0;

		if(solver_ == 2)
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
				pose = a.rotation();
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
			int id1 = iter->first;
			int id2 = iter->second.to();

			UASSERT(!iter->second.transform().isNull());

			g2o::HyperGraph::Edge * edge = 0;

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
				constraint = a.rotation();
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

			if (!optimizer.addEdge(edge))
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
		optimizer.clear();
		g2o::Factory::destroy();
		g2o::OptimizationAlgorithmFactory::destroy();
		g2o::HyperGraphActionLibrary::destroy();
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

std::map<int, Transform> OptimizerG2O::optimizeBA(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, Signature> & signatures)
{
	std::map<int, Transform> optimizedPoses;
#ifdef RTABMAP_G2O
	UDEBUG("Optimizing graph...");

	optimizedPoses.clear();
	if(links.size()>=1 && poses.size()>=2 && iterations() > 0)
	{
		g2o::SparseOptimizer optimizer;
		optimizer.setVerbose(ULogger::level()==ULogger::kDebug);
		g2o::BlockSolver_6_3::LinearSolverType * linearSolver = 0;
		bool robustKernel = true;

		if(solver_ == 2)
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

		g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

		if(optimizer_ == 1)
		{
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(solver_ptr));
		}
		else
		{
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(solver_ptr));
		}

		std::map<int, Transform> frames = poses;

		UDEBUG("fill poses to g2o...");
		std::map<int, CameraModel> models;
		for(std::map<int, Transform>::iterator iter=frames.begin(); iter!=frames.end(); )
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
				}
				else
				{
					UERROR("Missing calibration for node %d", iter->first);
					return optimizedPoses;
				}
			}
			else
			{
				UERROR("Did not find node %d in cache", iter->first);
			}

			if(model.isValidForProjection())
			{
				models.insert(std::make_pair(iter->first, model));
				Transform camPose = iter->second * model.localTransform();
				//iter->second = (iter->second * model.localTransform()).inverse();
				UDEBUG("%d t=%s", iter->first, camPose.prettyPrint().c_str());

				// Add node's pose
				UASSERT(!camPose.isNull());
				g2o::VertexCam * vCam = new g2o::VertexCam();

				Eigen::Affine3d a = camPose.toEigen3d();
				g2o::SBACam cam(Eigen::Quaterniond(a.rotation()), a.translation());
				cam.setKcam(model.fx(), model.fy(), model.cx(), model.cy(), 0);
				vCam->setEstimate(cam);
				if(iter->first == rootId)
				{
					vCam->setFixed(true);
				}
				vCam->setId(iter->first);
				std::cout << cam << std::endl;
				UASSERT_MSG(optimizer.addVertex(vCam), uFormat("cannot insert vertex %d!?", iter->first).c_str());

				++iter;
			}
			else
			{
				frames.erase(iter++);
			}
		}

		UDEBUG("fill edges to g2o and associate each 3D point to all frames observing it...");
		for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
		{
			Link link = iter->second;
			if(link.to() < link.from())
			{
				link = link.inverse();
			}
			if(uContains(signatures, link.from()) &&
			   uContains(signatures, link.to()) &&
			   uContains(frames, link.from()) &&
			   uContains(frames, link.to()))
			{
				// add edge
				int id1 = iter->first;
				int id2 = iter->second.to();

				UASSERT(!iter->second.transform().isNull());

				Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
				if(!isCovarianceIgnored())
				{
					memcpy(information.data(), iter->second.infMatrix().data, iter->second.infMatrix().total()*sizeof(double));
				}

				// between cameras, not base_link
				Transform camLink = models.at(id1).localTransform().inverse()*iter->second.transform()*models.at(id2).localTransform();
				//Transform t = iter->second.transform();
				UDEBUG("added edge %d=%s -> %d=%s",
						id1,
						iter->second.transform().prettyPrint().c_str(),
						id2,
						camLink.prettyPrint().c_str());
				Eigen::Affine3d a = camLink.toEigen3d();

				g2o::EdgeSBACam * e = new g2o::EdgeSBACam();
				g2o::VertexSE3* v1 = (g2o::VertexSE3*)optimizer.vertex(id1);
				g2o::VertexSE3* v2 = (g2o::VertexSE3*)optimizer.vertex(id2);
				UASSERT(v1 != 0);
				UASSERT(v2 != 0);
				e->setVertex(0, v1);
				e->setVertex(1, v2);
				e->setMeasurement(g2o::SE3Quat(a.rotation(), a.translation()));
				e->setInformation(information);

				if (!optimizer.addEdge(e))
				{
					delete e;
					UERROR("Map: Failed adding constraint between %d and %d, skipping", id1, id2);
					return optimizedPoses;
				}
			}
		}

		std::map<int, cv::Point3f> points3DMap;
		std::map<int, std::map<int, cv::Point2f> > wordReferences; // <ID words, IDs frames + keypoint>
		this->computeBACorrespondences(frames, links, signatures, points3DMap, wordReferences);

		UDEBUG("fill 3D points to g2o...");
		int stepVertexId = frames.rbegin()->first+1;
		for(std::map<int, std::map<int, cv::Point2f> >::iterator iter = wordReferences.begin(); iter!=wordReferences.end(); ++iter)
		{
			const cv::Point3f & pt3d = points3DMap.at(iter->first);
			g2o::VertexSBAPointXYZ* vpt3d = new g2o::VertexSBAPointXYZ();

			vpt3d->setEstimate(Eigen::Vector3d(pt3d.x, pt3d.y, pt3d.z));
			vpt3d->setId(stepVertexId + iter->first);
			vpt3d->setMarginalized(true);
			optimizer.addVertex(vpt3d);

			// set observations
			for(std::map<int, cv::Point2f>::const_iterator jter=iter->second.begin(); jter!=iter->second.end(); ++jter)
			{
				int camId = jter->first;

				const cv::Point2f & pt = jter->second;

				Eigen::Matrix<double,2,1> obs;
				obs << pt.x, pt.y;

				UDEBUG("Added observation pt=%d to cam=%d (%f,%f)", vpt3d->id(), camId, pt.x, pt.y);

				g2o::EdgeProjectP2MC* e = new g2o::EdgeProjectP2MC();

				e->setVertex(0, vpt3d);
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(camId)));
				e->setMeasurement(obs);
				e->setInformation(Eigen::Matrix2d::Identity() / pixelVariance_);

				if(robustKernel)
				{
					e->setRobustKernel(new g2o::RobustKernelHuber);
				}

				optimizer.addEdge(e);
			}
		}

		UDEBUG("Initial optimization...");
		optimizer.initializeOptimization();

		UASSERT(optimizer.verifyInformationMatrices());

		UINFO("g2o optimizing begin (max iterations=%d, epsilon=%f robustKernel=%d)", iterations(), this->epsilon(), robustKernel?1:0);

		int it = 0;
		UTimer timer;
		double lastError = 0.0;
		if(this->epsilon() > 0.0)
		{
			for(int i=0; i<iterations(); ++i)
			{
				it += optimizer.optimize(1);

				// early stop condition
				optimizer.computeActiveErrors();
				double chi2 = optimizer.activeRobustChi2();
				UDEBUG("iteration %d: %d nodes, %d edges, chi2: %f", i, (int)optimizer.vertices().size(), (int)optimizer.edges().size(), chi2);

				if(i>0 && (optimizer.activeRobustChi2() > 1000000000000.0 || !uIsFinite(optimizer.activeRobustChi2())))
				{
					UWARN("g2o: Large optimization error detected (%f), aborting optimization!");
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
		UINFO("g2o optimizing end (%d iterations done, error=%f, time = %f s)", it, optimizer.activeRobustChi2(), timer.ticks());

		if(optimizer.activeRobustChi2() > 1000000000000.0)
		{
			UWARN("g2o: Large optimimzation error detected (%f), aborting optimization!");
			return optimizedPoses;
		}

		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			const g2o::VertexCam* v = (const g2o::VertexCam*)optimizer.vertex(iter->first);
			if(v)
			{
				Transform t = Transform::fromEigen3d(v->estimate());
				UDEBUG("%d t=%s", iter->first, t.prettyPrint().c_str());
				// remove model local transform
				t *= models.at(iter->first).localTransform().inverse();
				optimizedPoses.insert(std::pair<int, Transform>(iter->first, t));
				UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
			}
			else
			{
				UERROR("Vertex %d not found!?", iter->first);
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
					iter->first,
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
