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

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <set>

#include <rtabmap/core/OptimizerG2O.h>

#ifdef WITH_G2O
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;

#include "vertigo/g2o/edge_switchPrior.h"
#include "vertigo/g2o/edge_se2Switchable.h"
#include "vertigo/g2o/edge_se3Switchable.h"
#include "vertigo/g2o/vertex_switchLinear.h"

#endif // end WITH_G2O

namespace rtabmap {

bool OptimizerG2O::available()
{
#ifdef WITH_G2O
	return true;
#else
	return false;
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
#ifdef WITH_G2O
	UDEBUG("Optimizing graph...");
	optimizedPoses.clear();
	if(edgeConstraints.size()>=1 && poses.size()>=2 && iterations() > 0)
	{
		// Apply g2o optimization

		g2o::SparseOptimizer optimizer;
		optimizer.setVerbose(ULogger::level()==ULogger::kDebug);
		int solverApproach = 0;
		int optimizationApproach = 1;

		SlamBlockSolver * blockSolver;
		if(solverApproach == 1)
		{
			//pcg
			SlamLinearPCGSolver * linearSolver = new SlamLinearPCGSolver();
			blockSolver = new SlamBlockSolver(linearSolver);
		}
		else if(solverApproach == 2)
		{
			//csparse
			SlamLinearCSparseSolver* linearSolver = new SlamLinearCSparseSolver();
			linearSolver->setBlockOrdering(false);
			blockSolver = new SlamBlockSolver(linearSolver);
		}
		else
		{
			//chmold
			SlamLinearCholmodSolver * linearSolver = new SlamLinearCholmodSolver();
			linearSolver->setBlockOrdering(false);
			blockSolver = new SlamBlockSolver(linearSolver);
		}

		if(optimizationApproach == 1)
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
