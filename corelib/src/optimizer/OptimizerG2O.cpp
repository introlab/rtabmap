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
#include <locale.h>
#include <set>

#include <rtabmap/core/Version.h>
#include <rtabmap/core/optimizer/OptimizerG2O.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_motion_estimation.h>
#include <rtabmap/core/util3d.h>


#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM)
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
namespace g2o {
// bug #include "g2o/core/eigen_types.h" not found on Indigo
typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor> MatrixXD;
}

#ifdef RTABMAP_G2O
#include "g2o/types/sba/types_sba.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/config.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/edge_se3_xyzprior.h" // Include after types_slam3d.h to be ignored on newest g2o versions
#include "g2o/edge_se3_gravity.h"
#include "g2o/edge_sbacam_gravity.h"
#include "g2o/edge_xy_prior.h"  // Include after types_slam2d.h to be ignored on newest g2o versions
#include "g2o/edge_xyz_prior.h" // Include after types_slam3d.h to be ignored on newest g2o versions
#ifdef G2O_HAVE_CSPARSE
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#endif
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#ifdef G2O_HAVE_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#endif
#endif // RTABMAP_G2O

#ifdef RTABMAP_ORB_SLAM
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

#ifdef G2O_SRC_DIR
namespace g2o {
typedef VertexPointXYZ VertexSBAPointXYZ;
}
#endif

#if defined(RTABMAP_VERTIGO)
#include "vertigo/g2o/edge_switchPrior.h"
#include "vertigo/g2o/edge_se2Switchable.h"
#include "vertigo/g2o/edge_se3Switchable.h"
#include "vertigo/g2o/vertex_switchLinear.h"
#endif
#endif

#endif // end defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM)

enum {
    PARAM_OFFSET=0,
};

#define MULTICAM_OFFSET 10 // 10 means max 10 cameras per pose

namespace rtabmap {

bool OptimizerG2O::available()
{
#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM)
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

OptimizerG2O::OptimizerG2O(const ParametersMap & parameters) :
		Optimizer(parameters),
		solver_(Parameters::defaultg2oSolver()),
		optimizer_(Parameters::defaultg2oOptimizer()),
		pixelVariance_(Parameters::defaultg2oPixelVariance()),
		robustKernelDelta_(Parameters::defaultg2oRobustKernelDelta()),
		baseline_(Parameters::defaultg2oBaseline())
{
#ifdef RTABMAP_G2O
	// Issue on android, have to explicitly register this type when using fixed root prior below
	if(!g2o::Factory::instance()->knowsTag("CACHE_SE3_OFFSET"))
	{
#if defined(RTABMAP_G2O_CPP11) && RTABMAP_G2O_CPP11 == 1
		g2o::Factory::instance()->registerType("CACHE_SE3_OFFSET", std::make_unique<g2o::HyperGraphElementCreator<g2o::CacheSE3Offset> >());
#else
		g2o::Factory::instance()->registerType("CACHE_SE3_OFFSET", new g2o::HyperGraphElementCreator<g2o::CacheSE3Offset>);
#endif
	}
#endif
	parseParameters(parameters);
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

#ifdef RTABMAP_ORB_SLAM
	if(solver_ != 3)
	{
		UWARN("g2o built with ORB_SLAM has only Eigen solver available, using Eigen=3 instead of %d.", solver_);
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
		cv::Mat & outputCovariance,
		std::list<std::map<int, Transform> > * intermediateGraphes,
		double * finalError,
		int * iterationsDone)
{
	outputCovariance = cv::Mat::eye(6,6,CV_64FC1);
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
	if(edgeConstraints.size()>=1 && poses.size()>=2 && iterations() > 0 && poses.rbegin()->first > 0)
	{
		// Apply g2o optimization

		g2o::SparseOptimizer optimizer;
		//optimizer.setVerbose(ULogger::level()==ULogger::kDebug);
		if (isSlam2d())
		{
			g2o::ParameterSE2Offset* odomOffset = new g2o::ParameterSE2Offset();
			odomOffset->setId(PARAM_OFFSET);
			optimizer.addParameter(odomOffset);
		}
		else
		{
			g2o::ParameterSE3Offset* odomOffset = new g2o::ParameterSE3Offset();
			odomOffset->setId(PARAM_OFFSET);
			optimizer.addParameter(odomOffset);
		}

#ifdef RTABMAP_G2O_CPP11

		std::unique_ptr<SlamBlockSolver> blockSolver;

		if(solver_ == 3)
		{
			//eigen
			auto linearSolver = std::make_unique<SlamLinearEigenSolver>();
			linearSolver->setBlockOrdering(false);
			blockSolver = std::make_unique<SlamBlockSolver>(std::move(linearSolver));
		}
#ifdef G2O_HAVE_CHOLMOD
		else if(solver_ == 2)
		{
			//chmold
			auto linearSolver = std::make_unique<SlamLinearCholmodSolver>();
			linearSolver->setBlockOrdering(false);
			blockSolver = std::make_unique<SlamBlockSolver>(std::move(linearSolver));
		}
#endif
#ifdef G2O_HAVE_CSPARSE
		else if(solver_ == 0)
		{

			//csparse
			auto linearSolver = std::make_unique<SlamLinearCSparseSolver>();
			linearSolver->setBlockOrdering(false);
			blockSolver = std::make_unique<SlamBlockSolver>(std::move(linearSolver));
		}
#endif
		else
		{
			//pcg
			auto linearSolver = std::make_unique<SlamLinearPCGSolver>();
			blockSolver = std::make_unique<SlamBlockSolver>(std::move(linearSolver));
		}

		if(optimizer_ == 1)
		{

			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver)));
		}
		else
		{
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver)));
		}

#else

		SlamBlockSolver * blockSolver = 0;

		if(solver_ == 3)
		{
			//eigen
			SlamLinearEigenSolver * linearSolver = new SlamLinearEigenSolver();
			linearSolver->setBlockOrdering(false);
			blockSolver = new SlamBlockSolver(linearSolver);
		}
#ifdef G2O_HAVE_CHOLMOD
		else if(solver_ == 2)
		{
			//chmold
			SlamLinearCholmodSolver * linearSolver = new SlamLinearCholmodSolver();
			linearSolver->setBlockOrdering(false);
			blockSolver = new SlamBlockSolver(linearSolver);
		}
#endif
#ifdef G2O_HAVE_CSPARSE
		else if(solver_ == 0)
		{
			//csparse
			SlamLinearCSparseSolver* linearSolver = new SlamLinearCSparseSolver();
			linearSolver->setBlockOrdering(false);
			blockSolver = new SlamBlockSolver(linearSolver);
		}
#endif
		else
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
#endif
		// detect if there is a global pose prior set, if so remove rootId
		bool hasGravityConstraints = false;
		if(!priorsIgnored() || (!isSlam2d() && gravitySigma() > 0))
		{
			for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
			{
				if(iter->second.from() == iter->second.to())
				{
					if(!priorsIgnored() && iter->second.type() == Link::kPosePrior)
					{
						rootId = 0;
						break;
					}
					else if(!isSlam2d() &&
							gravitySigma() > 0 &&
							iter->second.type() == Link::kGravity)
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

		int landmarkVertexOffset = poses.rbegin()->first+1;
		std::map<int, bool> isLandmarkWithRotation;

		UDEBUG("fill poses to g2o... (rootId=%d hasGravityConstraints=%d isSlam2d=%d)", rootId, hasGravityConstraints?1:0, isSlam2d()?1:0);
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			UASSERT(!iter->second.isNull());
			g2o::HyperGraph::Vertex * vertex = 0;
			int id = iter->first;
			if(isSlam2d())
			{
				if(id > 0)
				{
					g2o::VertexSE2 * v2 = new g2o::VertexSE2();
					v2->setEstimate(g2o::SE2(iter->second.x(), iter->second.y(), iter->second.theta()));
					if(id == rootId)
					{
						UDEBUG("Set %d fixed", id);
						v2->setFixed(true);
					}
					vertex = v2;
				}
				else if(!landmarksIgnored())
				{
					// check if it is SE2 or only PointXY
					std::multimap<int, Link>::const_iterator jter=edgeConstraints.find(id);
					UASSERT(jter != edgeConstraints.end());

					if (1 / static_cast<double>(jter->second.infMatrix().at<double>(5,5)) >= 9999.0)
					{
						g2o::VertexPointXY * v2 = new g2o::VertexPointXY();
						v2->setEstimate(Eigen::Vector2d(iter->second.x(), iter->second.y()));
						vertex = v2;
						isLandmarkWithRotation.insert(std::make_pair(id, false));
						id = landmarkVertexOffset - id;
					}
					else
					{
						g2o::VertexSE2 * v2 = new g2o::VertexSE2();
						v2->setEstimate(g2o::SE2(iter->second.x(), iter->second.y(), iter->second.theta()));
						if(id == rootId)
						{
							UDEBUG("Set %d fixed", id);
							v2->setFixed(true);
						}
						vertex = v2;
						isLandmarkWithRotation.insert(std::make_pair(id, true));
						id = landmarkVertexOffset - id;
					}
				}
				else
				{
					continue;
				}
			}
			else
			{
				if(id > 0)
				{
					g2o::VertexSE3 * v3 = new g2o::VertexSE3();

					Eigen::Affine3d a = iter->second.toEigen3d();
					Eigen::Isometry3d pose;
					pose = a.linear();
					pose.translation() = a.translation();
					v3->setEstimate(pose);
					if(id == rootId && !hasGravityConstraints)
					{
						UDEBUG("Set %d fixed", id);
						v3->setFixed(true);
					}
					vertex = v3;
				}
				else if(!landmarksIgnored())
				{
					// check if it is SE3 or only PointXYZ
					std::multimap<int, Link>::const_iterator jter=edgeConstraints.find(id);
					UASSERT(jter != edgeConstraints.end());

					if (1 / static_cast<double>(jter->second.infMatrix().at<double>(3,3)) >= 9999.0 ||
						1 / static_cast<double>(jter->second.infMatrix().at<double>(4,4)) >= 9999.0 ||
						1 / static_cast<double>(jter->second.infMatrix().at<double>(5,5)) >= 9999.0)
					{
						g2o::VertexPointXYZ * v3 = new g2o::VertexPointXYZ();
						v3->setEstimate(Eigen::Vector3d(iter->second.x(), iter->second.y(), iter->second.z()));
						vertex = v3;
						isLandmarkWithRotation.insert(std::make_pair(id, false));
						id = landmarkVertexOffset - id;
					}
					else
					{
						g2o::VertexSE3 * v3 = new g2o::VertexSE3();
						Eigen::Affine3d a = iter->second.toEigen3d();
						Eigen::Isometry3d pose;
						pose = a.linear();
						pose.translation() = a.translation();
						v3->setEstimate(pose);
						if(id == rootId && !hasGravityConstraints)
						{
							UDEBUG("Set %d fixed", id);
							v3->setFixed(true);
						}
						vertex = v3;
						isLandmarkWithRotation.insert(std::make_pair(id, true));
						id = landmarkVertexOffset - id;
					}
				}
				else
				{
					continue;
				}
			}
			if(vertex == 0)
			{
				UERROR("Could not create vertex for node %d", id);
			}
			else
			{
				vertex->setId(id);
				UASSERT_MSG(optimizer.addVertex(vertex), uFormat("cannot insert vertex %d!?", iter->first).c_str());
			}
		}

		// Setup root prior (fixed x,y,z,yaw)
		if(!isSlam2d() && rootId !=0 && hasGravityConstraints)
		{
			g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(rootId));
			if(v1)
			{
				g2o::EdgeSE3Prior * e = new g2o::EdgeSE3Prior();
				e->setVertex(0, v1);
				Eigen::Affine3d a = poses.at(rootId).toEigen3d();
				Eigen::Isometry3d pose;
				pose = a.linear();
				pose.translation() = a.translation();
				e->setMeasurement(pose);
				e->setParameterId(0, PARAM_OFFSET);
				Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity()*10e6;
				// pitch and roll not fixed
				information(3,3) = information(4,4) = 1;
				e->setInformation(information);
				if (!optimizer.addEdge(e))
				{
					delete e;
					UERROR("Map: Failed adding fixed constraint of rootid %d, set as fixed instead", rootId);
					v1->setFixed(true);
				}
				else
				{
					UDEBUG("Set %d fixed with prior (have gravity constraints)", rootId);
				}
			}
			else
			{
				UERROR("Map: Failed adding fixed constraint of rootid %d (not found in added vertices)", rootId);
			}
		}

		UDEBUG("fill edges to g2o...");
#if defined(RTABMAP_VERTIGO)
		int vertigoVertexId = landmarkVertexOffset - (poses.begin()->first<0?poses.begin()->first-1:0);
#endif
		for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
		{
			int id1 = iter->second.from();
			int id2 = iter->second.to();

			UASSERT(!iter->second.transform().isNull());

			g2o::HyperGraph::Edge * edge = 0;

			if(id1 == id2)
			{
				if(iter->second.type() == Link::kPosePrior && !priorsIgnored() &&
				   (!landmarksIgnored() || id1>0))
				{
					int idTag= id1;
					if(id1<0)
					{
						// landmark prior, offset ids
						id1 = landmarkVertexOffset - id1;
						id2 = landmarkVertexOffset - id2;
					}

					if(isSlam2d())
					{
						if(idTag < 0 && !isLandmarkWithRotation.at(idTag))
						{
							g2o::EdgeXYPrior * priorEdge = new g2o::EdgeXYPrior();
							g2o::VertexPointXY* v1 = (g2o::VertexPointXY*)optimizer.vertex(id1);
							priorEdge->setVertex(0, v1);
							priorEdge->setMeasurement(Eigen::Vector2d(iter->second.transform().x(), iter->second.transform().y()));
							Eigen::Matrix<double, 2, 2> information = Eigen::Matrix<double, 2, 2>::Identity();
							if(!isCovarianceIgnored())
							{
								information(0,0) = iter->second.infMatrix().at<double>(0,0); // x-x
								information(0,1) = iter->second.infMatrix().at<double>(0,1); // x-y
								information(1,0) = iter->second.infMatrix().at<double>(1,0); // y-x
								information(1,1) = iter->second.infMatrix().at<double>(1,1); // y-y
							}
							priorEdge->setInformation(information);
							edge = priorEdge;
						}
						else if (1 / static_cast<double>(iter->second.infMatrix().at<double>(5,5)) >= 9999.0)
						{
							g2o::EdgeSE2XYPrior * priorEdge = new g2o::EdgeSE2XYPrior();
							g2o::VertexSE2* v1 = (g2o::VertexSE2*)optimizer.vertex(id1);
							priorEdge->setVertex(0, v1);
							priorEdge->setMeasurement(Eigen::Vector2d(iter->second.transform().x(), iter->second.transform().y()));
							Eigen::Matrix<double, 2, 2> information = Eigen::Matrix<double, 2, 2>::Identity();
							if(!isCovarianceIgnored())
							{
								information(0,0) = iter->second.infMatrix().at<double>(0,0); // x-x
								information(0,1) = iter->second.infMatrix().at<double>(0,1); // x-y
								information(1,0) = iter->second.infMatrix().at<double>(1,0); // y-x
								information(1,1) = iter->second.infMatrix().at<double>(1,1); // y-y
							}
							priorEdge->setInformation(information);
							edge = priorEdge;
						}
						else
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
					}
					else
					{
						if(idTag < 0 && !isLandmarkWithRotation.at(idTag))
						{
							//XYZ case
							g2o::EdgeXYZPrior * priorEdge = new g2o::EdgeXYZPrior();
							g2o::VertexPointXYZ* v1 = (g2o::VertexPointXYZ*)optimizer.vertex(id1);
							priorEdge->setVertex(0, v1);
							priorEdge->setMeasurement(Eigen::Vector3d(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().z()));
							priorEdge->setParameterId(0, PARAM_OFFSET);
							Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
							if(!isCovarianceIgnored())
							{
								information(0,0) = iter->second.infMatrix().at<double>(0,0); // x-x
								information(0,1) = iter->second.infMatrix().at<double>(0,1); // x-y
								information(0,2) = iter->second.infMatrix().at<double>(0,2); // x-z
								information(1,0) = iter->second.infMatrix().at<double>(1,0); // y-x
								information(1,1) = iter->second.infMatrix().at<double>(1,1); // y-y
								information(1,2) = iter->second.infMatrix().at<double>(1,2); // y-z
								information(2,0) = iter->second.infMatrix().at<double>(2,0); // z-x
								information(2,1) = iter->second.infMatrix().at<double>(2,1); // z-y
								information(2,2) = iter->second.infMatrix().at<double>(2,2); // z-z
							}
							priorEdge->setInformation(information);
							edge = priorEdge;
						}
						else if (1 / static_cast<double>(iter->second.infMatrix().at<double>(3,3)) >= 9999.0 ||
						    1 / static_cast<double>(iter->second.infMatrix().at<double>(4,4)) >= 9999.0 ||
							1 / static_cast<double>(iter->second.infMatrix().at<double>(5,5)) >= 9999.0)
						{
							//GPS XYZ case
							g2o::EdgeSE3XYZPrior * priorEdge = new g2o::EdgeSE3XYZPrior();
							g2o::VertexSE3* v1 = (g2o::VertexSE3*)optimizer.vertex(id1);
							priorEdge->setVertex(0, v1);
							priorEdge->setMeasurement(Eigen::Vector3d(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().z()));
							priorEdge->setParameterId(0, PARAM_OFFSET);
							Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
							if(!isCovarianceIgnored())
							{
								information(0,0) = iter->second.infMatrix().at<double>(0,0); // x-x
								information(0,1) = iter->second.infMatrix().at<double>(0,1); // x-y
								information(0,2) = iter->second.infMatrix().at<double>(0,2); // x-z
								information(1,0) = iter->second.infMatrix().at<double>(1,0); // y-x
								information(1,1) = iter->second.infMatrix().at<double>(1,1); // y-y
								information(1,2) = iter->second.infMatrix().at<double>(1,2); // y-z
								information(2,0) = iter->second.infMatrix().at<double>(2,0); // z-x
								information(2,1) = iter->second.infMatrix().at<double>(2,1); // z-y
								information(2,2) = iter->second.infMatrix().at<double>(2,2); // z-z
							}
							priorEdge->setInformation(information);
							edge = priorEdge;
						}
						else
						{
							// XYZ+RPY case
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
				else if(!isSlam2d() && gravitySigma() > 0 && iter->second.type() == Link::kGravity && poses.find(iter->first) != poses.end())
				{
					Eigen::Matrix<double, 6, 1> m;
					// Up vector in robot frame
					m.head<3>() = Eigen::Vector3d::UnitZ();
					// Observed Gravity vector in world frame
					float roll, pitch, yaw;
					iter->second.transform().getEulerAngles(roll, pitch, yaw);
					m.tail<3>() = Transform(0,0,0,roll,pitch,0).toEigen3d() * -Eigen::Vector3d::UnitZ();

					Eigen::MatrixXd information = Eigen::MatrixXd::Identity(3, 3) * 1.0/(gravitySigma()*gravitySigma());

					g2o::VertexSE3* v1 = (g2o::VertexSE3*)optimizer.vertex(id1);
					EdgeSE3Gravity* priorEdge(new EdgeSE3Gravity());
					priorEdge->setMeasurement(m);
					priorEdge->setInformation(information);
					priorEdge->vertices()[0] = v1;
					edge = priorEdge;
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
					int idTag= id2;
					id2 = landmarkVertexOffset - id2;

					if(isSlam2d())
					{
						if(isLandmarkWithRotation.at(idTag))
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
							g2o::EdgeSE2 * e = new g2o::EdgeSE2();
							g2o::VertexSE2* v1 = (g2o::VertexSE2*)optimizer.vertex(id1);
							g2o::VertexSE2* v2 = (g2o::VertexSE2*)optimizer.vertex(id2);
							UASSERT(v1 != 0);
							UASSERT(v2 != 0);
							e->setVertex(0, v1);
							e->setVertex(1, v2);
							e->setMeasurement(g2o::SE2(t.x(), t.y(), t.theta()));
							e->setInformation(information);
							edge = e;
						}
						else
						{
							Eigen::Matrix<double, 2, 2> information = Eigen::Matrix<double, 2, 2>::Identity();
							if(!isCovarianceIgnored())
							{
								cv::Mat linearCov = cv::Mat(iter->second.infMatrix(), cv::Range(0,2), cv::Range(0,2)).clone();
								memcpy(information.data(), linearCov.data, linearCov.total()*sizeof(double));
							}
							g2o::EdgeSE2PointXY* e =  new g2o::EdgeSE2PointXY;
							e->vertices()[0] = optimizer.vertex(id1);
							e->vertices()[1] = optimizer.vertex(id2);
							e->setMeasurement(Eigen::Vector2d(t.x(), t.y()));
							e->setInformation(information);
							e->setParameterId(0, PARAM_OFFSET);
							edge = e;
						}
					}
					else
					{
						if(isLandmarkWithRotation.at(idTag))
						{
							Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
							if(!isCovarianceIgnored())
							{
								memcpy(information.data(), iter->second.infMatrix().data, iter->second.infMatrix().total()*sizeof(double));
							}

							Eigen::Affine3d a = t.toEigen3d();
							Eigen::Isometry3d constraint;
							constraint = a.linear();
							constraint.translation() = a.translation();

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
						else
						{
							Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
							if(!isCovarianceIgnored())
							{
								cv::Mat linearCov = cv::Mat(iter->second.infMatrix(), cv::Range(0,3), cv::Range(0,3)).clone();
								memcpy(information.data(), linearCov.data, linearCov.total()*sizeof(double));
							}

							g2o::EdgeSE3PointXYZ* e =  new g2o::EdgeSE3PointXYZ;
							e->vertices()[0] = optimizer.vertex(id1);
							e->vertices()[1] = optimizer.vertex(id2);
							e->setMeasurement(Eigen::Vector3d(t.x(), t.y(), t.z()));
							e->setInformation(information);
							e->setParameterId(0, PARAM_OFFSET);
							edge = e;
						}
					}
				}
			}
			else
			{
#if defined(RTABMAP_VERTIGO)
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

#if defined(RTABMAP_VERTIGO)
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

#if defined(RTABMAP_VERTIGO)
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

		if(!optimizer.verifyInformationMatrices(true))
		{
			UERROR("This error can be caused by (1) bad covariance matrix "
				"set in odometry messages "
				"(see requirements in g2o::OptimizableGraph::verifyInformationMatrices() function) "
				"or that (2) PCL and g2o hadn't "
				"been built both with or without \"-march=native\" compilation "
				"flag (if one library is built with this flag and not the other, "
				"this is causing Eigen to not work properly, resulting in segmentation faults).");
			return optimizedPoses;
		}

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
								int id = iter->first;
								if(id > 0)
								{
									const g2o::VertexSE2* v = (const g2o::VertexSE2*)optimizer.vertex(id);
									if(v)
									{
										float roll, pitch, yaw;
										iter->second.getEulerAngles(roll, pitch, yaw);
										Transform t(v->estimate().translation()[0], v->estimate().translation()[1], iter->second.z(), roll, pitch, v->estimate().rotation().angle());
										tmpPoses.insert(std::pair<int, Transform>(id, t));
										UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
									}
									else
									{
										UERROR("Vertex %d not found!?", id);
									}
								}
								else if(!landmarksIgnored())
								{
									const g2o::OptimizableGraph::Vertex* v = (const g2o::OptimizableGraph::Vertex*)optimizer.vertex(landmarkVertexOffset - id);
									if(v)
									{
										if(isLandmarkWithRotation.at(id))
										{
											const g2o::VertexSE2* vSE2 = (const g2o::VertexSE2*)v;
											float roll, pitch, yaw;
											iter->second.getEulerAngles(roll, pitch, yaw);
											Transform t(vSE2->estimate().translation()[0], vSE2->estimate().translation()[1], iter->second.z(), roll, pitch, vSE2->estimate().rotation().angle());
											tmpPoses.insert(std::pair<int, Transform>(id, t));
											UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
										}
										else
										{
											const g2o::VertexPointXY* vP = (const g2o::VertexPointXY*)v;
											float roll, pitch, yaw;
											iter->second.getEulerAngles(roll, pitch, yaw);
											Transform t(vP->estimate()[0], vP->estimate()[1], iter->second.z(), roll, pitch, yaw);
											tmpPoses.insert(std::pair<int, Transform>(id, t));
											UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
										}
									}
									else
									{
										UERROR("Vertex %d not found!?", id);
									}
								}
							}
						}
						else
						{
							for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
							{
								int id = iter->first;
								if(id > 0)
								{
									const g2o::VertexSE3* v = (const g2o::VertexSE3*)optimizer.vertex(id);
									if(v)
									{
										Transform t = Transform::fromEigen3d(v->estimate());
										tmpPoses.insert(std::pair<int, Transform>(id, t));
										UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
									}
									else
									{
										UERROR("Vertex %d not found!?", id);
									}
								}
								else if(!landmarksIgnored())
								{
									const g2o::OptimizableGraph::Vertex* v = (const g2o::OptimizableGraph::Vertex*)optimizer.vertex(landmarkVertexOffset - id);
									if(v)
									{
										if(isLandmarkWithRotation.at(id))
										{
											const g2o::VertexSE3* vSE3 = (const g2o::VertexSE3*)v;
											Transform t = Transform::fromEigen3d(vSE3->estimate());
											tmpPoses.insert(std::pair<int, Transform>(id, t));
											UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
										}
										else
										{
											const g2o::VertexPointXYZ* vP = (const g2o::VertexPointXYZ*)v;
											float roll, pitch, yaw;
											iter->second.getEulerAngles(roll, pitch, yaw);
											Transform t(vP->estimate()[0], vP->estimate()[1], vP->estimate()[2], roll, pitch, yaw);
											tmpPoses.insert(std::pair<int, Transform>(id, t));
											UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
										}
									}
									else
									{
										UERROR("Vertex %d not found!?", id);
									}
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
					UERROR("g2o: Large optimimzation error detected (%f), aborting optimization!");
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
			UERROR("g2o: Large optimimzation error detected (%f), aborting optimization!");
			return optimizedPoses;
		}

		if(isSlam2d())
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				int id = iter->first;
				if(id > 0)
				{
					const g2o::VertexSE2* v = (const g2o::VertexSE2*)optimizer.vertex(id);
					if(v)
					{
						float roll, pitch, yaw;
						iter->second.getEulerAngles(roll, pitch, yaw);
						Transform t(v->estimate().translation()[0], v->estimate().translation()[1], iter->second.z(), roll, pitch, v->estimate().rotation().angle());
						optimizedPoses.insert(std::pair<int, Transform>(id, t));
						UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
					}
					else
					{
						UERROR("Vertex %d not found!?", id);
					}
				}
				else if(!landmarksIgnored())
				{
					const g2o::OptimizableGraph::Vertex* v = (const g2o::OptimizableGraph::Vertex*)optimizer.vertex(landmarkVertexOffset - id);

					if(v)
					{
						if(isLandmarkWithRotation.at(id))
						{
							const g2o::VertexSE2* vSE2 = (const g2o::VertexSE2*)v;
							float roll, pitch, yaw;
							iter->second.getEulerAngles(roll, pitch, yaw);
							Transform t(vSE2->estimate().translation()[0], vSE2->estimate().translation()[1], iter->second.z(), roll, pitch, vSE2->estimate().rotation().angle());
							optimizedPoses.insert(std::pair<int, Transform>(id, t));
							UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
						}
						else
						{
							const g2o::VertexPointXY* vP = (const g2o::VertexPointXY*)v;
							float roll, pitch, yaw;
							iter->second.getEulerAngles(roll, pitch, yaw);
							Transform t(vP->estimate()[0], vP->estimate()[1], iter->second.z(), roll, pitch, yaw);
							optimizedPoses.insert(std::pair<int, Transform>(id, t));
							UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
						}
					}
					else
					{
						UERROR("Vertex %d not found!?", id);
					}
				}
			}

			g2o::VertexSE2* v = (g2o::VertexSE2*)optimizer.vertex(poses.rbegin()->first);
			if(v)
			{
				UTimer t;
				g2o::SparseBlockMatrix<g2o::MatrixXD> spinv;
				optimizer.computeMarginals(spinv, v);
				UINFO("Computed marginals = %fs (cols=%d rows=%d, v=%d id=%d)", t.ticks(), spinv.cols(), spinv.rows(), v->hessianIndex(), poses.rbegin()->first);
				if(v->hessianIndex() >= 0 && v->hessianIndex() < (int)spinv.blockCols().size())
				{
					g2o::SparseBlockMatrix<g2o::MatrixXD>::SparseMatrixBlock * block = spinv.blockCols()[v->hessianIndex()].begin()->second;
					UASSERT(block && block->cols() == 3 && block->cols() == 3);
					outputCovariance.at<double>(0,0) = (*block)(0,0); // x-x
					outputCovariance.at<double>(0,1) = (*block)(0,1); // x-y
					outputCovariance.at<double>(0,5) = (*block)(0,2); // x-theta
					outputCovariance.at<double>(1,0) = (*block)(1,0); // y-x
					outputCovariance.at<double>(1,1) = (*block)(1,1); // y-y
					outputCovariance.at<double>(1,5) = (*block)(1,2); // y-theta
					outputCovariance.at<double>(5,0) = (*block)(2,0); // theta-x
					outputCovariance.at<double>(5,1) = (*block)(2,1); // theta-y
					outputCovariance.at<double>(5,5) = (*block)(2,2); // theta-theta
				}
				else if(v->hessianIndex() < 0)
				{
					UWARN("Computing marginals: vertex %d has negative hessian index (%d). Cannot compute last pose covariance.", poses.rbegin()->first, v->hessianIndex());
				}
				else
				{
					UWARN("Computing marginals: vertex %d has hessian not valid (%d > block size=%d). Cannot compute last pose covariance.", poses.rbegin()->first, v->hessianIndex(), (int)spinv.blockCols().size());
				}
			}
			else
			{
				UERROR("Vertex %d not found!? Cannot compute marginals...", poses.rbegin()->first);
			}
		}
		else
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				int id = iter->first;
				if(id > 0)
				{
					const g2o::VertexSE3* v = (const g2o::VertexSE3*)optimizer.vertex(id);
					if(v)
					{
						Transform t = Transform::fromEigen3d(v->estimate());
						optimizedPoses.insert(std::pair<int, Transform>(id, t));
						UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
					}
					else
					{
						UERROR("Vertex %d not found!?", id);
					}
				}
				else if(!landmarksIgnored())
				{
					const g2o::OptimizableGraph::Vertex* v = (const g2o::OptimizableGraph::Vertex*)optimizer.vertex(landmarkVertexOffset - id);

					if(v)
					{
						if(isLandmarkWithRotation.at(id))
						{
							const g2o::VertexSE3* vSE3 = (const g2o::VertexSE3*)v;
							Transform t = Transform::fromEigen3d(vSE3->estimate());
							optimizedPoses.insert(std::pair<int, Transform>(id, t));
							UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
						}
						else
						{
							const g2o::VertexPointXYZ* vP = (const g2o::VertexPointXYZ*)v;
							float roll, pitch, yaw;
							iter->second.getEulerAngles(roll, pitch, yaw);
							Transform t(vP->estimate()[0], vP->estimate()[1], vP->estimate()[2], roll, pitch, yaw);
							optimizedPoses.insert(std::pair<int, Transform>(id, t));
							UASSERT_MSG(!t.isNull(), uFormat("Optimized pose %d is null!?!?", id).c_str());
						}
					}
					else
					{
						UERROR("Vertex %d not found!?", id);
					}
				}
			}

			g2o::VertexSE3* v = (g2o::VertexSE3*)optimizer.vertex(poses.rbegin()->first);
			if(v)
			{
				UTimer t;
				g2o::SparseBlockMatrix<g2o::MatrixXD> spinv;
				optimizer.computeMarginals(spinv, v);
				UINFO("Computed marginals = %fs (cols=%d rows=%d, v=%d id=%d)", t.ticks(), spinv.cols(), spinv.rows(), v->hessianIndex(), poses.rbegin()->first);
				if(v->hessianIndex() >= 0 && v->hessianIndex() < (int)spinv.blockCols().size())
				{
					g2o::SparseBlockMatrix<g2o::MatrixXD>::SparseMatrixBlock * block = spinv.blockCols()[v->hessianIndex()].begin()->second;
					UASSERT(block && block->cols() == 6 && block->cols() == 6);
					memcpy(outputCovariance.data, block->data(), outputCovariance.total()*sizeof(double));
				}
				else if(v->hessianIndex() < 0)
				{
					UWARN("Computing marginals: vertex %d has negative hessian index (%d). Cannot compute last pose covariance.", poses.rbegin()->first, v->hessianIndex());
				}
#ifdef RTABMAP_G2O_CPP11
				else
				{
					UWARN("Computing marginals: vertex %d has hessian not valid (%d > block size=%d). Cannot compute last pose covariance.", poses.rbegin()->first, v->hessianIndex(), (int)spinv.blockCols().size());
				}
#endif
			}
			else
			{
				UERROR("Vertex %d not found!? Cannot compute marginals...", poses.rbegin()->first);
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
#ifdef RTABMAP_ORB_SLAM
	UERROR("G2O graph optimization cannot be used with g2o built from ORB_SLAM, only SBA is available.");
#else
	UERROR("Not built with G2O support!");
#endif
#endif
	return optimizedPoses;
}

#ifdef RTABMAP_ORB_SLAM
/**
 * \brief 3D edge between two SBAcam
 */
 class EdgeSE3Expmap : public g2o::BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3Expmap():  BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>(){}
    bool read(std::istream& is)
      {
        return false;
      }

      bool write(std::ostream& os) const
      {
        return false;
      }

    void computeError()
    {
      const g2o::VertexSE3Expmap* v1 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
      const g2o::VertexSE3Expmap* v2 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
      g2o::SE3Quat delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
      _error[0]=delta.translation().x();
      _error[1]=delta.translation().y();
      _error[2]=delta.translation().z();
      _error[3]=delta.rotation().x();
      _error[4]=delta.rotation().y();
      _error[5]=delta.rotation().z();
    }

    virtual void setMeasurement(const g2o::SE3Quat& meas){
      _measurement=meas;
      _inverseMeasurement=meas.inverse();
    }

    virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}
    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from_, g2o::OptimizableGraph::Vertex* ){
    	g2o::VertexSE3Expmap* from = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
    	g2o::VertexSE3Expmap* to = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
		if (from_.count(from) > 0)
		  to->setEstimate((g2o::SE3Quat) from->estimate() * _measurement);
		else
		  from->setEstimate((g2o::SE3Quat) to->estimate() * _inverseMeasurement);
    }

    virtual bool setMeasurementData(const double* d){
      Eigen::Map<const g2o::Vector7d> v(d);
      _measurement.fromVector(v);
      _inverseMeasurement = _measurement.inverse();
      return true;
    }

    virtual bool getMeasurementData(double* d) const{
      Eigen::Map<g2o::Vector7d> v(d);
      v = _measurement.toVector();
      return true;
    }

    virtual int measurementDimension() const {return 7;}

    virtual bool setMeasurementFromState() {
    	const g2o::VertexSE3Expmap* v1 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
		const g2o::VertexSE3Expmap* v2 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
		_measurement = (v1->estimate().inverse()*v2->estimate());
		_inverseMeasurement = _measurement.inverse();
		return true;
    }

  protected:
    g2o::SE3Quat _inverseMeasurement;
};
#endif

std::map<int, Transform> OptimizerG2O::optimizeBA(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, std::vector<CameraModel> > & models,
		std::map<int, cv::Point3f> & points3DMap,
		const std::map<int, std::map<int, FeatureBA> > & wordReferences,
		std::set<int> * outliers)
{
	std::map<int, Transform> optimizedPoses;
#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM)
	UDEBUG("Optimizing graph...");

	optimizedPoses.clear();
	if(poses.size()>=2 && iterations() > 0 && (models.size() == poses.size() || poses.begin()->first < 0))
	{
		g2o::SparseOptimizer optimizer;
		//optimizer.setVerbose(ULogger::level()==ULogger::kDebug);
#if defined(RTABMAP_G2O_CPP11) && !defined(RTABMAP_ORB_SLAM)
		std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
#else
		g2o::BlockSolver_6_3::LinearSolverType * linearSolver = 0;
#endif

#ifdef RTABMAP_ORB_SLAM
		linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
#else
		if(solver_ == 3)
		{
			//eigen
#ifdef RTABMAP_G2O_CPP11
			linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType> >();
#else
			linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
#endif
		}
#ifdef G2O_HAVE_CHOLMOD
		else if(solver_ == 2)
		{
			//chmold
#ifdef RTABMAP_G2O_CPP11
			linearSolver = std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> >();
#else
			linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
#endif
		}
#endif
#ifdef G2O_HAVE_CSPARSE
		else if(solver_ == 0)
		{
			//csparse
#ifdef RTABMAP_G2O_CPP11
			linearSolver = std::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType> >();
#else
			linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();
#endif
		}
#endif
		else
		{
			//pcg
#ifdef RTABMAP_G2O_CPP11
			linearSolver = std::make_unique<g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType> >();
#else
			linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
#endif
		}
#endif // RTABMAP_ORB_SLAM

#ifndef RTABMAP_ORB_SLAM
		if(optimizer_ == 1)
		{
#ifdef RTABMAP_G2O_CPP11
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(
					std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))));
#else
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(new g2o::BlockSolver_6_3(linearSolver)));
#endif
		}
		else
#endif
		{
#if defined(RTABMAP_G2O_CPP11) && !defined(RTABMAP_ORB_SLAM)
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(
					std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))));
#else
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(new g2o::BlockSolver_6_3(linearSolver)));
#endif
		}


		UDEBUG("fill poses to g2o...");
		for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			if(iter->first > 0)
			{
				// Get camera model
				std::map<int, std::vector<CameraModel> >::const_iterator iterModel = models.find(iter->first);
				UASSERT(iterModel != models.end() && !iterModel->second.empty());
				for(size_t i=0; i<iterModel->second.size(); ++i)
				{
					UASSERT(iterModel->second[i].isValidForProjection());

					Transform camPose = iter->second * iterModel->second[i].localTransform();

					// Add node's pose
					UASSERT(!camPose.isNull());
#ifdef RTABMAP_ORB_SLAM
					g2o::VertexSE3Expmap * vCam = new g2o::VertexSE3Expmap();
#else
					g2o::VertexCam * vCam = new g2o::VertexCam();
#endif

					Eigen::Affine3d a = camPose.toEigen3d();
#ifdef RTABMAP_ORB_SLAM
					a = a.inverse();
					vCam->setEstimate(g2o::SE3Quat(a.linear(), a.translation()));
#else
					g2o::SBACam cam(Eigen::Quaterniond(a.linear()), a.translation());
					cam.setKcam(
							iterModel->second[i].fx(),
							iterModel->second[i].fy(),
							iterModel->second[i].cx(),
							iterModel->second[i].cy(),
							iterModel->second[i].Tx()<0.0?-iterModel->second[i].Tx()/iterModel->second[i].fx():baseline_); // baseline in meters
					vCam->setEstimate(cam);
#endif
					vCam->setId(iter->first*MULTICAM_OFFSET + i);

					// negative root means that all other poses should be fixed instead of the root
					vCam->setFixed((rootId >= 0 && iter->first == rootId) || (rootId < 0 && iter->first != -rootId));

					/*UDEBUG("camPose %d (camid=%d) (fixed=%d) fx=%f fy=%f cx=%f cy=%f Tx=%f baseline=%f t=%s",
							iter->first,
							vCam->id(),
							vCam->fixed()?1:0,
							iterModel->second[i].fx(),
							iterModel->second[i].fy(),
							iterModel->second[i].cx(),
							iterModel->second[i].cy(),
							iterModel->second[i].Tx(),
							iterModel->second[i].Tx()<0.0?-iterModel->second[i].Tx()/iterModel->second[i].fx():baseline_,
							camPose.prettyPrint().c_str());*/

					UASSERT_MSG(optimizer.addVertex(vCam), uFormat("cannot insert cam vertex %d (pose=%d)!?", vCam->id(), iter->first).c_str());
				}
			}
		}

		UDEBUG("fill edges to g2o...");
		for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
		{
			if(iter->second.from() > 0 &&
			   iter->second.to() > 0 &&
			   uContains(poses, iter->second.from()) &&
			   uContains(poses, iter->second.to()))
			{
				// add edge
				int id1 = iter->second.from();
				int id2 = iter->second.to();

				if(id1 == id2)
				{
#ifndef RTABMAP_ORB_SLAM
					g2o::HyperGraph::Edge * edge = 0;
					if(gravitySigma() > 0 && iter->second.type() == Link::kGravity && poses.find(iter->first) != poses.end())
					{
						Eigen::Matrix<double, 6, 1> m;
						// Up vector in robot frame
						m.head<3>() = Eigen::Vector3d::UnitZ();
						// Observed Gravity vector in world frame
						float roll, pitch, yaw;
						iter->second.transform().getEulerAngles(roll, pitch, yaw);
						m.tail<3>() = Transform(0,0,0,roll,pitch,0).toEigen3d() * -Eigen::Vector3d::UnitZ();

						Eigen::MatrixXd information = Eigen::MatrixXd::Identity(3, 3) * 1.0/(gravitySigma()*gravitySigma());

						g2o::VertexCam* v1 = (g2o::VertexCam*)optimizer.vertex(id1*MULTICAM_OFFSET);
						EdgeSBACamGravity* priorEdge(new EdgeSBACamGravity());
						std::map<int, std::vector<CameraModel> >::const_iterator iterModel = models.find(iter->first);
						// Gravity constraint added only to first camera of a pose
						UASSERT(iterModel != models.end() && !iterModel->second.empty() && !iterModel->second[0].localTransform().isNull());
						priorEdge->setCameraInvLocalTransform(iterModel->second[0].localTransform().inverse().toEigen3d().linear());
						priorEdge->setMeasurement(m);
						priorEdge->setInformation(information);
						priorEdge->vertices()[0] = v1;
						edge = priorEdge;
					}
					if (edge && !optimizer.addEdge(edge))
					{
						delete edge;
						UERROR("Map: Failed adding constraint between %d and %d, skipping", id1, id2);
						return optimizedPoses;
					}
#endif
				}
				else if(id1>0 && id2>0) // not supporting landmarks
				{
					UASSERT(!iter->second.transform().isNull());

					Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
					if(!isCovarianceIgnored())
					{
						memcpy(information.data(), iter->second.infMatrix().data, iter->second.infMatrix().total()*sizeof(double));
					}

					// between cameras, not base_link
					Transform camLink = models.at(id1)[0].localTransform().inverse()*iter->second.transform()*models.at(id2)[0].localTransform();
					/*UDEBUG("added edge %d->%d (camIDs %d->%d) (in cam frame=%s)",
							id1,
							id2,
							id1*MULTICAM_OFFSET,
							id2*MULTICAM_OFFSET,
							camLink.prettyPrint().c_str());*/
#ifdef RTABMAP_ORB_SLAM
					EdgeSE3Expmap * e = new EdgeSE3Expmap();
					g2o::VertexSE3Expmap* v1 = (g2o::VertexSE3Expmap*)optimizer.vertex(id1*MULTICAM_OFFSET);
					g2o::VertexSE3Expmap* v2 = (g2o::VertexSE3Expmap*)optimizer.vertex(id2*MULTICAM_OFFSET);

					Transform camPose1 = Transform::fromEigen3d(v1->estimate()).inverse();
					Transform camPose2Inv = Transform::fromEigen3d(v2->estimate());

					camLink =  camPose1 * camPose1 * camLink * camPose2Inv * camPose2Inv;
#else
					g2o::EdgeSBACam * e = new g2o::EdgeSBACam();
					g2o::VertexCam* v1 = (g2o::VertexCam*)optimizer.vertex(id1*MULTICAM_OFFSET);
					g2o::VertexCam* v2 = (g2o::VertexCam*)optimizer.vertex(id2*MULTICAM_OFFSET);
#endif
					UASSERT(v1 != 0);
					UASSERT(v2 != 0);
					e->setVertex(0, v1);
					e->setVertex(1, v2);
					Eigen::Affine3d a = camLink.toEigen3d();
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

		UDEBUG("fill hard edges between camera 0 and other cameras (multicam)...");
		for(std::map<int, std::vector<CameraModel> >::const_iterator iter=models.begin(); iter!=models.end(); ++iter)
		{
			int id = iter->first;
			if(uContains(poses, id))
			{
				for(size_t i=1; i<iter->second.size(); ++i)
				{
					// add edge
					// Set large information matrix to keep these links fixed
					Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity()*9999999;

					// between cameras, not base_link
					Transform camLink = iter->second[0].localTransform().inverse()*iter->second[i].localTransform();
#ifdef RTABMAP_ORB_SLAM
					EdgeSE3Expmap * e = new EdgeSE3Expmap();
					g2o::VertexSE3Expmap* v1 = (g2o::VertexSE3Expmap*)optimizer.vertex(id*MULTICAM_OFFSET);
					g2o::VertexSE3Expmap* v2 = (g2o::VertexSE3Expmap*)optimizer.vertex(id*MULTICAM_OFFSET+i);

					Transform camPose1 = Transform::fromEigen3d(v1->estimate()).inverse();
					Transform camPose2Inv = Transform::fromEigen3d(v2->estimate());

					camLink =  camPose1 * camPose1 * camLink * camPose2Inv * camPose2Inv;
#else
					g2o::EdgeSBACam * e = new g2o::EdgeSBACam();
					g2o::VertexCam* v1 = (g2o::VertexCam*)optimizer.vertex(id*MULTICAM_OFFSET);
					g2o::VertexCam* v2 = (g2o::VertexCam*)optimizer.vertex(id*MULTICAM_OFFSET+i);
#endif
					/*UDEBUG("added edge between subcam 0->%d id:%d->%d (in cam frame=%s)",
							i,
							v1->id(),
							v2->id(),
							camLink.prettyPrint().c_str());*/

					UASSERT(v1 != 0);
					UASSERT(v2 != 0);
					e->setVertex(0, v1);
					e->setVertex(1, v2);
					Eigen::Affine3d a = camLink.toEigen3d();
					e->setMeasurement(g2o::SE3Quat(a.linear(), a.translation()));
					e->setInformation(information);

					if (!optimizer.addEdge(e))
					{
						delete e;
						UERROR("Map: Failed adding constraint between %d and %d, skipping", v1->id(), v2->id());
						return optimizedPoses;
					}
				}
			}
		}

		UDEBUG("fill 3D points to g2o...");
		const int stepVertexId = poses.rbegin()->first*MULTICAM_OFFSET+MULTICAM_OFFSET;
		int negVertexOffset = stepVertexId;
		if(wordReferences.size() && wordReferences.rbegin()->first>0)
		{
			negVertexOffset += wordReferences.rbegin()->first;
		}
		UDEBUG("stepVertexId=%d, negVertexOffset=%d", stepVertexId, negVertexOffset);
		std::list<g2o::OptimizableGraph::Edge*> edges;
		for(std::map<int, std::map<int, FeatureBA> >::const_iterator iter = wordReferences.begin(); iter!=wordReferences.end(); ++iter)
		{
			int id = iter->first;
			if(points3DMap.find(id) != points3DMap.end())
			{
				cv::Point3f pt3d = points3DMap.at(id);
				if(!util3d::isFinite(pt3d))
				{
					UWARN("Ignoring 3D point %d because it has nan value(s)!", id);
					continue;
				}
				g2o::VertexSBAPointXYZ* vpt3d = new g2o::VertexSBAPointXYZ();

				vpt3d->setEstimate(Eigen::Vector3d(pt3d.x, pt3d.y, pt3d.z));
				if(id<0)
				{
					vpt3d->setId(negVertexOffset + id*-1);
				}
				else
				{
					vpt3d->setId(stepVertexId + id);
				}
				UASSERT(vpt3d->id() > 0);
				vpt3d->setMarginalized(true);
				optimizer.addVertex(vpt3d);

				//UDEBUG("Added 3D point %d (%f,%f,%f)", vpt3d->id()-stepVertexId, pt3d.x, pt3d.y, pt3d.z);

				// set observations
				for(std::map<int, FeatureBA>::const_iterator jter=iter->second.begin(); jter!=iter->second.end(); ++jter)
				{
					int poseId = jter->first;
					int camIndex = jter->second.cameraIndex;
					int camId = poseId*MULTICAM_OFFSET+camIndex;
					if(poses.find(poseId) != poses.end() && optimizer.vertex(camId) != 0)
					{
						const FeatureBA & pt = jter->second;
						double depth = pt.depth;

						//UDEBUG("Added observation pt=%d to cam=%d (%d,%d) depth=%f (camIndex=%d)", vpt3d->id()-stepVertexId, camId, (int)pt.kpt.pt.x, (int)pt.kpt.pt.y, depth, camIndex);

						g2o::OptimizableGraph::Edge * e;
						double baseline = 0.0;
#ifdef RTABMAP_ORB_SLAM
						g2o::VertexSE3Expmap* vcam = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(camId));
						std::map<int, std::vector<CameraModel> >::const_iterator iterModel = models.find(poseId);

						UASSERT(iterModel != models.end() && camIndex<iterModel->second.size() && iterModel->second[camIndex].isValidForProjection());
						baseline = iterModel->second[camIndex].Tx()<0.0?-iterModel->second[camIndex].Tx()/iterModel->second[camIndex].fx():baseline_;
#else
						g2o::VertexCam* vcam = dynamic_cast<g2o::VertexCam*>(optimizer.vertex(camId));
						baseline = vcam->estimate().baseline;
#endif
						double variance = pixelVariance_;
						if(uIsFinite(depth) && depth > 0.0 && baseline > 0.0)
						{
							// stereo edge
#ifdef RTABMAP_ORB_SLAM
							g2o::EdgeStereoSE3ProjectXYZ* es = new g2o::EdgeStereoSE3ProjectXYZ();
							float disparity = baseline * iterModel->second[camIndex].fx() / depth;
							Eigen::Vector3d obs( pt.kpt.pt.x, pt.kpt.pt.y, pt.kpt.pt.x-disparity);
							es->setMeasurement(obs);
							//variance *= log(exp(1)+disparity);
							es->setInformation(Eigen::Matrix3d::Identity() / variance);
							es->fx = iterModel->second[camIndex].fx();
							es->fy = iterModel->second[camIndex].fy();
							es->cx = iterModel->second[camIndex].cx();
							es->cy = iterModel->second[camIndex].cy();
							es->bf = baseline*es->fx;
							e = es;
#else
							g2o::EdgeProjectP2SC* es = new g2o::EdgeProjectP2SC();
							float disparity = baseline * vcam->estimate().Kcam(0,0) / depth;
							Eigen::Vector3d obs( pt.kpt.pt.x, pt.kpt.pt.y, pt.kpt.pt.x-disparity);
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
								UDEBUG("Stereo camera model detected but current "
										"observation (pt=%d to cam=%d, kpt=[%d,%d]) has null depth (%f m), adding "
										"mono observation instead.",
										vpt3d->id()-stepVertexId, camId, (int)pt.kpt.pt.x, (int)pt.kpt.pt.y, depth);
							}
							// mono edge
#ifdef RTABMAP_ORB_SLAM
							g2o::EdgeSE3ProjectXYZ* em = new g2o::EdgeSE3ProjectXYZ();
							Eigen::Vector2d obs( pt.kpt.pt.x, pt.kpt.pt.y);
							em->setMeasurement(obs);
							em->setInformation(Eigen::Matrix2d::Identity() / variance);
							em->fx = iterModel->second[camIndex].fx();
							em->fy = iterModel->second[camIndex].fy();
							em->cx = iterModel->second[camIndex].cx();
							em->cy = iterModel->second[camIndex].cy();
							e = em;

#else
							g2o::EdgeProjectP2MC* em = new g2o::EdgeProjectP2MC();
							Eigen::Vector2d obs( pt.kpt.pt.x, pt.kpt.pt.y);
							em->setMeasurement(obs);
							em->setInformation(Eigen::Matrix2d::Identity() / variance);
							e = em;
#endif
						}
						e->setVertex(0, vpt3d);
						e->setVertex(1, vcam);

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

		UDEBUG("g2o optimizing begin (max iterations=%d, robustKernel=%f)", iterations(), robustKernelDelta_);

		int it = 0;
		UTimer timer;
		int outliersCount = 0;
		int outliersCountFar = 0;

		for(int i=0; i<(robustKernelDelta_>0.0?2:1); ++i)
		{
			it += optimizer.optimize(i==0&&robustKernelDelta_>0.0?5:iterations());

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
#ifdef RTABMAP_ORB_SLAM
						if(dynamic_cast<g2o::EdgeStereoSE3ProjectXYZ*>(*iter) != 0)
						{
							d = ((g2o::EdgeStereoSE3ProjectXYZ*)(*iter))->measurement()[0]-((g2o::EdgeStereoSE3ProjectXYZ*)(*iter))->measurement()[2];
						}
						//UDEBUG("Ignoring edge (%d<->%d) d=%f var=%f kernel=%f chi2=%f", (*iter)->vertex(0)->id()-stepVertexId, (*iter)->vertex(1)->id(), d, 1.0/((g2o::EdgeStereoSE3ProjectXYZ*)(*iter))->information()(0,0), (*iter)->robustKernel()->delta(), (*iter)->chi2());
#else
						if(dynamic_cast<g2o::EdgeProjectP2SC*>(*iter) != 0)
						{
							d = ((g2o::EdgeProjectP2SC*)(*iter))->measurement()[0]-((g2o::EdgeProjectP2SC*)(*iter))->measurement()[2];
						}
						//UDEBUG("Ignoring edge (%d<->%d) d=%f var=%f kernel=%f chi2=%f", (*iter)->vertex(0)->id()-stepVertexId, (*iter)->vertex(1)->id(), d, 1.0/((g2o::EdgeProjectP2SC*)(*iter))->information()(0,0), (*iter)->robustKernel()->delta(), (*iter)->chi2());
#endif

						cv::Point3f pt3d;
						if((*iter)->vertex(0)->id() > negVertexOffset)
						{
							pt3d = points3DMap.at(negVertexOffset - (*iter)->vertex(0)->id());
						}
						else
						{
							pt3d = points3DMap.at((*iter)->vertex(0)->id()-stepVertexId);
						}
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
		UDEBUG("g2o optimizing end (%d iterations done, error=%f, outliers=%d/%d (delta=%f) time = %f s)", it, optimizer.activeRobustChi2(), outliersCount, (int)edges.size(), robustKernelDelta_, timer.ticks());

		if(optimizer.activeRobustChi2() > 1000000000000.0)
		{
			UWARN("g2o: Large optimization error detected (%f), aborting optimization!");
			return optimizedPoses;
		}

		// update poses
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if(iter->first > 0)
			{
				int camId = iter->first*MULTICAM_OFFSET;
#ifdef RTABMAP_ORB_SLAM
				const g2o::VertexSE3Expmap* v = (const g2o::VertexSE3Expmap*)optimizer.vertex(camId);
#else
				const g2o::VertexCam* v = (const g2o::VertexCam*)optimizer.vertex(camId);
#endif
				if(v)
				{
					Transform t = Transform::fromEigen3d(v->estimate());

#ifdef RTABMAP_ORB_SLAM
					t=t.inverse();
#endif

					// remove model local transform
					t *= models.at(iter->first)[0].localTransform().inverse();

					//UDEBUG("%d from=%s to=%s", iter->first, iter->second.prettyPrint().c_str(), t.prettyPrint().c_str());
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
					UERROR("Vertex (pose) %d (cam=%d) not found!?", iter->first, camId);
				}
			}
		}

		//update points3D

		for(std::map<int, cv::Point3f>::iterator iter = points3DMap.begin(); iter!=points3DMap.end(); ++iter)
		{
			const g2o::VertexSBAPointXYZ* v;
			int id = iter->first;
			if(id<0)
			{
				v = (const g2o::VertexSBAPointXYZ*)optimizer.vertex(negVertexOffset + id*-1);
			}
			else
			{
				v = (const g2o::VertexSBAPointXYZ*)optimizer.vertex(stepVertexId + id);
			}

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
	else if(poses.size() > 1 && (poses.size() != models.size() && poses.begin()->first > 0))
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
		const std::multimap<int, Link> & edgeConstraints)
{
	FILE * file = 0;

#ifdef _MSC_VER
	fopen_s(&file, fileName.c_str(), "w");
#else
	file = fopen(fileName.c_str(), "w");
#endif

	if(file)
	{
		// force periods to be used instead of commas
		setlocale(LC_ALL, "en_US.UTF-8");

		if(isSlam2d())
		{
			// PARAMS_SE2OFFSET id x y theta (set for priors)
			fprintf(file, "PARAMS_SE2OFFSET %d 0 0 0\n", PARAM_OFFSET);
		}
		else
		{
			// PARAMS_SE3OFFSET id x y z qw qx qy qz (set for priors)
			Eigen::Vector3f v = Eigen::Vector3f::Zero();
			Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
			fprintf(file, "PARAMS_SE3OFFSET %d %f %f %f %f %f %f %f\n",
				PARAM_OFFSET,
				v.x(),
				v.y(),
				v.z(),
				q.x(),
				q.y(),
				q.z(),
				q.w());
		}

		int landmarkOffset = poses.size()&&poses.rbegin()->first>0?poses.rbegin()->first+1:0;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if (isSlam2d())
			{
				if(iter->first > 0)
				{
					// VERTEX_SE2 id x y theta
					fprintf(file, "VERTEX_SE2 %d %f %f %f\n",
						landmarkOffset-iter->first,
						iter->second.x(),
						iter->second.y(),
						iter->second.theta());
				}
				else if(!landmarksIgnored())
				{
					// VERTEX_XY id x y
					fprintf(file, "VERTEX_XY %d %f %f\n",
						iter->first,
						iter->second.x(),
						iter->second.y());
				}
			}
			else
			{
				if(iter->first > 0)
				{
					// VERTEX_SE3 id x y z qw qx qy qz
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
				else if(!landmarksIgnored())
				{
					// VERTEX_TRACKXYZ id x y z
					fprintf(file, "VERTEX_TRACKXYZ %d %f %f %f\n",
						landmarkOffset-iter->first,
						iter->second.x(),
						iter->second.y(),
						iter->second.z());
				}
			}
		}

		int virtualVertexId = landmarkOffset - (poses.size()&&poses.rbegin()->first<0?poses.rbegin()->first:0);
		for(std::multimap<int, Link>::const_iterator iter = edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
		{
			if (iter->second.type() == Link::kLandmark)
			{
				if (this->landmarksIgnored())
				{
					continue;
				}
				if(isSlam2d())
				{
					// EDGE_SE2_XY observed_vertex_id observing_vertex_id x y inf_11 inf_12 inf_22
					fprintf(file, "EDGE_SE2_XY %d %d %f %f %f %f %f\n",
						iter->second.from()<0?landmarkOffset-iter->second.from():iter->second.from(),
						iter->second.to()<0?landmarkOffset-iter->second.to():iter->second.to(),
						iter->second.transform().x(),
						iter->second.transform().y(),
						iter->second.infMatrix().at<double>(0, 0),
						iter->second.infMatrix().at<double>(0, 1),
						iter->second.infMatrix().at<double>(1, 1));
				}
				else
				{
					// EDGE_SE3_TRACKXYZ observed_vertex_id observing_vertex_id param_offset x y z inf_11 inf_12 inf_13 inf_22 inf_23 inf_33
					fprintf(file, "EDGE_SE3_TRACKXYZ %d %d %d %f %f %f %f %f %f %f %f %f\n",
						iter->second.from()<0?landmarkOffset-iter->second.from():iter->second.from(),
						iter->second.to()<0?landmarkOffset-iter->second.to():iter->second.to(),
						PARAM_OFFSET,
						iter->second.transform().x(),
						iter->second.transform().y(),
						iter->second.transform().z(),
						iter->second.infMatrix().at<double>(0, 0),
						iter->second.infMatrix().at<double>(0, 1),
						iter->second.infMatrix().at<double>(0, 2),
						iter->second.infMatrix().at<double>(1, 1),
						iter->second.infMatrix().at<double>(1, 2),
						iter->second.infMatrix().at<double>(2, 2));
				}
				continue;
			}

			std::string prefix = isSlam2d()? "EDGE_SE2" :"EDGE_SE3:QUAT";
			std::string suffix = "";
			std::string to = uFormat(" %d", iter->second.to());

			bool isSE2 = true;
			bool isSE3 = true;

			if (iter->second.type() == Link::kGravity)
			{
				continue;
			}
			else if (iter->second.type() == Link::kPosePrior)
			{
				if (this->priorsIgnored())
				{
					continue;
				}
				if (isSlam2d())
				{
					if (1 / static_cast<double>(iter->second.infMatrix().at<double>(5,5)) >= 9999.0)
					{
						prefix = "EDGE_PRIOR_SE2_XY";
						isSE2 = false;
					}
					else
					{
						prefix = "EDGE_PRIOR_SE2";
					}

					//  based on https://github.com/RainerKuemmerle/g2o/blob/38347944c6ad7a3b31976b97406ff0de20be1530/g2o/types/slam2d/edge_se2_prior.cpp#L42
					//  there is no pid for the 2d prior case
					to = "";
				}
				else
				{
					if (1 / static_cast<double>(iter->second.infMatrix().at<double>(3,3)) >= 9999.0 ||
						1 / static_cast<double>(iter->second.infMatrix().at<double>(4,4)) >= 9999.0 ||
						1 / static_cast<double>(iter->second.infMatrix().at<double>(5,5)) >= 9999.0)
					{
						to = "";
						prefix = "EDGE_POINTXYZ_PRIOR";
						isSE3 = false;
					}
					else
					{
						to = uFormat(" %d", PARAM_OFFSET);
						prefix = "EDGE_SE3_PRIOR";
					}
				}
			}
			else if(this->isRobust() &&
			   iter->second.type() != Link::kNeighbor &&
			   iter->second.type() != Link::kNeighborMerged)
			{
				fprintf(file, "VERTEX_SWITCH %d 1\n", virtualVertexId);
				fprintf(file, "EDGE_SWITCH_PRIOR %d 1 1.0\n", virtualVertexId);
				prefix = isSlam2d() ? "EDGE_SE2_SWITCHABLE" : "EDGE_SE3_SWITCHABLE";
				suffix = uFormat(" %d", virtualVertexId++);
			}

			if(isSlam2d())
			{
				if (isSE2)
				{
					// EDGE_SE2 observed_vertex_id observing_vertex_id x y qx qy qz qw inf_11 inf_12 inf_13 inf_22 inf_23 inf_33
					// EDGE_SE2_PRIOR observed_vertex_id x y qx qy qz qw inf_11 inf_12 inf_13 inf_22 inf_23 inf_33
					fprintf(file, "%s %d%s%s %f %f %f %f %f %f %f %f %f\n",
        					prefix.c_str(),
        					iter->second.from(),
        					to.c_str(),
        					suffix.c_str(),
        					iter->second.transform().x(),
        					iter->second.transform().y(),
        					iter->second.transform().theta(),
        					iter->second.infMatrix().at<double>(0, 0),
        					iter->second.infMatrix().at<double>(0, 1),
        					iter->second.infMatrix().at<double>(0, 5),
        					iter->second.infMatrix().at<double>(1, 1),
        					iter->second.infMatrix().at<double>(1, 5),
        					iter->second.infMatrix().at<double>(5, 5));
				}
				else
				{
					// EDGE_XY observed_vertex_id observing_vertex_id x y inf_11 inf_12 inf_22
					// EDGE_POINTXY_PRIOR x y inf_11 inf_12 inf_22
					fprintf(file, "%s %d%s%s %f %f %f %f %f\n",
        					prefix.c_str(),
        					iter->second.from(),
        					to.c_str(),
        					suffix.c_str(),
        					iter->second.transform().x(),
        					iter->second.transform().y(),
        					iter->second.infMatrix().at<double>(0, 0),
        					iter->second.infMatrix().at<double>(0, 1),
        					iter->second.infMatrix().at<double>(1, 1));
				}
			}
			else
			{
				if (isSE3)
				{
					// EDGE_SE3 observed_vertex_id observing_vertex_id x y z qx qy qz qw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
					// EDGE_SE3_PRIOR observed_vertex_id offset_parameter_id x y z qx qy qz qw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
					Eigen::Quaternionf q = iter->second.transform().getQuaternionf();
					fprintf(file, "%s %d%s%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
        					prefix.c_str(),
        					iter->second.from(),
        					to.c_str(),
        					suffix.c_str(),
        					iter->second.transform().x(),
        					iter->second.transform().y(),
        					iter->second.transform().z(),
        					q.x(),
        					q.y(),
        					q.z(),
        					q.w(),
        					iter->second.infMatrix().at<double>(0, 0),
        					iter->second.infMatrix().at<double>(0, 1),
        					iter->second.infMatrix().at<double>(0, 2),
        					iter->second.infMatrix().at<double>(0, 3),
        					iter->second.infMatrix().at<double>(0, 4),
        					iter->second.infMatrix().at<double>(0, 5),
        					iter->second.infMatrix().at<double>(1, 1),
        					iter->second.infMatrix().at<double>(1, 2),
        					iter->second.infMatrix().at<double>(1, 3),
        					iter->second.infMatrix().at<double>(1, 4),
        					iter->second.infMatrix().at<double>(1, 5),
        					iter->second.infMatrix().at<double>(2, 2),
        					iter->second.infMatrix().at<double>(2, 3),
        					iter->second.infMatrix().at<double>(2, 4),
        					iter->second.infMatrix().at<double>(2, 5),
        					iter->second.infMatrix().at<double>(3, 3),
        					iter->second.infMatrix().at<double>(3, 4),
        					iter->second.infMatrix().at<double>(3, 5),
        					iter->second.infMatrix().at<double>(4, 4),
        					iter->second.infMatrix().at<double>(4, 5),
        					iter->second.infMatrix().at<double>(5, 5));
				}
				else
				{
					// EDGE_XYZ observed_vertex_id observing_vertex_id x y z qx qy qz qw inf_11 inf_12 .. inf_13 inf_22 .. inf_33
					// EDGE_POINTXYZ_PRIOR observed_vertex_id x y z inf_11 inf_12 .. inf_13 inf_22 .. inf_33
					fprintf(file, "%s %d%s%s %f %f %f %f %f %f %f %f %f\n",
        					prefix.c_str(),
        					iter->second.from(),
        					to.c_str(),
        					suffix.c_str(),
        					iter->second.transform().x(),
        					iter->second.transform().y(),
        					iter->second.transform().z(),
        					iter->second.infMatrix().at<double>(0, 0),
        					iter->second.infMatrix().at<double>(0, 1),
        					iter->second.infMatrix().at<double>(0, 2),
        					iter->second.infMatrix().at<double>(1, 1),
        					iter->second.infMatrix().at<double>(1, 2),
        					iter->second.infMatrix().at<double>(2, 2));
				}
			}
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
