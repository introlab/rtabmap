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
#include "rtabmap/core/Graph.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/core/Memory.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <set>
#include <queue>
#include "toro3d/treeoptimizer3.hh"
#include "toro3d/treeoptimizer2.hh"

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

#ifdef WITH_GTSAM
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include "vertigo/gtsam/betweenFactorMaxMix.h"
#include "vertigo/gtsam/betweenFactorSwitchable.h"
#include "vertigo/gtsam/switchVariableLinear.h"
#include "vertigo/gtsam/switchVariableSigmoid.h"
#endif // end WITH_GTSAM

#ifdef WITH_CVSBA
#include <cvsba/cvsba.h>
#include "rtabmap/core/util3d_motion_estimation.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_correspondences.h"
#endif

namespace rtabmap {

namespace graph {

////////////////////////////////////////////
// Graph optimizers
////////////////////////////////////////////

Optimizer * Optimizer::create(const ParametersMap & parameters)
{
	int optimizerTypeInt = Parameters::defaultRGBDOptimizeStrategy();
	Parameters::parse(parameters, Parameters::kRGBDOptimizeStrategy(), optimizerTypeInt);
	graph::Optimizer::Type type = (graph::Optimizer::Type)optimizerTypeInt;

	if(!G2OOptimizer::available() && type == Optimizer::kTypeG2O)
	{
		UWARN("g2o optimizer not available. TORO will be used instead.");
		type = Optimizer::kTypeTORO;
	}
	if(!GTSAMOptimizer::available() && type == Optimizer::kTypeGTSAM)
	{
		UWARN("GTSAM optimizer not available. TORO will be used instead.");
		type = Optimizer::kTypeTORO;
	}
	Optimizer * optimizer = 0;
	switch(type)
	{
	case Optimizer::kTypeGTSAM:
		optimizer = new GTSAMOptimizer(parameters);
		break;
	case Optimizer::kTypeG2O:
		optimizer = new G2OOptimizer(parameters);
		break;
	case Optimizer::kTypeTORO:
	default:
		optimizer = new TOROOptimizer(parameters);
		break;

	}
	return optimizer;
}

Optimizer * Optimizer::create(Optimizer::Type & type, const ParametersMap & parameters)
{
	if(!G2OOptimizer::available() && type == Optimizer::kTypeG2O)
	{
		UWARN("g2o optimizer not available. TORO will be used instead.");
		type = Optimizer::kTypeTORO;
	}
	if(!GTSAMOptimizer::available() && type == Optimizer::kTypeGTSAM)
	{
		UWARN("GTSAM optimizer not available. TORO will be used instead.");
		type = Optimizer::kTypeTORO;
	}
	Optimizer * optimizer = 0;
	switch(type)
	{
	case Optimizer::kTypeGTSAM:
		optimizer = new GTSAMOptimizer(parameters);
		break;
	case Optimizer::kTypeG2O:
		optimizer = new G2OOptimizer(parameters);
		break;
	case Optimizer::kTypeTORO:
	default:
		optimizer = new TOROOptimizer(parameters);
		type = Optimizer::kTypeTORO;
		break;

	}
	return optimizer;
}

Optimizer::Optimizer(int iterations, bool slam2d, bool covarianceIgnored, double epsilon, bool robust) :
		iterations_(iterations),
		slam2d_(slam2d),
		covarianceIgnored_(covarianceIgnored),
		epsilon_(epsilon),
		robust_(robust)
{
}

Optimizer::Optimizer(const ParametersMap & parameters) :
		iterations_(Parameters::defaultRGBDOptimizeIterations()),
		slam2d_(Parameters::defaultRGBDOptimizeSlam2D()),
		covarianceIgnored_(Parameters::defaultRGBDOptimizeVarianceIgnored()),
		epsilon_(Parameters::defaultRGBDOptimizeEpsilon()),
		robust_(Parameters::defaultRGBDOptimizeRobust())
{
	parseParameters(parameters);
}

void Optimizer::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kRGBDOptimizeIterations(), iterations_);
	Parameters::parse(parameters, Parameters::kRGBDOptimizeVarianceIgnored(), covarianceIgnored_);
	Parameters::parse(parameters, Parameters::kRGBDOptimizeSlam2D(), slam2d_);
	Parameters::parse(parameters, Parameters::kRGBDOptimizeEpsilon(), epsilon_);
	Parameters::parse(parameters, Parameters::kRGBDOptimizeRobust(), robust_);
}

std::map<int, Transform> Optimizer::optimize(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & constraints,
		std::list<std::map<int, Transform> > * intermediateGraphes)
{
	UERROR("Optimizer %d doesn't implement optimize() method.", (int)this->type());
	return std::map<int, Transform>();
}

std::map<int, Transform> Optimizer::optimizeBA(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, Signature> & signatures)
{
	UERROR("Optimizer %d doesn't implement optimizeBA() method.", (int)this->type());
	return std::map<int, Transform>();
}

void Optimizer::getConnectedGraph(
		int fromId,
		const std::map<int, Transform> & posesIn,
		const std::multimap<int, Link> & linksIn,
		std::map<int, Transform> & posesOut,
		std::multimap<int, Link> & linksOut,
		int depth)
{
	UASSERT(depth >= 0);
	UASSERT(fromId>0);
	UASSERT(uContains(posesIn, fromId));

	posesOut.clear();
	linksOut.clear();

	std::set<int> ids;
	std::set<int> curentDepth;
	std::set<int> nextDepth;
	nextDepth.insert(fromId);
	int d = 0;
	std::multimap<int, int> biLinks;
	for(std::multimap<int, Link>::const_iterator iter=linksIn.begin(); iter!=linksIn.end(); ++iter)
	{
		UASSERT_MSG(findLink(biLinks, iter->second.from(), iter->second.to()) == biLinks.end(), "Input links should be unique between two poses.");
		biLinks.insert(std::make_pair(iter->second.from(), iter->second.to()));
		biLinks.insert(std::make_pair(iter->second.to(), iter->second.from()));
	}

	while((depth == 0 || d < depth) && nextDepth.size())
	{
		curentDepth = nextDepth;
		nextDepth.clear();

		for(std::set<int>::iterator jter = curentDepth.begin(); jter!=curentDepth.end(); ++jter)
		{
			if(ids.find(*jter) == ids.end())
			{
				ids.insert(*jter);
				posesOut.insert(*posesIn.find(*jter));

				for(std::multimap<int, int>::const_iterator iter=biLinks.find(*jter); iter!=biLinks.end() && iter->first==*jter; ++iter)
				{
					int nextId = iter->second;
					if(ids.find(nextId) == ids.end() && uContains(posesIn, nextId))
					{
						nextDepth.insert(nextId);

						std::multimap<int, Link>::const_iterator kter = graph::findLink(linksIn, *jter, nextId);
						if(depth == 0 || d < depth-1)
						{
							linksOut.insert(*kter);
						}
						else if(curentDepth.find(nextId) != curentDepth.end() ||
								ids.find(nextId) != ids.end())
						{
							linksOut.insert(*kter);
						}
					}
				}
			}
		}
		++d;
	}
}

//////////////////
// TORO
//////////////////
std::map<int, Transform> TOROOptimizer::optimize(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		std::list<std::map<int, Transform> > * intermediateGraphes) // contains poses after tree init to last one before the end
{
	std::map<int, Transform> optimizedPoses;
	UDEBUG("Optimizing graph (pose=%d constraints=%d)...", (int)poses.size(), (int)edgeConstraints.size());
	if(edgeConstraints.size()>=1 && poses.size()>=2 && iterations() > 0)
	{
		// Apply TORO optimization
		AISNavigation::TreeOptimizer2 pg2;
		AISNavigation::TreeOptimizer3 pg3;
		pg2.verboseLevel = 0;
		pg3.verboseLevel = 0;

		UDEBUG("fill poses to TORO...");
		if(isSlam2d())
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				UASSERT(!iter->second.isNull());
				AISNavigation::TreePoseGraph2::Pose p(iter->second.x(), iter->second.y(), iter->second.theta());
				AISNavigation::TreePoseGraph2::Vertex* v = pg2.addVertex(iter->first, p);
				UASSERT_MSG(v != 0, uFormat("cannot insert vertex %d!?", iter->first).c_str());
			}
		}
		else
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				UASSERT(!iter->second.isNull());
				float x,y,z, roll,pitch,yaw;
				iter->second.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
				AISNavigation::TreePoseGraph3::Pose p(x, y, z, roll, pitch, yaw);
				AISNavigation::TreePoseGraph3::Vertex* v = pg3.addVertex(iter->first, p);
				UASSERT_MSG(v != 0, uFormat("cannot insert vertex %d!?", iter->first).c_str());
				v->transformation=AISNavigation::TreePoseGraph3::Transformation(p);
			}

		}

		UDEBUG("fill edges to TORO...");
		if(isSlam2d())
		{
			for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
			{
				UASSERT(!iter->second.transform().isNull());
				AISNavigation::TreePoseGraph2::Pose p(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().theta());
				AISNavigation::TreePoseGraph2::InformationMatrix inf;
				//Identity:
				if(isCovarianceIgnored())
				{
					inf.values[0][0] = 1.0; inf.values[0][1] = 0.0; inf.values[0][2] = 0.0; // x
					inf.values[1][0] = 0.0; inf.values[1][1] = 1.0; inf.values[1][2] = 0.0; // y
					inf.values[2][0] = 0.0; inf.values[2][1] = 0.0; inf.values[2][2] = 1.0; // theta/yaw
				}
				else
				{
					inf.values[0][0] = iter->second.infMatrix().at<double>(0,0); // x-x
					inf.values[0][1] = iter->second.infMatrix().at<double>(0,1); // x-y
					inf.values[0][2] = iter->second.infMatrix().at<double>(0,5); // x-theta
					inf.values[1][0] = iter->second.infMatrix().at<double>(1,0); // y-x
					inf.values[1][1] = iter->second.infMatrix().at<double>(1,1); // y-y
					inf.values[1][2] = iter->second.infMatrix().at<double>(1,5); // y-theta
					inf.values[2][0] = iter->second.infMatrix().at<double>(5,0); // theta-x
					inf.values[2][1] = iter->second.infMatrix().at<double>(5,1); // theta-y
					inf.values[2][2] = iter->second.infMatrix().at<double>(5,5); // theta-theta
				}

				int id1 = iter->first;
				int id2 = iter->second.to();
				AISNavigation::TreePoseGraph2::Vertex* v1=pg2.vertex(id1);
				AISNavigation::TreePoseGraph2::Vertex* v2=pg2.vertex(id2);
				UASSERT(v1 != 0);
				UASSERT(v2 != 0);
				AISNavigation::TreePoseGraph2::Transformation t(p);
				if (!pg2.addEdge(v1, v2, t, inf))
				{
					UERROR("Map: Edge already exits between nodes %d and %d, skipping", id1, id2);
				}
			}
		}
		else
		{
			for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
			{
				UASSERT(!iter->second.transform().isNull());
				float x,y,z, roll,pitch,yaw;
				iter->second.transform().getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);

				AISNavigation::TreePoseGraph3::Pose p(x, y, z, roll, pitch, yaw);
				AISNavigation::TreePoseGraph3::InformationMatrix inf = DMatrix<double>::I(6);
				if(!isCovarianceIgnored())
				{
					memcpy(inf[0], iter->second.infMatrix().data, iter->second.infMatrix().total()*sizeof(double));
				}

				int id1 = iter->first;
				int id2 = iter->second.to();
				AISNavigation::TreePoseGraph3::Vertex* v1=pg3.vertex(id1);
				AISNavigation::TreePoseGraph3::Vertex* v2=pg3.vertex(id2);
				UASSERT(v1 != 0);
				UASSERT(v2 != 0);
				AISNavigation::TreePoseGraph3::Transformation t(p);
				if (!pg3.addEdge(v1, v2, t, inf))
				{
					UERROR("Map: Edge already exits between nodes %d and %d, skipping", id1, id2);
				}
			}
		}
		UDEBUG("buildMST...");
		UASSERT(uContains(poses, rootId));
		if(isSlam2d())
		{
			pg2.buildMST(rootId); // pg.buildSimpleTree();
			pg2.initializeOnTree();
			pg2.initializeTreeParameters();
			UDEBUG("Building TORO tree... (if a crash happens just after this msg, "
				   "TORO is not able to find the root of the graph!)");
			pg2.initializeOptimization();
		}
		else
		{
			pg3.buildMST(rootId); // pg.buildSimpleTree();
			pg3.initializeOnTree();
			pg3.initializeTreeParameters();
			UDEBUG("Building TORO tree... (if a crash happens just after this msg, "
				   "TORO is not able to find the root of the graph!)");
			pg3.initializeOptimization();
		}

		UINFO("TORO optimizing begin (iterations=%d)", iterations());
		double lasterror = 0;
		double errorDelta = 0;
		int i=0;
		UTimer timer;
		for (; i<iterations(); i++)
		{
			if(intermediateGraphes && i>0)
			{
				std::map<int, Transform> tmpPoses;
				if(isSlam2d())
				{
					for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
					{
						AISNavigation::TreePoseGraph2::Vertex* v=pg2.vertex(iter->first);
						float roll, pitch, yaw;
						iter->second.getEulerAngles(roll, pitch, yaw);
						Transform newPose(v->pose.x(), v->pose.y(), iter->second.z(), roll, pitch, v->pose.theta());

						UASSERT_MSG(!newPose.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
						tmpPoses.insert(std::pair<int, Transform>(iter->first, newPose));
					}
				}
				else
				{
					for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
					{
						AISNavigation::TreePoseGraph3::Vertex* v=pg3.vertex(iter->first);
						AISNavigation::TreePoseGraph3::Pose pose=v->transformation.toPoseType();
						Transform newPose(pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());

						UASSERT_MSG(!newPose.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
						tmpPoses.insert(std::pair<int, Transform>(iter->first, newPose));
					}
				}
				intermediateGraphes->push_back(tmpPoses);
			}

			double error = 0;
			if(isSlam2d())
			{
				pg2.iterate();

				// compute the error and dump it
				error=pg2.error();
				UDEBUG("iteration %d global error=%f error/constraint=%f", i, error, error/pg2.edges.size());
			}
			else
			{
				pg3.iterate();

				// compute the error and dump it
				double mte, mre, are, ate;
				error=pg3.error(&mre, &mte, &are, &ate);
				UDEBUG("i %d RotGain=%f global error=%f error/constraint=%f",
						i, pg3.getRotGain(), error, error/pg3.edges.size());
			}

			// early stop condition
			errorDelta = lasterror - error;
			if(i>0 && errorDelta < this->epsilon())
			{
				UDEBUG("Stop optimizing, not enough improvement (%f < %f)", errorDelta, this->epsilon());
				break;
			}
			lasterror = error;
		}
		UINFO("TORO optimizing end (%d iterations done, error=%f, time = %f s)", i, errorDelta, timer.ticks());

		if(isSlam2d())
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				AISNavigation::TreePoseGraph2::Vertex* v=pg2.vertex(iter->first);
				float roll, pitch, yaw;
				iter->second.getEulerAngles(roll, pitch, yaw);
				Transform newPose(v->pose.x(), v->pose.y(), iter->second.z(), roll, pitch, v->pose.theta());

				UASSERT_MSG(!newPose.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
				optimizedPoses.insert(std::pair<int, Transform>(iter->first, newPose));
			}
		}
		else
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				AISNavigation::TreePoseGraph3::Vertex* v=pg3.vertex(iter->first);
				AISNavigation::TreePoseGraph3::Pose pose=v->transformation.toPoseType();
				Transform newPose(pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());

				UASSERT_MSG(!newPose.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
				optimizedPoses.insert(std::pair<int, Transform>(iter->first, newPose));
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
	return optimizedPoses;
}

bool TOROOptimizer::saveGraph(
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
		// VERTEX3 id x y z phi theta psi
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			float x,y,z, yaw,pitch,roll;
			pcl::getTranslationAndEulerAngles(iter->second.toEigen3f(), x,y,z, roll, pitch, yaw);
			fprintf(file, "VERTEX3 %d %f %f %f %f %f %f\n",
					iter->first,
					x,
					y,
					z,
					roll,
					pitch,
					yaw);
		}

		//EDGE3 observed_vertex_id observing_vertex_id x y z roll pitch yaw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
		for(std::multimap<int, Link>::const_iterator iter = edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
		{
			float x,y,z, yaw,pitch,roll;
			pcl::getTranslationAndEulerAngles(iter->second.transform().toEigen3f(), x,y,z, roll, pitch, yaw);
			fprintf(file, "EDGE3 %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
					iter->first,
					iter->second.to(),
					x,
					y,
					z,
					roll,
					pitch,
					yaw,
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

bool TOROOptimizer::loadGraph(
		const std::string & fileName,
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & edgeConstraints)
{
	FILE * file = 0;
#ifdef _MSC_VER
	fopen_s(&file, fileName.c_str(), "r");
#else
	file = fopen(fileName.c_str(), "r");
#endif

	if(file)
	{
		char line[400];
		while ( fgets (line , 400 , file) != NULL )
		{
			std::vector<std::string> strList = uListToVector(uSplit(uReplaceChar(line, '\n', ' '), ' '));
			if(strList.size() == 8)
			{
				//VERTEX3
				int id = atoi(strList[1].c_str());
				float x = uStr2Float(strList[2]);
				float y = uStr2Float(strList[3]);
				float z = uStr2Float(strList[4]);
				float roll = uStr2Float(strList[5]);
				float pitch = uStr2Float(strList[6]);
				float yaw = uStr2Float(strList[7]);
				Transform pose = Transform::fromEigen3f(pcl::getTransformation(x, y, z, roll, pitch, yaw));
				if(poses.find(id) == poses.end())
				{
					poses.insert(std::make_pair(id, pose));
				}
				else
				{
					UFATAL("Pose %d already added", id);
				}
			}
			else if(strList.size() == 30)
			{
				//EDGE3
				int idFrom = atoi(strList[1].c_str());
				int idTo = atoi(strList[2].c_str());
				float x = uStr2Float(strList[3]);
				float y = uStr2Float(strList[4]);
				float z = uStr2Float(strList[5]);
				float roll = uStr2Float(strList[6]);
				float pitch = uStr2Float(strList[7]);
				float yaw = uStr2Float(strList[8]);
				float infR = uStr2Float(strList[9]);
				float infP = uStr2Float(strList[15]);
				float infW = uStr2Float(strList[20]);
				UASSERT_MSG(infR > 0 && infP > 0 && infW > 0, uFormat("Information matrix should not be null! line=\"%s\"", line).c_str());
				float rotVariance = infR<=infP && infR<=infW?infR:infP<=infW?infP:infW; // maximum variance
				float infX = uStr2Float(strList[24]);
				float infY = uStr2Float(strList[27]);
				float infZ = uStr2Float(strList[29]);
				UASSERT_MSG(infX > 0 && infY > 0 && infZ > 0, uFormat("Information matrix should not be null! line=\"%s\"", line).c_str());
				float transVariance = 1.0f/(infX<=infY && infX<=infZ?infX:infY<=infW?infY:infZ); // maximum variance
				UINFO("id=%d rotV=%f transV=%f", idFrom, rotVariance, transVariance);
				Transform transform = Transform::fromEigen3f(pcl::getTransformation(x, y, z, roll, pitch, yaw));
				if(poses.find(idFrom) != poses.end() && poses.find(idTo) != poses.end())
				{
					//Link type is unknown
					Link link(idFrom, idTo, Link::kUndef, transform, rotVariance, transVariance);
					edgeConstraints.insert(std::pair<int, Link>(idFrom, link));
				}
				else
				{
					UFATAL("Referred poses from the link not exist!");
				}
			}
			else if(strList.size())
			{
				UFATAL("Error parsing graph file %s on line \"%s\" (strList.size()=%d)", fileName.c_str(), line, (int)strList.size());
			}
		}

		UINFO("Graph loaded from %s", fileName.c_str());
		fclose(file);
	}
	else
	{
		UERROR("Cannot open file %s", fileName.c_str());
		return false;
	}
	return true;
}


//////////////////////
// g2o
//////////////////////
bool G2OOptimizer::available()
{
#ifdef WITH_G2O
	return true;
#else
	return false;
#endif
}

std::map<int, Transform> G2OOptimizer::optimize(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		std::list<std::map<int, Transform> > * intermediateGraphes)
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
			if(this->isRobust() && iter->second.type() != Link::kNeighbor)
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

				if(this->isRobust() && iter->second.type() != Link::kNeighbor)
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

				if(this->isRobust() && iter->second.type() != Link::kNeighbor)
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

		UINFO("g2o optimizing begin (max iterations=%d, robust=%d)", iterations(), isRobust()?1:0);
		int it = 0;
		UTimer timer;
		if(intermediateGraphes)
		{
			for(int i=0; i<iterations(); ++i)
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

				it += optimizer.optimize(1);
				if(ULogger::level() == ULogger::kDebug)
				{
					optimizer.computeActiveErrors();
					UDEBUG("iteration %d: %d nodes, %d edges, chi2: %f", i, (int)optimizer.vertices().size(), (int)optimizer.edges().size(), optimizer.activeRobustChi2());
				}
			}
		}
		else
		{
			it = optimizer.optimize(iterations());
			optimizer.computeActiveErrors();
			UDEBUG("%d nodes, %d edges, chi2: %f", (int)optimizer.vertices().size(), (int)optimizer.edges().size(), optimizer.activeRobustChi2());
		}
		UINFO("g2o optimizing end (%d iterations done, error=%f, time = %f s)", it, optimizer.activeRobustChi2(), timer.ticks());

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

bool G2OOptimizer::saveGraph(
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

			if(useRobustConstraints && iter->second.type() != Link::kNeighbor)
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

//////////////////////
// GTSAM
//////////////////////
bool GTSAMOptimizer::available()
{
#ifdef WITH_GTSAM
	return true;
#else
	return false;
#endif
}

std::map<int, Transform> GTSAMOptimizer::optimize(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		std::list<std::map<int, Transform> > * intermediateGraphes)
{
	std::map<int, Transform> optimizedPoses;
#ifdef WITH_GTSAM
	UDEBUG("Optimizing graph...");
	if(edgeConstraints.size()>=1 && poses.size()>=2 && iterations() > 0)
	{
		gtsam::NonlinearFactorGraph graph;

		//prior first pose
		UASSERT(uContains(poses, rootId));
		const Transform & initialPose = poses.at(rootId);
		if(isSlam2d())
		{
			gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.01, 0.01, 0.01));
			graph.add(gtsam::PriorFactor<gtsam::Pose2>(rootId, gtsam::Pose2(initialPose.x(), initialPose.y(), initialPose.theta()), priorNoise));
		}
		else
		{
			gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
			graph.add(gtsam::PriorFactor<gtsam::Pose3>(rootId, gtsam::Pose3(initialPose.toEigen4d()), priorNoise));
		}

		UDEBUG("fill poses to gtsam...");
		gtsam::Values initialEstimate;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			UASSERT(!iter->second.isNull());
			if(isSlam2d())
			{
				initialEstimate.insert(iter->first, gtsam::Pose2(iter->second.x(), iter->second.y(), iter->second.theta()));
			}
			else
			{
				initialEstimate.insert(iter->first, gtsam::Pose3(iter->second.toEigen4d()));
			}
		}

		UDEBUG("fill edges to gtsam...");
		int switchCounter = poses.rbegin()->first+1;
		for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
		{
			int id1 = iter->first;
			int id2 = iter->second.to();

			UASSERT(!iter->second.transform().isNull());

			if(this->isRobust() && iter->second.type()!=Link::kNeighbor)
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

			if(isSlam2d())
			{
				Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
				if(!isCovarianceIgnored())
				{
					// For some reasons, dividing by 1000 avoids some exceptions (maybe too large numbers on optimization)
					information(0,0) = iter->second.infMatrix().at<double>(0,0)/1000.0; // x-x
					information(0,1) = iter->second.infMatrix().at<double>(0,1)/1000.0; // x-y
					information(0,2) = iter->second.infMatrix().at<double>(0,5)/1000.0; // x-theta
					information(1,0) = iter->second.infMatrix().at<double>(1,0)/1000.0; // y-x
					information(1,1) = iter->second.infMatrix().at<double>(1,1)/1000.0; // y-y
					information(1,2) = iter->second.infMatrix().at<double>(1,5)/1000.0; // y-theta
					information(2,0) = iter->second.infMatrix().at<double>(5,0)/1000.0; // theta-x
					information(2,1) = iter->second.infMatrix().at<double>(5,1)/1000.0; // theta-y
					information(2,2) = iter->second.infMatrix().at<double>(5,5)/1000.0; // theta-theta
				}
				gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Gaussian::Information(information);

				if(this->isRobust() && iter->second.type()!=Link::kNeighbor)
				{
					// create switchable edge factor
					graph.add(vertigo::BetweenFactorSwitchableLinear<gtsam::Pose2>(id1, id2, gtsam::Symbol('s', switchCounter++), gtsam::Pose2(iter->second.transform().x(), iter->second.transform().y(), iter->second.transform().theta()), model));
				}
				else
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
					// For some reasons, dividing by 1000 avoids some exceptions (maybe too large numbers on optimization)
					information = information / 1000.0;
				}

				gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Gaussian::Information(information);

				if(this->isRobust() && iter->second.type()!=Link::kNeighbor)
				{
					// create switchable edge factor
					graph.add(vertigo::BetweenFactorSwitchableLinear<gtsam::Pose3>(id1, id2, gtsam::Symbol('s', switchCounter++), gtsam::Pose3(iter->second.transform().toEigen4d()), model));
				}
				else
				{
					graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id1, id2, gtsam::Pose3(iter->second.transform().toEigen4d()), model));
				}
			}
		}

		UDEBUG("create optimizer");
		gtsam::GaussNewtonParams parameters;
		parameters.relativeErrorTol = epsilon();
		parameters.maxIterations = iterations();
		gtsam::GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
		//gtsam::LevenbergMarquardtParams parametersLev;
		//parametersLev.relativeErrorTol = epsilon();
		//parametersLev.maxIterations = iterations();
		//gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, parametersLev);
		//gtsam::DoglegParams parametersDogleg;
		//parametersDogleg.relativeErrorTol = epsilon();
		//parametersDogleg.maxIterations = iterations();
		//gtsam::DoglegOptimizer optimizer(graph, initialEstimate, parametersDogleg);

		UINFO("GTSAM optimizing begin (max iterations=%d, robust=%d)", iterations(), isRobust()?1:0);
		UTimer timer;
		for(int i=0; i<iterations(); ++i)
		{
			if(intermediateGraphes && i > 0)
			{
				std::map<int, Transform> tmpPoses;
				for(gtsam::Values::const_iterator iter=optimizer.values().begin(); iter!=optimizer.values().end(); ++iter)
				{
					if(iter->value.dim() > 1)
					{
						if(isSlam2d())
						{
							gtsam::Pose2 p = iter->value.cast<gtsam::Pose2>();
							tmpPoses.insert(std::make_pair((int)iter->key, Transform(p.x(), p.y(), p.theta())));
						}
						else
						{
							gtsam::Pose3 p = iter->value.cast<gtsam::Pose3>();
							tmpPoses.insert(std::make_pair((int)iter->key, Transform::fromEigen4d(p.matrix())));
						}
					}
				}
				intermediateGraphes->push_back(tmpPoses);
			}
			try
			{
				optimizer.iterate();
			}
			catch(gtsam::IndeterminantLinearSystemException & e)
			{
				UERROR("GTSAM exception catched: %s", e.what());
				return optimizedPoses;
			}
			UDEBUG("iteration %d error =%f", i+1, optimizer.error());
			if(optimizer.error() < epsilon())
			{
				break;
			}
		}
		UINFO("GTSAM optimizing end (%d iterations done, error=%f (initial=%f final=%f), time=%f s)", optimizer.iterations(), optimizer.error(), graph.error(initialEstimate), graph.error(optimizer.values()), timer.ticks());

		for(gtsam::Values::const_iterator iter=optimizer.values().begin(); iter!=optimizer.values().end(); ++iter)
		{
			if(iter->value.dim() > 1)
			{
				if(isSlam2d())
				{
					gtsam::Pose2 p = iter->value.cast<gtsam::Pose2>();
					optimizedPoses.insert(std::make_pair((int)iter->key, Transform(p.x(), p.y(), p.theta())));
				}
				else
				{
					gtsam::Pose3 p = iter->value.cast<gtsam::Pose3>();
					optimizedPoses.insert(std::make_pair((int)iter->key, Transform::fromEigen4d(p.matrix())));
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
	UERROR("Not built with GTSAM support!");
#endif
	return optimizedPoses;
}

//////////////////////
// cvsba
//////////////////////
bool CVSBAOptimizer::available()
{
#ifdef WITH_CVSBA
	return true;
#else
	return false;
#endif
}

std::map<int, Transform> CVSBAOptimizer::optimizeBA(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, Signature> & signatures)
{
#ifdef WITH_CVSBA
	// run sba optimization
	cvsba::Sba sba;

	// change params if desired
	cvsba::Sba::Params params ;
	params.type = cvsba::Sba::MOTIONSTRUCTURE;
	params.iterations = this->iterations();
	params.minError = this->epsilon();
	params.fixedIntrinsics = 5;
	params.fixedDistortion = 5;
	params.verbose=ULogger::level() <= ULogger::kInfo;
	sba.setParams(params);

	std::map<int, Transform> frames = poses;

	std::vector<cv::Mat> cameraMatrix(frames.size()); //nframes
	std::vector<cv::Mat> R(frames.size()); //nframes
	std::vector<cv::Mat> T(frames.size()); //nframes
	std::vector<cv::Mat> distCoeffs(frames.size()); //nframes
	std::map<int, int> frameIdToIndex;
	std::map<int, CameraModel> models;
	int oi=0;
	for(std::map<int, Transform>::iterator iter=frames.begin(); iter!=frames.end(); )
	{
		CameraModel model;
		if(uContains(signatures, iter->first))
		{
			if(signatures.at(iter->first).sensorData().cameraModels().size() == 1 && signatures.at(iter->first).sensorData().cameraModels().at(0).isValid())
			{
				model = signatures.at(iter->first).sensorData().cameraModels()[0];
			}
			else if(signatures.at(iter->first).sensorData().stereoCameraModel().isValid())
			{
				model = signatures.at(iter->first).sensorData().stereoCameraModel().left();
			}
			else
			{
				UERROR("Missing calibration for node %d", iter->first);
			}
		}
		else
		{
			UERROR("Did not find node %d in cache", iter->first);
		}

		if(model.isValid())
		{
			frameIdToIndex.insert(std::make_pair(iter->first, oi));

			cameraMatrix[oi] = model.K();
			distCoeffs[oi] = model.D();

			Transform t = (iter->second * model.localTransform()).inverse();

			R[oi] = (cv::Mat_<double>(3,3) <<
					(double)t.r11(), (double)t.r12(), (double)t.r13(),
					(double)t.r21(), (double)t.r22(), (double)t.r23(),
					(double)t.r31(), (double)t.r32(), (double)t.r33());
			T[oi] = (cv::Mat_<double>(1,3) << (double)t.x(), (double)t.y(), (double)t.z());
			++oi;

			models.insert(std::make_pair(iter->first, model));

			UDEBUG("Pose %d = %s", iter->first, t.prettyPrint().c_str());

			++iter;
		}
		else
		{
			frames.erase(iter++);
		}
	}
	cameraMatrix.resize(oi);
	R.resize(oi);
	T.resize(oi);
	distCoeffs.resize(oi);

	std::map<int, pcl::PointXYZ> points3DMap;
	std::multimap<int, std::pair<int, cv::Point2f> > wordReferences; // <ID words, IDs frames + keypoint>
	for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
	{
		Link link = iter->second;
		if(link.to() < link.from())
		{
			link = link.inverse();
		}
		if(uContains(signatures, link.from()) &&
		   uContains(signatures, link.to()) &&
		   uContains(frames, link.from()))
		{
			const Signature & sFrom = signatures.at(link.from());
			const Signature & sTo = signatures.at(link.to());

			std::vector<int> inliers;
			Transform t = util3d::estimateMotion3DTo3D(
					uMultimapToMapUnique(sFrom.getWords3()),
					uMultimapToMapUnique(sTo.getWords3()),
					minInliers_,
					inlierDistance_,
					100,
					10,
					0,
					0,
					&inliers);

			if(!t.isNull())
			{
				Transform pose = frames.at(sFrom.id());
				for(unsigned int i=0; i<inliers.size(); ++i)
				{
					pcl::PointXYZ p = util3d::transformPoint(sFrom.getWords3().lower_bound(inliers[i])->second, pose);
					std::map<int, pcl::PointXYZ>::iterator jter = points3DMap.find(inliers[i]);
					if(jter == points3DMap.end())
					{
						points3DMap.insert(std::make_pair(inliers[i], p));
						wordReferences.insert(std::make_pair(inliers[i], std::make_pair(sFrom.id(), sFrom.getWords().lower_bound(inliers[i])->second.pt)));
						wordReferences.insert(std::make_pair(inliers[i], std::make_pair(sTo.id(), sTo.getWords().lower_bound(inliers[i])->second.pt)));
					}
					else
					{
						float dist = uNorm(p.x - jter->second.x, p.y - jter->second.y, p.z - jter->second.z);
						if(dist <= inlierDistance_)
						{
							// in case of loop closure links
							wordReferences.insert(std::make_pair(inliers[i], std::make_pair(sFrom.id(), sFrom.getWords().lower_bound(inliers[i])->second.pt)));
							wordReferences.insert(std::make_pair(inliers[i], std::make_pair(sTo.id(), sTo.getWords().lower_bound(inliers[i])->second.pt)));
						}
					}
				}
			}
			else
			{
				UWARN("Not enough inliers (%d) between %d and %d", inliers.size(), sFrom.id(), sTo.id());
			}
		}
	}

	std::list<int> wordReferencesKeys = uUniqueKeys(wordReferences);
	UDEBUG("points=%d frames=%d", (int)wordReferencesKeys.size(), (int)frames.size());
	std::vector<cv::Point3f> points(wordReferencesKeys.size()); //npoints
	std::vector<std::vector<cv::Point2f> >  imagePoints(frames.size()); //nframes -> npoints
	std::vector<std::vector<int> > visibility(frames.size()); //nframes -> npoints
	for(unsigned int i=0; i<frames.size(); ++i)
	{
		imagePoints[i].resize(wordReferencesKeys.size(), cv::Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
		visibility[i].resize(wordReferencesKeys.size(), 0);
	}
	int i=0;
	for(std::list<int>::iterator iter = wordReferencesKeys.begin(); iter!=wordReferencesKeys.end(); ++iter)
	{
		pcl::PointXYZ & p = points3DMap.at(*iter);
		points[i].x = p.x;
		points[i].y = p.y;
		points[i].z = p.z;

		std::multimap<int, std::pair<int, cv::Point2f> >::iterator jter = wordReferences.lower_bound(*iter);
		while(jter->first == *iter && jter != wordReferences.end())
		{
			imagePoints[frameIdToIndex.at(jter->second.first)][i] = jter->second.second;
			visibility[frameIdToIndex.at(jter->second.first)][i] = 1;
			++jter;
		}

		++i;
	}

	// SBA
	try
	{
		sba.run( points, imagePoints, visibility, cameraMatrix, R, T, distCoeffs);
	}
	catch(cv::Exception & e)
	{
		UERROR("Running SBA... error! %s", e.what());
		return std::map<int, Transform>();
	}

	//update poses
	i=0;
	for(std::map<int, Transform>::iterator iter=frames.begin(); iter!=frames.end(); ++iter)
	{
		Transform t(R[i].at<double>(0,0), R[i].at<double>(0,1), R[i].at<double>(0,2), T[i].at<double>(0),
					R[i].at<double>(1,0), R[i].at<double>(1,1), R[i].at<double>(1,2), T[i].at<double>(1),
					R[i].at<double>(2,0), R[i].at<double>(2,1), R[i].at<double>(2,2), T[i].at<double>(2));

		UDEBUG("New pose %d = %s", iter->first, t.prettyPrint().c_str());

		iter->second = (models.at(iter->first).localTransform() * t).inverse();

		++i;
	}

	return frames;

#else
	UERROR("RTAB-Map is not built with cvsba!");
	return std::map<int, Transform>();
#endif
}

bool exportPoses(
		const std::string & filePath,
		int format, // 0=Raw, 1=RGBD-SLAM, 2=KITTI, 3=TORO, 4=g2o
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & constraints, // required for formats 3 and 4
		const std::map<int, double> & stamps) // required for format 1
{
	std::string tmpPath = filePath;
	if(format==3) // TORO
	{
		if(UFile::getExtension(tmpPath).empty())
		{
			tmpPath+=".graph";
		}
		return graph::TOROOptimizer::saveGraph(tmpPath, poses, constraints);
	}
	else if(format == 4) // g2o
	{
		if(UFile::getExtension(tmpPath).empty())
		{
			tmpPath+=".g2o";
		}
#ifdef WITH_G2O
		return graph::G2OOptimizer::saveGraph(tmpPath, poses, constraints);
#else
		UERROR("Cannot export in g2o format because RTAB-Map is not built with g2o support!");
		return false;
#endif
	}
	else
	{
		if(UFile::getExtension(tmpPath).empty())
		{
			tmpPath+=".txt";
		}

		if(format == 1)
		{
			if(stamps.size() != poses.size())
			{
				UERROR("When exporting poses to format 1 (RGBD-SLAM), stamps and poses maps should have the same size!");
				return false;
			}
		}

		FILE* fout = 0;
#ifdef _MSC_VER
		fopen_s(&fout, tmpPath.c_str(), "w");
#else
		fout = fopen(tmpPath.c_str(), "w");
#endif
		if(fout)
		{
			for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				if(format == 1) // rgbd-slam format
				{
					// Format: stamp x y z qw qx qy qz
					Eigen::Quaternionf q = (*iter).second.getQuaternionf();

					UASSERT(uContains(stamps, iter->first));
					fprintf(fout, "%f %f %f %f %f %f %f %f\n",
							stamps.at(iter->first),
							(*iter).second.x(),
							(*iter).second.y(),
							(*iter).second.z(),
							q.w(),
							q.x(),
							q.y(),
							q.z());
				}
				else // default / KITTI format
				{
					Transform pose = iter->second;
					if(format == 2)
					{
						// for KITTI, we need to remove optical rotation
						// z pointing front, x left, y down
						Transform t( 0, 0, 1, 0,
									-1, 0, 0, 0,
									 0,-1, 0, 0);
						pose = t.inverse() * pose * t;
					}

					// Format: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
					const float * p = (const float *)pose.data();

					fprintf(fout, "%f", p[0]);
					for(int i=1; i<pose.size(); i++)
					{
						fprintf(fout, " %f", p[i]);
					}
					fprintf(fout, "\n");
				}
			}
			fclose(fout);
			return true;
		}
	}
	return false;
}


////////////////////////////////////////////
// Graph utilities
////////////////////////////////////////////
std::multimap<int, Link>::iterator findLink(
		std::multimap<int, Link> & links,
		int from,
		int to)
{
	std::multimap<int, Link>::iterator iter = links.find(from);
	while(iter != links.end() && iter->first == from)
	{
		if(iter->second.to() == to)
		{
			return iter;
		}
		++iter;
	}

	// let's try to -> from
	iter = links.find(to);
	while(iter != links.end() && iter->first == to)
	{
		if(iter->second.to() == from)
		{
			return iter;
		}
		++iter;
	}
	return links.end();
}

std::multimap<int, int>::iterator findLink(
		std::multimap<int, int> & links,
		int from,
		int to)
{
	std::multimap<int, int>::iterator iter = links.find(from);
	while(iter != links.end() && iter->first == from)
	{
		if(iter->second == to)
		{
			return iter;
		}
		++iter;
	}

	// let's try to -> from
	iter = links.find(to);
	while(iter != links.end() && iter->first == to)
	{
		if(iter->second == from)
		{
			return iter;
		}
		++iter;
	}
	return links.end();
}
std::multimap<int, Link>::const_iterator findLink(
		const std::multimap<int, Link> & links,
		int from,
		int to)
{
	std::multimap<int, Link>::const_iterator iter = links.find(from);
	while(iter != links.end() && iter->first == from)
	{
		if(iter->second.to() == to)
		{
			return iter;
		}
		++iter;
	}

	// let's try to -> from
	iter = links.find(to);
	while(iter != links.end() && iter->first == to)
	{
		if(iter->second.to() == from)
		{
			return iter;
		}
		++iter;
	}
	return links.end();
}

std::multimap<int, int>::const_iterator findLink(
		const std::multimap<int, int> & links,
		int from,
		int to)
{
	std::multimap<int, int>::const_iterator iter = links.find(from);
	while(iter != links.end() && iter->first == from)
	{
		if(iter->second == to)
		{
			return iter;
		}
		++iter;
	}

	// let's try to -> from
	iter = links.find(to);
	while(iter != links.end() && iter->first == to)
	{
		if(iter->second == from)
		{
			return iter;
		}
		++iter;
	}
	return links.end();
}

std::map<int, Transform> radiusPosesFiltering(
		const std::map<int, Transform> & poses,
		float radius,
		float angle,
		bool keepLatest)
{
	if(poses.size() > 1 && radius > 0.0f)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(poses.size());
		int i=0;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter, ++i)
		{
			(*cloud)[i] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
			UASSERT_MSG(pcl::isFinite((*cloud)[i]), uFormat("Invalid pose (%d) %s", iter->first, iter->second.prettyPrint().c_str()).c_str());
		}

		// radius filtering
		std::vector<int> names = uKeys(poses);
		std::vector<Transform> transforms = uValues(poses);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> (false));
		tree->setInputCloud(cloud);
		std::set<int> indicesChecked;
		std::set<int> indicesKept;

		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			if(indicesChecked.find(i) == indicesChecked.end())
			{
				std::vector<int> kIndices;
				std::vector<float> kDistances;
				tree->radiusSearch(cloud->at(i), radius, kIndices, kDistances);

				std::set<int> cloudIndices;
				const Transform & currentT = transforms.at(i);
				Eigen::Vector3f vA = currentT.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
				for(unsigned int j=0; j<kIndices.size(); ++j)
				{
					if(indicesChecked.find(kIndices[j]) == indicesChecked.end())
					{
						if(angle > 0.0f)
						{
							const Transform & checkT = transforms.at(kIndices[j]);
							// same orientation?
							Eigen::Vector3f vB = checkT.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
							double a = pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
							if(a <= angle)
							{
								cloudIndices.insert(kIndices[j]);
							}
						}
						else
						{
							cloudIndices.insert(kIndices[j]);
						}
					}
				}

				if(keepLatest)
				{
					bool lastAdded = false;
					for(std::set<int>::reverse_iterator iter = cloudIndices.rbegin(); iter!=cloudIndices.rend(); ++iter)
					{
						if(!lastAdded)
						{
							indicesKept.insert(*iter);
							lastAdded = true;
						}
						indicesChecked.insert(*iter);
					}
				}
				else
				{
					bool firstAdded = false;
					for(std::set<int>::iterator iter = cloudIndices.begin(); iter!=cloudIndices.end(); ++iter)
					{
						if(!firstAdded)
						{
							indicesKept.insert(*iter);
							firstAdded = true;
						}
						indicesChecked.insert(*iter);
					}
				}
			}
		}

		//pcl::IndicesPtr indicesOut(new std::vector<int>);
		//indicesOut->insert(indicesOut->end(), indicesKept.begin(), indicesKept.end());
		UINFO("Cloud filtered In = %d, Out = %d", cloud->size(), indicesKept.size());
		//pcl::io::savePCDFile("duplicateIn.pcd", *cloud);
		//pcl::io::savePCDFile("duplicateOut.pcd", *cloud, *indicesOut);

		std::map<int, Transform> keptPoses;
		for(std::set<int>::iterator iter = indicesKept.begin(); iter!=indicesKept.end(); ++iter)
		{
			keptPoses.insert(std::make_pair(names.at(*iter), transforms.at(*iter)));
		}

		return keptPoses;
	}
	else
	{
		return poses;
	}
}

std::multimap<int, int> radiusPosesClustering(const std::map<int, Transform> & poses, float radius, float angle)
{
	std::multimap<int, int> clusters;
	if(poses.size() > 1 && radius > 0.0f)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(poses.size());
		int i=0;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter, ++i)
		{
			(*cloud)[i] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
			UASSERT_MSG(pcl::isFinite((*cloud)[i]), uFormat("Invalid pose (%d) %s", iter->first, iter->second.prettyPrint().c_str()).c_str());
		}

		// radius clustering (nearest neighbors)
		std::vector<int> ids = uKeys(poses);
		std::vector<Transform> transforms = uValues(poses);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> (false));
		tree->setInputCloud(cloud);

		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			std::vector<int> kIndices;
			std::vector<float> kDistances;
			tree->radiusSearch(cloud->at(i), radius, kIndices, kDistances);

			std::set<int> cloudIndices;
			const Transform & currentT = transforms.at(i);
			Eigen::Vector3f vA = currentT.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
			for(unsigned int j=0; j<kIndices.size(); ++j)
			{
				if((int)i != kIndices[j])
				{
					if(angle > 0.0f)
					{
						const Transform & checkT = transforms.at(kIndices[j]);
						// same orientation?
						Eigen::Vector3f vB = checkT.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
						double a = pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
						if(a <= angle)
						{
							clusters.insert(std::make_pair(ids[i], ids[kIndices[j]]));
						}
					}
					else
					{
						clusters.insert(std::make_pair(ids[i], ids[kIndices[j]]));
					}
				}
			}
		}
	}
	return clusters;
}


class Node
{
public:
	Node(int id, int fromId, const rtabmap::Transform & pose) :
		id_(id),
		costSoFar_(0.0f),
		distToEnd_(0.0f),
		fromId_(fromId),
		closed_(false),
		pose_(pose)
	{}

	int id() const {return id_;}
	int fromId() const {return fromId_;}
	bool isClosed() const {return closed_;}
	bool isOpened() const {return !closed_;}
	float costSoFar() const {return costSoFar_;} // Dijkstra cost
	float distToEnd() const {return distToEnd_;} // Breath-first cost
	float totalCost() const {return costSoFar_ + distToEnd_;} // A* cost
	rtabmap::Transform pose() const {return pose_;}
	float distFrom(const rtabmap::Transform & pose) const
	{
		return pose_.getDistance(pose); // use sqrt distance
	}

	void setClosed(bool closed) {closed_ = closed;}
	void setFromId(int fromId) {fromId_ = fromId;}
	void setCostSoFar(float costSoFar) {costSoFar_ = costSoFar;}
	void setDistToEnd(float distToEnd) {distToEnd_ = distToEnd;}
	void setPose(const Transform & pose) {pose_ = pose;}

private:
	int id_;
	float costSoFar_;
	float distToEnd_;
	int fromId_;
	bool closed_;
	rtabmap::Transform pose_;
};

typedef std::pair<int, float> Pair; // first is id, second is cost
struct Order
{
    bool operator()(Pair const& a, Pair const& b) const
    {
        return a.second > b.second;
    }
};

std::list<std::pair<int, Transform> > computePath(
			const std::map<int, rtabmap::Transform> & poses,
			const std::multimap<int, int> & links,
			int from,
			int to,
			bool updateNewCosts)
{
	std::list<std::pair<int, Transform> > path;

	//A*
	int startNode = from;
	int endNode = to;
	rtabmap::Transform endPose = poses.at(endNode);
	std::map<int, Node> nodes;
	nodes.insert(std::make_pair(startNode, Node(startNode, 0, poses.at(startNode))));
	std::priority_queue<Pair, std::vector<Pair>, Order> pq;
	std::multimap<float, int> pqmap;
	if(updateNewCosts)
	{
		pqmap.insert(std::make_pair(0, startNode));
	}
	else
	{
		pq.push(Pair(startNode, 0));
	}

	while((updateNewCosts && pqmap.size()) || (!updateNewCosts && pq.size()))
	{
		Node * currentNode;
		if(updateNewCosts)
		{
			currentNode = &nodes.find(pqmap.begin()->second)->second;
			pqmap.erase(pqmap.begin());
		}
		else
		{
			currentNode = &nodes.find(pq.top().first)->second;
			pq.pop();
		}

		currentNode->setClosed(true);

		if(currentNode->id() == endNode)
		{
			while(currentNode->id()!=startNode)
			{
				path.push_front(std::make_pair(currentNode->id(), currentNode->pose()));
				currentNode = &nodes.find(currentNode->fromId())->second;
			}
			path.push_front(std::make_pair(startNode, poses.at(startNode)));
			break;
		}

		// lookup neighbors
		for(std::multimap<int, int>::const_iterator iter = links.find(currentNode->id());
			iter!=links.end() && iter->first == currentNode->id();
			++iter)
		{
			std::map<int, Node>::iterator nodeIter = nodes.find(iter->second);
			if(nodeIter == nodes.end())
			{
				std::map<int, rtabmap::Transform>::const_iterator poseIter = poses.find(iter->second);
				if(poseIter == poses.end())
				{
					UERROR("Next pose %d (from %d) should be found in poses! Ignoring it!", iter->second, iter->first);
				}
				else
				{
					Node n(iter->second, currentNode->id(), poseIter->second);
					n.setCostSoFar(currentNode->costSoFar() + currentNode->distFrom(poseIter->second));
					n.setDistToEnd(n.distFrom(endPose));
					nodes.insert(std::make_pair(iter->second, n));
					if(updateNewCosts)
					{
						pqmap.insert(std::make_pair(n.totalCost(), n.id()));
					}
					else
					{
						pq.push(Pair(n.id(), n.totalCost()));
					}
				}
			}
			else if(updateNewCosts && nodeIter->second.isOpened())
			{
				float newCostSoFar = currentNode->costSoFar() + currentNode->distFrom(nodeIter->second.pose());
				if(nodeIter->second.costSoFar() > newCostSoFar)
				{
					// update the cost in the priority queue
					for(std::multimap<float, int>::iterator mapIter=pqmap.begin(); mapIter!=pqmap.end(); ++mapIter)
					{
						if(mapIter->second == nodeIter->first)
						{
							pqmap.erase(mapIter);
							nodeIter->second.setCostSoFar(newCostSoFar);
							pqmap.insert(std::make_pair(nodeIter->second.totalCost(), nodeIter->first));
							break;
						}
					}
				}
			}
		}
	}
	return path;
}


// return path starting from "fromId" (Identity pose for the first node)
std::list<std::pair<int, Transform> > computePath(
		int fromId,
		int toId,
		const Memory * memory,
		bool lookInDatabase,
		bool updateNewCosts,
		float linearVelocity,  // m/sec
		float angularVelocity) // rad/sec
{
	UASSERT(memory!=0);
	UASSERT(fromId>=0);
	UASSERT(toId>=0);
	std::list<std::pair<int, Transform> > path;
	UDEBUG("fromId=%d, toId=%d, lookInDatabase=%d, updateNewCosts=%d, linearVelocity=%f, angularVelocity=%f",
			fromId,
			toId,
			lookInDatabase?1:0,
			updateNewCosts?1:0,
			linearVelocity,
			angularVelocity);

	std::multimap<int, Link> allLinks;
	if(lookInDatabase)
	{
		// Faster to load all links in one query
		UTimer t;
		allLinks = memory->getAllLinks(lookInDatabase);
		UINFO("getting all %d links time = %f s", (int)allLinks.size(), t.ticks());
	}

	//dijkstra
	int startNode = fromId;
	int endNode = toId;
	std::map<int, Node> nodes;
	nodes.insert(std::make_pair(startNode, Node(startNode, 0, Transform::getIdentity())));
	std::priority_queue<Pair, std::vector<Pair>, Order> pq;
	std::multimap<float, int> pqmap;
	if(updateNewCosts)
	{
		pqmap.insert(std::make_pair(0, startNode));
	}
	else
	{
		pq.push(Pair(startNode, 0));
	}

	while((updateNewCosts && pqmap.size()) || (!updateNewCosts && pq.size()))
	{
		Node * currentNode;
		if(updateNewCosts)
		{
			currentNode = &nodes.find(pqmap.begin()->second)->second;
			pqmap.erase(pqmap.begin());
		}
		else
		{
			currentNode = &nodes.find(pq.top().first)->second;
			pq.pop();
		}

		currentNode->setClosed(true);

		if(currentNode->id() == endNode)
		{
			while(currentNode->id()!=startNode)
			{
				path.push_front(std::make_pair(currentNode->id(), currentNode->pose()));
				currentNode = &nodes.find(currentNode->fromId())->second;
			}
			path.push_front(std::make_pair(startNode, currentNode->pose()));
			break;
		}

		// lookup neighbors
		std::map<int, Link> links;
		if(allLinks.size() == 0)
		{
			links = memory->getLinks(currentNode->id(), lookInDatabase);
		}
		else
		{
			for(std::multimap<int, Link>::const_iterator iter = allLinks.lower_bound(currentNode->id());
				iter!=allLinks.end() && iter->first == currentNode->id();
				++iter)
			{
				links.insert(std::make_pair(iter->second.to(), iter->second));
			}
		}
		for(std::map<int, Link>::const_iterator iter = links.begin(); iter!=links.end(); ++iter)
		{
			Transform nextPose = currentNode->pose()*iter->second.transform();
			float cost = 0.0f;
			if(linearVelocity <= 0.0f && angularVelocity <= 0.0f)
			{
				// use distance only
				cost = iter->second.transform().getNorm();
			}
			else // use time
			{
				if(linearVelocity > 0.0f)
				{
					cost += iter->second.transform().getNorm()/linearVelocity;
				}
				if(angularVelocity > 0.0f)
				{
					Eigen::Vector4f v1 = Eigen::Vector4f(nextPose.x()-currentNode->pose().x(), nextPose.y()-currentNode->pose().y(), nextPose.z()-currentNode->pose().z(), 1.0f);
					Eigen::Vector4f v2 = nextPose.rotation().toEigen4f()*Eigen::Vector4f(1,0,0,1);
					float angle = pcl::getAngle3D(v1, v2);
					cost += angle / angularVelocity;
				}
			}

			std::map<int, Node>::iterator nodeIter = nodes.find(iter->first);
			if(nodeIter == nodes.end())
			{
				Node n(iter->second.to(), currentNode->id(), nextPose);

				n.setCostSoFar(currentNode->costSoFar() + cost);
				nodes.insert(std::make_pair(iter->second.to(), n));
				if(updateNewCosts)
				{
					pqmap.insert(std::make_pair(n.totalCost(), n.id()));
				}
				else
				{
					pq.push(Pair(n.id(), n.totalCost()));
				}
			}
			else if(updateNewCosts && nodeIter->second.isOpened())
			{
				float newCostSoFar = currentNode->costSoFar() + cost;
				if(nodeIter->second.costSoFar() > newCostSoFar)
				{
					// update pose with new link
					nodeIter->second.setPose(nextPose);

					// update the cost in the priority queue
					for(std::multimap<float, int>::iterator mapIter=pqmap.begin(); mapIter!=pqmap.end(); ++mapIter)
					{
						if(mapIter->second == nodeIter->first)
						{
							pqmap.erase(mapIter);
							nodeIter->second.setCostSoFar(newCostSoFar);
							pqmap.insert(std::make_pair(nodeIter->second.totalCost(), nodeIter->first));
							break;
						}
					}
				}
			}
		}
	}

	// Debugging stuff
	if(ULogger::level() == ULogger::kDebug)
	{
		std::stringstream stream;
		std::vector<int> linkTypes(Link::kUndef, 0);
		std::list<std::pair<int, Transform> >::const_iterator previousIter = path.end();
		float length = 0.0f;
		for(std::list<std::pair<int, Transform> >::const_iterator iter=path.begin(); iter!=path.end();++iter)
		{
			if(iter!=path.begin())
			{
				stream << ",";
			}

			if(previousIter!=path.end())
			{
				//UDEBUG("current  %d = %s", iter->first, iter->second.prettyPrint().c_str());
				if(allLinks.size())
				{
					std::multimap<int, Link>::iterator jter = graph::findLink(allLinks, previousIter->first, iter->first);
					if(jter != allLinks.end())
					{
						//Transform nextPose = iter->second;
						//Eigen::Vector4f v1 = Eigen::Vector4f(nextPose.x()-previousIter->second.x(), nextPose.y()-previousIter->second.y(), nextPose.z()-previousIter->second.z(), 1.0f);
						//Eigen::Vector4f v2 = nextPose.rotation().toEigen4f()*Eigen::Vector4f(1,0,0,1);
						//float angle = pcl::getAngle3D(v1, v2);
						//float cost = angle ;
						//UDEBUG("v1=%f,%f,%f v2=%f,%f,%f a=%f", v1[0], v1[1], v1[2], v2[0], v2[1], v2[2], cost);

						UASSERT(jter->second.type() >= Link::kNeighbor && jter->second.type()<Link::kUndef);
						++linkTypes[jter->second.type()];
						stream << "[" << jter->second.type() << "]";
						length += jter->second.transform().getNorm();
					}
				}
			}

			stream << iter->first;

			previousIter=iter;
		}
		UDEBUG("Path (%f m) = [%s]", length, stream.str().c_str());
		std::stringstream streamB;
		for(unsigned int i=0; i<linkTypes.size(); ++i)
		{
			if(i > 0)
			{
				streamB << " ";
			}
			streamB << i << "=" << linkTypes[i];
		}
		UDEBUG("Link types = %s", streamB.str().c_str());
	}

	return path;
}

int findNearestNode(
		const std::map<int, rtabmap::Transform> & nodes,
		const rtabmap::Transform & targetPose)
{
	int id = 0;
	if(nodes.size() && !targetPose.isNull())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(nodes.size());
		std::vector<int> ids(nodes.size());
		int oi = 0;
		for(std::map<int, Transform>::const_iterator iter = nodes.begin(); iter!=nodes.end(); ++iter)
		{
			(*cloud)[oi] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
			ids[oi++] = iter->first;
		}

		std::map<int, float> foundNodes;
		if(cloud->size())
		{
			pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
			kdTree->setInputCloud(cloud);
			std::vector<int> ind;
			std::vector<float> dist;
			pcl::PointXYZ pt(targetPose.x(), targetPose.y(), targetPose.z());
			kdTree->nearestKSearch(pt, 1, ind, dist);
			if(ind.size() && dist.size() && ind[0] >= 0)
			{
				UDEBUG("Nearest node = %d: %f", ids[ind[0]], dist[0]);
				id = ids[ind[0]];
			}
		}
	}
	return id;
}

// return <id, sqrd distance>, excluding query
std::map<int, float> getNodesInRadius(
		int nodeId,
		const std::map<int, Transform> & nodes,
		float radius)
{
	UASSERT(uContains(nodes, nodeId));
	std::map<int, float> foundNodes;
	if(nodes.size() <= 1)
	{
		return foundNodes;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(nodes.size());
	std::vector<int> ids(nodes.size());
	int oi = 0;
	for(std::map<int, Transform>::const_iterator iter = nodes.begin(); iter!=nodes.end(); ++iter)
	{
		if(iter->first != nodeId)
		{
			(*cloud)[oi] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
			UASSERT_MSG(pcl::isFinite((*cloud)[oi]), uFormat("Invalid pose (%d) %s", iter->first, iter->second.prettyPrint().c_str()).c_str());
			ids[oi] = iter->first;
			++oi;
		}
	}
	cloud->resize(oi);
	ids.resize(oi);

	Transform fromT = nodes.at(nodeId);

	if(cloud->size())
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
		kdTree->setInputCloud(cloud);
		std::vector<int> ind;
		std::vector<float> dist;
		pcl::PointXYZ pt(fromT.x(), fromT.y(), fromT.z());
		kdTree->radiusSearch(pt, radius, ind, dist, 0);
		for(unsigned int i=0; i<ind.size(); ++i)
		{
			if(ind[i] >=0)
			{
				UDEBUG("Inlier %d: %f", ids[ind[i]], sqrt(dist[i]));
				foundNodes.insert(std::make_pair(ids[ind[i]], dist[i]));
			}
		}
	}
	UDEBUG("found nodes=%d", (int)foundNodes.size());
	return foundNodes;
}

// return <id, Transform>, excluding query
std::map<int, Transform> getPosesInRadius(
		int nodeId,
		const std::map<int, Transform> & nodes,
		float radius)
{
	UASSERT(uContains(nodes, nodeId));
	std::map<int, Transform> foundNodes;
	if(nodes.size() <= 1)
	{
		return foundNodes;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(nodes.size());
	std::vector<int> ids(nodes.size());
	int oi = 0;
	for(std::map<int, Transform>::const_iterator iter = nodes.begin(); iter!=nodes.end(); ++iter)
	{
		if(iter->first != nodeId)
		{
			(*cloud)[oi] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
			UASSERT_MSG(pcl::isFinite((*cloud)[oi]), uFormat("Invalid pose (%d) %s", iter->first, iter->second.prettyPrint().c_str()).c_str());
			ids[oi] = iter->first;
			++oi;
		}
	}
	cloud->resize(oi);
	ids.resize(oi);

	Transform fromT = nodes.at(nodeId);

	if(cloud->size())
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
		kdTree->setInputCloud(cloud);
		std::vector<int> ind;
		std::vector<float> dist;
		pcl::PointXYZ pt(fromT.x(), fromT.y(), fromT.z());
		kdTree->radiusSearch(pt, radius, ind, dist, 0);
		for(unsigned int i=0; i<ind.size(); ++i)
		{
			if(ind[i] >=0)
			{
				UDEBUG("Inlier %d: %f", ids[ind[i]], sqrt(dist[i]));
				foundNodes.insert(std::make_pair(ids[ind[i]], nodes.at(ids[ind[i]])));
			}
		}
	}
	UDEBUG("found nodes=%d", (int)foundNodes.size());
	return foundNodes;
}

float computePathLength(
		const std::vector<std::pair<int, Transform> > & path,
		unsigned int fromIndex,
		unsigned int toIndex)
{
	float length = 0.0f;
	if(path.size() > 1)
	{
		UASSERT(fromIndex  < path.size() && toIndex < path.size() && fromIndex <= toIndex);
		if(fromIndex >= toIndex)
		{
			toIndex = (unsigned int)path.size()-1;
		}
		float x=0, y=0, z=0;
		for(unsigned int i=fromIndex; i<toIndex-1; ++i)
		{
			x += fabs(path[i].second.x() - path[i+1].second.x());
			y += fabs(path[i].second.y() - path[i+1].second.y());
			z += fabs(path[i].second.z() - path[i+1].second.z());
		}
		length = sqrt(x*x + y*y + z*z);
	}
	return length;
}

// return all paths linked only by neighbor links
std::list<std::map<int, Transform> > getPaths(
		std::map<int, Transform> poses,
		const std::multimap<int, Link> & links)
{
	std::list<std::map<int, Transform> > paths;
	if(poses.size() && links.size())
	{
		// Segment poses connected only by neighbor links
		while(poses.size())
		{
			std::map<int, Transform> path;
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end();)
			{
				std::multimap<int, Link>::const_iterator jter = findLink(links, path.rbegin()->first, iter->first);
				if(path.size() == 0 || (jter != links.end() && jter->second.type() == Link::kNeighbor))
				{
					path.insert(*iter);
					poses.erase(iter++);
				}
				else
				{
					break;
				}
			}
			UASSERT(path.size());
			paths.push_back(path);
		}

	}
	return paths;
}

} /* namespace graph */

} /* namespace rtabmap */
