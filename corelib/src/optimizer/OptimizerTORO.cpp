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

#include <rtabmap/core/optimizer/OptimizerTORO.h>

#ifdef RTABMAP_TORO
#include "toro3d/treeoptimizer3.h"
#include "toro3d/treeoptimizer2.h"
#endif

namespace rtabmap {

bool OptimizerTORO::available()
{
#ifdef RTABMAP_TORO
	return true;
#else
	return false;
#endif
}

std::map<int, Transform> OptimizerTORO::optimize(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		cv::Mat & outputCovariance,
		std::list<std::map<int, Transform> > * intermediateGraphes, // contains poses after tree init to last one before the end
		double * finalError,
		int * iterationsDone)
{
	outputCovariance = cv::Mat::eye(6,6,CV_64FC1);
	std::map<int, Transform> optimizedPoses;
#ifdef RTABMAP_TORO
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
				if(iter->first > 0)
				{
					UASSERT(!iter->second.isNull());
					AISNavigation::TreePoseGraph2::Pose p(iter->second.x(), iter->second.y(), iter->second.theta());
					AISNavigation::TreePoseGraph2::Vertex* v = pg2.addVertex(iter->first, p);
					UASSERT_MSG(v != 0, uFormat("cannot insert vertex %d!?", iter->first).c_str());
				}
			}
		}
		else
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				if(iter->first > 0)
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

				int id1 = iter->second.from();
				int id2 = iter->second.to();
				if(id1 != id2 && id1 > 0 && id2 > 0)
				{
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
				else if(id1 == id2)
				{
					UWARN("TORO optimizer doesn't support prior or gravity links, use GTSAM or g2o optimizers (see parameter %s). Link %d ignored...", Parameters::kOptimizerStrategy().c_str(), id1);
				}
				else if(id1 < 0 || id2 < 0)
				{
					UWARN("TORO optimizer doesn't support landmark links, use GTSAM or g2o optimizers (see parameter %s). Link %d->%d ignored...", Parameters::kOptimizerStrategy().c_str(), id1, id2);
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

				int id1 = iter->second.from();
				int id2 = iter->second.to();
				if(id1 != id2 && id1 > 0 && id2 > 0)
				{
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
				else if(id1 == id2)
				{
					UWARN("TORO optimizer doesn't support prior or gravity links, use GTSAM or g2o optimizers (see parameter %s). Link %d ignored...", Parameters::kOptimizerStrategy().c_str(), id1);
				}
				else if(id1 < 0 || id2 < 0)
				{
					UWARN("TORO optimizer doesn't support landmark links, use GTSAM or g2o optimizers (see parameter %s). Link %d->%d ignored...", Parameters::kOptimizerStrategy().c_str(), id1, id2);
				}
			}
		}
		UDEBUG("buildMST... root=%d", rootId);
		UASSERT(uContains(poses, rootId));
		if(isSlam2d())
		{
			pg2.buildMST(rootId); // pg.buildSimpleTree();
			//UDEBUG("initializeOnTree()");
			//pg2.initializeOnTree();
			UDEBUG("initializeTreeParameters()");
			pg2.initializeTreeParameters();
			UDEBUG("Building TORO tree... (if a crash happens just after this msg, "
				   "TORO is not able to find the root of the graph!)");
			pg2.initializeOptimization();
		}
		else
		{
			pg3.buildMST(rootId); // pg.buildSimpleTree();
			//UDEBUG("initializeOnTree()");
			//pg3.initializeOnTree();
			UDEBUG("initializeTreeParameters()");
			pg3.initializeTreeParameters();
			UDEBUG("Building TORO tree... (if a crash happens just after this msg, "
				   "TORO is not able to find the root of the graph!)");
			pg3.initializeOptimization();
		}

		UINFO("Initial error = %f", pg2.error());
		UINFO("TORO optimizing begin (iterations=%d)", iterations());
		double lastError = 0;
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
						if(iter->first > 0)
						{
							AISNavigation::TreePoseGraph2::Vertex* v=pg2.vertex(iter->first);
							float roll, pitch, yaw;
							iter->second.getEulerAngles(roll, pitch, yaw);
							Transform newPose(v->pose.x(), v->pose.y(), iter->second.z(), roll, pitch, v->pose.theta());

							UASSERT_MSG(!newPose.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
							tmpPoses.insert(std::pair<int, Transform>(iter->first, newPose));
						}
					}
				}
				else
				{
					for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
					{
						if(iter->first > 0)
						{
							AISNavigation::TreePoseGraph3::Vertex* v=pg3.vertex(iter->first);
							AISNavigation::TreePoseGraph3::Pose pose=v->transformation.toPoseType();
							Transform newPose(pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());

							UASSERT_MSG(!newPose.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
							tmpPoses.insert(std::pair<int, Transform>(iter->first, newPose));
						}
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
			double errorDelta = lastError - error;
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
			else if(i==0 && error < this->epsilon())
			{
				UINFO("Stop optimizing, error is already under epsilon (%f < %f)", error, this->epsilon());
				break;
			}
			lastError = error;
		}
		if(finalError)
		{
			*finalError = lastError;
		}
		if(iterationsDone)
		{
			*iterationsDone = i;
		}
		UINFO("TORO optimizing end (%d iterations done, error=%f, time = %f s)", i, lastError, timer.ticks());

		if(isSlam2d())
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				if(iter->first > 0)
				{
					AISNavigation::TreePoseGraph2::Vertex* v=pg2.vertex(iter->first);
					float roll, pitch, yaw;
					iter->second.getEulerAngles(roll, pitch, yaw);
					Transform newPose(v->pose.x(), v->pose.y(), iter->second.z(), roll, pitch, v->pose.theta());

					UASSERT_MSG(!newPose.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
					optimizedPoses.insert(std::pair<int, Transform>(iter->first, newPose));
				}
			}
		}
		else
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				if(iter->first > 0)
				{
					AISNavigation::TreePoseGraph3::Vertex* v=pg3.vertex(iter->first);
					AISNavigation::TreePoseGraph3::Pose pose=v->transformation.toPoseType();
					Transform newPose(pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw());

					UASSERT_MSG(!newPose.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
					optimizedPoses.insert(std::pair<int, Transform>(iter->first, newPose));
				}
			}
		}

		// TORO doesn't compute marginals...
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
	UERROR("Not built with TORO support!");
#endif
	return optimizedPoses;
}

bool OptimizerTORO::saveGraph(
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
		
		for (std::map<int, Transform>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter)
		{
			if (isSlam2d())
			{
				// VERTEX2 id x y theta
				fprintf(file, "VERTEX2 %d %f %f %f\n",
					iter->first,
					iter->second.x(),
					iter->second.y(),
					iter->second.theta());
			}
			else
			{
				// VERTEX3 id x y z phi theta psi
				float x, y, z, yaw, pitch, roll;
				iter->second.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
				fprintf(file, "VERTEX3 %d %f %f %f %f %f %f\n",
					iter->first,
					x,
					y,
					z,
					roll,
					pitch,
					yaw);
			}
		}

		for(std::multimap<int, Link>::const_iterator iter = edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
		{
			if (iter->second.type() != Link::kPosePrior && iter->second.type() != Link::kGravity)
			{
				if (isSlam2d())
				{
					//EDGE2 observed_vertex_id observing_vertex_id x y theta inf_11 inf_12 inf_13 inf_22 inf_23 inf_33
					fprintf(file, "EDGE2 %d %d %f %f %f %f %f %f %f %f %f\n",
						iter->second.from(),
						iter->second.to(),
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
					//EDGE3 observed_vertex_id observing_vertex_id x y z roll pitch yaw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
					float x, y, z, yaw, pitch, roll;
					iter->second.transform().getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
					fprintf(file, "EDGE3 %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
						iter->second.from(),
						iter->second.to(),
						x,
						y,
						z,
						roll,
						pitch,
						yaw,
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

bool OptimizerTORO::loadGraph(
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
				Transform pose(x, y, z, roll, pitch, yaw);
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
				cv::Mat informationMatrix(6,6,CV_64FC1);
				informationMatrix.at<double>(3,3) = uStr2Float(strList[9]);
				informationMatrix.at<double>(4,4) = uStr2Float(strList[15]);
				informationMatrix.at<double>(5,5) = uStr2Float(strList[20]);
				UASSERT_MSG(informationMatrix.at<double>(3,3) > 0.0 && informationMatrix.at<double>(4,4) > 0.0 && informationMatrix.at<double>(5,5) > 0.0, uFormat("Information matrix should not be null! line=\"%s\"", line).c_str());
				informationMatrix.at<double>(0,0) = uStr2Float(strList[24]);
				informationMatrix.at<double>(1,1) = uStr2Float(strList[27]);
				informationMatrix.at<double>(2,2) = uStr2Float(strList[29]);
				UASSERT_MSG(informationMatrix.at<double>(0,0) > 0.0 && informationMatrix.at<double>(1,1) > 0.0 && informationMatrix.at<double>(2,2) > 0.0, uFormat("Information matrix should not be null! line=\"%s\"", line).c_str());
				Transform transform(x, y, z, roll, pitch, yaw);
				if(poses.find(idFrom) != poses.end() && poses.find(idTo) != poses.end())
				{
					//Link type is unknown
					Link link(idFrom, idTo, Link::kUndef, transform, informationMatrix);
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

} /* namespace rtabmap */
