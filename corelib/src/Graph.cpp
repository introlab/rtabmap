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
#include <pcl/search/kdtree.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <set>
#include <queue>
#include "toro3d/treeoptimizer3.hh"

namespace rtabmap {

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


// <int, depth> margin=0 means infinite margin
std::map<int, int> generateDepthGraph(
		const std::multimap<int, Link> & links,
		int fromId,
		int depth)
{
	UASSERT(depth >= 0);
	//UDEBUG("signatureId=%d, neighborsMargin=%d", signatureId, margin);
	std::map<int, int> ids;
	if(fromId<=0)
	{
		return ids;
	}

	std::list<int> curentDepthList;
	std::set<int> nextDepth;
	nextDepth.insert(fromId);
	int d = 0;
	while((depth == 0 || d < depth) && nextDepth.size())
	{
		curentDepthList = std::list<int>(nextDepth.begin(), nextDepth.end());
		nextDepth.clear();

		for(std::list<int>::iterator jter = curentDepthList.begin(); jter!=curentDepthList.end(); ++jter)
		{
			if(ids.find(*jter) == ids.end())
			{
				std::set<int> marginIds;

				ids.insert(std::pair<int, int>(*jter, d));

				for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
				{
					if(iter->second.from() == *jter)
					{
						marginIds.insert(iter->second.to());
					}
					else if(iter->second.to() == *jter)
					{
						marginIds.insert(iter->second.from());
					}
				}

				// Margin links
				for(std::set<int>::const_iterator iter=marginIds.begin(); iter!=marginIds.end(); ++iter)
				{
					if( !uContains(ids, *iter) && nextDepth.find(*iter) == nextDepth.end())
					{
						nextDepth.insert(*iter);
					}
				}
			}
		}
		++d;
	}
	return ids;
}

void optimizeTOROGraph(
		const std::map<int, int> & depthGraph,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		std::map<int, Transform> & optimizedPoses,
		int toroIterations,
		bool toroInitialGuess,
		bool ignoreCovariance,
		std::list<std::map<int, Transform> > * intermediateGraphes)
{
	optimizedPoses.clear();
	if(depthGraph.size() && poses.size()>=2 && links.size()>=1)
	{
		// Modify IDs using the margin from the current signature (TORO root will be the last signature)
		int m = 0;
		int toroId = 1;
		std::map<int, int> rtabmapToToro; // <RTAB-Map ID, TORO ID>
		std::map<int, int> toroToRtabmap; // <TORO ID, RTAB-Map ID>
		std::map<int, int> idsTmp = depthGraph;
		while(idsTmp.size())
		{
			for(std::map<int, int>::iterator iter = idsTmp.begin(); iter!=idsTmp.end();)
			{
				if(m == iter->second)
				{
					rtabmapToToro.insert(std::make_pair(iter->first, toroId));
					toroToRtabmap.insert(std::make_pair(toroId, iter->first));
					++toroId;
					idsTmp.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
			++m;
		}

		std::map<int, rtabmap::Transform> posesToro;
		std::multimap<int, rtabmap::Link> edgeConstraintsToro;
		for(std::map<int, rtabmap::Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if(uContains(depthGraph, iter->first))
			{
				UASSERT(!iter->second.isNull());
				posesToro.insert(std::make_pair(rtabmapToToro.at(iter->first), iter->second));
			}
		}
		for(std::multimap<int, rtabmap::Link>::const_iterator iter = links.begin();
			iter!=links.end();
			++iter)
		{
			if(uContains(depthGraph, iter->second.from()) && uContains(depthGraph, iter->second.to()))
			{
				UASSERT(!iter->second.transform().isNull());
				edgeConstraintsToro.insert(std::make_pair(rtabmapToToro.at(iter->first), Link(rtabmapToToro.at(iter->first), rtabmapToToro.at(iter->second.to()), iter->second.type(), iter->second.transform(), iter->second.variance())));
			}
		}

		std::map<int, rtabmap::Transform> optimizedPosesToro;

		if(posesToro.size() && edgeConstraintsToro.size())
		{
			std::list<std::map<int, rtabmap::Transform> > graphesToro;

			// Optimize!
			optimizeTOROGraph(
					posesToro,
					edgeConstraintsToro,
					optimizedPosesToro,
					toroIterations,
					toroInitialGuess,
					ignoreCovariance,
					&graphesToro);

			for(std::map<int, rtabmap::Transform>::iterator iter=optimizedPosesToro.begin(); iter!=optimizedPosesToro.end(); ++iter)
			{
				optimizedPoses.insert(std::make_pair(toroToRtabmap.at(iter->first), iter->second));
			}

			if(intermediateGraphes)
			{
				for(std::list<std::map<int, rtabmap::Transform> >::iterator iter = graphesToro.begin(); iter!=graphesToro.end(); ++iter)
				{
					std::map<int, rtabmap::Transform> tmp;
					for(std::map<int, rtabmap::Transform>::iterator jter=iter->begin(); jter!=iter->end(); ++jter)
					{
						tmp.insert(std::make_pair(toroToRtabmap.at(jter->first), jter->second));
					}
					intermediateGraphes->push_back(tmp);
				}
			}
		}
		else
		{
			UERROR("No TORO poses and constraints!?");
		}
	}
	else if(links.size() == 0 && poses.size() == 1)
	{
		optimizedPoses = poses;
	}
	else
	{
		UERROR("Wrong inputs! depthGraph=%d poses=%d links=%d",
				(int)depthGraph.size(), (int)poses.size(), (int)links.size());
	}
}

//On success, optimizedPoses is cleared and new poses are inserted in
void optimizeTOROGraph(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		std::map<int, Transform> & optimizedPoses,
		int toroIterations,
		bool toroInitialGuess,
		bool ignoreCovariance,
		std::list<std::map<int, Transform> > * intermediateGraphes) // contains poses after tree init to last one before the end
{
	UASSERT(toroIterations>0);
	optimizedPoses.clear();
	if(edgeConstraints.size()>=1 && poses.size()>=2)
	{
		// Apply TORO optimization
		AISNavigation::TreeOptimizer3 pg;
		pg.verboseLevel = 0;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			float x,y,z, roll,pitch,yaw;
			UASSERT(!iter->second.isNull());
			pcl::getTranslationAndEulerAngles(iter->second.toEigen3f(), x,y,z, roll,pitch,yaw);
			AISNavigation::TreePoseGraph3::Pose p(x, y, z, roll, pitch, yaw);
			AISNavigation::TreePoseGraph<AISNavigation::Operations3D<double> >::Vertex* v = pg.addVertex(iter->first, p);
			if (v)
			{
				v->transformation=AISNavigation::TreePoseGraph3::Transformation(p);
			}
			else
			{
				UERROR("cannot insert vertex %d!?", iter->first);
			}
		}

		for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
		{
			int id1 = iter->first;
			int id2 = iter->second.to();
			float x,y,z, roll,pitch,yaw;
			UASSERT(!iter->second.transform().isNull());
			pcl::getTranslationAndEulerAngles(iter->second.transform().toEigen3f(), x,y,z, roll,pitch,yaw);
			AISNavigation::TreePoseGraph3::Pose p(x, y, z, roll, pitch, yaw);
			AISNavigation::TreePoseGraph3::InformationMatrix inf = DMatrix<double>::I(6);
			if(!ignoreCovariance && iter->second.variance()>0)
			{
				inf[0][0] = 1.0f/iter->second.variance(); // x
				inf[1][1] = 1.0f/iter->second.variance(); // y
				inf[2][2] = 1.0f/iter->second.variance(); // z
				inf[3][3] = 1.0f/iter->second.variance(); // roll
				inf[4][4] = 1.0f/iter->second.variance(); // pitch
				inf[5][5] = 1.0f/iter->second.variance(); // yaw
			}

			AISNavigation::TreePoseGraph<AISNavigation::Operations3D<double> >::Vertex* v1=pg.vertex(id1);
			AISNavigation::TreePoseGraph<AISNavigation::Operations3D<double> >::Vertex* v2=pg.vertex(id2);
			AISNavigation::TreePoseGraph3::Transformation t(p);
			if (!pg.addEdge(v1, v2, t, inf))
			{
				UERROR("Map: Edge already exits between nodes %d and %d, skipping", id1, id2);
				return;
			}
		}
		pg.buildMST(pg.vertices.begin()->first); // pg.buildSimpleTree();

		UDEBUG("Initial guess...");
		if(toroInitialGuess)
		{
			pg.initializeOnTree(); // optional
		}

		pg.initializeTreeParameters();
		UDEBUG("Building TORO tree... (if a crash happens just after this msg, "
			   "TORO is not able to find the root of the graph!)");
		pg.initializeOptimization();

		UDEBUG("TORO iterate begin (iterations=%d)", toroIterations);
		for (int i=0; i<toroIterations; i++)
		{
			if(intermediateGraphes && (toroInitialGuess || i>0))
			{
				std::map<int, Transform> tmpPoses;
				for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
				{
					AISNavigation::TreePoseGraph<AISNavigation::Operations3D<double> >::Vertex* v=pg.vertex(iter->first);
					v->pose=v->transformation.toPoseType();
					Transform newPose = Transform::fromEigen3f(pcl::getTransformation(v->pose.x(), v->pose.y(), v->pose.z(), v->pose.roll(), v->pose.pitch(), v->pose.yaw()));

					tmpPoses.insert(std::pair<int, Transform>(iter->first, newPose));
				}
				intermediateGraphes->push_back(tmpPoses);
			}

			pg.iterate();
		}
		UDEBUG("TORO iterate end");

		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			AISNavigation::TreePoseGraph<AISNavigation::Operations3D<double> >::Vertex* v=pg.vertex(iter->first);
			v->pose=v->transformation.toPoseType();
			Transform newPose = Transform::fromEigen3f(pcl::getTransformation(v->pose.x(), v->pose.y(), v->pose.z(), v->pose.roll(), v->pose.pitch(), v->pose.yaw()));

			optimizedPoses.insert(std::pair<int, Transform>(iter->first, newPose));
		}

		//Eigen::Matrix4f newPose = transformToEigen4f(optimizedPoses.at(poses.rbegin()->first));
		//Eigen::Matrix4f oldPose = transformToEigen4f(poses.rbegin()->second);
		//Eigen::Matrix4f poseCorrection = oldPose.inverse() * newPose; // transform from odom to correct odom
		//Eigen::Matrix4f result = oldPose*poseCorrection*oldPose.inverse();
		//mapCorrection = transformFromEigen4f(result);
	}
	else if(edgeConstraints.size() == 0 && poses.size() == 1)
	{
		optimizedPoses = poses;
	}
	else
	{
		UWARN("This method should be called at least with 1 pose!");
	}
}

bool saveTOROGraph(
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
			fprintf(file, "EDGE3 %d %d %f %f %f %f %f %f %f 0 0 0 0 0 %f 0 0 0 0 %f 0 0 0 %f 0 0 %f 0 %f\n",
					iter->first,
					iter->second.to(),
					x,
					y,
					z,
					roll,
					pitch,
					yaw,
					1.0f/iter->second.variance(),
					1.0f/iter->second.variance(),
					1.0f/iter->second.variance(),
					1.0f/iter->second.variance(),
					1.0f/iter->second.variance(),
					1.0f/iter->second.variance());
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

bool loadTOROGraph(const std::string & fileName,
		std::map<int, Transform> & poses,
		std::multimap<int, std::pair<int, Transform> > & edgeConstraints)
{
	FILE * file = 0;
#ifdef _MSC_VER
	fopen_s(&file, fileName.c_str(), "r");
#else
	file = fopen(fileName.c_str(), "r");
#endif

	if(file)
	{
		char line[200];
		while ( fgets (line , 200 , file) != NULL )
		{
			std::vector<std::string> strList = uListToVector(uSplit(line, ' '));
			if(strList.size() == 8)
			{
				//VERTEX3
				int id = atoi(strList[1].c_str());
				float x = atof(strList[2].c_str());
				float y = atof(strList[3].c_str());
				float z = atof(strList[4].c_str());
				float roll = atof(strList[5].c_str());
				float pitch = atof(strList[6].c_str());
				float yaw = atof(strList[7].c_str());
				Transform pose = Transform::fromEigen3f(pcl::getTransformation(x, y, z, roll, pitch, yaw));
				std::map<int, Transform>::iterator iter = poses.find(id);
				if(iter != poses.end())
				{
					iter->second = pose;
				}
				else
				{
					UFATAL("");
				}
			}
			else if(strList.size() == 30)
			{
				//EDGE3
				int idFrom = atoi(strList[1].c_str());
				int idTo = atoi(strList[2].c_str());
				float x = atof(strList[3].c_str());
				float y = atof(strList[4].c_str());
				float z = atof(strList[5].c_str());
				float roll = atof(strList[6].c_str());
				float pitch = atof(strList[7].c_str());
				float yaw = atof(strList[8].c_str());
				Transform transform = Transform::fromEigen3f(pcl::getTransformation(x, y, z, roll, pitch, yaw));
				if(poses.find(idFrom) != poses.end() && poses.find(idTo) != poses.end())
				{
					std::pair<int, Transform> edge(idTo, transform);
					edgeConstraints.insert(std::pair<int, std::pair<int, Transform> >(idFrom, edge));
				}
				else
				{
					UFATAL("");
				}
			}
			else
			{
				UFATAL("Error parsing map file %s", fileName.c_str());
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


std::map<int, Transform> radiusPosesFiltering(const std::map<int, Transform> & poses, float radius, float angle, bool keepLatest)
{
	if(poses.size() > 1 && radius > 0.0f && angle>0.0f)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(poses.size());
		int i=0;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			(*cloud)[i++] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
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
			// ignore scans
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
						const Transform & checkT = transforms.at(kIndices[j]);
						// same orientation?
						Eigen::Vector3f vB = checkT.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
						double a = pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
						if(a <= angle)
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
	if(poses.size() > 1 && radius > 0.0f && angle>0.0f)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(poses.size());
		int i=0;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			(*cloud)[i++] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
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
					const Transform & checkT = transforms.at(kIndices[j]);
					// same orientation?
					Eigen::Vector3f vB = checkT.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
					double a = pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
					if(a <= angle)
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
		return pose_.getDistance(pose);
	}

	void setClosed(bool closed) {closed_ = closed;}
	void setFromId(int fromId) {fromId_ = fromId;}
	void setCostSoFar(float costSoFar) {costSoFar_ = costSoFar;}
	void setDistToEnd(float distToEnd) {distToEnd_ = distToEnd;}

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

std::vector<int> computePath(
			const std::map<int, rtabmap::Transform> & poses,
			const std::multimap<int, int> & links,
			int from,
			int to)
{
	std::list<int> path;

	//A*
	int startNode = from;
	int endNode = to;
	rtabmap::Transform endPose = poses.at(endNode);
	std::map<int, Node> nodes;
	nodes.insert(std::make_pair(startNode, Node(startNode, 0, poses.at(startNode))));
	std::priority_queue<Pair, std::vector<Pair>, Order> pq;
	pq.push(Pair(startNode, 0));

	while(pq.size())
	{
		Node & currentNode = nodes.find(pq.top().first)->second;
		pq.pop();
		currentNode.setClosed(true);

		if(currentNode.id() == endNode)
		{
			while(currentNode.id()!=startNode)
			{
				path.push_front(currentNode.id());
				currentNode = nodes.find(currentNode.fromId())->second;
			}
			path.push_front(startNode);
			break;
		}

		// lookup neighbors
		for(std::multimap<int, int>::const_iterator iter = links.find(currentNode.id());
			iter!=links.end() && iter->first == currentNode.id();
			++iter)
		{
			std::map<int, Node>::iterator nodeIter = nodes.find(iter->second);
			if(nodeIter == nodes.end())
			{
				std::map<int, rtabmap::Transform>::const_iterator poseIter = poses.find(iter->second);
				UASSERT(poseIter != poses.end());
				Node n(iter->second, currentNode.id(), poseIter->second);
				n.setCostSoFar(currentNode.costSoFar() + currentNode.distFrom(poseIter->second));
				n.setDistToEnd(n.distFrom(endPose));
				nodes.insert(std::make_pair(iter->second, n));
				pq.push(Pair(n.id(), n.totalCost()));
			}
			else if(nodeIter->second.isOpened())
			{
				float newCostSoFar = currentNode.costSoFar() + currentNode.distFrom(nodeIter->second.pose());
				if(nodeIter->second.costSoFar() > newCostSoFar)
				{
					UWARN("newCostSoFar > previous cost (%f vs %f)", newCostSoFar, nodeIter->second.costSoFar());
				}
			}
		}
	}
	return uListToVector(path);
}


} /* namespace rtabmap */
