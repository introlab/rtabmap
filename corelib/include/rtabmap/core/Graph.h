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

#ifndef GRAPH_H_
#define GRAPH_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <map>
#include <list>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/GPS.h>
#include <rtabmap/core/CameraModel.h>

namespace rtabmap {
class Memory;

namespace graph {

////////////////////////////////////////////
// Graph utilities
////////////////////////////////////////////

bool RTABMAP_EXP exportPoses(
		const std::string & filePath,
		int format, // 0=Raw (*.txt), 1=RGBD-SLAM motion capture (*.txt) (10=without change of coordinate frame, 11=10+ID), 2=KITTI (*.txt), 3=TORO (*.graph), 4=g2o (*.g2o)
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & constraints = std::multimap<int, Link>(), // required for formats 3 and 4
		const std::map<int, double> & stamps = std::map<int, double>(),  // required for format 1
		const ParametersMap & parameters = ParametersMap()); // optional for formats 3 and 4

bool RTABMAP_EXP importPoses(
		const std::string & filePath,
		int format, // 0=Raw, 1=RGBD-SLAM motion capture (10=without change of coordinate frame, 11=10+ID), 2=KITTI, 3=TORO, 4=g2o, 5=NewCollege(t,x,y), 6=Malaga Urban GPS, 7=St Lucia INS, 8=Karlsruhe, 9=EuRoC MAV
		std::map<int, Transform> & poses,
		std::multimap<int, Link> * constraints = 0, // optional for formats 3 and 4
		std::map<int, double> * stamps = 0); // optional for format 1 and 9

bool RTABMAP_EXP exportGPS(
		const std::string & filePath,
		const std::map<int, GPS> & gpsValues,
		unsigned int rgba = 0xFFFFFFFF);

/**
 * Compute translation and rotation errors for KITTI datasets.
 * See http://www.cvlibs.net/datasets/kitti/eval_odometry.php.
 * @param poses_gt, Ground Truth poses
 * @param poses_result, Estimated poses
 * @param t_err, Output translation error (%)
 * @param r_err, Output rotation error (deg/m)
 */
void RTABMAP_EXP calcKittiSequenceErrors(
		const std::vector<Transform> &poses_gt,
		const std::vector<Transform> &poses_result,
		float & t_err,
		float & r_err);

/**
 * Compute average of translation and rotation errors between each poses.
 * @param poses_gt, Ground Truth poses
 * @param poses_result, Estimated poses
 * @param t_err, Output translation error (m)
 * @param r_err, Output rotation error (deg)
 */
void RTABMAP_EXP calcRelativeErrors (
		const std::vector<Transform> &poses_gt,
		const std::vector<Transform> &poses_result,
		float & t_err,
		float & r_err);

/**
 * Compute root-mean-square error (RMSE) like the TUM RGBD
 * dataset's evaluation tool (absolute trajectory error).
 * See https://vision.in.tum.de/data/datasets/rgbd-dataset
 * @param groundTruth, Ground Truth poses
 * @param poses, Estimated poses
 * @return Gt to Map transform
 */
Transform RTABMAP_EXP calcRMSE(
		const std::map<int, Transform> &groundTruth,
		const std::map<int, Transform> &poses,
		float & translational_rmse,
		float & translational_mean,
		float & translational_median,
		float & translational_std,
		float & translational_min,
		float & translational_max,
		float & rotational_rmse,
		float & rotational_mean,
		float & rotational_median,
		float & rotational_std,
		float & rotational_min,
		float & rotational_max);

void RTABMAP_EXP computeMaxGraphErrors(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		float & maxLinearErrorRatio,
		float & maxAngularErrorRatio,
		float & maxLinearError,
		float & maxAngularError,
		const Link ** maxLinearErrorLink = 0,
		const Link ** maxAngularErrorLink = 0,
		bool for3DoF = false);

std::vector<double> RTABMAP_EXP getMaxOdomInf(const std::multimap<int, Link> & links);

std::multimap<int, Link>::iterator RTABMAP_EXP findLink(
		std::multimap<int, Link> & links,
		int from,
		int to,
		bool checkBothWays = true,
		Link::Type type = Link::kUndef);
std::multimap<int, int>::iterator RTABMAP_EXP findLink(
		std::multimap<int, int> & links,
		int from,
		int to,
		bool checkBothWays = true);
std::multimap<int, Link>::const_iterator RTABMAP_EXP findLink(
		const std::multimap<int, Link> & links,
		int from,
		int to,
		bool checkBothWays = true,
		Link::Type type = Link::kUndef);
std::multimap<int, int>::const_iterator RTABMAP_EXP findLink(
		const std::multimap<int, int> & links,
		int from,
		int to,
		bool checkBothWays = true);
std::list<Link> RTABMAP_EXP findLinks(
		const std::multimap<int, Link> & links,
		int from);

std::multimap<int, Link> RTABMAP_EXP filterDuplicateLinks(
		const std::multimap<int, Link> & links);
/**
 * Return links not of type "filteredType". If inverted=true, return links of type "filteredType".
 */
std::multimap<int, Link> RTABMAP_EXP filterLinks(
		const std::multimap<int, Link> & links,
		Link::Type filteredType,
		bool inverted = false);
/**
 * Return links not of type "filteredType". If inverted=true, return links of type "filteredType".
 */
std::map<int, Link> RTABMAP_EXP filterLinks(
		const std::map<int, Link> & links,
		Link::Type filteredType,
		bool inverted = false);

//Note: This assumes a coordinate system where X is forward, * Y is up, and Z is right.
std::map<int, Transform> RTABMAP_EXP frustumPosesFiltering(
		const std::map<int, Transform> & poses,
		const Transform & cameraPose,
		float horizontalFOV = 45.0f, // in degrees, xfov = atan((image_width/2)/fx)*2
		float verticalFOV = 45.0f,   // in degrees, yfov = atan((image_height/2)/fy)*2
		float nearClipPlaneDistance = 0.1f,
		float farClipPlaneDistance = 100.0f,
		bool negative = false);

/**
 * Get only the the most recent or older poses in the defined radius.
 * @param poses The poses
 * @param radius Radius (m) of the search for near neighbors
 * @param angle Maximum angle (rad, [0,PI]) of accepted neighbor nodes in the radius (0 means ignore angle)
 * @param keepLatest keep the latest node if true, otherwise the oldest node is kept
 * @return A map containing only most recent or older poses in the the defined radius
 */
std::map<int, Transform> RTABMAP_EXP radiusPosesFiltering(
		const std::map<int, Transform> & poses,
		float radius,
		float angle,
		bool keepLatest = true);

/**
 * Get all neighbor nodes in a fixed radius around each pose.
 * @param poses The poses
 * @param radius Radius (m) of the search for near neighbors
 * @param angle Maximum angle (rad, [0,PI]) of accepted neighbor nodes in the radius (0 means ignore angle)
 * @return A map between each pose id and its neighbors found in the radius
 */
std::multimap<int, int> RTABMAP_EXP radiusPosesClustering(
		const std::map<int, Transform> & poses,
		float radius,
		float angle);

void reduceGraph(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		std::multimap<int, int> & hyperNodes, //<parent ID, child ID>
		std::multimap<int, Link> & hyperLinks);

/**
 * Perform A* path planning in the graph.
 * @param poses The graph's poses
 * @param links The graph's links (from node id -> to node id)
 * @param from initial node
 * @param to final node
 * @param updateNewCosts Keep up-to-date costs while traversing the graph.
 * @return the path ids from id "from" to id "to" including initial and final nodes.
 */
std::list<std::pair<int, Transform> > RTABMAP_EXP computePath(
			const std::map<int, rtabmap::Transform> & poses,
			const std::multimap<int, int> & links,
			int from,
			int to,
			bool updateNewCosts = false);

/**
 * Perform Dijkstra path planning in the graph.
 * @param poses The graph's poses
 * @param links The graph's links (from node id -> to node id)
 * @param from initial node
 * @param to final node
 * @param updateNewCosts Keep up-to-date costs while traversing the graph.
 * @param useSameCostForAllLinks Ignore distance between nodes
 * @return the path ids from id "from" to id "to" including initial and final nodes.
 */
std::list<int> RTABMAP_EXP computePath(
			const std::multimap<int, Link> & links,
			int from,
			int to,
			bool updateNewCosts = false,
			bool useSameCostForAllLinks = false);

/**
 * Perform Dijkstra path planning in the graph.
 * @param fromId initial node
 * @param toId final node
 * @param memory The graph's memory
 * @param lookInDatabase check links in database
 * @param updateNewCosts Keep up-to-date costs while traversing the graph.
 * @return the path ids from id "fromId" to id "toId" including initial and final nodes (Identity pose for the first node).
 */
std::list<std::pair<int, Transform> > RTABMAP_EXP computePath(
		int fromId,
		int toId,
		const Memory * memory,
		bool lookInDatabase = true,
		bool updateNewCosts = false,
		float linearVelocity = 0.0f,   // m/sec
		float angularVelocity = 0.0f); // rad/sec

/**
 * Find the nearest node of the target pose
 * @param nodes the nodes to search for
 * @param targetPose the target pose to search around
 * @param distance squared distance of the nearest node found (optional)
 * @return the node id.
 */
int RTABMAP_EXP findNearestNode(
		const std::map<int, rtabmap::Transform> & poses,
		const rtabmap::Transform & targetPose,
		float * distance = 0);

/**
 * Find the nearest nodes of the query pose or node
 * @param nodeId the query id
 * @param nodes the nodes to search for
 * @param radius radius to search for (m), if 0, k should be > 0.
 * @param k max nearest neighbors (0=all inside the radius)
 * @return the nodes with squared distance to query node.
 */
std::map<int, float> RTABMAP_EXP findNearestNodes(
		int nodeId,
		const std::map<int, Transform> & poses,
		float radius,
		float angle = 0.0f,
		int k=0);
std::map<int, float> RTABMAP_EXP findNearestNodes(
		const Transform & targetPose,
		const std::map<int, Transform> & poses,
		float radius,
		float angle = 0.0f,
		int k=0);
std::map<int, Transform> RTABMAP_EXP findNearestPoses(
		int nodeId,
		const std::map<int, Transform> & poses,
		float radius,
		float angle = 0.0f,
		int k=0);
std::map<int, Transform> RTABMAP_EXP findNearestPoses(
		const Transform & targetPose,
		const std::map<int, Transform> & poses,
		float radius,
		float angle = 0.0f,
		int k=0);

// typedef hack to avoid error with RTABMAP_DEPRECATED
typedef std::map<int, float> _mapIntFloat;
typedef std::map<int, Transform> _mapIntTransform;
RTABMAP_DEPRECATED(_mapIntFloat RTABMAP_EXP findNearestNodes(const std::map<int, rtabmap::Transform> & nodes, const rtabmap::Transform & targetPose, int k), "Use new findNearestNodes() interface with radius=0, angle=0.");
RTABMAP_DEPRECATED(_mapIntFloat RTABMAP_EXP getNodesInRadius(int nodeId, const std::map<int, Transform> & nodes, float radius), "Renamed to findNearestNodes()");
RTABMAP_DEPRECATED(_mapIntFloat RTABMAP_EXP getNodesInRadius(const Transform & targetPose, const std::map<int, Transform> & nodes, float radius), "Renamed to findNearestNodes()");
RTABMAP_DEPRECATED(_mapIntTransform RTABMAP_EXP getPosesInRadius(int nodeId, const std::map<int, Transform> & nodes, float radius, float angle = 0.0f), "Renamed to findNearestNodes()");
RTABMAP_DEPRECATED(_mapIntTransform RTABMAP_EXP getPosesInRadius(const Transform & targetPose, const std::map<int, Transform> & nodes, float radius, float angle = 0.0f), "Renamed to findNearestNodes()");

float RTABMAP_EXP computePathLength(
		const std::vector<std::pair<int, Transform> > & path,
		unsigned int fromIndex = 0,
		unsigned int toIndex = 0);

// assuming they are all linked in map order
float RTABMAP_EXP computePathLength(
		const std::map<int, Transform> & path);

std::list<std::map<int, Transform> > RTABMAP_EXP getPaths(
		std::map<int, Transform> poses,
		const std::multimap<int, Link> & links);

void RTABMAP_EXP computeMinMax(const std::map<int, Transform> & poses,
		cv::Vec3f & min,
		cv::Vec3f & max);

} /* namespace graph */

} /* namespace rtabmap */
#endif /* GRAPH_H_ */
