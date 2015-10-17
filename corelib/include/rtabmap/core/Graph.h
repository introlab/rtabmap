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

#ifndef GRAPH_H_
#define GRAPH_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <map>
#include <list>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>

namespace rtabmap {
class Memory;

namespace graph {

////////////////////////////////////////////
// Graph optimizers
////////////////////////////////////////////
class RTABMAP_EXP Optimizer
{
public:
	enum Type {
		kTypeUndef = -1,
		kTypeTORO = 0,
		kTypeG2O = 1,
		kTypeGTSAM = 2,
		kTypeCVSBA = 3
	};
	static Optimizer * create(const ParametersMap & parameters);
	static Optimizer * create(Optimizer::Type & type, const ParametersMap & parameters = ParametersMap());

	// Get connected poses and constraints from a set of links
	static void getConnectedGraph(
			int fromId,
			const std::map<int, Transform> & posesIn,
			const std::multimap<int, Link> & linksIn, // only one link between two poses
			std::map<int, Transform> & posesOut,
			std::multimap<int, Link> & linksOut,
			int depth = 0);

public:
	virtual ~Optimizer() {}

	virtual Type type() const = 0;

	int iterations() const {return iterations_;}
	bool isSlam2d() const {return slam2d_;}
	bool isCovarianceIgnored() const {return covarianceIgnored_;}
	double epsilon() const {return epsilon_;}
	bool isRobust() const {return robust_;}

	// inherited classes should implement one of these methods
	virtual std::map<int, Transform> optimize(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & constraints,
			std::list<std::map<int, Transform> > * intermediateGraphes = 0);
	virtual std::map<int, Transform> optimizeBA(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, Signature> & signatures);

	virtual void parseParameters(const ParametersMap & parameters);

protected:
	Optimizer(
			int iterations         = Parameters::defaultRGBDOptimizeIterations(),
			bool slam2d            = Parameters::defaultRGBDOptimizeSlam2D(),
			bool covarianceIgnored = Parameters::defaultRGBDOptimizeVarianceIgnored(),
			double epsilon         = Parameters::defaultRGBDOptimizeEpsilon(),
			bool robust            = Parameters::defaultRGBDOptimizeRobust());
	Optimizer(const ParametersMap & parameters);

private:
	int iterations_;
	bool slam2d_;
	bool covarianceIgnored_;
	double epsilon_;
	bool robust_;
};

class RTABMAP_EXP TOROOptimizer : public Optimizer
{
public:
	static bool saveGraph(
			const std::string & fileName,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & edgeConstraints);
	static bool loadGraph(
			const std::string & fileName,
			std::map<int, Transform> & poses,
			std::multimap<int, Link> & edgeConstraints);

public:
	TOROOptimizer(
			int iterations         = Parameters::defaultRGBDOptimizeIterations(),
			bool slam2d            = Parameters::defaultRGBDOptimizeSlam2D(),
			bool covarianceIgnored = Parameters::defaultRGBDOptimizeVarianceIgnored(),
			double epsilon         = Parameters::defaultRGBDOptimizeEpsilon()) :
		Optimizer(iterations, slam2d, covarianceIgnored, epsilon) {}
	TOROOptimizer(const ParametersMap & parameters) :
		Optimizer(parameters) {}
	virtual ~TOROOptimizer() {}

	virtual Type type() const {return kTypeTORO;}

	virtual std::map<int, Transform> optimize(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & edgeConstraints,
			std::list<std::map<int, Transform> > * intermediateGraphes = 0);
};

class RTABMAP_EXP G2OOptimizer : public Optimizer
{
public:
	static bool available();
	static bool saveGraph(
			const std::string & fileName,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & edgeConstraints,
			bool useRobustConstraints = false);

public:
	G2OOptimizer(
			int iterations         = Parameters::defaultRGBDOptimizeIterations(),
			bool slam2d            = Parameters::defaultRGBDOptimizeSlam2D(),
			bool covarianceIgnored = Parameters::defaultRGBDOptimizeVarianceIgnored(),
			double epsilon         = Parameters::defaultRGBDOptimizeEpsilon(),
			bool robust            = Parameters::defaultRGBDOptimizeRobust()) :
		Optimizer(iterations, slam2d, covarianceIgnored, epsilon, robust) {}

	G2OOptimizer(const ParametersMap & parameters) :
		Optimizer(parameters) {}
	virtual ~G2OOptimizer() {}

	virtual Type type() const {return kTypeG2O;}

	virtual std::map<int, Transform> optimize(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & edgeConstraints,
			std::list<std::map<int, Transform> > * intermediateGraphes = 0);
};

class RTABMAP_EXP GTSAMOptimizer : public Optimizer
{
public:
	static bool available();

public:
	GTSAMOptimizer(
			int iterations         = Parameters::defaultRGBDOptimizeIterations(),
			bool slam2d            = Parameters::defaultRGBDOptimizeSlam2D(),
			bool covarianceIgnored = Parameters::defaultRGBDOptimizeVarianceIgnored(),
			double epsilon         = Parameters::defaultRGBDOptimizeEpsilon(),
			bool robust            = Parameters::defaultRGBDOptimizeRobust()) :
		Optimizer(iterations, slam2d, covarianceIgnored, epsilon, robust) {}

	GTSAMOptimizer(const ParametersMap & parameters) :
		Optimizer(parameters) {}
	virtual ~GTSAMOptimizer() {}

	virtual Type type() const {return kTypeGTSAM;}

	virtual std::map<int, Transform> optimize(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & edgeConstraints,
			std::list<std::map<int, Transform> > * intermediateGraphes = 0);
};

class RTABMAP_EXP CVSBAOptimizer : public Optimizer
{
public:
	static bool available();

public:
	CVSBAOptimizer(int iterations = 100, bool slam2d = false, bool covarianceIgnored = false) :
		Optimizer(iterations, slam2d, covarianceIgnored),
		inlierDistance_(0.02),
		minInliers_(10){}
	CVSBAOptimizer(const ParametersMap & parameters) :
		Optimizer(parameters),
		inlierDistance_(0.02),
		minInliers_(10){}
	virtual ~CVSBAOptimizer() {}

	virtual Type type() const {return kTypeCVSBA;}

	void setInlierDistance(float inlierDistance) {inlierDistance_ = inlierDistance;}
	void setMinInliers(int minInliers) {minInliers_ = minInliers;}

	virtual std::map<int, Transform> optimizeBA(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, Signature> & signatures);

private:
	float inlierDistance_;
	float minInliers_;
};

bool RTABMAP_EXP exportPoses(
		const std::string & filePath,
		int format, // 0=Raw (*.txt), 1=RGBD-SLAM (*.txt), 2=KITTI (*.txt), 3=TORO (*.graph), 4=g2o (*.g2o)
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & constraints, // required for formats 3 and 4
		const std::map<int, double> & stamps); // required for format 1

////////////////////////////////////////////
// Graph utilities
////////////////////////////////////////////
std::multimap<int, Link>::iterator RTABMAP_EXP findLink(
		std::multimap<int, Link> & links,
		int from,
		int to);
std::multimap<int, int>::iterator RTABMAP_EXP findLink(
		std::multimap<int, int> & links,
		int from,
		int to);
std::multimap<int, Link>::const_iterator RTABMAP_EXP findLink(
		const std::multimap<int, Link> & links,
		int from,
		int to);
std::multimap<int, int>::const_iterator RTABMAP_EXP findLink(
		const std::multimap<int, int> & links,
		int from,
		int to);

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

int RTABMAP_EXP findNearestNode(
		const std::map<int, rtabmap::Transform> & nodes,
		const rtabmap::Transform & targetPose);

/**
 * Get nodes near the query
 * @param nodeId the query id
 * @param nodes the nodes to search for
 * @param maxNearestNeighbors Maximum nearest neighbor to get. 0 means all.
 * @param radius radius to search for (m)
 * @return the nodes with squared distance to query node.
 */
std::map<int, float> RTABMAP_EXP getNodesInRadius(
		int nodeId,
		const std::map<int, Transform> & nodes,
		float radius);
std::map<int, Transform> RTABMAP_EXP getPosesInRadius(
		int nodeId,
		const std::map<int, Transform> & nodes,
		float radius);

float RTABMAP_EXP computePathLength(
		const std::vector<std::pair<int, Transform> > & path,
		unsigned int fromIndex = 0,
		unsigned int toIndex = 0);

std::list<std::map<int, Transform> > RTABMAP_EXP getPaths(
		std::map<int, Transform> poses,
		const std::multimap<int, Link> & links);


} /* namespace graph */

} /* namespace rtabmap */
#endif /* GRAPH_H_ */
