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

namespace rtabmap {

std::multimap<int, Link>::iterator RTABMAP_EXP findLink(
		std::multimap<int, Link> & links,
		int from,
		int to);

// <int, depth> depth=0 means infinite depth
std::map<int, int> RTABMAP_EXP generateDepthGraph(
		const std::multimap<int, Link> & links,
		int fromId,
		int depth = 0);

void RTABMAP_EXP optimizeTOROGraph(
		const std::map<int, int> & depthGraph,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		std::map<int, Transform> & optimizedPoses,
		int toroIterations = 100,
		bool toroInitialGuess = true,
		bool ignoreCovariance = false,
		std::list<std::map<int, Transform> > * intermediateGraphes = 0);

void RTABMAP_EXP optimizeTOROGraph(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		std::map<int, Transform> & optimizedPoses,
		int toroIterations = 100,
		bool toroInitialGuess = true,
		bool ignoreCovariance = false,
		std::list<std::map<int, Transform> > * intermediateGraphes = 0);

bool RTABMAP_EXP saveTOROGraph(
		const std::string & fileName,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints);

bool RTABMAP_EXP loadTOROGraph(const std::string & fileName,
		std::map<int, Transform> & poses,
		std::multimap<int, std::pair<int, Transform> > & edgeConstraints);

std::map<int, Transform> RTABMAP_EXP radiusPosesFiltering(
		const std::map<int, Transform> & poses,
		float radius,
		float angle,
		bool keepLatest = true);

/**
 * Get all neighbor nodes in a fixed radius around each pose.
 * @param poses The poses
 * @param radius Radius (m) of the search for near neighbors
 * @param angle Maximum angle (rad, [0,PI]) of accepted neighbor nodes in the radius
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
 * @return the path ids from id "from" to id "to" including initial and final nodes.
 */
std::vector<int> RTABMAP_EXP computePath(
			const std::map<int, rtabmap::Transform> & poses,
			const std::multimap<int, int> & links,
			int from,
			int to);

} /* namespace rtabmap */
#endif /* GRAPH_H_ */
