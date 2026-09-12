// Comparison of the two ways of getting the graph depth of every node of a map,
// which is what Rtabmap::process() needs to reject proximity candidates that are
// too far in the graph (Parameters::kRGBDProximityMaxGraphDepth()):
//
//   - one graph::computePath() (A*) per candidate, which is what it did before, each
//     search paying for the whole graph again;
//   - one graph::computePathDepths() (BFS) for all of them, which is what it does now.
//
// The graph is a spiral walked inward, a pose every 30 cm: the shape a robot draws
// covering a room, and the worst case for the A* heuristic. Two nodes on neighboring
// turns are ~50 cm apart in space but a whole turn apart in the graph, so the straight
// line to the goal says nothing about the path to it and each A* expands nearly the
// whole graph. That is exactly the situation proximity detection is called for.
//
// Its own executable, run by ctest under the "performance" label, so that its seconds
// of benchmarking stay out of the unit test shards:
//   ctest -L performance    to run them
//   ctest -LE performance   to skip them
//   bin/test_graph_perf --gtest_filter=*Spiral*
//
// Each spiral it builds is written to the temp directory as a g2o file (the path is
// printed with the results), so that the graph a number was measured on can be looked at
// with rtabmap-graphViewer or g2o_viewer, or replayed by another tool.
//
// The times are reported rather than asserted on, as they depend on the machine. What
// is asserted is that both approaches answer the same thing on the spiral, so that the
// numbers below compare two ways of computing the same depths.
#include <gtest/gtest.h>
#include "TestUtils.h"
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <list>
#include <map>
#include <vector>

using namespace rtabmap;

namespace {

// All the spirals have their turns PITCH apart and a pose every SPACING meters, walked
// from their own radius in to RADIUS_END.
static const float RADIUS_END = 1.0f;
static const float PITCH = 0.5f;
static const float SPACING = 0.3f;

// An Archimedean spiral r(theta) = radiusStart - pitch*theta/(2*pi), walked from
// radiusStart inward to radiusEnd with one pose every `spacing` meters of arc length,
// linked as a chain in the order it was walked. Ids are 1..n, so the last id is the
// innermost pose: the one a session ends on, and the one the depths are computed from.
struct Spiral
{
	std::map<int, Transform> poses;
	std::multimap<int, int> links;   // bidirectional, as Rtabmap builds them
	std::vector<int> ids;            // in the order they were walked
	float length = 0.0f;             // walked arc length, meters
};

Spiral makeSpiral(float radiusStart, float radiusEnd, float pitch, float spacing)
{
	UASSERT(radiusStart > radiusEnd && pitch > 0.0f && spacing > 0.0f);
	Spiral spiral;
	const float b = pitch/(2.0f*M_PI);          // -dr/dtheta
	float theta = 0.0f;
	float r = radiusStart;
	while(r >= radiusEnd)
	{
		const int id = (int)spiral.ids.size()+1;
		// Heading along the tangent, so that the poses are what a robot would have.
		const float tangent = theta + M_PI_2 - std::atan2(b, r);
		spiral.poses.insert(std::make_pair(id,
				Transform(r*std::cos(theta), r*std::sin(theta), 0.0f, 0.0f, 0.0f, tangent)));
		spiral.ids.push_back(id);
		if(id > 1)
		{
			spiral.links.insert(std::make_pair(id-1, id));
			spiral.links.insert(std::make_pair(id, id-1));
			spiral.length += spacing;
		}
		// Arc length ds = sqrt(r^2 + (dr/dtheta)^2) dtheta, stepped by `spacing`.
		theta += spacing/std::sqrt(r*r + b*b);
		r = radiusStart - b*theta;
	}
	return spiral;
}

// The links between two poses closer than `maxDistance` in space but more than
// `minTrajectoryGap` meters apart along the trajectory: the proximity links a session
// would have added between neighboring turns, and the shortcuts that make the
// fewest-links path and the shortest-in-meters path two different paths. The gap is what
// makes them proximity links rather than trajectory ones: two poses a few steps apart are
// within `maxDistance` of each other as well, but linking them adds no shortcut, it just
// short-circuits the chain.
std::multimap<int, int> proximityLinks(
		const Spiral & spiral,
		float maxDistance,
		float minTrajectoryGap = 2.0f,
		int * added = 0)
{
	std::multimap<int, int> links = spiral.links;
	const size_t minStep = (size_t)std::ceil(minTrajectoryGap/SPACING);
	int count = 0;
	for(size_t i=0; i<spiral.ids.size(); ++i)
	{
		const Transform & a = spiral.poses.at(spiral.ids[i]);
		for(size_t j=i+minStep; j<spiral.ids.size(); ++j)
		{
			const Transform & b = spiral.poses.at(spiral.ids[j]);
			if(a.getDistance(b) <= maxDistance)
			{
				links.insert(std::make_pair(spiral.ids[i], spiral.ids[j]));
				links.insert(std::make_pair(spiral.ids[j], spiral.ids[i]));
				++count;
			}
		}
	}
	if(added)
	{
		*added = count;
	}
	return links;
}

// The links as constraints, one per pair (the multimap above holds both directions),
// with the transform the poses give between the two nodes. Only needed to write the
// graph to disk: exportPoses() needs Link objects, the searches only need the ids.
std::multimap<int, Link> constraints(const Spiral & spiral, const std::multimap<int, int> & links)
{
	std::multimap<int, Link> constraints;
	const cv::Mat information = cv::Mat::eye(6, 6, CV_64FC1);
	for(std::multimap<int, int>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
	{
		const int from = iter->first, to = iter->second;
		if(from > to)
		{
			continue;   // the other direction of a pair already written
		}
		const Transform & a = spiral.poses.at(from);
		const Transform & b = spiral.poses.at(to);
		// Consecutive poses are the trajectory, the rest are the proximity detections.
		const Link::Type type = (to == from+1) ? Link::kNeighbor : Link::kLocalSpaceClosure;
		constraints.insert(std::make_pair(from, Link(from, to, type, a.inverse()*b, information)));
	}
	return constraints;
}

// The graph these numbers were measured on, written next to the results so that it can be
// looked at (rtabmap-graphViewer, g2o_viewer) or replayed by another tool. Overwritten on
// every run, under a stable name rather than a pid-suffixed one: the file is there to be
// opened, and makeSpiral() builds the same graph every time anyway.
void saveG2o(const Spiral & spiral, const std::multimap<int, int> & links, const std::string & name)
{
	const std::string path = test::tempPath(uFormat("rtabmap_spiral_%s.g2o", name.c_str()));
	if(graph::exportPoses(path, /*format=*/4, spiral.poses, constraints(spiral, links)))
	{
		std::cout << "[          ]   graph saved to " << path << std::endl;
	}
	else
	{
		std::cout << "[          ]   could not save the graph to " << path << std::endl;
	}
}

// What Rtabmap::process() did before: one A* per candidate, from the last node, and the
// candidate is kept when the path it found is short enough. Returns the ids each path
// walks through, `from` first: its node count is what the old code compared against
// RGBD/ProximityMaxGraphDepth, one more than the depth graph::computePathDepths() gives.
std::map<int, std::vector<std::pair<int, Transform> > > pathsWithAStar(
		const std::map<int, Transform> & poses,
		const std::multimap<int, int> & links,
		int from,
		const std::vector<int> & targets)
{
	std::map<int, std::vector<std::pair<int, Transform> > > paths;
	for(size_t i=0; i<targets.size(); ++i)
	{
		const std::list<std::pair<int, Transform> > path =
				graph::computePath(poses, links, from, targets[i]);
		if(!path.empty())
		{
			// As a vector, which is what the checks below (and graph::computePathLength()) take.
			paths.insert(std::make_pair(targets[i],
					std::vector<std::pair<int, Transform> >(path.begin(), path.end())));
		}
	}
	return paths;
}

// A* needs one search per node, BFS answers for every node in the one search.
void report(size_t nodes, double aStarTime, double bfsTime)
{
	printf("[          ]   A* %8.2f ms (%ld searches), BFS %6.2f ms (1 search), speedup x%.0f\n",
			aStarTime*1000.0, (long)nodes, bfsTime*1000.0,
			bfsTime > 0.0 ? aStarTime/bfsTime : 0.0);
}

// The spirals compared. All of them have a pose every 30 cm and turns 50 cm apart; what
// changes is how far out they start, and so how many nodes they hold.
struct SpiralSize
{
	float radiusStart;
	const char * name;
	const char * fileName;
};
static const SpiralSize SPIRAL_SIZES[] = {
	{2.0f,  "2 m to 1 m",  "2m_to_1m"},
	{5.0f,  "5 m to 1 m",  "5m_to_1m"},
	{10.0f, "10 m to 1 m", "10m_to_1m"},
};
static const size_t SPIRAL_COUNT = sizeof(SPIRAL_SIZES)/sizeof(SPIRAL_SIZES[0]);

}

// The depths of every node of the spiral, from its last node, both ways. The spiral is a
// chain, so there is only one path between two of its nodes and both approaches have to
// agree: the A* path holds one more node than the BFS depth, the start node itself.
TEST(GraphPerfTest, PathDepthsOnSpiral)
{
	for(size_t s=0; s<SPIRAL_COUNT; ++s)
	{
		const Spiral spiral = makeSpiral(SPIRAL_SIZES[s].radiusStart, RADIUS_END, PITCH, SPACING);
		const int from = spiral.ids.back();
		std::cout << "[          ] spiral " << SPIRAL_SIZES[s].name << ", turns "
				<< PITCH << " m apart, a pose every " << SPACING << " m: "
				<< spiral.ids.size() << " nodes, " << spiral.length << " m walked, depths from "
				<< from << " (the innermost pose) to all of them" << std::endl;
		saveG2o(spiral, spiral.links, SPIRAL_SIZES[s].fileName);

		UTimer timer;
		const std::map<int, std::vector<std::pair<int, Transform> > > aStarPaths =
				pathsWithAStar(spiral.poses, spiral.links, from, spiral.ids);
		const double aStarTime = timer.ticks();

		const std::map<int, int> depths = graph::computePathDepths(spiral.links, from);
		const double bfsTime = timer.ticks();

		report(spiral.ids.size(), aStarTime, bfsTime);

		ASSERT_EQ(depths.size(), spiral.ids.size());
		ASSERT_EQ(aStarPaths.size(), spiral.ids.size());
		EXPECT_EQ(depths.at(from), 0);
		for(size_t i=0; i<spiral.ids.size(); ++i)
		{
			const int id = spiral.ids[i];
			// The chain gives the depth in closed form: the number of links back to `from`.
			EXPECT_EQ(depths.at(id), from-id) << "node " << id;

			// Same path, not only the same count: the only way from `from` to `id` walks the
			// chain, and A* walks it node by node, each step one deeper than the one before.
			const std::vector<std::pair<int, Transform> > & path = aStarPaths.at(id);
			ASSERT_EQ((int)path.size(), depths.at(id)+1) << "node " << id;
			for(size_t j=0; j<path.size(); ++j)
			{
				ASSERT_EQ(path[j].first, from-(int)j) << "node " << id << ", step " << j;
				ASSERT_EQ(depths.at(path[j].first), (int)j) << "node " << id << ", step " << j;
			}
		}
	}
}

// The same spiral once the proximity links between neighboring turns are added, which is
// what the graph looks like after a session closed on itself. Beyond the timings, this is
// where the two approaches stop answering the same thing: A* minimizes meters, so the path
// it returns is not always the one with the fewest links, and the node count it reports is
// then larger than the depth. Rejecting candidates on it rejected some that were within
// RGBD/ProximityMaxGraphDepth links of the current node.
TEST(GraphPerfTest, PathDepthsOnSpiralWithProximityLinks)
{
	for(size_t s=0; s<SPIRAL_COUNT; ++s)
	{
		const Spiral spiral = makeSpiral(SPIRAL_SIZES[s].radiusStart, RADIUS_END, PITCH, SPACING);
		int added = 0;
		const std::multimap<int, int> links = proximityLinks(spiral, /*maxDistance=*/0.6f,
				/*minTrajectoryGap=*/2.0f, &added);
		const int from = spiral.ids.back();
		std::cout << "[          ] spiral " << SPIRAL_SIZES[s].name << ": " << spiral.ids.size()
				<< " nodes, " << added << " proximity links added between turns" << std::endl;
		saveG2o(spiral, links, uFormat("%s_proximity", SPIRAL_SIZES[s].fileName));

		UTimer timer;
		const std::map<int, std::vector<std::pair<int, Transform> > > aStarPaths =
				pathsWithAStar(spiral.poses, links, from, spiral.ids);
		const double aStarTime = timer.ticks();

		const std::map<int, int> depths = graph::computePathDepths(links, from);
		const double bfsTime = timer.ticks();

		report(spiral.ids.size(), aStarTime, bfsTime);

		ASSERT_EQ(depths.size(), spiral.ids.size());
		ASSERT_EQ(aStarPaths.size(), spiral.ids.size());
		int overestimated = 0, maxOverestimation = 0;
		for(size_t i=0; i<spiral.ids.size(); ++i)
		{
			const int id = spiral.ids[i];
			const int over = (int)aStarPaths.at(id).size() - (depths.at(id)+1);
			// A* cannot beat the BFS depth, it can only walk more links to save meters.
			EXPECT_GE(over, 0) << "node " << id;
			if(over > 0)
			{
				++overestimated;
				maxOverestimation = std::max(maxOverestimation, over);
			}
		}
		printf("[          ]   A* counted more links than the depth on %d of the %ld nodes"
				" (up to %d more)\n", overestimated, (long)spiral.ids.size(), maxOverestimation);
	}
}
