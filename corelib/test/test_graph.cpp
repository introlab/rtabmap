#include <gtest/gtest.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Link.h>
#include <cmath>
#include <string>
#include <vector>

using namespace rtabmap;

namespace {

static cv::Mat infMatrixDiagonal(double x, double y, double z, double roll, double pitch, double yaw)
{
	cv::Mat inf = cv::Mat::zeros(6, 6, CV_64FC1);
	inf.at<double>(0, 0) = x;
	inf.at<double>(1, 1) = y;
	inf.at<double>(2, 2) = z;
	inf.at<double>(3, 3) = roll;
	inf.at<double>(4, 4) = pitch;
	inf.at<double>(5, 5) = yaw;
	return inf;
}

static Link neighborLink(int from, int to, float dx = 1.0f)
{
	return Link(from, to, Link::kNeighbor, Transform(dx, 0, 0, 0, 0, 0));
}

static void insertLink(std::multimap<int, Link> & links, const Link & link)
{
	links.insert(std::make_pair(link.from(), link));
}

static std::map<int, Transform> linePoses(unsigned int count, float step = 1.0f)
{
	std::map<int, Transform> poses;
	for(unsigned int i = 0; i < count; ++i)
	{
		poses.insert(std::make_pair(static_cast<int>(i + 1), Transform(step * i, 0, 0, 0, 0, 0)));
	}
	return poses;
}

// KITTI metrics use 100–800 m segments; need ~800 m of trajectory at 1 m/frame.
static std::vector<Transform> lineTrajectory(unsigned int count, float step = 1.0f)
{
	std::vector<Transform> traj;
	traj.reserve(count);
	for(unsigned int i = 0; i < count; ++i)
	{
		traj.push_back(Transform(step * i, 0, 0, 0, 0, 0));
	}
	return traj;
}

static std::map<int, Transform> transformPoses(
		const std::map<int, Transform> & poses,
		const Transform & t)
{
	std::map<int, Transform> out;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter)
	{
		out.insert(std::make_pair(iter->first, t * iter->second));
	}
	return out;
}

static float calcTranslationalRmse(
		const std::map<int, Transform> & groundTruth,
		const std::map<int, Transform> & poses,
		bool align2D = true)
{
	float tRmse = 0.0f;
	float tMean = 0.0f;
	float tMedian = 0.0f;
	float tStd = 0.0f;
	float tMin = 0.0f;
	float tMax = 0.0f;
	float rRmse = 0.0f;
	float rMean = 0.0f;
	float rMedian = 0.0f;
	float rStd = 0.0f;
	float rMin = 0.0f;
	float rMax = 0.0f;
	graph::calcRMSE(
			groundTruth,
			poses,
			tRmse,
			tMean,
			tMedian,
			tStd,
			tMin,
			tMax,
			rRmse,
			rMean,
			rMedian,
			rStd,
			rMin,
			rMax,
			align2D);
	return tRmse;
}

} // namespace

TEST(GraphTest, FindLinkForwardAndReverse)
{
	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2));

	EXPECT_NE(graph::findLink(links, 1, 2), links.end());
	EXPECT_EQ(graph::findLink(links, 1, 2)->second.to(), 2);

	EXPECT_EQ(graph::findLink(links, 2, 1, false), links.end());
	EXPECT_NE(graph::findLink(links, 2, 1, true), links.end());

	EXPECT_EQ(graph::findLink(links, 1, 2, true, Link::kGlobalClosure), links.end());
	EXPECT_NE(graph::findLink(links, 1, 2, true, Link::kNeighbor), links.end());
}

TEST(GraphTest, FindLinkIntMultimap)
{
	std::multimap<int, int> links;
	links.insert(std::make_pair(1, 2));
	links.insert(std::make_pair(2, 3));

	EXPECT_NE(graph::findLink(links, 1, 2), links.end());
	EXPECT_NE(graph::findLink(links, 3, 2, true), links.end());
	EXPECT_EQ(graph::findLink(links, 1, 3), links.end());
}

TEST(GraphTest, FindLinksIncludesIncomingAsInverse)
{
	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2));

	const std::list<Link> from1 = graph::findLinks(links, 1);
	ASSERT_EQ(from1.size(), 1u);
	EXPECT_EQ(from1.front().from(), 1);
	EXPECT_EQ(from1.front().to(), 2);

	const std::list<Link> from2 = graph::findLinks(links, 2);
	ASSERT_EQ(from2.size(), 1u);
	EXPECT_EQ(from2.front().from(), 2);
	EXPECT_EQ(from2.front().to(), 1);
}

TEST(GraphTest, FilterDuplicateLinks)
{
	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2));
	insertLink(links, neighborLink(1, 2));
	insertLink(links, neighborLink(2, 1));

	const std::multimap<int, Link> filtered = graph::filterDuplicateLinks(links);
	EXPECT_EQ(filtered.size(), 1u);
}

TEST(GraphTest, FilterLinksByType)
{
	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2));
	insertLink(links, Link(2, 3, Link::kGlobalClosure, Transform::getIdentity()));

	const std::multimap<int, Link> noClosure = graph::filterLinks(links, Link::kGlobalClosure, false);
	EXPECT_EQ(noClosure.size(), 1u);
	EXPECT_EQ(noClosure.begin()->second.type(), Link::kNeighbor);

	const std::multimap<int, Link> onlyClosure = graph::filterLinks(links, Link::kGlobalClosure, true);
	ASSERT_EQ(onlyClosure.size(), 1u);
	EXPECT_EQ(onlyClosure.begin()->second.type(), Link::kGlobalClosure);
}

TEST(GraphTest, FilterSelfReferenceLinks)
{
	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2));
	insertLink(links, Link(3, 3, Link::kPosePrior, Transform::getIdentity()));

	const std::multimap<int, Link> nonSelf = graph::filterLinks(links, Link::kSelfRefLink, false);
	EXPECT_EQ(nonSelf.size(), 1u);
	EXPECT_NE(nonSelf.begin()->second.from(), nonSelf.begin()->second.to());

	const std::multimap<int, Link> selfOnly = graph::filterLinks(links, Link::kSelfRefLink, true);
	ASSERT_EQ(selfOnly.size(), 1u);
	EXPECT_EQ(selfOnly.begin()->second.from(), selfOnly.begin()->second.to());
}

static std::list<int> computeDijkstraPath(bool updateNewCosts, bool useSameCostForAllLinks)
{
	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2));
	insertLink(links, neighborLink(2, 3));
	insertLink(links, neighborLink(1, 3, 5.0f));
	return graph::computePath(links, 1, 3, updateNewCosts, useSameCostForAllLinks);
}

static std::string dijkstraPathToString(const std::list<int> & path)
{
	std::string s;
	for(std::list<int>::const_iterator it = path.begin(); it != path.end(); ++it)
	{
		if(!s.empty())
		{
			s += "->";
		}
		s += std::to_string(*it);
	}
	return s;
}

static void expectDijkstraPath(
		const std::list<int> & path,
		const std::initializer_list<int> expected)
{
	ASSERT_EQ(path.size(), expected.size());
	auto it = path.begin();
	for(int id : expected)
	{
		ASSERT_NE(it, path.end());
		EXPECT_EQ(*it, id);
		++it;
	}
}

TEST(GraphTest, ComputePathDijkstraWeighted)
{
	// Same graph as computeDijkstraPath(); edge cost = link length (m).
	//
	//   1 -------- 5 m -------- 3   cost 5  ->  {1, 3} when updateNewCosts=false
	//   |                       ^
	//   +-- 1 m -- 2 -- 1 m ----+   cost 2  ->  {1, 2, 3} when updateNewCosts=true
	//
	expectDijkstraPath(computeDijkstraPath(false, false), {1, 3}); // updateNewCosts=false: 3 stays on direct link
	expectDijkstraPath(computeDijkstraPath(true, false), {1, 2, 3});
}

TEST(GraphTest, ComputePathDijkstraUnitCostUpdateNewCostsEquivalent)
{
	// Same topology; useSameCostForAllLinks=true (1 hop per edge).
	//
	//   1 --------------------- 3   1 hop  ->  {1, 3} (both updateNewCosts values)
	//   |
	//   +-- 1 hop -- 2 -- 1 hop -- 3   2 hops (never chosen)
	//
	// Relaxation only applies on a strictly lower hop count, so updateNewCosts cannot
	// change the result.
	expectDijkstraPath(computeDijkstraPath(false, true), {1, 3});
	expectDijkstraPath(computeDijkstraPath(true, true), {1, 3});
	EXPECT_EQ(
			dijkstraPathToString(computeDijkstraPath(false, true)),
			dijkstraPathToString(computeDijkstraPath(true, true)));
}

TEST(GraphTest, ComputePathAStar)
{
	std::map<int, Transform> poses = linePoses(3);
	std::multimap<int, int> links;
	links.insert(std::make_pair(1, 2));
	links.insert(std::make_pair(2, 3));

	const std::list<std::pair<int, Transform> > path = graph::computePath(poses, links, 1, 3, false);
	ASSERT_EQ(path.size(), 3u);
	EXPECT_EQ(path.front().first, 1);
	EXPECT_EQ(path.back().first, 3);
}

static std::string aStarPathToString(const std::list<std::pair<int, Transform> > & path)
{
	std::string s;
	for(std::list<std::pair<int, Transform> >::const_iterator it = path.begin(); it != path.end(); ++it)
	{
		if(it != path.begin())
		{
			s += "->";
		}
		s += std::to_string(it->first);
	}
	return s;
}

static void expectAStarPath(
		const std::list<std::pair<int, Transform> > & path,
		const std::initializer_list<int> expectedIds)
{
	ASSERT_EQ(path.size(), expectedIds.size()) << "path=" << aStarPathToString(path);
	auto it = path.begin();
	for(int id : expectedIds)
	{
		ASSERT_NE(it, path.end());
		EXPECT_EQ(it->first, id);
		++it;
	}
}

static std::list<std::pair<int, Transform> > computeAStarPath(
		const std::map<int, Transform> & poses,
		const std::multimap<int, int> & links,
		bool updateNewCosts)
{
	return graph::computePath(poses, links, 1, 3, updateNewCosts);
}

TEST(GraphTest, ComputePathAStarUpdateNewCostsChangesPath)
{
	// Detour 1→2→5→6→3 vs shortcut 1→2→4→6→3. No 5→3 edge so h(5,3) < cost(5→6→3).
	// Node 5 is expanded before 4 (lower f-score). Node 6 is first reached from 5;
	// expanding 4 relaxes the parent of 6 when updateNewCosts=true.
	//
	//        3  goal (10, 0)
	//        |
	//        6  (2, -5)
	//       / \
	//      5   4   (1,-2)  (1.5,-4)
	//       \ /
	//        2  (1, 0)
	//        |
	//        1  start (0, 0)
	//
	// Links: 1—2, 2—5, 2—4, 5—6, 4—6, 6—3  (no 5—3)
	const std::map<int, Transform> poses = {
		{1, Transform(0, 0, 0, 0, 0, 0)},
		{2, Transform(1, 0, 0, 0, 0, 0)},
		{3, Transform(10, 0, 0, 0, 0, 0)},
		{4, Transform(1.5f, -4, 0, 0, 0, 0)},
		{5, Transform(1, -2, 0, 0, 0, 0)},
		{6, Transform(2, -5, 0, 0, 0, 0)}};
	std::multimap<int, int> links;
	links.insert(std::make_pair(1, 2));
	links.insert(std::make_pair(2, 5)); // before 2→4
	links.insert(std::make_pair(2, 4));
	links.insert(std::make_pair(5, 6));
	links.insert(std::make_pair(4, 6));
	links.insert(std::make_pair(6, 3));

	const std::list<std::pair<int, Transform> > pathNoUpdate =
			computeAStarPath(poses, links, false);
	const std::list<std::pair<int, Transform> > pathUpdate =
			computeAStarPath(poses, links, true);

	expectAStarPath(pathNoUpdate, {1, 2, 5, 6, 3});
	expectAStarPath(pathUpdate, {1, 2, 4, 6, 3});
	EXPECT_NE(aStarPathToString(pathNoUpdate), aStarPathToString(pathUpdate));
}

TEST(GraphTest, FindNearestNode)
{
	const std::map<int, Transform> poses = linePoses(3, 2.0f);
	const Transform query(2.1f, 0.1f, 0, 0, 0, 0);

	float sqDist = -1.0f;
	const int id = graph::findNearestNode(poses, query, &sqDist);
	EXPECT_EQ(id, 2);
	EXPECT_NEAR(sqDist, 0.1f * 0.1f + 0.1f * 0.1f, 1e-4f);
}

TEST(GraphTest, FindNearestNodesKnn)
{
	const std::map<int, Transform> poses = linePoses(4);
	const std::map<int, float> nearest = graph::findNearestNodes(poses.at(2), poses, 0.0f, 0.0f, 2);
	ASSERT_EQ(nearest.size(), 2u);
	ASSERT_TRUE(nearest.find(2) != nearest.end());
	EXPECT_NEAR(nearest.at(2), 0.0f, 1e-6f);

	// nodeId overload excludes the query node from results
	const std::map<int, float> excludingSelf = graph::findNearestNodes(2, poses, 0.0f, 0.0f, 2);
	ASSERT_EQ(excludingSelf.size(), 2u);
	EXPECT_TRUE(excludingSelf.find(2) == excludingSelf.end());
	EXPECT_NEAR(excludingSelf.at(1), 1.0f, 1e-6f);
	EXPECT_NEAR(excludingSelf.at(3), 1.0f, 1e-6f);
}

TEST(GraphTest, FindNearestNodesRadius)
{
	const std::map<int, Transform> poses = linePoses(4);
	const std::map<int, float> inRadius = graph::findNearestNodes(poses.at(2), poses, 1.5f);
	EXPECT_EQ(inRadius.size(), 3u);
	EXPECT_TRUE(inRadius.find(2) != inRadius.end());
	EXPECT_NEAR(inRadius.at(2), 0.0f, 1e-6f);
	EXPECT_TRUE(inRadius.find(4) == inRadius.end());
}

TEST(GraphTest, ComputePathLength)
{
	const std::vector<std::pair<int, Transform> > vecPath = {
		{1, Transform(0, 0, 0, 0, 0, 0)},
		{2, Transform(3, 4, 0, 0, 0, 0)},
		{3, Transform(3, 9, 0, 0, 0, 0)}};
	EXPECT_NEAR(graph::computePathLength(vecPath), 10.0f, 1e-4f);

	const std::map<int, Transform> mapPath = linePoses(3);
	EXPECT_NEAR(graph::computePathLength(mapPath), 2.0f, 1e-4f);
}

TEST(GraphTest, ComputeMinMax)
{
	const std::map<int, Transform> poses = {
		{1, Transform(-1, 2, 3, 0, 0, 0)},
		{2, Transform(4, -5, 0, 0, 0, 0)}};

	cv::Vec3f min, max;
	graph::computeMinMax(poses, min, max);
	EXPECT_FLOAT_EQ(min[0], -1.0f);
	EXPECT_FLOAT_EQ(min[1], -5.0f);
	EXPECT_FLOAT_EQ(min[2], 0.0f);
	EXPECT_FLOAT_EQ(max[0], 4.0f);
	EXPECT_FLOAT_EQ(max[1], 2.0f);
	EXPECT_FLOAT_EQ(max[2], 3.0f);
}

TEST(GraphTest, CalcRelativeErrorsIdenticalTrajectories)
{
	const std::vector<Transform> traj = {
		Transform(0, 0, 0, 0, 0, 0),
		Transform(1, 0, 0, 0, 0, 0),
		Transform(2, 0, 0, 0, 0, 0)};

	float tErr = -1.0f;
	float rErr = -1.0f;
	graph::calcRelativeErrors(traj, traj, tErr, rErr);
	EXPECT_NEAR(tErr, 0.0f, 1e-5f);
	EXPECT_NEAR(rErr, 0.0f, 1e-5f);
}

TEST(GraphTest, CalcRelativeErrorsWithNoise)
{
	const std::vector<Transform> gt = {
		Transform(0, 0, 0, 0, 0, 0),
		Transform(1, 0, 0, 0, 0, 0),
		Transform(2, 0, 0, 0, 0, 0),
		Transform(3, 0, 0, 0, 0, 0),
		Transform(4, 0, 0, 0, 0, 0)};

	// Small position and orientation noise on the estimate.
	const std::vector<Transform> est = {
		Transform(0.01f, -0.02f, 0.005f, 0, 0, 0.01f),
		Transform(1.03f, 0.01f, -0.01f, 0, 0, -0.02f),
		Transform(2.02f, -0.03f, 0.02f, 0, 0, 0.015f),
		Transform(2.98f, 0.02f, 0.01f, 0, 0, -0.01f),
		Transform(4.01f, -0.01f, -0.02f, 0, 0, 0.005f)};

	float tErr = 0.0f;
	float rErr = 0.0f;
	graph::calcRelativeErrors(gt, est, tErr, rErr);
	EXPECT_GT(tErr, 0.0f);
	EXPECT_LT(tErr, 0.1f);
	EXPECT_GT(rErr, 0.0f);
	EXPECT_LT(rErr, 2.0f);
}

TEST(GraphTest, CalcKittiSequenceErrorsIdenticalTrajectories)
{
	const std::vector<Transform> traj = lineTrajectory(901, 1.0f);

	float tErr = -1.0f;
	float rErr = -1.0f;
	graph::calcKittiSequenceErrors(traj, traj, tErr, rErr);
	EXPECT_NEAR(tErr, 0.0f, 1e-5f);
	EXPECT_NEAR(rErr, 0.0f, 1e-5f);
}

TEST(GraphTest, CalcKittiSequenceErrorsWithNoise)
{
	const std::vector<Transform> gt = lineTrajectory(901, 1.0f);
	ASSERT_EQ(gt.size(), 901u);
	ASSERT_NEAR(gt[0].x(), 0.0f, 1e-5f);

	std::vector<Transform> est;
	est.reserve(gt.size());
	for(unsigned int i = 0; i < gt.size(); ++i)
	{
		const int ii = static_cast<int>(i);
		const float dx = 0.02f * static_cast<float>((ii % 3) - 1);
		const float dy = 0.01f * static_cast<float>((ii % 5) - 2);
		est.push_back(Transform(
				gt.at(i).x() + dx,
				gt.at(i).y() + dy,
				gt.at(i).z(),
				0.0f,
				0.0f,
				0.0f));
	}
	ASSERT_EQ(est.size(), 901u);
	ASSERT_NEAR(est.at(0).x(), -0.02f, 1e-3f);
	ASSERT_NEAR(est.at(800).x(), 800.02f, 1e-1f);

	const Transform poseDeltaEst = est.at(0).inverse() * est.at(800);
	ASSERT_NEAR(poseDeltaEst.getNorm(), 800.0f, 5.0f);

	float tErr = 0.0f;
	float rErr = 0.0f;
	graph::calcKittiSequenceErrors(gt, est, tErr, rErr);
	EXPECT_TRUE(std::isfinite(tErr)) << "tErr=" << tErr;
	EXPECT_TRUE(std::isfinite(rErr)) << "rErr=" << rErr;
	EXPECT_GT(tErr, 0.0f);
	EXPECT_LT(tErr, 2.0f);   // translation error (%)
	EXPECT_NEAR(rErr, 0.0f, 0.5f); // no orientation noise on the trajectory
}

TEST(GraphTest, CalcRMSEIdenticalMaps)
{
	const std::map<int, Transform> gt = linePoses(3);
	float tRmse = -1.0f;
	float tMean = -1.0f;
	float tMedian = -1.0f;
	float tStd = -1.0f;
	float tMin = -1.0f;
	float tMax = -1.0f;
	float rRmse = -1.0f;
	float rMean = -1.0f;
	float rMedian = -1.0f;
	float rStd = -1.0f;
	float rMin = -1.0f;
	float rMax = -1.0f;

	const Transform align = graph::calcRMSE(
			gt, gt,
			tRmse, tMean, tMedian, tStd, tMin, tMax,
			rRmse, rMean, rMedian, rStd, rMin, rMax,
			true);
	EXPECT_TRUE(align.isIdentity());
	EXPECT_NEAR(tRmse, 0.0f, 1e-4f);
	EXPECT_NEAR(rRmse, 0.0f, 1e-4f);
}

TEST(GraphTest, CalcRMSEWithNoise)
{
	const std::map<int, Transform> gt = linePoses(8, 1.0f);
	std::map<int, Transform> est = gt;
	est[2] = Transform(1.05f, 0.02f, 0, 0, 0, 0.01f);
	est[4] = Transform(3.02f, -0.03f, 0.01f, 0, 0, -0.02f);
	est[6] = Transform(5.01f, 0.01f, -0.02f, 0, 0, 0.015f);
	est[8] = Transform(7.0f, -0.01f, 0.02f, 0, 0, -0.005f);

	const float tRmse = calcTranslationalRmse(gt, est, true);
	EXPECT_GT(tRmse, 0.0f);
	EXPECT_LT(tRmse, 0.1f);
}

TEST(GraphTest, CalcRMSEAlignsRotatedTrajectory)
{
	// Eight poses so calcRMSE uses SVD alignment (more than five matched poses).
	const std::map<int, Transform> gt = linePoses(8, 1.0f);

	std::map<int, Transform> noisy = gt;
	noisy[2] = Transform(1.05f, 0.02f, 0, 0, 0, 0.01f);
	noisy[5] = Transform(4.02f, -0.02f, 0, 0, 0, -0.01f);
	noisy[7] = Transform(6.01f, 0.01f, 0, 0, 0, 0.02f);

	const Transform yaw90(0, 0, 0, 0, 0, static_cast<float>(CV_PI / 2.0));
	const std::map<int, Transform> rotated = transformPoses(gt, yaw90);

	// Without alignment, positions would differ a lot (x vs y).
	const Transform p = gt.at(4);
	const Transform r = rotated.at(4);
	EXPECT_GT(p.getDistance(r), 1.0f);

	const float rmseNoisy = calcTranslationalRmse(gt, noisy, true);

	float tRmse = 0.0f;
	float tMean = 0.0f;
	float tMedian = 0.0f;
	float tStd = 0.0f;
	float tMin = 0.0f;
	float tMax = 0.0f;
	float rRmse = 0.0f;
	float rMean = 0.0f;
	float rMedian = 0.0f;
	float rStd = 0.0f;
	float rMin = 0.0f;
	float rMax = 0.0f;
	const Transform align = graph::calcRMSE(
			gt,
			rotated,
			tRmse,
			tMean,
			tMedian,
			tStd,
			tMin,
			tMax,
			rRmse,
			rMean,
			rMedian,
			rStd,
			rMin,
			rMax,
			true);

	EXPECT_LT(rmseNoisy, 0.1f);
	EXPECT_LT(tRmse, 0.1f);
	EXPECT_NEAR(tRmse, rmseNoisy, 0.08f);

	// est = yaw90 * gt  =>  align * est ≈ gt  =>  align ≈ yaw90⁻¹
	const Transform expectedAlign = yaw90.inverse();
	EXPECT_NEAR(align.getAngle(expectedAlign), 0.0f, 0.05f);
	EXPECT_NEAR(align.x(), expectedAlign.x(), 1e-2f);
	EXPECT_NEAR(align.y(), expectedAlign.y(), 1e-2f);
	EXPECT_NEAR(align.theta(), expectedAlign.theta(), 1e-2f);
	EXPECT_LT((align * rotated.at(4)).getDistance(gt.at(4)), 1e-2f);
}

static graph::MaxGraphErrors maxGraphErrors(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		bool for3DoF = false)
{
	return graph::computeMaxGraphErrors(poses, links, for3DoF);
}

TEST(GraphTest, ComputeMaxGraphErrorsZeroResidual)
{
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform(0, 0, 0, 0, 0, 0)));
	poses.insert(std::make_pair(2, Transform(1, 0, 0, 0, 0, 0)));

	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2, 1.0f));

	const graph::MaxGraphErrors errors = maxGraphErrors(poses, links);
	EXPECT_NEAR(errors.linear, 0.0f, 1e-4f);
	EXPECT_NEAR(errors.angular, 0.0f, 1e-4f);
	EXPECT_NEAR(errors.linearRatio, 0.0f, 1e-4f);
	EXPECT_NEAR(errors.angularRatio, 0.0f, 1e-4f);
	EXPECT_TRUE(errors.linearLink.isValid());
	EXPECT_EQ(errors.linearLink.from(), 1);
	EXPECT_EQ(errors.linearLink.to(), 2);
}

TEST(GraphTest, ComputeMaxGraphErrorsKnownLinearResidual)
{
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform(0, 0, 0, 0, 0, 0)));
	poses.insert(std::make_pair(2, Transform(2, 0, 0, 0, 0, 0))); // 2 m apart

	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2, 1.0f)); // link says 1 m

	const graph::MaxGraphErrors errors = maxGraphErrors(poses, links);
	EXPECT_NEAR(errors.linear, 1.0f, 1e-4f);
	EXPECT_NEAR(errors.linearRatio, 1.0f, 1e-4f); // default inf: variance 1, stddev 1
	EXPECT_TRUE(errors.linearLink.isValid());
	EXPECT_EQ(errors.linearLink.from(), 1);
	EXPECT_EQ(errors.linearLink.to(), 2);
}

TEST(GraphTest, ComputeMaxGraphErrorsPicksHighestLinearRatio)
{
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform(0, 0, 0, 0, 0, 0)));
	poses.insert(std::make_pair(2, Transform(0.5f, 0, 0, 0, 0, 0))); // 0.5 m error vs link
	poses.insert(std::make_pair(3, Transform(2.5f, 0, 0, 0, 0, 0))); // 1 m error vs link

	std::multimap<int, Link> links;
	insertLink(links, Link(
			1,
			2,
			Link::kNeighbor,
			Transform(0, 0, 0, 0, 0, 0),
			infMatrixDiagonal(100, 100, 100, 1, 1, 1))); // ratio ≈ 0.5 / 0.1 = 5
	insertLink(links, neighborLink(2, 3, 1.0f)); // ratio ≈ 1 / 1 = 1

	const graph::MaxGraphErrors errors = maxGraphErrors(poses, links);
	EXPECT_NEAR(errors.linear, 0.5f, 1e-4f);
	EXPECT_NEAR(errors.linearRatio, 5.0f, 1e-3f);
	EXPECT_EQ(errors.linearLink.from(), 1);
	EXPECT_EQ(errors.linearLink.to(), 2);
}

TEST(GraphTest, ComputeMaxGraphErrorsAngularResidual)
{
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform(0, 0, 0, 0, 0, 0)));
	poses.insert(std::make_pair(2, Transform(1, 0, 0, 0, 0, 0.5f)));

	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2, 1.0f));

	const graph::MaxGraphErrors errors = maxGraphErrors(poses, links);
	EXPECT_NEAR(errors.linear, 0.0f, 1e-4f);
	EXPECT_NEAR(errors.angular, 0.5f, 1e-4f);
	EXPECT_NEAR(errors.angularRatio, 0.5f, 1e-4f);
	EXPECT_EQ(errors.angularLink.from(), 1);
	EXPECT_EQ(errors.angularLink.to(), 2);
}

TEST(GraphTest, ComputeMaxGraphErrorsDifferentWorstLinearAndAngularLinks)
{
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform(0, 0, 0, 0, 0, 0)));
	poses.insert(std::make_pair(2, Transform(0.6f, 0, 0, 0, 0, 0)));
	poses.insert(std::make_pair(3, Transform(1.6f, 0, 0, 0, 0, 0.5f)));

	std::multimap<int, Link> links;
	insertLink(links, Link(
			1,
			2,
			Link::kNeighbor,
			Transform(1, 0, 0, 0, 0, 0),
			infMatrixDiagonal(100, 100, 100, 1, 1, 1)));
	insertLink(links, Link(
			2,
			3,
			Link::kNeighbor,
			Transform(1, 0, 0, 0, 0, 0),
			infMatrixDiagonal(1, 1, 1, 100, 100, 100)));

	const graph::MaxGraphErrors errors = maxGraphErrors(poses, links);
	EXPECT_EQ(errors.linearLink.from(), 1);
	EXPECT_EQ(errors.linearLink.to(), 2);
	EXPECT_EQ(errors.angularLink.from(), 2);
	EXPECT_EQ(errors.angularLink.to(), 3);
	EXPECT_NE(errors.linearLink.from(), errors.angularLink.from());
}

TEST(GraphTest, ComputeMaxGraphErrorsFor3DoF)
{
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform(0, 0, 0, 0, 0, 0)));
	poses.insert(std::make_pair(2, Transform(1, 0, 0.3f, 0, 0, 0)));

	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2, 1.0f));

	const graph::MaxGraphErrors errors6 = maxGraphErrors(poses, links, false);
	const graph::MaxGraphErrors errors3 = maxGraphErrors(poses, links, true);
	EXPECT_NEAR(errors6.linear, 0.3f, 1e-4f);
	EXPECT_NEAR(errors3.linear, 0.0f, 1e-4f);
}

TEST(GraphTest, ComputeMaxGraphErrorsSkipsSelfLinks)
{
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform(0, 0, 0, 0, 0, 0)));

	std::multimap<int, Link> links;
	insertLink(links, Link(1, 1, Link::kPosePrior, Transform(1, 2, 3, 0, 0, 0)));

	const graph::MaxGraphErrors errors = maxGraphErrors(poses, links);
	EXPECT_FLOAT_EQ(errors.linear, -1.0f);
	EXPECT_FLOAT_EQ(errors.angular, -1.0f);
	EXPECT_FALSE(errors.linearLink.isValid());
}

TEST(GraphTest, ComputeMaxGraphErrorsAbortsOnMissingPose)
{
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform(0, 0, 0, 0, 0, 0)));

	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2, 1.0f));

	const graph::MaxGraphErrors errors = maxGraphErrors(poses, links);
	EXPECT_FLOAT_EQ(errors.linear, -1.0f);
	EXPECT_FLOAT_EQ(errors.angular, -1.0f);
	EXPECT_FALSE(errors.linearLink.isValid());
}

TEST(GraphTest, ComputeMaxGraphErrorsLandmarkSkipsUnconstrainedYaw)
{
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform(0, 0, 0, 0, 0, 0)));
	poses.insert(std::make_pair(2, Transform(1, 0, 0, 0, 0, 0.1f))); // 0.1 rad yaw vs neighbor link
	poses.insert(std::make_pair(-10, Transform(2, 1, 0, 0, 0, 0.5f))); // 1 m y and 0.5 rad yaw vs landmark link

	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2, 1.0f));
	// Landmark yaw is not constrained; angular error is skipped even though poses disagree by 0.5 rad.
	insertLink(links, Link(
			1,
			-10,
			Link::kLandmark,
			Transform(2, 0, 0, 0, 0, 0),
			infMatrixDiagonal(1, 1, 1, 1, 1, 0.00001)));

	const graph::MaxGraphErrors errors = maxGraphErrors(poses, links);
	EXPECT_NEAR(errors.linear, 1.0f, 1e-4f);
	EXPECT_EQ(errors.linearLink.from(), 1);
	EXPECT_EQ(errors.linearLink.to(), -10);
	EXPECT_NEAR(errors.angular, 0.1f, 1e-4f);
	EXPECT_EQ(errors.angularLink.from(), 1);
	EXPECT_EQ(errors.angularLink.to(), 2);
	EXPECT_NE(errors.angularLink.type(), Link::kLandmark);
}

TEST(GraphTest, ComputeMaxGraphErrorsLandmarkTwoPoseObservations)
{
	// Same landmark -10 observed from poses 1 and 2 (two links sharing the landmark id).
	//
	//     -10  (1, 1)
	//    /   \
	//   1     2
	// (0,0) (2,0)
	//
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform(0, 0, 0, 0, 0, 0)));
	poses.insert(std::make_pair(2, Transform(2, 0, 0, 0, 0, 0)));
	poses.insert(std::make_pair(-10, Transform(1, 1, 0, 0, 0, 0)));

	std::multimap<int, Link> links;
	insertLink(links, Link(1, -10, Link::kLandmark, Transform(1, 1, 0, 0, 0, 0)));
	insertLink(links, Link(2, -10, Link::kLandmark, Transform(-1, 1, 0, 0, 0, 0)));

	const graph::MaxGraphErrors consistent = maxGraphErrors(poses, links);
	EXPECT_NEAR(consistent.linear, 0.0f, 1e-4f);
	EXPECT_NEAR(consistent.angular, 0.0f, 1e-4f);

	// Poses still agree with observation from 1; link 2→-10 is wrong by 1 m in y.
	std::multimap<int, Link> linksOneInconsistent;
	insertLink(linksOneInconsistent, Link(1, -10, Link::kLandmark, Transform(1, 1, 0, 0, 0, 0)));
	insertLink(linksOneInconsistent, Link(2, -10, Link::kLandmark, Transform(-1, 0, 0, 0, 0, 0)));
	const graph::MaxGraphErrors oneInconsistent = maxGraphErrors(poses, linksOneInconsistent);
	EXPECT_NEAR(oneInconsistent.linear, 1.0f, 1e-4f);
	EXPECT_EQ(oneInconsistent.linearLink.from(), 2);
	EXPECT_EQ(oneInconsistent.linearLink.to(), -10);

	// Same mismatch with landmark as link.from (from < 0): measurement is inverted.
	std::multimap<int, Link> linksOneInconsistentLandmarkFrom;
	insertLink(linksOneInconsistentLandmarkFrom, Link(-10, 1, Link::kLandmark, Transform(-1, -1, 0, 0, 0, 0)));
	insertLink(linksOneInconsistentLandmarkFrom, Link(-10, 2, Link::kLandmark, Transform(1, 0, 0, 0, 0, 0)));
	const graph::MaxGraphErrors oneInconsistentLandmarkFrom =
			maxGraphErrors(poses, linksOneInconsistentLandmarkFrom);
	EXPECT_NEAR(oneInconsistentLandmarkFrom.linear, oneInconsistent.linear, 1e-4f);
	EXPECT_NEAR(oneInconsistentLandmarkFrom.linearRatio, oneInconsistent.linearRatio, 1e-4f);
	EXPECT_EQ(oneInconsistentLandmarkFrom.linearLink.from(), -10);
	EXPECT_EQ(oneInconsistentLandmarkFrom.linearLink.to(), 2);

	// Move the optimized landmark; both observations are now inconsistent by 1 m in y.
	poses[-10] = Transform(1, 0, 0, 0, 0, 0);
	const graph::MaxGraphErrors inconsistent = maxGraphErrors(poses, links);
	EXPECT_NEAR(inconsistent.linear, 1.0f, 1e-4f);
	EXPECT_TRUE(inconsistent.linearLink.isValid());
	EXPECT_EQ(inconsistent.linearLink.to(), -10);
	EXPECT_TRUE(inconsistent.linearLink.from() == 1 || inconsistent.linearLink.from() == 2);
}

TEST(GraphTest, GetMaxOdomInf)
{
	std::multimap<int, Link> links;
	insertLink(links, Link(1, 2, Link::kNeighbor, Transform::getIdentity(), infMatrixDiagonal(1, 2, 3, 4, 5, 6)));
	insertLink(links, Link(2, 3, Link::kNeighbor, Transform::getIdentity(), infMatrixDiagonal(6, 5, 4, 3, 2, 1)));
	insertLink(links, Link(3, 4, Link::kGlobalClosure, Transform::getIdentity(), infMatrixDiagonal(99, 99, 99, 99, 99, 99)));

	const std::vector<double> maxInf = graph::getMaxOdomInf(links);
	ASSERT_EQ(maxInf.size(), 6u);
	EXPECT_DOUBLE_EQ(maxInf[0], 6.0);
	EXPECT_DOUBLE_EQ(maxInf[5], 6.0);
}

TEST(GraphTest, GetPathsNeighborChain)
{
	std::map<int, Transform> poses = linePoses(3);
	std::multimap<int, Link> links;
	insertLink(links, neighborLink(1, 2));
	insertLink(links, neighborLink(2, 3));

	const std::list<std::map<int, Transform> > paths = graph::getPaths(poses, links);
	ASSERT_EQ(paths.size(), 1u);
	EXPECT_EQ(paths.front().size(), 3u);
	EXPECT_EQ(poses.size(), 3u); // poses passed by value, caller's map is unchanged
}
