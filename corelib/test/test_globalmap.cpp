#include <gtest/gtest.h>
#include <rtabmap/core/GlobalMap.h>
#include <rtabmap/core/global_map/OccupancyGrid.h>
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/core.hpp>

using namespace rtabmap;

namespace {

static cv::Mat grid2D(int cols)
{
	return cv::Mat(1, cols, CV_32FC2, cv::Scalar(1.f, 2.f));
}

class MockGlobalMap : public GlobalMap
{
public:
	MockGlobalMap(const LocalGridCache * cache, const ParametersMap & parameters = ParametersMap()) :
		GlobalMap(cache, parameters)
	{
	}

	std::list<std::pair<int, Transform> > lastAssembled;
	int assembleCount = 0;

protected:
	void assemble(const std::list<std::pair<int, Transform> > & newPoses) override
	{
		lastAssembled = newPoses;
		++assembleCount;
		for(std::list<std::pair<int, Transform> >::const_iterator it = newPoses.begin();
				it != newPoses.end();
				++it)
		{
			if(it->first > 0)
			{
				addAssembledNode(it->first, it->second);
			}
		}
	}
};

static ParametersMap globalMapTestParams()
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kGridCellSize(), "0.2"));
	params.insert(ParametersPair(Parameters::kGridGlobalUpdateError(), "0.1"));
	return params;
}

static void addGridToCache(LocalGridCache & cache, int nodeId)
{
	cache.add(nodeId, grid2D(1), grid2D(1), cv::Mat(), 0.2f);
}

} // namespace

TEST(GlobalMapTest, LogOddsProbabilityRoundTrip)
{
	const double p = 0.7;
	const float lo = GlobalMap::logodds(p);
	EXPECT_NEAR(GlobalMap::probability(lo), p, 1e-6);
	EXPECT_GT(lo, 0.f);
}

TEST(GlobalMapTest, ConstructorParsesParameters)
{
	LocalGridCache cache;
	addGridToCache(cache, 1);
	MockGlobalMap map(&cache, globalMapTestParams());

	EXPECT_FLOAT_EQ(map.getCellSize(), 0.2f);
	EXPECT_FLOAT_EQ(map.getUpdateError(), 0.1f);
	EXPECT_TRUE(map.addedNodes().empty());
}

TEST(GlobalMapTest, ClearResetsAssembledNodes)
{
	LocalGridCache cache;
	addGridToCache(cache, 1);
	MockGlobalMap map(&cache, globalMapTestParams());

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform::getIdentity()));
	ASSERT_TRUE(map.update(poses));
	ASSERT_EQ(map.addedNodes().size(), 1u);

	map.clear();
	EXPECT_TRUE(map.addedNodes().empty());
}

TEST(GlobalMapTest, UpdateAssemblesCachedNodes)
{
	LocalGridCache cache;
	addGridToCache(cache, 1);
	addGridToCache(cache, 2);
	MockGlobalMap map(&cache, globalMapTestParams());

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform::getIdentity()));
	poses.insert(std::make_pair(2, Transform(1, 0, 0, 0, 0, 0, 1)));

	EXPECT_TRUE(map.update(poses));
	EXPECT_EQ(map.assembleCount, 1);
	EXPECT_EQ(map.lastAssembled.size(), 2u);
	EXPECT_EQ(map.addedNodes().size(), 2u);
	EXPECT_EQ(map.addedNodes().count(1), 1u);
	EXPECT_EQ(map.addedNodes().count(2), 1u);
}

TEST(GlobalMapTest, UpdateSkipsNodesNotInCache)
{
	LocalGridCache cache;
	addGridToCache(cache, 1);
	MockGlobalMap map(&cache, globalMapTestParams());

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform::getIdentity()));
	poses.insert(std::make_pair(2, Transform(1, 0, 0, 0, 0, 0, 1)));

	EXPECT_TRUE(map.update(poses));
	EXPECT_EQ(map.lastAssembled.size(), 1u);
	EXPECT_EQ(map.addedNodes().size(), 1u);
}

TEST(GlobalMapTest, UpdateReturnsFalseWhenNothingToAssemble)
{
	LocalGridCache cache;
	addGridToCache(cache, 1);
	MockGlobalMap map(&cache, globalMapTestParams());

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform::getIdentity()));
	ASSERT_TRUE(map.update(poses));
	EXPECT_FALSE(map.update(poses));
	EXPECT_EQ(map.assembleCount, 1);
}

TEST(GlobalMapTest, UpdateAppendsPoseZeroAsTemporaryNode)
{
	LocalGridCache cache;
	addGridToCache(cache, 1);
	MockGlobalMap map(&cache, globalMapTestParams());

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(0, Transform(0, 0, 1, 0, 0, 0, 1)));
	poses.insert(std::make_pair(1, Transform::getIdentity()));

	EXPECT_TRUE(map.update(poses));
	ASSERT_EQ(map.lastAssembled.size(), 2u);
	EXPECT_EQ(map.lastAssembled.back().first, -1);
	EXPECT_EQ(map.addedNodes().size(), 1u);
	EXPECT_EQ(map.addedNodes().count(-1), 0u);
}

TEST(GlobalMapTest, FullUpdateNeededWhenPoseMovesBeyondThreshold)
{
	LocalGridCache cache;
	addGridToCache(cache, 1);
	MockGlobalMap map(&cache, globalMapTestParams());

	Transform t0 = Transform::getIdentity();
	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, t0));
	ASSERT_TRUE(map.update(poses));

	Transform t1(0.5f, 0, 0, 0, 0, 0, 1);
	poses[1] = t1;
	EXPECT_TRUE(map.fullUpdateNeeded(poses));
}

TEST(GlobalMapTest, FullUpdateNeededWhenAssembledGraphDisjoint)
{
	LocalGridCache cache;
	addGridToCache(cache, 1);
	MockGlobalMap map(&cache, globalMapTestParams());

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform::getIdentity()));
	ASSERT_TRUE(map.update(poses));

	std::map<int, Transform> newPoses;
	newPoses.insert(std::make_pair(2, Transform::getIdentity()));
	EXPECT_TRUE(map.fullUpdateNeeded(newPoses));
}

TEST(GlobalMapTest, FullUpdateClearsMapBeforeReassemble)
{
	LocalGridCache cache;
	addGridToCache(cache, 1);
	addGridToCache(cache, 2);
	MockGlobalMap map(&cache, globalMapTestParams());

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform::getIdentity()));
	ASSERT_TRUE(map.update(poses));

	std::map<int, Transform> newPoses;
	newPoses.insert(std::make_pair(2, Transform::getIdentity()));
	EXPECT_TRUE(map.fullUpdateNeeded(newPoses));
	EXPECT_TRUE(map.update(newPoses));
	EXPECT_EQ(map.addedNodes().size(), 1u);
	EXPECT_EQ(map.addedNodes().count(2), 1u);
	EXPECT_EQ(map.addedNodes().count(1), 0u);
}

TEST(GlobalMapTest, GetMemoryUsed)
{
	LocalGridCache cache;
	addGridToCache(cache, 1);
	MockGlobalMap map(&cache, globalMapTestParams());

	EXPECT_GT(map.getMemoryUsed(), 0u);

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform::getIdentity()));
	ASSERT_TRUE(map.update(poses));
	EXPECT_GT(map.getMemoryUsed(), 0u);
}

// ---------------------------------------------------------------------------
// The tests above drive GlobalMap through MockGlobalMap, which overrides
// assemble(), so the real rasterisation never runs. These use the concrete
// OccupancyGrid: cached local grids are assembled at known poses and the
// resulting map is checked for extent and content.
// ---------------------------------------------------------------------------

namespace {

// A local grid shaped like a short wall: `cols` obstacle cells one cell apart
// along +x, plus a matching strip of empty cells in front of it.
void addWallGridToCache(LocalGridCache & cache, int nodeId, int cols, float cellSize)
{
	cv::Mat obstacles(1, cols, CV_32FC2);
	cv::Mat empty(1, cols, CV_32FC2);
	for(int i = 0; i < cols; ++i)
	{
		obstacles.at<cv::Vec2f>(0, i) = cv::Vec2f(cellSize * i, 0.0f);
		empty.at<cv::Vec2f>(0, i)     = cv::Vec2f(cellSize * i, -cellSize);
	}
	cache.add(nodeId, cv::Mat(), obstacles, empty, cellSize);
}

}  // namespace

TEST(OccupancyGridTest, AssemblesCachedGridIntoMap)
{
	const float cellSize = 0.2f;
	LocalGridCache cache;
	addWallGridToCache(cache, 1, /*cols=*/5, cellSize);

	ParametersMap parameters;
	parameters.insert(ParametersPair(Parameters::kGridCellSize(), uNumber2Str(cellSize)));
	OccupancyGrid grid(&cache, parameters);

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform::getIdentity()));
	EXPECT_TRUE(grid.update(poses));

	float xMin = 0.0f, yMin = 0.0f;
	const cv::Mat map = grid.getMap(xMin, yMin);
	ASSERT_FALSE(map.empty()) << "assemble() produced no map";
	EXPECT_EQ(CV_8SC1, map.type()) << "occupancy maps are signed char (-1 unknown, 0 empty, 100 occupied)";
	EXPECT_GT(map.total(), 5u) << "map smaller than the grid that was assembled";

	// The wall spans 5 cells at 20 cm, so the map must be at least that wide.
	EXPECT_GE(map.cols * cellSize, 5 * cellSize - 1e-3f);

	int occupied = 0, empty = 0, unknown = 0;
	for(int y = 0; y < map.rows; ++y)
	{
		for(int x = 0; x < map.cols; ++x)
		{
			const signed char v = map.at<signed char>(y, x);
			if(v == 0)        ++empty;
			else if(v > 0)    ++occupied;
			else              ++unknown;
		}
	}
	EXPECT_GT(occupied, 0) << "no occupied cells rasterised";
	EXPECT_GT(empty, 0)    << "no empty cells rasterised";
	EXPECT_EQ(map.total(), static_cast<size_t>(occupied + empty + unknown));
}

// Two nodes offset along x must produce a map wider than either alone: this is
// what distinguishes real assembly from returning the last grid.
TEST(OccupancyGridTest, SecondPoseExtendsTheMap)
{
	const float cellSize = 0.2f;
	LocalGridCache cache;
	addWallGridToCache(cache, 1, /*cols=*/5, cellSize);
	addWallGridToCache(cache, 2, /*cols=*/5, cellSize);

	ParametersMap parameters;
	parameters.insert(ParametersPair(Parameters::kGridCellSize(), uNumber2Str(cellSize)));

	OccupancyGrid single(&cache, parameters);
	std::map<int, Transform> onePose;
	onePose.insert(std::make_pair(1, Transform::getIdentity()));
	single.update(onePose);
	float xMin1 = 0.0f, yMin1 = 0.0f;
	const cv::Mat mapOne = single.getMap(xMin1, yMin1);
	ASSERT_FALSE(mapOne.empty());

	OccupancyGrid both(&cache, parameters);
	std::map<int, Transform> twoPoses;
	twoPoses.insert(std::make_pair(1, Transform::getIdentity()));
	twoPoses.insert(std::make_pair(2, Transform(2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)));
	both.update(twoPoses);
	float xMin2 = 0.0f, yMin2 = 0.0f;
	const cv::Mat mapTwo = both.getMap(xMin2, yMin2);
	ASSERT_FALSE(mapTwo.empty());

	EXPECT_GT(mapTwo.cols, mapOne.cols)
			<< "a node 2 m away did not widen the map (" << mapOne.cols << " -> " << mapTwo.cols << ")";
	EXPECT_GT(mapTwo.total(), mapOne.total());
}

// An empty cache means nothing to assemble; the map must stay empty rather than
// producing a degenerate one.
TEST(OccupancyGridTest, EmptyCacheProducesEmptyMap)
{
	LocalGridCache cache;
	OccupancyGrid grid(&cache, globalMapTestParams());
	grid.update(std::map<int, Transform>());
	float xMin = 0.0f, yMin = 0.0f;
	EXPECT_TRUE(grid.getMap(xMin, yMin).empty());
}
