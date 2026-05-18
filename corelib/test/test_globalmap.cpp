#include <gtest/gtest.h>
#include <rtabmap/core/GlobalMap.h>
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/Parameters.h>
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
