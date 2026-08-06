#include <gtest/gtest.h>
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/utilite/UException.h>
#include <opencv2/core.hpp>

using namespace rtabmap;

namespace {

static cv::Mat grid2D(int cols)
{
	return cv::Mat(1, cols, CV_32FC2, cv::Scalar(1.f, 2.f));
}

static cv::Mat grid3D(int cols)
{
	return cv::Mat(1, cols, CV_32FC3, cv::Scalar(1.f, 2.f, 3.f));
}

} // namespace

TEST(LocalGridTest, ConstructorStoresFields)
{
	const cv::Mat ground = grid2D(2);
	const cv::Mat obstacles = grid2D(1);
	const cv::Mat empty = grid2D(3);
	const cv::Point3f viewPoint(1.f, 2.f, 3.f);

	const LocalGrid grid(ground, obstacles, empty, 0.05f, viewPoint);

	EXPECT_EQ(grid.groundCells.rows, 1);
	EXPECT_EQ(grid.groundCells.cols, 2);
	EXPECT_EQ(grid.obstacleCells.cols, 1);
	EXPECT_EQ(grid.emptyCells.cols, 3);
	EXPECT_FLOAT_EQ(grid.cellSize, 0.05f);
	EXPECT_FLOAT_EQ(grid.viewPoint.x, 1.f);
	EXPECT_FLOAT_EQ(grid.viewPoint.y, 2.f);
	EXPECT_FLOAT_EQ(grid.viewPoint.z, 3.f);
	EXPECT_FALSE(grid.is3D());
}

TEST(LocalGridTest, InvalidCellSizeThrows)
{
	const cv::Mat ground = grid2D(1);
	ASSERT_THROW(LocalGrid(ground, cv::Mat(), cv::Mat(), 0.f), UException);
}

TEST(LocalGridTest, Is3DEmptyCells)
{
	const LocalGrid grid(cv::Mat(), cv::Mat(), cv::Mat(), 0.1f);
	EXPECT_TRUE(grid.is3D());
}

TEST(LocalGridTest, Is3DDetects2DAnd3DCellTypes)
{
	LocalGrid grid2d(grid2D(1), cv::Mat(), cv::Mat(), 0.1f);
	EXPECT_FALSE(grid2d.is3D());

	LocalGrid grid3d(grid3D(1), cv::Mat(), cv::Mat(), 0.1f);
	EXPECT_TRUE(grid3d.is3D());

	LocalGrid mixed(grid2D(1), grid3D(1), cv::Mat(), 0.1f);
	EXPECT_FALSE(mixed.is3D());
}

TEST(LocalGridTest, CacheAddFindAndIterate)
{
	LocalGridCache cache;
	const LocalGrid grid(grid2D(2), grid2D(1), cv::Mat(), 0.1f);

	cache.add(5, grid);
	EXPECT_EQ(cache.size(), 1u);
	EXPECT_FALSE(cache.empty());

	const auto it = cache.find(5);
	ASSERT_NE(it, cache.end());
	EXPECT_EQ(it->first, 5);
	EXPECT_EQ(it->second.groundCells.cols, 2);

	EXPECT_EQ(cache.begin(), cache.find(5));
	EXPECT_EQ(cache.find(99), cache.end());
	EXPECT_EQ(cache.localGrids().size(), 1u);
}

TEST(LocalGridTest, CacheNodeIdZeroStoredAsTemporary)
{
	LocalGridCache cache;
	cache.add(0, grid2D(1), cv::Mat(), cv::Mat(), 0.1f);
	EXPECT_EQ(cache.size(), 1u);
	EXPECT_NE(cache.find(-1), cache.end());
	EXPECT_EQ(cache.find(0), cache.end());
}

TEST(LocalGridTest, CacheNegativeNodeIdIgnored)
{
	LocalGridCache cache;
	cache.add(-2, grid2D(1), cv::Mat(), cv::Mat(), 0.1f);
	EXPECT_TRUE(cache.empty());
}

TEST(LocalGridTest, ShareToCopiesWhenAbsentInTarget)
{
	LocalGridCache source;
	LocalGridCache target;
	source.add(3, grid2D(2), grid2D(1), cv::Mat(), 0.2f, cv::Point3f(4.f, 5.f, 6.f));

	EXPECT_TRUE(source.shareTo(3, target));
	EXPECT_EQ(target.size(), 1u);
	const auto it = target.find(3);
	ASSERT_NE(it, target.end());
	EXPECT_EQ(it->second.groundCells.cols, 2);
	EXPECT_FLOAT_EQ(it->second.cellSize, 0.2f);
	EXPECT_FLOAT_EQ(it->second.viewPoint.z, 6.f);

	EXPECT_FALSE(source.shareTo(3, target));
	EXPECT_FALSE(source.shareTo(99, target));
}

TEST(LocalGridTest, GetMemoryUsed)
{
	LocalGridCache cache;
	const unsigned long memEmpty = cache.getMemoryUsed();

	cache.add(1, grid2D(10), grid2D(5), grid2D(3), 0.1f);
	EXPECT_GT(cache.getMemoryUsed(), memEmpty);
}

TEST(LocalGridTest, ClearRemovesAllEntries)
{
	LocalGridCache cache;
	const unsigned long memEmpty = cache.getMemoryUsed();

	cache.add(1, grid2D(2), cv::Mat(), cv::Mat(), 0.1f);
	cache.clear();

	EXPECT_TRUE(cache.empty());
	EXPECT_EQ(cache.getMemoryUsed(), memEmpty);
}

TEST(LocalGridTest, ClearTemporaryRemovesNodeIdZero)
{
	LocalGridCache cache;
	cache.add(0, grid2D(2), cv::Mat(), cv::Mat(), 0.1f); // node id 0 -> key -1
	cache.add(2, grid2D(3), cv::Mat(), cv::Mat(), 0.1f);
	EXPECT_EQ(cache.size(), 2u);

	cache.clear(true);

	EXPECT_EQ(cache.size(), 1u);
	EXPECT_NE(cache.find(2), cache.end());
	EXPECT_EQ(cache.find(-1), cache.end());
}
