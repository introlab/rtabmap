#include <gtest/gtest.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/util3d_transforms.h>
#include <opencv2/core.hpp>

using namespace rtabmap;

namespace {

// An organized XYZIT scan of rings x columns points, each point holding its
// own index in the intensity field and its column in the time field.
LaserScan organizedScan(int rings, int columns)
{
	cv::Mat data(rings, columns, CV_32FC(5));
	for(int v=0; v<rings; ++v)
	{
		for(int u=0; u<columns; ++u)
		{
			float * p = data.ptr<float>(v, u);
			p[0] = float(u);
			p[1] = float(v);
			p[2] = 1.0f;
			p[3] = float(v*columns + u); // intensity: the point's index
			p[4] = float(u) * 0.001f;    // time
		}
	}
	return LaserScan(data, rings*columns, 0.0f, LaserScan::kXYZIT, Transform::getIdentity());
}

} // namespace

TEST(LaserScanTest, PointIndexRunsRowByRowOnAnOrganizedScan)
{
	const int rings = 4;
	const int columns = 8;
	const LaserScan scan = organizedScan(rings, columns);
	ASSERT_TRUE(scan.isOrganized());
	ASSERT_EQ(scan.size(), rings*columns);
	for(int i=0; i<scan.size(); ++i)
	{
		const float * p = scan.point(i);
		EXPECT_FLOAT_EQ(p[0], float(i % columns)) << "point " << i;
		EXPECT_FLOAT_EQ(p[1], float(i / columns)) << "point " << i;
		EXPECT_FLOAT_EQ(p[scan.getIntensityOffset()], float(i)) << "point " << i;
		EXPECT_FLOAT_EQ(scan.field(i, scan.getTimeOffset()), float(i % columns) * 0.001f) << "point " << i;
	}
	// The last point is on the last row, where data().ptr(0, size-1) would be out of bounds
	EXPECT_FLOAT_EQ(scan.field(scan.size()-1, scan.getTimeOffset()), float(columns-1) * 0.001f);
	EXPECT_NE(scan.field(scan.size()-1, scan.getTimeOffset()), scan.field(0, scan.getTimeOffset()));
}

TEST(LaserScanTest, PointIsWritableAndMatchesTheFlatLayout)
{
	LaserScan scan = organizedScan(3, 5);
	scan.point(7)[2] = 42.0f;
	EXPECT_FLOAT_EQ(scan.data().ptr<float>(1, 2)[2], 42.0f);
	scan.field(14, 2) = 43.0f;
	EXPECT_FLOAT_EQ(scan.data().ptr<float>(2, 4)[2], 43.0f);

	// Same index space as the flat (1xN) scan
	const LaserScan flat = organizedScan(3, 5).densify();
	ASSERT_FALSE(flat.isOrganized());
	ASSERT_EQ(flat.size(), 15);
	for(int i=0; i<flat.size(); ++i)
	{
		EXPECT_FLOAT_EQ(flat.point(i)[flat.getIntensityOffset()], float(i));
		EXPECT_EQ(flat.point(i), flat.data().ptr<float>(0, i));
	}
}

TEST(LaserScanTest, TransformLaserScanKeepsAnOrganizedScanOrganized)
{
	const LaserScan scan = organizedScan(4, 8);
	const LaserScan moved = util3d::transformLaserScan(scan, Transform(1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f));
	ASSERT_TRUE(moved.isOrganized());
	ASSERT_EQ(moved.data().rows, 4);
	ASSERT_EQ(moved.data().cols, 8);
	for(int i=0; i<scan.size(); ++i)
	{
		EXPECT_FLOAT_EQ(moved.point(i)[0], scan.point(i)[0] + 1.0f) << "point " << i;
		EXPECT_FLOAT_EQ(moved.point(i)[1], scan.point(i)[1] + 2.0f) << "point " << i;
		EXPECT_FLOAT_EQ(moved.point(i)[2], scan.point(i)[2] + 3.0f) << "point " << i;
		EXPECT_FLOAT_EQ(moved.point(i)[moved.getIntensityOffset()], float(i)) << "point " << i;
	}
}
