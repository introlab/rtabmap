#include <gtest/gtest.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/util3d_transforms.h>
#include <opencv2/core.hpp>
#include <cmath>
#include <limits>
#include <cstring>
#include <string>
#include <vector>

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

// ---------------------------------------------------------------------------
// The scan's own interface: formats, offsets, constructors and operations.
// ---------------------------------------------------------------------------

namespace {

struct FormatCase
{
	LaserScan::Format format;
	const char * name;
	int channels;
	bool is2d, normals, rgb, intensity, time, ring;
	int intensityOffset, rgbOffset, normalsOffset, timeOffset, ringOffset;
};

const FormatCase kFormats[] = {
	// format,                 name,           ch, 2d,    N,     RGB,   I,     T,     R,     Ioff, RGBoff, Noff, Toff, Roff
	{LaserScan::kXY,           "XY",           2,  true,  false, false, false, false, false, -1,   -1,     -1,   -1,   -1},
	{LaserScan::kXYZ,          "XYZ",          3,  false, false, false, false, false, false, -1,   -1,     -1,   -1,   -1},
	{LaserScan::kXYI,          "XYI",          3,  true,  false, false, true,  false, false,  2,   -1,     -1,   -1,   -1},
	{LaserScan::kXYZI,         "XYZI",         4,  false, false, false, true,  false, false,  3,   -1,     -1,   -1,   -1},
	{LaserScan::kXYZRGB,       "XYZRGB",       4,  false, false, true,  false, false, false, -1,    3,     -1,   -1,   -1},
	{LaserScan::kXYNormal,     "XYNormal",     5,  true,  true,  false, false, false, false, -1,   -1,      2,   -1,   -1},
	{LaserScan::kXYZNormal,    "XYZNormal",    6,  false, true,  false, false, false, false, -1,   -1,      3,   -1,   -1},
	{LaserScan::kXYINormal,    "XYINormal",    6,  true,  true,  false, true,  false, false,  2,   -1,      3,   -1,   -1},
	{LaserScan::kXYZINormal,   "XYZINormal",   7,  false, true,  false, true,  false, false,  3,   -1,      4,   -1,   -1},
	{LaserScan::kXYZRGBNormal, "XYZRGBNormal", 7,  false, true,  true,  false, false, false, -1,    3,      4,   -1,   -1},
	{LaserScan::kXYZIT,        "XYZIT",        5,  false, false, false, true,  true,  false,  3,   -1,     -1,    4,   -1},
	{LaserScan::kXYZIRT,       "XYZIRT",       6,  false, false, false, true,  true,  true,   3,   -1,     -1,    5,    4},
};

// A scan of `points` points of the given format, with 1-based point index in x
LaserScan scanOf(const FormatCase & format, int points)
{
	cv::Mat data = cv::Mat::zeros(1, points, CV_32FC(format.channels));
	for(int i=0; i<points; ++i)
	{
		data.ptr<float>(0, i)[0] = float(i+1);
	}
	if(format.is2d)
	{
		return LaserScan(data, format.format, 0.1f, 10.0f, -float(M_PI), float(M_PI), float(M_PI)/180.0f);
	}
	return LaserScan(data, 0, 10.0f, format.format);
}

} // namespace

TEST(LaserScanTest, FormatHelpersDescribeEveryFormat)
{
	for(const FormatCase & f : kFormats)
	{
		EXPECT_EQ(LaserScan::formatName(f.format), std::string(f.name));
		EXPECT_EQ(LaserScan::channels(f.format), f.channels) << f.name;
		EXPECT_EQ(LaserScan::isScan2d(f.format), f.is2d) << f.name;
		EXPECT_EQ(LaserScan::isScanHasNormals(f.format), f.normals) << f.name;
		EXPECT_EQ(LaserScan::isScanHasRGB(f.format), f.rgb) << f.name;
		EXPECT_EQ(LaserScan::isScanHasIntensity(f.format), f.intensity) << f.name;
		EXPECT_EQ(LaserScan::isScanHasTime(f.format), f.time) << f.name;
		EXPECT_EQ(LaserScan::isScanHasRing(f.format), f.ring) << f.name;
	}
	EXPECT_EQ(LaserScan::formatName(LaserScan::kUnknown), "Unknown");
}

TEST(LaserScanTest, ScanOfEachFormatReportsItsChannelOffsets)
{
	for(const FormatCase & f : kFormats)
	{
		const LaserScan scan = scanOf(f, 3);
		ASSERT_FALSE(scan.isEmpty()) << f.name;
		EXPECT_EQ(scan.format(), f.format) << f.name;
		EXPECT_EQ(scan.formatName(), std::string(f.name));
		EXPECT_EQ(scan.channels(), f.channels) << f.name;
		EXPECT_EQ(scan.dataType(), CV_32FC(f.channels)) << f.name;
		EXPECT_EQ(scan.size(), 3) << f.name;
		EXPECT_EQ(scan.is2d(), f.is2d) << f.name;
		EXPECT_EQ(scan.hasNormals(), f.normals) << f.name;
		EXPECT_EQ(scan.hasRGB(), f.rgb) << f.name;
		EXPECT_EQ(scan.hasIntensity(), f.intensity) << f.name;
		EXPECT_EQ(scan.hasTime(), f.time) << f.name;
		EXPECT_EQ(scan.hasRing(), f.ring) << f.name;
		EXPECT_EQ(scan.getIntensityOffset(), f.intensityOffset) << f.name;
		EXPECT_EQ(scan.getRGBOffset(), f.rgbOffset) << f.name;
		EXPECT_EQ(scan.getNormalsOffset(), f.normalsOffset) << f.name;
		EXPECT_EQ(scan.getTimeOffset(), f.timeOffset) << f.name;
		EXPECT_EQ(scan.getRingOffset(), f.ringOffset) << f.name;
		EXPECT_FALSE(scan.isCompressed()) << f.name;
		EXPECT_FALSE(scan.isOrganized()) << f.name;
		// Every offset addresses a channel of the point
		for(int offset : {f.intensityOffset, f.rgbOffset, f.normalsOffset, f.timeOffset, f.ringOffset})
		{
			if(offset >= 0)
			{
				EXPECT_LT(offset, f.channels) << f.name;
			}
		}
		if(f.normals)
		{
			EXPECT_EQ(f.normalsOffset + 3, f.channels) << f.name << ": the normal is the last 3 channels";
		}
	}
}

TEST(LaserScanTest, PackedRGBRoundTrips)
{
	const unsigned char colors[][3] = {{0, 0, 0}, {255, 255, 255}, {12, 200, 37}, {255, 0, 128}};
	for(const auto & c : colors)
	{
		const float packed = LaserScan::packRGB(c[0], c[1], c[2]);
		unsigned char r, g, b;
		LaserScan::unpackRGB(packed, r, g, b);
		EXPECT_EQ(r, c[0]);
		EXPECT_EQ(g, c[1]);
		EXPECT_EQ(b, c[2]);
	}
	// Packed as PCL does: an int holding 0x00RRGGBB in the float's bytes
	const float packed = LaserScan::packRGB(0x12, 0x34, 0x56);
	int bits;
	std::memcpy(&bits, &packed, sizeof(int));
	EXPECT_EQ(bits, 0x123456);
}

TEST(LaserScanTest, DefaultScanIsEmpty)
{
	const LaserScan scan;
	EXPECT_TRUE(scan.isEmpty());
	EXPECT_TRUE(scan.empty());
	EXPECT_EQ(scan.size(), 0);
	EXPECT_EQ(scan.format(), LaserScan::kUnknown);
	EXPECT_FALSE(scan.isOrganized());
	EXPECT_FALSE(scan.isCompressed());
	EXPECT_TRUE(scan.localTransform().isIdentity());
}

TEST(LaserScanTest, ThreeDConstructorKeepsItsSettings)
{
	const Transform localTransform(0.1f, 0.2f, 0.3f, 0.0f, 0.0f, 0.5f);
	const LaserScan scan(cv::Mat::zeros(1, 10, CV_32FC3), 100, 20.0f, LaserScan::kXYZ, localTransform);
	EXPECT_EQ(scan.maxPoints(), 100);
	EXPECT_FLOAT_EQ(scan.rangeMin(), 0.0f);
	EXPECT_FLOAT_EQ(scan.rangeMax(), 20.0f);
	EXPECT_FLOAT_EQ(scan.angleIncrement(), 0.0f);
	EXPECT_FALSE(scan.is2d());
	EXPECT_EQ(scan.localTransform().x(), localTransform.x());
	EXPECT_EQ(scan.localTransform().theta(), localTransform.theta());

	// An organized scan without a max points setting takes its own size
	const LaserScan organized(cv::Mat::zeros(16, 1800, CV_32FC3), 0, 100.0f, LaserScan::kXYZ);
	EXPECT_TRUE(organized.isOrganized());
	EXPECT_EQ(organized.maxPoints(), 16*1800);
	EXPECT_EQ(organized.size(), 16*1800);

	LaserScan moved = scan;
	moved.setLocalTransform(Transform::getIdentity());
	EXPECT_TRUE(moved.localTransform().isIdentity());
	EXPECT_FALSE(scan.localTransform().isIdentity()) << "a copy, not a reference";
}

TEST(LaserScanTest, TwoDConstructorDerivesMaxPointsFromTheAngles)
{
	const float increment = float(M_PI)/180.0f; // 1 degree
	const LaserScan scan(cv::Mat::zeros(1, 181, CV_32FC2), LaserScan::kXY, 0.2f, 30.0f, -float(M_PI)/2.0f, float(M_PI)/2.0f, increment);
	EXPECT_TRUE(scan.is2d());
	EXPECT_EQ(scan.maxPoints(), 181);
	EXPECT_FLOAT_EQ(scan.rangeMin(), 0.2f);
	EXPECT_FLOAT_EQ(scan.rangeMax(), 30.0f);
	EXPECT_FLOAT_EQ(scan.angleMin(), -float(M_PI)/2.0f);
	EXPECT_FLOAT_EQ(scan.angleMax(), float(M_PI)/2.0f);
	EXPECT_FLOAT_EQ(scan.angleIncrement(), increment);

	// Clockwise scans have a negative increment
	const LaserScan clockwise(cv::Mat::zeros(1, 10, CV_32FC2), LaserScan::kXY, 0.0f, 5.0f, 1.0f, 0.0f, -0.1f);
	EXPECT_EQ(clockwise.maxPoints(), 11);
}

TEST(LaserScanTest, UnknownFormatIsDeducedFromTheChannels)
{
	EXPECT_EQ(LaserScan(cv::Mat::zeros(1, 5, CV_32FC2), LaserScan::kUnknown, 0.0f, 5.0f, -1.0f, 1.0f, 0.1f).format(), LaserScan::kXY);
	EXPECT_EQ(LaserScan(cv::Mat::zeros(1, 5, CV_32FC3), 0, 5.0f, LaserScan::kUnknown).format(), LaserScan::kXYZ);
	EXPECT_EQ(LaserScan(cv::Mat::zeros(1, 5, CV_32FC(4)), 0, 5.0f, LaserScan::kUnknown).format(), LaserScan::kXYZRGB);
	// The LaserScan-based constructors keep the format of the scan
	const LaserScan source(cv::Mat::zeros(1, 5, CV_32FC(5)), 0, 5.0f, LaserScan::kXYZIT);
	const LaserScan copy(source, 50, 8.0f, Transform(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	EXPECT_EQ(copy.format(), LaserScan::kXYZIT);
	EXPECT_EQ(copy.maxPoints(), 50);
	EXPECT_FLOAT_EQ(copy.rangeMax(), 8.0f);
	EXPECT_FLOAT_EQ(copy.localTransform().x(), 1.0f);
	EXPECT_EQ(copy.data().data, source.data().data) << "the data is shared, not copied";
}

TEST(LaserScanTest, CloneCopiesTheDataAndTheSettings)
{
	const Transform localTransform(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
	const LaserScan scan(cv::Mat::ones(1, 4, CV_32FC(4)), 40, 12.0f, LaserScan::kXYZI, localTransform);
	LaserScan clone = scan.clone();
	EXPECT_NE(clone.data().data, scan.data().data);
	clone.point(0)[0] = 7.0f;
	EXPECT_FLOAT_EQ(scan.point(0)[0], 1.0f) << "the original is untouched";
	EXPECT_EQ(clone.format(), LaserScan::kXYZI);
	EXPECT_EQ(clone.maxPoints(), 40);
	EXPECT_FLOAT_EQ(clone.rangeMax(), 12.0f);
	EXPECT_FLOAT_EQ(clone.localTransform().z(), 1.0f);

	const LaserScan scan2d(cv::Mat::ones(1, 4, CV_32FC2), LaserScan::kXY, 0.1f, 5.0f, -1.0f, 1.0f, 0.5f);
	const LaserScan clone2d = scan2d.clone();
	EXPECT_TRUE(clone2d.is2d());
	EXPECT_FLOAT_EQ(clone2d.angleIncrement(), 0.5f);
	EXPECT_FLOAT_EQ(clone2d.rangeMin(), 0.1f);
}

TEST(LaserScanTest, DensifyDropsTheInvalidPointsOfAnOrganizedScan)
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	cv::Mat data(2, 3, CV_32FC3);
	for(int i=0; i<6; ++i)
	{
		float * p = data.ptr<float>(i/3, i%3);
		p[0] = float(i); p[1] = 0.0f; p[2] = 0.0f;
	}
	data.ptr<float>(1, 1)[2] = nan; // point 4 is invalid
	const LaserScan scan(data, 0, 10.0f, LaserScan::kXYZ, Transform(0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	ASSERT_TRUE(scan.isOrganized());

	const LaserScan dense = scan.densify();
	EXPECT_FALSE(dense.isOrganized());
	ASSERT_EQ(dense.size(), 5);
	const float expected[5] = {0, 1, 2, 3, 5};
	for(int i=0; i<5; ++i)
	{
		EXPECT_FLOAT_EQ(dense.point(i)[0], expected[i]) << "point " << i << ": row by row, without the invalid one";
	}
	EXPECT_EQ(dense.format(), LaserScan::kXYZ);
	EXPECT_EQ(dense.maxPoints(), 6) << "the organized size is kept as max points";
	EXPECT_FLOAT_EQ(dense.localTransform().x(), 0.5f);

	// Already dense: returned as is
	EXPECT_EQ(dense.densify().data().data, dense.data().data);
}

TEST(LaserScanTest, ConcatenationAppendsThePoints)
{
	cv::Mat a(1, 2, CV_32FC3), b(1, 3, CV_32FC3);
	for(int i=0; i<2; ++i) { a.ptr<float>(0, i)[0] = float(i); }
	for(int i=0; i<3; ++i) { b.ptr<float>(0, i)[0] = float(10+i); }
	const LaserScan scanA(a, 0, 10.0f, LaserScan::kXYZ);
	const LaserScan scanB(b, 0, 10.0f, LaserScan::kXYZ);

	LaserScan sum = LaserScan(scanA) + scanB;
	ASSERT_EQ(sum.size(), 5);
	EXPECT_EQ(sum.format(), LaserScan::kXYZ);
	const float expected[5] = {0, 1, 10, 11, 12};
	for(int i=0; i<5; ++i)
	{
		EXPECT_FLOAT_EQ(sum.point(i)[0], expected[i]) << "point " << i;
	}
	EXPECT_NE(sum.data().data, scanA.data().data) << "a new matrix";

	// With an empty operand, a copy of the other one
	EXPECT_EQ((LaserScan() + scanB).size(), 3);
	EXPECT_EQ((LaserScan(scanA) + LaserScan()).size(), 2);
	EXPECT_EQ((LaserScan() + LaserScan()).size(), 0);

	LaserScan accumulated;
	accumulated += scanA;
	accumulated += scanB;
	accumulated += scanA;
	EXPECT_EQ(accumulated.size(), 7);
	EXPECT_FLOAT_EQ(accumulated.point(6)[0], 1.0f);
}

TEST(LaserScanTest, CompressedAndClearedScans)
{
	const LaserScan compressed(cv::Mat::zeros(1, 100, CV_8UC1), 0, 10.0f, LaserScan::kXYZ);
	EXPECT_TRUE(compressed.isCompressed());
	EXPECT_FALSE(compressed.isEmpty());
	EXPECT_EQ(compressed.format(), LaserScan::kXYZ) << "the format is the one given, the channels can't be checked";

	LaserScan scan(cv::Mat::zeros(1, 10, CV_32FC3), 0, 10.0f, LaserScan::kXYZ, Transform(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	scan.clear();
	EXPECT_TRUE(scan.isEmpty());
	EXPECT_EQ(scan.size(), 0);
	EXPECT_EQ(scan.format(), LaserScan::kXYZ) << "clear() only drops the points";
	EXPECT_FLOAT_EQ(scan.localTransform().y(), 1.0f);
}
