#include <gtest/gtest.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include "TestUtils.h"
#include <fstream>

#ifdef RTABMAP_LIBLAS
#include <rtabmap/core/LASWriter.h>
#include <liblas/liblas.hpp>
#endif

using namespace rtabmap;

namespace {

static int g_fileCounter = 0;

static std::string tempLasPath(const std::string & extension)
{
	return test::tempPath(uFormat("rtabmap_laswriter_test_%d_%d.%s", test::getPid(), ++g_fileCounter, extension.c_str()));
}

#ifdef RTABMAP_LIBLAS

static void expectLasSignature(const std::string & path)
{
	std::ifstream file(path.c_str(), std::ios::binary);
	ASSERT_TRUE(file.good());
	char signature[4] = {0};
	file.read(signature, 4);
	EXPECT_EQ(signature[0], 'L');
	EXPECT_EQ(signature[1], 'A');
	EXPECT_EQ(signature[2], 'S');
	EXPECT_EQ(signature[3], 'F');
}

static void readLasPointCount(const std::string & path, size_t & count)
{
	std::ifstream ifs(path.c_str(), std::ios::in | std::ios::binary);
	ASSERT_TRUE(ifs.good());
	liblas::ReaderFactory factory;
	liblas::Reader reader = factory.CreateWithStream(ifs);
	count = reader.GetHeader().GetPointRecordsCount();
}

#endif // RTABMAP_LIBLAS

} // namespace

#ifndef RTABMAP_LIBLAS
TEST(LASWriterTest, SkippedWithoutLibLAS)
{
	GTEST_SKIP() << "RTAB-Map not built with libLAS support";
}
#else

TEST(LASWriterTest, SaveXYZCloud)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.push_back(pcl::PointXYZ(1.f, 2.f, 3.f));
	cloud.push_back(pcl::PointXYZ(4.f, 5.f, 6.f));

	const std::string path = tempLasPath("las");
	EXPECT_EQ(saveLASFile(path, cloud), 0);
	expectLasSignature(path);

	size_t count = 0;
	ASSERT_NO_FATAL_FAILURE(readLasPointCount(path, count));
	EXPECT_EQ(count, 2u);

	std::ifstream ifs(path.c_str(), std::ios::in | std::ios::binary);
	liblas::ReaderFactory factory;
	liblas::Reader reader = factory.CreateWithStream(ifs);
	ASSERT_TRUE(reader.ReadNextPoint());
	EXPECT_NEAR(reader.GetPoint().GetX(), 1.0, 1e-3);
	EXPECT_NEAR(reader.GetPoint().GetY(), 2.0, 1e-3);
	EXPECT_NEAR(reader.GetPoint().GetZ(), 3.0, 1e-3);

	UFile::erase(path);
}

TEST(LASWriterTest, SaveXYZCloudLaz)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.push_back(pcl::PointXYZ(1.f, 2.f, 3.f));
	cloud.push_back(pcl::PointXYZ(4.f, 5.f, 6.f));

	const std::string path = tempLasPath("laz");
	const int rc = saveLASFile(path, cloud);
	if(rc == 1)
	{
		// libLAS built without LASzip (common on distro packages).
		EXPECT_EQ(rc, 1);
		UFile::erase(path);
		return;
	}
	ASSERT_EQ(rc, 0);

	size_t count = 0;
	ASSERT_NO_FATAL_FAILURE(readLasPointCount(path, count));
	EXPECT_EQ(count, 2u);

	std::ifstream ifs(path.c_str(), std::ios::in | std::ios::binary);
	liblas::ReaderFactory factory;
	liblas::Reader reader = factory.CreateWithStream(ifs);
	ASSERT_TRUE(reader.ReadNextPoint());
	EXPECT_NEAR(reader.GetPoint().GetX(), 1.0, 1e-3);
	EXPECT_NEAR(reader.GetPoint().GetY(), 2.0, 1e-3);
	EXPECT_NEAR(reader.GetPoint().GetZ(), 3.0, 1e-3);

	UFile::erase(path);
}

TEST(LASWriterTest, SaveRGBCloudWithCameraIdsAndIntensity)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointXYZRGB pt;
	pt.x = 0.f;
	pt.y = 0.f;
	pt.z = 0.f;
	pt.r = 255;
	pt.g = 128;
	pt.b = 64;
	cloud.push_back(pt);

	const std::vector<int> cameraIds(1, 42);
	const std::vector<float> intensities(1, 12.f);

	const std::string path = tempLasPath("las");
	EXPECT_EQ(saveLASFile(path, cloud, cameraIds, intensities), 0);

	std::ifstream ifs(path.c_str(), std::ios::in | std::ios::binary);
	liblas::ReaderFactory factory;
	liblas::Reader reader = factory.CreateWithStream(ifs);
	ASSERT_TRUE(reader.ReadNextPoint());
	const liblas::Point & lasPt = reader.GetPoint();
	EXPECT_EQ(lasPt.GetPointSourceID(), 42u);
	EXPECT_NEAR(lasPt.GetIntensity(), 12.0, 1e-3);
	EXPECT_NEAR(lasPt.GetColor().GetRed() / 65535.0, 255.0 / 255.0, 0.01);
	EXPECT_NEAR(lasPt.GetColor().GetGreen() / 65535.0, 128.0 / 255.0, 0.01);
	EXPECT_NEAR(lasPt.GetColor().GetBlue() / 65535.0, 64.0 / 255.0, 0.01);

	UFile::erase(path);
}

TEST(LASWriterTest, SaveXYZICloudWithCameraIds)
{
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::PointXYZI pt;
	pt.x = 1.f;
	pt.y = 2.f;
	pt.z = 3.f;
	pt.intensity = 99.f;
	cloud.push_back(pt);

	const std::vector<int> cameraIds(1, 7);
	const std::string path = tempLasPath("las");
	EXPECT_EQ(saveLASFile(path, cloud, cameraIds), 0);

	std::ifstream ifs(path.c_str(), std::ios::in | std::ios::binary);
	liblas::ReaderFactory factory;
	liblas::Reader reader = factory.CreateWithStream(ifs);
	ASSERT_TRUE(reader.ReadNextPoint());
	EXPECT_NEAR(reader.GetPoint().GetX(), 1.0, 1e-3);
	EXPECT_NEAR(reader.GetPoint().GetY(), 2.0, 1e-3);
	EXPECT_NEAR(reader.GetPoint().GetZ(), 3.0, 1e-3);
	EXPECT_NEAR(reader.GetPoint().GetIntensity(), 99.0, 1e-3);
	EXPECT_EQ(reader.GetPoint().GetPointSourceID(), 7u);

	UFile::erase(path);
}

#endif // RTABMAP_LIBLAS
