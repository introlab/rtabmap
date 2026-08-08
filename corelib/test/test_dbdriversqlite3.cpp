#include <gtest/gtest.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/DBDriverSqlite3.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/DBReader.h>
#include <rtabmap/core/SensorCaptureInfo.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Link.h>
#include <opencv2/core.hpp>
#include <memory>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include "TestUtils.h"

using namespace rtabmap;

namespace {

std::string uniqueDbPath()
{
	static int counter = 0;
	return test::tempPath(uFormat("rtabmap_dbdriversqlite3_test_%d_%d.db", test::getPid(), ++counter));
}

class DBDriverSqlite3Fixture : public ::testing::Test
{
protected:
	void SetUp() override
	{
		dbPath_ = uniqueDbPath();
		driver_ = new DBDriverSqlite3();
		ASSERT_TRUE(driver_->openConnection(dbPath_, true));
	}

	void TearDown() override
	{
		if(driver_)
		{
			driver_->closeConnection(false);
			delete driver_;
			driver_ = nullptr;
		}
		if(!dbPath_.empty())
		{
			UFile::erase(dbPath_.c_str());
		}
	}

	void saveSignature(Signature * s)
	{
		driver_->asyncSave(s);
		driver_->emptyTrashes(false);
	}

	std::string dbPath_;
	DBDriverSqlite3 * driver_ = nullptr;
};

} // namespace

TEST(DBDriverSqlite3Test, CreateFactoryReturnsSqliteDriver)
{
	DBDriver * driver = DBDriver::create();
	ASSERT_NE(driver, nullptr);
	EXPECT_NE(dynamic_cast<DBDriverSqlite3 *>(driver), nullptr);
	driver->closeConnection(false);
	delete driver;
}

TEST(DBDriverSqlite3Test, EmptyUrlIsInMemory)
{
	DBDriverSqlite3 driver;
	ASSERT_TRUE(driver.openConnection(""));
	EXPECT_TRUE(driver.isInMemory());
	EXPECT_TRUE(driver.isConnected());
	EXPECT_FALSE(driver.getDatabaseVersion().empty());
	driver.closeConnection(false);
}

TEST(DBDriverSqlite3Test, FileBackedIsNotInMemory)
{
	const std::string path = uniqueDbPath();
	DBDriverSqlite3 driver;
	ASSERT_TRUE(driver.openConnection(path, true));
	EXPECT_FALSE(driver.isInMemory());
	EXPECT_EQ(driver.getUrl(), path);
	driver.closeConnection(false);
	UFile::erase(path.c_str());
}

TEST(DBDriverSqlite3Test, ParseParametersEnablesInMemory)
{
	const std::string path = uniqueDbPath();
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kDbSqlite3InMemory(), "true"));

	DBDriverSqlite3 driver(params);
	ASSERT_TRUE(driver.openConnection(path, true));
	EXPECT_TRUE(driver.isInMemory());
	EXPECT_TRUE(driver.isConnected());

	driver.asyncSave(new Signature(1));
	driver.emptyTrashes(false);
	EXPECT_EQ(driver.getTotalNodesSize(), 1);

	driver.closeConnection(false);
	UFile::erase(path.c_str());
}

TEST(DBDriverSqlite3Test, InMemorySaveToFileOnClose)
{
	const std::string path = uniqueDbPath();
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kDbSqlite3InMemory(), "true"));

	DBDriverSqlite3 driver(params);
	ASSERT_TRUE(driver.openConnection(path, true));
	EXPECT_TRUE(driver.isInMemory());

	driver.asyncSave(new Signature(1, 5, 1, 50.0, "sqlite_mem", Transform(1.f, 0.f, 0.f, 0.f, 0.f, 0.f)));
	driver.emptyTrashes(false);
	driver.closeConnection(true, path);

	DBDriverSqlite3 driver2;
	ASSERT_TRUE(driver2.openConnection(path));
	EXPECT_FALSE(driver2.isInMemory());

	Signature * loaded = driver2.loadSignature(1);
	ASSERT_NE(loaded, nullptr);
	EXPECT_EQ(loaded->getLabel(), "sqlite_mem");
	delete loaded;

	driver2.closeConnection(false);
	UFile::erase(path.c_str());
}

TEST_F(DBDriverSqlite3Fixture, SetPragmasWhileConnected)
{
	EXPECT_NO_THROW(driver_->setCacheSize(4000));
	EXPECT_NO_THROW(driver_->setJournalMode(1));
	EXPECT_NO_THROW(driver_->setSynchronous(1));
	EXPECT_NO_THROW(driver_->setTempStore(2));
	EXPECT_TRUE(driver_->isConnected());
}

TEST_F(DBDriverSqlite3Fixture, SaveAndLoadSignature)
{
	saveSignature(new Signature(1, 5, 7, 100.0, "node1", Transform(1.f, 2.f, 3.f, 0.f, 0.f, 0.f)));
	EXPECT_EQ(driver_->getTotalNodesSize(), 1);

	Signature * loaded = driver_->loadSignature(1);
	ASSERT_NE(loaded, nullptr);
	EXPECT_EQ(loaded->id(), 1);
	EXPECT_EQ(loaded->getLabel(), "node1");
	EXPECT_EQ(loaded->getWeight(), 7);
	delete loaded;
}

TEST_F(DBDriverSqlite3Fixture, ReopenPreservesData)
{
	saveSignature(new Signature(1, 5, 3, 50.0, "persist", Transform(0.5f, 0.f, 0.f, 0.f, 0.f, 0.f)));

	const std::string path = dbPath_;
	driver_->closeConnection(true);
	delete driver_;
	driver_ = nullptr;

	driver_ = new DBDriverSqlite3();
	ASSERT_TRUE(driver_->openConnection(path));

	Signature * loaded = driver_->loadSignature(1);
	ASSERT_NE(loaded, nullptr);
	EXPECT_EQ(loaded->getLabel(), "persist");
	delete loaded;
}

TEST_F(DBDriverSqlite3Fixture, ExecuteNoResultPragma)
{
	EXPECT_NO_THROW(driver_->executeNoResult("PRAGMA cache_size=8000;"));
	EXPECT_TRUE(driver_->isConnected());
}

// ---------------------------------------------------------------------------
// Full write/read round trip: build a small database with rich sensor data,
// then replay it with DBReader.
//
// The tests above save bare Signatures, which exercises the schema but not the
// payload paths (compressed image/depth/scan blobs, calibration, links) nor
// DBReader at all -- until now those only ran during the end-to-end replay
// tests, which need the fetched sample databases.
// ---------------------------------------------------------------------------

namespace {

// A frame carrying everything the payload paths serialise: RGB, depth,
// calibration and a laser scan.
SensorData makeRichData(int id, double stamp)
{
	cv::Mat rgb(60, 80, CV_8UC3);
	cv::randu(rgb, cv::Scalar::all(0), cv::Scalar::all(255));
	cv::Mat depth(60, 80, CV_16UC1);
	cv::randu(depth, cv::Scalar::all(500), cv::Scalar::all(4000));
	const CameraModel model(100.0, 100.0, 40.0, 30.0, Transform::getIdentity(), 0.0, cv::Size(80, 60));

	cv::Mat scanData(1, 50, CV_32FC3);
	for(int i = 0; i < 50; ++i)
	{
		scanData.at<cv::Vec3f>(0, i) = cv::Vec3f(0.01f * i, 0.02f * i, 0.0f);
	}

	// The driver persists the *compressed* buffers, so compress up front the
	// way Memory does before saving; raw-only data would be written as empty
	// blobs. setRGBDImage()/setLaserScan() detect a 1-row CV_8UC1 as compressed.
	SensorData data;
	data.setId(id);
	data.setStamp(stamp);
	data.setRGBDImage(compressImage2(rgb, ".png"), compressImage2(depth, ".png"), model);
	data.setLaserScan(LaserScan(compressData2(scanData), /*maxPoints=*/0, /*maxRange=*/0.0f,
			LaserScan::kXYZ));
	return data;
}

}  // namespace

TEST_F(DBDriverSqlite3Fixture, SavesAndLoadsRichSensorData)
{
	const Transform pose(1.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.3f);
	Signature * s = new Signature(10, 0, 1, 1.5, "node10", pose, Transform(), makeRichData(10, 1.5));
	saveSignature(s);

	std::list<Signature *> loaded;
	driver_->loadSignatures(std::list<int>(1, 10), loaded);
	ASSERT_EQ(1u, loaded.size());
	Signature * back = loaded.front();
	EXPECT_EQ(10, back->id());
	EXPECT_EQ(0, back->mapId());
	EXPECT_DOUBLE_EQ(1.5, back->getStamp());
	EXPECT_EQ("node10", back->getLabel());
	EXPECT_LT(back->getPose().getDistance(pose), 1e-4f);

	// Payloads come back compressed; ask the driver to fill them in.
	std::list<Signature *> toFill(1, back);
	driver_->loadNodeData(toFill);
	back->sensorData().uncompressData();
	EXPECT_FALSE(back->sensorData().imageRaw().empty()) << "image blob did not round-trip";
	EXPECT_FALSE(back->sensorData().depthRaw().empty()) << "depth blob did not round-trip";
	EXPECT_FALSE(back->sensorData().laserScanRaw().isEmpty()) << "scan blob did not round-trip";
	EXPECT_EQ(1u, back->sensorData().cameraModels().size());

	for(std::list<Signature *>::iterator iter = loaded.begin(); iter != loaded.end(); ++iter)
	{
		delete *iter;
	}
}

TEST_F(DBDriverSqlite3Fixture, DBReaderReplaysWrittenDatabase)
{
	// Three consecutive nodes with odometry poses and neighbour links, i.e.
	// the minimum a recorded session needs to be replayable.
	const int kNodes = 3;
	for(int i = 1; i <= kNodes; ++i)
	{
		const Transform pose(0.5f * i, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		Signature * s = new Signature(i, 0, 1, static_cast<double>(i), "", pose, Transform(),
				makeRichData(i, static_cast<double>(i)));
		if(i > 1)
		{
			const Transform motion(0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			s->addLink(Link(i, i - 1, Link::kNeighbor, motion.inverse()));
		}
		saveSignature(s);
	}
	// Close so the file is complete before DBReader opens it.
	driver_->closeConnection(true);
	delete driver_;
	driver_ = nullptr;

	std::unique_ptr<DBReader> reader(new DBReader(dbPath_, /*frameRate=*/0.0f,
			/*odometryIgnored=*/false));
	ASSERT_TRUE(reader->init()) << "DBReader could not open " << dbPath_;

	int frames = 0;
	Transform previousPose;
	while(frames < kNodes + 1)
	{
		SensorCaptureInfo info;
		SensorData data = reader->takeData(&info);
		if(data.id() == 0 && data.imageRaw().empty() && data.imageCompressed().empty())
		{
			break;  // end of database
		}
		++frames;
		EXPECT_FALSE(info.odomPose.isNull()) << "frame " << data.id() << " has no odometry pose";
		if(!previousPose.isNull() && !info.odomPose.isNull())
		{
			// Poses were written 50 cm apart along x.
			EXPECT_NEAR(previousPose.getDistance(info.odomPose), 0.5f, 1e-3f);
		}
		previousPose = info.odomPose;
	}
	EXPECT_EQ(kNodes, frames) << "DBReader returned " << frames << " of " << kNodes << " nodes";
}

// Only compressed buffers are persisted, so raw-only sensor data is stored with
// empty payloads (documented on DBDriver::asyncSave()). Pinned here because it
// is silent from the caller's point of view -- see makeRichData() above, which
// compresses first.
TEST_F(DBDriverSqlite3Fixture, RawOnlySensorDataIsNotPersisted)
{
	cv::Mat rgb(20, 20, CV_8UC3, cv::Scalar(10, 20, 30));
	const CameraModel model(100.0, 100.0, 10.0, 10.0, Transform::getIdentity(), 0.0, cv::Size(20, 20));
	SensorData raw(rgb, cv::Mat(), model, 42, 1.0);   // raw, never compressed
	ASSERT_FALSE(raw.imageRaw().empty());
	ASSERT_TRUE(raw.imageCompressed().empty());

	saveSignature(new Signature(42, 0, 1, 1.0, "", Transform::getIdentity(), Transform(), raw));

	std::list<Signature *> loaded;
	driver_->loadSignatures(std::list<int>(1, 42), loaded);
	ASSERT_EQ(1u, loaded.size());
	driver_->loadNodeData(loaded);
	loaded.front()->sensorData().uncompressData();
	EXPECT_TRUE(loaded.front()->sensorData().imageRaw().empty())
			<< "raw-only image unexpectedly survived a save/load round trip";
	delete loaded.front();
}

// ---------------------------------------------------------------------------
// Legacy schema round trips. Db/TargetVersion makes the driver create an older
// schema, which is how a database recorded by an earlier rtabmap looks. The
// readers for those layouts differ substantially -- e.g. link covariance was
// stored as separate rotVariance/transVariance columns before the full
// information matrix -- and none of it ran in the tests until now.
// ---------------------------------------------------------------------------

namespace {

class DBSchemaVersionTest : public ::testing::TestWithParam<const char *>
{
protected:
	void SetUp() override
	{
		dbPath_ = uniqueDbPath();
		ParametersMap parameters;
		parameters.insert(ParametersPair(Parameters::kDbTargetVersion(), GetParam()));
		driver_ = new DBDriverSqlite3(parameters);
		ASSERT_TRUE(driver_->openConnection(dbPath_, true))
				<< "could not create a " << GetParam() << " database";
	}

	void TearDown() override
	{
		if(driver_)
		{
			driver_->closeConnection(false);
			delete driver_;
			driver_ = nullptr;
		}
		UFile::erase(dbPath_.c_str());
	}

	void saveSignature(Signature * s)
	{
		driver_->asyncSave(s);
		driver_->emptyTrashes(false);
	}

	std::string dbPath_;
	DBDriverSqlite3 * driver_ = nullptr;
};

}  // namespace

TEST_P(DBSchemaVersionTest, NodesAndLinksSurviveARoundTrip)
{
	// Two nodes joined by a neighbour link carrying a non-default information
	// matrix: older schemas store that as rotVariance/transVariance, newer ones
	// as the full 6x6, so this exercises whichever reader the version needs.
	cv::Mat info = cv::Mat::eye(6, 6, CV_64FC1);
	info.at<double>(0,0) = info.at<double>(1,1) = info.at<double>(2,2) = 4.0;   // 1/transVariance
	info.at<double>(3,3) = info.at<double>(4,4) = info.at<double>(5,5) = 100.0; // 1/rotVariance

	const Transform motion(0.5f, 0.1f, 0.0f, 0.0f, 0.0f, 0.2f);
	for(int id = 1; id <= 2; ++id)
	{
		const Transform pose(0.5f * (id - 1), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		Signature * s = new Signature(id, 0, 1, static_cast<double>(id), "", pose, Transform(),
				makeRichData(id, static_cast<double>(id)));
		if(id == 2)
		{
			s->addLink(Link(2, 1, Link::kNeighbor, motion.inverse(), info));
		}
		saveSignature(s);
	}

	EXPECT_FALSE(driver_->getDatabaseVersion().empty());

	std::list<Signature *> loaded;
	driver_->loadSignatures(std::list<int>{1, 2}, loaded);
	ASSERT_EQ(2u, loaded.size()) << "nodes did not survive the round trip";

	// Payloads
	driver_->loadNodeData(loaded);
	for(Signature * s : loaded)
	{
		s->sensorData().uncompressData();
		EXPECT_FALSE(s->sensorData().imageRaw().empty()) << "node " << s->id() << " lost its image";
		EXPECT_EQ(1u, s->sensorData().cameraModels().size());
	}

	// Links: the second node must still point back at the first, with the
	// variances recovered from whatever columns this schema uses.
	std::multimap<int, Link> links;
	driver_->loadLinks(2, links);
	ASSERT_FALSE(links.empty()) << "link did not survive the round trip";
	const Link & link = links.begin()->second;
	EXPECT_EQ(1, link.to());
	EXPECT_LT(link.transform().getDistance(motion.inverse()), 1e-3f);
	EXPECT_NEAR(4.0, link.infMatrix().at<double>(0,0), 1e-6) << "translational variance lost";
	EXPECT_NEAR(100.0, link.infMatrix().at<double>(3,3), 1e-6) << "rotational variance lost";

	for(Signature * s : loaded)
	{
		delete s;
	}
}

INSTANTIATE_TEST_SUITE_P(
		Schemas,
		DBSchemaVersionTest,
		::testing::Values("0.16", "0.17", "0.18", "0.20", "0.22"),
		[](const ::testing::TestParamInfo<const char *> & info) {
			std::string name(info.param);
			for(size_t i = 0; i < name.size(); ++i)
			{
				if(!isalnum(name[i])) name[i] = '_';
			}
			return "v" + name;
		});
