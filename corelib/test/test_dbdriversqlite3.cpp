#include <gtest/gtest.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/DBDriverSqlite3.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Parameters.h>
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
