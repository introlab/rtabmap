#include <gtest/gtest.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Recovery.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include "TestUtils.h"
#include <set>
#include <string>

using namespace rtabmap;

namespace {

const int kNodes = 3;

// Small database of kNodes nodes (images and odometry only).
void createDatabase(const std::string & dbPath, const std::string & targetVersion = "")
{
	UFile::erase(dbPath);
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kKpMaxFeatures(), "-1"));
	params.insert(ParametersPair(Parameters::kMemBinDataKept(), "true"));
	params.insert(ParametersPair(Parameters::kRGBDLinearUpdate(), "0"));
	params.insert(ParametersPair(Parameters::kRGBDAngularUpdate(), "0"));
	params.insert(ParametersPair(Parameters::kDbTargetVersion(), targetVersion));
	Rtabmap rtabmap;
	rtabmap.init(params, dbPath);
	const cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	for(int i = 1; i <= kNodes; ++i)
	{
		SensorData data(cv::Mat(8, 8, CV_8UC1, cv::Scalar(128)), i, double(i));
		ASSERT_TRUE(rtabmap.process(data, Transform(float(i), 0, 0, 0, 0, 0), covariance));
	}
	rtabmap.close(true);
}

} // namespace

TEST(RecoveryTest, RecoveredDatabaseKeepsTheInputDatabaseVersion)
{
	// The recovered database replaces the input one, so it is created with the
	// same version, not with the Db/TargetVersion saved in its parameters (usually
	// empty, i.e., latest version, or a stale one).
	const std::string dbPath = test::tempPath(uFormat("rtabmap_recovery_test_%d.db", test::getPid()));
	const std::string kOldVersion = "0.20.0";
	createDatabase(dbPath, kOldVersion);

	// Saved Db/TargetVersion different from the database version.
	{
		DBDriver * driver = DBDriver::create();
		ASSERT_TRUE(driver->openConnection(dbPath, false));
		ASSERT_EQ(kOldVersion, driver->getDatabaseVersion());
		driver->executeNoResult(uFormat("UPDATE Info SET parameters = replace(parameters, '%s:%s', '%s:');",
				Parameters::kDbTargetVersion().c_str(), kOldVersion.c_str(), Parameters::kDbTargetVersion().c_str()));
		ParametersMap saved = driver->getLastParameters();
		ASSERT_EQ(1u, saved.count(Parameters::kDbTargetVersion()));
		ASSERT_EQ("", saved.at(Parameters::kDbTargetVersion()));
		driver->closeConnection(true);
		delete driver;
	}

	std::string errorMsg;
	ASSERT_TRUE(databaseRecovery(dbPath, false, &errorMsg)) << errorMsg;

	{
		DBDriver * driver = DBDriver::create();
		ASSERT_TRUE(driver->openConnection(dbPath, false));
		EXPECT_EQ(kOldVersion, driver->getDatabaseVersion());
		std::set<int> ids;
		driver->getAllNodeIds(ids);
		EXPECT_EQ(kNodes, (int)ids.size());
		driver->closeConnection(false);
		delete driver;
	}
	UFile::erase(dbPath);
}

TEST(RecoveryTest, TildeInsideThePathIsNotTheHomeDirectory)
{
	// Only a leading '~' is the home directory: paths can contain '~' elsewhere,
	// e.g., Windows short paths like C:\Users\RUNNER~1\AppData\Local\Temp (the
	// temporary directory on Windows CI).
	const std::string dbPath = test::tempPath(uFormat("rtabmap_recovery~test_%d.db", test::getPid()));
	createDatabase(dbPath);

	std::string errorMsg;
	ASSERT_TRUE(databaseRecovery(dbPath, false, &errorMsg)) << errorMsg;

	DBDriver * driver = DBDriver::create();
	ASSERT_TRUE(driver->openConnection(dbPath, false));
	std::set<int> ids;
	driver->getAllNodeIds(ids);
	EXPECT_EQ(kNodes, (int)ids.size());
	driver->closeConnection(false);
	delete driver;
	UFile::erase(dbPath);
}
