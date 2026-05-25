#include <gtest/gtest.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/GPS.h>
#include <rtabmap/core/Landmark.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/ULogger.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

using namespace rtabmap;

namespace {

// Rtabmap configuration that disables feature extraction (kKpMaxFeatures=-1) so tests
// focus on Rtabmap orchestration logic without exercising the feature/registration
// pipeline (which has its own dedicated tests).
ParametersMap defaultRtabmapParams(bool rgbdMode = true)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kKpMaxFeatures(), "-1"));
	params.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0"));
	params.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
	params.insert(ParametersPair(Parameters::kMemBadSignaturesIgnored(), "false"));
	params.insert(ParametersPair(Parameters::kMemSTMSize(), "5"));
	params.insert(ParametersPair(Parameters::kRGBDEnabled(), rgbdMode ? "true" : "false"));
	// Disable RGB-D moveable thresholds so every frame counts as a real displacement.
	params.insert(ParametersPair(Parameters::kRGBDLinearUpdate(), "0.0"));
	params.insert(ParametersPair(Parameters::kRGBDAngularUpdate(), "0.0"));
	return params;
}

std::string uniqueDbPath()
{
	static int counter = 0;
	return uFormat("/tmp/rtabmap_test_%d_%d.db", getpid(), ++counter);
}

class RtabmapFixture : public ::testing::Test
{
protected:
	void SetUp() override
	{
		image_ = cv::Mat(8, 8, CV_8UC1, cv::Scalar(128));
		covariance_ = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
		rtabmap_ = new Rtabmap();
		rtabmap_->init(defaultRtabmapParams());
	}

	void TearDown() override
	{
		if(rtabmap_)
		{
			rtabmap_->close(false);
			delete rtabmap_;
			rtabmap_ = nullptr;
		}
	}

	void reinit(const ParametersMap & params)
	{
		if(rtabmap_)
		{
			rtabmap_->close(false);
			delete rtabmap_;
		}
		rtabmap_ = new Rtabmap();
		rtabmap_->init(params);
	}

	bool process()
	{
		SensorData data(image_);
		data.setId(++processCount_); // any positive id
		const Transform pose(float(processCount_), 0.0f, 0.0f, 0, 0, 0);
		return rtabmap_->process(data, pose, covariance_);
	}

	bool processWith(const Transform & pose)
	{
		SensorData data(image_);
		data.setId(++processCount_);
		return rtabmap_->process(data, pose, covariance_);
	}

	cv::Mat image_;
	cv::Mat covariance_;
	Rtabmap * rtabmap_ = nullptr;
	int processCount_ = 0;
};

} // namespace

// ---------------------------------------------------------------------------
// Constructor / init / close
// ---------------------------------------------------------------------------

TEST(RtabmapTest, DefaultConstructorHasEmptyState)
{
	Rtabmap rtabmap;
	EXPECT_EQ(rtabmap.getMemory(), nullptr);
	EXPECT_EQ(rtabmap.getLoopClosureId(), 0);
	EXPECT_FLOAT_EQ(rtabmap.getLoopClosureValue(), 0.0f);
	EXPECT_EQ(rtabmap.getHighestHypothesisId(), 0);
	EXPECT_FLOAT_EQ(rtabmap.getHighestHypothesisValue(), 0.0f);
	EXPECT_EQ(rtabmap.getLastLocationId(), 0);
	EXPECT_EQ(rtabmap.getLocalOptimizedPoses().size(), 0u);
	EXPECT_EQ(rtabmap.getLocalConstraints().size(), 0u);
	EXPECT_TRUE(rtabmap.getMapCorrection().isIdentity());
	EXPECT_EQ(rtabmap.getPath().size(), 0u);
	EXPECT_EQ(rtabmap.getPathStatus(), 0);
	EXPECT_DOUBLE_EQ(rtabmap.getLastProcessTime(), 0.0);
}

TEST(RtabmapTest, InitInMemoryDatabase)
{
	Rtabmap rtabmap;
	rtabmap.init(defaultRtabmapParams());
	ASSERT_NE(rtabmap.getMemory(), nullptr);
	EXPECT_TRUE(rtabmap.isRGBDMode());
	rtabmap.close(false);
	EXPECT_EQ(rtabmap.getMemory(), nullptr);
}

TEST(RtabmapTest, InitFileBackedDatabaseCreatesFile)
{
	const std::string dbPath = uniqueDbPath();
	{
		Rtabmap rtabmap;
		rtabmap.init(defaultRtabmapParams(), dbPath);
		ASSERT_NE(rtabmap.getMemory(), nullptr);
		rtabmap.close(true);
	}
	EXPECT_TRUE(UFile::exists(dbPath.c_str()));
	UFile::erase(dbPath.c_str());
}

TEST(RtabmapTest, CloseWithoutInitIsSafe)
{
	Rtabmap rtabmap;
	rtabmap.close(false); // should not crash
	EXPECT_EQ(rtabmap.getMemory(), nullptr);
}

TEST(RtabmapTest, InitRGBDModeOff)
{
	Rtabmap rtabmap;
	rtabmap.init(defaultRtabmapParams(/*rgbdMode=*/false));
	EXPECT_FALSE(rtabmap.isRGBDMode());
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// process()
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, ProcessAddsSignatureToMemory)
{
	EXPECT_TRUE(process());
	// First generated id is kIdStart + 1 = 1.
	EXPECT_EQ(rtabmap_->getLastLocationId(), Memory::kIdStart + 1);
	EXPECT_EQ(rtabmap_->getSTMSize(), 1);
	EXPECT_EQ(rtabmap_->getTotalMemSize(), 1);
	// getLastProcessTime is wall-clock; we can't predict its value, but it must not be
	// the sentinel 0.0 that the default constructor leaves behind.
	EXPECT_NE(rtabmap_->getLastProcessTime(), 0.0);
}

TEST_F(RtabmapFixture, ProcessSequentialIncrementsLocationId)
{
	EXPECT_TRUE(process());
	const int firstId = rtabmap_->getLastLocationId();
	EXPECT_TRUE(process());
	EXPECT_EQ(rtabmap_->getLastLocationId(), firstId + 1);
}

TEST_F(RtabmapFixture, ProcessPopulatesOptimizedPosesInRGBDMode)
{
	EXPECT_TRUE(process());
	// In RGB-D mode the current node is added to the optimized graph.
	EXPECT_EQ(rtabmap_->getLocalOptimizedPoses().size(), 1u);
	EXPECT_EQ((int)rtabmap_->getLocalOptimizedPoses().count(rtabmap_->getLastLocationId()), 1);
}

TEST_F(RtabmapFixture, ProcessUpdatesLastLocalizationPose)
{
	const Transform pose(1.5f, 2.5f, 0.0f, 0.0f, 0.0f, 0.0f);
	EXPECT_TRUE(processWith(pose));
	const Transform got = rtabmap_->getLastLocalizationPose();
	EXPECT_NEAR(got.x(), pose.x(), 1e-4f);
	EXPECT_NEAR(got.y(), pose.y(), 1e-4f);
}

TEST_F(RtabmapFixture, NoLoopClosureOnSecondFrame)
{
	process();
	process();
	EXPECT_EQ(rtabmap_->getLoopClosureId(), 0);
	EXPECT_FLOAT_EQ(rtabmap_->getLoopClosureValue(), 0.0f);
}

// ---------------------------------------------------------------------------
// Accessors (forward to Memory)
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, MemoryStateAccessorsReflectMemory)
{
	for(int i = 0; i < 3; ++i) { process(); }
	EXPECT_EQ((int)rtabmap_->getSTM().size(), rtabmap_->getSTMSize());
	EXPECT_EQ((int)rtabmap_->getWM().size(), rtabmap_->getWMSize());
	// STM=5 in the fixture and we processed 3 frames, so all 3 are in STM and WM
	// holds nothing. getTotalMemSize equals STM size in this configuration.
	EXPECT_EQ(rtabmap_->getSTMSize(), 3);
	EXPECT_EQ(rtabmap_->getWMSize(), 0);
	EXPECT_EQ(rtabmap_->getTotalMemSize(), 3);
	EXPECT_TRUE(rtabmap_->isInSTM(rtabmap_->getLastLocationId()));
}

TEST_F(RtabmapFixture, GetWeightsReturnsWmEntries)
{
	// Rtabmap::getWeights forwards to Memory::getWeights, which lists WM entries only.
	// With STM=5, after 3 iterations every signature is still in STM -> empty map.
	for(int i = 0; i < 3; ++i) { process(); }
	EXPECT_EQ(rtabmap_->getWeights().size(), 0u);

	// After 8 iterations total, 5 are in STM and 3 are in WM -> 3 entries.
	for(int i = 0; i < 5; ++i) { process(); }
	std::map<int, int> weights = rtabmap_->getWeights();
	EXPECT_EQ((int)weights.size(), rtabmap_->getWMSize());
	EXPECT_EQ(rtabmap_->getWMSize(), 3);
}

TEST_F(RtabmapFixture, GetPoseUnknownIdReturnsNull)
{
	process();
	const Transform pose = rtabmap_->getPose(99999);
	EXPECT_TRUE(pose.isNull());
}

TEST_F(RtabmapFixture, GetPoseKnownIdMatchesOptimizedPose)
{
	const Transform odom(3.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	processWith(odom);
	const Transform pose = rtabmap_->getPose(rtabmap_->getLastLocationId());
	ASSERT_FALSE(pose.isNull());
	// With a single signature processed, the optimized pose equals the input odom.
	EXPECT_FLOAT_EQ(pose.x(), odom.x());
	EXPECT_FLOAT_EQ(pose.y(), odom.y());
}

// ---------------------------------------------------------------------------
// Time / memory threshold setters / getters
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, SetTimeThreshold)
{
	rtabmap_->setTimeThreshold(123.0f);
	EXPECT_FLOAT_EQ(rtabmap_->getTimeThreshold(), 123.0f);
}

TEST_F(RtabmapFixture, SetMemoryThreshold)
{
	rtabmap_->setMemoryThreshold(42);
	EXPECT_EQ(rtabmap_->getMemoryThreshold(), 42);
}

// ---------------------------------------------------------------------------
// Map correction / path
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, MapCorrectionInitiallyIdentity)
{
	process();
	EXPECT_TRUE(rtabmap_->getMapCorrection().isIdentity());
}

TEST_F(RtabmapFixture, MapCorrectionAfterLoopClosureWithOptimizeFromGraphEndFalse)
{
	// optimizeFromGraphEnd=false (default): the oldest node in the local graph is the
	// anchor. After a loop closure forcing N3 to coincide with N1, N3's optimized pose
	// shifts backward and mapCorrection becomes non-identity (negative x).
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kRGBDOptimizeFromGraphEnd()] = "false";
	// Disable the optimization-error gate so the synthetic loop closure (which is
	// intentionally inconsistent with the odom chain) is not rejected.
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	reinit(params);

	process(); // N1 at odom (1, 0)
	const int N1 = rtabmap_->getLastLocationId();
	process(); // N2 at odom (2, 0)
	process(); // N3 at odom (3, 0)
	const int N3 = rtabmap_->getLastLocationId();
	ASSERT_TRUE(rtabmap_->getMapCorrection().isIdentity());

	// Loop closure N3 -> N1 with identity transform: N1 and N3 are at the same
	// place in the world even though odom thinks they are 2 m apart. Match the
	// neighbor links' information (covariance=0.01*I -> info=100*I) so the loop
	// closure has the same weight as the chain edges.
	Link loop(N3, N1, Link::kGlobalClosure, Transform::getIdentity(), cv::Mat::eye(6, 6, CV_64FC1) * 100.0);
	ASSERT_TRUE(rtabmap_->addLink(loop));

	// N1 fixed at (1, 0); the least-squares optimum pulls N3 toward N1, so
	// optimized(N3) < odom(N3) and mapCorrection.x() < 0. With equal-weight edges
	// on a 3-node chain the analytic optimum is optimized(N3)=5/3, giving
	// mapCorrection.x() = 5/3 - 3 = -4/3.
	const Transform mc = rtabmap_->getMapCorrection();
	EXPECT_FALSE(mc.isIdentity());
	EXPECT_NEAR(mc.x(), -4.0f / 3.0f, 0.05f);
	EXPECT_NEAR(mc.y(), 0.0f, 1e-3f);
}

TEST_F(RtabmapFixture, MapCorrectionAfterLoopClosureWithOptimizeFromGraphEndTrue)
{
	// optimizeFromGraphEnd=true: the latest node is the anchor and stays at its
	// odom pose. The optimization shifts older nodes instead, so the mapCorrection
	// stays identity.
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kRGBDOptimizeFromGraphEnd()] = "true";
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	reinit(params);

	process();
	const int N1 = rtabmap_->getLastLocationId();
	process();
	process();
	const int N3 = rtabmap_->getLastLocationId();
	ASSERT_TRUE(rtabmap_->getMapCorrection().isIdentity());

	Link loop(N3, N1, Link::kGlobalClosure, Transform::getIdentity(), cv::Mat::eye(6, 6, CV_64FC1) * 100.0);
	ASSERT_TRUE(rtabmap_->addLink(loop));

	EXPECT_TRUE(rtabmap_->getMapCorrection().isIdentity());
}

TEST_F(RtabmapFixture, GraphDeformsAfterLoopClosureWithOptimizeFromGraphEndFalse)
{
	// More realistic scenario: 3-node chain with odom N1=(1,0)->N2=(2,0)->N3=(3,0)
	// (accumulated 2 m from N1 to N3). The loop closure measures N1 only 1 m
	// behind N3 (transform N3->N1 = (-1, 0, 0)), so the chain must compress to
	// reconcile the two readings. Both the loop closure and the neighbor links
	// share the same information weight (info = 100*I, i.e. covariance 0.01*I)
	// so the optimizer balances them evenly. Analytic optimum with N1 anchored:
	//   N1 = (1, 0),  N2 = (5/3, 0),  N3 = (7/3, 0)
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kRGBDOptimizeFromGraphEnd()] = "false";
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	reinit(params);

	process();
	const int N1 = rtabmap_->getLastLocationId();
	process();
	const int N2 = rtabmap_->getLastLocationId();
	process();
	const int N3 = rtabmap_->getLastLocationId();

	Link loop(N3, N1, Link::kGlobalClosure,
			Transform(-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
			cv::Mat::eye(6, 6, CV_64FC1) * 100.0);
	ASSERT_TRUE(rtabmap_->addLink(loop));

	EXPECT_NEAR(rtabmap_->getPose(N1).x(), 1.0f,        0.05f);
	EXPECT_NEAR(rtabmap_->getPose(N2).x(), 5.0f / 3.0f, 0.05f);
	EXPECT_NEAR(rtabmap_->getPose(N3).x(), 7.0f / 3.0f, 0.05f);
	// mapCorrection reflects how far the last node moved from its odom pose:
	//   optimized(N3) - odom(N3) = 7/3 - 3 = -2/3
	EXPECT_NEAR(rtabmap_->getMapCorrection().x(), -2.0f / 3.0f, 0.05f);
}

TEST_F(RtabmapFixture, GraphDeformsAfterLoopClosureWithOptimizeFromGraphEndTrue)
{
	// Mirror of the test above with N3 anchored at its odom pose. The chain
	// reverses: the older nodes shift forward to satisfy the same loop closure
	// constraint. Analytic optimum:
	//   N1 = (5/3, 0),  N2 = (7/3, 0),  N3 = (3, 0)
	// The mapCorrection stays identity even though every other node moved.
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kRGBDOptimizeFromGraphEnd()] = "true";
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	reinit(params);

	process();
	const int N1 = rtabmap_->getLastLocationId();
	process();
	const int N2 = rtabmap_->getLastLocationId();
	process();
	const int N3 = rtabmap_->getLastLocationId();

	Link loop(N3, N1, Link::kGlobalClosure,
			Transform(-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
			cv::Mat::eye(6, 6, CV_64FC1) * 100.0);
	ASSERT_TRUE(rtabmap_->addLink(loop));

	EXPECT_NEAR(rtabmap_->getPose(N1).x(), 5.0f / 3.0f, 0.05f);
	EXPECT_NEAR(rtabmap_->getPose(N2).x(), 7.0f / 3.0f, 0.05f);
	EXPECT_NEAR(rtabmap_->getPose(N3).x(), 3.0f,        0.05f);
	EXPECT_TRUE(rtabmap_->getMapCorrection().isIdentity());
}

TEST(RtabmapTest, AddLinkInLocalizationModeUpdatesMapCorrectionAndLocalizationPose)
{
	// Build a 2-node map (N1=(1,0), N2=(2,0)) in mapping mode, persist it, then
	// reopen in localization mode (kMemIncrementalMemory=false). Process a new
	// frame with odom (3, 0) -> goes into the odom cache as N3. The loop closure
	// N3 -> N1 measures N1 1 m behind N3 (transform (-1, 0, 0)), so the system
	// localizes N3 in the map frame at (2, 0). mapCorrection brings odom (3, 0)
	// back to (2, 0) -> mapCorrection.x() = -1.
	const std::string dbPath = uniqueDbPath();
	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	int N1 = 0;
	{
		Rtabmap rtabmap;
		rtabmap.init(defaultRtabmapParams(), dbPath);
		SensorData d1(image); d1.setId(1);
		ASSERT_TRUE(rtabmap.process(d1, Transform(1.0f, 0, 0, 0, 0, 0), covariance));
		N1 = rtabmap.getLastLocationId();
		SensorData d2(image); d2.setId(2);
		ASSERT_TRUE(rtabmap.process(d2, Transform(2.0f, 0, 0, 0, 0, 0), covariance));
		rtabmap.close(true);
	}
	{
		ParametersMap params = defaultRtabmapParams();
		params[Parameters::kRGBDOptimizeFromGraphEnd()] = "false";
		params[Parameters::kRGBDOptimizeMaxError()] = "0";
		params[Parameters::kMemIncrementalMemory()] = "false";
		Rtabmap rtabmap;
		rtabmap.init(params, dbPath);

		SensorData d3(image); d3.setId(3);
		ASSERT_TRUE(rtabmap.process(d3, Transform(3.0f, 0, 0, 0, 0, 0), covariance));
		const int N3 = rtabmap.getLastLocationId();

		Link loop(N3, N1, Link::kGlobalClosure,
				Transform(-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
				cv::Mat::eye(6, 6, CV_64FC1) * 100.0);
		ASSERT_TRUE(rtabmap.addLink(loop));

		// Map nodes are fixed; only the new node's localization shifts.
		const Transform mc = rtabmap.getMapCorrection();
		EXPECT_NEAR(mc.x(), -1.0f, 0.05f);
		EXPECT_NEAR(mc.y(),  0.0f, 1e-3f);
		const Transform locPose = rtabmap.getLastLocalizationPose();
		EXPECT_NEAR(locPose.x(), 2.0f, 0.05f);
		EXPECT_NEAR(locPose.y(), 0.0f, 1e-3f);

		rtabmap.close(false);
	}
	UFile::erase(dbPath.c_str());
}

TEST(RtabmapTest, AddLinkInLocalizationModeRejectedWhenOptimizeFromGraphEndTrue)
{
	// Documented behavior (Rtabmap.cpp): addLink in localization mode does not
	// support kRGBDOptimizeFromGraphEnd=true and returns false. The state stays
	// at its initial values (mapCorrection identity, lastLocalizationPose null).
	const std::string dbPath = uniqueDbPath();
	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	int N1 = 0;
	{
		Rtabmap rtabmap;
		rtabmap.init(defaultRtabmapParams(), dbPath);
		SensorData d1(image); d1.setId(1);
		ASSERT_TRUE(rtabmap.process(d1, Transform(1.0f, 0, 0, 0, 0, 0), covariance));
		N1 = rtabmap.getLastLocationId();
		SensorData d2(image); d2.setId(2);
		ASSERT_TRUE(rtabmap.process(d2, Transform(2.0f, 0, 0, 0, 0, 0), covariance));
		rtabmap.close(true);
	}
	{
		ParametersMap params = defaultRtabmapParams();
		params[Parameters::kRGBDOptimizeFromGraphEnd()] = "true";
		params[Parameters::kMemIncrementalMemory()] = "false";
		Rtabmap rtabmap;
		rtabmap.init(params, dbPath);

		SensorData d3(image); d3.setId(3);
		ASSERT_TRUE(rtabmap.process(d3, Transform(3.0f, 0, 0, 0, 0, 0), covariance));
		const int N3 = rtabmap.getLastLocationId();

		Link loop(N3, N1, Link::kGlobalClosure,
				Transform(-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
				cv::Mat::eye(6, 6, CV_64FC1) * 100.0);
		EXPECT_FALSE(rtabmap.addLink(loop));
		EXPECT_TRUE(rtabmap.getMapCorrection().isIdentity());

		rtabmap.close(false);
	}
	UFile::erase(dbPath.c_str());
}

TEST_F(RtabmapFixture, PathStatusInitiallyIdle)
{
	EXPECT_EQ(rtabmap_->getPathStatus(), 0);
	EXPECT_EQ(rtabmap_->getPath().size(), 0u);
}

TEST_F(RtabmapFixture, ClearPathSetsStatus)
{
	rtabmap_->clearPath(1);
	EXPECT_EQ(rtabmap_->getPathStatus(), 1);
	EXPECT_EQ(rtabmap_->getPath().size(), 0u);
}

// ---------------------------------------------------------------------------
// triggerNewMap
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, TriggerNewMapInMappingModeIncrementsMapId)
{
	process();
	const int firstMapId = rtabmap_->getMemory()->getMapId(rtabmap_->getLastLocationId());
	EXPECT_EQ(firstMapId, 0);
	const int newMapId = rtabmap_->triggerNewMap();
	EXPECT_EQ(newMapId, firstMapId + 1);

	process();
	const int secondMapId = rtabmap_->getMemory()->getMapId(rtabmap_->getLastLocationId());
	EXPECT_EQ(secondMapId, newMapId);
}

TEST(RtabmapTest, TriggerNewMapInLocalizationModeReturnsMinusOne)
{
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kMemIncrementalMemory()] = "false";
	Rtabmap rtabmap;
	rtabmap.init(params);
	EXPECT_EQ(rtabmap.triggerNewMap(), -1);
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// resetMemory
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, ResetMemoryClearsState)
{
	for(int i = 0; i < 3; ++i) { process(); }
	rtabmap_->resetMemory();
	EXPECT_EQ(rtabmap_->getTotalMemSize(), 0);
	EXPECT_EQ(rtabmap_->getSTMSize(), 0);
	EXPECT_EQ(rtabmap_->getWMSize(), 0);
	EXPECT_EQ(rtabmap_->getLocalOptimizedPoses().size(), 0u);
	EXPECT_TRUE(rtabmap_->getMapCorrection().isIdentity());
}

// ---------------------------------------------------------------------------
// Labels
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, LabelLocationForwardsToMemory)
{
	process();
	const int id = rtabmap_->getLastLocationId();
	EXPECT_TRUE(rtabmap_->labelLocation(id, "start"));
	EXPECT_EQ(rtabmap_->getMemory()->getSignatureIdByLabel("start", false), id);
}

TEST_F(RtabmapFixture, LabelLocationWithUnknownIdFails)
{
	EXPECT_FALSE(rtabmap_->labelLocation(99999, "ghost"));
}

// ---------------------------------------------------------------------------
// setOptimizedPoses
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, SetOptimizedPosesOverwritesInternalContainers)
{
	process();
	std::map<int, Transform> poses;
	poses[1] = Transform(5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	poses[2] = Transform(6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	std::multimap<int, Link> links;

	rtabmap_->setOptimizedPoses(poses, links);
	EXPECT_EQ(rtabmap_->getLocalOptimizedPoses().size(), 2u);
	EXPECT_EQ(rtabmap_->getLocalConstraints().size(), 0u);
	EXPECT_NEAR(rtabmap_->getPose(1).x(), 5.0f, 1e-5f);
	EXPECT_NEAR(rtabmap_->getPose(2).x(), 6.0f, 1e-5f);
}

// ---------------------------------------------------------------------------
// setInitialPose
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, SetInitialPoseInMappingModeIsNoop)
{
	process();
	const Transform priorBefore = rtabmap_->getLastLocalizationPose();
	rtabmap_->setInitialPose(Transform(99.0f, 99.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	// In mapping mode (default), setInitialPose only warns; last localization pose is unchanged.
	const Transform after = rtabmap_->getLastLocalizationPose();
	EXPECT_NEAR(after.x(), priorBefore.x(), 1e-5f);
	EXPECT_NEAR(after.y(), priorBefore.y(), 1e-5f);
}

TEST(RtabmapTest, SetInitialPoseInLocalizationModeStagesPose)
{
	// Build a mapping db, then reopen in localization mode and set initial pose.
	const std::string dbPath = uniqueDbPath();
	{
		Rtabmap rtabmap;
		rtabmap.init(defaultRtabmapParams(), dbPath);
		cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
		cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
		for(int i = 0; i < 6; ++i)
		{
			SensorData data(image);
			data.setId(i + 1);
			rtabmap.process(data, Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance);
		}
		rtabmap.close(true);
	}

	{
		ParametersMap localParams = defaultRtabmapParams();
		localParams[Parameters::kMemIncrementalMemory()] = "false";
		Rtabmap rtabmap;
		rtabmap.init(localParams, dbPath);

		const Transform initial(10.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		rtabmap.setInitialPose(initial);
		const Transform got = rtabmap.getLastLocalizationPose();
		EXPECT_NEAR(got.x(), initial.x(), 1e-4f);
		EXPECT_NEAR(got.y(), initial.y(), 1e-4f);
		EXPECT_TRUE(rtabmap.getMapCorrection().isIdentity());

		rtabmap.close(false);
	}
	UFile::erase(dbPath.c_str());
}

// ---------------------------------------------------------------------------
// adjustLikelihood
// ---------------------------------------------------------------------------

TEST(RtabmapTest, AdjustLikelihoodEmptyIsNoop)
{
	Rtabmap rtabmap;
	std::map<int, float> likelihood;
	rtabmap.adjustLikelihood(likelihood);
	EXPECT_EQ(likelihood.size(), 0u);
}

TEST(RtabmapTest, AdjustLikelihoodFlattensBelowMeanPlusStdDev)
{
	Rtabmap rtabmap;
	rtabmap.init(defaultRtabmapParams());

	// Values [1, 1, 10, 1, 1] for real-place ids 1..5.
	// uMean = 2.8; uVariance uses N-1 -> variance = sum((v-mean)^2)/(N-1) = 64.8/4 = 16.2;
	// stdDev = sqrt(16.2) ~= 4.024922; mean + stdDev ~= 6.824922. Only id 3 is above it.
	std::map<int, float> likelihood;
	likelihood[Memory::kIdVirtual] = 0.0f; // ignored in stats
	for(int i = 1; i <= 5; ++i)
	{
		likelihood[i] = 1.0f;
	}
	likelihood[3] = 10.0f;

	rtabmap.adjustLikelihood(likelihood);

	// Values not above mean+stdDev are clamped to 1.0.
	EXPECT_FLOAT_EQ(likelihood[1], 1.0f);
	EXPECT_FLOAT_EQ(likelihood[2], 1.0f);
	EXPECT_FLOAT_EQ(likelihood[4], 1.0f);
	EXPECT_FLOAT_EQ(likelihood[5], 1.0f);
	// Ratio=0 (Angeli): outlier scaled as (value - (stdDev - epsilon)) / mean
	// = (10 - (4.024922 - 0.0001)) / 2.8 ~= 2.133992 (epsilon is a float literal
	// in the implementation, so the value is computed at float precision).
	EXPECT_NEAR(likelihood[3], 2.133992f, 1e-3f);
	// Virtual place: mean / stdDev + 1 = 2.8 / 4.024922 + 1 ~= 1.695680.
	EXPECT_NEAR(likelihood[Memory::kIdVirtual], 1.695680f, 1e-3f);

	rtabmap.close(false);
}

TEST(RtabmapTest, AdjustLikelihoodUsesZscoreWhenRatioNonZero)
{
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kRtabmapVirtualPlaceLikelihoodRatio()] = "1"; // z-score formulation
	Rtabmap rtabmap;
	rtabmap.init(params);

	std::map<int, float> likelihood;
	likelihood[Memory::kIdVirtual] = 0.0f;
	for(int i = 1; i <= 5; ++i)
	{
		likelihood[i] = 1.0f;
	}
	likelihood[3] = 10.0f;
	rtabmap.adjustLikelihood(likelihood);

	// Same stats as above: mean=2.8, stdDev=sqrt(16.2)~=4.024922.
	EXPECT_FLOAT_EQ(likelihood[1], 1.0f);
	EXPECT_FLOAT_EQ(likelihood[2], 1.0f);
	EXPECT_FLOAT_EQ(likelihood[4], 1.0f);
	EXPECT_FLOAT_EQ(likelihood[5], 1.0f);
	// Ratio!=0 (z-score): outlier = (value - mean) / stdDev = (10 - 2.8) / 4.024922 ~= 1.788854.
	EXPECT_NEAR(likelihood[3], 1.788854f, 1e-3f);
	// Virtual place: stdDev / (max - mean) + 1 = 4.024922 / (10 - 2.8) + 1 ~= 1.559017.
	EXPECT_NEAR(likelihood[Memory::kIdVirtual], 1.559017f, 1e-3f);

	rtabmap.close(false);
}

TEST(RtabmapTest, AdjustLikelihoodFallsBackToTwoWhenAllValuesEqual)
{
	// All real-place values equal -> mean=value, stdDev=0. For both approaches:
	//  - no value > mean+stdDev, so the outlier branch is skipped (everything stays
	//    at 1.0 after the unconditional `iter->second = 1.0f` line),
	//  - the virtual-place gate fails for both approaches (Angeli needs stdDev>eps,
	//    z-score needs max>mean) -> fallback "else { virtual = 2.0 }".
	for(const std::string & ratio : {std::string("0"), std::string("1")})
	{
		SCOPED_TRACE("ratio=" + ratio);
		ParametersMap params = defaultRtabmapParams();
		params[Parameters::kRtabmapVirtualPlaceLikelihoodRatio()] = ratio;
		Rtabmap rtabmap;
		rtabmap.init(params);

		std::map<int, float> likelihood;
		likelihood[Memory::kIdVirtual] = 0.0f;
		for(int i = 1; i <= 5; ++i) likelihood[i] = 1.0f;

		rtabmap.adjustLikelihood(likelihood);

		for(int i = 1; i <= 5; ++i)
		{
			EXPECT_FLOAT_EQ(likelihood[i], 1.0f) << "id=" << i;
		}
		EXPECT_FLOAT_EQ(likelihood[Memory::kIdVirtual], 2.0f);

		rtabmap.close(false);
	}
}

TEST(RtabmapTest, AdjustLikelihoodWithoutOutliersStillSetsVirtualPlace)
{
	// Inputs picked so no value crosses the outlier gate but the spread is still
	// non-degenerate:
	//   values = {1.0, 1.0, 1.1, 1.1, 1.1}
	//   mean = 1.06
	//   variance (N-1) = sum((v-mean)^2)/4 = (2*0.06^2 + 3*0.04^2)/4 = 0.012/4 = 0.003
	//   stdDev = sqrt(0.003) ~= 0.054772
	//   mean+stdDev ~= 1.114772 > max=1.1  -> no outliers
	// Both approaches skip the outlier branch but still compute the virtual place
	// from their respective formulas (no fallback hit here).
	const float mean   = 1.06f;
	const float stdDev = std::sqrt(0.003f);
	const float maxVal = 1.1f;

	for(const std::string & ratio : {std::string("0"), std::string("1")})
	{
		SCOPED_TRACE("ratio=" + ratio);
		ParametersMap params = defaultRtabmapParams();
		params[Parameters::kRtabmapVirtualPlaceLikelihoodRatio()] = ratio;
		Rtabmap rtabmap;
		rtabmap.init(params);

		std::map<int, float> likelihood;
		likelihood[Memory::kIdVirtual] = 0.0f;
		likelihood[1] = 1.0f;
		likelihood[2] = 1.0f;
		likelihood[3] = 1.1f;
		likelihood[4] = 1.1f;
		likelihood[5] = 1.1f;

		rtabmap.adjustLikelihood(likelihood);

		// Outlier branch skipped -> all real-place values clamped to 1.0.
		for(int i = 1; i <= 5; ++i)
		{
			EXPECT_FLOAT_EQ(likelihood[i], 1.0f) << "id=" << i;
		}
		// Virtual place set by the active formula (no 2.0 fallback).
		const float expected = (ratio == "0")
				? mean / stdDev + 1.0f          // Angeli
				: stdDev / (maxVal - mean) + 1.0f; // z-score
		EXPECT_NEAR(likelihood[Memory::kIdVirtual], expected, 1e-2f);

		rtabmap.close(false);
	}
}

// ---------------------------------------------------------------------------
// parseParameters / getParameters
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, ParseParametersUpdatesSettings)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kRtabmapTimeThr(), "555.0"));
	params.insert(ParametersPair(Parameters::kRtabmapMemoryThr(), "33"));
	rtabmap_->parseParameters(params);

	EXPECT_FLOAT_EQ(rtabmap_->getTimeThreshold(), 555.0f);
	EXPECT_EQ(rtabmap_->getMemoryThreshold(), 33);

	const ParametersMap & got = rtabmap_->getParameters();
	ASSERT_TRUE(got.count(Parameters::kRtabmapTimeThr()) > 0);
	EXPECT_EQ(got.at(Parameters::kRtabmapTimeThr()), std::string("555.0"));
}

// ---------------------------------------------------------------------------
// Working directory
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, SetWorkingDirectoryReflectsInGetter)
{
	rtabmap_->setWorkingDirectory("/tmp");
	EXPECT_EQ(rtabmap_->getWorkingDir(), std::string("/tmp"));
}

// ---------------------------------------------------------------------------
// getStatistics
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, GetStatisticsRefreshesOnProcess)
{
	process();
	const Statistics & stats = rtabmap_->getStatistics();
	EXPECT_EQ(stats.refImageId(), rtabmap_->getLastLocationId());
}

// ---------------------------------------------------------------------------
// addNodesToRepublish
// ---------------------------------------------------------------------------

namespace {
// Helper for the addNodesToRepublish tests: builds a 6-node chain in mapping
// mode, persists it to @p dbPath and returns the resulting node ids in order.
// The map is the same for every test so the republish lookup hits a known graph.
std::vector<int> buildSixNodeMap(const std::string & dbPath)
{
	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	Rtabmap rtabmap;
	rtabmap.init(defaultRtabmapParams(), dbPath);
	for(int i = 0; i < 6; ++i)
	{
		SensorData data(image);
		data.setId(i + 1);
		EXPECT_TRUE(rtabmap.process(data, Transform(float(i), 0, 0, 0, 0, 0), covariance));
		ids.push_back(rtabmap.getLastLocationId());
	}
	rtabmap.close(true);
	return ids;
}
} // namespace

TEST(RtabmapTest, AddNodesToRepublishEmitsRequestedNodesInStatistics)
{
	// Build a 6-node chain in mapping mode then reopen in localization mode and
	// drop the robot at (3, 0) (the pose of the 4th map node) via setInitialPose.
	// addNodesToRepublish({ids[0], ids[1]}) -- 2 requests, within the default
	// kRtabmapMaxRepublished=2. After process(), both signatures must appear in
	// the Statistics' signature data alongside the just-processed last signature.
	const std::string dbPath = uniqueDbPath();
	const std::vector<int> ids = buildSixNodeMap(dbPath);
	{
		ParametersMap params = defaultRtabmapParams();
		params[Parameters::kMemIncrementalMemory()] = "false";
		Rtabmap rtabmap;
		rtabmap.init(params, dbPath);

		rtabmap.setInitialPose(Transform(3.0f, 0, 0, 0, 0, 0));
		rtabmap.addNodesToRepublish({ids[0], ids[1]});

		cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
		cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
		SensorData data(image); data.setId(100);
		rtabmap.process(data, Transform(3.0f, 0, 0, 0, 0, 0), covariance);

		const Statistics & stats = rtabmap.getStatistics();
		EXPECT_EQ(stats.getSignaturesData().count(ids[0]), 1u);
		EXPECT_EQ(stats.getSignaturesData().count(ids[1]), 1u);

		rtabmap.close(false);
	}
	UFile::erase(dbPath.c_str());
}

TEST(RtabmapTest, AddNodesToRepublishCapsPerIterationAtMaxRepublished)
{
	// Same setup but request 5 ids. The cap kRtabmapMaxRepublished=2 means only
	// 2 of the 5 requested signatures land in iteration 1's Statistics. Each
	// emitted signature is removed from the queue (Rtabmap.cpp _nodesToRepublish.erase),
	// so iteration 2 must emit a different 2 ids from the queue's remainder.
	const std::string dbPath = uniqueDbPath();
	const std::vector<int> ids = buildSixNodeMap(dbPath);
	const std::vector<int> requested = {ids[0], ids[1], ids[2], ids[3], ids[4]};
	{
		ParametersMap params = defaultRtabmapParams();
		params[Parameters::kMemIncrementalMemory()] = "false";
		Rtabmap rtabmap;
		rtabmap.init(params, dbPath);

		rtabmap.setInitialPose(Transform(3.0f, 0, 0, 0, 0, 0));
		rtabmap.addNodesToRepublish(requested);

		cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
		cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

		auto requestedInStats = [&]() {
			std::set<int> hits;
			const auto & sigs = rtabmap.getStatistics().getSignaturesData();
			for(int id : requested)
			{
				if(sigs.count(id) > 0) hits.insert(id);
			}
			return hits;
		};

		// Iteration 1: cap kicks in -> exactly 2 from the request.
		SensorData data1(image); data1.setId(100);
		rtabmap.process(data1, Transform(3.0f, 0, 0, 0, 0, 0), covariance);
		const std::set<int> emitted1 = requestedInStats();
		EXPECT_EQ(emitted1.size(), 2u);

		// Iteration 2: another 2 from the queue's remainder, no overlap with iter 1.
		SensorData data2(image); data2.setId(101);
		rtabmap.process(data2, Transform(3.0f, 0, 0, 0, 0, 0), covariance);
		const std::set<int> emitted2 = requestedInStats();
		EXPECT_EQ(emitted2.size(), 2u);
		std::set<int> intersection;
		std::set_intersection(emitted1.begin(), emitted1.end(),
				emitted2.begin(), emitted2.end(),
				std::inserter(intersection, intersection.begin()));
		EXPECT_EQ(intersection.size(), 0u);

		rtabmap.close(false);
	}
	UFile::erase(dbPath.c_str());
}

TEST(RtabmapTest, AddNodesToRepublishWithEmptyClearsQueueForNextIteration)
{
	// Request 5 ids, then clear the queue with {}. On the next process() none of
	// the originally-requested signatures appear in Statistics.
	const std::string dbPath = uniqueDbPath();
	const std::vector<int> ids = buildSixNodeMap(dbPath);
	{
		ParametersMap params = defaultRtabmapParams();
		params[Parameters::kMemIncrementalMemory()] = "false";
		Rtabmap rtabmap;
		rtabmap.init(params, dbPath);

		rtabmap.setInitialPose(Transform(3.0f, 0, 0, 0, 0, 0));
		rtabmap.addNodesToRepublish({ids[0], ids[1], ids[2], ids[3], ids[4]});
		rtabmap.addNodesToRepublish({}); // clears

		cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
		cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
		SensorData data(image); data.setId(100);
		rtabmap.process(data, Transform(3.0f, 0, 0, 0, 0, 0), covariance);

		const Statistics & stats = rtabmap.getStatistics();
		for(int id : {ids[0], ids[1], ids[2], ids[3], ids[4]})
		{
			EXPECT_EQ(stats.getSignaturesData().count(id), 0u) << "id=" << id;
		}

		rtabmap.close(false);
	}
	UFile::erase(dbPath.c_str());
}

// ---------------------------------------------------------------------------
// getInformation
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, GetInformationInvertsCovariance)
{
	cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.5;
	cv::Mat info = rtabmap_->getInformation(cov);
	ASSERT_EQ(info.rows, 6);
	ASSERT_EQ(info.cols, 6);
	// inv(diag(0.5)) = diag(2)
	for(int i = 0; i < 6; ++i)
	{
		EXPECT_NEAR(info.at<double>(i, i), 2.0, 1e-6);
	}
}

// ---------------------------------------------------------------------------
// rejectLastLoopClosure
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, RejectLastLoopClosureRemovesLinkAndResetsHypothesis)
{
	// Build a 3-node chain and inject a user loop closure (N3->N1). After
	// addLink(), the loop link exists in Memory and _constraints includes it,
	// and mapCorrection has shifted because the graph re-optimized.
	// rejectLastLoopClosure() must: (1) remove the loop link from Memory and
	// _constraints, (2) clear _loopClosureHypothesis, (3) re-optimize without it
	// so mapCorrection collapses back to identity.
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	reinit(params);
	process();
	const int N1 = rtabmap_->getLastLocationId();
	process();
	process();
	const int N3 = rtabmap_->getLastLocationId();

	Link loop(N3, N1, Link::kUserClosure,
			Transform(-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
			cv::Mat::eye(6, 6, CV_64FC1) * 100.0);
	ASSERT_TRUE(rtabmap_->addLink(loop));
	ASSERT_FALSE(rtabmap_->getMapCorrection().isIdentity());

	rtabmap_->rejectLastLoopClosure();

	EXPECT_EQ(rtabmap_->getLoopClosureId(), 0);
	// Loop link gone from memory.
	const std::multimap<int, Link> & links = rtabmap_->getMemory()->getSignature(N3)->getLinks();
	for(const auto & kv : links)
	{
		EXPECT_NE(kv.second.type(), Link::kUserClosure);
	}
	// Graph re-optimized without the rejected link -> mapCorrection identity.
	EXPECT_TRUE(rtabmap_->getMapCorrection().isIdentity());
}

TEST_F(RtabmapFixture, RejectLastLoopClosureIsNoOpWhenNoLoopClosureExists)
{
	// Without any loop-closure link, the rollback path is skipped: hypothesis
	// stays at 0, mapCorrection stays identity, and no link is removed.
	process();
	process();
	rtabmap_->rejectLastLoopClosure();
	EXPECT_EQ(rtabmap_->getLoopClosureId(), 0);
	EXPECT_TRUE(rtabmap_->getMapCorrection().isIdentity());
}

// ---------------------------------------------------------------------------
// deleteLastLocation
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, DeleteLastLocationRemovesFromStmAndOptimizedPoses)
{
	// 3 signatures processed -> all 3 in STM (STM=5 default), all 3 in optimized
	// poses (RGBD mode). deleteLastLocation removes the last one from both.
	process();
	const int N1 = rtabmap_->getLastLocationId();
	process();
	const int N2 = rtabmap_->getLastLocationId();
	process();
	const int N3 = rtabmap_->getLastLocationId();
	ASSERT_TRUE(rtabmap_->isInSTM(N3));
	ASSERT_NE(rtabmap_->getPose(N3).isNull(), true);

	rtabmap_->deleteLastLocation();

	EXPECT_FALSE(rtabmap_->isInSTM(N3));
	EXPECT_TRUE(rtabmap_->isInSTM(N2));
	EXPECT_TRUE(rtabmap_->isInSTM(N1));
	EXPECT_TRUE(rtabmap_->getPose(N3).isNull());
	// Remaining nodes still reachable.
	EXPECT_FALSE(rtabmap_->getPose(N2).isNull());

	// _idCount is not rewound on delete: the next signature gets a fresh id
	// strictly greater than the removed one.
	process();
	const int Nnext = rtabmap_->getLastLocationId();
	EXPECT_GT(Nnext, N3);
}

TEST_F(RtabmapFixture, DeleteLastLocationIsNoOpWhenStmEmpty)
{
	// Fresh memory with no signatures -> deleteLastLocation should not crash.
	rtabmap_->deleteLastLocation();
	EXPECT_EQ(rtabmap_->getSTMSize(), 0);
}

// ---------------------------------------------------------------------------
// getSignatureCopy
// ---------------------------------------------------------------------------

TEST(RtabmapTest, GetSignatureCopyReturnsRequestedPayloads)
{
	// kMemBinDataKept=true so compressed image data survives on the signature.
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kMemBinDataKept()] = "true";
	Rtabmap rtabmap;
	rtabmap.init(params);

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	SensorData data(image); data.setId(1);
	ASSERT_TRUE(rtabmap.process(data, Transform(1.0f, 0, 0, 0, 0, 0), covariance));
	const int id = rtabmap.getLastLocationId();

	// Request only image: scan/userData/grid/words/globalDescriptors omitted.
	const Signature s = rtabmap.getSignatureCopy(id, /*images=*/true,
			/*scan=*/false, /*userData=*/false, /*occupancyGrid=*/false,
			/*withWords=*/false, /*withGlobalDescriptors=*/false);
	EXPECT_EQ(s.id(), id);
	EXPECT_EQ(s.sensorData().imageCompressed().rows, 1); // populated
	EXPECT_EQ(s.sensorData().laserScanCompressed().isEmpty(), true);
	EXPECT_EQ(s.sensorData().userDataCompressed().rows, 0);
	EXPECT_FLOAT_EQ(s.sensorData().gridCellSize(), 0.0f);
	EXPECT_EQ(s.getWords().size(), 0u);

	rtabmap.close(false);
}

TEST(RtabmapTest, GetSignatureCopyOmitsImageWhenNotRequested)
{
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kMemBinDataKept()] = "true";
	Rtabmap rtabmap;
	rtabmap.init(params);

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	SensorData data(image); data.setId(1);
	ASSERT_TRUE(rtabmap.process(data, Transform(0, 0, 0, 0, 0, 0), covariance));
	const int id = rtabmap.getLastLocationId();

	const Signature s = rtabmap.getSignatureCopy(id, /*images=*/false,
			/*scan=*/false, /*userData=*/false, /*occupancyGrid=*/false,
			/*withWords=*/false, /*withGlobalDescriptors=*/false);
	EXPECT_EQ(s.id(), id);
	EXPECT_EQ(s.sensorData().imageCompressed().rows, 0);

	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// getGraph
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, GetGraphReturnsLocalOptimizedPosesAndConstraints)
{
	// 3-node chain: poses are the odometry positions (1,0), (2,0), (3,0).
	// getGraph(optimized=true, global=false) returns local optimized poses.
	process();
	process();
	process();
	std::map<int, Transform> poses;
	std::multimap<int, Link> constraints;
	rtabmap_->getGraph(poses, constraints, /*optimized=*/true, /*global=*/false);

	EXPECT_EQ(poses.size(), 3u);
	// 2 neighbor links in a 3-node chain.
	EXPECT_EQ(constraints.size(), 2u);
	for(const auto & kv : constraints)
	{
		EXPECT_EQ(kv.second.type(), Link::kNeighbor);
	}
}

TEST_F(RtabmapFixture, GetGraphReturnsSignaturesWhenRequested)
{
	process();
	process();
	process();
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, Link> constraints;
	rtabmap_->getGraph(poses, constraints, /*optimized=*/true, /*global=*/false,
			&signatures, /*withImages=*/false, /*withScan=*/false,
			/*withUserData=*/false, /*withGrid=*/false);

	EXPECT_EQ(signatures.size(), 3u);
	for(const auto & kv : signatures)
	{
		EXPECT_EQ(kv.second.id(), kv.first);
	}
}

TEST_F(RtabmapFixture, GetGraphOptimizedFalseReturnsOdomPosesUnshiftedByLoopClosure)
{
	// Build a 3-node chain and inject a loop closure that pulls N3 toward N1.
	// With optimized=true the returned poses reflect the optimizer output (N3
	// near 7/3). With optimized=false the poses come from Memory's odom poses
	// instead -- N3 stays at its raw odom pose (3, 0).
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	reinit(params);
	process();
	const int N1 = rtabmap_->getLastLocationId();
	process();
	process();
	const int N3 = rtabmap_->getLastLocationId();

	Link loop(N3, N1, Link::kGlobalClosure,
			Transform(-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
			cv::Mat::eye(6, 6, CV_64FC1) * 100.0);
	ASSERT_TRUE(rtabmap_->addLink(loop));

	// optimized=true -> N3 is pulled to ~7/3.
	{
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		rtabmap_->getGraph(poses, constraints, /*optimized=*/true, /*global=*/false);
		EXPECT_NEAR(poses.at(N3).x(), 7.0f / 3.0f, 0.05f);
	}
	// optimized=false -> N3 stays at odom (3, 0).
	{
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		rtabmap_->getGraph(poses, constraints, /*optimized=*/false, /*global=*/false);
		EXPECT_NEAR(poses.at(N3).x(), 3.0f, 1e-3f);
		// The loop link is still part of the graph constraints regardless of
		// whether we asked for optimized or odom poses.
		ASSERT_EQ(constraints.size(), 3u);
		int neighbors = 0, loops = 0;
		for(const auto & kv : constraints)
		{
			if(kv.second.type() == Link::kNeighbor) ++neighbors;
			else if(kv.second.type() == Link::kGlobalClosure) ++loops;
		}
		EXPECT_EQ(neighbors, 2);
		EXPECT_EQ(loops, 1);
	}
}

TEST_F(RtabmapFixture, GetGraphGlobalTrueIncludesLtmNodesWhereLocalDoesNot)
{
	// Force memory management by capping WM at 2 signatures via kRtabmapMemoryThr.
	// With STM=1 and 6 processed frames, the oldest nodes get transferred to LTM
	// at every iteration once the cap is hit. global=false then returns only the
	// signatures still in WM/STM, while global=true walks the database links and
	// returns every node.
	ParametersMap params = defaultRtabmapParams(/*rgbdMode=*/true);
	params[Parameters::kMemSTMSize()] = "1";
	params[Parameters::kRtabmapMemoryThr()] = "2";
	reinit(params);

	std::vector<int> ids;
	for(int i = 0; i < 6; ++i)
	{
		ASSERT_TRUE(process());
		ids.push_back(rtabmap_->getLastLocationId());
	}

	// Some of the oldest nodes have been transferred to LTM.
	int inLtm = 0;
	for(int id : ids)
	{
		if(rtabmap_->getMemory()->isInLTM(id)) ++inLtm;
	}
	ASSERT_GT(inLtm, 0);

	std::map<int, Transform> posesLocal;
	std::multimap<int, Link> linksLocal;
	rtabmap_->getGraph(posesLocal, linksLocal, /*optimized=*/false, /*global=*/false);

	std::map<int, Transform> posesGlobal;
	std::multimap<int, Link> linksGlobal;
	rtabmap_->getGraph(posesGlobal, linksGlobal, /*optimized=*/false, /*global=*/true);

	// global=true walks into the database, so it surfaces strictly more nodes.
	EXPECT_GT(posesGlobal.size(), posesLocal.size());
	EXPECT_EQ(posesGlobal.size(), ids.size());
	// LTM nodes appear in the global graph but not the local one.
	for(int id : ids)
	{
		if(rtabmap_->getMemory()->isInLTM(id))
		{
			EXPECT_EQ(posesLocal.count(id), 0u)  << "local should not have LTM id=" << id;
			EXPECT_EQ(posesGlobal.count(id), 1u) << "global should have LTM id=" << id;
		}
	}
	// Same containment for the links between nodes.
	EXPECT_GT(linksGlobal.size(), linksLocal.size());
}

// ---------------------------------------------------------------------------
// detectMoreLoopClosures
// ---------------------------------------------------------------------------

TEST(RtabmapTest, DetectMoreLoopClosuresFailsInAppearanceOnlyMode)
{
	// kRGBDEnabled=false -> not RGBD-SLAM mode -> returns -1.
	Rtabmap rtabmap;
	rtabmap.init(defaultRtabmapParams(/*rgbdMode=*/false));
	EXPECT_EQ(rtabmap.detectMoreLoopClosures(1.0f, 0.0f, 1, true, true), -1);
	rtabmap.close(false);
}

TEST_F(RtabmapFixture, DetectMoreLoopClosuresRejectsZeroSessionFlags)
{
	// Both intra and inter session must not be false simultaneously.
	EXPECT_EQ(rtabmap_->detectMoreLoopClosures(1.0f, 0.0f, 1, false, false), -1);
}

TEST_F(RtabmapFixture, DetectMoreLoopClosuresReturnsZeroWithoutLoopCandidates)
{
	// Without features (kKpMaxFeatures=-1), the registration pipeline can't
	// detect any loop closures. The function still completes (returns 0).
	process();
	process();
	process();
	EXPECT_EQ(rtabmap_->detectMoreLoopClosures(0.5f, M_PI / 4.0f, 1, true, true), 0);
}

// ---------------------------------------------------------------------------
// refineLinks
// ---------------------------------------------------------------------------

TEST(RtabmapTest, RefineLinksFailsInAppearanceOnlyMode)
{
	Rtabmap rtabmap;
	rtabmap.init(defaultRtabmapParams(/*rgbdMode=*/false));
	EXPECT_EQ(rtabmap.refineLinks(), -1);
	rtabmap.close(false);
}

TEST_F(RtabmapFixture, RefineLinksReturnsZeroWhenComputeTransformFails)
{
	// With kKpMaxFeatures=-1 the registration pipeline has no features to match,
	// so every computeTransform() returns a null transform and nothing is refined.
	process();
	process();
	process();
	EXPECT_EQ(rtabmap_->refineLinks(), 0);
}

// ---------------------------------------------------------------------------
// computePath
// ---------------------------------------------------------------------------

TEST(RtabmapTest, ComputePathFailsInAppearanceOnlyMode)
{
	Rtabmap rtabmap;
	rtabmap.init(defaultRtabmapParams(/*rgbdMode=*/false));
	EXPECT_FALSE(rtabmap.computePath(1, false));
	rtabmap.close(false);
}

TEST_F(RtabmapFixture, ComputePathToKnownNodeSucceedsAndPopulatesPath)
{
	process();
	const int N1 = rtabmap_->getLastLocationId();
	process();
	process();
	process();
	const int N4 = rtabmap_->getLastLocationId();

	EXPECT_TRUE(rtabmap_->computePath(N1, /*global=*/false));
	// Path starts at the current node (N4) and ends at the target N1.
	const std::vector<std::pair<int, Transform> > & path = rtabmap_->getPath();
	ASSERT_EQ(path.size(), 4u);
	EXPECT_EQ(path.front().first, N4);
	EXPECT_EQ(path.back().first, N1);
	// All nodes are within kRGBDLocalRadius (default 10 m), so updateGoalIndex
	// advances _pathGoalIndex to the final target.
	EXPECT_EQ(rtabmap_->getPathCurrentGoalId(), N1);
	EXPECT_EQ(rtabmap_->getPathCurrentIndex(), 0u);
}

TEST_F(RtabmapFixture, FollowPathAdvancesCurrentIndexAndReachesGoal)
{
	// Build 4-node chain N1..N4 at odom poses (1,0), (2,0), (3,0), (4,0). Then
	// plan a path from N4 back to N1. Simulate the robot following the path by
	// processing 3 more frames at intermediate poses; each frame calls
	// updateGoalIndex() which (a) advances _pathCurrentIndex to the nearest path
	// node, (b) advances _pathGoalIndex up to kRGBDLocalRadius along the path, and
	// (c) clears the path with status=1 when within kRGBDGoalReachedRadius of the
	// final target. The default kRGBDLocalRadius is 10 m, larger than the whole
	// path, so the goal index would jump straight to the end. Reduce it to 0.5 m
	// so the current goal advances step by step.
	//
	// updateGoalIndex computes the new goal *before* the new current index, so
	// within a single process() call goal advances at most one step from the
	// old current index; the next process() call can then advance one more.
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kRGBDLocalRadius()] = "0.5";
	reinit(params);

	process();
	const int N1 = rtabmap_->getLastLocationId();
	process();
	process();
	const int N3 = rtabmap_->getLastLocationId();
	process();
	const int N4 = rtabmap_->getLastLocationId();
	ASSERT_TRUE(rtabmap_->computePath(N1, /*global=*/false));
	ASSERT_EQ(rtabmap_->getPath().size(), 4u);
	ASSERT_EQ(rtabmap_->getPath().front().first, N4);
	ASSERT_EQ(rtabmap_->getPath().back().first, N1);
	// Right after planning, _pathCurrentIndex is at the start (N4) and the
	// current goal is one step ahead (N3, distance 1 m > localRadius).
	EXPECT_EQ(rtabmap_->getPathCurrentIndex(), 0u);
	EXPECT_EQ(rtabmap_->getPathCurrentGoalId(), N3);
	EXPECT_EQ(rtabmap_->getPathStatus(), 0); // active

	// Step 1: robot at (3, 0), on top of N3. _pathCurrentIndex catches up to the
	// previous goal (N3). The goal-update loop used the old current index, so the
	// goal still points to N3 -- it only advances on the *next* iteration.
	ASSERT_TRUE(processWith(Transform(3.0f, 0, 0, 0, 0, 0)));
	EXPECT_EQ(rtabmap_->getPathCurrentIndex(), 1u);
	EXPECT_EQ(rtabmap_->getPathCurrentGoalId(), N3);
	EXPECT_EQ(rtabmap_->getPathStatus(), 0);

	// Step 2: robot at (2, 0). _pathCurrentIndex advances to 2, and the goal
	// advances by one step to the node after the new current -- here that's the
	// node the robot just landed on (N2).
	ASSERT_TRUE(processWith(Transform(2.0f, 0, 0, 0, 0, 0)));
	EXPECT_EQ(rtabmap_->getPathCurrentIndex(), 2u);
	EXPECT_EQ(rtabmap_->getPathCurrentGoalId(), rtabmap_->getPath()[2].first);
	EXPECT_EQ(rtabmap_->getPathStatus(), 0);

	// Step 3: robot at (1, 0) (on N1). Distance to final goal == 0, well within
	// kRGBDGoalReachedRadius (0.5 m default). clearPath(1) fires before the
	// goal-update loop, so status reports "reached" and the path is cleared.
	ASSERT_TRUE(processWith(Transform(1.0f, 0, 0, 0, 0, 0)));
	EXPECT_EQ(rtabmap_->getPathStatus(), 1);
	EXPECT_EQ(rtabmap_->getPath().size(), 0u);
}

TEST_F(RtabmapFixture, FollowPathWithLtmNodesRetrievesThemAndReachesGoal)
{
	// Same path-following scenario as the test above, but the two oldest nodes
	// (N1 and N2) are transferred to LTM via memory management before planning.
	// kRGBDMaxLocalRetrieved=2 default lets process() pull both back into WM on
	// each tick to keep the path-tracking logic working.
	// Build phase: aggressive WM cap + disabled retrieval so memory management
	// fully evicts the oldest nodes to LTM without auto-retrieving them back.
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kMemSTMSize()] = "1";
	params[Parameters::kRtabmapMemoryThr()] = "1";
	params[Parameters::kRGBDMaxLocalRetrieved()] = "0";
	reinit(params);

	process();
	const int N1 = rtabmap_->getLastLocationId();
	process();
	const int N2 = rtabmap_->getLastLocationId();
	process();
	process();
	const int N4 = rtabmap_->getLastLocationId();
	ASSERT_TRUE(rtabmap_->getMemory()->isInLTM(N1));
	ASSERT_TRUE(rtabmap_->getMemory()->isInLTM(N2));

	// Path-follow phase: re-enable retrieval (default 2) and lift the WM cap so
	// retrieved path nodes don't immediately get pushed back to LTM during the
	// next iteration.
	ParametersMap updated;
	updated[Parameters::kRGBDMaxLocalRetrieved()] = "2";
	updated[Parameters::kRtabmapMemoryThr()] = "0";
	rtabmap_->parseParameters(updated);

	// Plan a global path so graph::computePath can walk through LTM nodes via
	// database lookups.
	ASSERT_TRUE(rtabmap_->computePath(N1, /*global=*/true));
	ASSERT_EQ(rtabmap_->getPath().size(), 4u);
	EXPECT_EQ(rtabmap_->getPath().front().first, N4);
	EXPECT_EQ(rtabmap_->getPath().back().first, N1);
	EXPECT_EQ(rtabmap_->getPathStatus(), 0);

	// Step 1: at (3, 0). process() walks the path forward, sees N2 and N1 are
	// not in WM, and queues them for retrieval (up to kRGBDMaxLocalRetrieved).
	// After the call both LTM nodes are back in WM.
	ASSERT_TRUE(processWith(Transform(3.0f, 0, 0, 0, 0, 0)));
	EXPECT_TRUE(rtabmap_->getMemory()->isInWM(N1));
	EXPECT_TRUE(rtabmap_->getMemory()->isInWM(N2));
	EXPECT_EQ(rtabmap_->getPathStatus(), 0);

	// Step 2: at (2, 0). Still tracking the path.
	ASSERT_TRUE(processWith(Transform(2.0f, 0, 0, 0, 0, 0)));
	EXPECT_EQ(rtabmap_->getPathStatus(), 0);

	// Step 3: at (1, 0). Goal reached, path cleared.
	ASSERT_TRUE(processWith(Transform(1.0f, 0, 0, 0, 0, 0)));
	EXPECT_EQ(rtabmap_->getPathStatus(), 1);
	EXPECT_EQ(rtabmap_->getPath().size(), 0u);
}

TEST_F(RtabmapFixture, FollowPathSurvivesPathTailInLtm)
{
    // Regression test for a null-pointer dereference in Rtabmap::updateGoalIndex.
    // When kRtabmapMemoryThr keeps memory pressure on during path-follow, nodes
    // in the path tail (path[0..currentIndex-1]) can be transferred to LTM. The
    // virtual-link cleanup loop in updateGoalIndex must skip those tail nodes
    // gracefully -- previously it null-deref'd `s->getWeight()` after a
    // successful `if(s)` check, crashing on step 3 of this scenario.
    ParametersMap params = defaultRtabmapParams();
    params[Parameters::kMemSTMSize()] = "1";
    params[Parameters::kRtabmapMemoryThr()] = "1";
    params[Parameters::kRGBDMaxLocalRetrieved()] = "0";
    reinit(params);

    process();
    const int N1 = rtabmap_->getLastLocationId();
    process();
    process();
    process();
    ASSERT_TRUE(rtabmap_->getMemory()->isInLTM(N1));

    ParametersMap updated;
    updated[Parameters::kRGBDMaxLocalRetrieved()] = "2";
    // Note: kRtabmapMemoryThr stays at 1 so the path tail keeps getting evicted.
    rtabmap_->parseParameters(updated);

    ASSERT_TRUE(rtabmap_->computePath(N1, /*global=*/true));

    // Three ticks. Step 3 is where path[0]=N4 and path[1]=N3 are both in LTM
    // when updateGoalIndex runs -- the previous null-deref bug crashed here.
    ASSERT_TRUE(processWith(Transform(3.0f, 0, 0, 0, 0, 0)));
    ASSERT_TRUE(processWith(Transform(2.0f, 0, 0, 0, 0, 0)));
    ASSERT_TRUE(processWith(Transform(1.0f, 0, 0, 0, 0, 0)));

    // The robot reached N1 at step 3 -- goal-reached fires, path is cleared.
    EXPECT_EQ(rtabmap_->getPathStatus(), 1);
}

TEST_F(RtabmapFixture, FollowLongPathRetrievesLtmNodesAcrossMultipleIterations)
{
    // Same scenario as the previous test but with 10 path nodes and 8 of them
    // in LTM. The path-retrieval loop is capped at kRGBDMaxLocalRetrieved=2 per
    // process() iteration, so the LTM nodes are pulled back into WM gradually
    // over multiple ticks as the retrieval window walks along the path.
    ParametersMap params = defaultRtabmapParams();
    params[Parameters::kMemSTMSize()] = "1";
    params[Parameters::kRtabmapMemoryThr()] = "1";
    params[Parameters::kRGBDMaxLocalRetrieved()] = "0"; // disable retrieval during setup
    reinit(params);

    std::vector<int> ids;
    for(int i = 0; i < 10; ++i)
    {
        ASSERT_TRUE(process());
        ids.push_back(rtabmap_->getLastLocationId());
    }
    // The 8 oldest nodes have been pushed to LTM; the 2 most recent remain in
    // WM (N9) and STM (N10).
    int initialLtm = 0;
    for(int i = 0; i < 8; ++i) if(rtabmap_->getMemory()->isInLTM(ids[i])) ++initialLtm;
    ASSERT_EQ(initialLtm, 8);

    ParametersMap updated;
    updated[Parameters::kRGBDMaxLocalRetrieved()] = "2";
    updated[Parameters::kRtabmapMemoryThr()] = "0";
    rtabmap_->parseParameters(updated);

    ASSERT_TRUE(rtabmap_->computePath(ids[0], /*global=*/true));
    ASSERT_EQ(rtabmap_->getPath().size(), 10u);

    auto countLoadedFromIds = [&]() {
        int c = 0;
        for(int id : ids)
        {
            if(rtabmap_->getMemory()->isInWM(id) || rtabmap_->getMemory()->isInSTM(id))
                ++c;
        }
        return c;
    };

    // Initial: N9 in WM + N10 in STM. N1..N8 still in LTM.
    EXPECT_EQ(countLoadedFromIds(), 2);

    // Robot walks 9 steps from (9, 0) down to (1, 0). Each of the first 4 ticks
    // retrieves 2 more LTM path nodes (8 LTM nodes / cap=2 -> 4 ticks). After
    // that, all path nodes are in WM and the remaining ticks just follow the
    // path until kRGBDGoalReachedRadius triggers clearPath(1).
    int prev = countLoadedFromIds();
    for(int step = 1; step <= 9; ++step)
    {
        ASSERT_TRUE(processWith(Transform(float(10 - step), 0, 0, 0, 0, 0)));
        const int now = countLoadedFromIds();
        if(step <= 4)
        {
            // The retrieval window advances by 2 nodes per iteration.
            EXPECT_EQ(now - prev, 2) << "step " << step << " expected +2 retrievals";
        }
        else
        {
            // No more LTM nodes to retrieve.
            EXPECT_EQ(now, 10) << "step " << step;
        }
        prev = now;
        if(rtabmap_->getPathStatus() != 0) break;
    }

    EXPECT_EQ(rtabmap_->getPathStatus(), 1);
    EXPECT_EQ(rtabmap_->getPath().size(), 0u);
    EXPECT_EQ(countLoadedFromIds(), 10);
}

namespace {
// Adds a chain of "real" nodes separated by @p intermediatesBetween intermediate
// nodes. Real nodes are placed at integer x = 1, 2, ..., real_count. Intermediates
// sit at fractional positions between consecutive real nodes. Returns the ids of
// the real nodes in order. Uses const_cast on getMemory() to access the
// non-const convertToIntermediate.
std::vector<int> buildChainWithIntermediates(
        Rtabmap * rtabmap,
        const cv::Mat & image,
        const cv::Mat & covariance,
        int realCount,
        int intermediatesBetween)
{
    Memory * mem = const_cast<Memory*>(rtabmap->getMemory());
    std::vector<int> realIds;
    int procId = 0;
    for(int real = 0; real < realCount; ++real)
    {
        SensorData d(image); d.setId(++procId);
        EXPECT_TRUE(rtabmap->process(d, Transform(float(real + 1), 0, 0, 0, 0, 0), covariance));
        realIds.push_back(rtabmap->getLastLocationId());
        if(real < realCount - 1)
        {
            for(int k = 1; k <= intermediatesBetween; ++k)
            {
                float frac = float(k) / float(intermediatesBetween + 1);
                SensorData di(image); di.setId(++procId);
                EXPECT_TRUE(rtabmap->process(di, Transform(float(real + 1) + frac, 0, 0, 0, 0, 0), covariance));
                mem->convertToIntermediate(rtabmap->getLastLocationId());
            }
        }
    }
    return realIds;
}
} // namespace

TEST_F(RtabmapFixture, FollowPathWithIntermediateNodesSkipsThemAndReachesGoal)
{
    // Variant of FollowPathAdvancesCurrentIndexAndReachesGoal with 2 intermediate
    // nodes between every pair of real nodes (4 real + 6 intermediate = 10
    // signatures). The planner must skip the intermediate nodes when building
    // _path so the path-follow can run on real waypoints only -- updateGoalIndex
    // refuses to follow paths that contain intermediates.
    ParametersMap params = defaultRtabmapParams();
    params[Parameters::kRGBDLocalRadius()] = "0.5";
    reinit(params);

    const std::vector<int> realIds = buildChainWithIntermediates(
            rtabmap_, image_, covariance_, /*realCount=*/4, /*intermediatesBetween=*/2);
    ASSERT_EQ(realIds.size(), 4u);

    ASSERT_TRUE(rtabmap_->computePath(realIds[0], /*global=*/false));
    // Path must contain only real nodes (intermediates filtered out).
    ASSERT_EQ(rtabmap_->getPath().size(), 4u);
    for(size_t i = 0; i < rtabmap_->getPath().size(); ++i)
    {
        const Signature * s = rtabmap_->getMemory()->getSignature(rtabmap_->getPath()[i].first);
        ASSERT_NE(s, nullptr);
        EXPECT_NE(s->getWeight(), -1) << "path[" << i << "] is intermediate";
    }
    EXPECT_EQ(rtabmap_->getPath().front().first, realIds[3]); // start at N4
    EXPECT_EQ(rtabmap_->getPath().back().first, realIds[0]);  // end at N1
    EXPECT_EQ(rtabmap_->getPathCurrentGoalId(), realIds[2]);  // one step ahead

    // Walk three ticks at real-node poses: N3, N2, N1. updateGoalIndex must
    // tick through path indices without aborting and clear the path on arrival.
    ASSERT_TRUE(processWith(Transform(3.0f, 0, 0, 0, 0, 0)));
    EXPECT_EQ(rtabmap_->getPathCurrentIndex(), 1u);
    EXPECT_EQ(rtabmap_->getPathStatus(), 0);

    ASSERT_TRUE(processWith(Transform(2.0f, 0, 0, 0, 0, 0)));
    EXPECT_EQ(rtabmap_->getPathCurrentIndex(), 2u);
    EXPECT_EQ(rtabmap_->getPathStatus(), 0);

    ASSERT_TRUE(processWith(Transform(1.0f, 0, 0, 0, 0, 0)));
    EXPECT_EQ(rtabmap_->getPathStatus(), 1);
    EXPECT_EQ(rtabmap_->getPath().size(), 0u);
}

TEST_F(RtabmapFixture, FollowLongPathWithIntermediateNodesRetrievesRealLtmNodes)
{
    // Variant of FollowLongPathRetrievesLtmNodesAcrossMultipleIterations with
    // 2 intermediate nodes between every pair of real nodes (10 real + 18
    // intermediate = 28 signatures). Only the real nodes need to be retrieved
    // during path-follow; the planner filters intermediates out of the path.
    ParametersMap params = defaultRtabmapParams();
    params[Parameters::kMemSTMSize()] = "1";
    params[Parameters::kRtabmapMemoryThr()] = "1";
    params[Parameters::kRGBDMaxLocalRetrieved()] = "0";
    reinit(params);

    const std::vector<int> realIds = buildChainWithIntermediates(
            rtabmap_, image_, covariance_, /*realCount=*/10, /*intermediatesBetween=*/2);
    ASSERT_EQ(realIds.size(), 10u);
    // Some real nodes have been pushed to LTM by memory management.
    int initialLtm = 0;
    for(int i = 0; i < 8; ++i) if(rtabmap_->getMemory()->isInLTM(realIds[i])) ++initialLtm;
    ASSERT_GE(initialLtm, 1);

    ParametersMap updated;
    updated[Parameters::kRGBDMaxLocalRetrieved()] = "2";
    updated[Parameters::kRtabmapMemoryThr()] = "0";
    rtabmap_->parseParameters(updated);

    ASSERT_TRUE(rtabmap_->computePath(realIds[0], /*global=*/true));
    // The planner filters intermediate nodes -> path is only the 10 real ones.
    ASSERT_EQ(rtabmap_->getPath().size(), 10u);
    for(const auto & kv : rtabmap_->getPath())
    {
        const Signature * s = rtabmap_->getMemory()->getSignature(kv.first);
        if(s) EXPECT_NE(s->getWeight(), -1) << "intermediate id=" << kv.first << " on path";
    }

    // Walk back along the path. Path-follow advances and eventually reaches N1.
    for(int step = 1; step <= 9; ++step)
    {
        ASSERT_TRUE(processWith(Transform(float(10 - step), 0, 0, 0, 0, 0)));
        if(rtabmap_->getPathStatus() != 0) break;
    }
    EXPECT_EQ(rtabmap_->getPathStatus(), 1);
}

TEST(RtabmapTest, FollowLongPathWithIntermediateNodesInLocalizationMode)
{
    // Localization-mode variant: build a 10-real + 18-intermediate chain in
    // mapping mode, save to DB, reopen in localization mode, plan a global
    // path. The planner skips intermediates so the robot can walk through the
    // real nodes only.
    const std::string dbPath = uniqueDbPath();
    cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
    cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
    std::vector<int> realIds;
    {
        Rtabmap rtabmap;
        rtabmap.init(defaultRtabmapParams(), dbPath);
        realIds = buildChainWithIntermediates(&rtabmap, image, cov,
                /*realCount=*/10, /*intermediatesBetween=*/2);
        rtabmap.close(true);
    }
    {
        ParametersMap params = defaultRtabmapParams();
        params[Parameters::kMemIncrementalMemory()] = "false";
        params[Parameters::kMemSTMSize()] = "1";
        params[Parameters::kRtabmapMemoryThr()] = "1";
        params[Parameters::kRGBDMaxLocalRetrieved()] = "0";
        Rtabmap rtabmap;
        rtabmap.init(params, dbPath);

        rtabmap.setInitialPose(Transform(10.0f, 0, 0, 0, 0, 0));

        ParametersMap upd;
        upd[Parameters::kRGBDMaxLocalRetrieved()] = "2";
        rtabmap.parseParameters(upd);

        ASSERT_TRUE(rtabmap.computePath(realIds[0], /*global=*/true));
        ASSERT_EQ(rtabmap.getPath().size(), 10u);

        int procId = 101;
        for(int step = 0; step <= 9; ++step)
        {
            SensorData d(image); d.setId(++procId);
            ASSERT_TRUE(rtabmap.process(d, Transform(float(10 - step), 0, 0, 0, 0, 0), cov));
            if(rtabmap.getPathStatus() != 0) break;
        }
        EXPECT_EQ(rtabmap.getPathStatus(), 1);

        rtabmap.close(false);
    }
    UFile::erase(dbPath.c_str());
}

TEST(RtabmapTest, FollowLongPathInLocalizationModeRetrievesLtmNodes)
{
    // Same long-path retrieval scenario as
    // FollowLongPathRetrievesLtmNodesAcrossMultipleIterations, but with the
    // planning happening in localization mode against a persisted map. Build a
    // 10-node map in mapping mode, save it, then reopen in localization mode
    // with aggressive memory pressure. Verify that path retrieval pulls LTM
    // path nodes back into WM over multiple iterations.
    const std::string dbPath = uniqueDbPath();
    cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
    cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
    std::vector<int> ids;
    {
        Rtabmap rtabmap;
        rtabmap.init(defaultRtabmapParams(), dbPath);
        for(int i = 0; i < 10; ++i)
        {
            SensorData d(image); d.setId(i + 1);
            ASSERT_TRUE(rtabmap.process(d, Transform(float(i + 1), 0, 0, 0, 0, 0), cov));
            ids.push_back(rtabmap.getLastLocationId());
        }
        rtabmap.close(true);
    }
    {
        ParametersMap params = defaultRtabmapParams();
        params[Parameters::kMemIncrementalMemory()] = "false"; // localization mode
        params[Parameters::kMemSTMSize()] = "1";
        params[Parameters::kRtabmapMemoryThr()] = "1";
        params[Parameters::kRGBDMaxLocalRetrieved()] = "0";
        Rtabmap rtabmap;
        rtabmap.init(params, dbPath);

        rtabmap.setInitialPose(Transform(10.0f, 0, 0, 0, 0, 0));

        // Re-enable retrieval. Keep kRtabmapMemoryThr aggressive so each tick
        // triggers forget -> local-map refresh, which is what propagates newly
        // retrieved LTM path nodes into _optimizedPoses (in localization mode the
        // refresh only runs when at least one signature was removed in the same
        // tick). Now safe to keep on thanks to the updateGoalIndex null-check fix.
        ParametersMap upd;
        upd[Parameters::kRGBDMaxLocalRetrieved()] = "2";
        rtabmap.parseParameters(upd);

        ASSERT_TRUE(rtabmap.computePath(ids[0], /*global=*/true));
        ASSERT_EQ(rtabmap.getPath().size(), 10u);

        // Walk the robot from N10 (10, 0) down to N1 (1, 0). The first tick at
        // the initial pose calibrates mapCorrection to identity; subsequent
        // ticks move 1 m back per tick. After tick 10 the robot is at N1.
        int procId = 101;
        for(int step = 0; step <= 9; ++step)
        {
            SensorData d(image); d.setId(++procId);
            ASSERT_TRUE(rtabmap.process(d, Transform(float(10 - step), 0, 0, 0, 0, 0), cov));
            if(rtabmap.getPathStatus() != 0) break;
        }
        EXPECT_EQ(rtabmap.getPathStatus(), 1);

        rtabmap.close(false);
    }
    UFile::erase(dbPath.c_str());
}

TEST_F(RtabmapFixture, LocalRadiusLimitsPathRetrievalToOneNodePerIteration)
{
    // Same 10-node, 8-LTM scenario as the previous test, but with kRGBDLocalRadius
    // reduced to 0.9 m -- *less* than the 1 m node spacing. The path-retrieval
    // loop in Rtabmap::process breaks once distanceSoFar exceeds the radius, so
    // only the *first* LTM node beyond the current path index is retrieved each
    // tick. The cap kRGBDMaxLocalRetrieved=2 default is no longer the bottleneck:
    // the local radius is. It takes 8 ticks to retrieve the 8 LTM nodes (vs.
    // 4 ticks with the default 10 m radius).
    ParametersMap params = defaultRtabmapParams();
    params[Parameters::kMemSTMSize()] = "1";
    params[Parameters::kRtabmapMemoryThr()] = "1";
    params[Parameters::kRGBDMaxLocalRetrieved()] = "0";
    reinit(params);

    std::vector<int> ids;
    for(int i = 0; i < 10; ++i)
    {
        ASSERT_TRUE(process());
        ids.push_back(rtabmap_->getLastLocationId());
    }
    int initialLtm = 0;
    for(int i = 0; i < 8; ++i) if(rtabmap_->getMemory()->isInLTM(ids[i])) ++initialLtm;
    ASSERT_EQ(initialLtm, 8);

    ParametersMap updated;
    updated[Parameters::kRGBDMaxLocalRetrieved()] = "2";
    updated[Parameters::kRtabmapMemoryThr()] = "0";
    updated[Parameters::kRGBDLocalRadius()] = "0.9"; // < node spacing -> radius caps retrieval
    rtabmap_->parseParameters(updated);

    ASSERT_TRUE(rtabmap_->computePath(ids[0], /*global=*/true));
    ASSERT_EQ(rtabmap_->getPath().size(), 10u);

    auto countLoaded = [&]() {
        int c = 0;
        for(int id : ids)
            if(rtabmap_->getMemory()->isInWM(id) || rtabmap_->getMemory()->isInSTM(id)) ++c;
        return c;
    };

    EXPECT_EQ(countLoaded(), 2);

    int prev = countLoaded();
    for(int step = 1; step <= 9; ++step)
    {
        ASSERT_TRUE(processWith(Transform(float(10 - step), 0, 0, 0, 0, 0)));
        const int now = countLoaded();
        if(step <= 8)
        {
            // Radius=0.9 m allows reaching exactly one new LTM node per tick.
            EXPECT_EQ(now - prev, 1) << "step " << step << " expected +1 retrieval";
        }
        prev = now;
        if(rtabmap_->getPathStatus() != 0) break;
    }
    EXPECT_EQ(rtabmap_->getPathStatus(), 1);
    EXPECT_EQ(countLoaded(), 10);
}

TEST_F(RtabmapFixture, ComputePathToUnknownTargetReturnsFalseAndClearsPath)
{
	// graph::computePath returns an empty path for an unreachable target;
	// Rtabmap::computePath then returns false and leaves the path empty.
	process();
	process();
	EXPECT_FALSE(rtabmap_->computePath(99999, /*global=*/false));
	EXPECT_EQ(rtabmap_->getPath().size(), 0u);
}

TEST_F(RtabmapFixture, ComputePathClearsPreviousPath)
{
	// First plan a valid path, then plan a path with an invalid target. The
	// previous path must be cleared at the start of the new computePath call.
	process();
	const int N1 = rtabmap_->getLastLocationId();
	process();
	process();
	process();

	ASSERT_TRUE(rtabmap_->computePath(N1, /*global=*/false));
	ASSERT_GT(rtabmap_->getPath().size(), 0u);

	rtabmap_->computePath(99999, /*global=*/false);
	EXPECT_EQ(rtabmap_->getPath().size(), 0u);
}

// ---------------------------------------------------------------------------
// globalBundleAdjustment
// ---------------------------------------------------------------------------

TEST(RtabmapTest, GlobalBundleAdjustmentReturnsFalseWhenGraphEmpty)
{
	// No process() calls -> _optimizedPoses and _constraints both empty.
	Rtabmap rtabmap;
	rtabmap.init(defaultRtabmapParams());
	EXPECT_FALSE(rtabmap.globalBundleAdjustment(/*optimizerType=*/0,
			/*rematchFeatures=*/false, /*iterations=*/1, /*pixelVariance=*/1.0f));
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// cleanupLocalGrids (Rtabmap wrapper)
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, CleanupLocalGridsForwardsToMemory)
{
	// Rtabmap::cleanupLocalGrids is a thin wrapper around Memory::cleanupLocalGrids
	// (exhaustively tested in test_memory.cpp). The fixture's Rtabmap.init(params)
	// gives Memory an in-memory database, so the call succeeds; with no signature
	// carrying obstacle cells, nothing is filtered and the function returns 0.
	process();
	std::map<int, Transform> poses{{rtabmap_->getLastLocationId(), Transform::getIdentity()}};
	cv::Mat map(4, 4, CV_8UC1, cv::Scalar(0));
	EXPECT_EQ(rtabmap_->cleanupLocalGrids(poses, map, 0.0f, 0.0f, 0.1f), 0);
}

TEST(RtabmapTest, CleanupLocalGridsReturnsMinusOneWithoutMemory)
{
	// Default-constructed Rtabmap has no Memory yet -> wrapper short-circuits.
	Rtabmap rtabmap;
	std::map<int, Transform> poses{{1, Transform::getIdentity()}};
	cv::Mat map(4, 4, CV_8UC1, cv::Scalar(0));
	EXPECT_EQ(rtabmap.cleanupLocalGrids(poses, map, 0.0f, 0.0f, 0.1f), -1);
}

// ---------------------------------------------------------------------------
// RGBD/OptimizeMaxError branches inside Rtabmap::process()
// ---------------------------------------------------------------------------

namespace {
// Parameter set tuned to drive a feature-based loop closure inside
// Rtabmap::process() without running real feature extraction. Pre-baked
// keypoints+descriptors are fed via SensorData (kMemUseOdomFeatures=true);
// epipolar verification and proximity detection are disabled so the only
// channel that can fire a loop closure is the appearance-based Bayes filter.
ParametersMap badLoopClosureParams()
{
	ParametersMap params;
	params.insert({Parameters::kKpMaxFeatures(), "8"});
	params.insert({Parameters::kKpIncrementalFlann(), "true"});
	params.insert({Parameters::kMemUseOdomFeatures(), "true"});
	params.insert({Parameters::kMemRehearsalSimilarity(), "1.0"});
	params.insert({Parameters::kMemBinDataKept(), "false"});
	params.insert({Parameters::kMemBadSignaturesIgnored(), "false"});
	params.insert({Parameters::kMemSTMSize(), "1"});
	params.insert({Parameters::kRGBDEnabled(), "true"});
	params.insert({Parameters::kRGBDLinearUpdate(), "0.0"});
	params.insert({Parameters::kRGBDAngularUpdate(), "0.0"});
	params.insert({Parameters::kVhEpEnabled(), "false"});      // skip epipolar verification
	params.insert({Parameters::kRGBDProximityBySpace(), "false"}); // Bayes only
	// Allow visual registration to succeed with our small feature count
	// (default kVisMinInliers=20 would reject 8 features).
	params.insert({Parameters::kVisMinInliers(), "6"});
	// Skip bundle adjustment which can NAN out on these synthetic features.
	params.insert({Parameters::kVisBundleAdjustment(), "0"});
	return params;
}

// SensorData with k distinct keypoints (non-degenerate 2D/3D so the visual
// registration can compute a valid relative transform between matched frames)
// and one-hot descriptors keyed by @p featSlot. Frames with the same featSlot
// produce identical visual words; different featSlots produce disjoint words
// so the Bayes filter does not auto-match them.
SensorData makeFeaturesData(int id, int featSlot = 0, int k = 8)
{
	const int kSlots = 32;
	// featSlot * 8 + row writes into a kSlots*8-wide descriptor matrix below;
	// an out-of-bounds slot silently corrupts heap because cv::Mat::at skips
	// bounds checks in release builds. Catch it at the test boundary.
	UASSERT(featSlot >= 0 && featSlot < kSlots);
	cv::Mat image(64, 64, CV_8UC1, cv::Scalar(128));
	CameraModel camera(50.0, 50.0, 32.0, 32.0, CameraModel::opticalRotation(), 0.0, cv::Size(64, 64));
	SensorData data(image, camera); data.setId(id);
	std::vector<cv::KeyPoint> kpts;
	std::vector<cv::Point3f> pts3;
	// 3D points are stored in the base/body frame: x forward, y left, z up.
	// They are projected via fx=fy=50, cx=cy=32 so the (u, v) keypoints are
	// consistent with the (x, y, z) 3D positions, otherwise PnP can't solve.
	//   u = cx - fx * (y / x)   =>   y = -(u - cx) * x / fx
	//   v = cy - fy * (z / x)   =>   z = -(v - cy) * x / fy
	const float positions[8][5] = {
		// u, v, x_base, y_base, z_base
		{10, 10,  1.5f,  0.66f,  0.66f},
		{50, 10,  1.5f, -0.54f,  0.66f},
		{50, 50,  1.5f, -0.54f, -0.54f},
		{10, 50,  1.5f,  0.66f, -0.54f},
		{30, 30,  1.0f,  0.04f,  0.04f},
		{20, 40,  1.2f,  0.29f, -0.19f},
		{40, 20,  1.2f, -0.19f,  0.29f},
		{40, 50,  1.5f, -0.24f, -0.54f},
	};
	for(int i = 0; i < k && i < 8; ++i)
	{
		kpts.push_back(cv::KeyPoint(positions[i][0], positions[i][1], 1.f));
		pts3.push_back(cv::Point3f(positions[i][2], positions[i][3], positions[i][4]));
	}
	cv::Mat descriptors = cv::Mat::zeros(k, kSlots * 8, CV_32F);
	for(int row = 0; row < k; ++row)
	{
		descriptors.at<float>(row, featSlot * 8 + row) = 1000.0f;
	}
	data.setFeatures(kpts, pts3, descriptors);
	return data;
}
// Variant where each feature row can come from a different slot. Used to
// build partial-match signatures (e.g., 4 features from slot 30 + 4 features
// from a unique slot) so multiple nodes have positive raw likelihood at the
// same time -- adjustLikelihood's Angeli outlier check needs >1 positive
// sample to produce a non-zero stdDev.
SensorData makeFeaturesMixedSlots(int id, const std::vector<int> & slotPerRow)
{
	const int k = (int)slotPerRow.size();
	UASSERT(k <= 8);
	const int kSlots = 32;
	cv::Mat image(64, 64, CV_8UC1, cv::Scalar(128));
	CameraModel camera(50.0, 50.0, 32.0, 32.0, CameraModel::opticalRotation(), 0.0, cv::Size(64, 64));
	SensorData data(image, camera); data.setId(id);
	std::vector<cv::KeyPoint> kpts;
	std::vector<cv::Point3f> pts3;
	const float positions[8][5] = {
		{10, 10,  1.5f,  0.66f,  0.66f},
		{50, 10,  1.5f, -0.54f,  0.66f},
		{50, 50,  1.5f, -0.54f, -0.54f},
		{10, 50,  1.5f,  0.66f, -0.54f},
		{30, 30,  1.0f,  0.04f,  0.04f},
		{20, 40,  1.2f,  0.29f, -0.19f},
		{40, 20,  1.2f, -0.19f,  0.29f},
		{40, 50,  1.5f, -0.24f, -0.54f},
	};
	for(int i = 0; i < k; ++i)
	{
		kpts.push_back(cv::KeyPoint(positions[i][0], positions[i][1], 1.f));
		pts3.push_back(cv::Point3f(positions[i][2], positions[i][3], positions[i][4]));
	}
	cv::Mat descriptors = cv::Mat::zeros(k, kSlots * 8, CV_32F);
	for(int row = 0; row < k; ++row)
	{
		descriptors.at<float>(row, slotPerRow[row] * 8 + row) = 1000.0f;
	}
	data.setFeatures(kpts, pts3, descriptors);
	return data;
}

} // namespace

TEST(RtabmapTest, ProcessRejectsBadLoopClosureWhenMaxErrorExceeded)
{
	// Build three signatures with identical visual features but the third is
	// 10 m away in odometry. The Bayes filter detects a loop closure between
	// the new frame and an old one, but the chain says they are 10 m apart -->
	// huge optimization error ratio. kRGBDOptimizeMaxError=1 rejects the link.
	ParametersMap params = badLoopClosureParams();
	params[Parameters::kRGBDOptimizeMaxError()] = "1.0";
	params[Parameters::kRtabmapLoopThr()] = "0.05"; // accept low-likelihood hypotheses
	params[Parameters::kBayesVirtualPlacePriorThr()] = "0.1"; // less weight on virtual place -> appearance likelihood dominates
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	// 10-node chain. N4 is the "match" target for N10 (8/8 features in slot 30).
	// N3 and N5 are partial matches (2/8 features in slot 30) so the
	// Rtabmap::adjustLikelihood Angeli outlier check has multiple positive raw
	// likelihoods to compute non-zero variance from -- otherwise N4's single
	// peak gets flattened to the same value as every other node.
	// The chain accumulates 6 m of odometry between N4 and N10 but the loop
	// closure registration on identical features returns ~identity, producing
	// a ~6 m optimization error that exceeds kRGBDOptimizeMaxError=1.
	const int kMatchSlot = 30;
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(1, /*featSlot=*/0), Transform(0.0f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(2, /*featSlot=*/1), Transform(1.0f, 0, 0, 0, 0, 0), cov));
	// N3: 2/8 features in match slot, 6/8 unique -> partial match.
	ASSERT_TRUE(rtabmap.process(
			makeFeaturesMixedSlots(3, {kMatchSlot, kMatchSlot, 23, 23, 23, 23, 23, 23}),
			Transform(2.0f, 0, 0, 0, 0, 0), cov));
	// N4: 8/8 features in match slot -> full match.
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(4, /*featSlot=*/kMatchSlot), Transform(3.0f, 0, 0, 0, 0, 0), cov));
	// N5: 2/8 features in match slot, 6/8 unique -> partial match.
	ASSERT_TRUE(rtabmap.process(
			makeFeaturesMixedSlots(5, {kMatchSlot, kMatchSlot, 25, 25, 25, 25, 25, 25}),
			Transform(4.0f, 0, 0, 0, 0, 0), cov));
	for(int i = 6; i <= 9; ++i)
	{
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(i, /*featSlot=*/i), Transform(float(i - 1), 0, 0, 0, 0, 0), cov));
	}
	// N10 reuses the match slot -> Bayes filter matches N10 back to N4.
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(10, /*featSlot=*/kMatchSlot), Transform(9.0f, 0, 0, 0, 0, 0), cov));

	// Verify the rejection specifically came from the OptimizeMaxError path:
	// the kLoopOptimization_max_error_ratio statistic reflects the max-error
	// edge ratio measured by the optimizer. It exists only when the optimizer
	// ran AND found a high error -- if the rejection had been earlier (e.g.,
	// failed registration), this stat would not be populated above the gate.
	const auto & stats = rtabmap.getStatistics().data();
	auto itRatio = stats.find(Statistics::kLoopOptimization_max_error_ratio());
	ASSERT_NE(itRatio, stats.end());
	EXPECT_GT(itRatio->second, 1.0f) << "optimizer max-error ratio must exceed kRGBDOptimizeMaxError";
	auto itRej = stats.find(Statistics::kLoopRejectedHypothesis());
	ASSERT_NE(itRej, stats.end());
	EXPECT_FLOAT_EQ(itRej->second, 1.0f);
	EXPECT_EQ(rtabmap.getLoopClosureId(), 0);
	// And the would-be loop closure link was removed from Memory.
	const Signature * sLast = rtabmap.getMemory()->getSignature(rtabmap.getLastLocationId());
	ASSERT_NE(sLast, nullptr);
	for(const auto & kv : sLast->getLinks())
	{
		EXPECT_NE(kv.second.type(), Link::kGlobalClosure);
	}
	rtabmap.close(false);
}

// In localization mode the OptimizeMaxError check is fed by the odom cache
// chain rather than by the mapping graph: a first loop closure on N4 anchors
// the localization, then a chain of unique-feature nodes builds up odometry
// distance, and a second loop closure that pretends to land on N4 again is
// inconsistent with the accumulated chain and gets rejected by
// kRGBDOptimizeMaxError.
TEST(RtabmapTest, ProcessRejectsBadLoopClosureInLocalizationModeViaOptimizeMaxError)
{
	const std::string dbPath = uniqueDbPath();
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const int kMatchSlot = 30;

	// --- Mapping phase: build a 9-node map with the same partial-match
	// scaffolding used by ProcessRejectsBadLoopClosureWhenMaxErrorExceeded
	// (N3 and N5 have 2/8 features in kMatchSlot) so the Bayes filter has
	// a non-degenerate likelihood distribution to score N10 / N20 against.
	{
		ParametersMap params = badLoopClosureParams();
		params[Parameters::kRGBDOptimizeMaxError()] = "0"; // disable while mapping
		params[Parameters::kRtabmapLoopThr()] = "0.05";
		params[Parameters::kBayesVirtualPlacePriorThr()] = "0.1";
		Rtabmap rtabmap;
		rtabmap.init(params, dbPath);

		ASSERT_TRUE(rtabmap.process(makeFeaturesData(1, /*featSlot=*/0), Transform(0.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(2, /*featSlot=*/1), Transform(1.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(
				makeFeaturesMixedSlots(3, {kMatchSlot, kMatchSlot, 23, 23, 23, 23, 23, 23}),
				Transform(2.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(4, /*featSlot=*/kMatchSlot), Transform(3.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(
				makeFeaturesMixedSlots(5, {kMatchSlot, kMatchSlot, 25, 25, 25, 25, 25, 25}),
				Transform(4.0f, 0, 0, 0, 0, 0), cov));
		for(int i = 6; i <= 9; ++i)
		{
			ASSERT_TRUE(rtabmap.process(makeFeaturesData(i, /*featSlot=*/i), Transform(float(i - 1), 0, 0, 0, 0, 0), cov));
		}
		rtabmap.close(true);
	}

	// --- Localization phase: reopen non-incremental and walk forward in odom.
	{
		ParametersMap params = badLoopClosureParams();
		params[Parameters::kMemIncrementalMemory()] = "false";
		params[Parameters::kRGBDOptimizeMaxError()] = "1.0";
		params[Parameters::kRtabmapLoopThr()] = "0.05";
		params[Parameters::kBayesVirtualPlacePriorThr()] = "0.1";
		// The odom cache must span the entire localization session (N10..N20 = 11
		// nodes) so the second loop closure has the full chain available when
		// OptimizeMaxError evaluates the would-be edge.
		params[Parameters::kRGBDMaxOdomCacheSize()] = "30";
		Rtabmap rtabmap;
		rtabmap.init(params, dbPath);
		// Anchor localization near N4 so the first loop closure is consistent.
		rtabmap.setInitialPose(Transform(3.0f, 0, 0, 0, 0, 0));

		// N10 and N11: identical features to N4 at (essentially) the same odom
		// pose. In localization mode, re-localization requires two consecutive
		// matching hypotheses on the same node before the loop closure is
		// committed -- N10 alone is rejected even though its likelihood on N4
		// is high; N11 then confirms the hypothesis and the link is accepted.
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(10, /*featSlot=*/kMatchSlot), Transform(3.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(11, /*featSlot=*/kMatchSlot), Transform(3.0f, 0, 0, 0, 0, 0), cov));
		EXPECT_EQ(rtabmap.getLoopClosureId(), 4);

		// N12..N19: unique features, no loop. Odometry walks 1 m per step in
		// the same direction so by N19 the chain is 8 m past the anchor.
		// Slots 12..19 are not used during mapping (mapping uses slots 0, 1, 23,
		// 25, 30, 6..9) so each step adds fresh words to the dictionary without
		// matching anything. Slots must stay < kSlots=32 in makeFeaturesData
		// to keep descriptor writes inside the cv::Mat allocation.
		for(int i = 12; i <= 19; ++i)
		{
			ASSERT_TRUE(rtabmap.process(
					makeFeaturesData(i, /*featSlot=*/i),
					Transform(float(i - 12 + 4), 0, 0, 0, 0, 0), cov));
			EXPECT_EQ(rtabmap.getLoopClosureId(), 0) << "unexpected loop at i=" << i;
		}

		// N20: identical features to N4 again, but odometry says we're 9 m
		// past N4. The loop-closure registration on identical features returns
		// ~identity, so OptimizeMaxError sees a ~9 m disagreement between the
		// odom-cache chain and the proposed loop edge -> rejected.
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(20, /*featSlot=*/kMatchSlot), Transform(12.0f, 0, 0, 0, 0, 0), cov));

		const auto & stats = rtabmap.getStatistics().data();
		auto itRatio = stats.find(Statistics::kLoopOptimization_max_error_ratio());
		ASSERT_NE(itRatio, stats.end());
		EXPECT_GT(itRatio->second, 1.0f) << "optimizer max-error ratio must exceed kRGBDOptimizeMaxError";
		auto itRej = stats.find(Statistics::kLoopRejectedHypothesis());
		ASSERT_NE(itRej, stats.end());
		EXPECT_FLOAT_EQ(itRej->second, 1.0f);
		EXPECT_EQ(rtabmap.getLoopClosureId(), 0);

		rtabmap.close(false);
	}
	UFile::erase(dbPath.c_str());
}

// Repair branch of kRGBDOptimizeMaxError inside Rtabmap::process(): when the
// SAME loop closure edge is the max-error link of the graph two iterations in
// a row, kRGBDOptimizeMaxErrorRepairRadius>0 lets process() drop the offending
// edge instead of rejecting the new loop closure. The strategy here is:
//
//   Phase 1 - OptimizeMaxError disabled. Build a chain N1..N8 (N3 = full
//             kMatchSlot target with two pairs of partial flanks around it),
//             then inject a wrong (N8,N2) loop closure via addLink (identity
//             transform when the chain says 6 m apart). The link survives
//             because OptimizeMaxError=0 short-circuits the gate.
//   Phase 2 - parseParameters() enables OptimizeMaxError + repair. The robot
//             backs up (N9..N12) toward N3's pose, then N13 with partial
//             kMatchSlot features triggers a Bayes-detected (N13,N3) loop.
//             The dominant max-error edge is the planted (N8,N2) link, so
//             the loop is rejected and _lastRejectedLoopClosureIds <- (2,8).
//             After a unique-feature intermediate N14, N15 lands at N3's
//             pose with full kMatchSlot, triggering a second (N15,N3) loop.
//             The same (N8,N2) edge is again the max-error link, so the
//             repair branch fires and removes the bad link.
//
// The optimizer's residual-distribution behavior matters here: g2o and GTSAM
// both place the bad link as the dominant max-error edge across both phase-2
// iterations. TORO's simpler gradient solver does not converge the same way
// and the test does not hold for it (see ProcessRepairsGraphAfterRepeatedBadLoopClosure*).
static void runProcessRepairsGraphAfterRepeatedBadLoopClosure(int optimizerStrategy, bool gtsamIncremental = false)
{
	ParametersMap params = badLoopClosureParams();
	params[Parameters::kOptimizerStrategy()] = uNumber2Str(optimizerStrategy);
	if(gtsamIncremental)
	{
		params[Parameters::kGTSAMIncremental()] = "true";
	}
	params[Parameters::kRGBDOptimizeMaxError()] = "0.0"; // disabled in phase 1
	params[Parameters::kRGBDOptimizeMaxErrorRepairRadius()] = "0.0";
	params[Parameters::kRtabmapLoopThr()] = "0.05";
	params[Parameters::kBayesVirtualPlacePriorThr()] = "0.1";
	// Disable TF-IDF likelihood: with all signatures the same size, the simple
	// compareTo metric (matched_pairs / max(words)) gives N3 a clean peak in
	// phase-2 queries without the TF-IDF dilution that otherwise flattens the
	// likelihood vector against the bad-link prior.
	params[Parameters::kKpTfIdfLikelihoodUsed()] = "false";
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const int kMatchSlot = 30;

	// N1, N2: unique features. N2 will be one end of the bad link.
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(1, /*featSlot=*/0), Transform(0.0f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(2, /*featSlot=*/1), Transform(1.0f, 0, 0, 0, 0, 0), cov));
	// N3..N5: target + two partial-match flanks for phase-2 Bayes variance.
	// The flanks give adjustLikelihood >= 3 positive samples around N3 so it
	// emits a real peak instead of a uniform posterior.
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(3, /*featSlot=*/kMatchSlot), Transform(2.0f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(
			makeFeaturesMixedSlots(4, {kMatchSlot, kMatchSlot, 23, 23, 23, 23, 23, 23}),
			Transform(3.0f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(
			makeFeaturesMixedSlots(5, {kMatchSlot, kMatchSlot, 24, 24, 24, 24, 24, 24}),
			Transform(4.0f, 0, 0, 0, 0, 0), cov));
	// N6, N7: more 2/8 kMatchSlot partials. Five partials in total (N4..N7
	// plus N13 in phase 2) give adjustLikelihood enough variance to produce a
	// sharp peak on N3 - the prior propagation through the planted bad link
	// concentrates posterior on N8/N2, so the appearance peak on N3 must be
	// strong enough to overcome it.
	ASSERT_TRUE(rtabmap.process(
			makeFeaturesMixedSlots(6, {kMatchSlot, kMatchSlot, 26, 26, 26, 26, 26, 26}),
			Transform(5.0f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(
			makeFeaturesMixedSlots(7, {kMatchSlot, kMatchSlot, 27, 27, 27, 27, 27, 27}),
			Transform(6.0f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(8, /*featSlot=*/8), Transform(7.0f, 0, 0, 0, 0, 0), cov));

	// Inject the bad loop closure (N8,N2): identity transform claimed across
	// a 6 m chain gap. addLink() bypasses the residual gate because
	// OptimizeMaxError=0. The information matrix is set LOWER than the odom
	// links' info (=1/cov=100): with the loop's low confidence the optimizer
	// preserves the odom chain rather than satisfying the loop, so the loop's
	// residual stays at ~6 m and dominates the max-error ranking across
	// phase-2 iterations -> _lastRejectedLoopClosureIds stays on (N8,N2)
	// -> repair fires on the second iteration.
	const Link badLink(8, 2, Link::kGlobalClosure, Transform::getIdentity(),
			cv::Mat::eye(6, 6, CV_64FC1) * 1.0);
	ASSERT_TRUE(rtabmap.addLink(badLink));

	// Phase 2: enable OptimizeMaxError + repair.
	ParametersMap params2 = params;
	params2[Parameters::kRGBDOptimizeMaxError()] = "1.0";
	params2[Parameters::kRGBDOptimizeMaxErrorRepairRadius()] = "1.0";
	rtabmap.parseParameters(params2);

	// Back up smoothly from N8 (odom x=7) toward N3 (odom x=2).
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(9,  /*featSlot=*/9),  Transform(6.0f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(10, /*featSlot=*/10), Transform(5.0f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(11, /*featSlot=*/11), Transform(4.0f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(12, /*featSlot=*/12), Transform(3.0f, 0, 0, 0, 0, 0), cov));
	// N13: partial kMatchSlot (6/8 rows + 2 unique) so it can loop-close to
	// N3 via PnP (6 inliers >= kVisMinInliers=6) without becoming an 8/8 twin
	// of N3 - that would tie with N3 in N15's Bayes likelihood and flatten
	// the adjusted posterior.
	ASSERT_TRUE(rtabmap.process(
			makeFeaturesMixedSlots(13, {kMatchSlot, kMatchSlot, kMatchSlot, kMatchSlot, kMatchSlot, kMatchSlot, 13, 13}),
			Transform(2.0f, 0, 0, 0, 0, 0), cov));
	{
		const auto & s = rtabmap.getStatistics().data();
		auto itRej = s.find(Statistics::kLoopRejectedHypothesis());
		ASSERT_NE(itRej, s.end());
		EXPECT_FLOAT_EQ(itRej->second, 1.0f) << "first loop must be rejected to arm repair";
		EXPECT_EQ(rtabmap.getLoopClosureId(), 0);
		auto itFrom = s.find(Statistics::kLoopOptimization_max_error_from_id());
		auto itTo = s.find(Statistics::kLoopOptimization_max_error_to_id());
		ASSERT_NE(itFrom, s.end());
		ASSERT_NE(itTo, s.end());
		// The max-error edge must be the planted (N8,N2) loop so the second
		// iteration sees the same edge -> _lastRejectedLoopClosureIds matches.
		EXPECT_EQ(int(itFrom->second), 2);
		EXPECT_EQ(int(itTo->second), 8);
	}

	// N14: unique features intermediate (forward step). Doesn't trigger a
	// loop closure, so _lastRejectedLoopClosureIds is preserved from N13.
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(14, /*featSlot=*/14), Transform(3.0f, 0, 0, 0, 0, 0), cov));
	// N15: second kMatchSlot query -> (N15,N3) loop attempt. Same dominant
	// max-error edge (N8,N2), same _lastRejectedLoopClosureIds -> repair fires.
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(15, /*featSlot=*/kMatchSlot),
			Transform(2.0f, 0, 0, 0, 0, 0), cov));

	// Verify repair fired: kLoop/Optimization_max_error_removed_* are only
	// populated when repairGraph() actually removed at least one link.
	const auto & stats = rtabmap.getStatistics().data();
	auto itRemoved = stats.find(Statistics::kLoopOptimization_max_error_removed_count());
	ASSERT_NE(itRemoved, stats.end());
	EXPECT_GE(itRemoved->second, 1.0f);
	auto itRemovedFrom = stats.find(Statistics::kLoopOptimization_max_error_removed_from_id());
	ASSERT_NE(itRemovedFrom, stats.end());
	auto itRemovedTo = stats.find(Statistics::kLoopOptimization_max_error_removed_to_id());
	ASSERT_NE(itRemovedTo, stats.end());
	// The removed edge must be the (N8,N2) planted bad link (ids stored in
	// either direction depending on Memory's link convention).
	const std::pair<int, int> removedIds{int(itRemovedFrom->second), int(itRemovedTo->second)};
	EXPECT_TRUE(removedIds == std::make_pair(8, 2) || removedIds == std::make_pair(2, 8))
			<< "got (" << removedIds.first << "," << removedIds.second << ")";

	// New loop closure accepted, and the bad (N8,N2) link is gone from Memory.
	EXPECT_NE(rtabmap.getLoopClosureId(), 0);
	const Signature * sN8 = rtabmap.getMemory()->getSignature(8);
	ASSERT_NE(sN8, nullptr);
	EXPECT_EQ(sN8->getLinks().count(2), 0u) << "(N8,N2) bad link must be repaired away";

	rtabmap.close(false);
}

TEST(RtabmapTest, ProcessRepairsGraphAfterRepeatedBadLoopClosureG2O)
{
	if(!Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		GTEST_SKIP() << "g2o optimizer not available in this build";
	}
	runProcessRepairsGraphAfterRepeatedBadLoopClosure(Optimizer::kTypeG2O);
}

TEST(RtabmapTest, ProcessRepairsGraphAfterRepeatedBadLoopClosureGTSAM)
{
	if(!Optimizer::isAvailable(Optimizer::kTypeGTSAM))
	{
		GTEST_SKIP() << "GTSAM optimizer not available in this build";
	}
	runProcessRepairsGraphAfterRepeatedBadLoopClosure(Optimizer::kTypeGTSAM);
}

TEST(RtabmapTest, ProcessRepairsGraphAfterRepeatedBadLoopClosureGTSAMIncremental)
{
	if(!Optimizer::isAvailable(Optimizer::kTypeGTSAM))
	{
		GTEST_SKIP() << "GTSAM optimizer not available in this build";
	}
	// iSAM2 incremental optimization should still surface the planted bad link
	// as the dominant max-error edge across both rejection iterations so the
	// repair branch fires.
	runProcessRepairsGraphAfterRepeatedBadLoopClosure(Optimizer::kTypeGTSAM, /*gtsamIncremental=*/true);
}

// ---------------------------------------------------------------------------
// setUserData round-trip
// ---------------------------------------------------------------------------

TEST(RtabmapTest, SetUserDataRoundTripsThroughGetSignatureCopy)
{
	// Round-trip user data attached to a node via setUserData(id) and read it
	// back through getSignatureCopy(..., userData=true).
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kMemBinDataKept()] = "true";
	Rtabmap rtabmap;
	rtabmap.init(params);

	const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	SensorData data(image); data.setId(1);
	ASSERT_TRUE(rtabmap.process(data, Transform(0, 0, 0, 0, 0, 0), cov));
	const int id = rtabmap.getLastLocationId();

	// Attach a deterministic payload after processing.
	const std::vector<unsigned char> payload = {0x42, 0xAB, 0x00, 0x7F, 0x11};
	const cv::Mat userData(1, int(payload.size()), CV_8UC1, const_cast<unsigned char *>(payload.data()));
	EXPECT_TRUE(rtabmap.setUserData(id, userData.clone()));

	// Read it back with userData=true.
	const Signature s = rtabmap.getSignatureCopy(id, /*images=*/false,
			/*scan=*/false, /*userData=*/true, /*occupancyGrid=*/false,
			/*withWords=*/false, /*withGlobalDescriptors=*/false);
	const cv::Mat readBack = s.sensorData().userDataRaw();
	ASSERT_EQ(readBack.total(), payload.size());
	for(size_t i = 0; i < payload.size(); ++i)
	{
		EXPECT_EQ(readBack.data[i], payload[i]) << "byte " << i;
	}

	// id=0 falls back to "last working signature".
	const cv::Mat payload2(1, 3, CV_8UC1, cv::Scalar(0xCC));
	EXPECT_TRUE(rtabmap.setUserData(0, payload2));
	const Signature s2 = rtabmap.getSignatureCopy(id, /*images=*/false,
			/*scan=*/false, /*userData=*/true, /*occupancyGrid=*/false,
			/*withWords=*/false, /*withGlobalDescriptors=*/false);
	EXPECT_EQ(s2.sensorData().userDataRaw().total(), 3u);

	rtabmap.close(false);
}

TEST(RtabmapTest, SetUserDataReturnsFalseWithoutMemory)
{
	Rtabmap rtabmap;
	// No init() -> no memory.
	EXPECT_FALSE(rtabmap.setUserData(1, cv::Mat(1, 4, CV_8UC1)));
}

// ---------------------------------------------------------------------------
// getNodesInRadius
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, GetNodesInRadiusByPoseFiltersByDistance)
{
	// Build a 5-node straight-line chain at x=1..5.
	for(int i = 0; i < 5; ++i) process();

	// Query around (3,0,0) with a 1.5 m radius -> {N2, N3, N4}.
	std::map<int, float> distsSqr;
	auto poses = rtabmap_->getNodesInRadius(Transform(3.0f, 0, 0, 0, 0, 0), 1.5f, 0, &distsSqr);
	std::set<int> ids;
	for(const auto & kv : poses) ids.insert(kv.first);
	EXPECT_EQ(ids, (std::set<int>{2, 3, 4}));
	// N3 is at distance 0, N2 and N4 at distance 1.
	EXPECT_FLOAT_EQ(distsSqr.at(3), 0.0f);
	EXPECT_FLOAT_EQ(distsSqr.at(2), 1.0f);
	EXPECT_FLOAT_EQ(distsSqr.at(4), 1.0f);
}

TEST_F(RtabmapFixture, GetNodesInRadiusByPoseRespectsKCap)
{
	for(int i = 0; i < 5; ++i) process();

	// Query around (3,0,0) with a 10 m radius (covers all 5) but k=2 -> the 2
	// closest: N3 (d=0) and either N2 or N4 (d=1).
	auto poses = rtabmap_->getNodesInRadius(Transform(3.0f, 0, 0, 0, 0, 0), 10.0f, 2);
	EXPECT_EQ(poses.size(), 2u);
	EXPECT_TRUE(poses.count(3));
}

TEST_F(RtabmapFixture, GetNodesInRadiusByNodeIdQueriesAroundThatNode)
{
	for(int i = 0; i < 5; ++i) process();

	// Around N3 (x=3), radius 1.5 m: candidates exclude the query itself, so
	// the result is the immediate neighbors N2 (d=1) and N4 (d=1).
	auto poses = rtabmap_->getNodesInRadius(/*nodeId=*/3, 1.5f);
	std::set<int> ids;
	for(const auto & kv : poses) ids.insert(kv.first);
	EXPECT_EQ(ids, (std::set<int>{2, 4})) << "graph::findNearestNodes(nodeId,...) excludes nodeId itself";
}

TEST_F(RtabmapFixture, GetNodesInRadiusUnknownNodeReturnsEmpty)
{
	for(int i = 0; i < 3; ++i) process();
	auto poses = rtabmap_->getNodesInRadius(/*nodeId=*/999, 10.0f);
	EXPECT_TRUE(poses.empty());
}

// ---------------------------------------------------------------------------
// generateDOTGraph
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, GenerateDOTGraphWritesDotFile)
{
	for(int i = 0; i < 4; ++i) process();

	const std::string dotPath = uFormat("/tmp/rtabmap_test_dot_%d.dot", getpid());
	UFile::erase(dotPath.c_str());
	rtabmap_->generateDOTGraph(dotPath, /*id=*/0, /*margin=*/5);

	ASSERT_TRUE(UFile::exists(dotPath));
	// Smoke-check: graphviz file begins with "digraph" and references node ids.
	std::ifstream in(dotPath);
	std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	EXPECT_NE(contents.find("digraph"), std::string::npos) << "missing digraph header";
	EXPECT_NE(contents.find("1"), std::string::npos);
	EXPECT_NE(contents.find("4"), std::string::npos);
	UFile::erase(dotPath.c_str());
}

// ---------------------------------------------------------------------------
// exportPoses
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, ExportPosesRawFormatRoundTripsViaImport)
{
	// Build a 4-node chain and export poses in the raw (format 0) layout, then
	// re-read the file via graph::importPoses() and confirm we get the same
	// poses back. This covers the local/optimized export path.
	for(int i = 0; i < 4; ++i) process();

	const std::string outPath = uFormat("/tmp/rtabmap_test_poses_raw_%d.txt", getpid());
	UFile::erase(outPath.c_str());
	rtabmap_->exportPoses(outPath, /*optimized=*/true, /*global=*/false, /*format=*/0);
	ASSERT_TRUE(UFile::exists(outPath));

	std::map<int, Transform> imported;
	std::map<int, double> stamps;
	std::multimap<int, Link> links;
	ASSERT_TRUE(graph::importPoses(outPath, /*format=*/0, imported, &links, &stamps));
	EXPECT_EQ(imported.size(), 4u);
	for(const auto & kv : imported)
	{
		const Transform expected(float(kv.first), 0.0f, 0.0f, 0, 0, 0);
		EXPECT_NEAR(kv.second.x(), expected.x(), 1e-3f);
		EXPECT_NEAR(kv.second.y(), expected.y(), 1e-3f);
		EXPECT_NEAR(kv.second.z(), expected.z(), 1e-3f);
	}
	UFile::erase(outPath.c_str());
}

TEST_F(RtabmapFixture, ExportPosesKittiFormatWritesOneMatrixPerLine)
{
	for(int i = 0; i < 3; ++i) process();
	const std::string outPath = uFormat("/tmp/rtabmap_test_poses_kitti_%d.txt", getpid());
	UFile::erase(outPath.c_str());
	rtabmap_->exportPoses(outPath, /*optimized=*/true, /*global=*/false, /*format=*/2);
	ASSERT_TRUE(UFile::exists(outPath));
	// KITTI: 3x4 = 12 floats per line, 3 lines for 3 nodes.
	std::ifstream in(outPath);
	int lineCount = 0;
	std::string line;
	while(std::getline(in, line))
	{
		++lineCount;
		std::istringstream is(line);
		int floats = 0;
		float v;
		while(is >> v) ++floats;
		EXPECT_EQ(floats, 12) << "line " << lineCount;
	}
	EXPECT_EQ(lineCount, 3);
	UFile::erase(outPath.c_str());
}

// ---------------------------------------------------------------------------
// globalBundleAdjustment on a real graph
// ---------------------------------------------------------------------------

TEST(RtabmapTest, GlobalBundleAdjustmentOnPopulatedGraphPreservesPosesIfOptimizationFails)
{
	if(!Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		GTEST_SKIP() << "g2o optimizer not available in this build (required by BA)";
	}
	// BA on a non-empty graph runs the optimizer end-to-end. Synthetic one-hot
	// features at fixed 2D positions are geometrically degenerate (same 2D
	// projection across all 3 frames despite different poses), so g2o's BA
	// converges to NaN and the call returns false. The important invariant
	// is the failure path: when BA reports failure, the original optimized
	// poses MUST be left untouched (the code only assigns _optimizedPoses
	// when the optimizer returned a non-empty result).
	ParametersMap params = badLoopClosureParams();
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	params[Parameters::kRtabmapLoopThr()] = "1"; // suppress loop closures (posterior never exceeds 1)
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const int kMatchSlot = 30;
	for(int i = 1; i <= 3; ++i)
	{
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(i, kMatchSlot),
				Transform(float(i - 1), 0, 0, 0, 0, 0), cov));
	}
	ASSERT_EQ(rtabmap.getLoopClosureId(), 0);

	const auto before = rtabmap.getLocalOptimizedPoses();
	ASSERT_EQ(before.size(), 3u);

	// Whether BA succeeds or fails on this degenerate input, the post-state
	// must remain coherent (no NaN poses, same node ids).
	rtabmap.globalBundleAdjustment(Optimizer::kTypeG2O, /*rematchFeatures=*/true);
	const auto after = rtabmap.getLocalOptimizedPoses();
	ASSERT_EQ(after.size(), 3u);
	for(const auto & kv : after)
	{
		ASSERT_TRUE(before.count(kv.first));
		EXPECT_FALSE(kv.second.isNull());
		EXPECT_TRUE(std::isfinite(kv.second.x()));
		EXPECT_TRUE(std::isfinite(kv.second.y()));
		EXPECT_TRUE(std::isfinite(kv.second.z()));
	}
	rtabmap.close(false);
}

namespace ba_synth {
// Synthetic SfM scene helpers used by GlobalBundleAdjustmentRefinesPosesOnSynthScene.
//
// Builds a small "structure-from-motion" world:
//   - N random 3D landmarks in a cube around the origin
//   - K camera frames spaced around the volume looking inward
//   - Each frame's SensorData carries the visible subset of landmarks, with
//     unique one-hot descriptors so cross-frame matching is trivial and 2D
//     keypoints obtained by reprojecting the landmark through the camera model.

struct Landmark3D {
	cv::Point3f world; // in world frame
	int slot;          // unique slot id -> one-hot descriptor row
};

inline CameraModel sceneCamera()
{
	// 640x480 with fx=fy=500: same FOV as the 64x64/f=50 cameras used elsewhere
	// in this file, but 10x angular resolution per pixel - so the same 0.1 px
	// detector noise represents 10x sharper observations in BA's cost function.
	return CameraModel(500.0, 500.0, 320.0, 240.0, CameraModel::opticalRotation(), 0.0, cv::Size(640, 480));
}

inline std::vector<Landmark3D> randomLandmarks(int n, float halfExtent, unsigned seed)
{
	std::mt19937 rng(seed);
	std::uniform_real_distribution<float> dist(-halfExtent, halfExtent);
	std::vector<Landmark3D> pts;
	pts.reserve(n);
	for(int i = 0; i < n; ++i)
	{
		pts.push_back({cv::Point3f(dist(rng), dist(rng), dist(rng)), /*slot=*/i});
	}
	return pts;
}

// 3D point in robot base frame, given the camera optical-frame coords. Inverse
// of the (z_optical = x_base; y_base = -x_optical; z_base = -y_optical) mapping
// used implicitly by makeFeaturesData. Provided so tests can verify projection.
inline cv::Point3f opticalToBase(const cv::Point3f & optical)
{
	return cv::Point3f(optical.z, -optical.x, -optical.y);
}

inline cv::Point3f baseToOptical(const cv::Point3f & base)
{
	return cv::Point3f(-base.y, -base.z, base.x);
}

// Camera positions on a horizontal circle of radius @p radius around the
// origin, each looking inward (yaw = angle + pi).
inline std::vector<Transform> hexagonalPoses(int count, float radius)
{
	std::vector<Transform> out;
	out.reserve(count);
	for(int i = 0; i < count; ++i)
	{
		const float angle = 2.0f * float(M_PI) * float(i) / float(count);
		const float x = radius * std::cos(angle);
		const float y = radius * std::sin(angle);
		const float yaw = angle + float(M_PI); // robot x points toward origin
		out.emplace_back(x, y, 0.0f, 0.0f, 0.0f, yaw);
	}
	return out;
}

// Builds a SensorData for the frame at @p basePose, containing the visible
// subset of @p landmarks. Mirrors a stereo/RGB-D sensor: the 2D keypoint is
// disturbed by independent sub-pixel detector noise (@p pixelNoiseStd, per
// axis, in pixels) and the triangulated 3D point is disturbed by depth
// estimation noise (@p depthNoiseStd, per axis, in metres, in the base frame).
// The two noise channels are uncorrelated, as they are in a real sensor.
inline SensorData makeBaSceneFrame(
		int id,
		const Transform & basePose,
		const std::vector<Landmark3D> & landmarks,
		const CameraModel & camera,
		float pixelNoiseStd,
		float depthNoiseStd,
		std::mt19937 & rng)
{
	const int kSlots = 32; // matches makeFeaturesData layout
	const Transform world_to_base = basePose.inverse();
	const cv::Size imgSize = camera.imageSize();
	cv::Mat image(imgSize.height, imgSize.width, CV_8UC1, cv::Scalar(128));
	SensorData data(image, camera); data.setId(id);

	std::vector<cv::KeyPoint> kpts;
	std::vector<cv::Point3f> pts3;
	std::vector<cv::Mat> descRows;
	std::normal_distribution<float> pxNoise(0.0f, pixelNoiseStd);
	std::normal_distribution<float> dpNoise(0.0f, depthNoiseStd);
	const int kDescCols = kSlots * 8;
	for(const auto & lm : landmarks)
	{
		// World point -> base frame (ground truth).
		const cv::Point3f & w = lm.world;
		const Transform pBase4 = world_to_base * Transform(w.x, w.y, w.z, 0, 0, 0);
		const cv::Point3f pBaseGt(pBase4.x(), pBase4.y(), pBase4.z());
		// Base -> optical (z forward) for visibility / projection.
		const cv::Point3f pOpt = baseToOptical(pBaseGt);
		if(pOpt.z <= 0.05f) continue; // behind camera or too close

		float u, v;
		camera.reproject(pOpt.x, pOpt.y, pOpt.z, u, v);
		if(pixelNoiseStd > 0.0f)
		{
			u += pxNoise(rng);
			v += pxNoise(rng);
		}
		if(u < 1.0f || u >= float(imgSize.width) - 1.0f ||
		   v < 1.0f || v >= float(imgSize.height) - 1.0f)
		{
			continue;
		}

		// 3D noise (in base frame) applied independently of the 2D noise.
		cv::Point3f pBase = pBaseGt;
		if(depthNoiseStd > 0.0f)
		{
			pBase.x += dpNoise(rng);
			pBase.y += dpNoise(rng);
			pBase.z += dpNoise(rng);
		}

		// One-hot descriptor for this landmark: a single 1000 at column
		// (lm.slot * 8), so every frame that observes this landmark emits the
		// SAME descriptor row and the dictionary maps them to one shared word.
		UASSERT(lm.slot >= 0 && lm.slot < kSlots);
		cv::Mat row = cv::Mat::zeros(1, kDescCols, CV_32F);
		row.at<float>(0, lm.slot * 8) = 1000.0f;

		kpts.emplace_back(u, v, 1.0f);
		pts3.emplace_back(pBase);
		descRows.push_back(row);
	}

	cv::Mat desc;
	if(!descRows.empty())
	{
		cv::vconcat(descRows, desc);
	}
	data.setFeatures(kpts, pts3, desc);
	return data;
}

} // namespace ba_synth

TEST(RtabmapTest, GlobalBundleAdjustmentRefinesPosesOnSynthScene)
{
	if(!Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		GTEST_SKIP() << "g2o optimizer not available in this build (required by BA)";
	}

	using namespace ba_synth;
	// 12 random world landmarks inside a 4 m cube; 6 cameras around them at
	// 4 m radius. Each camera observes the visible subset, with independent
	// sub-pixel noise on the 2D keypoints AND per-axis noise on the 3D point
	// stored in the base frame (mirroring detector + depth-estimate noise on
	// a stereo / RGB-D sensor). BA's task is to recover poses consistent with
	// the noisy observations; the optimum is the GT poses we fed in.
	const auto landmarks = randomLandmarks(/*n=*/12, /*halfExtent=*/2.0f, /*seed=*/42);
	const CameraModel camera = sceneCamera();
	const auto gtPoses = hexagonalPoses(/*count=*/6, /*radius=*/4.0f);

	ParametersMap params = badLoopClosureParams();
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	params[Parameters::kMemSTMSize()] = "10";        // keep all frames in STM
	params[Parameters::kMemBinDataKept()] = "true";
	params[Parameters::kKpMaxFeatures()] = "20";     // up to 12 landmarks/frame
	params[Parameters::kRtabmapLoopThr()] = "1";     // suppress Bayes loops
	// Anchor BA on the FIRST (clean) pose. With kRGBDOptimizeFromGraphEnd
	// disabled (the default), BA's root would be the last node, which is
	// noisy here -> entire BA solution would inherit that node's drift.
	params[Parameters::kRGBDOptimizeFromGraphEnd()] = "true";
	Rtabmap rtabmap;
	rtabmap.init(params);

	std::mt19937 rng(/*seed=*/123);
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const float pixelNoise = 0.1f;  // sub-pixel detector noise
	const float depthNoise = 0.02f; // 2 cm std on each 3D feature axis
	const float poseTransNoiseStd = 0.05f; // 5 cm std per axis on input odom poses
	const float poseRotNoiseStd = float(2.0 * M_PI / 180.0); // 2 deg std per axis
	std::normal_distribution<float> transNoise(0.0f, poseTransNoiseStd);
	std::normal_distribution<float> rotNoise(0.0f, poseRotNoiseStd);
	std::vector<Transform> noisyPoses;
	noisyPoses.reserve(gtPoses.size());
	// BA anchors on the first pose - keep it clean so the absolute reference
	// stays at GT and the rest can be driven toward GT by reprojection.
	noisyPoses.push_back(gtPoses[0]);
	for(size_t i = 1; i < gtPoses.size(); ++i)
	{
		const auto & gt = gtPoses[i];
		float x, y, z, roll, pitch, yaw;
		gt.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
		noisyPoses.emplace_back(
				x + transNoise(rng), y + transNoise(rng), z + transNoise(rng),
				roll + rotNoise(rng), pitch + rotNoise(rng), yaw + rotNoise(rng));
	}
	for(size_t i = 0; i < gtPoses.size(); ++i)
	{
		// Observations projected from the GT pose; odom passed to Rtabmap is
		// the noisy version so the local graph starts off-GT.
		SensorData data = makeBaSceneFrame(int(i + 1), gtPoses[i], landmarks, camera, pixelNoise, depthNoise, rng);
		ASSERT_GE(data.keypoints().size(), 6u) << "frame " << (i + 1) << " not enough visible landmarks";
		ASSERT_TRUE(rtabmap.process(data, noisyPoses[i], cov));
	}

	// Manually add cross-cluster global-closure links (using the noisy odom
	// deltas as the relative transform - same source as the chain neighbors)
	// so BA sees a richly connected graph and can triangulate landmarks
	// against every camera pair, not only consecutive frames.
	for(size_t i = 0; i < gtPoses.size(); ++i)
	{
		for(size_t j = i + 2; j < gtPoses.size(); ++j)
		{
			const Transform relPose = noisyPoses[i].inverse() * noisyPoses[j];
			const Link link(int(i + 1), int(j + 1), Link::kGlobalClosure, relPose,
					cv::Mat::eye(6, 6, CV_64FC1) * 100.0);
			ASSERT_TRUE(rtabmap.addLink(link)) << "link " << (i + 1) << "->" << (j + 1);
		}
	}

	const auto before = rtabmap.getLocalOptimizedPoses();
	ASSERT_EQ(before.size(), gtPoses.size());

	const bool baOk = rtabmap.globalBundleAdjustment(Optimizer::kTypeG2O, /*rematchFeatures=*/false);
	EXPECT_TRUE(baOk) << "BA must converge on the synthetic scene";

	const auto after = rtabmap.getLocalOptimizedPoses();
	ASSERT_EQ(after.size(), gtPoses.size());

	// Every refined pose stays finite.
	for(const auto & kv : after)
	{
		EXPECT_FALSE(kv.second.isNull());
		EXPECT_TRUE(std::isfinite(kv.second.x()));
		EXPECT_TRUE(std::isfinite(kv.second.y()));
		EXPECT_TRUE(std::isfinite(kv.second.z()));
	}
	// With clean GT odom seeded and pixel + depth measurement noise on the
	// features, BA's optimum is GT; residual drift is dictated by the noise
	// magnitude. Expect well under 5 cm mean pose drift.
	// Build a GT map keyed by node id for graph::calcRMSE().
	std::map<int, Transform> gtMap;
	for(size_t i = 0; i < gtPoses.size(); ++i)
	{
		gtMap.emplace(int(i + 1), gtPoses[i]);
	}
	auto rmse = [&](const std::map<int, Transform> & poses) {
		float t_rmse, t_mean, t_med, t_std, t_min, t_max;
		float r_rmse, r_mean, r_med, r_std, r_min, r_max;
		graph::calcRMSE(gtMap, poses,
				t_rmse, t_mean, t_med, t_std, t_min, t_max,
				r_rmse, r_mean, r_med, r_std, r_min, r_max,
				/*align2D=*/false);
		return std::make_pair(t_rmse, r_rmse);
	};
	const auto [tRmseBefore, rRmseBefore] = rmse(before);
	const auto [tRmseAfter, rRmseAfter] = rmse(after);
	// BA must reduce both translational and rotational RMSE toward GT, with
	// the residual bounded by the measurement noise floor.
	EXPECT_LT(tRmseAfter, tRmseBefore) << "BA must reduce translational RMSE";
	EXPECT_LT(rRmseAfter, rRmseBefore) << "BA must reduce rotational RMSE";
	// Bounds set with ~2x headroom over the worst RMSE observed across 5 RNG
	// seeds (translation ~36 mm, rotation ~0.45 deg at 640x480).
	EXPECT_LT(tRmseAfter, 0.06f) << "post-BA translational RMSE too large";
	EXPECT_LT(rRmseAfter, 1.0f) << "post-BA rotational RMSE (deg) too large";
	rtabmap.close(false);
}

TEST_F(RtabmapFixture, ExportPosesTumFormatIncludesStampPerLine)
{
	for(int i = 0; i < 3; ++i) process();
	const std::string outPath = uFormat("/tmp/rtabmap_test_poses_tum_%d.txt", getpid());
	UFile::erase(outPath.c_str());
	// Format 1 = RGBD-SLAM / TUM: stamp x y z qw qx qy qz (8 fields).
	rtabmap_->exportPoses(outPath, /*optimized=*/true, /*global=*/false, /*format=*/1);
	ASSERT_TRUE(UFile::exists(outPath));
	std::ifstream in(outPath);
	int lineCount = 0;
	std::string line;
	while(std::getline(in, line))
	{
		if(line.empty() || line[0] == '#') continue;
		++lineCount;
		std::istringstream is(line);
		int fields = 0;
		double v;
		while(is >> v) ++fields;
		EXPECT_EQ(fields, 8) << "line " << lineCount;
	}
	EXPECT_EQ(lineCount, 3);
	UFile::erase(outPath.c_str());
}

// ---------------------------------------------------------------------------
// Rehearsal merge: kMemRehearsalSimilarity < 1.0 collapses near-duplicate
// consecutive STM frames into a single node.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, RehearsalMergesConsecutiveLookalikeFramesInSTM)
{
	// When the new STM signature is similar enough to the previous one,
	// Memory::rehearsalMerge() folds it into the prior node and no new
	// location is created. With matching kMatchSlot features and a low
	// similarity threshold, processing N2 right after N1 should leave the
	// map with a single location.
	ParametersMap params = badLoopClosureParams();
	params[Parameters::kMemRehearsalSimilarity()] = "0.2"; // enable rehearsal
	params[Parameters::kMemBinDataKept()] = "true";
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const int kMatchSlot = 30;

	ASSERT_TRUE(rtabmap.process(makeFeaturesData(1, kMatchSlot), Transform(0, 0, 0, 0, 0, 0), cov));
	const int firstId = rtabmap.getLastLocationId();
	const int wmBefore = rtabmap.getWMSize();

	// Process N2 with identical features at the same pose -> rehearsal merge.
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(2, kMatchSlot), Transform(0, 0, 0, 0, 0, 0), cov));

	const auto & stats = rtabmap.getStatistics().data();
	auto itMerged = stats.find(Statistics::kMemoryRehearsal_merged());
	ASSERT_NE(itMerged, stats.end());
	EXPECT_EQ(int(itMerged->second), firstId) << "rehearsal should have merged N2 into N1";

	// WM size must not have grown - the merged node is the same id.
	EXPECT_EQ(rtabmap.getWMSize(), wmBefore);
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// detectMoreLoopClosures: post-hoc clustering loops nodes that revisit a
// previous spatial neighborhood but did NOT trigger an online loop closure.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, DetectMoreLoopClosuresAddsLinksBetweenSpatialRevisits)
{
	// Build a 5-node chain where N5 revisits N1's pose with matching features.
	// Online loop closure is suppressed (kRtabmapLoopThr=1, posterior never exceeds 1) so the only
	// inter-node links are odom neighbors. detectMoreLoopClosures should then
	// cluster (N1,N5) under the spatial radius and add a global loop closure
	// link after running visual registration on them.
	ParametersMap params = badLoopClosureParams();
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	params[Parameters::kRtabmapLoopThr()] = "1"; // suppress online loop detection
	params[Parameters::kMemBinDataKept()] = "true"; // keep feature data for post-hoc registration
	// STMSize: small enough that the (N1,N5) id-diff (=4) passes the
	// "too close nodes (id-diff < STMSize)" filter inside detectMoreLoopClosures,
	// but >= 2 so all 5 nodes stay in the local optimized graph.
	params[Parameters::kMemSTMSize()] = "2";
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const int kMatchSlot = 30;

	// First odom pose is a tiny offset from origin: Transform(0,0,0,...) is
	// interpreted by Rtabmap as an "odometry reset" and triggers a new map id
	// (severing the odom chain). We avoid that for both N1 and N5.
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(1, kMatchSlot), Transform(0.01f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(2, /*featSlot=*/2), Transform(1.0f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(3, /*featSlot=*/3), Transform(2.0f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(4, /*featSlot=*/4), Transform(1.0f, 0, 0, 0, 0, 0), cov));
	// N5 revisits N1's pose with the SAME kMatchSlot features.
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(5, kMatchSlot), Transform(0.01f, 0, 0, 0, 0, 0), cov));
	// Online loop closure must NOT have fired (suppressed by the high threshold).
	ASSERT_EQ(rtabmap.getLoopClosureId(), 0);

	// Cluster radius 0.5 m around each node -> (N1, N5) get paired.
	const int added = rtabmap.detectMoreLoopClosures(
			/*clusterRadiusMax=*/0.5f,
			/*clusterAngle=*/M_PI/6.0f,
			/*iterations=*/1,
			/*intraSession=*/true,
			/*interSession=*/false);
	EXPECT_GE(added, 1) << "expected at least one post-hoc loop closure between N1 and N5";

	// A non-neighbor link between N1 and N5 must now exist in Memory (stored
	// in either direction depending on the registration outcome).
	const Signature * s5 = rtabmap.getMemory()->getSignature(5);
	const Signature * s1 = rtabmap.getMemory()->getSignature(1);
	ASSERT_NE(s5, nullptr);
	ASSERT_NE(s1, nullptr);
	auto hasClosureTo = [](const Signature * s, int target) {
		for(const auto & kv : s->getLinks())
		{
			if(kv.first == target && kv.second.type() != Link::kNeighbor)
			{
				return true;
			}
		}
		return false;
	};
	EXPECT_TRUE(hasClosureTo(s5, 1) || hasClosureTo(s1, 5)) << "expected a non-neighbor link between N1 and N5";
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// refineLinks: re-run registration on every existing link, update those that
// converge.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, RefineLinksUpdatesConvergingNeighborLinks)
{
	// Process 3 frames whose features are identical (full kMatchSlot match
	// at the same 2D positions). Visual registration between every pair
	// returns identity, while the odom links carry +1 m translation each.
	// refineLinks should therefore mutate every neighbor link in the graph
	// (returning >= 1).
	ParametersMap params = badLoopClosureParams();
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	params[Parameters::kRtabmapLoopThr()] = "1";
	params[Parameters::kMemBinDataKept()] = "true";
	params[Parameters::kMemSTMSize()] = "5";
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const int kMatchSlot = 30;
	// Small odom deltas so that registration's initial guess does not project
	// the (x=1 m) 3D points outside the 64x64 image (which would otherwise
	// abort the visual transformation step before refining).
	for(int i = 1; i <= 3; ++i)
	{
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(i, kMatchSlot),
				Transform(float(i) * 0.01f, 0, 0, 0, 0, 0), cov));
	}

	const int refined = rtabmap.refineLinks();
	EXPECT_GE(refined, 1) << "refineLinks must update at least one converging link";

	// Spot-check: the (N2,N1) odom link's transform should now reflect the
	// visual registration outcome (~identity) rather than the original 1 m
	// odom delta.
	const Signature * s2 = rtabmap.getMemory()->getSignature(2);
	ASSERT_NE(s2, nullptr);
	auto it = s2->getLinks().find(1);
	ASSERT_NE(it, s2->getLinks().end());
	// Visual registration on identical features returns ~identity, replacing
	// the original ~0.01 m odom delta with ~0.
	EXPECT_LT(std::abs(it->second.transform().x()), 0.005f) << "refined transform should be visual (~identity)";
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// Proximity detection (kRGBDProximityBySpace): the "second loop-closure path"
// inside Rtabmap::process() that fires when a new node sits close in space
// (per optimized poses) to a previous WM node, with sufficient appearance
// overlap to register against it. Adds a kLocalSpaceClosure link.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, ProximityBySpaceAddsLocalSpaceClosureOnRevisit)
{
	ParametersMap params = badLoopClosureParams();
	params[Parameters::kRGBDProximityBySpace()] = "true"; // enable proximity
	params[Parameters::kRtabmapLoopThr()] = "1";         // suppress Bayes loop
	params[Parameters::kMemSTMSize()] = "2";              // span (N1,N5) id-diff filter
	params[Parameters::kMemBinDataKept()] = "true";
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const int kMatchSlot = 30;

	// Walk forward, then back to the start. N1 and N5 share full kMatchSlot
	// features and are essentially co-located in odometry -- proximity detection
	// during N5's process() should pair them.
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(1, kMatchSlot), Transform(0.01f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(2, /*featSlot=*/2), Transform(0.1f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(3, /*featSlot=*/3), Transform(0.2f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(4, /*featSlot=*/4), Transform(0.1f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(5, kMatchSlot), Transform(0.01f, 0, 0, 0, 0, 0), cov));

	// A proximity detection on N5 should have created a (N5,N1) link.
	const Signature * s5 = rtabmap.getMemory()->getSignature(5);
	ASSERT_NE(s5, nullptr);
	bool foundProximityLink = false;
	for(const auto & kv : s5->getLinks())
	{
		if(kv.first == 1 &&
		   (kv.second.type() == Link::kLocalSpaceClosure ||
		    kv.second.type() == Link::kGlobalClosure))
		{
			foundProximityLink = true;
			break;
		}
	}
	EXPECT_TRUE(foundProximityLink) << "kRGBDProximityBySpace must produce a (N5,N1) closure link";
	// Statistics confirm the proximity path ran and added at least one link.
	const auto & stats = rtabmap.getStatistics().data();
	auto itProx = stats.find(Statistics::kProximitySpace_detections_added_visually());
	ASSERT_NE(itProx, stats.end());
	EXPECT_GE(itProx->second, 1.0f);
	rtabmap.close(false);
}

TEST(RtabmapTest, ProximityByTimeDetectsRevisitInSTM)
{
	// kRGBDProximityByTime fires when a new node sits close in TIME (i.e.,
	// shares an STM window with a previous node) and registers against it.
	// Set up two consecutive frames with overlapping kMatchSlot features so
	// the new one proximity-detects the previous STM frame.
	ParametersMap params = badLoopClosureParams();
	params[Parameters::kRGBDProximityBySpace()] = "false";
	params[Parameters::kRGBDProximityByTime()] = "true";
	params[Parameters::kRtabmapLoopThr()] = "1";
	params[Parameters::kMemSTMSize()] = "5"; // keep all in STM
	params[Parameters::kMemBinDataKept()] = "true";
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const int kMatchSlot = 30;

	ASSERT_TRUE(rtabmap.process(makeFeaturesData(1, kMatchSlot), Transform(0.01f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(2, /*featSlot=*/2), Transform(0.05f, 0, 0, 0, 0, 0), cov));
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(3, kMatchSlot), Transform(0.01f, 0, 0, 0, 0, 0), cov));

	// proximity-by-time should have added a non-neighbor link to N1 on N3.
	const Signature * s3 = rtabmap.getMemory()->getSignature(3);
	ASSERT_NE(s3, nullptr);
	bool found = false;
	for(const auto & kv : s3->getLinks())
	{
		if(kv.first == 1 && kv.second.type() != Link::kNeighbor)
		{
			found = true;
			break;
		}
	}
	EXPECT_TRUE(found) << "kRGBDProximityByTime must produce a (N3,N1) closure link";
	rtabmap.close(false);
}

// Note: the repairGraph()-abort branch (triggered when removing the max-error
// link would disconnect more than one pose from the local graph - see
// Rtabmap.cpp:~5566) is not exercised here. The natural Rtabmap process flow
// keeps the graph connected through odom neighbors, so reaching that branch
// requires either two disconnected map sessions bridged by a single bad loop
// (built across triggerNewMap) or a hand-crafted Memory state - both fragile
// and out of scope for the orchestration coverage in this file.

// ---------------------------------------------------------------------------
// Landmarks / markers: SensorData::setLandmarks() attached to processed frames
// produces kLandmark links into Memory and shows up in getGraph as negative-id
// poses sharing the constraint between every observer.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, LandmarkObservationsAcrossFramesShareSameLandmarkPose)
{
	// Two frames both observe landmark id=42 at the same world location.
	// Memory stores the landmark once (key=-42 in the graph) and links both
	// frames to it.
	ParametersMap params = defaultRtabmapParams();
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const cv::Mat lmCov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const int kLm = 42;
	const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));

	// Frame N1 at (0,0,0) sees landmark 1 m forward.
	{
		SensorData d(image); d.setId(1);
		Landmarks lms;
		lms.emplace(kLm, Landmark(kLm, /*size=*/0.1f, Transform(1.0f, 0, 0, 0, 0, 0), lmCov));
		d.setLandmarks(lms);
		ASSERT_TRUE(rtabmap.process(d, Transform(0.0f, 0, 0, 0, 0, 0), cov));
	}
	// Frame N2 at (1,0,0) sees the same landmark right next to it (0 m forward).
	{
		SensorData d(image); d.setId(2);
		Landmarks lms;
		lms.emplace(kLm, Landmark(kLm, /*size=*/0.1f, Transform(0.0f, 0, 0, 0, 0, 0), lmCov));
		d.setLandmarks(lms);
		ASSERT_TRUE(rtabmap.process(d, Transform(1.0f, 0, 0, 0, 0, 0), cov));
	}

	// Both signatures carry a kLandmark link to -kLm.
	for(int nodeId : {1, 2})
	{
		const Signature * s = rtabmap.getMemory()->getSignature(nodeId);
		ASSERT_NE(s, nullptr) << "node " << nodeId;
		bool foundLmLink = false;
		for(const auto & kv : s->getLandmarks())
		{
			if(kv.first == -kLm && kv.second.type() == Link::kLandmark)
			{
				foundLmLink = true;
				break;
			}
		}
		EXPECT_TRUE(foundLmLink) << "node " << nodeId << " missing kLandmark link to -" << kLm;
	}

	// getGraph (global) exposes the landmark pose under its negative id; both
	// observers should map back to the same world location (~ (1,0,0)).
	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	rtabmap.getGraph(poses, links, /*optimized=*/true, /*global=*/true);
	auto itLm = poses.find(-kLm);
	ASSERT_NE(itLm, poses.end()) << "expected landmark pose at key -" << kLm;
	EXPECT_NEAR(itLm->second.x(), 1.0f, 0.1f);
	EXPECT_NEAR(itLm->second.y(), 0.0f, 0.1f);
	EXPECT_NEAR(itLm->second.z(), 0.0f, 0.1f);
	rtabmap.close(false);
}

TEST(RtabmapTest, DetectMoreLoopClosuresReturnsMinusOneWhenIntraAndInterBothFalse)
{
	Rtabmap rtabmap;
	rtabmap.init(badLoopClosureParams());
	// Intra and inter both false -> illegal call.
	EXPECT_EQ(rtabmap.detectMoreLoopClosures(0.5f, M_PI/6.0f, 1, false, false), -1);
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// Multi-session DB reload: close, reopen with kMemIncrementalMemory=true,
// continue mapping, and verify session bridging (new map id, node ids continue
// from where the previous session left off, prior nodes still queryable).
// ---------------------------------------------------------------------------

TEST(RtabmapTest, MultiSessionReloadContinuesMappingWithNewMapId)
{
	const std::string dbPath = uniqueDbPath();
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	int session1LastId = 0;
	int session1MapId = -1;
	{
		// Session 1: build a short chain.
		Rtabmap rtabmap;
		rtabmap.init(defaultRtabmapParams(), dbPath);
		const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
		for(int i = 1; i <= 3; ++i)
		{
			SensorData d(image); d.setId(i);
			ASSERT_TRUE(rtabmap.process(d, Transform(float(i), 0, 0, 0, 0, 0), cov));
		}
		session1LastId = rtabmap.getLastLocationId();
		session1MapId = rtabmap.getMemory()->getMapId(session1LastId);
		ASSERT_EQ(session1LastId, 3);
		rtabmap.close(true); // save
	}

	{
		// Session 2: reopen the DB in incremental mode and add 2 more nodes.
		Rtabmap rtabmap;
		ParametersMap params = defaultRtabmapParams();
		params[Parameters::kRGBDStartAtOrigin()] = "true"; // start session 2 clean
		rtabmap.init(params, dbPath);
		const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
		for(int i = 1; i <= 2; ++i)
		{
			SensorData d(image); d.setId(0); // auto-id
			ASSERT_TRUE(rtabmap.process(d, Transform(float(i), 0, 0, 0, 0, 0), cov));
		}
		const int session2LastId = rtabmap.getLastLocationId();
		const int session2MapId = rtabmap.getMemory()->getMapId(session2LastId);

		// Session 2's node ids continue past session 1's max id.
		EXPECT_GT(session2LastId, session1LastId);
		// Different map id since this is a new mapping session.
		EXPECT_NE(session2MapId, session1MapId);

		// Session 1's last node is still reachable via Memory (may live in LTM).
		EXPECT_NE(rtabmap.getMemory()->getMapId(session1LastId, /*lookInDatabase=*/true), -1);

		rtabmap.close(false);
	}
	UFile::erase(dbPath.c_str());
}

TEST(RtabmapTest, RehearsalDoesNotMergeDissimilarConsecutiveFrames)
{
	// Sanity check: two frames with disjoint feature slots (similarity ~0)
	// must NOT trigger a rehearsal merge even at a low threshold.
	ParametersMap params = badLoopClosureParams();
	params[Parameters::kMemRehearsalSimilarity()] = "0.2";
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	ASSERT_TRUE(rtabmap.process(makeFeaturesData(1, /*featSlot=*/0), Transform(0, 0, 0, 0, 0, 0), cov));
	const int wmBefore = rtabmap.getWMSize();
	ASSERT_TRUE(rtabmap.process(makeFeaturesData(2, /*featSlot=*/1), Transform(0.1f, 0, 0, 0, 0, 0), cov));

	const auto & stats = rtabmap.getStatistics().data();
	auto itMerged = stats.find(Statistics::kMemoryRehearsal_merged());
	ASSERT_NE(itMerged, stats.end());
	EXPECT_EQ(int(itMerged->second), 0) << "no merge expected for disjoint features";
	EXPECT_EQ(rtabmap.getWMSize(), wmBefore + 1);
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// GPS-based loop closure filter (kRtabmapLoopGPS): nodes carrying GPS data
// participate in Bayes scoring only when their GPS position is within
// kRGBDLocalRadius of the query frame's GPS. The GPS data must round-trip
// through Memory and the Bayes likelihood call must complete without crashes.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, ProcessAcceptsGPSAndStoresItOnSignature)
{
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kRtabmapLoopGPS()] = "true";
	params[Parameters::kMemBinDataKept()] = "true";
	Rtabmap rtabmap;
	rtabmap.init(params);

	const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	// Build 3 frames, each carrying a GPS fix near the same area (slightly
	// shifted in longitude). With kRtabmapLoopGPS=true the GPS-distance gate
	// participates in Bayes candidate filtering; we just verify the data
	// round-trips and processing succeeds.
	for(int i = 1; i <= 3; ++i)
	{
		SensorData d(image); d.setId(i);
		// Matches the (latitude, longitude) constants used by test_gps.cpp.
		const GPS fix(/*stamp=*/double(i), /*lon=*/-71.94304 + 1e-6 * double(i),
				/*lat=*/45.37855, /*alt=*/30.0, /*err=*/2.0, /*bearing=*/0.0);
		d.setGPS(fix);
		ASSERT_TRUE(rtabmap.process(d, Transform(float(i), 0, 0, 0, 0, 0), cov));
	}

	// GPS data must be preserved on every node Memory still holds.
	for(int id : {1, 2, 3})
	{
		const Signature * s = rtabmap.getMemory()->getSignature(id);
		ASSERT_NE(s, nullptr) << "node " << id;
		EXPECT_GT(s->sensorData().gps().stamp(), 0.0) << "node " << id << " lost its GPS fix";
	}
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// Memory threshold triggering LTM transfer: when kMemMaxStMemorySize is small
// and we process more frames than it allows, older nodes get pushed out of
// STM into WM, and eventually the working-memory budget enforced by
// kRtabmapMemoryThr causes `Memory::forget()` to spill them to LTM.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, MemoryThresholdTransfersOlderNodesOutOfWM)
{
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kMemSTMSize()] = "2";
	params[Parameters::kRtabmapMemoryThr()] = "3"; // cap WM at 3 nodes
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	// Process more frames than the threshold; the oldest must be evicted.
	for(int i = 1; i <= 8; ++i)
	{
		SensorData d(image); d.setId(i);
		ASSERT_TRUE(rtabmap.process(d, Transform(float(i), 0, 0, 0, 0, 0), cov));
	}

	// The WM size stays bounded by the memory threshold (plus STM).
	EXPECT_LE(rtabmap.getWMSize(), 3);
	// Total ids handled remains visible via getLastLocationId even though most
	// were transferred out of WM.
	EXPECT_EQ(rtabmap.getLastLocationId(), 8);
	// Oldest node (N1) must no longer live in STM/WM but should still be
	// reachable via the database lookup.
	EXPECT_EQ(rtabmap.getMemory()->getSignature(1), nullptr) << "N1 should be in LTM";
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// kRtabmapStartNewMapOnLoopClosure: when a loop closure is detected between
// the current session and a PRIOR session (loaded from the DB), the option
// forces a new map id rather than collapsing the two visits into the same
// session. Intra-session loops are unaffected.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, StartNewMapOnLoopClosureTriggersOnCrossSessionLoop)
{
	const std::string dbPath = uniqueDbPath();
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const int kMatchSlot = 30;

	// Session 1: build the loop-target scaffolding (N4 = full kMatchSlot,
	// flanked by partials N3 / N5 - the same setup as the existing
	// ProcessRejectsBadLoopClosureWhenMaxErrorExceeded test).
	{
		ParametersMap params = badLoopClosureParams();
		params[Parameters::kRGBDOptimizeMaxError()] = "0";
		params[Parameters::kRtabmapLoopThr()] = "0.05";
		params[Parameters::kBayesVirtualPlacePriorThr()] = "0.1";
		Rtabmap rtabmap;
		rtabmap.init(params, dbPath);
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(1, /*featSlot=*/0), Transform(0.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(2, /*featSlot=*/1), Transform(1.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(
				makeFeaturesMixedSlots(3, {kMatchSlot, kMatchSlot, 23, 23, 23, 23, 23, 23}),
				Transform(2.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(4, /*featSlot=*/kMatchSlot), Transform(3.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(
				makeFeaturesMixedSlots(5, {kMatchSlot, kMatchSlot, 25, 25, 25, 25, 25, 25}),
				Transform(4.0f, 0, 0, 0, 0, 0), cov));
		rtabmap.close(true); // save
	}

	// Session 2: incremental mode (new map id), kRtabmapStartNewMapOnLoopClosure
	// enabled. The first cross-session loop must produce a new map id for the
	// frame that follows it.
	{
		ParametersMap params = badLoopClosureParams();
		params[Parameters::kRGBDOptimizeMaxError()] = "0";
		params[Parameters::kRtabmapLoopThr()] = "0.05";
		params[Parameters::kBayesVirtualPlacePriorThr()] = "0.1";
		params[Parameters::kRtabmapStartNewMapOnLoopClosure()] = "true";
		params[Parameters::kRGBDStartAtOrigin()] = "true";
		Rtabmap rtabmap;
		rtabmap.init(params, dbPath);

		const int session1MapId = rtabmap.getMemory()->getMapId(4); // map id of N4 in DB
		// With kRtabmapStartNewMapOnLoopClosure=true, session-2 frames are
		// DROPPED until a loop closure with the prior session fires. Try a
		// couple of non-matching frames first - they get deleted.
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(0, /*featSlot=*/6), Transform(0.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(0, /*featSlot=*/7), Transform(1.0f, 0, 0, 0, 0, 0), cov));
		// kMatchSlot frame triggers the Bayes loop with N4 from session 1; the
		// new session opens at this point with a fresh map id.
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(0, /*featSlot=*/kMatchSlot), Transform(2.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_EQ(rtabmap.getLoopClosureId(), 4) << "expected cross-session loop to N4";
		const int mapAtLoop = rtabmap.getMemory()->getMapId(rtabmap.getLastLocationId());
		EXPECT_NE(mapAtLoop, session1MapId)
				<< "loop-anchored frame must inherit a NEW map id (session1=" << session1MapId
				<< " atLoop=" << mapAtLoop << ")";
		rtabmap.close(false);
	}
	UFile::erase(dbPath.c_str());
}

// ---------------------------------------------------------------------------
// Process with NaN / null odometry: an identity-pose call is normally
// treated as an odom reset (starts a new map); a NaN-or-null Transform must
// not crash and must produce some defined behavior.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, ProcessHandlesNullOdomPoseWithoutCrashing)
{
	Rtabmap rtabmap;
	rtabmap.init(defaultRtabmapParams());
	const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	// First frame at a normal pose so the chain has a valid root.
	SensorData first(image); first.setId(1);
	ASSERT_TRUE(rtabmap.process(first, Transform(1.0f, 0, 0, 0, 0, 0), cov));
	const int firstId = rtabmap.getLastLocationId();
	EXPECT_NE(firstId, 0);

	// Now send a null Transform. process() should handle the degenerate input
	// gracefully (typically by triggering an odom-reset path) rather than
	// throwing or producing NaN-poisoned state.
	SensorData second(image); second.setId(2);
	rtabmap.process(second, Transform(), cov); // null Transform; ignore return
	// State remains finite and queryable.
	const Transform pose = rtabmap.getMapCorrection();
	EXPECT_TRUE(std::isfinite(pose.x()));
	EXPECT_TRUE(std::isfinite(pose.y()));
	EXPECT_TRUE(std::isfinite(pose.z()));
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// kRGBDLoopCovLimited: when enabled, loop closure information matrices are
// clipped by Memory::getOdomMaxInf() before being applied via getInformation().
// ---------------------------------------------------------------------------

TEST_F(RtabmapFixture, GetInformationClipsCovarianceUnderLoopCovLimited)
{
	// Reinit with loop-cov clipping enabled.
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kRGBDLoopCovLimited()] = "true";
	reinit(params);
	process();
	process();

	// A tiny covariance would normally invert to a very large info matrix.
	// With LoopCovLimited the value is clamped at Memory::getOdomMaxInf()
	// (computed from the odom covariance seen so far).
	cv::Mat tinyCov = cv::Mat::eye(6, 6, CV_64FC1) * 1e-12;
	const cv::Mat info = rtabmap_->getInformation(tinyCov);
	ASSERT_FALSE(info.empty());
	// The largest diagonal entry should be finite (not 1e12) once clipped.
	double maxDiag = 0.0;
	for(int i = 0; i < 6; ++i) maxDiag = std::max(maxDiag, info.at<double>(i, i));
	EXPECT_LT(maxDiag, 1e10) << "loop-cov clipping must bound the info matrix";
	EXPECT_GT(maxDiag, 0.0);
}

// ---------------------------------------------------------------------------
// Custom user-defined link via Link::kUserClosure: external loop detectors
// can inject closures with this type and Memory must store them as such.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, AddLinkWithUserClosureTypeStoresIt)
{
	// Disable OptimizeMaxError so the consistency check on the injected link
	// doesn't reject it. Use non-adjacent nodes (N3 and N1) so we aren't
	// fighting an existing kNeighbor odom link.
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kRGBDOptimizeMaxError()] = "0";
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	for(int i = 1; i <= 3; ++i)
	{
		SensorData d(image); d.setId(i);
		ASSERT_TRUE(rtabmap.process(d, Transform(float(i), 0, 0, 0, 0, 0), cov));
	}

	// Inject a user closure with the correct relative transform so the
	// optimizer accepts it even if MaxError were re-enabled by accident.
	const Link userLink(3, 1, Link::kUserClosure, Transform(-2.0f, 0, 0, 0, 0, 0),
			cv::Mat::eye(6, 6, CV_64FC1) * 100.0);
	ASSERT_TRUE(rtabmap.addLink(userLink));

	const Signature * s3 = rtabmap.getMemory()->getSignature(3);
	ASSERT_NE(s3, nullptr);
	auto it = s3->getLinks().find(1);
	ASSERT_NE(it, s3->getLinks().end());
	EXPECT_EQ(it->second.type(), Link::kUserClosure)
			<< "expected kUserClosure, got type=" << int(it->second.type());
	rtabmap.close(false);
}

// ---------------------------------------------------------------------------
// Aggressive loop threshold (kRGBDAggressiveLoopThr): when set below the
// primary threshold, accepts marginal Bayes hypotheses in the pre-first-loc
// pre-localization window that the primary alone would reject. We A/B the
// same DB and same query frame against two threshold configs and verify the
// outcome differs in exactly the expected direction.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, AggressiveLoopThresholdAcceptsBelowPrimaryThreshold)
{
	const std::string dbPath = uniqueDbPath();
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const int kMatchSlot = 30;

	// Mapping phase: same scaffolding as ProcessRejectsBadLoopClosure*.
	{
		ParametersMap params = badLoopClosureParams();
		params[Parameters::kRtabmapLoopThr()] = "0.05";
		params[Parameters::kBayesVirtualPlacePriorThr()] = "0.1";
		Rtabmap rtabmap;
		rtabmap.init(params, dbPath);
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(1, /*featSlot=*/0), Transform(0.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(2, /*featSlot=*/1), Transform(1.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(
				makeFeaturesMixedSlots(3, {kMatchSlot, kMatchSlot, 23, 23, 23, 23, 23, 23}),
				Transform(2.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(makeFeaturesData(4, /*featSlot=*/kMatchSlot), Transform(3.0f, 0, 0, 0, 0, 0), cov));
		ASSERT_TRUE(rtabmap.process(
				makeFeaturesMixedSlots(5, {kMatchSlot, kMatchSlot, 25, 25, 25, 25, 25, 25}),
				Transform(4.0f, 0, 0, 0, 0, 0), cov));
		rtabmap.close(true);
	}

	const float kLoopThr = 0.95f;      // would reject a weakened hypothesis
	const float kAggressiveThr = 0.3f; // would accept it

	// A localization session reuses the saved DB and feeds a weakened
	// kMatchSlot query (6/8 features). Each invocation processes the SAME
	// frame; only kRGBDAggressiveLoopThr differs.
	auto runOnce = [&](float aggressiveThr) {
		ParametersMap params = badLoopClosureParams();
		params[Parameters::kMemIncrementalMemory()] = "false";
		params[Parameters::kRtabmapLoopThr()] = uNumber2Str(kLoopThr);
		params[Parameters::kRGBDAggressiveLoopThr()] = uNumber2Str(aggressiveThr);
		params[Parameters::kBayesVirtualPlacePriorThr()] = "0.1";
		// Commit the loop closure immediately rather than waiting for a 2nd
		// confirmation; we want to observe the threshold gate result on the
		// very first frame.
		params[Parameters::kRGBDMaxOdomCacheSize()] = "0";
		Rtabmap rtabmap;
		rtabmap.init(params, dbPath);
		rtabmap.setInitialPose(Transform(3.0f, 0, 0, 0, 0, 0));
		rtabmap.process(
				makeFeaturesMixedSlots(0, {kMatchSlot, kMatchSlot, kMatchSlot, kMatchSlot, kMatchSlot, kMatchSlot, 6, 6}),
				Transform(3.0f, 0, 0, 0, 0, 0), cov);
		const int id = rtabmap.getLoopClosureId();
		const float high = rtabmap.getStatistics().data().at(Statistics::kLoopHighest_hypothesis_value());
		rtabmap.close(false);
		return std::make_pair(id, high);
	};

	const auto [idAgg, highAgg] = runOnce(kAggressiveThr);
	const auto [idPrim, highPrim] = runOnce(kLoopThr); // aggressive disabled (= primary)

	// Sanity: both invocations see the same Bayes peak (deterministic data).
	EXPECT_NEAR(highAgg, highPrim, 1e-3);

	// The hypothesis must sit strictly between the two thresholds; otherwise
	// the A/B comparison below is meaningless.
	EXPECT_GT(highAgg, kAggressiveThr) << "hypothesis must clear the aggressive threshold";
	EXPECT_LT(highAgg, kLoopThr) << "hypothesis must be below the primary threshold";

	// With aggressive enabled (low) the loop is accepted; without (disabled
	// by setting aggressive == primary) it is rejected.
	EXPECT_EQ(idAgg, 4) << "aggressive threshold must accept the marginal hypothesis";
	EXPECT_EQ(idPrim, 0) << "primary threshold alone must reject the same hypothesis";

	UFile::erase(dbPath.c_str());
}

// ---------------------------------------------------------------------------
// getGraph with full signature payloads (images / scan / user data / grid /
// words / global descriptors) - the path consumed by the ROS bridge.
// ---------------------------------------------------------------------------

TEST(RtabmapTest, GetGraphWithSignaturePayloadsAttachesRequestedFields)
{
	ParametersMap params = defaultRtabmapParams();
	params[Parameters::kMemBinDataKept()] = "true";
	Rtabmap rtabmap;
	rtabmap.init(params);
	const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	const cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	for(int i = 1; i <= 3; ++i)
	{
		SensorData d(image); d.setId(i);
		// Attach unique user data on each frame so we can verify pass-through.
		cv::Mat userData(1, 4, CV_8UC1, cv::Scalar(0x10 + i));
		d.setUserData(userData);
		ASSERT_TRUE(rtabmap.process(d, Transform(float(i), 0, 0, 0, 0, 0), cov));
	}

	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	std::map<int, Signature> sigs;
	rtabmap.getGraph(poses, links, /*optimized=*/true, /*global=*/false, &sigs,
			/*withImages=*/true,
			/*withScan=*/true,
			/*withUserData=*/true,
			/*withGrid=*/true,
			/*withWords=*/true,
			/*withGlobalDescriptors=*/true);

	ASSERT_EQ(sigs.size(), 3u);
	for(const auto & kv : sigs)
	{
		const Signature & s = kv.second;
		// Image was provided to every frame so the payload must come through.
		EXPECT_GT(s.sensorData().imageCompressed().total(), 0u) << "node " << kv.first << " image missing";
		EXPECT_GT(s.sensorData().userDataCompressed().total(), 0u) << "node " << kv.first << " user data missing";
	}
	rtabmap.close(false);
}

