#include <gtest/gtest.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/VisualWord.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/GPS.h>
#include <rtabmap/core/EnvSensor.h>
#include <rtabmap/core/GlobalDescriptor.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <list>
#include <sstream>
#include <thread>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace rtabmap;

namespace {

std::string uniqueDbPath()
{
	static int counter = 0;
	return uFormat("/tmp/rtabmap_dbdriver_test_%d_%d.db", getpid(), ++counter);
}

class DbDriverFixture : public ::testing::Test
{
protected:
	void SetUp() override
	{
		dbPath_ = uniqueDbPath();
		driver_ = DBDriver::create();
		ASSERT_NE(driver_, nullptr);
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

	void saveVisualWord(VisualWord * w)
	{
		driver_->asyncSave(w);
		driver_->emptyTrashes(false);
	}

	static void freeVisualWords(std::list<VisualWord *> & words)
	{
		for(VisualWord * w : words)
		{
			delete w;
		}
		words.clear();
	}

	static void setSignatureFeatures(
			Signature & sig,
			const std::vector<int> & wordIds,
			const std::vector<cv::KeyPoint> & keypoints,
			const std::vector<cv::Point3f> & words3,
			const cv::Mat & descriptors)
	{
		ASSERT_EQ(wordIds.size(), keypoints.size());
		ASSERT_EQ(wordIds.size(), words3.size());
		ASSERT_EQ((int)wordIds.size(), descriptors.rows);

		std::multimap<int, int> words;
		for(size_t i = 0; i < wordIds.size(); ++i)
		{
			words.insert(std::make_pair(wordIds[i], (int)i));
		}
		sig.setWords(words, keypoints, words3, descriptors);
	}

	static cv::Mat infMatrixDiagonal(
			double x,
			double y,
			double z,
			double roll,
			double pitch,
			double yaw)
	{
		cv::Mat inf = cv::Mat::zeros(6, 6, CV_64FC1);
		inf.at<double>(0, 0) = x;
		inf.at<double>(1, 1) = y;
		inf.at<double>(2, 2) = z;
		inf.at<double>(3, 3) = roll;
		inf.at<double>(4, 4) = pitch;
		inf.at<double>(5, 5) = yaw;
		return inf;
	}

	static void expectTransformNear(const Transform & a, const Transform & b, float tol = 1e-4f)
	{
		EXPECT_NEAR(a.x(), b.x(), tol);
		EXPECT_NEAR(a.y(), b.y(), tol);
		EXPECT_NEAR(a.z(), b.z(), tol);
	}

	static void expectLinkEqual(const Link & expected, const Link & loaded)
	{
		EXPECT_EQ(loaded.from(), expected.from());
		EXPECT_EQ(loaded.to(), expected.to());
		EXPECT_EQ(loaded.type(), expected.type());
		expectTransformNear(expected.transform(), loaded.transform());
		ASSERT_EQ(expected.infMatrix().rows, loaded.infMatrix().rows);
		ASSERT_EQ(expected.infMatrix().cols, loaded.infMatrix().cols);
		EXPECT_LT(cv::norm(expected.infMatrix(), loaded.infMatrix(), cv::NORM_INF), 1e-6);
	}

	static void expectLinksEqual(
			const std::multimap<int, Link> & expected,
			const std::multimap<int, Link> & loaded)
	{
		EXPECT_EQ(expected.size(), loaded.size());
		for(std::multimap<int, Link>::const_iterator it = expected.begin(); it != expected.end(); ++it)
		{
			std::pair<std::multimap<int, Link>::const_iterator, std::multimap<int, Link>::const_iterator> range =
					loaded.equal_range(it->first);
			bool found = false;
			for(std::multimap<int, Link>::const_iterator j = range.first; j != range.second; ++j)
			{
				if(j->second.to() == it->second.to() && j->second.type() == it->second.type())
				{
					expectLinkEqual(it->second, j->second);
					found = true;
					break;
				}
			}
			EXPECT_TRUE(found) << "missing link to " << it->second.to();
		}
	}

	static void expectLandmarksEqual(
			const std::map<int, Link> & expected,
			const std::map<int, Link> & loaded)
	{
		EXPECT_EQ(expected.size(), loaded.size());
		for(std::map<int, Link>::const_iterator it = expected.begin(); it != expected.end(); ++it)
		{
			ASSERT_TRUE(loaded.count(it->first));
			expectLinkEqual(it->second, loaded.at(it->first));
		}
	}

	static void expectVelocityEqual(
			const std::vector<float> & expected,
			const std::vector<float> & loaded)
	{
		ASSERT_EQ(expected.size(), loaded.size());
		for(size_t i = 0; i < expected.size(); ++i)
		{
			EXPECT_NEAR(loaded[i], expected[i], 1e-5f);
		}
	}

	/** Attach SensorData payloads stored in the Data table (compressed where required for save). */
	static void attachSensorDataForDatabaseSave(Signature & sig)
	{
		const CameraModel model(525.0, 525.0, 320.0, 240.0);

		const cv::Mat image(16, 16, CV_8UC1, cv::Scalar(42));
		const cv::Mat depth(16, 16, CV_16UC1, cv::Scalar(1000));
		const cv::Mat imageCompressed = compressImage2(image, ".png");
		const cv::Mat depthCompressed = compressImage2(depth, ".png");
		ASSERT_FALSE(imageCompressed.empty());
		ASSERT_FALSE(depthCompressed.empty());
		sig.sensorData().setRGBDImage(imageCompressed, depthCompressed, model);

		cv::Mat scanMat(1, 4, CV_32FC2);
		for(int i = 0; i < 4; ++i)
		{
			scanMat.at<cv::Vec2f>(0, i) = cv::Vec2f(1.f + i, 0.1f * i);
		}
		const LaserScan scanRaw = LaserScan::backwardCompatibility(scanMat, 4, 10.f);
		const cv::Mat scanCompressed = compressData2(scanRaw.data());
		ASSERT_FALSE(scanCompressed.empty());
		const LaserScan scanForSave(
				scanCompressed,
				LaserScan::kXY,
				scanRaw.rangeMin(),
				scanRaw.rangeMax(),
				scanRaw.angleMin(),
				scanRaw.angleMax(),
				scanRaw.angleIncrement());
		sig.sensorData().setLaserScan(scanForSave);

		const cv::Mat userData(4, 4, CV_8UC1, cv::Scalar(128));
		sig.sensorData().setUserData(userData);

		cv::Mat ground(1, 3, CV_32FC2);
		ground.at<cv::Vec2f>(0, 0) = cv::Vec2f(0.f, 0.f);
		ground.at<cv::Vec2f>(0, 1) = cv::Vec2f(1.f, 0.f);
		ground.at<cv::Vec2f>(0, 2) = cv::Vec2f(2.f, 0.f);
		cv::Mat obstacles(1, 2, CV_32FC2);
		obstacles.at<cv::Vec2f>(0, 0) = cv::Vec2f(5.f, 0.f);
		obstacles.at<cv::Vec2f>(0, 1) = cv::Vec2f(6.f, 0.f);
		cv::Mat empty(1, 1, CV_32FC2);
		empty.at<cv::Vec2f>(0, 0) = cv::Vec2f(7.f, 0.f);
		sig.sensorData().setOccupancyGrid(ground, obstacles, empty, 0.05f, cv::Point3f(1.f, 2.f, 3.f));
	}

	static void expectLaserScanNear(const LaserScan & expected, const LaserScan & loaded)
	{
		EXPECT_EQ(loaded.format(), expected.format());
		EXPECT_NEAR(loaded.rangeMax(), expected.rangeMax(), 1e-3f);
		EXPECT_EQ(loaded.size(), expected.size());
		ASSERT_FALSE(expected.data().empty());
		ASSERT_FALSE(loaded.data().empty());
		EXPECT_EQ(cv::norm(expected.data(), loaded.data(), cv::NORM_INF), 0);
	}

	/**
	 * SensorData fields reloaded by loadNodeData() from the Data table (after uncompressData()).
	 * Not loaded by loadSignatures() alone.
	 */
	static void expectSensorDataFromLoadNodeDataEqual(
			const SensorData & expected,
			const SensorData & loaded)
	{
		ASSERT_FALSE(expected.imageRaw().empty());
		ASSERT_FALSE(loaded.imageRaw().empty());
		EXPECT_EQ(expected.imageRaw().rows, loaded.imageRaw().rows);
		EXPECT_EQ(expected.imageRaw().cols, loaded.imageRaw().cols);
		EXPECT_EQ(expected.imageRaw().type(), loaded.imageRaw().type());
		EXPECT_EQ(expected.imageRaw().at<unsigned char>(0, 0), loaded.imageRaw().at<unsigned char>(0, 0));

		ASSERT_FALSE(expected.depthRaw().empty());
		ASSERT_FALSE(loaded.depthRaw().empty());
		EXPECT_EQ(expected.depthRaw().type(), loaded.depthRaw().type());
		EXPECT_EQ(expected.depthRaw().at<uint16_t>(0, 0), loaded.depthRaw().at<uint16_t>(0, 0));

		expectLaserScanNear(expected.laserScanRaw(), loaded.laserScanRaw());

		ASSERT_FALSE(expected.userDataRaw().empty());
		ASSERT_FALSE(loaded.userDataRaw().empty());
		EXPECT_EQ(cv::norm(expected.userDataRaw(), loaded.userDataRaw(), cv::NORM_INF), 0);

		ASSERT_FALSE(expected.gridGroundCellsRaw().empty());
		ASSERT_FALSE(loaded.gridGroundCellsRaw().empty());
		EXPECT_EQ(expected.gridGroundCellsRaw().cols, loaded.gridGroundCellsRaw().cols);
		EXPECT_EQ(expected.gridObstacleCellsRaw().cols, loaded.gridObstacleCellsRaw().cols);
		EXPECT_EQ(expected.gridEmptyCellsRaw().cols, loaded.gridEmptyCellsRaw().cols);
		EXPECT_FLOAT_EQ(loaded.gridCellSize(), expected.gridCellSize());
		EXPECT_FLOAT_EQ(loaded.gridViewPoint().x, expected.gridViewPoint().x);
		EXPECT_FLOAT_EQ(loaded.gridViewPoint().y, expected.gridViewPoint().y);
		EXPECT_FLOAT_EQ(loaded.gridViewPoint().z, expected.gridViewPoint().z);
	}

	/**
	 * SensorData fields reloaded by loadSignatures() / loadSignaturesQuery():
	 * - GPS and env sensors (from Node table, stored in sensorData())
	 * - Camera / stereo calibration (from Data.calibration)
	 * - Global descriptors (from GlobalDescriptor table, DB >= 0.20.0)
	 *
	 * Not reloaded by loadSignatures() — use loadNodeData() instead:
	 * - RGB/depth images, depth confidence, laser scan, user data, occupancy grid
	 */
	static void expectSensorDataFromLoadSignaturesEqual(
			const SensorData & expected,
			const SensorData & loaded)
	{
		EXPECT_EQ(loaded.id(), expected.id());

		EXPECT_DOUBLE_EQ(loaded.gps().stamp(), expected.gps().stamp());
		EXPECT_DOUBLE_EQ(loaded.gps().longitude(), expected.gps().longitude());
		EXPECT_DOUBLE_EQ(loaded.gps().latitude(), expected.gps().latitude());
		EXPECT_DOUBLE_EQ(loaded.gps().altitude(), expected.gps().altitude());
		EXPECT_DOUBLE_EQ(loaded.gps().error(), expected.gps().error());
		EXPECT_DOUBLE_EQ(loaded.gps().bearing(), expected.gps().bearing());

		ASSERT_EQ(loaded.envSensors().size(), expected.envSensors().size());
		for(EnvSensors::const_iterator it = expected.envSensors().begin(); it != expected.envSensors().end(); ++it)
		{
			ASSERT_TRUE(loaded.envSensors().count(it->first));
			const EnvSensor & loadedSensor = loaded.envSensors().at(it->first);
			EXPECT_EQ(loadedSensor.type(), it->second.type());
			EXPECT_DOUBLE_EQ(loadedSensor.value(), it->second.value());
			EXPECT_DOUBLE_EQ(loadedSensor.stamp(), it->second.stamp());
		}

		ASSERT_EQ(loaded.cameraModels().size(), expected.cameraModels().size());
		for(size_t i = 0; i < expected.cameraModels().size(); ++i)
		{
			EXPECT_DOUBLE_EQ(loaded.cameraModels()[i].fx(), expected.cameraModels()[i].fx());
			EXPECT_DOUBLE_EQ(loaded.cameraModels()[i].fy(), expected.cameraModels()[i].fy());
			EXPECT_DOUBLE_EQ(loaded.cameraModels()[i].cx(), expected.cameraModels()[i].cx());
			EXPECT_DOUBLE_EQ(loaded.cameraModels()[i].cy(), expected.cameraModels()[i].cy());
			expectTransformNear(expected.cameraModels()[i].localTransform(), loaded.cameraModels()[i].localTransform());
		}

		ASSERT_EQ(loaded.stereoCameraModels().size(), expected.stereoCameraModels().size());
		for(size_t i = 0; i < expected.stereoCameraModels().size(); ++i)
		{
			EXPECT_DOUBLE_EQ(loaded.stereoCameraModels()[i].left().fx(), expected.stereoCameraModels()[i].left().fx());
			EXPECT_DOUBLE_EQ(loaded.stereoCameraModels()[i].left().fy(), expected.stereoCameraModels()[i].left().fy());
			EXPECT_DOUBLE_EQ(loaded.stereoCameraModels()[i].left().cx(), expected.stereoCameraModels()[i].left().cx());
			EXPECT_DOUBLE_EQ(loaded.stereoCameraModels()[i].left().cy(), expected.stereoCameraModels()[i].left().cy());
			EXPECT_DOUBLE_EQ(loaded.stereoCameraModels()[i].baseline(), expected.stereoCameraModels()[i].baseline());
		}

		ASSERT_EQ(loaded.globalDescriptors().size(), expected.globalDescriptors().size());
		for(size_t i = 0; i < expected.globalDescriptors().size(); ++i)
		{
			EXPECT_EQ(loaded.globalDescriptors()[i].type(), expected.globalDescriptors()[i].type());
			ASSERT_FALSE(expected.globalDescriptors()[i].data().empty());
			ASSERT_FALSE(loaded.globalDescriptors()[i].data().empty());
			EXPECT_EQ(cv::norm(expected.globalDescriptors()[i].data(), loaded.globalDescriptors()[i].data(), cv::NORM_INF), 0);
		}
	}

	/** Compare Signature fields persisted and reloaded by loadSignatures(). */
	static void expectSignatureMembersEqual(
			const Signature & expected,
			const Signature & loaded)
	{
		EXPECT_EQ(loaded.id(), expected.id());
		EXPECT_EQ(loaded.mapId(), expected.mapId());
		EXPECT_EQ(loaded.getWeight(), expected.getWeight());
		EXPECT_DOUBLE_EQ(loaded.getStamp(), expected.getStamp());
		EXPECT_EQ(loaded.getLabel(), expected.getLabel());

		expectTransformNear(expected.getPose(), loaded.getPose());
		expectTransformNear(expected.getGroundTruthPose(), loaded.getGroundTruthPose());
		expectVelocityEqual(expected.getVelocity(), loaded.getVelocity());

		expectSignatureFeaturesEqual(expected, loaded);
		EXPECT_EQ(loaded.getInvalidWordsCount(), expected.getInvalidWordsCount());
		EXPECT_EQ(loaded.isBadSignature(), expected.isBadSignature());

		expectLinksEqual(expected.getLinks(), loaded.getLinks());
		expectLandmarksEqual(expected.getLandmarks(), loaded.getLandmarks());

		if(!expected.getLinks().empty())
		{
			EXPECT_LT(cv::norm(expected.getPoseCovariance(), loaded.getPoseCovariance(), cv::NORM_INF), 1e-6);
		}

		EXPECT_TRUE(loaded.getWordsChanged().empty());
		EXPECT_TRUE(loaded.isSaved());
		EXPECT_FALSE(loaded.isModified());
		EXPECT_FALSE(loaded.isLinksModified());

		expectSensorDataFromLoadSignaturesEqual(expected.sensorData(), loaded.sensorData());
	}

	static void expectSignatureFeaturesEqual(
			const Signature & expected,
			const Signature & loaded)
	{
		EXPECT_EQ(loaded.getWords().size(), expected.getWords().size());
		EXPECT_EQ(loaded.getWordsKpts().size(), expected.getWordsKpts().size());
		EXPECT_EQ(loaded.getWords3().size(), expected.getWords3().size());
		EXPECT_EQ(loaded.getWordsDescriptors().rows, expected.getWordsDescriptors().rows);
		EXPECT_EQ(loaded.getWordsDescriptors().cols, expected.getWordsDescriptors().cols);
		EXPECT_EQ(loaded.getWordsDescriptors().type(), expected.getWordsDescriptors().type());

		for(std::multimap<int, int>::const_iterator it = expected.getWords().begin();
				it != expected.getWords().end();
				++it)
		{
			EXPECT_EQ(loaded.getWords().count(it->first), expected.getWords().count(it->first));
			EXPECT_EQ(loaded.getWords().find(it->first)->second, it->second);
		}

		for(size_t i = 0; i < expected.getWordsKpts().size(); ++i)
		{
			EXPECT_FLOAT_EQ(loaded.getWordsKpts()[i].pt.x, expected.getWordsKpts()[i].pt.x);
			EXPECT_FLOAT_EQ(loaded.getWordsKpts()[i].pt.y, expected.getWordsKpts()[i].pt.y);
			EXPECT_FLOAT_EQ(loaded.getWordsKpts()[i].size, expected.getWordsKpts()[i].size);
			EXPECT_NEAR(loaded.getWordsKpts()[i].angle, expected.getWordsKpts()[i].angle, 1e-3f);
			EXPECT_NEAR(loaded.getWordsKpts()[i].response, expected.getWordsKpts()[i].response, 1e-5f);
			EXPECT_EQ(loaded.getWordsKpts()[i].octave, expected.getWordsKpts()[i].octave);
		}

		for(size_t i = 0; i < expected.getWords3().size(); ++i)
		{
			EXPECT_FLOAT_EQ(loaded.getWords3()[i].x, expected.getWords3()[i].x);
			EXPECT_FLOAT_EQ(loaded.getWords3()[i].y, expected.getWords3()[i].y);
			EXPECT_FLOAT_EQ(loaded.getWords3()[i].z, expected.getWords3()[i].z);
		}

		if(!expected.getWordsDescriptors().empty())
		{
			EXPECT_EQ(cv::norm(expected.getWordsDescriptors(), loaded.getWordsDescriptors(), cv::NORM_INF), 0);
		}
	}

	std::string dbPath_;
	DBDriver * driver_ = nullptr;
};

} // namespace

TEST(DBDriverTest, CreateReturnsNonNullDriver)
{
	DBDriver * driver = DBDriver::create();
	ASSERT_NE(driver, nullptr);
	driver->closeConnection(false);
	delete driver;
}

TEST(DBDriverTest, OpenEmptyUrlUsesInMemoryDatabase)
{
	DBDriver * driver = DBDriver::create();
	ASSERT_TRUE(driver->openConnection(""));
	EXPECT_TRUE(driver->isInMemory());
	EXPECT_TRUE(driver->isConnected());
	EXPECT_FALSE(driver->getDatabaseVersion().empty());
	driver->closeConnection(false);
	delete driver;
}

TEST_F(DbDriverFixture, SaveAndLoadSignatureAllMembers)
{
	// Link targets must exist before saving links on signature 1.
	saveSignature(new Signature(2));
	saveSignature(new Signature(3));

	const Transform pose(1.f, 2.f, 3.f, 0.1f, 0.2f, 0.3f);
	const Transform groundTruth(1.1f, 2.1f, 3.1f, 0.f, 0.f, 0.f);
	Signature * sig = new Signature(1, 5, 7, 123.456, "room_a", pose, groundTruth);
	sig->setVelocity(0.5f, -0.2f, 0.1f, 0.01f, 0.02f, 0.03f);

	const std::vector<int> wordIds = {10, 20, 30};
	const std::vector<cv::KeyPoint> keypoints = {
		cv::KeyPoint(10.f, 20.f, 8.f, 45.f, 0.9f, 1),
		cv::KeyPoint(30.f, 40.f, 8.f, 90.f, 0.8f, 2),
		cv::KeyPoint(50.f, 60.f, 8.f, 135.f, 0.7f, 3)};
	const std::vector<cv::Point3f> words3 = {
		cv::Point3f(0.1f, 0.2f, 0.3f),
		cv::Point3f(0.4f, 0.5f, 0.6f),
		cv::Point3f(0.7f, 0.8f, 0.9f)};
	const cv::Mat descriptors = (cv::Mat_<float>(3, 8) <<
		1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f,
		9.f, 10.f, 11.f, 12.f, 13.f, 14.f, 15.f, 16.f,
		17.f, 18.f, 19.f, 20.f, 21.f, 22.f, 23.f, 24.f);
	setSignatureFeatures(*sig, wordIds, keypoints, words3, descriptors);

	sig->addLink(Link(1, 2, Link::kNeighbor,
			Transform(0.5f, 0.f, 0.f, 0.f, 0.f, 0.f),
			infMatrixDiagonal(100., 100., 100., 10., 10., 10.)));
	sig->addLink(Link(1, 3, Link::kGlobalClosure,
			Transform(0.1f, 0.2f, 0.f, 0.f, 0.f, 0.5f)));
	sig->addLandmark(Link(1, -10, Link::kLandmark,
			Transform(0.2f, -0.3f, 0.4f, 0.f, 0.f, 0.f)));

	sig->sensorData().setGPS(GPS(123.456, -71.94304, 45.37855, 250.0, 3.0, 180.0));
	EnvSensors envSensors;
	envSensors.insert(std::make_pair(EnvSensor::kAmbientTemperature,
			EnvSensor(EnvSensor::kAmbientTemperature, 22.5, 123.456)));
	sig->sensorData().setEnvSensors(envSensors);
	sig->sensorData().setCameraModels({CameraModel(525.0, 525.0, 320.0, 240.0)});
	sig->sensorData().setGlobalDescriptors({GlobalDescriptor(0,
			(cv::Mat_<float>(1, 4) << 1.f, 2.f, 3.f, 4.f))});

	Signature expected = *sig;
	saveSignature(sig);

	EXPECT_EQ(driver_->getTotalNodesSize(), 3);

	Signature * loaded = driver_->loadSignature(1);
	ASSERT_NE(loaded, nullptr);
	expectSignatureMembersEqual(expected, *loaded);

	// Cross-check node table fields via getNodeInfo.
	Transform loadedPose;
	Transform loadedGroundTruth;
	int mapId = 0;
	int weight = 0;
	std::string label;
	double stamp = 0.0;
	std::vector<float> velocity;
	GPS gps;
	EnvSensors sensors;
	ASSERT_TRUE(driver_->getNodeInfo(1, loadedPose, mapId, weight, label, stamp,
			loadedGroundTruth, velocity, gps, sensors));
	EXPECT_EQ(mapId, expected.mapId());
	EXPECT_EQ(weight, expected.getWeight());
	EXPECT_EQ(label, expected.getLabel());
	EXPECT_DOUBLE_EQ(stamp, expected.getStamp());
	expectTransformNear(expected.getPose(), loadedPose);
	expectTransformNear(expected.getGroundTruthPose(), loadedGroundTruth);
	expectVelocityEqual(expected.getVelocity(), velocity);
	EXPECT_DOUBLE_EQ(gps.stamp(), expected.sensorData().gps().stamp());
	EXPECT_DOUBLE_EQ(gps.longitude(), expected.sensorData().gps().longitude());
	EXPECT_DOUBLE_EQ(gps.latitude(), expected.sensorData().gps().latitude());
	EXPECT_EQ(sensors.size(), expected.sensorData().envSensors().size());

	// Heavy SensorData payloads are not loaded by loadSignatures().
	EXPECT_TRUE(loaded->sensorData().imageCompressed().empty());
	EXPECT_TRUE(loaded->sensorData().depthOrRightCompressed().empty());
	EXPECT_TRUE(loaded->sensorData().laserScanCompressed().isEmpty());
	EXPECT_TRUE(loaded->sensorData().userDataCompressed().empty());
	EXPECT_TRUE(loaded->sensorData().gridGroundCellsCompressed().empty());

	delete loaded;
}

TEST_F(DbDriverFixture, SaveAndLoadNodeDataAllFields)
{
	Signature * sig = new Signature(1);
	attachSensorDataForDatabaseSave(*sig);
	Signature expected(1);
	attachSensorDataForDatabaseSave(expected);
	saveSignature(sig);

	Signature * loaded = driver_->loadSignature(1);
	ASSERT_NE(loaded, nullptr);

	// loadSignatures() does not load Data table payloads.
	EXPECT_TRUE(loaded->sensorData().imageCompressed().empty());
	EXPECT_TRUE(loaded->sensorData().laserScanCompressed().isEmpty());
	EXPECT_TRUE(loaded->sensorData().userDataCompressed().empty());
	EXPECT_TRUE(loaded->sensorData().gridGroundCellsCompressed().empty());

	driver_->loadNodeData(*loaded, true, true, true, true);
	EXPECT_FALSE(loaded->sensorData().imageCompressed().empty());
	EXPECT_FALSE(loaded->sensorData().laserScanCompressed().isEmpty());
	EXPECT_FALSE(loaded->sensorData().userDataCompressed().empty());
	EXPECT_FALSE(loaded->sensorData().gridGroundCellsCompressed().empty());

	loaded->sensorData().uncompressData();
	expected.sensorData().uncompressData();
	expectSensorDataFromLoadNodeDataEqual(expected.sensorData(), loaded->sensorData());

	delete loaded;
}

TEST_F(DbDriverFixture, SaveAndLoadSignaturesWithFeatures)
{
	Signature * sig1 = new Signature(1);
	Signature * sig2 = new Signature(2);

	cv::Mat descriptors1 = (cv::Mat_<float>(2, 4) << 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f);
	cv::Mat descriptors2 = (cv::Mat_<float>(1, 4) << 10.f, 11.f, 12.f, 13.f);
	setSignatureFeatures(*sig1,
			{100, 101},
			{cv::KeyPoint(1.f, 2.f, 4.f), cv::KeyPoint(3.f, 4.f, 4.f)},
			{cv::Point3f(0.f, 0.f, 1.f), cv::Point3f(1.f, 0.f, 1.f)},
			descriptors1);
	setSignatureFeatures(*sig2,
			{200},
			{cv::KeyPoint(5.f, 6.f, 4.f)},
			{cv::Point3f(2.f, 0.f, 1.f)},
			descriptors2);

	saveSignature(sig1);
	saveSignature(sig2);

	std::list<int> ids;
	ids.push_back(1);
	ids.push_back(2);
	std::list<Signature *> loadedList;
	driver_->loadSignatures(ids, loadedList);
	ASSERT_EQ(loadedList.size(), 2u);

	std::map<int, Signature *> loadedById;
	for(Signature * s : loadedList)
	{
		loadedById[s->id()] = s;
	}

	Signature expected1(1);
	setSignatureFeatures(expected1, {100, 101},
			{cv::KeyPoint(1.f, 2.f, 4.f), cv::KeyPoint(3.f, 4.f, 4.f)},
			{cv::Point3f(0.f, 0.f, 1.f), cv::Point3f(1.f, 0.f, 1.f)},
			descriptors1);
	expectSignatureFeaturesEqual(expected1, *loadedById.at(1));

	Signature expected2(2);
	setSignatureFeatures(expected2, {200},
			{cv::KeyPoint(5.f, 6.f, 4.f)},
			{cv::Point3f(2.f, 0.f, 1.f)},
			descriptors2);
	expectSignatureFeaturesEqual(expected2, *loadedById.at(2));

	for(Signature * s : loadedList)
	{
		delete s;
	}
}

TEST_F(DbDriverFixture, AddLinkAndLoadLinks)
{
	saveSignature(new Signature(1));
	saveSignature(new Signature(2));

	Link link(1, 2, Link::kNeighbor, Transform(0.5f, 0.f, 0.f, 0.f, 0.f, 0.f));
	driver_->addLink(link);

	std::multimap<int, Link> links;
	driver_->loadLinks(1, links);
	ASSERT_EQ(links.size(), 1u);
	EXPECT_EQ(links.begin()->second.to(), 2);
	EXPECT_EQ(links.begin()->second.type(), Link::kNeighbor);
	EXPECT_FLOAT_EQ(links.begin()->second.transform().x(), 0.5f);
}

TEST_F(DbDriverFixture, SaveAndLoadOptimizedPoses)
{
	std::map<int, Transform> poses;
	poses[1] = Transform(1.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	Transform lastPose(2.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	driver_->saveOptimizedPoses(poses, lastPose);

	Transform loadedLast;
	std::map<int, Transform> loaded = driver_->loadOptimizedPoses(&loadedLast);
	ASSERT_EQ(loaded.size(), 1u);
	EXPECT_TRUE(loaded.count(1));
	EXPECT_FLOAT_EQ(loaded.at(1).x(), 1.f);
	EXPECT_FLOAT_EQ(loadedLast.x(), 2.f);
}

TEST_F(DbDriverFixture, SaveAndLoad2DMap)
{
	cv::Mat map(8, 8, CV_8S, cv::Scalar(0));
	map.at<int8_t>(2, 3) = 100;
	const float xMin = -4.f;
	const float yMin = -3.f;
	const float cellSize = 0.05f;
	driver_->save2DMap(map, xMin, yMin, cellSize);

	float loadedXMin = 0.f;
	float loadedYMin = 0.f;
	float loadedCellSize = 0.f;
	cv::Mat loaded = driver_->load2DMap(loadedXMin, loadedYMin, loadedCellSize);
	ASSERT_FALSE(loaded.empty());
	EXPECT_EQ(loaded.rows, map.rows);
	EXPECT_EQ(loaded.cols, map.cols);
	EXPECT_FLOAT_EQ(loadedXMin, xMin);
	EXPECT_FLOAT_EQ(loadedYMin, yMin);
	EXPECT_FLOAT_EQ(loadedCellSize, cellSize);
	EXPECT_EQ(loaded.at<int8_t>(2, 3), 100);
}

TEST_F(DbDriverFixture, GetAllNodeIdsAfterSave)
{
	saveSignature(new Signature(10));
	saveSignature(new Signature(20));

	std::set<int> ids;
	driver_->getAllNodeIds(ids);
	EXPECT_EQ(ids.size(), 2u);
	EXPECT_TRUE(ids.count(10));
	EXPECT_TRUE(ids.count(20));
}

TEST_F(DbDriverFixture, SaveAndLoadVisualWordFloatDescriptor)
{
	cv::Mat descriptor = (cv::Mat_<float>(1, 8) <<
		1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f);
	saveVisualWord(new VisualWord(42, descriptor));

	EXPECT_EQ(driver_->getTotalDictionarySize(), 1);

	int lastWordId = 0;
	driver_->getLastWordId(lastWordId);
	EXPECT_EQ(lastWordId, 42);

	std::set<int> wordIds;
	wordIds.insert(42);
	std::list<VisualWord *> loaded;
	driver_->loadWords(wordIds, loaded);
	ASSERT_EQ(loaded.size(), 1u);
	EXPECT_EQ(loaded.front()->id(), 42);
	EXPECT_EQ(loaded.front()->getDescriptor().cols, 8);
	EXPECT_EQ(loaded.front()->getDescriptor().type(), CV_32F);
	EXPECT_EQ(cv::norm(descriptor, loaded.front()->getDescriptor(), cv::NORM_INF), 0);
	EXPECT_TRUE(loaded.front()->isSaved());
	freeVisualWords(loaded);
}

TEST_F(DbDriverFixture, SaveAndLoadMultipleVisualWords)
{
	cv::Mat d1 = (cv::Mat_<float>(1, 4) << 0.f, 1.f, 2.f, 3.f);
	cv::Mat d2 = (cv::Mat_<float>(1, 4) << 10.f, 11.f, 12.f, 13.f);
	saveVisualWord(new VisualWord(1, d1));
	saveVisualWord(new VisualWord(2, d2));

	EXPECT_EQ(driver_->getTotalDictionarySize(), 2);

	std::set<int> wordIds;
	wordIds.insert(1);
	wordIds.insert(2);
	std::list<VisualWord *> loaded;
	driver_->loadWords(wordIds, loaded);
	ASSERT_EQ(loaded.size(), 2u);

	std::map<int, const VisualWord *> byId;
	for(VisualWord * w : loaded)
	{
		byId[w->id()] = w;
	}
	ASSERT_TRUE(byId.count(1));
	ASSERT_TRUE(byId.count(2));
	EXPECT_EQ(cv::norm(d1, byId.at(1)->getDescriptor(), cv::NORM_INF), 0);
	EXPECT_EQ(cv::norm(d2, byId.at(2)->getDescriptor(), cv::NORM_INF), 0);
	freeVisualWords(loaded);
}

TEST_F(DbDriverFixture, SaveAndLoadVisualWordBinaryDescriptor)
{
	cv::Mat descriptor(1, 32, CV_8U);
	for(int i = 0; i < 32; ++i)
	{
		descriptor.at<unsigned char>(0, i) = static_cast<unsigned char>(i * 7);
	}
	saveVisualWord(new VisualWord(7, descriptor));

	std::set<int> wordIds;
	wordIds.insert(7);
	std::list<VisualWord *> loaded;
	driver_->loadWords(wordIds, loaded);
	ASSERT_EQ(loaded.size(), 1u);
	EXPECT_EQ(loaded.front()->getDescriptor().type(), CV_8U);
	EXPECT_EQ(loaded.front()->getDescriptor().cols, 32);
	EXPECT_EQ(cv::countNonZero(descriptor != loaded.front()->getDescriptor()), 0);
	freeVisualWords(loaded);
}

TEST_F(DbDriverFixture, LoadWordsMissingIdReturnsEmpty)
{
	std::set<int> wordIds;
	wordIds.insert(9999);
	std::list<VisualWord *> loaded;
	driver_->loadWords(wordIds, loaded);
	EXPECT_TRUE(loaded.empty());
}

TEST_F(DbDriverFixture, UpdateLinkRemoveLinkRoundTrip)
{
	saveSignature(new Signature(1));
	saveSignature(new Signature(2));

	const Link initial(1, 2, Link::kNeighbor, Transform(0.5f, 0.f, 0.f, 0.f, 0.f, 0.f));
	driver_->addLink(initial);

	Link updated(1, 2, Link::kNeighbor, Transform(1.5f, 0.2f, 0.f, 0.f, 0.f, 0.f));
	driver_->updateLink(updated);

	std::multimap<int, Link> links;
	driver_->loadLinks(1, links);
	ASSERT_EQ(links.size(), 1u);
	expectLinkEqual(updated, links.begin()->second);

	driver_->removeLink(1, 2);
	driver_->loadLinks(1, links);
	EXPECT_TRUE(links.empty());
}

TEST_F(DbDriverFixture, UpdateNodeDataDirectApisRoundTrip)
{
	Signature * sig = new Signature(1);
	attachSensorDataForDatabaseSave(*sig);
	saveSignature(sig);

	const cv::Mat newDepth(16, 16, CV_16UC1, cv::Scalar(2000));
	driver_->updateDepthImage(1, newDepth, ".png");

	cv::Mat scanMat(1, 2, CV_32FC2);
	scanMat.at<cv::Vec2f>(0, 0) = cv::Vec2f(9.f, 0.f);
	scanMat.at<cv::Vec2f>(0, 1) = cv::Vec2f(10.f, 0.f);
	const LaserScan newScanRaw = LaserScan::backwardCompatibility(scanMat, 2, 10.f);
	driver_->updateLaserScan(1, newScanRaw);

	cv::Mat ground(1, 1, CV_32FC2);
	ground.at<cv::Vec2f>(0, 0) = cv::Vec2f(99.f, 0.f);
	cv::Mat obstacles(1, 1, CV_32FC2);
	obstacles.at<cv::Vec2f>(0, 0) = cv::Vec2f(100.f, 0.f);
	cv::Mat empty(1, 1, CV_32FC2);
	empty.at<cv::Vec2f>(0, 0) = cv::Vec2f(101.f, 0.f);
	driver_->updateOccupancyGrid(1, ground, obstacles, empty, 0.1f, cv::Point3f(4.f, 5.f, 6.f));

	const std::vector<CameraModel> models = {CameraModel(600.0, 600.0, 320.0, 240.0)};
	driver_->updateCalibration(1, models, {});

	std::vector<CameraModel> loadedModels;
	std::vector<StereoCameraModel> loadedStereo;
	ASSERT_TRUE(driver_->getCalibration(1, loadedModels, loadedStereo));
	ASSERT_EQ(loadedModels.size(), 1u);
	EXPECT_DOUBLE_EQ(loadedModels[0].fx(), 600.0);

	LaserScan scanInfo;
	ASSERT_TRUE(driver_->getLaserScanInfo(1, scanInfo));
	EXPECT_NEAR(scanInfo.rangeMax(), newScanRaw.rangeMax(), 1e-3f);
	EXPECT_EQ(scanInfo.format(), newScanRaw.format());

	Signature * loaded = driver_->loadSignature(1);
	ASSERT_NE(loaded, nullptr);
	driver_->loadNodeData(*loaded, true, true, false, true);
	loaded->sensorData().uncompressData();
	ASSERT_FALSE(loaded->sensorData().depthRaw().empty());
	EXPECT_EQ(loaded->sensorData().depthRaw().at<uint16_t>(0, 0), 2000);
	expectLaserScanNear(newScanRaw, loaded->sensorData().laserScanRaw());
	EXPECT_FLOAT_EQ(loaded->sensorData().gridCellSize(), 0.1f);
	EXPECT_FLOAT_EQ(loaded->sensorData().gridViewPoint().x, 4.f);
	EXPECT_EQ(loaded->sensorData().gridGroundCellsRaw().cols, 1);
	EXPECT_FLOAT_EQ(loaded->sensorData().gridGroundCellsRaw().at<cv::Vec2f>(0, 0)[0], 99.f);
	delete loaded;
}

TEST_F(DbDriverFixture, SaveAndLoadStatistics)
{
	saveSignature(new Signature(1));

	Statistics stats;
	stats.setRefImageId(1);
	stats.setStamp(123.456);
	stats.addStatistic(Statistics::kTimingMemory_update(), 42.5f);
	stats.setWmState({1, 2, 3});
	driver_->addStatistics(stats, true);

	double stamp = 0.0;
	std::vector<int> wmState;
	std::map<std::string, float> loaded = driver_->getStatistics(1, stamp, &wmState);
	ASSERT_FALSE(loaded.empty());
	EXPECT_NEAR(loaded.at(Statistics::kTimingMemory_update()), 42.5f, 1e-5f);
	EXPECT_DOUBLE_EQ(stamp, 123.456);
	ASSERT_EQ(wmState.size(), 3u);
	EXPECT_EQ(wmState[0], 1);
	EXPECT_EQ(wmState[1], 2);
	EXPECT_EQ(wmState[2], 3);

	std::map<int, std::pair<std::map<std::string, float>, double> > allStats = driver_->getAllStatistics();
	ASSERT_TRUE(allStats.count(1));
	EXPECT_NEAR(allStats.at(1).first.at(Statistics::kTimingMemory_update()), 42.5f, 1e-5f);

	std::map<int, std::vector<int> > allWm = driver_->getAllStatisticsWmStates();
	ASSERT_TRUE(allWm.count(1));
	EXPECT_EQ(allWm.at(1).size(), 3u);
}

TEST_F(DbDriverFixture, AddInfoAfterRunRestoresSessionInfo)
{
	saveSignature(new Signature(1));
	saveSignature(new Signature(2));
	std::this_thread::sleep_for(std::chrono::seconds(1));

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kRGBDLinearUpdate(), "0.2"));
	driver_->addInfoAfterRun(2, 2, 100, 200, 5, params);

	ParametersMap loadedParams = driver_->getLastParameters();
	ASSERT_FALSE(loadedParams.empty());
	EXPECT_FLOAT_EQ(std::atof(loadedParams.at(Parameters::kRGBDLinearUpdate()).c_str()), 0.2f);

	saveSignature(new Signature(3));
	EXPECT_EQ(driver_->getLastNodesSize(), 1);

	std::this_thread::sleep_for(std::chrono::seconds(1));
	saveVisualWord(new VisualWord(50, (cv::Mat_<float>(1, 4) << 1.f, 2.f, 3.f, 4.f)));
	EXPECT_EQ(driver_->getLastDictionarySize(), 1);
}

TEST_F(DbDriverFixture, SaveAndLoadPreviewImage)
{
	cv::Mat image(32, 32, CV_8UC1, cv::Scalar(77));
	driver_->savePreviewImage(image);

	cv::Mat loaded = driver_->loadPreviewImage();
	ASSERT_FALSE(loaded.empty());
	EXPECT_EQ(loaded.rows, image.rows);
	EXPECT_EQ(loaded.cols, image.cols);
	EXPECT_EQ(loaded.at<unsigned char>(0, 0), 77);
}

TEST_F(DbDriverFixture, LabelAndGraphQueries)
{
	const Transform pose(1.f, 2.f, 3.f, 0.f, 0.f, 0.f);
	saveSignature(new Signature(1, 5, 7, 100.0, "room_a", pose));
	saveSignature(new Signature(2, 5, 3, 101.0, "room_b", Transform(2.f, 0.f, 0.f, 0.f, 0.f, 0.f)));
	saveSignature(new Signature(3));

	driver_->addLink(Link(1, 2, Link::kNeighbor, Transform(0.5f, 0.f, 0.f, 0.f, 0.f, 0.f)));
	driver_->addLink(Link(1, 3, Link::kGlobalClosure, Transform(0.1f, 0.2f, 0.f, 0.f, 0.f, 0.5f)));
	driver_->addLink(Link(1, -10, Link::kLandmark, Transform(0.2f, -0.3f, 0.4f, 0.f, 0.f, 0.f)));

	int idByLabel = 0;
	driver_->getNodeIdByLabel("room_a", idByLabel);
	EXPECT_EQ(idByLabel, 1);

	std::map<int, std::string> labels;
	driver_->getAllLabels(labels);
	ASSERT_TRUE(labels.count(1));
	EXPECT_EQ(labels.at(1), "room_a");

	int weight = 0;
	driver_->getWeight(1, weight);
	EXPECT_EQ(weight, 7);

	int lastNodeId = 0;
	int lastMapId = 0;
	driver_->getLastNodeId(lastNodeId);
	driver_->getLastMapId(lastMapId);
	EXPECT_EQ(lastNodeId, 3);
	EXPECT_EQ(lastMapId, 5);

	std::map<int, Transform> odomPoses;
	driver_->getAllOdomPoses(odomPoses);
	ASSERT_EQ(odomPoses.size(), 3u);
	expectTransformNear(pose, odomPoses.at(1));

	std::multimap<int, Link> allLinks;
	driver_->getAllLinks(allLinks, true, true);
	EXPECT_GE(allLinks.size(), 2u);

	std::multimap<int, Link> neighborLinks;
	driver_->loadLinks(1, neighborLinks, Link::kNeighbor);
	ASSERT_EQ(neighborLinks.size(), 1u);
	EXPECT_EQ(neighborLinks.begin()->second.type(), Link::kNeighbor);
	EXPECT_EQ(neighborLinks.begin()->second.to(), 2);

	driver_->addInfoAfterRun(0, 0, 0, 0, 0, ParametersMap());
	std::this_thread::sleep_for(std::chrono::seconds(1));
	saveSignature(new Signature(4, 5, 1, 102.0, "wm_node", Transform(4.f, 0.f, 0.f, 0.f, 0.f, 0.f)));

	std::set<int> lastNodeIds;
	driver_->getLastNodeIds(lastNodeIds);
	EXPECT_TRUE(lastNodeIds.count(4));
}

TEST_F(DbDriverFixture, GetNodeDataAndLocalFeatures)
{
	Signature * sig = new Signature(1);
	attachSensorDataForDatabaseSave(*sig);
	const cv::Mat descriptors = (cv::Mat_<float>(1, 4) << 1.f, 2.f, 3.f, 4.f);
	setSignatureFeatures(*sig, {10}, {cv::KeyPoint(5.f, 6.f, 4.f)}, {cv::Point3f(0.f, 0.f, 1.f)}, descriptors);
	saveSignature(sig);

	SensorData data;
	driver_->getNodeData(1, data, true, true, true, true);
	data.uncompressData();
	Signature expected(1);
	attachSensorDataForDatabaseSave(expected);
	expected.sensorData().uncompressData();
	expectSensorDataFromLoadNodeDataEqual(expected.sensorData(), data);

	std::multimap<int, int> words;
	std::vector<cv::KeyPoint> keypoints;
	std::vector<cv::Point3f> points;
	cv::Mat loadedDescriptors;
	driver_->getLocalFeatures(1, words, keypoints, points, loadedDescriptors);
	ASSERT_EQ(words.size(), 1u);
	EXPECT_EQ(words.begin()->first, 10);
	ASSERT_EQ(keypoints.size(), 1u);
	EXPECT_FLOAT_EQ(keypoints[0].pt.x, 5.f);
	ASSERT_EQ(loadedDescriptors.rows, 1);
	EXPECT_EQ(cv::norm(descriptors, loadedDescriptors, cv::NORM_INF), 0);
}

TEST_F(DbDriverFixture, LoadVWDictionaryFromDatabase)
{
	cv::Mat d1 = (cv::Mat_<float>(1, 4) << 0.f, 1.f, 2.f, 3.f);
	cv::Mat d2 = (cv::Mat_<float>(1, 4) << 4.f, 5.f, 6.f, 7.f);
	saveVisualWord(new VisualWord(1, d1));
	saveVisualWord(new VisualWord(2, d2));

	VWDictionary dictionary;
	driver_->load(dictionary, false);
	EXPECT_EQ(dictionary.getVisualWords().size(), 2u);
	EXPECT_TRUE(dictionary.getVisualWords().count(1));
	EXPECT_TRUE(dictionary.getVisualWords().count(2));
	EXPECT_EQ(cv::norm(d1, dictionary.getVisualWords().at(1)->getDescriptor(), cv::NORM_INF), 0);
}

TEST_F(DbDriverFixture, UpdateSignatureRoundTrip)
{
	Signature * sig = new Signature(1, 5, 7, 100.0, "before", Transform(0.f, 0.f, 0.f, 0.f, 0.f, 0.f));
	saveSignature(sig);

	Signature * loaded = driver_->loadSignature(1);
	ASSERT_NE(loaded, nullptr);
	loaded->setWeight(99);
	loaded->setLabel("after");
	driver_->asyncSave(loaded);
	driver_->emptyTrashes(false);

	Signature * reloaded = driver_->loadSignature(1);
	ASSERT_NE(reloaded, nullptr);
	EXPECT_EQ(reloaded->getWeight(), 99);
	EXPECT_EQ(reloaded->getLabel(), "after");
	delete reloaded;
}

TEST_F(DbDriverFixture, GetNodesObservingLandmark)
{
	saveSignature(new Signature(1));
	driver_->addLink(Link(1, -10, Link::kLandmark, Transform(0.2f, -0.3f, 0.4f, 0.f, 0.f, 0.f)));

	std::map<int, Link> nodes;
	driver_->getNodesObservingLandmark(-10, nodes);
	ASSERT_EQ(nodes.size(), 1u);
	EXPECT_EQ(nodes.begin()->first, 1);
	EXPECT_EQ(nodes.begin()->second.type(), Link::kLandmark);
	EXPECT_EQ(nodes.begin()->second.to(), -10);
}

TEST_F(DbDriverFixture, LoadSignatureFromTrashBeforeFlush)
{
	Signature * sig = new Signature(99);
	driver_->asyncSave(sig);

	bool loadedFromTrash = false;
	Signature * loaded = driver_->loadSignature(99, &loadedFromTrash);
	ASSERT_NE(loaded, nullptr);
	EXPECT_TRUE(loadedFromTrash);
	EXPECT_EQ(loaded->id(), 99);

	// loadSignature() moves the object out of the trash; persist it explicitly.
	saveSignature(loaded);
	EXPECT_EQ(driver_->getTotalNodesSize(), 1);
}

TEST_F(DbDriverFixture, SaveAndLoadOptimizedMesh)
{
	cv::Mat cloud(1, 9, CV_32FC1);
	for(int i = 0; i < 9; ++i)
	{
		cloud.at<float>(0, i) = static_cast<float>(i + 1);
	}
	driver_->saveOptimizedMesh(cloud);

	cv::Mat loaded = driver_->loadOptimizedMesh();
	ASSERT_FALSE(loaded.empty());
	EXPECT_EQ(loaded.rows, cloud.rows);
	EXPECT_EQ(loaded.cols, cloud.cols);
	EXPECT_EQ(cv::norm(cloud, loaded, cv::NORM_INF), 0);
}

TEST_F(DbDriverFixture, SaveFlannIndexDoesNotFail)
{
	if(uStrNumCmp(driver_->getDatabaseVersion(), "0.23.0") >= 0)
	{
		std::vector<unsigned char> indexData = {1, 2, 3, 4, 5};
		EXPECT_NO_THROW(driver_->saveFlannIndex(indexData));
		EXPECT_NO_THROW(driver_->saveFlannIndex(std::vector<unsigned char>()));
	}
}

TEST_F(DbDriverFixture, MemoryUsageCountersAfterSave)
{
	Signature * sig = new Signature(1);
	attachSensorDataForDatabaseSave(*sig);
	const cv::Mat descriptors = (cv::Mat_<float>(1, 4) << 1.f, 2.f, 3.f, 4.f);
	setSignatureFeatures(*sig, {10}, {cv::KeyPoint(5.f, 6.f, 4.f)}, {cv::Point3f(0.f, 0.f, 1.f)}, descriptors);
	saveSignature(sig);
	saveVisualWord(new VisualWord(1, descriptors));

	EXPECT_GT(driver_->getMemoryUsed(), 0ul);
	EXPECT_GT(driver_->getNodesMemoryUsed(), 0L);
	EXPECT_GT(driver_->getImagesMemoryUsed(), 0L);
	EXPECT_GT(driver_->getDepthImagesMemoryUsed(), 0L);
	EXPECT_GT(driver_->getCalibrationsMemoryUsed(), 0L);
	EXPECT_GT(driver_->getGridsMemoryUsed(), 0L);
	EXPECT_GT(driver_->getLaserScansMemoryUsed(), 0L);
	EXPECT_GT(driver_->getUserDataMemoryUsed(), 0L);
	EXPECT_GT(driver_->getWordsMemoryUsed(), 0L);
	EXPECT_GT(driver_->getFeaturesMemoryUsed(), 0L);

	Statistics stats;
	stats.setRefImageId(1);
	stats.addStatistic(Statistics::kTimingMemory_update(), 1.f);
	driver_->addStatistics(stats, false);
	EXPECT_GT(driver_->getStatisticsMemoryUsed(), 0L);

	int ni = 0;
	driver_->getInvertedIndexNi(1, ni);
	EXPECT_EQ(ni, 1);
}

TEST_F(DbDriverFixture, LoadLastNodesAfterInfoBoundary)
{
	saveSignature(new Signature(1));
	saveSignature(new Signature(2));
	std::this_thread::sleep_for(std::chrono::seconds(1));
	driver_->addInfoAfterRun(2, 2, 0, 0, 0, ParametersMap());
	saveSignature(new Signature(3, 5, 1, 200.0, "latest", Transform(3.f, 0.f, 0.f, 0.f, 0.f, 0.f)));

	EXPECT_EQ(driver_->getLastNodesSize(), 1);

	std::list<Signature *> lastNodes;
	driver_->loadLastNodes(lastNodes);
	ASSERT_EQ(lastNodes.size(), 1u);
	EXPECT_EQ(lastNodes.front()->id(), 3);
	EXPECT_EQ(lastNodes.front()->getLabel(), "latest");
	for(Signature * s : lastNodes)
	{
		delete s;
	}
}

TEST_F(DbDriverFixture, ReopenDatabasePreservesData)
{
	const Transform pose(1.f, 2.f, 3.f, 0.f, 0.f, 0.f);
	saveSignature(new Signature(1, 5, 7, 100.0, "persisted", pose));

	const std::string path = dbPath_;
	driver_->closeConnection(true);
	delete driver_;
	driver_ = nullptr;

	driver_ = DBDriver::create();
	ASSERT_NE(driver_, nullptr);
	ASSERT_TRUE(driver_->openConnection(path));
	ASSERT_TRUE(driver_->isConnected());

	Signature * loaded = driver_->loadSignature(1);
	ASSERT_NE(loaded, nullptr);
	EXPECT_EQ(loaded->getLabel(), "persisted");
	EXPECT_EQ(loaded->getWeight(), 7);
	expectTransformNear(pose, loaded->getPose());
	delete loaded;
}

TEST(DBDriverTest, InMemoryDatabaseSaveToFileOnClose)
{
	const std::string path = uniqueDbPath();
	DBDriver * driver = DBDriver::create();
	ASSERT_NE(driver, nullptr);
	ASSERT_TRUE(driver->openConnection(""));
	EXPECT_TRUE(driver->isInMemory());

	driver->asyncSave(new Signature(1, 5, 1, 50.0, "mem_to_disk", Transform(1.f, 0.f, 0.f, 0.f, 0.f, 0.f)));
	driver->emptyTrashes(false);
	driver->closeConnection(true, path);
	delete driver;

	DBDriver * driver2 = DBDriver::create();
	ASSERT_TRUE(driver2->openConnection(path));
	EXPECT_FALSE(driver2->isInMemory());

	Signature * loaded = driver2->loadSignature(1);
	ASSERT_NE(loaded, nullptr);
	EXPECT_EQ(loaded->getLabel(), "mem_to_disk");
	delete loaded;

	driver2->closeConnection(false);
	delete driver2;
	UFile::erase(path.c_str());
}

TEST_F(DbDriverFixture, GenerateGraphWritesDotFile)
{
	saveSignature(new Signature(1, 5, 7, 100.0, "n1", Transform(0.f, 0.f, 0.f, 0.f, 0.f, 0.f)));
	saveSignature(new Signature(2, 5, 3, 101.0, "n2", Transform(1.f, 0.f, 0.f, 0.f, 0.f, 0.f)));
	driver_->addLink(Link(1, 2, Link::kNeighbor, Transform(0.5f, 0.f, 0.f, 0.f, 0.f, 0.f)));

	const std::string dotPath = dbPath_ + ".dot";
	driver_->generateGraph(dotPath);

	ASSERT_TRUE(UFile::exists(dotPath.c_str()));
	std::ifstream in(dotPath.c_str());
	ASSERT_TRUE(in.good());
	std::stringstream buffer;
	buffer << in.rdbuf();
	const std::string content = buffer.str();
	EXPECT_NE(content.find("digraph G"), std::string::npos);
	EXPECT_NE(content.find(" -> "), std::string::npos);
	UFile::erase(dotPath.c_str());
}

TEST_F(DbDriverFixture, GenerateGraphWithFilteredIds)
{
	saveSignature(new Signature(1));
	saveSignature(new Signature(2));
	saveSignature(new Signature(3));
	driver_->addLink(Link(1, 2, Link::kNeighbor, Transform(0.5f, 0.f, 0.f, 0.f, 0.f, 0.f)));

	const std::string dotPath = dbPath_ + "_filtered.dot";
	std::set<int> ids;
	ids.insert(1);
	ids.insert(2);
	driver_->generateGraph(dotPath, ids);

	std::ifstream in(dotPath.c_str());
	ASSERT_TRUE(in.good());
	std::stringstream buffer;
	buffer << in.rdbuf();
	const std::string content = buffer.str();
	EXPECT_NE(content.find("digraph G"), std::string::npos);
	EXPECT_NE(content.find("\"1\\n"), std::string::npos);
	EXPECT_NE(content.find("\"2\\n"), std::string::npos);
	EXPECT_EQ(content.find("\"3\\n"), std::string::npos);
	UFile::erase(dotPath.c_str());
}

TEST_F(DbDriverFixture, ExecuteNoResultRunsWithoutError)
{
	EXPECT_NO_THROW(driver_->executeNoResult("PRAGMA cache_size=2000;"));
	EXPECT_NO_THROW(driver_->executeNoResult("PRAGMA synchronous=NORMAL;"));
	EXPECT_TRUE(driver_->isConnected());
}
