#include <gtest/gtest.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/DBReader.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/SensorCaptureInfo.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include "TestUtils.h"
#include <opencv2/core.hpp>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace rtabmap;

namespace {

std::string uniqueDbPath()
{
	static int counter = 0;
	return test::tempPath(uFormat("rtabmap_dbreader_test_%d_%d.db", test::getPid(), ++counter));
}

const int kImageWidth = 8;
const int kImageHeight = 6;

CameraModel makeCameraModel(const Transform & localTransform = CameraModel::opticalRotation())
{
	return CameraModel(10.0, 10.0, kImageWidth/2.0, kImageHeight/2.0, localTransform, 0.0, cv::Size(kImageWidth, kImageHeight));
}

// Compressed RGB image (only compressed buffers are saved in the database) with
// a calibrated camera model.
SensorData makeImageData(int id, double stamp, unsigned char gray = 128)
{
	SensorData data(compressImage2(cv::Mat(kImageHeight, kImageWidth, CV_8UC3, cv::Scalar::all(gray)), ".png"), makeCameraModel(), id, stamp);
	return data;
}

// Writes nodes to a new database, then closes it so that DBReader can read it.
class DatabaseWriter
{
public:
	explicit DatabaseWriter(const std::string & path) :
		driver_(DBDriver::create())
	{
		EXPECT_TRUE(driver_->openConnection(path, true));
	}
	~DatabaseWriter()
	{
		close();
	}
	// Node with an image, at x=id along the x axis.
	Signature * addNode(int id, int mapId = 0, int weight = 0, const SensorData & data = SensorData())
	{
		SensorData nodeData = data.id() == 0 && data.imageCompressed().empty() ? makeImageData(id, double(id)) : data;
		nodeData.setId(id);
		Signature * s = new Signature(id, mapId, weight, double(id), "", Transform(float(id), 0, 0, 0, 0, 0), Transform(), nodeData);
		nodes_.push_back(s);
		return s;
	}
	// Neighbor link to the previous node (as Memory saves them, in both directions).
	void linkNeighbors(Signature * from, Signature * to, double information = 100.0)
	{
		const Transform t = from->getPose().inverse() * to->getPose();
		from->addLink(Link(from->id(), to->id(), Link::kNeighbor, t, cv::Mat::eye(6, 6, CV_64FC1) * information));
		to->addLink(Link(to->id(), from->id(), Link::kNeighbor, t.inverse(), cv::Mat::eye(6, 6, CV_64FC1) * information));
	}
	void close()
	{
		if(driver_)
		{
			for(size_t i=0; i<nodes_.size(); ++i)
			{
				driver_->asyncSave(nodes_[i]);
			}
			nodes_.clear();
			driver_->emptyTrashes(false);
			driver_->closeConnection(true);
			driver_.reset();
		}
	}
private:
	std::unique_ptr<DBDriver> driver_;
	std::vector<Signature *> nodes_;
};

// DBReader's constructor options, with their defaults.
struct ReaderOptions
{
	bool odometryIgnored = false;
	bool ignoreGoalDelay = true; // don't sleep in tests
	bool goalsIgnored = false;
	int startId = 0;
	std::vector<unsigned int> cameraIndices;
	int stopId = 0;
	bool intermediateNodesIgnored = false;
	bool landmarksIgnored = false;
	bool featuresIgnored = false;
	int startMapId = 0;
	int stopMapId = -1;
	bool priorsIgnored = false;
	bool imuIgnored = false;
	bool intermediateNodesAreNormalNodes = false;
	std::vector<Transform> cameraLocalTransformOverrides;
	bool occupancyGridsIgnored = false;
};

std::unique_ptr<DBReader> makeReader(const std::string & paths, const ReaderOptions & o = ReaderOptions())
{
	return std::unique_ptr<DBReader>(new DBReader(paths, 0.0f, o.odometryIgnored, o.ignoreGoalDelay, o.goalsIgnored,
			o.startId, o.cameraIndices, o.stopId, o.intermediateNodesIgnored, o.landmarksIgnored, o.featuresIgnored,
			o.startMapId, o.stopMapId, o.priorsIgnored, o.imuIgnored, o.intermediateNodesAreNormalNodes,
			o.cameraLocalTransformOverrides, o.occupancyGridsIgnored));
}

struct Frame
{
	SensorData data;
	SensorCaptureInfo info;
};

// Reads all frames until the end of the database(s).
std::vector<Frame> readAll(DBReader & reader)
{
	std::vector<Frame> frames;
	for(int i=0; i<100; ++i)
	{
		Frame frame;
		frame.data = reader.takeData(&frame.info);
		if(frame.data.id() == 0)
		{
			break;
		}
		frames.push_back(frame);
	}
	return frames;
}

std::vector<int> frameIds(const std::vector<Frame> & frames)
{
	std::vector<int> ids;
	for(size_t i=0; i<frames.size(); ++i)
	{
		ids.push_back(frames[i].data.id());
	}
	return ids;
}

bool isApprox(const Transform & a, const Transform & b)
{
	return a.toEigen4f().isApprox(b.toEigen4f(), 1e-5f);
}

class DBReaderFixture : public ::testing::Test
{
protected:
	void SetUp() override
	{
		dbPath_ = uniqueDbPath();
	}
	void TearDown() override
	{
		UFile::erase(dbPath_);
	}
	std::string dbPath_;
};

} // namespace

TEST_F(DBReaderFixture, InitFailsOnMissingDatabase)
{
	std::unique_ptr<DBReader> reader = makeReader(dbPath_);
	EXPECT_FALSE(reader->init());
}

TEST_F(DBReaderFixture, ReplaysNodesInOrderWithTheirOdometry)
{
	{
		DatabaseWriter writer(dbPath_);
		Signature * previous = 0;
		for(int id=1; id<=3; ++id)
		{
			Signature * s = writer.addNode(id);
			if(previous)
			{
				writer.linkNeighbors(s, previous);
			}
			previous = s;
		}
	}

	std::unique_ptr<DBReader> reader = makeReader(dbPath_);
	ASSERT_TRUE(reader->init());
	EXPECT_TRUE(reader->isCalibrated());
	std::vector<Frame> frames = readAll(*reader);
	ASSERT_EQ(std::vector<int>({1, 2, 3}), frameIds(frames));
	for(size_t i=0; i<frames.size(); ++i)
	{
		const int id = frames[i].data.id();
		EXPECT_DOUBLE_EQ(double(id), frames[i].data.stamp());
		EXPECT_FALSE(frames[i].data.imageRaw().empty()) << "node " << id;
		ASSERT_EQ(1u, frames[i].data.cameraModels().size());
		EXPECT_TRUE(isApprox(Transform(float(id), 0, 0, 0, 0, 0), frames[i].info.odomPose)) << "node " << id;
	}
}

TEST_F(DBReaderFixture, OdometryCovarianceComesFromNeighborLinksAndIsHighOnNewMaps)
{
	// The odometry covariance is the one of the link to the previous node. The
	// first node of each map has no previous node: its covariance is set to 9999,
	// so that Rtabmap starts a new map (used by reprocessing and recovery).
	{
		DatabaseWriter writer(dbPath_);
		Signature * n1 = writer.addNode(1, 0);
		Signature * n2 = writer.addNode(2, 0);
		writer.linkNeighbors(n2, n1, 100.0); // covariance 0.01
		Signature * n3 = writer.addNode(3, 1);
		Signature * n4 = writer.addNode(4, 1);
		writer.linkNeighbors(n4, n3, 25.0); // covariance 0.04
	}

	std::unique_ptr<DBReader> reader = makeReader(dbPath_);
	ASSERT_TRUE(reader->init());
	std::vector<Frame> frames = readAll(*reader);
	ASSERT_EQ(4u, frames.size());
	const double expected[] = {9999.0, 0.01, 9999.0, 0.04};
	for(size_t i=0; i<frames.size(); ++i)
	{
		SCOPED_TRACE(uFormat("node %d", frames[i].data.id()));
		ASSERT_EQ(6, frames[i].info.odomCovariance.rows);
		EXPECT_NEAR(expected[i], frames[i].info.odomCovariance.at<double>(0, 0), expected[i]*1e-6);
	}
}

TEST_F(DBReaderFixture, OdometryIgnored)
{
	{
		DatabaseWriter writer(dbPath_);
		writer.addNode(1);
		writer.addNode(2);
	}
	ReaderOptions options;
	options.odometryIgnored = true;
	std::unique_ptr<DBReader> reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	std::vector<Frame> frames = readAll(*reader);
	ASSERT_EQ(2u, frames.size());
	for(size_t i=0; i<frames.size(); ++i)
	{
		EXPECT_TRUE(frames[i].info.odomPose.isNull());
	}
}

TEST_F(DBReaderFixture, StartAndStopIds)
{
	{
		DatabaseWriter writer(dbPath_);
		for(int id=1; id<=5; ++id)
		{
			writer.addNode(id);
		}
	}
	ReaderOptions options;
	options.startId = 2;
	options.stopId = 4;
	std::unique_ptr<DBReader> reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	EXPECT_EQ(std::vector<int>({2, 3, 4}), frameIds(readAll(*reader)));

	// A start id not in the database starts from the beginning.
	options.startId = 10;
	options.stopId = 0;
	reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	EXPECT_EQ(std::vector<int>({1, 2, 3, 4, 5}), frameIds(readAll(*reader)));
}

TEST_F(DBReaderFixture, StartAndStopMapIds)
{
	{
		DatabaseWriter writer(dbPath_);
		writer.addNode(1, 0);
		writer.addNode(2, 0);
		writer.addNode(3, 1);
		writer.addNode(4, 1);
		writer.addNode(5, 2);
	}
	ReaderOptions options;
	options.startMapId = 1;
	options.stopMapId = 1;
	std::unique_ptr<DBReader> reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	EXPECT_EQ(std::vector<int>({3, 4}), frameIds(readAll(*reader)));

	options.stopMapId = -1; // until the end
	reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	EXPECT_EQ(std::vector<int>({3, 4, 5}), frameIds(readAll(*reader)));
}

TEST_F(DBReaderFixture, IntermediateNodes)
{
	// Intermediate nodes (weight=-1) are published with id -1 by default, can be
	// skipped, or published as normal nodes with their id.
	{
		DatabaseWriter writer(dbPath_);
		writer.addNode(1);
		writer.addNode(2, 0, -1);
		writer.addNode(3);
	}
	std::unique_ptr<DBReader> reader = makeReader(dbPath_);
	ASSERT_TRUE(reader->init());
	EXPECT_EQ(std::vector<int>({1, -1, 3}), frameIds(readAll(*reader)));

	ReaderOptions options;
	options.intermediateNodesIgnored = true;
	reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	EXPECT_EQ(std::vector<int>({1, 3}), frameIds(readAll(*reader)));

	options.intermediateNodesIgnored = false;
	options.intermediateNodesAreNormalNodes = true;
	reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	EXPECT_EQ(std::vector<int>({1, 2, 3}), frameIds(readAll(*reader)));
}

TEST_F(DBReaderFixture, PriorsGravityAndLandmarksUnlessIgnored)
{
	const Transform prior(1.0f, 2.0f, 0.0f, 0, 0, 0.5f);
	const Transform gravity(0, 0, 0, 0.1f, 0.2f, 0);
	const Transform landmarkPose(0.5f, 0, 0, 0, 0, 0);
	const int kLandmarkId = 7;
	{
		DatabaseWriter writer(dbPath_);
		Signature * s = writer.addNode(1);
		s->addLink(Link(1, 1, Link::kPosePrior, prior, cv::Mat::eye(6, 6, CV_64FC1) * 100.0));
		s->addLink(Link(1, 1, Link::kGravity, gravity, cv::Mat::eye(6, 6, CV_64FC1)));
		s->addLink(Link(1, -kLandmarkId, Link::kLandmark, landmarkPose, cv::Mat::eye(6, 6, CV_64FC1)));
	}

	{
		std::unique_ptr<DBReader> reader = makeReader(dbPath_);
		ASSERT_TRUE(reader->init());
		std::vector<Frame> frames = readAll(*reader);
		ASSERT_EQ(1u, frames.size());
		const SensorData & data = frames[0].data;
		EXPECT_TRUE(isApprox(prior, data.globalPose()));
		ASSERT_FALSE(data.imu().empty());
		const Eigen::Quaterniond q = gravity.getQuaterniond();
		EXPECT_NEAR(q.x(), data.imu().orientation()[0], 1e-6);
		EXPECT_NEAR(q.y(), data.imu().orientation()[1], 1e-6);
		EXPECT_NEAR(q.z(), data.imu().orientation()[2], 1e-6);
		EXPECT_NEAR(q.w(), data.imu().orientation()[3], 1e-6);
		ASSERT_EQ(1u, data.landmarks().count(kLandmarkId));
		EXPECT_TRUE(isApprox(landmarkPose, data.landmarks().at(kLandmarkId).pose()));
	}

	ReaderOptions options;
	options.priorsIgnored = true;
	options.imuIgnored = true;
	options.landmarksIgnored = true;
	std::unique_ptr<DBReader> reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	std::vector<Frame> frames = readAll(*reader);
	ASSERT_EQ(1u, frames.size());
	EXPECT_TRUE(frames[0].data.globalPose().isNull());
	EXPECT_TRUE(frames[0].data.imu().empty());
	EXPECT_TRUE(frames[0].data.landmarks().empty());
}

TEST_F(DBReaderFixture, FeaturesUnlessIgnored)
{
	const int kFeatures = 3;
	{
		DatabaseWriter writer(dbPath_);
		Signature * s = writer.addNode(1);
		std::multimap<int, int> words;
		std::vector<cv::KeyPoint> keypoints;
		std::vector<cv::Point3f> points;
		cv::Mat descriptors = cv::Mat::zeros(kFeatures, 4, CV_32FC1);
		for(int i=0; i<kFeatures; ++i)
		{
			words.insert(std::make_pair(i+1, i));
			keypoints.push_back(cv::KeyPoint(float(i+1), 2.0f, 3.0f));
			points.push_back(cv::Point3f(1.0f, float(i), 0.0f));
			descriptors.at<float>(i, i) = 1.0f;
		}
		s->setWords(words, keypoints, points, descriptors);
	}

	std::unique_ptr<DBReader> reader = makeReader(dbPath_);
	ASSERT_TRUE(reader->init());
	std::vector<Frame> frames = readAll(*reader);
	ASSERT_EQ(1u, frames.size());
	EXPECT_EQ(kFeatures, (int)frames[0].data.keypoints().size());
	EXPECT_EQ(kFeatures, (int)frames[0].data.keypoints3D().size());
	EXPECT_EQ(kFeatures, frames[0].data.descriptors().rows);

	ReaderOptions options;
	options.featuresIgnored = true;
	reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	frames = readAll(*reader);
	ASSERT_EQ(1u, frames.size());
	EXPECT_TRUE(frames[0].data.keypoints().empty());
	EXPECT_TRUE(frames[0].data.descriptors().empty());
}

TEST_F(DBReaderFixture, GoalsAreRemovedFromUserDataUnlessIgnored)
{
	// A goal saved in the user data ("GOAL:id") is posted as a goal event and
	// removed from the published user data.
	{
		DatabaseWriter writer(dbPath_);
		SensorData data = makeImageData(1, 1.0);
		const std::string goal = "GOAL:5";
		cv::Mat userData(1, (int)goal.size()+1, CV_8SC1);
		memcpy(userData.data, goal.c_str(), goal.size()+1);
		data.setUserData(userData);
		writer.addNode(1, 0, 0, data);
	}

	std::unique_ptr<DBReader> reader = makeReader(dbPath_);
	ASSERT_TRUE(reader->init());
	std::vector<Frame> frames = readAll(*reader);
	ASSERT_EQ(1u, frames.size());
	EXPECT_TRUE(frames[0].data.userDataRaw().empty());

	ReaderOptions options;
	options.goalsIgnored = true;
	reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	frames = readAll(*reader);
	ASSERT_EQ(1u, frames.size());
	ASSERT_FALSE(frames[0].data.userDataRaw().empty());
	EXPECT_EQ(std::string("GOAL:5"), std::string((const char *)frames[0].data.userDataRaw().data));
}

TEST_F(DBReaderFixture, ReadsMultipleDatabasesInSequence)
{
	const std::string secondPath = uniqueDbPath();
	{
		DatabaseWriter writer(dbPath_);
		writer.addNode(1);
		writer.addNode(2);
	}
	{
		DatabaseWriter writer(secondPath);
		writer.addNode(1);
		writer.addNode(2);
		writer.addNode(3);
	}
	std::unique_ptr<DBReader> reader = makeReader(dbPath_ + ";" + secondPath);
	ASSERT_TRUE(reader->init());
	EXPECT_EQ(std::vector<int>({1, 2, 1, 2, 3}), frameIds(readAll(*reader)));
	reader.reset();
	UFile::erase(secondPath);
}

TEST_F(DBReaderFixture, CameraIndicesSelectCamerasOfMultiCameraNodes)
{
	// Two cameras side by side in the same image: the left one is dark, the right
	// one is bright. Selecting camera 1 keeps only the right one.
	const unsigned char kLeft = 10;
	const unsigned char kRight = 200;
	{
		DatabaseWriter writer(dbPath_);
		cv::Mat rgb(kImageHeight, kImageWidth*2, CV_8UC3, cv::Scalar::all(kLeft));
		rgb(cv::Rect(kImageWidth, 0, kImageWidth, kImageHeight)).setTo(cv::Scalar::all(kRight));
		std::vector<CameraModel> models;
		models.push_back(makeCameraModel(Transform(0, 0, 0, 0, 0, 0.5f) * CameraModel::opticalRotation()));
		models.push_back(makeCameraModel(Transform(0, 0, 0, 0, 0, -0.5f) * CameraModel::opticalRotation()));
		SensorData data;
		data.setRGBDImage(compressImage2(rgb, ".png"), cv::Mat(), models);
		data.setStamp(1.0);
		writer.addNode(1, 0, 0, data);
	}

	ReaderOptions options;
	options.cameraIndices.push_back(1);
	std::unique_ptr<DBReader> reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	std::vector<Frame> frames = readAll(*reader);
	ASSERT_EQ(1u, frames.size());
	const SensorData & data = frames[0].data;
	ASSERT_EQ(1u, data.cameraModels().size());
	EXPECT_TRUE(isApprox(Transform(0, 0, 0, 0, 0, -0.5f) * CameraModel::opticalRotation(), data.cameraModels()[0].localTransform()));
	ASSERT_EQ(kImageWidth, data.imageRaw().cols);
	EXPECT_EQ(kRight, data.imageRaw().at<cv::Vec3b>(0, 0)[0]);

	// With a local transform override for the selected camera.
	const Transform override(0.1f, 0, 0.2f, 0, 0, 1.0f);
	options.cameraLocalTransformOverrides.push_back(override);
	reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	frames = readAll(*reader);
	ASSERT_EQ(1u, frames.size());
	ASSERT_EQ(1u, frames[0].data.cameraModels().size());
	EXPECT_TRUE(isApprox(override * CameraModel::opticalRotation(), frames[0].data.cameraModels()[0].localTransform()));
}

TEST_F(DBReaderFixture, CameraLocalTransformOverrides)
{
	{
		DatabaseWriter writer(dbPath_);
		writer.addNode(1);
	}
	const Transform override(0.1f, 0, 0.2f, 0, 0, 1.0f);
	ReaderOptions options;
	options.cameraLocalTransformOverrides.push_back(override);
	std::unique_ptr<DBReader> reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	std::vector<Frame> frames = readAll(*reader);
	ASSERT_EQ(1u, frames.size());
	ASSERT_EQ(1u, frames[0].data.cameraModels().size());
	EXPECT_TRUE(isApprox(override * CameraModel::opticalRotation(), frames[0].data.cameraModels()[0].localTransform()));

	// Overrides that don't match the number of cameras are ignored.
	options.cameraLocalTransformOverrides.push_back(override);
	reader = makeReader(dbPath_, options);
	ASSERT_TRUE(reader->init());
	frames = readAll(*reader);
	ASSERT_EQ(1u, frames.size());
	ASSERT_EQ(1u, frames[0].data.cameraModels().size());
	EXPECT_TRUE(isApprox(CameraModel::opticalRotation(), frames[0].data.cameraModels()[0].localTransform()));
}

TEST_F(DBReaderFixture, OccupancyGridsPublishedUnlessIgnored)
{
	// DBReader publishes the local occupancy grids of the database, which Memory
	// then uses as is. With occupancyGridsIgnored, they are not published, so that
	// they are regenerated from the depth images or laser scans (e.g., with new
	// grid parameters).
	cv::Mat obstacles(1, 3, CV_32FC3);
	for(int i = 0; i < 3; ++i)
	{
		obstacles.at<cv::Vec3f>(0, i) = cv::Vec3f(float(i) * 0.1f, 0.0f, 0.0f);
	}
	const float kCellSize = 0.123f;
	{
		DatabaseWriter writer(dbPath_);
		// Node 1: with a laser scan the grid could be generated from.
		SensorData withScan = makeImageData(1, 1.0);
		cv::Mat scan(1, 10, CV_32FC2, cv::Scalar(1.0f, 0.0f));
		withScan.setLaserScan(LaserScan(compressData2(scan), 0, 0.0f, LaserScan::kXY));
		withScan.setOccupancyGrid(cv::Mat(), obstacles, cv::Mat(), kCellSize, cv::Point3f(0, 0, 0));
		writer.addNode(1, 0, 0, withScan);
		// Node 2: only an image.
		SensorData imageOnly = makeImageData(2, 2.0);
		imageOnly.setOccupancyGrid(cv::Mat(), obstacles, cv::Mat(), kCellSize, cv::Point3f(0, 0, 0));
		writer.addNode(2, 0, 0, imageOnly);
	}

	for(int pass = 0; pass < 2; ++pass)
	{
		const bool ignored = pass == 1;
		SCOPED_TRACE(uFormat("occupancyGridsIgnored=%s", ignored?"true":"false"));
		ReaderOptions options;
		options.occupancyGridsIgnored = ignored;
		std::unique_ptr<DBReader> reader = makeReader(dbPath_, options);
		ASSERT_TRUE(reader->init());
		std::vector<Frame> frames = readAll(*reader);
		ASSERT_EQ(2u, frames.size());
		for(size_t i=0; i<frames.size(); ++i)
		{
			SCOPED_TRACE(uFormat("node %d", frames[i].data.id()));
			EXPECT_FLOAT_EQ(ignored?0.0f:kCellSize, frames[i].data.gridCellSize());
			EXPECT_EQ(ignored?0u:obstacles.total(), frames[i].data.gridObstacleCellsRaw().total());
		}
	}
}
