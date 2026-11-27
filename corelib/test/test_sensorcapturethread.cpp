#include <gtest/gtest.h>
#include <limits>
#include <algorithm>
#include <vector>
#include <rtabmap/utilite/UMutex.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/core/SensorEvent.h>
#include <rtabmap/core/SensorCaptureThread.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/Lidar.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/SensorCaptureInfo.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/core/camera/CameraImages.h>
#include <rtabmap/utilite/UStl.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "TestUtils.h"

using namespace rtabmap;

// Mock Camera implementation for testing
class MockCamera : public Camera
{
public:
	MockCamera(float imageRate = 0, const Transform & localTransform = Transform::getIdentity()) :
		Camera(imageRate, localTransform),
		initCalled_(false),
		initResult_(true),
		serial_("MOCK_CAMERA_001"),
		calibrated_(false),
		odomProvided_(false),
		captureCount_(0)
	{
	}

	virtual ~MockCamera() {}

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "")
	{
		initCalled_ = true;
		calibrationFolder_ = calibrationFolder;
		cameraName_ = cameraName;
		return initResult_;
	}

	virtual std::string getSerial() const
	{
		return serial_;
	}

	virtual bool isCalibrated() const
	{
		return calibrated_;
	}

	virtual bool odomProvided() const
	{
		return odomProvided_;
	}

	virtual bool getPose(double stamp, Transform & pose, cv::Mat & covariance, double maxWaitTime = 0.06)
	{
		if (odomProvided_)
		{
			// moving along x at poseVelocity_ (m/s) when set, else a fixed pose
			pose = poseVelocity_ != 0.0f ? Transform(poseVelocity_ * float(stamp), 0, 0, 0, 0, 0) : Transform(1, 0, 0, 0, 0, 0, 1);
			covariance = cv::Mat::eye(6, 6, CV_64FC1);
			return true;
		}
		return false;
	}
	void setPoseVelocity(float velocity) { poseVelocity_ = velocity; }

	// Test helpers
	void setInitResult(bool result) { initResult_ = result; }
	void setSerial(const std::string & serial) { serial_ = serial; }
	void setCalibrated(bool calibrated) { calibrated_ = calibrated; }
	void setOdomProvided(bool provided) { odomProvided_ = provided; }
	
	bool wasInitCalled() const { return initCalled_; }
	std::string getCalibrationFolder() const { return calibrationFolder_; }
	std::string getCameraName() const { return cameraName_; }
	int getCaptureCount() const { return captureCount_; }

protected:
	virtual SensorData captureImage(SensorCaptureInfo * info = 0)
	{
		++captureCount_;
		
		cv::Mat image = cv::Mat::ones(480, 640, CV_8UC3) * 128;
		SensorData data(image, getNextSeqID(), UTimer::now());
		return data;
	}

private:
	bool initCalled_;
	bool initResult_;
	std::string serial_;
	bool calibrated_;
	bool odomProvided_;
	float poseVelocity_ = 0.0f;
	std::string calibrationFolder_;
	std::string cameraName_;
	int captureCount_;
};

// Mock Lidar implementation for testing
class MockLidar : public Lidar
{
public:
	MockLidar(float lidarRate = 0, const Transform & localTransform = Transform::getIdentity()) :
		Lidar(lidarRate, localTransform),
		initCalled_(false),
		initResult_(true),
		serial_("MOCK_LIDAR_001"),
		captureCount_(0)
	{
	}

	virtual ~MockLidar() {}

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "")
	{
		initCalled_ = true;
		calibrationFolder_ = calibrationFolder;
		cameraName_ = cameraName;
		return initResult_;
	}

	virtual std::string getSerial() const
	{
		return serial_;
	}

	// Test helpers
	void setInitResult(bool result) { initResult_ = result; }
	void setSerial(const std::string & serial) { serial_ = serial; }
	// A scan to capture instead of the default one, stamped at scanStamp (s)
	void setScan(const LaserScan & scan, double scanStamp) { scan_ = scan; scanStamp_ = scanStamp; }
	
	bool wasInitCalled() const { return initCalled_; }
	std::string getCalibrationFolder() const { return calibrationFolder_; }
	std::string getCameraName() const { return cameraName_; }
	int getCaptureCount() const { return captureCount_; }

protected:
	virtual SensorData captureData(SensorCaptureInfo * info = 0)
	{
		++captureCount_;
		
		// Create a simple laser scan, unless one was given
		LaserScan scan = scan_.isEmpty() ? LaserScan::backwardCompatibility(cv::Mat(1, 100, CV_32FC2)) : scan_;
		SensorData data;
		data.setLaserScan(scan);
		data.setStamp(scan_.isEmpty() ? UTimer::now() : scanStamp_);
		return data;
	}

private:
	bool initCalled_;
	bool initResult_;
	std::string serial_;
	std::string calibrationFolder_;
	std::string cameraName_;
	int captureCount_;
	LaserScan scan_;
	double scanStamp_ = 0.0;
};

// Constructor Tests

// Collects the data of the SensorEvents posted by the thread (dispatched on the
// UEventsManager thread, read from the test thread: hence the mutex).
class SensorEventCollector : public UEventsHandler
{
public:
	std::vector<SensorData> snapshot() const
	{
		UScopeMutex lock(mutex_);
		return datas_;
	}
	size_t size() const
	{
		UScopeMutex lock(mutex_);
		return datas_.size();
	}
protected:
	virtual bool handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("SensorEvent") == 0)
		{
			SensorEvent * sensorEvent = (SensorEvent*)event;
			if(sensorEvent->getCode() == SensorEvent::kCodeData)
			{
				UScopeMutex lock(mutex_);
				datas_.push_back(sensorEvent->data());
			}
		}
		return false;
	}
private:
	mutable UMutex mutex_;
	std::vector<SensorData> datas_;
};

TEST(SensorCaptureThreadTest, ConstructorCameraOnly)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	EXPECT_FALSE(thread.isCapturing());
	EXPECT_TRUE(thread.isPaused());
	EXPECT_EQ(thread.camera(), camera);
	EXPECT_EQ(thread.lidar(), nullptr);
	EXPECT_EQ(thread.odomSensor(), nullptr);
	EXPECT_FALSE(thread.odomProvided());
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, ConstructorLidarOnly)
{
	MockLidar * lidar = new MockLidar();
	SensorCaptureThread thread(lidar);
	
	EXPECT_FALSE(thread.isCapturing());
	EXPECT_TRUE(thread.isPaused());
	EXPECT_EQ(thread.lidar(), lidar);
	EXPECT_EQ(thread.camera(), nullptr);
	EXPECT_EQ(thread.odomSensor(), nullptr);
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, ConstructorLidarWithCamera)
{
	MockLidar * lidar = new MockLidar();
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(lidar, camera);
	
	EXPECT_FALSE(thread.isCapturing());
	EXPECT_EQ(thread.lidar(), lidar);
	EXPECT_EQ(thread.camera(), camera);
	EXPECT_EQ(thread.odomSensor(), nullptr);
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, ConstructorCameraWithOdomSensor)
{
	MockCamera * camera = new MockCamera();
	MockCamera * odomSensor = new MockCamera();
	Transform extrinsics = Transform::getIdentity();
	
	SensorCaptureThread thread(camera, odomSensor, extrinsics, 0.0, 1.0f, 0.1);
	
	EXPECT_FALSE(thread.isCapturing());
	EXPECT_EQ(thread.camera(), camera);
	EXPECT_EQ(thread.odomSensor(), odomSensor);
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, ConstructorLidarWithOdomSensor)
{
	MockLidar * lidar = new MockLidar();
	MockCamera * odomSensor = new MockCamera();
	
	SensorCaptureThread thread(lidar, odomSensor, 0.0, 1.0f, 0.1);
	
	EXPECT_FALSE(thread.isCapturing());
	EXPECT_EQ(thread.lidar(), lidar);
	EXPECT_EQ(thread.odomSensor(), odomSensor);
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, ConstructorLidarCameraOdom)
{
	MockLidar * lidar = new MockLidar();
	MockCamera * camera = new MockCamera();
	MockCamera * odomSensor = new MockCamera();
	Transform extrinsics = Transform::getIdentity();
	
	SensorCaptureThread thread(lidar, camera, odomSensor, extrinsics, 0.0, 1.0f, 0.1);
	
	EXPECT_FALSE(thread.isCapturing());
	EXPECT_EQ(thread.lidar(), lidar);
	EXPECT_EQ(thread.camera(), camera);
	EXPECT_EQ(thread.odomSensor(), odomSensor);
	
	// Thread was never started, destructor will handle cleanup
}

// Configuration Tests

TEST(SensorCaptureThreadTest, SetGetMirroring)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	thread.setMirroringEnabled(true);
	// No getter, so we can't verify directly, but it shouldn't crash
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, SetGetStereoExposureCompensation)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	thread.setStereoExposureCompensation(true);
	// No getter, so we can't verify directly, but it shouldn't crash
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, SetGetColorOnly)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	thread.setColorOnly(true);
	// No getter, so we can't verify directly, but it shouldn't crash
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, SetGetImageDecimation)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	thread.setImageDecimation(2);
	// No getter, so we can't verify directly, but it shouldn't crash
	
	thread.setImageDecimation(4);
	thread.setImageDecimation(1);
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, SetGetHistogramMethod)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	thread.setHistogramMethod(0); // None
	thread.setHistogramMethod(1); // EqualizeHist
	thread.setHistogramMethod(2); // CLAHE
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, SetGetStereoToDepth)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	thread.setStereoToDepth(true);
	thread.setStereoToDepth(false);
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, SetGetFrameRate)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	thread.setFrameRate(30.0f);
	thread.setFrameRate(0.0f);
	thread.setFrameRate(60.0f);
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, SetOdomAsGroundTruth)
{
	MockCamera * camera = new MockCamera();
	MockCamera * odomSensor = new MockCamera();
	Transform extrinsics = Transform::getIdentity();
	
	SensorCaptureThread thread(camera, odomSensor, extrinsics);
	
	thread.setOdomAsGroundTruth(true);
	thread.setOdomAsGroundTruth(false);
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, EnableDisableBilateralFiltering)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	thread.enableBilateralFiltering(5.0f, 50.0f);
	thread.disableBilateralFiltering();
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, EnableDisableIMUFiltering)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	ParametersMap params;
	thread.enableIMUFiltering(1, params, false); // Complementary filter
	thread.disableIMUFiltering();
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, EnableDisableFeatureDetection)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	ParametersMap params;
	thread.enableFeatureDetection(params);
	thread.disableFeatureDetection();
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, SetScanParameters)
{
	MockLidar * lidar = new MockLidar();
	SensorCaptureThread thread(lidar);
	
	thread.setScanParameters(
		false, // fromDepth
		1,     // downsampleStep
		0.0f,  // rangeMin
		10.0f, // rangeMax
		0.05f, // voxelSize
		10,    // normalsK
		0.1f,  // normalsRadius
		0.8f,  // groundNormalsUp
		true   // deskewing
	);
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, SetScanParametersFromDepth)
{
	MockLidar * lidar = new MockLidar();
	SensorCaptureThread thread(lidar);
	
	thread.setScanParameters(
		true,  // fromDepth
		2,     // downsampleStep (image decimation)
		0.0f,  // rangeMin
		0.0f,  // rangeMax
		0.0f,  // voxelSize
		0,     // normalsK
		0.0f,  // normalsRadius
		0.0f,  // groundNormalsUp
		false  // deskewing
	);
	
	// Thread was never started, destructor will handle cleanup
}

// Thread State Tests

TEST(SensorCaptureThreadTest, InitialState)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	EXPECT_FALSE(thread.isCapturing());
	EXPECT_TRUE(thread.isPaused());
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, StartStopThread)
{
	MockCamera * camera = new MockCamera();
	camera->init(); // Initialize camera before starting thread
	SensorCaptureThread thread(camera);
	
	// Start thread
	thread.start();
	
	// Give it a moment to start
	uSleep(50);
	
	EXPECT_TRUE(thread.isCapturing());
	EXPECT_FALSE(thread.isPaused());
	
	// Stop thread
	thread.kill();
	thread.join();
	
	EXPECT_FALSE(thread.isCapturing());
	EXPECT_TRUE(thread.isPaused());
}

TEST(SensorCaptureThreadTest, OdomProvided)
{
	MockCamera * camera = new MockCamera();
	MockCamera * odomSensor = new MockCamera();
	odomSensor->setOdomProvided(true);
	Transform extrinsics = Transform::getIdentity();
	
	SensorCaptureThread thread(camera, odomSensor, extrinsics);
	
	EXPECT_TRUE(thread.odomProvided());
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, OdomNotProvided)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	EXPECT_FALSE(thread.odomProvided());
	
	// Thread was never started, destructor will handle cleanup
}

// Comprehensive Usage Test

TEST(SensorCaptureThreadTest, ComprehensiveConfiguration)
{
	MockCamera * camera = new MockCamera();
	ParametersMap params;
	SensorCaptureThread thread(camera, params);
	
	// Configure all settings
	thread.setMirroringEnabled(true);
	thread.setStereoExposureCompensation(false);
	thread.setColorOnly(false);
	thread.setImageDecimation(2);
	thread.setHistogramMethod(2); // CLAHE
	thread.setStereoToDepth(false);
	thread.setFrameRate(30.0f);
	thread.setOdomAsGroundTruth(false);
	thread.enableBilateralFiltering(5.0f, 50.0f);
	
	// Verify thread state
	EXPECT_FALSE(thread.isCapturing());
	EXPECT_TRUE(thread.isPaused());
	EXPECT_EQ(thread.camera(), camera);
	
	// Thread was never started, destructor will handle cleanup
}

// Edge Cases

TEST(SensorCaptureThreadTest, MultipleConfigurationChanges)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	// Change settings multiple times
	for (int i = 0; i < 10; ++i)
	{
		thread.setImageDecimation(i % 4 + 1);
		thread.setHistogramMethod(i % 3);
		thread.setFrameRate((float)(i * 10));
	}
	
	// Should not crash
	EXPECT_FALSE(thread.isCapturing());
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, ZeroFrameRate)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	thread.setFrameRate(0.0f); // Unlimited
	
	// Should handle zero frame rate gracefully
	EXPECT_FALSE(thread.isCapturing());
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, VeryHighFrameRate)
{
	MockCamera * camera = new MockCamera();
	SensorCaptureThread thread(camera);
	
	thread.setFrameRate(1000.0f); // Very high frame rate
	
	// Should handle high frame rate gracefully
	EXPECT_FALSE(thread.isCapturing());
	
	// Thread was never started, destructor will handle cleanup
}

TEST(SensorCaptureThreadTest, ScanParametersEdgeCases)
{
	MockLidar * lidar = new MockLidar();
	SensorCaptureThread thread(lidar);
	
	// Test with all zeros (disabled features)
	thread.setScanParameters(false, 1, 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, false);
	
	// Test with maximum values
	thread.setScanParameters(false, 10, 100.0f, 1000.0f, 1.0f, 100, 5.0f, 1.0f, true);
	
	// Should not crash
	EXPECT_FALSE(thread.isCapturing());
	
	// Thread was never started, destructor will handle cleanup
}


// With an odometry sensor and deskewing on, a timed scan is deskewed from the
// poses at its first and last points. The scan here is organized (rings x
// columns): its last point is on the last row, where a 1xN addressing would be
// out of bounds. Deskewing drops the time channel and densifies the scan.
TEST(SensorCaptureThreadTest, DeskewsAnOrganizedTimedScanWithTheOdomSensor)
{
	const int rings = 4;
	const int columns = 8;
	cv::Mat points(rings, columns, CV_32FC(5));
	for(int v=0; v<rings; ++v)
	{
		for(int u=0; u<columns; ++u)
		{
			float * p = points.ptr<float>(v, u);
			p[0] = 2.0f; p[1] = 0.1f*float(u); p[2] = 0.1f*float(v); p[3] = 1.0f;
			p[4] = 0.05f*float(u)/float(columns-1); // time: 50 ms sweep
		}
	}
	MockLidar * lidar = new MockLidar();
	lidar->setScan(LaserScan(points, rings*columns, 0.0f, LaserScan::kXYZIT), /*scanStamp=*/100.0);
	lidar->init();
	MockCamera * odomSensor = new MockCamera();
	odomSensor->setOdomProvided(true);
	odomSensor->setPoseVelocity(1.0f); // 1 m/s along x
	odomSensor->init();

	SensorCaptureThread thread(lidar, odomSensor, 0.0, 1.0f, 0.1);
	thread.setScanParameters(false, 1, 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, /*deskewing=*/true);

	SensorEventCollector collector;
	UEventsManager::addHandler(&collector);
	thread.start();
	UTimer timer;
	while(timer.elapsed() < 3.0 && collector.size() == 0)
	{
		uSleep(5);
	}
	thread.kill();
	thread.join(true);
	UEventsManager::removeHandler(&collector);

	const std::vector<SensorData> datas = collector.snapshot();
	ASSERT_FALSE(datas.empty()) << "no data posted by the capture thread";
	const LaserScan & scan = datas[0].laserScanRaw();
	ASSERT_FALSE(scan.isEmpty());
	EXPECT_EQ(scan.format(), LaserScan::kXYZI) << "deskewed: the time channel is consumed";
	EXPECT_EQ(scan.size(), rings*columns);
	EXPECT_FALSE(scan.isOrganized());
	// The sensor moved 5 cm along x during the 50 ms sweep: the points are
	// brought into one frame, so the last column ends up 5 cm further along x
	// than the first one (all were given the same x), whatever the order the
	// deskewed points come in
	float xMin = std::numeric_limits<float>::max();
	float xMax = -std::numeric_limits<float>::max();
	for(int i=0; i<scan.size(); ++i)
	{
		xMin = std::min(xMin, scan.point(i)[0]);
		xMax = std::max(xMax, scan.point(i)[0]);
	}
	EXPECT_NEAR(xMax - xMin, 0.05f, 5e-3f);
}

// Pausing in the GUI kills the capture thread (join(true)) and restarts it on
// resume, and shift-click on pause steps by restarting it and killing it half a
// frame period later: the kill arrives while takeData() waits for the frame rate.
// The frame read then must still be posted, otherwise it would be skipped (here,
// as each cycle is killed during that wait, no frame would ever be posted). A
// slow machine can read more than one frame per cycle: every frame read must be
// posted, in order.
TEST(SensorCaptureThreadTest, FrameReadWhileKilledIsPosted)
{
	rtabmap::test::TempDir dir("sensorcapturethread_images");
	const int images = 20; // more than the frames read, even at 2 per cycle
	for(int i=1; i<=images; ++i)
	{
		ASSERT_TRUE(cv::imwrite(uFormat("%s/%02d.png", dir.path().c_str(), i), cv::Mat(8, 8, CV_8UC1, cv::Scalar(i))));
	}
	CameraImages * camera = new CameraImages(dir.path(), 10.0f); // 100 ms period
	ASSERT_TRUE(camera->init());
	SensorCaptureThread thread(camera);

	SensorEventCollector collector;
	UEventsManager::addHandler(&collector);
	const int cycles = 5;
	for(int i=0; i<cycles; ++i)
	{
		thread.start();
		uSleep(50); // half the period, like a shift-click step
		thread.join(true);
	}
	// The frames are numbered from 1 as they are read: the next one tells how many
	// the thread read.
	const SensorData next = camera->takeData();
	ASSERT_FALSE(next.imageRaw().empty());
	const int read = next.id() - 1;
	ASSERT_GE(read, cycles); // each cycle reads at least the frame it is killed on

	// events are dispatched asynchronously
	UTimer timer;
	while(timer.elapsed() < 3.0 && (int)collector.size() < read)
	{
		uSleep(5);
	}
	uSleep(50); // let any unexpected extra event arrive
	UEventsManager::removeHandler(&collector);

	const std::vector<SensorData> datas = collector.snapshot();
	ASSERT_EQ((int)datas.size(), read) << "a frame read was not posted";
	for(size_t i=0; i<datas.size(); ++i)
	{
		EXPECT_EQ(datas[i].id(), (int)i+1) << "a frame was skipped";
	}
}
