#include <gtest/gtest.h>
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
#include <opencv2/core.hpp>

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
			pose = Transform(1, 0, 0, 0, 0, 0, 1);
			covariance = cv::Mat::eye(6, 6, CV_64FC1);
			return true;
		}
		return false;
	}

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
	
	bool wasInitCalled() const { return initCalled_; }
	std::string getCalibrationFolder() const { return calibrationFolder_; }
	std::string getCameraName() const { return cameraName_; }
	int getCaptureCount() const { return captureCount_; }

protected:
	virtual SensorData captureData(SensorCaptureInfo * info = 0)
	{
		++captureCount_;
		
		// Create a simple laser scan
		LaserScan scan = LaserScan::backwardCompatibility(cv::Mat(1, 100, CV_32FC2));
		SensorData data;
		data.setLaserScan(scan);
		data.setStamp(UTimer::now());
		return data;
	}

private:
	bool initCalled_;
	bool initResult_;
	std::string serial_;
	std::string calibrationFolder_;
	std::string cameraName_;
	int captureCount_;
};

// Constructor Tests

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

