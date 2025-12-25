#include <gtest/gtest.h>
#include <rtabmap/core/SensorCapture.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/SensorCaptureInfo.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/UTimer.h>
#include <opencv2/core.hpp>

using namespace rtabmap;

// Mock SensorCapture implementation for testing
class MockSensorCapture : public SensorCapture
{
public:
	MockSensorCapture(float frameRate = 0, const Transform & localTransform = Transform::getIdentity()) :
		SensorCapture(frameRate, localTransform),
		initCalled_(false),
		initResult_(true),
		serial_("MOCK_SENSOR_001"),
		odomProvided_(false),
		captureCount_(0)
	{
	}

	virtual ~MockSensorCapture() {}

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
	void setOdomProvided(bool provided) { odomProvided_ = true; }
	
	bool wasInitCalled() const { return initCalled_; }
	std::string getCalibrationFolder() const { return calibrationFolder_; }
	std::string getCameraName() const { return cameraName_; }
	int getCaptureCount() const { return captureCount_; }

protected:
	virtual SensorData captureData(SensorCaptureInfo * info = 0)
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
	bool odomProvided_;
	std::string calibrationFolder_;
	std::string cameraName_;
	int captureCount_;
};

// Constructor Tests

TEST(SensorCaptureTest, DefaultConstructor)
{
	MockSensorCapture sensor;
	
	EXPECT_EQ(sensor.getFrameRate(), 0.0f);
	EXPECT_TRUE(sensor.getLocalTransform().isIdentity());
	EXPECT_FALSE(sensor.wasInitCalled());
	EXPECT_EQ(sensor.getSerial(), "MOCK_SENSOR_001");
	EXPECT_FALSE(sensor.odomProvided());
}

TEST(SensorCaptureTest, ConstructorWithFrameRate)
{
	float frameRate = 30.0f;
	MockSensorCapture sensor(frameRate);
	
	EXPECT_FLOAT_EQ(sensor.getFrameRate(), frameRate);
	EXPECT_TRUE(sensor.getLocalTransform().isIdentity());
}

TEST(SensorCaptureTest, ConstructorWithLocalTransform)
{
	Transform localTransform(1, 0, 0, 0, 0, 0, 1);
	MockSensorCapture sensor(0, localTransform);
	
	EXPECT_EQ(sensor.getLocalTransform(), localTransform);
}

TEST(SensorCaptureTest, ConstructorWithFrameRateAndTransform)
{
	float frameRate = 15.0f;
	Transform localTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
	MockSensorCapture sensor(frameRate, localTransform);
	
	EXPECT_FLOAT_EQ(sensor.getFrameRate(), frameRate);
	EXPECT_EQ(sensor.getLocalTransform(), localTransform);
}

// Init Tests

TEST(SensorCaptureTest, InitDefault)
{
	MockSensorCapture sensor;
	
	EXPECT_TRUE(sensor.init());
	EXPECT_TRUE(sensor.wasInitCalled());
	EXPECT_EQ(sensor.getCalibrationFolder(), ".");
	EXPECT_EQ(sensor.getCameraName(), "");
}

TEST(SensorCaptureTest, InitWithCalibrationFolder)
{
	MockSensorCapture sensor;
	std::string folder = "/path/to/calibration";
	
	EXPECT_TRUE(sensor.init(folder));
	EXPECT_EQ(sensor.getCalibrationFolder(), folder);
	EXPECT_EQ(sensor.getCameraName(), "");
}

TEST(SensorCaptureTest, InitWithCalibrationFolderAndCameraName)
{
	MockSensorCapture sensor;
	std::string folder = "/path/to/calibration";
	std::string cameraName = "camera1";
	
	EXPECT_TRUE(sensor.init(folder, cameraName));
	EXPECT_EQ(sensor.getCalibrationFolder(), folder);
	EXPECT_EQ(sensor.getCameraName(), cameraName);
}

TEST(SensorCaptureTest, InitFailure)
{
	MockSensorCapture sensor;
	sensor.setInitResult(false);
	
	EXPECT_FALSE(sensor.init());
	EXPECT_TRUE(sensor.wasInitCalled());
}

// Serial Tests

TEST(SensorCaptureTest, GetSerial)
{
	MockSensorCapture sensor;
	
	EXPECT_EQ(sensor.getSerial(), "MOCK_SENSOR_001");
	
	sensor.setSerial("CUSTOM_SERIAL_123");
	EXPECT_EQ(sensor.getSerial(), "CUSTOM_SERIAL_123");
}

// Frame Rate Tests

TEST(SensorCaptureTest, SetGetFrameRate)
{
	MockSensorCapture sensor;
	
	EXPECT_EQ(sensor.getFrameRate(), 0.0f);
	
	sensor.setFrameRate(30.0f);
	EXPECT_FLOAT_EQ(sensor.getFrameRate(), 30.0f);
	
	sensor.setFrameRate(0.0f);
	EXPECT_FLOAT_EQ(sensor.getFrameRate(), 0.0f);
	
	sensor.setFrameRate(60.0f);
	EXPECT_FLOAT_EQ(sensor.getFrameRate(), 60.0f);
}

// Local Transform Tests

TEST(SensorCaptureTest, SetGetLocalTransform)
{
	MockSensorCapture sensor;
	
	EXPECT_TRUE(sensor.getLocalTransform().isIdentity());
	
	Transform transform(1, 0, 0, 0, 0, 0, 1);
	sensor.setLocalTransform(transform);
	EXPECT_EQ(sensor.getLocalTransform(), transform);
	
	Transform transform2(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
	sensor.setLocalTransform(transform2);
	EXPECT_EQ(sensor.getLocalTransform(), transform2);
}

// Reset Timer Tests

TEST(SensorCaptureTest, ResetTimer)
{
	MockSensorCapture sensor;
	
	// Reset should not throw
	EXPECT_NO_THROW(sensor.resetTimer());
	
	// After reset, timer should be ready for frame rate control
	sensor.setFrameRate(10.0f);
	sensor.resetTimer();
	
	// First capture should be fast (no throttling needed)
	SensorData data = sensor.takeData();
	EXPECT_TRUE(data.isValid());
}

// TakeData Tests

TEST(SensorCaptureTest, TakeDataBasic)
{
	MockSensorCapture sensor;
	
	SensorData data = sensor.takeData();
	
	EXPECT_TRUE(data.isValid());
	EXPECT_GT(data.id(), 0);
	EXPECT_GT(data.stamp(), 0.0);
	EXPECT_FALSE(data.imageRaw().empty());
	EXPECT_EQ(sensor.getCaptureCount(), 1);
}

TEST(SensorCaptureTest, TakeDataWithInfo)
{
	MockSensorCapture sensor;
	SensorCaptureInfo info;
	
	SensorData data = sensor.takeData(&info);
	
	EXPECT_TRUE(data.isValid());
	EXPECT_EQ(info.id, data.id());
	EXPECT_DOUBLE_EQ(info.stamp, data.stamp());
	EXPECT_GT(info.timeCapture, 0.0);
	EXPECT_LE(info.timeCapture, 0.1); // Should be very fast for mock
}

TEST(SensorCaptureTest, TakeDataSequenceIDs)
{
	MockSensorCapture sensor;
	
	SensorData data1 = sensor.takeData();
	SensorData data2 = sensor.takeData();
	SensorData data3 = sensor.takeData();
	
	EXPECT_EQ(data1.id() + 1, data2.id());
	EXPECT_EQ(data2.id() + 1, data3.id());
	EXPECT_EQ(sensor.getCaptureCount(), 3);
}

TEST(SensorCaptureTest, TakeDataNoFrameRateThrottling)
{
	MockSensorCapture sensor;
	sensor.setFrameRate(0.0f); // Unlimited
	
	UTimer timer;
	SensorData data1 = sensor.takeData();
	SensorData data2 = sensor.takeData();
	double elapsed = timer.ticks();
	
	// Without throttling, captures should be very fast
	EXPECT_LT(elapsed, 0.1);
	EXPECT_TRUE(data1.isValid());
	EXPECT_TRUE(data2.isValid());
}

TEST(SensorCaptureTest, TakeDataWithFrameRateThrottling)
{
	MockSensorCapture sensor;
	sensor.setFrameRate(10.0f); // 10 Hz = 100ms per frame
	sensor.resetTimer();
	
	UTimer timer;
	SensorData data1 = sensor.takeData();
	SensorData data2 = sensor.takeData();
	double elapsed = timer.ticks();
	
	// With 10 Hz throttling, two captures should take at least ~100ms
	EXPECT_GE(elapsed, 0.2); // Should be at least 200 ms for two frames
	EXPECT_LE(elapsed, 0.21); // Slight margin
	EXPECT_TRUE(data1.isValid());
	EXPECT_TRUE(data2.isValid());
}

// OdomProvided Tests

TEST(SensorCaptureTest, OdomProvidedDefault)
{
	MockSensorCapture sensor;
	
	EXPECT_FALSE(sensor.odomProvided());
}

TEST(SensorCaptureTest, OdomProvidedEnabled)
{
	MockSensorCapture sensor;
	sensor.setOdomProvided(true);
	
	EXPECT_TRUE(sensor.odomProvided());
}

// GetPose Tests

TEST(SensorCaptureTest, GetPoseDefault)
{
	MockSensorCapture sensor;
	Transform pose;
	cv::Mat covariance;
	
	EXPECT_FALSE(sensor.getPose(0.0, pose, covariance));
}

TEST(SensorCaptureTest, GetPoseWithOdom)
{
	MockSensorCapture sensor;
	sensor.setOdomProvided(true);
	
	Transform pose;
	cv::Mat covariance;
	
	EXPECT_TRUE(sensor.getPose(0.0, pose, covariance));
	EXPECT_FALSE(pose.isNull());
	EXPECT_FALSE(covariance.empty());
	EXPECT_EQ(covariance.rows, 6);
	EXPECT_EQ(covariance.cols, 6);
}

// Comprehensive Usage Test

TEST(SensorCaptureTest, ComprehensiveUsage)
{
	// Create sensor with frame rate and transform
	float frameRate = 20.0f;
	Transform localTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
	MockSensorCapture sensor(frameRate, localTransform);
	
	// Initialize
	EXPECT_TRUE(sensor.init("/calib", "test_camera"));
	EXPECT_EQ(sensor.getCalibrationFolder(), "/calib");
	EXPECT_EQ(sensor.getCameraName(), "test_camera");
	
	// Verify settings
	EXPECT_FLOAT_EQ(sensor.getFrameRate(), frameRate);
	EXPECT_EQ(sensor.getLocalTransform(), localTransform);
	EXPECT_EQ(sensor.getSerial(), "MOCK_SENSOR_001");
	
	// Change settings
	sensor.setFrameRate(30.0f);
	Transform newTransform = Transform::getIdentity();
	sensor.setLocalTransform(newTransform);
	sensor.setSerial("NEW_SERIAL");
	
	EXPECT_FLOAT_EQ(sensor.getFrameRate(), 30.0f);
	EXPECT_EQ(sensor.getLocalTransform(), newTransform);
	EXPECT_EQ(sensor.getSerial(), "NEW_SERIAL");
	
	// Capture data
	sensor.resetTimer();
	SensorCaptureInfo info;
	SensorData data = sensor.takeData(&info);
	
	EXPECT_TRUE(data.isValid());
	EXPECT_EQ(info.id, data.id());
	EXPECT_DOUBLE_EQ(info.stamp, data.stamp());
	EXPECT_GT(info.timeCapture, 0.0);
	EXPECT_EQ(sensor.getCaptureCount(), 1);
}

// Edge Cases

TEST(SensorCaptureTest, TakeDataMultipleCalls)
{
	MockSensorCapture sensor;
	
	for (int i = 0; i < 10; ++i)
	{
		SensorData data = sensor.takeData();
		EXPECT_TRUE(data.isValid());
	}
	
	EXPECT_EQ(sensor.getCaptureCount(), 10);
}

TEST(SensorCaptureTest, TakeDataWithNullInfo)
{
	MockSensorCapture sensor;
	
	// Should not crash with null info
	SensorData data = sensor.takeData(nullptr);
	EXPECT_TRUE(data.isValid());
}

TEST(SensorCaptureTest, HighFrameRate)
{
	MockSensorCapture sensor;
	sensor.setFrameRate(1000.0f); // Very high frame rate
	sensor.resetTimer();
	
	// Should handle high frame rate gracefully
	UTimer timer;
	SensorData data = sensor.takeData();
	double elapsed = timer.ticks();

	EXPECT_TRUE(data.isValid());
	EXPECT_GE(elapsed, 0.001);
	EXPECT_LT(elapsed, 0.01);
}

TEST(SensorCaptureTest, VeryLowFrameRate)
{
	MockSensorCapture sensor;
	sensor.setFrameRate(2.0f); // Low frame rate (500 ms seconds per frame)
	sensor.resetTimer();
	
	UTimer timer;
	SensorData data = sensor.takeData();
	double elapsed = timer.ticks();
	
	EXPECT_TRUE(data.isValid());
	EXPECT_GE(elapsed, 0.5);
	EXPECT_LT(elapsed, 0.51);
}

