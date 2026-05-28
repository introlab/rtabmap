#include <gtest/gtest.h>
#include <rtabmap/core/IMUThread.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/IMUFilter.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMutex.h>
#include <rtabmap/utilite/UTimer.h>
#include <fstream>
#include <cmath>
#include <unistd.h>
#include <vector>

using namespace rtabmap;

namespace {

static int g_fileCounter = 0;

static std::string tempImuCsvPath()
{
	return uFormat("/tmp/rtabmap_imuthread_test_%d_%d.csv", getpid(), ++g_fileCounter);
}

static bool writeImuCsv(const std::string & path, const std::vector<std::string> & rows)
{
	std::ofstream file(path.c_str());
	if(!file.good())
	{
		return false;
	}
	file << "#timestamp,wx,wy,wz,ax,ay,az\n";
	for(size_t i = 0; i < rows.size(); ++i)
	{
		file << rows[i] << "\n";
	}
	return file.good();
}

static bool orientationSet(const cv::Vec4d & orientation)
{
	return orientation[0] != 0.0 || orientation[1] != 0.0 || orientation[2] != 0.0 || orientation[3] != 0.0;
}

static void expectVec3Near(const cv::Vec3d & a, const cv::Vec3d & b, double tol = 1e-5)
{
	EXPECT_NEAR(a[0], b[0], tol);
	EXPECT_NEAR(a[1], b[1], tol);
	EXPECT_NEAR(a[2], b[2], tol);
}

static void expectQuatNear(
		const cv::Vec4d & q,
		double ex,
		double ey,
		double ez,
		double ew,
		double tol = 1e-3)
{
	EXPECT_NEAR(q[0], ex, tol);
	EXPECT_NEAR(q[1], ey, tol);
	EXPECT_NEAR(q[2], ez, tol);
	EXPECT_NEAR(q[3], ew, tol);
}

class IMUEventCollector : public UEventsHandler
{
public:
	struct Sample
	{
		IMU data;
		double stamp;
		bool valid;
	};

	// Dispatched on UEventsManager thread; reads (size/snapshot) come from the test
	// thread, so all access to samples_ goes through mutex_.
	void clear()
	{
		UScopeMutex lock(mutex_);
		samples_.clear();
	}
	std::vector<Sample> snapshot() const
	{
		UScopeMutex lock(mutex_);
		return samples_;
	}
	size_t size() const
	{
		UScopeMutex lock(mutex_);
		return samples_.size();
	}
	size_t validCount() const
	{
		UScopeMutex lock(mutex_);
		size_t count = 0;
		for(size_t i = 0; i < samples_.size(); ++i)
		{
			if(samples_[i].valid)
			{
				++count;
			}
		}
		return count;
	}

protected:
	virtual bool handleEvent(UEvent * event)
	{
		if(event->getClassName() == "IMUEvent")
		{
			const IMUEvent * imuEvent = static_cast<IMUEvent *>(event);
			Sample sample;
			sample.data = imuEvent->getData();
			sample.stamp = imuEvent->getStamp();
			sample.valid = !imuEvent->getData().empty();
			UScopeMutex lock(mutex_);
			samples_.push_back(sample);
		}
		return false;
	}

private:
	mutable UMutex mutex_;
	std::vector<Sample> samples_;
};

static std::vector<IMUEventCollector::Sample> runThread(
		IMUThread & thread,
		size_t minValidSamples = 0,
		double maxWaitSec = 2.0)
{
	IMUEventCollector collector;
	UEventsManager::addHandler(&collector);
	thread.start();

	UTimer timer;
	while(timer.ticks() < maxWaitSec)
	{
		if(thread.isKilled())
		{
			break;
		}
		if(minValidSamples > 0 && collector.validCount() >= minValidSamples)
		{
			break;
		}
		uSleep(5);
	}

	if(!thread.isKilled())
	{
		thread.kill();
	}
	thread.join(true);
	// Remove handler before snapshot. removeHandler does not block in-flight
	// dispatches, so take the snapshot under the collector's mutex to avoid a
	// race with a still-running dispatch posting one final event.
	UEventsManager::removeHandler(&collector);
	return collector.snapshot();
}

} // namespace

TEST(IMUThreadTest, InitFailsOnMissingFile)
{
	IMUThread thread(0, Transform::getIdentity());
	EXPECT_FALSE(thread.init("/tmp/rtabmap_imuthread_missing_file.csv"));
}

TEST(IMUThreadTest, InitFailsOnHeaderOnly)
{
	const std::string path = tempImuCsvPath();
	ASSERT_TRUE(writeImuCsv(path, std::vector<std::string>()));

	IMUThread thread(0, Transform::getIdentity());
	EXPECT_FALSE(thread.init(path));
	UFile::erase(path);
}

TEST(IMUThreadTest, InitSucceedsWithValidFile)
{
	const std::string path = tempImuCsvPath();
	ASSERT_TRUE(writeImuCsv(path, {"1.0,0,0,0,0,0,9.81"}));

	IMUThread thread(0, Transform::getIdentity());
	EXPECT_TRUE(thread.init(path));
	UFile::erase(path);
}

TEST(IMUThreadTest, PublishesSamplesFromCsv)
{
	const std::string path = tempImuCsvPath();
	// Equal stamps avoid captureDelay busy-wait when rate is 0.
	ASSERT_TRUE(writeImuCsv(path, {
			"1.0,0.1,0.2,0.3,0.0,0.0,9.81",
			"1.0,0.2,0.3,0.4,0.0,0.0,9.81"}));

	IMUThread thread(0, Transform::getIdentity());
	ASSERT_TRUE(thread.init(path));

	const std::vector<IMUEventCollector::Sample> samples = runThread(thread, 2);
	ASSERT_GE(samples.size(), 2u);

	EXPECT_TRUE(samples[0].valid);
	EXPECT_NEAR(samples[0].stamp, 1.0, 1e-6);
	expectVec3Near(samples[0].data.angularVelocity(), cv::Vec3d(0.1, 0.2, 0.3));
	expectVec3Near(samples[0].data.linearAcceleration(), cv::Vec3d(0.0, 0.0, 9.81));

	EXPECT_TRUE(samples[1].valid);
	EXPECT_NEAR(samples[1].stamp, 1.0, 1e-6);

	// End-of-file posts an invalid/empty event then kills the thread.
	EXPECT_FALSE(samples.back().valid);

	UFile::erase(path);
}

TEST(IMUThreadTest, PublishesEurocStampsInSeconds)
{
	// EuRoC IMU CSV: integer timestamp = seconds * 1e9 + nanoseconds (no '.').
	// 10.5 s  -> "10500000000",  10.6 s -> "10600000000"
	const std::string path = tempImuCsvPath();
	ASSERT_TRUE(writeImuCsv(path, {
			"10500000000,0.1,0.2,0.3,0.0,0.0,9.81",
			"10600000000,0.2,0.3,0.4,0.0,0.0,9.81"}));

	IMUThread thread(0, Transform::getIdentity());
	ASSERT_TRUE(thread.init(path));

	const std::vector<IMUEventCollector::Sample> samples = runThread(thread, 2);
	ASSERT_GE(samples.size(), 2u);

	EXPECT_TRUE(samples[0].valid);
	EXPECT_NEAR(samples[0].stamp, 10.5, 1e-9);
	EXPECT_TRUE(samples[1].valid);
	EXPECT_NEAR(samples[1].stamp, 10.6, 1e-9);

	UFile::erase(path);
}

TEST(IMUThreadTest, EnableFilteringSetsOrientation)
{
	const std::string path = tempImuCsvPath();
	ASSERT_TRUE(writeImuCsv(path, {
			"0.0,0,0,0,0,0,9.81",
			"0.01,0,0,0,0,0,9.81",
			"0.02,0,0,0,0,0,9.81",
			"0.03,0,0,0,0,0,9.81",
			"0.04,0,0,0,0,0,9.81"}));

	IMUThread thread(0, Transform::getIdentity());
	ASSERT_TRUE(thread.init(path));
	thread.enableIMUFiltering(IMUFilter::kComplementaryFilter);

	const std::vector<IMUEventCollector::Sample> samples = runThread(thread, 3);
	ASSERT_FALSE(samples.empty());

	bool foundOrientation = false;
	for(int i = static_cast<int>(samples.size()) - 1; i >= 0; --i)
	{
		if(samples[i].valid && orientationSet(samples[i].data.orientation()))
		{
			foundOrientation = true;
			const cv::Vec4d & q = samples[i].data.orientation();
			const double norm = std::sqrt(
					q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
			EXPECT_NEAR(norm, 1.0, 0.05);
			// Static gravity, zero gyro: filter should stay near identity.
			expectQuatNear(q, 0, 0, 0, 1, 0.05);
			break;
		}
	}
	EXPECT_TRUE(foundOrientation);

	UFile::erase(path);
}

TEST(IMUThreadTest, DisableFilteringLeavesOrientationUnset)
{
	const std::string path = tempImuCsvPath();
	ASSERT_TRUE(writeImuCsv(path, {"1.0,0.1,0.2,0.3,0.0,0.0,9.81"}));

	IMUThread thread(0, Transform::getIdentity());
	ASSERT_TRUE(thread.init(path));
	thread.enableIMUFiltering(IMUFilter::kComplementaryFilter);
	thread.disableIMUFiltering();

	const std::vector<IMUEventCollector::Sample> samples = runThread(thread, 1);
	ASSERT_GE(samples.size(), 1u);
	EXPECT_TRUE(samples[0].valid);
	EXPECT_FALSE(orientationSet(samples[0].data.orientation()));

	UFile::erase(path);
}

TEST(IMUThreadTest, StoresLocalTransformWithoutConvertingAcceleration)
{
	// Without IMU filtering, localTransform is only attached; acc/gyro are not rotated.
	const std::string path = tempImuCsvPath();
	ASSERT_TRUE(writeImuCsv(path, {"1.0,0,0,0,0,0,9.81"}));

	const Transform local(0.1f, 0.2f, 0.3f, 0.f, 0.f, 0.5f);
	IMUThread thread(0, local);
	ASSERT_TRUE(thread.init(path));

	const std::vector<IMUEventCollector::Sample> samples = runThread(thread, 1);
	ASSERT_GE(samples.size(), 1u);
	EXPECT_TRUE(samples[0].valid);
	expectVec3Near(samples[0].data.linearAcceleration(), cv::Vec3d(0, 0, 9.81));
	EXPECT_FLOAT_EQ(samples[0].data.localTransform().x(), 0.1f);
	EXPECT_FLOAT_EQ(samples[0].data.localTransform().theta(), 0.5f);

	UFile::erase(path);
}

TEST(IMUThreadTest, BaseFrameConversionRotatesAcceleration)
{
	// With filtering and baseFrameConversion=true, IMUThread calls convertToBaseFrame()
	// before fusion (same rotation as IMU::convertToBaseFrame()).
	const std::string path = tempImuCsvPath();
	ASSERT_TRUE(writeImuCsv(path, {"1.0,0,0,0,1.0,0.0,0.0"}));

	const float halfPi = static_cast<float>(CV_PI / 2.0);
	const Transform local(0.f, 0.f, 0.f, 0.f, 0.f, halfPi);
	IMUThread thread(0, local);
	ASSERT_TRUE(thread.init(path));
	thread.enableIMUFiltering(IMUFilter::kComplementaryFilter, ParametersMap(), true);

	const std::vector<IMUEventCollector::Sample> samples = runThread(thread, 1);
	ASSERT_GE(samples.size(), 1u);
	EXPECT_TRUE(samples[0].valid);
	expectVec3Near(samples[0].data.linearAcceleration(), cv::Vec3d(0, 1, 0), 1e-4);
	EXPECT_NEAR(samples[0].data.localTransform().theta(), 0.0f, 1e-5f);

	UFile::erase(path);
}
