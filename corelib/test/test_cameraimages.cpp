

#include <gtest/gtest.h>
#include <rtabmap/core/camera/CameraImages.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/SensorCaptureInfo.h>
#include <rtabmap/core/Transform.h>
#include <opencv2/core.hpp>
#include <string>

using namespace rtabmap;

// ---------------------------------------------------------------------------
// CameraImages over a real image directory. The tests above use a mock capture
// to cover the SensorCapture base class (frame-rate throttling, local
// transform, info filling); this drives the concrete directory-scanning
// implementation over the 84 committed images in data/samples.
// ---------------------------------------------------------------------------


TEST(CameraImagesTest, ScansSampleDirectoryAndCapturesFrames)
{
	const std::string path = std::string(RTABMAP_TEST_DATA_ROOT) + "/samples";
	CameraImages camera(path);
	ASSERT_TRUE(camera.init()) << "could not scan " << path;
	// data/samples ships no calibration, so the model stays unnamed and
	// getSerial() (which returns the model name) is empty -- the two must agree.
	EXPECT_FALSE(camera.isCalibrated());
	EXPECT_TRUE(camera.getSerial().empty());

	// Take a handful rather than all 84: enough to exercise the scan/decode
	// loop and the frame ordering without paying for the whole directory.
	Transform previousStamp;
	int frames = 0;
	double lastStamp = -1.0;
	for(int i = 0; i < 5; ++i)
	{
		SensorCaptureInfo info;
		SensorData data = camera.takeData(&info);
		if(data.imageRaw().empty() && data.imageCompressed().empty())
		{
			break;
		}
		++frames;
		EXPECT_GT(data.imageRaw().cols, 0);
		EXPECT_GT(data.imageRaw().rows, 0);
		EXPECT_GT(data.stamp(), lastStamp) << "stamps must increase across frames";
		lastStamp = data.stamp();
	}
	EXPECT_EQ(5, frames) << "expected 5 frames from a directory of 84 images";
}

// setStartIndex()/setMaxFrames() are how callers replay a slice of a
// directory; without them the reader always starts at the first file.
TEST(CameraImagesTest, HonoursStartIndexAndMaxFrames)
{
	const std::string path = std::string(RTABMAP_TEST_DATA_ROOT) + "/samples";

	CameraImages fromStart(path);
	ASSERT_TRUE(fromStart.init());
	SensorData first = fromStart.takeData();
	ASSERT_FALSE(first.imageRaw().empty());

	CameraImages skipped(path);
	skipped.setStartIndex(10);
	skipped.setMaxFrames(2);
	ASSERT_TRUE(skipped.init());

	SensorData tenth = skipped.takeData();
	ASSERT_FALSE(tenth.imageRaw().empty()) << "start index 10 returned nothing";
	// Different file, so the decoded pixels should differ from frame 0.
	EXPECT_NE(0.0, cv::norm(first.imageRaw(), tenth.imageRaw(), cv::NORM_L1))
			<< "start index had no effect: same image as the first frame";

	EXPECT_FALSE(skipped.takeData().imageRaw().empty()) << "second of maxFrames=2 missing";
	EXPECT_TRUE(skipped.takeData().imageRaw().empty()) << "maxFrames=2 should stop after two frames";
}

// The default constructor takes no path; a caller uses setPath()/setStartIndex()
// before init(). Covered separately because the path constructor above runs a
// different initialiser list.
TEST(CameraImagesTest, DefaultConstructedIsUnconfigured)
{
	CameraImages camera;
	EXPECT_FALSE(camera.isCalibrated());
	EXPECT_TRUE(camera.getSerial().empty());
	// No directory yet, so scanning must fail rather than assert.
	EXPECT_FALSE(camera.init());

	camera.setPath(std::string(RTABMAP_TEST_DATA_ROOT) + "/samples");
	ASSERT_TRUE(camera.init()) << "init() failed after setPath()";
	EXPECT_FALSE(camera.takeData().imageRaw().empty());
}
