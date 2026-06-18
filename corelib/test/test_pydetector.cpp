// Tests for PyDetector: rtabmap's bridge to Python-backed local-feature
// detectors (SuperPoint, custom networks, etc.).
//
// We stand in a tiny numpy-only stub script for whatever the user would
// normally point PyDetector at, so the test exercises the full pipeline
// (script load -> init() -> detect() -> keypoint/descriptor parsing) without
// pulling in heavy ML model weights. The whole file is a no-op when rtabmap
// is built without Python -- the CMakeLists.txt only registers it when
// WITH_PYTHON AND Python3_FOUND, and a defensive #ifdef matches that.

#include <gtest/gtest.h>
#include <rtabmap/core/Version.h>

#ifdef RTABMAP_PYTHON

#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>

#include <opencv2/core.hpp>

#include "TestUtils.h"
#include <fstream>
#include <memory>
#include <string>

using namespace rtabmap;

namespace {

// Drops a minimal detector script implementing the contract PyDetector.cpp
// expects:
//   init(cuda)            -- called once
//   detect(imageBuffer)   -- returns (Nx3 float32 [x, y, response], NxDIM float32)
// Three hard-coded keypoints at (w/4, h/4), (w/2, h/2), (3w/4, 3h/4) with
// descending responses; descriptor rows filled with (row_index + 1) so the
// test can identify them. Numpy-only, no heavy deps.
//
// Each test uses a unique filename via `tag` so the Python module cache
// reloads fresh content on each TEST().
std::string writeStubScript(int tag)
{
	const std::string path = test::tempPath(uFormat("rtabmap_test_pydetector_%d_%d.py", test::getPid(), tag));
	std::ofstream out(path);
	out <<
		"import numpy as np\n"
		"\n"
		"INITIALIZED = False\n"
		"CUDA_ARG = None\n"
		"DESCRIPTOR_DIM = 8\n"
		"\n"
		"def init(cuda):\n"
		"    global INITIALIZED, CUDA_ARG\n"
		"    INITIALIZED = True\n"
		"    CUDA_ARG = int(cuda)\n"
		"\n"
		"def detect(image):\n"
		"    h, w = image.shape\n"
		"    pts = np.array([\n"
		"        [w * 0.25, h * 0.25, 0.9],\n"
		"        [w * 0.50, h * 0.50, 0.8],\n"
		"        [w * 0.75, h * 0.75, 0.7],\n"
		"    ], dtype=np.float32)\n"
		"    desc = np.zeros((3, DESCRIPTOR_DIM), dtype=np.float32)\n"
		"    for i in range(3):\n"
		"        desc[i, :] = float(i + 1)\n"
		"    return pts, desc\n";
	return path;
}

// Variant stubs to exercise the edge cases that previously triggered
// asserts in PyDetector / generateDescriptorsImpl. Each returns shapes
// that violate the (Nx3, NxDIM) contract that the happy path expects.

// Returns 3 keypoints but a (0,0) descriptor array -- the original
// SuperPoint-style failure: nDesc=0, dim=0, but nKpts>0.
std::string writeStubScriptEmptyDescriptors(int tag)
{
	const std::string path = test::tempPath(uFormat("rtabmap_test_pydetector_emptydesc_%d_%d.py", test::getPid(), tag));
	std::ofstream out(path);
	out <<
		"import numpy as np\n"
		"def init(cuda):\n"
		"    pass\n"
		"def detect(image):\n"
		"    h, w = image.shape\n"
		"    pts = np.array([\n"
		"        [w * 0.25, h * 0.25, 0.9],\n"
		"        [w * 0.50, h * 0.50, 0.8],\n"
		"        [w * 0.75, h * 0.75, 0.7],\n"
		"    ], dtype=np.float32)\n"
		"    desc = np.zeros((0, 0), dtype=np.float32)\n"
		"    return pts, desc\n";
	return path;
}

// 3 keypoints but only 2 descriptors -- mismatched row count.
std::string writeStubScriptMismatchedCounts(int tag)
{
	const std::string path = test::tempPath(uFormat("rtabmap_test_pydetector_mismatch_%d_%d.py", test::getPid(), tag));
	std::ofstream out(path);
	out <<
		"import numpy as np\n"
		"def init(cuda):\n"
		"    pass\n"
		"def detect(image):\n"
		"    h, w = image.shape\n"
		"    pts = np.array([\n"
		"        [w * 0.25, h * 0.25, 0.9],\n"
		"        [w * 0.50, h * 0.50, 0.8],\n"
		"        [w * 0.75, h * 0.75, 0.7],\n"
		"    ], dtype=np.float32)\n"
		"    desc = np.zeros((2, 8), dtype=np.float32)\n"
		"    return pts, desc\n";
	return path;
}

// Both arrays empty -- the well-behaved "I found nothing" return.
std::string writeStubScriptBothEmpty(int tag)
{
	const std::string path = test::tempPath(uFormat("rtabmap_test_pydetector_bothempty_%d_%d.py", test::getPid(), tag));
	std::ofstream out(path);
	out <<
		"import numpy as np\n"
		"def init(cuda):\n"
		"    pass\n"
		"def detect(image):\n"
		"    pts  = np.zeros((0, 3), dtype=np.float32)\n"
		"    desc = np.zeros((0, 8), dtype=np.float32)\n"
		"    return pts, desc\n";
	return path;
}

ParametersMap baseParams(const std::string & scriptPath)
{
	ParametersMap p;
	p.insert(ParametersPair(Parameters::kPyDetectorPath(), scriptPath));
	p.insert(ParametersPair(Parameters::kPyDetectorCuda(), "false"));
	p.insert(ParametersPair(Parameters::kKpMaxFeatures(), "100"));
	p.insert(ParametersPair(Parameters::kKpSSC(), "false"));
	return p;
}

cv::Mat makeImage(int rows = 64, int cols = 64)
{
	// PyDetector requires CV_8UC1; uniform content is fine for the stub
	// which ignores pixel values.
	return cv::Mat(rows, cols, CV_8UC1, cv::Scalar(0));
}

}  // namespace

// Full happy path: the script loads, returns 3 keypoints + descriptors, and
// the keypoint positions, responses, and descriptor rows survive end to end.
TEST(PyDetector, BasicDetection)
{
	const std::string scriptPath = writeStubScript(1);
	std::unique_ptr<Feature2D> detector(Feature2D::create(
			Feature2D::kFeaturePyDetector, baseParams(scriptPath)));
	ASSERT_NE(detector.get(), nullptr);
	EXPECT_EQ(Feature2D::kFeaturePyDetector, detector->getType());

	cv::Mat image = makeImage(64, 64);
	std::vector<cv::KeyPoint> kpts = detector->generateKeypoints(image);
	ASSERT_EQ(3u, kpts.size());

	// Stub points: (w/4, h/4), (w/2, h/2), (3w/4, 3h/4) with responses
	// 0.9, 0.8, 0.7.
	EXPECT_NEAR(16.0f, kpts[0].pt.x, 1e-3f);
	EXPECT_NEAR(16.0f, kpts[0].pt.y, 1e-3f);
	EXPECT_NEAR(32.0f, kpts[1].pt.x, 1e-3f);
	EXPECT_NEAR(48.0f, kpts[2].pt.x, 1e-3f);
	EXPECT_NEAR(0.9f, kpts[0].response, 1e-5f);
	EXPECT_NEAR(0.8f, kpts[1].response, 1e-5f);
	EXPECT_NEAR(0.7f, kpts[2].response, 1e-5f);

	cv::Mat descriptors = detector->generateDescriptors(image, kpts);
	EXPECT_EQ(3, descriptors.rows);
	EXPECT_EQ(8, descriptors.cols);
	EXPECT_EQ(CV_32FC1, descriptors.type());
	for(int r = 0; r < descriptors.rows; ++r)
	{
		EXPECT_NEAR(float(r + 1), descriptors.at<float>(r, 0), 1e-5f);
	}

	UFile::erase(scriptPath);
}

// PyDetector logs an error and silently returns no keypoints when the
// script path doesn't exist -- the constructor doesn't throw. Pin that
// contract so callers can safely construct without pre-checking the path.
TEST(PyDetector, MissingPathReturnsEmpty)
{
	const std::string scriptPath = test::tempPath(uFormat("rtabmap_test_pydetector_does_not_exist_%d.py", test::getPid()));
	std::unique_ptr<Feature2D> detector(Feature2D::create(
			Feature2D::kFeaturePyDetector, baseParams(scriptPath)));
	ASSERT_NE(detector.get(), nullptr);

	cv::Mat image = makeImage(32, 32);
	std::vector<cv::KeyPoint> kpts = detector->generateKeypoints(image);
	EXPECT_TRUE(kpts.empty());
}

// A non-empty mask removes keypoints whose (x, y) falls on a 0 pixel. The
// stub returns one keypoint at (48, 48); we mask everything from row/col 33
// onward so it gets dropped while the two earlier points survive.
TEST(PyDetector, MaskFiltersKeypoints)
{
	const std::string scriptPath = writeStubScript(2);
	std::unique_ptr<Feature2D> detector(Feature2D::create(
			Feature2D::kFeaturePyDetector, baseParams(scriptPath)));
	ASSERT_NE(detector.get(), nullptr);

	cv::Mat image = makeImage(64, 64);
	cv::Mat mask(image.size(), CV_8UC1, cv::Scalar(255));
	mask(cv::Rect(33, 33, mask.cols - 33, mask.rows - 33)).setTo(0);
	std::vector<cv::KeyPoint> kpts = detector->generateKeypoints(image, mask);
	EXPECT_EQ(2u, kpts.size());

	UFile::erase(scriptPath);
}

// Kp/MaxFeatures caps the returned keypoint list inside generateKeypointsImpl
// (PyDetector calls limitKeypoints with its own descriptor matrix at the end).
// The stub returns 3 keypoints; cap at 2 and verify the rest are dropped.
TEST(PyDetector, MaxFeaturesCap)
{
	const std::string scriptPath = writeStubScript(3);
	ParametersMap p = baseParams(scriptPath);
	p[Parameters::kKpMaxFeatures()] = "2";
	std::unique_ptr<Feature2D> detector(
			Feature2D::create(Feature2D::kFeaturePyDetector, p));
	ASSERT_NE(detector.get(), nullptr);

	cv::Mat image = makeImage(64, 64);
	std::vector<cv::KeyPoint> kpts = detector->generateKeypoints(image);
	EXPECT_EQ(2u, kpts.size());

	UFile::erase(scriptPath);
}

// Regression: a Python script returned non-empty (Nx3) keypoints but an
// empty (0x0) descriptor array (observed with SuperPoint when its descriptor
// head short-circuits). The old code wrote `UASSERT(nDesc = nKpts)` -- a
// typo that assigned instead of compared -- and then silently produced
// keypoints with no descriptors, tripping generateDescriptorsImpl's
// `keypoints.size() == descriptors_.rows` assert downstream. Contract now:
// detector logs and returns empty keypoints + empty descriptors.
TEST(PyDetector, EmptyDescriptorsWithKeypoints)
{
	const std::string scriptPath = writeStubScriptEmptyDescriptors(1);
	std::unique_ptr<Feature2D> detector(Feature2D::create(
			Feature2D::kFeaturePyDetector, baseParams(scriptPath)));
	ASSERT_NE(detector.get(), nullptr);

	cv::Mat image = makeImage(64, 64);
	std::vector<cv::KeyPoint> kpts = detector->generateKeypoints(image);
	EXPECT_TRUE(kpts.empty());

	cv::Mat descriptors = detector->generateDescriptors(image, kpts);
	EXPECT_EQ(0, descriptors.rows);

	UFile::erase(scriptPath);
}

// If the script returns mismatched row counts (e.g. 3 keypoints, 2
// descriptors), the previous code would silently walk past the descriptor
// buffer (UB read) using nDesc clobbered to nKpts. Now we detect the
// mismatch and return empty.
TEST(PyDetector, MismatchedKeypointAndDescriptorCounts)
{
	const std::string scriptPath = writeStubScriptMismatchedCounts(1);
	std::unique_ptr<Feature2D> detector(Feature2D::create(
			Feature2D::kFeaturePyDetector, baseParams(scriptPath)));
	ASSERT_NE(detector.get(), nullptr);

	cv::Mat image = makeImage(64, 64);
	std::vector<cv::KeyPoint> kpts = detector->generateKeypoints(image);
	EXPECT_TRUE(kpts.empty());

	cv::Mat descriptors = detector->generateDescriptors(image, kpts);
	EXPECT_EQ(0, descriptors.rows);
}

// A script that returns (0x3, 0x8) -- the polite "I found nothing" case.
// Must not assert and must produce an empty result.
TEST(PyDetector, BothArraysEmpty)
{
	const std::string scriptPath = writeStubScriptBothEmpty(1);
	std::unique_ptr<Feature2D> detector(Feature2D::create(
			Feature2D::kFeaturePyDetector, baseParams(scriptPath)));
	ASSERT_NE(detector.get(), nullptr);

	cv::Mat image = makeImage(64, 64);
	std::vector<cv::KeyPoint> kpts = detector->generateKeypoints(image);
	EXPECT_TRUE(kpts.empty());

	cv::Mat descriptors = detector->generateDescriptors(image, kpts);
	EXPECT_EQ(0, descriptors.rows);

	UFile::erase(scriptPath);
}

// Full-zero mask drops every keypoint via keep_kpt. The keypoint loop pushes
// nothing, but the descriptor loop still iterates over the python-returned
// rows -- previously this could leave keypoints.size() != descriptors_.rows
// if the keep_kpt logic and descriptor-read loop disagreed on what to skip.
// Verify both stay empty.
TEST(PyDetector, AllKeypointsMaskedOut)
{
	const std::string scriptPath = writeStubScript(4);
	std::unique_ptr<Feature2D> detector(Feature2D::create(
			Feature2D::kFeaturePyDetector, baseParams(scriptPath)));
	ASSERT_NE(detector.get(), nullptr);

	cv::Mat image = makeImage(64, 64);
	cv::Mat mask(image.size(), CV_8UC1, cv::Scalar(0));
	std::vector<cv::KeyPoint> kpts = detector->generateKeypoints(image, mask);
	EXPECT_TRUE(kpts.empty());

	cv::Mat descriptors = detector->generateDescriptors(image, kpts);
	EXPECT_EQ(0, descriptors.rows);

	UFile::erase(scriptPath);
}

#endif  // RTABMAP_PYTHON
