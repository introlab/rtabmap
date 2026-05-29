// Tests for PyMatcher: rtabmap's bridge to Python-backed descriptor matchers
// (SuperGlue, OANet, etc.). PyMatcher is an internal class (not in the public
// include tree), so the test pulls in its private header via the corelib/src
// include path wired up in CMakeLists.txt.
//
// We stand in a tiny numpy-only stub script for what would otherwise be a
// heavy ML model. The whole file is a no-op when rtabmap is built without
// Python -- the CMakeLists.txt only registers it under
// WITH_PYTHON AND Python3_FOUND, and a defensive #ifdef matches that.

#include <gtest/gtest.h>
#include <rtabmap/core/Version.h>

#ifdef RTABMAP_PYTHON

#include "python/PyMatcher.h"  // private header (see CMakeLists.txt include dirs)

#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>

#include <opencv2/core.hpp>

#include <fstream>
#include <memory>
#include <string>
#ifdef _WIN32
#include <process.h>
#define getpid _getpid
#else
#include <unistd.h>
#endif

using namespace rtabmap;

namespace {

// Drops a minimal matcher script implementing the contract PyMatcher.cpp
// expects:
//   init(descriptorDim, matchThreshold, iterations, cuda, model) -- called once
//   match(kptsFrom, kptsTo, scoresFrom, scoresTo, descriptorsFrom,
//         descriptorsTo, imageWidth, imageHeight)
//     -> returns Nx2 int32 (queryIdx, trainIdx) pairs.
//
// Our stub returns a perfect 1:1 mapping for min(rowsFrom, rowsTo) rows
// (i -> i), echoing the count so the test can pin both the pipeline plumbing
// and the array-shape parsing in PyMatcher::match. Numpy-only, no heavy deps.
//
// Each test uses a unique filename via `tag` so the Python module cache
// reloads fresh content on each TEST().
std::string writeStubScript(int tag)
{
	const std::string path = uFormat(
			"/tmp/rtabmap_test_pymatcher_%d_%d.py", getpid(), tag);
	std::ofstream out(path);
	out <<
		"import numpy as np\n"
		"\n"
		"INITIALIZED = False\n"
		"INIT_DIM = 0\n"
		"INIT_THRESHOLD = 0.0\n"
		"INIT_ITERATIONS = 0\n"
		"INIT_CUDA = 0\n"
		"INIT_MODEL = ''\n"
		"\n"
		"def init(descriptorDim, matchThreshold, iterations, cuda, model):\n"
		"    global INITIALIZED, INIT_DIM, INIT_THRESHOLD, INIT_ITERATIONS\n"
		"    global INIT_CUDA, INIT_MODEL\n"
		"    INITIALIZED = True\n"
		"    INIT_DIM = int(descriptorDim)\n"
		"    INIT_THRESHOLD = float(matchThreshold)\n"
		"    INIT_ITERATIONS = int(iterations)\n"
		"    INIT_CUDA = int(cuda)\n"
		"    INIT_MODEL = str(model)\n"
		"\n"
		"def match(kptsFrom, kptsTo, scoresFrom, scoresTo,\n"
		"          descriptorsFrom, descriptorsTo, imageWidth, imageHeight):\n"
		"    nFrom = descriptorsFrom.shape[0]\n"
		"    nTo = descriptorsTo.shape[0]\n"
		"    n = min(nFrom, nTo)\n"
		"    # Perfect 1:1 mapping: query i <-> train i.\n"
		"    matches = np.zeros((n, 2), dtype=np.int32)\n"
		"    for i in range(n):\n"
		"        matches[i, 0] = i\n"
		"        matches[i, 1] = i\n"
		"    return matches\n";
	return path;
}

// Build N descriptors of dimension dim. Cell (i, j) = i + 0.01*j so the test
// can identify them, though the stub doesn't actually use the content.
cv::Mat makeDescriptors(int n, int dim)
{
	cv::Mat d(n, dim, CV_32FC1);
	for(int i = 0; i < n; ++i)
		for(int j = 0; j < dim; ++j)
			d.at<float>(i, j) = float(i) + 0.01f * float(j);
	return d;
}

std::vector<cv::KeyPoint> makeKeypoints(int n)
{
	std::vector<cv::KeyPoint> kpts;
	kpts.reserve(n);
	for(int i = 0; i < n; ++i)
	{
		kpts.emplace_back(float(i * 4), float(i * 4), 8.0f, -1.0f, 0.5f);
	}
	return kpts;
}

}  // namespace

// Full happy path: script loads, init() receives all 5 args, match() returns
// a 1:1 mapping that survives back into cv::DMatch entries.
TEST(PyMatcher, BasicMatching)
{
	const std::string scriptPath = writeStubScript(1);
	const int dim = 8;
	PyMatcher matcher(scriptPath,
			/*matchThreshold=*/0.2f,
			/*iterations=*/20,
			/*cuda=*/false,
			/*model=*/"indoor");
	EXPECT_EQ(scriptPath, matcher.path());
	EXPECT_FLOAT_EQ(0.2f, matcher.matchThreshold());
	EXPECT_EQ(20, matcher.iterations());
	EXPECT_FALSE(matcher.cuda());
	EXPECT_EQ("indoor", matcher.model());

	cv::Mat descFrom = makeDescriptors(3, dim);
	cv::Mat descTo = makeDescriptors(3, dim);
	std::vector<cv::KeyPoint> kptsFrom = makeKeypoints(3);
	std::vector<cv::KeyPoint> kptsTo = makeKeypoints(3);

	std::vector<cv::DMatch> matches = matcher.match(
			descFrom, descTo, kptsFrom, kptsTo, cv::Size(640, 480));
	ASSERT_EQ(3u, matches.size());
	for(int i = 0; i < 3; ++i)
	{
		EXPECT_EQ(i, matches[i].queryIdx);
		EXPECT_EQ(i, matches[i].trainIdx);
	}

	UFile::erase(scriptPath);
}

// Asymmetric descriptor counts: stub returns min(nFrom, nTo) matches. Pin
// that the C++ parser handles non-square Nx2 arrays correctly.
TEST(PyMatcher, AsymmetricCounts)
{
	const std::string scriptPath = writeStubScript(2);
	const int dim = 4;
	PyMatcher matcher(scriptPath, 0.2f, 20, false, "indoor");

	cv::Mat descFrom = makeDescriptors(5, dim);
	cv::Mat descTo = makeDescriptors(3, dim);
	std::vector<cv::KeyPoint> kptsFrom = makeKeypoints(5);
	std::vector<cv::KeyPoint> kptsTo = makeKeypoints(3);

	std::vector<cv::DMatch> matches = matcher.match(
			descFrom, descTo, kptsFrom, kptsTo, cv::Size(320, 240));
	EXPECT_EQ(3u, matches.size());  // limited by the smaller side

	UFile::erase(scriptPath);
}

// A non-existent script path is pre-validated by the constructor; match()
// then short-circuits because pModule_ is null.
TEST(PyMatcher, MissingPathReturnsEmpty)
{
	const std::string scriptPath = uFormat(
			"/tmp/rtabmap_test_pymatcher_does_not_exist_%d.py", getpid());
	PyMatcher matcher(scriptPath, 0.2f, 20, false, "indoor");

	cv::Mat descFrom = makeDescriptors(2, 8);
	cv::Mat descTo = makeDescriptors(2, 8);
	std::vector<cv::KeyPoint> kptsFrom = makeKeypoints(2);
	std::vector<cv::KeyPoint> kptsTo = makeKeypoints(2);

	std::vector<cv::DMatch> matches = matcher.match(
			descFrom, descTo, kptsFrom, kptsTo, cv::Size(64, 64));
	EXPECT_TRUE(matches.empty());
}

// match() requires same descriptor dim on both sides; mismatched cols hit
// the "Invalid inputs" guard and return empty.
TEST(PyMatcher, MismatchedDescriptorDimReturnsEmpty)
{
	const std::string scriptPath = writeStubScript(3);
	PyMatcher matcher(scriptPath, 0.2f, 20, false, "indoor");

	cv::Mat descFrom = makeDescriptors(2, 8);
	cv::Mat descTo = makeDescriptors(2, 16);
	std::vector<cv::KeyPoint> kptsFrom = makeKeypoints(2);
	std::vector<cv::KeyPoint> kptsTo = makeKeypoints(2);

	std::vector<cv::DMatch> matches = matcher.match(
			descFrom, descTo, kptsFrom, kptsTo, cv::Size(64, 64));
	EXPECT_TRUE(matches.empty());

	UFile::erase(scriptPath);
}

// match() also rejects non-CV_32F descriptors -- the input-guard in
// PyMatcher::match insists on float descriptors.
TEST(PyMatcher, NonFloatDescriptorsReturnEmpty)
{
	const std::string scriptPath = writeStubScript(4);
	PyMatcher matcher(scriptPath, 0.2f, 20, false, "indoor");

	cv::Mat descFrom = cv::Mat::zeros(2, 8, CV_8UC1);
	cv::Mat descTo = cv::Mat::zeros(2, 8, CV_8UC1);
	std::vector<cv::KeyPoint> kptsFrom = makeKeypoints(2);
	std::vector<cv::KeyPoint> kptsTo = makeKeypoints(2);

	std::vector<cv::DMatch> matches = matcher.match(
			descFrom, descTo, kptsFrom, kptsTo, cv::Size(64, 64));
	EXPECT_TRUE(matches.empty());

	UFile::erase(scriptPath);
}

// Zero-area imageSize is rejected by the input guard.
TEST(PyMatcher, ZeroImageSizeReturnsEmpty)
{
	const std::string scriptPath = writeStubScript(5);
	PyMatcher matcher(scriptPath, 0.2f, 20, false, "indoor");

	cv::Mat descFrom = makeDescriptors(2, 8);
	cv::Mat descTo = makeDescriptors(2, 8);
	std::vector<cv::KeyPoint> kptsFrom = makeKeypoints(2);
	std::vector<cv::KeyPoint> kptsTo = makeKeypoints(2);

	std::vector<cv::DMatch> matches = matcher.match(
			descFrom, descTo, kptsFrom, kptsTo, cv::Size(0, 0));
	EXPECT_TRUE(matches.empty());

	UFile::erase(scriptPath);
}

#endif  // RTABMAP_PYTHON
