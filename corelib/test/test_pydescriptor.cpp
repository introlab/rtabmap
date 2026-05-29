// Tests for PyDescriptor: rtabmap's bridge to Python-backed global-image
// descriptors (NetVLAD and similar).
//
// We stand in a tiny numpy-only stub script for whatever a real user would
// point PyDescriptor at, so the test exercises the full pipeline (script
// load -> init(dim) -> extract(image) -> descriptor parsing) without pulling
// in heavy ML model weights. The whole file is a no-op when rtabmap is built
// without Python -- the CMakeLists.txt only registers it under
// WITH_PYTHON AND Python3_FOUND, and a defensive #ifdef matches that.

#include <gtest/gtest.h>
#include <rtabmap/core/Version.h>

#ifdef RTABMAP_PYTHON

#include <rtabmap/core/GlobalDescriptor.h>
#include <rtabmap/core/GlobalDescriptorExtractor.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>

#include <opencv2/core.hpp>

#include "TestUtils.h"
#include <fstream>
#include <memory>
#include <string>

using namespace rtabmap;

namespace {

// Drops a minimal descriptor script implementing the contract
// PyDescriptor.cpp expects:
//   init(descriptorDim)  -- called once
//   extract(image)       -- returns 1xDIM float32, image is an HxWxC uint8 array
// Echoes the dim that init() received plus the image dimensions into the
// first four descriptor cells so the test can pin them. Numpy-only, no heavy
// deps.
//
// Each test uses a unique filename via `tag` so the Python module cache
// reloads fresh content on each TEST().
std::string writeStubScript(int tag)
{
	const std::string path = test::tempPath(uFormat("rtabmap_test_pydescriptor_%d_%d.py", test::getPid(), tag));
	std::ofstream out(path);
	out <<
		"import numpy as np\n"
		"\n"
		"INITIALIZED = False\n"
		"DIM = 0\n"
		"\n"
		"def init(descriptorDim):\n"
		"    global INITIALIZED, DIM\n"
		"    INITIALIZED = True\n"
		"    DIM = int(descriptorDim)\n"
		"\n"
		"def extract(image):\n"
		"    h = image.shape[0]\n"
		"    w = image.shape[1]\n"
		"    c = image.shape[2] if image.ndim == 3 else 1\n"
		"    desc = np.zeros((1, DIM), dtype=np.float32)\n"
		"    if DIM >= 4:\n"
		"        desc[0, 0] = float(DIM)\n"
		"        desc[0, 1] = float(h)\n"
		"        desc[0, 2] = float(w)\n"
		"        desc[0, 3] = float(c)\n"
		"    return desc\n";
	return path;
}

ParametersMap baseParams(const std::string & scriptPath, int dim)
{
	ParametersMap p;
	p.insert(ParametersPair(Parameters::kPyDescriptorPath(), scriptPath));
	p.insert(ParametersPair(Parameters::kPyDescriptorDim(), uNumber2Str(dim)));
	return p;
}

cv::Mat makeImage(int rows = 32, int cols = 48)
{
	// PyDescriptor passes the raw image as HxWxC uint8; CV_8UC3 mirrors what
	// a real RGB camera would supply.
	return cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
}

}  // namespace

// Full happy path: script loads, init() receives the configured dim, extract
// returns a 1xDIM float descriptor whose first cells echo DIM + image shape.
TEST(PyDescriptor, BasicExtraction)
{
	const std::string scriptPath = writeStubScript(1);
	const int dim = 16;
	std::unique_ptr<GlobalDescriptorExtractor> extractor(
			GlobalDescriptorExtractor::create(
					GlobalDescriptorExtractor::kPyDescriptor,
					baseParams(scriptPath, dim)));
	ASSERT_NE(extractor.get(), nullptr);
	EXPECT_EQ(GlobalDescriptorExtractor::kPyDescriptor, extractor->getType());

	cv::Mat image = makeImage(32, 48);
	SensorData data(image);
	GlobalDescriptor descriptor = extractor->extract(data);

	// type=1 matches kPyDescriptor (set inside PyDescriptor::extract).
	EXPECT_EQ(1, descriptor.type());
	ASSERT_EQ(1, descriptor.data().rows);
	ASSERT_EQ(dim, descriptor.data().cols);
	EXPECT_EQ(CV_32FC1, descriptor.data().type());

	// Stub echoes (DIM, H, W, C) in cells [0..3].
	EXPECT_NEAR(float(dim), descriptor.data().at<float>(0, 0), 1e-5f);
	EXPECT_NEAR(32.0f,      descriptor.data().at<float>(0, 1), 1e-5f);
	EXPECT_NEAR(48.0f,      descriptor.data().at<float>(0, 2), 1e-5f);
	EXPECT_NEAR(3.0f,       descriptor.data().at<float>(0, 3), 1e-5f);
	// Trailing cells are zero-initialized.
	for(int j = 4; j < dim; ++j)
	{
		EXPECT_NEAR(0.0f, descriptor.data().at<float>(0, j), 1e-5f);
	}

	UFile::erase(scriptPath);
}

// Changing Kp/PyDescriptor/Dim must reach init() in the script so its returned
// descriptor width follows. We construct with one dim, then check the stub
// echoes that value back.
TEST(PyDescriptor, DimParameterReachesScript)
{
	const std::string scriptPath = writeStubScript(2);
	const int dim = 8;
	std::unique_ptr<GlobalDescriptorExtractor> extractor(
			GlobalDescriptorExtractor::create(
					GlobalDescriptorExtractor::kPyDescriptor,
					baseParams(scriptPath, dim)));
	ASSERT_NE(extractor.get(), nullptr);

	GlobalDescriptor descriptor = extractor->extract(SensorData(makeImage()));
	ASSERT_EQ(dim, descriptor.data().cols);
	EXPECT_NEAR(float(dim), descriptor.data().at<float>(0, 0), 1e-5f);

	UFile::erase(scriptPath);
}

// When no script path is configured (empty string), PyDescriptor early-exits
// in parseParameters() with a null module. extract() then returns an empty
// descriptor.
TEST(PyDescriptor, EmptyPathReturnsEmpty)
{
	std::unique_ptr<GlobalDescriptorExtractor> extractor(
			GlobalDescriptorExtractor::create(
					GlobalDescriptorExtractor::kPyDescriptor,
					baseParams(/*scriptPath=*/"", 16)));
	ASSERT_NE(extractor.get(), nullptr);

	GlobalDescriptor descriptor = extractor->extract(SensorData(makeImage()));
	// Default-constructed: type==-1, data empty.
	EXPECT_EQ(-1, descriptor.type());
	EXPECT_TRUE(descriptor.data().empty());
}

// A non-existent script path is now pre-validated in parseParameters() (it
// used to crash inside getPythonTraceback when PyImport_Import returned NULL
// without setting an exception). extractor stays valid and extract returns
// an empty descriptor.
TEST(PyDescriptor, MissingPathReturnsEmpty)
{
	const std::string scriptPath = test::tempPath(uFormat("rtabmap_test_pydescriptor_does_not_exist_%d.py", test::getPid()));
	std::unique_ptr<GlobalDescriptorExtractor> extractor(
			GlobalDescriptorExtractor::create(
					GlobalDescriptorExtractor::kPyDescriptor,
					baseParams(scriptPath, 16)));
	ASSERT_NE(extractor.get(), nullptr);

	GlobalDescriptor descriptor = extractor->extract(SensorData(makeImage()));
	EXPECT_EQ(-1, descriptor.type());
	EXPECT_TRUE(descriptor.data().empty());
}

// extract() requires a non-empty raw image; when given an empty SensorData
// it logs an error and returns an empty descriptor.
TEST(PyDescriptor, EmptyImageReturnsEmpty)
{
	const std::string scriptPath = writeStubScript(3);
	std::unique_ptr<GlobalDescriptorExtractor> extractor(
			GlobalDescriptorExtractor::create(
					GlobalDescriptorExtractor::kPyDescriptor,
					baseParams(scriptPath, 16)));
	ASSERT_NE(extractor.get(), nullptr);

	GlobalDescriptor descriptor = extractor->extract(SensorData());
	EXPECT_EQ(-1, descriptor.type());
	EXPECT_TRUE(descriptor.data().empty());

	UFile::erase(scriptPath);
}

#endif  // RTABMAP_PYTHON
