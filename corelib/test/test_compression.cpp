#include <gtest/gtest.h>
#include <rtabmap/core/Compression.h>
#include <opencv2/core.hpp>
#include <cstring>
#include <limits>

using namespace rtabmap;

namespace {

void expectMatEqual(const cv::Mat & a, const cv::Mat & b)
{
	ASSERT_FALSE(a.empty());
	ASSERT_FALSE(b.empty());
	ASSERT_EQ(a.rows, b.rows);
	ASSERT_EQ(a.cols, b.cols);
	ASSERT_EQ(a.type(), b.type());
	ASSERT_EQ(a.channels(), b.channels());
	if(a.depth() == CV_32F)
	{
		for(int r = 0; r < a.rows; ++r)
		{
			for(int c = 0; c < a.cols; ++c)
			{
				EXPECT_NEAR(a.at<float>(r, c), b.at<float>(r, c), 1e-5f);
			}
		}
	}
	else if(a.channels() == 1)
	{
		EXPECT_EQ(cv::countNonZero(a != b), 0);
	}
	else
	{
		EXPECT_EQ(cv::norm(a, b, cv::NORM_INF), 0);
	}
}

} // namespace

TEST(CompressionTest, CompressImagePngRoundTrip)
{
	const cv::Mat image = (cv::Mat_<uchar>(2, 3) << 10, 20, 30, 40, 50, 60);
	const std::vector<unsigned char> bytes = compressImage(image, ".png");
	ASSERT_FALSE(bytes.empty());
	EXPECT_EQ(compressedDepthFormat(bytes), ".png");

	const cv::Mat restored = uncompressImage(bytes);
	expectMatEqual(restored, image);
}

TEST(CompressionTest, CompressImage2AndVectorOverloadMatch)
{
	const cv::Mat image = cv::Mat::ones(8, 8, CV_8UC1) * 127;
	const cv::Mat bytesMat = compressImage2(image, ".png");
	const std::vector<unsigned char> bytesVec = compressImage(image, ".png");

	ASSERT_FALSE(bytesMat.empty());
	ASSERT_EQ(bytesMat.type(), CV_8UC1);
	ASSERT_EQ(bytesVec.size(), static_cast<size_t>(bytesMat.cols));

	const cv::Mat restoredFromMat = uncompressImage(bytesMat);
	const cv::Mat restoredFromVec = uncompressImage(bytesVec);
	expectMatEqual(restoredFromMat, image);
	expectMatEqual(restoredFromVec, image);
}

TEST(CompressionTest, CompressImage2AndVectorOverloadMatchRgb)
{
	cv::Mat image(16, 16, CV_8UC3);
	for(int r = 0; r < image.rows; ++r)
	{
		for(int c = 0; c < image.cols; ++c)
		{
			image.at<cv::Vec3b>(r, c) = cv::Vec3b(
					static_cast<uchar>(r * 10),
					static_cast<uchar>(c * 10),
					static_cast<uchar>((r + c) * 5));
		}
	}

	const cv::Mat bytesMat = compressImage2(image, ".png");
	const std::vector<unsigned char> bytesVec = compressImage(image, ".png");

	ASSERT_FALSE(bytesMat.empty());
	ASSERT_EQ(bytesMat.type(), CV_8UC1);
	ASSERT_EQ(bytesVec.size(), static_cast<size_t>(bytesMat.cols));
	EXPECT_LT(bytesMat.total() * bytesMat.elemSize(), image.total() * image.elemSize());

	const cv::Mat restoredFromMat = uncompressImage(bytesMat);
	const cv::Mat restoredFromVec = uncompressImage(bytesVec);
	expectMatEqual(restoredFromMat, image);
	expectMatEqual(restoredFromVec, image);
}

TEST(CompressionTest, CompressImageRvlRoundTrip)
{
	cv::Mat depth(4, 5, CV_16UC1);
	for(int r = 0; r < depth.rows; ++r)
	{
		for(int c = 0; c < depth.cols; ++c)
		{
			depth.at<uint16_t>(r, c) = static_cast<uint16_t>(1000 + r * 10 + c);
		}
	}

	const cv::Mat bytes = compressImage2(depth, ".rvl");
	ASSERT_FALSE(bytes.empty());
	EXPECT_EQ(compressedDepthFormat(bytes), ".rvl");

	const cv::Mat restored = uncompressImage(bytes);
	expectMatEqual(restored, depth);
}

TEST(CompressionTest, CompressDataRoundTrip)
{
	const cv::Mat data = (cv::Mat_<float>(2, 2) << 1.f, 2.f, 3.f, 4.f);
	const std::vector<unsigned char> bytes = compressData(data);
	ASSERT_FALSE(bytes.empty());

	const cv::Mat restored = uncompressData(bytes);
	expectMatEqual(restored, data);
}

TEST(CompressionTest, CompressData2RoundTrip)
{
	const cv::Mat data = (cv::Mat_<double>(1, 4) << 1.0, -2.0, 3.5, 4.25);
	const cv::Mat bytes = compressData2(data);
	ASSERT_FALSE(bytes.empty());
	ASSERT_EQ(bytes.type(), CV_8UC1);

	const cv::Mat restored = uncompressData(bytes);
	expectMatEqual(restored, data);
}

TEST(CompressionTest, CompressStringRoundTrip)
{
	const std::string text = "rtabmap compression test";
	const cv::Mat bytes = compressString(text);
	ASSERT_FALSE(bytes.empty());

	EXPECT_EQ(uncompressString(bytes), text);
}

TEST(CompressionTest, EmptyInputReturnsEmptyOutput)
{
	EXPECT_TRUE(compressImage(cv::Mat(), ".png").empty());
	EXPECT_TRUE(compressImage2(cv::Mat(), ".png").empty());
	EXPECT_TRUE(compressData(cv::Mat()).empty());
	EXPECT_TRUE(compressData2(cv::Mat()).empty());
	EXPECT_TRUE(uncompressImage(cv::Mat()).empty());
	EXPECT_TRUE(uncompressData(cv::Mat()).empty());
	EXPECT_TRUE(compressedDepthFormat(cv::Mat()).empty());
	EXPECT_EQ(uncompressString(cv::Mat()), "");
}

TEST(CompressionTest, CompressionThreadUncompressImage)
{
	const cv::Mat image = cv::Mat::ones(16, 16, CV_8UC1) * 200;
	const cv::Mat compressed = compressImage2(image, ".png");
	ASSERT_FALSE(compressed.empty());
	EXPECT_LT(compressed.total() * compressed.elemSize(), image.total() * image.elemSize());

	CompressionThread uncompressThread(compressed, true);
	uncompressThread.start();
	uncompressThread.join();

	expectMatEqual(uncompressThread.getUncompressedData(), image);
}

TEST(CompressionTest, CompressionThreadDataRoundTrip)
{
	const cv::Mat data = (cv::Mat_<int>(2, 3) << 1, 2, 3, 4, 5, 6);

	CompressionThread compressThread(data);
	compressThread.start();
	compressThread.join();

	const cv::Mat compressed = compressThread.getCompressedData();
	ASSERT_FALSE(compressed.empty());

	CompressionThread uncompressThread(compressed, false);
	uncompressThread.start();
	uncompressThread.join();

	expectMatEqual(uncompressThread.getUncompressedData(), data);
}

namespace {

// 32FC1 depth image covering [minDepth, maxDepth[ with sub-millimeter values,
// and the invalid values of the inverse depth format on the first row.
cv::Mat makeFloatDepth(int rows, int cols, float minDepth, float maxDepth)
{
	cv::Mat depth(rows, cols, CV_32FC1);
	for(int r = 0; r < rows; ++r)
	{
		for(int c = 0; c < cols; ++c)
		{
			depth.at<float>(r, c) = minDepth + (maxDepth - minDepth) * float(r * cols + c) / float(rows * cols);
		}
	}
	return depth;
}

// Error bound of the inverse depth format: half a quantization step.
float invDepthTolerance(float d, float quantization)
{
	// (with some margin for the float rounding of A/d + B, up to ~66000)
	return 0.51f * d * d / (quantization * (quantization + 1.0f)) + 1e-6f;
}

} // namespace

TEST(CompressionTest, ParseImageCompressionFormat)
{
	std::string codec;
	float maxDepth, quantization;

	EXPECT_TRUE(parseImageCompressionFormat("", codec, maxDepth, quantization));
	EXPECT_TRUE(codec.empty());
	EXPECT_EQ(maxDepth, 0.0f);

	EXPECT_TRUE(parseImageCompressionFormat(".jpg", codec, maxDepth, quantization));
	EXPECT_EQ(codec, ".jpg");
	EXPECT_EQ(maxDepth, 0.0f);
	EXPECT_EQ(quantization, 0.0f);

	EXPECT_TRUE(parseImageCompressionFormat(".rvl", codec, maxDepth, quantization));
	EXPECT_EQ(codec, ".rvl");
	EXPECT_EQ(maxDepth, 0.0f);

	EXPECT_TRUE(parseImageCompressionFormat(".png:20", codec, maxDepth, quantization));
	EXPECT_EQ(codec, ".png");
	EXPECT_FLOAT_EQ(maxDepth, 20.0f);
	EXPECT_FLOAT_EQ(quantization, 100.0f);

	EXPECT_TRUE(parseImageCompressionFormat(".rvl:10.5:50", codec, maxDepth, quantization));
	EXPECT_EQ(codec, ".rvl");
	EXPECT_FLOAT_EQ(maxDepth, 10.5f);
	EXPECT_FLOAT_EQ(quantization, 50.0f);

	EXPECT_FALSE(parseImageCompressionFormat("png", codec, maxDepth, quantization));
	EXPECT_FALSE(parseImageCompressionFormat(".jpg:10:100", codec, maxDepth, quantization));
	EXPECT_FALSE(parseImageCompressionFormat(".png:abc", codec, maxDepth, quantization));
	EXPECT_FALSE(parseImageCompressionFormat(".png:0:100", codec, maxDepth, quantization));
	EXPECT_FALSE(parseImageCompressionFormat(".png:-10:100", codec, maxDepth, quantization));
	EXPECT_FALSE(parseImageCompressionFormat(".png:10:0", codec, maxDepth, quantization));
	EXPECT_FALSE(parseImageCompressionFormat(".png:10:100:1", codec, maxDepth, quantization));
}

TEST(CompressionTest, InvalidFormatReturnsEmpty)
{
	const cv::Mat depth = makeFloatDepth(4, 4, 1.0f, 2.0f);
	EXPECT_TRUE(compressImage(depth, ".jpg:10").empty());
	EXPECT_TRUE(compressImage(depth, ".png:x").empty());
}

TEST(CompressionTest, InverseDepthRoundTrip)
{
	const float maxDepth = 10.0f;
	const float quantization = 100.0f;
	const float minDepth = quantization * (quantization + 1.0f) / (65535.0f + quantization * (quantization + 1.0f) / maxDepth);
	cv::Mat depth = makeFloatDepth(48, 64, minDepth * 1.001f, maxDepth * 0.999f);
	const float invalid[] = {
			0.0f, -1.0f, maxDepth, maxDepth * 2.0f, minDepth * 0.9f,
			std::numeric_limits<float>::quiet_NaN(),
			std::numeric_limits<float>::infinity(),
			-std::numeric_limits<float>::infinity()};
	const int nInvalid = sizeof(invalid) / sizeof(float);
	for(int i = 0; i < nInvalid; ++i)
	{
		depth.at<float>(0, i) = invalid[i];
	}

	for(const std::string codec : {".png", ".rvl"})
	{
		SCOPED_TRACE(codec);
		const std::string format = codec + ":10:100";
		const std::vector<unsigned char> bytes = compressImage(depth, format);
		ASSERT_FALSE(bytes.empty());
		EXPECT_LT(bytes.size(), depth.total() * depth.elemSize() / 2);
		EXPECT_EQ(compressedDepthFormat(bytes), format);

		const cv::Mat restored = uncompressImage(bytes);
		ASSERT_EQ(restored.type(), CV_32FC1);
		ASSERT_EQ(restored.size(), depth.size());
		for(int r = 0; r < depth.rows; ++r)
		{
			for(int c = 0; c < depth.cols; ++c)
			{
				const float d = depth.at<float>(r, c);
				if(r == 0 && c < nInvalid)
				{
					EXPECT_EQ(restored.at<float>(r, c), 0.0f) << "input=" << d;
				}
				else
				{
					ASSERT_NEAR(restored.at<float>(r, c), d, invDepthTolerance(d, quantization)) << "r=" << r << " c=" << c;
				}
			}
		}

		// Re-compressing with the detected format gives back the same bytes
		// (e.g., DatabaseViewer saving an edited depth image).
		EXPECT_EQ(compressImage(restored, compressedDepthFormat(bytes)), compressImage(restored, format));

		// Same through cv::Mat and thread overloads
		CompressionThread compressThread(depth, format);
		compressThread.start();
		compressThread.join();
		const cv::Mat bytesMat = compressThread.getCompressedData();
		ASSERT_EQ(bytesMat.total(), bytes.size());
		EXPECT_EQ(memcmp(bytesMat.data, bytes.data(), bytes.size()), 0);
		CompressionThread uncompressThread(bytesMat, true);
		uncompressThread.start();
		uncompressThread.join();
		expectMatEqual(uncompressThread.getUncompressedData(), restored);
	}
}

TEST(CompressionTest, InverseDepthQuantizationParameters)
{
	const cv::Mat depth = makeFloatDepth(32, 32, 1.0f, 39.0f);
	const std::vector<unsigned char> bytes = compressImage(depth, ".png:40:50");
	EXPECT_EQ(compressedDepthFormat(bytes), ".png:40:50");
	const cv::Mat restored = uncompressImage(bytes);
	ASSERT_EQ(restored.type(), CV_32FC1);
	for(int r = 0; r < depth.rows; ++r)
	{
		for(int c = 0; c < depth.cols; ++c)
		{
			const float d = depth.at<float>(r, c);
			ASSERT_NEAR(restored.at<float>(r, c), d, invDepthTolerance(d, 50.0f));
		}
	}
}

TEST(CompressionTest, InverseDepthNonContinuousImage)
{
	const cv::Mat depth = makeFloatDepth(20, 30, 1.0f, 5.0f);
	const cv::Mat roi = depth(cv::Rect(3, 2, 10, 8));
	ASSERT_FALSE(roi.isContinuous());
	const cv::Mat restored = uncompressImage(compressImage(roi, ".rvl:10:100"));
	ASSERT_EQ(restored.size(), roi.size());
	for(int r = 0; r < roi.rows; ++r)
	{
		for(int c = 0; c < roi.cols; ++c)
		{
			const float d = roi.at<float>(r, c);
			ASSERT_NEAR(restored.at<float>(r, c), d, invDepthTolerance(d, 100.0f));
		}
	}
}

TEST(CompressionTest, DepthParametersIgnoredFor16UC1)
{
	cv::Mat depth(24, 32, CV_16UC1);
	cv::randu(depth, 0, 20000); // includes values over the max depth below
	for(const std::string codec : {".png", ".rvl"})
	{
		SCOPED_TRACE(codec);
		const std::vector<unsigned char> bytes = compressImage(depth, codec + ":10:100");
		EXPECT_EQ(bytes, compressImage(depth, codec));
		EXPECT_EQ(compressedDepthFormat(bytes), codec);
		expectMatEqual(uncompressImage(bytes), depth);
	}
}

TEST(CompressionTest, LegacyFloatDepthIsLossless)
{
	const cv::Mat depth = makeFloatDepth(16, 16, 0.01f, 100.0f);
	for(const std::string format : {".png", ".rvl"})
	{
		SCOPED_TRACE(format);
		const std::vector<unsigned char> bytes = compressImage(depth, format);
		EXPECT_EQ(compressedDepthFormat(bytes), ".png");
		const cv::Mat restored = uncompressImage(bytes);
		ASSERT_EQ(restored.type(), CV_32FC1);
		EXPECT_EQ(memcmp(restored.data, depth.data, depth.total() * depth.elemSize()), 0);
	}
}
