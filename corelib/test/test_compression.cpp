#include <gtest/gtest.h>
#include <rtabmap/core/Compression.h>
#include <opencv2/core.hpp>

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
