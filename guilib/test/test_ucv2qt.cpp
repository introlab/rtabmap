#include "gtest/gtest.h"
#include "rtabmap/utilite/UCv2Qt.h"
#include <limits>

namespace {

QVector<QRgb> grayTable(bool inverted)
{
	QVector<QRgb> table;
	for(int i = 0; i < 256; ++i)
	{
		int v = inverted?255-i:i;
		table.push_back(qRgb(v,v,v));
	}
	return table;
}

QVector<QRgb> hsvTable(bool reversed, bool blackFirst)
{
	QVector<QRgb> table;
	for(int i = 0; i < 256; ++i)
	{
		if(blackFirst && i == 0)
		{
			table.push_back(qRgb(0,0,0));
		}
		else
		{
			table.push_back(QColor::fromHsv(reversed?255-i:i, 255, 255, 255).rgb());
		}
	}
	return table;
}

std::vector<int> indices(const QImage & image)
{
	std::vector<int> values;
	for(int y = 0; y < image.height(); ++y)
	{
		for(int x = 0; x < image.width(); ++x)
		{
			values.push_back(image.pixelIndex(x, y));
		}
	}
	return values;
}

} // namespace

TEST(UCv2QtTest, EmptyImage)
{
	EXPECT_TRUE(uCvMat2QImage(cv::Mat()).isNull());
}

TEST(UCv2QtTest, UnsupportedFormat)
{
	EXPECT_TRUE(uCvMat2QImage(cv::Mat(2, 2, CV_8UC2, cv::Scalar::all(1))).isNull());
	EXPECT_TRUE(uCvMat2QImage(cv::Mat(2, 2, CV_32FC3, cv::Scalar::all(1))).isNull());
}

TEST(UCv2QtTest, BgrImage)
{
	cv::Mat image(2, 3, CV_8UC3);
	for(int y = 0; y < image.rows; ++y)
	{
		for(int x = 0; x < image.cols; ++x)
		{
			image.at<cv::Vec3b>(y, x) = cv::Vec3b(10*x+y, 100+x, 200+y); // b, g, r
		}
	}
	QImage qimage = uCvMat2QImage(image);
	ASSERT_EQ(QImage::Format_RGB32, qimage.format());
	ASSERT_EQ(3, qimage.width());
	ASSERT_EQ(2, qimage.height());
	for(int y = 0; y < image.rows; ++y)
	{
		for(int x = 0; x < image.cols; ++x)
		{
			EXPECT_EQ(qRgb(200+y, 100+x, 10*x+y), qimage.pixel(x, y)) << "x=" << x << " y=" << y;
		}
	}
}

TEST(UCv2QtTest, RgbImage)
{
	cv::Mat image(2, 3, CV_8UC3);
	for(int y = 0; y < image.rows; ++y)
	{
		for(int x = 0; x < image.cols; ++x)
		{
			image.at<cv::Vec3b>(y, x) = cv::Vec3b(10*x+y, 100+x, 200+y); // r, g, b
		}
	}
	QImage qimage = uCvMat2QImage(image, false);
	ASSERT_EQ(QImage::Format_RGB32, qimage.format());
	for(int y = 0; y < image.rows; ++y)
	{
		for(int x = 0; x < image.cols; ++x)
		{
			EXPECT_EQ(qRgb(10*x+y, 100+x, 200+y), qimage.pixel(x, y)) << "x=" << x << " y=" << y;
		}
	}
}

TEST(UCv2QtTest, GrayImage)
{
	cv::Mat image = cv::Mat_<unsigned char>({2, 3}, {0, 1, 2, 127, 128, 255});
	const uCvQtDepthColorMap maps[4] = {uCvQtDepthWhiteToBlack, uCvQtDepthBlackToWhite, uCvQtDepthRedToBlue, uCvQtDepthBlueToRed};
	const QVector<QRgb> tables[4] = {grayTable(true), grayTable(false), hsvTable(false, false), hsvTable(true, false)};
	for(int i = 0; i < 4; ++i)
	{
		QImage qimage = uCvMat2QImage(image, true, maps[i]);
		ASSERT_EQ(QImage::Format_Indexed8, qimage.format());
		EXPECT_EQ(std::vector<int>({0, 1, 2, 127, 128, 255}), indices(qimage)) << "colorMap=" << i;
		EXPECT_EQ(tables[i], qimage.colorTable()) << "colorMap=" << i;
	}
}

TEST(UCv2QtTest, FloatDepthAutoRange)
{
	// meters, 0 and NaN are invalid
	cv::Mat depth = cv::Mat_<float>({1, 5}, {1.0f, 0.0f, std::numeric_limits<float>::quiet_NaN(), 2.0f, 3.0f});

	// min=1, max=3: index = 255-(d-min)*255/(max-min) clamped to [1,255]
	QImage qimage = uCvMat2QImage(depth, true, uCvQtDepthWhiteToBlack);
	ASSERT_EQ(QImage::Format_Indexed8, qimage.format());
	EXPECT_EQ(std::vector<int>({255, 0, 0, 127, 1}), indices(qimage));
	EXPECT_EQ(grayTable(false), qimage.colorTable());

	// BlackToWhite and RedToBlue invert the valid indices (256-index)
	qimage = uCvMat2QImage(depth, true, uCvQtDepthBlackToWhite);
	EXPECT_EQ(std::vector<int>({1, 0, 0, 129, 255}), indices(qimage));
	EXPECT_EQ(grayTable(false), qimage.colorTable());

	qimage = uCvMat2QImage(depth, true, uCvQtDepthRedToBlue);
	EXPECT_EQ(std::vector<int>({1, 0, 0, 129, 255}), indices(qimage));
	EXPECT_EQ(hsvTable(false, true), qimage.colorTable());

	qimage = uCvMat2QImage(depth, true, uCvQtDepthBlueToRed);
	EXPECT_EQ(std::vector<int>({255, 0, 0, 127, 1}), indices(qimage));
	EXPECT_EQ(hsvTable(false, true), qimage.colorTable());
}

TEST(UCv2QtTest, FloatDepthFixedRange)
{
	cv::Mat depth = cv::Mat_<float>({1, 4}, {0.0f, 1.0f, 2.0f, 3.0f});
	// below min -> 255, above max -> 1
	QImage qimage = uCvMat2QImage(depth, true, uCvQtDepthWhiteToBlack, 1.5f, 2.5f);
	EXPECT_EQ(std::vector<int>({0, 255, 127, 1}), indices(qimage));
}

// The automatic range is computed only on valid pixels, whatever the first pixel is.
TEST(UCv2QtTest, FloatDepthAutoRangeInvalidFirstPixel)
{
	cv::Mat depth = cv::Mat_<float>({1, 4}, {0.0f, 1.0f, 2.0f, 3.0f});
	EXPECT_EQ(std::vector<int>({0, 255, 127, 1}), indices(uCvMat2QImage(depth, true, uCvQtDepthWhiteToBlack)));
	depth = cv::Mat_<float>({1, 4}, {std::numeric_limits<float>::quiet_NaN(), 1.0f, 2.0f, 3.0f});
	EXPECT_EQ(std::vector<int>({0, 255, 127, 1}), indices(uCvMat2QImage(depth, true, uCvQtDepthWhiteToBlack)));
	// no valid pixels
	depth = cv::Mat_<float>({1, 2}, {0.0f, std::numeric_limits<float>::quiet_NaN()});
	EXPECT_EQ(std::vector<int>({0, 0}), indices(uCvMat2QImage(depth, true, uCvQtDepthWhiteToBlack)));
}

TEST(UCv2QtTest, ShortDepthAutoRange)
{
	// millimeters, 0 is invalid
	cv::Mat depth = cv::Mat_<unsigned short>({1, 4}, {1000, 0, 2000, 3000});
	QImage qimage = uCvMat2QImage(depth, true, uCvQtDepthWhiteToBlack);
	ASSERT_EQ(QImage::Format_Indexed8, qimage.format());
	EXPECT_EQ(std::vector<int>({255, 0, 127, 1}), indices(qimage));
	EXPECT_EQ(grayTable(false), qimage.colorTable());

	qimage = uCvMat2QImage(depth, true, uCvQtDepthBlackToWhite);
	EXPECT_EQ(std::vector<int>({1, 0, 129, 255}), indices(qimage));
}

TEST(UCv2QtTest, ShortDepthAutoRangeInvalidFirstPixel)
{
	cv::Mat depth = cv::Mat_<unsigned short>({1, 4}, {0, 1000, 2000, 3000});
	EXPECT_EQ(std::vector<int>({0, 255, 127, 1}), indices(uCvMat2QImage(depth, true, uCvQtDepthWhiteToBlack)));
	// no valid pixels
	depth = cv::Mat::zeros(1, 2, CV_16UC1);
	EXPECT_EQ(std::vector<int>({0, 0}), indices(uCvMat2QImage(depth, true, uCvQtDepthWhiteToBlack)));
}

TEST(UCv2QtTest, ShortDepthFixedRange)
{
	cv::Mat depth = cv::Mat_<unsigned short>({1, 4}, {0, 1000, 2000, 3000});
	// range in meters
	QImage qimage = uCvMat2QImage(depth, true, uCvQtDepthWhiteToBlack, 1.5f, 2.5f);
	EXPECT_EQ(std::vector<int>({0, 255, 127, 1}), indices(qimage));
}

// Images that are a region of a larger image (rows not contiguous) should
// give the same result as their contiguous copy.
TEST(UCv2QtTest, RoiBgrImage)
{
	cv::Mat image(6, 7, CV_8UC3);
	cv::randu(image, 0, 256);
	cv::Mat roi = image(cv::Rect(1, 2, 4, 3));
	ASSERT_FALSE(roi.isContinuous());
	EXPECT_EQ(uCvMat2QImage(roi.clone()), uCvMat2QImage(roi));
}

TEST(UCv2QtTest, RoiGrayImage)
{
	cv::Mat image(6, 7, CV_8UC1);
	cv::randu(image, 0, 256);
	cv::Mat roi = image(cv::Rect(1, 2, 4, 3));
	ASSERT_FALSE(roi.isContinuous());
	EXPECT_EQ(indices(uCvMat2QImage(roi.clone())), indices(uCvMat2QImage(roi)));
}

TEST(UCv2QtTest, RoiFloatDepth)
{
	// values outside the region must not change its depth range
	cv::Mat depth(6, 7, CV_32FC1, cv::Scalar(10.0f));
	cv::Mat roi = depth(cv::Rect(1, 2, 4, 3));
	cv::randu(roi, 1.0f, 3.0f);
	ASSERT_FALSE(roi.isContinuous());
	EXPECT_EQ(indices(uCvMat2QImage(roi.clone())), indices(uCvMat2QImage(roi)));
}

TEST(UCv2QtTest, RoiShortDepth)
{
	cv::Mat depth(6, 7, CV_16UC1, cv::Scalar(10000));
	cv::Mat roi = depth(cv::Rect(1, 2, 4, 3));
	cv::randu(roi, 1000, 3000);
	ASSERT_FALSE(roi.isContinuous());
	EXPECT_EQ(indices(uCvMat2QImage(roi.clone())), indices(uCvMat2QImage(roi)));
}
