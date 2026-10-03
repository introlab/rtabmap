// Comparison of the depth image compression approaches of Compression.h, for each
// depth type rtabmap receives:
//
//   16UC1 (millimeters):
//     - ".png"                  lossless, 16 bits grayscale PNG
//     - ".rvl"                  lossless, RVL (Mem/DepthCompressionFormat default)
//     - zlib                    lossless, compressData2(), as a reference
//   32FC1 (meters):
//     - ".png"                  lossless, float bytes as a 4-channel 8 bits PNG (legacy)
//     - zlib                    lossless, compressData2(), as a reference
//     - 16UC1 mm + ".png/.rvl"  lossy, util2d::cvtDepthFromFloat() then 16 bits codec,
//                               what Mem/SaveDepth16Format=true does
//     - ".png:max:q/.rvl:max:q" lossy, 16 bits quantized inverse depth (same
//                               quantization than ROS's compressed_depth_image_transport)
//
// over the depth images of data/rgbd/depth (a structured light camera, millimeters),
// the same images converted to meters in 32FC1 (as many drivers publish them), and a
// synthetic 32FC1 image with continuous values, like stereo or lidar projected depth.
//
// Its own executable, run by ctest under the "performance" label, so that its seconds
// of benchmarking stay out of the unit test shards:
//   ctest -L performance    to run them
//   ctest -LE performance   to skip them
//   bin/test_compression_perf --gtest_filter=*Synthetic*
//
// The times are reported rather than asserted on, as they depend on the machine. What
// is asserted is that the lossless approaches give back the same image, and that the
// lossy ones stay within their error bounds for the depth range they keep.
#include <gtest/gtest.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

using namespace rtabmap;

namespace {

static const int ITERATIONS = 15;

struct Approach
{
	std::string name;
	std::function<std::vector<unsigned char>(const cv::Mat &)> encode;
	std::function<cv::Mat(const std::vector<unsigned char> &)> decode;
	bool lossless;
	float maxDepth;     // meters, lossy approaches only: depth kept under it
	float minDepth;     // meters, lossy approaches only: depth kept over it
	std::function<float(float)> tolerance; // meters, lossy approaches only, for a depth in meters
};

struct Result
{
	size_t bytes = 0;
	double encodeMs = 0.0;
	double decodeMs = 0.0;
	double maxError = 0.0;   // mm, over the depth range kept
	double rmse = 0.0;       // mm, over the depth range kept
	double lost = 0.0;       // % of the valid pixels set to 0
	int outOfTolerance = 0;  // pixels with an error over the tolerance
};

double median(std::vector<double> v)
{
	std::sort(v.begin(), v.end());
	return v[v.size()/2];
}

float toMeters(const cv::Mat & depth, int r, int c)
{
	return depth.type() == CV_16UC1 ? float(depth.at<uint16_t>(r, c)) * 0.001f : depth.at<float>(r, c);
}

Result run(const cv::Mat & depth, const Approach & approach)
{
	Result result;
	std::vector<unsigned char> bytes;
	cv::Mat restored;
	std::vector<double> encodeTimes, decodeTimes;
	for(int i=0; i<ITERATIONS; ++i)
	{
		UTimer timer;
		bytes = approach.encode(depth);
		encodeTimes.push_back(timer.restart() * 1000.0);
		restored = approach.decode(bytes);
		decodeTimes.push_back(timer.ticks() * 1000.0);
	}
	result.bytes = bytes.size();
	result.encodeMs = median(encodeTimes);
	result.decodeMs = median(decodeTimes);

	EXPECT_EQ(restored.size(), depth.size());
	EXPECT_EQ(restored.type(), approach.lossless ? depth.type() : restored.type());
	if(restored.size() != depth.size())
	{
		return result;
	}

	if(approach.lossless)
	{
		EXPECT_EQ(memcmp(restored.data, depth.data, depth.total()*depth.elemSize()), 0);
		return result;
	}

	int valid = 0, lost = 0, kept = 0;
	double sumSq = 0.0;
	for(int r=0; r<depth.rows; ++r)
	{
		for(int c=0; c<depth.cols; ++c)
		{
			const float d = toMeters(depth, r, c);
			if(!(std::isfinite(d) && d > 0.0f))
			{
				continue;
			}
			++valid;
			const float out = toMeters(restored, r, c);
			if(out == 0.0f)
			{
				++lost;
				// Only allowed outside the kept range
				if(d >= approach.minDepth && d < approach.maxDepth)
				{
					++result.outOfTolerance;
				}
				continue;
			}
			const double err = std::fabs(out - d);
			result.maxError = std::max(result.maxError, err*1000.0);
			sumSq += err*err*1e6;
			++kept;
			if(err > approach.tolerance(d))
			{
				++result.outOfTolerance;
			}
		}
	}
	result.rmse = kept ? std::sqrt(sumSq / kept) : 0.0;
	result.lost = valid ? 100.0 * lost / valid : 0.0;
	EXPECT_EQ(result.outOfTolerance, 0) << approach.name;
	return result;
}

void report(const std::string & title, const cv::Mat & depth, const std::vector<Approach> & approaches)
{
	const size_t raw = depth.total() * depth.elemSize();
	std::printf("\n%s: %dx%d %s, %zu bytes raw\n", title.c_str(), depth.cols, depth.rows,
			depth.type() == CV_16UC1 ? "16UC1" : "32FC1", raw);
	std::printf("  %-22s %10s %7s %10s %10s %11s %10s %8s\n",
			"approach", "bytes", "ratio", "encode ms", "decode ms", "max err mm", "rmse mm", "lost %");
	for(const Approach & approach : approaches)
	{
		SCOPED_TRACE(title + " " + approach.name);
		const Result r = run(depth, approach);
		if(approach.lossless)
		{
			std::printf("  %-22s %10zu %6.1fx %10.2f %10.2f %11s %10s %8s\n",
					approach.name.c_str(), r.bytes, double(raw)/double(r.bytes), r.encodeMs, r.decodeMs,
					"lossless", "-", "-");
		}
		else
		{
			std::printf("  %-22s %10zu %6.1fx %10.2f %10.2f %11.3f %10.3f %8.2f\n",
					approach.name.c_str(), r.bytes, double(raw)/double(r.bytes), r.encodeMs, r.decodeMs,
					r.maxError, r.rmse, r.lost);
		}
	}
	std::fflush(stdout);
}

std::vector<unsigned char> encode(const cv::Mat & depth, const std::string & format)
{
	return compressImage(depth, format);
}

cv::Mat decode(const std::vector<unsigned char> & bytes)
{
	return uncompressImage(bytes);
}

std::vector<unsigned char> encodeZlib(const cv::Mat & depth)
{
	return compressData(depth);
}

cv::Mat decodeZlib(const std::vector<unsigned char> & bytes)
{
	return uncompressData(bytes);
}

std::vector<Approach> approaches16U()
{
	using namespace std::placeholders;
	return {
		{".png", std::bind(encode, _1, ".png"), decode, true, 0, 0, nullptr},
		{".rvl", std::bind(encode, _1, ".rvl"), decode, true, 0, 0, nullptr},
		{"zlib", encodeZlib, decodeZlib, true, 0, 0, nullptr}};
}

Approach invDepth(const std::string & codec, float maxDepth, float quantization)
{
	using namespace std::placeholders;
	const float A = quantization * (quantization + 1.0f);
	const float B = 1.0f - A / maxDepth;
	const std::string format = uFormat("%s:%g:%g", codec.c_str(), maxDepth, quantization);
	return {format, std::bind(encode, _1, format), decode, false,
		maxDepth,
		A / (65535.0f - B) * 1.001f,
		[A](float d) { return 0.51f * d * d / A + 1e-6f; }};
}

Approach depth16(const std::string & codec)
{
	return {"16UC1 mm + " + codec,
		[codec](const cv::Mat & depth) { return compressImage(util2d::cvtDepthFromFloat(depth), codec); },
		[](const std::vector<unsigned char> & bytes) { return util2d::cvtDepthToFloat(uncompressImage(bytes)); },
		false,
		65.535f,
		0.0f,
		[](float) { return 0.001f + 1e-6f; }}; // truncated to millimeters
}

std::vector<Approach> approaches32F()
{
	using namespace std::placeholders;
	return {
		{".png (legacy RGBA)", std::bind(encode, _1, ".png"), decode, true, 0, 0, nullptr},
		{"zlib", encodeZlib, decodeZlib, true, 0, 0, nullptr},
		depth16(".png"),
		depth16(".rvl"),
		invDepth(".png", 10.0f, 100.0f),
		invDepth(".rvl", 10.0f, 100.0f),
		invDepth(".png", 40.0f, 100.0f),
		invDepth(".rvl", 40.0f, 100.0f),
		invDepth(".rvl", 40.0f, 200.0f)};
}

std::vector<cv::Mat> loadSampleDepths()
{
	std::vector<cv::Mat> depths;
	for(const std::string & name : {"17.png", "154.png"})
	{
		const std::string path = std::string(RTABMAP_TEST_DATA_ROOT) + "/rgbd/depth/" + name;
		cv::Mat depth = cv::imread(path, cv::IMREAD_UNCHANGED);
		if(depth.type() == CV_16UC1)
		{
			depths.push_back(depth);
		}
		else
		{
			std::printf("Cannot load 16UC1 depth image \"%s\", skipped.\n", path.c_str());
		}
	}
	return depths;
}

// Ground plane, walls and boxes seen by a 640x480 camera, with continuous
// values up to ~35 m, noise growing with depth (as stereo) and holes.
cv::Mat makeSyntheticDepth(int cols = 640, int rows = 480)
{
	cv::RNG rng(42);
	const float fx = 0.75f * cols, cx = cols / 2.0f, cy = rows / 2.0f;
	const float cameraHeight = 1.0f;
	cv::Mat depth(rows, cols, CV_32FC1);
	for(int v=0; v<rows; ++v)
	{
		for(int u=0; u<cols; ++u)
		{
			const float x = (u - cx) / fx; // ray direction, z = 1
			const float y = (v - cy) / fx;
			float d = 35.0f; // far wall
			if(y > 0.0f)
			{
				d = std::min(d, cameraHeight / y); // ground
			}
			if(x < 0.0f)
			{
				d = std::min(d, 3.0f / -x); // left wall, 3 m away
			}
			// boxes
			if(x > 0.05f && x < 0.25f && y > -0.1f && y < cameraHeight / 2.5f)
			{
				d = std::min(d, 2.5f - 1.5f * x);
			}
			if(x > -0.35f && x < -0.15f && y > -0.2f && y < cameraHeight / 12.0f)
			{
				d = std::min(d, 12.0f);
			}
			d += (float)rng.gaussian(0.002 * d * d); // stereo-like noise
			depth.at<float>(v, u) = d;
		}
	}
	// Holes
	for(int i=0; i<40; ++i)
	{
		const int u = rng.uniform(0, cols - 20), v = rng.uniform(0, rows - 20);
		depth(cv::Rect(u, v, rng.uniform(2, 20), rng.uniform(2, 20))).setTo(0.0f);
	}
	return depth;
}

} // namespace

TEST(CompressionPerf, SampleDepth16UC1)
{
	const std::vector<cv::Mat> depths = loadSampleDepths();
	if(depths.empty())
	{
		GTEST_SKIP() << "No sample depth images in " << RTABMAP_TEST_DATA_ROOT << "/rgbd/depth";
	}
	for(size_t i=0; i<depths.size(); ++i)
	{
		report(uFormat("Sample depth %d", (int)i), depths[i], approaches16U());
	}
}

TEST(CompressionPerf, SampleDepth32FC1)
{
	const std::vector<cv::Mat> depths = loadSampleDepths();
	if(depths.empty())
	{
		GTEST_SKIP() << "No sample depth images in " << RTABMAP_TEST_DATA_ROOT << "/rgbd/depth";
	}
	for(size_t i=0; i<depths.size(); ++i)
	{
		report(uFormat("Sample depth %d in meters", (int)i), util2d::cvtDepthToFloat(depths[i]), approaches32F());
	}
}

TEST(CompressionPerf, Synthetic32FC1)
{
	report("Synthetic continuous depth", makeSyntheticDepth(), approaches32F());
	report("Synthetic continuous depth HD", makeSyntheticDepth(1280, 720), approaches32F());
}
