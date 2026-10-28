// Bad orientations of small or ambiguous square markers.
//
// The orientation of a marker comes from the perspective distortion of its corners: on
// a marker small in the image, the noise on the corners is large relative to it, and
// the detector can return a wrong orientation. Marker/MinPixelWidth rejects the markers
// too small in the image. A square marker seen small or nearly frontal also has two
// poses, mirrored about the line of sight, reprojecting its corners almost equally well:
// MarkerDetector computes both solutions and, with Marker/MaxAmbiguity, rejects the
// detection when the ratio of their errors (returned/other) is over this threshold and
// their orientations differ by more than 1 sigma of Marker/VarianceAngular.
//
//   - MinPixelWidthSweep: markers rendered from 175 to 21 pixels wide (0.45 to 3.75 m for
//     15 cm markers), over tilt (0 to 85 deg, see sweepSamples()) and blur, detected by
//     OpenCV, the AprilTag library, and the AprilTag library with OpenCV's PnP
//     (MarkerAprilTag/OpenCVPnP). It reports, per distance, how many markers are detected
//     and how many have a bad pose (more than 10 deg off) without the filters. For each
//     Marker/MinPixelWidth (with the corresponding range), it reports how many bad poses
//     are rejected, and how many good poses are rejected with them, by the min width
//     alone and combined with Marker/MaxAmbiguity=0.1, 0.3 and 0.5.
//
//   - AmbiguitySweep: the same markers. For each distance, it reports how many detections
//     are kept and how many of them have a bad pose, for each Marker/MaxAmbiguity threshold.
//
//   - TiltSweep: markers at a fixed distance (1.5 m by default), over tilt from 0 to 85
//     deg by 5 deg and blur. It reports, per tilt, how many markers are detected and how
//     many have a bad pose without the filters.
//
// Its own executable, run by ctest under the "performance" label:
//   ctest -L performance    to run them
//   ctest -LE performance   to skip them
//   bin/test_markerdetector_perf --gtest_filter=*MinPixelWidth*
//   RTABMAP_PERF_MARKER_SIZE=0.1 bin/test_markerdetector_perf   for 10 cm markers (15 cm by default)
//   RTABMAP_PERF_IMAGE_WIDTH=1280 RTABMAP_PERF_IMAGE_HEIGHT=720 RTABMAP_PERF_FOCAL=910 bin/test_markerdetector_perf
//                                 for another camera (640x480 and 525 pixels by default)
//   RTABMAP_PERF_MARKER_DISTANCE=2 bin/test_markerdetector_perf --gtest_filter=*Tilt*
//                                 for the tilt sweep at 2 m (1.5 m by default)
//   RTABMAP_PERF_MAX_AMBIGUITY=0.3 bin/test_markerdetector_perf --gtest_filter=*Tilt*
//                                 for the tilt sweep with the ambiguity check (disabled by default)
//
// The sweeps are reported rather than asserted on: what a threshold catches depends on
// the noise of the corners, so on the detector and OpenCV versions.
#include <gtest/gtest.h>
#include <rtabmap/core/MarkerDetector.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Version.h> // RTABMAP_APRILTAG
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace rtabmap;

#if ((CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION==4 && CV_MINOR_VERSION >=7)) && defined(HAVE_OPENCV_OBJDETECT)) || defined(HAVE_OPENCV_ARUCO)
#define RTABMAP_TEST_HAVE_ARUCO
#endif

namespace {

const int    kDictionary  = 20; // DICT_APRILTAG_36h11, supported by both strategies

// Value of a positive environment variable, or the default value if it is not set
float envValue(const char * name, float defaultValue)
{
	const char * env = std::getenv(name);
	return env && uStr2Float(env) > 0.0f ? uStr2Float(env) : defaultValue;
}

// Side of the markers in the sweeps (m). The sweeps are over the width of the markers
// in the image, so the marker size only changes the corresponding distances.
float markerSize()
{
	static const float size = envValue("RTABMAP_PERF_MARKER_SIZE", 0.15f);
	return size;
}

// Camera: the markers are near the image center, so the resolution only limits the
// largest markers. At the same marker width in the image, a larger focal length
// means a smaller angular size, so less perspective distortion to recover the orientation.
int imageWidth()
{
	static const int width = (int)envValue("RTABMAP_PERF_IMAGE_WIDTH", 640);
	return width;
}
int imageHeight()
{
	static const int height = (int)envValue("RTABMAP_PERF_IMAGE_HEIGHT", 480);
	return height;
}
double focal()
{
	static const double f = envValue("RTABMAP_PERF_FOCAL", 525.0f);
	return f;
}

std::string setupDescription()
{
	return uFormat("Marker size: %g m (RTABMAP_PERF_MARKER_SIZE), camera: %dx%d (RTABMAP_PERF_IMAGE_WIDTH/HEIGHT), fx=fy=%g px (RTABMAP_PERF_FOCAL).",
			markerSize(), imageWidth(), imageHeight(), focal());
}

// Distance at which a frontal marker has this width (pixels) in the image
float distanceForWidth(float width)
{
	return focal()*markerSize()/width;
}

CameraModel testCameraModel()
{
	return CameraModel(
			focal(), focal(),
			imageWidth()/2.0, imageHeight()/2.0,
			Transform::getIdentity(),
			0.0,
			cv::Size(imageWidth(), imageHeight()));
}

float rotationErrorDeg(const Transform & a, const Transform & b)
{
	return a.getAngle(b) * 180.0f / float(M_PI);
}

#ifdef RTABMAP_TEST_HAVE_ARUCO
struct Config
{
	std::string name;
	int strategy;
	bool openCVPnP;
};

// The detectors compared, the OpenCV strategy first
std::vector<Config> sweepConfigs()
{
	std::vector<Config> configs;
	configs.push_back({"OpenCV", 0, false});
#ifdef RTABMAP_APRILTAG
	configs.push_back({"AprilTag", 1, false});
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 1)
	configs.push_back({"AprilTag+OpenCVPnP", 1, true});
#endif
#endif
	return configs;
}

ParametersMap sweepParams(const Config & c)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kMarkerLength(), uNumber2Str(markerSize())));
	params.insert(ParametersPair(Parameters::kMarkerStrategy(), uNumber2Str(c.strategy)));
	params.insert(ParametersPair(Parameters::kMarkerDictionary(), uNumber2Str(kDictionary)));
	params.insert(ParametersPair(Parameters::kMarkerAprilTagOpenCVPnP(), uBool2Str(c.openCVPnP)));
	return params;
}

// Marker facing the camera at `distance`, tilted about its horizontal or vertical axis,
// its center offset by (offsetX, offsetY) pixels from the image center.
Transform sweepPose(float distance, float tiltDeg, bool verticalAxis, float offsetX, float offsetY)
{
	const float tilt = tiltDeg*M_PI/180.0f;
	return Transform(
			1,  0,  0, offsetX*distance/focal(),
			0, -1,  0, offsetY*distance/focal(),
			0,  0, -1, distance) *
			(verticalAxis ? Transform(0, 0, 0, 0, tilt, 0) : Transform(0, 0, 0, tilt, 0, 0));
}

struct Sample
{
	float width; // of the frontal marker in the image (pixels)
	float distance;
	float blur;
	Transform cameraToMarker;
};

// Per marker width from 175 to 21 pixels: blur of 1, 1.5 and 2 pixels, tilt from 0 to 80
// deg by 10 deg about the horizontal axis and from 5 to 85 deg by 10 deg about the
// vertical axis, interleaved (54 images). The marker center
// is jittered by up to half a pixel, otherwise the results depend on where the corners
// fall in the pixels (e.g., all odd widths would have their corners on pixel centers).
std::vector<Sample> sweepSamples()
{
	std::mt19937 rng(42);
	std::uniform_real_distribution<float> jitter(-0.5f, 0.5f);
	std::vector<Sample> samples;
	for(float width : {175.0f, 131.0f, 105.0f, 88.0f, 75.0f, 66.0f, 58.0f, 53.0f, 44.0f, 38.0f, 33.0f, 29.0f, 26.0f, 21.0f})
	for(float blur : {1.0f, 1.5f, 2.0f})
	for(int tiltStep=0; tiltStep<=17; ++tiltStep)
	{
		const float tiltDeg = tiltStep*5.0f;
		const bool verticalAxis = tiltStep % 2 == 1;
		const float distance = distanceForWidth(width);
		const float offsetX = jitter(rng);
		const float offsetY = jitter(rng);
		samples.push_back({width, distance, blur, sweepPose(distance, tiltDeg, verticalAxis, offsetX, offsetY)});
	}
	return samples;
}

const char * kSamplesDescription = "blur 1-2 px, tilt 0-80 deg by 10 deg about the horizontal axis and 5-85 deg by 10 deg about the vertical axis, center jittered by +-0.5 px";

// The marker image (markerSize() side), rendered large then warped to its pose and blurred.
cv::Mat renderMarker(const Transform & cameraToMarker, float blur)
{
	cv::Mat marker;
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION==4 && CV_MINOR_VERSION >=7)
	cv::aruco::generateImageMarker(
			cv::aruco::getPredefinedDictionary(cv::aruco::PredefinedDictionaryType(kDictionary)), 0, 400, marker);
#else
	cv::aruco::drawMarker(
			cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(kDictionary)), 0, 400, marker);
#endif
	const cv::Point2f src[4] = {
			cv::Point2f(-0.5f, -0.5f), cv::Point2f(399.5f, -0.5f),
			cv::Point2f(399.5f, 399.5f), cv::Point2f(-0.5f, 399.5f)};
	const float markerCorners[4][2] = {{-0.5f, 0.5f}, {0.5f, 0.5f}, {0.5f, -0.5f}, {-0.5f, -0.5f}};
	const CameraModel model = testCameraModel();
	cv::Point2f dst[4];
	for(int i=0; i<4; ++i)
	{
		cv::Point3f p = util3d::transformPoint(cv::Point3f(markerCorners[i][0]*markerSize(), markerCorners[i][1]*markerSize(), 0), cameraToMarker);
		dst[i].x = model.fx()*p.x/p.z + model.cx();
		dst[i].y = model.fy()*p.y/p.z + model.cy();
	}
	cv::Mat image;
	cv::warpPerspective(marker, image, cv::getPerspectiveTransform(src, dst),
			cv::Size(imageWidth(), imageHeight()), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
	cv::GaussianBlur(image, image, cv::Size(0,0), blur);
	return image;
}
#endif

} // namespace

TEST(MarkerDetectorPerf, MinPixelWidthSweep)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	const std::vector<Config> configs = sweepConfigs();
	const std::vector<float> widths = {0.0f, 20.0f, 25.0f, 30.0f, 35.0f, 40.0f, 45.0f, 50.0f, 60.0f, 80.0f};
	// Ambiguity thresholds combined with the min width (0 = min width alone)
	const std::vector<float> maxAmbiguities = {0.0f, 0.1f, 0.3f, 0.5f};
	const CameraModel model = testCameraModel();

	// Per config and min width: detections and bad ones without the filters, and per
	// ambiguity threshold, the rejected bad and good ones.
	struct Counts { int n=0, bad=0; std::map<float, std::pair<int, int> > rejected; };
	std::map<std::string, std::map<float, Counts> > results;
	// Per config and distance: detections and bad ones without the filters
	struct Detected { int n=0, bad=0; };
	std::map<std::string, std::map<float, Detected> > detectedPerDistance;
	std::map<float, int> imagesPerDistance;

	for(const Sample & sample : sweepSamples())
	{
		const float distance = sample.distance;
		const Transform & cameraToMarker = sample.cameraToMarker;
		const cv::Mat image = renderMarker(cameraToMarker, sample.blur);
		++imagesPerDistance[distance];

		for(const Config & c : configs)
		{
			ParametersMap params = sweepParams(c);
			MarkerDetector detector(params);
			std::map<int, MarkerInfo> detections = detector.detect(image, model);
			if(detections.count(0) == 0)
			{
				continue;
			}
			const bool bad = rotationErrorDeg(cameraToMarker, detections.at(0).pose()) > 10.0f;
			Detected & detected = detectedPerDistance[c.name][distance];
			++detected.n;
			detected.bad += bad?1:0;

			for(float width : widths)
			{
				Counts & counts = results[c.name][width];
				++counts.n;
				counts.bad += bad?1:0;

				params[Parameters::kMarkerMinPixelWidth()] = uNumber2Str(width);
				for(float maxAmbiguity : maxAmbiguities)
				{
					params[Parameters::kMarkerMaxAmbiguity()] = uNumber2Str(maxAmbiguity);
					MarkerDetector filtered(params);
					const bool rejected = filtered.detect(image, model).count(0) == 0;
					std::pair<int, int> & r = counts.rejected[maxAmbiguity];
					r.first += rejected && bad ? 1:0;
					r.second += rejected && !bad ? 1:0;
				}
			}
		}
	}

	const int images = imagesPerDistance.begin()->second;
	printf("\n%s\nDetections and bad ones per distance without the filters, over %d images per distance\n"
			"(%s).\n"
			"Each cell: number of detections (out of %d, the others are not detected) and their %% of the images,\n"
			"number of bad ones (more than 10 deg off) and their %% of the detections, \"-\" if none is bad.\n",
			setupDescription().c_str(), images, kSamplesDescription, images);
	printf("  %9s %9s", "", "");
	for(const Config & c : configs)
	{
		printf(" | %-20s", c.name.c_str());
	}
	printf("\n  %9s %9s", "Range (m)", "Width");
	for(size_t i=0; i<configs.size(); ++i)
	{
		printf(" | %-20s", uFormat("(/%d)", images).c_str());
	}
	printf("\n");
	auto cell = [images](int n, int bad)
	{
		const std::string detected = uFormat("%3d %3.0f%%", n, 100.0*n/images);
		return bad ? uFormat("%s %3d (%3.0f%%)", detected.c_str(), bad, 100.0*bad/n) : uFormat("%s %3s %6s", detected.c_str(), "-", "");
	};
	for(const auto & d : imagesPerDistance)
	{
		printf("  %9.2f %6.0f px", d.first, focal()*markerSize()/d.first);
		for(const Config & c : configs)
		{
			const Detected & detected = detectedPerDistance[c.name][d.first];
			printf(" | %20s", cell(detected.n, detected.bad).c_str());
		}
		printf("\n");
	}

	printf("\n%s\nBad pose: more than 10 deg off without the filters (%s).\n", setupDescription().c_str(), kSamplesDescription);
	int totalImages = 0;
	for(const auto & d : imagesPerDistance)
	{
		totalImages += d.second;
	}
	printf("Range: distance at which a frontal marker has this width.\n"
			"n and bad: detections and bad ones without the filters, over the %d images of all distances (the same for all min widths).\n", totalImages);
	printf("Each group: bad and good detections rejected (and their %% of the bad and good ones) by %s alone,\n"
			"or combined with %s=x (+ x).\n", Parameters::kMarkerMinPixelWidth().c_str(), Parameters::kMarkerMaxAmbiguity().c_str());
	printf("  %-20s %9s %9s %5s %5s", "", "", "", "", "");
	for(float maxAmbiguity : maxAmbiguities)
	{
		printf(" | %-19s", maxAmbiguity>0.0f?uFormat("+ %.1f", maxAmbiguity).c_str():"alone");
	}
	printf("\n  %-20s %9s %9s %5s %5s", "Detector", "MinWidth", "Range (m)", "n", "bad");
	for(size_t i=0; i<maxAmbiguities.size(); ++i)
	{
		printf(" | %9s %9s", "bad rej.", "good rej.");
	}
	printf("\n");
	for(const Config & c : configs)
	{
		for(float width : widths)
		{
			Counts & counts = results[c.name][width];
			const int good = counts.n-counts.bad;
			printf("  %-20s %9s %9s %5d %5d",
					c.name.c_str(),
					width>0?uFormat("%.0f px", width).c_str():"off",
					width>0?uFormat("%.2f", distanceForWidth(width)).c_str():"-",
					counts.n, counts.bad);
			for(float maxAmbiguity : maxAmbiguities)
			{
				const std::pair<int, int> & r = counts.rejected[maxAmbiguity];
				printf(" | %4d %3.0f%% %4d %3.0f%%",
						r.first, counts.bad ? 100.0*r.first/counts.bad : 0.0,
						r.second, good ? 100.0*r.second/good : 0.0);
			}
			printf("\n");
		}
		EXPECT_GT(results[c.name][0.0f].n, 0) << c.name << " detected nothing";
	}
#endif
}

TEST(MarkerDetectorPerf, AmbiguitySweep)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	const std::vector<Config> configs = sweepConfigs();
	const std::vector<float> thresholds = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f};
	const CameraModel model = testCameraModel();

	// Per config, distance and threshold (0 = check disabled): detections kept and bad ones
	struct Kept { int n=0, bad=0; };
	std::map<std::string, std::map<float, std::map<float, Kept> > > results;
	std::map<float, int> imagesPerDistance;

	for(const Sample & sample : sweepSamples())
	{
		const cv::Mat image = renderMarker(sample.cameraToMarker, sample.blur);
		++imagesPerDistance[sample.distance];

		for(const Config & c : configs)
		{
			ParametersMap params = sweepParams(c);
			MarkerDetector detector(params);
			std::map<int, MarkerInfo> detections = detector.detect(image, model);
			if(detections.count(0) == 0)
			{
				continue;
			}
			const bool bad = rotationErrorDeg(sample.cameraToMarker, detections.at(0).pose()) > 10.0f;
			for(float threshold : thresholds)
			{
				bool kept = true;
				if(threshold > 0.0f)
				{
					params[Parameters::kMarkerMaxAmbiguity()] = uNumber2Str(threshold);
					MarkerDetector checked(params);
					kept = checked.detect(image, model).count(0) == 1;
				}
				if(kept)
				{
					Kept & k = results[c.name][sample.distance][threshold];
					++k.n;
					k.bad += bad?1:0;
				}
			}
		}
	}

	printf("\n%s\nDetections kept and bad ones (more than 10 deg off) per distance and %s threshold\n"
			"(off = check disabled), over %d images per distance (%s).\n"
			"Each cell: number of detections kept (out of %d, the others are not detected or rejected), number of bad ones\n"
			"and their %% of the detections kept, \"-\" if none is bad.\n",
			setupDescription().c_str(), Parameters::kMarkerMaxAmbiguity().c_str(), imagesPerDistance.begin()->second, kSamplesDescription,
			imagesPerDistance.begin()->second);
	for(const Config & c : configs)
	{
		printf("  %s\n  %9s %9s", c.name.c_str(), "Range (m)", "Width");
		for(float threshold : thresholds)
		{
			printf(" | %-12s", threshold>0.0f?uFormat("%.1f", threshold).c_str():"off");
		}
		printf("\n");
		for(const auto & d : imagesPerDistance)
		{
			printf("  %9.2f %6.0f px", d.first, focal()*markerSize()/d.first);
			for(float threshold : thresholds)
			{
				const Kept & k = results[c.name][d.first][threshold];
				printf(" | %12s", k.bad ? uFormat("%3d %3d %3.0f%%", k.n, k.bad, 100.0*k.bad/k.n).c_str() : uFormat("%3d %3s %4s", k.n, "-", "").c_str());
			}
			printf("\n");
		}
		EXPECT_FALSE(results[c.name].empty()) << c.name << " detected nothing";
	}
#endif
}

TEST(MarkerDetectorPerf, TiltSweep)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	const std::vector<Config> configs = sweepConfigs();
	const CameraModel model = testCameraModel();
	const float distance = envValue("RTABMAP_PERF_MARKER_DISTANCE", 1.5f);
	const float maxAmbiguity = envValue("RTABMAP_PERF_MAX_AMBIGUITY", 0.0f);

	// Per config and tilt: detections and bad ones without the filters
	struct Detected { int n=0, bad=0; };
	std::map<std::string, std::map<int, Detected> > results;
	std::map<int, int> imagesPerTilt;

	// Per tilt: blur of 1, 1.5 and 2 pixels, about the horizontal and vertical axes
	// (only once at 0 deg), the marker center jittered 4 times by up to half a pixel.
	std::mt19937 rng(42);
	std::uniform_real_distribution<float> jitter(-0.5f, 0.5f);
	for(int tiltDeg=0; tiltDeg<=85; tiltDeg+=5)
	for(float blur : {1.0f, 1.5f, 2.0f})
	for(int verticalAxis=0; verticalAxis<2; ++verticalAxis)
	for(int j=0; j<4; ++j)
	{
		if(tiltDeg == 0 && verticalAxis)
		{
			continue; // same as about the horizontal axis
		}
		const float offsetX = jitter(rng);
		const float offsetY = jitter(rng);
		const Transform cameraToMarker = sweepPose(distance, tiltDeg, verticalAxis, offsetX, offsetY);
		const cv::Mat image = renderMarker(cameraToMarker, blur);
		++imagesPerTilt[tiltDeg];

		for(const Config & c : configs)
		{
			ParametersMap params = sweepParams(c);
			params.insert(ParametersPair(Parameters::kMarkerMaxAmbiguity(), uNumber2Str(maxAmbiguity)));
			MarkerDetector detector(params);
			std::map<int, MarkerInfo> detections = detector.detect(image, model);
			if(detections.count(0) == 0)
			{
				continue;
			}
			Detected & detected = results[c.name][tiltDeg];
			++detected.n;
			detected.bad += rotationErrorDeg(cameraToMarker, detections.at(0).pose()) > 10.0f ? 1:0;
		}
	}

	printf("\n%s\nDetections and bad ones per tilt with %s=%g (RTABMAP_PERF_MAX_AMBIGUITY, 0 = disabled), at %g m (RTABMAP_PERF_MARKER_DISTANCE),\n"
			"%.0f px wide when frontal, over %d images per tilt (blur 1-2 px, about the horizontal and vertical axes,\n"
			"center jittered 4 times by +-0.5 px, half the images at 0 deg).\n"
			"Each cell: number of detections and their %% of the images, number of bad ones (more than 10 deg off)\n"
			"and their %% of the detections, \"-\" if none is bad.\n",
			setupDescription().c_str(), Parameters::kMarkerMaxAmbiguity().c_str(), maxAmbiguity, distance, focal()*markerSize()/distance, imagesPerTilt.rbegin()->second);
	printf("  %9s", "Tilt");
	for(const Config & c : configs)
	{
		printf(" | %-20s", c.name.c_str());
	}
	printf("\n");
	for(const auto & t : imagesPerTilt)
	{
		printf("  %5d deg", t.first);
		for(const Config & c : configs)
		{
			const Detected & detected = results[c.name][t.first];
			const std::string cell = uFormat("%3d %3.0f%%", detected.n, 100.0*detected.n/t.second);
			printf(" | %20s", detected.bad ?
					uFormat("%s %3d (%3.0f%%)", cell.c_str(), detected.bad, 100.0*detected.bad/detected.n).c_str() :
					uFormat("%s %3s %6s", cell.c_str(), "-", "").c_str());
		}
		printf("\n");
	}
	for(const Config & c : configs)
	{
		EXPECT_FALSE(results[c.name].empty()) << c.name << " detected nothing";
	}
#endif
}
