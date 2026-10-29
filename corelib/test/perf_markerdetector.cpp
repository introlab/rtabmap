// Bad orientations of small or nearly frontal square markers.
//
// The orientation of a marker comes from the perspective distortion of its corners: on
// a marker small in the image, the noise on the corners is large relative to it, and
// the detector can return a wrong orientation. A square marker seen small or nearly
// frontal also has two poses, mirrored about the line of sight, reprojecting its corners
// almost equally well, so the detector can return the "flipped" one. This shows up to
// which distance (Marker/MaxRange) and from which tilt the orientations can be trusted
// (otherwise, see RGBD/LandmarkMaxNormalError and Marker/VarianceOrientationIgnored).
//
//   RangeTiltSweep: markers rendered from 175 to 21 pixels wide (0.45 to 3.75 m for 15 cm
//   markers), tilted from 0 to 85 deg by 5 deg, with blur, detected by OpenCV, the AprilTag
//   library, and the AprilTag library with OpenCV's PnP (MarkerAprilTag/OpenCVPnP). It
//   reports, per detector, tilt and distance, how many markers are detected with a good
//   pose and how many with a bad pose (more than 10 deg off).
//
// Its own executable, run by ctest under the "performance" label:
//   ctest -L performance    to run them
//   ctest -LE performance   to skip them
//   RTABMAP_PERF_MARKER_SIZE=0.1 bin/test_markerdetector_perf   for 10 cm markers (15 cm by default)
//   RTABMAP_PERF_IMAGE_WIDTH=1280 RTABMAP_PERF_IMAGE_HEIGHT=720 RTABMAP_PERF_FOCAL=910 bin/test_markerdetector_perf
//                                 for another camera (640x480 and 525 pixels by default)
//
// The sweep is reported rather than asserted on: the bad poses depend on the noise of
// the corners, so on the detector and OpenCV versions.
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

// Widths (pixels) of the frontal markers in the image of the sweeps, the corresponding
// distances are given by distanceForWidth()
const std::vector<float> kWidths = {175.0f, 131.0f, 105.0f, 88.0f, 75.0f, 66.0f, 58.0f, 53.0f, 44.0f, 38.0f, 33.0f, 29.0f, 26.0f, 21.0f};

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

TEST(MarkerDetectorPerf, RangeTiltSweep)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	const std::vector<Config> configs = sweepConfigs();
	const CameraModel model = testCameraModel();

	// Per config, tilt and width: good and bad detections
	struct Detected { int good=0, bad=0; };
	std::map<std::string, std::map<int, std::map<float, Detected> > > results;
	std::map<int, int> imagesPerTilt; // per width

	// Per tilt and width: blur of 1, 1.5 and 2 pixels, about the horizontal and vertical
	// axes (only once at 0 deg), the marker center jittered 4 times by up to half a pixel,
	// otherwise the results depend on where the corners fall in the pixels (e.g., all odd
	// widths would have their corners on pixel centers).
	std::mt19937 rng(42);
	std::uniform_real_distribution<float> jitter(-0.5f, 0.5f);
	for(int tiltDeg=0; tiltDeg<=85; tiltDeg+=5)
	for(float width : kWidths)
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
		const Transform cameraToMarker = sweepPose(distanceForWidth(width), tiltDeg, verticalAxis, offsetX, offsetY);
		const cv::Mat image = renderMarker(cameraToMarker, blur);
		if(width == kWidths.front())
		{
			++imagesPerTilt[tiltDeg];
		}

		for(const Config & c : configs)
		{
			MarkerDetector detector(sweepParams(c));
			std::map<int, MarkerInfo> detections = detector.detect(image, model);
			if(detections.count(0) == 0)
			{
				continue;
			}
			Detected & detected = results[c.name][tiltDeg][width];
			if(rotationErrorDeg(cameraToMarker, detections.at(0).pose()) > 10.0f)
			{
				++detected.bad;
			}
			else
			{
				++detected.good;
			}
		}
	}

	printf("\n%s\nGood and bad detections per tilt and distance, over %d images per tilt and distance (blur 1-2 px,\n"
			"about the horizontal and vertical axes, center jittered 4 times by +-0.5 px, half the images at 0 deg).\n"
			"Each cell: number of good detections, number of bad ones (more than 10 deg off), \"-\" if none is bad.\n",
			setupDescription().c_str(), imagesPerTilt.rbegin()->second);
	for(const Config & c : configs)
	{
		printf("\n  %s\n  %9s", c.name.c_str(), "Range (m)");
		for(float width : kWidths)
		{
			printf(" | %7.2f", distanceForWidth(width));
		}
		printf("\n  %9s", "Width");
		for(float width : kWidths)
		{
			printf(" | %4.0f px", width);
		}
		printf("\n");
		for(const auto & t : imagesPerTilt)
		{
			printf("  %5d deg", t.first);
			for(float width : kWidths)
			{
				const Detected & detected = results[c.name][t.first][width];
				printf(" | %3d %3s", detected.good, detected.bad ? uNumber2Str(detected.bad).c_str() : "-");
			}
			printf("\n");
		}
		EXPECT_FALSE(results[c.name].empty()) << c.name << " detected nothing";
	}
#endif
}
