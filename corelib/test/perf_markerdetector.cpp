// Detection range and orientation accuracy of square markers per detector.
//
// The orientation of a marker comes from the perspective distortion of its corners: on
// a marker small in the image, the noise on the corners is large relative to it. A square
// marker seen small or nearly frontal also has two poses, mirrored about the line of sight,
// reprojecting its corners almost equally well, so the detector can return the "flipped"
// one if its corners are not accurate enough. This shows, for a camera and a marker size,
// up to which distance (Marker/MaxRange) and from which tilt the markers are detected and
// their orientations can be trusted.
//
// The markers are rendered at a higher resolution, then averaged over each pixel like a
// camera: with only point sampling, the edges of small markers are aliased and the
// detectors return a lot more bad orientations than on real images. Their contrast and
// noise are the ones measured on real images (see renderMarker()). Other causes of bad
// orientations on real images (e.g., motion blur, uneven lighting, partial occlusion) are
// not simulated, see RGBD/LandmarkMaxNormalError and Marker/VarianceOrientationIgnored for
// those.
//
//   RangeTiltSweep: markers rendered at 16 distances up to 6 m (with their width in the image for the
//   camera), tilted about their vertical (or horizontal) axis from 0 to 85 deg by 5 deg, 12 images
//   per tilt and distance,
//   detected by each strategy with its default parameters: OpenCV, and the AprilTag library
//   (with OpenCV's PnP, MarkerAprilTag/OpenCVPnP, and with the library's own pose). It reports, per
//   detector, tilt and distance, how many markers are detected with a good pose and how
//   many with a bad pose (more than 10 deg off).
//
//   RefinementSweep: the same with fewer tilts (by 15 deg), comparing the corner refinement
//   methods of the OpenCV strategy (MarkerOpenCV/CornerRefinementMethod) and the quad
//   decimation of the AprilTag library (MarkerAprilTag/QuadDecimate).
//
//   The bad poses on the smallest markers are pessimistic compared to real images. Here,
//   decimation 2 gives a lot fewer bad poses at long range than decimation 1. On real
//   images (wall and floor markers, Marker/MaxRange=3, compared to the optimized maps),
//   decimation 2 detected about half as many markers, and more bad poses on floor markers
//   seen at grazing angles, while decimation 1 added almost no bad poses: so it is the
//   default.
//
// Its own executable, run by ctest under the "performance" label:
//   ctest -L performance    to run them
//   ctest -LE performance   to skip them
//   RTABMAP_PERF_MARKER_SIZE=0.1 bin/test_markerdetector_perf   for 10 cm markers (15 cm by default)
//   RTABMAP_PERF_IMAGE_WIDTH=1280 RTABMAP_PERF_IMAGE_HEIGHT=720 RTABMAP_PERF_FOCAL=910 bin/test_markerdetector_perf
//                                 for another camera (640x480 and 525 pixels by default)
//   RTABMAP_PERF_BLUR=1 bin/test_markerdetector_perf   for another blur (sigma in pixels, 0 for none,
//                                 0.5 pixel by default)
//   RTABMAP_PERF_SUPERSAMPLING=1 bin/test_markerdetector_perf   to render without averaging over each pixel
//                                 (8x8 samples per pixel by default)
//   RTABMAP_PERF_ROLL=30 bin/test_markerdetector_perf   for markers rotated by 30 deg about their normal
//                                 (0 by default, their edges aligned with the pixels)
//   RTABMAP_PERF_HORIZONTAL_TILT=1 bin/test_markerdetector_perf   to tilt the markers about their horizontal
//                                 axis, like floor markers (about their vertical axis by default, like wall
//                                 markers seen from the side)
//   RTABMAP_PERF_WHITE_BORDER=1 bin/test_markerdetector_perf   for a white border as wide as the markers
//                                 around them (a third of their size by default)
//   RTABMAP_PERF_MAX_DISTANCE=3 bin/test_markerdetector_perf   for distances up to 3 m (6 m by default)
//   RTABMAP_PERF_IMAGES=24 bin/test_markerdetector_perf   for 24 images per tilt and distance in the sweeps
//                                 (12 by default)
//   RTABMAP_PERF_JPEG_QUALITY=95 bin/test_markerdetector_perf   for images compressed in JPEG like in a database
//                                 (raw images by default)
//   RTABMAP_PERF_IDEAL=1 bin/test_markerdetector_perf   to render pure black on white markers without noise
//                                 (like a real camera image by default, see renderMarker())
//   RTABMAP_PERF_SAMPLES=samples.png bin/test_markerdetector_perf --gtest_filter=*Samples*
//                                 to save crops of rendered markers (skipped if not set)
//
// The sweep is reported rather than asserted on: the detections and bad poses depend on
// the accuracy of the corners, so on the detector and OpenCV versions.
#include <gtest/gtest.h>
#include <rtabmap/core/MarkerDetector.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Version.h> // RTABMAP_APRILTAG
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#if CV_MAJOR_VERSION > 4
#include <opencv2/geometry.hpp> // boundingRect(), getPerspectiveTransform()
#endif
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

// Side of the markers in the sweeps (m).
float markerSize()
{
	static const float size = envValue("RTABMAP_PERF_MARKER_SIZE", 0.15f);
	return size;
}

// Camera: the markers are near the image center, so the resolution only limits the
// closest markers. The focal length sets the width of the markers in the image at a given
// distance. Note that at the same width in the image, a larger focal length means a farther
// marker (smaller angular size), so less perspective distortion to recover the orientation.
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

// Rotation (deg) of the markers about their normal, before being tilted: 0 by default
// (RTABMAP_PERF_ROLL), their edges are then aligned with the pixels
float markerRoll()
{
	static const float roll = envValue("RTABMAP_PERF_ROLL", 0.0f);
	return roll;
}

// Markers tilted about their horizontal axis (RTABMAP_PERF_HORIZONTAL_TILT=1), like markers
// on the floor, instead of their vertical axis by default, like markers on
// walls seen from the side
bool horizontalTilt()
{
	static const bool horizontal = envValue("RTABMAP_PERF_HORIZONTAL_TILT", 0.0f) > 0.0f;
	return horizontal;
}

// Width of the white border around the markers, relative to their size: a third by
// default, like the measured real markers (RTABMAP_PERF_WHITE_BORDER)
float whiteBorder()
{
	static const float border = envValue("RTABMAP_PERF_WHITE_BORDER", 1.0f/3.0f);
	return border;
}

std::string setupDescription()
{
	return uFormat("Marker size: %g m (RTABMAP_PERF_MARKER_SIZE), camera: %dx%d (RTABMAP_PERF_IMAGE_WIDTH/HEIGHT), fx=fy=%g px (RTABMAP_PERF_FOCAL), white border: %g x marker size (RTABMAP_PERF_WHITE_BORDER), marker roll: %g deg (RTABMAP_PERF_ROLL), tilted about the %s axis (RTABMAP_PERF_HORIZONTAL_TILT).",
			markerSize(), imageWidth(), imageHeight(), focal(), whiteBorder(), markerRoll(), horizontalTilt()?"horizontal":"vertical");
}

// Width (pixels) of a frontal marker in the image at this distance
float widthAtDistance(float distance)
{
	return focal()*markerSize()/distance;
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
	ParametersMap parameters; // over the defaults
};

Config makeConfig(const std::string & name, int strategy,
		const std::string & key1 = "", const std::string & value1 = "")
{
	Config c{name, ParametersMap()};
	c.parameters.insert(ParametersPair(Parameters::kMarkerStrategy(), uNumber2Str(strategy)));
	if(!key1.empty())
	{
		c.parameters.insert(ParametersPair(key1, value1));
	}
	return c;
}

// The strategies with their default parameters, the OpenCV strategy first
std::vector<Config> strategyConfigs()
{
	std::vector<Config> configs;
	configs.push_back(makeConfig("OpenCV", 0));
#ifdef RTABMAP_APRILTAG
	configs.push_back(makeConfig("AprilTag", 1));
	configs.push_back(makeConfig("AprilTag+LibraryPose", 1, Parameters::kMarkerAprilTagOpenCVPnP(), "false"));
#endif
	return configs;
}

// The corner refinement methods of the OpenCV strategy and the quad decimations of the AprilTag library
std::vector<Config> refinementConfigs()
{
	std::vector<Config> configs;
	const char * methods[] = {"None", "Subpixel", "Contour", "AprilTag"};
	for(int i=0; i<4; ++i)
	{
		configs.push_back(makeConfig(std::string("OpenCV, refinement ")+uNumber2Str(i)+" ("+methods[i]+")", 0,
				Parameters::kMarkerOpenCVCornerRefinementMethod(), uNumber2Str(i)));
	}
#ifdef RTABMAP_APRILTAG
	configs.push_back(makeConfig("AprilTag, decimation 1", 1, Parameters::kMarkerAprilTagQuadDecimate(), "1"));
	configs.push_back(makeConfig("AprilTag, decimation 2", 1, Parameters::kMarkerAprilTagQuadDecimate(), "2"));
#endif
	return configs;
}

ParametersMap sweepParams(const Config & c)
{
	ParametersMap params = c.parameters;
	params.insert(ParametersPair(Parameters::kMarkerLength(), uNumber2Str(markerSize())));
	params.insert(ParametersPair(Parameters::kMarkerDictionary(), uNumber2Str(kDictionary)));
	return params;
}

// Marker facing the camera at `distance`, tilted about its vertical axis (or horizontal
// axis, see horizontalTilt()), its center offset by (offsetX, offsetY) pixels from the image center.
Transform sweepPose(float distance, float tiltDeg, float offsetX, float offsetY)
{
	const float tilt = tiltDeg*M_PI/180.0f;
	return Transform(
			1,  0,  0, offsetX*distance/focal(),
			0, -1,  0, offsetY*distance/focal(),
			0,  0, -1, distance) *
			// About the horizontal axis like a floor marker: its far edge at the top of the image
			(horizontalTilt() ? Transform(0, 0, 0, -tilt, 0, 0) : Transform(0, 0, 0, 0, tilt, 0)) *
			Transform(0, 0, 0, 0, 0, markerRoll()*M_PI/180.0f);
}

// Distances (m) of the markers in the sweeps: 16 steps up to 6 m by default, or up to
// RTABMAP_PERF_MAX_DISTANCE. They don't depend on the camera so that cameras can be
// compared (see widthAtDistance() for their width in the image).
std::vector<float> sweepDistances()
{
	const float maxDistance = envValue("RTABMAP_PERF_MAX_DISTANCE", 6.0f);
	std::vector<float> distances;
	for(int i=1; i<=16; ++i)
	{
		distances.push_back(maxDistance*i/16.0f);
	}
	return distances;
}

// Images per tilt and distance in the sweeps, each with the marker center jittered by up to
// half a pixel: 12 by default (RTABMAP_PERF_IMAGES)
int jitteredImages()
{
	static const int images = (int)envValue("RTABMAP_PERF_IMAGES", 12);
	return images;
}

// Gaussian blur (sigma, pixels) of the rendered markers: 0.5 pixel by default (like real
// 480x300 images), or the one set by RTABMAP_PERF_BLUR (0 for no blur)
float blur()
{
	static const float sigma = std::getenv("RTABMAP_PERF_BLUR") ? uStr2Float(std::getenv("RTABMAP_PERF_BLUR")) : 0.5f;
	return sigma;
}

// Resolution factor at which the markers are rendered before being averaged over each
// pixel (8 by default, RTABMAP_PERF_SUPERSAMPLING)
int supersampling()
{
	static const int ss = (int)envValue("RTABMAP_PERF_SUPERSAMPLING", 8);
	return ss;
}

// JPEG quality (1-100) with which the rendered images are compressed then decompressed,
// like images saved in a database (95 by default in RTAB-Map), or 0 (default) for raw
// images, like online detection (RTABMAP_PERF_JPEG_QUALITY)
int jpegQuality()
{
	static const int quality = (int)envValue("RTABMAP_PERF_JPEG_QUALITY", 0.0f);
	return quality;
}

// Whether the markers are rendered pure black on white without noise
// (RTABMAP_PERF_IDEAL=1) instead of like real camera images (see renderMarker())
bool idealRendering()
{
	static const bool ideal = envValue("RTABMAP_PERF_IDEAL", 0.0f) > 0.0f;
	return ideal;
}

// The marker image (markerSize() side), warped to its pose at supersampling() times the
// resolution, then averaged over each pixel like a camera (with only point sampling, the
// edges of small markers would be aliased), blurred and with sensor noise. Only the bounding
// box of the marker is rendered at high resolution. Unless idealRendering(), the intensities
// and noise are the ones measured on 20-33 pixels wide markers printed with a white border
// on a floor (480x300 8-bit grayscale images): black at 75, white at 245, a white border
// of a third of the marker size around it (see whiteBorder()), the floor at 140 and a
// noise of 3 (sigma).
cv::Mat renderMarker(const Transform & cameraToMarker)
{
	const int ss = supersampling();
	const bool ideal = idealRendering();
	const float black = ideal?0.0f:75.0f;
	const float white = ideal?255.0f:245.0f;
	const float floor = ideal?255.0f:140.0f;
	const float noise = ideal?0.0f:3.0f;
	const float borderRatio = whiteBorder(); // white border width / marker size

	cv::Mat marker;
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION==4 && CV_MINOR_VERSION >=7)
	cv::aruco::generateImageMarker(
			cv::aruco::getPredefinedDictionary(cv::aruco::PredefinedDictionaryType(kDictionary)), 0, 400, marker);
#else
	cv::aruco::drawMarker(
			cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(kDictionary)), 0, 400, marker);
#endif
	// With its white border, in the camera's intensities
	const int border = (int)(400*borderRatio);
	cv::copyMakeBorder(marker, marker, border, border, border, border, cv::BORDER_CONSTANT, cv::Scalar(255));
	marker.convertTo(marker, CV_8U, (white-black)/255.0, black);
	const float side = (float)marker.cols;
	const cv::Point2f src[4] = {
			cv::Point2f(-0.5f, -0.5f), cv::Point2f(side-0.5f, -0.5f),
			cv::Point2f(side-0.5f, side-0.5f), cv::Point2f(-0.5f, side-0.5f)};
	const float h = 0.5f + borderRatio;
	const float markerCorners[4][2] = {{-h, h}, {h, h}, {h, -h}, {-h, -h}};
	const CameraModel model = testCameraModel();
	cv::Point2f dst[4];
	for(int i=0; i<4; ++i)
	{
		cv::Point3f p = util3d::transformPoint(cv::Point3f(markerCorners[i][0]*markerSize(), markerCorners[i][1]*markerSize(), 0), cameraToMarker);
		dst[i].x = model.fx()*p.x/p.z + model.cx();
		dst[i].y = model.fy()*p.y/p.z + model.cy();
	}
	const cv::Rect roi = (cv::boundingRect(std::vector<cv::Point2f>(dst, dst+4)) + cv::Size(2,2) - cv::Point(1,1)) &
			cv::Rect(0, 0, imageWidth(), imageHeight());
	for(int i=0; i<4; ++i)
	{
		// Pixel centers of the high resolution image
		dst[i] = (dst[i] - cv::Point2f(roi.x, roi.y))*ss + cv::Point2f((ss-1)/2.0f, (ss-1)/2.0f);
	}
	cv::Mat roiImage;
	cv::warpPerspective(marker, roiImage, cv::getPerspectiveTransform(src, dst),
			roi.size()*ss, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(floor));
	cv::Mat image(imageHeight(), imageWidth(), CV_8UC1, cv::Scalar(floor));
	cv::resize(roiImage, image(roi), roi.size(), 0, 0, cv::INTER_AREA);
	if(blur() > 0.0f)
	{
		cv::GaussianBlur(image, image, cv::Size(0,0), blur());
	}
	if(noise > 0.0f)
	{
		static cv::RNG rng(42); // same noise for each run
		cv::Mat noisy;
		image.convertTo(noisy, CV_32F);
		cv::Mat gaussian(image.size(), CV_32F);
		rng.fill(gaussian, cv::RNG::NORMAL, 0.0, noise);
		noisy += gaussian;
		noisy.convertTo(image, CV_8U); // saturated
	}
	if(jpegQuality() > 0)
	{
		std::vector<unsigned char> jpeg;
		cv::imencode(".jpg", image, jpeg, std::vector<int>{cv::IMWRITE_JPEG_QUALITY, jpegQuality()});
		image = cv::imdecode(jpeg, cv::IMREAD_GRAYSCALE);
	}
	return image;
}
#endif

} // namespace

#ifdef RTABMAP_TEST_HAVE_ARUCO
namespace {

// Prints, per config, the good and bad detections per tilt (from 0 to 85 deg by tiltStep) and distance
void rangeTiltSweep(const std::vector<Config> & configs, int tiltStep)
{
	const CameraModel model = testCameraModel();
	const std::vector<float> distances = sweepDistances();

	// Per config, tilt and distance: good and bad detections
	struct Detected { int good=0, bad=0; };
	std::map<std::string, std::map<int, std::map<float, Detected> > > results;
	std::map<int, int> imagesPerTilt; // per distance
	std::map<std::string, double> detectionTime; // per config (s)
	int images = 0;

	// Per tilt and distance: jitteredImages() images, each with the marker center jittered by up
	// to half a pixel, otherwise the results depend on where the corners fall in the pixels
	// (e.g., all odd widths would have their corners on pixel centers).
	std::mt19937 rng(42);
	std::uniform_real_distribution<float> jitter(-0.5f, 0.5f);
	for(int tiltDeg=0; tiltDeg<=85; tiltDeg+=tiltStep)
	for(float distance : distances)
	for(int jitteredImage=0; jitteredImage<jitteredImages(); ++jitteredImage) // new random offsets below
	{
		const float offsetX = jitter(rng);
		const float offsetY = jitter(rng);
		const Transform cameraToMarker = sweepPose(distance, tiltDeg, offsetX, offsetY);
		const cv::Mat image = renderMarker(cameraToMarker);
		++images;
		if(distance == distances.front())
		{
			++imagesPerTilt[tiltDeg];
		}

		for(const Config & c : configs)
		{
			MarkerDetector detector(sweepParams(c));
			UTimer timer;
			std::map<int, MarkerInfo> detections = detector.detect(image, model);
			detectionTime[c.name] += timer.elapsed();
			if(detections.count(0) == 0)
			{
				continue;
			}
			Detected & detected = results[c.name][tiltDeg][distance];
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

	printf("\n%s\nGood and bad detections per tilt and distance, over %d images per tilt and distance\n"
			"(RTABMAP_PERF_IMAGES, blur %g px (RTABMAP_PERF_BLUR), center jittered %d times by +-0.5 px per axis,\n"
			"%dx%d samples per pixel (RTABMAP_PERF_SUPERSAMPLING), JPEG quality %d\n"
			"(RTABMAP_PERF_JPEG_QUALITY, 0 for raw images)).\n"
			"Each cell: number of good detections, number of bad ones (more than 10 deg off), \"-\" if none is bad.\n",
			setupDescription().c_str(), imagesPerTilt.rbegin()->second, blur(), jitteredImages(),
			supersampling(), supersampling(), jpegQuality());
	for(const Config & c : configs)
	{
		printf("\n  %s (detection: %.1f ms per image)\n  %9s", c.name.c_str(), detectionTime[c.name]*1000.0/images, "Range (m)");
		for(float distance : distances)
		{
			printf(" | %7.2f", distance);
		}
		printf("\n  %9s", "Width");
		for(float distance : distances)
		{
			printf(" | %4.0f px", widthAtDistance(distance));
		}
		printf("\n");
		for(const auto & t : imagesPerTilt)
		{
			printf("  %5d deg", t.first);
			for(float distance : distances)
			{
				const Detected & detected = results[c.name][t.first][distance];
				printf(" | %3d %3s", detected.good, detected.bad ? uNumber2Str(detected.bad).c_str() : "-");
			}
			printf("\n");
		}
		EXPECT_FALSE(results[c.name].empty()) << c.name << " detected nothing";
	}
}

} // namespace
#endif

TEST(MarkerDetectorPerf, RangeTiltSweep)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	rangeTiltSweep(strategyConfigs(), 5);
#endif
}

TEST(MarkerDetectorPerf, RefinementSweep)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	rangeTiltSweep(refinementConfigs(), 15);
#endif
}

// Crops of rendered markers for a few distances (columns) and tilts (rows), enlarged
// without interpolation to see their pixels, to check that they look like camera images.
// Each crop is labeled with the AprilTag detection
// (or OpenCV if not available): good, bad (more than 10 deg off) or not detected.
TEST(MarkerDetectorPerf, SaveRenderedSamples)
{
	const char * path = std::getenv("RTABMAP_PERF_SAMPLES");
	if(!path)
	{
		GTEST_SKIP() << "Set RTABMAP_PERF_SAMPLES to the image file to save";
	}
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	const std::vector<Config> configs = strategyConfigs();
	Config config = configs.front();
	for(const Config & c : configs)
	{
		if(c.name == "AprilTag")
		{
			config = c;
		}
	}
	MarkerDetector detector(sweepParams(config));
	const CameraModel model = testCameraModel();

	const std::vector<float> distances = sweepDistances();
	std::vector<int> tilts; // the same as the sweep
	for(int tiltDeg=0; tiltDeg<=85; tiltDeg+=5)
	{
		tilts.push_back(tiltDeg);
	}
	const int cell = 120;
	const int label = 30;
	const int header = 60;
	const int rowHeader = 110;
	cv::Mat sheet(header + (int)tilts.size()*(cell+label),
			rowHeader + (int)distances.size()*cell, CV_8UC3, cv::Scalar(255,255,255));
	for(size_t c=0; c<distances.size(); ++c)
	{
		cv::putText(sheet, uFormat("%.2f m, %.0f px", distances[c], widthAtDistance(distances[c])),
				cv::Point(rowHeader + (int)c*cell + 5, header-12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,0), 1);
	}
	int row = 0;
	for(int tiltDeg : tilts)
	{
		const int y = header + row*(cell+label);
		cv::putText(sheet, uFormat("%d deg", tiltDeg), cv::Point(5, y+cell/2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);
		for(size_t c=0; c<distances.size(); ++c)
		{
			// Tilted like the sweep's images
			const Transform cameraToMarker = sweepPose(distances[c], tiltDeg, 0.0f, 0.0f);
			const cv::Mat image = renderMarker(cameraToMarker);

			std::string result = "not detected";
			cv::Scalar color(128,128,128);
			std::map<int, MarkerInfo> detections = detector.detect(image, model);
			if(detections.count(0))
			{
				const float error = rotationErrorDeg(cameraToMarker, detections.at(0).pose());
				result = uFormat("%s %.0f deg", error > 10.0f ? "bad" : "good", error);
				color = error > 10.0f ? cv::Scalar(0,0,255) : cv::Scalar(0,140,0);
			}

			// Square crop around the marker, enlarged without interpolation
			const int half = std::max(8, (int)(widthAtDistance(distances[c])*0.75f));
			cv::Rect roi = cv::Rect(imageWidth()/2-half, imageHeight()/2-half, 2*half, 2*half) & cv::Rect(0, 0, imageWidth(), imageHeight());
			cv::Mat crop;
			cv::resize(image(roi), crop, cv::Size(cell, cell), 0, 0, cv::INTER_NEAREST);
			cv::cvtColor(crop, crop, cv::COLOR_GRAY2BGR);
			const int x = rowHeader + (int)c*cell;
			crop.copyTo(sheet(cv::Rect(x, y, cell, cell)));
			cv::rectangle(sheet, cv::Rect(x, y, cell, cell), cv::Scalar(200,200,200), 1);
			cv::putText(sheet, result, cv::Point(x+5, y+cell+20), cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1);
		}
		++row;
	}
	cv::putText(sheet, uFormat("Marker size: %g m (RTABMAP_PERF_MARKER_SIZE), camera: %dx%d (RTABMAP_PERF_IMAGE_WIDTH/HEIGHT), fx=fy=%g px (RTABMAP_PERF_FOCAL)",
			markerSize(), imageWidth(), imageHeight(), focal()), cv::Point(5, 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,0), 1);
	cv::putText(sheet, uFormat("Detector: %s, blur: %g px (RTABMAP_PERF_BLUR), %dx%d samples per pixel (RTABMAP_PERF_SUPERSAMPLING)%s",
			config.name.c_str(), blur(), supersampling(), supersampling(), idealRendering()?", ideal (RTABMAP_PERF_IDEAL)":""),
			cv::Point(5, 32), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,0), 1);
	ASSERT_TRUE(cv::imwrite(path, sheet)) << "Cannot write " << path;
	printf("\nSaved %s\n", path);
#endif
}
