#include <gtest/gtest.h>
#include <rtabmap/core/MarkerDetector.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Version.h> // RTABMAP_APRILTAG, RTABMAP_APRILTAG_WITH_ARUCO
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#if CV_MAJOR_VERSION > 4
#include <opencv2/geometry.hpp> // getPerspectiveTransform()
#endif

using namespace rtabmap;

// The OpenCV strategy needs either the aruco contrib module or, since OpenCV 4.7,
// the aruco detector that moved into the objdetect module. Without one of them
// MarkerDetector cannot detect anything and every test here is meaningless.
#if ((CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION==4 && CV_MINOR_VERSION >=7)) && defined(HAVE_OPENCV_OBJDETECT)) || defined(HAVE_OPENCV_ARUCO)
#define RTABMAP_TEST_HAVE_ARUCO
#endif

namespace {

const int    kMarkerId      = 7;
const int    kMarkerPixels  = 100;   // marker side, in pixels, in the rendered image
const float  kMarkerLengthM = 0.2f;  // marker side, in meters
const int    kImageWidth    = 640;
const int    kImageHeight   = 480;
const double kFocal         = 525.0;

// Expected distance of a fronto-parallel marker: a side of kMarkerLengthM meters
// projected onto kMarkerPixels pixels sits at f * L / s meters from the camera.
const double kExpectedDistance = kFocal * kMarkerLengthM / double(kMarkerPixels);

// Camera looking straight ahead. Identity local transform keeps the returned pose
// in the optical frame (x right, y down, z forward), so z is the distance.
CameraModel testCameraModel()
{
	return CameraModel(
			kFocal, kFocal,
			kImageWidth/2.0, kImageHeight/2.0,
			Transform::getIdentity(),
			0.0,
			cv::Size(kImageWidth, kImageHeight));
}

#ifdef RTABMAP_TEST_HAVE_ARUCO
// Renders marker `id` of the default dictionary (Marker/Dictionary=0, DICT_4X4_50)
// centered in a white image, with a wide quiet zone so detection is unambiguous.
cv::Mat renderMarkerImage(int id, int sidePixels = kMarkerPixels)
{
	cv::Mat marker;
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION==4 && CV_MINOR_VERSION >=7)
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(
			cv::aruco::PredefinedDictionaryType(Parameters::defaultMarkerDictionary()));
	cv::aruco::generateImageMarker(dictionary, id, sidePixels, marker);
#else
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
			cv::aruco::PREDEFINED_DICTIONARY_NAME(Parameters::defaultMarkerDictionary()));
	cv::aruco::drawMarker(dictionary, id, sidePixels, marker);
#endif

	cv::Mat image(kImageHeight, kImageWidth, CV_8UC1, cv::Scalar(255));
	const int x = (kImageWidth - sidePixels)/2;
	const int y = (kImageHeight - sidePixels)/2;
	marker.copyTo(image(cv::Rect(x, y, sidePixels, sidePixels)));
	return image;
}
#endif

ParametersMap markerParams(float length)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kMarkerLength(), uNumber2Str(length)));
	return params;
}

#ifdef RTABMAP_TEST_HAVE_ARUCO
// Renders two markers of `side` pixels side by side, left one first.
cv::Mat renderTwoMarkers(int idLeft, int idRight, int side = 80)
{
	cv::Mat image(kImageHeight, kImageWidth, CV_8UC1, cv::Scalar(255));
	const int ids[2] = {idLeft, idRight};
	const cv::Rect centered((kImageWidth-side)/2, (kImageHeight-side)/2, side, side);
	for(int i=0; i<2; ++i)
	{
		renderMarkerImage(ids[i], side)(centered)
				.copyTo(image(cv::Rect(120 + i*280, (kImageHeight-side)/2, side, side)));
	}
	return image;
}
#endif

ParametersMap markerLengthsParams(const std::string & lengths)
{
	ParametersMap params;
	// Marker/Length keeps its default of 0 (estimate from depth): Marker/Lengths
	// gives an explicit length for every accepted marker, so no depth is needed.
	params.insert(ParametersPair(Parameters::kMarkerLength(), "0"));
	params.insert(ParametersPair(Parameters::kMarkerLengths(), lengths));
	return params;
}

} // namespace

TEST(MarkerDetectorTest, DetectsMarkerWithKnownLengthAndPose)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	MarkerDetector detector(markerParams(kMarkerLengthM));
	const cv::Mat image = renderMarkerImage(kMarkerId);

	std::map<int, MarkerInfo> detections = detector.detect(image, testCameraModel());

	ASSERT_EQ(detections.size(), 1u);
	ASSERT_EQ(detections.count(kMarkerId), 1u);

	const MarkerInfo & info = detections.at(kMarkerId);
	EXPECT_EQ(info.id(), kMarkerId);
	EXPECT_FLOAT_EQ(info.length(), kMarkerLengthM);

	// The marker is centered and fronto-parallel, so it sits on the optical axis
	// at the distance implied by its projected size.
	const Transform & pose = info.pose();
	ASSERT_FALSE(pose.isNull());
	EXPECT_NEAR(pose.z(), kExpectedDistance, 0.05*kExpectedDistance);
	EXPECT_NEAR(pose.x(), 0.0f, 0.02);
	EXPECT_NEAR(pose.y(), 0.0f, 0.02);
#endif
}

TEST(MarkerDetectorTest, DetectsMultipleMarkers)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// Two markers side by side, both well inside the image.
	const int ids[2] = {3, 11};
	const cv::Mat image = renderTwoMarkers(ids[0], ids[1]);

	MarkerDetector detector(markerParams(kMarkerLengthM));
	std::map<int, MarkerInfo> detections = detector.detect(image, testCameraModel());

	ASSERT_EQ(detections.size(), 2u);
	EXPECT_EQ(detections.count(ids[0]), 1u);
	EXPECT_EQ(detections.count(ids[1]), 1u);
	// Both are further away than the reference marker (smaller projected size) and
	// off to either side of the optical axis.
	EXPECT_LT(detections.at(ids[0]).pose().x(), 0.0f);
	EXPECT_GT(detections.at(ids[1]).pose().x(), 0.0f);
#endif
}

TEST(MarkerDetectorTest, MarkerLengthsParameterSetsPerIdLength)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// "id length|id length": each listed marker gets its own length, and the pose
	// scales with it (the raw pose is computed for a unit-length marker).
	const int idLeft = 3, idRight = 11;
	const cv::Mat image = renderTwoMarkers(idLeft, idRight);

	MarkerDetector detector(markerLengthsParams("3 0.08|11 0.15"));
	std::map<int, MarkerInfo> detections = detector.detect(image, testCameraModel());

	ASSERT_EQ(detections.size(), 2u);
	EXPECT_FLOAT_EQ(detections.at(idLeft).length(), 0.08f);
	EXPECT_FLOAT_EQ(detections.at(idRight).length(), 0.15f);
	// Same projected size, so the one declared ~1.9x longer is ~1.9x further away.
	EXPECT_GT(detections.at(idRight).pose().z(), detections.at(idLeft).pose().z());
	EXPECT_NEAR(detections.at(idRight).pose().z() / detections.at(idLeft).pose().z(),
			0.15/0.08, 0.05);
#endif
}

TEST(MarkerDetectorTest, MarkerLengthsParameterFiltersUnlistedMarkers)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// A non-empty Marker/Lengths acts as a whitelist: markers absent from it are
	// dropped even though they are detected in the image.
	const int idLeft = 3, idRight = 11;
	const cv::Mat image = renderTwoMarkers(idLeft, idRight);

	MarkerDetector detector(markerLengthsParams("11 0.15"));
	std::map<int, MarkerInfo> detections = detector.detect(image, testCameraModel());

	ASSERT_EQ(detections.size(), 1u);
	EXPECT_EQ(detections.count(idRight), 1u);
	EXPECT_EQ(detections.count(idLeft), 0u);
	EXPECT_FLOAT_EQ(detections.at(idRight).length(), 0.15f);
#endif
}

TEST(MarkerDetectorTest, MarkerLengthsParameterSupportsIdRanges)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// "id1:id2 length" applies to every id in the range, bounds included.
	const int idLeft = 3, idRight = 11;
	const cv::Mat image = renderTwoMarkers(idLeft, idRight);

	MarkerDetector detector(markerLengthsParams("3:11 0.1"));
	std::map<int, MarkerInfo> detections = detector.detect(image, testCameraModel());

	ASSERT_EQ(detections.size(), 2u);
	EXPECT_FLOAT_EQ(detections.at(idLeft).length(), 0.1f);
	EXPECT_FLOAT_EQ(detections.at(idRight).length(), 0.1f);

	// A range that stops just before an id excludes it.
	MarkerDetector detectorShortRange(markerLengthsParams("3:10 0.1"));
	std::map<int, MarkerInfo> partial = detectorShortRange.detect(image, testCameraModel());
	ASSERT_EQ(partial.size(), 1u);
	EXPECT_EQ(partial.count(idLeft), 1u);
#endif
}

TEST(MarkerDetectorTest, MarkerLengthsParameterIgnoredWhenMalformed)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// A malformed entry (missing length) invalidates the whole list, which is then
	// cleared. With Marker/Length=0 and no depth, nothing can be sized, so nothing
	// is reported -- rather than the list being silently half-applied.
	const cv::Mat image = renderTwoMarkers(3, 11);

	MarkerDetector detector(markerLengthsParams("3 0.08|11"));
	EXPECT_TRUE(detector.detect(image, testCameraModel()).empty());
#endif
}

TEST(MarkerDetectorTest, DetectArgumentOverridesMarkerLengthsParameter)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// When an id is declared both in Marker/Lengths and in the per-call map, the
	// per-call value wins (and a warning is logged).
	const cv::Mat image = renderMarkerImage(kMarkerId);

	MarkerDetector detector(markerLengthsParams(uFormat("%d 0.08", kMarkerId)));
	std::map<int, float> extraLengths;
	extraLengths.insert(std::make_pair(kMarkerId, 0.3f));

	std::map<int, MarkerInfo> detections =
			detector.detect(image, testCameraModel(), cv::Mat(), extraLengths);

	ASSERT_EQ(detections.size(), 1u);
	EXPECT_FLOAT_EQ(detections.at(kMarkerId).length(), 0.3f);
#endif
}

TEST(MarkerDetectorTest, NoDetectionOnBlankImage)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	MarkerDetector detector(markerParams(kMarkerLengthM));
	cv::Mat blank(kImageHeight, kImageWidth, CV_8UC1, cv::Scalar(255));

	EXPECT_TRUE(detector.detect(blank, testCameraModel()).empty());
#endif
}

TEST(MarkerDetectorTest, MarkerLengthsOverrideTheGlobalLength)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// Marker/Length is negative, so each marker takes its length from the
	// per-id map passed to detect().
	MarkerDetector detector(markerParams(-1.0f));
	const cv::Mat image = renderMarkerImage(kMarkerId);

	std::map<int, float> lengths;
	lengths.insert(std::make_pair(kMarkerId, 0.5f));
	std::map<int, MarkerInfo> detections =
			detector.detect(image, testCameraModel(), cv::Mat(), lengths);

	ASSERT_EQ(detections.size(), 1u);
	EXPECT_FLOAT_EQ(detections.at(kMarkerId).length(), 0.5f);
	// The pose is scaled by the marker length, so a 2.5x longer marker is 2.5x further.
	EXPECT_NEAR(detections.at(kMarkerId).pose().z(),
			kExpectedDistance*0.5/kMarkerLengthM, 0.05*kExpectedDistance*0.5/kMarkerLengthM);

	// An id with no length at all is dropped rather than reported with a bogus pose.
	MarkerDetector detectorNoLengths(markerParams(-1.0f));
	EXPECT_TRUE(detectorNoLengths.detect(image, testCameraModel()).empty());
#endif
}

TEST(MarkerDetectorTest, IgnoresDepthImageWithUnsupportedType)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// Only CV_16UC1/CV_32FC1 are real depth maps. Anything else (e.g. a stereo right
	// image passed by mistake) must be ignored with a warning instead of being fed to
	// util2d::getDepth().
	MarkerDetector detector(markerParams(kMarkerLengthM));
	const cv::Mat image = renderMarkerImage(kMarkerId);
	const cv::Mat notADepthMap(kImageHeight, kImageWidth, CV_8UC1, cv::Scalar(120));

	std::map<int, MarkerInfo> detections =
			detector.detect(image, testCameraModel(), notADepthMap);

	// Detection still succeeds, using the configured Marker/Length.
	ASSERT_EQ(detections.size(), 1u);
	EXPECT_FLOAT_EQ(detections.at(kMarkerId).length(), kMarkerLengthM);
	EXPECT_NEAR(detections.at(kMarkerId).pose().z(), kExpectedDistance, 0.05*kExpectedDistance);
#endif
}

TEST(MarkerDetectorTest, ReturnsEmptyWhenDepthMissingAndLengthUnset)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// Marker/Length=0 asks for automatic length estimation from depth; with no depth
	// image there is nothing to estimate from, so detection bails out early.
	MarkerDetector detector(markerParams(0.0f));
	const cv::Mat image = renderMarkerImage(kMarkerId);

	EXPECT_TRUE(detector.detect(image, testCameraModel()).empty());
#endif
}

TEST(MarkerDetectorTest, EstimatesLengthFromDepth)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// With Marker/Length=0 and a constant depth map, the marker length is derived
	// from the measured depth: length = depth / z_unit, where z_unit is the pose
	// obtained for a unit-length marker. A plane at kExpectedDistance therefore
	// yields back kMarkerLengthM.
	MarkerDetector detector(markerParams(0.0f));
	const cv::Mat image = renderMarkerImage(kMarkerId);
	cv::Mat depth(kImageHeight, kImageWidth, CV_32FC1, cv::Scalar(float(kExpectedDistance)));

	std::map<int, MarkerInfo> detections = detector.detect(image, testCameraModel(), depth);

	ASSERT_EQ(detections.size(), 1u);
	EXPECT_NEAR(detections.at(kMarkerId).length(), kMarkerLengthM, 0.05*kMarkerLengthM);
	EXPECT_NEAR(detections.at(kMarkerId).pose().z(), kExpectedDistance, 0.05*kExpectedDistance);
#endif
}

TEST(MarkerDetectorTest, RangeLimitsFilterDetections)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	const cv::Mat image = renderMarkerImage(kMarkerId);

	// Marker sits at ~kExpectedDistance: a max range below that filters it out...
	ParametersMap tooClose = markerParams(kMarkerLengthM);
	tooClose.insert(ParametersPair(Parameters::kMarkerMaxRange(), uNumber2Str(kExpectedDistance/2.0)));
	MarkerDetector detectorMax(tooClose);
	EXPECT_TRUE(detectorMax.detect(image, testCameraModel()).empty());

	// ... and so does a min range above it.
	ParametersMap tooFar = markerParams(kMarkerLengthM);
	tooFar.insert(ParametersPair(Parameters::kMarkerMinRange(), uNumber2Str(kExpectedDistance*2.0)));
	MarkerDetector detectorMin(tooFar);
	EXPECT_TRUE(detectorMin.detect(image, testCameraModel()).empty());

	// A range window containing the marker keeps it.
	ParametersMap window = markerParams(kMarkerLengthM);
	window.insert(ParametersPair(Parameters::kMarkerMinRange(), uNumber2Str(kExpectedDistance/2.0)));
	window.insert(ParametersPair(Parameters::kMarkerMaxRange(), uNumber2Str(kExpectedDistance*2.0)));
	MarkerDetector detectorWindow(window);
	EXPECT_EQ(detectorWindow.detect(image, testCameraModel()).size(), 1u);
#endif
}

TEST(MarkerDetectorTest, LocalTransformIsAppliedToPose)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// The returned pose is expressed in the camera's base frame, i.e. the optical
	// pose pre-multiplied by the model's local transform. With the standard optical
	// rotation, the marker straight ahead lands on +x instead of +z.
	MarkerDetector detector(markerParams(kMarkerLengthM));
	const cv::Mat image = renderMarkerImage(kMarkerId);

	CameraModel model(
			kFocal, kFocal,
			kImageWidth/2.0, kImageHeight/2.0,
			CameraModel::opticalRotation(),
			0.0,
			cv::Size(kImageWidth, kImageHeight));

	std::map<int, MarkerInfo> detections = detector.detect(image, model);

	ASSERT_EQ(detections.size(), 1u);
	const Transform & pose = detections.at(kMarkerId).pose();
	EXPECT_NEAR(pose.x(), kExpectedDistance, 0.05*kExpectedDistance);
	EXPECT_NEAR(pose.y(), 0.0f, 0.02);
	EXPECT_NEAR(pose.z(), 0.0f, 0.02);
#endif
}

TEST(MarkerDetectorTest, DrawsDetectionsOnOutputImage)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	MarkerDetector detector(markerParams(kMarkerLengthM));
	const cv::Mat image = renderMarkerImage(kMarkerId);

	cv::Mat imageWithDetections;
	std::map<int, MarkerInfo> detections =
			detector.detect(image, testCameraModel(), cv::Mat(), std::map<int, float>(), &imageWithDetections);

	ASSERT_EQ(detections.size(), 1u);
	ASSERT_FALSE(imageWithDetections.empty());
	EXPECT_EQ(imageWithDetections.size(), image.size());
	// The gray input is converted to color and annotated, so it must differ from a
	// plain gray-to-BGR copy of the source.
	EXPECT_EQ(imageWithDetections.channels(), 3);
	cv::Mat plain;
	cv::cvtColor(image, plain, cv::COLOR_GRAY2BGR);
	EXPECT_GT(cv::norm(imageWithDetections, plain, cv::NORM_L1), 0.0);
#endif
}

TEST(MarkerDetectorTest, DetectsMarkersInStitchedMultiCameraImage)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// The multi-camera overload takes one stitched image and one model per camera.
	// Corners are split by sub-image, each marker is posed with its own camera model,
	// then the corners are shifted back into stitched coordinates.
	// Camera 0 holds a centered marker, camera 1 an off-center one.
	const int side = kMarkerPixels;
	cv::Mat stitched(kImageHeight, kImageWidth*2, CV_8UC1, cv::Scalar(255));
	const int idCam0 = 3;
	const int idCam1 = 11;
	const cv::Rect centered((kImageWidth-side)/2, (kImageHeight-side)/2, side, side);

	// Centered in camera 0's half.
	renderMarkerImage(idCam0, side)(centered)
			.copyTo(stitched(cv::Rect((kImageWidth-side)/2, (kImageHeight-side)/2, side, side)));
	// Left of center inside camera 1's half (which starts at x=kImageWidth).
	const int cam1MarkerX = kImageWidth + 100;
	renderMarkerImage(idCam1, side)(centered)
			.copyTo(stitched(cv::Rect(cam1MarkerX, (kImageHeight-side)/2, side, side)));

	std::vector<CameraModel> models;
	models.push_back(testCameraModel());
	models.push_back(testCameraModel());

	MarkerDetector detector(markerParams(kMarkerLengthM));
	std::map<int, MarkerInfo> detections = detector.detect(stitched, models);

	ASSERT_EQ(detections.size(), 2u);
	ASSERT_EQ(detections.count(idCam0), 1u);
	ASSERT_EQ(detections.count(idCam1), 1u);

	// Both markers have the same projected size, so both sit at the same distance
	// even though they are in different sub-images.
	EXPECT_NEAR(detections.at(idCam0).pose().z(), kExpectedDistance, 0.05*kExpectedDistance);
	EXPECT_NEAR(detections.at(idCam1).pose().z(), kExpectedDistance, 0.05*kExpectedDistance);

	// Camera 0's marker is on its optical axis.
	EXPECT_NEAR(detections.at(idCam0).pose().x(), 0.0f, 0.02);
	EXPECT_NEAR(detections.at(idCam0).pose().y(), 0.0f, 0.02);

	// Camera 1's marker is posed relative to *camera 1's* optical axis, not to the
	// stitched image: its center is left of that camera's principal point, so x<0.
	// If the sub-image offset were mishandled, this would come out around
	// +(kImageWidth/2)*z/f instead.
	const double cam1CenterX = cam1MarkerX + side/2.0 - kImageWidth; // within camera 1
	const double expectedX = (cam1CenterX - kImageWidth/2.0) * kExpectedDistance / kFocal;
	EXPECT_LT(detections.at(idCam1).pose().x(), 0.0f);
	EXPECT_NEAR(detections.at(idCam1).pose().x(), expectedX, 0.02);
#endif
}

TEST(MarkerDetectorTest, RejectsImageWiderThanSingleCameraModel)
{
	// The single-model overload refuses a stitched multi-camera image instead of
	// silently mis-associating detections.
	MarkerDetector detector(markerParams(kMarkerLengthM));
	cv::Mat wide(kImageHeight, kImageWidth*2, CV_8UC1, cv::Scalar(255));

	EXPECT_TRUE(detector.detect(wide, testCameraModel()).empty());
}

// ---------------------------------------------------------------------------
// Marker frame convention, for every strategy and dictionary
//
// Whatever the strategy (OpenCV aruco or AprilTag library) and dictionary, the
// returned marker frame should follow OpenCV's convention, relative to the marker
// image as rendered by OpenCV: origin at the center, x to the right, y up and z
// out of the marker (toward the camera looking at it).
// ---------------------------------------------------------------------------
namespace {

#ifdef RTABMAP_TEST_HAVE_ARUCO
struct StrategyDictionary
{
	int strategy;
	int dictionary;
};

std::vector<StrategyDictionary> allStrategiesAndDictionaries()
{
	// OpenCV: DICT_4X4_50 (0) ... DICT_7X7_1000 (15), DICT_ARUCO_ORIGINAL (16),
	// DICT_APRILTAG_16h5 ... DICT_APRILTAG_36h11 (17-20), DICT_ARUCO_MIP_36h12 (21, OpenCV >= 4.8)
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 8)
	const int lastDictionary = 21;
#else
	const int lastDictionary = 20;
#endif
	std::vector<StrategyDictionary> cases;
	for(int dict=0; dict<=lastDictionary; ++dict)
	{
		cases.push_back({0, dict});
	}
#ifdef RTABMAP_APRILTAG
	for(int dict=0; dict<=lastDictionary; ++dict)
	{
		const bool isAprilTag = dict >= 17 && dict <= 20;
#ifndef RTABMAP_APRILTAG_WITH_ARUCO
		if(!isAprilTag) continue;
#endif
		if(dict == 16) continue; // ARUCO_ORIGINAL is not supported by the AprilTag library
		cases.push_back({1, dict});
	}
#endif
	return cases;
}

cv::Mat renderDictionaryMarker(int dictionary, int id, int sidePixels)
{
	cv::Mat marker;
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION==4 && CV_MINOR_VERSION >=7)
	cv::aruco::generateImageMarker(
			cv::aruco::getPredefinedDictionary(cv::aruco::PredefinedDictionaryType(dictionary)),
			id, sidePixels, marker);
#else
	cv::aruco::drawMarker(
			cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary)),
			id, sidePixels, marker);
#endif
	return marker;
}

// Renders the marker of side 1 m at pose `cameraToMarker` (optical frame, x right,
// y up and z out of the marker image) by projecting its outer corners with the test
// camera. With `officialAprilTag`, the image is the one of the official AprilTag
// images (apriltag-imgs), which is OpenCV's AprilTag image rotated by 180 deg.
cv::Mat renderMarkerAtPose(int dictionary, int id, const Transform & cameraToMarker, bool officialAprilTag = false)
{
	const int side = 400; // rendered large, then warped down
	cv::Mat marker = renderDictionaryMarker(dictionary, id, side);
	if(officialAprilTag)
	{
		cv::rotate(marker, marker, cv::ROTATE_180);
	}

	// Marker image corners, in pixel-center coordinates: top-left, top-right,
	// bottom-right, bottom-left; i.e. (-x,+y), (+x,+y), (+x,-y), (-x,-y) in the marker frame.
	const cv::Point2f src[4] = {
			cv::Point2f(-0.5f, -0.5f),
			cv::Point2f(side-0.5f, -0.5f),
			cv::Point2f(side-0.5f, side-0.5f),
			cv::Point2f(-0.5f, side-0.5f)};
	const float corners[4][2] = {{-0.5f, 0.5f}, {0.5f, 0.5f}, {0.5f, -0.5f}, {-0.5f, -0.5f}};
	const CameraModel model = testCameraModel();
	cv::Point2f dst[4];
	for(int i=0; i<4; ++i)
	{
		cv::Point3f p = util3d::transformPoint(cv::Point3f(corners[i][0], corners[i][1], 0), cameraToMarker);
		dst[i].x = model.fx()*p.x/p.z + model.cx();
		dst[i].y = model.fy()*p.y/p.z + model.cy();
	}

	cv::Mat image;
	cv::warpPerspective(marker, image, cv::getPerspectiveTransform(src, dst),
			cv::Size(kImageWidth, kImageHeight), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
	return image;
}

float rotationErrorDeg(const Transform & a, const Transform & b)
{
	Eigen::Matrix3f r = a.toEigen3f().rotation().transpose() * b.toEigen3f().rotation();
	return Eigen::AngleAxisf(r).angle() * 180.0f / float(M_PI);
}

std::string caseName(const StrategyDictionary & c)
{
	return uFormat("strategy=%d dictionary=%d", c.strategy, c.dictionary);
}

ParametersMap strategyParams(const StrategyDictionary & c, int aprilTagUpright = 0)
{
	ParametersMap params = markerParams(1.0f);
	params.insert(ParametersPair(Parameters::kMarkerAprilTagUpright(), uNumber2Str(aprilTagUpright)));
	params.insert(ParametersPair(Parameters::kMarkerStrategy(), uNumber2Str(c.strategy)));
	params.insert(ParametersPair(Parameters::kMarkerDictionary(), uNumber2Str(c.dictionary)));
	return params;
}
#endif

} // namespace

TEST(MarkerDetectorTest, MarkerFrameIsXRightYUpZTowardCameraForAllStrategies)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// Upright marker of 1 m facing the camera, 5 m straight ahead.
	const Transform cameraToMarker(
			1,  0,  0, 0,
			0, -1,  0, 0,
			0,  0, -1, 5);
	for(const StrategyDictionary & c : allStrategiesAndDictionaries())
	{
		SCOPED_TRACE(caseName(c));
		MarkerDetector detector(strategyParams(c));
		const cv::Mat image = renderMarkerAtPose(c.dictionary, 0, cameraToMarker);
		std::map<int, MarkerInfo> detections = detector.detect(image, testCameraModel());
		ASSERT_EQ(detections.count(0), 1u);
		const Transform & pose = detections.at(0).pose(); // optical frame (identity local transform)

		EXPECT_NEAR(pose.x(), 0.0f, 0.05);
		EXPECT_NEAR(pose.y(), 0.0f, 0.05);
		EXPECT_NEAR(pose.z(), 5.0f, 0.1);

		// Marker axes expressed in the optical frame (x right, y down, z forward).
		const Eigen::Matrix3f R = pose.toEigen3f().rotation();
		const Eigen::Vector3f xAxis = R.col(0), yAxis = R.col(1), zAxis = R.col(2);
		EXPECT_GT(xAxis.x(), 0.99f) << "marker x should point right, got " << xAxis.transpose();
		EXPECT_LT(yAxis.y(), -0.99f) << "marker y should point up, got " << yAxis.transpose();
		EXPECT_LT(zAxis.z(), -0.99f) << "marker z should point toward the camera, got " << zAxis.transpose();
	}
#endif
}

TEST(MarkerDetectorTest, TiltedMarkerPoseMatchesGroundTruthForAllStrategies)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// Off-center marker, rotated about each of its axes, so that any axis swap or
	// sign flip (not only a 180 deg flip) shows up as a large rotation error.
	const Transform facingCamera(
			1,  0,  0, 0.4f,
			0, -1,  0, -0.3f,
			0,  0, -1, 4.0f);
	const Transform cameraToMarker = facingCamera * Transform(0, 0, 0, 20.0f*M_PI/180.0f, -25.0f*M_PI/180.0f, 30.0f*M_PI/180.0f);
	for(const StrategyDictionary & c : allStrategiesAndDictionaries())
	{
		SCOPED_TRACE(caseName(c));
		MarkerDetector detector(strategyParams(c));
		const cv::Mat image = renderMarkerAtPose(c.dictionary, 0, cameraToMarker);
		std::map<int, MarkerInfo> detections = detector.detect(image, testCameraModel());
		ASSERT_EQ(detections.count(0), 1u);
		const Transform & pose = detections.at(0).pose();

		EXPECT_LT(rotationErrorDeg(cameraToMarker, pose), 5.0f)
				<< "expected " << cameraToMarker.prettyPrint() << ", got " << pose.prettyPrint();
		EXPECT_LT((pose.toEigen3f().translation() - cameraToMarker.toEigen3f().translation()).norm(), 0.1f)
				<< "expected " << cameraToMarker.prettyPrint() << ", got " << pose.prettyPrint();
	}
#endif
}

TEST(MarkerDetectorTest, MinPixelWidthFiltersSmallMarkersForAllStrategies)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// 1 m marker 4 m ahead: ~131 pixels wide. Tilted by 50 deg about the vertical
	// axis, its horizontal sides shrink to ~85 pixels, but its vertical sides don't.
	const Transform facingCamera(
			1,  0,  0, 0,
			0, -1,  0, 0,
			0,  0, -1, 4.0f);
	const Transform tilted = facingCamera * Transform(0, 0, 0, 0, 50.0f*M_PI/180.0f, 0);
	for(const StrategyDictionary & c : allStrategiesAndDictionaries())
	{
		SCOPED_TRACE(caseName(c));
		for(const Transform & cameraToMarker : {facingCamera, tilted})
		{
			SCOPED_TRACE(cameraToMarker.prettyPrint());
			const cv::Mat image = renderMarkerAtPose(c.dictionary, 0, cameraToMarker);

			ParametersMap params = strategyParams(c);
			MarkerDetector detectorDisabled(params);
			ASSERT_EQ(detectorDisabled.detect(image, testCameraModel()).count(0), 1u);

			// The longest side is used: the tilted marker is kept.
			params.insert(ParametersPair(Parameters::kMarkerMinPixelWidth(), "110"));
			MarkerDetector detectorKept(params);
			EXPECT_EQ(detectorKept.detect(image, testCameraModel()).count(0), 1u);

			params[Parameters::kMarkerMinPixelWidth()] = "150";
			MarkerDetector detectorFiltered(params);
			EXPECT_TRUE(detectorFiltered.detect(image, testCameraModel()).empty());
		}
	}
#endif
}

TEST(MarkerDetectorTest, AprilTagOpenCVPnPPoseMatchesGroundTruth)
{
#if !defined(RTABMAP_TEST_HAVE_ARUCO) || !defined(RTABMAP_APRILTAG) || CV_MAJOR_VERSION < 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 1)
	GTEST_SKIP() << "Needs OpenCV aruco/objdetect, the AprilTag library and OpenCV >= 4.1";
#else
	// Same tilted pose as TiltedMarkerPoseMatchesGroundTruthForAllStrategies: the pose
	// from OpenCV's PnP should be in the same frame as the AprilTag library's one.
	const Transform facingCamera(
			1,  0,  0, 0.4f,
			0, -1,  0, -0.3f,
			0,  0, -1, 4.0f);
	const Transform cameraToMarker = facingCamera * Transform(0, 0, 0, 20.0f*M_PI/180.0f, -25.0f*M_PI/180.0f, 30.0f*M_PI/180.0f);
	for(const StrategyDictionary & c : allStrategiesAndDictionaries())
	{
		if(c.strategy != 1) continue;
		SCOPED_TRACE(caseName(c));
		ParametersMap params = strategyParams(c);
		params.insert(ParametersPair(Parameters::kMarkerAprilTagOpenCVPnP(), "true"));
		MarkerDetector detector(params);
		std::map<int, MarkerInfo> detections = detector.detect(
				renderMarkerAtPose(c.dictionary, 0, cameraToMarker), testCameraModel());
		ASSERT_EQ(detections.count(0), 1u);
		const Transform & pose = detections.at(0).pose();
		EXPECT_LT(rotationErrorDeg(cameraToMarker, pose), 5.0f)
				<< "expected " << cameraToMarker.prettyPrint() << ", got " << pose.prettyPrint();
		EXPECT_LT((pose.toEigen3f().translation() - cameraToMarker.toEigen3f().translation()).norm(), 0.1f)
				<< "expected " << cameraToMarker.prettyPrint() << ", got " << pose.prettyPrint();
	}
#endif
}

TEST(MarkerDetectorTest, AprilTagUprightSelectsOpenCVOrOfficialOrientation)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// Same tilted pose as above, but relative to the official AprilTag image.
	const Transform facingCamera(
			1,  0,  0, 0.4f,
			0, -1,  0, -0.3f,
			0,  0, -1, 4.0f);
	const Transform cameraToMarker = facingCamera * Transform(0, 0, 0, 20.0f*M_PI/180.0f, -25.0f*M_PI/180.0f, 30.0f*M_PI/180.0f);
	const Transform rotated180 = cameraToMarker * Transform(0, 0, 0, 0, 0, M_PI);
	for(const StrategyDictionary & c : allStrategiesAndDictionaries())
	{
		SCOPED_TRACE(caseName(c));
		const bool isAprilTag = c.dictionary >= 17 && c.dictionary <= 20;
		const cv::Mat official = renderMarkerAtPose(c.dictionary, 0, cameraToMarker, isAprilTag);

		// 1=Official: the frame follows the official image.
		MarkerDetector officialDetector(strategyParams(c, 1));
		std::map<int, MarkerInfo> detections = officialDetector.detect(official, testCameraModel());
		ASSERT_EQ(detections.count(0), 1u);
		EXPECT_LT(rotationErrorDeg(cameraToMarker, detections.at(0).pose()), 5.0f)
				<< "expected " << cameraToMarker.prettyPrint() << ", got " << detections.at(0).pose().prettyPrint();

		// 0=OpenCV (default): on the same official AprilTag image, the frame is
		// rotated by 180 deg around its z axis. ArUco dictionaries are not affected.
		MarkerDetector opencvDetector(strategyParams(c, 0));
		detections = opencvDetector.detect(official, testCameraModel());
		ASSERT_EQ(detections.count(0), 1u);
		const Transform & expected = isAprilTag ? rotated180 : cameraToMarker;
		EXPECT_LT(rotationErrorDeg(expected, detections.at(0).pose()), 5.0f)
				<< "expected " << expected.prettyPrint() << ", got " << detections.at(0).pose().prettyPrint();
	}
#endif
}

TEST(MarkerDetectorTest, FirstCornerIsTopLeftOfRequestedUprightForAllStrategies)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// The first corner is not returned, but cv::aruco::drawDetectedMarkers() marks it
	// with a small red square on the debug image (the red x axis drawn from the center
	// ends in the middle of the right side, away from the corners). Upright marker of
	// 1 m at 5 m: it spans 100x100 pixels centered in the image.
	const Transform cameraToMarker(
			1,  0,  0, 0,
			0, -1,  0, 0,
			0,  0, -1, 5);
	const int half = 50;
	const cv::Point imageCorners[4] = {
			cv::Point(kImageWidth/2-half, kImageHeight/2-half),  // top-left
			cv::Point(kImageWidth/2+half, kImageHeight/2-half),  // top-right
			cv::Point(kImageWidth/2+half, kImageHeight/2+half),  // bottom-right
			cv::Point(kImageWidth/2-half, kImageHeight/2+half)}; // bottom-left
	for(const StrategyDictionary & c : allStrategiesAndDictionaries())
	{
		const bool isAprilTag = c.dictionary >= 17 && c.dictionary <= 20;
		for(int upright=0; upright<=1; ++upright)
		{
			SCOPED_TRACE(caseName(c) + uFormat(" upright=%d", upright));
			MarkerDetector detector(strategyParams(c, upright));
			cv::Mat debugImage;
			std::map<int, MarkerInfo> detections = detector.detect(
					renderMarkerAtPose(c.dictionary, 0, cameraToMarker, isAprilTag && upright==1),
					testCameraModel(), cv::Mat(), std::map<int, float>(), &debugImage);
			ASSERT_EQ(detections.count(0), 1u);
			ASSERT_EQ(debugImage.type(), CV_8UC3);

			for(int i=0; i<4; ++i)
			{
				cv::Rect window(imageCorners[i] - cv::Point(6, 6), cv::Size(13, 13));
				cv::Mat red; // BGR, anti-aliased
				cv::inRange(debugImage(window), cv::Scalar(0,0,200), cv::Scalar(60,60,255), red);
				if(i == 0)
				{
					EXPECT_GT(cv::countNonZero(red), 0) << "first corner not on the top-left";
				}
				else
				{
					EXPECT_EQ(cv::countNonZero(red), 0) << "first corner found on corner " << i << " (0=top-left, clockwise)";
				}
			}
		}
	}
#endif
}

// ---------------------------------------------------------------------------
// Planar pose ambiguity (Marker/MaxAmbiguity)
//
// A square marker seen small (weak perspective) and slightly tilted has two
// poses, mirrored about the line of sight, reprojecting its corners almost
// equally well: the detector can return the "flipped" orientation.
// ---------------------------------------------------------------------------
namespace {
#ifdef RTABMAP_TEST_HAVE_ARUCO
ParametersMap ambiguityParams(const StrategyDictionary & c, float maxAmbiguity)
{
	ParametersMap params = strategyParams(c);
	params.insert(ParametersPair(Parameters::kMarkerMaxAmbiguity(), uNumber2Str(maxAmbiguity)));
	return params;
}
#endif
} // namespace

TEST(MarkerDetectorTest, MaxAmbiguityKeepsWellConditionedPoseForAllStrategies)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// Same strongly tilted, close marker as TiltedMarkerPoseMatchesGroundTruthForAllStrategies:
	// strong perspective, so the second solution reprojects clearly worse.
	const Transform facingCamera(
			1,  0,  0, 0.4f,
			0, -1,  0, -0.3f,
			0,  0, -1, 4.0f);
	const Transform cameraToMarker = facingCamera * Transform(0, 0, 0, 20.0f*M_PI/180.0f, -25.0f*M_PI/180.0f, 30.0f*M_PI/180.0f);
	for(const StrategyDictionary & c : allStrategiesAndDictionaries())
	{
		SCOPED_TRACE(caseName(c));
		MarkerDetector detector(ambiguityParams(c, 0.1f));
		std::map<int, MarkerInfo> detections = detector.detect(
				renderMarkerAtPose(c.dictionary, 0, cameraToMarker), testCameraModel());
		ASSERT_EQ(detections.count(0), 1u);
		EXPECT_LT(rotationErrorDeg(cameraToMarker, detections.at(0).pose()), 5.0f)
				<< "expected " << cameraToMarker.prettyPrint() << ", got " << detections.at(0).pose().prettyPrint();
	}
#endif
}

TEST(MarkerDetectorTest, MaxAmbiguityRejectsFlippedPose)
{
#ifndef RTABMAP_TEST_HAVE_ARUCO
	GTEST_SKIP() << "RTAB-Map not built with OpenCV aruco/objdetect marker detection";
#else
	// Far marker (about 35 pixels wide), tilted by 20 deg around its vertical axis,
	// slightly blurred to add noise on its corners. Without the check, OpenCV
	// returns the flipped pose (tilted by about -20 deg).
	const Transform facingCamera(
			1,  0,  0, 0,
			0, -1,  0, 0,
			0,  0, -1, 15.0f);
	const Transform cameraToMarker = facingCamera * Transform(0, 0, 0, 0, 20.0f*M_PI/180.0f, 0);
	const StrategyDictionary c = {0, Parameters::defaultMarkerDictionary()};
	cv::Mat image = renderMarkerAtPose(c.dictionary, 0, cameraToMarker);
	cv::GaussianBlur(image, image, cv::Size(0,0), 1.5);

	// Disabled (default): detected
	MarkerDetector detector(strategyParams(c));
	EXPECT_EQ(detector.detect(image, testCameraModel()).count(0), 1u);

	// Enabled: rejected as ambiguous, the two solutions differ by about 40 deg,
	// which is over 1 sigma of the default Marker/VarianceAngular (5.7 deg)
	MarkerDetector ambiguityDetector(ambiguityParams(c, 0.1f));
	EXPECT_EQ(ambiguityDetector.detect(image, testCameraModel()).count(0), 0u);

	// Enabled, but within 1 sigma of the angular variance (57 deg): kept
	ParametersMap params = ambiguityParams(c, 0.1f);
	params.insert(ParametersPair(Parameters::kMarkerVarianceAngular(), "1"));
	MarkerDetector noisyDetector(params);
	EXPECT_EQ(noisyDetector.detect(image, testCameraModel()).count(0), 1u);

	// Enabled, but orientation ignored: kept
	params = ambiguityParams(c, 0.1f);
	params.insert(ParametersPair(Parameters::kMarkerVarianceOrientationIgnored(), "true"));
	MarkerDetector orientationIgnoredDetector(params);
	EXPECT_EQ(orientationIgnoredDetector.detect(image, testCameraModel()).count(0), 1u);
#endif
}
