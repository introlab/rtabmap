#include <gtest/gtest.h>
#include <rtabmap/core/MarkerDetector.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

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
