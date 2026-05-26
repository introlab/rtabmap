/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap/core/MarkerDetector.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>

#ifdef HAVE_OPENCV_ARUCO
#if CV_MAJOR_VERSION < 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <8)
namespace cv{
	namespace aruco {
		static const int DICT_ARUCO_MIP_36h12 = 21;
#if CV_MAJOR_VERSION < 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <7)
		typedef PREDEFINED_DICTIONARY_NAME PredefinedDictionaryType;
#endif
	}
}
#endif
#endif

#ifdef RTABMAP_APRILTAG
extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/apriltag_pose.h"
#ifdef RTABMAP_APRILTAG_WITH_ARUCO
#include "apriltag/aruco/tagAruco4x4_50.h"
#include "apriltag/aruco/tagAruco4x4_100.h"
#include "apriltag/aruco/tagAruco4x4_250.h"
#include "apriltag/aruco/tagAruco4x4_1000.h"
#include "apriltag/aruco/tagAruco5x5_50.h"
#include "apriltag/aruco/tagAruco5x5_100.h"
#include "apriltag/aruco/tagAruco5x5_250.h"
#include "apriltag/aruco/tagAruco5x5_1000.h"
#include "apriltag/aruco/tagAruco6x6_50.h"
#include "apriltag/aruco/tagAruco6x6_100.h"
#include "apriltag/aruco/tagAruco6x6_250.h"
#include "apriltag/aruco/tagAruco6x6_1000.h"
#include "apriltag/aruco/tagAruco7x7_50.h"
#include "apriltag/aruco/tagAruco7x7_100.h"
#include "apriltag/aruco/tagAruco7x7_250.h"
#include "apriltag/aruco/tagAruco7x7_1000.h"
#include "apriltag/aruco/tagArucoMIP36h12.h"
#endif
#include "apriltag/tag16h5.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag36h10.h"
#include "apriltag/tag36h11.h"
}

#ifndef HAVE_OPENCV_ARUCO
// To match opencv::aruco module, add opencv::aruco dictionary enum
namespace cv{
namespace aruco {
enum PredefinedDictionaryType {
	DICT_4X4_50 = 0,
	DICT_4X4_100,
	DICT_4X4_250,
	DICT_4X4_1000,
	DICT_5X5_50,
	DICT_5X5_100,
	DICT_5X5_250,
	DICT_5X5_1000,
	DICT_6X6_50,
	DICT_6X6_100,
	DICT_6X6_250,
	DICT_6X6_1000,
	DICT_7X7_50,
	DICT_7X7_100,
	DICT_7X7_250,
	DICT_7X7_1000,
	DICT_ARUCO_ORIGINAL,
	DICT_APRILTAG_16h5,
	DICT_APRILTAG_25h9,
	DICT_APRILTAG_36h10,
	DICT_APRILTAG_36h11,
	DICT_ARUCO_MIP_36h12
};
}
}
#endif
#endif

namespace rtabmap {

#ifdef RTABMAP_APRILTAG

#define ARUCO_CASE(N, TOTAL) \
case cv::aruco::DICT_##N##X##N##_##TOTAL: \
	af = tagAruco##N##x##N##_##TOTAL##_create(); \
	break;
#define APRILTAG_CASE(N1, N2) \
case cv::aruco::DICT_APRILTAG_##N1##h##N2: \
	af = tag##N1##h##N2##_create(); \
	break;

apriltag_family_t * createAprilTagPredefinedDictionary(int opencvArucoDictionary)
{
	apriltag_family_t * af = NULL;
	switch(opencvArucoDictionary)
	{
		APRILTAG_CASE(16, 5)
		APRILTAG_CASE(25, 9)
		APRILTAG_CASE(36, 10)
		APRILTAG_CASE(36, 11)
#ifdef RTABMAP_APRILTAG_WITH_ARUCO
		ARUCO_CASE(4, 50)
		ARUCO_CASE(4, 100)
		ARUCO_CASE(4, 250)
		ARUCO_CASE(4, 1000)
		ARUCO_CASE(5, 50)
		ARUCO_CASE(5, 100)
		ARUCO_CASE(5, 250)
		ARUCO_CASE(5, 1000)
		ARUCO_CASE(6, 50)
		ARUCO_CASE(6, 100)
		ARUCO_CASE(6, 250)
		ARUCO_CASE(6, 1000)
		ARUCO_CASE(7, 50)
		ARUCO_CASE(7, 100)
		ARUCO_CASE(7, 250)
		ARUCO_CASE(7, 1000)
		case cv::aruco::DICT_ARUCO_MIP_36h12:
			af = tagArucoMIP36h12_create();
			break;
#endif
		default:
			break;
	}
	return af;
}
void destroyAprilTagDictionary(apriltag_family_t * dictionary)
{
	if(dictionary == NULL)
	{
		return;
	}
	const char* name = dictionary->name;
	if(strcmp(name, "tag16h5") == 0)               tag16h5_destroy(dictionary);
	else if(strcmp(name, "tag25h9") == 0)          tag25h9_destroy(dictionary);
	else if(strcmp(name, "tag36h10") == 0)         tag36h10_destroy(dictionary);
	else if(strcmp(name, "tag36h11") == 0)         tag36h11_destroy(dictionary);
#ifdef RTABMAP_APRILTAG_WITH_ARUCO
	else if(strcmp(name, "tagAruco4x4_50") == 0)   tagAruco4x4_50_destroy(dictionary);
	else if(strcmp(name, "tagAruco4x4_100") == 0)  tagAruco4x4_100_destroy(dictionary);
	else if(strcmp(name, "tagAruco4x4_250") == 0)  tagAruco4x4_250_destroy(dictionary);
	else if(strcmp(name, "tagAruco4x4_1000") == 0) tagAruco4x4_1000_destroy(dictionary);
	else if(strcmp(name, "tagAruco5x5_50") == 0)   tagAruco5x5_50_destroy(dictionary);
	else if(strcmp(name, "tagAruco5x5_100") == 0)  tagAruco5x5_100_destroy(dictionary);
	else if(strcmp(name, "tagAruco5x5_250") == 0)  tagAruco5x5_250_destroy(dictionary);
	else if(strcmp(name, "tagAruco5x5_1000") == 0) tagAruco5x5_1000_destroy(dictionary);
	else if(strcmp(name, "tagAruco6x6_50") == 0)   tagAruco6x6_50_destroy(dictionary);
	else if(strcmp(name, "tagAruco6x6_100") == 0)  tagAruco6x6_100_destroy(dictionary);
	else if(strcmp(name, "tagAruco6x6_250") == 0)  tagAruco6x6_250_destroy(dictionary);
	else if(strcmp(name, "tagAruco6x6_1000") == 0) tagAruco6x6_1000_destroy(dictionary);
	else if(strcmp(name, "tagAruco7x7_50") == 0)   tagAruco7x7_50_destroy(dictionary);
	else if(strcmp(name, "tagAruco7x7_100") == 0)  tagAruco7x7_100_destroy(dictionary);
	else if(strcmp(name, "tagAruco7x7_250") == 0)  tagAruco7x7_250_destroy(dictionary);
	else if(strcmp(name, "tagAruco7x7_1000") == 0) tagAruco7x7_1000_destroy(dictionary);
	else if(strcmp(name, "tagArucoMIP_36h12") == 0) tagArucoMIP36h12_destroy(dictionary);
#endif	
	else
	{
		UFATAL("AprilTag: Didn't find the right destructor for dictionary \"%s\"", name);
	}
}
#endif

MarkerDetector::MarkerDetector(const ParametersMap & parameters) :
	strategy_((Strategy)Parameters::defaultMarkerStrategy()),
	markerLength_(Parameters::defaultMarkerLength()),
	maxDepthError_(Parameters::defaultMarkerMaxDepthError()),
	maxRange_(Parameters::defaultMarkerMaxRange()),
	minRange_(Parameters::defaultMarkerMinRange()),
	dictionaryId_(Parameters::defaultMarkerDictionary()),
	apriltagLibDetector_(NULL),
	apriltagLibFamily_(NULL)
{
#ifdef HAVE_OPENCV_ARUCO
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
    detectorParams_.reset(new cv::aruco::DetectorParameters());
#elif CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=2)
	detectorParams_ = cv::aruco::DetectorParameters::create();
#else
	detectorParams_.reset(new cv::aruco::DetectorParameters());
#endif
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
    detectorParams_->cornerRefinementMethod = (cv::aruco::CornerRefineMethod) Parameters::defaultMarkerOpenCVCornerRefinementMethod();
#elif CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=3)
	detectorParams_->cornerRefinementMethod = Parameters::defaultMarkerOpenCVCornerRefinementMethod();
#else
	detectorParams_->doCornerRefinement = Parameters::defaultMarkerOpenCVCornerRefinementMethod()!=0;
#endif
#endif

	parseParameters(parameters);
}

MarkerDetector::~MarkerDetector() {
#ifdef RTABMAP_APRILTAG
	if(apriltagLibDetector_)
	{
		apriltag_detector_destroy(((apriltag_detector_t*)apriltagLibDetector_));
	}
	if(apriltagLibFamily_)
	{
		destroyAprilTagDictionary((apriltag_family_t*)apriltagLibFamily_);
	}
#endif
}

void MarkerDetector::parseParameters(const ParametersMap & parameters)
{
	int strategy = strategy_;
	Parameters::parse(parameters, Parameters::kMarkerStrategy(), strategy);
	strategy_ = (Strategy)strategy;
	Parameters::parse(parameters, Parameters::kMarkerLength(), markerLength_);
	Parameters::parse(parameters, Parameters::kMarkerMaxDepthError(), maxDepthError_);
	Parameters::parse(parameters, Parameters::kMarkerMaxRange(), maxRange_);
	Parameters::parse(parameters, Parameters::kMarkerMinRange(), minRange_);
	Parameters::parse(parameters, Parameters::kMarkerDictionary(), dictionaryId_);
	
	if(parameters.find(Parameters::kMarkerLengths()) != parameters.end())
	{
		markerLengths_.clear();
		std::string strLengths;
		Parameters::parse(parameters, Parameters::kMarkerLengths(), strLengths);
		std::list<std::string> strList = uSplit(strLengths, '|');
		for(std::list<std::string>::iterator iter=strList.begin(); iter!=strList.end(); ++iter)
		{
			std::list<std::string> items = uSplit(*iter, ' ');
			if(items.size() != 2)
			{
				UERROR("Invalid string format \"%s\" for parameter %s, make "
					"sure the values are separated by single space and/or '|'. See "
					"description of the parameter for example. That parameter "
					"will be ignored.",
					strLengths.c_str(),
					Parameters::kMarkerLengths().c_str());
				markerLengths_.clear();
				break;
			}
			else
			{
				float length = uStr2Float(items.back());
				if(uStrContains(items.front(), ":"))
				{
					std::list<std::string> range = uSplit(items.front(), ':');
					if(range.size() != 2)
					{
						UERROR("Invalid string format \"%s\" for parameter %s, make "
							"sure the values are separated by single space and/or '|'. See "
							"description of the parameter for example. That parameter "
							"will be ignored.",
							strLengths.c_str(),
							Parameters::kMarkerLengths().c_str());
						markerLengths_.clear();
						break;
					}
					int id1 = uStr2Int(range.front());
					int id2 = uStr2Int(range.back());
					UDEBUG("Adding a range of markers %d -> %d with length %f", id1, id2, length);
					for(int id=id1; id<=id2; ++id)
					{
						auto inserted = markerLengths_.insert(std::make_pair(id, length));
						if(!inserted.second)
						{
							UWARN("Marker %d is already configured with length %f, not overwriting", inserted.first->first, inserted.first->second);
						}
					}
				}
				else {
					int id = uStr2Int(items.front());
					auto inserted = markerLengths_.insert(std::make_pair(id, length));
					if(!inserted.second)
					{
						UWARN("Marker %d is already configured with length %f, overwriting with %f", inserted.first->first, inserted.first->second, length);
						inserted.first->second = length;
					}
					else
					{
						UDEBUG("Added marker %d with length %f", id, length);
					}
				}
			}
		}
	}

#ifdef HAVE_OPENCV_ARUCO
	detectorParams_->adaptiveThreshWinSizeMin = 3;
	detectorParams_->adaptiveThreshWinSizeMax = 23;
	detectorParams_->adaptiveThreshWinSizeStep = 10;
	detectorParams_->adaptiveThreshConstant = 7;
	detectorParams_->minMarkerPerimeterRate = 0.03;
	detectorParams_->maxMarkerPerimeterRate = 4.0;
	detectorParams_->polygonalApproxAccuracyRate = 0.03;
	detectorParams_->minCornerDistanceRate = 0.05;
	detectorParams_->minDistanceToBorder = 3;
	detectorParams_->minMarkerDistanceRate = 0.05;
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
    int cornerRefinementMethod;
    Parameters::parse(parameters, Parameters::kMarkerOpenCVCornerRefinementMethod(), cornerRefinementMethod);
    detectorParams_->cornerRefinementMethod = (cv::aruco::CornerRefineMethod)cornerRefinementMethod;
#elif CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=3)
	Parameters::parse(parameters, Parameters::kMarkerOpenCVCornerRefinementMethod(), detectorParams_->cornerRefinementMethod);
#else
	int doCornerRefinement = detectorParams_->doCornerRefinement?1:0;
	Parameters::parse(parameters, Parameters::kMarkerOpenCVCornerRefinementMethod(), doCornerRefinement);
	detectorParams_->doCornerRefinement = doCornerRefinement!=0;
#endif
	detectorParams_->cornerRefinementWinSize = 5;
	detectorParams_->cornerRefinementMaxIterations = 30;
	detectorParams_->cornerRefinementMinAccuracy = 0.1;
	detectorParams_->markerBorderBits = 1;
	detectorParams_->perspectiveRemovePixelPerCell = 4;
	detectorParams_->perspectiveRemoveIgnoredMarginPerCell = 0.13;
	detectorParams_->maxErroneousBitsInBorderRate = 0.35;
	detectorParams_->minOtsuStdDev = 5.0;
	detectorParams_->errorCorrectionRate = 0.6;

	
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION <4 || (CV_MINOR_VERSION ==4 && CV_SUBMINOR_VERSION<2)))
	if(dictionaryId_ = 17 && strategy_ == 0)
	{
		UERROR("Cannot set AprilTag dictionary. OpenCV version should be at least 3.4.2, "
				"current version is %s. Setting %s to default (%d)",
				CV_VERSION,
				Parameters::kMarkerDictionary().c_str(),
				Parameters::defaultMarkerDictionary());
		dictionaryId_ = Parameters::defaultMarkerDictionary();
	}
#elif CV_MAJOR_VERSION < 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <8)
	if(dictionaryId_ == 21 && strategy_ == 0)
	{
		UERROR("Cannot set ARUCO_MIP_36h12 dictionary. OpenCV version should be at least 4.8.0, "
				"current version is %s. Setting %s to default (%d)",
				CV_VERSION,
				Parameters::kMarkerDictionary().c_str(),
				Parameters::defaultMarkerDictionary());
		dictionaryId_ = Parameters::defaultMarkerDictionary();
	}
#endif
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
    dictionary_.reset(new cv::aruco::Dictionary());
    *dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PredefinedDictionaryType(dictionaryId_));
#elif CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=2)
	dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PredefinedDictionaryType(dictionaryId_));
#else
	dictionary_.reset(new cv::aruco::Dictionary());
	*dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PredefinedDictionaryType(dictionaryId_));
#endif
#else
	if(strategy_ == 0)
	{
#ifdef RTABMAP_APRILTAG
		UERROR("RTAB-Map is not built with opencv-aruco library! Fallback to AprilTag strategy (%s=1).", Parameters::kMarkerStrategy().c_str());
		strategy_ = kStrategyApriltag;
#else
		UERROR("RTAB-Map is not built with opencv-aruco library!");
#endif
	}
#endif

#ifdef RTABMAP_APRILTAG
	if(apriltagLibDetector_ && apriltagLibFamily_) {
		apriltag_detector_remove_family(((apriltag_detector_t*)apriltagLibDetector_), (apriltag_family_t*)apriltagLibFamily_);
	}
	if(apriltagLibFamily_)
	{
		destroyAprilTagDictionary((apriltag_family_t*)apriltagLibFamily_);
		apriltagLibFamily_ = NULL;
	}
	if(strategy_ == 1)
	{
		if(apriltagLibDetector_ == NULL)
		{
			apriltagLibDetector_ = apriltag_detector_create();
		}
		Parameters::parse(parameters, Parameters::kMarkerAprilTagNThreads(), ((apriltag_detector_t*)apriltagLibDetector_)->nthreads);
		Parameters::parse(parameters, Parameters::kMarkerAprilTagQuadDecimate(), ((apriltag_detector_t*)apriltagLibDetector_)->quad_decimate);
		Parameters::parse(parameters, Parameters::kMarkerAprilTagQuadSigma(), ((apriltag_detector_t*)apriltagLibDetector_)->quad_sigma);
		Parameters::parse(parameters, Parameters::kMarkerAprilTagRefineEdges(), ((apriltag_detector_t*)apriltagLibDetector_)->refine_edges);
		Parameters::parse(parameters, Parameters::kMarkerAprilTagDecodeSharpening(), ((apriltag_detector_t*)apriltagLibDetector_)->decode_sharpening);
		Parameters::parse(parameters, Parameters::kMarkerAprilTagDebug(), ((apriltag_detector_t*)apriltagLibDetector_)->debug);

		cv::aruco::PredefinedDictionaryType dictFamily = cv::aruco::PredefinedDictionaryType(dictionaryId_);
#ifndef RTABMAP_APRILTAG_WITH_ARUCO
		if((dictFamily >= cv::aruco::DICT_4X4_50 && dictFamily < cv::aruco::DICT_APRILTAG_16h5) || dictFamily==cv::aruco::DICT_ARUCO_MIP_36h12)
		{
			UERROR("Cannot set aruco dictionaries with current AprilTag library version. "
					"Use opencv-aruco implementation instead or rebuild/install latest "
					"AprilTag from source. Setting %s to default apriltag dictionary (%d)",
					Parameters::kMarkerDictionary().c_str(),
					cv::aruco::DICT_APRILTAG_36h11);
			dictionaryId_ = cv::aruco::DICT_APRILTAG_36h11;
			dictFamily = cv::aruco::PredefinedDictionaryType(dictionaryId_);
		}
#endif
		if(dictFamily == cv::aruco::DICT_ARUCO_ORIGINAL)
		{
			UERROR("Cannot set ARUCO_ORIGINAL (%d) dictionary with AprilTag library implementation. "
					"Use opencv-aruco implementation instead. Setting %s to default apriltag dictionary (%d)",
					dictionaryId_,
					Parameters::kMarkerDictionary().c_str(),
					cv::aruco::DICT_APRILTAG_36h11);
			dictionaryId_ = cv::aruco::DICT_APRILTAG_36h11;
			dictFamily = cv::aruco::PredefinedDictionaryType(dictionaryId_);
		}
		apriltagLibFamily_  = createAprilTagPredefinedDictionary(dictFamily);
		UASSERT_MSG(apriltagLibFamily_ != NULL, uFormat("AprilTag library cannot be used with dictionary type %d", (int)dictionaryId_).c_str());
		errno = 0;
		apriltag_detector_add_family(((apriltag_detector_t*)apriltagLibDetector_), (apriltag_family_t*)apriltagLibFamily_);
		if (errno == ENOMEM) {
			UFATAL("Unable to add family to detector due to insufficient memory to allocate the tag-family decoder with the default maximum hamming value of 2. Try choosing an alternative tag family.");
		}
	}
#else
	if(strategy_ == 1)
	{
#ifdef HAVE_OPENCV_ARUCO
		UERROR("RTAB-Map is not built with apriltag library! Fallback to OpenCV (%s=0).", Parameters::kMarkerStrategy().c_str());
		strategy_ = kStrategyOpencv;
#else
		UERROR("RTAB-Map is not built with apriltag library!");
#endif
	}
#endif
}

// deprecated
std::map<int, Transform> MarkerDetector::detect(const cv::Mat & image, const CameraModel & model, const cv::Mat & depth, float * markerLengthOut, cv::Mat * imageWithDetections)
{
	UDEBUG("");
    std::map<int, Transform> detections;
    std::map<int, MarkerInfo> infos = detect(image, model, depth, std::map<int, float>(), imageWithDetections);
    
    for(std::map<int, MarkerInfo>::iterator iter=infos.begin(); iter!=infos.end(); ++iter)
    {
        detections.insert(std::make_pair(iter->first, iter->second.pose()));
        
        if(markerLengthOut)
        {
            *markerLengthOut = iter->second.length();
        }
    }
    
    return detections;
}

std::map<int, MarkerInfo> MarkerDetector::detect(const cv::Mat & image,
                           const std::vector<CameraModel> & models,
                           const cv::Mat & depth,
                           const std::map<int, float> & extraMarkerLengths,
                           cv::Mat * imageWithDetections)
{
	UASSERT(!models.empty() && !image.empty());
	UASSERT(int((image.cols/models.size())*models.size()) == image.cols);
	UASSERT(int((depth.cols/models.size())*models.size()) == depth.cols);
	int subRGBWidth = image.cols/models.size();

	float rgbToDepthFactorX = 1.0f;
	float rgbToDepthFactorY = 1.0f;
	if(!depth.empty())
	{
		rgbToDepthFactorX = float(depth.cols) / float(image.cols);
		rgbToDepthFactorY = float(depth.rows) / float(image.rows);
	}
	else if(markerLength_ == 0)
	{
		if(depth.empty())
		{
			UERROR("Depth image is empty, please set %s parameter to non-null.", Parameters::kMarkerLength().c_str());
			return std::map<int, MarkerInfo>();
		}
	}

	std::vector< int > ids;
	std::vector< int > cams;
	std::vector< std::vector< cv::Point2f > > corners; // in stitched image
	std::vector<Transform> poses; // in optical frame

	// detect markers and estimate pose
	if(strategy_ == kStrategyApriltag)
	{
#ifdef RTABMAP_APRILTAG
		cv::Mat grayImage = image;
		if(image.channels() > 1)
		{
			cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
		}
		image_u8_t im = {grayImage.cols, grayImage.rows, (int)grayImage.step, grayImage.data};

		errno = 0;
		zarray_t *apriltagDetections = apriltag_detector_detect(((apriltag_detector_t*)apriltagLibDetector_), &im);
		if (errno == EAGAIN) {
			UERROR("Unable to create the %d threads requested.", ((apriltag_detector_t*)apriltagLibDetector_)->nthreads);
			if(apriltagDetections)
			{
				apriltag_detections_destroy(apriltagDetections);
			}
			return std::map<int, MarkerInfo>();
		}

		std::set<int> idsAdded;
		for (int i = 0; i < zarray_size(apriltagDetections); i++)
		{
            apriltag_detection_t *det;
            zarray_get(apriltagDetections, i, &det);

			// Which camera?
			int cameraIndex = int(det->c[0]) / subRGBWidth;
			if(cameraIndex<0 || cameraIndex>=(int)models.size())
			{
				UWARN("Marker %d detected outside the image! camera index=%d (models=%ld subWidth=%d) marker center=(%f,%f). Ignoring...",
					det->id, cameraIndex, models.size(), subRGBWidth, det->c[0], det->c[1]);
				continue;
			}

			if(!markerLengths_.empty() &&
				markerLengths_.find(det->id) == markerLengths_.end() && 
				extraMarkerLengths.find(det->id) == extraMarkerLengths.end())
			{
				UDEBUG("Ignoring marker %d because it is not in the list of expected markers (see %s)",
					det->id,
					Parameters::kMarkerLengths().c_str());
				continue;
			}
			if(idsAdded.find(det->id)!=idsAdded.end())
			{
				UWARN("Marker %d already added by another camera, ignoring detection from camera %d", det->id, cameraIndex);
				continue;
			}

			const CameraModel & model = models[cameraIndex];
			float offsetX = cameraIndex*subRGBWidth;

			// Convert detection in local camera
			det->c[0] -= offsetX;
			for(int i=0; i<4; ++i)
			{
				det->p[i][0] -= offsetX;
			}

			// Shift the homography to match the local-camera pixel frame:
			// pre-multiply H by T = [[1,0,-offsetX],[0,1,0],[0,0,1]] so it
			// stays consistent with the shifted corners. estimate_tag_pose()
			// seeds its iterative refinement from H, so leaving it stale
			// produces a bad initial guess (often triggers the AprilTag
			// "more than one new minimum found" debug print).
			MATD_EL(det->H, 0, 0) -= offsetX * MATD_EL(det->H, 2, 0);
			MATD_EL(det->H, 0, 1) -= offsetX * MATD_EL(det->H, 2, 1);
			MATD_EL(det->H, 0, 2) -= offsetX * MATD_EL(det->H, 2, 2);

			apriltag_detection_info_t info;
			info.det = det;
			info.tagsize = 1.0f;
			info.fx = model.fx();
			info.fy = model.fy();
			info.cx = model.cx();
			info.cy = model.cy();

			// Then call estimate_tag_pose.
			apriltag_pose_t pose;
			double err = estimate_tag_pose(&info, &pose);

			if (pose.R && pose.t)
			{
				if(MATD_EL(pose.t, 2, 0) > 0)
				{
					Transform t(MATD_EL(pose.R, 0, 0), MATD_EL(pose.R, 0, 1), MATD_EL(pose.R, 0, 2), MATD_EL(pose.t, 0, 0),
								MATD_EL(pose.R, 1, 0), MATD_EL(pose.R, 1, 1), MATD_EL(pose.R, 1, 2), MATD_EL(pose.t, 1, 0),
								MATD_EL(pose.R, 2, 0), MATD_EL(pose.R, 2, 1), MATD_EL(pose.R, 2, 2), MATD_EL(pose.t, 2, 0));
					poses.push_back(t*Transform(-1,0,0,0, 0,1,0,0, 0,0,-1,0)); // Flip tag to match same frame than OpenCV (+x->-x, +z->-z)

					corners.push_back(std::vector<cv::Point2f>(4));
					for(int i=0; i<4; ++i)
					{
						// reconvert in original stitched image
						corners.back()[i].x = det->p[i][0]+offsetX;
						corners.back()[i].y = det->p[i][1];
					}
					ids.push_back(det->id);
					cams.push_back(cameraIndex);
					UDEBUG("Add marker %d (err = %f)", det->id, err);
					idsAdded.insert(det->id);
				}
				else
				{
					UWARN("Skipping %d because its estimated pose is behind the camera %d", det->id, cameraIndex);
				}
			}
			else
			{
				UWARN("Failed to compute pose for marker %d, ignoring...", det->id);
			}

			// Free pose memory
			if (pose.R) matd_destroy(pose.R);
			if (pose.t) matd_destroy(pose.t);
        }

		apriltag_detections_destroy(apriltagDetections);
#else
		UERROR("RTAB-Map is not built with apriltag library.");
#endif
	}
	else // opencv
	{
		std::vector< int > cvIds;
		std::vector< std::vector< cv::Point2f > > cvCorners, cvRejected;
#ifdef HAVE_OPENCV_ARUCO
#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=2)
		cv::aruco::detectMarkers(image, dictionary_, cvCorners, cvIds, detectorParams_, cvRejected);
#else
		cv::aruco::detectMarkers(image, *dictionary_, cvCorners, cvIds, *detectorParams_, cvRejected);
#endif
		UDEBUG("Markers detected=%d rejected=%d", (int)cvIds.size(), (int)cvRejected.size());

		// split detections per camera
		std::set<int> idsAdded;
		std::vector< std::vector< std::vector< cv::Point2f > > > cvCornersPerCam(models.size());
		std::vector< std::vector< int > > cvIdsPerCam(models.size());
		for(size_t i=0; i<cvIds.size(); ++i)
		{
			int id = cvIds[i];

			// Which camera?
			int cameraIndex = int(cvCorners[i][0].x) / subRGBWidth;
			if(cameraIndex<0 || cameraIndex>=(int)models.size())
			{
				UWARN("Marker %d detected outside the image! camera index=%d (models=%ld subWidth=%d) marker center=(%f,%f). Ignoring...",
					id, cameraIndex, models.size(), subRGBWidth, cvCorners[i][0].x, cvCorners[i][0].y);
				continue;
			}

			if(!markerLengths_.empty() &&
				markerLengths_.find(id) == markerLengths_.end() && 
				extraMarkerLengths.find(id) == extraMarkerLengths.end())
			{
				UDEBUG("Ignoring marker %d because it is not in the list of expected markers (see %s)",
					id,
					Parameters::kMarkerLengths().c_str());
				continue;
			}
			if(idsAdded.find(id) != idsAdded.end())
			{
				UWARN("Marker %d already added by another camera, ignoring detection from camera %d", id, cameraIndex);
				continue;
			}

			float offsetX = cameraIndex*subRGBWidth;
			for(size_t j=0; j<cvCorners[i].size(); ++j)
			{
				cvCorners[i][j].x -= offsetX;
			}
			
			cvCornersPerCam[cameraIndex].push_back(cvCorners[i]);
			cvIdsPerCam[cameraIndex].push_back(id);
			idsAdded.insert(id);
			UDEBUG("Detected marker %d on camera %d", id, cameraIndex);
		}

		for(size_t cam=0; cam < cvCornersPerCam.size(); ++cam)
		{
			std::vector< cv::Vec3d > rvecs, tvecs;
			const CameraModel & model = models[cam];
			cv::aruco::estimatePoseSingleMarkers(cvCornersPerCam[cam], 1.0f, model.K(), model.D(), rvecs, tvecs);
			float offsetX = cam*subRGBWidth;
			for(size_t i=0; i<cvIdsPerCam[cam].size(); ++i)
			{
				if(tvecs[i].val[2] <=0)
				{
					UWARN("Skipping %d because its estimated pose is behind the camera %d", cvIdsPerCam[cam][i], cam);
					continue;
				}
				cv::Mat R;
				cv::Rodrigues(rvecs[i], R);
				Transform t(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvecs[i].val[0],
							R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvecs[i].val[1],
							R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvecs[i].val[2]);
				poses.push_back(t);
				ids.push_back(cvIdsPerCam[cam][i]);
				cams.push_back(cam);
				// reconvert in original stitched image
				for(size_t j=0; j<cvCornersPerCam[cam][i].size(); ++j)
				{
					cvCornersPerCam[cam][i][j].x += offsetX;
				}
				corners.push_back(cvCornersPerCam[cam][i]);
			}
		}	
#else
	UERROR("RTAB-Map is not built with \"aruco\" module from OpenCV.");
#endif
	}

	// Estimate scale and fill up output detections
	std::vector<float> scales;
	std::map<int, MarkerInfo> detections;
	for(size_t i=0; i<ids.size(); ++i)
	{
		float length = 0.0f;
		std::map<int, float>::const_iterator findIter = extraMarkerLengths.find(ids[i]);
		if(markerLengths_.empty() && !depth.empty() && (markerLength_ == 0 || (markerLength_<0 && findIter==extraMarkerLengths.end())))
		{
			float d = util2d::getDepth(depth, (corners[i][0].x + (corners[i][2].x-corners[i][0].x)/2.0f)*rgbToDepthFactorX, (corners[i][0].y + (corners[i][2].y-corners[i][0].y)/2.0f)*rgbToDepthFactorY, true, 0.02f, true);
			float d1 = util2d::getDepth(depth, corners[i][0].x*rgbToDepthFactorX, corners[i][0].y*rgbToDepthFactorY, true, 0.02f, true);
			float d2 = util2d::getDepth(depth, corners[i][1].x*rgbToDepthFactorX, corners[i][1].y*rgbToDepthFactorY, true, 0.02f, true);
			float d3 = util2d::getDepth(depth, corners[i][2].x*rgbToDepthFactorX, corners[i][2].y*rgbToDepthFactorY, true, 0.02f, true);
			float d4 = util2d::getDepth(depth, corners[i][3].x*rgbToDepthFactorX, corners[i][3].y*rgbToDepthFactorY, true, 0.02f, true);
			// Accept measurement only if all 4 depth values are valid and
			// they are at the same depth (camera should be perpendicular to marker for
			// best depth estimation)
			if(d>0 && d1>0 && d2>0 && d3>0 && d4>0)
			{
				float scale = d / poses[i].z();
				
				if( fabs(d-d1) < maxDepthError_ &&
					fabs(d-d2) < maxDepthError_ &&
					fabs(d-d3) < maxDepthError_ &&
					fabs(d-d4) < maxDepthError_)
				{
					length = scale;
					scales.push_back(length);
					UWARN("Automatic marker length estimation: id=%d depth=%fm length=%fm", ids[i], d, length);
				}
				else
				{
					UWARN("The four marker's corners should be "
							"perpendicular to camera to estimate correctly "
							"the marker's length. Errors: %f, %f, %f > %fm (%s). Four corners: %f %f %f %f (middle=%f). "
							"Parameter %s can be set to non-null to skip automatic "
							"marker length estimation. Detections are ignored.",
							fabs(d1-d2), fabs(d1-d3), fabs(d1-d4), maxDepthError_, Parameters::kMarkerMaxDepthError().c_str(),
							d1, d2, d3, d4, d,
							Parameters::kMarkerLength().c_str());
					continue;
				}
			}
			else
			{
				UWARN("Some depth values (%f,%f,%f,%f, middle=%f) cannot be detected on the "
						"marker's corners, cannot initialize marker length. "
						"Parameter %s can be set to non-null to skip automatic "
						"marker length estimation. Detections are ignored.",
						d1,d2,d3,d4,d,
						Parameters::kMarkerLength().c_str());
				continue;
			}
		}
		else if(!markerLengths_.empty() || findIter!=extraMarkerLengths.end())
		{
			std::map<int, float>::const_iterator paramIter = markerLengths_.find(ids[i]);
			if(paramIter != markerLengths_.end())
			{
				if(findIter!=extraMarkerLengths.end() && findIter->second != paramIter->second)
				{
					UWARN("Marker's length of %d is defined both in extra lengths "
						"(%f m) and the parameter %s (%f m), we will use the length "
						"from extra lengths.",
						findIter->second,
						Parameters::kMarkerLengths().c_str(),
						paramIter->second);
					length = findIter->second;
				}
				else
				{
					length = paramIter->second;
				}
			}
			else if(findIter!=extraMarkerLengths.end())
			{
				length = findIter->second;
			}
			else
			{
				UERROR("Not supposed to reach this case, ignoring detection %d", ids[i]);
				continue;
			}
		}
		else if(markerLength_ < 0)
		{
			UWARN("Cannot find marker length for marker %d, ignoring this marker (count=%d)", ids[i], (int)extraMarkerLengths.size());
			continue;
		}
		else if(markerLength_ > 0)
		{
			length = markerLength_;
		}
		else
		{
			continue;
		}

		UASSERT(length > 0.0f);
		poses[i].x() *= length;
		poses[i].y() *= length;
		poses[i].z() *= length;

		// Limit the detection range to be between the min / max range.
		// If the ranges are -1, allow any detection within that direction.
		if((maxRange_ <= 0 || poses[i].z() < maxRange_) &&
			(minRange_ <= 0 || poses[i].z() > minRange_))
		{
			Transform pose = models[cams[i]].localTransform() * poses[i];
			detections.insert(std::make_pair(ids[i], MarkerInfo(ids[i], length, pose)));
			UDEBUG("Marker %d detected in base_link: %s, optical_link=%s, local transform=%s",
				ids[i],
				pose.prettyPrint().c_str(),
				poses[i].prettyPrint().c_str(),
				models[cams[i]].localTransform().prettyPrint().c_str());
		}
		else
		{
			UDEBUG("Filtered marker %d by distance (min=%f max=%f value=%f) from the camera %d",
				ids[i], minRange_, maxRange_, poses[i].z(), cams[i]);
		}
	}
	if(markerLength_ == 0 && !scales.empty())
	{
		float sum = 0.0f;
		float maxError = 0.0f;
		for(size_t i=0; i<scales.size(); ++i)
		{
			if(i>0)
			{
				float error = fabs(scales[i]-scales[0]);
				if(error > 0.001f)
				{
					UWARN("The marker's length detected between 2 of the "
							"markers doesn't match (%fm vs %fm)."
							"Parameter %s can be set to non-null to skip automatic "
							"marker length estimation or set to a negative value to "
							"estimate length for each marker. The current %ld detections "
							"are ignored!",
							scales[i], scales[0],
							Parameters::kMarkerLength().c_str(),
							detections.size());
					detections.clear();
					return detections;
				}
				if(error > maxError)
				{
					maxError = error;
				}
			}
			sum += scales[i];
		}
		markerLength_ = sum/float(scales.size());
		UWARN("Final marker length estimated = %fm, max error=%fm (used for subsequent detections)", markerLength_, maxError);
	}

	if(imageWithDetections)
	{
		if(image.channels()==1)
		{
			cv::cvtColor(image, *imageWithDetections, cv::COLOR_GRAY2BGR);
		}
		else
		{
			image.copyTo(*imageWithDetections);
		}

		if(!ids.empty())
		{
#ifdef HAVE_OPENCV_ARUCO
			cv::aruco::drawDetectedMarkers(*imageWithDetections, corners, ids);
#else
			UWARN("RTAB-Map is not built with \"aruco\" module from OpenCV. Cannot draw markers on image.");
#endif
			for(unsigned int i = 0; i < ids.size(); i++)
			{
                std::map<int, MarkerInfo>::iterator iter = detections.find(ids[i]);
                if(iter!=detections.end())
                {
					int cam = cams[i];
					UASSERT(cam >=0 && cam < (int)models.size());
					const CameraModel & model = models[cam];
					int roix = subRGBWidth*cam;
					UASSERT(roix >=0 && roix <= imageWithDetections->cols - subRGBWidth);
					cv::Mat subImage(*imageWithDetections, cv::Rect(roix, 0, subRGBWidth, imageWithDetections->rows));
					cv::Vec3d rvec;
					cv::Vec3d tvec(poses[i].x(), poses[i].y(), poses[i].z());
					cv::Mat R;
					poses[i].rotationMatrix().convertTo(R, CV_64F);
					cv::Rodrigues(R, rvec);			
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && (CV_MINOR_VERSION >1 || (CV_MINOR_VERSION==1 && CV_PATCH_VERSION>=1)))
                    cv::drawFrameAxes(subImage, model.K(), model.D(), rvec, tvec, iter->second.length() * 0.5f);
#elif defined(HAVE_OPENCV_ARUCO)
                    cv::aruco::drawAxis(subImage, model.K(), model.D(), rvec, tvec, iter->second.length() * 0.5f);
#endif
                }
			}
		}
	}

	return detections;
}

std::map<int, MarkerInfo> MarkerDetector::detect(const cv::Mat & image,
                           const CameraModel & model,
                           const cv::Mat & depth,
                           const std::map<int, float> & markerLengths,
                           cv::Mat * imageWithDetections)
{
	UDEBUG("");
	if(!image.empty() && image.cols != model.imageWidth())
	{
		UERROR("This method cannot handle multi-camera marker detection, use the other function version supporting it.");
		return std::map<int, MarkerInfo>();
	}

    std::vector<CameraModel> models;
	models.push_back(model);

	return detect(image, models, depth, markerLengths, imageWithDetections);
}


} /* namespace rtabmap */
