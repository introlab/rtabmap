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
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

MarkerDetector::MarkerDetector(const ParametersMap & parameters)
{
#ifdef HAVE_OPENCV_ARUCO
	markerLength_ = Parameters::defaultArucoMarkerLength();
	dictionaryId_ = Parameters::defaultArucoDictionary();
#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=2)
	detectorParams_ = cv::aruco::DetectorParameters::create();
#else
	detectorParams_.reset(new cv::aruco::DetectorParameters());
#endif
#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=3)
	detectorParams_->cornerRefinementMethod = Parameters::defaultArucoCornerRefinementMethod();
#else
	detectorParams_->doCornerRefinement = Parameters::defaultArucoCornerRefinementMethod()!=0;
#endif
	parseParameters(parameters);
#endif
}

MarkerDetector::~MarkerDetector() {

}

void MarkerDetector::parseParameters(const ParametersMap & parameters)
{
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
#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=3)
	Parameters::parse(parameters, Parameters::kArucoCornerRefinementMethod(), detectorParams_->cornerRefinementMethod);
#else
	int doCornerRefinement = detectorParams_->doCornerRefinement?1:0;
	Parameters::parse(parameters, Parameters::kArucoCornerRefinementMethod(), doCornerRefinement);
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

	Parameters::parse(parameters, Parameters::kArucoMarkerLength(), markerLength_);
	Parameters::parse(parameters, Parameters::kArucoDictionary(), dictionaryId_);
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION <4 || (CV_MINOR_VERSION ==4 && CV_SUBMINOR_VERSION<2)))
	if(dictionaryId_ >= 17)
	{
		UERROR("Cannot set AprilTag dictionary. OpenCV version should be at least 3.4.2, "
				"current version is %s. Setting dictionary type to default (%d)",
				CV_VERSION,
				Parameters::defaultArucoDictionary());
		dictionaryId_ = Parameters::defaultArucoDictionary();
	}
#endif
#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=2)
	dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId_));
#else
	dictionary_.reset(new cv::aruco::Dictionary());
	*dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId_));
#endif
#endif
}

std::map<int, Transform> MarkerDetector::detect(const cv::Mat & image, const CameraModel & model, cv::Mat * imageWithDetections)
{
	std::map<int, Transform> detections;

#ifdef HAVE_OPENCV_ARUCO

	std::vector< int > ids;
	std::vector< std::vector< cv::Point2f > > corners, rejected;
	std::vector< cv::Vec3d > rvecs, tvecs;

	// detect markers and estimate pose
#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=2)
	cv::aruco::detectMarkers(image, dictionary_, corners, ids, detectorParams_, rejected);
#else
	cv::aruco::detectMarkers(image, *dictionary_, corners, ids, *detectorParams_, rejected);
#endif
	UDEBUG("Markers detected=%d rejected=%d", (int)ids.size(), (int)rejected.size());
	if(ids.size() > 0)
	{
		cv::aruco::estimatePoseSingleMarkers(corners, markerLength_, model.K(), model.D(), rvecs, tvecs);
		for(size_t i=0; i<ids.size(); ++i)
		{
			cv::Mat R;
			cv::Rodrigues(rvecs[i], R);
			Transform t(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvecs[i].val[0],
						   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvecs[i].val[1],
						   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvecs[i].val[2]);

			Transform pose = model.localTransform() * t;
			detections.insert(std::make_pair(ids[i], pose));
			UDEBUG("Marker %d detected at %s (%s)", ids[i], pose.prettyPrint().c_str(), t.prettyPrint().c_str());
		}
	}

	if(imageWithDetections)
	{
		image.copyTo(*imageWithDetections);
		if(ids.size() > 0)
		{
			cv::aruco::drawDetectedMarkers(*imageWithDetections, corners, ids);

			for(unsigned int i = 0; i < ids.size(); i++)
			{
				cv::aruco::drawAxis(*imageWithDetections, model.K(), model.D(), rvecs[i], tvecs[i], markerLength_ * 0.5f);
			}
		}
	}

#else
	UERROR("RTAB-Map is not built with \"aruco\" module from OpenCV.");
#endif

	return detections;
}


} /* namespace rtabmap */

