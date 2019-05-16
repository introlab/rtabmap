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

namespace rtabmap {

MarkerDetector::MarkerDetector(const ParametersMap & parameters)
{
#ifdef HAVE_OPENCV_ARUCO
	markerLength_ = Parameters::defaultArucoMarkerLength();
	maxDepthError_ = Parameters::defaultArucoMaxDepthError();
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
	Parameters::parse(parameters, Parameters::kArucoMaxDepthError(), maxDepthError_);
	Parameters::parse(parameters, Parameters::kArucoDictionary(), dictionaryId_);
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION <4 || (CV_MINOR_VERSION ==4 && CV_SUBMINOR_VERSION<2)))
	if(dictionaryId_ >= 17)
	{
		UERROR("Cannot set AprilTag dictionary. OpenCV version should be at least 3.4.2, "
				"current version is %s. Setting %s to default (%d)",
				CV_VERSION,
				Parameters::kArucoDictionary().c_str(),
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

std::map<int, Transform> MarkerDetector::detect(const cv::Mat & image, const CameraModel & model, const cv::Mat & depth, float * markerLengthOut, cv::Mat * imageWithDetections)
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
		float rgbToDepthFactorX = 1.0f;
		float rgbToDepthFactorY = 1.0f;
		if(markerLength_ == 0)
		{
			if(depth.empty())
			{
				UERROR("Depth image is empty, please set %s parameter to non-null.", Parameters::kArucoMarkerLength().c_str());
				return detections;
			}
			rgbToDepthFactorX = 1.0f/(model.imageWidth()>0?model.imageWidth()/depth.cols:1);
			rgbToDepthFactorY = 1.0f/(model.imageHeight()>0?model.imageHeight()/depth.rows:1);
		}

		cv::aruco::estimatePoseSingleMarkers(corners, markerLength_==0?1.0f:markerLength_, model.K(), model.D(), rvecs, tvecs);
		std::vector<float> scales;
		for(size_t i=0; i<ids.size(); ++i)
		{
			if(markerLength_ == 0)
			{
				float d1 = util2d::getDepth(depth, corners[i][0].x*rgbToDepthFactorX, corners[i][0].y*rgbToDepthFactorY, true, 0.02f, true);
				float d2 = util2d::getDepth(depth, corners[i][1].x*rgbToDepthFactorX, corners[i][1].y*rgbToDepthFactorY, true, 0.02f, true);
				float d3 = util2d::getDepth(depth, corners[i][2].x*rgbToDepthFactorX, corners[i][2].y*rgbToDepthFactorY, true, 0.02f, true);
				float d4 = util2d::getDepth(depth, corners[i][3].x*rgbToDepthFactorX, corners[i][3].y*rgbToDepthFactorY, true, 0.02f, true);
				// Accept measurement only if all 4 depth values are valid and
				// they are at the same depth (camera should be perpendicular to marker for
				// best depth estimation)
				if(d1>0 && d2>0 && d3>0 && d4>0)
				{
					if( fabs(d1-d2) < maxDepthError_ &&
						fabs(d1-d3) < maxDepthError_ &&
						fabs(d1-d4) < maxDepthError_)
					{
						float depth = (d1+d2+d3+d4)/4.0f;
						scales.push_back(depth/tvecs[i].val[2]);
						tvecs[i] *= scales.back();
						UWARN("Automatic marker length estimation: id=%d depth=%fm length=%fm", ids[i], depth, scales.back());
					}
					else
					{
						UWARN("The four marker's corners should be "
							  "perpendicular to camera to estimate correctly "
							  "the marker's length. Errors: %f, %f, %f > %fm (%s). Four corners: %f %f %f %f. "
							  "Parameter %s can be set to non-null to skip automatic "
							  "marker length estimation. Detections are ignored.",
							  fabs(d1-d2), fabs(d1-d3), fabs(d1-d4), maxDepthError_, Parameters::kArucoMaxDepthError().c_str(),
							  d1, d2, d3, d4,
								Parameters::kArucoMarkerLength().c_str());
						detections.clear();
						return detections;
					}
				}
				else
				{
					UWARN("Some depth values (%f,%f,%f,%f) cannot be detected on the "
						  "marker's corners, cannot initialize marker length. "
						  "Parameter %s can be set to non-null to skip automatic "
						  "marker length estimation. Detections are ignored.",
						  d1,d2,d3,d4,
						  Parameters::kArucoMarkerLength().c_str());
					detections.clear();
					return detections;
				}
			}

			cv::Mat R;
			cv::Rodrigues(rvecs[i], R);
			Transform t(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvecs[i].val[0],
						   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvecs[i].val[1],
						   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvecs[i].val[2]);

			Transform pose = model.localTransform() * t;
			detections.insert(std::make_pair(ids[i], pose));
			UDEBUG("Marker %d detected at %s (%s)", ids[i], pose.prettyPrint().c_str(), t.prettyPrint().c_str());
		}
		if(markerLength_ == 0)
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
							  "markers doesn't match (%d->%fm vs %d->%fm)."
							  "Parameter %s can be set to non-null to skip automatic "
							  "marker length estimation. Detections are ignored.",
							  ids[i], scales[i], ids[0], scales[0],
							  Parameters::kArucoMarkerLength().c_str());
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
	}

	if(markerLengthOut)
	{
		*markerLengthOut = markerLength_;
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

