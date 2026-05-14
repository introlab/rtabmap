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

#ifdef RTABMAP_APRILTAG
extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/tag36h11.h"
}
#endif

namespace rtabmap {

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
    detectorParams_->cornerRefinementMethod = (cv::aruco::CornerRefineMethod) Parameters::defaultMarkerCornerRefinementMethod();
#elif CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=3)
	detectorParams_->cornerRefinementMethod = Parameters::defaultMarkerCornerRefinementMethod();
#else
	detectorParams_->doCornerRefinement = Parameters::defaultMarkerCornerRefinementMethod()!=0;
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
		tag36h11_destroy((apriltag_family_t*)apriltagLibFamily_);
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
    Parameters::parse(parameters, Parameters::kMarkerCornerRefinementMethod(), cornerRefinementMethod);
    detectorParams_->cornerRefinementMethod = (cv::aruco::CornerRefineMethod)cornerRefinementMethod;
#elif CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=3)
	Parameters::parse(parameters, Parameters::kMarkerCornerRefinementMethod(), detectorParams_->cornerRefinementMethod);
#else
	int doCornerRefinement = detectorParams_->doCornerRefinement?1:0;
	Parameters::parse(parameters, Parameters::kMarkerCornerRefinementMethod(), doCornerRefinement);
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
	if(dictionaryId_ >= 17 && strategy_ == 0)
	{
		UERROR("Cannot set AprilTag dictionary. OpenCV version should be at least 3.4.2, "
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
	dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId_));
#else
	dictionary_.reset(new cv::aruco::Dictionary());
	*dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId_));
#endif
#endif

#ifdef RTABMAP_APRILTAG
	if(apriltagLibDetector_)
	{
		apriltag_detector_destroy(((apriltag_detector_t*)apriltagLibDetector_));
		apriltagLibDetector_ = NULL;
	}
	if(apriltagLibFamily_)
	{
		tag36h11_destroy((apriltag_family_t*)apriltagLibFamily_);
		apriltagLibFamily_ = NULL;
	}
	apriltagLibDetector_ = apriltag_detector_create();
	((apriltag_detector_t*)apriltagLibDetector_)->nthreads = 2;
	((apriltag_detector_t*)apriltagLibDetector_)->quad_decimate = 1;
	apriltagLibFamily_  = tag36h11_create();
	apriltag_detector_add_family(((apriltag_detector_t*)apriltagLibDetector_), (apriltag_family_t*)apriltagLibFamily_);

	if (errno == ENOMEM) {
		UFATAL("Unable to add family to detector due to insufficient memory to allocate the tag-family decoder with the default maximum hamming value of 2. Try choosing an alternative tag family.");
	}
#endif
}

// deprecated
std::map<int, Transform> MarkerDetector::detect(const cv::Mat & image, const CameraModel & model, const cv::Mat & depth, float * markerLengthOut, cv::Mat * imageWithDetections)
{
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
                           const std::map<int, float> & markerLengths,
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
	std::vector<Transform> poses;

	// detect markers and estimate pose
	if(strategy_ == kStrategyApriltag)
	{
#ifdef RTABMAP_APRILTAG
		UASSERT(image.type() == CV_8UC1);
		image_u8_t im = {image.cols, image.rows, (int)image.step, image.data};

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
			UASSERT(cameraIndex>=0 && cameraIndex<(int)models.size());

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

			// FIXME, homography needs to be updated

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
				Transform t(MATD_EL(pose.R, 0, 0), MATD_EL(pose.R, 1, 0), MATD_EL(pose.R, 2, 0), MATD_EL(pose.t, 0, 0),
							MATD_EL(pose.R, 0, 1), MATD_EL(pose.R, 1, 1), MATD_EL(pose.R, 2, 1), MATD_EL(pose.t, 1, 0),
							MATD_EL(pose.R, 0, 2), MATD_EL(pose.R, 1, 2), MATD_EL(pose.R, 2, 2), MATD_EL(pose.t, 2, 0));
				poses.push_back(t);

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
			UASSERT(cameraIndex>=0 && cameraIndex<(int)models.size());

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
		std::map<int, float>::const_iterator findIter = markerLengths.find(ids[i]);
		if(!depth.empty() && (markerLength_ == 0 || (markerLength_<0 && findIter==markerLengths.end())))
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
		else if(markerLength_ < 0)
		{
			if(findIter!=markerLengths.end())
			{
				length = findIter->second;
			}
			else
			{
				UWARN("Cannot find marker length for marker %d, ignoring this marker (count=%d)", ids[i], (int)markerLengths.size());
				continue;
			}
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
#ifdef HAVE_OPENCV_ARUCO
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
			cv::aruco::drawDetectedMarkers(*imageWithDetections, corners, ids);

			for(unsigned int i = 0; i < ids.size(); i++)
			{
                std::map<int, MarkerInfo>::iterator iter = detections.find(ids[i]);
                if(iter!=detections.end())
                {
					int cam = cams[i];
					const CameraModel & model = models[cam];
					cv::Mat subImage(*imageWithDetections, cv::Rect(subRGBWidth*i, 0, subRGBWidth, imageWithDetections->rows));
					cv::Vec3d rvec;
					cv::Vec3d tvec(poses[i].x(), poses[i].y(), poses[i].z());
					cv::Rodrigues(poses[i].rotationMatrix(), rvec);								
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && (CV_MINOR_VERSION >1 || (CV_MINOR_VERSION==1 && CV_PATCH_VERSION>=1)))
                    cv::drawFrameAxes(subImage, model.K(), model.D(), rvec, tvec, iter->second.length() * 0.5f);
#else
                    cv::aruco::drawAxis(subImage, model.K(), model.D(), rvec, tvec, iter->second.length() * 0.5f);
#endif
                }
			}
		}
#else
		UERROR("RTAB-Map is not built with \"aruco\" module from OpenCV. Cannot draw markers on image.");
#endif
	}

	return detections;
}

std::map<int, MarkerInfo> MarkerDetector::detect(const cv::Mat & image,
                           const CameraModel & model,
                           const cv::Mat & depth,
                           const std::map<int, float> & markerLengths,
                           cv::Mat * imageWithDetections)
{
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
