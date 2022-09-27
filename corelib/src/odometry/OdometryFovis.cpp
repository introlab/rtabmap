/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/odometry/OdometryFovis.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_FOVIS
#include <libfovis/fovis.hpp>
#endif

namespace rtabmap {

OdometryFovis::OdometryFovis(const ParametersMap & parameters) :
	Odometry(parameters)
#ifdef RTABMAP_FOVIS
    ,
	fovis_(0),
	rect_(0),
	stereoCalib_(0),
	depthImage_(0),
	stereoDepth_(0),
	lost_(false)
#endif
{
	fovisParameters_ = Parameters::filterParameters(parameters, "OdomFovis");
	if(parameters.find(Parameters::kOdomVisKeyFrameThr()) != parameters.end())
	{
		fovisParameters_.insert(*parameters.find(Parameters::kOdomVisKeyFrameThr()));
	}
}

OdometryFovis::~OdometryFovis()
{
#ifdef RTABMAP_FOVIS
	delete fovis_;
	delete rect_;
	delete stereoCalib_;
	delete depthImage_;
	delete stereoDepth_;
#endif
}

void OdometryFovis::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_FOVIS
	if(fovis_)
	{
		delete fovis_;
		fovis_ = 0;
	}
	if(rect_)
	{
		delete rect_;
		rect_ = 0;
	}
	if(stereoCalib_)
	{
		delete stereoCalib_;
		stereoCalib_ = 0;
	}
	if(depthImage_)
	{
		delete depthImage_;
		depthImage_ = 0;
	}
	if(stereoDepth_)
	{
		delete stereoDepth_;
		stereoDepth_ = 0;
	}
	lost_ = false;
	previousLocalTransform_.setNull();
#endif
}

// return not null transform if odometry is correctly computed
Transform OdometryFovis::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	UDEBUG("");
	Transform t;

#ifdef RTABMAP_FOVIS
	UTimer timer;

	if(data.imageRaw().empty() ||
		data.imageRaw().rows != data.depthOrRightRaw().rows ||
		data.imageRaw().cols != data.depthOrRightRaw().cols)
	{
		UERROR("Not supported input!");
		return t;
	}

	if(!((data.cameraModels().size() == 1 && data.cameraModels()[0].isValidForReprojection()) ||
		(data.stereoCameraModels().size() == 1 && data.stereoCameraModels()[0].isValidForProjection())))
	{
		UERROR("Invalid camera model! Mono cameras=%d (reproj=%d), Stereo cameras=%d (reproj=%d)",
				(int)data.cameraModels().size(),
				data.cameraModels().size() && data.cameraModels()[0].isValidForReprojection()?1:0,
				(int)data.stereoCameraModels().size(),
				data.stereoCameraModels().size() && data.stereoCameraModels()[0].isValidForProjection()?1:0);
		return t;
	}

	cv::Mat gray;
	if(data.imageRaw().type() == CV_8UC3)
	{
		cv::cvtColor(data.imageRaw(), gray, CV_BGR2GRAY);
	}
	else if(data.imageRaw().type() == CV_8UC1)
	{
		gray = data.imageRaw();
	}
	else
	{
		UFATAL("Not supported color type!");
	}

	fovis::VisualOdometryOptions options;
	if(fovis_ == 0 || (data.cameraModels().size() != 1 && stereoDepth_ == 0))
	{
		options = fovis::VisualOdometry::getDefaultOptions();

		ParametersMap defaults = Parameters::getDefaultParameters("OdomFovis");
		options["feature-window-size"]           = uValue(fovisParameters_, Parameters::kOdomFovisFeatureWindowSize(), defaults.at(Parameters::kOdomFovisFeatureWindowSize()));
		options["max-pyramid-level"]             = uValue(fovisParameters_, Parameters::kOdomFovisMaxPyramidLevel(), defaults.at(Parameters::kOdomFovisMaxPyramidLevel()));
		options["min-pyramid-level"]             = uValue(fovisParameters_, Parameters::kOdomFovisMinPyramidLevel(), defaults.at(Parameters::kOdomFovisMinPyramidLevel()));
		options["target-pixels-per-feature"]     = uValue(fovisParameters_, Parameters::kOdomFovisTargetPixelsPerFeature(), defaults.at(Parameters::kOdomFovisTargetPixelsPerFeature()));
		options["fast-threshold"]                = uValue(fovisParameters_, Parameters::kOdomFovisFastThreshold(), defaults.at(Parameters::kOdomFovisFastThreshold()));
		options["use-adaptive-threshold"]        = uValue(fovisParameters_, Parameters::kOdomFovisUseAdaptiveThreshold(), defaults.at(Parameters::kOdomFovisUseAdaptiveThreshold()));
		options["fast-threshold-adaptive-gain"]  = uValue(fovisParameters_, Parameters::kOdomFovisFastThresholdAdaptiveGain(), defaults.at(Parameters::kOdomFovisFastThresholdAdaptiveGain()));
		options["use-homography-initialization"] = uValue(fovisParameters_, Parameters::kOdomFovisUseHomographyInitialization(), defaults.at(Parameters::kOdomFovisUseHomographyInitialization()));
		options["ref-frame-change-threshold"]    = uValue(fovisParameters_, Parameters::kOdomVisKeyFrameThr(), uNumber2Str(Parameters::defaultOdomVisKeyFrameThr()));

		  // OdometryFrame
		options["use-bucketing"]            = uValue(fovisParameters_, Parameters::kOdomFovisUseBucketing(), defaults.at(Parameters::kOdomFovisUseBucketing()));
		options["bucket-width"]             = uValue(fovisParameters_, Parameters::kOdomFovisBucketWidth(), defaults.at(Parameters::kOdomFovisBucketWidth()));
		options["bucket-height"]            = uValue(fovisParameters_, Parameters::kOdomFovisBucketHeight(), defaults.at(Parameters::kOdomFovisBucketHeight()));
		options["max-keypoints-per-bucket"] = uValue(fovisParameters_, Parameters::kOdomFovisMaxKeypointsPerBucket(), defaults.at(Parameters::kOdomFovisMaxKeypointsPerBucket()));
		options["use-image-normalization"]  = uValue(fovisParameters_, Parameters::kOdomFovisUseImageNormalization(), defaults.at(Parameters::kOdomFovisUseImageNormalization()));

		  // MotionEstimator
		options["inlier-max-reprojection-error"]       = uValue(fovisParameters_, Parameters::kOdomFovisInlierMaxReprojectionError(), defaults.at(Parameters::kOdomFovisInlierMaxReprojectionError()));
		options["clique-inlier-threshold"]             = uValue(fovisParameters_, Parameters::kOdomFovisCliqueInlierThreshold(), defaults.at(Parameters::kOdomFovisCliqueInlierThreshold()));
		options["min-features-for-estimate"]           = uValue(fovisParameters_, Parameters::kOdomFovisMinFeaturesForEstimate(), defaults.at(Parameters::kOdomFovisMinFeaturesForEstimate()));
		options["max-mean-reprojection-error"]         = uValue(fovisParameters_, Parameters::kOdomFovisMaxMeanReprojectionError(), defaults.at(Parameters::kOdomFovisMaxMeanReprojectionError()));
		options["use-subpixel-refinement"]             = uValue(fovisParameters_, Parameters::kOdomFovisUseSubpixelRefinement(), defaults.at(Parameters::kOdomFovisUseSubpixelRefinement()));
		options["feature-search-window"]               = uValue(fovisParameters_, Parameters::kOdomFovisFeatureSearchWindow(), defaults.at(Parameters::kOdomFovisFeatureSearchWindow()));
		options["update-target-features-with-refined"] = uValue(fovisParameters_, Parameters::kOdomFovisUpdateTargetFeaturesWithRefined(), defaults.at(Parameters::kOdomFovisUpdateTargetFeaturesWithRefined()));

		  // StereoDepth
		options["stereo-require-mutual-match"]        = uValue(fovisParameters_, Parameters::kOdomFovisStereoRequireMutualMatch(), defaults.at(Parameters::kOdomFovisStereoRequireMutualMatch()));
		options["stereo-max-dist-epipolar-line"]      = uValue(fovisParameters_, Parameters::kOdomFovisStereoMaxDistEpipolarLine(), defaults.at(Parameters::kOdomFovisStereoMaxDistEpipolarLine()));
		options["stereo-max-refinement-displacement"] = uValue(fovisParameters_, Parameters::kOdomFovisStereoMaxRefinementDisplacement(), defaults.at(Parameters::kOdomFovisStereoMaxRefinementDisplacement()));
		options["stereo-max-disparity"]               = uValue(fovisParameters_, Parameters::kOdomFovisStereoMaxDisparity(), defaults.at(Parameters::kOdomFovisStereoMaxDisparity()));
	}

	fovis::DepthSource * depthSource = 0;
	cv::Mat depth;
	cv::Mat right;
	Transform localTransform = Transform::getIdentity();
	if(data.cameraModels().size() == 1) //depth
	{
		UDEBUG("");
		fovis::CameraIntrinsicsParameters rgb_params;
		memset(&rgb_params, 0, sizeof(fovis::CameraIntrinsicsParameters));
		rgb_params.width = data.cameraModels()[0].imageWidth();
		rgb_params.height = data.cameraModels()[0].imageHeight();

		rgb_params.fx = data.cameraModels()[0].fx();
		rgb_params.fy = data.cameraModels()[0].fy();
		rgb_params.cx = data.cameraModels()[0].cx()==0.0?double(rgb_params.width) / 2.0:data.cameraModels()[0].cx();
		rgb_params.cy = data.cameraModels()[0].cy()==0.0?double(rgb_params.height) / 2.0:data.cameraModels()[0].cy();
		localTransform = data.cameraModels()[0].localTransform();

		if(rect_ == 0)
		{
			UINFO("Init rgbd fovis: %dx%d fx=%f fy=%f cx=%f cy=%f", rgb_params.width, rgb_params.height, rgb_params.fx, rgb_params.fy, rgb_params.cx, rgb_params.cy);
			rect_ = new fovis::Rectification(rgb_params);
		}

		if(depthImage_ == 0)
		{
			depthImage_ = new fovis::DepthImage(rgb_params, rgb_params.width, rgb_params.height);
		}
		// make sure all zeros are NAN
		if(data.depthRaw().type() == CV_32FC1)
		{
			depth = data.depthRaw();
			for(int i=0; i<depth.rows; ++i)
			{
				for(int j=0; j<depth.cols; ++j)
				{
					float & d = depth.at<float>(i,j);
					if(d == 0.0f)
					{
						d = NAN;
					}
				}
			}
		}
		else if(data.depthRaw().type() == CV_16UC1)
		{
			depth = cv::Mat(data.depthRaw().size(), CV_32FC1);
			for(int i=0; i<data.depthRaw().rows; ++i)
			{
				for(int j=0; j<data.depthRaw().cols; ++j)
				{
					float d = float(data.depthRaw().at<unsigned short>(i,j))/1000.0f;
					depth.at<float>(i, j) = d==0.0f?NAN:d;
				}
			}
		}
		else
		{
			UFATAL("Unknown depth format!");
		}
		depthImage_->setDepthImage((float*)depth.data);
		depthSource = depthImage_;
	}
	else if(data.stereoCameraModels().size() == 1) // stereo
	{
		UDEBUG("");
		// initialize left camera parameters
		fovis::CameraIntrinsicsParameters left_parameters;
		left_parameters.width = data.stereoCameraModels()[0].left().imageWidth();
		left_parameters.height = data.stereoCameraModels()[0].left().imageHeight();
		left_parameters.fx = data.stereoCameraModels()[0].left().fx();
		left_parameters.fy = data.stereoCameraModels()[0].left().fy();
		left_parameters.cx = data.stereoCameraModels()[0].left().cx()==0.0?double(left_parameters.width) / 2.0:data.stereoCameraModels()[0].left().cx();
		left_parameters.cy = data.stereoCameraModels()[0].left().cy()==0.0?double(left_parameters.height) / 2.0:data.stereoCameraModels()[0].left().cy();
		localTransform = data.stereoCameraModels()[0].localTransform();

		if(rect_ == 0)
		{
			UINFO("Init stereo fovis: %dx%d fx=%f fy=%f cx=%f cy=%f", left_parameters.width, left_parameters.height, left_parameters.fx, left_parameters.fy, left_parameters.cx, left_parameters.cy);
			rect_ = new fovis::Rectification(left_parameters);
		}

		if(stereoCalib_ == 0)
		{
			// initialize right camera parameters
			fovis::CameraIntrinsicsParameters right_parameters;
			right_parameters.width = data.stereoCameraModels()[0].right().imageWidth();
			right_parameters.height = data.stereoCameraModels()[0].right().imageHeight();
			right_parameters.fx = data.stereoCameraModels()[0].right().fx();
			right_parameters.fy = data.stereoCameraModels()[0].right().fy();
			right_parameters.cx = data.stereoCameraModels()[0].right().cx()==0.0?double(right_parameters.width) / 2.0:data.stereoCameraModels()[0].right().cx();
			right_parameters.cy = data.stereoCameraModels()[0].right().cy()==0.0?double(right_parameters.height) / 2.0:data.stereoCameraModels()[0].right().cy();

			// as we use rectified images, rotation is identity
			// and translation is baseline only
			fovis::StereoCalibrationParameters stereo_parameters;
			stereo_parameters.left_parameters = left_parameters;
			stereo_parameters.right_parameters = right_parameters;
			stereo_parameters.right_to_left_rotation[0] = 1.0;
			stereo_parameters.right_to_left_rotation[1] = 0.0;
			stereo_parameters.right_to_left_rotation[2] = 0.0;
			stereo_parameters.right_to_left_rotation[3] = 0.0;
			stereo_parameters.right_to_left_translation[0] = -data.stereoCameraModels()[0].baseline();
			stereo_parameters.right_to_left_translation[1] = 0.0;
			stereo_parameters.right_to_left_translation[2] = 0.0;

			stereoCalib_ = new fovis::StereoCalibration(stereo_parameters);
		}

		if(stereoDepth_ == 0)
		{
			stereoDepth_ = new fovis::StereoDepth(stereoCalib_, options);
		}
		if(data.rightRaw().type() == CV_8UC3)
		{
			cv::cvtColor(data.rightRaw(), right, CV_BGR2GRAY);
		}
		else if(data.rightRaw().type() == CV_8UC1)
		{
			right = data.rightRaw();
		}
		else
		{
			UFATAL("Not supported color type!");
		}
		stereoDepth_->setRightImage(right.data);
		depthSource = stereoDepth_;
	}

	if(fovis_ == 0)
	{
		fovis_ = new fovis::VisualOdometry(rect_, options);
	}

	UDEBUG("");
	fovis_->processFrame(gray.data, depthSource);

	// get the motion estimate for this frame to the previous frame.
	t = Transform::fromEigen3d(fovis_->getMotionEstimate());

	cv::Mat covariance;
	fovis::MotionEstimateStatusCode statusCode = fovis_->getMotionEstimator()->getMotionEstimateStatus();
	if(statusCode > fovis::SUCCESS)
	{
		UWARN("Fovis error status: %s", fovis::MotionEstimateStatusCodeStrings[statusCode]);
		t.setNull();
		lost_ = true;
		covariance = cv::Mat::eye(6,6, CV_64FC1)*9999.0;
		previousLocalTransform_.setNull();
	}
	else if(lost_)
	{
		lost_ = false;
		// we are not lost anymore but we don't know where we are now according to last valid pose
		covariance = cv::Mat::eye(6,6, CV_64FC1)*9999.0;
		previousLocalTransform_.setNull();
	}
	else
	{
		const Eigen::MatrixXd& cov = fovis_->getMotionEstimator()->getMotionEstimateCov();
		if(cov.cols() == 6 && cov.rows() == 6 && cov(0,0) > 0.0)
		{
			covariance = cv::Mat::eye(6,6, CV_64FC1);
			memcpy(covariance.data, cov.data(), 36*sizeof(double));
			covariance *= 100.0; // to be in the same scale than loop closure detection
		}
	}

	if(!t.isNull() && !t.isIdentity() && !localTransform.isIdentity() && !localTransform.isNull())
	{
		// from camera frame to base frame
		if(!previousLocalTransform_.isNull())
		{
			t = previousLocalTransform_ * t * localTransform.inverse();
		}
		else
		{
			t = localTransform * t * localTransform.inverse();
		}
		previousLocalTransform_ = localTransform;
	}

	if(info)
	{
		info->type = (int)kTypeFovis;
		info->keyFrameAdded = fovis_->getChangeReferenceFrames();
		info->features = fovis_->getTargetFrame()->getNumDetectedKeypoints();
		info->reg.matches = fovis_->getMotionEstimator()->getNumMatches();
		info->reg.inliers = fovis_->getMotionEstimator()->getNumInliers();
		info->reg.covariance = covariance;

		if(this->isInfoDataFilled())
		{
			const fovis::FeatureMatch * matches = fovis_->getMotionEstimator()->getMatches();
			int numMatches = fovis_->getMotionEstimator()->getNumMatches();
			if(matches && numMatches>0)
			{
				info->refCorners.resize(numMatches);
				info->newCorners.resize(numMatches);
				info->cornerInliers.resize(numMatches);
				int oi=0;
				for (int i = 0; i < numMatches; ++i)
				{
					info->refCorners[i].x = matches[i].ref_keypoint->base_uv[0];
					info->refCorners[i].y = matches[i].ref_keypoint->base_uv[1];
					info->newCorners[i].x = matches[i].target_keypoint->base_uv[0];
					info->newCorners[i].y = matches[i].target_keypoint->base_uv[1];
					info->cornerInliers[oi++] = i;
				}
				info->cornerInliers.resize(oi);
			}
		}
	}

	UINFO("Odom update time = %fs status=%s", timer.elapsed(), fovis::MotionEstimateStatusCodeStrings[statusCode]);

#else
	UERROR("RTAB-Map is not built with FOVIS support! Select another visual odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
