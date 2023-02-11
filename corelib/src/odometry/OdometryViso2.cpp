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

#include "rtabmap/core/odometry/OdometryViso2.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_VISO2
#include <viso_stereo.h>

double computeFeatureFlow(const std::vector<Matcher::p_match>& matches)
  {
    double total_flow = 0.0;
    for (size_t i = 0; i < matches.size(); ++i)
    {
      double x_diff = matches[i].u1c - matches[i].u1p;
      double y_diff = matches[i].v1c - matches[i].v1p;
      total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
    }
    return total_flow / matches.size();
}
#endif

namespace rtabmap {

OdometryViso2::OdometryViso2(const ParametersMap & parameters) :
	Odometry(parameters),
#ifdef RTABMAP_VISO2
	viso2_(0),
	ref_frame_change_method_(0),
	ref_frame_inlier_threshold_(Parameters::defaultOdomVisKeyFrameThr()),
	ref_frame_motion_threshold_(5.0),
	lost_(false),
	keep_reference_frame_(false),
#endif
	reference_motion_(Transform::getIdentity())
{
#ifdef RTABMAP_VISO2
	Parameters::parse(parameters, Parameters::kOdomVisKeyFrameThr(), ref_frame_inlier_threshold_);
#endif
	viso2Parameters_ = Parameters::filterParameters(parameters, "OdomViso2");
}

OdometryViso2::~OdometryViso2()
{
#ifdef RTABMAP_VISO2
	delete viso2_;
#endif
}

void OdometryViso2::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_VISO2
	if(viso2_)
	{
		delete viso2_;
		viso2_ = 0;
	}
	lost_ = false;
	reference_motion_.setIdentity();
	previousLocalTransform_.setNull();
#endif
}

// return not null transform if odometry is correctly computed
Transform OdometryViso2::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	Transform t;
#ifdef RTABMAP_VISO2
	//based on https://github.com/srv/viso2/blob/indigo/viso2_ros/src/stereo_odometer.cpp

	UTimer timer;

	if(!data.depthRaw().empty())
	{
		UERROR("viso2 odometry doesn't support RGB-D data, only stereo. Aborting odometry update...");
		return t;
	}

	if(data.imageRaw().empty() ||
		data.imageRaw().rows != data.rightRaw().rows ||
		data.imageRaw().cols != data.rightRaw().cols)
	{
		UERROR("Not compatible left (%dx%d) or right (%dx%d) image.",
				data.imageRaw().rows,
				data.imageRaw().cols,
				data.rightRaw().rows,
				data.rightRaw().cols);
		return t;
	}

	if(!(data.stereoCameraModels().size() == 1 &&
		data.stereoCameraModels()[0].isValidForProjection()))
	{
		UERROR("Invalid stereo camera model!");
		return t;
	}

	cv::Mat leftGray;
	if(data.imageRaw().type() == CV_8UC3)
	{
		cv::cvtColor(data.imageRaw(), leftGray, CV_BGR2GRAY);
	}
	else if(data.imageRaw().type() == CV_8UC1)
	{
		leftGray = data.imageRaw();
	}
	else
	{
		UFATAL("Not supported color type!");
	}
	cv::Mat rightGray;
	if(data.rightRaw().type() == CV_8UC3)
	{
		cv::cvtColor(data.rightRaw(), rightGray, CV_BGR2GRAY);
	}
	else if(data.rightRaw().type() == CV_8UC1)
	{
		rightGray = data.rightRaw();
	}
	else
	{
		UFATAL("Not supported color type!");
	}

	int32_t dims[] = {leftGray.cols, leftGray.rows, leftGray.cols};
	cv::Mat covariance;
	if(viso2_ == 0)
	{
		VisualOdometryStereo::parameters params;
		params.base      = params.match.base = data.stereoCameraModels()[0].baseline();
		params.calib.cu  = params.match.cu = data.stereoCameraModels()[0].left().cx();
		params.calib.cv  = params.match.cv = data.stereoCameraModels()[0].left().cy();
		params.calib.f   = params.match.f = data.stereoCameraModels()[0].left().fx();

		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2RansacIters(), params.ransac_iters);
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2InlierThreshold(), params.inlier_threshold);
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2Reweighting(), params.reweighting);

		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2MatchNmsN(), params.match.nms_n);
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2MatchNmsTau(), params.match.nms_tau);
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2MatchBinsize(), params.match.match_binsize);
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2MatchRadius(), params.match.match_radius);
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2MatchDispTolerance(), params.match.match_disp_tolerance);
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2MatchOutlierDispTolerance(), params.match.outlier_disp_tolerance);
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2MatchOutlierFlowTolerance(), params.match.outlier_flow_tolerance);
		bool multistage = Parameters::defaultOdomViso2MatchMultiStage();
		bool halfResolution = Parameters::defaultOdomViso2MatchHalfResolution();
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2MatchMultiStage(), multistage);
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2MatchHalfResolution() , halfResolution);
		params.match.multi_stage = multistage?1:0;
		params.match.half_resolution = halfResolution?1:0;
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2MatchRefinement(), params.match.refinement);

		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2BucketMaxFeatures(), params.bucket.max_features);
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2BucketWidth(), params.bucket.bucket_width);
		Parameters::parse(viso2Parameters_, Parameters::kOdomViso2BucketHeight(), params.bucket.bucket_height);

		viso2_ = new VisualOdometryStereo(params);

		viso2_->process(leftGray.data, rightGray.data, dims);
		t.setIdentity();
		covariance = cv::Mat::eye(6,6, CV_64FC1)*9999.0;
	}
	else
	{
		bool success = viso2_->process(leftGray.data, rightGray.data, dims, lost_ || keep_reference_frame_);
		if (success)
		{
			Matrix motionViso = Matrix::inv(viso2_->getMotion());
			Transform motion(motionViso.val[0][0], motionViso.val[0][1], motionViso.val[0][2],motionViso.val[0][3],
					motionViso.val[1][0], motionViso.val[1][1], motionViso.val[1][2],motionViso.val[1][3],
					motionViso.val[2][0], motionViso.val[2][1], motionViso.val[2][2], motionViso.val[2][3]);
			Transform camera_motion;

			if(lost_ || keep_reference_frame_)
			{
			  camera_motion = reference_motion_.inverse() * motion;
			}
			else
			{
			  camera_motion = motion;
			}
			reference_motion_ = motion; // store last motion as reference

			t=camera_motion;

			//based on values set in viso2_ros
			covariance = cv::Mat::eye(6,6, CV_64FC1);
			covariance.at<double>(0,0) = 0.002;
			covariance.at<double>(1,1) = 0.002;
			covariance.at<double>(2,2) = 0.05;
			covariance.at<double>(3,3) = 0.09;
			covariance.at<double>(4,4) = 0.09;
			covariance.at<double>(5,5) = 0.09;

			lost_=false;
		}
		else
		{
			covariance = cv::Mat::eye(6,6, CV_64FC1)*9999.0;
			lost_ = true;
		}

		if(success)
		{
			// Proceed depending on the reference frame change method
			if(ref_frame_change_method_==1)
			{
				// calculate current feature flow
				double feature_flow = computeFeatureFlow(viso2_->getMatches());
				keep_reference_frame_ = (feature_flow < ref_frame_motion_threshold_);
			}
			else
			{
				keep_reference_frame_ = ref_frame_inlier_threshold_==0 || viso2_->getNumberOfInliers() > ref_frame_inlier_threshold_;
			}
		}
		else
		{
			keep_reference_frame_ = false;
		}
	}

	const Transform & localTransform = data.stereoCameraModels()[0].localTransform();
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
		info->type = (int)kTypeViso2;
		info->keyFrameAdded = !keep_reference_frame_;
		info->reg.matches = viso2_->getNumberOfMatches();
		info->reg.inliers = viso2_->getNumberOfInliers();
		if(covariance.cols == 6 && covariance.rows == 6 && covariance.type() == CV_64FC1)
		{
			info->reg.covariance = covariance;
		}

		if(this->isInfoDataFilled())
		{
			std::vector<Matcher::p_match> matches = viso2_->getMatches();
			info->refCorners.resize(matches.size());
			info->newCorners.resize(matches.size());
			info->cornerInliers.resize(matches.size());
			for (size_t i = 0; i < matches.size(); ++i)
			{
				info->refCorners[i].x = matches[i].u1p;
				info->refCorners[i].y = matches[i].v1p;
				info->newCorners[i].x = matches[i].u1c;
				info->newCorners[i].y = matches[i].v1c;
				info->cornerInliers[i] = i;
			}
		}
	}

	UINFO("Odom update time = %fs lost=%s", timer.elapsed(), lost_?"true":"false");

#else
	UERROR("RTAB-Map is not built with VISO2 support! Select another visual odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
