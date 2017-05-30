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

#include "rtabmap/core/OdometryFovis.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/Version.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"

#ifdef RTABMAP_FOVIS
#include <fovis.hpp>
#endif

namespace rtabmap {

OdometryFovis::OdometryFovis(const ParametersMap & parameters) :
	Odometry(parameters),
	fovis_(0),
	rect_(0),
	stereoCalib_(0),
	depthImage_(0),
	stereoDepth_(0)
{
}

OdometryFovis::~OdometryFovis()
{
#ifdef RTABMAP_FOVIS
	if(fovis_)
	{
		delete fovis_;
	}
	if(rect_)
	{
		delete rect_;
	}
	if(stereoCalib_)
	{
		delete stereoCalib_;
	}
	if(depthImage_)
	{
		delete depthImage_;
	}
	if(stereoDepth_)
	{
		delete stereoDepth_;
	}
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
#endif
}

// return not null transform if odometry is correctly computed
Transform OdometryFovis::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
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

	if(!((data.cameraModels().size() == 1 &&
			data.cameraModels()[0].isValidForReprojection()) ||
		(data.stereoCameraModel().isValidForProjection() &&
			data.stereoCameraModel().left().isValidForReprojection() &&
			data.stereoCameraModel().right().isValidForReprojection())))
	{
		UERROR("Invalid camera model!");
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

	fovis::VisualOdometryOptions options = fovis::VisualOdometry::getDefaultOptions();
	fovis::DepthSource * depthSource = 0;
	cv::Mat depth;
	cv::Mat right;
	Transform localTransform = Transform::getIdentity();
	if(data.cameraModels().size() == 1) //depth
	{
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
	else // stereo
	{
		// initialize left camera parameters
		fovis::CameraIntrinsicsParameters left_parameters;
		left_parameters.width = data.stereoCameraModel().left().imageWidth();
		left_parameters.height = data.stereoCameraModel().left().imageHeight();
		left_parameters.fx = data.stereoCameraModel().left().fx();
		left_parameters.fy = data.stereoCameraModel().left().fy();
		left_parameters.cx = data.stereoCameraModel().left().cx()==0.0?double(left_parameters.width) / 2.0:data.stereoCameraModel().left().cx();
		left_parameters.cy = data.stereoCameraModel().left().cy()==0.0?double(left_parameters.height) / 2.0:data.stereoCameraModel().left().cy();
		localTransform = data.stereoCameraModel().localTransform();

		if(rect_ == 0)
		{
			UINFO("Init stereo fovis: %dx%d fx=%f fy=%f cx=%f cy=%f", left_parameters.width, left_parameters.height, left_parameters.fx, left_parameters.fy, left_parameters.cx, left_parameters.cy);
			rect_ = new fovis::Rectification(left_parameters);
		}

		if(stereoCalib_ == 0)
		{
			// initialize right camera parameters
			fovis::CameraIntrinsicsParameters right_parameters;
			right_parameters.width = data.stereoCameraModel().right().imageWidth();
			right_parameters.height = data.stereoCameraModel().right().imageHeight();
			right_parameters.fx = data.stereoCameraModel().right().fx();
			right_parameters.fy = data.stereoCameraModel().right().fy();
			right_parameters.cx = data.stereoCameraModel().right().cx()==0.0?double(right_parameters.width) / 2.0:data.stereoCameraModel().right().cx();
			right_parameters.cy = data.stereoCameraModel().right().cy()==0.0?double(right_parameters.height) / 2.0:data.stereoCameraModel().right().cy();

			// as we use rectified images, rotation is identity
			// and translation is baseline only
			fovis::StereoCalibrationParameters stereo_parameters;
			stereo_parameters.left_parameters = left_parameters;
			stereo_parameters.right_parameters = right_parameters;
			stereo_parameters.right_to_left_rotation[0] = 1.0;
			stereo_parameters.right_to_left_rotation[1] = 0.0;
			stereo_parameters.right_to_left_rotation[2] = 0.0;
			stereo_parameters.right_to_left_rotation[3] = 0.0;
			stereo_parameters.right_to_left_translation[0] = -data.stereoCameraModel().baseline();
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


	fovis_->processFrame(gray.data, depthSource);

	// get the motion estimate for this frame to the previous frame.
	t = Transform::fromEigen3d(fovis_->getMotionEstimate());

	fovis::MotionEstimateStatusCode statusCode = fovis::SUCCESS;
	if(fovis_->getMotionEstimator())
	{
		statusCode = fovis_->getMotionEstimator()->getMotionEstimateStatus();
	}

	if(statusCode > fovis::SUCCESS)
	{
		UWARN("Fovis error status: %s", fovis::MotionEstimateStatusCodeStrings[statusCode]);
		t.setNull();
	}

	if(!t.isNull() && !t.isIdentity() && !localTransform.isIdentity() && !localTransform.isNull())
	{
		// from camera frame to base frame
		t = localTransform * t * localTransform.inverse();
	}

	if(info)
	{
		info->type = (int)kTypeFovis;
		info->keyFrameAdded = fovis_->getChangeReferenceFrames();
		info->features = fovis_->getTargetFrame()->getNumDetectedKeypoints();
		info->matches = fovis_->getMotionEstimator()->getNumMatches();
		info->inliers = fovis_->getMotionEstimator()->getNumInliers();
		const Eigen::MatrixXd& cov = fovis_->getMotionEstimator()->getMotionEstimateCov();
		if(cov.cols() == 6 && cov.rows() == 6 && cov(0,0) > 0.0)
		{
			info->covariance = cv::Mat::eye(6,6, CV_64FC1);
			memcpy(info->covariance.data, cov.data(), 36*sizeof(double));
			info->covariance *= 100.0; // to be in the same scale than loop closure detection
		}
	}

	UINFO("Odom update time = %fs status=%s", timer.elapsed(), fovis::MotionEstimateStatusCodeStrings[statusCode]);

#else
	UERROR("RTAB-Map is not built with FOVIS support! Select another visual odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
