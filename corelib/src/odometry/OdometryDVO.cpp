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

#include "rtabmap/core/odometry/OdometryDVO.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_DVO
#include <dvo/dense_tracking.h>
#include <dvo/core/surface_pyramid.h>
#include <dvo/core/rgbd_image.h>
#endif

namespace rtabmap {

OdometryDVO::OdometryDVO(const ParametersMap & parameters) :
	Odometry(parameters),
#ifdef RTABMAP_DVO
	dvo_(0),
	reference_(0),
	camera_(0),
	lost_(false),
#endif
	motionFromKeyFrame_(Transform::getIdentity())
{
}

OdometryDVO::~OdometryDVO()
{
#ifdef RTABMAP_DVO
	delete dvo_;
	delete reference_;
	delete camera_;
#endif
}

void OdometryDVO::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_DVO
	if(dvo_)
	{
		delete dvo_;
		dvo_ = 0;
	}
	if(reference_)
	{
		delete reference_;
		reference_ = 0;
	}
	if(camera_)
	{
		delete camera_;
		camera_ = 0;
	}
	lost_ = false;
	motionFromKeyFrame_.setIdentity();
	previousLocalTransform_.setNull();
#endif
}

// return not null transform if odometry is correctly computed
Transform OdometryDVO::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	Transform t;

#ifdef RTABMAP_DVO
	UTimer timer;

	if(data.imageRaw().empty() ||
		data.imageRaw().rows != data.depthOrRightRaw().rows ||
		data.imageRaw().cols != data.depthOrRightRaw().cols)
	{
		UERROR("Not supported input!");
		return t;
	}

	if(!(data.cameraModels().size() == 1 && data.cameraModels()[0].isValidForReprojection()))
	{
		UERROR("Invalid camera model! Only single RGB-D camera supported by DVO. Try another odometry approach.");
		return t;
	}

	if(dvo_ == 0)
	{
		dvo::DenseTracker::Config cfg = dvo::DenseTracker::getDefaultConfig();

		dvo_ = new dvo::DenseTracker(cfg);
	}

	cv::Mat grey, grey_s16, depth_inpainted, depth_mask, depth_mono, depth_float;
	if(data.imageRaw().type() != CV_32FC1)
	{
		if(data.imageRaw().type() == CV_8UC3)
		{
			cv::cvtColor(data.imageRaw(), grey, CV_BGR2GRAY);
		}
		else
		{
			grey = data.imageRaw();
		}

		grey.convertTo(grey_s16, CV_32F);
	}
	else
	{
		grey_s16 = data.imageRaw();
	}

	// make sure all zeros are NAN
	if(data.depthRaw().type() == CV_32FC1)
	{
		depth_float = data.depthRaw();
		for(int i=0; i<depth_float.rows; ++i)
		{
			for(int j=0; j<depth_float.cols; ++j)
			{
				float & d = depth_float.at<float>(i,j);
				if(d == 0.0f)
				{
					d = NAN;
				}
			}
		}
	}
	else if(data.depthRaw().type() == CV_16UC1)
	{
		depth_float = cv::Mat(data.depthRaw().size(), CV_32FC1);
		for(int i=0; i<data.depthRaw().rows; ++i)
		{
			for(int j=0; j<data.depthRaw().cols; ++j)
			{
				float d = float(data.depthRaw().at<unsigned short>(i,j))/1000.0f;
				depth_float.at<float>(i, j) = d==0.0f?NAN:d;
			}
		}
	}
	else
	{
		UFATAL("Unknown depth format!");
	}

	if(camera_ == 0)
	{
		dvo::core::IntrinsicMatrix intrinsics = dvo::core::IntrinsicMatrix::create(
						data.cameraModels()[0].fx(),
						data.cameraModels()[0].fy(),
						data.cameraModels()[0].cx(),
						data.cameraModels()[0].cy());
		camera_ = new dvo::core::RgbdCameraPyramid(
				data.cameraModels()[0].imageWidth(),
				data.cameraModels()[0].imageHeight(),
				intrinsics);
	}

	dvo::core::RgbdImagePyramid * current = new dvo::core::RgbdImagePyramid(*camera_, grey_s16, depth_float);

	const Transform & localTransform = data.cameraModels()[0].localTransform();
	cv::Mat covariance;
	if(reference_ == 0)
	{
		reference_ = current;
		if(!lost_)
		{
			t.setIdentity();
		}
		covariance = cv::Mat::eye(6,6,CV_64FC1) * 9999.0;
	}
	else
	{
		dvo::DenseTracker::Result result;
		dvo_->match(*reference_, *current, result);
		t = Transform::fromEigen3d(result.Transformation);

		if(result.Information(0,0) > 0.0 && result.Information(0,0) != 1.0)
		{
			lost_ = false;
			cv::Mat information = cv::Mat::eye(6,6, CV_64FC1);
			memcpy(information.data, result.Information.data(), 36*sizeof(double));
			//copy only diagonal to avoid g2o/gtsam errors on graph optimization
			covariance  = cv::Mat::eye(6,6,CV_64FC1);
			covariance = information.inv().mul(covariance);
			//covariance *= 100.0; // to be in the same scale than loop closure detection

			Transform currentMotion = t;
			t = motionFromKeyFrame_.inverse() * t;

			// TODO make parameters?
			if(currentMotion.getNorm() > 0.01 || currentMotion.getAngle() > 0.01)
			{
				if(info)
				{
					info->keyFrameAdded = true;
				}
				// new keyframe
				delete reference_;
				reference_ = current;
				motionFromKeyFrame_.setIdentity();
			}
			else
			{
				delete current;
				motionFromKeyFrame_ = currentMotion;
			}
		}
		else
		{
			lost_ = true;
			delete reference_;
			delete current;
			reference_ = 0; // this will make restart from the next frame
			motionFromKeyFrame_.setIdentity();
			t.setNull();
			previousLocalTransform_.setNull();
			covariance = cv::Mat::eye(6,6,CV_64FC1) * 9999.0;
			UWARN("dvo failed to estimate motion, tracking will be reinitialized on next frame.");
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
	}

	if(info)
	{
		info->type = (int)kTypeDVO;
		info->reg.covariance = covariance;
	}

	UINFO("Odom update time = %fs", timer.elapsed());

#else
	UERROR("RTAB-Map is not built with DVO support! Select another visual odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
