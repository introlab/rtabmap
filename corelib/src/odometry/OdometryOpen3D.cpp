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

#include "rtabmap/core/odometry/OdometryOpen3D.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"

#ifdef RTABMAP_OPEN3D
#include <open3d/pipelines/odometry/Odometry.h>
#include <open3d/geometry/RGBDImage.h>
#include <open3d/t/pipelines/odometry/RGBDOdometry.h>
#endif

namespace rtabmap {

/**
 * https://github.com/laboshinl/loam_velodyne/pull/66
 */

OdometryOpen3D::OdometryOpen3D(const ParametersMap & parameters) :
	Odometry(parameters)
#ifdef RTABMAP_OPEN3D
	,method_(Parameters::defaultOdomOpen3DMethod()),
	maxDepth_(Parameters::defaultOdomOpen3DMaxDepth()),
	keyFrameThr_(Parameters::defaultOdomKeyFrameThr())
#endif
{
#ifdef RTABMAP_OPEN3D
	Parameters::parse(parameters, Parameters::kOdomOpen3DMethod(), method_);
	UASSERT(method_>=0 && method_<=2);
	Parameters::parse(parameters, Parameters::kOdomOpen3DMaxDepth(), maxDepth_);
	Parameters::parse(parameters, Parameters::kOdomKeyFrameThr(), keyFrameThr_);
#endif
}

OdometryOpen3D::~OdometryOpen3D()
{
}

void OdometryOpen3D::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_OPEN3D
	keyFrame_ = SensorData();
	lastKeyFramePose_.setNull();
#endif
}

#ifdef RTABMAP_OPEN3D
open3d::geometry::Image toOpen3D(const cv::Mat & image)
{
	if(image.type() == CV_16UC1)
	{
		// convert to float
		return toOpen3D(util2d::cvtDepthToFloat(image));
	}
	open3d::geometry::Image output;
	output.width_ = image.cols;
	output.height_ = image.rows;
	output.num_of_channels_ = image.channels();
	output.bytes_per_channel_ = image.elemSize()/image.channels();
	output.data_.resize(image.total()*image.elemSize());
	memcpy(output.data_.data(), image.data, output.data_.size());
	return output;
}

open3d::geometry::RGBDImage toOpen3D(const SensorData & data)
{
	return open3d::geometry::RGBDImage(
			toOpen3D(data.imageRaw()),
			toOpen3D(data.depthRaw()));
}

open3d::camera::PinholeCameraIntrinsic toOpen3D(const CameraModel & model)
{
	return open3d::camera::PinholeCameraIntrinsic(
			model.imageWidth(),
			model.imageHeight(),
			model.fx(),
			model.fy(),
			model.cx(),
			model.cy());
}
//Tensor versions
open3d::t::geometry::Image toOpen3Dt(const cv::Mat & image)
{
	if(image.type() == CV_16UC1)
	{
		// convert to float
		return toOpen3Dt(util2d::cvtDepthToFloat(image));
	}

	if(image.type()==CV_32FC1)
	{
		return open3d::core::Tensor(
			(const float_t*)image.data,
			{image.rows, image.cols, image.channels()},
			open3d::core::Float32);
	}
	else
	{
		return open3d::core::Tensor(
			static_cast<const uint8_t*>(image.data),
			{image.rows, image.cols, image.channels()},
			open3d::core::UInt8);
	}
}

open3d::t::geometry::RGBDImage toOpen3Dt(const SensorData & data)
{
	return open3d::t::geometry::RGBDImage(
			toOpen3Dt(data.imageRaw()),
			toOpen3Dt(data.depthRaw()));
}

open3d::core::Tensor toOpen3Dt(const CameraModel & model)
{
	return open3d::core::Tensor::Init<double>(
			{{model.fx(), 0, model.cx()},
			 {0, model.fy(), model.cy()},
			 {0, 0, 1}});
}

open3d::core::Tensor toOpen3Dt(const Transform & t)
{
	return open3d::core::Tensor::Init<double>(
			{{t.r11(), t.r12(), t.r13(), t.x()},
			 {t.r21(), t.r22(), t.r23(), t.y()},
			 {t.r31(), t.r32(), t.r33(), t.z()},
			 {0,0,0,1}});
}
#endif

// return not null transform if odometry is correctly computed
Transform OdometryOpen3D::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	Transform t;
#ifdef RTABMAP_OPEN3D
	UTimer timer;

	if(data.imageRaw().empty() || data.depthRaw().empty() || data.cameraModels().size()!=1)
	{
		UERROR("Open3D works only with single RGB-D data. Aborting odometry update...");
		return t;
	}

	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1)*9999;

	bool updateKeyFrame = false;
	if(lastKeyFramePose_.isNull())
	{
		lastKeyFramePose_ = this->getPose(); // reset to current pose
	}
	Transform motionSinceLastKeyFrame = lastKeyFramePose_.inverse()*this->getPose();

	if(keyFrame_.isValid())
	{
		/*bool tensor = true;
		if(!tensor)
		{
			// Approach in open3d/pipelines/odometry
			open3d::geometry::RGBDImage source = toOpen3D(data);
			open3d::geometry::RGBDImage target = toOpen3D(keyFrame_);
			open3d::camera::PinholeCameraIntrinsic intrinsics = toOpen3D(data.cameraModels()[0]);
			UDEBUG("Data conversion to Open3D format: %fs", timer.ticks());
			open3d::pipelines::odometry::OdometryOption option;
			std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> ret = open3d::pipelines::odometry::ComputeRGBDOdometry(
					source,
					target,
					intrinsics,
					Eigen::Matrix4d::Identity(),
					open3d::pipelines::odometry::RGBDOdometryJacobianFromHybridTerm(),
					option);
			UDEBUG("Compute Open3D odometry: %fs", timer.ticks());
			if(std::get<0>(ret))
			{
				t = Transform::fromEigen4d(std::get<1>(ret));
				// from camera frame to base frame
				t = data.cameraModels()[0].localTransform() * t * data.cameraModels()[0].localTransform().inverse();
				covariance = cv::Mat::eye(6,6,CV_64FC1)/100;
			}
			else
			{
				UWARN("Open3D odometry update failed!");
			}
		}
		else*/
		{
			// Approach in open3d/t/pipelines/odometry
			open3d::t::geometry::RGBDImage source = toOpen3Dt(data);
			open3d::t::geometry::RGBDImage target = toOpen3Dt(keyFrame_);
			open3d::core::Tensor intrinsics = toOpen3Dt(data.cameraModels()[0]);
			Transform baseToCamera = data.cameraModels()[0].localTransform();
			open3d::core::Tensor odomInit;
			if(guess.isNull())
			{
				odomInit = open3d::core::Tensor::Eye(4, open3d::core::Float64, open3d::core::Device("CPU:0"));
			}
			else
			{
				odomInit = toOpen3Dt(baseToCamera.inverse() * (motionSinceLastKeyFrame*guess) * baseToCamera);
			}
			UDEBUG("Data conversion to Open3D format: %fs", timer.ticks());
			open3d::t::pipelines::odometry::OdometryResult ret = open3d::t::pipelines::odometry::RGBDOdometryMultiScale(
					source,
					target,
					intrinsics,
					odomInit,
					1.0f,
					maxDepth_,
				    {10, 5, 3},
					(open3d::t::pipelines::odometry::Method)method_,
				   open3d::t::pipelines::odometry::OdometryLossParams());
			UDEBUG("Compute Open3D odometry: %fs", timer.ticks());
			if(ret.fitness_!=0)
			{
				const double * ptr = (const double *)ret.transformation_.GetDataPtr();
				t = Transform(
						ptr[0], ptr[1], ptr[2], ptr[3],
						ptr[4], ptr[5], ptr[6], ptr[7],
						ptr[8], ptr[9], ptr[10],ptr[11]);
				// from camera frame to base frame
				t = baseToCamera * t * baseToCamera.inverse();

				t = motionSinceLastKeyFrame.inverse() * t;

				//based on values set in viso2_ros
				covariance = cv::Mat::eye(6,6, CV_64FC1);
				covariance.at<double>(0,0) = 0.002;
				covariance.at<double>(1,1) = 0.002;
				covariance.at<double>(2,2) = 0.05;
				covariance.at<double>(3,3) = 0.09;
				covariance.at<double>(4,4) = 0.09;
				covariance.at<double>(5,5) = 0.09;

				if(info)
				{
					info->reg.icpRMS = ret.inlier_rmse_;
					info->reg.icpInliersRatio = ret.fitness_;
				}

				if(ret.fitness_ < keyFrameThr_)
				{
					updateKeyFrame = true;
				}
			}
			else
			{
				UWARN("Open3D odometry update failed!");
			}
		}
	}
	else
	{
		t.setIdentity();
		updateKeyFrame = true;
	}

	if(updateKeyFrame)
	{
		keyFrame_ = data;
		lastKeyFramePose_.setNull();
	}

	if(info)
	{
		info->reg.covariance = covariance;
		info->keyFrameAdded = updateKeyFrame;
	}

#else
	UERROR("RTAB-Map is not built with Open3D support! Select another odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
