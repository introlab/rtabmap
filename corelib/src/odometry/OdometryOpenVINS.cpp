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

#include "rtabmap/core/odometry/OdometryOpenVINS.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_OPENVINS
#include "core/VioManager.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#endif

namespace rtabmap {

OdometryOpenVINS::OdometryOpenVINS(const ParametersMap & parameters) :
	Odometry(parameters)
#ifdef RTABMAP_OPENVINS
    ,
	initGravity_(false),
	previousPoseInv_(Transform::getIdentity())
#endif
{
#ifdef RTABMAP_OPENVINS
	ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel(ULogger::level()+1));
	int enum_index;
	std::string left_mask_path, right_mask_path;
	params_ = std::make_unique<ov_msckf::VioManagerOptions>();
	Parameters::parse(parameters, Parameters::kOdomOpenVINSUseStereo(), params_->use_stereo);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSUseKLT(), params_->use_klt);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSNumPts(), params_->num_pts);
	Parameters::parse(parameters, Parameters::kFASTThreshold(), params_->fast_threshold);
	Parameters::parse(parameters, Parameters::kVisGridCols(), params_->grid_x);
	Parameters::parse(parameters, Parameters::kVisGridRows(), params_->grid_y);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSMinPxDist(), params_->min_px_dist);
	Parameters::parse(parameters, Parameters::kVisCorNNDR(), params_->knn_ratio);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSFiTriangulate1d(), params_->featinit_options.triangulate_1d);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSFiRefineFeatures(), params_->featinit_options.refine_features);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSFiMaxRuns(), params_->featinit_options.max_runs);
	Parameters::parse(parameters, Parameters::kVisMinDepth(), params_->featinit_options.min_dist);
	Parameters::parse(parameters, Parameters::kVisMaxDepth(), params_->featinit_options.max_dist);
	if(params_->featinit_options.max_dist == 0)
		params_->featinit_options.max_dist = std::numeric_limits<double>::infinity();
	Parameters::parse(parameters, Parameters::kOdomOpenVINSFiMaxBaseline(), params_->featinit_options.max_baseline);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSFiMaxCondNumber(), params_->featinit_options.max_cond_number);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSUseFEJ(), params_->state_options.do_fej);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSIntegration(), enum_index);
	params_->state_options.integration_method = ov_msckf::StateOptions::IntegrationMethod(enum_index);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSCalibCamExtrinsics(), params_->state_options.do_calib_camera_pose);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSCalibCamIntrinsics(), params_->state_options.do_calib_camera_intrinsics);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSCalibCamTimeoffset(), params_->state_options.do_calib_camera_timeoffset);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSCalibIMUIntrinsics(), params_->state_options.do_calib_imu_intrinsics);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSCalibIMUGSensitivity(), params_->state_options.do_calib_imu_g_sensitivity);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSMaxClones(), params_->state_options.max_clone_size);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSMaxSLAM(), params_->state_options.max_slam_features);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSMaxSLAMInUpdate(), params_->state_options.max_slam_in_update);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSMaxMSCKFInUpdate(), params_->state_options.max_msckf_in_update);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSFeatRepMSCKF(), enum_index);
	params_->state_options.feat_rep_msckf = ov_type::LandmarkRepresentation::Representation(enum_index);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSFeatRepSLAM(), enum_index);
	params_->state_options.feat_rep_slam = ov_type::LandmarkRepresentation::Representation(enum_index);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSDtSLAMDelay(), params_->dt_slam_delay);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSGravityMag(), params_->gravity_mag);
	Parameters::parse(parameters, Parameters::kVisDepthAsMask(), params_->use_mask);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSLeftMaskPath(), left_mask_path);
	if(!left_mask_path.empty())
	{
		if(!UFile::exists(left_mask_path))
			UWARN("OpenVINS: invalid left mask path: %s", left_mask_path.c_str());
		else
			params_->masks.emplace(0, cv::imread(left_mask_path, cv::IMREAD_GRAYSCALE));
	}
	Parameters::parse(parameters, Parameters::kOdomOpenVINSRightMaskPath(), right_mask_path);
	if(!right_mask_path.empty())
	{
		if(!UFile::exists(right_mask_path))
			UWARN("OpenVINS: invalid right mask path: %s", right_mask_path.c_str());
		else
			params_->masks.emplace(1, cv::imread(right_mask_path, cv::IMREAD_GRAYSCALE));
	}
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitWindowTime(), params_->init_options.init_window_time);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitIMUThresh(), params_->init_options.init_imu_thresh);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitMaxDisparity(), params_->init_options.init_max_disparity);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitMaxFeatures(), params_->init_options.init_max_features);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynUse(), params_->init_options.init_dyn_use);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynMLEOptCalib(), params_->init_options.init_dyn_mle_opt_calib);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynMLEMaxIter(), params_->init_options.init_dyn_mle_max_iter);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynMLEMaxTime(), params_->init_options.init_dyn_mle_max_time);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynMLEMaxThreads(), params_->init_options.init_dyn_mle_max_threads);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynNumPose(), params_->init_options.init_dyn_num_pose);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynMinDeg(), params_->init_options.init_dyn_min_deg);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynInflationOri(), params_->init_options.init_dyn_inflation_orientation);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynInflationVel(), params_->init_options.init_dyn_inflation_velocity);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynInflationBg(), params_->init_options.init_dyn_inflation_bias_gyro);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynInflationBa(), params_->init_options.init_dyn_inflation_bias_accel);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSInitDynMinRecCond(), params_->init_options.init_dyn_min_rec_cond);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSTryZUPT(), params_->try_zupt);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSZUPTChi2Multiplier(), params_->zupt_options.chi2_multipler);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSZUPTMaxVelodicy(), params_->zupt_max_velocity);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSZUPTNoiseMultiplier(), params_->zupt_noise_multiplier);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSZUPTMaxDisparity(), params_->zupt_max_disparity);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSZUPTOnlyAtBeginning(), params_->zupt_only_at_beginning);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSAccelerometerNoiseDensity(), params_->imu_noises.sigma_a);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSAccelerometerRandomWalk(), params_->imu_noises.sigma_ab);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSGyroscopeNoiseDensity(), params_->imu_noises.sigma_w);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSGyroscopeRandomWalk(), params_->imu_noises.sigma_wb);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSUpMSCKFSigmaPx(), params_->msckf_options.sigma_pix);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSUpMSCKFChi2Multiplier(), params_->msckf_options.chi2_multipler);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSUpSLAMSigmaPx(), params_->slam_options.sigma_pix);
	Parameters::parse(parameters, Parameters::kOdomOpenVINSUpSLAMChi2Multiplier(), params_->slam_options.chi2_multipler);
	params_->vec_dw << 1, 0, 0, 1, 0, 1;
	params_->vec_da << 1, 0, 0, 1, 0, 1;
	params_->vec_tg << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	params_->q_ACCtoIMU << 0, 0, 0, 1;
	params_->q_GYROtoIMU << 0, 0, 0, 1;
	params_->use_aruco = false;
	params_->num_opencv_threads = -1;
	params_->histogram_method = ov_core::TrackBase::HistogramMethod::NONE;
	params_->init_options.sigma_a = params_->imu_noises.sigma_a;
	params_->init_options.sigma_ab = params_->imu_noises.sigma_ab;
	params_->init_options.sigma_w = params_->imu_noises.sigma_w;
	params_->init_options.sigma_wb = params_->imu_noises.sigma_wb;
	params_->init_options.sigma_pix = params_->slam_options.sigma_pix;
	params_->init_options.gravity_mag = params_->gravity_mag;
#endif
}

void OdometryOpenVINS::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_OPENVINS
	if(!initGravity_)
	{
		vioManager_.reset();
		previousPoseInv_.setIdentity();
		imuLocalTransformInv_.setNull();
	}
	initGravity_ = false;
#endif
}

// return not null transform if odometry is correctly computed
Transform OdometryOpenVINS::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	Transform t;
#ifdef RTABMAP_OPENVINS

	if(!vioManager_)
	{
		if(!data.imu().empty())
		{
			imuLocalTransformInv_ = data.imu().localTransform().inverse();
			Phi_.setZero();
			Phi_.block(0,0,3,3) = data.imu().localTransform().toEigen4d().block(0,0,3,3);
			Phi_.block(3,3,3,3) = data.imu().localTransform().toEigen4d().block(0,0,3,3);
		}

		if(!data.imageRaw().empty() && !imuLocalTransformInv_.isNull())
		{
			Transform T_imu_left;
			Eigen::VectorXd left_calib(8), right_calib(8);
			if(!data.rightRaw().empty())
			{
				params_->state_options.num_cameras = params_->init_options.num_cameras = 2;
				T_imu_left = imuLocalTransformInv_ * data.stereoCameraModels()[0].localTransform();

				bool is_fisheye = data.stereoCameraModels()[0].left().isFisheye() && !this->imagesAlreadyRectified();
				if(is_fisheye)
				{
					params_->camera_intrinsics.emplace(0, std::make_shared<ov_core::CamEqui>(
						data.stereoCameraModels()[0].left().imageWidth(), data.stereoCameraModels()[0].left().imageHeight()));
					params_->camera_intrinsics.emplace(1, std::make_shared<ov_core::CamEqui>(
						data.stereoCameraModels()[0].right().imageWidth(), data.stereoCameraModels()[0].right().imageHeight()));
				}
				else
				{
					params_->camera_intrinsics.emplace(0, std::make_shared<ov_core::CamRadtan>(
						data.stereoCameraModels()[0].left().imageWidth(), data.stereoCameraModels()[0].left().imageHeight()));
					params_->camera_intrinsics.emplace(1, std::make_shared<ov_core::CamRadtan>(
						data.stereoCameraModels()[0].right().imageWidth(), data.stereoCameraModels()[0].right().imageHeight()));
				}

				if(this->imagesAlreadyRectified() || data.stereoCameraModels()[0].left().D_raw().empty())
				{
					left_calib << data.stereoCameraModels()[0].left().fx(),
						data.stereoCameraModels()[0].left().fy(),
					 	data.stereoCameraModels()[0].left().cx(),
					 	data.stereoCameraModels()[0].left().cy(), 0, 0, 0, 0;
					right_calib << data.stereoCameraModels()[0].right().fx(),
						data.stereoCameraModels()[0].right().fy(),
					 	data.stereoCameraModels()[0].right().cx(),
					 	data.stereoCameraModels()[0].right().cy(), 0, 0, 0, 0;
				}
				else
				{
					UASSERT(data.stereoCameraModels()[0].left().D_raw().cols == data.stereoCameraModels()[0].right().D_raw().cols);
					UASSERT(data.stereoCameraModels()[0].left().D_raw().cols >= 4);
					UASSERT(data.stereoCameraModels()[0].right().D_raw().cols >= 4);
					left_calib << data.stereoCameraModels()[0].left().K_raw().at<double>(0,0),
					 	data.stereoCameraModels()[0].left().K_raw().at<double>(1,1),
					 	data.stereoCameraModels()[0].left().K_raw().at<double>(0,2),
					 	data.stereoCameraModels()[0].left().K_raw().at<double>(1,2),
					 	data.stereoCameraModels()[0].left().D_raw().at<double>(0,0),
					 	data.stereoCameraModels()[0].left().D_raw().at<double>(0,1),
					 	data.stereoCameraModels()[0].left().D_raw().at<double>(0,is_fisheye?4:2),
					 	data.stereoCameraModels()[0].left().D_raw().at<double>(0,is_fisheye?5:3);
					right_calib << data.stereoCameraModels()[0].right().K_raw().at<double>(0,0),
					 	data.stereoCameraModels()[0].right().K_raw().at<double>(1,1),
					 	data.stereoCameraModels()[0].right().K_raw().at<double>(0,2),
					 	data.stereoCameraModels()[0].right().K_raw().at<double>(1,2),
					 	data.stereoCameraModels()[0].right().D_raw().at<double>(0,0),
					 	data.stereoCameraModels()[0].right().D_raw().at<double>(0,1),
					 	data.stereoCameraModels()[0].right().D_raw().at<double>(0,is_fisheye?4:2),
					 	data.stereoCameraModels()[0].right().D_raw().at<double>(0,is_fisheye?5:3);
				}
			}
			else
			{
				params_->state_options.num_cameras = params_->init_options.num_cameras = 1;
				T_imu_left = imuLocalTransformInv_ * data.cameraModels()[0].localTransform();

				bool is_fisheye = data.cameraModels()[0].isFisheye() && !this->imagesAlreadyRectified();
				if(is_fisheye)
				{
					params_->camera_intrinsics.emplace(0, std::make_shared<ov_core::CamEqui>(
						data.cameraModels()[0].imageWidth(), data.cameraModels()[0].imageHeight()));
				}
				else
				{
					params_->camera_intrinsics.emplace(0, std::make_shared<ov_core::CamRadtan>(
						data.cameraModels()[0].imageWidth(), data.cameraModels()[0].imageHeight()));
				}

				if(this->imagesAlreadyRectified() || data.cameraModels()[0].D_raw().empty())
				{
					left_calib << data.cameraModels()[0].fx(),
						data.cameraModels()[0].fy(),
					 	data.cameraModels()[0].cx(),
					 	data.cameraModels()[0].cy(), 0, 0, 0, 0;
				}
				else
				{
					UASSERT(data.cameraModels()[0].D_raw().cols >= 4);
					left_calib << data.cameraModels()[0].K_raw().at<double>(0,0),
					 	data.cameraModels()[0].K_raw().at<double>(1,1),
					 	data.cameraModels()[0].K_raw().at<double>(0,2),
					 	data.cameraModels()[0].K_raw().at<double>(1,2),
					 	data.cameraModels()[0].D_raw().at<double>(0,0),
					 	data.cameraModels()[0].D_raw().at<double>(0,1),
					 	data.cameraModels()[0].D_raw().at<double>(0,is_fisheye?4:2),
					 	data.cameraModels()[0].D_raw().at<double>(0,is_fisheye?5:3);
				}
			}

			Eigen::Matrix4d T_LtoI = T_imu_left.toEigen4d();
			Eigen::Matrix<double,7,1> left_eigen;
			left_eigen.block(0,0,4,1) = ov_core::rot_2_quat(T_LtoI.block(0,0,3,3).transpose());
			left_eigen.block(4,0,3,1) = -T_LtoI.block(0,0,3,3).transpose()*T_LtoI.block(0,3,3,1);
			params_->camera_intrinsics.at(0)->set_value(left_calib);
			params_->camera_extrinsics.emplace(0, left_eigen);
			if(!data.rightRaw().empty())
			{
				Transform T_left_right;
				if(this->imagesAlreadyRectified() || data.stereoCameraModels()[0].stereoTransform().isNull())
				{
					T_left_right = Transform(
						1, 0, 0, data.stereoCameraModels()[0].baseline(),
						0, 1, 0, 0,
						0, 0, 1, 0);
				}
				else
				{
					T_left_right = data.stereoCameraModels()[0].stereoTransform().inverse();
				}
				UASSERT(!T_left_right.isNull());
				Transform T_imu_right = T_imu_left * T_left_right;
				Eigen::Matrix4d T_RtoI = T_imu_right.toEigen4d();
				Eigen::Matrix<double,7,1> right_eigen;
				right_eigen.block(0,0,4,1) = ov_core::rot_2_quat(T_RtoI.block(0,0,3,3).transpose());
				right_eigen.block(4,0,3,1) = -T_RtoI.block(0,0,3,3).transpose()*T_RtoI.block(0,3,3,1);
				params_->camera_intrinsics.at(1)->set_value(right_calib);
				params_->camera_extrinsics.emplace(1, right_eigen);
			}
			params_->init_options.camera_intrinsics = params_->camera_intrinsics;
			params_->init_options.camera_extrinsics = params_->camera_extrinsics;
			vioManager_ = std::make_unique<ov_msckf::VioManager>(*params_);
		}
	}
	else
	{
		if(!data.imu().empty())
		{
			ov_core::ImuData message;
			message.timestamp = data.stamp();
			message.wm << data.imu().angularVelocity().val[0], data.imu().angularVelocity().val[1], data.imu().angularVelocity().val[2];
			message.am << data.imu().linearAcceleration().val[0], data.imu().linearAcceleration().val[1], data.imu().linearAcceleration().val[2];
			vioManager_->feed_measurement_imu(message);
		}

		if(!data.imageRaw().empty())
		{
			bool covFilled = false;
			Eigen::Matrix<double, 13, 1> state_plus = Eigen::Matrix<double, 13, 1>::Zero();
			Eigen::Matrix<double, 12, 12> cov_plus = Eigen::Matrix<double, 12, 12>::Zero();
			if(vioManager_->initialized())
				covFilled = vioManager_->get_propagator()->fast_state_propagate(vioManager_->get_state(), data.stamp(), state_plus, cov_plus);

			cv::Mat image;
			if(data.imageRaw().type() == CV_8UC3)
				cv::cvtColor(data.imageRaw(), image, CV_BGR2GRAY);
			else if(data.imageRaw().type() == CV_8UC1)
				image = data.imageRaw().clone();
			else
				UFATAL("Not supported color type!");
			ov_core::CameraData message;
			message.timestamp = data.stamp();
			message.sensor_ids.emplace_back(0);
			message.images.emplace_back(image);
			if(params_->masks.find(0) != params_->masks.end())
			{
				message.masks.emplace_back(params_->masks[0]);
			}
			else if(!data.depthRaw().empty() && params_->use_mask)
			{
				cv::Mat mask;
				if(data.depthRaw().type() == CV_32FC1)
					cv::inRange(data.depthRaw(), params_->featinit_options.min_dist,
						std::isinf(params_->featinit_options.max_dist)?std::numeric_limits<float>::max():params_->featinit_options.max_dist, mask);
				else if(data.depthRaw().type() == CV_16UC1)
					cv::inRange(data.depthRaw(), params_->featinit_options.min_dist*1000,
						std::isinf(params_->featinit_options.max_dist)?std::numeric_limits<uint16_t>::max():params_->featinit_options.max_dist*1000, mask);
				message.masks.emplace_back(255-mask);
			}
			else
			{
				message.masks.emplace_back(cv::Mat::zeros(image.size(), CV_8UC1));
			}
			if(!data.rightRaw().empty())
			{
				if(data.rightRaw().type() == CV_8UC3)
					cv::cvtColor(data.rightRaw(), image, CV_BGR2GRAY);
				else if(data.rightRaw().type() == CV_8UC1)
					image = data.rightRaw().clone();
				else
					UFATAL("Not supported color type!");
				message.sensor_ids.emplace_back(1);
				message.images.emplace_back(image);
				if(params_->masks.find(1) != params_->masks.end())
					message.masks.emplace_back(params_->masks[1]);
				else
					message.masks.emplace_back(cv::Mat::zeros(image.size(), CV_8UC1));
			}
			vioManager_->feed_measurement_camera(message);

			std::shared_ptr<ov_msckf::State> state = vioManager_->get_state();
			Transform p((float)state->_imu->pos()(0),
						(float)state->_imu->pos()(1),
						(float)state->_imu->pos()(2),
						(float)state->_imu->quat()(0),
						(float)state->_imu->quat()(1),
						(float)state->_imu->quat()(2),
						(float)state->_imu->quat()(3));
			if(!p.isNull() && !p.isIdentity())
			{
				p = p * imuLocalTransformInv_;

				if(this->getPose().rotation().isIdentity())
				{
					initGravity_ = true;
					this->reset(this->getPose() * p.rotation());
				}

				if(previousPoseInv_.isIdentity())
					previousPoseInv_ = p.inverse();

				t = previousPoseInv_ * p;

				if(info)
				{
					double timestamp;
					std::unordered_map<size_t, Eigen::Vector3d> feat_posinG, feat_tracks_uvd;
					vioManager_->get_active_tracks(timestamp, feat_posinG, feat_tracks_uvd);
					auto features_SLAM = vioManager_->get_features_SLAM();
					auto good_features_MSCKF = vioManager_->get_good_features_MSCKF();

					info->type = this->getType();
					info->localMapSize = feat_posinG.size();
					info->features = features_SLAM.size() + good_features_MSCKF.size();
					info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1);
					if(covFilled)
					{
						Eigen::Matrix<double, 6, 6> covariance = Phi_ * cov_plus.block(6,6,6,6) * Phi_.transpose();
						cv::eigen2cv(covariance, info->reg.covariance);
					}

					if(this->isInfoDataFilled())
					{
						Transform fixT = this->getPose() * previousPoseInv_;
						Transform camT;
						if(!data.rightRaw().empty())
							camT = data.stereoCameraModels()[0].localTransform().inverse() * t.inverse() * this->getPose().inverse() * fixT;
						else
							camT = data.cameraModels()[0].localTransform().inverse() * t.inverse() * this->getPose().inverse() * fixT;
						
						for(auto &feature : feat_posinG)
						{
							cv::Point3f pt3d(feature.second[0], feature.second[1], feature.second[2]);
							pt3d = util3d::transformPoint(pt3d, fixT);
							info->localMap.emplace(feature.first, pt3d);
						}

						if(this->imagesAlreadyRectified())
						{
							for(auto &feature : features_SLAM)
							{
								cv::Point3f pt3d(feature[0], feature[1], feature[2]);
								pt3d = util3d::transformPoint(pt3d, camT);
								cv::Point2f pt;
								if(!data.rightRaw().empty())
									data.stereoCameraModels()[0].left().reproject(pt3d.x, pt3d.y, pt3d.z, pt.x, pt.y);
								else
									data.cameraModels()[0].reproject(pt3d.x, pt3d.y, pt3d.z, pt.x, pt.y);
								info->reg.inliersIDs.emplace_back(info->newCorners.size());
								info->newCorners.emplace_back(pt);
							}

							for(auto &feature : good_features_MSCKF)
							{
								cv::Point3f pt3d(feature[0], feature[1], feature[2]);
								pt3d = util3d::transformPoint(pt3d, camT);
								cv::Point2f pt;
								if(!data.rightRaw().empty())
									data.stereoCameraModels()[0].left().reproject(pt3d.x, pt3d.y, pt3d.z, pt.x, pt.y);
								else
									data.cameraModels()[0].reproject(pt3d.x, pt3d.y, pt3d.z, pt.x, pt.y);
								info->reg.matchesIDs.emplace_back(info->newCorners.size());
								info->newCorners.emplace_back(pt);
							}
						}
					}
				}

				previousPoseInv_ = p.inverse();
			}
		}
	}

#else
	UERROR("RTAB-Map is not built with OpenVINS support! Select another visual odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
