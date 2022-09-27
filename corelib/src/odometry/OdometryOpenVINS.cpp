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
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/UDirectory.h"
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_OPENVINS
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "core/RosVisualizer.h"
#include "utils/dataset_reader.h"
#include "utils/parse_ros.h"
#include "utils/sensor_data.h"
#include "state/State.h"
#include "types/Type.h"
#endif

namespace rtabmap {

OdometryOpenVINS::OdometryOpenVINS(const ParametersMap & parameters) :
	Odometry(parameters)
#ifdef RTABMAP_OPENVINS
    ,
	vioManager_(0),
	initGravity_(false),
	previousPose_(Transform::getIdentity())
#endif
{
}

OdometryOpenVINS::~OdometryOpenVINS()
{
#ifdef RTABMAP_OPENVINS
	delete vioManager_;
#endif
}

void OdometryOpenVINS::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_OPENVINS
	if(!initGravity_)
	{
		delete vioManager_;
		vioManager_ = 0;
		previousPose_.setIdentity();
		previousLocalTransform_.setNull();
		imuBuffer_.clear();
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
	UTimer timer;

	// Buffer imus;
	if(!data.imu().empty())
	{
		imuBuffer_.insert(std::make_pair(data.stamp(), data.imu()));
	}

	// OpenVINS has to buffer image before computing transformation with IMU stamp > image stamp
	if(!data.imageRaw().empty() && !data.rightRaw().empty() && data.stereoCameraModels().size() == 1)
	{
		if(imuBuffer_.empty())
		{
			UWARN("Waiting IMU for initialization...");
			return t;
		}
		if(vioManager_ == 0)
		{
			UINFO("OpenVINS Initialization");

			// intialize
			ov_msckf::VioManagerOptions params;

			// ESTIMATOR ======================================================================

			// Main EKF parameters
			//params.state_options.do_fej = true;
			//params.state_options.imu_avg =false;
			//params.state_options.use_rk4_integration;
			//params.state_options.do_calib_camera_pose = false;
			//params.state_options.do_calib_camera_intrinsics = false;
			//params.state_options.do_calib_camera_timeoffset = false;
			//params.state_options.max_clone_size = 11;
			//params.state_options.max_slam_features = 25;
			//params.state_options.max_slam_in_update = INT_MAX;
			//params.state_options.max_msckf_in_update = INT_MAX;
			//params.state_options.max_aruco_features = 1024;
			params.state_options.num_cameras = 2;
			//params.dt_slam_delay = 2;

			// Set what representation we should be using
			//params.state_options.feat_rep_msckf = LandmarkRepresentation::from_string("ANCHORED_MSCKF_INVERSE_DEPTH"); // default GLOBAL_3D
			//params.state_options.feat_rep_slam = LandmarkRepresentation::from_string("ANCHORED_MSCKF_INVERSE_DEPTH");  // default GLOBAL_3D
			//params.state_options.feat_rep_aruco = LandmarkRepresentation::from_string("ANCHORED_MSCKF_INVERSE_DEPTH"); // default GLOBAL_3D
			if( params.state_options.feat_rep_msckf == LandmarkRepresentation::Representation::UNKNOWN ||
				params.state_options.feat_rep_slam == LandmarkRepresentation::Representation::UNKNOWN ||
				params.state_options.feat_rep_aruco == LandmarkRepresentation::Representation::UNKNOWN)
			{
				printf(RED "VioManager(): invalid feature representation specified:\n" RESET);
				printf(RED "\t- GLOBAL_3D\n" RESET);
				printf(RED "\t- GLOBAL_FULL_INVERSE_DEPTH\n" RESET);
				printf(RED "\t- ANCHORED_3D\n" RESET);
				printf(RED "\t- ANCHORED_FULL_INVERSE_DEPTH\n" RESET);
				printf(RED "\t- ANCHORED_MSCKF_INVERSE_DEPTH\n" RESET);
				printf(RED "\t- ANCHORED_INVERSE_DEPTH_SINGLE\n" RESET);
				std::exit(EXIT_FAILURE);
			}

			// Filter initialization
			//params.init_window_time = 1;
			//params.init_imu_thresh = 1;

			// Zero velocity update
			//params.try_zupt = false;
			//params.zupt_options.chi2_multipler = 5;
			//params.zupt_max_velocity = 1;
			//params.zupt_noise_multiplier = 1;

			// NOISE ======================================================================

			// Our noise values for inertial sensor
			//params.imu_noises.sigma_w = 1.6968e-04;
			//params.imu_noises.sigma_a = 2.0000e-3;
			//params.imu_noises.sigma_wb = 1.9393e-05;
			//params.imu_noises.sigma_ab = 3.0000e-03;

			// Read in update parameters
			//params.msckf_options.sigma_pix = 1;
			//params.msckf_options.chi2_multipler = 5;
			//params.slam_options.sigma_pix = 1;
			//params.slam_options.chi2_multipler = 5;
			//params.aruco_options.sigma_pix = 1;
			//params.aruco_options.chi2_multipler = 5;


			// STATE ======================================================================

			// Timeoffset from camera to IMU
			//params.calib_camimu_dt = 0.0;

			// Global gravity
			//params.gravity[2] = 9.81;


			// TRACKERS ======================================================================

			// Tracking flags
			params.use_stereo = true;
			//params.use_klt = true;
			params.use_aruco = false;
			//params.downsize_aruco = true;
			//params.downsample_cameras = false;
			//params.use_multi_threading = true;

			// General parameters
			//params.num_pts = 200;
			//params.fast_threshold = 10;
			//params.grid_x = 10;
			//params.grid_y = 5;
			//params.min_px_dist = 8;
			//params.knn_ratio = 0.7;

			// Feature initializer parameters
			//nh.param<bool>("fi_triangulate_1d", params.featinit_options.triangulate_1d, params.featinit_options.triangulate_1d);
			//nh.param<bool>("fi_refine_features", params.featinit_options.refine_features, params.featinit_options.refine_features);
			//nh.param<int>("fi_max_runs", params.featinit_options.max_runs, params.featinit_options.max_runs);
			//nh.param<double>("fi_init_lamda", params.featinit_options.init_lamda, params.featinit_options.init_lamda);
			//nh.param<double>("fi_max_lamda", params.featinit_options.max_lamda, params.featinit_options.max_lamda);
			//nh.param<double>("fi_min_dx", params.featinit_options.min_dx, params.featinit_options.min_dx);
			///nh.param<double>("fi_min_dcost", params.featinit_options.min_dcost, params.featinit_options.min_dcost);
			//nh.param<double>("fi_lam_mult", params.featinit_options.lam_mult, params.featinit_options.lam_mult);
			//nh.param<double>("fi_min_dist", params.featinit_options.min_dist, params.featinit_options.min_dist);
			//params.featinit_options.max_dist = 75;
			//params.featinit_options.max_baseline = 500;
			//params.featinit_options.max_cond_number = 5000;


			// CAMERA ======================================================================
			bool fisheye = data.stereoCameraModels()[0].left().isFisheye() && !this->imagesAlreadyRectified();
			params.camera_fisheye.insert(std::make_pair(0, fisheye));
			params.camera_fisheye.insert(std::make_pair(1, fisheye));

			Eigen::VectorXd camLeft(8), camRight(8);
			if(this->imagesAlreadyRectified() || data.stereoCameraModels()[0].left().D_raw().empty())
			{
				camLeft << data.stereoCameraModels()[0].left().fx(),
					 data.stereoCameraModels()[0].left().fy(),
					 data.stereoCameraModels()[0].left().cx(),
					 data.stereoCameraModels()[0].left().cy(), 0, 0, 0, 0;
				camRight << data.stereoCameraModels()[0].right().fx(),
					 data.stereoCameraModels()[0].right().fy(),
					 data.stereoCameraModels()[0].right().cx(),
					 data.stereoCameraModels()[0].right().cy(), 0, 0, 0, 0;
			}
			else
			{
				UASSERT(data.stereoCameraModels()[0].left().D_raw().cols == data.stereoCameraModels()[0].right().D_raw().cols);
				UASSERT(data.stereoCameraModels()[0].left().D_raw().cols >= 4);
				UASSERT(data.stereoCameraModels()[0].right().D_raw().cols >= 4);

				//https://github.com/ethz-asl/kalibr/wiki/supported-models
				///	    radial-tangential (radtan)
				//		(distortion_coeffs: [k1 k2 r1 r2])
				///		equidistant (equi)
				//		(distortion_coeffs: [k1 k2 k3 k4]) rtabmap: (k1,k2,p1,p2,k3,k4)

				camLeft <<
					 data.stereoCameraModels()[0].left().K_raw().at<double>(0,0),
					 data.stereoCameraModels()[0].left().K_raw().at<double>(1,1),
					 data.stereoCameraModels()[0].left().K_raw().at<double>(0,2),
					 data.stereoCameraModels()[0].left().K_raw().at<double>(1,2),
					 data.stereoCameraModels()[0].left().D_raw().at<double>(0,0),
					 data.stereoCameraModels()[0].left().D_raw().at<double>(0,1),
					 data.stereoCameraModels()[0].left().D_raw().at<double>(0,fisheye?4:2),
					 data.stereoCameraModels()[0].left().D_raw().at<double>(0,fisheye?5:3);
				camRight <<
					 data.stereoCameraModels()[0].right().K_raw().at<double>(0,0),
					 data.stereoCameraModels()[0].right().K_raw().at<double>(1,1),
					 data.stereoCameraModels()[0].right().K_raw().at<double>(0,2),
					 data.stereoCameraModels()[0].right().K_raw().at<double>(1,2),
					 data.stereoCameraModels()[0].right().D_raw().at<double>(0,0),
					 data.stereoCameraModels()[0].right().D_raw().at<double>(0,1),
					 data.stereoCameraModels()[0].right().D_raw().at<double>(0,fisheye?4:2),
					 data.stereoCameraModels()[0].right().D_raw().at<double>(0,fisheye?5:3);
			}
			params.camera_intrinsics.insert(std::make_pair(0, camLeft));
			params.camera_intrinsics.insert(std::make_pair(1, camRight));

			const IMU & imu = imuBuffer_.begin()->second;
			imuLocalTransform_ = imu.localTransform();
			Transform imuCam0 = imuLocalTransform_.inverse() * data.stereoCameraModels()[0].localTransform();
			Transform cam0cam1;
			if(this->imagesAlreadyRectified() || data.stereoCameraModels()[0].stereoTransform().isNull())
			{
				cam0cam1 = Transform(
						1, 0, 0, data.stereoCameraModels()[0].baseline(),
						0, 1, 0, 0,
						0, 0, 1, 0);
			}
			else
			{
				cam0cam1 = data.stereoCameraModels()[0].stereoTransform().inverse();
			}
			UASSERT(!cam0cam1.isNull());
			Transform imuCam1 = imuCam0 * cam0cam1;
			Eigen::Matrix4d cam0_eigen = imuCam0.toEigen4d();
			Eigen::Matrix4d cam1_eigen = imuCam1.toEigen4d();
			Eigen::Matrix<double,7,1> cam_eigen0;
			cam_eigen0.block(0,0,4,1) = rot_2_quat(cam0_eigen.block(0,0,3,3).transpose());
			cam_eigen0.block(4,0,3,1) = -cam0_eigen.block(0,0,3,3).transpose()*cam0_eigen.block(0,3,3,1);
			Eigen::Matrix<double,7,1> cam_eigen1;
			cam_eigen1.block(0,0,4,1) = rot_2_quat(cam1_eigen.block(0,0,3,3).transpose());
			cam_eigen1.block(4,0,3,1) = -cam1_eigen.block(0,0,3,3).transpose()*cam1_eigen.block(0,3,3,1);
			params.camera_extrinsics.insert(std::make_pair(0, cam_eigen0));
			params.camera_extrinsics.insert(std::make_pair(1, cam_eigen1));

			params.camera_wh.insert({0, std::make_pair(data.stereoCameraModels()[0].left().imageWidth(),data.stereoCameraModels()[0].left().imageHeight())});
			params.camera_wh.insert({1, std::make_pair(data.stereoCameraModels()[0].right().imageWidth(),data.stereoCameraModels()[0].right().imageHeight())});

			vioManager_ = new ov_msckf::VioManager(params);
		}

		cv::Mat left;
		cv::Mat right;
		if(data.imageRaw().type() == CV_8UC3)
		{
			cv::cvtColor(data.imageRaw(), left, CV_BGR2GRAY);
		}
		else if(data.imageRaw().type() == CV_8UC1)
		{
			left = data.imageRaw().clone();
		}
		else
		{
			UFATAL("Not supported color type!");
		}
		if(data.rightRaw().type() == CV_8UC3)
		{
			cv::cvtColor(data.rightRaw(), right, CV_BGR2GRAY);
		}
		else if(data.rightRaw().type() == CV_8UC1)
		{
			right = data.rightRaw().clone();
		}
		else
		{
			UFATAL("Not supported color type!");
		}

		// Create the measurement
		ov_core::CameraData message;
		message.timestamp = data.stamp();
		message.sensor_ids.push_back(0);
		message.sensor_ids.push_back(1);
		message.images.push_back(left);
		message.images.push_back(right);
		message.masks.push_back(cv::Mat::zeros(left.size(), CV_8UC1));
		message.masks.push_back(cv::Mat::zeros(right.size(), CV_8UC1));

		// send it to our VIO system
		vioManager_->feed_measurement_camera(message);
		UDEBUG("Image update stamp=%f", data.stamp());

		double lastIMUstamp = 0.0;
		while(!imuBuffer_.empty())
		{
			std::map<double, IMU>::iterator iter = imuBuffer_.begin();

			// Process IMU data until stamp is over image stamp
			ov_core::ImuData message;
			message.timestamp = iter->first;
			message.wm << iter->second.angularVelocity().val[0], iter->second.angularVelocity().val[1], iter->second.angularVelocity().val[2];
			message.am << iter->second.linearAcceleration().val[0], iter->second.linearAcceleration().val[1], iter->second.linearAcceleration().val[2];

			UDEBUG("IMU update stamp=%f", message.timestamp);

			// send it to our VIO system
			vioManager_->feed_measurement_imu(message);

			lastIMUstamp = iter->first;

			imuBuffer_.erase(iter);

			if(lastIMUstamp > data.stamp())
			{
				break;
			}
		}

		if(vioManager_->initialized())
		{
			// Get the current state
			std::shared_ptr<ov_msckf::State> state = vioManager_->get_state();

			if(state->_timestamp != data.stamp())
			{
				UWARN("OpenVINS: Stamp of the current state %f is not the same "
						"than last image processed %f (last IMU stamp=%f). There could be "
						"a synchronization issue between camera and IMU. ",
						state->_timestamp,
						data.stamp(),
						lastIMUstamp);
			}

			Transform p(
					(float)state->_imu->pos()(0),
					(float)state->_imu->pos()(1),
					(float)state->_imu->pos()(2),
					(float)state->_imu->quat()(0),
					(float)state->_imu->quat()(1),
					(float)state->_imu->quat()(2),
					(float)state->_imu->quat()(3));


			// Finally set the covariance in the message (in the order position then orientation as per ros convention)
			std::vector<std::shared_ptr<ov_type::Type>> statevars;
			statevars.push_back(state->_imu->pose()->p());
			statevars.push_back(state->_imu->pose()->q());

			cv::Mat covariance = cv::Mat::eye(6,6, CV_64FC1);
			if(this->framesProcessed() == 0)
			{
				covariance *= 9999;
			}
			else
			{
				Eigen::Matrix<double,6,6> covariance_posori = ov_msckf::StateHelper::get_marginal_covariance(vioManager_->get_state(),statevars);
				for(int r=0; r<6; r++) {
					for(int c=0; c<6; c++) {
						((double *)covariance.data)[6*r+c] = covariance_posori(r,c);
					}
				}
			}

			if(!p.isNull())
			{
				p =  p * imuLocalTransform_.inverse();

				if(this->getPose().rotation().isIdentity())
				{
					initGravity_ = true;
					this->reset(this->getPose()*p.rotation());
				}

				if(previousPose_.isIdentity())
				{
					previousPose_ = p;
				}

				// make it incremental
				Transform previousPoseInv = previousPose_.inverse();
				t = previousPoseInv*p;
				previousPose_ = p;

				if(info)
				{
					info->type = this->getType();
					info->reg.covariance = covariance;

					// feature map
					Transform fixT = this->getPose()*previousPoseInv;
					Transform camLocalTransformInv = data.stereoCameraModels()[0].localTransform().inverse()*this->getPose().inverse();
					for (auto &it_per_id : vioManager_->get_features_SLAM())
					{
						cv::Point3f pt3d;
						pt3d.x = it_per_id[0];
						pt3d.y = it_per_id[1];
						pt3d.z = it_per_id[2];
						pt3d = util3d::transformPoint(pt3d, fixT);
						info->localMap.insert(std::make_pair(info->localMap.size(), pt3d));

						if(this->imagesAlreadyRectified())
						{
							cv::Point2f pt;
							pt3d = util3d::transformPoint(pt3d, camLocalTransformInv);
							data.stereoCameraModels()[0].left().reproject(pt3d.x, pt3d.y, pt3d.z, pt.x, pt.y);
							info->reg.inliersIDs.push_back(info->newCorners.size());
							info->newCorners.push_back(pt);
						}
					}
					info->features = info->newCorners.size();
					info->localMapSize = info->localMap.size();
				}
				UINFO("Odom update time = %fs p=%s", timer.elapsed(), p.prettyPrint().c_str());
			}
		}
	}
	else if(!data.imageRaw().empty() && !data.depthRaw().empty())
	{
		UERROR("OpenVINS doesn't work with RGB-D data, stereo images are required!");
	}
	else if(!data.imageRaw().empty() && data.depthOrRightRaw().empty())
	{
		UERROR("OpenVINS requires stereo images!");
	}
	else
	{
		UERROR("OpenVINS requires stereo images (only one stereo camera and should be calibrated)!");
	}

#else
	UERROR("RTAB-Map is not built with OpenVINS support! Select another visual odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
