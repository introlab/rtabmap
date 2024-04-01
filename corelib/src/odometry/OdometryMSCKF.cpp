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

#include "rtabmap/core/odometry/OdometryMSCKF.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UThread.h"
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_MSCKF_VIO
#include <msckf_vio/image_processor.h>
#include <msckf_vio/msckf_vio.h>
#include <msckf_vio/math_utils.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <boost/math/distributions/chi_squared.hpp>
#include <pcl/common/transforms.h>
#endif

namespace rtabmap {

#ifdef RTABMAP_MSCKF_VIO
class ImageProcessorNoROS: public msckf_vio::ImageProcessor
{
public:
	ImageProcessorNoROS(
			const ParametersMap & parameters_in,
			const Transform & imuLocalTransform,
			const StereoCameraModel & model,
			bool rectified) :
				msckf_vio::ImageProcessor(0)
	{
		UDEBUG("");
		// Camera calibration parameters
		if(model.left().D_raw().cols == 6)
		{
			//equidistant
			cam0_distortion_model = "equidistant";
			cam0_distortion_coeffs[0] = rectified?0:model.left().D_raw().at<double>(0,0);
			cam0_distortion_coeffs[1] = rectified?0:model.left().D_raw().at<double>(0,1);
			cam0_distortion_coeffs[2] = rectified?0:model.left().D_raw().at<double>(0,4);
			cam0_distortion_coeffs[3] = rectified?0:model.left().D_raw().at<double>(0,5);
		}
		else
		{
			//radtan
			cam0_distortion_model = "radtan";
			cam0_distortion_coeffs[0] = rectified?0:model.left().D_raw().at<double>(0,0);
			cam0_distortion_coeffs[1] = rectified?0:model.left().D_raw().at<double>(0,1);
			cam0_distortion_coeffs[2] = rectified?0:model.left().D_raw().at<double>(0,2);
			cam0_distortion_coeffs[3] = rectified?0:model.left().D_raw().at<double>(0,3);
		}
		if(model.right().D_raw().cols == 6)
		{
			//equidistant
			cam1_distortion_model = "equidistant";
			cam1_distortion_coeffs[0] = rectified?0:model.right().D_raw().at<double>(0,0);
			cam1_distortion_coeffs[1] = rectified?0:model.right().D_raw().at<double>(0,1);
			cam1_distortion_coeffs[2] = rectified?0:model.right().D_raw().at<double>(0,4);
			cam1_distortion_coeffs[3] = rectified?0:model.right().D_raw().at<double>(0,5);
		}
		else
		{
			//radtan
			cam1_distortion_model = "radtan";
			cam1_distortion_coeffs[0] = rectified?0:model.right().D_raw().at<double>(0,0);
			cam1_distortion_coeffs[1] = rectified?0:model.right().D_raw().at<double>(0,1);
			cam1_distortion_coeffs[2] = rectified?0:model.right().D_raw().at<double>(0,2);
			cam1_distortion_coeffs[3] = rectified?0:model.right().D_raw().at<double>(0,3);
		}

		cam0_resolution[0] = model.left().imageWidth();
		cam0_resolution[1] = model.left().imageHeight();

		cam1_resolution[0] = model.right().imageWidth();
		cam1_resolution[1] = model.right().imageHeight();

		cam0_intrinsics[0] = rectified?model.left().fx():model.left().K_raw().at<double>(0,0);
		cam0_intrinsics[1] = rectified?model.left().fy():model.left().K_raw().at<double>(1,1);
		cam0_intrinsics[2] = rectified?model.left().cx():model.left().K_raw().at<double>(0,2);
		cam0_intrinsics[3] = rectified?model.left().cy():model.left().K_raw().at<double>(1,2);

		cam1_intrinsics[0] = rectified?model.right().fx():model.right().K_raw().at<double>(0,0);
		cam1_intrinsics[1] = rectified?model.right().fy():model.right().K_raw().at<double>(1,1);
		cam1_intrinsics[2] = rectified?model.right().cx():model.right().K_raw().at<double>(0,2);
		cam1_intrinsics[3] = rectified?model.right().cy():model.right().K_raw().at<double>(1,2);

		UINFO("Local transform=%s", model.localTransform().prettyPrint().c_str());
		UINFO("imuLocalTransform=%s", imuLocalTransform.prettyPrint().c_str());
		Transform imuCam = model.localTransform().inverse() * imuLocalTransform;
		UINFO("imuCam=%s", imuCam.prettyPrint().c_str());
		cv::Mat     T_imu_cam0 = imuCam.dataMatrix();
		cv::Matx33d R_imu_cam0(T_imu_cam0(cv::Rect(0,0,3,3)));
		cv::Vec3d   t_imu_cam0 = T_imu_cam0(cv::Rect(3,0,1,3));
		R_cam0_imu = R_imu_cam0.t();
		t_cam0_imu = -R_imu_cam0.t() * t_imu_cam0;

		Transform cam0cam1;
		if(rectified)
		{
			cam0cam1 = Transform(
					1, 0, 0, -model.baseline(),
					0, 1, 0, 0,
					0, 0, 1, 0);
		}
		else
		{
			cam0cam1 = model.stereoTransform();
		}
		UINFO("cam0cam1=%s", cam0cam1.prettyPrint().c_str());
		UASSERT(!cam0cam1.isNull());
		Transform imuCam1 = cam0cam1 * imuCam;
		cv::Mat T_imu_cam1 = imuCam1.dataMatrix();
		cv::Matx33d R_imu_cam1(T_imu_cam1(cv::Rect(0,0,3,3)));
		cv::Vec3d   t_imu_cam1 = T_imu_cam1(cv::Rect(3,0,1,3));
		R_cam1_imu = R_imu_cam1.t();
		t_cam1_imu = -R_imu_cam1.t() * t_imu_cam1;
		// Processor parameters
		// get all OdomMSCFK group to make sure all parameters are set
		ParametersMap parameters = Parameters::getDefaultParameters("OdomMSCKF");
		uInsert(parameters, parameters_in);

		Parameters::parse(parameters, Parameters::kOdomMSCKFGridRow(), processor_config.grid_row); //4
		Parameters::parse(parameters, Parameters::kOdomMSCKFGridCol(), processor_config.grid_col); //4
		Parameters::parse(parameters, Parameters::kOdomMSCKFGridMinFeatureNum(), processor_config.grid_min_feature_num); //2
		Parameters::parse(parameters, Parameters::kOdomMSCKFGridMaxFeatureNum(), processor_config.grid_max_feature_num); //4
		Parameters::parse(parameters, Parameters::kOdomMSCKFPyramidLevels(), processor_config.pyramid_levels); //3
		Parameters::parse(parameters, Parameters::kOdomMSCKFPatchSize(), processor_config.patch_size); //31
		Parameters::parse(parameters, Parameters::kOdomMSCKFFastThreshold(), processor_config.fast_threshold); //20
		Parameters::parse(parameters, Parameters::kOdomMSCKFMaxIteration(), processor_config.max_iteration); //30
		Parameters::parse(parameters, Parameters::kOdomMSCKFTrackPrecision(), processor_config.track_precision); //0.01
		Parameters::parse(parameters, Parameters::kOdomMSCKFRansacThreshold(), processor_config.ransac_threshold); //3
		Parameters::parse(parameters, Parameters::kOdomMSCKFStereoThreshold(), processor_config.stereo_threshold); //3

		UINFO("===========================================");
		UINFO("cam0_resolution: %d, %d",
				cam0_resolution[0], cam0_resolution[1]);
		UINFO("cam0_intrinscs: %f, %f, %f, %f",
				cam0_intrinsics[0], cam0_intrinsics[1],
				cam0_intrinsics[2], cam0_intrinsics[3]);
		UINFO("cam0_distortion_model: %s",
				cam0_distortion_model.c_str());
		UINFO("cam0_distortion_coefficients: %f, %f, %f, %f",
				cam0_distortion_coeffs[0], cam0_distortion_coeffs[1],
				cam0_distortion_coeffs[2], cam0_distortion_coeffs[3]);

		UINFO("cam1_resolution: %d, %d",
				cam1_resolution[0], cam1_resolution[1]);
		UINFO("cam1_intrinscs: %f, %f, %f, %f",
				cam1_intrinsics[0], cam1_intrinsics[1],
				cam1_intrinsics[2], cam1_intrinsics[3]);
		UINFO("cam1_distortion_model: %s",
				cam1_distortion_model.c_str());
		UINFO("cam1_distortion_coefficients: %f, %f, %f, %f",
				cam1_distortion_coeffs[0], cam1_distortion_coeffs[1],
				cam1_distortion_coeffs[2], cam1_distortion_coeffs[3]);

		std::cout << "R_imu_cam0: " << R_imu_cam0 << std::endl;
		std::cout << "t_imu_cam0.t(): " << t_imu_cam0.t() << std::endl;
		std::cout << "R_imu_cam1: " << R_imu_cam1 << std::endl;
		std::cout << "t_imu_cam1.t(): " << t_imu_cam1.t() << std::endl;

		UINFO("grid_row: %d",
				processor_config.grid_row);
		UINFO("grid_col: %d",
				processor_config.grid_col);
		UINFO("grid_min_feature_num: %d",
				processor_config.grid_min_feature_num);
		UINFO("grid_max_feature_num: %d",
				processor_config.grid_max_feature_num);
		UINFO("pyramid_levels: %d",
				processor_config.pyramid_levels);
		UINFO("patch_size: %d",
				processor_config.patch_size);
		UINFO("fast_threshold: %d",
				processor_config.fast_threshold);
		UINFO("max_iteration: %d",
				processor_config.max_iteration);
		UINFO("track_precision: %f",
				processor_config.track_precision);
		UINFO("ransac_threshold: %f",
				processor_config.ransac_threshold);
		UINFO("stereo_threshold: %f",
				processor_config.stereo_threshold);
		UINFO("===========================================");

		// Create feature detector.
		detector_ptr = cv::FastFeatureDetector::create(
		  processor_config.fast_threshold);
	}

	virtual ~ImageProcessorNoROS() {}

	msckf_vio::CameraMeasurementPtr stereoCallback2(
			const sensor_msgs::ImageConstPtr& cam0_img,
			const sensor_msgs::ImageConstPtr& cam1_img) {


		//cout << "==================================" << endl;

		// Get the current image.
		cam0_curr_img_ptr = cv_bridge::toCvShare(cam0_img,
				sensor_msgs::image_encodings::MONO8);
		cam1_curr_img_ptr = cv_bridge::toCvShare(cam1_img,
				sensor_msgs::image_encodings::MONO8);

		// Build the image pyramids once since they're used at multiple places
		createImagePyramids();

		// Detect features in the first frame.
		if (is_first_img) {
			//ros::Time start_time = ros::Time::now();
			initializeFirstFrame();
			//UINFO("Detection time: %f",
			//    (ros::Time::now()-start_time).toSec());
			is_first_img = false;

			// Draw results.
			//start_time = ros::Time::now();
			//drawFeaturesStereo();
			//UINFO("Draw features: %f",
			//    (ros::Time::now()-start_time).toSec());
		} else {
			// Track the feature in the previous image.
			//ros::Time start_time = ros::Time::now();
			trackFeatures();
			//UINFO("Tracking time: %f",
			//    (ros::Time::now()-start_time).toSec());

			// Add new features into the current image.
			//start_time = ros::Time::now();
			addNewFeatures();
			//UINFO("Addition time: %f",
			//    (ros::Time::now()-start_time).toSec());

			// Add new features into the current image.
			//start_time = ros::Time::now();
			pruneGridFeatures();
			//UINFO("Prune grid features: %f",
			//    (ros::Time::now()-start_time).toSec());

			// Draw results.
			//start_time = ros::Time::now();
			//drawFeaturesStereo();
			//UINFO("Draw features: %f",
			//    (ros::Time::now()-start_time).toSec());
		}

		//ros::Time start_time = ros::Time::now();
		//updateFeatureLifetime();
		//UINFO("Statistics: %f",
		//    (ros::Time::now()-start_time).toSec());

		// Publish features in the current image.
		//ros::Time start_time = ros::Time::now();
		msckf_vio::CameraMeasurementPtr measurements = publish();
		//UINFO("Publishing: %f",
		//    (ros::Time::now()-start_time).toSec());

		// Update the previous image and previous features.
		cam0_prev_img_ptr = cam0_curr_img_ptr;
		prev_features_ptr = curr_features_ptr;
		std::swap(prev_cam0_pyramid_, curr_cam0_pyramid_);

		// Initialize the current features to empty vectors.
		curr_features_ptr.reset(new GridFeatures());
		for (int code = 0; code <
		processor_config.grid_row*processor_config.grid_col; ++code) {
			(*curr_features_ptr)[code] = std::vector<FeatureMetaData>(0);
		}

		return measurements;
	}

	msckf_vio::CameraMeasurementPtr publish() {

		// Publish features.
		msckf_vio::CameraMeasurementPtr feature_msg_ptr(new msckf_vio::CameraMeasurement);
		feature_msg_ptr->header.stamp = cam0_curr_img_ptr->header.stamp;

		std::vector<FeatureIDType> curr_ids(0);
		std::vector<cv::Point2f> curr_cam0_points(0);
		std::vector<cv::Point2f> curr_cam1_points(0);

		for (const auto& grid_features : (*curr_features_ptr)) {
			for (const auto& feature : grid_features.second) {
				curr_ids.push_back(feature.id);
				curr_cam0_points.push_back(feature.cam0_point);
				curr_cam1_points.push_back(feature.cam1_point);
			}
		}

		std::vector<cv::Point2f> curr_cam0_points_undistorted(0);
		std::vector<cv::Point2f> curr_cam1_points_undistorted(0);

		undistortPoints(
				curr_cam0_points, cam0_intrinsics, cam0_distortion_model,
				cam0_distortion_coeffs, curr_cam0_points_undistorted);
		undistortPoints(
				curr_cam1_points, cam1_intrinsics, cam1_distortion_model,
				cam1_distortion_coeffs, curr_cam1_points_undistorted);

		for (unsigned int i = 0; i < curr_ids.size(); ++i) {
			feature_msg_ptr->features.push_back(msckf_vio::FeatureMeasurement());
			feature_msg_ptr->features[i].id = curr_ids[i];
			feature_msg_ptr->features[i].u0 = curr_cam0_points_undistorted[i].x;
			feature_msg_ptr->features[i].v0 = curr_cam0_points_undistorted[i].y;
			feature_msg_ptr->features[i].u1 = curr_cam1_points_undistorted[i].x;
			feature_msg_ptr->features[i].v1 = curr_cam1_points_undistorted[i].y;
		}

		//feature_pub.publish(feature_msg_ptr);

		// Publish tracking info.
		/*TrackingInfoPtr tracking_info_msg_ptr(new TrackingInfo());
		tracking_info_msg_ptr->header.stamp = cam0_curr_img_ptr->header.stamp;
		tracking_info_msg_ptr->before_tracking = before_tracking;
		tracking_info_msg_ptr->after_tracking = after_tracking;
		tracking_info_msg_ptr->after_matching = after_matching;
		tracking_info_msg_ptr->after_ransac = after_ransac;
		tracking_info_pub.publish(tracking_info_msg_ptr);*/

		return feature_msg_ptr;
	}
};

class MsckfVioNoROS: public msckf_vio::MsckfVio
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	MsckfVioNoROS(const ParametersMap & parameters_in,
			const Transform & imuLocalTransform,
			const StereoCameraModel & model,
			bool rectified) :
			msckf_vio::MsckfVio(0)
	{
		UDEBUG("");
		// get all OdomMSCFK group to make sure all parameters are set
		parameters_ = Parameters::getDefaultParameters("OdomMSCKF");
		uInsert(parameters_, parameters_in);

		// Frame id
		publish_tf = false;
		frame_rate = 1.0;
		Parameters::parse(parameters_, Parameters::kOdomMSCKFPositionStdThreshold(), position_std_threshold); //8.0

		Parameters::parse(parameters_, Parameters::kOdomMSCKFRotationThreshold(), rotation_threshold); //0.2618
		Parameters::parse(parameters_, Parameters::kOdomMSCKFTranslationThreshold(), translation_threshold); //0.4
		Parameters::parse(parameters_, Parameters::kOdomMSCKFTrackingRateThreshold(), tracking_rate_threshold); //0.5

		// Feature optimization parameters
		Parameters::parse(parameters_, Parameters::kOdomMSCKFOptTranslationThreshold(), msckf_vio::Feature::optimization_config.translation_threshold); //0.2

		// Noise related parameters
		Parameters::parse(parameters_, Parameters::kOdomMSCKFNoiseGyro(), msckf_vio::IMUState::gyro_noise); //0.001
		Parameters::parse(parameters_, Parameters::kOdomMSCKFNoiseAcc(), msckf_vio::IMUState::acc_noise); //0.01
		Parameters::parse(parameters_, Parameters::kOdomMSCKFNoiseGyroBias(), msckf_vio::IMUState::gyro_bias_noise); //0.001
		Parameters::parse(parameters_, Parameters::kOdomMSCKFNoiseAccBias(), msckf_vio::IMUState::acc_bias_noise); //0.01
		Parameters::parse(parameters_, Parameters::kOdomMSCKFNoiseFeature(), msckf_vio::Feature::observation_noise); //0.01

		// Use variance instead of standard deviation.
		msckf_vio::IMUState::gyro_noise *= msckf_vio::IMUState::gyro_noise;
		msckf_vio::IMUState::acc_noise *= msckf_vio::IMUState::acc_noise;
		msckf_vio::IMUState::gyro_bias_noise *= msckf_vio::IMUState::gyro_bias_noise;
		msckf_vio::IMUState::acc_bias_noise *= msckf_vio::IMUState::acc_bias_noise;
		msckf_vio::Feature::observation_noise *= msckf_vio::Feature::observation_noise;

		// Set the initial IMU state.
		// The intial orientation and position will be set to the origin
		// implicitly. But the initial velocity and bias can be
		// set by parameters.
		// TODO: is it reasonable to set the initial bias to 0?
		//Parameters::parse(parameters_, "initial_state/velocity/x", state_server.imu_state.velocity(0)); //0.0
		//Parameters::parse(parameters_, "initial_state/velocity/y", state_server.imu_state.velocity(1)); //0.0
		//Parameters::parse(parameters_, "initial_state/velocity/z", state_server.imu_state.velocity(2)); //0.0

		// The initial covariance of orientation and position can be
		// set to 0. But for velocity, bias and extrinsic parameters,
		// there should be nontrivial uncertainty.
		double gyro_bias_cov, acc_bias_cov, velocity_cov;
		Parameters::parse(parameters_, Parameters::kOdomMSCKFInitCovVel(), velocity_cov); //0.25
		Parameters::parse(parameters_, Parameters::kOdomMSCKFInitCovGyroBias(), gyro_bias_cov); //1e-4
		Parameters::parse(parameters_, Parameters::kOdomMSCKFInitCovAccBias(), acc_bias_cov); //1e-2

		double extrinsic_rotation_cov, extrinsic_translation_cov;
		Parameters::parse(parameters_, Parameters::kOdomMSCKFInitCovExRot(), extrinsic_rotation_cov); //3.0462e-4
		Parameters::parse(parameters_, Parameters::kOdomMSCKFInitCovExTrans(), extrinsic_translation_cov); //1e-4

		state_server.state_cov = Eigen::MatrixXd::Zero(21, 21);
		for (int i = 3; i < 6; ++i)
			state_server.state_cov(i, i) = gyro_bias_cov;
		for (int i = 6; i < 9; ++i)
			state_server.state_cov(i, i) = velocity_cov;
		for (int i = 9; i < 12; ++i)
			state_server.state_cov(i, i) = acc_bias_cov;
		for (int i = 15; i < 18; ++i)
			state_server.state_cov(i, i) = extrinsic_rotation_cov;
		for (int i = 18; i < 21; ++i)
			state_server.state_cov(i, i) = extrinsic_translation_cov;

		// Transformation offsets between the frames involved.
		UINFO("Local transform=%s", model.localTransform().prettyPrint().c_str());
		UINFO("imuLocalTransform=%s", imuLocalTransform.prettyPrint().c_str());
		Transform imuCam = model.localTransform().inverse() * imuLocalTransform;
		UINFO("imuCam=%s", imuCam.prettyPrint().c_str());
		Eigen::Isometry3d T_imu_cam0(imuCam.toEigen4d());
		Eigen::Isometry3d T_cam0_imu = T_imu_cam0.inverse();

		state_server.imu_state.R_imu_cam0 = T_cam0_imu.linear().transpose();
		state_server.imu_state.t_cam0_imu = T_cam0_imu.translation();
		Transform cam0cam1;
		if(rectified)
		{
			cam0cam1 = Transform(
					1, 0, 0, -model.baseline(),
					0, 1, 0, 0,
					0, 0, 1, 0);
		}
		else
		{
			cam0cam1 = model.stereoTransform();
		}
		UINFO("cam0cam1=%s", cam0cam1.prettyPrint().c_str());
		msckf_vio::CAMState::T_cam0_cam1 = cam0cam1.toEigen3d().matrix();
		msckf_vio::IMUState::T_imu_body = imuLocalTransform.toEigen3d().matrix();

		// Maximum number of camera states to be stored
		Parameters::parse(parameters_, Parameters::kOdomMSCKFMaxCamStateSize(), max_cam_state_size); //30

		UINFO("===========================================");
		UINFO("fixed frame id: %s", fixed_frame_id.c_str());
		UINFO("child frame id: %s", child_frame_id.c_str());
		UINFO("publish tf: %d", publish_tf);
		UINFO("frame rate: %f", frame_rate);
		UINFO("position std threshold: %f", position_std_threshold);
		UINFO("Keyframe rotation threshold: %f", rotation_threshold);
		UINFO("Keyframe translation threshold: %f", translation_threshold);
		UINFO("Keyframe tracking rate threshold: %f", tracking_rate_threshold);
		UINFO("gyro noise: %.10f", msckf_vio::IMUState::gyro_noise);
		UINFO("gyro bias noise: %.10f", msckf_vio::IMUState::gyro_bias_noise);
		UINFO("acc noise: %.10f", msckf_vio::IMUState::acc_noise);
		UINFO("acc bias noise: %.10f", msckf_vio::IMUState::acc_bias_noise);
		UINFO("observation noise: %.10f", msckf_vio::Feature::observation_noise);
		UINFO("initial velocity: %f, %f, %f",
				state_server.imu_state.velocity(0),
				state_server.imu_state.velocity(1),
				state_server.imu_state.velocity(2));
		UINFO("initial gyro bias cov: %f", gyro_bias_cov);
		UINFO("initial acc bias cov: %f", acc_bias_cov);
		UINFO("initial velocity cov: %f", velocity_cov);
		UINFO("initial extrinsic rotation cov: %f",
				extrinsic_rotation_cov);
		UINFO("initial extrinsic translation cov: %f",
				extrinsic_translation_cov);

		std::cout << "T_imu_cam0.linear(): " << T_imu_cam0.linear() << std::endl;
		std::cout << "T_imu_cam0.translation().transpose(): " << T_imu_cam0.translation().transpose() << std::endl;
		std::cout << "CAMState::T_cam0_cam1.linear(): " << msckf_vio::CAMState::T_cam0_cam1.linear() << std::endl;
		std::cout << "CAMState::T_cam0_cam1.translation().transpose(): " << msckf_vio::CAMState::T_cam0_cam1.translation().transpose() << std::endl;
		std::cout << "IMUState::T_imu_body.linear(): " << msckf_vio::IMUState::T_imu_body.linear() << std::endl;
		std::cout << "IMUState::T_imu_body.translation().transpose(): " << msckf_vio::IMUState::T_imu_body.translation().transpose() << std::endl;

		UINFO("max camera state #: %d", max_cam_state_size);
		UINFO("===========================================");

		//if (!loadParameters()) return false;
		//UINFO("Finish loading ROS parameters...");

		// Initialize state server
		state_server.continuous_noise_cov =
				Eigen::Matrix<double, 12, 12>::Zero();
		state_server.continuous_noise_cov.block<3, 3>(0, 0) =
				Eigen::Matrix3d::Identity()*msckf_vio::IMUState::gyro_noise;
		state_server.continuous_noise_cov.block<3, 3>(3, 3) =
				Eigen::Matrix3d::Identity()*msckf_vio::IMUState::gyro_bias_noise;
		state_server.continuous_noise_cov.block<3, 3>(6, 6) =
				Eigen::Matrix3d::Identity()*msckf_vio::IMUState::acc_noise;
		state_server.continuous_noise_cov.block<3, 3>(9, 9) =
				Eigen::Matrix3d::Identity()*msckf_vio::IMUState::acc_bias_noise;

		// Initialize the chi squared test table with confidence
		// level 0.95.
		for (int i = 1; i < 100; ++i) {
			boost::math::chi_squared chi_squared_dist(i);
			chi_squared_test_table[i] =
					boost::math::quantile(chi_squared_dist, 0.05);
		}

		// if (!createRosIO()) return false;
		//UINFO("Finish creating ROS IO...");
	}
	virtual ~MsckfVioNoROS() {}


	nav_msgs::Odometry featureCallback2(
			const msckf_vio::CameraMeasurementConstPtr& msg,
			pcl::PointCloud<pcl::PointXYZ>::Ptr & localMap) {

		nav_msgs::Odometry odom;

		// Return if the gravity vector has not been set.
		if (!is_gravity_set)
		{
			UINFO("Gravity not set yet... waiting for 200 IMU msgs (%d/200)...", (int)imu_msg_buffer.size());
			return odom;
		}

		// Start the system if the first image is received.
		// The frame where the first image is received will be
		// the origin.
		if (is_first_img) {
			is_first_img = false;
			state_server.imu_state.time = msg->header.stamp.toSec();
		}

		//static double max_processing_time = 0.0;
		//static int critical_time_cntr = 0;
		//double processing_start_time = ros::Time::now().toSec();

		// Propogate the IMU state.
		// that are received before the image msg.
		//ros::Time start_time = ros::Time::now();
		batchImuProcessing(msg->header.stamp.toSec());

		//double imu_processing_time = (
		//    ros::Time::now()-start_time).toSec();

		// Augment the state vector.
		//start_time = ros::Time::now();
		stateAugmentation(msg->header.stamp.toSec());
		//double state_augmentation_time = (
		//    ros::Time::now()-start_time).toSec();

		// Add new observations for existing features or new
		// features in the map server.
		//start_time = ros::Time::now();
		addFeatureObservations(msg);
		//double add_observations_time = (
		//    ros::Time::now()-start_time).toSec();

		// Perform measurement update if necessary.
		//start_time = ros::Time::now();
		removeLostFeatures();
		//double remove_lost_features_time = (
		//		ros::Time::now()-start_time).toSec();

		//start_time = ros::Time::now();
		pruneCamStateBuffer();
		//double prune_cam_states_time = (
		//		ros::Time::now()-start_time).toSec();

		// Publish the odometry.
		//start_time = ros::Time::now();
		odom = publish(localMap);
		//double publish_time = (
		//    ros::Time::now()-start_time).toSec();

		// Reset the system if necessary.
		onlineReset2();

		/*double processing_end_time = ros::Time::now().toSec();
		double processing_time =
				processing_end_time - processing_start_time;
		if (processing_time > 1.0/frame_rate) {
			++critical_time_cntr;
			UINFO("\033[1;31mTotal processing time %f/%d...\033[0m",
					processing_time, critical_time_cntr);
			//printf("IMU processing time: %f/%f\n",
			//    imu_processing_time, imu_processing_time/processing_time);
			//printf("State augmentation time: %f/%f\n",
			//    state_augmentation_time, state_augmentation_time/processing_time);
			//printf("Add observations time: %f/%f\n",
			//    add_observations_time, add_observations_time/processing_time);
			printf("Remove lost features time: %f/%f\n",
					remove_lost_features_time, remove_lost_features_time/processing_time);
			printf("Remove camera states time: %f/%f\n",
					prune_cam_states_time, prune_cam_states_time/processing_time);
			//printf("Publish time: %f/%f\n",
			//    publish_time, publish_time/processing_time);
		}*/

		return odom;
	}

	void onlineReset2() {

		// Never perform online reset if position std threshold
		// is non-positive.
		if (position_std_threshold <= 0) return;
		static long long int online_reset_counter = 0;

		// Check the uncertainty of positions to determine if
		// the system can be reset.
		double position_x_std = std::sqrt(state_server.state_cov(12, 12));
		double position_y_std = std::sqrt(state_server.state_cov(13, 13));
		double position_z_std = std::sqrt(state_server.state_cov(14, 14));

		if (position_x_std < position_std_threshold &&
				position_y_std < position_std_threshold &&
				position_z_std < position_std_threshold) return;

		UWARN("Start %lld online reset procedure...",
				++online_reset_counter);
		UINFO("Stardard deviation in xyz: %f, %f, %f",
				position_x_std, position_y_std, position_z_std);

		// Remove all existing camera states.
		state_server.cam_states.clear();

		// Clear all exsiting features in the map.
		map_server.clear();

		// Reset the state covariance.
		double gyro_bias_cov, acc_bias_cov, velocity_cov;
		Parameters::parse(parameters_, Parameters::kOdomMSCKFInitCovVel(), velocity_cov); //0.25
		Parameters::parse(parameters_, Parameters::kOdomMSCKFInitCovGyroBias(), gyro_bias_cov); //1e-4
		Parameters::parse(parameters_, Parameters::kOdomMSCKFInitCovAccBias(), acc_bias_cov); //1e-2

		double extrinsic_rotation_cov, extrinsic_translation_cov;
		Parameters::parse(parameters_, Parameters::kOdomMSCKFInitCovExRot(), extrinsic_rotation_cov); //3.0462e-4
		Parameters::parse(parameters_, Parameters::kOdomMSCKFInitCovExTrans(), extrinsic_translation_cov); //1e-4


		state_server.state_cov = Eigen::MatrixXd::Zero(21, 21);
		for (int i = 3; i < 6; ++i)
			state_server.state_cov(i, i) = gyro_bias_cov;
		for (int i = 6; i < 9; ++i)
			state_server.state_cov(i, i) = velocity_cov;
		for (int i = 9; i < 12; ++i)
			state_server.state_cov(i, i) = acc_bias_cov;
		for (int i = 15; i < 18; ++i)
			state_server.state_cov(i, i) = extrinsic_rotation_cov;
		for (int i = 18; i < 21; ++i)
			state_server.state_cov(i, i) = extrinsic_translation_cov;

		UWARN("%lld online reset complete...", online_reset_counter);
		return;
	}

	nav_msgs::Odometry publish(pcl::PointCloud<pcl::PointXYZ>::Ptr & feature_msg_ptr) {

		// Convert the IMU frame to the body frame.
		const msckf_vio::IMUState& imu_state = state_server.imu_state;
		Eigen::Isometry3d T_i_w = Eigen::Isometry3d::Identity();
		T_i_w.linear() = msckf_vio::quaternionToRotation(imu_state.orientation).transpose();
		T_i_w.translation() = imu_state.position;

		Eigen::Isometry3d T_b_w = T_i_w * msckf_vio::IMUState::T_imu_body.inverse();
		Eigen::Vector3d body_velocity =
				msckf_vio::IMUState::T_imu_body.linear() * imu_state.velocity;

		// Publish tf
		/*if (publish_tf) {
			tf::Transform T_b_w_tf;
			tf::transformEigenToTF(T_b_w, T_b_w_tf);
			tf_pub.sendTransform(tf::StampedTransform(
				T_b_w_tf, time, fixed_frame_id, child_frame_id));
		}*/

		// Publish the odometry
		nav_msgs::Odometry odom_msg;
		//odom_msg.header.stamp = time;
		odom_msg.header.frame_id = fixed_frame_id;
		odom_msg.child_frame_id = child_frame_id;

		tf::poseEigenToMsg(T_b_w, odom_msg.pose.pose);
		tf::vectorEigenToMsg(body_velocity, odom_msg.twist.twist.linear);

		// Convert the covariance.
		Eigen::Matrix3d P_oo = state_server.state_cov.block<3, 3>(0, 0);
		Eigen::Matrix3d P_op = state_server.state_cov.block<3, 3>(0, 12);
		Eigen::Matrix3d P_po = state_server.state_cov.block<3, 3>(12, 0);
		Eigen::Matrix3d P_pp = state_server.state_cov.block<3, 3>(12, 12);
		Eigen::Matrix<double, 6, 6> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
		P_imu_pose << P_pp, P_po, P_op, P_oo;

		Eigen::Matrix<double, 6, 6> H_pose = Eigen::Matrix<double, 6, 6>::Zero();
		H_pose.block<3, 3>(0, 0) = msckf_vio::IMUState::T_imu_body.linear();
		H_pose.block<3, 3>(3, 3) = msckf_vio::IMUState::T_imu_body.linear();
		Eigen::Matrix<double, 6, 6> P_body_pose = H_pose *
				P_imu_pose * H_pose.transpose();

		for (int i = 0; i < 6; ++i)
			for (int j = 0; j < 6; ++j)
				odom_msg.pose.covariance[6*i+j] = P_body_pose(i, j);

		// Construct the covariance for the velocity.
		Eigen::Matrix3d P_imu_vel = state_server.state_cov.block<3, 3>(6, 6);
		Eigen::Matrix3d H_vel = msckf_vio::IMUState::T_imu_body.linear();
		Eigen::Matrix3d P_body_vel = H_vel * P_imu_vel * H_vel.transpose();
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				odom_msg.twist.covariance[i*6+j] = P_body_vel(i, j);

		// odom_pub.publish(odom_msg);

		// Publish the 3D positions of the features that
		// has been initialized.
		feature_msg_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
		feature_msg_ptr->header.frame_id = fixed_frame_id;
		feature_msg_ptr->height = 1;
		for (const auto& item : map_server) {
			const auto& feature = item.second;
			if (feature.is_initialized) {
				feature_msg_ptr->points.push_back(pcl::PointXYZ(
					feature.position(0), feature.position(1), feature.position(2)));
			}
		}
		feature_msg_ptr->width = feature_msg_ptr->points.size();

		// feature_pub.publish(feature_msg_ptr);

		return odom_msg;
	}

private:
	ParametersMap parameters_;
};
#endif

OdometryMSCKF::OdometryMSCKF(const ParametersMap & parameters) :
									Odometry(parameters)
#ifdef RTABMAP_MSCKF_VIO
,
imageProcessor_(0),
msckf_(0),
parameters_(parameters),
fixPoseRotation_(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0),
previousPose_(Transform::getIdentity()),
initGravity_(false)
#endif
{
}

OdometryMSCKF::~OdometryMSCKF()
{
	UDEBUG("");
#ifdef RTABMAP_MSCKF_VIO
	delete imageProcessor_;
	delete msckf_;
#endif
}

void OdometryMSCKF::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_MSCKF_VIO
	if(!initGravity_)
	{
		if(imageProcessor_)
		{
			delete imageProcessor_;
			imageProcessor_ = 0;
		}
		if(msckf_)
		{
			delete msckf_;
			msckf_ = 0;
		}
		lastImu_ = IMU();
		previousPose_.setIdentity();
	}
	initGravity_ = false;
#endif
}

// return not null transform if odometry is correctly computed
Transform OdometryMSCKF::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	UDEBUG("");
	Transform t;

#ifdef RTABMAP_MSCKF_VIO
	UTimer timer;

	if(!data.imu().empty())
	{
		UDEBUG("IMU update stamp=%f acc=%f %f %f gyr=%f %f %f", data.stamp(),
				data.imu().linearAcceleration()[0],
				data.imu().linearAcceleration()[1],
				data.imu().linearAcceleration()[2],
				data.imu().angularVelocity()[0],
				data.imu().angularVelocity()[1],
				data.imu().angularVelocity()[2]);
		if(imageProcessor_ && msckf_)
		{
			sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);
			msg->angular_velocity.x = data.imu().angularVelocity()[0];
			msg->angular_velocity.y = data.imu().angularVelocity()[1];
			msg->angular_velocity.z = data.imu().angularVelocity()[2];
			msg->linear_acceleration.x = data.imu().linearAcceleration()[0];
			msg->linear_acceleration.y = data.imu().linearAcceleration()[1];
			msg->linear_acceleration.z = data.imu().linearAcceleration()[2];
			msg->header.stamp.fromSec(data.stamp());
			imageProcessor_->imuCallback(msg);
			msckf_->imuCallback(msg);
		}
		else
		{
			UWARN("Ignoring IMU, waiting for an image to initialize...");
			lastImu_ = data.imu();
		}
	}

	if(!data.imageRaw().empty() && !data.rightRaw().empty())
	{
		UDEBUG("Image update stamp=%f", data.stamp());
		if(data.stereoCameraModels().size() == 1 && data.stereoCameraModels()[0].isValidForProjection())
		{
			if(msckf_ == 0)
			{
				UINFO("Initialization");
				if(lastImu_.empty())
				{
					UWARN("Ignoring Image, waiting for imu to initialize...");
					return t;
				}
				UINFO("Creating ImageProcessorNoROS...");
				imageProcessor_ = new ImageProcessorNoROS(
						parameters_,
						lastImu_.localTransform(),
						data.stereoCameraModels()[0],
						this->imagesAlreadyRectified());
				UINFO("Creating MsckfVioNoROS...");
				msckf_ = new MsckfVioNoROS(
						parameters_,
						lastImu_.localTransform(),
						data.stereoCameraModels()[0],
						this->imagesAlreadyRectified());
			}

			// Convert to ROS
			cv_bridge::CvImage cam0;
			cv_bridge::CvImage cam1;
			cam0.header.stamp.fromSec(data.stamp());
			cam1.header.stamp.fromSec(data.stamp());

			if(data.imageRaw().type() == CV_8UC3)
			{
				cv::cvtColor(data.imageRaw(), cam0.image, CV_BGR2GRAY);
			}
			else
			{
				cam0.image = data.imageRaw();
			}
			if(data.rightRaw().type() == CV_8UC3)
			{
				cv::cvtColor(data.rightRaw(), cam1.image, CV_BGR2GRAY);
			}
			else
			{
				cam1.image = data.rightRaw();
			}

			sensor_msgs::ImagePtr cam0Msg(new sensor_msgs::Image);
			sensor_msgs::ImagePtr cam1Msg(new sensor_msgs::Image);
			cam0.toImageMsg(*cam0Msg);
			cam1.toImageMsg(*cam1Msg);
			cam0Msg->encoding = sensor_msgs::image_encodings::MONO8;
			cam1Msg->encoding = sensor_msgs::image_encodings::MONO8;

			msckf_vio::CameraMeasurementPtr measurements = imageProcessor_->stereoCallback2(cam0Msg, cam1Msg);
			pcl::PointCloud<pcl::PointXYZ>::Ptr localMap;
			nav_msgs::Odometry odom = msckf_->featureCallback2(measurements, localMap);

			if( odom.pose.pose.orientation.x != 0.0f ||
				odom.pose.pose.orientation.y != 0.0f ||
				odom.pose.pose.orientation.z != 0.0f ||
				odom.pose.pose.orientation.w != 0.0f)
			{
				Transform p = Transform(
						odom.pose.pose.position.x,
						odom.pose.pose.position.y,
						odom.pose.pose.position.z,
						odom.pose.pose.orientation.x,
						odom.pose.pose.orientation.y,
						odom.pose.pose.orientation.z,
						odom.pose.pose.orientation.w);

				p = fixPoseRotation_*p;

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
					info->features = measurements->features.size();

					info->reg.covariance = cv::Mat::zeros(6, 6, CV_64FC1);
					cv::Mat twistCov(6,6,CV_64FC1, odom.twist.covariance.elems);
					// twist covariance is not in base frame, but in world frame,
					// we have to convert the covariance in base frame
					cv::Matx31f covWorldFrame(twistCov.at<double>(0, 0),
											  twistCov.at<double>(1, 1),
											  twistCov.at<double>(2, 2));
					cv::Matx31f covBaseFrame = cv::Matx33f(previousPoseInv.rotationMatrix()) * covWorldFrame;
					// we set only diagonal values as there is an issue with g2o and off-diagonal values
					info->reg.covariance.at<double>(0, 0) = fabs(covBaseFrame.val[0])/10.0;
					info->reg.covariance.at<double>(1, 1) = fabs(covBaseFrame.val[1])/10.0;
					info->reg.covariance.at<double>(2, 2) = fabs(covBaseFrame.val[2])/10.0;
					if(info->reg.covariance.at<double>(0, 0) < 0.0001)
					{
						info->reg.covariance.at<double>(0, 0) = 0.0001;
					}
					if(info->reg.covariance.at<double>(1, 1) < 0.0001)
					{
						info->reg.covariance.at<double>(1, 1) = 0.0001;
					}
					if(info->reg.covariance.at<double>(2, 2) < 0.0001)
					{
						info->reg.covariance.at<double>(2, 2) = 0.0001;
					}
					info->reg.covariance.at<double>(3, 3) = msckf_vio::IMUState::gyro_noise*10.0;
					info->reg.covariance.at<double>(4, 4) = info->reg.covariance.at<double>(3, 3);
					info->reg.covariance.at<double>(5, 5) = info->reg.covariance.at<double>(3, 3);

					if(this->isInfoDataFilled())
					{
						if(localMap.get() && localMap->size())
						{
							Eigen::Affine3f fixRot = fixPoseRotation_.toEigen3f();
							for(unsigned int i=0; i<localMap->size(); ++i)
							{
								pcl::PointXYZ pt = pcl::transformPoint(localMap->at(i), fixRot);
								info->localMap.insert(std::make_pair(i, cv::Point3f(pt.x, pt.y, pt.z)));
							}
						}
						if(this->imagesAlreadyRectified())
						{
							info->newCorners.resize(measurements->features.size());
							float fx = data.stereoCameraModels()[0].left().fx();
							float fy = data.stereoCameraModels()[0].left().fy();
							float cx = data.stereoCameraModels()[0].left().cx();
							float cy = data.stereoCameraModels()[0].left().cy();
							info->reg.inliersIDs.resize(measurements->features.size());
							for(unsigned int i=0; i<measurements->features.size(); ++i)
							{
								info->newCorners[i].x = measurements->features[i].u0*fx+cx;
								info->newCorners[i].y = measurements->features[i].v0*fy+cy;
								info->reg.inliersIDs[i] = i;
							}
						}
					}
				}
				UINFO("Odom update time = %fs p=%s", timer.elapsed(), p.prettyPrint().c_str());
			}
		}
	}

#else
	UERROR("RTAB-Map is not built with MSCKF_VIO support! Select another visual odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
