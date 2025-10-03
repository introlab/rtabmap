/*
Copyright (c) 2025 Felix Toft
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

#include "rtabmap/core/odometry/OdometryCuVSLAM.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"

#ifdef RTABMAP_CUVSLAM
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/util3d_transforms.h"
#include <cuvslam.h>
#include <ground_constraint.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <cuda_runtime.h>

// ============================================================================
// Coordinate System Transformation Constants
// Based on Isaac ROS implementation:
// Source: isaac_ros_visual_slam/include/isaac_ros_visual_slam/impl/cuvslam_ros_conversion.hpp
// ============================================================================

// Transformation converting from
// Canonical ROS Frame (x-forward, y-left, z-up) to
// cuVSLAM Frame       (x-right, y-up, z-backward)
//  x     ->    -z
//  y     ->    -x
//  z     ->     y
const rtabmap::Transform cuvslam_pose_canonical(
    0, -1, 0, 0,
    0, 0, 1, 0,
    -1, 0, 0, 0
);

// Transformation converting from
// cuVSLAM Frame       (x-right, y-up, z-backward) to
// Canonical ROS Frame (x-forward, y-left, z-up)
const rtabmap::Transform canonical_pose_cuvslam = cuvslam_pose_canonical.inverse();

// Transformation converting from
// Optical Frame    (x-right, y-down, z-forward) to
// cuVSLAM Frame    (x-right, y-up, z-backward)
// Optical   ->  cuVSLAM
//    x      ->     x
//    y      ->    -y
//    z      ->    -z
const rtabmap::Transform cuvslam_pose_optical(
    1, 0, 0, 0,
    0, -1, 0, 0,
    0, 0, -1, 0
);

// Transformation converting from
// cuVSLAM Frame    (x-right, y-up, z-backward) to
// Optical Frame    (x-right, y-down, z-forward)
const rtabmap::Transform optical_pose_cuvslam = cuvslam_pose_optical.inverse();


// ============================================================================
// Forward Declarations
// ============================================================================

namespace rtabmap {

bool initializeCuVSLAM(const SensorData & data, 
                       CUVSLAM_TrackerHandle & cuvslam_handle,
                       CUVSLAM_GroundConstraintHandle & ground_constraint_handle,
                       bool planar_constraints,
                       std::vector<uint8_t *> & gpu_left_image_data,
                       std::vector<uint8_t *> & gpu_right_image_data,
                       std::vector<size_t> & gpu_left_image_sizes,
                       std::vector<size_t> & gpu_right_image_sizes,
                       std::vector<CUVSLAM_Camera> & cuvslam_cameras,
	                   std::vector<std::array<float, 12>> & intrinsics,
                       cudaStream_t & cuda_stream);
    
CUVSLAM_Configuration CreateConfiguration(const SensorData & data);

bool prepareImages(const SensorData & data, 
                    std::vector<CUVSLAM_Image> & cuvslam_images,
                    std::vector<uint8_t *> & gpu_left_image_data,
                    std::vector<uint8_t *> & gpu_right_image_data,
                    std::vector<size_t> & gpu_left_image_sizes,
                    std::vector<size_t> & gpu_right_image_sizes,
                    cudaStream_t & cuda_stream);

cv::Mat convertCuVSLAMCovariance(const float * cuvslam_covariance);

void printCovarianceMatrix(const cv::Mat & cov, const std::string & label);
void printRawCuvslamCovariance(const float * cuvslam_covariance, const std::string & label);


// ============================================================================
// Transform Conversion Functions and Misc Helpers
// ============================================================================

// Helper function that converts RTAB-Map Transform into CUVSLAM_Pose
CUVSLAM_Pose TocuVSLAMPose(const Transform & rtabmap_transform)
{
  CUVSLAM_Pose cuvslamPose;
  // RTAB-Map Transform is row major, but cuVSLAM is column major
  // We need to transpose the rotation matrix when converting
  const int32_t kRotationMatCol = 3;
  const int32_t kRotationMatRow = 3;
  int cuvslam_idx = 0;
  for (int col_idx = 0; col_idx < kRotationMatCol; ++col_idx) {
    for (int row_idx = 0; row_idx < kRotationMatRow; ++row_idx) {
      // Access RTAB-Map Transform as (row, col) but store in column-major order for cuVSLAM
      cuvslamPose.r[cuvslam_idx] = rtabmap_transform(row_idx, col_idx);
      cuvslam_idx++;
    }
  }

  cuvslamPose.t[0] = rtabmap_transform.x();
  cuvslamPose.t[1] = rtabmap_transform.y();
  cuvslamPose.t[2] = rtabmap_transform.z();
  return cuvslamPose;
}

// Helper function to convert cuVSLAM pose to RTAB-Map Transform
Transform FromcuVSLAMPose(const CUVSLAM_Pose & cuvslam_pose)
{
  const auto & r = cuvslam_pose.r;
  const auto & t = cuvslam_pose.t;
  // RTAB-Map Transform is row major and cuVSLAM rotation mat is column major.
  Transform rtabmap_transform(
    r[0], r[3], r[6], t[0],  // r11, r12, r13, tx
    r[1], r[4], r[7], t[1],  // r21, r22, r23, ty
    r[2], r[5], r[8], t[2]   // r31, r32, r33, tz
  );

  return rtabmap_transform;
}

void PrintConfiguration(const CUVSLAM_Configuration & cfg)
{
  UINFO("Use use_gpu: %s", cfg.use_gpu ? "true" : "false");
  UINFO("Enable IMU Fusion: %s", cfg.enable_imu_fusion ? "true" : "false");
  if (cfg.enable_imu_fusion) {
    UINFO("gyroscope_noise_density: %f",
      cfg.imu_calibration.gyroscope_noise_density);
    UINFO("gyroscope_random_walk: %f",
      cfg.imu_calibration.gyroscope_random_walk);
    UINFO("accelerometer_noise_density: %f",
      cfg.imu_calibration.accelerometer_noise_density);
    UINFO("accelerometer_random_walk: %f",
      cfg.imu_calibration.accelerometer_random_walk);
    UINFO("frequency: %f",
      cfg.imu_calibration.frequency);
  }
}

} // namespace rtabmap

#endif

// ============================================================================
// OdometryCuVSLAM Class Implementation
// ============================================================================

namespace rtabmap {

OdometryCuVSLAM::OdometryCuVSLAM(const ParametersMap & parameters) :
    Odometry(parameters)
#ifdef RTABMAP_CUVSLAM
    ,
    cuvslam_handle_(nullptr),
    ground_constraint_handle_(nullptr),
    initialized_(false),
    lost_(false),
    tracking_(false),
    planar_constraints_(false),
    previous_pose_(Transform::getIdentity()),
    last_timestamp_(-1.0),
    observations_(5000),
	landmarks_(5000),
    gpu_left_image_data_(),
    gpu_right_image_data_(),
    gpu_left_image_sizes_(),
    gpu_right_image_sizes_(),
    cuda_stream_(nullptr)
#endif
{
#ifdef RTABMAP_CUVSLAM
    Parameters::parse(parameters, Parameters::kRegForce3DoF(), planar_constraints_);
#endif
}

OdometryCuVSLAM::~OdometryCuVSLAM()
{
#ifdef RTABMAP_CUVSLAM
    // Clean up cuVSLAM handles
    if(cuvslam_handle_)
    {
        CUVSLAM_DestroyTracker(cuvslam_handle_);
    }
    if(ground_constraint_handle_){
        CUVSLAM_GroundConstraintDestroy(ground_constraint_handle_);
    }
    
    // Clean up GPU memory
    for(uint8_t * gpu_ptr : gpu_left_image_data_) {
        if(gpu_ptr) {
            cudaFree(gpu_ptr);
        }
    }
    for(uint8_t * gpu_ptr : gpu_right_image_data_) {
        if(gpu_ptr) {
            cudaFree(gpu_ptr);
        }
    }
    if(cuda_stream_) {
        cudaStreamDestroy(cuda_stream_);
        cuda_stream_ = nullptr;
    }
#endif
}

void OdometryCuVSLAM::reset(const Transform & initialPose)
{
    Odometry::reset(initialPose);
    
#ifdef RTABMAP_CUVSLAM
    // Clean up cuVSLAM handles
    if(cuvslam_handle_)
    {
        CUVSLAM_DestroyTracker(cuvslam_handle_);
        cuvslam_handle_ = nullptr;
    }
    if(ground_constraint_handle_){
        CUVSLAM_GroundConstraintDestroy(ground_constraint_handle_);
        ground_constraint_handle_ = nullptr;
    }
    
    // Clean up GPU memory
    for(uint8_t * gpu_ptr : gpu_left_image_data_) {
        if(gpu_ptr) {
            cudaFree(gpu_ptr);
        }
    }
    gpu_left_image_data_.clear();
    for(uint8_t * gpu_ptr : gpu_right_image_data_) {
        if(gpu_ptr) {
            cudaFree(gpu_ptr);
        }
    }
    gpu_right_image_data_.clear();
    if(cuda_stream_) {
        cudaStreamDestroy(cuda_stream_);
        cuda_stream_ = nullptr;
    }
    
    // Reset our internal state variables
    gpu_left_image_sizes_.clear();
    gpu_right_image_sizes_.clear();
    cuvslam_cameras_.clear();
    intrinsics_.clear();
    initialized_ = false;
    lost_ = false;
    tracking_ = false;
    previous_pose_ = Transform::getIdentity();
    last_timestamp_ = -1.0;
#endif
}

Transform OdometryCuVSLAM::computeTransform(
    SensorData & data,
    const Transform & guess,
    OdometryInfo * info)
{    
#ifdef RTABMAP_CUVSLAM
    UTimer timer;

    // If we are lost after tracking has begun, return null transform
    // We wait until a reset is triggered.
    if(lost_ && tracking_) {
        return Transform();
    }
    
    // Check if we have valid image data
    if(data.imageRaw().empty() || data.rightRaw().empty())
    {
        UERROR("cuVSLAM odometry only works with stereo cameras! It requires both left and right images! Left: %s, Right: %s", 
               data.imageRaw().empty() ? "empty" : "ok", 
               data.rightRaw().empty() ? "empty" : "ok");       
        return Transform();
    }

    // Check if we have valid stereo camera models
    if(data.stereoCameraModels().size() == 0)
    {
        UERROR("cuVSLAM odometry requires stereo camera models!");
        return Transform();
    }

    // Initialize cuVSLAM tracker on first frame
    if(!initialized_)
    {   
        if(!initializeCuVSLAM(
            data, 
            cuvslam_handle_,
            ground_constraint_handle_,
            planar_constraints_,
            gpu_left_image_data_,
            gpu_right_image_data_,
            gpu_left_image_sizes_,
            gpu_right_image_sizes_,
            cuvslam_cameras_,
	        intrinsics_,
            cuda_stream_))
        {
            UERROR("Failed to initialize cuVSLAM tracker");
            return Transform();
        }
        
        initialized_ = true;
        if(info)
        {
            info->type = 0;
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
            info->timeEstimation = timer.ticks();
        }
        last_timestamp_ = data.stamp();
        return Transform();
    }
    
    // Prepare images for cuVSLAM
    std::vector<CUVSLAM_Image> cuvslam_image_objects;
    if(!prepareImages(
        data,
        cuvslam_image_objects,
        gpu_left_image_data_,
        gpu_right_image_data_,
        gpu_left_image_sizes_,
        gpu_right_image_sizes_,
        cuda_stream_))
    {
        UERROR("Failed to prepare images for cuVSLAM");
        return Transform();
    }
    
    // Process IMU data if available
    if(!data.imu().empty())
    {
        // TODO: Implement IMU processing
        UWARN("IMU data available but processing not implemented yet");
    }
    
    // Validate images and tracker status
    if(cuvslam_image_objects.empty()) {
        UERROR("No images prepared for cuVSLAM tracking");
        return Transform();
    }
    if(!cuvslam_handle_) {
        UERROR("cuVSLAM tracker is null! initialized_: %s", initialized_ ? "true" : "false");
        return Transform();
    }

    // Process wheel odom pose if available
    CUVSLAM_Pose * predicted_pose_ptr = nullptr;
    CUVSLAM_Pose predicted_pose;
    if(!guess.isNull()) {
        Transform absolute_guess = previous_pose_ * guess;
        absolute_guess = cuvslam_pose_canonical * absolute_guess * canonical_pose_cuvslam;
        predicted_pose = TocuVSLAMPose(absolute_guess);
        predicted_pose_ptr = &predicted_pose;
    }

    CUVSLAM_PoseEstimate vo_pose_estimate;
    const CUVSLAM_Status vo_status = CUVSLAM_TrackGpuMem(
        cuvslam_handle_, 
        cuvslam_image_objects.data(), 
        cuvslam_image_objects.size(), 
        predicted_pose_ptr,  // can safely handle nullptr if no guess is provided
        &vo_pose_estimate
    );

    if(vo_status != CUVSLAM_SUCCESS)
    {
        // Provide specific error message
        const char * error_msg = "Unknown error";
        switch(vo_status) {
            case 1: error_msg = "CUVSLAM_TRACKING_LOST"; break;
            case 2: error_msg = "CUVSLAM_INVALID_PARAMETER"; break;
            case 3: error_msg = "CUVSLAM_INVALID_IMAGE_FORMAT or CUVSLAM_INVALID_CAMERA_CONFIG"; break;
            case 4: error_msg = "CUVSLAM_GPU_MEMORY_ERROR"; break;
            case 5: error_msg = "CUVSLAM_INITIALIZATION_ERROR"; break;
            default: error_msg = "Unknown cuVSLAM error"; break;
        }
        
        UERROR("cuVSLAM tracking error: %d (%s)", vo_status, error_msg);
        
        if(info)
        {
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
            info->timeEstimation = timer.ticks();
        }
    
        last_timestamp_ = data.stamp();    
        return Transform();
    }

    // Check if we have invalid covariance values
    bool valid_covariance = true;
    for(int i = 0; i < 6; i++)
    {
        float & diag_val = vo_pose_estimate.covariance[i*6+i];
        // We allow 1.0 as a valid value, since cuVSLAM sends identity covariance for the first few frames.
        if(!std::isfinite(diag_val) || diag_val <= 0.0 || (diag_val > 0.1 && diag_val != 1.0))
        {
            diag_val = 9999.0; // Set to high uncertainty
            valid_covariance = false;
        }
    }

    // Convert to RTABMAP covariance format and scale to meet RTABMAP expectations
    cv::Mat covMat = convertCuVSLAMCovariance(vo_pose_estimate.covariance);

    // Handle invalid covariance. Protect against low velocity cases.
    if(!valid_covariance) {
        double velocity_ms = 9999.0;
        double angular_velocity_rad_s = 9999.0;
        if(!guess.isNull()) {
            double time_s = data.stamp() - last_timestamp_;
            velocity_ms = guess.getNorm() / time_s;
            angular_velocity_rad_s = guess.getAngle(Transform::getIdentity()) / time_s;
        }
        if(velocity_ms < 0.1 && angular_velocity_rad_s < 0.1 && last_timestamp_ != -1.0) {
            covMat = cv::Mat::eye(6, 6, CV_64FC1) * 0.0001;
        } else {
            // If we have already begun tracking, now we are lost.
            if(tracking_) {
                UWARN("LOST: Velocity is high and covariance is invalid, setting lost to true");
                lost_ = true;
            }
            // Still send covariance for debugging
            if(info) {
                info->reg.covariance = covMat;
            }
            return Transform();
        }
    }
    
    // Tracking was successful and the covariance is valid, set tracking to true
    tracking_ = true;

    if(info)
    {
        info->reg.covariance = covMat;
        info->timeEstimation = timer.ticks();
    }

    // Apply ground constraint
    if(planar_constraints_) {
        if(CUVSLAM_GroundConstraintAddNextPose(ground_constraint_handle_, &vo_pose_estimate.pose) != CUVSLAM_SUCCESS) {
            UERROR("Failed to add next pose to ground constraint");
            return Transform();
        }
        if(CUVSLAM_GroundConstraintGetPoseOnGround(ground_constraint_handle_, &vo_pose_estimate.pose) != CUVSLAM_SUCCESS) {
            UERROR("Failed to get pose on ground");
            return Transform();
        }
    }

    // Convert cuVSLAM pose to RTAB-Map Transform
    Transform current_pose = FromcuVSLAMPose(vo_pose_estimate.pose);
    current_pose = canonical_pose_cuvslam * current_pose * cuvslam_pose_canonical;

    // Calculate incremental transform
    UASSERT(!previous_pose_.isNull());
    Transform transform = previous_pose_.inverse() * current_pose;

    // Fill info with visualization data
    if(info) {
        if(data.stereoCameraModels().size()==1) {
            info->type = kTypeF2F;

            // extract 2D VO observations for visualization
            CUVSLAM_ObservationVector observation_vector;
            observation_vector.max = observations_.size();
            observation_vector.observations = observations_.data();
            CUVSLAM_Status observation_status = CUVSLAM_GetLastLeftObservations(cuvslam_handle_, &observation_vector);
            if(observation_status == CUVSLAM_SUCCESS && observation_vector.num > 0) {
                info->newCorners.reserve(observation_vector.num);
                for(uint32_t i = 0; i < observation_vector.num; ++i)
                {
                    const CUVSLAM_Observation & observation = observation_vector.observations[i];
                    info->newCorners.emplace_back(observation.u, observation.v);
                }
            }
        }
        else {
            info->type = kTypeF2M;
        }
        
        // extract 3D VO landmarks for visualization
        CUVSLAM_LandmarkVector landmark_vector;
        landmark_vector.max = landmarks_.size();
        landmark_vector.landmarks = landmarks_.data();
        CUVSLAM_Status landmark_status = CUVSLAM_GetLastLandmarks(cuvslam_handle_, &landmark_vector);
        std::vector<Transform> local_transform_inv(data.stereoCameraModels().size());
        for(size_t i=0; i<data.stereoCameraModels().size(); ++i) {
            local_transform_inv[i] = data.stereoCameraModels()[i].localTransform().inverse();
        }
        int image_width = data.imageRaw().cols / data.stereoCameraModels().size();
        if(landmark_status == CUVSLAM_SUCCESS && landmark_vector.num > 0) {
            Transform absolute_pose = this->getPose() * transform;
            for(uint32_t i = 0; i < landmark_vector.num; ++i)
            {
                const CUVSLAM_Landmark & landmark = landmark_vector.landmarks[i];
                cv::Point3f pt = util3d::transformPoint(cv::Point3f(landmark.x, landmark.y, landmark.z), canonical_pose_cuvslam);
                info->localMap.insert(std::make_pair(landmark.id, util3d::transformPoint(pt, absolute_pose)));
                if(data.stereoCameraModels().size() > 1) {
                    for(size_t i=0; i<data.stereoCameraModels().size(); ++i) {
                        cv::Point3f pt_in_cam = util3d::transformPoint(pt, local_transform_inv[i]);
                        float u,v;
                        if(pt_in_cam.z > 0)
                        {
                            data.stereoCameraModels()[i].left().reproject(pt_in_cam.x, pt_in_cam.y, pt_in_cam.z, u, v);
                            if(data.stereoCameraModels()[i].left().inFrame(u,v))
                            {
                                info->words.insert(std::make_pair(landmark.id, cv::KeyPoint(u + i*image_width, v, 3)));
                                info->reg.inliersIDs.push_back(landmark.id);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    previous_pose_ = current_pose;
    last_timestamp_ = data.stamp();
    return transform;
#else
    UERROR("cuVSLAM support not compiled in RTAB-Map");\
    return Transform();
#endif
    
}

#ifdef RTABMAP_CUVSLAM

// ============================================================================
// cuVSLAM Initialization and Configuration
// ============================================================================

bool initializeCuVSLAM(const SensorData & data,
                       CUVSLAM_TrackerHandle & cuvslam_handle,
                       CUVSLAM_GroundConstraintHandle & ground_constraint_handle,
                       bool planar_constraints,
                       std::vector<uint8_t *> & gpu_left_image_data,
                       std::vector<uint8_t *> & gpu_right_image_data,
                       std::vector<size_t> & gpu_left_image_sizes,
                       std::vector<size_t> & gpu_right_image_sizes,
                       std::vector<CUVSLAM_Camera> & cuvslam_cameras,
	                   std::vector<std::array<float, 12>> & intrinsics,
                       cudaStream_t & cuda_stream)
{
    // cuVSLAM verbosity level (0=none, 1=errors, 2=warnings, 3=info)
    CUVSLAM_SetVerbosity(0);

    // Initialize cuVSLAM cameras and intrinsic vectors
    cuvslam_cameras.resize(data.stereoCameraModels().size()*2);
    intrinsics.resize(data.stereoCameraModels().size()*2);

    // Handle stereo cameras
    for(size_t i = 0; i < data.stereoCameraModels().size(); ++i)
    {
        const StereoCameraModel & stereoModel = data.stereoCameraModels()[i];
        if(!stereoModel.isValidForProjection())
        {
            UERROR("Invalid stereo camera model %d for cuVSLAM initialization!", static_cast<int>(i));
            return false;
        }
        const CameraModel & leftModel = stereoModel.left();
        const CameraModel & rightModel = stereoModel.right();

        auto & cam_left = cuvslam_cameras[i*2];
        auto & cam_right = cuvslam_cameras[i*2+1];
        auto & intrinsics_left = intrinsics[i*2];
        auto & intrinsics_right = intrinsics[i*2+1];

        // Left camera
        cam_left.parameters = intrinsics_left.data();
        cam_left.width = leftModel.imageWidth();
        cam_left.height = leftModel.imageHeight();
        cam_left.distortion_model = "pinhole";
        cam_left.num_parameters = 4;
        intrinsics_left[0] = leftModel.cx();
        intrinsics_left[1] = leftModel.cy();
        intrinsics_left[2] = leftModel.fx();
        intrinsics_left[3] = leftModel.fy();

        // Transform sequence:
        // cuvslam -> optical -> camera extrinsics in optical -> cuvslam
        rtabmap::Transform extrinsics = cuvslam_pose_canonical * stereoModel.localTransform() * optical_pose_cuvslam;
        cam_left.pose = TocuVSLAMPose(extrinsics);
        cam_left.border_top = 0;
        cam_left.border_bottom = leftModel.imageHeight();
        cam_left.border_left = 0;
        cam_left.border_right = leftModel.imageWidth();

        // Right camera
        cam_right.parameters = intrinsics_right.data();
        cam_right.width = rightModel.imageWidth();
        cam_right.height = rightModel.imageHeight();
        cam_right.distortion_model = "pinhole";
        cam_right.num_parameters = 4;
        intrinsics_right[0] = rightModel.cx();
        intrinsics_right[1] = rightModel.cy();
        intrinsics_right[2] = rightModel.fx();
        intrinsics_right[3] = rightModel.fy();
        Transform baseline_transform(1, 0, 0, stereoModel.baseline(),
                                     0, 1, 0, 0,
                                     0, 0, 1, 0);
        // Transform sequence:
        // cuvslam -> optical -> baseline offset in optical -> camera extrinsics in optical -> cuvslam
        extrinsics = cuvslam_pose_canonical * stereoModel.localTransform() * baseline_transform * optical_pose_cuvslam;
        cam_right.pose = TocuVSLAMPose(extrinsics);
        cam_right.border_top = 0;
        cam_right.border_bottom = rightModel.imageHeight();
        cam_right.border_left = 0;
        cam_right.border_right = rightModel.imageWidth();
    }
    
    // Set up camera rig
    CUVSLAM_CameraRig camera_rig;
    camera_rig.cameras = cuvslam_cameras.data();
    camera_rig.num_cameras = cuvslam_cameras.size();

    const CUVSLAM_Configuration configuration = CreateConfiguration(data);
    PrintConfiguration(configuration);

    // Create tracker
    CUVSLAM_TrackerHandle tracker_handle;
    UTimer create_timer; create_timer.start();
    const CUVSLAM_Status status_tracker = CUVSLAM_CreateTracker(&tracker_handle, &camera_rig, &configuration);
    
    if (status_tracker != CUVSLAM_SUCCESS) {
        UERROR("Failed to initialize CUVSLAM tracker: %d", status_tracker);
        return false;
    }

    cuvslam_handle = tracker_handle;

    // Initialize gpu image data vectors and sizes
    size_t stereo_pairs_count = data.stereoCameraModels().size();
    gpu_left_image_data.resize(stereo_pairs_count, nullptr);
    gpu_right_image_data.resize(stereo_pairs_count, nullptr);
    gpu_left_image_sizes.resize(stereo_pairs_count, 0);
    gpu_right_image_sizes.resize(stereo_pairs_count, 0);

    // initialize ground constraints
    if (planar_constraints) 
    {
        CUVSLAM_Pose identity_cuvslam = TocuVSLAMPose(Transform::getIdentity()); // same in both frames
    
        const CUVSLAM_Status status_ground = CUVSLAM_GroundConstraintCreate(
            &ground_constraint_handle,
            &identity_cuvslam,
            &identity_cuvslam,
            &identity_cuvslam
        );
        if(status_ground != CUVSLAM_SUCCESS) {
            UERROR("Failed to initialize CUVSLAM ground constraint: %d", status_ground);
            return false;
        }
    }

    return true;
}

/*
Implementation based on Isaac ROS VisualSlamNode::VisualSlamImpl::CreateConfiguration()
Source: isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp:379-422
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/19be8c781a55dee9cfbe9f097adca3986638feb1/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp#L379-L422    
*/
CUVSLAM_Configuration CreateConfiguration(const SensorData & data)
{
    CUVSLAM_Configuration configuration;
    CUVSLAM_InitDefaultConfiguration(&configuration);
    
    configuration.multicam_mode = data.stereoCameraModels().size()>1?1:0;
    
    // Core Visual Odometry Settings 
    configuration.use_motion_model = 1;                 // Enable motion model for better tracking
    configuration.use_denoising = 0;                    // Disable denoising by default
    configuration.use_gpu = 1;                          // Use GPU acceleration
    configuration.horizontal_stereo_camera = 1;         // Stereo camera configuration

    configuration.enable_observations_export = 1;       // Export observations for external reading
    
    // SLAM Enabled (required for observation buffer allocation) 
    configuration.enable_localization_n_mapping = 0;    // NO SLAM
    configuration.enable_landmarks_export = 0;          // SLAM feature (optional)
    configuration.enable_reading_slam_internals = 0;    // SLAM feature (optional)
    
    // IMU Configuration (If we later implement IMU support)
    configuration.enable_imu_fusion = 0;                //data.imu().empty()?0:1;
    configuration.debug_imu_mode = 0;                   // Disable IMU debug mode
    // imu_calibration.gyroscope_noise_density = 0.0002f;    
    // imu_calibration.gyroscope_random_walk = 0.00003f;      
    // imu_calibration.accelerometer_noise_density = 0.01f;   
    // imu_calibration.accelerometer_random_walk = 0.001f;    
    // imu_calibration.frequency = 200.0f; 
    // configuration.imu_calibration = imu_calibration;
    
    // configuration.max_frame_delta_ms = 100.0;        // Maximum frame interval (100ms default)
    
    // SLAM-specific parameters (disabled)
    configuration.planar_constraints = 0;
    configuration.slam_throttling_time_ms = 0;
    configuration.slam_max_map_size = 0;
    configuration.slam_sync_mode = 0;

    // Use for getting debug images and logs
    // configuration.debug_dump_directory = "/home/...your desired directory...";
    
    return configuration;
}

// ============================================================================
// GPU Memory Management
// ============================================================================

bool allocateGpuMemory(size_t size, uint8_t ** gpu_ptr, size_t * current_size)
{
    if(*current_size != size) {
        // Reallocate GPU memory if size changed
        if(*gpu_ptr != nullptr) {
            cudaFree(*gpu_ptr);
        }
        *gpu_ptr = nullptr;
        cudaError_t cuda_err = cudaMalloc(gpu_ptr, size);
        if(cuda_err != cudaSuccess) {
            UERROR("Failed to allocate GPU memory: %s", cudaGetErrorString(cuda_err));
            return false;
        }
        *current_size = size;
    }
    return true;
}

bool copyToGpuAsync(const cv::Mat & cpu_image, uint8_t * gpu_ptr, size_t size, cudaStream_t & cuda_stream)
{
    // Initialize CUDA stream if not already done
    if(cuda_stream == nullptr) {
        cudaError_t stream_err = cudaStreamCreate(&cuda_stream);
        if(stream_err != cudaSuccess) {
            UERROR("Failed to create CUDA stream: %s", cudaGetErrorString(stream_err));
            return false;
        }
    }
    
    // Copy CPU data to GPU memory with async operation for better performance
    cudaError_t cuda_err = cudaMemcpyAsync(gpu_ptr, cpu_image.data, size, 
                                          cudaMemcpyHostToDevice, cuda_stream);
    if(cuda_err != cudaSuccess) {
        UERROR("Failed to copy image to GPU: %s", cudaGetErrorString(cuda_err));
        return false;
    }
    
    return true;
}

bool synchronizeGpuOperations(cudaStream_t & cuda_stream)
{
    if(cuda_stream) {
        cudaError_t cuda_err = cudaStreamSynchronize(cuda_stream);
        if(cuda_err != cudaSuccess) {
            UERROR("Failed to synchronize GPU operations: %s", cudaGetErrorString(cuda_err));
            return false;
        }
    }
    return true;
}

// ============================================================================
// Image Processing and Preparation
// ============================================================================

bool prepareImages(const SensorData & data, 
                   std::vector<CUVSLAM_Image> & cuvslam_images,
                   std::vector<uint8_t *> & gpu_left_image_data,
                   std::vector<uint8_t *> & gpu_right_image_data,
                   std::vector<size_t> & gpu_left_image_sizes,
                   std::vector<size_t> & gpu_right_image_sizes,
                   cudaStream_t & cuda_stream)
{
    // Convert timestamp to nanoseconds (cuVSLAM expects nanoseconds)
    int64_t timestamp_ns = static_cast<int64_t>(data.stamp() * 1000000000.0);

    // Horizontally stitched images received by RTAB-Map
    cv::Mat left_image = data.imageRaw();
    cv::Mat right_image = data.rightRaw();

    // Validate basic image properties
    if(left_image.empty() || right_image.empty()) {
        UERROR("No left or right image available for stereo camera");
        return false;
    }
    if(left_image.channels() != 1 && left_image.channels() != 3) {
        UERROR("Unsupported left image format: %d channels", left_image.channels());
        return false;
    }
    if(right_image.channels() != 1 && right_image.channels() != 3) {
        UERROR("Unsupported right image format: %d channels", right_image.channels());
        return false;
    }

    // Convert image format for cuVSLAM - mono8 or rgb8
    cv::Mat processed_left_image;
    cv::Mat processed_right_image;
    CUVSLAM_ImageEncoding left_encoding;
    CUVSLAM_ImageEncoding right_encoding;
    
    // process left image - copies image if BGR to RGB conversion is needed
    processed_left_image = left_image;
    if(left_image.channels() == 1) {
        left_encoding = CUVSLAM_ImageEncoding::MONO8;
    } else if(left_image.channels() == 3) {
        // convert from BGR to RGB
        cv::cvtColor(left_image, processed_left_image, cv::COLOR_BGR2RGB);
        left_encoding = CUVSLAM_ImageEncoding::RGB8;
    } else {
        UERROR("Unsupported left image format: %d channels", left_image.channels());
        return false;
    }

    // process right image - copies image if BGR to RGB conversion is needed
    processed_right_image = right_image;
    if(right_image.channels() == 1) {
        right_encoding = CUVSLAM_ImageEncoding::MONO8;
    } else if(right_image.channels() == 3) {
        // convert from BGR to RGB
        cv::cvtColor(right_image, processed_right_image, cv::COLOR_BGR2RGB);
        right_encoding = CUVSLAM_ImageEncoding::RGB8;
    } else {
        UERROR("Unsupported right image format: %d channels", right_image.channels());
        return false;
    }
       
    int camera_index = 0;
    int stereo_index = 0;
    for(const StereoCameraModel & model : data.stereoCameraModels()) {
        // slice out the image for the current stereo pair
        // Assumes all images have the same width and height
        int left_image_width = model.left().imageWidth();
        int right_image_width = model.right().imageWidth();
        int left_image_height = model.left().imageHeight();
        int right_image_height = model.right().imageHeight();

        // makes a copy for the sliced images
        cv::Mat left_image_slice = processed_left_image(cv::Rect(stereo_index * left_image_width, 0, left_image_width, left_image_height)).clone();
        cv::Mat right_image_slice = processed_right_image(cv::Rect(stereo_index * right_image_width, 0, right_image_width, right_image_height)).clone();
        
        size_t left_image_size = left_image_slice.total() * left_image_slice.elemSize();
        size_t right_image_size = right_image_slice.total() * right_image_slice.elemSize();

        // Allocate GPU memory for left camera
        if(!allocateGpuMemory(left_image_size, &gpu_left_image_data[stereo_index], &gpu_left_image_sizes[stereo_index])) {
            UERROR("PREPARE IMAGES: Failed to allocate GPU memory for left image");
            return false;
        }
        if(!copyToGpuAsync(left_image_slice, gpu_left_image_data[stereo_index], left_image_size, cuda_stream)) {
            UERROR("PREPARE IMAGES: Failed to copy left image to GPU");
            return false;
        }

        // Create CUVSLAM_Image for left camera with GPU memory
        CUVSLAM_Image left_cuvslam_image;
        left_cuvslam_image.width = left_image_width;
        left_cuvslam_image.height = left_image_height;
        left_cuvslam_image.pixels = gpu_left_image_data[stereo_index];  // GPU memory pointer
        left_cuvslam_image.timestamp_ns = timestamp_ns;
        left_cuvslam_image.camera_index = camera_index;
        left_cuvslam_image.pitch = left_image_slice.step;
        left_cuvslam_image.image_encoding = left_encoding;
        
        cuvslam_images.push_back(left_cuvslam_image);

        camera_index++;

        // Allocate GPU memory for right camera
        if(!allocateGpuMemory(right_image_size, &gpu_right_image_data[stereo_index], &gpu_right_image_sizes[stereo_index])) {
            UERROR("PREPARE IMAGES: Failed to allocate GPU memory for right image");
            return false;
        }
        
        if(!copyToGpuAsync(right_image_slice, gpu_right_image_data[stereo_index], right_image_size, cuda_stream)) {
            UERROR("PREPARE IMAGES: Failed to copy right image to GPU");
            return false;
        }

        CUVSLAM_Image right_cuvslam_image;
        right_cuvslam_image.width = right_image_width;
        right_cuvslam_image.height = right_image_height;
        right_cuvslam_image.pixels = gpu_right_image_data[stereo_index];  // GPU memory pointer
        right_cuvslam_image.timestamp_ns = timestamp_ns;
        right_cuvslam_image.camera_index = camera_index;
        right_cuvslam_image.pitch = right_image_slice.step;
        right_cuvslam_image.image_encoding = right_encoding;

        cuvslam_images.push_back(right_cuvslam_image);
        
        stereo_index++;
        camera_index++;
    }

    // Synchronize all async GPU operations before returning
    if(!synchronizeGpuOperations(cuda_stream)) {
        UERROR("PREPARE IMAGES: Failed to synchronize GPU operations");
        return false;
    }

    return true;
} 


/*
Convert cuVSLAM covariance to RTAB-Map format.
Based on Isaac ROS implementation: FromcuVSLAMCovariance()
Source: isaac_ros_visual_slam/src/impl/cuvslam_ros_conversion.cpp:275-299
*/
cv::Mat convertCuVSLAMCovariance(const float * cuvslam_covariance)
{
    // Scale cuvslam covariance to make it more realistic
    const double scaling_factor = 10.0;

    // Handle null covariance pointer
    if(cuvslam_covariance == nullptr)
    {
        UWARN("Covariance was recieved as a nullptr, proceeding with default infinite covariance");
        cv::Mat default_infinite_covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
        return default_infinite_covariance;
    }
    
    const float * covariance = cuvslam_covariance;
    
    // Create transformation matrix for coordinate system conversion
    // cuVSLAM frame (x-right, y-up, z-backward) to RTAB-Map frame (x-forward, y-left, z-up)
    // Get rotation matrix from canonical_pose_cuvslam transform
    Eigen::Matrix<float, 3, 3> canonical_pose_cuvslam_mat;
    canonical_pose_cuvslam_mat << 
        canonical_pose_cuvslam.r11(), canonical_pose_cuvslam.r12(), canonical_pose_cuvslam.r13(),
        canonical_pose_cuvslam.r21(), canonical_pose_cuvslam.r22(), canonical_pose_cuvslam.r23(),
        canonical_pose_cuvslam.r31(), canonical_pose_cuvslam.r32(), canonical_pose_cuvslam.r33();
    
    // Create 6x6 block diagonal transformation matrix
    Eigen::Matrix<float, 6, 6> block_canonical_pose_cuvslam = Eigen::Matrix<float, 6, 6>::Zero();
    block_canonical_pose_cuvslam.block<3, 3>(0, 0) = canonical_pose_cuvslam_mat;
    block_canonical_pose_cuvslam.block<3, 3>(3, 3) = canonical_pose_cuvslam_mat;
    
    // Map cuVSLAM covariance array to Eigen matrix
    Eigen::Matrix<float, 6, 6> covariance_mat = 
        Eigen::Map<Eigen::Matrix<float, 6, 6, Eigen::StorageOptions::AutoAlign>>(const_cast<float*>(covariance));
    
    // Reorder covariance matrix elements
    // The covariance matrix from cuVSLAM arranges elements as follows:
    // (rotation about X axis, rotation about Y axis, rotation about Z axis, x, y, z)
    // However, in RTAB-Map, the order is:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    Eigen::Matrix<float, 6, 6> rtabmap_covariance_mat = Eigen::Matrix<float, 6, 6>::Zero();
    rtabmap_covariance_mat.block<3, 3>(0, 0) = covariance_mat.block<3, 3>(3, 3);  // translation-translation
    rtabmap_covariance_mat.block<3, 3>(0, 3) = covariance_mat.block<3, 3>(3, 0);  // translation-rotation
    rtabmap_covariance_mat.block<3, 3>(3, 0) = covariance_mat.block<3, 3>(0, 3);  // rotation-translation
    rtabmap_covariance_mat.block<3, 3>(3, 3) = covariance_mat.block<3, 3>(0, 0);  // rotation-rotation
    
    // Apply coordinate system transformation
    Eigen::Matrix<float, 6, 6> covariance_mat_change_basis = 
        block_canonical_pose_cuvslam * rtabmap_covariance_mat * block_canonical_pose_cuvslam.transpose();
    
    // Convert Eigen matrix to OpenCV Mat
    cv::Mat cv_covariance(6, 6, CV_64FC1);
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            cv_covariance.at<double>(i, j) = static_cast<double>(covariance_mat_change_basis(i, j));
            // for angular values, scale again to make it more realistic
            if(i > 2 || j > 2) {
                cv_covariance.at<double>(i, j) *= scaling_factor;
            }
        }
    }
    
    // Ensure diagonal elements are positive and finite (RTAB-Map requirement)
    return cv_covariance;
}

#endif // RTABMAP_CUVSLAM

} // namespace rtabmap

