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

#include "rtabmap/core/odometry/OdometryCuVSLAM.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/StereoCameraModel.h"

#ifdef RTABMAP_CUVSLAM
#include <cuvslam.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <eigen3/Eigen/Dense>

// ============================================================================
// cuVSLAM Constants and Enums
// ============================================================================

// cuVSLAM Image Encoding enum is already defined in cuvslam.h

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
const tf2::Transform cuvslam_pose_canonical(tf2::Matrix3x3(
    0, -1, 0,
    0, 0, 1,
    -1, 0, 0
));

// Transformation converting from
// cuVSLAM Frame       (x-right, y-up, z-backward) to
// Canonical ROS Frame (x-forward, y-left, z-up)
const tf2::Transform canonical_pose_cuvslam(cuvslam_pose_canonical.inverse());

// Transformation converting from
// Optical Frame    (x-right, y-down, z-forward) to
// cuVSLAM Frame    (x-right, y-up, z-backward)
// Optical   ->  cuVSLAM
//    x      ->     x
//    y      ->    -y
//    z      ->    -z
const tf2::Transform cuvslam_pose_optical(tf2::Matrix3x3(
    1, 0, 0,
    0, -1, 0,
    0, 0, -1
));

// Transformation converting from
// cuVSLAM Frame    (x-right, y-up, z-backward) to
// Optical Frame    (x-right, y-down, z-forward)
const tf2::Transform optical_pose_cuvslam(cuvslam_pose_optical.inverse());
#endif

namespace rtabmap {

// ============================================================================
// OdometryCuVSLAM Class Implementation
// ============================================================================

OdometryCuVSLAM::OdometryCuVSLAM(const ParametersMap & parameters) :
    Odometry(parameters),
#ifdef RTABMAP_CUVSLAM
    cuvslam_handle_(nullptr),
    camera_rig_(nullptr),
    configuration_(nullptr),
    initialized_(false),
    lost_(false),
    previous_pose_(Transform::getIdentity()),
    last_timestamp_(-1.0)
#endif
{
    UINFO("OdometryCuVSLAM created");
}

OdometryCuVSLAM::~OdometryCuVSLAM()
{
#ifdef RTABMAP_CUVSLAM
    // Clean up cuVSLAM tracker
    if(cuvslam_handle_)
    {
        CUVSLAM_DestroyTracker(static_cast<CUVSLAM_TrackerHandle>(cuvslam_handle_));
        cuvslam_handle_ = nullptr;
    }
    
    // Clean up camera rig and configuration
    if(camera_rig_) {
        delete static_cast<CUVSLAM_CameraRig*>(camera_rig_);
        camera_rig_ = nullptr;
    }
    if(configuration_) {
        delete static_cast<CUVSLAM_Configuration*>(configuration_);
        configuration_ = nullptr;
    }
    
    // Clean up camera objects
    for(void* camera : cuvslam_cameras_) {
        delete static_cast<CUVSLAM_Camera*>(camera);
    }
    cuvslam_cameras_.clear();
#endif
}

void OdometryCuVSLAM::reset(const Transform & initialPose)
{
    Odometry::reset(initialPose);
    
#ifdef RTABMAP_CUVSLAM
    // Destroy existing tracker first (before cleaning up parameters it might be using)
    if(cuvslam_handle_)
    {
        CUVSLAM_DestroyTracker(static_cast<CUVSLAM_TrackerHandle>(cuvslam_handle_));
        cuvslam_handle_ = nullptr;
    }
        
    // Reset member variables to initial state
    // Clean up camera rig and configuration
    if(camera_rig_) {
        delete static_cast<CUVSLAM_CameraRig*>(camera_rig_);
        camera_rig_ = nullptr;
    }
    if(configuration_) {
        delete static_cast<CUVSLAM_Configuration*>(configuration_);
        configuration_ = nullptr;
    }
    // Clean up camera objects
    for(void* camera : cuvslam_cameras_) {
        delete static_cast<CUVSLAM_Camera*>(camera);
    }
    cuvslam_cameras_.clear();
    initialized_ = false;
    lost_ = false;
    previous_pose_ = initialPose;
    last_timestamp_ = -1.0;
#endif
}

Transform OdometryCuVSLAM::computeTransform(
    SensorData & data,
    const Transform & guess,
    OdometryInfo * info)
{
    Transform transform;
    
#ifdef RTABMAP_CUVSLAM
    UTimer timer;
    
    // Check if we have valid image data
    if(data.imageRaw().empty() || data.rightRaw().empty())
    {
        UERROR("cuVSLAM odometry requires both left and right images! Left: %s, Right: %s", 
               data.imageRaw().empty() ? "empty" : "ok", 
               data.rightRaw().empty() ? "empty" : "ok");
        return transform;
    }
    
    // Check if we have valid stereo camera models
    if(data.stereoCameraModels().size() == 0)
    {
        UERROR("cuVSLAM odometry requires stereo camera models!");
        return transform;
    }
    
    // Initialize cuVSLAM tracker on first frame
    if(!initialized_)
    {
        UINFO("Initializing cuVSLAM tracker with first image (stamp: %f)", data.stamp());
        if(!initializeCuVSLAM(data))
        {
            UERROR("Failed to initialize cuVSLAM tracker");
            return transform;
        }
        initialized_ = true;
        
        // For first frame, return identity transform with high covariance
        transform = Transform::getIdentity();
        if(info)
        {
            info->type = 0; // Initialization
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0; // High uncertainty
            info->timeEstimation = timer.ticks();
        }
        
        previous_pose_ = transform;
        last_timestamp_ = data.stamp();
        
        UDEBUG("cuVSLAM initialization completed in %f ms", timer.ticks());
        return transform;
    }
    
    // Prepare images for cuVSLAM
    std::vector<void*> cuvslam_images;
    if(!this->prepareImages(data, cuvslam_images))
    {
        UERROR("Failed to prepare images for cuVSLAM");
        return transform;
    }
    
    // Process IMU data if available
    if(!data.imu().empty())
    {
        // TODO: Implement IMU processing
        // For now, we'll skip IMU processing as it's not implemented yet
        UDEBUG("IMU data available but processing not implemented yet");
    }

    // Call cuVSLAM tracking
    CUVSLAM_PoseEstimate vo_pose_estimate;
    const CUVSLAM_Status vo_status = CUVSLAM_TrackGpuMem(
        static_cast<CUVSLAM_TrackerHandle>(cuvslam_handle_), 
        reinterpret_cast<const CUVSLAM_Image*>(cuvslam_images.data()), 
        cuvslam_images.size(), 
        nullptr, 
        &vo_pose_estimate);
    
    // Clean up CUVSLAM_Image objects
    for(void* image : cuvslam_images) {
        delete static_cast<CUVSLAM_Image*>(image);
    }
    cuvslam_images.clear();
    
    // Convert cuVSLAM covariance to RTAB-Map format
    
    // Handle tracking status and odom info
    if(vo_status == CUVSLAM_TRACKING_LOST)
    {
        UDEBUG("cuVSLAM tracking lost");
        lost_ = true;
        if(info)
        {
            info->type = 1; // Tracking lost
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0; // High uncertainty
            info->timeEstimation = timer.ticks();
        }
        return transform;
    }
    else if(vo_status != CUVSLAM_SUCCESS)
    {
        UERROR("cuVSLAM tracking error: %d", vo_status);
        if(info)
        {
            info->type = 1; // Error
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0; // High uncertainty
            info->timeEstimation = timer.ticks();
        }
        return transform;
    }
    else
    {
        // TODO: Get covariance from cuVSLAM
        UDEBUG("cuVSLAM tracking success");
        if(info)
        {
            cv::Mat covMat = this->convertCuVSLAMCovariance(vo_pose_estimate.covariance);

            info->type = 0;  // Success
            info->reg.covariance = covMat;
            info->timeEstimation = timer.ticks();
        }
    }
    
    // Convert cuVSLAM pose to RTAB-Map Transform
    Transform current_pose = this->convertCuVSLAMPose(&vo_pose_estimate.pose);
    
    // Calculate incremental transform
    if(!previous_pose_.isNull())
    {
        transform = previous_pose_.inverse() * current_pose;
    }
    else
    {
        transform = current_pose;
    }
    
    // Update tracking state
    lost_ = false;
    previous_pose_ = current_pose;
    last_timestamp_ = data.stamp();
    
    UDEBUG("cuVSLAM odometry computed in %f ms", timer.ticks());
    
#else
    UERROR("cuVSLAM support not compiled in RTAB-Map");
#endif
    
    return transform;
}

#ifdef RTABMAP_CUVSLAM

// ============================================================================
// cuVSLAM Initialization and Configuration
// ============================================================================

bool OdometryCuVSLAM::initializeCuVSLAM(const SensorData & data)
{
    cuvslam_cameras_.clear();
        
    // cuVSLAM only supports stereo cameras
    if(data.stereoCameraModels().size() == 0)
    {
        UERROR("cuVSLAM odometry requires stereo camera models! Monocular cameras are not supported.");
        return false;
    }
    
    // Validate stereo camera models
    for(size_t i = 0; i < data.stereoCameraModels().size(); ++i)
    {
        if(!data.stereoCameraModels()[i].isValidForProjection())
        {
            UERROR("Invalid stereo camera model %d for cuVSLAM initialization!", static_cast<int>(i));
            return false;
        }
    }
    
    // Handle stereo cameras - process all stereo pairs
    for(size_t i = 0; i < data.stereoCameraModels().size(); ++i)
    {
        const StereoCameraModel & stereoModel = data.stereoCameraModels()[i];
        const CameraModel & leftModel = stereoModel.left();
        const CameraModel & rightModel = stereoModel.right();
        
        CUVSLAM_Camera left_camera;
        left_camera.width = leftModel.imageWidth();
        left_camera.height = leftModel.imageHeight();
        
        // Allocate parameters array for pinhole model (rectified images) - stack allocation
        float left_params[4] = {
            static_cast<float>(leftModel.cx()), 
            static_cast<float>(leftModel.cy()), 
            static_cast<float>(leftModel.fx()),
            static_cast<float>(leftModel.fy())
        };
        left_camera.parameters = left_params;
        
        // Set pinhole model with zero distortion (rectified images)
        left_camera.distortion_model = "pinhole";
        left_camera.num_parameters = 4;
        
        // Set camera pose (extrinsics) from localTransform with proper coordinate system conversion
        Transform left_pose = leftModel.localTransform();
        CUVSLAM_Pose * cuvslam_pose = static_cast<CUVSLAM_Pose*>(this->rtabmapTransformToCuVSLAMPose(left_pose));
        left_camera.pose = *cuvslam_pose;
        delete cuvslam_pose;
    
        cuvslam_cameras_.push_back(new CUVSLAM_Camera(left_camera));
        
        CUVSLAM_Camera right_camera;
        right_camera.width = rightModel.imageWidth();
        right_camera.height = rightModel.imageHeight();
        
        // Allocate parameters array for pinhole model (rectified images) - stack allocation
        float right_params[4] = {
            static_cast<float>(rightModel.cx()), 
            static_cast<float>(rightModel.cy()), 
            static_cast<float>(rightModel.fx()), 
            static_cast<float>(rightModel.fy())
        };
        right_camera.parameters = right_params;
        
        // Set pinhole model with zero distortion (rectified images)
        right_camera.distortion_model = "pinhole";
        right_camera.num_parameters = 4;
        
        // Set right camera pose relative to left with proper coordinate system conversion
        Transform right_pose = rightModel.localTransform();
        CUVSLAM_Pose * right_cuvslam_pose = static_cast<CUVSLAM_Pose*>(this->rtabmapTransformToCuVSLAMPose(right_pose));
        right_camera.pose = *right_cuvslam_pose;
        delete right_cuvslam_pose;
        
        cuvslam_cameras_.push_back(new CUVSLAM_Camera(right_camera));
    }
    
    // Set up camera rig
    CUVSLAM_CameraRig* camera_rig = new CUVSLAM_CameraRig();
    camera_rig->cameras = reinterpret_cast<CUVSLAM_Camera*>(cuvslam_cameras_.data());
    camera_rig->num_cameras = cuvslam_cameras_.size();
    camera_rig_ = camera_rig;
    
    // Create configuration using Isaac ROS pattern
    CUVSLAM_Pose cuvslam_imu_pose;
    // if (!data.imu().empty()) {
    //     // Get IMU pose relative to camera (base frame)
    //     // data.imu().localTransform() gives us camera_pose_imu (camera in IMU frame)
    //     // We need imu_pose_camera (IMU in camera frame) for cuVSLAM
    //     Transform imu_pose_camera = data.imu().localTransform().inverse();
    //     cuvslam_imu_pose = convertCameraPoseToCuVSLAM(imu_pose_camera);
    // } else {
    //     // Set identity pose if no IMU
    //     cuvslam_imu_pose = convertCameraPoseToCuVSLAM(Transform::getIdentity());
    // }
    
    // No IMU for now, use identity pose
    CUVSLAM_Pose * imu_pose = static_cast<CUVSLAM_Pose*>(this->rtabmapTransformToCuVSLAMPose(Transform::getIdentity()));
    cuvslam_imu_pose = *imu_pose;
    delete imu_pose;
    
    configuration_ = this->CreateConfiguration(&cuvslam_imu_pose);
    
    // Enable IMU fusion if we have IMU data
    if (!data.imu().empty()) {
        static_cast<CUVSLAM_Configuration*>(configuration_)->enable_imu_fusion = 1;
    }
    
    // Create tracker
    CUVSLAM_TrackerHandle tracker_handle;
    CUVSLAM_Status status = CUVSLAM_CreateTracker(&tracker_handle, static_cast<CUVSLAM_CameraRig*>(camera_rig_), static_cast<CUVSLAM_Configuration*>(configuration_));
    cuvslam_handle_ = tracker_handle;
    if(status != CUVSLAM_SUCCESS)
    {
        UERROR("Failed to create cuVSLAM tracker: %d", status);
        return false;
    }
    
    UINFO("cuVSLAM tracker initialized with %d cameras", cuvslam_cameras_.size());
    return true;
}

/*
Implementation based on Isaac ROS VisualSlamNode::VisualSlamImpl::CreateConfiguration()
Source: isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp:379-422
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/19be8c781a55dee9cfbe9f097adca3986638feb1/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp#L379-L422    
*/
void * OdometryCuVSLAM::CreateConfiguration(const void * cv_base_link_pose_cv_imu)
{
    // Note: cv_base_link_pose_cv_imu parameter is not used in current implementation
    // but kept for future IMU integration
    (void)cv_base_link_pose_cv_imu; // Suppress unused parameter warning
    CUVSLAM_Configuration * configuration = new CUVSLAM_Configuration();
    CUVSLAM_InitDefaultConfiguration(configuration);
    
    // Core Visual Odometry Settings 
    configuration->multicam_mode = 0;                    // Single camera setup
    configuration->use_motion_model = 1;                 // Enable motion model for better tracking
    configuration->use_denoising = 0;                    // Disable denoising by default
    configuration->use_gpu = 1;                          // Use GPU acceleration
    configuration->horizontal_stereo_camera = 1;         // Stereo camera configuration
    
    // SLAM Disabled (VO-only mode) 
    configuration->enable_localization_n_mapping = 0;    // Disable SLAM - VO only
    configuration->enable_observations_export = 0;       // Not needed for VO
    configuration->enable_landmarks_export = 0;          // Not needed for VO
    configuration->enable_reading_slam_internals = 0;    // Not needed for VO
    
    // IMU Configuration 
    configuration->enable_imu_fusion = 0;                // No IMU for now
    configuration->debug_imu_mode = 0;                   // Disable IMU debug mode
    // imu_calibration.gyroscope_noise_density = 0.0002f;    
    // imu_calibration.gyroscope_random_walk = 0.00003f;      
    // imu_calibration.accelerometer_noise_density = 0.01f;   
    // imu_calibration.accelerometer_random_walk = 0.001f;    
    // imu_calibration.frequency = 200.0f; 
    // configuration.imu_calibration = imu_calibration;
    
    // Timing and Performance 
    configuration->max_frame_delta_ms = 100;             // Maximum frame interval (100ms default)
    
    // SLAM-specific parameters (ignored when enable_localization_n_mapping = 0) 
    // These are set to 0 but won't be used since SLAM is disabled
    configuration->planar_constraints = 0;               // No planar constraints
    configuration->slam_throttling_time_ms = 0;          // No SLAM throttling
    configuration->slam_max_map_size = 0;                // No SLAM map size limit
    
    return static_cast<void*>(configuration);
}

// ============================================================================
// Image Processing and Preparation
// ============================================================================

bool OdometryCuVSLAM::prepareImages(const SensorData & data, std::vector<void*> & cuvslam_images)
{
    cuvslam_images.clear();
    
    // Convert timestamp to nanoseconds (cuVSLAM expects nanoseconds)
    int64_t timestamp_ns = static_cast<int64_t>(data.stamp() * 1000000000.0);
    
    // cuVSLAM only supports stereo cameras
    if(data.stereoCameraModels().size() == 0)
    {
        UERROR("cuVSLAM odometry requires stereo camera models! Monocular cameras are not supported.");
        return false;
    }
    
    // Process all stereo pairs, assume one camera pair
    // TODO: H
    if(!data.imageRaw().empty())
    {
        cv::Mat left_image = data.imageRaw();
        
        // Validate image format
        if(left_image.channels() != 1 && left_image.channels() != 3)
        {
            UERROR("Unsupported left image format: %d channels", left_image.channels());
            return false;
        }
        
        // Create CUVSLAM_Image for left camera
        CUVSLAM_Image* left_cuvslam_image = new CUVSLAM_Image();
        left_cuvslam_image->width = left_image.cols;
        left_cuvslam_image->height = left_image.rows;
        left_cuvslam_image->pixels = left_image.data;
        left_cuvslam_image->timestamp_ns = timestamp_ns;
        left_cuvslam_image->camera_index = 0;
        left_cuvslam_image->pitch = left_image.step;
        left_cuvslam_image->image_encoding = (left_image.channels() == 1) ? 
            CUVSLAM_ImageEncoding::MONO8 : CUVSLAM_ImageEncoding::RGB8;
        
        cuvslam_images.push_back(static_cast<void*>(left_cuvslam_image));
    }
    else
    {
        UERROR("No left image available for stereo camera");
        return false;
    }
    
    // Process right image (first stereo pair only)
    if(!data.rightRaw().empty())
    {
        cv::Mat right_image = data.rightRaw();
        
        // Validate image format
        if(right_image.channels() != 1 && right_image.channels() != 3)
        {
            UERROR("Unsupported right image format: %d channels", right_image.channels());
            return false;
        }
        
        // Create CUVSLAM_Image for right camera
        CUVSLAM_Image* right_cuvslam_image = new CUVSLAM_Image();
        right_cuvslam_image->width = right_image.cols;
        right_cuvslam_image->height = right_image.rows;
        right_cuvslam_image->pixels = right_image.data;
        right_cuvslam_image->timestamp_ns = timestamp_ns;
        right_cuvslam_image->camera_index = 1;  // Right camera index
        right_cuvslam_image->pitch = right_image.step;
        right_cuvslam_image->image_encoding = (right_image.channels() == 1) ? 
            CUVSLAM_ImageEncoding::MONO8 : CUVSLAM_ImageEncoding::RGB8;
        
        cuvslam_images.push_back(static_cast<void*>(right_cuvslam_image));
    } 
    else 
    {
        UERROR("No right image available for stereo camera");
        return false;
    }
    
    UDEBUG("Prepared %d images for cuVSLAM (timestamp: %ld ns)", 
           static_cast<int>(cuvslam_images.size()), timestamp_ns);
    
    return true;
}

// ============================================================================
// Coordinate System and Transform Conversion Functions
// ============================================================================

// Convert RTAB-Map Transform to tf2::Transform (same coordinate system!)
tf2::Transform rtabmapToTf2(const Transform & rtabmap_transform) {
    // https://docs.ros.org/en/jade/api/rtabmap/html/classrtabmap_1_1Transform.html
    tf2::Matrix3x3 rotation(
        rtabmap_transform.r11(), rtabmap_transform.r12(), rtabmap_transform.r13(),
        rtabmap_transform.r21(), rtabmap_transform.r22(), rtabmap_transform.r23(),
        rtabmap_transform.r31(), rtabmap_transform.r32(), rtabmap_transform.r33()
    );
    
    tf2::Vector3 translation(
        rtabmap_transform.x(),
        rtabmap_transform.y(), 
        rtabmap_transform.z()
    );
    
    return tf2::Transform(rotation, translation);
}

// Helper function that converts transform into CUVSLAM_Pose
// Taken directly from isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/cuvslam_ros_conversion.cpp
CUVSLAM_Pose TocuVSLAMPose(const tf2::Transform & tf_mat)
{
  CUVSLAM_Pose cuvslamPose;
  // tf2::Matrix3x3 is row major, but cuvslam is column major
  const tf2::Matrix3x3 rotation(tf_mat.getRotation());
  const int32_t kRotationMatCol = 3;
  const int32_t kRotationMatRow = 3;
  int cuvslam_idx = 0;
  for (int col_idx = 0; col_idx < kRotationMatCol; ++col_idx) {
    const tf2::Vector3 & rot_col = rotation.getColumn(col_idx);
    for (int row_idx = 0; row_idx < kRotationMatRow; ++row_idx) {
      cuvslamPose.r[cuvslam_idx] = rot_col[row_idx];
      cuvslam_idx++;
    }
  }

  const tf2::Vector3 & translation = tf_mat.getOrigin();
  cuvslamPose.t[0] = translation.x();
  cuvslamPose.t[1] = translation.y();
  cuvslamPose.t[2] = translation.z();
  return cuvslamPose;
}

void * OdometryCuVSLAM::rtabmapTransformToCuVSLAMPose(const Transform & rtabmap_transform) {
    tf2::Transform tf2_transform = rtabmapToTf2(rtabmap_transform);
    CUVSLAM_Pose* pose = new CUVSLAM_Pose(TocuVSLAMPose(tf2_transform));
    return static_cast<void*>(pose);
}

// Helper function to convert camera pose from RTABMap to cuVSLAM coordinate system
// Based on Isaac ROS FillExtrinsics implementation
void * OdometryCuVSLAM::convertCameraPoseToCuVSLAM(const Transform & rtabmap_camera_pose) {
    // Convert RTABMap Transform to tf2::Transform
    tf2::Transform tf2_camera_pose = rtabmapToTf2(rtabmap_camera_pose);
    
    // Apply coordinate system transformation:
    // RTABMap robot frame -> cuVSLAM robot frame -> cuVSLAM camera frame
    // This follows the same pattern as Isaac ROS FillExtrinsics
    const tf2::Transform cuvslam_robot_pose_cuvslam_camera = 
        cuvslam_pose_canonical * tf2_camera_pose * optical_pose_cuvslam;
    
    CUVSLAM_Pose * pose = new CUVSLAM_Pose(TocuVSLAMPose(cuvslam_robot_pose_cuvslam_camera));
    return static_cast<void*>(pose);
}



/*
Convert cuVSLAM pose to RTAB-Map Transform.
CuVSLAM uses column-major rotation matrix, RTAB-Map uses row-major
*/
Transform OdometryCuVSLAM::convertCuVSLAMPose(const void * cuvslam_pose) 
{
    Transform rtabmap_transform;
    const CUVSLAM_Pose* pose = static_cast<const CUVSLAM_Pose*>(cuvslam_pose);
    
    // Set rotation matrix (convert from column-major to row-major)
    rtabmap_transform(0,0) = pose->r[0];  // r11
    rtabmap_transform(0,1) = pose->r[3];  // r12
    rtabmap_transform(0,2) = pose->r[6];  // r13
    rtabmap_transform(1,0) = pose->r[1];  // r21
    rtabmap_transform(1,1) = pose->r[4];  // r22
    rtabmap_transform(1,2) = pose->r[7];  // r23
    rtabmap_transform(2,0) = pose->r[2];  // r31
    rtabmap_transform(2,1) = pose->r[5];  // r32
    rtabmap_transform(2,2) = pose->r[8];  // r33
    
    // Set translation
    rtabmap_transform(0,3) = pose->t[0];  // tx
    rtabmap_transform(1,3) = pose->t[1];  // ty
    rtabmap_transform(2,3) = pose->t[2];  // tz
    
    return rtabmap_transform;
}

/*
Convert cuVSLAM covariance to RTAB-Map format.
Based on Isaac ROS implementation: FromcuVSLAMCovariance()
Source: isaac_ros_visual_slam/src/impl/cuvslam_ros_conversion.cpp:275-299
*/
cv::Mat OdometryCuVSLAM::convertCuVSLAMCovariance(const void * cuvslam_covariance)
{
    const float* covariance = static_cast<const float*>(cuvslam_covariance);
    
    // Create transformation matrix for coordinate system conversion
    // cuVSLAM frame (x-right, y-up, z-backward) to RTAB-Map frame (x-forward, y-left, z-up)
    tf2::Quaternion quat = canonical_pose_cuvslam.getRotation();
    
    // Convert tf2::Quaternion to Eigen for matrix operations
    Eigen::Quaternion<float> q(quat.w(), quat.x(), quat.y(), quat.z());
    Eigen::Matrix<float, 3, 3> canonical_pose_cuvslam_mat = q.matrix();
    
    // Create 6x6 block diagonal transformation matrix
    Eigen::Matrix<float, 6, 6> block_canonical_pose_cuvslam = Eigen::Matrix<float, 6, 6>::Zero();
    block_canonical_pose_cuvslam.block<3, 3>(0, 0) = canonical_pose_cuvslam_mat;
    block_canonical_pose_cuvslam.block<3, 3>(3, 3) = canonical_pose_cuvslam_mat;
    
    // Map cuVSLAM covariance array to Eigen matrix
    Eigen::Matrix<float, 6, 6> covariance_mat = 
        Eigen::Map<Eigen::Matrix<float, 6, 6, Eigen::StorageOptions::AutoAlign>>(const_cast<float*>(covariance));
    
    // Reorder covariance matrix elements
    // cuVSLAM order: (rotation about X, rotation about Y, rotation about Z, x, y, z)
    // RTAB-Map order: (x, y, z, rotation about X, rotation about Y, rotation about Z)
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
        }
    }
    
    return cv_covariance;
}


#endif // RTABMAP_CUVSLAM

} // namespace rtabmap
