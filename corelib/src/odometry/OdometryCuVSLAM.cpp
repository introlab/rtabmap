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
#include <cuvslam/cuvslam.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

// cuVSLAM Image Encoding enum (based on Isaac ROS implementation)
enum CUVSLAM_ImageEncoding {
    MONO8 = 0,
    RGB8 = 1
};
#endif

namespace rtabmap {

OdometryCuVSLAM::OdometryCuVSLAM(const ParametersMap & parameters) :
    Odometry(parameters),
#ifdef RTABMAP_CUVSLAM
    cuvslam_handle_(nullptr),
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
        CUVSLAM_DestroyTracker(cuvslam_handle_);
        cuvslam_handle_ = nullptr;
    }
    
    // Clean up parameter arrays from camera structures
    for(CUVSLAM_Camera& camera : cuvslam_cameras_)
    {
        if(camera.parameters)
        {
            delete[] const_cast<float*>(camera.parameters);
            camera.parameters = nullptr;
        }
    }
#endif
}

void OdometryCuVSLAM::reset(const Transform & initialPose)
{
    Odometry::reset(initialPose);
    
#ifdef RTABMAP_CUVSLAM
    // Clean up existing parameter arrays from camera structures
    for(CUVSLAM_Camera& camera : cuvslam_cameras_)
    {
        if(camera.parameters)
        {
            delete[] const_cast<float*>(camera.parameters);
            camera.parameters = nullptr;
        }
    }
    
    // Destroy existing tracker if it exists
    if(cuvslam_handle_)
    {
        CUVSLAM_DestroyTracker(cuvslam_handle_);
        cuvslam_handle_ = nullptr;
    }
    
    // Reset member variables to initial state
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
    

    CUVSLAM_PoseEstimate vo_pose_estimate;
    
    const CUVSLAM_Status vo_status =
        CUVSLAM_TrackGpuMem(
        cuvslam_handle_, cuvslam_images.data(), cuvslam_images.size(), nullptr,
        &vo_pose_estimate);

    // TODO: Check if we have valid image data
    // Hint: Check data.imageRaw() and data.image()
    
    // TODO: Initialize cuVSLAM tracker on first frame
    // Hint: Call initializeCuVSLAM() if not initialized
    
    // TODO: Prepare images for cuVSLAM
    // Hint: Call prepareImages() to convert SensorData to CUVSLAM_Image format
    
    // TODO: Process IMU data if available
    // Hint: Call processIMUData() if data.imu() is not empty
    
    // TODO: Call cuVSLAM tracking (this is the core VO step you highlighted!)
    // Hint: Use CUVSLAM_TrackGpuMem() with the prepared images
    // This is where the magic happens - cuVSLAM processes the images and returns a pose estimate
    
    // TODO: Handle tracking status
    // Hint: Check for CUVSLAM_TRACKING_LOST and CUVSLAM_SUCCESS
    
    // TODO: Convert cuVSLAM pose to RTAB-Map Transform
    // Hint: Call convertCuVSLAMPose() to convert the pose format
    
    // TODO: Update tracking state
    // - Set lost_ flag
    // - Update previous_pose_ and last_timestamp_
    
    // TODO: Fill OdometryInfo if provided
    // Hint: Set info->type, info->reg.covariance, info->timeEstimation
    
    UDEBUG("cuVSLAM odometry computed in %f ms", timer.ticks());
    
#else
    UERROR("cuVSLAM support not compiled in RTAB-Map");
#endif
    
    return transform;
}

#ifdef RTABMAP_CUVSLAM

bool OdometryCuVSLAM::initializeCuVSLAM(const SensorData & data)
{
    cuvslam_cameras_.clear();
    
    // Local vector to track parameter pointers for cleanup
    std::vector<float*> local_camera_parameters;
    
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
        
        // Allocate parameters array for pinhole model (rectified images)
        float * left_params = new float[4];
        left_camera.parameters = left_params;
        local_camera_parameters.push_back(left_params);  // Store for cleanup
        
        // Set pinhole model with zero distortion (rectified images)
        left_camera.distortion_model = CUVSLAM_DISTORTION_MODEL_PINHOLE;
        left_camera.num_parameters = 4;
        // Pinhole: [cx, cy, fx, fy]
        left_params[0] = leftModel.cx();
        left_params[1] = leftModel.cy();
        left_params[2] = leftModel.fx();
        left_params[3] = leftModel.fy();
        
        // Set camera pose (extrinsics) from localTransform
        Transform left_pose = leftModel.localTransform();
        CUVSLAM_Pose cuvslam_pose = rtabmapTransformToCuVSLAMPose(left_pose);
        left_camera.pose = cuvslam_pose;
        
        cuvslam_cameras_.push_back(left_camera);
        
        CUVSLAM_Camera right_camera;
        right_camera.width = rightModel.imageWidth();
        right_camera.height = rightModel.imageHeight();
        
        // Allocate parameters array for pinhole model (rectified images)
        float * right_params = new float[4];
        right_camera.parameters = right_params;
        local_camera_parameters.push_back(right_params);  // Store for cleanup
        
        // Set pinhole model with zero distortion (rectified images)
        right_camera.distortion_model = CUVSLAM_DISTORTION_MODEL_PINHOLE;
        right_camera.num_parameters = 4;
        // Pinhole: [cx, cy, fx, fy]
        right_params[0] = rightModel.cx();
        right_params[1] = rightModel.cy();
        right_params[2] = rightModel.fx();
        right_params[3] = rightModel.fy();
        
        // Set right camera pose relative to left
        Transform right_pose = rightModel.localTransform();
        CUVSLAM_Pose right_cuvslam_pose = rtabmapTransformToCuVSLAMPose(right_pose);
        right_camera.pose = right_cuvslam_pose;
        
        cuvslam_cameras_.push_back(right_camera);
    }
    
    // Set up camera rig
    camera_rig_.cameras = cuvslam_cameras_.data();
    camera_rig_.num_cameras = cuvslam_cameras_.size();
    
    // Create configuration using Isaac ROS pattern
    CUVSLAM_Pose cuvslam_imu_pose;
    if (!data.imu().empty()) {
        // Get IMU pose relative to camera (base frame)
        // data.imu().localTransform() gives us camera_pose_imu (camera in IMU frame)
        // We need imu_pose_camera (IMU in camera frame) for cuVSLAM
        Transform imu_pose_camera = data.imu().localTransform().inverse();
        cuvslam_imu_pose = rtabmapTransformToCuVSLAMPose(imu_pose_camera);
    } else {
        // Set identity pose if no IMU
        cuvslam_imu_pose = rtabmapTransformToCuVSLAMPose(Transform::getIdentity());
    }
    
    CUVSLAM_Configuration configuration_ = CreateConfiguration(cuvslam_imu_pose);
    
    // Enable IMU fusion if we have IMU data
    if (!data.imu().empty()) {
        configuration.enable_imu_fusion = 1;
    }
    
    // Create tracker
    CUVSLAM_Status status = CUVSLAM_CreateTracker(&cuvslam_handle_, &camera_rig_, &configuration_);
    if(status != CUVSLAM_SUCCESS)
    {
        UERROR("Failed to create cuVSLAM tracker: %d", status);
        // Clean up any allocated parameters before returning
        for(float* params : local_camera_parameters)
        {
            delete[] params;
        }
        return false;
    }
    
    UINFO("cuVSLAM tracker initialized with %d cameras", cuvslam_cameras_.size());
    return true;
}

bool OdometryCuVSLAM::prepareImages(const SensorData & data, std::vector<CUVSLAM_Image> & cuvslam_images)
{
    cuvslam_images.clear();
    
    // Convert timestamp to microseconds (cuVSLAM expects microseconds)
    int64_t timestamp_us = static_cast<int64_t>(data.stamp() * 1000000.0);
    
    // cuVSLAM only supports stereo cameras
    if(data.stereoCameraModels().size() == 0)
    {
        UERROR("cuVSLAM odometry requires stereo camera models! Monocular cameras are not supported.");
        return false;
    }
    
    // Process all stereo pairs, assume one camera pair
    for(size_t i = 0; i < data.stereoCameraModels().size(); ++i)
    {
        // Only the first stereo pair has actual images
        if(i == 0)
        {
            // Process left image (first stereo pair only)
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
                CUVSLAM_Image left_cuvslam_image;
                left_cuvslam_image.width = left_image.cols;
                left_cuvslam_image.height = left_image.rows;
                left_cuvslam_image.data = left_image.data;
                left_cuvslam_image.timestamp = timestamp_us;
                left_cuvslam_image.camera_index = static_cast<int32_t>(2 * i);  // Left camera index
                left_cuvslam_image.pitch = left_image.step;
                left_cuvslam_image.image_encoding = (left_image.channels() == 1) ? 
                    CUVSLAM_ImageEncoding::MONO8 : CUVSLAM_ImageEncoding::RGB8;
                
                cuvslam_images.push_back(left_cuvslam_image);
            }
            else
            {
                UERROR("No left image available for stereo pair %d", static_cast<int>(i));
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
                CUVSLAM_Image right_cuvslam_image;
                right_cuvslam_image.width = right_image.cols;
                right_cuvslam_image.height = right_image.rows;
                right_cuvslam_image.data = right_image.data;
                right_cuvslam_image.timestamp = timestamp_us;
                right_cuvslam_image.camera_index = static_cast<int32_t>(2 * i + 1);  // Right camera index
                right_cuvslam_image.pitch = right_image.step;
                right_cuvslam_image.image_encoding = (right_image.channels() == 1) ? 
                    CUVSLAM_ImageEncoding::MONO8 : CUVSLAM_ImageEncoding::RGB8;
                
                cuvslam_images.push_back(right_cuvslam_image);
            }
            else
            {
                UERROR("No right image available for stereo pair %d", static_cast<int>(i));
                return false;
            }
        }
        else
        {
            // Additional stereo pairs have camera models but no images
            // This is normal - RTABMap only provides images for the first stereo pair
            UDEBUG("Stereo pair %d has camera model but no images (this is normal)", static_cast<int>(i));
        }
    }
    
    UDEBUG("Prepared %d images for cuVSLAM (timestamp: %ld us)", 
           static_cast<int>(cuvslam_images.size()), timestamp_us);
    
    return true;
}

// Convert RTAB-Map Transform to tf2::Transform (same coordinate system!)
tf2::Transform rtabmapToTf2(const Transform & rtabmap_transform) {
    const float* data = rtabmap_transform.data();
    
    // RTAB-Map and ROS use the same coordinate system
    // Just need to convert the matrix format
    tf2::Matrix3x3 rotation(
        data[0], data[1], data[2],
        data[4], data[5], data[6],
        data[8], data[9], data[10]
    );
    
    tf2::Vector3 translation(data[3], data[7], data[11]);
    
    return tf2::Transform(rotation, translation);
}

// Convert tf2::Transform to cuVSLAM pose
CUVSLAM_Pose TocuVSLAMPose(const tf2::Transform & tf2_transform) {
    CUVSLAM_Pose cuvslam_pose;
    
    // Extract rotation matrix (column-major format for cuVSLAM)
    const tf2::Matrix3x3 rotation = tf2_transform.getBasis();
    cuvslam_pose.r[0] = rotation[0][0];  // r11
    cuvslam_pose.r[1] = rotation[1][0];  // r21
    cuvslam_pose.r[2] = rotation[2][0];  // r31
    cuvslam_pose.r[3] = rotation[0][1];  // r12
    cuvslam_pose.r[4] = rotation[1][1];  // r22
    cuvslam_pose.r[5] = rotation[2][1];  // r32
    cuvslam_pose.r[6] = rotation[0][2];  // r13
    cuvslam_pose.r[7] = rotation[1][2];  // r23
    cuvslam_pose.r[8] = rotation[2][2];  // r33
    
    // Extract translation
    const tf2::Vector3 translation = tf2_transform.getOrigin();
    cuvslam_pose.t[0] = translation.x();
    cuvslam_pose.t[1] = translation.y();
    cuvslam_pose.t[2] = translation.z();
    
    return cuvslam_pose;
}

CUVSLAM_Pose rtabmapTransformToCuVSLAMPose(const Transform & rtabmap_transform) {
    tf2::Transform tf2_transform = rtabmapToTf2(rtabmap_transform);
    return TocuVSLAMPose(tf2_transform); 
}


/*
Implementation based on Isaac ROS VisualSlamNode::VisualSlamImpl::CreateConfiguration()
Source: isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp:379-422
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/19be8c781a55dee9cfbe9f097adca3986638feb1/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp#L379-L422    
*/
CUVSLAM_Configuration OdometryCuVSLAM::CreateConfiguration(const CUVSLAM_Pose & cv_base_link_pose_cv_imu)
{
    
    CUVSLAM_Configuration configuration;
    CUVSLAM_InitDefaultConfiguration(&configuration);
    
    // Set basic configuration parameters (VO-only mode)
    configuration.multicam_mode = cuvslam_cameras_.size() > 1 ? 1 : 0;
    configuration.use_motion_model = 1;
    configuration.use_denoising = 0;  // Disable denoising by default
    configuration.horizontal_stereo_camera = cuvslam_cameras_.size() == 2 ? 1 : 0;
    
    // VO-only: Disable all SLAM-related features
    configuration.enable_observations_export = 0;  
    configuration.enable_landmarks_export = 0; 
    configuration.enable_localization_n_mapping = 0; 
    configuration.enable_reading_slam_internals = 0; 
    configuration.slam_sync_mode = 0;
    configuration.planar_constraints = 0;    
    configuration.slam_throttling_time_ms = 0; 
    configuration.slam_max_map_size = 0; 
    
    // IMU and timing settings
    configuration.enable_imu_fusion = 0;  // Will be set based on IMU availability
    configuration.debug_imu_mode = 0;  // Disable by default
    configuration.max_frame_delta_ms = 100;  // Default 100ms
    
    // Set IMU calibration (only if IMU is available)
    // CUVSLAM_ImuCalibration imu_calibration;
    // imu_calibration.rig_from_imu = cv_base_link_pose_cv_imu;
    
    // Use reasonable defaults for IMU noise parameters
    // TODO: Find a better way to get these params without hardcoding
    // imu_calibration.gyroscope_noise_density = 0.0002f;    
    // imu_calibration.gyroscope_random_walk = 0.00003f;      
    // imu_calibration.accelerometer_noise_density = 0.01f;   
    // imu_calibration.accelerometer_random_walk = 0.001f;    
    // imu_calibration.frequency = 200.0f; 
    // configuration.imu_calibration = imu_calibration;
    
    // VO-only: Skip localization settings (not needed for VO)
    // localization_settings are only used for SLAM mode
    
    return configuration;
}

Transform OdometryCuVSLAM::convertCuVSLAMPose(const CUVSLAM_Pose & cuvslam_pose)
{
    // Convert cuVSLAM pose to RTAB-Map Transform
    // cuVSLAM uses column-major rotation matrix, RTAB-Map uses row-major
    Transform rtabmap_transform;
    
    // Set rotation matrix (convert from column-major to row-major)

    // TODO: Validate this conversion

    rtabmap_transform(0,0) = cuvslam_pose.r[0];  // r11
    rtabmap_transform(0,1) = cuvslam_pose.r[3];  // r12
    rtabmap_transform(0,2) = cuvslam_pose.r[6];  // r13
    rtabmap_transform(1,0) = cuvslam_pose.r[1];  // r21
    rtabmap_transform(1,1) = cuvslam_pose.r[4];  // r22
    rtabmap_transform(1,2) = cuvslam_pose.r[7];  // r23
    rtabmap_transform(2,0) = cuvslam_pose.r[2];  // r31
    rtabmap_transform(2,1) = cuvslam_pose.r[5];  // r32
    rtabmap_transform(2,2) = cuvslam_pose.r[8];  // r33
    
    // Set translation
    rtabmap_transform(0,3) = cuvslam_pose.t[0];  // tx
    rtabmap_transform(1,3) = cuvslam_pose.t[1];  // ty
    rtabmap_transform(2,3) = cuvslam_pose.t[2];  // tz
    
    return rtabmap_transform;
}

#endif // RTABMAP_CUVSLAM

} // namespace rtabmap
