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
#include <cuvslam.h>
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
                       CUVSLAM_Tracker * & cuvslam_handle);
    
CUVSLAM_Configuration * CreateConfiguration(const CUVSLAM_Pose * cv_base_link_pose_cv_imu);

bool prepareImages(const SensorData & data, 
                    std::vector<CUVSLAM_Image> & cuvslam_images,
                    uint8_t * & gpu_left_image_data,
                    uint8_t * & gpu_right_image_data,
                    size_t & gpu_left_image_size,
                    size_t & gpu_right_image_size,
                    cudaStream_t & cuda_stream);

CUVSLAM_Pose convertCameraPoseToCuVSLAM(const Transform & rtabmap_camera_pose);
CUVSLAM_Pose convertImuPoseToCuVSLAM(const Transform & rtabmap_imu_pose);
Transform convertCuVSLAMPose(const CUVSLAM_Pose * cuvslam_pose);
cv::Mat convertCuVSLAMCovariance(const float * cuvslam_covariance);

}

#endif

// ============================================================================
// OdometryCuVSLAM Class Implementation
// ============================================================================

namespace rtabmap {

OdometryCuVSLAM::OdometryCuVSLAM(const ParametersMap & parameters) :
    Odometry(parameters),
#ifdef RTABMAP_CUVSLAM
    cuvslam_handle_(nullptr),
    initialized_(false),
    lost_(false),
    previous_pose_(Transform::getIdentity()),
    last_timestamp_(-1.0),
    gpu_left_image_data_(nullptr),
    gpu_right_image_data_(nullptr),
    gpu_left_image_size_(0),
    gpu_right_image_size_(0)
#endif
{
}

OdometryCuVSLAM::~OdometryCuVSLAM()
{
#ifdef RTABMAP_CUVSLAM
    // Clean up cuVSLAM tracker
    if(cuvslam_handle_)
    {
        CUVSLAM_DestroyTracker(cuvslam_handle_);
    }
    
    // Clean up GPU memory
    if(gpu_left_image_data_) {
        cudaFree(gpu_left_image_data_);
    }
    if(gpu_right_image_data_) {
        cudaFree(gpu_right_image_data_);
    }
#endif
}

void OdometryCuVSLAM::reset(const Transform & initialPose)
{
    Odometry::reset(initialPose);
    
#ifdef RTABMAP_CUVSLAM
    if(cuvslam_handle_)
    {
        CUVSLAM_DestroyTracker(cuvslam_handle_);
        cuvslam_handle_ = nullptr;
    }
    
    // Clean up GPU memory
    if(gpu_left_image_data_) {
        cudaFree(gpu_left_image_data_);
        gpu_left_image_data_ = nullptr;
    }
    if(gpu_right_image_data_) {
        cudaFree(gpu_right_image_data_);
        gpu_right_image_data_ = nullptr;
    }

    // Reset our internal state variables
    gpu_left_image_size_ = 0;
    gpu_right_image_size_ = 0;
    initialized_ = false;
    lost_ = false;
    previous_pose_ = initialPose;
    
    UWARN("RESET: cuVSLAM reset process completed");
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
        
               transform.setNull();
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
        if(!initializeCuVSLAM(
            data, 
            cuvslam_handle_))
        {
            UERROR("Failed to initialize cuVSLAM tracker");
            return transform;
        }
        initialized_ = true;
        
        // For first frame after reset, return null transform to avoid reset cascade
        // The identity transform was causing rtabmap to detect another reset
        transform.setNull();
        if(info)
        {
            info->type = 0; // Success: Initialization successful
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0; // High uncertainty
            info->timeEstimation = timer.ticks();
        }
        
        
        // Set previous_pose_ to identity for next frame calculation (not null)
        last_timestamp_ = data.stamp();
        
        return transform;
    }
    
    // Prepare images for cuVSLAM
    std::vector<CUVSLAM_Image> cuvslam_image_objects;
    cudaStream_t cuda_stream = nullptr;

    if(!prepareImages(
        data,
        cuvslam_image_objects,
        gpu_left_image_data_,
        gpu_right_image_data_,
        gpu_left_image_size_,
        gpu_right_image_size_,
        cuda_stream))
    {
        UERROR("Failed to prepare images for cuVSLAM");
        // Clean up CUDA stream if allocation failed
        if(cuda_stream) {
            cudaStreamDestroy(cuda_stream);
        }
        return transform;
    }
    
    // Process IMU data if available
    if(!data.imu().empty())
    {
        // TODO: Implement IMU processing
        // For now, we'll skip IMU processing as it's not implemented yet
        UWARN("IMU data available but processing not implemented yet");
    }
    
    // Validate inputs before calling cuVSLAM
    if(cuvslam_image_objects.empty()) {
        UERROR("No images prepared for cuVSLAM tracking");
        return transform;
    }
    
    // Validate cuVSLAM tracker
    if(!cuvslam_handle_) {
        UERROR("cuVSLAM tracker is null! initialized_: %s", initialized_ ? "true" : "false");
        return transform;
    }
    
    CUVSLAM_PoseEstimate vo_pose_estimate;
    
    const CUVSLAM_Status vo_status = CUVSLAM_TrackGpuMem(
        cuvslam_handle_, 
        cuvslam_image_objects.data(), 
        cuvslam_image_objects.size(), 
        nullptr, 
        &vo_pose_estimate
    );
    
    // Clean up CUDA stream
    if(cuda_stream) {
        cudaStreamDestroy(cuda_stream);
    }

    // Handle tracking status and odom info
    if(vo_status == CUVSLAM_TRACKING_LOST)
    {
        UWARN("cuVSLAM tracking lost");
        lost_ = true;
        if(info)
        {
            info->type = 1; // Tracking lost
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0; // High uncertainty
            info->timeEstimation = timer.ticks();
        }

        transform.setNull();
        previous_pose_.setNull();
        last_timestamp_ = data.stamp();
        
        return transform;
    }
    else if(vo_status != CUVSLAM_SUCCESS)
    {
        // Provide more specific error information
        const char* error_msg = "Unknown error";
        switch(vo_status) {
            case 1: error_msg = "CUVSLAM_INVALID_PARAMETER"; break;
            case 2: error_msg = "CUVSLAM_INVALID_IMAGE_FORMAT or CUVSLAM_INVALID_CAMERA_CONFIG"; break;
            case 3: error_msg = "CUVSLAM_GPU_MEMORY_ERROR"; break;
            case 4: error_msg = "CUVSLAM_INITIALIZATION_ERROR"; break;
            default: error_msg = "Unknown cuVSLAM error"; break;
        }
        
        UERROR("cuVSLAM tracking error: %d (%s)", vo_status, error_msg);
        
        if(info)
        {
            info->type = 1; // Error
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0; // High uncertainty
            info->timeEstimation = timer.ticks();
        }
        
        transform.setNull();
        previous_pose_.setNull();
        last_timestamp_ = data.stamp();
        
        return transform;
    }
    else
    {
        if(info)
        {
            cv::Mat covMat = convertCuVSLAMCovariance(vo_pose_estimate.covariance);
            info->type = 0;  // Success
            info->reg.covariance = covMat;
            info->timeEstimation = timer.ticks();
        }
    }
    
    // Convert cuVSLAM pose to RTAB-Map Transform
    Transform current_pose = convertCuVSLAMPose(&vo_pose_estimate.pose);
    
    // Calculate incremental transform
    if(!previous_pose_.isNull())
    {
        transform = previous_pose_.inverse() * current_pose;
    }
    else
    {
        // If previous_pose_ is null (e.g., after tracking was lost), 
        // return the current pose as absolute transform
        transform = current_pose;
    }

    // Check if current pose is identity (which could trigger another reset)
    if(transform.isIdentity())
    {
        UWARN("Current pose is identity - returning null to avoid reset cascade");
        transform.setNull();
    }

    // Log time between odom publishing
    const double time_between_odom_publishing = data.stamp() - last_timestamp_;
    UWARN("Time between odom publishing: %fs", time_between_odom_publishing);
    const double odom_publishing_frequency = 1.0 / time_between_odom_publishing;
    UWARN("Odom publishing frequency: %fHz", odom_publishing_frequency);

    // Update tracking state and previous pose
    lost_ = false;
    previous_pose_ = current_pose;
    last_timestamp_ = data.stamp();
    
    
#else
    UERROR("cuVSLAM support not compiled in RTAB-Map");
#endif
    
    return transform;
}

#ifdef RTABMAP_CUVSLAM

// ============================================================================
// cuVSLAM Initialization and Configuration
// ============================================================================

bool initializeCuVSLAM(const SensorData & data, 
                       CUVSLAM_Tracker * & cuvslam_handle)
{    
    // Local camera parameters
    float left_camera_params[4];
    float right_camera_params[4];
    
    // Local distortion model strings
    const char * left_distortion_model = "pinhole";
    const char * right_distortion_model = "pinhole";
    
    // Create local camera objects vector
    std::vector<CUVSLAM_Camera> cuvslam_camera_objects;
    
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
        
        // Set parameters for pinhole model (rectified images) - use member variable
        left_camera_params[0] = static_cast<float>(leftModel.cx());   // cx
        left_camera_params[1] = static_cast<float>(leftModel.cy());   // cy
        left_camera_params[2] = static_cast<float>(leftModel.fx());   // fx
        left_camera_params[3] = static_cast<float>(leftModel.fy());   // fy
        left_camera.parameters = left_camera_params;
        
        // Set pinhole model with zero distortion (rectified images)
        left_camera.distortion_model = left_distortion_model;
        left_camera.num_parameters = 4;
        
        // Set camera pose (extrinsics) from localTransform with proper coordinate system conversion
        Transform left_pose = leftModel.localTransform();
        left_camera.pose = convertCameraPoseToCuVSLAM(left_pose);
    
        cuvslam_camera_objects.push_back(left_camera);
        
        CUVSLAM_Camera right_camera;
        right_camera.width = rightModel.imageWidth();
        right_camera.height = rightModel.imageHeight();
        
        // Set parameters for pinhole model (rectified images) - use member variable
        right_camera_params[0] = static_cast<float>(rightModel.cx());   // cx
        right_camera_params[1] = static_cast<float>(rightModel.cy());   // cy
        right_camera_params[2] = static_cast<float>(rightModel.fx());   // fx
        right_camera_params[3] = static_cast<float>(rightModel.fy());   // fy
        right_camera.parameters = right_camera_params;
        
        // Set pinhole model with zero distortion (rectified images)
        right_camera.distortion_model = right_distortion_model;
        right_camera.num_parameters = 4;
        
        // Set right camera pose using stereo baseline (proper stereo configuration)
        double baseline = stereoModel.baseline();
        
        // Create right camera pose with baseline offset (horizontal stereo)
        // Transform left pose by baseline offset, following OdometryOkvis pattern
        Transform baseline_transform(1, 0, 0, baseline,
                                     0, 1, 0, 0,
                                     0, 0, 1, 0);
        Transform right_pose = left_pose * baseline_transform;
        right_camera.pose = convertCameraPoseToCuVSLAM(right_pose);
        
        cuvslam_camera_objects.push_back(right_camera);
    }
    
    // Set up camera rig
    CUVSLAM_CameraRig * camera_rig = new CUVSLAM_CameraRig();
    camera_rig->cameras = cuvslam_camera_objects.data();
    camera_rig->num_cameras = cuvslam_camera_objects.size();
    
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
    cuvslam_imu_pose = convertImuPoseToCuVSLAM(Transform::getIdentity());

    
    CUVSLAM_Configuration * configuration = CreateConfiguration(&cuvslam_imu_pose);
    
    // Enable IMU fusion if we have IMU data
    if (!data.imu().empty()) {
        configuration->enable_imu_fusion = 1;
    }
    
    // Create tracker
    CUVSLAM_TrackerHandle tracker_handle;
    CUVSLAM_Status status = CUVSLAM_CreateTracker(&tracker_handle, camera_rig, configuration);
    cuvslam_handle = tracker_handle;

    delete configuration;
    delete camera_rig;

    if(status != CUVSLAM_SUCCESS)
    {
        UERROR("Failed to create cuVSLAM tracker: %d", status);
        return false;
    }
    
    return true;
}

/*
Implementation based on Isaac ROS VisualSlamNode::VisualSlamImpl::CreateConfiguration()
Source: isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp:379-422
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/19be8c781a55dee9cfbe9f097adca3986638feb1/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp#L379-L422    
*/
CUVSLAM_Configuration * CreateConfiguration(const CUVSLAM_Pose * cv_base_link_pose_cv_imu)
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
    configuration->enable_observations_export = 0;       // TODO: What does this do?
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

bool synchronizeGpuOperations(cudaStream_t cuda_stream)
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
                   uint8_t * & gpu_left_image_data,
                   uint8_t * & gpu_right_image_data,
                   size_t & gpu_left_image_size,
                   size_t & gpu_right_image_size,
                   cudaStream_t & cuda_stream)
{
    
    // Convert timestamp to nanoseconds (cuVSLAM expects nanoseconds)
    int64_t timestamp_ns = static_cast<int64_t>(data.stamp() * 1000000000.0);
    
    // Process all stereo pairs, assume one camera pair
    // TODO: Handle multiple camera pairs
    if(!data.imageRaw().empty())
    {
        cv::Mat left_image = data.imageRaw();
        
        // Validate image format
        if(left_image.channels() != 1 && left_image.channels() != 3)
        {
            UERROR("Unsupported left image format: %d channels", left_image.channels());
            return false;
        }
        
        // Convert image format for cuVSLAM
        cv::Mat processed_left_image;
        CUVSLAM_ImageEncoding left_encoding;
        if(left_image.channels() == 1) {
            // Keep grayscale as MONO8
            processed_left_image = left_image.clone();
            left_encoding = CUVSLAM_ImageEncoding::MONO8;
        } else if(left_image.channels() == 3) {
            // Convert BGR to RGB
            cv::cvtColor(left_image, processed_left_image, cv::COLOR_BGR2RGB);
            left_encoding = CUVSLAM_ImageEncoding::RGB8;
        } else {
            UERROR("Unsupported left image format: %d channels", left_image.channels());
            return false;
        }
        
        // Validate converted image
        int expected_channels = (left_encoding == CUVSLAM_ImageEncoding::MONO8) ? 1 : 3;
        if(processed_left_image.channels() != expected_channels) {
            UERROR("ERROR: Left image conversion failed - expected %d channels, got %d", expected_channels, processed_left_image.channels());
            return false;
        }
        if(processed_left_image.data == nullptr) {
            UERROR("ERROR: Left processed image has null data pointer!");
            return false;
        }
        
        // Efficient GPU memory allocation and async copy
        size_t left_image_size = processed_left_image.total() * processed_left_image.elemSize();
        if(!allocateGpuMemory(left_image_size, &gpu_left_image_data, &gpu_left_image_size)) {
            return false;
        }
        
        if(!copyToGpuAsync(processed_left_image, gpu_left_image_data, left_image_size, cuda_stream)) {
            return false;
        }
        
        // Create CUVSLAM_Image for left camera with GPU memory
        CUVSLAM_Image left_cuvslam_image;
        left_cuvslam_image.width = processed_left_image.cols;
        left_cuvslam_image.height = processed_left_image.rows;
        left_cuvslam_image.pixels = gpu_left_image_data;  // GPU memory pointer
        left_cuvslam_image.timestamp_ns = timestamp_ns;
        left_cuvslam_image.camera_index = 0;
        left_cuvslam_image.pitch = processed_left_image.step;
        left_cuvslam_image.image_encoding = left_encoding;
        
        cuvslam_images.push_back(left_cuvslam_image);
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
        
        // Convert image format for cuVSLAM
        cv::Mat processed_right_image;
        CUVSLAM_ImageEncoding right_encoding;
        if(right_image.channels() == 1) {
            // Keep grayscale as MONO8
            processed_right_image = right_image.clone();
            right_encoding = CUVSLAM_ImageEncoding::MONO8;
        } else if(right_image.channels() == 3) {
            // Convert BGR to RGB
            cv::cvtColor(right_image, processed_right_image, cv::COLOR_BGR2RGB);
            right_encoding = CUVSLAM_ImageEncoding::RGB8;
        } else {
            UERROR("Unsupported right image format: %d channels", right_image.channels());
            return false;
        }
        
        // Validate converted image
        int expected_channels = (right_encoding == CUVSLAM_ImageEncoding::MONO8) ? 1 : 3;
        if(processed_right_image.channels() != expected_channels) {
            UERROR("ERROR: Right image conversion failed - expected %d channels, got %d", expected_channels, processed_right_image.channels());
            return false;
        }
        if(processed_right_image.data == nullptr) {
            UERROR("ERROR: Right processed image has null data pointer!");
            return false;
        }
        
        // Efficient GPU memory allocation and async copy
        size_t right_image_size = processed_right_image.total() * processed_right_image.elemSize();
        if(!allocateGpuMemory(right_image_size, &gpu_right_image_data, &gpu_right_image_size)) {
            return false;
        }
        
        if(!copyToGpuAsync(processed_right_image, gpu_right_image_data, right_image_size, cuda_stream)) {
            return false;
        }
        
        // Create CUVSLAM_Image for right camera with GPU memory
        CUVSLAM_Image right_cuvslam_image;
        right_cuvslam_image.width = processed_right_image.cols;
        right_cuvslam_image.height = processed_right_image.rows;
        right_cuvslam_image.pixels = gpu_right_image_data;  // GPU memory pointer
        right_cuvslam_image.timestamp_ns = timestamp_ns;
        right_cuvslam_image.camera_index = 1;  // Right camera index
        right_cuvslam_image.pitch = processed_right_image.step;
        right_cuvslam_image.image_encoding = right_encoding;
        
        cuvslam_images.push_back(right_cuvslam_image);
    } 
    else 
    {
        UERROR("No right image available for stereo camera");
        return false;
    }
    
    
    // Synchronize all async GPU operations before returning
    if(!synchronizeGpuOperations(cuda_stream)) {
        return false;
    }
    
    return true;
}

// ============================================================================
// Coordinate System and Transform Conversion Functions
// ============================================================================

// Helper function that converts RTAB-Map Transform into CUVSLAM_Pose
// Based on Isaac ROS implementation but using RTAB-Map Transform directly
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
// Based on Isaac ROS implementation but using RTAB-Map Transform directly
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

// Helper function to change basis from frame source to frame target
// Based on Isaac ROS implementation but using RTAB-Map Transform directly
Transform ChangeBasis(
  const Transform & target_pose_source, const Transform & source_pose_source)
{
  return target_pose_source * source_pose_source * target_pose_source.inverse();
}


// Helper function to convert camera pose from RTABMap to cuVSLAM coordinate system
// Based on Isaac ROS FillExtrinsics implementation
CUVSLAM_Pose convertCameraPoseToCuVSLAM(const Transform & rtabmap_camera_pose) 
{
    // Apply coordinate system transformation:
    // RTABMap robot frame -> cuVSLAM robot frame -> cuVSLAM camera frame
    // This follows the same pattern as Isaac ROS FillExtrinsics
    const Transform camera_pose_cuvslam = 
        cuvslam_pose_canonical * rtabmap_camera_pose * optical_pose_cuvslam;
    
    return TocuVSLAMPose(camera_pose_cuvslam);
}

// Helper function to convert IMU pose from RTABMap to cuVSLAM coordinate system
// Based on Isaac ROS IMU handling - does NOT apply optical transform
CUVSLAM_Pose convertImuPoseToCuVSLAM(const Transform & rtabmap_imu_pose) {
    // Apply coordinate system transformation:
    // RTABMap robot frame -> cuVSLAM robot frame
    // This follows the same pattern as Isaac ROS IMU handling
    const Transform imu_pose_cuvslam = cuvslam_pose_canonical * rtabmap_imu_pose;
    
    return TocuVSLAMPose(imu_pose_cuvslam);
}

/*
Convert cuVSLAM pose to RTAB-Map Transform.
CuVSLAM uses column-major rotation matrix, RTAB-Map uses row-major.
Applies coordinate system transformation from cuVSLAM frame to RTAB-Map frame.
*/
Transform convertCuVSLAMPose(const CUVSLAM_Pose * cuvslam_pose) 
{
    // Convert cuVSLAM pose to RTAB-Map Transform (cuVSLAM coordinate system)
    Transform odom_pose_cuvslam = FromcuVSLAMPose(*cuvslam_pose);
    
    // Apply coordinate system transformation from cuVSLAM to RTAB-Map (ROS conventions)
    // Following Isaac ROS pattern exactly
    Transform rtabmap_transform = ChangeBasis(canonical_pose_cuvslam, odom_pose_cuvslam);
    
    return rtabmap_transform;
}

/*
Convert cuVSLAM covariance to RTAB-Map format.
Based on Isaac ROS implementation: FromcuVSLAMCovariance()
Source: isaac_ros_visual_slam/src/impl/cuvslam_ros_conversion.cpp:275-299
*/
cv::Mat convertCuVSLAMCovariance(const float * cuvslam_covariance)
{
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
        }
    }
    
    // Ensure diagonal elements are positive and finite (RTAB-Map requirement)
    for(int i = 0; i < 6; i++)
    {
        double diag_val = cv_covariance.at<double>(i, i);
        if(!std::isfinite(diag_val) || diag_val <= 0.0)
        {
            UWARN("Diagonal element %d of covariance is not finite or positive (value: %f), proceeding with default infinite covariance", i, diag_val);
            cv_covariance.at<double>(i, i) = 9999.0;  // High uncertainty
        }
    }
    
    return cv_covariance;
}

#endif // RTABMAP_CUVSLAM

} // namespace rtabmap

