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
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <eigen3/Eigen/Dense>
#include <cuda_runtime.h>


namespace rtabmap {

    // Forward declarations for helper functions
    bool initializeCuVSLAM(const SensorData & data, 
                           std::vector<CUVSLAM_Camera*> & cuvslam_cameras,
                           std::vector<CUVSLAM_Camera> * & cuvslam_camera_objects,
                           CUVSLAM_CameraRig * & camera_rig,
                           CUVSLAM_Configuration * & configuration,
                           CUVSLAM_Tracker * & cuvslam_handle_,
                           float left_camera_params_[4],
                           float right_camera_params_[4],
                           const char * & left_distortion_model_,
                           const char * & right_distortion_model_);
    
    CUVSLAM_Configuration * CreateConfiguration(const CUVSLAM_Pose * cv_base_link_pose_cv_imu);
    
    bool prepareImages(const SensorData & data, 
                       std::vector<CUVSLAM_Image*> & cuvslam_images,
                       const std::vector<CUVSLAM_Camera*> & cuvslam_cameras,
                       cv::Mat & processed_left_image_,
                       cv::Mat & processed_right_image_,
                       uint8_t * & gpu_left_image_data_,
                       uint8_t * & gpu_right_image_data_,
                       size_t & gpu_left_image_size_,
                       size_t & gpu_right_image_size_,
                       void * & cuda_stream_);
    
    CUVSLAM_Pose convertCameraPoseToCuVSLAM(const Transform & rtabmap_camera_pose);
    CUVSLAM_Pose convertImuPoseToCuVSLAM(const Transform & rtabmap_imu_pose);
    Transform convertCuVSLAMPose(const CUVSLAM_Pose * cuvslam_pose);
    cv::Mat convertCuVSLAMCovariance(const float * cuvslam_covariance);

} // namespace rtabmap


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

// ============================================================================
// OdometryCuVSLAM Class Implementation
// ============================================================================

namespace rtabmap {

OdometryCuVSLAM::OdometryCuVSLAM(const ParametersMap & parameters) :
    Odometry(parameters),
#ifdef RTABMAP_CUVSLAM
    cuvslam_handle_(nullptr),
    cuvslam_camera_objects_(nullptr),
    cuvslam_image_objects_(nullptr),
    camera_rig_(nullptr),
    configuration_(nullptr),
    initialized_(false),
    lost_(false),
    previous_pose_(Transform::getIdentity()),
    last_timestamp_(-1.0),
    left_camera_params_{0.0f, 0.0f, 0.0f, 0.0f},
    right_camera_params_{0.0f, 0.0f, 0.0f, 0.0f},
    left_distortion_model_(nullptr),
    right_distortion_model_(nullptr),
    gpu_left_image_data_(nullptr),
    gpu_right_image_data_(nullptr),
    gpu_left_image_size_(0),
    gpu_right_image_size_(0),
    cuda_stream_(nullptr)
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
        cuvslam_handle_ = nullptr;
    }
        
    for(CUVSLAM_Camera * camera : cuvslam_cameras_) {
        delete camera;
    }
    cuvslam_cameras_.clear();
    
    delete camera_rig_;
    camera_rig_ = nullptr;
    delete configuration_;
    configuration_ = nullptr;
    delete cuvslam_camera_objects_;
    cuvslam_camera_objects_ = nullptr;
    delete cuvslam_image_objects_;
    cuvslam_image_objects_ = nullptr;
    
    // Clean up GPU memory
    cudaFree(gpu_left_image_data_);
    gpu_left_image_data_ = nullptr;
    cudaFree(gpu_right_image_data_);
    gpu_right_image_data_ = nullptr;
    
    // Clean up CUDA stream
    if(cuda_stream_) {
        cudaStream_t stream = static_cast<cudaStream_t>(cuda_stream_);
        cudaStreamDestroy(stream);
        cuda_stream_ = nullptr;
    }
#endif
}

void OdometryCuVSLAM::reset(const Transform & initialPose)
{
    Odometry::reset(initialPose);
    
#ifdef RTABMAP_CUVSLAM
    // Destroy existing tracker first (before cleaning up parameters it might be using)
    if(cuvslam_handle_)
    {
        CUVSLAM_DestroyTracker(cuvslam_handle_);
        cuvslam_handle_ = nullptr;
    }
        
    // Reset member variables to initial state
    // Clean up camera rig and configuration
    delete camera_rig_;
    camera_rig_ = nullptr;
    delete configuration_;
    configuration_ = nullptr;

    // Clean up camera objects
    for(CUVSLAM_Camera* camera : cuvslam_cameras_) {
        delete camera;
    }
    cuvslam_cameras_.clear();
    
    delete cuvslam_camera_objects_;
    cuvslam_camera_objects_ = nullptr;
    
    delete cuvslam_image_objects_;
    cuvslam_image_objects_ = nullptr;
    
    // Clean up GPU memory
    cudaFree(gpu_left_image_data_);
    gpu_left_image_data_ = nullptr;
    cudaFree(gpu_right_image_data_);
    gpu_right_image_data_ = nullptr;
    
    // Clean up CUDA stream
    if(cuda_stream_) {
        cudaStream_t stream = static_cast<cudaStream_t>(cuda_stream_);
        cudaStreamDestroy(stream);
        cuda_stream_ = nullptr;
    }
    
    initialized_ = false;
    lost_ = false;
    previous_pose_ = initialPose;
    last_timestamp_ = -1.0;
    gpu_left_image_size_ = 0;
    gpu_right_image_size_ = 0;
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
        if(!initializeCuVSLAM(
            data, 
            cuvslam_cameras_, 
            cuvslam_camera_objects_,
            camera_rig_,
            configuration_,
            cuvslam_handle_,
            left_camera_params_,
            right_camera_params_,
            left_distortion_model_,
            right_distortion_model_))
        {
            UERROR("Failed to initialize cuVSLAM tracker");
            return transform;
        }
        initialized_ = true;
        
        // For first frame, return identity transform with high covariance
        transform = Transform::getIdentity();
        if(info)
        {
            info->type = 0; // Success: Initialization successful
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0; // High uncertainty
            info->timeEstimation = timer.ticks();
        }
        
        previous_pose_ = transform;
        last_timestamp_ = data.stamp();
        
        return transform;
    }
    
    // Prepare images for cuVSLAM
    std::vector<CUVSLAM_Image*> cuvslam_images;
    if(!prepareImages(
        data,
        cuvslam_images,
        cuvslam_cameras_,
        processed_left_image_,
        processed_right_image_,
        gpu_left_image_data_,
        gpu_right_image_data_,
        gpu_left_image_size_,
        gpu_right_image_size_,
        cuda_stream_))
    {
        UERROR("Failed to prepare images for cuVSLAM");
        // Clean up any images that were created before the failure
        for(CUVSLAM_Image* img : cuvslam_images) {
            delete img;
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

    
    // Create persistent array of image objects for cuVSLAM API
    if (!cuvslam_image_objects_) {
        cuvslam_image_objects_ = new std::vector<CUVSLAM_Image>();
    } else {
        cuvslam_image_objects_->clear();
    }
    for(CUVSLAM_Image* img_ptr : cuvslam_images) {
        cuvslam_image_objects_->push_back(*img_ptr);
    }
    
    // Validate inputs before calling cuVSLAM
    if(cuvslam_image_objects_->empty()) {
        UERROR("No images prepared for cuVSLAM tracking");
        return transform;
    }
    
    // Validate camera rig and configuration
    if(!camera_rig_) {
        UERROR("Camera rig is null!");
        return transform;
    }
    if(!configuration_) {
        UERROR("cuVSLAM configuration is null!");
        return transform;
    }
    
    
    CUVSLAM_PoseEstimate vo_pose_estimate;
    
    const CUVSLAM_Status vo_status = CUVSLAM_TrackGpuMem(
        cuvslam_handle_, 
        cuvslam_image_objects_->data(), 
        cuvslam_image_objects_->size(), 
        nullptr, 
        &vo_pose_estimate
    );
    
    // Clean up CUVSLAM_Image objects
    for(CUVSLAM_Image * image : cuvslam_images) {
        delete image;
    }
    cuvslam_images.clear();
    
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
        transform = current_pose;
    }

    // Log time between odom publishing
    const double time_between_odom_publishing = data.stamp() - last_timestamp_;
    UWARN("Time between odom publishing: %fs", time_between_odom_publishing);
    const double odom_publishing_frequency = 1.0 / time_between_odom_publishing;
    UWARN("Odom publishing frequency: %fHz", odom_publishing_frequency);

    // Update tracking state
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
                       std::vector<CUVSLAM_Camera*> & cuvslam_cameras,
                       std::vector<CUVSLAM_Camera> * & cuvslam_camera_objects,
                       CUVSLAM_CameraRig * & camera_rig,
                       CUVSLAM_Configuration * & configuration,
                       CUVSLAM_Tracker * & cuvslam_handle_,
                       float left_camera_params_[4],
                       float right_camera_params_[4],
                       const char * & left_distortion_model_,
                       const char * & right_distortion_model_)
{    
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
        left_camera_params_[0] = static_cast<float>(leftModel.cx());   // cx
        left_camera_params_[1] = static_cast<float>(leftModel.cy());   // cy
        left_camera_params_[2] = static_cast<float>(leftModel.fx());   // fx
        left_camera_params_[3] = static_cast<float>(leftModel.fy());   // fy
        left_camera.parameters = left_camera_params_;
        
        // Set pinhole model with zero distortion (rectified images)
        left_distortion_model_ = "pinhole";
        left_camera.distortion_model = left_distortion_model_;
        left_camera.num_parameters = 4;
        
        // Set camera pose (extrinsics) from localTransform with proper coordinate system conversion
        Transform left_pose = leftModel.localTransform();
        left_camera.pose = convertCameraPoseToCuVSLAM(left_pose);
    
        cuvslam_cameras.push_back(new CUVSLAM_Camera(left_camera));
        
        CUVSLAM_Camera right_camera;
        right_camera.width = rightModel.imageWidth();
        right_camera.height = rightModel.imageHeight();
        
        // Set parameters for pinhole model (rectified images) - use member variable
        right_camera_params_[0] = static_cast<float>(rightModel.cx());   // cx
        right_camera_params_[1] = static_cast<float>(rightModel.cy());   // cy
        right_camera_params_[2] = static_cast<float>(rightModel.fx());   // fx
        right_camera_params_[3] = static_cast<float>(rightModel.fy());   // fy
        right_camera.parameters = right_camera_params_;
        
        // Set pinhole model with zero distortion (rectified images)
        right_distortion_model_ = "pinhole";
        right_camera.distortion_model = right_distortion_model_;
        right_camera.num_parameters = 4;
        
        // Set right camera pose using stereo baseline (proper stereo configuration)
        double baseline = data.stereoCameraModels()[0].baseline();
        
        // Create right camera pose with baseline offset (horizontal stereo)
        // Transform left pose by baseline offset, following OdometryOkvis pattern
        Transform baseline_transform(1, 0, 0, baseline,
                                     0, 1, 0, 0,
                                     0, 0, 1, 0);
        Transform right_pose = left_pose * baseline_transform;
        right_camera.pose = convertCameraPoseToCuVSLAM(right_pose);
        
        cuvslam_cameras.push_back(new CUVSLAM_Camera(right_camera));
    }
    
    // Set up camera rig
    camera_rig = new CUVSLAM_CameraRig();
    // Create persistent array of camera objects for cuVSLAM API
    if (!cuvslam_camera_objects) {
        cuvslam_camera_objects = new std::vector<CUVSLAM_Camera>();
    }
    cuvslam_camera_objects->clear();
    for(CUVSLAM_Camera* cam_ptr : cuvslam_cameras) {
        cuvslam_camera_objects->push_back(*cam_ptr);
    }
    camera_rig->cameras = cuvslam_camera_objects->data();
    camera_rig->num_cameras = cuvslam_camera_objects->size();
    
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
    CUVSLAM_Pose cuvslam_imu_pose = convertImuPoseToCuVSLAM(Transform::getIdentity());

    
    configuration = CreateConfiguration(&cuvslam_imu_pose);
    
    // Enable IMU fusion if we have IMU data
    if (!data.imu().empty()) {
        configuration->enable_imu_fusion = 1;
    }
    
    // Create tracker
    CUVSLAM_TrackerHandle tracker_handle;
    CUVSLAM_Status status = CUVSLAM_CreateTracker(&tracker_handle, camera_rig, configuration);
    cuvslam_handle_ = tracker_handle;
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
CUVSLAM_Configuration * CreateConfiguration(const CUVSLAM_Pose* cv_base_link_pose_cv_imu)
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
    
    return configuration;
}

// ============================================================================
// GPU Memory Management
// ============================================================================

bool allocateGpuMemory(size_t size, uint8_t ** gpu_ptr, size_t * current_size)
{
    if(*current_size != size) {
        // Reallocate GPU memory if size changed
        cudaFree(*gpu_ptr);
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

bool copyToGpuAsync(const cv::Mat& cpu_image, uint8_t * gpu_ptr, size_t size, void * & cuda_stream_)
{
    // Initialize CUDA stream if not already done
    if(cuda_stream_ == nullptr) {
        cudaStream_t stream;
        cudaError_t stream_err = cudaStreamCreate(&stream);
        if(stream_err != cudaSuccess) {
            UERROR("Failed to create CUDA stream: %s", cudaGetErrorString(stream_err));
            return false;
        }
        cuda_stream_ = static_cast<void*>(stream);
    }
    
    // Cast void * back to cudaStream_t for CUDA API calls
    cudaStream_t stream = static_cast<cudaStream_t>(cuda_stream_);
    
    // Copy CPU data to GPU memory with async operation for better performance
    cudaError_t cuda_err = cudaMemcpyAsync(gpu_ptr, cpu_image.data, size, 
                                          cudaMemcpyHostToDevice, stream);
    if(cuda_err != cudaSuccess) {
        UERROR("Failed to copy image to GPU: %s", cudaGetErrorString(cuda_err));
        return false;
    }
    
    return true;
}

bool synchronizeGpuOperations(void * cuda_stream_)
{
    if(cuda_stream_) {
        cudaStream_t stream = static_cast<cudaStream_t>(cuda_stream_);
        cudaError_t cuda_err = cudaStreamSynchronize(stream);
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
                   std::vector<CUVSLAM_Image*> & cuvslam_images,
                   const std::vector<CUVSLAM_Camera*> & cuvslam_cameras,
                   cv::Mat & processed_left_image_,
                   cv::Mat & processed_right_image_,
                   uint8_t * & gpu_left_image_data_,
                   uint8_t * & gpu_right_image_data_,
                   size_t & gpu_left_image_size_,
                   size_t & gpu_right_image_size_,
                   void * & cuda_stream_)
{
    cuvslam_images.clear();
    
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
        CUVSLAM_ImageEncoding left_encoding;
        if(left_image.channels() == 1) {
            // Keep grayscale as MONO8
            processed_left_image_ = left_image.clone();
            left_encoding = CUVSLAM_ImageEncoding::MONO8;
        } else if(left_image.channels() == 3) {
            // Convert BGR to RGB
            cv::cvtColor(left_image, processed_left_image_, cv::COLOR_BGR2RGB);
            left_encoding = CUVSLAM_ImageEncoding::RGB8;
        } else {
            UERROR("Unsupported left image format: %d channels", left_image.channels());
            return false;
        }
        
        // Validate converted image
        int expected_channels = (left_encoding == CUVSLAM_ImageEncoding::MONO8) ? 1 : 3;
        if(processed_left_image_.channels() != expected_channels) {
            UERROR("ERROR: Left image conversion failed - expected %d channels, got %d", expected_channels, processed_left_image_.channels());
            return false;
        }
        if(processed_left_image_.data == nullptr) {
            UERROR("ERROR: Left processed image has null data pointer!");
            return false;
        }
        
        // Efficient GPU memory allocation and async copy
        size_t left_image_size = processed_left_image_.total() * processed_left_image_.elemSize();
        if(!allocateGpuMemory(left_image_size, &gpu_left_image_data_, &gpu_left_image_size_)) {
            return false;
        }
        
        if(!copyToGpuAsync(processed_left_image_, gpu_left_image_data_, left_image_size, cuda_stream_)) {
            return false;
        }
        
        // Create CUVSLAM_Image for left camera with GPU memory
        CUVSLAM_Image * left_cuvslam_image = new CUVSLAM_Image();
        left_cuvslam_image->width = processed_left_image_.cols;
        left_cuvslam_image->height = processed_left_image_.rows;
        left_cuvslam_image->pixels = gpu_left_image_data_;  // GPU memory pointer
        left_cuvslam_image->timestamp_ns = timestamp_ns;
        left_cuvslam_image->camera_index = 0;
        left_cuvslam_image->pitch = processed_left_image_.step;
        left_cuvslam_image->image_encoding = left_encoding;
        
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
        CUVSLAM_ImageEncoding right_encoding;
        if(right_image.channels() == 1) {
            // Keep grayscale as MONO8
            processed_right_image_ = right_image.clone();
            right_encoding = CUVSLAM_ImageEncoding::MONO8;
        } else if(right_image.channels() == 3) {
            // Convert BGR to RGB
            cv::cvtColor(right_image, processed_right_image_, cv::COLOR_BGR2RGB);
            right_encoding = CUVSLAM_ImageEncoding::RGB8;
        } else {
            UERROR("Unsupported right image format: %d channels", right_image.channels());
            return false;
        }
        
        // Validate converted image
        int expected_channels = (right_encoding == CUVSLAM_ImageEncoding::MONO8) ? 1 : 3;
        if(processed_right_image_.channels() != expected_channels) {
            UERROR("ERROR: Right image conversion failed - expected %d channels, got %d", expected_channels, processed_right_image_.channels());
            return false;
        }
        if(processed_right_image_.data == nullptr) {
            UERROR("ERROR: Right processed image has null data pointer!");
            return false;
        }
        
        // Efficient GPU memory allocation and async copy
        size_t right_image_size = processed_right_image_.total() * processed_right_image_.elemSize();
        if(!allocateGpuMemory(right_image_size, &gpu_right_image_data_, &gpu_right_image_size_)) {
            return false;
        }
        
        if(!copyToGpuAsync(processed_right_image_, gpu_right_image_data_, right_image_size, cuda_stream_)) {
            return false;
        }
        
        // Create CUVSLAM_Image for right camera with GPU memory
        CUVSLAM_Image * right_cuvslam_image = new CUVSLAM_Image();
        right_cuvslam_image->width = processed_right_image_.cols;
        right_cuvslam_image->height = processed_right_image_.rows;
        right_cuvslam_image->pixels = gpu_right_image_data_;  // GPU memory pointer
        right_cuvslam_image->timestamp_ns = timestamp_ns;
        right_cuvslam_image->camera_index = 1;  // Right camera index
        right_cuvslam_image->pitch = processed_right_image_.step;
        right_cuvslam_image->image_encoding = right_encoding;
        
        cuvslam_images.push_back(right_cuvslam_image);
    } 
    else 
    {
        UERROR("No right image available for stereo camera");
        return false;
    }
    
    // Validate prepared images
    for(size_t i = 0; i < cuvslam_images.size(); ++i) {
        const auto& img = cuvslam_images[i];
        if(img->width <= 0 || img->height <= 0) {
            UERROR("Prepared image %d has invalid dimensions: %dx%d", 
                   static_cast<int>(i), img->width, img->height);
            return false;
        }
        if(img->pixels == nullptr) {
            UERROR("Prepared image %d has null pixel data!", static_cast<int>(i));
            return false;
        }
        if(img->camera_index < 0 || img->camera_index >= static_cast<int>(cuvslam_cameras.size())) {
            UERROR("Prepared image %d has invalid camera index %d (max: %d)", 
                   static_cast<int>(i), img->camera_index, static_cast<int>(cuvslam_cameras.size()-1));
            return false;
        }
    }
    
    // Synchronize all async GPU operations before returning
    if(!synchronizeGpuOperations(cuda_stream_)) {
        return false;
    }
    
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

// Helper function to convert cuVSLAM pose to tf2::Transform
// Taken directly from isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/cuvslam_ros_conversion.cpp
tf2::Transform FromcuVSLAMPose(const CUVSLAM_Pose & cuvslam_pose)
{
  const auto & r = cuvslam_pose.r;
  const auto & t = cuvslam_pose.t;
  // tf2::Matrix3x3 is row major and cuVSLAM rotation mat is column major.
  const tf2::Matrix3x3 rotation(r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]);
  const tf2::Vector3 translation(t[0], t[1], t[2]);

  return tf2::Transform(rotation, translation);
}

// Helper function to change basis from frame source to frame target
// Taken directly from isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/cuvslam_ros_conversion.cpp
tf2::Transform ChangeBasis(
  const tf2::Transform & target_pose_source, const tf2::Transform & source_pose_source)
{
  return target_pose_source * source_pose_source * target_pose_source.inverse();
}


// Helper function to convert camera pose from RTABMap to cuVSLAM coordinate system
// Based on Isaac ROS FillExtrinsics implementation
CUVSLAM_Pose convertCameraPoseToCuVSLAM(const Transform & rtabmap_camera_pose) {
    // Convert RTABMap Transform to tf2::Transform
    tf2::Transform tf2_camera_pose = rtabmapToTf2(rtabmap_camera_pose);
    
    // Apply coordinate system transformation:
    // RTABMap robot frame -> cuVSLAM robot frame -> cuVSLAM camera frame
    // This follows the same pattern as Isaac ROS FillExtrinsics
    const tf2::Transform tf2_camera_pose_cuvslam = 
        cuvslam_pose_canonical * tf2_camera_pose * optical_pose_cuvslam;
    
    return TocuVSLAMPose(tf2_camera_pose_cuvslam);
}

// Helper function to convert IMU pose from RTABMap to cuVSLAM coordinate system
// Based on Isaac ROS IMU handling - does NOT apply optical transform
CUVSLAM_Pose convertImuPoseToCuVSLAM(const Transform & rtabmap_imu_pose) {
    // Convert RTABMap Transform to tf2::Transform
    tf2::Transform tf2_imu_pose = rtabmapToTf2(rtabmap_imu_pose);
    
    // Apply coordinate system transformation:
    // RTABMap robot frame -> cuVSLAM robot frame
    // This follows the same pattern as Isaac ROS IMU handling
    const tf2::Transform tf2_imu_pose_cuvslam = cuvslam_pose_canonical * tf2_imu_pose;
    
    return TocuVSLAMPose(tf2_imu_pose_cuvslam);
}

/*
Convert cuVSLAM pose to RTAB-Map Transform.
CuVSLAM uses column-major rotation matrix, RTAB-Map uses row-major.
Applies coordinate system transformation from cuVSLAM frame to RTAB-Map frame.
*/
Transform convertCuVSLAMPose(const CUVSLAM_Pose * cuvslam_pose) 
{
    // Convert cuVSLAM pose to tf2::Transform (cuVSLAM coordinate system)
    tf2::Transform tf2_odom_pose_cuvslam = FromcuVSLAMPose(*cuvslam_pose);
    
    // Apply coordinate system transformation from cuVSLAM to RTAB-Map (ROS conventions)
    // Following Isaac ROS pattern exactly
    tf2::Transform rtabmap_tf2_transform = ChangeBasis(canonical_pose_cuvslam, tf2_odom_pose_cuvslam);
    
    // Convert tf2::Transform to RTAB-Map Transform using the proper constructor
    const tf2::Matrix3x3 & rtabmap_rotation = rtabmap_tf2_transform.getBasis();
    const tf2::Vector3 & rtabmap_translation = rtabmap_tf2_transform.getOrigin();
    
    // Use RTAB-Map Transform constructor with rotation matrix and translation
    Transform rtabmap_transform(
        rtabmap_rotation[0][0], rtabmap_rotation[0][1], rtabmap_rotation[0][2], rtabmap_translation.x(),  // r11, r12, r13, tx
        rtabmap_rotation[1][0], rtabmap_rotation[1][1], rtabmap_rotation[1][2], rtabmap_translation.y(),  // r21, r22, r23, ty
        rtabmap_rotation[2][0], rtabmap_rotation[2][1], rtabmap_rotation[2][2], rtabmap_translation.z()   // r31, r32, r33, tz
    );
    
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
    
    // Ensure diagonal elements are positive and finite (RTAB-Map requirement)
    for(int i = 0; i < 6; i++)
    {
        double diag_val = cv_covariance.at<double>(i, i);
        if(!std::isfinite(diag_val) || diag_val <= 0.0)
        {
            UWARN("Diagonal element of covariance is not finite or positive, proceeding with default infinite covariance");
            cv_covariance.at<double>(i, i) = 9999.0;  // High uncertainty
        }
    }
    
    return cv_covariance;
}

#endif // RTABMAP_CUVSLAM

} // namespace rtabmap

