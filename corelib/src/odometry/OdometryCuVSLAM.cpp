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
                       CUVSLAM_Tracker * & cuvslam_handle,
                       std::vector<size_t> & gpu_left_image_sizes,
                       std::vector<size_t> & gpu_right_image_sizes);
    
CUVSLAM_Configuration * CreateConfiguration(const CUVSLAM_Pose * cv_base_link_pose_cv_imu);

bool prepareImages(const SensorData & data, 
                    std::vector<CUVSLAM_Image> & cuvslam_images,
                    std::vector<uint8_t *> & gpu_left_image_data,
                    std::vector<uint8_t *> & gpu_right_image_data,
                    std::vector<size_t> & gpu_left_image_sizes,
                    std::vector<size_t> & gpu_right_image_sizes,
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
    Odometry(parameters)
#ifdef RTABMAP_CUVSLAM
    ,
    cuvslam_handle_(nullptr),
    initialized_(false),
    lost_(false),
    previous_pose_(Transform::getIdentity()),
    last_timestamp_(-1.0),
    gpu_left_image_data_(),
    gpu_right_image_data_(),
    gpu_left_image_sizes_(),
    gpu_right_image_sizes_()
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
#endif
}

void OdometryCuVSLAM::reset(const Transform & initialPose)
{
    Odometry::reset(initialPose);
    
#ifdef RTABMAP_CUVSLAM
    UWARN("RESET: cuvSLAM reset hit _______________________________");
    UWARN("RESET: cuvslam_handle_ before destroy = %p", cuvslam_handle_);
    UWARN("RESET: initialized_ before reset = %s", initialized_ ? "true" : "false");
    
    if(cuvslam_handle_)
    {
        UWARN("RESET: Destroying cuVSLAM tracker...");
        CUVSLAM_DestroyTracker(cuvslam_handle_);
        cuvslam_handle_ = nullptr;
        UWARN("RESET: cuvslam_handle_ set to nullptr");
    }
    else
    {
        UWARN("RESET: cuvslam_handle_ was already nullptr");
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
    
    // Reset our internal state variables
    gpu_left_image_sizes_.clear();
    gpu_right_image_sizes_.clear();
    initialized_ = false;
    lost_ = false;
    previous_pose_ = initialPose;
    last_timestamp_ = -1.0;
    
    UWARN("RESET: State variables reset - initialized_ = %s, lost_ = %s", 
          initialized_ ? "true" : "false", lost_ ? "true" : "false");
    UWARN("RESET: cuvslam_handle_ after reset = %p", cuvslam_handle_);
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
        UWARN("=== INITIALIZATION CHECK: initialized_ = false, calling initializeCuVSLAM ===");
        UWARN("INIT CHECK: cuvslam_handle_ before init = %p", cuvslam_handle_);
        
        // Track initialization timing
        UTimer init_timer;
        init_timer.start();
        
        if(!initializeCuVSLAM(
            data, 
            cuvslam_handle_,
            gpu_left_image_sizes_,
            gpu_right_image_sizes_))
        {
            UERROR("Failed to initialize cuVSLAM tracker");
            return transform;
        }
        
        double init_time = init_timer.ticks();
        UWARN("INIT CHECK: cuvslam_handle_ after init = %p", cuvslam_handle_);
        UWARN("INIT CHECK: Initialization took %.3f seconds", init_time);
        initialized_ = true;
        UWARN("INIT CHECK: initialized_ set to true");
        
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
        gpu_left_image_sizes_,
        gpu_right_image_sizes_,
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
        UWARN("cuVSLAM tracking LOST ___________________________");
        lost_ = true;
        if(info)
        {
            info->type = 1; // Tracking lost
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0; // High uncertainty
            info->timeEstimation = timer.ticks();
        }

        transform.setNull();
        last_timestamp_ = data.stamp();
        
        return transform;
    }
    else if(vo_status != CUVSLAM_SUCCESS)
    {
        // Provide more specific error information
        const char* error_msg = "Unknown error";
        switch(vo_status) {
            case 2: error_msg = "CUVSLAM_INVALID_PARAMETER"; break;
            case 3: error_msg = "CUVSLAM_INVALID_IMAGE_FORMAT or CUVSLAM_INVALID_CAMERA_CONFIG"; break;
            case 4: error_msg = "CUVSLAM_GPU_MEMORY_ERROR"; break;
            case 5: error_msg = "CUVSLAM_INITIALIZATION_ERROR"; break;
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
        last_timestamp_ = data.stamp();
        
        return transform;
    }
    else
    {
        UWARN("cuVSLAM tracking SUCCESS ___________________________");

		// extract 2D VO observations for visualization
		CUVSLAM_ObservationVector observation_vector{};

		CUVSLAM_Status observation_status = CUVSLAM_GetLastLeftObservations(cuvslam_handle_, &observation_vector);
		UWARN("GetLastLeftObservations: status=%d, num=%u, max=%u, ptr=%p",
			observation_status,
			(unsigned)observation_vector.num,
			(unsigned)observation_vector.max,
			(void*)observation_vector.observations);
		
		if(observation_status != CUVSLAM_SUCCESS) {
			UWARN("Failed to extract observations from cuVSLAM (status=%d, num=%u, max=%u, ptr=%p)",
				observation_status,
				(unsigned)observation_vector.num,
				(unsigned)observation_vector.max,
				(void*)observation_vector.observations);
			if(info)
			{
				info->newCorners.clear();
			}
		}
		else
		{
			UWARN("Observation extraction success: num=%u, max=%u, ptr=%p",
				(unsigned)observation_vector.num,
				(unsigned)observation_vector.max,
				(void*)observation_vector.observations);
			
			if(observation_vector.num > 0 && observation_vector.observations)
			{
				std::vector<cv::Point2f> newCorners;
				newCorners.reserve(observation_vector.num);
				for(uint32_t i = 0; i < observation_vector.num; ++i)
				{
					const CUVSLAM_Observation & observation = observation_vector.observations[i];
					newCorners.emplace_back(observation.u, observation.v);
				}

				uint32_t sample_count = observation_vector.num < 5 ? observation_vector.num : 5;
				for(uint32_t i = 0; i < sample_count; ++i)
				{
					const CUVSLAM_Observation & o = observation_vector.observations[i];
					UWARN("Observation[%u]: id=%d, u=%.2f, v=%.2f", (unsigned)i, o.id, o.u, o.v);
				}

				if(info)
				{
					info->newCorners = std::move(newCorners);
				}
			}
			else if(info)
			{
				UWARN("Observation extraction returned empty or null data.");
				info->newCorners.clear();
			}
		}

        // Clean up observation vector
        
        if(info)
        {
            cv::Mat covMat = convertCuVSLAMCovariance(vo_pose_estimate.covariance);
            info->type = 0;  // Success
            info->reg.covariance = covMat;
            info->timeEstimation = timer.ticks();
        }
        
        // On SUCCESS, validate pose values and treat non-finite/huge values as lost
        bool pose_corrupted = false;
        std::string corruption_details = "";
        for(int i = 0; i < 3; i++) {
            if(!std::isfinite(vo_pose_estimate.pose.t[i]) || std::abs(vo_pose_estimate.pose.t[i]) > 1000.0) {
                corruption_details += "T[" + std::to_string(i) + "]=" + std::to_string(vo_pose_estimate.pose.t[i]) + " ";
                pose_corrupted = true;
            }
        }
        for(int i = 0; i < 9; i++) {
            if(!std::isfinite(vo_pose_estimate.pose.r[i]) || std::abs(vo_pose_estimate.pose.r[i]) > 1000.0) {
                corruption_details += "R[" + std::to_string(i) + "]=" + std::to_string(vo_pose_estimate.pose.r[i]) + " ";
                pose_corrupted = true;
            }
        }
        if(pose_corrupted) {
            UWARN("CORRUPTED POSE DETECTED - Status: %d, Details: %s", vo_status, corruption_details.c_str());
            UWARN("Full pose - Translation: [%.6f, %.6f, %.6f]", 
                   vo_pose_estimate.pose.t[0], vo_pose_estimate.pose.t[1], vo_pose_estimate.pose.t[2]);
            UWARN("Full pose - Rotation: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                   vo_pose_estimate.pose.r[0], vo_pose_estimate.pose.r[1], vo_pose_estimate.pose.r[2],
                   vo_pose_estimate.pose.r[3], vo_pose_estimate.pose.r[4], vo_pose_estimate.pose.r[5],
                   vo_pose_estimate.pose.r[6], vo_pose_estimate.pose.r[7], vo_pose_estimate.pose.r[8]);
            
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
    }
    
    // Convert cuVSLAM pose to RTAB-Map Transform
    Transform current_pose = convertCuVSLAMPose(&vo_pose_estimate.pose);
    
    // Log raw cuVSLAM pose and converted pose on SUCCESS
    UWARN("VO SUCCESS: cuVSLAM raw pose - T: [%.6f, %.6f, %.6f]",
        vo_pose_estimate.pose.t[0], vo_pose_estimate.pose.t[1], vo_pose_estimate.pose.t[2]);
    UWARN("VO SUCCESS: cuVSLAM raw pose - R: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
        vo_pose_estimate.pose.r[0], vo_pose_estimate.pose.r[1], vo_pose_estimate.pose.r[2],
        vo_pose_estimate.pose.r[3], vo_pose_estimate.pose.r[4], vo_pose_estimate.pose.r[5],
        vo_pose_estimate.pose.r[6], vo_pose_estimate.pose.r[7], vo_pose_estimate.pose.r[8]);
    UWARN("VO SUCCESS: converted pose - R: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f], T: [%.6f, %.6f, %.6f]",
        current_pose.r11(), current_pose.r12(), current_pose.r13(),
        current_pose.r21(), current_pose.r22(), current_pose.r23(),
        current_pose.r31(), current_pose.r32(), current_pose.r33(),
        current_pose.x(), current_pose.y(), current_pose.z());
    
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
        UWARN("TRACKING: Current pose is identity - returning null to avoid reset cascade");
        transform.setNull();
    }

    UWARN("VO SUCCESS: incremental transform - R: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f], T: [%.6f, %.6f, %.6f]",
        transform.r11(), transform.r12(), transform.r13(),
        transform.r21(), transform.r22(), transform.r23(),
        transform.r31(), transform.r32(), transform.r33(),
        transform.x(), transform.y(), transform.z());

    // Log time between odom publishing
    // const double time_between_odom_publishing = data.stamp() - last_timestamp_;
    // UWARN("Time between odom publishing: %fs", time_between_odom_publishing);
    // const double odom_publishing_frequency = 1.0 / time_between_odom_publishing;
    // UWARN("Odom publishing frequency: %fHz", odom_publishing_frequency);

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
                       CUVSLAM_Tracker * & cuvslam_handle,
                       std::vector<size_t> & gpu_left_image_sizes,
                       std::vector<size_t> & gpu_right_image_sizes)
{    
    // Minimal timing logs for initialization
    UTimer init_timer; init_timer.start();    

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
        left_camera_params[0] = static_cast<float>(leftModel.cx());
        left_camera_params[1] = static_cast<float>(leftModel.cy());
        left_camera_params[2] = static_cast<float>(leftModel.fx());
        left_camera_params[3] = static_cast<float>(leftModel.fy());
        left_camera.parameters = left_camera_params;
        left_camera.distortion_model = left_distortion_model;
        left_camera.num_parameters = 4;
        Transform left_pose = leftModel.localTransform();
        left_camera.pose = convertCameraPoseToCuVSLAM(left_pose);
        cuvslam_camera_objects.push_back(left_camera);
        
        CUVSLAM_Camera right_camera;
        right_camera.width = rightModel.imageWidth();
        right_camera.height = rightModel.imageHeight();
        right_camera_params[0] = static_cast<float>(rightModel.cx());
        right_camera_params[1] = static_cast<float>(rightModel.cy());
        right_camera_params[2] = static_cast<float>(rightModel.fx());
        right_camera_params[3] = static_cast<float>(rightModel.fy());
        right_camera.parameters = right_camera_params;
        right_camera.distortion_model = right_distortion_model;
        right_camera.num_parameters = 4;
        
        // Set right camera pose using stereo baseline (proper stereo configuration)
        double baseline = stereoModel.baseline();
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
    if(!data.imu().empty()) 
    {
        configuration->enable_imu_fusion = 1;
    }

    // Configure multicam mode
    gpu_left_image_sizes.resize(data.stereoCameraModels().size(), 0);
    gpu_right_image_sizes.resize(data.stereoCameraModels().size(), 0);

    if(data.stereoCameraModels().size() > 1)
    {
        configuration->multicam_mode = 1;
        UWARN("MULTI-CAMERA SUPPORT: Detected %zu stereo pairs - enabling multi-camera mode", data.stereoCameraModels().size());
    }
    else
    {
        UWARN("SINGLE-CAMERA MODE: Using single stereo pair configuration");
    }
    
    // Create tracker
    CUVSLAM_TrackerHandle tracker_handle;
    UTimer create_timer; create_timer.start();
    CUVSLAM_Status status = CUVSLAM_CreateTracker(&tracker_handle, camera_rig, configuration);
    double create_time_s = create_timer.ticks();
    UWARN("TIMING: CUVSLAM_CreateTracker() took %.3f s", create_time_s);
    cuvslam_handle = tracker_handle;

    // ============================================================================
    // COMPREHENSIVE POST-PROCESSING LOGGING
    // ============================================================================
    UWARN("=== cuVSLAM INITIALIZATION DATA PROCESSING COMPLETE ===");
    
    // 3. ALL DATA PROCESSED DURING INIT
    UWARN("3. SENSOR DATA INFO:");
    UWARN("   - Timestamp: %f", data.stamp());
    UWARN("   - Left image: %s (channels: %d, size: %dx%d)", 
          data.imageRaw().empty() ? "EMPTY" : "OK",
          data.imageRaw().empty() ? 0 : data.imageRaw().channels(),
          data.imageRaw().empty() ? 0 : data.imageRaw().cols,
          data.imageRaw().empty() ? 0 : data.imageRaw().rows);
    UWARN("   - Right image: %s (channels: %d, size: %dx%d)", 
          data.rightRaw().empty() ? "EMPTY" : "OK",
          data.rightRaw().empty() ? 0 : data.rightRaw().channels(),
          data.rightRaw().empty() ? 0 : data.rightRaw().cols,
          data.rightRaw().empty() ? 0 : data.rightRaw().rows);
    UWARN("   - Stereo camera models count: %zu", data.stereoCameraModels().size());
    UWARN("   - IMU data: %s", data.imu().empty() ? "EMPTY" : "AVAILABLE");
    
    UWARN("3. CAMERA CONFIGURATION:");
    UWARN("   - Number of cameras in rig: %zu", cuvslam_camera_objects.size());
    for(size_t i = 0; i < cuvslam_camera_objects.size(); ++i) {
        const CUVSLAM_Camera & cam = cuvslam_camera_objects[i];
        UWARN("   - Camera %zu: %dx%d, distortion: %s, params: [%.2f, %.2f, %.2f, %.2f]",
              i, cam.width, cam.height, cam.distortion_model,
              cam.parameters[0], cam.parameters[1], cam.parameters[2], cam.parameters[3]);
        
        // Log camera pose (extrinsics)
        UWARN("   - Camera %zu pose: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
              i, cam.pose.r[0], cam.pose.r[1], cam.pose.r[2], cam.pose.t[0],
              cam.pose.r[3], cam.pose.r[4], cam.pose.r[5], cam.pose.t[1],
              cam.pose.r[6], cam.pose.r[7], cam.pose.r[8], cam.pose.t[2]);
    }
    
    // Log stereo baseline information
    if(data.stereoCameraModels().size() > 0) {
        const StereoCameraModel & stereoModel = data.stereoCameraModels()[0];
        UWARN("3. STEREO BASELINE INFO:");
        UWARN("   - Baseline: %.6f meters", stereoModel.baseline());
        UWARN("   - Left camera local transform: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
              stereoModel.left().localTransform().r11(), stereoModel.left().localTransform().r12(), stereoModel.left().localTransform().r13(), stereoModel.left().localTransform().x(),
              stereoModel.left().localTransform().r21(), stereoModel.left().localTransform().r22(), stereoModel.left().localTransform().r23(), stereoModel.left().localTransform().y(),
              stereoModel.left().localTransform().r31(), stereoModel.left().localTransform().r32(), stereoModel.left().localTransform().r33(), stereoModel.left().localTransform().z());
        UWARN("   - Right camera local transform: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
              stereoModel.right().localTransform().r11(), stereoModel.right().localTransform().r12(), stereoModel.right().localTransform().r13(), stereoModel.right().localTransform().x(),
              stereoModel.right().localTransform().r21(), stereoModel.right().localTransform().r22(), stereoModel.right().localTransform().r23(), stereoModel.right().localTransform().y(),
              stereoModel.right().localTransform().r31(), stereoModel.right().localTransform().r32(), stereoModel.right().localTransform().r33(), stereoModel.right().localTransform().z());
    }
    
    UWARN("3. CONFIGURATION SETTINGS:");
    UWARN("   - Multicam mode: %d", configuration->multicam_mode);
    UWARN("   - Use motion model: %d", configuration->use_motion_model);
    UWARN("   - Use denoising: %d", configuration->use_denoising);
    UWARN("   - Use GPU: %d", configuration->use_gpu);
    UWARN("   - Horizontal stereo: %d", configuration->horizontal_stereo_camera);
    UWARN("   - Enable SLAM: %d", configuration->enable_localization_n_mapping);
    UWARN("   - Enable observations export: %d", configuration->enable_observations_export);
    UWARN("   - Enable landmarks export: %d", configuration->enable_landmarks_export);
    UWARN("   - Enable reading SLAM internals: %d", configuration->enable_reading_slam_internals);
    UWARN("   - Enable IMU fusion: %d", configuration->enable_imu_fusion);
    UWARN("   - Max frame delta: %f ms", configuration->max_frame_delta_ms);
    
    UWARN("3. TRACKER CREATION:");
    UWARN("   - CUVSLAM_CreateTracker status: %d", status);
    UWARN("   - Tracker handle created: %p", tracker_handle);
    UWARN("   - Tracker handle assigned: %p", cuvslam_handle);

    delete configuration;
    delete camera_rig;

    if(status != CUVSLAM_SUCCESS)
    {
        UERROR("Failed to create cuVSLAM tracker: %d", status);
        UWARN("=== cuVSLAM INITIALIZATION FAILED ===");
        return false;
    }
    
    // Enable cuVSLAM API debug logging
    CUVSLAM_SetVerbosity(10);  // Set to maximum verbosity level (0=none, 1=errors, 2=warnings, 3=info)
    UWARN("cuVSLAM verbosity set to level 3 (maximum - informational messages)");
    
    // 4. STATE OF TRACKER AT THE BOTTOM
    UWARN("4. FINAL TRACKER STATE:");
    UWARN("   - cuvslam_handle = %p", cuvslam_handle);
    UWARN("   - cuvslam_handle is %s", cuvslam_handle ? "NOT NULL" : "NULL");
    UWARN("   - Initialization SUCCESS: %s", cuvslam_handle ? "TRUE" : "FALSE");
    double total_time_s = init_timer.ticks();
    UWARN("TIMING: initializeCuVSLAM() total time %.3f s", total_time_s);
    UWARN("=== cuVSLAM INITIALIZATION COMPLETED SUCCESSFULLY ===");
    
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
    
    // Will override multicam mode if we have multiple stereo camera models
    configuration->multicam_mode = 0;
    
    // Core Visual Odometry Settings 
    configuration->use_motion_model = 1;                 // Enable motion model for better tracking
    configuration->use_denoising = 0;                    // Disable denoising by default
    configuration->use_gpu = 1;                          // Use GPU acceleration
    configuration->horizontal_stereo_camera = 1;         // Stereo camera configuration

    configuration->enable_observations_export = 1;       // Export observations for external reading
    
    // SLAM Enabled (required for observation buffer allocation) 
    // TODO: Enable SLAM properly
    configuration->enable_localization_n_mapping = 1;    // Enable SLAM for observation buffers
    configuration->enable_landmarks_export = 0;          // SLAM feature (optional)
    configuration->enable_reading_slam_internals = 0;    // SLAM feature (optional)
    
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
    // configuration->max_frame_delta_ms = 100.0;             // Maximum frame interval (100ms default)
    
    // SLAM-specific parameters (now enabled for observation buffers)
    configuration->planar_constraints = 0;               // No planar constraints
    configuration->slam_throttling_time_ms = 0;          // No SLAM throttling
    configuration->slam_max_map_size = 0;              // Reasonable map size for real-time
    configuration->slam_sync_mode = 0;                   // Async SLAM mode (better for real-time)

    // configuration->debug_dump_directory = "/home/felix/Documents/cuvslam_debug";
    
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
                   std::vector<uint8_t *> & gpu_left_image_data,
                   std::vector<uint8_t *> & gpu_right_image_data,
                   std::vector<size_t> & gpu_left_image_sizes,
                   std::vector<size_t> & gpu_right_image_sizes,
                   cudaStream_t & cuda_stream)
{
    // Convert timestamp to nanoseconds (cuVSLAM expects nanoseconds)
    int64_t timestamp_ns = static_cast<int64_t>(data.stamp() * 1000000000.0);

    // Horizontally stithed images recieved by RTAB-Map
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
    
    // process left image
    if(left_image.channels() == 1) {
        processed_left_image = left_image.clone();
        left_encoding = CUVSLAM_ImageEncoding::MONO8;
    } else if(left_image.channels() == 3) {
        cv::cvtColor(left_image, processed_left_image, cv::COLOR_BGR2RGB);
        left_encoding = CUVSLAM_ImageEncoding::RGB8;
    } else {
        UERROR("Unsupported left image format: %d channels", left_image.channels());
        return false;
    }

    // Process right image
    if(right_image.channels() == 1) {
        processed_right_image = right_image.clone();
        right_encoding = CUVSLAM_ImageEncoding::MONO8;
    } else if(right_image.channels() == 3) {
        cv::cvtColor(right_image, processed_right_image, cv::COLOR_BGR2RGB);
        right_encoding = CUVSLAM_ImageEncoding::RGB8;
    } else {
        UERROR("Unsupported right image format: %d channels", right_image.channels());
        return false;
    }

    // Resize GPU memory vectors to accommodate all cameras
    size_t stereo_pairs_count = data.stereoCameraModels().size(); // cameras per stereo pair
    gpu_left_image_data.resize(stereo_pairs_count, nullptr);
    gpu_right_image_data.resize(stereo_pairs_count, nullptr);
    
    UWARN("MULTI-CAMERA PROCESSING: Processing %zu stereo pairs", 
          data.stereoCameraModels().size());
    
    int camera_index = 0;
    int stereo_index = 0;
    for(const StereoCameraModel & model : data.stereoCameraModels()) {
        // slice out the image for the current stereo pair
        // Assumes all images have the same width and height
        int left_image_width = model.left().imageWidth();
        int right_image_width = model.right().imageWidth();
        int left_image_height = model.left().imageHeight();
        int right_image_height = model.right().imageHeight();

        cv::Mat left_image_slice = processed_left_image(cv::Rect(stereo_index * left_image_width, 0, left_image_width, left_image_height));
        cv::Mat right_image_slice = processed_right_image(cv::Rect(stereo_index * right_image_width, 0, right_image_width, right_image_height));
        
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

