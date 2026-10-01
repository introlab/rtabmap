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

#pragma once

#include "rtabmap/core/Transform.h"
#include <string>

namespace rtabmap
{

/**
 * @class SensorCaptureInfo
 * @brief Metadata structure for sensor data capture and processing
 * 
 * SensorCaptureInfo contains metadata about sensor data capture, including timing
 * information for various processing steps, camera identification, and odometry
 * information. This information is typically attached to SensorEvent objects to
 * provide context about how the sensor data was captured and processed.
 * 
 * The class tracks timing for various processing stages:
 * - **Capture**: Time to capture data from the sensor
 * - **Processing**: Time for various image processing steps (deskewing, disparity,
 *   mirroring, exposure compensation, decimation, histogram equalization, etc.)
 * - **Depth processing**: Time for depth-related operations (scan from depth,
 *   undistort depth, bilateral filtering)
 * - **Total**: Total processing time
 * 
 * Odometry information includes:
 * - **Pose**: The odometry pose at the time of capture
 * - **Covariance**: Uncertainty of the odometry twist (6x6 matrix)
 * - **Velocity**: 6DOF velocity vector [vx, vy, vz, vroll, vpitch, vyaw]
 * 
 * @note All timing values are in seconds (float).
 * @note The odometry covariance matrix is initialized to identity by default.
 * 
 * @see SensorEvent
 * @see SensorData
 */
class SensorCaptureInfo
{

public:
	/**
	 * @brief Default constructor
	 * 
	 * Initializes all fields to default values:
	 * - cameraName: empty string
	 * - id: 0
	 * - stamp: 0.0
	 * - All timing fields: 0.0f
	 * - odomPose: null transform
	 * - odomCovariance: 6x6 identity matrix
	 * - odomVelocity: empty vector
	 */
	SensorCaptureInfo() :
		cameraName(""),
		id(0),
		stamp(0.0),
		timeCapture(0.0f),
		timeDeskewing(0.0f),
		timeDisparity(0.0f),
		timeMirroring(0.0f),
		timeStereoExposureCompensation(0.0f),
		timeImageDecimation(0.0f),
		timeHistogramEqualization(0.0f),
		timeScanFromDepth(0.0f),
		timeUndistortDepth(0.0f),
		timeBilateralFiltering(0.0f),
		timeTotal(0.0f),
		odomCovariance(cv::Mat::eye(6,6,CV_64FC1))
	{
	}
	
	/**
	 * @brief Virtual destructor
	 */
	virtual ~SensorCaptureInfo() {}

	std::string cameraName; ///< Camera/sensor name identifier
	int id; ///< Capture ID (typically matches SensorData ID)
	double stamp; ///< Timestamp in seconds (typically matches SensorData stamp)
	
	// Timing information (all in seconds)
	float timeCapture; ///< Time to capture data from the sensor (seconds)
	float timeDeskewing; ///< Time for laser scan deskewing (seconds)
	float timeDisparity; ///< Time to compute stereo disparity (seconds)
	float timeMirroring; ///< Time for image mirroring/flipping (seconds)
	float timeStereoExposureCompensation; ///< Time for stereo exposure compensation (seconds)
	float timeImageDecimation; ///< Time for image decimation/downsampling (seconds)
	float timeHistogramEqualization; ///< Time for histogram equalization (seconds)
	float timeScanFromDepth; ///< Time to convert depth image to laser scan (seconds)
	float timeUndistortDepth; ///< Time to undistort depth image (seconds)
	float timeBilateralFiltering; ///< Time for bilateral filtering of depth (seconds)
	float timeTotal; ///< Total processing time (seconds)
	
	// Odometry information
	Transform odomPose; ///< Odometry pose at the time of capture (in odometry coordinate frame)
	cv::Mat odomCovariance; ///< Odometry twist covariance matrix (6x6, CV_64FC1). Default: identity matrix
	std::vector<float> odomVelocity; ///< 6DOF odometry velocity [vx, vy, vz, vroll, vpitch, vyaw] (m/s, rad/s)
};

/**
 * @deprecated Use SensorCaptureInfo instead
 * @brief Backward compatibility typedef
 * 
 * CameraInfo is deprecated. Use SensorCaptureInfo instead.
 */
RTABMAP_DEPRECATED typedef SensorCaptureInfo CameraInfo;

} // namespace rtabmap
