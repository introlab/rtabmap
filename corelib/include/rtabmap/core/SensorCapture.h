/*
Copyright (c) 2010-2022, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither names of its contributors may be used to endorse or promote products
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

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include <rtabmap/core/SensorCaptureInfo.h>
#include "rtabmap/core/SensorData.h"
#include <set>
#include <stack>
#include <list>
#include <vector>

class UDirectory;
class UTimer;

namespace rtabmap
{

/**
 * @class SensorCapture
 * @brief Abstract base class for sensor data capture (cameras, lidars, etc.)
 * 
 * SensorCapture provides a unified interface for capturing sensor data from various
 * sensor types including cameras (RGB-D, stereo, mono) and lidars. It handles frame
 * rate control, local transform management, and provides a common API for sensor
 * initialization and data capture.
 * 
 * The class implements the Template Method pattern:
 * - `takeData()` handles frame rate control and timing, then calls the pure virtual
 *   `captureData()` method implemented by derived classes
 * - Derived classes must implement `captureData()` to perform the actual sensor capture
 * 
 * Key features:
 * - **Frame rate control**: Automatic throttling to maintain target frame rate
 * - **Local transform**: Transform from robot base frame to sensor frame
 * - **Capture timing**: Tracks capture time and provides it via SensorCaptureInfo
 * - **Sequence IDs**: Automatic sequence number generation for captured data
 * 
 * Derived classes include:
 * - **Camera**: Base class for all camera types (RGB-D, stereo, mono, file readers, etc.)
 * - **Lidar**: Base class for lidar sensors
 * 
 * @note This is an abstract class. Use concrete implementations like CameraRGBD,
 *       CameraStereo, LidarVLP16, etc., or create custom derived classes.
 * 
 * @see Camera
 * @see Lidar
 * @see SensorData
 * @see SensorCaptureInfo
 * @see SensorCaptureThread
 */
class RTABMAP_CORE_EXPORT SensorCapture
{
public:
	/**
	 * @brief Virtual destructor
	 */
	virtual ~SensorCapture();
	
	/**
	 * @brief Captures sensor data with frame rate control
	 * 
	 * This method handles frame rate throttling and timing, then calls the
	 * pure virtual `captureData()` method to perform the actual capture.
	 * 
	 * The method:
	 * - Enforces the target frame rate by sleeping if necessary
	 * - Measures capture time and stores it in SensorCaptureInfo
	 * - Assigns sequence IDs to captured data
	 * - Warns if the target frame rate cannot be reached
	 * 
	 * @param info Optional pointer to SensorCaptureInfo to fill with capture metadata
	 *             (ID, timestamp, capture time). If null, no info is filled.
	 * @return SensorData containing the captured sensor data
	 * 
	 * @note If frame rate is 0, data is captured as fast as possible without throttling.
	 * @note The returned SensorData should have rectified images if calibration was loaded.
	 * 
	 * @see captureData()
	 */
	SensorData takeData(SensorCaptureInfo * info = 0);

	/**
	 * @brief Initializes the sensor
	 * 
	 * Pure virtual method that must be implemented by derived classes to initialize
	 * the sensor hardware or data source. This typically involves:
	 * - Opening device connections or file streams
	 * - Loading camera calibration parameters
	 * - Configuring sensor settings
	 * 
	 * @param calibrationFolder Directory path where calibration files are located
	 *                          (default: current directory ".")
	 * @param cameraName Base name of the camera for loading calibration files
	 *                   (default: empty string)
	 * @return True if initialization was successful, false otherwise
	 * 
	 * @note Must be called before calling takeData() or captureData()
	 */
	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "") = 0;
	
	/**
	 * @brief Returns the sensor's serial number or unique identifier
	 * 
	 * Pure virtual method that must be implemented by derived classes to return
	 * a unique identifier for the sensor (e.g., device serial number, file path,
	 * or other identifier).
	 * 
	 * @return String identifier for the sensor
	 */
	virtual std::string getSerial() const = 0;
	
	/**
	 * @brief Checks if the sensor provides odometry poses
	 * 
	 * Some sensors (e.g., visual-inertial cameras) can provide pose estimates
	 * directly. This method indicates whether the sensor supports pose queries.
	 * 
	 * @return True if the sensor provides odometry poses, false otherwise
	 * 
	 * @note Default implementation returns false. Derived classes should override
	 *       if they support pose estimation.
	 * 
	 * @see getPose()
	 */
	virtual bool odomProvided() const { return false; }
	
	/**
	 * @brief Gets the sensor's pose estimate at a specific timestamp
	 * 
	 * Queries the sensor for its pose estimate at the given timestamp. This is
	 * typically used for sensors that provide visual-inertial odometry or other
	 * pose estimation capabilities.
	 * 
	 * @param stamp Timestamp in seconds for which to query the pose
	 * @param[out] pose Output transform representing the sensor pose
	 * @param[out] covariance Output covariance matrix (6x6) representing twist uncertainty
	 * @param maxWaitTime Maximum time in seconds to wait for pose data (default: 0.06)
	 * @return True if pose was successfully retrieved, false otherwise
	 * 
	 * @note Default implementation returns false. Derived classes should override
	 *       if they support pose estimation.
	 * 
	 * @see odomProvided()
	 */
	virtual bool getPose(double stamp, Transform & pose, cv::Mat & covariance, double maxWaitTime = 0.06) { return false; }

	// Getters
	
	/**
	 * @brief Returns the target frame rate
	 * @return Frame rate in Hz (0 = unlimited, capture as fast as possible)
	 */
	float getFrameRate() const {return _frameRate;}
	
	/**
	 * @brief Returns the local transform from base frame to sensor frame
	 * @return Const reference to the local transform
	 */
	const Transform & getLocalTransform() const {return _localTransform;}

	// Setters
	
	/**
	 * @brief Sets the target frame rate
	 * 
	 * Controls how often `takeData()` will capture data. The method will throttle
	 * captures to maintain the target rate.
	 * 
	 * @param frameRate Target frame rate in Hz (0 = unlimited, capture as fast as possible)
	 * 
	 * @note Setting frame rate to 0 disables throttling and captures as fast as possible.
	 */
	void setFrameRate(float frameRate) {_frameRate = frameRate;}
	
	/**
	 * @brief Sets the local transform from base frame to sensor frame
	 * 
	 * The local transform represents the pose of the sensor relative to the robot's
	 * base frame. This is used to transform sensor data into the robot's coordinate system.
	 * 
	 * @param localTransform Transform from base frame to sensor frame
	 */
	void setLocalTransform(const Transform & localTransform) {_localTransform= localTransform;}

	/**
	 * @brief Resets the frame rate timer
	 * 
	 * Resets the internal timer used for frame rate control. This is useful when
	 * starting a new capture session or after a pause.
	 */
	void resetTimer();
	
protected:
	/**
	 * @brief Protected constructor
	 * 
	 * Creates a SensorCapture instance with the specified frame rate and local transform.
	 * This constructor is protected because SensorCapture is an abstract base class
	 * and should not be instantiated directly.
	 * 
	 * @param frameRate Target frame rate in Hz (0 = unlimited, capture as fast as possible)
	 * @param localTransform Transform from base frame to sensor frame (default: identity)
	 */
	SensorCapture(float frameRate = 0, const Transform & localTransform = Transform::getIdentity());

	/**
	 * @brief Pure virtual method to capture sensor data
	 * 
	 * This method must be implemented by derived classes to perform the actual
	 * sensor data capture. It is called by `takeData()` after frame rate throttling.
	 * 
	 * The returned SensorData should:
	 * - Have rectified images if calibration was loaded during `init()`
	 * - Include proper timestamps
	 * - Contain valid sensor data (images, depth, laser scans, etc.)
	 * 
	 * @param info Optional pointer to SensorCaptureInfo to fill with capture metadata.
	 *             The base class will fill ID, timestamp, and capture time, but derived
	 *             classes can add additional information.
	 * @return SensorData containing the captured sensor data
	 * 
	 * @note If capture fails, return an empty SensorData (id=0, stamp=0.0).
	 * @note RGB and depth images should be already rectified if calibration was loaded.
	 * 
	 * @see takeData()
	 */
	virtual SensorData captureData(SensorCaptureInfo * info = 0) = 0;

	/**
	 * @brief Gets the next sequence ID
	 * 
	 * Returns and increments the internal sequence counter. This is used to assign
	 * unique sequence numbers to captured data.
	 * 
	 * @return Next sequence ID (starts at 1, increments with each call)
	 */
	int getNextSeqID() {return ++_seq;}

private:
	float _frameRate; ///< Target frame rate in Hz (0 = unlimited)
	Transform _localTransform; ///< Transform from base frame to sensor frame
	UTimer * _frameRateTimer; ///< Timer for frame rate control
	int _seq; ///< Sequence counter for captured data
};


} // namespace rtabmap
