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

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsSender.h>

namespace clams
{
class DiscreteDepthDistortionModel;
}

namespace rtabmap
{

class Camera;
class Lidar;
class SensorCapture;
class SensorCaptureInfo;
class SensorData;
class StereoDense;
class IMUFilter;
class Feature2D;

/**
 * @class SensorCaptureThread
 * @brief Thread-based sensor data capture and event posting for RTAB-Map
 * 
 * SensorCaptureThread is a multi-threaded class that continuously captures sensor
 * data from cameras and/or lidars in a background thread. It processes the data,
 * optionally applies filtering and transformations, and posts SensorEvent events
 * to registered handlers through RTAB-Map's event system.
 * 
 * The class supports various sensor configurations:
 * - **Camera-only**: Single or multiple cameras for RGB-D or stereo imaging
 * - **Lidar-only**: Single lidar for 3D point cloud capture
 * - **Camera + Lidar**: Combined visual and range sensing
 * - **With odometry**: Optional odometry sensor for pose estimation and deskewing
 * 
 * Key features:
 * - **Automatic data synchronization**: Synchronizes camera and lidar data by timestamp
 * - **Deskewing**: Corrects lidar scans using odometry poses
 * - **Image processing**: Supports mirroring, decimation, histogram equalization, stereo-to-depth conversion
 * - **Depth filtering**: Optional bilateral filtering for depth images
 * - **IMU filtering**: Optional IMU data filtering and fusion
 * - **Feature detection**: Optional automatic feature detection on captured images
 * - **Frame rate control**: Configurable capture rate
 * - **Event-driven architecture**: Posts SensorEvent events for downstream processing
 * 
 * The class inherits from UThread (for threading) and UEventsSender (for event posting).
 * 
 * @note Ownership of Camera, Lidar, and SensorCapture pointers is transferred to this class.
 * @note The thread must be started with start() and stopped with kill() or join().
 * @warning **Thread Safety**: All configuration parameters (setters) should be called
 *          **before** starting the thread with start(). Changing configuration after the
 *          thread is started is not thread-safe and may lead to race conditions or
 *          undefined behavior. If you need to change parameters at runtime, stop the
 *          thread first, modify the configuration, then restart it.
 * 
 * @see Camera
 * @see Lidar
 * @see SensorCapture
 * @see SensorEvent
 * @see SensorData
 * @see UThread
 * @see UEventsSender
 */
class RTABMAP_CORE_EXPORT SensorCaptureThread :
	public UThread,
	public UEventsSender
{
public:
	/**
	 * @brief Constructor for camera-only capture
	 * 
	 * Creates a SensorCaptureThread that captures images from a single camera.
	 * The camera can be RGB-D, stereo, or mono.
	 * 
	 * @param camera Pointer to the camera to capture from (ownership transferred)
	 * @param parameters Optional parameters map for configuration
	 * 
	 * @note The camera pointer is owned by this class and will be deleted on destruction.
	 */
	SensorCaptureThread(
			Camera * camera,
			const ParametersMap & parameters = ParametersMap());
	
	/**
	 * @brief Constructor for camera with odometry sensor
	 * 
	 * Creates a SensorCaptureThread that captures images from a camera and uses
	 * an odometry sensor for pose estimation. The odometry sensor can be the same
	 * as the camera or a different sensor.
	 * 
	 * @param camera Pointer to the camera to capture images from (ownership transferred)
	 * @param odomSensor Pointer to the odometry sensor for pose estimation (can be the same as camera)
	 * @param extrinsics Static transform from odometry sensor's left lens frame to camera's left lens frame
	 *                   (without optical rotation applied)
	 * @param poseTimeOffset Time offset in seconds to add to data timestamp when querying pose (default: 0.0)
	 * @param poseScaleFactor Scale factor to apply to pose translation (default: 1.0, no scaling)
	 * @param poseWaitTime Maximum time in seconds to wait for pose data (default: 0.1)
	 * @param parameters Optional parameters map for configuration
	 * 
	 * @note If odomAsGt is set to true via setOdomAsGroundTruth(), the odometry pose
	 *       will be used as ground truth instead of odometry.
	 */
	SensorCaptureThread(
			Camera * camera,
			SensorCapture * odomSensor,
			const Transform & extrinsics,
			double poseTimeOffset = 0.0,
			float poseScaleFactor = 1.0f,
			double poseWaitTime = 0.1,
			const ParametersMap & parameters = ParametersMap());
	
	/**
	 * @brief Constructor for lidar-only capture
	 * 
	 * Creates a SensorCaptureThread that captures 3D point clouds from a lidar.
	 * 
	 * @param lidar Pointer to the lidar to capture scans from (ownership transferred)
	 * @param parameters Optional parameters map for configuration
	 * 
	 * @note The lidar pointer is owned by this class and will be deleted on destruction.
	 */
	SensorCaptureThread(
			Lidar * lidar,
			const ParametersMap & parameters = ParametersMap());
	
	/**
	 * @brief Constructor for lidar with camera
	 * 
	 * Creates a SensorCaptureThread that captures both lidar scans and camera images.
	 * If the camera provides pose information, it can be used for deskewing lidar scans.
	 * 
	 * @param lidar Pointer to the lidar to capture scans from (ownership transferred)
	 * @param camera Pointer to the camera to capture images from (ownership transferred).
	 *               If the camera provides pose, it can be used for deskewing.
	 * @param parameters Optional parameters map for configuration
	 */
	SensorCaptureThread(
			Lidar * lidar,
			Camera * camera,
			const ParametersMap & parameters = ParametersMap());
	
	/**
	 * @brief Constructor for lidar with odometry sensor
	 * 
	 * Creates a SensorCaptureThread that captures lidar scans and uses an odometry
	 * sensor for pose estimation and deskewing. The odometry sensor can be the same
	 * as the lidar or a different sensor.
	 * 
	 * @param lidar Pointer to the lidar to capture scans from (ownership transferred)
	 * @param odomSensor Pointer to the odometry sensor for pose estimation and deskewing
	 *                   (can be the same as lidar)
	 * @param poseTimeOffset Time offset in seconds to add to data timestamp when querying pose (default: 0.0)
	 * @param poseScaleFactor Scale factor to apply to pose translation (default: 1.0, no scaling)
	 * @param poseWaitTime Maximum time in seconds to wait for pose data (default: 0.1)
	 * @param parameters Optional parameters map for configuration
	 * 
	 * @note Deskewing is enabled by default when an odometry sensor is provided.
	 *       Use setScanParameters() to configure deskewing behavior.
	 * @note The lidar's local transform should be set as the extrinsics between odometry sensor's frame and lidar's frame.
	 */
	SensorCaptureThread(
			Lidar * lidar,
			SensorCapture * odomSensor,
			double poseTimeOffset = 0.0,
			float poseScaleFactor = 1.0f,
			double poseWaitTime = 0.1,
			const ParametersMap & parameters = ParametersMap());
	
	/**
	 * @brief Constructor for lidar with camera and odometry sensor
	 * 
	 * Creates a SensorCaptureThread that captures both lidar scans and camera images,
	 * and uses an odometry sensor for pose estimation and deskewing. This is the most
	 * comprehensive configuration, supporting multi-modal sensing with pose correction.
	 * 
	 * @param lidar Pointer to the lidar to capture scans from (ownership transferred)
	 * @param camera Pointer to the camera to capture images from (ownership transferred)
	 * @param odomSensor Pointer to the odometry sensor for pose estimation and deskewing
	 *                   (can be the same as camera or lidar)
	 * @param extrinsics Static transform from odometry frame to camera frame
	 *                   (without optical rotation applied)
	 * @param poseTimeOffset Time offset in seconds to add to data timestamp when querying pose (default: 0.0)
	 * @param poseScaleFactor Scale factor to apply to pose translation (default: 1.0, no scaling)
	 * @param poseWaitTime Maximum time in seconds to wait for pose data (default: 0.1)
	 * @param parameters Optional parameters map for configuration
	 * 
	 * @note The camera and lidar data are synchronized by timestamp, with the camera
	 *       frame being at least as recent as the lidar frame.
	 * @note The lidar's local transform should be set as the extrinsics between odometry sensor's frame and lidar's frame.
	 */
	SensorCaptureThread(
			Lidar * lidar,
			Camera * camera,
			SensorCapture * odomSensor,
			const Transform & extrinsics,
			double poseTimeOffset = 0.0,
			float poseScaleFactor = 1.0f,
			double poseWaitTime = 0.1,
			const ParametersMap & parameters = ParametersMap());
	
	/**
	 * @brief Virtual destructor
	 * 
	 * Stops the capture thread if running and deletes owned sensor pointers.
	 */
	virtual ~SensorCaptureThread();

	/**
	 * @brief Enables or disables image mirroring (horizontal flip)
	 * @param enabled True to enable mirroring, false to disable
	 */
	void setMirroringEnabled(bool enabled) {_mirroring = enabled;}
	
	/**
	 * @brief Enables or disables stereo exposure compensation
	 * 
	 * When enabled, adjusts exposure between left and right stereo cameras
	 * to improve matching quality.
	 * 
	 * @param enabled True to enable exposure compensation, false to disable
	 */
	void setStereoExposureCompensation(bool enabled) {_stereoExposureCompensation = enabled;}
	
	/**
	 * @brief Sets whether to capture color images only (skip depth)
	 * @param colorOnly True to capture color only, false to capture both color and depth
	 */
	void setColorOnly(bool colorOnly) {_colorOnly = colorOnly;}
	
	/**
	 * @brief Sets image decimation factor
	 * 
	 * Decimation reduces image resolution by skipping pixels. A decimation of N
	 * means every Nth pixel is kept (e.g., 2 = half resolution, 4 = quarter resolution).
	 * 
	 * @param decimation Decimation factor (1 = no decimation, 2 = half resolution, etc.)
	 */
	void setImageDecimation(int decimation) {_imageDecimation = decimation;}
	
	/**
	 * @brief Sets histogram equalization method
	 * 
	 * Controls the type of histogram equalization applied to captured images.
	 * Histogram equalization improves image contrast by redistributing pixel intensities.
	 * 
	 * @param histogramMethod Histogram equalization method:
	 *                        - 0 = None (disabled)
	 *                        - 1 = Standard histogram equalization (cv::equalizeHist)
	 *                        - 2 = CLAHE - Contrast Limited Adaptive Histogram Equalization (cv::createCLAHE)
	 * 
	 * @note For color images, histogram equalization is applied only to the luminance channel (Y in YCrCb color space).
	 * @note For stereo images, histogram equalization is applied to both left and right images.
	 */
	void setHistogramMethod(int histogramMethod) {_histogramMethod = histogramMethod;}
	
	/**
	 * @brief Enables or disables stereo-to-depth conversion
	 * 
	 * When enabled, converts stereo images to depth images using dense stereo matching.
	 * 
	 * @param enabled True to enable stereo-to-depth conversion, false to disable
	 */
	void setStereoToDepth(bool enabled) {_stereoToDepth = enabled;}
	
	/**
	 * @brief Sets the target frame rate for capture
	 * 
	 * Controls how often the capture loop runs. A frame rate of 0 means capture
	 * as fast as possible.
	 * 
	 * @param frameRate Target frame rate in Hz (0 = unlimited)
	 */
	void setFrameRate(float frameRate);
	
	/**
	 * @deprecated Use setFrameRate() instead
	 * @brief Sets the target image capture rate (deprecated)
	 */
	RTABMAP_DEPRECATED void setImageRate(float frameRate) {setFrameRate(frameRate);}
	
	/**
	 * @brief Sets the depth distortion model file path
	 * 
	 * Loads a discrete depth distortion model from the specified file path.
	 * This is used to correct systematic depth errors at longer ranges using
	 * the CLAMS (Calibration, Localization, And Mapping System) approach.
	 * 
	 * The distortion model is typically created using RTAB-Map's depth calibration
	 * tool, which uses visual odometry and 3D mapping to generate ground truth
	 * depth for calibration.
	 * 
	 * @param path Path to the distortion model file
	 * 
	 * @see https://github.com/introlab/rtabmap/wiki/Depth-Calibration
	 *      for detailed instructions on how to create and use depth distortion models
	 */
	void setDistortionModel(const std::string & path);
	
	/**
	 * @brief Sets whether odometry poses should be treated as ground truth
	 * 
	 * When enabled, odometry poses from the odometry sensor are stored as ground
	 * truth poses instead of odometry poses in the SensorData.
	 * 
	 * @param enabled True to use odometry as ground truth, false to use as odometry
	 */
	void setOdomAsGroundTruth(bool enabled) {_odomAsGt = enabled;}
	
	/**
	 * @brief Enables bilateral filtering for depth images
	 * 
	 * Bilateral filtering reduces noise in depth images while preserving edges.
	 * 
	 * @param sigmaS Spatial standard deviation (pixels)
	 * @param sigmaR Range standard deviation (depth units)
	 */
	void enableBilateralFiltering(float sigmaS, float sigmaR);
	
	/**
	 * @brief Disables bilateral filtering
	 */
	void disableBilateralFiltering() {_bilateralFiltering = false;}
	
	/**
	 * @brief Enables IMU data filtering
	 * 
	 * Enables filtering and fusion of IMU data from the sensor. The filtering
	 * strategy determines how IMU data is processed to estimate orientation.
	 * 
	 * @param filteringStrategy Filtering strategy:
	 *                          - 0 = Madgwick filter (requires RTAB-Map built with RTABMAP_MADGWICK option)
	 *                          - 1 = Complementary filter (default)
	 * @param parameters Optional parameters map for IMU filter configuration
	 * @param baseFrameConversion If true, converts IMU data to base frame
	 * 
	 * @note If Madgwick filter is requested but RTAB-Map is not built with the option enabled,
	 *       the Complementary filter will be used instead.
	 * 
	 * @see IMUFilter
	 */
	void enableIMUFiltering(int filteringStrategy=1, const ParametersMap & parameters = ParametersMap(), bool baseFrameConversion = false);
	
	/**
	 * @brief Disables IMU data filtering
	 */
	void disableIMUFiltering();
	
	/**
	 * @brief Enables automatic feature detection on captured images
	 * 
	 * When enabled, automatically detects and extracts visual features (keypoints,
	 * descriptors) from captured images.
	 * 
	 * @param parameters Optional parameters map for feature detector configuration.
	 *                   Parameters from the "Vis/" group (e.g., Vis/FeatureType, Vis/MaxFeatures)
	 *                   are automatically converted to "Kp/" parameters internally.
	 *                   If both Vis/ and Kp/ parameters are provided, Vis/ parameters take precedence.
	 * 
	 * @note The function converts Vis/ parameters to Kp/ parameters before creating the Feature2D detector.
	 */
	void enableFeatureDetection(const ParametersMap & parameters = ParametersMap());
	
	/**
	 * @brief Disables automatic feature detection
	 */
	void disableFeatureDetection();

	/**
	 * @deprecated Use the new version with groundNormalsUp parameter instead
	 * @brief Sets lidar scan processing parameters (deprecated)
	 * 
	 * Use the new version with groundNormalsUp parameter:
	 * - groundNormalsUp=0.8 for forceGroundNormalsUp=True
	 * - groundNormalsUp=0.0 for forceGroundNormalsUp=False
	 */
	RTABMAP_DEPRECATED void setScanParameters(
			bool fromDepth,
			int downsampleStep, // decimation of the depth image in case the scan is from depth image
			float rangeMin,
			float rangeMax,
			float voxelSize,
			int normalsK,
			float normalsRadius,
			bool forceGroundNormalsUp,
			bool deskewing);
	
	/**
	 * @brief Sets lidar scan processing parameters
	 * 
	 * Configures how lidar scans are processed, including conversion from depth images,
	 * filtering, normal estimation, and deskewing.
	 * 
	 * @param fromDepth If true, generate scans from depth images instead of raw lidar data
	 * @param downsampleStep Downsampling step:
	 *                       - If fromDepth is true: Image decimation step for depth images
	 *                         (1 = no decimation, 2 = every other pixel in both dimensions, etc.)
	 *                       - If fromDepth is false: Point skipping step for laser scan data
	 *                         (1 = no skipping, 2 = every other point, etc.)
	 * @param rangeMin Minimum range in meters (points closer are filtered out, 0 = no minimum)
	 * @param rangeMax Maximum range in meters (points farther are filtered out, 0 = no maximum)
	 * @param voxelSize Voxel size in meters for downsampling (0 = no voxelization)
	 * @param normalsK Number of neighbors for K-nearest neighbors normal estimation (0 = no normals)
	 * @param normalsRadius Radius in meters for radius-based normal estimation (0 = use K-nearest)
	 * @param groundNormalsUp Threshold for forcing ground normals upward (0.0-1.0, 0.8 = strong, 0.0 = disabled)
	 * @param deskewing If true, apply deskewing using odometry poses
	 */
	void setScanParameters(
			bool fromDepth,
			int downsampleStep=1, // decimation of the depth image in case the scan is from depth image
			float rangeMin=0.0f,
			float rangeMax=0.0f,
			float voxelSize = 0.0f,
			int normalsK = 0,
			float normalsRadius = 0.0f,
			float groundNormalsUp = 0.0f,
			bool deskewing = false);

	/**
	 * @brief Post-processing hook called before posting SensorEvent
	 * 
	 * This method is called after all processing is complete but before the
	 * SensorEvent is posted. Derived classes can override this to perform
	 * additional processing or modifications to the data.
	 * 
	 * @param data Pointer to the processed sensor data (can be modified)
	 * @param info Pointer to the sensor capture info (can be modified, may be null)
	 */
	void postUpdate(SensorData * data, SensorCaptureInfo * info = 0) const;

	// Getters
	
	/**
	 * @brief Checks if the capture thread is paused
	 * @return True if paused (not running), false if running
	 */
	bool isPaused() const {return !this->isRunning();}
	
	/**
	 * @brief Checks if the capture thread is actively capturing
	 * @return True if capturing (running), false if not running
	 */
	bool isCapturing() const {return this->isRunning();}
	
	/**
	 * @brief Checks if odometry is provided by the sensors
	 * @return True if an odometry sensor is configured, false otherwise
	 */
	bool odomProvided() const;

	/**
	 * @brief Returns the camera pointer
	 * @return Pointer to the camera (null if not set). Valid until SensorCaptureThread is deleted.
	 */
	Camera * camera() {return _camera;}
	
	/**
	 * @brief Returns the odometry sensor pointer
	 * @return Pointer to the odometry sensor (null if not set). Valid until SensorCaptureThread is deleted.
	 */
	SensorCapture * odomSensor() {return _odomSensor;}
	
	/**
	 * @brief Returns the lidar pointer
	 * @return Pointer to the lidar (null if not set). Valid until SensorCaptureThread is deleted.
	 */
	Lidar * lidar() {return _lidar;}

private:
	/**
	 * @brief Called once when the thread starts (before mainLoop)
	 * 
	 * Initializes resources needed for the capture loop.
	 */
	virtual void mainLoopBegin();
	
	/**
	 * @brief Main capture loop executed in the thread
	 * 
	 * Continuously captures sensor data, processes it, and posts SensorEvent events.
	 * This method runs until the thread is killed.
	 */
	virtual void mainLoop();
	
	/**
	 * @brief Called when the thread is being killed
	 * 
	 * Performs cleanup and ensures proper shutdown of sensors.
	 */
	virtual void mainLoopKill();

private:
	Camera * _camera; ///< Camera for image capture (owned, null if not used)
	SensorCapture * _odomSensor; ///< Odometry sensor for pose estimation (owned, null if not used)
	Lidar * _lidar; ///< Lidar for scan capture (owned, null if not used)
	Transform _extrinsicsOdomToCamera; ///< Static transform from odometry frame to camera frame
	bool _odomAsGt; ///< If true, use odometry poses as ground truth instead of odometry
	double _poseTimeOffset; ///< Time offset in seconds when querying poses
	float _poseScaleFactor; ///< Scale factor to apply to pose translation
	double _poseWaitTime; ///< Maximum time to wait for pose data
	bool _mirroring; ///< Enable horizontal image mirroring
	bool _stereoExposureCompensation; ///< Enable stereo exposure compensation
	bool _colorOnly; ///< Capture color images only (skip depth)
	int _imageDecimation; ///< Image decimation factor (1 = no decimation)
	int _histogramMethod; ///< Histogram equalization method
	bool _stereoToDepth; ///< Convert stereo images to depth using dense matching
	bool _scanDeskewing; ///< Enable lidar scan deskewing using odometry
	bool _scanFromDepth; ///< Generate scans from depth images instead of raw lidar
	int _scanDownsampleStep; ///< Decimation step for depth-to-scan conversion
	float _scanRangeMin; ///< Minimum scan range in meters (0 = no minimum)
	float _scanRangeMax; ///< Maximum scan range in meters (0 = no maximum)
	float _scanVoxelSize; ///< Voxel size for scan downsampling (0 = no voxelization)
	int _scanNormalsK; ///< K-nearest neighbors for normal estimation (0 = disabled)
	float _scanNormalsRadius; ///< Radius for normal estimation (0 = use K-nearest)
	float _scanForceGroundNormalsUp; ///< Threshold for forcing ground normals upward (0.0-1.0)
	StereoDense * _stereoDense; ///< Dense stereo matcher for stereo-to-depth conversion (owned)
	clams::DiscreteDepthDistortionModel * _distortionModel; ///< Depth distortion correction model (owned)
	bool _bilateralFiltering; ///< Enable bilateral filtering for depth images
	float _bilateralSigmaS; ///< Spatial standard deviation for bilateral filtering
	float _bilateralSigmaR; ///< Range standard deviation for bilateral filtering
	IMUFilter * _imuFilter; ///< IMU data filter (owned, null if disabled)
	bool _imuBaseFrameConversion; ///< Convert IMU data to base frame
	Feature2D * _featureDetector; ///< Feature detector for automatic feature extraction (owned, null if disabled)
	bool _depthAsMask; ///< Use depth as mask for feature detection
};

/**
 * @deprecated Use SensorCaptureThread instead
 * @brief Backward compatibility typedef
 * 
 * CameraThread is deprecated. Use SensorCaptureThread instead.
 */
RTABMAP_DEPRECATED typedef SensorCaptureThread CameraThread;

} // namespace rtabmap
