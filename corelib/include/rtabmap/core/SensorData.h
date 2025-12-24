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

#ifndef SENSORDATA_H_
#define SENSORDATA_H_

#include <rtabmap/core/rtabmap_core_export.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/GPS.h>
#include <rtabmap/core/EnvSensor.h>
#include <rtabmap/core/Landmark.h>
#include <rtabmap/core/GlobalDescriptor.h>

namespace rtabmap
{

/**
 * @class SensorData
 * @brief Container class for all sensor data captured at a specific time
 * 
 * SensorData is a comprehensive container class that holds all types of sensor
 * data captured at a specific timestamp, including:
 * 
 * - **Visual data**: RGB images, depth images, stereo image pairs (left/right)
 * - **3D data**: Laser scans, 3D point clouds
 * - **Camera models**: Mono camera models, stereo camera models (single or multiple)
 * - **Features**: Keypoints, 3D keypoints, descriptors
 * - **Global descriptors**: Scene-level descriptors for place recognition
 * - **Sensor fusion**: IMU data, GPS data, environmental sensors
 * - **Metadata**: Ground truth poses, global poses with covariance, landmarks
 * - **User data**: Custom data matrices
 * - **Occupancy grids**: Ground, obstacle, and empty cells
 * 
 * The class supports both **raw** and **compressed** data formats:
 * - Raw data: Uncompressed images, depth maps, laser scans
 * - Compressed data: JPEG-compressed images, compressed depth, compressed scans
 * 
 * Data compression is automatically detected when setting data:
 * - A matrix of type `CV_8UC1` with 1 row is considered compressed
 * - Use `uncompressData()` to decompress compressed data
 * 
 * The class supports multiple sensor configurations:
 * - **Appearance-only**: Single RGB image
 * - **Mono**: RGB image with camera model
 * - **RGB-D**: RGB + depth with camera model(s)
 * - **Stereo**: Left + right images with stereo camera model(s)
 * - **IMU-only**: IMU data without images
 * - **Multi-camera**: Multiple camera models (RGB-D or stereo)
 * - **Combined**: Any combination with laser scans, IMU, GPS, etc.
 * 
 * @note If `id=0` is provided to constructors, an ID is automatically generated.
 * @note The class manages memory efficiently by storing either raw or compressed
 *       data (or both), allowing for memory optimization in large datasets.
 * 
 * @see CameraModel
 * @see StereoCameraModel
 * @see LaserScan
 * @see IMU
 * @see GPS
 * @see SensorEvent
 */
class RTABMAP_CORE_EXPORT SensorData
{
public:
	/**
	 * @brief Default constructor
	 * 
	 * Creates an empty SensorData object with all fields initialized to default values.
	 * The ID will be 0 (invalid), and no sensor data will be present.
	 */
	SensorData();

	/**
	 * @brief Appearance-only constructor
	 * 
	 * Creates a SensorData object with a single RGB or grayscale image.
	 * No camera model is provided, so this is suitable for appearance-based
	 * place recognition without 3D reconstruction.
	 * 
	 * @param image RGB or grayscale image (CV_8UC1 or CV_8UC3)
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 */
	SensorData(
			const cv::Mat & image,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	/**
	 * @brief Mono camera constructor
	 * 
	 * Creates a SensorData object with a single RGB or grayscale image
	 * and a mono camera model for 3D projection.
	 * 
	 * @param image RGB or grayscale image (CV_8UC1 or CV_8UC3)
	 * @param cameraModel Camera model for the image
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 */
	SensorData(
			const cv::Mat & image,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	/**
	 * @brief RGB-D constructor
	 * 
	 * Creates a SensorData object with RGB and depth images and a camera model.
	 * 
	 * @param rgb RGB or grayscale image (CV_8UC1 or CV_8UC3)
	 * @param depth Depth image (CV_16UC1 for millimeters or CV_32FC1 for meters)
	 * @param cameraModel Camera model for RGB-D projection
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 */
	SensorData(
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	/**
	 * @brief RGB-D constructor with depth confidence
	 * 
	 * Creates a SensorData object with RGB, depth, and depth confidence images.
	 * 
	 * @param rgb RGB or grayscale image (CV_8UC1 or CV_8UC3)
	 * @param depth Depth image (CV_16UC1 for millimeters or CV_32FC1 for meters)
	 * @param depth_confidence Depth confidence map (CV_8UC1, typically 0-100)
	 * @param cameraModel Camera model for RGB-D projection
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 */
	SensorData(
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const cv::Mat & depth_confidence,
		const CameraModel & cameraModel,
		int id = 0,
		double stamp = 0.0,
		const cv::Mat & userData = cv::Mat());

	/**
	 * @brief RGB-D constructor with laser scan
	 * 
	 * Creates a SensorData object with RGB, depth, camera model, and laser scan.
	 * 
	 * @param laserScan 2D or 3D laser scan
	 * @param rgb RGB or grayscale image (CV_8UC1 or CV_8UC3)
	 * @param depth Depth image (CV_16UC1 for millimeters or CV_32FC1 for meters)
	 * @param cameraModel Camera model for RGB-D projection
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 */
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());
	
	/**
	 * @brief RGB-D constructor with depth confidence and laser scan
	 * 
	 * Creates a SensorData object with RGB, depth, depth confidence, camera model, and laser scan.
	 * 
	 * @param laserScan 2D or 3D laser scan
	 * @param rgb RGB or grayscale image (CV_8UC1 or CV_8UC3)
	 * @param depth Depth image (CV_16UC1 for millimeters or CV_32FC1 for meters)
	 * @param depthConfidence Depth confidence map (CV_8UC1)
	 * @param cameraModel Camera model for RGB-D projection
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 */
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const cv::Mat & depthConfidence,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	/**
	 * @brief Multi-camera RGB-D constructor
	 * 
	 * Creates a SensorData object with RGB, depth, and multiple camera models.
	 * `rgb` and `depth` images should contain stitched images from multiple cameras.
	 * All stitched images should have the same resolution.
	 * 
	 * @param rgb RGB or grayscale image (CV_8UC1 or CV_8UC3)
	 * @param depth Depth image (CV_16UC1 for millimeters or CV_32FC1 for meters)
	 * @param cameraModels Vector of camera models
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 * 
	 * @par Example: Stitching two camera images horizontally
	 * @code
	 * #include <rtabmap/core/Camera.h>
	 * #include <opencv2/imgproc/imgproc.hpp>
	 * 
	 * // Assume camera1 and camera2 are Camera* instances
	 * Camera* camera1 = ...;  // First camera
	 * Camera* camera2 = ...;  // Second camera
	 * 
	 * // Capture SensorData from both cameras
	 * SensorData data1 = camera1->takeImage();
	 * SensorData data2 = camera2->takeImage();
	 * 
	 * // Extract images and depth
	 * cv::Mat image1 = data1.imageRaw();      // 640x480
	 * cv::Mat depth1 = data1.depthOrRightRaw(); // 640x480
	 * cv::Mat image2 = data2.imageRaw();      // 640x480
	 * cv::Mat depth2 = data2.depthOrRightRaw(); // 640x480
	 * 
	 * // Stitch images horizontally
	 * cv::Mat stitchedRgb;
	 * cv::hconcat(image1, image2, stitchedRgb);  // Result: 1280x480
	 * 
	 * cv::Mat stitchedDepth;
	 * cv::hconcat(depth1, depth2, stitchedDepth);  // Result: 1280x480
	 * 
	 * // Get camera models from both SensorData objects
	 * std::vector<CameraModel> models = data1.cameraModels();
	 * std::vector<CameraModel> models2 = data2.cameraModels();
	 * models.insert(models.end(), models2.begin(), models2.end());
	 * 
	 * // Create SensorData with stitched images
	 * SensorData stitchedData(stitchedRgb, stitchedDepth, models, 0, data1.stamp());
	 * @endcode
	 */
	SensorData(
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const std::vector<CameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());
	
	/**
	 * @brief Multi-camera RGB-D constructor with depth confidence
	 * 
	 * Creates a SensorData object with RGB, depth, depth confidence, and multiple camera models.
	 * 
	 * @param rgb RGB or grayscale image (CV_8UC1 or CV_8UC3)
	 * @param depth Depth image (CV_16UC1 for millimeters or CV_32FC1 for meters)
	 * @param depthConfidence Depth confidence map (CV_8UC1)
	 * @param cameraModels Vector of camera models
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 * @see SensorData(const cv::Mat&, const cv::Mat&, const std::vector<CameraModel>&, int, double, const cv::Mat&)
	 *      for an example of stitching images from multiple cameras.
	 */
	SensorData(
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const cv::Mat & depthConfidence,
			const std::vector<CameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	/**
	 * @brief Multi-camera RGB-D constructor with laser scan
	 * 
	 * Creates a SensorData object with RGB, depth, multiple camera models, and laser scan.
	 * 
	 * @param laserScan 2D or 3D laser scan
	 * @param rgb RGB or grayscale image (CV_8UC1 or CV_8UC3)
	 * @param depth Depth image (CV_16UC1 for millimeters or CV_32FC1 for meters)
	 * @param cameraModels Vector of camera models
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 * @see SensorData(const cv::Mat&, const cv::Mat&, const std::vector<CameraModel>&, int, double, const cv::Mat&)
	 *      for an example of stitching images from multiple cameras.
	 */
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const std::vector<CameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	/**
	 * @brief Multi-camera RGB-D constructor with depth confidence and laser scan
	 * 
	 * Creates a SensorData object with RGB, depth, depth confidence, multiple camera models, and laser scan.
	 * 
	 * @param laserScan 2D or 3D laser scan
	 * @param rgb RGB or grayscale image (CV_8UC1 or CV_8UC3)
	 * @param depth Depth image (CV_16UC1 for millimeters or CV_32FC1 for meters)
	 * @param depthConfidence Depth confidence map (CV_8UC1)
	 * @param cameraModels Vector of camera models
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 * @see SensorData(const cv::Mat&, const cv::Mat&, const std::vector<CameraModel>&, int, double, const cv::Mat&)
	 *      for an example of stitching images from multiple cameras.
	 */
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const cv::Mat & depthConfidence,
			const std::vector<CameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	/**
	 * @brief Stereo camera constructor
	 * 
	 * Creates a SensorData object with left and right stereo images and a stereo camera model.
	 * 
	 * @param left Left stereo image (CV_8UC1 or CV_8UC3)
	 * @param right Right stereo image (CV_8UC1 or CV_8UC3)
	 * @param cameraModel Stereo camera model for disparity computation
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 */
	SensorData(
			const cv::Mat & left,
			const cv::Mat & right,
			const StereoCameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	/**
	 * @brief Stereo camera constructor with laser scan
	 * 
	 * Creates a SensorData object with left and right stereo images, stereo camera model, and laser scan.
	 * 
	 * @param laserScan 2D or 3D laser scan
	 * @param left Left stereo image (CV_8UC1 or CV_8UC3)
	 * @param right Right stereo image (CV_8UC1 or CV_8UC3)
	 * @param cameraModel Stereo camera model for disparity computation
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 */
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & left,
			const cv::Mat & right,
			const StereoCameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	/**
	 * @brief Multi-camera stereo constructor
	 * 
	 * Creates a SensorData object with left and right stereo images and multiple stereo camera models.
	 * 
	 * @param rgb Left stereo image (CV_8UC1 or CV_8UC3)
	 * @param depth Right stereo image (CV_8UC1 or CV_8UC3)
	 * @param cameraModels Vector of stereo camera models
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 * @see SensorData(const cv::Mat&, const cv::Mat&, const std::vector<CameraModel>&, int, double, const cv::Mat&)
	 *      for an example of stitching images from multiple cameras.
	 */
	SensorData(
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const std::vector<StereoCameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	/**
	 * @brief Multi-camera stereo constructor with laser scan
	 * 
	 * Creates a SensorData object with left and right stereo images, multiple stereo camera models, and laser scan.
	 * 
	 * @param laserScan 2D or 3D laser scan
	 * @param rgb Left stereo image (CV_8UC1 or CV_8UC3)
	 * @param depth Right stereo image (CV_8UC1 or CV_8UC3)
	 * @param cameraModels Vector of stereo camera models
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 * @param userData Optional user-defined data matrix
	 */
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const std::vector<StereoCameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	/**
	 * @brief IMU-only constructor
	 * 
	 * Creates a SensorData object with only IMU data (no images).
	 * Useful for IMU-only odometry or sensor fusion scenarios.
	 * 
	 * @param imu IMU data (accelerometer, gyroscope, orientation)
	 * @param id Sensor data ID (0 to auto-generate)
	 * @param stamp Timestamp in seconds
	 */
	SensorData(
			const IMU & imu,
			int id = 0,
			double stamp = 0.0);

	/**
	 * @brief Virtual destructor
	 */
	virtual ~SensorData();

	/**
	 * @brief Checks if the sensor data is valid
	 * 
	 * Returns true if the sensor data contains at least one of:
	 * - Valid ID (> 0) or non-zero stamp
	 * - Images (raw or compressed)
	 * - Depth/right images (raw or compressed)
	 * - Depth confidence (raw or compressed)
	 * - Laser scan (raw or compressed)
	 * - Camera models (mono or stereo)
	 * - User data (raw or compressed)
	 * - Keypoints and descriptors
	 * - IMU data
	 * 
	 * @return True if the sensor data contains any valid information, false otherwise
	 */
	bool isValid() const {
		return !(_id == 0 &&
			_stamp == 0.0 &&
			_imageRaw.empty() &&
			_imageCompressed.empty() &&
			_depthOrRightRaw.empty() &&
			_depthOrRightCompressed.empty() &&
			_depthConfidenceRaw.empty() &&
			_depthConfidenceCompressed.empty() &&
			_laserScanRaw.isEmpty() &&
			_laserScanCompressed.isEmpty() &&
			_cameraModels.empty() &&
			_stereoCameraModels.empty() &&
			_userDataRaw.empty() &&
			_userDataCompressed.empty() &&
			_keypoints.size() == 0 &&
			_descriptors.empty() &&
			imu_.empty());
	}

	/**
	 * @brief Returns the sensor data ID
	 * @return The unique sensor data ID (0 if invalid/uninitialized)
	 */
	int id() const {return _id;}
	
	/**
	 * @brief Sets the sensor data ID
	 * @param id The unique sensor data ID
	 */
	void setId(int id) {_id = id;}
	
	/**
	 * @brief Returns the timestamp
	 * @return Timestamp in seconds (typically seconds since epoch)
	 */
	double stamp() const {return _stamp;}
	
	/**
	 * @brief Sets the timestamp
	 * @param stamp Timestamp in seconds
	 */
	void setStamp(double stamp) {_stamp = stamp;}

	/**
	 * @brief Returns the compressed RGB/grayscale image
	 * @return Compressed image (CV_8UC1 with 1 row, JPEG/PNG-compressed bytes)
	 * @note Use Compression::uncompressImage() to uncompress the image.
	 */
	const cv::Mat & imageCompressed() const {return _imageCompressed;}
	
	/**
	 * @brief Returns the compressed depth or right stereo image
	 * @return Compressed depth/right image (CV_8UC1 with 1 row, compressed bytes)
	 * @note Use Compression::uncompressImage() to uncompress the image.
	 */
	const cv::Mat & depthOrRightCompressed() const {return _depthOrRightCompressed;}
	
	/**
	 * @brief Returns the compressed depth confidence map
	 * @return Compressed depth confidence (CV_8UC1 with 1 row, compressed bytes)
	 * @note Use Compression::uncompressData() to uncompress the data.
	 */
	const cv::Mat & depthConfidenceCompressed() const {return _depthConfidenceCompressed;}
	
	/**
	 * @brief Returns the compressed laser scan
	 * @return Compressed laser scan
	 */
	const LaserScan & laserScanCompressed() const {return _laserScanCompressed;}

	/**
	 * @brief Returns the raw RGB/grayscale image
	 * @return Raw image (CV_8UC1 for grayscale or CV_8UC3 for RGB)
	 */
	const cv::Mat & imageRaw() const {return _imageRaw;}
	
	/**
	 * @brief Returns the raw depth or right stereo image
	 * @return Raw depth (CV_16UC1 for millimeters or CV_32FC1 for meters) or
	 *         right stereo image (CV_8UC1 or CV_8UC3)
	 */
	const cv::Mat & depthOrRightRaw() const {return _depthOrRightRaw;}
	
	/**
	 * @brief Returns the raw depth confidence map
	 * @return Raw depth confidence (CV_8UC1, typically 0-100)
	 */
	const cv::Mat & depthConfidenceRaw() const {return _depthConfidenceRaw;}
	
	/**
	 * @brief Returns the raw laser scan
	 * @return Raw laser scan (2D or 3D)
	 */
	const LaserScan & laserScanRaw() const {return _laserScanRaw;}

	/**
	 * Set image data. Detect automatically if raw or compressed.
	 * A matrix of type CV_8UC1 with 1 row is considered as compressed.
	 * @param clearPreviousData, clear previous raw and compressed images before setting the new ones.
	 */
	void setRGBDImage(const cv::Mat & rgb, const cv::Mat & depth, const CameraModel & model, bool clearPreviousData = true);
	void setRGBDImage(const cv::Mat & rgb, const cv::Mat & depth, const cv::Mat & depth_confidence, const CameraModel & model, bool clearPreviousData = true);
	void setRGBDImage(const cv::Mat & rgb, const cv::Mat & depth, const std::vector<CameraModel> & models, bool clearPreviousData = true);
	void setRGBDImage(const cv::Mat & rgb, const cv::Mat & depth, const cv::Mat & depth_confidence, const std::vector<CameraModel> & models, bool clearPreviousData = true);
	void setStereoImage(const cv::Mat & left, const cv::Mat & right, const StereoCameraModel & stereoCameraModel, bool clearPreviousData = true);
	void setStereoImage(const cv::Mat & left, const cv::Mat & right, const std::vector<StereoCameraModel> & stereoCameraModels, bool clearPreviousData = true);

	/**
	 * Set laser scan data. Detect automatically if raw or compressed.
	 * A matrix of type CV_8UC1 with 1 row is considered as compressed.
	 * @param clearPreviousData, clear previous raw and compressed scans before setting the new one.
	 */
	void setLaserScan(const LaserScan & laserScan, bool clearPreviousData = true);

	/**
	 * @brief Sets a single camera model (clears previous models)
	 * @param model Camera model to set
	 */
	void setCameraModel(const CameraModel & model) {_cameraModels.clear(); _cameraModels.push_back(model);}
	
	/**
	 * @brief Sets multiple camera models
	 * @param models Vector of camera models
	 */
	void setCameraModels(const std::vector<CameraModel> & models) {_cameraModels = models;}
	
	/**
	 * @brief Sets a single stereo camera model (clears previous models)
	 * @param stereoCameraModel Stereo camera model to set
	 */
	void setStereoCameraModel(const StereoCameraModel & stereoCameraModel) {_stereoCameraModels.clear(); _stereoCameraModels.push_back(stereoCameraModel);}
	
	/**
	 * @brief Sets multiple stereo camera models
	 * @param stereoCameraModels Vector of stereo camera models
	 */
	void setStereoCameraModels(const std::vector<StereoCameraModel> & stereoCameraModels) {_stereoCameraModels = stereoCameraModels;}

	/**
	 * @brief Returns the depth image (convenience method)
	 * 
	 * Returns the depth image if `_depthOrRightRaw` contains depth data
	 * (not a right stereo image). Depth images are CV_16UC1 or CV_32FC1,
	 * while right stereo images are CV_8UC1 or CV_8UC3.
	 * 
	 * @return Depth image if available, empty Mat if right stereo image or empty
	 */
	cv::Mat depthRaw() const {return !(_depthOrRightRaw.type()==CV_8UC1 || _depthOrRightRaw.type()==CV_8UC3) ? _depthOrRightRaw : cv::Mat();}
	
	/**
	 * @brief Returns the right stereo image (convenience method)
	 * 
	 * Returns the right stereo image if `_depthOrRightRaw` contains stereo data
	 * (not a depth image). Right stereo images are CV_8UC1 or CV_8UC3,
	 * while depth images are CV_16UC1 or CV_32FC1.
	 * 
	 * @return Right stereo image if available, empty Mat if depth image or empty
	 */
	cv::Mat rightRaw() const {return   _depthOrRightRaw.type()==CV_8UC1 || _depthOrRightRaw.type()==CV_8UC3  ? _depthOrRightRaw : cv::Mat();}

	// Use setRGBDImage() or setStereoImage() with clearNotUpdated=false or removeRawData() instead. To be backward compatible, this function doesn't clear compressed data.
	RTABMAP_DEPRECATED void setImageRaw(const cv::Mat & image);
	// Use setRGBDImage() or setStereoImage() with clearNotUpdated=false or removeRawData() instead. To be backward compatible, this function doesn't clear compressed data.
	RTABMAP_DEPRECATED void setDepthOrRightRaw(const cv::Mat & image);
	// Use setLaserScan() with clearNotUpdated=false or removeRawData() instead. To be backward compatible, this function doesn't clear compressed data.
	RTABMAP_DEPRECATED void setLaserScanRaw(const LaserScan & scan);
	// Use setUserData() or removeRawData() instead.
	RTABMAP_DEPRECATED void setUserDataRaw(const cv::Mat & data);

	/**
	 * @brief Uncompresses all compressed data in-place
	 * 
	 * Decompresses all compressed images, depth, laser scans, and user data,
	 * storing the results in the raw data members. This modifies the object.
	 */
	void uncompressData();
	
	/**
	 * @brief Uncompresses compressed data into provided output buffers
	 * 
	 * Decompresses compressed data and stores results in the provided output buffers.
	 * If a buffer pointer is null, that data type is skipped. The raw fields of the object
	 * is updated with the uncompressed data so that consecutive calls with same field should 
	 * not trigger a uncompression again.
	 * 
	 * @param imageRaw Output buffer for uncompressed RGB/grayscale image
	 * @param depthOrRightRaw Output buffer for uncompressed depth or right stereo image
	 * @param laserScanRaw Output buffer for uncompressed laser scan
	 * @param userDataRaw Output buffer for uncompressed user data
	 * @param groundCellsRaw Output buffer for uncompressed ground occupancy grid cells
	 * @param obstacleCellsRaw Output buffer for uncompressed obstacle occupancy grid cells
	 * @param emptyCellsRaw Output buffer for uncompressed empty occupancy grid cells
	 * @param depthConfidenceRaw Output buffer for uncompressed depth confidence
	 */
	void uncompressData(
			cv::Mat * imageRaw,
			cv::Mat * depthOrRightRaw,
			LaserScan * laserScanRaw = 0,
			cv::Mat * userDataRaw = 0,
			cv::Mat * groundCellsRaw = 0,
			cv::Mat * obstacleCellsRaw = 0,
			cv::Mat * emptyCellsRaw = 0,
			cv::Mat * depthConfidenceRaw = 0);
	
	/**
	 * @brief Uncompresses compressed data into provided output buffers (const version)
	 * 
	 * Same as `uncompressData()` but does not modify the object's internal state.
	 * 
	 * @param imageRaw Output buffer for uncompressed RGB/grayscale image
	 * @param depthOrRightRaw Output buffer for uncompressed depth or right stereo image
	 * @param laserScanRaw Output buffer for uncompressed laser scan
	 * @param userDataRaw Output buffer for uncompressed user data
	 * @param groundCellsRaw Output buffer for uncompressed ground occupancy grid cells
	 * @param obstacleCellsRaw Output buffer for uncompressed obstacle occupancy grid cells
	 * @param emptyCellsRaw Output buffer for uncompressed empty occupancy grid cells
	 * @param depthConfidenceRaw Output buffer for uncompressed depth confidence
	 */
	void uncompressDataConst(
			cv::Mat * imageRaw,
			cv::Mat * depthOrRightRaw,
			LaserScan * laserScanRaw = 0,
			cv::Mat * userDataRaw = 0,
			cv::Mat * groundCellsRaw = 0,
			cv::Mat * obstacleCellsRaw = 0,
			cv::Mat * emptyCellsRaw = 0,
			cv::Mat * depthConfidenceRaw = 0) const;

	/**
	 * @brief Returns the camera models
	 * @return Const reference to vector of mono camera models
	 */
	const std::vector<CameraModel> & cameraModels() const {return _cameraModels;}
	
	/**
	 * @brief Returns the stereo camera models
	 * @return Const reference to vector of stereo camera models
	 */
	const std::vector<StereoCameraModel> & stereoCameraModels() const {return _stereoCameraModels;}

	/**
	 * Set user data. Detect automatically if raw or compressed. If raw, the data is
	 * compressed too. A matrix of type CV_8UC1 with 1 row is considered as compressed.
	 * If you have one dimension unsigned 8 bits raw data, make sure to transpose it
	 * (to have multiple rows instead of multiple columns) in order to be detected as
	 * not compressed.
	 * @param clearPreviousData, clear previous raw and compressed user data before setting the new one.
	 */
	void setUserData(const cv::Mat & userData, bool clearPreviousData = true);
	const cv::Mat & userDataRaw() const {return _userDataRaw;}
	const cv::Mat & userDataCompressed() const {return _userDataCompressed;}

	/**
	 * @brief Sets occupancy grid data
	 * 
	 * Sets the occupancy grid with ground, obstacle, and empty cells.
	 * The data is automatically detected as raw or compressed. If raw,
	 * it will be compressed automatically.
	 * 
	 * @param ground Ground cells matrix
	 * @param obstacles Obstacle cells matrix
	 * @param empty Empty cells matrix
	 * @param cellSize Size of each grid cell in meters
	 * @param viewPoint Viewpoint/origin of the occupancy grid (3D point)
	 */
	void setOccupancyGrid(
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			const cv::Mat & empty,
			float cellSize,
			const cv::Point3f & viewPoint);
	
	/**
	 * @brief Clears raw occupancy grid data (keeps compressed)
	 */
	void clearOccupancyGridRaw() {_groundCellsRaw = cv::Mat(); _obstacleCellsRaw = cv::Mat();}
	
	/**
	 * @brief Returns raw ground cells
	 * @return Const reference to raw ground cells matrix
	 */
	const cv::Mat & gridGroundCellsRaw() const {return _groundCellsRaw;}
	
	/**
	 * @brief Returns compressed ground cells
	 * @return Const reference to compressed ground cells matrix
	 * @note Use Compression::uncompressData() to uncompress the data.
	 */
	const cv::Mat & gridGroundCellsCompressed() const {return _groundCellsCompressed;}
	
	/**
	 * @brief Returns raw obstacle cells
	 * @return Const reference to raw obstacle cells matrix
	 */
	const cv::Mat & gridObstacleCellsRaw() const {return _obstacleCellsRaw;}
	
	/**
	 * @brief Returns compressed obstacle cells
	 * @return Const reference to compressed obstacle cells matrix
	 * @note Use Compression::uncompressData() to uncompress the data.
	 */
	const cv::Mat & gridObstacleCellsCompressed() const {return _obstacleCellsCompressed;}
	
	/**
	 * @brief Returns raw empty cells
	 * @return Const reference to raw empty cells matrix
	 */
	const cv::Mat & gridEmptyCellsRaw() const {return _emptyCellsRaw;}
	
	/**
	 * @brief Returns compressed empty cells
	 * @return Const reference to compressed empty cells matrix
	 * @note Use Compression::uncompressData() to uncompress the data.
	 */
	const cv::Mat & gridEmptyCellsCompressed() const {return _emptyCellsCompressed;}
	
	/**
	 * @brief Returns the occupancy grid cell size
	 * @return Cell size in meters
	 */
	float gridCellSize() const {return _cellSize;}
	
	/**
	 * @brief Returns the occupancy grid viewpoint
	 * @return Const reference to viewpoint/origin 3D point
	 */
	const cv::Point3f & gridViewPoint() const {return _viewPoint;}

	/**
	 * @brief Sets visual features (keypoints, 3D points, descriptors)
	 * 
	 * Sets the visual features extracted from the images. All arrays must have
	 * matching sizes (one keypoint per 3D point and descriptor).
	 * 
	 * @param keypoints 2D keypoints in image coordinates
	 * @param keypoints3D 3D points corresponding to keypoints (in base_link frame)
	 * @param descriptors Feature descriptors matrix (one row per keypoint)
	 */
	void setFeatures(const std::vector<cv::KeyPoint> & keypoints, const std::vector<cv::Point3f> & keypoints3D, const cv::Mat & descriptors);
	
	/**
	 * @brief Returns the 2D keypoints
	 * @return Const reference to vector of 2D keypoints
	 */
	const std::vector<cv::KeyPoint> & keypoints() const {return _keypoints;}
	
	/**
	 * @brief Returns the 3D keypoints
	 * @return Const reference to vector of 3D points (in base_link frame)
	 */
	const std::vector<cv::Point3f> & keypoints3D() const {return _keypoints3D;}
	
	/**
	 * @brief Returns the feature descriptors
	 * @return Const reference to descriptors matrix (one row per keypoint)
	 */
	const cv::Mat & descriptors() const {return _descriptors;}

	/**
	 * @brief Adds a global descriptor
	 * @param descriptor Global descriptor to add
	 */
	void addGlobalDescriptor(const GlobalDescriptor & descriptor) {_globalDescriptors.push_back(descriptor);}
	
	/**
	 * @brief Sets all global descriptors
	 * @param descriptors Vector of global descriptors
	 */
	void setGlobalDescriptors(const std::vector<GlobalDescriptor> & descriptors) {_globalDescriptors = descriptors;}
	
	/**
	 * @brief Clears all global descriptors
	 */
	void clearGlobalDescriptors() {_globalDescriptors.clear();}
	
	/**
	 * @brief Returns all global descriptors
	 * @return Const reference to vector of global descriptors
	 */
	const std::vector<GlobalDescriptor> & globalDescriptors() const {return _globalDescriptors;}

	/**
	 * @brief Sets the ground truth pose
	 * @param pose Ground truth transform (for evaluation purposes)
	 */
	void setGroundTruth(const Transform & pose) {groundTruth_ = pose;}
	
	/**
	 * @brief Returns the ground truth pose
	 * @return Const reference to ground truth transform
	 */
	const Transform & groundTruth() const {return groundTruth_;}

	/**
	 * @brief Sets the global pose with covariance
	 * @param pose Global pose transform
	 * @param covariance Pose covariance matrix (6x6, CV_64FC1)
	 */
	void setGlobalPose(const Transform & pose, const cv::Mat & covariance) {globalPose_ = pose; globalPoseCovariance_ = covariance;}
	
	/**
	 * @brief Returns the global pose
	 * @return Const reference to global pose transform
	 */
	const Transform & globalPose() const {return globalPose_;}
	
	/**
	 * @brief Returns the global pose covariance
	 * @return Const reference to pose covariance matrix (6x6, CV_64FC1)
	 */
	const cv::Mat & globalPoseCovariance() const {return globalPoseCovariance_;}

	/**
	 * @brief Sets GPS data
	 * @param gps GPS data (latitude, longitude, altitude, etc.)
	 */
	void setGPS(const GPS & gps) {gps_ = gps;}
	
	/**
	 * @brief Returns GPS data
	 * @return Const reference to GPS data
	 */
	const GPS & gps() const {return gps_;}

	/**
	 * @brief Sets IMU data
	 * @param imu IMU data (accelerometer, gyroscope, orientation)
	 */
	void setIMU(const IMU & imu) {imu_ = imu; }
	
	/**
	 * @brief Returns IMU data
	 * @return Const reference to IMU data
	 */
	const IMU & imu() const {return imu_;}

	/**
	 * @brief Sets all environmental sensors
	 * @param sensors Map of environmental sensor types to sensor data
	 */
	void setEnvSensors(const EnvSensors & sensors) {_envSensors = sensors;}
	
	/**
	 * @brief Adds a single environmental sensor
	 * @param sensor Environmental sensor to add
	 */
	void addEnvSensor(const EnvSensor & sensor) {_envSensors.insert(std::make_pair(sensor.type(), sensor));}
	
	/**
	 * @brief Returns all environmental sensors
	 * @return Const reference to map of environmental sensors
	 */
	const EnvSensors & envSensors() const {return _envSensors;}

	/**
	 * @brief Sets landmarks
	 * @param landmarks Map of landmark IDs to landmark data
	 * @note Landmark IDs are typically negative.
	 */
	void setLandmarks(const Landmarks & landmarks) {_landmarks = landmarks;}
	
	/**
	 * @brief Returns landmarks
	 * @return Const reference to map of landmarks
	 */
	const Landmarks & landmarks() const {return _landmarks;}

	/**
	 * @brief Computes the memory usage of this sensor data
	 * 
	 * Calculates the approximate memory footprint including all images,
	 * laser scans, features, descriptors, and other data structures.
	 * 
	 * @return Memory usage in bytes
	 */
	unsigned long getMemoryUsed() const;
	/**
	 * Clear compressed rgb/depth (left/right) images, compressed laser scan and compressed user data.
	 * Raw data are kept is set.
	 */
	void clearCompressedData(bool images = true, bool scan = true, bool userData = true);
	/**
	 * Clear raw rgb/depth (left/right) images, raw laser scan and raw user data.
	 * Compressed data are kept is set.
	 */
	void clearRawData(bool images = true, bool scan = true, bool userData = true);

	/**
	 * @brief Checks if a 3D point is visible from any camera
	 * 
	 * Projects the 3D point into all camera models and checks if it falls
	 * within the image bounds.
	 * 
	 * @param pt 3D point in robot frame (base_link)
	 * @return True if the point is visible from at least one camera, false otherwise
	 */
	bool isPointVisibleFromCameras(const cv::Point3f & pt) const;

#ifdef HAVE_OPENCV_CUDEV
	/**
	 * @brief Returns the GPU image buffer
	 * @return Const reference to GPU image matrix (for CUDA optimizations)
	 */
	const cv::cuda::GpuMat & imageRawGpu() const {return _imageRawGpu;}
	
	/**
	 * @brief Sets the GPU image buffer
	 * @param image GPU image matrix (for CUDA optimizations)
	 */
	void setImageRawGpu(const cv::cuda::GpuMat & image) {_imageRawGpu = image;}
	
	/**
	 * @brief Returns the GPU depth/right image buffer
	 * @return Const reference to GPU depth/right image matrix (for CUDA optimizations)
	 */
	const cv::cuda::GpuMat & depthOrRightRawGpu() const {return _depthOrRightRawGpu;}
	
	/**
	 * @brief Sets the GPU depth/right image buffer
	 * @param image GPU depth/right image matrix (for CUDA optimizations)
	 */
	void setDepthOrRightRawGpu(const cv::cuda::GpuMat & image) {_depthOrRightRawGpu = image;}
#endif

private:
	int _id; ///< Unique sensor data ID (0 if invalid)
	double _stamp; ///< Timestamp in seconds

	// Compressed data
	cv::Mat _imageCompressed; ///< Compressed RGB/grayscale image (CV_8UC1 with 1 row, JPEG/PNG-compressed bytes)
	cv::Mat _depthOrRightCompressed; ///< Compressed depth or right stereo image (CV_8UC1 with 1 row, PNG/RVL-compressed bytes)
	cv::Mat _depthConfidenceCompressed; ///< Compressed depth confidence map (CV_8UC1 with 1 row, PNG-compressed bytes)
	LaserScan _laserScanCompressed; ///< Compressed laser scan (ZIP-compressed bytes)

	// Raw data
	cv::Mat _imageRaw; ///< Raw RGB/grayscale image (CV_8UC1 for grayscale or CV_8UC3 for RGB)
	cv::Mat _depthOrRightRaw; ///< Raw depth (CV_16UC1 for millimeters or CV_32FC1 for meters) or right stereo image (CV_8UC1 or CV_8UC3)
	cv::Mat _depthConfidenceRaw; ///< Raw depth confidence map (CV_8UC1, typically 0-100)
	LaserScan _laserScanRaw; ///< Raw laser scan (2D or 3D)

	// Camera models
	std::vector<CameraModel> _cameraModels; ///< Mono camera models used with RGB-D cameras
	std::vector<StereoCameraModel> _stereoCameraModels; ///< Stereo camera models

	// User data
	cv::Mat _userDataCompressed; ///< Compressed user-defined data (CV_8UC1 with 1 row, ZIP-compressed bytes)
	cv::Mat _userDataRaw; ///< Raw user-defined data

	// Occupancy grid
	cv::Mat _groundCellsCompressed; ///< Compressed ground cells matrix (ZIP-compressed bytes)
	cv::Mat _obstacleCellsCompressed; ///< Compressed obstacle cells matrix (ZIP-compressed bytes)
	cv::Mat _emptyCellsCompressed; ///< Compressed empty cells matrix (ZIP-compressed bytes)
	cv::Mat _groundCellsRaw; ///< Raw ground cells matrix
	cv::Mat _obstacleCellsRaw; ///< Raw obstacle cells matrix
	cv::Mat _emptyCellsRaw; ///< Raw empty cells matrix
	float _cellSize; ///< Occupancy grid cell size in meters
	cv::Point3f _viewPoint; ///< Occupancy grid viewpoint/origin (3D point)

	// Environmental sensors
	EnvSensors _envSensors; ///< Map of environmental sensor types to sensor data

	// Landmarks
	Landmarks _landmarks; ///< Map of landmark IDs to landmark data

	// Visual features
	std::vector<cv::KeyPoint> _keypoints; ///< 2D keypoints in image coordinates
	std::vector<cv::Point3f> _keypoints3D; ///< 3D points corresponding to keypoints (in base_link frame)
	cv::Mat _descriptors; ///< Feature descriptors matrix (one row per keypoint)

	// Global descriptors
	std::vector<GlobalDescriptor> _globalDescriptors; ///< Scene-level descriptors for place recognition

	// Poses
	Transform groundTruth_; ///< Ground truth pose (for evaluation purposes)
	Transform globalPose_; ///< Global pose transform
	cv::Mat globalPoseCovariance_; ///< Global pose covariance matrix (6x6, CV_64FC1)

	// Sensor fusion
	GPS gps_; ///< GPS data (latitude, longitude, altitude, etc.)
	IMU imu_; ///< IMU data (accelerometer, gyroscope, orientation)

#ifdef HAVE_OPENCV_CUDEV
	/**
	 * @brief Temporary GPU buffers for CUDA optimizations
	 * 
	 * These buffers are used to avoid host<->device copies when the same
	 * data is re-used multiple times in GPU-accelerated operations.
	 */
	cv::cuda::GpuMat _imageRawGpu; ///< GPU buffer for RGB/grayscale image
	cv::cuda::GpuMat _depthOrRightRawGpu; ///< GPU buffer for depth or right stereo image
#endif
};

}


#endif /* SENSORDATA_H_ */
