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
#include <rtabmap/core/Transform.h>
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
 * An id is automatically generated if id=0.
 */
class RTABMAP_CORE_EXPORT SensorData
{
public:
	// empty constructor
	SensorData();

	// Appearance-only constructor
	SensorData(
			const cv::Mat & image,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	// Mono constructor
	SensorData(
			const cv::Mat & image,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	// RGB-D constructor
	SensorData(
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	// RGB-D constructor + laser scan
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	// Multi-cameras RGB-D constructor
	SensorData(
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const std::vector<CameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	// Multi-cameras RGB-D constructor + laser scan
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const std::vector<CameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	// Stereo constructor
	SensorData(
			const cv::Mat & left,
			const cv::Mat & right,
			const StereoCameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	// Stereo constructor + laser scan
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & left,
			const cv::Mat & right,
			const StereoCameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	// Multi-cameras stereo constructor
	SensorData(
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const std::vector<StereoCameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	// Multi-cameras stereo constructor + laser scan
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const std::vector<StereoCameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	// IMU constructor
	SensorData(
			const IMU & imu,
			int id = 0,
			double stamp = 0.0);

	virtual ~SensorData();

	bool isValid() const {
		return !(_id == 0 &&
			_stamp == 0.0 &&
			_imageRaw.empty() &&
			_imageCompressed.empty() &&
			_depthOrRightRaw.empty() &&
			_depthOrRightCompressed.empty() &&
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

	int id() const {return _id;}
	void setId(int id) {_id = id;}
	double stamp() const {return _stamp;}
	void setStamp(double stamp) {_stamp = stamp;}

	const cv::Mat & imageCompressed() const {return _imageCompressed;}
	const cv::Mat & depthOrRightCompressed() const {return _depthOrRightCompressed;}
	const LaserScan & laserScanCompressed() const {return _laserScanCompressed;}

	const cv::Mat & imageRaw() const {return _imageRaw;}
	const cv::Mat & depthOrRightRaw() const {return _depthOrRightRaw;}
	const LaserScan & laserScanRaw() const {return _laserScanRaw;}

	/**
	 * Set image data. Detect automatically if raw or compressed.
	 * A matrix of type CV_8UC1 with 1 row is considered as compressed.
	 * @param clearPreviousData, clear previous raw and compressed images before setting the new ones.
	 */
	void setRGBDImage(const cv::Mat & rgb, const cv::Mat & depth, const CameraModel & model, bool clearPreviousData = true);
	void setRGBDImage(const cv::Mat & rgb, const cv::Mat & depth, const std::vector<CameraModel> & models, bool clearPreviousData = true);
	void setStereoImage(const cv::Mat & left, const cv::Mat & right, const StereoCameraModel & stereoCameraModel, bool clearPreviousData = true);
	void setStereoImage(const cv::Mat & left, const cv::Mat & right, const std::vector<StereoCameraModel> & stereoCameraModels, bool clearPreviousData = true);

	/**
	 * Set laser scan data. Detect automatically if raw or compressed.
	 * A matrix of type CV_8UC1 with 1 row is considered as compressed.
	 * @param clearPreviousData, clear previous raw and compressed scans before setting the new one.
	 */
	void setLaserScan(const LaserScan & laserScan, bool clearPreviousData = true);

	void setCameraModel(const CameraModel & model) {_cameraModels.clear(); _cameraModels.push_back(model);}
	void setCameraModels(const std::vector<CameraModel> & models) {_cameraModels = models;}
	void setStereoCameraModel(const StereoCameraModel & stereoCameraModel) {_stereoCameraModels.clear(); _stereoCameraModels.push_back(stereoCameraModel);}
	void setStereoCameraModels(const std::vector<StereoCameraModel> & stereoCameraModels) {_stereoCameraModels = stereoCameraModels;}

	//for convenience
	cv::Mat depthRaw() const {return _depthOrRightRaw.type()!=CV_8UC1?_depthOrRightRaw:cv::Mat();}
	cv::Mat rightRaw() const {return _depthOrRightRaw.type()==CV_8UC1?_depthOrRightRaw:cv::Mat();}

	// Use setRGBDImage() or setStereoImage() with clearNotUpdated=false or removeRawData() instead. To be backward compatible, this function doesn't clear compressed data.
	RTABMAP_DEPRECATED void setImageRaw(const cv::Mat & image);
	// Use setRGBDImage() or setStereoImage() with clearNotUpdated=false or removeRawData() instead. To be backward compatible, this function doesn't clear compressed data.
	RTABMAP_DEPRECATED void setDepthOrRightRaw(const cv::Mat & image);
	// Use setLaserScan() with clearNotUpdated=false or removeRawData() instead. To be backward compatible, this function doesn't clear compressed data.
	RTABMAP_DEPRECATED void setLaserScanRaw(const LaserScan & scan);
	// Use setUserData() or removeRawData() instead.
	RTABMAP_DEPRECATED void setUserDataRaw(const cv::Mat & data);

	void uncompressData();
	void uncompressData(
			cv::Mat * imageRaw,
			cv::Mat * depthOrRightRaw,
			LaserScan * laserScanRaw = 0,
			cv::Mat * userDataRaw = 0,
			cv::Mat * groundCellsRaw = 0,
			cv::Mat * obstacleCellsRaw = 0,
			cv::Mat * emptyCellsRaw = 0);
	void uncompressDataConst(
			cv::Mat * imageRaw,
			cv::Mat * depthOrRightRaw,
			LaserScan * laserScanRaw = 0,
			cv::Mat * userDataRaw = 0,
			cv::Mat * groundCellsRaw = 0,
			cv::Mat * obstacleCellsRaw = 0,
			cv::Mat * emptyCellsRaw = 0) const;

	const std::vector<CameraModel> & cameraModels() const {return _cameraModels;}
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

	// detect automatically if raw or compressed. If raw, the data will be compressed.
	void setOccupancyGrid(
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			const cv::Mat & empty,
			float cellSize,
			const cv::Point3f & viewPoint);
	// remove raw occupancy grids
	void clearOccupancyGridRaw() {_groundCellsRaw = cv::Mat(); _obstacleCellsRaw = cv::Mat();}
	const cv::Mat & gridGroundCellsRaw() const {return _groundCellsRaw;}
	const cv::Mat & gridGroundCellsCompressed() const {return _groundCellsCompressed;}
	const cv::Mat & gridObstacleCellsRaw() const {return _obstacleCellsRaw;}
	const cv::Mat & gridObstacleCellsCompressed() const {return _obstacleCellsCompressed;}
	const cv::Mat & gridEmptyCellsRaw() const {return _emptyCellsRaw;}
	const cv::Mat & gridEmptyCellsCompressed() const {return _emptyCellsCompressed;}
	float gridCellSize() const {return _cellSize;}
	const cv::Point3f & gridViewPoint() const {return _viewPoint;}

	void setFeatures(const std::vector<cv::KeyPoint> & keypoints, const std::vector<cv::Point3f> & keypoints3D, const cv::Mat & descriptors);
	const std::vector<cv::KeyPoint> & keypoints() const {return _keypoints;}
	const std::vector<cv::Point3f> & keypoints3D() const {return _keypoints3D;}
	const cv::Mat & descriptors() const {return _descriptors;}

	void addGlobalDescriptor(const GlobalDescriptor & descriptor) {_globalDescriptors.push_back(descriptor);}
	void setGlobalDescriptors(const std::vector<GlobalDescriptor> & descriptors) {_globalDescriptors = descriptors;}
	void clearGlobalDescriptors() {_globalDescriptors.clear();}
	const std::vector<GlobalDescriptor> & globalDescriptors() const {return _globalDescriptors;}

	void setGroundTruth(const Transform & pose) {groundTruth_ = pose;}
	const Transform & groundTruth() const {return groundTruth_;}

	void setGlobalPose(const Transform & pose, const cv::Mat & covariance) {globalPose_ = pose; globalPoseCovariance_ = covariance;}
	const Transform & globalPose() const {return globalPose_;}
	const cv::Mat & globalPoseCovariance() const {return globalPoseCovariance_;}

	void setGPS(const GPS & gps) {gps_ = gps;}
	const GPS & gps() const {return gps_;}

	void setIMU(const IMU & imu) {imu_ = imu; }
	const IMU & imu() const {return imu_;}

	void setEnvSensors(const EnvSensors & sensors) {_envSensors = sensors;}
	void addEnvSensor(const EnvSensor & sensor) {_envSensors.insert(std::make_pair(sensor.type(), sensor));}
	const EnvSensors & envSensors() const {return _envSensors;}

	void setLandmarks(const Landmarks & landmarks) {_landmarks = landmarks;}
	const Landmarks & landmarks() const {return _landmarks;}

	unsigned long getMemoryUsed() const; // Return memory usage in Bytes
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

	bool isPointVisibleFromCameras(const cv::Point3f & pt) const; // assuming point is in robot frame

private:
	int _id;
	double _stamp;

	cv::Mat _imageCompressed;          // compressed image
	cv::Mat _depthOrRightCompressed;   // compressed image
	LaserScan _laserScanCompressed;      // compressed data

	cv::Mat _imageRaw;          // CV_8UC1 or CV_8UC3
	cv::Mat _depthOrRightRaw;   // depth CV_16UC1 or CV_32FC1, right image CV_8UC1
	LaserScan _laserScanRaw;

	std::vector<CameraModel> _cameraModels;
	std::vector<StereoCameraModel> _stereoCameraModels;

	// user data
	cv::Mat _userDataCompressed;      // compressed data
	cv::Mat _userDataRaw;

	// occupancy grid
	cv::Mat _groundCellsCompressed;
	cv::Mat _obstacleCellsCompressed;
	cv::Mat _emptyCellsCompressed;
	cv::Mat _groundCellsRaw;
	cv::Mat _obstacleCellsRaw;
	cv::Mat _emptyCellsRaw;
	float _cellSize;
	cv::Point3f _viewPoint;

	// environmental sensors
	EnvSensors _envSensors;

	// landmarks
	Landmarks _landmarks;

	// features
	std::vector<cv::KeyPoint> _keypoints;
	std::vector<cv::Point3f> _keypoints3D;
	cv::Mat _descriptors;

	// global descriptors
	std::vector<GlobalDescriptor> _globalDescriptors;

	Transform groundTruth_;

	Transform globalPose_;
	cv::Mat globalPoseCovariance_; // 6x6 double

	GPS gps_;

	IMU imu_;
};

}


#endif /* SENSORDATA_H_ */
