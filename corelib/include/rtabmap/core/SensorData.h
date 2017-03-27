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

#include <rtabmap/core/RtabmapExp.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/LaserScanInfo.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace rtabmap
{

/**
 * An id is automatically generated if id=0.
 */
class RTABMAP_EXP SensorData
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
			const cv::Mat & laserScan,
			const LaserScanInfo & laserScanInfo,
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
			const cv::Mat & laserScan,
			const LaserScanInfo & laserScanInfo,
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
			const cv::Mat & laserScan,
			const LaserScanInfo & laserScanInfo,
			const cv::Mat & left,
			const cv::Mat & right,
			const StereoCameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const cv::Mat & userData = cv::Mat());

	virtual ~SensorData() {}

	bool isValid() const {
		return !(_id == 0 &&
			_stamp == 0.0 &&
			_imageRaw.empty() &&
			_imageCompressed.empty() &&
			_depthOrRightRaw.empty() &&
			_depthOrRightCompressed.empty() &&
			_laserScanRaw.empty() &&
			_laserScanCompressed.empty() &&
			_cameraModels.size() == 0 &&
			!_stereoCameraModel.isValidForProjection() &&
			_userDataRaw.empty() &&
			_userDataCompressed.empty() &&
			_keypoints.size() == 0 &&
			_descriptors.empty());
	}

	int id() const {return _id;}
	void setId(int id) {_id = id;}
	double stamp() const {return _stamp;}
	void setStamp(double stamp) {_stamp = stamp;}
	const LaserScanInfo & laserScanInfo() const {return _laserScanInfo;}

	const cv::Mat & imageCompressed() const {return _imageCompressed;}
	const cv::Mat & depthOrRightCompressed() const {return _depthOrRightCompressed;}
	const cv::Mat & laserScanCompressed() const {return _laserScanCompressed;}

	const cv::Mat & imageRaw() const {return _imageRaw;}
	const cv::Mat & depthOrRightRaw() const {return _depthOrRightRaw;}
	const cv::Mat & laserScanRaw() const {return _laserScanRaw;}
	void setImageRaw(const cv::Mat & imageRaw) {_imageRaw = imageRaw;}
	void setDepthOrRightRaw(const cv::Mat & depthOrImageRaw) {_depthOrRightRaw =depthOrImageRaw;}
	void setLaserScanRaw(const cv::Mat & laserScanRaw, const LaserScanInfo & info) {_laserScanRaw =laserScanRaw;_laserScanInfo = info;}
	void setCameraModel(const CameraModel & model) {_cameraModels.clear(); _cameraModels.push_back(model);}
	void setCameraModels(const std::vector<CameraModel> & models) {_cameraModels = models;}
	void setStereoCameraModel(const StereoCameraModel & stereoCameraModel) {_stereoCameraModel = stereoCameraModel;}

	//for convenience
	cv::Mat depthRaw() const {return _depthOrRightRaw.type()!=CV_8UC1?_depthOrRightRaw:cv::Mat();}
	cv::Mat rightRaw() const {return _depthOrRightRaw.type()==CV_8UC1?_depthOrRightRaw:cv::Mat();}

	void uncompressData();
	void uncompressData(
			cv::Mat * imageRaw,
			cv::Mat * depthOrRightRaw,
			cv::Mat * laserScanRaw = 0,
			cv::Mat * userDataRaw = 0,
			cv::Mat * groundCellsRaw = 0,
			cv::Mat * obstacleCellsRaw = 0);
	void uncompressDataConst(
			cv::Mat * imageRaw,
			cv::Mat * depthOrRightRaw,
			cv::Mat * laserScanRaw = 0,
			cv::Mat * userDataRaw = 0,
			cv::Mat * groundCellsRaw = 0,
			cv::Mat * obstacleCellsRaw = 0) const;

	const std::vector<CameraModel> & cameraModels() const {return _cameraModels;}
	const StereoCameraModel & stereoCameraModel() const {return _stereoCameraModel;}

	void setUserDataRaw(const cv::Mat & userDataRaw); // only set raw
	/**
	 * Set user data. Detect automatically if raw or compressed. If raw, the data is
	 * compressed too. A matrix of type CV_8UC1 with 1 row is considered as compressed.
	 * If you have one dimension unsigned 8 bits raw data, make sure to transpose it
	 * (to have multiple rows instead of multiple columns) in order to be detected as
	 * not compressed.
	 */
	void setUserData(const cv::Mat & userData);
	const cv::Mat & userDataRaw() const {return _userDataRaw;}
	const cv::Mat & userDataCompressed() const {return _userDataCompressed;}

	// detect automatically if raw or compressed. If raw, the data will be compressed.
	void setOccupancyGrid(
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			float cellSize,
			const cv::Point3f & viewPoint);
	// remove raw occupancy grids
	void clearOccupancyGridRaw() {_groundCellsRaw = cv::Mat(); _obstacleCellsRaw = cv::Mat();}
	const cv::Mat & gridGroundCellsRaw() const {return _groundCellsRaw;}
	const cv::Mat & gridGroundCellsCompressed() const {return _groundCellsCompressed;}
	const cv::Mat & gridObstacleCellsRaw() const {return _obstacleCellsRaw;}
	const cv::Mat & gridObstacleCellsCompressed() const {return _obstacleCellsCompressed;}
	float gridCellSize() const {return _cellSize;}
	const cv::Point3f & gridViewPoint() const {return _viewPoint;}

	void setFeatures(const std::vector<cv::KeyPoint> & keypoints, const std::vector<cv::Point3f> & keypoints3D, const cv::Mat & descriptors);
	const std::vector<cv::KeyPoint> & keypoints() const {return _keypoints;}
	const std::vector<cv::Point3f> & keypoints3D() const {return _keypoints3D;}
	const cv::Mat & descriptors() const {return _descriptors;}

	void setGroundTruth(const Transform & pose) {groundTruth_ = pose;}
	const Transform & groundTruth() const {return groundTruth_;}

	long getMemoryUsed() const; // Return memory usage in Bytes

private:
	int _id;
	double _stamp;

	cv::Mat _imageCompressed;          // compressed image
	cv::Mat _depthOrRightCompressed;   // compressed image
	cv::Mat _laserScanCompressed;      // compressed data

	cv::Mat _imageRaw;          // CV_8UC1 or CV_8UC3
	cv::Mat _depthOrRightRaw;   // depth CV_16UC1 or CV_32FC1, right image CV_8UC1
	cv::Mat _laserScanRaw;      // CV_32FC2 or CV_32FC3

	std::vector<CameraModel> _cameraModels;
	StereoCameraModel _stereoCameraModel;

	LaserScanInfo _laserScanInfo;

	// user data
	cv::Mat _userDataCompressed;      // compressed data
	cv::Mat _userDataRaw;

	// occupancy grid
	cv::Mat _groundCellsCompressed;
	cv::Mat _obstacleCellsCompressed;
	cv::Mat _groundCellsRaw;
	cv::Mat _obstacleCellsRaw;
	float _cellSize;
	cv::Point3f _viewPoint;

	// features
	std::vector<cv::KeyPoint> _keypoints;
	std::vector<cv::Point3f> _keypoints3D;
	cv::Mat _descriptors;

	Transform groundTruth_;
};

}


#endif /* SENSORDATA_H_ */
