/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include <rtabmap/core/Transform.h>
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
			const std::vector<unsigned char> & userData = std::vector<unsigned char>());

	// Mono constructor
	SensorData(
			const cv::Mat & image,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const std::vector<unsigned char> & userData = std::vector<unsigned char>());

	// RGB-D constructor
	SensorData(
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const std::vector<unsigned char> & userData = std::vector<unsigned char>());

	// RGB-D constructor + 2d laser scan
	SensorData(
			const cv::Mat & laserScan,
			int laserScanMaxPts,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const std::vector<unsigned char> & userData = std::vector<unsigned char>());

	// Multi-cameras RGB-D constructor
	SensorData(
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const std::vector<CameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const std::vector<unsigned char> & userData = std::vector<unsigned char>());

	// Multi-cameras RGB-D constructor + 2d laser scan
	SensorData(
			const cv::Mat & laserScan,
			int laserScanMaxPts,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const std::vector<CameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0,
			const std::vector<unsigned char> & userData = std::vector<unsigned char>());

	// Stereo constructor
	SensorData(
			const cv::Mat & left,
			const cv::Mat & right,
			const StereoCameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const std::vector<unsigned char> & userData = std::vector<unsigned char>());

	// Stereo constructor + 2d laser scan
	SensorData(
			const cv::Mat & laserScan,
			int laserScanMaxPts,
			const cv::Mat & left,
			const cv::Mat & right,
			const StereoCameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0,
			const std::vector<unsigned char> & userData = std::vector<unsigned char>());

	virtual ~SensorData() {}

	bool isValid() const {
		return !(_id == 0 &&
			_stamp == 0.0 &&
			_laserScanMaxPts == 0 &&
			_imageRaw.empty() &&
			_imageCompressed.empty() &&
			_depthOrRightRaw.empty() &&
			_depthOrRightCompressed.empty() &&
			_laserScanRaw.empty() &&
			_laserScanCompressed.empty() &&
			_cameraModels.size() == 0 &&
			!_stereoCameraModel.isValid() &&
			_userData.size() == 0 &&
			_keypoints.size() == 0 &&
			_descriptors.empty());
	}

	int id() const {return _id;}
	void setId(int id) {_id = id;}
	double stamp() const {return _stamp;}
	void setStamp(double stamp) {_stamp = stamp;}
	int laserScanMaxPts() const {return _laserScanMaxPts;}

	const cv::Mat & imageCompressed() const {return _imageCompressed;}
	const cv::Mat & depthOrRightCompressed() const {return _depthOrRightCompressed;}
	const cv::Mat & laserScanCompressed() const {return _laserScanCompressed;}

	const cv::Mat & imageRaw() const {return _imageRaw;}
	const cv::Mat & depthOrRightRaw() const {return _depthOrRightRaw;}
	const cv::Mat & laserScanRaw() const {return _laserScanRaw;}
	void setImageRaw(const cv::Mat & imageRaw) {_imageRaw = imageRaw;}
	void setDepthOrRightRaw(const cv::Mat & depthOrImageRaw) {_depthOrRightRaw =depthOrImageRaw;}
	void setLaserScanRaw(const cv::Mat & laserScanRaw, int laserScanMaxPts) {_laserScanRaw =laserScanRaw;_laserScanMaxPts = laserScanMaxPts;}

	//for convenience
	cv::Mat depthRaw() const {return _depthOrRightRaw.type()!=CV_8UC1?_depthOrRightRaw:cv::Mat();}
	cv::Mat rightRaw() const {return _depthOrRightRaw.type()==CV_8UC1?_depthOrRightRaw:cv::Mat();}

	void uncompressData();
	void uncompressData(cv::Mat * imageRaw, cv::Mat * depthOrRightRaw, cv::Mat * laserScanRaw);
	void uncompressDataConst(cv::Mat * imageRaw, cv::Mat * depthOrRightRaw, cv::Mat * laserScanRaw) const;

	const std::vector<CameraModel> & cameraModels() const {return _cameraModels;}
	StereoCameraModel stereoCameraModel() const {return _stereoCameraModel;}

	void setUserData(const std::vector<unsigned char> & data) {_userData = data;}
	const std::vector<unsigned char> & userData() const {return _userData;}

	void setFeatures(const std::vector<cv::KeyPoint> & keypoints, const cv::Mat & descriptors)
	{
		_keypoints = keypoints;
		_descriptors = descriptors;
	}
	const std::vector<cv::KeyPoint> & keypoints() const {return _keypoints;}
	const cv::Mat & descriptors() const {return _descriptors;}

private:
	int _id;
	double _stamp;
	int _laserScanMaxPts;

	cv::Mat _imageCompressed;          // compressed image
	cv::Mat _depthOrRightCompressed;   // compressed image
	cv::Mat _laserScanCompressed;      // compressed data

	cv::Mat _imageRaw;          // CV_8UC1 or CV_8UC3
	cv::Mat _depthOrRightRaw;   // depth CV_16UC1 or CV_32FC1, right image CV_8UC1
	cv::Mat _laserScanRaw;      // CV_32FC2

	std::vector<CameraModel> _cameraModels;
	StereoCameraModel _stereoCameraModel;

	// user data
	std::vector<unsigned char> _userData;

	// features
	std::vector<cv::KeyPoint> _keypoints;
	cv::Mat _descriptors;
};

}


#endif /* SENSORDATA_H_ */
