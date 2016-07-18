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

#ifndef UTIL3D_H_
#define UTIL3D_H_

#include "rtabmap/core/RtabmapExp.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/TextureMesh.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <opencv2/core/core.hpp>
#include <map>
#include <list>

namespace rtabmap
{

namespace util3d
{

cv::Mat RTABMAP_EXP rgbFromCloud(
		const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		bool bgrOrder = true);

cv::Mat RTABMAP_EXP depthFromCloud(
		const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		float & fx,
		float & fy,
		bool depth16U = true);

void RTABMAP_EXP rgbdFromCloud(
		const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		cv::Mat & rgb,
		cv::Mat & depth,
		float & fx,
		float & fy,
		bool bgrOrder = true,
		bool depth16U = true);

pcl::PointXYZ RTABMAP_EXP projectDepthTo3D(
		const cv::Mat & depthImage,
		float x, float y,
		float cx, float cy,
		float fx, float fy,
		bool smoothing,
		float maxZError = 0.02f);

RTABMAP_DEPRECATED (pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromDepth(
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0), "Use cloudFromDepth with CameraModel interface.");
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromDepth(
		const cv::Mat & imageDepth,
		const CameraModel & model,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0);

RTABMAP_DEPRECATED (pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0), "Use cloudFromDepthRGB with CameraModel interface.");
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		const CameraModel & model,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromDisparity(
		const cv::Mat & imageDisparity,
		const StereoCameraModel & model,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromDisparityRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDisparity,
		const StereoCameraModel & model,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromStereoImages(
		const cv::Mat & imageLeft,
		const cv::Mat & imageRight,
		const StereoCameraModel & model,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0,
		const ParametersMap & parameters = ParametersMap());

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromSensorData(
		const SensorData & sensorData,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0,
		const ParametersMap & parameters = ParametersMap());

/**
 * Create an RGB cloud from the images contained in SensorData. If there is only one camera,
 * the returned cloud is organized. Otherwise, all NaN
 * points are removed and the cloud will be dense.
 *
 * @param sensorData, the sensor data.
 * @param decimation, images are decimated by this factor before projecting points to 3D. The factor
 * should be a factor of the image width and height.
 * @param maxDepth, maximum depth of the projected points (farther points are set to null in case of an organized cloud).
 * @param minDepth, minimum depth of the projected points (closer points are set to null in case of an organized cloud).
 * @param validIndices, the indices of valid points in the cloud
 * @return a RGB cloud.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudRGBFromSensorData(
		const SensorData & sensorData,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0,
		const ParametersMap & parameters = ParametersMap());

pcl::PointCloud<pcl::PointXYZ> RTABMAP_EXP laserScanFromDepthImage(
					const cv::Mat & depthImage,
					float fx,
					float fy,
					float cx,
					float cy,
					float maxDepth = 0,
					float minDepth = 0,
					const Transform & localTransform = Transform::getIdentity());

// return CV_32FC3
cv::Mat RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform = Transform());
// return CV_32FC6
cv::Mat RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const Transform & transform = Transform());
// return CV_32FC2
cv::Mat RTABMAP_EXP laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform = Transform());
// For laserScan of type CV_32FC2, z is set to null.
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP laserScanToPointCloud(const cv::Mat & laserScan, const Transform & transform = Transform());
// For laserScan of type CV_32FC2 or CV_32FC3, normals are set to null.
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP laserScanToPointCloudNormal(const cv::Mat & laserScan, const Transform & transform = Transform());

cv::Point3f RTABMAP_EXP projectDisparityTo3D(
		const cv::Point2f & pt,
		float disparity,
		const StereoCameraModel & model);

cv::Point3f RTABMAP_EXP projectDisparityTo3D(
		const cv::Point2f & pt,
		const cv::Mat & disparity,
		const StereoCameraModel & model);

// Register point cloud to camera (return registered depth image)
cv::Mat RTABMAP_EXP projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK,  
		const cv::Mat & laserScan,                   // assuming points are already in /base_link coordinate
		const rtabmap::Transform & cameraTransform); // /base_link -> /camera_link

// Register point cloud to camera (return registered depth image)
cv::Mat RTABMAP_EXP projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK,                       
		const pcl::PointCloud<pcl::PointXYZ>::Ptr laserScan, // assuming points are already in /base_link coordinate
		const rtabmap::Transform & cameraTransform);         // /base_link -> /camera_link

// Direction vertical (>=0), horizontal (<0)
void RTABMAP_EXP fillProjectedCloudHoles(
		cv::Mat & depthRegistered,
		bool verticalDirection,
		bool fillToBorder);

bool RTABMAP_EXP isFinite(const cv::Point3f & pt);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP concatenateClouds(
		const std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clouds);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP concatenateClouds(
		const std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds);

pcl::TextureMesh::Ptr RTABMAP_EXP concatenateTextureMeshes(
		const std::list<pcl::TextureMesh::Ptr> & meshes);

/**
 * @brief Concatenate a vector of indices to a single vector.
 *
 * @param indices the vector of indices to concatenate.
 * @note This methods doesn't check if indices exist in the two set and doesn't
 * sort the output indices. If we are not sure if the the
 * two set of indices set are disjoint and/or you need sorted indices, the use of mergeIndices().
 * @return the indices concatenated.
 */
pcl::IndicesPtr RTABMAP_EXP concatenate(
		const std::vector<pcl::IndicesPtr> & indices);

/**
 * @brief Concatenate two vector of indices to a single vector.
 *
 * @param indicesA the first vector of indices to concatenate.
 * @param indicesB the second vector of indices to concatenate.
 * @note This methods doesn't check if indices exist in the two set and doesn't
 * sort the output indices. If we are not sure if the the
 * two set of indices set are disjoint and/or you need sorted indices, the use of mergeIndices().
 * @return the indices concatenated.
 */
pcl::IndicesPtr RTABMAP_EXP concatenate(
		const pcl::IndicesPtr & indicesA,
		const pcl::IndicesPtr & indicesB);

void RTABMAP_EXP savePCDWords(
		const std::string & fileName,
		const std::multimap<int, pcl::PointXYZ> & words,
		const Transform & transform = Transform::getIdentity());

void RTABMAP_EXP savePCDWords(
		const std::string & fileName,
		const std::multimap<int, cv::Point3f> & words,
		const Transform & transform = Transform::getIdentity());

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP loadBINCloud(const std::string & fileName, int dim);

// Load *.pcd, *.ply or *.bin (KITTI format) with optional filtering.
// If normals are computed (normalsK>0), the returned scan type is CV_32FC6 instead of CV_32FC3
cv::Mat RTABMAP_EXP loadScan(
		const std::string & path,
		const Transform & transform = Transform::getIdentity(),
		int downsampleStep = 1,
		float voxelSize = 0.0f,
		int normalsK = 0);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP loadCloud(
		const std::string & path,
		const Transform & transform = Transform::getIdentity(),
		int downsampleStep = 1,
		float voxelSize = 0.0f);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_H_ */
