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
#include <rtabmap/core/ProgressState.h>
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
		float depthErrorRatio = 0.02f);

Eigen::Vector3f RTABMAP_EXP projectDepthTo3DRay(
		const cv::Size & imageSize,
		float x, float y,
		float cx, float cy,
		float fx, float fy);

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
		const ParametersMap & stereoParameters = ParametersMap(),
		const std::vector<float> & roiRatios = std::vector<float>()); // ignored for stereo

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
 * @param roiRatios, [left, right, top, bottom] region of interest (in ratios) of the image projected.
 * @return a RGB cloud.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudRGBFromSensorData(
		const SensorData & sensorData,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0,
		const ParametersMap & stereoParameters = ParametersMap(),
		const std::vector<float> & roiRatios = std::vector<float>()); // ignored for stereo

/**
 * Simulate a laser scan rotating counterclockwise, using middle line of the depth image.
 */
pcl::PointCloud<pcl::PointXYZ> RTABMAP_EXP laserScanFromDepthImage(
					const cv::Mat & depthImage,
					float fx,
					float fy,
					float cx,
					float cy,
					float maxDepth = 0,
					float minDepth = 0,
					const Transform & localTransform = Transform::getIdentity());
/**
 * Simulate a laser scan rotating counterclockwise, using middle line of the depth images.
 * The last value of the scan is the most left value of the first depth image. The first value of the scan is the most right value of the last depth image.
 *
 */
pcl::PointCloud<pcl::PointXYZ> RTABMAP_EXP laserScanFromDepthImages(
		const cv::Mat & depthImages,
		const std::vector<CameraModel> & cameraModels,
		float maxDepth,
		float minDepth);

template<typename PointCloud2T>
LaserScan laserScanFromPointCloud(const PointCloud2T & cloud, bool filterNaNs = true, bool is2D = false, const Transform & transform = Transform());
// return CV_32FC3  (x,y,z)
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);
// return CV_32FC6 (x,y,z,normal_x,normal_y,normal_z)
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform = Transform(), bool filterNaNs = true);
// return CV_32FC4 (x,y,z,rgb)
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);
// return CV_32FC4 (x,y,z,I)
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);
// return CV_32FC7 (x,y,z,rgb,normal_x,normal_y,normal_z)
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform = Transform(), bool filterNaNs = true);
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);
// return CV_32FC7 (x,y,z,I,normal_x,normal_y,normal_z)
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform = Transform(), bool filterNaNs = true);
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
LaserScan RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);
// return CV_32FC2  (x,y)
LaserScan RTABMAP_EXP laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
// return CV_32FC3  (x,y,I)
LaserScan RTABMAP_EXP laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
// return CV_32FC5  (x,y,normal_x, normal_y, normal_z)
LaserScan RTABMAP_EXP laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
LaserScan RTABMAP_EXP laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform = Transform(), bool filterNaNs = true);
// return CV_32FC6  (x,y,I,normal_x, normal_y, normal_z)
LaserScan RTABMAP_EXP laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
LaserScan RTABMAP_EXP laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform = Transform(), bool filterNaNs = true);

pcl::PCLPointCloud2::Ptr RTABMAP_EXP laserScanToPointCloud2(const LaserScan & laserScan, const Transform & transform = Transform());
// For 2d laserScan, z is set to null.
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP laserScanToPointCloud(const LaserScan & laserScan, const Transform & transform = Transform());
// For laserScan without normals, normals are set to null.
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP laserScanToPointCloudNormal(const LaserScan & laserScan, const Transform & transform = Transform());
// For laserScan without rgb, rgb is set to default r,g,b parameters.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP laserScanToPointCloudRGB(const LaserScan & laserScan, const Transform & transform = Transform(), unsigned char r = 100, unsigned char g = 100, unsigned char b = 100);
// For laserScan without intensity, intensity is set to intensity parameter.
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_EXP laserScanToPointCloudI(const LaserScan & laserScan, const Transform & transform = Transform(), float intensity = 0.0f);
// For laserScan without rgb, rgb is set to default r,g,b parameters.
// For laserScan without normals, normals are set to null.
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP laserScanToPointCloudRGBNormal(const LaserScan & laserScan, const Transform & transform = Transform(), unsigned char r = 100, unsigned char g = 100, unsigned char b = 100);
// For laserScan without intensity, intensity is set to default intensity parameter.
// For laserScan without normals, normals are set to null.
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_EXP laserScanToPointCloudINormal(const LaserScan & laserScan, const Transform & transform = Transform(), float intensity = 0.0f);

// For 2d laserScan, z is set to null.
pcl::PointXYZ RTABMAP_EXP laserScanToPoint(const LaserScan & laserScan, int index);
// For laserScan without normals, normals are set to null.
pcl::PointNormal RTABMAP_EXP laserScanToPointNormal(const LaserScan & laserScan, int index);
// For laserScan without rgb, rgb is set to default r,g,b parameters.
pcl::PointXYZRGB RTABMAP_EXP laserScanToPointRGB(const LaserScan & laserScan, int index, unsigned char r = 100, unsigned char g = 100, unsigned char b = 100);
// For laserScan without intensity, intensity is set to intensity parameter.
pcl::PointXYZI RTABMAP_EXP laserScanToPointI(const LaserScan & laserScan, int index, float intensity);
// For laserScan without rgb, rgb is set to default r,g,b parameters.
// For laserScan without normals, normals are set to null.
pcl::PointXYZRGBNormal RTABMAP_EXP laserScanToPointRGBNormal(const LaserScan & laserScan, int index, unsigned char r, unsigned char g, unsigned char b);
// For laserScan without intensity, intensity is set to default intensity parameter.
// For laserScan without normals, normals are set to null.
pcl::PointXYZINormal RTABMAP_EXP laserScanToPointINormal(const LaserScan & laserScan, int index, float intensity);

void RTABMAP_EXP getMinMax3D(const cv::Mat & laserScan, cv::Point3f & min, cv::Point3f & max);
void RTABMAP_EXP getMinMax3D(const cv::Mat & laserScan, pcl::PointXYZ & min, pcl::PointXYZ & max);

cv::Point3f RTABMAP_EXP projectDisparityTo3D(
		const cv::Point2f & pt,
		float disparity,
		const StereoCameraModel & model);

cv::Point3f RTABMAP_EXP projectDisparityTo3D(
		const cv::Point2f & pt,
		const cv::Mat & disparity,
		const StereoCameraModel & model);

// Register point cloud to camera (return registered depth image 32FC1)
cv::Mat RTABMAP_EXP projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK,  
		const cv::Mat & laserScan,                   // assuming points are already in /base_link coordinate
		const rtabmap::Transform & cameraTransform); // /base_link -> /camera_link

// Register point cloud to camera (return registered depth image 32FC1)
cv::Mat RTABMAP_EXP projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr laserScan, // assuming points are already in /base_link coordinate
		const rtabmap::Transform & cameraTransform);         // /base_link -> /camera_link

// Register point cloud to camera (return registered depth image 32FC1)
cv::Mat RTABMAP_EXP projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK,                       
		const pcl::PCLPointCloud2::Ptr laserScan, 			 // assuming points are already in /base_link coordinate
		const rtabmap::Transform & cameraTransform);         // /base_link -> /camera_link

// Direction vertical (>=0), horizontal (<0)
void RTABMAP_EXP fillProjectedCloudHoles(
		cv::Mat & depthRegistered,
		bool verticalDirection,
		bool fillToBorder);

/**
 * For each point, return pixel of the best camera (NodeID->CameraIndex)
 * looking at it based on the policy and parameters
 */
std::vector<std::pair< std::pair<int, int>, pcl::PointXY> > RTABMAP_EXP projectCloudToCameras (
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud,
		const std::map<int, Transform> & cameraPoses,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		float maxDistance = 0.0f,
		float maxAngle = 0.0f,
		const std::vector<float> & roiRatios = std::vector<float>(),
		const cv::Mat & projMask = cv::Mat(),
		bool distanceToCamPolicy = false,
		const ProgressState * state = 0);
/**
 * For each point, return pixel of the best camera (NodeID->CameraIndex)
 * looking at it based on the policy and parameters
 */
std::vector<std::pair< std::pair<int, int>, pcl::PointXY> > RTABMAP_EXP projectCloudToCameras (
		const pcl::PointCloud<pcl::PointXYZINormal> & cloud,
		const std::map<int, Transform> & cameraPoses,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		float maxDistance = 0.0f,
		float maxAngle = 0.0f,
		const std::vector<float> & roiRatios = std::vector<float>(),
		const cv::Mat & projMask = cv::Mat(),
		bool distanceToCamPolicy = false,
		const ProgressState * state = 0);

bool RTABMAP_EXP isFinite(const cv::Point3f & pt);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP concatenateClouds(
		const std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clouds);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP concatenateClouds(
		const std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds);

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

/**
 * Assume KITTI velodyne format
 * Return scan 4 channels (format=XYZI).
 */
cv::Mat RTABMAP_EXP loadBINScan(const std::string & fileName);
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP loadBINCloud(const std::string & fileName);
RTABMAP_DEPRECATED(pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP loadBINCloud(const std::string & fileName, int dim), "Use interface without dim argument.");

// Load *.pcd, *.ply or *.bin (KITTI format).
LaserScan RTABMAP_EXP loadScan(const std::string & path);

RTABMAP_DEPRECATED(pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP loadCloud(
		const std::string & path,
		const Transform & transform = Transform::getIdentity(),
		int downsampleStep = 1,
		float voxelSize = 0.0f), "Use loadScan() instead.");

} // namespace util3d
} // namespace rtabmap

#include "rtabmap/core/impl/util3d.hpp"

#endif /* UTIL3D_H_ */
