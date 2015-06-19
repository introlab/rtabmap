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

#ifndef UTIL3D_H_
#define UTIL3D_H_

#include "rtabmap/core/RtabmapExp.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
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

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromDepth(
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation = 1);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation = 1);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromDisparity(
		const cv::Mat & imageDisparity,
		float cx, float cy,
		float fx, float baseline,
		int decimation = 1);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromDisparityRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDisparity,
		float cx, float cy,
		float fx, float baseline,
		int decimation = 1);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromStereoImages(
		const cv::Mat & imageLeft,
		const cv::Mat & imageRight,
		float cx, float cy,
		float fx, float baseline,
		int decimation = 1);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromSensorData(
		const SensorData & sensorData,
		int decimation = 1,
		float maxDepth = 0.0f,
		float voxelSize = 0.0f,
		int samples = 0);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudRGBFromSensorData(
		const SensorData & sensorData,
		int decimation = 1,
		float maxDepth = 0.0f,
		float voxelSize = 0.0f,
		int samples = 0);

pcl::PointCloud<pcl::PointXYZ> RTABMAP_EXP laserScanFromDepthImage(
					const cv::Mat & depthImage,
					float fx,
					float fy,
					float cx,
					float cy,
					float maxDepth = 0,
					const Transform & localTransform = Transform::getIdentity());

cv::Mat RTABMAP_EXP cvtDepthFromFloat(const cv::Mat & depth32F);
cv::Mat RTABMAP_EXP cvtDepthToFloat(const cv::Mat & depth16U);

cv::Mat RTABMAP_EXP laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP laserScanToPointCloud(const cv::Mat & laserScan);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cvMat2Cloud(
		const cv::Mat & matrix,
		const Transform & tranform = Transform::getIdentity());

pcl::PointXYZ RTABMAP_EXP projectDisparityTo3D(
		const cv::Point2f & pt,
		float disparity,
		float cx, float cy, float fx, float baseline);

pcl::PointXYZ RTABMAP_EXP projectDisparityTo3D(
		const cv::Point2f & pt,
		const cv::Mat & disparity,
		float cx, float cy, float fx, float baseline);

cv::Mat RTABMAP_EXP depthFromDisparity(const cv::Mat & disparity,
		float fx, float baseline,
		int type = CV_32FC1);

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

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_H_ */
