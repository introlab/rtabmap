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

#ifndef UTIL3D_MAPPING_H_
#define UTIL3D_MAPPING_H_

#include "rtabmap/core/RtabmapExp.h"

#include <opencv2/core/core.hpp>
#include <map>
#include <rtabmap/core/Transform.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rtabmap
{

namespace util3d
{

RTABMAP_DEPRECATED(void RTABMAP_EXP occupancy2DFromLaserScan(
		const cv::Mat & scan, // in /base_link frame
		cv::Mat & empty,
		cv::Mat & occupied,
		float cellSize,
		bool unknownSpaceFilled = false,
		float scanMaxRange = 0.0f), "Use interface with \"viewpoint\" parameter to make sure the ray tracing origin is from the sensor and not the base.");

RTABMAP_DEPRECATED(void RTABMAP_EXP occupancy2DFromLaserScan(
		const cv::Mat & scan, // in /base_link frame
		const cv::Point3f & viewpoint, // /base_link -> /base_scan
		cv::Mat & empty,
		cv::Mat & occupied,
		float cellSize,
		bool unknownSpaceFilled = false,
		float scanMaxRange = 0.0f), "Use interface with scanHit/scanNoHit parameters: scanNoHit set to null matrix has the same functionality than this method.");

void RTABMAP_EXP occupancy2DFromLaserScan(
		const cv::Mat & scanHit, // in /base_link frame
		const cv::Mat & scanNoHit, // in /base_link frame
		const cv::Point3f & viewpoint, // /base_link -> /base_scan
		cv::Mat & empty,
		cv::Mat & occupied,
		float cellSize,
		bool unknownSpaceFilled = false,
		float scanMaxRange = 0.0f); // would be set if unknownSpaceFilled=true

cv::Mat RTABMAP_EXP create2DMapFromOccupancyLocalMaps(
		const std::map<int, Transform> & poses,
		const std::map<int, std::pair<cv::Mat, cv::Mat> > & occupancy,
		float cellSize,
		float & xMin,
		float & yMin,
		float minMapSize = 0.0f,
		bool erode = false,
		float footprintRadius = 0.0f);

RTABMAP_DEPRECATED(cv::Mat RTABMAP_EXP create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans, // in /base_link frame
		float cellSize,
		bool unknownSpaceFilled,
		float & xMin,
		float & yMin,
		float minMapSize = 0.0f,
		float scanMaxRange = 0.0f), "Use interface with \"viewpoints\" parameter to make sure the ray tracing origin is from the sensor and not the base.");

RTABMAP_DEPRECATED(cv::Mat RTABMAP_EXP create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans, // in /base_link frame
		const std::map<int, cv::Point3f > & viewpoints, // /base_link -> /base_scan
		float cellSize,
		bool unknownSpaceFilled,
		float & xMin,
		float & yMin,
		float minMapSize = 0.0f,
		float scanMaxRange = 0.0f), "Use interface with cv::Mat scans.");

/**
 * Create 2d Occupancy grid (CV_8S)
 * -1 = unknown
 * 0 = empty space
 * 100 = obstacle
 * @param poses
 * @param scans, should be CV_32FC2 type!
 * @param viewpoints
 * @param cellSize m
 * @param unknownSpaceFilled if false no fill, otherwise a virtual laser sweeps the unknown space from each pose (stopping on detected obstacle)
 * @param xMin
 * @param yMin
 * @param minMapSize minimum map size in meters
 * @param scanMaxRange laser scan maximum range, would be set if unknownSpaceFilled=true
 */
cv::Mat RTABMAP_EXP create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, std::pair<cv::Mat, cv::Mat> > & scans, // <id, <hit, no hit> >, in /base_link frame
		const std::map<int, cv::Point3f > & viewpoints, // /base_link -> /base_scan
		float cellSize,
		bool unknownSpaceFilled,
		float & xMin,
		float & yMin,
		float minMapSize = 0.0f,
		float scanMaxRange = 0.0f); // would be set if unknownSpaceFilled=true

void RTABMAP_EXP rayTrace(const cv::Point2i & start,
		const cv::Point2i & end,
		cv::Mat & grid,
		bool stopOnObstacle);

cv::Mat RTABMAP_EXP convertMap2Image8U(const cv::Mat & map8S, bool pgmFormat = false);
cv::Mat RTABMAP_EXP convertImage8U2Map(const cv::Mat & map8U, bool pgmFormat = false);

cv::Mat RTABMAP_EXP erodeMap(const cv::Mat & map);

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr projectCloudOnXYPlane(
		const typename pcl::PointCloud<PointT> & cloud);

// templated methods
template<typename PointT>
void segmentObstaclesFromGround(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		pcl::IndicesPtr & ground,
		pcl::IndicesPtr & obstacles,
		int normalKSearch,
		float groundNormalAngle,
		float clusterRadius,
		int minClusterSize,
		bool segmentFlatObstacles = false,
		float maxGroundHeight = 0.0f,
		pcl::IndicesPtr * flatObstacles = 0,
		const Eigen::Vector4f & viewPoint = Eigen::Vector4f(0,0,100,0),
		float groundNormalsUp = 0);
template<typename PointT>
void segmentObstaclesFromGround(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		pcl::IndicesPtr & ground,
		pcl::IndicesPtr & obstacles,
		int normalKSearch,
		float groundNormalAngle,
		float clusterRadius,
		int minClusterSize,
		bool segmentFlatObstacles = false,
		float maxGroundHeight = 0.0f,
		pcl::IndicesPtr * flatObstacles = 0,
		const Eigen::Vector4f & viewPoint = Eigen::Vector4f(0,0,100,0),
		float groundNormalsUp = 0);

template<typename PointT>
void occupancy2DFromGroundObstacles(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & groundIndices,
		const pcl::IndicesPtr & obstaclesIndices,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize);

template<typename PointT>
void occupancy2DFromGroundObstacles(
		const typename pcl::PointCloud<PointT>::Ptr & groundCloud,
		const typename pcl::PointCloud<PointT>::Ptr & obstaclesCloud,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize);

template<typename PointT>
void occupancy2DFromCloud3D(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize = 0.05f,
		float groundNormalAngle = M_PI_4,
		int minClusterSize = 20,
		bool segmentFlatObstacles = false,
		float maxGroundHeight = 0.0f);
template<typename PointT>
void occupancy2DFromCloud3D(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize = 0.05f,
		float groundNormalAngle = M_PI_4,
		int minClusterSize = 20,
		bool segmentFlatObstacles = false,
		float maxGroundHeight = 0.0f);

} // namespace util3d
} // namespace rtabmap

#include "rtabmap/core/impl/util3d_mapping.hpp"

#endif /* UTIL3D_MAPPING_H_ */
