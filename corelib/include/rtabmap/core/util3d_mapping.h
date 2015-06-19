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

void RTABMAP_EXP occupancy2DFromLaserScan(
		const cv::Mat & scan,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize);

cv::Mat RTABMAP_EXP create2DMapFromOccupancyLocalMaps(
		const std::map<int, Transform> & poses,
		const std::map<int, std::pair<cv::Mat, cv::Mat> > & occupancy,
		float cellSize,
		float & xMin,
		float & yMin,
		float minMapSize = 0.0f,
		bool erode = false);

cv::Mat RTABMAP_EXP create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans,
		float cellSize,
		bool unknownSpaceFilled,
		float & xMin,
		float & yMin,
		float minMapSize = 0.0f);

void RTABMAP_EXP rayTrace(const cv::Point2i & start,
		const cv::Point2i & end,
		cv::Mat & grid,
		bool stopOnObstacle);

cv::Mat RTABMAP_EXP convertMap2Image8U(const cv::Mat & map8S);

void RTABMAP_EXP projectCloudOnXYPlane(
		pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

// templated methods
template<typename PointT>
void segmentObstaclesFromGround(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		pcl::IndicesPtr & ground,
		pcl::IndicesPtr & obstacles,
		float normalRadiusSearch,
		float groundNormalAngle,
		int minClusterSize,
		bool segmentFlatObstacles = false);

template<typename PointT>
void occupancy2DFromCloud3D(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize = 0.05f,
		float groundNormalAngle = M_PI_4,
		int minClusterSize = 20);

} // namespace util3d
} // namespace rtabmap

#include "rtabmap/core/impl/util3d_mapping.hpp"

#endif /* UTIL3D_MAPPING_H_ */
