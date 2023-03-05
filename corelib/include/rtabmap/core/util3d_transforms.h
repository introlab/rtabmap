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

#ifndef UTIL3D_TRANSFORMS_H_
#define UTIL3D_TRANSFORMS_H_

#include <rtabmap/core/rtabmap_core_export.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/LaserScan.h>

namespace rtabmap
{

namespace util3d
{

LaserScan RTABMAP_CORE_EXPORT transformLaserScan(
		const LaserScan & laserScan,
		const Transform & transform);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & transform);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const Transform & transform);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & transform);
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const Transform & transform);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const Transform & transform);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const Transform & transform);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);

cv::Point3f RTABMAP_CORE_EXPORT transformPoint(
		const cv::Point3f & pt,
		const Transform & transform);
cv::Point3d RTABMAP_CORE_EXPORT transformPoint(
		const cv::Point3d & pt,
		const Transform & transform);
pcl::PointXYZ RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointXYZ & pt,
		const Transform & transform);
pcl::PointXYZI RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointXYZI & pt,
		const Transform & transform);
pcl::PointXYZRGB RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointXYZRGB & pt,
		const Transform & transform);
pcl::PointNormal RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointNormal & point,
		const Transform & transform);
pcl::PointXYZRGBNormal RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointXYZRGBNormal & point,
		const Transform & transform);
pcl::PointXYZINormal RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointXYZINormal & point,
		const Transform & transform);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_TRANSFORMS_H_ */
