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

/**
 * @brief Applies a 3D transform to all points (and normals if present) in a LaserScan.
 *
 * This function transforms each point in the input `LaserScan` using the specified `Transform`.
 * The transformation is applied on a cloned copy of the scan data, preserving the original.
 * It supports both 2D and 3D scans, and if normals are present, they are also properly transformed.
 *
 * The transformation is only applied if it is neither null nor the identity transform.
 *
 * @param laserScan The input `LaserScan` object containing scan data to be transformed.
 * @param transform A `Transform` representing the spatial transformation to apply (translation + rotation).
 *                  Can be 3DoF or 6DoF depending on the scan dimensionality.
 *
 * @return A new `LaserScan` object with transformed points (and optionally normals), and the same metadata
 *         such as range limits, angle information, format, and local transform as the input scan.
 *
 * ### Behavior:
 * - If the transform is null or identity, the scan is returned unchanged.
 * - If the scan has normals (e.g., format is `kXYZNormal`, `kXYZINormal`, etc.), both positions and normals are transformed.
 * - If the scan has no normals, only point positions are transformed.
 * - Angle-based scans (2D with valid angle increment) retain angular properties in the returned object.
 * - The `localTransform` of the original scan is preserved in the returned scan.
 *
 * ### Supported Formats:
 * This function works with all valid formats defined by `LaserScan::Format`, including:
 * - 2D: `kXY`, `kXYI`, `kXYNormal`, `kXYINormal`
 * - 3D: `kXYZ`, `kXYZI`, `kXYZNormal`, `kXYZINormal`, `kXYZRGB`, `kXYZRGBNormal`, `kXYZIT`
 *
 * @see LaserScan, Transform, util3d::transformPoint()
 */
LaserScan RTABMAP_CORE_EXPORT transformLaserScan(
		const LaserScan & laserScan,
		const Transform & transform);

/**
 * @defgroup TransformPointcloud Transform PCL Point Clouds
 * @brief Transforms various PCL point cloud types using the given transform.
 * 
 * - For types with normals, the normals are transformed appriopriatly.
 * - If indices are provided, the returned point cloud contains only points matching the indices.
 * 
 * @param cloud The input point cloud.
 * @param indices Optional subset of indices of the points to transform.
 * @param transform The transform to apply.
 * @return A new point cloud with the transform applied.
 */
/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointXYZ` point cloud type.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & transform);
/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointXYZI` point cloud type.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const Transform & transform);
/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointXYZRGB` point cloud type.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & transform);
/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointNormal` point cloud type.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const Transform & transform);
/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointXYZRGBNormal` point cloud type.
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const Transform & transform);
/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointXYZINormal` point cloud type.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const Transform & transform);

/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointXYZ` point cloud type with specified indices.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);
/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointXYZI` point cloud type with specified indices.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);
/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointXYZRGB` point cloud type with specified indices.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);
/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointNormal` point cloud type with specified indices.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);
/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointXYZRGBNormal` point cloud type with specified indices.
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);
/**
 * @ingroup TransformPointcloud
 * @brief Transforms `pcl::PointXYZINormal` point cloud type with specified indices.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform);

/**
 * @defgroup TransformPoint Transform a single point
 * @brief Transforms various point types using the given transform.
 * 
 * - For types with normal, the normal is transformed appriopriatly.
 * 
 * @param pt The input point.
 * @param transform The transform to apply.
 * @return A new point with the transform applied.
 */
/**
 * @ingroup TransformPoint
 * @brief Transforms `cv::Point3f` point type.
 */
cv::Point3f RTABMAP_CORE_EXPORT transformPoint(
		const cv::Point3f & pt,
		const Transform & transform);
/**
 * @ingroup TransformPoint
 * @brief Transforms `cv::Point3d` point type.
 */
cv::Point3d RTABMAP_CORE_EXPORT transformPoint(
		const cv::Point3d & pt,
		const Transform & transform);
/**
 * @ingroup TransformPoint
 * @brief Transforms `pcl::PointXYZ` point type.
 */
pcl::PointXYZ RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointXYZ & pt,
		const Transform & transform);
/**
 * @ingroup TransformPoint
 * @brief Transforms `pcl::PointXYZI` point type.
 */
pcl::PointXYZI RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointXYZI & pt,
		const Transform & transform);
/**
 * @ingroup TransformPoint
 * @brief Transforms `pcl::PointXYZRGB` point type.
 */
pcl::PointXYZRGB RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointXYZRGB & pt,
		const Transform & transform);
/**
 * @ingroup TransformPoint
 * @brief Transforms `pcl::PointNormal` point type.
 */
pcl::PointNormal RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointNormal & point,
		const Transform & transform);
/**
 * @ingroup TransformPoint
 * @brief Transforms `pcl::PointXYZRGBNormal` point type.
 */
pcl::PointXYZRGBNormal RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointXYZRGBNormal & point,
		const Transform & transform);
/**
 * @ingroup TransformPoint
 * @brief Transforms `pcl::PointXYZINormal` point type.
 */
pcl::PointXYZINormal RTABMAP_CORE_EXPORT transformPoint(
		const pcl::PointXYZINormal & point,
		const Transform & transform);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_TRANSFORMS_H_ */
