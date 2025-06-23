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

#ifndef UTIL3D_FILTERING_H_
#define UTIL3D_FILTERING_H_

#include <rtabmap/core/rtabmap_core_export.h>
#include <rtabmap/core/Transform.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <rtabmap/core/LaserScan.h>

namespace rtabmap
{

namespace util3d
{

/**
 * @brief Applies a common set of filters to a LaserScan, including downsampling, range limits, voxel grid filtering, and normal estimation.
 *
 * This function performs a sequence of optional preprocessing steps on the input LaserScan:
 * - Downsampling: Reduces the number of points by selecting every N-th point.
 * - Range filtering: Removes points outside the specified minimum and maximum range.
 * - Voxel grid filtering: Reduces point density using a voxel grid.
 * - Normal estimation: Computes surface normals using k-nearest neighbors or radius search.
 * - Normal reorientation: Optionally orients normals upward if this condition is fulfilled: for each normal, 
 *   if normal.z < `-groundNormalsUp` and the corresponding point is below viewpoint, it is flipped upward.
 *
 * The function supports both 2D and 3D scans and adapts behavior based on whether the scan contains RGB or intensity data.
 * 
 * Depending on the filtering options used, the output point cloud may be dense if the input is organized.
 *
 * @param scanIn Input LaserScan to be filtered.
 * @param downsamplingStep Step size for downsampling. A value >1 will reduce the scan resolution. For organized 
 *                         scans, only the largest dimension is downsampled and the result is dense. For example, 
 *                         if the input organized scan is 16x1024, the resulting scan will be 1x8192 (if all values are valid).
 * @param rangeMin Minimum range to keep points from the scan viewpoint. Points closer than this value will be discarded. Output point cloud will be dense. Set to 0 to disable.
 * @param rangeMax Maximum range to keep points from the scan viewpoint. Points farther than this value will be discarded. Output point cloud will be dense. Set to 0 to disable.
 * @param voxelSize Size of the voxel grid in meters. A value >0 enables voxel filtering. Output point cloud will be dense.
 * @param normalK Number of nearest neighbors to use for normal estimation. Set to 0 to disable.
 * @param normalRadius Radius used for normal estimation. Set to 0 to disable.
 * @param groundNormalsUp If >0, normal vectors close to -Z axis will be oriented upward (+Z). Expected value 
 *                        is around `0.8`.
 *
 * @return A new LaserScan instance with the applied filters and potential normals.
 * @see adjustNormalsToViewPoint()
 * 
 *
 */
LaserScan RTABMAP_CORE_EXPORT commonFiltering(
		const LaserScan & scan,
		int downsamplingStep,
		float rangeMin = 0.0f,
		float rangeMax = 0.0f,
		float voxelSize = 0.0f,
		int normalK = 0,
		float normalRadius = 0.0f,
		float groundNormalsUp = 0.0f);
/**
 * @brief Applies a common set of filters to a LaserScan, including downsampling, range limits, voxel grid filtering, and normal estimation.
 * @deprecated Use version with groundNormalsUp as float. For forceGroundNormalsUp=true, set groundNormalsUpAngle=0.8, otherwise set groundNormalsUpAngle=0.0. 
 */ 
RTABMAP_DEPRECATED LaserScan RTABMAP_CORE_EXPORT commonFiltering(
		const LaserScan & scan,
		int downsamplingStep,
		float rangeMin,
		float rangeMax,
		float voxelSize,
		int normalK,
		float normalRadius,
		bool forceGroundNormalsUp);

/**
 * @brief Filters a LaserScan data on a minimum and maximum Euclidean range.
 *
 * This function removes scan points from the input LaserScan that fall
 * outside the specified minimum and maximum range (in meters) from the LaserScan's viewpoint.
 *
 * @param scan The input LaserScan object containing the scan data.
 * @param rangeMin The minimum range threshold. Points closer than this will be excluded.
 * @param rangeMax The maximum range threshold. Points farther than this will be excluded.
 * @return A new LaserScan object containing only the points within the specified range.
 *         If the input scan is empty or the range limits are both zero, the original scan is returned.
 *
 * @note The function handles both 2D and 3D scans based on the `scan.is2d()` flag. 
 *       The function doesn't keep the scan organized if the input is.
 * @throws Assertion failure if either `rangeMin` or `rangeMax` is negative.
 */
LaserScan RTABMAP_CORE_EXPORT rangeFiltering(
		const LaserScan & scan,
		float rangeMin,
		float rangeMax);

/**
 * @defgroup PointCloudRangeFiltering Range Filtering of PCL Point Clouds
 * @brief Filters a point cloud based on a minimum and maximum Euclidean range.
 *
 * This templated function computes the squared Euclidean distance of each point
 * in the given point cloud and returns indices of points whose distance falls
 * within the specified range limits. If no indices are provided, the entire cloud
 * is evaluated.
 *
 * @param cloud The input point cloud to be filtered.
 * @param indices The subset of point indices to evaluate. If empty, the full cloud is used.
 * @param rangeMin Minimum distance threshold. Points closer than this are excluded.
 * @param rangeMax Maximum distance threshold. Points farther than this are excluded.
 * @return pcl::IndicesPtr Pointer to a vector of indices that passed the range filter.
 *
 * @note Both rangeMin and rangeMax must be non-negative. If both are zero, no filtering is applied.
 * @throws Assertion failure if rangeMin or rangeMax is negative.
 */
/**
 * @ingroup PointCloudRangeFiltering
 * @brief Filters a point cloud of type `pcl::PointXYZ`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT rangeFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float rangeMin,
		float rangeMax);
/**
 * @ingroup PointCloudRangeFiltering
 * @brief Filters a point cloud of type `pcl::PointXYZRGB`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT rangeFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float rangeMin,
		float rangeMax);
/**
 * @ingroup PointCloudRangeFiltering
 * @brief Filters a point cloud of type `pcl::PointNormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT rangeFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float rangeMin,
		float rangeMax);
/**
 * @ingroup PointCloudRangeFiltering
 * @brief Filters a point cloud of type `pcl::PointXYZRGBNormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT rangeFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float rangeMin,
		float rangeMax);

/**
 * @defgroup PointCloudSplitRangeFiltering Split Range Filtering of PCL Point Clouds
 * @brief Splits a point cloud into two groups based on a distance threshold.
 *
 * This function divides the input point cloud (or a subset specified by indices)
 * into two sets of indices: points closer than the given range (`closeIndices`)
 * and points farther than or equal to the range (`farIndices`), using Euclidean
 * distance.
 *
 * @param cloud The input point cloud.
 * @param indices The subset of point indices to consider. If empty, the full cloud is used.
 * @param range The distance threshold in meters used to classify points as "close" or "far".
 * @param[out] closeIndices Output indices for points with distance < range.
 * @param[out] farIndices Output indices for points with distance >= range.
 *
 * @note The function resets and fills both output index containers.
 *       If the input cloud is empty, both outputs will also be empty.
 */
/**
 * @ingroup PointCloudSplitRangeFiltering
 * @brief Splits a point cloud of type `pcl::PointXYZ`.
 */
void RTABMAP_CORE_EXPORT rangeSplitFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float range, 
		pcl::IndicesPtr & closeIndices,
		pcl::IndicesPtr & farIndices);
/**
 * @ingroup PointCloudSplitRangeFiltering
 * @brief Splits a point cloud of type `pcl::PointXYZRGB`.
 */
void RTABMAP_CORE_EXPORT rangeSplitFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float range,
		pcl::IndicesPtr & closeIndices,
		pcl::IndicesPtr & farIndices);
/**
 * @ingroup PointCloudSplitRangeFiltering
 * @brief Splits a point cloud of type `pcl::PointNormal`.
 */
void RTABMAP_CORE_EXPORT rangeSplitFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float range,
		pcl::IndicesPtr & closeIndices,
		pcl::IndicesPtr & farIndices);
/**
 * @ingroup PointCloudSplitRangeFiltering
 * @brief Splits a point cloud of type `pcl::PointXYZRGBNormal`.
 */
void RTABMAP_CORE_EXPORT rangeSplitFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float range,
		pcl::IndicesPtr & closeIndices,
		pcl::IndicesPtr & farIndices);

/**
 * @defgroup PointCloudDownSampling Point Cloud Downsampling
 * @brief Downsamples a point cloud or LaserScan by keeping only every N-th point.
 *
 * This function downsamples a point cloud based on a specified step size. The method adapts based on
 * whether the cloud is organized (2D) or unorganized (1D), and whether it's structured like a depth image or LiDAR scan.
 * 
 * - For **unorganized point clouds**: Every `step`-th point is retained linearly.
 * - For **organized point clouds** with LiDAR-like layout (e.g., 2048x64): Downsampling is performed along the "long" dimension,
 *   preserving the ring structure.
 * - For **depth-image-like organized clouds** (e.g., 640x480): Downsampling is performed in both row and column directions,
 *   similar to image decimation. Step size must divide both width and height exactly.
 *
 * @param cloud The input point cloud to downsample.
 * @param step The decimation step. Must be greater than 0.
 * @return A new point cloud containing only the sampled points.
 *
 * @note If `step <= 1` or the cloud has fewer points than `step`, the function returns a copy of the input cloud.
 * @throws Assertion failure if `step <= 0` or, in the case of depth-image-style clouds, if the width and height are not divisible by `step`.
 */
/**
 * @ingroup PointCloudDownSampling
 * @brief Downsamples a LaserScan.
 */
LaserScan RTABMAP_CORE_EXPORT downsample(
	const LaserScan & cloud,
	int step);
/**
 * @ingroup PointCloudDownSampling
 * @brief Downsamples a point cloud of type `pcl::PointXYZ`.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT downsample(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int step);
/**
 * @ingroup PointCloudDownSampling
 * @brief Downsamples a point cloud of type `pcl::PointXYZRGB`.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT downsample(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int step);
/**
 * @ingroup PointCloudDownSampling
 * @brief Downsamples a point cloud of type `pcl::PointXYZI`.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT downsample(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		int step);
/**
 * @ingroup PointCloudDownSampling
 * @brief Downsamples a point cloud of type `pcl::PointNormal`.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT downsample(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		int step);
/**
 * @ingroup PointCloudDownSampling
 * @brief Downsamples a point cloud of type `pcl::PointXYZRGBNormal`.
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT downsample(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		int step);
/**
 * @ingroup PointCloudDownSampling
 * @brief Downsamples a point cloud of type `pcl::PointXYZINormal`.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT downsample(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		int step);

/**
 * @defgroup VoxelFiltering Voxel Filtering
 * @brief Performs voxel grid downsampling on a point cloud with optional index filtering.
 *
 * This function downsamples a point cloud using a voxel grid filter. It optionally accepts a set of point indices to limit
 * the operation to a subset of the cloud. For very large point clouds or very small voxel sizes that would cause integer index
 * overflow in the voxel grid structure, the function automatically partitions the space into smaller regions and applies the voxel
 * filtering recursively.
 *
 * @param cloud The input point cloud to be voxelized.
 * @param indices (Optional) A shared pointer to a vector of indices indicating which points to consider. If empty, the entire cloud is used.
 * @param voxelSize The voxel (leaf) size for downsampling. Must be greater than zero.
 * @return A new downsampled point cloud as a shared pointer.
 *
 * @note If the computed voxel grid exceeds 32-bit integer capacity, the function splits the bounding volume into smaller regions
 *       (using a quadtree or octree-like subdivision depending on the Z-axis span) and processes each region recursively.
 * @note If the cloud is not dense and no indices are provided, a warning is issued and an empty cloud is returned.
 *
 * @throws Assertion failure if voxelSize <= 0.
 *
 * @see pcl::VoxelGrid, util3d::cropBox
 */
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointXYZ` on provided indices.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointNormal` on provided indices.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointXYZRGB` on provided indices.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointXYZRGBNormal` on provided indices.
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointXYZI` on provided indices.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointXYZINormal` on provided indices.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointXYZ`.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float voxelSize);
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointNormal`.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		float voxelSize);
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointXYZRGB`.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float voxelSize);
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointXYZRGBNormal`.
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		float voxelSize);
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointXYZI`.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		float voxelSize);
/**
 * @ingroup VoxelFiltering
 * @brief Performs voxel grid downsampling on a point cloud of type `pcl::PointXYZINormal`.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT voxelize(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		float voxelSize);

/**
 * @brief DEPRECATED: Use voxelize() instead.
 *
 * Performs uniform sampling of a point cloud by applying voxel grid filtering
 * with the specified voxel size. This is a legacy wrapper for `voxelize()`.
 *
 * @param cloud The input point cloud (pcl::PointXYZ).
 * @param voxelSize The voxel size (resolution) used for downsampling.
 * @return A downsampled point cloud using voxel grid filtering.
 *
 * @deprecated This function is deprecated. Use voxelize() for equivalent behavior.
 */
inline pcl::PointCloud<pcl::PointXYZ>::Ptr uniformSampling(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float voxelSize)
{
	return voxelize(cloud, voxelSize);
}
/**
 * @brief DEPRECATED: Use voxelize() instead.
 *
 * Performs uniform sampling of a point cloud by applying voxel grid filtering
 * with the specified voxel size. This is a legacy wrapper for `voxelize()`.
 *
 * @param cloud The input point cloud (pcl::PointXYZRGB).
 * @param voxelSize The voxel size (resolution) used for downsampling.
 * @return A downsampled point cloud using voxel grid filtering.
 *
 * @deprecated This function is deprecated. Use voxelize() for equivalent behavior.
 */
inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr uniformSampling(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float voxelSize)
{
	return voxelize(cloud, voxelSize);
}
/**
 * @brief DEPRECATED: Use voxelize() instead.
 *
 * Performs uniform sampling of a point cloud by applying voxel grid filtering
 * with the specified voxel size. This is a legacy wrapper for `voxelize()`.
 *
 * @param cloud The input point cloud (pcl::PointXYZRGBNormal).
 * @param voxelSize The voxel size (resolution) used for downsampling.
 * @return A downsampled point cloud using voxel grid filtering.
 *
 * @deprecated This function is deprecated. Use voxelize() for equivalent behavior.
 */
inline pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr uniformSampling(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		float voxelSize)
{
	return voxelize(cloud, voxelSize);
}


/**
 * @defgroup RandomSampling Random Sampling
 * @brief Randomly samples a subset of points from the input point cloud.
 *
 * This function uses the PCL (Point Cloud Library) `RandomSample` filter to randomly
 * select a specified number of points from the input cloud. It ensures the number
 * of samples is greater than zero and returns a new point cloud containing the sampled points.
 *
 * @param cloud A pointer to the input point cloud from which to sample points.
 * @param samples The number of points to randomly sample from the input cloud. Must be > 0.
 * @return A pointer to a new point cloud containing the randomly sampled points.
 *
 * @throws Assertion failure if `samples <= 0`.
 */
/**
 * @ingroup RandomSampling
 * @brief Performs random sampling on a point cloud of type `pcl::PointXYZ`.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT randomSampling(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int samples);
/**
 * @ingroup RandomSampling
 * @brief Performs random sampling on a point cloud of type `pcl::PointNormal`.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT randomSampling(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		int samples);
/**
 * @ingroup RandomSampling
 * @brief Performs random sampling on a point cloud of type `pcl::PointXYZRGB`.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT randomSampling(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int samples);
/**
 * @ingroup RandomSampling
 * @brief Performs random sampling on a point cloud of type `pcl::PointXYZRGBNormal`.
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT randomSampling(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		int samples);
/**
 * @ingroup RandomSampling
 * @brief Performs random sampling on a point cloud of type `pcl::PointXYZI`.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT randomSampling(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		int samples);
/**
 * @ingroup RandomSampling
 * @brief Performs random sampling on a point cloud of type `pcl::PointXYZINormal`.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT randomSampling(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		int samples);

/**
 * @defgroup PassThrough Pass-through Filtering
 * @brief Filters points from the input point cloud using a pass-through filter along a specified axis.
 *
 * This function applies a PCL `PassThrough` filter to include or exclude points in the input cloud
 * that fall within a specified range along a given axis (`x`, `y`, or `z`). It supports optional filtering
 * using a subset of point indices and can perform either inclusive or exclusive filtering depending on the
 * `negative` flag.
 *
 * @param cloud A pointer to the input point cloud to be filtered.
 * @param indices A pointer to the indices specifying which points in the cloud to consider for filtering.
 *                Pass `nullptr` to apply the filter to the entire cloud.
 * @param axis The axis (`"x"`, `"y"`, or `"z"`) along which the filtering will be applied.
 * @param min The minimum limit of the pass-through filter (inclusive by default).
 * @param max The maximum limit of the pass-through filter (inclusive by default). Must be greater than `min`.
 * @param negative If `true`, the filter will exclude points within the limits; if `false`, it will include only those within.
 *
 * @return The indices representing the points that passed the filter if indices are 
 *         passed to the function, or the new filtered point cloud otherwise.
 *
 * @throws Assertion failure if `max <= min` or if `axis` is not `"x"`, `"y"`, or `"z"`.
 */
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointXYZ` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointXYZRGB` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointXYZI` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointNormal` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointXYZRGBNormal` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointXYZINormal` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointXYZ` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointXYZRGB` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointXYZI` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointNormal` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointXYZRGBNormal` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
/**
 * @ingroup PassThrough
 * @brief Performs pass-through filtering on a point cloud of type `pcl::PointXYZINormal` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT passThrough(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);

/**
 * @defgroup CropBox Crop Box Filtering
 * @brief Filters points from a point cloud using a 3D crop box.
 *
 * This function applies a PCL `CropBox` filter to retain or exclude points from the input point cloud
 * that lie within a specified 3D axis-aligned bounding box, optionally transformed by a 3D transformation.
 * The filter can be applied to the entire cloud or limited to a subset via input indices.
 *
 * @param cloud A pointer to the input point cloud to be filtered.
 * @param indices A pointer to a vector of point indices that restricts the filtering to a subset of the cloud.
 *                Pass `nullptr` to apply the filter to all points in the cloud.
 * @param min The minimum (corner) boundary of the crop box, as an Eigen 4D vector (x, y, z, _).
 * @param max The maximum (corner) boundary of the crop box, as an Eigen 4D vector (x, y, z, _).
 * @param transform A transformation to be applied to the crop box (not the cloud). If the transform is null
 *                  or identity, no transformation is applied.
 * @param negative If `true`, the filter will exclude points inside the box; if `false`, it will include only those inside.
 *
 * @return The indices corresponding to the points that passed the filter, or the new filtered point cloud otherwise.
 *
 * @throws Assertion failure if any of `min[x] >= max[x]`, `min[y] >= max[y]`, or `min[z] >= max[z]`.
 */
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PCLPointCloud2` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PCLPointCloud2::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointXYZ` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointNormal` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointXYZRGB` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointXYZRGBNormal` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointXYZI` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointXYZINormal` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointXYZ` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointNormal` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointXYZRGB` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointXYZI` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointXYZINormal` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
/**
 * @ingroup CropBox
 * @brief Performs crop box filtering on a point cloud of type `pcl::PointXYZRGBNormal` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT cropBox(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);

/**
 * @defgroup FrustumFiltering Frustum Filtering
 * @brief Filters points from a point cloud based on a camera frustum.
 *
 * This function uses PCL's `FrustumCulling` filter to include or exclude points from a 3D point cloud
 * that lie within a specified camera frustum. The frustum is defined by vertical and horizontal field of view
 * angles, near and far clipping planes, and the camera pose. It can optionally apply filtering on a subset
 * of points specified by indices.
 *
 * @note This assumes the pose includes the optical rotation of the camera (X right, Y down, Z forward).
 *
 * @param cloud A pointer to the input point cloud to be filtered.
 * @param indices A pointer to the subset of indices in the point cloud to consider for filtering.
 *                Pass `nullptr` to apply the filter to the full cloud.
 * @param cameraPose The transformation representing the camera pose in world coordinates.
 *                   This defines the position and orientation of the frustum.
 * @param horizontalFOV The horizontal field of view in degrees (e.g., computed as atan((image width/2)/fx) * 2).
 * @param verticalFOV The vertical field of view in degrees (e.g., atan((image height/2)/fy) * 2).
 * @param nearClipPlaneDistance The distance from the camera to the near clipping plane.
 * @param farClipPlaneDistance The distance from the camera to the far clipping plane.
 * @param negative If `true`, the filter will exclude points inside the frustum.
 *                 If `false`, it will include only those inside.
 *
 * @return The indices representing the points that passed the filter, or a new filtered point cloud.
 * 
 * @throws Assertion failure if:
 * - `horizontalFOV <= 0` or `verticalFOV <= 0`,
 * - `farClipPlaneDistance <= nearClipPlaneDistance`,
 * - `cameraPose` is null.
 */
/**
 * @ingroup FrustumFiltering
 * @brief Performs frustum filtering on a point cloud of type `pcl::PointXYZ` and returns filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT frustumFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & cameraPose,
		float horizontalFOV,
		float verticalFOV,
		float nearClipPlaneDistance,
		float farClipPlaneDistance,
		bool negative = false);
/**
 * @ingroup FrustumFiltering
 * @brief Performs frustum filtering on a point cloud of type `pcl::PointXYZ` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT frustumFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & cameraPose,
		float horizontalFOV,
		float verticalFOV,
		float nearClipPlaneDistance,
		float farClipPlaneDistance,
		bool negative = false);
/**
 * @ingroup FrustumFiltering
 * @brief Performs frustum filtering on a point cloud of type `pcl::PointXYZRGB` and returns a new point cloud of the filtered points.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT frustumFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & cameraPose,
		float horizontalFOV,
		float verticalFOV,
		float nearClipPlaneDistance,
		float farClipPlaneDistance,
		bool negative = false);

/**
 * @defgroup RemoveNaNFromPointCloud Remove NaN Points from a Point Cloud
 * @brief Removes NaN (Not-a-Number) points from an organized point cloud.
 *
 * This function takes a pointer to an organized PCL point cloud and returns a new dense point cloud
 * with all NaN points removed. It uses the PCL function `pcl::removeNaNFromPointCloud`
 * to perform the filtering.
 * 
 * @note If the input cloud is already dense, no filtering happens and 
 *       a copy of the input cloud is returned.
 *
 * @param cloud The organized input point cloud containing possible NaN values.
 * @return A new dense point cloud with NaN points removed.
 *
 */
/**
 * @ingroup RemoveNaNFromPointCloud
 * @brief Remove NaN points from a point cloud of type `pcl::PointXYZ`.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
/**
 * @ingroup RemoveNaNFromPointCloud
 * @brief Remove NaN points from a point cloud of type `pcl::PointXYZRGB`.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
/**
 * @ingroup RemoveNaNFromPointCloud
 * @brief Remove NaN points from a point cloud of type `pcl::PointXYZI`.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud);
/**
 * @ingroup RemoveNaNFromPointCloud
 * @brief Remove NaN points from a point cloud of type `pcl::PCLPointCloud2`.
 */
pcl::PCLPointCloud2::Ptr RTABMAP_CORE_EXPORT removeNaNFromPointCloud(
		const pcl::PCLPointCloud2::Ptr & cloud);


/**
 * @defgroup RemoveNaNNormalsFromPointCloud Remove NaN Points from a Point Cloud
 * @brief Removes points with NaN normal values from a point cloud.
 *
 * This function filters out all points in the input point cloud whose normal components
 * contain NaN (Not-a-Number) values. It uses the PCL utility function 
 * `pcl::removeNaNNormalsFromPointCloud` to perform the filtering operation.
 *
 * @param cloud The input point cloud, which may contain NaN normal values.
 * @return A new point cloud with all NaN normals removed.
 *
 */
/**
 * @ingroup RemoveNaNNormalsFromPointCloud
 * @brief Removes points with NaN normal values from a point cloud of type `pcl::PointNormal`.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud);
/**
 * @ingroup RemoveNaNNormalsFromPointCloud
 * @brief Removes points with NaN normal values from a point cloud of type `pcl::PointXYZRGBNormal`.
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud);
/**
 * @ingroup RemoveNaNNormalsFromPointCloud
 * @brief Removes points with NaN normal values from a point cloud of type `pcl::PointXYZINormal`.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud);

/**
 * @defgroup RadiusFiltering Radius Filtering
 * @brief Filters points based on the number of neighbors within a given radius.
 *
 * This function performs a radius-based outlier filtering on a point cloud. It returns
 * a set of point indices where each corresponding point has more than 
 * `minNeighborsInRadius` neighboring points within the specified `radiusSearch`.
 *
 * If an input index list is provided (`indices` is non-empty), the filtering is applied
 * only to those points. Otherwise, the function processes the entire cloud.
 *
 * @param cloud The input point cloud.
 * @param indices An optional vector of indices specifying the subset of points to process.
 * @param radiusSearch The radius within which to search for neighboring points.
 * @param minNeighborsInRadius The minimum number of neighbors required within the radius
 *                              for a point to be retained.
 * @return A vector of indices of points that passed the filtering criteria.
 *
 * @note Uses a KdTree for efficient neighbor search. The `+1` in `minNeighborsInRadius + 1`
 * accounts for the point itself being included in the neighbor list.
 *
 */
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointXYZ`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointNormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointXYZRGB`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointXYZRGBNormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointXYZI`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointXYZINormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointXYZ` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointNormal` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointXYZRGB` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointXYZRGBNormal` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointXYZI` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
/**
 * @ingroup RadiusFiltering
 * @brief Radius filtering for point cloud of type `pcl::PointXYZINormal` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);

/**
 * @defgroup ProportionalRadiusFiltering Proportional Radius Filtering
 * @brief Performs proportional radius outlier filtering using viewpoint-dependent adaptive radii.
 *
 * This function filters a point cloud by performing a radius search for each point,
 * where the radius is proportional to the distance from a known viewpoint.
 * A point is retained only if it has at least one neighbor within the computed radius,
 * and all neighbors lie within an *adjusted* radius from their own viewpoints,
 * scaled by `neighborScale`.
 *
 * This method is useful in 3D reconstruction and SLAM where density can vary with distance from the sensor,
 * and traditional fixed-radius filtering is insufficient.
 * 
 *
 * @param cloud The input point cloud.
 * @param indices The subset of indices to process. If empty, the entire cloud is processed.
 * @param viewpointIndices A vector mapping each point index in the cloud to its associated viewpoint ID.
 *                         Must be the same size as the input cloud.
 * @param viewpoints A map from viewpoint IDs to 3D poses (transforms) representing sensor positions.
 * @param factor A scaling factor used to determine the radius search distance based on viewpoint distance.
 *               Must be greater than 0. Setting it higher will filter points farther from the more accurate points 
 *               (but processing time will be also higher).
 * @param neighborScale A scaling factor applied to neighbors' viewpoint-based radius. Must be ≥ 1. Setting 
 *                      it higher will accept more noisy points close to the more accurate points (but processing time 
 *                      will be also higher).
 * @return A vector of indices of the points that passed the filtering criteria.
 *
 * @note
 * - A KdTree is used for neighbor search.
 * - Points with no valid neighbors within the proportional radius are removed.
 * - For a point to be kept, all its neighbors (except itself) must also lie within their own
 *   scaled proportional radius (`factor * distance_to_own_viewpoint * neighborScale`).
 * - OpenMP is used to parallelize the full-cloud version for performance.
 * - The function throws exceptions to enforce internal assumptions (e.g., input sizes and valid viewpoint mappings).
 *
 */
/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointXYZ`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointNormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointXYZRGB`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointXYZRGBNormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointXYZI`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointXYZINormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);

/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointXYZ` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointNormal` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointXYZRGB` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointXYZRGBNormal` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointXYZI` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
/**
 * @ingroup ProportionalRadiusFiltering
 * @brief Proportional radius filtering for point cloud of type `pcl::PointXYZINormal` with indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);

/**
 * @defgroup SubtractFiltering Subtract Filtering
 * @brief Filters a point cloud by removing points that have a sufficient number of neighbors
 *        within a given radius in a second point cloud.
 *
 * This function performs a radius-based subtraction filter between two point clouds. It searches 
 * the `subtractCloud` for neighbors around each point in the `cloud`, and retains only those 
 * points from `cloud` (or from the `indices` subset of it) that have fewer than 
 * `minNeighborsInRadius` neighbors within the `radiusSearch` radius in `subtractCloud`.
 *
 * @param cloud The input point cloud to filter.
 * @param indices The indices of the input point cloud to consider. If empty, the entire cloud is used.
 * @param subtractCloud The point cloud to search against for neighboring points.
 * @param subtractIndices Optional indices to use from the subtractCloud. If empty, the whole cloud is used.
 * @param radiusSearch The radius within which to search for neighbors in the `subtractCloud`.
 * @param minNeighborsInRadius Minimum number of neighbors required within `radiusSearch` to consider
 *        a point in `cloud` as not filtered out. Points with fewer neighbors are kept.
 *
 * @return The indices of points from `cloud` that passed the filter, or the new filtered point cloud otherwise.
 */
/**
 * @ingroup SubtractFiltering
 * @brief Subtract filtering for point cloud of type `pcl::PointXYZ`, returning filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractFiltering(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
	const pcl::IndicesPtr & indices,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr & subtractCloud,
	const pcl::IndicesPtr & subtractIndices,
	float radiusSearch,
	int minNeighborsInRadius = 1);
 /**
 * @ingroup SubtractFiltering
 * @brief Subtract filtering for point cloud of type `pcl::PointXYZRGB`, returning filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & subtractCloud,
		const pcl::IndicesPtr & subtractIndices,
		float radiusSearch,
		int minNeighborsInRadius = 1);
/**
 * @ingroup SubtractFiltering
 * @brief Subtract filtering for point cloud of type `pcl::PointNormal`, returning filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointNormal>::Ptr & subtractCloud,
		const pcl::IndicesPtr & subtractIndices,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);
/**
 * @ingroup SubtractFiltering
 * @brief Subtract filtering for point cloud of type `pcl::PointXYZINormal`, returning filtered indices.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractFiltering(
	const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
	const pcl::IndicesPtr & indices,
	const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & subtractCloud,
	const pcl::IndicesPtr & subtractIndices,
	float radiusSearch,
	float maxAngle = M_PI/4.0f,
	int minNeighborsInRadius = 1);
/**
* @ingroup SubtractFiltering
* @brief Subtract filtering for point cloud of type `pcl::PointXYZRGBNormal`, returning filtered indices.
*/
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractFiltering(
	const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
	const pcl::IndicesPtr & indices,
	const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & subtractCloud,
	const pcl::IndicesPtr & subtractIndices,
	float radiusSearch,
	float maxAngle = M_PI/4.0f,
	int minNeighborsInRadius = 1);
/**
 * @ingroup SubtractFiltering
 * @brief Subtract filtering for point cloud of type `pcl::PointXYZ`, returning a new filtered point cloud.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT subtractFiltering(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr & subtractCloud,
	float radiusSearch,
	int minNeighborsInRadius = 1);
/**
 * @ingroup SubtractFiltering
 * @brief Subtract filtering for point cloud of type `pcl::PointXYZRGB`, returning a new filtered point cloud.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT subtractFiltering(
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & subtractCloud,
	float radiusSearch,
	int minNeighborsInRadius = 1);
/**
 * @ingroup SubtractFiltering
 * @brief Subtract filtering for point cloud of type `pcl::PointNormal`, returning a new filtered point cloud.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointNormal>::Ptr & subtractCloud,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);
/**
 * @ingroup SubtractFiltering
 * @brief Subtract filtering for point cloud of type `pcl::PointXYZINormal`, returning a new filtered point cloud.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & subtractCloud,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);
/**
 * @ingroup SubtractFiltering
 * @brief Subtract filtering for point cloud of type `pcl::PointXYZRGBNormal`, returning a new filtered point cloud.
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & subtractCloud,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);

/**
 * @brief Performs adaptive radius-based subtraction filtering on a point cloud.
 *
 * This function removes points from the input `cloud` that have at least 
 * `minNeighborsInRadius` neighbors within an adaptive radius in the `subtractCloud`.
 * The search radius is scaled proportionally to the distance of each point from the `viewpoint`,
 * using the `radiusSearchRatio`.
 *
 * @param cloud The input point cloud to filter.
 * @param indices The subset of points in `cloud` to consider. If empty, the entire cloud is used.
 * @param subtractCloud The reference point cloud to search against.
 * @param subtractIndices Optional indices for `subtractCloud`. If empty, the full cloud is used.
 * @param radiusSearchRatio The ratio to scale the radius based on distance to `viewpoint`.
 * @param minNeighborsInRadius Minimum number of neighbors required to consider a point "covered".
 * @param viewpoint The reference viewpoint used to compute adaptive search radius.
 *
 * @return The retained point indices from `cloud`.
 *
 * @note Points with fewer than `minNeighborsInRadius` neighbors in the subtract cloud are retained.
 * @warning This version does not consider surface normals; it is purely geometric.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractAdaptiveFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & subtractCloud,
		const pcl::IndicesPtr & subtractIndices,
		float radiusSearchRatio = 0.01,
		int minNeighborsInRadius = 1,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0));

/**
 * @brief Performs adaptive radius-based subtraction filtering on a point cloud with normals,
 *        also considering normal direction differences.
 *
 * This function removes points from the input `cloud` that have at least 
 * `minNeighborsInRadius` neighbors within an adaptive radius in the `subtractCloud`, 
 * unless the angular difference between normals exceeds `maxAngle`.
 *
 * The search radius is scaled based on the distance of each point from the `viewpoint`,
 * using the `radiusSearchRatio`. For neighbors found, the angle between their normals 
 * and the input point’s normal is evaluated. If the angle exceeds `maxAngle`, the neighbor
 * is ignored.
 *
 * @param cloud The input point cloud with normals to filter.
 * @param indices The subset of points in `cloud` to consider. If empty, the entire cloud is used.
 * @param subtractCloud The reference point cloud with normals to search against.
 * @param subtractIndices Optional indices for `subtractCloud`. If empty, the full cloud is used.
 * @param radiusSearchRatio The ratio to scale the radius based on distance to `viewpoint`.
 * @param maxAngle Maximum angle (in radians) allowed between normals of matched neighbors.
 * @param minNeighborsInRadius Minimum number of valid neighbors required to exclude a point.
 * @param viewpoint The reference viewpoint used to compute adaptive search radius.
 *
 * @return The retained point indices from `cloud`.
 *
 * @note Neighbors with normals exceeding `maxAngle` from the input point's normal are discarded.
 *       Points with fewer than `minNeighborsInRadius` valid neighbors are retained.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractAdaptiveFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & subtractCloud,
		const pcl::IndicesPtr & subtractIndices,
		float radiusSearchRatio = 0.01,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0));


/**
 * @defgroup PointNormalFiltering Point Normal Filtering
 * @brief Filters points in a point cloud based on the angle between their estimated surface normals
 *        and a given reference normal.
 *
 * This function estimates surface normals for each point (or subset of points) in the input point cloud,
 * and retains only those points whose normals are within `angleMax` radians from the given reference normal.
 * 
 * If `groundNormalsUp` is set, normals that are facing significantly downward (based on the Z-axis) may be
 * flipped to point upwards before comparison, especially useful for handling sensors like Velodyne where
 * some road-facing rays produce downward normals.
 *
 * @param cloud The input point cloud to process. If the input cloud type doesn't have normals, the 
 *              normals will be computed using `normalKSearh` and `viewpoint` parameters.
 * @param indices Optional subset of indices in `cloud` to consider. If empty, the entire cloud is used.
 * @param angleMax Maximum allowed angle (in radians) between estimated normals and the given reference `normal`.
 * @param normal The reference normal vector to compare against (must be normalized).
 * @param normalKSearch Number of nearest neighbors used for normal estimation, when input cloud doesn't have normals.
 * @param viewpoint The viewpoint position to be used in normal estimation (can affect direction of normals).
 * @param groundNormalsUp Threshold used to detect and flip ground-facing normals (set to 0.0f to disable).
 *        If the Z component of a normal is less than `-groundNormalsUp` and the corresponding point is
 *        below the viewpoint, the normal will be flipped.
 *
 * @return The indices of points whose normals are within `angleMax` of the reference `normal`.
 *
 * @note This function uses PCL's `NormalEstimationOMP` for efficient multi-threaded normal estimation when input cloud doesn't have normals.
 * @note Normal flipping is typically used to correct misoriented normals on flat ground surfaces.
 */
/**
 * @ingroup PointNormalFiltering
 * @brief Point normal filtering for point cloud of type `pcl::PointXYZ`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
/**
 * @ingroup PointNormalFiltering
 * @brief Point normal filtering for point cloud of type `pcl::PointXYZRGB`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
/**
 * @ingroup PointNormalFiltering
 * @brief Point normal filtering for point cloud of type `pcl::PointXYZ`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
/**
 * @ingroup PointNormalFiltering
 * @brief Point normal filtering for point cloud of type `pcl::PointXYZRGB`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
/**
 * @ingroup PointNormalFiltering
 * @brief Point normal filtering for point cloud of type `pcl::PointXYZI`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
/**
 * @ingroup PointNormalFiltering
 * @brief Point normal filtering for point cloud of type `pcl::PointNormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
/**
 * @ingroup PointNormalFiltering
 * @brief Point normal filtering for point cloud of type `pcl::PointXYZRGBNormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
/**
 * @ingroup PointNormalFiltering
 * @brief Point normal filtering for point cloud of type `pcl::PointXYZINormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);

/**
 * @defgroup ExtractClusters Point Cloud Cluster Extraction
 * @brief Performs Euclidean cluster extraction on a point cloud.
 *
 * This function uses PCL's Euclidean clustering algorithm to segment the input point cloud
 * into clusters based on spatial proximity. It optionally considers a subset of indices within
 * the cloud, and returns the list of clusters as vectors of point indices.
 *
 * Additionally, it can return the index of the largest cluster found.
 *
 * @param cloud The input point cloud from which clusters will be extracted.
 * @param indices Optional pointer to a list of indices to consider for clustering. If empty, the full cloud is used.
 * @param clusterTolerance The spatial distance threshold for clustering. Points within this distance are considered part of the same cluster.
 * @param minClusterSize The minimum number of points required for a valid cluster.
 * @param maxClusterSize The maximum number of points allowed in a cluster.
 * @param biggestClusterIndex Optional output pointer. If not null, it will be set to the index of the largest cluster in the output.
 *
 * @return A vector of indices, where each element contains the indices of one extracted cluster.
 *
 * @note Internally, this uses a KdTree for spatial searches and `pcl::EuclideanClusterExtraction` for segmentation.
 * @note The order of clusters in the output is not guaranteed to reflect size or position.
 * @note If no clusters are found, the output vector will be empty and biggestClusterIndex will be set to -1.
 */
/**
 * @ingroup ExtractClusters
 * @brief Extract clusters from point cloud of type `pcl::PointXYZ`.
 */
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
/**
 * @ingroup ExtractClusters
 * @brief Extract clusters from point cloud of type `pcl::PointXYZRGB`.
 */
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
/**
 * @ingroup ExtractClusters
 * @brief Extract clusters from point cloud of type `pcl::PointXYZ` inside provided indices.
 */
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
/**
 * @ingroup ExtractClusters
 * @brief Extract clusters from point cloud of type `pcl::PointNormal` inside provided indices.
 */
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
/**
 * @ingroup ExtractClusters
 * @brief Extract clusters from point cloud of type `pcl::PointXYZRGB` inside provided indices.
 */
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
/**
 * @ingroup ExtractClusters
 * @brief Extract clusters from point cloud of type `pcl::PointXYZRGBNormal` inside provided indices.
 */
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
/**
 * @ingroup ExtractClusters
 * @brief Extract clusters from point cloud of type `pcl::PointXYZI` inside provided indices.
 */
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
/**
 * @ingroup ExtractClusters
 * @brief Extract clusters from point cloud of type `pcl::PointXYZ`.
 */
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);

/**
 * @defgroup ExtractIndices Point Cloud Indices Extraction
 * @brief Extracts a subset of points from a point cloud based on specified indices.
 *
 * This function filters the input point cloud by extracting the points corresponding
 * to the provided set of indices. The `negative` parameter determines whether to
 * extract the points listed in the indices (if `negative` is false) or to exclude
 * the points listed in the indices (if `negative` is true).
 *
 * The function uses PCL's `ExtractIndices` to perform the extraction and returns the
 * resulting set of indices as a shared pointer to a vector of integers.
 *
 * @param cloud The input point cloud from which to extract points.
 * @param indices A pointer to the indices specifying the points to be extracted (or excluded).
 * @param negative If true, the points specified by `indices` are excluded; otherwise, they are included.
 *
 * @return A vector of indices representing the extracted points (or remaining points if `negative` is true), 
 *         or a new point cloud with the corresponding points.
 *
 * @note This function uses PCL's `ExtractIndices` filter to extract or remove the points based on the provided indices.
 * @note The returned `output` indices correspond to the points that are either extracted or excluded from the input cloud.
 */
/**
 * @ingroup ExtractIndices
 * @brief Extract indices from point cloud of type `pcl::PointXYZ`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
/**
 * @ingroup ExtractIndices
 * @brief Extract indices from point cloud of type `pcl::PointNormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
/**
 * @ingroup ExtractIndices
 * @brief Extract indices from point cloud of type `pcl::PointXYZRGB`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
/**
 * @ingroup ExtractIndices
 * @brief Extract indices from point cloud of type `pcl::PointXYZRGBNormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
/**
 * @ingroup ExtractIndices
 * @brief Extract indices from point cloud of type `pcl::PointXYZI`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
/**
 * @ingroup ExtractIndices
 * @brief Extract indices from point cloud of type `pcl::PointXYZINormal`.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
/**
 * @ingroup ExtractIndices
 * @brief Extract points from point cloud of type `pcl::PointXYZ` with corresponding indices.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
/**
 * @ingroup ExtractIndices
 * @brief Extract points from point cloud of type `pcl::PointXYZRGB` with corresponding indices.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
/**
 * @ingroup ExtractIndices
 * @brief Extract points from point cloud of type `pcl::PointXYZRGBNormal` with corresponding indices.
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
/**
 * @ingroup ExtractIndices
 * @brief Extract points from point cloud of type `pcl::PointXYZI` with corresponding indices.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
/**
 * @ingroup ExtractIndices
 * @brief Extract points from point cloud of type `pcl::PointXYZINormal` with corresponding indices.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);

/**
 * @brief Extracts the indices of the inliers that belong to a plane using RANSAC.
 *
 * This function uses the RANSAC algorithm to fit a plane model to a set of 3D points
 * and returns the indices of the points that are considered inliers (i.e., those that
 * lie close to the fitted plane). It also optionally outputs the coefficients of the plane
 * (normal and offset).
 *
 * @param cloud The input point cloud.
 * @param indices An optional set of indices in the point cloud to use for segmentation. If empty, the whole cloud is used.
 * @param distanceThreshold The distance threshold for considering points as inliers to the plane. Points within this threshold are classified as inliers.
 * @param maxIterations The maximum number of iterations the RANSAC algorithm should run.
 * @param coefficientsOut An optional output pointer to store the coefficients of the plane model. The coefficients include the plane normal and offset.
 * 
 * @return A list of indices of points that are considered inliers to the fitted plane.
 * 
 * @note If the input `indices` is empty, the entire point cloud will be used for segmentation.
 * @note If the `coefficientsOut` is not null, it will be filled with the model coefficients of the plane.
 * 
 */
pcl::IndicesPtr extractPlane(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float distanceThreshold,
		int maxIterations = 100,
		pcl::ModelCoefficients * coefficientsOut = 0);
pcl::IndicesPtr extractPlane(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float distanceThreshold,
		int maxIterations = 100,
		pcl::ModelCoefficients * coefficientsOut = 0);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_FILTERING_H_ */
