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


pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud);
pcl::PCLPointCloud2::Ptr RTABMAP_CORE_EXPORT removeNaNFromPointCloud(
		const pcl::PCLPointCloud2::Ptr & cloud);


pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud);

/**
 * For convenience.
 */

pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);

/**
 * @brief Wrapper of the pcl::RadiusOutlierRemoval class.
 *
 * Points in the cloud which have less than a minimum of neighbors in the
 * specified radius are filtered.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearch the radius in meter.
 * @param minNeighborsInRadius the minimum of neighbors to keep the point.
 * @return the indices of the points satisfying the parameters.
 */

pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_CORE_EXPORT radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);

/* for convenience */
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);

/**
 * @brief Filter points based on distance from their viewpoint.
 *
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param viewpointIndices should be same size than the input cloud, it tells the viewpoint index in viewpoints for each point.
 * @param viewpoints the viewpoints.
 * @param factor will determine the search radius based on the distance from a point and its viewpoint. Setting it higher will filter points farther from accurate points (but processing time will be also higher).
 * @param neighborScale will scale the search radius of neighbors found around a point. Setting it higher will accept more noisy points close to accurate points (but processing time will be also higher).
 * @return the indices of the points satisfying the parameters.
 */

pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);

/**
 * For convenience.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & substractCloud,
		float radiusSearch,
		int minNeighborsInRadius = 1);

/**
 * Subtract a cloud from another one using radius filtering.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param cloud the input cloud to subtract.
 * @param indices the input indices of the subtracted cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearch the radius in meter.
 * @return the indices of the points satisfying the parameters.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		int minNeighborsInRadius = 1);

/**
 * For convenience.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointNormal>::Ptr & substractCloud,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);

/**
 * Subtract a cloud from another one using radius filtering.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param cloud the input cloud to subtract.
 * @param indices the input indices of the subtracted cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearch the radius in meter.
 * @return the indices of the points satisfying the parameters.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointNormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);

/**
 * For convenience.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & substractCloud,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & substractCloud,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);

/**
 * Subtract a cloud from another one using radius filtering.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param cloud the input cloud to subtract.
 * @param indices the input indices of the subtracted cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearch the radius in meter.
 * @return the indices of the points satisfying the parameters.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);

/**
 * Subtract a cloud from another one using radius filtering.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param cloud the input cloud to subtract.
 * @param indices the input indices of the subtracted cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearchRatio the ratio used to compute the radius at different distances (e.g., a ratio of 0.1 at 4 m results in a radius of 4 cm).
 * @return the indices of the points satisfying the parameters.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractAdaptiveFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearchRatio = 0.01,
		int minNeighborsInRadius = 1,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0));

/**
 * Subtract a cloud from another one using radius filtering.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param cloud the input cloud to subtract.
 * @param indices the input indices of the subtracted cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearchRatio the ratio used to compute the radius at different distances (e.g., a ratio of 0.01 at 4 m results in a radius of 4 cm).
 * @return the indices of the points satisfying the parameters.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT subtractAdaptiveFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearchRatio = 0.01,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0));


/**
 * For convenience.
 */

pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);

/**
 * @brief Given a normal and a maximum angle error, keep all points of the cloud
 * respecting this normal.
 *
 * The normals are computed using the radius search parameter (pcl::NormalEstimation class is used for this), then
 * for each normal, the corresponding point is filtered if the
 * angle (using pcl::getAngle3D()) with the normal specified by the user is larger than the maximum
 * angle specified by the user.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to process, if empty, all points in the cloud are processed.
 * @param angleMax the maximum angle.
 * @param normal the normal to which each point's normal is compared.
 * @param normalKSearch number of neighbor points used for normal estimation (see pcl::NormalEstimation).
 * @param viewpoint from which viewpoint the normals should be estimated (see pcl::NormalEstimation).
 * @return the indices of the points which respect the normal constraint.
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_CORE_EXPORT normalFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);

/**
 * For convenience.
 */
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);

/**
 * @brief Wrapper of the pcl::EuclideanClusterExtraction class.
 *
 * Extract all clusters from a point cloud given a maximum cluster distance tolerance.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to process, if empty, all points in the cloud are processed.
 * @param clusterTolerance the cluster distance tolerance (see pcl::EuclideanClusterExtraction).
 * @param minClusterSize minimum size of the clusters to return (see pcl::EuclideanClusterExtraction).
 * @param maxClusterSize maximum size of the clusters to return (see pcl::EuclideanClusterExtraction).
 * @param biggestClusterIndex the index of the biggest cluster, if the clusters are empty, a negative index is set.
 * @return the indices of each cluster found.
 */
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_CORE_EXPORT extractClusters(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);

pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
pcl::IndicesPtr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
// PCL default lacks of pcl::PointNormal type support
//pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT extractIndices(
//		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
//		const pcl::IndicesPtr & indices,
//		bool negative,
//		bool keepOrganized);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT extractIndices(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);

pcl::IndicesPtr extractPlane(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float distanceThreshold,
		int maxIterations = 100,
		pcl::ModelCoefficients * coefficientsOut = 0);
pcl::IndicesPtr extractPlane(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float distanceThreshold,
		int maxIterations = 100,
		pcl::ModelCoefficients * coefficientsOut = 0);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_FILTERING_H_ */
