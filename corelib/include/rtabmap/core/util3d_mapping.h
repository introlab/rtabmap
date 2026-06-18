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

#include "rtabmap/core/rtabmap_core_export.h"

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

// Use interface with \"viewpoint\" parameter to make sure the ray tracing origin is from the sensor and not the base.
RTABMAP_DEPRECATED void RTABMAP_CORE_EXPORT occupancy2DFromLaserScan(
		const cv::Mat & scan, // in /base_link frame
		cv::Mat & empty,
		cv::Mat & occupied,
		float cellSize,
		bool unknownSpaceFilled = false,
		float scanMaxRange = 0.0f);

// Use interface with scanHit/scanNoHit parameters: scanNoHit set to null matrix has the same functionality than this method.
RTABMAP_DEPRECATED void RTABMAP_CORE_EXPORT occupancy2DFromLaserScan(
		const cv::Mat & scan, // in /base_link frame
		const cv::Point3f & viewpoint, // /base_link -> /base_scan
		cv::Mat & empty,
		cv::Mat & occupied,
		float cellSize,
		bool unknownSpaceFilled = false,
		float scanMaxRange = 0.0f);

/**
 * @brief Generates 2D occupancy grid maps (free and occupied cells) from laser scan data.
 *
 * This function takes in a laser scan composed of hit and no-hit points and computes two
 * 2D maps:
 * - `empty`: 2D coordinates of free space (where the laser passed without hitting obstacles).
 * - `occupied`: precise 2D coordinates of obstacle hits (where the laser reflected).
 *
 * The internal representation uses a temporary occupancy map generated from `create2DMap()`,
 * from which free cells are extracted. Obstacle points are directly passed through, potentially
 * clipped by a maximum range filter.
 *
 * @param[in] scanHitIn          CV_32FC2 or CV_32FC(n>=2) matrix representing obstacle hits in 2D or 3D space (relative to base frame, not laser frame).
 * @param[in] scanNoHitIn        CV_32FC2 or CV_32FC(n>=2) matrix representing laser rays that did not hit an obstacle (relative to base frame, not laser frame).
 * @param[in] viewpoint          The viewpoint (sensor origin) from which the scan was taken, in 3D space, relative to base frame.
 *                               This used to determinate the origin of ray tracing.
 * @param[out] empty             Output matrix (CV_32FC2) of free space points derived from ray tracing.
 * @param[out] occupied          Output matrix (CV_32FC2) of occupied (hit) points, filtered by max range if required.
 * @param[in] cellSize           The resolution of the occupancy map grid (in meters per cell).
 * @param[in] unknownSpaceFilled If true, unknown space between hits is also filled via ray tracing.
 * @param[in] scanMaxRange       Maximum range of the scan (in meters). Values beyond this are clipped.
 *
 * @note The function assumes a single scan already converted in base frame for internal processing.
 * @note If `scanMaxRange <= cellSize`, no range filtering is applied to the occupied points.
 *
 * @see create2DMap(), util3d::rangeFiltering()
 */
void RTABMAP_CORE_EXPORT occupancy2DFromLaserScan(
		const cv::Mat & scanHit, // in /base_link frame
		const cv::Mat & scanNoHit, // in /base_link frame
		const cv::Point3f & viewpoint, // /base_link -> /base_scan
		cv::Mat & empty,
		cv::Mat & occupied,
		float cellSize,
		bool unknownSpaceFilled = false,
		float scanMaxRange = 0.0f); // would be set if unknownSpaceFilled=true

/**
 * @brief Creates a 2D occupancy grid map from local occupancy data.
 *
 * Generates a 2D occupancy grid (`CV_8S`) where:
 * - -1 indicates unknown space,
 * - 0 indicates free space,
 * - 100 indicates an occupied (obstacle) cell.
 *
 * This function transforms and merges local empty/occupied occupancy maps
 * from multiple robot poses into a single global 2D grid map.
 *
 * @param posesIn            Map of robot poses, indexed by node ID.
 * @param occupancy          Map of local occupancy data, indexed by node ID.
 *                           Each pair contains two cv::Mat elements:
 *                           - First: empty cells (CV_32FC2) relative to base frame,
 *                           - Second: occupied cells (CV_32FC2) relative to base frame.
 * @param cellSize           The resolution of the map in meters per cell.
 * @param[out] xMin          Minimum x-coordinate (origin offset) of the resulting map (in meters).
 * @param[out] yMin          Minimum y-coordinate (origin offset) of the resulting map (in meters).
 * @param minMapSize         Minimum width/height of the output map in meters.
 *                           If 0, size is computed from poses and occupancy data.
 * @param erode              Whether to post-process (erode) noisy obstacles.
 *                           This helps remove isolated or thin obstacle artifacts.
 * @param footprintRadius    Radius of the robot footprint (in meters).
 *                           Free space will be cleared under the robot.
 *
 * @return cv::Mat           The resulting occupancy grid map (CV_8S):
 *                           -1 = unknown,
 *                            0 = free space,
 *                          100 = obstacle.
 *
 * @warning The output map can be very large if poses are far apart or cellSize is small.
 *          The function will not create a map if the estimated size exceeds reasonable limits (e.g. > 1.5 km).
 *
 * @note The function will fill small holes surrounded by known cells and optionally erode noisy borders.
 */
cv::Mat RTABMAP_CORE_EXPORT create2DMapFromOccupancyLocalMaps(
		const std::map<int, Transform> & poses,
		const std::map<int, std::pair<cv::Mat, cv::Mat> > & occupancy,
		float cellSize,
		float & xMin,
		float & yMin,
		float minMapSize = 0.0f,
		bool erode = false,
		float footprintRadius = 0.0f);

// Use interface with \"viewpoints\" parameter to make sure the ray tracing origin is from the sensor and not the base.
RTABMAP_DEPRECATED cv::Mat RTABMAP_CORE_EXPORT create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans, // in /base_link frame
		float cellSize,
		bool unknownSpaceFilled,
		float & xMin,
		float & yMin,
		float minMapSize = 0.0f,
		float scanMaxRange = 0.0f);

// Use interface with cv::Mat scans.
RTABMAP_DEPRECATED cv::Mat RTABMAP_CORE_EXPORT create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans, // in /base_link frame
		const std::map<int, cv::Point3f > & viewpoints, // /base_link -> /base_scan
		float cellSize,
		bool unknownSpaceFilled,
		float & xMin,
		float & yMin,
		float minMapSize = 0.0f,
		float scanMaxRange = 0.0f);

/**
 * @brief Generates a 2D occupancy grid map from a set of poses, laser scans, and viewpoints.
 * 
 * This function aggregates laser scan data from multiple poses and creates a 2D occupancy grid
 * (CV_8S: signed 8-bit) where:
 * - `-1` represents unknown space,
 * - `0` represents free space,
 * - `100` represents occupied space (obstacles).
 * 
 * The scans are first transformed into the global map frame using the corresponding pose.
 * Obstacles and free space are inserted using ray tracing. Optionally, unknown areas between
 * known rays can also be filled using radial sweeping.
 * 
 * @param poses             A map of node IDs to 3D poses (used to transform local scans to the global frame).
 * @param scans             A map of node IDs to pairs of laser scans (`<hit, no-hit>`), each as a `cv::Mat` of type `CV_32FC2`.
 *                          - `first`: endpoints of beams hitting obstacles (relative to base frame, not laser frame).
 *                          - `second`: endpoints of beams not hitting any obstacle (relative to base frame, not laser frame).
 * @param viewpoints        A map of node IDs to local sensor origin offsets relative to each pose (e.g., lidar offset /base_link -> /base_scan).
 *                          This is used to determinate the starting point for each ray trace.
 * @param cellSize          The size of each grid cell in meters.
 * @param unknownSpaceFilled If true, fills areas between known rays (fan sweeping) up to `scanMaxRange`.
 * @param[out] xMin         The minimum x value (in meters) of the grid origin relative to map coordinates.
 * @param[out] yMin         The minimum y value (in meters) of the grid origin relative to map coordinates.
 * @param minMapSize        The minimum width and height (in meters) of the map. Ensures the output map has a minimum footprint.
 * @param scanMaxRange      The maximum range (in meters) of the sensor. Used to limit ray tracing and padding.
 * 
 * @return A 2D occupancy grid map (`cv::Mat` of type `CV_8S`) where:
 *         - `-1` = unknown
 *         - `0`  = free space
 *         - `100` = obstacle
 *
 * @note If `scanMaxRange <= 0`, map size is determined based on scan data bounds.
 * @note Grid coordinates are calculated with padding to ensure all points fall within the map.
 * @note This function uses ray tracing internally via the `rayTrace()` function.
 */
cv::Mat RTABMAP_CORE_EXPORT create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, std::pair<cv::Mat, cv::Mat> > & scans, // <id, <hit, no hit> >, in /base_link frame
		const std::map<int, cv::Point3f > & viewpoints, // /base_link -> /base_scan
		float cellSize,
		bool unknownSpaceFilled,
		float & xMin,
		float & yMin,
		float minMapSize = 0.0f,
		float scanMaxRange = 0.0f);

/**
 * @brief Performs a 2D ray tracing operation between two points on a grid map.
 *
 * This function draws a line from the `start` point to the `end` point on a grid (e.g., occupancy grid),
 * marking all traversed cells as free (value = 0) unless an obstacle (value = 100) is encountered.
 * The line follows an integer rasterization algorithm (like Bresenham’s line), accounting for steep slopes
 * by transposing axes when needed.
 *
 * @param start           The starting point of the ray (2D grid coordinates).
 * @param end             The ending point of the ray (2D grid coordinates). This point is clipped to the grid bounds.
 * @param grid            A mutable 2D grid represented as a `cv::Mat` of signed char values. 
 *                        Assumes 100 denotes obstacles; 0 denotes free space.
 * @param stopOnObstacle  If true, the ray trace stops upon hitting a cell marked with 100 (an obstacle).
 *
 * @note 
 * - If the slope of the line is steep (outside the range [-1, 1]), the algorithm swaps x and y axes for correctness.
 * - The function ensures both the start and end points are within the bounds of the grid.
 * - All visited cells along the path (except obstacles when `stopOnObstacle` is true) will be updated to 0 (free).
 * - The grid must have type `CV_8SC1` (signed 8-bit single-channel matrix).
 *
 */
void RTABMAP_CORE_EXPORT rayTrace(const cv::Point2i & start,
		const cv::Point2i & end,
		cv::Mat & grid,
		bool stopOnObstacle);

/**
 * @brief Converts an occupancy grid map (CV_8S) to a grayscale image (CV_8U).
 *
 * This function takes a signed 8-bit occupancy grid map and produces a corresponding
 * 8-bit unsigned grayscale image. The pixel values are mapped based on the occupancy values:
 * 
 * - `0`   (free space)       → 178 (normal) or 254 (PGM format)
 * - `100` (obstacle)         → 0   (black)
 * - `-2`  (robot footprint)  → 200 (normal) or 254 (PGM format)
 * - `-1`  (unknown)          → 89  (normal) or 205 (PGM format)
 * - `v > 50` (partial obstacle): scaled to range [0, 89]
 * - `v < 50` (partial free): scaled to range [89, 178]
 *
 * If `pgmFormat` is true, the vertical axis is flipped (for PGM format compatibility).
 *
 * @param map8S The input occupancy grid map as a CV_8S single-channel matrix.
 *              Must contain values such as -1 (unknown), 0 (free), 100 (occupied).
 * @param pgmFormat If true, output will be formatted for PGM file format (inverted Y-axis and different gray scale mapping).
 *
 * @return A CV_8U grayscale image with pixel values representing occupancy status.
 *
 * @throws UASSERT if the input map is not a single-channel CV_8S matrix.
 */
cv::Mat RTABMAP_CORE_EXPORT convertMap2Image8U(const cv::Mat & map8S, bool pgmFormat = false);

/**
 * @brief Converts a grayscale occupancy image (CV_8U) to an occupancy grid map (CV_8S).
 *
 * This function interprets grayscale pixel values from an input image and converts
 * them into occupancy values used in a typical occupancy grid map:
 * - 100: Occupied
 * -   0: Free
 * -  -1: Unknown
 * -  -2: Free space under robot footprint (non-PGM only)
 *
 * The interpretation differs slightly depending on whether the input image is in
 * PGM format (common in ROS map_server) or in standard grayscale.
 *
 * @param map8U       Input grayscale image (type CV_8U, single-channel).
 * @param pgmFormat   If true, assumes PGM format:
 *                    - 0   = occupied
 *                    - 254 = free
 *                    - 205 = unknown
 *                    If false (normal format):
 *                    -   0 = occupied
 *                    - 178 = free
 *                    - 200 = footprint (free space under robot)
 *                    -  89 = unknown
 *
 * @return A CV_8S occupancy grid map with encoded occupancy values.
 *
 * @throws Assertion failure if input image is not of type CV_8U or not single-channel.
 */
cv::Mat RTABMAP_CORE_EXPORT convertImage8U2Map(const cv::Mat & map8U, bool pgmFormat = false);

/**
 * @brief Performs erosion on an occupancy grid map to reduce small noisy obstacles.
 *
 * This function scans a given occupancy grid (`CV_8SC1` format) and removes
 * obstacle cells (value `100`) that are surrounded by at least 3 empty cells (value `0`)
 * and no adjacent unknown cells (value `-1`). These obstacles are likely noise
 * and are converted into empty space (value `0`) in the resulting map.
 *
 * @param map Input occupancy grid map of type `CV_8SC1` where:
 *   - `100` represents obstacles,
 *   - `0` represents free space,
 *   - `-1` represents unknown space.
 *
 * @return A new `cv::Mat` of the same size and type as the input map,
 *         with eroded obstacles.
 *
 */
cv::Mat RTABMAP_CORE_EXPORT erodeMap(const cv::Mat & map);

// templated methods

/**
 * @brief Projects a point cloud onto the XY plane by setting all Z coordinates to zero.
 *
 * This function creates a copy of the input point cloud and modifies each point's Z coordinate
 * to be zero, effectively projecting the entire cloud onto the XY plane.
 *
 * @tparam PointT The type of point used in the point cloud (e.g., pcl::PointXYZ).
 * @param cloud The input point cloud to project.
 * @return typename pcl::PointCloud<PointT>::Ptr A pointer to the projected point cloud
 *         with Z coordinates set to zero.
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr projectCloudOnXYPlane(
		const typename pcl::PointCloud<PointT> & cloud);


/**
 * @brief Segments ground and obstacle indices from a point cloud using surface normals and clustering.
 *
 * This function analyzes a point cloud to identify flat surfaces (e.g., ground) and separates them
 * from potential obstacles based on normal orientation, height constraints, and optional clustering.
 * Optionally, flat obstacles (e.g., tables, ramps) can be segmented separately.
 *
 * @tparam PointT The type of point used in the point cloud (e.g., pcl::PointXYZ).
 * 
 * @param cloud The input point cloud.
 * @param indices Optional input indices to consider from the cloud (e.g., from a prior ROI extraction).
 * @param ground Output pointer where indices corresponding to ground points will be stored.
 * @param obstacles Output pointer where indices corresponding to obstacle points will be stored.
 * @param normalKSearch Number of neighbors to use for normal estimation.
 * @param groundNormalAngle Maximum angle (in radians) between the estimated normal and the "up" direction
 *                          for a surface to be considered ground.
 * @param clusterRadius The Euclidean distance threshold for clustering flat surfaces and obstacles.
 * @param minClusterSize The minimum number of points required to form a valid cluster.
 * @param segmentFlatObstacles If true, flat but non-ground surfaces (e.g., tables) are detected and
 *                             optionally returned via `flatObstacles`.
 * @param maxGroundHeight Maximum Z-height for a surface to be considered ground (0 disables filtering). 
 *                        Note that all obstacle points under that threshold will be ignored (i.e., won't 
 *                        be returned in `obstacles`).
 * @param flatObstacles Optional output pointer where indices corresponding to flat obstacles will be stored
 *                      (only valid if `segmentFlatObstacles` is true).
 * @param viewPoint The viewpoint to use for normal estimation (important for consistent orientation).
 * @param groundNormalsUp Threshold (between 0 and 1) used to detect and flip ground-facing normals 
 *        (set to 0.0f to disable). If the Z component of a normal is less than `-groundNormalsUp` 
 *        and the corresponding point is below the viewpoint, the normal will be flipped.
 */
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
/**
 * @brief Segments ground and obstacle indices from a point cloud using surface normals and clustering.
 * @see `segmentObstaclesFromGround()` with indices
 */
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

/**
 * @brief Projects 3D ground and obstacle point clouds onto the 2D XY plane and voxelizes them into 2D occupancy data.
 *
 * This function takes two 3D point clouds—representing ground and obstacles—and performs the following steps:
 * - Projects them onto the XY plane (setting Z = 0).
 * - Voxelizes them based on the specified cell size.
 * - Converts the resulting 2D points into OpenCV matrices (1-row, N-columns, `CV_32FC2`) where each element is a (x, y) coordinate.
 *
 * @tparam PointT The type of point in the input point clouds (e.g., pcl::PointXYZ).
 * 
 * @param groundCloud The input point cloud representing ground points.
 * @param obstaclesCloud The input point cloud representing obstacle points.
 * @param ground Output matrix containing 2D (x, y) coordinates of projected ground points (type: `CV_32FC2`).
 * @param obstacles Output matrix containing 2D (x, y) coordinates of projected obstacle points (type: `CV_32FC2`).
 * @param cellSize The size of each voxel/grid cell used for downsampling the projected cloud (in meters).
 */
template<typename PointT>
void occupancy2DFromGroundObstacles(
		const typename pcl::PointCloud<PointT>::Ptr & groundCloud,
		const typename pcl::PointCloud<PointT>::Ptr & obstaclesCloud,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize);
/**
 * @brief Projects 3D ground and obstacle point clouds onto the 2D XY plane and voxelizes them into 2D occupancy data.
 * @see occupancy2DFromGroundObstacles() without indices
 */
template<typename PointT>
void occupancy2DFromGroundObstacles(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & groundIndices,
		const pcl::IndicesPtr & obstaclesIndices,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize);

/**
 * @brief Generates 2D ground and obstacle occupancy data from a 3D point cloud.
 *
 * This function performs segmentation on a 3D point cloud to separate ground and obstacle points
 * based on normal orientation and clustering, then projects both onto the XY plane to create
 * 2D occupancy representations using voxelization.
 *
 * The resulting occupancy data is returned as two OpenCV matrices (`cv::Mat`) of type `CV_32FC2`,
 * where each entry contains a 2D point (x, y) in meters corresponding to a ground or obstacle voxel.
 *
 * @tparam PointT The type of point used in the input point cloud (e.g., pcl::PointXYZ).
 *
 * @param cloud The input 3D point cloud.
 * @param indices Optional subset of points from the cloud to use for processing (can be full cloud indices).
 * @param ground Output matrix containing 2D ground points projected and voxelized (`CV_32FC2`).
 * @param obstacles Output matrix containing 2D obstacle points projected and voxelized (`CV_32FC2`).
 * @param cellSize The voxel size (in meters) for projecting and grouping points in 2D.
 * @param groundNormalAngle Maximum allowable angle (in radians) between a point's normal and the vertical axis for it to be considered part of the ground.
 * @param minClusterSize Minimum number of points required to form a valid obstacle cluster.
 * @param segmentFlatObstacles Whether to separate flat horizontal surfaces (e.g., tables) from the ground and treat them as obstacles.
 * @param maxGroundHeight Maximum Z value (in meters) for a surface to be considered ground. If 0, height filtering is disabled.
 *
 * @note Internally, this function calls:
 * - `segmentObstaclesFromGround()` to classify ground vs. obstacle points.
 * - `occupancy2DFromGroundObstacles()` to project and voxelize the classified data.
 */
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
/**
 * @brief Generates 2D ground and obstacle occupancy data from a 3D point cloud.
 * @see `occupancy2DFromCloud3D()` with indices
 */
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

} // namespace util3d
} // namespace rtabmap

#include "rtabmap/core/impl/util3d_mapping.hpp"

#endif /* UTIL3D_MAPPING_H_ */
