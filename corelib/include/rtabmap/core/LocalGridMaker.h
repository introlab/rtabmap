/*
Copyright (c) 2010-2023, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef SRC_LOCAL_MAP_H_
#define SRC_LOCAL_MAP_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>

namespace rtabmap {

/**
 * @class LocalGridMaker
 * @brief Builds per-node local occupancy grids from laser scans or depth clouds.
 *
 * Configured via `Grid/` parameters (@ref parseParameters()). Used by @ref Memory
 * to populate @ref LocalGrid cells (ground / obstacles / empty) stored on
 * @ref SensorData and cached in @ref LocalGridCache for global maps
 * (@ref OccupancyGrid, @ref OctoMap, @ref GridMap).
 *
 * **Sensor source** (`Grid/Sensor`, @ref isGridFromDepth()):
 * - `0` — laser scan only
 * - `1` — depth image(s) only (default)
 * - `2` — laser scan and depth
 *
 * **2D laser path:** projects a 2D @ref LaserScan with @ref util3d::occupancy2DFromLaserScan().
 *
 * **3D path:** segments the scan/cloud into ground and obstacles (@ref segmentCloud()),
 * then outputs @ref LocalGrid cell matrices (2D projection or 3D cells per `Grid/3D`).
 *
 * @see LocalGrid
 * @see Memory
 */
class RTABMAP_CORE_EXPORT LocalGridMaker
{
public:
	/** @brief Constructs with @ref parseParameters() on @p parameters (or defaults). */
	LocalGridMaker(const ParametersMap & parameters = ParametersMap());
	virtual ~LocalGridMaker();

	/** @brief Updates grid settings from `Grid/` entries in @p parameters. */
	virtual void parseParameters(const ParametersMap & parameters);

	/** @return Current `Grid/CellSize` (m). */
	float getCellSize() const {return cellSize_;}
	/** @return True if occupancy is built from depth (`Grid/Sensor` is 1 or 2). */
	bool isGridFromDepth() const {return occupancySensor_;}
	/** @return True if `Grid/MapFrameProjection` is enabled. */
	bool isMapFrameProjection() const {return projMapFrame_;}

	/**
	 * @brief Segments a point cloud into ground and obstacle indices.
	 *
	 * Applies optional voxel filtering (`Grid/PreVoxelFiltering`, leaf size `Grid/CellSize`),
	 * footprint crop, height filtering, then either normal-based ground segmentation
	 * (`Grid/NormalsSegmentation`) or Z passthrough.
	 *
	 * @param cloud Input cloud (sensor frame, transformed internally using @p pose). Must be non-null; an empty cloud is allowed and returns empty outputs.
	 * @param indices Subset of @p cloud to process (must be non-null; empty = all points when the cloud is dense).
	 * @param pose Node pose (used for map-frame projection and footprint).
	 * @param viewPoint Sensor origin for segmentation / ray tracing.
	 * @param groundIndices Output indices of ground points.
	 * @param obstaclesIndices Output indices of obstacle points.
	 * @param flatObstacles Optional output for flat obstacle clusters.
	 * @return Segmented cloud (voxel-downsampled when `Grid/PreVoxelFiltering` is true, using `Grid/CellSize`).
	 */
	template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr segmentCloud(
			const typename pcl::PointCloud<PointT>::Ptr & cloud,
			const pcl::IndicesPtr & indices,
			const Transform & pose,
			const cv::Point3f & viewPoint,
			pcl::IndicesPtr & groundIndices,        // output cloud indices
			pcl::IndicesPtr & obstaclesIndices,     // output cloud indices
			pcl::IndicesPtr * flatObstacles = 0) const; // output cloud indices

	/**
	 * @brief Creates a local grid from a @ref Signature's laser scan or depth data.
	 * @param node Signature with sensor data (pose used for 3D scans).
	 * @param groundCells Output ground cells (@ref LocalGrid format).
	 * @param obstacleCells Output obstacle cells.
	 * @param emptyCells Output empty/free cells. Behavior depends on the sensor path:
	 *                   - **2D laser, laser-only mode** (`Grid/Sensor`=0 and a 2D @ref LaserScan):
	 *                     uses @ref util3d::occupancy2DFromLaserScan() / @ref util3d::create2DMap().
	 *                     Free space along each hit beam is always ray-traced (sensor → obstacle).
	 *                     `Grid/Scan2dUnknownSpaceFilled` is separate: when true, it additionally
	 *                     sweeps unknown angular gaps between the first and last hit (sparse FOV /
	 *                     “holes” in coverage) out to `Grid/RangeMax` or scan max range.
	 *                   - **Other cases** (3D scan, depth, `Grid/Sensor`=2, etc.): see
	 *                     @ref createLocalMap(const LaserScan&,...).
	 * @param viewPoint Output view point used for the grid.
	 */
	void createLocalMap(
			const Signature & node,
			cv::Mat & groundCells,
			cv::Mat & obstacleCells,
			cv::Mat & emptyCells,
			cv::Point3f & viewPoint);

	/**
	 * @brief Creates a local grid from a 3D (or organized) @ref LaserScan.
	 * @param cloud Input scan in sensor frame.
	 * @param pose Node pose in map/odom frame.
	 * @param groundCells Output ground cells.
	 * @param obstacleCells Output obstacle cells.
	 * @param emptyCells Output empty/free cells. Filled when `Grid/RayTracing` is true:
	 *                   3D via OctoMap if `Grid/3D` and OctoMap support are enabled
	 *                   (`Grid/RangeMax`, `Grid/CellSize`); otherwise 2D ray fill via
	 *                   `occupancy2DFromLaserScan` when `Grid/3D` is false (`Grid/RangeMax`,
	 *                   `Grid/CellSize`).
	 * @param viewPointInOut View point (may be rotated if @ref isMapFrameProjection()).
	 */
	void createLocalMap(
			const LaserScan & cloud,
			const Transform & pose,
			cv::Mat & groundCells,
			cv::Mat & obstacleCells,
			cv::Mat & emptyCells,
			cv::Point3f & viewPointInOut) const;

protected:
	ParametersMap parameters_;

	unsigned int cloudDecimation_;
	float rangeMax_;
	float rangeMin_;
	std::vector<float> roiRatios_;
	float footprintLength_;
	float footprintWidth_;
	float footprintHeight_;
	int scanDecimation_;
	float cellSize_;
	bool preVoxelFiltering_;
	int occupancySensor_;
	bool projMapFrame_;
	float maxObstacleHeight_;
	int normalKSearch_;
	float groundNormalsUp_;
	float maxGroundAngle_;
	float clusterRadius_;
	int minClusterSize_;
	bool flatObstaclesDetected_;
	float minGroundHeight_;
	float maxGroundHeight_;
	bool normalsSegmentation_;
	bool grid3D_;
	bool groundIsObstacle_;
	float noiseFilteringRadius_;
	int noiseFilteringMinNeighbors_;
	bool scan2dUnknownSpaceFilled_;
	bool rayTracing_;
};

} /* namespace rtabmap */

#include <rtabmap/core/impl/LocalMapMaker.hpp>

#endif /* SRC_LOCAL_MAP_H_ */
