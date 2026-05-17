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

#ifndef SRC_LOCALGRID_H_
#define SRC_LOCALGRID_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <opencv2/core.hpp>
#include <map>

namespace rtabmap {

/**
 * @class LocalGrid
 * @brief Local occupancy grid cells for one map node (ground, obstacles, empty).
 *
 * Each layer is stored as a 1×N `cv::Mat` of cell coordinates in the robot/base
 * frame. Layouts match @ref LaserScan grid storage (@ref LaserScan::Format):
 * - `CV_32FC2` (2 ch): `kXY` — (x, y)
 * - `CV_32FC3` (3 ch): `kXYZ` or `kXYI` — (x, y, z) or (x, y, intensity)
 * - `CV_32FC4` (4 ch): `kXYZI` or `kXYZRGB` — (x, y, z, intensity) or (x, y, z, RGB)
 * - `CV_32FC5` (5 ch): `kXYNormal` or `kXYZIT` — 2D (x, y, nx, ny, nz) or 3D (x, y, z, intensity, time)
 * - `CV_32FC6` (6 ch): `kXYINormal`, `kXYZNormal` or `kXYZIRT`
 * - `CV_32FC7` (7 ch): `kXYZINormal` or `kXYZRGBNormal`
 *
 * @ref is3D() is true when every non-empty layer is `CV_32FC3`, `CV_32FC4` or `CV_32FC6`
 * (5-channel layers are treated as 2D by @ref OccupancyGrid). Extra fields are not used for pose/projection.
 * @ref cellSize is the grid resolution in meters (must be &gt; 0).
 * @ref viewPoint is the sensor/view origin used when the grid was built.
 *
 * @see LaserScan
 * @see LocalGridMaker
 * @see OccupancyGrid
 * @see LocalGridCache
 */
class RTABMAP_CORE_EXPORT LocalGrid
{
public:
	/**
	 * @brief Builds a local grid from cell matrices.
	 * @param ground Ground cell coordinates (may be empty).
	 * @param obstacles Obstacle cell coordinates (may be empty).
	 * @param empty Empty/free cell coordinates (may be empty).
	 * @param cellSize Grid cell size in meters (must be &gt; 0).
	 * @param viewPoint View/sensor origin in the grid frame.
	 */
	LocalGrid(const cv::Mat & ground,
		 const cv::Mat & obstacles,
		 const cv::Mat & empty,
		 float cellSize,
		 const cv::Point3f & viewPoint = cv::Point3f(0,0,0));
	virtual ~LocalGrid() {}

	/** @return True if every non-empty layer is `CV_32FC3`, `CV_32FC4` or `CV_32FC6` (see class doc). */
	bool is3D() const;

	cv::Mat groundCells;   ///< Ground cells (1×N, `CV_32FC2`–`CV_32FC7`, see class doc).
	cv::Mat obstacleCells; ///< Obstacle cells (same layout as @ref groundCells).
	cv::Mat emptyCells;    ///< Empty/free cells (same layout as @ref groundCells).
	float cellSize;        ///< Cell size in meters.
	cv::Point3f viewPoint; ///< View point used to build the grid.
};

/**
 * @class LocalGridCache
 * @brief Cache of @ref LocalGrid entries keyed by map node id.
 *
 * Used by global mapping (e.g. @ref OccupancyGrid) to hold per-node local grids
 * before assembly. Node id `0` is stored as `-1` (temporary grid). Negative ids
 * passed to @ref add() are rejected.
 */
class RTABMAP_CORE_EXPORT LocalGridCache
{
public:
	LocalGridCache() {}
	virtual ~LocalGridCache() {}

	/** @brief Inserts or replaces the grid for @p nodeId (from separate cell mats). */
	void add(int nodeId,
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			const cv::Mat & empty,
			float cellSize,
			const cv::Point3f & viewPoint = cv::Point3f(0,0,0));

	/** @brief Inserts or replaces the grid for @p nodeId. */
	void add(int nodeId, const LocalGrid & localGrid);

	/**
	 * @brief Copies a grid to @p anotherCache if present here and absent there.
	 * @return True if a grid was shared.
	 */
	bool shareTo(int nodeId, LocalGridCache & anotherCache) const;

	/** @brief Approximate memory used by cached grids (bytes). */
	unsigned long getMemoryUsed() const;

	/**
	 * @brief Removes cached grids.
	 * @param temporaryOnly If true, removes only entries with negative ids (e.g. `-1`).
	 */
	void clear(bool temporaryOnly = false);

	size_t size() const {return localGrids_.size();}
	bool empty() const {return localGrids_.empty();}
	const std::map<int, LocalGrid> & localGrids() const {return localGrids_;}

	std::map<int, LocalGrid>::const_iterator find(int nodeId) const {return localGrids_.find(nodeId);}
	std::map<int, LocalGrid>::const_iterator begin() const {return localGrids_.begin();}
	std::map<int, LocalGrid>::const_iterator end() const {return localGrids_.end();}

private:
	std::map<int, LocalGrid> localGrids_;
};

} /* namespace rtabmap */

#endif /* SRC_LOCALGRID_H_ */
