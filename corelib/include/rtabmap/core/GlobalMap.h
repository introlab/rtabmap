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

#ifndef SRC_MAP_H_
#define SRC_MAP_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <list>

namespace rtabmap {

/**
 * @class GlobalMap
 * @brief Abstract base for assembling per-node @ref LocalGrid data into a global map.
 *
 * Subclasses (@ref OccupancyGrid, @ref OctoMap, @ref GridMap, @ref CloudMap) implement
 * @ref assemble() to merge new node poses from @ref LocalGridCache into their representation.
 *
 * @ref update() decides which poses need assembly (not yet in @ref addedNodes(), present in
 * cache, id &gt; 0) and may call @ref clear() when @ref fullUpdateNeeded() detects graph
 * optimization or a disjoint pose set (see @ref kGridGlobalUpdateError).
 *
 * Log-odds helpers @ref logodds() and @ref probability() convert between occupancy
 * probability and the internal log-odds representation used for global hit/miss/clamping
 * parameters (GridGlobal/Prob*).
 */
class RTABMAP_CORE_EXPORT GlobalMap
{
public:
	/** @brief Converts probability in (0, 1) to log-odds. */
	inline static float logodds(double probability)
	{
		return (float) log(probability/(1-probability));
	}

	/** @brief Converts log-odds back to probability in (0, 1). */
	inline static double probability(double logodds)
	{
		return 1. - ( 1. / (1. + exp(logodds)));
	}

public:
	virtual ~GlobalMap();

	/**
	 * @brief True if the map should be rebuilt from cache (loop closure or disjoint graph).
	 *
	 * Compares @p poses to @ref addedNodes() using @ref getUpdateError(), which is set from
	 * **Grid/GlobalUpdateError** (`Parameters::kGridGlobalUpdateError()`): if any assembled
	 * node's pose moved farther than that threshold (meters), or if none of the assembled
	 * nodes appear in @p poses, returns true and @ref update() will call @ref clear() first.
	 *
	 * @param poses Current graph poses (node id → transform).
	 */
	bool fullUpdateNeeded(const std::map<int, Transform> & poses) const;

	/**
	 * @brief Incrementally assemble new nodes from @p poses.
	 * @param poses Graph poses; ids &gt; 0 with a cached @ref LocalGrid are candidates.
	 * @return True if @ref assemble() was called (at least one new pose processed).
	 */
	bool update(const std::map<int, Transform> & poses);

	/** @brief Clears assembled nodes and grid bounds; does not clear @ref LocalGridCache. */
	virtual void clear();

	/** @return Grid cell size in meters (Grid/CellSize). */
	float getCellSize() const {return cellSize_;}
	/** @return Pose change threshold for full rebuild (Grid/GlobalUpdateError). */
	float getUpdateError() const {return updateError_;}
	/** @return Poses of nodes already assembled into the global map. */
	const std::map<int, Transform> & addedNodes() const {return addedNodes_;}

	/** @brief 2D grid minimum (x, y) in meters. */
	void getGridMin(double & x, double & y) const {x=minValues_[0];y=minValues_[1];}
	/** @brief 2D grid maximum (x, y) in meters. */
	void getGridMax(double & x, double & y) const {x=maxValues_[0];y=maxValues_[1];}
	/** @brief 3D grid minimum (x, y, z) in meters. */
	void getGridMin(double & x, double & y, double & z) const {x=minValues_[0];y=minValues_[1];z=minValues_[2];}
	/** @brief 3D grid maximum (x, y, z) in meters. */
	void getGridMax(double & x, double & y, double & z) const {x=maxValues_[0];y=maxValues_[1];z=maxValues_[2];}

	/** @brief Approximate memory used by assembled-node bookkeeping (bytes). */
	virtual unsigned long getMemoryUsed() const;

protected:
	/**
	 * @brief Constructs the base map; subclasses call from their constructor.
	 * @param cache Non-null cache of per-node local grids.
	 * @param parameters Optional Grid/ and GridGlobal/ parameter overrides.
	 */
	GlobalMap(const LocalGridCache * cache, const ParametersMap & parameters = ParametersMap());

	/** @brief Subclass hook: merge @p newPoses into the global map; call @ref addAssembledNode(). */
	virtual void assemble(const std::list<std::pair<int, Transform> > & newPoses) = 0;

	/** @return Cached local grids keyed by node id. */
	const std::map<int, LocalGrid> & cache() const {return cache_->localGrids();}

	/** @return Poses of nodes already assembled (same as @ref addedNodes()). */
	const std::map<int, Transform> & assembledNodes() const {return addedNodes_;}
	/** @return True if @p id is already in the assembled set. */
	bool isNodeAssembled(int id) {return addedNodes_.find(id) != addedNodes_.end();}
	/** @brief Records a successfully assembled node (ids &lt;= 0 are ignored). */
	void addAssembledNode(int id, const Transform & pose);

protected:
	float cellSize_;
	float updateError_;

	float occupancyThr_;
	float logOddsHit_;
	float logOddsMiss_;
	float logOddsClampingMin_;
	float logOddsClampingMax_;

	double minValues_[3];
	double maxValues_[3];

private:
	const LocalGridCache * cache_;
	std::map<int, Transform> addedNodes_;
};

} /* namespace rtabmap */

#endif /* SRC_MAP_H_ */
