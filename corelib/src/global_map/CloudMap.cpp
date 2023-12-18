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

#include <rtabmap/core/global_map/CloudMap.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#include <pcl/io/pcd_io.h>

namespace rtabmap {

CloudMap::CloudMap(const LocalGridCache * cache, const ParametersMap & parameters) :
	GlobalMap(cache, parameters),
	assembledGround_(new pcl::PointCloud<pcl::PointXYZRGB>),
	assembledObstacles_(new pcl::PointCloud<pcl::PointXYZRGB>),
	assembledEmptyCells_(new pcl::PointCloud<pcl::PointXYZ>)
{
}

void CloudMap::clear()
{
	assembledGround_->clear();
	assembledObstacles_->clear();
	assembledEmptyCells_->clear();
	GlobalMap::clear();
}

void CloudMap::assemble(const std::list<std::pair<int, Transform> > & newPoses)
{
	UTimer timer;

	bool assembledGroundUpdated = false;
	bool assembledObstaclesUpdated = false;
	bool assembledEmptyCellsUpdated = false;

	if(!cache().empty())
	{
		UDEBUG("Updating from cache");
		for(std::list<std::pair<int, Transform> >::const_iterator iter = newPoses.begin(); iter!=newPoses.end(); ++iter)
		{
			if(uContains(cache(), iter->first))
			{
				const LocalGrid & localGrid = cache().at(iter->first);

				UDEBUG("Adding grid %d: ground=%d obstacles=%d empty=%d", iter->first, localGrid.groundCells.cols, localGrid.obstacleCells.cols, localGrid.emptyCells.cols);

				addAssembledNode(iter->first, iter->second);

				//ground
				if(localGrid.groundCells.cols)
				{
					if(localGrid.groundCells.rows > 1 && localGrid.groundCells.cols == 1)
					{
						UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", localGrid.groundCells.rows, localGrid.groundCells.cols);
					}

					*assembledGround_ += *util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(localGrid.groundCells), iter->second, 0, 255, 0);
					assembledGroundUpdated = true;
				}

				//empty
				if(localGrid.emptyCells.cols)
				{
					if(localGrid.emptyCells.rows > 1 && localGrid.emptyCells.cols == 1)
					{
						UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", localGrid.emptyCells.rows, localGrid.emptyCells.cols);
					}

					*assembledEmptyCells_ += *util3d::laserScanToPointCloud(LaserScan::backwardCompatibility(localGrid.emptyCells), iter->second);
					assembledEmptyCellsUpdated = true;
				}

				//obstacles
				if(localGrid.obstacleCells.cols)
				{
					if(localGrid.obstacleCells.rows > 1 && localGrid.obstacleCells.cols == 1)
					{
						UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", localGrid.obstacleCells.rows, localGrid.obstacleCells.cols);
					}

					*assembledObstacles_ += *util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(localGrid.obstacleCells), iter->second, 255, 0, 0);
					assembledObstaclesUpdated = true;
				}
			}
		}
	}


	if(assembledGroundUpdated && assembledGround_->size() > 1)
	{
		assembledGround_ = util3d::voxelize(assembledGround_, cellSize_);
	}
	if(assembledObstaclesUpdated && assembledGround_->size() > 1)
	{
		assembledObstacles_ = util3d::voxelize(assembledObstacles_, cellSize_);
	}
	if(assembledEmptyCellsUpdated && assembledEmptyCells_->size() > 1)
	{
		assembledEmptyCells_ = util3d::voxelize(assembledEmptyCells_, cellSize_);
	}

	UDEBUG("Occupancy Grid update time = %f s", timer.ticks());
}

unsigned long CloudMap::getMemoryUsed() const
{
	unsigned long memoryUsage = GlobalMap::getMemoryUsed();

	if(assembledGround_.get())
	{
		memoryUsage += assembledGround_->points.size() * sizeof(pcl::PointXYZRGB);
	}
	if(assembledObstacles_.get())
	{
		memoryUsage += assembledObstacles_->points.size() * sizeof(pcl::PointXYZRGB);
	}
	if(assembledEmptyCells_.get())
	{
		memoryUsage += assembledEmptyCells_->points.size() * sizeof(pcl::PointXYZ);
	}
	return memoryUsage;
}

}
