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

#include <rtabmap/core/GlobalMap.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>

namespace rtabmap {

LocalGrid::LocalGrid(const cv::Mat & groundIn,
			 const cv::Mat & obstaclesIn,
			 const cv::Mat & emptyIn,
			 float cellSizeIn,
			 const cv::Point3f & viewPointIn) :
		groundCells(groundIn),
		obstacleCells(obstaclesIn),
		emptyCells(emptyIn),
		cellSize(cellSizeIn),
		viewPoint(viewPointIn)
{
	UASSERT(cellSize > 0.0f);
}

bool LocalGrid::is3D() const
{
	return (groundCells.empty() || groundCells.type() == CV_32FC3 || groundCells.type() == CV_32FC(4) || groundCells.type() == CV_32FC(6)) &&
		   (obstacleCells.empty() || obstacleCells.type() == CV_32FC3 || obstacleCells.type() == CV_32FC(4) || obstacleCells.type() == CV_32FC(6)) &&
		   (emptyCells.empty() || emptyCells.type() == CV_32FC3 || emptyCells.type() == CV_32FC(4) || emptyCells.type() == CV_32FC(6));
}

void LocalGridCache::add(int nodeId,
		const cv::Mat & ground,
		const cv::Mat & obstacles,
		const cv::Mat & empty,
		float cellSize,
		const cv::Point3f & viewPoint)
{
	add(nodeId, LocalGrid(ground, obstacles, empty, cellSize, viewPoint));
}

void LocalGridCache::add(int nodeId, const LocalGrid & localGrid)
{
	UDEBUG("nodeId=%d (ground=%d/%d obstacles=%d/%d empty=%d/%d)",
			nodeId, localGrid.groundCells.cols,  localGrid.groundCells.channels(),  localGrid.obstacleCells.cols,  localGrid.obstacleCells.channels(), localGrid.emptyCells.cols,  localGrid.emptyCells.channels());
	if(nodeId < 0)
	{
		UWARN("Cannot add nodes with negative id (nodeId=%d)", nodeId);
		return;
	}
	uInsert(localGrids_, std::make_pair(nodeId==0?-1:nodeId, localGrid));
}

bool LocalGridCache::shareTo(int nodeId, LocalGridCache & anotherCache) const
{
	if(uContains(localGrids_, nodeId) && !uContains(anotherCache.localGrids(), nodeId))
	{
		const LocalGrid & localGrid = localGrids_.at(nodeId);
		anotherCache.add(nodeId, localGrid.groundCells, localGrid.obstacleCells, localGrid.emptyCells, localGrid.cellSize, localGrid.viewPoint);
		return true;
	}
	return false;
}

unsigned long LocalGridCache::getMemoryUsed() const
{
	unsigned long memoryUsage = 0;
	memoryUsage += localGrids_.size()*(sizeof(int) + sizeof(LocalGrid) + sizeof(std::map<int, LocalGrid>::iterator)) + sizeof(std::map<int, LocalGrid>);
	for(std::map<int, LocalGrid>::const_iterator iter=localGrids_.begin(); iter!=localGrids_.end(); ++iter)
	{
		memoryUsage += iter->second.groundCells.total() * iter->second.groundCells.elemSize();
		memoryUsage += iter->second.obstacleCells.total() * iter->second.obstacleCells.elemSize();
		memoryUsage += iter->second.emptyCells.total() * iter->second.emptyCells.elemSize();
		memoryUsage += sizeof(int);
		memoryUsage += sizeof(cv::Point3f);
	}
	return memoryUsage;
}

void LocalGridCache::clear(bool temporaryOnly)
{
	if(temporaryOnly)
	{
		//clear only negative ids
		for(std::map<int, LocalGrid>::iterator iter=localGrids_.begin(); iter!=localGrids_.end();)
		{
			if(iter->first < 0)
			{
				localGrids_.erase(iter++);
			}
			else
			{
				break;
			}
		}
	}
	else
	{
		localGrids_.clear();
	}
}

} // namespace rtabmap
