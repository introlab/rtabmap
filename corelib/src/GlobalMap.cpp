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

GlobalMap::GlobalMap(const ParametersMap & parameters) :
	cellSize_(Parameters::defaultGridCellSize()),
	updateError_(Parameters::defaultGridGlobalUpdateError()),
	occupancyThr_(Parameters::defaultGridGlobalOccupancyThr()),
	probHit_(logodds(Parameters::defaultGridGlobalProbHit())),
	probMiss_(logodds(Parameters::defaultGridGlobalProbMiss())),
	probClampingMin_(logodds(Parameters::defaultGridGlobalProbClampingMin())),
	probClampingMax_(logodds(Parameters::defaultGridGlobalProbClampingMax()))
{
	minValues_[0] = minValues_[1] = minValues_[2] = 0.0;
	maxValues_[0] = maxValues_[1] = maxValues_[2] = 0.0;

	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	UASSERT(cellSize_>0.0f);

	Parameters::parse(parameters, Parameters::kGridGlobalUpdateError(), updateError_);

	UDEBUG("cellSize_           =%f", cellSize_);
	UDEBUG("updateError_        =%f", updateError_);

	// Probabilistic parameters
	Parameters::parse(parameters, Parameters::kGridGlobalOccupancyThr(), occupancyThr_);
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbHit(), probHit_))
	{
		probHit_ = logodds(probHit_);
		UASSERT_MSG(probHit_ >= 0.0f, uFormat("probHit_=%f",probHit_).c_str());
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbMiss(), probMiss_))
	{
		probMiss_ = logodds(probMiss_);
		UASSERT_MSG(probMiss_ <= 0.0f, uFormat("probMiss_=%f",probMiss_).c_str());
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbClampingMin(), probClampingMin_))
	{
		probClampingMin_ = logodds(probClampingMin_);
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbClampingMax(), probClampingMax_))
	{
		probClampingMax_ = logodds(probClampingMax_);
	}
	UASSERT(probClampingMax_ > probClampingMin_);
}

GlobalMap::~GlobalMap()
{
	clear();
}

void GlobalMap::clear(bool keepCache)
{
	UDEBUG("Clearing (keep cache = %d)", keepCache?1:0);
	if(!keepCache)
	{
		cache_.clear();
		cacheViewPoints_.clear();
	}
	addedNodes_.clear();
	minValues_[0] = minValues_[1] = minValues_[2] = 0.0;
	maxValues_[0] = maxValues_[1] = maxValues_[2] = 0.0;
}

unsigned long GlobalMap::getMemoryUsed() const
{
	unsigned long memoryUsage = 0;

	memoryUsage += cache_.size()*(sizeof(int) + sizeof(std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat>) + sizeof(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator)) + sizeof(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >);
	for(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::const_iterator iter=cache_.begin(); iter!=cache_.end(); ++iter)
	{
		memoryUsage += iter->second.first.first.total() * iter->second.first.first.elemSize();
		memoryUsage += iter->second.first.second.total() * iter->second.first.second.elemSize();
		memoryUsage += iter->second.second.total() * iter->second.second.elemSize();
	}

	for(std::map<int, cv::Point3f>::const_iterator iter=cacheViewPoints_.begin(); iter!=cacheViewPoints_.end(); ++iter)
	{
		memoryUsage += sizeof(int);
		memoryUsage += sizeof(cv::Point3f);
	}

	memoryUsage += addedNodes_.size()*(sizeof(int) + sizeof(Transform)+ sizeof(float)*12 + sizeof(std::map<int, Transform>::iterator)) + sizeof(std::map<int, Transform>);

	return memoryUsage;
}

void GlobalMap::addToCache(int nodeId,
		const cv::Mat & ground,
		const cv::Mat & obstacles,
		const cv::Mat & empty,
		const cv::Point3f & viewPoint)
{
	UDEBUG("nodeId=%d (ground=%d obstacles=%d empty=%d)", nodeId, ground.cols, obstacles.cols, empty.cols);
	if(nodeId < 0)
	{
		UWARN("Cannot add nodes with negative id (nodeId=%d)", nodeId);
		return;
	}
	uInsert(cache_, std::make_pair(nodeId==0?-1:nodeId, std::make_pair(std::make_pair(ground, obstacles), empty)));
	uInsert(cacheViewPoints_, std::make_pair(nodeId==0?-1:nodeId, viewPoint));
}

bool GlobalMap::update(const std::map<int, Transform> & poses)
{
	UDEBUG("Update (poses=%d addedNodes_=%d)", (int)poses.size(), (int)addedNodes_.size());

	// First, check of the graph has changed. If so, re-create the octree by moving all occupied nodes.
	bool graphOptimized = false; // If a loop closure happened (e.g., poses are modified)
	bool graphChanged = addedNodes_.size()>0; // If the new map doesn't have any node from the previous map
	float updateErrorSqrd = updateError_*updateError_;
	for(std::map<int, Transform>::iterator iter=addedNodes_.begin(); iter!=addedNodes_.end(); ++iter)
	{
		std::map<int, Transform>::const_iterator jter = poses.find(iter->first);
		if(jter != poses.end())
		{
			graphChanged = false;
			UASSERT(!iter->second.isNull() && !jter->second.isNull());
			if(iter->second.getDistanceSquared(jter->second) > updateErrorSqrd)
			{
				graphOptimized = true;
			}
		}
		else
		{
			UDEBUG("Updated pose for node %d is not found, some points may not be copied. Use negative ids to just update cell values without adding new ones.", jter->first);
		}
	}

	if(graphOptimized || graphChanged)
	{
		// clear all but keep cache
		clear(true);
	}

	std::list<std::pair<int, Transform> > orderedPoses;

	// add old poses that were not in the current map (they were just retrieved from LTM)
	for(std::map<int, Transform>::const_iterator iter=poses.lower_bound(1); iter!=poses.end(); ++iter)
	{
		if(!isNodeAssembled(iter->first))
		{
			UDEBUG("Pose %d not found in current added poses, it will be added to map", iter->first);
			orderedPoses.push_back(*iter);
		}
	}

	// insert zero after
	if(poses.find(0) != poses.end())
	{
		orderedPoses.push_back(std::make_pair(-1, poses.at(0)));
	}

	if(!orderedPoses.empty())
	{
		assemble(orderedPoses);
	}

	//clear only negative ids
	for(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator iter=cache_.begin(); iter!=cache_.end();)
	{
		if(iter->first < 0)
		{
			cacheViewPoints_.erase(iter->first);
			cache_.erase(iter++);
		}
		else
		{
			break;
		}
	}

	return !orderedPoses.empty();
}

void GlobalMap::addAssembledNode(int id, const Transform & pose)
{
	if(id > 0)
	{
		uInsert(addedNodes_, std::make_pair(id, pose));
	}
}


} // namespace rtabmap
