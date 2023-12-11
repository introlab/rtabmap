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

#include <rtabmap/core/Map.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>

namespace rtabmap {

Map::Map(const ParametersMap & parameters) :
	cellSize_(Parameters::defaultGridCellSize()),
	fullUpdate_(Parameters::defaultGridGlobalFullUpdate()),
	updateError_(Parameters::defaultGridGlobalUpdateError())
{
	minValues_[0] = minValues_[1] = minValues_[2] = 0.0;
	maxValues_[0] = maxValues_[1] = maxValues_[2] = 0.0;

	cellSize_ = Parameters::defaultGridCellSize();
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	UASSERT(cellSize_>0.0f);

	Parameters::parse(parameters, Parameters::kGridGlobalFullUpdate(), fullUpdate_);
	Parameters::parse(parameters, Parameters::kGridGlobalUpdateError(), updateError_);

	UDEBUG("cellSize_           =%f", cellSize_);
	UDEBUG("fullUpdate_         =%s", fullUpdate_?"true":"false");
	UDEBUG("updateError_        =%f", updateError_);
}

Map::~Map()
{
	clear();
}

void Map::clear(bool keepCache)
{
	if(!keepCache)
	{
		cache_.clear();
		cacheViewPoints_.clear();
	}
	addedNodes_.clear();
	minValues_[0] = minValues_[1] = minValues_[2] = 0.0;
	maxValues_[0] = maxValues_[1] = maxValues_[2] = 0.0;
}

unsigned long Map::getMemoryUsed() const
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

void Map::addToCache(int nodeId,
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

void Map::checkIfMapChanged(
		const std::map<int, Transform> & poses,
		bool & graphOptimized,
		bool & graphChanged,
		std::map<int, Transform> & transforms,
		std::map<int, Transform> & updatedAddedNodes)
{
	graphOptimized = false; // If a loop closure happened (e.g., poses are modified)
	graphChanged = addedNodes_.size()>0; // If the new map doesn't have any node from the previous map
	transforms.clear();
	updatedAddedNodes.clear();
	float updateErrorSqrd = updateError_*updateError_;
	for(std::map<int, Transform>::iterator iter=addedNodes_.begin(); iter!=addedNodes_.end(); ++iter)
	{
		std::map<int, Transform>::const_iterator jter = poses.find(iter->first);
		if(jter != poses.end())
		{
			graphChanged = false;
			UASSERT(!iter->second.isNull() && !jter->second.isNull());
			Transform t = Transform::getIdentity();
			if(iter->second.getDistanceSquared(jter->second) > updateErrorSqrd)
			{
				t = jter->second * iter->second.inverse();
				graphOptimized = true;
			}
			transforms.insert(std::make_pair(jter->first, t));
			updatedAddedNodes.insert(std::make_pair(jter->first, jter->second));
		}
		else
		{
			UDEBUG("Updated pose for node %d is not found, some points may not be copied. Use negative ids to just update cell values without adding new ones.", jter->first);
		}
	}
}

} // namespace rtabmap
