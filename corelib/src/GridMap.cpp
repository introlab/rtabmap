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

#include <rtabmap/core/GridMap.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <list>

namespace rtabmap {

GridMap::GridMap(const ParametersMap & parameters) :
	Map(parameters),
	minMapSize_(Parameters::defaultGridGlobalMinSize())
{
	Parameters::parse(parameters, Parameters::kGridGlobalMinSize(), minMapSize_);
}

void GridMap::clear(bool keepCache)
{
	gridMap_ = grid_map::GridMap();
	Map::clear(keepCache);
}

bool GridMap::update(const std::map<int, Transform> & posesIn)
{
	UTimer timer;
	UDEBUG("Update (poses=%d addedNodes_=%d)", (int)posesIn.size(), (int)addedNodes_.size());

	float margin = cellSize_*10.0f;

	float minX=-minMapSize_/2.0f;
	float minY=-minMapSize_/2.0f;
	float maxX=minMapSize_/2.0f;
	float maxY=minMapSize_/2.0f;
	bool undefinedSize = minMapSize_ == 0.0f;
	std::map<int, cv::Mat> occupiedLocalMaps;

	// First, check of the graph has changed. If so, re-create the map by moving all occupied nodes (fullUpdate==false).
	bool graphOptimized, graphChanged;
	std::map<int, Transform> transforms;
	std::map<int, Transform> updatedAddedNodes;
	checkIfMapChanged(posesIn, graphOptimized, graphChanged, transforms, updatedAddedNodes);

	for(std::map<int, Transform>::const_iterator iter=updatedAddedNodes.begin(); iter!=updatedAddedNodes.end(); ++iter)
	{
		float x = iter->second.x();
		float y  =iter->second.y();
		if(undefinedSize)
		{
			minX = maxX = x;
			minY = maxY = y;
			undefinedSize = false;
		}
		else
		{
			if(minX > x)
				minX = x;
			else if(maxX < x)
				maxX = x;

			if(minY > y)
				minY = y;
			else if(maxY < y)
				maxY = y;
		}
	}

	if(graphOptimized || graphChanged)
	{
		if(graphChanged)
		{
			UWARN("Graph has changed! The whole map should be rebuilt.");
		}
		else
		{
			UINFO("Graph optimized!");
		}

		// clear all but keep cache
		clear(true);
	}
	else if(gridMap_.hasBasicLayers())
	{
		// update
		minX=minValues_[0]+margin+cellSize_/2.0f;
		minY=minValues_[1]+margin+cellSize_/2.0f;
		maxX=minValues_[0]+float(gridMap_.getSize()[0])*cellSize_ - margin;
		maxY=minValues_[1]+float(gridMap_.getSize()[1])*cellSize_ - margin;
		undefinedSize = false;
	}

	std::list<std::pair<int, Transform> > poses;
	int lastId = addedNodes_.size()?addedNodes_.rbegin()->first:0;
	UDEBUG("Last id = %d", lastId);

	// add old poses that were not in the current map (they were just retrieved from LTM)
	for(std::map<int, Transform>::const_iterator iter=posesIn.lower_bound(1); iter!=posesIn.end(); ++iter)
	{
		if(addedNodes_.find(iter->first) == addedNodes_.end())
		{
			UDEBUG("Pose %d not found in current added poses, it will be added to map", iter->first);
			poses.push_back(*iter);
		}
	}

	// insert zero after
	if(posesIn.find(0) != posesIn.end())
	{
		poses.push_back(std::make_pair(-1, posesIn.at(0)));
	}

	if(!poses.empty())
	{
		for(std::list<std::pair<int, Transform> >::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			UASSERT(!iter->second.isNull());

			float x = iter->second.x();
			float y  =iter->second.y();
			if(undefinedSize)
			{
				minX = maxX = x;
				minY = maxY = y;
				undefinedSize = false;
			}
			else
			{
				if(minX > x)
					minX = x;
				else if(maxX < x)
					maxX = x;

				if(minY > y)
					minY = y;
				else if(maxY < y)
					maxY = y;
			}
		}

		if(!cache_.empty())
		{
			UDEBUG("Updating from cache");
			for(std::list<std::pair<int, Transform> >::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				if(uContains(cache_, iter->first))
				{
					const std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> & pair = cache_.at(iter->first);

					UDEBUG("Adding grid %d: ground=%d obstacles=%d empty=%d", iter->first, pair.first.first.cols, pair.first.second.cols, pair.second.cols);

					//ground
					cv::Mat occupied;
					if(pair.first.first.cols || pair.first.second.cols)
					{
						occupied = cv::Mat(1, pair.first.first.cols+pair.first.second.cols, CV_32FC3);
					}
					if(pair.first.first.cols)
					{
						if(pair.first.first.rows > 1 && pair.first.first.cols == 1)
						{
							UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", pair.first.first.rows, pair.first.first.cols);
						}
						for(int i=0; i<pair.first.first.cols; ++i)
						{
							const float * vi = pair.first.first.ptr<float>(0,i);
							float * vo = occupied.ptr<float>(0,i);
							cv::Point3f vt;
							if(pair.first.first.channels() != 2 && pair.first.first.channels() != 5)
							{
								vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], vi[2]), iter->second);
							}
							else
							{
								vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], 0), iter->second);
							}
							vo[0] = vt.x;
							vo[1] = vt.y;
							vo[2] = vt.z;
							if(minX > vo[0])
								minX = vo[0];
							else if(maxX < vo[0])
								maxX = vo[0];

							if(minY > vo[1])
								minY = vo[1];
							else if(maxY < vo[1])
								maxY = vo[1];
						}
					}

					//obstacles
					if(pair.first.second.cols)
					{
						if(pair.first.second.rows > 1 && pair.first.second.cols == 1)
						{
							UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", pair.first.second.rows, pair.first.second.cols);
						}
						for(int i=0; i<pair.first.second.cols; ++i)
						{
							const float * vi = pair.first.second.ptr<float>(0,i);
							float * vo = occupied.ptr<float>(0,i+pair.first.first.cols);
							cv::Point3f vt;
							if(pair.first.second.channels() != 2 && pair.first.second.channels() != 5)
							{
								vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], vi[2]), iter->second);
							}
							else
							{
								vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], 0), iter->second);
							}
							vo[0] = vt.x;
							vo[1] = vt.y;
							vo[2] = vt.z;
							if(minX > vo[0])
								minX = vo[0];
							else if(maxX < vo[0])
								maxX = vo[0];

							if(minY > vo[1])
								minY = vo[1];
							else if(maxY < vo[1])
								maxY = vo[1];
						}
					}
					uInsert(occupiedLocalMaps, std::make_pair(iter->first, occupied));
				}
			}
		}

		if(minX != maxX && minY != maxY)
		{
			//Get map size
			float xMin = minX-margin;
			xMin -= cellSize_/2.0f;
			float yMin = minY-margin;
			yMin -= cellSize_/2.0f;
			float xMax = maxX+margin;
			float yMax = maxY+margin;

			if(fabs((yMax - yMin) / cellSize_) > 99999 ||
			   fabs((xMax - xMin) / cellSize_) > 99999)
			{
				UERROR("Large map size!! map min=(%f, %f) max=(%f,%f). "
						"There's maybe an error with the poses provided! The map will not be created!",
						xMin, yMin, xMax, yMax);
			}
			else
			{
				UDEBUG("map min=(%f, %f) odlMin(%f,%f) max=(%f,%f)", xMin, yMin, minValues_[0], minValues_[1], xMax, yMax);
				cv::Size newMapSize((xMax - xMin) / cellSize_+0.5f, (yMax - yMin) / cellSize_+0.5f);
				if(!gridMap_.hasBasicLayers())
				{
					UDEBUG("Map empty!");
					grid_map::Length length = grid_map::Length(xMax - xMin, yMax - yMin);
					grid_map::Position position = grid_map::Position((xMax+xMin)/2.0f, (yMax+yMin)/2.0f);

					UDEBUG("length: %f, %f position: %f, %f", length[0], length[1], position[0], position[1]);
					gridMap_.setGeometry(length, cellSize_, position);
					UDEBUG("size: %d, %d", gridMap_.getSize()[0], gridMap_.getSize()[1]);
					// Add elevation layer
					gridMap_.add("elevation");
					gridMap_.add("node_ids");
					gridMap_.setBasicLayers({"elevation"});
				}
				else
				{
					if(xMin == minValues_[0] && yMin == minValues_[1] &&
							newMapSize.width == gridMap_.getSize()[0] &&
							newMapSize.height == gridMap_.getSize()[1])
					{
						// same map size and origin, don't do anything
						UDEBUG("Map same size!");
					}
					else
					{
						UASSERT_MSG(xMin <= minValues_[0]+cellSize_/2, uFormat("xMin=%f, xMin_=%f, cellSize_=%f", xMin, minValues_[0], cellSize_).c_str());
						UASSERT_MSG(yMin <= minValues_[1]+cellSize_/2, uFormat("yMin=%f, yMin_=%f, cellSize_=%f", yMin, minValues_[1], cellSize_).c_str());
						UASSERT_MSG(xMax >= minValues_[0]+float(gridMap_.getSize()[0])*cellSize_ - cellSize_/2, uFormat("xMin=%f, xMin_=%f, cols=%d cellSize_=%f", xMin, minValues_[0], gridMap_.getSize()[0], cellSize_).c_str());
						UASSERT_MSG(yMax >= minValues_[1]+float(gridMap_.getSize()[1])*cellSize_ - cellSize_/2, uFormat("yMin=%f, yMin_=%f, cols=%d cellSize_=%f", yMin, minValues_[1], gridMap_.getSize()[1], cellSize_).c_str());

						UDEBUG("Copy map");
						// copy the old map in the new map
						// make sure the translation is cellSize
						int deltaX = 0;
						if(xMin < minValues_[0])
						{
							deltaX = (minValues_[0] - xMin) / cellSize_ + 1.0f;
							xMin = minValues_[0]-float(deltaX)*cellSize_;
						}
						int deltaY = 0;
						if(yMin < minValues_[1])
						{
							deltaY = (minValues_[1] - yMin) / cellSize_ + 1.0f;
							yMin = minValues_[1]-float(deltaY)*cellSize_;
						}
						UDEBUG("deltaX=%d, deltaY=%d", deltaX, deltaY);
						newMapSize.width = (xMax - xMin) / cellSize_+0.5f;
						newMapSize.height = (yMax - yMin) / cellSize_+0.5f;
						UDEBUG("%d/%d -> %d/%d", gridMap_.getSize()[0], gridMap_.getSize()[1], newMapSize.width, newMapSize.height);
						UASSERT(newMapSize.width >= gridMap_.getSize()[0] && newMapSize.height >= gridMap_.getSize()[1]);
						UASSERT(newMapSize.width >= gridMap_.getSize()[0]+deltaX && newMapSize.height >= gridMap_.getSize()[1]+deltaY);
						UASSERT(deltaX>=0 && deltaY>=0);

						grid_map::Length length = grid_map::Length(xMax - xMin, yMax - yMin);
						grid_map::Position position = grid_map::Position((xMax+xMin)/2.0f, (yMax+yMin)/2.0f);

						grid_map::GridMap tmpExtendedMap;
						tmpExtendedMap.setGeometry(length, cellSize_, position);

						UDEBUG("%d/%d -> %d/%d", gridMap_.getSize()[0], gridMap_.getSize()[1], tmpExtendedMap.getSize()[0], tmpExtendedMap.getSize()[1]);

						UDEBUG("extendToInclude (%f,%f,%f,%f) -> (%f,%f,%f,%f)",
								gridMap_.getLength()[0], gridMap_.getLength()[1],
								gridMap_.getPosition()[0], gridMap_.getPosition()[1],
								tmpExtendedMap.getLength()[0], tmpExtendedMap.getLength()[1],
								tmpExtendedMap.getPosition()[0], tmpExtendedMap.getPosition()[1]);
						if(!gridMap_.extendToInclude(tmpExtendedMap))
						{
							UERROR("Failed to update size of the grid map");
						}
						UDEBUG("Updated side: %d %d", gridMap_.getSize()[0], gridMap_.getSize()[1]);
					}
				}
				UDEBUG("map %d %d", gridMap_.getSize()[0], gridMap_.getSize()[1]);
				if(poses.size())
				{
					UDEBUG("first pose= %d last pose=%d", poses.begin()->first, poses.rbegin()->first);
				}
				grid_map::Matrix& gridMapData = gridMap_["elevation"];
				grid_map::Matrix& gridMapNodeIds = gridMap_["node_ids"];
				for(std::list<std::pair<int, Transform> >::const_iterator kter = poses.begin(); kter!=poses.end(); ++kter)
				{
					std::map<int, cv::Mat>::iterator iter = occupiedLocalMaps.find(kter->first);
					if(iter!=occupiedLocalMaps.end())
					{
						if(kter->first > 0)
						{
							uInsert(addedNodes_, *kter);
						}

						for(int i=0; i<iter->second.cols; ++i)
						{
							float * ptf = iter->second.ptr<float>(0,i);
							grid_map::Position position(ptf[0], ptf[1]);
							grid_map::Index index;
							if(gridMap_.getIndex(position, index))
							{
								// If no elevation has been set, use current elevation.
								if (!gridMap_.isValid(index))
								{
									gridMapData(index(0), index(1)) = ptf[2];
									gridMapNodeIds(index(0), index(1)) = kter->first;
								}
								else
								{
									if ((gridMapData(index(0), index(1)) < ptf[2] && (gridMapNodeIds(index(0), index(1)) <= kter->first || kter->first == -1)) ||
										gridMapNodeIds(index(0), index(1)) < kter->first)
									{
										gridMapData(index(0), index(1)) = ptf[2];
										gridMapNodeIds(index(0), index(1)) = kter->first;
									}
								}
							}
							else
							{
								UERROR("Outside map!? (%d) (%f,%f) -> (%d,%d)", i, ptf[0], ptf[1], index[0], index[1]);
							}
						}
					}
				}

				minValues_[0] = xMin;
				minValues_[1] = yMin;
			}
		}
	}
	UDEBUG("");

	//clear only negative ids
	for(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator iter=cache_.begin(); iter!=cache_.end();)
	{
		if(iter->first < 0)
		{
			cache_.erase(iter++);
		}
		else
		{
			break;
		}
	}

	bool updated = !poses.empty() || graphOptimized || graphChanged;
	UDEBUG("Occupancy Grid update time = %f s (updated=%s)", timer.ticks(), updated?"true":"false");
	return updated;
}

}
