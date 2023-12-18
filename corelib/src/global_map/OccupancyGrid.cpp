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

#include <rtabmap/core/global_map/OccupancyGrid.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#include <pcl/io/pcd_io.h>

namespace rtabmap {

OccupancyGrid::OccupancyGrid(const LocalGridCache * cache, const ParametersMap & parameters) :
	GlobalMap(cache, parameters),
	minMapSize_(Parameters::defaultGridGlobalMinSize()),
	erode_(Parameters::defaultGridGlobalEroded()),
	footprintRadius_(Parameters::defaultGridGlobalFootprintRadius())
{
	Parameters::parse(parameters, Parameters::kGridGlobalMinSize(), minMapSize_);
	Parameters::parse(parameters, Parameters::kGridGlobalEroded(), erode_);
	Parameters::parse(parameters, Parameters::kGridGlobalFootprintRadius(), footprintRadius_);

	UASSERT(minMapSize_ >= 0.0f);
}

void OccupancyGrid::setMap(const cv::Mat & map, float xMin, float yMin, float cellSize, const std::map<int, Transform> & poses)
{
	UDEBUG("map=%d/%d xMin=%f yMin=%f cellSize=%f poses=%d",
			map.cols, map.rows, xMin, yMin, cellSize, (int)poses.size());
	this->clear();
	if(!poses.empty() && !map.empty())
	{
		UASSERT(cellSize > 0.0f);
		UASSERT(map.type() == CV_8SC1);
		map_ = map.clone();
		mapInfo_ = cv::Mat::zeros(map.size(), CV_32FC4);
		for(int i=0; i<map_.rows; ++i)
		{
			for(int j=0; j<map_.cols; ++j)
			{
				const char value = map_.at<char>(i,j);
				float * info = mapInfo_.ptr<float>(i,j);
				if(value == 0)
				{
					info[3] = logOddsClampingMin_;
				}
				else if(value == 100)
				{
					info[3] = logOddsClampingMax_;
				}
			}
		}
		minValues_[0] = xMin;
		minValues_[1] = yMin;
		cellSize_ = cellSize;
		addAssembledNode(poses.lower_bound(1)->first, poses.lower_bound(1)->second);
	}
}

void OccupancyGrid::clear()
{
	map_ = cv::Mat();
	mapInfo_ = cv::Mat();
	cellCount_.clear();
	GlobalMap::clear();
}

cv::Mat OccupancyGrid::getMap(float & xMin, float & yMin) const
{
	xMin = minValues_[0];
	yMin = minValues_[1];

	cv::Mat map = map_;

	UTimer t;
	if(occupancyThr_ != 0.0f && !map.empty())
	{
		float occThr = logodds(occupancyThr_);
		map = cv::Mat(map.size(), map.type());
		UASSERT(mapInfo_.cols == map.cols && mapInfo_.rows == map.rows);
		for(int i=0; i<map.rows; ++i)
		{
			for(int j=0; j<map.cols; ++j)
			{
				const float * info = mapInfo_.ptr<float>(i, j);
				if(info[3] == 0.0f)
				{
					map.at<char>(i, j) = -1; // unknown
				}
				else if(info[3] >= occThr)
				{
					map.at<char>(i, j) = 100; // unknown
				}
				else
				{
					map.at<char>(i, j) = 0; // empty
				}
			}
		}
		UDEBUG("Converting map from probabilities (thr=%f) = %fs", occupancyThr_, t.ticks());
	}

	if(erode_ && !map.empty())
	{
		map = util3d::erodeMap(map);
		UDEBUG("Eroding map = %fs", t.ticks());
	}
	return map;
}

cv::Mat OccupancyGrid::getProbMap(float & xMin, float & yMin) const
{
	xMin = minValues_[0];
	yMin = minValues_[1];

	cv::Mat map;
	if(!mapInfo_.empty())
	{
		map = cv::Mat(mapInfo_.size(), map_.type());
		for(int i=0; i<map.rows; ++i)
		{
			for(int j=0; j<map.cols; ++j)
			{
				const float * info = mapInfo_.ptr<float>(i, j);
				if(info[3] == 0.0f)
				{
					map.at<char>(i, j) = -1; // unknown
				}
				else
				{
					map.at<char>(i, j) = char(probability(info[3])*100.0f); // empty
				}
			}
		}
	}
	else
	{
		UWARN("Map info is empty, cannot generate probabilistic occupancy grid");
	}
	return map;
}

void OccupancyGrid::assemble(const std::list<std::pair<int, Transform> > & newPoses)
{
	UTimer timer;

	float margin = cellSize_*10.0f+(footprintRadius_>cellSize_*1.5f?float(int(footprintRadius_/cellSize_)+1):0.0f)*cellSize_;

	float minX=-minMapSize_/2.0f;
	float minY=-minMapSize_/2.0f;
	float maxX=minMapSize_/2.0f;
	float maxY=minMapSize_/2.0f;
	bool undefinedSize = minMapSize_ == 0.0f;
	std::map<int, cv::Mat> emptyLocalMaps;
	std::map<int, cv::Mat> occupiedLocalMaps;

	if(!map_.empty())
	{
		// update
		minX=minValues_[0]+margin+cellSize_/2.0f;
		minY=minValues_[1]+margin+cellSize_/2.0f;
		maxX=minValues_[0]+float(map_.cols)*cellSize_ - margin;
		maxY=minValues_[1]+float(map_.rows)*cellSize_ - margin;
		undefinedSize = false;
	}

	for(std::list<std::pair<int, Transform> >::const_iterator iter = newPoses.begin(); iter!=newPoses.end(); ++iter)
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

	if(!cache().empty())
	{
		UDEBUG("Updating from cache");
		for(std::list<std::pair<int, Transform> >::const_iterator iter = newPoses.begin(); iter!=newPoses.end(); ++iter)
		{
			if(uContains(cache(), iter->first))
			{
				const LocalGrid & localGrid = cache().at(iter->first);

				UDEBUG("Adding grid %d: ground=%d obstacles=%d empty=%d", iter->first, localGrid.groundCells.cols, localGrid.obstacleCells.cols, localGrid.emptyCells.cols);

				//ground
				cv::Mat ground;
				if(localGrid.groundCells.cols || localGrid.emptyCells.cols)
				{
					ground = cv::Mat(1, localGrid.groundCells.cols+localGrid.emptyCells.cols, CV_32FC2);
				}
				if(localGrid.groundCells.cols)
				{
					if(localGrid.groundCells.rows > 1 && localGrid.groundCells.cols == 1)
					{
						UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", localGrid.groundCells.rows, localGrid.groundCells.cols);
					}
					for(int i=0; i<localGrid.groundCells.cols; ++i)
					{
						const float * vi = localGrid.groundCells.ptr<float>(0,i);
						float * vo = ground.ptr<float>(0,i);
						cv::Point3f vt;
						if(localGrid.groundCells.channels() != 2 && localGrid.groundCells.channels() != 5)
						{
							vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], vi[2]), iter->second);
						}
						else
						{
							vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], 0), iter->second);
						}
						vo[0] = vt.x;
						vo[1] = vt.y;
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

				//empty
				if(localGrid.emptyCells.cols)
				{
					if(localGrid.emptyCells.rows > 1 && localGrid.emptyCells.cols == 1)
					{
						UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", localGrid.emptyCells.rows, localGrid.emptyCells.cols);
					}
					for(int i=0; i<localGrid.emptyCells.cols; ++i)
					{
						const float * vi = localGrid.emptyCells.ptr<float>(0,i);
						float * vo = ground.ptr<float>(0,i+localGrid.groundCells.cols);
						cv::Point3f vt;
						if(localGrid.emptyCells.channels() != 2 && localGrid.emptyCells.channels() != 5)
						{
							vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], vi[2]), iter->second);
						}
						else
						{
							vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], 0), iter->second);
						}
						vo[0] = vt.x;
						vo[1] = vt.y;
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
				uInsert(emptyLocalMaps, std::make_pair(iter->first, ground));

				//obstacles
				if(localGrid.obstacleCells.cols)
				{
					if(localGrid.obstacleCells.rows > 1 && localGrid.obstacleCells.cols == 1)
					{
						UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", localGrid.obstacleCells.rows, localGrid.obstacleCells.cols);
					}
					cv::Mat obstacles(1, localGrid.obstacleCells.cols, CV_32FC2);
					for(int i=0; i<obstacles.cols; ++i)
					{
						const float * vi = localGrid.obstacleCells.ptr<float>(0,i);
						float * vo = obstacles.ptr<float>(0,i);
						cv::Point3f vt;
						if(localGrid.obstacleCells.channels() != 2 && localGrid.obstacleCells.channels() != 5)
						{
							vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], vi[2]), iter->second);
						}
						else
						{
							vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], 0), iter->second);
						}
						vo[0] = vt.x;
						vo[1] = vt.y;
						if(minX > vo[0])
							minX = vo[0];
						else if(maxX < vo[0])
							maxX = vo[0];

						if(minY > vo[1])
							minY = vo[1];
						else if(maxY < vo[1])
							maxY = vo[1];
					}
					uInsert(occupiedLocalMaps, std::make_pair(iter->first, obstacles));
				}
			}
		}
	}

	cv::Mat map;
	cv::Mat mapInfo;
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
			if(map_.empty())
			{
				UDEBUG("Map empty!");
				map = cv::Mat::ones(newMapSize, CV_8S)*-1;
				mapInfo = cv::Mat::zeros(newMapSize, CV_32FC4);
			}
			else
			{
				if(xMin == minValues_[0] && yMin == minValues_[1] &&
						newMapSize.width == map_.cols &&
						newMapSize.height == map_.rows)
				{
					// same map size and origin, don't do anything
					UDEBUG("Map same size!");
					map = map_;
					mapInfo = mapInfo_;
				}
				else
				{
					UASSERT_MSG(xMin <= minValues_[0]+cellSize_/2, uFormat("xMin=%f, xMin_=%f, cellSize_=%f", xMin, minValues_[0], cellSize_).c_str());
					UASSERT_MSG(yMin <= minValues_[1]+cellSize_/2, uFormat("yMin=%f, yMin_=%f, cellSize_=%f", yMin, minValues_[1], cellSize_).c_str());
					UASSERT_MSG(xMax >= minValues_[0]+float(map_.cols)*cellSize_ - cellSize_/2, uFormat("xMin=%f, xMin_=%f, cols=%d cellSize_=%f", xMin, minValues_[0], map_.cols, cellSize_).c_str());
					UASSERT_MSG(yMax >= minValues_[1]+float(map_.rows)*cellSize_ - cellSize_/2, uFormat("yMin=%f, yMin_=%f, cols=%d cellSize_=%f", yMin, minValues_[1], map_.rows, cellSize_).c_str());

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
					UDEBUG("%d/%d -> %d/%d", map_.cols, map_.rows, newMapSize.width, newMapSize.height);
					UASSERT(newMapSize.width >= map_.cols && newMapSize.height >= map_.rows);
					UASSERT(newMapSize.width >= map_.cols+deltaX && newMapSize.height >= map_.rows+deltaY);
					UASSERT(deltaX>=0 && deltaY>=0);
					map = cv::Mat::ones(newMapSize, CV_8S)*-1;
					mapInfo = cv::Mat::zeros(newMapSize, mapInfo_.type());
					map_.copyTo(map(cv::Rect(deltaX, deltaY, map_.cols, map_.rows)));
					mapInfo_.copyTo(mapInfo(cv::Rect(deltaX, deltaY, map_.cols, map_.rows)));
				}
			}
			UASSERT(map.cols == mapInfo.cols && map.rows == mapInfo.rows);
			UDEBUG("map %d %d", map.cols, map.rows);
			if(newPoses.size())
			{
				UDEBUG("first pose= %d last pose=%d", newPoses.begin()->first, newPoses.rbegin()->first);
			}
			for(std::list<std::pair<int, Transform> >::const_iterator kter = newPoses.begin(); kter!=newPoses.end(); ++kter)
			{
				std::map<int, cv::Mat >::iterator iter = emptyLocalMaps.find(kter->first);
				std::map<int, cv::Mat >::iterator jter = occupiedLocalMaps.find(kter->first);
				if(iter != emptyLocalMaps.end() || jter!=occupiedLocalMaps.end())
				{
					addAssembledNode(kter->first, kter->second);
					std::map<int, std::pair<int, int> >::iterator cter = cellCount_.find(kter->first);
					if(cter == cellCount_.end() && kter->first > 0)
					{
						cter = cellCount_.insert(std::make_pair(kter->first, std::pair<int,int>(0,0))).first;
					}
					if(iter!=emptyLocalMaps.end())
					{
						for(int i=0; i<iter->second.cols; ++i)
						{
							float * ptf = iter->second.ptr<float>(0,i);
							cv::Point2i pt((ptf[0]-xMin)/cellSize_, (ptf[1]-yMin)/cellSize_);
							UASSERT_MSG(pt.y >=0 && pt.y < map.rows && pt.x >= 0 && pt.x < map.cols,
									uFormat("%d: pt=(%d,%d) map=%dx%d rawPt=(%f,%f) xMin=%f yMin=%f channels=%dvs%d",
											kter->first, pt.x, pt.y, map.cols, map.rows, ptf[0], ptf[1], xMin, yMin, iter->second.channels(), mapInfo.channels()-1).c_str());
							char & value = map.at<char>(pt.y, pt.x);
							if(value != -2)
							{
								float * info = mapInfo.ptr<float>(pt.y, pt.x);
								int nodeId = (int)info[0];
								if(value != -1)
								{
									if(kter->first > 0 && (kter->first <  nodeId || nodeId < 0))
									{
										// cannot rewrite on cells referred by more recent nodes
										continue;
									}
									if(nodeId > 0)
									{
										std::map<int, std::pair<int, int> >::iterator eter = cellCount_.find(nodeId);
										UASSERT_MSG(eter != cellCount_.end(), uFormat("current pose=%d nodeId=%d", kter->first, nodeId).c_str());
										if(value == 0)
										{
											eter->second.first -= 1;
										}
										else if(value == 100)
										{
											eter->second.second -= 1;
										}
										if(kter->first < 0)
										{
											eter->second.first += 1;
										}
									}
								}
								if(kter->first > 0)
								{
									info[0] = (float)kter->first;
									info[1] = ptf[0];
									info[2] = ptf[1];
									cter->second.first+=1;
								}
								value = 0; // free space

								// update odds
								if(nodeId != kter->first)
								{
									info[3] += logOddsMiss_;
									if (info[3] < logOddsClampingMin_)
									{
										info[3] = logOddsClampingMin_;
									}
									if (info[3] > logOddsClampingMax_)
									{
										info[3] = logOddsClampingMax_;
									}
								}
							}
						}
					}

					if(footprintRadius_ >= cellSize_*1.5f)
					{
						// place free space under the footprint of the robot
						cv::Point2i ptBegin((kter->second.x()-footprintRadius_-xMin)/cellSize_, (kter->second.y()-footprintRadius_-yMin)/cellSize_);
						cv::Point2i ptEnd((kter->second.x()+footprintRadius_-xMin)/cellSize_, (kter->second.y()+footprintRadius_-yMin)/cellSize_);
						if(ptBegin.x < 0)
							ptBegin.x = 0;
						if(ptEnd.x >= map.cols)
							ptEnd.x = map.cols-1;

						if(ptBegin.y < 0)
							ptBegin.y = 0;
						if(ptEnd.y >= map.rows)
							ptEnd.y = map.rows-1;

						for(int i=ptBegin.x; i<ptEnd.x; ++i)
						{
							for(int j=ptBegin.y; j<ptEnd.y; ++j)
							{
								UASSERT(j < map.rows && i < map.cols);
								char & value = map.at<char>(j, i);
								float * info = mapInfo.ptr<float>(j, i);
								int nodeId = (int)info[0];
								if(value != -1)
								{
									if(kter->first > 0 && (kter->first <  nodeId || nodeId < 0))
									{
										// cannot rewrite on cells referred by more recent nodes
										continue;
									}
									if(nodeId>0)
									{
										std::map<int, std::pair<int, int> >::iterator eter = cellCount_.find(nodeId);
										UASSERT_MSG(eter != cellCount_.end(), uFormat("current pose=%d nodeId=%d", kter->first, nodeId).c_str());
										if(value == 0)
										{
											eter->second.first -= 1;
										}
										else if(value == 100)
										{
											eter->second.second -= 1;
										}
										if(kter->first < 0)
										{
											eter->second.first += 1;
										}
									}
								}
								if(kter->first > 0)
								{
									info[0] = (float)kter->first;
									info[1] = float(i) * cellSize_ + xMin;
									info[2] = float(j) * cellSize_ + yMin;
									info[3] = logOddsClampingMin_;
									cter->second.first+=1;
								}
								value = -2; // free space (footprint)
							}
						}
					}

					if(jter!=occupiedLocalMaps.end())
					{
						for(int i=0; i<jter->second.cols; ++i)
						{
							float * ptf = jter->second.ptr<float>(0,i);
							cv::Point2i pt((ptf[0]-xMin)/cellSize_, (ptf[1]-yMin)/cellSize_);
							UASSERT_MSG(pt.y>=0 && pt.y < map.rows && pt.x>=0 && pt.x < map.cols,
										uFormat("%d: pt=(%d,%d) map=%dx%d rawPt=(%f,%f) xMin=%f yMin=%f channels=%dvs%d",
												kter->first, pt.x, pt.y, map.cols, map.rows, ptf[0], ptf[1], xMin, yMin, jter->second.channels(), mapInfo.channels()-1).c_str());
							char & value = map.at<char>(pt.y, pt.x);
							if(value != -2)
							{
								float * info = mapInfo.ptr<float>(pt.y, pt.x);
								int nodeId = (int)info[0];
								if(value != -1)
								{
									if(kter->first > 0 && (kter->first <  nodeId || nodeId < 0))
									{
										// cannot rewrite on cells referred by more recent nodes
										continue;
									}
									if(nodeId>0)
									{
										std::map<int, std::pair<int, int> >::iterator eter = cellCount_.find(nodeId);
										UASSERT_MSG(eter != cellCount_.end(), uFormat("current pose=%d nodeId=%d", kter->first, nodeId).c_str());
										if(value == 0)
										{
											eter->second.first -= 1;
										}
										else if(value == 100)
										{
											eter->second.second -= 1;
										}
										if(kter->first < 0)
										{
											eter->second.second += 1;
										}
									}
								}
								if(kter->first > 0)
								{
									info[0] = (float)kter->first;
									info[1] = ptf[0];
									info[2] = ptf[1];
									cter->second.second+=1;
								}

								// update odds
								if(nodeId != kter->first || value!=100)
								{
									info[3] += logOddsHit_;
									if (info[3] < logOddsClampingMin_)
									{
										info[3] = logOddsClampingMin_;
									}
									if (info[3] > logOddsClampingMax_)
									{
										info[3] = logOddsClampingMax_;
									}
								}

								value = 100; // obstacles
							}
						}
					}
				}
			}

			if(footprintRadius_ >= cellSize_*1.5f)
			{
				for(int i=1; i<map.rows-1; ++i)
				{
					for(int j=1; j<map.cols-1; ++j)
					{
						char & value = map.at<char>(i, j);
						if(value == -2)
						{
							value = 0;
						}
					}
				}
			}

			map_ = map;
			mapInfo_ = mapInfo;
			minValues_[0] = xMin;
			minValues_[1] = yMin;

			// clean cellCount_
			for(std::map<int, std::pair<int, int> >::iterator iter= cellCount_.begin(); iter!=cellCount_.end();)
			{
				UASSERT(iter->second.first >= 0 && iter->second.second >= 0);
				if(iter->second.first == 0 && iter->second.second == 0)
				{
					cellCount_.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
		}
	}

	UDEBUG("Occupancy Grid update time = %f s", timer.ticks());
}

unsigned long OccupancyGrid::getMemoryUsed() const
{
	unsigned long memoryUsage = GlobalMap::getMemoryUsed();

	memoryUsage += map_.total() * map_.elemSize();
	memoryUsage += mapInfo_.total() * mapInfo_.elemSize();
	memoryUsage += cellCount_.size()*(sizeof(int)*3 + sizeof(std::pair<int, int>) + sizeof(std::map<int, std::pair<int, int> >::iterator)) + sizeof(std::map<int, std::pair<int, int> >);

	return memoryUsage;
}

}
