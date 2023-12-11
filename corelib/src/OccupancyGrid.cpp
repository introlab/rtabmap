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

#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#include <pcl/io/pcd_io.h>

namespace rtabmap {

OccupancyGrid::OccupancyGrid(const ParametersMap & parameters) :
	ProbabilisticMap(parameters),
	minMapSize_(Parameters::defaultGridGlobalMinSize()),
	erode_(Parameters::defaultGridGlobalEroded()),
	footprintRadius_(Parameters::defaultGridGlobalFootprintRadius()),
	cloudAssembling_(false),
	assembledGround_(new pcl::PointCloud<pcl::PointXYZRGB>),
	assembledObstacles_(new pcl::PointCloud<pcl::PointXYZRGB>),
	assembledEmptyCells_(new pcl::PointCloud<pcl::PointXYZRGB>)
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
					info[3] = probClampingMin_;
				}
				else if(value == 100)
				{
					info[3] = probClampingMax_;
				}
			}
		}
		minValues_[0] = xMin;
		minValues_[1] = yMin;
		cellSize_ = cellSize;
		addedNodes_.insert(poses.lower_bound(1), poses.end());
	}
}

void OccupancyGrid::setCloudAssembling(bool enabled)
{
	cloudAssembling_ = enabled;
	if(!cloudAssembling_)
	{
		assembledGround_->clear();
		assembledObstacles_->clear();
	}
}

void OccupancyGrid::clear(bool keepCache)
{
	map_ = cv::Mat();
	mapInfo_ = cv::Mat();
	cellCount_.clear();
	assembledGround_->clear();
	assembledObstacles_->clear();
	ProbabilisticMap::clear(keepCache);
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

bool OccupancyGrid::update(const std::map<int, Transform> & posesIn)
{
	UTimer timer;
	UDEBUG("Update (poses=%d addedNodes_=%d)", (int)posesIn.size(), (int)addedNodes_.size());

	float margin = cellSize_*10.0f+(footprintRadius_>cellSize_*1.5f?float(int(footprintRadius_/cellSize_)+1):0.0f)*cellSize_;

	float minX=-minMapSize_/2.0f;
	float minY=-minMapSize_/2.0f;
	float maxX=minMapSize_/2.0f;
	float maxY=minMapSize_/2.0f;
	bool undefinedSize = minMapSize_ == 0.0f;
	std::map<int, cv::Mat> emptyLocalMaps;
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

	bool assembledGroundUpdated = false;
	bool assembledObstaclesUpdated = false;
	bool assembledEmptyCellsUpdated = false;

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

		if(cloudAssembling_)
		{
			assembledGround_->clear();
			assembledObstacles_->clear();
		}

		if(!fullUpdate_ && !graphChanged && !map_.empty()) // incremental, just move cells
		{
			// 1) recreate all local maps
			UASSERT(map_.cols == mapInfo_.cols &&
					map_.rows == mapInfo_.rows);
			std::map<int, std::pair<int, int> > tmpIndices;
			for(std::map<int, std::pair<int, int> >::iterator iter=cellCount_.begin(); iter!=cellCount_.end(); ++iter)
			{
				if(!uContains(cache_, iter->first) && transforms.find(iter->first) != transforms.end())
				{
					if(iter->second.first)
					{
						emptyLocalMaps.insert(std::make_pair( iter->first, cv::Mat(1, iter->second.first, CV_32FC2)));
					}
					if(iter->second.second)
					{
						occupiedLocalMaps.insert(std::make_pair( iter->first, cv::Mat(1, iter->second.second, CV_32FC2)));
					}
					tmpIndices.insert(std::make_pair(iter->first, std::make_pair(0,0)));
				}
			}
			for(int y=0; y<map_.rows; ++y)
			{
				for(int x=0; x<map_.cols; ++x)
				{
					float * info = mapInfo_.ptr<float>(y,x);
					int nodeId = (int)info[0];
					if(nodeId > 0 && map_.at<char>(y,x) >= 0)
					{
						if(tmpIndices.find(nodeId)!=tmpIndices.end())
						{
							std::map<int, Transform>::iterator tter = transforms.find(nodeId);
							UASSERT(tter != transforms.end());

							cv::Point3f pt(info[1], info[2], 0.0f);
							pt = util3d::transformPoint(pt, tter->second);

							if(minX > pt.x)
								minX = pt.x;
							else if(maxX < pt.x)
								maxX = pt.x;

							if(minY > pt.y)
								minY = pt.y;
							else if(maxY < pt.y)
								maxY = pt.y;

							std::map<int, std::pair<int, int> >::iterator jter = tmpIndices.find(nodeId);
							if(map_.at<char>(y, x) == 0)
							{
								// ground
								std::map<int, cv::Mat>::iterator iter = emptyLocalMaps.find(nodeId);
								UASSERT(iter != emptyLocalMaps.end());
								UASSERT(jter->second.first < iter->second.cols);
								float * ptf = iter->second.ptr<float>(0,jter->second.first++);
								ptf[0] = pt.x;
								ptf[1] = pt.y;
							}
							else
							{
								// obstacle
								std::map<int, cv::Mat>::iterator iter = occupiedLocalMaps.find(nodeId);
								UASSERT(iter != occupiedLocalMaps.end());
								UASSERT(iter!=occupiedLocalMaps.end());
								UASSERT(jter->second.second < iter->second.cols);
								float * ptf = iter->second.ptr<float>(0,jter->second.second++);
								ptf[0] = pt.x;
								ptf[1] = pt.y;
							}
						}
					}
					else if(nodeId > 0)
					{
						UERROR("Cell referred b node %d is unknown!?", nodeId);
					}
				}
			}

			//verify if all cells were added
			for(std::map<int, std::pair<int, int> >::iterator iter=tmpIndices.begin(); iter!=tmpIndices.end(); ++iter)
			{
				std::map<int, cv::Mat>::iterator jter = emptyLocalMaps.find(iter->first);
				UASSERT_MSG((iter->second.first == 0 && (jter==emptyLocalMaps.end() || jter->second.empty())) ||
						(iter->second.first != 0 && jter!=emptyLocalMaps.end() && jter->second.cols == iter->second.first),
						uFormat("iter->second.first=%d jter->second.cols=%d", iter->second.first, jter!=emptyLocalMaps.end()?jter->second.cols:-1).c_str());
				jter = occupiedLocalMaps.find(iter->first);
				UASSERT_MSG((iter->second.second == 0 && (jter==occupiedLocalMaps.end() || jter->second.empty())) ||
						(iter->second.second != 0 && jter!=occupiedLocalMaps.end() && jter->second.cols == iter->second.second),
						uFormat("iter->second.first=%d jter->second.cols=%d", iter->second.first, jter!=emptyLocalMaps.end()?jter->second.cols:-1).c_str());
			}

			UDEBUG("min (%f,%f) max(%f,%f)", minX, minY, maxX, maxY);
		}

		// clear all but keep cache
		clear(true);
	}
	else if(!map_.empty())
	{
		// update
		minX=minValues_[0]+margin+cellSize_/2.0f;
		minY=minValues_[1]+margin+cellSize_/2.0f;
		maxX=minValues_[0]+float(map_.cols)*cellSize_ - margin;
		maxY=minValues_[1]+float(map_.rows)*cellSize_ - margin;
		undefinedSize = false;
	}

	bool incrementalGraphUpdate = graphOptimized && !fullUpdate_ && !graphChanged;

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
					cv::Mat ground;
					if(pair.first.first.cols || pair.second.cols)
					{
						ground = cv::Mat(1, pair.first.first.cols+pair.second.cols, CV_32FC2);
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
							float * vo = ground.ptr<float>(0,i);
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
							if(minX > vo[0])
								minX = vo[0];
							else if(maxX < vo[0])
								maxX = vo[0];

							if(minY > vo[1])
								minY = vo[1];
							else if(maxY < vo[1])
								maxY = vo[1];
						}

						if(cloudAssembling_)
						{
							*assembledGround_ += *util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(pair.first.first), iter->second, 0, 255, 0);
							assembledGroundUpdated = true;
						}
					}

					//empty
					if(pair.second.cols)
					{
						if(pair.second.rows > 1 && pair.second.cols == 1)
						{
							UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", pair.second.rows, pair.second.cols);
						}
						for(int i=0; i<pair.second.cols; ++i)
						{
							const float * vi = pair.second.ptr<float>(0,i);
							float * vo = ground.ptr<float>(0,i+pair.first.first.cols);
							cv::Point3f vt;
							if(pair.second.channels() != 2 && pair.second.channels() != 5)
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

						if(cloudAssembling_)
						{
							*assembledEmptyCells_ += *util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(pair.second), iter->second, 0, 255, 0);
							assembledEmptyCellsUpdated = true;
						}
					}
					uInsert(emptyLocalMaps, std::make_pair(iter->first, ground));

					//obstacles
					if(pair.first.second.cols)
					{
						if(pair.first.second.rows > 1 && pair.first.second.cols == 1)
						{
							UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", pair.first.second.rows, pair.first.second.cols);
						}
						cv::Mat obstacles(1, pair.first.second.cols, CV_32FC2);
						for(int i=0; i<obstacles.cols; ++i)
						{
							const float * vi = pair.first.second.ptr<float>(0,i);
							float * vo = obstacles.ptr<float>(0,i);
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

						if(cloudAssembling_)
						{
							*assembledObstacles_ += *util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(pair.first.second), iter->second, 255, 0, 0);
							assembledObstaclesUpdated = true;
						}
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
				if(poses.size())
				{
					UDEBUG("first pose= %d last pose=%d", poses.begin()->first, poses.rbegin()->first);
				}
				for(std::list<std::pair<int, Transform> >::const_iterator kter = poses.begin(); kter!=poses.end(); ++kter)
				{
					std::map<int, cv::Mat >::iterator iter = emptyLocalMaps.find(kter->first);
					std::map<int, cv::Mat >::iterator jter = occupiedLocalMaps.find(kter->first);
					if(iter != emptyLocalMaps.end() || jter!=occupiedLocalMaps.end())
					{
						if(kter->first > 0)
						{
							uInsert(addedNodes_, *kter);
						}
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
										uFormat("%d: pt=(%d,%d) map=%dx%d rawPt=(%f,%f) xMin=%f yMin=%f channels=%dvs%d (graph modified=%d)",
												kter->first, pt.x, pt.y, map.cols, map.rows, ptf[0], ptf[1], xMin, yMin, iter->second.channels(), mapInfo.channels()-1, (graphOptimized || graphChanged)?1:0).c_str());
								char & value = map.at<char>(pt.y, pt.x);
								if(value != -2 && (!incrementalGraphUpdate || value==-1))
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
										info[3] += probMiss_;
										if (info[3] < probClampingMin_)
										{
											info[3] = probClampingMin_;
										}
										if (info[3] > probClampingMax_)
										{
											info[3] = probClampingMax_;
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
										info[3] = probClampingMin_;
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
											uFormat("%d: pt=(%d,%d) map=%dx%d rawPt=(%f,%f) xMin=%f yMin=%f channels=%dvs%d (graph modified=%d)",
													kter->first, pt.x, pt.y, map.cols, map.rows, ptf[0], ptf[1], xMin, yMin, jter->second.channels(), mapInfo.channels()-1, (graphOptimized || graphChanged)?1:0).c_str());
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
										info[3] += probHit_;
										if (info[3] < probClampingMin_)
										{
											info[3] = probClampingMin_;
										}
										if (info[3] > probClampingMax_)
										{
											info[3] = probClampingMax_;
										}
									}

									value = 100; // obstacles
								}
							}
						}
					}
				}

				if(footprintRadius_ >= cellSize_*1.5f || incrementalGraphUpdate)
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

							if(incrementalGraphUpdate && value == -1)
							{
								float * info = mapInfo.ptr<float>(i, j);

								// fill obstacle
								if(map.at<char>(i+1, j) == 100 && map.at<char>(i-1, j) == 100)
								{
									value = 100;
									// associate with the nearest pose
									if(mapInfo.ptr<float>(i+1, j)[0]>0.0f)
									{
										info[0] = mapInfo.ptr<float>(i+1, j)[0];
										info[1] = float(j) * cellSize_ + xMin;
										info[2] = float(i) * cellSize_ + yMin;
										std::map<int, std::pair<int, int> >::iterator cter = cellCount_.find(int(info[0]));
										UASSERT(cter!=cellCount_.end());
										cter->second.second+=1;
									}
									else if(mapInfo.ptr<float>(i-1, j)[0]>0.0f)
									{
										info[0] = mapInfo.ptr<float>(i-1, j)[0];
										info[1] = float(j) * cellSize_ + xMin;
										info[2] = float(i) * cellSize_ + yMin;
										std::map<int, std::pair<int, int> >::iterator cter = cellCount_.find(int(info[0]));
										UASSERT(cter!=cellCount_.end());
										cter->second.second+=1;
									}
								}
								else if(map.at<char>(i, j+1) == 100 && map.at<char>(i, j-1) == 100)
								{
									value = 100;
									// associate with the nearest pose
									if(mapInfo.ptr<float>(i, j+1)[0]>0.0f)
									{
										info[0] = mapInfo.ptr<float>(i, j+1)[0];
										info[1] = float(j) * cellSize_ + xMin;
										info[2] = float(i) * cellSize_ + yMin;
										std::map<int, std::pair<int, int> >::iterator cter = cellCount_.find(int(info[0]));
										UASSERT(cter!=cellCount_.end());
										cter->second.second+=1;
									}
									else if(mapInfo.ptr<float>(i, j-1)[0]>0.0f)
									{
										info[0] = mapInfo.ptr<float>(i, j-1)[0];
										info[1] = float(j) * cellSize_ + xMin;
										info[2] = float(i) * cellSize_ + yMin;
										std::map<int, std::pair<int, int> >::iterator cter = cellCount_.find(int(info[0]));
										UASSERT(cter!=cellCount_.end());
										cter->second.second+=1;
									}
								}
								else
								{
									// fill empty
									char sum =  (map.at<char>(i+1, j) == 0?1:0) +
												(map.at<char>(i-1, j) == 0?1:0) +
												(map.at<char>(i, j+1) == 0?1:0) +
												(map.at<char>(i, j-1) == 0?1:0);
									if(sum >=3)
									{
										value = 0;
										// associate with the nearest pose, only check two cases (as 3 are required)
										if(map.at<char>(i+1, j) != -1 && mapInfo.ptr<float>(i+1, j)[0]>0.0f)
										{
											info[0] = mapInfo.ptr<float>(i+1, j)[0];
											info[1] = float(j) * cellSize_ + xMin;
											info[2] = float(i) * cellSize_ + yMin;
											std::map<int, std::pair<int, int> >::iterator cter = cellCount_.find(int(info[0]));
											UASSERT(cter!=cellCount_.end());
											cter->second.first+=1;
										}
										else if(map.at<char>(i-1, j) != -1 && mapInfo.ptr<float>(i-1, j)[0]>0.0f)
										{
											info[0] = mapInfo.ptr<float>(i-1, j)[0];
											info[1] = float(j) * cellSize_ + xMin;
											info[2] = float(i) * cellSize_ + yMin;
											std::map<int, std::pair<int, int> >::iterator cter = cellCount_.find(int(info[0]));
											UASSERT(cter!=cellCount_.end());
											cter->second.first+=1;
										}
									}
								}
							}

							//float * info = mapInfo.ptr<float>(i,j);
							//if(info[0] > 0)
							//{
							//	cloud->at(oi).x = info[1];
							//	cloud->at(oi).y = info[2];
							//	oi++;
							//}
						}
					}
				}
				//if(graphChanged)
				//{
				//	cloud->resize(oi);
				//	pcl::io::savePCDFileBinary("mapInfo.pcd", *cloud);
				//	UWARN("Saved mapInfo.pcd");
				//}

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

		if(cloudAssembling_)
		{
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
		}
	}

	if(!fullUpdate_ && !cloudAssembling_)
	{
		cache_.clear();
	}
	else
	{
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
	}

	bool updated = !poses.empty() || graphOptimized || graphChanged;
	UDEBUG("Occupancy Grid update time = %f s (updated=%s)", timer.ticks(), updated?"true":"false");
	return updated;
}

unsigned long OccupancyGrid::getMemoryUsed() const
{
	unsigned long memoryUsage = ProbabilisticMap::getMemoryUsed();

	memoryUsage += map_.total() * map_.elemSize();
	memoryUsage += mapInfo_.total() * mapInfo_.elemSize();
	memoryUsage += cellCount_.size()*(sizeof(int)*3 + sizeof(std::pair<int, int>) + sizeof(std::map<int, std::pair<int, int> >::iterator)) + sizeof(std::map<int, std::pair<int, int> >);

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
		memoryUsage += assembledEmptyCells_->points.size() * sizeof(pcl::PointXYZRGB);
	}
	return memoryUsage;
}

}
