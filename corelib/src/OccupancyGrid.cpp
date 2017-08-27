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
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#include <pcl/io/pcd_io.h>

namespace rtabmap {

OccupancyGrid::OccupancyGrid(const ParametersMap & parameters) :
	parameters_(parameters),
	cloudDecimation_(Parameters::defaultGridDepthDecimation()),
	cloudMaxDepth_(Parameters::defaultGridDepthMax()),
	cloudMinDepth_(Parameters::defaultGridDepthMin()),
	//roiRatios_(Parameters::defaultGridDepthRoiRatios()), // initialized in parseParameters()
	footprintLength_(Parameters::defaultGridFootprintLength()),
	footprintWidth_(Parameters::defaultGridFootprintWidth()),
	footprintHeight_(Parameters::defaultGridFootprintHeight()),
	scanDecimation_(Parameters::defaultGridScanDecimation()),
	cellSize_(Parameters::defaultGridCellSize()),
	occupancyFromCloud_(Parameters::defaultGridFromDepth()),
	projMapFrame_(Parameters::defaultGridMapFrameProjection()),
	maxObstacleHeight_(Parameters::defaultGridMaxObstacleHeight()),
	normalKSearch_(Parameters::defaultGridNormalK()),
	maxGroundAngle_(Parameters::defaultGridMaxGroundAngle()*M_PI/180.0f),
	clusterRadius_(Parameters::defaultGridClusterRadius()),
	minClusterSize_(Parameters::defaultGridMinClusterSize()),
	flatObstaclesDetected_(Parameters::defaultGridFlatObstacleDetected()),
	minGroundHeight_(Parameters::defaultGridMinGroundHeight()),
	maxGroundHeight_(Parameters::defaultGridMaxGroundHeight()),
	normalsSegmentation_(Parameters::defaultGridNormalsSegmentation()),
	grid3D_(Parameters::defaultGrid3D()),
	groundIsObstacle_(Parameters::defaultGridGroundIsObstacle()),
	noiseFilteringRadius_(Parameters::defaultGridNoiseFilteringRadius()),
	noiseFilteringMinNeighbors_(Parameters::defaultGridNoiseFilteringMinNeighbors()),
	scan2dUnknownSpaceFilled_(Parameters::defaultGridScan2dUnknownSpaceFilled()),
	scan2dMaxUnknownSpaceFilledRange_(Parameters::defaultGridScan2dMaxFilledRange()),
	projRayTracing_(Parameters::defaultGridProjRayTracing()),
	fullUpdate_(Parameters::defaultGridGlobalFullUpdate()),
	minMapSize_(Parameters::defaultGridGlobalMinSize()),
	erode_(Parameters::defaultGridGlobalEroded()),
	footprintRadius_(Parameters::defaultGridGlobalFootprintRadius()),
	xMin_(0.0f),
	yMin_(0.0f)
{
	this->parseParameters(parameters);
}

void OccupancyGrid::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kGridFromDepth(), occupancyFromCloud_);
	Parameters::parse(parameters, Parameters::kGridDepthDecimation(), cloudDecimation_);
	if(cloudDecimation_ == 0)
	{
		cloudDecimation_ = 1;
	}
	Parameters::parse(parameters, Parameters::kGridDepthMin(), cloudMinDepth_);
	Parameters::parse(parameters, Parameters::kGridDepthMax(), cloudMaxDepth_);
	Parameters::parse(parameters, Parameters::kGridFootprintLength(), footprintLength_);
	Parameters::parse(parameters, Parameters::kGridFootprintWidth(), footprintWidth_);
	Parameters::parse(parameters, Parameters::kGridFootprintHeight(), footprintHeight_);
	Parameters::parse(parameters, Parameters::kGridScanDecimation(), scanDecimation_);
	float cellSize = cellSize_;
	if(Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize))
	{
		this->setCellSize(cellSize);
	}
	Parameters::parse(parameters, Parameters::kGridMapFrameProjection(), projMapFrame_);
	Parameters::parse(parameters, Parameters::kGridMaxObstacleHeight(), maxObstacleHeight_);
	Parameters::parse(parameters, Parameters::kGridMinGroundHeight(), minGroundHeight_);
	Parameters::parse(parameters, Parameters::kGridMaxGroundHeight(), maxGroundHeight_);
	Parameters::parse(parameters, Parameters::kGridNormalK(), normalKSearch_);
	if(Parameters::parse(parameters, Parameters::kGridMaxGroundAngle(), maxGroundAngle_))
	{
		maxGroundAngle_ *= M_PI/180.0f;
	}
	Parameters::parse(parameters, Parameters::kGridClusterRadius(), clusterRadius_);
	UASSERT_MSG(clusterRadius_ > 0.0f, uFormat("Param name is \"%s\"", Parameters::kGridClusterRadius().c_str()).c_str());
	Parameters::parse(parameters, Parameters::kGridMinClusterSize(), minClusterSize_);
	Parameters::parse(parameters, Parameters::kGridFlatObstacleDetected(), flatObstaclesDetected_);
	Parameters::parse(parameters, Parameters::kGridNormalsSegmentation(), normalsSegmentation_);
	Parameters::parse(parameters, Parameters::kGrid3D(), grid3D_);
	Parameters::parse(parameters, Parameters::kGridGroundIsObstacle(), groundIsObstacle_);
	Parameters::parse(parameters, Parameters::kGridNoiseFilteringRadius(), noiseFilteringRadius_);
	Parameters::parse(parameters, Parameters::kGridNoiseFilteringMinNeighbors(), noiseFilteringMinNeighbors_);
	Parameters::parse(parameters, Parameters::kGridScan2dUnknownSpaceFilled(), scan2dUnknownSpaceFilled_);
	Parameters::parse(parameters, Parameters::kGridScan2dMaxFilledRange(), scan2dMaxUnknownSpaceFilledRange_);
	Parameters::parse(parameters, Parameters::kGridProjRayTracing(), projRayTracing_);
	Parameters::parse(parameters, Parameters::kGridGlobalFullUpdate(), fullUpdate_);
	Parameters::parse(parameters, Parameters::kGridGlobalMinSize(), minMapSize_);
	Parameters::parse(parameters, Parameters::kGridGlobalEroded(), erode_);
	Parameters::parse(parameters, Parameters::kGridGlobalFootprintRadius(), footprintRadius_);

	UASSERT(minMapSize_ >= 0.0f);

	// convert ROI from string to vector
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kGridDepthRoiRatios())) != parameters.end())
	{
		std::list<std::string> strValues = uSplit(iter->second, ' ');
		if(strValues.size() != 4)
		{
			ULOGGER_ERROR("The number of values must be 4 (%s=\"%s\")", iter->first.c_str(), iter->second.c_str());
		}
		else
		{
			std::vector<float> tmpValues(4);
			unsigned int i=0;
			for(std::list<std::string>::iterator jter = strValues.begin(); jter!=strValues.end(); ++jter)
			{
				tmpValues[i] = uStr2Float(*jter);
				++i;
			}

			if(tmpValues[0] >= 0 && tmpValues[0] < 1 && tmpValues[0] < 1.0f-tmpValues[1] &&
				tmpValues[1] >= 0 && tmpValues[1] < 1 && tmpValues[1] < 1.0f-tmpValues[0] &&
				tmpValues[2] >= 0 && tmpValues[2] < 1 && tmpValues[2] < 1.0f-tmpValues[3] &&
				tmpValues[3] >= 0 && tmpValues[3] < 1 && tmpValues[3] < 1.0f-tmpValues[2])
			{
				roiRatios_ = tmpValues;
			}
			else
			{
				ULOGGER_ERROR("The roi ratios are not valid (%s=\"%s\")", iter->first.c_str(), iter->second.c_str());
			}
		}
	}

	if(maxGroundHeight_ == 0.0f && !normalsSegmentation_)
	{
		UWARN("\"%s\" should be not equal to 0 if not using normals "
				"segmentation approach. Setting it to cell size (%f).",
				Parameters::kGridMaxGroundHeight().c_str(), cellSize_);
		maxGroundHeight_ = cellSize_;
	}
	if(maxGroundHeight_ != 0.0f &&
		maxObstacleHeight_ != 0.0f &&
		maxObstacleHeight_ < maxGroundHeight_)
	{
		UWARN("\"%s\" should be lower than \"%s\", setting \"%s\" to 0 (disabled).",
				Parameters::kGridMaxGroundHeight().c_str(),
				Parameters::kGridMaxObstacleHeight().c_str(),
				Parameters::kGridMaxObstacleHeight().c_str());
		maxObstacleHeight_ = 0;
	}
	if(maxGroundHeight_ != 0.0f &&
		minGroundHeight_ != 0.0f &&
		maxGroundHeight_ < minGroundHeight_)
	{
		UWARN("\"%s\" should be lower than \"%s\", setting \"%s\" to 0 (disabled).",
				Parameters::kGridMinGroundHeight().c_str(),
				Parameters::kGridMaxGroundHeight().c_str(),
				Parameters::kGridMinGroundHeight().c_str());
		minGroundHeight_ = 0;
	}
}

void OccupancyGrid::setCellSize(float cellSize)
{
	UASSERT_MSG(cellSize > 0.0f, uFormat("Param name is \"%s\"", Parameters::kGridCellSize().c_str()).c_str());
	if(cellSize_ != cellSize)
	{
		if(!map_.empty())
		{
			UWARN("Grid cell size has changed, the map is cleared!");
		}
		this->clear();
		cellSize_ = cellSize;
	}
}

void OccupancyGrid::createLocalMap(
		const Signature & node,
		cv::Mat & ground,
		cv::Mat & obstacles,
		cv::Point3f & viewPoint) const
{
	UDEBUG("scan channels=%d, occupancyFromCloud_=%d normalsSegmentation_=%d grid3D_=%d",
			node.sensorData().laserScanRaw().empty()?0:node.sensorData().laserScanRaw().channels(), occupancyFromCloud_?1:0, normalsSegmentation_?1:0, grid3D_?1:0);

	if((node.sensorData().laserScanRaw().channels() == 2 || node.sensorData().laserScanRaw().channels() == 5) && !occupancyFromCloud_)
	{
		UDEBUG("2D laser scan");
		//2D
		viewPoint = cv::Point3f(
				node.sensorData().laserScanInfo().localTransform().x(),
				node.sensorData().laserScanInfo().localTransform().y(),
				node.sensorData().laserScanInfo().localTransform().z());

		util3d::occupancy2DFromLaserScan(
				util3d::transformLaserScan(node.sensorData().laserScanRaw(), node.sensorData().laserScanInfo().localTransform()),
				cv::Mat(),
				viewPoint,
				ground,
				obstacles,
				cellSize_,
				scan2dUnknownSpaceFilled_,
				node.sensorData().laserScanInfo().maxRange()>scan2dMaxUnknownSpaceFilledRange_?scan2dMaxUnknownSpaceFilledRange_:node.sensorData().laserScanInfo().maxRange());
	}
	else
	{
		// 3D
		pcl::IndicesPtr indices(new std::vector<int>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		if(!occupancyFromCloud_)
		{
			UDEBUG("3D laser scan");
			const Transform & t = node.sensorData().laserScanInfo().localTransform();
			cv::Mat scan = util3d::downsample(node.sensorData().laserScanRaw(), scanDecimation_);
			cloud = util3d::laserScanToPointCloudRGB(
					scan,
					t);

			// update viewpoint
			viewPoint = cv::Point3f(t.x(), t.y(), t.z());
		}
		else
		{
			UDEBUG("Depth image : decimation=%d max=%f min=%f",
					cloudDecimation_,
					cloudMaxDepth_,
					cloudMinDepth_);
			cloud = util3d::cloudRGBFromSensorData(
					node.sensorData(),
					cloudDecimation_,
					cloudMaxDepth_,
					cloudMinDepth_,
					indices.get(),
					parameters_,
					roiRatios_);

			// update viewpoint
			if(node.sensorData().cameraModels().size())
			{
				// average of all local transforms
				float sum = 0;
				for(unsigned int i=0; i<node.sensorData().cameraModels().size(); ++i)
				{
					const Transform & t = node.sensorData().cameraModels()[i].localTransform();
					if(!t.isNull())
					{
						viewPoint.x += t.x();
						viewPoint.y += t.y();
						viewPoint.z += t.z();
						sum += 1.0f;
					}
				}
				if(sum > 0.0f)
				{
					viewPoint.x /= sum;
					viewPoint.y /= sum;
					viewPoint.z /= sum;
				}
			}
			else
			{
				const Transform & t = node.sensorData().stereoCameraModel().localTransform();
				viewPoint = cv::Point3f(t.x(), t.y(), t.z());
			}
		}

		if(projMapFrame_)
		{
			//we should rotate viewPoint in /map frame
			float roll, pitch, yaw;
			node.getPose().getEulerAngles(roll, pitch, yaw);
			Transform viewpointRotated = Transform(0,0,0,roll,pitch,0) * Transform(viewPoint.x, viewPoint.y, viewPoint.z, 0,0,0);
			viewPoint.x = viewpointRotated.x();
			viewPoint.y = viewpointRotated.y();
			viewPoint.z = viewpointRotated.z();
		}

		if((cloud->is_dense && cloud->size()) ||
			(!cloud->is_dense && indices->size()))
		{
			pcl::IndicesPtr groundIndices(new std::vector<int>);
			pcl::IndicesPtr obstaclesIndices(new std::vector<int>);
			cloud = this->segmentCloud<pcl::PointXYZRGB>(
					cloud,
					indices,
					node.getPose(),
					viewPoint,
					groundIndices,
					obstaclesIndices);

			if(!groundIndices->empty() || !obstaclesIndices->empty())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

				if(groundIndices->size())
				{
					pcl::copyPointCloud(*cloud, *groundIndices, *groundCloud);
				}

				if(obstaclesIndices->size())
				{
					pcl::copyPointCloud(*cloud, *obstaclesIndices, *obstaclesCloud);
				}

				if(grid3D_)
				{
					UDEBUG("");
					if(groundIsObstacle_)
					{
						*obstaclesCloud += *groundCloud;
						groundCloud->clear();
					}

					// transform back in base frame
					float roll, pitch, yaw;
					node.getPose().getEulerAngles(roll, pitch, yaw);
					Transform tinv = Transform(0,0, projMapFrame_?node.getPose().z():0, roll, pitch, 0).inverse();
					ground = util3d::laserScanFromPointCloud(*groundCloud, tinv);
					obstacles = util3d::laserScanFromPointCloud(*obstaclesCloud, tinv);
				}
				else
				{
					UDEBUG("groundCloud=%d, obstaclesCloud=%d", (int)groundCloud->size(), (int)obstaclesCloud->size());
					// projection on the xy plane
					util3d::occupancy2DFromGroundObstacles<pcl::PointXYZRGB>(
							groundCloud,
							obstaclesCloud,
							ground,
							obstacles,
							cellSize_);

					if(projRayTracing_)
					{
						cv::Mat laserScan = obstacles;
						cv::Mat laserScanNoHit = ground;
						obstacles = cv::Mat();
						ground = cv::Mat();
						util3d::occupancy2DFromLaserScan(
								laserScan,
								laserScanNoHit,
								viewPoint,
								ground,
								obstacles,
								cellSize_,
								false, // don't fill unknown space
								0);
					}
				}
			}
		}
	}
	UDEBUG("ground=%d obstacles=%d channels=%d", ground.cols, obstacles.cols, ground.cols?ground.channels():obstacles.channels());
}

void OccupancyGrid::clear()
{
	cache_.clear();
	map_ = cv::Mat();
	mapInfo_ = cv::Mat();
	cellCount_.clear();
	xMin_ = 0.0f;
	yMin_ = 0.0f;
	addedNodes_.clear();
}

const cv::Mat OccupancyGrid::getMap(float & xMin, float & yMin) const
{
	xMin = xMin_;
	yMin = yMin_;
	if(erode_ && !map_.empty())
	{
		return util3d::erodeMap(map_);
	}
	return map_;
}

void OccupancyGrid::addToCache(
		int nodeId,
		const cv::Mat & ground,
		const cv::Mat & obstacles)
{
	UDEBUG("nodeId=%d", nodeId);
	uInsert(cache_, std::make_pair(nodeId, std::make_pair(ground, obstacles)));
}

void OccupancyGrid::update(const std::map<int, Transform> & posesIn)
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
	bool graphOptimized = false; // If a loop closure happened (e.g., poses are modified)
	bool graphChanged = addedNodes_.size()>0; // If the new map doesn't have any node from the previous map
	std::map<int, Transform> transforms;
	for(std::map<int, Transform>::iterator iter=addedNodes_.begin(); iter!=addedNodes_.end(); ++iter)
	{
		std::map<int, Transform>::const_iterator jter = posesIn.find(iter->first);
		if(jter != posesIn.end())
		{
			graphChanged = false;

			UASSERT(!iter->second.isNull() && !jter->second.isNull());
			Transform t = Transform::getIdentity();
			if(iter->second.getDistanceSquared(jter->second) > 0.0001)
			{
				t = jter->second * iter->second.inverse();
				graphOptimized = true;
			}
			transforms.insert(std::make_pair(jter->first, t));

			float x = jter->second.x();
			float y  =jter->second.y();
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
		else
		{
			UDEBUG("Updated pose for node %d is not found, some points may not be copied if graph has changed.", iter->first);
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

		if(!fullUpdate_ && !graphChanged && !map_.empty()) // incremental, just move cells
		{
			// 1) recreate all local maps
			UASSERT(map_.cols == mapInfo_.cols &&
					map_.rows == mapInfo_.rows);
			std::map<int, std::pair<int, int> > tmpIndices;
			for(std::map<int, std::pair<int, int> >::iterator iter=cellCount_.begin(); iter!=cellCount_.end(); ++iter)
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
			for(int y=1; y<map_.rows-1; ++y)
			{
				for(int x=1; x<map_.cols-1; ++x)
				{
					float * info = mapInfo_.ptr<float>(y,x);
					int nodeId = (int)info[0];
					if(nodeId > 0 && map_.at<char>(y,x) >= 0)
					{
						std::map<int, Transform>::iterator tter = transforms.find(nodeId);
						if(tter != transforms.end() && !uContains(cache_, nodeId))
						{
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
				}
			}

			UDEBUG("min (%f,%f) max(%f,%f)", minX, minY, maxX, maxY);
		}

		addedNodes_.clear();
		map_ = cv::Mat();
		mapInfo_ = cv::Mat();
		cellCount_.clear();
		xMin_ = 0.0f;
		yMin_ = 0.0f;
	}
	else if(!map_.empty())
	{
		// update
		minX=xMin_+margin+cellSize_/2.0f;
		minY=yMin_+margin+cellSize_/2.0f;
		maxX=xMin_+float(map_.cols)*cellSize_ - margin;
		maxY=yMin_+float(map_.rows)*cellSize_ - margin;
		undefinedSize = false;
	}

	bool incrementalGraphUpdate = graphOptimized && !fullUpdate_ && !graphChanged;

	std::list<std::pair<int, Transform> > poses;
	int lastId = addedNodes_.size()?addedNodes_.rbegin()->first:0;
	UDEBUG("Last id = %d", lastId);
	if(lastId >= 0)
	{
		for(std::map<int, Transform>::const_iterator iter=posesIn.upper_bound(lastId); iter!=posesIn.end(); ++iter)
		{
			poses.push_back(*iter);
		}
		// insert negative after
		for(std::map<int, Transform>::const_iterator iter=posesIn.begin(); iter!=posesIn.end(); ++iter)
		{
			if(iter->first < 0)
			{
				poses.push_back(*iter);
			}
			else
			{
				break;
			}
		}
	}
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
		for(std::list<std::pair<int, Transform> >::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if(uContains(cache_, iter->first))
			{
				const std::pair<cv::Mat, cv::Mat> & pair = cache_.at(iter->first);

				//ground
				if(pair.first.cols)
				{
					if(pair.first.rows > 1 && pair.first.cols == 1)
					{
						UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", pair.first.rows, pair.first.cols);
					}
					cv::Mat ground(1, pair.first.cols, CV_32FC2);
					for(int i=0; i<ground.cols; ++i)
					{
						const float * vi = pair.first.ptr<float>(0,i);
						float * vo = ground.ptr<float>(0,i);
						cv::Point3f vt;
						if(pair.first.channels() > 2)
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
					uInsert(emptyLocalMaps, std::make_pair(iter->first, ground));
				}

				//obstacles
				if(pair.second.cols)
				{
					if(pair.second.rows > 1 && pair.second.cols == 1)
					{
						UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", pair.second.rows, pair.second.cols);
					}
					cv::Mat obstacles(1, pair.second.cols, CV_32FC2);
					for(int i=0; i<obstacles.cols; ++i)
					{
						const float * vi = pair.second.ptr<float>(0,i);
						float * vo = obstacles.ptr<float>(0,i);
						cv::Point3f vt;
						if(pair.second.channels() > 2)
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
			UDEBUG("map min=(%f, %f) odlMin(%f,%f) max=(%f,%f)", xMin, yMin, xMin_, yMin_, xMax, yMax);
			cv::Size newMapSize((xMax - xMin) / cellSize_+0.5f, (yMax - yMin) / cellSize_+0.5f);
			if(map_.empty())
			{
				UDEBUG("Map empty!");
				map = cv::Mat::ones(newMapSize, CV_8S)*-1;
				mapInfo = cv::Mat::zeros(newMapSize, CV_32FC3);
			}
			else
			{
				if(xMin == xMin_ && yMin == yMin_ &&
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
					UASSERT_MSG(xMin <= xMin_+cellSize_/2, uFormat("xMin=%f, xMin_=%f, cellSize_=%f", xMin, xMin_, cellSize_).c_str());
					UASSERT_MSG(yMin <= yMin_+cellSize_/2, uFormat("yMin=%f, yMin_=%f, cellSize_=%f", yMin, yMin_, cellSize_).c_str());
					UASSERT_MSG(xMax >= xMin_+float(map_.cols)*cellSize_ - cellSize_/2, uFormat("xMin=%f, xMin_=%f, cols=%d cellSize_=%f", xMin, xMin_, map_.cols, cellSize_).c_str());
					UASSERT_MSG(yMax >= yMin_+float(map_.rows)*cellSize_ - cellSize_/2, uFormat("yMin=%f, yMin_=%f, cols=%d cellSize_=%f", yMin, yMin_, map_.rows, cellSize_).c_str());

					UDEBUG("Copy map");
					// copy the old map in the new map
					// make sure the translation is cellSize
					int deltaX = 0;
					if(xMin < xMin_)
					{
						deltaX = (xMin_ - xMin) / cellSize_ + 1.0f;
						xMin = xMin_-float(deltaX)*cellSize_;
					}
					int deltaY = 0;
					if(yMin < yMin_)
					{
						deltaY = (yMin_ - yMin) / cellSize_ + 1.0f;
						yMin = yMin_-float(deltaY)*cellSize_;
					}
					UDEBUG("deltaX=%d, deltaY=%d", deltaX, deltaY);
					newMapSize.width = (xMax - xMin) / cellSize_+0.5f;
					newMapSize.height = (yMax - yMin) / cellSize_+0.5f;
					UDEBUG("%d/%d -> %d/%d", map_.cols, map_.rows, newMapSize.width, newMapSize.height);
					UASSERT(newMapSize.width >= map_.cols && newMapSize.height >= map_.rows);
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
				if(kter->first > 0)
				{
					uInsert(addedNodes_, *kter);
				}
				std::map<int, cv::Mat >::iterator iter = emptyLocalMaps.find(kter->first);
				std::map<int, cv::Mat >::iterator jter = occupiedLocalMaps.find(kter->first);
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
						UASSERT_MSG(pt.y < map.rows && pt.x < map.cols,
								uFormat("%d: pt=(%d,%d) map=%dx%d rawPt=(%f,%f) xMin=%f yMin=%f channels=%dvs%d",
										kter->first, pt.x, pt.y, map.cols, map.rows, ptf[0], ptf[1], xMin, yMin, iter->second.channels(), mapInfo.channels()-1).c_str());
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
						UASSERT_MSG(pt.y < map.rows && pt.x < map.cols,
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
							value = 100; // obstacles
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
			xMin_ = xMin;
			yMin_ = yMin;

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

	if(!fullUpdate_)
	{
		cache_.clear();
	}

	UDEBUG("Occupancy Grid update time = %f s", timer.ticks());
}

}
