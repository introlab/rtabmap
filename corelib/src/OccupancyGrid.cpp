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

#ifdef RTABMAP_OCTOMAP
#include <rtabmap/core/OctoMap.h>
#endif

#include <pcl/io/pcd_io.h>

namespace rtabmap {

OccupancyGrid::OccupancyGrid(const ParametersMap & parameters) :
	parameters_(parameters),
	cloudDecimation_(Parameters::defaultGridDepthDecimation()),
	cloudMaxDepth_(Parameters::defaultGridRangeMax()),
	cloudMinDepth_(Parameters::defaultGridRangeMin()),
	//roiRatios_(Parameters::defaultGridDepthRoiRatios()), // initialized in parseParameters()
	footprintLength_(Parameters::defaultGridFootprintLength()),
	footprintWidth_(Parameters::defaultGridFootprintWidth()),
	footprintHeight_(Parameters::defaultGridFootprintHeight()),
	scanDecimation_(Parameters::defaultGridScanDecimation()),
	cellSize_(Parameters::defaultGridCellSize()),
	preVoxelFiltering_(Parameters::defaultGridPreVoxelFiltering()),
	occupancySensor_(Parameters::defaultGridSensor()),
	projMapFrame_(Parameters::defaultGridMapFrameProjection()),
	maxObstacleHeight_(Parameters::defaultGridMaxObstacleHeight()),
	normalKSearch_(Parameters::defaultGridNormalK()),
	groundNormalsUp_(Parameters::defaultIcpPointToPlaneGroundNormalsUp()),
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
	rayTracing_(Parameters::defaultGridRayTracing()),
	fullUpdate_(Parameters::defaultGridGlobalFullUpdate()),
	minMapSize_(Parameters::defaultGridGlobalMinSize()),
	erode_(Parameters::defaultGridGlobalEroded()),
	footprintRadius_(Parameters::defaultGridGlobalFootprintRadius()),
	updateError_(Parameters::defaultGridGlobalUpdateError()),
	occupancyThr_(Parameters::defaultGridGlobalOccupancyThr()),
	probHit_(logodds(Parameters::defaultGridGlobalProbHit())),
	probMiss_(logodds(Parameters::defaultGridGlobalProbMiss())),
	probClampingMin_(logodds(Parameters::defaultGridGlobalProbClampingMin())),
	probClampingMax_(logodds(Parameters::defaultGridGlobalProbClampingMax())),
	xMin_(0.0f),
	yMin_(0.0f),
	cloudAssembling_(false),
	assembledGround_(new pcl::PointCloud<pcl::PointXYZRGB>),
	assembledObstacles_(new pcl::PointCloud<pcl::PointXYZRGB>),
	assembledEmptyCells_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	this->parseParameters(parameters);
}

void OccupancyGrid::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kGridSensor(), occupancySensor_);
	Parameters::parse(parameters, Parameters::kGridDepthDecimation(), cloudDecimation_);
	if(cloudDecimation_ == 0)
	{
		cloudDecimation_ = 1;
	}
	Parameters::parse(parameters, Parameters::kGridRangeMin(), cloudMinDepth_);
	Parameters::parse(parameters, Parameters::kGridRangeMax(), cloudMaxDepth_);
	Parameters::parse(parameters, Parameters::kGridFootprintLength(), footprintLength_);
	Parameters::parse(parameters, Parameters::kGridFootprintWidth(), footprintWidth_);
	Parameters::parse(parameters, Parameters::kGridFootprintHeight(), footprintHeight_);
	Parameters::parse(parameters, Parameters::kGridScanDecimation(), scanDecimation_);
	float cellSize = cellSize_;
	if(Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize))
	{
		this->setCellSize(cellSize);
	}

	Parameters::parse(parameters, Parameters::kGridPreVoxelFiltering(), preVoxelFiltering_);
	Parameters::parse(parameters, Parameters::kGridMapFrameProjection(), projMapFrame_);
	Parameters::parse(parameters, Parameters::kGridMaxObstacleHeight(), maxObstacleHeight_);
	Parameters::parse(parameters, Parameters::kGridMinGroundHeight(), minGroundHeight_);
	Parameters::parse(parameters, Parameters::kGridMaxGroundHeight(), maxGroundHeight_);
	Parameters::parse(parameters, Parameters::kGridNormalK(), normalKSearch_);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneGroundNormalsUp(), groundNormalsUp_);
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
	Parameters::parse(parameters, Parameters::kGridRayTracing(), rayTracing_);
	Parameters::parse(parameters, Parameters::kGridGlobalFullUpdate(), fullUpdate_);
	Parameters::parse(parameters, Parameters::kGridGlobalMinSize(), minMapSize_);
	Parameters::parse(parameters, Parameters::kGridGlobalEroded(), erode_);
	Parameters::parse(parameters, Parameters::kGridGlobalFootprintRadius(), footprintRadius_);
	Parameters::parse(parameters, Parameters::kGridGlobalUpdateError(), updateError_);

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
		xMin_ = xMin;
		yMin_ = yMin;
		cellSize_ = cellSize;
		addedNodes_.insert(poses.lower_bound(1), poses.end());
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

void OccupancyGrid::setCloudAssembling(bool enabled)
{
	cloudAssembling_ = enabled;
	if(!cloudAssembling_)
	{
		assembledGround_->clear();
		assembledObstacles_->clear();
	}
}

void OccupancyGrid::createLocalMap(
		const Signature & node,
		cv::Mat & groundCells,
		cv::Mat & obstacleCells,
		cv::Mat & emptyCells,
		cv::Point3f & viewPoint)
{
	UDEBUG("scan format=%s, occupancySensor_=%d normalsSegmentation_=%d grid3D_=%d",
			node.sensorData().laserScanRaw().isEmpty()?"NA":node.sensorData().laserScanRaw().formatName().c_str(), occupancySensor_, normalsSegmentation_?1:0, grid3D_?1:0);

	if((node.sensorData().laserScanRaw().is2d()) && occupancySensor_ == 0)
	{
		UDEBUG("2D laser scan");
		//2D
		viewPoint = cv::Point3f(
				node.sensorData().laserScanRaw().localTransform().x(),
				node.sensorData().laserScanRaw().localTransform().y(),
				node.sensorData().laserScanRaw().localTransform().z());

		LaserScan scan = node.sensorData().laserScanRaw();
		if(cloudMinDepth_ > 0.0f)
		{
			scan = util3d::rangeFiltering(scan, cloudMinDepth_, 0.0f);
		}

		float maxRange = cloudMaxDepth_;
		if(cloudMaxDepth_>0.0f && node.sensorData().laserScanRaw().rangeMax()>0.0f)
		{
			maxRange = cloudMaxDepth_ < node.sensorData().laserScanRaw().rangeMax()?cloudMaxDepth_:node.sensorData().laserScanRaw().rangeMax();
		}
		else if(scan2dUnknownSpaceFilled_ && node.sensorData().laserScanRaw().rangeMax()>0.0f)
		{
			maxRange = node.sensorData().laserScanRaw().rangeMax();
		}
		util3d::occupancy2DFromLaserScan(
				util3d::transformLaserScan(scan, node.sensorData().laserScanRaw().localTransform()).data(),
				cv::Mat(),
				viewPoint,
				emptyCells,
				obstacleCells,
				cellSize_,
				scan2dUnknownSpaceFilled_,
				maxRange);

		UDEBUG("ground=%d obstacles=%d channels=%d", emptyCells.cols, obstacleCells.cols, obstacleCells.cols?obstacleCells.channels():emptyCells.channels());
	}
	else
	{
		// 3D
		if(occupancySensor_ == 0 || occupancySensor_ == 2)
		{
			if(!node.sensorData().laserScanRaw().isEmpty())
			{
				UDEBUG("3D laser scan");
				const Transform & t = node.sensorData().laserScanRaw().localTransform();
				LaserScan scan = util3d::downsample(node.sensorData().laserScanRaw(), scanDecimation_);
#ifdef RTABMAP_OCTOMAP
				// If ray tracing enabled, clipping will be done in OctoMap or in occupancy2DFromLaserScan()
				float maxRange = rayTracing_?0.0f:cloudMaxDepth_;
#else
				// If ray tracing enabled, clipping will be done in occupancy2DFromLaserScan()
				float maxRange = !grid3D_ && rayTracing_?0.0f:cloudMaxDepth_;
#endif
				if(cloudMinDepth_ > 0.0f || maxRange > 0.0f)
				{
					scan = util3d::rangeFiltering(scan, cloudMinDepth_, maxRange);
				}

				// update viewpoint
				viewPoint = cv::Point3f(t.x(), t.y(), t.z());

				UDEBUG("scan format=%d", scan.format());

				bool normalSegmentationTmp = normalsSegmentation_;
				float minGroundHeightTmp = minGroundHeight_;
				float maxGroundHeightTmp = maxGroundHeight_;
				if(scan.is2d())
				{
					// if 2D, assume the whole scan is obstacle
					normalsSegmentation_ = false;
					minGroundHeight_ = std::numeric_limits<int>::min();
					maxGroundHeight_ = std::numeric_limits<int>::min()+100;
				}

				createLocalMap(scan, node.getPose(), groundCells, obstacleCells, emptyCells, viewPoint);

				if(scan.is2d())
				{
					// restore
					normalsSegmentation_ = normalSegmentationTmp;
					minGroundHeight_ = minGroundHeightTmp;
					maxGroundHeight_ = maxGroundHeightTmp;
				}
			}
			else
			{
				UWARN("Cannot create local map, scan is empty (node=%d, %s=0).", node.id(), Parameters::kGridSensor().c_str());
			}
		}

		if(occupancySensor_ >= 1)
		{
			pcl::IndicesPtr indices(new std::vector<int>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
			UDEBUG("Depth image : decimation=%d max=%f min=%f",
					cloudDecimation_,
					cloudMaxDepth_,
					cloudMinDepth_);
			cloud = util3d::cloudRGBFromSensorData(
					node.sensorData(),
					cloudDecimation_,
#ifdef RTABMAP_OCTOMAP
					// If ray tracing enabled, clipping will be done in OctoMap or in occupancy2DFromLaserScan()
					rayTracing_?0.0f:cloudMaxDepth_,
#else
					// If ray tracing enabled, clipping will be done in occupancy2DFromLaserScan()
					!grid3D_&&rayTracing_?0.0f:cloudMaxDepth_,
#endif
					cloudMinDepth_,
					indices.get(),
					parameters_,
					roiRatios_);

			// update viewpoint
			viewPoint = cv::Point3f(0,0,0);
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

			cv::Mat scanGroundCells;
			cv::Mat scanObstacleCells;
			cv::Mat scanEmptyCells;
			if(occupancySensor_ == 2)
			{
				// backup
				scanGroundCells = groundCells.clone();
				scanObstacleCells = obstacleCells.clone();
				scanEmptyCells = emptyCells.clone();
			}

			createLocalMap(LaserScan(util3d::laserScanFromPointCloud(*cloud, indices), 0, 0.0f), node.getPose(), groundCells, obstacleCells, emptyCells, viewPoint);

			if(occupancySensor_ == 2)
			{
				if(grid3D_)
				{
					// We should convert scans to 4 channels (XYZRGB) to be compatible
					scanGroundCells = util3d::laserScanFromPointCloud(*util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(scanGroundCells), Transform::getIdentity(), 255, 255, 255)).data();
					scanObstacleCells = util3d::laserScanFromPointCloud(*util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(scanObstacleCells), Transform::getIdentity(), 255, 255, 255)).data();
					scanEmptyCells = util3d::laserScanFromPointCloud(*util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(scanEmptyCells), Transform::getIdentity(), 255, 255, 255)).data();
				}

				UDEBUG("groundCells, depth: size=%d channels=%d vs scan: size=%d channels=%d", groundCells.cols, groundCells.channels(), scanGroundCells.cols, scanGroundCells.channels());
				UDEBUG("obstacleCells, depth: size=%d channels=%d vs scan: size=%d channels=%d", obstacleCells.cols, obstacleCells.channels(), scanObstacleCells.cols, scanObstacleCells.channels());
				UDEBUG("emptyCells, depth: size=%d channels=%d vs scan: size=%d channels=%d", emptyCells.cols, emptyCells.channels(), scanEmptyCells.cols, scanEmptyCells.channels());

				if(!groundCells.empty() && !scanGroundCells.empty())
					cv::hconcat(groundCells, scanGroundCells, groundCells);
				else if(!scanGroundCells.empty())
					groundCells = scanGroundCells;

				if(!obstacleCells.empty() && !scanObstacleCells.empty())
					cv::hconcat(obstacleCells, scanObstacleCells, obstacleCells);
				else if(!scanObstacleCells.empty())
					obstacleCells = scanObstacleCells;

				if(!emptyCells.empty() && !scanEmptyCells.empty())
					cv::hconcat(emptyCells, scanEmptyCells, emptyCells);
				else if(!scanEmptyCells.empty())
					emptyCells = scanEmptyCells;
			}
		}
	}
}

void OccupancyGrid::createLocalMap(
		const LaserScan & scan,
		const Transform & pose,
		cv::Mat & groundCells,
		cv::Mat & obstacleCells,
		cv::Mat & emptyCells,
		cv::Point3f & viewPointInOut) const
{
	if(projMapFrame_)
	{
		//we should rotate viewPoint in /map frame
		float roll, pitch, yaw;
		pose.getEulerAngles(roll, pitch, yaw);
		Transform viewpointRotated = Transform(0,0,0,roll,pitch,0) * Transform(viewPointInOut.x, viewPointInOut.y, viewPointInOut.z, 0,0,0);
		viewPointInOut.x = viewpointRotated.x();
		viewPointInOut.y = viewpointRotated.y();
		viewPointInOut.z = viewpointRotated.z();
	}

	if(scan.size())
	{
		pcl::IndicesPtr groundIndices(new std::vector<int>);
		pcl::IndicesPtr obstaclesIndices(new std::vector<int>);
		cv::Mat groundCloud;
		cv::Mat obstaclesCloud;

		if(scan.hasRGB() && scan.hasNormals())
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = util3d::laserScanToPointCloudRGBNormal(scan, scan.localTransform());
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudSegmented = segmentCloud<pcl::PointXYZRGBNormal>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			if(grid3D_)
			{
				groundCloud = util3d::laserScanFromPointCloud(*cloudSegmented, groundIndices).data();
				obstaclesCloud = util3d::laserScanFromPointCloud(*cloudSegmented, obstaclesIndices).data();
			}
			else
			{
				util3d::occupancy2DFromGroundObstacles<pcl::PointXYZRGBNormal>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
			}
		}
		else if(scan.hasRGB())
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::laserScanToPointCloudRGB(scan, scan.localTransform());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented = segmentCloud<pcl::PointXYZRGB>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			if(grid3D_)
			{
				groundCloud = util3d::laserScanFromPointCloud(*cloudSegmented, groundIndices).data();
				obstaclesCloud = util3d::laserScanFromPointCloud(*cloudSegmented, obstaclesIndices).data();
			}
			else
			{
				util3d::occupancy2DFromGroundObstacles<pcl::PointXYZRGB>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
			}
		}
		else if(scan.hasNormals())
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud = util3d::laserScanToPointCloudNormal(scan, scan.localTransform());
			pcl::PointCloud<pcl::PointNormal>::Ptr cloudSegmented = segmentCloud<pcl::PointNormal>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			if(grid3D_)
			{
				groundCloud = util3d::laserScanFromPointCloud(*cloudSegmented, groundIndices).data();
				obstaclesCloud = util3d::laserScanFromPointCloud(*cloudSegmented, obstaclesIndices).data();
			}
			else
			{
				util3d::occupancy2DFromGroundObstacles<pcl::PointNormal>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
			}
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::laserScanToPointCloud(scan, scan.localTransform());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented = segmentCloud<pcl::PointXYZ>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			if(grid3D_)
			{
				groundCloud = util3d::laserScanFromPointCloud(*cloudSegmented, groundIndices).data();
				obstaclesCloud = util3d::laserScanFromPointCloud(*cloudSegmented, obstaclesIndices).data();
			}
			else
			{
				util3d::occupancy2DFromGroundObstacles<pcl::PointXYZ>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
			}
		}

		if(grid3D_ && (!obstaclesCloud.empty() || !groundCloud.empty()))
		{
			UDEBUG("ground=%d obstacles=%d", groundCloud.cols, obstaclesCloud.cols);
			if(groundIsObstacle_ && !groundCloud.empty())
			{
				if(obstaclesCloud.empty())
				{
					obstaclesCloud = groundCloud;
					groundCloud = cv::Mat();
				}
				else
				{
					UASSERT(obstaclesCloud.type() == groundCloud.type());
					cv::Mat merged(1,obstaclesCloud.cols+groundCloud.cols, obstaclesCloud.type());
					obstaclesCloud.copyTo(merged(cv::Range::all(), cv::Range(0, obstaclesCloud.cols)));
					groundCloud.copyTo(merged(cv::Range::all(), cv::Range(obstaclesCloud.cols, obstaclesCloud.cols+groundCloud.cols)));
				}
			}

			// transform back in base frame
			float roll, pitch, yaw;
			pose.getEulerAngles(roll, pitch, yaw);
			Transform tinv = Transform(0,0, projMapFrame_?pose.z():0, roll, pitch, 0).inverse();

			if(rayTracing_)
			{
#ifdef RTABMAP_OCTOMAP
				if(!groundCloud.empty() || !obstaclesCloud.empty())
				{
					//create local octomap
					ParametersMap params;
					params.insert(ParametersPair(Parameters::kGridCellSize(), uNumber2Str(cellSize_)));
					params.insert(ParametersPair(Parameters::kGridRangeMax(), uNumber2Str(cloudMaxDepth_)));
					params.insert(ParametersPair(Parameters::kGridRayTracing(), uNumber2Str(rayTracing_)));
					OctoMap octomap(params);
					octomap.addToCache(1, groundCloud, obstaclesCloud, cv::Mat(), cv::Point3f(viewPointInOut.x, viewPointInOut.y, viewPointInOut.z));
					std::map<int, Transform> poses;
					poses.insert(std::make_pair(1, Transform::getIdentity()));
					octomap.update(poses);

					pcl::IndicesPtr groundIndices(new std::vector<int>);
					pcl::IndicesPtr obstaclesIndices(new std::vector<int>);
					pcl::IndicesPtr emptyIndices(new std::vector<int>);
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithRayTracing = octomap.createCloud(0, obstaclesIndices.get(), emptyIndices.get(), groundIndices.get());
					UDEBUG("ground=%d obstacles=%d empty=%d", (int)groundIndices->size(), (int)obstaclesIndices->size(), (int)emptyIndices->size());
					if(scan.hasRGB())
					{
						groundCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing, groundIndices, tinv).data();
						obstacleCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing, obstaclesIndices, tinv).data();
						emptyCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing, emptyIndices, tinv).data();
					}
					else
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWithRayTracing2(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::copyPointCloud(*cloudWithRayTracing, *cloudWithRayTracing2);
						groundCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing2, groundIndices, tinv).data();
						obstacleCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing2, obstaclesIndices, tinv).data();
						emptyCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing2, emptyIndices, tinv).data();
					}
				}
			}
			else
#else
					UWARN("RTAB-Map is not built with OctoMap dependency, 3D ray tracing is ignored. Set \"%s\" to false to avoid this warning.", Parameters::kGridRayTracing().c_str());
				}
#endif
			{
				groundCells = util3d::transformLaserScan(LaserScan::backwardCompatibility(groundCloud), tinv).data();
				obstacleCells = util3d::transformLaserScan(LaserScan::backwardCompatibility(obstaclesCloud), tinv).data();
			}

		}
		else if(!grid3D_ && rayTracing_ && (!obstacleCells.empty() || !groundCells.empty()))
		{
			cv::Mat laserScan = obstacleCells;
			cv::Mat laserScanNoHit = groundCells;
			obstacleCells = cv::Mat();
			groundCells = cv::Mat();
			util3d::occupancy2DFromLaserScan(
					laserScan,
					laserScanNoHit,
					viewPointInOut,
					emptyCells,
					obstacleCells,
					cellSize_,
					false, // don't fill unknown space
					cloudMaxDepth_);
		}
	}
	UDEBUG("ground=%d obstacles=%d empty=%d, channels=%d", groundCells.cols, obstacleCells.cols, emptyCells.cols, obstacleCells.cols?obstacleCells.channels():groundCells.channels());
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
	assembledGround_->clear();
	assembledObstacles_->clear();
}

cv::Mat OccupancyGrid::getMap(float & xMin, float & yMin) const
{
	xMin = xMin_;
	yMin = yMin_;

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
	xMin = xMin_;
	yMin = yMin_;

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

void OccupancyGrid::addToCache(
		int nodeId,
		const cv::Mat & ground,
		const cv::Mat & obstacles,
		const cv::Mat & empty)
{
	UDEBUG("nodeId=%d", nodeId);
	if(nodeId < 0)
	{
		UWARN("Cannot add nodes with negative id (nodeId=%d)", nodeId);
		return;
	}
	uInsert(cache_, std::make_pair(nodeId==0?-1:nodeId, std::make_pair(std::make_pair(ground, obstacles), empty)));
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
	bool graphOptimized = false; // If a loop closure happened (e.g., poses are modified)
	bool graphChanged = !addedNodes_.empty(); // If the new map doesn't have any node from the previous map
	std::map<int, Transform> transforms;
	float updateErrorSqrd = updateError_*updateError_;
	for(std::map<int, Transform>::iterator iter=addedNodes_.begin(); iter!=addedNodes_.end(); ++iter)
	{
		std::map<int, Transform>::const_iterator jter = posesIn.find(iter->first);
		if(jter != posesIn.end())
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
			for(std::list<std::pair<int, Transform> >::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				if(uContains(cache_, iter->first))
				{
					const std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> & pair = cache_.at(iter->first);

					UDEBUG("Adding grid %d: ground=%d obstacles=%d empty=%d", iter->first, pair.first.first.cols, pair.first.second.cols, pair.second.cols);

					//ground
					if(pair.first.first.cols)
					{
						if(pair.first.first.rows > 1 && pair.first.first.cols == 1)
						{
							UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", pair.first.first.rows, pair.first.first.cols);
						}
						cv::Mat ground(1, pair.first.first.cols, CV_32FC2);
						for(int i=0; i<ground.cols; ++i)
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
						uInsert(emptyLocalMaps, std::make_pair(iter->first, ground));

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
						cv::Mat ground(1, pair.second.cols, CV_32FC2);
						for(int i=0; i<ground.cols; ++i)
						{
							const float * vi = pair.second.ptr<float>(0,i);
							float * vo = ground.ptr<float>(0,i);
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
						uInsert(emptyLocalMaps, std::make_pair(iter->first, ground));

						if(cloudAssembling_)
						{
							*assembledEmptyCells_ += *util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(pair.second), iter->second, 0, 255, 0);
							assembledEmptyCellsUpdated = true;
						}
					}

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
				UDEBUG("map min=(%f, %f) odlMin(%f,%f) max=(%f,%f)", xMin, yMin, xMin_, yMin_, xMax, yMax);
				cv::Size newMapSize((xMax - xMin) / cellSize_+0.5f, (yMax - yMin) / cellSize_+0.5f);
				if(map_.empty())
				{
					UDEBUG("Map empty!");
					map = cv::Mat::ones(newMapSize, CV_8S)*-1;
					mapInfo = cv::Mat::zeros(newMapSize, CV_32FC4);
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
	unsigned long memoryUsage = sizeof(OccupancyGrid);
	memoryUsage += parameters_.size()*(sizeof(std::string)*2+sizeof(ParametersMap::iterator)) + sizeof(ParametersMap);

	memoryUsage += cache_.size()*(sizeof(int) + sizeof(std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat>) + sizeof(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator)) + sizeof(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >);
	for(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::const_iterator iter=cache_.begin(); iter!=cache_.end(); ++iter)
	{
		memoryUsage += iter->second.first.first.total() * iter->second.first.first.elemSize();
		memoryUsage += iter->second.first.second.total() * iter->second.first.second.elemSize();
		memoryUsage += iter->second.second.total() * iter->second.second.elemSize();
	}
	memoryUsage += map_.total() * map_.elemSize();
	memoryUsage += mapInfo_.total() * mapInfo_.elemSize();
	memoryUsage += cellCount_.size()*(sizeof(int)*3 + sizeof(std::pair<int, int>) + sizeof(std::map<int, std::pair<int, int> >::iterator)) + sizeof(std::map<int, std::pair<int, int> >);
	memoryUsage += addedNodes_.size()*(sizeof(int) + sizeof(Transform)+ sizeof(float)*12 + sizeof(std::map<int, Transform>::iterator)) + sizeof(std::map<int, Transform>);

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
