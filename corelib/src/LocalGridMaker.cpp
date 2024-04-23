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

#include <rtabmap/core/LocalGridMaker.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#ifdef RTABMAP_OCTOMAP
#include <rtabmap/core/global_map/OctoMap.h>
#endif

#include <pcl/io/pcd_io.h>

namespace rtabmap {

LocalGridMaker::LocalGridMaker(const ParametersMap & parameters) :
	parameters_(parameters),
	cloudDecimation_(Parameters::defaultGridDepthDecimation()),
	rangeMax_(Parameters::defaultGridRangeMax()),
	rangeMin_(Parameters::defaultGridRangeMin()),
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
	rayTracing_(Parameters::defaultGridRayTracing())
{
	this->parseParameters(parameters);
}

LocalGridMaker::~LocalGridMaker()
{
}

void LocalGridMaker::parseParameters(const ParametersMap & parameters)
{
	uInsert(parameters_, parameters);

	Parameters::parse(parameters, Parameters::kGridSensor(), occupancySensor_);
	Parameters::parse(parameters, Parameters::kGridDepthDecimation(), cloudDecimation_);
	if(cloudDecimation_ == 0)
	{
		cloudDecimation_ = 1;
	}
	Parameters::parse(parameters, Parameters::kGridRangeMin(), rangeMin_);
	Parameters::parse(parameters, Parameters::kGridRangeMax(), rangeMax_);
	Parameters::parse(parameters, Parameters::kGridFootprintLength(), footprintLength_);
	Parameters::parse(parameters, Parameters::kGridFootprintWidth(), footprintWidth_);
	Parameters::parse(parameters, Parameters::kGridFootprintHeight(), footprintHeight_);
	Parameters::parse(parameters, Parameters::kGridScanDecimation(), scanDecimation_);
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	UASSERT(cellSize_>0.0f);

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

void LocalGridMaker::createLocalMap(
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
		if(rangeMin_ > 0.0f)
		{
			scan = util3d::rangeFiltering(scan, rangeMin_, 0.0f);
		}

		float maxRange = rangeMax_;
		if(rangeMax_>0.0f && node.sensorData().laserScanRaw().rangeMax()>0.0f)
		{
			maxRange = rangeMax_ < node.sensorData().laserScanRaw().rangeMax()?rangeMax_:node.sensorData().laserScanRaw().rangeMax();
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
				float maxRange = rayTracing_?0.0f:rangeMax_;
#else
				// If ray tracing enabled, clipping will be done in occupancy2DFromLaserScan()
				float maxRange = !grid3D_ && rayTracing_?0.0f:rangeMax_;
#endif
				if(rangeMin_ > 0.0f || maxRange > 0.0f)
				{
					scan = util3d::rangeFiltering(scan, rangeMin_, maxRange);
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
				UWARN("Cannot create local map from scan: scan is empty (node=%d, %s=%d).", node.id(), Parameters::kGridSensor().c_str(), occupancySensor_);
			}
		}

		if(occupancySensor_ >= 1)
		{
			pcl::IndicesPtr indices(new std::vector<int>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
			UDEBUG("Depth image : decimation=%d max=%f min=%f",
					cloudDecimation_,
					rangeMax_,
					rangeMin_);
			cloud = util3d::cloudRGBFromSensorData(
					node.sensorData(),
					cloudDecimation_,
#ifdef RTABMAP_OCTOMAP
					// If ray tracing enabled, clipping will be done in OctoMap or in occupancy2DFromLaserScan()
					rayTracing_?0.0f:rangeMax_,
#else
					// If ray tracing enabled, clipping will be done in occupancy2DFromLaserScan()
					!grid3D_&&rayTracing_?0.0f:rangeMax_,
#endif
					rangeMin_,
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
				// average of all local transforms
				float sum = 0;
				for(unsigned int i=0; i<node.sensorData().stereoCameraModels().size(); ++i)
				{
					const Transform & t = node.sensorData().stereoCameraModels()[i].localTransform();
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

			cv::Mat scanGroundCells;
			cv::Mat scanObstacleCells;
			cv::Mat scanEmptyCells;
			if(occupancySensor_ == 2)
			{
				// backup
				scanGroundCells = groundCells;
				scanObstacleCells = obstacleCells;
				scanEmptyCells = emptyCells;
				groundCells = cv::Mat();
				obstacleCells = cv::Mat();
				emptyCells = cv::Mat();
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

void LocalGridMaker::createLocalMap(
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
					params.insert(ParametersPair(Parameters::kGridRangeMax(), uNumber2Str(rangeMax_)));
					params.insert(ParametersPair(Parameters::kGridRayTracing(), uNumber2Str(rayTracing_)));
					LocalGridCache cache;
					OctoMap octomap(&cache, params);
					cache.add(1, groundCloud, obstaclesCloud, cv::Mat(), cellSize_, cv::Point3f(viewPointInOut.x, viewPointInOut.y, viewPointInOut.z));
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
					rangeMax_);
		}
	}
	UDEBUG("ground=%d obstacles=%d empty=%d, channels=%d", groundCells.cols, obstacleCells.cols, emptyCells.cols, obstacleCells.cols?obstacleCells.channels():groundCells.channels());
}

} // namespace rtabmap
