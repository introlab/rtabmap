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

#include <rtabmap/core/Occupancy.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

Occupancy::Occupancy(const ParametersMap & parameters) :
	parameters_(parameters),
	cloudDecimation_(Parameters::defaultGridDepthDecimation()),
	cloudMaxDepth_(Parameters::defaultGridDepthMax()),
	cloudMinDepth_(Parameters::defaultGridDepthMin()),
	cellSize_(Parameters::defaultGridCellSize()),
	occupancyFromCloud_(Parameters::defaultGridFromDepth()),
	projMapFrame_(Parameters::defaultGridMapFrameProjection()),
	maxObstacleHeight_(Parameters::defaultGridMaxObstacleHeight()),
	maxGroundAngle_(Parameters::defaultGridMaxGroundAngle()),
	minClusterSize_(Parameters::defaultGridMinClusterSize()),
	flatObstaclesDetected_(Parameters::defaultGridFlatObstacleDetected()),
	maxGroundHeight_(Parameters::defaultGridMaxGroundHeight()),
	grid3D_(Parameters::defaultGrid3D()),
	groundIsObstacle_(Parameters::defaultGrid3DGroundIsObstacle()),
	noiseFilteringRadius_(Parameters::defaultGridNoiseFilteringRadius()),
	noiseFilteringMinNeighbors_(Parameters::defaultGridNoiseFilteringMinNeighbors())
{
	this->parseParameters(parameters);
}

void Occupancy::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kGridFromDepth(), occupancyFromCloud_);
	Parameters::parse(parameters, Parameters::kGridDepthDecimation(), cloudDecimation_);
	Parameters::parse(parameters, Parameters::kGridDepthMin(), cloudMinDepth_);
	Parameters::parse(parameters, Parameters::kGridDepthMax(), cloudMaxDepth_);
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	Parameters::parse(parameters, Parameters::kGridMapFrameProjection(), projMapFrame_);
	Parameters::parse(parameters, Parameters::kGridMaxObstacleHeight(), maxObstacleHeight_);
	Parameters::parse(parameters, Parameters::kGridMaxGroundHeight(), maxGroundHeight_);
	Parameters::parse(parameters, Parameters::kGridMaxGroundAngle(), maxGroundAngle_);
	Parameters::parse(parameters, Parameters::kGridMinClusterSize(), minClusterSize_);
	Parameters::parse(parameters, Parameters::kGridFlatObstacleDetected(), flatObstaclesDetected_);
	Parameters::parse(parameters, Parameters::kGrid3D(), grid3D_);
	Parameters::parse(parameters, Parameters::kGrid3DGroundIsObstacle(), groundIsObstacle_);
	Parameters::parse(parameters, Parameters::kGridNoiseFilteringRadius(), noiseFilteringRadius_);
	Parameters::parse(parameters, Parameters::kGridNoiseFilteringMinNeighbors(), noiseFilteringMinNeighbors_);
}

void Occupancy::segment(const Signature & node, cv::Mat & obstacles, cv::Mat & ground)
{
	if(!occupancyFromCloud_ && node.sensorData().laserScanRaw().channels() == 2)
	{
		//2D
		util3d::occupancy2DFromLaserScan(
				node.sensorData().laserScanRaw(),
				ground,
				obstacles,
				cellSize_);
	}
	else
	{
		// 3D
		pcl::IndicesPtr indices(new std::vector<int>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		if(!occupancyFromCloud_)
		{
			cloud =util3d::laserScanToPointCloud(node.sensorData().laserScanRaw());
		}
		else
		{
			cloud = util3d::cloudFromSensorData(
					node.sensorData(),
					cloudDecimation_,
					cloudMaxDepth_,
					cloudMinDepth_,
					indices.get(),
					parameters_);
		}

		if(cloud->size())
		{
			// voxelize to grid cell size
			cloud = util3d::voxelize(cloud, indices, cellSize_);
			indices->clear();

			// Do radius filtering after voxel filtering ( a lot faster)
			if(noiseFilteringRadius_ > 0.0 &&
			   noiseFilteringMinNeighbors_ > 0)
			{
				indices = rtabmap::util3d::radiusFiltering(
						cloud,
						noiseFilteringRadius_,
						noiseFilteringMinNeighbors_);

				if(indices->empty())
				{
					UWARN("Cloud (with %d points) is empty after noise "
							"filtering. Occupancy grid of node %d cannot be "
							"created.",
							(int)cloud->size(), node.id());
					return;
				}
			}

			// add pose rotation without yaw
			float roll, pitch, yaw;
			node.getPose().getEulerAngles(roll, pitch, yaw);
			if(indices->size())
			{
				cloud = util3d::transformPointCloud(cloud, indices, Transform(0,0, projMapFrame_?node.getPose().z():0, roll, pitch, 0));
			}
			else
			{
				cloud = util3d::transformPointCloud(cloud, Transform(0,0, projMapFrame_?node.getPose().z():0, roll, pitch, 0));
			}

			if(maxObstacleHeight_ != 0.0f)
			{
				cloud = util3d::passThrough(cloud, "z", std::numeric_limits<int>::min(), maxObstacleHeight_);
			}

			pcl::IndicesPtr groundIndices, obstaclesIndices;
			util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
					cloud,
					groundIndices,
					obstaclesIndices,
					20,
					maxGroundAngle_,
					cellSize_*2.0f,
					minClusterSize_,
					flatObstaclesDetected_,
					maxGroundHeight_);

			pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZ>);

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
				if(groundIsObstacle_)
				{
					*obstaclesCloud += *groundCloud;
					groundCloud->clear();
				}

				// transform back in base frame
				Transform tinv = Transform(0,0, projMapFrame_?node.getPose().z():0, roll, pitch, 0).inverse();
				ground = util3d::laserScanFromPointCloud(*groundCloud, tinv);
				obstacles = util3d::laserScanFromPointCloud(*obstaclesCloud, tinv);
			}
			else
			{
				// projection on the xy plane
				util3d::occupancy2DFromGroundObstacles<pcl::PointXYZ>(
						groundCloud,
						obstaclesCloud,
						ground,
						obstacles,
						cellSize_);
			}
		}
	}
}

}
