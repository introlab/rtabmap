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

#ifndef UTIL3D_MAPPING_HPP_
#define UTIL3D_MAPPING_HPP_

#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>

namespace rtabmap{
namespace util3d{

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr projectCloudOnXYPlane(
		const typename pcl::PointCloud<PointT> & cloud)
{
	typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	*output = cloud;
	for(unsigned int i=0; i<output->size(); ++i)
	{
		output->at(i).z = 0;
	}
	return output;
}

template<typename PointT>
void segmentObstaclesFromGround(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const typename pcl::IndicesPtr & indices,
		pcl::IndicesPtr & ground,
		pcl::IndicesPtr & obstacles,
		int normalKSearch,
		float groundNormalAngle,
		float clusterRadius,
		int minClusterSize,
		bool segmentFlatObstacles,
		float maxGroundHeight,
		pcl::IndicesPtr * flatObstacles,
		const Eigen::Vector4f & viewPoint,
		float groundNormalsUp)
{
	ground.reset(new std::vector<int>);
	obstacles.reset(new std::vector<int>);
	if(flatObstacles)
	{
		flatObstacles->reset(new std::vector<int>);
	}

	if(cloud->size())
	{
		// Find the ground
		pcl::IndicesPtr flatSurfaces = normalFiltering(
				cloud,
				indices,
				groundNormalAngle,
				Eigen::Vector4f(0,0,1,0),
				normalKSearch,
				viewPoint,
				groundNormalsUp);

		if(segmentFlatObstacles && flatSurfaces->size())
		{
			int biggestFlatSurfaceIndex;
			std::vector<pcl::IndicesPtr> clusteredFlatSurfaces = extractClusters(
					cloud,
					flatSurfaces,
					clusterRadius,
					minClusterSize,
					std::numeric_limits<int>::max(),
					&biggestFlatSurfaceIndex);

			// cluster all surfaces for which the centroid is in the Z-range of the bigger surface
			if(clusteredFlatSurfaces.size())
			{
				Eigen::Vector4f biggestSurfaceMin,biggestSurfaceMax;
				if(maxGroundHeight != 0.0f)
				{
					// Search for biggest surface under max ground height
					size_t points = 0;
					biggestFlatSurfaceIndex = -1;
					for(size_t i=0;i<clusteredFlatSurfaces.size();++i)
					{
						Eigen::Vector4f min,max;
						pcl::getMinMax3D(*cloud, *clusteredFlatSurfaces.at(i), min, max);
						if(min[2]<maxGroundHeight && clusteredFlatSurfaces.size() > points)
						{
							points = clusteredFlatSurfaces.at(i)->size();
							biggestFlatSurfaceIndex = i;
							biggestSurfaceMin = min;
							biggestSurfaceMax = max;
						}
					}
				}
				else
				{
					pcl::getMinMax3D(*cloud, *clusteredFlatSurfaces.at(biggestFlatSurfaceIndex), biggestSurfaceMin, biggestSurfaceMax);
				}
				if(biggestFlatSurfaceIndex>=0)
				{
					ground = clusteredFlatSurfaces.at(biggestFlatSurfaceIndex);
				}

				if(!ground->empty() && (maxGroundHeight == 0.0f || biggestSurfaceMin[2] < maxGroundHeight))
				{
					for(unsigned int i=0; i<clusteredFlatSurfaces.size(); ++i)
					{
						if((int)i!=biggestFlatSurfaceIndex)
						{
							Eigen::Vector4f centroid(0,0,0,1);
							pcl::compute3DCentroid(*cloud, *clusteredFlatSurfaces.at(i), centroid);
							if(maxGroundHeight==0.0f || centroid[2] <= maxGroundHeight || centroid[2] <= biggestSurfaceMax[2]) // epsilon
							{
								ground = util3d::concatenate(ground, clusteredFlatSurfaces.at(i));
							}
							else if(flatObstacles)
							{
								*flatObstacles = util3d::concatenate(*flatObstacles, clusteredFlatSurfaces.at(i));
							}
						}
					}
				}
				else
				{
					// reject ground!
					ground.reset(new std::vector<int>);
					if(flatObstacles)
					{
						*flatObstacles = flatSurfaces;
					}
				}
			}
		}
		else
		{
			ground = flatSurfaces;
		}

		if(ground->size() != cloud->size())
		{
			// Remove ground
			pcl::IndicesPtr notObstacles = ground;
			if(indices->size())
			{
				notObstacles = util3d::extractIndices(cloud, indices, true);
				notObstacles = util3d::concatenate(notObstacles, ground);
			}
			pcl::IndicesPtr otherStuffIndices = util3d::extractIndices(cloud, notObstacles, true);

			// If ground height is set, remove obstacles under it
			if(maxGroundHeight != 0.0f)
			{
				otherStuffIndices = rtabmap::util3d::passThrough(cloud, otherStuffIndices, "z", maxGroundHeight, std::numeric_limits<float>::max());
			}

			//Cluster remaining stuff (obstacles)
			if(otherStuffIndices->size())
			{
				std::vector<pcl::IndicesPtr> clusteredObstaclesSurfaces = util3d::extractClusters(
						cloud,
						otherStuffIndices,
						clusterRadius,
						minClusterSize);

				// merge indices
				obstacles = util3d::concatenate(clusteredObstaclesSurfaces);
			}
		}
	}
}

template<typename PointT>
void segmentObstaclesFromGround(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		pcl::IndicesPtr & ground,
		pcl::IndicesPtr & obstacles,
		int normalKSearch,
		float groundNormalAngle,
		float clusterRadius,
		int minClusterSize,
		bool segmentFlatObstacles,
		float maxGroundHeight,
		pcl::IndicesPtr * flatObstacles,
		const Eigen::Vector4f & viewPoint,
		float groundNormalsUp)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	segmentObstaclesFromGround<PointT>(
			cloud,
			indices,
			ground,
			obstacles,
			normalKSearch,
			groundNormalAngle,
			clusterRadius,
			minClusterSize,
			segmentFlatObstacles,
			maxGroundHeight,
			flatObstacles,
			viewPoint,
			groundNormalsUp);
}

template<typename PointT>
void occupancy2DFromGroundObstacles(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & groundIndices,
		const pcl::IndicesPtr & obstaclesIndices,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize)
{
	typename pcl::PointCloud<PointT>::Ptr groundCloud(new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr obstaclesCloud(new pcl::PointCloud<PointT>);

	if(groundIndices->size())
	{
		pcl::copyPointCloud(*cloud, *groundIndices, *groundCloud);
	}

	if(obstaclesIndices->size())
	{
		pcl::copyPointCloud(*cloud, *obstaclesIndices, *obstaclesCloud);
	}

	occupancy2DFromGroundObstacles<PointT>(
			groundCloud,
			obstaclesCloud,
			ground,
			obstacles,
			cellSize);
}

template<typename PointT>
void occupancy2DFromGroundObstacles(
		const typename pcl::PointCloud<PointT>::Ptr & groundCloud,
		const typename pcl::PointCloud<PointT>::Ptr & obstaclesCloud,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize)
{
	ground = cv::Mat();
	if(groundCloud->size())
	{
		//project on XY plane
		typename pcl::PointCloud<PointT>::Ptr groundCloudProjected;
		groundCloudProjected = util3d::projectCloudOnXYPlane(*groundCloud);
		//voxelize to grid cell size
		groundCloudProjected = util3d::voxelize(groundCloudProjected, cellSize);

		ground = cv::Mat(1, (int)groundCloudProjected->size(), CV_32FC2);
		for(unsigned int i=0;i<groundCloudProjected->size(); ++i)
		{
			cv::Vec2f * ptr = ground.ptr<cv::Vec2f>();
			ptr[i][0] = groundCloudProjected->at(i).x;
			ptr[i][1] = groundCloudProjected->at(i).y;
		}
	}

	obstacles = cv::Mat();
	if(obstaclesCloud->size())
	{
		//project on XY plane
		typename pcl::PointCloud<PointT>::Ptr obstaclesCloudProjected;
		obstaclesCloudProjected = util3d::projectCloudOnXYPlane(*obstaclesCloud);
		//voxelize to grid cell size
		obstaclesCloudProjected = util3d::voxelize(obstaclesCloudProjected, cellSize);

		obstacles = cv::Mat(1, (int)obstaclesCloudProjected->size(), CV_32FC2);
		for(unsigned int i=0;i<obstaclesCloudProjected->size(); ++i)
		{
			cv::Vec2f * ptr = obstacles.ptr<cv::Vec2f>();
			ptr[i][0] = obstaclesCloudProjected->at(i).x;
			ptr[i][1] = obstaclesCloudProjected->at(i).y;
		}
	}
}

template<typename PointT>
void occupancy2DFromCloud3D(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize,
		float groundNormalAngle,
		int minClusterSize,
		bool segmentFlatObstacles,
		float maxGroundHeight)
{
	if(cloud->size() == 0)
	{
		return;
	}
	pcl::IndicesPtr groundIndices, obstaclesIndices;

	segmentObstaclesFromGround<PointT>(
			cloud,
			indices,
			groundIndices,
			obstaclesIndices,
			20,
			groundNormalAngle,
			cellSize*2.0f,
			minClusterSize,
			segmentFlatObstacles,
			maxGroundHeight);

	occupancy2DFromGroundObstacles<PointT>(
			cloud,
			groundIndices,
			obstaclesIndices,
			ground,
			obstacles,
			cellSize);
}

template<typename PointT>
void occupancy2DFromCloud3D(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize,
		float groundNormalAngle,
		int minClusterSize,
		bool segmentFlatObstacles,
		float maxGroundHeight)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	occupancy2DFromCloud3D<PointT>(cloud, indices, ground, obstacles, cellSize, groundNormalAngle, minClusterSize, segmentFlatObstacles, maxGroundHeight);
}

}
}

#endif /* UTIL3D_MAPPING_HPP_ */
