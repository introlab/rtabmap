/*
 * util3d_mapping.hpp
 *
 *  Created on: 2015-05-13
 *      Author: mathieu
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
		pcl::IndicesPtr * flatObstacles)
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
				Eigen::Vector4f(0,0,100,0));

		if(segmentFlatObstacles)
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
				ground = clusteredFlatSurfaces.at(biggestFlatSurfaceIndex);
				Eigen::Vector4f min,max;
				pcl::getMinMax3D(*cloud, *clusteredFlatSurfaces.at(biggestFlatSurfaceIndex), min, max);

				if(maxGroundHeight <= 0 || min[2] < maxGroundHeight)
				{
					for(unsigned int i=0; i<clusteredFlatSurfaces.size(); ++i)
					{
						if((int)i!=biggestFlatSurfaceIndex)
						{
							Eigen::Vector4f centroid;
							pcl::compute3DCentroid(*cloud, *clusteredFlatSurfaces.at(i), centroid);
							if(centroid[2] >= min[2]-0.01 &&
							  (centroid[2] <= max[2]+0.01 || (maxGroundHeight>0 && centroid[2] <= maxGroundHeight+0.01))) // epsilon
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
			pcl::IndicesPtr otherStuffIndices = util3d::extractIndices(cloud, ground, true);

			// If ground height is set, remove obstacles under it
			if(maxGroundHeight > 0.0f)
			{
				otherStuffIndices = rtabmap::util3d::passThrough(cloud, otherStuffIndices, "z", maxGroundHeight, std::numeric_limits<float>::max());
			}

			//Cluster remaining stuff (obstacles)
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
		pcl::IndicesPtr * flatObstacles)
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
			flatObstacles);
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

	pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZ>);

	if(groundIndices->size())
	{
		pcl::copyPointCloud(*cloud, *groundIndices, *groundCloud);
		//project on XY plane
		util3d::projectCloudOnXYPlane(groundCloud);
		//voxelize to grid cell size
		groundCloud = util3d::voxelize(groundCloud, cellSize);
	}

	if(obstaclesIndices->size())
	{
		pcl::copyPointCloud(*cloud, *obstaclesIndices, *obstaclesCloud);
		//project on XY plane
		util3d::projectCloudOnXYPlane(obstaclesCloud);
		//voxelize to grid cell size
		obstaclesCloud = util3d::voxelize(obstaclesCloud, cellSize);
	}

	ground = cv::Mat();
	if(groundCloud->size())
	{
		ground = cv::Mat((int)groundCloud->size(), 1, CV_32FC2);
		for(unsigned int i=0;i<groundCloud->size(); ++i)
		{
			ground.at<cv::Vec2f>(i)[0] = groundCloud->at(i).x;
			ground.at<cv::Vec2f>(i)[1] = groundCloud->at(i).y;
		}
	}

	obstacles = cv::Mat();
	if(obstaclesCloud->size())
	{
		obstacles = cv::Mat((int)obstaclesCloud->size(), 1, CV_32FC2);
		for(unsigned int i=0;i<obstaclesCloud->size(); ++i)
		{
			obstacles.at<cv::Vec2f>(i)[0] = obstaclesCloud->at(i).x;
			obstacles.at<cv::Vec2f>(i)[1] = obstaclesCloud->at(i).y;
		}
	}
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
