/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef UTIL3D_HPP_
#define UTIL3D_HPP_

#include <rtabmap/utilite/ULogger.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>

namespace rtabmap{
namespace util3d{

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr voxelize(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		float voxelSize)
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	UASSERT(voxelSize > 0.0f);
	PointCloudPtr output(new PointCloud);
	pcl::VoxelGrid<PointT> filter;
	filter.setLeafSize(voxelSize, voxelSize, voxelSize);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr sampling(
		const typename pcl::PointCloud<PointT>::Ptr & cloud, int samples)
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	UASSERT(samples > 0);
	PointCloudPtr output(new PointCloud);
	pcl::RandomSample<PointT> filter;
	filter.setSample(samples);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr passThrough(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max)
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	UASSERT(max > min);
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	PointCloudPtr output(new PointCloud);
	pcl::PassThrough<PointT> filter;
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr removeNaNFromPointCloud(
		const typename pcl::PointCloud<PointT>::Ptr & cloud)
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	PointCloudPtr output(new PointCloud);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud<PointT>(*cloud, *output, indices);
	return output;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr removeNaNNormalsFromPointCloud(
		const typename pcl::PointCloud<PointT>::Ptr & cloud)
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	PointCloudPtr output(new PointCloud);
	std::vector<int> indices;
	pcl::removeNaNNormalsFromPointCloud<PointT>(*cloud, *output, indices);
	return output;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr transformPointCloud(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const Transform & transform)
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	PointCloudPtr output(new PointCloud);
	pcl::transformPointCloud<PointT>(*cloud, *output, transformToEigen4f(transform));
	return output;
}

template<typename PointT>
PointT transformPoint(
		const PointT & pt,
		const Transform & transform)
{
	return pcl::transformPoint(pt, transformToEigen3f(transform));
}

template<typename PointT>
void segmentObstaclesFromGround(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		pcl::IndicesPtr & ground,
		pcl::IndicesPtr & obstacles,
		float normalRadiusSearch,
		float groundNormalAngle,
		int minClusterSize,
		bool segmentFlatObstacles)
{
	ground.reset(new std::vector<int>);
	obstacles.reset(new std::vector<int>);

	// Find the ground
	pcl::IndicesPtr flatSurfaces = util3d::normalFiltering<PointT>(
			cloud,
			groundNormalAngle,
			Eigen::Vector4f(0,0,1,0),
			normalRadiusSearch*2.0f,
			Eigen::Vector4f(0,0,100,0));

	if(segmentFlatObstacles)
	{
		int biggestFlatSurfaceIndex;
		std::vector<pcl::IndicesPtr> clusteredFlatSurfaces = util3d::extractClusters<PointT>(
				cloud,
				flatSurfaces,
				normalRadiusSearch*2.0f,
				minClusterSize,
				std::numeric_limits<int>::max(),
				&biggestFlatSurfaceIndex);


		// cluster all surfaces for which the centroid is in the Z-range of the bigger surface
		ground = clusteredFlatSurfaces.at(biggestFlatSurfaceIndex);
		Eigen::Vector4f min,max;
		pcl::getMinMax3D<PointT>(*cloud, *clusteredFlatSurfaces.at(biggestFlatSurfaceIndex), min, max);

		for(unsigned int i=0; i<clusteredFlatSurfaces.size(); ++i)
		{
			if((int)i!=biggestFlatSurfaceIndex)
			{
				Eigen::Vector4f centroid;
				pcl::compute3DCentroid<PointT>(*cloud, *clusteredFlatSurfaces.at(i), centroid);
				if(centroid[2] >= min[2] && centroid[2] <= max[2])
				{
					ground = util3d::concatenate(ground, clusteredFlatSurfaces.at(i));
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
		pcl::IndicesPtr otherStuffIndices = util3d::extractNegativeIndices<PointT>(cloud, ground);

		//Cluster remaining stuff (obstacles)
		std::vector<pcl::IndicesPtr> clusteredObstaclesSurfaces = util3d::extractClusters<PointT>(
				cloud,
				otherStuffIndices,
				normalRadiusSearch*2.0f,
				minClusterSize);

		// merge indices
		obstacles = util3d::concatenate(clusteredObstaclesSurfaces);
	}
}

template<typename PointT>
void projectCloudOnXYPlane(
		typename pcl::PointCloud<PointT>::Ptr & cloud)
{
	for(unsigned int i=0; i<cloud->size(); ++i)
	{
		cloud->at(i).z = 0;
	}
}

template<typename PointT>
pcl::IndicesPtr radiusFiltering(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return radiusFiltering<PointT>(cloud, indices, radiusSearch, minNeighborsInRadius);
}

template<typename PointT>
pcl::IndicesPtr radiusFiltering(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius)
{
	typedef typename pcl::search::KdTree<PointT> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;
	KdTreePtr tree (new KdTree(false));

	if(indices->size())
	{
		pcl::IndicesPtr output(new std::vector<int>(indices->size()));
		int oi = 0; // output iterator
		tree->setInputCloud(cloud, indices);
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			std::vector<int> kIndices;
			std::vector<float> kDistances;
			int k = tree->radiusSearch(cloud->at(indices->at(i)), radiusSearch, kIndices, kDistances);
			if(k > minNeighborsInRadius)
			{
				output->at(oi++) = indices->at(i);
			}
		}
		output->resize(oi);
		return output;
	}
	else
	{
		pcl::IndicesPtr output(new std::vector<int>(cloud->size()));
		int oi = 0; // output iterator
		tree->setInputCloud(cloud);
		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			std::vector<int> kIndices;
			std::vector<float> kDistances;
			int k = tree->radiusSearch(cloud->at(i), radiusSearch, kIndices, kDistances);
			if(k > minNeighborsInRadius)
			{
				output->at(oi++) = i;
			}
		}
		output->resize(oi);
		return output;
	}
}

template<typename PointT>
pcl::IndicesPtr normalFiltering(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		float angleMax,
		const Eigen::Vector4f & normal,
		float radiusSearch,
		const Eigen::Vector4f & viewpoint)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return normalFiltering<PointT>(cloud, indices, angleMax, normal, radiusSearch, viewpoint);
}

template<typename PointT>
pcl::IndicesPtr normalFiltering(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		float radiusSearch,
		const Eigen::Vector4f & viewpoint)
{
	typedef typename pcl::search::KdTree<PointT> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;

	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	if(indices->size())
	{
		ne.setIndices(indices);
	}

	KdTreePtr tree (new KdTree(false));

	if(indices->size())
	{
		tree->setInputCloud(cloud, indices);
	}
	else
	{
		tree->setInputCloud(cloud);
	}
	ne.setSearchMethod (tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	ne.setRadiusSearch (radiusSearch);
	if(viewpoint[0] != 0 || viewpoint[1] != 0 || viewpoint[2] != 0)
	{
		ne.setViewPoint(viewpoint[0], viewpoint[1], viewpoint[2]);
	}

	ne.compute (*cloud_normals);

	pcl::IndicesPtr output(new std::vector<int>(cloud_normals->size()));
	int oi = 0; // output iterator
	Eigen::Vector3f n(normal[0], normal[1], normal[2]);
	for(unsigned int i=0; i<cloud_normals->size(); ++i)
	{
		Eigen::Vector4f v(cloud_normals->at(i).normal_x, cloud_normals->at(i).normal_y, cloud_normals->at(i).normal_z, 0.0f);
		float angle = pcl::getAngle3D(normal, v);
		if(angle < angleMax)
		{
			output->at(oi++) = indices->size()!=0?indices->at(i):i;
		}
	}
	output->resize(oi);

	return output;
}

template<typename PointT>
std::vector<pcl::IndicesPtr> extractClusters(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return extractClusters<PointT>(cloud, indices, clusterTolerance, minClusterSize, maxClusterSize, biggestClusterIndex);
}

template<typename PointT>
std::vector<pcl::IndicesPtr> extractClusters(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	typedef typename pcl::search::KdTree<PointT> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;

	KdTreePtr tree(new KdTree);
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (clusterTolerance);
	ec.setMinClusterSize (minClusterSize);
	ec.setMaxClusterSize (maxClusterSize);
	ec.setInputCloud (cloud);

	if(indices->size())
	{
		ec.setIndices(indices);
		tree->setInputCloud(cloud, indices);
	}
	else
	{
		tree->setInputCloud(cloud);
	}
	ec.setSearchMethod (tree);

	std::vector<pcl::PointIndices> cluster_indices;
	ec.extract (cluster_indices);

	int maxIndex=-1;
	unsigned int maxSize = 0;
	std::vector<pcl::IndicesPtr> output(cluster_indices.size());
	for(unsigned int i=0; i<cluster_indices.size(); ++i)
	{
		output[i] = pcl::IndicesPtr(new std::vector<int>(cluster_indices[i].indices));

		if(maxSize < cluster_indices[i].indices.size())
		{
			maxSize = cluster_indices[i].indices.size();
			maxIndex = i;
		}
	}
	if(biggestClusterIndex)
	{
		*biggestClusterIndex = maxIndex;
	}

	return output;
}

template<typename PointT>
pcl::IndicesPtr extractNegativeIndices(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices)
{
	pcl::IndicesPtr output(new std::vector<int>);
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices(indices);
	extract.setNegative(true);
	extract.filter(*output);
	return output;
}

} // util3d
} // rtabmap
#endif //UTIL3D_HPP_
