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

#include "rtabmap/core/util3d_filtering.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>

#include <pcl/search/kdtree.h>

#include <pcl/common/common.h>

#include <pcl/segmentation/extract_clusters.h>

#include <rtabmap/utilite/ULogger.h>

namespace rtabmap
{

namespace util3d
{

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float voxelSize)
{
	UASSERT(voxelSize > 0.0f);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setLeafSize(voxelSize, voxelSize, voxelSize);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float voxelSize)
{
	UASSERT(voxelSize > 0.0f);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> filter;
	filter.setLeafSize(voxelSize, voxelSize, voxelSize);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr sampling(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, int samples)
{
	UASSERT(samples > 0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::RandomSample<pcl::PointXYZ> filter;
	filter.setSample(samples);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampling(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, int samples)
{
	UASSERT(samples > 0);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::RandomSample<pcl::PointXYZRGB> filter;
	filter.setSample(samples);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max)
{
	UASSERT(max > min);
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThrough(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max)
{
	UASSERT(max > min);
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *output, indices);
	return output;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *output, indices);
	return output;
}


pcl::PointCloud<pcl::PointNormal>::Ptr removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);
	std::vector<int> indices;
	pcl::removeNaNNormalsFromPointCloud(*cloud, *output, indices);
	return output;
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::vector<int> indices;
	pcl::removeNaNNormalsFromPointCloud(*cloud, *output, indices);
	return output;
}



pcl::IndicesPtr radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return radiusFiltering(cloud, indices, radiusSearch, minNeighborsInRadius);
}
pcl::IndicesPtr radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return radiusFiltering(cloud, indices, radiusSearch, minNeighborsInRadius);
}


pcl::IndicesPtr radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>(false));

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
pcl::IndicesPtr radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius)
{
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>(false));

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & substractCloud,
		float radiusSearch,
		int minNeighborsInRadius)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::IndicesPtr indicesOut = subtractFiltering(cloud, indices, substractCloud, indices, radiusSearch, minNeighborsInRadius);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud, *indicesOut, *out);
	return out;
}


pcl::IndicesPtr subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		int minNeighborsInRadius)
{
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>(false));

	if(indices->size())
	{
		pcl::IndicesPtr output(new std::vector<int>(indices->size()));
		int oi = 0; // output iterator
		if(substractIndices->size())
		{
			tree->setInputCloud(substractCloud, substractIndices);
		}
		else
		{
			tree->setInputCloud(substractCloud);
		}
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			std::vector<int> kIndices;
			std::vector<float> kDistances;
			int k = tree->radiusSearch(cloud->at(indices->at(i)), radiusSearch, kIndices, kDistances);
			if(k <= minNeighborsInRadius)
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
		if(substractIndices->size())
		{
			tree->setInputCloud(substractCloud, substractIndices);
		}
		else
		{
			tree->setInputCloud(substractCloud);
		}
		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			std::vector<int> kIndices;
			std::vector<float> kDistances;
			int k = tree->radiusSearch(cloud->at(i), radiusSearch, kIndices, kDistances);
			if(k <= minNeighborsInRadius)
			{
				output->at(oi++) = i;
			}
		}
		output->resize(oi);
		return output;
	}
}


pcl::IndicesPtr normalFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float angleMax,
		const Eigen::Vector4f & normal,
		float radiusSearch,
		const Eigen::Vector4f & viewpoint)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return normalFiltering(cloud, indices, angleMax, normal, radiusSearch, viewpoint);
}
pcl::IndicesPtr normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float angleMax,
		const Eigen::Vector4f & normal,
		float radiusSearch,
		const Eigen::Vector4f & viewpoint)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return normalFiltering(cloud, indices, angleMax, normal, radiusSearch, viewpoint);
}



pcl::IndicesPtr normalFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		float radiusSearch,
		const Eigen::Vector4f & viewpoint)
{
	pcl::IndicesPtr output(new std::vector<int>());

	if(cloud->size())
	{
		typedef pcl::search::KdTree<pcl::PointXYZ> KdTree;
		typedef KdTree::Ptr KdTreePtr;

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
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

		output->resize(cloud_normals->size());
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
	}

	return output;
}
pcl::IndicesPtr normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		float radiusSearch,
		const Eigen::Vector4f & viewpoint)
{
	pcl::IndicesPtr output(new std::vector<int>());

	if(cloud->size())
	{
		typedef pcl::search::KdTree<pcl::PointXYZRGB> KdTree;
		typedef KdTree::Ptr KdTreePtr;

		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
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

		output->resize(cloud_normals->size());
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
	}

	return output;
}

std::vector<pcl::IndicesPtr> extractClusters(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return extractClusters(cloud, indices, clusterTolerance, minClusterSize, maxClusterSize, biggestClusterIndex);
}
std::vector<pcl::IndicesPtr> extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return extractClusters(cloud, indices, clusterTolerance, minClusterSize, maxClusterSize, biggestClusterIndex);
}

std::vector<pcl::IndicesPtr> extractClusters(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	typedef pcl::search::KdTree<pcl::PointXYZ> KdTree;
	typedef KdTree::Ptr KdTreePtr;

	KdTreePtr tree(new KdTree);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
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
			maxSize = (unsigned int)cluster_indices[i].indices.size();
			maxIndex = i;
		}
	}
	if(biggestClusterIndex)
	{
		*biggestClusterIndex = maxIndex;
	}

	return output;
}
std::vector<pcl::IndicesPtr> extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	typedef pcl::search::KdTree<pcl::PointXYZRGB> KdTree;
	typedef KdTree::Ptr KdTreePtr;

	KdTreePtr tree(new KdTree);
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
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
			maxSize = (unsigned int)cluster_indices[i].indices.size();
			maxIndex = i;
		}
	}
	if(biggestClusterIndex)
	{
		*biggestClusterIndex = maxIndex;
	}

	return output;
}

pcl::IndicesPtr extractNegativeIndices(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices)
{
	pcl::IndicesPtr output(new std::vector<int>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud);
	extract.setIndices(indices);
	extract.setNegative(true);
	extract.filter(*output);
	return output;
}
pcl::IndicesPtr extractNegativeIndices(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices)
{
	pcl::IndicesPtr output(new std::vector<int>);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud (cloud);
	extract.setIndices(indices);
	extract.setNegative(true);
	extract.filter(*output);
	return output;
}

}

}
