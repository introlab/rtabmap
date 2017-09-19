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

#include "rtabmap/core/util3d_filtering.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/search/kdtree.h>

#include <pcl/common/common.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>

#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/extract_labeled_clusters.h>
#include <pcl/segmentation/impl/extract_labeled_clusters.hpp>

PCL_INSTANTIATE(EuclideanClusterExtraction, (pcl::PointXYZRGBNormal))
PCL_INSTANTIATE(extractEuclideanClusters, (pcl::PointXYZRGBNormal))
PCL_INSTANTIATE(extractEuclideanClusters_indices, (pcl::PointXYZRGBNormal))
#endif

namespace rtabmap
{

namespace util3d
{

cv::Mat downsample(
		const cv::Mat & cloud,
		int step)
{
	UASSERT(step > 0);
	cv::Mat output;
	if(step <= 1 || cloud.cols <= step)
	{
		// no sampling
		output = cloud.clone();
	}
	else
	{
		int finalSize = cloud.cols/step;
		output = cv::Mat(1, finalSize, cloud.type());
		int oi = 0;
		for(int i=0; i<cloud.cols-step+1; i+=step)
		{
			cv::Mat(cloud, cv::Range::all(), cv::Range(i,i+1)).copyTo(cv::Mat(output, cv::Range::all(), cv::Range(oi,oi+1)));
			++oi;
		}
	}
	return output;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int step)
{
	UASSERT(step > 0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	if(step <= 1 || (int)cloud->size() <= step)
	{
		// no sampling
		*output = *cloud;
	}
	else
	{
		int finalSize = int(cloud->size())/step;
		output->resize(finalSize);
		int oi = 0;
		for(unsigned int i=0; i<cloud->size()-step+1; i+=step)
		{
			(*output)[oi++] = cloud->at(i);
		}
	}
	return output;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int step)
{
	UASSERT(step > 0);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(step <= 1 || (int)cloud->size()<=step)
	{
		// no sampling
		*output = *cloud;
	}
	else
	{
		int finalSize = int(cloud->size())/step;
		output->resize(finalSize);
		int oi = 0;
		for(int i=0; i<(int)cloud->size()-step+1; i+=step)
		{
			(*output)[oi++] = cloud->at(i);
		}
	}
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize)
{
	UASSERT(voxelSize > 0.0f);
	UASSERT_MSG((cloud->is_dense && cloud->size()) || (!cloud->is_dense && indices->size()),
			uFormat("Cloud size=%d indices=%d is_dense=%s", (int)cloud->size(), (int)indices->size(), cloud->is_dense?"true":"false").c_str());
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setLeafSize(voxelSize, voxelSize, voxelSize);
	filter.setInputCloud(cloud);
	if(indices->size())
	{
		filter.setIndices(indices);
	}
	filter.filter(*output);
	return output;
}
pcl::PointCloud<pcl::PointNormal>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize)
{
	UASSERT(voxelSize > 0.0f);
	UASSERT_MSG((cloud->is_dense && cloud->size()) || (!cloud->is_dense && indices->size()),
			uFormat("Cloud size=%d indices=%d is_dense=%s", (int)cloud->size(), (int)indices->size(), cloud->is_dense?"true":"false").c_str());
	pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);
	pcl::VoxelGrid<pcl::PointNormal> filter;
	filter.setLeafSize(voxelSize, voxelSize, voxelSize);
	filter.setInputCloud(cloud);
	if(indices->size())
	{
		filter.setIndices(indices);
	}
	filter.filter(*output);
	return output;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize)
{
	UASSERT(voxelSize > 0.0f);
	UASSERT_MSG((cloud->is_dense && cloud->size()) || (!cloud->is_dense && indices->size()),
			uFormat("Cloud size=%d indices=%d is_dense=%s", (int)cloud->size(), (int)indices->size(), cloud->is_dense?"true":"false").c_str());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> filter;
	filter.setLeafSize(voxelSize, voxelSize, voxelSize);
	filter.setInputCloud(cloud);
	if(indices->size())
	{
		filter.setIndices(indices);
	}
	filter.filter(*output);
	return output;
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize)
{
	UASSERT(voxelSize > 0.0f);
	UASSERT_MSG((cloud->is_dense && cloud->size()) || (!cloud->is_dense && indices->size()),
			uFormat("Cloud size=%d indices=%d is_dense=%s", (int)cloud->size(), (int)indices->size(), cloud->is_dense?"true":"false").c_str());
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> filter;
	filter.setLeafSize(voxelSize, voxelSize, voxelSize);
	filter.setInputCloud(cloud);
	if(indices->size())
	{
		filter.setIndices(indices);
	}
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float voxelSize)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return voxelize(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointNormal>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		float voxelSize)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return voxelize(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float voxelSize)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return voxelize(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		float voxelSize)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return voxelize(cloud, indices, voxelSize);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr randomSampling(
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomSampling(
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

pcl::IndicesPtr passThrough(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative)
{
	UASSERT_MSG(max > min, uFormat("cloud=%d, max=%f min=%f axis=%s", (int)cloud->size(), max, min, axis.c_str()).c_str());
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	pcl::IndicesPtr output(new std::vector<int>);
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setNegative(negative);
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.setIndices(indices);
	filter.filter(*output);
	return output;
}
pcl::IndicesPtr passThrough(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative)
{
	UASSERT_MSG(max > min, uFormat("cloud=%d, max=%f min=%f axis=%s", (int)cloud->size(), max, min, axis.c_str()).c_str());
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	pcl::IndicesPtr output(new std::vector<int>);
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setNegative(negative);
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.setIndices(indices);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative)
{
	UASSERT_MSG(max > min, uFormat("cloud=%d, max=%f min=%f axis=%s", (int)cloud->size(), max, min, axis.c_str()).c_str());
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setNegative(negative);
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
		float max,
		bool negative)
{
	UASSERT_MSG(max > min, uFormat("cloud=%d, max=%f min=%f axis=%s", (int)cloud->size(), max, min, axis.c_str()).c_str());
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setNegative(negative);
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointNormal>::Ptr passThrough(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative)
{
	UASSERT_MSG(max > min, uFormat("cloud=%d, max=%f min=%f axis=%s", (int)cloud->size(), max, min, axis.c_str()).c_str());
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PassThrough<pcl::PointNormal> filter;
	filter.setNegative(negative);
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr passThrough(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative)
{
	UASSERT_MSG(max > min, uFormat("cloud=%d, max=%f min=%f axis=%s", (int)cloud->size(), max, min, axis.c_str()).c_str());
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PassThrough<pcl::PointXYZRGBNormal> filter;
	filter.setNegative(negative);
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::IndicesPtr cropBox(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform,
		bool negative)
{
	UASSERT(min[0] < max[0] && min[1] < max[1] && min[2] < max[2]);

	pcl::IndicesPtr output(new std::vector<int>);
	pcl::CropBox<pcl::PointXYZ> filter;
	filter.setNegative(negative);
	filter.setMin(min);
	filter.setMax(max);
	if(!transform.isNull() && !transform.isIdentity())
	{
		filter.setTransform(transform.toEigen3f());
	}
	filter.setInputCloud(cloud);
	filter.setIndices(indices);
	filter.filter(*output);
	return output;
}
pcl::IndicesPtr cropBox(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform,
		bool negative)
{
	UASSERT(min[0] < max[0] && min[1] < max[1] && min[2] < max[2]);

	pcl::IndicesPtr output(new std::vector<int>);
	pcl::CropBox<pcl::PointXYZRGB> filter;
	filter.setNegative(negative);
	filter.setMin(min);
	filter.setMax(max);
	if(!transform.isNull() && !transform.isIdentity())
	{
		filter.setTransform(transform.toEigen3f());
	}
	filter.setInputCloud(cloud);
	filter.setIndices(indices);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cropBox(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform,
		bool negative)
{
	UASSERT(min[0] < max[0] && min[1] < max[1] && min[2] < max[2]);

	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropBox<pcl::PointXYZ> filter;
	filter.setNegative(negative);
	filter.setMin(min);
	filter.setMax(max);
	if(!transform.isNull() && !transform.isIdentity())
	{
		filter.setTransform(transform.toEigen3f());
	}
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropBox(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform,
		bool negative)
{
	UASSERT(min[0] < max[0] && min[1] < max[1] && min[2] < max[2]);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::CropBox<pcl::PointXYZRGB> filter;
	filter.setNegative(negative);
	filter.setMin(min);
	filter.setMax(max);
	if(!transform.isNull() && !transform.isIdentity())
	{
		filter.setTransform(transform.toEigen3f());
	}
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::IndicesPtr frustumFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & cameraPose,
		float horizontalFOV, // in degrees
		float verticalFOV,   // in degrees
		float nearClipPlaneDistance,
		float farClipPlaneDistance,
		bool negative)
{
	UASSERT(horizontalFOV > 0.0f && verticalFOV > 0.0f);
	UASSERT(farClipPlaneDistance > nearClipPlaneDistance);
	UASSERT(!cameraPose.isNull());

	pcl::IndicesPtr output(new std::vector<int>);
	pcl::FrustumCulling<pcl::PointXYZ> fc;
	fc.setNegative(negative);
	fc.setInputCloud (cloud);
	if(indices.get() && indices->size())
	{
		fc.setIndices(indices);
	}
	fc.setVerticalFOV (verticalFOV);
	fc.setHorizontalFOV (horizontalFOV);
	fc.setNearPlaneDistance (nearClipPlaneDistance);
	fc.setFarPlaneDistance (farClipPlaneDistance);

	fc.setCameraPose (cameraPose.toEigen4f());
	fc.filter (*output);

	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr frustumFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & cameraPose,
		float horizontalFOV, // in degrees
		float verticalFOV,   // in degrees
		float nearClipPlaneDistance,
		float farClipPlaneDistance,
		bool negative)
{
	UASSERT(horizontalFOV > 0.0f && verticalFOV > 0.0f);
	UASSERT(farClipPlaneDistance > nearClipPlaneDistance);
	UASSERT(!cameraPose.isNull());

	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::FrustumCulling<pcl::PointXYZ> fc;
	fc.setNegative(negative);
	fc.setInputCloud (cloud);
	fc.setVerticalFOV (verticalFOV);
	fc.setHorizontalFOV (horizontalFOV);
	fc.setNearPlaneDistance (nearClipPlaneDistance);
	fc.setFarPlaneDistance (farClipPlaneDistance);

	fc.setCameraPose (cameraPose.toEigen4f());
	fc.filter (*output);

	return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr frustumFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & cameraPose,
		float horizontalFOV, // in degrees
		float verticalFOV,   // in degrees
		float nearClipPlaneDistance,
		float farClipPlaneDistance,
		bool negative)
{
	UASSERT(horizontalFOV > 0.0f && verticalFOV > 0.0f);
	UASSERT(farClipPlaneDistance > nearClipPlaneDistance);
	UASSERT(!cameraPose.isNull());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::FrustumCulling<pcl::PointXYZRGB> fc;
	fc.setNegative(negative);
	fc.setInputCloud (cloud);
	fc.setVerticalFOV (verticalFOV);
	fc.setHorizontalFOV (horizontalFOV);
	fc.setNearPlaneDistance (nearClipPlaneDistance);
	fc.setFarPlaneDistance (farClipPlaneDistance);

	fc.setCameraPose (cameraPose.toEigen4f());
	fc.filter (*output);

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
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
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
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
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
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius)
{
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>(false));

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
pcl::IndicesPtr radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius)
{
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>(false));

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
	UASSERT(minNeighborsInRadius > 0);
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
			if(k < minNeighborsInRadius)
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
			if(k < minNeighborsInRadius)
			{
				output->at(oi++) = i;
			}
		}
		output->resize(oi);
		return output;
	}
}

pcl::PointCloud<pcl::PointNormal>::Ptr subtractFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointNormal>::Ptr & substractCloud,
		float radiusSearch,
		float maxAngle,
		int minNeighborsInRadius)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::IndicesPtr indicesOut = subtractFiltering(cloud, indices, substractCloud, indices, radiusSearch, maxAngle, minNeighborsInRadius);
	pcl::PointCloud<pcl::PointNormal>::Ptr out(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud(*cloud, *indicesOut, *out);
	return out;
}


pcl::IndicesPtr subtractFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointNormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		float maxAngle,
		int minNeighborsInRadius)
{
	UASSERT(minNeighborsInRadius > 0);
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>(false));

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
			if(k>=minNeighborsInRadius && maxAngle > 0.0f)
			{
				Eigen::Vector4f normal(cloud->at(indices->at(i)).normal_x, cloud->at(indices->at(i)).normal_y, cloud->at(indices->at(i)).normal_z, 0.0f);
				if (uIsFinite(normal[0]) &&
					uIsFinite(normal[1]) &&
					uIsFinite(normal[2]))
				{
					int count = k;
					for(int j=0; j<count && k >= minNeighborsInRadius; ++j)
					{
						Eigen::Vector4f v(substractCloud->at(kIndices.at(j)).normal_x, substractCloud->at(kIndices.at(j)).normal_y, substractCloud->at(kIndices.at(j)).normal_z, 0.0f);
						if(uIsFinite(v[0]) &&
							uIsFinite(v[1]) &&
							uIsFinite(v[2]))
						{
							float angle = pcl::getAngle3D(normal, v);
							if(angle > maxAngle)
							{
								k-=1;
							}
						}
						else
						{
							k-=1;
						}
					}
				}
				else
				{
					k=0;
				}
			}
			if(k < minNeighborsInRadius)
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
			if(k>=minNeighborsInRadius && maxAngle > 0.0f)
			{
				Eigen::Vector4f normal(cloud->at(i).normal_x, cloud->at(i).normal_y, cloud->at(i).normal_z, 0.0f);
				if (uIsFinite(normal[0]) &&
					uIsFinite(normal[1]) &&
					uIsFinite(normal[2]))
				{
					int count = k;
					for(int j=0; j<count && k >= minNeighborsInRadius; ++j)
					{
						Eigen::Vector4f v(substractCloud->at(kIndices.at(j)).normal_x, substractCloud->at(kIndices.at(j)).normal_y, substractCloud->at(kIndices.at(j)).normal_z, 0.0f);
						if(uIsFinite(v[0]) &&
							uIsFinite(v[1]) &&
							uIsFinite(v[2]))
						{
							float angle = pcl::getAngle3D(normal, v);
							if(angle > maxAngle)
							{
								k-=1;
							}
						}
						else
						{
							k-=1;
						}
					}
				}
				else
				{
					k=0;
				}
			}
			if(k < minNeighborsInRadius)
			{
				output->at(oi++) = i;
			}
		}
		output->resize(oi);
		return output;
	}
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & substractCloud,
		float radiusSearch,
		float maxAngle,
		int minNeighborsInRadius)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::IndicesPtr indicesOut = subtractFiltering(cloud, indices, substractCloud, indices, radiusSearch, maxAngle, minNeighborsInRadius);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(*cloud, *indicesOut, *out);
	return out;
}


pcl::IndicesPtr subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		float maxAngle,
		int minNeighborsInRadius)
{
	UASSERT(minNeighborsInRadius > 0);
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>(false));

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
			if(k>=minNeighborsInRadius && maxAngle > 0.0f)
			{
				Eigen::Vector4f normal(cloud->at(indices->at(i)).normal_x, cloud->at(indices->at(i)).normal_y, cloud->at(indices->at(i)).normal_z, 0.0f);
				if (uIsFinite(normal[0]) &&
					uIsFinite(normal[1]) &&
					uIsFinite(normal[2]))
				{
					int count = k;
					for(int j=0; j<count && k >= minNeighborsInRadius; ++j)
					{
						Eigen::Vector4f v(substractCloud->at(kIndices.at(j)).normal_x, substractCloud->at(kIndices.at(j)).normal_y, substractCloud->at(kIndices.at(j)).normal_z, 0.0f);
						if(uIsFinite(v[0]) &&
							uIsFinite(v[1]) &&
							uIsFinite(v[2]))
						{
							float angle = pcl::getAngle3D(normal, v);
							if(angle > maxAngle)
							{
								k-=1;
							}
						}
						else
						{
							k-=1;
						}
					}
				}
				else
				{
					k=0;
				}
			}
			if(k < minNeighborsInRadius)
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
			if(k>=minNeighborsInRadius && maxAngle > 0.0f)
			{
				Eigen::Vector4f normal(cloud->at(i).normal_x, cloud->at(i).normal_y, cloud->at(i).normal_z, 0.0f);
				if (uIsFinite(normal[0]) &&
					uIsFinite(normal[1]) &&
					uIsFinite(normal[2]))
				{
					int count = k;
					for(int j=0; j<count && k >= minNeighborsInRadius; ++j)
					{
						Eigen::Vector4f v(substractCloud->at(kIndices.at(j)).normal_x, substractCloud->at(kIndices.at(j)).normal_y, substractCloud->at(kIndices.at(j)).normal_z, 0.0f);
						if(uIsFinite(v[0]) &&
							uIsFinite(v[1]) &&
							uIsFinite(v[2]))
						{
							float angle = pcl::getAngle3D(normal, v);
							if(angle > maxAngle)
							{
								k-=1;
							}
						}
						else
						{
							k-=1;
						}
					}
				}
				else
				{
					k=0;
				}
			}
			if(k < minNeighborsInRadius)
			{
				output->at(oi++) = i;
			}
		}
		output->resize(oi);
		return output;
	}
}

pcl::IndicesPtr subtractAdaptiveFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearchRatio,
		int minNeighborsInRadius,
		const Eigen::Vector3f & viewpoint)
{
	UWARN("Add angle to avoid subtraction of points with opposite normals");
	UASSERT(minNeighborsInRadius > 0);
	UASSERT(radiusSearchRatio > 0.0f);
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
			if(pcl::isFinite(cloud->at(indices->at(i))))
			{
				std::vector<int> kIndices;
				std::vector<float> kSqrdDistances;
				float radius = radiusSearchRatio*uNorm(
							cloud->at(indices->at(i)).x-viewpoint[0],
							cloud->at(indices->at(i)).y-viewpoint[1],
							cloud->at(indices->at(i)).z-viewpoint[2]);
				int k = tree->radiusSearch(cloud->at(indices->at(i)), radius, kIndices, kSqrdDistances);
				if(k < minNeighborsInRadius)
				{
					output->at(oi++) = indices->at(i);
				}
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
			if(pcl::isFinite(cloud->at(i)))
			{
				std::vector<int> kIndices;
				std::vector<float> kSqrdDistances;
				float radius = radiusSearchRatio*uNorm(
						cloud->at(i).x-viewpoint[0],
						cloud->at(i).y-viewpoint[1],
						cloud->at(i).z-viewpoint[2]);
				int k = tree->radiusSearch(cloud->at(i), radius, kIndices, kSqrdDistances);
				if(k < minNeighborsInRadius)
				{
					output->at(oi++) = i;
				}
			}
		}
		output->resize(oi);
		return output;
	}
}
pcl::IndicesPtr subtractAdaptiveFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearchRatio,
		float maxAngle,
		int minNeighborsInRadius,
		const Eigen::Vector3f & viewpoint)
{
	UASSERT(minNeighborsInRadius > 0);
	UASSERT(radiusSearchRatio > 0.0f);
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>(false));

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
			if(pcl::isFinite(cloud->at(indices->at(i))))
			{
				std::vector<int> kIndices;
				std::vector<float> kSqrdDistances;
				float radius = radiusSearchRatio*uNorm(
						cloud->at(indices->at(i)).x-viewpoint[0],
						cloud->at(indices->at(i)).y-viewpoint[1],
						cloud->at(indices->at(i)).z-viewpoint[2]);
				int k = tree->radiusSearch(cloud->at(indices->at(i)), radius, kIndices, kSqrdDistances);
				if(k>=minNeighborsInRadius && maxAngle > 0.0f)
				{
					Eigen::Vector4f normal(cloud->at(indices->at(i)).normal_x, cloud->at(indices->at(i)).normal_y, cloud->at(indices->at(i)).normal_z, 0.0f);
					if (uIsFinite(normal[0]) &&
						uIsFinite(normal[1]) &&
						uIsFinite(normal[2]))
					{
						int count = k;
						for(int j=0; j<count && k >= minNeighborsInRadius; ++j)
						{
							Eigen::Vector4f v(substractCloud->at(kIndices.at(j)).normal_x, substractCloud->at(kIndices.at(j)).normal_y, substractCloud->at(kIndices.at(j)).normal_z, 0.0f);
							if(uIsFinite(v[0]) &&
								uIsFinite(v[1]) &&
								uIsFinite(v[2]))
							{
								float angle = pcl::getAngle3D(normal, v);
								if(angle > maxAngle)
								{
									k-=1;
								}
							}
							else
							{
								k-=1;
							}
						}
					}
					else
					{
						k=0;
					}
				}

				if(k < minNeighborsInRadius)
				{
					output->at(oi++) = indices->at(i);
				}
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
			if(pcl::isFinite(cloud->at(i)))
			{
				std::vector<int> kIndices;
				std::vector<float> kSqrdDistances;
				float radius = radiusSearchRatio*uNorm(
						cloud->at(i).x-viewpoint[0],
						cloud->at(i).y-viewpoint[1],
						cloud->at(i).z-viewpoint[2]);
				int k = tree->radiusSearch(cloud->at(i), radius, kIndices, kSqrdDistances);
				if(k>=minNeighborsInRadius && maxAngle > 0.0f)
				{
					Eigen::Vector4f normal(cloud->at(i).normal_x, cloud->at(i).normal_y, cloud->at(i).normal_z, 0.0f);
					if (uIsFinite(normal[0]) &&
						uIsFinite(normal[1]) &&
						uIsFinite(normal[2]))
					{
						int count = k;
						for(int j=0; j<count && k >= minNeighborsInRadius; ++j)
						{
							Eigen::Vector4f v(substractCloud->at(kIndices.at(j)).normal_x, substractCloud->at(kIndices.at(j)).normal_y, substractCloud->at(kIndices.at(j)).normal_z, 0.0f);
							if(uIsFinite(v[0]) &&
								uIsFinite(v[1]) &&
								uIsFinite(v[2]))
							{
								float angle = pcl::getAngle3D(normal, v);
								if(angle > maxAngle)
								{
									k-=1;
								}
							}
							else
							{
								k-=1;
							}
						}
					}
					else
					{
						k=0;
					}
				}
				if(k < minNeighborsInRadius)
				{
					output->at(oi++) = i;
				}
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
		int normalKSearch,
		const Eigen::Vector4f & viewpoint)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return normalFiltering(cloud, indices, angleMax, normal, normalKSearch, viewpoint);
}
pcl::IndicesPtr normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return normalFiltering(cloud, indices, angleMax, normal, normalKSearch, viewpoint);
}



pcl::IndicesPtr normalFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint)
{
	pcl::IndicesPtr output(new std::vector<int>());

	if(cloud->size())
	{
		typedef pcl::search::KdTree<pcl::PointXYZ> KdTree;
		typedef KdTree::Ptr KdTreePtr;

		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
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

		ne.setKSearch(normalKSearch);
		if(viewpoint[0] != 0 || viewpoint[1] != 0 || viewpoint[2] != 0)
		{
			ne.setViewPoint(viewpoint[0], viewpoint[1], viewpoint[2]);
		}

		ne.compute (*cloud_normals);

		output->resize(cloud_normals->size());
		int oi = 0; // output iterator
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
		int normalKSearch,
		const Eigen::Vector4f & viewpoint)
{
	pcl::IndicesPtr output(new std::vector<int>());

	if(cloud->size())
	{
		typedef pcl::search::KdTree<pcl::PointXYZRGB> KdTree;
		typedef KdTree::Ptr KdTreePtr;

		pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
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

		ne.setKSearch (normalKSearch);
		if(viewpoint[0] != 0 || viewpoint[1] != 0 || viewpoint[2] != 0)
		{
			ne.setViewPoint(viewpoint[0], viewpoint[1], viewpoint[2]);
		}

		ne.compute (*cloud_normals);

		output->resize(cloud_normals->size());
		int oi = 0; // output iterator
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
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint)
{
	pcl::IndicesPtr output(new std::vector<int>());

	if(cloud->size())
	{
		int oi = 0; // output iterator
		if(indices->size())
		{
			output->resize(indices->size());
			for(unsigned int i=0; i<indices->size(); ++i)
			{
				Eigen::Vector4f v(cloud->at(indices->at(i)).normal_x, cloud->at(indices->at(i)).normal_y, cloud->at(indices->at(i)).normal_z, 0.0f);
				float angle = pcl::getAngle3D(normal, v);
				if(angle < angleMax)
				{
					output->at(oi++) = indices->size()!=0?indices->at(i):i;
				}
			}
		}
		else
		{
			output->resize(cloud->size());
			for(unsigned int i=0; i<cloud->size(); ++i)
			{
				Eigen::Vector4f v(cloud->at(i).normal_x, cloud->at(i).normal_y, cloud->at(i).normal_z, 0.0f);
				float angle = pcl::getAngle3D(normal, v);
				if(angle < angleMax)
				{
					output->at(oi++) = indices->size()!=0?indices->at(i):i;
				}
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
std::vector<pcl::IndicesPtr> extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	typedef pcl::search::KdTree<pcl::PointXYZRGBNormal> KdTree;
	typedef KdTree::Ptr KdTreePtr;

	KdTreePtr tree(new KdTree);
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
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

pcl::IndicesPtr extractIndices(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative)
{
	pcl::IndicesPtr output(new std::vector<int>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.filter(*output);
	return output;
}
pcl::IndicesPtr extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative)
{
	pcl::IndicesPtr output(new std::vector<int>);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud (cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.filter(*output);
	return output;
}
pcl::IndicesPtr extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative)
{
	pcl::IndicesPtr output(new std::vector<int>);
	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
	extract.setInputCloud (cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractIndices(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.setKeepOrganized(keepOrganized);
	extract.filter(*output);
	return output;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud (cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.setKeepOrganized(keepOrganized);
	extract.filter(*output);
	return output;
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
	extract.setInputCloud (cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.setKeepOrganized(keepOrganized);
	extract.filter(*output);
	return output;
}

pcl::IndicesPtr extractPlane(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float distanceThreshold,
		int maxIterations,
		pcl::ModelCoefficients * coefficientsOut)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return extractPlane(cloud, indices, distanceThreshold, maxIterations, coefficientsOut);
}

pcl::IndicesPtr extractPlane(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float distanceThreshold,
		int maxIterations,
		pcl::ModelCoefficients * coefficientsOut)
{
	// Extract plane
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	seg.setMaxIterations (maxIterations);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (distanceThreshold);

	seg.setInputCloud (cloud);
	if(indices->size())
	{
		seg.setIndices(indices);
	}
	seg.segment (*inliers, *coefficients);

	if(coefficientsOut)
	{
		*coefficientsOut = *coefficients;
	}

	return pcl::IndicesPtr(new std::vector<int>(inliers->indices));
}

}

}
