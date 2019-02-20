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

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_surface.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>

#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/extract_labeled_clusters.h>
#include <pcl/segmentation/impl/extract_labeled_clusters.hpp>
#include <pcl/filters/impl/extract_indices.hpp>

PCL_INSTANTIATE(EuclideanClusterExtraction, (pcl::PointXYZRGBNormal))
PCL_INSTANTIATE(extractEuclideanClusters, (pcl::PointXYZRGBNormal))
PCL_INSTANTIATE(extractEuclideanClusters_indices, (pcl::PointXYZRGBNormal))
PCL_INSTANTIATE(ExtractIndices, (pcl::PointNormal))

#endif

namespace rtabmap
{

namespace util3d
{

LaserScan commonFiltering(
		const LaserScan & scanIn,
		int downsamplingStep,
		float rangeMin,
		float rangeMax,
		float voxelSize,
		int normalK,
		float normalRadius,
		bool forceGroundNormalsUp)
{
	LaserScan scan = scanIn;
	UDEBUG("scan size=%d format=%d, step=%d, rangeMin=%f, rangeMax=%f, voxel=%f, normalK=%d, normalRadius=%f",
			scan.size(), (int)scan.format(), downsamplingStep, rangeMin, rangeMax, voxelSize, normalK, normalRadius);
	if(!scan.isEmpty())
	{
		// combined downsampling and range filtering step
		if(downsamplingStep<=1 || scan.size() <= downsamplingStep)
		{
			downsamplingStep = 1;
		}

		if(downsamplingStep > 1 || rangeMin > 0.0f || rangeMax > 0.0f)
		{
			cv::Mat tmp = cv::Mat(1, scan.size()/downsamplingStep, scan.dataType());
			bool is2d = scan.is2d();
			int oi = 0;
			float rangeMinSqrd = rangeMin * rangeMin;
			float rangeMaxSqrd = rangeMax * rangeMax;
			for(int i=0; i<scan.size()-downsamplingStep+1; i+=downsamplingStep)
			{
				const float * ptr = scan.data().ptr<float>(0, i);

				if(rangeMin>0.0f || rangeMax>0.0f)
				{
					float r;
					if(is2d)
					{
						r = ptr[0]*ptr[0] + ptr[1]*ptr[1];
					}
					else
					{
						r = ptr[0]*ptr[0] + ptr[1]*ptr[1] + ptr[2]*ptr[2];
					}

					if(rangeMin > 0.0f && r < rangeMinSqrd)
					{
						continue;
					}
					if(rangeMax > 0.0f && r > rangeMaxSqrd)
					{
						continue;
					}
				}

				cv::Mat(scan.data(), cv::Range::all(), cv::Range(i,i+1)).copyTo(cv::Mat(tmp, cv::Range::all(), cv::Range(oi,oi+1)));
				++oi;
			}
			int previousSize = scan.size();
			int scanMaxPtsTmp = scan.maxPoints();
			if(scan.angleIncrement() > 0.0f)
			{
				scan = LaserScan(
						cv::Mat(tmp, cv::Range::all(), cv::Range(0, oi)),
						scan.format(),
						rangeMin>0.0f&&rangeMin>scan.rangeMin()?rangeMin:scan.rangeMin(),
						rangeMax>0.0f&&rangeMax<scan.rangeMax()?rangeMax:scan.rangeMax(),
						scan.angleMin(),
						scan.angleMax(),
						scan.angleIncrement() * (float)downsamplingStep,
						scan.localTransform());
			}
			else
			{
				scan = LaserScan(
						cv::Mat(tmp, cv::Range::all(), cv::Range(0, oi)),
						scanMaxPtsTmp/downsamplingStep,
						rangeMax>0.0f&&rangeMax<scan.rangeMax()?rangeMax:scan.rangeMax(),
						scan.format(),
						scan.localTransform());
			}
			UDEBUG("Downsampling scan (step=%d): %d -> %d (scanMaxPts=%d->%d)", downsamplingStep, previousSize, scan.size(), scanMaxPtsTmp, scan.maxPoints());
		}

		if(scan.size() && (voxelSize > 0.0f || ((normalK > 0 || normalRadius>0.0f) && !scan.hasNormals())))
		{
			// convert to compatible PCL format and filter it
			if(scan.hasRGB())
			{
				UASSERT(!scan.is2d());
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = laserScanToPointCloudRGB(scan);
				if(cloud->size())
				{
					int scanMaxPts = scan.maxPoints();
					if(voxelSize > 0.0f)
					{
						cloud = voxelize(cloud, voxelSize);
						float ratio = float(cloud->size()) / scan.size();
						scanMaxPts = int(float(scanMaxPts) * ratio);
						UDEBUG("Voxel filtering scan (voxel=%f m): %d -> %d (scanMaxPts=%d->%d)", voxelSize, scan.size(), cloud->size(), scan.maxPoints(), scanMaxPts);
					}
					if(cloud->size() && (normalK > 0 || normalRadius>0.0f))
					{
						pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, normalK, normalRadius);
						scan = LaserScan(laserScanFromPointCloud(*cloud, *normals), scanMaxPts, scan.rangeMax(), LaserScan::kXYZRGBNormal, scan.localTransform());
						UDEBUG("Normals computed (k=%d radius=%f)", normalK, normalRadius);
					}
					else
					{
						if(scan.hasNormals())
						{
							UWARN("Voxel filter is applied, but normal parameters are not set and input scan has normals. The returned scan has no normals.");
						}
						scan = LaserScan(laserScanFromPointCloud(*cloud), scanMaxPts, scan.rangeMax(), LaserScan::kXYZRGB, scan.localTransform());
					}
				}
			}
			else if(scan.hasIntensity())
			{
				pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = laserScanToPointCloudI(scan);
				if(cloud->size())
				{
					int scanMaxPts = scan.maxPoints();
					if(voxelSize > 0.0f)
					{
						cloud = voxelize(cloud, voxelSize);
						float ratio = float(cloud->size()) / scan.size();
						scanMaxPts = int(float(scanMaxPts) * ratio);
						UDEBUG("Voxel filtering scan (voxel=%f m): %d -> %d (scanMaxPts=%d->%d)", voxelSize, scan.size(), cloud->size(), scan.maxPoints(), scanMaxPts);
					}
					if(cloud->size() && (normalK > 0 || normalRadius>0.0f))
					{
						pcl::PointCloud<pcl::Normal>::Ptr normals;
						if(scan.is2d())
						{
							normals = util3d::computeNormals2D(cloud, normalK, normalRadius);
							if(voxelSize == 0.0f && scan.angleIncrement() > 0.0f)
							{
								scan = LaserScan(laserScan2dFromPointCloud(*cloud, *normals), LaserScan::kXYINormal, scan.rangeMin(), scan.rangeMax(), scan.angleMin(), scan.angleMax(), scan.angleIncrement(), scan.localTransform());
							}
							else
							{
								scan = LaserScan(laserScan2dFromPointCloud(*cloud, *normals), scanMaxPts, scan.rangeMax(), LaserScan::kXYINormal, scan.localTransform());
							}
						}
						else
						{
							normals = util3d::computeNormals(cloud, normalK, normalRadius);
							scan = LaserScan(laserScanFromPointCloud(*cloud, *normals), scanMaxPts, scan.rangeMax(), LaserScan::kXYZINormal, scan.localTransform());
						}
						UDEBUG("Normals computed (k=%d radius=%f)", normalK, normalRadius);
					}
					else
					{
						if(scan.hasNormals())
						{
							UWARN("Voxel filter i applied, but normal parameters are not set and input scan has normals. The returned scan has no normals.");
						}
						if(scan.is2d())
						{
							scan = LaserScan(laserScan2dFromPointCloud(*cloud), scanMaxPts, scan.rangeMax(), LaserScan::kXYI, scan.localTransform());
						}
						else
						{
							scan = LaserScan(laserScanFromPointCloud(*cloud), scanMaxPts, scan.rangeMax(), LaserScan::kXYZI, scan.localTransform());
						}
					}
				}
			}
			else
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = laserScanToPointCloud(scan);
				if(cloud->size())
				{
					int scanMaxPts = scan.maxPoints();
					if(voxelSize > 0.0f)
					{
						cloud = voxelize(cloud, voxelSize);
						float ratio = float(cloud->size()) / scan.size();
						scanMaxPts = int(float(scanMaxPts) * ratio);
						UDEBUG("Voxel filtering scan (voxel=%f m): %d -> %d (scanMaxPts=%d->%d)", voxelSize, scan.size(), cloud->size(), scan.maxPoints(), scanMaxPts);
					}
					if(cloud->size() && (normalK > 0 || normalRadius>0.0f))
					{
						pcl::PointCloud<pcl::Normal>::Ptr normals;
						if(scan.is2d())
						{
							normals = util3d::computeNormals2D(cloud, normalK, normalRadius);
							if(voxelSize == 0.0f && scan.angleIncrement() > 0.0f)
							{
								scan = LaserScan(laserScan2dFromPointCloud(*cloud, *normals), LaserScan::kXYNormal, scan.rangeMin(), scan.rangeMax(), scan.angleMin(), scan.angleMax(), scan.angleIncrement(), scan.localTransform());
							}
							else
							{
								scan = LaserScan(laserScan2dFromPointCloud(*cloud, *normals), scanMaxPts, scan.rangeMax(), LaserScan::kXYNormal, scan.localTransform());
							}
						}
						else
						{
							normals = util3d::computeNormals(cloud, normalK, normalRadius);
							scan = LaserScan(laserScanFromPointCloud(*cloud, *normals), scanMaxPts, scan.rangeMax(), LaserScan::kXYZNormal, scan.localTransform());
						}
						UDEBUG("Normals computed (k=%d radius=%f)", normalK, normalRadius);
					}
					else
					{
						if(scan.hasNormals())
						{
							UWARN("Voxel filter i applied, but normal parameters are not set and input scan has normals. The returned scan has no normals.");
						}
						if(scan.is2d())
						{
							scan = LaserScan(laserScan2dFromPointCloud(*cloud), scanMaxPts, scan.rangeMax(), LaserScan::kXY, scan.localTransform());
						}
						else
						{
							scan = LaserScan(laserScanFromPointCloud(*cloud), scanMaxPts, scan.rangeMax(), LaserScan::kXYZ, scan.localTransform());
						}
					}
				}
			}
		}

		if(scan.size() && !scan.is2d() && scan.hasNormals() && forceGroundNormalsUp)
		{
			scan = util3d::adjustNormalsToViewPoint(scan, Eigen::Vector3f(0,0,0), forceGroundNormalsUp);
		}
	}
	return scan;
}

LaserScan rangeFiltering(
		const LaserScan & scan,
		float rangeMin,
		float rangeMax)
{
	UASSERT(rangeMin >=0.0f && rangeMax>=0.0f);
	if(!scan.isEmpty())
	{
		if(rangeMin > 0.0f || rangeMax > 0.0f)
		{
			cv::Mat output = cv::Mat(1, scan.size(), scan.dataType());
			bool is2d = scan.is2d();
			int oi = 0;
			float rangeMinSqrd = rangeMin * rangeMin;
			float rangeMaxSqrd = rangeMax * rangeMax;
			for(int i=0; i<scan.size(); ++i)
			{
				const float * ptr = scan.data().ptr<float>(0, i);
				float r;
				if(is2d)
				{
					r = ptr[0]*ptr[0] + ptr[1]*ptr[1];
				}
				else
				{
					r = ptr[0]*ptr[0] + ptr[1]*ptr[1] + ptr[2]*ptr[2];
				}

				if(rangeMin > 0.0f && r < rangeMinSqrd)
				{
					continue;
				}
				if(rangeMax > 0.0f && r > rangeMaxSqrd)
				{
					continue;
				}

				cv::Mat(scan.data(), cv::Range::all(), cv::Range(i,i+1)).copyTo(cv::Mat(output, cv::Range::all(), cv::Range(oi,oi+1)));
				++oi;
			}
			if(scan.angleIncrement() > 0.0f)
			{
				return LaserScan(cv::Mat(output, cv::Range::all(), cv::Range(0, oi)), scan.format(), scan.rangeMin(), scan.rangeMax(), scan.angleMin(), scan.angleMax(), scan.angleIncrement(), scan.localTransform());
			}
			return LaserScan(cv::Mat(output, cv::Range::all(), cv::Range(0, oi)), scan.maxPoints(), scan.rangeMax(), scan.format(), scan.localTransform());
		}
	}

	return scan;
}

LaserScan downsample(
		const LaserScan & scan,
		int step)
{
	UASSERT(step > 0);
	if(step <= 1 || scan.size() <= step)
	{
		// no sampling
		return scan;
	}
	else
	{
		int finalSize = scan.size()/step;
		cv::Mat output = cv::Mat(1, finalSize, scan.dataType());
		int oi = 0;
		for(int i=0; i<scan.size()-step+1; i+=step)
		{
			cv::Mat(scan.data(), cv::Range::all(), cv::Range(i,i+1)).copyTo(cv::Mat(output, cv::Range::all(), cv::Range(oi,oi+1)));
			++oi;
		}
		if(scan.angleIncrement() > 0.0f)
		{
			return LaserScan(output, scan.format(), scan.rangeMin(), scan.rangeMax(), scan.angleMin(), scan.angleMax(), scan.angleIncrement()*step, scan.localTransform());
		}
		return LaserScan(output, scan.maxPoints()/step, scan.rangeMax(), scan.format(), scan.localTransform());
	}
}
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr downsampleImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		int step)
{
	UASSERT(step > 0);
	typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
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
pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, int step)
{
	return downsampleImpl<pcl::PointXYZ>(cloud, step);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, int step)
{
	return downsampleImpl<pcl::PointXYZRGB>(cloud, step);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, int step)
{
	return downsampleImpl<pcl::PointXYZI>(cloud, step);
}
pcl::PointCloud<pcl::PointNormal>::Ptr downsample(const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, int step)
{
	return downsampleImpl<pcl::PointNormal>(cloud, step);
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, int step)
{
	return downsampleImpl<pcl::PointXYZRGBNormal>(cloud, step);
}
pcl::PointCloud<pcl::PointXYZINormal>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud, int step)
{
	return downsampleImpl<pcl::PointXYZINormal>(cloud, step);
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr voxelizeImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize)
{
	UASSERT(voxelSize > 0.0f);
	typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	if((cloud->is_dense && cloud->size()) || (!cloud->is_dense && indices->size()))
	{
		pcl::VoxelGrid<PointT> filter;
		filter.setLeafSize(voxelSize, voxelSize, voxelSize);
		filter.setInputCloud(cloud);
		if(indices->size())
		{
			filter.setIndices(indices);
		}
		filter.filter(*output);
	}
	else if(cloud->size() && !cloud->is_dense && indices->size() == 0)
	{
		UWARN("Cannot voxelize a not dense (organized) cloud with empty indices! (input=%d pts). Returning empty cloud!", (int)cloud->size());
	}
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::IndicesPtr & indices, float voxelSize)
{
	return voxelizeImpl<pcl::PointXYZ>(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointNormal>::Ptr voxelize(const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, float voxelSize)
{
	return voxelizeImpl<pcl::PointNormal>(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const pcl::IndicesPtr & indices, float voxelSize)
{
	return voxelizeImpl<pcl::PointXYZRGB>(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, float voxelSize)
{
	return voxelizeImpl<pcl::PointXYZRGBNormal>(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, const pcl::IndicesPtr & indices, float voxelSize)
{
	return voxelizeImpl<pcl::PointXYZI>(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointXYZINormal>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud, const pcl::IndicesPtr & indices, float voxelSize)
{
	return voxelizeImpl<pcl::PointXYZINormal>(cloud, indices, voxelSize);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float voxelSize)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return voxelize(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointNormal>::Ptr voxelize(const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, float voxelSize)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return voxelize(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, float voxelSize)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return voxelize(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, float voxelSize)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return voxelize(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, float voxelSize)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return voxelize(cloud, indices, voxelSize);
}
pcl::PointCloud<pcl::PointXYZINormal>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud, float voxelSize)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return voxelize(cloud, indices, voxelSize);
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr randomSamplingImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud, int samples)
{
	UASSERT(samples > 0);
	typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	pcl::RandomSample<PointT> filter;
	filter.setSample(samples);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr randomSampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, int samples)
{
	return randomSamplingImpl<pcl::PointXYZ>(cloud, samples);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomSampling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, int samples)
{
	return randomSamplingImpl<pcl::PointXYZRGB>(cloud, samples);
}

template<typename PointT>
pcl::IndicesPtr passThroughImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative)
{
	UASSERT_MSG(max > min, uFormat("cloud=%d, max=%f min=%f axis=%s", (int)cloud->size(), max, min, axis.c_str()).c_str());
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	pcl::IndicesPtr output(new std::vector<int>);
	pcl::PassThrough<PointT> filter;
	filter.setNegative(negative);
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.setIndices(indices);
	filter.filter(*output);
	return output;
}

pcl::IndicesPtr passThrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::IndicesPtr & indices, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointXYZ>(cloud, indices, axis, min, max, negative);
}
pcl::IndicesPtr passThrough(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const pcl::IndicesPtr & indices, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointXYZRGB>(cloud, indices, axis, min, max, negative);
}
pcl::IndicesPtr passThrough(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, const pcl::IndicesPtr & indices, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointXYZI>(cloud, indices, axis, min, max, negative);
}
pcl::IndicesPtr passThrough(const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointNormal>(cloud, indices, axis, min, max, negative);
}
pcl::IndicesPtr passThrough(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointXYZRGBNormal>(cloud, indices, axis, min, max, negative);
}
pcl::IndicesPtr passThrough(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud, const pcl::IndicesPtr & indices, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointXYZINormal>(cloud, indices, axis, min, max, negative);
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr passThroughImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative)
{
	UASSERT_MSG(max > min, uFormat("cloud=%d, max=%f min=%f axis=%s", (int)cloud->size(), max, min, axis.c_str()).c_str());
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	pcl::PassThrough<PointT> filter;
	filter.setNegative(negative);
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointXYZ>(cloud, axis, min ,max, negative);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThrough(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointXYZRGB>(cloud, axis, min ,max, negative);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr passThrough(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointXYZI>(cloud, axis, min ,max, negative);
}
pcl::PointCloud<pcl::PointNormal>::Ptr passThrough(const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointNormal>(cloud, axis, min ,max, negative);
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr passThrough(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointXYZRGBNormal>(cloud, axis, min ,max, negative);
}
pcl::PointCloud<pcl::PointXYZINormal>::Ptr passThrough(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud, const std::string & axis, float min, float max, bool negative)
{
	return passThroughImpl<pcl::PointXYZINormal>(cloud, axis, min ,max, negative);
}

template<typename PointT>
pcl::IndicesPtr cropBoxImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform,
		bool negative)
{
	UASSERT(min[0] < max[0] && min[1] < max[1] && min[2] < max[2]);

	pcl::IndicesPtr output(new std::vector<int>);
	pcl::CropBox<PointT> filter;
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

pcl::IndicesPtr cropBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::IndicesPtr & indices, const Eigen::Vector4f & min, const Eigen::Vector4f & max, const Transform & transform, bool negative)
{
	return cropBoxImpl<pcl::PointXYZ>(cloud, indices, min, max, transform, negative);
}
pcl::IndicesPtr cropBox(const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, const Eigen::Vector4f & min, const Eigen::Vector4f & max, const Transform & transform, bool negative)
{
	return cropBoxImpl<pcl::PointNormal>(cloud, indices, min, max, transform, negative);
}
pcl::IndicesPtr cropBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const pcl::IndicesPtr & indices, const Eigen::Vector4f & min, const Eigen::Vector4f & max, const Transform & transform, bool negative)
{
	return cropBoxImpl<pcl::PointXYZRGB>(cloud, indices, min, max, transform, negative);
}
pcl::IndicesPtr cropBox(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, const Eigen::Vector4f & min, const Eigen::Vector4f & max, const Transform & transform, bool negative)
{
	return cropBoxImpl<pcl::PointXYZRGBNormal>(cloud, indices, min, max, transform, negative);
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr cropBoxImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform,
		bool negative)
{
	UASSERT(min[0] < max[0] && min[1] < max[1] && min[2] < max[2]);

	typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	pcl::CropBox<PointT> filter;
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
pcl::PointCloud<pcl::PointXYZ>::Ptr cropBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const Eigen::Vector4f & min, const Eigen::Vector4f & max, const Transform & transform, bool negative)
{
	return cropBoxImpl<pcl::PointXYZ>(cloud, min, max, transform, negative);
}
pcl::PointCloud<pcl::PointNormal>::Ptr cropBox(const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, const Eigen::Vector4f & min, const Eigen::Vector4f & max, const Transform & transform, bool negative)
{
	return cropBoxImpl<pcl::PointNormal>(cloud, min, max, transform, negative);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const Eigen::Vector4f & min, const Eigen::Vector4f & max, const Transform & transform, bool negative)
{
	return cropBoxImpl<pcl::PointXYZRGB>(cloud, min, max, transform, negative);
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cropBox(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, const Eigen::Vector4f & min, const Eigen::Vector4f & max, const Transform & transform, bool negative)
{
	return cropBoxImpl<pcl::PointXYZRGBNormal>(cloud, min, max, transform, negative);
}

template<typename PointT>
pcl::IndicesPtr frustumFilteringImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
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
	pcl::FrustumCulling<PointT> fc;
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
pcl::IndicesPtr frustumFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::IndicesPtr & indices, const Transform & cameraPose, float horizontalFOV, float verticalFOV, float nearClipPlaneDistance, float farClipPlaneDistance, bool negative)
{
	return frustumFilteringImpl<pcl::PointXYZ>(cloud, indices, cameraPose, horizontalFOV, verticalFOV, nearClipPlaneDistance, farClipPlaneDistance, negative);
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr frustumFilteringImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
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

	typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	pcl::FrustumCulling<PointT> fc;
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
pcl::PointCloud<pcl::PointXYZ>::Ptr frustumFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const Transform & cameraPose, float horizontalFOV, float verticalFOV, float nearClipPlaneDistance, float farClipPlaneDistance, bool negative)
{
	return frustumFilteringImpl<pcl::PointXYZ>(cloud, cameraPose, horizontalFOV, verticalFOV, nearClipPlaneDistance, farClipPlaneDistance, negative);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr frustumFiltering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const Transform & cameraPose, float horizontalFOV, float verticalFOV, float nearClipPlaneDistance, float farClipPlaneDistance, bool negative)
{
	return frustumFilteringImpl<pcl::PointXYZRGB>(cloud, cameraPose, horizontalFOV, verticalFOV, nearClipPlaneDistance, farClipPlaneDistance, negative);
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr removeNaNFromPointCloudImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud)
{
	typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *output, indices);
	return output;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr removeNaNFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
	return removeNaNFromPointCloudImpl<pcl::PointXYZ>(cloud);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNaNFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
{
	return removeNaNFromPointCloudImpl<pcl::PointXYZRGB>(cloud);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr removeNaNFromPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
	return removeNaNFromPointCloudImpl<pcl::PointXYZI>(cloud);
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr removeNaNNormalsFromPointCloudImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud)
{
	typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	std::vector<int> indices;
	pcl::removeNaNNormalsFromPointCloud(*cloud, *output, indices);
	return output;
}
pcl::PointCloud<pcl::PointNormal>::Ptr removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud)
{
	return removeNaNNormalsFromPointCloudImpl<pcl::PointNormal>(cloud);
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud)
{
	return removeNaNNormalsFromPointCloudImpl<pcl::PointXYZRGBNormal>(cloud);
}
pcl::PointCloud<pcl::PointXYZINormal>::Ptr removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud)
{
	return removeNaNNormalsFromPointCloudImpl<pcl::PointXYZINormal>(cloud);
}


pcl::IndicesPtr radiusFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float radiusSearch, int minNeighborsInRadius)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return radiusFiltering(cloud, indices, radiusSearch, minNeighborsInRadius);
}
pcl::IndicesPtr radiusFiltering(const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, float radiusSearch, int minNeighborsInRadius)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return radiusFiltering(cloud, indices, radiusSearch, minNeighborsInRadius);
}
pcl::IndicesPtr radiusFiltering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, float radiusSearch, int minNeighborsInRadius)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return radiusFiltering(cloud, indices, radiusSearch, minNeighborsInRadius);
}
pcl::IndicesPtr radiusFiltering(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, float radiusSearch, int minNeighborsInRadius)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return radiusFiltering(cloud, indices, radiusSearch, minNeighborsInRadius);
}

template<typename PointT>
pcl::IndicesPtr radiusFilteringImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius)
{
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>(false));

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

pcl::IndicesPtr radiusFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::IndicesPtr & indices, float radiusSearch, int minNeighborsInRadius)
{
	return radiusFilteringImpl<pcl::PointXYZ>(cloud, indices, radiusSearch, minNeighborsInRadius);
}
pcl::IndicesPtr radiusFiltering(const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, float radiusSearch, int minNeighborsInRadius)
{
	return radiusFilteringImpl<pcl::PointNormal>(cloud, indices, radiusSearch, minNeighborsInRadius);
}
pcl::IndicesPtr radiusFiltering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const pcl::IndicesPtr & indices, float radiusSearch, int minNeighborsInRadius)
{
	return radiusFilteringImpl<pcl::PointXYZRGB>(cloud, indices, radiusSearch, minNeighborsInRadius);
}
pcl::IndicesPtr radiusFiltering(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, float radiusSearch, int minNeighborsInRadius)
{
	return radiusFilteringImpl<pcl::PointXYZRGBNormal>(cloud, indices, radiusSearch, minNeighborsInRadius);
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

template<typename PointT>
pcl::IndicesPtr subtractFilteringImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const typename pcl::PointCloud<PointT>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		int minNeighborsInRadius)
{
	UASSERT(minNeighborsInRadius > 0);
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>(false));

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
pcl::IndicesPtr subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		int minNeighborsInRadius)
{
	return subtractFilteringImpl<pcl::PointXYZRGB>(cloud, indices, substractCloud, substractIndices, radiusSearch, minNeighborsInRadius);
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

template<typename PointT>
pcl::IndicesPtr subtractFilteringImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const typename pcl::PointCloud<PointT>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		float maxAngle,
		int minNeighborsInRadius)
{
	UASSERT(minNeighborsInRadius > 0);
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>(false));

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

pcl::IndicesPtr subtractFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointNormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		float maxAngle,
		int minNeighborsInRadius)
{
	return subtractFilteringImpl<pcl::PointNormal>(cloud, indices, substractCloud, substractIndices, radiusSearch, maxAngle, minNeighborsInRadius);
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
	return subtractFilteringImpl<pcl::PointXYZRGBNormal>(cloud, indices, substractCloud, substractIndices, radiusSearch, maxAngle, minNeighborsInRadius);
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


template<typename PointT>
pcl::IndicesPtr normalFilteringImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint)
{
	pcl::IndicesPtr output(new std::vector<int>());

	if(cloud->size())
	{
		typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>(false));

		pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
		ne.setInputCloud (cloud);
		if(indices->size())
		{
			ne.setIndices(indices);
		}

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
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint)

{
	return normalFilteringImpl<pcl::PointXYZ>(cloud, indices, angleMax, normal, normalKSearch, viewpoint);
}
pcl::IndicesPtr normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint)
{
	return normalFilteringImpl<pcl::PointXYZRGB>(cloud, indices, angleMax, normal, normalKSearch, viewpoint);
}

template<typename PointT>
pcl::IndicesPtr normalFilteringImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal)
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
pcl::IndicesPtr normalFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint)
{
	return normalFilteringImpl<pcl::PointNormal>(cloud, indices, angleMax, normal);
}
pcl::IndicesPtr normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint)
{
	return normalFilteringImpl<pcl::PointXYZRGBNormal>(cloud, indices, angleMax, normal);
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

template<typename PointT>
std::vector<pcl::IndicesPtr> extractClustersImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>(true));
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
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	return extractClustersImpl<pcl::PointXYZ>(cloud, indices, clusterTolerance, minClusterSize, maxClusterSize, biggestClusterIndex);
}
std::vector<pcl::IndicesPtr> extractClusters(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	return extractClustersImpl<pcl::PointNormal>(cloud, indices, clusterTolerance, minClusterSize, maxClusterSize, biggestClusterIndex);
}
std::vector<pcl::IndicesPtr> extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	return extractClustersImpl<pcl::PointXYZRGB>(cloud, indices, clusterTolerance, minClusterSize, maxClusterSize, biggestClusterIndex);
}
std::vector<pcl::IndicesPtr> extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize,
		int * biggestClusterIndex)
{
	return extractClustersImpl<pcl::PointXYZRGBNormal>(cloud, indices, clusterTolerance, minClusterSize, maxClusterSize, biggestClusterIndex);
}

template<typename PointT>
pcl::IndicesPtr extractIndicesImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative)
{
	pcl::IndicesPtr output(new std::vector<int>);
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.filter(*output);
	return output;
}

pcl::IndicesPtr extractIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::IndicesPtr & indices, bool negative)
{
	return extractIndicesImpl<pcl::PointXYZ>(cloud, indices, negative);
}
pcl::IndicesPtr extractIndices(const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, bool negative)
{
	return extractIndicesImpl<pcl::PointNormal>(cloud, indices, negative);
}
pcl::IndicesPtr extractIndices(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const pcl::IndicesPtr & indices, bool negative)
{
	return extractIndicesImpl<pcl::PointXYZRGB>(cloud, indices, negative);
}
pcl::IndicesPtr extractIndices(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, bool negative)
{
	return extractIndicesImpl<pcl::PointXYZRGBNormal>(cloud, indices, negative);
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr extractIndicesImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized)
{
	typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.setKeepOrganized(keepOrganized);
	extract.filter(*output);
	return output;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr extractIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::IndicesPtr & indices, bool negative, bool keepOrganized)
{
	return extractIndicesImpl<pcl::PointXYZ>(cloud, indices, negative, keepOrganized);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractIndices(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const pcl::IndicesPtr & indices, bool negative, bool keepOrganized)
{
	return extractIndicesImpl<pcl::PointXYZRGB>(cloud, indices, negative, keepOrganized);
}
// PCL default lacks of pcl::PointNormal type support
//pcl::PointCloud<pcl::PointNormal>::Ptr extractIndices(const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, bool negative, bool keepOrganized)
//{
//	return extractIndicesImpl<pcl::PointNormal>(cloud, indices, negative, keepOrganized);
//}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr extractIndices(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, const pcl::IndicesPtr & indices, bool negative, bool keepOrganized)
{
	return extractIndicesImpl<pcl::PointXYZRGBNormal>(cloud, indices, negative, keepOrganized);
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
