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

#ifndef UTIL3D_FILTERING_H_
#define UTIL3D_FILTERING_H_

#include <rtabmap/core/RtabmapExp.h>
#include <rtabmap/core/Transform.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <rtabmap/core/LaserScan.h>

namespace rtabmap
{

namespace util3d
{

/**
 * Do some filtering approaches and try to
 * avoid converting between pcl and opencv and to avoid not needed
 * operations like computing normals while the scan has already
 * normals and voxel filtering is not used.
 */
LaserScan RTABMAP_EXP commonFiltering(
		const LaserScan & scan,
		int downsamplingStep,
		float rangeMin = 0.0f,
		float rangeMax = 0.0f,
		float voxelSize = 0.0f,
		int normalK = 0,
		float normalRadius = 0.0f,
		float groundNormalsUp = 0.0f);
RTABMAP_DEPRECATED(LaserScan RTABMAP_EXP commonFiltering(
		const LaserScan & scan,
		int downsamplingStep,
		float rangeMin,
		float rangeMax,
		float voxelSize,
		int normalK,
		float normalRadius,
		bool forceGroundNormalsUp), "Use version with groundNormalsUp as float. For forceGroundNormalsUp=true, set groundNormalsUp=0.8, otherwise set groundNormalsUp=0.0.");

LaserScan RTABMAP_EXP rangeFiltering(
		const LaserScan & scan,
		float rangeMin,
		float rangeMax);

LaserScan RTABMAP_EXP downsample(
		const LaserScan & cloud,
		int step);
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP downsample(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int step);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP downsample(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int step);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_EXP downsample(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		int step);
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP downsample(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		int step);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP downsample(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		int step);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_EXP downsample(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		int step);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize);
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float voxelSize);
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		float voxelSize);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float voxelSize);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		float voxelSize);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		float voxelSize);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		float voxelSize);

inline pcl::PointCloud<pcl::PointXYZ>::Ptr uniformSampling(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float voxelSize)
{
	return voxelize(cloud, voxelSize);
}
inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr uniformSampling(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float voxelSize)
{
	return voxelize(cloud, voxelSize);
}
inline pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr uniformSampling(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		float voxelSize)
{
	return voxelize(cloud, voxelSize);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP randomSampling(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int samples);
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP randomSampling(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		int samples);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP randomSampling(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int samples);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP randomSampling(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		int samples);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_EXP randomSampling(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		int samples);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_EXP randomSampling(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		int samples);


pcl::IndicesPtr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
pcl::IndicesPtr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
pcl::IndicesPtr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
pcl::IndicesPtr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
pcl::IndicesPtr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
pcl::IndicesPtr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max,
		bool negative = false);

pcl::IndicesPtr RTABMAP_EXP cropBox(
		const pcl::PCLPointCloud2::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::IndicesPtr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::IndicesPtr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::IndicesPtr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::IndicesPtr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::IndicesPtr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::IndicesPtr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP cropBox(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const Eigen::Vector4f & min,
		const Eigen::Vector4f & max,
		const Transform & transform = Transform::getIdentity(),
		bool negative = false);

//Note: This assumes a coordinate system where X is forward, * Y is up, and Z is right.
pcl::IndicesPtr RTABMAP_EXP frustumFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & cameraPose,
		float horizontalFOV, // in degrees, xfov = atan((image_width/2)/fx)*2
		float verticalFOV,   // in degrees, yfov = atan((image_height/2)/fy)*2
		float nearClipPlaneDistance,
		float farClipPlaneDistance,
		bool negative = false);
//Note: This assumes a coordinate system where X is forward, * Y is up, and Z is right.
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP frustumFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & cameraPose,
		float horizontalFOV, // in degrees, xfov = atan((image_width/2)/fx)*2
		float verticalFOV,   // in degrees, yfov = atan((image_height/2)/fy)*2
		float nearClipPlaneDistance,
		float farClipPlaneDistance,
		bool negative = false);
//Note: This assumes a coordinate system where X is forward, * Y is up, and Z is right.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP frustumFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & cameraPose,
		float horizontalFOV, // in degrees, xfov = atan((image_width/2)/fx)*2
		float verticalFOV,   // in degrees, yfov = atan((image_height/2)/fy)*2
		float nearClipPlaneDistance,
		float farClipPlaneDistance,
		bool negative = false);


pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_EXP removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud);
pcl::PCLPointCloud2::Ptr RTABMAP_EXP removeNaNFromPointCloud(
		const pcl::PCLPointCloud2::Ptr & cloud);


pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_EXP removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud);

/**
 * For convenience.
 */

pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);

/**
 * @brief Wrapper of the pcl::RadiusOutlierRemoval class.
 *
 * Points in the cloud which have less than a minimum of neighbors in the
 * specified radius are filtered.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearch the radius in meter.
 * @param minNeighborsInRadius the minimum of neighbors to keep the point.
 * @return the indices of the points satisfying the parameters.
 */

pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);
pcl::IndicesPtr RTABMAP_EXP radiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);

/* for convenience */
pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);

/**
 * @brief Filter points based on distance from their viewpoint.
 *
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param viewpointIndices should be same size than the input cloud, it tells the viewpoint index in viewpoints for each point.
 * @param viewpoints the viewpoints.
 * @param factor will determine the search radius based on the distance from a point and its viewpoint. Setting it higher will filter points farther from accurate points (but processing time will be also higher).
 * @param neighborScale will scale the search radius of neighbors found around a point. Setting it higher will accept more noisy points close to accurate points (but processing time will be also higher).
 * @return the indices of the points satisfying the parameters.
 */

pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);
pcl::IndicesPtr RTABMAP_EXP proportionalRadiusFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const std::vector<int> & viewpointIndices,
		const std::map<int, Transform> & viewpoints,
		float factor=0.01f,
		float neighborScale=2.0f);

/**
 * For convenience.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & substractCloud,
		float radiusSearch,
		int minNeighborsInRadius = 1);

/**
 * Subtract a cloud from another one using radius filtering.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param cloud the input cloud to subtract.
 * @param indices the input indices of the subtracted cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearch the radius in meter.
 * @return the indices of the points satisfying the parameters.
 */
pcl::IndicesPtr RTABMAP_EXP subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		int minNeighborsInRadius = 1);

/**
 * For convenience.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP subtractFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointNormal>::Ptr & substractCloud,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);

/**
 * Subtract a cloud from another one using radius filtering.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param cloud the input cloud to subtract.
 * @param indices the input indices of the subtracted cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearch the radius in meter.
 * @return the indices of the points satisfying the parameters.
 */
pcl::IndicesPtr RTABMAP_EXP subtractFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointNormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);

/**
 * For convenience.
 */
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_EXP subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & substractCloud,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & substractCloud,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);

/**
 * Subtract a cloud from another one using radius filtering.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param cloud the input cloud to subtract.
 * @param indices the input indices of the subtracted cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearch the radius in meter.
 * @return the indices of the points satisfying the parameters.
 */
pcl::IndicesPtr RTABMAP_EXP subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);
pcl::IndicesPtr RTABMAP_EXP subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearch,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1);

/**
 * Subtract a cloud from another one using radius filtering.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param cloud the input cloud to subtract.
 * @param indices the input indices of the subtracted cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearchRatio the ratio used to compute the radius at different distances (e.g., a ratio of 0.1 at 4 m results in a radius of 4 cm).
 * @return the indices of the points satisfying the parameters.
 */
pcl::IndicesPtr RTABMAP_EXP subtractAdaptiveFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearchRatio = 0.01,
		int minNeighborsInRadius = 1,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0));

/**
 * Subtract a cloud from another one using radius filtering.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param cloud the input cloud to subtract.
 * @param indices the input indices of the subtracted cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearchRatio the ratio used to compute the radius at different distances (e.g., a ratio of 0.01 at 4 m results in a radius of 4 cm).
 * @return the indices of the points satisfying the parameters.
 */
pcl::IndicesPtr RTABMAP_EXP subtractAdaptiveFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & substractCloud,
		const pcl::IndicesPtr & substractIndices,
		float radiusSearchRatio = 0.01,
		float maxAngle = M_PI/4.0f,
		int minNeighborsInRadius = 1,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0));


/**
 * For convenience.
 */

pcl::IndicesPtr RTABMAP_EXP normalFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_EXP normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);

/**
 * @brief Given a normal and a maximum angle error, keep all points of the cloud
 * respecting this normal.
 *
 * The normals are computed using the radius search parameter (pcl::NormalEstimation class is used for this), then
 * for each normal, the corresponding point is filtered if the
 * angle (using pcl::getAngle3D()) with the normal specified by the user is larger than the maximum
 * angle specified by the user.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to process, if empty, all points in the cloud are processed.
 * @param angleMax the maximum angle.
 * @param normal the normal to which each point's normal is compared.
 * @param normalKSearch number of neighbor points used for normal estimation (see pcl::NormalEstimation).
 * @param viewpoint from which viewpoint the normals should be estimated (see pcl::NormalEstimation).
 * @return the indices of the points which respect the normal constraint.
 */
pcl::IndicesPtr RTABMAP_EXP normalFiltering(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_EXP normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_EXP normalFiltering(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_EXP normalFiltering(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_EXP normalFiltering(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);
pcl::IndicesPtr RTABMAP_EXP normalFiltering(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp = 0.0f);

/**
 * For convenience.
 */
std::vector<pcl::IndicesPtr> RTABMAP_EXP extractClusters(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_EXP extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);

/**
 * @brief Wrapper of the pcl::EuclideanClusterExtraction class.
 *
 * Extract all clusters from a point cloud given a maximum cluster distance tolerance.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to process, if empty, all points in the cloud are processed.
 * @param clusterTolerance the cluster distance tolerance (see pcl::EuclideanClusterExtraction).
 * @param minClusterSize minimum size of the clusters to return (see pcl::EuclideanClusterExtraction).
 * @param maxClusterSize maximum size of the clusters to return (see pcl::EuclideanClusterExtraction).
 * @param biggestClusterIndex the index of the biggest cluster, if the clusters are empty, a negative index is set.
 * @return the indices of each cluster found.
 */
std::vector<pcl::IndicesPtr> RTABMAP_EXP extractClusters(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_EXP extractClusters(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_EXP extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_EXP extractClusters(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_EXP extractClusters(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);
std::vector<pcl::IndicesPtr> RTABMAP_EXP extractClusters(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);

pcl::IndicesPtr RTABMAP_EXP extractIndices(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
pcl::IndicesPtr RTABMAP_EXP extractIndices(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
pcl::IndicesPtr RTABMAP_EXP extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
pcl::IndicesPtr RTABMAP_EXP extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
pcl::IndicesPtr RTABMAP_EXP extractIndices(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);
pcl::IndicesPtr RTABMAP_EXP extractIndices(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP extractIndices(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
// PCL default lacks of pcl::PointNormal type support
//pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP extractIndices(
//		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
//		const pcl::IndicesPtr & indices,
//		bool negative,
//		bool keepOrganized);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP extractIndices(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_EXP extractIndices(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_EXP extractIndices(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool negative,
		bool keepOrganized);

pcl::IndicesPtr extractPlane(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float distanceThreshold,
		int maxIterations = 100,
		pcl::ModelCoefficients * coefficientsOut = 0);
pcl::IndicesPtr extractPlane(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float distanceThreshold,
		int maxIterations = 100,
		pcl::ModelCoefficients * coefficientsOut = 0);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_FILTERING_H_ */
