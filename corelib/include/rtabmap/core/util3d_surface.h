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

#ifndef UTIL3D_SURFACE_H_
#define UTIL3D_SURFACE_H_

#include <rtabmap/core/RtabmapExp.h>

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/TextureMesh.h>
#include <pcl/pcl_base.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <set>

namespace rtabmap
{

namespace util3d
{

/**
 * @brief Given a set of polygons, create two indexes: polygons to neighbor polygons and vertices to polygons.
 *
 * @param polygons the polygons to be indexed.
 * @param cloudSize the size of the cloud of the corresponding mesh to polygons (must be at least as high as
 * the highest vertex value contained in the polygons).
 * @param neighborPolygons returned index from polygons to neighbor polygons (index size = polygons size).
 * @param vertexPolygons returned index from vertices to polygons (index size = cloudSize).
 */
void RTABMAP_EXP createPolygonIndexes(
		const std::vector<pcl::Vertices> & polygons,
		int cloudSize,
		std::vector<std::set<int> > & neighborPolygons,
		std::vector<std::set<int> > & vertexPolygons);

std::vector<pcl::Vertices> RTABMAP_EXP organizedFastMesh(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		double angleTolerance = M_PI/16,
		bool quad=true,
		int trianglePixelSize = 2,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0));
std::vector<pcl::Vertices> RTABMAP_EXP organizedFastMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		double angleTolerance = M_PI/16,
		bool quad=true,
		int trianglePixelSize = 2,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0));

void RTABMAP_EXP appendMesh(
		pcl::PointCloud<pcl::PointXYZRGBNormal> & cloudA,
		std::vector<pcl::Vertices> & polygonsA,
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloudB,
		const std::vector<pcl::Vertices> & polygonsB);
void RTABMAP_EXP appendMesh(
		pcl::PointCloud<pcl::PointXYZRGB> & cloudA,
		std::vector<pcl::Vertices> & polygonsA,
		const pcl::PointCloud<pcl::PointXYZRGB> & cloudB,
		const std::vector<pcl::Vertices> & polygonsB);

// return map from new to old polygon indices
std::map<int, int> RTABMAP_EXP filterNotUsedVerticesFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		pcl::PointCloud<pcl::PointXYZRGBNormal> & outputCloud,
		std::vector<pcl::Vertices> & outputPolygons);
std::map<int, int> RTABMAP_EXP filterNotUsedVerticesFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		pcl::PointCloud<pcl::PointXYZRGB> & outputCloud,
		std::vector<pcl::Vertices> & outputPolygons);

std::vector<pcl::Vertices> RTABMAP_EXP filterCloseVerticesFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		const std::vector<pcl::Vertices> & polygons,
		float radius,
		float angle,
		bool keepLatestInRadius);

std::vector<pcl::Vertices> RTABMAP_EXP filterInvalidPolygons(
		const std::vector<pcl::Vertices> & polygons);

pcl::PolygonMesh::Ptr RTABMAP_EXP createMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloudWithNormals,
		float gp3SearchRadius = 0.025,
		float gp3Mu = 2.5,
		int gp3MaximumNearestNeighbors = 100,
		float gp3MaximumSurfaceAngle = M_PI/4,
		float gp3MinimumAngle = M_PI/18,
		float gp3MaximumAngle = 2*M_PI/3,
		bool gp3NormalConsistency = true);

pcl::TextureMesh::Ptr RTABMAP_EXP createTextureMesh(
		const pcl::PolygonMesh::Ptr & mesh,
		const std::map<int, Transform> & poses,
		const std::map<int, CameraModel> & cameraModels,
		const std::map<int, cv::Mat> & images,
		const std::string & tmpDirectory = ".",
		int kNormalSearch = 20); // if mesh doesn't have normals, compute them with k neighbors

pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_EXP computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int normalKSearch = 20,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_EXP computeNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int normalKSearch = 20,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_EXP computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int normalKSearch = 20,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_EXP computeNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int normalKSearch = 20,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));

pcl::PointCloud<pcl::Normal>::Ptr computeFastOrganizedNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float maxDepthChangeFactor = 0.02f,
		float normalSmoothingSize = 10.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr computeFastOrganizedNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float maxDepthChangeFactor = 0.02f,
		float normalSmoothingSize = 10.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP mls(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float searchRadius = 0.0f,
		int polygonialOrder = 2,
		int upsamplingMethod = 0, // NONE, DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
		float upsamplingRadius = 0.0f,   // SAMPLE_LOCAL_PLANE
		float upsamplingStep = 0.0f,     // SAMPLE_LOCAL_PLANE
		int pointDensity = 0,            // RANDOM_UNIFORM_DENSITY
		float dilationVoxelSize = 1.0f,  // VOXEL_GRID_DILATION
		int dilationIterations = 0);     // VOXEL_GRID_DILATION
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP mls(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float searchRadius = 0.0f,
		int polygonialOrder = 2,
		int upsamplingMethod = 0, // NONE, DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
		float upsamplingRadius = 0.0f,   // SAMPLE_LOCAL_PLANE
		float upsamplingStep = 0.0f,     // SAMPLE_LOCAL_PLANE
		int pointDensity = 0,            // RANDOM_UNIFORM_DENSITY
		float dilationVoxelSize = 1.0f,  // VOXEL_GRID_DILATION
		int dilationIterations = 0);     // VOXEL_GRID_DILATION

void RTABMAP_EXP adjustNormalsToViewPoints(
		const std::map<int, Transform> & poses,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & rawCloud,
		const std::vector<int> & rawCameraIndices,
		pcl::PointCloud<pcl::PointNormal>::Ptr & cloud);
void RTABMAP_EXP adjustNormalsToViewPoints(
		const std::map<int, Transform> & poses,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & rawCloud,
		const std::vector<int> & rawCameraIndices,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud);

pcl::PolygonMesh::Ptr RTABMAP_EXP meshDecimation(const pcl::PolygonMesh::Ptr & mesh, float factor);

template<typename pointT>
std::vector<pcl::Vertices> normalizePolygonsSide(
		const pcl::PointCloud<pointT> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		const pcl::PointXYZ & viewPoint = pcl::PointXYZ(0,0,0))
{
	std::vector<pcl::Vertices> output(polygons.size());
	for(unsigned int i=0; i<polygons.size(); ++i)
	{
		pcl::Vertices polygon = polygons[i];
		Eigen::Vector3f v1 = cloud.at(polygon.vertices[1]).getVector3fMap() - cloud.at(polygon.vertices[0]).getVector3fMap();
		Eigen::Vector3f v2 = cloud.at(polygon.vertices[2]).getVector3fMap() - cloud.at(polygon.vertices[0]).getVector3fMap();
		Eigen::Vector3f n = (v1.cross(v2)).normalized();

		Eigen::Vector3f p = Eigen::Vector3f(viewPoint.x, viewPoint.y, viewPoint.z) - cloud.at(polygon.vertices[1]).getVector3fMap();

		float result = n.dot(p);
		if(result < 0)
		{
			//reverse vertices order
			int tmp = polygon.vertices[0];
			polygon.vertices[0] = polygon.vertices[2];
			polygon.vertices[2] = tmp;
		}

		output[i] = polygon;
	}
	return output;
}

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_SURFACE_H_ */
