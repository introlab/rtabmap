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

#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/DBDriver.h"
#include "rtabmap/core/Compression.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/UTimer.h"
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/types_c.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl18/surface/texture_mapping.h>
#include <pcl/features/integral_image_normal.h>

#ifdef RTABMAP_ALICE_VISION
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <boost/algorithm/string.hpp>
using namespace aliceVision;
#endif

#ifndef DISABLE_VTK
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#endif

#if PCL_VERSION_COMPARE(>, 1, 11, 1)
#include <pcl/types.h>
#endif
#if PCL_VERSION_COMPARE(<, 1, 8, 0)
#include "pcl18/surface/organized_fast_mesh.h"
#else
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/impl/marching_cubes.hpp>
#include <pcl/surface/impl/organized_fast_mesh.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(OrganizedFastMesh, (pcl::PointXYZRGBNormal))

#include <pcl/features/impl/normal_3d_omp.hpp>
#if PCL_VERSION_COMPARE(<=, 1, 8, 0)
#ifdef PCL_ONLY_CORE_POINT_TYPES
PCL_INSTANTIATE_PRODUCT(NormalEstimationOMP, ((pcl::PointXYZRGB))((pcl::Normal)))
#endif
#endif
#endif

namespace rtabmap
{

namespace util3d
{

void createPolygonIndexes(
		const std::vector<pcl::Vertices> & polygons,
		int cloudSize,
		std::vector<std::set<int> > & neighbors,
		std::vector<std::set<int> > & vertexToPolygons)
{
	vertexToPolygons = std::vector<std::set<int> >(cloudSize);
	neighbors = std::vector<std::set<int> >(polygons.size());

	for(unsigned int i=0; i<polygons.size(); ++i)
	{
		std::set<int> vertices(polygons[i].vertices.begin(), polygons[i].vertices.end());

		for(unsigned int j=0; j<polygons[i].vertices.size(); ++j)
		{
			int v = polygons[i].vertices.at(j);
			for(std::set<int>::iterator iter=vertexToPolygons[v].begin(); iter!=vertexToPolygons[v].end(); ++iter)
			{
				int numSharedVertices = 0;
				for(unsigned int k=0; k<polygons.at(*iter).vertices.size() && numSharedVertices<2; ++k)
				{
					if(vertices.find(polygons.at(*iter).vertices.at(k)) != vertices.end())
					{
						++numSharedVertices;
					}
				}
				if(numSharedVertices >= 2)
				{
					neighbors[*iter].insert(i);
					neighbors[i].insert(*iter);
				}
			}
			vertexToPolygons[v].insert(i);
		}
	}
}

std::list<std::list<int> > clusterPolygons(
		const std::vector<std::set<int> > & neighborPolygons,
		int minClusterSize)
{
	std::set<int> polygonsChecked;

	std::list<std::list<int> > clusters;

	for(unsigned int i=0; i<neighborPolygons.size(); ++i)
	{
		if(polygonsChecked.find(i) == polygonsChecked.end())
		{
			std::list<int> currentCluster;
			currentCluster.push_back(i);
			polygonsChecked.insert(i);

			for(std::list<int>::iterator iter=currentCluster.begin(); iter!=currentCluster.end(); ++iter)
			{
				// get neighbor polygons
				std::set<int> neighbors = neighborPolygons[*iter];
				for(std::set<int>::iterator jter=neighbors.begin(); jter!=neighbors.end(); ++jter)
				{
					if(polygonsChecked.insert(*jter).second)
					{
						currentCluster.push_back(*jter);
					}
				}
			}
			if((int)currentCluster.size() > minClusterSize)
			{
				clusters.push_back(currentCluster);
			}
		}
	}
	return clusters;
}

std::vector<pcl::Vertices> organizedFastMesh(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		double angleTolerance,
		bool quad,
		int trianglePixelSize,
		const Eigen::Vector3f & viewpoint)
{
	UDEBUG("size=%d angle=%f quad=%d triangleSize=%d", (int)cloud->size(), angleTolerance, quad?1:0, trianglePixelSize);
	UASSERT(cloud->is_dense == false);
	UASSERT(cloud->width > 1 && cloud->height > 1);

	pcl::OrganizedFastMesh<pcl::PointXYZ> ofm;
	ofm.setTrianglePixelSize (trianglePixelSize);
	ofm.setTriangulationType (quad?pcl::OrganizedFastMesh<pcl::PointXYZ>::QUAD_MESH:pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_RIGHT_CUT);
	ofm.setInputCloud (cloud);
	ofm.setAngleTolerance(angleTolerance);
	ofm.setViewpoint(viewpoint);

	std::vector<pcl::Vertices> vertices;
	ofm.reconstruct (vertices);

	if(quad)
	{
		//flip all polygons (right handed)
		std::vector<pcl::Vertices> output(vertices.size());
		for(unsigned int i=0; i<vertices.size(); ++i)
		{
			output[i].vertices.resize(4);
			output[i].vertices[0] = vertices[i].vertices[0];
			output[i].vertices[3] = vertices[i].vertices[1];
			output[i].vertices[2] = vertices[i].vertices[2];
			output[i].vertices[1] = vertices[i].vertices[3];
		}
		return output;
	}

	return vertices;
}
std::vector<pcl::Vertices> organizedFastMesh(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		double angleTolerance,
		bool quad,
		int trianglePixelSize,
		const Eigen::Vector3f & viewpoint)
{
	UDEBUG("size=%d angle=%f quad=%d triangleSize=%d", (int)cloud->size(), angleTolerance, quad?1:0, trianglePixelSize);
	UASSERT(cloud->is_dense == false);
	UASSERT(cloud->width > 1 && cloud->height > 1);

	pcl::OrganizedFastMesh<pcl::PointXYZRGB> ofm;
	ofm.setTrianglePixelSize (trianglePixelSize);
	ofm.setTriangulationType (quad?pcl::OrganizedFastMesh<pcl::PointXYZRGB>::QUAD_MESH:pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_RIGHT_CUT);
	ofm.setInputCloud (cloud);
	ofm.setAngleTolerance(angleTolerance);
	ofm.setViewpoint(viewpoint);

	std::vector<pcl::Vertices> vertices;
	ofm.reconstruct (vertices);

	if(quad)
	{
		//flip all polygons (right handed)
		std::vector<pcl::Vertices> output(vertices.size());
		for(unsigned int i=0; i<vertices.size(); ++i)
		{
			output[i].vertices.resize(4);
			output[i].vertices[0] = vertices[i].vertices[0];
			output[i].vertices[3] = vertices[i].vertices[1];
			output[i].vertices[2] = vertices[i].vertices[2];
			output[i].vertices[1] = vertices[i].vertices[3];
		}
		return output;
	}

	return vertices;
}
std::vector<pcl::Vertices> organizedFastMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		double angleTolerance,
		bool quad,
		int trianglePixelSize,
		const Eigen::Vector3f & viewpoint)
{
	UDEBUG("size=%d angle=%f quad=%d triangleSize=%d", (int)cloud->size(), angleTolerance, quad?1:0, trianglePixelSize);
	UASSERT(cloud->is_dense == false);
	UASSERT(cloud->width > 1 && cloud->height > 1);

	pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal> ofm;
	ofm.setTrianglePixelSize (trianglePixelSize);
	ofm.setTriangulationType (quad?pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal>::QUAD_MESH:pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal>::TRIANGLE_RIGHT_CUT);
	ofm.setInputCloud (cloud);
	ofm.setAngleTolerance(angleTolerance);
	ofm.setViewpoint(viewpoint);

	std::vector<pcl::Vertices> vertices;
	ofm.reconstruct (vertices);

	if(quad)
	{
		//flip all polygons (right handed)
		std::vector<pcl::Vertices> output(vertices.size());
		for(unsigned int i=0; i<vertices.size(); ++i)
		{
			output[i].vertices.resize(4);
			output[i].vertices[0] = vertices[i].vertices[0];
			output[i].vertices[3] = vertices[i].vertices[1];
			output[i].vertices[2] = vertices[i].vertices[2];
			output[i].vertices[1] = vertices[i].vertices[3];
		}
		return output;
	}

	return vertices;
}

void appendMesh(
		pcl::PointCloud<pcl::PointXYZRGBNormal> & cloudA,
		std::vector<pcl::Vertices> & polygonsA,
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloudB,
		const std::vector<pcl::Vertices> & polygonsB)
{
	UDEBUG("cloudA=%d polygonsA=%d cloudB=%d polygonsB=%d", (int)cloudA.size(), (int)polygonsA.size(), (int)cloudB.size(), (int)polygonsB.size());
	UASSERT(!cloudA.isOrganized() && !cloudB.isOrganized());

	int sizeA = (int)cloudA.size();
	cloudA += cloudB;

	int sizePolygonsA = (int)polygonsA.size();
	polygonsA.resize(sizePolygonsA+polygonsB.size());

	for(unsigned int i=0; i<polygonsB.size(); ++i)
	{
		pcl::Vertices vertices = polygonsB[i];
		for(unsigned int j=0; j<vertices.vertices.size(); ++j)
		{
			vertices.vertices[j] += sizeA;
		}
		polygonsA[i+sizePolygonsA] = vertices;
	}
}

void appendMesh(
		pcl::PointCloud<pcl::PointXYZRGB> & cloudA,
		std::vector<pcl::Vertices> & polygonsA,
		const pcl::PointCloud<pcl::PointXYZRGB> & cloudB,
		const std::vector<pcl::Vertices> & polygonsB)
{
	UDEBUG("cloudA=%d polygonsA=%d cloudB=%d polygonsB=%d", (int)cloudA.size(), (int)polygonsA.size(), (int)cloudB.size(), (int)polygonsB.size());
	UASSERT(!cloudA.isOrganized() && !cloudB.isOrganized());

	int sizeA = (int)cloudA.size();
	cloudA += cloudB;

	int sizePolygonsA = (int)polygonsA.size();
	polygonsA.resize(sizePolygonsA+polygonsB.size());

	for(unsigned int i=0; i<polygonsB.size(); ++i)
	{
		pcl::Vertices vertices = polygonsB[i];
		for(unsigned int j=0; j<vertices.vertices.size(); ++j)
		{
			vertices.vertices[j] += sizeA;
		}
		polygonsA[i+sizePolygonsA] = vertices;
	}
}

std::vector<int> filterNotUsedVerticesFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		pcl::PointCloud<pcl::PointXYZRGBNormal> & outputCloud,
		std::vector<pcl::Vertices> & outputPolygons)
{
	UDEBUG("size=%d polygons=%d", (int)cloud.size(), (int)polygons.size());
	std::map<int, int> addedVertices; //<oldIndex, newIndex>
	std::vector<int> output; //<oldIndex>
	output.resize(cloud.size());
	outputCloud.resize(cloud.size());
	outputCloud.is_dense = true;
	outputPolygons.resize(polygons.size());
	int oi = 0;
	for(unsigned int i=0; i<polygons.size(); ++i)
	{
		pcl::Vertices & v = outputPolygons[i];
		v.vertices.resize(polygons[i].vertices.size());
		for(unsigned int j=0; j<polygons[i].vertices.size(); ++j)
		{
			std::map<int, int>::iterator iter = addedVertices.find(polygons[i].vertices[j]);
			if(iter == addedVertices.end())
			{
				outputCloud[oi] = cloud.at(polygons[i].vertices[j]);
				addedVertices.insert(std::make_pair(polygons[i].vertices[j], oi));
				output[oi] = polygons[i].vertices[j];
				v.vertices[j] = oi++;
			}
			else
			{
				v.vertices[j] = iter->second;
			}
		}
	}
	outputCloud.resize(oi);
	output.resize(oi);

	return output;
}

std::vector<int> filterNotUsedVerticesFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		pcl::PointCloud<pcl::PointXYZRGB> & outputCloud,
		std::vector<pcl::Vertices> & outputPolygons)
{
	UDEBUG("size=%d polygons=%d", (int)cloud.size(), (int)polygons.size());
	std::map<int, int> addedVertices; //<oldIndex, newIndex>
	std::vector<int> output; //<oldIndex>
	output.resize(cloud.size());
	outputCloud.resize(cloud.size());
	outputCloud.is_dense = true;
	outputPolygons.resize(polygons.size());
	int oi = 0;
	for(unsigned int i=0; i<polygons.size(); ++i)
	{
		pcl::Vertices & v = outputPolygons[i];
		v.vertices.resize(polygons[i].vertices.size());
		for(unsigned int j=0; j<polygons[i].vertices.size(); ++j)
		{
			std::map<int, int>::iterator iter = addedVertices.find(polygons[i].vertices[j]);
			if(iter == addedVertices.end())
			{
				outputCloud[oi] = cloud.at(polygons[i].vertices[j]);
				addedVertices.insert(std::make_pair(polygons[i].vertices[j], oi));
				output[oi] = polygons[i].vertices[j];
				v.vertices[j] = oi++;
			}
			else
			{
				v.vertices[j] = iter->second;
			}
		}
	}
	outputCloud.resize(oi);
	output.resize(oi);

	return output;
}

std::vector<int> filterNaNPointsFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		pcl::PointCloud<pcl::PointXYZRGB> & outputCloud,
		std::vector<pcl::Vertices> & outputPolygons)
{
	UDEBUG("size=%d polygons=%d", (int)cloud.size(), (int)polygons.size());
	std::map<int, int> addedVertices; //<oldIndex, newIndex>
	std::vector<int> output; //<oldIndex>
	output.resize(cloud.size());
	outputCloud.resize(cloud.size());
	outputCloud.is_dense = true;
	std::vector<int> organizedToDense(cloud.size(), -1);

	int oi = 0;
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(pcl::isFinite(cloud.at(i)))
		{
			outputCloud.at(oi) = cloud.at(i);
			output[oi] = i;
			organizedToDense[i] = oi;
			++oi;
 		}
	}
	outputCloud.resize(oi);
	output.resize(oi);

	// remap polygons to dense cloud
	outputPolygons = polygons;
	for(unsigned int i=0; i<outputPolygons.size(); ++i)
	{
		pcl::Vertices & v = outputPolygons[i];
		for(unsigned int j=0; j<v.vertices.size(); ++j)
		{
			UASSERT(organizedToDense[v.vertices[j]] >= 0);
			v.vertices[j] = organizedToDense[v.vertices[j]];
		}
	}

	return output;
}

std::vector<pcl::Vertices> filterCloseVerticesFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		const std::vector<pcl::Vertices> & polygons,
		float radius,
		float angle, // FIXME angle not used
		bool keepLatestInRadius)
{
	UDEBUG("size=%d polygons=%d radius=%f angle=%f keepLatest=%d",
			(int)cloud->size(), (int)polygons.size(), radius, angle, keepLatestInRadius?1:0);
	std::vector<pcl::Vertices> outputPolygons;
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>);
	kdtree->setInputCloud(cloud);

	std::map<int, int> verticesDone;
	outputPolygons = polygons;
	for(unsigned int i=0; i<outputPolygons.size(); ++i)
	{
		pcl::Vertices & polygon = outputPolygons[i];
		for(unsigned int j=0; j<polygon.vertices.size(); ++j)
		{
			std::map<int, int>::iterator iter = verticesDone.find(polygon.vertices[j]);
			if(iter != verticesDone.end())
			{
				polygon.vertices[j] = iter->second;
			}
			else
			{
				std::vector<int> kIndices;
				std::vector<float> kDistances;
				kdtree->radiusSearch(polygon.vertices[j], radius, kIndices, kDistances);
				if(kIndices.size())
				{
					int reference = -1;
					for(unsigned int z=0; z<kIndices.size(); ++z)
					{
						if(reference == -1)
						{
							reference = kIndices[z];
						}
						else if(keepLatestInRadius)
						{
							if(kIndices[z] < reference)
							{
								reference = kIndices[z];
							}
						}
						else
						{
							if(kIndices[z] > reference)
							{
								reference = kIndices[z];
							}
						}
					}
					if(reference >= 0)
					{
						for(unsigned int z=0; z<kIndices.size(); ++z)
						{
							verticesDone.insert(std::make_pair(kIndices[j], reference));
						}
						polygon.vertices[j] = reference;
					}
				}
				else
				{
					verticesDone.insert(std::make_pair(polygon.vertices[j], polygon.vertices[j]));
				}
			}
		}
	}
	return outputPolygons;
}

std::vector<pcl::Vertices> filterInvalidPolygons(const std::vector<pcl::Vertices> & polygons)
{
	std::vector<pcl::Vertices> output(polygons.size());
	int oi=0;
	for(unsigned int i=0; i<polygons.size(); ++i)
	{
		bool valid = true;
		for(unsigned int j=0; j<polygons[i].vertices.size(); ++j)
		{
			if(polygons[i].vertices[j] == polygons[i].vertices[(j+1)%polygons[i].vertices.size()])
			{
				valid = false;
				break;
			}
		}
		if(valid)
		{
			output[oi++] = polygons[i];
		}
	}
	output.resize(oi);
	return output;
}

pcl::PolygonMesh::Ptr createMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloudWithNormals,
		float gp3SearchRadius,
		float gp3Mu,
		int gp3MaximumNearestNeighbors,
		float gp3MaximumSurfaceAngle,
		float gp3MinimumAngle,
		float gp3MaximumAngle,
		bool gp3NormalConsistency)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormalsNoNaN = removeNaNNormalsFromPointCloud(cloudWithNormals);

	// Create search tree*
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud (cloudWithNormalsNoNaN);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (gp3SearchRadius);

	// Set typical values for the parameters
	gp3.setMu (gp3Mu);
	gp3.setMaximumNearestNeighbors (gp3MaximumNearestNeighbors);
	gp3.setMaximumSurfaceAngle(gp3MaximumSurfaceAngle); // 45 degrees
	gp3.setMinimumAngle(gp3MinimumAngle); // 10 degrees
	gp3.setMaximumAngle(gp3MaximumAngle); // 120 degrees
	gp3.setNormalConsistency(gp3NormalConsistency);
	gp3.setConsistentVertexOrdering(gp3NormalConsistency);

	// Get result
	gp3.setInputCloud (cloudWithNormalsNoNaN);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (*mesh);

	//UASSERT(mesh->cloud.data.size()/mesh->cloud.point_step == cloudWithNormalsNoNaN->size());
	//mesh->polygons = normalizePolygonsSide(*cloudWithNormalsNoNaN, mesh->polygons);

	return mesh;
}

pcl::texture_mapping::CameraVector createTextureCameras(
		const std::map<int, Transform> & poses,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		const std::map<int, cv::Mat> & cameraDepths,
		const std::vector<float> & roiRatios)
{
	UASSERT(roiRatios.empty() || roiRatios.size() == 4);
	pcl::texture_mapping::CameraVector cameras;

	for(std::map<int, Transform>::const_iterator poseIter=poses.begin(); poseIter!=poses.end(); ++poseIter)
	{
		std::map<int, std::vector<CameraModel> >::const_iterator modelIter=cameraModels.find(poseIter->first);

		if(modelIter!=cameraModels.end())
		{
			std::map<int, cv::Mat>::const_iterator depthIter = cameraDepths.find(poseIter->first);

			// for each sub camera
			for(unsigned int i=0; i<modelIter->second.size(); ++i)
			{
				pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
				// should be in camera frame
				UASSERT(!modelIter->second[i].localTransform().isNull() && !poseIter->second.isNull());
				Transform t = poseIter->second*modelIter->second[i].localTransform();

				cam.pose = t.toEigen3f();

				if(modelIter->second[i].imageHeight() <=0 || modelIter->second[i].imageWidth() <=0)
				{
					UERROR("Should have camera models with width/height set to create texture cameras!");
					return pcl::texture_mapping::CameraVector();
				}

				UASSERT(modelIter->second[i].fx()>0 && modelIter->second[i].imageHeight()>0 && modelIter->second[i].imageWidth()>0);
				cam.focal_length_w=modelIter->second[i].fx();
				cam.focal_length_h=modelIter->second[i].fy();
				cam.center_w=modelIter->second[i].cx();
				cam.center_h=modelIter->second[i].cy();
				cam.height=modelIter->second[i].imageHeight();
				cam.width=modelIter->second[i].imageWidth();
				if(modelIter->second.size() == 1)
				{
					cam.texture_file = uFormat("%d", poseIter->first); // camera index
				}
				else
				{
					cam.texture_file = uFormat("%d_%d", poseIter->first, (int)i); // camera index, sub camera model index
				}
				if(!roiRatios.empty())
				{
					cam.roi.resize(4);
					cam.roi[0] = cam.width * roiRatios[0]; // left -> x
					cam.roi[1] = cam.height * roiRatios[2]; // top -> y
					cam.roi[2] = cam.width * (1.0 - roiRatios[1]) - cam.roi[0]; // right -> width
					cam.roi[3] = cam.height * (1.0 - roiRatios[3]) - cam.roi[1]; // bottom -> height
				}

				if(depthIter != cameraDepths.end() && !depthIter->second.empty())
				{
					UASSERT(depthIter->second.type() == CV_32FC1 || depthIter->second.type() == CV_16UC1);
					UASSERT(depthIter->second.cols % modelIter->second.size() == 0);
					int subWidth = depthIter->second.cols/(modelIter->second.size());
					cam.depth = cv::Mat(depthIter->second, cv::Range(0, depthIter->second.rows), cv::Range(subWidth*i, subWidth*(i+1)));
				}

				UDEBUG("%f", cam.focal_length);
				UDEBUG("%f", cam.height);
				UDEBUG("%f", cam.width);
				UDEBUG("cam.pose=%s", t.prettyPrint().c_str());

				cameras.push_back(cam);
			}
		}
	}
	return cameras;
}

pcl::TextureMesh::Ptr createTextureMesh(
		const pcl::PolygonMesh::Ptr & mesh,
		const std::map<int, Transform> & poses,
		const std::map<int, CameraModel> & cameraModels,
		const std::map<int, cv::Mat> & cameraDepths,
		float maxDistance,
		float maxDepthError,
		float maxAngle,
		int minClusterSize,
		const std::vector<float> & roiRatios,
		const ProgressState * state,
		std::vector<std::map<int, pcl::PointXY> > * vertexToPixels,
		bool distanceToCamPolicy)
{
	std::map<int, std::vector<CameraModel> > cameraSubModels;
	for(std::map<int, CameraModel>::const_iterator iter=cameraModels.begin(); iter!=cameraModels.end(); ++iter)
	{
		std::vector<CameraModel> models;
		models.push_back(iter->second);
		cameraSubModels.insert(std::make_pair(iter->first, models));
	}

	return createTextureMesh(
			mesh,
			poses,
			cameraSubModels,
			cameraDepths,
			maxDistance,
			maxDepthError,
			maxAngle,
			minClusterSize,
			roiRatios,
			state,
			vertexToPixels,
			distanceToCamPolicy);
}

pcl::TextureMesh::Ptr createTextureMesh(
		const pcl::PolygonMesh::Ptr & mesh,
		const std::map<int, Transform> & poses,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		const std::map<int, cv::Mat> & cameraDepths,
		float maxDistance,
		float maxDepthError,
		float maxAngle,
		int minClusterSize,
		const std::vector<float> & roiRatios,
		const ProgressState * state,
		std::vector<std::map<int, pcl::PointXY> > * vertexToPixels,
		bool distanceToCamPolicy)
{
	UASSERT(mesh->polygons.size());
	pcl::TextureMesh::Ptr textureMesh(new pcl::TextureMesh);
	textureMesh->cloud = mesh->cloud;
	textureMesh->tex_polygons.push_back(mesh->polygons);

	// Original from pcl/gpu/kinfu_large_scale/tools/standalone_texture_mapping.cpp:
	// Author: Raphael Favier, Technical University Eindhoven, (r.mysurname <aT> tue.nl)

	// Create the texturemesh object that will contain our UV-mapped mesh

	// create cameras
	pcl::texture_mapping::CameraVector cameras = createTextureCameras(
			poses,
			cameraModels,
			cameraDepths,
			roiRatios);

	// Create materials for each texture (and one extra for occluded faces)
	textureMesh->tex_materials.resize (cameras.size () + 1);
	for(unsigned int i = 0 ; i <= cameras.size() ; ++i)
	{
		pcl::TexMaterial mesh_material;
		mesh_material.tex_Ka.r = 0.2f;
		mesh_material.tex_Ka.g = 0.2f;
		mesh_material.tex_Ka.b = 0.2f;

		mesh_material.tex_Kd.r = 0.8f;
		mesh_material.tex_Kd.g = 0.8f;
		mesh_material.tex_Kd.b = 0.8f;

		mesh_material.tex_Ks.r = 1.0f;
		mesh_material.tex_Ks.g = 1.0f;
		mesh_material.tex_Ks.b = 1.0f;

		mesh_material.tex_d = 1.0f;
		mesh_material.tex_Ns = 75.0f;
		mesh_material.tex_illum = 2;

		std::stringstream tex_name;
		tex_name << "material_" << i;
		tex_name >> mesh_material.tex_name;

		if(i < cameras.size ())
		{
			mesh_material.tex_file = cameras[i].texture_file;
		}
		else
		{
			mesh_material.tex_file = "occluded";
		}

		textureMesh->tex_materials[i] = mesh_material;
	}

	// Texture by projection
	pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
	tm.setMaxDistance(maxDistance);
	tm.setMaxAngle(maxAngle);
	if(maxDepthError > 0.0f)
	{
		tm.setMaxDepthError(maxDepthError);
	}
	tm.setMinClusterSize(minClusterSize);
	if(tm.textureMeshwithMultipleCameras2(*textureMesh, cameras, state, vertexToPixels, distanceToCamPolicy))
	{
		// compute normals for the mesh if not already here
		bool hasNormals = false;
		bool hasColors = false;
		for(unsigned int i=0; i<textureMesh->cloud.fields.size(); ++i)
		{
			if(textureMesh->cloud.fields[i].name.compare("normal_x") == 0)
			{
				hasNormals = true;
			}
			else if(textureMesh->cloud.fields[i].name.compare("rgb") == 0)
			{
				hasColors = true;
			}
		}
		if(!hasNormals)
		{
			// use polygons
			if(hasColors)
			{
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

				for(unsigned int i=0; i<mesh->polygons.size(); ++i)
				{
					pcl::Vertices & v = mesh->polygons[i];
					UASSERT(v.vertices.size()>2);
					Eigen::Vector3f v0(
							cloud->at(v.vertices[1]).x - cloud->at(v.vertices[0]).x,
							cloud->at(v.vertices[1]).y - cloud->at(v.vertices[0]).y,
							cloud->at(v.vertices[1]).z - cloud->at(v.vertices[0]).z);
					int last = v.vertices.size()-1;
					Eigen::Vector3f v1(
							cloud->at(v.vertices[last]).x - cloud->at(v.vertices[0]).x,
							cloud->at(v.vertices[last]).y - cloud->at(v.vertices[0]).y,
							cloud->at(v.vertices[last]).z - cloud->at(v.vertices[0]).z);
					Eigen::Vector3f normal = v0.cross(v1);
					normal.normalize();
					// flat normal (per face)
					for(unsigned int j=0; j<v.vertices.size(); ++j)
					{
						cloud->at(v.vertices[j]).normal_x = normal[0];
						cloud->at(v.vertices[j]).normal_y = normal[1];
						cloud->at(v.vertices[j]).normal_z = normal[2];
					}
				}
				pcl::toPCLPointCloud2 (*cloud, textureMesh->cloud);
			}
			else
			{
				pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
				pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

				for(unsigned int i=0; i<mesh->polygons.size(); ++i)
				{
					pcl::Vertices & v = mesh->polygons[i];
					UASSERT(v.vertices.size()>2);
					Eigen::Vector3f v0(
							cloud->at(v.vertices[1]).x - cloud->at(v.vertices[0]).x,
							cloud->at(v.vertices[1]).y - cloud->at(v.vertices[0]).y,
							cloud->at(v.vertices[1]).z - cloud->at(v.vertices[0]).z);
					int last = v.vertices.size()-1;
					Eigen::Vector3f v1(
							cloud->at(v.vertices[last]).x - cloud->at(v.vertices[0]).x,
							cloud->at(v.vertices[last]).y - cloud->at(v.vertices[0]).y,
							cloud->at(v.vertices[last]).z - cloud->at(v.vertices[0]).z);
					Eigen::Vector3f normal = v0.cross(v1);
					normal.normalize();
					// flat normal (per face)
					for(unsigned int j=0; j<v.vertices.size(); ++j)
					{
						cloud->at(v.vertices[j]).normal_x = normal[0];
						cloud->at(v.vertices[j]).normal_y = normal[1];
						cloud->at(v.vertices[j]).normal_z = normal[2];
					}
				}
				pcl::toPCLPointCloud2 (*cloud, textureMesh->cloud);
			}
		}
	}
	return textureMesh;
}

void cleanTextureMesh(
		pcl::TextureMesh & textureMesh,
		int minClusterSize)
{
	UDEBUG("minClusterSize=%d", minClusterSize);
	// Remove occluded polygons (polygons with no texture)
	if(textureMesh.tex_coordinates.size())
	{
		// assume last texture is the occluded texture
		textureMesh.tex_coordinates.pop_back();
		textureMesh.tex_polygons.pop_back();
		textureMesh.tex_materials.pop_back();

		if(minClusterSize!=0)
		{
			// concatenate all polygons
			unsigned int totalSize = 0;
			for(unsigned int t=0; t<textureMesh.tex_polygons.size(); ++t)
			{
				totalSize+=textureMesh.tex_polygons[t].size();
			}
			std::vector<pcl::Vertices> allPolygons(totalSize);
			int oi=0;
			for(unsigned int t=0; t<textureMesh.tex_polygons.size(); ++t)
			{
				for(unsigned int i=0; i<textureMesh.tex_polygons[t].size(); ++i)
				{
					allPolygons[oi++] =  textureMesh.tex_polygons[t][i];
				}
			}

			// filter polygons
			std::vector<std::set<int> > neighbors;
			std::vector<std::set<int> > vertexToPolygons;
			util3d::createPolygonIndexes(allPolygons,
					(int)textureMesh.cloud.data.size()/textureMesh.cloud.point_step,
					neighbors,
					vertexToPolygons);

			std::list<std::list<int> > clusters = util3d::clusterPolygons(
					neighbors,
					minClusterSize<0?0:minClusterSize);

			std::set<int> validPolygons;
			if(minClusterSize < 0)
			{
				// only keep the biggest cluster
				std::list<std::list<int> >::iterator biggestClusterIndex = clusters.end();
				unsigned int biggestClusterSize = 0;
				for(std::list<std::list<int> >::iterator iter=clusters.begin(); iter!=clusters.end(); ++iter)
				{
					if(iter->size() > biggestClusterSize)
					{
						biggestClusterIndex = iter;
						biggestClusterSize = iter->size();
					}
				}
				if(biggestClusterIndex != clusters.end())
				{
					for(std::list<int>::iterator jter=biggestClusterIndex->begin(); jter!=biggestClusterIndex->end(); ++jter)
					{
						validPolygons.insert(*jter);
					}
				}
			}
			else
			{
				for(std::list<std::list<int> >::iterator iter=clusters.begin(); iter!=clusters.end(); ++iter)
				{
					for(std::list<int>::iterator jter=iter->begin(); jter!=iter->end(); ++jter)
					{
						validPolygons.insert(*jter);
					}
				}
			}

			if(validPolygons.size() == 0)
			{
				UWARN("All %d polygons filtered after polygon cluster filtering. Cluster minimum size is %d.",totalSize, minClusterSize);
			}

			// for each texture
			unsigned int allPolygonsIndex = 0;
			for(unsigned int t=0; t<textureMesh.tex_polygons.size(); ++t)
			{
				std::vector<pcl::Vertices> filteredPolygons(textureMesh.tex_polygons[t].size());
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
				std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > filteredCoordinates(textureMesh.tex_coordinates[t].size());
#else
				std::vector<Eigen::Vector2f> filteredCoordinates(textureMesh.tex_coordinates[t].size());
#endif

				if(textureMesh.tex_polygons[t].size())
				{
					UASSERT_MSG(allPolygonsIndex < allPolygons.size(), uFormat("%d vs %d", (int)allPolygonsIndex, (int)allPolygons.size()).c_str());

					// make index polygon to coordinate
					std::vector<unsigned int> polygonToCoord(textureMesh.tex_polygons[t].size());
					unsigned int totalCoord = 0;
					for(unsigned int i=0; i<textureMesh.tex_polygons[t].size(); ++i)
					{
						polygonToCoord[i] = totalCoord;
						totalCoord+=textureMesh.tex_polygons[t][i].vertices.size();
					}
					UASSERT_MSG(totalCoord == textureMesh.tex_coordinates[t].size(), uFormat("%d vs %d", totalCoord, (int)textureMesh.tex_coordinates[t].size()).c_str());

					int oi=0;
					int ci=0;
					for(unsigned int i=0; i<textureMesh.tex_polygons[t].size(); ++i)
					{
						if(validPolygons.find(allPolygonsIndex) != validPolygons.end())
						{
							filteredPolygons[oi] = textureMesh.tex_polygons[t].at(i);
							for(unsigned int j=0; j<filteredPolygons[oi].vertices.size(); ++j)
							{
								UASSERT(polygonToCoord[i] < textureMesh.tex_coordinates[t].size());
								filteredCoordinates[ci] = textureMesh.tex_coordinates[t][polygonToCoord[i]+j];
								++ci;
							}
							++oi;
						}
						++allPolygonsIndex;
					}
					filteredPolygons.resize(oi);
					filteredCoordinates.resize(ci);
					textureMesh.tex_polygons[t] = filteredPolygons;
					textureMesh.tex_coordinates[t] = filteredCoordinates;
				}
			}
		}
	}
}

pcl::TextureMesh::Ptr concatenateTextureMeshes(const std::list<pcl::TextureMesh::Ptr> & meshes)
{
	pcl::TextureMesh::Ptr output(new pcl::TextureMesh);
	std::map<std::string, int> addedMaterials; //<file, index>
	for(std::list<pcl::TextureMesh::Ptr>::const_iterator iter = meshes.begin(); iter!=meshes.end(); ++iter)
	{
		if((*iter)->cloud.point_step &&
		   (*iter)->cloud.data.size()/(*iter)->cloud.point_step &&
		   (*iter)->tex_polygons.size() &&
		   (*iter)->tex_coordinates.size())
		{
			// append point cloud
			int polygonStep = output->cloud.height * output->cloud.width;
			pcl::PCLPointCloud2 tmp;
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
			pcl::concatenate(output->cloud, iter->get()->cloud, tmp);
#else
			pcl::concatenatePointCloud(output->cloud, iter->get()->cloud, tmp);
#endif
			output->cloud = tmp;

			UASSERT((*iter)->tex_polygons.size() == (*iter)->tex_coordinates.size() &&
					(*iter)->tex_polygons.size() == (*iter)->tex_materials.size());

			int materialCount = (*iter)->tex_polygons.size();
			for(int i=0; i<materialCount; ++i)
			{
				std::map<std::string, int>::iterator jter = addedMaterials.find((*iter)->tex_materials[i].tex_file);
				int index;
				if(jter != addedMaterials.end())
				{
					index = jter->second;
				}
				else
				{
					addedMaterials.insert(std::make_pair((*iter)->tex_materials[i].tex_file, output->tex_materials.size()));
					index = output->tex_materials.size();
					output->tex_materials.push_back((*iter)->tex_materials[i]);
					output->tex_materials.back().tex_name = uFormat("material_%d", index);
					output->tex_polygons.resize(output->tex_polygons.size() + 1);
					output->tex_coordinates.resize(output->tex_coordinates.size() + 1);
				}

				// update and append polygon indices
				int oi = output->tex_polygons[index].size();
				output->tex_polygons[index].resize(output->tex_polygons[index].size() + (*iter)->tex_polygons[i].size());
				for(unsigned int j=0; j<(*iter)->tex_polygons[i].size(); ++j)
				{
					pcl::Vertices polygon = (*iter)->tex_polygons[i][j];
					for(unsigned int k=0; k<polygon.vertices.size(); ++k)
					{
						polygon.vertices[k] += polygonStep;
					}
					output->tex_polygons[index][oi+j] = polygon;
				}

				// append uv coordinates
				oi = output->tex_coordinates[index].size();
				output->tex_coordinates[index].resize(output->tex_coordinates[index].size() + (*iter)->tex_coordinates[i].size());
				for(unsigned int j=0; j<(*iter)->tex_coordinates[i].size(); ++j)
				{
					output->tex_coordinates[index][oi+j] = (*iter)->tex_coordinates[i][j];
				}
			}
		}
	}
	return output;
}

int gcd(int a, int b) {
    return b == 0 ? a : gcd(b, a % b);
}

void concatenateTextureMaterials(pcl::TextureMesh & mesh, const cv::Size & imageSize, int textureSize, int maxTextures, float & scale, std::vector<bool> * materialsKept)
{
	UASSERT(textureSize>0 && imageSize.width>0 && imageSize.height>0);
	if(maxTextures < 1)
	{
		maxTextures = 1;
	}
	int materials = 0;
	for(unsigned int i=0; i<mesh.tex_materials.size(); ++i)
	{
		if(mesh.tex_polygons.size())
		{
			++materials;
		}
	}
	if(materials)
	{
		int w = imageSize.width; // 640
		int h = imageSize.height; // 480
		int g = gcd(w,h); // 160
		int a = w/g; // 4=640/160
		int b = h/g; // 3=480/160
		UDEBUG("w=%d h=%d g=%d a=%d b=%d", w, h, g, a, b);
		int colCount = 0;
		int rowCount = 0;
		float factor = 0.1f;
		float epsilon = 0.001f;
		scale = 1.0f;
		while((colCount*rowCount)*maxTextures < materials || (factor == 0.1f || scale > 1.0f))
		{
			// first run try scale = 1 (no scaling)
			if(factor!=0.1f)
			{
				scale = float(textureSize)/float(w*b*factor);
			}
			colCount = float(textureSize)/(scale*float(w));
			rowCount = float(textureSize)/(scale*float(h));
			factor+=epsilon; // search the maximum perfect fit
		}
		int outputTextures = (materials / (colCount*rowCount)) + (materials % (colCount*rowCount) > 0?1:0);
		UDEBUG("materials=%d col=%d row=%d output textures=%d factor=%f scale=%f", materials, colCount, rowCount, outputTextures, factor-epsilon, scale);

		UASSERT(mesh.tex_coordinates.size() == mesh.tex_materials.size() && mesh.tex_polygons.size() == mesh.tex_materials.size());

		// prepare size
		std::vector<int> totalPolygons(outputTextures, 0);
		std::vector<int> totalCoordinates(outputTextures, 0);
		int count = 0;
		for(unsigned int i=0; i<mesh.tex_materials.size(); ++i)
		{
			if(mesh.tex_polygons[i].size())
			{
				int indexMaterial = count / (colCount*rowCount);
				UASSERT(indexMaterial < outputTextures);

				totalPolygons[indexMaterial]+=mesh.tex_polygons[i].size();
				totalCoordinates[indexMaterial]+=mesh.tex_coordinates[i].size();

				++count;
			}
		}

		pcl::TextureMesh outputMesh;

		int pi = 0;
		int ci = 0;
		int ti=0;
		float scaledHeight = float(int(scale*float(h)))/float(textureSize);
		float scaledWidth = float(int(scale*float(w)))/float(textureSize);
		float lowerBorderSize = 1.0f - scaledHeight*float(rowCount);
		UDEBUG("scaledWidth=%f scaledHeight=%f lowerBorderSize=%f", scaledWidth, scaledHeight, lowerBorderSize);
		if(materialsKept)
		{
			materialsKept->resize(mesh.tex_materials.size(), false);
		}
		for(unsigned int t=0; t<mesh.tex_materials.size(); ++t)
		{
			if(mesh.tex_polygons[t].size())
			{
				int indexMaterial = ti / (colCount*rowCount);
				UASSERT(indexMaterial < outputTextures);
                                if((int)outputMesh.tex_polygons.size() <= indexMaterial)
				{
					std::vector<pcl::Vertices> newPolygons(totalPolygons[indexMaterial]);
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
					std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > newCoordinates(totalCoordinates[indexMaterial]); // UV coordinates
#else
					std::vector<Eigen::Vector2f> newCoordinates(totalCoordinates[indexMaterial]); // UV coordinates
#endif
					outputMesh.tex_polygons.push_back(newPolygons);
					outputMesh.tex_coordinates.push_back(newCoordinates);

					pi=0;
					ci=0;
				}

				int row = (ti/colCount) % rowCount;
				int col = ti%colCount;
				float offsetU = scaledWidth * float(col);
				float offsetV = scaledHeight * float((rowCount - 1) - row) + lowerBorderSize;
				// Texture coords have lower-left origin

				for(unsigned int i=0; i<mesh.tex_polygons[t].size(); ++i)
				{
					UASSERT(pi < (int)outputMesh.tex_polygons[indexMaterial].size());
					outputMesh.tex_polygons[indexMaterial][pi++] = mesh.tex_polygons[t].at(i);
				}

				for(unsigned int i=0; i<mesh.tex_coordinates[t].size(); ++i)
				{
					const Eigen::Vector2f & v = mesh.tex_coordinates[t].at(i);
					if(v[0] >= 0 && v[1] >=0)
					{
						outputMesh.tex_coordinates[indexMaterial][ci][0] = v[0]*scaledWidth + offsetU;
						outputMesh.tex_coordinates[indexMaterial][ci][1] = v[1]*scaledHeight + offsetV;
					}
					else
					{
						outputMesh.tex_coordinates[indexMaterial][ci] = v;
					}
					++ci;
				}
				++ti;
				if(materialsKept)
				{
					materialsKept->at(t) = true;
				}
			}
		}
		pcl::TexMaterial m = mesh.tex_materials.front();
		mesh.tex_materials.clear();
		for(int i=0; i<outputTextures; ++i)
		{
			m.tex_file = "texture";
			m.tex_name = "material";
			if(outputTextures > 1)
			{
				m.tex_file += uNumber2Str(i);
				m.tex_name += uNumber2Str(i);
			}

			mesh.tex_materials.push_back(m);
		}
		mesh.tex_coordinates = outputMesh.tex_coordinates;
		mesh.tex_polygons = outputMesh.tex_polygons;
	}
}

std::vector<std::vector<RTABMAP_PCL_INDEX> > convertPolygonsFromPCL(const std::vector<pcl::Vertices> & polygons)
{
	std::vector<std::vector<RTABMAP_PCL_INDEX> > polygonsOut(polygons.size());
	for(unsigned int p=0; p<polygons.size(); ++p)
	{
		polygonsOut[p] = polygons[p].vertices;
	}
	return polygonsOut;
}
std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > convertPolygonsFromPCL(const std::vector<std::vector<pcl::Vertices> > & tex_polygons)
{
	std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > polygonsOut(tex_polygons.size());
	for(unsigned int t=0; t<tex_polygons.size(); ++t)
	{
		polygonsOut[t].resize(tex_polygons[t].size());
		for(unsigned int p=0; p<tex_polygons[t].size(); ++p)
		{
			polygonsOut[t][p] = tex_polygons[t][p].vertices;
		}
	}
	return polygonsOut;
}
std::vector<pcl::Vertices> convertPolygonsToPCL(const std::vector<std::vector<RTABMAP_PCL_INDEX> > & polygons)
{
	std::vector<pcl::Vertices> polygonsOut(polygons.size());
	for(unsigned int p=0; p<polygons.size(); ++p)
	{
		polygonsOut[p].vertices = polygons[p];
	}
	return polygonsOut;
}
std::vector<std::vector<pcl::Vertices> > convertPolygonsToPCL(const std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > & tex_polygons)
{
	std::vector<std::vector<pcl::Vertices> > polygonsOut(tex_polygons.size());
	for(unsigned int t=0; t<tex_polygons.size(); ++t)
	{
		polygonsOut[t].resize(tex_polygons[t].size());
		for(unsigned int p=0; p<tex_polygons[t].size(); ++p)
		{
			polygonsOut[t][p].vertices = tex_polygons[t][p];
		}
	}
	return polygonsOut;
}

pcl::TextureMesh::Ptr assembleTextureMesh(
		const cv::Mat & cloudMat,
		const std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > & polygons,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
		const std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > & texCoords,
#else
		const std::vector<std::vector<Eigen::Vector2f> > & texCoords,
#endif
		cv::Mat & textures,
		bool mergeTextures)
{
	pcl::TextureMesh::Ptr textureMesh(new pcl::TextureMesh);

	if(cloudMat.channels() <= 3)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::laserScanToPointCloud(LaserScan::backwardCompatibility(cloudMat));
		pcl::toPCLPointCloud2(*cloud, textureMesh->cloud);
	}
	else if(cloudMat.channels() == 4)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = rtabmap::util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(cloudMat));
		pcl::toPCLPointCloud2(*cloud, textureMesh->cloud);
	}
	else if(cloudMat.channels() == 6)
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud = rtabmap::util3d::laserScanToPointCloudNormal(LaserScan::backwardCompatibility(cloudMat));
		pcl::toPCLPointCloud2(*cloud, textureMesh->cloud);
	}
	else if(cloudMat.channels() == 7)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = rtabmap::util3d::laserScanToPointCloudRGBNormal(LaserScan::backwardCompatibility(cloudMat));
		pcl::toPCLPointCloud2(*cloud, textureMesh->cloud);
	}

	if(textureMesh->cloud.data.size() && polygons.size())
	{
		textureMesh->tex_polygons.resize(polygons.size());
		for(unsigned int t=0; t<polygons.size(); ++t)
		{
			textureMesh->tex_polygons[t].resize(polygons[t].size());
			for(unsigned int p=0; p<polygons[t].size(); ++p)
			{
				textureMesh->tex_polygons[t][p].vertices = polygons[t][p];
			}
		}

		if(!texCoords.empty() && !textures.empty())
		{
			textureMesh->tex_coordinates = texCoords;

			textureMesh->tex_materials.resize (textureMesh->tex_coordinates.size());
			for(unsigned int i = 0 ; i < textureMesh->tex_coordinates.size() ; ++i)
			{
				pcl::TexMaterial mesh_material;
				mesh_material.tex_Ka.r = 0.2f;
				mesh_material.tex_Ka.g = 0.2f;
				mesh_material.tex_Ka.b = 0.2f;

				mesh_material.tex_Kd.r = 0.8f;
				mesh_material.tex_Kd.g = 0.8f;
				mesh_material.tex_Kd.b = 0.8f;

				mesh_material.tex_Ks.r = 1.0f;
				mesh_material.tex_Ks.g = 1.0f;
				mesh_material.tex_Ks.b = 1.0f;

				mesh_material.tex_d = 1.0f;
				mesh_material.tex_Ns = 75.0f;
				mesh_material.tex_illum = 2;

				std::stringstream tex_name;
				tex_name << "material_" << i;
				tex_name >> mesh_material.tex_name;

				mesh_material.tex_file = uFormat("%d", i);

				textureMesh->tex_materials[i] = mesh_material;
			}

			if(mergeTextures && textures.cols/textures.rows > 1)
			{
				UASSERT(textures.cols % textures.rows == 0 && textures.cols/textures.rows == (int)textureMesh->tex_coordinates.size());
				std::vector<bool> materialsKept;
				float scale = 0.0f;
				cv::Size imageSize(textures.rows, textures.rows);
				int imageType = textures.type();
				rtabmap::util3d::concatenateTextureMaterials(*textureMesh, imageSize, textures.rows, 1, scale, &materialsKept);
				if(scale && textureMesh->tex_materials.size() == 1)
				{
					int cols = float(textures.rows)/(scale*imageSize.width);
					int rows = float(textures.rows)/(scale*imageSize.height);

					cv::Mat mergedTextures = cv::Mat(textures.rows, textures.rows, imageType, cv::Scalar::all(255));

					// make a blank texture
					cv::Size resizedImageSize(int(imageSize.width*scale), int(imageSize.height*scale));
					int oi=0;
					for(int i=0; i<(int)materialsKept.size(); ++i)
					{
						if(materialsKept.at(i))
						{
							int u = oi%cols * resizedImageSize.width;
							int v = ((oi/cols) % rows ) * resizedImageSize.height;
							UASSERT(u < textures.rows-resizedImageSize.width);
							UASSERT(v < textures.rows-resizedImageSize.height);

							cv::Mat resizedImage;
							cv::resize(textures(cv::Range::all(), cv::Range(i*textures.rows, (i+1)*textures.rows)), resizedImage, resizedImageSize, 0.0f, 0.0f, cv::INTER_AREA);

							UASSERT(resizedImage.type() == mergedTextures.type());
							resizedImage.copyTo(mergedTextures(cv::Rect(u, v, resizedImage.cols, resizedImage.rows)));

							++oi;
						}
					}
					textures = mergedTextures;
				}
			}
		}
	}
	return textureMesh;
}

pcl::PolygonMesh::Ptr assemblePolygonMesh(
		const cv::Mat & cloudMat,
		const std::vector<std::vector<RTABMAP_PCL_INDEX> > & polygons)
{
	pcl::PolygonMesh::Ptr polygonMesh(new pcl::PolygonMesh);

	if(cloudMat.channels() <= 3)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::laserScanToPointCloud(LaserScan::backwardCompatibility(cloudMat));
		pcl::toPCLPointCloud2(*cloud, polygonMesh->cloud);
	}
	else if(cloudMat.channels() == 4)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = rtabmap::util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(cloudMat));
		pcl::toPCLPointCloud2(*cloud, polygonMesh->cloud);
	}
	else if(cloudMat.channels() == 6)
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud = rtabmap::util3d::laserScanToPointCloudNormal(LaserScan::backwardCompatibility(cloudMat));
		pcl::toPCLPointCloud2(*cloud, polygonMesh->cloud);
	}
	else if(cloudMat.channels() == 7)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = rtabmap::util3d::laserScanToPointCloudRGBNormal(LaserScan::backwardCompatibility(cloudMat));
		pcl::toPCLPointCloud2(*cloud, polygonMesh->cloud);
	}

	if(polygonMesh->cloud.data.size() && polygons.size())
	{
		polygonMesh->polygons.resize(polygons.size());
		for(unsigned int p=0; p<polygons.size(); ++p)
		{
			polygonMesh->polygons[p].vertices = polygons[p];
		}
	}
	return polygonMesh;
}

double sqr(uchar v)
{
	return double(v)*double(v);
}

cv::Mat mergeTextures(
		pcl::TextureMesh & mesh,
		const std::map<int, cv::Mat> & images,
		const std::map<int, CameraModel> & calibrations,
		const Memory * memory,
		const DBDriver * dbDriver,
		int textureSize,
		int textureCount,
		const std::vector<std::map<int, pcl::PointXY> > & vertexToPixels,
		bool gainCompensation,
		float gainBeta,
		bool gainRGB,
		bool blending,
		int blendingDecimation,
		int brightnessContrastRatioLow,
		int brightnessContrastRatioHigh,
		bool exposureFusion,
		const ProgressState * state,
		unsigned char blankValue,
		std::map<int, std::map<int, cv::Vec4d> > * gains,
		std::map<int, std::map<int, cv::Mat> > * blendingGains,
		std::pair<float, float> * contrastValues)
{
	std::map<int, std::vector<CameraModel> > calibVectors;
	for(std::map<int, CameraModel>::const_iterator iter=calibrations.begin(); iter!=calibrations.end(); ++iter)
	{
		std::vector<CameraModel> m;
		m.push_back(iter->second);
		calibVectors.insert(std::make_pair(iter->first, m));
	}
	return mergeTextures(mesh,
			images,
			calibVectors,
			memory,
			dbDriver,
			textureSize,
			textureCount,
			vertexToPixels,
			gainCompensation,
			gainBeta,
			gainRGB,
			blending,
			blendingDecimation,
			brightnessContrastRatioLow,
			brightnessContrastRatioHigh,
			exposureFusion,
			state,
			blankValue,
			gains,
			blendingGains,
			contrastValues);
}
cv::Mat mergeTextures(
		pcl::TextureMesh & mesh,
		const std::map<int, cv::Mat> & images,
		const std::map<int, std::vector<CameraModel> > & calibrations,
		const Memory * memory,
		const DBDriver * dbDriver,
		int textureSize,
		int textureCount,
		const std::vector<std::map<int, pcl::PointXY> > & vertexToPixels,
		bool gainCompensation,
		float gainBeta,
		bool gainRGB,
		bool blending,
		int blendingDecimation,
		int brightnessContrastRatioLow,
		int brightnessContrastRatioHigh,
		bool exposureFusion,
		const ProgressState * state,
		unsigned char blankValue,
		std::map<int, std::map<int, cv::Vec4d> > * gainsOut,
		std::map<int, std::map<int, cv::Mat> > * blendingGainsOut,
		std::pair<float, float> * contrastValuesOut)
{
	//get texture size, if disabled use default 1024
	UASSERT(textureSize%256 == 0);
	UDEBUG("textureSize = %d", textureSize);
	cv::Mat globalTextures;
	if(!mesh.tex_materials.empty())
	{
		std::vector<std::pair<int, int> > textures(mesh.tex_materials.size(), std::pair<int, int>(-1,0));
		cv::Size imageSize;
		const int imageType=CV_8UC3;

		UDEBUG("");
		for(unsigned int i=0; i<mesh.tex_materials.size(); ++i)
		{
			std::list<std::string> texFileSplit = uSplit(mesh.tex_materials[i].tex_file, '_');
			if(!mesh.tex_materials[i].tex_file.empty() &&
				mesh.tex_polygons[i].size() &&
			   uIsInteger(texFileSplit.front(), false))
			{
				textures[i].first = uStr2Int(texFileSplit.front());
				if(texFileSplit.size() == 2 &&
				   uIsInteger(texFileSplit.back(), false)	)
				{
					textures[i].second = uStr2Int(texFileSplit.back());
				}

				int textureId = textures[i].first;
				if(imageSize.width == 0 || imageSize.height == 0)
				{
					if(images.find(textureId) != images.end() &&
						!images.find(textureId)->second.empty() &&
						calibrations.find(textureId) != calibrations.end())
					{
						const std::vector<CameraModel> & models = calibrations.find(textureId)->second;
						UASSERT(models.size()>=1);
						if(	models[0].imageHeight()>0 &&
							models[0].imageWidth()>0)
						{
							imageSize = models[0].imageSize();
						}
						else if(images.find(textureId)!=images.end())
						{
							// backward compatibility for image size not set in CameraModel
							cv::Mat image = images.find(textureId)->second;
							if(image.rows == 1 && image.type() == CV_8UC1)
							{
								image = uncompressImage(image);
							}
							UASSERT(!image.empty());
							imageSize = image.size();
							if(models.size()>1)
							{
								imageSize.width/=models.size();
							}
						}
					}
					else if(memory)
					{
						SensorData data = memory->getNodeData(textureId, true, false, false, false);
						std::vector<CameraModel> models = data.cameraModels();
						StereoCameraModel stereoModel = data.stereoCameraModel();
						if(models.size()>=1 &&
							models[0].imageHeight()>0 &&
							models[0].imageWidth()>0)
						{
							imageSize = models[0].imageSize();
						}
						else if(stereoModel.left().imageHeight() > 0 &&
								stereoModel.left().imageWidth() > 0)
						{
							imageSize = stereoModel.left().imageSize();
						}
						else // backward compatibility for image size not set in CameraModel
						{
							cv::Mat image;
							data.uncompressDataConst(&image, 0);
							UASSERT(!image.empty());
							imageSize = image.size();
							if(data.cameraModels().size()>1)
							{
								imageSize.width/=data.cameraModels().size();
							}
						}
					}
					else if(dbDriver)
					{
						std::vector<CameraModel> models;
						StereoCameraModel stereoModel;
						dbDriver->getCalibration(textureId, models, stereoModel);
						if(models.size()>=1 &&
							models[0].imageHeight()>0 &&
							models[0].imageWidth()>0)
						{
							imageSize = models[0].imageSize();
						}
						else if(stereoModel.left().imageHeight() > 0 &&
								stereoModel.left().imageWidth() > 0)
						{
							imageSize = stereoModel.left().imageSize();
						}
						else // backward compatibility for image size not set in CameraModel
						{
							SensorData data;
							dbDriver->getNodeData(textureId, data, true, false, false, false);
							cv::Mat image;
							data.uncompressDataConst(&image, 0);
							UASSERT(!image.empty());
							imageSize = image.size();
							if(data.cameraModels().size()>1)
							{
								imageSize.width/=data.cameraModels().size();
							}
						}
					}
				}
			}
			else if(mesh.tex_polygons[i].size() && mesh.tex_materials[i].tex_file.compare("occluded")!=0)
			{
				UWARN("Failed parsing texture file name: %s", mesh.tex_materials[i].tex_file.c_str());
			}
		}
		UDEBUG("textures=%d imageSize=%dx%d", (int)textures.size(), imageSize.height, imageSize.width);
		if(textures.size() && imageSize.height>0 && imageSize.width>0)
		{
			float scale = 0.0f;
			UDEBUG("");
			std::vector<bool> materialsKept;
			util3d::concatenateTextureMaterials(mesh, imageSize, textureSize, textureCount, scale, &materialsKept);
			if(scale && mesh.tex_materials.size())
			{
				int materials = (int)mesh.tex_materials.size();
				int cols = float(textureSize)/(scale*imageSize.width);
				int rows = float(textureSize)/(scale*imageSize.height);

				globalTextures = cv::Mat(textureSize, materials*textureSize, imageType, cv::Scalar::all(blankValue));
				cv::Mat globalTextureMasks = cv::Mat(textureSize, materials*textureSize, CV_8UC1, cv::Scalar::all(0));

				// used for multi camera texturing, to avoid reloading same texture for sub cameras
				cv::Mat previousImage;
				int previousTextureId = 0;
				std::vector<CameraModel> previousCameraModels;

				// make a blank texture
				cv::Mat emptyImage(int(imageSize.height*scale), int(imageSize.width*scale), imageType, cv::Scalar::all(blankValue));
				cv::Mat emptyImageMask(int(imageSize.height*scale), int(imageSize.width*scale), CV_8UC1, cv::Scalar::all(255));
				int oi=0;
				std::vector<cv::Point2i> imageOrigin(textures.size());
				std::vector<int> newCamIndex(textures.size(), -1);
				for(int t=0; t<(int)textures.size(); ++t)
				{
					if(materialsKept.at(t))
					{
						int indexMaterial = oi / (cols*rows);
						UASSERT(indexMaterial < materials);

						newCamIndex[t] = oi;
						int u = oi%cols * emptyImage.cols;
						int v = ((oi/cols) % rows ) * emptyImage.rows;
						UASSERT_MSG(u < textureSize-emptyImage.cols, uFormat("u=%d textureSize=%d emptyImage.cols=%d", u, textureSize, emptyImage.cols).c_str());
						UASSERT_MSG(v < textureSize-emptyImage.rows, uFormat("v=%d textureSize=%d emptyImage.rows=%d", v, textureSize, emptyImage.rows).c_str());
						imageOrigin[t].x = u;
						imageOrigin[t].y = v;
						if(textures[t].first>=0)
						{
							cv::Mat image;
							std::vector<CameraModel> models;

							if(textures[t].first == previousTextureId)
							{
								image = previousImage;
								models = previousCameraModels;
							}
							else
							{
								if(images.find(textures[t].first) != images.end() &&
									!images.find(textures[t].first)->second.empty() &&
									calibrations.find(textures[t].first) != calibrations.end())
								{
									image = images.find(textures[t].first)->second;
									if(image.rows == 1 && image.type() == CV_8UC1)
									{
										image = uncompressImage(image);
									}
									models = calibrations.find(textures[t].first)->second;
								}
								else if(memory)
								{
									SensorData data = memory->getNodeData(textures[t].first, true, false, false, false);
									models = data.cameraModels();
									data.uncompressDataConst(&image, 0);
								}
								else if(dbDriver)
								{
									SensorData data;
									dbDriver->getNodeData(textures[t].first, data, true, false, false, false);
									data.uncompressDataConst(&image, 0);
									StereoCameraModel stereoModel;
									dbDriver->getCalibration(textures[t].first, models, stereoModel);
								}

								previousImage = image;
								previousCameraModels = models;
								previousTextureId = textures[t].first;
							}

							UASSERT(!image.empty());

							if(textures[t].second>=0)
							{
								UASSERT(textures[t].second < (int)models.size());
								int width = image.cols/models.size();
								image = image.colRange(width*textures[t].second, width*(textures[t].second+1));
							}

							cv::Mat resizedImage;
							cv::resize(image, resizedImage, emptyImage.size(), 0.0f, 0.0f, cv::INTER_AREA);
							UASSERT(resizedImage.type() == CV_8UC1 || resizedImage.type() == CV_8UC3);
							if(resizedImage.type() == CV_8UC1)
							{
								cv::Mat resizedImageColor;
								cv::cvtColor(resizedImage, resizedImageColor, CV_GRAY2BGR);
								resizedImage = resizedImageColor;
							}
							UASSERT(resizedImage.type() == globalTextures.type());
							resizedImage.copyTo(globalTextures(cv::Rect(u+indexMaterial*globalTextures.rows, v, resizedImage.cols, resizedImage.rows)));
							emptyImageMask.copyTo(globalTextureMasks(cv::Rect(u+indexMaterial*globalTextureMasks.rows, v, resizedImage.cols, resizedImage.rows)));
						}
						else
						{
							emptyImage.copyTo(globalTextures(cv::Rect(u+indexMaterial*globalTextures.rows, v, emptyImage.cols, emptyImage.rows)));
						}
						++oi;
					}

					if(state)
					{
						if(state->isCanceled())
						{
							return cv::Mat();
						}
						state->callback(uFormat("Assembled texture %d/%d.", t+1, (int)textures.size()));
					}
				}

				UTimer timer;
				if(vertexToPixels.size())
				{
					//UWARN("Saving original.png", globalTexture);
					//cv::imwrite("original.png", globalTexture);

					if(gainCompensation)
					{
						/**
						 * Original code from OpenCV: GainCompensator
						 */

						const int num_images = static_cast<int>(oi);
						cv::Mat_<int> N(num_images, num_images); N.setTo(0);
						cv::Mat_<double> I(num_images, num_images); I.setTo(0);

						cv::Mat_<double> IR(num_images, num_images); IR.setTo(0);
						cv::Mat_<double> IG(num_images, num_images); IG.setTo(0);
						cv::Mat_<double> IB(num_images, num_images); IB.setTo(0);

						// Adjust UV coordinates to globalTexture
						for(unsigned int p=0; p<vertexToPixels.size(); ++p)
						{
							for(std::map<int, pcl::PointXY>::const_iterator iter=vertexToPixels[p].begin(); iter!=vertexToPixels[p].end(); ++iter)
							{
								if(materialsKept.at(iter->first))
								{
									N(newCamIndex[iter->first], newCamIndex[iter->first]) +=1;

									std::map<int, pcl::PointXY>::const_iterator jter=iter;
									++jter;
									int k = 1;
									for(; jter!=vertexToPixels[p].end(); ++jter, ++k)
									{
										if(materialsKept.at(jter->first))
										{
											int i = newCamIndex[iter->first];
											int j = newCamIndex[jter->first];

											N(i, j) += 1;
											N(j, i) += 1;

											int indexMaterial = i / (cols*rows);

											// uv in globalTexture
											int ui = iter->second.x*emptyImage.cols + imageOrigin[iter->first].x;
											int vi = (1.0-iter->second.y)*emptyImage.rows + imageOrigin[iter->first].y;
											int uj = jter->second.x*emptyImage.cols + imageOrigin[jter->first].x;
											int vj = (1.0-jter->second.y)*emptyImage.rows + imageOrigin[jter->first].y;
											cv::Vec3b * pt1 = globalTextures.ptr<cv::Vec3b>(vi,ui+indexMaterial*globalTextures.rows);
											cv::Vec3b * pt2 = globalTextures.ptr<cv::Vec3b>(vj,uj+indexMaterial*globalTextures.rows);

											I(i, j) += std::sqrt(static_cast<double>(sqr(pt1->val[0]) + sqr(pt1->val[1]) + sqr(pt1->val[2])));
											I(j, i) += std::sqrt(static_cast<double>(sqr(pt2->val[0]) + sqr(pt2->val[1]) + sqr(pt2->val[2])));

											IR(i, j) += static_cast<double>(pt1->val[2]);
											IR(j, i) += static_cast<double>(pt2->val[2]);
											IG(i, j) += static_cast<double>(pt1->val[1]);
											IG(j, i) += static_cast<double>(pt2->val[1]);
											IB(i, j) += static_cast<double>(pt1->val[0]);
											IB(j, i) += static_cast<double>(pt2->val[0]);
										}
									}
								}
							}
						}

						for(int i=0; i<num_images; ++i)
						{
							for(int j=i; j<num_images; ++j)
							{
								if(i == j)
								{
									if(N(i,j) == 0)
									{
										N(i,j) = 1;
									}
								}
								else if(N(i, j))
								{
									I(i, j) /= N(i, j);
									I(j, i) /= N(j, i);

									IR(i, j) /= N(i, j);
									IR(j, i) /= N(j, i);
									IG(i, j) /= N(i, j);
									IG(j, i) /= N(j, i);
									IB(i, j) /= N(i, j);
									IB(j, i) /= N(j, i);
								}
							}
						}

						cv::Mat_<double> A(num_images, num_images); A.setTo(0);
						cv::Mat_<double> b(num_images, 1); b.setTo(0);
						cv::Mat_<double> AR(num_images, num_images); AR.setTo(0);
						cv::Mat_<double> AG(num_images, num_images); AG.setTo(0);
						cv::Mat_<double> AB(num_images, num_images); AB.setTo(0);
						double alpha = 0.01;
						double beta = gainBeta;
						for (int i = 0; i < num_images; ++i)
						{
							for (int j = 0; j < num_images; ++j)
							{
								b(i, 0) += beta * N(i, j);
								A(i, i) += beta * N(i, j);
								AR(i, i) += beta * N(i, j);
								AG(i, i) += beta * N(i, j);
								AB(i, i) += beta * N(i, j);
								if (j == i) continue;
								A(i, i) += 2 * alpha * I(i, j) * I(i, j) * N(i, j);
								A(i, j) -= 2 * alpha * I(i, j) * I(j, i) * N(i, j);

								AR(i, i) += 2 * alpha * IR(i, j) * IR(i, j) * N(i, j);
								AR(i, j) -= 2 * alpha * IR(i, j) * IR(j, i) * N(i, j);

								AG(i, i) += 2 * alpha * IG(i, j) * IG(i, j) * N(i, j);
								AG(i, j) -= 2 * alpha * IG(i, j) * IG(j, i) * N(i, j);

								AB(i, i) += 2 * alpha * IB(i, j) * IB(i, j) * N(i, j);
								AB(i, j) -= 2 * alpha * IB(i, j) * IB(j, i) * N(i, j);
							}
						}

						cv::Mat_<double> gainsGray, gainsR, gainsG, gainsB;
						cv::solve(A, b, gainsGray);

						cv::solve(AR, b, gainsR);
						cv::solve(AG, b, gainsG);
						cv::solve(AB, b, gainsB);

						cv::Mat_<double> gains(gainsGray.rows, 4);
						gainsGray.copyTo(gains.col(0));
						gainsR.copyTo(gains.col(1));
						gainsG.copyTo(gains.col(2));
						gainsB.copyTo(gains.col(3));

						for(int t=0; t<(int)textures.size(); ++t)
						{
							//break;
							if(materialsKept.at(t))
							{
								int u = imageOrigin[t].x;
								int v = imageOrigin[t].y;

								UDEBUG("Gain cam%d = %f", newCamIndex[t], gainsGray(newCamIndex[t], 0));

								int indexMaterial = newCamIndex[t] / (cols*rows);
								cv::Mat roi = globalTextures(cv::Rect(u+indexMaterial*globalTextures.rows, v, emptyImage.cols, emptyImage.rows));

								std::vector<cv::Mat> channels;
								cv::split(roi, channels);

								// assuming BGR
								cv::multiply(channels[0], gains(newCamIndex[t], gainRGB?3:0), channels[0]);
								cv::multiply(channels[1], gains(newCamIndex[t], gainRGB?2:0), channels[1]);
								cv::multiply(channels[2], gains(newCamIndex[t], gainRGB?1:0), channels[2]);

								cv::merge(channels, roi);

								if(gainsOut)
								{
									cv::Vec4d g(
										gains(newCamIndex[t], 0),
										gains(newCamIndex[t], 1),
										gains(newCamIndex[t], 2),
										gains(newCamIndex[t], 3));
									if(gainsOut->find(textures[t].first) == gainsOut->end())
									{
										std::map<int,cv::Vec4d> value;
										value.insert(std::make_pair(textures[t].second, g));
										gainsOut->insert(std::make_pair(textures[t].first, value));
									}
									else
									{
										gainsOut->at(textures[t].first).insert(std::make_pair(textures[t].second,  g));
									}
								}
							}
						}
						//UWARN("Saving gain.png", globalTexture);
						//cv::imwrite("gain.png", globalTexture);
						if(state) state->callback(uFormat("Gain compensation %fs", timer.ticks()));
					}

					if(blending)
					{
						// blending BGR
						int decimation = 1;
						if(blendingDecimation <= 0)
						{
							// determinate decimation to apply
							std::vector<float> edgeLengths;
							if(mesh.tex_coordinates.size() && mesh.tex_coordinates[0].size())
							{
								UASSERT(mesh.tex_polygons.size() && mesh.tex_polygons[0].size() && mesh.tex_polygons[0][0].vertices.size());
								int polygonSize = mesh.tex_polygons[0][0].vertices.size();
								UDEBUG("polygon size=%d", polygonSize);

								for(unsigned int k=0; k<mesh.tex_coordinates.size(); ++k)
								{
									for(unsigned int i=0; i<mesh.tex_coordinates[k].size(); i+=polygonSize)
									{
										for(int j=0; j<polygonSize; ++j)
										{
											const Eigen::Vector2f & uc1 = mesh.tex_coordinates[k][i + j];
											const Eigen::Vector2f & uc2 = mesh.tex_coordinates[k][i + (j+1)%polygonSize];
											Eigen::Vector2f edge = (uc1-uc2)*textureSize;
											edgeLengths.push_back(fabs(edge[0]));
											edgeLengths.push_back(fabs(edge[1]));
										}
									}
								}
								float edgeLength = 0.0f;
								if(edgeLengths.size())
								{
									std::sort(edgeLengths.begin(), edgeLengths.end());
									float m = uMean(edgeLengths.data(), edgeLengths.size());
									float stddev = std::sqrt(uVariance(edgeLengths.data(), edgeLengths.size(), m));
									edgeLength = m+stddev;
									decimation = 1 << 6;
									for(int i=1; i<=6; ++i)
									{
										if(float(1 << i) >= edgeLength)
										{
											decimation = 1 << i;
											break;
										}
									}
								}

								UDEBUG("edge length=%f decimation=%d", edgeLength, decimation);
							}
						}
						else
						{
							if(blendingDecimation > 1)
							{
								UASSERT(textureSize % blendingDecimation == 0);
							}
							decimation = blendingDecimation;
							UDEBUG("decimation=%d", decimation);
						}

						std::vector<cv::Mat> blendGains(materials);
						for(int i=0; i<materials;++i)
						{
							blendGains[i] = cv::Mat(globalTextures.rows/decimation, globalTextures.rows/decimation, CV_32FC3, cv::Scalar::all(1.0f));
						}

						for(unsigned int p=0; p<vertexToPixels.size(); ++p)
						{
							if(vertexToPixels[p].size() > 1)
							{
								std::vector<float> gainsB(vertexToPixels[p].size());
								std::vector<float> gainsG(vertexToPixels[p].size());
								std::vector<float> gainsR(vertexToPixels[p].size());
								float sumWeight = 0.0f;
								int k=0;
								for(std::map<int, pcl::PointXY>::const_iterator iter=vertexToPixels[p].begin(); iter!=vertexToPixels[p].end(); ++iter)
								{
									if(materialsKept.at(iter->first))
									{
										int u = iter->second.x*emptyImage.cols + imageOrigin[iter->first].x;
										int v = (1.0-iter->second.y)*emptyImage.rows + imageOrigin[iter->first].y;
										float x = iter->second.x - 0.5f;
										float y = iter->second.y - 0.5f;
										float weight = 0.7f - sqrt(x*x+y*y);
										if(weight<0.0f)
										{
											weight = 0.0f;
										}
										int indexMaterial = newCamIndex[iter->first] / (cols*rows);
										cv::Vec3b * pt = globalTextures.ptr<cv::Vec3b>(v,u+indexMaterial*globalTextures.rows);
										gainsB[k] = static_cast<double>(pt->val[0]) * weight;
										gainsG[k] = static_cast<double>(pt->val[1]) * weight;
										gainsR[k] = static_cast<double>(pt->val[2]) * weight;
										sumWeight += weight;
										++k;
									}
								}
								gainsB.resize(k);
								gainsG.resize(k);
								gainsR.resize(k);

								if(sumWeight > 0)
								{
									float targetColor[3];
									targetColor[0] = uSum(gainsB.data(), gainsB.size()) / sumWeight;
									targetColor[1] = uSum(gainsG.data(), gainsG.size()) / sumWeight;
									targetColor[2] = uSum(gainsR.data(), gainsR.size()) / sumWeight;
									for(std::map<int, pcl::PointXY>::const_iterator iter=vertexToPixels[p].begin(); iter!=vertexToPixels[p].end(); ++iter)
									{
										if(materialsKept.at(iter->first))
										{
											int u = iter->second.x*emptyImage.cols + imageOrigin[iter->first].x;
											int v = (1.0-iter->second.y)*emptyImage.rows + imageOrigin[iter->first].y;
											int indexMaterial = newCamIndex[iter->first] / (cols*rows);
											cv::Vec3b * pt = globalTextures.ptr<cv::Vec3b>(v,u+indexMaterial*globalTextures.rows);
											float gB = targetColor[0]/(pt->val[0]==0?1.0f:pt->val[0]);
											float gG = targetColor[1]/(pt->val[1]==0?1.0f:pt->val[1]);
											float gR = targetColor[2]/(pt->val[2]==0?1.0f:pt->val[2]);
											cv::Vec3f * ptr = blendGains[indexMaterial].ptr<cv::Vec3f>(v/decimation, u/decimation);
											ptr->val[0] = (gB>1.3f)?1.3f:(gB<0.7f)?0.7f:gB;
											ptr->val[1] = (gG>1.3f)?1.3f:(gG<0.7f)?0.7f:gG;
											ptr->val[2] = (gR>1.3f)?1.3f:(gR<0.7f)?0.7f:gR;
										}
									}
								}
							}
						}

						if(blendingGainsOut)
						{
							for(int t=0; t<(int)textures.size(); ++t)
							{
								//break;
								if(materialsKept.at(t))
								{
									int u = imageOrigin[t].x/decimation;
									int v = imageOrigin[t].y/decimation;

									int indexMaterial = newCamIndex[t] / (cols*rows);
									cv::Mat roi = blendGains[indexMaterial](cv::Rect(u, v, emptyImage.cols/decimation, emptyImage.rows/decimation));
									if(blendingGainsOut->find(textures[t].first) == blendingGainsOut->end())
									{
										std::map<int,cv::Mat> value;
										value.insert(std::make_pair(textures[t].second, roi.clone()));
										blendingGainsOut->insert(std::make_pair(textures[t].first, value));
									}
									else
									{
										blendingGainsOut->at(textures[t].first).insert(std::make_pair(textures[t].second, roi.clone()));
									}
								}
							}
						}

						for(int i=0; i<materials; ++i)
						{
							/*std::vector<cv::Mat> channels;
							cv::split(blendGains, channels);
							cv::Mat img;
							channels[0].convertTo(img,CV_8U,128.0,0);
							cv::imwrite("blendSmallB.png", img);
							channels[1].convertTo(img,CV_8U,128.0,0);
							cv::imwrite("blendSmallG.png", img);
							channels[2].convertTo(img,CV_8U,128.0,0);
							cv::imwrite("blendSmallR.png", img);*/

							cv::Mat globalTexturesROI = globalTextures(cv::Range::all(), cv::Range(i*globalTextures.rows, (i+1)*globalTextures.rows));
							cv::Mat dst;
							cv::blur(blendGains[i], dst, cv::Size(3,3));
							cv::resize(dst, blendGains[i], globalTexturesROI.size(), 0, 0, cv::INTER_LINEAR);

							/*cv::split(blendGains, channels);
							channels[0].convertTo(img,CV_8U,128.0,0);
							cv::imwrite("blendFullB.png", img);
							channels[1].convertTo(img,CV_8U,128.0,0);
							cv::imwrite("blendFullG.png", img);
							channels[2].convertTo(img,CV_8U,128.0,0);
							cv::imwrite("blendFullR.png", img);*/

							cv::multiply(globalTexturesROI, blendGains[i], globalTexturesROI, 1.0, CV_8UC3);

							//UWARN("Saving blending.png", globalTexture);
							//cv::imwrite("blending.png", globalTexture);
						}

						if(state) state->callback(uFormat("Blending (decimation=%d) %fs", decimation, timer.ticks()));
					}
				}

				if(brightnessContrastRatioLow > 0 || brightnessContrastRatioHigh > 0)
				{
					if(exposureFusion)
					{
						std::vector<cv::Mat> images;
						images.push_back(globalTextures);
						if (brightnessContrastRatioLow > 0)
						{
							images.push_back(util2d::brightnessAndContrastAuto(
									globalTextures,
									globalTextureMasks,
									(float)brightnessContrastRatioLow,
									0.0f));
						}
						if (brightnessContrastRatioHigh > 0)
						{
							images.push_back(util2d::brightnessAndContrastAuto(
									globalTextures,
									globalTextureMasks,
									0.0f,
									(float)brightnessContrastRatioHigh));
						}

						globalTextures = util2d::exposureFusion(images);
					}
					else
					{
						float alpha, beta;
						globalTextures = util2d::brightnessAndContrastAuto(
								globalTextures,
								globalTextureMasks,
								(float)brightnessContrastRatioLow,
								(float)brightnessContrastRatioHigh,
								&alpha,
								&beta);
						if(contrastValuesOut)
						{
							contrastValuesOut->first = alpha;
							contrastValuesOut->second = beta;
						}
					}
					if(state) state->callback(uFormat("Brightness and contrast auto %fs", timer.ticks()));
				}
			}
		}
	}
	UDEBUG("globalTextures=%d", globalTextures.cols?globalTextures.cols / globalTextures.rows:0);
	return globalTextures;
}

void fixTextureMeshForVisualization(pcl::TextureMesh & textureMesh)
{
	// VTK issue:
	//  tex_coordinates should be linked to points, not
	//  polygon vertices. Points linked to multiple different TCoords (different textures) should
	//  be duplicated.
	for (unsigned int t = 0; t < textureMesh.tex_coordinates.size(); ++t)
	{
		if(textureMesh.tex_polygons[t].size())
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromPCLPointCloud2(textureMesh.cloud, *originalCloud);

			// make a cloud with as many points than polygon vertices
			unsigned int nPoints = textureMesh.tex_coordinates[t].size();
			UASSERT(nPoints == textureMesh.tex_polygons[t].size()*textureMesh.tex_polygons[t][0].vertices.size()); // assuming polygon size is constant!

			pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
			newCloud->resize(nPoints);

			unsigned int oi = 0;
			for (unsigned int i = 0; i < textureMesh.tex_polygons[t].size(); ++i)
			{
				pcl::Vertices & vertices = textureMesh.tex_polygons[t][i];

				for(unsigned int j=0; j<vertices.vertices.size(); ++j)
				{
					UASSERT(oi < newCloud->size());
					UASSERT_MSG((size_t)vertices.vertices[j] < originalCloud->size(), uFormat("%d vs %d", vertices.vertices[j], (int)originalCloud->size()).c_str());
					newCloud->at(oi) = originalCloud->at(vertices.vertices[j]);
					vertices.vertices[j] = oi; // new vertex index
					++oi;
				}
			}
			pcl::toPCLPointCloud2(*newCloud, textureMesh.cloud);
		}
	}
}

bool multiBandTexturing(
		const std::string & outputOBJPath,
		const pcl::PCLPointCloud2 & cloud,
		const std::vector<pcl::Vertices> & polygons,
		const std::map<int, Transform> & cameraPoses,
		const std::vector<std::map<int, pcl::PointXY> > & vertexToPixels, // required output of util3d::createTextureMesh()
		const std::map<int, cv::Mat> & images,        // raw or compressed, can be empty if memory or dbDriver should be used
		const std::map<int, std::vector<CameraModel> > & cameraModels, // Should match images
		const Memory * memory,                    // Should be set if images are not set
		const DBDriver * dbDriver,                // Should be set if images and memory are not set
		int textureSize,
		const std::string & textureFormat,
		const std::map<int, std::map<int, cv::Vec4d> > & gains,       // optional output of util3d::mergeTextures()
		const std::map<int, std::map<int, cv::Mat> > & blendingGains, // optional output of util3d::mergeTextures()
		const std::pair<float, float> & contrastValues,               // optional output of util3d::mergeTextures()
		bool gainRGB)
{
	return multiBandTexturing(
			outputOBJPath,
			cloud,
			polygons,
			cameraPoses,
			vertexToPixels,
			images,
			cameraModels,
			memory,
			dbDriver,
			textureSize,
			2,
			"1 5 10 0",
			textureFormat,
			gains,
			blendingGains,
			contrastValues,
			gainRGB);
}

bool multiBandTexturing(
		const std::string & outputOBJPath,
		const pcl::PCLPointCloud2 & cloud,
		const std::vector<pcl::Vertices> & polygons,
		const std::map<int, Transform> & cameraPoses,
		const std::vector<std::map<int, pcl::PointXY> > & vertexToPixels,
		const std::map<int, cv::Mat> & images,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		const Memory * memory,
		const DBDriver * dbDriver,
		unsigned int textureSize,
		unsigned int textureDownScale,
		const std::string & nbContrib,
		const std::string & textureFormat,
		const std::map<int, std::map<int, cv::Vec4d> > & gains,
		const std::map<int, std::map<int, cv::Mat> > & blendingGains,
		const std::pair<float, float> & contrastValues,
		bool gainRGB,
		unsigned int unwrapMethod,
		bool fillHoles,
		unsigned int padding,
		double bestScoreThreshold,
		double angleHardThreshold,
		bool forceVisibleByAllVertices)
{

#ifdef RTABMAP_ALICE_VISION
	if(ULogger::level() == ULogger::kDebug)
	{
		system::Logger::get()->setLogLevel(system::EVerboseLevel::Trace);
	}
	else if(ULogger::level() == ULogger::kInfo)
	{
		system::Logger::get()->setLogLevel(system::EVerboseLevel::Info);
	}
	else if(ULogger::level() == ULogger::kWarning)
	{
		system::Logger::get()->setLogLevel(system::EVerboseLevel::Warning);
	}
	else
	{
		system::Logger::get()->setLogLevel(system::EVerboseLevel::Error);
	}

	sfmData::SfMData sfmData;
	pcl::PointCloud<pcl::PointXYZRGB> cloud2;
	pcl::fromPCLPointCloud2(cloud, cloud2);
	UASSERT(vertexToPixels.size() == cloud2.size());
	UINFO("Input mesh: %d points %d polygons", (int)cloud2.size(), (int)polygons.size());
	mesh::Texturing texturing;
#if RTABMAP_ALICE_VISION_MAJOR > 2 || (RTABMAP_ALICE_VISION_MAJOR==2 && RTABMAP_ALICE_VISION_MINOR>=3)
	texturing.mesh = new mesh::Mesh();
	texturing.mesh->pts.resize(cloud2.size());
	texturing.mesh->pointsVisibilities.resize(cloud2.size());
#else
	texturing.me = new mesh::Mesh();
	texturing.me->pts = new StaticVector<Point3d>(cloud2.size());
	texturing.pointsVisibilities = new mesh::PointsVisibility();
	texturing.pointsVisibilities->reserve(cloud2.size());
#endif
	texturing.texParams.textureSide = textureSize;
	texturing.texParams.downscale = textureDownScale;
	std::vector<int> multiBandNbContrib;
	std::list<std::string> values = uSplit(nbContrib, ' ');
	for(std::list<std::string>::iterator iter=values.begin(); iter!=values.end(); ++iter)
	{
		multiBandNbContrib.push_back(uStr2Int(*iter));
	}
	if(multiBandNbContrib.size() != 4)
	{
		UERROR("multiband: Wrong number of nb of contribution (vaue=\"%s\", should be 4), using default values instead.", nbContrib.c_str());
	}
	else
	{
		texturing.texParams.multiBandNbContrib = multiBandNbContrib;
	}
	texturing.texParams.padding = padding;
	texturing.texParams.fillHoles = fillHoles;
	texturing.texParams.bestScoreThreshold = bestScoreThreshold;
	texturing.texParams.angleHardThreshold = angleHardThreshold;
	texturing.texParams.forceVisibleByAllVertices = forceVisibleByAllVertices;
	texturing.texParams.visibilityRemappingMethod = mesh::EVisibilityRemappingMethod::Pull;


	for(size_t i=0;i<cloud2.size();++i)
	{
		pcl::PointXYZRGB pt = cloud2.at(i);
#if RTABMAP_ALICE_VISION_MAJOR > 2 || (RTABMAP_ALICE_VISION_MAJOR==2 && RTABMAP_ALICE_VISION_MINOR>=3)
		texturing.mesh->pointsVisibilities[i].reserve(vertexToPixels[i].size());
		for(std::map<int, pcl::PointXY>::const_iterator iter=vertexToPixels[i].begin(); iter!=vertexToPixels[i].end();++iter)
		{
			texturing.mesh->pointsVisibilities[i].push_back(iter->first);
		}
		texturing.mesh->pts[i] = Point3d(pt.x, pt.y, pt.z);
#else
		mesh::PointVisibility* pointVisibility = new mesh::PointVisibility();
		pointVisibility->reserve(vertexToPixels[i].size());
		for(std::map<int, pcl::PointXY>::const_iterator iter=vertexToPixels[i].begin(); iter!=vertexToPixels[i].end();++iter)
		{
			pointVisibility->push_back(iter->first);
		}
		texturing.pointsVisibilities->push_back(pointVisibility);
		(*texturing.me->pts)[i] = Point3d(pt.x, pt.y, pt.z);
#endif
	}

#if RTABMAP_ALICE_VISION_MAJOR > 2 || (RTABMAP_ALICE_VISION_MAJOR==2 && RTABMAP_ALICE_VISION_MINOR>=3)
	texturing.mesh->tris.resize(polygons.size());
	texturing.mesh->trisMtlIds().resize(polygons.size());
#else
	texturing.me->tris = new StaticVector<mesh::Mesh::triangle>(polygons.size());
#endif
	for(size_t i=0;i<polygons.size();++i)
	{
		UASSERT(polygons[i].vertices.size() == 3);
#if RTABMAP_ALICE_VISION_MAJOR > 2 || (RTABMAP_ALICE_VISION_MAJOR==2 && RTABMAP_ALICE_VISION_MINOR>=3)
		texturing.mesh->trisMtlIds()[i] = -1;
		texturing.mesh->tris[i] = mesh::Mesh::triangle(
#else
		(*texturing.me->tris)[i] = mesh::Mesh::triangle(
#endif
				polygons[i].vertices[0],
				polygons[i].vertices[1],
				polygons[i].vertices[2]);
	}
	UTimer timer;
	std::string outputDirectory = UDirectory::getDir(outputOBJPath);
	std::string tmpImageDirectory = outputDirectory+"/rtabmap_tmp_textures";
	UDirectory::removeDir(tmpImageDirectory);
	UDirectory::makeDir(tmpImageDirectory);
	UINFO("Temporary saving images in directory \"%s\"...", tmpImageDirectory.c_str());
	int viewId = 0;
	for(std::map<int, Transform>::const_iterator iter = cameraPoses.lower_bound(1); iter!=cameraPoses.end(); ++iter)
	{
		int camId = iter->first;
		cv::Mat image;
		std::vector<CameraModel> models;

		if( images.find(camId) != images.end() &&
			!images.find(camId)->second.empty() &&
			cameraModels.find(camId) != cameraModels.end())
		{
			image = images.find(camId)->second;
			models = cameraModels.find(camId)->second;
		}
		else if(memory)
		{
			SensorData data = memory->getNodeData(camId, true, false, false, false);
			models = data.cameraModels();
			if(models.empty() && data.stereoCameraModel().isValidForProjection())
			{
				models.push_back(data.stereoCameraModel().left());
			}
			if(data.imageRaw().empty())
			{
				image = data.imageCompressed();
			}
			else
			{
				image = data.imageRaw();
			}
		}
		else if(dbDriver)
		{
			StereoCameraModel stereoModel;
			dbDriver->getCalibration(camId, models, stereoModel);
			if(models.empty() && stereoModel.isValidForProjection())
			{
				models.push_back(stereoModel.left());
			}

			SensorData data;
			dbDriver->getNodeData(camId, data, true, false, false, false);
			if(data.imageRaw().empty())
			{
				image = data.imageCompressed();
			}
			else
			{
				image = data.imageRaw();
			}
		}
		if(models.empty())
		{
			UERROR("No camera models found for camera %d. Aborting multiband texturing...", iter->first);
			return false;
		}
		if(image.empty())
		{
			UERROR("No image found for camera %d. Aborting multiband texturing...", iter->first);
			return false;
		}

		if(image.rows == 1 && image.type() == CV_8UC1)
		{
			image = uncompressImage(image);
		}
		else
		{
			image = image.clone();
		}

		for(size_t i=0; i<models.size(); ++i)
		{
			const CameraModel & model = models.at(i);
			cv::Size imageSize = model.imageSize();
			if(imageSize.height == 0)
			{
				// backward compatibility
				imageSize.height = image.rows;
				imageSize.width = image.cols;
			}
			UASSERT(image.cols % imageSize.width == 0);
			cv::Mat imageRoi = image.colRange(i*imageSize.width, (i+1)*imageSize.width);
			if(gains.find(camId) != gains.end() &&
			   gains.at(camId).find(i) != gains.at(camId).end())
			{
				const cv::Vec4d & g = gains.at(camId).at(i);
				std::vector<cv::Mat> channels;
				cv::split(imageRoi, channels);

				// assuming BGR
				cv::multiply(channels[0], g.val[gainRGB?3:0], channels[0]);
				cv::multiply(channels[1], g.val[gainRGB?2:0], channels[1]);
				cv::multiply(channels[2], g.val[gainRGB?1:0], channels[2]);

				cv::Mat output;
				cv::merge(channels, output);
				imageRoi = output;
			}

			if(blendingGains.find(camId) != blendingGains.end() &&
			   blendingGains.at(camId).find(i) != blendingGains.at(camId).end())
			{
				cv::Mat g = blendingGains.at(camId).at(i);
				cv::Mat dst;
				cv::blur(g, dst, cv::Size(3,3));
				cv::Mat gResized;
				cv::resize(dst, gResized, imageRoi.size(), 0, 0, cv::INTER_LINEAR);
				cv::Mat output;
				cv::multiply(imageRoi, gResized, output, 1.0, CV_8UC3);
				imageRoi = output;
			}

			Transform t = (iter->second * model.localTransform()).inverse();
			Eigen::Matrix<double, 3, 4> m = t.toEigen3d().matrix().block<3,4>(0, 0);
			sfmData::CameraPose pose(geometry::Pose3(m), true);
			sfmData.setAbsolutePose((IndexT)viewId, pose);

			UDEBUG("%d %d %f %f %f %f", imageSize.width, imageSize.height, model.fx(), model.fy(), model.cx(), model.cy());
			std::shared_ptr<camera::IntrinsicBase> camPtr = std::make_shared<camera::Pinhole>(
#if RTABMAP_ALICE_VISION_MAJOR > 2 || (RTABMAP_ALICE_VISION_MAJOR==2 && RTABMAP_ALICE_VISION_MINOR>=4)
					//https://github.com/alicevision/AliceVision/commit/9fab5c79a1c65595fe5c5001267e1c5212bc93f0#diff-b0c0a3c30de50be8e4ed283dfe4c8ae4a9bc861aa9a83bd8bfda8182e9d67c08
					// [all] the camera principal point is now defined as an offset relative to the image center
					imageSize.width, imageSize.height, model.fx(), model.fy(), model.cx() - double(imageSize.width) * 0.5, model.cy() - double(imageSize.height) * 0.5);
#else
					imageSize.width, imageSize.height, model.fx(), model.cx(), model.cy());
#endif
			sfmData.intrinsics.insert(std::make_pair((IndexT)viewId, camPtr));

			std::string imagePath = tmpImageDirectory+uFormat("/%d.jpg", viewId);

			cv::imwrite(imagePath, imageRoi);

			std::shared_ptr<sfmData::View> viewPtr = std::make_shared<sfmData::View>(
					imagePath,
					(IndexT)viewId,
					(IndexT)viewId,
					(IndexT)viewId,
					imageSize.width,
					imageSize.height);
			sfmData.views.insert(std::make_pair((IndexT)viewId, viewPtr));
			++viewId;
		}
	}
	UINFO("Temporary saving images in directory \"%s\"... done (%d images). %fs", tmpImageDirectory.c_str(), viewId, (int)cameraPoses.size(), timer.ticks());

	mvsUtils::MultiViewParams mp(sfmData);

	UINFO("Unwrapping (method=%d=%s)...", unwrapMethod, mesh::EUnwrapMethod_enumToString((mesh::EUnwrapMethod)unwrapMethod).c_str());
	texturing.unwrap(mp, (mesh::EUnwrapMethod)unwrapMethod);
	UINFO("Unwrapping done. %fs", timer.ticks());

	// save final obj file
	std::string baseName = uSplit(UFile::getName(outputOBJPath), '.').front();
#if RTABMAP_ALICE_VISION_MAJOR > 2 || (RTABMAP_ALICE_VISION_MAJOR==2 && RTABMAP_ALICE_VISION_MINOR>=4)
	texturing.saveAs(outputDirectory, baseName, aliceVision::mesh::EFileType::OBJ, imageIO::EImageFileType::PNG);
#else
	texturing.saveAsOBJ(outputDirectory, baseName);
#endif
	UINFO("Saved %s. %fs", outputOBJPath.c_str(), timer.ticks());

	// generate textures
	UINFO("Generating textures...");
	texturing.generateTextures(mp, outputDirectory);
	UINFO("Generating textures done. %fs", timer.ticks());

	UINFO("Cleanup temporary directory \"%s\"...", tmpImageDirectory.c_str());
	UDirectory dir(tmpImageDirectory);
	std::string fp = dir.getNextFilePath();
	while(!fp.empty())
	{
		UFile::erase(fp);
		fp = dir.getNextFilePath();
	}
	UDirectory::removeDir(tmpImageDirectory);
	UINFO("Cleanup temporary directory \"%s\"... done.", tmpImageDirectory.c_str());

	UINFO("Rename/convert textures...");
	dir.setPath(outputDirectory, "png");
	std::map<std::string, std::string> texNames; // <old, new>
	std::string outputFormat = textureFormat;
	if(outputFormat.front() == '.')
	{
		outputFormat = outputFormat.substr(1, std::string::npos);
	}
	for(std::list<std::string>::const_iterator iter=dir.getFileNames().begin(); iter!=dir.getFileNames().end(); ++iter)
	{
		// Textures are called "texture_1001.png", "texture_1002.png", ...
		if(uStrContains(*iter, "texture_10"))
		{
			cv::Mat img = cv::imread(outputDirectory+"/"+*iter);
			if(contrastValues.first != 0.0f || contrastValues.second != 0.0f)
			{
				UASSERT(img.channels() == 3);
				// Re-use same contrast values with all images
				UINFO("Apply contrast values %f %f", contrastValues.first, contrastValues.second);
				img.convertTo(img, -1, contrastValues.first, contrastValues.second);
			}
			std::string newName = *iter;
			boost::replace_all(newName, "png", outputFormat);
			boost::replace_all(newName, "texture", baseName);
			texNames.insert(std::make_pair(*iter, newName));
			cv::imwrite(outputDirectory+"/"+newName, img);
			UFile::erase(outputDirectory+"/"+*iter);
		}
	}
	std::ifstream fi(outputDirectory+"/"+baseName+".mtl");
	std::string mtlStr((std::istreambuf_iterator<char>(fi)),
	                 std::istreambuf_iterator<char>());
	fi.close();
	UFile::erase(outputDirectory+"/"+baseName);
	for(std::map<std::string, std::string>::iterator iter=texNames.begin(); iter!=texNames.end(); ++iter)
	{
		boost::replace_all(mtlStr, iter->first, iter->second);
	}
	std::ofstream fo(outputDirectory+"/"+baseName+".mtl");
	fo.write(mtlStr.c_str(), mtlStr.size());
	fo.close();
	UINFO("Rename/convert textures... done. %fs", timer.ticks());

#if RTABMAP_ALICE_VISION_MAJOR > 2 || (RTABMAP_ALICE_VISION_MAJOR==2 && RTABMAP_ALICE_VISION_MINOR>=3)
	UINFO("Cleanup sfmdata...");
	sfmData.clear();
	UINFO("Cleanup sfmdata... done. %fs", timer.ticks());
#endif

	return true;
#else
	UERROR("Cannot unwrap texture mesh. RTAB-Map is not built with Alice Vision support! Returning false.");
	return false;
#endif
}

LaserScan computeNormals(
		const LaserScan & laserScan,
		int searchK,
		float searchRadius)
{
	if(laserScan.isEmpty())
	{
		return laserScan;
	}

	pcl::PointCloud<pcl::Normal>::Ptr normals;
	// convert to compatible PCL format and filter it
	if(laserScan.hasRGB())
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = laserScanToPointCloudRGB(laserScan);
		if(cloud->size())
		{
			UASSERT(!laserScan.is2d());
			pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, searchK, searchRadius);
			return LaserScan(laserScanFromPointCloud(*cloud, *normals), laserScan.maxPoints(), laserScan.rangeMax(), laserScan.localTransform());
		}
	}
	else if(laserScan.hasIntensity())
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = laserScanToPointCloudI(laserScan);
		if(cloud->size())
		{
			if(laserScan.is2d())
			{
				pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals2D(cloud, searchK, searchRadius);
				if(laserScan.angleIncrement() > 0.0f)
				{
					return LaserScan(laserScan2dFromPointCloud(*cloud, *normals), laserScan.rangeMin(), laserScan.rangeMax(), laserScan.angleMin(), laserScan.angleMax(), laserScan.angleIncrement(), laserScan.localTransform());
				}
				else
				{
					return LaserScan(laserScan2dFromPointCloud(*cloud, *normals), laserScan.maxPoints(), laserScan.rangeMax(), laserScan.localTransform());
				}

			}
			else
			{
				pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, searchK, searchRadius);
				return LaserScan(laserScanFromPointCloud(*cloud, *normals), laserScan.maxPoints(), laserScan.rangeMax(), laserScan.localTransform());
			}
		}
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = laserScanToPointCloud(laserScan);
		if(cloud->size())
		{
			if(laserScan.is2d())
			{
				pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals2D(cloud, searchK, searchRadius);
				if(laserScan.angleIncrement() > 0.0f)
				{
					return LaserScan(laserScan2dFromPointCloud(*cloud, *normals), laserScan.rangeMin(), laserScan.rangeMax(), laserScan.angleMin(), laserScan.angleMax(), laserScan.angleIncrement(), laserScan.localTransform());
				}
				else
				{
					return LaserScan(laserScan2dFromPointCloud(*cloud, *normals), laserScan.maxPoints(), laserScan.rangeMax(), laserScan.localTransform());
				}
			}
			else
			{
				pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, searchK, searchRadius);
				return LaserScan(laserScanFromPointCloud(*cloud, *normals), laserScan.maxPoints(), laserScan.rangeMax(), laserScan.localTransform());
			}
		}
	}
	return LaserScan();
}

template<typename PointT>
pcl::PointCloud<pcl::Normal>::Ptr computeNormalsImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	if(indices->size())
	{
		tree->setInputCloud(cloud, indices);
	}
	else
	{
		tree->setInputCloud (cloud);
	}

	// Normal estimation*
#ifdef PCL_OMP
	pcl::NormalEstimationOMP<PointT, pcl::Normal> n;
#else
	pcl::NormalEstimation<PointT, pcl::Normal> n;
#endif
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	n.setInputCloud (cloud);
	// Commented: Keep the output normals size the same as the input cloud
	//if(indices->size())
	//{
	//	n.setIndices(indices);
	//}
	n.setSearchMethod (tree);
	n.setKSearch (searchK);
	n.setRadiusSearch (searchRadius);
	n.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
	n.compute (*normals);

	return normals;
}
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return computeNormals(cloud, indices, searchK, searchRadius, viewPoint);
}
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return computeNormals(cloud, indices, searchK, searchRadius, viewPoint);
}
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return computeNormals(cloud, indices, searchK, searchRadius, viewPoint);
}
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	return computeNormalsImpl<pcl::PointXYZ>(cloud, indices, searchK, searchRadius, viewPoint);
}
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	return computeNormalsImpl<pcl::PointXYZRGB>(cloud, indices, searchK, searchRadius, viewPoint);
}
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	return computeNormalsImpl<pcl::PointXYZI>(cloud, indices, searchK, searchRadius, viewPoint);
}

template<typename PointT>
pcl::PointCloud<pcl::Normal>::Ptr computeNormals2DImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	UASSERT(searchK>0 || searchRadius>0.0f);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud);

	normals->resize(cloud->size());

	float bad_point = std::numeric_limits<float>::quiet_NaN ();

	// assuming that points are ordered
	for(unsigned int i=0; i<cloud->size(); ++i)
	{
		const PointT & pt = cloud->at(i);
		std::vector<Eigen::Vector3f> neighborNormals;
		Eigen::Vector3f direction;
		direction[0] = viewPoint[0] - pt.x;
		direction[1] = viewPoint[1] - pt.y;
		direction[2] = viewPoint[2] - pt.z;

		std::vector<int> k_indices;
		std::vector<float> k_sqr_distances;
		if(searchRadius>0.0f)
		{
			tree->radiusSearch(cloud->at(i), searchRadius, k_indices, k_sqr_distances, searchK);
		}
		else
		{
			tree->nearestKSearch(cloud->at(i), searchK, k_indices, k_sqr_distances);
		}

		for(unsigned int j=0; j<k_indices.size(); ++j)
		{
			if(k_indices.at(j) != (int)i)
			{
				const PointT & pt2 = cloud->at(k_indices.at(j));
				Eigen::Vector3f v(pt2.x-pt.x, pt2.y - pt.y, pt2.z - pt.z);
				Eigen::Vector3f up = v.cross(direction);
				Eigen::Vector3f n = up.cross(v);
				n.normalize();
				neighborNormals.push_back(n);
			}
		}

		if(neighborNormals.empty())
		{
			normals->at(i).normal_x = bad_point;
			normals->at(i).normal_y = bad_point;
			normals->at(i).normal_z = bad_point;
		}
		else
		{
			Eigen::Vector3f meanNormal(0,0,0);
			for(unsigned int j=0; j<neighborNormals.size(); ++j)
			{
				meanNormal+=neighborNormals[j];
			}
			meanNormal /= (float)neighborNormals.size();
			meanNormal.normalize();
			normals->at(i).normal_x = meanNormal[0];
			normals->at(i).normal_y = meanNormal[1];
			normals->at(i).normal_z = meanNormal[2];
		}
	}

	return normals;
}
pcl::PointCloud<pcl::Normal>::Ptr computeNormals2D(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	return computeNormals2DImpl<pcl::PointXYZ>(cloud, searchK, searchRadius, viewPoint);
}
pcl::PointCloud<pcl::Normal>::Ptr computeNormals2D(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	return computeNormals2DImpl<pcl::PointXYZI>(cloud, searchK, searchRadius, viewPoint);
}

template<typename PointT>
pcl::PointCloud<pcl::Normal>::Ptr computeFastOrganizedNormals2DImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	UASSERT(searchK>0);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

	normals->resize(cloud->size());
	searchRadius *= searchRadius; // squared distance

	float bad_point = std::numeric_limits<float>::quiet_NaN ();

	// assuming that points are ordered
	for(int i=0; i<(int)cloud->size(); ++i)
	{
		int li = i-searchK;
		if(li<0)
		{
			li=0;
		}
		int hi = i+searchK;
		if(hi>=(int)cloud->size())
		{
			hi=(int)cloud->size()-1;
		}

		// get points before not too far
		const PointT & pt = cloud->at(i);
		std::vector<Eigen::Vector3f> neighborNormals;
		Eigen::Vector3f direction;
		direction[0] = viewPoint[0] - cloud->at(i).x;
		direction[1] = viewPoint[1] - cloud->at(i).y;
		direction[2] = viewPoint[2] - cloud->at(i).z;
		for(int j=i-1; j>=li; --j)
		{
			const PointT & pt2 = cloud->at(j);
			Eigen::Vector3f vd(pt2.x-pt.x, pt2.y - pt.y, pt2.z - pt.z);
			if(searchRadius<=0.0f || (vd[0]*vd[0] + vd[1]*vd[1] + vd[2]*vd[2]) < searchRadius)
			{
				Eigen::Vector3f v(pt2.x-pt.x, pt2.y - pt.y, pt2.z - pt.z);
				Eigen::Vector3f up = v.cross(direction);
				Eigen::Vector3f n = up.cross(v);
				n.normalize();
				neighborNormals.push_back(n);
			}
			else
			{
				break;
			}
		}
		for(int j=i+1; j<=hi; ++j)
		{
			const PointT & pt2 = cloud->at(j);
			Eigen::Vector3f vd(pt2.x-pt.x, pt2.y - pt.y, pt2.z - pt.z);
			if(searchRadius<=0.0f || (vd[0]*vd[0] + vd[1]*vd[1] + vd[2]*vd[2]) < searchRadius)
			{
				Eigen::Vector3f v(pt2.x-pt.x, pt2.y - pt.y, pt2.z - pt.z);
				Eigen::Vector3f up = v[2]==0.0f?Eigen::Vector3f(0,0,1):v.cross(direction);
				Eigen::Vector3f n = up.cross(v);
				n.normalize();
				neighborNormals.push_back(n);
			}
			else
			{
				break;
			}
		}

		if(neighborNormals.empty())
		{
			normals->at(i).normal_x = bad_point;
			normals->at(i).normal_y = bad_point;
			normals->at(i).normal_z = bad_point;
		}
		else
		{
			Eigen::Vector3f meanNormal(0,0,0);
			for(unsigned int j=0; j<neighborNormals.size(); ++j)
			{
				meanNormal+=neighborNormals[j];
			}
			meanNormal /= (float)neighborNormals.size();
			meanNormal.normalize();
			normals->at(i).normal_x = meanNormal[0];
			normals->at(i).normal_y = meanNormal[1];
			normals->at(i).normal_z = meanNormal[2];
		}
	}

	return normals;
}

pcl::PointCloud<pcl::Normal>::Ptr computeFastOrganizedNormals2D(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	return computeFastOrganizedNormals2DImpl<pcl::PointXYZ>(cloud, searchK, searchRadius, viewPoint);
}
pcl::PointCloud<pcl::Normal>::Ptr computeFastOrganizedNormals2D(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
{
	return computeFastOrganizedNormals2DImpl<pcl::PointXYZI>(cloud, searchK, searchRadius, viewPoint);
}

pcl::PointCloud<pcl::Normal>::Ptr computeFastOrganizedNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float maxDepthChangeFactor,
		float normalSmoothingSize,
		const Eigen::Vector3f & viewPoint)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return computeFastOrganizedNormals(cloud, indices, maxDepthChangeFactor, normalSmoothingSize, viewPoint);
}
pcl::PointCloud<pcl::Normal>::Ptr computeFastOrganizedNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float maxDepthChangeFactor,
		float normalSmoothingSize,
		const Eigen::Vector3f & viewPoint)
{
	UASSERT(cloud->isOrganized());

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	if(indices->size())
	{
		tree->setInputCloud(cloud, indices);
	}
	else
	{
		tree->setInputCloud (cloud);
	}

	// Normal estimation
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(maxDepthChangeFactor);
	ne.setNormalSmoothingSize(normalSmoothingSize);
	ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);
	ne.setInputCloud(cloud);
	// Commented: Keep the output normals size the same as the input cloud
	//if(indices->size())
	//{
	//	ne.setIndices(indices);
	//}
	ne.setSearchMethod(tree);
	ne.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
	ne.compute(*normals);

	return normals;
}

float computeNormalsComplexity(
		const LaserScan & scan,
		const Transform & t,
		cv::Mat * pcaEigenVectors,
		cv::Mat * pcaEigenValues)
{
	if(!scan.isEmpty() && (scan.hasNormals()))
	{
		 //Construct a buffer used by the pca analysis
		int sz = static_cast<int>(scan.size()*2);
		bool is2d = scan.is2d();
		cv::Mat data_normals = cv::Mat::zeros(sz, is2d?2:3, CV_32FC1);
		int oi = 0;
		int nOffset = scan.getNormalsOffset();
		bool doTransform = false;
		Transform tn;
		if(!t.isIdentity() || !scan.localTransform().isIdentity())
		{
			tn = (t*scan.localTransform()).rotation();
			doTransform = true;
		}
		for (int i = 0; i < scan.size(); ++i)
		{
			const float * ptrScan = scan.data().ptr<float>(0, i);

			if(is2d)
			{
				if(uIsFinite(ptrScan[nOffset]) && uIsFinite(ptrScan[nOffset+1]))
				{
					cv::Point3f n(ptrScan[nOffset], ptrScan[nOffset+1], 0);
					if(doTransform)
					{
						n = util3d::transformPoint(n, tn);
					}
					float * ptr = data_normals.ptr<float>(oi++, 0);
					ptr[0] = n.x;
					ptr[1] = n.y;
				}
			}
			else
			{
				if(uIsFinite(ptrScan[nOffset]) && uIsFinite(ptrScan[nOffset+1]) && uIsFinite(ptrScan[nOffset+2]))
				{
					cv::Point3f n(ptrScan[nOffset], ptrScan[nOffset+1], ptrScan[nOffset+2]);
					if(doTransform)
					{
						n = util3d::transformPoint(n, tn);
					}
					float * ptr = data_normals.ptr<float>(oi++, 0);
					ptr[0] = n.x;
					ptr[1] = n.y;
					ptr[2] = n.z;
				}
			}
		}
		if(oi>1)
		{
			cv::PCA pca_analysis(cv::Mat(data_normals, cv::Range(0, oi*2)), cv::Mat(), CV_PCA_DATA_AS_ROW);

			if(pcaEigenVectors)
			{
				*pcaEigenVectors = pca_analysis.eigenvectors;
			}
			if(pcaEigenValues)
			{
				*pcaEigenValues = pca_analysis.eigenvalues;
			}
			UASSERT((is2d && pca_analysis.eigenvalues.total()>=2) || (!is2d && pca_analysis.eigenvalues.total()>=3));
			// Get last eigen value, scale between 0 and 1: 0=low complexity, 1=high complexity
			return pca_analysis.eigenvalues.at<float>(0, is2d?1:2)*(is2d?2.0f:3.0f);
		}
	}
	else if(!scan.isEmpty())
	{
		UERROR("Scan doesn't have normals!");
	}
	return 0.0f;
}

float computeNormalsComplexity(
		const pcl::PointCloud<pcl::PointNormal> & cloud,
		const Transform & t,
		bool is2d,
		cv::Mat * pcaEigenVectors,
		cv::Mat * pcaEigenValues)
{
	 //Construct a buffer used by the pca analysis
	int sz = static_cast<int>(cloud.size()*2);
	cv::Mat data_normals = cv::Mat::zeros(sz, is2d?2:3, CV_32FC1);
	int oi = 0;
	bool doTransform = false;
	Transform tn;
	if(!t.isIdentity() && !t.isNull())
	{
		tn = t.rotation();
		doTransform = true;
	}
	for (unsigned int i = 0; i < cloud.size(); ++i)
	{
		const pcl::PointNormal & pt = cloud.at(i);
		cv::Point3f n(pt.normal_x, pt.normal_y, pt.normal_z);
		if(doTransform)
		{
			n = util3d::transformPoint(n, tn);
		}
		if(uIsFinite(pt.normal_x) && uIsFinite(pt.normal_y) && uIsFinite(pt.normal_z))
		{
			float * ptr = data_normals.ptr<float>(oi++, 0);
			ptr[0] = n.x;
			ptr[1] = n.y;
			if(!is2d)
			{
				ptr[2] = n.z;
			}
		}
	}
	if(oi>1)
	{
		cv::PCA pca_analysis(cv::Mat(data_normals, cv::Range(0, oi*2)), cv::Mat(), CV_PCA_DATA_AS_ROW);

		if(pcaEigenVectors)
		{
			*pcaEigenVectors = pca_analysis.eigenvectors;
		}
		if(pcaEigenValues)
		{
			*pcaEigenValues = pca_analysis.eigenvalues;
		}

		// Get last eigen value, scale between 0 and 1: 0=low complexity, 1=high complexity
		return pca_analysis.eigenvalues.at<float>(0, is2d?1:2)*(is2d?2.0f:3.0f);
	}
	return 0.0f;
}

float computeNormalsComplexity(
		const pcl::PointCloud<pcl::Normal> & normals,
		const Transform & t,
		bool is2d,
		cv::Mat * pcaEigenVectors,
		cv::Mat * pcaEigenValues)
{
	 //Construct a buffer used by the pca analysis
	int sz = static_cast<int>(normals.size()*2);
	cv::Mat data_normals = cv::Mat::zeros(sz, is2d?2:3, CV_32FC1);
	int oi = 0;
	bool doTransform = false;
	Transform tn;
	if(!t.isIdentity())
	{
		tn = t.rotation();
		doTransform = true;
	}
	for (unsigned int i = 0; i < normals.size(); ++i)
	{
		const pcl::Normal & pt = normals.at(i);
		cv::Point3f n(pt.normal_x, pt.normal_y, pt.normal_z);
		if(doTransform)
		{
			n = util3d::transformPoint(n, tn);
		}
		if(uIsFinite(pt.normal_x) && uIsFinite(pt.normal_y) && uIsFinite(pt.normal_z))
		{
			float * ptr = data_normals.ptr<float>(oi++, 0);
			ptr[0] = n.x;
			ptr[1] = n.y;
			if(!is2d)
			{
				ptr[2] = n.z;
			}
		}
	}
	if(oi>1)
	{
		cv::PCA pca_analysis(cv::Mat(data_normals, cv::Range(0, oi*2)), cv::Mat(), CV_PCA_DATA_AS_ROW);

		if(pcaEigenVectors)
		{
			*pcaEigenVectors = pca_analysis.eigenvectors;
		}
		if(pcaEigenValues)
		{
			*pcaEigenValues = pca_analysis.eigenvalues;
		}

		// Get last eigen value, scale between 0 and 1: 0=low complexity, 1=high complexity
		return pca_analysis.eigenvalues.at<float>(0, is2d?1:2)*(is2d?2.0f:3.0f);
	}
	return 0.0f;
}

float computeNormalsComplexity(
		const pcl::PointCloud<pcl::PointXYZINormal> & cloud,
		const Transform & t,
		bool is2d,
		cv::Mat * pcaEigenVectors,
		cv::Mat * pcaEigenValues)
{
	 //Construct a buffer used by the pca analysis
	int sz = static_cast<int>(cloud.size()*2);
	cv::Mat data_normals = cv::Mat::zeros(sz, is2d?2:3, CV_32FC1);
	int oi = 0;
	bool doTransform = false;
	Transform tn;
	if(!t.isIdentity())
	{
		tn = t.rotation();
		doTransform = true;
	}
	for (unsigned int i = 0; i < cloud.size(); ++i)
	{
		const pcl::PointXYZINormal & pt = cloud.at(i);
		cv::Point3f n(pt.normal_x, pt.normal_y, pt.normal_z);
		if(doTransform)
		{
			n = util3d::transformPoint(n, tn);
		}
		if(uIsFinite(pt.normal_x) && uIsFinite(pt.normal_y) && uIsFinite(pt.normal_z))
		{
			float * ptr = data_normals.ptr<float>(oi++, 0);
			ptr[0] = n.x;
			ptr[1] = n.y;
			if(!is2d)
			{
				ptr[2] = n.z;
			}
		}
	}
	if(oi>1)
	{
		cv::PCA pca_analysis(cv::Mat(data_normals, cv::Range(0, oi*2)), cv::Mat(), CV_PCA_DATA_AS_ROW);

		if(pcaEigenVectors)
		{
			*pcaEigenVectors = pca_analysis.eigenvectors;
		}
		if(pcaEigenValues)
		{
			*pcaEigenValues = pca_analysis.eigenvalues;
		}

		// Get last eigen value, scale between 0 and 1: 0=low complexity, 1=high complexity
		return pca_analysis.eigenvalues.at<float>(0, is2d?1:2)*(is2d?2.0f:3.0f);
	}
	return 0.0f;
}

float computeNormalsComplexity(
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud,
		const Transform & t,
		bool is2d,
		cv::Mat * pcaEigenVectors,
		cv::Mat * pcaEigenValues)
{
	 //Construct a buffer used by the pca analysis
	int sz = static_cast<int>(cloud.size()*2);
	cv::Mat data_normals = cv::Mat::zeros(sz, is2d?2:3, CV_32FC1);
	int oi = 0;
	bool doTransform = false;
	Transform tn;
	if(!t.isIdentity())
	{
		tn = t.rotation();
		doTransform = true;
	}
	for (unsigned int i = 0; i < cloud.size(); ++i)
	{
		const pcl::PointXYZRGBNormal & pt = cloud.at(i);
		cv::Point3f n(pt.normal_x, pt.normal_y, pt.normal_z);
		if(doTransform)
		{
			n = util3d::transformPoint(n, tn);
		}
		if(uIsFinite(pt.normal_x) && uIsFinite(pt.normal_y) && uIsFinite(pt.normal_z))
		{
			float * ptr = data_normals.ptr<float>(oi++, 0);
			ptr[0] = n.x;
			ptr[1] = n.y;
			if(!is2d)
			{
				ptr[2] = n.z;
			}
		}
	}
	if(oi>1)
	{
		cv::PCA pca_analysis(cv::Mat(data_normals, cv::Range(0, oi*2)), cv::Mat(), CV_PCA_DATA_AS_ROW);

		if(pcaEigenVectors)
		{
			*pcaEigenVectors = pca_analysis.eigenvectors;
		}
		if(pcaEigenValues)
		{
			*pcaEigenValues = pca_analysis.eigenvalues;
		}

		// Get last eigen value, scale between 0 and 1: 0=low complexity, 1=high complexity
		return pca_analysis.eigenvalues.at<float>(0, is2d?1:2)*(is2d?2.0f:3.0f);
	}
	return 0.0f;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float searchRadius,
		int polygonialOrder,
		int upsamplingMethod, // NONE, DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
		float upsamplingRadius,      // SAMPLE_LOCAL_PLANE
		float upsamplingStep,        // SAMPLE_LOCAL_PLANE
		int pointDensity,             // RANDOM_UNIFORM_DENSITY
		float dilationVoxelSize,     // VOXEL_GRID_DILATION
		int dilationIterations)       // VOXEL_GRID_DILATION
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return mls(cloud,
			indices,
			searchRadius,
			polygonialOrder,
			upsamplingMethod,
			upsamplingRadius,
			upsamplingStep,
			pointDensity,
			dilationVoxelSize,
			dilationIterations);
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float searchRadius,
		int polygonialOrder,
		int upsamplingMethod, // NONE, DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
		float upsamplingRadius,      // SAMPLE_LOCAL_PLANE
		float upsamplingStep,        // SAMPLE_LOCAL_PLANE
		int pointDensity,             // RANDOM_UNIFORM_DENSITY
		float dilationVoxelSize,     // VOXEL_GRID_DILATION
		int dilationIterations)       // VOXEL_GRID_DILATION
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	if(indices->size())
	{
		tree->setInputCloud (cloud, indices);
	}
	else
	{
		tree->setInputCloud (cloud);
	}

	// Init object (second point type is for the normals)
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;

	// Set parameters
	mls.setComputeNormals (true);
	if(polygonialOrder > 0)
	{
#if PCL_VERSION_COMPARE(<, 1, 10, 0)
		mls.setPolynomialFit (true);
#endif
		mls.setPolynomialOrder(polygonialOrder);
	}
	else
	{
#if PCL_VERSION_COMPARE(<, 1, 10, 0)
		mls.setPolynomialFit (false);
#else
		mls.setPolynomialOrder(1);
#endif
	}
	UASSERT(upsamplingMethod >= mls.NONE &&
			upsamplingMethod <= mls.VOXEL_GRID_DILATION);
	mls.setUpsamplingMethod((pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>::UpsamplingMethod)upsamplingMethod);
	mls.setSearchRadius(searchRadius);
	mls.setUpsamplingRadius(upsamplingRadius);
	mls.setUpsamplingStepSize(upsamplingStep);
	mls.setPointDensity(pointDensity);
	mls.setDilationVoxelSize(dilationVoxelSize);
	mls.setDilationIterations(dilationIterations);

	// Reconstruct
	mls.setInputCloud (cloud);
	if(indices->size())
	{
		mls.setIndices(indices);
	}
	mls.setSearchMethod (tree);
	mls.process (*cloud_with_normals);

	// It seems that returned normals are not normalized!? FIXME: Is it a bug only in PCL 1.7.1?
	for(unsigned int i=0; i<cloud_with_normals->size(); ++i)
	{
		Eigen::Vector3f normal(cloud_with_normals->at(i).normal_x, cloud_with_normals->at(i).normal_y, cloud_with_normals->at(i).normal_z);
		normal.normalize();
		cloud_with_normals->at(i).normal_x = normal[0];
		cloud_with_normals->at(i).normal_y = normal[1];
		cloud_with_normals->at(i).normal_z = normal[2];
	}

	return cloud_with_normals;
}

LaserScan adjustNormalsToViewPoint(
		const LaserScan & scan,
		const Eigen::Vector3f & viewpoint,
		bool forceGroundNormalsUp)
{
	return adjustNormalsToViewPoint(scan, viewpoint, forceGroundNormalsUp?0.8f:0.0f);
}
LaserScan adjustNormalsToViewPoint(
		const LaserScan & scan,
		const Eigen::Vector3f & viewpoint,
		float groundNormalsUp)
{
	if(scan.size() && !scan.is2d() && scan.hasNormals())
	{
		int nx = scan.getNormalsOffset();
		int ny = nx+1;
		int nz = ny+1;
		cv::Mat output = scan.data().clone();
		#pragma omp parallel for
		for(int i=0; i<scan.size(); ++i)
		{
			float * ptr = output.ptr<float>(0, i);
			if(uIsFinite(ptr[nx]) && uIsFinite(ptr[ny]) && uIsFinite(ptr[nz]))
			{
				Eigen::Vector3f v = viewpoint - Eigen::Vector3f(ptr[0], ptr[1], ptr[2]);
				Eigen::Vector3f n(ptr[nx], ptr[ny], ptr[nz]);

				float result = v.dot(n);
				if(result < 0
				 || (groundNormalsUp>0.0f && ptr[nz] < -groundNormalsUp && ptr[2] < viewpoint[3])) // some far velodyne rays on road can have normals toward ground
				{
					//reverse normal
					ptr[nx] *= -1.0f;
					ptr[ny] *= -1.0f;
					ptr[nz] *= -1.0f;
				}
			}
		}
		if(scan.angleIncrement() > 0.0f)
		{
			return LaserScan(output, scan.format(), scan.rangeMin(), scan.rangeMax(), scan.angleMin(), scan.angleMax(), scan.angleIncrement(), scan.localTransform());
		}
		else
		{
			return LaserScan(output, scan.maxPoints(), scan.rangeMax(), scan.format(), scan.localTransform());
		}
	}
	return scan;
}

template<typename PointNormalT>
void adjustNormalsToViewPointImpl(
		typename pcl::PointCloud<PointNormalT>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint,
		float groundNormalsUp)
{
	#pragma omp parallel for
	for(int i=0; i<(int)cloud->size(); ++i)
	{
		pcl::PointXYZ normal(cloud->points[i].normal_x, cloud->points[i].normal_y, cloud->points[i].normal_z);
		if(pcl::isFinite(normal))
		{
			Eigen::Vector3f v = viewpoint - cloud->points[i].getVector3fMap();
			Eigen::Vector3f n(normal.x, normal.y, normal.z);

			float result = v.dot(n);
			if(result < 0
				|| (groundNormalsUp>0.0f && normal.z < -groundNormalsUp && cloud->points[i].z < viewpoint[3])) // some far velodyne rays on road can have normals toward ground
			{
				//reverse normal
				cloud->points[i].normal_x *= -1.0f;
				cloud->points[i].normal_y *= -1.0f;
				cloud->points[i].normal_z *= -1.0f;
			}
		}
	}
}

void adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint,
		bool forceGroundNormalsUp)
{
	adjustNormalsToViewPoint(cloud, viewpoint, forceGroundNormalsUp?0.8f:0.0f);
}
void adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint,
		float groundNormalsUp)
{
	adjustNormalsToViewPointImpl<pcl::PointNormal>(cloud, viewpoint, groundNormalsUp);
}

void adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint,
		bool forceGroundNormalsUp)
{
	adjustNormalsToViewPoint(cloud, viewpoint, forceGroundNormalsUp?0.8f:0.0f);
}
void adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint,
		float groundNormalsUp)
{
	adjustNormalsToViewPointImpl<pcl::PointXYZRGBNormal>(cloud, viewpoint, groundNormalsUp);
}

void adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint,
		bool forceGroundNormalsUp)
{
	adjustNormalsToViewPoint(cloud, viewpoint, forceGroundNormalsUp?0.8f:0.0f);
}
void adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint,
		float groundNormalsUp)
{
	adjustNormalsToViewPointImpl<pcl::PointXYZINormal>(cloud, viewpoint, groundNormalsUp);
}


template<typename PointT>
void adjustNormalsToViewPointsImpl(
		const std::map<int, Transform> & poses,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & rawCloud,
		const std::vector<int> & rawCameraIndices,
		typename pcl::PointCloud<PointT>::Ptr & cloud,
		float groundNormalsUp)
{
	if(poses.size() && rawCloud->size() && rawCloud->size() == rawCameraIndices.size() && cloud->size())
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr rawTree (new pcl::search::KdTree<pcl::PointXYZ>);
		rawTree->setInputCloud (rawCloud);

		#pragma omp parallel for
		for(int i=0; i<(int)cloud->size(); ++i)
		{
			pcl::PointXYZ normal(cloud->points[i].normal_x, cloud->points[i].normal_y, cloud->points[i].normal_z);
			if(pcl::isFinite(normal))
			{
				std::vector<int> indices;
				std::vector<float> dist;
				rawTree->nearestKSearch(pcl::PointXYZ(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z), 1, indices, dist);
				UASSERT(indices.size() == 1);
				if(indices.size() && indices[0]>=0)
				{
					const Transform & p = poses.at(rawCameraIndices[indices[0]]);
					pcl::PointXYZ viewpoint(p.x(), p.y(), p.z());
					Eigen::Vector3f v = viewpoint.getVector3fMap() - cloud->points[i].getVector3fMap();

					Eigen::Vector3f n(normal.x, normal.y, normal.z);

					float result = v.dot(n);
					if(result < 0 ||
					   (groundNormalsUp>0.0f && normal.z < -groundNormalsUp && cloud->points[i].z < viewpoint.z)) // some far velodyne rays on road can have normals toward ground)
					{
						//reverse normal
						cloud->points[i].normal_x *= -1.0f;
						cloud->points[i].normal_y *= -1.0f;
						cloud->points[i].normal_z *= -1.0f;
					}
				}
				else
				{
					UWARN("Not found camera viewpoint for point %d", i);
				}
			}
		}
	}
}

void adjustNormalsToViewPoints(
		const std::map<int, Transform> & poses,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & rawCloud,
		const std::vector<int> & rawCameraIndices,
		pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		float groundNormalsUp)
{
	adjustNormalsToViewPointsImpl<pcl::PointNormal>(poses, rawCloud, rawCameraIndices, cloud, groundNormalsUp);
}

void adjustNormalsToViewPoints(
		const std::map<int, Transform> & poses,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & rawCloud,
		const std::vector<int> & rawCameraIndices,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		float groundNormalsUp)
{
	adjustNormalsToViewPointsImpl<pcl::PointXYZRGBNormal>(poses, rawCloud, rawCameraIndices, cloud, groundNormalsUp);
}

void adjustNormalsToViewPoints(
		const std::map<int, Transform> & poses,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & rawCloud,
		const std::vector<int> & rawCameraIndices,
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		float groundNormalsUp)
{
	adjustNormalsToViewPointsImpl<pcl::PointXYZINormal>(poses, rawCloud, rawCameraIndices, cloud, groundNormalsUp);
}

void adjustNormalsToViewPoints(
		const std::map<int, Transform> & viewpoints,
		const LaserScan & rawScan,
		const std::vector<int> & viewpointIds,
		LaserScan & scan,
		float groundNormalsUp)
{
	UDEBUG("poses=%d, rawCloud=%d, rawCameraIndices=%d, cloud=%d", (int)viewpoints.size(), (int)rawScan.size(), (int)viewpointIds.size(), (int)scan.size());
	if(viewpoints.size() && rawScan.size() && rawScan.size() == (int)viewpointIds.size() && scan.size() && scan.hasNormals())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud = util3d::laserScanToPointCloud(rawScan);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr rawTree (new pcl::search::KdTree<pcl::PointXYZ>);
		rawTree->setInputCloud (rawCloud);
		#pragma omp parallel for
		for(int i=0; i<scan.size(); ++i)
		{
			pcl::PointNormal point = util3d::laserScanToPointNormal(scan, i);
			pcl::PointXYZ normal(point.normal_x, point.normal_y, point.normal_z);
			if(pcl::isFinite(normal))
			{
				std::vector<int> indices;
				std::vector<float> dist;
				rawTree->nearestKSearch(pcl::PointXYZ(point.x, point.y, point.z), 1, indices, dist);
				if(indices.size() && indices[0]>=0)
				{
					UASSERT_MSG(indices[0]<(int)viewpointIds.size(), uFormat("indices[0]=%d rawCameraIndices.size()=%d", indices[0], (int)viewpointIds.size()).c_str());
					UASSERT(uContains(viewpoints, viewpointIds[indices[0]]));
					Transform p = viewpoints.at(viewpointIds[indices[0]]);
					pcl::PointXYZ viewpoint(p.x(), p.y(), p.z());
					Eigen::Vector3f v = viewpoint.getVector3fMap() - point.getVector3fMap();

					Eigen::Vector3f n(normal.x, normal.y, normal.z);

					float result = v.dot(n);
					if(result < 0 ||
					   (groundNormalsUp>0.0f && normal.z < -groundNormalsUp && point.z < viewpoint.z)) // some far velodyne rays on road can have normals toward ground))
					{
						//reverse normal
						scan.field(i, scan.getNormalsOffset()) *= -1.0f;
						scan.field(i, scan.getNormalsOffset()+1) *= -1.0f;
						scan.field(i, scan.getNormalsOffset()+2) *= -1.0f;
					}
				}
				else
				{
					UWARN("Not found camera viewpoint for point %d!?", i);
				}
			}
		}
	}
}

pcl::PolygonMesh::Ptr meshDecimation(const pcl::PolygonMesh::Ptr & mesh, float factor)
{
	pcl::PolygonMesh::Ptr output(new pcl::PolygonMesh);
#ifndef DISABLE_VTK
	pcl::MeshQuadricDecimationVTK mqd;
	mqd.setTargetReductionFactor(factor);
	mqd.setInputMesh(mesh);
	mqd.process (*output);
#else
	UWARN("RTAB-Map is not built with VTK module so mesh decimation cannot be used!");
	*output = *mesh;
#endif
	return output;
}

bool intersectRayTriangle(
		const Eigen::Vector3f & p,
		const Eigen::Vector3f & dir,
		const Eigen::Vector3f & v0,
		const Eigen::Vector3f & v1,
		const Eigen::Vector3f & v2,
		float & distance,
		Eigen::Vector3f & normal)
{
    // get triangle edge cv::Vec3fs and plane normal
    const Eigen::Vector3f u = v1-v0;
    const Eigen::Vector3f v = v2-v0;
    normal = u.cross(v);              // cross product
    if (normal == Eigen::Vector3f(0,0,0))   // triangle is degenerate
        return false;                 // do not deal with this case

    const float denomimator = normal.dot(dir);
    if (fabs(denomimator) < 10e-9)    // ray is  parallel to triangle plane
       return false;

    // get intersect of ray with triangle plane
    distance = normal.dot(v0 - p) / denomimator;
    if (distance < 0.0)                      // ray goes away from triangle
        return false;

    // is I inside T?
    float    uu, uv, vv, wu, wv, D;
    uu = u.dot(u);
    uv = u.dot(v);
    vv = v.dot(v);
    const Eigen::Vector3f w = p + dir * distance - v0;
    wu = w.dot(u);
    wv = w.dot(v);
    D = uv * uv - uu * vv;

    // get and test parametric coords
    float s, t;
    s = (uv * wv - vv * wu) / D;
    if (s < 0.0 || s > 1.0)         // I is outside T
        return false;
    t = (uv * wu - uu * wv) / D;
    if (t < 0.0 || (s + t) > 1.0)   // I is outside T
        return false;

    return true;                    // I is in T
}

}

}
