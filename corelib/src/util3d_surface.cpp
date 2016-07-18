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
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UConversion.h"
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/features/integral_image_normal.h>

#ifndef DISABLE_VTK
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
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
PCL_INSTANTIATE_PRODUCT(NormalEstimationOMP, ((pcl::PointXYZRGB))((pcl::Normal)))
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

std::map<int, int> filterNotUsedVerticesFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		pcl::PointCloud<pcl::PointXYZRGBNormal> & outputCloud,
		std::vector<pcl::Vertices> & outputPolygons)
{
	UDEBUG("size=%d polygons=%d", (int)cloud.size(), (int)polygons.size());
	std::map<int, int> addedVertices; //<oldIndex, newIndex>
	std::map<int, int> output; //<newIndex, oldIndex>
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
				output.insert(std::make_pair(oi, polygons[i].vertices[j]));
				v.vertices[j] = oi++;
			}
			else
			{
				v.vertices[j] = iter->second;
			}
		}
	}
	outputCloud.resize(oi);

	return output;
}

std::map<int, int> filterNotUsedVerticesFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		pcl::PointCloud<pcl::PointXYZRGB> & outputCloud,
		std::vector<pcl::Vertices> & outputPolygons)
{
	UDEBUG("size=%d polygons=%d", (int)cloud.size(), (int)polygons.size());
	std::map<int, int> addedVertices; //<oldIndex, newIndex>
	std::map<int, int> output; //<newIndex, oldIndex>
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
				output.insert(std::make_pair(oi, polygons[i].vertices[j]));
				v.vertices[j] = oi++;
			}
			else
			{
				v.vertices[j] = iter->second;
			}
		}
	}
	outputCloud.resize(oi);

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
		const std::map<int, CameraModel> & cameraModels,
		const std::map<int, cv::Mat> & images,
		const std::string & tmpDirectory)
{
	UASSERT(poses.size() == cameraModels.size() && poses.size() == images.size());
	UASSERT(UDirectory::exists(tmpDirectory));
	pcl::texture_mapping::CameraVector cameras(poses.size());
	std::map<int, Transform>::const_iterator poseIter=poses.begin();
	std::map<int, CameraModel>::const_iterator modelIter=cameraModels.begin();
	std::map<int, cv::Mat>::const_iterator imageIter=images.begin();
	int oi=0;
	for(; poseIter!=poses.end(); ++poseIter, ++modelIter, ++imageIter)
	{
		UASSERT(poseIter->first == modelIter->first);
		UASSERT(poseIter->first == imageIter->first);
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam;

		// transform into optical referential
		Transform rotation(0,-1,0,0,
						   0,0,-1,0,
						   1,0,0,0);

		Transform t = poseIter->second*rotation.inverse();

		cam.pose = t.toEigen3f();

		UASSERT(modelIter->second.fx()>0 && imageIter->second.rows>0 && imageIter->second.cols>0);
		cam.focal_length=modelIter->second.fx();
		cam.height=imageIter->second.rows;
		cam.width=imageIter->second.cols;


		std::string fileName = uFormat("%s/%s%d.png", tmpDirectory.c_str(), "texture_", poseIter->first);
		if(!cv::imwrite(fileName, imageIter->second))
		{
			UERROR("Cannot save texture of image %d", poseIter->first);
		}
		else
		{
			UINFO("Saved temporary texture: \"%s\"", fileName.c_str());
		}
		cam.texture_file = fileName;
		cameras[oi++] = cam;
	}
	return cameras;
}

pcl::TextureMesh::Ptr createTextureMesh(
		const pcl::PolygonMesh::Ptr & mesh,
		const std::map<int, Transform> & poses,
		const std::map<int, CameraModel> & cameraModels,
		const std::map<int, cv::Mat> & images,
		const std::string & tmpDirectory,
		int kNormalSearch)
{
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
			images,
			tmpDirectory);

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
			mesh_material.tex_file = tmpDirectory+UDirectory::separator()+"occluded.png";
			cv::Mat emptyImage;
			if(i>0)
			{
				emptyImage = cv::Mat::zeros(cameras[i-1].height,cameras[i-1].width, CV_8UC1);
			}
			else
			{
				emptyImage = cv::Mat::zeros(480, 640, CV_8UC1);
			}
			cv::imwrite(mesh_material.tex_file, emptyImage);
		}

		textureMesh->tex_materials[i] = mesh_material;
	}

	// Texture by projection
	pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
	tm.textureMeshwithMultipleCameras(*textureMesh, cameras);

	// compute normals for the mesh if not already here
	bool hasNormals = false;
	for(unsigned int i=0; i<textureMesh->cloud.fields.size(); ++i)
	{
		if(textureMesh->cloud.fields[i].name.compare("normal_x") == 0)
		{
			hasNormals = true;
			break;
		}
	}
	if(!hasNormals)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(textureMesh->cloud, *cloud);
		pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(cloud, kNormalSearch);
		// Concatenate the XYZ and normal fields
		pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals;
		pcl::concatenateFields (*cloud, *normals, *cloudWithNormals);
		pcl::toPCLPointCloud2 (*cloudWithNormals, textureMesh->cloud);
	}

	return textureMesh;
}

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int normalKSearch,
		const Eigen::Vector3f & viewPoint)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return computeNormals(cloud, indices, normalKSearch, viewPoint);
}
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int normalKSearch,
		const Eigen::Vector3f & viewPoint)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
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
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
#else
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
#endif
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	n.setInputCloud (cloud);
	// Commented: Keep the output normals size the same as the input cloud
	//if(indices->size())
	//{
	//	n.setIndices(indices);
	//}
	n.setSearchMethod (tree);
	n.setKSearch (normalKSearch);
	n.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
	n.compute (*normals);

	return normals;
}

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int normalKSearch,
		const Eigen::Vector3f & viewPoint)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	return computeNormals(cloud, indices, normalKSearch, viewPoint);
}
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int normalKSearch,
		const Eigen::Vector3f & viewPoint)
{
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
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
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> n;
#else
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
#endif
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	n.setInputCloud (cloud);
	// Commented: Keep the output normals size the same as the input cloud
	//if(indices->size())
	//{
	//	n.setIndices(indices);
	//}
	n.setSearchMethod (tree);
	n.setKSearch (normalKSearch);
	n.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
	n.compute (*normals);

	return normals;
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

	// Normal estimation
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(maxDepthChangeFactor);
	ne.setNormalSmoothingSize(normalSmoothingSize);
	ne.setInputCloud(cloud);
	// Commented: Keep the output normals size the same as the input cloud
	//if(indices->size())
	//{
	//	ne.setIndices(indices);
	//}
	ne.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
	ne.compute(*normals);

	return normals;
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
		mls.setPolynomialFit (true);
		mls.setPolynomialOrder(polygonialOrder);
	}
	else
	{
		mls.setPolynomialFit (false);
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

void adjustNormalsToViewPoints(
		const std::map<int, Transform> & poses,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & rawCloud,
		const std::vector<int> & rawCameraIndices,
		pcl::PointCloud<pcl::PointNormal>::Ptr & cloud)
{
	if(poses.size() && rawCloud->size() && rawCloud->size() == rawCameraIndices.size() && cloud->size())
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr rawTree (new pcl::search::KdTree<pcl::PointXYZ>);
		rawTree->setInputCloud (rawCloud);

		for(unsigned int i=0; i<cloud->size(); ++i)
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
					Transform p = poses.at(rawCameraIndices[indices[0]]);
					pcl::PointXYZ viewpoint(p.x(), p.y(), p.z());
					Eigen::Vector3f v = viewpoint.getVector3fMap() - cloud->points[i].getVector3fMap();

					Eigen::Vector3f n(normal.x, normal.y, normal.z);

					float result = v.dot(n);
					if(result < 0)
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
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud)
{
	if(poses.size() && rawCloud->size() && rawCloud->size() == rawCameraIndices.size() && cloud->size())
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr rawTree (new pcl::search::KdTree<pcl::PointXYZ>);
		rawTree->setInputCloud (rawCloud);

		for(unsigned int i=0; i<cloud->size(); ++i)
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
					Transform p = poses.at(rawCameraIndices[indices[0]]);
					pcl::PointXYZ viewpoint(p.x(), p.y(), p.z());
					Eigen::Vector3f v = viewpoint.getVector3fMap() - cloud->points[i].getVector3fMap();

					Eigen::Vector3f n(normal.x, normal.y, normal.z);

					float result = v.dot(n);
					if(result < 0)
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

}

}
