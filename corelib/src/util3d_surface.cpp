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

#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UConversion.h"
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

namespace rtabmap
{

namespace util3d
{

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
		const std::string & tmpDirectory)
{
	// Original from pcl/gpu/kinfu_large_scale/tools/standalone_texture_mapping.cpp:
	// Author: Raphael Favier, Technical University Eindhoven, (r.mysurname <aT> tue.nl)

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

	// Create the texturemesh object that will contain our UV-mapped mesh
	pcl::TextureMesh::Ptr textureMesh(new pcl::TextureMesh);
	textureMesh->cloud = mesh->cloud;
	std::vector< pcl::Vertices> polygons;

	// push faces into the texturemesh object
	polygons.resize (mesh->polygons.size ());
	for(size_t i =0; i < mesh->polygons.size (); ++i)
	{
		polygons[i] = mesh->polygons[i];
	}
	textureMesh->tex_polygons.push_back(polygons);

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

	// Sort faces
	pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
	tm.textureMeshwithMultipleCameras(*textureMesh, cameras);

	// compute normals for the mesh
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals = computeNormals(cloud, 20);

	pcl::toPCLPointCloud2 (*cloudWithNormals, textureMesh->cloud);

	return textureMesh;
}

pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int normalKSearch)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	// Normal estimation*
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (normalKSearch);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals*/

	return cloud_with_normals;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int normalKSearch)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud);

	// Normal estimation*
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (normalKSearch);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals*/

	return cloud_with_normals;
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
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud);

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
	mls.setSearchMethod (tree);
	mls.process (*cloud_with_normals);

	return cloud_with_normals;
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
			std::vector<int> indices;
			std::vector<float> dist;
			rawTree->nearestKSearch(pcl::PointXYZ(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z), 1, indices, dist);
			UASSERT(indices.size() == 1);
			if(indices.size() && indices[0]>=0)
			{
				Transform p = poses.at(rawCameraIndices[indices[0]]);
				pcl::PointXYZ viewpoint(p.x(), p.y(), p.z());
				Eigen::Vector3f v = viewpoint.getVector3fMap() - cloud->points[i].getVector3fMap();

				Eigen::Vector3f n(cloud->points[i].normal_x, cloud->points[i].normal_y, cloud->points[i].normal_z);

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

pcl::PolygonMesh::Ptr meshDecimation(const pcl::PolygonMesh::Ptr & mesh, float factor)
{
	pcl::MeshQuadricDecimationVTK mqd;
	mqd.setTargetReductionFactor(factor);
	mqd.setInputMesh(mesh);
	pcl::PolygonMesh::Ptr output(new pcl::PolygonMesh);
	mqd.process (*output);
	return output;
}

}

}
