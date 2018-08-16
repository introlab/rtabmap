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

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/common.h>
#include <pcl/surface/poisson.h>
#include <stdio.h>

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-exportCloud [options] database.db\n"
			"Options:\n"
			"    --mesh          Create a mesh.\n"
			"    --texture       Create a mesh with texture.\n"
			"\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kError);

	if(argc < 2)
	{
		showUsage();
	}

	bool mesh = false;
	bool texture = false;
	for(int i=1; i<argc-1; ++i)
	{
		if(std::strcmp(argv[i], "--mesh") == 0)
		{
			mesh = true;
		}
		else if(std::strcmp(argv[i], "--texture") == 0)
		{
			texture = true;
		}
	}

	std::string dbPath = argv[argc-1];

	// Get parameters
	ParametersMap parameters;
	DBDriver * driver = DBDriver::create();
	if(driver->openConnection(dbPath))
	{
		parameters = driver->getLastParameters();
		driver->closeConnection(false);
	}
	else
	{
		UERROR("Cannot open database %s!", dbPath.c_str());
	}
	delete driver;

	// Get the global optimized map
	Rtabmap rtabmap;
	rtabmap.init(parameters, dbPath);

	std::map<int, Signature> nodes;
	std::map<int, Transform> optimizedPoses;
	std::multimap<int, Link> links;
	rtabmap.get3DMap(nodes, optimizedPoses, links, true, true);

	// Construct the cloud
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::map<int, rtabmap::Transform> cameraPoses;
	std::map<int, std::vector<rtabmap::CameraModel> > cameraModels;
	std::map<int, cv::Mat> cameraDepths;
	for(std::map<int, Transform>::iterator iter=optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
	{
		Signature node = nodes.find(iter->first)->second;

		// uncompress data
		node.sensorData().uncompressData();
		std::vector<CameraModel> models = node.sensorData().cameraModels();
		cv::Mat depth = node.sensorData().depthRaw();

		pcl::IndicesPtr indices(new std::vector<int>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
				node.sensorData(),
				4,           // image decimation before creating the clouds
				4.0f,        // maximum depth of the cloud
				0.0f,
				indices.get());

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		transformedCloud = rtabmap::util3d::voxelize(cloud, indices, 0.01);
		transformedCloud = rtabmap::util3d::transformPointCloud(transformedCloud, iter->second);

		Eigen::Vector3f viewpoint( iter->second.x(),  iter->second.y(),  iter->second.z());
		pcl::PointCloud<pcl::Normal>::Ptr normals = rtabmap::util3d::computeNormals(transformedCloud, 10, 0.0f, viewpoint);

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields(*transformedCloud, *normals, *cloudWithNormals);

		if(mergedClouds->size() == 0)
		{
			*mergedClouds = *cloudWithNormals;
		}
		else
		{
			*mergedClouds += *cloudWithNormals;
		}

		cameraPoses.insert(std::make_pair(iter->first, iter->second));
		if(!models.empty())
		{
			cameraModels.insert(std::make_pair(iter->first, models));
		}
		if(!depth.empty())
		{
			cameraDepths.insert(std::make_pair(iter->first, depth));
		}
	}
	if(mergedClouds->size())
	{
		if(!(mesh || texture))
		{
			printf("Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n", 0.01f, (int)mergedClouds->size());
			mergedClouds = util3d::voxelize(mergedClouds, 0.01f);

			printf("Saving cloud.ply... (%d points)\n", (int)mergedClouds->size());
			pcl::io::savePLYFile("cloud.ply", *mergedClouds);
			printf("Saving cloud.ply... done!\n");
		}
		else
		{
			Eigen::Vector4f min,max;
			pcl::getMinMax3D(*mergedClouds, min, max);
			float mapLength = uMax3(max[0]-min[0], max[1]-min[1], max[2]-min[2]);
			int optimizedDepth = 12;
			for(int i=6; i<12; ++i)
			{
				if(mapLength/float(1<<i) < 0.03f)
				{
					optimizedDepth = i;
					break;
				}
			}

			// Mesh reconstruction
			printf("Mesh reconstruction...\n");
			pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
			pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
			poisson.setDepth(optimizedDepth);
			poisson.setInputCloud(mergedClouds);
			UTimer timer;
			poisson.reconstruct(*mesh);
			printf("Mesh reconstruction... done! %fs (%d polygons)\n", timer.ticks(), (int)mesh->polygons.size());

			if(mesh->polygons.size())
			{
				rtabmap::util3d::denseMeshPostProcessing<pcl::PointXYZRGBNormal>(
						mesh,
						0.0f,
						0,
						mergedClouds,
						0.05,
						!texture);

				if(!texture)
				{
					printf("Saving mesh.ply...\n");
					pcl::io::savePLYFile("mesh.ply", *mesh);
					printf("Saving mesh.ply... done!\n");
				}
				else
				{
					printf("Texturing... cameraPoses=%d, cameraDepths=%d\n", (int)cameraPoses.size(), (int)cameraDepths.size());
					std::vector<std::map<int, pcl::PointXY> > vertexToPixels;
					pcl::TextureMeshPtr textureMesh = rtabmap::util3d::createTextureMesh(
							mesh,
							cameraPoses,
							cameraModels,
							cameraDepths,
							3,
							0.0f,
							0.0f,
							50,
							std::vector<float>(),
							0,
							&vertexToPixels);
					printf("Texturing... done! %fs\n", timer.ticks());

					// Remove occluded polygons (polygons with no texture)
					if(textureMesh->tex_coordinates.size())
					{
						printf("Cleanup mesh...\n");
						rtabmap::util3d::cleanTextureMesh(*textureMesh, 0);
						printf("Cleanup mesh... done! %fs\n", timer.ticks());
					}

					if(textureMesh->tex_materials.size())
					{
						printf("Merging %d textures...\n", (int)textureMesh->tex_materials.size());
						cv::Mat textures = rtabmap::util3d::mergeTextures(
								*textureMesh,
								std::map<int, cv::Mat>(),
								std::map<int, std::vector<rtabmap::CameraModel> >(),
								rtabmap.getMemory(),
								0,
								4096,
								1,
								vertexToPixels,
								true, 10.0f, true ,true, 0, 0, 0, false);


						// TextureMesh OBJ
						bool success = false;
						UASSERT(!textures.empty());
						UASSERT(textureMesh->tex_materials.size() == 1);

						std::string filePath = "mesh.jpg";
						textureMesh->tex_materials[0].tex_file = filePath;
						printf("Saving texture to %s.\n", filePath.c_str());
						success = cv::imwrite(filePath, textures);
						if(!success)
						{
							UERROR("Failed saving %s!", filePath.c_str());
						}
						else
						{
							printf("Saved %s.\n", filePath.c_str());
						}

						if(success)
						{

							std::string filePath = "mesh.obj";
							printf("Saving obj (%d vertices) to %s.\n", (int)textureMesh->cloud.data.size()/textureMesh->cloud.point_step, filePath.c_str());
							success = pcl::io::saveOBJFile(filePath, *textureMesh) == 0;

							if(success)
							{
								printf("Saved obj to %s!\n", filePath.c_str());
							}
							else
							{
								UERROR("Failed saving obj to %s!", filePath.c_str());
							}
						}
					}
				}
			}
		}
	}
	else
	{
		printf("Export failed! The cloud is empty.\n");
	}

	return 0;
}
