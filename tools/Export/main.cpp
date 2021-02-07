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
#include <rtabmap/core/optimizer/OptimizerG2O.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
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
			"    --mesh                Create a mesh.\n"
			"    --texture             Create a mesh with texture.\n"
			"    --texture_size  #     Texture size (default 4096).\n"
			"    --texture_count #     Maximum textures generated (default 1).\n"
			"    --texture_range #     Maximum camera range for texturing a polygon (default 0 meters: no limit).\n"
			"    --texture_d2c         Distance to camera policy.\n"
			"    --ba                  Do global bundle adjustment before assembling the clouds.\n"
			"    --gain          #     Gain compensation value (default 1, set 0 to disable).\n"
			"    --gain_gray           Do gain estimation compensation on gray channel only (default RGB channels).\n"
			"    --no_blending         Disable blending when texturing.\n"
			"    --no_clean            Disable cleaning colorless polygons.\n"
			"    --low_gain      #     Low brightness gain 0-100 (default 0).\n"
			"    --high_gain     #     High brightness gain 0-100 (default 10).\n"
			"    --multiband           Enable multiband texturing (AliceVision dependency required).\n"
			"    --poisson_depth #     Set Poisson depth for mesh reconstruction.\n"
			"    --max_polygons  #     Maximum polygons when creating a mesh (default 500000, set 0 for no limit).\n"
			"    --max_range     #     Maximum range of the created clouds (default 4 m, 0 m with --scan).\n"
			"    --decimation    #     Depth image decimation before creating the clouds (default 4, 1 with --scan).\n"
			"    --voxel         #     Voxel size of the created clouds (default 0.01 m, 0 m with --scan).\n"
			"    --color_radius  #     Radius used to colorize polygons (default 0.05 m, 0 m with --scan). Set 0 for nearest color.\n"
			"    --scan                Use laser scan for the point cloud.\n"
			"    --save_in_db          Save resulting assembled point cloud or mesh in the database.\n"
			"\n%s", Parameters::showUsage());
	;
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
	bool ba = false;
	bool doGainCompensationRGB = true;
	float gainValue = 1;
	bool doBlending = true;
	bool doClean = true;
	int poissonDepth = 0;
	int maxPolygons = 500000;
	int decimation = -1;
	float maxRange = -1.0f;
	float voxelSize = -1.0f;
	int textureSize = 4096;
	int textureCount = 1;
	int textureRange = 0;
	bool distanceToCamPolicy = false;
	bool multiband = false;
	float colorRadius = -1.0f;
	bool cloudFromScan = false;
	bool saveInDb = false;
	int lowBrightnessGain = 0;
	int highBrightnessGain = 10;
	for(int i=1; i<argc; ++i)
	{
		if(std::strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}
		else if(std::strcmp(argv[i], "--mesh") == 0)
		{
			mesh = true;
		}
		else if(std::strcmp(argv[i], "--texture") == 0)
		{
			texture = true;
		}
		else if(std::strcmp(argv[i], "--texture_size") == 0)
		{
			++i;
			if(i<argc-1)
			{
				textureSize = uStr2Int(argv[i]);
				UASSERT(textureSize%256==0);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--texture_count") == 0)
		{
			++i;
			if(i<argc-1)
			{
				textureCount = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--texture_range") == 0)
		{
			++i;
			if(i<argc-1)
			{
				textureRange = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--texture_d2c") == 0)
		{
			distanceToCamPolicy = true;
		}
		else if(std::strcmp(argv[i], "--ba") == 0)
		{
			ba = true;
		}
		else if(std::strcmp(argv[i], "--gain_gray") == 0)
		{
			doGainCompensationRGB = false;
		}
		else if(std::strcmp(argv[i], "--gain") == 0)
		{
			++i;
			if(i<argc-1)
			{
				gainValue = uStr2Float(argv[i]);
				UASSERT(gainValue>0.0f);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--no_blending") == 0)
		{
			doBlending = false;
		}
		else if(std::strcmp(argv[i], "--no_clean") == 0)
		{
			doClean = false;
		}
		else if(std::strcmp(argv[i], "--multiband") == 0)
		{
#ifdef RTABMAP_ALICE_VISION
			multiband = true;
#else
			printf("\"--multiband\" option cannot be used because RTAB-Map is not built with AliceVision support. Ignoring multiband...\n");
#endif
		}
		else if(std::strcmp(argv[i], "--poisson_depth") == 0)
		{
			++i;
			if(i<argc-1)
			{
				poissonDepth = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--max_polygons") == 0)
		{
			++i;
			if(i<argc-1)
			{
				maxPolygons = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--max_range") == 0)
		{
			++i;
			if(i<argc-1)
			{
				maxRange = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--decimation") == 0)
		{
			++i;
			if(i<argc-1)
			{
				decimation = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--voxel") == 0)
		{
			++i;
			if(i<argc-1)
			{
				voxelSize = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--color_radius") == 0)
		{
			++i;
			if(i<argc-1)
			{
				colorRadius = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--scan") == 0)
		{
			cloudFromScan = true;
		}
		else if(std::strcmp(argv[i], "--save_in_db") == 0)
		{
			saveInDb = true;
		}
		else if(std::strcmp(argv[i], "--low_gain") == 0)
		{
			++i;
			if(i<argc-1)
			{
				lowBrightnessGain = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--high_gain") == 0)
		{
			++i;
			if(i<argc-1)
			{
				highBrightnessGain = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
	}

	if(decimation < 1)
	{
		decimation = cloudFromScan?1:4;
	}
	if(maxRange < 0)
	{
		maxRange = cloudFromScan?0:4;
	}
	if(voxelSize < 0.0f)
	{
		voxelSize = cloudFromScan?0:0.01f;
	}
	if(colorRadius < 0.0f)
	{
		colorRadius = cloudFromScan?0:0.05f;
	}

	if(saveInDb)
	{
		if(multiband)
		{
			printf("Option --multiband is not supported with --save_in_db option, disabling multiband...\n");
			multiband = false;
		}
		if(textureCount>1)
		{
			printf("Option --texture_count > 1 is not supported with --save_in_db option, setting texture_count to 1...\n");
			textureCount = 1;
		}
	}

	ParametersMap params = Parameters::parseArguments(argc, argv, false);

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
		return -1;
	}
	delete driver;
	driver = 0;

	for(ParametersMap::iterator iter=params.begin(); iter!=params.end(); ++iter)
	{
		printf("Added custom parameter %s=%s\n",iter->first.c_str(), iter->second.c_str());
	}

	UTimer timer;

	printf("Loading database \"%s\"...\n", dbPath.c_str());
	// Get the global optimized map
	Rtabmap rtabmap;
	uInsert(parameters, params);
	rtabmap.init(parameters, dbPath);
	printf("Loading database \"%s\"... done (%fs).\n", dbPath.c_str(), timer.ticks());

	std::map<int, Signature> nodes;
	std::map<int, Transform> optimizedPoses;
	std::multimap<int, Link> links;
	printf("Optimizing the map...\n");
	rtabmap.getGraph(optimizedPoses, links, true, true, &nodes, true, true, true, true);
	printf("Optimizing the map... done (%fs, poses=%d).\n", timer.ticks(), (int)optimizedPoses.size());

	if(optimizedPoses.empty())
	{
		printf("The optimized graph is empty!? Aborting...\n");
		return -1;
	}

	std::string outputDirectory = UDirectory::getDir(dbPath);
	std::string baseName = uSplit(UFile::getName(dbPath), '.').front();

	if(ba)
	{
		printf("Global bundle adjustment...\n");
		OptimizerG2O g2o(parameters);
		std::map<int, cv::Point3f> points3DMap;
		std::map<int, std::map<int, FeatureBA> > wordReferences;
		g2o.computeBACorrespondences(optimizedPoses, links, nodes, points3DMap, wordReferences, true);
		std::map<int, rtabmap::CameraModel> cameraSingleModels;
		for(std::map<int, Transform>::iterator iter=optimizedPoses.lower_bound(1); iter!=optimizedPoses.end(); ++iter)
		{
			Signature node = nodes.find(iter->first)->second;
			UASSERT(node.sensorData().cameraModels().size()==1);
			cameraSingleModels.insert(std::make_pair(iter->first, node.sensorData().cameraModels().front()));
		}
		optimizedPoses = g2o.optimizeBA(optimizedPoses.begin()->first, optimizedPoses, links, cameraSingleModels, points3DMap, wordReferences);
		printf("Global bundle adjustment... done (%fs).\n", timer.ticks());
	}

	// Construct the cloud
	printf("Create and assemble the clouds...\n");
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::map<int, rtabmap::Transform> cameraPoses;
	std::map<int, std::vector<rtabmap::CameraModel> > cameraModels;
	std::map<int, cv::Mat> cameraDepths;

	for(std::map<int, Transform>::iterator iter=optimizedPoses.lower_bound(1); iter!=optimizedPoses.end(); ++iter)
	{
		Signature node = nodes.find(iter->first)->second;

		// uncompress data
		std::vector<CameraModel> models = node.sensorData().cameraModels();
		cv::Mat depth;

		pcl::IndicesPtr indices(new std::vector<int>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		if(cloudFromScan)
		{
			cv::Mat tmpDepth;
			LaserScan scan;
			node.sensorData().uncompressData(0, texture&&!node.sensorData().depthOrRightCompressed().empty()?&tmpDepth:0, &scan);
			if(decimation>1 || maxRange)
			{
				scan = util3d::commonFiltering(scan, decimation, 0, maxRange);
			}
			cloud = util3d::laserScanToPointCloudRGB(scan, scan.localTransform());
		}
		else
		{
			cv::Mat tmpRGB;
			node.sensorData().uncompressData(&tmpRGB, &depth);
			cloud = util3d::cloudRGBFromSensorData(
					node.sensorData(),
					decimation,      // image decimation before creating the clouds
					maxRange,        // maximum depth of the cloud
					0.0f,
					indices.get());
		}
		if(voxelSize>0.0f)
		{
			cloud = rtabmap::util3d::voxelize(cloud, indices, voxelSize);
		}
		cloud = rtabmap::util3d::transformPointCloud(cloud, iter->second);

		Eigen::Vector3f viewpoint(iter->second.x(), iter->second.y(), iter->second.z());
		if(cloudFromScan)
		{
			Transform lidarViewpoint = iter->second * node.sensorData().laserScanRaw().localTransform();
			viewpoint = Eigen::Vector3f(iter->second.x(),  iter->second.y(),  iter->second.z());
		}
		pcl::PointCloud<pcl::Normal>::Ptr normals = rtabmap::util3d::computeNormals(cloud, 20, 0.0f, viewpoint);

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);

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
	printf("Create and assemble the clouds... done (%fs, %d points).\n", timer.ticks(), (int)mergedClouds->size());

	if(mergedClouds->size())
	{
		if(saveInDb)
		{
			driver = DBDriver::create();
			UASSERT(driver->openConnection(dbPath, false));
			Transform lastlocalizationPose;
			driver->loadOptimizedPoses(&lastlocalizationPose);
			//optimized poses have changed, reset 2d map
			driver->save2DMap(cv::Mat(), 0, 0, 0);
			driver->saveOptimizedPoses(optimizedPoses, lastlocalizationPose);
		}

		if(!(mesh || texture))
		{
			if(voxelSize>0.0f)
			{
				printf("Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n", voxelSize, (int)mergedClouds->size());
				mergedClouds = util3d::voxelize(mergedClouds, voxelSize);
			}

			if(saveInDb)
			{
				printf("Saving in db... (%d points)\n", (int)mergedClouds->size());
				driver->saveOptimizedMesh(util3d::laserScanFromPointCloud(*mergedClouds, Transform(), false).data());
				printf("Saving in db... done!\n");
			}
			else
			{
				std::string outputPath=outputDirectory+"/"+baseName+"_cloud.ply";
				printf("Saving %s... (%d points)\n", outputPath.c_str(), (int)mergedClouds->size());
				pcl::io::savePLYFile(outputPath, *mergedClouds);
				printf("Saving %s... done!\n", outputPath.c_str());
			}
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
			if(poissonDepth>0)
			{
				optimizedDepth = poissonDepth;
			}

			// Mesh reconstruction
			printf("Mesh reconstruction... depth=%d\n", optimizedDepth);
			pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
			pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
			poisson.setDepth(optimizedDepth);
			poisson.setInputCloud(mergedClouds);
			poisson.reconstruct(*mesh);
			printf("Mesh reconstruction... done (%fs, %d polygons).\n", timer.ticks(), (int)mesh->polygons.size());

			if(mesh->polygons.size())
			{
				rtabmap::util3d::denseMeshPostProcessing<pcl::PointXYZRGBNormal>(
						mesh,
						0.0f,
						maxPolygons,
						mergedClouds,
						colorRadius,
						!texture,
						doClean,
						200);

				if(!texture)
				{
					if(saveInDb)
					{
						printf("Saving mesh in db...\n");
						std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > polygons;
						polygons.push_back(util3d::convertPolygonsFromPCL(mesh->polygons));
						driver->saveOptimizedMesh(
								util3d::laserScanFromPointCloud(mesh->cloud, false).data(),
								polygons);
						printf("Saving mesh in db... done!\n");
					}
					else
					{
						std::string outputPath=outputDirectory+"/"+baseName+"_mesh.ply";
						printf("Saving %s...\n", outputPath.c_str());
						pcl::io::savePLYFile(outputPath, *mesh);
						printf("Saving %s... done!\n", outputPath.c_str());
					}
				}
				else
				{
					printf("Texturing %d polygons... cameraPoses=%d, cameraDepths=%d\n", (int)mesh->polygons.size(), (int)cameraPoses.size(), (int)cameraDepths.size());
					std::vector<std::map<int, pcl::PointXY> > vertexToPixels;
					pcl::TextureMeshPtr textureMesh = rtabmap::util3d::createTextureMesh(
							mesh,
							cameraPoses,
							cameraModels,
							cameraDepths,
							textureRange,
							0.0f,
							0.0f,
							multiband?0:50, // Min polygons in camera view to be textured by this camera
							std::vector<float>(),
							0,
							&vertexToPixels,
							distanceToCamPolicy);
					printf("Texturing... done (%fs).\n", timer.ticks());

					// Remove occluded polygons (polygons with no texture)
					if(doClean && textureMesh->tex_coordinates.size())
					{
						printf("Cleanup mesh...\n");
						rtabmap::util3d::cleanTextureMesh(*textureMesh, 100); // Min polygons in a cluster to keep them
						printf("Cleanup mesh... done (%fs).\n", timer.ticks());
					}

					if(textureMesh->tex_materials.size())
					{
						if(multiband)
						{
							printf("Merging %d texture(s) to single one (multiband enabled)...\n", (int)textureMesh->tex_materials.size());
						}
						else
						{
							printf("Merging %d texture(s)... (%d max textures)\n", (int)textureMesh->tex_materials.size(), textureCount);
						}
						std::map<int, std::map<int, cv::Vec4d> > gains;
						std::map<int, std::map<int, cv::Mat> > blendingGains;
						std::pair<float, float> contrastValues(0,0);
						cv::Mat textures = rtabmap::util3d::mergeTextures(
								*textureMesh,
								std::map<int, cv::Mat>(),
								std::map<int, std::vector<rtabmap::CameraModel> >(),
								rtabmap.getMemory(),
								0,
								textureSize,
								multiband?1:textureCount, // to get contrast values based on all images in multiband mode
								vertexToPixels,
								gainValue>0.0f, gainValue, doGainCompensationRGB,
								doBlending, 0,
								lowBrightnessGain, highBrightnessGain, // low-high brightness/contrast balance
								false, // exposure fusion
								0,     // state
								0,     // blank value (0=black)
								&gains,
								&blendingGains,
								&contrastValues);
						printf("Merging to %d texture(s)... done (%fs).\n", (int)textureMesh->tex_materials.size(), timer.ticks());

						if(saveInDb)
						{
							printf("Saving texture mesh in db...\n");
							driver->saveOptimizedMesh(
									util3d::laserScanFromPointCloud(textureMesh->cloud, false).data(),
									util3d::convertPolygonsFromPCL(textureMesh->tex_polygons),
									textureMesh->tex_coordinates,
									textures);
							printf("Saving texture mesh in db... done!\n");
						}
						else
						{
							// TextureMesh OBJ
							bool success = false;
							UASSERT(!textures.empty());
							for(size_t i=0; i<textureMesh->tex_materials.size(); ++i)
							{
								textureMesh->tex_materials[i].tex_file += ".jpg";
								printf("Saving texture to %s.\n", textureMesh->tex_materials[i].tex_file.c_str());
								UASSERT(textures.cols % textures.rows == 0);
								success = cv::imwrite(outputDirectory+"/"+textureMesh->tex_materials[i].tex_file, cv::Mat(textures, cv::Range::all(), cv::Range(textures.rows*i, textures.rows*(i+1))));
								if(!success)
								{
									UERROR("Failed saving %s!", textureMesh->tex_materials[i].tex_file.c_str());
								}
								else
								{
									printf("Saved %s.\n", textureMesh->tex_materials[i].tex_file.c_str());
								}
							}
							if(success)
							{

								std::string outputPath=outputDirectory+"/"+baseName+"_mesh.obj";
								printf("Saving obj (%d vertices) to %s.\n", (int)textureMesh->cloud.data.size()/textureMesh->cloud.point_step, outputPath.c_str());
								success = pcl::io::saveOBJFile(outputPath, *textureMesh) == 0;

								if(success)
								{
									printf("Saved obj to %s!\n", outputPath.c_str());
								}
								else
								{
									UERROR("Failed saving obj to %s!", outputPath.c_str());
								}
							}
						}

						if(multiband)
						{
							timer.restart();
							std::string outputPath=outputDirectory+"/"+baseName+"_mesh_multiband.obj";
							printf("MultiBand texturing... \"%s\"\n", outputPath.c_str());
							if(util3d::multiBandTexturing(outputPath,
									textureMesh->cloud,
									textureMesh->tex_polygons[0],
									cameraPoses,
									vertexToPixels,
									std::map<int, cv::Mat >(),
									std::map<int, std::vector<CameraModel> >(),
									rtabmap.getMemory(),
									0,
									textureSize,
									"jpg",
									gains,
									blendingGains,
									contrastValues,
									doGainCompensationRGB))
							{
								printf("MultiBand texturing...done (%fs).\n", timer.ticks());
							}
							else
							{
								printf("MultiBand texturing...failed! (%fs)\n", timer.ticks());
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

	if(driver)
	{
		driver->closeConnection();
		delete driver;
		driver = 0;
	}

	return 0;
}
