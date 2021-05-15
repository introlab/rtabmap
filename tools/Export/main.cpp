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
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/optimizer/OptimizerG2O.h>
#include <rtabmap/core/Graph.h>
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

#ifdef RTABMAP_PDAL
#include <rtabmap/core/PDALWriter.h>
#endif

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-export [options] database.db\n"
			"Options:\n"
			"    --output              Output name (default: name of the database is used).\n"
			"    --bin                 Export PLY in binary format.\n"
			"    --las                 Export cloud in LAS instead of PLY (PDAL dependency required).\n"
			"    --mesh                Create a mesh.\n"
			"    --texture             Create a mesh with texture.\n"
			"    --texture_size  #     Texture size (default 4096).\n"
			"    --texture_count #     Maximum textures generated (default 1).\n"
			"    --texture_range #     Maximum camera range for texturing a polygon (default 0 meters: no limit).\n"
			"    --texture_depth_error # Maximum depth error between reprojected mesh and depth image to texture a face (-1=disabled, 0=edge length is used, default=0).\n"
			"    --texture_d2c         Distance to camera policy.\n"
			"    --cam_projection      Camera projection on assembled cloud and export node ID on each point (in PointSourceId field).\n"
			"    --poses               Export optimized poses of the robot frame (e.g., base_link).\n"
			"    --poses_camera        Export optimized poses of the camera frame (e.g., optical frame).\n"
			"    --poses_scan          Export optimized poses of the scan frame.\n"
			"    --poses_format #      Format used for exported poses (default is 10):\n"
			"                              0=Raw 3x4 transformation matrix (r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz)\n"
			"                              1=RGBD-SLAM (in motion capture coordinate frame)\n"
			"                              2=KITTI (same as raw but in optical frame)\n"
			"                              3=TORO\n"
			"                              4=g2o\n"
			"                              10=RGBD-SLAM in ROS coordinate frame (stamp x y z qx qy qz qw)\n"
			"                              11=RGBD-SLAM in ROS coordinate frame + ID (stamp x y z qx qy qz qw id)\n"
			"    --images              Export images with stamp as file name.\n"
			"    --images_id           Export images with node id as file name.\n"
			"    --ba                  Do global bundle adjustment before assembling the clouds.\n"
			"    --gain          #     Gain compensation value (default 1, set 0 to disable).\n"
			"    --gain_gray           Do gain estimation compensation on gray channel only (default RGB channels).\n"
			"    --no_blending         Disable blending when texturing.\n"
			"    --no_clean            Disable cleaning colorless polygons.\n"
			"    --low_gain      #     Low brightness gain 0-100 (default 0).\n"
			"    --high_gain     #     High brightness gain 0-100 (default 10).\n"
			"    --multiband           Enable multiband texturing (AliceVision dependency required).\n"
			"    --poisson_depth #     Set Poisson depth for mesh reconstruction.\n"
			"    --poisson_size  #     Set target polygon size when computing Poisson's depth for mesh reconstruction (default 0.03 m).\n"
			"    --max_polygons  #     Maximum polygons when creating a mesh (default 500000, set 0 for no limit).\n"
			"    --max_range     #     Maximum range of the created clouds (default 4 m, 0 m with --scan).\n"
			"    --decimation    #     Depth image decimation before creating the clouds (default 4, 1 with --scan).\n"
			"    --voxel         #     Voxel size of the created clouds (default 0.01 m, 0 m with --scan).\n"
			"    --noise_radius  #     Noise filtering search radius (default 0, 0=disabled).\n"
			"    --noise_k       #     Noise filtering minimum neighbors in search radius (default 5, 0=disabled)."
			"    --color_radius  #     Radius used to colorize polygons (default 0.05 m, 0 m with --scan). Set 0 for nearest color.\n"
			"    --scan                Use laser scan for the point cloud.\n"
			"    --save_in_db          Save resulting assembled point cloud or mesh in the database.\n"
			"    --xmin #              Minimum range on X axis to keep nodes to export.\n"
			"    --xmax #              Maximum range on X axis to keep nodes to export.\n"
			"    --ymin #              Minimum range on Y axis to keep nodes to export.\n"
			"    --ymax #              Maximum range on Y axis to keep nodes to export.\n"
			"    --zmin #              Minimum range on Z axis to keep nodes to export.\n"
			"    --zmax #              Maximum range on Z axis to keep nodes to export.\n"
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

	bool binary = false;
	bool las = false;
	bool mesh = false;
	bool texture = false;
	bool ba = false;
	bool doGainCompensationRGB = true;
	float gainValue = 1;
	bool doBlending = true;
	bool doClean = true;
	int poissonDepth = 0;
	float poissonSize = 0.03;
	int maxPolygons = 500000;
	int decimation = -1;
	float maxRange = -1.0f;
	float voxelSize = -1.0f;
	float noiseRadius = 0.0f;
	int noiseMinNeighbors = 5;
	int textureSize = 4096;
	int textureCount = 1;
	int textureRange = 0;
	float textureDepthError = 0;
	bool distanceToCamPolicy = false;
	bool multiband = false;
	float colorRadius = -1.0f;
	bool cloudFromScan = false;
	bool saveInDb = false;
	int lowBrightnessGain = 0;
	int highBrightnessGain = 10;
	bool camProjection = false;
	bool exportPoses = false;
	bool exportPosesCamera = false;
	bool exportPosesScan = false;
	int exportPosesFormat = 10;
	bool exportImages = false;
	bool exportImagesId = false;
	std::string outputName;
	cv::Vec3f min, max;
	for(int i=1; i<argc; ++i)
	{
		if(std::strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}
		else if(std::strcmp(argv[i], "--output") == 0)
		{
			++i;
			if(i<argc-1)
			{
				outputName = argv[i];
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--bin") == 0)
		{
			binary = true;
		}
		else if(std::strcmp(argv[i], "--las") == 0)
		{
#ifdef RTABMAP_PDAL
			las = true;
#else
			printf("\"--las\" option cannot be used because RTAB-Map is not built with PDAL support. Will export in PLY...\n");
#endif

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
		else if(std::strcmp(argv[i], "--texture_depth_error") == 0)
		{
			++i;
			if(i<argc-1)
			{
				textureDepthError = uStr2Float(argv[i]);
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
		else if(std::strcmp(argv[i], "--cam_projection") == 0)
		{
			camProjection = true;
		}
		else if(std::strcmp(argv[i], "--poses") == 0)
		{
			exportPoses = true;
		}
		else if(std::strcmp(argv[i], "--poses_camera") == 0)
		{
			exportPosesCamera = true;
		}
		else if(std::strcmp(argv[i], "--poses_scan") == 0)
		{
			exportPosesScan = true;
		}
		else if(std::strcmp(argv[i], "--poses_format") == 0)
		{
			++i;
			if(i<argc-1)
			{
				exportPosesFormat = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--images") == 0)
		{
			exportImages = true;
		}
		else if(std::strcmp(argv[i], "--images_id") == 0)
		{
			exportImages = true;
			exportImagesId = true;
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
		else if(std::strcmp(argv[i], "--poisson_size") == 0)
		{
			++i;
			if(i<argc-1)
			{
				poissonSize = uStr2Float(argv[i]);
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
		else if(std::strcmp(argv[i], "--noise_radius") == 0)
		{
			++i;
			if(i<argc-1)
			{
				noiseRadius = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--noise_k") == 0)
		{
			++i;
			if(i<argc-1)
			{
				noiseMinNeighbors = uStr2Int(argv[i]);
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
		else if(std::strcmp(argv[i], "--xmin") == 0)
		{
			++i;
			if(i<argc-1)
			{
				min[0] = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--xmax") == 0)
		{
			++i;
			if(i<argc-1)
			{
				max[0] = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--ymin") == 0)
		{
			++i;
			if(i<argc-1)
			{
				min[1] = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--ymax") == 0)
		{
			++i;
			if(i<argc-1)
			{
				max[1] = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--zmin") == 0)
		{
			++i;
			if(i<argc-1)
			{
				min[2] = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--zmax") == 0)
		{
			++i;
			if(i<argc-1)
			{
				max[2] = uStr2Float(argv[i]);
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

	if(!UFile::exists(dbPath))
	{
		UERROR("File \"%s\" doesn't exist!", dbPath.c_str());
		return -1;
	}

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

	if(min[0] != max[0] || min[1] != max[1] || min[2] != max[2])
	{
		cv::Vec3f minP,maxP;
		graph::computeMinMax(optimizedPoses, minP, maxP);
		printf("Filtering poses (range: x=%f<->%f, y=%f<->%f, z=%f<->%f, map size=%f x %f x %f)...\n",
				min[0],max[0],min[1],max[1],min[2],max[2],
				maxP[0]-minP[0],maxP[1]-minP[1],maxP[2]-minP[2]);
		std::map<int, Transform> posesFiltered;
		for(std::map<int, Transform>::const_iterator iter=optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
		{
			bool ignore = false;
			if(min[0] != max[0] && (iter->second.x() < min[0] || iter->second.x() > max[0]))
			{
				ignore = true;
			}
			if(min[1] != max[1] && (iter->second.y() < min[1] || iter->second.y() > max[1]))
			{
				ignore = true;
			}
			if(min[2] != max[2] && (iter->second.z() < min[2] || iter->second.z() > max[2]))
			{
				ignore = true;
			}
			if(!ignore)
			{
				posesFiltered.insert(*iter);
			}
		}
		graph::computeMinMax(posesFiltered, minP, maxP);
		printf("Filtering poses... done! %d/%d remaining (new map size=%f x %f x %f).\n", (int)posesFiltered.size(), (int)optimizedPoses.size(), maxP[0]-minP[0],maxP[1]-minP[1],maxP[2]-minP[2]);
		optimizedPoses = posesFiltered;
		if(optimizedPoses.empty())
		{
			return -1;
		}
	}

	std::string outputDirectory = UDirectory::getDir(dbPath);
	std::string baseName = outputName.empty()?uSplit(UFile::getName(dbPath), '.').front():outputName;

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
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr mergedCloudsI(new pcl::PointCloud<pcl::PointXYZINormal>);
	std::map<int, rtabmap::Transform> robotPoses;
	std::vector<std::map<int, rtabmap::Transform> > cameraPoses;
	std::map<int, rtabmap::Transform> scanPoses;
	std::map<int, double> cameraStamps;
	std::map<int, std::vector<rtabmap::CameraModel> > cameraModels;
	std::map<int, cv::Mat> cameraDepths;
	int imagesExported = 0;
	bool calibSaved = false;
	for(std::map<int, Transform>::iterator iter=optimizedPoses.lower_bound(1); iter!=optimizedPoses.end(); ++iter)
	{
		Signature node = nodes.find(iter->first)->second;

		// uncompress data
		std::vector<CameraModel> models = node.sensorData().cameraModels();
		cv::Mat rgb;
		cv::Mat depth;

		pcl::IndicesPtr indices(new std::vector<int>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudI;
		if(cloudFromScan)
		{
			cv::Mat tmpDepth;
			LaserScan scan;
			node.sensorData().uncompressData(exportImages?&rgb:0, (texture||exportImages)&&!node.sensorData().depthOrRightCompressed().empty()?&tmpDepth:0, &scan);
			if(decimation>1 || maxRange)
			{
				scan = util3d::commonFiltering(scan, decimation, 0, maxRange);
			}
			if(scan.hasRGB())
			{
				cloud = util3d::laserScanToPointCloudRGB(scan, scan.localTransform());
				if(noiseRadius>0.0f && noiseMinNeighbors>0)
				{
					indices = util3d::radiusFiltering(cloud, noiseRadius, noiseMinNeighbors);
				}
			}
			else
			{
				cloudI = util3d::laserScanToPointCloudI(scan, scan.localTransform());
				if(noiseRadius>0.0f && noiseMinNeighbors>0)
				{
					indices = util3d::radiusFiltering(cloudI, noiseRadius, noiseMinNeighbors);
				}
			}
		}
		else
		{
			node.sensorData().uncompressData(&rgb, &depth);
			cloud = util3d::cloudRGBFromSensorData(
					node.sensorData(),
					decimation,      // image decimation before creating the clouds
					maxRange,        // maximum depth of the cloud
					0.0f,
					indices.get());
			if(noiseRadius>0.0f && noiseMinNeighbors>0)
			{
				indices = util3d::radiusFiltering(cloud, indices, noiseRadius, noiseMinNeighbors);
			}
		}

		if(exportImages && !rgb.empty())
		{
			std::string dirSuffix = (depth.type() != CV_16UC1 && depth.type() != CV_32FC1)?"left":"rgb";
			std::string dir = outputDirectory+"/"+baseName+"_"+dirSuffix;
			UDirectory::makeDir(dir);
			std::string outputPath=dir+"/"+(exportImagesId?uNumber2Str(iter->first):uFormat("%f",node.getStamp()))+".jpg";
			cv::imwrite(outputPath, rgb);
			++imagesExported;
			if(!depth.empty())
			{
				std::string ext;
				cv::Mat depthExported = depth;
				if(depth.type() != CV_16UC1 && depth.type() != CV_32FC1)
				{
					ext = ".jpg";
					dir = outputDirectory+"/"+baseName+"_right";
				}
				else
				{
					ext = ".png";
					dir = outputDirectory+"/"+baseName+"_depth";
					if(depth.type() == CV_32FC1)
					{
						depthExported = rtabmap::util2d::cvtDepthFromFloat(depth);
					}
				}
				if(!UDirectory::exists(dir))
				{
					UDirectory::makeDir(dir);
				}

				outputPath=dir+"/"+(exportImagesId?uNumber2Str(iter->first):uFormat("%f",node.getStamp()))+ext;
				cv::imwrite(outputPath, depthExported);
			}
			if(!calibSaved)
			{
				for(size_t i=0; i<models.size(); ++i)
				{
					CameraModel model = models[i];
					if(models.size() > 1) {
						std::string prefix = model.name();
						if(prefix.empty())
							prefix = "camera";
						model.setName(prefix + "_" + uNumber2Str((int)i));
					}
					model.save(outputDirectory);
				}
				calibSaved = !models.empty();
				if(node.sensorData().stereoCameraModel().isValidForProjection())
				{
					node.sensorData().stereoCameraModel().save(outputDirectory);
					calibSaved = true;
				}
			}
		}

		if(voxelSize>0.0f)
		{
			if(cloud.get() && !cloud->empty())
				cloud = rtabmap::util3d::voxelize(cloud, indices, voxelSize);
			else if(cloudI.get() && !cloudI->empty())
				cloudI = rtabmap::util3d::voxelize(cloudI, indices, voxelSize);
		}
		if(cloud.get() && !cloud->empty())
			cloud = rtabmap::util3d::transformPointCloud(cloud, iter->second);
		else if(cloudI.get() && !cloudI->empty())
			cloudI = rtabmap::util3d::transformPointCloud(cloudI, iter->second);


		Eigen::Vector3f viewpoint(iter->second.x(), iter->second.y(), iter->second.z());
		if(cloudFromScan)
		{
			Transform lidarViewpoint = iter->second * node.sensorData().laserScanRaw().localTransform();
			viewpoint = Eigen::Vector3f(iter->second.x(),  iter->second.y(),  iter->second.z());
		}
		if(cloud.get() && !cloud->empty())
		{
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
		}
		else if(cloudI.get() && !cloudI->empty())
		{
			pcl::PointCloud<pcl::Normal>::Ptr normals = rtabmap::util3d::computeNormals(cloudI, 20, 0.0f, viewpoint);
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudIWithNormals(new pcl::PointCloud<pcl::PointXYZINormal>);
			pcl::concatenateFields(*cloudI, *normals, *cloudIWithNormals);
			if(mergedCloudsI->size() == 0)
			{
				*mergedCloudsI = *cloudIWithNormals;
			}
			else
			{
				*mergedCloudsI += *cloudIWithNormals;
			}
		}

		if(models.empty() && node.sensorData().stereoCameraModel().isValidForProjection())
		{
			models.push_back(node.sensorData().stereoCameraModel().left());
		}

		robotPoses.insert(std::make_pair(iter->first, iter->second));
		cameraStamps.insert(std::make_pair(iter->first, node.getStamp()));
		if(!models.empty())
		{
			cameraModels.insert(std::make_pair(iter->first, models));
			if(exportPosesCamera)
			{
				if(cameraPoses.empty())
				{
					cameraPoses.resize(models.size());
				}
				UASSERT_MSG(models.size() == cameraPoses.size(), "Not all nodes have same number of cameras to export camera poses.");
				for(size_t i=0; i<models.size(); ++i)
				{
					cameraPoses[i].insert(std::make_pair(iter->first, iter->second*models[i].localTransform()));
				}
			}
		}
		if(!depth.empty())
		{
			cameraDepths.insert(std::make_pair(iter->first, depth));
		}
		if(exportPosesScan && !node.sensorData().laserScanCompressed().empty())
		{
			scanPoses.insert(std::make_pair(iter->first, iter->second*node.sensorData().laserScanCompressed().localTransform()));
		}
	}
	printf("Create and assemble the clouds... done (%fs, %d points).\n", timer.ticks(), !mergedClouds->empty()?(int)mergedClouds->size():(int)mergedCloudsI->size());

	if(imagesExported>0)
		printf("%d images exported!\n", imagesExported);

	if(!mergedClouds->empty() || !mergedCloudsI->empty())
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
		else
		{
			std::string posesExt = (exportPosesFormat==3?"toro":exportPosesFormat==4?"g2o":"txt");
			if(exportPoses)
			{
				std::string outputPath=outputDirectory+"/"+baseName+"_poses." + posesExt;
				rtabmap::graph::exportPoses(outputPath, exportPosesFormat, robotPoses, links, cameraStamps);
				printf("Poses exported to \"%s\".\n", outputPath.c_str());
			}
			if(exportPosesCamera)
			{
				for(size_t i=0; i<cameraPoses.size(); ++i)
				{
					std::string outputPath;
					if(cameraPoses.size()==1)
						outputPath = outputDirectory+"/"+baseName+"_camera_poses." + posesExt;
					else
						outputPath = outputDirectory+"/"+baseName+"_camera_poses_"+uNumber2Str((int)i)+"." + posesExt;
					rtabmap::graph::exportPoses(outputPath, exportPosesFormat, cameraPoses[i], std::multimap<int, Link>(), cameraStamps);
					printf("Camera poses exported to \"%s\".\n", outputPath.c_str());
				}
			}
			if(exportPosesScan)
			{
				std::string outputPath=outputDirectory+"/"+baseName+"_scan_poses." + posesExt;
				rtabmap::graph::exportPoses(outputPath, exportPosesFormat, scanPoses, std::multimap<int, Link>(), cameraStamps);
				printf("Scan poses exported to \"%s\".\n", outputPath.c_str());
			}
		}

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudToExport = mergedClouds;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudIToExport = mergedCloudsI;
		if(voxelSize>0.0f)
		{
			printf("Voxel grid filtering of the assembled cloud... (voxel=%f, %d points)\n", voxelSize, !cloudToExport->empty()?(int)cloudToExport->size():(int)cloudIToExport->size());
			if(!cloudToExport->empty())
			{
				cloudToExport = util3d::voxelize(cloudToExport, voxelSize);
			}
			else if(!cloudIToExport->empty())
			{
				cloudIToExport = util3d::voxelize(cloudIToExport, voxelSize);
			}
			printf("Voxel grid filtering of the assembled cloud.... done! (%fs, %d points)\n", timer.ticks(), !cloudToExport->empty()?(int)cloudToExport->size():(int)cloudIToExport->size());
		}

		std::vector<int> pointToCamId;
		if(camProjection && !robotPoses.empty())
		{
			printf("Camera projection...\n");
			pointToCamId.resize(!cloudToExport->empty()?cloudToExport->size():cloudIToExport->size());
			std::vector<std::pair< std::pair<int, int>, pcl::PointXY> > pointToPixel;
			if(!cloudToExport->empty())
			{
				pointToPixel = util3d::projectCloudToCameras(
						*cloudToExport,
						robotPoses,
						cameraModels,
						textureRange,
						0,
						std::vector<float>(),
						distanceToCamPolicy);
			}
			else if(!cloudIToExport->empty())
			{
				pointToPixel = util3d::projectCloudToCameras(
						*cloudIToExport,
						robotPoses,
						cameraModels,
						textureRange,
						0,
						std::vector<float>(),
						distanceToCamPolicy);
			}

			// color the cloud
			UASSERT(pointToPixel.empty() || pointToPixel.size() == pointToCamId.size());
			std::map<int, cv::Mat> cachedImages;
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr assembledCloudValidPoints(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
			assembledCloudValidPoints->resize(pointToCamId.size());

			int oi=0;
			for(size_t i=0; i<pointToPixel.size(); ++i)
			{
				pcl::PointXYZRGBNormal pt;
				if(!cloudToExport->empty())
				{
					pt = cloudToExport->at(i);
				}
				else if(!cloudIToExport->empty())
				{
					pt.x = cloudIToExport->at(i).x;
					pt.y = cloudIToExport->at(i).y;
					pt.z = cloudIToExport->at(i).z;
					pt.normal_x = cloudIToExport->at(i).normal_x;
					pt.normal_y = cloudIToExport->at(i).normal_y;
					pt.normal_z = cloudIToExport->at(i).normal_z;
				}
				int nodeID = pointToPixel[i].first.first;
				int cameraIndex = pointToPixel[i].first.second;
				if(nodeID>0 && cameraIndex>=0)
				{
					cv::Mat image;
					if(uContains(cachedImages, nodeID))
					{
						image = cachedImages.at(nodeID);
					}
					else if(uContains(nodes,nodeID) && !nodes.at(nodeID).sensorData().imageCompressed().empty())
					{
						nodes.at(nodeID).sensorData().uncompressDataConst(&image, 0);
						cachedImages.insert(std::make_pair(nodeID, image));
					}

					if(!image.empty())
					{
						int subImageWidth = image.cols / cameraModels.at(nodeID).size();
						image = image(cv::Range::all(), cv::Range(cameraIndex*subImageWidth, (cameraIndex+1)*subImageWidth));


						int x = pointToPixel[i].second.x * (float)image.cols;
						int y = pointToPixel[i].second.y * (float)image.rows;
						UASSERT(x>=0 && x<image.cols);
						UASSERT(y>=0 && y<image.rows);

						if(image.type()==CV_8UC3)
						{
							cv::Vec3b bgr = image.at<cv::Vec3b>(y, x);
							pt.b = bgr[0];
							pt.g = bgr[1];
							pt.r = bgr[2];
						}
						else
						{
							UASSERT(image.type()==CV_8UC1);
							pt.r = pt.g = pt.b = image.at<unsigned char>(pointToPixel[i].second.y * image.rows, pointToPixel[i].second.x * image.cols);
						}
					}

					int exportedId = nodeID;
					pointToCamId[oi] = exportedId;
					assembledCloudValidPoints->at(oi++) = pt;
				}
			}

			assembledCloudValidPoints->resize(oi);
			cloudToExport = assembledCloudValidPoints;
			cloudIToExport->clear();
			pointToCamId.resize(oi);

			printf("Camera projection... done! (%fs)\n", timer.ticks());
		}

		if(!(mesh || texture))
		{
			if(saveInDb)
			{
				printf("Saving in db... (%d points)\n", !cloudToExport->empty()?(int)cloudToExport->size():(int)cloudIToExport->size());
				if(!cloudToExport->empty())
					driver->saveOptimizedMesh(util3d::laserScanFromPointCloud(*cloudToExport, Transform(), false).data());
				else if(!cloudIToExport->empty())
					driver->saveOptimizedMesh(util3d::laserScanFromPointCloud(*cloudIToExport, Transform(), false).data());
				printf("Saving in db... done!\n");
			}
			else
			{
				std::string ext = las?"las":"ply";
				std::string outputPath=outputDirectory+"/"+baseName+"_cloud."+ext;
				printf("Saving %s... (%d points)\n", outputPath.c_str(), !cloudToExport->empty()?(int)cloudToExport->size():(int)cloudIToExport->size());
#ifdef RTABMAP_PDAL
				if(!cloudToExport->empty())
					savePDALFile(outputPath, *cloudToExport, pointToCamId, binary);
				else if(!cloudIToExport->empty())
					savePDALFile(outputPath, *cloudIToExport, pointToCamId, binary);
#else
				if(!pointToCamId.empty())
				{
					printf("Option --cam_projection is enabled but rtabmap is not built "
							"with PDAL support, so camera IDs won't be exported in the output cloud.\n");
				}
				if(!cloudToExport->empty())
					pcl::io::savePLYFile(outputPath, *cloudToExport, binary);
				else if(!cloudIToExport->empty())
					pcl::io::savePLYFile(outputPath, *cloudIToExport, binary);
#endif
				printf("Saving %s... done!\n", outputPath.c_str());
			}
		}

		// Meshing...
		if(mesh || texture)
		{
			if(!mergedCloudsI->empty())
			{
				pcl::copyPointCloud(*mergedCloudsI, *mergedClouds);
				mergedCloudsI->clear();
			}

			Eigen::Vector4f min,max;
			pcl::getMinMax3D(*mergedClouds, min, max);
			float mapLength = uMax3(max[0]-min[0], max[1]-min[1], max[2]-min[2]);
			int optimizedDepth = 12;
			for(int i=6; i<12; ++i)
			{
				if(mapLength/float(1<<i) < poissonSize)
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
				printf("Mesh color transfer...\n");
				rtabmap::util3d::denseMeshPostProcessing<pcl::PointXYZRGBNormal>(
						mesh,
						0.0f,
						maxPolygons,
						mergedClouds,
						colorRadius,
						!texture,
						doClean,
						200);
				printf("Mesh color transfer... done (%fs).\n", timer.ticks());

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
						if(binary)
							pcl::io::savePLYFileBinary(outputPath, *mesh);
						else
							pcl::io::savePLYFile(outputPath, *mesh);
						printf("Saving %s... done!\n", outputPath.c_str());
					}
				}
				else
				{
					printf("Texturing %d polygons... robotPoses=%d, cameraDepths=%d\n", (int)mesh->polygons.size(), (int)robotPoses.size(), (int)cameraDepths.size());
					std::vector<std::map<int, pcl::PointXY> > vertexToPixels;
					pcl::TextureMeshPtr textureMesh = rtabmap::util3d::createTextureMesh(
							mesh,
							robotPoses,
							cameraModels,
							cameraDepths,
							textureRange,
							textureDepthError,
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
									robotPoses,
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
