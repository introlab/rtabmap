/*
Copyright (c) 2010-2022, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the names of its contributors may be used to endorse or promote products
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

// Should be first on windows to avoid "WinSock.h has already been included" error
#include "rtabmap/core/lidar/LidarVLP16.h"

#include <rtabmap/core/Odometry.h>
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/utilite/UEventsManager.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UDirectory.h"
#include <QApplication>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <rtabmap/core/SensorCaptureThread.h>

#include "MapBuilder.h"

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-lidar_mapping IP PORT\n"
			"rtabmap-lidar_mapping PCAP_FILEPATH\n"
			"\n"
			"Example:"
			"  rtabmap-lidar_mapping 192.168.1.201 2368\n\n");
	exit(1);
}

using namespace rtabmap;
int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	std::string filepath;
	std::string ip;
	int port = 2368;
	if(argc < 2)
	{
		showUsage();
	}
	else if(argc == 2)
	{
		filepath = argv[1];
	}
	else
	{
		ip = argv[1];
		port = uStr2Int(argv[2]);
	}

	// Here is the pipeline that we will use:
	// LidarVLP16 -> "SensorEvent" -> OdometryThread -> "OdometryEvent" -> RtabmapThread -> "RtabmapEvent"

	// Create the Lidar sensor, it will send a SensorEvent
	LidarVLP16 * lidar;
	if(!ip.empty())
	{
		printf("Using ip=%s port=%d\n", ip.c_str(), port);
#if BOOST_VERSION >= 108700  // Version 1.87.0
		lidar = new LidarVLP16(boost::asio::ip::make_address(ip), port);
#else
		lidar = new LidarVLP16(boost::asio::ip::address_v4::from_string(ip), port);
#endif
	}
	else
	{
		filepath = uReplaceChar(filepath, '~', UDirectory::homeDir());
		printf("Using file=%s\n", filepath.c_str());
		lidar = new LidarVLP16(filepath);
	}
	lidar->setOrganized(true); //faster deskewing

	if(!lidar->init())
	{
		UERROR("Lidar init failed!");
		delete lidar;
		return -1;
	}

	SensorCaptureThread lidarThread(lidar);

	// GUI stuff, there the handler will receive RtabmapEvent and construct the map
	// We give it the lidar so the GUI can pause/resume the lidar
	QApplication app(argc, argv);
	MapBuilder mapBuilder(&lidarThread);

	ParametersMap params;

	float resolution = 0.05;

	// ICP parameters
	params.insert(ParametersPair(Parameters::kRegStrategy(), "1"));
	params.insert(ParametersPair(Parameters::kIcpFiltersEnabled(), "2"));
	params.insert(ParametersPair(Parameters::kIcpPointToPlane(), "true"));
	params.insert(ParametersPair(Parameters::kIcpPointToPlaneK(), "20"));
	params.insert(ParametersPair(Parameters::kIcpPointToPlaneRadius(), "0"));
	params.insert(ParametersPair(Parameters::kIcpIterations(), "10"));
	params.insert(ParametersPair(Parameters::kIcpVoxelSize(), uNumber2Str(resolution)));
	params.insert(ParametersPair(Parameters::kIcpEpsilon(), "0.001"));
	params.insert(ParametersPair(Parameters::kIcpMaxCorrespondenceDistance(), uNumber2Str(resolution*10.0f)));
	params.insert(ParametersPair(Parameters::kIcpMaxTranslation(), "2"));
	params.insert(ParametersPair(Parameters::kIcpStrategy(), "1"));
	params.insert(ParametersPair(Parameters::kIcpOutlierRatio(), "0.7"));
	params.insert(ParametersPair(Parameters::kIcpCorrespondenceRatio(), "0.01"));
	// Uncomment if lidar never pitch/roll (on a car or wheeled robot), for hand-held mapping, keep it commented
	//params.insert(ParametersPair(Parameters::kIcpPointToPlaneGroundNormalsUp(), "0.8"));

	// Odom parameters
	params.insert(ParametersPair(Parameters::kOdomStrategy(), "0")); // F2M
	params.insert(ParametersPair(Parameters::kOdomScanKeyFrameThr(), "0.6"));
	params.insert(ParametersPair(Parameters::kOdomF2MScanSubtractRadius(), uNumber2Str(resolution)));
	params.insert(ParametersPair(Parameters::kOdomF2MScanMaxSize(), "15000"));
	params.insert(ParametersPair(Parameters::kOdomGuessSmoothingDelay(), "0.3"));
	params.insert(ParametersPair(Parameters::kOdomDeskewing(), "true"));

	// Create an odometry thread to process lidar events, it will send OdometryEvent.
	OdometryThread odomThread(Odometry::create(params));

	// Rtabmap params
	params.insert(ParametersPair(Parameters::kRGBDProximityBySpace(), "true"));
	params.insert(ParametersPair(Parameters::kRGBDProximityMaxGraphDepth(), "0"));
	params.insert(ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "1"));
	params.insert(ParametersPair(Parameters::kRGBDAngularUpdate(), "0.05"));
	params.insert(ParametersPair(Parameters::kRGBDLinearUpdate(), "0.05"));
	params.insert(ParametersPair(Parameters::kMemNotLinkedNodesKept(), "false"));
	params.insert(ParametersPair(Parameters::kMemSTMSize(), "30"));
	uInsert(params, ParametersPair(Parameters::kIcpCorrespondenceRatio(), "0.2")); // overwritten


	// Create RTAB-Map to process OdometryEvent
	Rtabmap * rtabmap = new Rtabmap();
	rtabmap->init(params);
	RtabmapThread rtabmapThread(rtabmap); // ownership is transfered

	// Setup handlers
	odomThread.registerToEventsManager();
	rtabmapThread.registerToEventsManager();
	mapBuilder.registerToEventsManager();

	// The RTAB-Map is subscribed by default to SensorEvent, but we want
	// RTAB-Map to process OdometryEvent instead, ignoring the SensorEvent.
	// We can do that by creating a "pipe" between the lidar and odometry, then
	// only the odometry will receive SensorEvent from that lidar. RTAB-Map is
	// also subscribed to OdometryEvent by default, so no need to create a pipe between
	// odometry and RTAB-Map.
	UEventsManager::createPipe(&lidarThread, &odomThread, "SensorEvent");

	// Let's start the threads
	rtabmapThread.start();
	odomThread.start();
	lidarThread.start();

	printf("Press Tab key to switch between map and odom views (or both).\n");
	printf("Press Space key to pause.\n");

	mapBuilder.show();
	app.exec(); // main loop

	// remove handlers
	mapBuilder.unregisterFromEventsManager();
	rtabmapThread.unregisterFromEventsManager();
	odomThread.unregisterFromEventsManager();

	// Kill all threads
	lidarThread.kill();
	odomThread.join(true);
	rtabmapThread.join(true);

	// Save 3D map
	printf("Saving rtabmap_cloud.pcd...\n");
	std::map<int, Signature> nodes;
	std::map<int, Transform> optimizedPoses;
	std::multimap<int, Link> links;
	rtabmap->getGraph(optimizedPoses, links, true, true, &nodes, true, true, true, true);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	for(std::map<int, Transform>::iterator iter=optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
	{
		Signature node = nodes.find(iter->first)->second;

		// uncompress data
		node.sensorData().uncompressData();

		pcl::PointCloud<pcl::PointXYZI>::Ptr tmp = util3d::laserScanToPointCloudI(node.sensorData().laserScanRaw(), node.sensorData().laserScanRaw().localTransform());
		*cloud += *util3d::transformPointCloud(tmp, iter->second); // transform the point cloud to its pose
	}
	if(cloud->size())
	{
		printf("Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n", 0.01f, (int)cloud->size());
		cloud = util3d::voxelize(cloud, 0.01f);

		printf("Saving rtabmap_cloud.pcd... done! (%d points)\n", (int)cloud->size());
		pcl::io::savePCDFile("rtabmap_cloud.pcd", *cloud);
		//pcl::io::savePLYFile("rtabmap_cloud.ply", *cloud); // to save in PLY format
	}
	else
	{
		printf("Saving rtabmap_cloud.pcd... failed! The cloud is empty.\n");
	}

	// Save trajectory
	printf("Saving rtabmap_trajectory.txt ...\n");
	if(optimizedPoses.size() && graph::exportPoses("rtabmap_trajectory.txt", 0, optimizedPoses, links))
	{
		printf("Saving rtabmap_trajectory.txt... done!\n");
	}
	else
	{
		printf("Saving rtabmap_trajectory.txt... failed!\n");
	}

	rtabmap->close(false);

	return 0;
}
