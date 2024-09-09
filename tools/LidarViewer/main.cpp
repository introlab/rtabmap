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

// Should be first on windows to avoid "WinSock.h has already been included" error
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/vlp_grabber.h>

#include "rtabmap/core/util2d.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UConversion.h"
#include <stdio.h>
#include <signal.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/parse.h>
#include <rtabmap/core/lidar/LidarVLP16.h>
#include <rtabmap/core/lidar/LidarOuster.h>

using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-lidar_viewer DRIVER IP [PORT]\n"
			"\n"
			"DRIVER: 0=VLP16, 1=Ouster\n"
			"PORT:   should be set if DRIVER=0\n"
			"\n"
			"Examples:\n"
			"  rtabmap-lidar_viewer 0 192.168.1.201 2368\n"
			"  rtabmap-lidar_viewer 1 192.168.1.2\n\n");
	exit(1);
}

// catch ctrl-c
bool running = true;
void sighandler(int sig)
{
	printf("\nSignal %d caught...\n", sig);
	running = false;
}

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);
	//ULogger::setPrintTime(false);
	//ULogger::setPrintWhere(false);

	int driver = 0;
	std::string ip;
	int port = 2368;
	if(argc < 3)
	{
		printf("Not enough arguments.\n");
		showUsage();
	}
	else
	{
		driver = uStr2Int(argv[1]);
		if(driver < 0 || driver > 1)
		{
			printf("driver should be 0 or 1.\n");
			showUsage();
		}

		ip = argv[2];
		if(driver==0)
		{
			if(argc>=3)
			{
				port = uStr2Int(argv[3]);
			}
			else
			{
				printf("IP and PORT should be set with VLP16 driver\n");
				showUsage();
			}
		}
	}
	printf("Using driver %d (ip=%s%s)\n", driver, ip.c_str(), driver==0?uFormat(" port=%d", port).c_str():"");

	rtabmap::Lidar * lidar = 0;
	if(driver == 0)
	{
		lidar = new rtabmap::LidarVLP16(boost::asio::ip::address_v4::from_string(ip), port);
	}
	else if(driver == 1)
	{
		lidar = new rtabmap::LidarOuster(ip);
	}
	else
	{
		UFATAL("");
	}

	if(!lidar->init())
	{
		printf("Lidar init failed!\n");
		delete lidar;
		exit(1);
	}

	pcl::visualization::CloudViewer * viewer = new pcl::visualization::CloudViewer("cloud");

	// to catch the ctrl-c
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	rtabmap::SensorData data = lidar->takeData();
	while(!data.laserScanRaw().empty() && (viewer==0 || !viewer->wasStopped()) && running)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = rtabmap::util3d::laserScanToPointCloudI(data.laserScanRaw(), data.laserScanRaw().localTransform());
		printf("Scan size: %ld points\n", cloud->size());
		viewer->showCloud(cloud, "cloud");

		int c = cv::waitKey(10); // wait 10 ms or for key stroke
		if(c == 27)
			break; // if ESC, break and quit

		data = lidar->takeData();
	}
	printf("Closing...\n");
	if(viewer)
	{
		delete viewer;
	}
	delete lidar;
	return 0;
}
