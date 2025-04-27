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
#include <pcl/visualization/cloud_viewer.h>
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

using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-lidar_viewer IP PORT driver\n"
			"  driver       Driver number to use: 0=VLP16 (default IP and port are 192.168.1.201 2368)\n");
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
	if(argc < 4)
	{
		showUsage();
	}
	else
	{
		ip = argv[1];
		port = uStr2Int(argv[2]);
		driver = atoi(argv[3]);
		if(driver < 0 || driver > 0)
		{
			UERROR("driver should be 0.");
			showUsage();
		}
	}
	printf("Using driver %d (ip=%s port=%d)\n", driver, ip.c_str(), port);

	rtabmap::LidarVLP16 * lidar = 0;
	if(driver == 0)
	{
#if BOOST_VERSION >= 108700  // Version 1.87.0
		lidar = new rtabmap::LidarVLP16(boost::asio::ip::make_address(ip), port);
#else
		lidar = new rtabmap::LidarVLP16(boost::asio::ip::address_v4::from_string(ip), port);
#endif
	}
	else
	{
		UFATAL("");
	}

	if(!lidar->init())
	{
		printf("Lidar init failed! Please select another driver (see \"--help\").\n");
		delete lidar;
		exit(1);
	}

	pcl::visualization::CloudViewer * viewer = new pcl::visualization::CloudViewer("cloud");

	// to catch the ctrl-c
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	rtabmap::SensorData data = lidar->takeScan();
	while(!data.laserScanRaw().empty() && (viewer==0 || !viewer->wasStopped()) && running)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = rtabmap::util3d::laserScanToPointCloudI(data.laserScanRaw(), data.laserScanRaw().localTransform());
		viewer->showCloud(cloud, "cloud");
		printf("Scan size: %ld points\n", cloud->size());

		int c = cv::waitKey(10); // wait 10 ms or for key stroke
		if(c == 27)
			break; // if ESC, break and quit

		data = lidar->takeScan();
	}
	printf("Closing...\n");
	if(viewer)
	{
		delete viewer;
	}
	delete lidar;
	return 0;
}
