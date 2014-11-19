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

#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/utilite/ULogger.h"
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <stdio.h>

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-rgbd_camera driver\n"
			"  driver       Driver number to use: 0=OpenNI-PCL, 1=OpenNI2, 2=Freenect, 3=OpenNI-CV, 4=OpenNI-CV-ASUS\n\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	int driver = 0;
	if(argc < 2)
	{
		showUsage();
	}
	else
	{
		driver = atoi(argv[argc-1]);
		if(driver < 0 || driver > 4)
		{
			UERROR("driver should be between 0 and 4.");
			showUsage();
		}
	}
	UINFO("Using driver %d", driver);

	rtabmap::CameraRGBD * camera = 0;
	if(driver == 0)
	{
		camera = new rtabmap::CameraOpenni("", 0);
	}
	else if(driver == 1)
	{
		if(!rtabmap::CameraOpenNI2::available())
		{
			UERROR("Not built with OpenNI2 support...");
			exit(-1);
		}
		camera = new rtabmap::CameraOpenNI2(0);
	}
	else if(driver == 2)
	{
		if(!rtabmap::CameraFreenect::available())
		{
			UERROR("Not built with Freenect support...");
			exit(-1);
		}
		camera = new rtabmap::CameraFreenect(0, 0);
	}
	else if(driver == 3)
	{
		if(!rtabmap::CameraOpenNICV::available())
		{
			UERROR("Not built with OpenNI from OpenCV support...");
			exit(-1);
		}
		camera = new rtabmap::CameraOpenNICV(false, 0);
	}
	else if(driver == 4)
	{
		if(!rtabmap::CameraOpenNICV::available())
		{
			UERROR("Not built with OpenNI from OpenCV support...");
			exit(-1);
		}
		camera = new rtabmap::CameraOpenNICV(true, 0);
	}
	else
	{
		UFATAL("");
	}

	if(!camera->init())
	{
		printf("Camera init failed!\n");
		delete camera;
		exit(1);
	}

	cv::Mat rgb, depth;
	float fx, fy, cx, cy;
	camera->takeImage(rgb, depth, fx, fy, cx, cy);
	cv::namedWindow("Video", CV_WINDOW_AUTOSIZE); // create window
	cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE); // create window
	pcl::visualization::CloudViewer viewer("cloud");
	rtabmap::Transform opticalTransform(0,0,1,0, -1,0,0,0, 0,-1,0,0);
	while(!rgb.empty() && !viewer.wasStopped())
	{
		cv::Mat tmp;
		depth.convertTo(tmp, CV_8UC1, 255.0/2048.0);

		cv::imshow("Video", rgb); // show frame
		cv::imshow("Depth",tmp);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = rtabmap::util3d::cloudFromDepthRGB(rgb, depth, cx, cy, fx, fy);
		cloud = rtabmap::util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, opticalTransform);
		viewer.showCloud(cloud, "cloud");

		int c = cv::waitKey(10); // wait 10 ms or for key stroke
		if(c == 27)
			break; // if ESC, break and quit

		rgb = cv::Mat();
		depth = cv::Mat();
		camera->takeImage(rgb, depth, fx, fy, cx, cy);
	}
	cv::destroyWindow("Video");
	cv::destroyWindow("Depth");
	delete camera;
	return 0;
}
