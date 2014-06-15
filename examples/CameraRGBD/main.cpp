/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
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
	if(argc > 1)
	{
		driver = atoi(argv[1]);
		if(driver < 0 || driver > 4)
		{
			UERROR("driver should be between 0 and 4.");
			showUsage();
		}
	}
	UINFO("Using driver %d", driver);

	rtabmap::CameraRGBD * camera;
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
	float constant;
	camera->takeImage(rgb, depth, constant);
	cv::namedWindow("Video", CV_WINDOW_AUTOSIZE); // create window
	cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE); // create window
	pcl::visualization::CloudViewer viewer("cloud");
	while(!rgb.empty() && !viewer.wasStopped())
	{
		cv::Mat tmp;
		depth.convertTo(tmp, CV_8UC1, 255.0/2048.0);

		cv::imshow("Video", rgb); // show frame
		cv::imshow("Depth",tmp);

		viewer.showCloud(rtabmap::util3d::cloudFromDepthRGB(rgb, depth, constant), "cloud");

		int c = cv::waitKey(10); // wait 10 ms or for key stroke
		if(c == 27)
			break; // if ESC, break and quit

		rgb = cv::Mat();
		depth = cv::Mat();
		camera->takeImage(rgb, depth, constant);
	}
	cv::destroyWindow("Video");
	cv::destroyWindow("Depth");
	delete camera;
	return 0;
}
