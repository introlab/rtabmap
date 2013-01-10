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

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Camera.h"
#include <opencv2/core/core.hpp>

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-example \"path\"\n"
			"  path   Path to a directory of images\n ");
	exit(1);
}

int main(int argc, char * argv[])
{
	if(argc < 2)
	{
		showUsage();
	}
	std::string path = argv[1];

	// rtabmap::Camera is simply a convenience wrapper of OpenCV cv::VideoCapture and cv::imread
	float imageRate = 1.0f; // 1 Hz
	rtabmap::CameraImages camera(path, 1, false, imageRate);

	if(!camera.init())
	{
		printf("Camera init failed, using path \"%s\"\n", path.c_str());
		exit(1);
	}

	// Create tasks
	rtabmap::Rtabmap rtabmap;

	// Where database is saved
	rtabmap.setWorkingDirectory(".");

	// Initialize rtabmap: load database...
	// It will automatically create config.ini
	// with default parameters if it not exists
	rtabmap.init("./config.ini");

	// For this example, we want to delete the memory at each start.
	rtabmap.deleteMemory();

	rtabmap.setMaxTimeAllowed(700); // Time threshold : 700 ms

	// Start thread's task
	int countLoopDetected=0;

	printf("\nProcessing images at %f Hz... from directory \"%s\"\n", imageRate, path.c_str());

	int imagesProcessed = 0;

	cv::Mat img = camera.takeImage();
	int i=0;
	while(!img.empty())
	{
		++imagesProcessed;
		rtabmap.process(rtabmap::Image(img));
		if(rtabmap.getLoopClosureId())
		{
			++countLoopDetected;
		}
		img = camera.takeImage();

		++i;

		if(rtabmap.getLoopClosureId())
		{
			printf(" iteration(%d) ptime(%fs) loop(%d) hyp(%.2f)\n",
					i, rtabmap.getLastProcessTime(), rtabmap.getLoopClosureId(), rtabmap.getLcHypValue());
		}
		else
		{
			printf(" iteration(%d) ptime(%fs)\n", i, rtabmap.getLastProcessTime());
		}
	}

	printf("Processing images completed. Loop closures found = %d\n", countLoopDetected);

	return 0;
}
