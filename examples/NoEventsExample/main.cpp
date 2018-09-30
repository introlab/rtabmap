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

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/CameraStereo.h>
#include <rtabmap/utilite/UThread.h>
#include "MapBuilder.h"
#include <pcl/visualization/cloud_viewer.h>
#include <rtabmap/core/Odometry.h>
#include <QApplication>
#include <stdio.h>

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-noEventsExample camera_rate odom_update map_update calibration_dir calibration_name path_left_images path_right_images\n"
			"Description:\n"
			"    camera_rate          Rate (Hz) of the camera.\n"
			"    odom_update          Do odometry update each X camera frames.\n"
			"    map_update           Do map update each X odometry frames.\n"
			"\n"
			"Example:\n"
			"     (with images from \"https://github.com/introlab/rtabmap/wiki/Stereo-mapping#process-a-directory-of-stereo-images\") \n"
			"     $ rtabmap-noEventsExample 20 2 10 stereo_20Hz stereo_20Hz stereo_20Hz/left stereo_20Hz/right\n"
			"       Camera rate = 20 Hz\n"
			"       Odometry update rate = 10 Hz\n"
			"       Map update rate = 1 Hz\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kError);

	if(argc < 8)
	{
		showUsage();
	}

	int argIndex = 1;
	int cameraRate = atoi(argv[argIndex++]);
	if(cameraRate <= 0)
	{
		printf("camera_rate should be > 0\n");
		showUsage();
	}
	int odomUpdate = atoi(argv[argIndex++]);
	if(odomUpdate <= 0)
	{
		printf("odom_update should be > 0\n");
		showUsage();
	}
	int mapUpdate = atoi(argv[argIndex++]);
	if(mapUpdate <= 0)
	{
		printf("map_update should be > 0\n");
		showUsage();
	}

	printf("Camera rate = %d Hz\n", cameraRate);
	printf("Odometry update rate = %d Hz\n", cameraRate/odomUpdate);
	printf("Map update rate = %d Hz\n", (cameraRate/odomUpdate)/mapUpdate);

	std::string calibrationDir = argv[argIndex++];
	std::string calibrationName = argv[argIndex++];
	std::string pathLeftImages = argv[argIndex++];
	std::string pathRightImages = argv[argIndex++];

	Transform opticalRotation(0,0,1,0, -1,0,0,0, 0,-1,0,0);
	CameraStereoImages camera(
			pathLeftImages,
			pathRightImages,
			false, // assume that images are already rectified
			(float)cameraRate,
			opticalRotation);

	if(camera.init(calibrationDir, calibrationName))
	{
		Odometry * odom = Odometry::create();
		Rtabmap rtabmap;
		rtabmap.init();

		QApplication app(argc, argv);
		MapBuilder mapBuilder;
		mapBuilder.show();
		QApplication::processEvents();

		SensorData data = camera.takeImage();
		int cameraIteration = 0;
		int odometryIteration = 0;
		printf("Press \"Space\" in the window to pause\n");
		while(data.isValid() && mapBuilder.isVisible())
		{
			if(cameraIteration++ % odomUpdate == 0)
			{
				OdometryInfo info;
				Transform pose = odom->process(data, &info);

				if(odometryIteration++ % mapUpdate == 0)
				{
					if(rtabmap.process(data, pose))
					{
						mapBuilder.processStatistics(rtabmap.getStatistics());
						if(rtabmap.getLoopClosureId() > 0)
						{
							printf("Loop closure detected!\n");
						}
					}
				}

				mapBuilder.processOdometry(data, pose, info);
			}

			QApplication::processEvents();

			while(mapBuilder.isPaused() && mapBuilder.isVisible())
			{
				uSleep(100);
				QApplication::processEvents();
			}

			data = camera.takeImage();
		}
		delete odom;

		if(mapBuilder.isVisible())
		{
			printf("Processed all frames\n");
			app.exec();
		}
	}
	else
	{
		UERROR("Camera init failed!");
	}

	return 0;
}
