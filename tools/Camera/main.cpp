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

#include "rtabmap/core/CameraRGB.h"
#include "rtabmap/core/DBReader.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UConversion.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <stdio.h>

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-camera [option] \n"
			" Options:\n"
			"    --device #            USB camera device id (default 0).\n"
			"    --rate #              Frame rate (default 0 Hz). 0 means as fast as possible.\n"
			"    --path ""             Path to a directory of images or a video file.\n"
			"    --calibration ""      Calibration file (*.yaml).\n\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	int device = 0;
	std::string path;
	float rate = 0.0f;
	std::string calibrationFile;
	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "--rate") == 0)
		{
			++i;
			if(i < argc)
			{
				rate = uStr2Float(argv[i]);
				if(rate < 0)
				{
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "--device") == 0)
		{
			++i;
			if(i < argc)
			{
				device = std::atoi(argv[i]);
				if(device < 0)
				{
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "--path") == 0)
		{
			++i;
			if(i < argc)
			{
				path = argv[i];
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "--calibration") == 0)
		{
			++i;
			if(i < argc)
			{
				calibrationFile = argv[i];
			}
			else
			{
				showUsage();
			}
			continue;
		}

		printf("Unrecognized option : %s\n", argv[i]);
		showUsage();
	}

	if(path.empty())
	{
		UINFO("Using device %d", device);
	}
	else
	{
		UINFO("Using path %s", path.c_str());
	}

	rtabmap::Camera * camera = 0;

	if(!path.empty())
	{
		if(UFile::exists(path))
		{
			if(UFile::getExtension(path).compare("db") == 0)
			{
				camera = new rtabmap::DBReader(path, rate);
			}
			else
			{
				camera = new rtabmap::CameraVideo(path, false, rate);
			}
		}
		else if(UDirectory::exists(path))
		{
			camera = new rtabmap::CameraImages(path, rate);
		}
		else
		{
			UERROR("Path not valid! \"%s\"", path.c_str());
			return -1;
		}
	}
	else
	{
		camera = new rtabmap::CameraVideo(device, false, rate);
	}

	if(camera)
	{
		if(!calibrationFile.empty())
		{
			UINFO("Set calibration: %s", calibrationFile.c_str());
		}
		if(!camera->init(UDirectory::getDir(calibrationFile), UFile::getName(calibrationFile)))
		{
			delete camera;
			UERROR("Cannot initialize the camera.");
			return -1;
		}
	}

	cv::Mat rgb;
	rgb = camera->takeImage().imageRaw();
	cv::namedWindow("Video", CV_WINDOW_AUTOSIZE); // create window
	while(!rgb.empty())
	{
		cv::imshow("Video", rgb); // show frame

		int c = cv::waitKey(10); // wait 10 ms or for key stroke
		if(c == 27)
			break; // if ESC, break and quit

		rgb = camera->takeImage().imageRaw();
	}
	cv::destroyWindow("Video");
	if(camera)
	{
		delete camera;
	}
	return 0;
}
