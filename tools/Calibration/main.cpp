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
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/gui/CalibrationDialog.h"
#include <QApplication>

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-calibration [options]\n"
			"Options:\n"
			"  --driver #     Driver number to use:-1=USB camera\n"
			"                                       0=OpenNI-PCL (Kinect)\n"
			"                                       1=OpenNI2    (Kinect and Xtion PRO Live)\n"
			"                                       2=Freenect   (Kinect)\n"
			"                                       3=OpenNI-CV  (Kinect)\n"
			"                                       4=OpenNI-CV-ASUS (Xtion PRO Live)\n"
			"                                       5=Freenect2  (Kinect v2)\n"
			"                                       6=DC1394     (Bumblebee2)\n"
			"                                       7=FlyCapture2 (Bumblebee2)\n"
			"  --device #     Device id\n"
			"  --debug        Debug log\n"
			"  --stereo       Stereo: assuming device provides \n"
			"                 side-by-side stereo images, otherwise \n"
			"                 add also \"--device_r #\" for the right device.\n\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);
	ULogger::setPrintTime(false);
	ULogger::setPrintWhere(false);

	int driver = -1;
	int device = 0;
	int deviceRight = -1;
	bool stereo = false;
	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "--driver") == 0)
		{
			++i;
			if(i < argc)
			{
				driver = std::atoi(argv[i]);
				if(driver < -1)
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
		if(strcmp(argv[i], "--device_r") == 0)
		{
			++i;
			if(i < argc)
			{
				deviceRight = std::atoi(argv[i]);
				if(deviceRight < 0)
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
		if(strcmp(argv[i], "--debug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
			ULogger::setPrintTime(true);
			ULogger::setPrintWhere(true);
			continue;
		}
		if(strcmp(argv[i], "--stereo") == 0)
		{
			stereo=true;
			continue;
		}
		if(strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}
		printf("Unrecognized option : %s\n", argv[i]);
		showUsage();
	}
	if(driver < -1 || driver > 7)
	{
		UERROR("driver should be between -1 and 7.");
		showUsage();
	}

	UINFO("Using driver %d", driver);
	UINFO("Using device %d", device);
	UINFO("Stereo: %s", stereo?"true":"false");
	if(stereo && deviceRight >= 0)
	{
		UINFO("Using right device %d", deviceRight);
	}

	QApplication app(argc, argv);
	rtabmap::CalibrationDialog dialog(stereo, ".");

	rtabmap::Camera * camera = 0;
	if(driver == -1)
	{
		if(stereo)
		{
			if(deviceRight>=0)
			{
				// left and right videos
				camera = new rtabmap::CameraStereoVideo(device, deviceRight);
			}
			else
			{
				// side-by-side video
				camera = new rtabmap::CameraStereoVideo(device);
			}
		}
		else
		{
			camera = new rtabmap::CameraVideo(device);
		}
		dialog.setStereoMode(stereo);
	}
	else if(driver == 0)
	{
		camera = new rtabmap::CameraOpenni();
	}
	else if(driver == 1)
	{
		if(!rtabmap::CameraOpenNI2::available())
		{
			UERROR("Not built with OpenNI2 support...");
			exit(-1);
		}
		camera = new rtabmap::CameraOpenNI2();
	}
	else if(driver == 2)
	{
		if(!rtabmap::CameraFreenect::available())
		{
			UERROR("Not built with Freenect support...");
			exit(-1);
		}
		camera = new rtabmap::CameraFreenect();
	}
	else if(driver == 3)
	{
		if(!rtabmap::CameraOpenNICV::available())
		{
			UERROR("Not built with OpenNI from OpenCV support...");
			exit(-1);
		}
		camera = new rtabmap::CameraOpenNICV(false);
	}
	else if(driver == 4)
	{
		if(!rtabmap::CameraOpenNICV::available())
		{
			UERROR("Not built with OpenNI from OpenCV support...");
			exit(-1);
		}
		camera = new rtabmap::CameraOpenNICV(true);
	}
	else if(driver == 5)
	{
		if(!rtabmap::CameraFreenect2::available())
		{
			UERROR("Not built with Freenect2 support...");
			exit(-1);
		}
		camera = new rtabmap::CameraFreenect2(0, rtabmap::CameraFreenect2::kTypeColorIR);
		dialog.setSwitchedImages(true);
		dialog.setStereoMode(stereo, "rgb", "depth");
	}
	else if(driver == 6)
	{
		if(!rtabmap::CameraStereoDC1394::available())
		{
			UERROR("Not built with DC1394 support...");
			exit(-1);
		}
		camera = new rtabmap::CameraStereoDC1394();
		dialog.setStereoMode(stereo);
	}
	else if(driver == 7)
	{
		if(!rtabmap::CameraStereoFlyCapture2::available())
		{
			UERROR("Not built with FlyCapture2/Triclops support...");
			exit(-1);
		}
		camera = new rtabmap::CameraStereoFlyCapture2();
		dialog.setStereoMode(stereo);
	}
	else
	{
		UFATAL("");
	}

	rtabmap::CameraThread * cameraThread = 0;

	if(camera)
	{
		if(!camera->init(""))
		{
			printf("Camera init failed!\n");
			delete camera;
			exit(1);
		}
		cameraThread = new rtabmap::CameraThread(camera);
	}

	dialog.registerToEventsManager();

	dialog.show();
	cameraThread->start();
	app.exec();
	cameraThread->join(true);
	delete cameraThread;
}
