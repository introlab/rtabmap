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

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/CameraStereo.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/gui/DataRecorder.h>
#include <QApplication>
#include <signal.h>

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"dataRecorder [options] output.db\n"
			"Options:\n"
			"  -hide                    Don't display the current cloud recorded.\n"
			"  -debug                   Set debug level for the logger.\n"
			"  -rate #.#                Input rate Hz (default 0=inf)\n"
			"  -driver                  Driver number to use:\n"
			"                                     0=OpenNI-PCL (Kinect)\n"
			"                                     1=OpenNI2    (Kinect and Xtion PRO Live)\n"
			"                                     2=Freenect   (Kinect)\n"
			"                                     3=OpenNI-CV  (Kinect)\n"
			"                                     4=OpenNI-CV-ASUS (Xtion PRO Live)\n"
			"                                     5=Freenect2  (Kinect v2)\n"
			"                                     6=DC1394     (Bumblebee2)\n"
			"                                     7=FlyCapture2 (Bumblebee2)\n"
			"  -device ""                Device ID (default \"\")\n");
	exit(1);
}

rtabmap::CameraThread * cam = 0;
QApplication * app = 0;
// catch ctrl-c
void sighandler(int sig)
{
	printf("\nSignal %d caught...\n", sig);
	if(cam)
	{
		cam->join(true);
	}
	if(app)
	{
		QMetaObject::invokeMethod(app, "quit");
	}
}

int main (int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	// parse arguments
	QString fileName;
	bool show = true;
	int driver = 0;
	std::string deviceId;
	float rate = 0.0f;

	if(argc < 2)
	{
		showUsage();
	}
	for(int i=1; i<argc-1; ++i)
	{
		if(strcmp(argv[i], "-rate") == 0)
		{
			++i;
			if(i < argc)
			{
				rate = uStr2Float(argv[i]);
				if(rate < 0.0f)
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
		if(strcmp(argv[i], "-debug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
			continue;
		}
		if(strcmp(argv[i], "-hide") == 0)
		{
			show = false;
			continue;
		}
		if(strcmp(argv[i], "-driver") == 0)
		{
			++i;
			if(i < argc)
			{
				driver = std::atoi(argv[i]);
				if(driver < 0 || driver > 7)
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
		if(strcmp(argv[i], "-device") == 0)
		{
			++i;
			if(i < argc)
			{
				deviceId = argv[i];
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
	fileName = argv[argc-1]; // the last is the output path

	if(UFile::getExtension(fileName.toStdString()).compare("db") != 0)
	{
		printf("Database names must end with .db extension\n");
		showUsage();
	}

	UINFO("Output = %s", fileName.toStdString().c_str());
	UINFO("Show = %s", show?"true":"false");
	UINFO("Rate =%f Hz", rate);

	app = new QApplication(argc, argv);

	// Catch ctrl-c to close the gui
	// (Place this after QApplication's constructor)
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	rtabmap::Camera * camera = 0;
	rtabmap::Transform t=rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0);
	if(driver == 0)
	{
		camera = new rtabmap::CameraOpenni(deviceId, rate, t);
	}
	else if(driver == 1)
	{
		if(!rtabmap::CameraOpenNI2::available())
		{
			UERROR("Not built with OpenNI2 support...");
			exit(-1);
		}
		camera = new rtabmap::CameraOpenNI2(deviceId, CameraOpenNI2::kTypeColorDepth, rate, t);
	}
	else if(driver == 2)
	{
		if(!rtabmap::CameraFreenect::available())
		{
			UERROR("Not built with Freenect support...");
			exit(-1);
		}
		camera = new rtabmap::CameraFreenect(deviceId.size()?atoi(deviceId.c_str()):0, CameraFreenect::kTypeColorDepth, rate, t);
	}
	else if(driver == 3)
	{
		if(!rtabmap::CameraOpenNICV::available())
		{
			UERROR("Not built with OpenNI from OpenCV support...");
			exit(-1);
		}
		camera = new rtabmap::CameraOpenNICV(false, rate, t);
	}
	else if(driver == 4)
	{
		if(!rtabmap::CameraOpenNICV::available())
		{
			UERROR("Not built with OpenNI from OpenCV support...");
			exit(-1);
		}
		camera = new rtabmap::CameraOpenNICV(true, rate, t);
	}
	else if(driver == 5)
	{
		if(!rtabmap::CameraFreenect2::available())
		{
			UERROR("Not built with Freenect2 support...");
			exit(-1);
		}
		camera = new rtabmap::CameraFreenect2(deviceId.size()?atoi(deviceId.c_str()):0, rtabmap::CameraFreenect2::kTypeColor2DepthSD, rate, t);
	}
	else if(driver == 6)
	{
		if(!rtabmap::CameraStereoDC1394::available())
		{
			UERROR("Not built with dc1394 support...");
			exit(-1);
		}
		camera = new rtabmap::CameraStereoDC1394(rate, t);
	}
	else if(driver == 7)
	{
		if(!rtabmap::CameraStereoFlyCapture2::available())
		{
			UERROR("Not built with FlyCapture2/Triclops support...");
			exit(-1);
		}
		camera = new rtabmap::CameraStereoFlyCapture2(rate, t);
	}
	else
	{
		UFATAL("Camera driver (%d) not found!", driver);
	}
	cam = new CameraThread(camera);

	DataRecorder recorder;

	if(recorder.init(fileName))
	{
		recorder.registerToEventsManager();
		if(show)
		{
			recorder.setWindowTitle("Data recorder");
			recorder.setMinimumWidth(500);
			recorder.setMinimumHeight(300);
			recorder.showNormal();
			app->processEvents();
		}

		if(camera->init())
		{
			cam->start();

			app->exec();

			UINFO("Closing...");

			recorder.close();
		}
		else
		{
			UERROR("Cannot initialize the camera!");
		}
	}
	else
	{
		UERROR("Cannot initialize the recorder! Maybe the path is wrong: \"%s\"", fileName.toStdString().c_str());
	}

	if(cam)
	{
		delete cam;
	}

	return 0;
}
