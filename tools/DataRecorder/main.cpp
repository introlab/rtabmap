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

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/gui/DataRecorder.h>
#include <QtGui/QApplication>
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
			"  -openni                  Use openni camera instead of the usb camera.\n"
			"  -openni2                 Use openni2 camera instead of the usb camera.\n");
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
	bool openni = false;
	bool openni2 = false;
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
				rate = std::atof(argv[i]);
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
		if(strcmp(argv[i], "-openni") == 0)
		{
			openni = true;
			continue;
		}
		if(strcmp(argv[i], "-openni2") == 0)
		{
			openni2 = true;
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
	UINFO("Openni = %s", openni?"true":"false");
	UINFO("Rate =%f Hz", rate);

	app = new QApplication(argc, argv);

	// Catch ctrl-c to close the gui
	// (Place this after QApplication's constructor)
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	if(openni2)
	{
		cam = new rtabmap::CameraThread(new rtabmap::CameraOpenNI2(rate, rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0)));
	}
	else if(openni)
	{
		cam = new rtabmap::CameraThread(new rtabmap::CameraOpenni("", rate, rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0)));
	}
	else
	{
		cam = new rtabmap::CameraThread(new rtabmap::CameraVideo(0, rate));
	}

	DataRecorder recorder;

	if(recorder.init(fileName))
	{
		recorder.registerToEventsManager();
		if(show)
		{
			recorder.setWindowTitle("Cloud viewer");
			recorder.setMinimumWidth(500);
			recorder.setMinimumHeight(300);
			recorder.showNormal();
			app->processEvents();
		}

		if(cam->init())
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
