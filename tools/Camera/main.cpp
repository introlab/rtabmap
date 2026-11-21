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
#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/UCv2Qt.h"
#include <QApplication>
#include <rtabmap/gui/QtPlatform.h>
#include <QKeyEvent>
#include <QLabel>
#include <QPixmap>
#include <stdio.h>

// Images are shown with Qt instead of cv::imshow(): linking opencv_highgui
// can load a different Qt major version than rtabmap_gui (e.g., Qt5 vs Qt6
// on Ubuntu 26.04), which crashes the application on startup.
bool running = true;
class EscapeFilter : public QObject
{
protected:
	virtual bool eventFilter(QObject * obj, QEvent * event)
	{
		if(event->type() == QEvent::KeyPress && ((QKeyEvent*)event)->key() == Qt::Key_Escape)
		{
			running = false; // if ESC, quit
		}
		return QObject::eventFilter(obj, event);
	}
};

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

	rtabmap::useXWaylandOnGnome(); // must be before QApplication
	QApplication app(argc, argv);
	EscapeFilter escapeFilter;
	app.installEventFilter(&escapeFilter);
	QLabel view;
	view.setWindowTitle("Video");

	cv::Mat rgb;
	rgb = camera->takeImage().imageRaw();
	while(!rgb.empty() && running)
	{
		view.setPixmap(QPixmap::fromImage(uCvMat2QImage(rgb))); // show frame
		if(!view.isVisible())
		{
			view.resize(rgb.cols, rgb.rows);
			view.show();
		}

		app.processEvents();
		uSleep(10);
		if(!view.isVisible())
			break; // window closed

		rgb = camera->takeImage().imageRaw();
	}
	if(camera)
	{
		delete camera;
	}
	return 0;
}
