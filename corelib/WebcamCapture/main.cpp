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

#include "utilite/ULogger.h"
#include "utilite/UTimer.h"
#include "utilite/UDirectory.h"
#include "utilite/UConversion.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/SMState.h"
#include "utilite/UEventsManager.h"
#include "utilite/UEventsHandler.h"
#include "rtabmap/core/CameraEvent.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

void showUsage()
{
	printf("Usage:\n"
			"webcamCapture [option]\n"
			"   -device #:			Id of the webcam (default 0)\n"
			"   -width #:			Image width (default 640)\n"
			"   -height #:			Image height (default 480)\n"
			"   -fps #.#:			Frame rate (Hz) (default 2.0)\n"
			"   -show bool:			Image shown while capturing (default true)\n"
			"   -dir \"path\":		Path of the images saved (default \"./imagesCaptured\")\n"
			"   -save bool:			Save images captured (default true)\n"
			"   -startId #:			Image id to start with (default 1)\n"
			"   -ext \"ext\":		Image extension (default \"jpg\")\n"
			"   -debug:				Debug trace\n");
	exit(1);
}

class ImagesHandler : public UEventsHandler
{
public:
	ImagesHandler(const std::string & targetDir, const std::string & ext, int startId,  bool show, bool save) :
		targetDir_(targetDir),
		ext_(ext),
		startId_(startId),
		show_(show),
		save_(save),
		id_(startId)
	{
		if(show_)
		{
			cvNamedWindow("Webcam", CV_WINDOW_AUTOSIZE);
		}
	}
	virtual ~ImagesHandler()
	{
		if(show_)
		{
			cvDestroyWindow("Webcam");
		}
	}

protected:
	virtual void handleEvent(UEvent * e)
	{
		if(e->getClassName().compare("SMStateEvent") == 0)
		{
			const rtabmap::SMStateEvent * event = (const rtabmap::SMStateEvent*)e;
			const rtabmap::SMState * sm = event->getData();
			const IplImage * image = 0;
			if(sm)
			{
				image = sm->getImage();
			}
			if(image)
			{
				if(show_)
				{
					cvShowImage("Webcam", image);
				}
				if(save_)
				{
					std::string fileName = targetDir_ + "/";
					fileName += uNumber2str(id_++);
					fileName += ".";
					fileName += ext_;
					cvSaveImage(fileName.c_str(), image);
					printf("Image %s saved!\n", fileName.c_str());
				}
			}
			else
			{
				printf("Image is null?!?\n");
			}
		}

	}
private:
	std::string targetDir_;
	std::string ext_;
	int startId_;
	bool show_;
	bool save_;
	int id_;
};

int main(int argc, char * argv[])
{
	bool show = true;
	int usbDevice = 0;
	int imageWidth = 640;
	int imageHeight = 480;
	float imageRate = 2.0;
	int startId = 1;
	std::string extension = "jpg";
	bool save = true;
	std::string targetDirectory = UDirectory::currentDir(true) + "imagesCaptured";

	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "-h") == 0 ||
			strcmp(argv[i], "-help") == 0 ||
			strcmp(argv[i], "?") == 0)
		{
			showUsage();
		}

		if(strcmp(argv[i], "-device") == 0 && i+1<argc)
		{
			usbDevice = std::atoi(argv[i+1]);
			if(usbDevice < 0 || usbDevice > 100)
			{
				showUsage();
			}
			++i;
		}

		if(strcmp(argv[i], "-width") == 0 && i+1<argc)
		{
			imageWidth = std::atoi(argv[i+1]);
			if(imageWidth < 0)
			{
				showUsage();
			}
			++i;
		}

		if(strcmp(argv[i], "-height") == 0 && i+1<argc)
		{
			imageHeight = std::atoi(argv[i+1]);
			if(imageHeight < 0)
			{
				showUsage();
			}
			++i;
		}

		if(strcmp(argv[i], "-hz") == 0 && i+1<argc)
		{
			imageRate = std::atof(argv[i+1]);
			if(imageRate < 0)
			{
				showUsage();
			}
			++i;
		}

		if(strcmp(argv[i], "-show") == 0 && i+1<argc)
		{
			show = uStr2Bool(argv[i+1]);
			++i;
		}

		if(strcmp(argv[i], "-dir") == 0 && i+1<argc)
		{
			targetDirectory = argv[i+1];
			++i;
		}

		if(strcmp(argv[i], "-save") == 0 && i+1<argc)
		{
			save = uStr2Bool(argv[i+1]);
			++i;
		}
		
		if(strcmp(argv[i], "-debug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
			ULogger::setType(ULogger::kTypeConsole);
		}
		
		if(strcmp(argv[i], "-startId") == 0 && i+1<argc)
		{
			startId = std::atoi(argv[i+1]);
			++i;
		}
		
		if(strcmp(argv[i], "-ext") == 0 && i+1<argc)
		{
			extension = argv[i+1];
			++i;
		}
	}

	printf("Parameters:\n"
		   "   device=%d\n"
		   "   width=%d\n"
		   "   height=%d\n"
		   "   hz=%f\n"
		   "   show=%s\n"
		   "   dir=%s\n"
		   "   extension=%s\n"
		   "   startId=%d\n"
		   "   save=%s\n", 
		   usbDevice, 
		   imageWidth, 
		   imageHeight, 
		   imageRate, 
		   uBool2str(show).c_str(), 
		   targetDirectory.c_str(),
		   extension.c_str(),
		   startId,
		   uBool2str(save).c_str());

	UDirectory::makeDir(targetDirectory);

	rtabmap::CameraVideo cam(usbDevice, imageWidth, imageHeight, false, imageRate);
	if(!cam.init())
	{
		printf("Can't initialize the camera...\n");
		return 1;
	}

	ImagesHandler imgHandler(targetDirectory, extension, startId, show, save);
	UEventsManager::addHandler(&imgHandler);

	cam.start();

	if(show)
	{
		cvWaitKey(0);
	}
	else
	{
		std::cin.ignore();
	}

	return 0;
}
