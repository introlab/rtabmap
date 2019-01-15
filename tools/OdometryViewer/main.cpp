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
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryThread.h>
#include <rtabmap/gui/OdometryViewer.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/CameraStereo.h>
#include <rtabmap/core/DBReader.h>
#include <rtabmap/core/VWDictionary.h>
#include <QApplication>
#include <QPushButton>
#include <pcl/console/print.h>

void showUsage()
{
	printf("\nUsage:\n"
			"odometryViewer [options]\n"
			"Options:\n"
			"  -driver #                 Driver number to use: 0=OpenNI-PCL, 1=OpenNI2, 2=Freenect, 3=OpenNI-CV, 4=OpenNI-CV-ASUS, 5=Freenect2, 6=dc1394, 7=FlyCapture2\n"
			"  -hz #.#                   Camera rate (default 0, 0 means as fast as the camera can)\n"
			"  -db \"input.db\"          Use database instead of camera (recorded with rtabmap-dataRecorder)\n"
			"  -clouds #                 Maximum clouds shown (default 10, zero means inf)\n"
			"  -sec #.#                  Delay (seconds) before reading the database (if set)\n"
			"%s\n",
			rtabmap::Parameters::showUsage());
	exit(1);
}

int main (int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	// parse arguments
	float rate = 0.0;
	std::string inputDatabase;
	int driver = 0;
	int maxClouds = 10;
	float sec = 0.0f;

	for(int i=1; i<argc; ++i)
	{
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
		if(strcmp(argv[i], "-hz") == 0)
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
		if(strcmp(argv[i], "-db") == 0)
		{
			++i;
			if(i < argc)
			{
				inputDatabase = argv[i];
				if(UFile::getExtension(inputDatabase).compare("db") != 0)
				{
					printf("Database path (%s) should end with \"db\" \n", inputDatabase.c_str());
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-clouds") == 0)
		{
			++i;
			if(i < argc)
			{
				maxClouds = std::atoi(argv[i]);
				if(maxClouds < 0)
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
		if(strcmp(argv[i], "-sec") == 0)
		{
			++i;
			if(i < argc)
			{
				sec = uStr2Float(argv[i]);
				if(sec < 0.0f)
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
		if(strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}
	}

	if(inputDatabase.size())
	{
		UINFO("Using database input \"%s\"", inputDatabase.c_str());
	}
	else
	{
		UINFO("Using OpenNI camera");
	}

	UINFO("Camera rate =             %f Hz", rate);
	UINFO("Maximum clouds shown =    %d", maxClouds);
	UINFO("Delay =                   %f s", sec);

	rtabmap::ParametersMap parameters = rtabmap::Parameters::parseArguments(argc, argv);
	for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		UINFO(" Param \"%s\"=\"%s\"", iter->first.c_str(), iter->second.c_str());
	}

	bool icp = false;
	int regStrategy = rtabmap::Parameters::defaultRegStrategy();
	rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kRegStrategy(), regStrategy);
	int decimation = 8;
	float maxDepth = 4.0f;
	float voxelSize = rtabmap::Parameters::defaultIcpVoxelSize();
	int normalsK = 0;
	float normalsRadius = 0.0f;
	if(regStrategy == 1 || regStrategy == 2)
	{
		// icp requires scans
		icp = true;

		rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kIcpDownsamplingStep(), decimation);
		rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kIcpVoxelSize(), voxelSize);

		bool pointToPlane = rtabmap::Parameters::defaultIcpPointToPlane();
		rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kIcpPointToPlane(), pointToPlane);
		if(pointToPlane)
		{
			normalsK = rtabmap::Parameters::defaultIcpPointToPlaneK();
			rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kIcpPointToPlaneK(), normalsK);
			normalsRadius = rtabmap::Parameters::defaultIcpPointToPlaneRadius();
			rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kIcpPointToPlaneRadius(), normalsRadius);
		}

		uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kIcpDownsamplingStep(), "1"));
		uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kIcpVoxelSize(), "0"));
	}

	QApplication app(argc, argv);

	rtabmap::Odometry * odom = rtabmap::Odometry::create(parameters);

	rtabmap::OdometryThread odomThread(odom);
	rtabmap::OdometryViewer odomViewer(maxClouds, 2, 0.0, 50);
	UEventsManager::addHandler(&odomThread);
	UEventsManager::addHandler(&odomViewer);

	odomViewer.setWindowTitle("Odometry view");
	odomViewer.resize(1280, 480+QPushButton().minimumHeight());

	rtabmap::Camera * camera = 0;
	rtabmap::Transform t=rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0);

	if(inputDatabase.size())
	{
		camera =  new rtabmap::DBReader(inputDatabase, rate, true);
	}
	else if(driver == 0)
	{
		camera = new rtabmap::CameraOpenni("", rate, t);
	}
	else if(driver == 1)
	{
		if(!rtabmap::CameraOpenNI2::available())
		{
			UERROR("Not built with OpenNI2 support...");
			exit(-1);
		}
		camera = new rtabmap::CameraOpenNI2("", rtabmap::CameraOpenNI2::kTypeColorDepth, rate, t);
	}
	else if(driver == 2)
	{
		if(!rtabmap::CameraFreenect::available())
		{
			UERROR("Not built with Freenect support...");
			exit(-1);
		}
		camera = new rtabmap::CameraFreenect(0, rtabmap::CameraFreenect::kTypeColorDepth, rate, t);
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
		camera = new rtabmap::CameraFreenect2(0, rtabmap::CameraFreenect2::kTypeColor2DepthSD, rate, t);
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

	//pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);

	if(camera->init())
	{
		if(camera->isCalibrated())
		{
			rtabmap::CameraThread cameraThread(camera, parameters);

			cameraThread.setScanParameters(icp, decimation<1?1:decimation, 0, maxDepth, voxelSize, normalsK, normalsRadius);

			odomThread.start();
			cameraThread.start();

			odomViewer.exec();

			cameraThread.join(true);
			odomThread.join(true);
		}
		else
		{
			printf("The camera is not calibrated! You should calibrate the camera first.\n");
			delete camera;
		}
	}
	else
	{
		printf("Failed to initialize the camera! Please select another driver (see \"--help\").\n");
		delete camera;
	}

	return 0;
}
