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
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
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
			"  -o #                      Odometry type (default 6): 0=SURF, 1=SIFT, 2=ORB, 3=FAST/FREAK, 4=FAST/BRIEF, 5=GFTT/FREAK, 6=GFTT/BRIEF, 7=BRISK\n"
			"  -nn #                     Nearest neighbor strategy (default 3): kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4\n"
			"  -nndr #                   Nearest neighbor distance ratio (default 0.7)\n"
			"  -flow                     Use optical flow odometry.\n"
			"  -icp                      Use ICP odometry\n"
			"  -mono                     Use Mono odometry\n"
			"\n"
			"  -hz #.#                   Camera rate (default 0, 0 means as fast as the camera can)\n"
			"  -db \"input.db\"          Use database instead of camera (recorded with rtabmap-dataRecorder)\n"
			"  -clouds #                 Maximum clouds shown (default 10, zero means inf)\n"
			"  -sec #.#                  Delay (seconds) before reading the database (if set)\n"
			"\n"
			"  -in #.#                   Inliers maximum distance, features/ICP (default 0.01 m)\n"
			"  -max #                    Max features used for matching (default 0=inf)\n"
			"  -min #                    Minimum inliers to accept the transform (default 20)\n"
			"  -depth #.#                Maximum features depth (default 5.0 m)\n"
			"  -i #                      RANSAC/ICP iterations (default 30)\n"
			"  -reset #                  Reset countdown (default 0 = disabled)\n"
			"  -gpu                      Use GPU\n"
			"  -lh #                     Local history (default 1000)\n"
			"\n"
			"  -brief_bytes #        BRIEF bytes (default 32)\n"
			"  -fast_thr #           FAST threshold (default 30)\n"
			"\n"
			"  -d #                      ICP decimation (default 4)\n"
			"  -v #                      ICP voxel size (default 0.005)\n"
			"  -s #                      ICP samples (default 0, not used if voxel is set.)\n"
			"  -cr #.#                   ICP correspondence ratio (default 0.7)\n"
			"  -p2p                      ICP point to point (default point to plane)"
			"\n"
			"  -debug                    Log debug messages\n"
			"\n"
			"Examples:\n"
			"  odometryViewer -odom 0 -lh 5000                      SURF example\n"
			"  odometryViewer -odom 1 -lh 10000                     SIFT example\n"
			"  odometryViewer -odom 4 -nn 2 -lh 1000                FAST/BRIEF example\n"
			"  odometryViewer -odom 3 -nn 2 -lh 1000                FAST/FREAK example\n"
			"  odometryViewer -icp -in 0.05 -i 30                   ICP example\n"
			"  odometryViewer -flow -in 0.02                        Optical flow example\n");
	exit(1);
}

int main (int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);
	ULogger::setPrintTime(false);
	ULogger::setPrintWhere(false);

	// parse arguments
	float rate = 0.0;
	std::string inputDatabase;
	int driver = 0;
	int odomType = rtabmap::Parameters::defaultOdomFeatureType();
	bool icp = false;
	bool flow = false;
	bool mono = false;
	int nnType = rtabmap::Parameters::defaultOdomBowNNType();
	float nndr = rtabmap::Parameters::defaultOdomBowNNDR();
	float distance = rtabmap::Parameters::defaultOdomInlierDistance();
	int maxWords = rtabmap::Parameters::defaultOdomMaxFeatures();
	int minInliers = rtabmap::Parameters::defaultOdomMinInliers();
	float maxDepth = rtabmap::Parameters::defaultOdomMaxDepth();
	int iterations = rtabmap::Parameters::defaultOdomIterations();
	int resetCountdown = rtabmap::Parameters::defaultOdomResetCountdown();
	int decimation = 4;
	float voxel = 0.005;
	int samples = 10000;
	float ratio = 0.7f;
	int maxClouds = 10;
	int briefBytes = rtabmap::Parameters::defaultBRIEFBytes();
	int fastThr = rtabmap::Parameters::defaultFASTThreshold();
	float sec = 0.0f;
	bool gpu = false;
	int localHistory = rtabmap::Parameters::defaultOdomBowLocalHistorySize();
	bool p2p = false;

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
		if(strcmp(argv[i], "-o") == 0)
		{
			++i;
			if(i < argc)
			{
				odomType = std::atoi(argv[i]);
				if(odomType < 0 || odomType > 6)
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
		if(strcmp(argv[i], "-nn") == 0)
		{
			++i;
			if(i < argc)
			{
				nnType = std::atoi(argv[i]);
				if(nnType < 0 || nnType > 4)
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
		if(strcmp(argv[i], "-nndr") == 0)
		{
			++i;
			if(i < argc)
			{
				nndr = uStr2Float(argv[i]);
				if(nndr < 0.0f)
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
		if(strcmp(argv[i], "-in") == 0)
		{
			++i;
			if(i < argc)
			{
				distance = uStr2Float(argv[i]);
				if(distance <= 0)
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
		if(strcmp(argv[i], "-max") == 0)
		{
			++i;
			if(i < argc)
			{
				maxWords = std::atoi(argv[i]);
				if(maxWords < 0)
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
		if(strcmp(argv[i], "-min") == 0)
		{
			++i;
			if(i < argc)
			{
				minInliers = std::atoi(argv[i]);
				if(minInliers < 0)
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
		if(strcmp(argv[i], "-depth") == 0)
		{
			++i;
			if(i < argc)
			{
				maxDepth = uStr2Float(argv[i]);
				if(maxDepth < 0)
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
		if(strcmp(argv[i], "-i") == 0)
		{
			++i;
			if(i < argc)
			{
				iterations = std::atoi(argv[i]);
				if(iterations <= 0)
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
		if(strcmp(argv[i], "-reset") == 0)
		{
			++i;
			if(i < argc)
			{
				resetCountdown = std::atoi(argv[i]);
				if(resetCountdown < 0)
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
		if(strcmp(argv[i], "-d") == 0)
		{
			++i;
			if(i < argc)
			{
				decimation = std::atoi(argv[i]);
				if(decimation < 1)
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
		if(strcmp(argv[i], "-v") == 0)
		{
			++i;
			if(i < argc)
			{
				voxel = uStr2Float(argv[i]);
				if(voxel < 0)
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
		if(strcmp(argv[i], "-s") == 0)
		{
			++i;
			if(i < argc)
			{
				samples = std::atoi(argv[i]);
				if(samples < 0)
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
		if(strcmp(argv[i], "-cr") == 0)
		{
			++i;
			if(i < argc)
			{
				ratio = uStr2Float(argv[i]);
				if(ratio < 0.0f)
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
		if(strcmp(argv[i], "-gpu") == 0)
		{
			gpu = true;
			continue;
		}
		if(strcmp(argv[i], "-lh") == 0)
		{
			++i;
			if(i < argc)
			{
				localHistory = std::atoi(argv[i]);
				if(localHistory < 0)
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
		if(strcmp(argv[i], "-brief_bytes") == 0)
		{
			++i;
			if(i < argc)
			{
				briefBytes = std::atoi(argv[i]);
				if(briefBytes < 1)
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
		if(strcmp(argv[i], "-fast_thr") == 0)
		{
			++i;
			if(i < argc)
			{
				fastThr = std::atoi(argv[i]);
				if(fastThr < 1)
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
		if(strcmp(argv[i], "-icp") == 0)
		{
			icp = true;
			continue;
		}
		if(strcmp(argv[i], "-flow") == 0)
		{
			flow = true;
			continue;
		}
		if(strcmp(argv[i], "-mono") == 0)
		{
			mono = true;
			continue;
		}
		if(strcmp(argv[i], "-p2p") == 0)
		{
			p2p = true;
			continue;
		}
		if(strcmp(argv[i], "-debug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
			ULogger::setPrintTime(true);
			ULogger::setPrintWhere(true);
			continue;
		}

		printf("Unrecognized option : %s\n", argv[i]);
		showUsage();
	}

	if(odomType > 1 && nnType == rtabmap::VWDictionary::kNNFlannKdTree)
	{
		UERROR("You set \"-o %d\" (binary descriptor), you must use \"-nn 2\" (any \"-nn\" other than kNNFlannKdTree)", odomType);
		showUsage();
	}
	else if(odomType <= 1 && nnType == rtabmap::VWDictionary::kNNFlannLSH)
	{
		UERROR("You set \"-o %d\" (float descriptor), you must use \"-nn 1\" (any \"-nn\" other than kNNFlannLSH)", odomType);
		showUsage();
	}

	if(inputDatabase.size())
	{
		UINFO("Using database input \"%s\"", inputDatabase.c_str());
	}
	else
	{
		UINFO("Using OpenNI camera");
	}

	std::string odomName;
	if(odomType == 0)
	{
		odomName = "SURF";
	}
	else if(odomType == 1)
	{
		odomName = "SIFT";
	}
	else if(odomType == 2)
	{
		odomName = "ORB";
	}
	else if(odomType == 3)
	{
		odomName = "FAST+FREAK";
	}
	else if(odomType == 4)
	{
		odomName = "FAST+BRIEF";
	}
	else if(odomType == 5)
	{
		odomName = "GFTT+FREAK";
	}
	else if(odomType == 6)
	{
		odomName = "GFTT+BRIEF";
	}
	else if(odomType == 7)
	{
		odomName = "BRISK";
	}

	if(icp)
	{
		odomName= "ICP";
	}

	if(flow)
	{
		odomName= "Optical Flow";
	}

	std::string nnName;
	if(nnType == 0)
	{
		nnName = "kNNFlannLinear";
	}
	else if(nnType == 1)
	{
		nnName = "kNNFlannKdTree";
	}
	else if(nnType == 2)
	{
		nnName= "kNNFlannLSH";
	}
	else if(nnType == 3)
	{
		nnName= "kNNBruteForce";
	}
	else if(nnType == 4)
	{
		nnName= "kNNBruteForceGPU";
	}

	UINFO("Odometry used =           %s", odomName.c_str());
	UINFO("Camera rate =             %f Hz", rate);
	UINFO("Maximum clouds shown =    %d", maxClouds);
	UINFO("Delay =                   %f s", sec);
	UINFO("Max depth =               %f", maxDepth);
	UINFO("Reset odometry coutdown = %d", resetCountdown);

	QApplication app(argc, argv);

	rtabmap::Odometry * odom = 0;

	rtabmap::ParametersMap parameters;

	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomMaxDepth(), uNumber2Str(maxDepth)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomResetCountdown(), uNumber2Str(resetCountdown)));

	if(!icp)
	{
		UINFO("Min inliers =             %d", minInliers);
		UINFO("Inlier maximum correspondences distance = %f", distance);
		UINFO("RANSAC iterations =       %d", iterations);
		UINFO("Max features =            %d", maxWords);
		UINFO("GPU =                     %s", gpu?"true":"false");
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomInlierDistance(), uNumber2Str(distance)));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomMinInliers(), uNumber2Str(minInliers)));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomIterations(), uNumber2Str(iterations)));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomMaxFeatures(), uNumber2Str(maxWords)));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomFeatureType(), uNumber2Str(odomType)));
		if(odomType == 0)
		{
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kSURFGpuVersion(), uBool2Str(gpu)));
		}
		if(odomType == 2)
		{
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kORBGpu(), uBool2Str(gpu)));
		}
		if(odomType == 3 || odomType == 4)
		{
			UINFO("FAST threshold =          %d", fastThr);
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kFASTThreshold(), uNumber2Str(fastThr)));
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kFASTGpu(), uBool2Str(gpu)));
		}
		if(odomType == 4 || odomType == 6)
		{
			UINFO("BRIEF bytes =             %d", briefBytes);
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kBRIEFBytes(), uNumber2Str(briefBytes)));
		}

		if(flow)
		{
			// Optical Flow
			odom = new rtabmap::OdometryOpticalFlow(parameters);
		}
		else
		{
			//BOW
			UINFO("Nearest neighbor =         %s", nnName.c_str());
			UINFO("Nearest neighbor ratio =  %f", nndr);
			UINFO("Local history =           %d", localHistory);
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomBowNNType(), uNumber2Str(nnType)));
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomBowNNDR(), uNumber2Str(nndr)));
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomBowLocalHistorySize(), uNumber2Str(localHistory)));

			if(mono)
			{
				parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomPnPFlags(), uNumber2Str(0))); //CV_ITERATIVE
				parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomPnPReprojError(), "4.0"));
				parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomIterations(), "100"));
				odom = new rtabmap::OdometryMono(parameters);
			}
			else
			{
				odom = new rtabmap::OdometryBOW(parameters);
			}
		}
	}
	else if(icp) // ICP
	{
		UINFO("ICP maximum correspondences distance = %f", distance);
		UINFO("ICP iterations =          %d", iterations);
		UINFO("Cloud decimation =        %d", decimation);
		UINFO("Cloud voxel size =        %f", voxel);
		UINFO("Cloud samples =           %d", samples);
		UINFO("Cloud correspondence ratio = %f", ratio);
		UINFO("Cloud point to plane =    %s", p2p?"false":"true");

		odom = new rtabmap::OdometryICP(decimation, voxel, samples, distance, iterations, ratio, !p2p);
	}

	rtabmap::OdometryThread odomThread(odom);
	rtabmap::OdometryViewer odomViewer(maxClouds, 2, 0.0, 50);
	UEventsManager::addHandler(&odomThread);
	UEventsManager::addHandler(&odomViewer);

	odomViewer.setWindowTitle("Odometry view");
	odomViewer.resize(1280, 480+QPushButton().minimumHeight());

	if(inputDatabase.size())
	{
		rtabmap::DBReader camera(inputDatabase, rate, true);
		if(camera.init())
		{
			odomThread.start();

			if(sec > 0)
			{
				uSleep(sec*1000);
			}

			camera.start();

			app.exec();

			camera.join(true);
			odomThread.join(true);
		}
	}
	else
	{
		rtabmap::Camera * camera = 0;
		rtabmap::Transform t=rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0);
		if(driver == 0)
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
			camera = new rtabmap::CameraOpenNI2("", rate, t);
		}
		else if(driver == 2)
		{
			if(!rtabmap::CameraFreenect::available())
			{
				UERROR("Not built with Freenect support...");
				exit(-1);
			}
			camera = new rtabmap::CameraFreenect(0, rate, t);
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
			camera = new rtabmap::CameraFreenect2(0, rtabmap::CameraFreenect2::kTypeRGBDepthSD, rate, t);
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
				rtabmap::CameraThread cameraThread(camera);

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
	}

	return 0;
}
