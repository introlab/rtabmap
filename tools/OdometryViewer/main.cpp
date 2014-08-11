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
#include <rtabmap/gui/OdometryViewer.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/DBReader.h>
#include <rtabmap/core/VWDictionary.h>
#include <QtGui/QApplication>

void showUsage()
{
	printf("\nUsage:\n"
			"odometryViewer [options]\n"
			"Options:\n"
			"  -o #                      Odometry type (default 0): 0=SURF, 1=SIFT, 2=ORB, 3=FAST/FREAK, 4=FAST/BRIEF\n"
			"  -nn #                     Nearest neighbor strategy (default 1): kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4\n"
			"  -nndr #                   Nearest neighbor distance ratio (default 0.7)\n"
			"  -icp                      Use ICP odometry\n"
			"\n"
			"  -hz #.#                   Camera rate (default 0, 0 means as fast as the camera can)\n"
			"  -db \"input.db\"          Use database instead of camera (recorded with rtabmap-dataRecorder)\n"
			"  -clouds #                 Maximum clouds shown (default 10, zero means inf)\n"
			"  -sec #.#                  Delay (seconds) before reading the database (if set)\n"
			"\n"
			"  -in #.#                   Inliers maximum distance, features/ICP (default 0.005 m)\n"
			"  -max #                    Max features used for matching (default 0=inf)\n"
			"  -min #                    Minimum inliers to accept the transform (default 20)\n"
			"  -depth #.#                Maximum features depth (default 5.0 m)\n"
			"  -i #                      RANSAC/ICP iterations (default 100)\n"
			"  -r #.#                    Words ratio (default 0.5)\n"
			"  -lu #                     Linear update (default 0.0 m)\n"
			"  -au #                     Angular update (default 0.0 radian)\n"
			"  -reset #                  Reset countdown (default 0 = disabled)\n"
			"  -gpu                      Use GPU\n"
			"  -lh #                     Local history (default 0)\n"
			"\n"
			"  -brief_bytes #        BRIEF bytes (default 32)\n"
			"  -fast_thr #           FAST threshold (default 30)\n"
			"\n"
			"  -d #                      ICP decimation (default 4)\n"
			"  -v #                      ICP voxel size (default 0.005)\n"
			"  -s #                      ICP samples (default 0, not used if voxel is set.)\n"
			"  -f #.#                    ICP fitness (default 0.01)\n"
			"  -p2p                      ICP point to point (default point to plane)"
			"\n"
			"  -debug                    Log debug messages\n"
			"\n"
			"Examples:\n"
			"  odometryViewer -odom 0 -lh 5000                      SURF example\n"
			"  odometryViewer -odom 1 -lh 10000                     SIFT example\n"
			"  odometryViewer -odom 4 -nn 2 -lh 1000                FAST/BRIEF example\n"
			"  odometryViewer -odom 3 -nn 2 -lh 1000                FAST/FREAK example\n"
			"  odometryViewer -icp -in 0.05 -i 30                   ICP example\n");
	exit(1);
}

int main (int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	// parse arguments
	float rate = 0.0;
	std::string inputDatabase;
	int odomType = 0;
	bool icp = false;
	int nnType =1;
	float nndr = 0.7f;
	float distance = 0.005;
	int maxWords = 0;
	int minInliers = 20;
	float wordsRatio = 0.5;
	float maxDepth = 5.0f;
	int iterations = 100;
	float linearUpdate = 0.0f;
	float angularUpdate = 0.0f;
	int resetCountdown = 0;
	int decimation = 4;
	float voxel = 0.005;
	int samples = 10000;
	float fitness = 0.01f;
	int maxClouds = 10;
	int briefBytes = 32;
	int fastThr = 30;
	float sec = 0.0f;
	bool gpu = false;
	int localHistory = 0;
	bool p2p = false;

	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "-o") == 0)
		{
			++i;
			if(i < argc)
			{
				odomType = std::atoi(argv[i]);
				if(odomType < 0 || odomType > 4)
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
				nndr = std::atof(argv[i]);
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
				rate = std::atof(argv[i]);
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
				sec = std::atof(argv[i]);
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
				distance = std::atof(argv[i]);
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
				maxDepth = std::atof(argv[i]);
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
		if(strcmp(argv[i], "-r") == 0)
		{
			++i;
			if(i < argc)
			{
				wordsRatio = std::atof(argv[i]);
				if(wordsRatio < 0.0f || wordsRatio > 1.0f)
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
		if(strcmp(argv[i], "-lu") == 0)
		{
			++i;
			if(i < argc)
			{
				linearUpdate = std::atof(argv[i]);
				if(linearUpdate < 0.0f)
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
		if(strcmp(argv[i], "-au") == 0)
		{
			++i;
			if(i < argc)
			{
				angularUpdate = std::atof(argv[i]);
				if(angularUpdate < 0.0f)
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
				voxel = std::atof(argv[i]);
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
		if(strcmp(argv[i], "-f") == 0)
		{
			++i;
			if(i < argc)
			{
				fitness = std::atof(argv[i]);
				if(fitness < 0.0f)
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
				if(fitness <= 0)
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
		if(strcmp(argv[i], "-p2p") == 0)
		{
			p2p = true;
			continue;
		}
		if(strcmp(argv[i], "-debug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
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

	if(icp)
	{
		odomName= "ICP";
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
	UINFO("Linear update =           %f", linearUpdate);
	UINFO("Angular update =          %f", angularUpdate);
	UINFO("Reset odometry coutdown = %d", resetCountdown);
	UINFO("Local history =           %d", localHistory);

	QApplication app(argc, argv);

	rtabmap::Odometry * odom = 0;

	rtabmap::ParametersMap parameters;

	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomMaxDepth(), uNumber2Str(maxDepth)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomLinearUpdate(), uNumber2Str(linearUpdate)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomAngularUpdate(), uNumber2Str(angularUpdate)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomResetCountdown(), uNumber2Str(resetCountdown)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomLocalHistory(), uNumber2Str(localHistory)));

	if(!icp)
	{
		UINFO("Nearest neighbor =         %s", nnName.c_str());
		UINFO("Nearest neighbor ratio =  %f", nndr);
		UINFO("Max features =            %d", maxWords);
		UINFO("Min inliers =             %d", minInliers);
		UINFO("Words ratio =             %f", wordsRatio);
		UINFO("Inlier maximum correspondences distance = %f", distance);
		UINFO("RANSAC iterations =       %d", iterations);
		UINFO("GPU =                     %s", gpu?"true":"false");
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomMaxWords(), uNumber2Str(maxWords)));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomWordsRatio(), uNumber2Str(wordsRatio)));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomInlierDistance(), uNumber2Str(distance)));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomMinInliers(), uNumber2Str(minInliers)));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomIterations(), uNumber2Str(iterations)));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomNearestNeighbor(), uNumber2Str(nnType)));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomNNDR(), uNumber2Str(nndr)));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomType(), uNumber2Str(odomType)));
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
		if(odomType == 4)
		{
			UINFO("BRIEF bytes =             %d", briefBytes);
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kBRIEFBytes(), uNumber2Str(briefBytes)));
		}

		odom = new rtabmap::OdometryBOW(parameters);
	}
	else // ICP
	{
		UINFO("ICP maximum correspondences distance = %f", distance);
		UINFO("ICP iterations =          %d", iterations);
		UINFO("Cloud decimation =        %d", decimation);
		UINFO("Cloud voxel size =        %f", voxel);
		UINFO("Cloud samples =           %d", samples);
		UINFO("Cloud fitness =           %f", fitness);
		UINFO("Cloud point to plane =    %s", p2p?"false":"true");

		odom = new rtabmap::OdometryICP(decimation, voxel, samples, distance, iterations, fitness, !p2p);
	}
	rtabmap::OdometryThread odomThread(odom);
	rtabmap::OdometryViewer odomViewer(maxClouds, 2, 0.0, 50);
	UEventsManager::addHandler(&odomThread);
	UEventsManager::addHandler(&odomViewer);

	odomViewer.setCameraFree();
	odomViewer.setGridShown(true);

	odomViewer.setWindowTitle("Odometry viewer");
	odomViewer.setMinimumWidth(800);
	odomViewer.setMinimumHeight(500);
	odomViewer.showNormal();

	app.processEvents();

	if(inputDatabase.size())
	{
		rtabmap::DBReader camera(inputDatabase, rate, true, sec);
		if(camera.init())
		{
			odomThread.start();
			camera.start();

			app.exec();

			camera.kill();
			odomThread.join(true);
		}
	}
	else
	{
		rtabmap::CameraThread camera(new rtabmap::CameraOpenni("", rate, rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0)));
		if(camera.init())
		{
			odomThread.start();
			camera.start();

			app.exec();

			camera.kill();
			odomThread.join(true);
		}
	}

	return 0;
}
