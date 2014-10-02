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
#include <rtabmap/utilite/UTimer.h>
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Camera.h"
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <fstream>
#include <queue>
#include <opencv2/core/core.hpp>
#include <signal.h>

using namespace rtabmap;

#define GENERATED_GT_NAME "GroundTruth_generated.bmp"

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-console [options] \"path\"\n"
			"  path                            For images, use the directory path. For videos or databases, use full\n "
			"                                  path name\n"
			"Options:\n"
			"  -t #.##                         Time threshold (ms)\n"
			"  -rate #.##                      Acquisition time (seconds)\n"
			"  -rateHz #.##                    Acquisition rate (Hz), for convenience\n"
			"  -repeat #                       Repeat the process on the data set # times (minimum of 1)\n"
			"  -createGT                       Generate a ground truth file\n"
			"  -image_width #                  Force an image width (Default 0: original size used).\n"
			"                                   The height must be also specified if changed.\n"
			"  -image_height #                 Force an image height (Default 0: original size used)\n"
			"                                   The height must be also specified if changed.\n"
			"  -start_at #                     When \"path\" is a directory of images, set this parameter\n"
			"                                   to start processing at image # (default 1)."
			"  -\"parameter name\" \"value\"       Overwrite a specific RTAB-Map's parameter :\n"
			"                                     -SURF/HessianThreshold 150\n"
			"                                   For parameters in table format, add ',' between values :\n"
			"                                     -Kp/RoiRatios 0,0,0.1,0\n"
			"  -default_params                 Show default RTAB-Map's parameters\n"
			"  -debug                          Set Log level to Debug (Default Error)\n"
			"  -info                           Set Log level to Info (Default Error)\n"
			"  -warn                           Set Log level to Warning (Default Error)\n"
			"  -exit_warn                      Set exit level to Warning (Default Fatal)\n"
			"  -exit_error                     Set exit level to Error (Default Fatal)\n"
			"  -v                              Get version of RTAB-Map\n");
	exit(1);
}

// catch ctrl-c
bool g_forever = true;
void sighandler(int sig)
{
	printf("\nSignal %d caught...\n", sig);
	g_forever = false;
}

int main(int argc, char * argv[])
{
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	/*for(int i=0; i<argc; i++)
	{
		printf("argv[%d] = %s\n", i, argv[i]);
	}*/
	const ParametersMap & defaultParameters = Parameters::getDefaultParameters();
	if(argc < 2)
	{
		showUsage();
	}
	else if(argc == 2 && strcmp(argv[1], "-v") == 0)
	{
		printf("%s\n", Rtabmap::getVersion().c_str());
		exit(0);
	}
	else if(argc == 2 && strcmp(argv[1], "-default_params") == 0)
	{
		for(ParametersMap::const_iterator iter = defaultParameters.begin(); iter!=defaultParameters.end(); ++iter)
		{
			printf("%s=%s\n", iter->first.c_str(), iter->second.c_str());
		}
		exit(0);
	}
	printf("\n");

	std::string path;
	float timeThreshold = 0.0;
	float rate = 0.0;
	int loopDataset = 0;
	int repeat = 0;
	bool createGT = false;
	int imageWidth = 0;
	int imageHeight = 0;
	int startAt = 1;
	ParametersMap pm;
	ULogger::Level logLevel = ULogger::kError;
	ULogger::Level exitLevel = ULogger::kFatal;

	for(int i=1; i<argc; ++i)
	{
		if(i == argc-1)
		{
			// The last must be the path
			path = argv[i];
			if(!UDirectory::exists(path.c_str()) && !UFile::exists(path.c_str()))
			{
				printf("Path not valid : %s\n", path.c_str());
				showUsage();
				exit(1);
			}
			break;
		}
		if(strcmp(argv[i], "-t") == 0)
		{
			++i;
			if(i < argc)
			{
				timeThreshold = std::atof(argv[i]);
				if(timeThreshold < 0)
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
		if(strcmp(argv[i], "-rate") == 0)
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
		if(strcmp(argv[i], "-rateHz") == 0)
		{
			++i;
			if(i < argc)
			{
				rate = std::atof(argv[i]);
				if(rate < 0)
				{
					showUsage();
				}
				else if(rate)
				{
					rate = 1/rate;
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-repeat") == 0)
		{
			++i;
			if(i < argc)
			{
				repeat = std::atoi(argv[i]);
				if(repeat < 1)
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
		if(strcmp(argv[i], "-image_width") == 0)
		{
			++i;
			if(i < argc)
			{
				imageWidth = std::atoi(argv[i]);
				if(imageWidth < 0)
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
		if(strcmp(argv[i], "-image_height") == 0)
		{
			++i;
			if(i < argc)
			{
				imageHeight = std::atoi(argv[i]);
				if(imageHeight < 0)
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
		if(strcmp(argv[i], "-start_at") == 0)
		{
			++i;
			if(i < argc)
			{
				startAt = std::atoi(argv[i]);
				if(startAt < 0)
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
		if(strcmp(argv[i], "-createGT") == 0)
		{
			createGT = true;
			continue;
		}
		if(strcmp(argv[i], "-debug") == 0)
		{
			logLevel = ULogger::kDebug;
			continue;
		}
		if(strcmp(argv[i], "-info") == 0)
		{
			logLevel = ULogger::kInfo;
			continue;
		}
		if(strcmp(argv[i], "-warn") == 0)
		{
			logLevel = ULogger::kWarning;
			continue;
		}
		if(strcmp(argv[i], "-exit_warn") == 0)
		{
			exitLevel = ULogger::kWarning;
			continue;
		}
		if(strcmp(argv[i], "-exit_error") == 0)
		{
			exitLevel = ULogger::kError;
			continue;
		}

		// Check for RTAB-Map's parameters
		std::string key = argv[i];
		key = uSplit(key, '-').back();
		if(defaultParameters.find(key) != defaultParameters.end())
		{
			++i;
			if(i < argc)
			{
				std::string value = argv[i];
				if(value.empty())
				{
					showUsage();
				}
				else
				{
					value = uReplaceChar(value, ',', ' ');
				}
				std::pair<ParametersMap::iterator, bool> inserted = pm.insert(ParametersPair(key, value));
				if(inserted.second == false)
				{
					inserted.first->second = value;
				}
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

	if(repeat && createGT)
	{
		printf("Cannot create a Ground truth if repeat is on.\n");
		showUsage();
	}
	else if((imageWidth && imageHeight == 0) ||
			(imageHeight && imageWidth == 0))
	{
		printf("If imageWidth is set, imageHeight must be too.\n");
		showUsage();
	}

	UTimer timer;
	timer.start();
	std::queue<double> iterationMeanTime;

	Camera * camera = 0;
	if(UDirectory::exists(path))
	{
		camera = new CameraImages(path, startAt, false, 1/rate, imageWidth, imageHeight);
	}
	else
	{
		camera = new CameraVideo(path, 1/rate, imageWidth, imageHeight);
	}

	if(!camera || !camera->init())
	{
		printf("Camera init failed, using path \"%s\"\n", path.c_str());
		exit(1);
	}

	std::map<int, int> groundTruth;

	ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setType(ULogger::kTypeFile, rtabmap.getWorkingDir()+"/LogConsole.txt", false);
	//ULogger::setBuffered(true);
	ULogger::setLevel(logLevel);
	ULogger::setExitLevel(exitLevel);

	// Create tasks : memory is deleted
	Rtabmap rtabmap;
	// Disable statistics (we don't need them)
	pm.insert(ParametersPair(Parameters::kRtabmapPublishStats(), "false"));
	// use default working directory
	pm.insert(ParametersPair(Parameters::kRtabmapWorkingDirectory(), Parameters::defaultRtabmapWorkingDirectory()));
	pm.insert(ParametersPair(Parameters::kRGBDEnabled(), "false"));

	rtabmap.init(pm);
	rtabmap.setTimeThreshold(timeThreshold); // in ms

	printf("Avpd init time = %fs\n", timer.ticks());

	// Start thread's task
	int loopClosureId;
	int count = 0;
	int countLoopDetected=0;

	printf("\nParameters : \n");
	printf(" Data set : %s\n", path.c_str());
	printf(" Time threshold = %1.2f ms\n", timeThreshold);
	printf(" Image rate = %1.2f s (%1.2f Hz)\n", rate, 1/rate);
	printf(" Repeating data set = %s\n", repeat?"true":"false");
	printf(" Camera width=%d, height=%d (0 is default)\n", imageWidth, imageHeight);
	printf(" Camera starts at image %d (default 1)\n", startAt);
	if(createGT)
	{
		printf(" Creating the ground truth matrix.\n");
	}
	printf(" INFO: All other parameters are set to defaults\n");
	if(pm.size()>1)
	{
		printf("   Overwritten parameters :\n");
		for(ParametersMap::iterator iter = pm.begin(); iter!=pm.end(); ++iter)
		{
			printf("    %s=%s\n",iter->first.c_str(), iter->second.c_str());
		}
	}
	if(rtabmap.getWM().size() || rtabmap.getSTM().size())
	{
		printf("[Warning] RTAB-Map database is not empty (%s)\n", (rtabmap.getWorkingDir()+Parameters::getDefaultDatabaseName()).c_str());
	}
	printf("\nProcessing images...\n");

	UTimer iterationTimer;
	UTimer rtabmapTimer;
	int imagesProcessed = 0;
	std::list<std::vector<float> > teleopActions;
	while(loopDataset <= repeat && g_forever)
	{
		cv::Mat img = camera->takeImage();
		int i=0;
		double maxIterationTime = 0.0;
		int maxIterationTimeId = 0;
		while(!img.empty() && g_forever)
		{
			++imagesProcessed;
			iterationTimer.start();
			rtabmapTimer.start();
			rtabmap.process(img);
			double rtabmapTime = rtabmapTimer.elapsed();
			loopClosureId = rtabmap.getLoopClosureId();
			if(rtabmap.getLoopClosureId())
			{
				++countLoopDetected;
			}
			img = camera->takeImage();
			if(++count % 100 == 0)
			{
				printf(" count = %d, loop closures = %d, max time (at %d) = %fs\n",
						count, countLoopDetected, maxIterationTimeId, maxIterationTime);
				maxIterationTime = 0.0;
				maxIterationTimeId = 0;
				std::map<int, int> wm = rtabmap.getWeights();
				printf(" WM(%d)=[", (int)wm.size());
				for(std::map<int, int>::iterator iter=wm.begin(); iter!=wm.end();++iter)
				{
					if(iter != wm.begin())
					{
						printf(";");
					}
					printf("%d,%d", iter->first, iter->second);
				}
				printf("]\n");
			}

			// Update generated ground truth matrix
			if(createGT)
			{
				if(loopClosureId > 0)
				{
					groundTruth.insert(std::make_pair(i, loopClosureId-1));
				}
			}

			++i;

			double iterationTime = iterationTimer.ticks();

			if(rtabmapTime > maxIterationTime)
			{
				maxIterationTime = rtabmapTime;
				maxIterationTimeId = count;
			}

			ULogger::flush();

			if(rtabmap.getLoopClosureId())
			{
				printf(" iteration(%d) loop(%d) hyp(%.2f) time=%fs/%fs *\n",
						count, rtabmap.getLoopClosureId(), rtabmap.getLcHypValue(), rtabmapTime, iterationTime);
			}
			else if(rtabmap.getRetrievedId())
			{
				printf(" iteration(%d) high(%d) hyp(%.2f) time=%fs/%fs\n",
						count, rtabmap.getRetrievedId(), rtabmap.getLcHypValue(), rtabmapTime, iterationTime);
			}
			else
			{
				printf(" iteration(%d) time=%fs/%fs\n", count, rtabmapTime, iterationTime);
			}

			if(timeThreshold && rtabmapTime > timeThreshold*100.0f)
			{
				printf(" ERROR,  there is  problem, too much time taken... %fs", rtabmapTime);
				break; // there is  problem, don't continue
			}
		}
		++loopDataset;
		if(loopDataset <= repeat)
		{
			camera->init();
			printf(" Beginning loop %d...\n", loopDataset);
		}
	}
	printf("Processing images completed. Loop closures found = %d\n", countLoopDetected);
	printf(" Total time = %fs\n", timer.ticks());

	if(imagesProcessed && createGT)
	{
		cv::Mat groundTruthMat = cv::Mat::zeros(imagesProcessed, imagesProcessed, CV_8U);

		for(std::map<int, int>::iterator iter = groundTruth.begin(); iter!=groundTruth.end(); ++iter)
		{
			groundTruthMat.at<unsigned char>(iter->first, iter->second) = 255;
		}

		// Generate the ground truth file
		printf("Generate ground truth to file %s, size of %d\n", (rtabmap.getWorkingDir()+GENERATED_GT_NAME).c_str(), groundTruthMat.rows);
		IplImage img = groundTruthMat;
		cvSaveImage((rtabmap.getWorkingDir()+GENERATED_GT_NAME).c_str(), &img);
		printf(" Creating ground truth file = %fs\n", timer.ticks());
	}

	if(camera)
	{
		delete camera;
		camera = 0 ;
	}

	rtabmap.close();

	printf(" Cleanup time = %fs\n", timer.ticks());

	return 0;
}
