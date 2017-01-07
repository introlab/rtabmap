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
#include <rtabmap/utilite/UTimer.h>
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/CameraRGB.h"
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
			"  -rate #.##                      Acquisition time (seconds)\n"
			"  -rateHz #.##                    Acquisition rate (Hz), for convenience\n"
			"  -repeat #                       Repeat the process on the data set # times (minimum of 1)\n"
			"  -createGT                       Generate a ground truth file\n"
			"  -start_at #                     When \"path\" is a directory of images, set this parameter\n"
			"                                   to start processing at image # (default 0).\n"
			"  -skip #                         Skip X images while reading directory (default 0).\n"
			"  -v                              Get version of RTAB-Map\n"
			"  -input \"path\"                 Load previous database if it exists.\n"
			"%s\n",
			rtabmap::Parameters::showUsage());
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

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	ParametersMap pm = Parameters::parseArguments(argc, argv);
	pm.insert(ParametersPair(Parameters::kRtabmapWorkingDirectory(), "."));

	if(argc < 2)
	{
		showUsage();
	}
	else if(argc == 2 && strcmp(argv[1], "-v") == 0)
	{
		printf("%s\n", Parameters::getVersion().c_str());
		exit(0);
	}
	printf("\n");

	std::string path;
	float rate = 0.0;
	int loopDataset = 0;
	int repeat = 0;
	bool createGT = false;
	std::string inputDbPath;
	int startAt = 0;
	int skip = 0;

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
		if(strcmp(argv[i], "-rate") == 0)
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
		if(strcmp(argv[i], "-rateHz") == 0)
		{
			++i;
			if(i < argc)
			{
				rate = uStr2Float(argv[i]);
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
		if (strcmp(argv[i], "-skip") == 0)
		{
			++i;
			if (i < argc)
			{
				skip = std::atoi(argv[i]);
				if (skip < 0)
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
		if(strcmp(argv[i], "-input") == 0)
		{
			++i;
			if(i < argc)
			{
				inputDbPath = argv[i];
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

	if(repeat && createGT)
	{
		printf("Cannot create a Ground truth if repeat is on.\n");
		showUsage();
	}

	UTimer timer;
	timer.start();
	std::queue<double> iterationMeanTime;

	Camera * camera = 0;
	if(UDirectory::exists(path))
	{
		camera = new CameraImages(path, rate>0.0f?1.0f/rate:0.0f);
		((CameraImages*)camera)->setStartIndex(startAt);
	}
	else
	{
		camera = new CameraVideo(path, false, rate>0.0f?1.0f/rate:0.0f);
	}

	if(!camera || !camera->init())
	{
		printf("Camera init failed, using path \"%s\"\n", path.c_str());
		exit(1);
	}

	std::map<int, int> groundTruth;

	// Create tasks
	Rtabmap rtabmap;
	if(inputDbPath.empty())
	{
		inputDbPath = "rtabmapconsole.db";
		if(UFile::erase(inputDbPath) == 0)
		{
			printf("Deleted database \"%s\".\n", inputDbPath.c_str());
		}
	}
	else
	{
		printf("Loading database \"%s\".\n", inputDbPath.c_str());
	}

	// Disable statistics (we don't need them)
	uInsert(pm, ParametersPair(Parameters::kRtabmapPublishStats(), "false"));
	uInsert(pm, ParametersPair(Parameters::kRGBDEnabled(), "false"));

	// Process an empty image to make sure every libraries are loaded.
	ULogger::Level level = ULogger::level();
	ULogger::setLevel(ULogger::kError);
	cv::Mat tmp = cv::Mat::zeros(640,480,CV_8UC1);
	rtabmap.init(pm);
	rtabmap.process(tmp);
	rtabmap.close(false);
	ULogger::setLevel(level);

	rtabmap.init(pm, inputDbPath);

	printf("rtabmap init time = %fs\n", timer.ticks());

	// Start thread's task
	int loopClosureId;
	int count = 0;
	int countLoopDetected=0;

	printf("\nParameters : \n");
	printf(" Data set : %s\n", path.c_str());
	printf(" Time threshold = %1.2f ms\n", rtabmap.getTimeThreshold());
	printf(" Image rate = %1.2f s (%1.2f Hz)\n", rate, 1/rate);
	printf(" Repeating data set = %s\n", repeat?"true":"false");
	printf(" Camera starts at image %d (default 0)\n", startAt);
	printf(" Skip image = %d\n", skip);
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
		printf("[Warning] RTAB-Map database is not empty (%s)\n", inputDbPath.c_str());
	}
	printf("\nProcessing images...\n");

	UTimer iterationTimer;
	UTimer rtabmapTimer;
	int imagesProcessed = 0;
	std::list<std::vector<float> > teleopActions;
	while(loopDataset <= repeat && g_forever)
	{
		SensorData data = camera->takeImage();
		int i=0;
		double maxIterationTime = 0.0;
		int maxIterationTimeId = 0;
		while(!data.imageRaw().empty() && g_forever)
		{
			++imagesProcessed;
			iterationTimer.start();
			rtabmapTimer.start();
			rtabmap.process(data.imageRaw());
			double rtabmapTime = rtabmapTimer.elapsed();
			loopClosureId = rtabmap.getLoopClosureId();
			if(rtabmap.getLoopClosureId())
			{
				++countLoopDetected;
			}
			for(int j=0; j<=skip; ++j)
			{
				data = camera->takeImage();
			}
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
						count, rtabmap.getLoopClosureId(), rtabmap.getLoopClosureValue(), rtabmapTime, iterationTime);
			}
			else if(rtabmap.getHighestHypothesisId())
			{
				printf(" iteration(%d) high(%d) hyp(%.2f) time=%fs/%fs\n",
						count, rtabmap.getHighestHypothesisId(), rtabmap.getHighestHypothesisValue(), rtabmapTime, iterationTime);
			}
			else
			{
				printf(" iteration(%d) time=%fs/%fs\n", count, rtabmapTime, iterationTime);
			}

			if(rtabmap.getTimeThreshold() && rtabmapTime > rtabmap.getTimeThreshold()*100.0f)
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
		printf("Generate ground truth to file %s, size of %d\n", GENERATED_GT_NAME, groundTruthMat.rows);
		cv::imwrite(GENERATED_GT_NAME, groundTruthMat);
		printf(" Creating ground truth file = %fs\n", timer.ticks());
	}

	if(camera)
	{
		delete camera;
		camera = 0 ;
	}

	rtabmap.close();

	printf(" Cleanup time = %fs\n", timer.ticks());

	printf("Database (\"%s\") and log files saved to current directory.\n", inputDbPath.c_str());

	return 0;
}
