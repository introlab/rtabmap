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

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/common.h>
#include <pcl/surface/poisson.h>
#include <stdio.h>
#include <signal.h>

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-detectMoreLoopClosures [options] database.db\n"
			"Options:\n"
			"    -r #          Cluster radius (default 1 m).\n"
			"    -rx #         Cluster radius min (default 0 m).\n"
			"    -a #          Cluster angle (default 30 deg).\n"
			"    -i #          Iterations (default 1).\n"
			"    --intra       Add only intra-session loop closures.\n"
			"    --inter       Add only inter-session loop closures.\n"
			"\n%s", Parameters::showUsage());
	exit(1);
}

// catch ctrl-c
bool g_loopForever = true;
void sighandler(int sig)
{
	printf("\nSignal %d caught...\n", sig);
	g_loopForever = false;
}

class PrintProgressState : public ProgressState
{
public:
	virtual bool callback(const std::string & msg) const
	{
		if(!msg.empty())
			printf("%s \n", msg.c_str());
		return g_loopForever;
	}
};

int main(int argc, char * argv[])
{
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kError);

	if(argc < 2)
	{
		showUsage();
	}

	float clusterRadiusMin = 0.0f;
	float clusterRadiusMax = 1.0f;
	float clusterAngle = CV_PI/6.0f;
	int iterations = 1;
	bool intraSession = false;
	bool interSession = false;
	for(int i=1; i<argc-1; ++i)
	{
		if(std::strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}
		else if(std::strcmp(argv[i], "--intra") == 0)
		{
			intraSession = true;
			if(interSession)
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--inter") == 0)
		{
			interSession = true;
			if(intraSession)
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "-r") == 0)
		{
			++i;
			if(i<argc-1)
			{
				clusterRadiusMax = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "-rx") == 0)
		{
			++i;
			if(i<argc-1)
			{
				clusterRadiusMin = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "-a") == 0)
		{
			++i;
			if(i<argc-1)
			{
				clusterAngle = uStr2Float(argv[i])*CV_PI/180.0f;
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "-i") == 0)
		{
			++i;
			if(i<argc-1)
			{
				iterations = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
	}
	ParametersMap inputParams = Parameters::parseArguments(argc,  argv);

	std::string dbPath = argv[argc-1];
	if(!UFile::exists(dbPath))
	{
		printf("Database %s doesn't exist!\n", dbPath.c_str());
	}

	printf("\nDatabase: %s\n", dbPath.c_str());
	printf("Cluster radius min = %f m\n", clusterRadiusMin);
	printf("Cluster radius max = %f m\n", clusterRadiusMax);
	printf("Cluster angle = %f deg\n", clusterAngle*180.0f/CV_PI);
	if(intraSession)
	{
		printf("Intra-session only\n");
	}
	else if(interSession)
	{
		printf("Inter-session only\n");
	}

	if(!intraSession && !interSession)
	{
		intraSession = true;
		interSession = true;
	}

	// Get parameters
	ParametersMap parameters;
	DBDriver * driver = DBDriver::create();
	if(driver->openConnection(dbPath))
	{
		parameters = driver->getLastParameters();
		driver->closeConnection(false);
	}
	else
	{
		UERROR("Cannot open database %s!", dbPath.c_str());
	}
	delete driver;

	for(ParametersMap::iterator iter=inputParams.begin(); iter!=inputParams.end(); ++iter)
	{
		printf(" Using parameter \"%s=%s\" from arguments\n", iter->first.c_str(), iter->second.c_str());
	}

	// Get the global optimized map
	Rtabmap rtabmap;
	printf("Initialization...\n");
	uInsert(parameters, inputParams);
	rtabmap.init(parameters, dbPath);

	float xMin, yMin, cellSize;
	bool haveOptimizedMap = !rtabmap.getMemory()->load2DMap(xMin, yMin, cellSize).empty();

	PrintProgressState progress;
	printf("Detecting...\n");
	int detected = rtabmap.detectMoreLoopClosures(clusterRadiusMax, clusterAngle, iterations, intraSession, interSession, &progress, clusterRadiusMin);
	if(detected < 0)
	{
		if(!g_loopForever)
		{
			printf("Detection interrupted. Loop closures found so far (if any) are not saved.\n");
		}
		else
		{
			printf("Loop closure detection failed!\n");
		}
	}
	else if(detected > 0 && haveOptimizedMap)
	{
		printf("The database has a global occupancy grid, regenerating one with new optimized graph!\n");
		OccupancyGrid grid(parameters);
		std::map<int, Transform> optimizedPoses = rtabmap.getLocalOptimizedPoses();
		for(std::map<int, Transform>::iterator iter=optimizedPoses.lower_bound(0); iter!=optimizedPoses.end(); ++iter)
		{
			cv::Mat occupancyGrid;
			SensorData data = rtabmap.getMemory()->getNodeData(iter->first, false, false, false, true);
			data.uncompressData();
			grid.addToCache(iter->first, data.gridGroundCellsRaw(), data.gridObstacleCellsRaw(), data.gridEmptyCellsRaw());
		}
		grid.update(optimizedPoses);
		cv::Mat map = grid.getMap(xMin, yMin);
		if(map.empty())
		{
			printf("Could not generate the global occupancy grid!\n");
		}
		else
		{
			rtabmap.getMemory()->save2DMap(map, xMin, yMin, cellSize);
			printf("Save new global occupancy grid!\n");
		}
	}

	rtabmap.close();

	return 0;
}
