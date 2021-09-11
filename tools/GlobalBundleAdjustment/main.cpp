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
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-globalBundleAdjustment database.db\n"
			"\n%s", Parameters::showUsage());
	exit(1);
}

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kError);

	if(argc < 2)
	{
		showUsage();
	}

	for(int i=1; i<argc-1; ++i)
	{
		if(std::strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}
	}
	ParametersMap inputParams = Parameters::parseArguments(argc,  argv);

	std::string dbPath = argv[argc-1];
	if(!UFile::exists(dbPath))
	{
		printf("Database %s doesn't exist!\n", dbPath.c_str());
	}

	// Get parameters
	ParametersMap parameters;
	DBDriver * driver = DBDriver::create();
	if(driver->openConnection(dbPath))
	{
		if(uStrNumCmp(driver->getDatabaseVersion(), "0.17.0")<0)
		{
			printf("Database is too old (%s), we cannot save back optimized poses. "
					"Consider upgrading the database with:\n"
					"rtabmap-reprocess --Db/TargetVersion \"\" \"%s\" \"output.db\"\n",
					driver->getDatabaseVersion().c_str(),
					dbPath.c_str());
			driver->closeConnection(false);
			delete driver;
			return -1;
		}

		parameters = driver->getLastParameters();
		// This will force rtabmap_ros to regenerate the global occupancy grid if there was one
		driver->save2DMap(cv::Mat(), 0, 0, 0);
		driver->saveOptimizedMesh(cv::Mat());
		driver->closeConnection(false);
	}
	else
	{
		UERROR("Cannot open database %s!", dbPath.c_str());
	}
	delete driver;

	for(ParametersMap::iterator iter=inputParams.begin(); iter!=inputParams.end(); ++iter)
	{
		printf("Added custom parameter %s=%s\n",iter->first.c_str(), iter->second.c_str());
	}

	UTimer timer;

	printf("Loading database \"%s\"...\n", dbPath.c_str());
	// Get the global optimized map
	Rtabmap rtabmap;
	uInsert(parameters, inputParams);
	rtabmap.init(parameters, dbPath);
	printf("Loading database \"%s\"... done (%fs).\n", dbPath.c_str(), timer.ticks());

	std::map<int, Signature> nodes;
	std::map<int, Transform> optimizedPoses;
	std::multimap<int, Link> links;
	printf("Optimizing the map...\n");
	rtabmap.getGraph(optimizedPoses, links, true, true, &nodes, true, true, true, true);
	printf("Optimizing the map... done (%fs, poses=%d).\n", timer.ticks(), (int)optimizedPoses.size());

	printf("Global bundle adjustment...\n");
	Optimizer * optimizer = Optimizer::create(Optimizer::kTypeG2O, parameters);
	optimizedPoses = optimizer->optimizeBA(optimizedPoses.lower_bound(1)->first, optimizedPoses, links, nodes, true);
	delete optimizer;
	printf("Global bundle adjustment... done (%fs).\n", timer.ticks());

	if(!optimizedPoses.empty())
	{
		rtabmap.setOptimizedPoses(optimizedPoses, links);
	}
	else
	{
		UERROR("Returned empty poses!");
	}

	rtabmap.close();

	return 0;
}
