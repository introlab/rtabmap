/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include <rtabmap/core/global_map/OccupancyGrid.h>
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

void showUsage(const char * exec)
{
	printf("\nUsage:\n"
			"%s [Options] database.db\n"
			"Options:\n"
			"    --keep_latest  Merge old nodes to newer nodes, thus keeping only latest nodes.\n"
			"    --keep_linked  Keep reduced nodes linked to graph.\n"
			"\n%s", exec, Parameters::showUsage());
	exit(1);
}

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	if(argc < 2)
	{
		showUsage(argv[0]);
	}

	bool keepLatest = false;
	bool keepLinked = false;
	for(int i=1; i<argc-1; ++i)
	{
		if(std::strcmp(argv[i], "--help") == 0)
		{
			showUsage(argv[0]);
		}
		else if(std::strcmp(argv[i], "--keep_latest") == 0)
		{
			keepLatest = uStr2Bool(argv[i]);
		}
		else if(std::strcmp(argv[i], "--keep_linked") == 0)
		{
			keepLinked = uStr2Bool(argv[i]);
		}
	}
	ParametersMap inputParams = Parameters::parseArguments(argc,  argv);

	// Add some optimizations (soft set, can be overriden by arguments)
	inputParams.insert(ParametersPair(Parameters::kMemInitWMWithAllNodes(), "true")); // load the whole map in RAM
	inputParams.insert(ParametersPair(Parameters::kMemLoadVisualLocalFeaturesOnInit(), "false")); // don't need features already loaded in RAM
	inputParams.insert(ParametersPair(Parameters::kKpNNStrategy(), "3")); // don't need flann index

	std::string dbPath = argv[argc-1];
	if(!UFile::exists(dbPath))
	{
		printf("Database %s doesn't exist!\n", dbPath.c_str());
	}

	printf("\nDatabase: %s\n", dbPath.c_str());

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

	Memory memory;
	printf("Initialization...\n");
	UTimer timer;
	uInsert(parameters, inputParams);
	if(!memory.init(dbPath, false, parameters))
	{
		printf("Initialization... failed! Aborting!\n");
		return 1;
	}
	std::set<int> ids = memory.getAllSignatureIds();
	printf("Initialization... done! %ld nodes loaded. (%f sec)\n", ids.size(), timer.ticks());

	if(ids.empty())
	{
		printf("IDs are empty?! Aborting.\n");
		return 1;
	}

	Transform lastLocalizationPose;
	bool hasOptimizedPoses = !memory.loadOptimizedPoses(&lastLocalizationPose).empty();

	float proximityMaxRadius = Parameters::defaultRGBDProximityPathFilteringRadius();
	Parameters::parse(parameters, Parameters::kRGBDProximityPathFilteringRadius(), proximityMaxRadius);
	int totalNodesReduced = 0;
	if(keepLatest)
	{
		// we process older to newer nodes, merging to new nodes
		for(std::set<int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
		{
			int id = *iter;
			int reducedId = memory.reduceNode(id, proximityMaxRadius, keepLinked);
			if(reducedId > 0)
			{
				printf("Reduced node %d to node %d!\n", id, reducedId);
				++totalNodesReduced;
			}
		}
	}
	else
	{
		// we process newer to older nodes, merging to old nodes
		for(std::set<int>::reverse_iterator iter = ids.rbegin(); iter!=ids.rend(); ++iter)
		{
			int id = *iter;
			int reducedId = memory.reduceNode(id, proximityMaxRadius, keepLinked);
			if(reducedId > 0)
			{
				printf("Reduced node %d to node %d!\n", id, reducedId);
				++totalNodesReduced;
			}
		}
	}
	printf("Reduced a total of %d nodes out of %ld nodes\n", totalNodesReduced, ids.size());

	// Clear the optimized poses, this will force rtabmap to re-optimize the graph on initialization
	if(hasOptimizedPoses)
	{
		printf("Note that there were optimized poses/map saved in the database, "
			   "clearing them so that they are regenerated next time rtabmap loads that map.\n");
		memory.saveOptimizedPoses(std::map<int, Transform>(), lastLocalizationPose);
	}
	// This will force rtabmap_ros to regenerate the global occupancy grid if there was one
	memory.save2DMap(cv::Mat(), 0, 0, 0);
	memory.saveOptimizedMesh(cv::Mat());

	memory.close(true);

	return 0;
}
