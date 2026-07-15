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
#include <rtabmap/core/Optimizer.h>
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
#ifdef RTABMAP_PYTHON
#include <rtabmap/core/PythonInterface.h>
#endif
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
			"    --pre_cleanup  Remove all user loop closures linking nodes closer than %s nodes in the graph before reducing the graph.\n"
			"    --radius #.#   Maximum loop closure distance that can be merged. Default is 1 m. Should be > 0.\n"
			"    --remove_orphan_nodes  Remove all orphan nodes created by graph reduction from WM and LTM. This assumes that the original global graph connected all nodes in WM and LTM, otherwise it is skipped.\n"
			"    --remove_all_orphan_nodes Remove all orphan nodes created or not by graph reduction from WM and LTM. Warning: this could remove completly unconnected graphes from WM and LTM. Usage of --remove_orphan_nodes is safer. Backup you database before trying this.\n"
			"    --udebug/--uinfo/--warn can also be used to change verbosity.\n"
			"\n", exec, Parameters::kMemSTMSize().c_str());
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
	float radius = 1.0f;
	bool preCleanup = false;
	bool removeOrphanNodes = false;
	bool removeAllOrphanNodes = false;
	for(int i=1; i<argc; ++i)
	{
		if(std::strcmp(argv[i], "--help") == 0)
		{
			showUsage(argv[0]);
		}
		else if(std::strcmp(argv[i], "--keep_latest") == 0)
		{
			keepLatest = true;
		}
		else if(std::strcmp(argv[i], "--keep_linked") == 0)
		{
			keepLinked = true;
		}
		else if(std::strcmp(argv[i], "--pre_cleanup") == 0)
		{
			preCleanup = true;
		}
		else if(std::strcmp(argv[i], "--remove_orphan_nodes") == 0)
		{
			removeOrphanNodes = true;
		}
		else if(std::strcmp(argv[i], "--remove_all_orphan_nodes") == 0)
		{
			removeAllOrphanNodes = true;
		}
		else if(std::strcmp(argv[i], "--radius") == 0)
		{
			++i;
			if(i < argc-1)
			{
				radius = uStr2Float(argv[i]);
				if(radius <= 0.0f)
				{
					printf("--radius should be > 0, parsed %f\n", radius);
					showUsage(argv[0]);
				}
			}
			else {
				showUsage(argv[0]);
			}
		}
	}
	printf("Parameters:\n");
	printf("  radius = %f m\n", radius);
	printf("  keep_latest = %s\n", keepLatest?"true":"false");
	printf("  keep_linked = %s\n", keepLinked?"true":"false");
	printf("  pre_cleanup = %s\n", preCleanup?"true":"false");
	printf("  remove_orphan_nodes = %s\n", removeOrphanNodes?"true":"false");
	printf("  remove_all_orphan_nodes = %s\n", removeAllOrphanNodes?"true":"false");
	removeOrphanNodes = removeAllOrphanNodes || removeOrphanNodes;

#ifdef RTABMAP_PYTHON
	rtabmap::PythonInterface pythonInterface;
#endif
	
	// Just parse logging options
	Parameters::parseArguments(argc, argv);

	// Add some optimizations
	ParametersMap inputParams;
	inputParams.insert(ParametersPair(Parameters::kMemInitWMWithAllNodes(), "true")); // load the whole map in RAM
	inputParams.insert(ParametersPair(Parameters::kMemLoadVisualLocalFeaturesOnInit(), "false")); // don't need features already loaded in RAM

	std::string dbPath = argv[argc-1];
	if(!UFile::exists(dbPath))
	{
		printf("Database %s doesn't exist!\n", dbPath.c_str());
		return 1;
	}

	printf("Database: %s\n", dbPath.c_str());

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

	Memory memory;
	printf("Initialization...\n");
	UTimer timer;
	ParametersMap originalParameters = parameters;
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

	bool isWholeGraphConnected = false;
	std::shared_ptr<Optimizer> optimizer(Optimizer::create(parameters));

	std::map<int, Transform> poses;
	std::multimap<int, Link> constraints;
	memory.getMetricConstraints(ids, poses, constraints, false, true);
	if(!poses.empty())
	{
		std::map<int, Transform> posesOut;
		std::multimap<int, Link> linksOut;
		optimizer->getConnectedGraph(
			keepLatest?poses.rbegin()->first:poses.begin()->first,
			poses,
			constraints,
			posesOut,
			linksOut);
		isWholeGraphConnected = posesOut.size() == poses.size();	

		printf("Whole global graph is%s connected to all nodes of WM/LTM (%ld/%ld)\n", isWholeGraphConnected?"":" not", posesOut.size(), ids.size());
	}

	int totalNodesReduced = 0;
	std::vector<int> vids;
	vids.reserve(ids.size());
	if(keepLatest)
	{
		// we process older to newer nodes, merging to new nodes
		vids.insert(vids.end(), ids.begin(), ids.end());
	}
	else
	{
		// we process newer to older nodes, merging to old nodes
		vids.insert(vids.end(), ids.rbegin(), ids.rend());
	}

	if(preCleanup)
	{
		if(memory.getMaxStMemSize() <= 1)
		{
			printf("--pre_cleanup is used but %s <= 1, skipping pre cleanup...\n", Parameters::kMemSTMSize().c_str());
		}
		else
		{
			int totalRemoved = 0;
			for(auto id: vids)
			{
				auto nids = memory.getNeighborsId(id, memory.getMaxStMemSize(), -1, true, true, true);
				auto links = memory.getLinks(id, true, false);
				for(auto link:links)
				{
					if( link.second.type() == Link::kUserClosure &&
						nids.find(link.first)!=nids.end())
					{
						memory.removeLink(id, link.first);
						++totalRemoved;
					}
				}
			}
			printf("Removed %d user links that were linking nodes that were close in the graph (below %s=%d)\n",
				totalRemoved, Parameters::kMemSTMSize().c_str(), memory.getMaxStMemSize());
		}
	}

	// Get local optimized graph before reduction
	Transform lastLocalizationPose;
	std::map<int, Transform> optimizedPoses = memory.loadOptimizedPoses(&lastLocalizationPose);

	// Graph reduction
	for(auto id: vids)
	{
		// Nodes can be already reduced by other nodes, check if they are still there
		if(memory.getSignature(id) != 0)
		{
			int reducedId = memory.reduceNode(id, radius, keepLinked, keepLatest?1:-1);
			if(reducedId > 0)
			{
				printf("Reduced node %d to node %d!\n", id, reducedId);
				++totalNodesReduced;
			}
		}
	}
	printf("Reduced a total of %d nodes out of %ld nodes\n", totalNodesReduced, ids.size());

	if(isWholeGraphConnected || removeAllOrphanNodes)
	{
		// refetch the global graph after graph reduction
		ids = memory.getAllSignatureIds();

		// Check if some nodes got disconnected from the global graph
		size_t totalBefore = poses.size();
		poses.clear();
		constraints.clear();
		memory.getMetricConstraints(ids, poses, constraints, false, true);
		std::map<int, Transform> posesOut;
		std::multimap<int, Link> linksOut;
		optimizer->getConnectedGraph(
			keepLatest?poses.rbegin()->first:poses.lower_bound(0)->first,
			poses,
			constraints,
			posesOut,
			linksOut);

		isWholeGraphConnected = posesOut.size() == poses.size();
		
		if(!isWholeGraphConnected)
		{
			if(removeOrphanNodes)
			{
				printf("Option %s is enabled, let's cleanup WM and LTM "
					"%ld from nodes not in the global graph anymore (%ld -> reduced to %ld -> %ld connected globally).\n",
					removeAllOrphanNodes?"--remove_all_orphan_nodes":"--remove_orphan_nodes",
					poses.size()-posesOut.size(),
					totalBefore, ids.size(), posesOut.size());
				// Not all graph is connected anymore while it was before reduction: 
				// that means we created orphan nodes, remove them
				for(std::map<int, Transform>::iterator iter=poses.lower_bound(0); iter!=poses.end(); ++iter)
				{
					if(posesOut.find(iter->first) == posesOut.end())
					{
						memory.deleteLocation(iter->first, 0);
						printf("Removed %d from WM/LTM.\n", iter->first);
					}
				}
			}
			else {
				printf("[Warning] After graph reduction, the global graph is not all "
					"connected anymore while it was before (%ld -> reduced to %ld -> %ld connected globally). "
					"Add %s option to remove the %ld orphan nodes from WM and LTM.\n",
					totalBefore, ids.size(), posesOut.size(),
					removeAllOrphanNodes?"--remove_all_orphan_nodes":"--remove_orphan_nodes",
					poses.size()-posesOut.size());
			}
		}
		else
		{
			printf("Whole optimized graph is still all connected after graph reduction (%ld/%ld)\n", posesOut.size(), ids.size());
		}
	} // else: we cannot detect orphan nodes if the original graph was not all connected.

	if(!optimizedPoses.empty())
	{
		size_t removed = 0;
		// cleanup reduced nodes from the optimized poses
		for(std::map<int, Transform>::iterator iter=optimizedPoses.lower_bound(0); iter!=optimizedPoses.end();)
		{
			if(memory.getSignature(iter->first) == 0)
			{
				iter = optimizedPoses.erase(iter);
				++removed;
			}
			else
			{
				++iter;
			}
		}
		printf("Updated local optimized graph from %ld poses to %ld poses\n", optimizedPoses.size()+removed, optimizedPoses.size());
		printf("Saving back %ld optimized poses to database.\n", optimizedPoses.size());
		memory.saveOptimizedPoses(optimizedPoses, lastLocalizationPose);
	}

	float xMin, yMin, cellSize;
	bool hasOptimizedMap = !memory.load2DMap(xMin, yMin, cellSize).empty();
	if(hasOptimizedMap)
	{
		printf("The database has a global occupancy grid, regenerating one with the remaining nodes of the optimized graph!\n");
		LocalGridCache cache;
		OccupancyGrid grid(&cache, parameters);
		for(std::map<int, Transform>::iterator iter=optimizedPoses.lower_bound(0); iter!=optimizedPoses.end(); ++iter)
		{
			SensorData data = memory.getNodeData(iter->first, false, false, false, true);
			data.uncompressData();
			cache.add(iter->first, data.gridGroundCellsRaw(), data.gridObstacleCellsRaw(), data.gridEmptyCellsRaw(), data.gridCellSize(), data.gridViewPoint());
		}
		grid.update(optimizedPoses);
		cv::Mat map = grid.getMap(xMin, yMin);
		if(map.empty())
		{
			printf("Could not regenerate the global occupancy grid! The grid is not updated.\n");
		}
		else
		{
			memory.save2DMap(map, xMin, yMin, grid.getCellSize());
			printf("Saved the new global occupancy grid!\n");
		}
	}

	// Restore original parameters before saving back the database
	memory.parseParameters(originalParameters);
	
	printf("Saving all changes to database...\n");
	memory.close(true);

	return 0;
}
