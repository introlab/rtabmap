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

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/DBReader.h>
#ifdef RTABMAP_OCTOMAP
#include <rtabmap/core/global_map/OctoMap.h>
#endif
#include <rtabmap/core/global_map/OccupancyGrid.h>
#include <rtabmap/core/global_map/CloudMap.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/SensorCaptureThread.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include <signal.h>
#include <algorithm>
#include <tuple>
#include <iterator>

using namespace rtabmap;

// Links between the nodes of a graph, one per pair of nodes and type, with the
// transform from the lowest id to the highest. Self links (priors, gravity),
// landmarks and virtual links are ignored.
std::map<std::tuple<int, int, int>, Transform> graphLinks(const std::multimap<int, Link> & links, const std::set<int> & nodes)
{
	std::map<std::tuple<int, int, int>, Transform> out;
	for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
	{
		const Link & link = iter->second;
		if(link.from() == link.to() || link.from() <= 0 || link.to() <= 0 ||
		   link.type() == Link::kVirtualClosure || link.type() >= Link::kPosePrior ||
		   nodes.find(link.from()) == nodes.end() || nodes.find(link.to()) == nodes.end())
		{
			continue;
		}
		const bool forward = link.from() < link.to();
		out.insert(std::make_pair(
				std::make_tuple(forward?link.from():link.to(), forward?link.to():link.from(), (int)link.type()),
				forward?link.transform():link.transform().inverse()));
	}
	return out;
}

// Compares the nodes and links of two graphs, returns an empty string if they are the same.
// With neighborTransformsIgnored, only the nodes of the neighbor links are compared,
// not their transform.
std::string compareGraphs(
		const std::set<int> & nodesA, const std::multimap<int, Link> & linksA,
		const std::set<int> & nodesB, const std::multimap<int, Link> & linksB,
		bool neighborTransformsIgnored = false)
{
	if(nodesA != nodesB)
	{
		std::vector<int> onlyA, onlyB;
		std::set_difference(nodesA.begin(), nodesA.end(), nodesB.begin(), nodesB.end(), std::back_inserter(onlyA));
		std::set_difference(nodesB.begin(), nodesB.end(), nodesA.begin(), nodesA.end(), std::back_inserter(onlyB));
		return uFormat("different nodes (%d vs %d): %d only in the original (e.g., %d), %d only in the reprocessed (e.g., %d)",
				(int)nodesA.size(), (int)nodesB.size(),
				(int)onlyA.size(), onlyA.empty()?0:onlyA.front(), (int)onlyB.size(), onlyB.empty()?0:onlyB.front());
	}
	std::map<std::tuple<int, int, int>, Transform> a = graphLinks(linksA, nodesA);
	std::map<std::tuple<int, int, int>, Transform> b = graphLinks(linksB, nodesB);
	int onlyA = 0, onlyB = 0, different = 0;
	std::string example;
	for(std::map<std::tuple<int, int, int>, Transform>::iterator iter=a.begin(); iter!=a.end(); ++iter)
	{
		std::map<std::tuple<int, int, int>, Transform>::iterator jter = b.find(iter->first);
		if(jter == b.end())
		{
			if(onlyA++ == 0 && example.empty())
				example = uFormat("link %d->%d (type %d) only in the original", std::get<0>(iter->first), std::get<1>(iter->first), std::get<2>(iter->first));
		}
		else if(!neighborTransformsIgnored || std::get<2>(iter->first) != Link::kNeighbor)
		{
			const Transform d = iter->second.inverse() * jter->second;
			const float angle = Eigen::AngleAxisf(d.toEigen3f().rotation()).angle();
			if(d.getNorm() > 0.001f || fabs(angle) > 0.001f)
			{
				if(different++ == 0 && example.empty())
					example = uFormat("link %d->%d (type %d) differs by %f m and %f rad", std::get<0>(iter->first), std::get<1>(iter->first), std::get<2>(iter->first), d.getNorm(), angle);
			}
		}
	}
	for(std::map<std::tuple<int, int, int>, Transform>::iterator iter=b.begin(); iter!=b.end(); ++iter)
	{
		if(a.find(iter->first) == a.end() && onlyB++ == 0 && example.empty())
			example = uFormat("link %d->%d (type %d) only in the reprocessed", std::get<0>(iter->first), std::get<1>(iter->first), std::get<2>(iter->first));
	}
	if(onlyA || onlyB || different)
	{
		return uFormat("different links (%d vs %d): %d only in the original, %d only in the reprocessed, %d with a different transform (e.g., %s)",
				(int)a.size(), (int)b.size(), onlyA, onlyB, different, example.c_str());
	}
	return "";
}

void showUsage()
{
	printf("\nUsage:\n"
			"   rtabmap-reprocess [options] \"input.db\" \"output.db\"\n"
			"   rtabmap-reprocess [options] \"input1.db;input2.db;input3.db\" \"output.db\"\n"
			"\n"
			"   For the second example, only parameters from the first database are used (unless -params_last or -default are used).\n"
			"   If Mem/IncrementalMemory is false, RTAB-Map is initialized with the first input database,\n"
			"   then localization-only is done with next databases against the first one.\n"
			"   To see warnings when loop closures are rejected, add \"--uwarn\" argument.\n"
			"   To upgrade version of an old database to newest version:\n"
			"      rtabmap-reprocess --Db/TargetVersion \"\" \"input.db\" \"output.db\"\n"
			"\n"
			"  Options:\n"
			"     -r                Use database stamps as input rate.\n"
			"     -skip #           Skip # frames after each processed frame (default 0=don't skip any frames).\n"
			"     -c \"path.ini\"   Configuration file, overwriting parameters read \n"
			"                       from the database. If custom parameters are also set as \n"
			"                       arguments, they overwrite those in config file and the database.\n"
			"     -default    Input database's parameters are ignored, using default ones instead.\n"
			"     -params_last  Parameters of the last database is used instead of the first one (ignored if -default is also used).\n"
			"     -odom       Recompute odometry. See \"Odom/\" parameters with --params. If -skip option\n"
			"                 is used, it will be applied to odometry frames, not rtabmap frames. Multi-session\n"
			"                 may not be detected correctly if the input covariance between sessions doesn't have 9999.\n"
			"     -odom_input_guess   Forward input database's odometry (if exists) as guess when recomputing odometry.\n"
			"     -odom_lin_var #.#   Override computed odometry linear covariance.\n"
			"     -odom_ang_var #.#   Override computed odometry angular covariance.\n"
			"     -start #    Start from this node ID.\n"
			"     -stop #     Last node to process.\n"
			"     -start_s #  Start from this map session ID.\n"
			"     -stop_s #   Last map session to process.\n"
			"     -stop_loop  Stop after the first loop closure is detected.\n"
			"     -a          Append mode: if Mem/IncrementalMemory is true, RTAB-Map is initialized with the first input database,\n"
			"                 then next databases are reprocessed on top of the first one.\n"
			"     -cam #      Camera index to stream. Ignored if a database doesn't contain multi-camera data. Can also be multiple \n"
			"                 indices split by spaces in a string like \"0 2\" to stream cameras 0 and 2 only.\n"
			"     -cam_tf \"x y z roll pitch yaw\" Camera local transform override(s) without optical rotation. For multi-cameras, \n"
			"                                      use a \";\" between each transform.\n"
			"     -cam_tf_lens_offset #.#          Override camera local transform with an y-axis offset before optical rotation. \n"
			"                                      If -cam_tf is also used, it is combined before optical rotation. For multi-cameras,\n"
			"                                      -cam_tf should be also used, and explicitly enumerate offsets if they are different, \n"
			"                                      e.g., \"0.05 0.075\" for two cameras setup.\n"
			"     -nolandmark Don't republish landmarks contained in input database.\n"
			"     -nopriors   Don't republish priors contained in input database.\n"
			"     -noimu      Don't republish IMU contained in input database.\n"
			"     -nogrids    Don't republish local occupancy grids contained in input database: they are\n"
			"                 regenerated from the depth images or laser scans. Enabled automatically if a\n"
			"                 \"Grid/\" parameter is overridden.\n"
			"     -pub_loops  Republish loop closures contained in input database.\n"
			"     -pub_loops_only  Same as -pub_loops, but loop closure and proximity detection are disabled,\n"
			"                      so that the output graph has only the links of the input database. The\n"
			"                      parameters changed to do so are restored before saving the output database.\n"
			"     -keep_opt_graph  When reprocessing a single database, save back its original optimized graph,\n"
			"                      last localization pose, 2D map, optimized mesh (if any) and neighbor links in\n"
			"                      the output database, only if the reprocessed graph has exactly the same nodes\n"
			"                      and links (the transforms of the neighbor links, recomputed from the odometry\n"
			"                      poses, can differ). With -pub_loops_only, graph optimization is disabled\n"
			"                      while reprocessing (Optimizer/Iterations=0 unless set explicitly, restored\n"
			"                      before saving). If the graphs differ, the reprocessed graph is optimized once\n"
			"                      at the end instead (if it was not optimized).\n"
			"     -pub_inter_as_normal Republish intermediate nodes as normal nodes.\n"
			"     -abort_disconnected_sessions Return error if not all sessions are connected together.\n"
			"     -loc_null   On localization mode, reset localization pose to null and map correction to identity between sessions.\n"
			"     -gt         When reprocessing a single database, load its original optimized graph, then \n"
			"                 set it as ground truth for output database. If there was a ground truth in the input database, it will be ignored.\n"
			"     -g2         Assemble 2D occupancy grid map and save it to \"[output]_map.pgm\". Use with -db to save in database.\n"
			"     -g3         Assemble 3D cloud map and save it to \"[output]_map.pcd\".\n"
			"     -o2         Assemble OctoMap 2D projection and save it to \"[output]_octomap.pgm\". Use with -db to save in database.\n"
			"     -o3         Assemble OctoMap 3D cloud and save it to \"[output]_octomap.pcd\".\n"
			"     -db         Save assembled 2D occupancy grid in database instead of a file.\n"
			"     -p          Save odometry and localization poses (*.g2o).\n"
			"     -scan_from_depth    Generate scans from depth images (overwrite previous\n"
			"                         scans if they exist).\n"
			"     -scan_downsample #  Downsample input scans.\n"
			"     -scan_range_min #.#   Filter input scans with minimum range (m).\n"
			"     -scan_range_max #.#   Filter input scans with maximum range (m).\n"
			"     -scan_voxel_size #.#  Voxel filter input scans (m).\n"
			"     -scan_normal_k #         Compute input scan normals (k-neighbors approach).\n"
			"     -scan_normal_radius #.#  Compute input scan normals (radius(m)-neighbors approach).\n\n"
			"%s\n"
			"\n", Parameters::showUsage());
	exit(1);
}

// catch ctrl-c
bool g_loopForever = true;
void sighandler(int sig)
{
	printf("\nSignal %d caught...\n", sig);
	g_loopForever = false;
}

int loopCount = 0;
int proxCount = 0;
int loopCountMotion = 0;
int loopInter = 0;
int loopIntra = 0;
int totalFrames = 0;
int totalFramesMotion = 0;
std::vector<float> previousLocalizationDistances;
std::vector<float> odomDistances;
std::vector<float> localizationVariations;
std::vector<float> localizationAngleVariations;
std::vector<float> localizationTime;
std::map<int, Transform> odomTrajectoryPoses;
std::multimap<int, Link> odomTrajectoryLinks;
std::map<int, Transform> localizationPoses;
bool exportPoses = false;
int sessionCount = 0;
void showLocalizationStats(const std::string & outputDatabasePath)
{
	printf("Total localizations on previous session = %d/%d (Loop=%d, Prox=%d, In Motion=%d/%d)\n", loopCount+proxCount, totalFrames, loopCount, proxCount, loopCountMotion, totalFramesMotion);
	{
		float m = uMean(localizationTime);
		float var = uVariance(localizationTime, m);
		float stddev = -1;
		if(var>0)
		{
			stddev = sqrt(var);
		}
		printf("Average localization time = %f ms (stddev=%f ms)\n", m, stddev);
	}
	if(localizationVariations.size()>=2)
	{
		//ignore first localization
		localizationVariations = std::vector<float>(++localizationVariations.begin(), localizationVariations.end());
		localizationAngleVariations = std::vector<float>(++localizationAngleVariations.begin(), localizationAngleVariations.end());

		float m = uMean(localizationVariations);
		float max = uMax(localizationVariations);
		float var = uVariance(localizationVariations, m);
		float stddev = -1;
		if(var>0)
		{
			stddev = sqrt(var);
		}
		float mA = uMean(localizationAngleVariations);
		float maxA = uMax(localizationAngleVariations);
		float varA = uVariance(localizationAngleVariations, mA);
		float stddevA = -1;
		if(varA>0)
		{
			stddevA = sqrt(varA);
		}
		printf("Average localization variations = %f m, %f deg (stddev=%f m, %f deg) (max=%f m, %f deg)\n", m, mA, stddev, stddevA, max, maxA);
	}
	if(!previousLocalizationDistances.empty())
	{
		float m = uMean(previousLocalizationDistances);
		float var = uVariance(previousLocalizationDistances, m);
		float stddev = -1;
		if(var>0)
		{
			stddev = sqrt(var);
		}
		printf("Average distance from previous localization = %f m (stddev=%f m)\n", m, stddev);
	}
	if(!odomDistances.empty())
	{
		float m = uMean(odomDistances);
		float var = uVariance(odomDistances, m);
		float stddev = -1;
		if(var>0)
		{
			stddev = sqrt(var);
		}
		printf("Average odometry distances = %f m (stddev=%f m)\n", m, stddev);
	}

	if(exportPoses)
	{
		std::string outputPath = outputDatabasePath.substr(0, outputDatabasePath.size()-3);
		std::string oName = outputPath+uFormat("_session_%d_odom.g2o", sessionCount);
		std::string lName = outputPath+uFormat("_session_%d_loc.g2o", sessionCount);
		graph::exportPoses(oName, 4, odomTrajectoryPoses, odomTrajectoryLinks);
		graph::exportPoses(lName, 4, localizationPoses, odomTrajectoryLinks);
		printf("Exported %s and %s\n", oName.c_str(), lName.c_str());
	}

	loopCount = 0;
	proxCount = 0;
	totalFrames = 0;
	loopCountMotion = 0;
	totalFramesMotion = 0;
	previousLocalizationDistances.clear();
	odomDistances.clear();
	localizationVariations.clear();
	localizationAngleVariations.clear();
	localizationTime.clear();
	odomTrajectoryPoses.clear();
	odomTrajectoryLinks.clear();
	localizationPoses.clear();
	++sessionCount;
}

std::set<int> getMapIds(const std::set<int> & ids, const rtabmap::DBDriver & driver)
{
	std::set<int> mapIds;
	for(auto id: ids)
	{
		Transform p;
		int mapId;
		int w;
		std::string l;
		double s;
		Transform gt;
		std::vector<float> v;
		GPS g;
		EnvSensors ss;

		if(driver.getNodeInfo(id, p, mapId, w, l, s, gt, v, g, ss) && mapId>=0)
		{
			mapIds.insert(mapId);
		}
	}
	return mapIds;
}

int main(int argc, char * argv[])
{
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kError);

	// override help from Parameters to show the whole thing
	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}
	}

	ParametersMap customParameters = Parameters::parseArguments(argc, argv);

	if(argc < 3)
	{
		showUsage();
	}

	bool save2DMap = false;
	bool assemble2dMap = false;
	bool assemble3dMap = false;
	bool assemble2dOctoMap = false;
	bool assemble3dOctoMap = false;
	bool useDatabaseRate = false;
	bool useDefaultParameters = false;
	bool useLastDatabaseParameters = false;
	bool recomputeOdometry = false;
	bool useInputOdometryAsGuess = false;
	double odomLinVarOverride = 0.0;
	double odomAngVarOverride = 0.0;
	int startId = 0;
	int stopId = 0;
	int startMapId = 0;
	int stopMapId = -1;
	bool stopOnLoopClosure = false;
	bool appendMode = false;
	std::vector<unsigned int> cameraIndices;
	std::vector<Transform> cameraLocalTransformOverrides;
	std::vector<float> cameraLocalTransformOffsetOverrides;
	int framesToSkip = 0;
	bool ignoreLandmarks = false;
	bool ignorePriors = false;
	bool ignoreImu = false;
	bool republishLoopClosures = false;
	bool republishLoopClosuresOnly = false;
	bool ignoreLocalGrids = false;
	bool keepOptimizedGraph = false;
	bool pubInterNodesAsNormalNodes = false;
	bool abortDisconnectedSessions = false;
	bool locNull = false;
	bool originalGraphAsGT = false;
	bool scanFromDepth = false;
	int scanDecimation = 1;
	float scanRangeMin = 0.0f;
	float scanRangeMax = 0.0f;
	float scanVoxelSize = 0;
	int scanNormalK = 0;
	float scanNormalRadius = 0.0f;
	ParametersMap configParameters;
	for(int i=1; i<argc-2; ++i)
	{
		if(strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--r") == 0)
		{
			useDatabaseRate = true;
			printf("Using database stamps as input rate.\n");
		}
		else if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--c") == 0)
		{
			++i;
			if (i < argc - 2 && UFile::exists(argv[i]) && UFile::getExtension(argv[i]).compare("ini") == 0)
			{
				Parameters::readINI(argv[i], configParameters);
				printf("Using %d parameters from config file \"%s\"\n", (int)configParameters.size(), argv[i]);
			}
			else if(i < argc - 2)
			{
				printf("Config file \"%s\" is not valid or doesn't exist!\n", argv[i]);
				showUsage();
			}
			else
			{
				printf("Config file is not set!\n");
				showUsage();
			}
		}
		else if(strcmp(argv[i], "-default") == 0 || strcmp(argv[i], "--default") == 0)
		{
			useDefaultParameters = true;
			printf("Using default parameters.\n");
		}
		else if(strcmp(argv[i], "-params_last") == 0 || strcmp(argv[i], "--params_last") == 0)
		{
			useLastDatabaseParameters = true;
		}
		else if(strcmp(argv[i], "-odom") == 0 || strcmp(argv[i], "--odom") == 0)
		{
			recomputeOdometry = true;
		}
		else if(strcmp(argv[i], "-odom_input_guess") == 0 || strcmp(argv[i], "--odom_input_guess") == 0)
		{
			recomputeOdometry = true;
			useInputOdometryAsGuess = true;
		}
		else if (strcmp(argv[i], "-odom_lin_var") == 0 || strcmp(argv[i], "--odom_lin_var") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				odomLinVarOverride = uStr2Double(argv[i]);
				printf("Odometry linear variance overriden to = %f.\n", odomLinVarOverride);
			}
			else
			{
				printf("-odom_lin_var option requires a value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-odom_ang_var") == 0 || strcmp(argv[i], "--odom_ang_var") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				odomAngVarOverride = uStr2Double(argv[i]);
				printf("Odometry angular variance overriden to = %f.\n", odomAngVarOverride);
			}
			else
			{
				printf("-odom_ang_var option requires a value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-start") == 0 || strcmp(argv[i], "--start") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				startId = atoi(argv[i]);
				printf("Start at node ID = %d.\n", startId);
			}
			else
			{
				printf("-start option requires a value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-stop") == 0 || strcmp(argv[i], "--stop") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				stopId = atoi(argv[i]);
				printf("Stop at node ID = %d.\n", stopId);
			}
			else
			{
				printf("-stop option requires a value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-start_s") == 0 || strcmp(argv[i], "--start_s") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				startMapId = atoi(argv[i]);
				printf("Start at map session ID = %d.\n", startMapId);
			}
			else
			{
				printf("-start_s option requires a value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-stop_s") == 0 || strcmp(argv[i], "--stop_s") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				stopMapId = atoi(argv[i]);
				printf("Stop at map session ID = %d.\n", stopMapId);
			}
			else
			{
				printf("-stop option requires a value\n");
				showUsage();
			}
		}
		else if(strcmp(argv[i], "-stop_loop") == 0 || strcmp(argv[i], "--stop_loop") == 0)
		{
			stopOnLoopClosure = true;
		}
		else if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--a") == 0)
		{
			appendMode = true;
			printf("Append mode enabled (initialize with first database then reprocess next ones)\n");
		}
		else if (strcmp(argv[i], "-cam") == 0 || strcmp(argv[i], "--cam") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				std::list<std::string> indicesStr = uSplit(argv[i], ' ');
				for(std::list<std::string>::iterator iter=indicesStr.begin(); iter!=indicesStr.end(); ++iter)
				{
					cameraIndices.push_back(uStr2Int(*iter));
					printf("Camera index = %d.\n", cameraIndices.back());
				}
			}
			else
			{
				printf("-cam option requires a value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-cam_tf") == 0 || strcmp(argv[i], "--cam_tf") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				std::list<std::string> tfStr = uSplit(argv[i], ';');
				for(std::list<std::string>::iterator iter=tfStr.begin(); iter!=tfStr.end(); ++iter)
				{
					cameraLocalTransformOverrides.push_back(Transform::fromString(*iter));
					printf("Camera transform = %s\n", cameraLocalTransformOverrides.back().prettyPrint().c_str());
				}
			}
			else
			{
				printf("-cam option requires a value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-cam_tf_lens_offset") == 0 || strcmp(argv[i], "--cam_tf_lens_offset") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				std::list<std::string> offsetStr = uSplit(argv[i], ' ');
				for(std::list<std::string>::iterator iter=offsetStr.begin(); iter!=offsetStr.end(); ++iter)
				{
					cameraLocalTransformOffsetOverrides.push_back(uStr2Float(*iter));
					printf("Camera offset = %f\n", cameraLocalTransformOffsetOverrides.back());
				}
			}
			else
			{
				printf("-cam option requires a value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-skip") == 0 || strcmp(argv[i], "--skip") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				framesToSkip = atoi(argv[i]);
				printf("Will skip %d frames.\n", framesToSkip);
			}
			else
			{
				printf("-skip option requires a value\n");
				showUsage();
			}
		}
		else if(strcmp(argv[i], "-nolandmark") == 0 || strcmp(argv[i], "--nolandmark") == 0)
		{
			ignoreLandmarks = true;
			printf("Ignoring landmarks from input database (-nolandmark option).\n");
		}
		else if(strcmp(argv[i], "-nopriors") == 0 || strcmp(argv[i], "--nopriors") == 0)
		{
			ignorePriors = true;
			printf("Ignoring priors from input database (-nopriors option).\n");
		}
		else if(strcmp(argv[i], "-noimu") == 0 || strcmp(argv[i], "--noimu") == 0)
		{
			ignoreImu = true;
			printf("Ignoring IMU from input database (-noimu option).\n");
		}
		else if(strcmp(argv[i], "-pub_loops") == 0 || strcmp(argv[i], "--pub_loops") == 0)
		{
			republishLoopClosures = true;
			printf("Republish loop closures from input database (-pub_loops option).\n");
		}
		else if(strcmp(argv[i], "-pub_loops_only") == 0 || strcmp(argv[i], "--pub_loops_only") == 0)
		{
			republishLoopClosures = true;
			republishLoopClosuresOnly = true;
			printf("Republish loop closures from input database, without detecting new ones (-pub_loops_only option).\n");
		}
		else if(strcmp(argv[i], "-nogrids") == 0 || strcmp(argv[i], "--nogrids") == 0)
		{
			ignoreLocalGrids = true;
			printf("Ignoring local occupancy grids from input database, they will be regenerated (-nogrids option).\n");
		}
		else if(strcmp(argv[i], "-keep_opt_graph") == 0 || strcmp(argv[i], "--keep_opt_graph") == 0)
		{
			keepOptimizedGraph = true;
			printf("Keep original optimized graph of the input database if the nodes and links match (-keep_opt_graph option).\n");
		}
		else if(strcmp(argv[i], "-pub_inter_as_normal") == 0 || strcmp(argv[i], "--pub_inter_as_normal") == 0)
		{
			pubInterNodesAsNormalNodes = true;
			printf("Republish intermdiate nodes as normal nodes (-pub_inter_as_normal option).\n");
		}
		else if(strcmp(argv[i], "-abort_disconnected_sessions") == 0 || strcmp(argv[i], "--abort_disconnected_sessions") == 0)
		{
			abortDisconnectedSessions = true;
			printf("Will abort if not all sessions are connected at the end (-abort_disconnected_sessions option).\n");
		}
		else if(strcmp(argv[i], "-loc_null") == 0 || strcmp(argv[i], "--loc_null") == 0)
		{
			locNull = true;
			printf("In localization mode, when restarting a new session, the current localization pose is set to null (-loc_null option).\n");
		}
		else if(strcmp(argv[i], "-gt") == 0 || strcmp(argv[i], "--gt") == 0)
		{
			originalGraphAsGT = true;
			printf("Original graph is used as ground truth for output database (-gt option).\n");
		}
		else if(strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--p") == 0)
		{
			exportPoses = true;
			printf("Odometry trajectory and localization poses will be exported in g2o format (-p option).\n");
		}
		else if(strcmp(argv[i], "-db") == 0 || strcmp(argv[i], "--db") == 0)
		{
			save2DMap = true;
			printf("2D occupancy grid will be saved in database (-db option).\n");
		}
		else if(strcmp(argv[i], "-g2") == 0 || strcmp(argv[i], "--g2") == 0)
		{
			assemble2dMap = true;
			printf("2D occupancy grid will be assembled (-g2 option).\n");
		}
		else if(strcmp(argv[i], "-g3") == 0 || strcmp(argv[i], "--g3") == 0)
		{
			assemble3dMap = true;
			printf("3D cloud map will be assembled (-g3 option).\n");
		}
		else if(strcmp(argv[i], "-o2") == 0 || strcmp(argv[i], "--o2") == 0)
		{
#ifdef RTABMAP_OCTOMAP
			assemble2dOctoMap = true;
			printf("OctoMap will be assembled (-o2 option).\n");
#else
			printf("RTAB-Map is not built with OctoMap support, cannot set -o2 option!\n");
#endif
		}
		else if(strcmp(argv[i], "-o3") == 0 || strcmp(argv[i], "--o3") == 0)
		{
#ifdef RTABMAP_OCTOMAP
			assemble3dOctoMap = true;
			printf("OctoMap will be assembled (-o3 option).\n");
#else
			printf("RTAB-Map is not built with OctoMap support, cannot set -o3 option!\n");
#endif
		}
		else if (strcmp(argv[i], "-scan_from_depth") == 0 || strcmp(argv[i], "--scan_from_depth") == 0)
		{
			scanFromDepth = true;
		}
		else if (strcmp(argv[i], "-scan_downsample") == 0 || strcmp(argv[i], "--scan_downsample") == 0 ||
				strcmp(argv[i], "-scan_decimation") == 0 || strcmp(argv[i], "--scan_decimation") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				scanDecimation = atoi(argv[i]);
				printf("Scan from depth decimation = %d.\n", scanDecimation);
			}
			else
			{
				printf("-scan_downsample option requires 1 value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-scan_range_min") == 0 || strcmp(argv[i], "--scan_range_min") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				scanRangeMin = atof(argv[i]);
				printf("Scan range min = %f m.\n", scanRangeMin);
			}
			else
			{
				printf("-scan_range_min option requires 1 value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-scan_range_max") == 0 || strcmp(argv[i], "--scan_range_max") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				scanRangeMax = atof(argv[i]);
				printf("Scan range max = %f m.\n", scanRangeMax);
			}
			else
			{
				printf("-scan_range_max option requires 1 value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-scan_voxel_size") == 0 || strcmp(argv[i], "--scan_voxel_size") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				scanVoxelSize = atof(argv[i]);
				printf("Scan voxel size = %f m.\n", scanVoxelSize);
			}
			else
			{
				printf("-scan_voxel_size option requires 1 value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-scan_normal_k") == 0 || strcmp(argv[i], "--scan_normal_k") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				scanNormalK = atoi(argv[i]);
				printf("Scan normal k = %d.\n", scanNormalK);
			}
			else
			{
				printf("-scan_normal_k option requires 1 value\n");
				showUsage();
			}
		}
		else if (strcmp(argv[i], "-scan_normal_radius") == 0 || strcmp(argv[i], "--scan_normal_radius") == 0)
		{
			++i;
			if(i < argc - 2)
			{
				scanNormalRadius = atof(argv[i]);
				printf("Scan normal radius = %f m.\n", scanNormalRadius);
			}
			else
			{
				printf("-scan_normal_radius option requires 1 value\n");
				showUsage();
			}
		}
	}

	std::string inputDatabasePath = UDirectory::expandHomeDir(argv[argc-2]);
	std::string outputDatabasePath = UDirectory::expandHomeDir(argv[argc-1]);

	std::list<std::string> databases = uSplit(inputDatabasePath, ';');
	if (databases.empty())
	{
		printf("[Error] No input database \"%s\" detected!\n", inputDatabasePath.c_str());
		return 1;
	}
	for (std::list<std::string>::iterator iter = databases.begin(); iter != databases.end(); ++iter)
	{
		if (!UFile::exists(*iter))
		{
			printf("[Error] Input database \"%s\" doesn't exist!\n", iter->c_str());
			if(uStrContains(inputDatabasePath,":"))
			{
				printf("Did you mean \"%s\"?\n", uReplaceChar(inputDatabasePath, ':', ";").c_str());
			}
			return 1;
		}

		if (UFile::getExtension(*iter).compare("db") != 0)
		{
			printf("[Error] File \"%s\" is not a database format (*.db)! Aborting.\n", iter->c_str());
			return 1;
		}
	}

	if(UFile::getExtension(outputDatabasePath).compare("db") != 0)
	{
		printf("[Error] File \"%s\" is not a database format (*.db)! Aborting.\n", outputDatabasePath.c_str());
		return 1;
	}

	if(UFile::exists(outputDatabasePath))
	{
		UFile::erase(outputDatabasePath);
	}

	// Get parameters of the first database
	std::shared_ptr<DBDriver> dbDriver(DBDriver::create());
	if(!dbDriver->openConnection(databases.front(), false))
	{
		printf("[Error] Failed opening the input database!\n");
		return 1;
	}

	ParametersMap parameters;
	std::string targetVersion;
	if(!useDefaultParameters)
	{
		if(databases.size() > 1 && useLastDatabaseParameters)
		{
			printf("Using last database's parameters.\n");
			std::shared_ptr<DBDriver> lastDbDriver(DBDriver::create());
			if(!lastDbDriver->openConnection(databases.back(), true))
			{
				printf("[Error] Failed opening the last input database!\n");
				return 1;
			}
			parameters = lastDbDriver->getLastParameters();
			targetVersion = lastDbDriver->getDatabaseVersion();
		}
		else
		{
			parameters = dbDriver->getLastParameters();
			targetVersion = dbDriver->getDatabaseVersion();
		}
		if(parameters.empty())
		{
			printf("[Warning] Failed getting parameters from database, reprocessing will be done with default parameters! Database version may be too old (%s).\n", targetVersion.c_str());
		}
	}
	// Parameters of the input database (defaults for the missing ones), to show
	// below which ones are changed for the reprocessing.
	ParametersMap inputParameters = Parameters::getDefaultParameters();
	uInsert(inputParameters, parameters);
	if(!useDefaultParameters)
	{
		// Output database with the same version as the input database (overwriting
		// the one saved in its parameters), unless set explicitly (e.g., "" for latest).
		uInsert(parameters, ParametersPair(Parameters::kDbTargetVersion(), targetVersion));
	}

	if(customParameters.size())
	{
		printf("Custom parameters:\n");
		for(ParametersMap::iterator iter=customParameters.begin(); iter!=customParameters.end(); ++iter)
		{
			printf("  %s %s\n", uPad(iter->first+" =", 25).c_str(), iter->second.c_str());
		}
	}

	// Parameters changed only for this reprocessing by the options of this tool,
	// with their original values, restored before saving the output database
	// so that it doesn't keep them.
	ParametersMap parametersToRestore;

	bool useOdomFeatures = Parameters::defaultMemUseOdomFeatures();
	if((configParameters.find(Parameters::kKpDetectorStrategy())!=configParameters.end() ||
		configParameters.find(Parameters::kVisFeatureType())!=configParameters.end() ||
		customParameters.find(Parameters::kKpDetectorStrategy())!=customParameters.end() ||
		customParameters.find(Parameters::kVisFeatureType())!=customParameters.end()) &&
			configParameters.find(Parameters::kMemUseOdomFeatures())==configParameters.end() &&
			customParameters.find(Parameters::kMemUseOdomFeatures())==customParameters.end())
	{
		Parameters::parse(parameters, Parameters::kMemUseOdomFeatures(), useOdomFeatures);
		if(useOdomFeatures)
		{
			printf("[Warning] %s and/or %s are overwritten but parameter %s is true in the opened database. "
					"Setting it to false for convenience to use the new selected feature detector. Set %s "
					"explicitly to suppress this warning.\n",
					Parameters::kKpDetectorStrategy().c_str(),
					Parameters::kVisFeatureType().c_str(),
					Parameters::kMemUseOdomFeatures().c_str(),
					Parameters::kMemUseOdomFeatures().c_str());
			parametersToRestore.insert(ParametersPair(Parameters::kMemUseOdomFeatures(), "true"));
			uInsert(parameters, ParametersPair(Parameters::kMemUseOdomFeatures(), "false"));
			useOdomFeatures = false;
		}
	}

	if(useOdomFeatures && databases.size() > 1 &&
		configParameters.find(Parameters::kMemUseOdomFeatures())==configParameters.end() &&
		customParameters.find(Parameters::kMemUseOdomFeatures())==customParameters.end())
	{
		printf("[Warning] Parameter %s is set to false for convenience as "
				"there are more than one input database (which could "
				"contain different features). Set %s "
				"explicitly to suppress this warning.\n",
				Parameters::kMemUseOdomFeatures().c_str(),
				Parameters::kMemUseOdomFeatures().c_str());
		useOdomFeatures = false;
	}

	if(republishLoopClosuresOnly && databases.size() > 1)
	{
		printf("[Error] \"pub_loops_only\" option cannot be used with multiple databases input. Aborting.\n");
		return 1;
	}
	if(keepOptimizedGraph && databases.size() > 1)
	{
		printf("[Error] \"keep_opt_graph\" option cannot be used with multiple databases input. Aborting.\n");
		return 1;
	}
	const std::string singleInputDatabasePath = databases.front();

	if(republishLoopClosures)
	{
		if(databases.size() > 1)
		{
			printf("[Warning] \"pub_loops\" option cannot be used with multiple databases input. "
					"Disabling \"pub_loops\" to avoid mismatched loop closure ids.\n");
			republishLoopClosures = false;
		}
		else
		{
			bool generateIds = Parameters::defaultMemGenerateIds();
			Parameters::parse(parameters, Parameters::kMemGenerateIds(), generateIds);
			Parameters::parse(configParameters, Parameters::kMemGenerateIds(), generateIds);
			Parameters::parse(customParameters, Parameters::kMemGenerateIds(), generateIds);
			if(generateIds)
			{
				if(configParameters.find(Parameters::kMemGenerateIds())!=configParameters.end() ||
				   customParameters.find(Parameters::kMemGenerateIds())!=customParameters.end())
				{
					printf("[Warning] \"pub_loops\" option is used but parameter %s is set to true in custom arguments. "
							"Disabling \"pub_loops\" to avoid mismatched loop closure ids.\n",
							Parameters::kMemGenerateIds().c_str());
					republishLoopClosures = false;
				}
				else
				{
					printf("[Warning] \"pub_loops\" option is used but parameter %s is true in the opened database. "
							"Setting parameter %s to false for convenience so that republished loop closure ids match.\n",
							Parameters::kMemGenerateIds().c_str(),
							Parameters::kMemGenerateIds().c_str());
					parametersToRestore.insert(ParametersPair(Parameters::kMemGenerateIds(), "true"));
					uInsert(parameters, ParametersPair(Parameters::kMemGenerateIds(), "false"));
				}
			}
		}
	}
	uInsert(parameters, configParameters);
	uInsert(parameters, customParameters);

	if(!ignoreLocalGrids)
	{
		// The local occupancy grids of the input database are republished and used
		// as is, unless the parameters used to generate them are changed.
		for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end() && !ignoreLocalGrids; ++iter)
		{
			if(iter->first.compare(0, 5, "Grid/") == 0 &&
			   (customParameters.find(iter->first) != customParameters.end() ||
			    configParameters.find(iter->first) != configParameters.end()))
			{
				printf("Parameter %s is overridden, local occupancy grids of the input database are ignored "
						"and will be regenerated (like -nogrids option).\n", iter->first.c_str());
				ignoreLocalGrids = true;
			}
		}
	}

	ParametersMap toolParameters;
	if(republishLoopClosuresOnly)
	{
		// Only the republished links: no proximity detection, and loop closure
		// detection disabled (a threshold the hypotheses (<=1) cannot reach,
		// which also disables RGBD/AggressiveLoopThr).
		toolParameters.insert(ParametersPair(Parameters::kRGBDProximityBySpace(), "false"));
		toolParameters.insert(ParametersPair(Parameters::kRGBDProximityByTime(), "false"));
		toolParameters.insert(ParametersPair(Parameters::kRtabmapLoopThr(), "1.1"));
		if(keepOptimizedGraph &&
		   customParameters.find(Parameters::kOptimizerIterations()) == customParameters.end() &&
		   configParameters.find(Parameters::kOptimizerIterations()) == configParameters.end())
		{
			// Only the links of the input database (no new loop closures to validate with
			// the optimization error): the optimized graph is replaced by the original one
			// at the end, no need to optimize it while reprocessing (unless set explicitly).
			toolParameters.insert(ParametersPair(Parameters::kOptimizerIterations(), "0"));
		}
	}
	for(ParametersMap::iterator iter=toolParameters.begin(); iter!=toolParameters.end(); ++iter)
	{
		if(customParameters.find(iter->first) != customParameters.end() ||
		   configParameters.find(iter->first) != configParameters.end())
		{
			printf("[Warning] Parameter %s is set explicitly, but it is overwritten to %s by the selected options.\n",
					iter->first.c_str(), iter->second.c_str());
		}
		ParametersMap::const_iterator jter = parameters.find(iter->first);
		std::string originalValue = jter!=parameters.end()?jter->second:Parameters::getDefaultParameters().at(iter->first);
		parametersToRestore.insert(ParametersPair(iter->first, originalValue));
		uInsert(parameters, ParametersPair(iter->first, iter->second));
	}
	int optimizerIterations = Parameters::defaultOptimizerIterations();
	Parameters::parse(parameters, Parameters::kOptimizerIterations(), optimizerIterations);
	const bool graphOptimizedWhileReprocessing = optimizerIterations > 0;

	if(republishLoopClosures)
	{
		int memoryThr = Parameters::defaultRtabmapMemoryThr();
		float timeThr = Parameters::defaultRtabmapTimeThr();
		Parameters::parse(parameters, Parameters::kRtabmapMemoryThr(), memoryThr);
		Parameters::parse(parameters, Parameters::kRtabmapTimeThr(), timeThr);
		if(memoryThr != 0 || timeThr != 0.0f)
		{
			printf("[Warning] Memory management is enabled (%s=%d, %s=%f): loop closures of the input database "
					"with a node transferred to long-term memory cannot be republished and will be ignored. "
					"Set them to 0 to republish all of them.\n",
					Parameters::kRtabmapMemoryThr().c_str(), memoryThr,
					Parameters::kRtabmapTimeThr().c_str(), timeThr);
		}
	}

	bool incrementalMemory = Parameters::defaultMemIncrementalMemory();
	Parameters::parse(parameters, Parameters::kMemIncrementalMemory(), incrementalMemory);
	Parameters::parse(parameters, Parameters::kDbTargetVersion(), targetVersion);
	bool intermediateNodes = Parameters::defaultRtabmapCreateIntermediateNodes();
	Parameters::parse(parameters, Parameters::kRtabmapCreateIntermediateNodes(), intermediateNodes);

	int totalIds = 0;
	std::set<int> ids;
	dbDriver->getAllNodeIds(ids, false, false, !pubInterNodesAsNormalNodes && !intermediateNodes);
	if(ids.empty())
	{
		printf("[Error] Input database doesn't have any nodes saved in it.\n");
		dbDriver->closeConnection(false);
		return 1;
	}

	int firstDbMapIds = 0;
	int firstDbLastMapId = -1;
	firstDbMapIds = getMapIds(ids, *dbDriver).size();
	if(appendMode || !incrementalMemory)
	{
		if(databases.size() < 2)
		{
			if(appendMode) {
				printf("[Error] -a (append mode) requires at least 2 input databases, provided \"%s\"! Aborting.\n", inputDatabasePath.c_str());
			}
			else if(!incrementalMemory) {
				printf("[Error] Localization mode is enabled (%s=false), but it requires at least 2 input databases, provided \"%s\"! Aborting.\n",
					Parameters::kMemIncrementalMemory().c_str(), inputDatabasePath.c_str());
			}
			return 1;
		}
		dbDriver->getLastMapId(firstDbLastMapId);
	}
	else
	{
		totalIds = ids.size();
	}

	std::map<int, Transform> gt;
	if(databases.size() == 1 && originalGraphAsGT)
	{
		gt = dbDriver->loadOptimizedPoses();
	}

	dbDriver->closeConnection(false);

	// Count remaining ids in the other databases
	int newDbMapIds = 0;
	for (std::list<std::string>::iterator iter = ++databases.begin(); iter != databases.end(); ++iter)
	{
		if (!dbDriver->openConnection(*iter, false))
		{
			printf("[Error] Failed opening input database!\n");
			return 1;
		}
		ids.clear();
		dbDriver->getAllNodeIds(ids, false, false, !pubInterNodesAsNormalNodes && !intermediateNodes);
		totalIds += ids.size();
		newDbMapIds += getMapIds(ids, *dbDriver).size();
		dbDriver->closeConnection(false);
	}
	dbDriver.reset();

	std::string workingDirectory = UDirectory::getDir(outputDatabasePath);
	printf("Set working directory to \"%s\".\n", workingDirectory.c_str());
	if(!targetVersion.empty())
	{
		printf("Target database version: \"%s\" (set explicitly --%s \"\" to output with latest version.\n", targetVersion.c_str(), Parameters::kDbTargetVersion().c_str());
	}
	uInsert(parameters, ParametersPair(Parameters::kRtabmapWorkingDirectory(), workingDirectory));
	{
		ParametersMap::const_iterator iter = parameters.find(Parameters::kRtabmapPublishStats());
		parametersToRestore.insert(ParametersPair(Parameters::kRtabmapPublishStats(),
				iter!=parameters.end()?iter->second:uBool2Str(Parameters::defaultRtabmapPublishStats())));
	}
	uInsert(parameters, ParametersPair(Parameters::kRtabmapPublishStats(), "true")); // to log status below

	{
		std::list<std::string> changes;
		for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			if(iter->first == Parameters::kRtabmapWorkingDirectory())
			{
				continue; // not saved in the database
			}
			ParametersMap::const_iterator jter = inputParameters.find(iter->first);
			const std::string original = jter!=inputParameters.end()?jter->second:"";
			if(original != iter->second)
			{
				std::string source;
				if(parametersToRestore.find(iter->first) != parametersToRestore.end())
				{
					source = "tool option, restored before saving";
				}
				else if(customParameters.find(iter->first) != customParameters.end())
				{
					source = "argument";
				}
				else if(configParameters.find(iter->first) != configParameters.end())
				{
					source = "config file";
				}
				else
				{
					source = "tool";
				}
				changes.push_back(uFormat("  %s %s -> %s (%s)",
						uPad(iter->first+":", 30).c_str(), original.c_str(), iter->second.c_str(), source.c_str()));
			}
		}
		printf("Parameters changed from the %s (%d):\n", useDefaultParameters?"default ones":"input database", (int)changes.size());
		for(std::list<std::string>::iterator iter=changes.begin(); iter!=changes.end(); ++iter)
		{
			printf("%s\n", iter->c_str());
		}
	}

	if((!incrementalMemory || appendMode ) && databases.size() > 1)
	{
		UFile::copy(databases.front(), outputDatabasePath);
		if(!incrementalMemory)
		{
			printf("Parameter \"%s\" is set to false, initializing RTAB-Map with \"%s\" for localization...\n", Parameters::kMemIncrementalMemory().c_str(), databases.front().c_str());
		}
		databases.pop_front();
		inputDatabasePath = uJoin(databases, ";");
	}

	Rtabmap rtabmap;
	rtabmap.init(parameters, outputDatabasePath);

	// -pub_loops_only: a node not moving enough is kept if a loop closure is detected
	// on its update, but the loop closures of the input database are republished after
	// its update. The update thresholds are disabled for the nodes having loop closures
	// with previous nodes in the input database, so that they are kept as in the input
	// database.
	ParametersMap updateThresholds;
	ParametersMap updateThresholdsDisabled;
	if(republishLoopClosuresOnly)
	{
		float linearUpdate = Parameters::defaultRGBDLinearUpdate();
		float angularUpdate = Parameters::defaultRGBDAngularUpdate();
		Parameters::parse(parameters, Parameters::kRGBDLinearUpdate(), linearUpdate);
		Parameters::parse(parameters, Parameters::kRGBDAngularUpdate(), angularUpdate);
		if(linearUpdate > 0.0f || angularUpdate > 0.0f)
		{
			updateThresholds.insert(ParametersPair(Parameters::kRGBDLinearUpdate(), uNumber2Str(linearUpdate)));
			updateThresholds.insert(ParametersPair(Parameters::kRGBDAngularUpdate(), uNumber2Str(angularUpdate)));
			updateThresholdsDisabled.insert(ParametersPair(Parameters::kRGBDLinearUpdate(), "0"));
			updateThresholdsDisabled.insert(ParametersPair(Parameters::kRGBDAngularUpdate(), "0"));
		}
	}

	if(!incrementalMemory && locNull)
	{
		rtabmap.setInitialPose(Transform());
	}

	bool rgbdEnabled = Parameters::defaultRGBDEnabled();
	Parameters::parse(parameters, Parameters::kRGBDEnabled(), rgbdEnabled);
	bool odometryIgnored = !rgbdEnabled;

	if(rgbdEnabled && appendMode)
	{
		// Get the number of sessions linked in the current global graph
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		rtabmap.getGraph(poses, constraints, /*optimized*/ false, /*global*/ true, 0, false, false, false, false, false, false);
		std::set<int> mapIds;
		for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			int id;
			if((id=rtabmap.getMemory()->getMapId(iter->first, true))>=0)
			{
				mapIds.insert(id);
			}
		}
		printf("Sessions in global graph of the initial database: %ld/%d\n", mapIds.size(), firstDbMapIds);
		firstDbMapIds = mapIds.size();
	}

	if(!cameraLocalTransformOffsetOverrides.empty())
	{
		if(!cameraLocalTransformOverrides.empty() && cameraLocalTransformOffsetOverrides.size() > 1 && cameraLocalTransformOffsetOverrides.size() != cameraLocalTransformOverrides.size())
		{
			printf("[Error] -cam_tf_lens_offset size (%ld) is not equal to -cam_tf argument (%ld). "
				   "-cam_tf_lens_offset should be one to affect all cameras or same size than -cam_tf argument.\n",
				   cameraLocalTransformOffsetOverrides.size(), cameraLocalTransformOverrides.size());
			showUsage();
			return 1;
		}
		if(cameraLocalTransformOverrides.empty())
		{
			if(cameraLocalTransformOffsetOverrides.size() > 1)
			{
				printf("[Error] -cam_tf_lens_offset size (%ld) should be one if -cam_tf is not set.\n",
				   cameraLocalTransformOffsetOverrides.size());
				showUsage();
				return 1;
			}
			cameraLocalTransformOverrides.push_back(Transform::getIdentity());
		}
		for(size_t i=0; i<cameraLocalTransformOverrides.size(); ++i)
		{
			float offset = cameraLocalTransformOffsetOverrides.size()==1?cameraLocalTransformOffsetOverrides[0]:cameraLocalTransformOffsetOverrides[i];
			cameraLocalTransformOverrides[i] *= Transform(0, offset, 0);
			printf("Overriding camera's local transform %ld to %s (offset=%f)\n", i, cameraLocalTransformOverrides[i].prettyPrint().c_str(), offset);
		}
	}

	int loopClosuresRepublished = 0;
	int loopClosuresInLTM = 0;
	int loopClosuresNotInMap = 0;
	int loopClosuresRejected = 0;
	DBReader * dbReader = new DBReader(
			inputDatabasePath,
			useDatabaseRate?-1:0,
			odometryIgnored,
			false,
			false,
			startId,
			cameraIndices,
			stopId,
			!pubInterNodesAsNormalNodes && !intermediateNodes,
			ignoreLandmarks,
			!useOdomFeatures,
			startMapId,
			stopMapId,
			ignorePriors,
			ignoreImu,
			pubInterNodesAsNormalNodes,
			cameraLocalTransformOverrides,
			ignoreLocalGrids);

	dbReader->init();

	LocalGridCache mapCache;
	OccupancyGrid grid(&mapCache, parameters);
#ifdef RTABMAP_OCTOMAP
	OctoMap octomap(&mapCache, parameters);
#endif
	CloudMap cloudMap(&mapCache, parameters);

	float linearUpdate = Parameters::defaultRGBDLinearUpdate();
	float angularUpdate = Parameters::defaultRGBDAngularUpdate();
	Parameters::parse(parameters, Parameters::kRGBDLinearUpdate(), linearUpdate);
	Parameters::parse(parameters, Parameters::kRGBDAngularUpdate(), angularUpdate);

	Odometry * odometry = 0;
	float rtabmapUpdateRate = Parameters::defaultRtabmapDetectionRate();
	Parameters::parse(parameters, Parameters::kRtabmapDetectionRate(), rtabmapUpdateRate);
	if(rtabmapUpdateRate!=0)
	{
		rtabmapUpdateRate = 1.0f/rtabmapUpdateRate;
	}
	double lastUpdateStamp = 0;
	if(recomputeOdometry)
	{
		if(odometryIgnored)
		{
			printf("odom option is set but %s parameter is false, odometry won't be recomputed...\n", Parameters::kRGBDEnabled().c_str());
			recomputeOdometry = false;
		}
		else
		{
			printf("Odometry will be recomputed (\"odom\" option is set)%s.\n",
				useInputOdometryAsGuess?" with input odometry guess (\"odom_guess_input\" option is set)":"");
			odometry = Odometry::create(parameters);
		}
	}
	else if(!intermediateNodes && framesToSkip == 0 &&
		(configParameters.find(Parameters::kRtabmapDetectionRate())!=configParameters.end() ||
	     customParameters.find(Parameters::kRtabmapDetectionRate())!=customParameters.end()))
	{
		printf("[Warning] Parameter %s is ignored because parameter %s=false.\n",
				Parameters::kRtabmapDetectionRate().c_str(),
				Parameters::kRtabmapCreateIntermediateNodes().c_str());
	}

	printf("Reprocessing data of \"%s\"...\n", inputDatabasePath.c_str());
	std::map<std::string, float> globalMapStats;
	int processed = 0;
	SensorCaptureInfo info;
	SensorData data = dbReader->takeData(&info);
	SensorCaptureThread camThread(dbReader, parameters); // take ownership of dbReader
	camThread.setScanParameters(scanFromDepth, scanDecimation, scanRangeMin, scanRangeMax, scanVoxelSize, scanNormalK, scanNormalRadius);
	if(scanFromDepth)
	{
		data.setLaserScan(LaserScan());
	}
	camThread.postUpdate(&data, &info);
	Transform lastLocalizationOdomPose = info.odomPose;
	Transform previousOdomPose;
	cv::Mat odomCovariance;
	bool inMotion = true;
	while(data.isValid() && g_loopForever)
	{
		if(recomputeOdometry)
		{
			if(useInputOdometryAsGuess && !info.odomCovariance.empty() && info.odomCovariance.at<double>(0,0) >= 9999)
			{
				if(!odometry->getPose().isIdentity()) {
					printf("Reset odometry as input odometry triggered new map\n");
					odometry->reset(odometry->getPose());
				}
				previousOdomPose.setNull();
			}
			OdometryInfo odomInfo;
			Transform pose = odometry->process(data, 
				(useInputOdometryAsGuess && !info.odomPose.isNull() && !previousOdomPose.isNull())?previousOdomPose.inverse() * info.odomPose:Transform(),
				&odomInfo);
			if(!pose.isNull() && odomInfo.reg.covariance.total() == 36)
			{
				previousOdomPose = info.odomPose;
				if(odomLinVarOverride > 0.0)
				{
					odomInfo.reg.covariance.at<double>(0,0) = odomLinVarOverride;
					odomInfo.reg.covariance.at<double>(1,1) = odomLinVarOverride;
					odomInfo.reg.covariance.at<double>(2,2) = odomLinVarOverride;
				}
				if(odomAngVarOverride > 0.0)
				{
					odomInfo.reg.covariance.at<double>(3,3) = odomAngVarOverride;
					odomInfo.reg.covariance.at<double>(4,4) = odomAngVarOverride;
					odomInfo.reg.covariance.at<double>(5,5) = odomAngVarOverride;
				}
				if(uIsFinite(odomInfo.reg.covariance.at<double>(0,0)) &&
					odomInfo.reg.covariance.at<double>(0,0)>0.0)
				{
					if( useInputOdometryAsGuess && 
						odomInfo.reg.covariance.at<double>(0,0) >= 9999 && 
						!previousOdomPose.isNull() &&
						(pose.x() != 0.0f || pose.y() != 0.0f || pose.z() != 0.0f)) // not the first frame
					{
						// In case of external guess and auto reset, keep reporting lost till we
						// process the second frame with valid covariance. This way it
						// won't trigger a new map.
						pose = Transform();
					}
					// Use largest covariance error (to be independent of the odometry frame rate)
					else if(odomCovariance.empty() || odomInfo.reg.covariance.at<double>(0,0) > odomCovariance.at<double>(0,0))
					{
						odomCovariance = odomInfo.reg.covariance;
					}
				}
			}
			printf("Processed %d/%d frames (visual=%s lidar=%s lost=%s)... odometry = %dms\n",
					processed+1,
					totalIds,
					odomInfo.reg.matches!=0?uFormat("%d/%d", odomInfo.reg.inliers, odomInfo.reg.matches).c_str():"NA",
					odomInfo.reg.icpInliersRatio!=0.0?uNumber2Str(odomInfo.reg.icpInliersRatio).c_str():"NA",
					odomInfo.lost?"true":"false",
					int(odomInfo.timeEstimation * 1000));
			if(lastUpdateStamp > 0.0 && (data.stamp() < lastUpdateStamp + rtabmapUpdateRate || framesToSkip>0))
			{
				if(framesToSkip>0)
				{
					int skippedFrames = framesToSkip;
					while(skippedFrames-- > 0)
					{
						++processed;
						data = dbReader->takeData();
					}
				}

				if(framesToSkip==0 && intermediateNodes)
				{
					data.setId(-1); // intermediate node
				}
				else
				{
					data = dbReader->takeData(&info);
					if(scanFromDepth)
					{
						data.setLaserScan(LaserScan());
					}
					camThread.postUpdate(&data, &info);
					++processed;
					continue;
				}
			}
			info.odomPose = pose;
			info.odomCovariance = odomCovariance;
			odomCovariance = cv::Mat();

			uInsert(globalMapStats, odomInfo.statistics(pose));
		}
		else if(framesToSkip==0 && intermediateNodes && lastUpdateStamp > 0.0 && data.stamp() > lastUpdateStamp && (data.stamp() < lastUpdateStamp + rtabmapUpdateRate))
		{
			data.setId(-1); // intermediate node
		}

		if(data.id() != -1)
			lastUpdateStamp = data.stamp();

		UTimer iterationTime;
		std::string status;
		if(!odometryIgnored && info.odomPose.isNull() && incrementalMemory)
		{
			printf("Skipping node %d as it doesn't have odometry pose set.\n", data.id());
		}
		else
		{
			if(!odometryIgnored && !info.odomCovariance.empty() && info.odomCovariance.at<double>(0,0)>=9999)
			{
				printf("High variance detected, triggering a new map...\n");
				if(!incrementalMemory && processed>0)
				{
					showLocalizationStats(outputDatabasePath);
					lastLocalizationOdomPose = info.odomPose;
				}
				rtabmap.triggerNewMap();
				if(!incrementalMemory && locNull)
				{
					rtabmap.setInitialPose(Transform());
				}
				inMotion = true;
			}

			if(originalGraphAsGT)
			{
				data.setGroundTruth(gt.find(data.id()) != gt.end()?gt.at(data.id()):Transform());
			}

			// Loop closures of the input database with previous nodes, republished after the update
			std::multimap<int, Link> inputLoopClosures;
			if(republishLoopClosures && dbReader->driver())
			{
				std::multimap<int, Link> links;
				dbReader->driver()->loadLinks(data.id(), links);
				for(std::multimap<int, Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
				{
					if(iter->second.isLoopClosure() && iter->second.to() < data.id())
					{
						inputLoopClosures.insert(*iter);
					}
				}
			}
			const bool updateThresholdsIgnored = !updateThresholdsDisabled.empty() && !inputLoopClosures.empty();
			if(updateThresholdsIgnored)
			{
				rtabmap.parseParameters(updateThresholdsDisabled);
			}

			UTimer t;
			const bool processSucceeded = rtabmap.process(data, info.odomPose, info.odomCovariance, info.odomVelocity, globalMapStats);
			if(updateThresholdsIgnored)
			{
				rtabmap.parseParameters(updateThresholds);
			}
			if(!processSucceeded)
			{
				printf("Failed processing node %d.\n", data.id());
				globalMapStats.clear();
			}
			else
			{
				for(std::multimap<int, Link>::iterator iter=inputLoopClosures.begin(); iter!=inputLoopClosures.end(); ++iter)
				{
					// Both nodes should be loaded in memory (STM or WM)
					const Memory * memory = rtabmap.getMemory();
					int notLoadedId = 0;
					if(!memory->isInSTM(iter->second.to()) && !memory->isInWM(iter->second.to()))
					{
						notLoadedId = iter->second.to();
					}
					else if(!memory->isInSTM(iter->second.from()) && !memory->isInWM(iter->second.from()))
					{
						notLoadedId = iter->second.from();
					}
					if(notLoadedId && memory->getMapId(notLoadedId, true) >= 0)
					{
						printf("[Warning] Link %d->%d from input database not republished: node %d has been "
								"transferred to long-term memory (memory management).\n",
								iter->second.from(), iter->second.to(), notLoadedId);
						++loopClosuresInLTM;
					}
					else if(notLoadedId)
					{
						printf("[Warning] Link %d->%d from input database not republished: node %d has not been "
								"added to the map (e.g., skipped or merged with another node).\n",
								iter->second.from(), iter->second.to(), notLoadedId);
						++loopClosuresNotInMap;
					}
					else if(!iter->second.transform().isNull() && rtabmap.addLink(iter->second))
					{
						printf("Added link %d->%d from input database.\n", iter->second.from(), iter->second.to());
						++loopClosuresRepublished;
					}
					else
					{
						printf("[Warning] Link %d->%d from input database not republished: rejected.\n",
								iter->second.from(), iter->second.to());
						++loopClosuresRejected;
					}
				}

				if(assemble2dMap || assemble3dMap || assemble2dOctoMap || assemble3dOctoMap)
				{
					globalMapStats.clear();
					double timeRtabmap = t.ticks();
					double timeUpdateInit = 0.0;
					double timeUpdateGrid = 0.0;
					double timeUpdateCloudMap = 0.0;
#ifdef RTABMAP_OCTOMAP
					double timeUpdateOctoMap = 0.0;
#endif
					const rtabmap::Statistics & stats = rtabmap.getStatistics();
					if(stats.poses().size() && stats.getLastSignatureData().id())
					{
						int id = stats.poses().rbegin()->first;
						if(id == stats.getLastSignatureData().id() &&
						   stats.getLastSignatureData().sensorData().gridCellSize() > 0.0f)
						{
							bool updateGridMap = false;
							bool updateOctoMap = false;
							bool updateCloudMap = false;
							if(assemble2dMap && grid.addedNodes().find(id) == grid.addedNodes().end())
							{
								updateGridMap = true;
							}
							if(assemble3dMap && cloudMap.addedNodes().find(id) == cloudMap.addedNodes().end())
							{
								updateCloudMap = true;
							}
#ifdef RTABMAP_OCTOMAP
							if((assemble2dOctoMap || assemble3dOctoMap) && octomap.addedNodes().find(id) == octomap.addedNodes().end())
							{
								updateOctoMap = true;
							}
#endif
							if(updateGridMap || updateOctoMap || updateCloudMap)
							{
								cv::Mat ground, obstacles, empty;
								stats.getLastSignatureData().sensorData().uncompressDataConst(0, 0, 0, 0, &ground, &obstacles, &empty);
								float cellSize = stats.getLastSignatureData().sensorData().gridCellSize();
								const cv::Point3f & viewpoint = stats.getLastSignatureData().sensorData().gridViewPoint();

								timeUpdateInit = t.ticks();

								mapCache.add(id, ground, obstacles, empty, cellSize, viewpoint);

								if(updateGridMap)
								{
									grid.update(stats.poses());
									timeUpdateGrid = t.ticks() + timeUpdateInit;
								}
								if(updateCloudMap)
								{
										cloudMap.update(stats.poses());
									timeUpdateCloudMap = t.ticks() + timeUpdateInit;
								}
#ifdef RTABMAP_OCTOMAP
								if(updateOctoMap)
								{
									octomap.update(stats.poses());
									timeUpdateOctoMap = t.ticks() + timeUpdateInit;
								}
#endif
							}
						}
					}

					globalMapStats.insert(std::make_pair(std::string("GlobalGrid/GridUpdate/ms"), timeUpdateGrid*1000.0f));
					globalMapStats.insert(std::make_pair(std::string("GlobalGrid/CloudUpdate/ms"), timeUpdateCloudMap*1000.0f));
#ifdef RTABMAP_OCTOMAP
					//Simulate publishing
					double timePub2dOctoMap = 0.0;
					double timePub3dOctoMap = 0.0;
					if(assemble2dOctoMap)
					{
						float xMin, yMin, size;
						octomap.createProjectionMap(xMin, yMin, size);
						timePub2dOctoMap = t.ticks();
					}
					if(assemble3dOctoMap)
					{
						octomap.createCloud();
						timePub3dOctoMap = t.ticks();
					}

					globalMapStats.insert(std::make_pair(std::string("GlobalGrid/OctoMapUpdate/ms"), timeUpdateOctoMap*1000.0f));
					globalMapStats.insert(std::make_pair(std::string("GlobalGrid/OctoMapProjection/ms"), timePub2dOctoMap*1000.0f));
					globalMapStats.insert(std::make_pair(std::string("GlobalGrid/OctomapToCloud/ms"), timePub3dOctoMap*1000.0f));
					globalMapStats.insert(std::make_pair(std::string("GlobalGrid/TotalWithRtabmap/ms"), (timeUpdateGrid+timeUpdateCloudMap+timeUpdateOctoMap+timePub2dOctoMap+timePub3dOctoMap+timeRtabmap)*1000.0f));
#else
					globalMapStats.insert(std::make_pair(std::string("GlobalGrid/TotalWithRtabmap/ms"), (timeUpdateGrid+timeUpdateCloudMap+timeRtabmap)*1000.0f));
#endif
				}
			}
		}

		const rtabmap::Statistics & stats = rtabmap.getStatistics();
		int refId = stats.refImageId();
		bool rejected = uValue(stats.data(), rtabmap::Statistics::kLoopRejectedHypothesis(), 0.0f) != 0.0f;
		int loopId = stats.loopClosureId() > 0? stats.loopClosureId(): stats.proximityDetectionId() > 0?stats.proximityDetectionId() :0;
		int landmarkId = rejected?0:(int)uValue(stats.data(), rtabmap::Statistics::kLoopLandmark_detected(), 0.0f);
		int refMapId = stats.refImageMapId();
		++totalFrames;

		if(inMotion)
		{
			++totalFramesMotion;
		}
		if (loopId>0)
		{
			if(stats.loopClosureId()>0)
			{
				++loopCount;
			}
			else
			{
				++proxCount;
			}
			if(inMotion)
			{
				++loopCountMotion;
			}
			int loopMapId = stats.loopClosureId() > 0? stats.loopClosureMapId(): stats.proximityDetectionMapId();
			if(loopMapId != stats.refImageMapId())
			{
				++loopInter;
			}
			else
			{
				++loopIntra;
			}
			printf("[%f] Processed %d/%d nodes [id=%d map=%d graph=%d hyp=%d]... %dms %s on %d [%d]\n", data.stamp(), ++processed, totalIds, refId, refMapId, int(stats.poses().size()), int(uValue(stats.data(), Statistics::kLoopHighest_hypothesis_value())*100.0f), int(iterationTime.ticks() * 1000), stats.loopClosureId() > 0?"Loop":"Prox", loopId, loopMapId);
			if(stopOnLoopClosure)
			{
				printf("First loop closure has been detected and --stop_loop option is enabled, stop processing...\n");
				break;
			}
		}
		else if(landmarkId != 0)
		{
			printf("[%f] Processed %d/%d nodes [id=%d map=%d graph=%d hyp=%d]... %dms Loop on landmark %d\n", data.stamp(), ++processed, totalIds, refId, refMapId, int(stats.poses().size()), int(uValue(stats.data(), Statistics::kLoopHighest_hypothesis_value())*100.0f),  int(iterationTime.ticks() * 1000), landmarkId);
		}
		else if(data.id() == -1)
		{
			printf("[%f] Processed %d/%d nodes [id=%d map=%d graph=%d hyp=%d]... %dms Intermediate node\n", data.stamp(), ++processed, totalIds, refId, refMapId, int(stats.poses().size()), int(uValue(stats.data(), Statistics::kLoopHighest_hypothesis_value())*100.0f),  int(iterationTime.ticks() * 1000));
		}
		else
		{
			printf("[%f] Processed %d/%d nodes [id=%d map=%d graph=%d hyp=%d]... %dms\n", data.stamp(), ++processed, totalIds, refId, refMapId, int(stats.poses().size()), int(uValue(stats.data(), Statistics::kLoopHighest_hypothesis_value())*100.0f), int(iterationTime.ticks() * 1000));
		}

		// Here we accumulate statistics about distance from last localization
		if(!incrementalMemory &&
		   !lastLocalizationOdomPose.isNull() &&
		   !info.odomPose.isNull())
		{
			if(loopId>0 || landmarkId != 0)
			{
				previousLocalizationDistances.push_back(lastLocalizationOdomPose.getDistance(info.odomPose));
				lastLocalizationOdomPose = info.odomPose;
			}
		}
		if(!incrementalMemory)
		{
			float totalTime = uValue(stats.data(), rtabmap::Statistics::kTimingTotal(), 0.0f);
			localizationTime.push_back(totalTime);
			if(stats.data().find(Statistics::kLoopOdom_correction_norm()) != stats.data().end())
			{
				localizationVariations.push_back(stats.data().at(Statistics::kLoopOdom_correction_norm()));
				localizationAngleVariations.push_back(stats.data().at(Statistics::kLoopOdom_correction_angle()));
			}

			if(exportPoses)
			{
				if(!info.odomPose.isNull())
				{
					if(!odomTrajectoryPoses.empty())
					{
						int previousId = odomTrajectoryPoses.rbegin()->first;
						odomTrajectoryLinks.insert(std::make_pair(previousId, Link(previousId, refId, Link::kNeighbor, odomTrajectoryPoses.rbegin()->second.inverse()*info.odomPose, info.odomCovariance)));
					}
					odomTrajectoryPoses.insert(std::make_pair(refId, info.odomPose));
					localizationPoses.insert(std::make_pair(refId, stats.mapCorrection()*info.odomPose));
				}
				else
				{
					localizationPoses.insert(std::make_pair(refId, rtabmap.getLastLocalizationPose()));
				}
			}
		}

		Transform odomPose = info.odomPose;

		if(framesToSkip>0 && !recomputeOdometry)
		{
			int skippedFrames = framesToSkip;
			while(skippedFrames-- > 0)
			{
				processed++;
				data = dbReader->takeData(&info);
				if(!odometryIgnored && !info.odomCovariance.empty() && info.odomCovariance.at<double>(0,0)>=9999)
				{
					printf("High variance detected, triggering a new map...\n");
					if(!incrementalMemory && processed>0)
					{
						showLocalizationStats(outputDatabasePath);
						lastLocalizationOdomPose = info.odomPose;
					}
					rtabmap.triggerNewMap();
				}
			}
		}

		data = dbReader->takeData(&info);
		if(scanFromDepth)
		{
			data.setLaserScan(LaserScan());
		}
		camThread.postUpdate(&data, &info);

		inMotion = true;
		if(!incrementalMemory &&
		   !odomPose.isNull() &&
		   !info.odomPose.isNull())
		{
			float distance = odomPose.getDistance(info.odomPose);
			float angle = odomPose.getAngle(info.odomPose);
			odomDistances.push_back(distance);
			if(distance < linearUpdate && angle <= angularUpdate)
			{
				inMotion = false;
			}
		}
	}

	if(!incrementalMemory)
	{
		showLocalizationStats(outputDatabasePath);
	}
	else
	{
		printf("Total loop closures = %d (Loop=%d, Prox=%d, In Motion=%d/%d, Intra=%d, Inter=%d)\n",
			loopCount+proxCount, loopCount, proxCount, loopCountMotion, totalFramesMotion, loopIntra, loopInter);

		if(rgbdEnabled && incrementalMemory)
		{
			std::map<int, Transform> poses;
			std::multimap<int, Link> constraints;
			rtabmap.getGraph(poses, constraints, /*optimized*/ false, /*global*/ true, 0, false, false, false, false, false, false);
			std::set<int> mapIds;
			std::set<int> newMapIds;
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				int id;
				if((id=rtabmap.getMemory()->getMapId(iter->first, true))>=0)
				{
					mapIds.insert(id);
					if(id>firstDbLastMapId)
					{
						newMapIds.insert(id);
					}
				}
			}
			printf("Sessions linked to the last pose: %ld/%d\n", mapIds.size(), firstDbMapIds + newDbMapIds);
			if(appendMode)
			{
				printf("New sessions connected to the global graph: %ld/%d\n", newMapIds.size(), newDbMapIds);
				// Check if all new sessions were appended to the original graph.
				if((int)newMapIds.size() != newDbMapIds)
				{
					if(abortDisconnectedSessions) {
						printf("[Error] Not all new sessions are connected (option -abort_disconnected_sessions is set).\n");
						return 1;
					}
					printf("[Warning] Not all new sessions are connected. Add option -abort_disconnected_sessions to report this as an error.\n");
				}
				if(mapIds.size() == newMapIds.size())
				{
					if(abortDisconnectedSessions) {
						printf("[Error] The new sessions are not connected to original map (option -abort_disconnected_sessions is set).\n");
						return 1;
					}
					printf("[Warning] The new sessions are not connected to original map. Add option -abort_disconnected_sessions to report this as an error.\n");
				}
			}
			else // we reprocessed everything
			{
				if((int)mapIds.size() != firstDbMapIds + newDbMapIds)
				{
					if(abortDisconnectedSessions) {
						printf("[Error] Not all sessions are connected (option -abort_disconnected_sessions is set).\n");
						return 1;
					}
					printf("[Warning] Not all sessions are connected. Add option -abort_disconnected_sessions to report this as an error.\n");
				}
			}
		}
	}

	if(!parametersToRestore.empty())
	{
		printf("Restoring original parameters before saving the output database:\n");
		for(ParametersMap::iterator iter=parametersToRestore.begin(); iter!=parametersToRestore.end(); ++iter)
		{
			printf("  %s %s\n", uPad(iter->first+" =", 25).c_str(), iter->second.c_str());
		}
		rtabmap.parseParameters(parametersToRestore);
	}

	bool originalGraphKept = false;
	if(keepOptimizedGraph)
	{
		// The original optimized graph is saved back only if the reprocessed graph has
		// the same nodes and links. Otherwise, if the graph was not optimized while
		// reprocessing, optimize it now so that the output database doesn't keep a
		// graph not optimized.
		std::shared_ptr<DBDriver> inputDriver(DBDriver::create());
		std::set<int> originalIds;
		std::multimap<int, Link> originalLinks;
		if(inputDriver->openConnection(singleInputDatabasePath, false, true))
		{
			std::map<int, Transform> originalPoses = inputDriver->loadOptimizedPoses();
			for(std::map<int, Transform>::iterator iter=originalPoses.lower_bound(1); iter!=originalPoses.end(); ++iter)
			{
				originalIds.insert(iter->first);
			}
			inputDriver->getAllLinks(originalLinks, true, false);
			inputDriver->closeConnection(false);
		}
		std::set<int> reprocessedIds;
		for(std::map<int, Transform>::const_iterator iter=rtabmap.getLocalOptimizedPoses().lower_bound(1); iter!=rtabmap.getLocalOptimizedPoses().end(); ++iter)
		{
			reprocessedIds.insert(iter->first);
		}
		std::string differences;
		if(originalIds.empty())
		{
			differences = "the input database doesn't have an optimized graph";
		}
		else
		{
			// The neighbor links are recomputed from the odometry poses (and refined if
			// RGBD/NeighborLinkRefining is enabled), the ones of the input database are
			// restored with the original optimized graph.
			differences = compareGraphs(originalIds, originalLinks, reprocessedIds, rtabmap.getLocalConstraints(), true);
		}
		originalGraphKept = differences.empty();
		if(originalGraphKept)
		{
			printf("-keep_opt_graph: The reprocessed graph has the same nodes and links as the original optimized graph (%d nodes).\n", (int)originalIds.size());
		}
		else if(graphOptimizedWhileReprocessing)
		{
			printf("[Warning] -keep_opt_graph: The original optimized graph cannot be kept, %s. Keeping the reprocessed one.\n", differences.c_str());
		}
		else
		{
			printf("[Warning] -keep_opt_graph: The original optimized graph cannot be kept, %s. Optimizing the reprocessed graph...\n", differences.c_str());
			std::map<int, Transform> poses;
			std::multimap<int, Link> constraints;
			// The poses of the local graph are not optimized (each session in its own
			// odometry frame): clear them so that they are not used as initial guess,
			// the poses are then recomputed from the links.
			rtabmap.setOptimizedPoses(std::map<int, Transform>(), std::multimap<int, Link>());
			rtabmap.getGraph(poses, constraints, /*optimized*/ true, /*global*/ true);
			const Memory * memory = rtabmap.getMemory();
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end();)
			{
				if(iter->first > 0 && !memory->isInWM(iter->first) && !memory->isInSTM(iter->first))
				{
					poses.erase(iter++); // only nodes loaded in memory can be in the optimized graph
				}
				else
				{
					++iter;
				}
			}
			for(std::multimap<int, Link>::iterator iter=constraints.begin(); iter!=constraints.end();)
			{
				if(poses.find(iter->second.from()) == poses.end() || poses.find(iter->second.to()) == poses.end())
				{
					constraints.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
			if(poses.empty())
			{
				printf("[Error] -keep_opt_graph: Optimization of the reprocessed graph failed, the output optimized graph is not optimized.\n");
			}
			else
			{
				rtabmap.setOptimizedPoses(poses, constraints);
				printf("-keep_opt_graph: Optimizing the reprocessed graph... done (%d nodes).\n", (int)poses.size());
			}
		}
	}

	if(republishLoopClosures)
	{
		const int total = loopClosuresRepublished + loopClosuresInLTM + loopClosuresNotInMap + loopClosuresRejected;
		printf("%sRepublished %d/%d loop closures from input database (%d with a node in long-term memory, "
				"%d with a node not added to the map, %d rejected).\n",
				loopClosuresRepublished<total?"[Warning] ":"",
				loopClosuresRepublished, total, loopClosuresInLTM, loopClosuresNotInMap, loopClosuresRejected);
	}

	printf("Closing database \"%s\"...\n", outputDatabasePath.c_str());
	rtabmap.close(true);
	printf("Closing database \"%s\"... done!\n", outputDatabasePath.c_str());

	delete odometry;

	if(assemble2dMap)
	{
		std::string outputPath = outputDatabasePath.substr(0, outputDatabasePath.size()-3) + "_map.pgm";
		float xMin,yMin;
		cv::Mat map = grid.getMap(xMin, yMin);
		if(!map.empty())
		{
			if(save2DMap)
			{
				std::shared_ptr<DBDriver> driver(DBDriver::create());
				if(driver->openConnection(outputDatabasePath))
				{
					driver->save2DMap(map, xMin, yMin, grid.getCellSize());
					printf("Saving occupancy grid to database... done!\n");
				}
			}
			else
			{
				cv::Mat map8U(map.rows, map.cols, CV_8U);
				//convert to gray scaled map
				for (int i = 0; i < map.rows; ++i)
				{
					for (int j = 0; j < map.cols; ++j)
					{
						char v = map.at<char>(i, j);
						unsigned char gray;
						if(v == 0)
						{
							gray = 178;
						}
						else if(v == 100)
						{
							gray = 0;
						}
						else // -1
						{
							gray = 89;
						}
						map8U.at<unsigned char>(i, j) = gray;
					}
				}

				if(cv::imwrite(outputPath, map8U))
				{
					printf("Saving occupancy grid \"%s\"... done!\n", outputPath.c_str());
				}
				else
				{
					printf("Saving occupancy grid \"%s\"... failed!\n", outputPath.c_str());
				}
			}
		}
		else
		{
			printf("2D map is empty! Cannot save it!\n");
		}
	}
	if(assemble3dMap)
	{
		std::string outputPath = outputDatabasePath.substr(0, outputDatabasePath.size()-3) + "_obstacles.pcd";
		if(pcl::io::savePCDFileBinary(outputPath, *cloudMap.getMapObstacles()) == 0)
		{
			printf("Saving 3d obstacles \"%s\"... done!\n", outputPath.c_str());
		}
		else
		{
			printf("Saving 3d obstacles \"%s\"... failed!\n", outputPath.c_str());
		}
		if(cloudMap.getMapGround()->size())
		{
			outputPath = outputDatabasePath.substr(0, outputDatabasePath.size()-3) + "_ground.pcd";
			if(pcl::io::savePCDFileBinary(outputPath, *cloudMap.getMapGround()) == 0)
			{
				printf("Saving 3d ground \"%s\"... done!\n", outputPath.c_str());
			}
			else
			{
				printf("Saving 3d ground \"%s\"... failed!\n", outputPath.c_str());
			}
		}
		if(cloudMap.getMapEmptyCells()->size())
		{
			outputPath = outputDatabasePath.substr(0, outputDatabasePath.size()-3) + "_empty.pcd";
			if(pcl::io::savePCDFileBinary(outputPath, *cloudMap.getMapEmptyCells()) == 0)
			{
				printf("Saving 3d empty cells \"%s\"... done!\n", outputPath.c_str());
			}
			else
			{
				printf("Saving 3d empty cells \"%s\"... failed!\n", outputPath.c_str());
			}
		}
	}
#ifdef RTABMAP_OCTOMAP
	if(assemble2dOctoMap)
	{
		std::string outputPath = outputDatabasePath.substr(0, outputDatabasePath.size()-3) + "_octomap.pgm";
		float xMin,yMin,cellSize;
		cv::Mat map = octomap.createProjectionMap(xMin, yMin, cellSize);
		if(!map.empty())
		{
			if(save2DMap)
			{
				std::shared_ptr<DBDriver> driver(DBDriver::create());
				if(driver->openConnection(outputDatabasePath))
				{
					driver->save2DMap(map, xMin, yMin, cellSize);
					printf("Saving occupancy grid to database... done!\n");
				}
			}
			else
			{
				cv::Mat map8U(map.rows, map.cols, CV_8U);
				//convert to gray scaled map
				for (int i = 0; i < map.rows; ++i)
				{
					for (int j = 0; j < map.cols; ++j)
					{
						char v = map.at<char>(i, j);
						unsigned char gray;
						if(v == 0)
						{
							gray = 178;
						}
						else if(v == 100)
						{
							gray = 0;
						}
						else // -1
						{
							gray = 89;
						}
						map8U.at<unsigned char>(i, j) = gray;
					}
				}
				if(cv::imwrite(outputPath, map8U))
				{
					printf("Saving octomap 2D projection \"%s\"... done!\n", outputPath.c_str());
				}
				else
				{
					printf("Saving octomap 2D projection \"%s\"... failed!\n", outputPath.c_str());
				}
			}
		}
		else
		{
			printf("OctoMap 2D projection map is empty! Cannot save it!\n");
		}
	}
	if(assemble3dOctoMap)
	{
		std::string outputPath = outputDatabasePath.substr(0, outputDatabasePath.size()-3) + "_octomap_occupied.pcd";
		std::vector<int> obstacles, emptySpace, ground;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = octomap.createCloud(0, &obstacles, &emptySpace, &ground);
		if(pcl::io::savePCDFile(outputPath, *cloud, obstacles, true) == 0)
		{
			printf("Saving obstacles cloud \"%s\"... done!\n", outputPath.c_str());
		}
		else
		{
			printf("Saving obstacles cloud \"%s\"... failed!\n", outputPath.c_str());
		}
		if(ground.size())
		{
			outputPath = outputDatabasePath.substr(0, outputDatabasePath.size()-3) + "_octomap_ground.pcd";
			if(pcl::io::savePCDFile(outputPath, *cloud, ground, true) == 0)
			{
				printf("Saving empty space cloud \"%s\"... done!\n", outputPath.c_str());
			}
			else
			{
				printf("Saving empty space cloud \"%s\"... failed!\n", outputPath.c_str());
			}
		}
		if(emptySpace.size())
		{
			outputPath = outputDatabasePath.substr(0, outputDatabasePath.size()-3) + "_octomap_empty.pcd";
			if(pcl::io::savePCDFile(outputPath, *cloud, emptySpace, true) == 0)
			{
				printf("Saving empty space cloud \"%s\"... done!\n", outputPath.c_str());
			}
			else
			{
				printf("Saving empty space cloud \"%s\"... failed!\n", outputPath.c_str());
			}
		}
	}
#endif

	if(keepOptimizedGraph)
	{
		// Put back the original optimized graph (and what was generated from it),
		// so that the output map stays exactly aligned with the input one. Only
		// if the reprocessed graph has the same nodes, otherwise it would not
		// match the output database anymore.
		std::shared_ptr<DBDriver> inputDriver(DBDriver::create());
		std::shared_ptr<DBDriver> outputDriver(DBDriver::create());
		if(!inputDriver->openConnection(singleInputDatabasePath, false, true))
		{
			printf("[Error] -keep_opt_graph: Failed opening input database \"%s\".\n", singleInputDatabasePath.c_str());
			return 1;
		}
		if(!outputDriver->openConnection(outputDatabasePath, false))
		{
			printf("[Error] -keep_opt_graph: Failed opening output database \"%s\".\n", outputDatabasePath.c_str());
			return 1;
		}
		Transform lastLocalizationPose;
		std::map<int, Transform> inputPoses = inputDriver->loadOptimizedPoses(&lastLocalizationPose);
		if(!originalGraphKept)
		{
			printf("[Warning] -keep_opt_graph: Keeping the reprocessed optimized graph and map (see warning above).\n");
		}
		else
		{
			outputDriver->saveOptimizedPoses(inputPoses, lastLocalizationPose);
			printf("-keep_opt_graph: Saved back the original optimized graph (%d nodes) and last localization pose.\n", (int)inputPoses.size());

			float xMin=0.0f, yMin=0.0f, cellSize=0.0f;
			cv::Mat map = inputDriver->load2DMap(xMin, yMin, cellSize);
			if(!map.empty())
			{
				outputDriver->save2DMap(map, xMin, yMin, cellSize);
				printf("-keep_opt_graph: Saved back the original 2D map (%dx%d, cell=%fm)%s.\n",
						map.cols, map.rows, cellSize,
						save2DMap?", replacing the one assembled with -db":"");
			}

			std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > polygons;
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
			std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > texCoords;
#else
			std::vector<std::vector<Eigen::Vector2f> > texCoords;
#endif
			cv::Mat textures;
			cv::Mat cloud = inputDriver->loadOptimizedMesh(&polygons, &texCoords, &textures);
			if(!cloud.empty())
			{
				outputDriver->saveOptimizedMesh(cloud, polygons, texCoords, textures);
				printf("-keep_opt_graph: Saved back the original optimized mesh.\n");
			}

			// The neighbor links of the input database may have been refined (e.g.,
			// RGBD/NeighborLinkRefining or post-processing), the original optimized graph
			// was optimized with them. They may be saved in one direction only.
			std::multimap<int, Link> inputLinks;
			std::multimap<int, Link> outputLinks;
			inputDriver->getAllLinks(inputLinks, true, false);
			outputDriver->getAllLinks(outputLinks, true, false);
			std::map<std::pair<int, int>, Link> inputNeighborLinks;
			for(std::multimap<int, Link>::iterator iter=inputLinks.begin(); iter!=inputLinks.end(); ++iter)
			{
				if(iter->second.type() == Link::kNeighbor)
				{
					inputNeighborLinks.insert(std::make_pair(std::make_pair(iter->second.from(), iter->second.to()), iter->second));
				}
			}
			int neighborLinksRestored = 0;
			outputDriver->executeNoResult("BEGIN TRANSACTION;");
			for(std::multimap<int, Link>::iterator iter=outputLinks.begin(); iter!=outputLinks.end(); ++iter)
			{
				const Link & link = iter->second;
				if(link.type() != Link::kNeighbor)
				{
					continue;
				}
				Link inputLink;
				std::map<std::pair<int, int>, Link>::iterator jter = inputNeighborLinks.find(std::make_pair(link.from(), link.to()));
				if(jter != inputNeighborLinks.end())
				{
					inputLink = jter->second;
				}
				else if((jter = inputNeighborLinks.find(std::make_pair(link.to(), link.from()))) != inputNeighborLinks.end())
				{
					inputLink = jter->second.inverse();
				}
				if(inputLink.isValid() &&
				   (inputLink.transform() != link.transform() ||
				    inputLink.infMatrix().size() != link.infMatrix().size() ||
				    (!link.infMatrix().empty() && cv::norm(inputLink.infMatrix(), link.infMatrix(), cv::NORM_INF) > 0.0)))
				{
					outputDriver->updateLink(inputLink);
					++neighborLinksRestored;
				}
			}
			outputDriver->executeNoResult("COMMIT;");
			printf("-keep_opt_graph: Saved back the original neighbor links (%d updated).\n", neighborLinksRestored);
		}
		inputDriver->closeConnection(false);
		outputDriver->closeConnection(true);
	}

	return 0;
}
