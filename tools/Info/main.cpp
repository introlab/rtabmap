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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/utilite/UDirectory.h>
#include "rtabmap/utilite/UFile.h"

using namespace rtabmap;

#ifdef _WIN32
#include <Windows.h>
#define COLOR_NORMAL FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_RED
#define COLOR_RED FOREGROUND_RED | FOREGROUND_INTENSITY
#define COLOR_GREEN FOREGROUND_GREEN
#define COLOR_YELLOW FOREGROUND_GREEN | FOREGROUND_RED
#else
#define COLOR_NORMAL "\033[0m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#endif

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-info [options] \"map.db\"\n"
			"  Options:\n"
			"     --diff                   Show only modified parameters.\n"
			"     --diff  \"other_map.db\"   Compare parameters with other database.\n"
			"\n");
	exit(1);
}

std::string pad(const std::string & title, int padding = 20)
{
	int emptySize = padding - (int)title.size();
	if(emptySize>0)
	{
		return title + std::string(emptySize, ' ');
	}
	return title;
}

int main(int argc, char * argv[])
{
	if(argc < 2)
	{
		showUsage();
	}

	std::string otherDatabasePath;
	bool diff = false;
	for(int i=1; i<argc-1; ++i)
	{
		if(strcmp(argv[i], "--diff") == 0)
		{
			++i;
			if(i<argc-1)
			{
				otherDatabasePath = uReplaceChar(argv[i], '~', UDirectory::homeDir());
				printf("Comparing with other database \"%s\"...\n", otherDatabasePath.c_str());
			}
			diff = true;
		}
	}

	std::string databasePath = uReplaceChar(argv[argc-1], '~', UDirectory::homeDir());
	if(!UFile::exists(databasePath))
	{
		printf("Database \"%s\" doesn't exist!\n", databasePath.c_str());
		return -1;
	}

	DBDriver * driver = DBDriver::create();
	if(!driver->openConnection(databasePath))
	{
		printf("Cannot open database \"%s\".\n", databasePath.c_str());
		delete driver;
		return -1;
	}

	ParametersMap parameters = driver->getLastParameters();
	ParametersMap defaultParameters = Parameters::getDefaultParameters();
	ParametersMap removedParameters = Parameters::getBackwardCompatibilityMap();
	std::string otherDatabasePathName;
	if(!otherDatabasePath.empty())
	{
		driver->closeConnection(false);

		if(!UFile::exists(otherDatabasePath))
		{
			printf("Database \"%s\" doesn't exist!\n", otherDatabasePath.c_str());
			delete driver;
			return -1;
		}

		if(!driver->openConnection(otherDatabasePath))
		{
			printf("Cannot open database \"%s\".\n", otherDatabasePath.c_str());
			delete driver;
			return -1;
		}
		otherDatabasePathName = UFile::getName(otherDatabasePath);
		defaultParameters = driver->getLastParameters();
		removedParameters.clear();
	}

#ifdef _WIN32
	HANDLE H = GetStdHandle(STD_OUTPUT_HANDLE);
#endif
	int padding = 35;
	std::cout << ("Parameters (Yellow=modified, Red=old parameter not used anymore):\n");
	for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		ParametersMap::const_iterator jter = defaultParameters.find(iter->first);
		std::string defaultValue;
		bool defaultValueSet = false;
		if(jter == defaultParameters.end())
		{
			jter = removedParameters.find(iter->first);
			if(jter != removedParameters.end())
			{
				defaultValue = jter->second;
				defaultValueSet = true;
			}
		}
		else
		{
			defaultValue = jter->second;
			defaultValueSet = true;
		}

		if(defaultValueSet &&
		   iter->second.compare(defaultValue) != 0 &&
		   iter->first.compare(Parameters::kRtabmapWorkingDirectory()) != 0)
		{
			bool different = true;
			if(Parameters::getType(iter->first).compare("double") ==0 ||
			   Parameters::getType(iter->first).compare("float") == 0)
			{
				if(uStr2Double(iter->second) == uStr2Double(defaultValue))
				{
					different = false;
				}
			}

			if(different)
			{
				//yellow
#ifdef _WIN32
				SetConsoleTextAttribute(H,COLOR_YELLOW);
#else
				printf("%s", COLOR_YELLOW);
#endif
				std::cout << (uFormat("%s%s  (%s=%s)\n", pad(iter->first + "=", padding).c_str(), iter->second.c_str(), otherDatabasePath.empty()?"default":otherDatabasePathName.c_str(), defaultValue.c_str()));
			}
			else if(!diff)
			{
				//green
#ifdef _WIN32
				SetConsoleTextAttribute(H,COLOR_NORMAL);
#else
				printf("%s", COLOR_NORMAL);
#endif
				std::cout << (uFormat("%s%s\n", pad(iter->first + "=", padding).c_str(), iter->second.c_str()));
			}
		}
		else if(!defaultValueSet && otherDatabasePath.empty())
		{
			//red
#ifdef _WIN32
			SetConsoleTextAttribute(H,COLOR_RED);
#else
			printf("%s", COLOR_RED);
#endif
			std::cout << (uFormat("%s%s\n", pad(iter->first + "=", padding).c_str(), iter->second.c_str()));
		}
		else if(!diff)
		{
			//green
#ifdef _WIN32
			SetConsoleTextAttribute(H,COLOR_NORMAL);
#else
			printf("%s", COLOR_NORMAL);
#endif
			std::cout << (uFormat("%s%s\n", pad(iter->first + "=", padding).c_str(), iter->second.c_str()));
		}
#ifdef _WIN32
		SetConsoleTextAttribute(H,COLOR_NORMAL);
#else
		printf("%s", COLOR_NORMAL);
#endif
	}

	if(otherDatabasePath.empty())
	{
		printf("\nInfo:\n\n");
		std::string info;
		std::set<int> ids;
		driver->getAllNodeIds(ids);
		Transform lastLocalization;
		std::map<int, Transform> optimizedPoses = driver->loadOptimizedPoses(&lastLocalization);
		std::set<int> mapsLinkedToLastGraph;
		int lastMapId=0;
		double previousStamp = 0.0f;
		Transform previousPose;
		float infoTotalOdom = 0.0f;
		double infoTotalTime = 0.0f;
		int sessions = !ids.empty()?1:0;
		int odomPoses = 0;
		int gtPoses = 0;
		int gpsValues = 0;
		for(std::set<int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
		{
			Transform p, g;
			int w;
			std::string l;
			double s;
			int mapId;
			std::vector<float> v;
			GPS gps;
			EnvSensors sensors;
			int id = *iter;
			driver->getNodeInfo(id, p, mapId, w, l, s, g, v, gps, sensors);
			if(!p.isNull())
			{
				++odomPoses;
			}
			if(!g.isNull())
			{
				++gtPoses;
			}
			if(gps.stamp()>0.0)
			{
				++gpsValues;
			}
			if(optimizedPoses.find(id) != optimizedPoses.end())
			{
				mapsLinkedToLastGraph.insert(mapId);
			}
			if(iter!=ids.begin())
			{
				if(lastMapId == mapId)
				{
					if(!p.isNull() && !previousPose.isNull())
					{
						infoTotalOdom += p.getDistance(previousPose);
					}

					if(previousStamp > 0.0 && s > 0.0)
					{
						infoTotalTime += s-previousStamp;
					}
				}
				else
				{
					++sessions;
				}
			}
			lastMapId = mapId;
			previousStamp=s;
			previousPose=p;
		}
		std::cout << (uFormat("%s%s\n", pad("Path:").c_str(), driver->getUrl().c_str()));
		std::cout << (uFormat("%s%s\n", pad("Version:").c_str(), driver->getDatabaseVersion().c_str()));
		std::cout << (uFormat("%s%d\n", pad("Sessions:").c_str(), sessions));
		std::multimap<int, Link> links;
		driver->getAllLinks(links, true, true);
		bool reducedGraph = false;
		std::vector<int> linkTypes(Link::kEnd, 0);
		for(std::multimap<int, Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
		{
			if(iter->second.type() == Link::kNeighborMerged)
			{
				reducedGraph = true;
			}
			if(iter->second.type()>=0 && iter->second.type()<Link::kEnd)
			{
				++linkTypes[iter->second.type()];
			}
		}
		if(reducedGraph)
		{
			std::cout << (uFormat("%s%f m (approx. as graph has been reduced)\n", pad("Total odom:").c_str(), infoTotalOdom));
		}
		else
		{
			std::cout << (uFormat("%s%f m\n", pad("Total odometry length:").c_str(), infoTotalOdom));
		}

		std::stringstream sessionsInOptGraphStr;
		for(std::set<int>::iterator iter=mapsLinkedToLastGraph.begin(); iter!=mapsLinkedToLastGraph.end(); ++iter)
		{
			if(iter!=mapsLinkedToLastGraph.begin())
			{
				sessionsInOptGraphStr << ", ";
			}
			sessionsInOptGraphStr << *iter;
		}

		std::cout << (uFormat("%s%fs\n", pad("Total time:").c_str(), infoTotalTime));
		std::cout << (uFormat("%s%d nodes and %d words\n", pad("LTM:").c_str(), (int)ids.size(), driver->getTotalDictionarySize()));
		std::cout << (uFormat("%s%d nodes and %d words\n", pad("WM:").c_str(), driver->getLastNodesSize(), driver->getLastDictionarySize()));
		std::cout << (uFormat("%s%d poses and %d links\n", pad("Global graph:").c_str(), odomPoses, links.size()));
		std::cout << (uFormat("%s%d poses\n", pad("Optimized graph:").c_str(), (int)optimizedPoses.size(), links.size()));
		std::cout << (uFormat("%s%d/%d [%s]\n", pad("Maps in graph:").c_str(), (int)mapsLinkedToLastGraph.size(), sessions, sessionsInOptGraphStr.str().c_str()));
		std::cout << (uFormat("%s%d poses\n", pad("Ground truth:").c_str(), gtPoses));
		std::cout << (uFormat("%s%d poses\n", pad("GPS:").c_str(), gpsValues));
		std::cout << (uFormat("Links:\n"));
		for(size_t i=0; i<linkTypes.size(); ++i)
		{
			std::cout << (uFormat("%s%d\n", pad(uFormat("  %s:", Link::typeName((Link::Type)i).c_str())).c_str(), linkTypes[i]));
		}
		std::cout << ("\n");
		long total = 0;
		long dbSize = UFile::length(driver->getUrl());
		long mem = dbSize;
		std::cout << (uFormat("%s%d %s\n", pad("Database size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes"));
		mem = driver->getNodesMemoryUsed();
		total+=mem;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("Nodes size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		mem = driver->getLinksMemoryUsed();
		total+=mem;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("Links size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		mem = driver->getImagesMemoryUsed();
		total+=mem;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("RGB Images size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		mem = driver->getDepthImagesMemoryUsed();
		total+=mem;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("Depth Images size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		mem = driver->getCalibrationsMemoryUsed();
		total+=mem;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("Calibrations size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		mem = driver->getGridsMemoryUsed();
		total+=mem;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("Grids size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		mem = driver->getLaserScansMemoryUsed();
		total+=mem;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("Scans size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		mem = driver->getUserDataMemoryUsed();
		total+=mem;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("User data size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		mem = driver->getWordsMemoryUsed();
		total+=mem;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("Dictionary size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		mem = driver->getFeaturesMemoryUsed();
		total+=mem;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("Features size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		mem = driver->getStatisticsMemoryUsed();
		total+=mem;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("Statistics size:").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		mem = dbSize - total;
		std::cout << (uFormat("%s%d %s\t(%.2f%%)\n", pad("Other (indexing):").c_str(), mem>1000000?mem/1000000:mem>1000?mem/1000:mem, mem>1000000?"MB":mem>1000?"KB":"Bytes", dbSize>0?double(mem)/double(dbSize)*100.0:0.0));
		std::cout << ("\n");
	}

	return 0;
}
