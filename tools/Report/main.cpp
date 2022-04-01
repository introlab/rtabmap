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
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/ULogger.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#ifdef WITH_QT
#include <rtabmap/utilite/UPlot.h>
#include <QApplication>
#include <QFile>
#endif

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-report [\"Statistic/Id\"] [options] path\n"
#ifndef WITH_QT
			"[Not built with Qt, statistics cannot be plotted]\n"
#endif
			"  path               Directory containing rtabmap databases or path of a database.\n"
			"  Options:"
			"    --latex            Print table formatted in LaTeX with results.\n"
			"    --kitti            Compute error based on KITTI benchmark.\n"
			"    --relative         Compute relative motion error between poses.\n"
			"    --loop             Compute relative motion error of loop closures.\n"
			"    --scale            Find the best scale for the map against the ground truth\n"
			"                         and compute error based on the scaled path.\n"
			"    --poses            Export poses to [path]_poses.txt, ground truth to [path]_gt.txt\n"
			"                         and valid ground truth indices to [path]_indices.txt \n"
			"    --inc              Incremental optimization. \n"
			"    --stats            Show available statistics \"Statistic/Id\" to plot or get localization statistics (if path is a file). \n"
#ifdef WITH_QT
			"    --invert           When reading many databases, put all curves from a same \n"
			"                       database in same figure, instead of all same curves from \n"
			"                       different database in same figure. When reading a single \n"
			"                       database, the inverse will be done. \n"
			"    --ids              Use IDs for x axis instead of time in the figures. \n"
			"    --start #          Start from this node ID for the figures.\n"
			"    --export           Export figures' data to txt files.\n"
			"    --export_prefix    Prefix to filenames of exported figures' data (default is \"Stat\").\n"
#endif
			"    --report           Export all evaluation statistics values in report.txt \n"
			"    --loc [#]          Show localization statistics for each \"Statistic/Id\" per\n"
			"                       session. Optionally set number 1=min,2=max,4=mean,8=stddev,16=total,32=nonnull%%\n"
			"                       to show cumulative results on console.\n"
			"    --loc_delay #      Delay to split sessions for localization statistics (default 60 seconds)\n"
			"                       (it is a mask, we can combine those numbers, e.g., 63 for all) \n"
			"    --ignore_inter_nodes  Ignore intermediate poses and statistics.\n"
			"    --udebug           Show debug log.\n"
			"    --help,-h          Show usage\n\n");
	exit(1);
}

struct LocStats
{
	static LocStats from(const std::vector<float> & array)
	{
		LocStats values;
		values.mean = uMean(array);
		values.stddev = std::sqrt(uVariance(array, values.mean));
		uMinMax(array, values.min, values.max);
		values.total = array.size();
		values.nonNull = 0.0f;
		if(!array.empty())
		{
			for(size_t j=0; j<array.size(); ++j)
			{
				if(array[j] != 0)
				{
					values.nonNull+=1.0f;
				}
			}
			values.nonNull = values.nonNull/float(array.size());
		}
		return values;
	}
	float mean;
	float stddev;
	float min;
	float max;
	int total;
	float nonNull;
};

int main(int argc, char * argv[])
{
	if(argc < 2)
	{
		showUsage();
	}

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);
#ifdef WITH_QT
	QApplication app(argc, argv);
#endif

	bool outputLatex = false;
	bool outputScaled = false;
	bool outputPoses = false;
	bool outputKittiError = false;
	bool outputRelativeError = false;
	bool outputReport = false;
	bool outputLoopAccuracy = false;
	bool incrementalOptimization = false;
	bool showAvailableStats = false;
	bool invertFigures = false;
	bool ignoreInterNodes = false;
	bool useIds = false;
	int startId = 0;
	bool exportFigures = false;
	std::string exportPrefix = "Stat";
	int showLoc = 0;
	float locDelay = 60;
	std::vector<std::string> statsToShow;
#ifdef WITH_QT
	std::map<std::string, UPlot*> figures;
#endif
	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "--h") == 0)
		{
			showUsage();
		}
		else if(strcmp(argv[i], "--udebug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
		}
		else if(strcmp(argv[i], "--latex") == 0)
		{
			outputLatex = true;
		}
		else if(strcmp(argv[i], "--kitti") == 0)
		{
			outputKittiError = true;
		}
		else if(strcmp(argv[i], "--relative") == 0)
		{
			outputRelativeError = true;
		}
		else if(strcmp(argv[i], "--scale") == 0)
		{
			outputScaled = true;
		}
		else if(strcmp(argv[i], "--poses") == 0)
		{
			outputPoses = true;
		}
		else if(strcmp(argv[i], "--loop") == 0)
		{
			outputLoopAccuracy = true;
		}
		else if(strcmp(argv[i],"--report") == 0)
		{
			outputReport = true;
		}
		else if(strcmp(argv[i],"--inc") == 0)
		{
			incrementalOptimization = true;
		}
		else if(strcmp(argv[i],"--stats") == 0)
		{
			showAvailableStats = true;
		}
		else if(strcmp(argv[i],"--invert") == 0)
		{
			invertFigures = true;
		}
		else if(strcmp(argv[i],"--ids") == 0)
		{
			useIds = true;
		}
		else if(strcmp(argv[i],"--ignore_inter_nodes") == 0)
		{
			ignoreInterNodes = true;
		}
		else if(strcmp(argv[i],"--export") == 0)
		{
			exportFigures = true;
		}
		else if(strcmp(argv[i],"--export_prefix") == 0)
		{
			++i;
			if(i<argc-1)
			{
				exportPrefix = argv[i];
				printf("Export prefix=%s (--export_prefix)\n", exportPrefix.c_str());
			}
			else
			{
				printf("Missing value for \"--export_prefix\" option.\n");
				showUsage();
			}
		}
		else if(strcmp(argv[i],"--loc") == 0)
		{
			++i;
			if(i<argc-1)
			{
				if(uIsNumber(argv[i]))
				{
					showLoc = atoi(argv[i]);
					printf("Localization statistics=%d (--loc)\n", showLoc);
				}
				else
				{
					showLoc = -1;
					--i; // reset
					printf("Localization statistics (--loc)\n");
				}
			}
			else
			{
				printf("Missing type for \"--loc\" option.\n");
				showUsage();
			}
		}
		else if(strcmp(argv[i],"--loc_delay") == 0)
		{
			++i;
			if(i<argc-1)
			{
				locDelay = atof(argv[i]);
				printf("Localization delay=%fs (--loc_delay)\n", locDelay);
			}
			else
			{
				printf("Missing value for \"--loc_delay\" option.\n");
				showUsage();
			}
		}
		else if(strcmp(argv[i],"--start") == 0)
		{
			++i;
			if(i<argc-1)
			{
				startId = atoi(argv[i]);
				printf("Figures will be plotted from id=%d (--start)\n", startId);
			}
			else
			{
				printf("Missing id for \"--start\" option.\n");
				showUsage();
			}
		}
		else if(i<argc-1)
		{

			statsToShow.push_back(argv[i]);
		}
	}

	std::string path = argv[argc-1];
	path = uReplaceChar(path, '~', UDirectory::homeDir());

	if(!UDirectory::exists(path) && UFile::getExtension(path).compare("db") == 0)
	{
		invertFigures = !invertFigures;
	}
	std::map<std::string, std::vector<std::pair<std::string, std::vector<LocStats> > > > localizationMultiStats; //<statsName, <Database<Session>> >
	for(size_t i=0; i<statsToShow.size(); ++i)
	{
		std::string figureTitle = statsToShow[i];
		if(!invertFigures)
		{
#ifdef WITH_QT
			printf("Plot %s\n", figureTitle.c_str());
			UPlot * fig = new UPlot();
			fig->resize(QSize(640,480));
			fig->setWindowTitle(figureTitle.c_str());
			if(useIds)
			{
				fig->setXLabel("Node ID");
			}
			else
			{
				fig->setXLabel("Time (s)");
			}
			figures.insert(std::make_pair(figureTitle, fig));
#endif
		}
		if(showLoc & 0b111111)
		{
			localizationMultiStats.insert(std::make_pair(figureTitle, std::vector<std::pair<std::string, std::vector<LocStats> > >()));
		}
	}
	if(!invertFigures)
	{
		statsToShow.clear();
	}

	std::string fileName;
	std::list<std::string> paths;
	paths.push_back(path);
	std::vector<std::map<std::string, std::vector<float> > > outputLatexStatistics;
	std::map<std::string, std::vector<float> > outputLatexStatisticsMap;
	bool odomRAMSet = false;
	std::set<std::string> topDirs;
	while(paths.size())
	{
		std::string currentPath = paths.front();
		UDirectory currentDir(currentPath);
		paths.pop_front();
		bool currentPathIsDatabase = false;
		if(!currentDir.isValid())
		{
			if(UFile::getExtension(currentPath).compare("db") == 0)
			{
				currentPathIsDatabase=true;
				printf("Database: %s\n", currentPath.c_str());
			}
			else
			{
				continue;
			}
		}
		std::list<std::string> subDirs;
		if(!currentPathIsDatabase)
		{
			printf("Directory: %s\n", currentPath.c_str());
			std::list<std::string> fileNames = currentDir.getFileNames();
			if(topDirs.empty())
			{
				for(std::list<std::string>::iterator iter = fileNames.begin(); iter!=fileNames.end(); ++iter)
				{
					topDirs.insert(currentPath+"/"+*iter);
				}
			}
			else
			{
				if(topDirs.find(currentPath) != topDirs.end())
				{
					if(outputLatexStatisticsMap.size())
					{
						outputLatexStatistics.push_back(outputLatexStatisticsMap);
						outputLatexStatisticsMap.clear();
					}
				}
			}
		}

		// For all databases in currentDir
		while(currentPathIsDatabase || !(fileName = currentDir.getNextFileName()).empty())
		{
			int startIdPerDb = startId;
			if(currentPathIsDatabase || UFile::getExtension(fileName).compare("db") == 0)
			{
				std::string filePath;
				if(currentPathIsDatabase)
				{
					filePath = currentPath;
					fileName = UFile::getName(currentPath);
				}
				else
				{
					filePath = currentPath + UDirectory::separator() + fileName;
				}

				DBDriver * driver = DBDriver::create();
				ParametersMap params;
				if(driver->openConnection(filePath))
				{
					ULogger::Level logLevel = ULogger::level();
					if(ULogger::level() == ULogger::kWarning)
					{
						ULogger::setLevel(ULogger::kError); // to suppress parameter warnings
					}
					params = driver->getLastParameters();
					ULogger::setLevel(logLevel);
					std::set<int> ids;
					driver->getAllNodeIds(ids, false, false, ignoreInterNodes);
					std::map<int, std::pair<std::map<std::string, float>, double> > stats = driver->getAllStatistics();
					std::map<int, Transform> odomPoses, gtPoses;
					std::map<int, double> odomStamps;
					std::vector<float> cameraTime;
					cameraTime.reserve(ids.size());
					std::vector<float> odomTime;
					odomTime.reserve(ids.size());
					std::vector<float> slamTime;
					slamTime.reserve(ids.size());
					float rmse = -1;
					float maxRMSE = -1;
					float maxOdomRAM = -1;
					float maxMapRAM = -1;
#ifdef WITH_QT
					if(currentPathIsDatabase && showAvailableStats)
					{
						std::map<std::string, int> availableStats;
						for(std::set<int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
						{
							if(stats.find(*iter) != stats.end())
							{
								for(std::map<std::string, float>::iterator jter=stats.at(*iter).first.begin(); jter!=stats.at(*iter).first.end(); ++jter)
								{
									if(availableStats.find(jter->first) != availableStats.end())
									{
										++availableStats.at(jter->first);
									}
									else
									{
										availableStats.insert(std::make_pair(jter->first, 1));
									}
								}
							}
						}
						printf("Showing available statistics in \"%s\":\n", filePath.c_str());
						for(std::map<std::string, int>::iterator iter=availableStats.begin(); iter!=availableStats.end(); ++iter)
						{
							printf("%s (%d)\n", iter->first.c_str(), iter->second);
						}
						printf("\n");
						exit(1);
					}

					std::map<std::string, UPlotCurve*> curves;
					if(statsToShow.empty())
					{
						for(std::map<std::string, UPlot*>::iterator iter=figures.begin(); iter!=figures.end(); ++iter)
						{
							curves.insert(std::make_pair(iter->first, iter->second->addCurve(filePath.c_str())));
							if(!localizationMultiStats.empty())
								localizationMultiStats.at(iter->first).push_back(std::make_pair(fileName, std::vector<LocStats>()));
						}
					}
					else
					{
						UPlot * fig = new UPlot();
						fig->setWindowTitle(filePath.c_str());
						if(useIds)
						{
							fig->setXLabel("Node ID");
						}
						else
						{
							fig->setXLabel("Time (s)");
						}
						if(!figures.insert(std::make_pair(filePath.c_str(), fig)).second)
						{
							delete fig;
							printf("Figure %s already added!\n", filePath.c_str());
						}
						else
						{
							for(size_t i=0; i<statsToShow.size(); ++i)
							{
								curves.insert(std::make_pair(statsToShow[i], fig->addCurve(statsToShow[i].c_str())));
								if(!localizationMultiStats.empty())
									localizationMultiStats.at(statsToShow[i]).push_back(std::make_pair(fileName, std::vector<LocStats>()));
							}
						}
					}
#else
					for(size_t i=0; i<statsToShow.size(); ++i)
					{
						if(!localizationMultiStats.empty())
							localizationMultiStats.at(statsToShow[i]).push_back(std::make_pair(fileName, std::vector<LocStats>()));
					}
#endif

					// Find localization sessions and adjust startId
					std::set<int> mappingSessionIds;
					if(!localizationMultiStats.empty())
					{
						std::map<int, Transform> poses = driver->loadOptimizedPoses();
						if(!poses.empty())
						{
							for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
							{
								Transform p, gt;
								GPS gps;
								int m=-1, w=-1;
								std::string l;
								double s;
								std::vector<float> v;
								EnvSensors sensors;
								if(driver->getNodeInfo(iter->first, p, m, w, l, s, gt, v, gps, sensors))
								{
									mappingSessionIds.insert(m);
								}
							}

							if(startIdPerDb ==0)
							{
								startIdPerDb = poses.rbegin()->first+1;
							}
						}
					}

					std::map<std::string, std::vector<float> > localizationSessionStats;
					double previousStamp = 0.0;
					for(std::set<int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
					{
						Transform p, gt;
						GPS gps;
						int m=-1, w=-1;
						std::string l;
						double s;
						std::vector<float> v;
						EnvSensors sensors;
						if(driver->getNodeInfo(*iter, p, m, w, l, s, gt, v, gps, sensors))
						{
							odomPoses.insert(std::make_pair(*iter, p));
							odomStamps.insert(std::make_pair(*iter, s));
							if(!gt.isNull())
							{
								gtPoses.insert(std::make_pair(*iter, gt));
							}

							if(!localizationMultiStats.empty() && mappingSessionIds.find(m) != mappingSessionIds.end())
							{
								continue;
							}

							if(*iter >= startIdPerDb && uContains(stats, *iter))
							{
								const std::map<std::string, float> & stat = stats.at(*iter).first;
								if(uContains(stat, Statistics::kGtTranslational_rmse()))
								{
									rmse = stat.at(Statistics::kGtTranslational_rmse());
									if(maxRMSE==-1 || maxRMSE < rmse)
									{
										maxRMSE = rmse;
									}
								}
								if(uContains(stat, std::string("Camera/TotalTime/ms")))
								{
									cameraTime.push_back(stat.at(std::string("Camera/TotalTime/ms")));
								}
								if(uContains(stat, std::string("Odometry/TotalTime/ms")))
								{
									odomTime.push_back(stat.at(std::string("Odometry/TotalTime/ms")));
								}
								else if(uContains(stat, std::string("Odometry/TimeEstimation/ms")))
								{
									odomTime.push_back(stat.at(std::string("Odometry/TimeEstimation/ms")));
								}

								if(uContains(stat, std::string("RtabmapROS/TotalTime/ms")))
								{
									if(w!=-1)
									{
										slamTime.push_back(stat.at("RtabmapROS/TotalTime/ms"));
									}
								}
								else if(uContains(stat, Statistics::kTimingTotal()))
								{
									if(w!=-1)
									{
										slamTime.push_back(stat.at(Statistics::kTimingTotal()));
									}
								}

								if(uContains(stat, std::string(Statistics::kMemoryRAM_usage())))
								{
									float ram = stat.at(Statistics::kMemoryRAM_usage());
									if(maxMapRAM==-1 || maxMapRAM < ram)
									{
										maxMapRAM = ram;
									}
								}
								if(uContains(stat, std::string("Odometry/RAM_usage/MB")))
								{
									float ram = stat.at("Odometry/RAM_usage/MB");
									if(maxOdomRAM==-1 || maxOdomRAM < ram)
									{
										maxOdomRAM = ram;
									}
								}

#ifdef WITH_QT
								for(std::map<std::string, UPlotCurve*>::iterator jter=curves.begin(); jter!=curves.end(); ++jter)
								{
#else
								for(std::map<std::string, std::vector<std::pair<std::string, std::vector<LocStats> > > >::iterator jter=localizationMultiStats.begin();
									jter!=localizationMultiStats.end();
									++jter)
								{
#endif
									if(uContains(stat, jter->first))
									{
										double y = stat.at(jter->first);
#ifdef WITH_QT
										double x = s;
										if(useIds)
										{
											x = *iter;
										}
										jter->second->addValue(x,y);
#endif

										if(!localizationMultiStats.empty())
										{
											if(previousStamp > 0 && fabs(s - previousStamp) > locDelay && uContains(localizationSessionStats, jter->first))
											{
												// changed session
												for(std::map<std::string, std::vector<float> >::iterator kter=localizationSessionStats.begin(); kter!=localizationSessionStats.end(); ++kter)
												{
													LocStats values = LocStats::from(localizationSessionStats.at(kter->first));
													localizationMultiStats.at(kter->first).rbegin()->second.push_back(values);
													localizationSessionStats.at(kter->first).clear();
												}

												previousStamp = s;
											}

											if(!uContains(localizationSessionStats, jter->first))
											{
												localizationSessionStats.insert(std::make_pair(jter->first, std::vector<float>()));
											}
											localizationSessionStats.at(jter->first).push_back(y);
										}
									}
								}
								previousStamp = s;
							}
						}
					}

					for(std::map<std::string, std::vector<std::pair<std::string, std::vector<LocStats> > > >::iterator jter=localizationMultiStats.begin();
						jter!=localizationMultiStats.end();
						++jter)
					{
						if(uContains(localizationSessionStats, jter->first) &&
							!localizationSessionStats.at(jter->first).empty())
						{
							// changed session
							LocStats values = LocStats::from(localizationSessionStats.at(jter->first));
							localizationMultiStats.at(jter->first).rbegin()->second.push_back(values);
						}
					}

					std::multimap<int, Link> links;
					std::multimap<int, Link> allLinks;
					driver->getAllLinks(allLinks, true, true);
					std::multimap<int, Link> loopClosureLinks;
					for(std::multimap<int, Link>::iterator jter=allLinks.begin(); jter!=allLinks.end(); ++jter)
					{
						if(jter->second.from() == jter->second.to() || graph::findLink(links, jter->second.from(), jter->second.to(), true) == links.end())
						{
							links.insert(*jter);
						}
						if( jter->second.type() != Link::kNeighbor &&
							jter->second.type() != Link::kNeighborMerged &&
							graph::findLink(loopClosureLinks, jter->second.from(), jter->second.to()) == loopClosureLinks.end())
						{
							loopClosureLinks.insert(*jter);
						}
					}

					float bestScale = 1.0f;
					float bestRMSE = -1;
					float bestRMSEAng = -1;
					float bestVoRMSE = -1;
					Transform bestGtToMap = Transform::getIdentity();
					float kitti_t_err = 0.0f;
					float kitti_r_err = 0.0f;
					float relative_t_err = 0.0f;
					float relative_r_err = 0.0f;
					float loop_t_err = 0.0f;
					float loop_r_err = 0.0f;

					if(ids.size())
					{
						std::map<int, Transform> posesOut;
						std::multimap<int, Link> linksOut;
						int firstId = *ids.begin();
						rtabmap::Optimizer * optimizer = rtabmap::Optimizer::create(params);
						bool useOdomGravity = Parameters::defaultMemUseOdomGravity();
						Parameters::parse(params, Parameters::kMemUseOdomGravity(), useOdomGravity);
						if(useOdomGravity)
						{
							for(std::map<int, Transform>::iterator iter=odomPoses.begin(); iter!=odomPoses.end(); ++iter)
							{
								links.insert(std::make_pair(iter->first, Link(iter->first, iter->first, Link::kGravity, iter->second)));
							}
						}
						std::map<int, Transform> posesWithLandmarks = odomPoses;
						// Added landmark poses if there are some
						for(std::multimap<int, Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
						{
							if(iter->second.type() == Link::kLandmark)
							{
								UASSERT(iter->second.from() > 0 && iter->second.to() < 0);
								if(posesWithLandmarks.find(iter->second.from()) != posesWithLandmarks.end() && posesWithLandmarks.find(iter->second.to()) == posesWithLandmarks.end())
								{
									posesWithLandmarks.insert(std::make_pair(iter->second.to(), posesWithLandmarks.at(iter->second.from())*iter->second.transform()));
								}
							}
						}
						optimizer->getConnectedGraph(firstId, posesWithLandmarks, links, posesOut, linksOut);
						std::list<std::map<int, Transform> > intermediateGraphes;
						std::map<int, Transform> poses = optimizer->optimize(firstId, posesOut, linksOut, incrementalOptimization?&intermediateGraphes:0);
						if(poses.empty())
						{
							// try incremental optimization
							UWARN("Optimization failed! Try incremental optimization...");
							poses = optimizer->optimizeIncremental(firstId, posesOut, linksOut);
							if(poses.empty())
							{
								UERROR("Incremental optimization also failed! Only original RMSE will be shown.");
								bestRMSE = rmse;
							}
							else
							{
								UWARN("Incremental optimization succeeded!");
							}
						}

						if(poses.size())
						{
							//remove landmarks
							std::map<int, Transform>::iterator iter=poses.begin();
							while(iter!=poses.end() && iter->first < 0)
							{
								poses.erase(iter++);
							}

							std::map<int, Transform> groundTruth;
							for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
							{
								if(gtPoses.find(iter->first) != gtPoses.end())
								{
									groundTruth.insert(*gtPoses.find(iter->first));
								}
							}

							outputScaled = outputScaled && groundTruth.size();
							for(float scale=outputScaled?0.900f:1.0f; scale<1.100f; scale+=0.001)
							{
								std::map<int, Transform> scaledPoses;
								std::map<int, Transform> scaledOdomPoses;

								for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
								{
									Transform t = iter->second.clone();
									t.x() *= scale;
									t.y() *= scale;
									t.z() *= scale;
									scaledPoses.insert(std::make_pair(iter->first, t));

									UASSERT(posesOut.find(iter->first)!=posesOut.end());
									t = posesOut.at(iter->first).clone();
									t.x() *= scale;
									t.y() *= scale;
									t.z() *= scale;
									scaledOdomPoses.insert(std::make_pair(iter->first, t));
								}
								// compute RMSE statistics
								float translational_rmse = 0.0f;
								float translational_mean = 0.0f;
								float translational_median = 0.0f;
								float translational_std = 0.0f;
								float translational_min = 0.0f;
								float translational_max = 0.0f;
								float rotational_rmse = 0.0f;
								float rotational_mean = 0.0f;
								float rotational_median = 0.0f;
								float rotational_std = 0.0f;
								float rotational_min = 0.0f;
								float rotational_max = 0.0f;

								graph::calcRMSE(
										groundTruth,
										scaledOdomPoses,
										translational_rmse,
										translational_mean,
										translational_median,
										translational_std,
										translational_min,
										translational_max,
										rotational_rmse,
										rotational_mean,
										rotational_median,
										rotational_std,
										rotational_min,
										rotational_max);
								float translational_rmse_vo = translational_rmse;

								Transform gtToMap = graph::calcRMSE(
										groundTruth,
										scaledPoses,
										translational_rmse,
										translational_mean,
										translational_median,
										translational_std,
										translational_min,
										translational_max,
										rotational_rmse,
										rotational_mean,
										rotational_median,
										rotational_std,
										rotational_min,
										rotational_max);

								if(bestRMSE!=-1 && translational_rmse > bestRMSE)
								{
									break;
								}
								bestRMSE = translational_rmse;
								bestVoRMSE = translational_rmse_vo;
								bestRMSEAng = rotational_rmse;
								bestScale = scale;
								bestGtToMap = gtToMap;
								if(!outputScaled)
								{
									// just did iteration without any scale, then exit
									break;
								}
							}

							for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
							{
								iter->second.x()*=bestScale;
								iter->second.y()*=bestScale;
								iter->second.z()*=bestScale;
								iter->second = bestGtToMap * iter->second;
							}

							if(outputRelativeError)
							{
								if(groundTruth.size() == poses.size())
								{
									// compute Motion statistics
									graph::calcRelativeErrors(uValues(groundTruth), uValues(poses), relative_t_err, relative_r_err);
								}
								else
								{
									std::vector<Transform> gtPoses;
									std::vector<Transform> rPoses;
									for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
									{
										if(groundTruth.find(iter->first) != groundTruth.end())
										{
											gtPoses.push_back(groundTruth.at(iter->first));
											rPoses.push_back(poses.at(iter->first));
										}
									}
									if(!gtPoses.empty())
									{
										graph::calcRelativeErrors(gtPoses, rPoses, relative_t_err, relative_r_err);
									}
								}
							}

							if(outputKittiError)
							{
								if(groundTruth.size() == poses.size())
								{
									// compute KITTI statistics
									graph::calcKittiSequenceErrors(uValues(groundTruth), uValues(poses), kitti_t_err, kitti_r_err);
								}
								else
								{
									printf("Cannot compute KITTI statistics as optimized poses and ground truth don't have the same size (%d vs %d).\n",
											(int)poses.size(), (int)groundTruth.size());
								}
							}

							if(outputPoses)
							{
								std::string dir = UDirectory::getDir(filePath);
								std::string dbName = UFile::getName(filePath);
								dbName = dbName.substr(0, dbName.size()-3); // remove db
								std::string path = dir+UDirectory::separator()+dbName+"_slam.txt";
								std::multimap<int, Link> dummyLinks;
								std::map<int, double> stamps;
								if(!outputKittiError)
								{
									for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
									{
										UASSERT(odomStamps.find(iter->first) != odomStamps.end());
										stamps.insert(*odomStamps.find(iter->first));
									}
								}
								if(!graph::exportPoses(path, outputKittiError?2:10, poses, dummyLinks, stamps))
								{
									printf("Could not export the poses to \"%s\"!?!\n", path.c_str());
								}

								//export odom
								path = dir+UDirectory::separator()+dbName+"_odom.txt";
								stamps.clear();
								if(!outputKittiError)
								{
									for(std::map<int, Transform>::iterator iter=odomPoses.begin(); iter!=odomPoses.end(); ++iter)
									{
										UASSERT(odomStamps.find(iter->first) != odomStamps.end());
										stamps.insert(*odomStamps.find(iter->first));
									}
								}
								if(!graph::exportPoses(path, outputKittiError?2:10, odomPoses, dummyLinks, stamps))
								{
									printf("Could not export the odometry to \"%s\"!?!\n", path.c_str());
								}

								//export ground truth
								if(groundTruth.size())
								{
									path = dir+UDirectory::separator()+dbName+"_gt.txt";
									stamps.clear();
									if(!outputKittiError)
									{
										for(std::map<int, Transform>::iterator iter=groundTruth.begin(); iter!=groundTruth.end(); ++iter)
										{
											UASSERT(odomStamps.find(iter->first) != odomStamps.end());
											stamps.insert(*odomStamps.find(iter->first));
										}
									}
									if(!graph::exportPoses(path, outputKittiError?2:10, groundTruth, dummyLinks, stamps))
									{
										printf("Could not export the ground truth to \"%s\"!?!\n", path.c_str());
									}
								}
							}

							if (outputReport)
							{
								bool fillHeader = false;
								std::ifstream f("report.csv");
    							if(!f.good())
								{
									fillHeader = true;
								}

								std::ofstream myfile;
								myfile.open("report.csv", std::fstream::in | std::fstream::out | std::fstream::app);
								if(fillHeader)
								{
									myfile 	<< "Rosbag name"<<";"<<"error linear (m)"<<";"<<"error linear max (m)"<<";"<<"error linear odom (m)"<<";"
											<<"error angular"<<";"
											<<"Slam avg (hz)"<<";"<<"Slam max (hz)"<<";"
											<<"Odom avg (hz)"<<";"<<"Odom max (hz)"<<std::endl;
								}

								myfile 	<<fileName.c_str()<<";"
										<<bestRMSE<< ";"
										<<maxRMSE<<";"
										<<bestVoRMSE<<";"
										<<bestRMSEAng<<";"
										<<(1/(uMean(slamTime)/1000.0))<<";"
										<<(1/(uMax(slamTime)/1000.0))<<";"
										<<(1/(uMean(odomTime)/1000.0))<<";"
										<<(1/(uMax(odomTime)/1000.0))<<";"<<std::endl;
								myfile.close();
							}

							if(outputLoopAccuracy && !groundTruth.empty() && !linksOut.empty())
							{
								float sumDist = 0.0f;
								float sumAngle = 0.0f;
								int count = 0;
								for(std::multimap<int, Link>::iterator iter=loopClosureLinks.begin(); iter!=loopClosureLinks.end(); ++iter)
								{
									if(	groundTruth.find(iter->second.from())!=groundTruth.end() &&
										groundTruth.find(iter->second.to())!=groundTruth.end())
									{
										Transform gtLink = groundTruth.at(iter->second.from()).inverse()*groundTruth.at(iter->second.to());
										const Transform & t = iter->second.transform();
										Transform scaledLink(
												t.r11(), t.r12(), t.r13(), t.x()*bestScale,
												t.r21(), t.r22(), t.r23(), t.y()*bestScale,
												t.r31(), t.r32(), t.r33(), t.z()*bestScale);
										Transform diff = gtLink.inverse()*scaledLink;
										sumDist += diff.getNorm();
										sumAngle += diff.getAngle();
										++count;
									}
								}
								if(count>0)
								{
									loop_t_err = sumDist/float(count);
									loop_r_err = sumAngle/float(count);
									loop_r_err *= 180/CV_PI; // Rotation error (deg)
								}
							}
						}
					}
					printf("   %s (%d, s=%.3f):\terror lin=%.3fm (max=%.3fm, odom=%.3fm) ang=%.1fdeg%s%s, %s: avg=%dms (max=%dms) loops=%d%s, odom: avg=%dms (max=%dms), camera: avg=%dms, %smap=%dMB\n",
							fileName.c_str(),
							(int)ids.size(),
							bestScale,
							bestRMSE,
							maxRMSE,
							bestVoRMSE,
							bestRMSEAng,
							!outputKittiError?"":uFormat(", KITTI: t_err=%.2f%% r_err=%.2f deg/100m", kitti_t_err, kitti_r_err*100).c_str(),
							!outputRelativeError?"":uFormat(", Relative: t_err=%.3fm r_err=%.2f deg", relative_t_err, relative_r_err).c_str(),
							!localizationMultiStats.empty()?"loc":"slam",
							(int)uMean(slamTime), (int)uMax(slamTime),
							(int)loopClosureLinks.size(),
							!outputLoopAccuracy?"":uFormat(" (t_err=%.3fm r_err=%.2f deg)", loop_t_err, loop_r_err).c_str(),
							(int)uMean(odomTime), (int)uMax(odomTime),
							(int)uMean(cameraTime),
							maxOdomRAM!=-1.0f?uFormat("RAM odom=%dMB ", (int)maxOdomRAM).c_str():"",
							(int)maxMapRAM);

					if(outputLatex)
					{
						std::vector<float> stats;
						stats.push_back(ids.size());
						stats.push_back(bestRMSE);
						stats.push_back(maxRMSE);
						stats.push_back(bestRMSEAng);
						stats.push_back(uMean(odomTime));
						stats.push_back(uMean(slamTime));
						stats.push_back(uMax(slamTime));
						stats.push_back(maxOdomRAM);
						stats.push_back(maxMapRAM);
						outputLatexStatisticsMap.insert(std::make_pair(filePath, stats));

						if(maxOdomRAM != -1.0f)
						{
							odomRAMSet = true;
						}
					}
				}
				driver->closeConnection();
				delete driver;
			}
			else if(uSplit(fileName, '.').size() == 1)
			{
				//sub directory
				subDirs.push_front(currentPath + UDirectory::separator() + fileName);
			}
			currentPathIsDatabase = false;
		}

		if(!localizationMultiStats.empty() && showLoc!=-1)
		{
			printf("---Localization results---\n");
			std::string prefix = "header={";
			printf("%s", prefix.c_str());
			for(std::vector<std::pair<std::string, std::vector<LocStats> > >::iterator iter=localizationMultiStats.begin()->second.begin();
						iter!=localizationMultiStats.begin()->second.end();)
			{
				if(iter!=localizationMultiStats.begin()->second.begin())
				{
					printf("%s",  std::string(prefix.size(), ' ').c_str());
				}
				printf("%s", iter->first.c_str());
				++iter;
				if(iter!=localizationMultiStats.begin()->second.end())
				{
					printf(";\n");
				}
			}
			printf("}\n");


			for(std::map<std::string, std::vector<std::pair<std::string, std::vector<LocStats> > > >::iterator iter=localizationMultiStats.begin();
				iter!=localizationMultiStats.end();
				++iter)
			{
				printf("%s\n", iter->first.c_str());
				for(int k=0; k<6; ++k)
				{
					if(showLoc & (0x1 << k))
					{
						std::string prefix = uFormat("  %s=[",
								k==0?"min":
								k==1?"max":
								k==2?"mean":
								k==3?"stddev":
								k==4?"total":
								"nonnull%");
						printf("%s", prefix.c_str());
						for(std::vector<std::pair<std::string, std::vector<LocStats> > >::iterator jter=iter->second.begin(); jter!=iter->second.end();)
						{
							if(jter!=iter->second.begin())
							{
								printf("%s", std::string(prefix.size(), ' ').c_str());
							}
							for(size_t j=0; j<jter->second.size(); ++j)
							{
								if(k<4)
								{
									printf("%f",
											k==0?jter->second[j].min:
											k==1?jter->second[j].max:
											k==2?jter->second[j].mean:
											jter->second[j].stddev);
								}
								else if(k==4)
								{
									printf("%d",jter->second[j].total);
								}
								else if(k==5)
								{
									printf("%.2f", (jter->second[j].nonNull*100));
								}
								if(j+1 < jter->second.size())
								{
									printf(" ");
								}
							}
							++jter;
							if(jter!=iter->second.end())
							{
								printf(";\n");
							}
						}
						printf("]\n");
					}
				}
				iter->second.clear();
			}
		}

		for(std::list<std::string>::iterator iter=subDirs.begin(); iter!=subDirs.end(); ++iter)
		{
			paths.push_front(*iter);
		}

		if(outputLatexStatisticsMap.size() && paths.empty())
		{
			outputLatexStatistics.push_back(outputLatexStatisticsMap);
			outputLatexStatisticsMap.clear();
		}
	}

	if(outputLatex && outputLatexStatistics.size())
	{
		printf("\nLaTeX output:\n----------------\n");
		printf("\\begin{table*}[!t]\n");
		printf("\\caption{$t_{end}$ is the absolute translational RMSE value at the end "
				"of the experiment as $ATE_{max}$ is the maximum during the experiment. "
				"$r_{end}$ is rotational RMSE value at the end of the experiment. "
				"$o_{avg}$ and $m_{avg}$ are the average computational time "
				"for odometry (front-end) and map update (back-end). "
				"$m_{avg}$ is the maximum computational time for map update. "
				"$O_{end}$ and $M_{end}$ are the RAM usage at the end of the experiment "
				"for odometry and map management respectively.}\n");
		printf("\\label{}\n");
		printf("\\centering\n");
		if(odomRAMSet)
		{
			printf("\\begin{tabular}{l|c|c|c|c|c|c|c|c|c}\n");
			printf("\\cline{2-10}\n");
			printf(" & Size & $t_{end}$ & $t_{max}$ & $r_{end}$ & $o_{avg}$ & $m_{avg}$ & $m_{max}$ & $O_{end}$ & $M_{end}$  \\\\\n");
			printf(" & (nodes) & (m) & (m) & (deg) & (ms) & (ms) & (ms) & (MB) & (MB) \\\\\n");
		}
		else
		{
			printf("\\begin{tabular}{l|c|c|c|c|c|c|c|c}\n");
			printf("\\cline{2-9}\n");
			printf(" & Size & $t_{end}$ & $t_{max}$ & $r_{end}$ & $o_{avg}$ & $m_{avg}$ & $m_{max}$ & $M_{end}$  \\\\\n");
			printf(" & (nodes) & (m) & (m) & (deg) & (ms) & (ms) & (ms) & (MB)  \\\\\n");
		}

		printf("\\hline\n");

		for(unsigned int j=0; j<outputLatexStatistics.size(); ++j)
		{
			if(outputLatexStatistics[j].size())
			{
				std::vector<int> lowestIndex;
				if(outputLatexStatistics[j].size() > 1)
				{
					std::vector<float> lowestValue(outputLatexStatistics[j].begin()->second.size(),-1);
					lowestIndex = std::vector<int>(lowestValue.size(),0);
					int index = 0;
					for(std::map<std::string, std::vector<float> >::iterator iter=outputLatexStatistics[j].begin(); iter!=outputLatexStatistics[j].end(); ++iter)
					{
						UASSERT(lowestValue.size() == iter->second.size());
						for(unsigned int i=0; i<iter->second.size(); ++i)
						{
							if(lowestValue[i] == -1 || (iter->second[i]>0.0f && lowestValue[i]>iter->second[i]))
							{
								lowestValue[i] = iter->second[i];
								lowestIndex[i] = index;
							}
						}
						++index;
					}
				}

				int index = 0;
				for(std::map<std::string, std::vector<float> >::iterator iter=outputLatexStatistics[j].begin(); iter!=outputLatexStatistics[j].end(); ++iter)
				{
					UASSERT(iter->second.size() == 9);
					printf("%s & ", uReplaceChar(iter->first.c_str(), '_', '-').c_str());
					printf("%d & ", (int)iter->second[0]);
					printf("%s%.3f%s & ", lowestIndex.size()&&lowestIndex[1]==index?"\\textbf{":"", iter->second[1], lowestIndex.size()&&lowestIndex[1]==index?"}":"");
					printf("%s%.3f%s & ", lowestIndex.size()&&lowestIndex[2]==index?"\\textbf{":"", iter->second[2], lowestIndex.size()&&lowestIndex[2]==index?"}":"");
					printf("%s%.2f%s & ", lowestIndex.size()&&lowestIndex[3]==index?"\\textbf{":"", iter->second[3], lowestIndex.size()&&lowestIndex[3]==index?"}":"");
					printf("%s%d%s & ", lowestIndex.size()&&lowestIndex[4]==index?"\\textbf{":"", (int)iter->second[4], lowestIndex.size()&&lowestIndex[4]==index?"}":"");
					printf("%s%d%s & ", lowestIndex.size()&&lowestIndex[5]==index?"\\textbf{":"", (int)iter->second[5], lowestIndex.size()&&lowestIndex[5]==index?"}":"");
					printf("%s%d%s & ", lowestIndex.size()&&lowestIndex[6]==index?"\\textbf{":"", (int)iter->second[6], lowestIndex.size()&&lowestIndex[6]==index?"}":"");
					if(odomRAMSet)
					{
						printf("%s%d%s & ", lowestIndex.size()&&lowestIndex[7]==index?"\\textbf{":"", (int)iter->second[7], lowestIndex.size()&&lowestIndex[7]==index?"}":"");
					}
					printf("%s%d%s ", lowestIndex.size()&&lowestIndex[8]==index?"\\textbf{":"", (int)iter->second[8], lowestIndex.size()&&lowestIndex[8]==index?"}":"");
					printf("\\\\\n");
					++index;
				}
				printf("\\hline\n");
			}
		}


		printf("\\end{tabular}\n");
		printf("\\end{table*}\n----------------\n");
	}
#ifdef WITH_QT
	if(figures.size())
	{
		for(std::map<std::string, UPlot*>::iterator iter=figures.begin(); iter!=figures.end(); ++iter)
		{
			if(!useIds)
			{
				iter->second->frameData();
			}
			if(exportFigures)
			{
				QString data = iter->second->getAllCurveDataAsText();
				if(!data.isEmpty())
				{
					QString filePath = QString(exportPrefix.c_str()) + (exportPrefix.empty()?"":"-") + iter->second->windowTitle().replace('/', "-") + ".txt";
					QFile file(filePath);
					if(file.open(QIODevice::Text | QIODevice::WriteOnly))
					{
						file.write(data.toUtf8());
						file.close();
						printf("Exported \"%s\".\n", filePath.toStdString().c_str());
					}
					else
					{
						printf("ERROR: could not open file \"%s\" for writing!\n", filePath.toStdString().c_str());
					}
				}
			}
			else
			{
				iter->second->show();
			}
		}
		if(!exportFigures)
		{
			return app.exec();
		}
	}
#endif
	return 0;
}
