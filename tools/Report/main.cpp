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
#endif

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
#ifdef WITH_QT
			"rtabmap-report [\"Statistic/Id\"] [options] path\n"
#else
			"rtabmap-report [options] path\n"
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
#ifdef WITH_QT
			"    --stats            Show available statistics to plot (if path is a file). \n"
			"    --invert           When reading many databases, put all curves from a same \n"
			"                       database in same figure, instead of all same curves from \n"
			"                       different database in same figure. When reading a single \n"
			"                       database, the inverse will be done. \n"
			"    --ids              Use IDs for x axis instead of time in the figures. \n"
			"    --start #          Start from this node ID for the figures.\n"
#endif
			"    --report           Export all statistics values in report.txt \n"
			"    --help             Show usage\n\n");
	exit(1);
}

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
	bool showAvailableStats = false;
	bool invertFigures = false;
	bool useIds = false;
	int startId = 0;
	std::vector<std::string> statsToShow;
#ifdef WITH_QT
	std::map<std::string, UPlot*> figures;
#endif
	for(int i=1; i<argc-1; ++i)
	{
		if(strcmp(argv[i], "--help") == 0)
		{
			showUsage();
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
#ifdef WITH_QT
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
		else
		{

			statsToShow.push_back(argv[i]);
		}
#endif
	}

	std::string path = argv[argc-1];
	path = uReplaceChar(path, '~', UDirectory::homeDir());

#ifdef WITH_QT
	if(!UDirectory::exists(path) && UFile::getExtension(path).compare("db") == 0)
	{
		invertFigures = !invertFigures;
	}
	if(!invertFigures)
	{
		for(size_t i=0; i<statsToShow.size(); ++i)
		{
			std::string figureTitle = statsToShow[i];
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
		}
		statsToShow.clear();
	}
#endif

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

		while(currentPathIsDatabase || !(fileName = currentDir.getNextFileName()).empty())
		{
			if(currentPathIsDatabase || UFile::getExtension(fileName).compare("db") == 0)
			{
				std::string filePath;
				if(currentPathIsDatabase)
				{
					filePath = currentPath;
				}
				else
				{
					filePath = currentPath + UDirectory::separator() + fileName;
				}

				DBDriver * driver = DBDriver::create();
				ParametersMap params;
				if(driver->openConnection(filePath))
				{
					ULogger::setLevel(ULogger::kError); // to suppress parameter warnings
					params = driver->getLastParameters();
					ULogger::setLevel(ULogger::kWarning);
					std::set<int> ids;
					driver->getAllNodeIds(ids);
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
					std::map<std::string, double> firstStamps;
					if(statsToShow.empty())
					{
						for(std::map<std::string, UPlot*>::iterator iter=figures.begin(); iter!=figures.end(); ++iter)
						{
							curves.insert(std::make_pair(iter->first, iter->second->addCurve(filePath.c_str())));
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
							}
						}
					}
#endif

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
							if(uContains(stats, *iter))
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
									if(w>=0)
									{
										slamTime.push_back(stat.at("RtabmapROS/TotalTime/ms"));
									}
								}
								else if(uContains(stat, Statistics::kTimingTotal()))
								{
									if(w>=0)
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
									if(uContains(stat, jter->first) && *iter >= startId)
									{
										if(!uContains(firstStamps, jter->first))
										{
											firstStamps.insert(std::make_pair(jter->first, s));
										}
										float x = s - firstStamps.at(jter->first);
										if(useIds)
										{
											x = *iter;
										}
										float y = stat.at(jter->first);
										jter->second->addValue(x,y);
									}
								}
#endif
							}
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
						optimizer->getConnectedGraph(firstId, odomPoses, links, posesOut, linksOut);

						std::map<int, Transform> poses = optimizer->optimize(firstId, posesOut, linksOut);
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
									printf("Could not export the ground truth to \"%s\"!?!\n", path.c_str());
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
					printf("   %s (%d, s=%.3f):\terror lin=%.3fm (max=%.3fm, odom=%.3fm) ang=%.1fdeg%s%s, slam: avg=%dms (max=%dms) loops=%d%s, odom: avg=%dms (max=%dms), camera: avg=%dms, %smap=%dMB\n",
							fileName.c_str(),
							(int)ids.size(),
							bestScale,
							bestRMSE,
							maxRMSE,
							bestVoRMSE,
							bestRMSEAng,
							!outputKittiError?"":uFormat(", KITTI: t_err=%.2f%% r_err=%.2f deg/100m", kitti_t_err, kitti_r_err*100).c_str(),
							!outputRelativeError?"":uFormat(", Relative: t_err=%.3fm r_err=%.2f deg", relative_t_err, relative_r_err).c_str(),
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
			iter->second->show();
		}
		return app.exec();
	}
#endif
	return 0;
}
