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
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <stdio.h>

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-report path\n"
			"  path               Directory containing rtabmap databases.\n\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	if(argc < 2)
	{
		showUsage();
	}

	std::string path = argv[1];
	path = uReplaceChar(path, '~', UDirectory::homeDir());

	std::string fileName;
	std::list<std::string> paths;
	paths.push_back(path);
	while(paths.size())
	{
		std::string currentPath = paths.front();
		UDirectory currentDir(currentPath);
		paths.pop_front();
		if(!currentDir.isValid())
		{
			continue;
		}
		std::list<std::string> subDirs;
		printf("Directory: %s\n", currentPath.c_str());
		while(!(fileName = currentDir.getNextFileName()).empty())
		{
			if(UFile::getExtension(fileName).compare("db") == 0)
			{
				std::string filePath = currentPath + UDirectory::separator() + fileName;
				DBDriver * driver = DBDriver::create();
				if(driver->openConnection(filePath))
				{
					std::set<int> ids;
					driver->getAllNodeIds(ids);
					std::map<int, std::pair<std::map<std::string, float>, double> > stats = driver->getAllStatistics();
					std::vector<float> cameraTime;
					cameraTime.reserve(ids.size());
					std::vector<float> odomTime;
					odomTime.reserve(ids.size());
					std::vector<float> slamTime;
					slamTime.reserve(ids.size());
					float rmse = -1;
					float maxRMSE = -1;
					for(std::set<int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
					{
						Transform p, gt;
						GPS gps;
						int m, w;
						std::string l;
						double s;
						std::vector<float> v;
						driver->getNodeInfo(*iter, p, m, w, l, s, gt, v, gps);

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
							if(w >= 0 && uContains(stat, Statistics::kTimingTotal()))
							{
								slamTime.push_back(stat.at(Statistics::kTimingTotal()));
							}
						}
					}
					printf("   %s (%d): %fm (max=%fm), slam: avg=%dms max=%dms, odom: avg=%dms max=%dms, camera: avg=%dms max=%dms\n",
							fileName.c_str(),
							(int)ids.size(),
							rmse,
							maxRMSE,
							(int)uMean(slamTime), (int)uMax(slamTime),
							(int)uMean(odomTime), (int)uMax(odomTime),
							(int)uMean(cameraTime), (int)uMax(cameraTime));
				}
				driver->closeConnection();
				delete driver;
			}
			else if(uSplit(fileName, '.').size() == 1)
			{
				//sub directory
				subDirs.push_front(currentPath + UDirectory::separator() + fileName);
			}
		}
		for(std::list<std::string>::iterator iter=subDirs.begin(); iter!=subDirs.end(); ++iter)
		{
			paths.push_front(*iter);
		}
	}

	return 0;
}
