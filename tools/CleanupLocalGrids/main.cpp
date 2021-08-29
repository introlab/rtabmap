/*
Copyright (c) 2010-2021, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/core/util3d_transforms.h>

using namespace rtabmap;

void showUsage()
{
	printf("\n"
			"Clear empty space from local occupancy grids and laser scans based on the saved optimized global 2d grid map.\n"
			"Advantages:\n"
			"   * If the map needs to be regenerated in the future (e.g., when \n"
			"     we re-use the map in SLAM mode), removed obstacles won't reappear.\n"
			"   * [--scan] The cropped laser scans will be also used for localization,\n"
			"     so if dynamic obstacles have been removed, localization won't try to\n"
			"     match them anymore.\n\n"
			"Disadvantage:\n"
			"   * [--scan] Cropping the laser scans cannot be reverted, but grids can.\n"
			"\nUsage:\n"
			"rtabmap-cleanupLocalGrids [options] database.db\n"
			"Options:\n"
			"    --radius #   Radius in cells around empty cell without obstacles to clear\n"
			"                  underlying obstacles. Default is 1.\n"
			"    --scan       Filter also scans, otherwise only local grids are filtered.\n"
			"\n");
	;
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

	int cropRadius = 1;
	bool filterScans = false;

	for(int i=1; i<argc; ++i)
	{
		if(std::strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}
		else if(std::strcmp(argv[i], "--scan") == 0)
		{
			filterScans = true;
		}
		else if(std::strcmp(argv[i], "--radius") == 0)
		{
			++i;
			if(i<argc-1)
			{
				cropRadius = uStr2Int(argv[i]);
				UASSERT(cropRadius>=0);
			}
			else
			{
				showUsage();
			}
		}
	}

	std::string dbPath = argv[argc-1];

	if(!UFile::exists(dbPath))
	{
		UERROR("File \"%s\" doesn't exist!", dbPath.c_str());
		return -1;
	}

	// Get parameters
	ParametersMap parameters;
	DBDriver * driver = DBDriver::create();
	if(driver->openConnection(dbPath))
	{
		float xMin, yMin, cellSize;
		cv::Mat map = driver->load2DMap(xMin, yMin, cellSize);
		if(map.empty())
		{
			UERROR("Database %s doesn't have optimized 2d map saved in it!", dbPath.c_str());
			return -1;
		}

		printf("Options:\n");
		printf("  --radius:  %d cell(s) (cell size=%.3fm)\n", cropRadius, cellSize);
		printf("  --scan: %s\n", filterScans?"true":"false");

		Transform lastLocalizationPose;
		std::map<int, Transform> poses = driver->loadOptimizedPoses(&lastLocalizationPose);
		if(poses.empty() || poses.lower_bound(1) == poses.end())
		{
			UERROR("Database %s doesn't have optimized poses saved in it!", dbPath.c_str());
			return -1;
		}

		int maxPoses = 0;
		for(std::map<int, Transform>::iterator iter=poses.lower_bound(1); iter!=poses.end(); ++iter)
		{
			++maxPoses;
		}

		printf("Processing %d grids...\n", maxPoses);
		int processedGrids = 1;
		for(std::map<int, Transform>::iterator iter=poses.lower_bound(1); iter!=poses.end(); ++iter, ++processedGrids)
		{
			// local grid
			cv::Mat gridGround;
			cv::Mat gridObstacles;
			cv::Mat gridEmpty;

			// scan
			SensorData data;
			driver->getNodeData(iter->first, data);
			LaserScan scan;
			data.uncompressData(0,0,&scan,0,&gridGround,&gridObstacles,&gridEmpty);

			if(!gridObstacles.empty())
			{
				cv::Mat filtered = cv::Mat(1, gridObstacles.cols, gridObstacles.type());
				int oi = 0;
				for(int i=0; i<gridObstacles.cols; ++i)
				{
					const float * ptr = gridObstacles.ptr<float>(0, i);
					cv::Point3f pt(ptr[0], ptr[1], gridObstacles.channels()==2?0:ptr[2]);
					pt = util3d::transformPoint(pt, iter->second);

					int x = int((pt.x - xMin) / cellSize + 0.5f);
					int y = int((pt.y - yMin) / cellSize + 0.5f);

					if(x>=0 && x<map.cols &&
					   y>=0 && y<map.rows)
					{
						bool obstacleDetected = false;

						for(int j=-cropRadius; j<=cropRadius && !obstacleDetected; ++j)
						{
							for(int k=-cropRadius; k<=cropRadius && !obstacleDetected; ++k)
							{
								if(x+j>=0 && x+j<map.cols &&
								   y+k>=0 && y+k<map.rows &&
								   map.at<unsigned char>(y+k,x+j) == 100)
								{
									obstacleDetected = true;
								}
							}
						}

						if(map.at<unsigned char>(y,x) != 0 || obstacleDetected)
						{
							// Verify that we don't have an obstacle on neighbor cells
							cv::Mat(gridObstacles, cv::Range::all(), cv::Range(i,i+1)).copyTo(cv::Mat(filtered, cv::Range::all(), cv::Range(oi,oi+1)));
							++oi;
						}
					}
				}

				if(oi != gridObstacles.cols)
				{
					printf("Grid id=%d (%d/%d) filtered %d -> %d\n", iter->first, processedGrids, maxPoses, gridObstacles.cols, oi);

					// update
					driver->updateOccupancyGrid(iter->first,
							gridGround,
							cv::Mat(filtered, cv::Range::all(), cv::Range(0, oi)),
							gridEmpty,
							cellSize,
							data.gridViewPoint());
				}
			}

			if(filterScans && !scan.isEmpty())
			{
				Transform mapToScan = iter->second * scan.localTransform();

				cv::Mat filtered = cv::Mat(1, scan.size(), scan.dataType());
				int oi = 0;
				for(int i=0; i<scan.size(); ++i)
				{
					const float * ptr = scan.data().ptr<float>(0, i);
					cv::Point3f pt(ptr[0], ptr[1], scan.is2d()?0:ptr[2]);
					pt = util3d::transformPoint(pt, mapToScan);

					int x = int((pt.x - xMin) / cellSize + 0.5f);
					int y = int((pt.y - yMin) / cellSize + 0.5f);

					if(x>=0 && x<map.cols &&
					   y>=0 && y<map.rows)
					{
						bool obstacleDetected = false;

						for(int j=-cropRadius; j<=cropRadius && !obstacleDetected; ++j)
						{
							for(int k=-cropRadius; k<=cropRadius && !obstacleDetected; ++k)
							{
								if(x+j>=0 && x+j<map.cols &&
								   y+k>=0 && y+k<map.rows &&
								   map.at<unsigned char>(y+k,x+j) == 100)
								{
									obstacleDetected = true;
								}
							}
						}

						if(map.at<unsigned char>(y,x) != 0 || obstacleDetected)
						{
							// Verify that we don't have an obstacle on neighbor cells
							cv::Mat(scan.data(), cv::Range::all(), cv::Range(i,i+1)).copyTo(cv::Mat(filtered, cv::Range::all(), cv::Range(oi,oi+1)));
							++oi;
						}
					}
				}

				if(oi != scan.size())
				{
					printf("Scan id=%d (%d/%d) filtered %d -> %d\n", iter->first, processedGrids, maxPoses, (int)scan.size(), oi);

					// update
					if(scan.angleIncrement()!=0)
					{
						// copy meta data
						scan = LaserScan(
								cv::Mat(filtered, cv::Range::all(), cv::Range(0, oi)),
								scan.format(),
								scan.rangeMin(),
								scan.rangeMax(),
								scan.angleMin(),
								scan.angleMax(),
								scan.angleIncrement(),
								scan.localTransform());
					}
					else
					{
						// copy meta data
						scan = LaserScan(
								cv::Mat(filtered, cv::Range::all(), cv::Range(0, oi)),
								scan.maxPoints(),
								scan.rangeMax(),
								scan.format(),
								scan.localTransform());
					}
					driver->updateLaserScan(iter->first, scan);
				}
			}
		}
	}
	else
	{
		UERROR("Cannot open database %s!", dbPath.c_str());
		return -1;
	}
	driver->closeConnection();
	delete driver;
	driver = 0;

	printf("Done!\n");
	return 0;
}
