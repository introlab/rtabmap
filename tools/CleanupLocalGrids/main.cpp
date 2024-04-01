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

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UTimer.h>
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
	ULogger::setLevel(ULogger::kInfo);

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
	Rtabmap rtabmap;
	rtabmap.init(ParametersMap(), dbPath, true);

	float xMin, yMin, cellSize;
	cv::Mat map = rtabmap.getMemory()->load2DMap(xMin, yMin, cellSize);
	if(map.empty())
	{
		UERROR("Database %s doesn't have optimized 2d map saved in it!", dbPath.c_str());
		return -1;
	}

	printf("Options:\n");
	printf("  --radius:  %d cell(s) (cell size=%.3fm)\n", cropRadius, cellSize);
	printf("  --scan: %s\n", filterScans?"true":"false");

	std::map<int, Transform> poses = rtabmap.getLocalOptimizedPoses();
	if(poses.empty() || poses.lower_bound(1) == poses.end())
	{
		UERROR("Database %s doesn't have optimized poses saved in it!", dbPath.c_str());
		return -1;
	}

	UTimer timer;
	printf("Cleaning grids...\n");
	int modifiedCells = rtabmap.cleanupLocalGrids(poses, map, xMin, yMin, cellSize, cropRadius, filterScans);
	printf("Cleanup %d cells! (%fs)\n", modifiedCells, timer.ticks());

	rtabmap.close();

	printf("Done!\n");
	return 0;
}
