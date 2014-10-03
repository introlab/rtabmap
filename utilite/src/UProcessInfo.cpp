/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "rtabmap/utilite/UProcessInfo.h"

#ifdef _WIN32
#include "Windows.h"
#include "Psapi.h"
#elif __APPLE__
#include <sys/resource.h>
#else
#include <fstream>
#include <stdlib.h>
#include "rtabmap/utilite/UStl.h"
#endif

UProcessInfo::UProcessInfo() {}

UProcessInfo::~UProcessInfo() {}

// return in bytes
long int UProcessInfo::getMemoryUsage()
{
	long int memoryUsage = -1;

#ifdef _WIN32
		HANDLE hProc = GetCurrentProcess();
		PROCESS_MEMORY_COUNTERS info;
		BOOL okay = GetProcessMemoryInfo(hProc, &info, sizeof(info));
		if(okay)
		{
			memoryUsage = info.WorkingSetSize;
		}
#elif __APPLE__
		rusage u;
		if(getrusage(RUSAGE_SELF, &u) == 0)
		{
			memoryUsage = u.ru_maxrss;
		}
#else
		std::fstream file("/proc/self/status", std::fstream::in);
		if(file.is_open())
		{
			std::string bytes;
			while(std::getline(file, bytes))
			{
				if(bytes.find("VmRSS") != bytes.npos)
				{
					std::list<std::string> strs = uSplit(bytes, ' ');
					if(strs.size()>1)
					{
						memoryUsage = atol(uValueAt(strs,1).c_str()) * 1024;
					}
					break;
				}
			}
			file.close();
		}
#endif

	return memoryUsage;
}
