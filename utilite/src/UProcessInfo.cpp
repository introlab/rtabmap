/*
Copyright (c) 2010, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
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
