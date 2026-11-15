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

#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/ULogger.h"

///////////////////////
// public:
///////////////////////
UTimer::UTimer()
{
#ifdef _WIN32
    QueryPerformanceFrequency(&frequency_);
#endif
    start(); // This will initialize the private counters
}

UTimer::~UTimer() {}

#ifdef _WIN32
double UTimer::now()
{
#if defined(_WIN32_WINNT) && (_WIN32_WINNT >= _WIN32_WINNT_WIN8)
	FILETIME ft;
	GetSystemTimePreciseAsFileTime(&ft);
	__int64* val = (__int64*)&ft;
	return static_cast<double>(*val) / 10000000.0 - 11644473600.0; // The Windows epoch is Jan 1 1601, the Unix epoch Jan 1 1970.
#else
    LARGE_INTEGER count, freq;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return double(count.QuadPart) / freq.QuadPart;
#endif
}

void UTimer::start()
{
    QueryPerformanceCounter(&startTimeRecorded_);
    stopTimeRecorded_ = startTimeRecorded_;
}
void UTimer::stop()
{
    QueryPerformanceCounter(&stopTimeRecorded_);

}
double UTimer::getElapsedTime()
{
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
    return double(now.QuadPart - startTimeRecorded_.QuadPart) / frequency_.QuadPart;
}
#else
double UTimer::now()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return double(tv.tv_sec) + double(tv.tv_usec) / 1000000.0;
}

void UTimer::start()
{
    gettimeofday(&startTimeRecorded_, NULL);
    stopTimeRecorded_ = startTimeRecorded_;
}
void UTimer::stop()
{
    gettimeofday(&stopTimeRecorded_, NULL);

}
double UTimer::getElapsedTime()
{
	return UTimer::now() - (double(startTimeRecorded_.tv_sec) + double(startTimeRecorded_.tv_usec) / 1000000.0);

}
#endif

double UTimer::ticks()        // Stop->start and return Interval
{
    double inter = elapsed();
    start();
    return inter;
}
