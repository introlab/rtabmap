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
    LARGE_INTEGER count, freq;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return double(count.QuadPart) / freq.QuadPart;
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
double UTimer::getInterval()
{
	if(stopTimeRecorded_.QuadPart == startTimeRecorded_.QuadPart)
	{
		return getElapsedTime();
	}
	else
	{
		return double(stopTimeRecorded_.QuadPart - startTimeRecorded_.QuadPart) / frequency_.QuadPart;
	}
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
double UTimer::getInterval()
{
	if(startTimeRecorded_.tv_sec == stopTimeRecorded_.tv_sec && startTimeRecorded_.tv_usec == stopTimeRecorded_.tv_usec)
	{
		return getElapsedTime();
	}
	else
	{
		double start = double(startTimeRecorded_.tv_sec) + double(startTimeRecorded_.tv_usec) / 1000000.0;
		double stop = double(stopTimeRecorded_.tv_sec) + double(stopTimeRecorded_.tv_usec) / 1000000.0;
		return stop - start;
	}
}
#endif

double UTimer::ticks()        // Stop->start and return Interval
{
    double inter = elapsed();
    start();
    return inter;
}
