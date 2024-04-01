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

#ifndef UTIMER_H
#define UTIMER_H

#include "rtabmap/utilite/utilite_export.h" // DLL export/import defines

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#include <time.h>
#endif

/**
 * This class is used to time some codes (in seconds).
 * On Unix, the resolution is up to microseconds (see gettimeofday()).
 * On Windows, the performance counter is used (see QueryPerformanceCounter() and QueryPerformanceFrequency()).
 * Example:
 * @code
 *      UTimer timer;
 *      timer.start();
 *      ... (do some work)
 *      timer.stop();
 *      double seconds = timer.getInterval();
 *      ...
 * @endcode
 */
class UTILITE_EXPORT UTimer
{
public:
    UTimer();
    ~UTimer();

    /** 
     * This method is used to get 
     * the time of the system right now.
     * @return double the time in seconds.
     */
    static double now();

    /** 
     * This method starts the timer.
     */
    void start();

    /** 
     * This method stops the timer.
     */
    void stop();

    /** 
     * This method is used to get the elapsed time
     * between now and the start(). If timer is stopped, the interval time
	 * between stop() and the start() is returned.
     * @return double the interval in seconds.
     */
    double elapsed() {return getElapsedTime();}
    double getElapsedTime();

    /**
	 * This method is used to get the interval time
	 * between stop() and the start().
	 * @return double the interval in seconds.
	 * @deprecated use elapsed() instead.
	 */
    UTILITE_DEPRECATED double getInterval();

    /** 
     * This method is used to get the interval of 
     * the timer while it is running. It's automatically 
     * stop the timer, get the interval and restart 
     * the timer. It's the same of calling stop(), 
     * elapsed() and start(). Method restart() does the same thing, for convenience.
     * @return double the interval in seconds.
     */
    double restart() {return ticks();}
    double ticks();

private:
#ifdef _WIN32
    LARGE_INTEGER startTimeRecorded_; /* When we start the timer, timeRecorded is copied over lastTimeRecorded.*/
    LARGE_INTEGER stopTimeRecorded_;  /* When we stop the timer. */

    LARGE_INTEGER frequency_;          /* Keep the frequency of the counter */
#else
    struct timeval startTimeRecorded_; /* When we start the timer, timeRecorded is copied over lastTimeRecorded.*/
    struct timeval stopTimeRecorded_;  /* When we stop the timer. */
#endif
};

#endif //UTIMER_H
