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
