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

/*
 * Originally written by Phillip Sitbon
 *  Copyright 2003
 */

#ifndef USEMAPHORE_H
#define USEMAPHORE_H

#include <errno.h>

#ifdef _WIN32
#include "rtabmap/utilite/Win32/UWin32.h"
#include <atomic>
#define SEM_VALUE_MAX ((int) ((~0u) >> 1))
#else
#include <pthread.h>
#include <sys/time.h>
#endif

/**
 * A semaphore class.
 *
 * On an acquire() call, the calling thread is blocked if the
 * USemaphore's value is <= 0. It is unblocked when release() is called.
 * The function acquire() decreases by 1 (default) the
 * semaphore's value and release() increases it by 1 (default).
 *
 * Example:
 * @code
 * USemaphore s;
 * s.acquire(); // Will wait until s.release() is called by another thread.
 * @endcode
 *
 * @see UMutex
 */
class USemaphore
{
public:
	/**
	 * The constructor. The semaphore waits on acquire() when its value is <= 0.
	 * @param n number to initialize
	 */
	USemaphore( int initValue = 0 )
#ifdef _WIN32
		: _count(initValue)
#endif
	{
#ifdef _WIN32
		S = CreateSemaphore(0,initValue,SEM_VALUE_MAX,0);
#else
		_available = initValue;
		pthread_mutex_init(&_waitMutex, NULL);
		pthread_cond_init(&_cond, NULL);
#endif
	}

	virtual ~USemaphore()
	{
#ifdef _WIN32
		CloseHandle(S);
#else
		pthread_cond_destroy(&_cond);
		pthread_mutex_destroy(&_waitMutex);
#endif
	}

	/**
	 * Acquire the semaphore. If semaphore's value is <=0, the
	 * calling thread will wait until the count acquired is released.
	 * @see release()
	 * @param n number to acquire
	 * @param t time to wait (ms), a value <=0 means infinite
	 * @return true on success, false on error/timeout
	 */
#ifdef _WIN32
	bool acquire(int n = 1, int ms = 0) const
	{
		int rt = 0;
		while(n-- > 0 && rt==0)
		{
			rt = WaitForSingleObject((HANDLE)S, ms<=0?INFINITE:ms);
			if(rt == 0) {
				--_count;
			}
		}
		return rt == 0;
	}
#else
	bool acquire(int n = 1, int ms = 0)
	{
		int rt = 0;
		pthread_mutex_lock(&_waitMutex);
		while (n > _available && rt == 0)
		{
			if(ms > 0)
			{
				struct timespec timeToWait;
				struct timeval now;

				gettimeofday(&now,NULL);

				timeToWait.tv_sec = now.tv_sec + ms/1000;
				timeToWait.tv_nsec = (now.tv_usec+1000UL*(ms%1000))*1000UL;

				rt = pthread_cond_timedwait(&_cond, &_waitMutex, &timeToWait);
			}
			else
			{
				rt = pthread_cond_wait(&_cond, &_waitMutex);
			}
		}
		if(rt == 0)
		{
			// only remove them if waiting did not fail
			_available -= n;
		}
		pthread_mutex_unlock(&_waitMutex);
		return rt == 0;
	}
#endif

	/*
	 * Try to acquire the semaphore, not a blocking call.
	 * @return 0 if the semaphore was acquired, EAGAIN if it couldn't be taken
	 *         without waiting.
	 * @note The Windows overload takes no argument (it always acquires 1).
	 */
#ifdef _WIN32
	int acquireTry() const
	{
		// Non-blocking try-acquire: timeout 0, not INFINITE. INFINITE here
		// turned this into a blocking acquire and hung tests when the count
		// reached 0.
		if(WaitForSingleObject((HANDLE)S, 0) == WAIT_OBJECT_0)
		{
			--_count;
			return 0;
		}
		return EAGAIN;
	}
#else
	/**
	 * @param n the number to acquire.
	 */
	int acquireTry(int n)
	{
		pthread_mutex_lock(&_waitMutex);
		if(n > _available)
		{
			pthread_mutex_unlock(&_waitMutex);
			return EAGAIN;
		}
		_available -= n;
		pthread_mutex_unlock(&_waitMutex);
		return 0;
	}
#endif

	/**
	 * Release the semaphore, increasing its value by n and
	 * signaling waiting threads (which called acquire()).
	 * @param n the number to release.
	 * @return 0 on success, ERANGE if the release would push the semaphore over
	 *         SEM_VALUE_MAX (the count is then left unchanged). The Unix
	 *         implementation cannot fail and always returns 0.
	 */
#ifdef _WIN32
	int release(int n = 1) const
	{
		if(ReleaseSemaphore((HANDLE)S, n, 0))
		{
			_count += n;
			return 0;
		}
		return ERANGE;
	}
#else
	int release(int n = 1)
	{
		pthread_mutex_lock(&_waitMutex);
		_available += n;
		pthread_cond_broadcast(&_cond);
		pthread_mutex_unlock(&_waitMutex);
		return 0;
	}
#endif

	/**
	 * Get the USempahore's value.
	 * @return the semaphore's value
	 */
#ifdef _WIN32
	int value() const
	{
		// ReleaseSemaphore(S, 0, &V) returns FALSE on Windows (release count
		// must be >= 1) and never writes V, so we mirror the count in an atomic
		// instead. The value is eventually-consistent under concurrent
		// acquire/release but accurate at quiescence.
		return (int)_count.load();
	}
#else
	int value()
	{
		int value = 0;
		pthread_mutex_lock(&_waitMutex);
		value = _available;
		pthread_mutex_unlock(&_waitMutex);
		return value;
	}
#endif

#ifdef _WIN32
	/*
	 * Reset the semaphore count.
	 * @param init the initial value
	 * TODO implement on posix ?
	 */
	void reset( int init = 0 )
	{
		CloseHandle(S);
		S = CreateSemaphore(0,init,SEM_VALUE_MAX,0);
		_count = init;
	}
#endif

private:
	void operator=(const USemaphore &){}
#ifdef _WIN32
	USemaphore(const USemaphore &S){}
	HANDLE S;
	mutable std::atomic<long> _count;
#else
	USemaphore(const USemaphore &):_available(0){}
	pthread_mutex_t _waitMutex;
	pthread_cond_t _cond;
	int _available;
#endif
};

#endif // USEMAPHORE_H
