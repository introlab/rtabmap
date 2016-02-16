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

/*
 * Originally written by Phillip Sitbon
 *  Copyright 2003
 */

#ifndef USEMAPHORE_H
#define USEMAPHORE_H

#include <errno.h>

#ifdef _WIN32
#include "rtabmap/utilite/Win32/UWin32.h"
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
	 * @return false if the semaphore can't be taken without waiting (value <= 0), true otherwise
	 */
#ifdef _WIN32
	int acquireTry() const
	{
		return ((WaitForSingleObject((HANDLE)S,INFINITE)==WAIT_OBJECT_0)?0:EAGAIN);
	}
#else
	int acquireTry(int n)
	{
		pthread_mutex_lock(&_waitMutex);
		if(n > _available)
		{
			pthread_mutex_unlock(&_waitMutex);
			return false;
		}
		_available -= n;
		pthread_mutex_unlock(&_waitMutex);
		return true;
	}
#endif

	/**
	 * Release the semaphore, increasing its value by 1 and
	 * signaling waiting threads (which called acquire()).
	 */
#ifdef _WIN32
	int release(int n = 1) const
	{
		return (ReleaseSemaphore((HANDLE)S,n,0)?0:ERANGE);
	}
#else
	void release(int n = 1)
	{
		pthread_mutex_lock(&_waitMutex);
		_available += n;
		pthread_cond_broadcast(&_cond);
		pthread_mutex_unlock(&_waitMutex);
	}
#endif

	/**
	 * Get the USempahore's value.
	 * @return the semaphore's value
	 */
#ifdef _WIN32
	int value() const
	{
		LONG V = -1; ReleaseSemaphore((HANDLE)S,0,&V); return V;
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
	}
#endif

private:
	void operator=(const USemaphore &S){}
#ifdef _WIN32
	USemaphore(const USemaphore &S){}
	HANDLE S;
#else
	USemaphore(const USemaphore &S):_available(0){}
	pthread_mutex_t _waitMutex;
	pthread_cond_t _cond;
	int _available;
#endif
};

#endif // USEMAPHORE_H
