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

#ifndef UMUTEX_H
#define UMUTEX_H

#include <errno.h>

#ifdef _WIN32
  #include "rtabmap/utilite/Win32/UWin32.h"
#else
  #include <pthread.h>
#endif


/**
 * A mutex class.
 *
 * On a lock() call, the calling thread is blocked if the
 * UMutex was previously locked by another thread. It is unblocked when unlock() is called.
 *
 * On Unix (not yet tested on Windows), UMutex is recursive: the same thread can
 * call multiple times lock() without being blocked.
 *
 * Example:
 * @code
 * UMutex m; // Mutex shared with another thread(s).
 * ...
 * m.lock();
 * // Data is protected here from the second thread
 * //(assuming the second one protects also with the same mutex the same data).
 * m.unlock();
 *
 * @endcode
 *
 * @see USemaphore
 */
class UMutex
{

public:

	/**
	 * The constructor.
	 */
	UMutex()
	{
#ifdef _WIN32
		InitializeCriticalSection(&C);
#else
		pthread_mutexattr_t attr;
		pthread_mutexattr_init(&attr);
		pthread_mutexattr_settype(&attr,PTHREAD_MUTEX_RECURSIVE);
		pthread_mutex_init(&M,&attr);
		pthread_mutexattr_destroy(&attr);
#endif
	}

	virtual ~UMutex()
	{
#ifdef _WIN32
		DeleteCriticalSection(&C);
#else
		pthread_mutex_unlock(&M); pthread_mutex_destroy(&M);
#endif
	}

	/**
	 * Lock the mutex.
	 * @return 0 on success, an error code otherwise.
	 */
	int lock() const
	{
#ifdef _WIN32
		EnterCriticalSection(&C); return 0;
#else
		return pthread_mutex_lock(&M);
#endif
	}

	/**
	 * Try locking the mutex.
	 * @return 0 if the mutex has been locked by this call, EBUSY (or another
	 *         error code) otherwise.
	 */
#ifdef _WIN32
	#if(_WIN32_WINNT >= 0x0400)
	int lockTry() const
	{
		return (TryEnterCriticalSection(&C)?0:EBUSY);
	}
	#endif
#else
	int lockTry() const
	{
		return pthread_mutex_trylock(&M);
	}
#endif

	/**
	 * Unlock the mutex.
	 * @return 0 on success, an error code otherwise.
	 */
	int unlock() const
	{
#ifdef _WIN32
		LeaveCriticalSection(&C); return 0;
#else
		return pthread_mutex_unlock(&M);
#endif
	}

	private:
#ifdef _WIN32
		mutable CRITICAL_SECTION C;
#else
		mutable pthread_mutex_t M;
#endif
		void operator=(UMutex &) {}
		UMutex( const UMutex & ) {}
};

/**
 * Automatically lock the referenced mutex on constructor and unlock mutex on destructor.
 *
 * Example:
 * @code
 * UMutex m; // Mutex shared with another thread(s).
 * ...
 * int myMethod()
 * {
 *    UScopeMutex sm(m); // automatically lock the mutex m
 *    if(cond1)
 *    {
 *       return 1; // automatically unlock the mutex m
 *    }
 *    else if(cond2)
 *    {
 *       return 2; // automatically unlock the mutex m
 *    }
 *    return 0; // automatically unlock the mutex m
 * }
 *
 * @endcode
 *
 * The lock can also be deferred, for example to only try locking it. The destructor
 * then unlocks the mutex only if this object locked it:
 * @code
 * void callback()
 * {
 *    UScopeMutex sm(m, false); // not locked yet
 *    if(sm.lockTry() == 0)
 *    {
 *       if(cond1)
 *       {
 *          return; // automatically unlock the mutex m
 *       }
 *       ...
 *    }
 *    // the mutex m is unlocked only if lockTry() succeeded
 * }
 * @endcode
 *
 * @see UMutex
 */
class UScopeMutex
{
public:
	/**
	 * @param mutex the mutex to lock.
	 * @param lockNow if true (default), the mutex is locked here. If false, it is not
	 *        locked until lock() or lockTry() is called.
	 */
	UScopeMutex(const UMutex & mutex, bool lockNow = true) :
		mutex_(mutex),
		locked_(false)
	{
		if(lockNow)
		{
			lock();
		}
	}
	// backward compatibility
	UScopeMutex(UMutex * mutex) :
		mutex_(*mutex),
		locked_(false)
	{
		lock();
	}
	/**
	 * Unlock the mutex, only if this object locked it.
	 */
	~UScopeMutex()
	{
		unlock();
	}

	/**
	 * Lock the mutex, if this object doesn't hold it already.
	 * @return 0 on success, an error code otherwise.
	 */
	int lock()
	{
		if(locked_)
		{
			return 0;
		}
		int r = mutex_.lock();
		locked_ = r == 0;
		return r;
	}

#if !defined(_WIN32) || (_WIN32_WINNT >= 0x0400)
	/**
	 * Try locking the mutex, if this object doesn't hold it already.
	 * @return 0 if the mutex is held by this object, EBUSY (or another
	 *         error code) otherwise.
	 */
	int lockTry()
	{
		if(locked_)
		{
			return 0;
		}
		int r = mutex_.lockTry();
		locked_ = r == 0;
		return r;
	}
#endif

	/**
	 * Unlock the mutex before this object goes out of scope, only if this object locked it.
	 * @return 0 on success (or if this object didn't hold the mutex), an error code otherwise.
	 */
	int unlock()
	{
		if(!locked_)
		{
			return 0;
		}
		locked_ = false;
		return mutex_.unlock();
	}

	/**
	 * @return true if this object currently holds the mutex.
	 */
	bool isLocked() const
	{
		return locked_;
	}

private:
	UScopeMutex(const UScopeMutex &);
	void operator=(const UScopeMutex &);

private:
	const UMutex & mutex_;
	bool locked_;
};

#endif // UMUTEX_H
