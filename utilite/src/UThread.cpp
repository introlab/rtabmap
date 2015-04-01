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

#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/ULogger.h"
#ifdef __APPLE__
#include <mach/thread_policy.h>
#include <mach/mach.h>
#endif

#define PRINT_DEBUG 0

////////////////////////////
// public:
////////////////////////////

UThread::UThread(Priority priority) : 
    state_(kSIdle), 
    priority_(priority), 
    handle_(0), 
    threadId_(0),
    cpuAffinity_(-1)
{}

UThread::~UThread()
{
#if PRINT_DEBUG
	ULOGGER_DEBUG("");
#endif
}

void UThread::kill()
{
#if PRINT_DEBUG
	ULOGGER_DEBUG("");
#endif
    killSafelyMutex_.lock();
    {
    	if(this->isRunning())
    	{
    		// Thread is creating
    		while(state_ == kSCreating)
			{
				uSleep(1);
			}

			if(state_ == kSRunning)
			{
				state_ = kSKilled;

				// Call function to do something before wait
				mainLoopKill();
			}
			else
			{
				UERROR("thread (%d) is supposed to be running...", threadId_);
			}
    	}
    	else
    	{
#if PRINT_DEBUG
    		UDEBUG("thread (%d) is not running...", threadId_);
#endif
    	}
    }
    killSafelyMutex_.unlock();
}

void UThread::join(bool killFirst)
{
	//make sure the thread is created
	while(this->isCreating())
	{
		uSleep(1);
	}

#ifdef _WIN32
#if PRINT_DEBUG
	UDEBUG("Thread %d joining %d", UThreadC<void>::Self(), threadId_);
#endif
	if(UThreadC<void>::Self() == threadId_)
#else
#if PRINT_DEBUG
	UDEBUG("Thread %d joining %d", UThreadC<void>::Self(), handle_);
#endif
	if(UThreadC<void>::Self() == handle_)
#endif
	{
		UERROR("Thread cannot join itself");
		return;
	}

	if(killFirst)
	{
		this->kill();
	}

	runningMutex_.lock();
	runningMutex_.unlock();

#if PRINT_DEBUG
	UDEBUG("Join ended for %d", UThreadC<void>::Self());
#endif
}

void UThread::start()
{
#if PRINT_DEBUG
	ULOGGER_DEBUG("");
#endif

    if(state_ == kSIdle || state_ == kSKilled)
    {
    	if(state_ == kSKilled)
    	{
			// make sure it is finished
			runningMutex_.lock();
			runningMutex_.unlock();
    	}

        state_ = kSCreating;
        int r = UThreadC<void>::Create(threadId_, &handle_, true); // Create detached
        if(r)
        {
        	UERROR("Failed to create a thread! errno=%d", r);
        	threadId_=0;
        	handle_=0;
        	state_ = kSIdle;
        }
        else
        {
#if PRINT_DEBUG
        ULOGGER_DEBUG("StateThread::startThread() thread id=%d _handle=%d", threadId_, handle_);
#endif
        }
    }
}

//TODO : Support pThread
void UThread::setPriority(Priority priority)
{
	priority_ = priority;
}

//TODO : Support pThread
void UThread::applyPriority()
{
    if(handle_)
    {
#ifdef _WIN32
        int p = THREAD_PRIORITY_NORMAL;
        switch(priority_)
        {
            case kPLow:
                p = THREAD_PRIORITY_LOWEST;
                break;

            case kPBelowNormal:
                p = THREAD_PRIORITY_BELOW_NORMAL;
                break;

            case kPNormal:
                p = THREAD_PRIORITY_NORMAL;
                break;

            case kPAboveNormal:
                p = THREAD_PRIORITY_ABOVE_NORMAL;
                break;

            case kPRealTime:
                p = THREAD_PRIORITY_TIME_CRITICAL;
                break;

            default:
                break;
        }
        SetThreadPriority(handle_, p);
#endif
    }
}

void UThread::setAffinity(int cpu)
{
	cpuAffinity_ = cpu;
	if(cpuAffinity_<0)
	{
		cpuAffinity_ = 0;
	}
}

//TODO : Support Windows and linux
void UThread::applyAffinity()
{
	if(cpuAffinity_>0)
	{
#ifdef _WIN32
#elif __APPLE__
		thread_affinity_policy_data_t affPolicy;
		affPolicy.affinity_tag = cpuAffinity_;
		kern_return_t ret = thread_policy_set(
				mach_thread_self(),
				THREAD_AFFINITY_POLICY,
				(integer_t*) &affPolicy,
				THREAD_AFFINITY_POLICY_COUNT);
		if(ret != KERN_SUCCESS)
		{
			UERROR("thread_policy_set returned %d", ret);
		}
#else
		/*unsigned long mask = cpuAffinity_;

		if (pthread_setaffinity_np(
			pthread_self(),
			sizeof(mask),
			&mask) <0)
		{
			UERROR("pthread_setaffinity_np failed");
		}
		}*/
#endif
	}
}

bool UThread::isCreating() const
{
    return state_ == kSCreating;
}

bool UThread::isRunning() const
{
    return state_ == kSRunning || state_ == kSCreating;
}

bool UThread::isIdle() const
{
    return state_ == kSIdle;
}

bool UThread::isKilled() const
{
    return state_ == kSKilled;
}

////////////////////////////
// private:
////////////////////////////

void UThread::ThreadMain()
{
	runningMutex_.lock();
	applyPriority();
	applyAffinity();

#if PRINT_DEBUG
	ULOGGER_DEBUG("before mainLoopBegin()");
#endif

	state_ = kSRunning;
    mainLoopBegin();

#if PRINT_DEBUG
	ULOGGER_DEBUG("before mainLoop()");
#endif

	while(state_ == kSRunning)
	{
		mainLoop();
	}

#if PRINT_DEBUG
	ULOGGER_DEBUG("before mainLoopEnd()");
#endif

	mainLoopEnd();

    handle_ = 0;
    threadId_ = 0;
    state_ = kSIdle;

    runningMutex_.unlock();
#if PRINT_DEBUG
	ULOGGER_DEBUG("Exiting thread loop");
#endif
}

