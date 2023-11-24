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

#ifndef UTHREADNODE_H
#define UTHREADNODE_H

#include "rtabmap/utilite/utilite_export.h" // DLL export/import defines

#include "rtabmap/utilite/UThreadC.h"

/**
 * The class UThread is an abstract class for creating thread objects.
 * A UThread provides methods to create threads as an object-style fashion.
 *
 * For most of inherited classes, only mainLoop() needs to be implemented, then only start() needs
 * to be called from the outside.
 * The main loop is called until the thread itself calls kill() or another thread
 * calls kill() or join() (with parameter to true) on this thread. Unlike kill(), join() is a blocking call:
 * the calling thread will wait until this thread has finished, thus join() must not be
 * called inside the mainLoop().
 *
 * If inside the mainLoop(), at some time, the thread needs to wait on a mutex/semaphore
 * (like for the acquisition of a resource), the function mainLoopKill() should be
 * implemented to release the mutex/semaphore when the thread is killed, to avoid a deadlock.
 * The function killCleanup() is called after the thread's state is set to kSKilled.
 * After the mutex/semaphore is released in killCleanup(), on wake up, the thread can know if
 * it needs to stop by calling isKilled().
 *
 * To do an initialization process (executed by the worker thread) just one time before
 * entering the mainLoop(), mainLoopBegin() can be implemented.
 *
 * Example:
 * @code
 * #include "utilite/UThread.h"
 * class SimpleThread : public UThread
 * {
 * public:
 * 	SimpleThread() {}
 * 	virtual ~SimpleThread() {
 * 		// The calling thread will wait until this thread has finished.
 * 		this->join(true);
 *	}
 *
 * protected:
 * 	virtual void mainLoop() {
 * 		// Do some works...
 *
 * 		// This will stop the thread, otherwise the mainLoop() is recalled.
 *		this->kill();
 * 	}
 * };
 *
 * int main(int argc, char * argv[])
 * {
 *	SimpleThread t;
 *	t.start();
 *	t.join(); // Wait until the thread has finished.
 * 	return 0;
 * }
 * @endcode
 *
 * @see start()
 * @see kill()
 * @see join()
 * @see mainLoopBegin()
 * @see mainLoopKill()
 * @see mainLoop()
 *
 */
class UTILITE_EXPORT UThread : public UThreadC<void>
{
public:
    /**
     * Enum of priorities : kPLow, kPBelowNormal, kPNormal, kPAboveNormal, kPRealTime.
     */
    enum Priority{kPLow, kPBelowNormal, kPNormal, kPAboveNormal, kPRealTime};

public:
    //return caller thread id
    static unsigned long currentThreadId() {return (unsigned long)UThreadC<void>::Self();}

public:
    /**
     * The constructor.
     * @see Priority
     * @param priority the thread priority
     */
    UThread(Priority priority = kPNormal);

    /**
     * The destructor. Inherited classes must call join(true) inside their destructor
     * to avoid memory leaks where the underlying c-thread is still running.
     *
     * Note: not safe to delete a thread while other threads are joining it.
     */
    virtual ~UThread();

    /**
     * Start the thread. Once the thread is started, subsequent calls
     * to start() are ignored until the thread is killed.
     * @see kill()
     */
    void start();

    /**
	 * Kill the thread.
	 * This functions does nothing if the thread is not started or is killed.
	 *
	 * Note : not a blocking call
	 */
	void kill();

	/**
	 * The caller thread will wait until the thread has finished.
	 *
	 * Note : blocking call
	 * @param killFirst if you want kill() to be called before joining (default false), otherwise not.
	 */
	void join(bool killFirst = false);

    /**
     * Set the thread priority.
     * @param priority the priority
     */
    void setPriority(Priority priority);

    /**
	 * Set the thread affinity. This is applied during start of the thread.
	 *
	 * MAC OS X : http://developer.apple.com/library/mac/#releasenotes/Performance/RN-AffinityAPI/_index.html.
	 * @param cpu the cpu id (start at 1), 0 means no affinity (default).
	 */
	void setAffinity(int cpu = 0);

	/**
	 * @return if the state of the thread is kSCreating (after start() is called but before entering the mainLoop()).
	 */
    bool isCreating() const;

    /**
	 * @return if the state of the thread is kSRunning (it is executing the mainLoop()) or kSCreating.
	 */
    bool isRunning() const;

    /**
	 * @return if the state of the thread is kSIdle (before start() is called and after the thread is totally killed (or after join(true))).
	 */
    bool isIdle() const;

    /**
	 * @return if the state of the thread is kSKilled (after kill() is called and before the thread is totally killed).
	 */
    bool isKilled() const;

    Handle getThreadHandle() const {return handle_;}
    unsigned long getThreadId() const {return threadId_;}

protected:

private:
	/**
	 * Virtual method mainLoopBegin().
	 * User can implement this function to add a behavior
	 * before the main loop is started. It is
	 * called once (before entering mainLoop()).
	 */
	virtual void mainLoopBegin() {}

	/**
	 * Pure virtual method mainLoop().
	 * The inner loop of the thread. This method is called repetitively
	 * until the thread is killed. Note that if kill() is called in mainLoopBegin(),
	 * mainLoop() is not called, terminating immediately the thread.
	 *
	 * @see mainLoop()
	 * @see kill()
	 */
	virtual void mainLoop() = 0;

	/**
	 * Virtual method mainLoopKill().
	 * User can implement this function to add a behavior
	 * before the thread is killed. When this
	 * function is called, the state of the thread is set to kSKilled. It is useful to
	 * wake up a sleeping thread to finish his loop and to avoid a deadlock.
	 */
	virtual void mainLoopKill() {}

	/**
	 * Virtual method mainLoopEnd().
	 * User can implement this function to add a behavior
	 * after the thread is killed (after exiting the mainLoop(), work is
	 * still done in the thread before exiting).
	 */
	virtual void mainLoopEnd() {}

    /*
     * Inherited method ThreadMain() from Thread.
     * @see Thread<void>
     */
    void ThreadMain();

    /*
	 * Apply thread priority. This is called when starting the thread.
	 * *@todo : Support pthread
	 */
	void applyPriority();

	/*
	 * Apply cpu affinity. This is called when starting the thread.
	 * *@todo : Support Windows
	 */
	void applyAffinity();

    /*
     * Inherited method Create() from Thread.
     * Here we force this function to be private so the
     * inherited class can't have access to it.
     * @see Thread<void>
     */
    int Create(
      Handle  * const     & H               = 0,
      const bool          & CreateDetached  = false,
      const unsigned int  & StackSize       = 0,
      const bool          & CancelEnable    = false,   // UNUSED
      const bool          & CancelAsync     = false    // UNUSED
    ) const;

    //Methods from UThread<void> class hided
    static int Join( Handle H )
	  { return UThreadC<void>::Join(H); }
#ifndef ANDROID
	static int Kill( Handle H )
	  { return UThreadC<void>::Kill(H); }
#endif
	static int Detach( Handle H )
	  { return UThreadC<void>::Detach(H); }

private:
	void operator=(UThread &) {}
	UThread( const UThread &) : state_(kSIdle) {}

private:
    enum State{kSIdle, kSCreating, kSRunning, kSKilled}; /* Enum of states. */
    State state_; 			/* The thread state. */
    Priority priority_; 	/* The thread priority. */
    Handle handle_; 	/* The thread handle. */
    unsigned long threadId_; /* The thread id. */
    int cpuAffinity_; /* The cpu affinity. */
    UMutex killSafelyMutex_;	/* Mutex used to protect the kill() method. */
    UMutex runningMutex_;	    /* Mutex used to notify the join method when the thread has finished. */
};

#endif // UTHREADNODE_H
