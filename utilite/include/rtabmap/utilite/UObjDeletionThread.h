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

#ifndef UOBJDELETIONTHREAD_H
#define UOBJDELETIONTHREAD_H

#include "rtabmap/utilite/UThreadNode.h"
#include "rtabmap/utilite/UEvent.h"
#include "rtabmap/utilite/UEventsManager.h"

/**
 * Event used by UObjDeletionThread to notify when its object is deleted. It contains
 * the object id used for deletion (can be retrieved by UEvent::getCode()).
 */
class UObjDeletedEvent : public UEvent
{
public:
	UObjDeletedEvent(int objDeletionThreadId) : UEvent(objDeletionThreadId) {}
	virtual ~UObjDeletedEvent() {}
	/**
	 * @return string "UObjDeletedEvent"
	 */
	virtual std::string getClassName() const {return std::string("UObjDeletedEvent");}
};

/**
 * This class can be used to delete a dynamically created object in another thread. Give the
 * dynamic reference to object to it and it will notify with a UObjDeletedEvent when the object is deleted.
 * The deletion can be delayed on startDeletion(), the thread will wait the time given before deleting the object.
 */
template<class T>
class UObjDeletionThread : public UThread
{
public:
	/**
	 * The constructor.
	 * @param obj the object to delete
	 * @param id the custom id which will be sent in a event UObjDeletedEvent after the object is deleted
	 */
	UObjDeletionThread(T * obj, int id=0) :
		obj_(obj),
		id_(id),
		waitMs_(0) {}

	/**
	 * The destructor. If this thread is not started but with an object set, the
	 * object is deleted. If the thread has not finished to delete the object, the
	 * calling thread will wait (on a UThreadNode::join()) until the object is deleted.
	 * @param obj the object to delete
	 * @param id the custom id which will be sent in a event UObjDeletedEvent after the object is deleted
	 */
	virtual ~UObjDeletionThread()
	{
		join(true);
		if(obj_)
		{
			delete obj_;
		}
	}

	/**
	 * Start the thread after optional delay.
	 * @param waitMs the delay before deletion
	 */
	void startDeletion(int waitMs = 0) {waitMs_ = waitMs; this->start();}

	/**
	 * Get id of the deleted object.
	 * @return the id
	 */
	int id() const {return id_;}

	/**
	 * Set a new object, if one was already set, the old one is deleted.
	 * @param obj the object to delete
	 */
	void setObj(T * obj)
	{
		join();
		if(obj_)
		{
			delete obj_;
			obj_ = 0;
		}
		obj_ = obj;
	}

private:
	/**
	 * Thread main loop...
	 */
	virtual void mainLoop()
	{
		if(waitMs_)
		{
			uSleep(waitMs_);
		}
		if(obj_)
		{
			delete obj_;
			obj_ = 0;
		}
		this->kill();
		UEventsManager::post(new UObjDeletedEvent(id_), false);
	}

private:
	T * obj_;
	int id_;
	int waitMs_;
};

#endif /* UOBJDELETIONTHREAD_H */
