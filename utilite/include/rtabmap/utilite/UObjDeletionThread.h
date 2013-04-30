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
