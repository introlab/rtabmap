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

#include "rtabmap/utilite/UEventsManager.h"
#include "rtabmap/utilite/UEvent.h"
#include <list>
#include "rtabmap/utilite/UStl.h"

UEventsManager* UEventsManager::instance_ = 0;
UDestroyer<UEventsManager> UEventsManager::destroyer_;

void UEventsManager::addHandler(UEventsHandler* handler)
{
	if(!handler)
	{
		UERROR("Handler is null!");
		return;
	}
	else
	{
		UEventsManager::getInstance()->_addHandler(handler);
	}
}

void UEventsManager::removeHandler(UEventsHandler* handler)
{
	if(!handler)
	{
		UERROR("Handler is null!");
		return;
	}
	else
	{
		UEventsManager::getInstance()->_removeHandler(handler);
	}
}

void UEventsManager::post(UEvent * event, bool async)
{
	if(!event)
	{
		UERROR("Event is null!");
		return;
	}
	else
	{
		UEventsManager::getInstance()->_postEvent(event, async);
	}
}

UEventsManager* UEventsManager::getInstance()
{
    if(!instance_)
    {
        instance_ = new UEventsManager();
        destroyer_.setDoomed(instance_);
        instance_->start(); // Start the thread
    }
    return instance_;
}

UEventsManager::UEventsManager()
{
}

UEventsManager::~UEventsManager()
{
   	join(true);

    // Free memory
    for(std::list<UEvent*>::iterator it=events_.begin(); it!=events_.end(); ++it)
    {
        delete *it;
    }
    events_.clear();

    handlers_.clear();

    instance_ = 0;
}

void UEventsManager::mainLoop()
{
    postEventSem_.acquire();
    if(!this->isKilled())
    {
		dispatchEvents();
    }
}

void UEventsManager::mainLoopKill()
{
    postEventSem_.release();
}

void UEventsManager::dispatchEvents()
{
    if(events_.size() == 0)
    {
        return;
    }

    std::list<UEvent*>::iterator it;
    std::list<UEvent*> eventsBuf;

    // Copy events in a buffer :
    // Other threads can post events 
    // while events are handled.
    eventsMutex_.lock();
    {
        eventsBuf = events_;
        events_.clear();
    }
    eventsMutex_.unlock();

	// Past events to handlers
	for(it=eventsBuf.begin(); it!=eventsBuf.end(); ++it)
	{
		dispatchEvent(*it);
		delete *it;
	}
    eventsBuf.clear();
}

void UEventsManager::dispatchEvent(UEvent * event)
{
	UEventsHandler * handler;
	handlersMutex_.lock();
	std::list<UEventsHandler*> handlers = handlers_;
	for(std::list<UEventsHandler*>::iterator it=handlers.begin(); it!=handlers.end(); ++it)
	{
		// Check if the handler is still in the
		// handlers_ list (may be changed if addHandler() or
		// removeHandler() is called in EventsHandler::handleEvent())
		if(std::find(handlers_.begin(), handlers_.end(), *it) != handlers_.end())
		{
			handler = *it;
			handlersMutex_.unlock();

			// To be able to add/remove an handler in a handleEvent call (without a deadlock)
			// @see _addHandler(), _removeHandler()
			handler->handleEvent(event);

			handlersMutex_.lock();
		}
	}
	handlersMutex_.unlock();

}

void UEventsManager::_addHandler(UEventsHandler* handler)
{
    if(!this->isKilled())
    {
        handlersMutex_.lock();
        {
        	//make sure it is not already in the list
        	bool handlerFound = false;
        	for(std::list<UEventsHandler*>::iterator it=handlers_.begin(); it!=handlers_.end(); ++it)
        	{
        		if(*it == handler)
        		{
        			handlerFound = true;
        		}
        	}
        	if(!handlerFound)
        	{
        		handlers_.push_back(handler);
        	}
        }
        handlersMutex_.unlock();
    }
}

void UEventsManager::_removeHandler(UEventsHandler* handler)
{
    if(!this->isKilled())
    {
        handlersMutex_.lock();
        {
            for (std::list<UEventsHandler*>::iterator it = handlers_.begin(); it!=handlers_.end(); ++it)
            {
                if(*it == handler)
                {
                    handlers_.erase(it);
                    break;
                }
            }
        }
        handlersMutex_.unlock();
    }
}

void UEventsManager::_postEvent(UEvent * event, bool async)
{
    if(!this->isKilled())
    {
    	if(async)
    	{
			eventsMutex_.lock();
			{
				events_.push_back(event);
			}
			eventsMutex_.unlock();

			// Signal the EventsManager that an Event is added
			postEventSem_.release();
    	}
    	else
    	{
    		dispatchEvent(event);
    		delete event;
    	}
    }
    else
    {
    	delete event;
    }
}
