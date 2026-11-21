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

void UEventsManager::post(UEvent * event, bool async, const UEventsSender * sender)
{
	if(!event)
	{
		UERROR("Event is null!");
		return;
	}
	else
	{
		UEventsManager::getInstance()->_postEvent(event, async, sender);
	}
}

void UEventsManager::createPipe(
		const UEventsSender * sender,
		const UEventsHandler * receiver,
		const std::string & eventName)
{
	if(!sender || !receiver)
	{
		UERROR("Sender and/or receiver is null!");
		return;
	}
	else
	{
		UEventsManager::getInstance()->_createPipe(sender, receiver, eventName);
	}
}

void UEventsManager::removePipe(
		const UEventsSender * sender,
		const UEventsHandler * receiver,
		const std::string & eventName)
{
	if(!sender || !receiver)
	{
		UERROR("Sender and/or receiver is null!");
		return;
	}
	else
	{
		UEventsManager::getInstance()->_removePipe(sender, receiver, eventName);
	}
}

void UEventsManager::removeAllPipes(const UEventsSender * sender)
{
	if(!sender)
	{
		UERROR("Sender is null!");
		return;
	}
	else
	{
		UEventsManager::getInstance()->_removeAllPipes(sender);
	}
}

void UEventsManager::removeNullPipes(const UEventsSender * sender)
{
	if(!sender)
	{
		UERROR("Sender is null!");
		return;
	}
	else
	{
		UEventsManager::getInstance()->_removeNullPipes(sender);
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
    for(std::list<std::pair<UEvent*, const UEventsSender*> >::iterator it=events_.begin(); it!=events_.end(); ++it)
    {
        delete it->first;
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

    std::list<std::pair<UEvent*, const UEventsSender*> >::iterator it;
    std::list<std::pair<UEvent*, const UEventsSender*> > eventsBuf;

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
		if(!dispatchEvent(it->first, it->second))
		{
			delete it->first;
		}
	}
    eventsBuf.clear();
}

bool UEventsManager::dispatchEvent(UEvent * event, const UEventsSender * sender)
{
	std::list<UEventsHandler*> handlers;

	// Verify if there are pipes with the sender for his type of event
	if(sender)
	{
		handlers = getPipes(sender, event->getClassName());
	}

	handlersMutex_.lock();
	if(handlers.size() == 0)
	{
		//No pipes, send to all handlers
		handlers = handlers_;
	}

	bool handled = false;

	for(std::list<UEventsHandler*>::iterator it=handlers.begin(); it!=handlers.end() && !handled; ++it)
	{
		// Check if the handler is still in the
		// handlers_ list (may be changed if addHandler() or
		// removeHandler() is called in EventsHandler::handleEvent())
		if(std::find(handlers_.begin(), handlers_.end(), *it) != handlers_.end())
		{
			UEventsHandler * handler = *it;

			// Don't process event if the handler is the same as the sender
			if(handler != sender)
			{
				// Keep handlersMutex_ held across handleEvent() so a concurrent
				// removeHandler() in another thread cannot return while this
				// dispatch is in flight (which would let the handler be
				// destroyed under us). The mutex is recursive, so a handler
				// that calls addHandler()/removeHandler() from within
				// handleEvent() still works.
				handled = handler->handleEvent(event);
			}
		}
	}
	handlersMutex_.unlock();
	return handled;
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

        pipesMutex_.lock();
        {
        	for(std::list<Pipe>::iterator iter=pipes_.begin(); iter!= pipes_.end(); ++iter)
			{
				if(iter->receiver_ == handler)
				{
					iter->receiver_ = 0; // set to null
				}
			}
        }
        pipesMutex_.unlock();
    }
}

void UEventsManager::_postEvent(UEvent * event, bool async, const UEventsSender * sender)
{
    if(!this->isKilled())
    {
    	if(async)
    	{
			eventsMutex_.lock();
			{
				events_.push_back(std::make_pair(event, sender));
			}
			eventsMutex_.unlock();

			// Signal the EventsManager that an Event is added
			postEventSem_.release();
    	}
    	else
    	{
    		if(!dispatchEvent(event, sender))
    		{
    			delete event;
    		}
    	}
    }
    else
    {
    	delete event;
    }
}

std::list<UEventsHandler*> UEventsManager::getPipes(
		const UEventsSender * sender,
		const std::string & eventName)
{
	std::list<UEventsHandler*> pipes;
	pipesMutex_.lock();

	for(std::list<Pipe>::iterator iter=pipes_.begin(); iter!= pipes_.end(); ++iter)
	{
		if(iter->sender_ == sender && iter->eventName_.compare(eventName) == 0)
		{
			bool added = false;
			if(iter->receiver_)
			{
				handlersMutex_.lock();
				for(std::list<UEventsHandler*>::iterator jter=handlers_.begin(); jter!=handlers_.end(); ++jter)
				{
					if(*jter == iter->receiver_)
					{
						pipes.push_back(*jter);
						added = true;
						break;
					}
				}
				handlersMutex_.unlock();
			}
			if(!added)
			{
				// Add nulls
				pipes.push_back(0);
			}
		}
	}

	pipesMutex_.unlock();
	return pipes;
}

void UEventsManager::_createPipe(
		const UEventsSender * sender,
		const UEventsHandler * receiver,
		const std::string & eventName)
{
	pipesMutex_.lock();
	bool exist = false;
	for(std::list<Pipe>::iterator iter=pipes_.begin(); iter!= pipes_.end();++iter)
	{
		if(iter->sender_ == sender && iter->receiver_ == receiver && iter->eventName_.compare(eventName) == 0)
		{
			exist = true;
			break;
		}
	}

	if(!exist)
	{
		bool handlerFound = false;
		handlersMutex_.lock();
		for(std::list<UEventsHandler*>::iterator iter=handlers_.begin(); iter!=handlers_.end(); ++iter)
		{
			if(*iter == receiver)
			{
				handlerFound = true;
				break;
			}
		}
		handlersMutex_.unlock();
		if(handlerFound)
		{
			pipes_.push_back(Pipe(sender, receiver, eventName));
		}
		else
		{
			UERROR("Cannot create the pipe because the receiver is not yet "
				   "added to UEventsManager's handlers list.");
		}
	}
	else
	{
		UWARN("Pipe between sender %p and receiver %p with event %s was already created.",
				sender, receiver, eventName.c_str());
	}
	pipesMutex_.unlock();
}

void UEventsManager::_removePipe(
		const UEventsSender * sender,
		const UEventsHandler * receiver,
		const std::string & eventName)
{
	pipesMutex_.lock();

	bool removed = false;
	for(std::list<Pipe>::iterator iter=pipes_.begin(); iter!= pipes_.end();)
	{
		if(iter->sender_ == sender && iter->receiver_ == receiver && iter->eventName_.compare(eventName) == 0)
		{
			iter = pipes_.erase(iter);
			removed = true;
		}
		else
		{
			++iter;
		}
	}

	if(!removed)
	{
		UWARN("Pipe between sender %p and receiver %p with event %s didn't exist.",
				sender, receiver, eventName.c_str());
	}

	pipesMutex_.unlock();
}

void UEventsManager::_removeAllPipes(const UEventsSender * sender)
{
	pipesMutex_.lock();
	for(std::list<Pipe>::iterator iter=pipes_.begin(); iter!=pipes_.end();)
	{
		if(iter->sender_ == sender)
		{
			iter = pipes_.erase(iter);
		}
		else
		{
			++iter;
		}
	}
	pipesMutex_.unlock();
}

void UEventsManager::_removeNullPipes(const UEventsSender * sender)
{
	pipesMutex_.lock();
	for(std::list<Pipe>::iterator iter=pipes_.begin(); iter!=pipes_.end();)
	{
		if(iter->receiver_ == 0)
		{
			iter = pipes_.erase(iter);
		}
		else
		{
			++iter;
		}
	}
	pipesMutex_.unlock();
}
