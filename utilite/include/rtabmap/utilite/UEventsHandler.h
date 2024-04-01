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

#ifndef UEVENTSHANDLER_H
#define UEVENTSHANDLER_H

#include "rtabmap/utilite/utilite_export.h" // DLL export/import defines

#include "rtabmap/utilite/UEventsSender.h"
class UEvent;

/**
 * The class UEventsHandler is an abstract class for
 * handling events.
 *
 * Inherited classes must implement handleEvent() function, which
 * is called by the UEventsManager when an event is dispatched. Once the handler is
 * created, it must be added to events manager with UEventsManager::addHandler() function.
 * Note that it is not safe to automatically add the handler to UEventsManager in the handler's constructor.
 *
 * Note for multi-threading: the handleEvent() method is called
 * inside the UEventsManager thread. If the inherited class also inherits
 * from UThreadNode, handleEvent() is done as well outside the thread's main loop, so
 * be careful to protect private data of the thread used in its main loop.
 *
 * Example for a useful combination of an UEventsHandler and a UThreadNode, with safe data
 * modification while not blocking the handleEvent() call on a mutex:
 * @code
 * #include "utilite/UThreadNode.h"
 * #include "utilite/UEventsHandler.h"
 * #include "utilite/UEventsManager.h"
 * #include "utilite/UEvent.h"
 *
 * // Implement a simple event
 * class ResetEvent : public UEvent {
 * public:
 *    ResetEvent() {}
 *    virtual ~ResetEvent() {}
 *    virtual std::string getClassName() const {return "ResetEvent";} // Must be implemented
 * };
 *
 * // There is the thread counting indefinitely, the count can be reseted by sending a ResetEvent.
 * class CounterThread : public UThreadNode, public UEventsHandler {
 * public:
 *    CounterThread() : state_(0), count_(0) {}
 *    virtual ~CounterThread() {this->join(true);}
 *
 * protected:
 *    virtual void mainLoop() {
 *       if(state_ == 1) {
 *          state_ = 0;
 *          // Do a long initialization, reset memory or other special long works... here
 *          // we reset the count. This could be done in the handleEvent() but
 *          // with many objects, it is more safe to do it here (in the thread's loop). A safe
 *          // way could be also to use a UMutex to protect this initialization in
 *          // the handleEvent(), but it is not recommended to do long works in handleEvent()
 *          // because this will add latency in the UEventsManager dispatching events loop.
 *          count_ = 0; // Reset the count
 *          printf("Reset!\n");
 *       }
 *
 *       // Do some works...
 *       printf("count=%d\n", count_++);
 *       uSleep(100); // wait 100 ms
 *    }
 *    virtual void handleEvent(UEvent * event) {
 *       if(event->getClassName().compare("ResetEvent") == 0) {
 *          state_ = 1;
 *       }
 *    }
 * private:
 *    int state_;
 *    int count_;
 * };
 *
 * int main(int argc, char * argv[])
 * {
 *    CounterThread counter;
 *    counter.start();
 *    UEventsManager::addHandler(&counter);
 *
 *    uSleep(500); // wait 500 ms before sending a reset event
 *    UEventsManager::post(new ResetEvent());
 *    uSleep(500); // wait 500 ms before termination
 *
 *    UEventsManager::removeHandler(&counter);
 *    counter.join(true); // Kill and wait to finish
 *    return 0;
 * }
 * @endcode
 *
 * The output is:
 * @code
 * count=0
 * count=1
 * count=2
 * count=3
 * count=4
 * Reset!
 * count=0
 * count=1
 * count=2
 * count=3
 * count=4
 * @endcode
 *
 * @see UEventsManager
 * @see UEvent
 * @see UThreadNode
 *
 */
class UTILITE_EXPORT UEventsHandler : public UEventsSender {
public:

	void registerToEventsManager();
	void unregisterFromEventsManager();
    

protected:
    /**
     * Only the UEventsManager has access
     * to the handleEvent() method.
     */
    friend class UEventsManager;

    /**
     * Method called by the UEventsManager
     * to handle an event. Important : this method 
     * must do a minimum of work because the faster 
     * the dispatching loop is done; the faster the 
     * events are received. If a handling function 
     * takes too much time, the events list can grow 
     * faster than it is emptied. The event can be
     * modified.
     * @return "true" to notify UEventsManager that this handler took ownership of the
     *         event (meaning it must delete it). The event will
     *         not be dispatched to next handlers.
     * @return "false" to let event be dispatched to next handlers (default behavior). UEventsManager
     *         will take care of deleting the event.
     *
     */
    virtual bool handleEvent(UEvent * event) = 0;

protected:
    /**
     * UEventsHandler constructor.
     *
     * Note : You can call EventsManager::addHandler(this) at
     * the end of the constructor of the inherited class where the virtual
     * method handleEvent(...) is defined. If so, the UEventsHandler doesn't
     * need to be manually added to the EventsManager where the handler
     * is instantiated. We decided to not include UEventsManager::addHandler(this)
     * in this abstract class constructor because an event can be handled (calling
     * the pure virtual method) while the concrete class is constructed.
     */
    UEventsHandler() {}

    /**
     * UEventsHandler destructor.
     *
     * By default, it removes the handler reference from the UEventsManager. To be thread-safe,
     * the inherited class must remove itself from the UEventsManager before it is deleted because
     * an event can be handled (calling the pure virtual method handleEvent()) after the concrete class
     * is deleted.
     */
    virtual ~UEventsHandler();

private:
    
};

#endif // UEVENTSHANDLER_H
