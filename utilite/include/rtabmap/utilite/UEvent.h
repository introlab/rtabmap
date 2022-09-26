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

#ifndef UEVENT_H
#define UEVENT_H

#include "rtabmap/utilite/utilite_export.h" // DLL export/import defines

#include <string>

class UEventsHandler;

/**
 * This is the base class for all events used 
 * with the UEventsManager. Inherited classes
 * must redefined the virtual method getClassName() 
 * to return their class name.
 *
 * Example:
 * @code
 *  class MyEvent : public UEvent
 *  {
 *  public:
 *     MyEvent() {}
 *     virtual ~MyEvent() {}
 *     std::string getClassName() const {return "MyEvent";}
 *  };
 *
 *  int main(int argc, char * argv[])
 *  {
 *  	...
 *  	UEventsManager::post(new MyEvent()); // UEventsManager take ownership of the event (deleted by UEventsManager).
 *  	...
 *  }
 * @endcode
 *
 * @see UEventsManager
 * @see UEventsHandler
 * @see getClassName()
 */
class UTILITE_EXPORT UEvent{
public:
    virtual ~UEvent() {}

    /**
     * This method is used to get the class name 
     * of the event. For example, if a class MouseEvent
     * inherits from UEvent, it must return the
     * "MouseEvent" string.
     * @return the class name
     */
    virtual std::string getClassName() const = 0; // TODO : macro?

    /**
     * Get event's code.
     * @return the code
     */
    int getCode() const {return code_;}

protected:
    /**
	 * @param code the event code.
	 * TODO : Remove the code, not required for most of all implemented events
	 */
	UEvent(int code = 0) : code_(code) {}

private:
    int code_; /**< The event's code. */
};

#endif // UEVENT_H
