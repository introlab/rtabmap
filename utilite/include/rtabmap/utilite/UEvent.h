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
