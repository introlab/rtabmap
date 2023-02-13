/*
 * UEventsSender.h
 *
 *  Created on: 2013-10-14
 *      Author: Mathieu
 */

#ifndef UEVENTSSENDER_H_
#define UEVENTSSENDER_H_

#include "rtabmap/utilite/utilite_export.h" // DLL export/import defines

class UEvent;

class UTILITE_EXPORT UEventsSender
{
public:
	UEventsSender(){}
	virtual ~UEventsSender();

protected:

	/**
	 * For convenience to post an event. This is the same than calling UEventsManager::post()
	 * with the sender reference.
	 */
	void post(UEvent * event, bool async = true) const;
};


#endif /* UEVENTSSENDER_H_ */
