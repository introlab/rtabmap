/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef OBJDELETIONHANDLER_H_
#define OBJDELETIONHANDLER_H_

#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/UEvent.h"
#include <QtCore/QObject>

class ObjDeletionHandler : public QObject, public UEventsHandler
{
	Q_OBJECT

public:
	ObjDeletionHandler(int watchedId, QObject * receiver = 0, const char * member = 0) : _watchedId(watchedId)
	{
		if(receiver && member)
		{
			connect(this, SIGNAL(objDeletionEventReceived(int)), receiver, member);
		}
	}
	virtual ~ObjDeletionHandler() {}

signals:
	void objDeletionEventReceived(int);

protected:
	virtual void handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("UObjDeletedEvent") == 0 &&
		   event->getCode() == _watchedId)
		{
			emit objDeletionEventReceived(_watchedId);
		}
	}
private:
	int _watchedId;
};

#endif /* OBJDELETIONHANDLER_H_ */
