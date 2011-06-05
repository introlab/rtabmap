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

#ifndef CONSOLEWIDGET_H_
#define CONSOLEWIDGET_H_

#include "ui_consoleWidget.h"
#include <utilite/UEventsHandler.h>

namespace rtabmap {

class ConsoleWidget : public QWidget, public UEventsHandler
{
	Q_OBJECT;

public:
	ConsoleWidget(QWidget * parent = 0);
	virtual ~ConsoleWidget();

public slots:
	void appendMsg(const QString & msg, int level);

signals:
	void msgReceived(const QString &, int);

protected:
	virtual void handleEvent(UEvent * anEvent);

private:
	Ui_consoleWidget _ui;

};

}

#endif /* CONSOLEWIDGET_H_ */
