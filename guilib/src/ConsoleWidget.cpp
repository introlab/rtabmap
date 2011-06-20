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

#include "ConsoleWidget.h"
#include <utilite/ULogger.h>
#include <utilite/UEventsManager.h>
#include <QtGui/QMessageBox>

namespace rtabmap {

ConsoleWidget::ConsoleWidget(QWidget * parent) :
	QWidget(parent)
{
	_ui.setupUi(this);
	UEventsManager::addHandler(this);
	connect(this, SIGNAL(msgReceived(const QString &, int)), this, SLOT(appendMsg(const QString &, int)));
	_ui.textEdit->document()->setMaximumBlockCount(100);
	_ui.textEdit->setFontPointSize(10);
}

ConsoleWidget::~ConsoleWidget()
{
}

void ConsoleWidget::handleEvent(UEvent * anEvent)
{
	// WARNING, don't put a log message here! otherwise it could be recursively called.
	if(anEvent->getClassName().compare("ULogEvent") == 0)
	{
		ULogEvent * logEvent = (ULogEvent*)anEvent;
		emit msgReceived(logEvent->getMsg().c_str(), logEvent->getCode());
		if(logEvent->getCode() == ULogger::kFatal)
		{
			//The application will exit, so warn the user.
			QMessageBox::critical(this, tr("Fatal error occurred"), tr("Error! :\n \"%1\"\nThe application will now exit...").arg(logEvent->getMsg().c_str()), QMessageBox::Ok);
		}
	}
}

void ConsoleWidget::appendMsg(const QString & msg, int level)
{
	switch(level)
	{
	case 0:
		_ui.textEdit->setTextColor(Qt::darkGreen);
		break;
	case 2:
		_ui.textEdit->setTextColor(Qt::darkYellow);
		break;
	case 3:
	case 4:
		_ui.textEdit->setTextColor(Qt::darkRed);
		break;
	default:
		_ui.textEdit->setTextColor(Qt::black);
		break;
	}
	_ui.textEdit->append(msg);
}

}
