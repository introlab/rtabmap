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
#include "ui_consoleWidget.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <QtGui/QMessageBox>
#include <QtGui/QTextCursor>
#include <QtCore/QTimer>

#define OLD_TIME 1000 //ms
#define MAXIMUM_ITEMS 100

namespace rtabmap {

ConsoleWidget::ConsoleWidget(QWidget * parent) :
	QWidget(parent)
{
	_ui = new Ui_consoleWidget();
	_ui->setupUi(this);
	UEventsManager::addHandler(this);
	_ui->textEdit->document()->setMaximumBlockCount(MAXIMUM_ITEMS);
	_textCursor = new QTextCursor(_ui->textEdit->document());
	_ui->textEdit->setFontPointSize(10);
	QPalette p(_ui->textEdit->palette());
	p.setColor(QPalette::Base, Qt::black);
	_ui->textEdit->setPalette(p);
	_errorMessage = new QMessageBox(QMessageBox::Critical, tr("Fatal error occurred"), "", QMessageBox::Ok, this);
	_errorMessageMutex.lock();
	_time.start();
	_timer.setSingleShot(true);
	connect(_ui->pushButton_clear, SIGNAL(clicked()), _ui->textEdit, SLOT(clear()));
	connect(this, SIGNAL(msgReceived(const QString &, int)), this, SLOT(appendMsg(const QString &, int)));
	connect(&_timer, SIGNAL(timeout()), this, SLOT(flushConsole()));
}

ConsoleWidget::~ConsoleWidget()
{
	delete _ui;
}

void ConsoleWidget::handleEvent(UEvent * anEvent)
{
	// WARNING, don't put a log message here! otherwise it could be recursively called.
	if(anEvent->getClassName().compare("ULogEvent") == 0)
	{
		ULogEvent * logEvent = (ULogEvent*)anEvent;
		_msgListMutex.lock();
		_msgList.append(QPair<QString, int>(logEvent->getMsg().c_str(), logEvent->getCode()));
		if(_msgList.size()>MAXIMUM_ITEMS)
		{
			_msgList.pop_front();
		}
		_msgListMutex.unlock();

		if(_time.restart() < OLD_TIME)
		{
			if(logEvent->getCode() == ULogger::kFatal)
			{
				QMetaObject::invokeMethod(&_timer, "start", Q_ARG(int, 0));
			}
			else
			{
				QMetaObject::invokeMethod(&_timer, "start", Q_ARG(int, OLD_TIME));
			}
		}
		else
		{
			QMetaObject::invokeMethod(&_timer, "start", Q_ARG(int, 0));
		}

		if(logEvent->getCode() == ULogger::kFatal)
		{
			//This thread will wait until the message box is closed...
			// Assuming that error messages come from a different thread.
			_errorMessageMutex.lock();
		}

	}
}

void ConsoleWidget::appendMsg(const QString & msg, int level)
{
	switch(level)
	{
	case 0:
		_ui->textEdit->setTextColor(Qt::darkGreen);
		break;
	case 2:
		_ui->textEdit->setTextColor(Qt::yellow);
		break;
	case 3:
	case 4:
		_ui->textEdit->setTextColor(Qt::red);
		break;
	default:
		_ui->textEdit->setTextColor(Qt::white);
		break;
	}
	_ui->textEdit->append(msg);

	if(level == ULogger::kFatal)
	{
		_textCursor->endEditBlock();
		QTextCursor cursor = _ui->textEdit->textCursor();
		cursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
		_ui->textEdit->setTextCursor(cursor);
		//The application will exit, so warn the user.
		_errorMessage->setText(tr("Description:\n\n%1\n\nThe application will now exit...").arg(msg));
		_errorMessage->exec();
		_errorMessageMutex.unlock();
	}
}

void ConsoleWidget::flushConsole()
{
	_msgListMutex.lock();
	_textCursor->beginEditBlock();
	for(int i=0; i<_msgList.size(); ++i)
	{
		appendMsg(_msgList[i].first, _msgList[i].second);
	}
	_textCursor->endEditBlock();
	_msgList.clear();
	_msgListMutex.unlock();
	QTextCursor cursor = _ui->textEdit->textCursor();
	cursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
	_ui->textEdit->setTextCursor(cursor);
}

}
