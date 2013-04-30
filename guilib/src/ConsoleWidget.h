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

#include <rtabmap/utilite/UEventsHandler.h>
#include <QtGui/QWidget>
#include <QtCore/QMutex>
#include <QtCore/QTimer>
#include <QtCore/QTime>

class Ui_consoleWidget;
class QMessageBox;
class QTextCursor;

namespace rtabmap {

class ConsoleWidget : public QWidget, public UEventsHandler
{
	Q_OBJECT;

public:
	ConsoleWidget(QWidget * parent = 0);
	virtual ~ConsoleWidget();

public slots:
	void appendMsg(const QString & msg, int level = 1);

signals:
	void msgReceived(const QString &, int);

private slots:
	void flushConsole();

protected:
	virtual void handleEvent(UEvent * anEvent);

private:
	Ui_consoleWidget * _ui;
	QMessageBox * _errorMessage;
	QMutex _errorMessageMutex;
	QMutex _msgListMutex;
	QTimer _timer;
	QTime _time;
	QTextCursor * _textCursor;
	QList<QPair<QString, int> > _msgList;
};

}

#endif /* CONSOLEWIDGET_H_ */
