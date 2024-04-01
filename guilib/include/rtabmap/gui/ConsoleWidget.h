/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
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

#ifndef RTABMAP_CONSOLEWIDGET_H_
#define RTABMAP_CONSOLEWIDGET_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <rtabmap/utilite/UEventsHandler.h>
#include <QWidget>
#include <QtCore/QMutex>
#include <QtCore/QTimer>
#include <QtCore/QTime>
#include <QtCore/QElapsedTimer>

class Ui_consoleWidget;
class QMessageBox;
class QTextCursor;

namespace rtabmap {

class RTABMAP_GUI_EXPORT ConsoleWidget : public QWidget, public UEventsHandler
{
	Q_OBJECT;

public:
	ConsoleWidget(QWidget * parent = 0);
	virtual ~ConsoleWidget();

public Q_SLOTS:
	void appendMsg(const QString & msg, int level = 1);

Q_SIGNALS:
	void msgReceived(const QString &, int);

private Q_SLOTS:
	void flushConsole();
	void updateTextEditBufferSize();

protected:
	virtual bool handleEvent(UEvent * anEvent);

private:
	Ui_consoleWidget * _ui;
	QMessageBox * _errorMessage;
	QMutex _errorMessageMutex;
	QMutex _msgListMutex;
	QTimer _timer;
	QElapsedTimer _time;
	QTextCursor * _textCursor;
	QList<QPair<QString, int> > _msgList;
};

}

#endif /* CONSOLEWIDGET_H_ */
