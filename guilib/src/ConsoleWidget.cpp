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

#include "rtabmap/gui/ConsoleWidget.h"
#include "ui_consoleWidget.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <QMessageBox>
#include <QtGui/QTextCursor>
#include <QtCore/QTimer>

namespace rtabmap {

ConsoleWidget::ConsoleWidget(QWidget * parent) :
	QWidget(parent)
{
	_ui = new Ui_consoleWidget();
	_ui->setupUi(this);
	UEventsManager::addHandler(this);
	_ui->textEdit->document()->setMaximumBlockCount(_ui->spinBox_lines->value());
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
	connect(_ui->spinBox_lines, SIGNAL(valueChanged(int)), this, SLOT(updateTextEditBufferSize()));
	connect(this, SIGNAL(msgReceived(const QString &, int)), this, SLOT(appendMsg(const QString &, int)));
	connect(&_timer, SIGNAL(timeout()), this, SLOT(flushConsole()));
}

ConsoleWidget::~ConsoleWidget()
{
	delete _ui;
	_errorMessageMutex.unlock();
}

bool ConsoleWidget::handleEvent(UEvent * anEvent)
{
	// WARNING, don't put a log message here! otherwise it could be recursively called.
	if(anEvent->getClassName().compare("ULogEvent") == 0)
	{
		ULogEvent * logEvent = (ULogEvent*)anEvent;
		_msgListMutex.lock();
		_msgList.append(QPair<QString, int>(logEvent->getMsg().c_str(), logEvent->getCode()));
		while(_ui->spinBox_lines->value()>0 && _msgList.size()>_ui->spinBox_lines->value())
		{
			_msgList.pop_front();
		}
		_msgListMutex.unlock();

		if(_ui->spinBox_time->value()>0 && _time.restart() < _ui->spinBox_time->value())
		{
			if(logEvent->getCode() == ULogger::kFatal)
			{
				QMetaObject::invokeMethod(&_timer, "start", Q_ARG(int, 0));
			}
			else
			{
				QMetaObject::invokeMethod(&_timer, "start", Q_ARG(int, _ui->spinBox_time->value()));
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
	return false;
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

void ConsoleWidget::updateTextEditBufferSize()
{
	_ui->textEdit->document()->setMaximumBlockCount(_ui->spinBox_lines->value());
}

}
