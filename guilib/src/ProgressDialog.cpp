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

#include "rtabmap/gui/ProgressDialog.h"
#include <QLayout>
#include <QProgressBar>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>
#include <QtGui/QCloseEvent>
#include <QCheckBox>
#include <QtCore/QTimer>
#include <QtCore/QTime>
#include <QScrollBar>
#include "rtabmap/utilite/ULogger.h"

namespace rtabmap {

ProgressDialog::ProgressDialog(QWidget *parent, Qt::WindowFlags flags) :
		QDialog(parent, flags),
		_delayedClosingTime(1),
		_canceled(false)
{
	_text = new QLabel(this);
	_text->setWordWrap(true);
	_progressBar = new QProgressBar(this);
	_progressBar->setMaximum(1);
	_detailedText  = new QTextEdit(this);
	_detailedText->setReadOnly(true);
	_detailedText->setLineWrapMode(QTextEdit::NoWrap);
	_closeButton = new QPushButton(this);
	_closeButton->setText("Close");
	_cancelButton = new QPushButton(this);
	_cancelButton->setText("Cancel");
	_cancelButton-> setVisible(false);
	_closeWhenDoneCheckBox = new QCheckBox(this);
	_closeWhenDoneCheckBox->setChecked(true);
	_closeWhenDoneCheckBox->setText("Close when done.");
	_endMessage = "Finished!";
	this->clear();
	connect(_closeButton, SIGNAL(clicked()), this, SLOT(close()));
	connect(_cancelButton, SIGNAL(clicked()), this, SLOT(cancel()));

	QVBoxLayout * layout = new QVBoxLayout(this);
	layout->addWidget(_text);
	layout->addWidget(_progressBar);
	layout->addWidget(_detailedText);
	QHBoxLayout * hLayout = new QHBoxLayout();
	layout->addLayout(hLayout);
	hLayout->addWidget(_closeWhenDoneCheckBox);
	hLayout->addStretch();
	hLayout->addWidget(_cancelButton);
	hLayout->addWidget(_closeButton);
	this->setLayout(layout);

	this->setModal(true);
}

ProgressDialog::~ProgressDialog()
{

}

void ProgressDialog::setAutoClose(bool on, int delayedClosingTimeSec)
{
	if(delayedClosingTimeSec >= 0)
	{
		_delayedClosingTime = delayedClosingTimeSec;
	}
	_closeWhenDoneCheckBox->setChecked(on);
}

void ProgressDialog::setCancelButtonVisible(bool visible)
{
	_cancelButton->setVisible(visible);
}

void ProgressDialog::appendText(const QString & text, const QColor & color)
{
	UDEBUG(text.toStdString().c_str());
	_text->setText(text);
	QString html = tr("<html><font color=\"#999999\">%1 </font><font color=\"%2\">%3</font></html>").arg(QTime::currentTime().toString("HH:mm:ss")).arg(color.name()).arg(text);
	_detailedText->append(html);
	_detailedText->ensureCursorVisible();
	_detailedText->horizontalScrollBar()->setSliderPosition(0);
	_detailedText->verticalScrollBar()->setSliderPosition(_detailedText->verticalScrollBar()->maximum());
}
void ProgressDialog::setValue(int value)
{
	_progressBar->setValue(value);
	if(value == _progressBar->maximum())
	{
		_text->setText(_endMessage);
		_closeButton->setEnabled(true);
		if(_closeWhenDoneCheckBox->isChecked() && _delayedClosingTime == 0)
		{
			this->close();
		}
		else if(_closeWhenDoneCheckBox->isChecked())
		{
			QTimer::singleShot(_delayedClosingTime*1000, this, SLOT(closeDialog()));
		}
	}
}
int ProgressDialog::maximumSteps() const
{
	return _progressBar->maximum();
}
void ProgressDialog::setMaximumSteps(int steps)
{
	_progressBar->setMaximum(steps);
}

void ProgressDialog::incrementStep(int steps)
{
	//incremental progress bar (if we don't know how many items will be added)
	if(_progressBar->value() >= _progressBar->maximum()-steps)
	{
		_progressBar->setMaximum(_progressBar->maximum()+steps+1);
	}
	_progressBar->setValue(_progressBar->value()+steps);
}

void ProgressDialog::clear()
{
	_text->clear();
	_detailedText->clear();
	resetProgress();
}

void ProgressDialog::resetProgress()
{
	_progressBar->reset();
	_closeButton->setEnabled(false);
	_canceled = false;
}

void ProgressDialog::closeDialog()
{
	if(_closeWhenDoneCheckBox->isChecked())
	{
		close();
	}
}

void ProgressDialog::closeEvent(QCloseEvent *event)
{
	if(_progressBar->value() == _progressBar->maximum())
	{
		_canceled = false;
		event->accept();
	}
	else
	{
		event->ignore();
	}
}

void ProgressDialog::cancel()
{
	_canceled = true;
	Q_EMIT canceled();
}

}
