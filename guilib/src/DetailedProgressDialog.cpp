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

#include "DetailedProgressDialog.h"
#include <QtGui/QLayout>
#include <QtGui/QProgressBar>
#include <QtGui/QTextEdit>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QCloseEvent>
#include <QtGui/QCheckBox>
#include <QtCore/QTimer>
#include <QtCore/QTime>
#include "rtabmap/utilite/ULogger.h"

namespace rtabmap {

DetailedProgressDialog::DetailedProgressDialog(QWidget *parent, Qt::WindowFlags flags) :
		QDialog(parent, flags),
		_autoClose(false),
		_delayedClosingTime(0)
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
	_closeWhenDoneCheckBox = new QCheckBox(this);
	_closeWhenDoneCheckBox->setChecked(false);
	_closeWhenDoneCheckBox->setText("Close when done.");
	_endMessage = "Finished!";
	this->clear();
	connect(_closeButton, SIGNAL(clicked()), this, SLOT(close()));

	QVBoxLayout * layout = new QVBoxLayout(this);
	layout->addWidget(_text);
	layout->addWidget(_progressBar);
	layout->addWidget(_detailedText);
	QHBoxLayout * hLayout = new QHBoxLayout();
	layout->addLayout(hLayout);
	hLayout->addWidget(_closeWhenDoneCheckBox);
	hLayout->addWidget(_closeButton);
	this->setLayout(layout);
}

DetailedProgressDialog::~DetailedProgressDialog()
{

}

void DetailedProgressDialog::setAutoClose(bool on, int delayedClosingTimeSec)
{
	_delayedClosingTime = delayedClosingTimeSec;
	_closeWhenDoneCheckBox->setChecked(on);
}

void DetailedProgressDialog::appendText(const QString & text)
{
	_text->setText(text);
	QString html = tr("<html><font color=\"#999999\">%1 </font>%2</html>").arg(QTime::currentTime().toString("HH:mm:ss")).arg(text);
	_detailedText->append(html);
	_detailedText->ensureCursorVisible();
}
void DetailedProgressDialog::setValue(int value)
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
			QTimer::singleShot(_delayedClosingTime*1000, this, SLOT(close()));
		}
	}
}
int DetailedProgressDialog::maximumSteps() const
{
	return _progressBar->maximum();
}
void DetailedProgressDialog::setMaximumSteps(int steps)
{
	_progressBar->setMaximum(steps);
}

void DetailedProgressDialog::incrementStep()
{
	//incremental progress bar (if we don't know how many items will be added)
	if(_progressBar->value() == _progressBar->maximum()-1)
	{
		_progressBar->setMaximum(_progressBar->maximum()+1);
	}
	_progressBar->setValue(_progressBar->value()+1);
}

void DetailedProgressDialog::clear()
{
	_text->clear();
	_progressBar->reset();
	_detailedText->clear();
	_closeButton->setEnabled(false);
}

void DetailedProgressDialog::resetProgress()
{
	_progressBar->reset();
	_closeButton->setEnabled(false);
}

void DetailedProgressDialog::closeEvent(QCloseEvent *event)
{
	if(_progressBar->value() == _progressBar->maximum())
	{
		event->accept();
	}
	else
	{
		event->ignore();
	}
}

}
