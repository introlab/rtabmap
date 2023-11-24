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

#ifndef RTABMAP_PROGRESSDIALOG_H_
#define RTABMAP_PROGRESSDIALOG_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <QDialog>

class QLabel;
class QTextEdit;
class QProgressBar;
class QPushButton;
class QCheckBox;

namespace rtabmap {

class RTABMAP_GUI_EXPORT ProgressDialog : public QDialog
{
	Q_OBJECT

public:
	ProgressDialog(QWidget *parent = 0, Qt::WindowFlags flags = Qt::WindowFlags());
	virtual ~ProgressDialog();

	void setEndMessage(const QString & message) {_endMessage = message;} // Message shown when the progress is finished
	void setValue(int value);
	int maximumSteps() const;
	void setMaximumSteps(int steps);
	void setAutoClose(bool on, int delayedClosingTimeSec = -1);
	void setCancelButtonVisible(bool visible);
	bool isCanceled() const {return _canceled;}

Q_SIGNALS:
	void canceled();

protected:
	virtual void closeEvent(QCloseEvent * event);

public Q_SLOTS:
	void appendText(const QString & text ,const QColor & color = Qt::black);
	void incrementStep(int steps = 1);
	void clear();
	void resetProgress();

private Q_SLOTS:
	void closeDialog();
	void cancel();

private:
	QLabel * _text;
	QTextEdit * _detailedText;
	QProgressBar * _progressBar;
	QPushButton * _closeButton;
	QPushButton * _cancelButton;
	QCheckBox * _closeWhenDoneCheckBox;
	QString _endMessage;
	int _delayedClosingTime; // sec
	bool _canceled;
};

}

#endif /* PROGRESSDIALOG_H_ */
