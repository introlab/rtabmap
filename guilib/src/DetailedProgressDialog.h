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

#ifndef DETAILEDPROGRESSDIALOG_H_
#define DETAILEDPROGRESSDIALOG_H_

#include <QtGui/QDialog>

class QLabel;
class QTextEdit;
class QProgressBar;
class QPushButton;
class QCheckBox;

namespace rtabmap {

class DetailedProgressDialog : public QDialog
{
	Q_OBJECT

public:
	DetailedProgressDialog(QWidget *parent = 0, Qt::WindowFlags flags = 0);
	virtual ~DetailedProgressDialog();

	void setEndMessage(const QString & message) {_endMessage = message;} // Message shown when the progress is finished
	void setValue(int value);
	int maximumSteps() const;
	void setMaximumSteps(int steps);
	void setAutoClose(bool on, int delayedClosingTimeMsec = 0);

protected:
	virtual void closeEvent(QCloseEvent * event);

public slots:
	void appendText(const QString & text);
	void incrementStep();
	void clear();
	void resetProgress();

private:
	QLabel * _text;
	QTextEdit * _detailedText;
	QProgressBar * _progressBar;
	QPushButton * _closeButton;
	QCheckBox * _closeWhenDoneCheckBox;
	QString _endMessage;
	bool _autoClose;
	int _delayedClosingTime; // sec
};

}

#endif /* DETAILEDPROGRESSDIALOG_H_ */
