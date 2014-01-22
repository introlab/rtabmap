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

#ifndef EXPORTDIALOG_H_
#define EXPORTDIALOG_H_

#include <QtGui/QDialog>

class Ui_ExportDialog;

namespace rtabmap {

class ExportDialog : public QDialog
{
	Q_OBJECT

public:
	ExportDialog(QWidget * parent = 0);

	virtual ~ExportDialog();

	QString outputPath() const;
	int framesIgnored() const;
	bool isRgbExported() const;
	bool isDepthExported() const;
	bool isDepth2dExported() const;
	bool isOdomExported() const;

private slots:
	void getPath();

private:
	Ui_ExportDialog * _ui;
};

}

#endif /* ABOUTDIALOG_H_ */
