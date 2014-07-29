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

#ifndef EXPORTCLOUDSDIALOG_H_
#define EXPORTCLOUDSDIALOG_H_

#include <QtGui/QDialog>

class Ui_ExportCloudsDialog;

namespace rtabmap {

class ExportCloudsDialog : public QDialog
{
	Q_OBJECT

public:
	ExportCloudsDialog(QWidget *parent = 0, bool toSave = false);

	virtual ~ExportCloudsDialog();

	bool getAssemble() const;
	double getAssembleVoxel() const;
	bool getGenerate() const;
	int getGenerateDecimation() const;
	double getGenerateVoxel() const;
	double getGenerateMaxDepth() const;
	bool getMLS() const;
	double getMLSRadius() const;
	bool getMesh() const;
	int getMeshNormalKSearch() const;
	double getMeshGp3Radius() const;

private:
	Ui_ExportCloudsDialog * _ui;
};

}

#endif /* EXPORTCLOUDSDIALOG_H_ */
