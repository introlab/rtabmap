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

#include "ExportCloudsDialog.h"
#include "ui_exportCloudsDialog.h"

#include <QtGui/QFileDialog>

namespace rtabmap {

ExportCloudsDialog::ExportCloudsDialog(QWidget *parent, bool toSave) :
	QDialog(parent)
{
	_ui = new Ui_ExportCloudsDialog();
	_ui->setupUi(this);
	if(toSave)
	{
		_ui->buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Save);
	}
}

ExportCloudsDialog::~ExportCloudsDialog()
{
	delete _ui;
}

bool ExportCloudsDialog::getAssemble() const
{
	return _ui->groupBox_assemble->isChecked();
}

double ExportCloudsDialog::getAssembleVoxel() const
{
	return _ui->doubleSpinBox_voxelSize_assembled->value();
}

bool ExportCloudsDialog::getGenerate() const
{
	return _ui->groupBox_regenerate->isChecked();
}

int ExportCloudsDialog::getGenerateDecimation() const
{
	return _ui->spinBox_decimation->value();
}

double ExportCloudsDialog::getGenerateVoxel() const
{
	return _ui->doubleSpinBox_voxelSize->value();
}

double ExportCloudsDialog::getGenerateMaxDepth() const
{
	return _ui->doubleSpinBox_maxDepth->value();
}

bool ExportCloudsDialog::getMLS() const
{
	return _ui->groupBox_mls->isChecked();
}

double ExportCloudsDialog::getMLSRadius() const
{
	return _ui->doubleSpinBox_mlsRadius->value();
}

bool ExportCloudsDialog::getMesh() const
{
	return _ui->groupBox_gp3->isChecked();
}

int ExportCloudsDialog::getMeshNormalKSearch() const
{
	return _ui->spinBox_normalKSearch->value();
}

double ExportCloudsDialog::getMeshGp3Radius() const
{
	return _ui->doubleSpinBox_gp3Radius->value();
}


}
