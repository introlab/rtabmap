/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "ExportCloudsDialog.h"
#include "ui_exportCloudsDialog.h"

#include <QtGui/QFileDialog>

namespace rtabmap {

ExportCloudsDialog::ExportCloudsDialog(QWidget *parent) :
	QDialog(parent)
{
	_ui = new Ui_ExportCloudsDialog();
	_ui->setupUi(this);
}

ExportCloudsDialog::~ExportCloudsDialog()
{
	delete _ui;
}

void ExportCloudsDialog::setSaveButton()
{
	_ui->buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Save);
	_ui->checkBox_binary->setVisible(true);
}

void ExportCloudsDialog::setOkButton()
{
	_ui->buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
	_ui->checkBox_binary->setVisible(false);
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

bool ExportCloudsDialog::getBinaryFile() const
{
	return _ui->checkBox_binary->isChecked();
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
