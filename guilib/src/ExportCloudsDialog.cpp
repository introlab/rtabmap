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

#include <QtGui/QPushButton>

namespace rtabmap {

ExportCloudsDialog::ExportCloudsDialog(QWidget *parent) :
	QDialog(parent)
{
	_ui = new Ui_ExportCloudsDialog();
	_ui->setupUi(this);

	connect(_ui->buttonBox->button(QDialogButtonBox::RestoreDefaults), SIGNAL(clicked()), this, SLOT(restoreDefaults()));

	connect(_ui->groupBox_assemble, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_voxelSize_assembled, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->groupBox_regenerate, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_decimation, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_voxelSize, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_maxDepth, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_binary, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->groupBox_mls, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_mlsRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->groupBox_gp3, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_normalKSearch, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gp3Radius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
}

ExportCloudsDialog::~ExportCloudsDialog()
{
	delete _ui;
}

void ExportCloudsDialog::restoreDefaults()
{
	setAssemble(true);
	setAssembleVoxel(0.005);
	if(_ui->groupBox_regenerate->isEnabled())
	{
		setGenerate(true);
	}
	setGenerateDecimation(1);
	setGenerateVoxel(0.005);
	setGenerateMaxDepth(4);
	setBinaryFile(true);
	setMLS(false);
	setMLSRadius(0.04);
	setMesh(false);
	setMeshNormalKSearch(20);
	setMeshGp3Radius(0.04);
}

void ExportCloudsDialog::setSaveButton()
{
	_ui->buttonBox->button(QDialogButtonBox::Ok)->setVisible(false);
	_ui->buttonBox->button(QDialogButtonBox::Save)->setVisible(true);
	_ui->checkBox_binary->setVisible(true);
}

void ExportCloudsDialog::setOkButton()
{
	_ui->buttonBox->button(QDialogButtonBox::Ok)->setVisible(true);
	_ui->buttonBox->button(QDialogButtonBox::Save)->setVisible(false);
	_ui->checkBox_binary->setVisible(false);
}

void ExportCloudsDialog::enableRegeneration(bool enabled)
{
	if(!enabled)
	{
		_ui->groupBox_regenerate->setChecked(false);
	}
	_ui->groupBox_regenerate->setEnabled(enabled);
}

//getters
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

//setters
void ExportCloudsDialog::setAssemble(bool on)
{
	_ui->groupBox_assemble->setChecked(on);
}
void ExportCloudsDialog::setAssembleVoxel(double voxel)
{
	_ui->doubleSpinBox_voxelSize_assembled->setValue(voxel);
}
void ExportCloudsDialog::setGenerate(bool on)
{
	_ui->groupBox_regenerate->setChecked(on);
}
void ExportCloudsDialog::setGenerateDecimation(int decimation)
{
	_ui->spinBox_decimation->setValue(decimation);
}
void ExportCloudsDialog::setGenerateVoxel(double voxel)
{
	_ui->doubleSpinBox_voxelSize->setValue(voxel);
}
void ExportCloudsDialog::setGenerateMaxDepth(double maxDepth)
{
	_ui->doubleSpinBox_maxDepth->setValue(maxDepth);
}
void ExportCloudsDialog::setBinaryFile(bool on)
{
	_ui->checkBox_binary->setChecked(on);
}
void ExportCloudsDialog::setMLS(bool on)
{
	_ui->groupBox_mls->setChecked(on);
}
void ExportCloudsDialog::setMLSRadius(double radius)
{
	_ui->doubleSpinBox_mlsRadius->setValue(radius);
}
void ExportCloudsDialog::setMesh(bool on)
{
	_ui->groupBox_gp3->setChecked(on);
}
void ExportCloudsDialog::setMeshNormalKSearch(int k)
{
	_ui->spinBox_normalKSearch->setValue(k);
}
void ExportCloudsDialog::setMeshGp3Radius(double radius)
{
	_ui->doubleSpinBox_gp3Radius->setValue(radius);
}


}
