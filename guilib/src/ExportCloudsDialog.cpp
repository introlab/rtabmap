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

#include <QPushButton>

namespace rtabmap {

ExportCloudsDialog::ExportCloudsDialog(QWidget *parent) :
	QDialog(parent)
{
	_ui = new Ui_ExportCloudsDialog();
	_ui->setupUi(this);

	connect(_ui->buttonBox->button(QDialogButtonBox::RestoreDefaults), SIGNAL(clicked()), this, SLOT(restoreDefaults()));

	restoreDefaults();
	_ui->comboBox_upsamplingMethod->setItemData(1, 0, Qt::UserRole - 1); // disable DISTINCT_CLOUD

	connect(_ui->checkBox_binary, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_normalKSearch, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->groupBox_regenerate, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_decimation, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_voxelSize, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_maxDepth, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

	connect(_ui->groupBox_filtering, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_filteringRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_filteringMinNeighbors, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->groupBox_assemble, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_voxelSize_assembled, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

	connect(_ui->groupBox_mls, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_mlsRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_polygonialOrder, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_upsamplingMethod, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_sampleStep, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_randomPoints, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_dilationVoxelSize, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_dilationSteps, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	_ui->stackedWidget_upsampling->setCurrentIndex(_ui->comboBox_upsamplingMethod->currentIndex());
	connect(_ui->comboBox_upsamplingMethod, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_upsampling, SLOT(setCurrentIndex(int)));
	connect(_ui->comboBox_upsamplingMethod, SIGNAL(currentIndexChanged(int)), this, SLOT(updateMLSGrpVisibility()));
	updateMLSGrpVisibility();

	connect(_ui->groupBox_gp3, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gp3Radius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gp3Mu, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_meshDecimationFactor, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_textureMapping, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
}

ExportCloudsDialog::~ExportCloudsDialog()
{
	delete _ui;
}

void ExportCloudsDialog::updateMLSGrpVisibility()
{
	_ui->groupBox->setVisible(_ui->comboBox_upsamplingMethod->currentIndex() == 0);
	_ui->groupBox_2->setVisible(_ui->comboBox_upsamplingMethod->currentIndex() == 1);
	_ui->groupBox_3->setVisible(_ui->comboBox_upsamplingMethod->currentIndex() == 2);
	_ui->groupBox_4->setVisible(_ui->comboBox_upsamplingMethod->currentIndex() == 3);
	_ui->groupBox_5->setVisible(_ui->comboBox_upsamplingMethod->currentIndex() == 4);
}

void ExportCloudsDialog::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	settings.setValue("binary", this->getBinaryFile());
	settings.setValue("normals_k", this->getNormalKSearch());

	settings.setValue("regenerate", this->getGenerate());
	settings.setValue("regenerate_decimation", this->getGenerateDecimation());
	settings.setValue("regenerate_voxel", this->getGenerateVoxel());
	settings.setValue("regenerate_max_depth", this->getGenerateMaxDepth());

	settings.setValue("filtering", this->getFiltering());
	settings.setValue("filtering_radius", this->getFilteringRadius());
	settings.setValue("filtering_min_neighbors", this->getFilteringMinNeighbors());

	settings.setValue("assemble", this->getAssemble());
	settings.setValue("assemble_voxel", this->getAssembleVoxel());

	settings.setValue("mls", this->getMLS());
	settings.setValue("mls_radius", this->getMLSRadius());
	settings.setValue("mls_polygonial_order", this->getMLSPolygonialOrder());
	settings.setValue("mls_upsampling_method", this->getMLSUpsamplingMethod());
	settings.setValue("mls_upsampling_radius", this->getMLSUpsamplingRadius());
	settings.setValue("mls_upsampling_step", this->getMLSUpsamplingStep());
	settings.setValue("mls_point_density", this->getMLSPointDensity());
	settings.setValue("mls_dilation_voxel_size", this->getMLSDilationVoxelSize());
	settings.setValue("mls_dilation_iterations", this->getMLSDilationIterations());

	settings.setValue("mesh", this->getMesh());
	settings.setValue("mesh_radius", this->getMeshGp3Radius());
	settings.setValue("mesh_mu", this->getMeshGp3Mu());
	settings.setValue("mesh_decimation_factor", this->getMeshDecimationFactor());

	settings.setValue("mesh_texture", this->getMeshTexture());

	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportCloudsDialog::loadSettings(QSettings & settings, const QString & group)
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}

	_ui->checkBox_binary->setChecked(settings.value("binary", this->getBinaryFile()).toBool());
	_ui->spinBox_normalKSearch->setValue(settings.value("normals_k", this->getNormalKSearch()).toInt());

	_ui->groupBox_regenerate->setChecked(settings.value("regenerate", this->getGenerate()).toBool());
	_ui->spinBox_decimation->setValue(settings.value("regenerate_decimation", this->getGenerateDecimation()).toInt());
	_ui->doubleSpinBox_voxelSize->setValue(settings.value("regenerate_voxel", this->getGenerateVoxel()).toDouble());
	_ui->doubleSpinBox_maxDepth->setValue(settings.value("regenerate_max_depth", this->getGenerateMaxDepth()).toDouble());

	_ui->groupBox_filtering->setChecked(settings.value("filtering", this->getFiltering()).toBool());
	_ui->doubleSpinBox_filteringRadius->setValue(settings.value("filtering_radius", this->getFilteringRadius()).toDouble());
	_ui->spinBox_filteringMinNeighbors->setValue(settings.value("filtering_min_neighbors", this->getFilteringMinNeighbors()).toInt());

	_ui->groupBox_assemble->setChecked(settings.value("assemble", this->getAssemble()).toBool());
	_ui->doubleSpinBox_voxelSize_assembled->setValue(settings.value("assemble_voxel", this->getAssembleVoxel()).toDouble());

	_ui->groupBox_mls->setChecked(settings.value("mls", this->getMLS()).toBool());
	_ui->doubleSpinBox_mlsRadius->setValue(settings.value("mls_radius", this->getMLSRadius()).toDouble());
	_ui->spinBox_polygonialOrder->setValue(settings.value("mls_polygonial_order", this->getMLSPolygonialOrder()).toInt());
	_ui->comboBox_upsamplingMethod->setCurrentIndex(settings.value("mls_upsampling_method", this->getMLSUpsamplingMethod()).toInt());
	_ui->doubleSpinBox_sampleRadius->setValue(settings.value("mls_upsampling_radius", this->getMLSRadius()).toDouble());
	_ui->doubleSpinBox_sampleStep->setValue(settings.value("mls_upsampling_step", this->getMLSUpsamplingStep()).toDouble());
	_ui->spinBox_randomPoints->setValue(settings.value("mls_point_density", this->getMLSPointDensity()).toInt());
	_ui->doubleSpinBox_dilationVoxelSize->setValue(settings.value("mls_dilation_voxel_size", this->getMLSDilationVoxelSize()).toDouble());
	_ui->spinBox_dilationSteps->setValue(settings.value("mls_dilation_iterations", this->getMLSDilationIterations()).toInt());

	_ui->groupBox_gp3->setChecked(settings.value("mesh", this->getMesh()).toBool());
	_ui->doubleSpinBox_gp3Radius->setValue(settings.value("mesh_radius", this->getMeshGp3Radius()).toDouble());
	_ui->doubleSpinBox_gp3Mu->setValue(settings.value("mesh_mu", this->getMeshGp3Mu()).toDouble());
	_ui->doubleSpinBox_meshDecimationFactor->setValue(settings.value("mesh_decimation_factor", this->getMeshDecimationFactor()).toDouble());

	_ui->checkBox_textureMapping->setChecked(settings.value("mesh_texture", this->getGenerate()).toBool());

	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportCloudsDialog::restoreDefaults()
{
	_ui->checkBox_binary->setChecked(true);
	_ui->spinBox_normalKSearch->setValue(20);

	if(_ui->groupBox_regenerate->isEnabled())
	{
		_ui->groupBox_regenerate->setChecked(true);
	}
	_ui->spinBox_decimation->setValue(1);
	_ui->doubleSpinBox_voxelSize->setValue(0.01);
	_ui->doubleSpinBox_maxDepth->setValue(4);

	_ui->groupBox_filtering->setChecked(false);
	_ui->doubleSpinBox_filteringRadius->setValue(0.02);
	_ui->spinBox_filteringMinNeighbors->setValue(2);

	_ui->groupBox_assemble->setChecked(true);
	_ui->doubleSpinBox_voxelSize_assembled->setValue(0.01);

	_ui->groupBox_mls->setChecked(false);
	_ui->doubleSpinBox_mlsRadius->setValue(0.04);
	_ui->spinBox_polygonialOrder->setValue(2);
	_ui->comboBox_upsamplingMethod->setCurrentIndex(0);
	_ui->doubleSpinBox_sampleRadius->setValue(0.01);
	_ui->doubleSpinBox_sampleStep->setValue(0.0);
	_ui->spinBox_randomPoints->setValue(0);
	_ui->doubleSpinBox_dilationVoxelSize->setValue(0.01);
	_ui->spinBox_dilationSteps->setValue(0);

	_ui->groupBox_gp3->setChecked(false);
	_ui->doubleSpinBox_gp3Radius->setValue(0.04);
	_ui->doubleSpinBox_gp3Mu->setValue(2.5);
	_ui->doubleSpinBox_meshDecimationFactor->setValue(0.0);
	_ui->checkBox_textureMapping->setChecked(false);

	this->update();
}

void ExportCloudsDialog::setSaveButton()
{
	_ui->buttonBox->button(QDialogButtonBox::Ok)->setVisible(false);
	_ui->buttonBox->button(QDialogButtonBox::Save)->setVisible(true);
	_ui->checkBox_binary->setVisible(true);
	_ui->label_binaryFile->setVisible(true);
	_ui->checkBox_textureMapping->setVisible(true);
	_ui->label_textureMapping->setVisible(true);
}

void ExportCloudsDialog::setOkButton()
{
	_ui->buttonBox->button(QDialogButtonBox::Ok)->setVisible(true);
	_ui->buttonBox->button(QDialogButtonBox::Save)->setVisible(false);
	_ui->checkBox_binary->setVisible(false);
	_ui->label_binaryFile->setVisible(false);
	_ui->checkBox_textureMapping->setVisible(false);
	_ui->label_textureMapping->setVisible(false);
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
bool ExportCloudsDialog::getBinaryFile() const
{
	return _ui->checkBox_binary->isChecked();
}
int ExportCloudsDialog::getNormalKSearch() const
{
	return _ui->spinBox_normalKSearch->value();
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

bool ExportCloudsDialog::getFiltering() const
{
	return _ui->groupBox_filtering->isChecked();
}
double ExportCloudsDialog::getFilteringRadius() const
{
	return _ui->doubleSpinBox_filteringRadius->value();
}
int ExportCloudsDialog::getFilteringMinNeighbors() const
{
	return _ui->spinBox_filteringMinNeighbors->value();
}

bool ExportCloudsDialog::getAssemble() const
{
	return _ui->groupBox_assemble->isChecked();
}
double ExportCloudsDialog::getAssembleVoxel() const
{
	return _ui->doubleSpinBox_voxelSize_assembled->value();
}

bool ExportCloudsDialog::getMLS() const
{
	return _ui->groupBox_mls->isChecked();
}
double ExportCloudsDialog::getMLSRadius() const
{
	return _ui->doubleSpinBox_mlsRadius->value();
}
int ExportCloudsDialog::getMLSPolygonialOrder() const
{
	return _ui->spinBox_polygonialOrder->value();
}
int ExportCloudsDialog::getMLSUpsamplingMethod() const
{
	return _ui->comboBox_upsamplingMethod->currentIndex();
}
double ExportCloudsDialog::getMLSUpsamplingRadius() const
{
	return _ui->doubleSpinBox_sampleRadius->value();
}
double ExportCloudsDialog::getMLSUpsamplingStep() const
{
	return _ui->doubleSpinBox_sampleStep->value();
}
int ExportCloudsDialog::getMLSPointDensity() const
{
	return _ui->spinBox_randomPoints->value();
}
double ExportCloudsDialog::getMLSDilationVoxelSize() const
{
	return _ui->doubleSpinBox_dilationVoxelSize->value();
}
int ExportCloudsDialog::getMLSDilationIterations() const
{
	return _ui->spinBox_dilationSteps->value();
}

bool ExportCloudsDialog::getMesh() const
{
	return _ui->groupBox_gp3->isChecked();
}
double ExportCloudsDialog::getMeshGp3Radius() const
{
	return _ui->doubleSpinBox_gp3Radius->value();
}
double ExportCloudsDialog::getMeshGp3Mu() const
{
	return _ui->doubleSpinBox_gp3Mu->value();
}
double ExportCloudsDialog::getMeshDecimationFactor() const
{
	return _ui->doubleSpinBox_meshDecimationFactor->value();
}
bool ExportCloudsDialog::getMeshTexture() const
{
	return _ui->checkBox_textureMapping->isChecked();
}

}
