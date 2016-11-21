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

#include "ExportCloudsDialog.h"
#include "ui_exportCloudsDialog.h"

#include "rtabmap/gui/ProgressDialog.h"
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/UStl.h"

#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/GainCompensator.h"
#include "rtabmap/core/clams/discrete_depth_distortion_model.h"

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/pcl_config.h>

#include <QPushButton>
#include <QDir>
#include <QFileInfo>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QDesktopWidget>

namespace rtabmap {

ExportCloudsDialog::ExportCloudsDialog(QWidget *parent) :
	QDialog(parent),
	_canceled(false)
{
	_ui = new Ui_ExportCloudsDialog();
	_ui->setupUi(this);

	connect(_ui->buttonBox->button(QDialogButtonBox::RestoreDefaults), SIGNAL(clicked()), this, SLOT(restoreDefaults()));

	restoreDefaults();
	_ui->comboBox_upsamplingMethod->setItemData(1, 0, Qt::UserRole - 1); // disable DISTINCT_CLOUD

	connect(_ui->checkBox_binary, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_normalKSearch, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_pipeline, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_pipeline, SIGNAL(currentIndexChanged(int)), this, SLOT(updateReconstructionFlavor()));

	connect(_ui->groupBox_regenerate, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_decimation, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_maxDepth, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_minDepth, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->lineEdit_distortionModel, SIGNAL(textChanged(const QString &)), this, SIGNAL(configChanged()));
	connect(_ui->toolButton_distortionModel, SIGNAL(clicked()), this, SLOT(selectDistortionModel()));

	connect(_ui->groupBox_bilateral, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_bilateral_sigmaS, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_bilateral_sigmaR, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

	connect(_ui->groupBox_filtering, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_filteringRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_filteringMinNeighbors, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->checkBox_assemble, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_assemble, SIGNAL(clicked(bool)), this, SLOT(updateTexturingAvailability()));
	connect(_ui->doubleSpinBox_voxelSize_assembled, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

	connect(_ui->groupBox_subtraction, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_subtractPointFilteringRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_subtractPointFilteringAngle, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_subtractFilteringMinPts, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

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

	connect(_ui->groupBox_gain, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gainRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gainOverlap, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gainAlpha, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gainBeta, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_gainLinkedLocationsOnly, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->groupBox_meshing, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gp3Radius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gp3Mu, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_meshDecimationFactor, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_textureMapping, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_textureMapping, SIGNAL(stateChanged(int)), this, SLOT(updateTexturingAvailability()));

	_progressDialog = new ProgressDialog(this);
	_progressDialog->setVisible(false);
	_progressDialog->setAutoClose(true, 2);
	_progressDialog->setMinimumWidth(600);
	_progressDialog->setCancelButtonVisible(true);
	connect(_progressDialog, SIGNAL(canceled()), this, SLOT(cancel()));

#ifdef DISABLE_VTK
	_ui->doubleSpinBox_meshDecimationFactor->setEnabled(false);
#endif
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
void ExportCloudsDialog::updateTexturingAvailability()
{
	updateTexturingAvailability(_ui->checkBox_binary->isEnabled());
}
void ExportCloudsDialog::updateTexturingAvailability(bool isExporting)
{
	_ui->checkBox_textureMapping->setEnabled(!_ui->checkBox_assemble->isChecked() || isExporting);
	_ui->label_textureMapping->setEnabled(_ui->checkBox_textureMapping->isEnabled());
	_ui->doubleSpinBox_meshDecimationFactor->setEnabled(!_ui->checkBox_textureMapping->isEnabled() || !_ui->checkBox_textureMapping->isChecked() || _ui->comboBox_pipeline->currentIndex() == 1);
	_ui->label_meshDecimation->setEnabled(_ui->doubleSpinBox_meshDecimationFactor->isEnabled());
}

void ExportCloudsDialog::cancel()
{
	_canceled = true;
	_progressDialog->appendText(tr("Canceled!"));
}

void ExportCloudsDialog::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	settings.setValue("pipeline", _ui->comboBox_pipeline->currentIndex());
	settings.setValue("binary", _ui->checkBox_binary->isChecked());
	settings.setValue("normals_k", _ui->spinBox_normalKSearch->value());

	settings.setValue("regenerate", _ui->groupBox_regenerate->isChecked());
	settings.setValue("regenerate_decimation", _ui->spinBox_decimation->value());
	settings.setValue("regenerate_max_depth", _ui->doubleSpinBox_maxDepth->value());
	settings.setValue("regenerate_min_depth", _ui->doubleSpinBox_minDepth->value());
	settings.setValue("regenerate_distortion_model", _ui->lineEdit_distortionModel->text());

	settings.setValue("bilateral", _ui->groupBox_bilateral->isChecked());
	settings.setValue("bilateral_sigma_s", _ui->doubleSpinBox_bilateral_sigmaS->value());
	settings.setValue("bilateral_sigma_r", _ui->doubleSpinBox_bilateral_sigmaR->value());

	settings.setValue("filtering", _ui->groupBox_filtering->isChecked());
	settings.setValue("filtering_radius", _ui->doubleSpinBox_filteringRadius->value());
	settings.setValue("filtering_min_neighbors", _ui->spinBox_filteringMinNeighbors->value());

	settings.setValue("assemble", _ui->checkBox_assemble->isChecked());
	settings.setValue("assemble_voxel",_ui->doubleSpinBox_voxelSize_assembled->value());

	settings.setValue("subtract",_ui->groupBox_subtraction->isChecked());
	settings.setValue("subtract_point_radius",_ui->doubleSpinBox_subtractPointFilteringRadius->value());
	settings.setValue("subtract_point_angle",_ui->doubleSpinBox_subtractPointFilteringAngle->value());
	settings.setValue("subtract_min_neighbors",_ui->spinBox_subtractFilteringMinPts->value());

	settings.setValue("mls", _ui->groupBox_mls->isChecked());
	settings.setValue("mls_radius", _ui->doubleSpinBox_mlsRadius->value());
	settings.setValue("mls_polygonial_order", _ui->spinBox_polygonialOrder->value());
	settings.setValue("mls_upsampling_method", _ui->comboBox_upsamplingMethod->currentIndex());
	settings.setValue("mls_upsampling_radius", _ui->doubleSpinBox_sampleRadius->value());
	settings.setValue("mls_upsampling_step", _ui->doubleSpinBox_sampleStep->value());
	settings.setValue("mls_point_density", _ui->spinBox_randomPoints->value());
	settings.setValue("mls_dilation_voxel_size", _ui->doubleSpinBox_dilationVoxelSize->value());
	settings.setValue("mls_dilation_iterations", _ui->spinBox_dilationSteps->value());

	settings.setValue("gain", _ui->groupBox_gain->isChecked());
	settings.setValue("gain_radius", _ui->doubleSpinBox_gainRadius->value());
	settings.setValue("gain_overlap", _ui->doubleSpinBox_gainOverlap->value());
	settings.setValue("gain_alpha", _ui->doubleSpinBox_gainAlpha->value());
	settings.setValue("gain_beta", _ui->doubleSpinBox_gainBeta->value());
	settings.setValue("gain_linked_locations", _ui->checkBox_gainLinkedLocationsOnly->isChecked());

	settings.setValue("mesh", _ui->groupBox_meshing->isChecked());
	settings.setValue("mesh_radius", _ui->doubleSpinBox_gp3Radius->value());
	settings.setValue("mesh_mu", _ui->doubleSpinBox_gp3Mu->value());
	settings.setValue("mesh_decimation_factor", _ui->doubleSpinBox_meshDecimationFactor->value());
	settings.setValue("mesh_min_cluster_size", _ui->spinBox_mesh_minClusterSize->value());

	settings.setValue("mesh_texture", _ui->checkBox_textureMapping->isChecked());

	settings.setValue("mesh_angle_tolerance", _ui->doubleSpinBox_mesh_angleTolerance->value());
	settings.setValue("mesh_quad", _ui->checkBox_mesh_quad->isChecked());
	settings.setValue("mesh_triangle_size", _ui->spinBox_mesh_triangleSize->value());

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

	_ui->comboBox_pipeline->setCurrentIndex(settings.value("pipeline", _ui->comboBox_pipeline->currentIndex()).toInt());
	_ui->checkBox_binary->setChecked(settings.value("binary", _ui->checkBox_binary->isChecked()).toBool());
	_ui->spinBox_normalKSearch->setValue(settings.value("normals_k", _ui->spinBox_normalKSearch->value()).toInt());

	_ui->groupBox_regenerate->setChecked(settings.value("regenerate", _ui->groupBox_regenerate->isChecked()).toBool());
	_ui->spinBox_decimation->setValue(settings.value("regenerate_decimation", _ui->spinBox_decimation->value()).toInt());
	_ui->doubleSpinBox_maxDepth->setValue(settings.value("regenerate_max_depth", _ui->doubleSpinBox_maxDepth->value()).toDouble());
	_ui->doubleSpinBox_minDepth->setValue(settings.value("regenerate_min_depth", _ui->doubleSpinBox_minDepth->value()).toDouble());
	_ui->lineEdit_distortionModel->setText(settings.value("regenerate_distortion_model", _ui->lineEdit_distortionModel->text()).toString());

	_ui->groupBox_bilateral->setChecked(settings.value("bilateral", _ui->groupBox_bilateral->isChecked()).toBool());
	_ui->doubleSpinBox_bilateral_sigmaS->setValue(settings.value("bilateral_sigma_s", _ui->doubleSpinBox_bilateral_sigmaS->value()).toDouble());
	_ui->doubleSpinBox_bilateral_sigmaR->setValue(settings.value("bilateral_sigma_r", _ui->doubleSpinBox_bilateral_sigmaR->value()).toDouble());

	_ui->groupBox_filtering->setChecked(settings.value("filtering", _ui->groupBox_filtering->isChecked()).toBool());
	_ui->doubleSpinBox_filteringRadius->setValue(settings.value("filtering_radius", _ui->doubleSpinBox_filteringRadius->value()).toDouble());
	_ui->spinBox_filteringMinNeighbors->setValue(settings.value("filtering_min_neighbors", _ui->spinBox_filteringMinNeighbors->value()).toInt());

	_ui->checkBox_assemble->setChecked(settings.value("assemble", _ui->checkBox_assemble->isChecked()).toBool());
	_ui->doubleSpinBox_voxelSize_assembled->setValue(settings.value("assemble_voxel", _ui->doubleSpinBox_voxelSize_assembled->value()).toDouble());

	_ui->groupBox_subtraction->setChecked(settings.value("subtract",_ui->groupBox_subtraction->isChecked()).toBool());
	_ui->doubleSpinBox_subtractPointFilteringRadius->setValue(settings.value("subtract_point_radius",_ui->doubleSpinBox_subtractPointFilteringRadius->value()).toDouble());
	_ui->doubleSpinBox_subtractPointFilteringAngle->setValue(settings.value("subtract_point_angle",_ui->doubleSpinBox_subtractPointFilteringAngle->value()).toDouble());
	_ui->spinBox_subtractFilteringMinPts->setValue(settings.value("subtract_min_neighbors",_ui->spinBox_subtractFilteringMinPts->value()).toInt());

	_ui->groupBox_mls->setChecked(settings.value("mls", _ui->groupBox_mls->isChecked()).toBool());
	_ui->doubleSpinBox_mlsRadius->setValue(settings.value("mls_radius", _ui->doubleSpinBox_mlsRadius->value()).toDouble());
	_ui->spinBox_polygonialOrder->setValue(settings.value("mls_polygonial_order", _ui->spinBox_polygonialOrder->value()).toInt());
	_ui->comboBox_upsamplingMethod->setCurrentIndex(settings.value("mls_upsampling_method", _ui->comboBox_upsamplingMethod->currentIndex()).toInt());
	_ui->doubleSpinBox_sampleRadius->setValue(settings.value("mls_upsampling_radius", _ui->doubleSpinBox_sampleRadius->value()).toDouble());
	_ui->doubleSpinBox_sampleStep->setValue(settings.value("mls_upsampling_step", _ui->doubleSpinBox_sampleStep->value()).toDouble());
	_ui->spinBox_randomPoints->setValue(settings.value("mls_point_density", _ui->spinBox_randomPoints->value()).toInt());
	_ui->doubleSpinBox_dilationVoxelSize->setValue(settings.value("mls_dilation_voxel_size", _ui->doubleSpinBox_dilationVoxelSize->value()).toDouble());
	_ui->spinBox_dilationSteps->setValue(settings.value("mls_dilation_iterations", _ui->spinBox_dilationSteps->value()).toInt());

	_ui->groupBox_gain->setChecked(settings.value("gain", _ui->groupBox_gain->isChecked()).toBool());
	_ui->doubleSpinBox_gainRadius->setValue(settings.value("gain_radius", _ui->doubleSpinBox_gainRadius->value()).toDouble());
	_ui->doubleSpinBox_gainOverlap->setValue(settings.value("gain_overlap", _ui->doubleSpinBox_gainOverlap->value()).toDouble());
	_ui->doubleSpinBox_gainAlpha->setValue(settings.value("gain_alpha", _ui->doubleSpinBox_gainAlpha->value()).toDouble());
	_ui->doubleSpinBox_gainBeta->setValue(settings.value("gain_beta", _ui->doubleSpinBox_gainBeta->value()).toDouble());
	_ui->checkBox_gainLinkedLocationsOnly->setChecked(settings.value("gain_linked_locations", _ui->checkBox_gainLinkedLocationsOnly->isChecked()).toBool());

	_ui->groupBox_meshing->setChecked(settings.value("mesh", _ui->groupBox_meshing->isChecked()).toBool());
	_ui->doubleSpinBox_gp3Radius->setValue(settings.value("mesh_radius", _ui->doubleSpinBox_gp3Radius->value()).toDouble());
	_ui->doubleSpinBox_gp3Mu->setValue(settings.value("mesh_mu", _ui->doubleSpinBox_gp3Mu->value()).toDouble());
	_ui->doubleSpinBox_meshDecimationFactor->setValue(settings.value("mesh_decimation_factor",_ui->doubleSpinBox_meshDecimationFactor->value()).toDouble());
	_ui->spinBox_mesh_minClusterSize->setValue(settings.value("mesh_min_cluster_size", _ui->spinBox_mesh_minClusterSize->value()).toInt());

	_ui->checkBox_textureMapping->setChecked(settings.value("mesh_texture", _ui->checkBox_textureMapping->isChecked()).toBool());

	_ui->doubleSpinBox_mesh_angleTolerance->setValue(settings.value("mesh_angle_tolerance", _ui->doubleSpinBox_mesh_angleTolerance->value()).toDouble());
	_ui->checkBox_mesh_quad->setChecked(settings.value("mesh_quad", _ui->checkBox_mesh_quad->isChecked()).toBool());
	_ui->spinBox_mesh_triangleSize->setValue(settings.value("mesh_triangle_size", _ui->spinBox_mesh_triangleSize->value()).toInt());

	updateReconstructionFlavor();
	updateTexturingAvailability();
	updateMLSGrpVisibility();

	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportCloudsDialog::restoreDefaults()
{
	_ui->comboBox_pipeline->setCurrentIndex(0);
	_ui->checkBox_binary->setChecked(true);
	_ui->spinBox_normalKSearch->setValue(10);

	_ui->groupBox_regenerate->setChecked(false);
	_ui->spinBox_decimation->setValue(1);
	_ui->doubleSpinBox_maxDepth->setValue(4);
	_ui->doubleSpinBox_minDepth->setValue(0);
	_ui->lineEdit_distortionModel->setText("");

	_ui->groupBox_bilateral->setChecked(false);
	_ui->doubleSpinBox_bilateral_sigmaS->setValue(10.0);
	_ui->doubleSpinBox_bilateral_sigmaR->setValue(0.1);

	_ui->groupBox_filtering->setChecked(false);
	_ui->doubleSpinBox_filteringRadius->setValue(0.02);
	_ui->spinBox_filteringMinNeighbors->setValue(2);

	_ui->checkBox_assemble->setChecked(true);
	_ui->doubleSpinBox_voxelSize_assembled->setValue(0.0);

	_ui->groupBox_subtraction->setChecked(false);
	_ui->doubleSpinBox_subtractPointFilteringRadius->setValue(0.02);
	_ui->doubleSpinBox_subtractPointFilteringAngle->setValue(0);
	_ui->spinBox_subtractFilteringMinPts->setValue(5);

	_ui->groupBox_mls->setChecked(false);
	_ui->doubleSpinBox_mlsRadius->setValue(0.04);
	_ui->spinBox_polygonialOrder->setValue(2);
	_ui->comboBox_upsamplingMethod->setCurrentIndex(0);
	_ui->doubleSpinBox_sampleRadius->setValue(0.01);
	_ui->doubleSpinBox_sampleStep->setValue(0.0);
	_ui->spinBox_randomPoints->setValue(0);
	_ui->doubleSpinBox_dilationVoxelSize->setValue(0.01);
	_ui->spinBox_dilationSteps->setValue(0);

	_ui->groupBox_gain->setChecked(false);
	_ui->doubleSpinBox_gainRadius->setValue(0.02);
	_ui->doubleSpinBox_gainOverlap->setValue(0.05);
	_ui->doubleSpinBox_gainAlpha->setValue(0.01);
	_ui->doubleSpinBox_gainBeta->setValue(10);
	_ui->checkBox_gainLinkedLocationsOnly->setChecked(true);

	_ui->groupBox_meshing->setChecked(false);
	_ui->doubleSpinBox_gp3Radius->setValue(0.04);
	_ui->doubleSpinBox_gp3Mu->setValue(2.5);
	_ui->doubleSpinBox_meshDecimationFactor->setValue(0.0);

	_ui->checkBox_textureMapping->setChecked(false);

	_ui->doubleSpinBox_mesh_angleTolerance->setValue(15.0);
	_ui->checkBox_mesh_quad->setChecked(false);
	_ui->spinBox_mesh_triangleSize->setValue(1);

	updateReconstructionFlavor();
	updateTexturingAvailability();
	updateMLSGrpVisibility();

	this->update();
}

void ExportCloudsDialog::updateReconstructionFlavor()
{
	_ui->groupBox_mls->setVisible(_ui->comboBox_pipeline->currentIndex() == 1);
	_ui->groupBox_mls->setEnabled(_ui->comboBox_pipeline->currentIndex() == 1);
	_ui->groupBox_gp3->setVisible(_ui->comboBox_pipeline->currentIndex() == 1);
	_ui->groupBox_organized->setVisible(_ui->comboBox_pipeline->currentIndex() == 0);
}

void ExportCloudsDialog::selectDistortionModel()
{
	QString dir = _ui->lineEdit_distortionModel->text();
	if(dir.isEmpty())
	{
		dir = _workingDirectory;
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Distortion model (*.bin *.txt)"));
	if(path.size())
	{
		_ui->lineEdit_distortionModel->setText(path);
	}
}

void ExportCloudsDialog::setSaveButton()
{
	_ui->buttonBox->button(QDialogButtonBox::Ok)->setVisible(false);
	_ui->buttonBox->button(QDialogButtonBox::Save)->setVisible(true);
	_ui->checkBox_binary->setVisible(true);
	_ui->checkBox_binary->setEnabled(true);
	_ui->label_binaryFile->setVisible(true);
	_ui->checkBox_mesh_quad->setVisible(false);
	_ui->checkBox_mesh_quad->setEnabled(false);
	_ui->label_quad->setVisible(false);
	updateTexturingAvailability(true);
}

void ExportCloudsDialog::setOkButton()
{
	_ui->buttonBox->button(QDialogButtonBox::Ok)->setVisible(true);
	_ui->buttonBox->button(QDialogButtonBox::Save)->setVisible(false);
	_ui->checkBox_binary->setVisible(false);
	_ui->checkBox_binary->setEnabled(false);
	_ui->label_binaryFile->setVisible(false);
	_ui->checkBox_mesh_quad->setVisible(true);
	_ui->checkBox_mesh_quad->setEnabled(true);
	_ui->label_quad->setVisible(true);
	updateTexturingAvailability(false);
}

void ExportCloudsDialog::enableRegeneration(bool enabled)
{
	if(!enabled)
	{
		_ui->groupBox_regenerate->setChecked(false);
	}
	_ui->groupBox_regenerate->setEnabled(enabled);
}

void ExportCloudsDialog::exportClouds(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, int> & mapIds,
		const QMap<int, Signature> & cachedSignatures,
		const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> > & cachedClouds,
		const QString & workingDirectory,
		const ParametersMap & parameters)
{
	std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds;
	std::map<int, pcl::PolygonMesh::Ptr> meshes;
	std::map<int, pcl::TextureMesh::Ptr> textureMeshes;

	setSaveButton();

	if(getExportedClouds(
			poses,
			links,
			mapIds,
			cachedSignatures,
			cachedClouds,
			workingDirectory,
			parameters,
			clouds,
			meshes,
			textureMeshes))
	{
		if(textureMeshes.size())
		{
			saveTextureMeshes(workingDirectory, poses, textureMeshes);
		}
		else if(meshes.size())
		{
			bool exportMeshes = true;
			if(_ui->checkBox_textureMapping->isEnabled() && _ui->checkBox_textureMapping->isChecked())
			{
				QMessageBox::StandardButton r = QMessageBox::warning(this, tr("Exporting Texture Mesh"),
						tr("No texture mesh could be created, do you want to continue with saving only the meshes (%1)?").arg(meshes.size()),
						QMessageBox::Yes|QMessageBox::No, QMessageBox::Yes);
				exportMeshes = r == QMessageBox::Yes;
			}
			if(exportMeshes)
			{
				saveMeshes(workingDirectory, poses, meshes, _ui->checkBox_binary->isChecked());
			}
		}
		else
		{
			saveClouds(workingDirectory, poses, clouds, _ui->checkBox_binary->isChecked());
		}
	}
	else if(!_canceled)
	{
		_progressDialog->setAutoClose(false);
	}
	_progressDialog->setValue(_progressDialog->maximumSteps());
}

void ExportCloudsDialog::viewClouds(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, int> & mapIds,
		const QMap<int, Signature> & cachedSignatures,
		const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> > & cachedClouds,
		const QString & workingDirectory,
		const ParametersMap & parameters)
{
	std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds;
	std::map<int, pcl::PolygonMesh::Ptr> meshes;
	std::map<int, pcl::TextureMesh::Ptr> textureMeshes;

	setOkButton();

	if(getExportedClouds(
			poses,
			links,
			mapIds,
			cachedSignatures,
			cachedClouds,
			workingDirectory,
			parameters,
			clouds,
			meshes,
			textureMeshes))
	{
		QDialog * window = new QDialog(this->parentWidget()?this->parentWidget():this, Qt::Window);
		window->setAttribute(Qt::WA_DeleteOnClose, true);
		if(meshes.size())
		{
			window->setWindowTitle(tr("Meshes (%1 nodes)").arg(meshes.size()));
		}
		else
		{
			window->setWindowTitle(tr("Clouds (%1 nodes)").arg(clouds.size()));
		}
		window->setMinimumWidth(120);
		window->setMinimumHeight(90);
		window->resize(QDesktopWidget().availableGeometry(this).size() * 0.7);

		CloudViewer * viewer = new CloudViewer(window);
		viewer->setCameraLockZ(false);
		if(_ui->comboBox_pipeline->currentIndex() == 0)
		{
			viewer->setBackfaceCulling(true, false);
		}

		QVBoxLayout *layout = new QVBoxLayout();
		layout->addWidget(viewer);
		layout->setContentsMargins(0,0,0,0);
		window->setLayout(layout);
		connect(window, SIGNAL(finished(int)), viewer, SLOT(clear()));

		window->show();

		uSleep(500);

		if(textureMeshes.size())
		{
			for(std::map<int, pcl::TextureMesh::Ptr>::iterator iter = textureMeshes.begin(); iter!=textureMeshes.end(); ++iter)
			{
				_progressDialog->appendText(tr("Viewing the mesh %1 (%2 polygons)...").arg(iter->first).arg(iter->second->tex_polygons.size()?iter->second->tex_polygons[0].size():0));
				_progressDialog->incrementStep();
				bool isRGB = false;
				for(unsigned int i=0; i<iter->second->cloud.fields.size(); ++i)
				{
					if(iter->second->cloud.fields[i].name.compare("rgb") == 0)
					{
						isRGB=true;
						break;
					}
				}
				if(isRGB)
				{
					viewer->addCloudTextureMesh(uFormat("mesh%d",iter->first), iter->second, iter->first>0?poses.at(iter->first):Transform::getIdentity());
				}
				else
				{
					viewer->addCloudTextureMesh(uFormat("mesh%d",iter->first), iter->second, iter->first>0?poses.at(iter->first):Transform::getIdentity());
				}
				_progressDialog->appendText(tr("Viewing the mesh %1 (%2 polygons)... done.").arg(iter->first).arg(iter->second->tex_polygons.size()?iter->second->tex_polygons[0].size():0));
				QApplication::processEvents();
			}
		}
		else if(meshes.size())
		{
			for(std::map<int, pcl::PolygonMesh::Ptr>::iterator iter = meshes.begin(); iter!=meshes.end(); ++iter)
			{
				_progressDialog->appendText(tr("Viewing the mesh %1 (%2 polygons)...").arg(iter->first).arg(iter->second->polygons.size()));
				_progressDialog->incrementStep();
				bool isRGB = false;
				for(unsigned int i=0; i<iter->second->cloud.fields.size(); ++i)
				{
					if(iter->second->cloud.fields[i].name.compare("rgb") == 0)
					{
						isRGB=true;
						break;
					}
				}
				if(isRGB)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::fromPCLPointCloud2(iter->second->cloud, *cloud);
					viewer->addCloudMesh(uFormat("mesh%d",iter->first), cloud, iter->second->polygons, iter->first>0?poses.at(iter->first):Transform::getIdentity());
				}
				else
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::fromPCLPointCloud2(iter->second->cloud, *cloud);
					viewer->addCloudMesh(uFormat("mesh%d",iter->first), cloud, iter->second->polygons, iter->first>0?poses.at(iter->first):Transform::getIdentity());
				}
				_progressDialog->appendText(tr("Viewing the mesh %1 (%2 polygons)... done.").arg(iter->first).arg(iter->second->polygons.size()));
				QApplication::processEvents();
			}
		}
		else if(clouds.size())
		{
			for(std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>::iterator iter = clouds.begin(); iter!=clouds.end(); ++iter)
			{
				_progressDialog->appendText(tr("Viewing the cloud %1 (%2 points)...").arg(iter->first).arg(iter->second->size()));
				_progressDialog->incrementStep();

				QColor color = Qt::gray;
				int mapId = uValue(mapIds, iter->first, -1);
				if(mapId >= 0)
				{
					color = (Qt::GlobalColor)(mapId % 12 + 7 );
				}
				viewer->addCloud(uFormat("cloud%d",iter->first), iter->second, iter->first>0?poses.at(iter->first):Transform::getIdentity());
				_progressDialog->appendText(tr("Viewing the cloud %1 (%2 points)... done.").arg(iter->first).arg(iter->second->size()));
			}
		}
		viewer->update();
	}
	else
	{
		_progressDialog->setAutoClose(false);
	}
	_progressDialog->setValue(_progressDialog->maximumSteps());
}

bool ExportCloudsDialog::removeDirRecursively(const QString & dirName)
{
    bool result = true;
    QDir dir(dirName);

    if (dir.exists(dirName)) {
        Q_FOREACH(QFileInfo info, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::System | QDir::Hidden  | QDir::AllDirs | QDir::Files, QDir::DirsFirst)) {
            if (info.isDir()) {
                result = removeDirRecursively(info.absoluteFilePath());
            }
            else {
                result = QFile::remove(info.absoluteFilePath());
            }

            if (!result) {
                return result;
            }
        }
        result = dir.rmdir(dirName);
    }
    return result;
}

bool ExportCloudsDialog::getExportedClouds(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, int> & mapIds,
		const QMap<int, Signature> & cachedSignatures,
		const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> > & cachedClouds,
		const QString & workingDirectory,
		const ParametersMap & parameters,
		std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> & cloudsWithNormals,
		std::map<int, pcl::PolygonMesh::Ptr> & meshes,
		std::map<int, pcl::TextureMesh::Ptr> & textureMeshes)
{
	_canceled = false;
	_workingDirectory = workingDirectory;
	enableRegeneration(cachedSignatures.size());
	if(this->exec() == QDialog::Accepted)
	{
		if(poses.empty())
		{
			QMessageBox::critical(this, tr("Creating clouds..."), tr("Poses are null! Cannot export/view clouds."));
			return false;
		}
		_progressDialog->resetProgress();
		_progressDialog->show();
		int mul = 1;
		if(_ui->groupBox_meshing->isChecked())
		{
			mul+=1;
		}
		if(_ui->checkBox_assemble->isChecked())
		{
			mul+=1;
		}

		if(_ui->groupBox_subtraction->isChecked())
		{
			mul+=1;
		}
		mul+=1; // normals
		if(_ui->checkBox_textureMapping->isChecked())
		{
			mul+=1;
		}
		if(_ui->groupBox_gain->isChecked())
		{
			mul+=1;
		}
		if(_ui->checkBox_mesh_quad->isEnabled()) // when enabled we are viewing the clouds
		{
			mul+=1;
		}
		_progressDialog->setMaximumSteps(int(poses.size())*mul+1);

		std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> > clouds = this->getClouds(
				poses,
				cachedSignatures,
				cachedClouds,
				parameters);

		if(_canceled)
		{
			return false;
		}

		if(clouds.empty())
		{
			_progressDialog->setAutoClose(false);
			if(_ui->groupBox_regenerate->isEnabled() && !_ui->groupBox_regenerate->isChecked())
			{
				QMessageBox::warning(this, tr("Creating clouds..."), tr("Could create clouds for %1 node(s). You "
						"may want to activate clouds regeneration option.").arg(poses.size()));
			}
			else
			{
				QMessageBox::warning(this, tr("Creating clouds..."), tr("Could not create clouds for %1 "
						"node(s). The cache may not contain point cloud data. Try re-downloading the map.").arg(poses.size()));
			}
			return false;
		}

		GainCompensator compensator(_ui->doubleSpinBox_gainRadius->value(), _ui->doubleSpinBox_gainOverlap->value(), _ui->doubleSpinBox_gainAlpha->value(), _ui->doubleSpinBox_gainBeta->value());
		if(_ui->groupBox_gain->isChecked() && clouds.size() > 1)
		{
			if(!_ui->checkBox_gainLinkedLocationsOnly->isChecked())
			{
				_progressDialog->appendText(tr("Full gain compensation of %1 clouds...").arg(clouds.size()));
			}
			else
			{
				_progressDialog->appendText(tr("Gain compensation of %1 clouds...").arg(clouds.size()));
			}
			QApplication::processEvents();
			QApplication::processEvents();

			if(!_ui->checkBox_gainLinkedLocationsOnly->isChecked())
			{
				std::multimap<int, Link> allLinks;
				for(std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> >::const_iterator iter=clouds.begin(); iter!=clouds.end(); ++iter)
				{
					int from = iter->first;
					std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> >::const_iterator jter = iter;
					++jter;
					for(;jter!=clouds.end(); ++jter)
					{
						int to = jter->first;
						allLinks.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, poses.at(from).inverse()*poses.at(to))));
					}
				}

				compensator.feed(clouds, allLinks);
			}
			else
			{
				compensator.feed(clouds, links);
			}

			_progressDialog->appendText(tr("Applying gain compensation..."));
			for(std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> >::iterator jter=clouds.begin();jter!=clouds.end(); ++jter)
			{
				if(jter!=clouds.end())
				{
					double gain = compensator.getGain(jter->first);;
					compensator.apply(jter->first, jter->second.first, jter->second.second);

					_progressDialog->appendText(tr("Cloud %1 has gain %2").arg(jter->first).arg(gain));
					_progressDialog->incrementStep();
					QApplication::processEvents();
					if(_canceled)
					{
						return false;
					}
				}
			}
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr rawAssembledCloud(new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<int> rawCameraIndices;
		if(_ui->checkBox_assemble->isChecked() &&
		   !(_ui->comboBox_pipeline->currentIndex()==0 && _ui->groupBox_meshing->isChecked()))
		{
			_progressDialog->appendText(tr("Assembling %1 clouds...").arg(clouds.size()));
			QApplication::processEvents();

			int i =0;
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			for(std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> >::iterator iter=clouds.begin();
				iter!= clouds.end();
				++iter)
			{
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				if(iter->second.first->isOrganized())
				{
					pcl::copyPointCloud(*iter->second.first, *iter->second.second, *transformed);
					transformed = util3d::transformPointCloud(transformed, poses.at(iter->first));
				}
				else
				{
					transformed = util3d::transformPointCloud(iter->second.first, poses.at(iter->first));
				}

				*assembledCloud += *transformed;
				rawCameraIndices.resize(assembledCloud->size(), iter->first);

				_progressDialog->appendText(tr("Assembled cloud %1, total=%2 (%3/%4).").arg(iter->first).arg(assembledCloud->size()).arg(++i).arg(clouds.size()));
				_progressDialog->incrementStep();
				QApplication::processEvents();
				if(_canceled)
				{
					return false;
				}
			}

			pcl::copyPointCloud(*assembledCloud, *rawAssembledCloud);

			if(_ui->doubleSpinBox_voxelSize_assembled->value())
			{
				_progressDialog->appendText(tr("Voxelize cloud (%1 points, voxel size = %2 m)...")
						.arg(assembledCloud->size())
						.arg(_ui->doubleSpinBox_voxelSize_assembled->value()));
				QApplication::processEvents();
				assembledCloud = util3d::voxelize(
						assembledCloud,
						_ui->doubleSpinBox_voxelSize_assembled->value());
			}

			clouds.clear();
			clouds.insert(std::make_pair(0, std::make_pair(assembledCloud, pcl::IndicesPtr(new std::vector<int>))));
		}

		if(_canceled)
		{
			return false;
		}

		std::map<int, Transform> viewPoints = poses;
		if(_ui->groupBox_mls->isEnabled() && _ui->groupBox_mls->isChecked())
		{
			_progressDialog->appendText(tr("Smoothing the surface using Moving Least Squares (MLS) algorithm... "
					"[search radius=%1m voxel=%2m]").arg(_ui->doubleSpinBox_mlsRadius->value()).arg(_ui->doubleSpinBox_voxelSize_assembled->value()));
			QApplication::processEvents();
			QApplication::processEvents();

			// Adjust view points with local transforms
			for(std::map<int, Transform>::iterator iter= viewPoints.begin(); iter!=viewPoints.end(); ++iter)
			{
				if(cachedSignatures.contains(iter->first))
				{
					const SensorData & data = cachedSignatures.find(iter->first)->sensorData();
					if(data.cameraModels().size() && !data.cameraModels()[0].localTransform().isNull())
					{
						iter->second *= data.cameraModels()[0].localTransform();
					}
					else if(!data.stereoCameraModel().localTransform().isNull())
					{
						iter->second *= data.stereoCameraModel().localTransform();
					}
				}
			}
		}

		//fill cloudWithNormals
		for(std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> >::iterator iter=clouds.begin();
			iter!= clouds.end();
			++iter)
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals = iter->second.first;

			if(_ui->groupBox_mls->isEnabled() && _ui->groupBox_mls->isChecked())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutNormals(new pcl::PointCloud<pcl::PointXYZRGB>);
				if(iter->second.second->size())
				{
					pcl::copyPointCloud(*cloudWithNormals, *iter->second.second, *cloudWithoutNormals);
				}
				else
				{
					pcl::copyPointCloud(*cloudWithNormals, *cloudWithoutNormals);
				}

				_progressDialog->appendText(tr("Smoothing (MLS) the cloud (%1 points)...").arg(cloudWithNormals->size()));
				QApplication::processEvents();
				if(_canceled)
				{
					return false;
				}

				cloudWithNormals = util3d::mls(
						cloudWithoutNormals,
						(float)_ui->doubleSpinBox_mlsRadius->value(),
						_ui->spinBox_polygonialOrder->value(),
						_ui->comboBox_upsamplingMethod->currentIndex(),
						(float)_ui->doubleSpinBox_sampleRadius->value(),
						(float)_ui->doubleSpinBox_sampleStep->value(),
						_ui->spinBox_randomPoints->value(),
						(float)_ui->doubleSpinBox_dilationVoxelSize->value(),
						_ui->spinBox_dilationSteps->value());

				if(_ui->checkBox_assemble->isChecked())
				{
					// Re-voxelize to make sure to have uniform density
					if(_ui->doubleSpinBox_voxelSize_assembled->value())
					{
						_progressDialog->appendText(tr("Voxelize cloud (%1 points, voxel size = %2 m)...")
								.arg(cloudWithNormals->size())
								.arg(_ui->doubleSpinBox_voxelSize_assembled->value()));
						QApplication::processEvents();

						cloudWithNormals = util3d::voxelize(
								cloudWithNormals,
								_ui->doubleSpinBox_voxelSize_assembled->value());
					}
				
					_progressDialog->appendText(tr("Update %1 normals with %2 camera views...").arg(cloudWithNormals->size()).arg(poses.size()));

					util3d::adjustNormalsToViewPoints(
							viewPoints,
							rawAssembledCloud,
							rawCameraIndices,
							cloudWithNormals);
				}
			}
			else if(iter->second.first->isOrganized() && _ui->groupBox_filtering->isChecked())
			{
				cloudWithNormals = util3d::extractIndices(iter->second.first, iter->second.second, false, true);
			}

			cloudsWithNormals.insert(std::make_pair(iter->first, cloudWithNormals));

			_progressDialog->incrementStep();
			QApplication::processEvents();
			if(_canceled)
			{
				return false;
			}
		}

		//used for organized texturing below
		std::map<int, std::vector<int> > organizedIndices;
		std::map<int, cv::Size> organizedCloudSizes;

		//mesh
		UDEBUG("Meshing=%d", _ui->groupBox_meshing->isChecked()?1:0);
		if(_ui->groupBox_meshing->isChecked())
		{
			if(_ui->comboBox_pipeline->currentIndex() == 0)
			{
				_progressDialog->appendText(tr("Organized fast mesh... "));
				QApplication::processEvents();
				QApplication::processEvents();

				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				std::vector<pcl::Vertices> mergedPolygons;

				int i=0;
				for(std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr >::iterator iter=cloudsWithNormals.begin();
					iter!= cloudsWithNormals.end();
					++iter)
				{
					if(iter->second->isOrganized())
					{
						if(iter->second->size())
						{
							Eigen::Vector3f viewpoint(0.0f,0.0f,0.0f);
							if(cachedSignatures.contains(iter->first))
							{
								const SensorData & data = cachedSignatures.find(iter->first)->sensorData();
								if(data.cameraModels().size() && !data.cameraModels()[0].localTransform().isNull())
								{
									viewpoint[0] = data.cameraModels()[0].localTransform().x();
									viewpoint[1] = data.cameraModels()[0].localTransform().y();
									viewpoint[2] = data.cameraModels()[0].localTransform().z();
								}
								else if(!data.stereoCameraModel().localTransform().isNull())
								{
									viewpoint[0] = data.stereoCameraModel().localTransform().x();
									viewpoint[1] = data.stereoCameraModel().localTransform().y();
									viewpoint[2] = data.stereoCameraModel().localTransform().z();
								}
							}
							std::vector<pcl::Vertices> polygons = util3d::organizedFastMesh(
									iter->second,
									_ui->doubleSpinBox_mesh_angleTolerance->value()*M_PI/180.0,
									_ui->checkBox_mesh_quad->isEnabled() && _ui->checkBox_mesh_quad->isChecked(),
									_ui->spinBox_mesh_triangleSize->value(),
									viewpoint);

							if(_ui->spinBox_mesh_minClusterSize->value())
							{
								// filter polygons
								std::vector<std::set<int> > neighbors;
								std::vector<std::set<int> > vertexToPolygons;
								util3d::createPolygonIndexes(polygons,
										iter->second->size(),
										neighbors,
										vertexToPolygons);
								std::list<std::list<int> > clusters = util3d::clusterPolygons(
										neighbors,
										_ui->spinBox_mesh_minClusterSize->value());
								std::vector<pcl::Vertices> filteredPolygons(polygons.size());
								int oi=0;
								for(std::list<std::list<int> >::iterator iter=clusters.begin(); iter!=clusters.end(); ++iter)
								{
									for(std::list<int>::iterator jter=iter->begin(); jter!=iter->end(); ++jter)
									{
										filteredPolygons[oi++] = polygons.at(*jter);
									}
								}
								filteredPolygons.resize(oi);
								polygons = filteredPolygons;
							}

							_progressDialog->appendText(tr("Mesh %1 created with %2 polygons (%3/%4).").arg(iter->first).arg(polygons.size()).arg(++i).arg(clouds.size()));

							pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr denseCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
							std::vector<pcl::Vertices> densePolygons;
							std::vector<int> denseToOrganizedIndices = util3d::filterNotUsedVerticesFromMesh(*iter->second, polygons, *denseCloud, densePolygons);

							if(!_ui->checkBox_assemble->isChecked() ||
								 (_ui->checkBox_textureMapping->isEnabled() &&
								  _ui->checkBox_textureMapping->isChecked() &&
								  _ui->doubleSpinBox_voxelSize_assembled->value() == 0.0)) // don't assemble now if we are texturing
							{
								if(_ui->checkBox_assemble->isChecked())
								{
									denseCloud = util3d::transformPointCloud(denseCloud, poses.at(iter->first));
								}

								pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
								pcl::toPCLPointCloud2(*denseCloud, mesh->cloud);
								mesh->polygons = densePolygons;
								if(_ui->doubleSpinBox_meshDecimationFactor->isEnabled() &&
								   _ui->doubleSpinBox_meshDecimationFactor->value() > 0.0)
								{
									int count = mesh->polygons.size();
									mesh = util3d::meshDecimation(mesh, (float)_ui->doubleSpinBox_meshDecimationFactor->value());
									_progressDialog->appendText(tr("Mesh decimation (factor=%1) from %2 to %3 polygons").arg(_ui->doubleSpinBox_meshDecimationFactor->value()).arg(count).arg(mesh->polygons.size()));
								}
								else
								{
									organizedIndices.insert(std::make_pair(iter->first, denseToOrganizedIndices));
									organizedCloudSizes.insert(std::make_pair(iter->first, cv::Size(iter->second->width, iter->second->height)));
								}
								meshes.insert(std::make_pair(iter->first, mesh));
							}
							else
							{
								denseCloud = util3d::transformPointCloud(denseCloud, poses.at(iter->first));
								if(mergedClouds->size() == 0)
								{
									*mergedClouds = *denseCloud;
									mergedPolygons = densePolygons;
								}
								else
								{
									util3d::appendMesh(*mergedClouds, mergedPolygons, *denseCloud, densePolygons);
								}
							}
						}
						else
						{
							_progressDialog->appendText(tr("Mesh %1 not created (no valid points) (%2/%3).").arg(iter->first).arg(++i).arg(clouds.size()));
						}
					}
					else
					{
						_progressDialog->appendText(tr("Mesh %1 not created (cloud is not organized). You may want to check cloud regeneration option (%2/%3).").arg(iter->first).arg(++i).arg(clouds.size()));
					}

					_progressDialog->incrementStep();
					QApplication::processEvents();
					if(_canceled)
					{
						return false;
					}
				}

				if(_ui->checkBox_assemble->isChecked() && mergedClouds->size())
				{
					if(_ui->doubleSpinBox_voxelSize_assembled->value())
					{
						_progressDialog->appendText(tr("Filtering assembled mesh for close vertices (points=%1, polygons=%2)...").arg(mergedClouds->size()).arg(mergedPolygons.size()));
						QApplication::processEvents();

						mergedPolygons = util3d::filterCloseVerticesFromMesh(
								mergedClouds,
								mergedPolygons,
								_ui->doubleSpinBox_voxelSize_assembled->value(),
								M_PI/4,
								true);

						// filter invalid polygons
						unsigned int count = mergedPolygons.size();
						mergedPolygons = util3d::filterInvalidPolygons(mergedPolygons);
						_progressDialog->appendText(tr("Filtered %1 invalid polygons.").arg(count-mergedPolygons.size()));
						QApplication::processEvents();

						// filter not used vertices
						pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
						std::vector<pcl::Vertices> filteredPolygons;
						util3d::filterNotUsedVerticesFromMesh(*mergedClouds, mergedPolygons, *filteredCloud, filteredPolygons);
						mergedClouds = filteredCloud;
						mergedPolygons = filteredPolygons;
					}

					pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
					pcl::toPCLPointCloud2(*mergedClouds, mesh->cloud);
					mesh->polygons = mergedPolygons;

					if(_ui->doubleSpinBox_meshDecimationFactor->isEnabled() &&
					   _ui->doubleSpinBox_meshDecimationFactor->value() > 0.0)
					{
						int count = mesh->polygons.size();
						mesh = util3d::meshDecimation(mesh, (float)_ui->doubleSpinBox_meshDecimationFactor->value());
						_progressDialog->appendText(tr("Assembled mesh decimation (factor=%1) from %2 to %3 polygons").arg(_ui->doubleSpinBox_meshDecimationFactor->value()).arg(count).arg(mesh->polygons.size()));
					}

					meshes.insert(std::make_pair(0, mesh));

					_progressDialog->incrementStep();
					QApplication::processEvents();
				}
			}
			else
			{
				_progressDialog->appendText(tr("Greedy projection triangulation... [radius=%1m]").arg(_ui->doubleSpinBox_gp3Radius->value()));
				QApplication::processEvents();
				QApplication::processEvents();

				int i=0;
				for(std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>::iterator iter=cloudsWithNormals.begin();
					iter!= cloudsWithNormals.end();
					++iter)
				{
					pcl::PolygonMesh::Ptr mesh = util3d::createMesh(
							iter->second,
							_ui->doubleSpinBox_gp3Radius->value(),
							_ui->doubleSpinBox_gp3Mu->value());

					if(_ui->spinBox_mesh_minClusterSize->value())
					{
						// filter polygons
						std::vector<std::set<int> > neighbors;
						std::vector<std::set<int> > vertexToPolygons;
						util3d::createPolygonIndexes(mesh->polygons,
								mesh->cloud.height*mesh->cloud.width,
								neighbors,
								vertexToPolygons);
						std::list<std::list<int> > clusters = util3d::clusterPolygons(
								neighbors,
								_ui->spinBox_mesh_minClusterSize->value());
						std::vector<pcl::Vertices> filteredPolygons(mesh->polygons.size());
						int oi=0;
						for(std::list<std::list<int> >::iterator iter=clusters.begin(); iter!=clusters.end(); ++iter)
						{
							for(std::list<int>::iterator jter=iter->begin(); jter!=iter->end(); ++jter)
							{
								filteredPolygons[oi++] = mesh->polygons.at(*jter);
							}
						}
						filteredPolygons.resize(oi);
						mesh->polygons = filteredPolygons;
					}

					_progressDialog->appendText(tr("Mesh %1 created with %2 polygons (%3/%4).").arg(iter->first).arg(mesh->polygons.size()).arg(++i).arg(clouds.size()));
					QApplication::processEvents();

					if(_ui->doubleSpinBox_meshDecimationFactor->isEnabled() &&
					   _ui->doubleSpinBox_meshDecimationFactor->value() > 0.0)
					{
						int count = mesh->polygons.size();
						mesh = util3d::meshDecimation(mesh, (float)_ui->doubleSpinBox_meshDecimationFactor->value());
						_progressDialog->appendText(tr("Assembled mesh decimation (factor=%1) from %2 to %3 polygons").arg(_ui->doubleSpinBox_meshDecimationFactor->value()).arg(count).arg(mesh->polygons.size()));
					}

					meshes.insert(std::make_pair(iter->first, mesh));

					_progressDialog->incrementStep();
					QApplication::processEvents();
					if(_canceled)
					{
						return false;
					}
				}
			}
		}

		if(_canceled)
		{
			return false;
		}

		// texture mesh
		UDEBUG("texture mapping=%d", _ui->checkBox_textureMapping->isEnabled() && _ui->checkBox_textureMapping->isChecked()?1:0);
		if(_ui->checkBox_textureMapping->isEnabled() && _ui->checkBox_textureMapping->isChecked())
		{
			QDir dir(workingDirectory);
			removeDirRecursively(workingDirectory+QDir::separator()+"tmp_textures");
			dir.mkdir("tmp_textures");
			int i=0;
			for(std::map<int, pcl::PolygonMesh::Ptr>::iterator iter=meshes.begin();
				iter!= meshes.end();
				++iter)
			{
				std::map<int, Transform> cameras;
				if(iter->first == 0)
				{
					cameras = poses;
				}
				else
				{
					UASSERT(uContains(poses, iter->first));
					cameras.insert(std::make_pair(iter->first, _ui->checkBox_assemble->isChecked()?poses.at(iter->first):Transform::getIdentity()));
				}
				std::map<int, Transform> cameraPoses;
				std::map<int, CameraModel> cameraModels;
				std::map<int, cv::Mat> images;
				for(std::map<int, Transform>::iterator jter=cameras.begin(); jter!=cameras.end(); ++jter)
				{
					if(cachedSignatures.contains(jter->first))
					{
						const Signature & s = cachedSignatures.value(jter->first);
						CameraModel model;
						if(s.sensorData().stereoCameraModel().isValidForProjection())
						{
							model = s.sensorData().stereoCameraModel().left();
						}
						else if(s.sensorData().cameraModels().size() == 1 && s.sensorData().cameraModels()[0].isValidForProjection())
						{
							model = s.sensorData().cameraModels()[0];
						}
						cv::Mat image = s.sensorData().imageRaw();
						if(image.empty() && !s.sensorData().imageCompressed().empty())
						{
							s.sensorData().uncompressDataConst(&image, 0, 0, 0);
						}
						if(!jter->second.isNull() && model.isValidForProjection() && !image.empty())
						{
							cameraPoses.insert(std::make_pair(jter->first, jter->second));
							cameraModels.insert(std::make_pair(jter->first, model));
							if(_ui->groupBox_gain->isChecked() && compensator.getIndex(jter->first) >= 0)
							{
								compensator.apply(jter->first, image);
							}
							images.insert(std::make_pair(jter->first, image));
						}
					}
				}
				if(cameraPoses.size())
				{
					pcl::TextureMesh::Ptr textureMesh(new pcl::TextureMesh);
					std::map<int, std::vector<int> >::iterator oter = organizedIndices.find(iter->first);
					std::map<int, cv::Size>::iterator ster = organizedCloudSizes.find(iter->first);
					if(iter->first != 0 && oter != organizedIndices.end())
					{
						UASSERT(ster!=organizedCloudSizes.end()&&ster->first == oter->first);
						UDEBUG("Texture by pixels");
						textureMesh->cloud = iter->second->cloud;
						textureMesh->tex_polygons.push_back(iter->second->polygons);
						int w = ster->second.width;
						int h = ster->second.height;
						UASSERT(w > 1 && h > 1);
						if(textureMesh->tex_polygons.size() && textureMesh->tex_polygons[0].size())
						{
							textureMesh->tex_coordinates.resize(1);
							if(!_ui->checkBox_mesh_quad->isEnabled()) // disabled -> we are exporting to file
							{
								UDEBUG("");
								// When saving to file, tex_coordinates should be linked to polygon vertices, not points
								int polygonSize = textureMesh->tex_polygons[0][0].vertices.size();
								textureMesh->tex_coordinates[0].resize(polygonSize*textureMesh->tex_polygons[0].size());
								for(unsigned int i=0; i<textureMesh->tex_polygons[0].size(); ++i)
								{
									const pcl::Vertices & vertices = textureMesh->tex_polygons[0][i];
									UASSERT(polygonSize == (int)vertices.vertices.size());
									for(int k=0; k<polygonSize; ++k)
									{
										//uv
										UASSERT(vertices.vertices[k] < oter->second.size());
										int originalVertex = oter->second[vertices.vertices[k]];
										textureMesh->tex_coordinates[0][i*polygonSize+k] = Eigen::Vector2f(
												float(originalVertex % w) / float(w),      // u
												float(h - originalVertex / w) / float(h)); // v
									}
								}
							}
							else
							{
								UDEBUG("");
								int nPoints = textureMesh->cloud.data.size()/textureMesh->cloud.point_step;
								textureMesh->tex_coordinates[0].resize(nPoints);
								for(int i=0; i<nPoints; ++i)
								{
									//uv
									UASSERT(i < (int)oter->second.size());
									int originalVertex = oter->second[i];
									textureMesh->tex_coordinates[0][i] = Eigen::Vector2f(
											float(originalVertex % w) / float(w),      // u
											float(h - originalVertex / w) / float(h)); // v
								}
							}

							pcl::TexMaterial mesh_material;
							mesh_material.tex_d = 1.0f;
							mesh_material.tex_Ns = 75.0f;
							mesh_material.tex_illum = 1;

							std::stringstream tex_name;
							tex_name << "material_" << iter->first;
							tex_name >> mesh_material.tex_name;

							std::string tmpDirectory = dir.filePath("tmp_textures").toStdString();
							mesh_material.tex_file = uFormat("%s/%s%d.png", tmpDirectory.c_str(), "texture_", iter->first);
							if(!cv::imwrite(mesh_material.tex_file, images.at(iter->first)))
							{
								UERROR("Cannot save texture of image %d", iter->first);
							}
							else
							{
								UINFO("Saved temporary texture: \"%s\"", mesh_material.tex_file.c_str());
							}

							textureMesh->tex_materials.push_back(mesh_material);
						}
						else
						{
							UWARN("No polygons for mesh %d!?", iter->first);
						}
					}
					else
					{
						UDEBUG("Texture by projection");
						textureMesh = util3d::createTextureMesh(
								iter->second,
								cameraPoses,
								cameraModels,
								images,
								dir.filePath("tmp_textures").toStdString(),
								_ui->spinBox_normalKSearch->value());

						if(!_ui->checkBox_binary->isEnabled() && // not enabled -> we are not exporting to file
							textureMesh->tex_coordinates.size() &&
							textureMesh->tex_coordinates[0].size()) // assume first is the texture, second is occluded texture
						{
							UDEBUG("");
							// When not saving to file, tex_coordinates should be linked to points, not polygon vertices
							int nPoints = textureMesh->cloud.data.size()/textureMesh->cloud.point_step;
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
							std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > tmpCoordinates = textureMesh->tex_coordinates[0];
#else
							std::vector<Eigen::Vector2f> tmpCoordinates = textureMesh->tex_coordinates[0];
#endif
							textureMesh->tex_coordinates[0].resize(nPoints);

							int polygonSize = textureMesh->tex_polygons[0][0].vertices.size();
							UASSERT(textureMesh->tex_polygons[0].size() == tmpCoordinates.size()/polygonSize);
							for(unsigned int i=0; i<textureMesh->tex_polygons[0].size(); ++i)
							{
								const pcl::Vertices & vertices = textureMesh->tex_polygons[0][i];
								UASSERT(polygonSize == (int)vertices.vertices.size());
								for(int j=0; j<polygonSize; ++j)
								{
									//uv
									UASSERT((int)vertices.vertices[j] < nPoints);
									UASSERT(i*polygonSize+j < tmpCoordinates.size());
									textureMesh->tex_coordinates[0][vertices.vertices[j]] = tmpCoordinates[i*polygonSize+j];
								}
							}
						}
					}

					textureMeshes.insert(std::make_pair(iter->first, textureMesh));
				}
				else
				{
					UWARN("No camera poses!?");
				}

				_progressDialog->appendText(tr("TextureMesh %1 created [cameras=%2] (%3/%4).").arg(iter->first).arg(cameraPoses.size()).arg(++i).arg(meshes.size()));
				_progressDialog->incrementStep();
				QApplication::processEvents();
				if(_canceled)
				{
					return false;
				}
			}

			if(textureMeshes.size() > 1 && _ui->checkBox_assemble->isChecked())
			{
				UDEBUG("Concatenate texture meshes");
				_progressDialog->appendText(tr("Assembling %1 meshes...").arg(textureMeshes.size()));
				QApplication::processEvents();
				uSleep(100);
				QApplication::processEvents();

				pcl::TextureMesh::Ptr assembledMesh = util3d::concatenateTextureMeshes(uValuesList(textureMeshes));
				textureMeshes.clear();
				textureMeshes.insert(std::make_pair(0, assembledMesh));
			}

		}
		UDEBUG("");
		return true;
	}
	return false;
}

std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> > ExportCloudsDialog::getClouds(
		const std::map<int, Transform> & poses,
		const QMap<int, Signature> & cachedSignatures,
		const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> > & cachedClouds,
		const ParametersMap & parameters) const
{
	std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> > clouds;
	int index=1;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr previousCloud;
	pcl::IndicesPtr previousIndices;
	Transform previousPose;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end() && !_canceled; ++iter, ++index)
	{
		int points = 0;
		int totalIndices = 0;
		if(!iter->second.isNull())
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::IndicesPtr indices(new std::vector<int>);
			if(_ui->groupBox_regenerate->isChecked())
			{
				if(cachedSignatures.contains(iter->first))
				{
					const Signature & s = cachedSignatures.find(iter->first).value();
					SensorData d = s.sensorData();
					cv::Mat image, depth;
					d.uncompressData(&image, &depth, 0);
					if(!image.empty() && !depth.empty())
					{
						if(!_ui->lineEdit_distortionModel->text().isEmpty() &&
						   QFileInfo(_ui->lineEdit_distortionModel->text()).exists())
						{
							clams::DiscreteDepthDistortionModel model;
							model.load(_ui->lineEdit_distortionModel->text().toStdString());
							depth = depth.clone();// make sure we are not modifying data in cached signatures.
							model.undistort(depth);
							d.setDepthOrRightRaw(depth);
						}

						// bilateral filtering
						if(_ui->groupBox_bilateral->isChecked())
						{
							depth = util2d::fastBilateralFiltering(depth,
									_ui->doubleSpinBox_bilateral_sigmaS->value(),
									_ui->doubleSpinBox_bilateral_sigmaR->value());
							d.setDepthOrRightRaw(depth);
						}

						UASSERT(iter->first == d.id());
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutNormals;
						cloudWithoutNormals = util3d::cloudRGBFromSensorData(
								d,
								_ui->spinBox_decimation->value() == 0?1:_ui->spinBox_decimation->value(),
								_ui->doubleSpinBox_maxDepth->value(),
								_ui->doubleSpinBox_minDepth->value(),
								indices.get(),
								parameters);

						if(cloudWithoutNormals->size())
						{
							// Don't voxelize if we create organized mesh
							if(!(_ui->comboBox_pipeline->currentIndex()==0 && _ui->groupBox_meshing->isChecked()) && _ui->doubleSpinBox_voxelSize_assembled->value()>0.0)
							{
								cloudWithoutNormals = util3d::voxelize(cloudWithoutNormals, indices, _ui->doubleSpinBox_voxelSize_assembled->value());
								indices->resize(cloudWithoutNormals->size());
								for(unsigned int i=0; i<indices->size(); ++i)
								{
									indices->at(i) = i;
								}
							}

							// view point
							Eigen::Vector3f viewPoint(0.0f,0.0f,0.0f);
							if(d.cameraModels().size() && !d.cameraModels()[0].localTransform().isNull())
							{
								viewPoint[0] = d.cameraModels()[0].localTransform().x();
								viewPoint[1] = d.cameraModels()[0].localTransform().y();
								viewPoint[2] = d.cameraModels()[0].localTransform().z();
							}
							else if(!d.stereoCameraModel().localTransform().isNull())
							{
								viewPoint[0] = d.stereoCameraModel().localTransform().x();
								viewPoint[1] = d.stereoCameraModel().localTransform().y();
								viewPoint[2] = d.stereoCameraModel().localTransform().z();
							}

							pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloudWithoutNormals, indices, _ui->spinBox_normalKSearch->value(), viewPoint);
							pcl::concatenateFields(*cloudWithoutNormals, *normals, *cloud);

							if(_ui->groupBox_subtraction->isChecked() &&
							   _ui->doubleSpinBox_subtractPointFilteringRadius->value() > 0.0)
							{
								pcl::IndicesPtr beforeSubtractionIndices = indices;
								if(	cloud->size() &&
									previousCloud.get() != 0 &&
									previousIndices.get() != 0 &&
									previousIndices->size() &&
									!previousPose.isNull())
								{
									rtabmap::Transform t = iter->second.inverse() * previousPose;
									pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloud = rtabmap::util3d::transformPointCloud(previousCloud, t);
									indices = rtabmap::util3d::subtractFiltering(
											cloud,
											indices,
											transformedCloud,
											previousIndices,
											_ui->doubleSpinBox_subtractPointFilteringRadius->value(),
											_ui->doubleSpinBox_subtractPointFilteringAngle->value(),
											_ui->spinBox_subtractFilteringMinPts->value());
								}
								previousCloud = cloud;
								previousIndices = beforeSubtractionIndices;
								previousPose = iter->second;
							}
						}
					}
				}
				else
				{
					UERROR("Cloud %d not found in cache!", iter->first);
				}
			}
			else if(uContains(cachedClouds, iter->first))
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutNormals;
				if(!_ui->groupBox_meshing->isChecked() &&
				   _ui->doubleSpinBox_voxelSize_assembled->value() > 0.0)
				{
					cloudWithoutNormals = util3d::voxelize(
							cachedClouds.at(iter->first).first,
							cachedClouds.at(iter->first).second,
							_ui->doubleSpinBox_voxelSize_assembled->value());

					//generate indices for all points (they are all valid)
					indices->resize(cloudWithoutNormals->size());
					for(unsigned int i=0; i<cloudWithoutNormals->size(); ++i)
					{
						indices->at(i) = i;
					}
				}
				else
				{
					cloudWithoutNormals = cachedClouds.at(iter->first).first;
					indices = cachedClouds.at(iter->first).second;
				}

				// view point
				Eigen::Vector3f viewPoint(0.0f,0.0f,0.0f);
				if(cachedSignatures.contains(iter->first))
				{
					const Signature & s = cachedSignatures.find(iter->first).value();
					SensorData d = s.sensorData();
					if(d.cameraModels().size() && !d.cameraModels()[0].localTransform().isNull())
					{
						viewPoint[0] = d.cameraModels()[0].localTransform().x();
						viewPoint[1] = d.cameraModels()[0].localTransform().y();
						viewPoint[2] = d.cameraModels()[0].localTransform().z();
					}
					else if(!d.stereoCameraModel().localTransform().isNull())
					{
						viewPoint[0] = d.stereoCameraModel().localTransform().x();
						viewPoint[1] = d.stereoCameraModel().localTransform().y();
						viewPoint[2] = d.stereoCameraModel().localTransform().z();
					}
				}
				else
				{
					_progressDialog->appendText(tr("Cached cloud %1 is not found in cached data, the view point for normal computation will not be set (%2/%3).").arg(iter->first).arg(index).arg(poses.size()), Qt::darkYellow);
				}

				pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloudWithoutNormals, indices, _ui->spinBox_normalKSearch->value(), viewPoint);
				pcl::concatenateFields(*cloudWithoutNormals, *normals, *cloud);
			}
			else
			{
				_progressDialog->appendText(tr("Cached cloud %1 not found. You may want to regenerate the clouds (%2/%3).").arg(iter->first).arg(index).arg(poses.size()), Qt::darkYellow);
			}

			if(indices->size())
			{
				if(_ui->groupBox_filtering->isChecked() &&
					_ui->doubleSpinBox_filteringRadius->value() > 0.0f &&
					_ui->spinBox_filteringMinNeighbors->value() > 0)
				{
					indices = util3d::radiusFiltering(cloud, indices, _ui->doubleSpinBox_filteringRadius->value(), _ui->spinBox_filteringMinNeighbors->value());
				}

				clouds.insert(std::make_pair(iter->first, std::make_pair(cloud, indices)));
				points = cloud->size();
				totalIndices = indices->size();
			}
		}
		else
		{
			UERROR("transform is null!?");
		}

		if(points>0)
		{
			if(_ui->groupBox_regenerate->isChecked())
			{
				_progressDialog->appendText(tr("Generated cloud %1 with %2 points and %3 indices (%4/%5).")
						.arg(iter->first).arg(points).arg(totalIndices).arg(index).arg(poses.size()));
			}
			else
			{
				_progressDialog->appendText(tr("Copied cloud %1 from cache with %2 points and %3 indices (%4/%5).")
						.arg(iter->first).arg(points).arg(totalIndices).arg(index).arg(poses.size()));
			}
		}
		else
		{
			_progressDialog->appendText(tr("Ignored cloud %1 (%2/%3).").arg(iter->first).arg(index).arg(poses.size()));
		}
		_progressDialog->incrementStep();
		QApplication::processEvents();
	}

	return clouds;
}


void ExportCloudsDialog::saveClouds(
		const QString & workingDirectory,
		const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> & clouds,
		bool binaryMode)
{
	if(clouds.size() == 1)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save cloud to ..."), workingDirectory+QDir::separator()+"cloud.ply", tr("Point cloud data (*.ply *.pcd)"));
		if(!path.isEmpty())
		{
			if(clouds.begin()->second->size())
			{
				_progressDialog->appendText(tr("Saving the cloud (%1 points)...").arg(clouds.begin()->second->size()));

				bool success =false;
				if(QFileInfo(path).suffix() == "pcd")
				{
					success = pcl::io::savePCDFile(path.toStdString(), *clouds.begin()->second, binaryMode) == 0;
				}
				else if(QFileInfo(path).suffix() == "ply")
				{
					success = pcl::io::savePLYFile(path.toStdString(), *clouds.begin()->second, binaryMode) == 0;
				}
				else if(QFileInfo(path).suffix() == "")
				{
					//use ply by default
					path += ".ply";
					success = pcl::io::savePLYFile(path.toStdString(), *clouds.begin()->second, binaryMode) == 0;
				}
				else
				{
					UERROR("Extension not recognized! (%s) Should be one of (*.ply *.pcd).", QFileInfo(path).suffix().toStdString().c_str());
				}
				if(success)
				{
					_progressDialog->incrementStep();
					_progressDialog->appendText(tr("Saving the cloud (%1 points)... done.").arg(clouds.begin()->second->size()));

					QMessageBox::information(this, tr("Save successful!"), tr("Cloud saved to \"%1\"").arg(path));
				}
				else
				{
					QMessageBox::warning(this, tr("Save failed!"), tr("Failed to save to \"%1\"").arg(path));
				}
			}
			else
			{
				QMessageBox::warning(this, tr("Save failed!"), tr("Cloud is empty..."));
			}
		}
	}
	else if(clouds.size())
	{
		QString path = QFileDialog::getExistingDirectory(this, tr("Save clouds to (*.ply *.pcd)..."), workingDirectory, 0);
		if(!path.isEmpty())
		{
			bool ok = false;
			QStringList items;
			items.push_back("ply");
			items.push_back("pcd");
			QString suffix = QInputDialog::getItem(this, tr("File format"), tr("Which format?"), items, 0, false, &ok);

			if(ok)
			{
				QString prefix = QInputDialog::getText(this, tr("File prefix"), tr("Prefix:"), QLineEdit::Normal, "cloud", &ok);

				if(ok)
				{
					for(std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr >::const_iterator iter=clouds.begin(); iter!=clouds.end(); ++iter)
					{
						if(iter->second->size())
						{
							pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloud;
							transformedCloud = util3d::transformPointCloud(iter->second, poses.at(iter->first));

							QString pathFile = path+QDir::separator()+QString("%1%2.%3").arg(prefix).arg(iter->first).arg(suffix);
							bool success =false;
							if(suffix == "pcd")
							{
								success = pcl::io::savePCDFile(pathFile.toStdString(), *transformedCloud, binaryMode) == 0;
							}
							else if(suffix == "ply")
							{
								success = pcl::io::savePLYFile(pathFile.toStdString(), *transformedCloud, binaryMode) == 0;
							}
							else
							{
								UFATAL("Extension not recognized! (%s)", suffix.toStdString().c_str());
							}
							if(success)
							{
								_progressDialog->appendText(tr("Saved cloud %1 (%2 points) to %3.").arg(iter->first).arg(iter->second->size()).arg(pathFile));
							}
							else
							{
								_progressDialog->appendText(tr("Failed saving cloud %1 (%2 points) to %3.").arg(iter->first).arg(iter->second->size()).arg(pathFile), Qt::darkRed);
								_progressDialog->setAutoClose(false);
							}
						}
						else
						{
							_progressDialog->appendText(tr("Cloud %1 is empty!").arg(iter->first));
						}
						_progressDialog->incrementStep();
						QApplication::processEvents();
						if(_canceled)
						{
							return;
						}
					}
				}
			}
		}
	}
}

void ExportCloudsDialog::saveMeshes(
		const QString & workingDirectory,
		const std::map<int, Transform> & poses,
		const std::map<int, pcl::PolygonMesh::Ptr> & meshes,
		bool binaryMode)
{
	if(meshes.size() == 1)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save mesh to ..."), workingDirectory+QDir::separator()+"mesh.ply", tr("Mesh (*.ply)"));
		if(!path.isEmpty())
		{
			if(meshes.begin()->second->polygons.size())
			{
				_progressDialog->appendText(tr("Saving the mesh (%1 polygons)...").arg(meshes.begin()->second->polygons.size()));
				QApplication::processEvents();
				uSleep(100);
				QApplication::processEvents();

				bool success =false;
				if(QFileInfo(path).suffix() == "")
				{
					path += ".ply";
				}

				if(QFileInfo(path).suffix() == "ply")
				{
					if(binaryMode)
					{
						success = pcl::io::savePLYFileBinary(path.toStdString(), *meshes.begin()->second) == 0;
					}
					else
					{
						success = pcl::io::savePLYFile(path.toStdString(), *meshes.begin()->second) == 0;
					}
				}
				else if(QFileInfo(path).suffix() == "obj")
				{
					success = pcl::io::saveOBJFile(path.toStdString(), *meshes.begin()->second) == 0;
				}
				else
				{
					UERROR("Extension not recognized! (%s) Should be (*.ply).", QFileInfo(path).suffix().toStdString().c_str());
				}
				if(success)
				{
					_progressDialog->incrementStep();
					_progressDialog->appendText(tr("Saving the mesh (%1 polygons)... done.").arg(meshes.begin()->second->polygons.size()));

					QMessageBox::information(this, tr("Save successful!"), tr("Mesh saved to \"%1\"").arg(path));
				}
				else
				{
					QMessageBox::warning(this, tr("Save failed!"), tr("Failed to save to \"%1\"").arg(path));
				}
			}
			else
			{
				QMessageBox::warning(this, tr("Save failed!"), tr("Cloud is empty..."));
			}
		}
	}
	else if(meshes.size())
	{
		QString path = QFileDialog::getExistingDirectory(this, tr("Save meshes to (*.ply *.obj)..."), workingDirectory, 0);
		if(!path.isEmpty())
		{
			bool ok = false;
			QStringList items;
			items.push_back("ply");
			items.push_back("obj");
			QString suffix = QInputDialog::getItem(this, tr("File format"), tr("Which format?"), items, 0, false, &ok);

			if(ok)
			{
				QString prefix = QInputDialog::getText(this, tr("File prefix"), tr("Prefix:"), QLineEdit::Normal, "mesh", &ok);

				if(ok)
				{
					for(std::map<int, pcl::PolygonMesh::Ptr>::const_iterator iter=meshes.begin(); iter!=meshes.end(); ++iter)
					{
						if(iter->second->polygons.size())
						{
							pcl::PolygonMesh mesh;
							mesh.polygons = iter->second->polygons;
							bool isRGB = false;
							for(unsigned int i=0; i<iter->second->cloud.fields.size(); ++i)
							{
								if(iter->second->cloud.fields[i].name.compare("rgb") == 0)
								{
									isRGB=true;
									break;
								}
							}
							if(isRGB)
							{
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
								pcl::fromPCLPointCloud2(iter->second->cloud, *tmp);
								tmp = util3d::transformPointCloud(tmp, poses.at(iter->first));
								pcl::toPCLPointCloud2(*tmp, mesh.cloud);
							}
							else
							{
								pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
								pcl::fromPCLPointCloud2(iter->second->cloud, *tmp);
								tmp = util3d::transformPointCloud(tmp, poses.at(iter->first));
								pcl::toPCLPointCloud2(*tmp, mesh.cloud);
							}

							QString pathFile = path+QDir::separator()+QString("%1%2.%3").arg(prefix).arg(iter->first).arg(suffix);
							bool success =false;
							if(suffix == "ply")
							{
								if(binaryMode)
								{
									success = pcl::io::savePLYFileBinary(pathFile.toStdString(), mesh) == 0;
								}
								else
								{
									success = pcl::io::savePLYFile(pathFile.toStdString(), mesh) == 0;
								}
							}
							else if(suffix == "obj")
							{
								success = pcl::io::saveOBJFile(pathFile.toStdString(), mesh) == 0;
							}
							else
							{
								UFATAL("Extension not recognized! (%s)", suffix.toStdString().c_str());
							}
							if(success)
							{
								_progressDialog->appendText(tr("Saved mesh %1 (%2 polygons) to %3.")
										.arg(iter->first).arg(iter->second->polygons.size()).arg(pathFile));
							}
							else
							{
								_progressDialog->appendText(tr("Failed saving mesh %1 (%2 polygons) to %3.")
										.arg(iter->first).arg(iter->second->polygons.size()).arg(pathFile), Qt::darkRed);
								_progressDialog->setAutoClose(false);
							}
						}
						else
						{
							_progressDialog->appendText(tr("Mesh %1 is empty!").arg(iter->first));
						}
						_progressDialog->incrementStep();
						QApplication::processEvents();
						if(_canceled)
						{
							return;
						}
					}
				}
			}
		}
	}
}

void ExportCloudsDialog::saveTextureMeshes(
		const QString & workingDirectory,
		const std::map<int, Transform> & poses,
		const std::map<int, pcl::TextureMesh::Ptr> & meshes)
{
	if(meshes.size() == 1)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save texture mesh to ..."), workingDirectory+QDir::separator()+"mesh.obj", tr("Mesh (*.obj)"));
		if(!path.isEmpty())
		{
			if(meshes.begin()->second->tex_materials.size())
			{
				_progressDialog->appendText(tr("Saving the mesh (with %1 textures)...").arg(meshes.begin()->second->tex_materials.size()));
				QApplication::processEvents();
				uSleep(100);
				QApplication::processEvents();

				bool success =false;
				if(QFileInfo(path).suffix() == "")
				{
					path += ".obj";
				}

				pcl::TextureMesh mesh;
				mesh.tex_coordinates = meshes.begin()->second->tex_coordinates;
				mesh.tex_materials = meshes.begin()->second->tex_materials;
				removeDirRecursively(QFileInfo(path).absoluteDir().absolutePath()+QDir::separator()+QFileInfo(path).baseName());
				QDir(QFileInfo(path).absoluteDir().absolutePath()).mkdir(QFileInfo(path).baseName());
				for(unsigned int i=0;i<meshes.begin()->second->tex_materials.size(); ++i)
				{
					QFileInfo info(mesh.tex_materials[i].tex_file.c_str());
					QString fullPath = QFileInfo(path).absoluteDir().absolutePath()+QDir::separator()+QFileInfo(path).baseName()+QDir::separator()+info.fileName();
					// relative path
					mesh.tex_materials[i].tex_file=(QFileInfo(path).baseName()+QDir::separator()+info.fileName()).toStdString();
					if(!QFile::copy(meshes.begin()->second->tex_materials[i].tex_file.c_str(), fullPath))
					{
						_progressDialog->appendText(tr("Failed copying texture \"%1\" to \"%2\".")
								.arg(meshes.begin()->second->tex_materials[i].tex_file.c_str()).arg(fullPath), Qt::darkRed);
						_progressDialog->setAutoClose(false);
					}
				}
				mesh.tex_polygons = meshes.begin()->second->tex_polygons;
				mesh.cloud = meshes.begin()->second->cloud;

				success = pcl::io::saveOBJFile(path.toStdString(), mesh) == 0;
				if(success)
				{
					_progressDialog->incrementStep();
					_progressDialog->appendText(tr("Saving the mesh (with %1 textures)... done.").arg(mesh.tex_materials.size()));

					QMessageBox::information(this, tr("Save successful!"), tr("Mesh saved to \"%1\"").arg(path));
				}
				else
				{
					QMessageBox::warning(this, tr("Save failed!"), tr("Failed to save to \"%1\"").arg(path));
				}
			}
			else
			{
				QMessageBox::warning(this, tr("Save failed!"), tr("No textures..."));
			}
		}
	}
	else if(meshes.size())
	{
		QString path = QFileDialog::getExistingDirectory(this, tr("Save texture meshes to (*.obj)..."), workingDirectory, 0);
		if(!path.isEmpty())
		{
			bool ok = false;
			QString prefix = QInputDialog::getText(this, tr("File prefix"), tr("Prefix:"), QLineEdit::Normal, "mesh", &ok);
			QString suffix = "obj";

			if(ok)
			{
				for(std::map<int, pcl::TextureMesh::Ptr>::const_iterator iter=meshes.begin(); iter!=meshes.end(); ++iter)
				{
					QString currentPrefix=prefix+QString::number(iter->first);
					if(iter->second->tex_materials.size())
					{
						pcl::TextureMesh mesh;
						mesh.tex_coordinates = iter->second->tex_coordinates;
						mesh.tex_materials = iter->second->tex_materials;
						QDir(path).rmdir(currentPrefix);
						QDir(path).mkdir(currentPrefix);
						for(unsigned int i=0;i<iter->second->tex_materials.size(); ++i)
						{
							QFileInfo info(mesh.tex_materials[i].tex_file.c_str());
							QString fullPath = path+QDir::separator()+currentPrefix+QDir::separator()+info.fileName();
							// relative path
							mesh.tex_materials[i].tex_file=(currentPrefix+QDir::separator()+info.fileName()).toStdString();
							if(!QFile::copy(iter->second->tex_materials[i].tex_file.c_str(), fullPath) &&
									info.fileName().compare("occluded.png") != 0)
							{
								_progressDialog->appendText(tr("Failed copying texture \"%1\" to \"%2\".")
										.arg(iter->second->tex_materials[i].tex_file.c_str()).arg(fullPath), Qt::darkRed);
								_progressDialog->setAutoClose(false);
							}
						}
						mesh.tex_polygons = iter->second->tex_polygons;
						pcl::PointCloud<pcl::PointNormal>::Ptr tmp(new pcl::PointCloud<pcl::PointNormal>);
						pcl::fromPCLPointCloud2(iter->second->cloud, *tmp);
						tmp = util3d::transformPointCloud(tmp, poses.at(iter->first));
						pcl::toPCLPointCloud2(*tmp, mesh.cloud);

						QString pathFile = path+QDir::separator()+QString("%1.%3").arg(currentPrefix).arg(suffix);
						bool success =false;
						if(suffix == "obj")
						{
							success = pcl::io::saveOBJFile(pathFile.toStdString(), mesh) == 0;
						}
						else
						{
							UFATAL("Extension not recognized! (%s)", suffix.toStdString().c_str());
						}
						if(success)
						{
							_progressDialog->appendText(tr("Saved mesh %1 (%2 textures) to %3.")
									.arg(iter->first).arg(iter->second->tex_materials.size()-1).arg(pathFile));
						}
						else
						{
							_progressDialog->appendText(tr("Failed saving mesh %1 (%2 textures) to %3.")
									.arg(iter->first).arg(iter->second->tex_materials.size()-1).arg(pathFile), Qt::darkRed);
							_progressDialog->setAutoClose(false);
						}
					}
					else
					{
						_progressDialog->appendText(tr("Mesh %1 is empty!").arg(iter->first));
					}
					_progressDialog->incrementStep();
					QApplication::processEvents();
					if(_canceled)
					{
						return;
					}
				}
			}
		}
	}
}

}
