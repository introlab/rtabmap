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

#include "rtabmap/gui/ExportCloudsDialog.h"
#include "ui_exportCloudsDialog.h"

#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/gui/TexturingState.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UMath.h"

#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/GainCompensator.h"
#include "rtabmap/core/clams/discrete_depth_distortion_model.h"
#include "rtabmap/core/DBDriver.h"
#include "rtabmap/core/Version.h"

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/pcl_config.h>
#include <pcl/surface/poisson.h>
#include <pcl/common/common.h>

#include <QPushButton>
#include <QDir>
#include <QFileInfo>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QWindow>
#include <QScreen>

#ifdef RTABMAP_CPUTSDF
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#endif

#ifdef RTABMAP_OPENCHISEL
#include "chisel_conversions.h"
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>
#endif

#ifdef RTABMAP_PDAL
#include <rtabmap/core/PDALWriter.h>
#endif

namespace rtabmap {

ExportCloudsDialog::ExportCloudsDialog(QWidget *parent) :
	QDialog(parent),
	_canceled(false),
	_compensator(0),
	_dbDriver(0),
	_scansHaveRGB(false)
{
	_ui = new Ui_ExportCloudsDialog();
	_ui->setupUi(this);

	connect(_ui->buttonBox->button(QDialogButtonBox::RestoreDefaults), SIGNAL(clicked()), this, SLOT(restoreDefaults()));
	QPushButton * loadSettingsButton = _ui->buttonBox->addButton("Load Settings", QDialogButtonBox::ActionRole);
	QPushButton * saveSettingsButton = _ui->buttonBox->addButton("Save Settings", QDialogButtonBox::ActionRole);
	connect(loadSettingsButton, SIGNAL(clicked()), this, SLOT(loadSettings()));
	connect(saveSettingsButton, SIGNAL(clicked()), this, SLOT(saveSettings()));

	restoreDefaults();
	_ui->comboBox_upsamplingMethod->setItemData(1, 0, Qt::UserRole - 1); // disable DISTINCT_CLOUD

	connect(_ui->checkBox_fromDepth, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_fromDepth, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->checkBox_binary, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_normalKSearch, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_normalRadiusSearch, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_groundNormalsUp, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_pipeline, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_pipeline, SIGNAL(currentIndexChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->comboBox_meshingApproach, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_meshingApproach, SIGNAL(currentIndexChanged(int)), this, SLOT(updateReconstructionFlavor()));

	connect(_ui->checkBox_nodes_filtering, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_nodes_filtering, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->doubleSpinBox_nodes_filtering_xmin, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_nodes_filtering_xmax, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_nodes_filtering_ymin, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_nodes_filtering_ymax, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_nodes_filtering_zmin, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_nodes_filtering_zmax, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

	connect(_ui->checkBox_regenerate, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_regenerate, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->spinBox_decimation, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_maxDepth, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_minDepth, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_ceilingHeight, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_floorHeight, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_footprintWidth, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_footprintLength, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_footprintHeight, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_decimation_scan, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_rangeMin, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_rangeMax, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_fillDepthHoles, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_fillDepthHolesError, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->lineEdit_roiRatios, SIGNAL(textChanged(const QString &)), this, SIGNAL(configChanged()));
	connect(_ui->lineEdit_distortionModel, SIGNAL(textChanged(const QString &)), this, SIGNAL(configChanged()));
	connect(_ui->toolButton_distortionModel, SIGNAL(clicked()), this, SLOT(selectDistortionModel()));

	connect(_ui->checkBox_bilateral, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_bilateral, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->doubleSpinBox_bilateral_sigmaS, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_bilateral_sigmaR, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

	connect(_ui->checkBox_filtering, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_filtering, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->doubleSpinBox_filteringRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_filteringMinNeighbors, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->checkBox_assemble, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_assemble, SIGNAL(clicked(bool)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->doubleSpinBox_voxelSize_assembled, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_randomSamples_assembled, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_frame, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_frame, SIGNAL(currentIndexChanged(int)), this, SLOT(updateReconstructionFlavor()));

	connect(_ui->checkBox_subtraction, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_subtraction, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->doubleSpinBox_subtractPointFilteringRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_subtractPointFilteringAngle, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_subtractFilteringMinPts, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->checkBox_smoothing, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_smoothing, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->doubleSpinBox_mlsRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_polygonialOrder, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_upsamplingMethod, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_sampleStep, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_randomPoints, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_dilationVoxelSize, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_dilationSteps, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_mls_outputVoxelSize, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	_ui->stackedWidget_upsampling->setCurrentIndex(_ui->comboBox_upsamplingMethod->currentIndex());
	connect(_ui->comboBox_upsamplingMethod, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_upsampling, SLOT(setCurrentIndex(int)));
	connect(_ui->comboBox_upsamplingMethod, SIGNAL(currentIndexChanged(int)), this, SLOT(updateMLSGrpVisibility()));

	connect(_ui->checkBox_gainCompensation, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_gainCompensation, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->doubleSpinBox_gainRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gainOverlap, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gainBeta, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_gainRGB, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_gainFull, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_textureBrightnessContrastRatioLow, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_textureBrightnessContrastRatioHigh, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_exposureFusion, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_blending, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_blendingDecimation, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->checkBox_cameraProjection, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_cameraProjection, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->lineEdit_camProjRoiRatios, SIGNAL(textChanged(const QString &)), this, SIGNAL(configChanged()));
	connect(_ui->toolButton_camProjMaskFilePath, SIGNAL(clicked()), this, SLOT(selectCamProjMask()));
	connect(_ui->lineEdit_camProjMaskFilePath, SIGNAL(textChanged(const QString &)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_camProjDecimation, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_camProjMaxDistance, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_camProjMaxAngle, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_camProjDistanceToCamPolicy, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_camProjKeepPointsNotSeenByCameras, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_camProjRecolorPoints, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_camProjExportCamera, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
#ifndef RTABMAP_PDAL
	_ui->comboBox_camProjExportCamera->setEnabled(false);
	_ui->label_camProjExportCamera->setEnabled(false);
	_ui->label_camProjExportCamera->setText(_ui->label_camProjExportCamera->text() + " (PDAL dependency required)");
#endif

	connect(_ui->checkBox_meshing, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_meshing, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->doubleSpinBox_gp3Radius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_gp3Mu, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_meshDecimationFactor, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_meshDecimationFactor, SIGNAL(valueChanged(double)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->spinBox_meshMaxPolygons, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_meshMaxPolygons, SIGNAL(valueChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->doubleSpinBox_transferColorRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_cleanMesh, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_mesh_minClusterSize, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_textureMapping, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_textureMapping, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->comboBox_meshingTextureFormat, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_meshingTextureSize, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_mesh_maxTextures, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_meshingTextureMaxDistance, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_meshingTextureMaxDepthError, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_meshingTextureMaxAngle, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_mesh_minTextureClusterSize, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->lineEdit_meshingTextureRoiRatios, SIGNAL(textChanged(const QString &)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_cameraFilter, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_cameraFilter, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->doubleSpinBox_cameraFilterRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_cameraFilterAngle, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_cameraFilterVel, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_cameraFilterVelRad, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_laplacianVariance, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_multiband, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_multiband, SIGNAL(stateChanged(int)), this, SLOT(updateReconstructionFlavor()));
	connect(_ui->spinBox_multiband_downscale, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->lineEdit_multiband_nbcontrib, SIGNAL(textChanged(const QString &)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_multiband_unwrap, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_multiband_fillholes, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_multiband_padding, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_multiband_bestscore, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_multiband_angle, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_multiband_forcevisible, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->checkBox_poisson_outputPolygons, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_poisson_manifold, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_poisson_depth, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_poisson_targetPolygonSize, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_poisson_iso, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_poisson_solver, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_poisson_minDepth, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_poisson_samples, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_poisson_pointWeight, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_poisson_scale, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

	connect(_ui->doubleSpinBox_cputsdf_size, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_cputsdf_resolution, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_cputsdf_tuncPos, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_cputsdf_tuncNeg, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_cputsdf_minWeight, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_cputsdf_flattenRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_cputsdf_randomSplit, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->checkBox_openchisel_mergeVertices, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_openchisel_chunk_size_x, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_openchisel_chunk_size_y, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_openchisel_chunk_size_z, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_openchisel_truncation_constant, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_openchisel_truncation_linear, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_openchisel_truncation_quadratic, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_openchisel_truncation_scale, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_openchisel_integration_weight, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_openchisel_use_voxel_carving, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_openchisel_carving_dist_m, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_openchisel_near_plane_dist, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_openchisel_far_plane_dist, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));


	_progressDialog = new ProgressDialog(this);
	_progressDialog->setVisible(false);
	_progressDialog->setAutoClose(true, 2);
	_progressDialog->setMinimumWidth(600);
	_progressDialog->setCancelButtonVisible(true);
	connect(_progressDialog, SIGNAL(canceled()), this, SLOT(cancel()));

#ifdef DISABLE_VTK
	_ui->doubleSpinBox_meshDecimationFactor->setEnabled(false);
	_ui->spinBox_meshMaxPolygons->setEnabled(false);
	_ui->label_meshDecimation->setEnabled(false);
	_ui->label_meshMaxPolygons->setEnabled(false);
#endif

#if CV_MAJOR_VERSION < 3
	_ui->checkBox_exposureFusion->setEnabled(false);
	_ui->checkBox_exposureFusion->setChecked(false);
	_ui->label_exposureFusion->setEnabled(false);
#endif
}

ExportCloudsDialog::~ExportCloudsDialog()
{
	delete _ui;
	delete _compensator;
}

void ExportCloudsDialog::updateMLSGrpVisibility()
{
	_ui->groupBox->setVisible(_ui->comboBox_upsamplingMethod->currentIndex() == 0);
	_ui->groupBox_2->setVisible(_ui->comboBox_upsamplingMethod->currentIndex() == 1);
	_ui->groupBox_3->setVisible(_ui->comboBox_upsamplingMethod->currentIndex() == 2);
	_ui->groupBox_4->setVisible(_ui->comboBox_upsamplingMethod->currentIndex() == 3);
	_ui->groupBox_5->setVisible(_ui->comboBox_upsamplingMethod->currentIndex() == 4);
}

void ExportCloudsDialog::cancel()
{
	_canceled = true;
	_progressDialog->appendText(tr("Canceled!"));
}

void ExportCloudsDialog::forceAssembling(bool enabled)
{
	if(enabled)
	{
		_ui->checkBox_assemble->setChecked(true);
		_ui->checkBox_assemble->setEnabled(false);
	}
	else
	{
		_ui->checkBox_assemble->setEnabled(true);
	}
}

void ExportCloudsDialog::setProgressDialogToMax()
{
	_progressDialog->setValue(_progressDialog->maximumSteps());
}

void ExportCloudsDialog::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	settings.setValue("pipeline", _ui->comboBox_pipeline->currentIndex());
	settings.setValue("from_depth", _ui->checkBox_fromDepth->isChecked());
	settings.setValue("binary", _ui->checkBox_binary->isChecked());
	settings.setValue("normals_k", _ui->spinBox_normalKSearch->value());
	settings.setValue("normals_radius", _ui->doubleSpinBox_normalRadiusSearch->value());
	settings.setValue("normals_ground_normals_up", _ui->doubleSpinBox_groundNormalsUp->value());
	settings.setValue("intensity_colormap", _ui->comboBox_intensityColormap->currentIndex());

	settings.setValue("nodes_filtering", _ui->checkBox_nodes_filtering->isChecked());
	settings.setValue("nodes_filtering_xmin", _ui->doubleSpinBox_nodes_filtering_xmin->value());
	settings.setValue("nodes_filtering_xmax", _ui->doubleSpinBox_nodes_filtering_xmax->value());
	settings.setValue("nodes_filtering_ymin", _ui->doubleSpinBox_nodes_filtering_ymin->value());
	settings.setValue("nodes_filtering_ymax", _ui->doubleSpinBox_nodes_filtering_ymax->value());
	settings.setValue("nodes_filtering_zmin", _ui->doubleSpinBox_nodes_filtering_zmin->value());
	settings.setValue("nodes_filtering_zmax", _ui->doubleSpinBox_nodes_filtering_zmax->value());

	settings.setValue("regenerate", _ui->checkBox_regenerate->isChecked());
	settings.setValue("regenerate_decimation", _ui->spinBox_decimation->value());
	settings.setValue("regenerate_max_depth", _ui->doubleSpinBox_maxDepth->value());
	settings.setValue("regenerate_min_depth", _ui->doubleSpinBox_minDepth->value());
	settings.setValue("regenerate_ceiling", _ui->doubleSpinBox_ceilingHeight->value());
	settings.setValue("regenerate_floor", _ui->doubleSpinBox_floorHeight->value());
	settings.setValue("regenerate_footprint_height", _ui->doubleSpinBox_footprintHeight->value());
	settings.setValue("regenerate_footprint_width", _ui->doubleSpinBox_footprintWidth->value());
	settings.setValue("regenerate_footprint_length", _ui->doubleSpinBox_footprintLength->value());
	settings.setValue("regenerate_scan_decimation", _ui->spinBox_decimation_scan->value());
	settings.setValue("regenerate_scan_max_range", _ui->doubleSpinBox_rangeMax->value());
	settings.setValue("regenerate_scan_min_range", _ui->doubleSpinBox_rangeMin->value());
	settings.setValue("regenerate_fill_size", _ui->spinBox_fillDepthHoles->value());
	settings.setValue("regenerate_fill_error", _ui->spinBox_fillDepthHolesError->value());
	settings.setValue("regenerate_roi", _ui->lineEdit_roiRatios->text());
	settings.setValue("regenerate_distortion_model", _ui->lineEdit_distortionModel->text());

	settings.setValue("bilateral", _ui->checkBox_bilateral->isChecked());
	settings.setValue("bilateral_sigma_s", _ui->doubleSpinBox_bilateral_sigmaS->value());
	settings.setValue("bilateral_sigma_r", _ui->doubleSpinBox_bilateral_sigmaR->value());

	settings.setValue("filtering", _ui->checkBox_filtering->isChecked());
	settings.setValue("filtering_radius", _ui->doubleSpinBox_filteringRadius->value());
	settings.setValue("filtering_min_neighbors", _ui->spinBox_filteringMinNeighbors->value());

	settings.setValue("assemble", _ui->checkBox_assemble->isChecked());
	settings.setValue("assemble_voxel",_ui->doubleSpinBox_voxelSize_assembled->value());
	settings.setValue("assemble_samples",_ui->spinBox_randomSamples_assembled->value());
	settings.setValue("frame",_ui->comboBox_frame->currentIndex());

	settings.setValue("subtract",_ui->checkBox_subtraction->isChecked());
	settings.setValue("subtract_point_radius",_ui->doubleSpinBox_subtractPointFilteringRadius->value());
	settings.setValue("subtract_point_angle",_ui->doubleSpinBox_subtractPointFilteringAngle->value());
	settings.setValue("subtract_min_neighbors",_ui->spinBox_subtractFilteringMinPts->value());

	settings.setValue("mls", _ui->checkBox_smoothing->isChecked());
	settings.setValue("mls_radius", _ui->doubleSpinBox_mlsRadius->value());
	settings.setValue("mls_polygonial_order", _ui->spinBox_polygonialOrder->value());
	settings.setValue("mls_upsampling_method", _ui->comboBox_upsamplingMethod->currentIndex());
	settings.setValue("mls_upsampling_radius", _ui->doubleSpinBox_sampleRadius->value());
	settings.setValue("mls_upsampling_step", _ui->doubleSpinBox_sampleStep->value());
	settings.setValue("mls_point_density", _ui->spinBox_randomPoints->value());
	settings.setValue("mls_dilation_voxel_size", _ui->doubleSpinBox_dilationVoxelSize->value());
	settings.setValue("mls_dilation_iterations", _ui->spinBox_dilationSteps->value());
	settings.setValue("mls_output_voxel_size", _ui->doubleSpinBox_mls_outputVoxelSize->value());

	settings.setValue("gain", _ui->checkBox_gainCompensation->isChecked());
	settings.setValue("gain_radius", _ui->doubleSpinBox_gainRadius->value());
	settings.setValue("gain_overlap", _ui->doubleSpinBox_gainOverlap->value());
	settings.setValue("gain_beta", _ui->doubleSpinBox_gainBeta->value());
	settings.setValue("gain_rgb", _ui->checkBox_gainRGB->isChecked());
	settings.setValue("gain_full", _ui->checkBox_gainFull->isChecked());

	settings.setValue("cam_proj", _ui->checkBox_cameraProjection->isChecked());
	settings.setValue("cam_proj_roi_ratios", _ui->lineEdit_camProjRoiRatios->text());
	settings.setValue("cam_proj_mask", _ui->lineEdit_camProjMaskFilePath->text());
	settings.setValue("cam_proj_decimation", _ui->spinBox_camProjDecimation->value());
	settings.setValue("cam_proj_max_distance", _ui->doubleSpinBox_camProjMaxDistance->value());
	settings.setValue("cam_proj_max_angle", _ui->doubleSpinBox_camProjMaxAngle->value());
	settings.setValue("cam_proj_distance_policy", _ui->checkBox_camProjDistanceToCamPolicy->isChecked());
	settings.setValue("cam_proj_keep_points", _ui->checkBox_camProjKeepPointsNotSeenByCameras->isChecked());
	settings.setValue("cam_proj_recolor_points", _ui->checkBox_camProjRecolorPoints->isChecked());
	settings.setValue("cam_proj_export_format", _ui->comboBox_camProjExportCamera->currentIndex());

	settings.setValue("mesh", _ui->checkBox_meshing->isChecked());
	settings.setValue("mesh_radius", _ui->doubleSpinBox_gp3Radius->value());
	settings.setValue("mesh_mu", _ui->doubleSpinBox_gp3Mu->value());
	settings.setValue("mesh_decimation_factor", _ui->doubleSpinBox_meshDecimationFactor->value());
	settings.setValue("mesh_max_polygons", _ui->spinBox_meshMaxPolygons->value());
	settings.setValue("mesh_color_radius", _ui->doubleSpinBox_transferColorRadius->value());
	settings.setValue("mesh_clean", _ui->checkBox_cleanMesh->isChecked());
	settings.setValue("mesh_min_cluster_size", _ui->spinBox_mesh_minClusterSize->value());

	settings.setValue("mesh_dense_strategy", _ui->comboBox_meshingApproach->currentIndex());

	settings.setValue("mesh_texture", _ui->checkBox_textureMapping->isChecked());
	settings.setValue("mesh_textureFormat", _ui->comboBox_meshingTextureFormat->currentIndex());
	settings.setValue("mesh_textureSize", _ui->comboBox_meshingTextureSize->currentIndex());
	settings.setValue("mesh_textureMaxCount", _ui->spinBox_mesh_maxTextures->value());
	settings.setValue("mesh_textureMaxDistance", _ui->doubleSpinBox_meshingTextureMaxDistance->value());
	settings.setValue("mesh_textureMaxDepthError", _ui->doubleSpinBox_meshingTextureMaxDepthError->value());
	settings.setValue("mesh_textureMaxAngle", _ui->doubleSpinBox_meshingTextureMaxAngle->value());
	settings.setValue("mesh_textureMinCluster", _ui->spinBox_mesh_minTextureClusterSize->value());
	settings.setValue("mesh_textureRoiRatios", _ui->lineEdit_meshingTextureRoiRatios->text());
	settings.setValue("mesh_textureDistanceToCamPolicy", _ui->checkBox_distanceToCamPolicy->isChecked());
	settings.setValue("mesh_textureCameraFiltering", _ui->checkBox_cameraFilter->isChecked());
	settings.setValue("mesh_textureCameraFilteringRadius", _ui->doubleSpinBox_cameraFilterRadius->value());
	settings.setValue("mesh_textureCameraFilteringAngle", _ui->doubleSpinBox_cameraFilterAngle->value());
	settings.setValue("mesh_textureCameraFilteringVel", _ui->doubleSpinBox_cameraFilterVel->value());
	settings.setValue("mesh_textureCameraFilteringVelRad", _ui->doubleSpinBox_cameraFilterVelRad->value());
	settings.setValue("mesh_textureCameraFilteringLaplacian", _ui->doubleSpinBox_laplacianVariance->value());
	settings.setValue("mesh_textureBrightnessConstrastRatioLow", _ui->spinBox_textureBrightnessContrastRatioLow->value());
	settings.setValue("mesh_textureBrightnessConstrastRatioHigh", _ui->spinBox_textureBrightnessContrastRatioHigh->value());
	settings.setValue("mesh_textureExposureFusion", _ui->checkBox_exposureFusion->isChecked());
	settings.setValue("mesh_textureBlending", _ui->checkBox_blending->isChecked());
	settings.setValue("mesh_textureBlendingDecimation", _ui->comboBox_blendingDecimation->currentIndex());
	settings.setValue("mesh_textureMultiband", _ui->checkBox_multiband->isChecked());
	settings.setValue("mesh_textureMultibandDownScale", _ui->spinBox_multiband_downscale->value());
	settings.setValue("mesh_textureMultibandNbContrib", _ui->lineEdit_multiband_nbcontrib->text());
	settings.setValue("mesh_textureMultibandUnwrap", _ui->comboBox_multiband_unwrap->currentIndex());
	settings.setValue("mesh_textureMultibandFillHoles", _ui->checkBox_multiband_fillholes->isChecked());
	settings.setValue("mesh_textureMultibandPadding", _ui->spinBox_multiband_padding->value());
	settings.setValue("mesh_textureMultibandBestScoreThr", _ui->doubleSpinBox_multiband_bestscore->value());
	settings.setValue("mesh_textureMultibandAngleHardThr", _ui->doubleSpinBox_multiband_angle->value());
	settings.setValue("mesh_textureMultibandForceVisible", _ui->checkBox_multiband_forcevisible->isChecked());


	settings.setValue("mesh_angle_tolerance", _ui->doubleSpinBox_mesh_angleTolerance->value());
	settings.setValue("mesh_quad", _ui->checkBox_mesh_quad->isChecked());
	settings.setValue("mesh_triangle_size", _ui->spinBox_mesh_triangleSize->value());

	settings.setValue("poisson_outputPolygons", _ui->checkBox_poisson_outputPolygons->isChecked());
	settings.setValue("poisson_manifold", _ui->checkBox_poisson_manifold->isChecked());
	settings.setValue("poisson_depth", _ui->spinBox_poisson_depth->value());
	settings.setValue("poisson_polygon_size", _ui->doubleSpinBox_poisson_targetPolygonSize->value());
	settings.setValue("poisson_iso", _ui->spinBox_poisson_iso->value());
	settings.setValue("poisson_solver", _ui->spinBox_poisson_solver->value());
	settings.setValue("poisson_minDepth", _ui->spinBox_poisson_minDepth->value());
	settings.setValue("poisson_samples", _ui->doubleSpinBox_poisson_samples->value());
	settings.setValue("poisson_pointWeight", _ui->doubleSpinBox_poisson_pointWeight->value());
	settings.setValue("poisson_scale", _ui->doubleSpinBox_poisson_scale->value());

	settings.setValue("cputsdf_size", _ui->doubleSpinBox_cputsdf_size->value());
	settings.setValue("cputsdf_resolution", _ui->doubleSpinBox_cputsdf_resolution->value());
	settings.setValue("cputsdf_truncPos", _ui->doubleSpinBox_cputsdf_tuncPos->value());
	settings.setValue("cputsdf_truncNeg", _ui->doubleSpinBox_cputsdf_tuncNeg->value());
	settings.setValue("cputsdf_minWeight", _ui->doubleSpinBox_cputsdf_minWeight->value());
	settings.setValue("cputsdf_flattenRadius", _ui->doubleSpinBox_cputsdf_flattenRadius->value());
	settings.setValue("cputsdf_randomSplit", _ui->spinBox_cputsdf_randomSplit->value());

	settings.setValue("openchisel_merge_vertices", _ui->checkBox_openchisel_mergeVertices->isChecked());
	settings.setValue("openchisel_chunk_size_x", _ui->spinBox_openchisel_chunk_size_x->value());
	settings.setValue("openchisel_chunk_size_y", _ui->spinBox_openchisel_chunk_size_y->value());
	settings.setValue("openchisel_chunk_size_z", _ui->spinBox_openchisel_chunk_size_z->value());
	settings.setValue("openchisel_truncation_constant", _ui->doubleSpinBox_openchisel_truncation_constant->value());
	settings.setValue("openchisel_truncation_linear", _ui->doubleSpinBox_openchisel_truncation_linear->value());
	settings.setValue("openchisel_truncation_quadratic", _ui->doubleSpinBox_openchisel_truncation_quadratic->value());
	settings.setValue("openchisel_truncation_scale", _ui->doubleSpinBox_openchisel_truncation_scale->value());
	settings.setValue("openchisel_integration_weight", _ui->spinBox_openchisel_integration_weight->value());
	settings.setValue("openchisel_use_voxel_carving", _ui->checkBox_openchisel_use_voxel_carving->isChecked());
	settings.setValue("openchisel_carving_dist_m", _ui->doubleSpinBox_openchisel_carving_dist_m->value());
	settings.setValue("openchisel_near_plane_dist", _ui->doubleSpinBox_openchisel_near_plane_dist->value());
	settings.setValue("openchisel_far_plane_dist", _ui->doubleSpinBox_openchisel_far_plane_dist->value());

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
	_ui->checkBox_fromDepth->setChecked(settings.value("from_depth", _ui->checkBox_fromDepth->isChecked()).toBool());
	_ui->checkBox_binary->setChecked(settings.value("binary", _ui->checkBox_binary->isChecked()).toBool());
	_ui->spinBox_normalKSearch->setValue(settings.value("normals_k", _ui->spinBox_normalKSearch->value()).toInt());
	_ui->doubleSpinBox_normalRadiusSearch->setValue(settings.value("normals_radius", _ui->doubleSpinBox_normalRadiusSearch->value()).toDouble());
	_ui->doubleSpinBox_groundNormalsUp->setValue(settings.value("normals_ground_normals_up", _ui->doubleSpinBox_groundNormalsUp->value()).toDouble());
	_ui->comboBox_intensityColormap->setCurrentIndex(settings.value("intensity_colormap", _ui->comboBox_intensityColormap->currentIndex()).toInt());

	_ui->checkBox_nodes_filtering->setChecked(settings.value("nodes_filtering", _ui->checkBox_nodes_filtering->isChecked()).toBool());
	_ui->doubleSpinBox_nodes_filtering_xmin->setValue(settings.value("nodes_filtering_xmin", _ui->doubleSpinBox_nodes_filtering_xmin->value()).toInt());
	_ui->doubleSpinBox_nodes_filtering_xmax->setValue(settings.value("nodes_filtering_xmax", _ui->doubleSpinBox_nodes_filtering_xmax->value()).toInt());
	_ui->doubleSpinBox_nodes_filtering_ymin->setValue(settings.value("nodes_filtering_ymin", _ui->doubleSpinBox_nodes_filtering_ymin->value()).toInt());
	_ui->doubleSpinBox_nodes_filtering_ymax->setValue(settings.value("nodes_filtering_ymax", _ui->doubleSpinBox_nodes_filtering_ymax->value()).toInt());
	_ui->doubleSpinBox_nodes_filtering_zmin->setValue(settings.value("nodes_filtering_zmin", _ui->doubleSpinBox_nodes_filtering_zmin->value()).toInt());
	_ui->doubleSpinBox_nodes_filtering_zmax->setValue(settings.value("nodes_filtering_zmax", _ui->doubleSpinBox_nodes_filtering_zmax->value()).toInt());

	_ui->checkBox_regenerate->setChecked(settings.value("regenerate", _ui->checkBox_regenerate->isChecked()).toBool());
	_ui->spinBox_decimation->setValue(settings.value("regenerate_decimation", _ui->spinBox_decimation->value()).toInt());
	_ui->doubleSpinBox_maxDepth->setValue(settings.value("regenerate_max_depth", _ui->doubleSpinBox_maxDepth->value()).toDouble());
	_ui->doubleSpinBox_minDepth->setValue(settings.value("regenerate_min_depth", _ui->doubleSpinBox_minDepth->value()).toDouble());
	_ui->doubleSpinBox_ceilingHeight->setValue(settings.value("regenerate_ceiling", _ui->doubleSpinBox_ceilingHeight->value()).toDouble());
	_ui->doubleSpinBox_floorHeight->setValue(settings.value("regenerate_floor", _ui->doubleSpinBox_floorHeight->value()).toDouble());
	_ui->doubleSpinBox_footprintHeight->setValue(settings.value("regenerate_footprint_height", _ui->doubleSpinBox_footprintHeight->value()).toDouble());
	_ui->doubleSpinBox_footprintWidth->setValue(settings.value("regenerate_footprint_width", _ui->doubleSpinBox_footprintWidth->value()).toDouble());
	_ui->doubleSpinBox_footprintLength->setValue(settings.value("regenerate_footprint_length", _ui->doubleSpinBox_footprintLength->value()).toDouble());
	_ui->spinBox_decimation_scan->setValue(settings.value("regenerate_scan_decimation", _ui->spinBox_decimation_scan->value()).toInt());
	_ui->doubleSpinBox_rangeMax->setValue(settings.value("regenerate_scan_max_range", _ui->doubleSpinBox_rangeMax->value()).toDouble());
	_ui->doubleSpinBox_rangeMin->setValue(settings.value("regenerate_scan_min_range", _ui->doubleSpinBox_rangeMin->value()).toDouble());
	_ui->spinBox_fillDepthHoles->setValue(settings.value("regenerate_fill_size", _ui->spinBox_fillDepthHoles->value()).toInt());
	_ui->spinBox_fillDepthHolesError->setValue(settings.value("regenerate_fill_error", _ui->spinBox_fillDepthHolesError->value()).toInt());
	_ui->lineEdit_roiRatios->setText(settings.value("regenerate_roi", _ui->lineEdit_roiRatios->text()).toString());
	_ui->lineEdit_distortionModel->setText(settings.value("regenerate_distortion_model", _ui->lineEdit_distortionModel->text()).toString());

	_ui->checkBox_bilateral->setChecked(settings.value("bilateral", _ui->checkBox_bilateral->isChecked()).toBool());
	_ui->doubleSpinBox_bilateral_sigmaS->setValue(settings.value("bilateral_sigma_s", _ui->doubleSpinBox_bilateral_sigmaS->value()).toDouble());
	_ui->doubleSpinBox_bilateral_sigmaR->setValue(settings.value("bilateral_sigma_r", _ui->doubleSpinBox_bilateral_sigmaR->value()).toDouble());

	_ui->checkBox_filtering->setChecked(settings.value("filtering", _ui->checkBox_filtering->isChecked()).toBool());
	_ui->doubleSpinBox_filteringRadius->setValue(settings.value("filtering_radius", _ui->doubleSpinBox_filteringRadius->value()).toDouble());
	_ui->spinBox_filteringMinNeighbors->setValue(settings.value("filtering_min_neighbors", _ui->spinBox_filteringMinNeighbors->value()).toInt());

	if(_ui->checkBox_assemble->isEnabled())
	{
		_ui->checkBox_assemble->setChecked(settings.value("assemble", _ui->checkBox_assemble->isChecked()).toBool());
	}
	_ui->doubleSpinBox_voxelSize_assembled->setValue(settings.value("assemble_voxel", _ui->doubleSpinBox_voxelSize_assembled->value()).toDouble());
	_ui->spinBox_randomSamples_assembled->setValue(settings.value("assemble_samples", _ui->spinBox_randomSamples_assembled->value()).toInt());
	_ui->comboBox_frame->setCurrentIndex(settings.value("frame", _ui->comboBox_frame->currentIndex()).toInt());

	_ui->checkBox_subtraction->setChecked(settings.value("subtract",_ui->checkBox_subtraction->isChecked()).toBool());
	_ui->doubleSpinBox_subtractPointFilteringRadius->setValue(settings.value("subtract_point_radius",_ui->doubleSpinBox_subtractPointFilteringRadius->value()).toDouble());
	_ui->doubleSpinBox_subtractPointFilteringAngle->setValue(settings.value("subtract_point_angle",_ui->doubleSpinBox_subtractPointFilteringAngle->value()).toDouble());
	_ui->spinBox_subtractFilteringMinPts->setValue(settings.value("subtract_min_neighbors",_ui->spinBox_subtractFilteringMinPts->value()).toInt());

	_ui->checkBox_smoothing->setChecked(settings.value("mls", _ui->checkBox_smoothing->isChecked()).toBool());
	_ui->doubleSpinBox_mlsRadius->setValue(settings.value("mls_radius", _ui->doubleSpinBox_mlsRadius->value()).toDouble());
	_ui->spinBox_polygonialOrder->setValue(settings.value("mls_polygonial_order", _ui->spinBox_polygonialOrder->value()).toInt());
	_ui->comboBox_upsamplingMethod->setCurrentIndex(settings.value("mls_upsampling_method", _ui->comboBox_upsamplingMethod->currentIndex()).toInt());
	_ui->doubleSpinBox_sampleRadius->setValue(settings.value("mls_upsampling_radius", _ui->doubleSpinBox_sampleRadius->value()).toDouble());
	_ui->doubleSpinBox_sampleStep->setValue(settings.value("mls_upsampling_step", _ui->doubleSpinBox_sampleStep->value()).toDouble());
	_ui->spinBox_randomPoints->setValue(settings.value("mls_point_density", _ui->spinBox_randomPoints->value()).toInt());
	_ui->doubleSpinBox_dilationVoxelSize->setValue(settings.value("mls_dilation_voxel_size", _ui->doubleSpinBox_dilationVoxelSize->value()).toDouble());
	_ui->spinBox_dilationSteps->setValue(settings.value("mls_dilation_iterations", _ui->spinBox_dilationSteps->value()).toInt());
	_ui->doubleSpinBox_mls_outputVoxelSize->setValue(settings.value("mls_output_voxel_size", _ui->doubleSpinBox_mls_outputVoxelSize->value()).toInt());

	_ui->checkBox_gainCompensation->setChecked(settings.value("gain", _ui->checkBox_gainCompensation->isChecked()).toBool());
	_ui->doubleSpinBox_gainRadius->setValue(settings.value("gain_radius", _ui->doubleSpinBox_gainRadius->value()).toDouble());
	_ui->doubleSpinBox_gainOverlap->setValue(settings.value("gain_overlap", _ui->doubleSpinBox_gainOverlap->value()).toDouble());
	_ui->doubleSpinBox_gainBeta->setValue(settings.value("gain_beta", _ui->doubleSpinBox_gainBeta->value()).toDouble());
	_ui->checkBox_gainRGB->setChecked(settings.value("gain_rgb", _ui->checkBox_gainRGB->isChecked()).toBool());
	_ui->checkBox_gainFull->setChecked(settings.value("gain_full", _ui->checkBox_gainFull->isChecked()).toBool());

	_ui->checkBox_cameraProjection->setChecked(settings.value("cam_proj", _ui->checkBox_cameraProjection->isChecked()).toBool());
	_ui->lineEdit_camProjRoiRatios->setText(settings.value("cam_proj_roi_ratios", _ui->lineEdit_camProjRoiRatios->text()).toString());
	_ui->lineEdit_camProjMaskFilePath->setText(settings.value("cam_proj_mask", _ui->lineEdit_camProjMaskFilePath->text()).toString());
	_ui->spinBox_camProjDecimation->setValue(settings.value("cam_proj_decimation", _ui->spinBox_camProjDecimation->value()).toInt());
	_ui->doubleSpinBox_camProjMaxDistance->setValue(settings.value("cam_proj_max_distance", _ui->doubleSpinBox_camProjMaxDistance->value()).toDouble());
	_ui->doubleSpinBox_camProjMaxAngle->setValue(settings.value("cam_proj_max_angle", _ui->doubleSpinBox_camProjMaxAngle->value()).toDouble());
	_ui->checkBox_camProjDistanceToCamPolicy->setChecked(settings.value("cam_proj_distance_policy", _ui->checkBox_camProjDistanceToCamPolicy->isChecked()).toBool());
	_ui->checkBox_camProjKeepPointsNotSeenByCameras->setChecked(settings.value("cam_proj_keep_points", _ui->checkBox_camProjKeepPointsNotSeenByCameras->isChecked()).toBool());
	_ui->checkBox_camProjRecolorPoints->setChecked(settings.value("cam_proj_recolor_points", _ui->checkBox_camProjRecolorPoints->isChecked()).toBool());
	_ui->comboBox_camProjExportCamera->setCurrentIndex(settings.value("cam_proj_export_format", _ui->comboBox_camProjExportCamera->currentIndex()).toInt());

	_ui->checkBox_meshing->setChecked(settings.value("mesh", _ui->checkBox_meshing->isChecked()).toBool());
	_ui->doubleSpinBox_gp3Radius->setValue(settings.value("mesh_radius", _ui->doubleSpinBox_gp3Radius->value()).toDouble());
	_ui->doubleSpinBox_gp3Mu->setValue(settings.value("mesh_mu", _ui->doubleSpinBox_gp3Mu->value()).toDouble());
	_ui->doubleSpinBox_meshDecimationFactor->setValue(settings.value("mesh_decimation_factor",_ui->doubleSpinBox_meshDecimationFactor->value()).toDouble());
	_ui->spinBox_meshMaxPolygons->setValue(settings.value("mesh_max_polygons",_ui->spinBox_meshMaxPolygons->value()).toDouble());
	_ui->doubleSpinBox_transferColorRadius->setValue(settings.value("mesh_color_radius",_ui->doubleSpinBox_transferColorRadius->value()).toDouble());
	_ui->checkBox_cleanMesh->setChecked(settings.value("mesh_clean",_ui->checkBox_cleanMesh->isChecked()).toBool());
	_ui->spinBox_mesh_minClusterSize->setValue(settings.value("mesh_min_cluster_size", _ui->spinBox_mesh_minClusterSize->value()).toInt());

	_ui->comboBox_meshingApproach->setCurrentIndex(settings.value("mesh_dense_strategy", _ui->comboBox_meshingApproach->currentIndex()).toInt());

	_ui->checkBox_textureMapping->setChecked(settings.value("mesh_texture", _ui->checkBox_textureMapping->isChecked()).toBool());
	_ui->comboBox_meshingTextureFormat->setCurrentIndex(settings.value("mesh_textureFormat", _ui->comboBox_meshingTextureFormat->currentIndex()).toInt());
	_ui->comboBox_meshingTextureSize->setCurrentIndex(settings.value("mesh_textureSize", _ui->comboBox_meshingTextureSize->currentIndex()).toInt());
	_ui->spinBox_mesh_maxTextures->setValue(settings.value("mesh_textureMaxCount", _ui->spinBox_mesh_maxTextures->value()).toInt());
	_ui->doubleSpinBox_meshingTextureMaxDistance->setValue(settings.value("mesh_textureMaxDistance", _ui->doubleSpinBox_meshingTextureMaxDistance->value()).toDouble());
	_ui->doubleSpinBox_meshingTextureMaxDepthError->setValue(settings.value("mesh_textureMaxDepthError", _ui->doubleSpinBox_meshingTextureMaxDepthError->value()).toDouble());
	_ui->doubleSpinBox_meshingTextureMaxAngle->setValue(settings.value("mesh_textureMaxAngle", _ui->doubleSpinBox_meshingTextureMaxAngle->value()).toDouble());
	_ui->spinBox_mesh_minTextureClusterSize->setValue(settings.value("mesh_textureMinCluster", _ui->spinBox_mesh_minTextureClusterSize->value()).toDouble());
	_ui->lineEdit_meshingTextureRoiRatios->setText(settings.value("mesh_textureRoiRatios", _ui->lineEdit_meshingTextureRoiRatios->text()).toString());
	_ui->checkBox_distanceToCamPolicy->setChecked(settings.value("mesh_textureDistanceToCamPolicy", _ui->checkBox_distanceToCamPolicy->isChecked()).toBool());
	_ui->checkBox_cameraFilter->setChecked(settings.value("mesh_textureCameraFiltering", _ui->checkBox_cameraFilter->isChecked()).toBool());
	_ui->doubleSpinBox_cameraFilterRadius->setValue(settings.value("mesh_textureCameraFilteringRadius", _ui->doubleSpinBox_cameraFilterRadius->value()).toDouble());
	_ui->doubleSpinBox_cameraFilterAngle->setValue(settings.value("mesh_textureCameraFilteringAngle", _ui->doubleSpinBox_cameraFilterAngle->value()).toDouble());
	_ui->doubleSpinBox_cameraFilterVel->setValue(settings.value("mesh_textureCameraFilteringVel", _ui->doubleSpinBox_cameraFilterVel->value()).toDouble());
	_ui->doubleSpinBox_cameraFilterVelRad->setValue(settings.value("mesh_textureCameraFilteringVelRad", _ui->doubleSpinBox_cameraFilterVelRad->value()).toDouble());
	_ui->doubleSpinBox_laplacianVariance->setValue(settings.value("mesh_textureCameraFilteringLaplacian", _ui->doubleSpinBox_laplacianVariance->value()).toDouble());
	_ui->spinBox_textureBrightnessContrastRatioLow->setValue(settings.value("mesh_textureBrightnessConstrastRatioLow", _ui->spinBox_textureBrightnessContrastRatioLow->value()).toDouble());
	_ui->spinBox_textureBrightnessContrastRatioHigh->setValue(settings.value("mesh_textureBrightnessConstrastRatioHigh", _ui->spinBox_textureBrightnessContrastRatioHigh->value()).toDouble());
	if(_ui->checkBox_exposureFusion->isEnabled())
	{
		_ui->checkBox_exposureFusion->setChecked(settings.value("mesh_textureExposureFusion", _ui->checkBox_exposureFusion->isChecked()).toBool());
	}
	_ui->checkBox_blending->setChecked(settings.value("mesh_textureBlending", _ui->checkBox_blending->isChecked()).toBool());
	_ui->comboBox_blendingDecimation->setCurrentIndex(settings.value("mesh_textureBlendingDecimation", _ui->comboBox_blendingDecimation->currentIndex()).toInt());
	_ui->checkBox_multiband->setChecked(settings.value("mesh_textureMultiband", _ui->checkBox_multiband->isChecked()).toBool());
	_ui->spinBox_multiband_downscale->setValue(settings.value("mesh_textureMultibandDownScale", _ui->spinBox_multiband_downscale->value()).toInt());
	_ui->lineEdit_multiband_nbcontrib->setText(settings.value("mesh_textureMultibandNbContrib", _ui->lineEdit_multiband_nbcontrib->text()).toString());
	_ui->comboBox_multiband_unwrap->setCurrentIndex(settings.value("mesh_textureMultibandUnwrap", _ui->comboBox_multiband_unwrap->currentIndex()).toInt());
	_ui->checkBox_multiband_fillholes->setChecked(settings.value("mesh_textureMultibandFillHoles", _ui->checkBox_multiband_fillholes->isChecked()).toBool());
	_ui->spinBox_multiband_padding->setValue(settings.value("mesh_textureMultibandPadding", _ui->spinBox_multiband_padding->value()).toInt());
	_ui->doubleSpinBox_multiband_bestscore->setValue(settings.value("mesh_textureMultibandBestScoreThr", _ui->doubleSpinBox_multiband_bestscore->value()).toDouble());
	_ui->doubleSpinBox_multiband_angle->setValue(settings.value("mesh_textureMultibandAngleHardThr", _ui->doubleSpinBox_multiband_angle->value()).toDouble());
	_ui->checkBox_multiband_forcevisible->setChecked(settings.value("mesh_textureMultibandForceVisible", _ui->checkBox_multiband_forcevisible->isChecked()).toBool());

	_ui->doubleSpinBox_mesh_angleTolerance->setValue(settings.value("mesh_angle_tolerance", _ui->doubleSpinBox_mesh_angleTolerance->value()).toDouble());
	_ui->checkBox_mesh_quad->setChecked(settings.value("mesh_quad", _ui->checkBox_mesh_quad->isChecked()).toBool());
	_ui->spinBox_mesh_triangleSize->setValue(settings.value("mesh_triangle_size", _ui->spinBox_mesh_triangleSize->value()).toInt());

	_ui->checkBox_poisson_outputPolygons->setChecked(settings.value("poisson_outputPolygons", _ui->checkBox_poisson_outputPolygons->isChecked()).toBool());
	_ui->checkBox_poisson_manifold->setChecked(settings.value("poisson_manifold", _ui->checkBox_poisson_manifold->isChecked()).toBool());
	_ui->spinBox_poisson_depth->setValue(settings.value("poisson_depth", _ui->spinBox_poisson_depth->value()).toInt());
	_ui->doubleSpinBox_poisson_targetPolygonSize->setValue(settings.value("poisson_polygon_size", _ui->doubleSpinBox_poisson_targetPolygonSize->value()).toDouble());
	_ui->spinBox_poisson_iso->setValue(settings.value("poisson_iso", _ui->spinBox_poisson_iso->value()).toInt());
	_ui->spinBox_poisson_solver->setValue(settings.value("poisson_solver", _ui->spinBox_poisson_solver->value()).toInt());
	_ui->spinBox_poisson_minDepth->setValue(settings.value("poisson_minDepth", _ui->spinBox_poisson_minDepth->value()).toInt());
	_ui->doubleSpinBox_poisson_samples->setValue(settings.value("poisson_samples", _ui->doubleSpinBox_poisson_samples->value()).toDouble());
	_ui->doubleSpinBox_poisson_pointWeight->setValue(settings.value("poisson_pointWeight", _ui->doubleSpinBox_poisson_pointWeight->value()).toDouble());
	_ui->doubleSpinBox_poisson_scale->setValue(settings.value("poisson_scale", _ui->doubleSpinBox_poisson_scale->value()).toDouble());

	_ui->doubleSpinBox_cputsdf_size->setValue(settings.value("cputsdf_size", _ui->doubleSpinBox_cputsdf_size->value()).toDouble());
	_ui->doubleSpinBox_cputsdf_resolution->setValue(settings.value("cputsdf_resolution", _ui->doubleSpinBox_cputsdf_resolution->value()).toDouble());
	_ui->doubleSpinBox_cputsdf_tuncPos->setValue(settings.value("cputsdf_truncPos", _ui->doubleSpinBox_cputsdf_tuncPos->value()).toDouble());
	_ui->doubleSpinBox_cputsdf_tuncNeg->setValue(settings.value("cputsdf_truncNeg", _ui->doubleSpinBox_cputsdf_tuncNeg->value()).toDouble());
	_ui->doubleSpinBox_cputsdf_minWeight->setValue(settings.value("cputsdf_minWeight", _ui->doubleSpinBox_cputsdf_minWeight->value()).toDouble());
	_ui->doubleSpinBox_cputsdf_flattenRadius->setValue(settings.value("cputsdf_flattenRadius", _ui->doubleSpinBox_cputsdf_flattenRadius->value()).toDouble());
	_ui->spinBox_cputsdf_randomSplit->setValue(settings.value("cputsdf_randomSplit", _ui->spinBox_cputsdf_randomSplit->value()).toInt());

	_ui->checkBox_openchisel_mergeVertices->setChecked(settings.value("openchisel_merge_vertices", _ui->checkBox_openchisel_mergeVertices->isChecked()).toBool());
	_ui->spinBox_openchisel_chunk_size_x->setValue(settings.value("openchisel_chunk_size_x", _ui->spinBox_openchisel_chunk_size_x->value()).toInt());
	_ui->spinBox_openchisel_chunk_size_y->setValue(settings.value("openchisel_chunk_size_y", _ui->spinBox_openchisel_chunk_size_y->value()).toInt());
	_ui->spinBox_openchisel_chunk_size_z->setValue(settings.value("openchisel_chunk_size_z", _ui->spinBox_openchisel_chunk_size_z->value()).toInt());
	_ui->doubleSpinBox_openchisel_truncation_constant->setValue(settings.value("openchisel_truncation_constant", _ui->doubleSpinBox_openchisel_truncation_constant->value()).toDouble());
	_ui->doubleSpinBox_openchisel_truncation_linear->setValue(settings.value("openchisel_truncation_linear", _ui->doubleSpinBox_openchisel_truncation_linear->value()).toDouble());
	_ui->doubleSpinBox_openchisel_truncation_quadratic->setValue(settings.value("openchisel_truncation_quadratic", _ui->doubleSpinBox_openchisel_truncation_quadratic->value()).toDouble());
	_ui->doubleSpinBox_openchisel_truncation_scale->setValue(settings.value("openchisel_truncation_scale", _ui->doubleSpinBox_openchisel_truncation_scale->value()).toDouble());
	_ui->spinBox_openchisel_integration_weight->setValue(settings.value("openchisel_integration_weight", _ui->spinBox_openchisel_integration_weight->value()).toInt());
	_ui->checkBox_openchisel_use_voxel_carving->setChecked(settings.value("openchisel_use_voxel_carving", _ui->checkBox_openchisel_use_voxel_carving->isChecked()).toBool());
	_ui->doubleSpinBox_openchisel_carving_dist_m->setValue(settings.value("openchisel_carving_dist_m", _ui->doubleSpinBox_openchisel_carving_dist_m->value()).toDouble());
	_ui->doubleSpinBox_openchisel_near_plane_dist->setValue(settings.value("openchisel_near_plane_dist", _ui->doubleSpinBox_openchisel_near_plane_dist->value()).toDouble());
	_ui->doubleSpinBox_openchisel_far_plane_dist->setValue(settings.value("openchisel_far_plane_dist", _ui->doubleSpinBox_openchisel_far_plane_dist->value()).toDouble());

	updateReconstructionFlavor();
	updateMLSGrpVisibility();

	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportCloudsDialog::restoreDefaults()
{
	_ui->comboBox_pipeline->setCurrentIndex(1);
	_ui->checkBox_fromDepth->setChecked(true);
	_ui->checkBox_binary->setChecked(true);
	_ui->spinBox_normalKSearch->setValue(20);
	_ui->doubleSpinBox_normalRadiusSearch->setValue(0.0);
	_ui->doubleSpinBox_groundNormalsUp->setValue(0.0);
	_ui->comboBox_intensityColormap->setCurrentIndex(0);

	_ui->checkBox_nodes_filtering->setChecked(false);
	_ui->doubleSpinBox_nodes_filtering_xmin->setValue(0);
	_ui->doubleSpinBox_nodes_filtering_xmax->setValue(0);
	_ui->doubleSpinBox_nodes_filtering_ymin->setValue(0);
	_ui->doubleSpinBox_nodes_filtering_ymax->setValue(0);
	_ui->doubleSpinBox_nodes_filtering_zmin->setValue(0);
	_ui->doubleSpinBox_nodes_filtering_zmax->setValue(0);

	_ui->checkBox_regenerate->setChecked(_dbDriver!=0?true:false);
	_ui->spinBox_decimation->setValue(1);
	_ui->doubleSpinBox_maxDepth->setValue(4);
	_ui->doubleSpinBox_minDepth->setValue(0);
	_ui->doubleSpinBox_ceilingHeight->setValue(0);
	_ui->doubleSpinBox_floorHeight->setValue(0);
	_ui->doubleSpinBox_footprintHeight->setValue(0);
	_ui->doubleSpinBox_footprintLength->setValue(0);
	_ui->doubleSpinBox_footprintWidth->setValue(0);
	_ui->spinBox_decimation_scan->setValue(1);
	_ui->doubleSpinBox_rangeMax->setValue(0);
	_ui->doubleSpinBox_rangeMin->setValue(0);
	_ui->spinBox_fillDepthHoles->setValue(0);
	_ui->spinBox_fillDepthHolesError->setValue(2);
	_ui->lineEdit_roiRatios->setText("0.0 0.0 0.0 0.0");
	_ui->lineEdit_distortionModel->setText("");

	_ui->checkBox_bilateral->setChecked(false);
	_ui->doubleSpinBox_bilateral_sigmaS->setValue(10.0);
	_ui->doubleSpinBox_bilateral_sigmaR->setValue(0.1);

	_ui->checkBox_filtering->setChecked(false);
	_ui->doubleSpinBox_filteringRadius->setValue(0.00);
	_ui->spinBox_filteringMinNeighbors->setValue(5);

	_ui->checkBox_assemble->setChecked(true);
	_ui->doubleSpinBox_voxelSize_assembled->setValue(0.01);
	_ui->spinBox_randomSamples_assembled->setValue(0);
	_ui->comboBox_frame->setCurrentIndex(0);

	_ui->checkBox_subtraction->setChecked(false);
	_ui->doubleSpinBox_subtractPointFilteringRadius->setValue(0.02);
	_ui->doubleSpinBox_subtractPointFilteringAngle->setValue(0);
	_ui->spinBox_subtractFilteringMinPts->setValue(5);

	_ui->checkBox_smoothing->setChecked(false);
	_ui->doubleSpinBox_mlsRadius->setValue(0.04);
	_ui->spinBox_polygonialOrder->setValue(2);
	_ui->comboBox_upsamplingMethod->setCurrentIndex(0);
	_ui->doubleSpinBox_sampleRadius->setValue(0.01);
	_ui->doubleSpinBox_sampleStep->setValue(0.005);
	_ui->spinBox_randomPoints->setValue(10);
	_ui->doubleSpinBox_dilationVoxelSize->setValue(0.005);
	_ui->spinBox_dilationSteps->setValue(1);
	_ui->doubleSpinBox_mls_outputVoxelSize->setValue(0);

	_ui->checkBox_gainCompensation->setChecked(false);
	_ui->doubleSpinBox_gainRadius->setValue(0.02);
	_ui->doubleSpinBox_gainOverlap->setValue(0.0);
	_ui->doubleSpinBox_gainBeta->setValue(10);
	_ui->checkBox_gainRGB->setChecked(true);
	_ui->checkBox_gainFull->setChecked(false);

	_ui->checkBox_cameraProjection->setChecked(false);
	_ui->lineEdit_camProjRoiRatios->setText("0.0 0.0 0.0 0.0");
	_ui->lineEdit_camProjMaskFilePath->setText("");
	_ui->spinBox_camProjDecimation->setValue(1);
	_ui->doubleSpinBox_camProjMaxDistance->setValue(0);
	_ui->doubleSpinBox_camProjMaxAngle->setValue(0);
	_ui->checkBox_camProjDistanceToCamPolicy->setChecked(true);
	_ui->checkBox_camProjKeepPointsNotSeenByCameras->setChecked(false);
	_ui->checkBox_camProjRecolorPoints->setChecked(true);
	_ui->comboBox_camProjExportCamera->setCurrentIndex(0);

	_ui->checkBox_meshing->setChecked(false);
	_ui->doubleSpinBox_gp3Radius->setValue(0.2);
	_ui->doubleSpinBox_gp3Mu->setValue(2.5);
	_ui->doubleSpinBox_meshDecimationFactor->setValue(0.0);
	_ui->spinBox_meshMaxPolygons->setValue(0);
	_ui->doubleSpinBox_transferColorRadius->setValue(0.025);
	_ui->checkBox_cleanMesh->setChecked(true);
	_ui->spinBox_mesh_minClusterSize->setValue(0);

	_ui->comboBox_meshingApproach->setCurrentIndex(1);

	_ui->checkBox_textureMapping->setChecked(false);
	_ui->comboBox_meshingTextureFormat->setCurrentIndex(0);
	_ui->comboBox_meshingTextureSize->setCurrentIndex(6); // 8192
	_ui->spinBox_mesh_maxTextures->setValue(1);
	_ui->doubleSpinBox_meshingTextureMaxDistance->setValue(3.0);
	_ui->doubleSpinBox_meshingTextureMaxDepthError->setValue(0.0);
	_ui->doubleSpinBox_meshingTextureMaxAngle->setValue(0.0);
	_ui->spinBox_mesh_minTextureClusterSize->setValue(50);
	_ui->lineEdit_meshingTextureRoiRatios->setText("0.0 0.0 0.0 0.0");
	_ui->checkBox_distanceToCamPolicy->setChecked(false);
	_ui->checkBox_cameraFilter->setChecked(false);
	_ui->doubleSpinBox_cameraFilterRadius->setValue(0);
	_ui->doubleSpinBox_cameraFilterAngle->setValue(30);
	_ui->doubleSpinBox_cameraFilterVel->setValue(0);
	_ui->doubleSpinBox_cameraFilterVelRad->setValue(0);
	_ui->doubleSpinBox_laplacianVariance->setValue(0);
	_ui->spinBox_textureBrightnessContrastRatioLow->setValue(0);
	_ui->spinBox_textureBrightnessContrastRatioHigh->setValue(5);
	_ui->checkBox_exposureFusion->setChecked(false);
	_ui->checkBox_blending->setChecked(true);
	_ui->comboBox_blendingDecimation->setCurrentIndex(0);
	_ui->checkBox_multiband->setChecked(false);
	_ui->spinBox_multiband_downscale->setValue(2);
	_ui->lineEdit_multiband_nbcontrib->setText("1 5 10 0");
	_ui->comboBox_multiband_unwrap->setCurrentIndex(0);
	_ui->checkBox_multiband_fillholes->setChecked(false);
	_ui->spinBox_multiband_padding->setValue(5);
	_ui->doubleSpinBox_multiband_bestscore->setValue(0.1);
	_ui->doubleSpinBox_multiband_angle->setValue(90.0);
	_ui->checkBox_multiband_forcevisible->setChecked(false);


	_ui->doubleSpinBox_mesh_angleTolerance->setValue(15.0);
	_ui->checkBox_mesh_quad->setChecked(false);
	_ui->spinBox_mesh_triangleSize->setValue(1);

	_ui->checkBox_poisson_outputPolygons->setChecked(false);
	_ui->checkBox_poisson_manifold->setChecked(true);
	_ui->spinBox_poisson_depth->setValue(0);
	_ui->doubleSpinBox_poisson_targetPolygonSize->setValue(0.03);
	_ui->spinBox_poisson_iso->setValue(8);
	_ui->spinBox_poisson_solver->setValue(8);
	_ui->spinBox_poisson_minDepth->setValue(5);
	_ui->doubleSpinBox_poisson_samples->setValue(1.0);
	_ui->doubleSpinBox_poisson_pointWeight->setValue(4.0);
	_ui->doubleSpinBox_poisson_scale->setValue(1.1);

	_ui->doubleSpinBox_cputsdf_size->setValue(12.0);
	_ui->doubleSpinBox_cputsdf_resolution->setValue(0.01);
	_ui->doubleSpinBox_cputsdf_tuncPos->setValue(0.03);
	_ui->doubleSpinBox_cputsdf_tuncNeg->setValue(0.03);
	_ui->doubleSpinBox_cputsdf_minWeight->setValue(0);
	_ui->doubleSpinBox_cputsdf_flattenRadius->setValue(0.005);
	_ui->spinBox_cputsdf_randomSplit->setValue(1);

	_ui->checkBox_openchisel_mergeVertices->setChecked(true);
	_ui->spinBox_openchisel_chunk_size_x->setValue(16);
	_ui->spinBox_openchisel_chunk_size_y->setValue(16);
	_ui->spinBox_openchisel_chunk_size_z->setValue(16);
	_ui->doubleSpinBox_openchisel_truncation_constant->setValue(0.001504);
	_ui->doubleSpinBox_openchisel_truncation_linear->setValue(0.00152);
	_ui->doubleSpinBox_openchisel_truncation_quadratic->setValue(0.0019);
	_ui->doubleSpinBox_openchisel_truncation_scale->setValue(10.0);
	_ui->spinBox_openchisel_integration_weight->setValue(1);
	_ui->checkBox_openchisel_use_voxel_carving->setChecked(false);
	_ui->doubleSpinBox_openchisel_carving_dist_m->setValue(0.05);
	_ui->doubleSpinBox_openchisel_near_plane_dist->setValue(0.05);
	_ui->doubleSpinBox_openchisel_far_plane_dist->setValue(1.1);


	updateReconstructionFlavor();
	updateMLSGrpVisibility();

	this->update();
}

void ExportCloudsDialog::loadSettings()
{
	QString path = QFileDialog::getOpenFileName(this, tr("Load Settings"), _workingDirectory, tr("Config (*.ini)"));
	if(path.size())
	{
		QSettings settings(path, QSettings::IniFormat);
		settings.beginGroup("Gui");
		settings.beginGroup(this->objectName());
		this->loadSettings(settings);
		settings.endGroup(); // "name"
		settings.endGroup(); // Gui
	}
}

void ExportCloudsDialog::saveSettings()
{
	QString path = QFileDialog::getSaveFileName(this, tr("Save Settings"), _workingDirectory, tr("Config (*.ini)"));
	if(path.size())
	{
		QSettings settings(path, QSettings::IniFormat);
		settings.beginGroup("Gui");
		settings.beginGroup(this->objectName());
		this->saveSettings(settings);
		settings.endGroup(); // "name"
		settings.endGroup(); // Gui
	}
}

void ExportCloudsDialog::updateReconstructionFlavor()
{
	if(!_ui->checkBox_fromDepth->isChecked())
	{
		_ui->comboBox_pipeline->setCurrentIndex(1);
		_ui->comboBox_pipeline->setEnabled(false);
		_ui->comboBox_frame->setItemData(2, 0,Qt::UserRole - 1);
		_ui->comboBox_frame->setItemData(3, 1|32,Qt::UserRole - 1);
		if(_ui->comboBox_frame->currentIndex() == 2)
		{
			_ui->comboBox_frame->setCurrentIndex(0);
		}
	}
	else
	{
		_ui->comboBox_pipeline->setEnabled(true);
		_ui->comboBox_frame->setItemData(2, 1|32,Qt::UserRole - 1);
		_ui->comboBox_frame->setItemData(3, 0,Qt::UserRole - 1);
		if(_ui->comboBox_frame->currentIndex() == 3)
		{
			_ui->comboBox_frame->setCurrentIndex(0);
		}
	}
	_ui->comboBox_intensityColormap->setVisible(!_ui->checkBox_fromDepth->isChecked() && !_ui->checkBox_binary->isEnabled());
	_ui->comboBox_intensityColormap->setEnabled(!_ui->checkBox_fromDepth->isChecked() && !_ui->checkBox_binary->isEnabled());
	_ui->label_intensityColormap->setVisible(!_ui->checkBox_fromDepth->isChecked() && !_ui->checkBox_binary->isEnabled());

	_ui->checkBox_smoothing->setVisible(_ui->comboBox_pipeline->currentIndex() == 1);
	_ui->checkBox_smoothing->setEnabled(_ui->comboBox_pipeline->currentIndex() == 1);
	_ui->label_smoothing->setVisible(_ui->comboBox_pipeline->currentIndex() == 1);

	_ui->comboBox_frame->setEnabled(!_ui->checkBox_assemble->isChecked() && _ui->checkBox_binary->isEnabled());
	_ui->comboBox_frame->setVisible(_ui->comboBox_frame->isEnabled());
	_ui->label_frame->setVisible(_ui->comboBox_frame->isEnabled());
	_ui->checkBox_gainCompensation->setEnabled(!(_ui->comboBox_frame->isEnabled() && _ui->comboBox_frame->currentIndex() == 2));
	_ui->checkBox_gainCompensation->setVisible(_ui->checkBox_gainCompensation->isEnabled());
	_ui->label_gainCompensation->setVisible(_ui->checkBox_gainCompensation->isEnabled());

	_ui->checkBox_cameraProjection->setEnabled(_ui->checkBox_assemble->isChecked() && !_ui->checkBox_meshing->isChecked());
	_ui->label_cameraProjection->setEnabled(_ui->checkBox_cameraProjection->isEnabled());

	_ui->groupBox_nodes_filtering->setVisible(_ui->checkBox_nodes_filtering->isChecked());
	_ui->groupBox_regenerate->setVisible(_ui->checkBox_regenerate->isChecked() && _ui->checkBox_fromDepth->isChecked());
	_ui->groupBox_regenerateScans->setVisible(_ui->checkBox_regenerate->isChecked() && !_ui->checkBox_fromDepth->isChecked());
	_ui->groupBox_bilateral->setVisible(_ui->checkBox_bilateral->isChecked());
	_ui->groupBox_filtering->setVisible(_ui->checkBox_filtering->isChecked());
	_ui->groupBox_gain->setVisible(_ui->checkBox_gainCompensation->isEnabled() && _ui->checkBox_gainCompensation->isChecked());
	_ui->groupBox_cameraProjection->setVisible(_ui->checkBox_cameraProjection->isEnabled() && _ui->checkBox_cameraProjection->isChecked());
	_ui->groupBox_mls->setVisible(_ui->checkBox_smoothing->isEnabled() && _ui->checkBox_smoothing->isChecked());
	_ui->groupBox_meshing->setVisible(_ui->checkBox_meshing->isChecked());
	_ui->groupBox_subtraction->setVisible(_ui->checkBox_subtraction->isChecked());
	_ui->groupBox_textureMapping->setVisible(_ui->checkBox_textureMapping->isChecked());
	_ui->groupBox_cameraFilter->setVisible(_ui->checkBox_cameraFilter->isChecked());
	_ui->groupBox_multiband->setVisible(_ui->checkBox_multiband->isChecked());

	// dense texturing options
	if(_ui->checkBox_meshing->isChecked())
	{
		//GP3
		_ui->comboBox_meshingApproach->setItemData(0, _ui->comboBox_pipeline->currentIndex() == 1?1 | 32:0,Qt::UserRole - 1);

		//Poisson
		_ui->comboBox_meshingApproach->setItemData(1, _ui->comboBox_pipeline->currentIndex() == 1 && _ui->checkBox_assemble->isChecked()?1 | 32:0,Qt::UserRole - 1);

		//CPU-TSDF
#ifdef RTABMAP_CPUTSDF
		_ui->comboBox_meshingApproach->setItemData(2, _ui->comboBox_pipeline->currentIndex() == 0 && _ui->checkBox_assemble->isChecked()?1 | 32:0,Qt::UserRole - 1);
#else
		_ui->comboBox_meshingApproach->setItemData(2, 0, Qt::UserRole - 1);
#endif

		// Organized
		_ui->comboBox_meshingApproach->setItemData(3, _ui->comboBox_pipeline->currentIndex() == 0?1 | 32:0,Qt::UserRole - 1);

		//Open Chisel
#ifdef RTABMAP_OPENCHISEL
		_ui->comboBox_meshingApproach->setItemData(4, _ui->checkBox_assemble->isChecked()?1 | 32:0,Qt::UserRole - 1);
#else
		_ui->comboBox_meshingApproach->setItemData(4, 0, Qt::UserRole - 1);
#endif

		if(_ui->comboBox_pipeline->currentIndex() == 0 && _ui->comboBox_meshingApproach->currentIndex()<2)
		{
			_ui->comboBox_meshingApproach->setCurrentIndex(3);
		}
		if(_ui->comboBox_pipeline->currentIndex() == 1 && (_ui->comboBox_meshingApproach->currentIndex()==2 || _ui->comboBox_meshingApproach->currentIndex()==3))
		{
			_ui->comboBox_meshingApproach->setCurrentIndex(1);
		}
		if(!_ui->checkBox_assemble->isChecked())
		{
			_ui->comboBox_meshingApproach->setCurrentIndex(_ui->comboBox_pipeline->currentIndex() == 1?0:3);
		}

		_ui->checkBox_poisson_outputPolygons->setDisabled(
					_ui->checkBox_binary->isEnabled() ||
					_ui->doubleSpinBox_meshDecimationFactor->value()!=0.0 ||
					_ui->spinBox_meshMaxPolygons->value()!=0 ||
					_ui->checkBox_textureMapping->isChecked());
		_ui->label_outputPolygons->setEnabled(_ui->checkBox_poisson_outputPolygons->isEnabled());

		_ui->checkBox_cleanMesh->setEnabled(_ui->comboBox_pipeline->currentIndex() == 1);
		_ui->label_meshClean->setEnabled(_ui->comboBox_pipeline->currentIndex() == 1);

		_ui->groupBox_gp3->setVisible(_ui->comboBox_pipeline->currentIndex() == 1 && _ui->comboBox_meshingApproach->currentIndex()==0);
		_ui->groupBox_poisson->setVisible(_ui->comboBox_pipeline->currentIndex() == 1 && _ui->comboBox_meshingApproach->currentIndex()==1);
		_ui->groupBox_cputsdf->setVisible(_ui->comboBox_pipeline->currentIndex() == 0 && _ui->comboBox_meshingApproach->currentIndex()==2);
		_ui->groupBox_organized->setVisible(_ui->comboBox_pipeline->currentIndex() == 0 && _ui->comboBox_meshingApproach->currentIndex()==3);
		_ui->groupBox_openchisel->setVisible(_ui->comboBox_meshingApproach->currentIndex()==4);

#ifndef DISABLE_VTK
		_ui->doubleSpinBox_meshDecimationFactor->setEnabled(_ui->comboBox_meshingApproach->currentIndex()!=3);
		_ui->label_meshDecimation->setEnabled(_ui->comboBox_meshingApproach->currentIndex()!=3);
		_ui->spinBox_meshMaxPolygons->setEnabled(_ui->comboBox_meshingApproach->currentIndex()!=3);
		_ui->label_meshMaxPolygons->setEnabled(_ui->comboBox_meshingApproach->currentIndex()!=3);
#endif

#ifndef RTABMAP_ALICE_VISION
		_ui->checkBox_multiband->setEnabled(false);
#else
		_ui->checkBox_multiband->setEnabled(_ui->checkBox_binary->isEnabled());
#endif
		_ui->label_multiband->setEnabled(_ui->checkBox_multiband->isEnabled());
	}
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

void ExportCloudsDialog::selectCamProjMask()
{
	QString dir = _ui->lineEdit_camProjMaskFilePath->text();
	if(dir.isEmpty())
	{
		dir = _workingDirectory;
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Mask (grayscale) (*.png *.pgm *bmp)"));
	if(path.size())
	{
		_ui->lineEdit_camProjMaskFilePath->setText(path);
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
	updateReconstructionFlavor();
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
	updateReconstructionFlavor();
}

std::map<int, Transform> ExportCloudsDialog::filterNodes(const std::map<int, Transform> & poses)
{
	if(_ui->checkBox_nodes_filtering->isChecked())
	{
		std::map<int, Transform> posesFiltered;
		for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			bool ignore = false;
			if(_ui->doubleSpinBox_nodes_filtering_xmin->value() != _ui->doubleSpinBox_nodes_filtering_xmax->value() &&
				(iter->second.x() < _ui->doubleSpinBox_nodes_filtering_xmin->value() ||
				 iter->second.x() > _ui->doubleSpinBox_nodes_filtering_xmax->value()))
			{
				ignore = true;
			}
			if(_ui->doubleSpinBox_nodes_filtering_ymin->value() != _ui->doubleSpinBox_nodes_filtering_ymax->value() &&
				(iter->second.y() < _ui->doubleSpinBox_nodes_filtering_ymin->value() ||
				 iter->second.y() > _ui->doubleSpinBox_nodes_filtering_ymax->value()))
			{
				ignore = true;
			}
			if(_ui->doubleSpinBox_nodes_filtering_zmin->value() != _ui->doubleSpinBox_nodes_filtering_zmax->value() &&
				(iter->second.z() < _ui->doubleSpinBox_nodes_filtering_zmin->value() ||
				 iter->second.z() > _ui->doubleSpinBox_nodes_filtering_zmax->value()))
			{
				ignore = true;
			}
			if(!ignore)
			{
				posesFiltered.insert(*iter);
			}
		}
		return posesFiltered;
	}
	return poses;
}

void ExportCloudsDialog::exportClouds(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, int> & mapIds,
		const QMap<int, Signature> & cachedSignatures,
		const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> > & cachedClouds,
		const std::map<int, LaserScan> & cachedScans,
		const QString & workingDirectory,
		const ParametersMap & parameters)
{
	std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds;
	std::map<int, pcl::PolygonMesh::Ptr> meshes;
	std::map<int, pcl::TextureMesh::Ptr> textureMeshes;
	std::vector<std::map<int, pcl::PointXY> > textureVertexToPixels;

	setSaveButton();

	if(getExportedClouds(
			poses,
			links,
			mapIds,
			cachedSignatures,
			cachedClouds,
			cachedScans,
			workingDirectory,
			parameters,
			clouds,
			meshes,
			textureMeshes,
			textureVertexToPixels))
	{
		if(textureMeshes.size())
		{
			saveTextureMeshes(workingDirectory, poses, textureMeshes, cachedSignatures, textureVertexToPixels);
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
			saveClouds(workingDirectory, poses, clouds, _ui->checkBox_binary->isChecked(), textureVertexToPixels);
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
		const std::map<int, LaserScan> & cachedScans,
		const QString & workingDirectory,
		const ParametersMap & parameters)
{
	std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds;
	std::map<int, pcl::PolygonMesh::Ptr> meshes;
	std::map<int, pcl::TextureMesh::Ptr> textureMeshes;
	std::vector<std::map<int, pcl::PointXY> > textureVertexToPixels;

	setOkButton();

	if(getExportedClouds(
			poses,
			links,
			mapIds,
			cachedSignatures,
			cachedClouds,
			cachedScans,
			workingDirectory,
			parameters,
			clouds,
			meshes,
			textureMeshes,
			textureVertexToPixels))
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
		window->resize(this->window()->windowHandle()->screen()->availableGeometry().size() * 0.7);

		CloudViewer * viewer = new CloudViewer(window);
		if(_ui->comboBox_pipeline->currentIndex() == 0)
		{
			viewer->setBackfaceCulling(true, false);
		}
		viewer->setLighting(false);
		viewer->setDefaultBackgroundColor(QColor(40, 40, 40, 255));
		viewer->buildPickingLocator(true);
		if(_ui->comboBox_intensityColormap->currentIndex()==1)
		{
			viewer->setIntensityRedColormap(true);
		}
		else if(_ui->comboBox_intensityColormap->currentIndex() == 2)
		{
			viewer->setIntensityRainbowColormap(true);
		}

		QVBoxLayout *layout = new QVBoxLayout();
		layout->addWidget(viewer);
		layout->setContentsMargins(0,0,0,0);
		window->setLayout(layout);
		connect(window, SIGNAL(finished(int)), viewer, SLOT(clear()));

		window->show();

		_progressDialog->appendText(tr("Opening visualizer..."));
		QApplication::processEvents();
		uSleep(500);
		QApplication::processEvents();

		if(textureMeshes.size())
		{
			viewer->setPolygonPicking(true);
			std::map<int, cv::Mat> images;
			std::map<int, std::vector<CameraModel> > calibrations;
			for(QMap<int, Signature>::const_iterator iter=cachedSignatures.constBegin(); iter!=cachedSignatures.constEnd(); ++iter)
			{
				std::vector<CameraModel> models;
				if(iter->sensorData().cameraModels().size())
				{
					models = iter->sensorData().cameraModels();
				}
				else if(iter->sensorData().stereoCameraModels().size())
				{
					for(size_t i=0; i<iter->sensorData().stereoCameraModels().size(); ++i)
					{
						models.push_back(iter->sensorData().stereoCameraModels()[i].left());
					}
				}

				if(!models.empty())
				{
					if(!iter->sensorData().imageRaw().empty())
					{
						calibrations.insert(std::make_pair(iter.key(), models));
						images.insert(std::make_pair(iter.key(), iter->sensorData().imageRaw()));
					}
					else if(!iter->sensorData().imageCompressed().empty())
					{
						calibrations.insert(std::make_pair(iter.key(), models));
						images.insert(std::make_pair(iter.key(), iter->sensorData().imageCompressed()));
					}
				}
			}
			int textureSize = 1024;
			if(_ui->comboBox_meshingTextureSize->currentIndex() > 0)
			{
				textureSize = 128 << _ui->comboBox_meshingTextureSize->currentIndex(); // start at 256
			}
			int blendingDecimation = 0;
			if(_ui->checkBox_blending->isChecked())
			{
				if(_ui->comboBox_blendingDecimation->currentIndex() > 0)
				{
					blendingDecimation = 1 << (_ui->comboBox_blendingDecimation->currentIndex()-1);
				}
			}

			for (std::map<int, pcl::TextureMesh::Ptr>::iterator iter = textureMeshes.begin(); iter != textureMeshes.end(); ++iter)
			{
				pcl::TextureMesh::Ptr mesh = iter->second;

				// As CloudViewer is not supporting more than one texture per mesh, merge them all by default
				cv::Mat globalTexture;
				if (mesh->tex_materials.size() > 1)
				{
					cv::Mat globalTextures;
					globalTextures = util3d::mergeTextures(
							*mesh,
							images,
							calibrations,
							0,
							_dbDriver,
							textureSize,
							1,
							textureVertexToPixels,
							_ui->checkBox_gainCompensation->isChecked(),
							_ui->doubleSpinBox_gainBeta->value(),
							_ui->checkBox_gainRGB->isChecked(),
							_ui->checkBox_blending->isChecked(),
							blendingDecimation,
							_ui->spinBox_textureBrightnessContrastRatioLow->value(),
							_ui->spinBox_textureBrightnessContrastRatioHigh->value(),
							_ui->checkBox_exposureFusion->isEnabled() && _ui->checkBox_exposureFusion->isChecked());
					if(globalTextures.rows == globalTextures.cols)
					{
						globalTexture = globalTextures;
					}
				}

				_progressDialog->appendText(tr("Viewing the mesh %1 (%2 polygons)...").arg(iter->first).arg(mesh->tex_polygons.size()?mesh->tex_polygons[0].size():0));
				_progressDialog->incrementStep();

				// VTK issue:
				//  tex_coordinates should be linked to points, not
				//  polygon vertices. Points linked to multiple different TCoords (different textures) should
				//  be duplicated.
				for (unsigned int t = 0; t < mesh->tex_coordinates.size(); ++t)
				{
					if(mesh->tex_polygons[t].size())
					{

						pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::fromPCLPointCloud2(mesh->cloud, *originalCloud);

						// make a cloud with as many points than polygon vertices
						unsigned int nPoints = mesh->tex_coordinates[t].size();
						UASSERT(nPoints == mesh->tex_polygons[t].size()*mesh->tex_polygons[t][0].vertices.size()); // assuming polygon size is constant!

						pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
						cloud->resize(nPoints);

						unsigned int oi = 0;
						for (unsigned int i = 0; i < mesh->tex_polygons[t].size(); ++i)
						{
							pcl::Vertices & vertices = mesh->tex_polygons[t][i];

							for(unsigned int j=0; j<vertices.vertices.size(); ++j)
							{
								UASSERT(oi < cloud->size());
								UASSERT_MSG((int)vertices.vertices[j] < (int)originalCloud->size(), uFormat("%d vs %d", vertices.vertices[j], (int)originalCloud->size()).c_str());
								cloud->at(oi) = originalCloud->at(vertices.vertices[j]);
								vertices.vertices[j] = oi; // new vertice index
								++oi;
							}
						}
						pcl::toPCLPointCloud2(*cloud, mesh->cloud);
					}
					else
					{
						UWARN("No polygons for texture %d of mesh %d?!", t, iter->first);
					}
				}

				if (globalTexture.empty())
				{
					if(mesh->tex_materials.size()==1 &&
						!mesh->tex_materials[0].tex_file.empty() &&
						uIsInteger(mesh->tex_materials[0].tex_file, false))
					{
						int textureId = uStr2Int(mesh->tex_materials[0].tex_file);
						SensorData data;
						if(cachedSignatures.contains(textureId) && !cachedSignatures.value(textureId).sensorData().imageCompressed().empty())
						{
							data = cachedSignatures.value(textureId).sensorData();
						}
						else if(_dbDriver)
						{
							_dbDriver->getNodeData(textureId, data, true, false, false, false);
						}
						UASSERT(!data.imageCompressed().empty());
						data.uncompressDataConst(&globalTexture, 0);
						UASSERT(!globalTexture.empty());
						if (_ui->checkBox_gainCompensation->isChecked() && _compensator && _compensator->getIndex(textureId) >= 0)
						{
							_compensator->apply(textureId, globalTexture, _ui->checkBox_gainRGB->isChecked());
						}
					}
					else
					{
						_progressDialog->appendText(tr("No textures added to mesh %1!?").arg(iter->first), Qt::darkRed);
						_progressDialog->setAutoClose(false);
					}
				}

				viewer->addCloudTextureMesh(uFormat("mesh%d",iter->first), mesh, globalTexture, iter->first>0?poses.at(iter->first):Transform::getIdentity());
				_progressDialog->appendText(tr("Viewing the mesh %1 (%2 polygons)... done.").arg(iter->first).arg(mesh->tex_polygons.size()?mesh->tex_polygons[0].size():0));
				QApplication::processEvents();
			}
		}
		else if(meshes.size())
		{
			viewer->setPolygonPicking(true);
			viewer->setLighting(_ui->doubleSpinBox_transferColorRadius->value() < 0.0);
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

				if(!_ui->checkBox_fromDepth->isChecked() && !_scansHaveRGB &&
					!(_ui->checkBox_cameraProjection->isEnabled() &&
					_ui->checkBox_cameraProjection->isChecked() &&
					_ui->checkBox_camProjRecolorPoints->isChecked() &&
					clouds.size()==1 && clouds.begin()->first==0))
				{
					// When laser scans are exported (and camera RGB was not applied), convert RGB to Intensity
					if(_ui->spinBox_normalKSearch->value()<=0 && _ui->doubleSpinBox_normalRadiusSearch->value()<=0.0)
					{
						// remove normals if not used
						pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIWithoutNormals(new pcl::PointCloud<pcl::PointXYZI>);
						cloudIWithoutNormals->resize(iter->second->size());
						for(unsigned int i=0; i<cloudIWithoutNormals->size(); ++i)
						{
							cloudIWithoutNormals->points[i].x = iter->second->points[i].x;
							cloudIWithoutNormals->points[i].y = iter->second->points[i].y;
							cloudIWithoutNormals->points[i].z = iter->second->points[i].z;
							int * intensity = (int *)&cloudIWithoutNormals->points[i].intensity;
							*intensity =
									int(iter->second->points[i].r) |
									int(iter->second->points[i].g) << 8 |
									int(iter->second->points[i].b) << 16 |
									int(iter->second->points[i].a) << 24;
						}
						viewer->addCloud(uFormat("cloud%d",iter->first), cloudIWithoutNormals, iter->first>0?poses.at(iter->first):Transform::getIdentity());
					}
					else
					{
						pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudI(new pcl::PointCloud<pcl::PointXYZINormal>);
						cloudI->resize(iter->second->size());
						for(unsigned int i=0; i<cloudI->size(); ++i)
						{
							cloudI->points[i].x = iter->second->points[i].x;
							cloudI->points[i].y = iter->second->points[i].y;
							cloudI->points[i].z = iter->second->points[i].z;
							cloudI->points[i].normal_x = iter->second->points[i].normal_x;
							cloudI->points[i].normal_y = iter->second->points[i].normal_y;
							cloudI->points[i].normal_z = iter->second->points[i].normal_z;
							cloudI->points[i].curvature = iter->second->points[i].curvature;
							int * intensity = (int *)&cloudI->points[i].intensity;
							*intensity =
									int(iter->second->points[i].r) |
									int(iter->second->points[i].g) << 8 |
									int(iter->second->points[i].b) << 16 |
									int(iter->second->points[i].a) << 24;
						}
						viewer->addCloud(uFormat("cloud%d",iter->first), cloudI, iter->first>0?poses.at(iter->first):Transform::getIdentity());
					}
				}
				else
				{
					viewer->addCloud(uFormat("cloud%d",iter->first), iter->second, iter->first>0?poses.at(iter->first):Transform::getIdentity());
				}

				viewer->setCloudPointSize(uFormat("cloud%d",iter->first), 1);
				_progressDialog->appendText(tr("Viewing the cloud %1 (%2 points)... done.").arg(iter->first).arg(iter->second->size()));
			}
		}
		viewer->refreshView();
	}
	else
	{
		_progressDialog->setAutoClose(false);
	}
	_progressDialog->setValue(_progressDialog->maximumSteps());
}

int ExportCloudsDialog::getTextureSize() const
{
	int textureSize = 1024;
	if(_ui->comboBox_meshingTextureSize->currentIndex() > 0)
	{
		textureSize = 128 << _ui->comboBox_meshingTextureSize->currentIndex(); // start at 256
	}
	return textureSize;
}
int ExportCloudsDialog::getMaxTextures() const
{
	return _ui->spinBox_mesh_maxTextures->value();
}
bool ExportCloudsDialog::isGainCompensation() const
{
	return _ui->checkBox_gainCompensation->isChecked();
}
double ExportCloudsDialog::getGainBeta() const
{
	return _ui->doubleSpinBox_gainBeta->value();
}
bool ExportCloudsDialog::isGainRGB() const
{
	return _ui->checkBox_gainRGB->isChecked();
}
bool ExportCloudsDialog::isBlending() const
{
	return _ui->checkBox_blending->isChecked();
}
int ExportCloudsDialog::getBlendingDecimation() const
{
	int blendingDecimation = 0;
	if(_ui->checkBox_blending->isChecked())
	{
		if(_ui->comboBox_blendingDecimation->currentIndex() > 0)
		{
			blendingDecimation = 1 << (_ui->comboBox_blendingDecimation->currentIndex()-1);
		}
	}
	return blendingDecimation;
}
int ExportCloudsDialog::getTextureBrightnessConstrastRatioLow() const
{
	return _ui->spinBox_textureBrightnessContrastRatioLow->value();
}
int ExportCloudsDialog::getTextureBrightnessConstrastRatioHigh() const
{
	return _ui->spinBox_textureBrightnessContrastRatioHigh->value();
}
bool ExportCloudsDialog::isExposeFusion() const
{
	return _ui->checkBox_exposureFusion->isEnabled() && _ui->checkBox_exposureFusion->isChecked();
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
		const std::map<int, Transform> & posesIn,
		const std::multimap<int, Link> & links,
		const std::map<int, int> & mapIds,
		const QMap<int, Signature> & cachedSignatures,
		const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> > & cachedClouds,
		const std::map<int, LaserScan> & cachedScans,
		const QString & workingDirectory,
		const ParametersMap & parameters,
		std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> & cloudsWithNormals,
		std::map<int, pcl::PolygonMesh::Ptr> & meshes,
		std::map<int, pcl::TextureMesh::Ptr> & textureMeshes,
		std::vector<std::map<int, pcl::PointXY> > & textureVertexToPixels)
{
	_canceled = false;
	_workingDirectory = workingDirectory;
	_ui->checkBox_regenerate->setEnabled(true);
	if(cachedSignatures.empty() && _dbDriver)
	{
		_ui->checkBox_regenerate->setChecked(true);
		_ui->checkBox_regenerate->setEnabled(false);
	}
	if(_compensator)
	{
		delete _compensator;
		_compensator = 0;
	}
	if(this->exec() == QDialog::Accepted)
	{
		std::map<int, Transform> poses = filterNodes(posesIn);

		if(poses.empty())
		{
			QMessageBox::critical(this, tr("Creating clouds..."), tr("Poses are empty! Cannot export/view clouds."));
			return false;
		}
		_progressDialog->resetProgress();
		_progressDialog->show();
		int mul = 1;
		if(_ui->checkBox_meshing->isChecked())
		{
			mul+=1;
		}
		if(_ui->checkBox_assemble->isChecked())
		{
			mul+=1;
		}

		if(_ui->checkBox_subtraction->isChecked())
		{
			mul+=1;
		}
		if(_ui->checkBox_textureMapping->isChecked())
		{
			mul+=1;
		}
		if(_ui->checkBox_gainCompensation->isChecked())
		{
			mul+=1;
		}
		if(_ui->checkBox_mesh_quad->isEnabled()) // when enabled we are viewing the clouds
		{
			mul+=1;
		}
		_progressDialog->setMaximumSteps(int(poses.size())*mul+1);

		bool loadClouds = true;
#ifdef RTABMAP_OPENCHISEL
		if(_ui->comboBox_meshingApproach->currentIndex()==4 && _ui->checkBox_assemble->isChecked())
		{
			loadClouds = !_ui->checkBox_fromDepth->isChecked();
		}
#endif

		bool has2dScans = false;
		std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> > clouds;
		if(loadClouds)
		{
			clouds = this->getClouds(
					poses,
					cachedSignatures,
					cachedClouds,
					cachedScans,
					parameters,
					has2dScans,
					_scansHaveRGB);
		}
		else
		{
			// just create empty clouds
			for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				clouds.insert(std::make_pair(iter->first,
						std::make_pair(
								pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>),
								pcl::IndicesPtr(new std::vector<int>))));

			}
		}

		std::set<int> validCameras = uKeysSet(clouds);

		UDEBUG("");
		if(_canceled)
		{
			return false;
		}

		if(clouds.empty())
		{
			_progressDialog->setAutoClose(false);
			if(_ui->checkBox_regenerate->isEnabled() && !_ui->checkBox_regenerate->isChecked())
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

		UDEBUG("");
		if(_ui->checkBox_gainCompensation->isChecked() && _ui->checkBox_fromDepth->isChecked() && clouds.size() > 1 &&
				// Do compensation later if we are merging textures on a dense assembled cloud
				!(_ui->checkBox_meshing->isChecked() &&
					_ui->checkBox_textureMapping->isEnabled() &&
					_ui->checkBox_textureMapping->isChecked() &&
					_ui->comboBox_pipeline->currentIndex()==1 &&
					_ui->checkBox_assemble->isChecked() &&
					_ui->comboBox_meshingTextureSize->isEnabled() &&
					_ui->comboBox_meshingTextureSize->currentIndex() > 0) &&
				// Don't do compensation if clouds are in camera frame
				!(_ui->comboBox_frame->isEnabled() && _ui->comboBox_frame->currentIndex()==2))
		{
			UASSERT(_compensator == 0);
			_compensator = new GainCompensator(_ui->doubleSpinBox_gainRadius->value(), _ui->doubleSpinBox_gainOverlap->value(), 0.01, _ui->doubleSpinBox_gainBeta->value());

			if(_ui->checkBox_gainFull->isChecked())
			{
				_progressDialog->appendText(tr("Full gain compensation of %1 clouds...").arg(clouds.size()));
			}
			else
			{
				_progressDialog->appendText(tr("Gain compensation of %1 clouds...").arg(clouds.size()));
			}
			QApplication::processEvents();
			uSleep(100);
			QApplication::processEvents();

			if(_ui->checkBox_gainFull->isChecked())
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

				_compensator->feed(clouds, allLinks);
			}
			else
			{
				_compensator->feed(clouds, links);
			}

			if(!(_ui->checkBox_meshing->isChecked() &&
				 _ui->checkBox_textureMapping->isEnabled() &&
				 _ui->checkBox_textureMapping->isChecked()))
			{
				_progressDialog->appendText(tr("Applying gain compensation..."));
				for(std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> >::iterator jter=clouds.begin();jter!=clouds.end(); ++jter)
				{
					if(jter!=clouds.end())
					{
						double gain = _compensator->getGain(jter->first);;
						_compensator->apply(jter->first, jter->second.first, jter->second.second, _ui->checkBox_gainRGB->isChecked());

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
		}

		UDEBUG("");
		std::map<int, Transform> normalViewpoints = poses;
		if(_ui->checkBox_assemble->isChecked())
		{
			// Adjust view points with local transforms
			for(std::map<int, Transform>::iterator iter= normalViewpoints.begin(); iter!=normalViewpoints.end(); ++iter)
			{
				if(_ui->checkBox_fromDepth->isChecked())
				{
					std::vector<CameraModel> models;
					std::vector<StereoCameraModel> stereoModels;
					if(cachedSignatures.contains(iter->first))
					{
						const SensorData & data = cachedSignatures.find(iter->first)->sensorData();
						models = data.cameraModels();
						stereoModels = data.stereoCameraModels();
					}
					else if(_dbDriver)
					{
						_dbDriver->getCalibration(iter->first, models, stereoModels);
					}

					if(models.size() && !models[0].localTransform().isNull())
					{
						iter->second *= models[0].localTransform();
					}
					else if(stereoModels.size() && !stereoModels[0].localTransform().isNull())
					{
						iter->second *= stereoModels[0].localTransform();
					}
				}
				else
				{
					if(uContains(cachedScans, iter->first))
					{
						iter->second *= cachedScans.at(iter->first).localTransform();
					}
					else if(cachedSignatures.contains(iter->first))
					{
						const SensorData & data = cachedSignatures.find(iter->first)->sensorData();
						if(!data.laserScanCompressed().isEmpty())
						{
							iter->second *= data.laserScanCompressed().localTransform();
						}
						else if(!data.laserScanRaw().isEmpty())
						{
							iter->second *= data.laserScanRaw().localTransform();
						}
					}
					else if(_dbDriver)
					{
						LaserScan scan;
						_dbDriver->getLaserScanInfo(iter->first, scan);
						iter->second *= scan.localTransform();
					}
				}
			}
		}

		UDEBUG("");
		pcl::PointCloud<pcl::PointXYZ>::Ptr rawAssembledCloud(new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<int> rawCameraIndices;
		if(_ui->checkBox_assemble->isChecked() &&
		   !((_ui->comboBox_pipeline->currentIndex()==0 || _ui->comboBox_meshingApproach->currentIndex()==4) && _ui->checkBox_meshing->isChecked()))
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
					// it looks like that using only transformPointCloud with indices
					// flushes the colors, so we should extract points before... maybe a too old PCL version
					pcl::copyPointCloud(*iter->second.first, *iter->second.second, *transformed);
					transformed = rtabmap::util3d::transformPointCloud(transformed, poses.at(iter->first));
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

			assembledCloud->is_dense = true;
			if(_ui->spinBox_normalKSearch->value()>0 || _ui->doubleSpinBox_normalRadiusSearch->value()>0.0)
			{
				pcl::copyPointCloud(*assembledCloud, *rawAssembledCloud);
			}

			if(_ui->doubleSpinBox_voxelSize_assembled->value())
			{
				_progressDialog->appendText(tr("Voxelize cloud (%1 points, voxel size = %2 m)...")
						.arg(assembledCloud->size())
						.arg(_ui->doubleSpinBox_voxelSize_assembled->value()));
				QApplication::processEvents();
				unsigned int before = assembledCloud->size();
				assembledCloud = util3d::voxelize(
						assembledCloud,
						_ui->doubleSpinBox_voxelSize_assembled->value());
				_progressDialog->appendText(tr("Voxelize cloud (%1 points, voxel size = %2 m)...done! (%3 points)")
						.arg(before)
						.arg(_ui->doubleSpinBox_voxelSize_assembled->value())
						.arg(assembledCloud->size()));
			}

			clouds.clear();
			pcl::IndicesPtr indices(new std::vector<int>);
			indices->resize(assembledCloud->size());
			for(unsigned int i=0; i<indices->size(); ++i)
			{
				indices->at(i) = i;
			}

			if(!_ui->checkBox_fromDepth->isChecked() && !has2dScans &&
					(_ui->spinBox_normalKSearch->value()>0 || _ui->doubleSpinBox_normalRadiusSearch->value()>0.0))
			{
				_progressDialog->appendText(tr("Computing normals (%1 points, K=%2, radius=%3 m)...")
										.arg(assembledCloud->size())
										.arg(_ui->spinBox_normalKSearch->value())
										.arg(_ui->doubleSpinBox_normalRadiusSearch->value()));

				// recompute normals
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWithoutNormals(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*assembledCloud, *cloudWithoutNormals);
				pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloudWithoutNormals, indices, _ui->spinBox_normalKSearch->value(), _ui->doubleSpinBox_normalRadiusSearch->value());

				UASSERT(assembledCloud->size() == normals->size());
				for(unsigned int i=0; i<normals->size(); ++i)
				{
					assembledCloud->points[i].normal_x = normals->points[i].normal_x;
					assembledCloud->points[i].normal_y = normals->points[i].normal_y;
					assembledCloud->points[i].normal_z = normals->points[i].normal_z;
				}
				_progressDialog->appendText(tr("Adjusting normals to viewpoints (%1 points)...").arg(assembledCloud->size()));

				// adjust with point of views
				util3d::adjustNormalsToViewPoints(
											normalViewpoints,
											rawAssembledCloud,
											rawCameraIndices,
											assembledCloud,
											_ui->doubleSpinBox_groundNormalsUp->value());
			}

			if(_ui->spinBox_randomSamples_assembled->value()>0 &&
			   (int)assembledCloud->size() > _ui->spinBox_randomSamples_assembled->value())
			{
				_progressDialog->appendText(tr("Random samples filtering (in=%1 points, samples=%2)...")
														.arg(assembledCloud->size())
														.arg(_ui->spinBox_randomSamples_assembled->value()));
				assembledCloud = util3d::randomSampling(assembledCloud, _ui->spinBox_randomSamples_assembled->value());
				_progressDialog->appendText(tr("Random samples filtering (out=%1 points, samples=%2)... done!")
																		.arg(assembledCloud->size())
																		.arg(_ui->spinBox_randomSamples_assembled->value()));
			}

			clouds.insert(std::make_pair(0, std::make_pair(assembledCloud, indices)));
		}

		UDEBUG("");
		if(_canceled)
		{
			return false;
		}

		if(_ui->checkBox_smoothing->isEnabled() && _ui->checkBox_smoothing->isChecked() && !has2dScans)
		{
			_progressDialog->appendText(tr("Smoothing the surface using Moving Least Squares (MLS) algorithm... "
					"[search radius=%1m voxel=%2m]").arg(_ui->doubleSpinBox_mlsRadius->value()).arg(_ui->doubleSpinBox_voxelSize_assembled->value()));
			QApplication::processEvents();
			uSleep(100);
			QApplication::processEvents();
		}

		//fill cloudWithNormals
		for(std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> >::iterator iter=clouds.begin();
			iter!= clouds.end();)
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals = iter->second.first;

			if(_ui->checkBox_smoothing->isEnabled() && _ui->checkBox_smoothing->isChecked() && !has2dScans)
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
				uSleep(100);
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

				// make sure there are no nans
				UDEBUG("NaNs filtering... size before = %d", cloudWithNormals->size());
				cloudWithNormals = util3d::removeNaNNormalsFromPointCloud(cloudWithNormals);
				UDEBUG("NaNs filtering... size after = %d", cloudWithNormals->size());

				if(_ui->checkBox_assemble->isChecked())
				{
					// Re-voxelize to make sure to have uniform density
                    if(_ui->doubleSpinBox_mls_outputVoxelSize->value())
					{
						_progressDialog->appendText(tr("Voxelize cloud (%1 points, voxel size = %2 m)...")
								.arg(cloudWithNormals->size())
								.arg(_ui->doubleSpinBox_mls_outputVoxelSize->value()));
						QApplication::processEvents();

						cloudWithNormals = util3d::voxelize(
								cloudWithNormals,
								_ui->doubleSpinBox_mls_outputVoxelSize->value());
					}
				
					_progressDialog->appendText(tr("Update %1 normals with %2 camera views...").arg(cloudWithNormals->size()).arg(poses.size()));

					util3d::adjustNormalsToViewPoints(
							normalViewpoints,
							rawAssembledCloud,
							rawCameraIndices,
							cloudWithNormals);
				}
			}
			else if(iter->second.first->isOrganized() && _ui->checkBox_filtering->isChecked())
			{
				cloudWithNormals = util3d::extractIndices(iter->second.first, iter->second.second, false, true);
			}

			cloudsWithNormals.insert(std::make_pair(iter->first, cloudWithNormals));

			// clear memory
			clouds.erase(iter++);

			_progressDialog->incrementStep();
			QApplication::processEvents();
			if(_canceled)
			{
				return false;
			}
		}

		UDEBUG("");
#ifdef RTABMAP_CPUTSDF
		cpu_tsdf::TSDFVolumeOctree::Ptr tsdf;
#endif
#ifdef RTABMAP_OPENCHISEL
		chisel::ChiselPtr chiselMap;
		chisel::ProjectionIntegrator projectionIntegrator;
#endif

		//used for organized texturing below
		std::map<int, std::vector<int> > organizedIndices;
		std::map<int, cv::Size> organizedCloudSizes;

		//mesh
		UDEBUG("Meshing=%d", _ui->checkBox_meshing->isChecked()?1:0);
		if(_ui->checkBox_meshing->isChecked() && !has2dScans)
		{

#ifdef RTABMAP_OPENCHISEL
			if(_ui->comboBox_meshingApproach->currentIndex()==4 && _ui->checkBox_assemble->isChecked())
			{
				_progressDialog->appendText(tr("Creating TSDF volume with OpenChisel... "));

				QApplication::processEvents();
				uSleep(100);
				QApplication::processEvents();

				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				std::vector<pcl::Vertices> mergedPolygons;

				int cloudsAdded = 1;
				for(std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr >::iterator iter=cloudsWithNormals.begin();
					iter!= cloudsWithNormals.end();
					++iter,++cloudsAdded)
				{
					std::vector<CameraModel> models;
					StereoCameraModel stereoModel;
					bool cacheHasCompressedImage = false;
					LaserScan scanInfo;
					if(cachedSignatures.contains(iter->first))
					{
						const SensorData & data = cachedSignatures.find(iter->first)->sensorData();
						models = data.cameraModels();
						cacheHasCompressedImage = !data.imageCompressed().empty();
						scanInfo = !data.laserScanRaw().isEmpty()?data.laserScanRaw():data.laserScanCompressed();
					}
					else if(_dbDriver)
					{
						_dbDriver->getCalibration(iter->first, models, stereoModel);
						_dbDriver->getLaserScanInfo(iter->first, scanInfo);
					}

					if(chiselMap.get() == 0)
					{
						UDEBUG("");
						int chunkSizeX = _ui->spinBox_openchisel_chunk_size_x->value();
						int chunkSizeY = _ui->spinBox_openchisel_chunk_size_y->value();
						int chunkSizeZ = _ui->spinBox_openchisel_chunk_size_z->value();
						float voxelResolution = _ui->doubleSpinBox_voxelSize_assembled->value();
						if(voxelResolution <=0.0f)
						{
							_progressDialog->appendText(tr("OpenChisel: Voxel size should not be null!"), Qt::darkYellow);
							_progressDialog->setAutoClose(false);
							break;
						}
						bool useColor = _ui->checkBox_fromDepth->isChecked();
						chiselMap.reset(new chisel::Chisel(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ), voxelResolution, useColor));
						double truncationDistConst = _ui->doubleSpinBox_openchisel_truncation_constant->value();
						double truncationDistLinear = _ui->doubleSpinBox_openchisel_truncation_linear->value();
						double truncationDistQuad = _ui->doubleSpinBox_openchisel_truncation_quadratic->value();
						double truncationDistScale = _ui->doubleSpinBox_openchisel_truncation_scale->value();
						int weight = _ui->spinBox_openchisel_integration_weight->value();
						bool useCarving = _ui->checkBox_openchisel_use_voxel_carving->isChecked();
						double carvingDist = _ui->doubleSpinBox_openchisel_carving_dist_m->value();
						chisel::Vec4 truncation(truncationDistQuad, truncationDistLinear, truncationDistConst, truncationDistScale);
						UDEBUG("If crashing just after this message, make sure PCL and OpenChisel are built both with -march=native or both without -march=native");
						projectionIntegrator.SetCentroids(chiselMap->GetChunkManager().GetCentroids());
						projectionIntegrator.SetTruncator(chisel::TruncatorPtr(new chisel::QuadraticTruncator(truncation(0), truncation(1), truncation(2), truncation(3))));
						projectionIntegrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(weight)));
						projectionIntegrator.SetCarvingDist(carvingDist);
						projectionIntegrator.SetCarvingEnabled(useCarving);
					}

					UDEBUG("");
					double nearPlaneDist = _ui->doubleSpinBox_openchisel_near_plane_dist->value();
					double farPlaneDist = _ui->doubleSpinBox_openchisel_far_plane_dist->value();
					if(_ui->checkBox_fromDepth->isChecked())
					{
						if(models.size() == 1 && !models[0].localTransform().isNull())
						{
							// get just the depth
							cv::Mat rgb;
							cv::Mat depth;
							if(cacheHasCompressedImage)
							{
								cachedSignatures.find(iter->first)->sensorData().uncompressDataConst(&rgb, &depth);
							}
							else if(_dbDriver)
							{
								SensorData data;
								_dbDriver->getNodeData(iter->first, data, true, false, false, false);
								data.uncompressDataConst(&rgb, &depth);
							}
							if(!rgb.empty() && !depth.empty())
							{
								CameraModel rgbModel = models[0];
								CameraModel depthModel = rgbModel;
								if(rgb.cols > depth.cols)
								{
									UASSERT(rgb.cols % depth.cols == 0);
									depthModel = depthModel.scaled(double(depth.cols)/double(rgb.cols));
								}

								if(depth.type() == CV_16UC1)
								{
									depth = util2d::cvtDepthToFloat(depth);
								}

								std::shared_ptr<chisel::ColorImage<unsigned char> > colorChisel = colorImageToChisel(rgb);
								std::shared_ptr<chisel::DepthImage<float> > depthChisel = depthImageToChisel(depth);

								chisel::PinholeCamera cameraColor = cameraModelToChiselCamera(rgbModel);
								chisel::PinholeCamera cameraDepth = cameraModelToChiselCamera(depthModel);
								cameraColor.SetNearPlane(nearPlaneDist);
								cameraColor.SetFarPlane(farPlaneDist);
								cameraDepth.SetNearPlane(nearPlaneDist);
								cameraDepth.SetFarPlane(farPlaneDist);

								chisel::Transform  pose_rel_to_first_frame = (poses.at(iter->first)*models[0].localTransform()).toEigen3f();
								chiselMap->IntegrateDepthScanColor<float, unsigned char>(projectionIntegrator, depthChisel, pose_rel_to_first_frame, cameraDepth, colorChisel, pose_rel_to_first_frame, cameraColor);
								UDEBUG("");
							}
							else
							{
								_progressDialog->appendText(tr("OpenChisel: Depth and RGB images not found for %1!").arg(iter->first), Qt::darkYellow);
							}
						}
						else
						{
							_progressDialog->appendText(tr("OpenChisel: Invalid camera model for cloud %1! Only single RGB-D camera supported.").arg(iter->first), Qt::darkYellow);
							_progressDialog->setAutoClose(false);
							break;
						}
					}
					else if(!scanInfo.localTransform().isNull())
					{
						chisel::PointCloudPtr chiselCloud = pointCloudRGBToChisel(*iter->second, scanInfo.localTransform().inverse());
						chisel::Transform  pose_rel_to_first_frame = (poses.at(iter->first)*scanInfo.localTransform()).toEigen3f();
						chiselMap->IntegratePointCloud(projectionIntegrator, *chiselCloud, pose_rel_to_first_frame, farPlaneDist);
						UDEBUG("");
					}
					else
					{
						_progressDialog->appendText(tr("OpenChisel: not valid scan info for cloud %1!").arg(iter->first), Qt::darkYellow);
						_progressDialog->setAutoClose(false);
						break;
					}
					chiselMap->UpdateMeshes();
					UDEBUG("");
					_progressDialog->appendText(tr("OpenChisel: Integrated cloud %1 (%2/%3) to TSDF volume").arg(iter->first).arg(cloudsAdded).arg(cloudsWithNormals.size()));

					_progressDialog->incrementStep();
					QApplication::processEvents();
					if(_canceled)
					{
						return false;
					}
				}
			}
			else
#endif

			if(_ui->comboBox_pipeline->currentIndex() == 0)
			{
				if(_ui->comboBox_meshingApproach->currentIndex()==2)
				{
					_progressDialog->appendText(tr("Creating TSDF volume with CPUTSDF... "));
				}
				else
				{
					_progressDialog->appendText(tr("Organized fast mesh... "));
				}
				QApplication::processEvents();
				uSleep(100);
				QApplication::processEvents();

				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				std::vector<pcl::Vertices> mergedPolygons;

				int cloudsAdded = 1;
				for(std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr >::iterator iter=cloudsWithNormals.begin();
					iter!= cloudsWithNormals.end();
					++iter,++cloudsAdded)
				{
					if(iter->second->isOrganized())
					{
						if(iter->second->size())
						{
							Eigen::Vector3f viewpoint(0.0f,0.0f,0.0f);

							std::vector<CameraModel> models;
							std::vector<StereoCameraModel> stereoModels;
							if(cachedSignatures.contains(iter->first))
							{
								const SensorData & data = cachedSignatures.find(iter->first)->sensorData();
								models = data.cameraModels();
								stereoModels = data.stereoCameraModels();
							}
							else if(_dbDriver)
							{
								_dbDriver->getCalibration(iter->first, models, stereoModels);
							}

#ifdef RTABMAP_CPUTSDF
							if(_ui->comboBox_meshingApproach->currentIndex()==2 && _ui->checkBox_assemble->isChecked())
							{
								if(tsdf.get() == 0)
								{
									if(models.size()==1 && models[0].isValidForProjection() && models[0].imageHeight()>0 && models[0].imageWidth()>0)
									{
										float tsdf_size = _ui->doubleSpinBox_cputsdf_size->value();
										float cell_size = _ui->doubleSpinBox_cputsdf_resolution->value();
										int num_random_splits = _ui->spinBox_cputsdf_randomSplit->value();
										int tsdf_res;
										int desired_res = tsdf_size / cell_size;
										// Snap to nearest power of 2;
										int n = 1;
										while (desired_res > n)
										{
											n *= 2;
										}
										tsdf_res = n;
										_progressDialog->appendText(tr("CPU-TSDF: Setting resolution: %1 with grid size %2").arg(tsdf_res).arg(tsdf_size));
										tsdf.reset (new cpu_tsdf::TSDFVolumeOctree);
										tsdf->setGridSize (tsdf_size, tsdf_size, tsdf_size);
										tsdf->setResolution (tsdf_res, tsdf_res, tsdf_res);
										float decimation = float(models[0].imageWidth()) / float(iter->second->width);
										tsdf->setImageSize (models[0].imageWidth()/decimation, models[0].imageHeight()/decimation);
										tsdf->setCameraIntrinsics (models[0].fx()/decimation, models[0].fy()/decimation, models[0].cx()/decimation, models[0].cy()/decimation);
										tsdf->setNumRandomSplts (num_random_splits);
										tsdf->setSensorDistanceBounds (0, 9999);
										tsdf->setIntegrateColor(true);
										tsdf->setDepthTruncationLimits (_ui->doubleSpinBox_cputsdf_tuncPos->value(), _ui->doubleSpinBox_cputsdf_tuncNeg->value());
										tsdf->reset ();
									}
									else
									{
										_progressDialog->appendText(tr("CPU-TSDF: Invalid camera model! Only single RGB-D camera supported."), Qt::darkYellow);
										_progressDialog->setAutoClose(false);
										break;
									}
								}

								if(tsdf.get())
								{
									if(tsdf.get() && models.size() == 1 && !models[0].localTransform().isNull())
									{
										Eigen::Affine3d  pose_rel_to_first_frame = ((poses.begin()->second.inverse() * poses.at(iter->first))*models[0].localTransform()).toEigen3d();
										if(!tsdf->integrateCloud(*util3d::transformPointCloud(iter->second, models[0].localTransform().inverse()), pcl::PointCloud<pcl::Normal>(), pose_rel_to_first_frame))
										{
											_progressDialog->appendText(tr("CPU-TSDF: Failed integrating cloud %1 (%2/%3) to TSDF volume").arg(iter->first).arg(cloudsAdded).arg(cloudsWithNormals.size()));
										}
										else
										{
											_progressDialog->appendText(tr("CPU-TSDF: Integrated cloud %1 (%2/%3) to TSDF volume").arg(iter->first).arg(cloudsAdded).arg(cloudsWithNormals.size()));
										}
									}
								}
							}
							else
#endif
							{
								if((_ui->comboBox_frame->isEnabled() && _ui->comboBox_frame->currentIndex() != 2) ||
									!iter->second->isOrganized())
								{
									if(models.size() && !models[0].localTransform().isNull())
									{
										viewpoint[0] = models[0].localTransform().x();
										viewpoint[1] = models[0].localTransform().y();
										viewpoint[2] = models[0].localTransform().z();
									}
									else if(stereoModels.size() && !stereoModels[0].localTransform().isNull())
									{
										viewpoint[0] = stereoModels[0].localTransform().x();
										viewpoint[1] = stereoModels[0].localTransform().y();
										viewpoint[2] = stereoModels[0].localTransform().z();
									}
								}

								std::vector<pcl::Vertices> polygons = util3d::organizedFastMesh(
										iter->second,
										_ui->doubleSpinBox_mesh_angleTolerance->value()*M_PI/180.0,
										_ui->checkBox_mesh_quad->isEnabled() && _ui->checkBox_mesh_quad->isChecked(),
										_ui->spinBox_mesh_triangleSize->value(),
										viewpoint);

								if(_ui->spinBox_mesh_minClusterSize->value() != 0)
								{
									_progressDialog->appendText(tr("Filter small polygon clusters..."));
									QApplication::processEvents();

									// filter polygons
									std::vector<std::set<int> > neighbors;
									std::vector<std::set<int> > vertexToPolygons;
									util3d::createPolygonIndexes(polygons,
											(int)iter->second->size(),
											neighbors,
											vertexToPolygons);
									std::list<std::list<int> > clusters = util3d::clusterPolygons(
											neighbors,
											_ui->spinBox_mesh_minClusterSize->value()<0?0:_ui->spinBox_mesh_minClusterSize->value());

									std::vector<pcl::Vertices> filteredPolygons(polygons.size());
									if(_ui->spinBox_mesh_minClusterSize->value() < 0)
									{
										// only keep the biggest cluster
										std::list<std::list<int> >::iterator biggestClusterIndex = clusters.end();
										unsigned int biggestClusterSize = 0;
										for(std::list<std::list<int> >::iterator iter=clusters.begin(); iter!=clusters.end(); ++iter)
										{
											if(iter->size() > biggestClusterSize)
											{
												biggestClusterIndex = iter;
												biggestClusterSize = iter->size();
											}
										}
										if(biggestClusterIndex != clusters.end())
										{
											int oi=0;
											for(std::list<int>::iterator jter=biggestClusterIndex->begin(); jter!=biggestClusterIndex->end(); ++jter)
											{
												filteredPolygons[oi++] = polygons.at(*jter);
											}
											filteredPolygons.resize(oi);
										}
									}
									else
									{
										int oi=0;
										for(std::list<std::list<int> >::iterator iter=clusters.begin(); iter!=clusters.end(); ++iter)
										{
											for(std::list<int>::iterator jter=iter->begin(); jter!=iter->end(); ++jter)
											{
												filteredPolygons[oi++] = polygons.at(*jter);
											}
										}
										filteredPolygons.resize(oi);
									}
									int before = (int)polygons.size();
									polygons = filteredPolygons;

									if(polygons.size() == 0)
									{
										std::string msg = uFormat("All %d polygons filtered after polygon cluster filtering. Cluster minimum size is %d.", before, _ui->spinBox_mesh_minClusterSize->value());
										_progressDialog->appendText(msg.c_str(), Qt::darkYellow);
										UWARN(msg.c_str());
									}

									_progressDialog->appendText(tr("Filtered %1 polygons.").arg(before-(int)polygons.size()));
									QApplication::processEvents();
								}

								_progressDialog->appendText(tr("Mesh %1 created with %2 polygons (%3/%4).").arg(iter->first).arg(polygons.size()).arg(cloudsAdded).arg(cloudsWithNormals.size()));

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

									organizedIndices.insert(std::make_pair(iter->first, denseToOrganizedIndices));
									organizedCloudSizes.insert(std::make_pair(iter->first, cv::Size(iter->second->width, iter->second->height)));

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
						}
						else
						{
							_progressDialog->appendText(tr("Mesh %1 not created (no valid points) (%2/%3).").arg(iter->first).arg(cloudsAdded).arg(cloudsWithNormals.size()));
						}
					}
					else
					{
						int weight = 0;
						if(cachedSignatures.contains(iter->first))
						{
							const Signature & s = cachedSignatures.find(iter->first).value();
							weight = s.getWeight();
						}
						else if(_dbDriver)
						{
							_dbDriver->getWeight(iter->first, weight);
						}
						if(weight>=0) // don't show error for intermediate nodes
						{
							_progressDialog->appendText(tr("Mesh %1 not created (cloud is not organized). You may want to check cloud regeneration option (%2/%3).").arg(iter->first).arg(cloudsAdded).arg(cloudsWithNormals.size()));
						}
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

					meshes.insert(std::make_pair(0, mesh));

					_progressDialog->incrementStep();
					QApplication::processEvents();
				}
			}
			else // dense pipeline
			{
				if(_ui->comboBox_meshingApproach->currentIndex() == 0)
				{
					_progressDialog->appendText(tr("Greedy projection triangulation... [radius=%1m]").arg(_ui->doubleSpinBox_gp3Radius->value()));
				}
				else
				{
					_progressDialog->appendText(tr("Poisson surface reconstruction..."));
				}
				QApplication::processEvents();
				uSleep(100);
				QApplication::processEvents();

				int cloudsAdded=1;
				for(std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>::iterator iter=cloudsWithNormals.begin();
					iter!= cloudsWithNormals.end();
					++iter,++cloudsAdded)
				{
					pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
					if(_ui->comboBox_meshingApproach->currentIndex() == 0)
					{
						mesh = util3d::createMesh(
								iter->second,
								_ui->doubleSpinBox_gp3Radius->value(),
								_ui->doubleSpinBox_gp3Mu->value());
					}
					else
					{
						pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
						poisson.setOutputPolygons(_ui->checkBox_poisson_outputPolygons->isEnabled()?_ui->checkBox_poisson_outputPolygons->isChecked():false);
						poisson.setManifold(_ui->checkBox_poisson_manifold->isChecked());
						poisson.setSamplesPerNode(_ui->doubleSpinBox_poisson_samples->value());
						int depth = _ui->spinBox_poisson_depth->value();
						if(depth == 0)
						{
							Eigen::Vector4f min,max;
							pcl::getMinMax3D(*iter->second, min, max);
							float mapLength = uMax3(max[0]-min[0], max[1]-min[1], max[2]-min[2]);
							depth = 12;
							for(int i=6; i<12; ++i)
							{
								if(mapLength/float(1<<i) < _ui->doubleSpinBox_poisson_targetPolygonSize->value())
								{
									depth = i;
									break;
								}
							}
							_progressDialog->appendText(tr("Poisson depth resolution chosen is %1, map size (m) = %2x%3x%4")
									.arg(depth)
									.arg(int(max[0]-min[0]))
									.arg(int(max[1]-min[1]))
									.arg(int(max[2]-min[2])));
							QApplication::processEvents();
							uSleep(100);
							QApplication::processEvents();
						}
						poisson.setDepth(depth);
						poisson.setIsoDivide(_ui->spinBox_poisson_iso->value());
						poisson.setSolverDivide(_ui->spinBox_poisson_solver->value());
						poisson.setMinDepth(_ui->spinBox_poisson_minDepth->value());
						poisson.setPointWeight(_ui->doubleSpinBox_poisson_pointWeight->value());
						poisson.setScale(_ui->doubleSpinBox_poisson_scale->value());
						poisson.setInputCloud(iter->second);
						poisson.reconstruct(*mesh);
					}

					_progressDialog->appendText(tr("Mesh %1 created with %2 polygons (%3/%4).").arg(iter->first).arg(mesh->polygons.size()).arg(cloudsAdded).arg(cloudsWithNormals.size()));
					QApplication::processEvents();

					if(mesh->polygons.size()>0)
					{
						TexturingState texturingState(_progressDialog, false);

						if(!_ui->checkBox_fromDepth->isChecked() && !_scansHaveRGB)
						{
							// When laser scans are exported, convert Intensity to GrayScale
							int maxIntensity = 1;
							// first: get max intensity
							for(size_t i=0; i<iter->second->size(); ++i)
							{
								int intensity =
										int(iter->second->points[i].r) |
										int(iter->second->points[i].g) << 8 |
										int(iter->second->points[i].b) << 16 |
										int(iter->second->points[i].a) << 24;
								if(intensity > maxIntensity)
								{
									maxIntensity = intensity;
								}
							}
							// second: convert to grayscale
							for(size_t i=0; i<iter->second->size(); ++i)
							{
								int intensity =
										int(iter->second->points[i].r) |
										int(iter->second->points[i].g) << 8 |
										int(iter->second->points[i].b) << 16 |
										int(iter->second->points[i].a) << 24;
								intensity = intensity*255/maxIntensity;
								iter->second->points[i].r = (unsigned char)intensity;
								iter->second->points[i].g = (unsigned char)intensity;
								iter->second->points[i].b = (unsigned char)intensity;
								iter->second->points[i].a = (unsigned char)255;
							}
						}

						util3d::denseMeshPostProcessing<pcl::PointXYZRGBNormal>(
								mesh,
								_ui->doubleSpinBox_meshDecimationFactor->isEnabled()?(float)_ui->doubleSpinBox_meshDecimationFactor->value():0.0f,
								_ui->spinBox_meshMaxPolygons->isEnabled()?_ui->spinBox_meshMaxPolygons->value():0,
								iter->second,
								(float)_ui->doubleSpinBox_transferColorRadius->value(),
								!(_ui->checkBox_textureMapping->isEnabled() && _ui->checkBox_textureMapping->isChecked()),
								_ui->checkBox_cleanMesh->isChecked(),
								_ui->spinBox_mesh_minClusterSize->value(),
								&texturingState);
						meshes.insert(std::make_pair(iter->first, mesh));
					}
					else
					{
						_progressDialog->appendText(tr("No polygons created for cloud %1!").arg(iter->first), Qt::darkYellow);
						_progressDialog->setAutoClose(false);
					}

					_progressDialog->incrementStep(_ui->checkBox_assemble->isChecked()?poses.size():1);
					QApplication::processEvents();
					if(_canceled)
					{
						return false;
					}
				}
			}
		}
		else if(_ui->checkBox_meshing->isChecked())
		{
			std::string msg = uFormat("Some clouds are 2D laser scans. Meshing can be done only from RGB-D clouds or 3D laser scans.");
			_progressDialog->appendText(msg.c_str(), Qt::darkYellow);
			UWARN(msg.c_str());
		}
		else if(_ui->checkBox_cameraProjection->isEnabled() &&
				_ui->checkBox_cameraProjection->isChecked() &&
				cloudsWithNormals.size()==1 && cloudsWithNormals.begin()->first==0) // only for assembled point cloud
		{
			_progressDialog->appendText(tr("Camera projection..."));
			QApplication::processEvents();
			uSleep(100);
			QApplication::processEvents();

			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr assembledCloud = cloudsWithNormals.begin()->second;

			std::map<int, Transform> cameraPoses;
			std::map<int, std::vector<CameraModel> > cameraModels;
			for(std::map<int, Transform>::const_iterator iter=poses.lower_bound(0); iter!=poses.end(); ++iter)
			{
				std::vector<CameraModel> models;
				std::vector<StereoCameraModel> stereoModels;
				if(cachedSignatures.contains(iter->first))
				{
					const SensorData & data = cachedSignatures.find(iter->first)->sensorData();
					models = data.cameraModels();
					stereoModels = data.stereoCameraModels();
				}
				else if(_dbDriver)
				{
					_dbDriver->getCalibration(iter->first, models, stereoModels);
				}

				if(stereoModels.size())
				{
					models.clear();
					for(size_t i=0; i<stereoModels.size(); ++i)
					{
						models.push_back(stereoModels[i].left());
					}
				}

				if(models.size() == 0 || !models[0].isValidForProjection())
				{
					models.clear();
				}

				if(!models.empty() && models[0].imageWidth() != 0 && models[0].imageHeight() != 0)
				{
					cameraPoses.insert(std::make_pair(iter->first, iter->second));
					cameraModels.insert(std::make_pair(iter->first, models));
				}
				else
				{
					UWARN("%d node has invalid camera models, ignoring it.", iter->first);
				}
			}
			if(!cameraPoses.empty())
			{
				TexturingState texturingState(_progressDialog, true);
				_progressDialog->setMaximumSteps(_progressDialog->maximumSteps() + assembledCloud->size()/10000+1 + cameraPoses.size());
				std::vector<float> roiRatios;
				QStringList strings = _ui->lineEdit_camProjRoiRatios->text().split(' ');
				if(!_ui->lineEdit_meshingTextureRoiRatios->text().isEmpty())
				{
					if(strings.size()==4)
					{
						roiRatios.resize(4);
						roiRatios[0]=strings[0].toDouble();
						roiRatios[1]=strings[1].toDouble();
						roiRatios[2]=strings[2].toDouble();
						roiRatios[3]=strings[3].toDouble();
						if(!(roiRatios[0]>=0.0f && roiRatios[0]<=1.0f &&
							roiRatios[1]>=0.0f && roiRatios[1]<=1.0f &&
							roiRatios[2]>=0.0f && roiRatios[2]<=1.0f &&
							roiRatios[3]>=0.0f && roiRatios[3]<=1.0f))
						{
							roiRatios.clear();
						}
					}
					if(roiRatios.empty())
					{
						QString msg = tr("Wrong ROI format. Region of Interest (ROI) must "
								"have 4 values [left right top bottom] between 0 and 1 "
								"separated by space (%1), ignoring it for projecting cameras...")
										.arg(_ui->lineEdit_camProjRoiRatios->text());
						UWARN(msg.toStdString().c_str());
						_progressDialog->appendText(msg, Qt::darkYellow);
						_progressDialog->setAutoClose(false);
					}
				}

				std::map<int, std::vector<rtabmap::CameraModel> > cameraModelsProj;
				if(_ui->spinBox_camProjDecimation->value()>1)
				{
					for(std::map<int, std::vector<rtabmap::CameraModel> >::iterator iter=cameraModels.begin();
							iter!=cameraModels.end();
							++iter)
					{
						std::vector<rtabmap::CameraModel> models;
						for(size_t i=0; i<iter->second.size(); ++i)
						{
							models.push_back(iter->second[i].scaled(1.0/double(_ui->spinBox_camProjDecimation->value())));
						}
						cameraModelsProj.insert(std::make_pair(iter->first, models));
					}
				}
				else
				{
					cameraModelsProj = cameraModels;
				}

				cv::Mat projMask;
				if(!_ui->lineEdit_camProjMaskFilePath->text().isEmpty())
				{
					projMask = cv::imread(_ui->lineEdit_camProjMaskFilePath->text().toStdString(), cv::IMREAD_GRAYSCALE);
					if(_ui->spinBox_camProjDecimation->value()>1)
					{
						cv::Mat out = projMask;
						cv::resize(projMask, out, cv::Size(), 1.0f/float(_ui->spinBox_camProjDecimation->value()), 1.0f/float(_ui->spinBox_camProjDecimation->value()), cv::INTER_NEAREST);
						projMask = out;
					}
				}

				std::vector<std::pair< std::pair<int, int>, pcl::PointXY> > pointToPixel;
				pointToPixel = util3d::projectCloudToCameras(
						*assembledCloud,
						cameraPoses,
						cameraModelsProj,
						_ui->doubleSpinBox_camProjMaxDistance->value(),
						_ui->doubleSpinBox_camProjMaxAngle->value()*M_PI/180.0,
						roiRatios,
						projMask,
						_ui->checkBox_camProjDistanceToCamPolicy->isChecked(),
						&texturingState);

				if(texturingState.isCanceled())
				{
					return false;
				}

				// color the cloud
				UASSERT(pointToPixel.empty() || pointToPixel.size() == assembledCloud->size());
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr assembledCloudValidPoints;
				if(!_ui->checkBox_camProjKeepPointsNotSeenByCameras->isChecked())
				{
					assembledCloudValidPoints.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
					assembledCloudValidPoints->resize(assembledCloud->size());
				}
				if(_ui->comboBox_camProjExportCamera->isEnabled() &&
				   _ui->comboBox_camProjExportCamera->currentIndex()>0)
				{
					textureVertexToPixels.resize(assembledCloud->size());
				}

				if(_ui->checkBox_camProjRecolorPoints->isChecked())
				{
					int imagesDone = 1;
					for(std::map<int, rtabmap::Transform>::iterator iter=cameraPoses.begin(); iter!=cameraPoses.end() && !_canceled; ++iter)
					{
						int nodeID = iter->first;

						cv::Mat image;
						if(cachedSignatures.contains(nodeID) && !cachedSignatures.value(nodeID).sensorData().imageCompressed().empty())
						{
							cachedSignatures.value(nodeID).sensorData().uncompressDataConst(&image, 0);
						}
						else if(_dbDriver)
						{
							SensorData data;
							_dbDriver->getNodeData(nodeID, data, true, false, false, false);
							data.uncompressDataConst(&image, 0);
						}
						if(!image.empty())
						{

							if(_ui->spinBox_camProjDecimation->value()>1)
							{
								image = util2d::decimate(image, _ui->spinBox_camProjDecimation->value());
							}

							UASSERT(cameraModelsProj.find(nodeID) != cameraModelsProj.end());
							int modelsSize = cameraModelsProj.at(nodeID).size();
							for(size_t i=0; i<pointToPixel.size(); ++i)
							{
								int cameraIndex = pointToPixel[i].first.second;
								if(nodeID == pointToPixel[i].first.first && cameraIndex>=0)
								{
									int subImageWidth = image.cols / modelsSize;
									cv::Mat subImage = image(cv::Range::all(), cv::Range(cameraIndex*subImageWidth, (cameraIndex+1)*subImageWidth));

									int x = pointToPixel[i].second.x * (float)subImage.cols;
									int y = pointToPixel[i].second.y * (float)subImage.rows;
									UASSERT(x>=0 && x<subImage.cols);
									UASSERT(y>=0 && y<subImage.rows);

									pcl::PointXYZRGBNormal & pt = assembledCloud->at(i);
									if(subImage.type()==CV_8UC3)
									{
										cv::Vec3b bgr = subImage.at<cv::Vec3b>(y, x);
										pt.b = bgr[0];
										pt.g = bgr[1];
										pt.r = bgr[2];
									}
									else
									{
										UASSERT(subImage.type()==CV_8UC1);
										pt.r = pt.g = pt.b = subImage.at<unsigned char>(pointToPixel[i].second.y * subImage.rows, pointToPixel[i].second.x * subImage.cols);
									}
								}
							}
						}
						QString msg = tr("Processed %1/%2 images").arg(imagesDone++).arg(cameraPoses.size());
						UINFO(msg.toStdString().c_str());
						_progressDialog->appendText(msg);
						QApplication::processEvents();
					}
				}

				pcl::IndicesPtr validIndices(new std::vector<int>(pointToPixel.size()));
				size_t oi = 0;
				for(size_t i=0; i<pointToPixel.size() && !_canceled; ++i)
				{
					pcl::PointXYZRGBNormal & pt = assembledCloud->at(i);
					if(pointToPixel[i].first.first <=0)
					{
						if(_ui->checkBox_camProjRecolorPoints->isChecked() && !_ui->checkBox_fromDepth->isChecked() && !_scansHaveRGB)
						{
							pt.r = 255;
							pt.g = 0;
							pt.b = 0;
							pt.a = 255;
						}
					}
					else
					{
						int nodeID = pointToPixel[i].first.first;
						int cameraIndex = pointToPixel[i].first.second;
						int exportedId = nodeID;
						if(_ui->comboBox_camProjExportCamera->currentIndex() == 2)
						{
							exportedId = cameraIndex+1;
						}
						else if(_ui->comboBox_camProjExportCamera->currentIndex() == 3)
						{
							UASSERT_MSG(cameraIndex<10, "Exporting cam id as NodeId+CamIndex is only supported when the number of cameras per node < 10.");
							exportedId = nodeID*10 + cameraIndex;
						}

						if(assembledCloudValidPoints.get())
						{
							if(!textureVertexToPixels.empty())
							{
								textureVertexToPixels[oi].insert(std::make_pair(exportedId, pointToPixel[i].second));
							}
							assembledCloudValidPoints->at(oi++) = pt;
						}
						else if(!textureVertexToPixels.empty())
						{
							textureVertexToPixels[i].insert(std::make_pair(exportedId, pointToPixel[i].second));
						}
					}
				}

				if(assembledCloudValidPoints.get())
				{
					assembledCloudValidPoints->resize(oi);
					cloudsWithNormals.begin()->second = assembledCloudValidPoints;

					if(!textureVertexToPixels.empty())
					{
						textureVertexToPixels.resize(oi);
					}
				}

				if(_canceled)
				{
					return false;
				}
			}
		}

		UDEBUG("");
#ifdef RTABMAP_CPUTSDF
		if(tsdf.get())
		{
			_progressDialog->appendText(tr("CPU-TSDF: Creating mesh from TSDF volume..."));
			QApplication::processEvents();
			uSleep(100);
			QApplication::processEvents();

			cpu_tsdf::MarchingCubesTSDFOctree mc;
			mc.setMinWeight (_ui->doubleSpinBox_cputsdf_minWeight->value());
			mc.setInputTSDF (tsdf);
			mc.setColorByRGB (true);
			pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
			mc.reconstruct (*mesh);
			_progressDialog->appendText(tr("CPU-TSDF: Creating mesh from TSDF volume...done! %1 polygons").arg(mesh->polygons.size()));
			meshes.clear();

			if(mesh->polygons.size()>0)
			{

				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				pcl::fromPCLPointCloud2 (mesh->cloud, *vertices);

				if(_ui->doubleSpinBox_cputsdf_flattenRadius->value()>0.0)
				{
					_progressDialog->appendText(tr("CPU-TSDF: Flattening mesh (radius=%1)...").arg(_ui->doubleSpinBox_cputsdf_flattenRadius->value()));
					QApplication::processEvents();
					uSleep(100);
					QApplication::processEvents();

					float min_dist = _ui->doubleSpinBox_cputsdf_flattenRadius->value();
					pcl::search::KdTree<pcl::PointXYZRGBNormal> vert_tree (true);
					vert_tree.setInputCloud (vertices);
					// Find duplicates
					std::vector<int> vertex_remap (vertices->size (), -1);
					int idx = 0;
					std::vector<int> neighbors;
					std::vector<float> dists;
					pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr vertices_new(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
					vertices_new->resize(vertices->size ());
					for (size_t i = 0; i < vertices->size (); i++)
					{
						if (vertex_remap[i] >= 0)
							continue;
						vertex_remap[i] = idx;
						vert_tree.radiusSearch (i, min_dist, neighbors, dists);
						for (size_t j = 1; j < neighbors.size (); j++)
						{
							if (dists[j] < min_dist)
								vertex_remap[neighbors[j]] = idx;
						}
						vertices_new->at(idx++) = vertices->at (i);
					}
					vertices_new->resize(idx);
					std::vector<size_t> faces_to_remove;
					size_t face_idx = 0;
					for (size_t i = 0; i < mesh->polygons.size (); i++)
					{
						pcl::Vertices &v = mesh->polygons[i];
						for (size_t j = 0; j < v.vertices.size (); j++)
						{
							v.vertices[j] = vertex_remap[v.vertices[j]];
						}
						if (!(v.vertices[0] == v.vertices[1] || v.vertices[1] == v.vertices[2] || v.vertices[2] == v.vertices[0]))
						{
							mesh->polygons[face_idx++] = mesh->polygons[i];
						}
					}
					mesh->polygons.resize (face_idx);
					pcl::toPCLPointCloud2 (*vertices_new, mesh->cloud);
					vertices = vertices_new;
				}

				pcl::fromPCLPointCloud2(mesh->cloud, *vertices);
				TexturingState texturingState(_progressDialog, false);
				util3d::denseMeshPostProcessing<pcl::PointXYZRGBNormal>(
						mesh,
						_ui->doubleSpinBox_meshDecimationFactor->isEnabled()?(float)_ui->doubleSpinBox_meshDecimationFactor->value():0.0f,
						_ui->spinBox_meshMaxPolygons->isEnabled()?_ui->spinBox_meshMaxPolygons->value():0,
						vertices,
						(float)_ui->doubleSpinBox_transferColorRadius->value(),
						!(_ui->checkBox_textureMapping->isEnabled() && _ui->checkBox_textureMapping->isChecked()),
						_ui->checkBox_cleanMesh->isChecked(),
						_ui->spinBox_mesh_minClusterSize->value(),
						&texturingState);
				meshes.insert(std::make_pair(0, mesh));
			}
			else
			{
				_progressDialog->appendText(tr("No polygons created TSDF volume!"), Qt::darkYellow);
				_progressDialog->setAutoClose(false);
			}
		}
#endif
#ifdef RTABMAP_OPENCHISEL
		if(chiselMap.get())
		{
			_progressDialog->appendText(tr("OpenChisel: Creating mesh from TSDF volume..."));
			QApplication::processEvents();
			uSleep(100);
			QApplication::processEvents();

			const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();
			pcl::PolygonMesh::Ptr mesh = chiselToPolygonMesh(meshMap);

			// To debug...
			//std::string filePly = _workingDirectory.toStdString()+"/"+"chisel.ply";
			//chiselMap->SaveAllMeshesToPLY(filePly);
			//UWARN("Saved %s", filePly.c_str());

			_progressDialog->appendText(tr("OpenChisel: Creating mesh from TSDF volume...done! %1 polygons").arg(mesh->polygons.size()));

			meshes.clear();
			if(mesh->polygons.size()>0)
			{
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				pcl::fromPCLPointCloud2(mesh->cloud, *mergedClouds);
				if(_ui->checkBox_openchisel_mergeVertices->isChecked())
				{
					_progressDialog->appendText(tr("Filtering assembled mesh for close vertices (points=%1, polygons=%2)...").arg(mergedClouds->size()).arg(mesh->polygons.size()));
					QApplication::processEvents();

					mesh->polygons = util3d::filterCloseVerticesFromMesh(
							mergedClouds,
							mesh->polygons,
							_ui->doubleSpinBox_voxelSize_assembled->value()/2.0,
							M_PI/4,
							true);

					// filter invalid polygons
					unsigned int count = mesh->polygons.size();
					mesh->polygons = util3d::filterInvalidPolygons(mesh->polygons);
					_progressDialog->appendText(tr("Filtered %1 invalid polygons.").arg(count-mesh->polygons.size()));
					QApplication::processEvents();

					// filter not used vertices
					pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
					std::vector<pcl::Vertices> filteredPolygons;
					count = mergedClouds->size();
					util3d::filterNotUsedVerticesFromMesh(*mergedClouds, mesh->polygons, *filteredCloud, filteredPolygons);
					mergedClouds = filteredCloud;
					pcl::toPCLPointCloud2(*mergedClouds, mesh->cloud);
					mesh->polygons = filteredPolygons;
					_progressDialog->appendText(tr("Filtered %1 duplicate vertices.").arg(count-mergedClouds->size()));
					QApplication::processEvents();
				}
				TexturingState texturingState(_progressDialog, false);
				util3d::denseMeshPostProcessing<pcl::PointXYZRGBNormal>(
						mesh,
						_ui->doubleSpinBox_meshDecimationFactor->isEnabled()?(float)_ui->doubleSpinBox_meshDecimationFactor->value():0.0f,
						_ui->spinBox_meshMaxPolygons->isEnabled()?_ui->spinBox_meshMaxPolygons->value():0,
						mergedClouds,
						(float)_ui->doubleSpinBox_transferColorRadius->value(),
						!(_ui->checkBox_textureMapping->isEnabled() && _ui->checkBox_textureMapping->isChecked()),
						_ui->checkBox_cleanMesh->isChecked(),
						_ui->spinBox_mesh_minClusterSize->value(),
						&texturingState);
				meshes.insert(std::make_pair(0, mesh));
			}
			else
			{
				_progressDialog->appendText(tr("No polygons created TSDF volume!"), Qt::darkYellow);
				_progressDialog->setAutoClose(false);
			}
		}
#endif

		UDEBUG("");
		if(_canceled)
		{
			return false;
		}

		// texture mesh
		UDEBUG("texture mapping=%d", _ui->checkBox_textureMapping->isEnabled() && _ui->checkBox_textureMapping->isChecked()?1:0);
		if(!has2dScans && !meshes.empty() && _ui->checkBox_textureMapping->isEnabled() && _ui->checkBox_textureMapping->isChecked())
		{
			_progressDialog->appendText(tr("Texturing..."));
			QApplication::processEvents();
			uSleep(100);
			QApplication::processEvents();

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
				std::map<int, std::vector<CameraModel> > cameraModels;
				std::map<int, cv::Mat > cameraDepths;
				int ignoredCameras = 0;
				for(std::map<int, Transform>::iterator jter=cameras.begin(); jter!=cameras.end(); ++jter)
				{
					if(validCameras.find(jter->first) != validCameras.end())
					{
						std::vector<CameraModel> models;
						std::vector<StereoCameraModel> stereoModels;
						bool cacheHasCompressedImage = false;
						if(cachedSignatures.contains(jter->first))
						{
							const SensorData & data = cachedSignatures.find(jter->first)->sensorData();
							models = data.cameraModels();
							stereoModels = data.stereoCameraModels();
							cacheHasCompressedImage = !data.imageCompressed().empty();
						}
						else if(_dbDriver)
						{
							_dbDriver->getCalibration(jter->first, models, stereoModels);
						}

						bool stereo=false;
						if(stereoModels.size())
						{
							stereo = true;
							models.clear();
							for(size_t i=0; i<stereoModels.size(); ++i)
							{
								models.push_back(stereoModels[i].left());
							}
						}

						if(models.size() == 0 || !models[0].isValidForProjection())
						{
							models.clear();
						}

						if(!jter->second.isNull() && models.size())
						{
							cv::Mat depth;
							bool blurryImage = false;
							bool getDepth = !stereo && _ui->doubleSpinBox_meshingTextureMaxDepthError->value() >= 0.0f;
							cv::Mat img;
							std::vector<float> velocity;
							if(models[0].imageWidth() == 0 || models[0].imageHeight() == 0)
							{
								// we are using an old database format (image size not saved in calibrations), we should
								// uncompress images to get their size
								if(cacheHasCompressedImage)
								{
									cachedSignatures.find(jter->first)->sensorData().uncompressDataConst(&img, getDepth?&depth:0);
									velocity = cachedSignatures.find(jter->first)->getVelocity();
								}
								else if(_dbDriver)
								{
									SensorData data;
									_dbDriver->getNodeData(jter->first, data, true, false, false, false);
									data.uncompressDataConst(&img, getDepth?&depth:0);

									if(_ui->checkBox_cameraFilter->isChecked() &&
										(_ui->doubleSpinBox_cameraFilterVel->value()>0.0 || _ui->doubleSpinBox_cameraFilterVelRad->value()>0.0))
									{
										Transform p,gt;
										int m,w;
										std::string l;
										double s;
										GPS gps;
										EnvSensors sensors;
										_dbDriver->getNodeInfo(jter->first, p, m, w, l, s, gt, velocity, gps, sensors);
									}
								}
								cv::Size imageSize = img.size();
								imageSize.width /= models.size();
								for(unsigned int i=0; i<models.size(); ++i)
								{
									models[i].setImageSize(imageSize);
								}
							}
							else
							{
								bool getImg = _ui->checkBox_cameraFilter->isChecked() && _ui->doubleSpinBox_laplacianVariance->value()>0.0;
								// get just the depth
								if(cacheHasCompressedImage)
								{
									cachedSignatures.find(jter->first)->sensorData().uncompressDataConst(getImg?&img:0, getDepth?&depth:0);
									velocity = cachedSignatures.find(jter->first)->getVelocity();
								}
								else if(_dbDriver)
								{
									SensorData data;
									_dbDriver->getNodeData(jter->first, data, true, false, false, false);
									data.uncompressDataConst(getImg?&img:0, getDepth?&depth:0);

									if(_ui->checkBox_cameraFilter->isChecked() &&
										(_ui->doubleSpinBox_cameraFilterVel->value()>0.0 || _ui->doubleSpinBox_cameraFilterVelRad->value()>0.0))
									{
										Transform p,gt;
										int m,w;
										std::string l;
										double s;
										GPS gps;
										EnvSensors sensors;
										_dbDriver->getNodeInfo(jter->first, p, m, w, l, s, gt, velocity, gps, sensors);
									}
								}
							}
							if(_ui->checkBox_cameraFilter->isChecked())
							{
								std::string msg;
								if(!blurryImage &&
									(_ui->doubleSpinBox_cameraFilterVel->value()>0.0 || _ui->doubleSpinBox_cameraFilterVelRad->value()>0.0))
								{
									if(velocity.size() == 6)
									{
										float transVel = uMax3(fabs(velocity[0]), fabs(velocity[1]), fabs(velocity[2]));
										float rotVel = uMax3(fabs(velocity[3]), fabs(velocity[4]), fabs(velocity[5]));
										if(_ui->doubleSpinBox_cameraFilterVel->value()>0.0 && transVel > _ui->doubleSpinBox_cameraFilterVel->value())
										{
											msg = uFormat("Fast motion detected for camera %d (speed=%f m/s > thr=%f m/s), camera is ignored for texturing.", jter->first, transVel, _ui->doubleSpinBox_cameraFilterVel->value());
											blurryImage = true;
										}
										else if(_ui->doubleSpinBox_cameraFilterVelRad->value()>0.0 && rotVel > _ui->doubleSpinBox_cameraFilterVelRad->value())
										{
											msg = uFormat("Fast motion detected for camera %d (speed=%f rad/s > thr=%f rad/s), camera is ignored for texturing.", jter->first, rotVel, _ui->doubleSpinBox_cameraFilterVelRad->value());
											blurryImage = true;
										}
									}
									else
									{
										UWARN("Camera motion filtering is set, but velocity of camera %d is not available.", jter->first);
									}
								}

								if(!blurryImage && !img.empty() && _ui->doubleSpinBox_laplacianVariance->value()>0.0)
								{
									cv::Mat imgLaplacian;
									cv::Laplacian(img, imgLaplacian, CV_16S);
									cv::Mat m, s;
									cv::meanStdDev(imgLaplacian, m, s);
									double stddev_pxl = s.at<double>(0);
									double var = stddev_pxl*stddev_pxl;
									if(var < _ui->doubleSpinBox_laplacianVariance->value())
									{
										blurryImage = true;
										msg = uFormat("Camera's image %d is detected as blurry (var=%f, thr=%f), camera is ignored for texturing.", jter->first, var, _ui->doubleSpinBox_laplacianVariance->value());
									}
								}
								if(blurryImage)
								{
									_progressDialog->appendText(msg.c_str());
									QApplication::processEvents();
									++ignoredCameras;
								}
							}

							if(!blurryImage && models[0].imageWidth() != 0 && models[0].imageHeight() != 0)
							{
								cameraPoses.insert(std::make_pair(jter->first, jter->second));
								cameraModels.insert(std::make_pair(jter->first, models));
								if(!depth.empty())
								{
									cameraDepths.insert(std::make_pair(jter->first, depth));
								}
							}
						}
					}
				}
				if(ignoredCameras > (int)validCameras.size()/2)
				{
					std::string msg = uFormat("More than 50%% of the cameras (%d/%d) have been filtered for "
							"too fast motion and/or blur level. You may adjust the corresponding thresholds.",
							ignoredCameras, (int)validCameras.size());
					UWARN(msg.c_str());
					_progressDialog->appendText(msg.c_str(), Qt::darkYellow);
					_progressDialog->setAutoClose(false);
					QApplication::processEvents();
				}

				if(cameraPoses.size() && iter->second->polygons.size())
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

							//tex_coordinates should be linked to polygon vertices
							int polygonSize = (int)textureMesh->tex_polygons[0][0].vertices.size();
							textureMesh->tex_coordinates[0].resize(polygonSize*textureMesh->tex_polygons[0].size());
							for(unsigned int i=0; i<textureMesh->tex_polygons[0].size(); ++i)
							{
								const pcl::Vertices & vertices = textureMesh->tex_polygons[0][i];
								UASSERT(polygonSize == (int)vertices.vertices.size());
								for(int k=0; k<polygonSize; ++k)
								{
									//uv
									UASSERT((int)vertices.vertices[k] < (int)oter->second.size());
									int originalVertex = oter->second[vertices.vertices[k]];
									textureMesh->tex_coordinates[0][i*polygonSize+k] = Eigen::Vector2f(
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

							mesh_material.tex_file = uFormat("%d", iter->first);
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

						if(cameraPoses.size() && _ui->checkBox_cameraFilter->isChecked())
						{
							int before = (int)cameraPoses.size();
							cameraPoses = graph::radiusPosesFiltering(cameraPoses,
									_ui->doubleSpinBox_cameraFilterRadius->value(),
									_ui->doubleSpinBox_cameraFilterAngle->value());
							for(std::map<int, std::vector<CameraModel> >::iterator modelIter = cameraModels.begin(); modelIter!=cameraModels.end();)
							{
								if(cameraPoses.find(modelIter->first)==cameraPoses.end())
								{
									cameraModels.erase(modelIter++);
									cameraDepths.erase(modelIter->first);
								}
								else
								{
									++modelIter;
								}
							}
							_progressDialog->appendText(tr("Camera filtering: keeping %1/%2/%3 cameras for texturing.").arg(cameraPoses.size()).arg(before).arg(validCameras.size()));
							QApplication::processEvents();
							uSleep(100);
							QApplication::processEvents();
						}

						if(_canceled)
						{
							return false;
						}

						TexturingState texturingState(_progressDialog, true);
						_progressDialog->setMaximumSteps(_progressDialog->maximumSteps()+iter->second->polygons.size()/10000+1);
						if(cameraModels.size() && cameraModels.begin()->second.size()>1)
						{
							_progressDialog->setMaximumSteps(_progressDialog->maximumSteps()+cameraModels.size()*(cameraModels.begin()->second.size()-1));
						}

						std::vector<float> roiRatios;
						QStringList strings = _ui->lineEdit_meshingTextureRoiRatios->text().split(' ');
						if(!_ui->lineEdit_meshingTextureRoiRatios->text().isEmpty())
						{
							if(strings.size()==4)
							{
								roiRatios.resize(4);
								roiRatios[0]=strings[0].toDouble();
								roiRatios[1]=strings[1].toDouble();
								roiRatios[2]=strings[2].toDouble();
								roiRatios[3]=strings[3].toDouble();
								if(!(roiRatios[0]>=0.0f && roiRatios[0]<=1.0f &&
									roiRatios[1]>=0.0f && roiRatios[1]<=1.0f &&
									roiRatios[2]>=0.0f && roiRatios[2]<=1.0f &&
									roiRatios[3]>=0.0f && roiRatios[3]<=1.0f))
								{
									roiRatios.clear();
								}
							}
							if(roiRatios.empty())
							{
								QString msg = tr("Wrong ROI format. Region of Interest (ROI) must have 4 "
										"values [left right top bottom] between 0 and 1 "
										"separated by space (%1), ignoring it for texturing...")
												.arg(_ui->lineEdit_meshingTextureRoiRatios->text());
								UWARN(msg.toStdString().c_str());
								_progressDialog->appendText(msg, Qt::darkYellow);
								_progressDialog->setAutoClose(false);
							}
						}

						textureMesh = util3d::createTextureMesh(
								iter->second,
								cameraPoses,
								cameraModels,
								cameraDepths,
								_ui->doubleSpinBox_meshingTextureMaxDistance->value(),
								_ui->doubleSpinBox_meshingTextureMaxDepthError->value(),
								_ui->doubleSpinBox_meshingTextureMaxAngle->value()*M_PI/180.0,
								_ui->spinBox_mesh_minTextureClusterSize->value(),
								roiRatios,
								&texturingState,
								cameraPoses.size()>1?&textureVertexToPixels:0, // only get vertexToPixels if merged clouds with multi textures
								_ui->checkBox_distanceToCamPolicy->isChecked());

						if(_canceled)
						{
							return false;
						}

						// Remove occluded polygons (polygons with no texture)
						if(_ui->checkBox_cleanMesh->isChecked())
						{
							unsigned int totalSize = 0;
							for(unsigned int t=0; t<textureMesh->tex_polygons.size(); ++t)
							{
								totalSize+=textureMesh->tex_polygons[t].size();
							}

							util3d::cleanTextureMesh(*textureMesh, _ui->spinBox_mesh_minClusterSize->value());

							unsigned int totalSizeAfter = 0;
							for(unsigned int t=0; t<textureMesh->tex_polygons.size(); ++t)
							{
								totalSizeAfter+=textureMesh->tex_polygons[t].size();
							}
							_progressDialog->appendText(tr("Cleaned texture mesh: %1 -> %2 polygons").arg(totalSize).arg(totalSizeAfter));
						}
					}

					textureMeshes.insert(std::make_pair(iter->first, textureMesh));
				}
				else if(cameraPoses.size() == 0)
				{
					_progressDialog->appendText(tr("No cameras from %1 poses with valid calibration found!").arg(poses.size()), Qt::darkYellow);
					_progressDialog->setAutoClose(false);
					UWARN("No camera poses!?");
				}
				else
				{
					_progressDialog->appendText(tr("No polygons!?"), Qt::darkYellow);
					_progressDialog->setAutoClose(false);
					UWARN("No polygons!");
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
		const std::map<int, LaserScan> & cachedScans,
		const ParametersMap & parameters,
		bool & has2dScans,
		bool & scansHaveRGB) const
{
	scansHaveRGB = false;
	has2dScans = false;
	std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> > clouds;
	int index=1;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr previousCloud;
	pcl::IndicesPtr previousIndices;
	Transform previousPose;
	for(std::map<int, Transform>::const_iterator iter = poses.lower_bound(1); iter!=poses.end() && !_canceled; ++iter, ++index)
	{
		int points = 0;
		int totalIndices = 0;
		if(!iter->second.isNull())
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::IndicesPtr indices(new std::vector<int>);
			Transform localTransform = Transform::getIdentity();
			if(_ui->checkBox_regenerate->isChecked())
			{
				SensorData data;
				LaserScan scan;
				if(cachedSignatures.contains(iter->first))
				{
					const Signature & s = cachedSignatures.find(iter->first).value();
					data = s.sensorData();
					cv::Mat image,depth;
					data.uncompressData(
							_ui->checkBox_fromDepth->isChecked()?&image:0,
							_ui->checkBox_fromDepth->isChecked()?&depth:0,
							!_ui->checkBox_fromDepth->isChecked()?&scan:0);
				}
				else if(_dbDriver)
				{
					cv::Mat image,depth;
					_dbDriver->getNodeData(iter->first, data, _ui->checkBox_fromDepth->isChecked(), !_ui->checkBox_fromDepth->isChecked(), false, false);
					data.uncompressData(
							_ui->checkBox_fromDepth->isChecked()?&image:0,
							_ui->checkBox_fromDepth->isChecked()?&depth:0,
							!_ui->checkBox_fromDepth->isChecked()?&scan:0);
				}

				if(_ui->checkBox_fromDepth->isChecked() && !data.imageRaw().empty() && !data.depthOrRightRaw().empty())
				{
					cv::Mat depth = data.depthRaw();
					if(!depth.empty() && _ui->spinBox_fillDepthHoles->value() > 0)
					{
						depth = util2d::fillDepthHoles(depth, _ui->spinBox_fillDepthHoles->value(), float(_ui->spinBox_fillDepthHolesError->value())/100.f);
					}

					if(!depth.empty() &&
					  !_ui->lineEdit_distortionModel->text().isEmpty() &&
					   QFileInfo(_ui->lineEdit_distortionModel->text()).exists())
					{
						clams::DiscreteDepthDistortionModel model;
						model.load(_ui->lineEdit_distortionModel->text().toStdString());
						depth = depth.clone();// make sure we are not modifying data in cached signatures.
						model.undistort(depth);
					}

					// bilateral filtering
					if(!depth.empty() && _ui->checkBox_bilateral->isChecked())
					{
						depth = util2d::fastBilateralFiltering(depth,
								_ui->doubleSpinBox_bilateral_sigmaS->value(),
								_ui->doubleSpinBox_bilateral_sigmaR->value());
					}

					if(!depth.empty())
					{
						data.setRGBDImage(data.imageRaw(), depth, data.cameraModels());
					}

					UASSERT(iter->first == data.id());
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutNormals;
					std::vector<float> roiRatios;
					if(!_ui->lineEdit_roiRatios->text().isEmpty())
					{
						QStringList values = _ui->lineEdit_roiRatios->text().split(' ');
						if(values.size() == 4)
						{
							roiRatios.resize(4);
							for(int i=0; i<values.size(); ++i)
							{
								roiRatios[i] = uStr2Float(values[i].toStdString().c_str());
							}
						}
					}
					cloudWithoutNormals = util3d::cloudRGBFromSensorData(
							data,
							_ui->spinBox_decimation->value() == 0?1:_ui->spinBox_decimation->value(),
							_ui->doubleSpinBox_maxDepth->value(),
							_ui->doubleSpinBox_minDepth->value(),
							indices.get(),
							parameters,
							roiRatios);

					if(cloudWithoutNormals->size())
					{
						// Don't voxelize if we create organized mesh
						if(!(_ui->comboBox_pipeline->currentIndex()==0 && _ui->checkBox_meshing->isChecked()) && _ui->doubleSpinBox_voxelSize_assembled->value()>0.0)
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
						if(data.cameraModels().size() && !data.cameraModels()[0].localTransform().isNull())
						{
							localTransform = data.cameraModels()[0].localTransform();
							viewPoint[0] = data.cameraModels()[0].localTransform().x();
							viewPoint[1] = data.cameraModels()[0].localTransform().y();
							viewPoint[2] = data.cameraModels()[0].localTransform().z();
						}
						else if(data.stereoCameraModels().size() && !data.stereoCameraModels()[0].localTransform().isNull())
						{
							localTransform = data.stereoCameraModels()[0].localTransform();
							viewPoint[0] = data.stereoCameraModels()[0].localTransform().x();
							viewPoint[1] = data.stereoCameraModels()[0].localTransform().y();
							viewPoint[2] = data.stereoCameraModels()[0].localTransform().z();
						}

						if(_ui->spinBox_normalKSearch->value()>0 || _ui->doubleSpinBox_normalRadiusSearch->value()>0.0)
						{
							pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloudWithoutNormals, indices, _ui->spinBox_normalKSearch->value(), _ui->doubleSpinBox_normalRadiusSearch->value(), viewPoint);
							pcl::concatenateFields(*cloudWithoutNormals, *normals, *cloud);
							if(_ui->doubleSpinBox_groundNormalsUp->value() > 0.0)
							{
								util3d::adjustNormalsToViewPoint(cloud, viewPoint, (float)_ui->doubleSpinBox_groundNormalsUp->value());
							}
						}
						else
						{
							pcl::copyPointCloud(*cloudWithoutNormals, *cloud);
						}

						if(_ui->checkBox_subtraction->isChecked() &&
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
				else if(!_ui->checkBox_fromDepth->isChecked() && !scan.isEmpty())
				{
					scan = util3d::commonFiltering(scan,
							_ui->spinBox_decimation_scan->value(),
							_ui->doubleSpinBox_rangeMin->value(),
							_ui->doubleSpinBox_rangeMax->value(),
							_ui->doubleSpinBox_voxelSize_assembled->value(),
							_ui->spinBox_normalKSearch->value(),
							_ui->doubleSpinBox_normalRadiusSearch->value());

					if(!scan.empty())
					{
						scansHaveRGB = scan.hasRGB();
					}
					localTransform = scan.localTransform();
					cloud = util3d::laserScanToPointCloudRGBNormal(scan, localTransform); // put in base frame by default
					indices->resize(cloud->size());
					for(unsigned int i=0; i<indices->size(); ++i)
					{
						indices->at(i) = i;
					}
				}
				else
				{
					int weight = 0;
					if(cachedSignatures.contains(iter->first))
					{
						const Signature & s = cachedSignatures.find(iter->first).value();
						weight = s.getWeight();
					}
					else if(_dbDriver)
					{
						_dbDriver->getWeight(iter->first, weight);
					}
					if(weight>=0) // don't show error for intermediate nodes
					{
						UERROR("Cloud %d not found in cache!", iter->first);
					}
				}
			}
			else if(_ui->checkBox_fromDepth->isChecked() && uContains(cachedClouds, iter->first))
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutNormals;
				if(!_ui->checkBox_meshing->isChecked() &&
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
				std::vector<CameraModel> models;
				std::vector<StereoCameraModel> stereoModels;
				if(cachedSignatures.contains(iter->first))
				{
					const Signature & s = cachedSignatures.find(iter->first).value();
					models = s.sensorData().cameraModels();
					stereoModels = s.sensorData().stereoCameraModels();
				}
				else if(_dbDriver)
				{
					_dbDriver->getCalibration(iter->first, models, stereoModels);
				}

				if(models.size() && !models[0].localTransform().isNull())
				{
					localTransform = models[0].localTransform();
					viewPoint[0] = models[0].localTransform().x();
					viewPoint[1] = models[0].localTransform().y();
					viewPoint[2] = models[0].localTransform().z();
				}
				else if(stereoModels.size() && !stereoModels[0].localTransform().isNull())
				{
					localTransform = stereoModels[0].localTransform();
					viewPoint[0] = stereoModels[0].localTransform().x();
					viewPoint[1] = stereoModels[0].localTransform().y();
					viewPoint[2] = stereoModels[0].localTransform().z();
				}
				else
				{
					_progressDialog->appendText(tr("Cached cloud %1 is not found in cached data, the view point for normal computation will not be set (%2/%3).").arg(iter->first).arg(index).arg(poses.size()), Qt::darkYellow);
				}

				if(_ui->spinBox_normalKSearch->value()>0 || _ui->doubleSpinBox_normalRadiusSearch->value()>0.0)
				{
					pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloudWithoutNormals, indices, _ui->spinBox_normalKSearch->value(), _ui->doubleSpinBox_normalRadiusSearch->value(), viewPoint);
					pcl::concatenateFields(*cloudWithoutNormals, *normals, *cloud);
					if(_ui->doubleSpinBox_groundNormalsUp->value() > 0.0)
					{
						util3d::adjustNormalsToViewPoint(cloud, viewPoint, (float)_ui->doubleSpinBox_groundNormalsUp->value());
					}
				}
				else
				{
					pcl::copyPointCloud(*cloudWithoutNormals, *cloud);
				}
			}
			else if(!_ui->checkBox_fromDepth->isChecked() && uContains(cachedScans, iter->first))
			{
				LaserScan scan = util3d::commonFiltering(cachedScans.at(iter->first),
							_ui->spinBox_decimation_scan->value(),
							_ui->doubleSpinBox_rangeMin->value(),
							_ui->doubleSpinBox_rangeMax->value(),
							_ui->doubleSpinBox_voxelSize_assembled->value(),
							_ui->spinBox_normalKSearch->value(),
							_ui->doubleSpinBox_normalRadiusSearch->value());

				if(!scan.empty())
				{
					scansHaveRGB = scan.hasRGB();
				}
				localTransform = scan.localTransform();
				cloud = util3d::laserScanToPointCloudRGBNormal(scan, localTransform); // put in base frame by default
				indices->resize(cloud->size());
				for(unsigned int i=0; i<indices->size(); ++i)
				{
					indices->at(i) = i;
				}
			}
			else
			{
				int weight = 0;
				if(cachedSignatures.contains(iter->first))
				{
					const Signature & s = cachedSignatures.find(iter->first).value();
					weight = s.getWeight();
				}
				else if(_dbDriver)
				{
					_dbDriver->getWeight(iter->first, weight);
				}
				if(weight>=0) // don't show error for intermediate nodes
				{
					_progressDialog->appendText(tr("Cached cloud %1 not found. You may want to regenerate the clouds (%2/%3).").arg(iter->first).arg(index).arg(poses.size()), Qt::darkYellow);
				}
			}

			if(_ui->checkBox_filtering->isChecked())
			{
				if(!indices->empty() &&
				   (_ui->doubleSpinBox_ceilingHeight->value() != 0.0 ||
					_ui->doubleSpinBox_floorHeight->value() != 0.0))
				{
					float min = _ui->doubleSpinBox_floorHeight->value();
					float max = _ui->doubleSpinBox_ceilingHeight->value();
					indices = util3d::passThrough(
							util3d::transformPointCloud(cloud, iter->second),
							indices,
							"z",
							min!=0.0f&&(min<max || max==0.0f)?min:std::numeric_limits<int>::min(),
							max!=0.0f?max:std::numeric_limits<int>::max());
				}
				if(!indices->empty() &&
				   ( _ui->doubleSpinBox_footprintHeight->value() != 0.0 &&
					 _ui->doubleSpinBox_footprintWidth->value() != 0.0 &&
					 _ui->doubleSpinBox_footprintLength->value() != 0.0))
				{
					// filter footprint
					float h = _ui->doubleSpinBox_footprintHeight->value();
					float w = _ui->doubleSpinBox_footprintWidth->value();
					float l = _ui->doubleSpinBox_footprintLength->value();
					indices = util3d::cropBox(
							cloud,
							indices,
							Eigen::Vector4f(
									-l/2.0f,
									-w/2.0f,
									h<0.0f?h:0,
									1),
							Eigen::Vector4f(
									l/2.0f,
									w/2.0f,
									h<0.0f?-h:h,
									1),
							Transform::getIdentity(),
							true);
				}

				if( !indices->empty() &&
					_ui->doubleSpinBox_filteringRadius->value() > 0.0f &&
					_ui->spinBox_filteringMinNeighbors->value() > 0)
				{
					indices = util3d::radiusFiltering(cloud, indices, _ui->doubleSpinBox_filteringRadius->value(), _ui->spinBox_filteringMinNeighbors->value());
					if(indices->empty())
					{
						UWARN("Point cloud %d doesn't have anymore points (had %d points) after radius filtering.", iter->first, (int)cloud->size());
					}
				}
			}

			if(!indices->empty())
			{
				if((_ui->comboBox_frame->isEnabled() && _ui->comboBox_frame->currentIndex()==2) && cloud->isOrganized())
				{
					cloud = util3d::transformPointCloud(cloud, localTransform.inverse()); // put back in camera frame
				}
				else if(_ui->comboBox_frame->isEnabled() && _ui->comboBox_frame->currentIndex()==3)
				{
					cloud = util3d::transformPointCloud(cloud, localTransform.inverse()); // put back in scan frame
				}

				clouds.insert(std::make_pair(iter->first, std::make_pair(cloud, indices)));
				points = (int)cloud->size();
				totalIndices = (int)indices->size();
			}
		}
		else
		{
			UERROR("transform is null!?");
		}

		if(points>0)
		{
			if(_ui->checkBox_regenerate->isChecked())
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
		bool binaryMode,
		const std::vector<std::map<int, pcl::PointXY> > & pointToPixels)
{
	if(clouds.size() == 1)
	{
#ifdef RTABMAP_PDAL
		QString extensions = tr("Point cloud data (*.ply *.pcd");
		std::list<std::string> pdalFormats = uSplit(getPDALSupportedWriters(), ' ');
		for(std::list<std::string>::iterator iter=pdalFormats.begin(); iter!=pdalFormats.end(); ++iter)
		{
			if(iter->compare("ply") == 0 || iter->compare("pcd") == 0)
			{
				continue;
			}
			extensions += QString(" *.") + iter->c_str();
		}
		extensions += ")";
#else
		QString extensions = tr("Point cloud data (*.ply *.pcd)");
#endif
		QString path = QFileDialog::getSaveFileName(this, tr("Save cloud to ..."), workingDirectory+QDir::separator()+"cloud.ply", extensions);

		if(!path.isEmpty())
		{
			if(QFileInfo(path).suffix().isEmpty())
			{
				//use ply by default
				path += ".ply";
			}

			if(clouds.begin()->second->size())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGBWithoutNormals;
				pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIWithoutNormals;
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudIWithNormals;
				if(!_ui->checkBox_fromDepth->isChecked() && !_scansHaveRGB &&
					!(_ui->checkBox_cameraProjection->isEnabled() &&
					_ui->checkBox_cameraProjection->isChecked() &&
					_ui->checkBox_camProjRecolorPoints->isChecked() &&
					clouds.size()==1 && clouds.begin()->first==0))
				{
					// When laser scans are exported (and camera RGB was not applied), convert RGB to Intensity
					if(_ui->spinBox_normalKSearch->value()>0 || _ui->doubleSpinBox_normalRadiusSearch->value()>0.0)
					{
						cloudIWithNormals.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
						cloudIWithNormals->resize(clouds.begin()->second->size());
						for(unsigned int i=0; i<cloudIWithNormals->size(); ++i)
						{
							cloudIWithNormals->points[i].x = clouds.begin()->second->points[i].x;
							cloudIWithNormals->points[i].y = clouds.begin()->second->points[i].y;
							cloudIWithNormals->points[i].z = clouds.begin()->second->points[i].z;
							cloudIWithNormals->points[i].normal_x = clouds.begin()->second->points[i].normal_x;
							cloudIWithNormals->points[i].normal_y = clouds.begin()->second->points[i].normal_y;
							cloudIWithNormals->points[i].normal_z = clouds.begin()->second->points[i].normal_z;
							cloudIWithNormals->points[i].curvature = clouds.begin()->second->points[i].curvature;
							int * intensity = (int *)&cloudIWithNormals->points[i].intensity;
							*intensity =
									int(clouds.begin()->second->points[i].r) |
									int(clouds.begin()->second->points[i].g) << 8 |
									int(clouds.begin()->second->points[i].b) << 16 |
									int(clouds.begin()->second->points[i].a) << 24;
						}
					}
					else
					{
						cloudIWithoutNormals.reset(new pcl::PointCloud<pcl::PointXYZI>);
						cloudIWithoutNormals->resize(clouds.begin()->second->size());
						for(unsigned int i=0; i<cloudIWithoutNormals->size(); ++i)
						{
							cloudIWithoutNormals->points[i].x = clouds.begin()->second->points[i].x;
							cloudIWithoutNormals->points[i].y = clouds.begin()->second->points[i].y;
							cloudIWithoutNormals->points[i].z = clouds.begin()->second->points[i].z;
							int * intensity = (int *)&cloudIWithoutNormals->points[i].intensity;
							*intensity =
									int(clouds.begin()->second->points[i].r) |
									int(clouds.begin()->second->points[i].g) << 8 |
									int(clouds.begin()->second->points[i].b) << 16 |
									int(clouds.begin()->second->points[i].a) << 24;
						}
					}
				}
				else if(_ui->spinBox_normalKSearch->value()<=0 && _ui->doubleSpinBox_normalRadiusSearch->value()<=0.0)
				{
					cloudRGBWithoutNormals.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::copyPointCloud(*clouds.begin()->second, *cloudRGBWithoutNormals);
				}

				_progressDialog->appendText(tr("Saving the cloud (%1 points)...").arg(clouds.begin()->second->size()));

				bool success =false;
				if(QFileInfo(path).suffix() == "pcd")
				{
					if(cloudIWithNormals.get())
					{
						success = pcl::io::savePCDFile(path.toStdString(), *cloudIWithNormals, binaryMode) == 0;
					}
					else if(cloudIWithoutNormals.get())
					{
						success = pcl::io::savePCDFile(path.toStdString(), *cloudIWithoutNormals, binaryMode) == 0;
					}
					else if(cloudRGBWithoutNormals.get())
					{
						success = pcl::io::savePCDFile(path.toStdString(), *cloudRGBWithoutNormals, binaryMode) == 0;
					}
					else
					{
						success = pcl::io::savePCDFile(path.toStdString(), *clouds.begin()->second, binaryMode) == 0;
					}
				}
#ifdef RTABMAP_PDAL
				else if(QFileInfo(path).suffix() == "ply" && pointToPixels.empty()) {
#else
				else if(QFileInfo(path).suffix() == "ply") {
#endif
					if(cloudIWithNormals.get())
					{
						success = pcl::io::savePLYFile(path.toStdString(), *cloudIWithNormals, binaryMode) == 0;
					}
					else if(cloudIWithoutNormals.get())
					{
						success = pcl::io::savePLYFile(path.toStdString(), *cloudIWithoutNormals, binaryMode) == 0;
					}
					else if(cloudRGBWithoutNormals.get())
					{
						success = pcl::io::savePLYFile(path.toStdString(), *cloudRGBWithoutNormals, binaryMode) == 0;
					}
					else
					{
						success = pcl::io::savePLYFile(path.toStdString(), *clouds.begin()->second, binaryMode) == 0;
					}
				}
#ifdef RTABMAP_PDAL
				else if(!QFileInfo(path).suffix().isEmpty())
				{
					std::vector<int> cameraIds(pointToPixels.size(), 0);
					for(size_t i=0;i<pointToPixels.size(); ++i)
					{
						if(!pointToPixels[i].empty())
						{
							cameraIds[i] = pointToPixels[i].begin()->first;
						}
					}
					if(cloudIWithNormals.get())
					{
						success = savePDALFile(path.toStdString(), *cloudIWithNormals, cameraIds, binaryMode) == 0;
					}
					else if(cloudIWithoutNormals.get())
					{
						success = savePDALFile(path.toStdString(), *cloudIWithoutNormals, cameraIds, binaryMode) == 0;
					}
					else if(cloudRGBWithoutNormals.get())
					{
						success = savePDALFile(path.toStdString(), *cloudRGBWithoutNormals, cameraIds, binaryMode) == 0;
					}
					else
					{
						success = savePDALFile(path.toStdString(), *clouds.begin()->second, cameraIds, binaryMode) == 0;
					}
				}
#endif
				else
				{
					UERROR("Extension not recognized! (%s) Should be one of (*.ply *.pcd *.las).", QFileInfo(path).suffix().toStdString().c_str());
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
		QStringList items;
		items.push_back("ply");
		items.push_back("pcd");
#ifdef RTABMAP_PDAL
		QString extensions = tr("Save clouds to (*.ply *.pcd");
		std::list<std::string> pdalFormats = uSplit(getPDALSupportedWriters(), ' ');
		for(std::list<std::string>::iterator iter=pdalFormats.begin(); iter!=pdalFormats.end(); ++iter)
		{
			if(iter->compare("ply") == 0 || iter->compare("pcd") == 0)
			{
				continue;
			}
			extensions += QString(" *.") + iter->c_str();

			items.push_back(iter->c_str());
		}
		extensions += ")...";
#else
		QString extensions = tr("Save clouds to (*.ply *.pcd)...");
#endif
		QString path = QFileDialog::getExistingDirectory(this, extensions, workingDirectory, QFileDialog::ShowDirsOnly);
		if(!path.isEmpty())
		{
			bool ok = false;
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
							transformedCloud = util3d::transformPointCloud(iter->second, !_ui->comboBox_frame->isEnabled()||_ui->comboBox_frame->currentIndex()==0?poses.at(iter->first):Transform::getIdentity());

							pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGBWithoutNormals;
							pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIWithoutNormals;
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudIWithNormals;
							if(!_ui->checkBox_fromDepth->isChecked() && !_scansHaveRGB)
							{
								// When laser scans are exported, convert RGB to Intensity
								if(_ui->spinBox_normalKSearch->value()>0 || _ui->doubleSpinBox_normalRadiusSearch->value()>0.0)
								{
									cloudIWithNormals.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
									cloudIWithNormals->resize(transformedCloud->size());
									for(unsigned int i=0; i<cloudIWithNormals->size(); ++i)
									{
										cloudIWithNormals->points[i].x = transformedCloud->points[i].x;
										cloudIWithNormals->points[i].y = transformedCloud->points[i].y;
										cloudIWithNormals->points[i].z = transformedCloud->points[i].z;
										cloudIWithNormals->points[i].normal_x = transformedCloud->points[i].normal_x;
										cloudIWithNormals->points[i].normal_y = transformedCloud->points[i].normal_y;
										cloudIWithNormals->points[i].normal_z = transformedCloud->points[i].normal_z;
										cloudIWithNormals->points[i].curvature = transformedCloud->points[i].curvature;
										int * intensity = (int *)&cloudIWithNormals->points[i].intensity;
										*intensity =
												int(transformedCloud->points[i].r) |
												int(transformedCloud->points[i].g) << 8 |
												int(transformedCloud->points[i].b) << 16 |
												int(transformedCloud->points[i].a) << 24;
									}
								}
								else
								{
									cloudIWithoutNormals.reset(new pcl::PointCloud<pcl::PointXYZI>);
									cloudIWithoutNormals->resize(transformedCloud->size());
									for(unsigned int i=0; i<cloudIWithoutNormals->size(); ++i)
									{
										cloudIWithoutNormals->points[i].x = transformedCloud->points[i].x;
										cloudIWithoutNormals->points[i].y = transformedCloud->points[i].y;
										cloudIWithoutNormals->points[i].z = transformedCloud->points[i].z;
										int * intensity = (int *)&cloudIWithoutNormals->points[i].intensity;
										*intensity =
												int(transformedCloud->points[i].r) |
												int(transformedCloud->points[i].g) << 8 |
												int(transformedCloud->points[i].b) << 16 |
												int(transformedCloud->points[i].a) << 24;
									}
								}
							}
							else if(_ui->spinBox_normalKSearch->value()<=0 && _ui->doubleSpinBox_normalRadiusSearch->value()<=0.0)
							{
								cloudRGBWithoutNormals.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
								pcl::copyPointCloud(*transformedCloud, *cloudRGBWithoutNormals);
							}

							QString pathFile = path+QDir::separator()+QString("%1%2.%3").arg(prefix).arg(iter->first).arg(suffix);
							bool success =false;
							if(suffix == "pcd")
							{
								if(cloudIWithNormals.get())
								{
									success = pcl::io::savePCDFile(pathFile.toStdString(), *cloudIWithNormals, binaryMode) == 0;
								}
								else if(cloudIWithoutNormals.get())
								{
									success = pcl::io::savePCDFile(pathFile.toStdString(), *cloudIWithoutNormals, binaryMode) == 0;
								}
								else if(cloudRGBWithoutNormals.get())
								{
									success = pcl::io::savePCDFile(pathFile.toStdString(), *cloudRGBWithoutNormals, binaryMode) == 0;
								}
								else
								{
									success = pcl::io::savePCDFile(pathFile.toStdString(), *transformedCloud, binaryMode) == 0;
								}
							}
							else if(suffix == "ply")
							{
								if(cloudIWithNormals.get())
								{
									success = pcl::io::savePLYFile(pathFile.toStdString(), *cloudIWithNormals, binaryMode) == 0;
								}
								else if(cloudIWithoutNormals.get())
								{
									success = pcl::io::savePLYFile(pathFile.toStdString(), *cloudIWithoutNormals, binaryMode) == 0;
								}
								else if(cloudRGBWithoutNormals.get())
								{
									success = pcl::io::savePLYFile(pathFile.toStdString(), *cloudRGBWithoutNormals, binaryMode) == 0;
								}
								else
								{
									success = pcl::io::savePLYFile(pathFile.toStdString(), *transformedCloud, binaryMode) == 0;
								}
							}
#ifdef RTABMAP_PDAL
							else if(!suffix.isEmpty())
							{
								if(cloudIWithNormals.get())
								{
									success = savePDALFile(pathFile.toStdString(), *cloudIWithNormals) == 0;
								}
								else if(cloudIWithoutNormals.get())
								{
									success = savePDALFile(pathFile.toStdString(), *cloudIWithoutNormals) == 0;
								}
								else if(cloudRGBWithoutNormals.get())
								{
									success = savePDALFile(pathFile.toStdString(), *cloudRGBWithoutNormals) == 0;
								}
								else
								{
									success = savePDALFile(pathFile.toStdString(), *transformedCloud) == 0;
								}
							}
#endif
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
					success = saveOBJFile(path, *meshes.begin()->second);
				}
				else
				{
					UERROR("Extension not recognized! (%s) Should be (*.ply) or (*.obj).", QFileInfo(path).suffix().toStdString().c_str());
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
		QString path = QFileDialog::getExistingDirectory(this, tr("Save meshes to (*.ply *.obj)..."), workingDirectory, QFileDialog::ShowDirsOnly);
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
								tmp = util3d::transformPointCloud(tmp, !_ui->comboBox_frame->isEnabled()||_ui->comboBox_frame->currentIndex()==0?poses.at(iter->first):Transform::getIdentity());
								pcl::toPCLPointCloud2(*tmp, mesh.cloud);
							}
							else
							{
								pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
								pcl::fromPCLPointCloud2(iter->second->cloud, *tmp);
								tmp = util3d::transformPointCloud(tmp, !_ui->comboBox_frame->isEnabled()||_ui->comboBox_frame->currentIndex()==0?poses.at(iter->first):Transform::getIdentity());
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
								success = saveOBJFile(pathFile, mesh);
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
		std::map<int, pcl::TextureMesh::Ptr> & meshes,
		const QMap<int, Signature> & cachedSignatures,
		const std::vector<std::map<int, pcl::PointXY> > & textureVertexToPixels)
{
	std::map<int, cv::Mat> images;
	std::map<int, std::vector<CameraModel> > calibrations;
	for(QMap<int, Signature>::const_iterator iter=cachedSignatures.constBegin(); iter!=cachedSignatures.constEnd(); ++iter)
	{
		std::vector<CameraModel> models;
		if(iter->sensorData().cameraModels().size())
		{
			models = iter->sensorData().cameraModels();
		}
		else if(iter->sensorData().stereoCameraModels().size())
		{
			for(size_t i=0; i<iter->sensorData().stereoCameraModels().size(); ++i)
			{
				models.push_back(iter->sensorData().stereoCameraModels()[i].left());
			}
		}

		if(!models.empty())
		{
			if(!iter->sensorData().imageRaw().empty())
			{
				calibrations.insert(std::make_pair(iter.key(), models));
				images.insert(std::make_pair(iter.key(), iter->sensorData().imageRaw()));
			}
			else if(!iter->sensorData().imageCompressed().empty())
			{
				calibrations.insert(std::make_pair(iter.key(), models));
				images.insert(std::make_pair(iter.key(), iter->sensorData().imageCompressed()));
			}
		}
	}
	int textureSize = 1024;
	if(_ui->comboBox_meshingTextureSize->currentIndex() > 0)
	{
		textureSize = 128 << _ui->comboBox_meshingTextureSize->currentIndex(); // start at 256
	}
	int blendingDecimation = 0;
	if(_ui->checkBox_blending->isChecked())
	{
		if(_ui->comboBox_blendingDecimation->currentIndex() > 0)
		{
			blendingDecimation = 1 << (_ui->comboBox_blendingDecimation->currentIndex()-1);
		}
	}

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

				pcl::TextureMesh::Ptr mesh = meshes.begin()->second;

				cv::Mat globalTextures;
				bool texturesMerged = _ui->comboBox_meshingTextureSize->isEnabled() && _ui->comboBox_meshingTextureSize->currentIndex() > 0;
				if(texturesMerged && mesh->tex_materials.size()>1)
				{
					_progressDialog->appendText(tr("Merging textures..."));
					QApplication::processEvents();
					uSleep(100);
					QApplication::processEvents();

					std::map<int, std::map<int, cv::Vec4d> > gains;
					std::map<int, std::map<int, cv::Mat> > blendingGains;
					std::pair<float, float> contrastValues(0,0);
					globalTextures = util3d::mergeTextures(
							*mesh,
							images,
							calibrations,
							0,
							_dbDriver,
							textureSize,
							_ui->checkBox_multiband->isEnabled() && _ui->checkBox_multiband->isChecked()?1:_ui->spinBox_mesh_maxTextures->value(),
							textureVertexToPixels,
							_ui->checkBox_gainCompensation->isChecked(),
							_ui->doubleSpinBox_gainBeta->value(),
							_ui->checkBox_gainRGB->isChecked(),
							_ui->checkBox_blending->isChecked(),
							blendingDecimation,
							_ui->spinBox_textureBrightnessContrastRatioLow->value(),
							_ui->spinBox_textureBrightnessContrastRatioHigh->value(),
							_ui->checkBox_exposureFusion->isEnabled() && _ui->checkBox_exposureFusion->isChecked(),
							0,
							0,
							&gains,
							&blendingGains,
							&contrastValues);

					_progressDialog->appendText(tr("Merging textures... done."));
					QApplication::processEvents();
					uSleep(100);
					QApplication::processEvents();

					if(_ui->checkBox_multiband->isEnabled() && _ui->checkBox_multiband->isChecked() && mesh->tex_polygons.size() == 1)
					{
						_progressDialog->appendText(tr("Multiband texturing... (this may take a couple of minutes!)"));
						QApplication::processEvents();
						uSleep(100);
						QApplication::processEvents();

						success = util3d::multiBandTexturing(
								path.toStdString(),
								mesh->cloud,
								mesh->tex_polygons[0],
								poses,
								textureVertexToPixels,
								images,
								calibrations,
								0,
								_dbDriver,
								textureSize,
								_ui->spinBox_multiband_downscale->value(),
								_ui->lineEdit_multiband_nbcontrib->text().toStdString(),
								_ui->comboBox_meshingTextureFormat->currentText().toStdString(),
								gains,
								blendingGains,
								contrastValues,
								true,
								_ui->comboBox_multiband_unwrap->currentIndex(),
								_ui->checkBox_multiband_fillholes->isChecked(),
								_ui->spinBox_multiband_padding->value(),
								_ui->doubleSpinBox_multiband_bestscore->value(),
								_ui->doubleSpinBox_multiband_angle->value(),
								_ui->checkBox_multiband_forcevisible->isChecked());
						if(success)
						{
							_progressDialog->incrementStep();
							_progressDialog->appendText(tr("Saving the mesh... done."));

							QMessageBox::information(this, tr("Save successful!"), tr("Mesh saved to \"%1\"").arg(path));
						}
						else
						{
							QMessageBox::warning(this, tr("Save failed!"), tr("Failed to save to \"%1\"").arg(path));
						}
						return;
					}
				}

				bool singleTexture = mesh->tex_materials.size() == 1;
				if(!singleTexture)
				{
					removeDirRecursively(QFileInfo(path).absoluteDir().absolutePath()+QDir::separator()+QFileInfo(path).baseName());
					QDir(QFileInfo(path).absoluteDir().absolutePath()).mkdir(QFileInfo(path).baseName());
				}

				// used for multi camera texturing, to avoid reloading same texture for sub cameras
				cv::Mat previousImage;
				int previousTextureId = 0;
				std::vector<CameraModel> previousCameraModels;

				cv::Size imageSize;
				for(unsigned int i=0; i<mesh->tex_materials.size(); ++i)
				{
					if(!mesh->tex_materials[i].tex_file.empty())
					{
						// absolute path
						QString fullPath;
						if(singleTexture)
						{
							fullPath = QFileInfo(path).absoluteDir().absolutePath()+QDir::separator()+QFileInfo(path).baseName()+_ui->comboBox_meshingTextureFormat->currentText();
						}
						else
						{
							fullPath = QFileInfo(path).absoluteDir().absolutePath()+QDir::separator()+QFileInfo(path).baseName()+QDir::separator()+QString(mesh->tex_materials[i].tex_file.c_str())+_ui->comboBox_meshingTextureFormat->currentText();
						}
						UDEBUG("Saving %s...", fullPath.toStdString().c_str());
						if(singleTexture || !QFileInfo(fullPath).exists())
						{
							std::list<std::string> texFileSplit = uSplit(mesh->tex_materials[i].tex_file, '_');
							if(texFileSplit.size() && uIsInteger(texFileSplit.front(), false))
							{
								int textureId = uStr2Int(texFileSplit.front());
								int textureSubCamera = -1;
								if(texFileSplit.size() == 2 &&
								   uIsInteger(texFileSplit.back(), false))
								{
									textureSubCamera = uStr2Int(texFileSplit.back());
								}
								cv::Mat image;
								std::vector<CameraModel> cameraModels;

								if(textureId == previousTextureId)
								{
									image = previousImage;
									cameraModels = previousCameraModels;
								}
								else
								{
									if(cachedSignatures.contains(textureId) && !cachedSignatures.value(textureId).sensorData().imageCompressed().empty())
									{
										cachedSignatures.value(textureId).sensorData().uncompressDataConst(&image, 0);
										cameraModels = cachedSignatures.value(textureId).sensorData().cameraModels();
									}
									else if(_dbDriver)
									{
										SensorData data;
										_dbDriver->getNodeData(textureId, data, true, false, false, false);
										data.uncompressDataConst(&image, 0);
										std::vector<StereoCameraModel> stereoModels;
										_dbDriver->getCalibration(textureId, cameraModels, stereoModels);
										if(cameraModels.empty())
										{
											for(size_t i=0; i<stereoModels.size(); ++i)
											{
												cameraModels.push_back(stereoModels[i].left());
											}
										}
									}

									previousImage = image;
									previousCameraModels = cameraModels;
									previousTextureId = textureId;
								}
								UASSERT(!image.empty());
								imageSize = image.size();
								if(textureSubCamera>=0)
								{
									UASSERT(cameraModels.size());
									imageSize.width/=cameraModels.size();
									image = image.colRange(imageSize.width*textureSubCamera, imageSize.width*(textureSubCamera+1));
								}
								if(_ui->checkBox_gainCompensation->isChecked() && _compensator && _compensator->getIndex(textureId) >= 0)
								{
									_compensator->apply(textureId, image, _ui->checkBox_gainRGB->isChecked());
								}

								if(!cv::imwrite(fullPath.toStdString(), image))
								{
									_progressDialog->appendText(tr("Failed saving texture \"%1\" to \"%2\".")
											.arg(mesh->tex_materials[i].tex_file.c_str()).arg(fullPath), Qt::darkRed);
									_progressDialog->setAutoClose(false);
								}
							}
							else if(imageSize.height && imageSize.width)
							{
								// make a blank texture
								cv::Mat image = cv::Mat::ones(imageSize, CV_8UC1)*255;
								cv::imwrite(fullPath.toStdString(), image);
							}
							else if(!globalTextures.empty())
							{
								if(!cv::imwrite(fullPath.toStdString(), globalTextures(cv::Range::all(), cv::Range(i*globalTextures.rows, (i+1)*globalTextures.rows))))
								{
									_progressDialog->appendText(tr("Failed saving texture \"%1\" to \"%2\".")
											.arg(mesh->tex_materials[i].tex_file.c_str()).arg(fullPath), Qt::darkRed);
									_progressDialog->setAutoClose(false);
								}
							}
							else
							{
								UWARN("Ignored texture %s (no image size set yet)", mesh->tex_materials[i].tex_file.c_str());
							}
						}
						else
						{
							UWARN("File %s already exists!", fullPath.toStdString().c_str());
						}
						// relative path
						if(singleTexture)
						{
							mesh->tex_materials[i].tex_file=QFileInfo(path).baseName().toStdString()+_ui->comboBox_meshingTextureFormat->currentText().toStdString();
						}
						else
						{
							mesh->tex_materials[i].tex_file=(QFileInfo(path).baseName()+QDir::separator()+QString(mesh->tex_materials[i].tex_file.c_str())+_ui->comboBox_meshingTextureFormat->currentText()).toStdString();
						}
					}
				}

                if(saveOBJFile(path, mesh))
				{
					_progressDialog->incrementStep();
					_progressDialog->appendText(tr("Saving the mesh (with %1 textures)... done.").arg(mesh->tex_materials.size()));

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
		QString path = QFileDialog::getExistingDirectory(this, tr("Save texture meshes to (*.obj)..."), workingDirectory, QFileDialog::ShowDirsOnly);
		if(!path.isEmpty())
		{
			bool ok = false;
			QString prefix = QInputDialog::getText(this, tr("File prefix"), tr("Prefix:"), QLineEdit::Normal, "mesh", &ok);
			QString suffix = "obj";

			if(ok)
			{
				for(std::map<int, pcl::TextureMesh::Ptr>::iterator iter=meshes.begin(); iter!=meshes.end(); ++iter)
				{
					QString currentPrefix=prefix+QString::number(iter->first);
					if(iter->second->tex_materials.size())
					{
						pcl::TextureMesh::Ptr mesh = iter->second;
						cv::Mat globalTextures;
						bool texturesMerged = _ui->comboBox_meshingTextureSize->isEnabled() && _ui->comboBox_meshingTextureSize->currentIndex() > 0;
						if(texturesMerged && mesh->tex_materials.size()>1)
						{
							globalTextures = util3d::mergeTextures(
									*mesh,
									images,
									calibrations,
									0,
									_dbDriver,
									textureSize,
									_ui->spinBox_mesh_maxTextures->value(),
									textureVertexToPixels,
									_ui->checkBox_gainCompensation->isChecked(),
									_ui->doubleSpinBox_gainBeta->value(),
									_ui->checkBox_gainRGB->isChecked(),
									_ui->checkBox_blending->isChecked(),
									blendingDecimation,
									_ui->spinBox_textureBrightnessContrastRatioLow->value(),
									_ui->spinBox_textureBrightnessContrastRatioHigh->value(),
									_ui->checkBox_exposureFusion->isEnabled() && _ui->checkBox_exposureFusion->isChecked());
						}
						bool singleTexture = mesh->tex_materials.size() == 1;
						if(!singleTexture)
						{
							removeDirRecursively(path+QDir::separator()+currentPrefix);
							QDir(path).mkdir(currentPrefix);
						}

						// used for multi camera texturing, to avoid reloading same texture for sub cameras
						cv::Mat previousImage;
						int previousTextureId = 0;
						std::vector<CameraModel> previousCameraModels;

						cv::Size imageSize;
						for(unsigned int i=0;i<mesh->tex_materials.size(); ++i)
						{
							if(!mesh->tex_materials[i].tex_file.empty())
							{
								std::list<std::string> texFileSplit = uSplit(mesh->tex_materials[i].tex_file, '_');
								int textureId = 0;
								int textureSubCamera = -1;
								if(texFileSplit.size() && uIsInteger(texFileSplit.front(), false))
								{
									textureId = uStr2Int(texFileSplit.front());
									if(texFileSplit.size() == 2 &&
									   uIsInteger(texFileSplit.back(), false))
									{
										textureSubCamera = uStr2Int(texFileSplit.back());
									}
								}

								// absolute path
								QString fullPath;
								if(singleTexture)
								{
									mesh->tex_materials[i].tex_file = uNumber2Str(iter->first);
									fullPath = path+QDir::separator()+prefix + QString(mesh->tex_materials[i].tex_file.c_str())+_ui->comboBox_meshingTextureFormat->currentText();
								}
								else
								{
									fullPath = path+QDir::separator()+currentPrefix+QDir::separator()+QString(mesh->tex_materials[i].tex_file.c_str())+_ui->comboBox_meshingTextureFormat->currentText();
								}
								if(textureId>0)
								{
									cv::Mat image;
									std::vector<CameraModel> cameraModels;

									if(textureId == previousTextureId)
									{
										image = previousImage;
										cameraModels = previousCameraModels;
									}
									else
									{
										if(cachedSignatures.contains(textureId) && !cachedSignatures.value(textureId).sensorData().imageCompressed().empty())
										{
											cachedSignatures.value(textureId).sensorData().uncompressDataConst(&image, 0);
											cameraModels = cachedSignatures.value(textureId).sensorData().cameraModels();
										}
										else if(_dbDriver)
										{
											SensorData data;
											_dbDriver->getNodeData(textureId, data, true, false, false, false);
											data.uncompressDataConst(&image, 0);
											std::vector<StereoCameraModel> stereoModels;
											_dbDriver->getCalibration(textureId, cameraModels, stereoModels);
											if(cameraModels.empty())
											{
												for(size_t i=0; i<stereoModels.size(); ++i)
												{
													cameraModels.push_back(stereoModels[i].left());
												}
											}
										}

										previousImage = image;
										previousCameraModels = cameraModels;
										previousTextureId = textureId;
									}


									UASSERT(!image.empty());
									imageSize = image.size();
									if(textureSubCamera>=0)
									{
										UASSERT(cameraModels.size());
										imageSize.width/=cameraModels.size();
										image = image.colRange(imageSize.width*textureSubCamera, imageSize.width*(textureSubCamera+1));
									}
									if(_ui->checkBox_gainCompensation->isChecked() && _compensator && _compensator->getIndex(textureId) >= 0)
									{
										_compensator->apply(textureId, image, _ui->checkBox_gainRGB->isChecked());
									}

									if(!cv::imwrite(fullPath.toStdString(), image))
									{
										_progressDialog->appendText(tr("Failed saving texture \"%1\" to \"%2\".")
												.arg(mesh->tex_materials[i].tex_file.c_str()).arg(fullPath), Qt::darkRed);
										_progressDialog->setAutoClose(false);
									}
								}
								else if(imageSize.height && imageSize.width)
								{
									// make a blank texture
									cv::Mat image = cv::Mat::ones(imageSize, CV_8UC1)*255;
									cv::imwrite(fullPath.toStdString(), image);
								}
								else if(!globalTextures.empty())
								{
									if(!cv::imwrite(fullPath.toStdString(), globalTextures(cv::Range::all(), cv::Range(i*globalTextures.rows, (i+1)*globalTextures.rows))))
									{
										_progressDialog->appendText(tr("Failed saving texture \"%1\" to \"%2\".")
												.arg(mesh->tex_materials[i].tex_file.c_str()).arg(fullPath), Qt::darkRed);
										_progressDialog->setAutoClose(false);
									}
								}
								else
								{
									UWARN("Ignored texture %s (no image size set yet)", mesh->tex_materials[i].tex_file.c_str());
								}
								// relative path
								if(singleTexture)
								{
									mesh->tex_materials[i].tex_file=(prefix+ QString(mesh->tex_materials[i].tex_file.c_str())+_ui->comboBox_meshingTextureFormat->currentText()).toStdString();
								}
								else
								{
									mesh->tex_materials[i].tex_file=(currentPrefix+QDir::separator()+QString(mesh->tex_materials[i].tex_file.c_str())+_ui->comboBox_meshingTextureFormat->currentText()).toStdString();
								}
							}
						}
						pcl::PointCloud<pcl::PointNormal>::Ptr tmp(new pcl::PointCloud<pcl::PointNormal>);
						pcl::fromPCLPointCloud2(mesh->cloud, *tmp);
						tmp = util3d::transformPointCloud(tmp, !_ui->comboBox_frame->isEnabled()||_ui->comboBox_frame->currentIndex()==0?poses.at(iter->first):Transform::getIdentity());
						pcl::toPCLPointCloud2(*tmp, mesh->cloud);

						QString pathFile = path+QDir::separator()+QString("%1.%3").arg(currentPrefix).arg(suffix);
						bool success =false;
						if(suffix == "obj")
						{
							success = saveOBJFile(pathFile, mesh);
						}
						else
						{
							UFATAL("Extension not recognized! (%s)", suffix.toStdString().c_str());
						}
						if(success)
						{
							_progressDialog->appendText(tr("Saved mesh %1 (%2 textures) to %3.")
									.arg(iter->first).arg(mesh->tex_materials.size()).arg(pathFile));
						}
						else
						{
							_progressDialog->appendText(tr("Failed saving mesh %1 (%2 textures) to %3.")
									.arg(iter->first).arg(mesh->tex_materials.size()).arg(pathFile), Qt::darkRed);
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

    bool ExportCloudsDialog::saveOBJFile(const QString &path, pcl::TextureMesh::Ptr &mesh) const {
#if PCL_VERSION_COMPARE(>=, 1, 13, 0)
        mesh->tex_coord_indices = std::vector<std::vector<pcl::Vertices>>();
        auto nr_meshes = static_cast<unsigned>(mesh->tex_polygons.size());
        unsigned f_idx = 0;
        for (unsigned m = 0; m < nr_meshes; m++) {
            std::vector<pcl::Vertices> ci = mesh->tex_polygons[m];
            for(std::size_t i = 0; i < ci.size(); i++) {
                for (std::size_t j = 0; j < ci[i].vertices.size(); j++) {
                    ci[i].vertices[j] = ci[i].vertices.size() * (i + f_idx) + j;
                }
            }
            mesh->tex_coord_indices.push_back(ci);
            f_idx += static_cast<unsigned>(mesh->tex_polygons[m].size());
        }
#endif
        return pcl::io::saveOBJFile(path.toStdString(), *mesh) == 0;
    }

    bool ExportCloudsDialog::saveOBJFile(const QString &path, pcl::PolygonMesh &mesh) const {
        return pcl::io::saveOBJFile(path.toStdString(), mesh) == 0;
    }


}
