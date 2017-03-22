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

#include "rtabmap/gui/PreferencesDialog.h"
#include "rtabmap/gui/DatabaseViewer.h"
#include "rtabmap/gui/OdometryViewer.h"
#include "rtabmap/gui/CalibrationDialog.h"

#include <QtCore/QSettings>
#include <QtCore/QDir>
#include <QtCore/QTimer>
#include <QUrl>

#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QtGui/QStandardItemModel>
#include <QMainWindow>
#include <QProgressDialog>
#include <QScrollBar>
#include <QStatusBar>
#include <QDesktopServices>
#include <QtGui/QCloseEvent>

#include "ui_preferencesDialog.h"

#include "rtabmap/core/Version.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/CameraRGB.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/Optimizer.h"
#include "rtabmap/core/OptimizerG2O.h"
#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/clams/discrete_depth_distortion_model.h"

#include "rtabmap/gui/LoopClosureViewer.h"
#include "rtabmap/gui/CameraViewer.h"
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/gui/ImageView.h"
#include "rtabmap/gui/GraphViewer.h"
#include "ExportCloudsDialog.h"
#include "ExportScansDialog.h"
#include "PostProcessingDialog.h"
#include "CreateSimpleCalibrationDialog.h"
#include "DepthCalibrationDialog.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UEventsManager.h>
#include "rtabmap/utilite/UPlot.h"

#include <opencv2/opencv_modules.hpp>
#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
  #include <opencv2/gpu/gpu.hpp>
#endif
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
  #include <opencv2/core/cuda.hpp>
#endif
#endif

using namespace rtabmap;

namespace rtabmap {

PreferencesDialog::PreferencesDialog(QWidget * parent) :
	QDialog(parent),
	_obsoletePanels(kPanelDummy),
	_ui(0),
	_indexModel(0),
	_initialized(false),
	_progressDialog(new QProgressDialog(this)),
	_calibrationDialog(new CalibrationDialog(false, ".", false, this)),
	_createCalibrationDialog(new CreateSimpleCalibrationDialog(".", "", this))
{
	ULOGGER_DEBUG("");
	_calibrationDialog->setWindowFlags(Qt::Window);
	_calibrationDialog->setWindowTitle(tr("Calibration"));

	_progressDialog->setWindowTitle(tr("Read parameters..."));
	_progressDialog->setMaximum(2);
	_progressDialog->setValue(2);

	_ui = new Ui_preferencesDialog();
	_ui->setupUi(this);

	bool haveCuda = false;
#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
	haveCuda = cv::gpu::getCudaEnabledDeviceCount() != 0;
#endif
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
	haveCuda = cv::cuda::getCudaEnabledDeviceCount() != 0;
#endif
#endif
	if(!haveCuda)
	{
		_ui->surf_checkBox_gpuVersion->setChecked(false);
		_ui->surf_checkBox_gpuVersion->setEnabled(false);
		_ui->label_surf_checkBox_gpuVersion->setEnabled(false);
		_ui->surf_doubleSpinBox_gpuKeypointsRatio->setEnabled(false);
		_ui->label_surf_checkBox_gpuKeypointsRatio->setEnabled(false);

		_ui->fastGpu->setChecked(false);
		_ui->fastGpu->setEnabled(false);
		_ui->label_fastGPU->setEnabled(false);
		_ui->fastKeypointRatio->setEnabled(false);
		_ui->label_fastGPUKptRatio->setEnabled(false);

		_ui->checkBox_ORBGpu->setChecked(false);
		_ui->checkBox_ORBGpu->setEnabled(false);
		_ui->label_orbGpu->setEnabled(false);

		// remove BruteForceGPU option
		_ui->comboBox_dictionary_strategy->removeItem(4);
		_ui->reextract_nn->removeItem(4);
	}

#ifndef RTABMAP_OCTOMAP
	_ui->groupBox_octomap->setChecked(false);
	_ui->groupBox_octomap->setEnabled(false);
#endif

#ifndef RTABMAP_NONFREE
		_ui->comboBox_detector_strategy->setItemData(0, 0, Qt::UserRole - 1);
		_ui->comboBox_detector_strategy->setItemData(1, 0, Qt::UserRole - 1);
		_ui->reextract_type->setItemData(0, 0, Qt::UserRole - 1);
		_ui->reextract_type->setItemData(1, 0, Qt::UserRole - 1);

#if CV_MAJOR_VERSION == 3
		_ui->comboBox_detector_strategy->setItemData(0, 0, Qt::UserRole - 1);
		_ui->comboBox_detector_strategy->setItemData(1, 0, Qt::UserRole - 1);
		_ui->comboBox_detector_strategy->setItemData(3, 0, Qt::UserRole - 1);
		_ui->comboBox_detector_strategy->setItemData(4, 0, Qt::UserRole - 1);
		_ui->comboBox_detector_strategy->setItemData(5, 0, Qt::UserRole - 1);
		_ui->comboBox_detector_strategy->setItemData(6, 0, Qt::UserRole - 1);
		_ui->reextract_type->setItemData(0, 0, Qt::UserRole - 1);
		_ui->reextract_type->setItemData(1, 0, Qt::UserRole - 1);
		_ui->reextract_type->setItemData(3, 0, Qt::UserRole - 1);
		_ui->reextract_type->setItemData(4, 0, Qt::UserRole - 1);
		_ui->reextract_type->setItemData(5, 0, Qt::UserRole - 1);
		_ui->reextract_type->setItemData(6, 0, Qt::UserRole - 1);
#endif
#endif

#if CV_MAJOR_VERSION >= 3
	_ui->groupBox_fast_opencv2->setEnabled(false);
#else
	_ui->comboBox_detector_strategy->setItemData(9, 0, Qt::UserRole - 1); // No FREAK (detector+descriptor version)
	_ui->reextract_type->setItemData(9, 0, Qt::UserRole - 1); // No FREAK (detector+descriptor version)
#endif

	_ui->comboBox_cameraImages_odomFormat->setItemData(4, 0, Qt::UserRole - 1);
	_ui->comboBox_cameraImages_gtFormat->setItemData(4, 0, Qt::UserRole - 1);
	if(!Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		_ui->graphOptimization_type->setItemData(1, 0, Qt::UserRole - 1);
		_ui->odom_f2m_bundleStrategy->setItemData(1, 0, Qt::UserRole - 1);
		_ui->loopClosure_bundle->setItemData(1, 0, Qt::UserRole - 1);
		_ui->groupBoxx_g2o->setEnabled(false);
	}
	if(!OptimizerG2O::isCSparseAvailable())
	{
		_ui->comboBox_g2o_solver->setItemData(0, 0, Qt::UserRole - 1);
	}
	if(!OptimizerG2O::isCholmodAvailable())
	{
		_ui->comboBox_g2o_solver->setItemData(2, 0, Qt::UserRole - 1);
	}
	if(!Optimizer::isAvailable(Optimizer::kTypeTORO))
	{
		_ui->graphOptimization_type->setItemData(0, 0, Qt::UserRole - 1);
	}
	if(!Optimizer::isAvailable(Optimizer::kTypeGTSAM))
	{
		_ui->graphOptimization_type->setItemData(2, 0, Qt::UserRole - 1);
	}
	if(!Optimizer::isAvailable(Optimizer::kTypeCVSBA))
	{
		_ui->odom_f2m_bundleStrategy->setItemData(2, 0, Qt::UserRole - 1);
		_ui->loopClosure_bundle->setItemData(2, 0, Qt::UserRole - 1);
	}
#ifdef RTABMAP_VERTIGO
	if(!Optimizer::isAvailable(Optimizer::kTypeG2O) && !Optimizer::isAvailable(Optimizer::kTypeGTSAM))
#endif
	{
		_ui->graphOptimization_robust->setEnabled(false);
	}
	if(!CameraOpenni::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(0, 0, Qt::UserRole - 1);
	}
	if(!CameraFreenect::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(1, 0, Qt::UserRole - 1);
	}
	if(!CameraOpenNICV::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(2, 0, Qt::UserRole - 1);
		_ui->comboBox_cameraRGBD->setItemData(3, 0, Qt::UserRole - 1);
	}
	if(!CameraOpenNI2::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(4, 0, Qt::UserRole - 1);
	}
	if(!CameraFreenect2::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(5, 0, Qt::UserRole - 1);
	}
	if (!CameraRealSense::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(6, 0, Qt::UserRole - 1);
	}
	if(!CameraStereoDC1394::available())
	{
		_ui->comboBox_cameraStereo->setItemData(0, 0, Qt::UserRole - 1);
	}
	if(!CameraStereoFlyCapture2::available())
	{
		_ui->comboBox_cameraStereo->setItemData(1, 0, Qt::UserRole - 1);
	}
	if (!CameraStereoZed::available())
	{
		_ui->comboBox_cameraStereo->setItemData(4, 0, Qt::UserRole - 1);
	}
	_ui->openni2_exposure->setEnabled(CameraOpenNI2::exposureGainAvailable());
	_ui->openni2_gain->setEnabled(CameraOpenNI2::exposureGainAvailable());

#if PCL_VERSION_COMPARE(<, 1, 7, 2)
	_ui->checkBox_showFrustums->setEnabled(false);
	_ui->checkBox_showFrustums->setChecked(false);
#endif

	// in case we change the ui, we should not forget to change stuff related to this parameter
	UASSERT(_ui->odom_registration->count() == 4);

	// Default Driver
	connect(_ui->comboBox_sourceType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateSourceGrpVisibility()));
	connect(_ui->comboBox_cameraRGBD, SIGNAL(currentIndexChanged(int)), this, SLOT(updateSourceGrpVisibility()));
	connect(_ui->source_comboBox_image_type, SIGNAL(currentIndexChanged(int)), this, SLOT(updateSourceGrpVisibility()));
	connect(_ui->comboBox_cameraStereo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateSourceGrpVisibility()));
	this->resetSettings(_ui->groupBox_source0);

	_ui->predictionPlot->showLegend(false);

	QButtonGroup * buttonGroup = new QButtonGroup(this);
	buttonGroup->addButton(_ui->radioButton_basic);
	buttonGroup->addButton(_ui->radioButton_advanced);

	// Connect
	connect(_ui->buttonBox_global, SIGNAL(clicked(QAbstractButton *)), this, SLOT(closeDialog(QAbstractButton *)));
	connect(_ui->buttonBox_local, SIGNAL(clicked(QAbstractButton *)), this, SLOT(resetApply(QAbstractButton *)));
	connect(_ui->pushButton_loadConfig, SIGNAL(clicked()), this, SLOT(loadConfigFrom()));
	connect(_ui->pushButton_saveConfig, SIGNAL(clicked()), this, SLOT(saveConfigTo()));
	connect(_ui->pushButton_resetConfig, SIGNAL(clicked()), this, SLOT(resetConfig()));
	connect(_ui->radioButton_basic, SIGNAL(toggled(bool)), this, SLOT(setupTreeView()));
	connect(_ui->pushButton_testOdometry, SIGNAL(clicked()), this, SLOT(testOdometry()));
	connect(_ui->pushButton_test_camera, SIGNAL(clicked()), this, SLOT(testCamera()));

	// General panel
	connect(_ui->general_checkBox_imagesKept, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->general_checkBox_cloudsKept, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_verticalLayoutUsed, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_imageRejectedShown, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_imageHighestHypShown, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_beep, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_stamps, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_cacheStatistics, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_notifyWhenNewGlobalPathIsReceived, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->spinBox_odomQualityWarnThr, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_posteriorGraphView, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkbox_odomDisabled, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->odom_registration, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkbox_groundTruthAlign, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));

	// Cloud rendering panel
	_3dRenderingShowClouds.resize(2);
	_3dRenderingShowClouds[0] = _ui->checkBox_showClouds;
	_3dRenderingShowClouds[1] = _ui->checkBox_showOdomClouds;

	_3dRenderingDecimation.resize(2);
	_3dRenderingDecimation[0] = _ui->spinBox_decimation;
	_3dRenderingDecimation[1] = _ui->spinBox_decimation_odom;

	_3dRenderingMaxDepth.resize(2);
	_3dRenderingMaxDepth[0] = _ui->doubleSpinBox_maxDepth;
	_3dRenderingMaxDepth[1] = _ui->doubleSpinBox_maxDepth_odom;

	_3dRenderingMinDepth.resize(2);
	_3dRenderingMinDepth[0] = _ui->doubleSpinBox_minDepth;
	_3dRenderingMinDepth[1] = _ui->doubleSpinBox_minDepth_odom;

	_3dRenderingRoiRatios.resize(2);
	_3dRenderingRoiRatios[0] = _ui->lineEdit_roiRatios;
	_3dRenderingRoiRatios[1] = _ui->lineEdit_roiRatios_odom;

	_3dRenderingOpacity.resize(2);
	_3dRenderingOpacity[0] = _ui->doubleSpinBox_opacity;
	_3dRenderingOpacity[1] = _ui->doubleSpinBox_opacity_odom;

	_3dRenderingPtSize.resize(2);
	_3dRenderingPtSize[0] = _ui->spinBox_ptsize;
	_3dRenderingPtSize[1] = _ui->spinBox_ptsize_odom;

	_3dRenderingShowScans.resize(2);
	_3dRenderingShowScans[0] = _ui->checkBox_showScans;
	_3dRenderingShowScans[1] = _ui->checkBox_showOdomScans;

	_3dRenderingDownsamplingScan.resize(2);
	_3dRenderingDownsamplingScan[0] = _ui->spinBox_downsamplingScan;
	_3dRenderingDownsamplingScan[1] = _ui->spinBox_downsamplingScan_odom;

	_3dRenderingVoxelSizeScan.resize(2);
	_3dRenderingVoxelSizeScan[0] = _ui->doubleSpinBox_voxelSizeScan;
	_3dRenderingVoxelSizeScan[1] = _ui->doubleSpinBox_voxelSizeScan_odom;

	_3dRenderingOpacityScan.resize(2);
	_3dRenderingOpacityScan[0] = _ui->doubleSpinBox_opacity_scan;
	_3dRenderingOpacityScan[1] = _ui->doubleSpinBox_opacity_odom_scan;

	_3dRenderingPtSizeScan.resize(2);
	_3dRenderingPtSizeScan[0] = _ui->spinBox_ptsize_scan;
	_3dRenderingPtSizeScan[1] = _ui->spinBox_ptsize_odom_scan;

	_3dRenderingShowFeatures.resize(2);
	_3dRenderingShowFeatures[0] = _ui->checkBox_showFeatures;
	_3dRenderingShowFeatures[1] = _ui->checkBox_showOdomFeatures;

	_3dRenderingPtSizeFeatures.resize(2);
	_3dRenderingPtSizeFeatures[0] = _ui->spinBox_ptsize_features;
	_3dRenderingPtSizeFeatures[1] = _ui->spinBox_ptsize_odom_features;

	for(int i=0; i<2; ++i)
	{
		connect(_3dRenderingShowClouds[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingDecimation[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingMaxDepth[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingMinDepth[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingRoiRatios[i], SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingShowScans[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingShowFeatures[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

		connect(_3dRenderingDownsamplingScan[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingVoxelSizeScan[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingOpacity[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingPtSize[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingOpacityScan[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingPtSizeScan[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingPtSizeFeatures[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	}
	connect(_ui->doubleSpinBox_voxel, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_noiseRadius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_noiseMinNeighbors, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_ceilingFilterHeight, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_floorFilterHeight, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_normalKSearch, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->checkBox_showGraphs, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_showFrustums, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_showLabels, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->radioButton_noFiltering, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->radioButton_nodeFiltering, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->radioButton_subtractFiltering, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_cloudFilterRadius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_cloudFilterAngle, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_subtractFilteringMinPts, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_subtractFilteringRadius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_subtractFilteringAngle, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->checkBox_map_shown, SIGNAL(clicked(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_map_resolution, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_map_opacity, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_map_erode, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_map_footprintRadius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->groupBox_octomap, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_octomap_treeDepth, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_octomap_2dgrid, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_octomap_show3dMap, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_octomap_cubeRendering, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->groupBox_organized, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_mesh_angleTolerance, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_mesh_quad, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_mesh_texture, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_mesh_triangleSize, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	//Logging panel
	connect(_ui->comboBox_loggerLevel, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteLoggingPanel()));
	connect(_ui->comboBox_loggerEventLevel, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteLoggingPanel()));
	connect(_ui->comboBox_loggerPauseLevel, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteLoggingPanel()));
	connect(_ui->checkBox_logger_printTime, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteLoggingPanel()));
	connect(_ui->checkBox_logger_printThreadId, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteLoggingPanel()));
	connect(_ui->comboBox_loggerType, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteLoggingPanel()));
	_ui->comboBox_loggerFilter->SetDisplayText("Select:");
	connect(_ui->comboBox_loggerFilter, SIGNAL(itemChanged()), this, SLOT(makeObsoleteLoggingPanel()));

	//Source panel
	connect(_ui->general_doubleSpinBox_imgRate, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_mirroring, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->toolButton_source_path_calibration, SIGNAL(clicked()), this, SLOT(selectCalibrationPath()));
	connect(_ui->lineEdit_calibrationFile, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	_ui->stackedWidget_src->setCurrentIndex(_ui->comboBox_sourceType->currentIndex());
	connect(_ui->comboBox_sourceType, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_src, SLOT(setCurrentIndex(int)));
	connect(_ui->comboBox_sourceType, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_sourceDevice, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_sourceLocalTransform, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));

	//Image source
	_ui->stackedWidget_image->setCurrentIndex(_ui->source_comboBox_image_type->currentIndex());
	connect(_ui->source_comboBox_image_type, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_image, SLOT(setCurrentIndex(int)));
	connect(_ui->source_comboBox_image_type, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//images group
	connect(_ui->source_images_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceImagesPath()));
	connect(_ui->source_images_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_images_spinBox_startPos, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_images_refreshDir, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_rgbImages_rectify, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_cameraImages_bayerMode, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//video group
	connect(_ui->source_video_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceVideoPath()));
	connect(_ui->source_video_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_rgbVideo_rectify, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//database group
	connect(_ui->source_database_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceDatabase()));
	connect(_ui->toolButton_dbViewer, SIGNAL(clicked()), this, SLOT(openDatabaseViewer()));
	connect(_ui->groupBox_sourceDatabase, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_database_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_ignoreOdometry, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_ignoreGoalDelay, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_ignoreGoals, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_databaseStartPos, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_useDbStamps, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_database_cameraIndex, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	//openni group
	_ui->stackedWidget_rgbd->setCurrentIndex(_ui->comboBox_cameraRGBD->currentIndex());
	connect(_ui->comboBox_cameraRGBD, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_rgbd, SLOT(setCurrentIndex(int)));
	connect(_ui->comboBox_cameraRGBD, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	_ui->stackedWidget_stereo->setCurrentIndex(_ui->comboBox_cameraStereo->currentIndex());
	connect(_ui->comboBox_cameraStereo, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_stereo, SLOT(setCurrentIndex(int)));
	connect(_ui->comboBox_cameraStereo, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->openni2_autoWhiteBalance, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->openni2_autoExposure, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->openni2_exposure, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->openni2_gain, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->openni2_mirroring, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->openni2_stampsIdsUsed, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_freenect2Format, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_freenect2MinDepth, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_freenect2MaxDepth, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_freenect2BilateralFiltering, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_freenect2EdgeAwareFiltering, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_freenect2NoiseFiltering, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_realsensePresetRGB, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_realsensePresetDepth, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->toolButton_cameraImages_timestamps, SIGNAL(clicked()), this, SLOT(selectSourceImagesStamps()));
	connect(_ui->lineEdit_cameraImages_timestamps, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->toolButton_cameraRGBDImages_path_rgb, SIGNAL(clicked()), this, SLOT(selectSourceRGBDImagesPathRGB()));
	connect(_ui->toolButton_cameraRGBDImages_path_depth, SIGNAL(clicked()), this, SLOT(selectSourceRGBDImagesPathDepth()));
	connect(_ui->toolButton_cameraImages_path_scans, SIGNAL(clicked()), this, SLOT(selectSourceImagesPathScans()));
	connect(_ui->toolButton_cameraImages_odom, SIGNAL(clicked()), this, SLOT(selectSourceImagesPathOdom()));
	connect(_ui->toolButton_cameraImages_gt, SIGNAL(clicked()), this, SLOT(selectSourceImagesPathGt()));
	connect(_ui->lineEdit_cameraRGBDImages_path_rgb, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraRGBDImages_path_depth, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_cameraImages_timestamps, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_cameraImages_syncTimeStamps, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_cameraRGBDImages_scale, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraImages_path_scans, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraImages_laser_transform, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_cameraImages_max_scan_pts, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_cameraImages_scanDownsampleStep, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_cameraImages_scanVoxelSize, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraImages_odom, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_cameraImages_odomFormat, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraImages_gt, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_cameraImages_gtFormat, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->groupBox_depthFromScan, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->groupBox_depthFromScan_fillHoles, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->radioButton_depthFromScan_vertical, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->radioButton_depthFromScan_horizontal, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_depthFromScan_fillBorders, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->toolButton_cameraStereoImages_path_left, SIGNAL(clicked()), this, SLOT(selectSourceStereoImagesPathLeft()));
	connect(_ui->toolButton_cameraStereoImages_path_right, SIGNAL(clicked()), this, SLOT(selectSourceStereoImagesPathRight()));
	connect(_ui->lineEdit_cameraStereoImages_path_left, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraStereoImages_path_right, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_stereoImages_rectify, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->toolButton_cameraStereoVideo_path, SIGNAL(clicked()), this, SLOT(selectSourceStereoVideoPath()));
	connect(_ui->toolButton_cameraStereoVideo_path_2, SIGNAL(clicked()), this, SLOT(selectSourceStereoVideoPath2()));
	connect(_ui->lineEdit_cameraStereoVideo_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraStereoVideo_path_2, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_stereoVideo_rectify, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->comboBox_stereoZed_resolution, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_stereoZed_quality, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_stereoZed_quality, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStereoDisparityVisibility()));
	connect(_ui->checkbox_stereoZed_selfCalibration, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_cameraStereo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStereoDisparityVisibility()));
	connect(_ui->comboBox_stereoZed_sensingMode, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_stereoZed_confidenceThr, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_stereoZed_odom, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->toolButton_zedSvoPath, SIGNAL(clicked()), this, SLOT(selectSourceSvoPath()));
	connect(_ui->lineEdit_zedSvoPath, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->checkbox_rgbd_colorOnly, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_source_imageDecimation, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_stereo_depthGenerated, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->pushButton_calibrate, SIGNAL(clicked()), this, SLOT(calibrate()));
	connect(_ui->pushButton_calibrate_simple, SIGNAL(clicked()), this, SLOT(calibrateSimple()));
	connect(_ui->toolButton_openniOniPath, SIGNAL(clicked()), this, SLOT(selectSourceOniPath()));
	connect(_ui->toolButton_openni2OniPath, SIGNAL(clicked()), this, SLOT(selectSourceOni2Path()));
	connect(_ui->toolButton_source_distortionModel, SIGNAL(clicked()), this, SLOT(selectSourceDistortionModel()));
	connect(_ui->toolButton_distortionModel, SIGNAL(clicked()), this, SLOT(visualizeDistortionModel()));
	connect(_ui->lineEdit_openniOniPath, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_openni2OniPath, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_source_distortionModel, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->groupBox_bilateral, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_bilateral_sigmaS, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_bilateral_sigmaR, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->groupBox_scanFromDepth, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_cameraScanFromDepth_decimation, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_cameraSCanFromDepth_maxDepth, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_cameraImages_scanVoxelSize, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_cameraImages_scanNormalsK, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	//Rtabmap basic
	connect(_ui->general_doubleSpinBox_timeThr, SIGNAL(valueChanged(double)), _ui->general_doubleSpinBox_timeThr_2, SLOT(setValue(double)));
	connect(_ui->general_doubleSpinBox_hardThr, SIGNAL(valueChanged(double)), _ui->general_doubleSpinBox_hardThr_2, SLOT(setValue(double)));
	connect(_ui->doubleSpinBox_similarityThreshold, SIGNAL(valueChanged(double)), _ui->doubleSpinBox_similarityThreshold_2, SLOT(setValue(double)));
	connect(_ui->general_doubleSpinBox_detectionRate, SIGNAL(valueChanged(double)), _ui->general_doubleSpinBox_detectionRate_2, SLOT(setValue(double)));
	connect(_ui->general_spinBox_imagesBufferSize, SIGNAL(valueChanged(int)), _ui->general_spinBox_imagesBufferSize_2, SLOT(setValue(int)));
	connect(_ui->general_spinBox_maxStMemSize, SIGNAL(valueChanged(int)), _ui->general_spinBox_maxStMemSize_2, SLOT(setValue(int)));

	connect(_ui->general_doubleSpinBox_timeThr_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_doubleSpinBox_hardThr_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->doubleSpinBox_similarityThreshold_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_doubleSpinBox_detectionRate_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_spinBox_imagesBufferSize_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_spinBox_maxStMemSize_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->groupBox_publishing, SIGNAL(toggled(bool)), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_publishStats_2, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_activateRGBD, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_activateRGBD_2, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_SLAM_mode, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_SLAM_mode_2, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));

	// Map objects name with the corresponding parameter key, needed for the addParameter() slots
	//Rtabmap
	_ui->groupBox_publishing->setObjectName(Parameters::kRtabmapPublishStats().c_str());
	_ui->general_checkBox_publishRawData->setObjectName(Parameters::kRtabmapPublishLastSignature().c_str());
	_ui->general_checkBox_publishPdf->setObjectName(Parameters::kRtabmapPublishPdf().c_str());
	_ui->general_checkBox_publishLikelihood->setObjectName(Parameters::kRtabmapPublishLikelihood().c_str());
	_ui->general_checkBox_statisticLogsBufferedInRAM->setObjectName(Parameters::kRtabmapStatisticLogsBufferedInRAM().c_str());
	_ui->groupBox_statistics->setObjectName(Parameters::kRtabmapStatisticLogged().c_str());
	_ui->general_checkBox_statisticLoggedHeaders->setObjectName(Parameters::kRtabmapStatisticLoggedHeaders().c_str());
	_ui->general_doubleSpinBox_timeThr->setObjectName(Parameters::kRtabmapTimeThr().c_str());
	_ui->general_spinBox_memoryThr->setObjectName(Parameters::kRtabmapMemoryThr().c_str());
	_ui->general_doubleSpinBox_detectionRate->setObjectName(Parameters::kRtabmapDetectionRate().c_str());
	_ui->general_spinBox_imagesBufferSize->setObjectName(Parameters::kRtabmapImageBufferSize().c_str());
	_ui->general_checkBox_createIntermediateNodes->setObjectName(Parameters::kRtabmapCreateIntermediateNodes().c_str());
	_ui->general_spinBox_maxRetrieved->setObjectName(Parameters::kRtabmapMaxRetrieved().c_str());
	_ui->general_checkBox_startNewMapOnLoopClosure->setObjectName(Parameters::kRtabmapStartNewMapOnLoopClosure().c_str());
	_ui->lineEdit_workingDirectory->setObjectName(Parameters::kRtabmapWorkingDirectory().c_str());
	connect(_ui->toolButton_workingDirectory, SIGNAL(clicked()), this, SLOT(changeWorkingDirectory()));

	// Memory
	_ui->general_checkBox_keepRawData->setObjectName(Parameters::kMemImageKept().c_str());
	_ui->general_checkBox_keepBinaryData->setObjectName(Parameters::kMemBinDataKept().c_str());
	_ui->general_checkBox_keepDescriptors->setObjectName(Parameters::kMemRawDescriptorsKept().c_str());
	_ui->general_checkBox_saveDepth16bits->setObjectName(Parameters::kMemSaveDepth16Format().c_str());
	_ui->general_checkBox_compressionParallelized->setObjectName(Parameters::kMemCompressionParallelized().c_str());
	_ui->general_checkBox_reduceGraph->setObjectName(Parameters::kMemReduceGraph().c_str());
	_ui->general_checkBox_keepNotLinkedNodes->setObjectName(Parameters::kMemNotLinkedNodesKept().c_str());
	_ui->general_spinBox_maxStMemSize->setObjectName(Parameters::kMemSTMSize().c_str());
	_ui->doubleSpinBox_similarityThreshold->setObjectName(Parameters::kMemRehearsalSimilarity().c_str());
	_ui->general_checkBox_SLAM_mode->setObjectName(Parameters::kMemIncrementalMemory().c_str());
	_ui->general_doubleSpinBox_recentWmRatio->setObjectName(Parameters::kMemRecentWmRatio().c_str());
	_ui->general_checkBox_transferSortingByWeightId->setObjectName(Parameters::kMemTransferSortingByWeightId().c_str());
	_ui->general_checkBox_RehearsalIdUpdatedToNewOne->setObjectName(Parameters::kMemRehearsalIdUpdatedToNewOne().c_str());
	_ui->general_checkBox_generateIds->setObjectName(Parameters::kMemGenerateIds().c_str());
	_ui->general_checkBox_badSignaturesIgnored->setObjectName(Parameters::kMemBadSignaturesIgnored().c_str());
	_ui->general_checkBox_createMapLabels->setObjectName(Parameters::kMemMapLabelsAdded().c_str());
	_ui->general_checkBox_initWMWithAllNodes->setObjectName(Parameters::kMemInitWMWithAllNodes().c_str());
	_ui->checkBox_localSpaceScanMatchingIDsSaved->setObjectName(Parameters::kRGBDScanMatchingIdsSavedInLinks().c_str());
	_ui->spinBox_imagePreDecimation->setObjectName(Parameters::kMemImagePreDecimation().c_str());
	_ui->spinBox_imagePostDecimation->setObjectName(Parameters::kMemImagePostDecimation().c_str());
	_ui->general_spinBox_laserScanDownsample->setObjectName(Parameters::kMemLaserScanDownsampleStepSize().c_str());
	_ui->general_spinBox_laserScanNormalK->setObjectName(Parameters::kMemLaserScanNormalK().c_str());
	_ui->checkBox_useOdomFeatures->setObjectName(Parameters::kMemUseOdomFeatures().c_str());

	// Database
	_ui->checkBox_dbInMemory->setObjectName(Parameters::kDbSqlite3InMemory().c_str());
	_ui->spinBox_dbCacheSize->setObjectName(Parameters::kDbSqlite3CacheSize().c_str());
	_ui->comboBox_dbJournalMode->setObjectName(Parameters::kDbSqlite3JournalMode().c_str());
	_ui->comboBox_dbSynchronous->setObjectName(Parameters::kDbSqlite3Synchronous().c_str());
	_ui->comboBox_dbTempStore->setObjectName(Parameters::kDbSqlite3TempStore().c_str());

	// Create hypotheses
	_ui->general_doubleSpinBox_hardThr->setObjectName(Parameters::kRtabmapLoopThr().c_str());
	_ui->general_doubleSpinBox_loopRatio->setObjectName(Parameters::kRtabmapLoopRatio().c_str());

	//Bayes
	_ui->general_doubleSpinBox_vp->setObjectName(Parameters::kBayesVirtualPlacePriorThr().c_str());
	_ui->lineEdit_bayes_predictionLC->setObjectName(Parameters::kBayesPredictionLC().c_str());
	_ui->checkBox_bayes_fullPredictionUpdate->setObjectName(Parameters::kBayesFullPredictionUpdate().c_str());
	connect(_ui->lineEdit_bayes_predictionLC, SIGNAL(textChanged(const QString &)), this, SLOT(updatePredictionPlot()));

	//Keypoint-based
	_ui->comboBox_dictionary_strategy->setObjectName(Parameters::kKpNNStrategy().c_str());
	_ui->checkBox_dictionary_incremental->setObjectName(Parameters::kKpIncrementalDictionary().c_str());
	_ui->checkBox_kp_incrementalFlann->setObjectName(Parameters::kKpIncrementalFlann().c_str());
	_ui->comboBox_detector_strategy->setObjectName(Parameters::kKpDetectorStrategy().c_str());
	_ui->surf_doubleSpinBox_nndrRatio->setObjectName(Parameters::kKpNndrRatio().c_str());
	_ui->surf_doubleSpinBox_maxDepth->setObjectName(Parameters::kKpMaxDepth().c_str());
	_ui->surf_doubleSpinBox_minDepth->setObjectName(Parameters::kKpMinDepth().c_str());
	_ui->surf_spinBox_wordsPerImageTarget->setObjectName(Parameters::kKpMaxFeatures().c_str());
	_ui->surf_doubleSpinBox_ratioBadSign->setObjectName(Parameters::kKpBadSignRatio().c_str());
	_ui->checkBox_kp_tfIdfLikelihoodUsed->setObjectName(Parameters::kKpTfIdfLikelihoodUsed().c_str());
	_ui->checkBox_kp_parallelized->setObjectName(Parameters::kKpParallelized().c_str());
	_ui->lineEdit_kp_roi->setObjectName(Parameters::kKpRoiRatios().c_str());
	_ui->lineEdit_dictionaryPath->setObjectName(Parameters::kKpDictionaryPath().c_str());
	connect(_ui->toolButton_dictionaryPath, SIGNAL(clicked()), this, SLOT(changeDictionaryPath()));
	_ui->checkBox_kp_newWordsComparedTogether->setObjectName(Parameters::kKpNewWordsComparedTogether().c_str());
	_ui->subpix_winSize_kp->setObjectName(Parameters::kKpSubPixWinSize().c_str());
	_ui->subpix_iterations_kp->setObjectName(Parameters::kKpSubPixIterations().c_str());
	_ui->subpix_eps_kp->setObjectName(Parameters::kKpSubPixEps().c_str());

	_ui->subpix_winSize->setObjectName(Parameters::kKpSubPixWinSize().c_str());
	_ui->subpix_iterations->setObjectName(Parameters::kKpSubPixIterations().c_str());
	_ui->subpix_eps->setObjectName(Parameters::kKpSubPixEps().c_str());

	//SURF detector
	_ui->surf_doubleSpinBox_hessianThr->setObjectName(Parameters::kSURFHessianThreshold().c_str());
	_ui->surf_spinBox_octaves->setObjectName(Parameters::kSURFOctaves().c_str());
	_ui->surf_spinBox_octaveLayers->setObjectName(Parameters::kSURFOctaveLayers().c_str());
	_ui->checkBox_surfExtended->setObjectName(Parameters::kSURFExtended().c_str());
	_ui->surf_checkBox_upright->setObjectName(Parameters::kSURFUpright().c_str());
	_ui->surf_checkBox_gpuVersion->setObjectName(Parameters::kSURFGpuVersion().c_str());
	_ui->surf_doubleSpinBox_gpuKeypointsRatio->setObjectName(Parameters::kSURFGpuKeypointsRatio().c_str());

	//SIFT detector
	_ui->sift_spinBox_nFeatures->setObjectName(Parameters::kSIFTNFeatures().c_str());
	_ui->sift_spinBox_nOctaveLayers->setObjectName(Parameters::kSIFTNOctaveLayers().c_str());
	_ui->sift_doubleSpinBox_contrastThr->setObjectName(Parameters::kSIFTContrastThreshold().c_str());
	_ui->sift_doubleSpinBox_edgeThr->setObjectName(Parameters::kSIFTEdgeThreshold().c_str());
	_ui->sift_doubleSpinBox_sigma->setObjectName(Parameters::kSIFTSigma().c_str());

	//BRIEF descriptor
	_ui->briefBytes->setObjectName(Parameters::kBRIEFBytes().c_str());

	//FAST detector
	_ui->fastSuppressNonMax->setObjectName(Parameters::kFASTNonmaxSuppression().c_str());
	_ui->fastThreshold->setObjectName(Parameters::kFASTThreshold().c_str());
	_ui->fastThresholdMin->setObjectName(Parameters::kFASTMinThreshold().c_str());
	_ui->fastThresholdMax->setObjectName(Parameters::kFASTMaxThreshold().c_str());
	_ui->fastGridRows->setObjectName(Parameters::kFASTGridRows().c_str());
	_ui->fastGridCols->setObjectName(Parameters::kFASTGridCols().c_str());
	_ui->fastGpu->setObjectName(Parameters::kFASTGpu().c_str());
	_ui->fastKeypointRatio->setObjectName(Parameters::kFASTGpuKeypointsRatio().c_str());

	//ORB detector
	_ui->doubleSpinBox_ORBScaleFactor->setObjectName(Parameters::kORBScaleFactor().c_str());
	_ui->spinBox_ORBNLevels->setObjectName(Parameters::kORBNLevels().c_str());
	_ui->spinBox_ORBEdgeThreshold->setObjectName(Parameters::kORBEdgeThreshold().c_str());
	_ui->spinBox_ORBFirstLevel->setObjectName(Parameters::kORBFirstLevel().c_str());
	_ui->spinBox_ORBWTA_K->setObjectName(Parameters::kORBWTA_K().c_str());
	_ui->spinBox_ORBScoreType->setObjectName(Parameters::kORBScoreType().c_str());
	_ui->spinBox_ORBPatchSize->setObjectName(Parameters::kORBPatchSize().c_str());
	_ui->checkBox_ORBGpu->setObjectName(Parameters::kORBGpu().c_str());

	//FREAK descriptor
	_ui->checkBox_FREAKOrientationNormalized->setObjectName(Parameters::kFREAKOrientationNormalized().c_str());
	_ui->checkBox_FREAKScaleNormalized->setObjectName(Parameters::kFREAKScaleNormalized().c_str());
	_ui->doubleSpinBox_FREAKPatternScale->setObjectName(Parameters::kFREAKPatternScale().c_str());
	_ui->spinBox_FREAKNOctaves->setObjectName(Parameters::kFREAKNOctaves().c_str());

	//GFTT detector
	_ui->doubleSpinBox_GFTT_qualityLevel->setObjectName(Parameters::kGFTTQualityLevel().c_str());
	_ui->doubleSpinBox_GFTT_minDistance->setObjectName(Parameters::kGFTTMinDistance().c_str());
	_ui->spinBox_GFTT_blockSize->setObjectName(Parameters::kGFTTBlockSize().c_str());
	_ui->checkBox_GFTT_useHarrisDetector->setObjectName(Parameters::kGFTTUseHarrisDetector().c_str());
	_ui->doubleSpinBox_GFTT_k->setObjectName(Parameters::kGFTTK().c_str());

	//BRISK
	_ui->spinBox_BRISK_thresh->setObjectName(Parameters::kBRISKThresh().c_str());
	_ui->spinBox_BRISK_octaves->setObjectName(Parameters::kBRISKOctaves().c_str());
	_ui->doubleSpinBox_BRISK_patterScale->setObjectName(Parameters::kBRISKPatternScale().c_str());

	// verifyHypotheses
	_ui->comboBox_vh_strategy->setObjectName(Parameters::kRtabmapVhStrategy().c_str());
	_ui->surf_spinBox_matchCountMinAccepted->setObjectName(Parameters::kVhEpMatchCountMin().c_str());
	_ui->surf_doubleSpinBox_ransacParam1->setObjectName(Parameters::kVhEpRansacParam1().c_str());
	_ui->surf_doubleSpinBox_ransacParam2->setObjectName(Parameters::kVhEpRansacParam2().c_str());

	// RGB-D SLAM
	_ui->general_checkBox_activateRGBD->setObjectName(Parameters::kRGBDEnabled().c_str());
	_ui->rgdb_linearUpdate->setObjectName(Parameters::kRGBDLinearUpdate().c_str());
	_ui->rgdb_angularUpdate->setObjectName(Parameters::kRGBDAngularUpdate().c_str());
	_ui->rgdb_rehearsalWeightIgnoredWhileMoving->setObjectName(Parameters::kMemRehearsalWeightIgnoredWhileMoving().c_str());
	_ui->rgdb_newMapOdomChange->setObjectName(Parameters::kRGBDNewMapOdomChangeDistance().c_str());
	_ui->odomScanHistory->setObjectName(Parameters::kRGBDNeighborLinkRefining().c_str());
	_ui->spinBox_maxLocalLocationsRetrieved->setObjectName(Parameters::kRGBDMaxLocalRetrieved().c_str());

	_ui->graphOptimization_type->setObjectName(Parameters::kOptimizerStrategy().c_str());
	_ui->graphOptimization_iterations->setObjectName(Parameters::kOptimizerIterations().c_str());
	_ui->graphOptimization_covarianceIgnored->setObjectName(Parameters::kOptimizerVarianceIgnored().c_str());
	_ui->graphOptimization_fromGraphEnd->setObjectName(Parameters::kRGBDOptimizeFromGraphEnd().c_str());
	_ui->graphOptimization_maxError->setObjectName(Parameters::kRGBDOptimizeMaxError().c_str());
	_ui->graphOptimization_stopEpsilon->setObjectName(Parameters::kOptimizerEpsilon().c_str());
	_ui->graphOptimization_robust->setObjectName(Parameters::kOptimizerRobust().c_str());

	_ui->comboBox_g2o_solver->setObjectName(Parameters::kg2oSolver().c_str());
	_ui->comboBox_g2o_optimizer->setObjectName(Parameters::kg2oOptimizer().c_str());
	_ui->doubleSpinBox_g2o_pixelVariance->setObjectName(Parameters::kg2oPixelVariance().c_str());
	_ui->doubleSpinBox_g2o_robustKernelDelta->setObjectName(Parameters::kg2oRobustKernelDelta().c_str());
	_ui->doubleSpinBox_g2o_baseline->setObjectName(Parameters::kg2oBaseline().c_str());

	_ui->comboBox_gtsam_optimizer->setObjectName(Parameters::kGTSAMOptimizer().c_str());

	_ui->graphPlan_goalReachedRadius->setObjectName(Parameters::kRGBDGoalReachedRadius().c_str());
	_ui->graphPlan_goalsSavedInUserData->setObjectName(Parameters::kRGBDGoalsSavedInUserData().c_str());
	_ui->graphPlan_stuckIterations->setObjectName(Parameters::kRGBDPlanStuckIterations().c_str());
	_ui->graphPlan_linearVelocity->setObjectName(Parameters::kRGBDPlanLinearVelocity().c_str());
	_ui->graphPlan_angularVelocity->setObjectName(Parameters::kRGBDPlanAngularVelocity().c_str());

	_ui->groupBox_localDetection_time->setObjectName(Parameters::kRGBDProximityByTime().c_str());
	_ui->groupBox_localDetection_space->setObjectName(Parameters::kRGBDProximityBySpace().c_str());
	_ui->localDetection_radius->setObjectName(Parameters::kRGBDLocalRadius().c_str());
	_ui->localDetection_maxDiffID->setObjectName(Parameters::kRGBDProximityMaxGraphDepth().c_str());
	_ui->localDetection_maxNeighbors->setObjectName(Parameters::kRGBDProximityPathMaxNeighbors().c_str());
	_ui->localDetection_maxPaths->setObjectName(Parameters::kRGBDProximityMaxPaths().c_str());
	_ui->localDetection_pathFilteringRadius->setObjectName(Parameters::kRGBDProximityPathFilteringRadius().c_str());
	_ui->localDetection_angle->setObjectName(Parameters::kRGBDProximityAngle().c_str());
	_ui->checkBox_localSpacePathOdomPosesUsed->setObjectName(Parameters::kRGBDProximityPathRawPosesUsed().c_str());
	_ui->rgdb_localImmunizationRatio->setObjectName(Parameters::kRGBDLocalImmunizationRatio().c_str());
	_ui->loopClosure_reextract->setObjectName(Parameters::kRGBDLoopClosureReextractFeatures().c_str());
	_ui->checkbox_rgbd_createOccupancyGrid->setObjectName(Parameters::kRGBDCreateOccupancyGrid().c_str());

	// Registration
	_ui->loopClosure_bowVarianceFromInliersCount->setObjectName(Parameters::kRegVarianceFromInliersCount().c_str());
	_ui->comboBox_registrationStrategy->setObjectName(Parameters::kRegStrategy().c_str());
	_ui->loopClosure_bowForce2D->setObjectName(Parameters::kRegForce3DoF().c_str());

	//RegistrationVis
	_ui->loopClosure_bowMinInliers->setObjectName(Parameters::kVisMinInliers().c_str());
	_ui->loopClosure_bowInlierDistance->setObjectName(Parameters::kVisInlierDistance().c_str());
	_ui->loopClosure_bowIterations->setObjectName(Parameters::kVisIterations().c_str());
	_ui->loopClosure_bowRefineIterations->setObjectName(Parameters::kVisRefineIterations().c_str());
	_ui->loopClosure_estimationType->setObjectName(Parameters::kVisEstimationType().c_str());
	connect(_ui->loopClosure_estimationType, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_loopClosureEstimation, SLOT(setCurrentIndex(int)));
	_ui->stackedWidget_loopClosureEstimation->setCurrentIndex(Parameters::defaultVisEstimationType());
	_ui->loopClosure_forwardEst->setObjectName(Parameters::kVisForwardEstOnly().c_str());
	_ui->loopClosure_bowEpipolarGeometryVar->setObjectName(Parameters::kVisEpipolarGeometryVar().c_str());
	_ui->loopClosure_pnpReprojError->setObjectName(Parameters::kVisPnPReprojError().c_str());
	_ui->loopClosure_pnpFlags->setObjectName(Parameters::kVisPnPFlags().c_str());
	_ui->loopClosure_pnpRefineIterations->setObjectName(Parameters::kVisPnPRefineIterations().c_str());
	_ui->loopClosure_correspondencesType->setObjectName(Parameters::kVisCorType().c_str());
	connect(_ui->loopClosure_correspondencesType, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_loopClosureCorrespondences, SLOT(setCurrentIndex(int)));
	_ui->stackedWidget_loopClosureCorrespondences->setCurrentIndex(Parameters::defaultVisCorType());
	_ui->reextract_nn->setObjectName(Parameters::kVisCorNNType().c_str());
	_ui->reextract_nndrRatio->setObjectName(Parameters::kVisCorNNDR().c_str());
	_ui->spinBox_visCorGuessWinSize->setObjectName(Parameters::kVisCorGuessWinSize().c_str());
	_ui->reextract_type->setObjectName(Parameters::kVisFeatureType().c_str());
	_ui->reextract_maxFeatures->setObjectName(Parameters::kVisMaxFeatures().c_str());
	_ui->loopClosure_bowMaxDepth->setObjectName(Parameters::kVisMaxDepth().c_str());
	_ui->loopClosure_bowMinDepth->setObjectName(Parameters::kVisMinDepth().c_str());
	_ui->loopClosure_roi->setObjectName(Parameters::kVisRoiRatios().c_str());
	_ui->subpix_winSize->setObjectName(Parameters::kVisSubPixWinSize().c_str());
	_ui->subpix_iterations->setObjectName(Parameters::kVisSubPixIterations().c_str());
	_ui->subpix_eps->setObjectName(Parameters::kVisSubPixEps().c_str());
	_ui->odom_flow_winSize_2->setObjectName(Parameters::kVisCorFlowWinSize().c_str());
	_ui->odom_flow_maxLevel_2->setObjectName(Parameters::kVisCorFlowMaxLevel().c_str());
	_ui->odom_flow_iterations_2->setObjectName(Parameters::kVisCorFlowIterations().c_str());
	_ui->odom_flow_eps_2->setObjectName(Parameters::kVisCorFlowEps().c_str());
	_ui->loopClosure_bundle->setObjectName(Parameters::kVisBundleAdjustment().c_str());

	//RegistrationIcp
	_ui->globalDetection_icpMaxTranslation->setObjectName(Parameters::kIcpMaxTranslation().c_str());
	_ui->globalDetection_icpMaxRotation->setObjectName(Parameters::kIcpMaxRotation().c_str());
	_ui->loopClosure_icpVoxelSize->setObjectName(Parameters::kIcpVoxelSize().c_str());
	_ui->loopClosure_icpDownsamplingStep->setObjectName(Parameters::kIcpDownsamplingStep().c_str());
	_ui->loopClosure_icpMaxCorrespondenceDistance->setObjectName(Parameters::kIcpMaxCorrespondenceDistance().c_str());
	_ui->loopClosure_icpIterations->setObjectName(Parameters::kIcpIterations().c_str());
	_ui->loopClosure_icpEpsilon->setObjectName(Parameters::kIcpEpsilon().c_str());
	_ui->loopClosure_icpRatio->setObjectName(Parameters::kIcpCorrespondenceRatio().c_str());
	_ui->loopClosure_icpPointToPlane->setObjectName(Parameters::kIcpPointToPlane().c_str());
	_ui->loopClosure_icpPointToPlaneNormals->setObjectName(Parameters::kIcpPointToPlaneNormalNeighbors().c_str());

	// Occupancy grid
	_ui->groupBox_grid_3d->setObjectName(Parameters::kGrid3D().c_str());
	_ui->checkBox_grid_groundObstacle->setObjectName(Parameters::kGridGroundIsObstacle().c_str());
	_ui->doubleSpinBox_grid_resolution->setObjectName(Parameters::kGridCellSize().c_str());
	_ui->spinBox_grid_decimation->setObjectName(Parameters::kGridDepthDecimation().c_str());
	_ui->doubleSpinBox_grid_maxDepth->setObjectName(Parameters::kGridDepthMax().c_str());
	_ui->doubleSpinBox_grid_minDepth->setObjectName(Parameters::kGridDepthMin().c_str());
	_ui->lineEdit_grid_roi->setObjectName(Parameters::kGridDepthRoiRatios().c_str());
	_ui->checkBox_grid_projRayTracing->setObjectName(Parameters::kGridProjRayTracing().c_str());
	connect(_ui->checkBox_grid_projRayTracing, SIGNAL(stateChanged(int)), this, SLOT(useGridProjRayTracing()));
	_ui->doubleSpinBox_grid_footprintLength->setObjectName(Parameters::kGridFootprintLength().c_str());
	_ui->doubleSpinBox_grid_footprintWidth->setObjectName(Parameters::kGridFootprintWidth().c_str());
	_ui->doubleSpinBox_grid_footprintHeight->setObjectName(Parameters::kGridFootprintHeight().c_str());
	_ui->checkBox_grid_flatObstaclesDetected->setObjectName(Parameters::kGridFlatObstacleDetected().c_str());
	_ui->groupBox_grid_fromDepthImage->setObjectName(Parameters::kGridFromDepth().c_str());
	_ui->checkBox_grid_projMapFrame->setObjectName(Parameters::kGridMapFrameProjection().c_str());
	_ui->doubleSpinBox_grid_maxGroundAngle->setObjectName(Parameters::kGridMaxGroundAngle().c_str());
	_ui->spinBox_grid_normalK->setObjectName(Parameters::kGridNormalK().c_str());
	_ui->doubleSpinBox_grid_maxGroundHeight->setObjectName(Parameters::kGridMaxGroundHeight().c_str());
	_ui->doubleSpinBox_grid_maxObstacleHeight->setObjectName(Parameters::kGridMaxObstacleHeight().c_str());
	_ui->doubleSpinBox_grid_clusterRadius->setObjectName(Parameters::kGridClusterRadius().c_str());
	_ui->spinBox_grid_minClusterSize->setObjectName(Parameters::kGridMinClusterSize().c_str());
	_ui->doubleSpinBox_grid_minGroundHeight->setObjectName(Parameters::kGridMinGroundHeight().c_str());
	_ui->spinBox_grid_noiseMinNeighbors->setObjectName(Parameters::kGridNoiseFilteringMinNeighbors().c_str());
	_ui->doubleSpinBox_grid_noiseRadius->setObjectName(Parameters::kGridNoiseFilteringRadius().c_str());
	_ui->groupBox_grid_normalsSegmentation->setObjectName(Parameters::kGridNormalsSegmentation().c_str());
	_ui->checkBox_grid_unknownSpaceFilled->setObjectName(Parameters::kGridScan2dUnknownSpaceFilled().c_str());
	_ui->doubleSpinBox_grid_unknownSpaceFilledMaxRange->setObjectName(Parameters::kGridScan2dMaxFilledRange().c_str());
	_ui->spinBox_grid_scanDecimation->setObjectName(Parameters::kGridScanDecimation().c_str());

	//Odometry
	_ui->odom_strategy->setObjectName(Parameters::kOdomStrategy().c_str());
	connect(_ui->odom_strategy, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_odometryType, SLOT(setCurrentIndex(int)));
	_ui->odom_strategy->setCurrentIndex(Parameters::defaultOdomStrategy());
	_ui->odom_countdown->setObjectName(Parameters::kOdomResetCountdown().c_str());
	_ui->odom_holonomic->setObjectName(Parameters::kOdomHolonomic().c_str());
	_ui->odom_fillInfoData->setObjectName(Parameters::kOdomFillInfoData().c_str());
	_ui->odom_dataBufferSize->setObjectName(Parameters::kOdomImageBufferSize().c_str());
	_ui->odom_flow_keyframeThr->setObjectName(Parameters::kOdomKeyFrameThr().c_str());
	_ui->odom_VisKeyFrameThr->setObjectName(Parameters::kOdomVisKeyFrameThr().c_str());
	_ui->odom_flow_scanKeyframeThr->setObjectName(Parameters::kOdomScanKeyFrameThr().c_str());
	_ui->odom_flow_guessMotion->setObjectName(Parameters::kOdomGuessMotion().c_str());
	_ui->odom_imageDecimation->setObjectName(Parameters::kOdomImageDecimation().c_str());
	_ui->odom_alignWithGround->setObjectName(Parameters::kOdomAlignWithGround().c_str());

	//Odometry Frame to Map
	_ui->odom_localHistory->setObjectName(Parameters::kOdomF2MMaxSize().c_str());
	_ui->spinBox_odom_f2m_maxNewFeatures->setObjectName(Parameters::kOdomF2MMaxNewFeatures().c_str());
	_ui->spinBox_odom_f2m_scanMaxSize->setObjectName(Parameters::kOdomF2MScanMaxSize().c_str());
	_ui->doubleSpinBox_odom_f2m_scanRadius->setObjectName(Parameters::kOdomF2MScanSubtractRadius().c_str());
	_ui->odom_f2m_bundleStrategy->setObjectName(Parameters::kOdomF2MBundleAdjustment().c_str());
	_ui->odom_f2m_bundleMaxFrames->setObjectName(Parameters::kOdomF2MBundleAdjustmentMaxFrames().c_str());

	//Odometry Mono
	_ui->doubleSpinBox_minFlow->setObjectName(Parameters::kOdomMonoInitMinFlow().c_str());
	_ui->doubleSpinBox_minInitTranslation->setObjectName(Parameters::kOdomMonoInitMinTranslation().c_str());
	_ui->doubleSpinBox_minTranslation->setObjectName(Parameters::kOdomMonoMinTranslation().c_str());
	_ui->doubleSpinBox_maxVariance->setObjectName(Parameters::kOdomMonoMaxVariance().c_str());

	//Odometry particle filter
	_ui->odom_filteringStrategy->setObjectName(Parameters::kOdomFilteringStrategy().c_str());
	_ui->stackedWidget_odometryFiltering->setCurrentIndex(_ui->odom_filteringStrategy->currentIndex());
	connect(_ui->odom_filteringStrategy, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_odometryFiltering, SLOT(setCurrentIndex(int)));
	_ui->spinBox_particleSize->setObjectName(Parameters::kOdomParticleSize().c_str());
	_ui->doubleSpinBox_particleNoiseT->setObjectName(Parameters::kOdomParticleNoiseT().c_str());
	_ui->doubleSpinBox_particleLambdaT->setObjectName(Parameters::kOdomParticleLambdaT().c_str());
	_ui->doubleSpinBox_particleNoiseR->setObjectName(Parameters::kOdomParticleNoiseR().c_str());
	_ui->doubleSpinBox_particleLambdaR->setObjectName(Parameters::kOdomParticleLambdaR().c_str());

	//Odometry Kalman filter
	_ui->doubleSpinBox_kalmanProcessNoise->setObjectName(Parameters::kOdomKalmanProcessNoise().c_str());
	_ui->doubleSpinBox_kalmanMeasurementNoise->setObjectName(Parameters::kOdomKalmanMeasurementNoise().c_str());

	//Stereo
	_ui->stereo_winWidth->setObjectName(Parameters::kStereoWinWidth().c_str());
	_ui->stereo_winHeight->setObjectName(Parameters::kStereoWinHeight().c_str());
	_ui->stereo_maxLevel->setObjectName(Parameters::kStereoMaxLevel().c_str());
	_ui->stereo_iterations->setObjectName(Parameters::kStereoIterations().c_str());
	_ui->stereo_minDisparity->setObjectName(Parameters::kStereoMinDisparity().c_str());
	_ui->stereo_maxDisparity->setObjectName(Parameters::kStereoMaxDisparity().c_str());
	_ui->stereo_ssd->setObjectName(Parameters::kStereoSSD().c_str());
	_ui->stereo_flow_eps->setObjectName(Parameters::kStereoEps().c_str());
	_ui->stereo_opticalFlow->setObjectName(Parameters::kStereoOpticalFlow().c_str());

	//StereoBM
	_ui->stereobm_blockSize->setObjectName(Parameters::kStereoBMBlockSize().c_str());
	_ui->stereobm_minDisparity->setObjectName(Parameters::kStereoBMMinDisparity().c_str());
	_ui->stereobm_numDisparities->setObjectName(Parameters::kStereoBMNumDisparities().c_str());
	_ui->stereobm_preFilterCap->setObjectName(Parameters::kStereoBMPreFilterCap().c_str());
	_ui->stereobm_preFilterSize->setObjectName(Parameters::kStereoBMPreFilterSize().c_str());
	_ui->stereobm_speckleRange->setObjectName(Parameters::kStereoBMSpeckleRange().c_str());
	_ui->stereobm_speckleWinSize->setObjectName(Parameters::kStereoBMSpeckleWindowSize().c_str());
	_ui->stereobm_tetureThreshold->setObjectName(Parameters::kStereoBMTextureThreshold().c_str());
	_ui->stereobm_uniquessRatio->setObjectName(Parameters::kStereoBMUniquenessRatio().c_str());

	// reset default settings for the gui
	resetSettings(_ui->groupBox_generalSettingsGui0);
	resetSettings(_ui->groupBox_cloudRendering1);
	resetSettings(_ui->groupBox_filtering2);
	resetSettings(_ui->groupBox_gridMap2);
	resetSettings(_ui->groupBox_logging1);
	resetSettings(_ui->groupBox_source0);

	setupSignals();
	// custom signals
	connect(_ui->doubleSpinBox_kp_roi0, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));
	connect(_ui->doubleSpinBox_kp_roi1, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));
	connect(_ui->doubleSpinBox_kp_roi2, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));
	connect(_ui->doubleSpinBox_kp_roi3, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));
	connect(_ui->checkBox_useOdomFeatures, SIGNAL(toggled(bool)), this, SLOT(useOdomFeatures()));

	//Create a model from the stacked widgets
	// This will add all parameters to the parameters Map
	_ui->stackedWidget->setCurrentIndex(0);
	this->setupTreeView();

	_obsoletePanels = kPanelAll;
}

PreferencesDialog::~PreferencesDialog() {
	// remove tmp ini file
	QFile::remove(getTmpIniFilePath());
	delete _ui;
}

void PreferencesDialog::init()
{
	UDEBUG("");
	//First set all default values
	const ParametersMap & defaults = Parameters::getDefaultParameters();
	for(ParametersMap::const_iterator iter=defaults.begin(); iter!=defaults.end(); ++iter)
	{
		this->setParameter(iter->first, iter->second);
	}

	this->readSettings();
	this->writeSettings(getTmpIniFilePath());

	_initialized = true;
}

void PreferencesDialog::setCurrentPanelToSource()
{
	QList<QGroupBox*> boxes = this->getGroupBoxes();
	for(int i =0;i<boxes.size();++i)
	{
		if(boxes[i] == _ui->groupBox_source0)
		{
			_ui->stackedWidget->setCurrentIndex(i);
			_ui->treeView->setCurrentIndex(_indexModel->index(i-2, 0));
			break;
		}
	}
}

void PreferencesDialog::saveSettings()
{
	writeSettings();
}

void PreferencesDialog::setupTreeView()
{
	if(_indexModel)
	{
		_ui->treeView->setModel(0);
		delete _indexModel;
	}
	_indexModel = new QStandardItemModel(this);
	// Parse the model
	QList<QGroupBox*> boxes = this->getGroupBoxes();
	if(_ui->radioButton_basic->isChecked())
	{
		boxes = boxes.mid(0,7);
	}
	else // Advanced
	{
		boxes.removeAt(6);
	}

	QStandardItem * parentItem = _indexModel->invisibleRootItem();
	int index = 0;
	this->parseModel(boxes, parentItem, 0, index); // recursive method
	if(_ui->radioButton_advanced->isChecked() && index != _ui->stackedWidget->count()-1)
	{
		ULOGGER_ERROR("The tree model is not the same size of the stacked widgets...%d vs %d advanced stacks", index, _ui->stackedWidget->count()-1);
	}
	int currentIndex = _ui->stackedWidget->currentIndex();
	if(_ui->radioButton_basic->isChecked())
	{
		if(currentIndex >= 6)
		{
			_ui->stackedWidget->setCurrentIndex(6);
			currentIndex = 6;
		}
	}
	else // Advanced
	{
		if(currentIndex == 6)
		{
			_ui->stackedWidget->setCurrentIndex(7);
		}
	}
	_ui->treeView->setModel(_indexModel);
	_ui->treeView->expandToDepth(1);

	// should be after setModel()
	connect(_ui->treeView->selectionModel(), SIGNAL(currentChanged(const QModelIndex &, const QModelIndex &)), this, SLOT(clicked(const QModelIndex &, const QModelIndex &)));
}

// recursive...
bool PreferencesDialog::parseModel(QList<QGroupBox*> & boxes, QStandardItem * parentItem, int currentLevel, int & absoluteIndex)
{
	if(parentItem == 0)
	{
		ULOGGER_ERROR("Parent item is null !");
		return false;
	}

	QStandardItem * currentItem = 0;
	while(absoluteIndex < boxes.size())
	{
		QString objectName = boxes.at(absoluteIndex)->objectName();
		QString title = boxes.at(absoluteIndex)->title();
		bool ok = false;
		int lvl = QString(objectName.at(objectName.size()-1)).toInt(&ok);
		if(!ok)
		{
			ULOGGER_ERROR("Error while parsing the first number of the QGroupBox title (%s), the first character must be the number in the hierarchy", title.toStdString().c_str());
			return false;
		}


		if(lvl == currentLevel)
		{
			QStandardItem * item = new QStandardItem(title);
			item->setData(absoluteIndex);
			currentItem = item;
			//ULOGGER_DEBUG("PreferencesDialog::parseModel() lvl(%d) Added %s", currentLevel, title.toStdString().c_str());
			parentItem->appendRow(item);
			++absoluteIndex;
		}
		else if(lvl > currentLevel)
		{
			if(lvl>currentLevel+1)
			{
				ULOGGER_ERROR("Intermediary lvl doesn't exist, lvl %d to %d, indexes %d and %d", currentLevel, lvl, absoluteIndex-1, absoluteIndex);
				return false;
			}
			else
			{
				parseModel(boxes, currentItem, currentLevel+1, absoluteIndex); // recursive
			}
		}
		else
		{
			return false;
		}
	}
	return true;
}

void PreferencesDialog::setupSignals()
{
	const rtabmap::ParametersMap & parameters = Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		QWidget * obj = _ui->stackedWidget->findChild<QWidget*>((*iter).first.c_str());
		if(obj)
		{
			// set tooltip as the parameter name
			obj->setToolTip(tr("%1 (Default=\"%2\")").arg(iter->first.c_str()).arg(iter->second.c_str()));

			QSpinBox * spin = qobject_cast<QSpinBox *>(obj);
			QDoubleSpinBox * doubleSpin = qobject_cast<QDoubleSpinBox *>(obj);
			QComboBox * combo = qobject_cast<QComboBox *>(obj);
			QCheckBox * check = qobject_cast<QCheckBox *>(obj);
			QRadioButton * radio = qobject_cast<QRadioButton *>(obj);
			QLineEdit * lineEdit = qobject_cast<QLineEdit *>(obj);
			QGroupBox * groupBox = qobject_cast<QGroupBox *>(obj);
			if(spin)
			{
				connect(spin, SIGNAL(valueChanged(int)), this, SLOT(addParameter(int)));
			}
			else if(doubleSpin)
			{
				connect(doubleSpin, SIGNAL(valueChanged(double)), this, SLOT(addParameter(double)));
			}
			else if(combo)
			{
				connect(combo, SIGNAL(currentIndexChanged(int)), this, SLOT(addParameter(int)));
			}
			else if(check)
			{
				connect(check, SIGNAL(toggled(bool)), this, SLOT(addParameter(bool)));
			}
			else if(radio)
			{
				connect(radio, SIGNAL(toggled(bool)), this, SLOT(addParameter(bool)));
			}
			else if(lineEdit)
			{
				connect(lineEdit, SIGNAL(textChanged(const QString &)), this, SLOT(addParameter(const QString &)));
			}
			else if(groupBox)
			{
				connect(groupBox, SIGNAL(toggled(bool)), this, SLOT(addParameter(bool)));
			}
			else
			{
				ULOGGER_WARN("QWidget called %s can't be cast to a supported widget", (*iter).first.c_str());
			}
		}
		else
		{
			ULOGGER_WARN("Can't find the related QWidget for parameter %s", (*iter).first.c_str());
		}
	}
}

void PreferencesDialog::clicked(const QModelIndex & current, const QModelIndex & previous)
 {
	QStandardItem * item = _indexModel->itemFromIndex(current);
	if(item && item->isEnabled())
	{
		int index = item->data().toInt();
		if(_ui->radioButton_advanced->isChecked() && index >= 6)
		{
			++index;
		}
		_ui->stackedWidget->setCurrentIndex(index);
		_ui->scrollArea->horizontalScrollBar()->setValue(0);
		_ui->scrollArea->verticalScrollBar()->setValue(0);
	}
 }

void PreferencesDialog::closeEvent(QCloseEvent *event)
{
	UDEBUG("");
	_modifiedParameters.clear();
	_obsoletePanels = kPanelDummy;
	this->readGuiSettings(getTmpIniFilePath());
	this->readCameraSettings(getTmpIniFilePath());
	event->accept();
}

void PreferencesDialog::closeDialog ( QAbstractButton * button )
{
	UDEBUG("");

	QDialogButtonBox::ButtonRole role = _ui->buttonBox_global->buttonRole(button);
	switch(role)
	{
	case QDialogButtonBox::RejectRole:
		_modifiedParameters.clear();
		_obsoletePanels = kPanelDummy;
		this->readGuiSettings(getTmpIniFilePath());
		this->readCameraSettings(getTmpIniFilePath());
		this->reject();
		break;

	case QDialogButtonBox::AcceptRole:
		updateBasicParameter();// make that changes without editing finished signal are updated.
		if((_obsoletePanels & kPanelAll) || _modifiedParameters.size())
		{
			if(validateForm())
			{
				writeSettings(getTmpIniFilePath());
				this->accept();
			}
		}
		else
		{
			this->accept();
		}
		break;

	default:
		break;
	}
}

void PreferencesDialog::resetApply ( QAbstractButton * button )
{
	QDialogButtonBox::ButtonRole role = _ui->buttonBox_local->buttonRole(button);
	switch(role)
	{
	case QDialogButtonBox::ApplyRole:
		updateBasicParameter();// make that changes without editing finished signal are updated.
		if(validateForm())
		{
			writeSettings(getTmpIniFilePath());
		}
		break;

	case QDialogButtonBox::ResetRole:
		resetSettings(_ui->stackedWidget->currentIndex());
		break;

	default:
		break;
	}
}

void PreferencesDialog::resetSettings(QGroupBox * groupBox)
{
	if(groupBox->objectName() == _ui->groupBox_generalSettingsGui0->objectName())
	{
		_ui->general_checkBox_imagesKept->setChecked(true);
		_ui->general_checkBox_cloudsKept->setChecked(true);
		_ui->checkBox_beep->setChecked(false);
		_ui->checkBox_stamps->setChecked(true);
		_ui->checkBox_cacheStatistics->setChecked(true);
		_ui->checkBox_notifyWhenNewGlobalPathIsReceived->setChecked(false);
		_ui->checkBox_verticalLayoutUsed->setChecked(true);
		_ui->checkBox_imageRejectedShown->setChecked(true);
		_ui->checkBox_imageHighestHypShown->setChecked(false);
		_ui->spinBox_odomQualityWarnThr->setValue(50);
		_ui->checkBox_posteriorGraphView->setChecked(true);
		_ui->checkbox_odomDisabled->setChecked(false);
		_ui->checkbox_groundTruthAlign->setChecked(true);
	}
	else if(groupBox->objectName() == _ui->groupBox_cloudRendering1->objectName())
	{
		for(int i=0; i<2; ++i)
		{
			_3dRenderingShowClouds[i]->setChecked(true);
			_3dRenderingDecimation[i]->setValue(4);
			_3dRenderingMaxDepth[i]->setValue(0.0);
			_3dRenderingMinDepth[i]->setValue(0.0);
			_3dRenderingRoiRatios[i]->setText("0.0 0.0 0.0 0.0");
			_3dRenderingShowScans[i]->setChecked(true);
			_3dRenderingShowFeatures[i]->setChecked(i==0?false:true);

			_3dRenderingDownsamplingScan[i]->setValue(1);
			_3dRenderingVoxelSizeScan[i]->setValue(0.0);
			_3dRenderingOpacity[i]->setValue(i==0?1.0:0.75);
			_3dRenderingPtSize[i]->setValue(2);
			_3dRenderingOpacityScan[i]->setValue(i==0?1.0:0.5);
			_3dRenderingPtSizeScan[i]->setValue(2);
			_3dRenderingPtSizeFeatures[i]->setValue(3);
		}
		_ui->doubleSpinBox_voxel->setValue(0);
		_ui->doubleSpinBox_noiseRadius->setValue(0);
		_ui->spinBox_noiseMinNeighbors->setValue(5);

		_ui->doubleSpinBox_ceilingFilterHeight->setValue(0);
		_ui->doubleSpinBox_floorFilterHeight->setValue(0);

		_ui->checkBox_showGraphs->setChecked(true);
		_ui->checkBox_showFrustums->setChecked(false);
		_ui->checkBox_showLabels->setChecked(false);

		_ui->spinBox_normalKSearch->setValue(10);

		_ui->doubleSpinBox_mesh_angleTolerance->setValue(15.0);
		_ui->groupBox_organized->setChecked(false);
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
		_ui->checkBox_mesh_quad->setChecked(true);
#else
		_ui->checkBox_mesh_quad->setChecked(false);
#endif
		_ui->checkBox_mesh_texture->setChecked(false);
		_ui->spinBox_mesh_triangleSize->setValue(2);
	}
	else if(groupBox->objectName() == _ui->groupBox_filtering2->objectName())
	{
		_ui->radioButton_noFiltering->setChecked(true);
		_ui->radioButton_nodeFiltering->setChecked(false);
		_ui->radioButton_subtractFiltering->setChecked(false);
		_ui->doubleSpinBox_cloudFilterRadius->setValue(0.1);
		_ui->doubleSpinBox_cloudFilterAngle->setValue(30);
		_ui->spinBox_subtractFilteringMinPts->setValue(5);
		_ui->doubleSpinBox_subtractFilteringRadius->setValue(0.02);
		_ui->doubleSpinBox_subtractFilteringAngle->setValue(0);
	}
	else if(groupBox->objectName() == _ui->groupBox_gridMap2->objectName())
	{
		_ui->checkBox_map_shown->setChecked(false);
		_ui->doubleSpinBox_map_resolution->setValue(0.05);
		_ui->checkBox_map_erode->setChecked(false);
		_ui->checkBox_map_incremental->setChecked(false);
		_ui->doubleSpinBox_map_footprintRadius->setValue(0);
		_ui->doubleSpinBox_map_opacity->setValue(0.75);

		_ui->groupBox_octomap->setChecked(false);
		_ui->spinBox_octomap_treeDepth->setValue(16);
		_ui->checkBox_octomap_2dgrid->setChecked(true);
		_ui->checkBox_octomap_show3dMap->setChecked(true);
		_ui->checkBox_octomap_cubeRendering->setChecked(true);
		_ui->doubleSpinBox_octomap_occupancyThr->setValue(0.5);
	}
	else if(groupBox->objectName() == _ui->groupBox_logging1->objectName())
	{
		_ui->comboBox_loggerLevel->setCurrentIndex(2);
		_ui->comboBox_loggerEventLevel->setCurrentIndex(3);
		_ui->comboBox_loggerPauseLevel->setCurrentIndex(3);
		_ui->checkBox_logger_printTime->setChecked(true);
		_ui->checkBox_logger_printThreadId->setChecked(false);
		_ui->comboBox_loggerType->setCurrentIndex(1);
		for(int i=0; i<_ui->comboBox_loggerFilter->count(); ++i)
		{
			_ui->comboBox_loggerFilter->setItemChecked(i, false);
		}
	}
	else if(groupBox->objectName() == _ui->groupBox_source0->objectName())
	{
		_ui->general_doubleSpinBox_imgRate->setValue(0.0);
		_ui->source_mirroring->setChecked(false);
		_ui->lineEdit_calibrationFile->clear();
		_ui->comboBox_sourceType->setCurrentIndex(kSrcRGBD);
		_ui->lineEdit_sourceDevice->setText("");
		_ui->lineEdit_sourceLocalTransform->setText("0 0 1 -1 0 0 0 -1 0");

		_ui->source_comboBox_image_type->setCurrentIndex(kSrcUsbDevice-kSrcUsbDevice);
		_ui->source_images_spinBox_startPos->setValue(0);
		_ui->source_images_refreshDir->setChecked(false);
		_ui->checkBox_rgbImages_rectify->setChecked(false);
		_ui->comboBox_cameraImages_bayerMode->setCurrentIndex(0);
		_ui->checkBox_rgbVideo_rectify->setChecked(false);

		_ui->source_checkBox_ignoreOdometry->setChecked(false);
		_ui->source_checkBox_ignoreGoalDelay->setChecked(true);
		_ui->source_checkBox_ignoreGoals->setChecked(true);
		_ui->source_spinBox_databaseStartPos->setValue(0);
		_ui->source_spinBox_database_cameraIndex->setValue(-1);
		_ui->source_checkBox_useDbStamps->setChecked(true);

#ifdef _WIN32
		_ui->comboBox_cameraRGBD->setCurrentIndex(kSrcOpenNI2-kSrcRGBD); // openni2
#else
		if(CameraFreenect::available())
		{
			_ui->comboBox_cameraRGBD->setCurrentIndex(kSrcFreenect-kSrcRGBD); // freenect
		}
		else if(CameraOpenNI2::available())
		{
			_ui->comboBox_cameraRGBD->setCurrentIndex(kSrcOpenNI2-kSrcRGBD); // openni2
		}
		else
		{
			_ui->comboBox_cameraRGBD->setCurrentIndex(kSrcOpenNI_PCL-kSrcRGBD); // openni-pcl
		}
#endif
		if(CameraStereoDC1394::available())
		{
			_ui->comboBox_cameraStereo->setCurrentIndex(kSrcDC1394-kSrcStereo); // dc1394
		}
		else if(CameraStereoFlyCapture2::available())
		{
			_ui->comboBox_cameraStereo->setCurrentIndex(kSrcFlyCapture2-kSrcStereo); // flycapture
		}
		else
		{
			_ui->comboBox_cameraStereo->setCurrentIndex(kSrcStereoImages-kSrcStereo); // stereo images
		}

		_ui->checkbox_rgbd_colorOnly->setChecked(false);
		_ui->spinBox_source_imageDecimation->setValue(1);
		_ui->checkbox_stereo_depthGenerated->setChecked(false);
		_ui->openni2_autoWhiteBalance->setChecked(true);
		_ui->openni2_autoExposure->setChecked(true);
		_ui->openni2_exposure->setValue(0);
		_ui->openni2_gain->setValue(100);
		_ui->openni2_mirroring->setChecked(false);
		_ui->openni2_stampsIdsUsed->setChecked(false);
		_ui->comboBox_freenect2Format->setCurrentIndex(0);
		_ui->doubleSpinBox_freenect2MinDepth->setValue(0.3);
		_ui->doubleSpinBox_freenect2MaxDepth->setValue(12.0);
		_ui->checkBox_freenect2BilateralFiltering->setChecked(true);
		_ui->checkBox_freenect2EdgeAwareFiltering->setChecked(true);
		_ui->checkBox_freenect2NoiseFiltering->setChecked(true);
		_ui->comboBox_realsensePresetRGB->setCurrentIndex(0);
		_ui->comboBox_realsensePresetDepth->setCurrentIndex(2);
		_ui->lineEdit_openniOniPath->clear();
		_ui->lineEdit_openni2OniPath->clear();
		_ui->lineEdit_cameraRGBDImages_path_rgb->setText("");
		_ui->lineEdit_cameraRGBDImages_path_depth->setText("");
		_ui->doubleSpinBox_cameraRGBDImages_scale->setValue(1.0);
		_ui->lineEdit_source_distortionModel->setText("");
		_ui->groupBox_bilateral->setChecked(false);
		_ui->doubleSpinBox_bilateral_sigmaS->setValue(10.0);
		_ui->doubleSpinBox_bilateral_sigmaR->setValue(0.1);

		_ui->source_comboBox_image_type->setCurrentIndex(kSrcDC1394-kSrcDC1394);
		_ui->lineEdit_cameraStereoImages_path_left->setText("");
		_ui->lineEdit_cameraStereoImages_path_right->setText("");
		_ui->checkBox_stereoImages_rectify->setChecked(false);
		_ui->lineEdit_cameraStereoVideo_path->setText("");
		_ui->lineEdit_cameraStereoVideo_path_2->setText("");
		_ui->checkBox_stereoVideo_rectify->setChecked(false);
		_ui->comboBox_stereoZed_resolution->setCurrentIndex(2);
		_ui->comboBox_stereoZed_quality->setCurrentIndex(1);
		_ui->checkbox_stereoZed_selfCalibration->setChecked(false);
		_ui->comboBox_stereoZed_sensingMode->setCurrentIndex(1);
		_ui->spinBox_stereoZed_confidenceThr->setValue(100);
		_ui->checkbox_stereoZed_odom->setChecked(false);
		_ui->lineEdit_zedSvoPath->clear();

		_ui->checkBox_cameraImages_timestamps->setChecked(false);
		_ui->checkBox_cameraImages_syncTimeStamps->setChecked(true);
		_ui->lineEdit_cameraImages_timestamps->setText("");
		_ui->lineEdit_cameraImages_path_scans->setText("");
		_ui->lineEdit_cameraImages_laser_transform->setText("0 0 0 0 0 0");
		_ui->spinBox_cameraImages_max_scan_pts->setValue(0);
		_ui->spinBox_cameraImages_scanDownsampleStep->setValue(1);
		_ui->doubleSpinBox_cameraImages_scanVoxelSize->setValue(0.0f);
		_ui->lineEdit_cameraImages_odom->setText("");
		_ui->comboBox_cameraImages_odomFormat->setCurrentIndex(0);
		_ui->lineEdit_cameraImages_gt->setText("");
		_ui->comboBox_cameraImages_gtFormat->setCurrentIndex(0);

		_ui->groupBox_scanFromDepth->setChecked(false);
		_ui->spinBox_cameraScanFromDepth_decimation->setValue(8);
		_ui->doubleSpinBox_cameraSCanFromDepth_maxDepth->setValue(4.0);
		_ui->doubleSpinBox_cameraImages_scanVoxelSize->setValue(0.025f);
		_ui->spinBox_cameraImages_scanNormalsK->setValue(20);

		_ui->groupBox_depthFromScan->setChecked(false);
		_ui->groupBox_depthFromScan_fillHoles->setChecked(true);
		_ui->radioButton_depthFromScan_vertical->setChecked(true);
		_ui->radioButton_depthFromScan_horizontal->setChecked(false);
		_ui->checkBox_depthFromScan_fillBorders->setChecked(false);
	}
	else if(groupBox->objectName() == _ui->groupBox_rtabmap_basic0->objectName())
	{
		_ui->general_doubleSpinBox_timeThr_2->setValue(Parameters::defaultRtabmapTimeThr());
		_ui->general_doubleSpinBox_hardThr_2->setValue(Parameters::defaultRtabmapLoopThr());
		_ui->doubleSpinBox_similarityThreshold_2->setValue(Parameters::defaultMemRehearsalSimilarity());
		_ui->general_spinBox_imagesBufferSize_2->setValue(Parameters::defaultRtabmapImageBufferSize());
		_ui->general_spinBox_maxStMemSize_2->setValue(Parameters::defaultMemSTMSize());
		_ui->general_checkBox_publishStats_2->setChecked(Parameters::defaultRtabmapPublishStats());
		_ui->general_checkBox_activateRGBD_2->setChecked(Parameters::defaultRGBDEnabled());
		_ui->general_checkBox_SLAM_mode_2->setChecked(Parameters::defaultMemIncrementalMemory());
		_ui->general_doubleSpinBox_detectionRate_2->setValue(Parameters::defaultRtabmapDetectionRate());
		// match the advanced (spin and doubleSpin boxes)
		_ui->general_doubleSpinBox_timeThr->setValue(Parameters::defaultRtabmapTimeThr());
		_ui->general_doubleSpinBox_hardThr->setValue(Parameters::defaultRtabmapLoopThr());
		_ui->doubleSpinBox_similarityThreshold->setValue(Parameters::defaultMemRehearsalSimilarity());
		_ui->general_spinBox_imagesBufferSize->setValue(Parameters::defaultRtabmapImageBufferSize());
		_ui->general_spinBox_maxStMemSize->setValue(Parameters::defaultMemSTMSize());
		_ui->general_doubleSpinBox_detectionRate->setValue(Parameters::defaultRtabmapDetectionRate());
	}
	else
	{
		QObjectList children = groupBox->children();
		rtabmap::ParametersMap defaults = Parameters::getDefaultParameters();
		std::string key;
		for(int i=0; i<children.size(); ++i)
		{
			key = children.at(i)->objectName().toStdString();
			if(uContains(defaults, key))
			{
				if(key.compare(Parameters::kRtabmapWorkingDirectory().c_str()) == 0)
				{
					this->setParameter(key, Parameters::createDefaultWorkingDirectory());
				}
				else
				{
					this->setParameter(key, defaults.at(key));
				}
				if(qobject_cast<const QGroupBox*>(children.at(i)))
				{
					this->resetSettings((QGroupBox*)children.at(i));
				}
			}
			else if(qobject_cast<const QGroupBox*>(children.at(i)))
			{
				this->resetSettings((QGroupBox*)children.at(i));
			}
			else if(qobject_cast<const QStackedWidget*>(children.at(i)))
			{
				QStackedWidget * stackedWidget = (QStackedWidget*)children.at(i);
				for(int j=0; j<stackedWidget->count(); ++j)
				{
					const QObjectList & children2 = stackedWidget->widget(j)->children();
					for(int k=0; k<children2.size(); ++k)
					{
						if(qobject_cast<QGroupBox *>(children2.at(k)))
						{
							this->resetSettings((QGroupBox*)children2.at(k));
						}
					}
				}
			}
		}

		if(groupBox->findChild<QLineEdit*>(_ui->lineEdit_kp_roi->objectName()))
		{
			this->setupKpRoiPanel();
		}

		if(groupBox->objectName() == _ui->groupBox_odometry1->objectName())
		{
			_ui->odom_registration->setCurrentIndex(3);
		}
	}
}

void PreferencesDialog::resetSettings(int panelNumber)
{
	QList<QGroupBox*> boxes = this->getGroupBoxes();
	if(panelNumber >= 0 && panelNumber < boxes.size())
	{
		this->resetSettings(boxes.at(panelNumber));
	}
	else if(panelNumber == -1)
	{
		for(QList<QGroupBox*>::iterator iter = boxes.begin(); iter!=boxes.end(); ++iter)
		{
			this->resetSettings(*iter);
		}
	}
	else
	{
		ULOGGER_WARN("panel number and the number of stacked widget doesn't match");
	}

}

QString PreferencesDialog::getWorkingDirectory() const
{
	return _ui->lineEdit_workingDirectory->text();
}

QString PreferencesDialog::getIniFilePath() const
{
	QString privatePath = QDir::homePath() + "/.rtabmap";
	if(!QDir(privatePath).exists())
	{
		QDir::home().mkdir(".rtabmap");
	}
	return privatePath + "/rtabmap.ini";
}

QString PreferencesDialog::getTmpIniFilePath() const
{
	return getIniFilePath()+".tmp";
}

void PreferencesDialog::loadConfigFrom()
{
	QString path = QFileDialog::getOpenFileName(this, tr("Load settings..."), this->getWorkingDirectory(), "*.ini");
	if(!path.isEmpty())
	{
		this->readSettings(path);
	}
}

void PreferencesDialog::readSettings(const QString & filePath)
{
	ULOGGER_DEBUG("%s", filePath.toStdString().c_str());
	readGuiSettings(filePath);
	readCameraSettings(filePath);
	if(!readCoreSettings(filePath))
	{
		_modifiedParameters.clear();
		_obsoletePanels = kPanelDummy;

		// only keep GUI settings
		QStandardItem * parentItem = _indexModel->invisibleRootItem();
		if(parentItem)
		{
			for(int i=1; i<parentItem->rowCount(); ++i)
			{
				parentItem->child(i)->setEnabled(false);
			}
		}
		_ui->radioButton_basic->setEnabled(false);
		_ui->radioButton_advanced->setEnabled(false);
	}
	else
	{
		// enable settings
		QStandardItem * parentItem = _indexModel->invisibleRootItem();
		if(parentItem)
		{
			for(int i=0; i<parentItem->rowCount(); ++i)
			{
				parentItem->child(i)->setEnabled(true);
			}
		}
		_ui->radioButton_basic->setEnabled(true);
		_ui->radioButton_advanced->setEnabled(true);
	}
}

void PreferencesDialog::readGuiSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup("General");
	_ui->general_checkBox_imagesKept->setChecked(settings.value("imagesKept", _ui->general_checkBox_imagesKept->isChecked()).toBool());
	_ui->general_checkBox_cloudsKept->setChecked(settings.value("cloudsKept", _ui->general_checkBox_cloudsKept->isChecked()).toBool());
	_ui->comboBox_loggerLevel->setCurrentIndex(settings.value("loggerLevel", _ui->comboBox_loggerLevel->currentIndex()).toInt());
	_ui->comboBox_loggerEventLevel->setCurrentIndex(settings.value("loggerEventLevel", _ui->comboBox_loggerEventLevel->currentIndex()).toInt());
	_ui->comboBox_loggerPauseLevel->setCurrentIndex(settings.value("loggerPauseLevel", _ui->comboBox_loggerPauseLevel->currentIndex()).toInt());
	_ui->comboBox_loggerType->setCurrentIndex(settings.value("loggerType", _ui->comboBox_loggerType->currentIndex()).toInt());
	_ui->checkBox_logger_printTime->setChecked(settings.value("loggerPrintTime", _ui->checkBox_logger_printTime->isChecked()).toBool());
	_ui->checkBox_logger_printThreadId->setChecked(settings.value("loggerPrintThreadId", _ui->checkBox_logger_printThreadId->isChecked()).toBool());
	_ui->checkBox_verticalLayoutUsed->setChecked(settings.value("verticalLayoutUsed", _ui->checkBox_verticalLayoutUsed->isChecked()).toBool());
	_ui->checkBox_imageRejectedShown->setChecked(settings.value("imageRejectedShown", _ui->checkBox_imageRejectedShown->isChecked()).toBool());
	_ui->checkBox_imageHighestHypShown->setChecked(settings.value("imageHighestHypShown", _ui->checkBox_imageHighestHypShown->isChecked()).toBool());
	_ui->checkBox_beep->setChecked(settings.value("beep", _ui->checkBox_beep->isChecked()).toBool());
	_ui->checkBox_stamps->setChecked(settings.value("figure_time", _ui->checkBox_stamps->isChecked()).toBool());
	_ui->checkBox_cacheStatistics->setChecked(settings.value("figure_cache", _ui->checkBox_cacheStatistics->isChecked()).toBool());
	_ui->checkBox_notifyWhenNewGlobalPathIsReceived->setChecked(settings.value("notifyNewGlobalPath", _ui->checkBox_notifyWhenNewGlobalPathIsReceived->isChecked()).toBool());
	_ui->spinBox_odomQualityWarnThr->setValue(settings.value("odomQualityThr", _ui->spinBox_odomQualityWarnThr->value()).toInt());
	_ui->checkBox_posteriorGraphView->setChecked(settings.value("posteriorGraphView", _ui->checkBox_posteriorGraphView->isChecked()).toBool());
	_ui->checkbox_odomDisabled->setChecked(settings.value("odomDisabled", _ui->checkbox_odomDisabled->isChecked()).toBool());
	_ui->odom_registration->setCurrentIndex(settings.value("odomRegistration", _ui->odom_registration->currentIndex()).toInt());
	_ui->checkbox_groundTruthAlign->setChecked(settings.value("gtAlign", _ui->checkbox_groundTruthAlign->isChecked()).toBool());

	for(int i=0; i<2; ++i)
	{
		_3dRenderingShowClouds[i]->setChecked(settings.value(QString("showClouds%1").arg(i), _3dRenderingShowClouds[i]->isChecked()).toBool());
		_3dRenderingDecimation[i]->setValue(settings.value(QString("decimation%1").arg(i), _3dRenderingDecimation[i]->value()).toInt());
		_3dRenderingMaxDepth[i]->setValue(settings.value(QString("maxDepth%1").arg(i), _3dRenderingMaxDepth[i]->value()).toDouble());
		_3dRenderingMinDepth[i]->setValue(settings.value(QString("minDepth%1").arg(i), _3dRenderingMinDepth[i]->value()).toDouble());
		_3dRenderingRoiRatios[i]->setText(settings.value(QString("roiRatios%1").arg(i), _3dRenderingRoiRatios[i]->text()).toString());
		_3dRenderingShowScans[i]->setChecked(settings.value(QString("showScans%1").arg(i), _3dRenderingShowScans[i]->isChecked()).toBool());
		_3dRenderingShowFeatures[i]->setChecked(settings.value(QString("showFeatures%1").arg(i), _3dRenderingShowFeatures[i]->isChecked()).toBool());

		_3dRenderingDownsamplingScan[i]->setValue(settings.value(QString("downsamplingScan%1").arg(i), _3dRenderingDownsamplingScan[i]->value()).toInt());
		_3dRenderingVoxelSizeScan[i]->setValue(settings.value(QString("voxelSizeScan%1").arg(i), _3dRenderingVoxelSizeScan[i]->value()).toDouble());
		_3dRenderingOpacity[i]->setValue(settings.value(QString("opacity%1").arg(i), _3dRenderingOpacity[i]->value()).toDouble());
		_3dRenderingPtSize[i]->setValue(settings.value(QString("ptSize%1").arg(i), _3dRenderingPtSize[i]->value()).toInt());
		_3dRenderingOpacityScan[i]->setValue(settings.value(QString("opacityScan%1").arg(i), _3dRenderingOpacityScan[i]->value()).toDouble());
		_3dRenderingPtSizeScan[i]->setValue(settings.value(QString("ptSizeScan%1").arg(i), _3dRenderingPtSizeScan[i]->value()).toInt());
		_3dRenderingPtSizeFeatures[i]->setValue(settings.value(QString("ptSizeFeatures%1").arg(i), _3dRenderingPtSizeFeatures[i]->value()).toInt());
	}
	_ui->doubleSpinBox_voxel->setValue(settings.value("cloudVoxel", _ui->doubleSpinBox_voxel->value()).toDouble());
	_ui->doubleSpinBox_noiseRadius->setValue(settings.value("cloudNoiseRadius", _ui->doubleSpinBox_noiseRadius->value()).toDouble());
	_ui->spinBox_noiseMinNeighbors->setValue(settings.value("cloudNoiseMinNeighbors", _ui->spinBox_noiseMinNeighbors->value()).toInt());
	_ui->doubleSpinBox_ceilingFilterHeight->setValue(settings.value("cloudCeilingHeight", _ui->doubleSpinBox_ceilingFilterHeight->value()).toDouble());
	_ui->doubleSpinBox_floorFilterHeight->setValue(settings.value("cloudFloorHeight", _ui->doubleSpinBox_floorFilterHeight->value()).toDouble());
	_ui->spinBox_normalKSearch->setValue(settings.value("normalKSearch", _ui->spinBox_normalKSearch->value()).toInt());

	_ui->checkBox_showGraphs->setChecked(settings.value("showGraphs", _ui->checkBox_showGraphs->isChecked()).toBool());
	_ui->checkBox_showFrustums->setChecked(settings.value("showFrustums", _ui->checkBox_showFrustums->isChecked()).toBool());
	_ui->checkBox_showLabels->setChecked(settings.value("showLabels", _ui->checkBox_showLabels->isChecked()).toBool());

	_ui->radioButton_noFiltering->setChecked(settings.value("noFiltering", _ui->radioButton_noFiltering->isChecked()).toBool());
	_ui->radioButton_nodeFiltering->setChecked(settings.value("cloudFiltering", _ui->radioButton_nodeFiltering->isChecked()).toBool());
	_ui->doubleSpinBox_cloudFilterRadius->setValue(settings.value("cloudFilteringRadius", _ui->doubleSpinBox_cloudFilterRadius->value()).toDouble());
	_ui->doubleSpinBox_cloudFilterAngle->setValue(settings.value("cloudFilteringAngle", _ui->doubleSpinBox_cloudFilterAngle->value()).toDouble());
	_ui->radioButton_subtractFiltering->setChecked(settings.value("subtractFiltering", _ui->radioButton_subtractFiltering->isChecked()).toBool());
	_ui->spinBox_subtractFilteringMinPts->setValue(settings.value("subtractFilteringMinPts", _ui->spinBox_subtractFilteringMinPts->value()).toInt());
	_ui->doubleSpinBox_subtractFilteringRadius->setValue(settings.value("subtractFilteringRadius", _ui->doubleSpinBox_subtractFilteringRadius->value()).toDouble());
	_ui->doubleSpinBox_subtractFilteringAngle->setValue(settings.value("subtractFilteringAngle", _ui->doubleSpinBox_subtractFilteringAngle->value()).toDouble());

	_ui->checkBox_map_shown->setChecked(settings.value("gridMapShown", _ui->checkBox_map_shown->isChecked()).toBool());
	_ui->doubleSpinBox_map_resolution->setValue(settings.value("gridMapResolution", _ui->doubleSpinBox_map_resolution->value()).toDouble());
	_ui->checkBox_map_erode->setChecked(settings.value("gridMapEroded", _ui->checkBox_map_erode->isChecked()).toBool());
	_ui->checkBox_map_incremental->setChecked(settings.value("gridMapIncremental", _ui->checkBox_map_incremental->isChecked()).toBool());
	_ui->doubleSpinBox_map_footprintRadius->setValue(settings.value("gridMapFootprintRadius", _ui->doubleSpinBox_map_footprintRadius->value()).toDouble());
	_ui->doubleSpinBox_map_opacity->setValue(settings.value("gridMapOpacity", _ui->doubleSpinBox_map_opacity->value()).toDouble());

	_ui->groupBox_octomap->setChecked(settings.value("octomap", _ui->groupBox_octomap->isChecked()).toBool());
	_ui->spinBox_octomap_treeDepth->setValue(settings.value("octomap_depth", _ui->spinBox_octomap_treeDepth->value()).toInt());
	_ui->checkBox_octomap_2dgrid->setChecked(settings.value("octomap_2dgrid", _ui->checkBox_octomap_2dgrid->isChecked()).toBool());
	_ui->checkBox_octomap_show3dMap->setChecked(settings.value("octomap_3dmap", _ui->checkBox_octomap_show3dMap->isChecked()).toBool());
	_ui->checkBox_octomap_cubeRendering->setChecked(settings.value("octomap_cube", _ui->checkBox_octomap_cubeRendering->isChecked()).toBool());
	_ui->doubleSpinBox_octomap_occupancyThr->setValue(settings.value("octomap_occupancy_thr", _ui->doubleSpinBox_octomap_occupancyThr->value()).toDouble());

	_ui->groupBox_organized->setChecked(settings.value("meshing", _ui->groupBox_organized->isChecked()).toBool());
	_ui->doubleSpinBox_mesh_angleTolerance->setValue(settings.value("meshing_angle", _ui->doubleSpinBox_mesh_angleTolerance->value()).toDouble());
	_ui->checkBox_mesh_quad->setChecked(settings.value("meshing_quad", _ui->checkBox_mesh_quad->isChecked()).toBool());
	_ui->checkBox_mesh_texture->setChecked(settings.value("meshing_texture", _ui->checkBox_mesh_texture->isChecked()).toBool());
	_ui->spinBox_mesh_triangleSize->setValue(settings.value("meshing_triangle_size", _ui->spinBox_mesh_triangleSize->value()).toInt());

	settings.endGroup(); // General

	settings.endGroup(); // rtabmap
}

void PreferencesDialog::readCameraSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);

	settings.beginGroup("Camera");
	_ui->general_doubleSpinBox_imgRate->setValue(settings.value("imgRate", _ui->general_doubleSpinBox_imgRate->value()).toDouble());
	_ui->source_mirroring->setChecked(settings.value("mirroring", _ui->source_mirroring->isChecked()).toBool());
	_ui->lineEdit_calibrationFile->setText(settings.value("calibrationName", _ui->lineEdit_calibrationFile->text()).toString());
	_ui->comboBox_sourceType->setCurrentIndex(settings.value("type", _ui->comboBox_sourceType->currentIndex()).toInt());
	_ui->lineEdit_sourceDevice->setText(settings.value("device",_ui->lineEdit_sourceDevice->text()).toString());
	_ui->lineEdit_sourceLocalTransform->setText(settings.value("localTransform",_ui->lineEdit_sourceLocalTransform->text()).toString());
	_ui->spinBox_source_imageDecimation->setValue(settings.value("imageDecimation",_ui->spinBox_source_imageDecimation->value()).toInt());

	settings.beginGroup("rgbd");
	_ui->comboBox_cameraRGBD->setCurrentIndex(settings.value("driver", _ui->comboBox_cameraRGBD->currentIndex()).toInt());
	_ui->checkbox_rgbd_colorOnly->setChecked(settings.value("rgbdColorOnly", _ui->checkbox_rgbd_colorOnly->isChecked()).toBool());
	_ui->lineEdit_source_distortionModel->setText(settings.value("distortion_model", _ui->lineEdit_source_distortionModel->text()).toString());
	_ui->groupBox_bilateral->setChecked(settings.value("bilateral", _ui->groupBox_bilateral->isChecked()).toBool());
	_ui->doubleSpinBox_bilateral_sigmaS->setValue(settings.value("bilateral_sigma_s", _ui->doubleSpinBox_bilateral_sigmaS->value()).toDouble());
	_ui->doubleSpinBox_bilateral_sigmaR->setValue(settings.value("bilateral_sigma_r", _ui->doubleSpinBox_bilateral_sigmaR->value()).toDouble());
	settings.endGroup(); // rgbd

	settings.beginGroup("stereo");
	_ui->comboBox_cameraStereo->setCurrentIndex(settings.value("driver", _ui->comboBox_cameraStereo->currentIndex()).toInt());
	_ui->checkbox_stereo_depthGenerated->setChecked(settings.value("depthGenerated", _ui->checkbox_stereo_depthGenerated->isChecked()).toBool());
	settings.endGroup(); // stereo

	settings.beginGroup("rgb");
	_ui->source_comboBox_image_type->setCurrentIndex(settings.value("driver", _ui->source_comboBox_image_type->currentIndex()).toInt());
	settings.endGroup(); // rgb

	settings.beginGroup("Openni");
	_ui->lineEdit_openniOniPath->setText(settings.value("oniPath", _ui->lineEdit_openniOniPath->text()).toString());
	settings.endGroup(); // Openni

	settings.beginGroup("Openni2");
	_ui->openni2_autoWhiteBalance->setChecked(settings.value("autoWhiteBalance", _ui->openni2_autoWhiteBalance->isChecked()).toBool());
	_ui->openni2_autoExposure->setChecked(settings.value("autoExposure", _ui->openni2_autoExposure->isChecked()).toBool());
	_ui->openni2_exposure->setValue(settings.value("exposure", _ui->openni2_exposure->value()).toInt());
	_ui->openni2_gain->setValue(settings.value("gain", _ui->openni2_gain->value()).toInt());
	_ui->openni2_mirroring->setChecked(settings.value("mirroring", _ui->openni2_mirroring->isChecked()).toBool());
	_ui->openni2_stampsIdsUsed->setChecked(settings.value("stampsIdsUsed", _ui->openni2_stampsIdsUsed->isChecked()).toBool());
	_ui->lineEdit_openni2OniPath->setText(settings.value("oniPath", _ui->lineEdit_openni2OniPath->text()).toString());
	settings.endGroup(); // Openni2

	settings.beginGroup("Freenect2");
	_ui->comboBox_freenect2Format->setCurrentIndex(settings.value("format", _ui->comboBox_freenect2Format->currentIndex()).toInt());
	_ui->doubleSpinBox_freenect2MinDepth->setValue(settings.value("minDepth", _ui->doubleSpinBox_freenect2MinDepth->value()).toDouble());
	_ui->doubleSpinBox_freenect2MaxDepth->setValue(settings.value("maxDepth", _ui->doubleSpinBox_freenect2MaxDepth->value()).toDouble());
	_ui->checkBox_freenect2BilateralFiltering->setChecked(settings.value("bilateralFiltering", _ui->checkBox_freenect2BilateralFiltering->isChecked()).toBool());
	_ui->checkBox_freenect2EdgeAwareFiltering->setChecked(settings.value("edgeAwareFiltering", _ui->checkBox_freenect2EdgeAwareFiltering->isChecked()).toBool());
	_ui->checkBox_freenect2NoiseFiltering->setChecked(settings.value("noiseFiltering", _ui->checkBox_freenect2NoiseFiltering->isChecked()).toBool());
	settings.endGroup(); // Freenect2

	settings.beginGroup("RealSense");
	_ui->comboBox_realsensePresetRGB->setCurrentIndex(settings.value("presetRGB", _ui->comboBox_realsensePresetRGB->currentIndex()).toInt());
	_ui->comboBox_realsensePresetDepth->setCurrentIndex(settings.value("presetDepth", _ui->comboBox_realsensePresetDepth->currentIndex()).toInt());
	settings.endGroup(); // RealSense

	settings.beginGroup("RGBDImages");
	_ui->lineEdit_cameraRGBDImages_path_rgb->setText(settings.value("path_rgb", _ui->lineEdit_cameraRGBDImages_path_rgb->text()).toString());
	_ui->lineEdit_cameraRGBDImages_path_depth->setText(settings.value("path_depth", _ui->lineEdit_cameraRGBDImages_path_depth->text()).toString());
	_ui->doubleSpinBox_cameraRGBDImages_scale->setValue(settings.value("scale", _ui->doubleSpinBox_cameraRGBDImages_scale->value()).toDouble());
	settings.endGroup(); // RGBDImages

	settings.beginGroup("StereoImages");
	_ui->lineEdit_cameraStereoImages_path_left->setText(settings.value("path_left", _ui->lineEdit_cameraStereoImages_path_left->text()).toString());
	_ui->lineEdit_cameraStereoImages_path_right->setText(settings.value("path_right", _ui->lineEdit_cameraStereoImages_path_right->text()).toString());
	_ui->checkBox_stereoImages_rectify->setChecked(settings.value("rectify",_ui->checkBox_stereoImages_rectify->isChecked()).toBool());
	settings.endGroup(); // StereoImages

	settings.beginGroup("StereoVideo");
	_ui->lineEdit_cameraStereoVideo_path->setText(settings.value("path", _ui->lineEdit_cameraStereoVideo_path->text()).toString());
	_ui->lineEdit_cameraStereoVideo_path_2->setText(settings.value("path2", _ui->lineEdit_cameraStereoVideo_path_2->text()).toString());
	_ui->checkBox_stereoVideo_rectify->setChecked(settings.value("rectify",_ui->checkBox_stereoVideo_rectify->isChecked()).toBool());
	settings.endGroup(); // StereoVideo

	settings.beginGroup("StereoZed");
	_ui->comboBox_stereoZed_resolution->setCurrentIndex(settings.value("resolution", _ui->comboBox_stereoZed_resolution->currentIndex()).toInt());
	_ui->comboBox_stereoZed_quality->setCurrentIndex(settings.value("quality", _ui->comboBox_stereoZed_quality->currentIndex()).toInt());
	_ui->checkbox_stereoZed_selfCalibration->setChecked(settings.value("self_calibration", _ui->checkbox_stereoZed_selfCalibration->isChecked()).toBool());
	_ui->comboBox_stereoZed_sensingMode->setCurrentIndex(settings.value("sensing_mode", _ui->comboBox_stereoZed_sensingMode->currentIndex()).toInt());
	_ui->spinBox_stereoZed_confidenceThr->setValue(settings.value("confidence_thr", _ui->spinBox_stereoZed_confidenceThr->value()).toInt());
	_ui->checkbox_stereoZed_odom->setChecked(settings.value("odom", _ui->checkbox_stereoZed_odom->isChecked()).toBool());
	_ui->lineEdit_zedSvoPath->setText(settings.value("svo_path", _ui->lineEdit_zedSvoPath->text()).toString());
	settings.endGroup(); // StereoZed
	

	settings.beginGroup("Images");
	_ui->source_images_lineEdit_path->setText(settings.value("path", _ui->source_images_lineEdit_path->text()).toString());
	_ui->source_images_spinBox_startPos->setValue(settings.value("startPos",_ui->source_images_spinBox_startPos->value()).toInt());
	_ui->source_images_refreshDir->setChecked(settings.value("refreshDir",_ui->source_images_refreshDir->isChecked()).toBool());
	_ui->checkBox_rgbImages_rectify->setChecked(settings.value("rectify",_ui->checkBox_rgbImages_rectify->isChecked()).toBool());
	_ui->comboBox_cameraImages_bayerMode->setCurrentIndex(settings.value("bayerMode",_ui->comboBox_cameraImages_bayerMode->currentIndex()).toInt());

	_ui->checkBox_cameraImages_timestamps->setChecked(settings.value("filenames_as_stamps",_ui->checkBox_cameraImages_timestamps->isChecked()).toBool());
	_ui->checkBox_cameraImages_syncTimeStamps->setChecked(settings.value("sync_stamps",_ui->checkBox_cameraImages_syncTimeStamps->isChecked()).toBool());
	_ui->lineEdit_cameraImages_timestamps->setText(settings.value("stamps", _ui->lineEdit_cameraImages_timestamps->text()).toString());
	_ui->lineEdit_cameraImages_path_scans->setText(settings.value("path_scans", _ui->lineEdit_cameraImages_path_scans->text()).toString());
	_ui->lineEdit_cameraImages_laser_transform->setText(settings.value("scan_transform", _ui->lineEdit_cameraImages_laser_transform->text()).toString());
	_ui->spinBox_cameraImages_max_scan_pts->setValue(settings.value("scan_max_pts", _ui->spinBox_cameraImages_max_scan_pts->value()).toInt());
	_ui->spinBox_cameraImages_scanDownsampleStep->setValue(settings.value("scan_downsample_step", _ui->spinBox_cameraImages_scanDownsampleStep->value()).toInt());
	_ui->doubleSpinBox_cameraImages_scanVoxelSize->setValue(settings.value("scan_voxel_size", _ui->doubleSpinBox_cameraImages_scanVoxelSize->value()).toDouble());
	_ui->lineEdit_cameraImages_odom->setText(settings.value("odom_path", _ui->lineEdit_cameraImages_odom->text()).toString());
	_ui->comboBox_cameraImages_odomFormat->setCurrentIndex(settings.value("odom_format", _ui->comboBox_cameraImages_odomFormat->currentIndex()).toInt());
	_ui->lineEdit_cameraImages_gt->setText(settings.value("gt_path", _ui->lineEdit_cameraImages_gt->text()).toString());
	_ui->comboBox_cameraImages_gtFormat->setCurrentIndex(settings.value("gt_format", _ui->comboBox_cameraImages_gtFormat->currentIndex()).toInt());
	settings.endGroup(); // images

	settings.beginGroup("Video");
	_ui->source_video_lineEdit_path->setText(settings.value("path", _ui->source_video_lineEdit_path->text()).toString());
	_ui->checkBox_rgbVideo_rectify->setChecked(settings.value("rectify",_ui->checkBox_rgbVideo_rectify->isChecked()).toBool());
	settings.endGroup(); // video

	settings.beginGroup("ScanFromDepth");
	_ui->groupBox_scanFromDepth->setChecked(settings.value("enabled", _ui->groupBox_scanFromDepth->isChecked()).toBool());
	_ui->spinBox_cameraScanFromDepth_decimation->setValue(settings.value("decimation", _ui->spinBox_cameraScanFromDepth_decimation->value()).toInt());
	_ui->doubleSpinBox_cameraSCanFromDepth_maxDepth->setValue(settings.value("maxDepth", _ui->doubleSpinBox_cameraSCanFromDepth_maxDepth->value()).toDouble());
	_ui->doubleSpinBox_cameraImages_scanVoxelSize->setValue(settings.value("voxelSize", _ui->doubleSpinBox_cameraImages_scanVoxelSize->value()).toDouble());
	_ui->spinBox_cameraImages_scanNormalsK->setValue(settings.value("normalsK", _ui->spinBox_cameraImages_scanNormalsK->value()).toInt());
	settings.endGroup();//ScanFromDepth

	settings.beginGroup("DepthFromScan");
	_ui->groupBox_depthFromScan->setChecked(settings.value("depthFromScan", _ui->groupBox_depthFromScan->isChecked()).toBool());
	_ui->groupBox_depthFromScan_fillHoles->setChecked(settings.value("depthFromScanFillHoles", _ui->groupBox_depthFromScan_fillHoles->isChecked()).toBool());
	_ui->radioButton_depthFromScan_vertical->setChecked(settings.value("depthFromScanVertical", _ui->radioButton_depthFromScan_vertical->isChecked()).toBool());
	_ui->radioButton_depthFromScan_horizontal->setChecked(settings.value("depthFromScanHorizontal", _ui->radioButton_depthFromScan_horizontal->isChecked()).toBool());
	_ui->checkBox_depthFromScan_fillBorders->setChecked(settings.value("depthFromScanFillBorders", _ui->checkBox_depthFromScan_fillBorders->isChecked()).toBool());
	settings.endGroup();

	settings.beginGroup("Database");
	_ui->source_database_lineEdit_path->setText(settings.value("path",_ui->source_database_lineEdit_path->text()).toString());
	_ui->source_checkBox_ignoreOdometry->setChecked(settings.value("ignoreOdometry", _ui->source_checkBox_ignoreOdometry->isChecked()).toBool());
	_ui->source_checkBox_ignoreGoalDelay->setChecked(settings.value("ignoreGoalDelay", _ui->source_checkBox_ignoreGoalDelay->isChecked()).toBool());
	_ui->source_checkBox_ignoreGoals->setChecked(settings.value("ignoreGoals", _ui->source_checkBox_ignoreGoals->isChecked()).toBool());
	_ui->source_spinBox_databaseStartPos->setValue(settings.value("startPos", _ui->source_spinBox_databaseStartPos->value()).toInt());
	_ui->source_spinBox_database_cameraIndex->setValue(settings.value("cameraIndex", _ui->source_spinBox_database_cameraIndex->value()).toInt());
	_ui->source_checkBox_useDbStamps->setChecked(settings.value("useDatabaseStamps", _ui->source_checkBox_useDbStamps->isChecked()).toBool());
	settings.endGroup(); // Database

	settings.endGroup(); // Camera

	_calibrationDialog->loadSettings(settings, "CalibrationDialog");

}

bool PreferencesDialog::readCoreSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}

	UDEBUG("%s", path.toStdString().c_str());

	if(!QFile::exists(path))
	{
		QMessageBox::information(this, tr("INI file doesn't exist..."), tr("The configuration file \"%1\" does not exist, it will be created with default parameters.").arg(path));
	}

	ParametersMap parameters;
	Parameters::readINI(path.toStdString(), parameters);

	for(rtabmap::ParametersMap::const_iterator iter = parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string value = iter->second;
		if(iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
		{
			// The directory should exist if not the default one
			if(!QDir(value.c_str()).exists() && value.compare(Parameters::createDefaultWorkingDirectory().c_str()) != 0)
			{
				if(QDir(this->getWorkingDirectory().toStdString().c_str()).exists())
				{
					UWARN("Reading config: Not existing working directory \"%s\". Keeping old one (\"%s\").",
						value.c_str(),
						this->getWorkingDirectory().toStdString().c_str());
					value = this->getWorkingDirectory().toStdString();
				}
				else
				{
					std::string defaultWorkingDir = Parameters::createDefaultWorkingDirectory();
					UWARN("Reading config: Not existing working directory \"%s\". Using default one (\"%s\").",
						value.c_str(),
						defaultWorkingDir.c_str());
					value = defaultWorkingDir;
				}
			}
		}
		this->setParameter(iter->first, value);
	}

	// Add information about the working directory if not in the config file
	if(parameters.find(Parameters::kRtabmapWorkingDirectory()) == parameters.end())
	{
		if(!_initialized)
		{
			QMessageBox::information(this,
					tr("Working directory"),
					tr("RTAB-Map needs a working directory to put the database.\n\n"
					   "By default, the directory \"%1\" is used.\n\n"
					   "The working directory can be changed any time in the "
					   "preferences menu.").arg(
							   Parameters::createDefaultWorkingDirectory().c_str()));
		}
		this->setParameter(Parameters::kRtabmapWorkingDirectory(), Parameters::createDefaultWorkingDirectory());
		UDEBUG("key.toStdString()=%s", Parameters::createDefaultWorkingDirectory().c_str());
	}

	return true;
}

bool PreferencesDialog::saveConfigTo()
{
	QString path = QFileDialog::getSaveFileName(this, tr("Save settings..."), this->getWorkingDirectory()+QDir::separator()+"config.ini", "*.ini");
	if(!path.isEmpty())
	{
		writeGuiSettings(path);
		writeCameraSettings(path);
		writeCoreSettings(path);
		return true;
	}
	return false;
}

void PreferencesDialog::resetConfig()
{
	int button = QMessageBox::warning(this,
			tr("Reset settings..."),
			tr("This will reset all settings. Restore all settings to default without saving them first?"),
			   QMessageBox::Cancel | QMessageBox::Yes | QMessageBox::Save,
			   QMessageBox::Cancel);
	if(button == QMessageBox::Yes ||
	   (button == QMessageBox::Save && saveConfigTo()))
	{
		this->resetSettings(-1);

		_calibrationDialog->resetSettings();
	}
}

void PreferencesDialog::writeSettings(const QString & filePath)
{
	writeGuiSettings(filePath);
	writeCameraSettings(filePath);
	writeCoreSettings(filePath);

	UDEBUG("_obsoletePanels=%d modified parameters=%d", (int)_obsoletePanels, (int)_modifiedParameters.size());

	if(_modifiedParameters.size())
	{
		emit settingsChanged(_modifiedParameters);
	}

	if(_obsoletePanels)
	{
		emit settingsChanged(_obsoletePanels);
	}

	for(ParametersMap::iterator iter = _modifiedParameters.begin(); iter!=_modifiedParameters.end(); ++iter)
	{
		if (_parameters.at(iter->first).compare(iter->second) != 0)
		{
			bool different = true;
			if (Parameters::getType(iter->first).compare("double") == 0 ||
				Parameters::getType(iter->first).compare("float") == 0)
			{
				if (uStr2Double(_parameters.at(iter->first)) == uStr2Double(iter->second))
				{
					different = false;
				}
			}
			if (different)
			{
				UINFO("modified %s = %s->%s", iter->first.c_str(), _parameters.at(iter->first).c_str(), iter->second.c_str());
			}
		}
	}

	uInsert(_parameters, _modifiedParameters); // update cached parameters
	_modifiedParameters.clear();
	_obsoletePanels = kPanelDummy;
}

void PreferencesDialog::writeGuiSettings(const QString & filePath) const
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);
	settings.beginGroup("Gui");

	settings.beginGroup("General");
	settings.remove("");
	settings.setValue("imagesKept",           _ui->general_checkBox_imagesKept->isChecked());
	settings.setValue("cloudsKept",           _ui->general_checkBox_cloudsKept->isChecked());
	settings.setValue("loggerLevel",          _ui->comboBox_loggerLevel->currentIndex());
	settings.setValue("loggerEventLevel",     _ui->comboBox_loggerEventLevel->currentIndex());
	settings.setValue("loggerPauseLevel",     _ui->comboBox_loggerPauseLevel->currentIndex());
	settings.setValue("loggerType",           _ui->comboBox_loggerType->currentIndex());
	settings.setValue("loggerPrintTime",      _ui->checkBox_logger_printTime->isChecked());
	settings.setValue("loggerPrintThreadId",  _ui->checkBox_logger_printThreadId->isChecked());
	settings.setValue("verticalLayoutUsed",   _ui->checkBox_verticalLayoutUsed->isChecked());
	settings.setValue("imageRejectedShown",   _ui->checkBox_imageRejectedShown->isChecked());
	settings.setValue("imageHighestHypShown", _ui->checkBox_imageHighestHypShown->isChecked());
	settings.setValue("beep",                 _ui->checkBox_beep->isChecked());
	settings.setValue("figure_time",          _ui->checkBox_stamps->isChecked());
	settings.setValue("figure_cache",         _ui->checkBox_cacheStatistics->isChecked());
	settings.setValue("notifyNewGlobalPath",  _ui->checkBox_notifyWhenNewGlobalPathIsReceived->isChecked());
	settings.setValue("odomQualityThr",       _ui->spinBox_odomQualityWarnThr->value());
	settings.setValue("posteriorGraphView",   _ui->checkBox_posteriorGraphView->isChecked());
	settings.setValue("odomDisabled",         _ui->checkbox_odomDisabled->isChecked());
	settings.setValue("odomRegistration",     _ui->odom_registration->currentIndex());
	settings.setValue("gtAlign",              _ui->checkbox_groundTruthAlign->isChecked());

	for(int i=0; i<2; ++i)
	{
		settings.setValue(QString("showClouds%1").arg(i), _3dRenderingShowClouds[i]->isChecked());
		settings.setValue(QString("decimation%1").arg(i), _3dRenderingDecimation[i]->value());
		settings.setValue(QString("maxDepth%1").arg(i), _3dRenderingMaxDepth[i]->value());
		settings.setValue(QString("minDepth%1").arg(i), _3dRenderingMinDepth[i]->value());
		settings.setValue(QString("roiRatios%1").arg(i), _3dRenderingRoiRatios[i]->text());
		settings.setValue(QString("showScans%1").arg(i), _3dRenderingShowScans[i]->isChecked());
		settings.setValue(QString("showFeatures%1").arg(i), _3dRenderingShowFeatures[i]->isChecked());

		settings.setValue(QString("downsamplingScan%1").arg(i), _3dRenderingDownsamplingScan[i]->value());
		settings.setValue(QString("voxelSizeScan%1").arg(i), _3dRenderingVoxelSizeScan[i]->value());
		settings.setValue(QString("opacity%1").arg(i), _3dRenderingOpacity[i]->value());
		settings.setValue(QString("ptSize%1").arg(i), _3dRenderingPtSize[i]->value());
		settings.setValue(QString("opacityScan%1").arg(i), _3dRenderingOpacityScan[i]->value());
		settings.setValue(QString("ptSizeScan%1").arg(i), _3dRenderingPtSizeScan[i]->value());
		settings.setValue(QString("ptSizeFeatures%1").arg(i), _3dRenderingPtSizeFeatures[i]->value());
	}
	settings.setValue("cloudVoxel",             _ui->doubleSpinBox_voxel->value());
	settings.setValue("cloudNoiseRadius",       _ui->doubleSpinBox_noiseRadius->value());
	settings.setValue("cloudNoiseMinNeighbors", _ui->spinBox_noiseMinNeighbors->value());
	settings.setValue("cloudCeilingHeight",     _ui->doubleSpinBox_ceilingFilterHeight->value());
	settings.setValue("cloudFloorHeight",       _ui->doubleSpinBox_floorFilterHeight->value());
	settings.setValue("normalKSearch",           _ui->spinBox_normalKSearch->value());

	settings.setValue("showGraphs", _ui->checkBox_showGraphs->isChecked());
	settings.setValue("showFrustums", _ui->checkBox_showFrustums->isChecked());
	settings.setValue("showLabels", _ui->checkBox_showLabels->isChecked());

	settings.setValue("noFiltering",             _ui->radioButton_noFiltering->isChecked());
	settings.setValue("cloudFiltering",          _ui->radioButton_nodeFiltering->isChecked());
	settings.setValue("cloudFilteringRadius",    _ui->doubleSpinBox_cloudFilterRadius->value());
	settings.setValue("cloudFilteringAngle",     _ui->doubleSpinBox_cloudFilterAngle->value());
	settings.setValue("subtractFiltering",       _ui->radioButton_subtractFiltering->isChecked());
	settings.setValue("subtractFilteringMinPts", _ui->spinBox_subtractFilteringMinPts->value());
	settings.setValue("subtractFilteringRadius", _ui->doubleSpinBox_subtractFilteringRadius->value());
	settings.setValue("subtractFilteringAngle",  _ui->doubleSpinBox_subtractFilteringAngle->value());

	settings.setValue("gridMapShown",                _ui->checkBox_map_shown->isChecked());
	settings.setValue("gridMapResolution",           _ui->doubleSpinBox_map_resolution->value());
	settings.setValue("gridMapEroded",               _ui->checkBox_map_erode->isChecked());
	settings.setValue("gridMapIncremental",          _ui->checkBox_map_incremental->isChecked());
	settings.setValue("gridMapFootprintRadius",      _ui->doubleSpinBox_map_footprintRadius->value());
	settings.setValue("gridMapOpacity",              _ui->doubleSpinBox_map_opacity->value());

	settings.setValue("octomap",                     _ui->groupBox_octomap->isChecked());
	settings.setValue("octomap_depth",               _ui->spinBox_octomap_treeDepth->value());
	settings.setValue("octomap_2dgrid",              _ui->checkBox_octomap_2dgrid->isChecked());
	settings.setValue("octomap_3dmap",               _ui->checkBox_octomap_show3dMap->isChecked());
	settings.setValue("octomap_cube",                _ui->checkBox_octomap_cubeRendering->isChecked());
	settings.setValue("octomap_occupancy_thr",       _ui->doubleSpinBox_octomap_occupancyThr->value());


	settings.setValue("meshing",               _ui->groupBox_organized->isChecked());
	settings.setValue("meshing_angle",         _ui->doubleSpinBox_mesh_angleTolerance->value());
	settings.setValue("meshing_quad",          _ui->checkBox_mesh_quad->isChecked());
	settings.setValue("meshing_texture",       _ui->checkBox_mesh_texture->isChecked());
	settings.setValue("meshing_triangle_size", _ui->spinBox_mesh_triangleSize->value());

	settings.endGroup(); // General

	settings.endGroup(); // rtabmap
}

void PreferencesDialog::writeCameraSettings(const QString & filePath) const
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);

	settings.beginGroup("Camera");
	settings.remove("");
	settings.setValue("imgRate", 		 _ui->general_doubleSpinBox_imgRate->value());
	settings.setValue("mirroring",       _ui->source_mirroring->isChecked());
	settings.setValue("calibrationName", _ui->lineEdit_calibrationFile->text());
	settings.setValue("type",            _ui->comboBox_sourceType->currentIndex());
	settings.setValue("device", 		 _ui->lineEdit_sourceDevice->text());
	settings.setValue("localTransform",  _ui->lineEdit_sourceLocalTransform->text());
	settings.setValue("imageDecimation",  _ui->spinBox_source_imageDecimation->value());

	settings.beginGroup("rgbd");
	settings.setValue("driver", 	       _ui->comboBox_cameraRGBD->currentIndex());
	settings.setValue("rgbdColorOnly",     _ui->checkbox_rgbd_colorOnly->isChecked());
	settings.setValue("distortion_model",  _ui->lineEdit_source_distortionModel->text());
	settings.setValue("bilateral",	       _ui->groupBox_bilateral->isChecked());
	settings.setValue("bilateral_sigma_s", _ui->doubleSpinBox_bilateral_sigmaS->value());
	settings.setValue("bilateral_sigma_r", _ui->doubleSpinBox_bilateral_sigmaR->value());
	settings.endGroup(); // rgbd

	settings.beginGroup("stereo");
	settings.setValue("driver", 	_ui->comboBox_cameraStereo->currentIndex());
	settings.setValue("depthGenerated", _ui->checkbox_stereo_depthGenerated->isChecked());
	settings.endGroup(); // stereo

	settings.beginGroup("rgb");
	settings.setValue("driver", 	_ui->source_comboBox_image_type->currentIndex());
	settings.endGroup(); // rgb

	settings.beginGroup("Openni");
	settings.setValue("oniPath", 	_ui->lineEdit_openniOniPath->text());
	settings.endGroup(); // Openni

	settings.beginGroup("Openni2");
	settings.setValue("autoWhiteBalance", _ui->openni2_autoWhiteBalance->isChecked());
	settings.setValue("autoExposure", 	  _ui->openni2_autoExposure->isChecked());
	settings.setValue("exposure", 		  _ui->openni2_exposure->value());
	settings.setValue("gain", 		      _ui->openni2_gain->value());
	settings.setValue("mirroring", 		  _ui->openni2_mirroring->isChecked());
	settings.setValue("stampsIdsUsed",    _ui->openni2_stampsIdsUsed->isChecked());
	settings.setValue("oniPath", 		  _ui->lineEdit_openni2OniPath->text());
	settings.endGroup(); // Openni2

	settings.beginGroup("Freenect2");
	settings.setValue("format",             _ui->comboBox_freenect2Format->currentIndex());
	settings.setValue("minDepth",           _ui->doubleSpinBox_freenect2MinDepth->value());
	settings.setValue("maxDepth",           _ui->doubleSpinBox_freenect2MaxDepth->value());
	settings.setValue("bilateralFiltering", _ui->checkBox_freenect2BilateralFiltering->isChecked());
	settings.setValue("edgeAwareFiltering", _ui->checkBox_freenect2EdgeAwareFiltering->isChecked());
	settings.setValue("noiseFiltering",     _ui->checkBox_freenect2NoiseFiltering->isChecked());
	settings.endGroup(); // Freenect2

	settings.beginGroup("RealSense");
	settings.setValue("presetRGB",           _ui->comboBox_realsensePresetRGB->currentIndex());
	settings.setValue("presetDepth",         _ui->comboBox_realsensePresetDepth->currentIndex());
	settings.endGroup(); // RealSense

	settings.beginGroup("RGBDImages");
	settings.setValue("path_rgb",            _ui->lineEdit_cameraRGBDImages_path_rgb->text());
	settings.setValue("path_depth",          _ui->lineEdit_cameraRGBDImages_path_depth->text());
	settings.setValue("scale",               _ui->doubleSpinBox_cameraRGBDImages_scale->value());
	settings.endGroup(); // RGBDImages

	settings.beginGroup("StereoImages");
	settings.setValue("path_left",      _ui->lineEdit_cameraStereoImages_path_left->text());
	settings.setValue("path_right",     _ui->lineEdit_cameraStereoImages_path_right->text());
	settings.setValue("rectify", 	    _ui->checkBox_stereoImages_rectify->isChecked());
	settings.endGroup(); // StereoImages

	settings.beginGroup("StereoVideo");
	settings.setValue("path", 			_ui->lineEdit_cameraStereoVideo_path->text());
	settings.setValue("path2", 			_ui->lineEdit_cameraStereoVideo_path_2->text());
	settings.setValue("rectify", 	    _ui->checkBox_stereoVideo_rectify->isChecked());
	settings.endGroup(); // StereoVideo

	settings.beginGroup("StereoZed");
	settings.setValue("resolution", _ui->comboBox_stereoZed_resolution->currentIndex());
	settings.setValue("quality", _ui->comboBox_stereoZed_quality->currentIndex());
	settings.setValue("self_calibration", _ui->checkbox_stereoZed_selfCalibration->isChecked());
	settings.setValue("sensing_mode", _ui->comboBox_stereoZed_sensingMode->currentIndex());
	settings.setValue("confidence_thr", _ui->spinBox_stereoZed_confidenceThr->value());
	settings.setValue("odom", _ui->checkbox_stereoZed_odom->isChecked());
	settings.setValue("svo_path", _ui->lineEdit_zedSvoPath->text());
	settings.endGroup(); // StereoZed

	

	settings.beginGroup("Images");
	settings.setValue("path", 			_ui->source_images_lineEdit_path->text());
	settings.setValue("startPos", 		_ui->source_images_spinBox_startPos->value());
	settings.setValue("refreshDir", 	_ui->source_images_refreshDir->isChecked());
	settings.setValue("rectify", 	    _ui->checkBox_rgbImages_rectify->isChecked());
	settings.setValue("bayerMode", 	    _ui->comboBox_cameraImages_bayerMode->currentIndex());
	settings.setValue("filenames_as_stamps", _ui->checkBox_cameraImages_timestamps->isChecked());
	settings.setValue("sync_stamps",    _ui->checkBox_cameraImages_syncTimeStamps->isChecked());
	settings.setValue("stamps",              _ui->lineEdit_cameraImages_timestamps->text());
	settings.setValue("path_scans",          _ui->lineEdit_cameraImages_path_scans->text());
	settings.setValue("scan_transform",      _ui->lineEdit_cameraImages_laser_transform->text());
	settings.setValue("scan_max_pts",        _ui->spinBox_cameraImages_max_scan_pts->value());
	settings.setValue("scan_downsample_step", _ui->spinBox_cameraImages_scanDownsampleStep->value());
	settings.setValue("scan_voxel_size",     _ui->doubleSpinBox_cameraImages_scanVoxelSize->value());
	settings.setValue("odom_path",           _ui->lineEdit_cameraImages_odom->text());
	settings.setValue("odom_format",         _ui->comboBox_cameraImages_odomFormat->currentIndex());
	settings.setValue("gt_path",             _ui->lineEdit_cameraImages_gt->text());
	settings.setValue("gt_format",           _ui->comboBox_cameraImages_gtFormat->currentIndex());
	settings.endGroup(); // images

	settings.beginGroup("Video");
	settings.setValue("path", 			_ui->source_video_lineEdit_path->text());
	settings.setValue("rectify", 	    _ui->checkBox_rgbVideo_rectify->isChecked());
	settings.endGroup(); // video

	settings.beginGroup("ScanFromDepth");
	settings.setValue("enabled", 			_ui->groupBox_scanFromDepth->isChecked());
	settings.setValue("decimation", 		_ui->spinBox_cameraScanFromDepth_decimation->value());
	settings.setValue("maxDepth", 			_ui->doubleSpinBox_cameraSCanFromDepth_maxDepth->value());
	settings.setValue("voxelSize", 			_ui->doubleSpinBox_cameraImages_scanVoxelSize->value());
	settings.setValue("normalsK", 			_ui->spinBox_cameraImages_scanNormalsK->value());
	settings.endGroup();

	settings.beginGroup("DepthFromScan");
	settings.setValue("depthFromScan", _ui->groupBox_depthFromScan->isChecked());
	settings.setValue("depthFromScanFillHoles", _ui->groupBox_depthFromScan_fillHoles->isChecked());
	settings.setValue("depthFromScanVertical", _ui->radioButton_depthFromScan_vertical->isChecked());
	settings.setValue("depthFromScanHorizontal", _ui->radioButton_depthFromScan_horizontal->isChecked());
	settings.setValue("depthFromScanFillBorders", _ui->checkBox_depthFromScan_fillBorders->isChecked());
	settings.endGroup();

	settings.beginGroup("Database");
	settings.setValue("path", 			   _ui->source_database_lineEdit_path->text());
	settings.setValue("ignoreOdometry",    _ui->source_checkBox_ignoreOdometry->isChecked());
	settings.setValue("ignoreGoalDelay",   _ui->source_checkBox_ignoreGoalDelay->isChecked());
	settings.setValue("ignoreGoals",       _ui->source_checkBox_ignoreGoals->isChecked());
	settings.setValue("startPos",          _ui->source_spinBox_databaseStartPos->value());
	settings.setValue("cameraIndex",       _ui->source_spinBox_database_cameraIndex->value());
	settings.setValue("useDatabaseStamps", _ui->source_checkBox_useDbStamps->isChecked());
	settings.endGroup(); // Database

	settings.endGroup(); // Camera

	_calibrationDialog->saveSettings(settings, "CalibrationDialog");
}

void PreferencesDialog::writeCoreSettings(const QString & filePath) const
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}

	Parameters::writeINI(path.toStdString(), this->getAllParameters());
}

bool PreferencesDialog::validateForm()
{
#ifndef RTABMAP_NONFREE
	// verify that SURF/SIFT cannot be selected if not built with OpenCV nonfree module
	// BOW dictionary type
	if(_ui->comboBox_detector_strategy->currentIndex() <= 1)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected feature type (SURF/SIFT) is not available. RTAB-Map is not built "
				   "with the nonfree module from OpenCV. ORB is set instead for the bag-of-words dictionary."));
		_ui->comboBox_detector_strategy->setCurrentIndex(Feature2D::kFeatureOrb);
	}
	// BOW Reextract features type
	if(_ui->reextract_type->currentIndex() <= 1)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected feature type (SURF/SIFT) is not available. RTAB-Map is not built "
				   "with the nonfree module from OpenCV. Fast/Brief is set instead for the re-extraction "
				   "of features on loop closure."));
		_ui->reextract_type->setCurrentIndex(Feature2D::kFeatureFastBrief);
	}
#endif

#if CV_MAJOR_VERSION < 3
	if (_ui->comboBox_detector_strategy->currentIndex() == Feature2D::kFeatureFreak)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
			tr("Selected feature type (FREAK detector) is not available on OpenCV2. ORB is set instead "
				"for the bag-of-words dictionary."));
		_ui->comboBox_detector_strategy->setCurrentIndex(Feature2D::kFeatureOrb);
	}
	if (_ui->reextract_type->currentIndex() == Feature2D::kFeatureFreak)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
			tr("Selected feature type (FREAK detector) is not available on OpenCV2. ORB is set instead "
				"for the re-extraction of features on loop closure."));
				_ui->reextract_type->setCurrentIndex(Feature2D::kFeatureOrb);
	}
#endif

	// optimization strategy
	if(_ui->graphOptimization_type->currentIndex() == 0 && !Optimizer::isAvailable(Optimizer::kTypeTORO))
	{
		if(Optimizer::isAvailable(Optimizer::kTypeGTSAM))
		{
			QMessageBox::warning(this, tr("Parameter warning"),
					tr("Selected graph optimization strategy (TORO) is not available. RTAB-Map is not built "
					   "with TORO. GTSAM is set instead for graph optimization strategy."));
			_ui->graphOptimization_type->setCurrentIndex(Optimizer::kTypeGTSAM);
		}
		else if(Optimizer::isAvailable(Optimizer::kTypeG2O))
		{
			QMessageBox::warning(this, tr("Parameter warning"),
					tr("Selected graph optimization strategy (TORO) is not available. RTAB-Map is not built "
					   "with TORO. g2o is set instead for graph optimization strategy."));
			_ui->graphOptimization_type->setCurrentIndex(Optimizer::kTypeG2O);
		}
	}
	if(_ui->graphOptimization_type->currentIndex() == 1 && !Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		if(Optimizer::isAvailable(Optimizer::kTypeGTSAM))
		{
			QMessageBox::warning(this, tr("Parameter warning"),
					tr("Selected graph optimization strategy (g2o) is not available. RTAB-Map is not built "
					   "with g2o. GTSAM is set instead for graph optimization strategy."));
			_ui->graphOptimization_type->setCurrentIndex(Optimizer::kTypeGTSAM);
		}
		else if(Optimizer::isAvailable(Optimizer::kTypeTORO))
		{
			QMessageBox::warning(this, tr("Parameter warning"),
					tr("Selected graph optimization strategy (g2o) is not available. RTAB-Map is not built "
					   "with g2o. TORO is set instead for graph optimization strategy."));
			_ui->graphOptimization_type->setCurrentIndex(Optimizer::kTypeTORO);
		}
	}
	if(_ui->graphOptimization_type->currentIndex() == 2 && !Optimizer::isAvailable(Optimizer::kTypeGTSAM))
	{
		if(Optimizer::isAvailable(Optimizer::kTypeG2O))
		{
			QMessageBox::warning(this, tr("Parameter warning"),
					tr("Selected graph optimization strategy (GTSAM) is not available. RTAB-Map is not built "
					   "with GTSAM. g2o is set instead for graph optimization strategy."));
			_ui->graphOptimization_type->setCurrentIndex(Optimizer::kTypeG2O);
		}
		else if(Optimizer::isAvailable(Optimizer::kTypeTORO))
		{
			QMessageBox::warning(this, tr("Parameter warning"),
					tr("Selected graph optimization strategy (GTSAM) is not available. RTAB-Map is not built "
					   "with GTSAM. TORO is set instead for graph optimization strategy."));
			_ui->graphOptimization_type->setCurrentIndex(Optimizer::kTypeTORO);
		}
	}

	// Registration bundle adjustment strategy
	if(_ui->loopClosure_bundle->currentIndex() == 1 && !Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected visual registration bundle adjustment optimization strategy (g2o) is not available. RTAB-Map is not built "
				   "with g2o. Bundle adjustment is disabled."));
		_ui->loopClosure_bundle->setCurrentIndex(0);
	}
	if(_ui->loopClosure_bundle->currentIndex() == 2 && !Optimizer::isAvailable(Optimizer::kTypeCVSBA))
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected visual registration bundle adjustment optimization strategy (cvsba) is not available. RTAB-Map is not built "
				   "with cvsba. Bundle adjustment is disabled."));
		_ui->loopClosure_bundle->setCurrentIndex(0);
	}

	// Local bundle adjustment strategy for odometry F2M
	if(_ui->odom_f2m_bundleStrategy->currentIndex() == 1 && !Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected odometry local bundle adjustment optimization strategy (g2o) is not available. RTAB-Map is not built "
				   "with g2o. Bundle adjustment is disabled."));
		_ui->odom_f2m_bundleStrategy->setCurrentIndex(0);
	}
	if(_ui->odom_f2m_bundleStrategy->currentIndex() == 2 && !Optimizer::isAvailable(Optimizer::kTypeCVSBA))
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected odometry local bundle adjustment optimization strategy (cvsba) is not available. RTAB-Map is not built "
				   "with cvsba. Bundle adjustment is disabled."));
		_ui->odom_f2m_bundleStrategy->setCurrentIndex(0);
	}
	if(_ui->odom_strategy->currentIndex() == 0 && // F2M
		_ui->odom_f2m_bundleStrategy->currentIndex() > 0 &&
		_ui->loopClosure_correspondencesType->currentIndex() == 1)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Odometry local bundle adjustment optimization cannot be used at the same time than Optical Flow correspondences "
					"strategy (see Visual Registration panel). Bundle adjustment is disabled."));
		_ui->odom_f2m_bundleStrategy->setCurrentIndex(0);
	}

	// verify that Robust and Reject threshold are not set at the same time
	if(_ui->graphOptimization_robust->isChecked() && _ui->graphOptimization_maxError->value()>0.0)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Robust graph optimization and maximum optimization error threshold cannot be "
				   "both used at the same time. Disabling robust optimization."));
		_ui->graphOptimization_robust->setChecked(false);
	}

	//verify binary features and nearest neighbor
	// BOW dictionary type
	if(_ui->comboBox_dictionary_strategy->currentIndex() == VWDictionary::kNNFlannLSH && _ui->comboBox_detector_strategy->currentIndex() <= 1)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("With the selected feature type (SURF or SIFT), parameter \"Visual word->Nearest Neighbor\" "
				   "cannot be LSH (used for binary descriptor). KD-tree is set instead for the bag-of-words dictionary."));
		_ui->comboBox_dictionary_strategy->setCurrentIndex(VWDictionary::kNNFlannKdTree);
	}

	// BOW Reextract features type
	if(_ui->reextract_nn->currentIndex() == VWDictionary::kNNFlannLSH && _ui->reextract_type->currentIndex() <= 1)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("With the selected feature type (SURF or SIFT), parameter \"Visual word->Nearest Neighbor\" "
				   "cannot be LSH (used for binary descriptor). KD-tree is set instead for the re-extraction "
					   "of features on loop closure."));
		_ui->reextract_nn->setCurrentIndex(VWDictionary::kNNFlannKdTree);
	}

	if(_ui->doubleSpinBox_freenect2MinDepth->value() >= _ui->doubleSpinBox_freenect2MaxDepth->value())
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Freenect2 minimum depth (%1 m) should be lower than maximum depth (%2 m). Setting maximum depth to %3 m.")
				   .arg(_ui->doubleSpinBox_freenect2MinDepth->value())
				   .arg(_ui->doubleSpinBox_freenect2MaxDepth->value())
				   .arg(_ui->doubleSpinBox_freenect2MaxDepth->value()+1));
		_ui->doubleSpinBox_freenect2MaxDepth->setValue(_ui->doubleSpinBox_freenect2MaxDepth->value()+1);
	}

	if(_ui->surf_doubleSpinBox_maxDepth->value() > 0.0 &&
	   _ui->surf_doubleSpinBox_minDepth->value() >= _ui->surf_doubleSpinBox_maxDepth->value())
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Visual word minimum depth (%1 m) should be lower than maximum depth (%2 m). Setting maximum depth to %3 m.")
				   .arg(_ui->surf_doubleSpinBox_minDepth->value())
				   .arg(_ui->surf_doubleSpinBox_maxDepth->value())
				   .arg(_ui->surf_doubleSpinBox_maxDepth->value()+1));
		_ui->doubleSpinBox_freenect2MinDepth->setValue(0);
	}

	if(_ui->loopClosure_bowMaxDepth->value() > 0.0 &&
	   _ui->loopClosure_bowMinDepth->value() >= _ui->loopClosure_bowMaxDepth->value())
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Visual registration word minimum depth (%1 m) should be lower than maximum depth (%2 m). Setting maximum depth to %3 m.")
				   .arg(_ui->loopClosure_bowMinDepth->value())
				   .arg(_ui->loopClosure_bowMaxDepth->value())
				   .arg(_ui->loopClosure_bowMaxDepth->value()+1));
		_ui->loopClosure_bowMinDepth->setValue(0);
	}

	if(_ui->fastThresholdMax->value() < _ui->fastThresholdMin->value())
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("FAST minimum threshold cannot be lower than maximum threshold. Setting minimum to 1."));
		_ui->fastThresholdMin->setValue(1);
	}
	if(_ui->fastThreshold->value() > _ui->fastThresholdMax->value())
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("FAST threshold cannot be higher than maximum threshold. Maximum value set to current threshold."));
		_ui->fastThresholdMax->setValue(_ui->fastThreshold->value());
	}
	if(_ui->fastThreshold->value() < _ui->fastThresholdMin->value())
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("FAST threshold cannot be lower than minimum threshold. Minimum value set to current threshold."));
		_ui->fastThresholdMin->setValue(_ui->fastThreshold->value());
	}

	if(_ui->checkbox_odomDisabled->isChecked() &&
		_ui->general_checkBox_SLAM_mode->isChecked() &&
		_ui->general_checkBox_activateRGBD->isChecked())
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Odometry is disabled but incremental RGB-D SLAM is activated! Re-enabling odometry."));
		_ui->checkbox_odomDisabled->setChecked(false);
	}

	return true;
}

QString PreferencesDialog::getParamMessage()
{
	return tr("Reading parameters from the configuration file...");
}

void PreferencesDialog::showEvent ( QShowEvent * event )
{
	if(_monitoringState)
	{
		// In monitoring state, you cannot change remote paths
		_ui->lineEdit_dictionaryPath->setEnabled(false);
		_ui->toolButton_dictionaryPath->setEnabled(false);
		_ui->label_dictionaryPath->setEnabled(false);

		_ui->groupBox_source0->setEnabled(false);
		_ui->groupBox_odometry1->setEnabled(false);

		_ui->checkBox_useOdomFeatures->setChecked(false);
		_ui->checkBox_useOdomFeatures->setEnabled(false);

		this->setWindowTitle(tr("Preferences [Monitoring mode]"));
	}
	else
	{
		_ui->lineEdit_dictionaryPath->setEnabled(true);
		_ui->toolButton_dictionaryPath->setEnabled(true);
		_ui->label_dictionaryPath->setEnabled(true);

		_ui->groupBox_source0->setEnabled(true);
		_ui->groupBox_odometry1->setEnabled(true);

		_ui->checkBox_useOdomFeatures->setEnabled(true);

		this->setWindowTitle(tr("Preferences"));
	}

	// setup logger filter
	std::map<std::string, unsigned long> threads = ULogger::getRegisteredThreads();
	std::vector<std::string> threadsChecked = this->getGeneralLoggerThreads();
	std::set<std::string> threadsCheckedSet(threadsChecked.begin(), threadsChecked.end());
	_ui->comboBox_loggerFilter->clear();
	for(std::map<std::string, unsigned long>::iterator iter=threads.begin(); iter!=threads.end(); ++iter)
	{
		if(threadsCheckedSet.find(iter->first.c_str()) != threadsCheckedSet.end())
		{
			_ui->comboBox_loggerFilter->addItem(QString(iter->first.c_str()), QVariant(true));
		}
		else
		{
			_ui->comboBox_loggerFilter->addItem(QString(iter->first.c_str()), QVariant(false));
		}
	}

	this->readSettingsBegin();
}

void PreferencesDialog::readSettingsBegin()
{
	_progressDialog->setLabelText(this->getParamMessage());
	_progressDialog->show();

	// this will let the MainThread to display the progress dialog before reading the parameters...
	QTimer::singleShot(10, this, SLOT(readSettingsEnd()));
}

void PreferencesDialog::readSettingsEnd()
{
	QApplication::processEvents();

	this->readSettings(getTmpIniFilePath());

	_progressDialog->setValue(1);
	if(this->isVisible())
	{
		_progressDialog->setLabelText(tr("Setup dialog..."));

		this->updatePredictionPlot();
		this->setupKpRoiPanel();
	}

	_progressDialog->setValue(2); // this will make closing...
}

void PreferencesDialog::saveWindowGeometry(const QWidget * window)
{
	if(!window->objectName().isEmpty() && !window->isMaximized())
	{
		QSettings settings(getIniFilePath(), QSettings::IniFormat);
		settings.beginGroup("Gui");
		settings.beginGroup(window->objectName());
		settings.setValue("geometry", window->saveGeometry());
		settings.endGroup(); // "windowName"
		settings.endGroup(); // rtabmap
	}
}

void PreferencesDialog::loadWindowGeometry(QWidget * window)
{
	if(!window->objectName().isEmpty())
	{
		QByteArray bytes;
		QSettings settings(getIniFilePath(), QSettings::IniFormat);
		settings.beginGroup("Gui");
		settings.beginGroup(window->objectName());
		bytes = settings.value("geometry", QByteArray()).toByteArray();
		if(!bytes.isEmpty())
		{
			window->restoreGeometry(bytes);
		}
		settings.endGroup(); // "windowName"
		settings.endGroup(); // rtabmap
	}
}

void PreferencesDialog::saveMainWindowState(const QMainWindow * mainWindow)
{
	if(!mainWindow->objectName().isEmpty())
	{
		saveWindowGeometry(mainWindow);

		QSettings settings(getIniFilePath(), QSettings::IniFormat);
		settings.beginGroup("Gui");
		settings.beginGroup(mainWindow->objectName());
		settings.setValue("state", mainWindow->saveState());
		settings.setValue("maximized", mainWindow->isMaximized());
		settings.setValue("status_bar", mainWindow->statusBar()->isVisible());
		settings.endGroup(); // "MainWindow"
		settings.endGroup(); // rtabmap
	}
}

void PreferencesDialog::loadMainWindowState(QMainWindow * mainWindow,  bool & maximized, bool & statusBarShown)
{
	if(!mainWindow->objectName().isEmpty())
	{
		loadWindowGeometry(mainWindow);

		QByteArray bytes;
		QSettings settings(getIniFilePath(), QSettings::IniFormat);
		settings.beginGroup("Gui");
		settings.beginGroup(mainWindow->objectName());
		bytes = settings.value("state", QByteArray()).toByteArray();
		if(!bytes.isEmpty())
		{
			mainWindow->restoreState(bytes);
		}
		maximized = settings.value("maximized", false).toBool();
		statusBarShown = settings.value("status_bar", false).toBool();
		mainWindow->statusBar()->setVisible(statusBarShown);
		settings.endGroup(); // "MainWindow"
		settings.endGroup(); // rtabmap
	}
}

void PreferencesDialog::saveWidgetState(const QWidget * widget)
{
	if(!widget->objectName().isEmpty())
	{
		QSettings settings(getIniFilePath(), QSettings::IniFormat);
		settings.beginGroup("Gui");
		settings.beginGroup(widget->objectName());

		const CloudViewer * cloudViewer = qobject_cast<const CloudViewer*>(widget);
		const ImageView * imageView = qobject_cast<const ImageView*>(widget);
		const ExportCloudsDialog * exportCloudsDialog = qobject_cast<const ExportCloudsDialog*>(widget);
		const ExportScansDialog * exportScansDialog = qobject_cast<const ExportScansDialog*>(widget);
		const PostProcessingDialog * postProcessingDialog = qobject_cast<const PostProcessingDialog *>(widget);
		const GraphViewer * graphViewer = qobject_cast<const GraphViewer *>(widget);
		const CalibrationDialog * calibrationDialog = qobject_cast<const CalibrationDialog *>(widget);
		const DepthCalibrationDialog * depthCalibrationDialog = qobject_cast<const DepthCalibrationDialog *>(widget);

		if(cloudViewer)
		{
			cloudViewer->saveSettings(settings);
		}
		else if(imageView)
		{
			imageView->saveSettings(settings);
		}
		else if(exportCloudsDialog)
		{
			exportCloudsDialog->saveSettings(settings);
		}
		else if(exportScansDialog)
		{
			exportScansDialog->saveSettings(settings);
		}
		else if(postProcessingDialog)
		{
			postProcessingDialog->saveSettings(settings);
		}
		else if(graphViewer)
		{
			graphViewer->saveSettings(settings);
		}
		else if(calibrationDialog)
		{
			calibrationDialog->saveSettings(settings);
		}
		else if(depthCalibrationDialog)
		{
			depthCalibrationDialog->saveSettings(settings);
		}
		else
		{
			UERROR("Widget \"%s\" cannot be exported in config file.", widget->objectName().toStdString().c_str());
		}

		settings.endGroup(); // "name"
		settings.endGroup(); // Gui
	}
}

void PreferencesDialog::loadWidgetState(QWidget * widget)
{
	if(!widget->objectName().isEmpty())
	{
		QByteArray bytes;
		QSettings settings(getIniFilePath(), QSettings::IniFormat);
		settings.beginGroup("Gui");
		settings.beginGroup(widget->objectName());

		CloudViewer * cloudViewer = qobject_cast<CloudViewer*>(widget);
		ImageView * imageView = qobject_cast<ImageView*>(widget);
		ExportCloudsDialog * exportCloudsDialog = qobject_cast<ExportCloudsDialog*>(widget);
		ExportScansDialog * exportScansDialog = qobject_cast<ExportScansDialog*>(widget);
		PostProcessingDialog * postProcessingDialog = qobject_cast<PostProcessingDialog *>(widget);
		GraphViewer * graphViewer = qobject_cast<GraphViewer *>(widget);
		CalibrationDialog * calibrationDialog = qobject_cast<CalibrationDialog *>(widget);
		DepthCalibrationDialog * depthCalibrationDialog = qobject_cast<DepthCalibrationDialog *>(widget);

		if(cloudViewer)
		{
			cloudViewer->loadSettings(settings);
		}
		else if(imageView)
		{
			imageView->loadSettings(settings);
		}
		else if(exportCloudsDialog)
		{
			exportCloudsDialog->loadSettings(settings);
		}
		else if(exportScansDialog)
		{
			exportScansDialog->loadSettings(settings);
		}
		else if(postProcessingDialog)
		{
			postProcessingDialog->loadSettings(settings);
		}
		else if(graphViewer)
		{
			graphViewer->loadSettings(settings);
		}
		else if(calibrationDialog)
		{
			calibrationDialog->loadSettings(settings);
		}
		else if(depthCalibrationDialog)
		{
			depthCalibrationDialog->loadSettings(settings);
		}
		else
		{
			UERROR("Widget \"%s\" cannot be loaded from config file.", widget->objectName().toStdString().c_str());
		}

		settings.endGroup(); //"name"
		settings.endGroup(); // Gui
	}
}


void PreferencesDialog::saveCustomConfig(const QString & section, const QString & key, const QString & value)
{
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup(section);
	settings.setValue(key, value);
	settings.endGroup(); // "section"
	settings.endGroup(); // rtabmap
}

QString PreferencesDialog::loadCustomConfig(const QString & section, const QString & key)
{
	QString value;
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup(section);
	value = settings.value(key, QString()).toString();
	settings.endGroup(); // "section"
	settings.endGroup(); // rtabmap
	return value;
}

rtabmap::ParametersMap PreferencesDialog::getAllParameters() const
{
	if(_parameters.size() != Parameters::getDefaultParameters().size())
	{
		UWARN("%d vs %d (Is PreferencesDialog::init() called?)", (int)_parameters.size(), (int)Parameters::getDefaultParameters().size());
	}
	ParametersMap parameters = _parameters;
	uInsert(parameters, _modifiedParameters);

	return parameters;
}

std::string PreferencesDialog::getParameter(const std::string & key) const
{
	if(_modifiedParameters.find(key) != _modifiedParameters.end())
	{
		return _modifiedParameters.at(key);
	}

	UASSERT(_parameters.find(key) != _parameters.end());
	return _parameters.at(key);
}

void PreferencesDialog::updateParameters(const ParametersMap & parameters)
{
	if(parameters.size())
	{
		for(rtabmap::ParametersMap::const_iterator iter = parameters.begin(); iter!=parameters.end(); ++iter)
		{
			this->setParameter(iter->first, iter->second);
		}
		if(!this->isVisible())
		{
			this->writeSettings(getTmpIniFilePath());
		}
	}
}

void PreferencesDialog::selectSourceDriver(Src src)
{
	if(src >= kSrcRGBD && src<kSrcStereo)
	{
		_ui->comboBox_sourceType->setCurrentIndex(0);
		_ui->comboBox_cameraRGBD->setCurrentIndex(src - kSrcRGBD);
		if(src == kSrcOpenNI_PCL)
		{
			_ui->lineEdit_openniOniPath->clear();
		}
		else if(src == kSrcOpenNI2)
		{
			_ui->lineEdit_openni2OniPath->clear();
		}
	}
	else if(src >= kSrcStereo && src<kSrcRGB)
	{
		_ui->comboBox_sourceType->setCurrentIndex(1);
		_ui->comboBox_cameraStereo->setCurrentIndex(src - kSrcStereo);
	}
	else if(src >= kSrcRGB && src<kSrcDatabase)
	{
		_ui->comboBox_sourceType->setCurrentIndex(2);
		_ui->source_comboBox_image_type->setCurrentIndex(src - kSrcRGB);
	}
	else if(src >= kSrcDatabase)
	{
		_ui->comboBox_sourceType->setCurrentIndex(3);
	}

	if(validateForm())
	{
		// Even if there is no change, MainWindow should be notified
		makeObsoleteSourcePanel();

		this->writeSettings(getTmpIniFilePath());
	}
	else
	{
		this->readSettingsBegin();
	}
}

void PreferencesDialog::selectSourceDatabase()
{
	QString dir = _ui->source_database_lineEdit_path->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QStringList paths = QFileDialog::getOpenFileNames(this, tr("Select file"), dir, tr("RTAB-Map database files (*.db)"));
	if(paths.size())
	{
		int r = QMessageBox::question(this, tr("Odometry in database..."), tr("Use odometry saved in database (if some saved)?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
		_ui->source_checkBox_ignoreOdometry->setChecked(r != QMessageBox::Yes);
		
		if (_ui->general_doubleSpinBox_detectionRate->value() != 0 && _ui->general_spinBox_imagesBufferSize->value() != 0)
		{
			r = QMessageBox::question(this, tr("Detection rate..."), 
				tr("Do you want to process all frames? \n\nClicking \"Yes\" will set "
					"RTAB-Map's detection rate (%1 Hz) and buffer size (%2) to 0 to make "
					"sure that all frames are processed.")
					.arg(_ui->general_doubleSpinBox_detectionRate->value())
					.arg(_ui->general_spinBox_imagesBufferSize->value()),
						QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
			if (r == QMessageBox::Yes)
			{
				_ui->general_doubleSpinBox_detectionRate->setValue(0.0);
				_ui->general_spinBox_imagesBufferSize->setValue(0);
			}
		}
		_ui->source_database_lineEdit_path->setText(paths.size()==1?paths.front():paths.join(";"));
		_ui->source_spinBox_databaseStartPos->setValue(0);
		_ui->source_spinBox_database_cameraIndex->setValue(-1);
	}
}

void PreferencesDialog::openDatabaseViewer()
{
	DatabaseViewer * viewer = new DatabaseViewer(getIniFilePath(), this);
	viewer->setWindowModality(Qt::WindowModal);
	viewer->setAttribute(Qt::WA_DeleteOnClose, true);
	viewer->showCloseButton();
	if(viewer->openDatabase(_ui->source_database_lineEdit_path->text()))
	{
		viewer->show();
	}
	else
	{
		delete viewer;
	}
}

void PreferencesDialog::visualizeDistortionModel()
{
	if(!_ui->lineEdit_source_distortionModel->text().isEmpty() &&
			QFileInfo(_ui->lineEdit_source_distortionModel->text()).exists())
	{
		clams::DiscreteDepthDistortionModel model;
		model.load(_ui->lineEdit_source_distortionModel->text().toStdString());
		if(model.isValid())
		{
			cv::Mat img = model.visualize();
			QString name = QFileInfo(_ui->lineEdit_source_distortionModel->text()).baseName()+".png";
			cv::imwrite(name.toStdString(), img);
			QDesktopServices::openUrl(QUrl::fromLocalFile(name));
		}
		else
		{
			QMessageBox::warning(this, tr("Distortion Model"), tr("Model loaded from \"%1\" is not valid!").arg(_ui->lineEdit_source_distortionModel->text()));
		}
	}
	else
	{
		QMessageBox::warning(this, tr("Distortion Model"), tr("File \"%1\" doesn't exist!").arg(_ui->lineEdit_source_distortionModel->text()));
	}
}

void PreferencesDialog::selectCalibrationPath()
{
	QString dir = _ui->lineEdit_calibrationFile->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory()+"/camera_info";
	}
	else if(!dir.contains('.'))
	{
		dir = getWorkingDirectory()+"/camera_info/"+dir;
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Calibration file (*.yaml)"));
	if(path.size())
	{
		_ui->lineEdit_calibrationFile->setText(path);
	}
}

void PreferencesDialog::selectSourceImagesStamps()
{
	QString dir = _ui->lineEdit_cameraImages_timestamps->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Timestamps file (*.txt)"));
	if(path.size())
	{
		_ui->lineEdit_cameraImages_timestamps->setText(path);
	}
}

void PreferencesDialog::selectSourceRGBDImagesPathRGB()
{
	QString dir = _ui->lineEdit_cameraRGBDImages_path_rgb->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getExistingDirectory(this, tr("Select RGB images directory"), dir);
	if(path.size())
	{
		_ui->lineEdit_cameraRGBDImages_path_rgb->setText(path);
	}
}


void PreferencesDialog::selectSourceImagesPathScans()
{
	QString dir = _ui->lineEdit_cameraImages_path_scans->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getExistingDirectory(this, tr("Select scans directory"), dir);
	if(path.size())
	{
		_ui->lineEdit_cameraImages_path_scans->setText(path);
	}
}

void PreferencesDialog::selectSourceRGBDImagesPathDepth()
{
	QString dir = _ui->lineEdit_cameraRGBDImages_path_depth->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getExistingDirectory(this, tr("Select depth images directory"), dir);
	if(path.size())
	{
		_ui->lineEdit_cameraRGBDImages_path_depth->setText(path);
	}
}

void PreferencesDialog::selectSourceImagesPathOdom()
{
	QString dir = _ui->lineEdit_cameraImages_odom->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Odometry (*.txt *.log *.toro *.g2o)"));
	if(path.size())
	{
		QStringList list;
		for(int i=0; i<_ui->comboBox_cameraImages_odomFormat->count(); ++i)
		{
			list.push_back(_ui->comboBox_cameraImages_odomFormat->itemText(i));
		}
		QString item = QInputDialog::getItem(this, tr("Odometry Format"), tr("Format:"), list, _ui->comboBox_cameraImages_odomFormat->currentIndex(), false);
		if(!item.isEmpty())
		{
			_ui->lineEdit_cameraImages_odom->setText(path);
			_ui->comboBox_cameraImages_odomFormat->setCurrentIndex(_ui->comboBox_cameraImages_odomFormat->findText(item));
		}
	}
}

void PreferencesDialog::selectSourceImagesPathGt()
{
	QString dir = _ui->lineEdit_cameraImages_gt->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Ground Truth (*.txt *.log *.toro *.g2o)"));
	if(path.size())
	{
		QStringList list;
		for(int i=0; i<_ui->comboBox_cameraImages_gtFormat->count(); ++i)
		{
			list.push_back(_ui->comboBox_cameraImages_gtFormat->itemText(i));
		}
		QString item = QInputDialog::getItem(this, tr("Ground Truth Format"), tr("Format:"), list, _ui->comboBox_cameraImages_gtFormat->currentIndex(), false);
		if(!item.isEmpty())
		{
			_ui->lineEdit_cameraImages_gt->setText(path);
			_ui->comboBox_cameraImages_gtFormat->setCurrentIndex(_ui->comboBox_cameraImages_gtFormat->findText(item));
		}
	}
}

void PreferencesDialog::selectSourceStereoImagesPathLeft()
{
	QString dir = _ui->lineEdit_cameraStereoImages_path_left->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getExistingDirectory(this, tr("Select left images directory"), dir);
	if(path.size())
	{
		_ui->lineEdit_cameraStereoImages_path_left->setText(path);
	}
}

void PreferencesDialog::selectSourceStereoImagesPathRight()
{
	QString dir = _ui->lineEdit_cameraStereoImages_path_right->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getExistingDirectory(this, tr("Select right images directory"), dir);
	if(path.size())
	{
		_ui->lineEdit_cameraStereoImages_path_right->setText(path);
	}
}

void PreferencesDialog::selectSourceImagesPath()
{
	QString dir = _ui->source_images_lineEdit_path->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getExistingDirectory(this, tr("Select images directory"), _ui->source_images_lineEdit_path->text());
	if(!path.isEmpty())
	{
		_ui->source_images_lineEdit_path->setText(path);
		_ui->source_images_spinBox_startPos->setValue(0);
	}
}

void PreferencesDialog::selectSourceVideoPath()
{
	QString dir = _ui->source_video_lineEdit_path->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->source_video_lineEdit_path->text(), tr("Videos (*.avi *.mpg *.mp4 *.mov *.mkv)"));
	if(!path.isEmpty())
	{
		_ui->source_video_lineEdit_path->setText(path);
	}
}

void PreferencesDialog::selectSourceStereoVideoPath()
{
	QString dir = _ui->lineEdit_cameraStereoVideo_path->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->lineEdit_cameraStereoVideo_path->text(), tr("Videos (*.avi *.mpg *.mp4 *.mov *.mkv)"));
	if(!path.isEmpty())
	{
		_ui->lineEdit_cameraStereoVideo_path->setText(path);
	}
}

void PreferencesDialog::selectSourceStereoVideoPath2()
{
	QString dir = _ui->lineEdit_cameraStereoVideo_path_2->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->lineEdit_cameraStereoVideo_path_2->text(), tr("Videos (*.avi *.mpg *.mp4 *.mov *.mkv)"));
	if(!path.isEmpty())
	{
		_ui->lineEdit_cameraStereoVideo_path_2->setText(path);
	}
}

void PreferencesDialog::selectSourceDistortionModel()
{
	QString dir = _ui->lineEdit_source_distortionModel->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->lineEdit_source_distortionModel->text(), tr("Distortion model (*.bin *.txt)"));
	if(!path.isEmpty())
	{
		_ui->lineEdit_source_distortionModel->setText(path);
	}
}

void PreferencesDialog::selectSourceOniPath()
{
	QString dir = _ui->lineEdit_openniOniPath->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->lineEdit_openniOniPath->text(), tr("OpenNI (*.oni)"));
	if(!path.isEmpty())
	{
		_ui->lineEdit_openniOniPath->setText(path);
	}
}

void PreferencesDialog::selectSourceOni2Path()
{
	QString dir = _ui->lineEdit_openni2OniPath->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->lineEdit_openni2OniPath->text(), tr("OpenNI (*.oni)"));
	if(!path.isEmpty())
	{
		_ui->lineEdit_openni2OniPath->setText(path);
	}
}

void PreferencesDialog::selectSourceSvoPath()
{
	QString dir = _ui->lineEdit_zedSvoPath->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->lineEdit_zedSvoPath->text(), tr("ZED (*.svo)"));
	if(!path.isEmpty())
	{
		_ui->lineEdit_zedSvoPath->setText(path);
	}
}

void PreferencesDialog::setParameter(const std::string & key, const std::string & value)
{
	UDEBUG("%s=%s", key.c_str(), value.c_str());
	QWidget * obj = _ui->stackedWidget->findChild<QWidget*>(key.c_str());
	if(obj)
	{
		uInsert(_parameters, ParametersPair(key, value));

		QSpinBox * spin = qobject_cast<QSpinBox *>(obj);
		QDoubleSpinBox * doubleSpin = qobject_cast<QDoubleSpinBox *>(obj);
		QComboBox * combo = qobject_cast<QComboBox *>(obj);
		QCheckBox * check = qobject_cast<QCheckBox *>(obj);
		QRadioButton * radio = qobject_cast<QRadioButton *>(obj);
		QLineEdit * lineEdit = qobject_cast<QLineEdit *>(obj);
		QGroupBox * groupBox = qobject_cast<QGroupBox *>(obj);
		bool ok;
		if(spin)
		{
			spin->setValue(QString(value.c_str()).toInt(&ok));
			if(!ok)
			{
				UERROR("Conversion failed from \"%s\" for parameter %s", value.c_str(), key.c_str());
			}
		}
		else if(doubleSpin)
		{
			doubleSpin->setValue(QString(value.c_str()).toDouble(&ok));
			if(!ok)
			{
				UERROR("Conversion failed from \"%s\" for parameter %s", value.c_str(), key.c_str());
			}
		}
		else if(combo)
		{
			int valueInt = QString(value.c_str()).toInt(&ok);
			if(!ok)
			{
				UERROR("Conversion failed from \"%s\" for parameter %s", value.c_str(), key.c_str());
			}
			else
			{
#ifndef RTABMAP_NONFREE
				if(valueInt <= 1 &&
						(combo->objectName().toStdString().compare(Parameters::kKpDetectorStrategy()) == 0 ||
						 combo->objectName().toStdString().compare(Parameters::kVisFeatureType()) == 0))
				{
					UWARN("Trying to set \"%s\" to SIFT/SURF but RTAB-Map isn't built "
						  "with the nonfree module from OpenCV. Keeping default combo value: %s.",
						  combo->objectName().toStdString().c_str(),
						  combo->currentText().toStdString().c_str());
					ok = false;
				}
#endif
				if(!Optimizer::isAvailable(Optimizer::kTypeG2O))
				{
					if(valueInt==1 && combo->objectName().toStdString().compare(Parameters::kOptimizerStrategy()) == 0)
					{
						UWARN("Trying to set \"%s\" to g2o but RTAB-Map isn't built "
							  "with g2o. Keeping default combo value: %s.",
							  combo->objectName().toStdString().c_str(),
							  combo->currentText().toStdString().c_str());
						ok = false;
					}
				}
				if(!Optimizer::isAvailable(Optimizer::kTypeGTSAM))
				{
					if(valueInt==2 && combo->objectName().toStdString().compare(Parameters::kOptimizerStrategy()) == 0)
					{
						UWARN("Trying to set \"%s\" to GTSAM but RTAB-Map isn't built "
							  "with GTSAM. Keeping default combo value: %s.",
							  combo->objectName().toStdString().c_str(),
							  combo->currentText().toStdString().c_str());
						ok = false;
					}
				}
				if(ok)
				{
					combo->setCurrentIndex(valueInt);
				}
			}

		}
		else if(check)
		{
			_ui->checkBox_useOdomFeatures->blockSignals(true);
			check->setChecked(uStr2Bool(value.c_str()));
			_ui->checkBox_useOdomFeatures->blockSignals(false);
		}
		else if(radio)
		{
			radio->setChecked(uStr2Bool(value.c_str()));
		}
		else if(lineEdit)
		{
			lineEdit->setText(value.c_str());
		}
		else if(groupBox)
		{
			groupBox->setChecked(uStr2Bool(value.c_str()));
		}
		else
		{
			ULOGGER_WARN("QObject called %s can't be cast to a supported widget", key.c_str());
		}
	}
	else
	{
		ULOGGER_WARN("Can't find the related QObject for parameter %s", key.c_str());
	}
}

void PreferencesDialog::addParameter(int value)
{
	if(sender())
	{
		this->addParameter(sender(), value);
	}
	else
	{
		ULOGGER_ERROR("This slot must be triggered by a signal, not a direct call...");
	}
}

void PreferencesDialog::addParameter(bool value)
{
	if(sender())
	{
		this->addParameter(sender(), value);
	}
	else
	{
		ULOGGER_ERROR("This slot must be triggered by a signal, not a direct call...");
	}
}

void PreferencesDialog::addParameter(double value)
{
	if(sender())
	{
		this->addParameter(sender(), value);
	}
	else
	{
		ULOGGER_ERROR("This slot must be triggered by a signal, not a direct call...");
	}
}

void PreferencesDialog::addParameter(const QString & value)
{
	if(sender())
	{
		this->addParameter(sender(), value);
	}
	else
	{
		ULOGGER_ERROR("This slot must be triggered by a signal, not a direct call...");
	}
}

void PreferencesDialog::addParameter(const QObject * object, int value)
{
	if(object)
	{
		const QComboBox * comboBox = qobject_cast<const QComboBox*>(object);
		const QSpinBox * spinbox = qobject_cast<const QSpinBox*>(object);
		if(comboBox || spinbox)
		{
			// Add parameter
			UDEBUG("modify param %s=%s", object->objectName().toStdString().c_str(), uNumber2Str(value).c_str());
			uInsert(_modifiedParameters, rtabmap::ParametersPair(object->objectName().toStdString(), QString::number(value).toStdString()));
		}
		else
		{
			UWARN("Undefined object \"%s\"", object->objectName().toStdString().c_str());
		}

	}
	else
	{
		ULOGGER_ERROR("Object is null");
	}
}

void PreferencesDialog::addParameter(const QObject * object, bool value)
{
	if(object)
	{
		const QCheckBox * checkbox = qobject_cast<const QCheckBox*>(object);
		const QRadioButton * radio = qobject_cast<const QRadioButton*>(object);
		const QGroupBox * groupBox = qobject_cast<const QGroupBox*>(object);
		if(checkbox || radio || groupBox)
		{
			// Add parameter
			UDEBUG("modify param %s=%s", object->objectName().toStdString().c_str(), uBool2Str(value).c_str());
			uInsert(_modifiedParameters, rtabmap::ParametersPair(object->objectName().toStdString(), uBool2Str(value)));
		}
		else
		{
			UWARN("Undefined object \"%s\"", object->objectName().toStdString().c_str());
		}

	}
	else
	{
		ULOGGER_ERROR("Object is null");
	}
}

void PreferencesDialog::addParameter(const QObject * object, double value)
{
	if(object)
	{
		UDEBUG("modify param %s=%f", object->objectName().toStdString().c_str(), value);
		uInsert(_modifiedParameters, rtabmap::ParametersPair(object->objectName().toStdString(), QString::number(value).toStdString()));
	}
	else
	{
		ULOGGER_ERROR("Object is null");
	}
}

void PreferencesDialog::addParameter(const QObject * object, const QString & value)
{
	if(object)
	{
		UDEBUG("modify param %s=%s", object->objectName().toStdString().c_str(), value.toStdString().c_str());
		uInsert(_modifiedParameters, rtabmap::ParametersPair(object->objectName().toStdString(), value.toStdString()));
	}
	else
	{
		ULOGGER_ERROR("Object is null");
	}
}

void PreferencesDialog::addParameters(const QObjectList & children)
{
	//ULOGGER_DEBUG("PreferencesDialog::addParameters(QGroupBox) box %s has %d children", box->objectName().toStdString().c_str(), children.size());
	for(int i=0; i<children.size(); ++i)
	{
		const QSpinBox * spin = qobject_cast<const QSpinBox *>(children[i]);
		const QDoubleSpinBox * doubleSpin = qobject_cast<const QDoubleSpinBox *>(children[i]);
		const QComboBox * combo = qobject_cast<const QComboBox *>(children[i]);
		const QCheckBox * check = qobject_cast<const QCheckBox *>(children[i]);
		const QRadioButton * radio = qobject_cast<const QRadioButton *>(children[i]);
		const QLineEdit * lineEdit = qobject_cast<const QLineEdit *>(children[i]);
		const QGroupBox * groupBox = qobject_cast<const QGroupBox *>(children[i]);
		const QStackedWidget * stackedWidget = qobject_cast<const QStackedWidget *>(children[i]);
		if(spin)
		{
			this->addParameter(spin, spin->value());
		}
		else if(doubleSpin)
		{
			this->addParameter(doubleSpin, doubleSpin->value());
		}
		else if(combo)
		{
			this->addParameter(combo, combo->currentIndex());
		}
		else if(check)
		{
			this->addParameter(check, check->isChecked());
		}
		else if(radio)
		{
			this->addParameter(radio, radio->isChecked());
		}
		else if(lineEdit)
		{
			this->addParameter(lineEdit, lineEdit->text());
		}
		else if(groupBox)
		{
			if(groupBox->isCheckable())
			{
				this->addParameter(groupBox, groupBox->isChecked());
			}
			else
			{
				this->addParameters(groupBox);
			}
		}
		else if(stackedWidget)
		{
			this->addParameters(stackedWidget);
		}
	}
}

void PreferencesDialog::addParameters(const QStackedWidget * stackedWidget, int panel)
{
	if(stackedWidget)
	{
		if(panel == -1)
		{
			for(int i=0; i<stackedWidget->count(); ++i)
			{
				const QObjectList & children = stackedWidget->widget(i)->children();
				addParameters(children);
			}
		}
		else
		{
			UASSERT(panel<stackedWidget->count());
			const QObjectList & children = stackedWidget->widget(panel)->children();
			addParameters(children);
		}
	}
}

void PreferencesDialog::addParameters(const QGroupBox * box)
{
	if(box)
	{
		const QObjectList & children = box->children();
		addParameters(children);
	}
}

void PreferencesDialog::updateBasicParameter()
{
	// This method is used to update basic/advanced referred parameters, see above editingFinished()

	// basic to advanced (advanced to basic must be done by connecting signal valueChanged())
	if(sender() == _ui->general_doubleSpinBox_timeThr_2)
	{
		_ui->general_doubleSpinBox_timeThr->setValue(_ui->general_doubleSpinBox_timeThr_2->value());
	}
	else if(sender() == _ui->general_doubleSpinBox_hardThr_2)
	{
		_ui->general_doubleSpinBox_hardThr->setValue(_ui->general_doubleSpinBox_hardThr_2->value());
	}
	else if(sender() == _ui->general_doubleSpinBox_detectionRate_2)
	{
		_ui->general_doubleSpinBox_detectionRate->setValue(_ui->general_doubleSpinBox_detectionRate_2->value());
	}
	else if(sender() == _ui->general_spinBox_imagesBufferSize_2)
	{
		_ui->general_spinBox_imagesBufferSize->setValue(_ui->general_spinBox_imagesBufferSize_2->value());
	}
	else if(sender() == _ui->general_spinBox_maxStMemSize_2)
	{
		_ui->general_spinBox_maxStMemSize->setValue(_ui->general_spinBox_maxStMemSize_2->value());
	}
	else if(sender() == _ui->groupBox_publishing)
	{
		_ui->general_checkBox_publishStats_2->setChecked(_ui->groupBox_publishing->isChecked());
	}
	else if(sender() == _ui->general_checkBox_publishStats_2)
	{
		_ui->groupBox_publishing->setChecked(_ui->general_checkBox_publishStats_2->isChecked());
	}
	else if(sender() == _ui->doubleSpinBox_similarityThreshold_2)
	{
		_ui->doubleSpinBox_similarityThreshold->setValue(_ui->doubleSpinBox_similarityThreshold_2->value());
	}
	else if(sender() == _ui->general_checkBox_activateRGBD)
	{
		_ui->general_checkBox_activateRGBD_2->setChecked(_ui->general_checkBox_activateRGBD->isChecked());
	}
	else if(sender() == _ui->general_checkBox_activateRGBD_2)
	{
		_ui->general_checkBox_activateRGBD->setChecked(_ui->general_checkBox_activateRGBD_2->isChecked());
	}
	else if(sender() == _ui->general_checkBox_SLAM_mode)
	{
		_ui->general_checkBox_SLAM_mode_2->setChecked(_ui->general_checkBox_SLAM_mode->isChecked());
	}
	else if(sender() == _ui->general_checkBox_SLAM_mode_2)
	{
		_ui->general_checkBox_SLAM_mode->setChecked(_ui->general_checkBox_SLAM_mode_2->isChecked());
	}
	else
	{
		//update all values (only those using editingFinished signal)
		_ui->general_doubleSpinBox_timeThr->setValue(_ui->general_doubleSpinBox_timeThr_2->value());
		_ui->general_doubleSpinBox_hardThr->setValue(_ui->general_doubleSpinBox_hardThr_2->value());
		_ui->general_doubleSpinBox_detectionRate->setValue(_ui->general_doubleSpinBox_detectionRate_2->value());
		_ui->general_spinBox_imagesBufferSize->setValue(_ui->general_spinBox_imagesBufferSize_2->value());
		_ui->general_spinBox_maxStMemSize->setValue(_ui->general_spinBox_maxStMemSize_2->value());
		_ui->doubleSpinBox_similarityThreshold->setValue(_ui->doubleSpinBox_similarityThreshold_2->value());
	}
}

void PreferencesDialog::makeObsoleteGeneralPanel()
{
	ULOGGER_DEBUG("");
	_obsoletePanels = _obsoletePanels | kPanelGeneral;
}

void PreferencesDialog::makeObsoleteCloudRenderingPanel()
{
	ULOGGER_DEBUG("");
	_obsoletePanels = _obsoletePanels | kPanelCloudRendering;
}

void PreferencesDialog::makeObsoleteLoggingPanel()
{
	ULOGGER_DEBUG("");
	_obsoletePanels = _obsoletePanels | kPanelLogging;
}

void PreferencesDialog::makeObsoleteSourcePanel()
{
	_obsoletePanels = _obsoletePanels | kPanelSource;
}

QList<QGroupBox*> PreferencesDialog::getGroupBoxes()
{
	QList<QGroupBox*> boxes;
	for(int i=0; i<_ui->stackedWidget->count(); ++i)
	{
		QGroupBox * gb = 0;
		const QObjectList & children = _ui->stackedWidget->widget(i)->children();
		for(int j=0; j<children.size(); ++j)
		{
			if((gb = qobject_cast<QGroupBox *>(children.at(j))))
			{
				//ULOGGER_DEBUG("PreferencesDialog::setupTreeView() index(%d) Added %s, box name=%s", i, gb->title().toStdString().c_str(), gb->objectName().toStdString().c_str());
				break;
			}
		}
		if(gb)
		{
			boxes.append(gb);
		}
		else
		{
			ULOGGER_ERROR("A QGroupBox must be included in the first level of children in stacked widget, index=%d", i);
		}
	}
	return boxes;
}

void PreferencesDialog::updatePredictionPlot()
{
	QStringList values = _ui->lineEdit_bayes_predictionLC->text().simplified().split(' ');
	if(values.size() < 2)
	{
		UERROR("Error parsing prediction (must have at least 2 items) : %s",
				_ui->lineEdit_bayes_predictionLC->text().toStdString().c_str());
		return;
	}
	QVector<float> dataX((values.size()-2)*2 + 1);
	QVector<float> dataY((values.size()-2)*2 + 1);
	double value;
	double sum = 0;
	int lvl = 1;
	bool ok = false;
	bool error = false;
	int loopClosureIndex = (dataX.size()-1)/2;
	for(int i=0; i<values.size(); ++i)
	{
		value = values.at(i).toDouble(&ok);
		if(!ok)
		{
			UERROR("Error parsing prediction : \"%s\"", values.at(i).toStdString().c_str());
			error = true;
		}
		sum+=value;
		if(i==0)
		{
			//do nothing
		}
		else if(i == 1)
		{
			dataY[loopClosureIndex] = value;
			dataX[loopClosureIndex] = 0;
		}
		else
		{
			dataY[loopClosureIndex-lvl] = value;
			dataX[loopClosureIndex-lvl] = -lvl;
			dataY[loopClosureIndex+lvl] = value;
			dataX[loopClosureIndex+lvl] = lvl;
			++lvl;
		}
	}

	_ui->label_prediction_sum->setNum(sum);
	if(error)
	{
		_ui->label_prediction_sum->setText(QString("<font color=#FF0000>") + _ui->label_prediction_sum->text() + "</font>");
	}
	else if(sum == 1.0)
	{
		_ui->label_prediction_sum->setText(QString("<font color=#00FF00>") + _ui->label_prediction_sum->text() + "</font>");
	}
	else if(sum > 1.0)
	{
		_ui->label_prediction_sum->setText(QString("<font color=#FFa500>") + _ui->label_prediction_sum->text() + "</font>");
	}
	else
	{
		_ui->label_prediction_sum->setText(QString("<font color=#000000>") + _ui->label_prediction_sum->text() + "</font>");
	}

	_ui->predictionPlot->removeCurves();
	_ui->predictionPlot->addCurve(new UPlotCurve("Prediction", dataX, dataY, _ui->predictionPlot));
}

void PreferencesDialog::setupKpRoiPanel()
{
	QStringList strings = _ui->lineEdit_kp_roi->text().split(' ');
	if(strings.size()!=4)
	{
		UERROR("ROI must have 4 values (%s)", _ui->lineEdit_kp_roi->text().toStdString().c_str());
		return;
	}
	_ui->doubleSpinBox_kp_roi0->setValue(strings[0].toDouble()*100.0);
	_ui->doubleSpinBox_kp_roi1->setValue(strings[1].toDouble()*100.0);
	_ui->doubleSpinBox_kp_roi2->setValue(strings[2].toDouble()*100.0);
	_ui->doubleSpinBox_kp_roi3->setValue(strings[3].toDouble()*100.0);
}

void PreferencesDialog::updateKpROI()
{
	QStringList strings;
	strings.append(QString::number(_ui->doubleSpinBox_kp_roi0->value()/100.0));
	strings.append(QString::number(_ui->doubleSpinBox_kp_roi1->value()/100.0));
	strings.append(QString::number(_ui->doubleSpinBox_kp_roi2->value()/100.0));
	strings.append(QString::number(_ui->doubleSpinBox_kp_roi3->value()/100.0));
	_ui->lineEdit_kp_roi->setText(strings.join(" "));
}

void PreferencesDialog::updateStereoDisparityVisibility()
{
	Src driver = this->getSourceDriver();
	_ui->checkbox_stereo_depthGenerated->setVisible(
		driver != PreferencesDialog::kSrcStereoZed ||
		_ui->comboBox_stereoZed_quality->currentIndex() == 0);
	_ui->label_stereo_depthGenerated->setVisible(
		driver != PreferencesDialog::kSrcStereoZed ||
		_ui->comboBox_stereoZed_quality->currentIndex() == 0);
}

void PreferencesDialog::useOdomFeatures()
{
	if(this->isVisible() && _ui->checkBox_useOdomFeatures->isChecked())
	{
		int r = QMessageBox::question(this, tr("Using odometry features for vocabulary..."),
				tr("Do you want to match vocabulary feature parameters "
				   "with corresponding ones used for odometry?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

		if(r == QMessageBox::Yes)
		{
			_ui->comboBox_detector_strategy->setCurrentIndex(_ui->reextract_type->currentIndex());
			_ui->surf_doubleSpinBox_maxDepth->setValue(_ui->loopClosure_bowMaxDepth->value());
			_ui->surf_doubleSpinBox_minDepth->setValue(_ui->loopClosure_bowMinDepth->value());
			_ui->surf_spinBox_wordsPerImageTarget->setValue(_ui->reextract_maxFeatures->value());
			_ui->lineEdit_kp_roi->setText(_ui->loopClosure_roi->text());
			_ui->subpix_winSize_kp->setValue(_ui->subpix_winSize->value());
			_ui->subpix_iterations_kp->setValue(_ui->subpix_iterations->value());
			_ui->subpix_eps_kp->setValue(_ui->subpix_eps->value());
		}
	}
}


void PreferencesDialog::useGridProjRayTracing()
{
	if(this->isVisible() && _ui->checkBox_grid_projRayTracing->isChecked() && _ui->groupBox_grid_3d->isChecked())
	{
		int r = QMessageBox::question(this, tr("Using ray tracing for 2D projection..."),
				tr("Currently the 3D occupancy grid parameter is checked, but 2D ray tracing "
				  "only works with 2D occupancy grids. Do you want to uncheck 3D occupancy grid?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

		if(r == QMessageBox::Yes)
		{
			_ui->groupBox_grid_3d->setChecked(false);
		}
	}
}

void PreferencesDialog::changeWorkingDirectory()
{
	QString directory = QFileDialog::getExistingDirectory(this, tr("Working directory"), _ui->lineEdit_workingDirectory->text());
	if(!directory.isEmpty())
	{
		ULOGGER_DEBUG("New working directory = %s", directory.toStdString().c_str());
		_ui->lineEdit_workingDirectory->setText(directory);
	}
}

void PreferencesDialog::changeDictionaryPath()
{
	QString path;
	if(_ui->lineEdit_dictionaryPath->text().isEmpty())
	{
		path = QFileDialog::getOpenFileName(this, tr("Dictionary"), this->getWorkingDirectory());
	}
	else
	{
		path = QFileDialog::getOpenFileName(this, tr("Dictionary"), _ui->lineEdit_dictionaryPath->text());
	}
	if(!path.isEmpty())
	{
		_ui->lineEdit_dictionaryPath->setText(path);
	}
}

void PreferencesDialog::updateSourceGrpVisibility()
{
	_ui->groupBox_sourceRGBD->setVisible(_ui->comboBox_sourceType->currentIndex() == 0);
	_ui->groupBox_sourceStereo->setVisible(_ui->comboBox_sourceType->currentIndex() == 1);
	_ui->groupBox_sourceRGB->setVisible(_ui->comboBox_sourceType->currentIndex() == 2);
	_ui->groupBox_sourceDatabase->setVisible(_ui->comboBox_sourceType->currentIndex() == 3);

	_ui->groupBox_scanFromDepth->setVisible(_ui->comboBox_sourceType->currentIndex() <= 1 || _ui->comboBox_sourceType->currentIndex() == 3);

	_ui->stackedWidget_rgbd->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 &&
			(_ui->comboBox_cameraRGBD->currentIndex() == kSrcOpenNI2-kSrcRGBD ||
			 _ui->comboBox_cameraRGBD->currentIndex() == kSrcFreenect2-kSrcRGBD ||
			 _ui->comboBox_cameraRGBD->currentIndex() == kSrcRealSense - kSrcRGBD ||
			 _ui->comboBox_cameraRGBD->currentIndex() == kSrcRGBDImages-kSrcRGBD ||
			 _ui->comboBox_cameraRGBD->currentIndex() == kSrcOpenNI_PCL-kSrcRGBD));
	_ui->groupBox_openni2->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcOpenNI2-kSrcRGBD);
	_ui->groupBox_freenect2->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcFreenect2-kSrcRGBD);
	_ui->groupBox_realsense->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcRealSense - kSrcRGBD);
	_ui->groupBox_cameraRGBDImages->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcRGBDImages-kSrcRGBD);
	_ui->groupBox_openni->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcOpenNI_PCL-kSrcRGBD);

	_ui->stackedWidget_stereo->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && 
		(_ui->comboBox_cameraStereo->currentIndex() == kSrcStereoVideo-kSrcStereo || 
		 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoImages-kSrcStereo ||
		 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoZed - kSrcStereo));
	_ui->groupBox_cameraStereoImages->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoImages-kSrcStereo);
	_ui->groupBox_cameraStereoVideo->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoVideo - kSrcStereo);
	_ui->groupBox_cameraStereoZed->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoZed - kSrcStereo);

	_ui->stackedWidget_image->setVisible(_ui->comboBox_sourceType->currentIndex() == 2 && (_ui->source_comboBox_image_type->currentIndex() == kSrcImages-kSrcRGB || _ui->source_comboBox_image_type->currentIndex() == kSrcVideo-kSrcRGB));
	_ui->source_groupBox_images->setVisible(_ui->comboBox_sourceType->currentIndex() == 2 && _ui->source_comboBox_image_type->currentIndex() == kSrcImages-kSrcRGB);
	_ui->source_groupBox_video->setVisible(_ui->comboBox_sourceType->currentIndex() == 2 && _ui->source_comboBox_image_type->currentIndex() == kSrcVideo-kSrcRGB);

	_ui->groupBox_sourceImages_optional->setVisible(
			(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcRGBDImages-kSrcRGBD) ||
			(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoImages-kSrcStereo) ||
			(_ui->comboBox_sourceType->currentIndex() == 2 && _ui->source_comboBox_image_type->currentIndex() == kSrcImages-kSrcRGB));

	_ui->groupBox_depthImageFiltering->setEnabled(
				_ui->comboBox_sourceType->currentIndex() == 0 || // RGBD
				_ui->comboBox_sourceType->currentIndex() == 3);  // Database
	_ui->groupBox_depthImageFiltering->setVisible(_ui->groupBox_depthImageFiltering->isEnabled());

	//_ui->groupBox_scan->setVisible(_ui->comboBox_sourceType->currentIndex() != 3);

	_ui->groupBox_depthFromScan->setVisible(_ui->comboBox_sourceType->currentIndex() == 2 && _ui->source_comboBox_image_type->currentIndex() == kSrcImages-kSrcRGB);
}

/*** GETTERS ***/
//General
int PreferencesDialog::getGeneralLoggerLevel() const
{
	return _ui->comboBox_loggerLevel->currentIndex();
}
int PreferencesDialog::getGeneralLoggerEventLevel() const
{
	return _ui->comboBox_loggerEventLevel->currentIndex();
}
int PreferencesDialog::getGeneralLoggerPauseLevel() const
{
	return _ui->comboBox_loggerPauseLevel->currentIndex();
}
int PreferencesDialog::getGeneralLoggerType() const
{
	return _ui->comboBox_loggerType->currentIndex();
}
bool PreferencesDialog::getGeneralLoggerPrintTime() const
{
	return _ui->checkBox_logger_printTime->isChecked();
}
bool PreferencesDialog::getGeneralLoggerPrintThreadId() const
{
	return _ui->checkBox_logger_printThreadId->isChecked();
}
std::vector<std::string> PreferencesDialog::getGeneralLoggerThreads() const
{
	std::vector<std::string> threads;
	for(int i=0; i<_ui->comboBox_loggerFilter->count(); ++i)
	{
		if(_ui->comboBox_loggerFilter->itemData(i).toBool())
		{
			threads.push_back(_ui->comboBox_loggerFilter->itemText(i).toStdString());
		}
	}
	return threads;
}
bool PreferencesDialog::isVerticalLayoutUsed() const
{
	return _ui->checkBox_verticalLayoutUsed->isChecked();
}
bool PreferencesDialog::imageRejectedShown() const
{
	return _ui->checkBox_imageRejectedShown->isChecked();
}
bool PreferencesDialog::imageHighestHypShown() const
{
	return _ui->checkBox_imageHighestHypShown->isChecked();
}
bool PreferencesDialog::beepOnPause() const
{
	return _ui->checkBox_beep->isChecked();
}
bool PreferencesDialog::isTimeUsedInFigures() const
{
	return _ui->checkBox_stamps->isChecked();
}
bool PreferencesDialog::isCacheSavedInFigures() const
{
	return _ui->checkBox_cacheStatistics->isChecked();
}
bool PreferencesDialog::notifyWhenNewGlobalPathIsReceived() const
{
	return _ui->checkBox_notifyWhenNewGlobalPathIsReceived->isChecked();
}
int PreferencesDialog::getOdomQualityWarnThr() const
{
	return _ui->spinBox_odomQualityWarnThr->value();
}
bool PreferencesDialog::isPosteriorGraphView() const
{
	return _ui->checkBox_posteriorGraphView->isChecked();
}
bool PreferencesDialog::isOdomDisabled() const
{
	return _ui->checkbox_odomDisabled->isChecked();
}
int PreferencesDialog::getOdomRegistrationApproach() const
{
	return _ui->odom_registration->currentIndex();
}
bool PreferencesDialog::isGroundTruthAligned() const
{
	return _ui->checkbox_groundTruthAlign->isChecked();
}

bool PreferencesDialog::isCloudsShown(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingShowClouds[index]->isChecked();
}
bool PreferencesDialog::isOctomapUpdated() const
{
#ifdef RTABMAP_OCTOMAP
	return _ui->groupBox_octomap->isChecked();
#endif
	return false;
}
bool PreferencesDialog::isOctomapShown() const
{
#ifdef RTABMAP_OCTOMAP
	return _ui->groupBox_octomap->isChecked() && _ui->checkBox_octomap_show3dMap->isChecked();
#endif
	return false;
}
bool PreferencesDialog::isOctomapCubeRendering() const
{
	return _ui->checkBox_octomap_cubeRendering->isChecked();
}
bool PreferencesDialog::isOctomap2dGrid() const
{
#ifdef RTABMAP_OCTOMAP
	return _ui->groupBox_octomap->isChecked() && _ui->checkBox_octomap_2dgrid->isChecked();
#endif
	return false;
}
int PreferencesDialog::getOctomapTreeDepth() const
{
	return _ui->spinBox_octomap_treeDepth->value();
}
bool PreferencesDialog::isOctomapGroundAnObstacle() const
{
	return _ui->checkBox_grid_groundObstacle->isChecked();
}
double PreferencesDialog::getOctomapOccupancyThr() const
{
	return _ui->doubleSpinBox_octomap_occupancyThr->value();
}

double PreferencesDialog::getVoxel() const
{
	return _ui->doubleSpinBox_voxel->value();
}
double PreferencesDialog::getNoiseRadius() const
{
	return _ui->doubleSpinBox_noiseRadius->value();
}
int PreferencesDialog::getNoiseMinNeighbors() const
{
	return _ui->spinBox_noiseMinNeighbors->value();
}
double PreferencesDialog::getCeilingFilteringHeight() const
{
	return _ui->doubleSpinBox_ceilingFilterHeight->value();
}
double PreferencesDialog::getFloorFilteringHeight() const
{
	return _ui->doubleSpinBox_floorFilterHeight->value();
}
int PreferencesDialog::getNormalKSearch() const
{
	return _ui->spinBox_normalKSearch->value();
}

bool PreferencesDialog::isGraphsShown() const
{
	return _ui->checkBox_showGraphs->isChecked();
}
bool PreferencesDialog::isFrustumsShown() const
{
	return _ui->checkBox_showFrustums->isEnabled() && _ui->checkBox_showFrustums->isChecked();
}
bool PreferencesDialog::isLabelsShown() const
{
	return _ui->checkBox_showLabels->isChecked();
}
bool PreferencesDialog::isCloudMeshing() const
{
	return _ui->groupBox_organized->isChecked();
}
double PreferencesDialog::getCloudMeshingAngle() const
{
	return _ui->doubleSpinBox_mesh_angleTolerance->value()*M_PI/180.0;
}
bool PreferencesDialog::isCloudMeshingQuad() const
{
	return _ui->checkBox_mesh_quad->isChecked();
}
bool PreferencesDialog::isCloudMeshingTexture() const
{
	return _ui->checkBox_mesh_texture->isChecked();
}
int PreferencesDialog::getCloudMeshingTriangleSize()
{
	return _ui->spinBox_mesh_triangleSize->value();
}
int PreferencesDialog::getCloudDecimation(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingDecimation[index]->value()==0?1:_3dRenderingDecimation[index]->value();
}
double PreferencesDialog::getCloudMaxDepth(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingMaxDepth[index]->value();
}
double PreferencesDialog::getCloudMinDepth(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingMinDepth[index]->value();
}
std::vector<float> PreferencesDialog::getCloudRoiRatios(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	std::vector<float> roiRatios;
	if(!_3dRenderingRoiRatios[index]->text().isEmpty())
	{
		QStringList values = _3dRenderingRoiRatios[index]->text().split(' ');
		if(values.size() == 4)
		{
			roiRatios.resize(4);
			for(int i=0; i<values.size(); ++i)
			{
				roiRatios[i] = uStr2Float(values[i].toStdString().c_str());
			}
		}
	}
	return roiRatios;
}
double PreferencesDialog::getCloudOpacity(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingOpacity[index]->value();
}
int PreferencesDialog::getCloudPointSize(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingPtSize[index]->value();
}

bool PreferencesDialog::isScansShown(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingShowScans[index]->isChecked();
}
int PreferencesDialog::getDownsamplingStepScan(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingDownsamplingScan[index]->value();
}
double PreferencesDialog::getCloudVoxelSizeScan(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingVoxelSizeScan[index]->value();
}
double PreferencesDialog::getScanOpacity(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingOpacityScan[index]->value();
}
int PreferencesDialog::getScanPointSize(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingPtSizeScan[index]->value();
}

bool PreferencesDialog::isFeaturesShown(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingShowFeatures[index]->isChecked();
}
int PreferencesDialog::getFeaturesPointSize(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingPtSizeFeatures[index]->value();
}

bool PreferencesDialog::isCloudFiltering() const
{
	return _ui->radioButton_nodeFiltering->isChecked();
}
bool PreferencesDialog::isSubtractFiltering() const
{
	return _ui->radioButton_subtractFiltering->isChecked();
}
double PreferencesDialog::getCloudFilteringRadius() const
{
	return _ui->doubleSpinBox_cloudFilterRadius->value();
}
double PreferencesDialog::getCloudFilteringAngle() const
{
	return _ui->doubleSpinBox_cloudFilterAngle->value();
}
int PreferencesDialog::getSubtractFilteringMinPts() const
{
	return _ui->spinBox_subtractFilteringMinPts->value();
}
double PreferencesDialog::getSubtractFilteringRadius() const
{
	return _ui->doubleSpinBox_subtractFilteringRadius->value();
}
double PreferencesDialog::getSubtractFilteringAngle() const
{
	return _ui->doubleSpinBox_subtractFilteringAngle->value()*M_PI/180.0;
}
bool PreferencesDialog::getGridMapShown() const
{
	return _ui->checkBox_map_shown->isChecked();
}
double PreferencesDialog::getGridMapResolution() const
{
	return _ui->doubleSpinBox_map_resolution->value();
}
bool PreferencesDialog::isGridMapEroded() const
{
	return _ui->checkBox_map_erode->isChecked();
}
bool PreferencesDialog::isGridMapIncremental() const
{
	return _ui->checkBox_map_incremental->isChecked();
}
double PreferencesDialog::getGridMapFootprintRadius() const
{
	return _ui->doubleSpinBox_map_footprintRadius->value();
}
bool PreferencesDialog::isGridMapFrom3DCloud() const
{
	return _ui->groupBox_grid_fromDepthImage->isChecked();
}
bool PreferencesDialog::projMapFrame() const
{
	return _ui->checkBox_grid_projMapFrame->isChecked();
}
double PreferencesDialog::projMaxGroundAngle() const
{
	return _ui->doubleSpinBox_grid_maxGroundAngle->value()*M_PI/180.0;
}
double PreferencesDialog::projMaxGroundHeight() const
{
	return _ui->doubleSpinBox_grid_maxGroundHeight->value();
}
int PreferencesDialog::projMinClusterSize() const
{
	return _ui->spinBox_grid_minClusterSize->value();
}
double PreferencesDialog::projMaxObstaclesHeight() const
{
	return _ui->doubleSpinBox_grid_maxObstacleHeight->value();
}
bool PreferencesDialog::projFlatObstaclesDetected() const
{
	return _ui->checkBox_grid_flatObstaclesDetected->isChecked();
}
double PreferencesDialog::getGridMapOpacity() const
{
	return _ui->doubleSpinBox_map_opacity->value();
}


// Source
double PreferencesDialog::getGeneralInputRate() const
{
	return _ui->general_doubleSpinBox_imgRate->value();
}
bool PreferencesDialog::isSourceMirroring() const
{
	return _ui->source_mirroring->isChecked();
}
PreferencesDialog::Src PreferencesDialog::getSourceType() const
{
	int index = _ui->comboBox_sourceType->currentIndex();
	if(index == 0)
	{
		return kSrcRGBD;
	}
	else if(index == 1)
	{
		return kSrcStereo;
	}
	else if(index == 2)
	{
		return kSrcRGB;
	}
	else if(index == 3)
	{
		return kSrcDatabase;
	}
	return kSrcUndef;
}
PreferencesDialog::Src PreferencesDialog::getSourceDriver() const
{
	PreferencesDialog::Src type = getSourceType();
	if(type==kSrcRGBD)
	{
		return (PreferencesDialog::Src)(_ui->comboBox_cameraRGBD->currentIndex()+kSrcRGBD);
	}
	else if(type==kSrcStereo)
	{
		return (PreferencesDialog::Src)(_ui->comboBox_cameraStereo->currentIndex()+kSrcStereo);
	}
	else if(type==kSrcRGB)
	{
		return (PreferencesDialog::Src)(_ui->source_comboBox_image_type->currentIndex()+kSrcRGB);
	}
	else if(type==kSrcDatabase)
	{
		return kSrcDatabase;
	}
	return kSrcUndef;
}
QString PreferencesDialog::getSourceDriverStr() const
{
	PreferencesDialog::Src type = getSourceType();
	if(type==kSrcRGBD)
	{
		return _ui->comboBox_cameraRGBD->currentText();
	}
	else if(type==kSrcStereo)
	{
		return _ui->comboBox_cameraStereo->currentText();
	}
	else if(type==kSrcRGB)
	{
		return _ui->source_comboBox_image_type->currentText();
	}
	else if(type==kSrcDatabase)
	{
		return "Database";
	}
	return "";
}

QString PreferencesDialog::getSourceDevice() const
{
	return _ui->lineEdit_sourceDevice->text();
}

Transform PreferencesDialog::getSourceLocalTransform() const
{
	Transform t = Transform::fromString(_ui->lineEdit_sourceLocalTransform->text().replace("PI_2", QString::number(3.141592/2.0)).toStdString());
	if(t.isNull())
	{
		return Transform::getIdentity();
	}
	return t;
}

Transform PreferencesDialog::getLaserLocalTransform() const
{
	Transform t = Transform::fromString(_ui->lineEdit_cameraImages_laser_transform->text().replace("PI_2", QString::number(3.141592/2.0)).toStdString());
	if(t.isNull())
	{
		return Transform::getIdentity();
	}
	return t;
}

bool PreferencesDialog::isSourceDatabaseStampsUsed() const
{
	return _ui->source_checkBox_useDbStamps->isChecked();
}
bool PreferencesDialog::isSourceRGBDColorOnly() const
{
	return _ui->checkbox_rgbd_colorOnly->isChecked();
}
bool PreferencesDialog::isDepthFilteringAvailable() const
{
	return _ui->groupBox_depthImageFiltering->isEnabled();
}
QString PreferencesDialog::getSourceDistortionModel() const
{
	return _ui->lineEdit_source_distortionModel->text();
}
bool PreferencesDialog::isBilateralFiltering() const
{
	return _ui->groupBox_bilateral->isChecked();
}
double PreferencesDialog::getBilateralSigmaS() const
{
	return _ui->doubleSpinBox_bilateral_sigmaS->value();
}
double PreferencesDialog::getBilateralSigmaR() const
{
	return _ui->doubleSpinBox_bilateral_sigmaR->value();
}
int PreferencesDialog::getSourceImageDecimation() const
{
	return _ui->spinBox_source_imageDecimation->value();
}
bool PreferencesDialog::isSourceStereoDepthGenerated() const
{
	return _ui->checkbox_stereo_depthGenerated->isChecked();
}
bool PreferencesDialog::isSourceScanFromDepth() const
{
	return _ui->groupBox_scanFromDepth->isChecked();
}
int PreferencesDialog::getSourceScanFromDepthDecimation() const
{
	return _ui->spinBox_cameraScanFromDepth_decimation->value();
}
double PreferencesDialog::getSourceScanFromDepthMaxDepth() const
{
	return _ui->doubleSpinBox_cameraSCanFromDepth_maxDepth->value();
}
double PreferencesDialog::getSourceScanVoxelSize() const
{
	return _ui->doubleSpinBox_cameraImages_scanVoxelSize->value();
}
int PreferencesDialog::getSourceScanNormalsK() const
{
	return _ui->spinBox_cameraImages_scanNormalsK->value();
}

Camera * PreferencesDialog::createCamera(bool useRawImages, bool useColor)
{
	UDEBUG("");
	Src driver = this->getSourceDriver();
	Camera * camera = 0;
	if(driver == PreferencesDialog::kSrcOpenNI_PCL)
	{
		if(useRawImages)
		{
			QMessageBox::warning(this, tr("Calibration"),
					tr("Using raw images for \"OpenNI\" driver is not yet supported. "
					    "Factory calibration loaded from OpenNI is used."), QMessageBox::Ok);
			return 0;
		}
		else
		{
			camera = new CameraOpenni(
					_ui->lineEdit_openniOniPath->text().isEmpty()?this->getSourceDevice().toStdString():_ui->lineEdit_openniOniPath->text().toStdString(),
					this->getGeneralInputRate(),
					this->getSourceLocalTransform());
		}
	}
	else if(driver == PreferencesDialog::kSrcOpenNI2)
	{
		camera = new CameraOpenNI2(
				_ui->lineEdit_openni2OniPath->text().isEmpty()?this->getSourceDevice().toStdString():_ui->lineEdit_openni2OniPath->text().toStdString(),
				useColor?CameraOpenNI2::kTypeColorDepth:CameraOpenNI2::kTypeIRDepth,
				this->getGeneralInputRate(),
				this->getSourceLocalTransform());
	}
	else if(driver == PreferencesDialog::kSrcFreenect)
	{
		camera = new CameraFreenect(
				this->getSourceDevice().isEmpty()?0:atoi(this->getSourceDevice().toStdString().c_str()),
				useColor?CameraFreenect::kTypeColorDepth:CameraFreenect::kTypeIRDepth,
				this->getGeneralInputRate(),
				this->getSourceLocalTransform());
	}
	else if(driver == PreferencesDialog::kSrcOpenNI_CV ||
			driver == PreferencesDialog::kSrcOpenNI_CV_ASUS)
	{
		if(useRawImages)
		{
			QMessageBox::warning(this, tr("Calibration"),
					tr("Using raw images for \"OpenNI\" driver is not yet supported. "
						"Factory calibration loaded from OpenNI is used."), QMessageBox::Ok);
			return 0;
		}
		else
		{
			camera = new CameraOpenNICV(
					driver == PreferencesDialog::kSrcOpenNI_CV_ASUS,
					this->getGeneralInputRate(),
					this->getSourceLocalTransform());
		}
	}
	else if(driver == kSrcFreenect2)
	{
		camera = new CameraFreenect2(
			this->getSourceDevice().isEmpty()?0:atoi(this->getSourceDevice().toStdString().c_str()),
			useRawImages?CameraFreenect2::kTypeColorIR:(CameraFreenect2::Type)_ui->comboBox_freenect2Format->currentIndex(),
			this->getGeneralInputRate(),
			this->getSourceLocalTransform(),
			_ui->doubleSpinBox_freenect2MinDepth->value(),
			_ui->doubleSpinBox_freenect2MaxDepth->value(),
			_ui->checkBox_freenect2BilateralFiltering->isChecked(),
			_ui->checkBox_freenect2EdgeAwareFiltering->isChecked(),
			_ui->checkBox_freenect2NoiseFiltering->isChecked());
	}
	else if (driver == kSrcRealSense)
	{
		if(useRawImages)
		{
			QMessageBox::warning(this, tr("Calibration"),
					tr("Using raw images for \"RealSense\" driver is not yet supported. "
						"Factory calibration loaded from RealSense is used."), QMessageBox::Ok);
			return 0;
		}
		else
		{
			camera = new CameraRealSense(
				this->getSourceDevice().isEmpty() ? 0 : atoi(this->getSourceDevice().toStdString().c_str()),
				_ui->comboBox_realsensePresetRGB->currentIndex(),
				_ui->comboBox_realsensePresetDepth->currentIndex(),
				this->getGeneralInputRate(),
				this->getSourceLocalTransform());
		}
	}
	else if(driver == kSrcRGBDImages)
	{
		camera = new CameraRGBDImages(
			_ui->lineEdit_cameraRGBDImages_path_rgb->text().append(QDir::separator()).toStdString(),
			_ui->lineEdit_cameraRGBDImages_path_depth->text().append(QDir::separator()).toStdString(),
			_ui->doubleSpinBox_cameraRGBDImages_scale->value(),
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());
		((CameraRGBDImages*)camera)->setBayerMode(_ui->comboBox_cameraImages_bayerMode->currentIndex()-1);
		((CameraRGBDImages*)camera)->setOdometryPath(_ui->lineEdit_cameraImages_odom->text().toStdString(), _ui->comboBox_cameraImages_odomFormat->currentIndex());
		((CameraRGBDImages*)camera)->setGroundTruthPath(_ui->lineEdit_cameraImages_gt->text().toStdString(), _ui->comboBox_cameraImages_gtFormat->currentIndex());
		((CameraRGBDImages*)camera)->setScanPath(
						_ui->lineEdit_cameraImages_path_scans->text().isEmpty()?"":_ui->lineEdit_cameraImages_path_scans->text().append(QDir::separator()).toStdString(),
						_ui->spinBox_cameraImages_max_scan_pts->value(),
						_ui->spinBox_cameraImages_scanDownsampleStep->value(),
						_ui->doubleSpinBox_cameraImages_scanVoxelSize->value(),
						_ui->spinBox_cameraImages_scanNormalsK->value(),
						this->getLaserLocalTransform());
		((CameraRGBDImages*)camera)->setTimestamps(
				_ui->checkBox_cameraImages_timestamps->isChecked(),
				_ui->lineEdit_cameraImages_timestamps->text().toStdString(),
				_ui->checkBox_cameraImages_syncTimeStamps->isChecked());
	}
	else if(driver == kSrcDC1394)
	{
		camera = new CameraStereoDC1394(
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());
	}
	else if(driver == kSrcFlyCapture2)
	{
		if(useRawImages)
		{
			QMessageBox::warning(this, tr("Calibration"),
					tr("Using raw images for \"FlyCapture2\" driver is not yet supported. "
						"Factory calibration loaded from FlyCapture2 is used."), QMessageBox::Ok);
			return 0;
		}
		else
		{
			camera = new CameraStereoFlyCapture2(
				this->getGeneralInputRate(),
				this->getSourceLocalTransform());
		}
	}
	else if(driver == kSrcStereoImages)
	{
		camera = new CameraStereoImages(
			_ui->lineEdit_cameraStereoImages_path_left->text().append(QDir::separator()).toStdString(),
			_ui->lineEdit_cameraStereoImages_path_right->text().append(QDir::separator()).toStdString(),
			_ui->checkBox_stereoImages_rectify->isChecked() && !useRawImages,
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());
		((CameraStereoImages*)camera)->setBayerMode(_ui->comboBox_cameraImages_bayerMode->currentIndex()-1);
		((CameraStereoImages*)camera)->setOdometryPath(_ui->lineEdit_cameraImages_odom->text().toStdString(), _ui->comboBox_cameraImages_odomFormat->currentIndex());
		((CameraStereoImages*)camera)->setGroundTruthPath(_ui->lineEdit_cameraImages_gt->text().toStdString(), _ui->comboBox_cameraImages_gtFormat->currentIndex());
		((CameraStereoImages*)camera)->setScanPath(
						_ui->lineEdit_cameraImages_path_scans->text().isEmpty()?"":_ui->lineEdit_cameraImages_path_scans->text().append(QDir::separator()).toStdString(),
						_ui->spinBox_cameraImages_max_scan_pts->value(),
						_ui->spinBox_cameraImages_scanDownsampleStep->value(),
						_ui->doubleSpinBox_cameraImages_scanVoxelSize->value(),
						_ui->spinBox_cameraImages_scanNormalsK->value(),
						this->getLaserLocalTransform());
		((CameraStereoImages*)camera)->setTimestamps(
				_ui->checkBox_cameraImages_timestamps->isChecked(),
				_ui->lineEdit_cameraImages_timestamps->text().toStdString(),
				_ui->checkBox_cameraImages_syncTimeStamps->isChecked());
	}
	else if (driver == kSrcStereoUsb)
	{
		camera = new CameraStereoVideo(
			this->getSourceDevice().isEmpty() ? 0 : atoi(this->getSourceDevice().toStdString().c_str()),
			!useRawImages,
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());
	}
	else if(driver == kSrcStereoVideo)
	{
		if(!_ui->lineEdit_cameraStereoVideo_path_2->text().isEmpty())
		{
			// left and right videos
			camera = new CameraStereoVideo(
					_ui->lineEdit_cameraStereoVideo_path->text().toStdString(),
					_ui->lineEdit_cameraStereoVideo_path_2->text().toStdString(),
					_ui->checkBox_stereoVideo_rectify->isChecked() && !useRawImages,
					this->getGeneralInputRate(),
					this->getSourceLocalTransform());
		}
		else
		{
			// side-by-side video
			camera = new CameraStereoVideo(
					_ui->lineEdit_cameraStereoVideo_path->text().toStdString(),
					_ui->checkBox_stereoVideo_rectify->isChecked() && !useRawImages,
					this->getGeneralInputRate(),
					this->getSourceLocalTransform());
		}
	}
	else if (driver == kSrcStereoZed)
	{
		UDEBUG("ZED");
		if(!_ui->lineEdit_zedSvoPath->text().isEmpty())
		{
			camera = new CameraStereoZed(
				_ui->lineEdit_zedSvoPath->text().toStdString(),
				_ui->comboBox_stereoZed_quality->currentIndex(),
				_ui->comboBox_stereoZed_sensingMode->currentIndex(),
				_ui->spinBox_stereoZed_confidenceThr->value(),
				_ui->checkbox_stereoZed_odom->isChecked(),
				this->getGeneralInputRate(),
				this->getSourceLocalTransform(),
				_ui->checkbox_stereoZed_selfCalibration->isChecked());
		}
		else
		{
			camera = new CameraStereoZed(
				this->getSourceDevice().isEmpty()?0:atoi(this->getSourceDevice().toStdString().c_str()),
				_ui->comboBox_stereoZed_resolution->currentIndex(),
				_ui->comboBox_stereoZed_quality->currentIndex(),
				_ui->comboBox_stereoZed_sensingMode->currentIndex(),
				_ui->spinBox_stereoZed_confidenceThr->value(),
				_ui->checkbox_stereoZed_odom->isChecked(),
				this->getGeneralInputRate(),
				this->getSourceLocalTransform(),
				_ui->checkbox_stereoZed_selfCalibration->isChecked());
		}
	}
	else if(driver == kSrcUsbDevice)
	{
		camera = new CameraVideo(
			this->getSourceDevice().isEmpty()?0:atoi(this->getSourceDevice().toStdString().c_str()),
			!useRawImages,
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());
	}
	else if(driver == kSrcVideo)
	{
		camera = new CameraVideo(
			_ui->source_video_lineEdit_path->text().toStdString(),
			_ui->checkBox_rgbVideo_rectify->isChecked() && !useRawImages,
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());
	}
	else if(driver == kSrcImages)
	{
		camera = new CameraImages(
			_ui->source_images_lineEdit_path->text().toStdString(),
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());

		((CameraImages*)camera)->setStartIndex(_ui->source_images_spinBox_startPos->value());
		((CameraImages*)camera)->setDirRefreshed(_ui->source_images_refreshDir->isChecked());
		((CameraImages*)camera)->setImagesRectified(_ui->checkBox_rgbImages_rectify->isChecked() && !useRawImages);

		((CameraImages*)camera)->setBayerMode(_ui->comboBox_cameraImages_bayerMode->currentIndex()-1);
		((CameraImages*)camera)->setOdometryPath(
				_ui->lineEdit_cameraImages_odom->text().toStdString(),
				_ui->comboBox_cameraImages_odomFormat->currentIndex());
		((CameraImages*)camera)->setGroundTruthPath(
				_ui->lineEdit_cameraImages_gt->text().toStdString(),
				_ui->comboBox_cameraImages_gtFormat->currentIndex());
		((CameraImages*)camera)->setScanPath(
						_ui->lineEdit_cameraImages_path_scans->text().isEmpty()?"":_ui->lineEdit_cameraImages_path_scans->text().append(QDir::separator()).toStdString(),
						_ui->spinBox_cameraImages_max_scan_pts->value(),
						_ui->spinBox_cameraImages_scanDownsampleStep->value(),
						_ui->doubleSpinBox_cameraImages_scanVoxelSize->value(),
						_ui->spinBox_cameraImages_scanNormalsK->value(),
						this->getLaserLocalTransform());
		((CameraImages*)camera)->setDepthFromScan(
				_ui->groupBox_depthFromScan->isChecked(),
				!_ui->groupBox_depthFromScan_fillHoles->isChecked()?0:_ui->radioButton_depthFromScan_vertical->isChecked()?1:-1,
				_ui->checkBox_depthFromScan_fillBorders->isChecked());
		((CameraImages*)camera)->setTimestamps(
				_ui->checkBox_cameraImages_timestamps->isChecked(),
				_ui->lineEdit_cameraImages_timestamps->text().toStdString(),
				_ui->checkBox_cameraImages_syncTimeStamps->isChecked());
	}
	else if(driver == kSrcDatabase)
	{
		camera = new DBReader(_ui->source_database_lineEdit_path->text().toStdString(),
				_ui->source_checkBox_useDbStamps->isChecked()?-1:this->getGeneralInputRate(),
				_ui->source_checkBox_ignoreOdometry->isChecked(),
				_ui->source_checkBox_ignoreGoalDelay->isChecked(),
				_ui->source_checkBox_ignoreGoals->isChecked(),
				_ui->source_spinBox_databaseStartPos->value(),
				_ui->source_spinBox_database_cameraIndex->value());
	}
	else
	{
		UFATAL("Source driver undefined (%d)!", driver);
	}

	if(camera)
	{
		UDEBUG("Init");
		QString dir = this->getCameraInfoDir();
		QString calibrationFile = _ui->lineEdit_calibrationFile->text();
		if(!(driver >= kSrcRGB && driver <= kSrcVideo))
		{
			calibrationFile.remove("_left.yaml").remove("_right.yaml").remove("_pose.yaml").remove("_rgb.yaml").remove("_depth.yaml");
		}
		QString name = QFileInfo(calibrationFile.remove(".yaml")).baseName();
		if(!_ui->lineEdit_calibrationFile->text().isEmpty())
		{
			QDir d = QFileInfo(_ui->lineEdit_calibrationFile->text()).dir();
			if(!_ui->lineEdit_calibrationFile->text().remove(QFileInfo(_ui->lineEdit_calibrationFile->text()).baseName()).isEmpty())
			{
				dir = d.absolutePath();
			}
		}

		// don't set calibration folder if we want raw images
		if(!camera->init(useRawImages?"":dir.toStdString(), name.toStdString()))
		{
			UWARN("init camera failed... ");
			QMessageBox::warning(this,
					   tr("RTAB-Map"),
					   tr("Camera initialization failed..."));
			delete camera;
			camera = 0;
		}
		else
		{
			//should be after initialization
			if(driver == kSrcOpenNI2)
			{
				((CameraOpenNI2*)camera)->setAutoWhiteBalance(_ui->openni2_autoWhiteBalance->isChecked());
				((CameraOpenNI2*)camera)->setAutoExposure(_ui->openni2_autoExposure->isChecked());
				((CameraOpenNI2*)camera)->setMirroring(_ui->openni2_mirroring->isChecked());
				((CameraOpenNI2*)camera)->setOpenNI2StampsAndIDsUsed(_ui->openni2_stampsIdsUsed->isChecked());
				if(CameraOpenNI2::exposureGainAvailable())
				{
					((CameraOpenNI2*)camera)->setExposure(_ui->openni2_exposure->value());
					((CameraOpenNI2*)camera)->setGain(_ui->openni2_gain->value());
				}
			}
		}
	}

	UDEBUG("");
	return camera;
}

bool PreferencesDialog::isStatisticsPublished() const
{
	return _ui->groupBox_publishing->isChecked();
}
double PreferencesDialog::getLoopThr() const
{
	return _ui->general_doubleSpinBox_hardThr->value();
}
double PreferencesDialog::getVpThr() const
{
	return _ui->general_doubleSpinBox_vp->value();
}
double PreferencesDialog::getSimThr() const
{
	return _ui->doubleSpinBox_similarityThreshold->value();
}
int PreferencesDialog::getOdomStrategy() const
{
	return _ui->odom_strategy->currentIndex();
}
int PreferencesDialog::getOdomBufferSize() const
{
	return _ui->odom_dataBufferSize->value();
}
bool PreferencesDialog::getRegVarianceFromInliersCount() const
{
	return _ui->loopClosure_bowVarianceFromInliersCount->isChecked();
}

QString PreferencesDialog::getCameraInfoDir() const
{
	return (this->getWorkingDirectory().isEmpty()?".":this->getWorkingDirectory())+"/camera_info";
}

bool PreferencesDialog::isImagesKept() const
{
	return _ui->general_checkBox_imagesKept->isChecked();
}
bool PreferencesDialog::isCloudsKept() const
{
	return _ui->general_checkBox_cloudsKept->isChecked();
}
float PreferencesDialog::getTimeLimit() const
{
	return _ui->general_doubleSpinBox_timeThr->value();
}
float PreferencesDialog::getDetectionRate() const
{
	return _ui->general_doubleSpinBox_detectionRate->value();
}
bool PreferencesDialog::isSLAMMode() const
{
	return _ui->general_checkBox_SLAM_mode->isChecked();
}
bool PreferencesDialog::isRGBDMode() const
{
	return _ui->general_checkBox_activateRGBD->isChecked();
}

/*** SETTERS ***/
void PreferencesDialog::setInputRate(double value)
{
	ULOGGER_DEBUG("imgRate=%2.2f", value);
	if(_ui->general_doubleSpinBox_imgRate->value() != value)
	{
		_ui->general_doubleSpinBox_imgRate->setValue(value);
		if(validateForm())
		{
			this->writeSettings(getTmpIniFilePath());
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::setDetectionRate(double value)
{
	ULOGGER_DEBUG("detectionRate=%2.2f", value);
	if(_ui->general_doubleSpinBox_detectionRate->value() != value)
	{
		_ui->general_doubleSpinBox_detectionRate->setValue(value);
		if(validateForm())
		{
			this->writeSettings(getTmpIniFilePath());
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::setTimeLimit(float value)
{
	ULOGGER_DEBUG("timeLimit=%fs", value);
	if(_ui->general_doubleSpinBox_timeThr->value() != value)
	{
		_ui->general_doubleSpinBox_timeThr->setValue(value);
		if(validateForm())
		{
			this->writeSettings(getTmpIniFilePath());
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::setSLAMMode(bool enabled)
{
	ULOGGER_DEBUG("slam mode=%s", enabled?"true":"false");
	if(_ui->general_checkBox_SLAM_mode->isChecked() != enabled)
	{
		_ui->general_checkBox_SLAM_mode->setChecked(enabled);
		if(validateForm())
		{
			this->writeSettings(getTmpIniFilePath());
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::testOdometry()
{
	Camera * camera = this->createCamera();
	if(!camera)
	{
		return;
	}

	ParametersMap parameters = this->getAllParameters();
	if(getOdomRegistrationApproach() < 3)
	{
		uInsert(parameters, ParametersPair(Parameters::kRegStrategy(), uNumber2Str(getOdomRegistrationApproach())));
	}
	Odometry * odometry = Odometry::create(parameters);

	OdometryThread odomThread(
			odometry, // take ownership of odometry
			_ui->odom_dataBufferSize->value());
	odomThread.registerToEventsManager();

	OdometryViewer * odomViewer = new OdometryViewer(10,
					_ui->spinBox_decimation_odom->value(),
					0.0f,
					_ui->doubleSpinBox_maxDepth_odom->value(),
					this->getOdomQualityWarnThr(),
					this,
					this->getAllParameters());
	odomViewer->setWindowTitle(tr("Odometry viewer"));
	odomViewer->resize(1280, 480+QPushButton().minimumHeight());
	odomViewer->registerToEventsManager();

	CameraThread cameraThread(camera, this->getAllParameters()); // take ownership of camera
	cameraThread.setMirroringEnabled(isSourceMirroring());
	cameraThread.setColorOnly(_ui->checkbox_rgbd_colorOnly->isChecked());
	cameraThread.setImageDecimation(_ui->spinBox_source_imageDecimation->value());
	cameraThread.setStereoToDepth(_ui->checkbox_stereo_depthGenerated->isChecked());
	cameraThread.setScanFromDepth(
			_ui->groupBox_scanFromDepth->isChecked(),
			_ui->spinBox_cameraScanFromDepth_decimation->value(),
			_ui->doubleSpinBox_cameraSCanFromDepth_maxDepth->value(),
			_ui->doubleSpinBox_cameraImages_scanVoxelSize->value(),
			_ui->spinBox_cameraImages_scanNormalsK->value());
	if(isDepthFilteringAvailable())
	{
		if(_ui->groupBox_bilateral->isChecked())
		{
			cameraThread.enableBilateralFiltering(
					_ui->doubleSpinBox_bilateral_sigmaS->value(),
					_ui->doubleSpinBox_bilateral_sigmaR->value());
		}
		if(!_ui->lineEdit_source_distortionModel->text().isEmpty())
		{
			cameraThread.setDistortionModel(_ui->lineEdit_source_distortionModel->text().toStdString());
		}
	}
	UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");
	UEventsManager::createPipe(&odomThread, odomViewer, "OdometryEvent");
	UEventsManager::createPipe(odomViewer, &odomThread, "OdometryResetEvent");

	odomThread.start();
	cameraThread.start();

	odomViewer->exec();
	delete odomViewer;

	cameraThread.join(true);
	odomThread.join(true);
}

void PreferencesDialog::testCamera()
{
	CameraViewer * window = new CameraViewer(this, this->getAllParameters());
	window->setWindowTitle(tr("Camera viewer"));
	window->resize(1280, 480+QPushButton().minimumHeight());
	window->registerToEventsManager();

	Camera * camera = this->createCamera();
	if(camera)
	{
		CameraThread cameraThread(camera, this->getAllParameters());
		cameraThread.setMirroringEnabled(isSourceMirroring());
		cameraThread.setColorOnly(_ui->checkbox_rgbd_colorOnly->isChecked());
		cameraThread.setImageDecimation(_ui->spinBox_source_imageDecimation->value());
		cameraThread.setStereoToDepth(_ui->checkbox_stereo_depthGenerated->isChecked());
		cameraThread.setScanFromDepth(
				_ui->groupBox_scanFromDepth->isChecked(),
				_ui->spinBox_cameraScanFromDepth_decimation->value(),
				_ui->doubleSpinBox_cameraSCanFromDepth_maxDepth->value(),
				_ui->doubleSpinBox_cameraImages_scanVoxelSize->value(),
				_ui->spinBox_cameraImages_scanNormalsK->value());
		if(isDepthFilteringAvailable())
		{
			if(_ui->groupBox_bilateral->isChecked())
			{
				cameraThread.enableBilateralFiltering(
						_ui->doubleSpinBox_bilateral_sigmaS->value(),
						_ui->doubleSpinBox_bilateral_sigmaR->value());
			}
			if(!_ui->lineEdit_source_distortionModel->text().isEmpty())
			{
				cameraThread.setDistortionModel(_ui->lineEdit_source_distortionModel->text().toStdString());
			}
		}
		UEventsManager::createPipe(&cameraThread, window, "CameraEvent");

		cameraThread.start();
		window->exec();
		delete window;
		cameraThread.join(true);
	}
	else
	{
		delete window;
	}
}

void PreferencesDialog::calibrate()
{
	if(this->getSourceType() == kSrcDatabase)
	{
		QMessageBox::warning(this,
				   tr("Calibration"),
				   tr("Cannot calibrate database source!"));
		return;
	}

	if(!this->getCameraInfoDir().isEmpty())
	{
		QDir dir(this->getCameraInfoDir());
		if (!dir.exists())
		{
			UINFO("Creating camera_info directory: \"%s\"", this->getCameraInfoDir().toStdString().c_str());
			if(!dir.mkpath(this->getCameraInfoDir()))
			{
				UWARN("Could create camera_info directory: \"%s\"", this->getCameraInfoDir().toStdString().c_str());
			}
		}
	}

	Src driver = this->getSourceDriver();
	if(driver == PreferencesDialog::kSrcFreenect || driver == PreferencesDialog::kSrcOpenNI2)
	{
		// 3 steps calibration: RGB -> IR -> Extrinsic
		QMessageBox::StandardButton button = QMessageBox::question(this, tr("Calibration"),
				tr("With \"%1\" driver, Color and IR cameras cannot be streamed at the "
					"same time. A 3-steps calibration is required (Color -> IR -> extrinsics). We will "
					"start with the Color camera calibration. Do you want to continue?").arg(this->getSourceDriverStr()),
					QMessageBox::Yes | QMessageBox::No | QMessageBox::Ignore, QMessageBox::Yes);

		if(button == QMessageBox::Yes || button == QMessageBox::Ignore)
		{
			_calibrationDialog->setSavingDirectory(this->getCameraInfoDir());

			Camera * camera = 0;

			// Step 1: RGB
			if(button != QMessageBox::Ignore)
			{
				camera = this->createCamera(true, true); // RAW color
				if(!camera)
				{
					return;
				}

				_calibrationDialog->setStereoMode(false); // this forces restart
				_calibrationDialog->setCameraName(QString(camera->getSerial().c_str())+"_rgb");
				_calibrationDialog->registerToEventsManager();
				CameraThread cameraThread(camera, this->getAllParameters());
				UEventsManager::createPipe(&cameraThread, _calibrationDialog, "CameraEvent");
				cameraThread.start();
				_calibrationDialog->exec();
				_calibrationDialog->unregisterFromEventsManager();
				cameraThread.join(true);
				camera = 0;
			}

			button = QMessageBox::question(this, tr("Calibration"),
						tr("We will now calibrate the IR camera. Hide the IR projector with a Post-It and "
							"make sure you have enough ambient IR light (e.g., external IR source or sunlight!) to see the "
							"checkboard with the IR camera. Do you want to continue?"),
							QMessageBox::Yes | QMessageBox::No | QMessageBox::Ignore, QMessageBox::Yes);
			if(button == QMessageBox::Yes || button == QMessageBox::Ignore)
			{
				// Step 2: IR
				if(button != QMessageBox::Ignore)
				{
					camera = this->createCamera(true, false); // RAW ir
					if(!camera)
					{
						return;
					}

					_calibrationDialog->setStereoMode(false); // this forces restart
					_calibrationDialog->setCameraName(QString(camera->getSerial().c_str())+"_depth");
					_calibrationDialog->registerToEventsManager();
					CameraThread cameraThread(camera, this->getAllParameters());
					UEventsManager::createPipe(&cameraThread, _calibrationDialog, "CameraEvent");
					cameraThread.start();
					_calibrationDialog->exec();
					_calibrationDialog->unregisterFromEventsManager();
					cameraThread.join(true);
					camera = 0;
				}

				button = QMessageBox::question(this, tr("Calibration"),
							tr("We will now calibrate the extrinsics. Important: Make sure "
								"the cameras and the checkboard don't move and that both "
								"cameras can see the checkboard. We will repeat this "
								"multiple times. Each time, you will have to move the camera (or "
								"checkboard) for a different point of view. Do you want to "
								"continue?"),
								QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

				bool ok = false;
				int totalSamples = 0;
				if(button == QMessageBox::Yes)
				{
					totalSamples = QInputDialog::getInt(this, tr("Calibration"), tr("Samples: "), 3, 1, 99, 1, &ok);
				}

				if(ok)
				{
					int count = 0;
					_calibrationDialog->setStereoMode(true, "depth", "rgb"); // this forces restart
					_calibrationDialog->setCameraName("");
					_calibrationDialog->setModal(true);
					_calibrationDialog->setProgressVisibility(false);
					_calibrationDialog->show();
					CameraModel irModel;
					CameraModel rgbModel;
					std::string serial;
					for(;count < totalSamples && button == QMessageBox::Yes; )
					{
						// Step 3: Extrinsics
						camera = this->createCamera(false, true); // Rectified color
						if(!camera)
						{
							return;
						}
						SensorData rgbData = camera->takeImage();
						UASSERT(rgbData.cameraModels().size() == 1);
						rgbModel = rgbData.cameraModels()[0];
						delete camera;
						camera = this->createCamera(false, false); // Rectified ir
						if(!camera)
						{
							return;
						}
						SensorData irData = camera->takeImage();
						serial = camera->getSerial();
						UASSERT(irData.cameraModels().size() == 1);
						irModel = irData.cameraModels()[0];
						delete camera;

						if(!rgbData.imageRaw().empty() && !irData.imageRaw().empty())
						{
							// assume rgb sensor is on right (e.g., Kinect, Xtion Live Pro)
							int pair = _calibrationDialog->getStereoPairs();
							_calibrationDialog->processImages(irData.imageRaw(), rgbData.imageRaw(), serial.c_str());
							if(_calibrationDialog->getStereoPairs() - pair > 0)
							{
								++count;
								if(count < totalSamples)
								{
									button = QMessageBox::question(this, tr("Calibration"),
										tr("A stereo pair has been taken (total=%1/%2). Move the checkboard or "
											"camera to another position. Press \"Yes\" when you are ready "
											"for the next capture.").arg(count).arg(totalSamples),
											QMessageBox::Yes | QMessageBox::Abort, QMessageBox::Yes);
								}
							}
							else
							{
								button = QMessageBox::question(this, tr("Calibration"),
										tr("Could not detect the checkboard on both images or "
										   "the point of view didn't change enough. Try again?"),
											QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
							}
						}
						else
						{
							button = QMessageBox::question(this, tr("Calibration"),
										tr("Failed to start the camera. Try again?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
						}
					}
					if(count == totalSamples && button == QMessageBox::Yes)
					{
						StereoCameraModel stereoModel = _calibrationDialog->stereoCalibration(irModel, rgbModel, true);
						stereoModel.setName(stereoModel.name(), "depth", "rgb");
						if(stereoModel.stereoTransform().isNull())
						{
							QMessageBox::warning(this, tr("Calibration"),
									tr("Extrinsic calibration has failed! Look on the terminal "
									   "for possible error messages. Make sure corresponding calibration files exist "
									   "in \"%1\" folder for camera \"%2\" or calibration name set. If not, re-do "
									   "step 1 and 2 and make sure to export the calibration files.").arg(this->getCameraInfoDir()).arg(serial.c_str()), QMessageBox::Ok);
						}
						else if(stereoModel.saveStereoTransform(this->getCameraInfoDir().toStdString()))
						{
							QMessageBox::information(this, tr("Calibration"),
									tr("Calibration is completed! Extrinsics have been saved to \"%1/%2_pose.yaml\"").arg(this->getCameraInfoDir()).arg(stereoModel.name().c_str()), QMessageBox::Ok);
						}
					}
				}
			}
		}
	}
	else // standard calibration
	{
		Camera * camera = this->createCamera(true);
		if(!camera)
		{
			return;
		}

		bool freenect2 = driver == kSrcFreenect2;
		_calibrationDialog->setStereoMode(this->getSourceType() != kSrcRGB, freenect2?"rgb":"left", freenect2?"depth":"right"); // RGB+Depth or left+right
		_calibrationDialog->setSwitchedImages(freenect2);
		_calibrationDialog->setSavingDirectory(this->getCameraInfoDir());
		_calibrationDialog->registerToEventsManager();

		CameraThread cameraThread(camera, this->getAllParameters());
		UEventsManager::createPipe(&cameraThread, _calibrationDialog, "CameraEvent");

		cameraThread.start();

		_calibrationDialog->exec();
		_calibrationDialog->unregisterFromEventsManager();

		cameraThread.join(true);
	}
}

void PreferencesDialog::calibrateSimple()
{
	_createCalibrationDialog->setCameraInfoDir(this->getCameraInfoDir());
	if(_createCalibrationDialog->exec() == QMessageBox::Accepted)
	{
		_ui->lineEdit_calibrationFile->setText(_createCalibrationDialog->cameraName());
	}
}

}

