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

#include "rtabmap/gui/PreferencesDialog.h"
#include "rtabmap/gui/DatabaseViewer.h"
#include "rtabmap/gui/OdometryViewer.h"
#include "rtabmap/gui/CalibrationDialog.h"

#include <QtCore/QSettings>
#include <QtCore/QDir>
#include <QtCore/QTimer>

#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <QtGui/QStandardItemModel>
#include <QtGui/QMainWindow>
#include <QtGui/QProgressDialog>
#include <QtGui/QScrollBar>
#include <QtGui/QCloseEvent>

#include "ui_preferencesDialog.h"

#include "rtabmap/core/Version.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/VWDictionary.h"

#include "rtabmap/gui/LoopClosureViewer.h"
#include "rtabmap/gui/DataRecorder.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UEventsManager.h>
#include "utilite/UPlot.h"

#include <opencv2/gpu/gpu.hpp>

using namespace rtabmap;

namespace rtabmap {

PreferencesDialog::PreferencesDialog(QWidget * parent) :
	QDialog(parent),
	_obsoletePanels(kPanelDummy),
	_ui(0),
	_indexModel(0),
	_initialized(false),
	_cameraThread(0),
	_odomThread(0)
{
	ULOGGER_DEBUG("");

	_ui = new Ui_preferencesDialog();
	_ui->setupUi(this);

	if(cv::gpu::getCudaEnabledDeviceCount() == 0)
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
		_ui->odom_bin_nn->removeItem(4);
		_ui->reextract_nn->removeItem(4);
	}

#if PCL_VERSION_COMPARE(<, 1, 7, 2)
	_ui->checkBox_map_shown->setChecked(false);
	_ui->checkBox_map_shown->setEnabled(false);
	_ui->label_map_shown->setText(_ui->label_map_shown->text() + " (Disabled, PCL >=1.7.2 required)");
#endif

	if(RTABMAP_NONFREE == 0)
	{
		_ui->comboBox_detector_strategy->setItemData(0, 0, Qt::UserRole - 1);
		_ui->comboBox_detector_strategy->setItemData(1, 0, Qt::UserRole - 1);
		_ui->reextract_type->setItemData(0, 0, Qt::UserRole - 1);
		_ui->reextract_type->setItemData(1, 0, Qt::UserRole - 1);
		_ui->odom_type->setItemData(0, 0, Qt::UserRole - 1);
		_ui->odom_type->setItemData(1, 0, Qt::UserRole - 1);

		_ui->comboBox_dictionary_strategy->setItemData(1, 0, Qt::UserRole - 1);
		_ui->reextract_nn->setItemData(1, 0, Qt::UserRole - 1);
		_ui->odom_bin_nn->setItemData(1, 0, Qt::UserRole - 1);
	}

	_ui->predictionPlot->showLegend(false);
	_ui->groupBox_openni2->setVisible(false);

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
	connect(_ui->pushButton_test_rgbd_camera, SIGNAL(clicked()), this, SLOT(testRGBDCamera()));

	// General panel
	connect(_ui->general_checkBox_imagesKept, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_verticalLayoutUsed, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_imageFlipped, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_imageRejectedShown, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_imageHighestHypShown, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_beep, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->horizontalSlider_keypointsOpacity, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->spinBox_odomQualityWarnThr, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));

	// Cloud rendering panel
	_3dRenderingShowClouds.resize(2);
	_3dRenderingShowClouds[0] = _ui->checkBox_showClouds;
	_3dRenderingShowClouds[1] = _ui->checkBox_showOdomClouds;

	_3dRenderingVoxelSize.resize(2);
	_3dRenderingVoxelSize[0] = _ui->doubleSpinBox_voxelSize;
	_3dRenderingVoxelSize[1] = _ui->doubleSpinBox_voxelSize_odom;

	_3dRenderingDecimation.resize(2);
	_3dRenderingDecimation[0] = _ui->spinBox_decimation;
	_3dRenderingDecimation[1] = _ui->spinBox_decimation_odom;

	_3dRenderingMaxDepth.resize(2);
	_3dRenderingMaxDepth[0] = _ui->doubleSpinBox_maxDepth;
	_3dRenderingMaxDepth[1] = _ui->doubleSpinBox_maxDepth_odom;

	_3dRenderingOpacity.resize(2);
	_3dRenderingOpacity[0] = _ui->doubleSpinBox_opacity;
	_3dRenderingOpacity[1] = _ui->doubleSpinBox_opacity_odom;

	_3dRenderingPtSize.resize(2);
	_3dRenderingPtSize[0] = _ui->spinBox_ptsize;
	_3dRenderingPtSize[1] = _ui->spinBox_ptsize_odom;

	_3dRenderingShowScans.resize(2);
	_3dRenderingShowScans[0] = _ui->checkBox_showScans;
	_3dRenderingShowScans[1] = _ui->checkBox_showOdomScans;

	_3dRenderingOpacityScan.resize(2);
	_3dRenderingOpacityScan[0] = _ui->doubleSpinBox_opacity_scan;
	_3dRenderingOpacityScan[1] = _ui->doubleSpinBox_opacity_odom_scan;

	_3dRenderingPtSizeScan.resize(2);
	_3dRenderingPtSizeScan[0] = _ui->spinBox_ptsize_scan;
	_3dRenderingPtSizeScan[1] = _ui->spinBox_ptsize_odom_scan;

	for(int i=0; i<2; ++i)
	{
		connect(_3dRenderingShowClouds[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingVoxelSize[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingDecimation[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingMaxDepth[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingShowScans[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

		connect(_3dRenderingOpacity[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingPtSize[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingOpacityScan[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingPtSizeScan[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	}

	connect(_ui->checkBox_showGraphs, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->checkBox_meshing, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_gp3Radius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_normalKSearch, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_mls, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_mlsRadius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->groupBox_poseFiltering, SIGNAL(clicked(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_cloudFilterRadius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_cloudFilterAngle, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->checkBox_map_shown, SIGNAL(clicked(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_map_resolution, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_map_fillEmptySpace, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_map_opacity, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinbox_map_fillEmptyRadius, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_map_occupancyFrom3DCloud, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	//Logging panel
	connect(_ui->comboBox_loggerLevel, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteLoggingPanel()));
	connect(_ui->comboBox_loggerEventLevel, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteLoggingPanel()));
	connect(_ui->comboBox_loggerPauseLevel, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteLoggingPanel()));
	connect(_ui->checkBox_logger_printTime, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteLoggingPanel()));
	connect(_ui->comboBox_loggerType, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteLoggingPanel()));

	//Source panel
	connect(_ui->general_doubleSpinBox_imgRate, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	//Image source
	connect(_ui->groupBox_sourceImage, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	_ui->stackedWidget_image->setCurrentIndex(_ui->source_comboBox_image_type->currentIndex());
	connect(_ui->source_comboBox_image_type, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_image, SLOT(setCurrentIndex(int)));
	connect(_ui->source_comboBox_image_type, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//usbDevice group
	connect(_ui->source_usbDevice_spinBox_id, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_imgWidth, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_imgheight, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//images group
	connect(_ui->source_images_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceImage()));
	connect(_ui->source_images_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_images_spinBox_startPos, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_images_refreshDir, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//video group
	connect(_ui->source_video_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceImage()));
	connect(_ui->source_video_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	//database group
	connect(_ui->source_database_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceDatabase()));
	connect(_ui->toolButton_dbViewer, SIGNAL(clicked()), this, SLOT(openDatabaseViewer()));
	connect(_ui->groupBox_sourceDatabase, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_database_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_ignoreOdometry, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_databaseStartPos, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//openni group
	connect(_ui->groupBox_sourceOpenni, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->radioButton_opennipcl, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->radioButton_freenect, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	_ui->radioButton_freenect->setEnabled(CameraFreenect::available());
	connect(_ui->radioButton_opennicv, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->radioButton_opennicvasus, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	_ui->radioButton_opennicv->setEnabled(CameraOpenNICV::available());
	_ui->radioButton_opennicvasus->setEnabled(CameraOpenNICV::available());
	connect(_ui->radioButton_openni2, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->radioButton_openni2, SIGNAL(toggled(bool)), this, SLOT(showOpenNI2GroupBox(bool)));
	_ui->radioButton_openni2->setEnabled(CameraOpenNI2::available());
	_ui->openni2_exposure->setEnabled(CameraOpenNI2::exposureGainAvailable());
	_ui->openni2_gain->setEnabled(CameraOpenNI2::exposureGainAvailable());
	connect(_ui->openni2_autoWhiteBalance, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->openni2_autoExposure, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->openni2_exposure, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->openni2_gain, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_openniDevice, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_openniLocalTransform, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_openniFx, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_openniFy, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_openniCx, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_openniCy, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->pushButton_calibrate, SIGNAL(clicked()), this, SLOT(calibrate()));
	connect(_ui->pushButton_calibrate_reset, SIGNAL(clicked()), this, SLOT(resetCalibration()));


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
	connect(_ui->groupBox_publishing, SIGNAL(clicked(bool)), this, SLOT(updateBasicParameter()));
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
	_ui->general_spinBox_maxRetrieved->setObjectName(Parameters::kRtabmapMaxRetrieved().c_str());
	_ui->general_checkBox_startNewMapOnLoopClosure->setObjectName(Parameters::kRtabmapStartNewMapOnLoopClosure().c_str());
	_ui->lineEdit_workingDirectory->setObjectName(Parameters::kRtabmapWorkingDirectory().c_str());
	connect(_ui->toolButton_workingDirectory, SIGNAL(clicked()), this, SLOT(changeWorkingDirectory()));

	// Memory
	_ui->general_checkBox_keepRawData->setObjectName(Parameters::kMemImageKept().c_str());
	_ui->general_checkBox_keepRehearsedNodes->setObjectName(Parameters::kMemRehearsedNodesKept().c_str());
	_ui->general_spinBox_maxStMemSize->setObjectName(Parameters::kMemSTMSize().c_str());
	_ui->doubleSpinBox_similarityThreshold->setObjectName(Parameters::kMemRehearsalSimilarity().c_str());
	_ui->general_checkBox_SLAM_mode->setObjectName(Parameters::kMemIncrementalMemory().c_str());
	_ui->general_doubleSpinBox_recentWmRatio->setObjectName(Parameters::kMemRecentWmRatio().c_str());
	_ui->general_checkBox_RehearsalIdUpdatedToNewOne->setObjectName(Parameters::kMemRehearsalIdUpdatedToNewOne().c_str());
	_ui->general_checkBox_generateIds->setObjectName(Parameters::kMemGenerateIds().c_str());
	_ui->general_checkBox_badSignaturesIgnored->setObjectName(Parameters::kMemBadSignaturesIgnored().c_str());
	_ui->general_checkBox_initWMWithAllNodes->setObjectName(Parameters::kMemInitWMWithAllNodes().c_str());

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
	_ui->comboBox_detector_strategy->setObjectName(Parameters::kKpDetectorStrategy().c_str());
	_ui->surf_doubleSpinBox_nndrRatio->setObjectName(Parameters::kKpNndrRatio().c_str());
	_ui->surf_doubleSpinBox_maxDepth->setObjectName(Parameters::kKpMaxDepth().c_str());
	_ui->surf_spinBox_wordsPerImageTarget->setObjectName(Parameters::kKpWordsPerImage().c_str());
	_ui->surf_doubleSpinBox_ratioBadSign->setObjectName(Parameters::kKpBadSignRatio().c_str());
	_ui->checkBox_kp_tfIdfLikelihoodUsed->setObjectName(Parameters::kKpTfIdfLikelihoodUsed().c_str());
	_ui->checkBox_kp_parallelized->setObjectName(Parameters::kKpParallelized().c_str());
	_ui->lineEdit_kp_roi->setObjectName(Parameters::kKpRoiRatios().c_str());
	_ui->lineEdit_dictionaryPath->setObjectName(Parameters::kKpDictionaryPath().c_str());
	connect(_ui->toolButton_dictionaryPath, SIGNAL(clicked()), this, SLOT(changeDictionaryPath()));
	_ui->checkBox_kp_newWordsComparedTogether->setObjectName(Parameters::kKpNewWordsComparedTogether().c_str());

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
	_ui->fastGpu->setObjectName(Parameters::kFASTGpu().c_str());
	_ui->fastKeypointRatio->setObjectName(Parameters::kFASTGpuKeypointsRatio().c_str());

	//ORB detector
	_ui->spinBox_ORBNFeatures->setObjectName(Parameters::kORBNFeatures().c_str());
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
	_ui->spinBox_GFTT_maxCorners->setObjectName(Parameters::kGFTTMaxCorners().c_str());
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
	_ui->rgdb_newMapOdomChange->setObjectName(Parameters::kRGBDNewMapOdomChangeDistance().c_str());
	_ui->odomScanHistory->setObjectName(Parameters::kRGBDPoseScanMatching().c_str());
	_ui->globalDetection_toroIterations->setObjectName(Parameters::kRGBDToroIterations().c_str());
	_ui->globalDetection_optimizeFromGraphEnd->setObjectName(Parameters::kRGBDOptimizeFromGraphEnd().c_str());

	_ui->groupBox_localDetection_time->setObjectName(Parameters::kRGBDLocalLoopDetectionTime().c_str());
	_ui->groupBox_localDetection_space->setObjectName(Parameters::kRGBDLocalLoopDetectionSpace().c_str());
	_ui->localDetection_radius->setObjectName(Parameters::kRGBDLocalLoopDetectionRadius().c_str());
	_ui->localDetection_maxNeighbors->setObjectName(Parameters::kRGBDLocalLoopDetectionNeighbors().c_str());
	_ui->localDetection_maxDiffID->setObjectName(Parameters::kRGBDLocalLoopDetectionMaxDiffID().c_str());

	_ui->loopClosure_bowMinInliers->setObjectName(Parameters::kLccBowMinInliers().c_str());
	_ui->loopClosure_bowInlierDistance->setObjectName(Parameters::kLccBowInlierDistance().c_str());
	_ui->loopClosure_bowIterations->setObjectName(Parameters::kLccBowIterations().c_str());
	_ui->loopClosure_bowMaxDepth->setObjectName(Parameters::kLccBowMaxDepth().c_str());
	_ui->loopClosure_bowForce2D->setObjectName(Parameters::kLccBowForce2D().c_str());

	_ui->groupBox_reextract->setObjectName(Parameters::kLccReextractActivated().c_str());
	_ui->reextract_nn->setObjectName(Parameters::kLccReextractNNType().c_str());
	_ui->reextract_nndrRatio->setObjectName(Parameters::kLccReextractNNDR().c_str());
	_ui->reextract_type->setObjectName(Parameters::kLccReextractFeatureType().c_str());
	_ui->reextract_maxFeatures->setObjectName(Parameters::kLccReextractMaxWords().c_str());

	_ui->globalDetection_icpType->setObjectName(Parameters::kLccIcpType().c_str());
	_ui->globalDetection_icpMaxDistance->setObjectName(Parameters::kLccIcpMaxDistance().c_str());

	_ui->loopClosure_icpDecimation->setObjectName(Parameters::kLccIcp3Decimation().c_str());
	_ui->loopClosure_icpMaxDepth->setObjectName(Parameters::kLccIcp3MaxDepth().c_str());
	_ui->loopClosure_icpVoxelSize->setObjectName(Parameters::kLccIcp3VoxelSize().c_str());
	_ui->loopClosure_icpSamples->setObjectName(Parameters::kLccIcp3Samples().c_str());
	_ui->loopClosure_icpMaxCorrespondenceDistance->setObjectName(Parameters::kLccIcp3MaxCorrespondenceDistance().c_str());
	_ui->loopClosure_icpIterations->setObjectName(Parameters::kLccIcp3Iterations().c_str());
	_ui->loopClosure_icpMaxFitness->setObjectName(Parameters::kLccIcp3MaxFitness().c_str());
	_ui->loopClosure_icpPointToPlane->setObjectName(Parameters::kLccIcp3PointToPlane().c_str());
	_ui->loopClosure_icpPointToPlaneNormals->setObjectName(Parameters::kLccIcp3PointToPlaneNormalNeighbors().c_str());

	_ui->loopClosure_icp2MaxCorrespondenceDistance->setObjectName(Parameters::kLccIcp2MaxCorrespondenceDistance().c_str());
	_ui->loopClosure_icp2Iterations->setObjectName(Parameters::kLccIcp2Iterations().c_str());
	_ui->loopClosure_icp2MaxFitness->setObjectName(Parameters::kLccIcp2MaxFitness().c_str());
	_ui->loopClosure_icp2Ratio->setObjectName(Parameters::kLccIcp2CorrespondenceRatio().c_str());
	_ui->loopClosure_icp2Voxel->setObjectName(Parameters::kLccIcp2VoxelSize().c_str());

	//Odometry
	_ui->odom_strategy->setObjectName(Parameters::kOdomStrategy().c_str());
	_ui->odom_type->setObjectName(Parameters::kOdomFeatureType().c_str());
	_ui->odom_linearUpdate->setObjectName(Parameters::kOdomLinearUpdate().c_str());
	_ui->odom_angularUpdate->setObjectName(Parameters::kOdomAngularUpdate().c_str());
	_ui->odom_countdown->setObjectName(Parameters::kOdomResetCountdown().c_str());
	_ui->odom_maxFeatures->setObjectName(Parameters::kOdomMaxFeatures().c_str());
	_ui->odom_ratio->setObjectName(Parameters::kOdomFeaturesRatio().c_str());
	_ui->odom_inlierDistance->setObjectName(Parameters::kOdomInlierDistance().c_str());
	_ui->odom_iterations->setObjectName(Parameters::kOdomIterations().c_str());
	_ui->odom_maxDepth->setObjectName(Parameters::kOdomMaxDepth().c_str());
	_ui->odom_minInliers->setObjectName(Parameters::kOdomMinInliers().c_str());
	_ui->odom_refine_iterations->setObjectName(Parameters::kOdomRefineIterations().c_str());
	_ui->odom_force2D->setObjectName(Parameters::kOdomForce2D().c_str());
	_ui->lineEdit_odom_roi->setObjectName(Parameters::kOdomRoiRatios().c_str());

	//Odometry BOW
	_ui->odom_localHistory->setObjectName(Parameters::kOdomBowLocalHistorySize().c_str());
	_ui->odom_bin_nn->setObjectName(Parameters::kOdomBowNNType().c_str());
	_ui->odom_bin_nndrRatio->setObjectName(Parameters::kOdomBowNNDR().c_str());

	//Odometry Optical Flow
	_ui->odom_flow_winSize->setObjectName(Parameters::kOdomFlowWinSize().c_str());
	_ui->odom_flow_maxLevel->setObjectName(Parameters::kOdomFlowMaxLevel().c_str());
	_ui->odom_flow_iterations->setObjectName(Parameters::kOdomFlowIterations().c_str());
	_ui->odom_flow_eps->setObjectName(Parameters::kOdomFlowEps().c_str());
	_ui->odom_subpix_winSize->setObjectName(Parameters::kOdomSubPixWinSize().c_str());
	_ui->odom_subpix_iterations->setObjectName(Parameters::kOdomSubPixIterations().c_str());
	_ui->odom_subpix_eps->setObjectName(Parameters::kOdomSubPixEps().c_str());

	//Stereo
	_ui->stereo_flow_winSize->setObjectName(Parameters::kStereoWinSize().c_str());
	_ui->stereo_flow_maxLevel->setObjectName(Parameters::kStereoMaxLevel().c_str());
	_ui->stereo_flow_iterations->setObjectName(Parameters::kStereoIterations().c_str());
	_ui->stereo_flow_eps->setObjectName(Parameters::kStereoEps().c_str());
	_ui->stereo_maxSlope->setObjectName(Parameters::kStereoMaxSlope().c_str());


	setupSignals();
	// custom signals
	connect(_ui->doubleSpinBox_kp_roi0, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));
	connect(_ui->doubleSpinBox_kp_roi1, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));
	connect(_ui->doubleSpinBox_kp_roi2, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));
	connect(_ui->doubleSpinBox_kp_roi3, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));

	//Create a model from the stacked widgets
	// This will add all parameters to the parameters Map
	_ui->stackedWidget->setCurrentIndex(0);
	this->setupTreeView();
	connect(_ui->treeView, SIGNAL(clicked(QModelIndex)), this, SLOT(clicked(QModelIndex)));

	_obsoletePanels = kPanelAll;

	_progressDialog = new QProgressDialog(this);
	_progressDialog->setWindowTitle(tr("Read parameters..."));
	_progressDialog->setMaximum(2);
}

PreferencesDialog::~PreferencesDialog() {
	this->saveWindowGeometry("PreferencesDialog", this);
	delete _ui;
}

void PreferencesDialog::init()
{
	//First set all default values
	const ParametersMap & defaults = Parameters::getDefaultParameters();
	for(ParametersMap::const_iterator iter=defaults.begin(); iter!=defaults.end(); ++iter)
	{
		this->setParameter(iter->first, iter->second);
	}

	this->readSettings();
	this->writeSettings();// This will create the ini file if not exist

	this->loadWindowGeometry("PreferencesDialog", this);

	_obsoletePanels = kPanelAll;
	_initialized = true;
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
		boxes = boxes.mid(0,5);
	}
	else // Advanced
	{
		boxes.removeAt(4);
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
		if(currentIndex >= 4)
		{
			_ui->stackedWidget->setCurrentIndex(4);
			currentIndex = 4;
		}
	}
	else // Advanced
	{
		if(currentIndex == 4)
		{
			_ui->stackedWidget->setCurrentIndex(5);
		}
	}
	_ui->treeView->setModel(_indexModel);
	if(currentIndex == 0) // GUI
	{
		_ui->treeView->setCurrentIndex(_indexModel->index(0, 0));
	}
	else if(currentIndex == 1) //3d rendering
	{
		_ui->treeView->setCurrentIndex(_indexModel->index(0, 0).child(0,0));
	}
	else if(currentIndex == 2) // logging
	{
		_ui->treeView->setCurrentIndex(_indexModel->index(0, 0).child(1,0));
	}
	else // source, rtabmap
	{
		_ui->treeView->setCurrentIndex(_indexModel->index(currentIndex-2, 0));
	}
	_ui->treeView->expandToDepth(0);
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
		QObject * obj = _ui->stackedWidget->findChild<QObject*>((*iter).first.c_str());
		if(obj)
		{
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
				connect(groupBox, SIGNAL(clicked(bool)), this, SLOT(addParameter(bool)));
			}
			else
			{
				ULOGGER_WARN("QObject called %s can't be cast to a supported widget", (*iter).first.c_str());
			}
		}
		else
		{
			ULOGGER_WARN("Can't find the related QObject for parameter %s", (*iter).first.c_str());
		}
	}
}

void PreferencesDialog::clicked(const QModelIndex &index)
 {
	QStandardItem * item = _indexModel->itemFromIndex(index);
	if(item && item->isEnabled())
	{
		int index = item->data().toInt();
		if(_ui->radioButton_advanced->isChecked() && index >= 4)
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
	_parameters.clear();
	_obsoletePanels = kPanelDummy;
	event->accept();
}

void PreferencesDialog::closeDialog ( QAbstractButton * button )
{
	UDEBUG("");

	QDialogButtonBox::ButtonRole role = _ui->buttonBox_global->buttonRole(button);
	switch(role)
	{
	case QDialogButtonBox::RejectRole:
		_parameters.clear();
		_obsoletePanels = kPanelDummy;
		this->reject();
		break;

	case QDialogButtonBox::AcceptRole:
		updateBasicParameter();// make that changes without editing finished signal are updated.
		if((_obsoletePanels & kPanelAll) || _parameters.size())
		{
			if(validateForm())
			{
				writeSettings();
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
			writeSettings();
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
		_ui->checkBox_beep->setChecked(false);
		_ui->checkBox_verticalLayoutUsed->setChecked(true);
		_ui->checkBox_imageFlipped->setChecked(false);
		_ui->checkBox_imageRejectedShown->setChecked(true);
		_ui->checkBox_imageHighestHypShown->setChecked(false);
		_ui->horizontalSlider_keypointsOpacity->setSliderPosition(20);
		_ui->spinBox_odomQualityWarnThr->setValue(50);
	}
	else if(groupBox->objectName() == _ui->groupBox_cloudRendering1->objectName())
	{
		for(int i=0; i<2; ++i)
		{
			_3dRenderingShowClouds[i]->setChecked(true);
			_3dRenderingVoxelSize[i]->setValue(i==2?0.005:0.00);
			_3dRenderingDecimation[i]->setValue(i==0?4:i==1?2:1);
			_3dRenderingMaxDepth[i]->setValue(i==1?0.0:4.0);
			_3dRenderingShowScans[i]->setChecked(true);

			_3dRenderingOpacity[i]->setValue(1.0);
			_3dRenderingPtSize[i]->setValue(i==0?1:2);
			_3dRenderingOpacityScan[i]->setValue(1.0);
			_3dRenderingPtSizeScan[i]->setValue(1);
		}

		_ui->checkBox_showGraphs->setChecked(true);

		_ui->checkBox_meshing->setChecked(false);
		_ui->doubleSpinBox_gp3Radius->setValue(0.04);
		_ui->spinBox_normalKSearch->setValue(20);
		_ui->checkBox_mls->setChecked(false);
		_ui->doubleSpinBox_mlsRadius->setValue(0.04);

		_ui->groupBox_poseFiltering->setChecked(true);
		_ui->doubleSpinBox_cloudFilterRadius->setValue(0.3);
		_ui->doubleSpinBox_cloudFilterAngle->setValue(30);

		_ui->checkBox_map_shown->setChecked(false);
		_ui->doubleSpinBox_map_resolution->setValue(0.05);
		_ui->checkBox_map_fillEmptySpace->setChecked(true);
		_ui->checkBox_map_occupancyFrom3DCloud->setChecked(false);
		_ui->spinbox_map_fillEmptyRadius->setValue(0);
		_ui->doubleSpinBox_map_opacity->setValue(0.75);
	}
	else if(groupBox->objectName() == _ui->groupBox_logging1->objectName())
	{
		_ui->comboBox_loggerLevel->setCurrentIndex(2);
		_ui->comboBox_loggerEventLevel->setCurrentIndex(3);
		_ui->comboBox_loggerPauseLevel->setCurrentIndex(3);
		_ui->checkBox_logger_printTime->setChecked(true);
		_ui->comboBox_loggerType->setCurrentIndex(1);
	}
	else if(groupBox->objectName() == _ui->groupBox_source0->objectName())
	{
		_ui->general_doubleSpinBox_imgRate->setValue(30.0);

		_ui->groupBox_sourceImage->setChecked(false);
		_ui->source_spinBox_imgWidth->setValue(0);
		_ui->source_spinBox_imgheight->setValue(0);
		_ui->source_images_spinBox_startPos->setValue(1);
		_ui->source_images_refreshDir->setChecked(false);

		_ui->groupBox_sourceDatabase->setChecked(false);
		_ui->source_checkBox_ignoreOdometry->setChecked(false);
		_ui->source_spinBox_databaseStartPos->setValue(0);

		_ui->groupBox_sourceOpenni->setChecked(true);
		_ui->radioButton_opennipcl->setChecked(true);
		_ui->radioButton_freenect->setChecked(false);
		_ui->radioButton_openni2->setChecked(false);
		_ui->radioButton_opennicv->setChecked(false);
		_ui->radioButton_opennicvasus->setChecked(false);
		_ui->openni2_autoWhiteBalance->setChecked(true);
		_ui->openni2_autoExposure->setChecked(true);
		_ui->openni2_exposure->setValue(0);
		_ui->openni2_gain->setValue(100);
		_ui->lineEdit_openniDevice->setText("");
		_ui->lineEdit_openniLocalTransform->setText("0 0 0 -PI_2 0 -PI_2");
		_ui->doubleSpinBox_openniFx->setValue(0.0);
		_ui->doubleSpinBox_openniFy->setValue(0.0);
		_ui->doubleSpinBox_openniCx->setValue(0.0);
		_ui->doubleSpinBox_openniCy->setValue(0.0);
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
				this->setParameter(key, defaults.at(key));
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
	ULOGGER_DEBUG("");
	readGuiSettings(filePath);
	readCameraSettings(filePath);
	if(!readCoreSettings(filePath))
	{
		_parameters.clear();
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
	_ui->comboBox_loggerLevel->setCurrentIndex(settings.value("loggerLevel", _ui->comboBox_loggerLevel->currentIndex()).toInt());
	_ui->comboBox_loggerEventLevel->setCurrentIndex(settings.value("loggerEventLevel", _ui->comboBox_loggerEventLevel->currentIndex()).toInt());
	_ui->comboBox_loggerPauseLevel->setCurrentIndex(settings.value("loggerPauseLevel", _ui->comboBox_loggerPauseLevel->currentIndex()).toInt());
	_ui->comboBox_loggerType->setCurrentIndex(settings.value("loggerType", _ui->comboBox_loggerType->currentIndex()).toInt());
	_ui->checkBox_logger_printTime->setChecked(settings.value("loggerPrintTime", _ui->checkBox_logger_printTime->isChecked()).toBool());
	_ui->checkBox_verticalLayoutUsed->setChecked(settings.value("verticalLayoutUsed", _ui->checkBox_verticalLayoutUsed->isChecked()).toBool());
	_ui->checkBox_imageFlipped->setChecked(settings.value("imageFlipped", _ui->checkBox_imageFlipped->isChecked()).toBool());
	_ui->checkBox_imageRejectedShown->setChecked(settings.value("imageRejectedShown", _ui->checkBox_imageRejectedShown->isChecked()).toBool());
	_ui->checkBox_imageHighestHypShown->setChecked(settings.value("imageHighestHypShown", _ui->checkBox_imageHighestHypShown->isChecked()).toBool());
	_ui->checkBox_beep->setChecked(settings.value("beep", _ui->checkBox_beep->isChecked()).toBool());
	_ui->horizontalSlider_keypointsOpacity->setValue(settings.value("keypointsOpacity", _ui->horizontalSlider_keypointsOpacity->value()).toInt());
	_ui->spinBox_odomQualityWarnThr->setValue(settings.value("odomQualityThr", _ui->spinBox_odomQualityWarnThr->value()).toInt());

	for(int i=0; i<2; ++i)
	{
		_3dRenderingShowClouds[i]->setChecked(settings.value(QString("showClouds%1").arg(i), _3dRenderingShowClouds[i]->isChecked()).toBool());
		_3dRenderingVoxelSize[i]->setValue(settings.value(QString("voxelSize%1").arg(i), _3dRenderingVoxelSize[i]->value()).toDouble());
		_3dRenderingDecimation[i]->setValue(settings.value(QString("decimation%1").arg(i), _3dRenderingDecimation[i]->value()).toInt());
		_3dRenderingMaxDepth[i]->setValue(settings.value(QString("maxDepth%1").arg(i), _3dRenderingMaxDepth[i]->value()).toDouble());
		_3dRenderingShowScans[i]->setChecked(settings.value(QString("showScans%1").arg(i), _3dRenderingShowScans[i]->isChecked()).toBool());

		_3dRenderingOpacity[i]->setValue(settings.value(QString("opacity%1").arg(i), _3dRenderingOpacity[i]->value()).toDouble());
		_3dRenderingPtSize[i]->setValue(settings.value(QString("ptSize%1").arg(i), _3dRenderingPtSize[i]->value()).toInt());
		_3dRenderingOpacityScan[i]->setValue(settings.value(QString("opacityScan%1").arg(i), _3dRenderingOpacityScan[i]->value()).toDouble());
		_3dRenderingPtSizeScan[i]->setValue(settings.value(QString("ptSizeScan%1").arg(i), _3dRenderingPtSizeScan[i]->value()).toInt());
	}
	_ui->checkBox_showGraphs->setChecked(settings.value("showGraphs", _ui->checkBox_showGraphs->isChecked()).toBool());

	_ui->checkBox_meshing->setChecked(settings.value("meshing", _ui->checkBox_meshing->isChecked()).toBool());
	_ui->doubleSpinBox_gp3Radius->setValue(settings.value("meshGP3Radius", _ui->doubleSpinBox_gp3Radius->value()).toDouble());
	_ui->spinBox_normalKSearch->setValue(settings.value("meshNormalKSearch", _ui->spinBox_normalKSearch->value()).toInt());
	_ui->checkBox_mls->setChecked(settings.value("meshSmoothing", _ui->checkBox_mls->isChecked()).toBool());
	_ui->doubleSpinBox_mlsRadius->setValue(settings.value("meshSmoothingRadius", _ui->doubleSpinBox_mlsRadius->value()).toDouble());

	_ui->groupBox_poseFiltering->setChecked(settings.value("cloudFiltering", _ui->groupBox_poseFiltering->isChecked()).toBool());
	_ui->doubleSpinBox_cloudFilterRadius->setValue(settings.value("cloudFilteringRadius", _ui->doubleSpinBox_cloudFilterRadius->value()).toDouble());
	_ui->doubleSpinBox_cloudFilterAngle->setValue(settings.value("cloudFilteringAngle", _ui->doubleSpinBox_cloudFilterAngle->value()).toDouble());

	_ui->checkBox_map_shown->setChecked(settings.value("gridMapShown", _ui->checkBox_map_shown->isChecked()).toBool());
	_ui->doubleSpinBox_map_resolution->setValue(settings.value("gridMapResolution", _ui->doubleSpinBox_map_resolution->value()).toDouble());
	_ui->checkBox_map_fillEmptySpace->setChecked(settings.value("gridMapFillEmptySpace", _ui->checkBox_map_fillEmptySpace->isChecked()).toBool());
	_ui->checkBox_map_occupancyFrom3DCloud->setChecked(settings.value("gridMapOccupancyFrom3DCloud", _ui->checkBox_map_occupancyFrom3DCloud->isChecked()).toBool());
	_ui->spinbox_map_fillEmptyRadius->setValue(settings.value("gridMapFillEmptyRadius", _ui->spinbox_map_fillEmptyRadius->value()).toInt());
	_ui->doubleSpinBox_map_opacity->setValue(settings.value("gridMapOpacity", _ui->doubleSpinBox_map_opacity->value()).toDouble());

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
	_ui->groupBox_sourceImage->setChecked(settings.value("imageUsed", _ui->groupBox_sourceImage->isChecked()).toBool());
	_ui->general_doubleSpinBox_imgRate->setValue(settings.value("imgRate", _ui->general_doubleSpinBox_imgRate->value()).toDouble());
	_ui->source_comboBox_image_type->setCurrentIndex(settings.value("type", _ui->source_comboBox_image_type->currentIndex()).toInt());
	_ui->source_spinBox_imgWidth->setValue(settings.value("imgWidth",_ui->source_spinBox_imgWidth->value()).toInt());
	_ui->source_spinBox_imgheight->setValue(settings.value("imgHeight",_ui->source_spinBox_imgheight->value()).toInt());
	//usbDevice group
	settings.beginGroup("usbDevice");
	_ui->source_usbDevice_spinBox_id->setValue(settings.value("id",_ui->source_usbDevice_spinBox_id->value()).toInt());
	settings.endGroup(); // usbDevice
	//images group
	settings.beginGroup("images");
	_ui->source_images_lineEdit_path->setText(settings.value("path", _ui->source_images_lineEdit_path->text()).toString());
	_ui->source_images_spinBox_startPos->setValue(settings.value("startPos",_ui->source_images_spinBox_startPos->value()).toInt());
	_ui->source_images_refreshDir->setChecked(settings.value("refreshDir",_ui->source_images_refreshDir->isChecked()).toBool());
	settings.endGroup(); // images
	//video group
	settings.beginGroup("video");
	_ui->source_video_lineEdit_path->setText(settings.value("path", _ui->source_video_lineEdit_path->text()).toString());
	settings.endGroup(); // video
	settings.endGroup(); // Camera

	settings.beginGroup("Database");
	_ui->groupBox_sourceDatabase->setChecked(settings.value("databaseUsed", _ui->groupBox_sourceDatabase->isChecked()).toBool());
	_ui->source_database_lineEdit_path->setText(settings.value("path",_ui->source_database_lineEdit_path->text()).toString());
	_ui->source_checkBox_ignoreOdometry->setChecked(settings.value("ignoreOdometry", _ui->source_checkBox_ignoreOdometry->isChecked()).toBool());
	_ui->source_spinBox_databaseStartPos->setValue(settings.value("startPos", _ui->source_spinBox_databaseStartPos->value()).toInt());
	settings.endGroup(); // Database

	settings.beginGroup("Openni");
	_ui->groupBox_sourceOpenni->setChecked(settings.value("openniUsed", _ui->groupBox_sourceOpenni->isChecked()).toBool());
	_ui->radioButton_opennipcl->setChecked(settings.value("openniType", _ui->radioButton_opennipcl->isChecked()).toBool());
	_ui->radioButton_freenect->setChecked(settings.value("freenectType", _ui->radioButton_freenect->isChecked()).toBool());
	_ui->radioButton_openni2->setChecked(settings.value("openni2", _ui->radioButton_openni2->isChecked()).toBool());
	_ui->radioButton_opennicv->setChecked(settings.value("openniCvType", _ui->radioButton_opennicv->isChecked()).toBool());
	_ui->radioButton_opennicvasus->setChecked(settings.value("openniCvAsusType", _ui->radioButton_opennicvasus->isChecked()).toBool());
	_ui->openni2_autoWhiteBalance->setChecked(settings.value("openni2AutoWhiteBalance", _ui->openni2_autoWhiteBalance->isChecked()).toBool());
	_ui->openni2_autoExposure->setChecked(settings.value("openni2AutoExposure", _ui->openni2_autoExposure->isChecked()).toBool());
	_ui->openni2_exposure->setValue(settings.value("openni2Exposure", _ui->openni2_exposure->value()).toInt());
	_ui->openni2_gain->setValue(settings.value("openni2Gain", _ui->openni2_gain->value()).toInt());
	_ui->lineEdit_openniDevice->setText(settings.value("device",_ui->lineEdit_openniDevice->text()).toString());
	_ui->lineEdit_openniLocalTransform->setText(settings.value("localTransform",_ui->lineEdit_openniLocalTransform->text()).toString());
	_ui->doubleSpinBox_openniFx->setValue(settings.value("fx", _ui->doubleSpinBox_openniFx->value()).toDouble());
	_ui->doubleSpinBox_openniFy->setValue(settings.value("fy", _ui->doubleSpinBox_openniFy->value()).toDouble());
	_ui->doubleSpinBox_openniCx->setValue(settings.value("cx", _ui->doubleSpinBox_openniCx->value()).toDouble());
	_ui->doubleSpinBox_openniCy->setValue(settings.value("cy", _ui->doubleSpinBox_openniCy->value()).toDouble());
	settings.endGroup(); // Openni
}

bool PreferencesDialog::readCoreSettings(const QString & filePath)
{
	UDEBUG("");
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}

	if(!QFile::exists(path))
	{
		QMessageBox::information(this, tr("INI file doesn't exist..."), tr("The configuration file \"%1\" does not exist, it will be created with default parameters.").arg(path));
	}

	QSettings settings(path, QSettings::IniFormat);


	settings.beginGroup("Core");

	// Compare version in ini with the current RTAB-Map version
	QStringList version = settings.value("Version", "").toString().split('.');
	if(version.size() == 3)
	{
		if(!RTABMAP_VERSION_COMPARE(version[0].toInt(), version[1].toInt(), version[2].toInt()))
		{
			if(path.contains(".rtabmap"))
			{
				UWARN("Version in the config file \"%s\" is more recent (\"%s\") than "
					   "current RTAB-Map version used (\"%s\"). The config file will be upgraded "
					   "to new version.",
					   path.toStdString().c_str(),
					   settings.value("Version", "").toString().toStdString().c_str(),
					   RTABMAP_VERSION);
			}
			else
			{
				UERROR("Version in the config file \"%s\" is more recent (\"%s\") than "
					   "current RTAB-Map version used (\"%s\"). New parameters (if there are some) will "
					   "be ignored.",
					   path.toStdString().c_str(),
					   settings.value("Version", "").toString().toStdString().c_str(),
					   RTABMAP_VERSION);
			}
		}
	}

	QStringList keys = settings.allKeys();
	// Get profile
	const rtabmap::ParametersMap & parameters = Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::const_iterator iter = parameters.begin(); iter!=parameters.end(); ++iter)
	{
		QString key((*iter).first.c_str());
		QString value = settings.value(key, "").toString();
		if(!value.isEmpty())
		{
			this->setParameter(key.toStdString(), value.toStdString());
		}
		else
		{
			UDEBUG("key.toStdString()=%s", key.toStdString().c_str());
			// Use the default value if the key doesn't exist yet
			this->setParameter(key.toStdString(), (*iter).second);

			// Add information about the working directory if not in the config file
			if(key.toStdString().compare(Parameters::kRtabmapWorkingDirectory()) == 0)
			{
				if(!_initialized)
				{
					QMessageBox::information(this,
							tr("Working directory"),
							tr("RTAB-Map needs a working directory to put the database.\n\n"
							   "By default, the directory \"%1\" is used.\n\n"
							   "The working directory can be changed any time in the "
							   "preferences menu.").arg(
									   Parameters::defaultRtabmapWorkingDirectory().c_str()));
				}
			}
		}
	}
	settings.endGroup(); // Core
	return true;
}

bool PreferencesDialog::saveConfigTo()
{
	QString path = QFileDialog::getSaveFileName(this, tr("Save settings..."), this->getWorkingDirectory()+QDir::separator()+"config.ini", "*.ini");
	if(!path.isEmpty())
	{
		this->writeSettings(path);
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
	}
}

void PreferencesDialog::writeSettings(const QString & filePath)
{
	writeGuiSettings(filePath);
	writeCameraSettings(filePath);
	writeCoreSettings(filePath);

	if(_parameters.size())
	{
		emit settingsChanged(_parameters);
	}

	if(_obsoletePanels)
	{
		emit settingsChanged(_obsoletePanels);
	}

	_parameters.clear();
	_obsoletePanels = kPanelDummy;
}

void PreferencesDialog::writeGuiSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);
	settings.beginGroup("Gui");

	settings.beginGroup("General");
	settings.setValue("imagesKept", _ui->general_checkBox_imagesKept->isChecked());
	settings.setValue("loggerLevel", _ui->comboBox_loggerLevel->currentIndex());
	settings.setValue("loggerEventLevel", _ui->comboBox_loggerEventLevel->currentIndex());
	settings.setValue("loggerPauseLevel", _ui->comboBox_loggerPauseLevel->currentIndex());
	settings.setValue("loggerType", _ui->comboBox_loggerType->currentIndex());
	settings.setValue("loggerPrintTime", _ui->checkBox_logger_printTime->isChecked());
	settings.setValue("verticalLayoutUsed", _ui->checkBox_verticalLayoutUsed->isChecked());
	settings.setValue("imageFlipped", _ui->checkBox_imageFlipped->isChecked());
	settings.setValue("imageRejectedShown", _ui->checkBox_imageRejectedShown->isChecked());
	settings.setValue("imageHighestHypShown", _ui->checkBox_imageHighestHypShown->isChecked());
	settings.setValue("beep", _ui->checkBox_beep->isChecked());
	settings.setValue("keypointsOpacity", _ui->horizontalSlider_keypointsOpacity->value());
	settings.setValue("odomQualityThr", _ui->spinBox_odomQualityWarnThr->value());

	for(int i=0; i<2; ++i)
	{
		settings.setValue(QString("showClouds%1").arg(i), _3dRenderingShowClouds[i]->isChecked());
		settings.setValue(QString("voxelSize%1").arg(i), _3dRenderingVoxelSize[i]->value());
		settings.setValue(QString("decimation%1").arg(i), _3dRenderingDecimation[i]->value());
		settings.setValue(QString("maxDepth%1").arg(i), _3dRenderingMaxDepth[i]->value());
		settings.setValue(QString("showScans%1").arg(i), _3dRenderingShowScans[i]->isChecked());

		settings.setValue(QString("opacity%1").arg(i), _3dRenderingOpacity[i]->value());
		settings.setValue(QString("ptSize%1").arg(i), _3dRenderingPtSize[i]->value());
		settings.setValue(QString("opacityScan%1").arg(i), _3dRenderingOpacityScan[i]->value());
		settings.setValue(QString("ptSizeScan%1").arg(i), _3dRenderingPtSizeScan[i]->value());
	}
	settings.setValue("showGraphs", _ui->checkBox_showGraphs->isChecked());

	settings.setValue("meshing", _ui->checkBox_meshing->isChecked());
	settings.setValue("meshGP3Radius", _ui->doubleSpinBox_gp3Radius->value());
	settings.setValue("meshNormalKSearch", _ui->spinBox_normalKSearch->value());
	settings.setValue("meshSmoothing", _ui->checkBox_mls->isChecked());
	settings.setValue("meshSmoothingRadius", _ui->doubleSpinBox_mlsRadius->value());

	settings.setValue("cloudFiltering", _ui->groupBox_poseFiltering->isChecked());
	settings.setValue("cloudFilteringRadius", _ui->doubleSpinBox_cloudFilterRadius->value());
	settings.setValue("cloudFilteringAngle", _ui->doubleSpinBox_cloudFilterAngle->value());

	settings.setValue("gridMapShown", _ui->checkBox_map_shown->isChecked());
	settings.setValue("gridMapResolution", _ui->doubleSpinBox_map_resolution->value());
	settings.setValue("gridMapFillEmptySpace", _ui->checkBox_map_fillEmptySpace->isChecked());
	settings.setValue("gridMapOccupancyFrom3DCloud", _ui->checkBox_map_occupancyFrom3DCloud->isChecked());
	settings.setValue("gridMapFillEmptyRadius", _ui->spinbox_map_fillEmptyRadius->value());
	settings.setValue("gridMapOpacity", _ui->doubleSpinBox_map_opacity->value());
	settings.endGroup(); // General

	settings.endGroup(); // rtabmap
}

void PreferencesDialog::writeCameraSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);
	settings.beginGroup("Camera");
	settings.setValue("imageUsed", 		_ui->groupBox_sourceImage->isChecked());
	settings.setValue("imgRate", 		_ui->general_doubleSpinBox_imgRate->value());
	settings.setValue("type", 			_ui->source_comboBox_image_type->currentIndex());
	settings.setValue("imgWidth", 		_ui->source_spinBox_imgWidth->value());
	settings.setValue("imgHeight", 		_ui->source_spinBox_imgheight->value());
	//usbDevice group
	settings.beginGroup("usbDevice");
	settings.setValue("id", 			_ui->source_usbDevice_spinBox_id->value());
	settings.endGroup(); //usbDevice
	//images group
	settings.beginGroup("images");
	settings.setValue("path", 			_ui->source_images_lineEdit_path->text());
	settings.setValue("startPos", 		_ui->source_images_spinBox_startPos->value());
	settings.setValue("refreshDir", 	_ui->source_images_refreshDir->isChecked());
	settings.endGroup(); //images
	//video group
	settings.beginGroup("video");
	settings.setValue("path", 			_ui->source_video_lineEdit_path->text());
	settings.endGroup(); //video

	settings.endGroup(); // Camera

	settings.beginGroup("Database");
	settings.setValue("databaseUsed", 	_ui->groupBox_sourceDatabase->isChecked());
	settings.setValue("path", 			_ui->source_database_lineEdit_path->text());
	settings.setValue("ignoreOdometry", _ui->source_checkBox_ignoreOdometry->isChecked());
	settings.setValue("startPos",       _ui->source_spinBox_databaseStartPos->value());
	settings.endGroup();

	settings.beginGroup("Openni");
	settings.setValue("openniUsed", 	_ui->groupBox_sourceOpenni->isChecked());
	settings.setValue("openniType", 	_ui->radioButton_opennipcl->isChecked());
	settings.setValue("freenectType", 	_ui->radioButton_freenect->isChecked());
	settings.setValue("openni2", 		_ui->radioButton_openni2->isChecked());
	settings.setValue("openniCvType", 	_ui->radioButton_opennicv->isChecked());
	settings.setValue("openniCvAsusType", 	_ui->radioButton_opennicvasus->isChecked());
	settings.setValue("openni2AutoWhiteBalance", _ui->openni2_autoWhiteBalance->isChecked());
	settings.setValue("openni2AutoExposure", 	_ui->openni2_autoExposure->isChecked());
	settings.setValue("openni2Exposure", 		_ui->openni2_exposure->value());
	settings.setValue("openni2Gain", 			_ui->openni2_gain->value());
	settings.setValue("device", 		_ui->lineEdit_openniDevice->text());
	settings.setValue("localTransform", _ui->lineEdit_openniLocalTransform->text());
	settings.setValue("fx", _ui->doubleSpinBox_openniFx->value());
	settings.setValue("fy", _ui->doubleSpinBox_openniFy->value());
	settings.setValue("cx", _ui->doubleSpinBox_openniCx->value());
	settings.setValue("cy", _ui->doubleSpinBox_openniCy->value());
	settings.endGroup();
}

void PreferencesDialog::writeCoreSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);
	settings.beginGroup("Core");

	// save current RTAB-Map version
	settings.setValue("Version", QString(RTABMAP_VERSION));

	const rtabmap::ParametersMap & parameters = Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		QObject * obj = _ui->stackedWidget->findChild<QObject*>((*iter).first.c_str());
		if(obj)
		{
			QSpinBox * spin = qobject_cast<QSpinBox *>(obj);
			QDoubleSpinBox * doubleSpin = qobject_cast<QDoubleSpinBox *>(obj);
			QComboBox * combo = qobject_cast<QComboBox *>(obj);
			QCheckBox * check = qobject_cast<QCheckBox *>(obj);
			QRadioButton * radio = qobject_cast<QRadioButton *>(obj);
			QLineEdit * lineEdit = qobject_cast<QLineEdit *>(obj);
			QGroupBox * groupBox = qobject_cast<QGroupBox *>(obj);
			if(spin)
			{
				settings.setValue(obj->objectName(), spin->value());
			}
			else if(doubleSpin)
			{
				settings.setValue(obj->objectName(), doubleSpin->value());
			}
			else if(combo)
			{
				settings.setValue(obj->objectName(), combo->currentIndex());
			}
			else if(check)
			{
				settings.setValue(obj->objectName(), uBool2Str(check->isChecked()).c_str());
			}
			else if(radio)
			{
				settings.setValue(obj->objectName(), uBool2Str(radio->isChecked()).c_str());
			}
			else if(lineEdit)
			{
				settings.setValue(obj->objectName(), lineEdit->text());
			}
			else if(groupBox)
			{
				settings.setValue(obj->objectName(), uBool2Str(groupBox->isChecked()).c_str());
			}
			else
			{
				ULOGGER_WARN("QObject called %s can't be cast to a supported widget", (*iter).first.c_str());
			}
		}
		else
		{
			ULOGGER_WARN("Can't find the related QObject for parameter %s", (*iter).first.c_str());
		}
	}
	settings.endGroup(); // Core
}

bool PreferencesDialog::validateForm()
{
	if(RTABMAP_NONFREE == 0)
	{
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
		// odom type
		if(_ui->odom_type->currentIndex() <= 1)
		{
			QMessageBox::warning(this, tr("Parameter warning"),
					tr("Selected feature type (SURF/SIFT) is not available. RTAB-Map is not built "
					   "with the nonfree module from OpenCV. GFTT/Brief is set instead for odometry."));
			_ui->odom_type->setCurrentIndex(Feature2D::kFeatureGfttBrief);
		}
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
	else if(_ui->comboBox_dictionary_strategy->currentIndex() == VWDictionary::kNNFlannKdTree && _ui->comboBox_detector_strategy->currentIndex() >1)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("With the selected feature type (ORB, FAST, FREAK or BRIEF), parameter \"Visual word->Nearest Neighbor\" "
				   "cannot be KD-Tree (used for float descriptor). BruteForce matching is set instead for the bag-of-words dictionary."));
		_ui->comboBox_dictionary_strategy->setCurrentIndex(VWDictionary::kNNBruteForce);
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
	else if(_ui->reextract_nn->currentIndex() == VWDictionary::kNNFlannKdTree && _ui->reextract_type->currentIndex() >1)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("With the selected feature type (ORB, FAST, FREAK or BRIEF), parameter \"Visual word->Nearest Neighbor\" "
				   "cannot be KD-Tree (used for float descriptor). BruteForce matching is set instead for the re-extraction "
					   "of features on loop closure."));
		_ui->reextract_nn->setCurrentIndex(VWDictionary::kNNBruteForce);
	}

	// odom type
	if(_ui->odom_bin_nn->currentIndex() == VWDictionary::kNNFlannLSH && _ui->odom_type->currentIndex() <= 1)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("With the selected feature type (SURF or SIFT), parameter \"Odometry->Nearest Neighbor\" "
				   "cannot be LSH (used for binary descriptor). KD-tree is set instead for odometry."));
		_ui->odom_bin_nn->setCurrentIndex(VWDictionary::kNNFlannKdTree);
	}
	else if(_ui->odom_bin_nn->currentIndex() == VWDictionary::kNNFlannKdTree && _ui->odom_type->currentIndex() >1)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("With the selected feature type (ORB, FAST, FREAK or BRIEF), parameter \"Odometry->Nearest Neighbor\" "
				   "cannot be KD-Tree (used for float descriptor). BruteForce matching is set instead for odometry."));
		_ui->odom_bin_nn->setCurrentIndex(VWDictionary::kNNBruteForce);
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
		_ui->lineEdit_workingDirectory->setEnabled(false);
		_ui->toolButton_workingDirectory->setEnabled(false);
		_ui->label_workingDirectory->setEnabled(false);

		_ui->lineEdit_dictionaryPath->setEnabled(false);
		_ui->toolButton_dictionaryPath->setEnabled(false);
		_ui->label_dictionaryPath->setEnabled(false);

		_ui->groupBox_source0->setEnabled(false);
		_ui->groupBox_odometry2->setEnabled(false);

		this->setWindowTitle(tr("Preferences [Monitoring mode]"));
	}
	else
	{
		_ui->lineEdit_workingDirectory->setEnabled(true);
		_ui->toolButton_workingDirectory->setEnabled(true);
		_ui->label_workingDirectory->setEnabled(true);

		_ui->lineEdit_dictionaryPath->setEnabled(true);
		_ui->toolButton_dictionaryPath->setEnabled(true);
		_ui->label_dictionaryPath->setEnabled(true);

		_ui->groupBox_source0->setEnabled(true);
		_ui->groupBox_odometry2->setEnabled(true);

		this->setWindowTitle(tr("Preferences"));
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

	this->readSettings();

	_progressDialog->setValue(1);
	if(this->isVisible())
	{
		_progressDialog->setLabelText(tr("Setup dialog..."));

		this->updatePredictionPlot();
		this->setupKpRoiPanel();
	}

	_progressDialog->setValue(2); // this will make closing...
}

void PreferencesDialog::saveWindowGeometry(const QString & windowName, const QWidget * window)
{
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup(windowName);
	settings.setValue("geometry", window->saveGeometry());
	settings.endGroup(); // "windowName"
	settings.endGroup(); // rtabmap
}

void PreferencesDialog::loadWindowGeometry(const QString & windowName, QWidget * window)
{
	QByteArray bytes;
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup(windowName);
	bytes = settings.value("geometry", QByteArray()).toByteArray();
	if(!bytes.isEmpty())
	{
		window->restoreGeometry(bytes);
	}
	settings.endGroup(); // "windowName"
	settings.endGroup(); // rtabmap
}

void PreferencesDialog::saveMainWindowState(const QMainWindow * mainWindow)
{
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup("MainWindow");
	settings.setValue("state", mainWindow->saveState());
	settings.endGroup(); // "MainWindow"
	settings.endGroup(); // rtabmap

	saveWindowGeometry("MainWindow", mainWindow);
}

void PreferencesDialog::loadMainWindowState(QMainWindow * mainWindow)
{
	QByteArray bytes;
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup("MainWindow");
	bytes = settings.value("state", QByteArray()).toByteArray();
	if(!bytes.isEmpty())
	{
		mainWindow->restoreState(bytes);
	}
	settings.endGroup(); // "MainWindow"
	settings.endGroup(); // rtabmap

	loadWindowGeometry("MainWindow", mainWindow);
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

void PreferencesDialog::selectSourceImage(Src src)
{
	ULOGGER_DEBUG("");

	if(_ui->general_checkBox_activateRGBD->isChecked())
	{
		int button = QMessageBox::information(this,
				tr("Desactivate RGB-D SLAM?"),
				tr("You've selected source input as images only and RGB-D SLAM mode is activated. "
				   "RGB-D SLAM cannot work with images only so do you want to desactivate it?"),
				QMessageBox::Yes | QMessageBox::No);
		if(button & QMessageBox::Yes)
		{
			_ui->general_checkBox_activateRGBD->setChecked(false);
		}
	}

	bool fromPrefDialog = false;
	//bool modified = false;
	if(src == kSrcUndef)
	{
		fromPrefDialog = true;
		if(_ui->source_comboBox_image_type->currentIndex() == 1)
		{
			src = kSrcImages;
		}
		else if(_ui->source_comboBox_image_type->currentIndex() == 2)
		{
			src = kSrcVideo;
		}
		else
		{
			src = kSrcUsbDevice;
		}
	}
	else
	{
		// from user
		_ui->groupBox_sourceImage->setChecked(true);
	}

	if(src == kSrcImages)
	{
		QString path = QFileDialog::getExistingDirectory(this, QString(), _ui->source_images_lineEdit_path->text());
		QDir dir(path);
		if(!path.isEmpty() && dir.exists())
		{
			QStringList filters;
			filters << "*.jpg" << "*.ppm" << "*.bmp" << "*.png" << "*.pnm";
			dir.setNameFilters(filters);
			QFileInfoList files = dir.entryInfoList();
			if(!files.empty())
			{
				_ui->source_comboBox_image_type->setCurrentIndex(1);
				_ui->source_images_lineEdit_path->setText(path);
				_ui->source_images_spinBox_startPos->setValue(1);
				_ui->source_images_refreshDir->setChecked(false);
			}
			else
			{
				QMessageBox::information(this,
										   tr("RTAB-Map"),
										   tr("Images must be one of these formats: ") + filters.join(" "));
			}
		}
	}
	else if(src == kSrcVideo)
	{
		QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->source_video_lineEdit_path->text(), tr("Videos (*.avi *.mpg *.mp4)"));
		QFile file(path);
		if(!path.isEmpty() && file.exists())
		{
			_ui->source_comboBox_image_type->setCurrentIndex(2);
			_ui->source_video_lineEdit_path->setText(path);
		}
	}
	else // kSrcUsbDevice
	{
		_ui->source_comboBox_image_type->setCurrentIndex(0);
	}

	if(!fromPrefDialog && _obsoletePanels)
	{
		_ui->groupBox_sourceDatabase->setChecked(false);
		_ui->groupBox_sourceOpenni->setChecked(false);
		if(validateForm())
		{
			this->writeSettings();
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::selectSourceDatabase(bool user)
{
	ULOGGER_DEBUG("");

	QString dir = _ui->source_database_lineEdit_path->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("RTAB-Map database files (*.db)"));
	QFile file(path);
	if(!path.isEmpty() && file.exists())
	{
		int r = QMessageBox::question(this, tr("Odometry in database..."), tr("Use odometry saved in database (if some saved)?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

		if(user)
		{
			// from user
			_ui->groupBox_sourceDatabase->setChecked(true);
		}

		_ui->source_checkBox_ignoreOdometry->setChecked(r != QMessageBox::Yes);
		_ui->source_database_lineEdit_path->setText(path);
		_ui->source_spinBox_databaseStartPos->setValue(0);

		if(user && _obsoletePanels)
		{
			_ui->groupBox_sourceImage->setChecked(false);
			_ui->groupBox_sourceOpenni->setChecked(false);
			if(validateForm())
			{
				this->writeSettings();
			}
			else
			{
				this->readSettingsBegin();
			}
		}
	}
}

void PreferencesDialog::selectSourceRGBD(Src src)
{
	ULOGGER_DEBUG("");

	if(!_ui->general_checkBox_activateRGBD->isChecked())
	{
		int button = QMessageBox::information(this,
				tr("Activate RGB-D SLAM?"),
				tr("You've selected RGB-D camera as source input, "
				   "would you want to activate RGB-D SLAM mode?"),
				QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
		if(button & QMessageBox::Yes)
		{
			_ui->general_checkBox_activateRGBD->setChecked(true);
		}
	}

	_ui->groupBox_sourceOpenni->setChecked(true);
	_ui->radioButton_opennipcl->setChecked(src == kSrcOpenNI_PCL);
	_ui->radioButton_freenect->setChecked(src == kSrcFreenect);
	_ui->radioButton_opennicv->setChecked(src == kSrcOpenNI_CV);
	_ui->radioButton_opennicvasus->setChecked(src == kSrcOpenNI_CV_ASUS);
	_ui->radioButton_openni2->setChecked(src == kSrcOpenNI2);

	if(_obsoletePanels)
	{
		_ui->groupBox_sourceImage->setChecked(false);
		_ui->groupBox_sourceDatabase->setChecked(false);

		if(_ui->doubleSpinBox_openniFx->value() != 0 ||
			_ui->doubleSpinBox_openniFy->value() != 0 ||
			_ui->doubleSpinBox_openniCx->value() != 0 ||
			_ui->doubleSpinBox_openniCy->value() != 0 )
		{
			int button = QMessageBox::information(this,
					tr("Calibration detected"),
					tr("Some calibration values (fx=%1, fy=%2, cx=%3, cy=%4) are set.\n"
					   "Do you want to reset them to factory defaults?")
					   .arg(_ui->doubleSpinBox_openniFx->value())
					   .arg(_ui->doubleSpinBox_openniFy->value())
					   .arg(_ui->doubleSpinBox_openniCx->value())
					   .arg(_ui->doubleSpinBox_openniCy->value()),
					QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
			if(button & QMessageBox::Yes)
			{
				this->resetCalibration();
			}
		}

		if(validateForm())
		{
			this->writeSettings();
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::openDatabaseViewer()
{
	DatabaseViewer * viewer = new DatabaseViewer(this);
	viewer->setWindowModality(Qt::WindowModal);
	if(viewer->openDatabase(_ui->source_database_lineEdit_path->text()))
	{
		viewer->show();
	}
	else
	{
		delete viewer;
	}
}

void PreferencesDialog::setParameter(const std::string & key, const std::string & value)
{
	UDEBUG("%s=%s", key.c_str(), value.c_str());
	QWidget * obj = _ui->stackedWidget->findChild<QWidget*>(key.c_str());
	if(obj)
	{
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
				if(RTABMAP_NONFREE == 0)
				{
					if(valueInt <= 1 &&
							(combo->objectName().toStdString().compare(Parameters::kKpDetectorStrategy()) == 0 ||
							combo->objectName().toStdString().compare(Parameters::kLccReextractFeatureType()) == 0 ||
							combo->objectName().toStdString().compare(Parameters::kOdomFeatureType()) == 0))
					{
						UWARN("Trying to set \"%s\" to SIFT/SURF but RTAB-Map isn't built "
							  "with the nonfree module from OpenCV. Keeping default combo value: %s.",
							  combo->objectName().toStdString().c_str(),
							  combo->currentText().toStdString().c_str());
					}
					else if(valueInt==1 &&
							(combo->objectName().toStdString().compare(Parameters::kKpNNStrategy()) == 0 ||
							combo->objectName().toStdString().compare(Parameters::kLccReextractNNType()) == 0 ||
							combo->objectName().toStdString().compare(Parameters::kOdomBowNNType()) == 0))

					{
						UWARN("Trying to set \"%s\" to KdTree but RTAB-Map isn't built "
							  "with the nonfree module from OpenCV and kdTree cannot be used "
							  "with binary descriptors. Keeping default combo value: %s.",
							  combo->objectName().toStdString().c_str(),
							  combo->currentText().toStdString().c_str());
					}
					else
					{
						combo->setCurrentIndex(valueInt);
					}
				}
				else
				{
					combo->setCurrentIndex(valueInt);
				}
			}

		}
		else if(check)
		{
			check->setChecked(uStr2Bool(value.c_str()));
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
		// Make sure the value is inserted, check if the same key already exists
		rtabmap::ParametersMap::iterator iter = _parameters.find(object->objectName().toStdString());
		if(iter != _parameters.end())
		{
			_parameters.erase(iter);
		}

		const QComboBox * comboBox = qobject_cast<const QComboBox*>(object);
		const QSpinBox * spinbox = qobject_cast<const QSpinBox*>(object);
		if(comboBox || spinbox)
		{
			if(comboBox)
			{
				// Add related panels to parameters
				if(comboBox == _ui->comboBox_vh_strategy)
				{
					if(value == 0) // 0 none
					{
						// No panel related...
					}
					else if(value == 1) // 2 epipolar
					{
						this->addParameters(_ui->groupBox_vh_epipolar2);
					}
				}
				else if(comboBox == _ui->comboBox_detector_strategy ||
						comboBox == _ui->odom_type ||
						comboBox == _ui->reextract_type)
				{
					if(value == 0) //  surf
					{
						this->addParameters(_ui->groupBox_detector_surf2);
					}
					else if(value == 1) //  sift
					{
						this->addParameters(_ui->groupBox_detector_sift2);
					}
					else if(value == 2) //  orb
					{
						this->addParameters(_ui->groupBox_detector_orb2);
					}
					else if(value == 3) //  fast+freak
					{
						this->addParameters(_ui->groupBox_detector_fast2);
						this->addParameters(_ui->groupBox_detector_freak2);
					}
					else if(value == 4) //  fast+brief
					{
						this->addParameters(_ui->groupBox_detector_fast2);
						this->addParameters(_ui->groupBox_detector_brief2);
					}
					else if(value == 5) //  gftt+freak
					{
						this->addParameters(_ui->groupBox_detector_gftt2);
						this->addParameters(_ui->groupBox_detector_freak2);
					}
					else if(value == 6) //  gftt+brief
					{
						this->addParameters(_ui->groupBox_detector_gftt2);
						this->addParameters(_ui->groupBox_detector_brief2);
					}
					else if(value == 7) //  brisk
					{
						this->addParameters(_ui->groupBox_detector_brisk2);
					}
				}
				else if(comboBox == _ui->globalDetection_icpType)
				{
					if(value == 1) // 1 icp3
					{
						this->addParameters(_ui->groupBox_loopClosure_icp3);
					}
					else if(value == 2) // 2 icp2
					{
						this->addParameters(_ui->groupBox_loopClosure_icp2);
					}
				}
			}
			// Add parameter
			_parameters.insert(rtabmap::ParametersPair(object->objectName().toStdString(), QString::number(value).toStdString()));
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
		// Make sure the value is inserted, check if the same key already exists
		rtabmap::ParametersMap::iterator iter = _parameters.find(object->objectName().toStdString());
		if(iter != _parameters.end())
		{
			_parameters.erase(iter);
		}

		const QCheckBox * checkbox = qobject_cast<const QCheckBox*>(object);
		const QRadioButton * radio = qobject_cast<const QRadioButton*>(object);
		const QGroupBox * groupBox = qobject_cast<const QGroupBox*>(object);
		if(checkbox || radio || groupBox)
		{
			// Add parameter
			_parameters.insert(rtabmap::ParametersPair(object->objectName().toStdString(), uBool2Str(value)));

			// RGBD panel
			if(value && checkbox == _ui->general_checkBox_activateRGBD)
			{
				// add all RGBD parameters!
				this->addParameters(_ui->groupBox_slam_update);
				this->addParameters(_ui->groupBox_graphOptimization);
				this->addParameters(_ui->groupBox_localDetection_time);
				this->addParameters(_ui->groupBox_localDetection_space);
				this->addParameters(_ui->groupBox_globalConstraints);
				this->addParameters(_ui->groupBox_visualTransform2);
			}

			if(groupBox)
			{
				// RGBD panel
				if(value && groupBox == _ui->groupBox_localDetection_time)
				{
					this->addParameters(_ui->groupBox_globalConstraints);
					this->addParameters(_ui->groupBox_visualTransform2);
				}
				if(value && groupBox == _ui->groupBox_localDetection_space)
				{
					this->addParameters(_ui->groupBox_loopClosure_icp2);
				}

				this->addParameters(groupBox);
			}
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
		// Make sure the value is inserted, check if the same key already exists
		rtabmap::ParametersMap::iterator iter = _parameters.find(object->objectName().toStdString());
		if(iter != _parameters.end())
		{
			_parameters.erase(iter);
		}
		_parameters.insert(rtabmap::ParametersPair(object->objectName().toStdString(), QString::number(value).toStdString()));
		//ULOGGER_DEBUG("PreferencesDialog::addParameter(object, double) Added [\"%s\",\"%s\"]", object->objectName().toStdString().c_str(), QString::number(value).toStdString().c_str());
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
		// Make sure the value is inserted, check if the same key already exists
		rtabmap::ParametersMap::iterator iter = _parameters.find(object->objectName().toStdString());
		if(iter != _parameters.end())
		{
			_parameters.erase(iter);
		}
		_parameters.insert(rtabmap::ParametersPair(object->objectName().toStdString(), value.toStdString()));
		//ULOGGER_DEBUG("PreferencesDialog::addParameter(object, QString) Added [\"%s\",\"%s\"]", object->objectName().toStdString().c_str(), QString::number(value).toStdString().c_str());
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

void PreferencesDialog::addParameters(const QStackedWidget * stackedWidget)
{
	if(stackedWidget)
	{
		for(int i=0; i<stackedWidget->count(); ++i)
		{
			const QObjectList & children = stackedWidget->widget(i)->children();
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
	if(sender() == _ui->groupBox_sourceDatabase && _ui->groupBox_sourceDatabase->isChecked())
	{
		_ui->groupBox_sourceImage->setChecked(false);
		_ui->groupBox_sourceOpenni->setChecked(false);
	}
	else if(sender() == _ui->groupBox_sourceImage && _ui->groupBox_sourceImage->isChecked())
	{
		_ui->groupBox_sourceDatabase->setChecked(false);
		_ui->groupBox_sourceOpenni->setChecked(false);
	}
	else if(sender() == _ui->groupBox_sourceOpenni && _ui->groupBox_sourceOpenni->isChecked())
	{
		_ui->groupBox_sourceImage->setChecked(false);
		_ui->groupBox_sourceDatabase->setChecked(false);
	}
	ULOGGER_DEBUG("");
	_obsoletePanels = _obsoletePanels | kPanelSource;
}

rtabmap::ParametersMap PreferencesDialog::getAllParameters()
{
	rtabmap::ParametersMap result;
	rtabmap::ParametersMap tmpParameters = _parameters;
	_parameters.clear();

	QList<QGroupBox*> boxes = this->getGroupBoxes();
	for(int i=0; i<boxes.size(); ++i)
	{
		this->addParameters(boxes.at(i));
	}

	result = _parameters;
	_parameters = tmpParameters;
	return result;
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

void PreferencesDialog::showOpenNI2GroupBox(bool shown)
{
	_ui->groupBox_openni2->setVisible(shown);
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
bool PreferencesDialog::isVerticalLayoutUsed() const
{
	return _ui->checkBox_verticalLayoutUsed->isChecked();
}
bool PreferencesDialog::isImageFlipped() const
{
	return _ui->checkBox_imageFlipped->isChecked();
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
int PreferencesDialog::getKeypointsOpacity() const
{
	return _ui->horizontalSlider_keypointsOpacity->value();
}
int PreferencesDialog::getOdomQualityWarnThr() const
{
	return _ui->spinBox_odomQualityWarnThr->value();
}

bool PreferencesDialog::isCloudsShown(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingShowClouds[index]->isChecked();
}
bool PreferencesDialog::isGraphsShown() const
{
	return _ui->checkBox_showGraphs->isChecked();
}
bool PreferencesDialog::isCloudMeshing() const
{
	return _ui->checkBox_meshing->isChecked();
}
double PreferencesDialog::getCloudVoxelSize(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingVoxelSize[index]->value();
}
int PreferencesDialog::getCloudDecimation(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingDecimation[index]->value();
}
double PreferencesDialog::getCloudMaxDepth(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingMaxDepth[index]->value();
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
int PreferencesDialog::getMeshNormalKSearch() const
{
	return _ui->spinBox_normalKSearch->value();
}
double PreferencesDialog::getMeshGP3Radius() const
{
	return _ui->doubleSpinBox_gp3Radius->value();
}
bool PreferencesDialog::getMeshSmoothing() const
{
	return _ui->checkBox_mls->isChecked();
}
double PreferencesDialog::getMeshSmoothingRadius() const
{
	return _ui->doubleSpinBox_mlsRadius->value();
}
bool PreferencesDialog::isCloudFiltering() const
{
	return _ui->groupBox_poseFiltering->isChecked();
}
double PreferencesDialog::getCloudFilteringRadius() const
{
	return _ui->doubleSpinBox_cloudFilterRadius->value();
}
double PreferencesDialog::getCloudFilteringAngle() const
{
	return _ui->doubleSpinBox_cloudFilterAngle->value();
}
bool PreferencesDialog::getGridMapShown() const
{
	return _ui->checkBox_map_shown->isChecked();
}
double PreferencesDialog::getGridMapResolution() const
{
	return _ui->doubleSpinBox_map_resolution->value();
}
bool PreferencesDialog::getGridMapFillEmptySpace() const
{
	return _ui->checkBox_map_fillEmptySpace->isChecked();
}
bool PreferencesDialog::isGridMapFrom3DCloud() const
{
	return _ui->checkBox_map_occupancyFrom3DCloud->isChecked();
}
int PreferencesDialog::getGridMapFillEmptyRadius() const
{
	return _ui->spinbox_map_fillEmptyRadius->value();
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
bool PreferencesDialog::isSourceImageUsed() const
{
	return _ui->groupBox_sourceImage->isChecked();
}
bool PreferencesDialog::isSourceDatabaseUsed() const
{
	return _ui->groupBox_sourceDatabase->isChecked();
}
bool PreferencesDialog::isSourceOpenniUsed() const
{
	return _ui->groupBox_sourceOpenni->isChecked();
}


int PreferencesDialog::getSourceImageType() const
{
	return _ui->source_comboBox_image_type->currentIndex();
}
QString PreferencesDialog::getSourceImageTypeStr() const
{
	return _ui->source_comboBox_image_type->currentText();
}
int PreferencesDialog::getSourceWidth() const
{
	return _ui->source_spinBox_imgWidth->value();
}
int PreferencesDialog::getSourceHeight() const
{
	return _ui->source_spinBox_imgheight->value();
}
QString PreferencesDialog::getSourceImagesPath() const
{
	return _ui->source_images_lineEdit_path->text();
}
int PreferencesDialog::getSourceImagesStartPos() const
{
	return _ui->source_images_spinBox_startPos->value();
}
bool PreferencesDialog::getSourceImagesRefreshDir() const
{
	return _ui->source_images_refreshDir->isChecked();
}
QString PreferencesDialog::getSourceVideoPath() const
{
	return _ui->source_video_lineEdit_path->text();
}
int PreferencesDialog::getSourceUsbDeviceId() const
{
	return _ui->source_usbDevice_spinBox_id->value();
}
QString PreferencesDialog::getSourceDatabasePath() const
{
	return _ui->source_database_lineEdit_path->text();
}
bool PreferencesDialog::getSourceDatabaseOdometryIgnored() const
{
	return _ui->source_checkBox_ignoreOdometry->isChecked();
}
int PreferencesDialog::getSourceDatabaseStartPos() const
{
	return _ui->source_spinBox_databaseStartPos->value();
}

PreferencesDialog::Src PreferencesDialog::getSourceRGBD() const
{
	if(_ui->radioButton_freenect->isChecked())
	{
		return kSrcFreenect;
	}
	else if(_ui->radioButton_opennicv->isChecked())
	{
		return kSrcOpenNI_CV;
	}
	else if (_ui->radioButton_opennicvasus->isChecked())
	{
		return kSrcOpenNI_CV_ASUS;
	}
	else if (_ui->radioButton_openni2->isChecked())
	{
		return kSrcOpenNI2;
	}
	else
	{
		return kSrcOpenNI_PCL;
	}
}
bool PreferencesDialog::getSourceOpenni2AutoWhiteBalance() const
{
	return _ui->openni2_autoWhiteBalance->isChecked();
}
bool PreferencesDialog::getSourceOpenni2AutoExposure() const
{
	return _ui->openni2_autoExposure->isChecked();
}
int PreferencesDialog::getSourceOpenni2Exposure() const
{
	return _ui->openni2_exposure->value();
}
int PreferencesDialog::getSourceOpenni2Gain() const
{
	return _ui->openni2_gain->value();
}
QString PreferencesDialog::getSourceOpenniDevice() const
{
	return _ui->lineEdit_openniDevice->text();
}
Transform PreferencesDialog::getSourceOpenniLocalTransform() const
{
	Transform t = Transform::getIdentity();
	QString str = _ui->lineEdit_openniLocalTransform->text();
	str.replace("PI_2", QString::number(3.141592/2.0));
	QStringList list = str.split(' ');
	if(list.size() != 6)
	{
		UERROR("Local transform is wrong! must have 6 items (%s)", str.toStdString().c_str());
	}
	else
	{
		std::vector<float> numbers(6);
		bool ok = false;
		for(int i=0; i<list.size(); ++i)
		{
			numbers[i] = list.at(i).toDouble(&ok);
			if(!ok)
			{
				UERROR("Parsing local transform failed! \"%s\" not recognized (%s)",
						list.at(i).toStdString().c_str(), str.toStdString().c_str());
				break;
			}
		}
		if(ok)
		{
			t = Transform(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]);
		}
	}
	return t;
}

float PreferencesDialog::getSourceOpenniFx() const
{
	return _ui->doubleSpinBox_openniFx->value();
}
float PreferencesDialog::getSourceOpenniFy() const
{
	return _ui->doubleSpinBox_openniFy->value();
}
float PreferencesDialog::getSourceOpenniCx() const
{
	return _ui->doubleSpinBox_openniCx->value();
}
float PreferencesDialog::getSourceOpenniCy() const
{
	return _ui->doubleSpinBox_openniCy->value();
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
int PreferencesDialog::getOdomStrategy() const
{
	return _ui->odom_strategy->currentIndex();
}

bool PreferencesDialog::isImagesKept() const
{
	return _ui->general_checkBox_imagesKept->isChecked();
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

/*** SETTERS ***/
void PreferencesDialog::setHardThr(int value)
{
	double dValue = double(value)/100;
	ULOGGER_DEBUG("PreferencesDialog::setHardThr(%f)", dValue);
	if(_ui->general_doubleSpinBox_hardThr->value() != dValue)
	{
		_ui->general_doubleSpinBox_hardThr->setValue(dValue);
		if(validateForm())
		{
			this->writeSettings();
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::setInputRate(double value)
{
	ULOGGER_DEBUG("imgRate=%2.2f", value);
	if(_ui->general_doubleSpinBox_imgRate->value() != value)
	{
		_ui->general_doubleSpinBox_imgRate->setValue(value);
		if(validateForm())
		{
			this->writeSettings();
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
			this->writeSettings();
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
			this->writeSettings();
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
			this->writeSettings();
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::testOdometry()
{
	testOdometry(_ui->odom_type->currentIndex());
}

void PreferencesDialog::testOdometry(int type)
{
	if(_cameraThread)
	{
		QMessageBox::warning(this,
			   tr("RTAB-Map"),
			   tr("A camera is already running!"));
		return;
	}

	UASSERT(_odomThread == 0 && _cameraThread == 0);

	CameraRGBD * camera = 0;
	if(this->getSourceRGBD() == kSrcOpenNI_PCL)
	{
		camera = new CameraOpenni(
			this->getSourceOpenniDevice().toStdString(),
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());
	}
	else if(this->getSourceRGBD() == kSrcOpenNI2)
	{
		camera = new CameraOpenNI2(
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());

	}
	else if(this->getSourceRGBD() == kSrcFreenect)
	{
		camera = new CameraFreenect(
			this->getSourceOpenniDevice().isEmpty()?0:atoi(this->getSourceOpenniDevice().toStdString().c_str()),
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());
	}
	else if(this->getSourceRGBD() == kSrcOpenNI_CV || this->getSourceRGBD() == kSrcOpenNI_CV_ASUS)
	{
		camera = new CameraOpenNICV(
			this->getSourceRGBD() == kSrcOpenNI_CV_ASUS,
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());
	}
	else
	{
		UFATAL("RGBD Source type undefined!");
	}

	if(!camera->init())
	{
		QMessageBox::warning(this,
			   tr("RTAB-Map"),
			   tr("RGBD camera initialization failed!"));
		delete camera;
		camera = 0;
	}
	else if(this->getSourceRGBD() == kSrcOpenNI2)
	{
		((CameraOpenNI2*)camera)->setAutoWhiteBalance(getSourceOpenni2AutoWhiteBalance());
		((CameraOpenNI2*)camera)->setAutoExposure(getSourceOpenni2AutoExposure());
		if(CameraOpenNI2::exposureGainAvailable())
		{
			((CameraOpenNI2*)camera)->setExposure(getSourceOpenni2Exposure());
			((CameraOpenNI2*)camera)->setGain(getSourceOpenni2Gain());
		}
	}


	if(camera)
	{
		ParametersMap parameters = this->getAllParameters();
		Odometry * odometry;
		if(this->getOdomStrategy() == 1)
		{
			odometry = new OdometryOpticalFlow(parameters);
		}
		else
		{
			odometry = new OdometryBOW(parameters);
		}

		_odomThread = new OdometryThread(odometry); // take ownership of odometry

		QDialog * window = new QDialog(this);
		window->setWindowModality(Qt::WindowModal);
		window->setWindowTitle(tr("Odometry viewer"));
		window->setMinimumWidth(800);
		window->setMinimumHeight(600);
		connect( window, SIGNAL(finished(int)), this, SLOT(cleanOdometryTest()) );

		OdometryViewer * odomViewer = new OdometryViewer(10,
				_ui->spinBox_decimation_odom->value(),
				_ui->doubleSpinBox_voxelSize_odom->value(),
				this->getOdomQualityWarnThr(),
				window);
		connect( window, SIGNAL(finished(int)), odomViewer, SLOT(clear()) );

		odomViewer->setCameraFree();
		odomViewer->setGridShown(true);

		QVBoxLayout *layout = new QVBoxLayout();
		layout->addWidget(odomViewer);
		window->setLayout(layout);

		UEventsManager::addHandler(_odomThread);
		UEventsManager::addHandler(odomViewer);

		_cameraThread = new CameraThread(camera);
		UEventsManager::createPipe(_cameraThread, _odomThread, "CameraEvent");
		UEventsManager::createPipe(_odomThread, odomViewer, "OdometryEvent");

		window->showNormal();

		_ui->pushButton_testOdometry->setEnabled(false);

		QApplication::processEvents();
		uSleep(500);
		QApplication::processEvents();

		_odomThread->start();
		_cameraThread->start();
	}
}

void PreferencesDialog::cleanOdometryTest()
{
	UDEBUG("");
	if(_cameraThread)
	{
		_cameraThread->join(true);
		delete _cameraThread;
		_cameraThread = 0;
	}
	if(_odomThread)
	{
		_odomThread->join(true);
		delete _odomThread;
		_odomThread = 0;
	}
	_ui->pushButton_testOdometry->setEnabled(true);
}

void PreferencesDialog::cleanRGBDCameraTest()
{
	UDEBUG("");
	if(_cameraThread)
	{
		_cameraThread->join(true);
		delete _cameraThread;
		_cameraThread = 0;
	}
	_ui->pushButton_test_rgbd_camera->setEnabled(true);
}

void PreferencesDialog::testRGBDCamera()
{
	if(_cameraThread)
	{
		QMessageBox::warning(this,
			   tr("RTAB-Map"),
			   tr("A camera is already running!"));
	}

	CameraRGBD * camera = 0;
	if(this->getSourceRGBD() == kSrcOpenNI_PCL)
	{
		camera = new CameraOpenni(
			this->getSourceOpenniDevice().toStdString(),
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());
	}
	else if(this->getSourceRGBD() == kSrcOpenNI2)
	{
		camera = new CameraOpenNI2(
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());

	}
	else if(this->getSourceRGBD() == kSrcFreenect)
	{
		camera = new CameraFreenect(
			this->getSourceOpenniDevice().isEmpty()?0:atoi(this->getSourceOpenniDevice().toStdString().c_str()),
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());
	}
	else if(this->getSourceRGBD() == kSrcOpenNI_CV || this->getSourceRGBD() == kSrcOpenNI_CV_ASUS)
	{
		camera = new CameraOpenNICV(
			this->getSourceRGBD() == kSrcOpenNI_CV_ASUS,
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());
	}
	else
	{
		UFATAL("RGBD Source type undefined!");
	}

	if(!camera->init())
	{
		QMessageBox::warning(this,
			   tr("RTAB-Map"),
			   tr("RGBD camera initialization failed!"));
		delete camera;
		camera = 0;
	}
	else if(this->getSourceRGBD() == kSrcOpenNI2)
	{
		((CameraOpenNI2*)camera)->setAutoWhiteBalance(getSourceOpenni2AutoWhiteBalance());
		((CameraOpenNI2*)camera)->setAutoExposure(getSourceOpenni2AutoExposure());
		if(CameraOpenNI2::exposureGainAvailable())
		{
			((CameraOpenNI2*)camera)->setExposure(getSourceOpenni2Exposure());
			((CameraOpenNI2*)camera)->setGain(getSourceOpenni2Gain());
		}
	}


	if(camera)
	{
		_ui->pushButton_test_rgbd_camera->setEnabled(false);

		// Create DataRecorder without init it, just to show images...
		DataRecorder * window = new DataRecorder(this);
		window->setWindowModality(Qt::WindowModal);
		window->setAttribute(Qt::WA_DeleteOnClose);
		window->setWindowFlags(Qt::Dialog);
		window->setWindowTitle(tr("RGBD camera viewer"));
		window->setMinimumWidth(800);
		window->setMinimumHeight(600);
		connect( window, SIGNAL(destroyed(QObject*)), this, SLOT(cleanRGBDCameraTest()) );
		window->registerToEventsManager();

		_cameraThread = new CameraThread(camera);
		UEventsManager::createPipe(_cameraThread, window, "CameraEvent");

		window->showNormal();

		_cameraThread->start();
	}
}

void PreferencesDialog::calibrate()
{
	CameraRGBD * camera = 0;
	if(this->getSourceRGBD() == kSrcOpenNI_PCL)
	{
		camera = new CameraOpenni(
			this->getSourceOpenniDevice().toStdString(),
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());
	}
	else if(this->getSourceRGBD() == kSrcOpenNI2)
	{
		camera = new CameraOpenNI2(
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());

	}
	else if(this->getSourceRGBD() == kSrcFreenect)
	{
		camera = new CameraFreenect(
			this->getSourceOpenniDevice().isEmpty()?0:atoi(this->getSourceOpenniDevice().toStdString().c_str()),
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());
	}
	else if(this->getSourceRGBD() == kSrcOpenNI_CV || this->getSourceRGBD() == kSrcOpenNI_CV_ASUS)
	{
		camera = new CameraOpenNICV(
			this->getSourceRGBD() == kSrcOpenNI_CV_ASUS,
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform(),
			this->getSourceOpenniFx(),
			this->getSourceOpenniFy(),
			this->getSourceOpenniCx(),
			this->getSourceOpenniCy());
	}
	else
	{
		UFATAL("RGBD Source type undefined!");
	}

	if(!camera->init())
	{
		QMessageBox::warning(this,
			   tr("RTAB-Map"),
			   tr("RGBD camera initialization failed!"));
		delete camera;
		camera = 0;
	}
	else if(this->getSourceRGBD() == kSrcOpenNI2)
	{
		((CameraOpenNI2*)camera)->setAutoWhiteBalance(getSourceOpenni2AutoWhiteBalance());
		((CameraOpenNI2*)camera)->setAutoExposure(getSourceOpenni2AutoExposure());
		if(CameraOpenNI2::exposureGainAvailable())
		{
			((CameraOpenNI2*)camera)->setExposure(getSourceOpenni2Exposure());
			((CameraOpenNI2*)camera)->setGain(getSourceOpenni2Gain());
		}
	}


	if(camera)
	{
		CalibrationDialog * dialog = new CalibrationDialog(this);
		dialog->registerToEventsManager();

		CameraThread cameraThread(camera);
		UEventsManager::createPipe(&cameraThread, dialog, "CameraEvent");

		cameraThread.start();

		if(dialog->exec() == QDialog::Accepted)
		{
			_ui->doubleSpinBox_openniFx->setValue(dialog->fx());
			_ui->doubleSpinBox_openniFy->setValue(dialog->fy());
			_ui->doubleSpinBox_openniCx->setValue(dialog->cx());
			_ui->doubleSpinBox_openniCy->setValue(dialog->cy());
		}

		cameraThread.join(true);
		delete dialog;
	}
}

void PreferencesDialog::resetCalibration()
{
	_ui->doubleSpinBox_openniFx->setValue(0);
	_ui->doubleSpinBox_openniFy->setValue(0);
	_ui->doubleSpinBox_openniCx->setValue(0);
	_ui->doubleSpinBox_openniCy->setValue(0);
}

}
