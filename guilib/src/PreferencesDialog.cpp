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

#include "rtabmap/gui/PreferencesDialog.h"
#include "rtabmap/gui/DatabaseViewer.h"
#include "rtabmap/gui/OdometryViewer.h"

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

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/CameraOpenni.h"
#include "rtabmap/core/Memory.h"

#include "rtabmap/gui/LoopClosureViewer.h"

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
	_odomCamera(0),
	_odomThread(0)
{
	ULOGGER_DEBUG("");

	_ui = new Ui_preferencesDialog();
	_ui->setupUi(this);

	if(cv::gpu::getCudaEnabledDeviceCount() == 0)
	{
		_ui->surf_checkBox_gpuVersion->setEnabled(false);
		_ui->label_surf_checkBox_gpuVersion->setEnabled(false);
	}

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

	// General panel
	connect(_ui->general_checkBox_imagesKept, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_verticalLayoutUsed, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_imageFlipped, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_imageRejectedShown, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_imageHighestHypShown, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_beep, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->horizontalSlider_keypointsOpacity, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));

	// Cloud rendering panel
	_3dRenderingShowClouds.resize(3);
	_3dRenderingShowClouds[0] = _ui->checkBox_showClouds;
	_3dRenderingShowClouds[1] = _ui->checkBox_showOdomClouds;
	_3dRenderingShowClouds[2] = _ui->checkBox_showSaveClouds;

	_3dRenderingVoxelSize.resize(3);
	_3dRenderingVoxelSize[0] = _ui->doubleSpinBox_voxelSize;
	_3dRenderingVoxelSize[1] = _ui->doubleSpinBox_voxelSize_odom;
	_3dRenderingVoxelSize[2] = _ui->doubleSpinBox_voxelSize_save;

	_3dRenderingDecimation.resize(3);
	_3dRenderingDecimation[0] = _ui->spinBox_decimation;
	_3dRenderingDecimation[1] = _ui->spinBox_decimation_odom;
	_3dRenderingDecimation[2] = _ui->spinBox_decimation_save;

	_3dRenderingMaxDepth.resize(3);
	_3dRenderingMaxDepth[0] = _ui->doubleSpinBox_maxDepth;
	_3dRenderingMaxDepth[1] = _ui->doubleSpinBox_maxDepth_odom;
	_3dRenderingMaxDepth[2] = _ui->doubleSpinBox_maxDepth_save;

	_3dRenderingOpacity.resize(2);
	_3dRenderingOpacity[0] = _ui->doubleSpinBox_opacity;
	_3dRenderingOpacity[1] = _ui->doubleSpinBox_opacity_odom;

	_3dRenderingPtSize.resize(2);
	_3dRenderingPtSize[0] = _ui->spinBox_ptsize;
	_3dRenderingPtSize[1] = _ui->spinBox_ptsize_odom;

	_3dRenderingShowScans.resize(3);
	_3dRenderingShowScans[0] = _ui->checkBox_showScans;
	_3dRenderingShowScans[1] = _ui->checkBox_showOdomScans;
	_3dRenderingShowScans[2] = _ui->checkBox_showSaveScans;

	_3dRenderingOpacityScan.resize(2);
	_3dRenderingOpacityScan[0] = _ui->doubleSpinBox_opacity_scan;
	_3dRenderingOpacityScan[1] = _ui->doubleSpinBox_opacity_odom_scan;

	_3dRenderingPtSizeScan.resize(2);
	_3dRenderingPtSizeScan[0] = _ui->spinBox_ptsize_scan;
	_3dRenderingPtSizeScan[1] = _ui->spinBox_ptsize_odom_scan;

	_3dRenderingMeshing.resize(1);
	_3dRenderingMeshing[0] = _ui->checkBox_meshing;

	for(int i=0; i<3; ++i)
	{
		connect(_3dRenderingShowClouds[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingVoxelSize[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingDecimation[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingMaxDepth[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingShowScans[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

		if(i<2)
		{
			connect(_3dRenderingOpacity[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
			connect(_3dRenderingPtSize[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
			connect(_3dRenderingOpacityScan[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
			connect(_3dRenderingPtSizeScan[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		}

		if(i<1)
		{
			connect(_3dRenderingMeshing[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		}
	}

	connect(_ui->groupBox_poseFiltering, SIGNAL(clicked(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_cloudFilterRadius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_cloudFilterAngle, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));

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
	connect(_ui->general_checkBox_autoRestart, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->general_checkBox_cameraKeypoints, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//usbDevice group
	connect(_ui->source_usbDevice_spinBox_id, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_imgWidth, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_imgheight, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_framesDropped, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
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
	connect(_ui->lineEdit_openniDevice, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_openniLocalTransform, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));

	//Rtabmap basic
	connect(_ui->general_doubleSpinBox_timeThr, SIGNAL(valueChanged(double)), _ui->general_doubleSpinBox_timeThr_2, SLOT(setValue(double)));
	connect(_ui->general_doubleSpinBox_hardThr, SIGNAL(valueChanged(double)), _ui->general_doubleSpinBox_hardThr_2, SLOT(setValue(double)));
	connect(_ui->doubleSpinBox_similarityThreshold, SIGNAL(valueChanged(double)), _ui->doubleSpinBox_similarityThreshold_2, SLOT(setValue(double)));
	connect(_ui->general_doubleSpinBox_detectionRate, SIGNAL(valueChanged(double)), _ui->general_doubleSpinBox_detectionRate_2, SLOT(setValue(double)));
	connect(_ui->general_spinBox_imagesBufferSize, SIGNAL(valueChanged(int)), _ui->general_spinBox_imagesBufferSize_2, SLOT(setValue(int)));
	connect(_ui->general_spinBox_maxStMemSize, SIGNAL(valueChanged(int)), _ui->general_spinBox_maxStMemSize_2, SLOT(setValue(int)));

	connect(_ui->lineEdit_databasePath, SIGNAL(textChanged(const QString &)), _ui->lineEdit_databasePath_2, SLOT(setText(const QString &)));

	connect(_ui->general_doubleSpinBox_timeThr_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_doubleSpinBox_hardThr_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->doubleSpinBox_similarityThreshold_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_doubleSpinBox_detectionRate_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_spinBox_imagesBufferSize_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_spinBox_maxStMemSize_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_publishStats, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_publishStats_2, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_activateRGBD, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_activateRGBD_2, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_SLAM_mode, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_SLAM_mode_2, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));

	connect(_ui->lineEdit_databasePath_2, SIGNAL(textChanged(const QString &)), _ui->lineEdit_databasePath, SLOT(setText(const QString &)));
	connect(_ui->toolButton_databasePath_2, SIGNAL(clicked()), this, SLOT(changeDatabasePath()));

	// Map objects name with the corresponding parameter key, needed for the addParameter() slots
	//Rtabmap
	_ui->general_checkBox_publishStats->setObjectName(Parameters::kRtabmapPublishStats().c_str());
	_ui->general_checkBox_publishRawData->setObjectName(Parameters::kRtabmapPublishImage().c_str());
	_ui->general_checkBox_publishPdf->setObjectName(Parameters::kRtabmapPublishPdf().c_str());
	_ui->general_checkBox_publishLikelihood->setObjectName(Parameters::kRtabmapPublishLikelihood().c_str());
	_ui->general_checkBox_statisticLogsBufferedInRAM->setObjectName(Parameters::kRtabmapStatisticLogsBufferedInRAM().c_str());
	_ui->general_checkBox_statisticLogged->setObjectName(Parameters::kRtabmapStatisticLogged().c_str());
	_ui->general_doubleSpinBox_timeThr->setObjectName(Parameters::kRtabmapTimeThr().c_str());
	_ui->general_spinBox_memoryThr->setObjectName(Parameters::kRtabmapMemoryThr().c_str());
	_ui->general_doubleSpinBox_detectionRate->setObjectName(Parameters::kRtabmapDetectionRate().c_str());
	_ui->general_spinBox_imagesBufferSize->setObjectName(Parameters::kRtabmapImageBufferSize().c_str());
	_ui->general_spinBox_maxRetrieved->setObjectName(Parameters::kRtabmapMaxRetrieved().c_str());
	_ui->lineEdit_databasePath->setObjectName(Parameters::kRtabmapDatabasePath().c_str());
	_ui->lineEdit_workingDirectory->setObjectName(Parameters::kRtabmapWorkingDirectory().c_str());
	connect(_ui->toolButton_databasePath, SIGNAL(clicked()), this, SLOT(changeDatabasePath()));
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
	_ui->checkBox_kp_publishKeypoints->setObjectName(Parameters::kKpPublishKeypoints().c_str());
	_ui->comboBox_dictionary_strategy->setObjectName(Parameters::kKpNNStrategy().c_str());
	_ui->checkBox_dictionary_incremental->setObjectName(Parameters::kKpIncrementalDictionary().c_str());
	_ui->comboBox_detector_strategy->setObjectName(Parameters::kKpDetectorStrategy().c_str());
	_ui->checkBox_dictionary_minDistUsed->setObjectName(Parameters::kKpMinDistUsed().c_str());
	_ui->surf_doubleSpinBox_matchThr->setObjectName(Parameters::kKpMinDist().c_str());
	_ui->checkBox_dictionary_nndrUsed->setObjectName(Parameters::kKpNndrUsed().c_str());
	_ui->surf_doubleSpinBox_nndrRatio->setObjectName(Parameters::kKpNndrRatio().c_str());
	_ui->surf_spinBox_maxLeafs->setObjectName(Parameters::kKpMaxLeafs().c_str());
	_ui->surf_doubleSpinBox_maxDepth->setObjectName(Parameters::kKpMaxDepth().c_str());
	_ui->surf_spinBox_wordsPerImageTarget->setObjectName(Parameters::kKpWordsPerImage().c_str());
	_ui->surf_doubleSpinBox_ratioBadSign->setObjectName(Parameters::kKpBadSignRatio().c_str());
	_ui->checkBox_kp_tfIdfLikelihoodUsed->setObjectName(Parameters::kKpTfIdfLikelihoodUsed().c_str());
	_ui->checkBox_kp_parallelized->setObjectName(Parameters::kKpParallelized().c_str());
	_ui->lineEdit_kp_roi->setObjectName(Parameters::kKpRoiRatios().c_str());
	_ui->lineEdit_dictionaryPath->setObjectName(Parameters::kKpDictionaryPath().c_str());
	connect(_ui->toolButton_dictionaryPath, SIGNAL(clicked()), this, SLOT(changeDictionaryPath()));

	//SURF detector
	_ui->surf_doubleSpinBox_hessianThr->setObjectName(Parameters::kSURFHessianThreshold().c_str());
	_ui->surf_spinBox_octaves->setObjectName(Parameters::kSURFOctaves().c_str());
	_ui->surf_spinBox_octaveLayers->setObjectName(Parameters::kSURFOctaveLayers().c_str());
	_ui->checkBox_surfExtended->setObjectName(Parameters::kSURFExtended().c_str());
	_ui->surf_checkBox_upright->setObjectName(Parameters::kSURFUpright().c_str());
	_ui->surf_checkBox_gpuVersion->setObjectName(Parameters::kSURFGpuVersion().c_str());

	//SIFT detector
	_ui->sift_spinBox_nFeatures->setObjectName(Parameters::kSIFTNFeatures().c_str());
	_ui->sift_spinBox_nOctaveLayers->setObjectName(Parameters::kSIFTNOctaveLayers().c_str());
	_ui->sift_doubleSpinBox_contrastThr->setObjectName(Parameters::kSIFTContrastThreshold().c_str());
	_ui->sift_doubleSpinBox_edgeThr->setObjectName(Parameters::kSIFTEdgeThreshold().c_str());
	_ui->sift_doubleSpinBox_sigma->setObjectName(Parameters::kSIFTSigma().c_str());

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
	_ui->odomScanHistory->setObjectName(Parameters::kRGBDScanMatchingSize().c_str());
	_ui->globalDetection_toroIterations->setObjectName(Parameters::kRGBDToroIterations().c_str());

	_ui->groupBox_localDetection_time->setObjectName(Parameters::kRGBDLocalLoopDetectionTime().c_str());
	_ui->groupBox_localDetection_space->setObjectName(Parameters::kRGBDLocalLoopDetectionSpace().c_str());
	_ui->localDetection_radius->setObjectName(Parameters::kRGBDLocalLoopDetectionRadius().c_str());
	_ui->localDetection_maxNeighbors->setObjectName(Parameters::kRGBDLocalLoopDetectionNeighbors().c_str());
	_ui->localDetection_maxDiffID->setObjectName(Parameters::kRGBDLocalLoopDetectionMaxDiffID().c_str());

	_ui->loopClosure_bowMinInliers->setObjectName(Parameters::kLccBowMinInliers().c_str());
	_ui->loopClosure_bowInlierDistance->setObjectName(Parameters::kLccBowInlierDistance().c_str());
	_ui->loopClosure_bowIterations->setObjectName(Parameters::kLccBowIterations().c_str());
	_ui->loopClosure_bowMaxDepth->setObjectName(Parameters::kLccBowMaxDepth().c_str());

	_ui->globalDetection_icpType->setObjectName(Parameters::kLccIcpType().c_str());
	_ui->globalDetection_icpMaxDistance->setObjectName(Parameters::kLccIcpMaxDistance().c_str());

	_ui->loopClosure_icpDecimation->setObjectName(Parameters::kLccIcp3Decimation().c_str());
	_ui->loopClosure_icpMaxDepth->setObjectName(Parameters::kLccIcp3MaxDepth().c_str());
	_ui->loopClosure_icpVoxelSize->setObjectName(Parameters::kLccIcp3VoxelSize().c_str());
	_ui->loopClosure_icpSamples->setObjectName(Parameters::kLccIcp3Samples().c_str());
	_ui->loopClosure_icpMaxCorrespondenceDistance->setObjectName(Parameters::kLccIcp3MaxCorrespondenceDistance().c_str());
	_ui->loopClosure_icpIterations->setObjectName(Parameters::kLccIcp3Iterations().c_str());
	_ui->loopClosure_icpMaxFitness->setObjectName(Parameters::kLccIcp3MaxFitness().c_str());

	_ui->loopClosure_icp2MaxCorrespondenceDistance->setObjectName(Parameters::kLccIcp2MaxCorrespondenceDistance().c_str());
	_ui->loopClosure_icp2Iterations->setObjectName(Parameters::kLccIcp2Iterations().c_str());
	_ui->loopClosure_icp2MaxFitness->setObjectName(Parameters::kLccIcp2MaxFitness().c_str());
	_ui->loopClosure_icp2Ratio->setObjectName(Parameters::kLccIcp2CorrespondenceRatio().c_str());
	_ui->loopClosure_icp2Voxel->setObjectName(Parameters::kLccIcp2VoxelSize().c_str());

	//Odometry
	_ui->odom_type->setObjectName(Parameters::kOdomType().c_str());
	_ui->odom_linearUpdate->setObjectName(Parameters::kOdomLinearUpdate().c_str());
	_ui->odom_angularUpdate->setObjectName(Parameters::kOdomAngularUpdate().c_str());
	_ui->odom_countdown->setObjectName(Parameters::kOdomResetCountdown().c_str());
	_ui->odom_maxFeatures->setObjectName(Parameters::kOdomMaxWords().c_str());
	_ui->odom_inlierDistance->setObjectName(Parameters::kOdomInlierDistance().c_str());
	_ui->odom_iterations->setObjectName(Parameters::kOdomIterations().c_str());
	_ui->odom_maxDepth->setObjectName(Parameters::kOdomMaxDepth().c_str());
	_ui->odom_minInliers->setObjectName(Parameters::kOdomMinInliers().c_str());
	_ui->stackedWidget_odom->setCurrentIndex(_ui->odom_type->currentIndex());
	connect(_ui->odom_type, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_odom, SLOT(setCurrentIndex(int)));

	_ui->odom_bin_briefBytes->setObjectName(Parameters::kOdomBinBriefBytes().c_str());
	_ui->odom_bin_fastSuppressNonMax->setObjectName(Parameters::kOdomBinFastNonmaxSuppression().c_str());
	_ui->odom_bin_fastThreshold->setObjectName(Parameters::kOdomBinFastThreshold().c_str());
	_ui->odom_bin_bruteForceMatching->setObjectName(Parameters::kOdomBinBruteForceMatching().c_str());

	_ui->odom_icpDecimation->setObjectName(Parameters::kOdomICPDecimation().c_str());
	_ui->odom_icpVoxelSize->setObjectName(Parameters::kOdomICPVoxelSize().c_str());
	_ui->odom_icpSamples->setObjectName(Parameters::kOdomICPSamples().c_str());
	_ui->odom_icpMaxCorrespondenceDistance->setObjectName(Parameters::kOdomICPCorrespondencesDistance().c_str());
	_ui->odom_icpIterations->setObjectName(Parameters::kOdomICPIterations().c_str());
	_ui->odom_icpMaxFitness->setObjectName(Parameters::kOdomICPMaxFitness().c_str());

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
		_ui->checkBox_verticalLayoutUsed->setChecked(false);
		_ui->checkBox_imageFlipped->setChecked(false);
		_ui->checkBox_imageRejectedShown->setChecked(true);
		_ui->checkBox_imageHighestHypShown->setChecked(false);
		_ui->horizontalSlider_keypointsOpacity->setSliderPosition(20);
	}
	else if(groupBox->objectName() == _ui->groupBox_cloudRendering1->objectName())
	{
		for(int i=0; i<3; ++i)
		{
			_3dRenderingShowClouds[i]->setChecked(true);
			_3dRenderingVoxelSize[i]->setValue(i==2?0.01:0.00);
			_3dRenderingDecimation[i]->setValue(i==0?4:i==1?2:1);
			_3dRenderingMaxDepth[i]->setValue(i==1?0.0:4.0);
			_3dRenderingShowScans[i]->setChecked(true);

			if(i<2)
			{
				_3dRenderingOpacity[i]->setValue(i==0?0.6:1.0);
				_3dRenderingPtSize[i]->setValue(i==0?1:2);
				_3dRenderingOpacityScan[i]->setValue(1.0);
				_3dRenderingPtSizeScan[i]->setValue(1);
			}

			if(i<1)
			{
				_3dRenderingMeshing[i]->setChecked(false);
			}
		}

		_ui->groupBox_poseFiltering->setChecked(false);
		_ui->doubleSpinBox_cloudFilterRadius->setValue(0.5);
		_ui->doubleSpinBox_cloudFilterAngle->setValue(30);
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
		_ui->general_doubleSpinBox_imgRate->setValue(0.0);

		_ui->groupBox_sourceImage->setChecked(false);
		_ui->source_spinBox_imgWidth->setValue(0);
		_ui->source_spinBox_imgheight->setValue(0);
		_ui->source_spinBox_framesDropped->setValue(0);
		_ui->general_checkBox_autoRestart->setChecked(false);
		_ui->general_checkBox_cameraKeypoints->setChecked(false);
		_ui->source_images_spinBox_startPos->setValue(1);
		_ui->source_images_refreshDir->setChecked(false);

		_ui->groupBox_sourceDatabase->setChecked(false);
		_ui->source_checkBox_ignoreOdometry->setChecked(false);
		_ui->source_spinBox_databaseStartPos->setValue(0);

		_ui->groupBox_sourceOpenni->setChecked(true);
		_ui->lineEdit_openniDevice->setText("#1");
		_ui->lineEdit_openniLocalTransform->setText("0 0 0 -PI_2 0 -PI_2");
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
		_ui->lineEdit_databasePath_2->setText(Parameters::defaultRtabmapDatabasePath().c_str());
		_ui->general_doubleSpinBox_detectionRate_2->setValue(Parameters::defaultRtabmapDetectionRate());
		// match the advanced (spin and doubleSpin boxes)
		_ui->general_doubleSpinBox_timeThr->setValue(Parameters::defaultRtabmapTimeThr());
		_ui->general_doubleSpinBox_hardThr->setValue(Parameters::defaultRtabmapLoopThr());
		_ui->surf_doubleSpinBox_hessianThr->setValue(Parameters::defaultSURFHessianThreshold());
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

QString PreferencesDialog::getDatabasePath() const
{
	return _ui->lineEdit_databasePath->text();
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

	for(int i=0; i<3; ++i)
	{
		_3dRenderingShowClouds[i]->setChecked(settings.value(tr("showClouds%1").arg(i), _3dRenderingShowClouds[i]->isChecked()).toBool());
		_3dRenderingVoxelSize[i]->setValue(settings.value(tr("voxelSize%1").arg(i), _3dRenderingVoxelSize[i]->value()).toDouble());
		_3dRenderingDecimation[i]->setValue(settings.value(tr("decimation%1").arg(i), _3dRenderingDecimation[i]->value()).toInt());
		_3dRenderingMaxDepth[i]->setValue(settings.value(tr("maxDepth%1").arg(i), _3dRenderingMaxDepth[i]->value()).toDouble());
		_3dRenderingShowScans[i]->setChecked(settings.value(tr("showScans%1").arg(i), _3dRenderingShowScans[i]->isChecked()).toBool());

		if(i<2)
		{
			_3dRenderingOpacity[i]->setValue(settings.value(tr("opacity%1").arg(i), _3dRenderingOpacity[i]->value()).toDouble());
			_3dRenderingPtSize[i]->setValue(settings.value(tr("ptSize%1").arg(i), _3dRenderingPtSize[i]->value()).toInt());
			_3dRenderingOpacityScan[i]->setValue(settings.value(tr("opacityScan%1").arg(i), _3dRenderingOpacityScan[i]->value()).toDouble());
			_3dRenderingPtSizeScan[i]->setValue(settings.value(tr("ptSizeScan%1").arg(i), _3dRenderingPtSizeScan[i]->value()).toInt());
		}

		if(i<1)
		{
			_3dRenderingMeshing[i]->setChecked(settings.value(tr("meshing%1").arg(i), _3dRenderingMeshing[i]->isChecked()).toBool());
		}
	}

	_ui->groupBox_poseFiltering->setChecked(settings.value("cloudFiltering", _ui->groupBox_poseFiltering->isChecked()).toBool());
	_ui->doubleSpinBox_cloudFilterRadius->setValue(settings.value("cloudFilteringRadius", _ui->doubleSpinBox_cloudFilterRadius->value()).toDouble());
	_ui->doubleSpinBox_cloudFilterAngle->setValue(settings.value("cloudFilteringAngle", _ui->doubleSpinBox_cloudFilterAngle->value()).toDouble());

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
	_ui->general_checkBox_autoRestart->setChecked(settings.value("autoRestart", _ui->general_checkBox_autoRestart->isChecked()).toBool());
	_ui->general_checkBox_cameraKeypoints->setChecked(settings.value("cameraKeypoints", _ui->general_checkBox_cameraKeypoints->isChecked()).toBool());
	_ui->source_comboBox_image_type->setCurrentIndex(settings.value("type", _ui->source_comboBox_image_type->currentIndex()).toInt());
	_ui->source_spinBox_imgWidth->setValue(settings.value("imgWidth",_ui->source_spinBox_imgWidth->value()).toInt());
	_ui->source_spinBox_imgheight->setValue(settings.value("imgHeight",_ui->source_spinBox_imgheight->value()).toInt());
	_ui->source_spinBox_framesDropped->setValue(settings.value("framesDropped",_ui->source_spinBox_framesDropped->value()).toInt());
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
	_ui->lineEdit_openniDevice->setText(settings.value("device",_ui->lineEdit_openniDevice->text()).toString());
	_ui->lineEdit_openniLocalTransform->setText(settings.value("localTransform",_ui->lineEdit_openniLocalTransform->text()).toString());
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
	QString path = QFileDialog::getSaveFileName(this, tr("Save settings..."), this->getWorkingDirectory()+"/config.ini", "*.ini");
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

	for(int i=0; i<3; ++i)
	{
		settings.setValue(tr("showClouds%1").arg(i), _3dRenderingShowClouds[i]->isChecked());
		settings.setValue(tr("voxelSize%1").arg(i), _3dRenderingVoxelSize[i]->value());
		settings.setValue(tr("decimation%1").arg(i), _3dRenderingDecimation[i]->value());
		settings.setValue(tr("maxDepth%1").arg(i), _3dRenderingMaxDepth[i]->value());
		settings.setValue(tr("showScans%1").arg(i), _3dRenderingShowScans[i]->isChecked());

		if(i<2)
		{
			settings.setValue(tr("opacity%1").arg(i), _3dRenderingOpacity[i]->value());
			settings.setValue(tr("ptSize%1").arg(i), _3dRenderingPtSize[i]->value());
			settings.setValue(tr("opacityScan%1").arg(i), _3dRenderingOpacityScan[i]->value());
			settings.setValue(tr("ptSizeScan%1").arg(i), _3dRenderingPtSizeScan[i]->value());
		}

		if(i<1)
		{
			settings.setValue(tr("meshing%1").arg(i), _3dRenderingMeshing[i]->isChecked());
		}
	}
	settings.setValue("cloudFiltering", _ui->groupBox_poseFiltering->isChecked());
	settings.setValue("cloudFilteringRadius", _ui->doubleSpinBox_cloudFilterRadius->value());
	settings.setValue("cloudFilteringAngle", _ui->doubleSpinBox_cloudFilterAngle->value());

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
	settings.setValue("autoRestart", 	_ui->general_checkBox_autoRestart->isChecked());
	settings.setValue("cameraKeypoints", _ui->general_checkBox_cameraKeypoints->isChecked());
	settings.setValue("type", 			_ui->source_comboBox_image_type->currentIndex());
	settings.setValue("imgWidth", 		_ui->source_spinBox_imgWidth->value());
	settings.setValue("imgHeight", 		_ui->source_spinBox_imgheight->value());
	settings.setValue("framesDropped",  _ui->source_spinBox_framesDropped->value());
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
	settings.setValue("device", 		_ui->lineEdit_openniDevice->text());
	settings.setValue("localTransform", _ui->lineEdit_openniLocalTransform->text());
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
	//TODO...
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
		_ui->lineEdit_databasePath->setEnabled(false);
		_ui->lineEdit_databasePath_2->setEnabled(false);
		_ui->toolButton_databasePath->setEnabled(false);
		_ui->toolButton_databasePath_2->setEnabled(false);
		_ui->label_databasePath->setEnabled(false);
		_ui->label_databasePath_2->setEnabled(false);

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
		_ui->lineEdit_databasePath->setEnabled(true);
		_ui->lineEdit_databasePath_2->setEnabled(true);
		_ui->toolButton_databasePath->setEnabled(true);
		_ui->toolButton_databasePath_2->setEnabled(true);
		_ui->label_databasePath->setEnabled(true);
		_ui->label_databasePath_2->setEnabled(true);

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

	if(user)
	{
		// from user
		_ui->groupBox_sourceDatabase->setChecked(true);
	}

	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->source_database_lineEdit_path->text(), tr("RTAB-Map database files (*.db)"));
	QFile file(path);
	if(!path.isEmpty() && file.exists())
	{
		_ui->source_database_lineEdit_path->setText(path);
		_ui->source_spinBox_databaseStartPos->setValue(0);
	}

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

void PreferencesDialog::selectSourceOpenni(bool user)
{
	ULOGGER_DEBUG("");

	if(!_ui->general_checkBox_activateRGBD->isChecked())
	{
		int button = QMessageBox::information(this,
				tr("Activate RGB-D SLAM?"),
				tr("You've selected OpenNI camera as source input, "
				   "would you want to activate RGB-D SLAM mode?"),
				QMessageBox::Yes | QMessageBox::No);
		if(button & QMessageBox::Yes)
		{
			_ui->general_checkBox_activateRGBD->setChecked(true);
		}
	}

	if(user)
	{
		// from user
		_ui->groupBox_sourceOpenni->setChecked(true);
	}

	if(user && _obsoletePanels)
	{
		_ui->groupBox_sourceImage->setChecked(false);
		_ui->groupBox_sourceDatabase->setChecked(false);
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
	viewer->setAttribute(Qt::WA_DeleteOnClose, true);
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
	QObject * obj = _ui->stackedWidget->findChild<QObject*>(key.c_str());
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
			combo->setCurrentIndex(QString(value.c_str()).toInt(&ok));
			if(!ok)
			{
				UERROR("Conversion failed from \"%s\" for parameter %s", value.c_str(), key.c_str());
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
				else if(comboBox == _ui->comboBox_detector_strategy)
				{
					if(value == 0) // 0 surf
					{
						this->addParameters(_ui->groupBox_detector_surf);
						_ui->stackedWidget_visualWord->setCurrentIndex(0);
					}
					else if(value == 1) // 1 sift
					{
						this->addParameters(_ui->groupBox_detector_sift);
						_ui->stackedWidget_visualWord->setCurrentIndex(1);
					}
				}
				else if(comboBox == _ui->odom_type)
				{
					if(value == 0) // 0 bow
					{
						this->addParameters(_ui->groupBox_odom_bow);
						_ui->stackedWidget_odom->setCurrentIndex(0);
					}
					else if(value == 1) // 1 fast
					{
						this->addParameters(_ui->groupBox_odom_fast);
						_ui->stackedWidget_odom->setCurrentIndex(1);
					}
					else if(value == 2) // 1 fast
					{
						this->addParameters(_ui->groupBox_odom_icp);
						_ui->stackedWidget_odom->setCurrentIndex(2);
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
				this->addParameters(_ui->groupBox_odom_correction);
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
	else if(sender() == _ui->general_checkBox_publishStats)
	{
		_ui->general_checkBox_publishStats_2->setChecked(_ui->general_checkBox_publishStats->isChecked());
	}
	else if(sender() == _ui->general_checkBox_publishStats_2)
	{
		_ui->general_checkBox_publishStats->setChecked(_ui->general_checkBox_publishStats_2->isChecked());
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

void PreferencesDialog::changeDatabasePath()
{
	QString path = QFileDialog::getOpenFileName(
			this,
			tr("Select database file..."),
			_ui->lineEdit_databasePath->text(),
			tr("RTAB-Map database files (*.db)"),
			0);

	if(!path.isEmpty())
	{
		_ui->lineEdit_databasePath->setText(path);
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

bool PreferencesDialog::isCloudsShown(int index) const
{
	UASSERT(index >= 0 && index <= 2);
	return _3dRenderingShowClouds[index]->isChecked();
}
bool PreferencesDialog::isCloudMeshing(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingMeshing[index]->isChecked();
}
double PreferencesDialog::getCloudVoxelSize(int index) const
{
	UASSERT(index >= 0 && index <= 2);
	return _3dRenderingVoxelSize[index]->value();
}
int PreferencesDialog::getCloudDecimation(int index) const
{
	UASSERT(index >= 0 && index <= 2);
	return _3dRenderingDecimation[index]->value();
}
double PreferencesDialog::getCloudMaxDepth(int index) const
{
	UASSERT(index >= 0 && index <= 2);
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
	UASSERT(index >= 0 && index <= 2);
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

PreferencesDialog::OdomType PreferencesDialog::getOdometryType() const
{
	if(_ui->odom_type->currentIndex() == 0)
	{
		return kOdomBOW;
	}
	else if(_ui->odom_type->currentIndex() == 1)
	{
		return kOdomBIN;
	}
	return kOdomICP;
}

bool PreferencesDialog::getGeneralAutoRestart() const
{
	return _ui->general_checkBox_autoRestart->isChecked();
}
bool PreferencesDialog::getGeneralCameraKeypoints() const
{
	return _ui->general_checkBox_cameraKeypoints->isChecked();
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
int PreferencesDialog::getFramesDropped() const
{
	return _ui->source_spinBox_framesDropped->value();
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

bool PreferencesDialog::isStatisticsPublished() const
{
	return _ui->general_checkBox_publishStats->isChecked();
}
double PreferencesDialog::getLoopThr() const
{
	return _ui->general_doubleSpinBox_hardThr->value();
}
double PreferencesDialog::getVpThr() const
{
	return _ui->general_doubleSpinBox_vp->value();
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

void PreferencesDialog::setAutoRestart(bool value)
{
	ULOGGER_DEBUG("autoRestart=%d", value);
	if(_ui->general_checkBox_autoRestart->isChecked() != value)
	{
		_ui->general_checkBox_autoRestart->setChecked(value);
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
	if(_ui->odom_type->currentIndex() == 0)
	{
		testOdometry(kOdomBOW);
	}
	else if(_ui->odom_type->currentIndex() == 1)
	{
		testOdometry(kOdomBIN);
	}
	else // ICP
	{
		testOdometry(kOdomICP);
	}
}

void PreferencesDialog::testOdometry(OdomType type)
{
	UASSERT(_odomCamera == 0 && _odomThread == 0);

	_odomCamera = new CameraOpenni(
			this->getSourceOpenniDevice().toStdString(),
			this->getGeneralInputRate(),
			this->getSourceOpenniLocalTransform());

	if(_odomCamera->init())
	{
		Odometry * odometry;
		ParametersMap parameters = this->getAllParameters();
		if(type == kOdomBIN)
		{
			odometry = new OdometryBinary(parameters);
		}
		else if(type == kOdomICP)
		{
			odometry = new OdometryICP(parameters);
		}
		else // kOdomBOW
		{
			odometry = new OdometryBOW(parameters);
		}

		_odomThread = new OdometryThread(odometry); // take ownership of odometry

		QWidget * window = new QWidget(this, Qt::Popup);
		window->setAttribute(Qt::WA_DeleteOnClose);
		window->setWindowFlags(Qt::Dialog);
		window->setWindowTitle(tr("%1 Odometry viewer").arg(type==kOdomBIN?"Binary":"Bag-of-words"));
		window->setMinimumWidth(800);
		window->setMinimumHeight(600);
		connect( window, SIGNAL(destroyed(QObject*)), this, SLOT(cleanOdometryTest()) );

		OdometryViewer * odomViewer = new OdometryViewer(10, 2, 0.0, window);

		QVBoxLayout *layout = new QVBoxLayout();
		layout->addWidget(odomViewer);
		window->setLayout(layout);

		UEventsManager::addHandler(_odomThread);
		UEventsManager::addHandler(odomViewer);

		UEventsManager::createPipe(_odomCamera, _odomThread, "CameraEvent");
		UEventsManager::createPipe(_odomThread, odomViewer, "OdometryEvent");

		window->showNormal();

		QApplication::processEvents();
		uSleep(500);
		QApplication::processEvents();

		_odomThread->start();
		_odomCamera->start();
	}
	else
	{
		QMessageBox::warning(this, "Initialization failed!", tr("Openni camera initialization failed!"));
		delete _odomCamera;
		_odomCamera = 0;
	}
}

void PreferencesDialog::cleanOdometryTest()
{
	UDEBUG("");
	if(_odomCamera)
	{
		_odomCamera->kill();
		delete _odomCamera;
		_odomCamera = 0;
	}
	if(_odomThread)
	{
		_odomThread->join(true);
		delete _odomThread;
		_odomThread = 0;
	}
}

}
