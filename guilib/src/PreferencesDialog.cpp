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

#include <QButtonGroup>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QtGui/QStandardItemModel>
#include <QMainWindow>
#include <QProgressDialog>
#include <QScrollBar>
#include <QStatusBar>
#include <QFormLayout>
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
#include "rtabmap/core/IMUThread.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/Optimizer.h"
#include "rtabmap/core/optimizer/OptimizerG2O.h"
#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/clams/discrete_depth_distortion_model.h"

#include "rtabmap/gui/LoopClosureViewer.h"
#include "rtabmap/gui/CameraViewer.h"
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/gui/ImageView.h"
#include "rtabmap/gui/GraphViewer.h"
#include "rtabmap/gui/ExportCloudsDialog.h"
#include "rtabmap/gui/ExportBundlerDialog.h"
#include "rtabmap/gui/PostProcessingDialog.h"
#include "rtabmap/gui/CreateSimpleCalibrationDialog.h"
#include "rtabmap/gui/DepthCalibrationDialog.h"

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

		// disable BruteForceGPU option
		_ui->comboBox_dictionary_strategy->setItemData(4, 0, Qt::UserRole - 1);
		_ui->reextract_nn->setItemData(4, 0, Qt::UserRole - 1);
	}

#ifndef RTABMAP_OCTOMAP
	_ui->groupBox_octomap->setChecked(false);
	_ui->groupBox_octomap->setEnabled(false);
#endif

#ifndef RTABMAP_REALSENSE_SLAM
	_ui->checkbox_realsenseOdom->setChecked(false);
	_ui->checkbox_realsenseOdom->setEnabled(false);
	_ui->label_realsenseOdom->setEnabled(false);
#endif

#ifndef RTABMAP_FOVIS
	_ui->odom_strategy->setItemData(2, 0, Qt::UserRole - 1);
#endif
#ifndef RTABMAP_VISO2
	_ui->odom_strategy->setItemData(3, 0, Qt::UserRole - 1);
#endif
#ifndef RTABMAP_DVO
	_ui->odom_strategy->setItemData(4, 0, Qt::UserRole - 1);
#endif
#ifndef RTABMAP_ORB_SLAM
	_ui->odom_strategy->setItemData(5, 0, Qt::UserRole - 1);
#elif RTABMAP_ORB_SLAM == 3
	_ui->odom_strategy->setItemText(5, "ORB SLAM 3");
	_ui->groupBox_odomORBSLAM->setTitle("ORB SLAM 3");
#else
	_ui->odom_strategy->setItemText(5, "ORB SLAM 2");
	_ui->groupBox_odomORBSLAM->setTitle("ORB SLAM 2");
#endif
#ifndef RTABMAP_OKVIS
	_ui->odom_strategy->setItemData(6, 0, Qt::UserRole - 1);
#endif
#ifndef RTABMAP_LOAM
	_ui->odom_strategy->setItemData(7, 0, Qt::UserRole - 1);
#endif
#ifndef RTABMAP_MSCKF_VIO
	_ui->odom_strategy->setItemData(8, 0, Qt::UserRole - 1);
#endif
#ifndef RTABMAP_VINS
	_ui->odom_strategy->setItemData(9, 0, Qt::UserRole - 1);
#endif
#ifndef RTABMAP_OPENVINS
	_ui->odom_strategy->setItemData(10, 0, Qt::UserRole - 1);
#endif
#ifndef RTABMAP_FLOAM
	_ui->odom_strategy->setItemData(11, 0, Qt::UserRole - 1);
#endif
#ifndef RTABMAP_OPEN3D
	_ui->odom_strategy->setItemData(12, 0, Qt::UserRole - 1);
#endif

#if CV_MAJOR_VERSION < 3
	_ui->stereosgbm_mode->setItemData(2, 0, Qt::UserRole - 1);
#endif

//SURF
#ifndef RTABMAP_NONFREE
	_ui->comboBox_detector_strategy->setItemData(0, 0, Qt::UserRole - 1);
	_ui->comboBox_detector_strategy->setItemData(12, 0, Qt::UserRole - 1);
	_ui->comboBox_detector_strategy->setItemData(14, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(0, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(12, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(14, 0, Qt::UserRole - 1);
#endif

// SIFT
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 3) || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION < 4 || (CV_MINOR_VERSION==4 && CV_SUBMINOR_VERSION<11)))
#ifndef RTABMAP_NONFREE
	_ui->comboBox_detector_strategy->setItemData(1, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(1, 0, Qt::UserRole - 1);
#endif
#endif

#if CV_MAJOR_VERSION >= 3 && !defined(HAVE_OPENCV_XFEATURES2D)
	_ui->comboBox_detector_strategy->setItemData(3, 0, Qt::UserRole - 1);
	_ui->comboBox_detector_strategy->setItemData(4, 0, Qt::UserRole - 1);
	_ui->comboBox_detector_strategy->setItemData(5, 0, Qt::UserRole - 1);
	_ui->comboBox_detector_strategy->setItemData(6, 0, Qt::UserRole - 1);
	_ui->comboBox_detector_strategy->setItemData(12, 0, Qt::UserRole - 1);
	_ui->comboBox_detector_strategy->setItemData(13, 0, Qt::UserRole - 1);
	_ui->comboBox_detector_strategy->setItemData(14, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(3, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(4, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(5, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(6, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(12, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(13, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(14, 0, Qt::UserRole - 1);
#endif

#ifndef RTABMAP_ORB_OCTREE
		_ui->comboBox_detector_strategy->setItemData(10, 0, Qt::UserRole - 1);
		_ui->vis_feature_detector->setItemData(10, 0, Qt::UserRole - 1);
#endif

#ifndef RTABMAP_TORCH
	_ui->comboBox_detector_strategy->setItemData(11, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(11, 0, Qt::UserRole - 1);
#endif

#ifndef RTABMAP_PYTHON
	_ui->comboBox_detector_strategy->setItemData(15, 0, Qt::UserRole - 1);
	_ui->vis_feature_detector->setItemData(15, 0, Qt::UserRole - 1);
	_ui->reextract_nn->setItemData(6, 0, Qt::UserRole - 1);
#endif

#if !defined(HAVE_OPENCV_XFEATURES2D) || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION<4 || CV_MINOR_VERSION==4 && CV_SUBMINOR_VERSION<1))
	_ui->reextract_nn->setItemData(7, 0, Qt::UserRole - 1);
#endif

#if CV_MAJOR_VERSION >= 3
	_ui->groupBox_fast_opencv2->setEnabled(false);
#else
	_ui->comboBox_detector_strategy->setItemData(9, 0, Qt::UserRole - 1); // No KAZE
	_ui->comboBox_detector_strategy->setItemData(13, 0, Qt::UserRole - 1); // No DAISY
	_ui->comboBox_detector_strategy->setItemData(14, 0, Qt::UserRole - 1); // No DAISY
	_ui->vis_feature_detector->setItemData(9, 0, Qt::UserRole - 1); // No KAZE
	_ui->vis_feature_detector->setItemData(13, 0, Qt::UserRole - 1); // No DAISY
	_ui->vis_feature_detector->setItemData(14, 0, Qt::UserRole - 1); // No DAISY
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
#ifdef RTABMAP_ORB_SLAM
	else
	{
		// only graph optimization is disabled, g2o (from ORB_SLAM2) is valid only for SBA
		_ui->graphOptimization_type->setItemData(1, 0, Qt::UserRole - 1);
	}
#endif
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
	if(!Optimizer::isAvailable(Optimizer::kTypeCeres))
	{
		_ui->graphOptimization_type->setItemData(3, 0, Qt::UserRole - 1);
		_ui->odom_f2m_bundleStrategy->setItemData(3, 0, Qt::UserRole - 1);
		_ui->loopClosure_bundle->setItemData(3, 0, Qt::UserRole - 1);
	}
#ifdef RTABMAP_VERTIGO
	if(!Optimizer::isAvailable(Optimizer::kTypeG2O) && !Optimizer::isAvailable(Optimizer::kTypeGTSAM))
#endif
	{
		_ui->graphOptimization_robust->setEnabled(false);
	}
#ifndef RTABMAP_POINTMATCHER
	_ui->comboBox_icpStrategy->setItemData(1, 0, Qt::UserRole - 1);
#endif
#ifndef RTABMAP_CCCORELIB
	_ui->comboBox_icpStrategy->setItemData(2, 0, Qt::UserRole - 1);
#endif
	if(!CameraOpenni::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(kSrcOpenNI_PCL - kSrcRGBD, 0, Qt::UserRole - 1);
	}
	if(!CameraFreenect::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(kSrcFreenect - kSrcRGBD, 0, Qt::UserRole - 1);
	}
	if(!CameraOpenNICV::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(kSrcOpenNI_CV - kSrcRGBD, 0, Qt::UserRole - 1);
		_ui->comboBox_cameraRGBD->setItemData(kSrcOpenNI_CV_ASUS - kSrcRGBD, 0, Qt::UserRole - 1);
	}
	if(!CameraOpenNI2::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(kSrcOpenNI2 - kSrcRGBD, 0, Qt::UserRole - 1);
	}
	if(!CameraFreenect2::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(kSrcFreenect2 - kSrcRGBD, 0, Qt::UserRole - 1);
	}
	if (!CameraK4W2::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(kSrcK4W2 - kSrcRGBD, 0, Qt::UserRole - 1);
	}
	if (!CameraK4A::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(kSrcK4A - kSrcRGBD, 0, Qt::UserRole - 1);
	}
	if (!CameraRealSense::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(kSrcRealSense - kSrcRGBD, 0, Qt::UserRole - 1);
	}
	if (!CameraRealSense2::available())
	{
		_ui->comboBox_cameraRGBD->setItemData(kSrcRealSense2 - kSrcRGBD, 0, Qt::UserRole - 1);
		_ui->comboBox_cameraStereo->setItemData(kSrcStereoRealSense2 - kSrcStereo, 0, Qt::UserRole - 1);
		_ui->comboBox_odom_sensor->setItemData(1, 0, Qt::UserRole - 1);
	}
	if(!CameraStereoDC1394::available())
	{
		_ui->comboBox_cameraStereo->setItemData(kSrcDC1394 - kSrcStereo, 0, Qt::UserRole - 1);
	}
	if(!CameraStereoFlyCapture2::available())
	{
		_ui->comboBox_cameraStereo->setItemData(kSrcFlyCapture2 - kSrcStereo, 0, Qt::UserRole - 1);
	}
	if (!CameraStereoZed::available())
	{
		_ui->comboBox_cameraStereo->setItemData(kSrcStereoZed - kSrcStereo, 0, Qt::UserRole - 1);
		_ui->comboBox_odom_sensor->setItemData(2, 0, Qt::UserRole - 1);
	}
    if (!CameraStereoTara::available())
    {
        _ui->comboBox_cameraStereo->setItemData(kSrcStereoTara - kSrcStereo, 0, Qt::UserRole - 1);
    }
    if (!CameraMyntEye::available())
    {
        _ui->comboBox_cameraStereo->setItemData(kSrcStereoMyntEye - kSrcStereo, 0, Qt::UserRole - 1);
    }
    if (!CameraStereoZedOC::available())
	{
		_ui->comboBox_cameraStereo->setItemData(kSrcStereoZedOC - kSrcStereo, 0, Qt::UserRole - 1);
	}
    if (!CameraDepthAI::available())
	{
		_ui->comboBox_cameraStereo->setItemData(kSrcStereoDepthAI - kSrcStereo, 0, Qt::UserRole - 1);
	}
	_ui->openni2_exposure->setEnabled(CameraOpenNI2::exposureGainAvailable());
	_ui->openni2_gain->setEnabled(CameraOpenNI2::exposureGainAvailable());

#if PCL_VERSION_COMPARE(<, 1, 7, 2)
	_ui->checkBox_showFrustums->setEnabled(false);
	_ui->checkBox_showFrustums->setChecked(false);
	_ui->checkBox_showOdomFrustums->setEnabled(false);
	_ui->checkBox_showOdomFrustums->setChecked(false);
#endif

	//if OpenCV < 3.4.2
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION <4 || (CV_MINOR_VERSION ==4 && CV_SUBMINOR_VERSION<2)))
	_ui->ArucoDictionary->setItemData(17, 0, Qt::UserRole - 1);
	_ui->ArucoDictionary->setItemData(18, 0, Qt::UserRole - 1);
	_ui->ArucoDictionary->setItemData(19, 0, Qt::UserRole - 1);
	_ui->ArucoDictionary->setItemData(20, 0, Qt::UserRole - 1);
#endif
#ifndef HAVE_OPENCV_ARUCO
	_ui->label_markerDetection->setText(_ui->label_markerDetection->text()+" This option works only if OpenCV has been built with \"aruco\" module.");
#endif

#ifndef RTABMAP_MADGWICK
	_ui->comboBox_imuFilter_strategy->setItemData(1, 0, Qt::UserRole - 1);
#endif

	// in case we change the ui, we should not forget to change stuff related to this parameter
	UASSERT(_ui->odom_registration->count() == 4);

	// Default Driver
	connect(_ui->comboBox_sourceType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateSourceGrpVisibility()));
	connect(_ui->comboBox_cameraRGBD, SIGNAL(currentIndexChanged(int)), this, SLOT(updateSourceGrpVisibility()));
	connect(_ui->source_comboBox_image_type, SIGNAL(currentIndexChanged(int)), this, SLOT(updateSourceGrpVisibility()));
	connect(_ui->comboBox_cameraStereo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateSourceGrpVisibility()));
	connect(_ui->comboBox_imuFilter_strategy, SIGNAL(currentIndexChanged(int)), this, SLOT(updateSourceGrpVisibility()));
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
	connect(_ui->checkBox_odom_onlyInliersShown, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->radioButton_posteriorGraphView, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->radioButton_wordsGraphView, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->radioButton_localizationsGraphView, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->radioButton_nochangeGraphView, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkbox_odomDisabled, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->odom_registration, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->odom_f2m_gravitySigma, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteGeneralPanel()));
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

	_3dRenderingColorScheme.resize(2);
	_3dRenderingColorScheme[0] = _ui->spinBox_colorScheme;
	_3dRenderingColorScheme[1] = _ui->spinBox_colorScheme_odom;

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

	_3dRenderingMaxRange.resize(2);
	_3dRenderingMaxRange[0] = _ui->doubleSpinBox_maxRange;
	_3dRenderingMaxRange[1] = _ui->doubleSpinBox_maxRange_odom;

	_3dRenderingMinRange.resize(2);
	_3dRenderingMinRange[0] = _ui->doubleSpinBox_minRange;
	_3dRenderingMinRange[1] = _ui->doubleSpinBox_minRange_odom;

	_3dRenderingVoxelSizeScan.resize(2);
	_3dRenderingVoxelSizeScan[0] = _ui->doubleSpinBox_voxelSizeScan;
	_3dRenderingVoxelSizeScan[1] = _ui->doubleSpinBox_voxelSizeScan_odom;

	_3dRenderingColorSchemeScan.resize(2);
	_3dRenderingColorSchemeScan[0] = _ui->spinBox_colorSchemeScan;
	_3dRenderingColorSchemeScan[1] = _ui->spinBox_colorSchemeScan_odom;

	_3dRenderingOpacityScan.resize(2);
	_3dRenderingOpacityScan[0] = _ui->doubleSpinBox_opacity_scan;
	_3dRenderingOpacityScan[1] = _ui->doubleSpinBox_opacity_odom_scan;

	_3dRenderingPtSizeScan.resize(2);
	_3dRenderingPtSizeScan[0] = _ui->spinBox_ptsize_scan;
	_3dRenderingPtSizeScan[1] = _ui->spinBox_ptsize_odom_scan;

	_3dRenderingShowFeatures.resize(2);
	_3dRenderingShowFeatures[0] = _ui->checkBox_showFeatures;
	_3dRenderingShowFeatures[1] = _ui->checkBox_showOdomFeatures;

	_3dRenderingShowFrustums.resize(2);
	_3dRenderingShowFrustums[0] = _ui->checkBox_showFrustums;
	_3dRenderingShowFrustums[1] = _ui->checkBox_showOdomFrustums;

	_3dRenderingPtSizeFeatures.resize(2);
	_3dRenderingPtSizeFeatures[0] = _ui->spinBox_ptsize_features;
	_3dRenderingPtSizeFeatures[1] = _ui->spinBox_ptsize_odom_features;

	_3dRenderingGravity.resize(2);
	_3dRenderingGravity[0] = _ui->checkBox_showIMUGravity;
	_3dRenderingGravity[1] = _ui->checkBox_showIMUGravity_odom;

	_3dRenderingGravityLength.resize(2);
	_3dRenderingGravityLength[0] = _ui->doubleSpinBox_lengthIMUGravity;
	_3dRenderingGravityLength[1] = _ui->doubleSpinBox_lengthIMUGravity_odom;

	for(int i=0; i<2; ++i)
	{
		connect(_3dRenderingShowClouds[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingDecimation[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingMaxDepth[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingMinDepth[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingRoiRatios[i], SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingShowScans[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingShowFeatures[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingShowFrustums[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

		connect(_3dRenderingDownsamplingScan[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingMaxRange[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingMinRange[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingVoxelSizeScan[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingColorScheme[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingOpacity[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingPtSize[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingColorSchemeScan[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingOpacityScan[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingPtSizeScan[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingPtSizeFeatures[i], SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingGravity[i], SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
		connect(_3dRenderingGravityLength[i], SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	}
	connect(_ui->doubleSpinBox_voxel, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_noiseRadius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_noiseMinNeighbors, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_ceilingFilterHeight, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_floorFilterHeight, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_normalKSearch, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_normalRadiusSearch, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_ceilingFilterHeight_scan, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_floorFilterHeight_scan, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_normalKSearch_scan, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_normalRadiusSearch_scan, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->checkBox_showGraphs, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_showLabels, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_showFrames, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_showLandmarks, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_landmarkSize, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_showIMUGravity, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_showIMUAcc, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->radioButton_noFiltering, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->radioButton_nodeFiltering, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->radioButton_subtractFiltering, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_cloudFilterRadius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_cloudFilterAngle, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_subtractFilteringMinPts, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_subtractFilteringRadius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_subtractFilteringAngle, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->checkBox_map_shown, SIGNAL(clicked(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->doubleSpinBox_map_opacity, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteCloudRenderingPanel()));

	connect(_ui->groupBox_octomap, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_octomap_treeDepth, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_octomap_2dgrid, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->checkBox_octomap_show3dMap, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->comboBox_octomap_renderingType, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));
	connect(_ui->spinBox_octomap_pointSize, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteCloudRenderingPanel()));

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
	connect(_ui->general_doubleSpinBox_imgRate, SIGNAL(valueChanged(double)), _ui->doubleSpinBox_OdomORBSLAMFps, SLOT(setValue(double)));
	connect(_ui->source_mirroring, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->toolButton_source_path_calibration, SIGNAL(clicked()), this, SLOT(selectCalibrationPath()));
	connect(_ui->lineEdit_calibrationFile, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	_ui->stackedWidget_src->setCurrentIndex(_ui->comboBox_sourceType->currentIndex());
	connect(_ui->comboBox_sourceType, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_src, SLOT(setCurrentIndex(int)));
	connect(_ui->comboBox_sourceType, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_sourceDevice, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_sourceLocalTransform, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));

	//RGB source
	_ui->stackedWidget_image->setCurrentIndex(_ui->source_comboBox_image_type->currentIndex());
	connect(_ui->source_comboBox_image_type, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_image, SLOT(setCurrentIndex(int)));
	connect(_ui->source_comboBox_image_type, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_rgb_rectify, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//images group
	connect(_ui->source_images_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceImagesPath()));
	connect(_ui->source_images_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_images_spinBox_startPos, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_images_spinBox_maxFrames, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_cameraImages_bayerMode, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	// usb group
	connect(_ui->spinBox_usbcam_streamWidth, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_usbcam_streamHeight, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//video group
	connect(_ui->source_video_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceVideoPath()));
	connect(_ui->source_video_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	//database group
	connect(_ui->source_database_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceDatabase()));
	connect(_ui->toolButton_dbViewer, SIGNAL(clicked()), this, SLOT(openDatabaseViewer()));
	connect(_ui->groupBox_sourceDatabase, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_database_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_ignoreOdometry, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_ignoreGoalDelay, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_ignoreGoals, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_ignoreLandmarks, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_ignoreFeatures, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_databaseStartId, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_databaseStopId, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_useDbStamps, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_database_cameraIndex, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_stereoToDepthDB, SIGNAL(toggled(bool)), _ui->checkbox_stereo_depthGenerated, SLOT(setChecked(bool)));
	connect(_ui->checkbox_stereo_depthGenerated, SIGNAL(toggled(bool)), _ui->source_checkBox_stereoToDepthDB, SLOT(setChecked(bool)));

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
	connect(_ui->openni2_hshift, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->openni2_vshift, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->openni2_depth_decimation, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_freenect2Format, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_freenect2MinDepth, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_freenect2MaxDepth, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_freenect2BilateralFiltering, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_freenect2EdgeAwareFiltering, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_freenect2NoiseFiltering, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_freenect2Pipeline, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_k4w2Format, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_realsensePresetRGB, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_realsensePresetDepth, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_realsenseOdom, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_realsenseDepthScaledToRGBSize, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_realsenseRGBSource, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_rs2_emitter, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_rs2_irMode, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_rs2_irDepth, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_rs2_width, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_rs2_height, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_rs2_rate, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_rs2_width_depth, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_rs2_height_depth, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_rs2_rate_depth, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_rs2_globalTimeStync, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_rs2_jsonFile, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->toolButton_rs2_jsonFile, SIGNAL(clicked()), this, SLOT(selectSourceRealsense2JsonPath()));

	connect(_ui->toolButton_cameraImages_timestamps, SIGNAL(clicked()), this, SLOT(selectSourceImagesStamps()));
	connect(_ui->lineEdit_cameraImages_timestamps, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->toolButton_cameraRGBDImages_path_rgb, SIGNAL(clicked()), this, SLOT(selectSourceRGBDImagesPathRGB()));
	connect(_ui->toolButton_cameraRGBDImages_path_depth, SIGNAL(clicked()), this, SLOT(selectSourceRGBDImagesPathDepth()));
	connect(_ui->toolButton_cameraImages_path_scans, SIGNAL(clicked()), this, SLOT(selectSourceImagesPathScans()));
	connect(_ui->toolButton_cameraImages_path_imu, SIGNAL(clicked()), this, SLOT(selectSourceImagesPathIMU()));
	connect(_ui->toolButton_cameraImages_odom, SIGNAL(clicked()), this, SLOT(selectSourceImagesPathOdom()));
	connect(_ui->toolButton_cameraImages_gt, SIGNAL(clicked()), this, SLOT(selectSourceImagesPathGt()));
	connect(_ui->lineEdit_cameraRGBDImages_path_rgb, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraRGBDImages_path_depth, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_cameraImages_configForEachFrame, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_cameraImages_timestamps, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_cameraImages_syncTimeStamps, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_cameraRGBDImages_scale, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_cameraRGBDImages_startIndex, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_cameraRGBDImages_maxFrames, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraImages_path_scans, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraImages_laser_transform, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_cameraImages_max_scan_pts, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraImages_odom, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_cameraImages_odomFormat, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraImages_gt, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_cameraImages_gtFormat, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_maxPoseTimeDiff, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraImages_path_imu, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraImages_imu_transform, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_cameraImages_max_imu_rate, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->groupBox_depthFromScan, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->groupBox_depthFromScan_fillHoles, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->radioButton_depthFromScan_vertical, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->radioButton_depthFromScan_horizontal, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_depthFromScan_fillBorders, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->toolButton_cameraStereoImages_path_left, SIGNAL(clicked()), this, SLOT(selectSourceStereoImagesPathLeft()));
	connect(_ui->toolButton_cameraStereoImages_path_right, SIGNAL(clicked()), this, SLOT(selectSourceStereoImagesPathRight()));
	connect(_ui->lineEdit_cameraStereoImages_path_left, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraStereoImages_path_right, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_stereo_rectify, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_cameraStereoImages_startIndex, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_cameraStereoImages_maxFrames, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->toolButton_cameraStereoVideo_path, SIGNAL(clicked()), this, SLOT(selectSourceStereoVideoPath()));
	connect(_ui->toolButton_cameraStereoVideo_path_2, SIGNAL(clicked()), this, SLOT(selectSourceStereoVideoPath2()));
	connect(_ui->lineEdit_cameraStereoVideo_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_cameraStereoVideo_path_2, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->spinBox_stereo_right_device, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_stereousbcam_streamWidth, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_stereousbcam_streamHeight, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->comboBox_stereoZed_resolution, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_stereoZed_quality, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_stereoZed_quality, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStereoDisparityVisibility()));
	connect(_ui->checkbox_stereoZed_selfCalibration, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_cameraStereo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStereoDisparityVisibility()));
	connect(_ui->comboBox_stereoZed_sensingMode, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_stereoZed_confidenceThr, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_stereoZed_texturenessConfidenceThr, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->toolButton_zedSvoPath, SIGNAL(clicked()), this, SLOT(selectSourceSvoPath()));
	connect(_ui->lineEdit_zedSvoPath, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->checkbox_stereoMyntEye_rectify, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_stereoMyntEye_depth, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_stereoMyntEye_autoExposure, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_stereoMyntEye_gain, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_stereoMyntEye_brightness, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_stereoMyntEye_contrast, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_stereoMyntEye_irControl, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->comboBox_depthai_resolution, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_depthai_depth, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_depthai_confidence, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_depthai_imu_firmware_update, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->checkbox_rgbd_colorOnly, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_source_imageDecimation, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_stereo_depthGenerated, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_stereo_exposureCompensation, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->pushButton_calibrate, SIGNAL(clicked()), this, SLOT(calibrate()));
	connect(_ui->pushButton_calibrate_simple, SIGNAL(clicked()), this, SLOT(calibrateSimple()));
	connect(_ui->toolButton_openniOniPath, SIGNAL(clicked()), this, SLOT(selectSourceOniPath()));
	connect(_ui->toolButton_openni2OniPath, SIGNAL(clicked()), this, SLOT(selectSourceOni2Path()));
	connect(_ui->comboBox_k4a_rgb_resolution, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_k4a_framerate, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_k4a_depth_resolution, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_k4a_irDepth, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->toolButton_k4a_mkv, SIGNAL(clicked()), this, SLOT(selectSourceMKVPath()));
	connect(_ui->toolButton_source_distortionModel, SIGNAL(clicked()), this, SLOT(selectSourceDistortionModel()));
	connect(_ui->toolButton_distortionModel, SIGNAL(clicked()), this, SLOT(visualizeDistortionModel()));
	connect(_ui->lineEdit_openniOniPath, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_openni2OniPath, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_k4a_mkv, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_checkBox_useMKVStamps, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_source_distortionModel, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->groupBox_bilateral, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_bilateral_sigmaS, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_bilateral_sigmaR, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->lineEdit_odom_sensor_extrinsics, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_odom_sensor, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->toolButton_odom_sensor_path_calibration, SIGNAL(clicked()), this, SLOT(selectOdomSensorCalibrationPath()));
	connect(_ui->pushButton_odom_sensor_calibrate, SIGNAL(clicked()), this, SLOT(calibrateOdomSensorExtrinsics()));
	connect(_ui->lineEdit_odom_sensor_path_calibration, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->lineEdit_odomSourceDevice, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_odom_sensor_time_offset, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_odom_sensor_scale_factor, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkBox_odom_sensor_use_as_gt, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->comboBox_imuFilter_strategy, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->comboBox_imuFilter_strategy, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_imuFilter, SLOT(setCurrentIndex(int)));
	_ui->stackedWidget_imuFilter->setCurrentIndex(_ui->comboBox_imuFilter_strategy->currentIndex());
	connect(_ui->checkBox_imuFilter_baseFrameConversion, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->checkbox_publishInterIMU, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	connect(_ui->checkBox_source_scanFromDepth, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_source_scanDownsampleStep, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_source_scanRangeMin, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_source_scanRangeMax, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_source_scanVoxelSize, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->spinBox_source_scanNormalsK, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_source_scanNormalsRadius, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->doubleSpinBox_source_scanNormalsForceGroundUp, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));


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
	_ui->general_checkBox_publishRMSE->setObjectName(Parameters::kRtabmapComputeRMSE().c_str());
	_ui->general_checkBox_saveWMState->setObjectName(Parameters::kRtabmapSaveWMState().c_str());
	_ui->general_checkBox_publishRAM->setObjectName(Parameters::kRtabmapPublishRAMUsage().c_str());
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
	_ui->general_checkBox_startNewMapOnGoodSignature->setObjectName(Parameters::kRtabmapStartNewMapOnGoodSignature().c_str());
	_ui->general_checkBox_imagesAlreadyRectified->setObjectName(Parameters::kRtabmapImagesAlreadyRectified().c_str());
	_ui->general_checkBox_rectifyOnlyFeatures->setObjectName(Parameters::kRtabmapRectifyOnlyFeatures().c_str());
	_ui->checkBox_rtabmap_loopGPS->setObjectName(Parameters::kRtabmapLoopGPS().c_str());
	_ui->lineEdit_workingDirectory->setObjectName(Parameters::kRtabmapWorkingDirectory().c_str());
	connect(_ui->toolButton_workingDirectory, SIGNAL(clicked()), this, SLOT(changeWorkingDirectory()));

	// Memory
	_ui->general_checkBox_keepRawData->setObjectName(Parameters::kMemImageKept().c_str());
	_ui->general_checkBox_keepBinaryData->setObjectName(Parameters::kMemBinDataKept().c_str());
	_ui->general_checkBox_saveIntermediateNodeData->setObjectName(Parameters::kMemIntermediateNodeDataKept().c_str());
	_ui->lineEdit_rgbCompressionFormat->setObjectName(Parameters::kMemImageCompressionFormat().c_str());
	_ui->general_checkBox_keepDescriptors->setObjectName(Parameters::kMemRawDescriptorsKept().c_str());
	_ui->general_checkBox_saveDepth16bits->setObjectName(Parameters::kMemSaveDepth16Format().c_str());
	_ui->general_checkBox_compressionParallelized->setObjectName(Parameters::kMemCompressionParallelized().c_str());
	_ui->general_checkBox_reduceGraph->setObjectName(Parameters::kMemReduceGraph().c_str());
	_ui->general_checkBox_keepNotLinkedNodes->setObjectName(Parameters::kMemNotLinkedNodesKept().c_str());
	_ui->general_spinBox_maxStMemSize->setObjectName(Parameters::kMemSTMSize().c_str());
	_ui->doubleSpinBox_similarityThreshold->setObjectName(Parameters::kMemRehearsalSimilarity().c_str());
	_ui->general_checkBox_SLAM_mode->setObjectName(Parameters::kMemIncrementalMemory().c_str());
	_ui->general_checkBox_saveLocalizationData->setObjectName(Parameters::kMemLocalizationDataSaved().c_str());
	_ui->general_doubleSpinBox_recentWmRatio->setObjectName(Parameters::kMemRecentWmRatio().c_str());
	_ui->general_checkBox_transferSortingByWeightId->setObjectName(Parameters::kMemTransferSortingByWeightId().c_str());
	_ui->general_checkBox_RehearsalIdUpdatedToNewOne->setObjectName(Parameters::kMemRehearsalIdUpdatedToNewOne().c_str());
	_ui->general_checkBox_generateIds->setObjectName(Parameters::kMemGenerateIds().c_str());
	_ui->general_checkBox_badSignaturesIgnored->setObjectName(Parameters::kMemBadSignaturesIgnored().c_str());
	_ui->general_checkBox_createMapLabels->setObjectName(Parameters::kMemMapLabelsAdded().c_str());
	_ui->general_checkBox_initWMWithAllNodes->setObjectName(Parameters::kMemInitWMWithAllNodes().c_str());
	_ui->checkBox_localSpaceScanMatchingIDsSaved->setObjectName(Parameters::kRGBDScanMatchingIdsSavedInLinks().c_str());
	_ui->checkBox_localSpaceCreateGlobalScanMap->setObjectName(Parameters::kRGBDProximityGlobalScanMap().c_str());
	_ui->spinBox_imagePreDecimation->setObjectName(Parameters::kMemImagePreDecimation().c_str());
	_ui->spinBox_imagePostDecimation->setObjectName(Parameters::kMemImagePostDecimation().c_str());
	_ui->general_spinBox_laserScanDownsample->setObjectName(Parameters::kMemLaserScanDownsampleStepSize().c_str());
	_ui->general_doubleSpinBox_laserScanVoxelSize->setObjectName(Parameters::kMemLaserScanVoxelSize().c_str());
	_ui->general_spinBox_laserScanNormalK->setObjectName(Parameters::kMemLaserScanNormalK().c_str());
	_ui->general_doubleSpinBox_laserScanNormalRadius->setObjectName(Parameters::kMemLaserScanNormalRadius().c_str());
	_ui->checkBox_useOdomFeatures->setObjectName(Parameters::kMemUseOdomFeatures().c_str());
	_ui->memCovOffDiagIgnored->setObjectName(Parameters::kMemCovOffDiagIgnored().c_str());

	// Database
	_ui->checkBox_dbInMemory->setObjectName(Parameters::kDbSqlite3InMemory().c_str());
	_ui->spinBox_dbCacheSize->setObjectName(Parameters::kDbSqlite3CacheSize().c_str());
	_ui->comboBox_dbJournalMode->setObjectName(Parameters::kDbSqlite3JournalMode().c_str());
	_ui->comboBox_dbSynchronous->setObjectName(Parameters::kDbSqlite3Synchronous().c_str());
	_ui->comboBox_dbTempStore->setObjectName(Parameters::kDbSqlite3TempStore().c_str());
	_ui->lineEdit_targetDatabaseVersion->setObjectName(Parameters::kDbTargetVersion().c_str());


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
	_ui->checkBox_kp_byteToFloat->setObjectName(Parameters::kKpByteToFloat().c_str());
	_ui->surf_doubleSpinBox_rebalancingFactor->setObjectName(Parameters::kKpFlannRebalancingFactor().c_str());
	_ui->comboBox_detector_strategy->setObjectName(Parameters::kKpDetectorStrategy().c_str());
	_ui->surf_doubleSpinBox_nndrRatio->setObjectName(Parameters::kKpNndrRatio().c_str());
	_ui->surf_doubleSpinBox_maxDepth->setObjectName(Parameters::kKpMaxDepth().c_str());
	_ui->surf_doubleSpinBox_minDepth->setObjectName(Parameters::kKpMinDepth().c_str());
	_ui->checkBox_memDepthAsMask->setObjectName(Parameters::kMemDepthAsMask().c_str());
	_ui->checkBox_memStereoFromMotion->setObjectName(Parameters::kMemStereoFromMotion().c_str());
	_ui->surf_spinBox_wordsPerImageTarget->setObjectName(Parameters::kKpMaxFeatures().c_str());
	_ui->spinBox_KPGridRows->setObjectName(Parameters::kKpGridRows().c_str());
	_ui->spinBox_KPGridCols->setObjectName(Parameters::kKpGridCols().c_str());
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
	_ui->sift_checkBox_rootsift->setObjectName(Parameters::kSIFTRootSIFT().c_str());

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
	_ui->fastCV->setObjectName(Parameters::kFASTCV().c_str());

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

	//KAZE
	_ui->checkBox_kaze_extended->setObjectName(Parameters::kKAZEExtended().c_str());
	_ui->checkBox_kaze_upright->setObjectName(Parameters::kKAZEUpright().c_str());
	_ui->doubleSpinBox_kaze_threshold->setObjectName(Parameters::kKAZEThreshold().c_str());
	_ui->spinBox_kaze_octaves->setObjectName(Parameters::kKAZENOctaves().c_str());
	_ui->spinBox_kaze_octavelayers->setObjectName(Parameters::kKAZENOctaveLayers().c_str());
	_ui->spinBox_kaze_diffusivity->setObjectName(Parameters::kKAZEDiffusivity().c_str());

	// SuperPoint Torch
	_ui->lineEdit_sptorch_path->setObjectName(Parameters::kSuperPointModelPath().c_str());
	connect(_ui->toolButton_sptorch_path, SIGNAL(clicked()), this, SLOT(changeSuperPointModelPath()));
	_ui->doubleSpinBox_sptorch_threshold->setObjectName(Parameters::kSuperPointThreshold().c_str());
	_ui->checkBox_sptorch_nms->setObjectName(Parameters::kSuperPointNMS().c_str());
	_ui->spinBox_sptorch_minDistance->setObjectName(Parameters::kSuperPointNMSRadius().c_str());
	_ui->checkBox_sptorch_cuda->setObjectName(Parameters::kSuperPointCuda().c_str());

	// PyMatcher
	_ui->lineEdit_pymatcher_path->setObjectName(Parameters::kPyMatcherPath().c_str());
	connect(_ui->toolButton_pymatcher_path, SIGNAL(clicked()), this, SLOT(changePyMatcherPath()));
	_ui->pymatcher_matchThreshold->setObjectName(Parameters::kPyMatcherThreshold().c_str());
	_ui->pymatcher_iterations->setObjectName(Parameters::kPyMatcherIterations().c_str());
	_ui->checkBox_pymatcher_cuda->setObjectName(Parameters::kPyMatcherCuda().c_str());
	_ui->lineEdit_pymatcher_model->setObjectName(Parameters::kPyMatcherModel().c_str());
	connect(_ui->toolButton_pymatcher_model, SIGNAL(clicked()), this, SLOT(changePyMatcherModel()));

	// PyDetector
	_ui->lineEdit_pydetector_path->setObjectName(Parameters::kPyDetectorPath().c_str());
	connect(_ui->toolButton_pydetector_path, SIGNAL(clicked()), this, SLOT(changePyDetectorPath()));
	_ui->checkBox_pydetector_cuda->setObjectName(Parameters::kPyDetectorCuda().c_str());

	// GMS
	_ui->checkBox_gms_withRotation->setObjectName(Parameters::kGMSWithRotation().c_str());
	_ui->checkBox_gms_withScale->setObjectName(Parameters::kGMSWithScale().c_str());
	_ui->gms_thresholdFactor->setObjectName(Parameters::kGMSThresholdFactor().c_str());

	// verifyHypotheses
	_ui->groupBox_vh_epipolar2->setObjectName(Parameters::kVhEpEnabled().c_str());
	_ui->surf_spinBox_matchCountMinAccepted->setObjectName(Parameters::kVhEpMatchCountMin().c_str());
	_ui->surf_doubleSpinBox_ransacParam1->setObjectName(Parameters::kVhEpRansacParam1().c_str());
	_ui->surf_doubleSpinBox_ransacParam2->setObjectName(Parameters::kVhEpRansacParam2().c_str());

	// RGB-D SLAM
	_ui->general_checkBox_activateRGBD->setObjectName(Parameters::kRGBDEnabled().c_str());
	_ui->rgdb_linearUpdate->setObjectName(Parameters::kRGBDLinearUpdate().c_str());
	_ui->rgdb_angularUpdate->setObjectName(Parameters::kRGBDAngularUpdate().c_str());
	_ui->rgdb_linearSpeedUpdate->setObjectName(Parameters::kRGBDLinearSpeedUpdate().c_str());
	_ui->rgdb_angularSpeedUpdate->setObjectName(Parameters::kRGBDAngularSpeedUpdate().c_str());
	_ui->rgbd_savedLocalizationIgnored->setObjectName(Parameters::kRGBDStartAtOrigin().c_str());
	_ui->rgdb_rehearsalWeightIgnoredWhileMoving->setObjectName(Parameters::kMemRehearsalWeightIgnoredWhileMoving().c_str());
	_ui->rgdb_newMapOdomChange->setObjectName(Parameters::kRGBDNewMapOdomChangeDistance().c_str());
	_ui->odomScanHistory->setObjectName(Parameters::kRGBDNeighborLinkRefining().c_str());
	_ui->odomGravity->setObjectName(Parameters::kMemUseOdomGravity().c_str());
	_ui->spinBox_maxLocalLocationsRetrieved->setObjectName(Parameters::kRGBDMaxLocalRetrieved().c_str());
	_ui->rgbd_loopCovLimited->setObjectName(Parameters::kRGBDLoopCovLimited().c_str());

	_ui->graphOptimization_type->setObjectName(Parameters::kOptimizerStrategy().c_str());
	_ui->graphOptimization_iterations->setObjectName(Parameters::kOptimizerIterations().c_str());
	_ui->graphOptimization_covarianceIgnored->setObjectName(Parameters::kOptimizerVarianceIgnored().c_str());
	_ui->graphOptimization_fromGraphEnd->setObjectName(Parameters::kRGBDOptimizeFromGraphEnd().c_str());
	_ui->graphOptimization_maxError->setObjectName(Parameters::kRGBDOptimizeMaxError().c_str());
	_ui->graphOptimization_gravitySigma->setObjectName(Parameters::kOptimizerGravitySigma().c_str());
	_ui->graphOptimization_stopEpsilon->setObjectName(Parameters::kOptimizerEpsilon().c_str());
	_ui->graphOptimization_robust->setObjectName(Parameters::kOptimizerRobust().c_str());
	_ui->graphOptimization_priorsIgnored->setObjectName(Parameters::kOptimizerPriorsIgnored().c_str());
	_ui->graphOptimization_landmarksIgnored->setObjectName(Parameters::kOptimizerLandmarksIgnored().c_str());

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
	_ui->maxLocalizationDistance->setObjectName(Parameters::kRGBDMaxLoopClosureDistance().c_str());
	_ui->localDetection_maxDiffID->setObjectName(Parameters::kRGBDProximityMaxGraphDepth().c_str());
	_ui->localDetection_maxNeighbors->setObjectName(Parameters::kRGBDProximityPathMaxNeighbors().c_str());
	_ui->localDetection_maxPaths->setObjectName(Parameters::kRGBDProximityMaxPaths().c_str());
	_ui->localDetection_pathFilteringRadius->setObjectName(Parameters::kRGBDProximityPathFilteringRadius().c_str());
	_ui->localDetection_angle->setObjectName(Parameters::kRGBDProximityAngle().c_str());
	_ui->localDetection_mergedScanCovFactor->setObjectName(Parameters::kRGBDProximityMergedScanCovFactor().c_str());
	_ui->checkBox_localSpaceOdomGuess->setObjectName(Parameters::kRGBDProximityOdomGuess().c_str());
	_ui->checkBox_localSpacePathOdomPosesUsed->setObjectName(Parameters::kRGBDProximityPathRawPosesUsed().c_str());
	_ui->rgdb_localImmunizationRatio->setObjectName(Parameters::kRGBDLocalImmunizationRatio().c_str());
	_ui->loopClosure_identityGuess->setObjectName(Parameters::kRGBDLoopClosureIdentityGuess().c_str());
	_ui->loopClosure_reextract->setObjectName(Parameters::kRGBDLoopClosureReextractFeatures().c_str());
	_ui->loopClosure_bunlde->setObjectName(Parameters::kRGBDLocalBundleOnLoopClosure().c_str());
	_ui->loopClosure_invertedReg->setObjectName(Parameters::kRGBDInvertedReg().c_str());
	_ui->checkbox_rgbd_createOccupancyGrid->setObjectName(Parameters::kRGBDCreateOccupancyGrid().c_str());
	_ui->RGBDMarkerDetection->setObjectName(Parameters::kRGBDMarkerDetection().c_str());
	_ui->spinBox_maxOdomCacheSize->setObjectName(Parameters::kRGBDMaxOdomCacheSize().c_str());

	// Registration
	_ui->reg_repeatOnce->setObjectName(Parameters::kRegRepeatOnce().c_str());
	_ui->comboBox_registrationStrategy->setObjectName(Parameters::kRegStrategy().c_str());
	_ui->loopClosure_bowForce2D->setObjectName(Parameters::kRegForce3DoF().c_str());

	//RegistrationVis
	_ui->loopClosure_bowMinInliers->setObjectName(Parameters::kVisMinInliers().c_str());
	_ui->loopClosure_bowInlierDistance->setObjectName(Parameters::kVisInlierDistance().c_str());
	_ui->loopClosure_bowIterations->setObjectName(Parameters::kVisIterations().c_str());
	_ui->loopClosure_bowRefineIterations->setObjectName(Parameters::kVisRefineIterations().c_str());
	_ui->visMeanDistance->setObjectName(Parameters::kVisMeanInliersDistance().c_str());
	_ui->visMinDistribution->setObjectName(Parameters::kVisMinInliersDistribution().c_str());
	_ui->loopClosure_estimationType->setObjectName(Parameters::kVisEstimationType().c_str());
	connect(_ui->loopClosure_estimationType, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_loopClosureEstimation, SLOT(setCurrentIndex(int)));
	_ui->stackedWidget_loopClosureEstimation->setCurrentIndex(Parameters::defaultVisEstimationType());
	_ui->loopClosure_forwardEst->setObjectName(Parameters::kVisForwardEstOnly().c_str());
	_ui->loopClosure_bowEpipolarGeometryVar->setObjectName(Parameters::kVisEpipolarGeometryVar().c_str());
	_ui->loopClosure_pnpReprojError->setObjectName(Parameters::kVisPnPReprojError().c_str());
	_ui->loopClosure_pnpFlags->setObjectName(Parameters::kVisPnPFlags().c_str());
	_ui->loopClosure_pnpRefineIterations->setObjectName(Parameters::kVisPnPRefineIterations().c_str());
	_ui->loopClosure_pnpMaxVariance->setObjectName(Parameters::kVisPnPMaxVariance().c_str());
	_ui->reextract_nn->setObjectName(Parameters::kVisCorNNType().c_str());
	connect(_ui->reextract_nn, SIGNAL(currentIndexChanged(int)), this, SLOT(updateFeatureMatchingVisibility()));
	_ui->reextract_nndrRatio->setObjectName(Parameters::kVisCorNNDR().c_str());
	_ui->spinBox_visCorGuessWinSize->setObjectName(Parameters::kVisCorGuessWinSize().c_str());
	_ui->checkBox__visCorGuessMatchToProjection->setObjectName(Parameters::kVisCorGuessMatchToProjection().c_str());
	_ui->vis_feature_detector->setObjectName(Parameters::kVisFeatureType().c_str());
	_ui->reextract_maxFeatures->setObjectName(Parameters::kVisMaxFeatures().c_str());
	_ui->reextract_gridrows->setObjectName(Parameters::kVisGridRows().c_str());
	_ui->reextract_gridcols->setObjectName(Parameters::kVisGridCols().c_str());
	_ui->loopClosure_bowMaxDepth->setObjectName(Parameters::kVisMaxDepth().c_str());
	_ui->loopClosure_bowMinDepth->setObjectName(Parameters::kVisMinDepth().c_str());
	_ui->checkBox_visDepthAsMask->setObjectName(Parameters::kVisDepthAsMask().c_str());
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
	_ui->comboBox_icpStrategy->setObjectName(Parameters::kIcpStrategy().c_str());
	connect(_ui->comboBox_icpStrategy, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_icpStrategy, SLOT(setCurrentIndex(int)));
	_ui->comboBox_icpStrategy->setCurrentIndex(Parameters::defaultIcpStrategy());
	_ui->globalDetection_icpMaxTranslation->setObjectName(Parameters::kIcpMaxTranslation().c_str());
	_ui->globalDetection_icpMaxRotation->setObjectName(Parameters::kIcpMaxRotation().c_str());
	_ui->loopClosure_icpVoxelSize->setObjectName(Parameters::kIcpVoxelSize().c_str());
	_ui->loopClosure_icpDownsamplingStep->setObjectName(Parameters::kIcpDownsamplingStep().c_str());
	_ui->loopClosure_icpRangeMin->setObjectName(Parameters::kIcpRangeMin().c_str());
	_ui->loopClosure_icpRangeMax->setObjectName(Parameters::kIcpRangeMax().c_str());
	_ui->loopClosure_icpMaxCorrespondenceDistance->setObjectName(Parameters::kIcpMaxCorrespondenceDistance().c_str());
	_ui->loopClosure_icpIterations->setObjectName(Parameters::kIcpIterations().c_str());
	_ui->loopClosure_icpEpsilon->setObjectName(Parameters::kIcpEpsilon().c_str());
	_ui->loopClosure_icpRatio->setObjectName(Parameters::kIcpCorrespondenceRatio().c_str());
	_ui->doubleSpinBox_icpOutlierRatio->setObjectName(Parameters::kIcpOutlierRatio().c_str());
	_ui->loopClosure_icpPointToPlane->setObjectName(Parameters::kIcpPointToPlane().c_str());
	_ui->loopClosure_icpPointToPlaneNormals->setObjectName(Parameters::kIcpPointToPlaneK().c_str());
	_ui->loopClosure_icpPointToPlaneNormalsRadius->setObjectName(Parameters::kIcpPointToPlaneRadius().c_str());
	_ui->loopClosure_icpPointToPlaneGroundNormalsUp->setObjectName(Parameters::kIcpPointToPlaneGroundNormalsUp().c_str());
	_ui->loopClosure_icpPointToPlaneNormalsMinComplexity->setObjectName(Parameters::kIcpPointToPlaneMinComplexity().c_str());
	_ui->loopClosure_icpPointToPlaneLowComplexityStrategy->setObjectName(Parameters::kIcpPointToPlaneLowComplexityStrategy().c_str());
	_ui->loopClosure_icpDebugExportFormat->setObjectName(Parameters::kIcpDebugExportFormat().c_str());

	_ui->lineEdit_IcpPMConfigPath->setObjectName(Parameters::kIcpPMConfig().c_str());
	connect(_ui->toolButton_IcpConfigPath, SIGNAL(clicked()), this, SLOT(changeIcpPMConfigPath()));
	_ui->spinBox_icpPMMatcherKnn->setObjectName(Parameters::kIcpPMMatcherKnn().c_str());
	_ui->doubleSpinBox_icpPMMatcherEpsilon->setObjectName(Parameters::kIcpPMMatcherEpsilon().c_str());
	_ui->loopClosure_icpPMMatcherIntensity->setObjectName(Parameters::kIcpPMMatcherIntensity().c_str());
	_ui->loopClosure_icpForce4DoF->setObjectName(Parameters::kIcpForce4DoF().c_str());

	_ui->spinBox_icpCCSamplingLimit->setObjectName(Parameters::kIcpCCSamplingLimit().c_str());
	_ui->checkBox_icpCCFilterOutFarthestPoints->setObjectName(Parameters::kIcpCCFilterOutFarthestPoints().c_str());
	_ui->doubleSpinBox_icpCCMaxFinalRMS->setObjectName(Parameters::kIcpCCMaxFinalRMS().c_str());


	// Occupancy grid
	_ui->groupBox_grid_3d->setObjectName(Parameters::kGrid3D().c_str());
	_ui->checkBox_grid_groundObstacle->setObjectName(Parameters::kGridGroundIsObstacle().c_str());
	_ui->doubleSpinBox_grid_resolution->setObjectName(Parameters::kGridCellSize().c_str());
	_ui->checkBox_grid_preVoxelFiltering->setObjectName(Parameters::kGridPreVoxelFiltering().c_str());
	_ui->spinBox_grid_decimation->setObjectName(Parameters::kGridDepthDecimation().c_str());
	_ui->doubleSpinBox_grid_maxDepth->setObjectName(Parameters::kGridRangeMax().c_str());
	_ui->doubleSpinBox_grid_minDepth->setObjectName(Parameters::kGridRangeMin().c_str());
	_ui->lineEdit_grid_roi->setObjectName(Parameters::kGridDepthRoiRatios().c_str());
	_ui->checkBox_grid_projRayTracing->setObjectName(Parameters::kGridRayTracing().c_str());
	_ui->doubleSpinBox_grid_footprintLength->setObjectName(Parameters::kGridFootprintLength().c_str());
	_ui->doubleSpinBox_grid_footprintWidth->setObjectName(Parameters::kGridFootprintWidth().c_str());
	_ui->doubleSpinBox_grid_footprintHeight->setObjectName(Parameters::kGridFootprintHeight().c_str());
	_ui->checkBox_grid_flatObstaclesDetected->setObjectName(Parameters::kGridFlatObstacleDetected().c_str());
	_ui->comboBox_grid_sensor->setObjectName(Parameters::kGridSensor().c_str());
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
	_ui->spinBox_grid_scanDecimation->setObjectName(Parameters::kGridScanDecimation().c_str());

	_ui->checkBox_grid_fullUpdate->setObjectName(Parameters::kGridGlobalFullUpdate().c_str());
	_ui->doubleSpinBox_grid_updateError->setObjectName(Parameters::kGridGlobalUpdateError().c_str());
	_ui->doubleSpinBox_grid_minMapSize->setObjectName(Parameters::kGridGlobalMinSize().c_str());
	_ui->spinBox_grid_maxNodes->setObjectName(Parameters::kGridGlobalMaxNodes().c_str());
	_ui->doubleSpinBox_grid_altitudeDelta->setObjectName(Parameters::kGridGlobalAltitudeDelta().c_str());
	_ui->doubleSpinBox_grid_footprintRadius->setObjectName(Parameters::kGridGlobalFootprintRadius().c_str());
	_ui->doubleSpinBox_grid_occThr->setObjectName(Parameters::kGridGlobalOccupancyThr().c_str());
	_ui->doubleSpinBox_grid_probHit->setObjectName(Parameters::kGridGlobalProbHit().c_str());
	_ui->doubleSpinBox_grid_probMiss->setObjectName(Parameters::kGridGlobalProbMiss().c_str());
	_ui->doubleSpinBox_grid_clampingMin->setObjectName(Parameters::kGridGlobalProbClampingMin().c_str());
	_ui->doubleSpinBox_grid_clampingMax->setObjectName(Parameters::kGridGlobalProbClampingMax().c_str());
	_ui->checkBox_grid_erode->setObjectName(Parameters::kGridGlobalEroded().c_str());
	_ui->spinBox_grid_floodfilldepth->setObjectName(Parameters::kGridGlobalFloodFillDepth().c_str());

	//Odometry
	_ui->odom_strategy->setObjectName(Parameters::kOdomStrategy().c_str());
	connect(_ui->odom_strategy, SIGNAL(currentIndexChanged(int)), this, SLOT(updateOdometryStackedIndex(int)));
	_ui->odom_strategy->setCurrentIndex(Parameters::defaultOdomStrategy());
	updateOdometryStackedIndex(Parameters::defaultOdomStrategy());
	_ui->odom_countdown->setObjectName(Parameters::kOdomResetCountdown().c_str());
	_ui->odom_holonomic->setObjectName(Parameters::kOdomHolonomic().c_str());
	_ui->odom_fillInfoData->setObjectName(Parameters::kOdomFillInfoData().c_str());
	_ui->odom_dataBufferSize->setObjectName(Parameters::kOdomImageBufferSize().c_str());
	_ui->odom_flow_keyframeThr->setObjectName(Parameters::kOdomKeyFrameThr().c_str());
	_ui->odom_VisKeyFrameThr->setObjectName(Parameters::kOdomVisKeyFrameThr().c_str());
	_ui->odom_flow_scanKeyframeThr->setObjectName(Parameters::kOdomScanKeyFrameThr().c_str());
	_ui->odom_flow_guessMotion->setObjectName(Parameters::kOdomGuessMotion().c_str());
	_ui->odom_guess_smoothing_delay->setObjectName(Parameters::kOdomGuessSmoothingDelay().c_str());
	_ui->odom_imageDecimation->setObjectName(Parameters::kOdomImageDecimation().c_str());
	_ui->odom_alignWithGround->setObjectName(Parameters::kOdomAlignWithGround().c_str());

	//Odometry Frame to Map
	_ui->odom_localHistory->setObjectName(Parameters::kOdomF2MMaxSize().c_str());
	_ui->spinBox_odom_f2m_maxNewFeatures->setObjectName(Parameters::kOdomF2MMaxNewFeatures().c_str());
	_ui->spinBox_odom_f2m_scanMaxSize->setObjectName(Parameters::kOdomF2MScanMaxSize().c_str());
	_ui->doubleSpinBox_odom_f2m_scanRadius->setObjectName(Parameters::kOdomF2MScanSubtractRadius().c_str());
	_ui->doubleSpinBox_odom_f2m_scanAngle->setObjectName(Parameters::kOdomF2MScanSubtractAngle().c_str());
	_ui->doubleSpinBox_odom_f2m_scanRange->setObjectName(Parameters::kOdomF2MScanRange().c_str());
	_ui->odom_f2m_validDepthRatio->setObjectName(Parameters::kOdomF2MValidDepthRatio().c_str());
	_ui->odom_f2m_bundleStrategy->setObjectName(Parameters::kOdomF2MBundleAdjustment().c_str());
	_ui->odom_f2m_bundleMaxFrames->setObjectName(Parameters::kOdomF2MBundleAdjustmentMaxFrames().c_str());

	//Odometry Frame To Frame
	_ui->comboBox_odomf2f_corType->setObjectName(Parameters::kVisCorType().c_str());

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

	//Odometry Fovis
	_ui->spinBox_OdomFovisFeatureWindowSize->setObjectName(Parameters::kOdomFovisFeatureWindowSize().c_str());
	_ui->spinBox_OdomFovisMaxPyramidLevel->setObjectName(Parameters::kOdomFovisMaxPyramidLevel().c_str());
	_ui->spinBox_OdomFovisMinPyramidLevel->setObjectName(Parameters::kOdomFovisMinPyramidLevel().c_str());
	_ui->spinBox_OdomFovisTargetPixelsPerFeature->setObjectName(Parameters::kOdomFovisTargetPixelsPerFeature().c_str());
	_ui->spinBox_OdomFovisFastThreshold->setObjectName(Parameters::kOdomFovisFastThreshold().c_str());
	_ui->checkBox_OdomFovisUseAdaptiveThreshold->setObjectName(Parameters::kOdomFovisUseAdaptiveThreshold().c_str());
	_ui->doubleSpinBox_OdomFovisFastThresholdAdaptiveGain->setObjectName(Parameters::kOdomFovisFastThresholdAdaptiveGain().c_str());
	_ui->checkBox_OdomFovisUseHomographyInitialization->setObjectName(Parameters::kOdomFovisUseHomographyInitialization().c_str());

	_ui->checkBox_OdomFovisUseBucketing->setObjectName(Parameters::kOdomFovisUseBucketing().c_str());
	_ui->spinBox_OdomFovisBucketWidth->setObjectName(Parameters::kOdomFovisBucketWidth().c_str());
	_ui->spinBox_OdomFovisBucketHeight->setObjectName(Parameters::kOdomFovisBucketHeight().c_str());
	_ui->spinBox_OdomFovisMaxKeypointsPerBucket->setObjectName(Parameters::kOdomFovisMaxKeypointsPerBucket().c_str());
	_ui->checkBox_OdomFovisUseImageNormalization->setObjectName(Parameters::kOdomFovisUseImageNormalization().c_str());

	_ui->doubleSpinBox_OdomFovisInlierMaxReprojectionError->setObjectName(Parameters::kOdomFovisInlierMaxReprojectionError().c_str());
	_ui->doubleSpinBox_OdomFovisCliqueInlierThreshold->setObjectName(Parameters::kOdomFovisCliqueInlierThreshold().c_str());
	_ui->spinBox_OdomFovisMinFeaturesForEstimate->setObjectName(Parameters::kOdomFovisMinFeaturesForEstimate().c_str());
	_ui->doubleSpinBox_OdomFovisMaxMeanReprojectionError->setObjectName(Parameters::kOdomFovisMaxMeanReprojectionError().c_str());
	_ui->checkBox_OdomFovisUseSubpixelRefinement->setObjectName(Parameters::kOdomFovisUseSubpixelRefinement().c_str());
	_ui->spinBox_OdomFovisFeatureSearchWindow->setObjectName(Parameters::kOdomFovisFeatureSearchWindow().c_str());
	_ui->checkBox_OdomFovisUpdateTargetFeaturesWithRefined->setObjectName(Parameters::kOdomFovisUpdateTargetFeaturesWithRefined().c_str());

	_ui->checkBox_OdomFovisStereoRequireMutualMatch->setObjectName(Parameters::kOdomFovisStereoRequireMutualMatch().c_str());
	_ui->doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine->setObjectName(Parameters::kOdomFovisStereoMaxDistEpipolarLine().c_str());
	_ui->doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement->setObjectName(Parameters::kOdomFovisStereoMaxRefinementDisplacement().c_str());
	_ui->spinBox_OdomFovisStereoMaxDisparity->setObjectName(Parameters::kOdomFovisStereoMaxDisparity().c_str());

	// Odometry viso2
	_ui->spinBox_OdomViso2RansacIters->setObjectName(Parameters::kOdomViso2RansacIters().c_str());
	_ui->doubleSpinBox_OdomViso2InlierThreshold->setObjectName(Parameters::kOdomViso2InlierThreshold().c_str());
	_ui->checkBox_OdomViso2Reweighting->setObjectName(Parameters::kOdomViso2Reweighting().c_str());

	_ui->spinBox_OdomViso2MatchNmsN->setObjectName(Parameters::kOdomViso2MatchNmsN().c_str());
	_ui->spinBox_OdomViso2MatchNmsTau->setObjectName(Parameters::kOdomViso2MatchNmsTau().c_str());
	_ui->spinBox_OdomViso2MatchBinsize->setObjectName(Parameters::kOdomViso2MatchBinsize().c_str());
	_ui->spinBox_OdomViso2MatchRadius->setObjectName(Parameters::kOdomViso2MatchRadius().c_str());
	_ui->spinBox_OdomViso2MatchDispTolerance->setObjectName(Parameters::kOdomViso2MatchDispTolerance().c_str());
	_ui->spinBox_OdomViso2MatchOutlierDispTolerance->setObjectName(Parameters::kOdomViso2MatchOutlierDispTolerance().c_str());
	_ui->spinBox_OdomViso2MatchOutlierFlowTolerance->setObjectName(Parameters::kOdomViso2MatchOutlierFlowTolerance().c_str());
	_ui->checkBox_OdomViso2MatchMultiStage->setObjectName(Parameters::kOdomViso2MatchMultiStage().c_str());
	_ui->checkBox_OdomViso2MatchHalfResolution->setObjectName(Parameters::kOdomViso2MatchHalfResolution().c_str());
	_ui->spinBox_OdomViso2MatchRefinement->setObjectName(Parameters::kOdomViso2MatchRefinement().c_str());

	_ui->spinBox_OdomViso2BucketMaxFeatures->setObjectName(Parameters::kOdomViso2BucketMaxFeatures().c_str());
	_ui->doubleSpinBox_OdomViso2BucketWidth->setObjectName(Parameters::kOdomViso2BucketWidth().c_str());
	_ui->doubleSpinBox_OdomViso2BucketHeight->setObjectName(Parameters::kOdomViso2BucketHeight().c_str());

	// Odometry ORBSLAM
	_ui->lineEdit_OdomORBSLAMVocPath->setObjectName(Parameters::kOdomORBSLAMVocPath().c_str());
	connect(_ui->toolButton_OdomORBSLAMVocPath, SIGNAL(clicked()), this, SLOT(changeOdometryORBSLAMVocabulary()));
	_ui->doubleSpinBox_OdomORBSLAMBf->setObjectName(Parameters::kOdomORBSLAMBf().c_str());
	_ui->doubleSpinBox_OdomORBSLAMThDepth->setObjectName(Parameters::kOdomORBSLAMThDepth().c_str());
	_ui->doubleSpinBox_OdomORBSLAMFps->setObjectName(Parameters::kOdomORBSLAMFps().c_str());
	_ui->spinBox_OdomORBSLAMMaxFeatures->setObjectName(Parameters::kOdomORBSLAMMaxFeatures().c_str());
	_ui->spinBox_OdomORBSLAMMapSize->setObjectName(Parameters::kOdomORBSLAMMapSize().c_str());

	// Odometry Okvis
	_ui->lineEdit_OdomOkvisPath->setObjectName(Parameters::kOdomOKVISConfigPath().c_str());
	connect(_ui->toolButton_OdomOkvisPath, SIGNAL(clicked()), this, SLOT(changeOdometryOKVISConfigPath()));

	// Odometry LOAM
	_ui->odom_loam_sensor->setObjectName(Parameters::kOdomLOAMSensor().c_str());
	_ui->odom_loam_scan_period->setObjectName(Parameters::kOdomLOAMScanPeriod().c_str());
	_ui->odom_loam_resolution->setObjectName(Parameters::kOdomLOAMResolution().c_str());
	_ui->odom_loam_linvar->setObjectName(Parameters::kOdomLOAMLinVar().c_str());
	_ui->odom_loam_angvar->setObjectName(Parameters::kOdomLOAMAngVar().c_str());
	_ui->odom_loam_localMapping->setObjectName(Parameters::kOdomLOAMLocalMapping().c_str());

	// Odometry MSCKF_VIO
	_ui->OdomMSCKFGridRow->setObjectName(Parameters::kOdomMSCKFGridRow().c_str());
	_ui->OdomMSCKFGridCol->setObjectName(Parameters::kOdomMSCKFGridCol().c_str());
	_ui->OdomMSCKFGridMinFeatureNum->setObjectName(Parameters::kOdomMSCKFGridMinFeatureNum().c_str());
	_ui->OdomMSCKFGridMaxFeatureNum->setObjectName(Parameters::kOdomMSCKFGridMaxFeatureNum().c_str());
	_ui->OdomMSCKFPyramidLevels->setObjectName(Parameters::kOdomMSCKFPyramidLevels().c_str());
	_ui->OdomMSCKFPatchSize->setObjectName(Parameters::kOdomMSCKFPatchSize().c_str());
	_ui->OdomMSCKFFastThreshold->setObjectName(Parameters::kOdomMSCKFFastThreshold().c_str());
	_ui->OdomMSCKFMaxIteration->setObjectName(Parameters::kOdomMSCKFMaxIteration().c_str());
	_ui->OdomMSCKFMaxCamStateSize->setObjectName(Parameters::kOdomMSCKFMaxCamStateSize().c_str());
	_ui->OdomMSCKFTrackPrecision->setObjectName(Parameters::kOdomMSCKFTrackPrecision().c_str());
	_ui->OdomMSCKFRansacThreshold->setObjectName(Parameters::kOdomMSCKFRansacThreshold().c_str());
	_ui->OdomMSCKFStereoThreshold->setObjectName(Parameters::kOdomMSCKFStereoThreshold().c_str());
	_ui->OdomMSCKFPositionStdThreshold->setObjectName(Parameters::kOdomMSCKFPositionStdThreshold().c_str());
	_ui->OdomMSCKFRotationThreshold->setObjectName(Parameters::kOdomMSCKFRotationThreshold().c_str());
	_ui->OdomMSCKFTranslationThreshold->setObjectName(Parameters::kOdomMSCKFTranslationThreshold().c_str());
	_ui->OdomMSCKFTrackingRateThreshold->setObjectName(Parameters::kOdomMSCKFTrackingRateThreshold().c_str());
	_ui->OdomMSCKFOptTranslationThreshold->setObjectName(Parameters::kOdomMSCKFOptTranslationThreshold().c_str());
	_ui->OdomMSCKFNoiseGyro->setObjectName(Parameters::kOdomMSCKFNoiseGyro().c_str());
	_ui->OdomMSCKFNoiseAcc->setObjectName(Parameters::kOdomMSCKFNoiseAcc().c_str());
	_ui->OdomMSCKFNoiseGyroBias->setObjectName(Parameters::kOdomMSCKFNoiseGyroBias().c_str());
	_ui->OdomMSCKFNoiseAccBias->setObjectName(Parameters::kOdomMSCKFNoiseAccBias().c_str());
	_ui->OdomMSCKFNoiseFeature->setObjectName(Parameters::kOdomMSCKFNoiseFeature().c_str());
	_ui->OdomMSCKFInitCovVel->setObjectName(Parameters::kOdomMSCKFInitCovVel().c_str());
	_ui->OdomMSCKFInitCovGyroBias->setObjectName(Parameters::kOdomMSCKFInitCovGyroBias().c_str());
	_ui->OdomMSCKFInitCovAccBias->setObjectName(Parameters::kOdomMSCKFInitCovAccBias().c_str());
	_ui->OdomMSCKFInitCovExRot->setObjectName(Parameters::kOdomMSCKFInitCovExRot().c_str());
	_ui->OdomMSCKFInitCovExTrans->setObjectName(Parameters::kOdomMSCKFInitCovExTrans().c_str());

	// Odometry VINS
	_ui->lineEdit_OdomVinsPath->setObjectName(Parameters::kOdomVINSConfigPath().c_str());
	connect(_ui->toolButton_OdomVinsPath, SIGNAL(clicked()), this, SLOT(changeOdometryVINSConfigPath()));

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

	// Odometry Open3D
	_ui->odom_open3d_method->setObjectName(Parameters::kOdomOpen3DMethod().c_str());
	_ui->odom_open3d_max_depth->setObjectName(Parameters::kOdomOpen3DMaxDepth().c_str());

	//StereoDense
	_ui->comboBox_stereoDense_strategy->setObjectName(Parameters::kStereoDenseStrategy().c_str());
	connect(_ui->comboBox_stereoDense_strategy, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_stereoDense, SLOT(setCurrentIndex(int)));
	_ui->comboBox_stereoDense_strategy->setCurrentIndex(Parameters::defaultStereoDenseStrategy());
	_ui->stackedWidget_stereoDense->setCurrentIndex(Parameters::defaultStereoDenseStrategy());

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
	_ui->stereobm_disp12MaxDiff->setObjectName(Parameters::kStereoBMDisp12MaxDiff().c_str());

	//StereoSGBM
	_ui->stereosgbm_blockSize->setObjectName(Parameters::kStereoSGBMBlockSize().c_str());
	_ui->stereosgbm_minDisparity->setObjectName(Parameters::kStereoSGBMMinDisparity().c_str());
	_ui->stereosgbm_numDisparities->setObjectName(Parameters::kStereoSGBMNumDisparities().c_str());
	_ui->stereosgbm_preFilterCap->setObjectName(Parameters::kStereoSGBMPreFilterCap().c_str());
	_ui->stereosgbm_speckleRange->setObjectName(Parameters::kStereoSGBMSpeckleRange().c_str());
	_ui->stereosgbm_speckleWinSize->setObjectName(Parameters::kStereoSGBMSpeckleWindowSize().c_str());
	_ui->stereosgbm_uniquessRatio->setObjectName(Parameters::kStereoSGBMUniquenessRatio().c_str());
	_ui->stereosgbm_disp12MaxDiff->setObjectName(Parameters::kStereoSGBMDisp12MaxDiff().c_str());
	_ui->stereosgbm_p1->setObjectName(Parameters::kStereoSGBMP1().c_str());
	_ui->stereosgbm_p2->setObjectName(Parameters::kStereoSGBMP2().c_str());
	_ui->stereosgbm_mode->setObjectName(Parameters::kStereoSGBMMode().c_str());

	// Aruco marker
	_ui->ArucoDictionary->setObjectName(Parameters::kMarkerDictionary().c_str());
	_ui->ArucoMarkerLength->setObjectName(Parameters::kMarkerLength().c_str());
	_ui->ArucoMaxDepthError->setObjectName(Parameters::kMarkerMaxDepthError().c_str());
	_ui->ArucoVarianceLinear->setObjectName(Parameters::kMarkerVarianceLinear().c_str());
	_ui->ArucoVarianceAngular->setObjectName(Parameters::kMarkerVarianceAngular().c_str());
	_ui->ArucoMarkerRangeMin->setObjectName(Parameters::kMarkerMinRange().c_str());
	_ui->ArucoMarkerRangeMax->setObjectName(Parameters::kMarkerMaxRange().c_str());
	_ui->ArucoMarkerPriors->setObjectName(Parameters::kMarkerPriors().c_str());
	_ui->ArucoPriorsVarianceLinear->setObjectName(Parameters::kMarkerPriorsVarianceLinear().c_str());
	_ui->ArucoPriorsVarianceAngular->setObjectName(Parameters::kMarkerPriorsVarianceAngular().c_str());
	_ui->ArucoCornerRefinementMethod->setObjectName(Parameters::kMarkerCornerRefinementMethod().c_str());

	// IMU filter
	_ui->doubleSpinBox_imuFilterMadgwickGain->setObjectName(Parameters::kImuFilterMadgwickGain().c_str());
	_ui->doubleSpinBox_imuFilterMadgwickZeta->setObjectName(Parameters::kImuFilterMadgwickZeta().c_str());
	_ui->doubleSpinBox_imuFilterComplementaryGainAcc->setObjectName(Parameters::kImuFilterComplementaryGainAcc().c_str());
	_ui->doubleSpinBox_imuFilterComplementaryBiasAlpha->setObjectName(Parameters::kImuFilterComplementaryBiasAlpha().c_str());
	_ui->checkBox_imuFilterComplementaryDoAdaptiveGain->setObjectName(Parameters::kImuFilterComplementaryDoAdpativeGain().c_str());
	_ui->checkBox_imuFilterComplementaryDoBiasEstimation->setObjectName(Parameters::kImuFilterComplementaryDoBiasEstimation().c_str());

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

void PreferencesDialog::init(const QString & iniFilePath)
{
	UDEBUG("");
	//First set all default values
	const ParametersMap & defaults = Parameters::getDefaultParameters();
	for(ParametersMap::const_iterator iter=defaults.begin(); iter!=defaults.end(); ++iter)
	{
		this->setParameter(iter->first, iter->second);
	}

	this->readSettings(iniFilePath);
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
		_ui->checkBox_odom_onlyInliersShown->setChecked(false);
		_ui->radioButton_posteriorGraphView->setChecked(true);
		_ui->radioButton_wordsGraphView->setChecked(false);
		_ui->radioButton_localizationsGraphView->setChecked(false);
		_ui->radioButton_nochangeGraphView->setChecked(false);
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
			_3dRenderingShowFrustums[i]->setChecked(false);

			_3dRenderingDownsamplingScan[i]->setValue(1);
			_3dRenderingMaxRange[i]->setValue(0.0);
			_3dRenderingMinRange[i]->setValue(0.0);
			_3dRenderingVoxelSizeScan[i]->setValue(0.0);
			_3dRenderingColorScheme[i]->setValue(0);
			_3dRenderingOpacity[i]->setValue(i==0?1.0:0.75);
			_3dRenderingPtSize[i]->setValue(i==0?1:2);
			_3dRenderingColorSchemeScan[i]->setValue(0);
			_3dRenderingOpacityScan[i]->setValue(i==0?1.0:0.5);
			_3dRenderingPtSizeScan[i]->setValue(i==0?1:2);
			_3dRenderingPtSizeFeatures[i]->setValue(3);
			_3dRenderingGravity[i]->setChecked(false);
			_3dRenderingGravityLength[i]->setValue(1);
		}
		_ui->doubleSpinBox_voxel->setValue(0);
		_ui->doubleSpinBox_noiseRadius->setValue(0);
		_ui->spinBox_noiseMinNeighbors->setValue(5);

		_ui->doubleSpinBox_ceilingFilterHeight->setValue(0);
		_ui->doubleSpinBox_floorFilterHeight->setValue(0);
		_ui->spinBox_normalKSearch->setValue(10);
		_ui->doubleSpinBox_normalRadiusSearch->setValue(0.0);

		_ui->doubleSpinBox_ceilingFilterHeight_scan->setValue(0);
		_ui->doubleSpinBox_floorFilterHeight_scan->setValue(0);
		_ui->spinBox_normalKSearch_scan->setValue(0);
		_ui->doubleSpinBox_normalRadiusSearch_scan->setValue(0.0);

		_ui->checkBox_showGraphs->setChecked(true);
		_ui->checkBox_showLabels->setChecked(false);
		_ui->checkBox_showFrames->setChecked(false);
		_ui->checkBox_showLandmarks->setChecked(true);
		_ui->doubleSpinBox_landmarkSize->setValue(0);
		_ui->checkBox_showIMUGravity->setChecked(false);
		_ui->checkBox_showIMUAcc->setChecked(false);

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
		_ui->doubleSpinBox_map_opacity->setValue(0.75);

		_ui->groupBox_octomap->setChecked(false);
		_ui->spinBox_octomap_treeDepth->setValue(16);
		_ui->checkBox_octomap_2dgrid->setChecked(true);
		_ui->checkBox_octomap_show3dMap->setChecked(true);
		_ui->comboBox_octomap_renderingType->setCurrentIndex(0);
		_ui->spinBox_octomap_pointSize->setValue(5);
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
		_ui->lineEdit_sourceLocalTransform->setText("0 0 0 0 0 0");

		_ui->source_comboBox_image_type->setCurrentIndex(kSrcUsbDevice-kSrcUsbDevice);
		_ui->source_images_spinBox_startPos->setValue(0);
		_ui->source_images_spinBox_maxFrames->setValue(0);
		_ui->spinBox_usbcam_streamWidth->setValue(0);
		_ui->spinBox_usbcam_streamHeight->setValue(0);
		_ui->checkBox_rgb_rectify->setChecked(false);
		_ui->comboBox_cameraImages_bayerMode->setCurrentIndex(0);

		_ui->source_checkBox_ignoreOdometry->setChecked(false);
		_ui->source_checkBox_ignoreGoalDelay->setChecked(true);
		_ui->source_checkBox_ignoreGoals->setChecked(true);
		_ui->source_checkBox_ignoreLandmarks->setChecked(true);
		_ui->source_checkBox_ignoreFeatures->setChecked(true);
		_ui->source_spinBox_databaseStartId->setValue(0);
		_ui->source_spinBox_databaseStopId->setValue(0);
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
		_ui->checkBox_stereo_exposureCompensation->setChecked(false);
		_ui->openni2_autoWhiteBalance->setChecked(true);
		_ui->openni2_autoExposure->setChecked(true);
		_ui->openni2_exposure->setValue(0);
		_ui->openni2_gain->setValue(100);
		_ui->openni2_mirroring->setChecked(false);
		_ui->openni2_stampsIdsUsed->setChecked(false);
		_ui->openni2_hshift->setValue(0);
		_ui->openni2_vshift->setValue(0);
		_ui->openni2_depth_decimation->setValue(1);
		_ui->comboBox_freenect2Format->setCurrentIndex(1);
		_ui->doubleSpinBox_freenect2MinDepth->setValue(0.3);
		_ui->doubleSpinBox_freenect2MaxDepth->setValue(12.0);
		_ui->checkBox_freenect2BilateralFiltering->setChecked(true);
		_ui->checkBox_freenect2EdgeAwareFiltering->setChecked(true);
		_ui->checkBox_freenect2NoiseFiltering->setChecked(true);
		_ui->lineEdit_freenect2Pipeline->setText("");
		_ui->comboBox_k4w2Format->setCurrentIndex(1);
		_ui->comboBox_realsensePresetRGB->setCurrentIndex(0);
		_ui->comboBox_realsensePresetDepth->setCurrentIndex(2);
		_ui->checkbox_realsenseOdom->setChecked(false);
		_ui->checkbox_realsenseDepthScaledToRGBSize->setChecked(false);
		_ui->comboBox_realsenseRGBSource->setCurrentIndex(0);
		_ui->checkbox_rs2_emitter->setChecked(true);
		_ui->checkbox_rs2_irMode->setChecked(false);
		_ui->checkbox_rs2_irDepth->setChecked(true);
		_ui->spinBox_rs2_width->setValue(848);
		_ui->spinBox_rs2_height->setValue(480);
		_ui->spinBox_rs2_rate->setValue(60);
		_ui->spinBox_rs2_width_depth->setValue(640);
		_ui->spinBox_rs2_height_depth->setValue(480);
		_ui->spinBox_rs2_rate_depth->setValue(30);
		_ui->checkbox_rs2_globalTimeStync->setChecked(true);
		_ui->lineEdit_rs2_jsonFile->clear();
		_ui->lineEdit_openniOniPath->clear();
		_ui->lineEdit_openni2OniPath->clear();
		_ui->comboBox_k4a_rgb_resolution->setCurrentIndex(0);
		_ui->comboBox_k4a_framerate->setCurrentIndex(2);
		_ui->comboBox_k4a_depth_resolution->setCurrentIndex(2);
		_ui->checkbox_k4a_irDepth->setChecked(false);
		_ui->lineEdit_k4a_mkv->clear();
		_ui->source_checkBox_useMKVStamps->setChecked(true);
		_ui->lineEdit_cameraRGBDImages_path_rgb->setText("");
		_ui->lineEdit_cameraRGBDImages_path_depth->setText("");
		_ui->doubleSpinBox_cameraRGBDImages_scale->setValue(1.0);
		_ui->spinBox_cameraRGBDImages_startIndex->setValue(0);
		_ui->spinBox_cameraRGBDImages_maxFrames->setValue(0);
		_ui->lineEdit_source_distortionModel->setText("");
		_ui->groupBox_bilateral->setChecked(false);
		_ui->doubleSpinBox_bilateral_sigmaS->setValue(10.0);
		_ui->doubleSpinBox_bilateral_sigmaR->setValue(0.1);

		_ui->source_comboBox_image_type->setCurrentIndex(kSrcDC1394-kSrcDC1394);
		_ui->lineEdit_cameraStereoImages_path_left->setText("");
		_ui->lineEdit_cameraStereoImages_path_right->setText("");
		_ui->checkBox_stereo_rectify->setChecked(false);
		_ui->spinBox_cameraStereoImages_startIndex->setValue(0);
		_ui->spinBox_cameraStereoImages_maxFrames->setValue(0);
		_ui->lineEdit_cameraStereoVideo_path->setText("");
		_ui->lineEdit_cameraStereoVideo_path_2->setText("");
		_ui->spinBox_stereo_right_device->setValue(-1);
		_ui->spinBox_stereousbcam_streamWidth->setValue(0);
		_ui->spinBox_stereousbcam_streamHeight->setValue(0);
		_ui->comboBox_stereoZed_resolution->setCurrentIndex(2);
		_ui->comboBox_stereoZed_quality->setCurrentIndex(1);
		_ui->checkbox_stereoZed_selfCalibration->setChecked(true);
		_ui->comboBox_stereoZed_sensingMode->setCurrentIndex(0);
		_ui->spinBox_stereoZed_confidenceThr->setValue(100);
		_ui->spinBox_stereoZed_texturenessConfidenceThr->setValue(90);
		_ui->lineEdit_zedSvoPath->clear();
		_ui->comboBox_stereoZedOC_resolution->setCurrentIndex(3);
		_ui->checkbox_stereoMyntEye_rectify->setChecked(false);
		_ui->checkbox_stereoMyntEye_depth->setChecked(false);
		_ui->checkbox_stereoMyntEye_autoExposure->setChecked(true);
		_ui->spinBox_stereoMyntEye_gain->setValue(24);
		_ui->spinBox_stereoMyntEye_brightness->setValue(120);
		_ui->spinBox_stereoMyntEye_contrast->setValue(116);
		_ui->spinBox_stereoMyntEye_irControl->setValue(0);
		_ui->comboBox_depthai_resolution->setCurrentIndex(1);
		_ui->checkBox_depthai_depth->setChecked(false);
		_ui->spinBox_depthai_confidence->setValue(200);
		_ui->checkBox_depthai_imu_firmware_update->setChecked(false);

		_ui->checkBox_cameraImages_configForEachFrame->setChecked(false);
		_ui->checkBox_cameraImages_timestamps->setChecked(false);
		_ui->checkBox_cameraImages_syncTimeStamps->setChecked(true);
		_ui->lineEdit_cameraImages_timestamps->setText("");
		_ui->lineEdit_cameraImages_path_scans->setText("");
		_ui->lineEdit_cameraImages_laser_transform->setText("0 0 0 0 0 0");
		_ui->spinBox_cameraImages_max_scan_pts->setValue(0);
		_ui->lineEdit_cameraImages_odom->setText("");
		_ui->comboBox_cameraImages_odomFormat->setCurrentIndex(0);
		_ui->lineEdit_cameraImages_gt->setText("");
		_ui->comboBox_cameraImages_gtFormat->setCurrentIndex(0);
		_ui->doubleSpinBox_maxPoseTimeDiff->setValue(0.02);
		_ui->lineEdit_cameraImages_path_imu->setText("");
		_ui->lineEdit_cameraImages_imu_transform->setText("0 0 1 0 -1 0 1 0 0");
		_ui->spinBox_cameraImages_max_imu_rate->setValue(0);

		_ui->comboBox_odom_sensor->setCurrentIndex(0);
		_ui->lineEdit_odom_sensor_extrinsics->setText("-0.000622602 0.0303752 0.031389 -0.00272485 0.00749254 0.0");
		_ui->lineEdit_odom_sensor_path_calibration->setText("");
		_ui->lineEdit_odomSourceDevice->setText("");
		_ui->doubleSpinBox_odom_sensor_time_offset->setValue(0.0);
		_ui->doubleSpinBox_odom_sensor_scale_factor->setValue(1);
		_ui->checkBox_odom_sensor_use_as_gt->setChecked(false);

		_ui->comboBox_imuFilter_strategy->setCurrentIndex(2);
		_ui->doubleSpinBox_imuFilterMadgwickGain->setValue(Parameters::defaultImuFilterMadgwickGain());
		_ui->doubleSpinBox_imuFilterMadgwickZeta->setValue(Parameters::defaultImuFilterMadgwickZeta());
		_ui->doubleSpinBox_imuFilterComplementaryGainAcc->setValue(Parameters::defaultImuFilterComplementaryGainAcc());
		_ui->doubleSpinBox_imuFilterComplementaryBiasAlpha->setValue(Parameters::defaultImuFilterComplementaryBiasAlpha());
		_ui->checkBox_imuFilterComplementaryDoAdaptiveGain->setChecked(Parameters::defaultImuFilterComplementaryDoAdpativeGain());
		_ui->checkBox_imuFilterComplementaryDoBiasEstimation->setChecked(Parameters::defaultImuFilterComplementaryDoBiasEstimation());
		_ui->checkBox_imuFilter_baseFrameConversion->setChecked(true);
		_ui->checkbox_publishInterIMU->setChecked(false);

		_ui->checkBox_source_scanFromDepth->setChecked(false);
		_ui->spinBox_source_scanDownsampleStep->setValue(1);
		_ui->doubleSpinBox_source_scanRangeMin->setValue(0);
		_ui->doubleSpinBox_source_scanRangeMax->setValue(0);
		_ui->doubleSpinBox_source_scanVoxelSize->setValue(0.0f);
		_ui->spinBox_source_scanNormalsK->setValue(0);
		_ui->doubleSpinBox_source_scanNormalsRadius->setValue(0.0);
		_ui->doubleSpinBox_source_scanNormalsForceGroundUp->setValue(0);

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
					this->setParameter(key, getDefaultWorkingDirectory().toStdString());
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
			_ui->odom_f2m_gravitySigma->setValue(-1);
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
	_ui->checkBox_odom_onlyInliersShown->setChecked(settings.value("odomOnlyInliersShown", _ui->checkBox_odom_onlyInliersShown->isChecked()).toBool());
	_ui->radioButton_posteriorGraphView->setChecked(settings.value("posteriorGraphView", _ui->radioButton_posteriorGraphView->isChecked()).toBool());
	_ui->radioButton_wordsGraphView->setChecked(settings.value("wordsGraphView", _ui->radioButton_wordsGraphView->isChecked()).toBool());
	_ui->radioButton_localizationsGraphView->setChecked(settings.value("localizationsGraphView", _ui->radioButton_localizationsGraphView->isChecked()).toBool());
	_ui->radioButton_nochangeGraphView->setChecked(settings.value("nochangeGraphView", _ui->radioButton_nochangeGraphView->isChecked()).toBool());
	_ui->checkbox_odomDisabled->setChecked(settings.value("odomDisabled", _ui->checkbox_odomDisabled->isChecked()).toBool());
	_ui->odom_registration->setCurrentIndex(settings.value("odomRegistration", _ui->odom_registration->currentIndex()).toInt());
	_ui->odom_f2m_gravitySigma->setValue(settings.value("odomF2MGravitySigma", _ui->odom_f2m_gravitySigma->value()).toDouble());
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
		_3dRenderingShowFrustums[i]->setChecked(settings.value(QString("showFrustums%1").arg(i), _3dRenderingShowFrustums[i]->isChecked()).toBool());

		_3dRenderingDownsamplingScan[i]->setValue(settings.value(QString("downsamplingScan%1").arg(i), _3dRenderingDownsamplingScan[i]->value()).toInt());
		_3dRenderingMaxRange[i]->setValue(settings.value(QString("maxRange%1").arg(i), _3dRenderingMaxRange[i]->value()).toDouble());
		_3dRenderingMinRange[i]->setValue(settings.value(QString("minRange%1").arg(i), _3dRenderingMinRange[i]->value()).toDouble());
		_3dRenderingVoxelSizeScan[i]->setValue(settings.value(QString("voxelSizeScan%1").arg(i), _3dRenderingVoxelSizeScan[i]->value()).toDouble());
		_3dRenderingColorScheme[i]->setValue(settings.value(QString("colorScheme%1").arg(i), _3dRenderingColorScheme[i]->value()).toInt());
		_3dRenderingOpacity[i]->setValue(settings.value(QString("opacity%1").arg(i), _3dRenderingOpacity[i]->value()).toDouble());
		_3dRenderingPtSize[i]->setValue(settings.value(QString("ptSize%1").arg(i), _3dRenderingPtSize[i]->value()).toInt());
		_3dRenderingColorSchemeScan[i]->setValue(settings.value(QString("colorSchemeScan%1").arg(i), _3dRenderingColorSchemeScan[i]->value()).toInt());
		_3dRenderingOpacityScan[i]->setValue(settings.value(QString("opacityScan%1").arg(i), _3dRenderingOpacityScan[i]->value()).toDouble());
		_3dRenderingPtSizeScan[i]->setValue(settings.value(QString("ptSizeScan%1").arg(i), _3dRenderingPtSizeScan[i]->value()).toInt());
		_3dRenderingPtSizeFeatures[i]->setValue(settings.value(QString("ptSizeFeatures%1").arg(i), _3dRenderingPtSizeFeatures[i]->value()).toInt());
		_3dRenderingGravity[i]->setChecked(settings.value(QString("gravityShown%1").arg(i), _3dRenderingGravity[i]->isChecked()).toBool());
		_3dRenderingGravityLength[i]->setValue(settings.value(QString("gravityLength%1").arg(i), _3dRenderingGravityLength[i]->value()).toDouble());

	}
	_ui->doubleSpinBox_voxel->setValue(settings.value("cloudVoxel", _ui->doubleSpinBox_voxel->value()).toDouble());
	_ui->doubleSpinBox_noiseRadius->setValue(settings.value("cloudNoiseRadius", _ui->doubleSpinBox_noiseRadius->value()).toDouble());
	_ui->spinBox_noiseMinNeighbors->setValue(settings.value("cloudNoiseMinNeighbors", _ui->spinBox_noiseMinNeighbors->value()).toInt());
	_ui->doubleSpinBox_ceilingFilterHeight->setValue(settings.value("cloudCeilingHeight", _ui->doubleSpinBox_ceilingFilterHeight->value()).toDouble());
	_ui->doubleSpinBox_floorFilterHeight->setValue(settings.value("cloudFloorHeight", _ui->doubleSpinBox_floorFilterHeight->value()).toDouble());
	_ui->spinBox_normalKSearch->setValue(settings.value("normalKSearch", _ui->spinBox_normalKSearch->value()).toInt());
	_ui->doubleSpinBox_normalRadiusSearch->setValue(settings.value("normalRadiusSearch", _ui->doubleSpinBox_normalRadiusSearch->value()).toDouble());
	_ui->doubleSpinBox_ceilingFilterHeight_scan->setValue(settings.value("scanCeilingHeight", _ui->doubleSpinBox_ceilingFilterHeight_scan->value()).toDouble());
	_ui->doubleSpinBox_floorFilterHeight_scan->setValue(settings.value("scanFloorHeight", _ui->doubleSpinBox_floorFilterHeight_scan->value()).toDouble());
	_ui->spinBox_normalKSearch_scan->setValue(settings.value("scanNormalKSearch", _ui->spinBox_normalKSearch_scan->value()).toInt());
	_ui->doubleSpinBox_normalRadiusSearch_scan->setValue(settings.value("scanNormalRadiusSearch", _ui->doubleSpinBox_normalRadiusSearch_scan->value()).toDouble());

	_ui->checkBox_showGraphs->setChecked(settings.value("showGraphs", _ui->checkBox_showGraphs->isChecked()).toBool());
	_ui->checkBox_showLabels->setChecked(settings.value("showLabels", _ui->checkBox_showLabels->isChecked()).toBool());
	_ui->checkBox_showFrames->setChecked(settings.value("showFrames", _ui->checkBox_showFrames->isChecked()).toBool());
	_ui->checkBox_showLandmarks->setChecked(settings.value("showLandmarks", _ui->checkBox_showLandmarks->isChecked()).toBool());
	_ui->doubleSpinBox_landmarkSize->setValue(settings.value("landmarkSize", _ui->doubleSpinBox_landmarkSize->value()).toDouble());
	_ui->checkBox_showIMUGravity->setChecked(settings.value("showIMUGravity", _ui->checkBox_showIMUGravity->isChecked()).toBool());
	_ui->checkBox_showIMUAcc->setChecked(settings.value("showIMUAcc", _ui->checkBox_showIMUAcc->isChecked()).toBool());

	_ui->radioButton_noFiltering->setChecked(settings.value("noFiltering", _ui->radioButton_noFiltering->isChecked()).toBool());
	_ui->radioButton_nodeFiltering->setChecked(settings.value("cloudFiltering", _ui->radioButton_nodeFiltering->isChecked()).toBool());
	_ui->doubleSpinBox_cloudFilterRadius->setValue(settings.value("cloudFilteringRadius", _ui->doubleSpinBox_cloudFilterRadius->value()).toDouble());
	_ui->doubleSpinBox_cloudFilterAngle->setValue(settings.value("cloudFilteringAngle", _ui->doubleSpinBox_cloudFilterAngle->value()).toDouble());
	_ui->radioButton_subtractFiltering->setChecked(settings.value("subtractFiltering", _ui->radioButton_subtractFiltering->isChecked()).toBool());
	_ui->spinBox_subtractFilteringMinPts->setValue(settings.value("subtractFilteringMinPts", _ui->spinBox_subtractFilteringMinPts->value()).toInt());
	_ui->doubleSpinBox_subtractFilteringRadius->setValue(settings.value("subtractFilteringRadius", _ui->doubleSpinBox_subtractFilteringRadius->value()).toDouble());
	_ui->doubleSpinBox_subtractFilteringAngle->setValue(settings.value("subtractFilteringAngle", _ui->doubleSpinBox_subtractFilteringAngle->value()).toDouble());

	_ui->checkBox_map_shown->setChecked(settings.value("gridMapShown", _ui->checkBox_map_shown->isChecked()).toBool());
	_ui->doubleSpinBox_map_opacity->setValue(settings.value("gridMapOpacity", _ui->doubleSpinBox_map_opacity->value()).toDouble());

	_ui->groupBox_octomap->setChecked(settings.value("octomap", _ui->groupBox_octomap->isChecked()).toBool());
	_ui->spinBox_octomap_treeDepth->setValue(settings.value("octomap_depth", _ui->spinBox_octomap_treeDepth->value()).toInt());
	_ui->checkBox_octomap_2dgrid->setChecked(settings.value("octomap_2dgrid", _ui->checkBox_octomap_2dgrid->isChecked()).toBool());
	_ui->checkBox_octomap_show3dMap->setChecked(settings.value("octomap_3dmap", _ui->checkBox_octomap_show3dMap->isChecked()).toBool());
	_ui->comboBox_octomap_renderingType->setCurrentIndex(settings.value("octomap_rendering_type", _ui->comboBox_octomap_renderingType->currentIndex()).toInt());
	_ui->spinBox_octomap_pointSize->setValue(settings.value("octomap_point_size", _ui->spinBox_octomap_pointSize->value()).toInt());

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
	// Backward compatibility
	if(_ui->lineEdit_sourceLocalTransform->text().compare("0 0 1 -1 0 0 0 -1 0") == 0)
	{
		UWARN("From 0.20.11, the local transform of the camera should not contain optical rotation (read=\"%s\"). Resetting to default Identity for convenience.", _ui->lineEdit_sourceLocalTransform->text().toStdString().c_str());
		_ui->lineEdit_sourceLocalTransform->setText("0 0 0 0 0 0");
	}

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
	_ui->checkBox_stereo_exposureCompensation->setChecked(settings.value("exposureCompensation", _ui->checkBox_stereo_exposureCompensation->isChecked()).toBool());
	settings.endGroup(); // stereo

	settings.beginGroup("rgb");
	_ui->source_comboBox_image_type->setCurrentIndex(settings.value("driver", _ui->source_comboBox_image_type->currentIndex()).toInt());
	_ui->checkBox_rgb_rectify->setChecked(settings.value("rectify",_ui->checkBox_rgb_rectify->isChecked()).toBool());
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
	_ui->openni2_hshift->setValue(settings.value("hshift", _ui->openni2_hshift->value()).toInt());
	_ui->openni2_vshift->setValue(settings.value("vshift", _ui->openni2_vshift->value()).toInt());
	_ui->openni2_depth_decimation->setValue(settings.value("depthDecimation", _ui->openni2_depth_decimation->value()).toInt());
	settings.endGroup(); // Openni2

	settings.beginGroup("Freenect2");
	_ui->comboBox_freenect2Format->setCurrentIndex(settings.value("format", _ui->comboBox_freenect2Format->currentIndex()).toInt());
	_ui->doubleSpinBox_freenect2MinDepth->setValue(settings.value("minDepth", _ui->doubleSpinBox_freenect2MinDepth->value()).toDouble());
	_ui->doubleSpinBox_freenect2MaxDepth->setValue(settings.value("maxDepth", _ui->doubleSpinBox_freenect2MaxDepth->value()).toDouble());
	_ui->checkBox_freenect2BilateralFiltering->setChecked(settings.value("bilateralFiltering", _ui->checkBox_freenect2BilateralFiltering->isChecked()).toBool());
	_ui->checkBox_freenect2EdgeAwareFiltering->setChecked(settings.value("edgeAwareFiltering", _ui->checkBox_freenect2EdgeAwareFiltering->isChecked()).toBool());
	_ui->checkBox_freenect2NoiseFiltering->setChecked(settings.value("noiseFiltering", _ui->checkBox_freenect2NoiseFiltering->isChecked()).toBool());
	_ui->lineEdit_freenect2Pipeline->setText(settings.value("pipeline", _ui->lineEdit_freenect2Pipeline->text()).toString());
	settings.endGroup(); // Freenect2

	settings.beginGroup("K4W2");
	_ui->comboBox_k4w2Format->setCurrentIndex(settings.value("format", _ui->comboBox_k4w2Format->currentIndex()).toInt());
	settings.endGroup(); // K4W2

	settings.beginGroup("K4A");
	_ui->comboBox_k4a_rgb_resolution->setCurrentIndex(settings.value("rgb_resolution", _ui->comboBox_k4a_rgb_resolution->currentIndex()).toInt());
	_ui->comboBox_k4a_framerate->setCurrentIndex(settings.value("framerate", _ui->comboBox_k4a_framerate->currentIndex()).toInt());
	_ui->comboBox_k4a_depth_resolution->setCurrentIndex(settings.value("depth_resolution", _ui->comboBox_k4a_depth_resolution->currentIndex()).toInt());
	_ui->checkbox_k4a_irDepth->setChecked(settings.value("ir", _ui->checkbox_k4a_irDepth->isChecked()).toBool());
	_ui->lineEdit_k4a_mkv->setText(settings.value("mkvPath", _ui->lineEdit_k4a_mkv->text()).toString());
	_ui->source_checkBox_useMKVStamps->setChecked(settings.value("useMkvStamps", _ui->source_checkBox_useMKVStamps->isChecked()).toBool());
	settings.endGroup(); // K4A

	settings.beginGroup("RealSense");
	_ui->comboBox_realsensePresetRGB->setCurrentIndex(settings.value("presetRGB", _ui->comboBox_realsensePresetRGB->currentIndex()).toInt());
	_ui->comboBox_realsensePresetDepth->setCurrentIndex(settings.value("presetDepth", _ui->comboBox_realsensePresetDepth->currentIndex()).toInt());
	_ui->checkbox_realsenseOdom->setChecked(settings.value("odom", _ui->checkbox_realsenseOdom->isChecked()).toBool());
	_ui->checkbox_realsenseDepthScaledToRGBSize->setChecked(settings.value("depthScaled", _ui->checkbox_realsenseDepthScaledToRGBSize->isChecked()).toBool());
	_ui->comboBox_realsenseRGBSource->setCurrentIndex(settings.value("rgbSource", _ui->comboBox_realsenseRGBSource->currentIndex()).toInt());
	settings.endGroup(); // RealSense

	settings.beginGroup("RealSense2");
	_ui->checkbox_rs2_emitter->setChecked(settings.value("emitter", _ui->checkbox_rs2_emitter->isChecked()).toBool());
	_ui->checkbox_rs2_irMode->setChecked(settings.value("ir", _ui->checkbox_rs2_irMode->isChecked()).toBool());
	_ui->checkbox_rs2_irDepth->setChecked(settings.value("irdepth", _ui->checkbox_rs2_irDepth->isChecked()).toBool());
	_ui->spinBox_rs2_width->setValue(settings.value("width", _ui->spinBox_rs2_width->value()).toInt());
	_ui->spinBox_rs2_height->setValue(settings.value("height", _ui->spinBox_rs2_height->value()).toInt());
	_ui->spinBox_rs2_rate->setValue(settings.value("rate", _ui->spinBox_rs2_rate->value()).toInt());
	_ui->spinBox_rs2_width_depth->setValue(settings.value("width_depth", _ui->spinBox_rs2_width_depth->value()).toInt());
	_ui->spinBox_rs2_height_depth->setValue(settings.value("height_depth", _ui->spinBox_rs2_height_depth->value()).toInt());
	_ui->spinBox_rs2_rate_depth->setValue(settings.value("rate_depth", _ui->spinBox_rs2_rate_depth->value()).toInt());
	_ui->checkbox_rs2_globalTimeStync->setChecked(settings.value("global_time_sync", _ui->checkbox_rs2_globalTimeStync->isChecked()).toBool());
	_ui->lineEdit_rs2_jsonFile->setText(settings.value("json_preset", _ui->lineEdit_rs2_jsonFile->text()).toString());
	settings.endGroup(); // RealSense

	settings.beginGroup("RGBDImages");
	_ui->lineEdit_cameraRGBDImages_path_rgb->setText(settings.value("path_rgb", _ui->lineEdit_cameraRGBDImages_path_rgb->text()).toString());
	_ui->lineEdit_cameraRGBDImages_path_depth->setText(settings.value("path_depth", _ui->lineEdit_cameraRGBDImages_path_depth->text()).toString());
	_ui->doubleSpinBox_cameraRGBDImages_scale->setValue(settings.value("scale", _ui->doubleSpinBox_cameraRGBDImages_scale->value()).toDouble());
	_ui->spinBox_cameraRGBDImages_startIndex->setValue(settings.value("start_index", _ui->spinBox_cameraRGBDImages_startIndex->value()).toInt());
	_ui->spinBox_cameraRGBDImages_maxFrames->setValue(settings.value("max_frames", _ui->spinBox_cameraRGBDImages_maxFrames->value()).toInt());
	settings.endGroup(); // RGBDImages

	settings.beginGroup("StereoImages");
	_ui->lineEdit_cameraStereoImages_path_left->setText(settings.value("path_left", _ui->lineEdit_cameraStereoImages_path_left->text()).toString());
	_ui->lineEdit_cameraStereoImages_path_right->setText(settings.value("path_right", _ui->lineEdit_cameraStereoImages_path_right->text()).toString());
	_ui->checkBox_stereo_rectify->setChecked(settings.value("rectify",_ui->checkBox_stereo_rectify->isChecked()).toBool());
	_ui->spinBox_cameraStereoImages_startIndex->setValue(settings.value("start_index",_ui->spinBox_cameraStereoImages_startIndex->value()).toInt());
	_ui->spinBox_cameraStereoImages_maxFrames->setValue(settings.value("max_frames",_ui->spinBox_cameraStereoImages_maxFrames->value()).toInt());
	settings.endGroup(); // StereoImages

	settings.beginGroup("StereoVideo");
	_ui->lineEdit_cameraStereoVideo_path->setText(settings.value("path", _ui->lineEdit_cameraStereoVideo_path->text()).toString());
	_ui->lineEdit_cameraStereoVideo_path_2->setText(settings.value("path2", _ui->lineEdit_cameraStereoVideo_path_2->text()).toString());
	_ui->spinBox_stereo_right_device->setValue(settings.value("device2", _ui->spinBox_stereo_right_device->value()).toInt());
	_ui->spinBox_stereousbcam_streamWidth->setValue(settings.value("width", _ui->spinBox_stereousbcam_streamWidth->value()).toInt());
	_ui->spinBox_stereousbcam_streamHeight->setValue(settings.value("height", _ui->spinBox_stereousbcam_streamHeight->value()).toInt());
	settings.endGroup(); // StereoVideo

	settings.beginGroup("StereoZed");
	_ui->comboBox_stereoZed_resolution->setCurrentIndex(settings.value("resolution", _ui->comboBox_stereoZed_resolution->currentIndex()).toInt());
	_ui->comboBox_stereoZed_quality->setCurrentIndex(settings.value("quality", _ui->comboBox_stereoZed_quality->currentIndex()).toInt());
	_ui->checkbox_stereoZed_selfCalibration->setChecked(settings.value("self_calibration", _ui->checkbox_stereoZed_selfCalibration->isChecked()).toBool());
	_ui->comboBox_stereoZed_sensingMode->setCurrentIndex(settings.value("sensing_mode", _ui->comboBox_stereoZed_sensingMode->currentIndex()).toInt());
	_ui->spinBox_stereoZed_confidenceThr->setValue(settings.value("confidence_thr", _ui->spinBox_stereoZed_confidenceThr->value()).toInt());
	_ui->spinBox_stereoZed_texturenessConfidenceThr->setValue(settings.value("textureness_confidence_thr", _ui->spinBox_stereoZed_texturenessConfidenceThr->value()).toInt());
	_ui->lineEdit_zedSvoPath->setText(settings.value("svo_path", _ui->lineEdit_zedSvoPath->text()).toString());
	settings.endGroup(); // StereoZed
	
	settings.beginGroup("MyntEye");
	_ui->checkbox_stereoMyntEye_rectify->setChecked(settings.value("rectify", _ui->checkbox_stereoMyntEye_rectify->isChecked()).toBool());
	_ui->checkbox_stereoMyntEye_depth->setChecked(settings.value("depth", _ui->checkbox_stereoMyntEye_depth->isChecked()).toBool());
	_ui->checkbox_stereoMyntEye_autoExposure->setChecked(settings.value("auto_exposure", _ui->checkbox_stereoMyntEye_autoExposure->isChecked()).toBool());
	_ui->spinBox_stereoMyntEye_gain->setValue(settings.value("gain", _ui->spinBox_stereoMyntEye_gain->value()).toInt());
	_ui->spinBox_stereoMyntEye_brightness->setValue(settings.value("brightness", _ui->spinBox_stereoMyntEye_brightness->value()).toInt());
	_ui->spinBox_stereoMyntEye_contrast->setValue(settings.value("contrast", _ui->spinBox_stereoMyntEye_contrast->value()).toInt());
	_ui->spinBox_stereoMyntEye_irControl->setValue(settings.value("ir_control", _ui->spinBox_stereoMyntEye_irControl->value()).toInt());
	settings.endGroup(); // MyntEye

	settings.beginGroup("DepthAI");
	_ui->comboBox_depthai_resolution->setCurrentIndex(settings.value("resolution", _ui->comboBox_depthai_resolution->currentIndex()).toInt());
	_ui->checkBox_depthai_depth->setChecked(settings.value("depth", _ui->checkBox_depthai_depth->isChecked()).toBool());
	_ui->spinBox_depthai_confidence->setValue(settings.value("confidence", _ui->spinBox_depthai_confidence->value()).toInt());
	_ui->checkBox_depthai_imu_firmware_update->setChecked(settings.value("imu_firmware_update", _ui->checkBox_depthai_imu_firmware_update->isChecked()).toBool());
	settings.endGroup(); // DepthAI

	settings.beginGroup("Images");
	_ui->source_images_lineEdit_path->setText(settings.value("path", _ui->source_images_lineEdit_path->text()).toString());
	_ui->source_images_spinBox_startPos->setValue(settings.value("startPos",_ui->source_images_spinBox_startPos->value()).toInt());
	_ui->source_images_spinBox_maxFrames->setValue(settings.value("maxFrames",_ui->source_images_spinBox_maxFrames->value()).toInt());
	_ui->comboBox_cameraImages_bayerMode->setCurrentIndex(settings.value("bayerMode",_ui->comboBox_cameraImages_bayerMode->currentIndex()).toInt());

	_ui->checkBox_cameraImages_configForEachFrame->setChecked(settings.value("config_each_frame",_ui->checkBox_cameraImages_configForEachFrame->isChecked()).toBool());
	_ui->checkBox_cameraImages_timestamps->setChecked(settings.value("filenames_as_stamps",_ui->checkBox_cameraImages_timestamps->isChecked()).toBool());
	_ui->checkBox_cameraImages_syncTimeStamps->setChecked(settings.value("sync_stamps",_ui->checkBox_cameraImages_syncTimeStamps->isChecked()).toBool());
	_ui->lineEdit_cameraImages_timestamps->setText(settings.value("stamps", _ui->lineEdit_cameraImages_timestamps->text()).toString());
	_ui->lineEdit_cameraImages_path_scans->setText(settings.value("path_scans", _ui->lineEdit_cameraImages_path_scans->text()).toString());
	_ui->lineEdit_cameraImages_laser_transform->setText(settings.value("scan_transform", _ui->lineEdit_cameraImages_laser_transform->text()).toString());
	_ui->spinBox_cameraImages_max_scan_pts->setValue(settings.value("scan_max_pts", _ui->spinBox_cameraImages_max_scan_pts->value()).toInt());
	_ui->lineEdit_cameraImages_odom->setText(settings.value("odom_path", _ui->lineEdit_cameraImages_odom->text()).toString());
	_ui->comboBox_cameraImages_odomFormat->setCurrentIndex(settings.value("odom_format", _ui->comboBox_cameraImages_odomFormat->currentIndex()).toInt());
	_ui->lineEdit_cameraImages_gt->setText(settings.value("gt_path", _ui->lineEdit_cameraImages_gt->text()).toString());
	_ui->comboBox_cameraImages_gtFormat->setCurrentIndex(settings.value("gt_format", _ui->comboBox_cameraImages_gtFormat->currentIndex()).toInt());
	_ui->doubleSpinBox_maxPoseTimeDiff->setValue(settings.value("max_pose_time_diff", _ui->doubleSpinBox_maxPoseTimeDiff->value()).toDouble());

	_ui->lineEdit_cameraImages_path_imu->setText(settings.value("imu_path", _ui->lineEdit_cameraImages_path_imu->text()).toString());
	_ui->lineEdit_cameraImages_imu_transform->setText(settings.value("imu_local_transform", _ui->lineEdit_cameraImages_imu_transform->text()).toString());
	_ui->spinBox_cameraImages_max_imu_rate->setValue(settings.value("imu_rate", _ui->spinBox_cameraImages_max_imu_rate->value()).toInt());
	settings.endGroup(); // images

	settings.beginGroup("OdomSensor");
	_ui->comboBox_odom_sensor->setCurrentIndex(settings.value("odom_sensor", _ui->comboBox_odom_sensor->currentIndex()).toInt());
	_ui->lineEdit_odom_sensor_extrinsics->setText(settings.value("odom_sensor_extrinsics", _ui->lineEdit_odom_sensor_extrinsics->text()).toString());
	_ui->lineEdit_odom_sensor_path_calibration->setText(settings.value("odom_sensor_calibration_path", _ui->lineEdit_odom_sensor_path_calibration->text()).toString());
	_ui->lineEdit_odomSourceDevice->setText(settings.value("odom_sensor_device", _ui->lineEdit_odomSourceDevice->text()).toString());
	_ui->doubleSpinBox_odom_sensor_time_offset->setValue(settings.value("odom_sensor_offset_time", _ui->doubleSpinBox_odom_sensor_time_offset->value()).toDouble());
	_ui->doubleSpinBox_odom_sensor_scale_factor->setValue(settings.value("odom_sensor_scale_factor", _ui->doubleSpinBox_odom_sensor_scale_factor->value()).toDouble());
	_ui->checkBox_odom_sensor_use_as_gt->setChecked(settings.value("odom_sensor_odom_as_gt", _ui->checkBox_odom_sensor_use_as_gt->isChecked()).toBool());
	settings.endGroup(); // OdomSensor

	settings.beginGroup("UsbCam");
	_ui->spinBox_usbcam_streamWidth->setValue(settings.value("width", _ui->spinBox_usbcam_streamWidth->value()).toInt());
	_ui->spinBox_usbcam_streamHeight->setValue(settings.value("height", _ui->spinBox_usbcam_streamHeight->value()).toInt());
	settings.endGroup(); // UsbCam

	settings.beginGroup("Video");
	_ui->source_video_lineEdit_path->setText(settings.value("path", _ui->source_video_lineEdit_path->text()).toString());
	settings.endGroup(); // video

	settings.beginGroup("IMU");
	_ui->comboBox_imuFilter_strategy->setCurrentIndex(settings.value("strategy", _ui->comboBox_imuFilter_strategy->currentIndex()).toInt());
	_ui->doubleSpinBox_imuFilterMadgwickGain->setValue(settings.value("madgwick_gain", _ui->doubleSpinBox_imuFilterMadgwickGain->value()).toDouble());
	_ui->doubleSpinBox_imuFilterMadgwickZeta->setValue(settings.value("madgwick_zeta", _ui->doubleSpinBox_imuFilterMadgwickZeta->value()).toDouble());
	_ui->doubleSpinBox_imuFilterComplementaryGainAcc->setValue(settings.value("complementary_gain_acc", _ui->doubleSpinBox_imuFilterComplementaryGainAcc->value()).toDouble());
	_ui->doubleSpinBox_imuFilterComplementaryBiasAlpha->setValue(settings.value("complementary_bias_alpha", _ui->doubleSpinBox_imuFilterComplementaryBiasAlpha->value()).toDouble());
	_ui->checkBox_imuFilterComplementaryDoAdaptiveGain->setChecked(settings.value("complementary_adaptive_gain", _ui->checkBox_imuFilterComplementaryDoAdaptiveGain->isChecked()).toBool());
	_ui->checkBox_imuFilterComplementaryDoBiasEstimation->setChecked(settings.value("complementary_biais_estimation", _ui->checkBox_imuFilterComplementaryDoBiasEstimation->isChecked()).toBool());
	_ui->checkBox_imuFilter_baseFrameConversion->setChecked(settings.value("base_frame_conversion", _ui->checkBox_imuFilter_baseFrameConversion->isChecked()).toBool());
	_ui->checkbox_publishInterIMU->setChecked(settings.value("publish_inter_imu", _ui->checkbox_publishInterIMU->isChecked()).toBool());
	settings.endGroup();//IMU

	settings.beginGroup("Scan");
	_ui->checkBox_source_scanFromDepth->setChecked(settings.value("fromDepth", _ui->checkBox_source_scanFromDepth->isChecked()).toBool());
	_ui->spinBox_source_scanDownsampleStep->setValue(settings.value("downsampleStep", _ui->spinBox_source_scanDownsampleStep->value()).toInt());
	_ui->doubleSpinBox_source_scanRangeMin->setValue(settings.value("rangeMin", _ui->doubleSpinBox_source_scanRangeMin->value()).toDouble());
	_ui->doubleSpinBox_source_scanRangeMax->setValue(settings.value("rangeMax", _ui->doubleSpinBox_source_scanRangeMax->value()).toDouble());
	_ui->doubleSpinBox_source_scanVoxelSize->setValue(settings.value("voxelSize", _ui->doubleSpinBox_source_scanVoxelSize->value()).toDouble());
	_ui->spinBox_source_scanNormalsK->setValue(settings.value("normalsK", _ui->spinBox_source_scanNormalsK->value()).toInt());
	_ui->doubleSpinBox_source_scanNormalsRadius->setValue(settings.value("normalsRadius", _ui->doubleSpinBox_source_scanNormalsRadius->value()).toDouble());
	_ui->doubleSpinBox_source_scanNormalsForceGroundUp->setValue(settings.value("normalsUpF", _ui->doubleSpinBox_source_scanNormalsForceGroundUp->value()).toDouble());
	settings.endGroup();//Scan

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
	_ui->source_checkBox_ignoreLandmarks->setChecked(settings.value("ignoreLandmarks", _ui->source_checkBox_ignoreLandmarks->isChecked()).toBool());
	_ui->source_checkBox_ignoreFeatures->setChecked(settings.value("ignoreFeatures", _ui->source_checkBox_ignoreFeatures->isChecked()).toBool());
	_ui->source_spinBox_databaseStartId->setValue(settings.value("startId", _ui->source_spinBox_databaseStartId->value()).toInt());
	_ui->source_spinBox_databaseStopId->setValue(settings.value("stopId", _ui->source_spinBox_databaseStopId->value()).toInt());
	_ui->source_spinBox_database_cameraIndex->setValue(settings.value("cameraIndex", _ui->source_spinBox_database_cameraIndex->value()).toInt());
	_ui->source_checkBox_useDbStamps->setChecked(settings.value("useDatabaseStamps", _ui->source_checkBox_useDbStamps->isChecked()).toBool());
	settings.endGroup(); // Database

	settings.endGroup(); // Camera

	_calibrationDialog->loadSettings(settings, "CalibrationDialog");

}

QString PreferencesDialog::getDefaultWorkingDirectory() const
{
	return Parameters::createDefaultWorkingDirectory().c_str();
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
			if(value.empty())
			{
				UWARN("Reading config: Working directory is empty. Keeping old one (\"%s\").",
					this->getWorkingDirectory().toStdString().c_str());
				value = this->getWorkingDirectory().toStdString();
			}

			// The directory should exist if not the default one
			if(!QDir(value.c_str()).exists() && value.compare(getDefaultWorkingDirectory().toStdString()) != 0)
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
					std::string defaultWorkingDir = getDefaultWorkingDirectory().toStdString();
					UWARN("Reading config: Not existing working directory \"%s\". Using default one (\"%s\").",
						value.c_str(),
						defaultWorkingDir.c_str());
					value = defaultWorkingDir;
				}
			}
		}

		//backward compatibility
		if(iter->first.compare(Parameters::kIcpStrategy()) == 0)
		{
			if(value.compare("true") == 0)
			{
				value =  "1";
			}
			else if(value.compare("false") == 0)
			{
				value =  "0";
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
					   "preferences menu.").arg(getDefaultWorkingDirectory()));
		}
		this->setParameter(Parameters::kRtabmapWorkingDirectory(), getDefaultWorkingDirectory().toStdString());
		UDEBUG("key.toStdString()=%s", getDefaultWorkingDirectory().toStdString().c_str());
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
		Q_EMIT settingsChanged(_modifiedParameters);
	}

	if(_obsoletePanels)
	{
		Q_EMIT settingsChanged(_obsoletePanels);
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
	settings.setValue("odomOnlyInliersShown", _ui->checkBox_odom_onlyInliersShown->isChecked());
	settings.setValue("posteriorGraphView",   _ui->radioButton_posteriorGraphView->isChecked());
	settings.setValue("wordsGraphView",       _ui->radioButton_wordsGraphView->isChecked());
	settings.setValue("localizationsGraphView", _ui->radioButton_localizationsGraphView->isChecked());
	settings.setValue("nochangeGraphView",    _ui->radioButton_nochangeGraphView->isChecked());
	settings.setValue("odomDisabled",         _ui->checkbox_odomDisabled->isChecked());
	settings.setValue("odomRegistration",     _ui->odom_registration->currentIndex());
	settings.setValue("odomF2MGravitySigma",  _ui->odom_f2m_gravitySigma->value());
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
		settings.setValue(QString("showFrustums%1").arg(i), _3dRenderingShowFrustums[i]->isChecked());

		settings.setValue(QString("downsamplingScan%1").arg(i), _3dRenderingDownsamplingScan[i]->value());
		settings.setValue(QString("maxRange%1").arg(i), _3dRenderingMaxRange[i]->value());
		settings.setValue(QString("minRange%1").arg(i), _3dRenderingMinRange[i]->value());
		settings.setValue(QString("voxelSizeScan%1").arg(i), _3dRenderingVoxelSizeScan[i]->value());
		settings.setValue(QString("colorScheme%1").arg(i), _3dRenderingColorScheme[i]->value());
		settings.setValue(QString("opacity%1").arg(i), _3dRenderingOpacity[i]->value());
		settings.setValue(QString("ptSize%1").arg(i), _3dRenderingPtSize[i]->value());
		settings.setValue(QString("colorSchemeScan%1").arg(i), _3dRenderingColorSchemeScan[i]->value());
		settings.setValue(QString("opacityScan%1").arg(i), _3dRenderingOpacityScan[i]->value());
		settings.setValue(QString("ptSizeScan%1").arg(i), _3dRenderingPtSizeScan[i]->value());
		settings.setValue(QString("ptSizeFeatures%1").arg(i), _3dRenderingPtSizeFeatures[i]->value());
		settings.setValue(QString("gravityShown%1").arg(i), _3dRenderingGravity[i]->isChecked());
		settings.setValue(QString("gravityLength%1").arg(i), _3dRenderingGravityLength[i]->value());
	}
	settings.setValue("cloudVoxel",             _ui->doubleSpinBox_voxel->value());
	settings.setValue("cloudNoiseRadius",       _ui->doubleSpinBox_noiseRadius->value());
	settings.setValue("cloudNoiseMinNeighbors", _ui->spinBox_noiseMinNeighbors->value());
	settings.setValue("cloudCeilingHeight",     _ui->doubleSpinBox_ceilingFilterHeight->value());
	settings.setValue("cloudFloorHeight",       _ui->doubleSpinBox_floorFilterHeight->value());
	settings.setValue("normalKSearch",          _ui->spinBox_normalKSearch->value());
	settings.setValue("normalRadiusSearch",     _ui->doubleSpinBox_normalRadiusSearch->value());
	settings.setValue("scanCeilingHeight",      _ui->doubleSpinBox_ceilingFilterHeight_scan->value());
	settings.setValue("scanFloorHeight",        _ui->doubleSpinBox_floorFilterHeight_scan->value());
	settings.setValue("scanNormalKSearch",      _ui->spinBox_normalKSearch_scan->value());
	settings.setValue("scanNormalRadiusSearch", _ui->doubleSpinBox_normalRadiusSearch_scan->value());

	settings.setValue("showGraphs", _ui->checkBox_showGraphs->isChecked());
	settings.setValue("showLabels", _ui->checkBox_showLabels->isChecked());
	settings.setValue("showFrames", _ui->checkBox_showFrames->isChecked());
	settings.setValue("showLandmarks", _ui->checkBox_showLandmarks->isChecked());
	settings.setValue("landmarkSize", _ui->doubleSpinBox_landmarkSize->value());
	settings.setValue("showIMUGravity", _ui->checkBox_showIMUGravity->isChecked());
	settings.setValue("showIMUAcc", _ui->checkBox_showIMUAcc->isChecked());


	settings.setValue("noFiltering",             _ui->radioButton_noFiltering->isChecked());
	settings.setValue("cloudFiltering",          _ui->radioButton_nodeFiltering->isChecked());
	settings.setValue("cloudFilteringRadius",    _ui->doubleSpinBox_cloudFilterRadius->value());
	settings.setValue("cloudFilteringAngle",     _ui->doubleSpinBox_cloudFilterAngle->value());
	settings.setValue("subtractFiltering",       _ui->radioButton_subtractFiltering->isChecked());
	settings.setValue("subtractFilteringMinPts", _ui->spinBox_subtractFilteringMinPts->value());
	settings.setValue("subtractFilteringRadius", _ui->doubleSpinBox_subtractFilteringRadius->value());
	settings.setValue("subtractFilteringAngle",  _ui->doubleSpinBox_subtractFilteringAngle->value());

	settings.setValue("gridMapShown",                _ui->checkBox_map_shown->isChecked());
	settings.setValue("gridMapOpacity",              _ui->doubleSpinBox_map_opacity->value());

	settings.setValue("octomap",                     _ui->groupBox_octomap->isChecked());
	settings.setValue("octomap_depth",               _ui->spinBox_octomap_treeDepth->value());
	settings.setValue("octomap_2dgrid",              _ui->checkBox_octomap_2dgrid->isChecked());
	settings.setValue("octomap_3dmap",               _ui->checkBox_octomap_show3dMap->isChecked());
	settings.setValue("octomap_rendering_type",      _ui->comboBox_octomap_renderingType->currentIndex());
	settings.setValue("octomap_point_size",          _ui->spinBox_octomap_pointSize->value());


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
	settings.setValue("exposureCompensation", _ui->checkBox_stereo_exposureCompensation->isChecked());
	settings.endGroup(); // stereo

	settings.beginGroup("rgb");
	settings.setValue("driver", 	_ui->source_comboBox_image_type->currentIndex());
	settings.setValue("rectify", 	    _ui->checkBox_rgb_rectify->isChecked());
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
	settings.setValue("hshift",           _ui->openni2_hshift->value());
	settings.setValue("vshift",           _ui->openni2_vshift->value());
	settings.setValue("depthDecimation",  _ui->openni2_depth_decimation->value());
	settings.endGroup(); // Openni2

	settings.beginGroup("Freenect2");
	settings.setValue("format",             _ui->comboBox_freenect2Format->currentIndex());
	settings.setValue("minDepth",           _ui->doubleSpinBox_freenect2MinDepth->value());
	settings.setValue("maxDepth",           _ui->doubleSpinBox_freenect2MaxDepth->value());
	settings.setValue("bilateralFiltering", _ui->checkBox_freenect2BilateralFiltering->isChecked());
	settings.setValue("edgeAwareFiltering", _ui->checkBox_freenect2EdgeAwareFiltering->isChecked());
	settings.setValue("noiseFiltering",     _ui->checkBox_freenect2NoiseFiltering->isChecked());
	settings.setValue("pipeline",           _ui->lineEdit_freenect2Pipeline->text());
	settings.endGroup(); // Freenect2

	settings.beginGroup("K4W2");
	settings.setValue("format",				_ui->comboBox_k4w2Format->currentIndex());
	settings.endGroup(); // K4W2

	settings.beginGroup("K4A");
	settings.setValue("rgb_resolution", _ui->comboBox_k4a_rgb_resolution->currentIndex());
	settings.setValue("framerate", _ui->comboBox_k4a_framerate->currentIndex());
	settings.setValue("depth_resolution", _ui->comboBox_k4a_depth_resolution->currentIndex());
	settings.setValue("ir", _ui->checkbox_k4a_irDepth->isChecked());
	settings.setValue("mkvPath", _ui->lineEdit_k4a_mkv->text());
	settings.setValue("useMkvStamps", _ui->source_checkBox_useMKVStamps->isChecked());
	settings.endGroup(); // K4A

	settings.beginGroup("RealSense");
	settings.setValue("presetRGB",           _ui->comboBox_realsensePresetRGB->currentIndex());
	settings.setValue("presetDepth",         _ui->comboBox_realsensePresetDepth->currentIndex());
	settings.setValue("odom",                _ui->checkbox_realsenseOdom->isChecked());
	settings.setValue("depthScaled",         _ui->checkbox_realsenseDepthScaledToRGBSize->isChecked());
	settings.setValue("rgbSource",           _ui->comboBox_realsenseRGBSource->currentIndex());
	settings.endGroup(); // RealSense

	settings.beginGroup("RealSense2");
	settings.setValue("emitter",                _ui->checkbox_rs2_emitter->isChecked());
	settings.setValue("ir",                     _ui->checkbox_rs2_irMode->isChecked());
	settings.setValue("irdepth",                _ui->checkbox_rs2_irDepth->isChecked());
	settings.setValue("width",                  _ui->spinBox_rs2_width->value());
	settings.setValue("height",                 _ui->spinBox_rs2_height->value());
	settings.setValue("rate",                   _ui->spinBox_rs2_rate->value());
	settings.setValue("width_depth",            _ui->spinBox_rs2_width_depth->value());
	settings.setValue("height_depth",           _ui->spinBox_rs2_height_depth->value());
	settings.setValue("rate_depth",             _ui->spinBox_rs2_rate_depth->value());
	settings.setValue("global_time_sync",       _ui->checkbox_rs2_globalTimeStync->isChecked());
	settings.setValue("json_preset",            _ui->lineEdit_rs2_jsonFile->text());
	settings.endGroup(); // RealSense2

	settings.beginGroup("RGBDImages");
	settings.setValue("path_rgb",            _ui->lineEdit_cameraRGBDImages_path_rgb->text());
	settings.setValue("path_depth",          _ui->lineEdit_cameraRGBDImages_path_depth->text());
	settings.setValue("scale",               _ui->doubleSpinBox_cameraRGBDImages_scale->value());
	settings.setValue("start_index",         _ui->spinBox_cameraRGBDImages_startIndex->value());
	settings.setValue("max_frames",         _ui->spinBox_cameraRGBDImages_maxFrames->value());
	settings.endGroup(); // RGBDImages

	settings.beginGroup("StereoImages");
	settings.setValue("path_left",      _ui->lineEdit_cameraStereoImages_path_left->text());
	settings.setValue("path_right",     _ui->lineEdit_cameraStereoImages_path_right->text());
	settings.setValue("rectify", 	    _ui->checkBox_stereo_rectify->isChecked());
	settings.setValue("start_index",    _ui->spinBox_cameraStereoImages_startIndex->value());
	settings.setValue("max_frames",    _ui->spinBox_cameraStereoImages_maxFrames->value());
	settings.endGroup(); // StereoImages

	settings.beginGroup("StereoVideo");
	settings.setValue("path", 			_ui->lineEdit_cameraStereoVideo_path->text());
	settings.setValue("path2", 			_ui->lineEdit_cameraStereoVideo_path_2->text());
	settings.setValue("device2",		_ui->spinBox_stereo_right_device->value());
	settings.setValue("width",		    _ui->spinBox_stereousbcam_streamWidth->value());
	settings.setValue("height",		    _ui->spinBox_stereousbcam_streamHeight->value());
	settings.endGroup(); // StereoVideo

	settings.beginGroup("StereoZed");
	settings.setValue("resolution", _ui->comboBox_stereoZed_resolution->currentIndex());
	settings.setValue("quality", _ui->comboBox_stereoZed_quality->currentIndex());
	settings.setValue("self_calibration", _ui->checkbox_stereoZed_selfCalibration->isChecked());
	settings.setValue("sensing_mode", _ui->comboBox_stereoZed_sensingMode->currentIndex());
	settings.setValue("confidence_thr", _ui->spinBox_stereoZed_confidenceThr->value());
	settings.setValue("textureness_confidence_thr", _ui->spinBox_stereoZed_texturenessConfidenceThr->value());
	settings.setValue("svo_path", _ui->lineEdit_zedSvoPath->text());
	settings.endGroup(); // StereoZed

	settings.beginGroup("MyntEye");
	settings.setValue("rectify",       _ui->checkbox_stereoMyntEye_rectify->isChecked());
	settings.setValue("depth",         _ui->checkbox_stereoMyntEye_depth->isChecked());
	settings.setValue("auto_exposure", _ui->checkbox_stereoMyntEye_autoExposure->isChecked());
	settings.setValue("gain",          _ui->spinBox_stereoMyntEye_gain->value());
	settings.setValue("brightness",    _ui->spinBox_stereoMyntEye_brightness->value());
	settings.setValue("contrast",      _ui->spinBox_stereoMyntEye_contrast->value());
	settings.setValue("ir_control",    _ui->spinBox_stereoMyntEye_irControl->value());
	settings.endGroup(); // MyntEye

	settings.beginGroup("DepthAI");
	settings.setValue("resolution",    _ui->comboBox_depthai_resolution->currentIndex());
	settings.setValue("depth",         _ui->checkBox_depthai_depth->isChecked());
	settings.setValue("confidence",    _ui->spinBox_depthai_confidence->value());
	settings.setValue("imu_firmware_update", _ui->checkBox_depthai_imu_firmware_update->isChecked());
	settings.endGroup(); // DepthAI

	settings.beginGroup("Images");
	settings.setValue("path", 			_ui->source_images_lineEdit_path->text());
	settings.setValue("startPos", 		_ui->source_images_spinBox_startPos->value());
	settings.setValue("maxFrames", 		_ui->source_images_spinBox_maxFrames->value());
	settings.setValue("bayerMode", 	    _ui->comboBox_cameraImages_bayerMode->currentIndex());
	settings.setValue("config_each_frame", _ui->checkBox_cameraImages_configForEachFrame->isChecked());
	settings.setValue("filenames_as_stamps", _ui->checkBox_cameraImages_timestamps->isChecked());
	settings.setValue("sync_stamps",    _ui->checkBox_cameraImages_syncTimeStamps->isChecked());
	settings.setValue("stamps",              _ui->lineEdit_cameraImages_timestamps->text());
	settings.setValue("path_scans",          _ui->lineEdit_cameraImages_path_scans->text());
	settings.setValue("scan_transform",      _ui->lineEdit_cameraImages_laser_transform->text());
	settings.setValue("scan_max_pts",        _ui->spinBox_cameraImages_max_scan_pts->value());
	settings.setValue("odom_path",           _ui->lineEdit_cameraImages_odom->text());
	settings.setValue("odom_format",         _ui->comboBox_cameraImages_odomFormat->currentIndex());
	settings.setValue("gt_path",             _ui->lineEdit_cameraImages_gt->text());
	settings.setValue("gt_format",           _ui->comboBox_cameraImages_gtFormat->currentIndex());
	settings.setValue("max_pose_time_diff",  _ui->doubleSpinBox_maxPoseTimeDiff->value());
	settings.setValue("imu_path",            _ui->lineEdit_cameraImages_path_imu->text());
	settings.setValue("imu_local_transform", _ui->lineEdit_cameraImages_imu_transform->text());
	settings.setValue("imu_rate",            _ui->spinBox_cameraImages_max_imu_rate->value());
	settings.endGroup(); // images

	settings.beginGroup("OdomSensor");
	settings.setValue("odom_sensor",            _ui->comboBox_odom_sensor->currentIndex());
	settings.setValue("odom_sensor_extrinsics", _ui->lineEdit_odom_sensor_extrinsics->text());
	settings.setValue("odom_sensor_calibration_path", _ui->lineEdit_odom_sensor_path_calibration->text());
	settings.setValue("odom_sensor_device",     _ui->lineEdit_odomSourceDevice->text());
	settings.setValue("odom_sensor_offset_time", _ui->doubleSpinBox_odom_sensor_time_offset->value());
	settings.setValue("odom_sensor_scale_factor", _ui->doubleSpinBox_odom_sensor_scale_factor->value());
	settings.setValue("odom_sensor_odom_as_gt", _ui->checkBox_odom_sensor_use_as_gt->isChecked());
	settings.endGroup(); // OdomSensor

	settings.beginGroup("UsbCam");
	settings.setValue("width",          _ui->spinBox_usbcam_streamWidth->value());
	settings.setValue("height",         _ui->spinBox_usbcam_streamHeight->value());
	settings.endGroup(); // UsbCam

	settings.beginGroup("Video");
	settings.setValue("path", 			_ui->source_video_lineEdit_path->text());
	settings.endGroup(); // video

	settings.beginGroup("IMU");
	settings.setValue("strategy",                       _ui->comboBox_imuFilter_strategy->currentIndex());
	settings.setValue("madgwick_gain",                  _ui->doubleSpinBox_imuFilterMadgwickGain->value());
	settings.setValue("madgwick_zeta",                  _ui->doubleSpinBox_imuFilterMadgwickZeta->value());
	settings.setValue("complementary_gain_acc",         _ui->doubleSpinBox_imuFilterComplementaryGainAcc->value());
	settings.setValue("complementary_bias_alpha",       _ui->doubleSpinBox_imuFilterComplementaryBiasAlpha->value());
	settings.setValue("complementary_adaptive_gain",    _ui->checkBox_imuFilterComplementaryDoAdaptiveGain->isChecked());
	settings.setValue("complementary_biais_estimation", _ui->checkBox_imuFilterComplementaryDoBiasEstimation->isChecked());
	settings.setValue("base_frame_conversion",          _ui->checkBox_imuFilter_baseFrameConversion->isChecked());
	settings.setValue("publish_inter_imu",              _ui->checkbox_publishInterIMU->isChecked());
	settings.endGroup();//IMU

	settings.beginGroup("Scan");
	settings.setValue("fromDepth", 			_ui->checkBox_source_scanFromDepth->isChecked());
	settings.setValue("downsampleStep", 	_ui->spinBox_source_scanDownsampleStep->value());
	settings.setValue("rangeMin", 			_ui->doubleSpinBox_source_scanRangeMin->value());
	settings.setValue("rangeMax", 			_ui->doubleSpinBox_source_scanRangeMax->value());
	settings.setValue("voxelSize", 			_ui->doubleSpinBox_source_scanVoxelSize->value());
	settings.setValue("normalsK", 			_ui->spinBox_source_scanNormalsK->value());
	settings.setValue("normalsRadius", 		_ui->doubleSpinBox_source_scanNormalsRadius->value());
	settings.setValue("normalsUpF", 	    _ui->doubleSpinBox_source_scanNormalsForceGroundUp->value());
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
	settings.setValue("ignoreLandmarks", _ui->source_checkBox_ignoreLandmarks->isChecked());
	settings.setValue("ignoreFeatures",  _ui->source_checkBox_ignoreFeatures->isChecked());
	settings.setValue("startId",          _ui->source_spinBox_databaseStartId->value());
	settings.setValue("stopId",          _ui->source_spinBox_databaseStopId->value());
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
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 3) || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION < 4 || (CV_MINOR_VERSION==4 && CV_SUBMINOR_VERSION<11)))
#ifndef RTABMAP_NONFREE
	// verify that SURF/SIFT cannot be selected if not built with OpenCV nonfree module
	// BOW dictionary type
	if(_ui->comboBox_detector_strategy->currentIndex() <= 1 || _ui->comboBox_detector_strategy->currentIndex() == 12)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected feature type (SURF/SIFT) is not available. RTAB-Map is not built "
				   "with the nonfree module from OpenCV. ORB is set instead for the bag-of-words dictionary."));
		_ui->comboBox_detector_strategy->setCurrentIndex(Feature2D::kFeatureOrb);
	}
	// BOW Reextract features type
	if(_ui->vis_feature_detector->currentIndex() <= 1 || _ui->vis_feature_detector->currentIndex() == 12)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected feature type (SURF/SIFT) is not available. RTAB-Map is not built "
				   "with the nonfree module from OpenCV. Fast/Brief is set instead for the re-extraction "
				   "of features on loop closure."));
		_ui->vis_feature_detector->setCurrentIndex(Feature2D::kFeatureFastBrief);
	}
#endif
#else //>= 4.4.0 >= 3.4.11
#ifndef RTABMAP_NONFREE
	// verify that SURF cannot be selected if not built with OpenCV nonfree module
	// BOW dictionary type
	if(_ui->comboBox_detector_strategy->currentIndex() < 1 || _ui->comboBox_detector_strategy->currentIndex() == 12)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected feature type (SURF) is not available. RTAB-Map is not built "
				   "with the nonfree module from OpenCV. SIFT is set instead for the bag-of-words dictionary."));
		_ui->comboBox_detector_strategy->setCurrentIndex(Feature2D::kFeatureSift);
	}
	// BOW Reextract features type
	if(_ui->vis_feature_detector->currentIndex() < 1 || _ui->vis_feature_detector->currentIndex() == 12)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected feature type (SURF) is not available. RTAB-Map is not built "
				   "with the nonfree module from OpenCV. Fast/Brief is set instead for the re-extraction "
				   "of features on loop closure."));
		_ui->vis_feature_detector->setCurrentIndex(Feature2D::kFeatureFastBrief);
	}
#endif
#endif

#if CV_MAJOR_VERSION < 3
	if (_ui->comboBox_detector_strategy->currentIndex() == Feature2D::kFeatureKaze)
	{
#ifdef RTABMAP_NONFREE
		QMessageBox::warning(this, tr("Parameter warning"),
			tr("Selected feature type (KAZE) is not available on OpenCV2. SURF is set instead "
				"for the bag-of-words dictionary."));
		_ui->comboBox_detector_strategy->setCurrentIndex(Feature2D::kFeatureSurf);
#else
		QMessageBox::warning(this, tr("Parameter warning"),
			tr("Selected feature type (KAZE) is not available on OpenCV2. ORB is set instead "
				"for the bag-of-words dictionary."));
		_ui->comboBox_detector_strategy->setCurrentIndex(Feature2D::kFeatureOrb);
#endif
	}
	if (_ui->vis_feature_detector->currentIndex() == Feature2D::kFeatureKaze)
	{
#ifdef RTABMAP_NONFREE
		QMessageBox::warning(this, tr("Parameter warning"),
			tr("Selected feature type (KAZE) is not available on OpenCV2. SURF is set instead "
				"for the re-extraction of features on loop closure."));
				_ui->vis_feature_detector->setCurrentIndex(Feature2D::kFeatureSurf);
#else
		QMessageBox::warning(this, tr("Parameter warning"),
			tr("Selected feature type (KAZE) is not available on OpenCV2. ORB is set instead "
				"for the re-extraction of features on loop closure."));
				_ui->vis_feature_detector->setCurrentIndex(Feature2D::kFeatureOrb);
#endif
	}
#endif

#ifndef RTABMAP_ORB_OCTREE
	if (_ui->comboBox_detector_strategy->currentIndex() == Feature2D::kFeatureOrbOctree)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
			tr("Selected feature type (ORB Octree) is not available. ORB is set instead "
				"for the bag-of-words dictionary."));
		_ui->comboBox_detector_strategy->setCurrentIndex(Feature2D::kFeatureOrb);
	}
	if (_ui->vis_feature_detector->currentIndex() == Feature2D::kFeatureOrbOctree)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
			tr("Selected feature type (ORB Octree) is not available on OpenCV2. ORB is set instead "
				"for the re-extraction of features on loop closure."));
				_ui->vis_feature_detector->setCurrentIndex(Feature2D::kFeatureOrb);
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
#ifndef RTABMAP_ORB_SLAM
		else if(Optimizer::isAvailable(Optimizer::kTypeG2O))
		{
			QMessageBox::warning(this, tr("Parameter warning"),
					tr("Selected graph optimization strategy (TORO) is not available. RTAB-Map is not built "
					   "with TORO. g2o is set instead for graph optimization strategy."));
			_ui->graphOptimization_type->setCurrentIndex(Optimizer::kTypeG2O);
		}
#endif
	}
#ifdef RTABMAP_ORB_SLAM
	if(_ui->graphOptimization_type->currentIndex() == 1)
#else
	if(_ui->graphOptimization_type->currentIndex() == 1 && !Optimizer::isAvailable(Optimizer::kTypeG2O))
#endif
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
#ifndef RTABMAP_ORB_SLAM
		if(Optimizer::isAvailable(Optimizer::kTypeG2O))
		{
			QMessageBox::warning(this, tr("Parameter warning"),
					tr("Selected graph optimization strategy (GTSAM) is not available. RTAB-Map is not built "
					   "with GTSAM. g2o is set instead for graph optimization strategy."));
			_ui->graphOptimization_type->setCurrentIndex(Optimizer::kTypeG2O);
		}
		else
#endif
			if(Optimizer::isAvailable(Optimizer::kTypeTORO))
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
	if(_ui->loopClosure_bundle->currentIndex() == 3 && !Optimizer::isAvailable(Optimizer::kTypeCeres))
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected visual registration bundle adjustment optimization strategy (Ceres) is not available. RTAB-Map is not built "
				   "with cERES. Bundle adjustment is disabled."));
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
	if(_ui->odom_f2m_bundleStrategy->currentIndex() == 3 && !Optimizer::isAvailable(Optimizer::kTypeCeres))
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("Selected odometry local bundle adjustment optimization strategy (Ceres) is not available. RTAB-Map is not built "
				   "with Ceres. Bundle adjustment is disabled."));
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
	if(_ui->reextract_nn->currentIndex() == VWDictionary::kNNFlannLSH && _ui->vis_feature_detector->currentIndex() <= 1)
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

#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION <4 || (CV_MINOR_VERSION ==4 && CV_SUBMINOR_VERSION<2)))
	if(_ui->ArucoDictionary->currentIndex()>=17)
	{
		QMessageBox::warning(this, tr("Parameter warning"),
				tr("ArUco dictionary: cannot select AprilTag dictionary, OpenCV version should be at least 3.4.2. Setting back to 0."));
		_ui->ArucoDictionary->setCurrentIndex(0);
	}
#endif

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

		this->setWindowTitle(tr("Preferences [Monitoring mode]"));
	}
	else
	{
		_ui->lineEdit_dictionaryPath->setEnabled(true);
		_ui->toolButton_dictionaryPath->setEnabled(true);
		_ui->label_dictionaryPath->setEnabled(true);

		_ui->groupBox_source0->setEnabled(true);
		_ui->groupBox_odometry1->setEnabled(true);

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

		QSettings settingsTmp(getTmpIniFilePath(), QSettings::IniFormat);
		settingsTmp.beginGroup("Gui");
		settingsTmp.beginGroup(window->objectName());
		settingsTmp.setValue("geometry", window->saveGeometry());
		settingsTmp.endGroup(); // "windowName"
		settingsTmp.endGroup(); // rtabmap
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

		QSettings settingsTmp(getTmpIniFilePath(), QSettings::IniFormat);
		settingsTmp.beginGroup("Gui");
		settingsTmp.beginGroup(window->objectName());
		settingsTmp.setValue("geometry", window->saveGeometry());
		settingsTmp.endGroup(); // "windowName"
		settingsTmp.endGroup(); // rtabmap
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

		QSettings settingsTmp(getTmpIniFilePath(), QSettings::IniFormat);
		settingsTmp.beginGroup("Gui");
		settingsTmp.beginGroup(mainWindow->objectName());
		settingsTmp.setValue("state", mainWindow->saveState());
		settingsTmp.setValue("maximized", mainWindow->isMaximized());
		settingsTmp.setValue("status_bar", mainWindow->statusBar()->isVisible());
		settingsTmp.endGroup(); // "MainWindow"
		settingsTmp.endGroup(); // rtabmap
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

		QSettings settingsTmp(getTmpIniFilePath(), QSettings::IniFormat);
		settingsTmp.beginGroup("Gui");
		settingsTmp.beginGroup(mainWindow->objectName());
		settingsTmp.setValue("state", mainWindow->saveState());
		settingsTmp.setValue("maximized", maximized);
		settingsTmp.setValue("status_bar", statusBarShown);
		settingsTmp.endGroup(); // "MainWindow"
		settingsTmp.endGroup(); // rtabmap
	}
}

void PreferencesDialog::saveWidgetState(const QWidget * widget)
{
	if(!widget->objectName().isEmpty())
	{
		QSettings settings(getIniFilePath(), QSettings::IniFormat);
		settings.beginGroup("Gui");
		settings.beginGroup(widget->objectName());

		QSettings settingsTmp(getTmpIniFilePath(), QSettings::IniFormat);
		settingsTmp.beginGroup("Gui");
		settingsTmp.beginGroup(widget->objectName());

		const CloudViewer * cloudViewer = qobject_cast<const CloudViewer*>(widget);
		const ImageView * imageView = qobject_cast<const ImageView*>(widget);
		const ExportCloudsDialog * exportCloudsDialog = qobject_cast<const ExportCloudsDialog*>(widget);
		const ExportBundlerDialog * exportBundlerDialog = qobject_cast<const ExportBundlerDialog*>(widget);
		const PostProcessingDialog * postProcessingDialog = qobject_cast<const PostProcessingDialog *>(widget);
		const GraphViewer * graphViewer = qobject_cast<const GraphViewer *>(widget);
		const CalibrationDialog * calibrationDialog = qobject_cast<const CalibrationDialog *>(widget);
		const DepthCalibrationDialog * depthCalibrationDialog = qobject_cast<const DepthCalibrationDialog *>(widget);

		if(cloudViewer)
		{
			cloudViewer->saveSettings(settings);
			cloudViewer->saveSettings(settingsTmp);
		}
		else if(imageView)
		{
			imageView->saveSettings(settings);
			imageView->saveSettings(settingsTmp);
		}
		else if(exportCloudsDialog)
		{
			exportCloudsDialog->saveSettings(settings);
			exportCloudsDialog->saveSettings(settingsTmp);
		}
		else if(exportBundlerDialog)
		{
			exportBundlerDialog->saveSettings(settings);
			exportBundlerDialog->saveSettings(settingsTmp);
		}
		else if(postProcessingDialog)
		{
			postProcessingDialog->saveSettings(settings);
			postProcessingDialog->saveSettings(settingsTmp);
		}
		else if(graphViewer)
		{
			graphViewer->saveSettings(settings);
			graphViewer->saveSettings(settingsTmp);
		}
		else if(calibrationDialog)
		{
			calibrationDialog->saveSettings(settings);
			calibrationDialog->saveSettings(settingsTmp);
		}
		else if(depthCalibrationDialog)
		{
			depthCalibrationDialog->saveSettings(settings);
			depthCalibrationDialog->saveSettings(settingsTmp);
		}
		else
		{
			UERROR("Widget \"%s\" cannot be exported in config file.", widget->objectName().toStdString().c_str());
		}

		settings.endGroup(); // "name"
		settings.endGroup(); // Gui
		settingsTmp.endGroup();
		settingsTmp.endGroup();
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

		QSettings settingsTmp(getTmpIniFilePath(), QSettings::IniFormat);
		settingsTmp.beginGroup("Gui");
		settingsTmp.beginGroup(widget->objectName());

		CloudViewer * cloudViewer = qobject_cast<CloudViewer*>(widget);
		ImageView * imageView = qobject_cast<ImageView*>(widget);
		ExportCloudsDialog * exportCloudsDialog = qobject_cast<ExportCloudsDialog*>(widget);
		ExportBundlerDialog * exportBundlerDialog = qobject_cast<ExportBundlerDialog*>(widget);
		PostProcessingDialog * postProcessingDialog = qobject_cast<PostProcessingDialog *>(widget);
		GraphViewer * graphViewer = qobject_cast<GraphViewer *>(widget);
		CalibrationDialog * calibrationDialog = qobject_cast<CalibrationDialog *>(widget);
		DepthCalibrationDialog * depthCalibrationDialog = qobject_cast<DepthCalibrationDialog *>(widget);

		if(cloudViewer)
		{
			cloudViewer->loadSettings(settings);
			cloudViewer->saveSettings(settingsTmp);
		}
		else if(imageView)
		{
			imageView->loadSettings(settings);
			imageView->saveSettings(settingsTmp);
		}
		else if(exportCloudsDialog)
		{
			exportCloudsDialog->loadSettings(settings);
			exportCloudsDialog->saveSettings(settingsTmp);
		}
		else if(exportBundlerDialog)
		{
			exportBundlerDialog->loadSettings(settings);
			exportBundlerDialog->saveSettings(settingsTmp);
		}
		else if(postProcessingDialog)
		{
			postProcessingDialog->loadSettings(settings);
			postProcessingDialog->saveSettings(settingsTmp);
		}
		else if(graphViewer)
		{
			graphViewer->loadSettings(settings);
			graphViewer->saveSettings(settingsTmp);
		}
		else if(calibrationDialog)
		{
			calibrationDialog->loadSettings(settings);
			calibrationDialog->saveSettings(settingsTmp);
		}
		else if(depthCalibrationDialog)
		{
			depthCalibrationDialog->loadSettings(settings);
			depthCalibrationDialog->saveSettings(settingsTmp);
		}
		else
		{
			UERROR("Widget \"%s\" cannot be loaded from config file.", widget->objectName().toStdString().c_str());
		}

		settings.endGroup(); //"name"
		settings.endGroup(); // Gui
		settingsTmp.endGroup(); //"name"
		settingsTmp.endGroup(); // Gui
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

	QSettings settingsTmp(getTmpIniFilePath(), QSettings::IniFormat);
	settingsTmp.beginGroup("Gui");
	settingsTmp.beginGroup(section);
	settingsTmp.setValue(key, value);
	settingsTmp.endGroup(); // "section"
	settingsTmp.endGroup(); // rtabmap
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

	QSettings settingsTmp(getTmpIniFilePath(), QSettings::IniFormat);
	settingsTmp.beginGroup("Gui");
	settingsTmp.beginGroup(section);
	settingsTmp.setValue(key, value);
	settingsTmp.endGroup(); // "section"
	settingsTmp.endGroup(); // rtabmap

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

void PreferencesDialog::updateParameters(const ParametersMap & parameters, bool setOtherParametersToDefault)
{
	if(parameters.size())
	{
		for(rtabmap::ParametersMap::const_iterator iter = parameters.begin(); iter!=parameters.end(); ++iter)
		{
			this->setParameter(iter->first, iter->second);
		}
		if(setOtherParametersToDefault)
		{
			for(ParametersMap::const_iterator iter=Parameters::getDefaultParameters().begin();
				iter!=Parameters::getDefaultParameters().end();
				++iter)
			{
				if(parameters.find(iter->first) == parameters.end() &&
					iter->first.compare(Parameters::kRtabmapWorkingDirectory())!=0)
				{
					this->setParameter(iter->first, iter->second);
				}
			}
		}
		if(!this->isVisible())
		{
			this->writeSettings(getTmpIniFilePath());
		}
	}
}

void PreferencesDialog::selectSourceDriver(Src src, int variant)
{
	if(_ui->comboBox_imuFilter_strategy->currentIndex()==0)
	{
		_ui->comboBox_imuFilter_strategy->setCurrentIndex(2);
	}
	_3dRenderingRoiRatios[0]->setText("0.0 0.0 0.0 0.0");
	_3dRenderingRoiRatios[1]->setText("0.0 0.0 0.0 0.0");

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
		else if (src == kSrcK4A)
		{
			_ui->lineEdit_k4a_mkv->clear();
			_3dRenderingRoiRatios[0]->setText("0.05 0.05 0.05 0.05");
			_3dRenderingRoiRatios[1]->setText("0.05 0.05 0.05 0.05");
		}
		else if (src == kSrcRealSense2)
		{
			if(variant > 0) // L515
			{
				_ui->spinBox_rs2_width->setValue(1280);
				_ui->spinBox_rs2_height->setValue(720);
				_ui->spinBox_rs2_rate->setValue(30);
			}
			else
			{
				_ui->spinBox_rs2_width->setValue(848);
				_ui->spinBox_rs2_height->setValue(480);
				_ui->spinBox_rs2_rate->setValue(60);
			}
		}
	}
	else if(src >= kSrcStereo && src<kSrcRGB)
	{
		_ui->comboBox_sourceType->setCurrentIndex(1);
		_ui->comboBox_cameraStereo->setCurrentIndex(src - kSrcStereo);

		if(src == kSrcStereoZed) // Zedm, Zed2
		{
			// disable IMU filtering (zed sends already quaternion)
			_ui->comboBox_imuFilter_strategy->setCurrentIndex(0);
		}
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

bool sortCallback(const std::string & a, const std::string & b)
{
	return uStrNumCmp(a,b) < 0;
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

		if(paths.size() > 1)
		{
			std::vector<std::string> vFileNames(paths.size());
			for(int i=0; i<paths.size(); ++i)
			{
				vFileNames[i] = paths[i].toStdString();
			}
			std::sort(vFileNames.begin(), vFileNames.end(), sortCallback);
			for(int i=0; i<paths.size(); ++i)
			{
				paths[i] = vFileNames[i].c_str();
			}
		}

		_ui->source_database_lineEdit_path->setText(paths.size()==1?paths.front():paths.join(";"));
		_ui->source_spinBox_databaseStartId->setValue(0);
		_ui->source_spinBox_databaseStopId->setValue(0);
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

void PreferencesDialog::selectOdomSensorCalibrationPath()
{
	QString dir = _ui->lineEdit_odom_sensor_path_calibration->text();
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
		_ui->lineEdit_odom_sensor_path_calibration->setText(path);
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

void PreferencesDialog::selectSourceImagesPathIMU()
{
	QString dir = _ui->lineEdit_cameraImages_path_imu->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file "), dir, tr("EuRoC IMU file (*.csv)"));
	if(path.size())
	{
		_ui->lineEdit_cameraImages_path_imu->setText(path);
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
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Odometry (*.txt *.log *.toro *.g2o *.csv)"));
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
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Ground Truth (*.txt *.log *.toro *.g2o *.csv)"));
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
		_ui->source_images_spinBox_maxFrames->setValue(0);
	}
}

void PreferencesDialog::selectSourceVideoPath()
{
	QString dir = _ui->source_video_lineEdit_path->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Videos (*.avi *.mpg *.mp4 *.mov *.mkv)"));
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
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Videos (*.avi *.mpg *.mp4 *.mov *.mkv)"));
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
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Videos (*.avi *.mpg *.mp4 *.mov *.mkv)"));
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
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("Distortion model (*.bin *.txt)"));
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
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("OpenNI (*.oni)"));
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
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("OpenNI (*.oni)"));
	if(!path.isEmpty())
	{
		_ui->lineEdit_openni2OniPath->setText(path);
	}
}

void PreferencesDialog::selectSourceMKVPath()
{
	QString dir = _ui->lineEdit_k4a_mkv->text();
	if (dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("K4A recording (*.mkv)"));
	if (!path.isEmpty())
	{
		_ui->lineEdit_k4a_mkv->setText(path);
	}
}

void PreferencesDialog::selectSourceSvoPath()
{
	QString dir = _ui->lineEdit_zedSvoPath->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("ZED (*.svo)"));
	if(!path.isEmpty())
	{
		_ui->lineEdit_zedSvoPath->setText(path);
	}
}

void PreferencesDialog::selectSourceRealsense2JsonPath()
{
	QString dir = _ui->lineEdit_rs2_jsonFile->text();
	if(dir.isEmpty())
	{
		dir = getWorkingDirectory();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Select RealSense2 preset"), dir, tr("JSON (*.json)"));
	if(path.size())
	{
		_ui->lineEdit_rs2_jsonFile->setText(path);
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
			//backward compatibility
			std::string valueCpy = value;
			if( key.compare(Parameters::kIcpStrategy()) == 0 ||
				key.compare(Parameters::kGridSensor()) == 0)
			{
				if(value.compare("true") == 0)
				{
					valueCpy =  "1";
				}
				else if(value.compare("false") == 0)
				{
					valueCpy =  "0";
				}
			}

			int valueInt = QString(valueCpy.c_str()).toInt(&ok);
			if(!ok)
			{
				UERROR("Conversion failed from \"%s\" for parameter %s", valueCpy.c_str(), key.c_str());
			}
			else
			{
#ifndef RTABMAP_NONFREE
				if(valueInt == 0 &&
						(combo->objectName().toStdString().compare(Parameters::kKpDetectorStrategy()) == 0 ||
						 combo->objectName().toStdString().compare(Parameters::kVisFeatureType()) == 0))
				{
					UWARN("Trying to set \"%s\" to SURF but RTAB-Map isn't built "
						  "with the nonfree module from OpenCV. Keeping default combo value: %s.",
						  combo->objectName().toStdString().c_str(),
						  combo->currentText().toStdString().c_str());
					ok = false;
				}
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 3) || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION < 4 || (CV_MINOR_VERSION==4 && CV_SUBMINOR_VERSION<11)))
				if(valueInt == 1 &&
						(combo->objectName().toStdString().compare(Parameters::kKpDetectorStrategy()) == 0 ||
						 combo->objectName().toStdString().compare(Parameters::kVisFeatureType()) == 0))
				{
					UWARN("Trying to set \"%s\" to SIFT but RTAB-Map isn't built "
						  "with the nonfree module from OpenCV. Keeping default combo value: %s.",
						  combo->objectName().toStdString().c_str(),
						  combo->currentText().toStdString().c_str());
					ok = false;
				}
#endif
#endif
#ifndef RTABMAP_ORB_SLAM
				if(!Optimizer::isAvailable(Optimizer::kTypeG2O))
#endif
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
	QVector<qreal> dataX((values.size()-2)*2 + 1);
	QVector<qreal> dataY((values.size()-2)*2 + 1);
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

	_ui->checkBox_stereo_rectify->setEnabled(
			 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoImages - kSrcStereo ||
			 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoUsb - kSrcStereo ||
              _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoTara - kSrcStereo ||
			 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoVideo - kSrcStereo ||
			 _ui->comboBox_cameraStereo->currentIndex() == kSrcDC1394 - kSrcStereo ||
			 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoRealSense2 - kSrcStereo);
	_ui->checkBox_stereo_rectify->setVisible(_ui->checkBox_stereo_rectify->isEnabled());
	_ui->label_stereo_rectify->setVisible(_ui->checkBox_stereo_rectify->isEnabled());
}

void PreferencesDialog::updateFeatureMatchingVisibility()
{
	_ui->groupBox_pymatcher->setVisible(_ui->reextract_nn->currentIndex() == 6);
	_ui->groupBox_gms->setVisible(_ui->reextract_nn->currentIndex() == 7);
}

void PreferencesDialog::updateOdometryStackedIndex(int index)
{
	if(index == 11) // FLOAM -> LOAM
	{
		_ui->stackedWidget_odometryType->setCurrentIndex(7);
	}
	else
	{
		_ui->stackedWidget_odometryType->setCurrentIndex(index);
	}
	_ui->groupBox_odomF2M->setVisible(index==0);
	_ui->groupBox_odomF2F->setVisible(index==1);
	_ui->groupBox_odomFovis->setVisible(index==2);
	_ui->groupBox_odomViso2->setVisible(index==3);
	_ui->groupBox_odomDVO->setVisible(index==4);
	_ui->groupBox_odomORBSLAM->setVisible(index==5);
	_ui->groupBox_odomOKVIS->setVisible(index==6);
	_ui->groupBox_odomLOAM->setVisible(index==7);
	_ui->groupBox_odomMSCKF->setVisible(index==8);
	_ui->groupBox_odomVINS->setVisible(index==9);
	_ui->groupBox_odomOpenVINS->setVisible(index==10);
	_ui->groupBox_odomOpen3D->setVisible(index==12);
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
			_ui->comboBox_detector_strategy->setCurrentIndex(_ui->vis_feature_detector->currentIndex());
			_ui->surf_doubleSpinBox_maxDepth->setValue(_ui->loopClosure_bowMaxDepth->value());
			_ui->surf_doubleSpinBox_minDepth->setValue(_ui->loopClosure_bowMinDepth->value());
			_ui->surf_spinBox_wordsPerImageTarget->setValue(_ui->reextract_maxFeatures->value());
			_ui->spinBox_KPGridRows->setValue(_ui->reextract_gridrows->value());
			_ui->spinBox_KPGridCols->setValue(_ui->reextract_gridcols->value());
			_ui->lineEdit_kp_roi->setText(_ui->loopClosure_roi->text());
			_ui->subpix_winSize_kp->setValue(_ui->subpix_winSize->value());
			_ui->subpix_iterations_kp->setValue(_ui->subpix_iterations->value());
			_ui->subpix_eps_kp->setValue(_ui->subpix_eps->value());
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
		path = QFileDialog::getOpenFileName(this, tr("Dictionary"), this->getWorkingDirectory(), tr("Dictionary (*.txt *.db)"));
	}
	else
	{
		path = QFileDialog::getOpenFileName(this, tr("Dictionary"), _ui->lineEdit_dictionaryPath->text(), tr("Dictionary (*.txt *.db)"));
	}
	if(!path.isEmpty())
	{
		_ui->lineEdit_dictionaryPath->setText(path);
	}
}

void PreferencesDialog::changeOdometryORBSLAMVocabulary()
{
	QString path;
	if(_ui->lineEdit_OdomORBSLAMVocPath->text().isEmpty())
	{
		path = QFileDialog::getOpenFileName(this, tr("ORBSLAM Vocabulary"), this->getWorkingDirectory(), tr("Vocabulary (*.txt)"));
	}
	else
	{
		path = QFileDialog::getOpenFileName(this, tr("ORBSLAM Vocabulary"), _ui->lineEdit_OdomORBSLAMVocPath->text(),  tr("Vocabulary (*.txt)"));
	}
	if(!path.isEmpty())
	{
		_ui->lineEdit_OdomORBSLAMVocPath->setText(path);
	}
}

void PreferencesDialog::changeOdometryOKVISConfigPath()
{
	QString path;
	if(_ui->lineEdit_OdomOkvisPath->text().isEmpty())
	{
		path = QFileDialog::getOpenFileName(this, tr("OKVIS Config"), this->getWorkingDirectory(), tr("OKVIS config (*.yaml)"));
	}
	else
	{
		path = QFileDialog::getOpenFileName(this, tr("OKVIS Config"), _ui->lineEdit_OdomOkvisPath->text(), tr("OKVIS config (*.yaml)"));
	}
	if(!path.isEmpty())
	{
		_ui->lineEdit_OdomOkvisPath->setText(path);
	}
}

void PreferencesDialog::changeOdometryVINSConfigPath()
{
	QString path;
	if(_ui->lineEdit_OdomVinsPath->text().isEmpty())
	{
		path = QFileDialog::getOpenFileName(this, tr("VINS-Fusion Config"), this->getWorkingDirectory(), tr("VINS-Fusion config (*.yaml)"));
	}
	else
	{
		path = QFileDialog::getOpenFileName(this, tr("VINS-Fusion Config"), _ui->lineEdit_OdomVinsPath->text(), tr("VINS-Fusion config (*.yaml)"));
	}
	if(!path.isEmpty())
	{
		_ui->lineEdit_OdomVinsPath->setText(path);
	}
}

void PreferencesDialog::changeIcpPMConfigPath()
{
	QString path;
	if(_ui->lineEdit_IcpPMConfigPath->text().isEmpty())
	{
		path = QFileDialog::getOpenFileName(this, tr("Select file"), this->getWorkingDirectory(), tr("libpointmatcher (*.yaml)"));
	}
	else
	{
		path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->lineEdit_IcpPMConfigPath->text(), tr("libpointmatcher (*.yaml)"));
	}
	if(!path.isEmpty())
	{
		_ui->lineEdit_IcpPMConfigPath->setText(path);
	}
}

void PreferencesDialog::changeSuperPointModelPath()
{
	QString path;
	if(_ui->lineEdit_sptorch_path->text().isEmpty())
	{
		path = QFileDialog::getOpenFileName(this, tr("Select file"), this->getWorkingDirectory(), tr("SuperPoint weights (*.pt)"));
	}
	else
	{
		path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->lineEdit_sptorch_path->text(), tr("SuperPoint weights (*.pt)"));
	}
	if(!path.isEmpty())
	{
		_ui->lineEdit_sptorch_path->setText(path);
	}
}

void PreferencesDialog::changePyMatcherPath()
{
	QString path;
	if(_ui->lineEdit_pymatcher_path->text().isEmpty())
	{
		path = QFileDialog::getOpenFileName(this, tr("Select file"), this->getWorkingDirectory(), tr("Python wrapper (*.py)"));
	}
	else
	{
		path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->lineEdit_pymatcher_path->text(), tr("Python wrapper (*.py)"));
	}
	if(!path.isEmpty())
	{
		_ui->lineEdit_pymatcher_path->setText(path);
	}
}

void PreferencesDialog::changePyMatcherModel()
{
	QString path;
	if(_ui->lineEdit_pymatcher_model->text().isEmpty())
	{
		path = QFileDialog::getOpenFileName(this, tr("Select file"), this->getWorkingDirectory(), tr("PyTorch model (*.pth *.pt)"));
	}
	else
	{
		path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->lineEdit_pymatcher_model->text(), tr("PyTorch model (*.pth *.pt)"));
	}
	if(!path.isEmpty())
	{
		_ui->lineEdit_pymatcher_model->setText(path);
	}
}

void PreferencesDialog::changePyDetectorPath()
{
	QString path;
	if(_ui->lineEdit_pydetector_path->text().isEmpty())
	{
		path = QFileDialog::getOpenFileName(this, tr("Select file"), this->getWorkingDirectory(), tr("Python wrapper (*.py)"));
	}
	else
	{
		path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->lineEdit_pydetector_path->text(), tr("Python wrapper (*.py)"));
	}
	if(!path.isEmpty())
	{
		_ui->lineEdit_pydetector_path->setText(path);
	}
}

void PreferencesDialog::updateSourceGrpVisibility()
{
	_ui->groupBox_sourceRGBD->setVisible(_ui->comboBox_sourceType->currentIndex() == 0);
	_ui->groupBox_sourceStereo->setVisible(_ui->comboBox_sourceType->currentIndex() == 1);
	_ui->groupBox_sourceRGB->setVisible(_ui->comboBox_sourceType->currentIndex() == 2);
	_ui->groupBox_sourceDatabase->setVisible(_ui->comboBox_sourceType->currentIndex() == 3);

	_ui->checkBox_source_scanFromDepth->setVisible(_ui->comboBox_sourceType->currentIndex() <= 1 || _ui->comboBox_sourceType->currentIndex() == 3);
	_ui->label_source_scanFromDepth->setVisible(_ui->comboBox_sourceType->currentIndex() <= 1 || _ui->comboBox_sourceType->currentIndex() == 3);

	_ui->stackedWidget_rgbd->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 &&
			(_ui->comboBox_cameraRGBD->currentIndex() == kSrcOpenNI2-kSrcRGBD ||
			 _ui->comboBox_cameraRGBD->currentIndex() == kSrcFreenect2-kSrcRGBD ||
			 _ui->comboBox_cameraRGBD->currentIndex() == kSrcK4W2 - kSrcRGBD ||
			 _ui->comboBox_cameraRGBD->currentIndex() == kSrcK4A - kSrcRGBD ||
			 _ui->comboBox_cameraRGBD->currentIndex() == kSrcRealSense - kSrcRGBD ||
			 _ui->comboBox_cameraRGBD->currentIndex() == kSrcRGBDImages-kSrcRGBD ||
			 _ui->comboBox_cameraRGBD->currentIndex() == kSrcOpenNI_PCL-kSrcRGBD ||
			 _ui->comboBox_cameraRGBD->currentIndex() == kSrcRealSense2-kSrcRGBD));
	_ui->groupBox_openni2->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcOpenNI2-kSrcRGBD);
	_ui->groupBox_freenect2->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcFreenect2-kSrcRGBD);
	_ui->groupBox_k4w2->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcK4W2 - kSrcRGBD);
	_ui->groupBox_k4a->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcK4A - kSrcRGBD);
	_ui->groupBox_realsense->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcRealSense - kSrcRGBD);
	_ui->groupBox_realsense2->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcRealSense2 - kSrcRGBD);
	_ui->groupBox_cameraRGBDImages->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcRGBDImages-kSrcRGBD);
	_ui->groupBox_openni->setVisible(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcOpenNI_PCL - kSrcRGBD);

	_ui->stackedWidget_stereo->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && 
		(_ui->comboBox_cameraStereo->currentIndex() == kSrcStereoVideo-kSrcStereo || 
		 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoImages-kSrcStereo ||
		 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoZed - kSrcStereo ||
		 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoUsb - kSrcStereo ||
		 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoMyntEye - kSrcStereo ||
		 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoZedOC - kSrcStereo ||
		 _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoDepthAI - kSrcStereo));
	_ui->groupBox_cameraStereoImages->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoImages-kSrcStereo);
	_ui->groupBox_cameraStereoVideo->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoVideo - kSrcStereo);
	_ui->groupBox_cameraStereoUsb->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoUsb - kSrcStereo);
	_ui->groupBox_cameraStereoZed->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoZed - kSrcStereo);
	_ui->groupBox_cameraMyntEye->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoMyntEye - kSrcStereo);
	_ui->groupBox_cameraStereoZedOC->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoZedOC - kSrcStereo);
	_ui->groupBox_cameraDepthAI->setVisible(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoDepthAI - kSrcStereo);

	_ui->stackedWidget_image->setVisible(_ui->comboBox_sourceType->currentIndex() == 2 &&
			(_ui->source_comboBox_image_type->currentIndex() == kSrcImages-kSrcRGB ||
			 _ui->source_comboBox_image_type->currentIndex() == kSrcVideo-kSrcRGB ||
			 _ui->source_comboBox_image_type->currentIndex() == kSrcUsbDevice-kSrcRGB));
	_ui->source_groupBox_images->setVisible(_ui->comboBox_sourceType->currentIndex() == 2 && _ui->source_comboBox_image_type->currentIndex() == kSrcImages-kSrcRGB);
	_ui->source_groupBox_video->setVisible(_ui->comboBox_sourceType->currentIndex() == 2 && _ui->source_comboBox_image_type->currentIndex() == kSrcVideo-kSrcRGB);
	_ui->source_groupBox_usbcam->setVisible(_ui->comboBox_sourceType->currentIndex() == 2 && _ui->source_comboBox_image_type->currentIndex() == kSrcUsbDevice-kSrcRGB);

	_ui->groupBox_sourceImages_optional->setVisible(
			(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcRGBDImages-kSrcRGBD) ||
			(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoImages-kSrcStereo) ||
			(_ui->comboBox_sourceType->currentIndex() == 2 && _ui->source_comboBox_image_type->currentIndex() == kSrcImages-kSrcRGB));

	_ui->groupBox_depthImageFiltering->setEnabled(
				_ui->comboBox_sourceType->currentIndex() == 0 || // RGBD
				_ui->comboBox_sourceType->currentIndex() == 3);  // Database
	_ui->groupBox_depthImageFiltering->setVisible(_ui->groupBox_depthImageFiltering->isEnabled());

	_ui->groupBox_imuFiltering->setEnabled(
			(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcRGBDImages-kSrcRGBD) ||
			(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoImages-kSrcStereo) ||
			(_ui->comboBox_sourceType->currentIndex() == 2 && _ui->source_comboBox_image_type->currentIndex() == kSrcImages-kSrcRGB) ||
			(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcFreenect - kSrcRGBD) || //Kinect360
			(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcK4A - kSrcRGBD) || //K4A
			(_ui->comboBox_sourceType->currentIndex() == 0 && _ui->comboBox_cameraRGBD->currentIndex() == kSrcRealSense2 - kSrcRGBD) || //D435i
			(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoRealSense2 - kSrcStereo) || //T265
			(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoZed - kSrcStereo) || // ZEDm, ZED2
			(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoMyntEye - kSrcStereo) || // MYNT EYE S
			(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoZedOC - kSrcStereo) ||
			(_ui->comboBox_sourceType->currentIndex() == 1 && _ui->comboBox_cameraStereo->currentIndex() == kSrcStereoDepthAI - kSrcStereo));
	_ui->stackedWidget_imuFilter->setVisible(_ui->comboBox_imuFilter_strategy->currentIndex() > 0);
	_ui->groupBox_madgwickfilter->setVisible(_ui->comboBox_imuFilter_strategy->currentIndex() == 1);
	_ui->groupBox_complementaryfilter->setVisible(_ui->comboBox_imuFilter_strategy->currentIndex() == 2);
	_ui->groupBox_imuFiltering->setVisible(_ui->groupBox_imuFiltering->isEnabled());

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
bool PreferencesDialog::isOdomOnlyInliersShown() const
{
	return _ui->checkBox_odom_onlyInliersShown->isChecked();
}
bool PreferencesDialog::isPosteriorGraphView() const
{
	return _ui->radioButton_posteriorGraphView->isChecked();
}
bool PreferencesDialog::isWordsCountGraphView() const
{
	return _ui->radioButton_wordsGraphView->isChecked();
}
bool PreferencesDialog::isLocalizationsCountGraphView() const
{
	return _ui->radioButton_localizationsGraphView->isChecked();
}
bool PreferencesDialog::isOdomDisabled() const
{
	return _ui->checkbox_odomDisabled->isChecked();
}
bool PreferencesDialog::isOdomSensorAsGt() const
{
	return _ui->checkBox_odom_sensor_use_as_gt->isChecked();
}
int PreferencesDialog::getOdomRegistrationApproach() const
{
	return _ui->odom_registration->currentIndex();
}
double PreferencesDialog::getOdomF2MGravitySigma() const
{
	return _ui->odom_f2m_gravitySigma->value();
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
int PreferencesDialog::getOctomapRenderingType() const
{
	return _ui->comboBox_octomap_renderingType->currentIndex();
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
int PreferencesDialog::getOctomapPointSize() const
{
	return _ui->spinBox_octomap_pointSize->value();
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
double PreferencesDialog::getNormalRadiusSearch() const
{
	return _ui->doubleSpinBox_normalRadiusSearch->value();
}
double PreferencesDialog::getScanCeilingFilteringHeight() const
{
	return _ui->doubleSpinBox_ceilingFilterHeight_scan->value();
}
double PreferencesDialog::getScanFloorFilteringHeight() const
{
	return _ui->doubleSpinBox_floorFilterHeight_scan->value();
}
int PreferencesDialog::getScanNormalKSearch() const
{
	return _ui->spinBox_normalKSearch_scan->value();
}
double PreferencesDialog::getScanNormalRadiusSearch() const
{
	return _ui->doubleSpinBox_normalRadiusSearch_scan->value();
}

bool PreferencesDialog::isGraphsShown() const
{
	return _ui->checkBox_showGraphs->isChecked();
}
bool PreferencesDialog::isLabelsShown() const
{
	return _ui->checkBox_showLabels->isChecked();
}
bool PreferencesDialog::isFramesShown() const
{
	return _ui->checkBox_showFrames->isChecked();
}
bool PreferencesDialog::isLandmarksShown() const
{
	return _ui->checkBox_showLandmarks->isChecked();
}
double PreferencesDialog::landmarkVisSize() const
{
	return _ui->doubleSpinBox_landmarkSize->value();
}
bool PreferencesDialog::isIMUGravityShown(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingGravity[index]->isChecked();
}
double PreferencesDialog::getIMUGravityLength(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingGravityLength[index]->value();
}
bool PreferencesDialog::isIMUAccShown() const
{
	return _ui->checkBox_showIMUAcc->isChecked();
}
bool PreferencesDialog::isMarkerDetection() const
{
	return _ui->RGBDMarkerDetection->isChecked();
}
double PreferencesDialog::getMarkerLength() const
{
	return _ui->ArucoMarkerLength->value();
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
int PreferencesDialog::getCloudColorScheme(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingColorScheme[index]->value();
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
double PreferencesDialog::getScanMaxRange(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingMaxRange[index]->value();
}
double PreferencesDialog::getScanMinRange(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingMinRange[index]->value();
}
double PreferencesDialog::getCloudVoxelSizeScan(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingVoxelSizeScan[index]->value();
}
int PreferencesDialog::getScanColorScheme(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingColorSchemeScan[index]->value();
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
bool PreferencesDialog::isFrustumsShown(int index) const
{
	UASSERT(index >= 0 && index <= 1);
	return _3dRenderingShowFrustums[index]->isEnabled() && _3dRenderingShowFrustums[index]->isChecked();
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
int PreferencesDialog::getGridMapSensor() const
{
	return _ui->comboBox_grid_sensor->currentIndex();
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

PreferencesDialog::Src PreferencesDialog::getOdomSourceDriver() const
{
	if(_ui->comboBox_odom_sensor->currentIndex() == 1)
	{
		//RealSense2
		return kSrcStereoRealSense2;
	}
	else if(_ui->comboBox_odom_sensor->currentIndex() == 2)
	{
		//Zed SDK
		return kSrcStereoZed;
	}
	else if(_ui->comboBox_odom_sensor->currentIndex() != 0)
	{
		UERROR("Not implemented!");
	}
	return kSrcUndef;
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

QString PreferencesDialog::getIMUPath() const
{
	return _ui->lineEdit_cameraImages_path_imu->text();
}
Transform PreferencesDialog::getIMULocalTransform() const
{
	Transform t = Transform::fromString(_ui->lineEdit_cameraImages_imu_transform->text().replace("PI_2", QString::number(3.141592/2.0)).toStdString());
	if(t.isNull())
	{
		return Transform::getIdentity();
	}
	return t;
}
int PreferencesDialog::getIMURate() const
{
	return _ui->spinBox_cameraImages_max_imu_rate->value();
}

bool PreferencesDialog::isSourceDatabaseStampsUsed() const
{
	return _ui->source_checkBox_useDbStamps->isChecked();
}
bool PreferencesDialog::isSourceRGBDColorOnly() const
{
	return _ui->checkbox_rgbd_colorOnly->isChecked();
}
int PreferencesDialog::getIMUFilteringStrategy() const
{
	return _ui->comboBox_imuFilter_strategy->currentIndex();
}
bool PreferencesDialog::getIMUFilteringBaseFrameConversion() const
{
	return _ui->checkBox_imuFilter_baseFrameConversion->isChecked();
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
bool PreferencesDialog::isSourceStereoExposureCompensation() const
{
	return _ui->checkBox_stereo_exposureCompensation->isChecked();
}
bool PreferencesDialog::isSourceScanFromDepth() const
{
	return _ui->checkBox_source_scanFromDepth->isChecked();
}
int PreferencesDialog::getSourceScanDownsampleStep() const
{
	return _ui->spinBox_source_scanDownsampleStep->value();
}
double PreferencesDialog::getSourceScanRangeMin() const
{
	return _ui->doubleSpinBox_source_scanRangeMin->value();
}
double PreferencesDialog::getSourceScanRangeMax() const
{
	return _ui->doubleSpinBox_source_scanRangeMax->value();
}
double PreferencesDialog::getSourceScanVoxelSize() const
{
	return _ui->doubleSpinBox_source_scanVoxelSize->value();
}
int PreferencesDialog::getSourceScanNormalsK() const
{
	return _ui->spinBox_source_scanNormalsK->value();
}
double PreferencesDialog::getSourceScanNormalsRadius() const
{
	return _ui->doubleSpinBox_source_scanNormalsRadius->value();
}
double PreferencesDialog::getSourceScanForceGroundNormalsUp() const
{
	return _ui->doubleSpinBox_source_scanNormalsForceGroundUp->value();
}

Camera * PreferencesDialog::createCamera(bool useRawImages, bool useColor)
{
	return createCamera(
		this->getSourceDriver(), 
		_ui->lineEdit_sourceDevice->text(), 
		_ui->lineEdit_calibrationFile->text(), 
		useRawImages, 
		useColor, 
		false,
		false);
}

Camera * PreferencesDialog::createCamera(
		Src driver,
		const QString & device,
		const QString & calibrationPath,
		bool useRawImages,
		bool useColor,
		bool odomOnly,
		bool odomSensorExtrinsicsCalib)
{
	if(odomOnly && !(driver == kSrcStereoRealSense2 || driver == kSrcStereoZed))
	{
		QMessageBox::warning(this, tr("Odometry Sensor"),
				tr("Driver \%1 cannot support odometry only mode.").arg(driver), QMessageBox::Ok);
		return 0;
	}

	UDEBUG("");
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
					_ui->lineEdit_openniOniPath->text().isEmpty()?device.toStdString():_ui->lineEdit_openniOniPath->text().toStdString(),
					this->getGeneralInputRate(),
					this->getSourceLocalTransform());
		}
	}
	else if(driver == PreferencesDialog::kSrcOpenNI2)
	{
		camera = new CameraOpenNI2(
				_ui->lineEdit_openni2OniPath->text().isEmpty()?device.toStdString():_ui->lineEdit_openni2OniPath->text().toStdString(),
				useColor?CameraOpenNI2::kTypeColorDepth:CameraOpenNI2::kTypeIRDepth,
				this->getGeneralInputRate(),
				this->getSourceLocalTransform());
	}
	else if(driver == PreferencesDialog::kSrcFreenect)
	{
		camera = new CameraFreenect(
				device.isEmpty()?0:atoi(device.toStdString().c_str()),
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
			device.isEmpty()?0:atoi(device.toStdString().c_str()),
			useRawImages?CameraFreenect2::kTypeColorIR:(CameraFreenect2::Type)_ui->comboBox_freenect2Format->currentIndex(),
			this->getGeneralInputRate(),
			this->getSourceLocalTransform(),
			_ui->doubleSpinBox_freenect2MinDepth->value(),
			_ui->doubleSpinBox_freenect2MaxDepth->value(),
			_ui->checkBox_freenect2BilateralFiltering->isChecked(),
			_ui->checkBox_freenect2EdgeAwareFiltering->isChecked(),
			_ui->checkBox_freenect2NoiseFiltering->isChecked(),
			_ui->lineEdit_freenect2Pipeline->text().toStdString());
	}
	else if (driver == kSrcK4W2)
	{
		camera = new CameraK4W2(
			device.isEmpty() ? 0 : atoi(device.toStdString().c_str()),
			(CameraK4W2::Type)_ui->comboBox_k4w2Format->currentIndex(),
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());
	}
	else if (driver == kSrcK4A)
	{
		if (!_ui->lineEdit_k4a_mkv->text().isEmpty())
		{
			// playback
			camera = new CameraK4A(
				_ui->lineEdit_k4a_mkv->text().toStdString().c_str(),
				_ui->source_checkBox_useMKVStamps->isChecked() ? -1 : this->getGeneralInputRate(),
				this->getSourceLocalTransform());
		}
		else
		{
			camera = new CameraK4A(
				device.isEmpty() ? 0 : atoi(device.toStdString().c_str()),
				this->getGeneralInputRate(),
				this->getSourceLocalTransform());
		}
		
		((CameraK4A*)camera)->setIRDepthFormat(_ui->checkbox_k4a_irDepth->isChecked());
		((CameraK4A*)camera)->setPreferences(_ui->comboBox_k4a_rgb_resolution->currentIndex(),
						     _ui->comboBox_k4a_framerate->currentIndex(),
						     _ui->comboBox_k4a_depth_resolution->currentIndex());
	}
	else if (driver == kSrcRealSense)
	{
		if(useRawImages && _ui->comboBox_realsenseRGBSource->currentIndex()!=2)
		{
			QMessageBox::warning(this, tr("Calibration"),
					tr("Using raw images for \"RealSense\" driver is not yet supported for color and infrared streams. "
						"Factory calibration loaded from RealSense is used."), QMessageBox::Ok);
			return 0;
		}
		else
		{
			camera = new CameraRealSense(
				device.isEmpty() ? 0 : atoi(device.toStdString().c_str()),
				_ui->comboBox_realsensePresetRGB->currentIndex(),
				_ui->comboBox_realsensePresetDepth->currentIndex(),
				_ui->checkbox_realsenseOdom->isChecked(),
				this->getGeneralInputRate(),
				this->getSourceLocalTransform());
			((CameraRealSense*)camera)->setDepthScaledToRGBSize(_ui->checkbox_realsenseDepthScaledToRGBSize->isChecked());
			((CameraRealSense*)camera)->setRGBSource((CameraRealSense::RGBSource)_ui->comboBox_realsenseRGBSource->currentIndex());
		}
	}
	else if (driver == kSrcRealSense2 || driver == kSrcStereoRealSense2)
	{
		if(driver == kSrcRealSense2 && useRawImages)
		{
			QMessageBox::warning(this, tr("Calibration"),
					tr("Using raw images for \"RealSense\" driver is not yet supported. "
						"Factory calibration loaded from RealSense2 is used."), QMessageBox::Ok);
			return 0;
		}
		else
		{
			camera = new CameraRealSense2(
				device.isEmpty()&&driver == kSrcStereoRealSense2?"T265":device.toStdString(),
				this->getGeneralInputRate(),
				this->getSourceLocalTransform());
			((CameraRealSense2*)camera)->publishInterIMU(_ui->checkbox_publishInterIMU->isChecked());
			if(driver == kSrcStereoRealSense2)
			{
				((CameraRealSense2*)camera)->setImagesRectified((_ui->checkBox_stereo_rectify->isEnabled() && _ui->checkBox_stereo_rectify->isChecked()) && !useRawImages);
				((CameraRealSense2*)camera)->setOdomProvided(_ui->comboBox_odom_sensor->currentIndex() == 1 || odomOnly, odomOnly, odomSensorExtrinsicsCalib);
			}
			else
			{
				((CameraRealSense2*)camera)->setEmitterEnabled(_ui->checkbox_rs2_emitter->isChecked());
				((CameraRealSense2*)camera)->setIRFormat(_ui->checkbox_rs2_irMode->isChecked(), _ui->checkbox_rs2_irDepth->isChecked());
				((CameraRealSense2*)camera)->setResolution(_ui->spinBox_rs2_width->value(), _ui->spinBox_rs2_height->value(), _ui->spinBox_rs2_rate->value());
				((CameraRealSense2*)camera)->setDepthResolution(_ui->spinBox_rs2_width_depth->value(), _ui->spinBox_rs2_height_depth->value(), _ui->spinBox_rs2_rate_depth->value());
				((CameraRealSense2*)camera)->setGlobalTimeSync(_ui->checkbox_rs2_globalTimeStync->isChecked());
				((CameraRealSense2*)camera)->setDualMode(_ui->comboBox_odom_sensor->currentIndex()==1, Transform::fromString(_ui->lineEdit_odom_sensor_extrinsics->text().toStdString()));
				((CameraRealSense2*)camera)->setJsonConfig(_ui->lineEdit_rs2_jsonFile->text().toStdString());
			}
		}
	}
	else if (driver == kSrcStereoMyntEye)
	{
		if(driver == kSrcStereoMyntEye && useRawImages)
		{
			QMessageBox::warning(this, tr("Calibration"),
					tr("Using raw images for \"MyntEye\" driver is not yet supported. "
						"Factory calibration loaded from MyntEye is used."), QMessageBox::Ok);
			return 0;
		}
		else
		{
			camera = new CameraMyntEye(
				device.toStdString(),
				_ui->checkbox_stereoMyntEye_rectify->isChecked(),
				_ui->checkbox_stereoMyntEye_depth->isChecked(),
				this->getGeneralInputRate(),
				this->getSourceLocalTransform());
			((CameraMyntEye*)camera)->publishInterIMU(_ui->checkbox_publishInterIMU->isChecked());
			if(_ui->checkbox_stereoMyntEye_autoExposure->isChecked())
			{
				((CameraMyntEye*)camera)->setAutoExposure();
			}
			else
			{
				((CameraMyntEye*)camera)->setManualExposure(
						_ui->spinBox_stereoMyntEye_gain->value(),
						_ui->spinBox_stereoMyntEye_brightness->value(),
						_ui->spinBox_stereoMyntEye_contrast->value());
			}
			((CameraMyntEye*)camera)->setIrControl(
					_ui->spinBox_stereoMyntEye_irControl->value());
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
		((CameraRGBDImages*)camera)->setStartIndex(_ui->spinBox_cameraRGBDImages_startIndex->value());
		((CameraRGBDImages*)camera)->setMaxFrames(_ui->spinBox_cameraRGBDImages_maxFrames->value());
		((CameraRGBDImages*)camera)->setBayerMode(_ui->comboBox_cameraImages_bayerMode->currentIndex()-1);
		((CameraRGBDImages*)camera)->setOdometryPath(_ui->lineEdit_cameraImages_odom->text().toStdString(), _ui->comboBox_cameraImages_odomFormat->currentIndex());
		((CameraRGBDImages*)camera)->setGroundTruthPath(_ui->lineEdit_cameraImages_gt->text().toStdString(), _ui->comboBox_cameraImages_gtFormat->currentIndex());
		((CameraRGBDImages*)camera)->setMaxPoseTimeDiff(_ui->doubleSpinBox_maxPoseTimeDiff->value());
		((CameraRGBDImages*)camera)->setScanPath(
						_ui->lineEdit_cameraImages_path_scans->text().isEmpty()?"":_ui->lineEdit_cameraImages_path_scans->text().append(QDir::separator()).toStdString(),
						_ui->spinBox_cameraImages_max_scan_pts->value(),
						this->getLaserLocalTransform());
		((CameraRGBDImages*)camera)->setTimestamps(
				_ui->checkBox_cameraImages_timestamps->isChecked(),
				_ui->lineEdit_cameraImages_timestamps->text().toStdString(),
				_ui->checkBox_cameraImages_syncTimeStamps->isChecked());
		((CameraRGBDImages*)camera)->setConfigForEachFrame(_ui->checkBox_cameraImages_configForEachFrame->isChecked());
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
			(_ui->checkBox_stereo_rectify->isEnabled() && _ui->checkBox_stereo_rectify->isChecked()) && !useRawImages,
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());
		((CameraStereoImages*)camera)->setStartIndex(_ui->spinBox_cameraStereoImages_startIndex->value());
		((CameraStereoImages*)camera)->setMaxFrames(_ui->spinBox_cameraStereoImages_maxFrames->value());
		((CameraStereoImages*)camera)->setBayerMode(_ui->comboBox_cameraImages_bayerMode->currentIndex()-1);
		((CameraStereoImages*)camera)->setOdometryPath(_ui->lineEdit_cameraImages_odom->text().toStdString(), _ui->comboBox_cameraImages_odomFormat->currentIndex());
		((CameraStereoImages*)camera)->setGroundTruthPath(_ui->lineEdit_cameraImages_gt->text().toStdString(), _ui->comboBox_cameraImages_gtFormat->currentIndex());
		((CameraStereoImages*)camera)->setMaxPoseTimeDiff(_ui->doubleSpinBox_maxPoseTimeDiff->value());
		((CameraStereoImages*)camera)->setScanPath(
						_ui->lineEdit_cameraImages_path_scans->text().isEmpty()?"":_ui->lineEdit_cameraImages_path_scans->text().append(QDir::separator()).toStdString(),
						_ui->spinBox_cameraImages_max_scan_pts->value(),
						this->getLaserLocalTransform());
		((CameraStereoImages*)camera)->setTimestamps(
				_ui->checkBox_cameraImages_timestamps->isChecked(),
				_ui->lineEdit_cameraImages_timestamps->text().toStdString(),
				_ui->checkBox_cameraImages_syncTimeStamps->isChecked());
		((CameraRGBDImages*)camera)->setConfigForEachFrame(_ui->checkBox_cameraImages_configForEachFrame->isChecked());

	}
	else if (driver == kSrcStereoUsb)
	{
		if(_ui->spinBox_stereo_right_device->value()>=0)
		{
			camera = new CameraStereoVideo(
				device.isEmpty() ? 0 : atoi(device.toStdString().c_str()),
				_ui->spinBox_stereo_right_device->value(),
				(_ui->checkBox_stereo_rectify->isEnabled() && _ui->checkBox_stereo_rectify->isChecked()) && !useRawImages,
				this->getGeneralInputRate(),
				this->getSourceLocalTransform());
		}
		else
		{
			camera = new CameraStereoVideo(
				device.isEmpty() ? 0 : atoi(device.toStdString().c_str()),
				(_ui->checkBox_stereo_rectify->isEnabled() && _ui->checkBox_stereo_rectify->isChecked()) && !useRawImages,
				this->getGeneralInputRate(),
				this->getSourceLocalTransform());
		}
		((CameraStereoVideo*)camera)->setResolution(_ui->spinBox_stereousbcam_streamWidth->value(), _ui->spinBox_stereousbcam_streamHeight->value());
	}
	else if(driver == kSrcStereoVideo)
	{
		if(!_ui->lineEdit_cameraStereoVideo_path_2->text().isEmpty())
		{
			// left and right videos
			camera = new CameraStereoVideo(
					_ui->lineEdit_cameraStereoVideo_path->text().toStdString(),
					_ui->lineEdit_cameraStereoVideo_path_2->text().toStdString(),
					(_ui->checkBox_stereo_rectify->isEnabled() && _ui->checkBox_stereo_rectify->isChecked()) && !useRawImages,
					this->getGeneralInputRate(),
					this->getSourceLocalTransform());
		}
		else
		{
			// side-by-side video
			camera = new CameraStereoVideo(
					_ui->lineEdit_cameraStereoVideo_path->text().toStdString(),
					(_ui->checkBox_stereo_rectify->isEnabled() && _ui->checkBox_stereo_rectify->isChecked()) && !useRawImages,
					this->getGeneralInputRate(),
					this->getSourceLocalTransform());
		}
	}
    
    else if (driver == kSrcStereoTara)
    {

            camera = new CameraStereoTara(
                device.isEmpty()?0:atoi(device.toStdString().c_str()),
                (_ui->checkBox_stereo_rectify->isEnabled() && _ui->checkBox_stereo_rectify->isChecked()) && !useRawImages,
                this->getGeneralInputRate(),
                this->getSourceLocalTransform());

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
				_ui->comboBox_odom_sensor->currentIndex() == 2,
				this->getGeneralInputRate(),
				this->getSourceLocalTransform(),
				_ui->checkbox_stereoZed_selfCalibration->isChecked(),
				_ui->loopClosure_bowForce2D->isChecked(),
				_ui->spinBox_stereoZed_texturenessConfidenceThr->value());
		}
		else
		{
			camera = new CameraStereoZed(
				device.isEmpty()?0:atoi(device.toStdString().c_str()),
				_ui->comboBox_stereoZed_resolution->currentIndex(),
				// depth should be enabled for zed vo to work
				_ui->comboBox_stereoZed_quality->currentIndex()==0&&odomOnly?1:_ui->comboBox_stereoZed_quality->currentIndex(),
				_ui->comboBox_stereoZed_sensingMode->currentIndex(),
				_ui->spinBox_stereoZed_confidenceThr->value(),
				_ui->comboBox_odom_sensor->currentIndex() == 2 || odomOnly,
				this->getGeneralInputRate(),
				this->getSourceLocalTransform(),
				_ui->checkbox_stereoZed_selfCalibration->isChecked(),
				_ui->loopClosure_bowForce2D->isChecked(),
				_ui->spinBox_stereoZed_texturenessConfidenceThr->value());
		}
		((CameraStereoZed*)camera)->publishInterIMU(_ui->checkbox_publishInterIMU->isChecked());
	}
	else if (driver == kSrcStereoZedOC)
	{
		UDEBUG("ZEDOC");
		camera = new CameraStereoZedOC(
			device.isEmpty()?-1:atoi(device.toStdString().c_str()),
			_ui->comboBox_stereoZedOC_resolution->currentIndex(),
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());
	}
	else if (driver == kSrcStereoDepthAI)
	{
		UDEBUG("DepthAI");
		camera = new CameraDepthAI(
			device.toStdString().c_str(),
			_ui->comboBox_depthai_resolution->currentIndex(),
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());
		((CameraDepthAI*)camera)->setOutputDepth(_ui->checkBox_depthai_depth->isChecked(), _ui->spinBox_depthai_confidence->value());
		((CameraDepthAI*)camera)->setIMUFirmwareUpdate(_ui->checkBox_depthai_imu_firmware_update->isChecked());
	}
	else if(driver == kSrcUsbDevice)
	{
		camera = new CameraVideo(
			device.isEmpty()?0:atoi(device.toStdString().c_str()),
			(_ui->checkBox_rgb_rectify->isEnabled() && _ui->checkBox_rgb_rectify->isChecked()) && !useRawImages,
			this->getGeneralInputRate(),
			this->getSourceLocalTransform());
		((CameraVideo*)camera)->setResolution(_ui->spinBox_usbcam_streamWidth->value(), _ui->spinBox_usbcam_streamHeight->value());
	}
	else if(driver == kSrcVideo)
	{
		camera = new CameraVideo(
			_ui->source_video_lineEdit_path->text().toStdString(),
			(_ui->checkBox_rgb_rectify->isEnabled() && _ui->checkBox_rgb_rectify->isChecked()) && !useRawImages,
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
		((CameraImages*)camera)->setMaxFrames(_ui->source_images_spinBox_maxFrames->value());
		((CameraImages*)camera)->setImagesRectified((_ui->checkBox_rgb_rectify->isEnabled() && _ui->checkBox_rgb_rectify->isChecked()) && !useRawImages);

		((CameraImages*)camera)->setBayerMode(_ui->comboBox_cameraImages_bayerMode->currentIndex()-1);
		((CameraImages*)camera)->setOdometryPath(
				_ui->lineEdit_cameraImages_odom->text().toStdString(),
				_ui->comboBox_cameraImages_odomFormat->currentIndex());
		((CameraImages*)camera)->setGroundTruthPath(
				_ui->lineEdit_cameraImages_gt->text().toStdString(),
				_ui->comboBox_cameraImages_gtFormat->currentIndex());
		((CameraImages*)camera)->setMaxPoseTimeDiff(_ui->doubleSpinBox_maxPoseTimeDiff->value());
		((CameraImages*)camera)->setScanPath(
						_ui->lineEdit_cameraImages_path_scans->text().isEmpty()?"":_ui->lineEdit_cameraImages_path_scans->text().append(QDir::separator()).toStdString(),
						_ui->spinBox_cameraImages_max_scan_pts->value(),
						this->getLaserLocalTransform());
		((CameraImages*)camera)->setDepthFromScan(
				_ui->groupBox_depthFromScan->isChecked(),
				!_ui->groupBox_depthFromScan_fillHoles->isChecked()?0:_ui->radioButton_depthFromScan_vertical->isChecked()?1:-1,
				_ui->checkBox_depthFromScan_fillBorders->isChecked());
		((CameraImages*)camera)->setTimestamps(
				_ui->checkBox_cameraImages_timestamps->isChecked(),
				_ui->lineEdit_cameraImages_timestamps->text().toStdString(),
				_ui->checkBox_cameraImages_syncTimeStamps->isChecked());
		((CameraRGBDImages*)camera)->setConfigForEachFrame(_ui->checkBox_cameraImages_configForEachFrame->isChecked());
	}
	else if(driver == kSrcDatabase)
	{
		camera = new DBReader(_ui->source_database_lineEdit_path->text().toStdString(),
				_ui->source_checkBox_useDbStamps->isChecked()?-1:this->getGeneralInputRate(),
				_ui->source_checkBox_ignoreOdometry->isChecked(),
				_ui->source_checkBox_ignoreGoalDelay->isChecked(),
				_ui->source_checkBox_ignoreGoals->isChecked(),
				_ui->source_spinBox_databaseStartId->value(),
				_ui->source_spinBox_database_cameraIndex->value(),
				_ui->source_spinBox_databaseStopId->value(),
				!_ui->general_checkBox_createIntermediateNodes->isChecked(),
				_ui->source_checkBox_ignoreLandmarks->isChecked(),
				_ui->source_checkBox_ignoreFeatures->isChecked());
	}
	else
	{
		UFATAL("Source driver undefined (%d)!", driver);
	}

	if(camera)
	{
		UDEBUG("Init");
		QString dir = this->getCameraInfoDir();
		QString calibrationFile = calibrationPath;
		if(!(driver >= kSrcRGB && driver <= kSrcVideo))
		{
			calibrationFile.remove("_left.yaml").remove("_right.yaml").remove("_pose.yaml").remove("_rgb.yaml").remove("_depth.yaml");
		}
		QString name = QFileInfo(calibrationFile.remove(".yaml")).fileName();
		if(!calibrationPath.isEmpty())
		{
			QDir d = QFileInfo(calibrationPath).dir();
			QString tmp = calibrationPath;
			if(!tmp.remove(QFileInfo(calibrationPath).baseName()).isEmpty())
			{
				dir = d.absolutePath();
			}
		}

		UDEBUG("useRawImages=%d dir=%s", useRawImages?1:0, dir.toStdString().c_str());
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
				((CameraOpenNI2*)camera)->setIRDepthShift(_ui->openni2_hshift->value(), _ui->openni2_vshift->value());
				((CameraOpenNI2*)camera)->setMirroring(_ui->openni2_mirroring->isChecked());
				((CameraOpenNI2*)camera)->setDepthDecimation(_ui->openni2_depth_decimation->value());
			}
		}
	}

	UDEBUG("");
	return camera;
}

Camera * PreferencesDialog::createOdomSensor(Transform & extrinsics, double & timeOffset, float & scaleFactor)
{
	Src driver = getOdomSourceDriver();
	if(driver != kSrcUndef)
	{
		if(driver == getSourceDriver() &&
		  _ui->lineEdit_odomSourceDevice->text().compare(_ui->lineEdit_sourceDevice->text()) == 0)
		{
			QMessageBox::warning(this, tr("Odometry Sensor"),
					tr("Cannot create an odometry sensor with same driver than camera AND with same "
					"device. Change device ID of the odometry or camera sensor. To use odometry option "
					"from a single camera, look at the parameters of that driver to enable it, "
					"then disable odometry sensor. "), QMessageBox::Ok);
			return 0;
		}

		extrinsics = Transform::fromString(_ui->lineEdit_odom_sensor_extrinsics->text().replace("PI_2", QString::number(3.141592/2.0)).toStdString());
		timeOffset = _ui->doubleSpinBox_odom_sensor_time_offset->value()/1000.0;
		scaleFactor = (float)_ui->doubleSpinBox_odom_sensor_scale_factor->value();

		return createCamera(driver, _ui->lineEdit_odomSourceDevice->text(), _ui->lineEdit_odom_sensor_path_calibration->text(), false, true, true, false);
	}
	return 0;
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
int PreferencesDialog::getKpMaxFeatures() const
{
	return _ui->surf_spinBox_wordsPerImageTarget->value();
}
bool PreferencesDialog::isPriorIgnored() const
{
	return _ui->graphOptimization_priorsIgnored->isChecked();
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

	IMUThread * imuThread = 0;
	if((this->getSourceDriver() == kSrcStereoImages ||
	   this->getSourceDriver() == kSrcRGBDImages ||
	   this->getSourceDriver() == kSrcImages) &&
	   !_ui->lineEdit_cameraImages_path_imu->text().isEmpty())
	{
		if(this->getOdomStrategy() != Odometry::kTypeOkvis &&
		   this->getOdomStrategy() != Odometry::kTypeMSCKF &&
		   this->getOdomStrategy() != Odometry::kTypeVINS &&
		   this->getOdomStrategy() != Odometry::kTypeOpenVINS)
		{
			QMessageBox::warning(this, tr("Source IMU Path"),
					tr("IMU path is set but odometry chosen doesn't support asynchronous IMU, ignoring IMU..."), QMessageBox::Ok);
		}
		else
		{
			imuThread = new IMUThread(_ui->spinBox_cameraImages_max_imu_rate->value(), this->getIMULocalTransform());
			if(!imuThread->init(_ui->lineEdit_cameraImages_path_imu->text().toStdString()))
			{
				QMessageBox::warning(this, tr("Source IMU Path"),
					tr("Initialization of IMU data has failed! Path=%1.").arg(_ui->lineEdit_cameraImages_path_imu->text()), QMessageBox::Ok);
				delete camera;
				delete imuThread;
				return;
			}
		}
	}

	ParametersMap parameters = this->getAllParameters();
	if(getOdomRegistrationApproach() < 3)
	{
		uInsert(parameters, ParametersPair(Parameters::kRegStrategy(), uNumber2Str(getOdomRegistrationApproach())));
	}

	int odomStrategy = Parameters::defaultOdomStrategy();
	Parameters::parse(parameters, Parameters::kOdomStrategy(), odomStrategy);
	if(odomStrategy != 1)
	{
		// Only Frame To Frame supports  all VisCorType
		parameters.erase(Parameters::kVisCorType());
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
	cameraThread.setStereoExposureCompensation(_ui->checkBox_stereo_exposureCompensation->isChecked());
	cameraThread.setScanParameters(
			_ui->checkBox_source_scanFromDepth->isChecked(),
			_ui->spinBox_source_scanDownsampleStep->value(),
			_ui->doubleSpinBox_source_scanRangeMin->value(),
			_ui->doubleSpinBox_source_scanRangeMax->value(),
			_ui->doubleSpinBox_source_scanVoxelSize->value(),
			_ui->spinBox_source_scanNormalsK->value(),
			_ui->doubleSpinBox_source_scanNormalsRadius->value(),
			(float)_ui->doubleSpinBox_source_scanNormalsForceGroundUp->value());
	if(_ui->comboBox_imuFilter_strategy->currentIndex()>0 && dynamic_cast<DBReader*>(camera) == 0)
	{
		cameraThread.enableIMUFiltering(_ui->comboBox_imuFilter_strategy->currentIndex()-1, this->getAllParameters(), _ui->checkBox_imuFilter_baseFrameConversion->isChecked());
	}
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
	if(imuThread)
	{
		UEventsManager::createPipe(imuThread, &odomThread, "IMUEvent");
	}
	UEventsManager::createPipe(&odomThread, odomViewer, "OdometryEvent");
	UEventsManager::createPipe(odomViewer, &odomThread, "OdometryResetEvent");

	odomThread.start();
	cameraThread.start();

	if(imuThread)
	{
		imuThread->start();
	}

	odomViewer->exec();
	delete odomViewer;

	if(imuThread)
	{
		imuThread->join(true);
		delete imuThread;
	}
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
		cameraThread.setStereoExposureCompensation(_ui->checkBox_stereo_exposureCompensation->isChecked());
		cameraThread.setScanParameters(
				_ui->checkBox_source_scanFromDepth->isChecked(),
				_ui->spinBox_source_scanDownsampleStep->value(),
				_ui->doubleSpinBox_source_scanRangeMin->value(),
				_ui->doubleSpinBox_source_scanRangeMax->value(),
				_ui->doubleSpinBox_source_scanVoxelSize->value(),
				_ui->spinBox_source_scanNormalsK->value(),
				_ui->doubleSpinBox_source_scanNormalsRadius->value(),
				(float)_ui->doubleSpinBox_source_scanNormalsForceGroundUp->value());
		if(_ui->comboBox_imuFilter_strategy->currentIndex()>0 && dynamic_cast<DBReader*>(camera) == 0)
		{
			cameraThread.enableIMUFiltering(_ui->comboBox_imuFilter_strategy->currentIndex()-1, this->getAllParameters(), _ui->checkBox_imuFilter_baseFrameConversion->isChecked());
		}
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
							"checkerboard with the IR camera. Do you want to continue?"),
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
								"the cameras and the checkerboard don't move and that both "
								"cameras can see the checkerboard. We will repeat this "
								"multiple times. Each time, you will have to move the camera (or "
								"checkerboard) for a different point of view. Do you want to "
								"continue?"),
								QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

				bool ok = false;
				int totalSamples = 0;
				if(button == QMessageBox::Yes)
				{
					totalSamples = QInputDialog::getInt(this, tr("Calibration"), tr("Samples: "), 1, 1, 99, 1, &ok);
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
										tr("A stereo pair has been taken (total=%1/%2). Move the checkerboard or "
											"camera to another position. Press \"Yes\" when you are ready "
											"for the next capture.").arg(count).arg(totalSamples),
											QMessageBox::Yes | QMessageBox::Abort, QMessageBox::Yes);
								}
							}
							else
							{
								button = QMessageBox::question(this, tr("Calibration"),
										tr("Could not detect the checkerboard on both images or "
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
		bool fisheye = driver == kSrcStereoRealSense2;
		_calibrationDialog->setStereoMode(this->getSourceType() != kSrcRGB && driver != kSrcRealSense, freenect2?"rgb":"left", freenect2?"depth":"right"); // RGB+Depth or left+right
		_calibrationDialog->setCameraName("");
		_calibrationDialog->setSwitchedImages(freenect2);
		_calibrationDialog->setFisheyeImages(fisheye);
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

void PreferencesDialog::calibrateOdomSensorExtrinsics()
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

	Src odomDriver;
	if(_ui->comboBox_odom_sensor->currentIndex() == 0)
	{
		QMessageBox::warning(this,
				   tr("Calibration"),
				   tr("No odometry sensor selected!"));
		return;
	}
	else if(_ui->comboBox_odom_sensor->currentIndex() == 1)
	{
		odomDriver = kSrcStereoRealSense2;
	}
	else if(_ui->comboBox_odom_sensor->currentIndex() == 2)
	{
		odomDriver = kSrcStereoZed;
	}
	else
	{
		UERROR("Odom sensor %d not implemented", _ui->comboBox_odom_sensor->currentIndex());
		return;
	}


	// 3 steps calibration: RGB -> IR -> Extrinsic
	QMessageBox::StandardButton button = QMessageBox::question(this, tr("Calibration"),
			tr("We will calibrate the extrinsics. Important: Make sure "
				"the cameras and the checkerboard don't move and that both "
				"cameras can see the checkerboard. We will repeat this "
				"multiple times. Each time, you will have to move the camera (or "
				"checkerboard) for a different point of view. Do you want to "
				"continue?"),
				QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

	if(button == QMessageBox::Yes)
	{
		_calibrationDialog->setSavingDirectory(this->getCameraInfoDir());

		bool ok = false;
		int totalSamples = 0;
		if(button == QMessageBox::Yes)
		{
			QDialog dialog(this);
			QFormLayout form(&dialog);

			// Add the lineEdits with their respective labels
			QSpinBox * samples = new QSpinBox(&dialog);
			samples->setMinimum(1);
			samples->setMaximum(99);
			samples->setValue(1);
			QSpinBox * boardWidth = new QSpinBox(&dialog);
			boardWidth->setMinimum(2);
			boardWidth->setMaximum(99);
			boardWidth->setValue(_calibrationDialog->boardWidth());
			QSpinBox * boardHeight = new QSpinBox(&dialog);
			boardHeight->setMinimum(2);
			boardHeight->setMaximum(99);
			boardHeight->setValue(_calibrationDialog->boardHeight());
			QDoubleSpinBox * squareSize = new QDoubleSpinBox(&dialog);
			squareSize->setDecimals(4);
			squareSize->setMinimum(0.0001);
			squareSize->setMaximum(9);
			squareSize->setValue(_calibrationDialog->squareSize());
			squareSize->setSuffix(" m");
			form.addRow("Samples: ", samples);
			form.addRow("Checkerboard Width: ", boardWidth);
			form.addRow("Checkerboard Height: ", boardHeight);
			form.addRow("Checkerboard Square Size: ", squareSize);

			// Add some standard buttons (Cancel/Ok) at the bottom of the dialog
			QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
			                           Qt::Horizontal, &dialog);
			form.addRow(&buttonBox);
			QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
			QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));

			// Show the dialog as modal
			if (dialog.exec() == QDialog::Accepted) {
				_calibrationDialog->setBoardWidth(boardWidth->value());
				_calibrationDialog->setBoardHeight(boardHeight->value());
				_calibrationDialog->setSquareSize(squareSize->value());
				totalSamples = samples->value();
			    ok = true;
			}
		}

		if(ok)
		{
			int count = 0;
			_calibrationDialog->setStereoMode(true, "camera", "odom_sensor"); // this forces restart
			_calibrationDialog->setCameraName("");
			_calibrationDialog->setModal(true);
			_calibrationDialog->setProgressVisibility(false);
			_calibrationDialog->show();

			CameraModel cameraModel;
			CameraModel odomSensorModel;
			std::string serial;
			for(;count < totalSamples && button == QMessageBox::Yes; )
			{
				// Step 3: Extrinsics
				Camera * camera = this->createCamera(
						odomDriver,
						_ui->lineEdit_odomSourceDevice->text(),
						_ui->lineEdit_odom_sensor_path_calibration->text(),
						false, true, false, true); // Odom sensor
				if(!camera)
				{
					return;
				}
				else if(!camera->isCalibrated())
				{
					QMessageBox::warning(_calibrationDialog, tr("Calibration"),
							tr("Odom sensor is not calibrated. Camera and odometry sensor should be individually calibrated (intrinsics) before calibrating the extrinsics between them. Aborting..."), QMessageBox::Ok);
					delete camera;
					return;
				}
				SensorData odomSensorData = camera->takeImage();
				if(odomSensorData.cameraModels().size() == 1) {
					odomSensorModel = odomSensorData.cameraModels()[0];
				}
				else if(odomSensorData.stereoCameraModels().size() == 1) {
					odomSensorModel = odomSensorData.stereoCameraModels()[0].left();
				}
				delete camera;

				int currentIndex = _ui->comboBox_odom_sensor->currentIndex();
				_ui->comboBox_odom_sensor->setCurrentIndex(0);
				camera = this->createCamera(false, true); // Camera
				_ui->comboBox_odom_sensor->setCurrentIndex(currentIndex);
				if(!camera)
				{
					return;
				}
				else if(!camera->isCalibrated())
				{
					QMessageBox::warning(_calibrationDialog, tr("Calibration"),
							tr("Odom sensor is not calibrated. Camera and odometry sensor should be individually calibrated (intrinsics) before calibrating the extrinsics between them. Aborting..."), QMessageBox::Ok);
					delete camera;
					return;
				}
				SensorData camData = camera->takeImage();
				serial = camera->getSerial();
				if(camData.cameraModels().size() == 1) {
					cameraModel = camData.cameraModels()[0];
				}
				else if(camData.stereoCameraModels().size() == 1) {
					cameraModel = camData.stereoCameraModels()[0].left();
				}
				delete camera;

				if(!odomSensorData.imageRaw().empty() && !camData.imageRaw().empty())
				{
					int pair = _calibrationDialog->getStereoPairs();
					_calibrationDialog->processImages(camData.imageRaw(), odomSensorData.imageRaw(), serial.c_str());
					if(_calibrationDialog->getStereoPairs() - pair > 0)
					{
						++count;
						if(count < totalSamples)
						{
							button = QMessageBox::question(_calibrationDialog, tr("Calibration"),
								tr("A stereo pair has been taken (total=%1/%2). Move the checkerboard or "
									"camera to another position. Press \"Yes\" when you are ready "
									"for the next capture.").arg(count).arg(totalSamples),
									QMessageBox::Yes | QMessageBox::Abort, QMessageBox::Yes);
						}
					}
					else
					{
						button = QMessageBox::question(_calibrationDialog, tr("Calibration"),
								tr("Could not detect the checkerboard on both images or "
								   "the point of view didn't change enough. Try again?"),
									QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
					}
				}
				else
				{
					button = QMessageBox::question(_calibrationDialog, tr("Calibration"),
								tr("Failed to start the camera. Try again?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
				}
			}
			if(count == totalSamples && button == QMessageBox::Yes)
			{
				// Assume cameras are already rectified!
				cameraModel = CameraModel("camera", cameraModel.imageSize(), cameraModel.K(), cv::Mat::zeros(1,5,CV_64FC1), cv::Mat(), cv::Mat(), cameraModel.localTransform());
				odomSensorModel = CameraModel("odom_sensor", odomSensorModel.imageSize(), odomSensorModel.K(), cv::Mat::zeros(1,5,CV_64FC1), cv::Mat(), cv::Mat(), odomSensorModel.localTransform());

				StereoCameraModel stereoModel = _calibrationDialog->stereoCalibration(cameraModel, odomSensorModel, true);
				stereoModel.setName(stereoModel.name(), "camera", "odom_sensor");
				if(stereoModel.stereoTransform().isNull())
				{
					QMessageBox::warning(_calibrationDialog, tr("Calibration"),
							tr("Extrinsic calibration has failed! Look on the terminal "
							   "for possible error messages."), QMessageBox::Ok);
					std::cout << "stereoModel: " << stereoModel << std::endl;
				}
				else
				{
					UINFO("Odom sensor local transform (pose to left cam): %s", odomSensorModel.localTransform().prettyPrint().c_str());
					UINFO("Extrinsics (odom left cam to camera left cam): %s", stereoModel.stereoTransform().prettyPrint().c_str());

					Transform t = odomSensorModel.localTransform() * stereoModel.stereoTransform() * CameraModel::opticalRotation().inverse();
					UINFO("Odom sensor frame to camera frame: %s", t.prettyPrint().c_str());

					float x,y,z,roll,pitch,yaw;
					t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
					_ui->lineEdit_odom_sensor_extrinsics->setText(QString("%1 %2 %3 %4 %5 %6")
							.arg(x).arg(y).arg(z)
							.arg(roll).arg(pitch).arg(yaw));
					QMessageBox::information(_calibrationDialog, tr("Calibration"),
							tr("Calibration is completed! Extrinsics have been computed: %1. "
							   "You can close the calibration dialog.").arg(t.prettyPrint().c_str()), QMessageBox::Ok);
				}
			}
		}
	}
}

}

