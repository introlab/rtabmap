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

#include "rtabmap/gui/MainWindow.h"

#include "ui_mainWindow.h"

#include "rtabmap/core/CameraRGB.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/IMUThread.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/ParamEvent.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/DBDriver.h"
#include "rtabmap/core/RegistrationVis.h"
#include "rtabmap/core/OccupancyGrid.h"
#include "rtabmap/core/GainCompensator.h"
#include "rtabmap/core/Recovery.h"
#include "rtabmap/core/util2d.h"

#include "rtabmap/gui/ImageView.h"
#include "rtabmap/gui/KeypointItem.h"
#include "rtabmap/gui/DataRecorder.h"
#include "rtabmap/gui/DatabaseViewer.h"
#include "rtabmap/gui/PdfPlot.h"
#include "rtabmap/gui/StatsToolBox.h"
#include "rtabmap/gui/ProgressDialog.h"
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/gui/LoopClosureViewer.h"
#include "rtabmap/gui/ExportCloudsDialog.h"
#include "rtabmap/gui/ExportBundlerDialog.h"
#include "rtabmap/gui/AboutDialog.h"
#include "rtabmap/gui/PostProcessingDialog.h"
#include "rtabmap/gui/DepthCalibrationDialog.h"
#include "rtabmap/gui/RecoveryState.h"
#include "rtabmap/gui/MultiSessionLocWidget.h"

#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include "rtabmap/utilite/UPlot.h"
#include "rtabmap/utilite/UCv2Qt.h"

#include <QtGui/QCloseEvent>
#include <QtGui/QPixmap>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <QtCore/QFileInfo>
#include <QMessageBox>
#include <QFileDialog>
#include <QGraphicsEllipseItem>
#include <QDockWidget>
#include <QtCore/QBuffer>
#include <QtCore/QTimer>
#include <QtCore/QTime>
#include <QActionGroup>
#include <QtGui/QDesktopServices>
#include <QtCore/QStringList>
#include <QtCore/QProcess>
#include <QSplashScreen>
#include <QInputDialog>
#include <QToolButton>

//RGB-D stuff
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_mapping.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/optimizer/OptimizerCVSBA.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/RegistrationIcp.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>

#ifdef RTABMAP_OCTOMAP
#include <rtabmap/core/OctoMap.h>
#endif

#ifdef HAVE_OPENCV_ARUCO
#include <opencv2/aruco.hpp>
#endif

#define LOG_FILE_NAME "LogRtabmap.txt"
#define SHARE_SHOW_LOG_FILE "share/rtabmap/showlogs.m"
#define SHARE_GET_PRECISION_RECALL_FILE "share/rtabmap/getPrecisionRecall.m"
#define SHARE_IMPORT_FILE   "share/rtabmap/importfile.m"

using namespace rtabmap;

inline static void initGuiResource() { Q_INIT_RESOURCE(GuiLib); }


namespace rtabmap {

MainWindow::MainWindow(PreferencesDialog * prefDialog, QWidget * parent, bool showSplashScreen) :
	QMainWindow(parent),
	_ui(0),
	_state(kIdle),
	_camera(0),
	_odomThread(0),
	_imuThread(0),
	_preferencesDialog(0),
	_aboutDialog(0),
	_exportCloudsDialog(0),
	_exportBundlerDialog(0),
	_dataRecorder(0),
	_lastId(0),
	_firstStamp(0.0f),
	_processingStatistics(false),
	_processingDownloadedMap(false),
	_recovering(false),
	_odometryReceived(false),
	_newDatabasePath(""),
	_newDatabasePathOutput(""),
	_openedDatabasePath(""),
	_databaseUpdated(false),
	_odomImageShow(true),
	_odomImageDepthShow(false),
	_savedMaximized(false),
	_waypointsIndex(0),
	_cachedMemoryUsage(0),
	_createdCloudsMemoryUsage(0),
	_occupancyGrid(0),
	_octomap(0),
	_odometryCorrection(Transform::getIdentity()),
	_processingOdometry(false),
	_oneSecondTimer(0),
	_elapsedTime(0),
	_posteriorCurve(0),
	_likelihoodCurve(0),
	_rawLikelihoodCurve(0),
	_exportPosesFrame(0),
	_autoScreenCaptureOdomSync(false),
	_autoScreenCaptureRAM(false),
	_autoScreenCapturePNG(false),
	_firstCall(true),
	_progressCanceled(false)
{
	ULogger::registerCurrentThread("MainWindow");
	UDEBUG("");

	initGuiResource();

	QSplashScreen * splash = 0;
	if (showSplashScreen)
	{
		QPixmap pixmap(":images/RTAB-Map.png");
		splash = new QSplashScreen(pixmap);
		splash->show();
		splash->showMessage(tr("Loading..."));
		QApplication::processEvents();
	}

	// Create dialogs
	_aboutDialog = new AboutDialog(this);
	_aboutDialog->setObjectName("AboutDialog");
	_exportCloudsDialog = new ExportCloudsDialog(this);
	_exportCloudsDialog->setObjectName("ExportCloudsDialog");
	_exportBundlerDialog = new ExportBundlerDialog(this);
	_exportBundlerDialog->setObjectName("ExportBundlerDialog");
	_postProcessingDialog = new PostProcessingDialog(this);
	_postProcessingDialog->setObjectName("PostProcessingDialog");
	_depthCalibrationDialog = new DepthCalibrationDialog(this);
	_depthCalibrationDialog->setObjectName("DepthCalibrationDialog");

	_ui = new Ui_mainWindow();
	UDEBUG("Setup ui...");
	_ui->setupUi(this);

	// Add cloud viewers
	// Note that we add them here manually because there is a crash issue
	// when adding them in a DockWidget of the *.ui file. The cloud viewer is
	// created in a widget which is not yet linked to main window when the CloudViewer constructor
	// is called (see order in generated ui file). VTK needs to get the top
	// level window at the time CloudViewer is created, otherwise it may crash on some systems.
	_cloudViewer = new CloudViewer(_ui->layout_cloudViewer);
	_cloudViewer->setObjectName("widget_cloudViewer");
	_ui->layout_cloudViewer->layout()->addWidget(_cloudViewer);
	_loopClosureViewer = new LoopClosureViewer(_ui->layout_loopClosureViewer);
	_loopClosureViewer->setObjectName("widget_loopClosureViewer");
	_ui->layout_loopClosureViewer->layout()->addWidget(_loopClosureViewer);
	UDEBUG("Setup ui... end");

	QString title("RTAB-Map[*]");
	this->setWindowTitle(title);
	this->setWindowIconText(tr("RTAB-Map"));
	this->setObjectName("MainWindow");

	//Setup dock widgets position if it is the first time the application is started.
	setDefaultViews();

	_ui->widget_mainWindow->setVisible(false);

	if(prefDialog)
	{
		_preferencesDialog = prefDialog;
		_preferencesDialog->setParent(this, Qt::Dialog);
	}
	else // Default dialog
	{
		_preferencesDialog = new PreferencesDialog(this);
	}

	_preferencesDialog->setObjectName("PreferencesDialog");
	_preferencesDialog->init();

	// Restore window geometry
	bool statusBarShown = false;
	_preferencesDialog->loadMainWindowState(this, _savedMaximized, statusBarShown);
	_preferencesDialog->loadWindowGeometry(_preferencesDialog);
	_preferencesDialog->loadWindowGeometry(_exportCloudsDialog);
	_preferencesDialog->loadWindowGeometry(_exportBundlerDialog);
	_preferencesDialog->loadWindowGeometry(_postProcessingDialog);
	_preferencesDialog->loadWindowGeometry(_depthCalibrationDialog);
	_preferencesDialog->loadWindowGeometry(_aboutDialog);
	setupMainLayout(_preferencesDialog->isVerticalLayoutUsed());

	ParametersMap parameters = _preferencesDialog->getAllParameters();
	_occupancyGrid = new OccupancyGrid(parameters);
#ifdef RTABMAP_OCTOMAP
	_octomap = new OctoMap(parameters);
#endif

	// Timer
	_oneSecondTimer = new QTimer(this);
	_oneSecondTimer->setInterval(1000);
	_elapsedTime = new QTime();
	_ui->label_elapsedTime->setText("00:00:00");
	connect(_oneSecondTimer, SIGNAL(timeout()), this, SLOT(updateElapsedTime()));
	_logEventTime = new QTime();
	_logEventTime->start();

	//Graphics scenes
	_ui->imageView_source->setBackgroundColor(_ui->imageView_source->getDefaultBackgroundColor());
	_ui->imageView_loopClosure->setBackgroundColor(_ui->imageView_loopClosure->getDefaultBackgroundColor());
	_ui->imageView_odometry->setBackgroundColor(_ui->imageView_odometry->getDefaultBackgroundColor());
	_ui->imageView_odometry->setAlpha(200);
	_preferencesDialog->loadWidgetState(_ui->imageView_source);
	_preferencesDialog->loadWidgetState(_ui->imageView_loopClosure);
	_preferencesDialog->loadWidgetState(_ui->imageView_odometry);
	_preferencesDialog->loadWidgetState(_ui->graphicsView_graphView);

	_posteriorCurve = new PdfPlotCurve("Posterior", &_cachedSignatures, this);
	_ui->posteriorPlot->addCurve(_posteriorCurve, false);
	_ui->posteriorPlot->showLegend(false);
	_ui->posteriorPlot->setFixedYAxis(0,1);
	UPlotCurveThreshold * tc;
	tc = _ui->posteriorPlot->addThreshold("Loop closure thr", float(_preferencesDialog->getLoopThr()));
	connect(this, SIGNAL(loopClosureThrChanged(qreal)), tc, SLOT(setThreshold(qreal)));

	_likelihoodCurve = new PdfPlotCurve("Likelihood", &_cachedSignatures, this);
	_ui->likelihoodPlot->addCurve(_likelihoodCurve, false);
	_ui->likelihoodPlot->showLegend(false);

	_rawLikelihoodCurve = new PdfPlotCurve("Likelihood", &_cachedSignatures, this);
	_ui->rawLikelihoodPlot->addCurve(_rawLikelihoodCurve, false);
	_ui->rawLikelihoodPlot->showLegend(false);

	_multiSessionLocWidget = new MultiSessionLocWidget(&_cachedSignatures, &_currentMapIds, this);
	_ui->layout_multiSessionLoc->layout()->addWidget(_multiSessionLocWidget);

	_progressDialog = new ProgressDialog(this);
	_progressDialog->setMinimumWidth(800);
	connect(_progressDialog, SIGNAL(canceled()), this, SLOT(cancelProgress()));

	connect(_ui->widget_mapVisibility, SIGNAL(visibilityChanged(int, bool)), this, SLOT(updateNodeVisibility(int, bool)));

	//connect stuff
	connect(_ui->actionExit, SIGNAL(triggered()), this, SLOT(close()));
	qRegisterMetaType<MainWindow::State>("MainWindow::State");
	connect(this, SIGNAL(stateChanged(MainWindow::State)), this, SLOT(changeState(MainWindow::State)));
	connect(this, SIGNAL(rtabmapEventInitReceived(int, const QString &)), this, SLOT(processRtabmapEventInit(int, const QString &)));
	qRegisterMetaType<rtabmap::RtabmapEvent3DMap>("rtabmap::RtabmapEvent3DMap");
	connect(this, SIGNAL(rtabmapEvent3DMapReceived(const rtabmap::RtabmapEvent3DMap &)), this, SLOT(processRtabmapEvent3DMap(const rtabmap::RtabmapEvent3DMap &)));
	qRegisterMetaType<rtabmap::RtabmapGlobalPathEvent>("rtabmap::RtabmapGlobalPathEvent");
	connect(this, SIGNAL(rtabmapGlobalPathEventReceived(const rtabmap::RtabmapGlobalPathEvent &)), this, SLOT(processRtabmapGlobalPathEvent(const rtabmap::RtabmapGlobalPathEvent &)));
	connect(this, SIGNAL(rtabmapLabelErrorReceived(int, const QString &)), this, SLOT(processRtabmapLabelErrorEvent(int, const QString &)));
	connect(this, SIGNAL(rtabmapGoalStatusEventReceived(int)), this, SLOT(processRtabmapGoalStatusEvent(int)));

	// Dock Widget view actions (Menu->Window)
	_ui->menuShow_view->addAction(_ui->dockWidget_imageView->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_posterior->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_likelihood->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_rawlikelihood->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_statsV2->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_console->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_cloudViewer->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_loopClosureViewer->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_mapVisibility->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_graphViewer->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_odometry->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_multiSessionLoc->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->toolBar->toggleViewAction());
	_ui->toolBar->setWindowTitle(tr("File toolbar"));
	_ui->menuShow_view->addAction(_ui->toolBar_2->toggleViewAction());
	_ui->toolBar_2->setWindowTitle(tr("Control toolbar"));
	QAction * a = _ui->menuShow_view->addAction("Progress dialog");
	a->setCheckable(false);
	connect(a, SIGNAL(triggered(bool)), _progressDialog, SLOT(show()));
	QAction * statusBarAction = _ui->menuShow_view->addAction("Status bar");
	statusBarAction->setCheckable(true);
	statusBarAction->setChecked(statusBarShown);
	connect(statusBarAction, SIGNAL(toggled(bool)), this->statusBar(), SLOT(setVisible(bool)));

	// connect actions with custom slots
	connect(_ui->actionSave_GUI_config, SIGNAL(triggered()), this, SLOT(saveConfigGUI()));
	connect(_ui->actionNew_database, SIGNAL(triggered()), this, SLOT(newDatabase()));
	connect(_ui->actionOpen_database, SIGNAL(triggered()), this, SLOT(openDatabase()));
	connect(_ui->actionClose_database, SIGNAL(triggered()), this, SLOT(closeDatabase()));
	connect(_ui->actionEdit_database, SIGNAL(triggered()), this, SLOT(editDatabase()));
	connect(_ui->actionStart, SIGNAL(triggered()), this, SLOT(startDetection()));
	connect(_ui->actionPause, SIGNAL(triggered()), this, SLOT(pauseDetection()));
	connect(_ui->actionStop, SIGNAL(triggered()), this, SLOT(stopDetection()));
	connect(_ui->actionDump_the_memory, SIGNAL(triggered()), this, SLOT(dumpTheMemory()));
	connect(_ui->actionDump_the_prediction_matrix, SIGNAL(triggered()), this, SLOT(dumpThePrediction()));
	connect(_ui->actionSend_goal, SIGNAL(triggered()), this, SLOT(sendGoal()));
	connect(_ui->actionSend_waypoints, SIGNAL(triggered()), this, SLOT(sendWaypoints()));
	connect(_ui->actionCancel_goal, SIGNAL(triggered()), this, SLOT(cancelGoal()));
	connect(_ui->actionLabel_current_location, SIGNAL(triggered()), this, SLOT(label()));
	connect(_ui->actionRemove_label, SIGNAL(triggered()), this, SLOT(removeLabel()));
	connect(_ui->actionClear_cache, SIGNAL(triggered()), this, SLOT(clearTheCache()));
	connect(_ui->actionAbout, SIGNAL(triggered()), _aboutDialog , SLOT(exec()));
	connect(_ui->actionHelp, SIGNAL(triggered()), this , SLOT(openHelp()));
	connect(_ui->actionPrint_loop_closure_IDs_to_console, SIGNAL(triggered()), this, SLOT(printLoopClosureIds()));
	connect(_ui->actionGenerate_map, SIGNAL(triggered()), this , SLOT(generateGraphDOT()));
	connect(_ui->actionRaw_format_txt, SIGNAL(triggered()), this , SLOT(exportPosesRaw()));
	connect(_ui->actionRGBD_SLAM_format_txt, SIGNAL(triggered()), this , SLOT(exportPosesRGBDSLAM()));
	connect(_ui->actionRGBD_SLAM_motion_capture_format_txt, SIGNAL(triggered()), this , SLOT(exportPosesRGBDSLAMMotionCapture()));
	connect(_ui->actionRGBD_SLAM_ID_format_txt, SIGNAL(triggered()), this , SLOT(exportPosesRGBDSLAMID()));
	connect(_ui->actionKITTI_format_txt, SIGNAL(triggered()), this , SLOT(exportPosesKITTI()));
	connect(_ui->actionTORO_graph, SIGNAL(triggered()), this , SLOT(exportPosesTORO()));
	connect(_ui->actionG2o_g2o, SIGNAL(triggered()), this , SLOT(exportPosesG2O()));
	connect(_ui->actionDelete_memory, SIGNAL(triggered()), this , SLOT(deleteMemory()));
	connect(_ui->actionDownload_all_clouds, SIGNAL(triggered()), this , SLOT(downloadAllClouds()));
	connect(_ui->actionDownload_graph, SIGNAL(triggered()), this , SLOT(downloadPoseGraph()));
	connect(_ui->actionUpdate_cache_from_database, SIGNAL(triggered()), this, SLOT(updateCacheFromDatabase()));
	connect(_ui->actionAnchor_clouds_to_ground_truth, SIGNAL(triggered()), this, SLOT(anchorCloudsToGroundTruth()));
	connect(_ui->menuEdit, SIGNAL(aboutToShow()), this, SLOT(updateEditMenu()));
	connect(_ui->actionDefault_views, SIGNAL(triggered(bool)), this, SLOT(setDefaultViews()));
	connect(_ui->actionAuto_screen_capture, SIGNAL(triggered(bool)), this, SLOT(selectScreenCaptureFormat(bool)));
	connect(_ui->actionScreenshot, SIGNAL(triggered()), this, SLOT(takeScreenshot()));
	connect(_ui->action16_9, SIGNAL(triggered()), this, SLOT(setAspectRatio16_9()));
	connect(_ui->action16_10, SIGNAL(triggered()), this, SLOT(setAspectRatio16_10()));
	connect(_ui->action4_3, SIGNAL(triggered()), this, SLOT(setAspectRatio4_3()));
	connect(_ui->action240p, SIGNAL(triggered()), this, SLOT(setAspectRatio240p()));
	connect(_ui->action360p, SIGNAL(triggered()), this, SLOT(setAspectRatio360p()));
	connect(_ui->action480p, SIGNAL(triggered()), this, SLOT(setAspectRatio480p()));
	connect(_ui->action720p, SIGNAL(triggered()), this, SLOT(setAspectRatio720p()));
	connect(_ui->action1080p, SIGNAL(triggered()), this, SLOT(setAspectRatio1080p()));
	connect(_ui->actionCustom, SIGNAL(triggered()), this, SLOT(setAspectRatioCustom()));
	connect(_ui->actionSave_point_cloud, SIGNAL(triggered()), this, SLOT(exportClouds()));
	connect(_ui->actionExport_2D_Grid_map_bmp_png, SIGNAL(triggered()), this, SLOT(exportGridMap()));
	connect(_ui->actionExport_images_RGB_jpg_Depth_png, SIGNAL(triggered()), this , SLOT(exportImages()));
	connect(_ui->actionExport_cameras_in_Bundle_format_out, SIGNAL(triggered()), SLOT(exportBundlerFormat()));
	connect(_ui->actionExport_octomap, SIGNAL(triggered()), this, SLOT(exportOctomap()));
	connect(_ui->actionView_high_res_point_cloud, SIGNAL(triggered()), this, SLOT(viewClouds()));
	connect(_ui->actionReset_Odometry, SIGNAL(triggered()), this, SLOT(resetOdometry()));
	connect(_ui->actionTrigger_a_new_map, SIGNAL(triggered()), this, SLOT(triggerNewMap()));
	connect(_ui->actionData_recorder, SIGNAL(triggered()), this, SLOT(dataRecorder()));
	connect(_ui->actionPost_processing, SIGNAL(triggered()), this, SLOT(showPostProcessingDialog()));
	connect(_ui->actionDepth_Calibration, SIGNAL(triggered()), this, SLOT(depthCalibration()));

	_ui->actionPause->setShortcut(Qt::Key_Space);
	_ui->actionSave_GUI_config->setShortcut(QKeySequence::Save);
	// Qt5 issue, we should explicitly add actions not in
	// menu bar to have shortcut working
	this->addAction(_ui->actionSave_GUI_config);
	_ui->actionReset_Odometry->setEnabled(false);
	_ui->actionPost_processing->setEnabled(false);
	_ui->actionAnchor_clouds_to_ground_truth->setEnabled(false);

	QToolButton* toolButton = new QToolButton(this);
	toolButton->setMenu(_ui->menuSelect_source);
	toolButton->setPopupMode(QToolButton::InstantPopup);
	toolButton->setIcon(QIcon(":images/kinect_xbox_360.png"));
	toolButton->setToolTip("Select sensor driver");
	_ui->toolBar->addWidget(toolButton)->setObjectName("toolbar_source");

#if defined(Q_WS_MAC) || defined(Q_WS_WIN)
	connect(_ui->actionOpen_working_directory, SIGNAL(triggered()), SLOT(openWorkingDirectory()));
#else
	_ui->menuEdit->removeAction(_ui->actionOpen_working_directory);
#endif

	//Settings menu
	connect(_ui->actionMore_options, SIGNAL(triggered()), this, SLOT(openPreferencesSource()));
	connect(_ui->actionUsbCamera, SIGNAL(triggered()), this, SLOT(selectStream()));
	connect(_ui->actionOpenNI_PCL, SIGNAL(triggered()), this, SLOT(selectOpenni()));
	connect(_ui->actionOpenNI_PCL_ASUS, SIGNAL(triggered()), this, SLOT(selectOpenni()));
	connect(_ui->actionFreenect, SIGNAL(triggered()), this, SLOT(selectFreenect()));
	connect(_ui->actionOpenNI_CV, SIGNAL(triggered()), this, SLOT(selectOpenniCv()));
	connect(_ui->actionOpenNI_CV_ASUS, SIGNAL(triggered()), this, SLOT(selectOpenniCvAsus()));
	connect(_ui->actionOpenNI2, SIGNAL(triggered()), this, SLOT(selectOpenni2()));
	connect(_ui->actionOpenNI2_kinect, SIGNAL(triggered()), this, SLOT(selectOpenni2()));
	connect(_ui->actionOpenNI2_orbbec, SIGNAL(triggered()), this, SLOT(selectOpenni2()));
	connect(_ui->actionOpenNI2_sense, SIGNAL(triggered()), this, SLOT(selectOpenni2()));
	connect(_ui->actionFreenect2, SIGNAL(triggered()), this, SLOT(selectFreenect2()));
	connect(_ui->actionKinect_for_Windows_SDK_v2, SIGNAL(triggered()), this, SLOT(selectK4W2()));
	connect(_ui->actionKinect_for_Azure, SIGNAL(triggered()), this, SLOT(selectK4A()));
	connect(_ui->actionRealSense_R200, SIGNAL(triggered()), this, SLOT(selectRealSense()));
	connect(_ui->actionRealSense_ZR300, SIGNAL(triggered()), this, SLOT(selectRealSense()));
	connect(_ui->actionRealSense2_SR300, SIGNAL(triggered()), this, SLOT(selectRealSense2()));
	connect(_ui->actionRealSense2_D400, SIGNAL(triggered()), this, SLOT(selectRealSense2()));
	connect(_ui->actionRealSense2_L515, SIGNAL(triggered()), this, SLOT(selectRealSense2L515()));
	connect(_ui->actionStereoDC1394, SIGNAL(triggered()), this, SLOT(selectStereoDC1394()));
	connect(_ui->actionStereoFlyCapture2, SIGNAL(triggered()), this, SLOT(selectStereoFlyCapture2()));
	connect(_ui->actionStereoZed, SIGNAL(triggered()), this, SLOT(selectStereoZed()));
	connect(_ui->actionZed_Open_Capture, SIGNAL(triggered()), this, SLOT(selectStereoZedOC()));
    connect(_ui->actionStereoTara, SIGNAL(triggered()), this, SLOT(selectStereoTara()));
	connect(_ui->actionStereoUsb, SIGNAL(triggered()), this, SLOT(selectStereoUsb()));
	connect(_ui->actionRealSense2_T265, SIGNAL(triggered()), this, SLOT(selectRealSense2Stereo()));
	connect(_ui->actionMYNT_EYE_S_SDK, SIGNAL(triggered()), this, SLOT(selectMyntEyeS()));
	connect(_ui->actionDepthAI, SIGNAL(triggered()), this, SLOT(selectDepthAI()));
	_ui->actionFreenect->setEnabled(CameraFreenect::available());
	_ui->actionOpenNI_CV->setEnabled(CameraOpenNICV::available());
	_ui->actionOpenNI_CV_ASUS->setEnabled(CameraOpenNICV::available());
	_ui->actionOpenNI2->setEnabled(CameraOpenNI2::available());
	_ui->actionOpenNI2_kinect->setEnabled(CameraOpenNI2::available());
	_ui->actionOpenNI2_orbbec->setEnabled(CameraOpenNI2::available());
	_ui->actionOpenNI2_sense->setEnabled(CameraOpenNI2::available());
	_ui->actionFreenect2->setEnabled(CameraFreenect2::available());
	_ui->actionKinect_for_Windows_SDK_v2->setEnabled(CameraK4W2::available());
	_ui->actionKinect_for_Azure->setEnabled(CameraK4A::available());
	_ui->actionRealSense_R200->setEnabled(CameraRealSense::available());
	_ui->actionRealSense_ZR300->setEnabled(CameraRealSense::available());
	_ui->actionRealSense2_SR300->setEnabled(CameraRealSense2::available());
	_ui->actionRealSense2_D400->setEnabled(CameraRealSense2::available());
	_ui->actionRealSense2_L515->setEnabled(CameraRealSense2::available());
	_ui->actionRealSense2_T265->setEnabled(CameraRealSense2::available());
	_ui->actionStereoDC1394->setEnabled(CameraStereoDC1394::available());
	_ui->actionStereoFlyCapture2->setEnabled(CameraStereoFlyCapture2::available());
	_ui->actionStereoZed->setEnabled(CameraStereoZed::available());
	_ui->actionZed_Open_Capture->setEnabled(CameraStereoZedOC::available());
    _ui->actionStereoTara->setEnabled(CameraStereoTara::available());
    _ui->actionMYNT_EYE_S_SDK->setEnabled(CameraMyntEye::available());
    _ui->actionDepthAI->setEnabled(CameraDepthAI::available());
	this->updateSelectSourceMenu();

	connect(_ui->actionPreferences, SIGNAL(triggered()), this, SLOT(openPreferences()));

	QActionGroup * modeGrp = new QActionGroup(this);
	modeGrp->addAction(_ui->actionSLAM_mode);
	modeGrp->addAction(_ui->actionLocalization_mode);
	_ui->actionSLAM_mode->setChecked(_preferencesDialog->isSLAMMode());
	_ui->actionLocalization_mode->setChecked(!_preferencesDialog->isSLAMMode());
	connect(_ui->actionSLAM_mode, SIGNAL(triggered()), this, SLOT(changeMappingMode()));
	connect(_ui->actionLocalization_mode, SIGNAL(triggered()), this, SLOT(changeMappingMode()));
	connect(this, SIGNAL(mappingModeChanged(bool)), _preferencesDialog, SLOT(setSLAMMode(bool)));

	// Settings changed
	qRegisterMetaType<PreferencesDialog::PANEL_FLAGS>("PreferencesDialog::PANEL_FLAGS");
	connect(_preferencesDialog, SIGNAL(settingsChanged(PreferencesDialog::PANEL_FLAGS)), this, SLOT(applyPrefSettings(PreferencesDialog::PANEL_FLAGS)));
	qRegisterMetaType<rtabmap::ParametersMap>("rtabmap::ParametersMap");
	connect(_preferencesDialog, SIGNAL(settingsChanged(rtabmap::ParametersMap)), this, SLOT(applyPrefSettings(rtabmap::ParametersMap)));
	// config GUI modified
	connect(_preferencesDialog, SIGNAL(settingsChanged(PreferencesDialog::PANEL_FLAGS)), this, SLOT(configGUIModified()));
	if(prefDialog == 0)
	{
		connect(_preferencesDialog, SIGNAL(settingsChanged(rtabmap::ParametersMap)), this, SLOT(configGUIModified()));
	}
	connect(_ui->imageView_source, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_ui->imageView_loopClosure, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_ui->imageView_odometry, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_ui->graphicsView_graphView, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_cloudViewer, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_exportCloudsDialog, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_exportBundlerDialog, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_postProcessingDialog, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_depthCalibrationDialog, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_multiSessionLocWidget->getImageView(), SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_ui->toolBar->toggleViewAction(), SIGNAL(toggled(bool)), this, SLOT(configGUIModified()));
	connect(_ui->toolBar, SIGNAL(orientationChanged(Qt::Orientation)), this, SLOT(configGUIModified()));
	connect(statusBarAction, SIGNAL(toggled(bool)), this, SLOT(configGUIModified()));
	QList<QDockWidget*> dockWidgets = this->findChildren<QDockWidget*>();
	for(int i=0; i<dockWidgets.size(); ++i)
	{
		connect(dockWidgets[i], SIGNAL(dockLocationChanged(Qt::DockWidgetArea)), this, SLOT(configGUIModified()));
		connect(dockWidgets[i]->toggleViewAction(), SIGNAL(toggled(bool)), this, SLOT(configGUIModified()));
	}
	connect(_ui->dockWidget_graphViewer->toggleViewAction(), SIGNAL(triggered()), this, SLOT(updateGraphView()));
	// catch resize events
	_ui->dockWidget_posterior->installEventFilter(this);
	_ui->dockWidget_likelihood->installEventFilter(this);
	_ui->dockWidget_rawlikelihood->installEventFilter(this);
	_ui->dockWidget_statsV2->installEventFilter(this);
	_ui->dockWidget_console->installEventFilter(this);
	_ui->dockWidget_loopClosureViewer->installEventFilter(this);
	_ui->dockWidget_mapVisibility->installEventFilter(this);
	_ui->dockWidget_graphViewer->installEventFilter(this);
	_ui->dockWidget_odometry->installEventFilter(this);
	_ui->dockWidget_cloudViewer->installEventFilter(this);
	_ui->dockWidget_imageView->installEventFilter(this);
	_ui->dockWidget_multiSessionLoc->installEventFilter(this);

	// more connects...
	_ui->doubleSpinBox_stats_imgRate->setValue(_preferencesDialog->getGeneralInputRate());
	_ui->doubleSpinBox_stats_detectionRate->setValue(_preferencesDialog->getDetectionRate());
	_ui->doubleSpinBox_stats_timeLimit->setValue(_preferencesDialog->getTimeLimit());
	connect(_ui->doubleSpinBox_stats_imgRate, SIGNAL(editingFinished()), this, SLOT(changeImgRateSetting()));
	connect(_ui->doubleSpinBox_stats_detectionRate, SIGNAL(editingFinished()), this, SLOT(changeDetectionRateSetting()));
	connect(_ui->doubleSpinBox_stats_timeLimit, SIGNAL(editingFinished()), this, SLOT(changeTimeLimitSetting()));
	connect(this, SIGNAL(imgRateChanged(double)), _preferencesDialog, SLOT(setInputRate(double)));
	connect(this, SIGNAL(detectionRateChanged(double)), _preferencesDialog, SLOT(setDetectionRate(double)));
	connect(this, SIGNAL(timeLimitChanged(float)), _preferencesDialog, SLOT(setTimeLimit(float)));

	// Statistics from the detector
	qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");
	connect(this, SIGNAL(statsReceived(rtabmap::Statistics)), this, SLOT(processStats(rtabmap::Statistics)));

	qRegisterMetaType<rtabmap::CameraInfo>("rtabmap::CameraInfo");
	connect(this, SIGNAL(cameraInfoReceived(rtabmap::CameraInfo)), this, SLOT(processCameraInfo(rtabmap::CameraInfo)));

	qRegisterMetaType<rtabmap::OdometryEvent>("rtabmap::OdometryEvent");
	connect(this, SIGNAL(odometryReceived(rtabmap::OdometryEvent, bool)), this, SLOT(processOdometry(rtabmap::OdometryEvent, bool)));

	connect(this, SIGNAL(noMoreImagesReceived()), this, SLOT(notifyNoMoreImages()));

	// Apply state
	this->changeState(kIdle);
	this->applyPrefSettings(PreferencesDialog::kPanelAll);

	_ui->statsToolBox->setNewFigureMaxItems(50);
	_ui->statsToolBox->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_ui->graphicsView_graphView->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_exportBundlerDialog->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_cloudViewer->setBackfaceCulling(true, false);
	_preferencesDialog->loadWidgetState(_cloudViewer);
	_preferencesDialog->loadWidgetState(_multiSessionLocWidget->getImageView());

	//dialog states
	_preferencesDialog->loadWidgetState(_exportCloudsDialog);
	_preferencesDialog->loadWidgetState(_exportBundlerDialog);
	_preferencesDialog->loadWidgetState(_postProcessingDialog);
	_preferencesDialog->loadWidgetState(_depthCalibrationDialog);

	if(_ui->statsToolBox->findChildren<StatItem*>().size() == 0)
	{
		const std::map<std::string, float> & statistics = Statistics::defaultData();
		for(std::map<std::string, float>::const_iterator iter = statistics.begin(); iter != statistics.end(); ++iter)
		{
			// Don't add Gt panels yet if we don't know if we will receive Gt values.
			if(!QString((*iter).first.c_str()).contains("Gt/"))
			{
				_ui->statsToolBox->updateStat(QString((*iter).first.c_str()).replace('_', ' '), false);
			}
		}
	}
	// Specific MainWindow
	_ui->statsToolBox->updateStat("Planning/From/", false);
	_ui->statsToolBox->updateStat("Planning/Time/ms", false);
	_ui->statsToolBox->updateStat("Planning/Goal/", false);
	_ui->statsToolBox->updateStat("Planning/Poses/", false);
	_ui->statsToolBox->updateStat("Planning/Length/m", false);

	_ui->statsToolBox->updateStat("Camera/Time capturing/ms", false);
	_ui->statsToolBox->updateStat("Camera/Time decimation/ms", false);
	_ui->statsToolBox->updateStat("Camera/Time disparity/ms", false);
	_ui->statsToolBox->updateStat("Camera/Time mirroring/ms", false);
	_ui->statsToolBox->updateStat("Camera/Time scan from depth/ms", false);

	_ui->statsToolBox->updateStat("Odometry/ID/", false);
	_ui->statsToolBox->updateStat("Odometry/Features/", false);
	_ui->statsToolBox->updateStat("Odometry/Matches/", false);
	_ui->statsToolBox->updateStat("Odometry/MatchesRatio/", false);
	_ui->statsToolBox->updateStat("Odometry/Inliers/", false);
	_ui->statsToolBox->updateStat("Odometry/InliersMeanDistance/m", false);
	_ui->statsToolBox->updateStat("Odometry/InliersDistribution/", false);
	_ui->statsToolBox->updateStat("Odometry/InliersRatio/", false);
	_ui->statsToolBox->updateStat("Odometry/ICPInliersRatio/", false);
	_ui->statsToolBox->updateStat("Odometry/ICPRotation/rad", false);
	_ui->statsToolBox->updateStat("Odometry/ICPTranslation/m", false);
	_ui->statsToolBox->updateStat("Odometry/ICPStructuralComplexity/", false);
	_ui->statsToolBox->updateStat("Odometry/ICPStructuralDistribution/", false);
	_ui->statsToolBox->updateStat("Odometry/ICPCorrespondences/", false);
	_ui->statsToolBox->updateStat("Odometry/ICPRMS/", false);
	_ui->statsToolBox->updateStat("Odometry/StdDevLin/m", false);
	_ui->statsToolBox->updateStat("Odometry/StdDevAng/rad", false);
	_ui->statsToolBox->updateStat("Odometry/VarianceLin/", false);
	_ui->statsToolBox->updateStat("Odometry/VarianceAng/", false);
	_ui->statsToolBox->updateStat("Odometry/TimeEstimation/ms", false);
	_ui->statsToolBox->updateStat("Odometry/TimeFiltering/ms", false);
	_ui->statsToolBox->updateStat("Odometry/GravityRollError/deg", false);
	_ui->statsToolBox->updateStat("Odometry/GravityPitchError/deg", false);
	_ui->statsToolBox->updateStat("Odometry/LocalMapSize/", false);
	_ui->statsToolBox->updateStat("Odometry/LocalScanMapSize/", false);
	_ui->statsToolBox->updateStat("Odometry/LocalKeyFrames/", false);
	_ui->statsToolBox->updateStat("Odometry/localBundleOutliers/", false);
	_ui->statsToolBox->updateStat("Odometry/localBundleConstraints/", false);
	_ui->statsToolBox->updateStat("Odometry/localBundleTime/ms", false);
	_ui->statsToolBox->updateStat("Odometry/KeyFrameAdded/", false);
	_ui->statsToolBox->updateStat("Odometry/Interval/ms", false);
	_ui->statsToolBox->updateStat("Odometry/Speed/kph", false);
	_ui->statsToolBox->updateStat("Odometry/Speed/mph", false);
	_ui->statsToolBox->updateStat("Odometry/Speed/mps", false);
	_ui->statsToolBox->updateStat("Odometry/SpeedGuess/kph", false);
	_ui->statsToolBox->updateStat("Odometry/SpeedGuess/mph", false);
	_ui->statsToolBox->updateStat("Odometry/SpeedGuess/mps", false);
	_ui->statsToolBox->updateStat("Odometry/Distance/m", false);
	_ui->statsToolBox->updateStat("Odometry/T/m", false);
	_ui->statsToolBox->updateStat("Odometry/Tx/m", false);
	_ui->statsToolBox->updateStat("Odometry/Ty/m", false);
	_ui->statsToolBox->updateStat("Odometry/Tz/m", false);
	_ui->statsToolBox->updateStat("Odometry/Troll/deg", false);
	_ui->statsToolBox->updateStat("Odometry/Tpitch/deg", false);
	_ui->statsToolBox->updateStat("Odometry/Tyaw/deg", false);
	_ui->statsToolBox->updateStat("Odometry/Px/m", false);
	_ui->statsToolBox->updateStat("Odometry/Py/m", false);
	_ui->statsToolBox->updateStat("Odometry/Pz/m", false);
	_ui->statsToolBox->updateStat("Odometry/Proll/deg", false);
	_ui->statsToolBox->updateStat("Odometry/Ppitch/deg", false);
	_ui->statsToolBox->updateStat("Odometry/Pyaw/deg", false);

	_ui->statsToolBox->updateStat("GUI/Refresh odom/ms", false);
	_ui->statsToolBox->updateStat("GUI/RGB-D cloud/ms", false);
	_ui->statsToolBox->updateStat("GUI/Graph Update/ms", false);
#ifdef RTABMAP_OCTOMAP
	_ui->statsToolBox->updateStat("GUI/Octomap Update/ms", false);
	_ui->statsToolBox->updateStat("GUI/Octomap Rendering/ms", false);
#endif
	_ui->statsToolBox->updateStat("GUI/Grid Update/ms", false);
	_ui->statsToolBox->updateStat("GUI/Grid Rendering/ms", false);
	_ui->statsToolBox->updateStat("GUI/Refresh stats/ms", false);
	_ui->statsToolBox->updateStat("GUI/Cache Data Size/MB", false);
	_ui->statsToolBox->updateStat("GUI/Cache Clouds Size/MB", false);
#ifdef RTABMAP_OCTOMAP
	_ui->statsToolBox->updateStat("GUI/Octomap Size/MB", false);
#endif

	this->loadFigures();
	connect(_ui->statsToolBox, SIGNAL(figuresSetupChanged()), this, SLOT(configGUIModified()));

	// update loop closure viewer parameters
	_loopClosureViewer->setDecimation(_preferencesDialog->getCloudDecimation(0));
	_loopClosureViewer->setMaxDepth(_preferencesDialog->getCloudMaxDepth(0));

	if (splash)
	{
		splash->close();
		delete splash;
	}

	this->setFocus();

	UDEBUG("");
}

MainWindow::~MainWindow()
{
	UDEBUG("");
	this->stopDetection();
	delete _ui;
	delete _elapsedTime;
#ifdef RTABMAP_OCTOMAP
	delete _octomap;
#endif
	delete _occupancyGrid;
	UDEBUG("");
}

void MainWindow::setupMainLayout(bool vertical)
{
	if(vertical)
	{
		qobject_cast<QHBoxLayout *>(_ui->layout_imageview->layout())->setDirection(QBoxLayout::TopToBottom);
	}
	else if(!vertical)
	{
		qobject_cast<QHBoxLayout *>(_ui->layout_imageview->layout())->setDirection(QBoxLayout::LeftToRight);
	}
}

std::map<int, Transform> MainWindow::currentVisiblePosesMap() const
{
	return _ui->widget_mapVisibility->getVisiblePoses();
}

void MainWindow::setCloudViewer(rtabmap::CloudViewer * cloudViewer)
{
	UASSERT(cloudViewer);
	delete _cloudViewer;
	_cloudViewer = cloudViewer;
	_cloudViewer->setParent(_ui->layout_cloudViewer);
	_cloudViewer->setObjectName("widget_cloudViewer");
	_ui->layout_cloudViewer->layout()->addWidget(_cloudViewer);

	_cloudViewer->setBackfaceCulling(true, false);
	_preferencesDialog->loadWidgetState(_cloudViewer);

	connect(_cloudViewer, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
}
void MainWindow::setLoopClosureViewer(rtabmap::LoopClosureViewer * loopClosureViewer)
{
	UASSERT(loopClosureViewer);
	delete _loopClosureViewer;
	_loopClosureViewer = loopClosureViewer;
	_loopClosureViewer->setParent(_ui->layout_loopClosureViewer);
	_loopClosureViewer->setObjectName("widget_loopClosureViewer");
	_ui->layout_loopClosureViewer->layout()->addWidget(_loopClosureViewer);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
	// Try to close all children
	/*QList<QMainWindow *> windows = this->findChildren<QMainWindow *>();
	for(int i=0; i<windows.size(); i++) {
		if(!windows[i]->close()) {
			event->setAccepted(false);
			return;
		}
	}*/
	UDEBUG("");
	bool processStopped = true;
	if(_state != kIdle && _state != kMonitoring && _state != kMonitoringPaused)
	{
		this->stopDetection();
		if(_state == kInitialized)
		{
			if(this->closeDatabase())
			{
				this->changeState(kApplicationClosing);
			}
		}
		if(_state != kIdle)
		{
			processStopped = false;
		}
	}

	if(processStopped)
	{
		//write settings before quit?
		bool save = false;
		if(this->isWindowModified())
		{
			QMessageBox::Button b=QMessageBox::question(this,
					tr("RTAB-Map"),
					tr("There are unsaved changed settings. Save them?"),
					QMessageBox::Save | QMessageBox::Cancel | QMessageBox::Discard);
			if(b == QMessageBox::Save)
			{
				save = true;
			}
			else if(b != QMessageBox::Discard)
			{
				event->ignore();
				return;
			}
		}

		if(save)
		{
			saveConfigGUI();
		}

		_ui->statsToolBox->closeFigures();

		_ui->dockWidget_imageView->close();
		_ui->dockWidget_likelihood->close();
		_ui->dockWidget_rawlikelihood->close();
		_ui->dockWidget_posterior->close();
		_ui->dockWidget_statsV2->close();
		_ui->dockWidget_console->close();
		_ui->dockWidget_cloudViewer->close();
		_ui->dockWidget_loopClosureViewer->close();
		_ui->dockWidget_mapVisibility->close();
		_ui->dockWidget_graphViewer->close();
		_ui->dockWidget_odometry->close();
		_ui->dockWidget_multiSessionLoc->close();

		if(_camera)
		{
			UERROR("Camera must be already deleted here!");
			delete _camera;
			_camera = 0;
			if(_imuThread)
			{
				delete _imuThread;
				_imuThread = 0;
			}
		}
		if(_odomThread)
		{
			UERROR("OdomThread must be already deleted here!");
			delete _odomThread;
			_odomThread = 0;
		}
		event->accept();
	}
	else
	{
		event->ignore();
	}
	UDEBUG("");
}

bool MainWindow::handleEvent(UEvent* anEvent)
{
	if(anEvent->getClassName().compare("IMUEvent") == 0)
	{
		// IMU events are published at high frequency, early exit
		return false;
	}
	else if(anEvent->getClassName().compare("RtabmapEvent") == 0)
	{
		RtabmapEvent * rtabmapEvent = (RtabmapEvent*)anEvent;
		Statistics stats = rtabmapEvent->getStats();
		int highestHypothesisId = int(uValue(stats.data(), Statistics::kLoopHighest_hypothesis_id(), 0.0f));
		int proximityClosureId = int(uValue(stats.data(), Statistics::kProximitySpace_last_detection_id(), 0.0f));
		bool rejectedHyp = bool(uValue(stats.data(), Statistics::kLoopRejectedHypothesis(), 0.0f));
		float highestHypothesisValue = uValue(stats.data(), Statistics::kLoopHighest_hypothesis_value(), 0.0f);
		if((stats.loopClosureId() > 0 &&
			_ui->actionPause_on_match->isChecked())
		   ||
		   (stats.loopClosureId() == 0 &&
		    highestHypothesisId > 0 &&
		    highestHypothesisValue >= _preferencesDialog->getLoopThr() &&
		    _ui->actionPause_when_a_loop_hypothesis_is_rejected->isChecked() &&
		    rejectedHyp)
		   ||
		   (proximityClosureId > 0 &&
		    _ui->actionPause_on_local_loop_detection->isChecked()))
		{
			if(_state != kPaused && _state != kMonitoringPaused && !_processingDownloadedMap)
			{
				if(_preferencesDialog->beepOnPause())
				{
					QMetaObject::invokeMethod(this, "beep");
				}
				this->pauseDetection();
			}
		}

		if(!_processingDownloadedMap)
		{
			_processingStatistics = true;
			Q_EMIT statsReceived(stats);
		}
	}
	else if(anEvent->getClassName().compare("RtabmapEventInit") == 0)
	{
		if(!_recovering)
		{
			RtabmapEventInit * rtabmapEventInit = (RtabmapEventInit*)anEvent;
			Q_EMIT rtabmapEventInitReceived((int)rtabmapEventInit->getStatus(), rtabmapEventInit->getInfo().c_str());
		}
	}
	else if(anEvent->getClassName().compare("RtabmapEvent3DMap") == 0)
	{
		RtabmapEvent3DMap * rtabmapEvent3DMap = (RtabmapEvent3DMap*)anEvent;
		Q_EMIT rtabmapEvent3DMapReceived(*rtabmapEvent3DMap);
	}
	else if(anEvent->getClassName().compare("RtabmapGlobalPathEvent") == 0)
	{
		RtabmapGlobalPathEvent * rtabmapGlobalPathEvent = (RtabmapGlobalPathEvent*)anEvent;
		Q_EMIT rtabmapGlobalPathEventReceived(*rtabmapGlobalPathEvent);
	}
	else if(anEvent->getClassName().compare("RtabmapLabelErrorEvent") == 0)
	{
		RtabmapLabelErrorEvent * rtabmapLabelErrorEvent = (RtabmapLabelErrorEvent*)anEvent;
		Q_EMIT rtabmapLabelErrorReceived(rtabmapLabelErrorEvent->id(), QString(rtabmapLabelErrorEvent->label().c_str()));
	}
	else if(anEvent->getClassName().compare("RtabmapGoalStatusEvent") == 0)
	{
		Q_EMIT rtabmapGoalStatusEventReceived(anEvent->getCode());
	}
	else if(anEvent->getClassName().compare("CameraEvent") == 0)
	{
		CameraEvent * cameraEvent = (CameraEvent*)anEvent;
		if(cameraEvent->getCode() == CameraEvent::kCodeNoMoreImages)
		{
			if(_preferencesDialog->beepOnPause())
			{
				QMetaObject::invokeMethod(this, "beep");
			}
			Q_EMIT noMoreImagesReceived();
		}
		else
		{
			Q_EMIT cameraInfoReceived(cameraEvent->info());
			if (_odomThread == 0 && (_camera->odomProvided()) && _preferencesDialog->isRGBDMode())
			{
				OdometryInfo odomInfo;
				odomInfo.reg.covariance = cameraEvent->info().odomCovariance;
				if (!_processingOdometry && !_processingStatistics)
				{
					_processingOdometry = true; // if we receive too many odometry events!
					OdometryEvent tmp(cameraEvent->data(), cameraEvent->info().odomPose, odomInfo);
					Q_EMIT odometryReceived(tmp, false);
				}
				else
				{
					// we receive too many odometry events! ignore them
				}
			}
		}
	}
	else if(anEvent->getClassName().compare("OdometryEvent") == 0)
	{
		OdometryEvent * odomEvent = (OdometryEvent*)anEvent;
		if(!_processingOdometry && !_processingStatistics)
		{
			_processingOdometry = true; // if we receive too many odometry events!
			Q_EMIT odometryReceived(*odomEvent, false);
		}
		else
		{
			// we receive too many odometry events! just send without data
			SensorData data(cv::Mat(), odomEvent->data().id(), odomEvent->data().stamp());
			data.setCameraModels(odomEvent->data().cameraModels());
			data.setStereoCameraModels(odomEvent->data().stereoCameraModels());
			data.setGroundTruth(odomEvent->data().groundTruth());
			OdometryEvent tmp(data, odomEvent->pose(), odomEvent->info().copyWithoutData());
			Q_EMIT odometryReceived(tmp, true);
		}
	}
	else if(anEvent->getClassName().compare("ULogEvent") == 0)
	{
		ULogEvent * logEvent = (ULogEvent*)anEvent;
		if(logEvent->getCode() >= _preferencesDialog->getGeneralLoggerPauseLevel())
		{
			QMetaObject::invokeMethod(_ui->dockWidget_console, "show");
			// The timer prevents multiple calls to pauseDetection() before the state can be changed
			if(_state != kPaused && _state != kMonitoringPaused && _state != kMonitoring && _logEventTime->elapsed() > 1000)
			{
				_logEventTime->start();
				if(_preferencesDialog->beepOnPause())
				{
					QMetaObject::invokeMethod(this, "beep");
				}
				pauseDetection();
			}
		}
	}
	return false;
}

void MainWindow::processCameraInfo(const rtabmap::CameraInfo & info)
{
	if(_firstStamp == 0.0)
	{
		_firstStamp = info.stamp;
	}
	if(_preferencesDialog->isCacheSavedInFigures() || _ui->statsToolBox->isVisible())
	{
		_ui->statsToolBox->updateStat("Camera/Time total/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeTotal*1000.0f, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Camera/Time capturing/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeCapture*1000.0f, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Camera/Time undistort depth/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeUndistortDepth*1000.0f, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Camera/Time bilateral filtering/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeBilateralFiltering*1000.0f, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Camera/Time decimation/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeImageDecimation*1000.0f, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Camera/Time disparity/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeDisparity*1000.0f, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Camera/Time mirroring/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeMirroring*1000.0f, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Camera/Time exposure compensation/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeStereoExposureCompensation*1000.0f, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Camera/Time scan from depth/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeScanFromDepth*1000.0f, _preferencesDialog->isCacheSavedInFigures());
	}

	Q_EMIT(cameraInfoProcessed());
}

void MainWindow::processOdometry(const rtabmap::OdometryEvent & odom, bool dataIgnored)
{
	if(_firstStamp == 0.0)
	{
		_firstStamp = odom.data().stamp();
	}

	UDEBUG("");
	_processingOdometry = true;
	UTimer time;
	// Process Data

	// Set color code as tooltip
	if(_ui->imageView_odometry->toolTip().isEmpty())
	{
		_ui->imageView_odometry->setToolTip(
			"Background Color Code:\n"
			"  Dark Red = Odometry Lost\n"
			"  Dark Yellow = Low Inliers\n"
			"Feature Color code:\n"
			"  Green = Inliers\n"
			"  Yellow = Not matched features from previous frame(s)\n"
			"  Red = Outliers");
	}

	Transform pose = odom.pose();
	bool lost = false;
	bool lostStateChanged = false;

	if(pose.isNull())
	{
		UDEBUG("odom lost"); // use last pose
		lostStateChanged = _cloudViewer->getBackgroundColor() != Qt::darkRed;
		_cloudViewer->setBackgroundColor(Qt::darkRed);
		_ui->imageView_odometry->setBackgroundColor(Qt::darkRed);

		pose = _lastOdomPose;
		lost = true;
	}
	else if(odom.info().reg.inliers>0 &&
			_preferencesDialog->getOdomQualityWarnThr() &&
			odom.info().reg.inliers < _preferencesDialog->getOdomQualityWarnThr())
	{
		UDEBUG("odom warn, quality(inliers)=%d thr=%d", odom.info().reg.inliers, _preferencesDialog->getOdomQualityWarnThr());
		lostStateChanged = _cloudViewer->getBackgroundColor() == Qt::darkRed;
		_cloudViewer->setBackgroundColor(Qt::darkYellow);
		_ui->imageView_odometry->setBackgroundColor(Qt::darkYellow);
	}
	else
	{
		UDEBUG("odom ok");
		lostStateChanged = _cloudViewer->getBackgroundColor() == Qt::darkRed;
		_cloudViewer->setBackgroundColor(_cloudViewer->getDefaultBackgroundColor());
		_ui->imageView_odometry->setBackgroundColor(_ui->imageView_odometry->getDefaultBackgroundColor());
	}

	if(!pose.isNull() && (_ui->dockWidget_cloudViewer->isVisible() || _ui->graphicsView_graphView->isVisible()))
	{
		_lastOdomPose = pose;
	}

	const SensorData * data = &odom.data();

	SensorData rectifiedData;
	if(!data->imageRaw().empty() &&
		((_ui->dockWidget_cloudViewer->isVisible() &&  _preferencesDialog->isCloudsShown(1)) ||
		  (_ui->dockWidget_odometry->isVisible() && (_ui->imageView_odometry->isImageShown() || _ui->imageView_odometry->isImageDepthShown()))))
	{
		// Do we need to rectify images?
		ParametersMap allParameters = _preferencesDialog->getAllParameters();
		bool imagesAlreadyRectified = Parameters::defaultRtabmapImagesAlreadyRectified();
		Parameters::parse(allParameters, Parameters::kRtabmapImagesAlreadyRectified(), imagesAlreadyRectified);
		if(!imagesAlreadyRectified)
		{
			rectifiedData = odom.data();
			if(data->cameraModels().size())
			{
				// Note that only RGB image is rectified, the depth image is assumed to be already registered to rectified RGB camera.
				UASSERT(int((data->imageRaw().cols/data->cameraModels().size())*data->cameraModels().size()) == data->imageRaw().cols);
				int subImageWidth = data->imageRaw().cols/data->cameraModels().size();
				cv::Mat rectifiedImages = data->imageRaw().clone();
				bool initRectMaps = _rectCameraModelsOdom.empty() || _rectCameraModelsOdom.size()!=data->cameraModels().size();
				if(initRectMaps)
				{
					_rectCameraModelsOdom.resize(data->cameraModels().size());
				}
				for(unsigned int i=0; i<data->cameraModels().size(); ++i)
				{
					if(data->cameraModels()[i].isValidForRectification())
					{
						if(initRectMaps)
						{
							_rectCameraModelsOdom[i] = data->cameraModels()[i];
							if(!_rectCameraModelsOdom[i].isRectificationMapInitialized())
							{
								UWARN("Initializing rectification maps for camera %d (only done for the first image received)...", i);
								_rectCameraModelsOdom[i].initRectificationMap();
								UWARN("Initializing rectification maps for camera %d (only done for the first image received)... done!", i);
							}
						}
						UASSERT(_rectCameraModelsOdom[i].imageWidth() == data->cameraModels()[i].imageWidth() &&
								_rectCameraModelsOdom[i].imageHeight() == data->cameraModels()[i].imageHeight());
						cv::Mat rectifiedImage = _rectCameraModelsOdom[i].rectifyImage(cv::Mat(data->imageRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, data->imageRaw().rows)));
						rectifiedImage.copyTo(cv::Mat(rectifiedImages, cv::Rect(subImageWidth*i, 0, subImageWidth, data->imageRaw().rows)));
					}
					else
					{
						UWARN("Camera %d of data %d is not valid for rectification (%dx%d).",
								i, data->id(),
								data->cameraModels()[i].imageWidth(),
								data->cameraModels()[i].imageHeight());
					}
				}
				rectifiedData.setRGBDImage(rectifiedImages, data->depthOrRightRaw(), data->cameraModels());
			}
			else if(!data->rightRaw().empty() && data->stereoCameraModels().size())
			{
				UASSERT(int((data->imageRaw().cols/data->stereoCameraModels().size())*data->stereoCameraModels().size()) == data->imageRaw().cols);
				int subImageWidth = data->imageRaw().cols/data->stereoCameraModels().size();
				cv::Mat rectifiedLeftImages = data->imageRaw().clone();
				cv::Mat rectifiedRightImages = data->imageRaw().clone();
				bool initRectMaps = _rectCameraModelsOdom.empty() || _rectCameraModelsOdom.size()!=data->stereoCameraModels().size()*2;
				if(initRectMaps)
				{
					_rectCameraModelsOdom.resize(data->stereoCameraModels().size()*2);
				}
				for(unsigned int i=0; i<data->stereoCameraModels().size(); ++i)
				{
					if(data->stereoCameraModels()[i].isValidForRectification())
					{
						if(initRectMaps)
						{
							_rectCameraModelsOdom[i*2] = data->stereoCameraModels()[i].left();
							_rectCameraModelsOdom[i*2+1] = data->stereoCameraModels()[i].right();
							if(!_rectCameraModelsOdom[i*2].isRectificationMapInitialized())
							{
								UWARN("Initializing rectification maps for stereo camera %d (only done for the first image received)...", i);
								_rectCameraModelsOdom[i*2].initRectificationMap();
								_rectCameraModelsOdom[i*2+1].initRectificationMap();
								UWARN("Initializing rectification maps for stereo camera %d (only done for the first image received)... done!", i);
							}
						}
						UASSERT(_rectCameraModelsOdom[i*2].imageWidth() == data->stereoCameraModels()[i].left().imageWidth() &&
								_rectCameraModelsOdom[i*2].imageHeight() == data->stereoCameraModels()[i].left().imageHeight() &&
								_rectCameraModelsOdom[i*2+1].imageWidth() == data->stereoCameraModels()[i].right().imageWidth() &&
								_rectCameraModelsOdom[i*2+1].imageHeight() == data->stereoCameraModels()[i].right().imageHeight());
						cv::Mat rectifiedLeftImage = _rectCameraModelsOdom[i*2].rectifyImage(cv::Mat(data->imageRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, data->imageRaw().rows)));
						cv::Mat rectifiedRightImage = _rectCameraModelsOdom[i*2+1].rectifyImage(cv::Mat(data->rightRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, data->rightRaw().rows)));
						rectifiedLeftImage.copyTo(cv::Mat(rectifiedLeftImages, cv::Rect(subImageWidth*i, 0, subImageWidth, data->imageRaw().rows)));
						rectifiedRightImage.copyTo(cv::Mat(rectifiedRightImages, cv::Rect(subImageWidth*i, 0, subImageWidth, data->rightRaw().rows)));
					}
					else
					{
						UWARN("Stereo camera %d of data %d is not valid for rectification (%dx%d).",
								i, data->id(),
								data->stereoCameraModels()[i].left().imageWidth(),
								data->stereoCameraModels()[i].right().imageHeight());
					}
				}
				rectifiedData.setStereoImage(rectifiedLeftImages, rectifiedRightImages, data->stereoCameraModels());
			}
			UDEBUG("Time rectification: %fs", time.ticks());
			data = &rectifiedData;
		}
	}

	if(_ui->dockWidget_cloudViewer->isVisible())
	{
		bool cloudUpdated = false;
		bool scanUpdated = false;
		bool featuresUpdated = false;
		bool filteredGravityUpdated = false;
		bool accelerationUpdated = false;
		if(!pose.isNull())
		{
			// 3d cloud
			if(!data->imageRaw().empty() &&
			   !data->depthOrRightRaw().empty() &&
			   (data->cameraModels().size() || data->stereoCameraModels().size()) &&
			   _preferencesDialog->isCloudsShown(1))
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
				pcl::IndicesPtr indices(new std::vector<int>);

				cloud = util3d::cloudRGBFromSensorData(*data,
						_preferencesDialog->getCloudDecimation(1),
						_preferencesDialog->getCloudMaxDepth(1),
						_preferencesDialog->getCloudMinDepth(1),
						indices.get(),
						_preferencesDialog->getAllParameters(),
						_preferencesDialog->getCloudRoiRatios(1));
				if(indices->size())
				{
					cloud = util3d::transformPointCloud(cloud, pose);

					if(_preferencesDialog->isCloudMeshing())
					{
						// we need to extract indices as pcl::OrganizedFastMesh doesn't take indices
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
						output = util3d::extractIndices(cloud, indices, false, true);

						// Fast organized mesh
						Eigen::Vector3f viewpoint(0.0f,0.0f,0.0f);
						if(data->cameraModels().size() && !data->cameraModels()[0].localTransform().isNull())
						{
							viewpoint[0] = data->cameraModels()[0].localTransform().x();
							viewpoint[1] = data->cameraModels()[0].localTransform().y();
							viewpoint[2] = data->cameraModels()[0].localTransform().z();
						}
						else if(data->stereoCameraModels().size() && !data->stereoCameraModels()[0].localTransform().isNull())
						{
							viewpoint[0] = data->stereoCameraModels()[0].localTransform().x();
							viewpoint[1] = data->stereoCameraModels()[0].localTransform().y();
							viewpoint[2] = data->stereoCameraModels()[0].localTransform().z();
						}
						std::vector<pcl::Vertices> polygons = util3d::organizedFastMesh(
								output,
								_preferencesDialog->getCloudMeshingAngle(),
								_preferencesDialog->isCloudMeshingQuad(),
								_preferencesDialog->getCloudMeshingTriangleSize(),
								Eigen::Vector3f(pose.x(), pose.y(), pose.z()) + viewpoint);
						if(polygons.size())
						{
							if(_preferencesDialog->isCloudMeshingTexture() && !data->imageRaw().empty())
							{
								pcl::TextureMesh::Ptr textureMesh(new pcl::TextureMesh);
								pcl::toPCLPointCloud2(*cloud, textureMesh->cloud);
								textureMesh->tex_polygons.push_back(polygons);
								int w = cloud->width;
								int h = cloud->height;
								UASSERT(w > 1 && h > 1);
								textureMesh->tex_coordinates.resize(1);
								int nPoints = (int)(textureMesh->cloud.data.size()/textureMesh->cloud.point_step);
								textureMesh->tex_coordinates[0].resize(nPoints);
								for(int i=0; i<nPoints; ++i)
								{
									//uv
									textureMesh->tex_coordinates[0][i] = Eigen::Vector2f(
											float(i % w) / float(w),      // u
											float(h - i / w) / float(h)); // v
								}

								pcl::TexMaterial mesh_material;
								mesh_material.tex_d = 1.0f;
								mesh_material.tex_Ns = 75.0f;
								mesh_material.tex_illum = 1;

								mesh_material.tex_name = "material_odom";
								mesh_material.tex_file = "";
								textureMesh->tex_materials.push_back(mesh_material);

								if(!_cloudViewer->addCloudTextureMesh("cloudOdom", textureMesh, data->imageRaw(), _odometryCorrection))
								{
									UERROR("Adding cloudOdom to viewer failed!");
								}
							}
							else if(!_cloudViewer->addCloudMesh("cloudOdom", output, polygons, _odometryCorrection))
							{
								UERROR("Adding cloudOdom to viewer failed!");
							}
						}
					}
					else
					{
						if(!_cloudViewer->addCloud("cloudOdom", cloud, _odometryCorrection))
						{
							UERROR("Adding cloudOdom to viewer failed!");
						}
					}
					_cloudViewer->setCloudVisibility("cloudOdom", true);
					_cloudViewer->setCloudColorIndex("cloudOdom", _preferencesDialog->getCloudColorScheme(1));
					_cloudViewer->setCloudOpacity("cloudOdom", _preferencesDialog->getCloudOpacity(1));
					_cloudViewer->setCloudPointSize("cloudOdom", _preferencesDialog->getCloudPointSize(1));

					cloudUpdated = true;
				}
			}

			if(_preferencesDialog->isScansShown(1))
			{
				// F2M: scan local map
				if(!odom.info().localScanMap.isEmpty())
				{
					if(!lost)
					{
						bool scanAlreadyThere = _cloudViewer->getAddedClouds().contains("scanMapOdom");
						bool scanAdded = false;
						if(odom.info().localScanMap.hasIntensity() && odom.info().localScanMap.hasNormals())
						{
							scanAdded = _cloudViewer->addCloud("scanMapOdom",
									util3d::laserScanToPointCloudINormal(odom.info().localScanMap, odom.info().localScanMap.localTransform()),
									_odometryCorrection, Qt::blue);
						}
						else if(odom.info().localScanMap.hasNormals())
						{
							scanAdded = _cloudViewer->addCloud("scanMapOdom",
									util3d::laserScanToPointCloudNormal(odom.info().localScanMap, odom.info().localScanMap.localTransform()),
									_odometryCorrection, Qt::blue);
						}
						else if(odom.info().localScanMap.hasIntensity())
						{
							scanAdded = _cloudViewer->addCloud("scanMapOdom",
									util3d::laserScanToPointCloudI(odom.info().localScanMap, odom.info().localScanMap.localTransform()),
									_odometryCorrection, Qt::blue);
						}
						else
						{
							scanAdded = _cloudViewer->addCloud("scanMapOdom",
									util3d::laserScanToPointCloud(odom.info().localScanMap, odom.info().localScanMap.localTransform()),
									_odometryCorrection, Qt::blue);
						}


						if(!scanAdded)
						{
							UERROR("Adding scanMapOdom to viewer failed!");
						}
						else
						{
							_cloudViewer->setCloudVisibility("scanMapOdom", true);
							_cloudViewer->setCloudColorIndex("scanMapOdom", scanAlreadyThere && _preferencesDialog->getScanColorScheme(1)==0 && odom.info().localScanMap.is2d()?2:_preferencesDialog->getScanColorScheme(1));
							_cloudViewer->setCloudOpacity("scanMapOdom", _preferencesDialog->getScanOpacity(1));
							_cloudViewer->setCloudPointSize("scanMapOdom", _preferencesDialog->getScanPointSize(1));
						}
					}
					scanUpdated = true;
				}
				// scan cloud
				if(!data->laserScanRaw().isEmpty())
				{
					LaserScan scan = data->laserScanRaw();

					if(_preferencesDialog->getDownsamplingStepScan(1) > 1 ||
						_preferencesDialog->getScanMaxRange(1) > 0.0f ||
						_preferencesDialog->getScanMinRange(1) > 0.0f)
					{
						scan = util3d::commonFiltering(scan,
								_preferencesDialog->getDownsamplingStepScan(1),
								_preferencesDialog->getScanMinRange(1),
								_preferencesDialog->getScanMaxRange(1));
					}
					bool scanAlreadyThere = _cloudViewer->getAddedClouds().contains("scanOdom");
					bool scanAdded = false;

					if(odom.info().localScanMap.hasIntensity() && odom.info().localScanMap.hasNormals())
					{
						pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud;
						cloud = util3d::laserScanToPointCloudINormal(scan, pose*scan.localTransform());
						if(_preferencesDialog->getCloudVoxelSizeScan(1) > 0.0)
						{
							cloud = util3d::voxelize(cloud, _preferencesDialog->getCloudVoxelSizeScan(1));
						}
						scanAdded = _cloudViewer->addCloud("scanOdom", cloud, _odometryCorrection, Qt::magenta);
					}
					else if(odom.info().localScanMap.hasNormals())
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
						cloud = util3d::laserScanToPointCloudNormal(scan, pose*scan.localTransform());
						if(_preferencesDialog->getCloudVoxelSizeScan(1) > 0.0)
						{
							cloud = util3d::voxelize(cloud, _preferencesDialog->getCloudVoxelSizeScan(1));
						}
						scanAdded = _cloudViewer->addCloud("scanOdom", cloud, _odometryCorrection, Qt::magenta);
					}
					else if(odom.info().localScanMap.hasIntensity())
					{
						pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
						cloud = util3d::laserScanToPointCloudI(scan, pose*scan.localTransform());
						if(_preferencesDialog->getCloudVoxelSizeScan(1) > 0.0)
						{
							cloud = util3d::voxelize(cloud, _preferencesDialog->getCloudVoxelSizeScan(1));
						}
						scanAdded = _cloudViewer->addCloud("scanOdom", cloud, _odometryCorrection, Qt::magenta);
					}
					else
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
						cloud = util3d::laserScanToPointCloud(scan, pose*scan.localTransform());
						if(_preferencesDialog->getCloudVoxelSizeScan(1) > 0.0)
						{
							cloud = util3d::voxelize(cloud, _preferencesDialog->getCloudVoxelSizeScan(1));
						}
						scanAdded = _cloudViewer->addCloud("scanOdom", cloud, _odometryCorrection, Qt::magenta);
					}

					if(!scanAdded)
					{
						UERROR("Adding scanOdom to viewer failed!");
					}
					else
					{
						_cloudViewer->setCloudVisibility("scanOdom", true);
						_cloudViewer->setCloudColorIndex("scanOdom", scanAlreadyThere && _preferencesDialog->getScanColorScheme(1)==0 && scan.is2d()?2:_preferencesDialog->getScanColorScheme(1));
						_cloudViewer->setCloudOpacity("scanOdom", _preferencesDialog->getScanOpacity(1));
						_cloudViewer->setCloudPointSize("scanOdom", _preferencesDialog->getScanPointSize(1));
						scanUpdated = true;
					}
				}
			}

			// 3d features
			if(_preferencesDialog->isFeaturesShown(1) && !odom.info().localMap.empty())
			{
				if(!lost)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					cloud->resize(odom.info().localMap.size());
					int i=0;
					for(std::map<int, cv::Point3f>::const_iterator iter=odom.info().localMap.begin(); iter!=odom.info().localMap.end(); ++iter)
					{
						// filter very far features from current location
						if(uNormSquared(iter->second.x-odom.pose().x(), iter->second.y-odom.pose().y(), iter->second.z-odom.pose().z()) < 100*100)
						{
							(*cloud)[i].x = iter->second.x;
							(*cloud)[i].y = iter->second.y;
							(*cloud)[i].z = iter->second.z;

							// green = inlier, yellow = outliers
							bool inlier = odom.info().words.find(iter->first) != odom.info().words.end();
							(*cloud)[i].r = inlier?0:255;
							(*cloud)[i].g = 255;
							(*cloud)[i].b = 0;
							if(!_preferencesDialog->isOdomOnlyInliersShown() || inlier)
							{
								++i;
							}
						}
					}
					cloud->resize(i);

					_cloudViewer->addCloud("featuresOdom", cloud, _odometryCorrection);
					_cloudViewer->setCloudVisibility("featuresOdom", true);
					_cloudViewer->setCloudPointSize("featuresOdom", _preferencesDialog->getFeaturesPointSize(1));
				}
				featuresUpdated = true;
			}

			if(_preferencesDialog->isFrustumsShown(1))
			{
				QMap<std::string, Transform> addedFrustums = _cloudViewer->getAddedFrustums();
				for(QMap<std::string, Transform>::iterator iter = addedFrustums.begin(); iter!=addedFrustums.end(); ++iter)
				{
					std::list<std::string> splitted = uSplitNumChar(iter.key());
					if(splitted.size() == 2)
					{
						int id = std::atoi(splitted.back().c_str());
						id -= id%10;
						if(splitted.front().compare("f_odom_") == 0 &&
								odom.info().localBundlePoses.find(id) == odom.info().localBundlePoses.end())
						{
							_cloudViewer->removeFrustum(iter.key());
						}
					}
				}

				for(std::map<int, Transform>::const_iterator iter=odom.info().localBundlePoses.begin();iter!=odom.info().localBundlePoses.end(); ++iter)
				{
					std::string frustumId = uFormat("f_odom_%d", iter->first*10);
					if(_cloudViewer->getAddedFrustums().contains(frustumId))
					{
						for(size_t i=0; i<10; ++i)
						{
							std::string subFrustumId = uFormat("f_odom_%d", iter->first*10+i);
							_cloudViewer->updateFrustumPose(subFrustumId, _odometryCorrection*iter->second);
						}
					}
					else if(odom.info().localBundleModels.find(iter->first) != odom.info().localBundleModels.end())
					{
						const std::vector<CameraModel> & models = odom.info().localBundleModels.at(iter->first);
						for(size_t i=0; i<models.size(); ++i)
						{
							Transform t = models[i].localTransform();
							if(!t.isNull())
							{
								QColor color = Qt::yellow;
								std::string subFrustumId = uFormat("f_odom_%d", iter->first*10+i);
								_cloudViewer->addOrUpdateFrustum(subFrustumId, _odometryCorrection*iter->second, t, _cloudViewer->getFrustumScale(), color, models[i].fovX(), models[i].fovY());
							}
						}
					}
				}
			}
			if(  _preferencesDialog->isIMUGravityShown(1) &&
				(data->imu().orientation().val[0]!=0 ||
				data->imu().orientation().val[1]!=0 ||
				data->imu().orientation().val[2]!=0 ||
				data->imu().orientation().val[3]!=0))
			{
				Eigen::Vector3f gravity(0,0,-_preferencesDialog->getIMUGravityLength(1));
				Transform orientation(0,0,0, data->imu().orientation()[0], data->imu().orientation()[1], data->imu().orientation()[2], data->imu().orientation()[3]);
				gravity = (orientation* data->imu().localTransform().inverse()*(_odometryCorrection*pose).rotation().inverse()).toEigen3f()*gravity;
				_cloudViewer->addOrUpdateLine("odom_imu_orientation", _odometryCorrection*pose, (_odometryCorrection*pose).translation()*Transform(gravity[0], gravity[1], gravity[2], 0, 0, 0)*pose.rotation().inverse(), Qt::yellow, true, true);
				filteredGravityUpdated = true;
			}
			if( _preferencesDialog->isIMUAccShown() &&
				(data->imu().linearAcceleration().val[0]!=0 ||
				data->imu().linearAcceleration().val[1]!=0 ||
				data->imu().linearAcceleration().val[2]!=0))
			{
				Eigen::Vector3f gravity(
						-data->imu().linearAcceleration().val[0],
						-data->imu().linearAcceleration().val[1],
						-data->imu().linearAcceleration().val[2]);
				gravity = gravity.normalized() * _preferencesDialog->getIMUGravityLength(1);
				gravity = data->imu().localTransform().toEigen3f()*gravity;
				_cloudViewer->addOrUpdateLine("odom_imu_acc", _odometryCorrection*pose, _odometryCorrection*pose*Transform(gravity[0], gravity[1], gravity[2], 0, 0, 0), Qt::red, true, true);
				accelerationUpdated = true;
			}
		}
		if(!dataIgnored)
		{
			if(!cloudUpdated && _cloudViewer->getAddedClouds().contains("cloudOdom"))
			{
				_cloudViewer->setCloudVisibility("cloudOdom", false);
			}
			if(!scanUpdated && _cloudViewer->getAddedClouds().contains("scanOdom"))
			{
				_cloudViewer->setCloudVisibility("scanOdom", false);
			}
			if(!scanUpdated && _cloudViewer->getAddedClouds().contains("scanMapOdom"))
			{
				_cloudViewer->setCloudVisibility("scanMapOdom", false);
			}
			if(!featuresUpdated && _cloudViewer->getAddedClouds().contains("featuresOdom"))
			{
				_cloudViewer->setCloudVisibility("featuresOdom", false);
			}
			if(!filteredGravityUpdated && _cloudViewer->getAddedLines().find("odom_imu_orientation") != _cloudViewer->getAddedLines().end())
			{
				_cloudViewer->removeLine("odom_imu_orientation");
			}
			if(!accelerationUpdated && _cloudViewer->getAddedLines().find("odom_imu_acc") != _cloudViewer->getAddedLines().end())
			{
				_cloudViewer->removeLine("odom_imu_acc");
			}
		}
		UDEBUG("Time 3D Rendering: %fs", time.ticks());
	}

	if(!odom.pose().isNull())
	{
		_odometryReceived = true;
		// update camera position
		if(data->cameraModels().size() && data->cameraModels()[0].isValidForProjection())
		{
			_cloudViewer->updateCameraFrustums(_odometryCorrection*odom.pose(), data->cameraModels());
		}
		else if(data->stereoCameraModels().size() && data->stereoCameraModels()[0].isValidForProjection())
		{
			_cloudViewer->updateCameraFrustums(_odometryCorrection*odom.pose(), data->stereoCameraModels());
		}
		else if(!data->laserScanRaw().isEmpty() ||
				!data->laserScanCompressed().isEmpty())
		{
			Transform scanLocalTransform;
			if(!data->laserScanRaw().isEmpty())
			{
				scanLocalTransform = data->laserScanRaw().localTransform();
			}
			else
			{
				scanLocalTransform = data->laserScanCompressed().localTransform();
			}
			//fake frustum
			CameraModel model(
					2,
					2,
					2,
					1.5,
					scanLocalTransform*CameraModel::opticalRotation(),
					0,
					cv::Size(4,3));
			_cloudViewer->updateCameraFrustum(_odometryCorrection*odom.pose(), model);

		}
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
		if(_preferencesDialog->isFramesShown())
		{
			_cloudViewer->addOrUpdateLine("odom_to_base_link", _odometryCorrection, _odometryCorrection*odom.pose(), qRgb(255, 128, 0), false, false);
		}
		else
		{
			_cloudViewer->removeLine("odom_to_base_link");
		}
#endif
		_cloudViewer->updateCameraTargetPosition(_odometryCorrection*odom.pose());
		UDEBUG("Time Update Pose: %fs", time.ticks());
	}
	_cloudViewer->refreshView();

	if(_ui->graphicsView_graphView->isVisible())
	{
		if(!pose.isNull() && !odom.pose().isNull())
		{
			_ui->graphicsView_graphView->updateReferentialPosition(_odometryCorrection*odom.pose());
			_ui->graphicsView_graphView->update();
			UDEBUG("Time Update graphview: %fs", time.ticks());
		}
	}

	if(_ui->dockWidget_odometry->isVisible() &&
	   !data->imageRaw().empty())
	{
		if(_ui->imageView_odometry->isFeaturesShown())
		{
			if(odom.info().type == (int)Odometry::kTypeF2M || odom.info().type == (int)Odometry::kTypeORBSLAM)
			{
				if(_preferencesDialog->isOdomOnlyInliersShown())
				{
					std::multimap<int, cv::KeyPoint> kpInliers;
					for(unsigned int i=0; i<odom.info().reg.inliersIDs.size(); ++i)
					{
						kpInliers.insert(*odom.info().words.find(odom.info().reg.inliersIDs[i]));
					}
					_ui->imageView_odometry->setFeatures(
							kpInliers,
							data->depthRaw(),
							Qt::green);
				}
				else
				{
					_ui->imageView_odometry->setFeatures(
							odom.info().words,
							data->depthRaw(),
							Qt::yellow);
				}
			}
			else if(odom.info().type == (int)Odometry::kTypeF2F ||
					odom.info().type == (int)Odometry::kTypeViso2 ||
					odom.info().type == (int)Odometry::kTypeFovis ||
					odom.info().type == (int)Odometry::kTypeMSCKF ||
					odom.info().type == (int)Odometry::kTypeVINS ||
					odom.info().type == (int)Odometry::kTypeOpenVINS)
			{
				std::vector<cv::KeyPoint> kpts;
				cv::KeyPoint::convert(odom.info().newCorners, kpts, 7);
				_ui->imageView_odometry->setFeatures(
						kpts,
						data->depthRaw(),
						Qt::red);
			}
		}

		//detect if it is OdometryMono initialization
		bool monoInitialization = false;
		if(_preferencesDialog->getOdomStrategy() ==  6 && odom.info().type == (int)Odometry::kTypeF2F)
		{
			monoInitialization = true;
		}

		_ui->imageView_odometry->clearLines();
		if(lost && !monoInitialization)
		{
			if(lostStateChanged)
			{
				// save state
				_odomImageShow = _ui->imageView_odometry->isImageShown();
				_odomImageDepthShow = _ui->imageView_odometry->isImageDepthShown();
			}
			_ui->imageView_odometry->setImageDepth(data->imageRaw());
			_ui->imageView_odometry->setImageShown(true);
			_ui->imageView_odometry->setImageDepthShown(true);
		}
		else
		{
			if(lostStateChanged)
			{
				// restore state
				_ui->imageView_odometry->setImageShown(_odomImageShow);
				_ui->imageView_odometry->setImageDepthShown(_odomImageDepthShow);
			}

			_ui->imageView_odometry->setImage(uCvMat2QImage(data->imageRaw()));
			if(_ui->imageView_odometry->isImageDepthShown() && !data->depthOrRightRaw().empty())
			{
				_ui->imageView_odometry->setImageDepth(data->depthOrRightRaw());
			}

			if( odom.info().type == (int)Odometry::kTypeF2M ||
				odom.info().type == (int)Odometry::kTypeORBSLAM ||
				odom.info().type == (int)Odometry::kTypeMSCKF ||
				odom.info().type == (int)Odometry::kTypeVINS ||
				odom.info().type == (int)Odometry::kTypeOpenVINS)
			{
				if(_ui->imageView_odometry->isFeaturesShown() && !_preferencesDialog->isOdomOnlyInliersShown())
				{
					for(unsigned int i=0; i<odom.info().reg.matchesIDs.size(); ++i)
					{
						_ui->imageView_odometry->setFeatureColor(odom.info().reg.matchesIDs[i], Qt::red); // outliers
					}
					for(unsigned int i=0; i<odom.info().reg.inliersIDs.size(); ++i)
					{
						_ui->imageView_odometry->setFeatureColor(odom.info().reg.inliersIDs[i], Qt::green); // inliers
					}
				}
			}
			if((odom.info().type == (int)Odometry::kTypeF2F ||
				odom.info().type == (int)Odometry::kTypeViso2 ||
				odom.info().type == (int)Odometry::kTypeFovis) && odom.info().refCorners.size())
			{
				if(_ui->imageView_odometry->isFeaturesShown() || _ui->imageView_odometry->isLinesShown())
				{
					//draw lines
					UASSERT(odom.info().refCorners.size() == odom.info().newCorners.size());
					std::set<int> inliers(odom.info().cornerInliers.begin(), odom.info().cornerInliers.end());
					for(unsigned int i=0; i<odom.info().refCorners.size(); ++i)
					{
						if(_ui->imageView_odometry->isFeaturesShown() && inliers.find(i) != inliers.end())
						{
							_ui->imageView_odometry->setFeatureColor(i, Qt::green); // inliers
						}
						if(_ui->imageView_odometry->isLinesShown())
						{
							_ui->imageView_odometry->addLine(
									odom.info().newCorners[i].x,
									odom.info().newCorners[i].y,
									odom.info().refCorners[i].x,
									odom.info().refCorners[i].y,
									inliers.find(i) != inliers.end()?Qt::blue:Qt::yellow);
						}
					}
				}
			}
		}
		if(!data->imageRaw().empty())
		{
			_ui->imageView_odometry->setSceneRect(QRectF(0,0,(float)data->imageRaw().cols, (float)data->imageRaw().rows));
		}

		_ui->imageView_odometry->update();

		UDEBUG("Time update imageview: %fs", time.ticks());
	}

	if(_ui->actionAuto_screen_capture->isChecked() && _autoScreenCaptureOdomSync)
	{
		this->captureScreen(_autoScreenCaptureRAM, _autoScreenCapturePNG);
	}

	//Process info
	if(_preferencesDialog->isCacheSavedInFigures() || _ui->statsToolBox->isVisible())
	{
		_ui->statsToolBox->updateStat("Odometry/Inliers/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.inliers, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/InliersMeanDistance/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.inliersMeanDistance, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/InliersDistribution/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.inliersDistribution, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/InliersRatio/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), odom.info().features<=0?0.0f:float(odom.info().reg.inliers)/float(odom.info().features), _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/ICPInliersRatio/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.icpInliersRatio, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/ICPRotation/rad", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.icpRotation, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/ICPTranslation/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.icpTranslation, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/ICPStructuralComplexity/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.icpStructuralComplexity, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/ICPStructuralDistribution/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.icpStructuralDistribution, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/ICPCorrespondences/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.icpCorrespondences, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/ICPRMS/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.icpRMS, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/Matches/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.matches, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/MatchesRatio/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), odom.info().features<=0?0.0f:float(odom.info().reg.matches)/float(odom.info().features), _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/StdDevLin/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), sqrt((float)odom.info().reg.covariance.at<double>(0,0)), _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/VarianceLin/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.covariance.at<double>(0,0), _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/StdDevAng/rad", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), sqrt((float)odom.info().reg.covariance.at<double>(5,5)), _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/VarianceAng/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().reg.covariance.at<double>(5,5), _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/TimeEstimation/ms", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().timeEstimation*1000.0f, _preferencesDialog->isCacheSavedInFigures());
		if(odom.info().timeParticleFiltering>0.0f)
		{
			_ui->statsToolBox->updateStat("Odometry/TimeFiltering/ms", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().timeParticleFiltering*1000.0f, _preferencesDialog->isCacheSavedInFigures());
		}
		if(odom.info().gravityRollError>0.0f || odom.info().gravityPitchError > 0.0f)
		{
			_ui->statsToolBox->updateStat("Odometry/GravityRollError/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().gravityRollError*180/M_PI, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/GravityPitchError/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().gravityPitchError*180/M_PI, _preferencesDialog->isCacheSavedInFigures());
		}
		_ui->statsToolBox->updateStat("Odometry/Features/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().features, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/LocalMapSize/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().localMapSize, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/LocalScanMapSize/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().localScanMapSize, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/LocalKeyFrames/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().localKeyFrames, _preferencesDialog->isCacheSavedInFigures());
		if(odom.info().localBundleTime > 0.0f)
		{
			_ui->statsToolBox->updateStat("Odometry/localBundleOutliers/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().localBundleOutliers, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/localBundleConstraints/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().localBundleConstraints, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/localBundleTime/ms", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().localBundleTime*1000.0f, _preferencesDialog->isCacheSavedInFigures());
		}
		_ui->statsToolBox->updateStat("Odometry/KeyFrameAdded/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)odom.info().keyFrameAdded?1.0f:0.0f, _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Odometry/ID/", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), (float)data->id(), _preferencesDialog->isCacheSavedInFigures());

		Transform odomT;
		float dist=0.0f, x,y,z, roll,pitch,yaw;
		if(!odom.info().transform.isNull())
		{
			odomT = odom.info().transform;
			odom.info().transform.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
			dist = odom.info().transform.getNorm();
			_ui->statsToolBox->updateStat("Odometry/T/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Tx/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), x, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Ty/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), y, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Tz/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), z, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Troll/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), roll*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Tpitch/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), pitch*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Tyaw/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), yaw*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
		}

		if(!odom.info().transformFiltered.isNull())
		{
			odomT = odom.info().transformFiltered;
			odom.info().transformFiltered.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
			dist = odom.info().transformFiltered.getNorm();
			_ui->statsToolBox->updateStat("Odometry/TF/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TFx/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), x, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TFy/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), y, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TFz/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), z, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TFroll/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), roll*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TFpitch/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), pitch*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TFyaw/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), yaw*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
		}
		if(odom.info().interval > 0)
		{
			_ui->statsToolBox->updateStat("Odometry/Interval/ms", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), odom.info().interval*1000.f, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Speed/kph", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist/odom.info().interval*3.6f, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Speed/mph", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist/odom.info().interval*2.237f, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Speed/mps", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist/odom.info().interval, _preferencesDialog->isCacheSavedInFigures());

			if(!odom.info().guess.isNull())
			{
				odom.info().guess.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
				dist = odom.info().guess.getNorm();
				_ui->statsToolBox->updateStat("Odometry/SpeedGuess/kph", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist/odom.info().interval*3.6f, _preferencesDialog->isCacheSavedInFigures());
				_ui->statsToolBox->updateStat("Odometry/SpeedGuess/mph", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist/odom.info().interval*2.237f, _preferencesDialog->isCacheSavedInFigures());
				_ui->statsToolBox->updateStat("Odometry/SpeedGuess/mps", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist/odom.info().interval, _preferencesDialog->isCacheSavedInFigures());
			}
		}

		if(!odom.info().transformGroundTruth.isNull())
		{
			if(!odomT.isNull())
			{
				rtabmap::Transform diff = odom.info().transformGroundTruth.inverse()*odomT;
				_ui->statsToolBox->updateStat("Odometry/TG_error_lin/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), diff.getNorm(), _preferencesDialog->isCacheSavedInFigures());
				_ui->statsToolBox->updateStat("Odometry/TG_error_ang/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), diff.getAngle()*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			}

			odom.info().transformGroundTruth.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
			dist = odom.info().transformGroundTruth.getNorm();
			_ui->statsToolBox->updateStat("Odometry/TG/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TGx/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), x, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TGy/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), y, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TGz/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), z, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TGroll/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), roll*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TGpitch/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), pitch*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/TGyaw/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), yaw*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			if(odom.info().interval > 0)
			{
				_ui->statsToolBox->updateStat("Odometry/SpeedG/kph", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist/odom.info().interval*3.6f, _preferencesDialog->isCacheSavedInFigures());
				_ui->statsToolBox->updateStat("Odometry/SpeedG/mph", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist/odom.info().interval*2.237f, _preferencesDialog->isCacheSavedInFigures());
				_ui->statsToolBox->updateStat("Odometry/SpeedG/mps", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), dist/odom.info().interval, _preferencesDialog->isCacheSavedInFigures());
			}
		}

		//cumulative pose
		if(!odom.pose().isNull())
		{
			odom.pose().getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
			_ui->statsToolBox->updateStat("Odometry/Px/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), x, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Py/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), y, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Pz/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), z, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Proll/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), roll*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Ppitch/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), pitch*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/Pyaw/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), yaw*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
		}
		if(!data->groundTruth().isNull())
		{
			data->groundTruth().getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
			_ui->statsToolBox->updateStat("Odometry/PGx/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), x, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/PGy/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), y, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/PGz/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), z, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/PGroll/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), roll*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/PGpitch/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), pitch*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
			_ui->statsToolBox->updateStat("Odometry/PGyaw/deg", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), yaw*180.0/CV_PI, _preferencesDialog->isCacheSavedInFigures());
		}

		if(odom.info().distanceTravelled > 0)
		{
			_ui->statsToolBox->updateStat("Odometry/Distance/m", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), odom.info().distanceTravelled, _preferencesDialog->isCacheSavedInFigures());
		}

		_ui->statsToolBox->updateStat("GUI/Refresh odom/ms", _preferencesDialog->isTimeUsedInFigures()?data->stamp()-_firstStamp:(float)data->id(), time.elapsed()*1000.0, _preferencesDialog->isCacheSavedInFigures());
		UDEBUG("Time updating Stats toolbox: %fs", time.ticks());
	}
	_processingOdometry = false;

	Q_EMIT(odometryProcessed());
}

void MainWindow::processStats(const rtabmap::Statistics & stat)
{
	_processingStatistics = true;
	ULOGGER_DEBUG("");
	QTime time, totalTime;
	time.start();
	totalTime.start();
	//Affichage des stats et images

	if(_firstStamp == 0.0)
	{
		_firstStamp = stat.stamp();
	}

	int refMapId = -1, loopMapId = -1;
	if(stat.getLastSignatureData().id() == stat.refImageId())
	{
		refMapId = stat.getLastSignatureData().mapId();
	}
	int highestHypothesisId = static_cast<float>(uValue(stat.data(), Statistics::kLoopHighest_hypothesis_id(), 0.0f));
	int loopId = stat.loopClosureId()>0?stat.loopClosureId():stat.proximityDetectionId()>0?stat.proximityDetectionId():highestHypothesisId;
	if(loopId>0 && _cachedSignatures.contains(loopId))
	{
		loopMapId = _cachedSignatures.value(loopId).mapId();
	}

	_ui->label_refId->setText(QString("New ID = %1 [%2]").arg(stat.refImageId()).arg(refMapId));

	if(stat.extended())
	{
		float totalTime = static_cast<float>(uValue(stat.data(), Statistics::kTimingTotal(), 0.0f));
		if(totalTime/1000.0f > float(1.0/_preferencesDialog->getDetectionRate()))
		{
			UWARN("Processing time (%fs) is over detection rate (%fs), real-time problem!", totalTime/1000.0f, 1.0/_preferencesDialog->getDetectionRate());
		}

		UDEBUG("");
		bool highestHypothesisIsSaved = (bool)uValue(stat.data(), Statistics::kLoopHypothesis_reactivated(), 0.0f);

		bool smallMovement = (bool)uValue(stat.data(), Statistics::kMemorySmall_movement(), 0.0f);

		bool fastMovement = (bool)uValue(stat.data(), Statistics::kMemoryFast_movement(), 0.0f);

		int rehearsalMerged = (int)uValue(stat.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);

		// update cache
		Signature signature;
		if(stat.getLastSignatureData().id() == stat.refImageId())
		{
			signature = stat.getLastSignatureData();
		}
		else if(rehearsalMerged>0 &&
				rehearsalMerged == stat.getLastSignatureData().id() &&
				_cachedSignatures.contains(rehearsalMerged))
		{
			signature = _cachedSignatures.value(rehearsalMerged);
		}

		if(signature.id()!=0)
		{
			// make sure data are uncompressed
			// We don't need to uncompress images if we don't show them
			bool uncompressImages = !signature.sensorData().imageCompressed().empty() && (
					_ui->imageView_source->isVisible() ||
					(_loopClosureViewer->isVisible() &&
							!signature.sensorData().depthOrRightCompressed().empty()) ||
					(_cloudViewer->isVisible() &&
							_preferencesDialog->isCloudsShown(0) &&
							!signature.sensorData().depthOrRightCompressed().empty()));
			bool uncompressScan = !signature.sensorData().laserScanCompressed().isEmpty() && (
					_loopClosureViewer->isVisible() ||
					(_cloudViewer->isVisible() && _preferencesDialog->isScansShown(0)));
			cv::Mat tmpRgb, tmpDepth, tmpG, tmpO, tmpE;
			LaserScan tmpScan;
			signature.sensorData().uncompressData(
					uncompressImages?&tmpRgb:0,
					uncompressImages && !signature.sensorData().depthOrRightCompressed().empty()?&tmpDepth:0,
					uncompressScan?&tmpScan:0,
					0, &tmpG, &tmpO, &tmpE);

			if( stat.getLastSignatureData().id() == stat.refImageId() &&
				uStr2Bool(_preferencesDialog->getParameter(Parameters::kMemIncrementalMemory())) &&
				signature.getWeight()>=0) // ignore intermediate nodes for the cache
			{
				if(smallMovement || fastMovement)
				{
					_cachedSignatures.insert(0, signature); // zero means temporary
				}
				else
				{
					_cachedSignatures.insert(signature.id(), signature);
					_cachedMemoryUsage += signature.sensorData().getMemoryUsed();
					unsigned int count = 0;
					if(!signature.getWords3().empty())
					{
						for(std::multimap<int, int>::const_iterator jter=signature.getWords().upper_bound(-1); jter!=signature.getWords().end(); ++jter)
						{
							if(util3d::isFinite(signature.getWords3()[jter->second]))
							{
								++count;
							}
						}
					}
					_cachedWordsCount.insert(std::make_pair(signature.id(), (float)count));
				}
			}
		}

		// For intermediate empty nodes, keep latest image shown
		if(signature.getWeight() >= 0)
		{
			_ui->imageView_source->clear();
			_ui->imageView_loopClosure->clear();

			if(signature.sensorData().imageRaw().empty() && signature.getWords().empty())
			{
				// To see colors
				_ui->imageView_source->setSceneRect(QRect(0,0,640,480));
			}

			_ui->imageView_source->setBackgroundColor(_ui->imageView_source->getDefaultBackgroundColor());
			_ui->imageView_loopClosure->setBackgroundColor(_ui->imageView_loopClosure->getDefaultBackgroundColor());

			_ui->label_matchId->clear();

			bool rehearsedSimilarity = (float)uValue(stat.data(), Statistics::kMemoryRehearsal_id(), 0.0f) != 0.0f;
			int proximityTimeDetections = (int)uValue(stat.data(), Statistics::kProximityTime_detections(), 0.0f);
			bool scanMatchingSuccess = (bool)uValue(stat.data(), Statistics::kNeighborLinkRefiningAccepted(), 0.0f);
			_ui->label_stats_imageNumber->setText(QString("%1 [%2]").arg(stat.refImageId()).arg(refMapId));

			if(rehearsalMerged > 0)
			{
				_ui->imageView_source->setBackgroundColor(Qt::blue);
			}
			else if(proximityTimeDetections > 0)
			{
				_ui->imageView_source->setBackgroundColor(Qt::darkYellow);
			}
			else if(scanMatchingSuccess)
			{
				_ui->imageView_source->setBackgroundColor(Qt::darkCyan);
			}
			else if(rehearsedSimilarity)
			{
				_ui->imageView_source->setBackgroundColor(Qt::darkBlue);
			}
			else if(smallMovement)
			{
				_ui->imageView_source->setBackgroundColor(Qt::gray);
			}
			else if(fastMovement)
			{
				_ui->imageView_source->setBackgroundColor(Qt::magenta);
			}
			// Set color code as tooltip
			if(_ui->label_refId->toolTip().isEmpty())
			{
				_ui->label_refId->setToolTip(
					"Background Color Code:\n"
					"  Blue = Weight Update Merged\n"
					"  Dark Blue = Weight Update\n"
					"  Dark Yellow = Proximity Detection in Time\n"
					"  Dark Cyan = Neighbor Link Refined\n"
					"  Gray = Small Movement\n"
					"  Magenta = Fast Movement\n"
					"Feature Color code:\n"
					"  Green = New\n"
					"  Yellow = New but Not Unique\n"
					"  Red = In Vocabulary\n"
					"  Blue = In Vocabulary and in Previous Signature\n"
					"  Pink = In Vocabulary and in Loop Closure Signature\n"
					"  Gray = Not Quantized to Vocabulary");
			}
			// Set color code as tooltip
			if(_ui->label_matchId->toolTip().isEmpty())
			{
				_ui->label_matchId->setToolTip(
					"Background Color Code:\n"
					"  Green = Accepted Loop Closure Detection\n"
					"  Red = Rejected Loop Closure Detection\n"
					"  Yellow = Proximity Detection in Space\n"
					"Feature Color code:\n"
					"  Red = In Vocabulary\n"
					"  Pink = In Vocabulary and in Loop Closure Signature\n"
					"  Gray = Not Quantized to Vocabulary");
			}

			int rejectedHyp = bool(uValue(stat.data(), Statistics::kLoopRejectedHypothesis(), 0.0f));
			float highestHypothesisValue = uValue(stat.data(), Statistics::kLoopHighest_hypothesis_value(), 0.0f);
			int landmarkId = static_cast<int>(uValue(stat.data(), Statistics::kLoopLandmark_detected(), 0.0f));
			int landmarkNodeRef = static_cast<int>(uValue(stat.data(), Statistics::kLoopLandmark_detected_node_ref(), 0.0f));
			int matchId = 0;
			Signature loopSignature;
			int shownLoopId = 0;
			if(highestHypothesisId > 0 || stat.proximityDetectionId()>0 || landmarkId>0)
			{
				bool show = true;
				if(stat.loopClosureId() > 0)
				{
					_ui->imageView_loopClosure->setBackgroundColor(Qt::green);
					_ui->label_stats_loopClosuresDetected->setText(QString::number(_ui->label_stats_loopClosuresDetected->text().toInt() + 1));
					if(highestHypothesisIsSaved)
					{
						_ui->label_stats_loopClosuresReactivatedDetected->setText(QString::number(_ui->label_stats_loopClosuresReactivatedDetected->text().toInt() + 1));
					}
					_ui->label_matchId->setText(QString("Match ID = %1 [%2]").arg(stat.loopClosureId()).arg(loopMapId));
					matchId = stat.loopClosureId();
				}
				else if(stat.proximityDetectionId())
				{
					_ui->imageView_loopClosure->setBackgroundColor(Qt::yellow);
					_ui->label_matchId->setText(QString("Local match = %1 [%2]").arg(stat.proximityDetectionId()).arg(loopMapId));
					matchId = stat.proximityDetectionId();
				}
				else if(landmarkId!=0)
				{
					highestHypothesisId = landmarkNodeRef;
					if(rejectedHyp)
					{
						show = _preferencesDialog->imageRejectedShown();
						if(show)
						{
							_ui->imageView_loopClosure->setBackgroundColor(Qt::red);
							_ui->label_stats_loopClosuresRejected->setText(QString::number(_ui->label_stats_loopClosuresRejected->text().toInt() + 1));
							_ui->label_matchId->setText(QString("Landmark rejected = %1 with %2").arg(landmarkId).arg(landmarkNodeRef));
						}
					}
					else
					{
						_ui->imageView_loopClosure->setBackgroundColor(QColor("orange"));
						_ui->label_matchId->setText(QString("Landmark match = %1 with %2").arg(landmarkId).arg(landmarkNodeRef));
						matchId = landmarkNodeRef;
					}
				}
				else if(rejectedHyp && highestHypothesisValue >= _preferencesDialog->getLoopThr())
				{
					show = _preferencesDialog->imageRejectedShown() || _preferencesDialog->imageHighestHypShown();
					if(show)
					{
						_ui->imageView_loopClosure->setBackgroundColor(Qt::red);
						_ui->label_stats_loopClosuresRejected->setText(QString::number(_ui->label_stats_loopClosuresRejected->text().toInt() + 1));
						_ui->label_matchId->setText(QString("Loop hypothesis %1 rejected!").arg(highestHypothesisId));
					}
				}
				else
				{
					show = _preferencesDialog->imageHighestHypShown();
					if(show)
					{
						_ui->label_matchId->setText(QString("Highest hypothesis (%1)").arg(highestHypothesisId));
					}
				}

				if(show)
				{
					shownLoopId = matchId>0?matchId:highestHypothesisId;
					QMap<int, Signature>::iterator iter = _cachedSignatures.find(shownLoopId);
					if(iter != _cachedSignatures.end())
					{
						// uncompress after copy to avoid keeping uncompressed data in memory
						loopSignature = iter.value();
						bool uncompressImages = !loopSignature.sensorData().imageCompressed().empty() && (
								_ui->imageView_source->isVisible() ||
								(_loopClosureViewer->isVisible() &&
										!loopSignature.sensorData().depthOrRightCompressed().empty()));
						bool uncompressScan = _loopClosureViewer->isVisible() &&
								!loopSignature.sensorData().laserScanCompressed().isEmpty();
						if(uncompressImages || uncompressScan)
						{
							cv::Mat tmpRGB, tmpDepth;
							LaserScan tmpScan;
							loopSignature.sensorData().uncompressData(
									uncompressImages?&tmpRGB:0,
									uncompressImages?&tmpDepth:0,
									uncompressScan?&tmpScan:0);
						}
					}
				}
			}
			_refIds.push_back(stat.refImageId());
			_loopClosureIds.push_back(matchId);
			if(matchId > 0)
			{
				_cachedLocalizationsCount[matchId] += 1.0f;
			}
			UDEBUG("time= %d ms (update detection ui)", time.restart());

			//update image views
			if(!signature.sensorData().imageRaw().empty() || signature.getWords().size())
			{
				cv::Mat refImage = signature.sensorData().imageRaw();
				cv::Mat loopImage = loopSignature.sensorData().imageRaw();

				if( _preferencesDialog->isMarkerDetection() &&
					_preferencesDialog->isLandmarksShown())
				{
					//draw markers
					if(!signature.getLandmarks().empty())
					{
						if(refImage.channels() == 1)
						{
							cv::Mat imgColor;
							cvtColor(refImage, imgColor, cv::COLOR_GRAY2BGR);
							refImage = imgColor;
						}
						else
						{
							refImage = refImage.clone();
						}
						drawLandmarks(refImage, signature);
					}
					if(!loopSignature.getLandmarks().empty())
					{
						if(loopImage.channels() == 1)
						{
							cv::Mat imgColor;
							cv::cvtColor(loopImage, imgColor, cv::COLOR_GRAY2BGR);
							loopImage = imgColor;
						}
						else
						{
							loopImage = loopImage.clone();
						}
						drawLandmarks(loopImage, loopSignature);
					}
				}

				UCvMat2QImageThread qimageThread(refImage);
				UCvMat2QImageThread qimageLoopThread(loopImage);
				qimageThread.start();
				qimageLoopThread.start();
				qimageThread.join();
				qimageLoopThread.join();
				QImage img = qimageThread.getQImage();
				QImage lcImg = qimageLoopThread.getQImage();
				UDEBUG("time= %d ms (convert image to qt)", time.restart());

				if(!img.isNull())
				{
					_ui->imageView_source->setImage(img);
				}
				if(!signature.sensorData().depthOrRightRaw().empty())
				{
					_ui->imageView_source->setImageDepth(signature.sensorData().depthOrRightRaw());
				}
				if(img.isNull() && signature.sensorData().depthOrRightRaw().empty())
				{
					QRect sceneRect;
					if(signature.sensorData().cameraModels().size())
					{
						for(unsigned int i=0; i<signature.sensorData().cameraModels().size(); ++i)
						{
							sceneRect.setWidth(sceneRect.width()+signature.sensorData().cameraModels()[i].imageWidth());
							sceneRect.setHeight(sceneRect.height()+signature.sensorData().cameraModels()[i].imageHeight());
						}
					}
					else if(signature.sensorData().stereoCameraModels().size())
					{
						for(unsigned int i=0; i<signature.sensorData().cameraModels().size(); ++i)
						{
							sceneRect.setWidth(sceneRect.width()+signature.sensorData().stereoCameraModels()[i].left().imageWidth());
							sceneRect.setHeight(sceneRect.height()+signature.sensorData().stereoCameraModels()[i].left().imageHeight());
						}
					}
					if(sceneRect.isValid())
					{
						_ui->imageView_source->setSceneRect(sceneRect);
					}
				}
				if(!lcImg.isNull())
				{
					_ui->imageView_loopClosure->setImage(lcImg);
				}
				if(!loopSignature.sensorData().depthOrRightRaw().empty())
				{
					_ui->imageView_loopClosure->setImageDepth(loopSignature.sensorData().depthOrRightRaw());
				}
				if(_ui->imageView_loopClosure->sceneRect().isNull())
				{
					_ui->imageView_loopClosure->setSceneRect(_ui->imageView_source->sceneRect());
				}
			}
			else if(_ui->imageView_loopClosure->sceneRect().isNull() &&
					!_ui->imageView_source->sceneRect().isNull())
			{
				_ui->imageView_loopClosure->setSceneRect(_ui->imageView_source->sceneRect());
			}

			UDEBUG("time= %d ms (update detection imageviews)", time.restart());

			// do it after scaling
			std::multimap<int, cv::KeyPoint> wordsA;
			std::multimap<int, cv::KeyPoint> wordsB;
			for(std::map<int, int>::const_iterator iter=signature.getWords().begin(); iter!=signature.getWords().end(); ++iter)
			{
				wordsA.insert(wordsA.end(), std::make_pair(iter->first, signature.getWordsKpts()[iter->second]));
			}
			for(std::map<int, int>::const_iterator iter=loopSignature.getWords().begin(); iter!=loopSignature.getWords().end(); ++iter)
			{
				wordsB.insert(wordsB.end(), std::make_pair(iter->first, loopSignature.getWordsKpts()[iter->second]));
			}
			this->drawKeypoints(wordsA, wordsB);

			UDEBUG("time= %d ms (draw keypoints)", time.restart());

			// loop closure view
			if((stat.loopClosureId() > 0 || stat.proximityDetectionId() > 0)  &&
			   !stat.loopClosureTransform().isNull() &&
			   !loopSignature.sensorData().imageRaw().empty())
			{
				// the last loop closure data
				Transform loopClosureTransform = stat.loopClosureTransform();
				signature.setPose(loopClosureTransform);
				_loopClosureViewer->setData(loopSignature, signature);
				if(_ui->dockWidget_loopClosureViewer->isVisible())
				{
					UTimer loopTimer;
					_loopClosureViewer->updateView(Transform(), _preferencesDialog->getAllParameters());
					UINFO("Updating loop closure cloud view time=%fs", loopTimer.elapsed());
					if(_preferencesDialog->isCacheSavedInFigures() || _ui->statsToolBox->isVisible())
					{
						_ui->statsToolBox->updateStat("GUI/RGB-D closure view/ms", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), int(loopTimer.elapsed()*1000.0f), _preferencesDialog->isCacheSavedInFigures());
					}
				}

				UDEBUG("time= %d ms (update loop closure viewer)", time.restart());
			}
		}

		// PDF AND LIKELIHOOD
		if(!stat.posterior().empty() && _ui->dockWidget_posterior->isVisible())
		{
			UDEBUG("");
			_posteriorCurve->setData(QMap<int, float>(stat.posterior()), QMap<int, int>(stat.weights()));

			ULOGGER_DEBUG("");
			//Adjust thresholds
			Q_EMIT(loopClosureThrChanged(_preferencesDialog->getLoopThr()));
		}
		if(!stat.likelihood().empty() && _ui->dockWidget_likelihood->isVisible())
		{
			_likelihoodCurve->setData(QMap<int, float>(stat.likelihood()), QMap<int, int>(stat.weights()));
		}
		if(!stat.rawLikelihood().empty() && _ui->dockWidget_rawlikelihood->isVisible())
		{
			_rawLikelihoodCurve->setData(QMap<int, float>(stat.rawLikelihood()), QMap<int, int>(stat.weights()));
		}
		UDEBUG("time= %d ms (update likelihood and posterior)", time.restart());

		// Update statistics tool box
		if(_preferencesDialog->isCacheSavedInFigures() || _ui->statsToolBox->isVisible())
		{
			const std::map<std::string, float> & statistics = stat.data();
			std::string odomStr =  "Odometry/";
			for(std::map<std::string, float>::const_iterator iter = statistics.begin(); iter != statistics.end(); ++iter)
			{
				//ULOGGER_DEBUG("Updating stat \"%s\"", (*iter).first.c_str());
				if((*iter).first.size()<odomStr.size() || (*iter).first.substr(0, odomStr.size()).compare(odomStr)!=0)
				{
					_ui->statsToolBox->updateStat(QString((*iter).first.c_str()).replace('_', ' '), _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), (*iter).second, _preferencesDialog->isCacheSavedInFigures());
				}
			}
		}

		UDEBUG("time= %d ms (update stats toolbox)", time.restart());

		//======================
		// RGB-D Mapping stuff
		//======================
		_odometryCorrection = stat.mapCorrection();
		// update clouds
		if(stat.poses().size())
		{
			// update pose only if odometry is not received
			std::map<int, int> mapIds = _currentMapIds;
			std::map<int, Transform> groundTruth = _currentGTPosesMap;

			mapIds.insert(std::make_pair(stat.getLastSignatureData().id(), stat.getLastSignatureData().mapId()));
			if(!stat.getLastSignatureData().getGroundTruthPose().isNull() &&
				_cachedSignatures.contains(stat.getLastSignatureData().id()))
			{
				groundTruth.insert(std::make_pair(stat.getLastSignatureData().id(), stat.getLastSignatureData().getGroundTruthPose()));
			}

			if(_preferencesDialog->isPriorIgnored() &&
				_ui->graphicsView_graphView->getWorldMapRotation()==0.0f &&
				stat.getLastSignatureData().sensorData().gps().stamp()!=0.0 &&
				stat.poses().find(stat.getLastSignatureData().id())!=stat.poses().end())
			{
				float bearing = (float)((-(stat.getLastSignatureData().sensorData().gps().bearing()-90))*M_PI/180.0);
				float gpsRotationOffset = stat.poses().at(stat.getLastSignatureData().id()).theta()-bearing;
				_ui->graphicsView_graphView->setWorldMapRotation(gpsRotationOffset);
			}
			else if(!_preferencesDialog->isPriorIgnored() &&
					_ui->graphicsView_graphView->getWorldMapRotation() != 0.0f)
			{
				_ui->graphicsView_graphView->setWorldMapRotation(0.0f);
			}


			std::map<int, Transform> poses = stat.poses();

			UDEBUG("time= %d ms (update gt-gps stuff)", time.restart());

#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
			if(_preferencesDialog->isFramesShown())
			{
				_cloudViewer->addOrUpdateCoordinate("map_frame", Transform::getIdentity(), 0.5, false);
				_cloudViewer->addOrUpdateCoordinate("odom_frame", _odometryCorrection, 0.35, false);
				_cloudViewer->addOrUpdateLine("map_to_odom", Transform::getIdentity(), _odometryCorrection, qRgb(255, 128, 0), false, false);
			}
			else
			{
				_cloudViewer->removeLine("map_to_odom");
				_cloudViewer->removeCoordinate("odom_frame");
				_cloudViewer->removeCoordinate("map_frame");
			}
#endif

			UDEBUG("%d %d %d", poses.size(), poses.size()?poses.rbegin()->first:0, stat.refImageId());
			if(!_odometryReceived && poses.size() && poses.rbegin()->first == stat.refImageId())
			{
				if(poses.rbegin()->first == stat.getLastSignatureData().id())
				{
					if(stat.getLastSignatureData().sensorData().cameraModels().size() && stat.getLastSignatureData().sensorData().cameraModels()[0].isValidForProjection())
					{
						_cloudViewer->updateCameraFrustums(poses.rbegin()->second, stat.getLastSignatureData().sensorData().cameraModels());
					}
					else if(stat.getLastSignatureData().sensorData().stereoCameraModels().size() && stat.getLastSignatureData().sensorData().stereoCameraModels()[0].isValidForProjection())
					{
						_cloudViewer->updateCameraFrustums(poses.rbegin()->second, stat.getLastSignatureData().sensorData().stereoCameraModels());
					}
					else if(!stat.getLastSignatureData().sensorData().laserScanRaw().isEmpty() ||
							!stat.getLastSignatureData().sensorData().laserScanCompressed().isEmpty())
					{
						Transform scanLocalTransform;
						if(!stat.getLastSignatureData().sensorData().laserScanRaw().isEmpty())
						{
							scanLocalTransform = stat.getLastSignatureData().sensorData().laserScanRaw().localTransform();
						}
						else
						{
							scanLocalTransform = stat.getLastSignatureData().sensorData().laserScanCompressed().localTransform();
						}
						//fake frustum
						CameraModel model(
								2,
								2,
								2,
								1.5,
								scanLocalTransform*CameraModel::opticalRotation(),
								0,
								cv::Size(4,3));
						_cloudViewer->updateCameraFrustum(poses.rbegin()->second, model);
					}
				}

				_cloudViewer->updateCameraTargetPosition(poses.rbegin()->second);

				if(_ui->graphicsView_graphView->isVisible())
				{
					_ui->graphicsView_graphView->updateReferentialPosition(poses.rbegin()->second);
				}
			}

			if(_cachedSignatures.contains(0) && stat.refImageId()>0)
			{
				if(poses.find(stat.refImageId())!=poses.end())
				{
					poses.insert(std::make_pair(0, poses.at(stat.refImageId())));
					poses.erase(stat.refImageId());
				}
				if(groundTruth.find(stat.refImageId())!=groundTruth.end())
				{
					groundTruth.insert(std::make_pair(0, groundTruth.at(stat.refImageId())));
					groundTruth.erase(stat.refImageId());
				}
			}

			std::map<std::string, float> updateCloudSats;
			updateMapCloud(
					poses,
					stat.constraints(),
					mapIds,
					stat.labels(),
					groundTruth,
					stat.odomCachePoses(),
					stat.odomCacheConstraints(),
					false,
					&updateCloudSats);

			_odometryReceived = false;

			UDEBUG("time= %d ms (update map cloud)", time.restart());

			if(_preferencesDialog->isCacheSavedInFigures() || _ui->statsToolBox->isVisible())
			{
				for(std::map<std::string, float>::iterator iter=updateCloudSats.begin(); iter!=updateCloudSats.end(); ++iter)
				{
					_ui->statsToolBox->updateStat(iter->first.c_str(), _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), int(iter->second), _preferencesDialog->isCacheSavedInFigures());
				}
			}
		}

		if( _ui->graphicsView_graphView->isVisible())
		{
			// update posterior on the graph view
			if(_preferencesDialog->isPosteriorGraphView() &&
			   stat.posterior().size())
			{
				_ui->graphicsView_graphView->updatePosterior(stat.posterior());
			}
			else if(_preferencesDialog->isRGBDMode())
			{
				if(_preferencesDialog->isWordsCountGraphView() &&
						_cachedWordsCount.size())
				{
					_ui->graphicsView_graphView->updatePosterior(_cachedWordsCount, (float)_preferencesDialog->getKpMaxFeatures());
				}
				else if(_preferencesDialog->isLocalizationsCountGraphView() &&
						_cachedLocalizationsCount.size())
				{
					_ui->graphicsView_graphView->updatePosterior(_cachedLocalizationsCount, 1.0f);
				}
			}
			// update local path on the graph view
			_ui->graphicsView_graphView->updateLocalPath(stat.localPath());
			if(stat.localPath().size() == 0)
			{
				// clear the global path if set (goal reached)
				_ui->graphicsView_graphView->setGlobalPath(std::vector<std::pair<int, Transform> >());
			}
			// update current goal id
			if(stat.currentGoalId() > 0)
			{
				_ui->graphicsView_graphView->setCurrentGoalID(stat.currentGoalId(), uValue(stat.poses(), stat.currentGoalId(), Transform()));
			}
			UDEBUG("time= %d ms (update graph view)", time.restart());
		}

		if(_multiSessionLocWidget->isVisible())
		{
			_multiSessionLocWidget->updateView(signature, stat);
		}

		_cachedSignatures.remove(0); // remove tmp negative ids

		// keep only compressed data in cache
		if(_cachedSignatures.contains(stat.refImageId()))
		{
			Signature & s = *_cachedSignatures.find(stat.refImageId());
			_cachedMemoryUsage -= s.sensorData().getMemoryUsed();
			s.sensorData().clearRawData();
			s.sensorData().clearOccupancyGridRaw();
			_cachedMemoryUsage += s.sensorData().getMemoryUsed();
		}

		UDEBUG("time= %d ms (update cache)", time.restart());
	}
	else if(!stat.extended() && stat.loopClosureId()>0)
	{
		_ui->label_stats_loopClosuresDetected->setText(QString::number(_ui->label_stats_loopClosuresDetected->text().toInt() + 1));
		_ui->label_matchId->setText(QString("Match ID = %1 [%2]").arg(stat.loopClosureId()).arg(loopMapId));
	}
	else
	{
		_ui->label_matchId->clear();
	}
	float elapsedTime = static_cast<float>(totalTime.elapsed());
	UINFO("Updating GUI time = %fs", elapsedTime/1000.0f);
	if(_preferencesDialog->isCacheSavedInFigures() || _ui->statsToolBox->isVisible())
	{
		_ui->statsToolBox->updateStat("GUI/Refresh stats/ms", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), elapsedTime, _preferencesDialog->isCacheSavedInFigures());
	}
	if(_ui->actionAuto_screen_capture->isChecked() && !_autoScreenCaptureOdomSync)
	{
		this->captureScreen(_autoScreenCaptureRAM, _autoScreenCapturePNG);
	}

	if(!_preferencesDialog->isImagesKept())
	{
		_cachedSignatures.clear();
		_cachedMemoryUsage = 0;
		_cachedWordsCount.clear();
	}
	if(_preferencesDialog->isCacheSavedInFigures() || _ui->statsToolBox->isVisible())
	{
		_ui->statsToolBox->updateStat("GUI/Cache Data Size/MB", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), _cachedMemoryUsage/(1024*1024), _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("GUI/Cache Clouds Size/MB", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), _createdCloudsMemoryUsage/(1024*1024), _preferencesDialog->isCacheSavedInFigures());
#ifdef RTABMAP_OCTOMAP
		_ui->statsToolBox->updateStat("GUI/Octomap Size/MB", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), _octomap->octree()->memoryUsage()/(1024*1024), _preferencesDialog->isCacheSavedInFigures());
#endif
	}
	if(_state != kMonitoring && _state != kDetecting)
	{
		_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionDepth_Calibration->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
	}

	_processingStatistics = false;

	Q_EMIT(statsProcessed());
}

void MainWindow::updateMapCloud(
		const std::map<int, Transform> & posesIn,
		const std::multimap<int, Link> & constraints,
		const std::map<int, int> & mapIdsIn,
		const std::map<int, std::string> & labels,
		const std::map<int, Transform> & groundTruths, // ground truth should contain only valid transforms
		const std::map<int, Transform> & odomCachePoses,
		const std::multimap<int, Link> & odomCacheConstraints,
		bool verboseProgress,
		std::map<std::string, float> * stats)
{
	UTimer timer;
	std::map<int, Transform> nodePoses(posesIn.lower_bound(0), posesIn.end());
	UDEBUG("nodes=%d landmarks=%d constraints=%d mapIdsIn=%d labelsIn=%d",
			(int)nodePoses.size(), (int)(posesIn.size() - nodePoses.size()), (int)constraints.size(), (int)mapIdsIn.size(), (int)labels.size());
	if(posesIn.size())
	{
		_currentPosesMap = posesIn;
		_currentPosesMap.erase(0); // don't keep 0 if it is there
		_currentLinksMap = constraints;
		_currentMapIds = mapIdsIn;
		_currentLabels = labels;
		_currentGTPosesMap = groundTruths;
		_currentGTPosesMap.erase(0);
		if(_state != kMonitoring && _state != kDetecting)
		{
			_ui->actionPost_processing->setEnabled(_cachedSignatures.size() >= 2 && nodePoses.size() >= 2 && _currentLinksMap.size() >= 1);
			_ui->menuExport_poses->setEnabled(!_currentPosesMap.empty());
		}
		_ui->actionAnchor_clouds_to_ground_truth->setEnabled(!_currentGTPosesMap.empty());
	}

	// filter duplicated poses
	std::map<int, Transform> poses;
	std::map<int, int> mapIds;
	if(_preferencesDialog->isCloudFiltering() && nodePoses.size())
	{
		float radius = _preferencesDialog->getCloudFilteringRadius();
		float angle = _preferencesDialog->getCloudFilteringAngle()*CV_PI/180.0; // convert to rad
		bool hasZero = nodePoses.find(0) != nodePoses.end();
		if(hasZero)
		{
			std::map<int, Transform> posesInTmp = nodePoses;
			posesInTmp.erase(0);
			poses = rtabmap::graph::radiusPosesFiltering(posesInTmp, radius, angle);
		}
		else
		{
			poses = rtabmap::graph::radiusPosesFiltering(nodePoses, radius, angle);
		}
		for(std::map<int, Transform>::iterator iter= poses.begin(); iter!=poses.end(); ++iter)
		{
			std::map<int, int>::const_iterator jter = mapIdsIn.find(iter->first);
			if(jter!=mapIdsIn.end())
			{
				mapIds.insert(*jter);
			}
		}
		//keep 0
		if(hasZero)
		{
			poses.insert(*nodePoses.find(0));
		}

		if(verboseProgress)
		{
			_progressDialog->appendText(tr("Map update: %1 nodes shown of %2 (cloud filtering is on)").arg(poses.size()).arg(nodePoses.size()));
			QApplication::processEvents();
		}
	}
	else
	{
		poses = nodePoses;
		mapIds = mapIdsIn;
	}

	std::map<int, bool> posesMask;
	for(std::map<int, Transform>::const_iterator iter = nodePoses.begin(); iter!=nodePoses.end(); ++iter)
	{
		posesMask.insert(posesMask.end(), std::make_pair(iter->first, poses.find(iter->first) != poses.end()));
	}
	_ui->widget_mapVisibility->setMap(nodePoses, posesMask);

	if(groundTruths.size() && _ui->actionAnchor_clouds_to_ground_truth->isChecked())
	{
		for(std::map<int, Transform>::iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			std::map<int, Transform>::const_iterator gtIter = groundTruths.find(iter->first);
			if(gtIter!=groundTruths.end())
			{
				iter->second = gtIter->second;
			}
			else
			{
				UWARN("Not found ground truth pose for node %d", iter->first);
			}
		}
	}
	else if(_currentGTPosesMap.size() == 0)
	{
		_ui->actionAnchor_clouds_to_ground_truth->setChecked(false);
	}

	int maxNodes = uStr2Int(_preferencesDialog->getParameter(Parameters::kGridGlobalMaxNodes()));
	int altitudeDelta = uStr2Int(_preferencesDialog->getParameter(Parameters::kGridGlobalAltitudeDelta()));
	if((maxNodes > 0 || altitudeDelta>0.0) && poses.size()>1)
	{
		Transform currentPose = poses.rbegin()->second;
		if(poses.find(0) != poses.end())
		{
			currentPose = poses.at(0);
		}

		std::map<int, Transform> nearestPoses;
		if(maxNodes > 0)
		{
			std::map<int, float> nodes = graph::findNearestNodes(currentPose, poses, 0, 0, maxNodes);
			for(std::map<int, float>::iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
			{
				if(altitudeDelta<=0.0 ||
				   fabs(poses.at(iter->first).z()-currentPose.z())<altitudeDelta)
				{
					nearestPoses.insert(*poses.find(iter->first));
				}
			}
		}
		else // altitudeDelta>0.0
		{
			for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				if(fabs(iter->second.z()-currentPose.z())<altitudeDelta)
				{
					nearestPoses.insert(*iter);
				}
			}
		}

		//add zero...
		if(poses.find(0) != poses.end())
		{
			nearestPoses.insert(*poses.find(0));
		}
		poses=nearestPoses;
	}

	// Map updated! regenerate the assembled cloud, last pose is the new one
	UDEBUG("Update map with %d locations", poses.size());
	QMap<std::string, Transform> viewerClouds = _cloudViewer->getAddedClouds();
	std::set<std::string> viewerLines = _cloudViewer->getAddedLines();
	int i=1;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(!iter->second.isNull())
		{
			std::string cloudName = uFormat("cloud%d", iter->first);

			if(iter->first == 0)
			{
				viewerClouds.remove(cloudName);
				_cloudViewer->removeCloud(cloudName);
			}

			// 3d point cloud
			bool update3dCloud = _cloudViewer->isVisible() && _preferencesDialog->isCloudsShown(0);
			if(update3dCloud)
			{
				// update cloud
				if(viewerClouds.contains(cloudName))
				{
					// Update only if the pose has changed
					Transform tCloud;
					_cloudViewer->getPose(cloudName, tCloud);
					if(tCloud.isNull() || iter->second != tCloud)
					{
						if(!_cloudViewer->updateCloudPose(cloudName, iter->second))
						{
							UERROR("Updating pose cloud %d failed!", iter->first);
						}
					}
					_cloudViewer->setCloudVisibility(cloudName, (_cloudViewer->isVisible() && _preferencesDialog->isCloudsShown(0)));
					_cloudViewer->setCloudColorIndex(cloudName, _preferencesDialog->getCloudColorScheme(0));
					_cloudViewer->setCloudOpacity(cloudName, _preferencesDialog->getCloudOpacity(0));
					_cloudViewer->setCloudPointSize(cloudName, _preferencesDialog->getCloudPointSize(0));
				}
				else if(_cachedEmptyClouds.find(iter->first) == _cachedEmptyClouds.end() &&
						_cachedClouds.find(iter->first) == _cachedClouds.end() &&
						_cachedSignatures.contains(iter->first))
				{
					std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> createdCloud = this->createAndAddCloudToMap(iter->first, iter->second, uValue(mapIds, iter->first, -1));
					if(_cloudViewer->getAddedClouds().contains(cloudName))
					{
						_cloudViewer->setCloudVisibility(cloudName.c_str(), _cloudViewer->isVisible() && _preferencesDialog->isCloudsShown(0));
					}
				}
			}
			else if(viewerClouds.contains(cloudName))
			{
				_cloudViewer->setCloudVisibility(cloudName.c_str(), false);
			}

			// 2d point cloud
			std::string scanName = uFormat("scan%d", iter->first);
			if(iter->first == 0)
			{
				viewerClouds.remove(scanName);
				_cloudViewer->removeCloud(scanName);
			}
			if(_cloudViewer->isVisible() && _preferencesDialog->isScansShown(0))
			{
				if(viewerClouds.contains(scanName))
				{
					// Update only if the pose has changed
					Transform tScan;
					_cloudViewer->getPose(scanName, tScan);
					if(tScan.isNull() || iter->second != tScan)
					{
						if(!_cloudViewer->updateCloudPose(scanName, iter->second))
						{
							UERROR("Updating pose scan %d failed!", iter->first);
						}
					}
					_cloudViewer->setCloudVisibility(scanName, _preferencesDialog->isScansShown(0));
					_cloudViewer->setCloudColorIndex(scanName, _preferencesDialog->getScanColorScheme(0));
					_cloudViewer->setCloudOpacity(scanName, _preferencesDialog->getScanOpacity(0));
					_cloudViewer->setCloudPointSize(scanName, _preferencesDialog->getScanPointSize(0));
				}
				else if(_cachedSignatures.contains(iter->first))
				{
					QMap<int, Signature>::iterator jter = _cachedSignatures.find(iter->first);
					if(!jter->sensorData().laserScanCompressed().isEmpty() || !jter->sensorData().laserScanRaw().isEmpty())
					{
						this->createAndAddScanToMap(iter->first, iter->second, uValue(mapIds, iter->first, -1));
					}
				}
			}
			else if(viewerClouds.contains(scanName))
			{
				_cloudViewer->setCloudVisibility(scanName.c_str(), false);
			}

			// occupancy grids
			bool updateGridMap =
					((_ui->graphicsView_graphView->isVisible() && _ui->graphicsView_graphView->isGridMapVisible()) ||
					 (_cloudViewer->isVisible() && _preferencesDialog->getGridMapShown())) &&
					_occupancyGrid->addedNodes().find(iter->first) == _occupancyGrid->addedNodes().end();
			bool updateOctomap = false;
#ifdef RTABMAP_OCTOMAP
			updateOctomap =
					_cloudViewer->isVisible() &&
					_preferencesDialog->isOctomapUpdated() &&
					_octomap->addedNodes().find(iter->first) == _octomap->addedNodes().end();
#endif
			if(updateGridMap || updateOctomap)
			{
				QMap<int, Signature>::iterator jter = _cachedSignatures.find(iter->first);
				if(jter!=_cachedSignatures.end() && jter->sensorData().gridCellSize() > 0.0f)
				{
					cv::Mat ground;
					cv::Mat obstacles;
					cv::Mat empty;

					jter->sensorData().uncompressDataConst(0, 0, 0, 0, &ground, &obstacles, &empty);

					_occupancyGrid->addToCache(iter->first, ground, obstacles, empty);

#ifdef RTABMAP_OCTOMAP
					if(updateOctomap)
					{
						if((ground.empty() || ground.channels() > 2) &&
						   (obstacles.empty() || obstacles.channels() > 2))
						{
							cv::Point3f viewpoint = jter->sensorData().gridViewPoint();
							_octomap->addToCache(iter->first, ground, obstacles, empty, viewpoint);
						}
						else if(!ground.empty() || !obstacles.empty())
						{
							UWARN("Node %d: Cannot update octomap with 2D occupancy grids.", iter->first);
						}
					}
#endif
				}
			}

			// 3d features
			std::string featuresName = uFormat("features%d", iter->first);
			if(iter->first == 0)
			{
				viewerClouds.remove(featuresName);
				_cloudViewer->removeCloud(featuresName);
			}
			if(_cloudViewer->isVisible() && _preferencesDialog->isFeaturesShown(0))
			{
				if(viewerClouds.contains(featuresName))
				{
					// Update only if the pose has changed
					Transform tFeatures;
					_cloudViewer->getPose(featuresName, tFeatures);
					if(tFeatures.isNull() || iter->second != tFeatures)
					{
						if(!_cloudViewer->updateCloudPose(featuresName, iter->second))
						{
							UERROR("Updating pose features %d failed!", iter->first);
						}
					}
					_cloudViewer->setCloudVisibility(featuresName, _preferencesDialog->isFeaturesShown(0));
					_cloudViewer->setCloudPointSize(featuresName, _preferencesDialog->getFeaturesPointSize(0));
				}
				else if(_cachedSignatures.contains(iter->first))
				{
					QMap<int, Signature>::iterator jter = _cachedSignatures.find(iter->first);
					if(!jter->getWords3().empty())
					{
						this->createAndAddFeaturesToMap(iter->first, iter->second, uValue(mapIds, iter->first, -1));
					}
				}
			}
			else if(viewerClouds.contains(featuresName))
			{
				_cloudViewer->setCloudVisibility(featuresName.c_str(), false);
			}

			// Gravity arrows
			std::string gravityName = uFormat("gravity%d", iter->first);
			if(iter->first == 0)
			{
				viewerLines.erase(gravityName);
				_cloudViewer->removeLine(gravityName);
			}
			if(_cloudViewer->isVisible() && _preferencesDialog->isIMUGravityShown(0))
			{
				std::multimap<int, Link>::const_iterator linkIter = graph::findLink(constraints, iter->first, iter->first, false, Link::kGravity);
				if(linkIter != constraints.end())
				{
					Transform gravityT = linkIter->second.transform();
					Eigen::Vector3f gravity(0,0,-_preferencesDialog->getIMUGravityLength(0));
					gravity = (gravityT.rotation()*(iter->second).rotation().inverse()).toEigen3f()*gravity;
					_cloudViewer->addOrUpdateLine(gravityName, iter->second, (iter->second).translation()*Transform(gravity[0], gravity[1], gravity[2], 0, 0, 0)*iter->second.rotation().inverse(), Qt::yellow, false, false);
				}
			}
			else if(viewerLines.find(gravityName)!=viewerLines.end())
			{
				_cloudViewer->removeLine(gravityName.c_str());
			}

			if(verboseProgress)
			{
				_progressDialog->appendText(tr("Updated cloud %1 (%2/%3)").arg(iter->first).arg(i).arg(poses.size()));
				_progressDialog->incrementStep();
				if(poses.size() < 200 || i % 100 == 0)
				{
					QApplication::processEvents();
					if(_progressCanceled)
					{
						break;
					}
				}
			}
		}

		++i;
	}

	//remove not used clouds
	for(QMap<std::string, Transform>::iterator iter = viewerClouds.begin(); iter!=viewerClouds.end(); ++iter)
	{
		std::list<std::string> splitted = uSplitNumChar(iter.key());
		int id = 0;
		if(splitted.size() == 2)
		{
			id = std::atoi(splitted.back().c_str());
			if(poses.find(id) == poses.end())
			{
				if(_cloudViewer->getCloudVisibility(iter.key()))
				{
					UDEBUG("Hide %s", iter.key().c_str());
					_cloudViewer->setCloudVisibility(iter.key(), false);
				}
			}
		}
	}
	// remove not used gravity lines
	for(std::set<std::string>::iterator iter = viewerLines.begin(); iter!=viewerLines.end(); ++iter)
	{
		std::list<std::string> splitted = uSplitNumChar(*iter);
		int id = 0;
		if(splitted.size() == 2)
		{
			id = std::atoi(splitted.back().c_str());
			if(poses.find(id) == poses.end())
			{
				UDEBUG("Remove %s", iter->c_str());
				_cloudViewer->removeLine(*iter);
			}
		}
	}

	UDEBUG("");
	if(stats)
	{
		stats->insert(std::make_pair("GUI/RGB-D cloud/ms", (float)timer.restart()*1000.0f));
	}

	// update 3D graphes (show all poses)
	_cloudViewer->removeAllGraphs();
	_cloudViewer->removeCloud("graph_nodes");
	if(!_preferencesDialog->isFrustumsShown(0))
	{
		QMap<std::string, Transform> addedFrustums = _cloudViewer->getAddedFrustums();
		for(QMap<std::string, Transform>::iterator iter = addedFrustums.begin(); iter!=addedFrustums.end(); ++iter)
		{
			std::list<std::string> splitted = uSplitNumChar(iter.key());
			if(splitted.size() == 2)
			{
				if((splitted.front().compare("f_") == 0 || splitted.front().compare("f_gt_") == 0))
				{
					_cloudViewer->removeFrustum(iter.key());
				}
			}
		}
	}

	Transform mapToGt = Transform::getIdentity();
	if(_preferencesDialog->isGroundTruthAligned() && _currentGTPosesMap.size())
	{
		mapToGt = alignPosesToGroundTruth(_currentPosesMap, _currentGTPosesMap).inverse();
	}

	std::map<int, Transform> posesWithOdomCache;

	if(_ui->graphicsView_graphView->isVisible() ||
	   ((_preferencesDialog->isGraphsShown() || _preferencesDialog->isFrustumsShown(0)) && _currentPosesMap.size()))
	{
		posesWithOdomCache = posesIn;
		for(std::map<int, Transform>::const_iterator iter=odomCachePoses.begin(); iter!=odomCachePoses.end(); ++iter)
		{
			posesWithOdomCache.insert(std::make_pair(iter->first, _odometryCorrection*iter->second));
		}
	}

	if((_preferencesDialog->isGraphsShown() || _preferencesDialog->isFrustumsShown(0)) && _currentPosesMap.size())
	{
		UTimer timerGraph;
		// Find all graphs
		std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > graphs;
		for(std::map<int, Transform>::iterator iter=posesWithOdomCache.lower_bound(1); iter!=posesWithOdomCache.end(); ++iter)
		{
			int mapId = uValue(_currentMapIds, iter->first, -1);

			if(_preferencesDialog->isGraphsShown())
			{
				//edges
				std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator kter = graphs.find(mapId);
				if(kter == graphs.end())
				{
					kter = graphs.insert(std::make_pair(mapId, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>))).first;
				}
				pcl::PointXYZ pt(iter->second.x(), iter->second.y(), iter->second.z());
				kter->second->push_back(pt);
			}

			// get local transforms for frustums on the graph
			if(_preferencesDialog->isFrustumsShown(0))
			{
				std::string frustumId = uFormat("f_%d", iter->first);
				if(_cloudViewer->getAddedFrustums().contains(frustumId))
				{
					_cloudViewer->updateFrustumPose(frustumId, iter->second);
				}
				else if(_cachedSignatures.contains(iter->first))
				{
					const Signature & s = _cachedSignatures.value(iter->first);
					// Supporting only one frustum per node
					if(s.sensorData().cameraModels().size() == 1 || s.sensorData().stereoCameraModels().size()==1)
					{
						const CameraModel & model = s.sensorData().stereoCameraModels().size()?s.sensorData().stereoCameraModels()[0].left():s.sensorData().cameraModels()[0];
						Transform t = model.localTransform();
						if(!t.isNull())
						{
							QColor color = (Qt::GlobalColor)((mapId+3) % 12 + 7 );
							_cloudViewer->addOrUpdateFrustum(frustumId, iter->second, t, _cloudViewer->getFrustumScale(), color, model.fovX(), model.fovY());

							if(_currentGTPosesMap.find(iter->first)!=_currentGTPosesMap.end())
							{
								std::string gtFrustumId = uFormat("f_gt_%d", iter->first);
								color = Qt::gray;
								_cloudViewer->addOrUpdateFrustum(gtFrustumId, _currentGTPosesMap.at(iter->first), t, _cloudViewer->getFrustumScale(), color, model.fovX(), model.fovY());
							}
						}
					}
				}
			}
		}

		//Ground truth graph?
		for(std::map<int, Transform>::iterator iter=_currentGTPosesMap.begin(); iter!=_currentGTPosesMap.end(); ++iter)
		{
			int mapId = -100;

			if(_preferencesDialog->isGraphsShown())
			{
				//edges
				std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator kter = graphs.find(mapId);
				if(kter == graphs.end())
				{
					kter = graphs.insert(std::make_pair(mapId, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>))).first;
				}
				Transform t = mapToGt*iter->second;
				pcl::PointXYZ pt(t.x(), t.y(), t.z());
				kter->second->push_back(pt);
			}
		}

		// add graphs
		for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter=graphs.begin(); iter!=graphs.end(); ++iter)
		{
			QColor color = Qt::gray;
			if(iter->first >= 0)
			{
				color = (Qt::GlobalColor)((iter->first+3) % 12 + 7 );
			}
			_cloudViewer->addOrUpdateGraph(uFormat("graph_%d", iter->first), iter->second, color);
		}

		if(_preferencesDialog->isFrustumsShown(0))
		{
			QMap<std::string, Transform> addedFrustums = _cloudViewer->getAddedFrustums();
			UDEBUG("remove not used frustums");
			for(QMap<std::string, Transform>::iterator iter = addedFrustums.begin(); iter!=addedFrustums.end(); ++iter)
			{
				std::list<std::string> splitted = uSplitNumChar(iter.key());
				if(splitted.size() == 2)
				{
					int id = std::atoi(splitted.back().c_str());
					if((splitted.front().compare("f_") == 0 || splitted.front().compare("f_gt_") == 0) &&
						posesWithOdomCache.find(id) == posesWithOdomCache.end())
					{
						_cloudViewer->removeFrustum(iter.key());
					}
				}
			}
		}

		UDEBUG("timerGraph=%fs", timerGraph.ticks());
	}

	UDEBUG("labels.size()=%d", (int)labels.size());

	// Update labels
	_cloudViewer->removeAllTexts();
	if(_preferencesDialog->isLabelsShown() && labels.size())
	{
		for(std::map<int, std::string>::const_iterator iter=labels.begin(); iter!=labels.end(); ++iter)
		{
			if(nodePoses.find(iter->first)!=nodePoses.end())
			{
				int mapId = uValue(mapIdsIn, iter->first, -1);
				QColor color = Qt::gray;
				if(mapId >= 0)
				{
					color = (Qt::GlobalColor)((mapId+3) % 12 + 7 );
				}
				_cloudViewer->addOrUpdateText(
						std::string("label_") + uNumber2Str(iter->first),
						iter->second,
						_currentPosesMap.at(iter->first),
						0.1,
						color);
			}
		}
	}

	UDEBUG("");
	if(stats)
	{
		stats->insert(std::make_pair("GUI/Graph Update/ms", (float)timer.restart()*1000.0f));
	}

#ifdef RTABMAP_OCTOMAP
	_cloudViewer->removeOctomap();
	_cloudViewer->removeCloud("octomap_cloud");
	if(_preferencesDialog->isOctomapUpdated())
	{
		UDEBUG("");
		UTimer time;
		_octomap->update(poses);
		UINFO("Octomap update time = %fs", time.ticks());
	}
	if(stats)
	{
		stats->insert(std::make_pair("GUI/Octomap Update/ms", (float)timer.restart()*1000.0f));
	}
	if(_preferencesDialog->isOctomapShown())
	{
		UDEBUG("");
		UTimer time;
		if(_preferencesDialog->getOctomapRenderingType() > 0)
		{
			_cloudViewer->addOctomap(_octomap, _preferencesDialog->getOctomapTreeDepth(), _preferencesDialog->getOctomapRenderingType()>1);
		}
		else
		{
			pcl::IndicesPtr obstacles(new std::vector<int>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = _octomap->createCloud(_preferencesDialog->getOctomapTreeDepth(), obstacles.get());
			if(obstacles->size())
			{
				_cloudViewer->addCloud("octomap_cloud", cloud);
				_cloudViewer->setCloudPointSize("octomap_cloud", _preferencesDialog->getOctomapPointSize());
			}
		}
		UINFO("Octomap show 3d map time = %fs", time.ticks());
	}
	UDEBUG("");
	if(stats)
	{
		stats->insert(std::make_pair("GUI/Octomap Rendering/ms", (float)timer.restart()*1000.0f));
	}
#endif

	// Add landmarks to 3D Map view
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
	_cloudViewer->removeAllCoordinates("landmark_");
#endif
	if(_preferencesDialog->isLandmarksShown())
	{
		for(std::map<int, Transform>::const_iterator iter=posesIn.begin(); iter!=posesIn.end() && iter->first<0; ++iter)
		{
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
			_cloudViewer->addOrUpdateCoordinate(uFormat("landmark_%d", -iter->first), iter->second, _preferencesDialog->landmarkVisSize()>0.0?_preferencesDialog->landmarkVisSize():_preferencesDialog->getMarkerLength()<=0?0.1:_preferencesDialog->getMarkerLength()/2.0, false);
#endif
			if(_preferencesDialog->isLabelsShown())
			{
				std::string num = uNumber2Str(-iter->first);
				_cloudViewer->addOrUpdateText(
						std::string("landmark_str_") + num,
						num,
						iter->second,
						0.1,
						Qt::yellow);
			}
		}
	}

	// Update occupancy grid map in 3D map view and graph view
	if(_ui->graphicsView_graphView->isVisible())
	{
		std::multimap<int, Link> constraintsWithOdomCache;
		constraintsWithOdomCache = constraints;
		constraintsWithOdomCache.insert(odomCacheConstraints.begin(), odomCacheConstraints.end());
		_ui->graphicsView_graphView->updateGraph(posesWithOdomCache, constraintsWithOdomCache, mapIdsIn, std::map<int, int>(), uKeysSet(odomCachePoses));
		if(_preferencesDialog->isGroundTruthAligned() && !mapToGt.isIdentity())
		{
			std::map<int, Transform> gtPoses = _currentGTPosesMap;
			for(std::map<int, Transform>::iterator iter=gtPoses.begin(); iter!=gtPoses.end(); ++iter)
			{
				iter->second = mapToGt * iter->second;
			}
			_ui->graphicsView_graphView->updateGTGraph(gtPoses);
		}
		else
		{
			_ui->graphicsView_graphView->updateGTGraph(_currentGTPosesMap);
		}
	}
	cv::Mat map8U;
	if((_ui->graphicsView_graphView->isVisible() || _preferencesDialog->getGridMapShown()))
	{
		float xMin, yMin;
		float resolution = _occupancyGrid->getCellSize();
		cv::Mat map8S;
#ifdef RTABMAP_OCTOMAP
		if(_preferencesDialog->isOctomap2dGrid())
		{
			map8S = _octomap->createProjectionMap(xMin, yMin, resolution, 0, _preferencesDialog->getOctomapTreeDepth());

		}
		else
#endif
		{
			if(_occupancyGrid->addedNodes().size() || _occupancyGrid->cacheSize()>0)
			{
				_occupancyGrid->update(poses);
			}
			if(stats)
			{
				stats->insert(std::make_pair("GUI/Grid Update/ms", (float)timer.restart()*1000.0f));
			}
			map8S = _occupancyGrid->getMap(xMin, yMin);
		}
		if(!map8S.empty())
		{
			//convert to gray scaled map
			map8U = util3d::convertMap2Image8U(map8S);

			if(_preferencesDialog->getGridMapShown())
			{
				float opacity = _preferencesDialog->getGridMapOpacity();
				_cloudViewer->addOccupancyGridMap(map8U, resolution, xMin, yMin, opacity);
			}
			if(_ui->graphicsView_graphView->isVisible())
			{
				_ui->graphicsView_graphView->updateMap(map8U, resolution, xMin, yMin);
			}
		}
	}
	_ui->graphicsView_graphView->update();

	UDEBUG("");
	if(stats)
	{
		stats->insert(std::make_pair("GUI/Grid Rendering/ms", (float)timer.restart()*1000.0f));
	}

	if(!_preferencesDialog->getGridMapShown())
	{
		UDEBUG("");
		_cloudViewer->removeOccupancyGridMap();
	}

	if(viewerClouds.contains("cloudOdom"))
	{
		if(!_preferencesDialog->isCloudsShown(1))
		{
			UDEBUG("");
			_cloudViewer->setCloudVisibility("cloudOdom", false);
		}
		else
		{
			UDEBUG("");
			_cloudViewer->updateCloudPose("cloudOdom", _odometryCorrection);
			_cloudViewer->setCloudColorIndex("cloudOdom", _preferencesDialog->getCloudColorScheme(1));
			_cloudViewer->setCloudOpacity("cloudOdom", _preferencesDialog->getCloudOpacity(1));
			_cloudViewer->setCloudPointSize("cloudOdom", _preferencesDialog->getCloudPointSize(1));
		}
	}
	if(viewerClouds.contains("scanOdom"))
	{
		if(!_preferencesDialog->isScansShown(1))
		{
			UDEBUG("");
			_cloudViewer->setCloudVisibility("scanOdom", false);
		}
		else
		{
			UDEBUG("");
			_cloudViewer->updateCloudPose("scanOdom", _odometryCorrection);
			_cloudViewer->setCloudColorIndex("scanOdom", _preferencesDialog->getScanColorScheme(1));
			_cloudViewer->setCloudOpacity("scanOdom", _preferencesDialog->getScanOpacity(1));
			_cloudViewer->setCloudPointSize("scanOdom", _preferencesDialog->getScanPointSize(1));
		}
	}
	if(viewerClouds.contains("scanMapOdom"))
	{
		if(!_preferencesDialog->isScansShown(1))
		{
			UDEBUG("");
			_cloudViewer->setCloudVisibility("scanMapOdom", false);
		}
		else if(_cloudViewer->getBackgroundColor() != Qt::darkRed) // not lost
		{
			UDEBUG("");
			_cloudViewer->updateCloudPose("scanMapOdom", _odometryCorrection);
			_cloudViewer->setCloudColorIndex("scanMapOdom", _preferencesDialog->getScanColorScheme(1));
			_cloudViewer->setCloudOpacity("scanMapOdom", _preferencesDialog->getScanOpacity(1));
			_cloudViewer->setCloudPointSize("scanMapOdom", _preferencesDialog->getScanPointSize(1));
		}
	}
	if(viewerClouds.contains("featuresOdom"))
	{
		if(!_preferencesDialog->isFeaturesShown(1))
		{
			UDEBUG("");
			_cloudViewer->setCloudVisibility("featuresOdom", false);
		}
		else if(_cloudViewer->getBackgroundColor() != Qt::darkRed) // not lost
		{
			UDEBUG("");
			_cloudViewer->updateCloudPose("featuresOdom", _odometryCorrection);
			_cloudViewer->setCloudPointSize("featuresOdom", _preferencesDialog->getFeaturesPointSize(1));
		}
	}

	// activate actions
	if(_state != kMonitoring && _state != kDetecting)
	{
		_ui->actionSave_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionView_high_res_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(!_occupancyGrid->addedNodes().empty());
#ifdef RTABMAP_OCTOMAP
		_ui->actionExport_octomap->setEnabled(_octomap->octree()->size());
#else
		_ui->actionExport_octomap->setEnabled(false);
#endif
	}

	UDEBUG("");
	_cloudViewer->refreshView();
	UDEBUG("");
}

std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> MainWindow::createAndAddCloudToMap(int nodeId, const Transform & pose, int mapId)
{
	UASSERT(!pose.isNull());
	std::string cloudName = uFormat("cloud%d", nodeId);
	std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> outputPair;
	if(_cloudViewer->getAddedClouds().contains(cloudName))
	{
		UERROR("Cloud %d already added to map.", nodeId);
		return outputPair;
	}

	QMap<int, Signature>::iterator iter = _cachedSignatures.find(nodeId);
	if(iter == _cachedSignatures.end())
	{
		UERROR("Node %d is not in the cache.", nodeId);
		return outputPair;
	}

	UASSERT(_cachedClouds.find(nodeId) == _cachedClouds.end());

	if((!iter->sensorData().imageCompressed().empty() || !iter->sensorData().imageRaw().empty()) &&
	   (!iter->sensorData().depthOrRightCompressed().empty() || !iter->sensorData().depthOrRightRaw().empty()))
	{
		cv::Mat image, depth;
		SensorData data = iter->sensorData();
		data.uncompressData(&image, &depth, 0);
		ParametersMap allParameters = _preferencesDialog->getAllParameters();
		bool rectifyOnlyFeatures = Parameters::defaultRtabmapRectifyOnlyFeatures();
		bool imagesAlreadyRectified = Parameters::defaultRtabmapImagesAlreadyRectified();
		Parameters::parse(allParameters, Parameters::kRtabmapRectifyOnlyFeatures(), rectifyOnlyFeatures);
		Parameters::parse(allParameters, Parameters::kRtabmapImagesAlreadyRectified(), imagesAlreadyRectified);
		if(rectifyOnlyFeatures && !imagesAlreadyRectified)
		{
			if(data.cameraModels().size())
			{
				UTimer time;
				// Note that only RGB image is rectified, the depth image is assumed to be already registered to rectified RGB camera.
				UASSERT(int((data.imageRaw().cols/data.cameraModels().size())*data.cameraModels().size()) == data.imageRaw().cols);
				int subImageWidth = data.imageRaw().cols/data.cameraModels().size();
				cv::Mat rectifiedImages = data.imageRaw().clone();
				bool initRectMaps = _rectCameraModels.empty();
				if(initRectMaps)
				{
					_rectCameraModels.resize(data.cameraModels().size());
				}
				for(unsigned int i=0; i<data.cameraModels().size(); ++i)
				{
					if(data.cameraModels()[i].isValidForRectification())
					{
						if(initRectMaps)
						{
							_rectCameraModels[i] = data.cameraModels()[i];
							if(!_rectCameraModels[i].isRectificationMapInitialized())
							{
								UWARN("Initializing rectification maps for camera %d (only done for the first image received)...", i);
								_rectCameraModels[i].initRectificationMap();
								UWARN("Initializing rectification maps for camera %d (only done for the first image received)... done!", i);
							}
						}
						UASSERT(_rectCameraModels[i].imageWidth() == data.cameraModels()[i].imageWidth() &&
								_rectCameraModels[i].imageHeight() == data.cameraModels()[i].imageHeight());
						cv::Mat rectifiedImage = _rectCameraModels[i].rectifyImage(cv::Mat(data.imageRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows)));
						rectifiedImage.copyTo(cv::Mat(rectifiedImages, cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows)));
					}
					else
					{
						UWARN("Camera %d of data %d is not valid for rectification (%dx%d).",
								i, data.id(),
								data.cameraModels()[i].imageWidth(),
								data.cameraModels()[i].imageHeight());
					}
				}
				UINFO("Time rectification: %fs", time.ticks());
				data.setRGBDImage(rectifiedImages, data.depthOrRightRaw(), data.cameraModels());
				image = rectifiedImages;
			}
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		pcl::IndicesPtr indices(new std::vector<int>);
		UASSERT_MSG(nodeId == 0 || nodeId == data.id(), uFormat("nodeId=%d data.id()=%d", nodeId, data.id()).c_str());

		// Create organized cloud
		cloud = util3d::cloudRGBFromSensorData(data,
				_preferencesDialog->getCloudDecimation(0),
				_preferencesDialog->getCloudMaxDepth(0),
				_preferencesDialog->getCloudMinDepth(0),
				indices.get(),
				allParameters,
				_preferencesDialog->getCloudRoiRatios(0));

		// view point
		Eigen::Vector3f viewPoint(0.0f,0.0f,0.0f);
		if(data.cameraModels().size() && !data.cameraModels()[0].localTransform().isNull())
		{
			viewPoint[0] = data.cameraModels()[0].localTransform().x();
			viewPoint[1] = data.cameraModels()[0].localTransform().y();
			viewPoint[2] = data.cameraModels()[0].localTransform().z();
		}
		else if(data.stereoCameraModels().size() && !data.stereoCameraModels()[0].localTransform().isNull())
		{
			viewPoint[0] = data.stereoCameraModels()[0].localTransform().x();
			viewPoint[1] = data.stereoCameraModels()[0].localTransform().y();
			viewPoint[2] = data.stereoCameraModels()[0].localTransform().z();
		}

		// filtering pipeline
		if(indices->size() && _preferencesDialog->getVoxel() > 0.0)
		{
			cloud = util3d::voxelize(cloud, indices, _preferencesDialog->getVoxel());
			//generate indices for all points (they are all valid)
			indices->resize(cloud->size());
			for(unsigned int i=0; i<cloud->size(); ++i)
			{
				indices->at(i) = i;
			}
		}

		// Do ceiling/floor filtering
		if(indices->size() &&
		   (_preferencesDialog->getFloorFilteringHeight() != 0.0 ||
		   _preferencesDialog->getCeilingFilteringHeight() != 0.0))
		{
			// perform in /map frame
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTransformed = util3d::transformPointCloud(cloud, pose);
			indices = rtabmap::util3d::passThrough(
					cloudTransformed,
					indices,
					"z",
					_preferencesDialog->getFloorFilteringHeight()==0.0?(float)std::numeric_limits<int>::min():_preferencesDialog->getFloorFilteringHeight(),
					_preferencesDialog->getCeilingFilteringHeight()==0.0?(float)std::numeric_limits<int>::max():_preferencesDialog->getCeilingFilteringHeight());
		}

		// Do radius filtering after voxel filtering ( a lot faster)
		if(indices->size() &&
		   _preferencesDialog->getNoiseRadius() > 0.0 &&
		   _preferencesDialog->getNoiseMinNeighbors() > 0)
		{
			indices = rtabmap::util3d::radiusFiltering(
					cloud,
					indices,
					_preferencesDialog->getNoiseRadius(),
					_preferencesDialog->getNoiseMinNeighbors());
		}

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		if(_preferencesDialog->isSubtractFiltering() &&
		   _preferencesDialog->getSubtractFilteringRadius() > 0.0 &&
		   nodeId > 0)
		{
			pcl::IndicesPtr beforeFiltering = indices;
			if(	cloud->size() &&
				_previousCloud.first>0 &&
				_previousCloud.second.first.first.get() != 0 &&
				_previousCloud.second.second.get() != 0 &&
				_previousCloud.second.second->size() &&
				_currentPosesMap.find(_previousCloud.first) != _currentPosesMap.end())
			{
				UTimer time;

				rtabmap::Transform t = pose.inverse() * _currentPosesMap.at(_previousCloud.first);

				//UWARN("saved new.pcd and old.pcd");
				//pcl::io::savePCDFile("new.pcd", *cloud, *indices);
				//pcl::io::savePCDFile("old.pcd", *previousCloud, *_previousCloud.second.second);

				if(_preferencesDialog->isSubtractFiltering())
				{
					if(_preferencesDialog->getSubtractFilteringAngle() > 0.0f)
					{
						//normals required
						if(_preferencesDialog->getNormalKSearch() > 0 || _preferencesDialog->getNormalRadiusSearch() > 0)
						{
							pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, indices, _preferencesDialog->getNormalKSearch(), _preferencesDialog->getNormalRadiusSearch(), viewPoint);
							pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);
						}
						else
						{
							UWARN("Cloud subtraction with angle filtering is activated but "
								  "cloud normal K search is 0. Subtraction is done with angle.");
						}
					}

					if(cloudWithNormals->size() &&
						_previousCloud.second.first.second.get() &&
						_previousCloud.second.first.second->size())
					{
						pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr previousCloud = rtabmap::util3d::transformPointCloud(_previousCloud.second.first.second, t);
						indices = rtabmap::util3d::subtractFiltering(
								cloudWithNormals,
								indices,
								previousCloud,
								_previousCloud.second.second,
								_preferencesDialog->getSubtractFilteringRadius(),
								_preferencesDialog->getSubtractFilteringAngle(),
								_preferencesDialog->getSubtractFilteringMinPts());
					}
					else
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr previousCloud = rtabmap::util3d::transformPointCloud(_previousCloud.second.first.first, t);
						indices = rtabmap::util3d::subtractFiltering(
								cloud,
								indices,
								previousCloud,
								_previousCloud.second.second,
								_preferencesDialog->getSubtractFilteringRadius(),
								_preferencesDialog->getSubtractFilteringMinPts());
					}


					UINFO("Time subtract filtering %d from %d -> %d (%fs)",
							(int)_previousCloud.second.second->size(),
							(int)beforeFiltering->size(),
							(int)indices->size(),
							time.ticks());
				}
			}
			// keep all indices for next subtraction
			_previousCloud.first = nodeId;
			_previousCloud.second.first.first = cloud;
			_previousCloud.second.first.second = cloudWithNormals;
			_previousCloud.second.second = beforeFiltering;
		}

		if(indices->size())
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr output;
			bool added = false;
			if(_preferencesDialog->isCloudMeshing() && cloud->isOrganized())
			{
				// Fast organized mesh
				// we need to extract indices as pcl::OrganizedFastMesh doesn't take indices
				output = util3d::extractIndices(cloud, indices, false, true);
				std::vector<pcl::Vertices> polygons = util3d::organizedFastMesh(
						output,
						_preferencesDialog->getCloudMeshingAngle(),
						_preferencesDialog->isCloudMeshingQuad(),
						_preferencesDialog->getCloudMeshingTriangleSize(),
						viewPoint);
				if(polygons.size())
				{
					// remove unused vertices to save memory
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
					std::vector<pcl::Vertices> outputPolygons;
					std::vector<int> denseToOrganizedIndices = util3d::filterNotUsedVerticesFromMesh(*output, polygons, *outputFiltered, outputPolygons);

					if(_preferencesDialog->isCloudMeshingTexture() && !image.empty())
					{
						pcl::TextureMesh::Ptr textureMesh(new pcl::TextureMesh);
						pcl::toPCLPointCloud2(*outputFiltered, textureMesh->cloud);
						textureMesh->tex_polygons.push_back(outputPolygons);
						int w = cloud->width;
						int h = cloud->height;
						UASSERT(w > 1 && h > 1);
						textureMesh->tex_coordinates.resize(1);
						int nPoints = (int)outputFiltered->size();
						textureMesh->tex_coordinates[0].resize(nPoints);
						for(int i=0; i<nPoints; ++i)
						{
							//uv
							UASSERT(i < (int)denseToOrganizedIndices.size());
							int originalVertex = denseToOrganizedIndices[i];
							textureMesh->tex_coordinates[0][i] = Eigen::Vector2f(
									float(originalVertex % w) / float(w),      // u
									float(h - originalVertex / w) / float(h)); // v
						}

						pcl::TexMaterial mesh_material;
						mesh_material.tex_d = 1.0f;
						mesh_material.tex_Ns = 75.0f;
						mesh_material.tex_illum = 1;

						std::stringstream tex_name;
						tex_name << "material_" << nodeId;
						tex_name >> mesh_material.tex_name;

						mesh_material.tex_file = "";

						textureMesh->tex_materials.push_back(mesh_material);

						if(!_cloudViewer->addCloudTextureMesh(cloudName, textureMesh, image, pose))
						{
							UERROR("Adding texture mesh %d to viewer failed!", nodeId);
						}
						else
						{
							added = true;
						}
					}
					else if(!_cloudViewer->addCloudMesh(cloudName, outputFiltered, outputPolygons, pose))
					{
						UERROR("Adding mesh cloud %d to viewer failed!", nodeId);
					}
					else
					{
						added = true;
					}
				}
			}
			else
			{
				if(_preferencesDialog->isCloudMeshing())
				{
					UWARN("Online meshing is activated but the generated cloud is "
						  "dense (voxel filtering is used or multiple cameras are used). Disable "
						  "online meshing in Preferences->3D Rendering to hide this warning.");
				}

				if(_preferencesDialog->getNormalKSearch() > 0 && cloudWithNormals->size() == 0)
				{
					pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, indices, _preferencesDialog->getNormalKSearch(), _preferencesDialog->getNormalRadiusSearch(), viewPoint);
					pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);
				}

				QColor color = Qt::gray;
				if(mapId >= 0)
				{
					color = (Qt::GlobalColor)(mapId+3 % 12 + 7 );
				}

				output = util3d::extractIndices(cloud, indices, false, true);

				if(cloudWithNormals->size())
				{
					pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputWithNormals;
					outputWithNormals = util3d::extractIndices(cloudWithNormals, indices, false, false);

					if(!_cloudViewer->addCloud(cloudName, outputWithNormals, pose, color))
					{
						UERROR("Adding cloud %d to viewer failed!", nodeId);
					}
					else
					{
						added = true;
					}
				}
				else
				{
					if(!_cloudViewer->addCloud(cloudName, output, pose, color))
					{
						UERROR("Adding cloud %d to viewer failed!", nodeId);
					}
					else
					{
						added = true;
					}
				}
			}
			if(added)
			{
				outputPair.first = output;
				outputPair.second = indices;
				if(_preferencesDialog->isCloudsKept() && nodeId > 0)
				{
					_cachedClouds.insert(std::make_pair(nodeId, outputPair));
					_createdCloudsMemoryUsage += (long)(output->size() * sizeof(pcl::PointXYZRGB) + indices->size()*sizeof(int));
				}
				_cloudViewer->setCloudColorIndex(cloudName, _preferencesDialog->getCloudColorScheme(0));
				_cloudViewer->setCloudOpacity(cloudName, _preferencesDialog->getCloudOpacity(0));
				_cloudViewer->setCloudPointSize(cloudName, _preferencesDialog->getCloudPointSize(0));
			}
			else if(nodeId>0)
			{
				_cachedEmptyClouds.insert(nodeId);
			}
		}
		else if(nodeId>0)
		{
			_cachedEmptyClouds.insert(nodeId);
		}
	}

	return outputPair;
}

void MainWindow::createAndAddScanToMap(int nodeId, const Transform & pose, int mapId)
{
	std::string scanName = uFormat("scan%d", nodeId);
	if(_cloudViewer->getAddedClouds().contains(scanName))
	{
		UERROR("Scan %d already added to map.", nodeId);
		return;
	}

	QMap<int, Signature>::iterator iter = _cachedSignatures.find(nodeId);
	if(iter == _cachedSignatures.end())
	{
		UERROR("Node %d is not in the cache.", nodeId);
		return;
	}

	if(!iter->sensorData().laserScanCompressed().isEmpty() || !iter->sensorData().laserScanRaw().isEmpty())
	{
		LaserScan scan;
		iter->sensorData().uncompressData(0, 0, &scan);

		if(_preferencesDialog->getDownsamplingStepScan(0) > 1 ||
			_preferencesDialog->getScanMaxRange(0) > 0.0f ||
			_preferencesDialog->getScanMinRange(0) > 0.0f)
		{
			scan = util3d::commonFiltering(scan,
					_preferencesDialog->getDownsamplingStepScan(0),
					_preferencesDialog->getScanMinRange(0),
					_preferencesDialog->getScanMaxRange(0));
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudI;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals;
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudRGBWithNormals;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudIWithNormals;
		if(scan.hasNormals() && scan.hasRGB() && _preferencesDialog->getCloudVoxelSizeScan(0) <= 0.0)
		{
			cloudRGBWithNormals = util3d::laserScanToPointCloudRGBNormal(scan, scan.localTransform());
		}
		else if(scan.hasNormals() && scan.hasIntensity() && _preferencesDialog->getCloudVoxelSizeScan(0) <= 0.0)
		{
			cloudIWithNormals = util3d::laserScanToPointCloudINormal(scan, scan.localTransform());
		}
		else if((scan.hasNormals()) && _preferencesDialog->getCloudVoxelSizeScan(0) <= 0.0)
		{
			cloudWithNormals = util3d::laserScanToPointCloudNormal(scan, scan.localTransform());
		}
		else if(scan.hasRGB())
		{
			cloudRGB = util3d::laserScanToPointCloudRGB(scan, scan.localTransform());
		}
		else if(scan.hasIntensity())
		{
			cloudI = util3d::laserScanToPointCloudI(scan, scan.localTransform());
		}
		else
		{
			cloud = util3d::laserScanToPointCloud(scan, scan.localTransform());
		}

		if(_preferencesDialog->getCloudVoxelSizeScan(0) > 0.0)
		{
			if(cloud.get())
			{
				cloud = util3d::voxelize(cloud, _preferencesDialog->getCloudVoxelSizeScan(0));
			}
			if(cloudRGB.get())
			{
				cloudRGB = util3d::voxelize(cloudRGB, _preferencesDialog->getCloudVoxelSizeScan(0));
			}
			if(cloudI.get())
			{
				cloudI = util3d::voxelize(cloudI, _preferencesDialog->getCloudVoxelSizeScan(0));
			}
		}

		// Do ceiling/floor filtering
		if((!scan.is2d()) && // don't filter 2D scans
		   (_preferencesDialog->getScanFloorFilteringHeight() != 0.0 ||
		   _preferencesDialog->getScanCeilingFilteringHeight() != 0.0))
		{
			if(cloudRGBWithNormals.get())
			{
				// perform in /map frame
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudTransformed = util3d::transformPointCloud(cloudRGBWithNormals, pose);
				cloudTransformed = rtabmap::util3d::passThrough(
						cloudTransformed,
						"z",
						_preferencesDialog->getScanFloorFilteringHeight()==0.0?(float)std::numeric_limits<int>::min():_preferencesDialog->getScanFloorFilteringHeight(),
						_preferencesDialog->getScanCeilingFilteringHeight()==0.0?(float)std::numeric_limits<int>::max():_preferencesDialog->getScanCeilingFilteringHeight());

				//transform back in sensor frame
				cloudRGBWithNormals = util3d::transformPointCloud(cloudTransformed, pose.inverse());
			}
			if(cloudIWithNormals.get())
			{
				// perform in /map frame
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudTransformed = util3d::transformPointCloud(cloudIWithNormals, pose);
				cloudTransformed = rtabmap::util3d::passThrough(
						cloudTransformed,
						"z",
						_preferencesDialog->getScanFloorFilteringHeight()==0.0?(float)std::numeric_limits<int>::min():_preferencesDialog->getScanFloorFilteringHeight(),
						_preferencesDialog->getScanCeilingFilteringHeight()==0.0?(float)std::numeric_limits<int>::max():_preferencesDialog->getScanCeilingFilteringHeight());

				//transform back in sensor frame
				cloudIWithNormals = util3d::transformPointCloud(cloudTransformed, pose.inverse());
			}
			if(cloudWithNormals.get())
			{
				// perform in /map frame
				pcl::PointCloud<pcl::PointNormal>::Ptr cloudTransformed = util3d::transformPointCloud(cloudWithNormals, pose);
				cloudTransformed = rtabmap::util3d::passThrough(
						cloudTransformed,
						"z",
						_preferencesDialog->getScanFloorFilteringHeight()==0.0?(float)std::numeric_limits<int>::min():_preferencesDialog->getScanFloorFilteringHeight(),
						_preferencesDialog->getScanCeilingFilteringHeight()==0.0?(float)std::numeric_limits<int>::max():_preferencesDialog->getScanCeilingFilteringHeight());

				//transform back in sensor frame
				cloudWithNormals = util3d::transformPointCloud(cloudTransformed, pose.inverse());
			}
			if(cloudRGB.get())
			{
				// perform in /map frame
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTransformed = util3d::transformPointCloud(cloudRGB, pose);
				cloudTransformed = rtabmap::util3d::passThrough(
						cloudTransformed,
						"z",
						_preferencesDialog->getScanFloorFilteringHeight()==0.0?(float)std::numeric_limits<int>::min():_preferencesDialog->getScanFloorFilteringHeight(),
						_preferencesDialog->getScanCeilingFilteringHeight()==0.0?(float)std::numeric_limits<int>::max():_preferencesDialog->getScanCeilingFilteringHeight());

				//transform back in sensor frame
				cloudRGB = util3d::transformPointCloud(cloudTransformed, pose.inverse());
			}
			if(cloudI.get())
			{
				// perform in /map frame
				pcl::PointCloud<pcl::PointXYZI>::Ptr cloudTransformed = util3d::transformPointCloud(cloudI, pose);
				cloudTransformed = rtabmap::util3d::passThrough(
						cloudTransformed,
						"z",
						_preferencesDialog->getScanFloorFilteringHeight()==0.0?(float)std::numeric_limits<int>::min():_preferencesDialog->getScanFloorFilteringHeight(),
						_preferencesDialog->getScanCeilingFilteringHeight()==0.0?(float)std::numeric_limits<int>::max():_preferencesDialog->getScanCeilingFilteringHeight());

				//transform back in sensor frame
				cloudI = util3d::transformPointCloud(cloudTransformed, pose.inverse());
			}
			if(cloud.get())
			{
				// perform in /map frame
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransformed = util3d::transformPointCloud(cloud, pose);
				cloudTransformed = rtabmap::util3d::passThrough(
						cloudTransformed,
						"z",
						_preferencesDialog->getScanFloorFilteringHeight()==0.0?(float)std::numeric_limits<int>::min():_preferencesDialog->getScanFloorFilteringHeight(),
						_preferencesDialog->getScanCeilingFilteringHeight()==0.0?(float)std::numeric_limits<int>::max():_preferencesDialog->getScanCeilingFilteringHeight());

				//transform back in sensor frame
				cloud = util3d::transformPointCloud(cloudTransformed, pose.inverse());
			}
		}

		if(	(cloud.get() || cloudRGB.get() || cloudI.get()) &&
		   (_preferencesDialog->getScanNormalKSearch() > 0 || _preferencesDialog->getScanNormalRadiusSearch() > 0.0))
		{
			Eigen::Vector3f scanViewpoint(
					scan.localTransform().x(),
					scan.localTransform().y(),
					scan.localTransform().z());

			pcl::PointCloud<pcl::Normal>::Ptr normals;
			if(cloud.get() && cloud->size())
			{
				if(scan.is2d())
				{
					normals = util3d::computeFastOrganizedNormals2D(cloud, _preferencesDialog->getScanNormalKSearch(), _preferencesDialog->getScanNormalRadiusSearch(), scanViewpoint);
				}
				else
				{
					normals = util3d::computeNormals(cloud, _preferencesDialog->getScanNormalKSearch(), _preferencesDialog->getScanNormalRadiusSearch(), scanViewpoint);
				}
				cloudWithNormals.reset(new pcl::PointCloud<pcl::PointNormal>);
				pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);
				cloud.reset();
			}
			else if(cloudRGB.get() && cloudRGB->size())
			{
				// Assuming 3D
				normals = util3d::computeNormals(cloudRGB, _preferencesDialog->getScanNormalKSearch(), _preferencesDialog->getScanNormalRadiusSearch(), scanViewpoint);
				cloudRGBWithNormals.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				pcl::concatenateFields(*cloudRGB, *normals, *cloudRGBWithNormals);
				cloudRGB.reset();
			}
			else if(cloudI.get())
			{
				if(scan.is2d())
				{
					normals = util3d::computeFastOrganizedNormals2D(cloudI, _preferencesDialog->getScanNormalKSearch(), _preferencesDialog->getScanNormalRadiusSearch(), scanViewpoint);
				}
				else
				{
					normals = util3d::computeNormals(cloudI, _preferencesDialog->getScanNormalKSearch(), _preferencesDialog->getScanNormalRadiusSearch(), scanViewpoint);
				}
				cloudIWithNormals.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
				pcl::concatenateFields(*cloudI, *normals, *cloudIWithNormals);
				cloudI.reset();
			}
		}

		QColor color = Qt::gray;
		if(mapId >= 0)
		{
			color = (Qt::GlobalColor)(mapId+3 % 12 + 7 );
		}
		bool added = false;
		if(cloudRGBWithNormals.get())
		{
			added = _cloudViewer->addCloud(scanName, cloudRGBWithNormals, pose, color);
			if(added && nodeId > 0)
			{
				scan = LaserScan(util3d::laserScanFromPointCloud(*cloudRGBWithNormals, scan.localTransform().inverse()), scan.maxPoints(), scan.rangeMax(), scan.localTransform());
			}
		}
		else if(cloudIWithNormals.get())
		{
			added = _cloudViewer->addCloud(scanName, cloudIWithNormals, pose, color);
			if(added && nodeId > 0)
			{
				if(scan.is2d())
				{
					scan = LaserScan(util3d::laserScan2dFromPointCloud(*cloudIWithNormals, scan.localTransform().inverse()), scan.maxPoints(), scan.rangeMax(), scan.localTransform());
				}
				else
				{
					scan = LaserScan(util3d::laserScanFromPointCloud(*cloudIWithNormals, scan.localTransform().inverse()), scan.maxPoints(), scan.rangeMax(), scan.localTransform());
				}
			}
		}
		else if(cloudWithNormals.get())
		{
			added = _cloudViewer->addCloud(scanName, cloudWithNormals, pose, color);
			if(added && nodeId > 0)
			{
				if(scan.is2d())
				{
					scan = LaserScan(util3d::laserScan2dFromPointCloud(*cloudWithNormals, scan.localTransform().inverse()), scan.maxPoints(), scan.rangeMax(), scan.localTransform());
				}
				else
				{
					scan = LaserScan(util3d::laserScanFromPointCloud(*cloudWithNormals, scan.localTransform().inverse()), scan.maxPoints(), scan.rangeMax(), scan.localTransform());
				}
			}
		}
		else if(cloudRGB.get())
		{
			added = _cloudViewer->addCloud(scanName, cloudRGB, pose, color);
			if(added && nodeId > 0)
			{
				scan = LaserScan(util3d::laserScanFromPointCloud(*cloudRGB, scan.localTransform().inverse()), scan.maxPoints(), scan.rangeMax(), scan.localTransform());
			}
		}
		else if(cloudI.get())
		{
			added = _cloudViewer->addCloud(scanName, cloudI, pose, color);
			if(added && nodeId > 0)
			{
				if(scan.is2d())
				{
					scan = LaserScan(util3d::laserScan2dFromPointCloud(*cloudI, scan.localTransform().inverse()), scan.maxPoints(), scan.rangeMax(), scan.localTransform());
				}
				else
				{
					scan = LaserScan(util3d::laserScanFromPointCloud(*cloudI, scan.localTransform().inverse()), scan.maxPoints(), scan.rangeMax(), scan.localTransform());
				}
			}
		}
		else
		{
			UASSERT(cloud.get());
			added = _cloudViewer->addCloud(scanName, cloud, pose, color);
			if(added && nodeId > 0)
			{
				if(scan.is2d())
				{
					scan = LaserScan(util3d::laserScan2dFromPointCloud(*cloud, scan.localTransform().inverse()), scan.maxPoints(), scan.rangeMax(), scan.localTransform());
				}
				else
				{
					scan = LaserScan(util3d::laserScanFromPointCloud(*cloud, scan.localTransform().inverse()), scan.maxPoints(), scan.rangeMax(), scan.localTransform());
				}
			}
		}
		if(!added)
		{
			UERROR("Adding cloud %d to viewer failed!", nodeId);
		}
		else
		{
			if(nodeId > 0)
			{
				_createdScans.insert(std::make_pair(nodeId, scan)); // keep scan in scan frame
			}

			_cloudViewer->setCloudColorIndex(scanName, _preferencesDialog->getScanColorScheme(0)==0 && scan.is2d()?2:_preferencesDialog->getScanColorScheme(0));
			_cloudViewer->setCloudOpacity(scanName, _preferencesDialog->getScanOpacity(0));
			_cloudViewer->setCloudPointSize(scanName, _preferencesDialog->getScanPointSize(0));
		}
	}
}

void MainWindow::createAndAddFeaturesToMap(int nodeId, const Transform & pose, int mapId)
{
	UDEBUG("");
	UASSERT(!pose.isNull());
	std::string cloudName = uFormat("features%d", nodeId);
	if(_cloudViewer->getAddedClouds().contains(cloudName))
	{
		UERROR("Features cloud %d already added to map.", nodeId);
		return;
	}

	QMap<int, Signature>::iterator iter = _cachedSignatures.find(nodeId);
	if(iter == _cachedSignatures.end())
	{
		UERROR("Node %d is not in the cache.", nodeId);
		return;
	}

	if(_createdFeatures.find(nodeId) != _createdFeatures.end())
	{
		UDEBUG("Features cloud %d already created.");
		return;
	}

	if(iter->getWords3().size())
	{
		UINFO("Create cloud from 3D words");
		QColor color = Qt::gray;
		if(mapId >= 0)
		{
			color = (Qt::GlobalColor)(mapId+3 % 12 + 7 );
		}

		cv::Mat rgb;
		if(!iter->sensorData().imageCompressed().empty() || !iter->sensorData().imageRaw().empty())
		{
			SensorData data = iter->sensorData();
			data.uncompressData(&rgb, 0, 0);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud->resize(iter->getWords3().size());
		int oi=0;
		UASSERT(iter->getWords().size() == iter->getWords3().size());
		float maxDepth = _preferencesDialog->getCloudMaxDepth(0);
		UDEBUG("rgb.channels()=%d");
		if(!iter->getWords3().empty() && !iter->getWordsKpts().empty())
		{
			Transform invLocalTransform = Transform::getIdentity();
			if(iter.value().sensorData().cameraModels().size() == 1 &&
			   iter.value().sensorData().cameraModels().at(0).isValidForProjection())
			{
				invLocalTransform = iter.value().sensorData().cameraModels()[0].localTransform().inverse();
			}
			else if(iter.value().sensorData().stereoCameraModels().size() == 1 &&
					iter.value().sensorData().stereoCameraModels()[0].isValidForProjection())
			{
				invLocalTransform = iter.value().sensorData().stereoCameraModels()[0].left().localTransform().inverse();
			}

			for(std::multimap<int, int>::const_iterator jter=iter->getWords().begin(); jter!=iter->getWords().end(); ++jter)
			{
				const cv::Point3f & pt = iter->getWords3()[jter->second];
				if(util3d::isFinite(pt) &&
					(maxDepth == 0.0f ||
							//move back point in camera frame (to get depth along z), ignore for multi-camera
							(iter.value().sensorData().cameraModels().size()<=1 &&
							 util3d::transformPoint(pt, invLocalTransform).z < maxDepth)))
				{
					(*cloud)[oi].x = pt.x;
					(*cloud)[oi].y = pt.y;
					(*cloud)[oi].z = pt.z;
					const cv::KeyPoint & kpt = iter->getWordsKpts()[jter->second];
					int u = kpt.pt.x+0.5;
					int v = kpt.pt.y+0.5;
					if(!rgb.empty() &&
						uIsInBounds(u, 0, rgb.cols-1) &&
						uIsInBounds(v, 0, rgb.rows-1))
					{
						if(rgb.channels() == 1)
						{
							(*cloud)[oi].r = (*cloud)[oi].g = (*cloud)[oi].b = rgb.at<unsigned char>(v, u);
						}
						else
						{
							cv::Vec3b bgr = rgb.at<cv::Vec3b>(v, u);
							(*cloud)[oi].b = bgr.val[0];
							(*cloud)[oi].g = bgr.val[1];
							(*cloud)[oi].r = bgr.val[2];
						}
					}
					else
					{
						(*cloud)[oi].r = (*cloud)[oi].g = (*cloud)[oi].b = 255;
					}
					++oi;
				}
			}
		}
		cloud->resize(oi);
		if(!_cloudViewer->addCloud(cloudName, cloud, pose, color))
		{
			UERROR("Adding features cloud %d to viewer failed!", nodeId);
		}
		else if(nodeId > 0)
		{
			_createdFeatures.insert(std::make_pair(nodeId, cloud));
		}
	}
	else
	{
		return;
	}

	_cloudViewer->setCloudPointSize(cloudName, _preferencesDialog->getFeaturesPointSize(0));
	UDEBUG("");
}

Transform MainWindow::alignPosesToGroundTruth(
		const std::map<int, Transform> & poses,
		const std::map<int, Transform> & groundTruth)
{
	Transform t = Transform::getIdentity();
	if(groundTruth.size() && poses.size())
	{
		float translational_rmse = 0.0f;
		float translational_mean = 0.0f;
		float translational_median = 0.0f;
		float translational_std = 0.0f;
		float translational_min = 0.0f;
		float translational_max = 0.0f;
		float rotational_rmse = 0.0f;
		float rotational_mean = 0.0f;
		float rotational_median = 0.0f;
		float rotational_std = 0.0f;
		float rotational_min = 0.0f;
		float rotational_max = 0.0f;

		t = graph::calcRMSE(
				groundTruth,
				poses,
				translational_rmse,
				translational_mean,
				translational_median,
				translational_std,
				translational_min,
				translational_max,
				rotational_rmse,
				rotational_mean,
				rotational_median,
				rotational_std,
				rotational_min,
				rotational_max);

		// ground truth live statistics
		UINFO("translational_rmse=%f", translational_rmse);
		UINFO("translational_mean=%f", translational_mean);
		UINFO("translational_median=%f", translational_median);
		UINFO("translational_std=%f", translational_std);
		UINFO("translational_min=%f", translational_min);
		UINFO("translational_max=%f", translational_max);

		UINFO("rotational_rmse=%f", rotational_rmse);
		UINFO("rotational_mean=%f", rotational_mean);
		UINFO("rotational_median=%f", rotational_median);
		UINFO("rotational_std=%f", rotational_std);
		UINFO("rotational_min=%f", rotational_min);
		UINFO("rotational_max=%f", rotational_max);
	}
	return t;
}

void MainWindow::updateNodeVisibility(int nodeId, bool visible)
{
	UINFO("Update visibility %d", nodeId);
	QMap<std::string, Transform> viewerClouds = _cloudViewer->getAddedClouds();
	Transform pose;
	if(_currentGTPosesMap.size() &&
		_ui->actionAnchor_clouds_to_ground_truth->isChecked() &&
		_currentGTPosesMap.find(nodeId)!=_currentGTPosesMap.end())
	{
		pose = _currentGTPosesMap.at(nodeId);
	}
	else if(_currentPosesMap.find(nodeId) != _currentPosesMap.end())
	{
		pose = _currentPosesMap.at(nodeId);
	}

	if(!pose.isNull() || !visible)
	{
		if(_preferencesDialog->isCloudsShown(0))
		{
			std::string cloudName = uFormat("cloud%d", nodeId);
			if(visible && !viewerClouds.contains(cloudName) && _cachedSignatures.contains(nodeId))
			{
				createAndAddCloudToMap(nodeId, pose, uValue(_currentMapIds, nodeId, -1));
			}
			else if(viewerClouds.contains(cloudName))
			{
				if(visible)
				{
					//make sure the transformation was done
					_cloudViewer->updateCloudPose(cloudName, pose);
				}
				_cloudViewer->setCloudVisibility(cloudName, visible);
			}
		}

		if(_preferencesDialog->isScansShown(0))
		{
			std::string scanName = uFormat("scan%d", nodeId);
			if(visible && !viewerClouds.contains(scanName) && _cachedSignatures.contains(nodeId))
			{
				createAndAddScanToMap(nodeId, pose, uValue(_currentMapIds, nodeId, -1));
			}
			else if(viewerClouds.contains(scanName))
			{
				if(visible)
				{
					//make sure the transformation was done
					_cloudViewer->updateCloudPose(scanName, pose);
				}
				_cloudViewer->setCloudVisibility(scanName, visible);
			}
		}

		_cloudViewer->refreshView();
	}
}

void MainWindow::updateGraphView()
{
	if(_ui->dockWidget_graphViewer->isVisible())
	{
		UDEBUG("Graph visible!");
		if(_currentPosesMap.size())
		{
			this->updateMapCloud(
					std::map<int, Transform>(_currentPosesMap),
					std::multimap<int, Link>(_currentLinksMap),
					std::map<int, int>(_currentMapIds),
					std::map<int, std::string>(_currentLabels),
					std::map<int, Transform>(_currentGTPosesMap));
		}
	}
}

void MainWindow::processRtabmapEventInit(int status, const QString & info)
{
	if((RtabmapEventInit::Status)status == RtabmapEventInit::kInitializing)
	{
		_progressDialog->resetProgress();
		_progressDialog->show();
		this->changeState(MainWindow::kInitializing);
	}
	else if((RtabmapEventInit::Status)status == RtabmapEventInit::kInitialized)
	{
		_progressDialog->setValue(_progressDialog->maximumSteps());
		this->changeState(MainWindow::kInitialized);

		if(!_openedDatabasePath.isEmpty())
		{
			this->downloadAllClouds();
		}
	}
	else if((RtabmapEventInit::Status)status == RtabmapEventInit::kClosing)
	{
		_progressDialog->resetProgress();
		_progressDialog->show();
		if(_state!=kApplicationClosing)
		{
			this->changeState(MainWindow::kClosing);
		}
	}
	else if((RtabmapEventInit::Status)status == RtabmapEventInit::kClosed)
	{
		_progressDialog->setValue(_progressDialog->maximumSteps());

		if(_databaseUpdated)
		{
			if(!_newDatabasePath.isEmpty())
			{
				if(!_newDatabasePathOutput.isEmpty())
				{
					bool removed = true;
					if(QFile::exists(_newDatabasePathOutput))
					{
						removed = QFile::remove(_newDatabasePathOutput);
					}
					if(removed)
					{
						if(QFile::rename(_newDatabasePath, _newDatabasePathOutput))
						{
							std::string msg = uFormat("Database saved to \"%s\".", _newDatabasePathOutput.toStdString().c_str());
							UINFO(msg.c_str());
							QMessageBox::information(this, tr("Database saved!"), QString(msg.c_str()));
						}
						else
						{
							std::string msg = uFormat("Failed to rename temporary database from \"%s\" to \"%s\".",
									_newDatabasePath.toStdString().c_str(), _newDatabasePathOutput.toStdString().c_str());
							UERROR(msg.c_str());
							QMessageBox::critical(this, tr("Closing failed!"), QString(msg.c_str()));
						}
					}
					else
					{
						std::string msg = uFormat("Failed to overwrite the database \"%s\". The temporary database is still correctly saved at \"%s\".",
								_newDatabasePathOutput.toStdString().c_str(), _newDatabasePath.toStdString().c_str());
						UERROR(msg.c_str());
						QMessageBox::critical(this, tr("Closing failed!"), QString(msg.c_str()));
					}
				}
				else if(QFile::remove(_newDatabasePath))
				{
					UINFO("Deleted temporary database \"%s\".", _newDatabasePath.toStdString().c_str());
				}
				else if(!uStr2Bool(_preferencesDialog->getAllParameters().at(Parameters::kDbSqlite3InMemory())))
				{
					UERROR("Temporary database \"%s\" could not be deleted.", _newDatabasePath.toStdString().c_str());
				}

			}
			else if(!_openedDatabasePath.isEmpty())
			{
				std::string msg = uFormat("Database \"%s\" updated.", _openedDatabasePath.toStdString().c_str());
				UINFO(msg.c_str());
				QMessageBox::information(this, tr("Database updated!"), QString(msg.c_str()));
			}
		}
		else if(!_newDatabasePath.isEmpty())
		{
			// just remove temporary database;
			if(QFile::remove(_newDatabasePath))
			{
				UINFO("Deleted temporary database \"%s\".", _newDatabasePath.toStdString().c_str());
			}
			else if(!uStr2Bool(_preferencesDialog->getAllParameters().at(Parameters::kDbSqlite3InMemory())))
			{
				UERROR("Temporary database \"%s\" could not be deleted.", _newDatabasePath.toStdString().c_str());
			}
		}
		_openedDatabasePath.clear();
		_newDatabasePath.clear();
		_newDatabasePathOutput.clear();
		bool applicationClosing = _state == kApplicationClosing;
		this->changeState(MainWindow::kIdle);
		if(applicationClosing)
		{
			this->close();
		}
	}
	else
	{
		_progressDialog->incrementStep();
		QString msg(info);
		if((RtabmapEventInit::Status)status == RtabmapEventInit::kError)
		{
			_openedDatabasePath.clear();
			_newDatabasePath.clear();
			_newDatabasePathOutput.clear();
			_progressDialog->setAutoClose(false);
			msg.prepend(tr("[ERROR] "));
			_progressDialog->appendText(msg);
			this->changeState(MainWindow::kIdle);
		}
		else
		{
			_progressDialog->appendText(msg);
		}
	}
}

void MainWindow::processRtabmapEvent3DMap(const rtabmap::RtabmapEvent3DMap & event)
{
	_progressDialog->appendText("Downloading the map... done.");
	_progressDialog->incrementStep();

	if(event.getCode())
	{
		UERROR("Map received with code error %d!", event.getCode());
		_progressDialog->appendText(uFormat("[ERROR] Map received with code error %d!", event.getCode()).c_str());
		_progressDialog->setAutoClose(false);
	}
	else
	{

		_processingDownloadedMap = true;
		UINFO("Received map!");
		_progressDialog->appendText(tr(" poses = %1").arg(event.getPoses().size()));
		_progressDialog->appendText(tr(" constraints = %1").arg(event.getConstraints().size()));

		_progressDialog->setMaximumSteps(int(event.getSignatures().size()+event.getPoses().size()+1));
		_progressDialog->appendText(QString("Inserting data in the cache (%1 signatures downloaded)...").arg(event.getSignatures().size()));
		QApplication::processEvents();

		int addedSignatures = 0;
		std::map<int, int> mapIds;
		std::map<int, Transform> groundTruth;
		std::map<int, std::string> labels;
		for(std::map<int, Signature>::const_iterator iter = event.getSignatures().begin();
			iter!=event.getSignatures().end();
			++iter)
		{
			mapIds.insert(std::make_pair(iter->first, iter->second.mapId()));
			if(!iter->second.getGroundTruthPose().isNull())
			{
				groundTruth.insert(std::make_pair(iter->first, iter->second.getGroundTruthPose()));
			}
			if(!iter->second.getLabel().empty())
			{
				labels.insert(std::make_pair(iter->first, iter->second.getLabel()));
			}
			if(!_cachedSignatures.contains(iter->first) ||
			   (_cachedSignatures.value(iter->first).sensorData().imageCompressed().empty() && !iter->second.sensorData().imageCompressed().empty()))
			{
				_cachedSignatures.insert(iter->first, iter->second);
				_cachedMemoryUsage += iter->second.sensorData().getMemoryUsed();
				unsigned int count = 0;
				if(!iter->second.getWords3().empty())
				{
					for(std::multimap<int, int>::const_iterator jter=iter->second.getWords().upper_bound(-1); jter!=iter->second.getWords().end(); ++jter)
					{
						if(util3d::isFinite(iter->second.getWords3()[jter->second]))
						{
							++count;
						}
					}
				}
				_cachedWordsCount.insert(std::make_pair(iter->first, (float)count));
				++addedSignatures;
			}
			_progressDialog->incrementStep();
			QApplication::processEvents();
		}
		_progressDialog->appendText(tr("Inserted %1 new signatures.").arg(addedSignatures));
		_progressDialog->incrementStep();
		QApplication::processEvents();

		_progressDialog->appendText("Inserting data in the cache... done.");

		if(event.getPoses().size())
		{
			_progressDialog->appendText("Updating the 3D map cloud...");
			_progressDialog->incrementStep();
			_progressDialog->setCancelButtonVisible(true);
			_progressCanceled = false;
			QApplication::processEvents();
			std::map<int, Transform> poses = event.getPoses();
			this->updateMapCloud(poses, event.getConstraints(), mapIds, labels, groundTruth, std::map<int, Transform>(), std::multimap<int, Link>(), true);

			if( _ui->graphicsView_graphView->isVisible() &&
			    _preferencesDialog->isWordsCountGraphView() &&
				_preferencesDialog->isRGBDMode()&&
				_cachedWordsCount.size())
			{
				_ui->graphicsView_graphView->updatePosterior(_cachedWordsCount, (float)_preferencesDialog->getKpMaxFeatures());
			}

			_progressDialog->appendText("Updating the 3D map cloud... done.");
		}
		else
		{
			_progressDialog->appendText("No poses received! The map cloud cannot be updated...");
			UINFO("Map received is empty! Cannot update the map cloud...");
		}

		_progressDialog->appendText(tr("%1 locations are updated to/inserted in the cache.").arg(event.getPoses().size()));

		if(!_preferencesDialog->isImagesKept())
		{
			_cachedSignatures.clear();
			_cachedMemoryUsage = 0;
			_cachedWordsCount.clear();
		}
		if(_state != kMonitoring && _state != kDetecting)
		{
			_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
			_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
			_ui->actionDepth_Calibration->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		}
		_processingDownloadedMap = false;
	}
	_progressDialog->setValue(_progressDialog->maximumSteps());
	_progressDialog->setCancelButtonVisible(false);
	_progressCanceled = false;

	Q_EMIT(rtabmapEvent3DMapProcessed());
}

void MainWindow::processRtabmapGlobalPathEvent(const rtabmap::RtabmapGlobalPathEvent & event)
{
	if(!event.getPoses().empty())
	{
		_ui->graphicsView_graphView->setGlobalPath(event.getPoses());
	}

	if(_preferencesDialog->isCacheSavedInFigures() || _ui->statsToolBox->isVisible())
	{
		_ui->statsToolBox->updateStat("Planning/From/", float(event.getPoses().size()?event.getPoses().begin()->first:0), _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Planning/Time/ms", float(event.getPlanningTime()*1000.0), _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Planning/Goal/", float(event.getGoal()), _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Planning/Poses/", float(event.getPoses().size()), _preferencesDialog->isCacheSavedInFigures());
		_ui->statsToolBox->updateStat("Planning/Length/m", float(graph::computePathLength(event.getPoses())), _preferencesDialog->isCacheSavedInFigures());
	}
	if(_preferencesDialog->notifyWhenNewGlobalPathIsReceived())
	{
		// use MessageBox
		if(event.getPoses().empty())
		{
			QMessageBox * warn = new QMessageBox(
					QMessageBox::Warning,
					tr("Setting goal failed!"),
					tr("Setting goal to location %1%2 failed. "
						"Some reasons: \n"
						"1) the robot is not yet localized in the map,\n"
						"2) the location doesn't exist in the map,\n"
						"3) the location is not linked to the global map or\n"
						"4) the location is too near of the current location (goal already reached).")
						.arg(event.getGoal())
						.arg(!event.getGoalLabel().empty()?QString(" \"%1\"").arg(event.getGoalLabel().c_str()):""),
					QMessageBox::Ok,
					this);
			warn->setAttribute(Qt::WA_DeleteOnClose, true);
			warn->show();
		}
		else
		{
			QMessageBox * info = new QMessageBox(
					QMessageBox::Information,
					tr("Goal detected!"),
					tr("Global path computed to %1%2 (%3 poses, %4 m).")
					.arg(event.getGoal())
					.arg(!event.getGoalLabel().empty()?QString(" \"%1\"").arg(event.getGoalLabel().c_str()):"")
					.arg(event.getPoses().size())
					.arg(graph::computePathLength(event.getPoses())),
					QMessageBox::Ok,
					this);
			info->setAttribute(Qt::WA_DeleteOnClose, true);
			info->show();
		}
	}
	else if(event.getPoses().empty() && _waypoints.size())
	{
		// resend the same goal
		uSleep(1000);
		this->postGoal(_waypoints.at(_waypointsIndex % _waypoints.size()));
	}
}

void MainWindow::processRtabmapLabelErrorEvent(int id, const QString & label)
{
	QMessageBox * warn = new QMessageBox(
			QMessageBox::Warning,
			tr("Setting label failed!"),
			tr("Setting label %1 to location %2 failed. "
				"Some reasons: \n"
				"1) the location doesn't exist in the map,\n"
				"2) the location has already a label.").arg(label).arg(id),
			QMessageBox::Ok,
			this);
	warn->setAttribute(Qt::WA_DeleteOnClose, true);
	warn->show();
}

void MainWindow::processRtabmapGoalStatusEvent(int status)
{
	_ui->widget_console->appendMsg(tr("Goal status received=%1").arg(status), ULogger::kInfo);
	if(_waypoints.size())
	{
		this->postGoal(_waypoints.at(++_waypointsIndex % _waypoints.size()));
	}
}

void MainWindow::applyPrefSettings(PreferencesDialog::PANEL_FLAGS flags)
{
	ULOGGER_DEBUG("");
	if(flags & PreferencesDialog::kPanelSource)
	{
		// Camera settings...
		_ui->doubleSpinBox_stats_imgRate->setValue(_preferencesDialog->getGeneralInputRate());
		this->updateSelectSourceMenu();
		_ui->label_stats_source->setText(_preferencesDialog->getSourceDriverStr());

		if(_camera)
		{
			if(dynamic_cast<DBReader*>(_camera->camera()) != 0)
			{
				_camera->setImageRate( _preferencesDialog->isSourceDatabaseStampsUsed()?-1:_preferencesDialog->getGeneralInputRate());
			}
			else
			{
				_camera->setImageRate(_preferencesDialog->getGeneralInputRate());
			}
		}

	}//This will update the statistics toolbox

	if(flags & PreferencesDialog::kPanelGeneral)
	{
		UDEBUG("General settings changed...");
		setupMainLayout(_preferencesDialog->isVerticalLayoutUsed());
		if(!_preferencesDialog->isLocalizationsCountGraphView())
		{
			_cachedLocalizationsCount.clear();
		}
		if(!_preferencesDialog->isPosteriorGraphView() && _ui->graphicsView_graphView->isVisible())
		{
			_ui->graphicsView_graphView->clearPosterior();
		}
	}

	if(flags & PreferencesDialog::kPanelCloudRendering)
	{
		UDEBUG("Cloud rendering settings changed...");
		if(_currentPosesMap.size())
		{
			this->updateMapCloud(
					std::map<int, Transform>(_currentPosesMap),
					std::multimap<int, Link>(_currentLinksMap),
					std::map<int, int>(_currentMapIds),
					std::map<int, std::string>(_currentLabels),
					std::map<int, Transform>(_currentGTPosesMap));
		}
	}

	if(flags & PreferencesDialog::kPanelLogging)
	{
		UDEBUG("Logging settings changed...");
		ULogger::setLevel((ULogger::Level)_preferencesDialog->getGeneralLoggerLevel());
		ULogger::setEventLevel((ULogger::Level)_preferencesDialog->getGeneralLoggerEventLevel());
		ULogger::setType((ULogger::Type)_preferencesDialog->getGeneralLoggerType(),
						 (_preferencesDialog->getWorkingDirectory()+QDir::separator()+LOG_FILE_NAME).toStdString(), true);
		ULogger::setPrintTime(_preferencesDialog->getGeneralLoggerPrintTime());
		ULogger::setPrintThreadId(_preferencesDialog->getGeneralLoggerPrintThreadId());
		ULogger::setTreadIdFilter(_preferencesDialog->getGeneralLoggerThreads());
	}
}

void MainWindow::applyPrefSettings(const rtabmap::ParametersMap & parameters)
{
	applyPrefSettings(parameters, true); //post parameters
}

void MainWindow::applyPrefSettings(const rtabmap::ParametersMap & parameters, bool postParamEvent)
{
	ULOGGER_DEBUG("");
	_occupancyGrid->parseParameters(_preferencesDialog->getAllParameters());
	if(parameters.size())
	{
		for(rtabmap::ParametersMap::const_iterator iter = parameters.begin(); iter!=parameters.end(); ++iter)
		{
			UDEBUG("Parameter changed: Key=%s Value=%s", iter->first.c_str(), iter->second.c_str());
		}

		rtabmap::ParametersMap parametersModified = parameters;

		if(uContains(parameters, Parameters::kRtabmapWorkingDirectory()))
		{
			_ui->statsToolBox->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
			_ui->graphicsView_graphView->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
			_exportBundlerDialog->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
		}

		if(_state != kIdle && parametersModified.size())
		{
			if(postParamEvent)
			{
				this->post(new ParamEvent(parametersModified));
			}
		}

		// update loop closure viewer parameters (Use Map parameters)
		_loopClosureViewer->setDecimation(_preferencesDialog->getCloudDecimation(0));
		_loopClosureViewer->setMaxDepth(_preferencesDialog->getCloudMaxDepth(0));

		// update graph view parameters
		if(uContains(parameters, Parameters::kRGBDLocalRadius()))
		{
			_ui->graphicsView_graphView->setLocalRadius(uStr2Float(parameters.at(Parameters::kRGBDLocalRadius())));
		}
	}

	//update ui
	_ui->doubleSpinBox_stats_detectionRate->setValue(_preferencesDialog->getDetectionRate());
	_ui->doubleSpinBox_stats_timeLimit->setValue(_preferencesDialog->getTimeLimit());
	_ui->actionSLAM_mode->setChecked(_preferencesDialog->isSLAMMode());

	Q_EMIT(loopClosureThrChanged(_preferencesDialog->getLoopThr()));
}

void MainWindow::drawKeypoints(const std::multimap<int, cv::KeyPoint> & refWords, const std::multimap<int, cv::KeyPoint> & loopWords)
{
	UTimer timer;

	timer.start();
	ULOGGER_DEBUG("refWords.size() = %d", refWords.size());
	if(refWords.size())
	{
		_ui->imageView_source->clearFeatures();
	}
	for(std::multimap<int, cv::KeyPoint>::const_iterator iter = refWords.begin(); iter != refWords.end(); ++iter )
	{
		int id = iter->first;
		QColor color;
		if(id<0)
		{
			// GRAY = NOT QUANTIZED
			color = Qt::gray;
		}
		else if(uContains(loopWords, id))
		{
			// PINK = FOUND IN LOOP SIGNATURE
			color = Qt::magenta;
		}
		else if(_lastIds.contains(id))
		{
			// BLUE = FOUND IN LAST SIGNATURE
			color = Qt::blue;
		}
		else if(id<=_lastId)
		{
			// RED = ALREADY EXISTS
			color = Qt::red;
		}
		else if(refWords.count(id) > 1)
		{
			// YELLOW = NEW and multiple times
			color = Qt::yellow;
		}
		else
		{
			// GREEN = NEW
			color = Qt::green;
		}
		_ui->imageView_source->addFeature(iter->first, iter->second, 0, color);
	}
	ULOGGER_DEBUG("source time = %f s", timer.ticks());

	timer.start();
	ULOGGER_DEBUG("loopWords.size() = %d", loopWords.size());
	QList<QPair<cv::Point2f, cv::Point2f> > uniqueCorrespondences;
	if(loopWords.size())
	{
		_ui->imageView_loopClosure->clearFeatures();
	}
	for(std::multimap<int, cv::KeyPoint>::const_iterator iter = loopWords.begin(); iter != loopWords.end(); ++iter )
	{
		int id = iter->first;
		QColor color;
		if(id<0)
		{
			// GRAY = NOT QUANTIZED
			color = Qt::gray;
		}
		else if(uContains(refWords, id))
		{
			// PINK = FOUND IN LOOP SIGNATURE
			color = Qt::magenta;
			//To draw lines... get only unique correspondences
			if(uValues(refWords, id).size() == 1 && uValues(loopWords, id).size() == 1)
			{
				const cv::KeyPoint & a = refWords.find(id)->second;
				const cv::KeyPoint & b = iter->second;
				uniqueCorrespondences.push_back(QPair<cv::Point2f, cv::Point2f>(a.pt, b.pt));
			}
		}
		else if(id<=_lastId)
		{
			// RED = ALREADY EXISTS
			color = Qt::red;
		}
		else if(refWords.count(id) > 1)
		{
			// YELLOW = NEW and multiple times
			color = Qt::yellow;
		}
		else
		{
			// GREEN = NEW
			color = Qt::green;
		}
		_ui->imageView_loopClosure->addFeature(iter->first, iter->second, 0, color);
	}

	ULOGGER_DEBUG("loop closure time = %f s", timer.ticks());

	if(refWords.size()>0)
	{
		if((*refWords.rbegin()).first > _lastId)
		{
			_lastId = (*refWords.rbegin()).first;
		}
		_lastIds = QSet<int>::fromList(QList<int>::fromStdList(uKeysList(refWords)));
	}

	// Draw lines between corresponding features...
	float scaleSource = _ui->imageView_source->viewScale();
	float scaleLoop = _ui->imageView_loopClosure->viewScale();
	UDEBUG("scale source=%f loop=%f", scaleSource, scaleLoop);
	// Delta in actual window pixels
	float sourceMarginX = (_ui->imageView_source->width()   - _ui->imageView_source->sceneRect().width()*scaleSource)/2.0f;
	float sourceMarginY = (_ui->imageView_source->height()  - _ui->imageView_source->sceneRect().height()*scaleSource)/2.0f;
	float loopMarginX   = (_ui->imageView_loopClosure->width()   - _ui->imageView_loopClosure->sceneRect().width()*scaleLoop)/2.0f;
	float loopMarginY   = (_ui->imageView_loopClosure->height()  - _ui->imageView_loopClosure->sceneRect().height()*scaleLoop)/2.0f;

	float deltaX = 0;
	float deltaY = 0;

	if(_preferencesDialog->isVerticalLayoutUsed())
	{
		deltaY = _ui->label_matchId->height() + _ui->imageView_source->height();
	}
	else
	{
		deltaX = _ui->imageView_source->width();
	}

	if(refWords.size() && loopWords.size())
	{
		_ui->imageView_source->clearLines();
		_ui->imageView_loopClosure->clearLines();
	}

	for(QList<QPair<cv::Point2f, cv::Point2f> >::iterator iter = uniqueCorrespondences.begin();
		iter!=uniqueCorrespondences.end();
		++iter)
	{

		_ui->imageView_source->addLine(
				iter->first.x,
				iter->first.y,
				(iter->second.x*scaleLoop+loopMarginX+deltaX-sourceMarginX)/scaleSource,
				(iter->second.y*scaleLoop+loopMarginY+deltaY-sourceMarginY)/scaleSource,
				_ui->imageView_source->getDefaultMatchingLineColor());

		_ui->imageView_loopClosure->addLine(
				(iter->first.x*scaleSource+sourceMarginX-deltaX-loopMarginX)/scaleLoop,
				(iter->first.y*scaleSource+sourceMarginY-deltaY-loopMarginY)/scaleLoop,
				iter->second.x,
				iter->second.y,
				_ui->imageView_loopClosure->getDefaultMatchingLineColor());
	}
	_ui->imageView_source->update();
	_ui->imageView_loopClosure->update();
}

void MainWindow::drawLandmarks(cv::Mat & image, const Signature & signature)
{
	for(std::map<int, Link>::const_iterator iter=signature.getLandmarks().begin(); iter!=signature.getLandmarks().end(); ++iter)
	{
		// Project in all cameras in which the landmark is visible
		for(size_t i=0; i<signature.sensorData().cameraModels().size() || i<signature.sensorData().stereoCameraModels().size(); ++i)
		{
			CameraModel model;
			if(i<signature.sensorData().cameraModels().size())
			{
				model = signature.sensorData().cameraModels()[i];
			}
			else if(i<signature.sensorData().stereoCameraModels().size())
			{
				model = signature.sensorData().stereoCameraModels()[i].left();
			}
			if(model.isValidForProjection())
			{
				Transform t = model.localTransform().inverse() * iter->second.transform();
				cv::Vec3d rvec, tvec;
				tvec.val[0] = t.x();
				tvec.val[1] = t.y();
				tvec.val[2] = t.z();

				// In front of the camera?
				if(t.z() > 0)
				{
					cv::Mat R;
					t.rotationMatrix().convertTo(R, CV_64F);
					cv::Rodrigues(R, rvec);

					//cv::aruco::drawAxis(image, model.K(), model.D(), rvec, tvec, _preferencesDialog->getMarkerLength()<=0?0.1:_preferencesDialog->getMarkerLength() * 0.5f);

					// project axis points
					std::vector< cv::Point3f > axisPoints;
					float length = _preferencesDialog->getMarkerLength()<=0?0.1:_preferencesDialog->getMarkerLength() * 0.5f;
					axisPoints.push_back(cv::Point3f(0, 0, 0));
					axisPoints.push_back(cv::Point3f(length, 0, 0));
					axisPoints.push_back(cv::Point3f(0, length, 0));
					axisPoints.push_back(cv::Point3f(0, 0, length));
					std::vector< cv::Point2f > imagePoints;
					projectPoints(axisPoints, rvec, tvec, model.K(), model.D(), imagePoints);

					//offset x based on camera index
					bool valid = true;
					if(i!=0)
					{
						if(model.imageWidth() <= 0)
						{
							valid = false;
							UWARN("Cannot draw correctly landmark %d with provided camera model %d (missing image width)", -iter->first, (int)i);
						}
						else
						{
							for(int j=0; j<4; ++j)
							{
								imagePoints[j].x += i*model.imageWidth();
							}
						}
					}
					if(valid)
					{
						// draw axis lines
						cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);
						cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3);
						cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
						cv::putText(image, uNumber2Str(-iter->first), imagePoints[0], cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 255), 2);
					}
				}
			}
		}
	}
}

void MainWindow::showEvent(QShowEvent* anEvent)
{
	//if the config file doesn't exist, make the GUI obsolete
	this->setWindowModified(!QFile::exists(_preferencesDialog->getIniFilePath()));
}

void MainWindow::moveEvent(QMoveEvent* anEvent)
{
	if(this->isVisible())
	{
		// HACK, there is a move event when the window is shown the first time.
		if(!_firstCall)
		{
			this->configGUIModified();
		}
		_firstCall = false;
	}
}

void MainWindow::resizeEvent(QResizeEvent* anEvent)
{
	if(this->isVisible())
	{
		this->configGUIModified();
	}
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
	//catch ctrl-s to save settings
	if((event->modifiers() & Qt::ControlModifier) && event->key() == Qt::Key_S)
	{
		this->saveConfigGUI();
	}
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
	if (event->type() == QEvent::Resize && qobject_cast<QDockWidget*>(obj))
	{
		this->setWindowModified(true);
	}
    else if(event->type() == QEvent::FileOpen )
    {
        openDatabase(((QFileOpenEvent*)event)->file());
    }
	return QWidget::eventFilter(obj, event);
}

void MainWindow::updateSelectSourceMenu()
{
	_ui->actionUsbCamera->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcUsbDevice);

	_ui->actionMore_options->setChecked(
			_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcDatabase ||
			_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcImages ||
			_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcVideo ||
			_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoImages ||
			_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoVideo ||
			_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcRGBDImages
	);

	_ui->actionOpenNI_PCL->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcOpenNI_PCL);
	_ui->actionOpenNI_PCL_ASUS->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcOpenNI_PCL);
	_ui->actionFreenect->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcFreenect);
	_ui->actionOpenNI_CV->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcOpenNI_CV);
	_ui->actionOpenNI_CV_ASUS->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcOpenNI_CV_ASUS);
	_ui->actionOpenNI2->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcOpenNI2);
	_ui->actionOpenNI2_kinect->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcOpenNI2);
	_ui->actionOpenNI2_orbbec->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcOpenNI2);
	_ui->actionOpenNI2_sense->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcOpenNI2);
	_ui->actionFreenect2->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcFreenect2);
	_ui->actionKinect_for_Windows_SDK_v2->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcK4W2);
	_ui->actionKinect_for_Azure->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcK4A);
	_ui->actionRealSense_R200->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcRealSense);
	_ui->actionRealSense_ZR300->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcRealSense);
	_ui->actionRealSense2_SR300->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcRealSense2);
	_ui->actionRealSense2_D400->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcRealSense2);
	_ui->actionRealSense2_L515->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcRealSense2);
	_ui->actionStereoDC1394->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcDC1394);
	_ui->actionStereoFlyCapture2->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcFlyCapture2);
	_ui->actionStereoZed->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoZed);
	_ui->actionZed_Open_Capture->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoZedOC);
    _ui->actionStereoTara->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoTara);
	_ui->actionStereoUsb->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoUsb);
	_ui->actionRealSense2_T265->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoRealSense2);
	_ui->actionMYNT_EYE_S_SDK->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoMyntEye);
	_ui->actionDepthAI->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoDepthAI);
}

void MainWindow::changeImgRateSetting()
{
	Q_EMIT imgRateChanged(_ui->doubleSpinBox_stats_imgRate->value());
}

void MainWindow::changeDetectionRateSetting()
{
	Q_EMIT detectionRateChanged(_ui->doubleSpinBox_stats_detectionRate->value());
}

void MainWindow::changeTimeLimitSetting()
{
	Q_EMIT timeLimitChanged((float)_ui->doubleSpinBox_stats_timeLimit->value());
}

void MainWindow::changeMappingMode()
{
	Q_EMIT mappingModeChanged(_ui->actionSLAM_mode->isChecked());
}

QString MainWindow::captureScreen(bool cacheInRAM, bool png)
{
	QString name = (QDateTime::currentDateTime().toString("yyMMddhhmmsszzz") + (png?".png":".jpg"));
	_ui->statusbar->clearMessage();
	QPixmap figure = QPixmap::grabWidget(this);

	QString targetDir = _preferencesDialog->getWorkingDirectory() + QDir::separator() + "ScreensCaptured";
	QString msg;
	if(cacheInRAM)
	{
		msg = tr("Screen captured \"%1\"").arg(name);
		QByteArray bytes;
		QBuffer buffer(&bytes);
		buffer.open(QIODevice::WriteOnly);
		figure.save(&buffer, png?"PNG":"JPEG");
		_autoScreenCaptureCachedImages.insert(name, bytes);
	}
	else
	{
		QDir dir;
		if(!dir.exists(targetDir))
		{
			dir.mkdir(targetDir);
		}
		targetDir += QDir::separator();
		targetDir += "Main_window";
		if(!dir.exists(targetDir))
		{
			dir.mkdir(targetDir);
		}
		targetDir += QDir::separator();

		figure.save(targetDir + name);
		msg = tr("Screen captured \"%1\"").arg(targetDir + name);
	}
	_ui->statusbar->showMessage(msg, _preferencesDialog->getTimeLimit()*500);
	_ui->widget_console->appendMsg(msg);

	return targetDir + name;
}

void MainWindow::beep()
{
	QApplication::beep();
}

void MainWindow::cancelProgress()
{
	_progressCanceled = true;
	_progressDialog->appendText(tr("Canceled!"));
}

void MainWindow::configGUIModified()
{
	this->setWindowModified(true);
}

void MainWindow::updateParameters(const ParametersMap & parameters)
{
	if(parameters.size())
	{
		for(ParametersMap::const_iterator iter= parameters.begin(); iter!=parameters.end(); ++iter)
		{
			QString msg = tr("Parameter update \"%1\"=\"%2\"")
							.arg(iter->first.c_str())
							.arg(iter->second.c_str());
			_ui->widget_console->appendMsg(msg);
			UWARN(msg.toStdString().c_str());
		}
		QMessageBox::StandardButton button = QMessageBox::question(this,
				tr("Parameters"),
				tr("Some parameters have been set on command line, do you "
					"want to set all other RTAB-Map's parameters to default?"),
					QMessageBox::Yes | QMessageBox::No,
					QMessageBox::No);
		_preferencesDialog->updateParameters(parameters, button==QMessageBox::Yes);
	}
}

//ACTIONS
void MainWindow::saveConfigGUI()
{
	_savedMaximized = this->isMaximized();
	_preferencesDialog->saveMainWindowState(this);
	_preferencesDialog->saveWindowGeometry(_preferencesDialog);
	_preferencesDialog->saveWindowGeometry(_aboutDialog);
	_preferencesDialog->saveWidgetState(_cloudViewer);
	_preferencesDialog->saveWidgetState(_ui->imageView_source);
	_preferencesDialog->saveWidgetState(_ui->imageView_loopClosure);
	_preferencesDialog->saveWidgetState(_ui->imageView_odometry);
	_preferencesDialog->saveWidgetState(_exportCloudsDialog);
	_preferencesDialog->saveWidgetState(_exportBundlerDialog);
	_preferencesDialog->saveWidgetState(_postProcessingDialog);
	_preferencesDialog->saveWidgetState(_depthCalibrationDialog);
	_preferencesDialog->saveWidgetState(_ui->graphicsView_graphView);
	_preferencesDialog->saveWidgetState(_multiSessionLocWidget->getImageView());
	_preferencesDialog->saveSettings();
	this->saveFigures();
	this->setWindowModified(false);
}

void MainWindow::newDatabase()
{
	if(_state != MainWindow::kIdle)
	{
		UERROR("This method can be called only in IDLE state.");
		return;
	}
	_openedDatabasePath.clear();
	_newDatabasePath.clear();
	_newDatabasePathOutput.clear();
	_databaseUpdated = false;
	_cloudViewer->removeLine("map_to_odom");
	_cloudViewer->removeLine("odom_to_base_link");
	_cloudViewer->removeCoordinate("odom_frame");
	_cloudViewer->removeCoordinate("map_frame");
	ULOGGER_DEBUG("");
	this->clearTheCache();
	std::string databasePath = (_preferencesDialog->getWorkingDirectory()+QDir::separator()+QString("rtabmap.tmp.db")).toStdString();
	if(QFile::exists(databasePath.c_str()))
	{
		int r = QMessageBox::question(this,
				tr("Creating temporary database"),
				tr("Cannot create a new database because the temporary database \"%1\" already exists. "
				  "There may be another instance of RTAB-Map running with the same Working Directory or "
				  "the last time RTAB-Map was not closed correctly. "
				  "Do you want to recover the database (click Ignore to delete it and create a new one)?").arg(databasePath.c_str()),
				  QMessageBox::Yes | QMessageBox::No | QMessageBox::Ignore, QMessageBox::No);

		if(r == QMessageBox::Ignore)
		{
			if(QFile::remove(databasePath.c_str()))
			{
				UINFO("Deleted temporary database \"%s\".", databasePath.c_str());
			}
			else
			{
				UERROR("Temporary database \"%s\" could not be deleted!", databasePath.c_str());
				return;
			}
		}
		else if(r == QMessageBox::Yes)
		{
			std::string errorMsg;
			rtabmap::ProgressDialog * progressDialog = new rtabmap::ProgressDialog(this);
			progressDialog->setAttribute(Qt::WA_DeleteOnClose);
			progressDialog->setMaximumSteps(100);
			progressDialog->show();
			progressDialog->setCancelButtonVisible(true);
			RecoveryState state(progressDialog);
			_recovering = true;
			if(databaseRecovery(databasePath, false, &errorMsg, &state))
			{
				_recovering = false;
				progressDialog->setValue(progressDialog->maximumSteps());
				QString newPath = QFileDialog::getSaveFileName(this, tr("Save recovered database"), _preferencesDialog->getWorkingDirectory()+QDir::separator()+QString("recovered.db"), tr("RTAB-Map database files (*.db)"));
				if(newPath.isEmpty())
				{
					return;
				}
				if(QFileInfo(newPath).suffix() == "")
				{
					newPath += ".db";
				}
				if(QFile::exists(newPath))
				{
					QFile::remove(newPath);
				}
				QFile::rename(databasePath.c_str(), newPath);
				return;
			}
			else
			{
				_recovering = false;
				progressDialog->setValue(progressDialog->maximumSteps());
				QMessageBox::warning(this, "Database recovery", tr("Database recovery failed: \"%1\".").arg(errorMsg.c_str()));
				return;
			}
		}
		else
		{
			return;
		}
	}
	_newDatabasePath = databasePath.c_str();
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdInit, databasePath, _preferencesDialog->getAllParameters()));
	applyPrefSettings(_preferencesDialog->getAllParameters(), false);
}

void MainWindow::openDatabase()
{
	QString path = QFileDialog::getOpenFileName(this, tr("Open database..."), _defaultOpenDatabasePath.isEmpty()?_preferencesDialog->getWorkingDirectory():_defaultOpenDatabasePath, tr("RTAB-Map database files (*.db)"));
	if(!path.isEmpty())
	{
		this->openDatabase(path);
	}
}

void MainWindow::openDatabase(const QString & path, const ParametersMap & overridedParameters)
{
	if(_state != MainWindow::kIdle)
	{
		UERROR("Database can only be opened in IDLE state.");
		return;
	}

	std::string value = path.toStdString();
	if(UFile::exists(value) &&
	   UFile::getExtension(value).compare("db") == 0)
	{
		_openedDatabasePath.clear();
		_newDatabasePath.clear();
		_newDatabasePathOutput.clear();
		_databaseUpdated = false;

		this->clearTheCache();
		_openedDatabasePath = path;
		_defaultOpenDatabasePath = path;

		// look if there are saved parameters
		DBDriver * driver = DBDriver::create();
		if(driver->openConnection(value, false))
		{
			ParametersMap parameters = driver->getLastParameters();
			driver->closeConnection(false);
			delete driver;

			if(parameters.size())
			{
				//backward compatibility with databases not saving all parameters, use default for not saved ones
				for(ParametersMap::const_iterator iter=Parameters::getDefaultParameters().begin(); iter!=Parameters::getDefaultParameters().end(); ++iter)
				{
					parameters.insert(*iter);
				}

				uInsert(parameters, overridedParameters);

				ParametersMap currentParameters = _preferencesDialog->getAllParameters();
				ParametersMap differentParameters;
				for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
				{
					ParametersMap::iterator jter = currentParameters.find(iter->first);
					if(jter!=currentParameters.end() &&
					   iter->second.compare(jter->second) != 0 &&
					   iter->first.compare(Parameters::kRtabmapWorkingDirectory()) != 0)
					{
						bool different = true;
						if(Parameters::getType(iter->first).compare("double") ==0 ||
						   Parameters::getType(iter->first).compare("float") == 0)
						{
							if(uStr2Double(iter->second) == uStr2Double(jter->second))
							{
								different = false;
							}
						}
						else if(Parameters::getType(iter->first).compare("bool") == 0)
						{
							if(uStr2Bool(iter->second) == uStr2Bool(jter->second))
							{
								different = false;
							}
						}
						if(different)
						{
							differentParameters.insert(*iter);
							QString msg = tr("Parameter \"%1\": database=\"%2\" Preferences=\"%3\"")
											.arg(iter->first.c_str())
											.arg(iter->second.c_str())
											.arg(jter->second.c_str());
							_ui->widget_console->appendMsg(msg);
							UWARN(msg.toStdString().c_str());
						}
					}
				}

				if(differentParameters.size())
				{
					int r = QMessageBox::question(this,
							tr("Update parameters..."),
							tr("The database is using %1 different parameter(s) than "
							   "those currently set in Preferences. Do you want "
							   "to use database's parameters?").arg(differentParameters.size()),
							QMessageBox::Yes | QMessageBox::No,
							QMessageBox::Yes);
					if(r == QMessageBox::Yes)
					{
						_preferencesDialog->updateParameters(differentParameters);
					}
				}
			}
		}

		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdInit, value, 0, _preferencesDialog->getAllParameters()));
		applyPrefSettings(_preferencesDialog->getAllParameters(), false);
	}
	else
	{
		UERROR("File \"%s\" not valid.", value.c_str());
	}
}

bool MainWindow::closeDatabase()
{
	if(_state != MainWindow::kInitialized)
	{
		UERROR("This method can be called only in INITIALIZED state.");
		return false;
	}

	_newDatabasePathOutput.clear();
	if(!_newDatabasePath.isEmpty() && _databaseUpdated)
	{
		QMessageBox::Button b = QMessageBox::question(this,
				tr("Save database"),
				tr("Save the new database?"),
				QMessageBox::Save | QMessageBox::Cancel | QMessageBox::Discard,
				QMessageBox::Save);

		if(b == QMessageBox::Save)
		{
			// Temp database used, automatically backup with unique name (timestamp)
			QString newName = QDateTime::currentDateTime().toString("yyMMdd-hhmmss");
			QString newPath = _preferencesDialog->getWorkingDirectory()+QDir::separator()+newName+".db";

			newPath = QFileDialog::getSaveFileName(this, tr("Save database"), newPath, tr("RTAB-Map database files (*.db)"));
			if(newPath.isEmpty())
			{
				return false;
			}

			if(QFileInfo(newPath).suffix() == "")
			{
				newPath += ".db";
			}

			_newDatabasePathOutput = newPath;
		}
		else if(b != QMessageBox::Discard)
		{
			return false;
		}
	}

	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdClose, !_openedDatabasePath.isEmpty() || !_newDatabasePathOutput.isEmpty()));
	return true;
}

void MainWindow::editDatabase()
{
	if(_state != MainWindow::kIdle)
	{
		UERROR("This method can be called only in IDLE state.");
		return;
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Edit database..."), _preferencesDialog->getWorkingDirectory(), tr("RTAB-Map database files (*.db)"));
	if(!path.isEmpty())
	{
		{
			// copy database settings to tmp ini file
			QSettings dbSettingsIn(_preferencesDialog->getIniFilePath(), QSettings::IniFormat);
			QSettings dbSettingsOut(_preferencesDialog->getTmpIniFilePath(), QSettings::IniFormat);
			dbSettingsIn.beginGroup("DatabaseViewer");
			dbSettingsOut.beginGroup("DatabaseViewer");
			QStringList keys = dbSettingsIn.childKeys();
			for(QStringList::iterator iter = keys.begin(); iter!=keys.end(); ++iter)
			{
				dbSettingsOut.setValue(*iter, dbSettingsIn.value(*iter));
			}
			dbSettingsIn.endGroup();
			dbSettingsOut.endGroup();
		}

		DatabaseViewer * viewer = new DatabaseViewer(_preferencesDialog->getTmpIniFilePath(), this);
		viewer->setWindowModality(Qt::WindowModal);
		viewer->setAttribute(Qt::WA_DeleteOnClose, true);
		viewer->showCloseButton();

		if(viewer->isSavedMaximized())
		{
			viewer->showMaximized();
		}
		else
		{
			viewer->show();
		}

		viewer->openDatabase(path);
	}
}

Camera * MainWindow::createCamera(
		Camera ** odomSensor,
		Transform & odomSensorExtrinsics,
		double & odomSensorTimeOffset,
		float & odomSensorScaleFactor)
{
	Camera * camera = _preferencesDialog->createCamera();

	if(camera &&
	   _preferencesDialog->getOdomSourceDriver() != PreferencesDialog::kSrcUndef &&
	   _preferencesDialog->getOdomSourceDriver() != _preferencesDialog->getSourceDriver() &&
		!(_preferencesDialog->getOdomSourceDriver() == PreferencesDialog::kSrcStereoRealSense2 &&
		  _preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcRealSense2))
	{
		UINFO("Create Odom Sensor %d (camera = %d)",
				_preferencesDialog->getOdomSourceDriver(),
				_preferencesDialog->getSourceDriver());
		*odomSensor = _preferencesDialog->createOdomSensor(odomSensorExtrinsics, odomSensorTimeOffset, odomSensorScaleFactor);
	}

	return camera;
}

void MainWindow::startDetection()
{
	UDEBUG("");
	ParametersMap parameters = _preferencesDialog->getAllParameters();
	uInsert(parameters, this->getCustomParameters());

	// verify source with input rates
	if(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcImages ||
	   _preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcVideo ||
	   _preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcRGBDImages ||
	   _preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoImages ||
	   _preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoVideo ||
	   _preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcDatabase)
	{
		float inputRate = _preferencesDialog->getGeneralInputRate();
		float detectionRate = uStr2Float(parameters.at(Parameters::kRtabmapDetectionRate()));
		int bufferingSize = uStr2Float(parameters.at(Parameters::kRtabmapImageBufferSize()));
		if(((detectionRate!=0.0f && detectionRate <= inputRate) || (detectionRate > 0.0f && inputRate == 0.0f)) &&
			(_preferencesDialog->getSourceDriver() != PreferencesDialog::kSrcDatabase || !_preferencesDialog->isSourceDatabaseStampsUsed()))
		{
			int button = QMessageBox::question(this,
					tr("Incompatible frame rates!"),
					tr("\"Source/Input rate\" (%1 Hz) is equal to/higher than \"RTAB-Map/Detection rate\" (%2 Hz). As the "
					   "source input is a directory of images/video/database, some images may be "
					   "skipped by the detector. You may want to increase the \"RTAB-Map/Detection rate\" over "
					   "the \"Source/Input rate\" to guaranty that all images are processed. Would you want to "
					   "start the detection anyway?").arg(inputRate).arg(detectionRate),
					 QMessageBox::Yes | QMessageBox::No);
			if(button == QMessageBox::No)
			{
				return;
			}
		}
		if(_preferencesDialog->getSourceDriver() != PreferencesDialog::kSrcDatabase || !_preferencesDialog->isSourceDatabaseStampsUsed())
		{
			if(bufferingSize != 0)
			{
				int button = QMessageBox::question(this,
						tr("Some images may be skipped!"),
						tr("\"RTAB-Map/Images buffer size\" is not infinite (size=%1). As the "
						   "source input is a directory of images/video/database, some images may be "
						   "skipped by the detector if the \"Source/Input rate\" (which is %2 Hz) is higher than the "
						   "rate at which RTAB-Map can process the images. You may want to set the "
						   "\"RTAB-Map/Images buffer size\" to 0 (infinite) to guaranty that all "
						   "images are processed. Would you want to start the detection "
						   "anyway?").arg(bufferingSize).arg(inputRate),
						 QMessageBox::Yes | QMessageBox::No);
				if(button == QMessageBox::No)
				{
					return;
				}
			}
			else if(inputRate == 0)
			{
				int button = QMessageBox::question(this,
						tr("Large number of images may be buffered!"),
						tr("\"RTAB-Map/Images buffer size\" is infinite. As the "
						   "source input is a directory of images/video/database and "
						   "that \"Source/Input rate\" is infinite too, a lot of images "
						   "could be buffered at the same time (e.g., reading all images "
						   "of a directory at once). This could make the GUI not responsive. "
						   "You may want to set \"Source/Input rate\" at the rate at "
						   "which the images have been recorded. "
						   "Would you want to start the detection "
						   "anyway?").arg(bufferingSize).arg(inputRate),
						 QMessageBox::Yes | QMessageBox::No);
				if(button == QMessageBox::No)
				{
					return;
				}
			}
		}
	}

	UDEBUG("");
	Q_EMIT stateChanged(kStartingDetection);

	if(_camera != 0)
	{
		QMessageBox::warning(this,
						     tr("RTAB-Map"),
						     tr("A camera is running, stop it first."));
		UWARN("_camera is not null... it must be stopped first");
		Q_EMIT stateChanged(kInitialized);
		return;
	}

	// Adjust pre-requirements
	if(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcUndef)
	{
		QMessageBox::warning(this,
				 tr("RTAB-Map"),
				 tr("No sources are selected. See Preferences->Source panel."));
		UWARN("No sources are selected. See Preferences->Source panel.");
		Q_EMIT stateChanged(kInitialized);
		return;
	}


	double poseTimeOffset = 0.0;
	float scaleFactor = 0.0f;
	Transform extrinsics;
	Camera * odomSensor = 0;
	Camera * camera = this->createCamera(&odomSensor, extrinsics, poseTimeOffset, scaleFactor);
	if(!camera)
	{
		Q_EMIT stateChanged(kInitialized);
		return;
	}

	if(odomSensor)
	{
		_camera = new CameraThread(camera, odomSensor, extrinsics, poseTimeOffset, scaleFactor, _preferencesDialog->isOdomSensorAsGt(), parameters);
	}
	else
	{
		_camera = new CameraThread(camera, _preferencesDialog->isOdomSensorAsGt(), parameters);
	}
	_camera->setMirroringEnabled(_preferencesDialog->isSourceMirroring());
	_camera->setColorOnly(_preferencesDialog->isSourceRGBDColorOnly());
	_camera->setImageDecimation(_preferencesDialog->getSourceImageDecimation());
	_camera->setStereoToDepth(_preferencesDialog->isSourceStereoDepthGenerated());
	_camera->setStereoExposureCompensation(_preferencesDialog->isSourceStereoExposureCompensation());
	_camera->setScanParameters(
			_preferencesDialog->isSourceScanFromDepth(),
			_preferencesDialog->getSourceScanDownsampleStep(),
			_preferencesDialog->getSourceScanRangeMin(),
			_preferencesDialog->getSourceScanRangeMax(),
			_preferencesDialog->getSourceScanVoxelSize(),
			_preferencesDialog->getSourceScanNormalsK(),
			_preferencesDialog->getSourceScanNormalsRadius(),
			(float)_preferencesDialog->getSourceScanForceGroundNormalsUp());
	if(_preferencesDialog->getIMUFilteringStrategy()>0 && dynamic_cast<DBReader*>(camera) == 0)
	{
		_camera->enableIMUFiltering(_preferencesDialog->getIMUFilteringStrategy()-1, parameters, _preferencesDialog->getIMUFilteringBaseFrameConversion());
	}
	if(_preferencesDialog->isDepthFilteringAvailable())
	{
		if(_preferencesDialog->isBilateralFiltering())
		{
			_camera->enableBilateralFiltering(
					_preferencesDialog->getBilateralSigmaS(),
					_preferencesDialog->getBilateralSigmaR());
		}
		_camera->setDistortionModel(_preferencesDialog->getSourceDistortionModel().toStdString());
	}

	//Create odometry thread if rgbd slam
	if(uStr2Bool(parameters.at(Parameters::kRGBDEnabled()).c_str()))
	{
		// Require calibrated camera
		if(!camera->isCalibrated())
		{
			UWARN("Camera is not calibrated!");
			Q_EMIT stateChanged(kInitialized);
			delete _camera;
			_camera = 0;

			int button = QMessageBox::question(this,
					tr("Camera is not calibrated!"),
					tr("RTAB-Map in metric SLAM mode cannot run with an uncalibrated camera. Do you want to calibrate the camera now?"),
					 QMessageBox::Yes | QMessageBox::No);
			if(button == QMessageBox::Yes)
			{
				QTimer::singleShot(0, _preferencesDialog, SLOT(calibrate()));
			}
			return;
		}
		else
		{
			if(_odomThread)
			{
				UERROR("OdomThread must be already deleted here?!");
				delete _odomThread;
				_odomThread = 0;
			}

			if(_imuThread)
			{
				UERROR("ImuThread must be already deleted here?!");
				delete _imuThread;
				_imuThread = 0;
			}

			if((!_camera->odomProvided() || _preferencesDialog->isOdomSensorAsGt()) && !_preferencesDialog->isOdomDisabled())
			{
				ParametersMap odomParameters = parameters;
				if(_preferencesDialog->getOdomRegistrationApproach() < 3)
				{
					uInsert(odomParameters, ParametersPair(Parameters::kRegStrategy(), uNumber2Str(_preferencesDialog->getOdomRegistrationApproach())));
				}
				odomParameters.erase(Parameters::kRtabmapPublishRAMUsage()); // as odometry is in the same process than rtabmap, don't get RAM usage in odometry.
				int odomStrategy = Parameters::defaultOdomStrategy();
				Parameters::parse(odomParameters, Parameters::kOdomStrategy(), odomStrategy);
				double gravitySigma = _preferencesDialog->getOdomF2MGravitySigma();
				UDEBUG("Odom gravitySigma=%f", gravitySigma);
				if(gravitySigma >= 0.0)
				{
					uInsert(odomParameters, ParametersPair(Parameters::kOptimizerGravitySigma(), uNumber2Str(gravitySigma)));
				}
				if(odomStrategy != 1)
				{
					// Only Frame To Frame supports all VisCorType
					odomParameters.erase(Parameters::kVisCorType());
				}
				_imuThread = 0;
				if((_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoImages ||
					_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcRGBDImages ||
					_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcImages) &&
				   !_preferencesDialog->getIMUPath().isEmpty())
				{
					if( odomStrategy != Odometry::kTypeOkvis &&
						odomStrategy != Odometry::kTypeMSCKF &&
						odomStrategy != Odometry::kTypeVINS &&
						odomStrategy != Odometry::kTypeOpenVINS)
					{
						QMessageBox::warning(this, tr("Source IMU Path"),
								tr("IMU path is set but odometry chosen doesn't support asynchronous IMU, ignoring IMU..."), QMessageBox::Ok);
					}
					else
					{
						_imuThread = new IMUThread(_preferencesDialog->getIMURate(), _preferencesDialog->getIMULocalTransform());
						if(!_imuThread->init(_preferencesDialog->getIMUPath().toStdString()))
						{
							QMessageBox::warning(this, tr("Source IMU Path"),
								tr("Initialization of IMU data has failed! Path=%1.").arg(_preferencesDialog->getIMUPath()), QMessageBox::Ok);
							delete _camera;
							_camera = 0;
							delete _imuThread;
							_imuThread = 0;
							return;
						}
					}
				}
				Odometry * odom = Odometry::create(odomParameters);
				_odomThread = new OdometryThread(odom, _preferencesDialog->getOdomBufferSize());

				UEventsManager::addHandler(_odomThread);
				UEventsManager::createPipe(_camera, _odomThread, "CameraEvent");
				UEventsManager::createPipe(_camera, this, "CameraEvent");
				if(_imuThread)
				{
					UEventsManager::createPipe(_imuThread, _odomThread, "IMUEvent");
				}
				_odomThread->start();
			}
		}
	}

	if(_dataRecorder && _camera && _odomThread)
	{
		UEventsManager::createPipe(_camera, _dataRecorder, "CameraEvent");
	}

	_lastOdomPose.setNull();
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdCleanDataBuffer)); // clean sensors buffer
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdTriggerNewMap)); // Trigger a new map

	if(_odomThread)
	{
		_ui->actionReset_Odometry->setEnabled(true);
	}

	if(!_preferencesDialog->isStatisticsPublished())
	{
		QMessageBox::information(this,
				tr("Information"),
				tr("Note that publishing statistics is disabled, "
				   "progress will not be shown in the GUI."));
	}

	_occupancyGrid->clear();
	_occupancyGrid->parseParameters(parameters);

#ifdef RTABMAP_OCTOMAP
	UASSERT(_octomap != 0);
	delete _octomap;
	_octomap = new OctoMap(parameters);
#endif

	// clear odometry visual stuff
	_cloudViewer->removeCloud("cloudOdom");
	_cloudViewer->removeCloud("scanOdom");
	_cloudViewer->removeCloud("scanMapOdom");
	_cloudViewer->removeCloud("featuresOdom");
	_cloudViewer->setBackgroundColor(_cloudViewer->getDefaultBackgroundColor());

	Q_EMIT stateChanged(kDetecting);
}

// Could not be in the main thread here! (see handleEvents())
void MainWindow::pauseDetection()
{
	if(_camera)
	{
		if(_state == kPaused && (QApplication::keyboardModifiers() & Qt::ShiftModifier))
		{
			// On Ctrl-click, start the camera and pause it automatically
			Q_EMIT stateChanged(kPaused);
			if(_preferencesDialog->getGeneralInputRate())
			{
				QTimer::singleShot(500.0/_preferencesDialog->getGeneralInputRate(), this, SLOT(pauseDetection()));
			}
			else
			{
				Q_EMIT stateChanged(kPaused);
			}
		}
		else
		{
			Q_EMIT stateChanged(kPaused);
		}
	}
	else if(_state == kMonitoring)
	{
		UINFO("Sending pause event!");
		Q_EMIT stateChanged(kMonitoringPaused);
	}
	else if(_state == kMonitoringPaused)
	{
		UINFO("Sending unpause event!");
		Q_EMIT stateChanged(kMonitoring);
	}
}

void MainWindow::stopDetection()
{
	if(!_camera && !_odomThread)
	{
		return;
	}

	if(_state == kDetecting &&
	   (_camera && _camera->isRunning()) )
	{
		QMessageBox::StandardButton button = QMessageBox::question(this, tr("Stopping process..."), tr("Are you sure you want to stop the process?"), QMessageBox::Yes|QMessageBox::No, QMessageBox::No);

		if(button != QMessageBox::Yes)
		{
			return;
		}
	}

	ULOGGER_DEBUG("");
	// kill the processes
	if(_imuThread)
	{
		_imuThread->join(true);
	}

	if(_camera)
	{
		_camera->join(true);
	}

	if(_odomThread)
	{
		_ui->actionReset_Odometry->setEnabled(false);
		_odomThread->kill();
	}

	// delete the processes
	if(_imuThread)
	{
		delete _imuThread;
		_imuThread = 0;
	}
	if(_camera)
	{
		delete _camera;
		_camera = 0;
	}
	if(_odomThread)
	{
		delete _odomThread;
		_odomThread = 0;
	}

	if(_dataRecorder)
	{
		delete _dataRecorder;
		_dataRecorder = 0;
	}

	Q_EMIT stateChanged(kInitialized);
}

void MainWindow::notifyNoMoreImages()
{
	QMessageBox::information(this,
			tr("No more images..."),
			tr("The camera has reached the end of the stream."));
}

void MainWindow::printLoopClosureIds()
{
	_ui->dockWidget_console->show();
	QString msgRef;
	QString msgLoop;
	for(int i = 0; i<_refIds.size(); ++i)
	{
		msgRef.append(QString::number(_refIds[i]));
		msgLoop.append(QString::number(_loopClosureIds[i]));
		if(i < _refIds.size() - 1)
		{
			msgRef.append(" ");
			msgLoop.append(" ");
		}
	}
	_ui->widget_console->appendMsg(QString("IDs = [%1];").arg(msgRef));
	_ui->widget_console->appendMsg(QString("LoopIDs = [%1];").arg(msgLoop));
}

void MainWindow::generateGraphDOT()
{
	if(_graphSavingFileName.isEmpty())
	{
		_graphSavingFileName = _preferencesDialog->getWorkingDirectory() + QDir::separator() + "Graph.dot";
	}

	bool ok;
	int id = QInputDialog::getInt(this, tr("Around which location?"), tr("Location ID (0=full map)"), 0, 0, 999999, 0, &ok);
	if(ok)
	{
		int margin = 0;
		if(id > 0)
		{
			margin = QInputDialog::getInt(this, tr("Depth around the location?"), tr("Margin"), 4, 1, 100, 1, &ok);
		}

		if(ok)
		{
			QString path = QFileDialog::getSaveFileName(this, tr("Save File"), _graphSavingFileName, tr("Graphiz file (*.dot)"));
			if(!path.isEmpty())
			{
				_graphSavingFileName = path;
				this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGenerateDOTGraph, false, path.toStdString(), id, margin));

				_ui->dockWidget_console->show();
				_ui->widget_console->appendMsg(QString("Graph saved... Tip:\nneato -Tpdf \"%1\" -o out.pdf").arg(_graphSavingFileName));
			}
		}
	}
}

void MainWindow::exportPosesRaw()
{
	exportPoses(0);
}
void MainWindow::exportPosesRGBDSLAMMotionCapture()
{
	exportPoses(1);
}
void MainWindow::exportPosesRGBDSLAM()
{
	exportPoses(10);
}
void MainWindow::exportPosesRGBDSLAMID()
{
	exportPoses(11);
}
void MainWindow::exportPosesKITTI()
{
	exportPoses(2);
}
void MainWindow::exportPosesTORO()
{
	exportPoses(3);
}
void MainWindow::exportPosesG2O()
{
	exportPoses(4);
}

void MainWindow::exportPoses(int format)
{
	if(_currentPosesMap.size())
	{
		std::map<int, Transform> localTransforms;
		QStringList items;
		items.push_back("Robot (base frame)");
		items.push_back("Camera");
		items.push_back("Scan");
		bool ok;
		QString item = QInputDialog::getItem(this, tr("Export Poses"), tr("Frame: "), items, _exportPosesFrame, false, &ok);
		if(!ok || item.isEmpty())
		{
			return;
		}
		if(item.compare("Robot (base frame)") != 0)
		{
			bool cameraFrame = item.compare("Camera") == 0;
			_exportPosesFrame = cameraFrame?1:2;
			for(std::map<int, Transform>::iterator iter=_currentPosesMap.lower_bound(1); iter!=_currentPosesMap.end(); ++iter)
			{
				if(_cachedSignatures.contains(iter->first))
				{
					Transform localTransform;
					if(cameraFrame)
					{
						if(_cachedSignatures[iter->first].sensorData().cameraModels().size() == 1 &&
						   !_cachedSignatures[iter->first].sensorData().cameraModels().at(0).localTransform().isNull())
						{
							localTransform = _cachedSignatures[iter->first].sensorData().cameraModels().at(0).localTransform();
						}
						else if(_cachedSignatures[iter->first].sensorData().stereoCameraModels().size() == 1 &&
								!_cachedSignatures[iter->first].sensorData().stereoCameraModels()[0].localTransform().isNull())
						{
							localTransform = _cachedSignatures[iter->first].sensorData().stereoCameraModels()[0].localTransform();
						}
						else if(_cachedSignatures[iter->first].sensorData().cameraModels().size()>1 ||
								_cachedSignatures[iter->first].sensorData().stereoCameraModels().size()>1)
						{
							UWARN("Multi-camera is not supported (node %d)", iter->first);
						}
						else
						{
							UWARN("Missing calibration for node %d", iter->first);
						}
					}
					else
					{
						if(!_cachedSignatures[iter->first].sensorData().laserScanRaw().localTransform().isNull())
						{
							localTransform = _cachedSignatures[iter->first].sensorData().laserScanRaw().localTransform();
						}
						else if(!_cachedSignatures[iter->first].sensorData().laserScanCompressed().localTransform().isNull())
						{
							localTransform = _cachedSignatures[iter->first].sensorData().laserScanCompressed().localTransform();
						}
						else
						{
							UWARN("Missing scan info for node %d", iter->first);
						}
					}
					if(!localTransform.isNull())
					{
						localTransforms.insert(std::make_pair(iter->first, localTransform));
					}
				}
				else
				{
					UWARN("Did not find node %d in cache", iter->first);
				}
			}
			if(localTransforms.empty())
			{
				QMessageBox::warning(this,
						tr("Export Poses"),
						tr("Could not find any \"%1\" frame, exporting in Robot frame instead.").arg(item));
			}
		}
		else
		{
			_exportPosesFrame = 0;
		}

		std::map<int, Transform> poses;
		std::multimap<int, Link> links;
		if(localTransforms.empty())
		{
			poses = std::map<int, Transform>(_currentPosesMap.lower_bound(1), _currentPosesMap.end());
			links = std::multimap<int, Link>(_currentLinksMap.lower_bound(1), _currentLinksMap.end());
		}
		else
		{
			//adjust poses and links
			for(std::map<int, Transform>::iterator iter=localTransforms.begin(); iter!=localTransforms.end(); ++iter)
			{
				poses.insert(std::make_pair(iter->first, _currentPosesMap.at(iter->first) * iter->second));
			}
			for(std::multimap<int, Link>::iterator iter=_currentLinksMap.lower_bound(1); iter!=_currentLinksMap.end(); ++iter)
			{
				if(uContains(poses, iter->second.from()) && uContains(poses, iter->second.to()))
				{
					std::multimap<int, Link>::iterator inserted = links.insert(*iter);
					int from = iter->second.from();
					int to = iter->second.to();
					inserted->second.setTransform(localTransforms.at(from).inverse()*iter->second.transform()*localTransforms.at(to));
				}
			}
		}

		if(format != 4 && !poses.empty() && poses.begin()->first<0) // not g2o, landmark not supported
		{
			UWARN("Only g2o format (4) can export landmarks, they are ignored with format %d", format);
			std::map<int, Transform>::iterator iter=poses.begin();
			while(iter!=poses.end() && iter->first < 0)
			{
				poses.erase(iter++);
			}
		}

		std::map<int, double> stamps;
		if(format == 1 || format == 10 || format == 11)
		{
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				if(_cachedSignatures.contains(iter->first))
				{
					stamps.insert(std::make_pair(iter->first, _cachedSignatures.value(iter->first).getStamp()));
				}
			}
			if(stamps.size()!=poses.size())
			{
				QMessageBox::warning(this, tr("Export poses..."), tr("RGB-D SLAM format: Poses (%1) and stamps (%2) have not the same size! Try again after updating the cache.")
						.arg(poses.size()).arg(stamps.size()));
				return;
			}
		}

		if(_exportPosesFileName[format].isEmpty())
		{
			_exportPosesFileName[format] = _preferencesDialog->getWorkingDirectory() + QDir::separator() + (format==3?"toro.graph":format==4?"poses.g2o":"poses.txt");
		}

		QString path = QFileDialog::getSaveFileName(
				this,
				tr("Save File"),
				_exportPosesFileName[format],
				format == 3?tr("TORO file (*.graph)"):format==4?tr("g2o file (*.g2o)"):tr("Text file (*.txt)"));

		if(!path.isEmpty())
		{
			if(QFileInfo(path).suffix() == "")
			{
				if(format == 3)
				{
					path += ".graph";
				}
				else if(format==4)
				{
					path += ".g2o";
				}
				else
				{
					path += ".txt";
				}
			}

			_exportPosesFileName[format] = path;
			bool saved = graph::exportPoses(path.toStdString(), format, poses, links, stamps, _preferencesDialog->getAllParameters());

			if(saved)
			{
				QMessageBox::information(this,
						tr("Export poses..."),
						tr("%1 saved to \"%2\".")
						.arg(format == 3?"TORO graph":format == 4?"g2o graph":"Poses")
						.arg(_exportPosesFileName[format]));
			}
			else
			{
				QMessageBox::information(this,
						tr("Export poses..."),
						tr("Failed to save %1 to \"%2\"!")
						.arg(format == 3?"TORO graph":format == 4?"g2o graph":"poses")
						.arg(_exportPosesFileName[format]));
			}
		}
	}
}

void MainWindow::showPostProcessingDialog()
{
	if(_postProcessingDialog->exec() != QDialog::Accepted)
	{
		return;
	}

	postProcessing(
			_postProcessingDialog->isRefineNeighborLinks(),
			_postProcessingDialog->isRefineLoopClosureLinks(),
			_postProcessingDialog->isDetectMoreLoopClosures(),
			_postProcessingDialog->clusterRadius(),
			_postProcessingDialog->clusterAngle(),
			_postProcessingDialog->iterations(),
			_postProcessingDialog->interSession(),
			_postProcessingDialog->intraSession(),
			_postProcessingDialog->isSBA(),
			_postProcessingDialog->sbaIterations(),
			_postProcessingDialog->sbaVariance(),
			_postProcessingDialog->sbaType(),
			_postProcessingDialog->sbaRematchFeatures());
}

void MainWindow::postProcessing(
		bool refineNeighborLinks,
		bool refineLoopClosureLinks,
		bool detectMoreLoopClosures,
		double clusterRadius,
		double clusterAngle,
		int iterations,
		bool interSession,
		bool intraSession,
		bool sba,
		int sbaIterations,
		double sbaVariance,
		Optimizer::Type sbaType,
		double sbaRematchFeatures,
		bool abortIfDataMissing)
{
	if(_cachedSignatures.size() == 0)
	{
		QMessageBox::warning(this,
				tr("Post-processing..."),
				tr("Signatures must be cached in the GUI for post-processing. "
				   "Check the option in Preferences->General Settings (GUI), then "
				   "refresh the cache."));
		return;
	}

	if(!detectMoreLoopClosures && !refineNeighborLinks && !refineLoopClosureLinks && !sba)
	{
		UWARN("No post-processing selection...");
		return;
	}

	if(_currentPosesMap.lower_bound(1) == _currentPosesMap.end())
	{
		UWARN("No nodes to process...");
		return;
	}

	// First, verify that we have all data required in the GUI
	std::map<int, Transform> odomPoses;
	bool allDataAvailable = true;
	for(std::map<int, Transform>::iterator iter = _currentPosesMap.lower_bound(1);
			iter!=_currentPosesMap.end() && allDataAvailable;
			++iter)
	{
		QMap<int, Signature>::iterator jter = _cachedSignatures.find(iter->first);
		if(jter == _cachedSignatures.end())
		{
			UWARN("Node %d missing.", iter->first);
			allDataAvailable = false;
		}
		else if(!jter.value().getPose().isNull())
		{
			odomPoses.insert(std::make_pair(iter->first, jter.value().getPose()));
		}
	}

	if(!allDataAvailable)
	{
		QString msg = tr("Some data missing in the cache to respect the constraints chosen. "
				   "Try \"Edit->Download all clouds\" to update the cache and try again.");
		UWARN(msg.toStdString().c_str());
		if(abortIfDataMissing)
		{
			QMessageBox::warning(this, tr("Not all data available in the GUI..."), msg);
			return;
		}
	}

	_progressDialog->resetProgress();
	_progressDialog->clear();
	_progressDialog->show();
	_progressDialog->appendText("Post-processing beginning!");
	_progressDialog->setCancelButtonVisible(true);
	_progressCanceled = false;

	int totalSteps = 0;
	if(refineNeighborLinks)
	{
		totalSteps+=(int)_currentPosesMap.size();
	}
	if(refineLoopClosureLinks)
	{
		totalSteps+=(int)_currentLinksMap.size() - (int)_currentPosesMap.size();
	}
	if(sba)
	{
		totalSteps+=1;
	}
	_progressDialog->setMaximumSteps(totalSteps);
	_progressDialog->show();

	ParametersMap parameters = _preferencesDialog->getAllParameters();
	Optimizer * optimizer = Optimizer::create(parameters);
	bool optimizeFromGraphEnd =  Parameters::defaultRGBDOptimizeFromGraphEnd();
	float optimizeMaxError =  Parameters::defaultRGBDOptimizeMaxError();
	int optimizeIterations =  Parameters::defaultOptimizerIterations();
	bool reextractFeatures = Parameters::defaultRGBDLoopClosureReextractFeatures();
	Parameters::parse(parameters, Parameters::kRGBDOptimizeFromGraphEnd(), optimizeFromGraphEnd);
	Parameters::parse(parameters, Parameters::kRGBDOptimizeMaxError(), optimizeMaxError);
	Parameters::parse(parameters, Parameters::kOptimizerIterations(), optimizeIterations);
	Parameters::parse(parameters, Parameters::kRGBDLoopClosureReextractFeatures(), reextractFeatures);

	bool warn = false;
	int loopClosuresAdded = 0;
	std::multimap<int, int> checkedLoopClosures;
	if(detectMoreLoopClosures)
	{
		UDEBUG("");

		bool loopCovLimited = Parameters::defaultRGBDLoopCovLimited();
		Parameters::parse(parameters, Parameters::kRGBDLoopCovLimited(), loopCovLimited);
		std::vector<double> odomMaxInf;
		if(loopCovLimited)
		{
			odomMaxInf = graph::getMaxOdomInf(_currentLinksMap);
		}

		UASSERT(iterations>0);
		for(int n=0; n<iterations && !_progressCanceled; ++n)
		{
			_progressDialog->appendText(tr("Looking for more loop closures, clustering poses... (iteration=%1/%2, radius=%3 m angle=%4 degrees)")
					.arg(n+1).arg(iterations).arg(clusterRadius).arg(clusterAngle));

			std::multimap<int, int> clusters = graph::radiusPosesClustering(
					std::map<int, Transform>(_currentPosesMap.upper_bound(0), _currentPosesMap.end()),
					clusterRadius,
					clusterAngle*CV_PI/180.0);

			_progressDialog->setMaximumSteps(_progressDialog->maximumSteps()+(int)clusters.size());
			_progressDialog->appendText(tr("Looking for more loop closures, clustering poses... found %1 clusters.").arg(clusters.size()));
			QApplication::processEvents();

			int i=0;
			std::set<int> addedLinks;
			for(std::multimap<int, int>::iterator iter=clusters.begin(); iter!= clusters.end() && !_progressCanceled; ++iter, ++i)
			{
				int from = iter->first;
				int to = iter->second;
				if(iter->first < iter->second)
				{
					from = iter->second;
					to = iter->first;
				}

				int mapIdFrom = uValue(_currentMapIds, from, 0);
				int mapIdTo = uValue(_currentMapIds, to, 0);

				if((interSession && mapIdFrom != mapIdTo) ||
				   (intraSession && mapIdFrom == mapIdTo))
				{
					bool alreadyChecked = false;
					for(std::multimap<int, int>::iterator jter = checkedLoopClosures.lower_bound(from);
						!alreadyChecked && jter!=checkedLoopClosures.end() && jter->first == from;
						++jter)
					{
						if(to == jter->second)
						{
							alreadyChecked = true;
						}
					}

					if(!alreadyChecked)
					{
						// only add new links and one per cluster per iteration
						if(addedLinks.find(from) == addedLinks.end() &&
						   addedLinks.find(to) == addedLinks.end() &&
						   rtabmap::graph::findLink(_currentLinksMap, from, to) == _currentLinksMap.end())
						{
							// Reverify if in the bounds with the current optimized graph
							Transform delta = _currentPosesMap.at(from).inverse() * _currentPosesMap.at(to);
							if(delta.getNorm() < clusterRadius)
							{
								checkedLoopClosures.insert(std::make_pair(from, to));

								if(!_cachedSignatures.contains(from))
								{
									UERROR("Didn't find signature %d", from);
								}
								else if(!_cachedSignatures.contains(to))
								{
									UERROR("Didn't find signature %d", to);
								}
								else
								{
									Signature signatureFrom = _cachedSignatures[from];
									Signature signatureTo = _cachedSignatures[to];

									if(signatureFrom.getWeight() >= 0 &&
									   signatureTo.getWeight() >= 0) // ignore intermediate nodes
									{
										Transform transform;
										RegistrationInfo info;
										if(parameters.find(Parameters::kRegStrategy()) != parameters.end() &&
											parameters.at(Parameters::kRegStrategy()).compare("1") == 0)
										{
											uInsert(parameters, ParametersPair(Parameters::kRegStrategy(), "2"));
										}
										Registration * registration = Registration::create(parameters);

										if(reextractFeatures)
										{
											signatureFrom.sensorData().uncompressData();
											signatureTo.sensorData().uncompressData();

											if(signatureFrom.sensorData().imageRaw().empty() &&
											   signatureTo.sensorData().imageRaw().empty())
											{
												UWARN("\"%s\" is false and signatures (%d and %d) don't have raw "
														"images. Update the cache.",
													Parameters::kRGBDLoopClosureReextractFeatures().c_str());
											}
											else
											{
												signatureFrom.removeAllWords();
												signatureFrom.sensorData().setFeatures(std::vector<cv::KeyPoint>(), std::vector<cv::Point3f>(), cv::Mat());
												signatureTo.removeAllWords();
												signatureTo.sensorData().setFeatures(std::vector<cv::KeyPoint>(), std::vector<cv::Point3f>(), cv::Mat());
											}
										}
										else if(!reextractFeatures && signatureFrom.getWords().empty() && signatureTo.getWords().empty())
										{
											UWARN("\"%s\" is false and signatures (%d and %d) don't have words, "
													"registration will not be possible. Set \"%s\" to true.",
													Parameters::kRGBDLoopClosureReextractFeatures().c_str(),
													signatureFrom.id(),
													signatureTo.id(),
													Parameters::kRGBDLoopClosureReextractFeatures().c_str());
										}
										transform = registration->computeTransformation(signatureFrom, signatureTo, Transform(), &info);
										delete registration;
										if(!transform.isNull())
										{
											//optimize the graph to see if the new constraint is globally valid
											bool updateConstraint = true;
											cv::Mat information = info.covariance.inv();
											if(odomMaxInf.size() == 6 && information.cols==6 && information.rows==6)
											{
												for(int i=0; i<6; ++i)
												{
													if(information.at<double>(i,i) > odomMaxInf[i])
													{
														information.at<double>(i,i) = odomMaxInf[i];
													}
												}
											}
											if(optimizeMaxError > 0.0f && optimizeIterations > 0)
											{
												int fromId = from;
												int mapId = _currentMapIds.at(from);
												// use first node of the map containing from
												for(std::map<int, int>::iterator iter=_currentMapIds.begin(); iter!=_currentMapIds.end(); ++iter)
												{
													if(iter->second == mapId && _currentPosesMap.find(iter->first)!=_currentPosesMap.end())
													{
														fromId = iter->first;
														break;
													}
												}
												std::multimap<int, Link> linksIn = _currentLinksMap;
												linksIn.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, transform, information)));
												const Link * maxLinearLink = 0;
												const Link * maxAngularLink = 0;
												float maxLinearError = 0.0f;
												float maxAngularError = 0.0f;
												std::map<int, Transform> poses;
												std::multimap<int, Link> links;
												UASSERT(_currentPosesMap.find(fromId) != _currentPosesMap.end());
												UASSERT_MSG(_currentPosesMap.find(from) != _currentPosesMap.end(), uFormat("id=%d poses=%d links=%d", from, (int)poses.size(), (int)links.size()).c_str());
												UASSERT_MSG(_currentPosesMap.find(to) != _currentPosesMap.end(), uFormat("id=%d poses=%d links=%d", to, (int)poses.size(), (int)links.size()).c_str());
												optimizer->getConnectedGraph(fromId, _currentPosesMap, linksIn, poses, links);
												UASSERT(poses.find(fromId) != poses.end());
												UASSERT_MSG(poses.find(from) != poses.end(), uFormat("id=%d poses=%d links=%d", from, (int)poses.size(), (int)links.size()).c_str());
												UASSERT_MSG(poses.find(to) != poses.end(), uFormat("id=%d poses=%d links=%d", to, (int)poses.size(), (int)links.size()).c_str());
												UASSERT(graph::findLink(links, from, to) != links.end());
												poses = optimizer->optimize(fromId, poses, links);
												std::string msg;
												if(poses.size())
												{
													float maxLinearErrorRatio = 0.0f;
													float maxAngularErrorRatio = 0.0f;
													graph::computeMaxGraphErrors(
															poses,
															links,
															maxLinearErrorRatio,
															maxAngularErrorRatio,
															maxLinearError,
															maxAngularError,
															&maxLinearLink,
															&maxAngularLink);
													if(maxLinearLink)
													{
														UINFO("Max optimization linear error = %f m (link %d->%d)", maxLinearError, maxLinearLink->from(), maxLinearLink->to());
														if(maxLinearErrorRatio > optimizeMaxError)
														{
															msg = uFormat("Rejecting edge %d->%d because "
																	  "graph error is too large after optimization (%f m for edge %d->%d with ratio %f > std=%f m). "
																	  "\"%s\" is %f.",
																	  from,
																	  to,
																	  maxLinearError,
																	  maxLinearLink->from(),
																	  maxLinearLink->to(),
																	  maxLinearErrorRatio,
																	  sqrt(maxLinearLink->transVariance()),
																	  Parameters::kRGBDOptimizeMaxError().c_str(),
																	  optimizeMaxError);
														}
													}
													else if(maxAngularLink)
													{
														UINFO("Max optimization angular error = %f deg (link %d->%d)", maxAngularError*180.0f/M_PI, maxAngularLink->from(), maxAngularLink->to());
														if(maxAngularErrorRatio > optimizeMaxError)
														{
															msg = uFormat("Rejecting edge %d->%d because "
																	  "graph error is too large after optimization (%f deg for edge %d->%d with ratio %f > std=%f deg). "
																	  "\"%s\" is %f m.",
																	  from,
																	  to,
																	  maxAngularError*180.0f/M_PI,
																	  maxAngularLink->from(),
																	  maxAngularLink->to(),
																	  maxAngularErrorRatio,
																	  sqrt(maxAngularLink->rotVariance()),
																	  Parameters::kRGBDOptimizeMaxError().c_str(),
																	  optimizeMaxError);
														}
													}
												}
												else
												{
													msg = uFormat("Rejecting edge %d->%d because graph optimization has failed!",
															  from,
															  to);
												}
												if(!msg.empty())
												{
													UWARN("%s", msg.c_str());
													_progressDialog->appendText(tr("%1").arg(msg.c_str()));
													QApplication::processEvents();
													updateConstraint = false;
												}
												else
												{
													_currentPosesMap = poses;
												}
											}

											if(updateConstraint)
											{
												UINFO("Added new loop closure between %d and %d.", from, to);
												addedLinks.insert(from);
												addedLinks.insert(to);

												_currentLinksMap.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, transform, information)));
												++loopClosuresAdded;
												_progressDialog->appendText(tr("Detected loop closure %1->%2! (%3/%4)").arg(from).arg(to).arg(i+1).arg(clusters.size()));
											}
										}
									}
								}
							}
						}
					}
				}
				QApplication::processEvents();
				_progressDialog->incrementStep();
			}
			_progressDialog->appendText(tr("Iteration %1/%2: Detected %3 loop closures!").arg(n+1).arg(iterations).arg(addedLinks.size()/2));
			if(addedLinks.size() == 0)
			{
				break;
			}

			if(n+1 < iterations)
			{
				_progressDialog->appendText(tr("Optimizing graph with new links (%1 nodes, %2 constraints)...")
						.arg(_currentPosesMap.size()).arg(_currentLinksMap.size()));
				QApplication::processEvents();

				UASSERT(_currentPosesMap.lower_bound(1) != _currentPosesMap.end());
				int fromId = optimizeFromGraphEnd?_currentPosesMap.rbegin()->first:_currentPosesMap.lower_bound(1)->first;
				std::map<int, rtabmap::Transform> posesOut;
				std::multimap<int, rtabmap::Link> linksOut;
				std::map<int, rtabmap::Transform> optimizedPoses;
				std::multimap<int, rtabmap::Link> linksIn = _currentLinksMap;
				optimizer->getConnectedGraph(
						fromId,
						_currentPosesMap,
						linksIn,
						posesOut,
						linksOut);
				optimizedPoses = optimizer->optimize(fromId, posesOut, linksOut);
				_currentPosesMap = optimizedPoses;
				_progressDialog->appendText(tr("Optimizing graph with new links... done!"));
			}
		}
		UINFO("Added %d loop closures.", loopClosuresAdded);
		_progressDialog->appendText(tr("Total new loop closures detected=%1").arg(loopClosuresAdded));
	}

	if(!_progressCanceled && (refineNeighborLinks || refineLoopClosureLinks))
	{
		UDEBUG("");
		if(refineLoopClosureLinks)
		{
			_progressDialog->setMaximumSteps(_progressDialog->maximumSteps()+loopClosuresAdded);
		}
		// TODO: support ICP from laser scans?
		_progressDialog->appendText(tr("Refining links..."));
		QApplication::processEvents();

		RegistrationIcp regIcp(parameters);

		int i=0;
		for(std::multimap<int, Link>::iterator iter = _currentLinksMap.lower_bound(1); iter!=_currentLinksMap.end() && !_progressCanceled; ++iter, ++i)
		{
			int type = iter->second.type();
			int from = iter->second.from();
			int to = iter->second.to();

			if((refineNeighborLinks && type==Link::kNeighbor) ||
			   (refineLoopClosureLinks && type!=Link::kNeighbor && type!=Link::kLandmark && from!=to))
			{
				_progressDialog->appendText(tr("Refining link %1->%2 (%3/%4)").arg(from).arg(to).arg(i+1).arg(_currentLinksMap.size()));
				_progressDialog->incrementStep();
				QApplication::processEvents();

				if(!_cachedSignatures.contains(from))
				{
					UERROR("Didn't find signature %d",from);
				}
				else if(!_cachedSignatures.contains(to))
				{
					UERROR("Didn't find signature %d", to);
				}
				else
				{
					Signature & signatureFrom = _cachedSignatures[from];
					Signature & signatureTo = _cachedSignatures[to];

					LaserScan tmp;
					signatureFrom.sensorData().uncompressData(0,0,&tmp);
					signatureTo.sensorData().uncompressData(0,0,&tmp);

					if(!signatureFrom.sensorData().laserScanRaw().isEmpty() &&
					   !signatureTo.sensorData().laserScanRaw().isEmpty())
					{
						RegistrationInfo info;
						Transform transform = regIcp.computeTransformation(signatureFrom.sensorData(), signatureTo.sensorData(), iter->second.transform(), &info);

						if(!transform.isNull())
						{
							Link newLink(from, to, iter->second.type(), transform, info.covariance.inv());
							iter->second = newLink;
						}
						else
						{
							QString str = tr("Cannot refine link %1->%2 (%3").arg(from).arg(to).arg(info.rejectedMsg.c_str());
							_progressDialog->appendText(str, Qt::darkYellow);
							UWARN("%s", str.toStdString().c_str());
							warn = true;
						}
					}
					else
					{
						QString str;
						if(signatureFrom.getWeight() < 0 || signatureTo.getWeight() < 0)
						{
							str = tr("Cannot refine link %1->%2 (Intermediate node detected!)").arg(from).arg(to);
						}
						else
						{
							str = tr("Cannot refine link %1->%2 (scans empty!)").arg(from).arg(to);
						}

						_progressDialog->appendText(str, Qt::darkYellow);
						UWARN("%s", str.toStdString().c_str());
						warn = true;
					}
				}
			}
		}
		_progressDialog->appendText(tr("Refining links...done!"));
	}

	_progressDialog->appendText(tr("Optimizing graph with updated links (%1 nodes, %2 constraints)...")
			.arg(_currentPosesMap.size()).arg(_currentLinksMap.size()));

	UASSERT(_currentPosesMap.lower_bound(1) != _currentPosesMap.end());
	int fromId = optimizeFromGraphEnd?_currentPosesMap.rbegin()->first:_currentPosesMap.lower_bound(1)->first;
	std::map<int, rtabmap::Transform> posesOut;
	std::multimap<int, rtabmap::Link> linksOut;
	std::map<int, rtabmap::Transform> optimizedPoses;
	std::multimap<int, rtabmap::Link> linksIn = _currentLinksMap;
	optimizer->getConnectedGraph(
			fromId,
			_currentPosesMap,
			linksIn,
			posesOut,
			linksOut);
	optimizedPoses = optimizer->optimize(fromId, posesOut, linksOut);
	_progressDialog->appendText(tr("Optimizing graph with updated links... done!"));
	_progressDialog->incrementStep();

	if(!_progressCanceled && sba)
	{
		UASSERT(Optimizer::isAvailable(sbaType));
		_progressDialog->appendText(tr("SBA (%1 nodes, %2 constraints, %3 iterations)...")
					.arg(optimizedPoses.size()).arg(linksOut.size()).arg(sbaIterations));
		QApplication::processEvents();
		uSleep(100);
		QApplication::processEvents();

		ParametersMap parametersSBA = _preferencesDialog->getAllParameters();
		uInsert(parametersSBA, std::make_pair(Parameters::kOptimizerIterations(), uNumber2Str(sbaIterations)));
		uInsert(parametersSBA, std::make_pair(Parameters::kg2oPixelVariance(), uNumber2Str(sbaVariance)));
		Optimizer * sbaOptimizer = Optimizer::create(sbaType, parametersSBA);
		std::map<int, Transform>  newPoses = sbaOptimizer->optimizeBA(optimizedPoses.begin()->first, optimizedPoses, linksOut, _cachedSignatures.toStdMap(), sbaRematchFeatures);
		delete sbaOptimizer;
		if(newPoses.size())
		{
			optimizedPoses = newPoses;
			_progressDialog->appendText(tr("SBA... done!"));
		}
		else
		{
			_progressDialog->appendText(tr("SBA... failed!"));
			_progressDialog->setAutoClose(false);
		}
		_progressDialog->incrementStep();
	}

	_progressDialog->appendText(tr("Updating map..."));
	this->updateMapCloud(
			optimizedPoses,
			std::multimap<int, Link>(_currentLinksMap),
			std::map<int, int>(_currentMapIds),
			std::map<int, std::string>(_currentLabels),
			std::map<int, Transform>(_currentGTPosesMap),
			std::map<int, Transform>(),
			std::multimap<int, Link>(),
			false);
	_progressDialog->appendText(tr("Updating map... done!"));

	if(warn)
	{
		_progressDialog->setAutoClose(false);
	}

	_progressDialog->setValue(_progressDialog->maximumSteps());
	_progressDialog->appendText("Post-processing finished!");
	_progressDialog->setCancelButtonVisible(false);
	_progressCanceled = false;

	delete optimizer;
}

void MainWindow::depthCalibration()
{
	if(_currentPosesMap.size() && _cachedSignatures.size())
	{
		_depthCalibrationDialog->calibrate(
				_currentPosesMap,
				_cachedSignatures,
				_preferencesDialog->getWorkingDirectory(),
				_preferencesDialog->getAllParameters());
	}
	else
	{
		QMessageBox::warning(this, tr("Depth Calibration"), tr("No data in cache. Try to refresh the cache."));
	}
}

void MainWindow::deleteMemory()
{
	QMessageBox::StandardButton button;
	if(_state == kMonitoring || _state == kMonitoringPaused)
	{
		button = QMessageBox::question(this,
				tr("Deleting memory..."),
				tr("The remote database and log files will be deleted. Are you sure you want to continue? (This cannot be reverted)"),
				QMessageBox::Yes|QMessageBox::No,
				QMessageBox::No);
	}
	else
	{
		button = QMessageBox::question(this,
				tr("Deleting memory..."),
				tr("The current database and log files will be deleted. Are you sure you want to continue? (This cannot be reverted)"),
				QMessageBox::Yes|QMessageBox::No,
				QMessageBox::No);
	}

	if(button != QMessageBox::Yes)
	{
		return;
	}

	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdResetMemory));
	if(_state!=kDetecting)
	{
		_databaseUpdated = false;
	}
	this->clearTheCache();
}

QString MainWindow::getWorkingDirectory() const
{
	return _preferencesDialog->getWorkingDirectory();
}

void MainWindow::openWorkingDirectory()
{
	QString filePath = _preferencesDialog->getWorkingDirectory();
#if defined(Q_WS_MAC)
    QStringList args;
    args << "-e";
    args << "tell application \"Finder\"";
    args << "-e";
    args << "activate";
    args << "-e";
    args << "select POSIX file \""+filePath+"\"";
    args << "-e";
    args << "end tell";
    QProcess::startDetached("osascript", args);
#elif defined(Q_WS_WIN)
    QStringList args;
    args << "/select," << QDir::toNativeSeparators(filePath);
    QProcess::startDetached("explorer", args);
#else
    UERROR("Only works on Mac and Windows");
#endif
}

void MainWindow::updateEditMenu()
{
	// Update Memory delete database size
	if(_state != kMonitoring && _state != kMonitoringPaused && (!_openedDatabasePath.isEmpty() || !_newDatabasePath.isEmpty()))
	{
		if(!_openedDatabasePath.isEmpty())
		{
			_ui->actionDelete_memory->setText(tr("Delete memory (%1 MB)").arg(UFile::length(_openedDatabasePath.toStdString())/1000000));
		}
		else
		{
			_ui->actionDelete_memory->setText(tr("Delete memory (%1 MB)").arg(UFile::length(_newDatabasePath.toStdString())/1000000));
		}
	}
}

void MainWindow::selectStream()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcUsbDevice);
}

void MainWindow::selectOpenni()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcOpenNI_PCL);
}

void MainWindow::selectFreenect()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcFreenect);
}

void MainWindow::selectOpenniCv()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcOpenNI_CV);
}

void MainWindow::selectOpenniCvAsus()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcOpenNI_CV_ASUS);
}

void MainWindow::selectOpenni2()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcOpenNI2);
}

void MainWindow::selectFreenect2()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcFreenect2);
}

void MainWindow::selectK4W2()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcK4W2);
}

void MainWindow::selectK4A()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcK4A);
}

void MainWindow::selectRealSense()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcRealSense);
}

void MainWindow::selectRealSense2()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcRealSense2);
}
void MainWindow::selectRealSense2L515()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcRealSense2, 1);
}

void MainWindow::selectRealSense2Stereo()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcStereoRealSense2);
}

void MainWindow::selectStereoDC1394()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcDC1394);
}

void MainWindow::selectStereoFlyCapture2()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcFlyCapture2);
}
void MainWindow::selectStereoZed()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcStereoZed);
}
void MainWindow::selectStereoZedOC()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcStereoZedOC);
}

void MainWindow::selectStereoTara()
{
    _preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcStereoTara);
}

void MainWindow::selectStereoUsb()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcStereoUsb);
}

void MainWindow::selectMyntEyeS()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcStereoMyntEye);
}

void MainWindow::selectDepthAI()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcStereoDepthAI);
}

void MainWindow::dumpTheMemory()
{
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDumpMemory));
}

void MainWindow::dumpThePrediction()
{
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDumpPrediction));
}

void MainWindow::sendGoal()
{
	UINFO("Sending a goal...");
	bool ok = false;
	QString text = QInputDialog::getText(this, tr("Send a goal"), tr("Goal location ID or label: "), QLineEdit::Normal, "", &ok);
	if(ok && !text.isEmpty())
	{
		_waypoints.clear();
		_waypointsIndex = 0;

		this->postGoal(text);
	}
}

void MainWindow::sendWaypoints()
{
	UINFO("Sending waypoints...");
	bool ok = false;
	QString text = QInputDialog::getText(this, tr("Send waypoints"), tr("Waypoint IDs or labels (separated by spaces): "), QLineEdit::Normal, "", &ok);
	if(ok && !text.isEmpty())
	{
		QStringList wp = text.split(' ');
		if(wp.size() < 2)
		{
			QMessageBox::warning(this, tr("Send waypoints"), tr("At least two waypoints should be set. For only one goal, use send goal action."));
		}
		else
		{
			_waypoints = wp;
			_waypointsIndex = 0;
			this->postGoal(_waypoints.at(_waypointsIndex));
		}
	}
}

void MainWindow::postGoal(const QString & goal)
{
	if(!goal.isEmpty())
	{
		bool ok = false;
		int id = goal.toInt(&ok);
		_ui->graphicsView_graphView->setGlobalPath(std::vector<std::pair<int, Transform> >()); // clear
		UINFO("Posting event with goal %s", goal.toStdString().c_str());
		if(ok)
		{
			this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGoal, id));
		}
		else
		{
			this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGoal, goal.toStdString()));
		}
	}
}

void MainWindow::cancelGoal()
{
	UINFO("Cancelling goal...");
	_waypoints.clear();
	_waypointsIndex = 0;
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdCancelGoal));
}

void MainWindow::label()
{
	UINFO("Labelling current location...");
	bool ok = false;
	QString label = QInputDialog::getText(this, tr("Label current location"), tr("Label: "), QLineEdit::Normal, "", &ok);
	if(ok && !label.isEmpty())
	{
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdLabel, label.toStdString(), 0));
	}
}

void MainWindow::removeLabel()
{
	UINFO("Removing label...");
	bool ok = false;
	QString label = QInputDialog::getText(this, tr("Remove label"), tr("Label: "), QLineEdit::Normal, "", &ok);
	if(ok && !label.isEmpty())
	{
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdRemoveLabel, label.toStdString(), 0));
	}
}

void MainWindow::updateCacheFromDatabase()
{
	QString dir = getWorkingDirectory();
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), dir, tr("RTAB-Map database files (*.db)"));
	if(!path.isEmpty())
	{
		updateCacheFromDatabase(path);
	}
}

void MainWindow::updateCacheFromDatabase(const QString & path)
{
	if(!path.isEmpty())
	{
		DBDriver * driver = DBDriver::create();
		if(driver->openConnection(path.toStdString()))
		{
			UINFO("Update cache...");
			_progressDialog->resetProgress();
			_progressDialog->show();
			_progressDialog->appendText(tr("Downloading the map from \"%1\" (without poses and links)...")
					.arg(path));

			std::set<int> ids;
			driver->getAllNodeIds(ids, true);
			std::list<Signature*> signaturesList;
			driver->loadSignatures(std::list<int>(ids.begin(), ids.end()), signaturesList);
			std::map<int, Signature> signatures;
			driver->loadNodeData(signaturesList);
			for(std::list<Signature *>::iterator iter=signaturesList.begin(); iter!=signaturesList.end(); ++iter)
			{
				signatures.insert(std::make_pair((*iter)->id(), *(*iter)));
				delete *iter;
			}
			RtabmapEvent3DMap event(signatures, _currentPosesMap, _currentLinksMap);
			processRtabmapEvent3DMap(event);
		}
		else
		{
			QMessageBox::warning(this, tr("Update cache"), tr("Failed to open database \"%1\"").arg(path));
		}
		delete driver;
	}
}

void MainWindow::downloadAllClouds()
{
	QStringList items;
	items.append("Local map optimized");
	items.append("Local map not optimized");
	items.append("Global map optimized");
	items.append("Global map not optimized");

	bool ok;
	QString item = QInputDialog::getItem(this, tr("Download map"), tr("Options:"), items, 0, false, &ok);
	if(ok)
	{
		bool optimized=false, global=false;
		if(item.compare("Local map optimized") == 0)
		{
			optimized = true;
		}
		else if(item.compare("Local map not optimized") == 0)
		{

		}
		else if(item.compare("Global map optimized") == 0)
		{
			global=true;
			optimized=true;
		}
		else if(item.compare("Global map not optimized") == 0)
		{
			global=true;
		}
		else
		{
			UFATAL("Item \"%s\" not found?!?", item.toStdString().c_str());
		}

		UINFO("Download clouds...");
		_progressDialog->resetProgress();
		_progressDialog->show();
		_progressDialog->appendText(tr("Downloading the map (global=%1 ,optimized=%2)...")
				.arg(global?"true":"false").arg(optimized?"true":"false"));
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPublish3DMap, global, optimized, false));
	}
}

void MainWindow::downloadPoseGraph()
{
	QStringList items;
	items.append("Local map optimized");
	items.append("Local map not optimized");
	items.append("Global map optimized");
	items.append("Global map not optimized");

	bool ok;
	QString item = QInputDialog::getItem(this, tr("Download graph"), tr("Options:"), items, 0, false, &ok);
	if(ok)
	{
		bool optimized=false, global=false;
		if(item.compare("Local map optimized") == 0)
		{
			optimized = true;
		}
		else if(item.compare("Local map not optimized") == 0)
		{

		}
		else if(item.compare("Global map optimized") == 0)
		{
			global=true;
			optimized=true;
		}
		else if(item.compare("Global map not optimized") == 0)
		{
			global=true;
		}
		else
		{
			UFATAL("Item \"%s\" not found?!?", item.toStdString().c_str());
		}

		UINFO("Download the graph...");
		_progressDialog->resetProgress();
		_progressDialog->show();
		_progressDialog->appendText(tr("Downloading the graph (global=%1 ,optimized=%2)...")
				.arg(global?"true":"false").arg(optimized?"true":"false"));

		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPublish3DMap, global, optimized, true));
	}
}

void MainWindow::anchorCloudsToGroundTruth()
{
	this->updateMapCloud(
			std::map<int, Transform>(_currentPosesMap),
			std::multimap<int, Link>(_currentLinksMap),
			std::map<int, int>(_currentMapIds),
			std::map<int, std::string>(_currentLabels),
			std::map<int, Transform>(_currentGTPosesMap));
}

void MainWindow::clearTheCache()
{
	_cachedSignatures.clear();
	_cachedMemoryUsage = 0;
	_cachedWordsCount.clear();
	_cachedClouds.clear();
	_createdCloudsMemoryUsage = 0;
	_cachedEmptyClouds.clear();
	_previousCloud.first = 0;
	_previousCloud.second.first.first.reset();
	_previousCloud.second.first.second.reset();
	_previousCloud.second.second.reset();
	_createdScans.clear();
	_createdFeatures.clear();
	_cloudViewer->clear();
	_cloudViewer->setBackgroundColor(_cloudViewer->getDefaultBackgroundColor());
	_cloudViewer->clearTrajectory();
	_ui->widget_mapVisibility->clear();
	_currentPosesMap.clear();
	_currentGTPosesMap.clear();
	_currentLinksMap.clear();
	_currentMapIds.clear();
	_currentLabels.clear();
	_odometryCorrection = Transform::getIdentity();
	_lastOdomPose.setNull();
	_ui->statsToolBox->clear();
	//disable save cloud action
	_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(false);
	_ui->actionPost_processing->setEnabled(false);
	_ui->actionSave_point_cloud->setEnabled(false);
	_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(false);
	_ui->actionDepth_Calibration->setEnabled(false);
	_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(false);
	_ui->actionExport_octomap->setEnabled(false);
	_ui->actionView_high_res_point_cloud->setEnabled(false);
	_likelihoodCurve->clear();
	_rawLikelihoodCurve->clear();
	_posteriorCurve->clear();
	_lastId = 0;
	_lastIds.clear();
	_firstStamp = 0.0f;
	_ui->label_stats_loopClosuresDetected->setText("0");
	_ui->label_stats_loopClosuresReactivatedDetected->setText("0");
	_ui->label_stats_loopClosuresRejected->setText("0");
	_refIds.clear();
	_loopClosureIds.clear();
	_cachedLocalizationsCount.clear();
	_ui->label_refId->clear();
	_ui->label_matchId->clear();
	_ui->graphicsView_graphView->clearAll();
	_ui->imageView_source->clear();
	_ui->imageView_loopClosure->clear();
	_ui->imageView_odometry->clear();
	_ui->imageView_source->setBackgroundColor(_ui->imageView_source->getDefaultBackgroundColor());
	_ui->imageView_loopClosure->setBackgroundColor(_ui->imageView_loopClosure->getDefaultBackgroundColor());
	_ui->imageView_odometry->setBackgroundColor(_ui->imageView_odometry->getDefaultBackgroundColor());
	_multiSessionLocWidget->clear();
#ifdef RTABMAP_OCTOMAP
	// re-create one if the resolution has changed
	UASSERT(_octomap != 0);
	delete _octomap;
	_octomap = new OctoMap(_preferencesDialog->getAllParameters());
#endif
	_occupancyGrid->clear();
	_rectCameraModels.clear();
	_rectCameraModelsOdom.clear();
}

void MainWindow::openHelp()
{
	if(_state == kMonitoringPaused || _state == kMonitoring)
	{
		QDesktopServices::openUrl(QUrl("http://wiki.ros.org/rtabmap_ros"));
	}
	else
	{
		QDesktopServices::openUrl(QUrl("https://github.com/introlab/rtabmap/wiki"));
	}
}

void MainWindow::updateElapsedTime()
{
	if(_state == kDetecting || _state == kMonitoring)
	{
		QString format = "hh:mm:ss";
		_ui->label_elapsedTime->setText((QTime().fromString(_ui->label_elapsedTime->text(), format).addMSecs(_elapsedTime->restart())).toString(format));
	}
}

void MainWindow::saveFigures()
{
	QList<int> curvesPerFigure;
	QStringList curveNames;
	_ui->statsToolBox->getFiguresSetup(curvesPerFigure, curveNames);

	QStringList curvesPerFigureStr;
	for(int i=0; i<curvesPerFigure.size(); ++i)
	{
		curvesPerFigureStr.append(QString::number(curvesPerFigure[i]));
	}
	for(int i=0; i<curveNames.size(); ++i)
	{
		curveNames[i].replace(' ', '_');
	}
	_preferencesDialog->saveCustomConfig("Figures", "counts", curvesPerFigureStr.join(" "));
	_preferencesDialog->saveCustomConfig("Figures", "curves", curveNames.join(" "));
}

void MainWindow::loadFigures()
{
	QString curvesPerFigure = _preferencesDialog->loadCustomConfig("Figures", "counts");
	QString curveNames = _preferencesDialog->loadCustomConfig("Figures", "curves");

	if(!curvesPerFigure.isEmpty())
	{
		QStringList curvesPerFigureList = curvesPerFigure.split(" ");
		QStringList curvesNamesList = curveNames.split(" ");

		int j=0;
		for(int i=0; i<curvesPerFigureList.size(); ++i)
		{
			bool ok = false;
			int count = curvesPerFigureList[i].toInt(&ok);
			if(!ok)
			{
				QMessageBox::warning(this, "Loading failed", "Corrupted figures setup...");
				break;
			}
			else
			{
				_ui->statsToolBox->addCurve(curvesNamesList[j++].replace('_', ' '));
				for(int k=1; k<count && j<curveNames.size(); ++k)
				{
					_ui->statsToolBox->addCurve(curvesNamesList[j++].replace('_', ' '), false);
				}
			}
		}
	}

}

void MainWindow::openPreferences()
{
	_preferencesDialog->setMonitoringState(_state == kMonitoring || _state == kMonitoringPaused);
	_preferencesDialog->exec();
}

void MainWindow::openPreferencesSource()
{
	_preferencesDialog->setCurrentPanelToSource();
	openPreferences();
	this->updateSelectSourceMenu();
}

void MainWindow::setDefaultViews()
{
	_ui->dockWidget_posterior->setVisible(false);
	_ui->dockWidget_likelihood->setVisible(false);
	_ui->dockWidget_rawlikelihood->setVisible(false);
	_ui->dockWidget_statsV2->setVisible(false);
	_ui->dockWidget_console->setVisible(false);
	_ui->dockWidget_loopClosureViewer->setVisible(false);
	_ui->dockWidget_mapVisibility->setVisible(false);
	_ui->dockWidget_graphViewer->setVisible(false);
	_ui->dockWidget_odometry->setVisible(true);
	_ui->dockWidget_cloudViewer->setVisible(true);
	_ui->dockWidget_imageView->setVisible(true);
	_ui->dockWidget_multiSessionLoc->setVisible(false);
	_ui->toolBar->setVisible(_state != kMonitoring && _state != kMonitoringPaused);
	_ui->toolBar_2->setVisible(true);
	_ui->statusbar->setVisible(false);
	this->setAspectRatio720p();
	_cloudViewer->resetCamera();
	_cloudViewer->setCameraLockZ(true);
	_cloudViewer->setCameraTargetFollow(true);
}

void MainWindow::selectScreenCaptureFormat(bool checked)
{
	if(checked)
	{
		QStringList items;
		items << QString("Synchronize with map update") << QString("Synchronize with odometry update");
		bool ok;
		QString item = QInputDialog::getItem(this, tr("Select synchronization behavior"), tr("Sync:"), items, 0, false, &ok);
		if(ok && !item.isEmpty())
		{
			if(item.compare("Synchronize with map update") == 0)
			{
				_autoScreenCaptureOdomSync = false;
			}
			else
			{
				_autoScreenCaptureOdomSync = true;
			}

			if(_state != kMonitoring && _state != kMonitoringPaused)
			{
				int r = QMessageBox::question(this, tr("Hard drive or RAM?"), tr("Save in RAM? Images will be saved on disk when clicking auto screen capture again."), QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
				if(r == QMessageBox::No || r == QMessageBox::Yes)
				{
					_autoScreenCaptureRAM = r == QMessageBox::Yes;
				}
				else
				{
					_ui->actionAuto_screen_capture->setChecked(false);
				}

				r = QMessageBox::question(this, tr("Save in JPEG?"), tr("Save in JPEG format? Otherwise they are saved in PNG."), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
				if(r == QMessageBox::No || r == QMessageBox::Yes)
				{
					_autoScreenCapturePNG = r == QMessageBox::No;
				}
				else
				{
					_ui->actionAuto_screen_capture->setChecked(false);
				}
			}
		}
		else
		{
			_ui->actionAuto_screen_capture->setChecked(false);
		}
	}
	else if(_autoScreenCaptureCachedImages.size())
	{
		QString targetDir = _preferencesDialog->getWorkingDirectory() + QDir::separator() + "ScreensCaptured";
		QDir dir;
		if(!dir.exists(targetDir))
		{
			dir.mkdir(targetDir);
		}
		targetDir += QDir::separator();
		targetDir += "Main_window";
		if(!dir.exists(targetDir))
		{
			dir.mkdir(targetDir);
		}
		targetDir += QDir::separator();

		_progressDialog->setCancelButtonVisible(true);
		_progressDialog->resetProgress();
		_progressDialog->show();
		_progressDialog->setMaximumSteps(_autoScreenCaptureCachedImages.size());
		int i=0;
		for(QMap<QString, QByteArray>::iterator iter=_autoScreenCaptureCachedImages.begin(); iter!=_autoScreenCaptureCachedImages.end() && !_progressDialog->isCanceled(); ++iter)
		{
			QPixmap figure;
			figure.loadFromData(iter.value(), _autoScreenCapturePNG?"PNG":"JPEG");
			figure.save(targetDir + iter.key(), _autoScreenCapturePNG?"PNG":"JPEG");
			_progressDialog->appendText(tr("Saved image \"%1\" (%2/%3).").arg(targetDir + iter.key()).arg(++i).arg(_autoScreenCaptureCachedImages.size()));
			_progressDialog->incrementStep();
			QApplication::processEvents();
		}
		_autoScreenCaptureCachedImages.clear();
		_progressDialog->setValue(_progressDialog->maximumSteps());
		_progressDialog->setCancelButtonVisible(false);
	}
}

void MainWindow::takeScreenshot()
{
	QDesktopServices::openUrl(QUrl::fromLocalFile(this->captureScreen()));
}

void MainWindow::setAspectRatio(int w, int h)
{
	QRect rect = this->geometry();
	if(h<100 && w<100)
	{
		// it is a ratio
		if(float(rect.width())/float(rect.height()) > float(w)/float(h))
		{
			rect.setWidth(w*(rect.height()/h));
			rect.setHeight((rect.height()/h)*h);
		}
		else
		{
			rect.setHeight(h*(rect.width()/w));
			rect.setWidth((rect.width()/w)*w);
		}
	}
	else
	{
		// it is absolute size
		rect.setWidth(w);
		rect.setHeight(h);
	}
	this->setGeometry(rect);
}

void MainWindow::setAspectRatio16_9()
{
	this->setAspectRatio(16, 9);
}

void MainWindow::setAspectRatio16_10()
{
	this->setAspectRatio(16, 10);
}

void MainWindow::setAspectRatio4_3()
{
	this->setAspectRatio(4, 3);
}

void MainWindow::setAspectRatio240p()
{
	this->setAspectRatio((240*16)/9, 240);
}

void MainWindow::setAspectRatio360p()
{
	this->setAspectRatio((360*16)/9, 360);
}

void MainWindow::setAspectRatio480p()
{
	this->setAspectRatio((480*16)/9, 480);
}

void MainWindow::setAspectRatio720p()
{
	this->setAspectRatio((720*16)/9, 720);
}

void MainWindow::setAspectRatio1080p()
{
	this->setAspectRatio((1080*16)/9, 1080);
}

void MainWindow::setAspectRatioCustom()
{
	bool ok;
	int width = QInputDialog::getInt(this, tr("Aspect ratio"), tr("Width (pixels):"), this->geometry().width(), 100, 10000, 100, &ok);
	if(ok)
	{
		int height = QInputDialog::getInt(this, tr("Aspect ratio"), tr("Height (pixels):"), this->geometry().height(), 100, 10000, 100, &ok);
		if(ok)
		{
			this->setAspectRatio(width, height);
		}
	}
}

void MainWindow::exportGridMap()
{
	float gridCellSize = 0.05f;
	bool ok;
	gridCellSize = (float)QInputDialog::getDouble(this, tr("Grid cell size"), tr("Size (m):"), (double)gridCellSize, 0.01, 1, 2, &ok);
	if(!ok)
	{
		return;
	}

	// create the map
	float xMin=0.0f, yMin=0.0f;
	cv::Mat pixels;
#ifdef RTABMAP_OCTOMAP
		if(_preferencesDialog->isOctomap2dGrid())
		{
			pixels = _octomap->createProjectionMap(xMin, yMin, gridCellSize, 0);

		}
		else
#endif
		{
			pixels = _occupancyGrid->getMap(xMin, yMin);
		}

	if(!pixels.empty())
	{
		cv::Mat map8U(pixels.rows, pixels.cols, CV_8U);
		//convert to gray scaled map
		for (int i = 0; i < pixels.rows; ++i)
		{
			for (int j = 0; j < pixels.cols; ++j)
			{
				char v = pixels.at<char>(i, j);
				unsigned char gray;
				if(v == 0)
				{
					gray = 178;
				}
				else if(v == 100)
				{
					gray = 0;
				}
				else // -1
				{
					gray = 89;
				}
				map8U.at<unsigned char>(i, j) = gray;
			}
		}

		QImage image = uCvMat2QImage(map8U, false);

		QString path = QFileDialog::getSaveFileName(this, tr("Save to ..."), "grid.png", tr("Image (*.png *.bmp)"));
		if(!path.isEmpty())
		{
			if(QFileInfo(path).suffix() != "png" && QFileInfo(path).suffix() != "bmp")
			{
				//use png by default
				path += ".png";
			}

			QImage img = image.mirrored(false, true).transformed(QTransform().rotate(-90));
			QPixmap::fromImage(img).save(path);

			QDesktopServices::openUrl(QUrl::fromLocalFile(path));
		}
	}
}

void MainWindow::exportClouds()
{
	if(_exportCloudsDialog->isVisible())
	{
		return;
	}

	std::map<int, Transform> poses = _ui->widget_mapVisibility->getVisiblePoses();

	// Use ground truth poses if current clouds are using them
	if(_currentGTPosesMap.size() && _ui->actionAnchor_clouds_to_ground_truth->isChecked())
	{
		for(std::map<int, Transform>::iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			std::map<int, Transform>::iterator gtIter = _currentGTPosesMap.find(iter->first);
			if(gtIter!=_currentGTPosesMap.end())
			{
				iter->second = gtIter->second;
			}
			else
			{
				UWARN("Not found ground truth pose for node %d", iter->first);
			}
		}
	}

	_exportCloudsDialog->exportClouds(
			poses,
			_currentLinksMap,
			_currentMapIds,
			_cachedSignatures,
			_cachedClouds,
			_createdScans,
			_preferencesDialog->getWorkingDirectory(),
			_preferencesDialog->getAllParameters());
}

void MainWindow::viewClouds()
{
	if(_exportCloudsDialog->isVisible())
	{
		return;
	}

	std::map<int, Transform> poses = _ui->widget_mapVisibility->getVisiblePoses();

	// Use ground truth poses if current clouds are using them
	if(_currentGTPosesMap.size() && _ui->actionAnchor_clouds_to_ground_truth->isChecked())
	{
		for(std::map<int, Transform>::iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			std::map<int, Transform>::iterator gtIter = _currentGTPosesMap.find(iter->first);
			if(gtIter!=_currentGTPosesMap.end())
			{
				iter->second = gtIter->second;
			}
			else
			{
				UWARN("Not found ground truth pose for node %d", iter->first);
			}
		}
	}

	_exportCloudsDialog->viewClouds(
			poses,
			_currentLinksMap,
			_currentMapIds,
			_cachedSignatures,
			_cachedClouds,
			_createdScans,
			_preferencesDialog->getWorkingDirectory(),
			_preferencesDialog->getAllParameters());

}

void MainWindow::exportOctomap()
{
#ifdef RTABMAP_OCTOMAP
	if(_octomap->octree()->size())
	{
		QString path = QFileDialog::getSaveFileName(
				this,
				tr("Save File"),
				this->getWorkingDirectory()+"/"+"octomap.bt",
				tr("Octomap file (*.bt)"));

		if(!path.isEmpty())
		{
			if(_octomap->writeBinary(path.toStdString()))
			{
				QMessageBox::information(this,
						tr("Export octomap..."),
						tr("Octomap successfully saved to \"%1\".")
						.arg(path));
			}
			else
			{
				QMessageBox::information(this,
						tr("Export octomap..."),
						tr("Failed to save octomap to \"%1\"!")
						.arg(path));
			}
		}
	}
	else
	{
		UERROR("Empty octomap.");
	}
#else
	UERROR("Cannot export octomap, RTAB-Map is not built with it.");
#endif
}

void MainWindow::exportImages()
{
	if(_cachedSignatures.empty())
	{
		QMessageBox::warning(this, tr("Export images..."), tr("Cannot export images, the cache is empty!"));
		return;
	}
	std::map<int, Transform> poses = _ui->widget_mapVisibility->getVisiblePoses();

	if(poses.empty())
	{
		QMessageBox::warning(this, tr("Export images..."), tr("There is no map!"));
		return;
	}

	QStringList formats;
	formats.push_back("id.jpg");
	formats.push_back("id.png");
	formats.push_back("timestamp.jpg");
	formats.push_back("timestamp.png");
	bool ok;
	QString format = QInputDialog::getItem(this, tr("Which RGB format?"), tr("Format:"), formats, 0, false, &ok);
	if(!ok)
	{
		return;
	}
	QString ext = format.split('.').back();
	bool useStamp = format.split('.').front().compare("timestamp") == 0;

	QMap<int, double> stamps;
	QString path = QFileDialog::getExistingDirectory(this, tr("Select directory where to save images..."), this->getWorkingDirectory());
	if(!path.isEmpty())
	{
		SensorData data;
		if(_cachedSignatures.contains(poses.rbegin()->first))
		{
			data = _cachedSignatures.value(poses.rbegin()->first).sensorData();
			data.uncompressData();
		}

		_progressDialog->resetProgress();
		_progressDialog->show();
		_progressDialog->setMaximumSteps(_cachedSignatures.size());

		unsigned int saved = 0;
		bool calibrationSaved = false;
		for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			QString id = QString::number(iter->first);

			SensorData data;
			if(_cachedSignatures.contains(iter->first))
			{
				data = _cachedSignatures.value(iter->first).sensorData();
				data.uncompressData();

				if(!calibrationSaved)
				{
					if(!data.imageRaw().empty() && !data.rightRaw().empty())
					{
						QDir dir;
						dir.mkdir(QString("%1/left").arg(path));
						dir.mkdir(QString("%1/right").arg(path));
						if(data.stereoCameraModels().size() > 1)
						{
							UERROR("Only one stereo camera calibration can be saved at this time (%d detected)", (int)data.stereoCameraModels().size());
						}
						else if(data.stereoCameraModels().size() == 1 && data.stereoCameraModels().front().isValidForProjection())
						{
							std::string cameraName = "calibration";
							StereoCameraModel model(
									cameraName,
									data.imageRaw().size(),
									data.stereoCameraModels()[0].left().K(),
									data.stereoCameraModels()[0].left().D(),
									data.stereoCameraModels()[0].left().R(),
									data.stereoCameraModels()[0].left().P(),
									data.rightRaw().size(),
									data.stereoCameraModels()[0].right().K(),
									data.stereoCameraModels()[0].right().D(),
									data.stereoCameraModels()[0].right().R(),
									data.stereoCameraModels()[0].right().P(),
									data.stereoCameraModels()[0].R(),
									data.stereoCameraModels()[0].T(),
									data.stereoCameraModels()[0].E(),
									data.stereoCameraModels()[0].F(),
									data.stereoCameraModels()[0].left().localTransform());
							if(model.save(path.toStdString()))
							{
								calibrationSaved = true;
								UINFO("Saved stereo calibration \"%s\"", (path.toStdString()+"/"+cameraName).c_str());
							}
							else
							{
								UERROR("Failed saving calibration \"%s\"", (path.toStdString()+"/"+cameraName).c_str());
							}
						}
					}
					else if(!data.imageRaw().empty())
					{
						if(!data.depthRaw().empty())
						{
							QDir dir;
							dir.mkdir(QString("%1/rgb").arg(path));
							dir.mkdir(QString("%1/depth").arg(path));
						}

						if(data.cameraModels().size() > 1)
						{
							UERROR("Only one camera calibration can be saved at this time (%d detected)", (int)data.cameraModels().size());
						}
						else if(data.cameraModels().size() == 1 && data.cameraModels().front().isValidForProjection())
						{
							std::string cameraName = "calibration";
							CameraModel model(cameraName,
									data.imageRaw().size(),
									data.cameraModels().front().K(),
									data.cameraModels().front().D(),
									data.cameraModels().front().R(),
									data.cameraModels().front().P(),
									data.cameraModels().front().localTransform());
							if(model.save(path.toStdString()))
							{
								calibrationSaved = true;
								UINFO("Saved calibration \"%s\"", (path.toStdString()+"/"+cameraName).c_str());
							}
							else
							{
								UERROR("Failed saving calibration \"%s\"", (path.toStdString()+"/"+cameraName).c_str());
							}
						}
					}
				}

				if(!data.imageRaw().empty() && useStamp)
				{
					double stamp = _cachedSignatures.value(iter->first).getStamp();
					if(stamp == 0.0)
					{
						UWARN("Node %d has null timestamp! Using id instead!", iter->first);
					}
					else
					{
						id = QString::number(stamp, 'f');
					}
				}
			}
			QString info;
			bool warn = false;
			if(!data.imageRaw().empty() && !data.rightRaw().empty())
			{
				if(!cv::imwrite(QString("%1/left/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.imageRaw()))
					UWARN("Failed saving \"%s\"", QString("%1/left/%2.%3").arg(path).arg(id).arg(ext).toStdString().c_str());
				if(!cv::imwrite(QString("%1/right/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.rightRaw()))
					UWARN("Failed saving \"%s\"", QString("%1/right/%2.%3").arg(path).arg(id).arg(ext).toStdString().c_str());
				info = tr("Saved left/%1.%2 and right/%1.%2.").arg(id).arg(ext);
			}
			else if(!data.imageRaw().empty() && !data.depthRaw().empty())
			{
				if(!cv::imwrite(QString("%1/rgb/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.imageRaw()))
					UWARN("Failed saving \"%s\"", QString("%1/rgb/%2.%3").arg(path).arg(id).arg(ext).toStdString().c_str());
				if(!cv::imwrite(QString("%1/depth/%2.png").arg(path).arg(id).toStdString(), data.depthRaw().type()==CV_32FC1?util2d::cvtDepthFromFloat(data.depthRaw()):data.depthRaw()))
					UWARN("Failed saving \"%s\"", QString("%1/depth/%2.png").arg(path).arg(id).toStdString().c_str());
				info = tr("Saved rgb/%1.%2 and depth/%1.png.").arg(id).arg(ext);
			}
			else if(!data.imageRaw().empty())
			{
				if(!cv::imwrite(QString("%1/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.imageRaw()))
					UWARN("Failed saving \"%s\"", QString("%1/%2.%3").arg(path).arg(id).arg(ext).toStdString().c_str());
				else
					info = tr("Saved %1.%2.").arg(id).arg(ext);
			}
			else
			{
				info = tr("No images saved for node %1!").arg(id);
				warn = true;
			}
			saved += warn?0:1;
			_progressDialog->appendText(info, !warn?Qt::black:Qt::darkYellow);
			_progressDialog->incrementStep();
			QApplication::processEvents();

		}
		if(saved!=poses.size())
		{
			_progressDialog->setAutoClose(false);
			_progressDialog->appendText(tr("%1 images of %2 saved to \"%3\".").arg(saved).arg(poses.size()).arg(path));
		}
		else
		{
			_progressDialog->appendText(tr("%1 images saved to \"%2\".").arg(saved).arg(path));
		}

		if(!calibrationSaved)
		{
			QMessageBox::warning(this,
					tr("Export images..."),
					tr("Data in the cache don't seem to have valid calibration. Calibration file will not be saved. Try refreshing the cache (with clouds)."));
		}

		_progressDialog->setValue(_progressDialog->maximumSteps());
	}
}

void MainWindow::exportBundlerFormat()
{
	if(_exportBundlerDialog->isVisible())
	{
		return;
	}

	std::map<int, Transform> posesIn = _ui->widget_mapVisibility->getVisiblePoses();

	// Use ground truth poses if current clouds are using them
	if(_currentGTPosesMap.size() && _ui->actionAnchor_clouds_to_ground_truth->isChecked())
	{
		for(std::map<int, Transform>::iterator iter = posesIn.begin(); iter!=posesIn.end(); ++iter)
		{
			std::map<int, Transform>::iterator gtIter = _currentGTPosesMap.find(iter->first);
			if(gtIter!=_currentGTPosesMap.end())
			{
				iter->second = gtIter->second;
			}
			else
			{
				UWARN("Not found ground truth pose for node %d", iter->first);
			}
		}
	}

	std::map<int, Transform> poses;
	for(std::map<int, Transform>::iterator iter=posesIn.begin(); iter!=posesIn.end(); ++iter)
	{
		if(_cachedSignatures.contains(iter->first))
		{
			if(_cachedSignatures[iter->first].sensorData().imageRaw().empty() &&
			   _cachedSignatures[iter->first].sensorData().imageCompressed().empty())
			{
				UWARN("Missing image in cache for node %d", iter->first);
			}
			else if((_cachedSignatures[iter->first].sensorData().cameraModels().size() == 1 &&
					 _cachedSignatures[iter->first].sensorData().cameraModels().at(0).isValidForProjection()) ||
			        (_cachedSignatures[iter->first].sensorData().stereoCameraModels().size() == 1 &&
			         _cachedSignatures[iter->first].sensorData().stereoCameraModels()[0].isValidForProjection()))
			{
				poses.insert(*iter);
			}
			else
			{
				UWARN("Missing calibration for node %d", iter->first);
			}
		}
		else
		{
			UWARN("Did not find node %d in cache", iter->first);
		}
	}

	if(poses.size())
	{
		_exportBundlerDialog->exportBundler(
					poses,
					_currentLinksMap,
					_cachedSignatures,
					_preferencesDialog->getAllParameters());
	}
	else
	{
		QMessageBox::warning(this, tr("Exporting cameras..."), tr("No poses exported because of missing images. Try refreshing the cache (with clouds)."));
	}
}

void MainWindow::resetOdometry()
{
	UINFO("reset odometry");
	this->post(new OdometryResetEvent());
}

void MainWindow::triggerNewMap()
{
	UINFO("trigger a new map");
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdTriggerNewMap));
}

void MainWindow::dataRecorder()
{
	if(_dataRecorder == 0)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save to..."), _preferencesDialog->getWorkingDirectory()+"/output.db", "RTAB-Map database (*.db)");
		if(!path.isEmpty())
		{
			int r = QMessageBox::question(this, tr("Hard drive or RAM?"), tr("Save in RAM?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

			if(r == QMessageBox::No || r == QMessageBox::Yes)
			{
				bool recordInRAM = r == QMessageBox::Yes;

				_dataRecorder = new DataRecorder(this);
				_dataRecorder->setWindowFlags(Qt::Dialog);
				_dataRecorder->setAttribute(Qt::WA_DeleteOnClose, true);
				_dataRecorder->setWindowTitle(tr("Data recorder (%1)").arg(path));

				if(_dataRecorder->init(path, recordInRAM))
				{
					this->connect(_dataRecorder, SIGNAL(destroyed(QObject*)), this, SLOT(dataRecorderDestroyed()));
					_dataRecorder->show();
					_dataRecorder->registerToEventsManager();
					if(_camera)
					{
						UEventsManager::createPipe(_camera, _dataRecorder, "CameraEvent");
					}
					_ui->actionData_recorder->setEnabled(false);
				}
				else
				{
					QMessageBox::warning(this, tr(""), tr("Cannot initialize the data recorder!"));
					UERROR("Cannot initialize the data recorder!");
					delete _dataRecorder;
					_dataRecorder = 0;
				}
			}
		}
	}
	else
	{
		UERROR("Only one recorder at the same time.");
	}
}

void MainWindow::dataRecorderDestroyed()
{
	_ui->actionData_recorder->setEnabled(true);
	_dataRecorder = 0;
}

//END ACTIONS

// STATES

// in monitoring state, only some actions are enabled
void MainWindow::setMonitoringState(bool pauseChecked)
{
	this->changeState(pauseChecked?kMonitoringPaused:kMonitoring);
}

// Must be called by the GUI thread, use signal StateChanged()
void MainWindow::changeState(MainWindow::State newState)
{
	bool monitoring = newState==kMonitoring || newState == kMonitoringPaused;
	_ui->label_source->setVisible(!monitoring);
	_ui->label_stats_source->setVisible(!monitoring);
	_ui->actionNew_database->setVisible(!monitoring);
	_ui->actionOpen_database->setVisible(!monitoring);
	_ui->actionClose_database->setVisible(!monitoring);
	_ui->actionEdit_database->setVisible(!monitoring);
	_ui->actionStart->setVisible(!monitoring);
	_ui->actionStop->setVisible(!monitoring);
	_ui->actionDump_the_memory->setVisible(!monitoring);
	_ui->actionDump_the_prediction_matrix->setVisible(!monitoring);
	_ui->actionGenerate_map->setVisible(!monitoring);
	_ui->actionUpdate_cache_from_database->setVisible(monitoring);
	_ui->actionData_recorder->setVisible(!monitoring);
	_ui->menuSelect_source->menuAction()->setVisible(!monitoring);
	_ui->doubleSpinBox_stats_imgRate->setVisible(!monitoring);
	_ui->doubleSpinBox_stats_imgRate_label->setVisible(!monitoring);
	bool wasMonitoring = _state==kMonitoring || _state == kMonitoringPaused;
	if(wasMonitoring != monitoring)
	{
		_ui->toolBar->setVisible(!monitoring);
		_ui->toolBar->toggleViewAction()->setVisible(!monitoring);
	}
	QList<QAction*> actions = _ui->menuTools->actions();
	for(int i=0; i<actions.size(); ++i)
	{
		if(actions.at(i)->isSeparator())
		{
			actions.at(i)->setVisible(!monitoring);
		}
	}
	actions = _ui->menuFile->actions();
	if(actions.size()==16)
	{
		if(actions.at(2)->isSeparator())
		{
			actions.at(2)->setVisible(!monitoring);
		}
		else
		{
			UWARN("Menu File separators have not the same order.");
		}
		if(actions.at(12)->isSeparator())
		{
			actions.at(12)->setVisible(!monitoring);
		}
		else
		{
			UWARN("Menu File separators have not the same order.");
		}
	}
	else
	{
		UWARN("Menu File actions size has changed (%d)", actions.size());
	}
	actions = _ui->menuProcess->actions();
	if(actions.size()>=2)
	{
		if(actions.at(1)->isSeparator())
		{
			actions.at(1)->setVisible(!monitoring);
		}
		else
		{
			UWARN("Menu File separators have not the same order.");
		}
	}
	else
	{
		UWARN("Menu File separators have not the same order.");
	}

	_ui->actionAnchor_clouds_to_ground_truth->setEnabled(!_currentGTPosesMap.empty());

	switch (newState)
	{
	case kIdle: // RTAB-Map is not initialized yet
		_ui->actionNew_database->setEnabled(true);
		_ui->actionOpen_database->setEnabled(true);
		_ui->actionClose_database->setEnabled(false);
		_ui->actionEdit_database->setEnabled(true);
		_ui->actionStart->setEnabled(false);
		_ui->actionPause->setEnabled(false);
		_ui->actionPause->setChecked(false);
		_ui->actionPause->setToolTip(tr("Pause"));
		_ui->actionStop->setEnabled(false);
		_ui->actionPause_on_match->setEnabled(true);
		_ui->actionPause_on_local_loop_detection->setEnabled(true);
		_ui->actionPause_when_a_loop_hypothesis_is_rejected->setEnabled(true);
		_ui->actionDump_the_memory->setEnabled(false);
		_ui->actionDump_the_prediction_matrix->setEnabled(false);
		_ui->actionDelete_memory->setEnabled(false);
		_ui->actionPost_processing->setEnabled(_cachedSignatures.size() >= 2 && _currentPosesMap.size() >= 2 && _currentLinksMap.size() >= 1);
		_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionGenerate_map->setEnabled(false);
		_ui->menuExport_poses->setEnabled(!_currentPosesMap.empty());
		_ui->actionSave_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionView_high_res_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(!_occupancyGrid->addedNodes().empty());
#ifdef RTABMAP_OCTOMAP
		_ui->actionExport_octomap->setEnabled(_octomap->octree()->size());
#else
		_ui->actionExport_octomap->setEnabled(false);
#endif
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionDepth_Calibration->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionDownload_all_clouds->setEnabled(false);
		_ui->actionDownload_graph->setEnabled(false);
		_ui->menuSelect_source->setEnabled(true);
		_ui->actionLabel_current_location->setEnabled(false);
		_ui->actionSend_goal->setEnabled(false);
		_ui->actionCancel_goal->setEnabled(false);
		_ui->toolBar->findChild<QAction*>("toolbar_source")->setEnabled(true);
		_ui->actionTrigger_a_new_map->setEnabled(false);
		_ui->doubleSpinBox_stats_imgRate->setEnabled(true);
		_ui->statusbar->clearMessage();
		_state = newState;
		_oneSecondTimer->stop();
		break;

	case kApplicationClosing:
	case kClosing:
		_ui->actionStart->setEnabled(false);
		_ui->actionPause->setEnabled(false);
		_ui->actionStop->setEnabled(false);
		_state = newState;
		break;

	case kInitializing:
		_ui->actionNew_database->setEnabled(false);
		_ui->actionOpen_database->setEnabled(false);
		_ui->actionClose_database->setEnabled(false);
		_ui->actionEdit_database->setEnabled(false);
		_state = newState;
		break;

	case kInitialized:
		_ui->actionNew_database->setEnabled(false);
		_ui->actionOpen_database->setEnabled(false);
		_ui->actionClose_database->setEnabled(true);
		_ui->actionEdit_database->setEnabled(false);
		_ui->actionStart->setEnabled(true);
		_ui->actionPause->setEnabled(false);
		_ui->actionPause->setChecked(false);
		_ui->actionPause->setToolTip(tr("Pause"));
		_ui->actionStop->setEnabled(false);
		_ui->actionPause_on_match->setEnabled(true);
		_ui->actionPause_on_local_loop_detection->setEnabled(true);
		_ui->actionPause_when_a_loop_hypothesis_is_rejected->setEnabled(true);
		_ui->actionDump_the_memory->setEnabled(true);
		_ui->actionDump_the_prediction_matrix->setEnabled(true);
		_ui->actionDelete_memory->setEnabled(_openedDatabasePath.isEmpty());
		_ui->actionPost_processing->setEnabled(_cachedSignatures.size() >= 2 && _currentPosesMap.size() >= 2 && _currentLinksMap.size() >= 1);
		_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionGenerate_map->setEnabled(true);
		_ui->menuExport_poses->setEnabled(!_currentPosesMap.empty());
		_ui->actionSave_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionView_high_res_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(!_occupancyGrid->addedNodes().empty());
#ifdef RTABMAP_OCTOMAP
		_ui->actionExport_octomap->setEnabled(_octomap->octree()->size());
#else
		_ui->actionExport_octomap->setEnabled(false);
#endif
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionDepth_Calibration->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->actionDownload_graph->setEnabled(true);
		_ui->menuSelect_source->setEnabled(true);
		_ui->actionLabel_current_location->setEnabled(true);
		_ui->actionSend_goal->setEnabled(true);
		_ui->actionCancel_goal->setEnabled(true);
		_ui->toolBar->findChild<QAction*>("toolbar_source")->setEnabled(true);
		_ui->actionTrigger_a_new_map->setEnabled(true);
		_ui->doubleSpinBox_stats_imgRate->setEnabled(true);
		_ui->statusbar->clearMessage();
		_state = newState;
		_oneSecondTimer->stop();
		break;

	case kStartingDetection:
		_ui->actionStart->setEnabled(false);
		_state = newState;
		break;

	case kDetecting:
		_ui->actionNew_database->setEnabled(false);
		_ui->actionOpen_database->setEnabled(false);
		_ui->actionClose_database->setEnabled(false);
		_ui->actionEdit_database->setEnabled(false);
		_ui->actionStart->setEnabled(false);
		_ui->actionPause->setEnabled(true);
		_ui->actionPause->setChecked(false);
		_ui->actionPause->setToolTip(tr("Pause"));
		_ui->actionStop->setEnabled(true);
		_ui->actionPause_on_match->setEnabled(true);
		_ui->actionPause_on_local_loop_detection->setEnabled(true);
		_ui->actionPause_when_a_loop_hypothesis_is_rejected->setEnabled(true);
		_ui->actionDump_the_memory->setEnabled(false);
		_ui->actionDump_the_prediction_matrix->setEnabled(false);
		_ui->actionDelete_memory->setEnabled(false);
		_ui->actionPost_processing->setEnabled(false);
		_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(false);
		_ui->actionGenerate_map->setEnabled(false);
		_ui->menuExport_poses->setEnabled(false);
		_ui->actionSave_point_cloud->setEnabled(false);
		_ui->actionView_high_res_point_cloud->setEnabled(false);
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(false);
		_ui->actionExport_octomap->setEnabled(false);
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(false);
		_ui->actionDepth_Calibration->setEnabled(false);
		_ui->actionDownload_all_clouds->setEnabled(false);
		_ui->actionDownload_graph->setEnabled(false);
		_ui->menuSelect_source->setEnabled(false);
		_ui->actionLabel_current_location->setEnabled(true);
		_ui->actionSend_goal->setEnabled(true);
		_ui->actionCancel_goal->setEnabled(true);
		_ui->toolBar->findChild<QAction*>("toolbar_source")->setEnabled(false);
		_ui->actionTrigger_a_new_map->setEnabled(true);
		_ui->doubleSpinBox_stats_imgRate->setEnabled(true);
		_ui->statusbar->showMessage(tr("Detecting..."));
		_state = newState;
		_ui->label_elapsedTime->setText("00:00:00");
		_elapsedTime->start();
		_oneSecondTimer->start();

		_databaseUpdated = true; // if a new database is used, it won't be empty anymore...

		if(_camera)
		{
			_camera->start();
			if(_imuThread)
			{
				_imuThread->start();
			}
			ULogger::setTreadIdFilter(_preferencesDialog->getGeneralLoggerThreads());
		}
		break;

	case kPaused:
		if(_state == kPaused)
		{
			_ui->actionPause->setToolTip(tr("Pause"));
			_ui->actionPause->setChecked(false);
			_ui->statusbar->showMessage(tr("Detecting..."));
			_ui->actionDump_the_memory->setEnabled(false);
			_ui->actionDump_the_prediction_matrix->setEnabled(false);
			_ui->actionDelete_memory->setEnabled(false);
			_ui->actionPost_processing->setEnabled(false);
			_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(false);
			_ui->actionGenerate_map->setEnabled(false);
			_ui->menuExport_poses->setEnabled(false);
			_ui->actionSave_point_cloud->setEnabled(false);
			_ui->actionView_high_res_point_cloud->setEnabled(false);
			_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(false);
			_ui->actionExport_octomap->setEnabled(false);
			_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(false);
			_ui->actionDepth_Calibration->setEnabled(false);
			_ui->actionDownload_all_clouds->setEnabled(false);
			_ui->actionDownload_graph->setEnabled(false);
			_state = kDetecting;
			_elapsedTime->start();
			_oneSecondTimer->start();

			if(_camera)
			{
				_camera->start();
				if(_imuThread)
				{
					_imuThread->start();
				}
				ULogger::setTreadIdFilter(_preferencesDialog->getGeneralLoggerThreads());
			}
		}
		else if(_state == kDetecting)
		{
			_ui->actionPause->setToolTip(tr("Continue (shift-click for step-by-step)"));
			_ui->actionPause->setChecked(true);
			_ui->statusbar->showMessage(tr("Paused..."));
			_ui->actionDump_the_memory->setEnabled(true);
			_ui->actionDump_the_prediction_matrix->setEnabled(true);
			_ui->actionDelete_memory->setEnabled(false);
			_ui->actionPost_processing->setEnabled(_cachedSignatures.size() >= 2 && _currentPosesMap.size() >= 2 && _currentLinksMap.size() >= 1);
			_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
			_ui->actionGenerate_map->setEnabled(true);
			_ui->menuExport_poses->setEnabled(!_currentPosesMap.empty());
			_ui->actionSave_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
			_ui->actionView_high_res_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
			_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(!_occupancyGrid->addedNodes().empty());
#ifdef RTABMAP_OCTOMAP
			_ui->actionExport_octomap->setEnabled(_octomap->octree()->size());
#else
			_ui->actionExport_octomap->setEnabled(false);
#endif
			_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
			_ui->actionDepth_Calibration->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
			_ui->actionDownload_all_clouds->setEnabled(true);
			_ui->actionDownload_graph->setEnabled(true);
			_state = kPaused;
			_oneSecondTimer->stop();

			// kill sensors
			if(_camera)
			{
				if(_imuThread)
				{
					_imuThread->join(true);
				}
				_camera->join(true);
			}
		}
		break;
	case kMonitoring:
		_ui->actionPause->setEnabled(true);
		_ui->actionPause->setChecked(false);
		_ui->actionPause->setToolTip(tr("Pause"));
		_ui->actionPause_on_match->setEnabled(true);
		_ui->actionPause_on_local_loop_detection->setEnabled(true);
		_ui->actionPause_when_a_loop_hypothesis_is_rejected->setEnabled(true);
		_ui->actionReset_Odometry->setEnabled(true);
		_ui->actionPost_processing->setEnabled(false);
		_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(false);
		_ui->menuExport_poses->setEnabled(false);
		_ui->actionSave_point_cloud->setEnabled(false);
		_ui->actionView_high_res_point_cloud->setEnabled(false);
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(false);
		_ui->actionExport_octomap->setEnabled(false);
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(false);
		_ui->actionDepth_Calibration->setEnabled(false);
		_ui->actionDelete_memory->setEnabled(true);
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->actionDownload_graph->setEnabled(true);
		_ui->actionTrigger_a_new_map->setEnabled(true);
		_ui->actionLabel_current_location->setEnabled(true);
		_ui->actionSend_goal->setEnabled(true);
		_ui->actionCancel_goal->setEnabled(true);
		_ui->statusbar->showMessage(tr("Monitoring..."));
		_state = newState;
		_elapsedTime->start();
		_oneSecondTimer->start();
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdResume));
		break;
	case kMonitoringPaused:
		_ui->actionPause->setToolTip(tr("Continue"));
		_ui->actionPause->setChecked(true);
		_ui->actionPause->setEnabled(true);
		_ui->actionPause_on_match->setEnabled(true);
		_ui->actionPause_on_local_loop_detection->setEnabled(true);
		_ui->actionPause_when_a_loop_hypothesis_is_rejected->setEnabled(true);
		_ui->actionReset_Odometry->setEnabled(true);
		_ui->actionPost_processing->setEnabled(_cachedSignatures.size() >= 2 && _currentPosesMap.size() >= 2 && _currentLinksMap.size() >= 1);
		_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->menuExport_poses->setEnabled(!_currentPosesMap.empty());
		_ui->actionSave_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionView_high_res_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(!_occupancyGrid->addedNodes().empty());
#ifdef RTABMAP_OCTOMAP
		_ui->actionExport_octomap->setEnabled(_octomap->octree()->size());
#else
		_ui->actionExport_octomap->setEnabled(false);
#endif
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionDepth_Calibration->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionDelete_memory->setEnabled(true);
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->actionDownload_graph->setEnabled(true);
		_ui->actionTrigger_a_new_map->setEnabled(true);
		_ui->actionLabel_current_location->setEnabled(true);
		_ui->actionSend_goal->setEnabled(true);
		_ui->actionCancel_goal->setEnabled(true);
		_ui->statusbar->showMessage(tr("Monitoring paused..."));
		_state = newState;
		_oneSecondTimer->stop();
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPause));
		break;
	default:
		break;
	}

}

}
