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
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/ParamEvent.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/DBDriver.h"
#include "rtabmap/core/RegistrationVis.h"

#include "rtabmap/gui/ImageView.h"
#include "rtabmap/gui/KeypointItem.h"
#include "rtabmap/gui/DataRecorder.h"
#include "rtabmap/gui/DatabaseViewer.h"
#include "rtabmap/gui/PdfPlot.h"
#include "rtabmap/gui/StatsToolBox.h"
#include "rtabmap/gui/ProgressDialog.h"
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/gui/LoopClosureViewer.h"

#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include "rtabmap/utilite/UPlot.h"
#include "rtabmap/utilite/UCv2Qt.h"

#include "ExportCloudsDialog.h"
#include "ExportScansDialog.h"
#include "AboutDialog.h"
#include "PostProcessingDialog.h"

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
#include <QtCore/QThread>
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
#include "rtabmap/core/Optimizer.h"
#include "rtabmap/core/OptimizerCVSBA.h"
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

#define LOG_FILE_NAME "LogRtabmap.txt"
#define SHARE_SHOW_LOG_FILE "share/rtabmap/showlogs.m"
#define SHARE_GET_PRECISION_RECALL_FILE "share/rtabmap/getPrecisionRecall.m"
#define SHARE_IMPORT_FILE   "share/rtabmap/importfile.m"

using namespace rtabmap;

inline static void initGuiResource() { Q_INIT_RESOURCE(GuiLib); }


namespace rtabmap {

MainWindow::MainWindow(PreferencesDialog * prefDialog, QWidget * parent) :
	QMainWindow(parent),
	_ui(0),
	_state(kIdle),
	_camera(0),
	_odomThread(0),
	_preferencesDialog(0),
	_aboutDialog(0),
	_exportCloudsDialog(0),
	_exportScansDialog(0),
	_dataRecorder(0),
	_lastId(0),
	_firstStamp(0.0f),
	_processingStatistics(false),
	_processingDownloadedMap(false),
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
	_octomap(0),
	_odometryCorrection(Transform::getIdentity()),
	_processingOdometry(false),
	_oneSecondTimer(0),
	_elapsedTime(0),
	_posteriorCurve(0),
	_likelihoodCurve(0),
	_rawLikelihoodCurve(0),
	_autoScreenCaptureOdomSync(false),
	_autoScreenCaptureRAM(false),
	_firstCall(true)
{
	UDEBUG("");

	initGuiResource();

	QPixmap pixmap(":images/RTAB-Map.png");
	QSplashScreen splash(pixmap);
	splash.show();
	splash.showMessage(tr("Loading..."));
	QApplication::processEvents();

	// Create dialogs
	_aboutDialog = new AboutDialog(this);
	_aboutDialog->setObjectName("AboutDialog");
	_exportCloudsDialog = new ExportCloudsDialog(this);
	_exportCloudsDialog->setObjectName("ExportCloudsDialog");
	_exportScansDialog = new ExportScansDialog(this);
	_exportScansDialog->setObjectName("ExportScansDialog");
	_postProcessingDialog = new PostProcessingDialog(this);
	_postProcessingDialog->setObjectName("PostProcessingDialog");

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
	_preferencesDialog->loadWindowGeometry(_exportScansDialog);
	_preferencesDialog->loadWindowGeometry(_postProcessingDialog);
	_preferencesDialog->loadWindowGeometry(_aboutDialog);
	setupMainLayout(_preferencesDialog->isVerticalLayoutUsed());

#ifdef RTABMAP_OCTOMAP
	_octomap = new OctoMap(_preferencesDialog->getGridMapResolution());
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
	_ui->imageView_source->setBackgroundColor(Qt::black);
	_ui->imageView_loopClosure->setBackgroundColor(Qt::black);
	_ui->imageView_odometry->setBackgroundColor(Qt::black);
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
	connect(this, SIGNAL(loopClosureThrChanged(float)), tc, SLOT(setThreshold(float)));

	_likelihoodCurve = new PdfPlotCurve("Likelihood", &_cachedSignatures, this);
	_ui->likelihoodPlot->addCurve(_likelihoodCurve, false);
	_ui->likelihoodPlot->showLegend(false);

	_rawLikelihoodCurve = new PdfPlotCurve("Likelihood", &_cachedSignatures, this);
	_ui->rawLikelihoodPlot->addCurve(_rawLikelihoodCurve, false);
	_ui->rawLikelihoodPlot->showLegend(false);

	_initProgressDialog = new ProgressDialog(this);
	_initProgressDialog->setWindowTitle(tr("Progress dialog"));
	_initProgressDialog->setMinimumWidth(800);

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
	_ui->menuShow_view->addAction(_ui->toolBar->toggleViewAction());
	_ui->toolBar->setWindowTitle(tr("File toolbar"));
	_ui->menuShow_view->addAction(_ui->toolBar_2->toggleViewAction());
	_ui->toolBar_2->setWindowTitle(tr("Control toolbar"));
	QAction * a = _ui->menuShow_view->addAction("Progress dialog");
	a->setCheckable(false);
	connect(a, SIGNAL(triggered(bool)), _initProgressDialog, SLOT(show()));
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
	connect(_ui->actionClear_cache, SIGNAL(triggered()), this, SLOT(clearTheCache()));
	connect(_ui->actionAbout, SIGNAL(triggered()), _aboutDialog , SLOT(exec()));
	connect(_ui->actionPrint_loop_closure_IDs_to_console, SIGNAL(triggered()), this, SLOT(printLoopClosureIds()));
	connect(_ui->actionGenerate_map, SIGNAL(triggered()), this , SLOT(generateGraphDOT()));
	connect(_ui->actionRaw_format_txt, SIGNAL(triggered()), this , SLOT(exportPosesRaw()));
	connect(_ui->actionRGBD_SLAM_format_txt, SIGNAL(triggered()), this , SLOT(exportPosesRGBDSLAM()));
	connect(_ui->actionKITTI_format_txt, SIGNAL(triggered()), this , SLOT(exportPosesKITTI()));
	connect(_ui->actionTORO_graph, SIGNAL(triggered()), this , SLOT(exportPosesTORO()));
	connect(_ui->actionG2o_g2o, SIGNAL(triggered()), this , SLOT(exportPosesG2O()));
	_ui->actionG2o_g2o->setVisible(Optimizer::isAvailable(Optimizer::kTypeG2O));
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
	connect(_ui->actionExport_2D_scans_ply_pcd, SIGNAL(triggered()), this, SLOT(exportScans()));
	connect(_ui->actionExport_2D_Grid_map_bmp_png, SIGNAL(triggered()), this, SLOT(exportGridMap()));
	connect(_ui->actionExport_images_RGB_jpg_Depth_png, SIGNAL(triggered()), this , SLOT(exportImages()));
	connect(_ui->actionExport_cameras_in_Bundle_format_out, SIGNAL(triggered()), SLOT(exportBundlerFormat()));
	connect(_ui->actionView_scans, SIGNAL(triggered()), this, SLOT(viewScans()));
	connect(_ui->actionExport_octomap, SIGNAL(triggered()), this, SLOT(exportOctomap()));
	connect(_ui->actionView_high_res_point_cloud, SIGNAL(triggered()), this, SLOT(viewClouds()));
	connect(_ui->actionReset_Odometry, SIGNAL(triggered()), this, SLOT(resetOdometry()));
	connect(_ui->actionTrigger_a_new_map, SIGNAL(triggered()), this, SLOT(triggerNewMap()));
	connect(_ui->actionData_recorder, SIGNAL(triggered()), this, SLOT(dataRecorder()));
	connect(_ui->actionPost_processing, SIGNAL(triggered()), this, SLOT(postProcessing()));

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
	connect(_ui->actionOpenNI2_sense, SIGNAL(triggered()), this, SLOT(selectOpenni2()));
	connect(_ui->actionFreenect2, SIGNAL(triggered()), this, SLOT(selectFreenect2()));
	connect(_ui->actionStereoDC1394, SIGNAL(triggered()), this, SLOT(selectStereoDC1394()));
	connect(_ui->actionStereoFlyCapture2, SIGNAL(triggered()), this, SLOT(selectStereoFlyCapture2()));
	connect(_ui->actionStereoZed, SIGNAL(triggered()), this, SLOT(selectStereoZed()));
	connect(_ui->actionStereoUsb, SIGNAL(triggered()), this, SLOT(selectStereoUsb()));
	_ui->actionFreenect->setEnabled(CameraFreenect::available());
	_ui->actionOpenNI_CV->setEnabled(CameraOpenNICV::available());
	_ui->actionOpenNI_CV_ASUS->setEnabled(CameraOpenNICV::available());
	_ui->actionOpenNI2->setEnabled(CameraOpenNI2::available());
	_ui->actionOpenNI2_kinect->setEnabled(CameraOpenNI2::available());
	_ui->actionOpenNI2_sense->setEnabled(CameraOpenNI2::available());
	_ui->actionFreenect2->setEnabled(CameraFreenect2::available());
	_ui->actionStereoDC1394->setEnabled(CameraStereoDC1394::available());
	_ui->actionStereoFlyCapture2->setEnabled(CameraStereoFlyCapture2::available());
	_ui->actionStereoZed->setEnabled(CameraStereoZed::available());
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
	connect(_exportScansDialog, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_postProcessingDialog, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
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

	_ui->statsToolBox->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_ui->graphicsView_graphView->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_cloudViewer->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_cloudViewer->setBackfaceCulling(true, false);
	_preferencesDialog->loadWidgetState(_cloudViewer);

	//dialog states
	_preferencesDialog->loadWidgetState(_exportCloudsDialog);
	_preferencesDialog->loadWidgetState(_exportScansDialog);
	_preferencesDialog->loadWidgetState(_postProcessingDialog);

	if(_ui->statsToolBox->findChildren<StatItem*>().size() == 0)
	{
		const std::map<std::string, float> & statistics = Statistics::defaultData();
		for(std::map<std::string, float>::const_iterator iter = statistics.begin(); iter != statistics.end(); ++iter)
		{
			_ui->statsToolBox->updateStat(QString((*iter).first.c_str()).replace('_', ' '), 0, (*iter).second);
		}
	}
	// Specific MainWindow
	_ui->statsToolBox->updateStat("Planning/From/", 0.0f);
	_ui->statsToolBox->updateStat("Planning/Time/ms", 0.0f);
	_ui->statsToolBox->updateStat("Planning/Goal/", 0.0f);
	_ui->statsToolBox->updateStat("Planning/Poses/", 0.0f);
	_ui->statsToolBox->updateStat("Planning/Length/m", 0.0f);

	_ui->statsToolBox->updateStat("Camera/Time capturing/ms", 0.0f);
	_ui->statsToolBox->updateStat("Camera/Time decimation/ms", 0.0f);
	_ui->statsToolBox->updateStat("Camera/Time disparity/ms", 0.0f);
	_ui->statsToolBox->updateStat("Camera/Time mirroring/ms", 0.0f);
	_ui->statsToolBox->updateStat("Camera/Time scan from depth/ms", 0.0f);

	_ui->statsToolBox->updateStat("Odometry/ID/", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Features/", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Matches/", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Inliers/", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/ICPInliersRatio/", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/StdDev/", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Variance/", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/TimeEstimation/ms", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/TimeFiltering/ms", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/LocalMapSize/", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/LocalScanMapSize/", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Interval/ms", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Speed/kph", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Distance/m", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Tx/m", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Ty/m", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Tz/m", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Troll/deg", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Tpitch/deg", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Tyaw/deg", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Px/m", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Py/m", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Pz/m", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Proll/deg", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Ppitch/deg", 0.0f);
	_ui->statsToolBox->updateStat("Odometry/Pyaw/deg", 0.0f);

	_ui->statsToolBox->updateStat("GUI/Refresh odom/ms", 0.0f);
	_ui->statsToolBox->updateStat("GUI/RGB-D cloud/ms", 0.0f);
	_ui->statsToolBox->updateStat("GUI/RGB-D closure_view/ms", 0.0f);
	_ui->statsToolBox->updateStat("GUI/Refresh stats/ms", 0.0f);
	_ui->statsToolBox->updateStat("GUI/Cache Data Size/MB", 0.0f);
	_ui->statsToolBox->updateStat("GUI/Cache Clouds Size/MB", 0.0f);

	this->loadFigures();
	connect(_ui->statsToolBox, SIGNAL(figuresSetupChanged()), this, SLOT(configGUIModified()));

	// update loop closure viewer parameters
	_loopClosureViewer->setDecimation(_preferencesDialog->getCloudDecimation(0));
	_loopClosureViewer->setMaxDepth(_preferencesDialog->getCloudMaxDepth(0));

	splash.close();

	this->setFocus();

	UDEBUG("");
}

MainWindow::~MainWindow()
{
	UDEBUG("");
	this->stopDetection();
	delete _ui;
	delete _elapsedTime;
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

		if(_camera)
		{
			UERROR("Camera must be already deleted here!");
			delete _camera;
			_camera = 0;
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

void MainWindow::handleEvent(UEvent* anEvent)
{
	if(anEvent->getClassName().compare("RtabmapEvent") == 0)
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
			emit statsReceived(stats);
		}
	}
	else if(anEvent->getClassName().compare("RtabmapEventInit") == 0)
	{
		RtabmapEventInit * rtabmapEventInit = (RtabmapEventInit*)anEvent;
		emit rtabmapEventInitReceived((int)rtabmapEventInit->getStatus(), rtabmapEventInit->getInfo().c_str());
	}
	else if(anEvent->getClassName().compare("RtabmapEvent3DMap") == 0)
	{
		RtabmapEvent3DMap * rtabmapEvent3DMap = (RtabmapEvent3DMap*)anEvent;
		emit rtabmapEvent3DMapReceived(*rtabmapEvent3DMap);
	}
	else if(anEvent->getClassName().compare("RtabmapGlobalPathEvent") == 0)
	{
		RtabmapGlobalPathEvent * rtabmapGlobalPathEvent = (RtabmapGlobalPathEvent*)anEvent;
		emit rtabmapGlobalPathEventReceived(*rtabmapGlobalPathEvent);
	}
	else if(anEvent->getClassName().compare("RtabmapLabelErrorEvent") == 0)
	{
		RtabmapLabelErrorEvent * rtabmapLabelErrorEvent = (RtabmapLabelErrorEvent*)anEvent;
		emit rtabmapLabelErrorReceived(rtabmapLabelErrorEvent->id(), QString(rtabmapLabelErrorEvent->label().c_str()));
	}
	else if(anEvent->getClassName().compare("RtabmapGoalStatusEvent") == 0)
	{
		emit rtabmapGoalStatusEventReceived(anEvent->getCode());
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
			emit noMoreImagesReceived();
		}
		else
		{
			emit cameraInfoReceived(cameraEvent->info());
			if (_odomThread == 0 && _camera->camera()->odomProvided() && _preferencesDialog->isRGBDMode())
			{
				if (!_processingOdometry && !_processingStatistics)
				{
					_processingOdometry = true; // if we receive too many odometry events!
					OdometryEvent tmp(cameraEvent->data(), cameraEvent->info().odomPose, cameraEvent->info().odomCovariance);
					emit odometryReceived(tmp, false);
				}
				else
				{
					// we receive too many odometry events! just send without data
					SensorData data(cv::Mat(), cameraEvent->data().id(), cameraEvent->data().stamp());
					data.setCameraModels(cameraEvent->data().cameraModels());
					data.setStereoCameraModel(cameraEvent->data().stereoCameraModel());
					data.setGroundTruth(cameraEvent->data().groundTruth());
					OdometryEvent tmp(data, cameraEvent->info().odomPose, cameraEvent->info().odomCovariance);
					emit odometryReceived(tmp, true);
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
			emit odometryReceived(*odomEvent, false);
		}
		else
		{
			// we receive too many odometry events! just send without data
			SensorData data(cv::Mat(), odomEvent->data().id(), odomEvent->data().stamp());
			data.setCameraModels(odomEvent->data().cameraModels());
			data.setStereoCameraModel(odomEvent->data().stereoCameraModel());
			data.setGroundTruth(odomEvent->data().groundTruth());
			OdometryEvent tmp(data, odomEvent->pose(), odomEvent->covariance(), odomEvent->info().copyWithoutData());
			emit odometryReceived(tmp, true);
		}
	}
	else if(anEvent->getClassName().compare("ULogEvent") == 0)
	{
		ULogEvent * logEvent = (ULogEvent*)anEvent;
		if(logEvent->getCode() >= _preferencesDialog->getGeneralLoggerPauseLevel())
		{
			QMetaObject::invokeMethod(_ui->dockWidget_console, "show");
			// The timer prevents multiple calls to pauseDetection() before the state can be changed
			if(_state != kPaused && _state != kMonitoringPaused && _logEventTime->elapsed() > 1000)
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
}

void MainWindow::processCameraInfo(const rtabmap::CameraInfo & info)
{
	if(_firstStamp == 0.0)
	{
		_firstStamp = info.stamp;
	}

	_ui->statsToolBox->updateStat("Camera/Time capturing/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeCapture*1000.0f);
	_ui->statsToolBox->updateStat("Camera/Time decimation/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeImageDecimation*1000.0f);
	_ui->statsToolBox->updateStat("Camera/Time disparity/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeDisparity*1000.0f);
	_ui->statsToolBox->updateStat("Camera/Time mirroring/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeMirroring*1000.0f);
	_ui->statsToolBox->updateStat("Camera/Time scan_from_depth/ms", _preferencesDialog->isTimeUsedInFigures()?info.stamp-_firstStamp:(float)info.id, info.timeScanFromDepth*1000.0f);
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
	else if(odom.info().inliers>0 &&
			_preferencesDialog->getOdomQualityWarnThr() &&
			odom.info().inliers < _preferencesDialog->getOdomQualityWarnThr())
	{
		UDEBUG("odom warn, quality(inliers)=%d thr=%d", odom.info().inliers, _preferencesDialog->getOdomQualityWarnThr());
		lostStateChanged = _cloudViewer->getBackgroundColor() == Qt::darkRed;
		_cloudViewer->setBackgroundColor(Qt::darkYellow);
		_ui->imageView_odometry->setBackgroundColor(Qt::darkYellow);
	}
	else
	{
		UDEBUG("odom ok");
		lostStateChanged = _cloudViewer->getBackgroundColor() == Qt::darkRed;
		_cloudViewer->setBackgroundColor(_cloudViewer->getDefaultBackgroundColor());
		_ui->imageView_odometry->setBackgroundColor(Qt::black);
	}

	if(!pose.isNull() && (_ui->dockWidget_cloudViewer->isVisible() || _ui->graphicsView_graphView->isVisible()))
	{
		_lastOdomPose = pose;
		_odometryReceived = true;
	}

	if(_ui->dockWidget_cloudViewer->isVisible())
	{
		bool cloudUpdated = false;
		bool scanUpdated = false;
		bool featuresUpdated = false;
		if(!pose.isNull())
		{
			// 3d cloud
			if(odom.data().depthOrRightRaw().cols == odom.data().imageRaw().cols &&
			   odom.data().depthOrRightRaw().rows == odom.data().imageRaw().rows &&
			   !odom.data().depthOrRightRaw().empty() &&
			   (odom.data().cameraModels().size() || odom.data().stereoCameraModel().isValidForProjection()) &&
			   _preferencesDialog->isCloudsShown(1))
			{

				if(odom.data().imageRaw().cols % _preferencesDialog->getCloudDecimation(1) != 0 ||
				   odom.data().imageRaw().rows % _preferencesDialog->getCloudDecimation(1) != 0)
				{
					UERROR("Decimation (%d) is not modulo of the image resolution (%dx%d)! The cloud cannot be "
							"created. Go to Preferences->3D Rendering under \"Odom\" column to modify this parameter.",
							_preferencesDialog->getCloudDecimation(1),
							odom.data().imageRaw().cols,
							odom.data().imageRaw().rows);
				}
				else
				{

					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
					pcl::IndicesPtr indices(new std::vector<int>);
					cloud = util3d::cloudRGBFromSensorData(odom.data(),
							_preferencesDialog->getCloudDecimation(1),
							_preferencesDialog->getCloudMaxDepth(1),
							_preferencesDialog->getCloudMinDepth(1),
							indices.get(),
							_preferencesDialog->getAllParameters());
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
							if(odom.data().cameraModels().size() && !odom.data().cameraModels()[0].localTransform().isNull())
							{
								viewpoint[0] = odom.data().cameraModels()[0].localTransform().x();
								viewpoint[1] = odom.data().cameraModels()[0].localTransform().y();
								viewpoint[2] = odom.data().cameraModels()[0].localTransform().z();
							}
							else if(!odom.data().stereoCameraModel().localTransform().isNull())
							{
								viewpoint[0] = odom.data().stereoCameraModel().localTransform().x();
								viewpoint[1] = odom.data().stereoCameraModel().localTransform().y();
								viewpoint[2] = odom.data().stereoCameraModel().localTransform().z();
							}
							std::vector<pcl::Vertices> polygons = util3d::organizedFastMesh(
									output,
									_preferencesDialog->getCloudMeshingAngle(),
									_preferencesDialog->isCloudMeshingQuad(),
									_preferencesDialog->getCloudMeshingTriangleSize(),
									Eigen::Vector3f(pose.x(), pose.y(), pose.z()) + viewpoint);
							if(polygons.size())
							{
								if(!_cloudViewer->addCloudMesh("cloudOdom", output, polygons, _odometryCorrection))
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
						_cloudViewer->setCloudOpacity("cloudOdom", _preferencesDialog->getCloudOpacity(1));
						_cloudViewer->setCloudPointSize("cloudOdom", _preferencesDialog->getCloudPointSize(1));

						cloudUpdated = true;
					}
				}
			}

			if(_preferencesDialog->isScansShown(1))
			{
				// scan local map
				if(!odom.info().localScanMap.empty())
				{
					pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
					cloud = util3d::laserScanToPointCloudNormal(odom.info().localScanMap);
					if(!_cloudViewer->addCloud("scanMapOdom", cloud, _odometryCorrection, Qt::blue))
					{
						UERROR("Adding scanMapOdom to viewer failed!");
					}
					else
					{
						_cloudViewer->setCloudVisibility("scanMapOdom", true);
						_cloudViewer->setCloudOpacity("scanMapOdom", _preferencesDialog->getScanOpacity(1));
						_cloudViewer->setCloudPointSize("scanMapOdom", _preferencesDialog->getScanPointSize(1));
						scanUpdated = true;
					}
				}
				// scan cloud
				if(!odom.data().laserScanRaw().empty())
				{
					cv::Mat scan = odom.data().laserScanRaw();

					if(_preferencesDialog->getDownsamplingStepScan(1) > 0)
					{
						scan = util3d::downsample(scan, _preferencesDialog->getDownsamplingStepScan(1));
					}

					pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
					cloud = util3d::laserScanToPointCloudNormal(scan, pose);
					if(_preferencesDialog->getCloudVoxelSizeScan(1) > 0.0)
					{
						cloud = util3d::voxelize(cloud, _preferencesDialog->getCloudVoxelSizeScan(1));
					}

					if(!_cloudViewer->addCloud("scanOdom", cloud, _odometryCorrection, Qt::magenta))
					{
						UERROR("Adding scanOdom to viewer failed!");
					}
					else
					{
						_cloudViewer->setCloudVisibility("scanOdom", true);
						_cloudViewer->setCloudOpacity("scanOdom", _preferencesDialog->getScanOpacity(1));
						_cloudViewer->setCloudPointSize("scanOdom", _preferencesDialog->getScanPointSize(1));
						scanUpdated = true;
					}
				}
			}

			// 3d features
			if(_preferencesDialog->isFeaturesShown(1))
			{
				if(!odom.info().localMap.empty())
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					cloud->resize(odom.info().localMap.size());
					int i=0;
					for(std::map<int, cv::Point3f>::const_iterator iter=odom.info().localMap.begin(); iter!=odom.info().localMap.end(); ++iter)
					{
						(*cloud)[i].x = iter->second.x;
						(*cloud)[i].y = iter->second.y;
						(*cloud)[i].z = iter->second.z;

						// green = inlier, yellow = outliers
						bool inlier = odom.info().words.find(iter->first) != odom.info().words.end();
						(*cloud)[i].r = inlier?0:255;
						(*cloud)[i].g = 255;
						(*cloud)[i++].b = 0;
					}

					_cloudViewer->addCloud("featuresOdom", cloud, _odometryCorrection);
					_cloudViewer->setCloudVisibility("featuresOdom", true);
					_cloudViewer->setCloudPointSize("featuresOdom", _preferencesDialog->getFeaturesPointSize(1));

					featuresUpdated = true;
				}
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
		}
	}

	if(!odom.pose().isNull())
	{
		// update camera position
		_cloudViewer->updateCameraTargetPosition(_odometryCorrection*odom.pose());

		if(odom.data().cameraModels().size() && !odom.data().cameraModels()[0].localTransform().isNull())
		{
			_cloudViewer->updateCameraFrustums(_odometryCorrection*odom.pose(), odom.data().cameraModels());
		}
		else if(!odom.data().stereoCameraModel().localTransform().isNull())
		{
			_cloudViewer->updateCameraFrustum(_odometryCorrection*odom.pose(), odom.data().stereoCameraModel());
		}

	}
	_cloudViewer->update();

	if(_ui->graphicsView_graphView->isVisible())
	{
		if(!pose.isNull() && !odom.pose().isNull())
		{
			_ui->graphicsView_graphView->updateReferentialPosition(_odometryCorrection*odom.pose());
			_ui->graphicsView_graphView->update();
		}
	}

	if(_ui->dockWidget_odometry->isVisible() &&
	   !odom.data().imageRaw().empty())
	{
		if(_ui->imageView_odometry->isFeaturesShown())
		{
			if(odom.info().type == 0)
			{
				_ui->imageView_odometry->setFeatures(
						odom.info().words,
						odom.data().depthRaw(),
						Qt::yellow);
			}
			else if(odom.info().type == 1)
			{
				std::vector<cv::KeyPoint> kpts;
				cv::KeyPoint::convert(odom.info().refCorners, kpts);
				_ui->imageView_odometry->setFeatures(
						kpts,
						odom.data().depthRaw(),
						Qt::red);
			}
		}

		//detect if it is OdometryMono intitialization
		bool monoInitialization = false;
		if(_preferencesDialog->getOdomStrategy() == 2 && odom.info().type == 1)
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
			_ui->imageView_odometry->setImageDepth(uCvMat2QImage(odom.data().imageRaw()));
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

			_ui->imageView_odometry->setImage(uCvMat2QImage(odom.data().imageRaw()));
			if(_ui->imageView_odometry->isImageDepthShown())
			{
				_ui->imageView_odometry->setImageDepth(uCvMat2QImage(odom.data().depthOrRightRaw()));
			}

			if(odom.info().type == 0)
			{
				if(_ui->imageView_odometry->isFeaturesShown())
				{
					for(unsigned int i=0; i<odom.info().wordMatches.size(); ++i)
					{
						_ui->imageView_odometry->setFeatureColor(odom.info().wordMatches[i], Qt::red); // outliers
					}
					for(unsigned int i=0; i<odom.info().wordInliers.size(); ++i)
					{
						_ui->imageView_odometry->setFeatureColor(odom.info().wordInliers[i], Qt::green); // inliers
					}
				}
			}
			if(odom.info().type == 1 && odom.info().refCorners.size())
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
									odom.info().refCorners[i].x,
									odom.info().refCorners[i].y,
									odom.info().newCorners[i].x,
									odom.info().newCorners[i].y,
									inliers.find(i) != inliers.end()?Qt::blue:Qt::yellow);
						}
					}
				}
			}
		}
		if(!odom.data().imageRaw().empty())
		{
			_ui->imageView_odometry->setSceneRect(QRectF(0,0,(float)odom.data().imageRaw().cols, (float)odom.data().imageRaw().rows));
		}

		_ui->imageView_odometry->update();
	}

	if(_ui->actionAuto_screen_capture->isChecked() && _autoScreenCaptureOdomSync)
	{
		this->captureScreen(_autoScreenCaptureRAM);
	}

	//Process info
	_ui->statsToolBox->updateStat("Odometry/Inliers/", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), (float)odom.info().inliers);
	_ui->statsToolBox->updateStat("Odometry/ICPInliersRatio/", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), (float)odom.info().icpInliersRatio);
	_ui->statsToolBox->updateStat("Odometry/Matches/", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), (float)odom.info().matches);
	_ui->statsToolBox->updateStat("Odometry/StdDev/", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), sqrt((float)odom.info().variance));
	_ui->statsToolBox->updateStat("Odometry/Variance/", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), (float)odom.info().variance);
	_ui->statsToolBox->updateStat("Odometry/TimeEstimation/ms", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), (float)odom.info().timeEstimation*1000.0f);
	_ui->statsToolBox->updateStat("Odometry/TimeFiltering/ms", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), (float)odom.info().timeParticleFiltering*1000.0f);
	_ui->statsToolBox->updateStat("Odometry/Features/", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), (float)odom.info().features);
	_ui->statsToolBox->updateStat("Odometry/LocalMapSize/", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), (float)odom.info().localMapSize);
	_ui->statsToolBox->updateStat("Odometry/LocalScanMapSize/", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), (float)odom.info().localScanMapSize);
	_ui->statsToolBox->updateStat("Odometry/ID/", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), (float)odom.data().id());

	float x=0.0f,y,z, roll,pitch,yaw;
	if(!odom.info().transform.isNull())
	{
		odom.info().transform.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		_ui->statsToolBox->updateStat("Odometry/Tx/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), x);
		_ui->statsToolBox->updateStat("Odometry/Ty/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), y);
		_ui->statsToolBox->updateStat("Odometry/Tz/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), z);
		_ui->statsToolBox->updateStat("Odometry/Troll/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), roll*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/Tpitch/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), pitch*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/Tyaw/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), yaw*180.0/CV_PI);
	}

	if(!odom.info().transformFiltered.isNull())
	{
		odom.info().transformFiltered.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		_ui->statsToolBox->updateStat("Odometry/TFx/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), x);
		_ui->statsToolBox->updateStat("Odometry/TFy/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), y);
		_ui->statsToolBox->updateStat("Odometry/TFz/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), z);
		_ui->statsToolBox->updateStat("Odometry/TFroll/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), roll*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/TFpitch/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), pitch*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/TFyaw/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), yaw*180.0/CV_PI);
	}
	if(odom.info().interval > 0)
	{
		_ui->statsToolBox->updateStat("Odometry/Interval/ms", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), odom.info().interval*1000.f);
		_ui->statsToolBox->updateStat("Odometry/Speed/kph", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), x/odom.info().interval*3.6f);
	}

	if(!odom.info().transformGroundTruth.isNull())
	{
		odom.info().transformGroundTruth.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		_ui->statsToolBox->updateStat("Odometry/TGx/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), x);
		_ui->statsToolBox->updateStat("Odometry/TGy/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), y);
		_ui->statsToolBox->updateStat("Odometry/TGz/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), z);
		_ui->statsToolBox->updateStat("Odometry/TGroll/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), roll*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/TGpitch/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), pitch*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/TGyaw/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), yaw*180.0/CV_PI);
	}

	//cumulative pose
	if(!odom.pose().isNull())
	{
		odom.pose().getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		_ui->statsToolBox->updateStat("Odometry/Px/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), x);
		_ui->statsToolBox->updateStat("Odometry/Py/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), y);
		_ui->statsToolBox->updateStat("Odometry/Pz/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), z);
		_ui->statsToolBox->updateStat("Odometry/Proll/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), roll*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/Ppitch/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), pitch*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/Pyaw/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), yaw*180.0/CV_PI);
	}
	if(!odom.data().groundTruth().isNull())
	{
		odom.data().groundTruth().getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		_ui->statsToolBox->updateStat("Odometry/PGx/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), x);
		_ui->statsToolBox->updateStat("Odometry/PGy/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), y);
		_ui->statsToolBox->updateStat("Odometry/PGz/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), z);
		_ui->statsToolBox->updateStat("Odometry/PGroll/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), roll*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/PGpitch/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), pitch*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/PGyaw/deg", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), yaw*180.0/CV_PI);
	}

	if(odom.info().distanceTravelled > 0)
	{
		_ui->statsToolBox->updateStat("Odometry/Distance/m", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), odom.info().distanceTravelled);
	}

	_ui->statsToolBox->updateStat("GUI/Refresh odom/ms", _preferencesDialog->isTimeUsedInFigures()?odom.data().stamp()-_firstStamp:(float)odom.data().id(), time.elapsed()*1000.0);
	_processingOdometry = false;
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
	if(uContains(stat.getSignatures(), stat.refImageId()))
	{
		refMapId = stat.getSignatures().at(stat.refImageId()).mapId();
	}
	int highestHypothesisId = static_cast<float>(uValue(stat.data(), Statistics::kLoopHighest_hypothesis_id(), 0.0f));
	int loopId = stat.loopClosureId()>0?stat.loopClosureId():stat.proximityDetectionId()>0?stat.proximityDetectionId():highestHypothesisId;
	if(_cachedSignatures.contains(loopId))
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

		// update cache
		Signature signature;
		if(uContains(stat.getSignatures(), stat.refImageId()))
		{
			signature = stat.getSignatures().at(stat.refImageId());
			signature.sensorData().uncompressData(); // make sure data are uncompressed

			if(!smallMovement)
			{
				// keep in cache only compressed data
				Signature signatureWithoutRawData = signature;
				signatureWithoutRawData.sensorData().setImageRaw(cv::Mat());
				signatureWithoutRawData.sensorData().setDepthOrRightRaw(cv::Mat());
				signatureWithoutRawData.sensorData().setUserDataRaw(cv::Mat());
				signatureWithoutRawData.sensorData().setLaserScanRaw(cv::Mat(), 0, 0);
				_cachedSignatures.insert(signature.id(), signatureWithoutRawData);
				_cachedMemoryUsage += signatureWithoutRawData.sensorData().getMemoryUsed();
			}
		}

		// For intermediate empty nodes, keep latest image shown
		if(!signature.sensorData().imageRaw().empty() || signature.getWords().size())
		{
			_ui->imageView_source->clear();
			_ui->imageView_loopClosure->clear();

			_ui->imageView_source->setBackgroundColor(Qt::black);
			_ui->imageView_loopClosure->setBackgroundColor(Qt::black);

			_ui->label_matchId->clear();
		}

		int rehearsalMerged = (int)uValue(stat.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
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
				"Feature Color code:\n"
				"  Green = New\n"
				"  Yellow = New but Not Unique\n"
				"  Red = In Vocabulary\n"
		        "  Blue = In Vocabulary and in Previous Signature\n"
				"  Pink = In Vocabulary and in Loop Closure Signature");
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
				"  Pink = In Vocabulary and in Loop Closure Signature");
		}

		UDEBUG("time= %d ms", time.restart());

		int rejectedHyp = bool(uValue(stat.data(), Statistics::kLoopRejectedHypothesis(), 0.0f));
		float highestHypothesisValue = uValue(stat.data(), Statistics::kLoopHighest_hypothesis_value(), 0.0f);
		int matchId = 0;
		Signature loopSignature;
		int shownLoopId = 0;
		if(highestHypothesisId > 0 || stat.proximityDetectionId()>0)
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
				shownLoopId = stat.loopClosureId()>0?stat.loopClosureId():stat.proximityDetectionId()>0?stat.proximityDetectionId():highestHypothesisId;
				QMap<int, Signature>::iterator iter = _cachedSignatures.find(shownLoopId);
				if(iter != _cachedSignatures.end())
				{
					// uncompress after copy to avoid keeping uncompressed data in memory
					loopSignature = iter.value();
					loopSignature.sensorData().uncompressData();
				}
			}
		}
		_refIds.push_back(stat.refImageId());
		_loopClosureIds.push_back(matchId);

		//update image views
		{
			UCvMat2QImageThread qimageThread(signature.sensorData().imageRaw());
			UCvMat2QImageThread qimageLoopThread(loopSignature.sensorData().imageRaw());
			UCvMat2QImageThread qdepthThread(signature.sensorData().depthOrRightRaw());
			UCvMat2QImageThread qdepthLoopThread(loopSignature.sensorData().depthOrRightRaw());
			qimageThread.start();
			qdepthThread.start();
			qimageLoopThread.start();
			qdepthLoopThread.start();
			qimageThread.join();
			qdepthThread.join();
			qimageLoopThread.join();
			qdepthLoopThread.join();
			QImage img = qimageThread.getQImage();
			QImage lcImg = qimageLoopThread.getQImage();
			QImage depth = qdepthThread.getQImage();
			QImage lcDepth = qdepthLoopThread.getQImage();
			UDEBUG("time= %d ms", time.restart());

			if(!img.isNull())
			{
				_ui->imageView_source->setImage(img);
			}
			if(!depth.isNull())
			{
				_ui->imageView_source->setImageDepth(depth);
			}
			if(!lcImg.isNull())
			{
				_ui->imageView_loopClosure->setImage(lcImg);
			}
			if(!lcDepth.isNull())
			{
				_ui->imageView_loopClosure->setImageDepth(lcDepth);
			}
			if(_ui->imageView_loopClosure->sceneRect().isNull())
			{
				_ui->imageView_loopClosure->setSceneRect(_ui->imageView_source->sceneRect());
			}
		}

		UDEBUG("time= %d ms", time.restart());

		// do it after scaling
		this->drawKeypoints(signature.getWords(), loopSignature.getWords());

		UDEBUG("time= %d ms", time.restart());

		_ui->statsToolBox->updateStat("Keypoint/Keypoints count in the last signature/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), signature.getWords().size());
		_ui->statsToolBox->updateStat("Keypoint/Keypoints count in the loop signature/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), loopSignature.getWords().size());

		// PDF AND LIKELIHOOD
		if(!stat.posterior().empty() && _ui->dockWidget_posterior->isVisible())
		{
			UDEBUG("");
			_posteriorCurve->setData(QMap<int, float>(stat.posterior()), QMap<int, int>(stat.weights()));

			ULOGGER_DEBUG("");
			//Adjust thresholds
			float value;
			value = float(_preferencesDialog->getLoopThr());
			emit(loopClosureThrChanged(value));
		}
		if(!stat.likelihood().empty() && _ui->dockWidget_likelihood->isVisible())
		{
			_likelihoodCurve->setData(QMap<int, float>(stat.likelihood()), QMap<int, int>(stat.weights()));
		}
		if(!stat.rawLikelihood().empty() && _ui->dockWidget_rawlikelihood->isVisible())
		{
			_rawLikelihoodCurve->setData(QMap<int, float>(stat.rawLikelihood()), QMap<int, int>(stat.weights()));
		}

		// Update statistics tool box
		const std::map<std::string, float> & statistics = stat.data();
		for(std::map<std::string, float>::const_iterator iter = statistics.begin(); iter != statistics.end(); ++iter)
		{
			//ULOGGER_DEBUG("Updating stat \"%s\"", (*iter).first.c_str());
			_ui->statsToolBox->updateStat(QString((*iter).first.c_str()).replace('_', ' '), _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), (*iter).second);
		}

		UDEBUG("time= %d ms", time.restart());

		//======================
		// RGB-D Mapping stuff
		//======================
		UTimer timerVis;

		// update clouds
		if(stat.poses().size())
		{
			// update pose only if odometry is not received
			std::map<int, int> mapIds;
			std::map<int, Transform> groundTruth;
			std::map<int, std::string> labels;
			for(std::map<int, Signature>::const_iterator iter=stat.getSignatures().begin(); iter!=stat.getSignatures().end();++iter)
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
			}

			std::map<int, Transform> poses = stat.poses();
			Transform groundTruthOffset = alignPosesToGroundTruth(poses, groundTruth);
			UDEBUG("time= %d ms", time.restart());

			if(!_odometryReceived && poses.size())
			{
				_cloudViewer->updateCameraTargetPosition(poses.rbegin()->second);

				Transform localTransform = Transform::getIdentity();
				std::map<int, Signature>::const_iterator iter = stat.getSignatures().find(poses.rbegin()->first);
				if(iter != stat.getSignatures().end())
				{
					if(iter->second.sensorData().cameraModels().size() && !iter->second.sensorData().cameraModels()[0].localTransform().isNull())
					{
						_cloudViewer->updateCameraFrustums(poses.rbegin()->second, iter->second.sensorData().cameraModels());
					}
					else if(!iter->second.sensorData().stereoCameraModel().localTransform().isNull())
					{
						_cloudViewer->updateCameraFrustum(poses.rbegin()->second, iter->second.sensorData().stereoCameraModel());
					}
				}

				if(_ui->graphicsView_graphView->isVisible())
				{
					_ui->graphicsView_graphView->updateReferentialPosition(poses.rbegin()->second);
				}
			}

			updateMapCloud(
					poses,
					stat.constraints(),
					mapIds,
					labels,
					groundTruth);

			_odometryReceived = false;

			_odometryCorrection = groundTruthOffset * stat.mapCorrection();

			UDEBUG("time= %d ms", time.restart());
			_ui->statsToolBox->updateStat("GUI/RGB-D cloud/ms", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), int(timerVis.elapsed()*1000.0f));

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
					_ui->statsToolBox->updateStat("GUI/RGB-D closure view/ms", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), int(loopTimer.elapsed()*1000.0f));
				}

				UDEBUG("time= %d ms", time.restart());
			}

			// ground truth live statistics
			if(poses.size() && groundTruth.size())
			{
				std::vector<float> translationalErrors(poses.size());
				std::vector<float> rotationalErrors(poses.size());
				int oi=0;
				float sumTranslationalErrors = 0.0f;
				float sumRotationalErrors = 0.0f;
				float sumSqrdTranslationalErrors = 0.0f;
				float sumSqrdRotationalErrors = 0.0f;
				float radToDegree = 180.0f / M_PI;
				float translational_min = 0.0f;
				float translational_max = 0.0f;
				float rotational_min = 0.0f;
				float rotational_max = 0.0f;
				for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					std::map<int, Transform>::iterator jter = groundTruth.find(iter->first);
					if(jter!=groundTruth.end())
					{
						Eigen::Vector3f vA = iter->second.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
						Eigen::Vector3f vB = jter->second.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
						double a = pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
						rotationalErrors[oi] = a*radToDegree;
						translationalErrors[oi] = iter->second.getDistance(jter->second);

						sumTranslationalErrors+=translationalErrors[oi];
						sumSqrdTranslationalErrors+=translationalErrors[oi]*translationalErrors[oi];
						sumRotationalErrors+=rotationalErrors[oi];
						sumSqrdRotationalErrors+=rotationalErrors[oi]*rotationalErrors[oi];

						if(oi == 0)
						{
							translational_min = translational_max = translationalErrors[oi];
							rotational_min = rotational_max = rotationalErrors[oi];
						}
						else
						{
							if(translationalErrors[oi] < translational_min)
							{
								translational_min = translationalErrors[oi];
							}
							else if(translationalErrors[oi] > translational_max)
							{
								translational_max = translationalErrors[oi];
							}

							if(rotationalErrors[oi] < rotational_min)
							{
								rotational_min = rotationalErrors[oi];
							}
							else if(rotationalErrors[oi] > rotational_max)
							{
								rotational_max = rotationalErrors[oi];
							}
						}

						++oi;
					}
				}
				translationalErrors.resize(oi);
				rotationalErrors.resize(oi);
				if(oi)
				{
					float total = float(oi);
					float translational_rmse = std::sqrt(sumSqrdTranslationalErrors/total);
					float translational_mean = sumTranslationalErrors/total;
					float translational_median = translationalErrors[oi/2];
					float translational_std = std::sqrt(uVariance(translationalErrors, translational_mean));

					float rotational_rmse = std::sqrt(sumSqrdRotationalErrors/total);
					float rotational_mean = sumRotationalErrors/total;
					float rotational_median = rotationalErrors[oi/2];
					float rotational_std = std::sqrt(uVariance(rotationalErrors, rotational_mean));

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

					_ui->statsToolBox->updateStat("GT/translational_rmse/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), translational_rmse);
					_ui->statsToolBox->updateStat("GT/translational_mean/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), translational_mean);
					_ui->statsToolBox->updateStat("GT/translational_median/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), translational_median);
					_ui->statsToolBox->updateStat("GT/translational_std/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), translational_std);
					_ui->statsToolBox->updateStat("GT/translational_min/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), translational_min);
					_ui->statsToolBox->updateStat("GT/translational_max/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), translational_max);

					_ui->statsToolBox->updateStat("GT/rotational_rmse/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), rotational_rmse);
					_ui->statsToolBox->updateStat("GT/rotational_mean/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), rotational_mean);
					_ui->statsToolBox->updateStat("GT/rotational_median/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), rotational_median);
					_ui->statsToolBox->updateStat("GT/rotational_std/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), rotational_std);
					_ui->statsToolBox->updateStat("GT/rotational_min/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), rotational_min);
					_ui->statsToolBox->updateStat("GT/rotational_max/", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), rotational_max);
				}
			}
			UDEBUG("time= %d ms", time.restart());
		}

		if( _ui->graphicsView_graphView->isVisible())
		{
			// update posterior on the graph view
			if(_preferencesDialog->isPosteriorGraphView() &&
			   stat.posterior().size())
			{
				_ui->graphicsView_graphView->updatePosterior(stat.posterior());
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
		}

		UDEBUG("");
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
	_ui->statsToolBox->updateStat("GUI/Refresh stats/ms", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), elapsedTime);
	if(_ui->actionAuto_screen_capture->isChecked() && !_autoScreenCaptureOdomSync)
	{
		this->captureScreen(_autoScreenCaptureRAM);
	}

	if(!_preferencesDialog->isImagesKept())
	{
		_cachedSignatures.clear();
		_cachedMemoryUsage = 0;
	}
	_ui->statsToolBox->updateStat("GUI/Cache Data Size/MB", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), _cachedMemoryUsage/(1024*1024));
	_ui->statsToolBox->updateStat("GUI/Cache Clouds Size/MB", _preferencesDialog->isTimeUsedInFigures()?stat.stamp()-_firstStamp:stat.refImageId(), _createdCloudsMemoryUsage/(1024*1024));

	if(_state != kMonitoring && _state != kDetecting)
	{
		_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
	}

	_processingStatistics = false;
}

void MainWindow::updateMapCloud(
		const std::map<int, Transform> & posesIn,
		const std::multimap<int, Link> & constraints,
		const std::map<int, int> & mapIdsIn,
		const std::map<int, std::string> & labels,
		const std::map<int, Transform> & groundTruths, // ground truth should contain only valid transforms
		bool verboseProgress)
{
	UDEBUG("posesIn=%d constraints=%d mapIdsIn=%d labelsIn=%d",
			(int)posesIn.size(), (int)constraints.size(), (int)mapIdsIn.size(), (int)labels.size());
	if(posesIn.size())
	{
		_currentPosesMap = posesIn;
		_currentLinksMap = constraints;
		_currentMapIds = mapIdsIn;
		_currentLabels = labels;
		_currentGTPosesMap = groundTruths;
		if(_state != kMonitoring && _state != kDetecting)
		{
			_ui->actionPost_processing->setEnabled(_cachedSignatures.size() >= 2 && _currentPosesMap.size() >= 2 && _currentLinksMap.size() >= 1);
			_ui->menuExport_poses->setEnabled(!_currentPosesMap.empty());
		}
		_ui->actionAnchor_clouds_to_ground_truth->setEnabled(!_currentGTPosesMap.empty());
	}

	// filter duplicated poses
	std::map<int, Transform> poses;
	std::map<int, int> mapIds;
	if(_preferencesDialog->isCloudFiltering() && posesIn.size())
	{
		float radius = _preferencesDialog->getCloudFilteringRadius();
		float angle = _preferencesDialog->getCloudFilteringAngle()*CV_PI/180.0; // convert to rad
		poses = rtabmap::graph::radiusPosesFiltering(posesIn, radius, angle);
		for(std::map<int, Transform>::iterator iter= poses.begin(); iter!=poses.end(); ++iter)
		{
			std::map<int, int>::const_iterator jter = mapIdsIn.find(iter->first);
			if(jter!=mapIdsIn.end())
			{
				mapIds.insert(*jter);
			}
			else
			{
				UERROR("map id of node %d not found!", iter->first);
			}
		}

		if(verboseProgress)
		{
			_initProgressDialog->appendText(tr("Map update: %1 nodes shown of %2 (cloud filtering is on)").arg(poses.size()).arg(posesIn.size()));
			QApplication::processEvents();
		}
	}
	else
	{
		poses = posesIn;
		mapIds = mapIdsIn;
	}

	std::map<int, bool> posesMask;
	for(std::map<int, Transform>::const_iterator iter = posesIn.begin(); iter!=posesIn.end(); ++iter)
	{
		posesMask.insert(posesMask.end(), std::make_pair(iter->first, poses.find(iter->first) != poses.end()));
	}
	_ui->widget_mapVisibility->setMap(posesIn, posesMask);

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
	else if(_currentGTPosesMap.size() == 0)
	{
		_ui->actionAnchor_clouds_to_ground_truth->setChecked(false);
	}

	// Map updated! regenerate the assembled cloud, last pose is the new one
	UDEBUG("Update map with %d locations", poses.size());
	QMap<std::string, Transform> viewerClouds = _cloudViewer->getAddedClouds();
	int i=1;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(!iter->second.isNull())
		{
			std::string cloudName = uFormat("cloud%d", iter->first);

			// 3d point cloud
			bool update3dCloud = _cloudViewer->isVisible() && _preferencesDialog->isCloudsShown(0);
			bool updateProjMap =
					_ui->graphicsView_graphView->isVisible() &&
					_ui->graphicsView_graphView->isGridMapVisible() &&
					_preferencesDialog->isGridMapFrom3DCloud() &&
					_projectionLocalMaps.find(iter->first) == _projectionLocalMaps.end();
			bool updateOctomap = false;
#ifdef RTABMAP_OCTOMAP
			updateOctomap =
					_cloudViewer->isVisible() &&
					_preferencesDialog->isOctomapShown() &&
					_octomap->addedNodes().find(iter->first) == _octomap->addedNodes().end();
#endif
			if(update3dCloud ||	updateProjMap || updateOctomap)
			{
				// update cloud
				std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> createdCloud;
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
					_cloudViewer->setCloudOpacity(cloudName, _preferencesDialog->getCloudOpacity(0));
					_cloudViewer->setCloudPointSize(cloudName, _preferencesDialog->getCloudPointSize(0));
				}
				else if(_cachedClouds.find(iter->first) == _cachedClouds.end() && _cachedSignatures.contains(iter->first))
				{
					createdCloud = this->createAndAddCloudToMap(iter->first, iter->second, uValue(mapIds, iter->first, -1));
					if(_cloudViewer->getAddedClouds().contains(cloudName))
					{
						_cloudViewer->setCloudVisibility(cloudName.c_str(), _cloudViewer->isVisible() && _preferencesDialog->isCloudsShown(0));
					}
				}

				//Update projection map
				if(updateProjMap || updateOctomap)
				{
					std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> >::iterator cloudIter = _cachedClouds.find(iter->first);
					if(cloudIter != _cachedClouds.end())
					{
						createAndAddProjectionMap(cloudIter->second.first, cloudIter->second.second, iter->first, iter->second, updateOctomap);
					}
					else if(createdCloud.first.get() && createdCloud.first->size() && createdCloud.second->size())
					{
						createAndAddProjectionMap(createdCloud.first, createdCloud.second, iter->first, iter->second, updateOctomap);
					}
				}
			}
			else if(viewerClouds.contains(cloudName))
			{
				_cloudViewer->setCloudVisibility(cloudName.c_str(), false);
			}

			// 2d point cloud
			std::string scanName = uFormat("scan%d", iter->first);
			if((_cloudViewer->isVisible() && (_preferencesDialog->isScansShown(0) || _preferencesDialog->getGridMapShown())) ||
				(_ui->graphicsView_graphView->isVisible() && _ui->graphicsView_graphView->isGridMapVisible()))
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
					_cloudViewer->setCloudOpacity(scanName, _preferencesDialog->getScanOpacity(0));
					_cloudViewer->setCloudPointSize(scanName, _preferencesDialog->getScanPointSize(0));
				}
				else if(_cachedSignatures.contains(iter->first))
				{
					QMap<int, Signature>::iterator jter = _cachedSignatures.find(iter->first);
					if(!jter->sensorData().laserScanCompressed().empty() || !jter->sensorData().laserScanRaw().empty())
					{
						this->createAndAddScanToMap(iter->first, iter->second, uValue(mapIds, iter->first, -1));
					}
				}
			}
			else if(viewerClouds.contains(scanName))
			{
				_cloudViewer->setCloudVisibility(scanName.c_str(), false);
			}

			// 3d features
			std::string featuresName = uFormat("features%d", iter->first);
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

			if(verboseProgress)
			{
				_initProgressDialog->appendText(tr("Updated cloud %1 (%2/%3)").arg(iter->first).arg(i).arg(poses.size()));
				_initProgressDialog->incrementStep();
				if(poses.size() < 200 || i % 100 == 0)
				{
					QApplication::processEvents();
				}
			}
		}

		++i;
	}

	// activate actions
	if(_state != kMonitoring && _state != kDetecting)
	{
		_ui->actionSave_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionView_high_res_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionExport_2D_scans_ply_pcd->setEnabled(!_createdScans.empty());
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(!_gridLocalMaps.empty() || !_projectionLocalMaps.empty());
		_ui->actionView_scans->setEnabled(!_createdScans.empty());
#ifdef RTABMAP_OCTOMAP
		_ui->actionExport_octomap->setEnabled(_octomap->octree()->size());
#else
		_ui->actionExport_octomap->setEnabled(false);
#endif
	}

	//remove not used clouds
	for(QMap<std::string, Transform>::iterator iter = viewerClouds.begin(); iter!=viewerClouds.end(); ++iter)
	{
		std::list<std::string> splitted = uSplitNumChar(iter.key());
		if(splitted.size() == 2)
		{
			int id = std::atoi(splitted.back().c_str());
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

	UDEBUG("");

	// update 3D graphes (show all poses)
	_cloudViewer->removeAllGraphs();
	_cloudViewer->removeCloud("graph_nodes");
	if(_preferencesDialog->isGraphsShown() && _currentPosesMap.size())
	{
		// Find all graphs
		std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > graphs;
		for(std::map<int, Transform>::iterator iter=_currentPosesMap.begin(); iter!=_currentPosesMap.end(); ++iter)
		{
			int mapId = uValue(_currentMapIds, iter->first, -1);

			//edges
			std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator kter = graphs.find(mapId);
			if(kter == graphs.end())
			{
				kter = graphs.insert(std::make_pair(mapId, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>))).first;
			}
			pcl::PointXYZ pt(iter->second.x(), iter->second.y(), iter->second.z());
			kter->second->push_back(pt);
		}

		//Ground truth graph?
		for(std::map<int, Transform>::iterator iter=_currentGTPosesMap.begin(); iter!=_currentGTPosesMap.end(); ++iter)
		{
			int mapId = -100;
			//edges
			std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator kter = graphs.find(mapId);
			if(kter == graphs.end())
			{
				kter = graphs.insert(std::make_pair(mapId, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>))).first;
			}
			pcl::PointXYZ pt(iter->second.x(), iter->second.y(), iter->second.z());
			kter->second->push_back(pt);
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
	}

	UDEBUG("labels.size()=%d", (int)labels.size());

	// Update labels
	_cloudViewer->removeAllTexts();
	if(_preferencesDialog->isLabelsShown() && labels.size())
	{
		for(std::map<int, std::string>::const_iterator iter=labels.begin(); iter!=labels.end(); ++iter)
		{
			if(posesIn.find(iter->first)!=posesIn.end())
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

	// Update occupancy grid map in 3D map view and graph view
	if(_ui->graphicsView_graphView->isVisible())
	{
		_ui->graphicsView_graphView->updateGraph(posesIn, constraints, mapIdsIn);
		_ui->graphicsView_graphView->updateGTGraph(_currentGTPosesMap);
	}
	cv::Mat map8U;
	if((_ui->graphicsView_graphView->isVisible() || _preferencesDialog->getGridMapShown()) &&
			((_gridLocalMaps.size() && !_preferencesDialog->isGridMapFrom3DCloud()) ||
			 (_projectionLocalMaps.size() && _preferencesDialog->isGridMapFrom3DCloud())))
	{
		float xMin, yMin;
		float resolution = _preferencesDialog->getGridMapResolution();
		cv::Mat map8S = util3d::create2DMapFromOccupancyLocalMaps(
					poses,
					_preferencesDialog->isGridMapFrom3DCloud()?_projectionLocalMaps:_gridLocalMaps,
					resolution,
					xMin, yMin,
					0,
					_preferencesDialog->isGridMapEroded());
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

	if(!_preferencesDialog->getGridMapShown())
	{
		UDEBUG("");
		_cloudViewer->removeOccupancyGridMap();
	}

#ifdef RTABMAP_OCTOMAP
	_cloudViewer->removeOctomap();
	if(_preferencesDialog->isOctomapShown())
	{
		UDEBUG("");
		UTimer time;
		_octomap->update(poses);
		_cloudViewer->addOctomap(_octomap, _preferencesDialog->getOctomapTreeDepth());
		UINFO("Octomap update time = %fs", time.ticks());
	}
#endif

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
		else
		{
			UDEBUG("");
			_cloudViewer->updateCloudPose("scanMapOdom", _odometryCorrection);
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
		else
		{
			UDEBUG("");
			_cloudViewer->updateCloudPose("featuresOdom", _odometryCorrection);
			_cloudViewer->setCloudPointSize("featuresOdom", _preferencesDialog->getFeaturesPointSize(1));
		}
	}

	UDEBUG("");
	_cloudViewer->update();
	UDEBUG("");
}

std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> MainWindow::createAndAddCloudToMap(int nodeId, const Transform & pose, int mapId)
{
	UDEBUG("");
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

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		pcl::IndicesPtr indices(new std::vector<int>);
		UASSERT(nodeId == data.id());

		if(image.cols % _preferencesDialog->getCloudDecimation(0) != 0 ||
		   image.rows % _preferencesDialog->getCloudDecimation(0) != 0)
		{
			UERROR("Decimation (%d) is not modulo of the image resolution (%dx%d)! The cloud cannot be "
					"created. Go to Preferences->3D Rendering under \"Map\" column to modify this parameter.",
					_preferencesDialog->getCloudDecimation(0),
					image.cols,
					image.rows);
			return outputPair;
		}

		// Create organized cloud
		cloud = util3d::cloudRGBFromSensorData(data,
				_preferencesDialog->getCloudDecimation(0),
				_preferencesDialog->getCloudMaxDepth(0),
				_preferencesDialog->getCloudMinDepth(0),
				indices.get(),
				_preferencesDialog->getAllParameters());

		// view point
		Eigen::Vector3f viewPoint(0.0f,0.0f,0.0f);
		if(data.cameraModels().size() && !data.cameraModels()[0].localTransform().isNull())
		{
			viewPoint[0] = data.cameraModels()[0].localTransform().x();
			viewPoint[1] = data.cameraModels()[0].localTransform().y();
			viewPoint[2] = data.cameraModels()[0].localTransform().z();
		}
		else if(!data.stereoCameraModel().localTransform().isNull())
		{
			viewPoint[0] = data.stereoCameraModel().localTransform().x();
			viewPoint[1] = data.stereoCameraModel().localTransform().y();
			viewPoint[2] = data.stereoCameraModel().localTransform().z();
		}

		// filtering pipeline
		if(indices->size() && _preferencesDialog->getMapVoxel() > 0.0)
		{
			cloud = util3d::voxelize(cloud, indices, _preferencesDialog->getMapVoxel());
			//generate indices for all points (they are all valid)
			indices->resize(cloud->size());
			for(unsigned int i=0; i<cloud->size(); ++i)
			{
				indices->at(i) = i;
			}
		}

		// Do radius filtering after voxel filtering ( a lot faster)
		if(indices->size() &&
		   _preferencesDialog->getMapNoiseRadius() > 0.0 &&
		   _preferencesDialog->getMapNoiseMinNeighbors() > 0)
		{
			indices = rtabmap::util3d::radiusFiltering(
					cloud,
					indices,
					_preferencesDialog->getMapNoiseRadius(),
					_preferencesDialog->getMapNoiseMinNeighbors());
		}

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		if(_preferencesDialog->isSubtractFiltering() &&
		   _preferencesDialog->getSubtractFilteringRadius() > 0.0)
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

				if(_preferencesDialog->getSubtractFilteringAngle() > 0.0f)
				{
					//normals required
					if(_preferencesDialog->getNormalKSearch() > 0)
					{
						pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, indices, _preferencesDialog->getNormalKSearch(), viewPoint);
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
					util3d::filterNotUsedVerticesFromMesh(*output, polygons, *outputFiltered, outputPolygons);
					if(!_cloudViewer->addCloudMesh(cloudName, outputFiltered, outputPolygons, pose))
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
					pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, indices, _preferencesDialog->getNormalKSearch(), viewPoint);
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
				if(_preferencesDialog->isCloudsKept())
				{
					_cachedClouds.insert(std::make_pair(nodeId, outputPair));
					_createdCloudsMemoryUsage += output->size() * sizeof(pcl::PointXYZRGB) + indices->size()*sizeof(int);
				}
			}
		}
		_cloudViewer->setCloudOpacity(cloudName, _preferencesDialog->getCloudOpacity(0));
		_cloudViewer->setCloudPointSize(cloudName, _preferencesDialog->getCloudPointSize(0));
	}

	UDEBUG("");
	return outputPair;
}

void MainWindow::createAndAddProjectionMap(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int nodeId,
		const Transform & pose,
		bool updateOctomap)
{
	UDEBUG("");
	UASSERT(!pose.isNull());

	if(_projectionLocalMaps.find(nodeId) != _projectionLocalMaps.end() && !updateOctomap)
	{
		UERROR("Projection map %d already added.", nodeId);
		return;
	}

	if(indices->size())
	{
		UTimer timer;
		cv::Mat ground, obstacles;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelCloud = cloud;

		// voxelize to grid cell size
		if(_preferencesDialog->getMapVoxel() < _preferencesDialog->getGridMapResolution())
		{
			voxelCloud = util3d::voxelize(voxelCloud, indices, _preferencesDialog->getGridMapResolution());
		}

		// add pose rotation without yaw
		float roll, pitch, yaw;
		pose.getEulerAngles(roll, pitch, yaw);
		voxelCloud = util3d::transformPointCloud(voxelCloud, Transform(0,0, _preferencesDialog->projMapFrame()?pose.z():0, roll, pitch, 0));

		if(_preferencesDialog->projMaxObstaclesHeight())
		{
			voxelCloud = util3d::passThrough(voxelCloud, "z", std::numeric_limits<int>::min(), _preferencesDialog->projMaxObstaclesHeight());
		}

		pcl::IndicesPtr groundIndices, obstaclesIndices;
		util3d::segmentObstaclesFromGround<pcl::PointXYZRGB>(
				voxelCloud,
				groundIndices,
				obstaclesIndices,
				20,
				_preferencesDialog->projMaxGroundAngle(),
				_preferencesDialog->getGridMapResolution()*2.0f,
				_preferencesDialog->projMinClusterSize(),
				_preferencesDialog->projFlatObstaclesDetected(),
				_preferencesDialog->projMaxGroundHeight());

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		if(groundIndices->size())
		{
			pcl::copyPointCloud(*voxelCloud, *groundIndices, *groundCloud);
		}

		if(obstaclesIndices->size())
		{
			pcl::copyPointCloud(*voxelCloud, *obstaclesIndices, *obstaclesCloud);
		}

		util3d::occupancy2DFromGroundObstacles<pcl::PointXYZRGB>(
				groundCloud,
				obstaclesCloud,
				ground,
				obstacles,
				_preferencesDialog->getGridMapResolution());

		if(updateOctomap)
		{
			// Update octomap
#ifdef RTABMAP_OCTOMAP
			if(_octomap->addedNodes().empty() ||
				nodeId > _octomap->addedNodes().rbegin()->first)
			{
				Transform tinv = Transform(0,0,_preferencesDialog->projMapFrame()?pose.z():0, roll, pitch, 0).inverse();
				groundCloud = util3d::transformPointCloud(groundCloud, tinv);
				obstaclesCloud = util3d::transformPointCloud(obstaclesCloud, tinv);

				if(_preferencesDialog->isOctomapGroundAnObstacle())
				{
					*obstaclesCloud += *groundCloud;
					groundCloud->clear();
				}
				_octomap->addToCache(nodeId, groundCloud, obstaclesCloud);
			}
#endif
		}

		_projectionLocalMaps.insert(std::make_pair(nodeId, std::make_pair(ground, obstacles)));
		UDEBUG("time gridMapFrom3DCloud = %f s", timer.ticks());
	}
	UDEBUG("");
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

	if(!iter->sensorData().laserScanCompressed().empty() || !iter->sensorData().laserScanRaw().empty())
	{
		cv::Mat scan;
		iter->sensorData().uncompressData(0, 0, &scan);

		if(_preferencesDialog->getDownsamplingStepScan(0) > 0)
		{
			scan = util3d::downsample(scan, _preferencesDialog->getDownsamplingStepScan(0));
		}

		if(scan.channels() == 6)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
			cloud = util3d::laserScanToPointCloudNormal(scan);
			if(_preferencesDialog->getCloudVoxelSizeScan(0) > 0.0)
			{
				cloud = util3d::voxelize(cloud, _preferencesDialog->getCloudVoxelSizeScan(0));
			}
			QColor color = Qt::gray;
			if(mapId >= 0)
			{
				color = (Qt::GlobalColor)(mapId+3 % 12 + 7 );
			}
			if(!_cloudViewer->addCloud(scanName, cloud, pose, color))
			{
				UERROR("Adding cloud %d to viewer failed!", nodeId);
			}
			else
			{
				if(_preferencesDialog->getCloudVoxelSizeScan(0) > 0.0)
				{
					//reconvert the voxelized cloud
					scan = util3d::laserScanFromPointCloud(*cloud);
				}
				_createdScans.insert(std::make_pair(nodeId, scan));
			}
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
			cloud = util3d::laserScanToPointCloud(scan);
			if(_preferencesDialog->getCloudVoxelSizeScan(0) > 0.0)
			{
				cloud = util3d::voxelize(cloud, _preferencesDialog->getCloudVoxelSizeScan(0));
			}
			QColor color = Qt::gray;
			if(mapId >= 0)
			{
				color = (Qt::GlobalColor)(mapId+3 % 12 + 7 );
			}
			if(!_cloudViewer->addCloud(scanName, cloud, pose, color))
			{
				UERROR("Adding cloud %d to viewer failed!", nodeId);
			}
			else
			{
				if(_preferencesDialog->getCloudVoxelSizeScan(0) > 0.0)
				{
					//reconvert the voxelized cloud
					if(scan.channels() == 2)
					{
						scan = util3d::laserScan2dFromPointCloud(*cloud);
					}
					else
					{
						scan = util3d::laserScanFromPointCloud(*cloud);
					}
				}
				_createdScans.insert(std::make_pair(nodeId, scan));

				if(scan.channels() == 2)
				{
					cv::Mat ground, obstacles;
					util3d::occupancy2DFromLaserScan(scan, ground, obstacles, _preferencesDialog->getGridMapResolution());
					_gridLocalMaps.insert(std::make_pair(nodeId, std::make_pair(ground, obstacles)));
				}
			}
		}
		_cloudViewer->setCloudOpacity(scanName, _preferencesDialog->getScanOpacity(0));
		_cloudViewer->setCloudPointSize(scanName, _preferencesDialog->getScanPointSize(0));
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
		std::multimap<int, cv::KeyPoint>::const_iterator kter=iter->getWords().begin();
		for(std::multimap<int, cv::Point3f>::const_iterator jter=iter->getWords3().begin();
				jter!=iter->getWords3().end(); ++jter, ++kter, ++oi)
		{
			(*cloud)[oi].x = jter->second.x;
			(*cloud)[oi].y = jter->second.y;
			(*cloud)[oi].z = jter->second.z;
			int u = kter->second.pt.x+0.5;
			int v = kter->second.pt.y+0.5;
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
		}
		if(!_cloudViewer->addCloud(cloudName, cloud, pose, color))
		{
			UERROR("Adding features cloud %d to viewer failed!", nodeId);
		}
		else
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
		std::map<int, Transform> & poses,
		const std::map<int, Transform> & groundTruth)
{
	Transform t = Transform::getIdentity();
	if(groundTruth.size() && poses.size())
	{
		unsigned int maxSize = poses.size()>groundTruth.size()? (unsigned int)poses.size(): (unsigned int)groundTruth.size();
		pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
		cloud1.resize(maxSize);
		cloud2.resize(maxSize);
		int oi = 0;
		int idFirst = 0;
		for(std::map<int, Transform>::const_iterator iter=groundTruth.begin(); iter!=groundTruth.end(); ++iter)
		{
			std::map<int, Transform>::iterator iter2 = poses.find(iter->first);
			if(iter2!=poses.end())
			{
				if(oi==0)
				{
					idFirst = iter->first;
				}
				cloud1[oi] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
				cloud2[oi++] = pcl::PointXYZ(iter2->second.x(), iter2->second.y(), iter2->second.z());
			}
		}

		if(oi>5)
		{
			cloud1.resize(oi);
			cloud2.resize(oi);
		
			t = util3d::transformFromXYZCorrespondencesSVD(cloud2, cloud1);
		}
		else if(idFirst)
		{
			t = groundTruth.at(idFirst) * poses.at(idFirst).inverse();
		}
		if(!t.isIdentity())
		{
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				iter->second = t * iter->second;
			}
		}
		UDEBUG("t=%s", t.prettyPrint().c_str());
	}
	return t;
}

void MainWindow::updateNodeVisibility(int nodeId, bool visible)
{
	if(_currentPosesMap.find(nodeId) != _currentPosesMap.end())
	{
		QMap<std::string, Transform> viewerClouds = _cloudViewer->getAddedClouds();
		if(_preferencesDialog->isCloudsShown(0))
		{
			std::string cloudName = uFormat("cloud%d", nodeId);
			if(visible && !viewerClouds.contains(cloudName) && _cachedSignatures.contains(nodeId))
			{
				createAndAddCloudToMap(nodeId, _currentPosesMap.find(nodeId)->second, uValue(_currentMapIds, nodeId, -1));
			}
			else if(viewerClouds.contains(cloudName))
			{
				if(visible)
				{
					//make sure the transformation was done
					_cloudViewer->updateCloudPose(cloudName, _currentPosesMap.find(nodeId)->second);
				}
				_cloudViewer->setCloudVisibility(cloudName, visible);
			}
		}

		if(_preferencesDialog->isScansShown(0))
		{
			std::string scanName = uFormat("scan%d", nodeId);
			if(visible && !viewerClouds.contains(scanName) && _cachedSignatures.contains(nodeId))
			{
				createAndAddScanToMap(nodeId, _currentPosesMap.find(nodeId)->second, uValue(_currentMapIds, nodeId, -1));
			}
			else if(viewerClouds.contains(scanName))
			{
				if(visible)
				{
					//make sure the transformation was done
					_cloudViewer->updateCloudPose(scanName, _currentPosesMap.find(nodeId)->second);
				}
				_cloudViewer->setCloudVisibility(scanName, visible);
			}
		}
	}
	_cloudViewer->update();
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
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		this->changeState(MainWindow::kInitializing);
	}
	else if((RtabmapEventInit::Status)status == RtabmapEventInit::kInitialized)
	{
		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
		this->changeState(MainWindow::kInitialized);

		if(!_openedDatabasePath.isEmpty())
		{
			this->downloadAllClouds();
		}
	}
	else if((RtabmapEventInit::Status)status == RtabmapEventInit::kClosing)
	{
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		if(_state!=kApplicationClosing)
		{
			this->changeState(MainWindow::kClosing);
		}
	}
	else if((RtabmapEventInit::Status)status == RtabmapEventInit::kClosed)
	{
		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());

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
				else
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
			else
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
		_initProgressDialog->incrementStep();
		QString msg(info);
		if((RtabmapEventInit::Status)status == RtabmapEventInit::kError)
		{
			_openedDatabasePath.clear();
			_newDatabasePath.clear();
			_newDatabasePathOutput.clear();
			_initProgressDialog->setAutoClose(false);
			msg.prepend(tr("[ERROR] "));
			_initProgressDialog->appendText(msg);
			this->changeState(MainWindow::kIdle);
		}
		else
		{
			_initProgressDialog->appendText(msg);
		}
	}
}

void MainWindow::processRtabmapEvent3DMap(const rtabmap::RtabmapEvent3DMap & event)
{
	_initProgressDialog->appendText("Downloading the map... done.");
	_initProgressDialog->incrementStep();

	if(event.getCode())
	{
		UERROR("Map received with code error %d!", event.getCode());
		_initProgressDialog->appendText(uFormat("[ERROR] Map received with code error %d!", event.getCode()).c_str());
		_initProgressDialog->setAutoClose(false);
	}
	else
	{

		_processingDownloadedMap = true;
		UINFO("Received map!");
		_initProgressDialog->appendText(tr(" poses = %1").arg(event.getPoses().size()));
		_initProgressDialog->appendText(tr(" constraints = %1").arg(event.getConstraints().size()));

		_initProgressDialog->setMaximumSteps(int(event.getSignatures().size()+event.getPoses().size()+1));
		_initProgressDialog->appendText(QString("Inserting data in the cache (%1 signatures downloaded)...").arg(event.getSignatures().size()));
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
				++addedSignatures;
			}
			_initProgressDialog->incrementStep();
			QApplication::processEvents();
		}
		_initProgressDialog->appendText(tr("Inserted %1 new signatures.").arg(addedSignatures));
		_initProgressDialog->incrementStep();
		QApplication::processEvents();

		_initProgressDialog->appendText("Inserting data in the cache... done.");

		if(event.getPoses().size())
		{
			_initProgressDialog->appendText("Updating the 3D map cloud...");
			_initProgressDialog->incrementStep();
			QApplication::processEvents();
			std::map<int, Transform> poses = event.getPoses();
			alignPosesToGroundTruth(poses, groundTruth);
			this->updateMapCloud(poses, event.getConstraints(), mapIds, labels, groundTruth, true);
			_initProgressDialog->appendText("Updating the 3D map cloud... done.");
		}
		else
		{
			_initProgressDialog->appendText("No poses received! The map cloud cannot be updated...");
			UINFO("Map received is empty! Cannot update the map cloud...");
		}

		_initProgressDialog->appendText(tr("%1 locations are updated to/inserted in the cache.").arg(event.getPoses().size()));

		if(!_preferencesDialog->isImagesKept())
		{
			_cachedSignatures.clear();
			_cachedMemoryUsage = 0;
		}
		if(_state != kMonitoring && _state != kDetecting)
		{
			_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
			_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		}
		_processingDownloadedMap = false;
	}
	_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
}

void MainWindow::processRtabmapGlobalPathEvent(const rtabmap::RtabmapGlobalPathEvent & event)
{
	if(!event.getPoses().empty())
	{
		_ui->graphicsView_graphView->setGlobalPath(event.getPoses());
	}

	_ui->statsToolBox->updateStat("Planning/From/", float(event.getPoses().size()?event.getPoses().begin()->first:0));
	_ui->statsToolBox->updateStat("Planning/Time/ms", float(event.getPlanningTime()*1000.0));
	_ui->statsToolBox->updateStat("Planning/Goal/", float(event.getGoal()));
	_ui->statsToolBox->updateStat("Planning/Poses/", float(event.getPoses().size()));
	_ui->statsToolBox->updateStat("Planning/Length/m", float(graph::computePathLength(event.getPoses())));

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
				_camera->setImageRate( _preferencesDialog->getSourceDatabaseStampsUsed()?-1:_preferencesDialog->getGeneralInputRate());
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
	}
}

void MainWindow::applyPrefSettings(const rtabmap::ParametersMap & parameters)
{
	applyPrefSettings(parameters, true); //post parameters
}

void MainWindow::applyPrefSettings(const rtabmap::ParametersMap & parameters, bool postParamEvent)
{
	ULOGGER_DEBUG("");
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
			_cloudViewer->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
		}

		if(_state != kIdle && parametersModified.size())
		{
			if(parametersModified.erase(Parameters::kRtabmapWorkingDirectory()) &&
				_state != kMonitoring &&
				_state != kMonitoringPaused)
			{
				QMessageBox::information(this, tr("Working memory changed"),
						tr("The working directory can't be changed while the "
								"detector is running (state=%1). This will be "
								"applied when the detector will stop.").arg(_state));
			}
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

	float value;
	value = float(_preferencesDialog->getLoopThr());
	emit(loopClosureThrChanged(value));
}

void MainWindow::drawKeypoints(const std::multimap<int, cv::KeyPoint> & refWords, const std::multimap<int, cv::KeyPoint> & loopWords)
{
	UTimer timer;

	timer.start();
	ULOGGER_DEBUG("refWords.size() = %d", refWords.size());
	for(std::multimap<int, cv::KeyPoint>::const_iterator iter = refWords.begin(); iter != refWords.end(); ++iter )
	{
		int id = iter->first;
		QColor color;
		if(uContains(loopWords, id))
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
	for(std::multimap<int, cv::KeyPoint>::const_iterator iter = loopWords.begin(); iter != loopWords.end(); ++iter )
	{
		int id = iter->first;
		QColor color;
		if(uContains(refWords, id))
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

	for(QList<QPair<cv::Point2f, cv::Point2f> >::iterator iter = uniqueCorrespondences.begin();
		iter!=uniqueCorrespondences.end();
		++iter)
	{

		_ui->imageView_source->addLine(
				iter->first.x,
				iter->first.y,
				(iter->second.x*scaleLoop+loopMarginX+deltaX-sourceMarginX)/scaleSource,
				(iter->second.y*scaleLoop+loopMarginY+deltaY-sourceMarginY)/scaleSource,
				Qt::cyan);

		_ui->imageView_loopClosure->addLine(
				(iter->first.x*scaleSource+sourceMarginX-deltaX-loopMarginX)/scaleLoop,
				(iter->first.y*scaleSource+sourceMarginY-deltaY-loopMarginY)/scaleLoop,
				iter->second.x,
				iter->second.y,
				Qt::cyan);
	}
	_ui->imageView_source->update();
	_ui->imageView_loopClosure->update();
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
	_ui->actionOpenNI2_sense->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcOpenNI2);
	_ui->actionFreenect2->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcFreenect2);
	_ui->actionStereoDC1394->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcDC1394);
	_ui->actionStereoFlyCapture2->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcFlyCapture2);
	_ui->actionStereoZed->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoZed);
	_ui->actionStereoUsb->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoUsb);
}

void MainWindow::changeImgRateSetting()
{
	emit imgRateChanged(_ui->doubleSpinBox_stats_imgRate->value());
}

void MainWindow::changeDetectionRateSetting()
{
	emit detectionRateChanged(_ui->doubleSpinBox_stats_detectionRate->value());
}

void MainWindow::changeTimeLimitSetting()
{
	emit timeLimitChanged((float)_ui->doubleSpinBox_stats_timeLimit->value());
}

void MainWindow::changeMappingMode()
{
	emit mappingModeChanged(_ui->actionSLAM_mode->isChecked());
}

QString MainWindow::captureScreen(bool cacheInRAM)
{
	QString name = (QDateTime::currentDateTime().toString("yyMMddhhmmsszzz") + ".png");
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
		figure.save(&buffer, "PNG");
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

void MainWindow::configGUIModified()
{
	this->setWindowModified(true);
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
	_preferencesDialog->saveWidgetState(_exportScansDialog);
	_preferencesDialog->saveWidgetState(_postProcessingDialog);
	_preferencesDialog->saveWidgetState(_ui->graphicsView_graphView);
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
				  "Do you want to continue (the database will be deleted to create the new one)?").arg(databasePath.c_str()),
				  QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

		if(r == QMessageBox::Yes)
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
	QString path = QFileDialog::getOpenFileName(this, tr("Open database..."), _preferencesDialog->getWorkingDirectory(), tr("RTAB-Map database files (*.db)"));
	if(!path.isEmpty())
	{
		this->openDatabase(path);
	}
}

void MainWindow::openDatabase(const QString & path)
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

		// look if there are saved parameters
		DBDriver * driver = DBDriver::create();
		if(driver->openConnection(value, false))
		{
			ParametersMap parameters = driver->getLastParameters();
			driver->closeConnection(false);
			delete driver;

			if(parameters.size())
			{
				ParametersMap currentParameters = _preferencesDialog->getAllParameters();
				ParametersMap differentParameters;
				for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
				{
					ParametersMap::iterator jter = currentParameters.find(iter->first);
					if(jter!=currentParameters.end() &&
					   iter->second.compare(jter->second) != 0 &&
					   iter->first.compare(Parameters::kRtabmapWorkingDirectory()) != 0)
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

void MainWindow::startDetection()
{
	UDEBUG("");
	ParametersMap parameters = _preferencesDialog->getAllParameters();
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
			(_preferencesDialog->getSourceDriver() != PreferencesDialog::kSrcDatabase || !_preferencesDialog->getSourceDatabaseStampsUsed()))
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
		if(bufferingSize != 0 &&
		  (_preferencesDialog->getSourceDriver() != PreferencesDialog::kSrcDatabase || !_preferencesDialog->getSourceDatabaseStampsUsed()))
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
	}

	UDEBUG("");
	emit stateChanged(kStartingDetection);

	if(_camera != 0)
	{
		QMessageBox::warning(this,
						     tr("RTAB-Map"),
						     tr("A camera is running, stop it first."));
		UWARN("_camera is not null... it must be stopped first");
		emit stateChanged(kInitialized);
		return;
	}

	// Adjust pre-requirements
	if(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcUndef)
	{
		QMessageBox::warning(this,
				 tr("RTAB-Map"),
				 tr("No sources are selected. See Preferences->Source panel."));
		UWARN("No sources are selected. See Preferences->Source panel.");
		emit stateChanged(kInitialized);
		return;
	}


	Camera * camera = _preferencesDialog->createCamera();
	if(!camera)
	{
		emit stateChanged(kInitialized);
		return;
	}

	_camera = new CameraThread(camera, parameters);
	_camera->setMirroringEnabled(_preferencesDialog->isSourceMirroring());
	_camera->setColorOnly(_preferencesDialog->isSourceRGBDColorOnly());
	_camera->setImageDecimation(_preferencesDialog->getSourceImageDecimation());
	_camera->setStereoToDepth(_preferencesDialog->isSourceStereoDepthGenerated());
	_camera->setScanFromDepth(
			_preferencesDialog->isSourceScanFromDepth(),
			_preferencesDialog->getSourceScanFromDepthDecimation(),
			_preferencesDialog->getSourceScanFromDepthMaxDepth(),
			_preferencesDialog->getSourceScanVoxelSize(),
			_preferencesDialog->getSourceScanNormalsK());

	//Create odometry thread if rgbd slam
	if(uStr2Bool(parameters.at(Parameters::kRGBDEnabled()).c_str()))
	{
		// Require calibrated camera
		if(!camera->isCalibrated())
		{
			UWARN("Camera is not calibrated!");
			emit stateChanged(kInitialized);
			delete _camera;
			_camera = 0;

			int button = QMessageBox::question(this,
					tr("Camera is not calibrated!"),
					tr("RTAB-Map cannot run with an uncalibrated camera. Do you want to calibrate the camera now?"),
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

			if(!camera->odomProvided())
			{
				Odometry * odom = Odometry::create(parameters);
				_odomThread = new OdometryThread(odom, _preferencesDialog->getOdomBufferSize());

				UEventsManager::addHandler(_odomThread);
				UEventsManager::createPipe(_camera, _odomThread, "CameraEvent");
				UEventsManager::createPipe(_camera, this, "CameraEvent");
				_odomThread->start();
			}
		}
	}

	if(_dataRecorder && _camera)
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

#ifdef RTABMAP_OCTOMAP
	UASSERT(_octomap != 0);
	delete _octomap;
	_octomap = new OctoMap(_preferencesDialog->getGridMapResolution());
#endif

	emit stateChanged(kDetecting);
}

// Could not be in the main thread here! (see handleEvents())
void MainWindow::pauseDetection()
{
	if(_camera)
	{
		if(_state == kPaused && (QApplication::keyboardModifiers() & Qt::ShiftModifier))
		{
			// On Ctrl-click, start the camera and pause it automatically
			emit stateChanged(kPaused);
			if(_preferencesDialog->getGeneralInputRate())
			{
				QTimer::singleShot(1000.0/_preferencesDialog->getGeneralInputRate() + 10, this, SLOT(pauseDetection()));
			}
			else
			{
				emit stateChanged(kPaused);
			}
		}
		else
		{
			emit stateChanged(kPaused);
		}
	}
	else if(_state == kMonitoring)
	{
		UINFO("Sending pause event!");
		emit stateChanged(kMonitoringPaused);
	}
	else if(_state == kMonitoringPaused)
	{
		UINFO("Sending unpause event!");
		emit stateChanged(kMonitoring);
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

	emit stateChanged(kInitialized);
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
void MainWindow::exportPosesRGBDSLAM()
{
	exportPoses(1);
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
		std::map<int, double> stamps;
		if(format == 1)
		{
			for(std::map<int, Transform>::iterator iter=_currentPosesMap.begin(); iter!=_currentPosesMap.end(); ++iter)
			{
				if(_cachedSignatures.contains(iter->first))
				{
					stamps.insert(std::make_pair(iter->first, _cachedSignatures.value(iter->first).getStamp()));
				}
			}
			if(stamps.size()!=_currentPosesMap.size())
			{
				QMessageBox::warning(this, tr("Export poses..."), tr("RGB-D SLAM format: Poses (%1) and stamps (%2) have not the same size! Try again after updating the cache.")
						.arg(_currentPosesMap.size()).arg(stamps.size()));
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
			_exportPosesFileName[format] = path;

			bool saved = graph::exportPoses(path.toStdString(), format, _currentPosesMap, _currentLinksMap, stamps);

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

void MainWindow::postProcessing()
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
	if(_postProcessingDialog->exec() != QDialog::Accepted)
	{
		return;
	}

	bool detectMoreLoopClosures = _postProcessingDialog->isDetectMoreLoopClosures();
	bool refineNeighborLinks = _postProcessingDialog->isRefineNeighborLinks();
	bool refineLoopClosureLinks = _postProcessingDialog->isRefineLoopClosureLinks();
	double clusterRadius = _postProcessingDialog->clusterRadius();
	double clusterAngle = _postProcessingDialog->clusterAngle();
	int detectLoopClosureIterations = _postProcessingDialog->iterations();
	bool sba = _postProcessingDialog->isSBA();
	int sbaIterations = _postProcessingDialog->sbaIterations();
	double sbaEpsilon = _postProcessingDialog->sbaEpsilon();
	double sbaVariance = _postProcessingDialog->sbaVariance();
	Optimizer::Type sbaType = _postProcessingDialog->sbaType();

	if(!detectMoreLoopClosures && !refineNeighborLinks && !refineLoopClosureLinks && !sba)
	{
		UWARN("No post-processing selection...");
		return;
	}

	// First, verify that we have all data required in the GUI
	bool allDataAvailable = true;
	std::map<int, Transform> odomPoses;
	for(std::map<int, Transform>::iterator iter = _currentPosesMap.begin();
			iter!=_currentPosesMap.end() && allDataAvailable;
			++iter)
	{
		QMap<int, Signature>::iterator jter = _cachedSignatures.find(iter->first);
		if(jter != _cachedSignatures.end())
		{
			if(jter->getPose().isNull())
			{
				UWARN("Odometry pose of %d is null.", iter->first);
				allDataAvailable = false;
			}
			else
			{
				odomPoses.insert(*iter); // fill raw poses
			}
		}
		else
		{
			UWARN("Node %d missing.", iter->first);
			allDataAvailable = false;
		}
	}

	if(!allDataAvailable)
	{
		QMessageBox::warning(this, tr("Not all data available in the GUI..."),
				tr("Some data missing in the cache to respect the constraints chosen. "
				   "Try \"Edit->Download all clouds\" to update the cache and try again."));
		return;
	}

	_initProgressDialog->resetProgress();
	_initProgressDialog->clear();
	_initProgressDialog->show();
	_initProgressDialog->appendText("Post-processing beginning!");

	int totalSteps = 0;
	if(refineNeighborLinks)
	{
		totalSteps+=(int)odomPoses.size();
	}
	if(refineLoopClosureLinks)
	{
		totalSteps+=(int)_currentLinksMap.size() - (int)odomPoses.size();
	}
	if(sba)
	{
		totalSteps+=1;
	}
	_initProgressDialog->setMaximumSteps(totalSteps);
	_initProgressDialog->show();

	ParametersMap parameters = _preferencesDialog->getAllParameters();
	Optimizer * optimizer = Optimizer::create(parameters);
	bool optimizeFromGraphEnd =  Parameters::defaultRGBDOptimizeFromGraphEnd();
	Parameters::parse(parameters, Parameters::kRGBDOptimizeFromGraphEnd(), optimizeFromGraphEnd);

	bool warn = false;
	int loopClosuresAdded = 0;
	std::multimap<int, int> checkedLoopClosures;
	if(detectMoreLoopClosures)
	{
		UDEBUG("");

		UASSERT(detectLoopClosureIterations>0);
		for(int n=0; n<detectLoopClosureIterations; ++n)
		{
			_initProgressDialog->appendText(tr("Looking for more loop closures, clustering poses... (iteration=%1/%2, radius=%3 m angle=%4 degrees)")
					.arg(n+1).arg(detectLoopClosureIterations).arg(clusterRadius).arg(clusterAngle));

			std::multimap<int, int> clusters = graph::radiusPosesClustering(
					_currentPosesMap,
					clusterRadius,
					clusterAngle*CV_PI/180.0);

			_initProgressDialog->setMaximumSteps(_initProgressDialog->maximumSteps()+(int)clusters.size());
			_initProgressDialog->appendText(tr("Looking for more loop closures, clustering poses... found %1 clusters.").arg(clusters.size()));

			int i=0;
			std::set<int> addedLinks;
			for(std::multimap<int, int>::iterator iter=clusters.begin(); iter!= clusters.end(); ++iter, ++i)
			{
				int from = iter->first;
				int to = iter->second;
				if(iter->first < iter->second)
				{
					from = iter->second;
					to = iter->first;
				}

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
					   rtabmap::graph::findLink(_currentLinksMap, from, to) == _currentLinksMap.end())
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
							_initProgressDialog->incrementStep();
							QApplication::processEvents();

							Signature signatureFrom = _cachedSignatures[from];
							Signature signatureTo = _cachedSignatures[to];

							if(signatureFrom.getWeight() >= 0 &&
							   signatureTo.getWeight() >= 0) // ignore intermediate nodes
							{
								Transform transform;
								RegistrationInfo info;
								RegistrationVis registration(parameters);
								transform = registration.computeTransformation(signatureFrom, signatureTo, Transform(), &info);

								if(!transform.isNull())
								{
									UINFO("Added new loop closure between %d and %d.", from, to);
									addedLinks.insert(from);
									addedLinks.insert(to);
									_currentLinksMap.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, transform, info.variance, info.variance)));
									++loopClosuresAdded;
									_initProgressDialog->appendText(tr("Detected loop closure %1->%2! (%3/%4)").arg(from).arg(to).arg(i+1).arg(clusters.size()));
								}
							}
						}
					}
				}
			}
			_initProgressDialog->appendText(tr("Iteration %1/%2: Detected %3 loop closures!")
					.arg(n+1).arg(detectLoopClosureIterations).arg(addedLinks.size()/2));
			if(addedLinks.size() == 0)
			{
				break;
			}

			if(n+1 < detectLoopClosureIterations)
			{
				_initProgressDialog->appendText(tr("Optimizing graph with new links (%1 nodes, %2 constraints)...")
						.arg(odomPoses.size()).arg(_currentLinksMap.size()));
				int fromId = optimizeFromGraphEnd?odomPoses.rbegin()->first:odomPoses.begin()->first;
				std::map<int, rtabmap::Transform> posesOut;
				std::multimap<int, rtabmap::Link> linksOut;
				std::map<int, rtabmap::Transform> optimizedPoses;
				optimizer->getConnectedGraph(
						fromId,
						odomPoses,
						_currentLinksMap,
						posesOut,
						linksOut);
				optimizedPoses = optimizer->optimize(fromId, posesOut, linksOut);
				_currentPosesMap = optimizedPoses;
				_initProgressDialog->appendText(tr("Optimizing graph with new links... done!"));
			}
		}
		UINFO("Added %d loop closures.", loopClosuresAdded);
		_initProgressDialog->appendText(tr("Total new loop closures detected=%1").arg(loopClosuresAdded));
	}

	if(refineNeighborLinks || refineLoopClosureLinks)
	{
		UDEBUG("");
		if(refineLoopClosureLinks)
		{
			_initProgressDialog->setMaximumSteps(_initProgressDialog->maximumSteps()+loopClosuresAdded);
		}
		// TODO: support ICP from laser scans?
		_initProgressDialog->appendText(tr("Refining links..."));

		RegistrationIcp regIcp(parameters);

		int i=0;
		for(std::multimap<int, Link>::iterator iter = _currentLinksMap.begin(); iter!=_currentLinksMap.end(); ++iter, ++i)
		{
			int type = iter->second.type();

			if((refineNeighborLinks && type==Link::kNeighbor) ||
			   (refineLoopClosureLinks && type!=Link::kNeighbor))
			{
				int from = iter->second.from();
				int to = iter->second.to();

				_initProgressDialog->appendText(tr("Refining link %1->%2 (%3/%4)").arg(from).arg(to).arg(i+1).arg(_currentLinksMap.size()));
				_initProgressDialog->incrementStep();
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

					if(!signatureFrom.sensorData().laserScanRaw().empty() &&
					   !signatureTo.sensorData().laserScanRaw().empty())
					{
						RegistrationInfo info;
						Transform transform = regIcp.computeTransformation(signatureFrom.sensorData(), signatureTo.sensorData(), iter->second.transform(), &info);

						if(!transform.isNull())
						{
							Link newLink(from, to, iter->second.type(), transform*iter->second.transform(), info.variance, info.variance);
							iter->second = newLink;
						}
						else
						{
							QString str = tr("Cannot refine link %1->%2 (%3").arg(from).arg(to).arg(info.rejectedMsg.c_str());
							_initProgressDialog->appendText(str, Qt::darkYellow);
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
							str = tr("Cannot refine link %1->%2 (clouds empty!)").arg(from).arg(to);
						}

						_initProgressDialog->appendText(str, Qt::darkYellow);
						UWARN("%s", str.toStdString().c_str());
						warn = true;
					}
				}
			}
		}
		_initProgressDialog->appendText(tr("Refining links...done!"));
	}

	_initProgressDialog->appendText(tr("Optimizing graph with updated links (%1 nodes, %2 constraints)...")
			.arg(odomPoses.size()).arg(_currentLinksMap.size()));

	int fromId = optimizeFromGraphEnd?odomPoses.rbegin()->first:odomPoses.begin()->first;
	std::map<int, rtabmap::Transform> posesOut;
	std::multimap<int, rtabmap::Link> linksOut;
	std::map<int, rtabmap::Transform> optimizedPoses;
	optimizer->getConnectedGraph(
			fromId,
			odomPoses,
			_currentLinksMap,
			posesOut,
			linksOut);
	optimizedPoses = optimizer->optimize(fromId, posesOut, linksOut);
	_initProgressDialog->appendText(tr("Optimizing graph with updated links... done!"));
	_initProgressDialog->incrementStep();

	if(sba)
	{
		UASSERT(Optimizer::isAvailable(sbaType));
		_initProgressDialog->appendText(tr("SBA (%1 nodes, %2 constraints, %3 iterations)...")
					.arg(optimizedPoses.size()).arg(linksOut.size()).arg(sbaIterations));
		QApplication::processEvents();
		uSleep(100);
		QApplication::processEvents();

		ParametersMap parametersSBA = _preferencesDialog->getAllParameters();
		uInsert(parametersSBA, std::make_pair(Parameters::kOptimizerIterations(), uNumber2Str(sbaIterations)));
		uInsert(parametersSBA, std::make_pair(Parameters::kOptimizerEpsilon(), uNumber2Str(sbaEpsilon)));
		uInsert(parametersSBA, std::make_pair(Parameters::kg2oPixelVariance(), uNumber2Str(sbaVariance)));
		Optimizer * sba = Optimizer::create(sbaType, parametersSBA);
		std::map<int, Transform>  newPoses = sba->optimizeBA(optimizedPoses.begin()->first, optimizedPoses, linksOut, _cachedSignatures.toStdMap());
		delete sba;
		if(newPoses.size())
		{
			optimizedPoses = newPoses;
			_initProgressDialog->appendText(tr("SBA... done!"));
		}
		else
		{
			_initProgressDialog->appendText(tr("SBA... failed!"));
			_initProgressDialog->setAutoClose(false);
		}
		_initProgressDialog->incrementStep();
	}

	_initProgressDialog->appendText(tr("Updating map..."));
	alignPosesToGroundTruth(optimizedPoses, _currentGTPosesMap);
	this->updateMapCloud(
			optimizedPoses,
			std::multimap<int, Link>(_currentLinksMap),
			std::map<int, int>(_currentMapIds),
			std::map<int, std::string>(_currentLabels),
			std::map<int, Transform>(_currentGTPosesMap),
			false);
	_initProgressDialog->appendText(tr("Updating map... done!"));

	if(warn)
	{
		_initProgressDialog->setAutoClose(false);
	}

	_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
	_initProgressDialog->appendText("Post-processing finished!");

	delete optimizer;
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

void MainWindow::selectStereoUsb()
{
	_preferencesDialog->selectSourceDriver(PreferencesDialog::kSrcStereoUsb);
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
			_initProgressDialog->resetProgress();
			_initProgressDialog->show();
			_initProgressDialog->appendText(tr("Downloading the map from \"%1\" (without poses and links)...")
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
	QString item = QInputDialog::getItem(this, tr("Download map"), tr("Options:"), items, 2, false, &ok);
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
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		_initProgressDialog->appendText(tr("Downloading the map (global=%1 ,optimized=%2)...")
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
	QString item = QInputDialog::getItem(this, tr("Download graph"), tr("Options:"), items, 2, false, &ok);
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
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		_initProgressDialog->appendText(tr("Downloading the graph (global=%1 ,optimized=%2)...")
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
	_cachedClouds.clear();
	_createdCloudsMemoryUsage = 0;
	_previousCloud.first = 0;
	_previousCloud.second.first.first.reset();
	_previousCloud.second.first.second.reset();
	_previousCloud.second.second.reset();
	_createdScans.clear();
	_gridLocalMaps.clear();
	_projectionLocalMaps.clear();
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
	//disable save cloud action
	_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(false);
	_ui->actionExport_2D_scans_ply_pcd->setEnabled(false);
	_ui->actionPost_processing->setEnabled(false);
	_ui->actionSave_point_cloud->setEnabled(false);
	_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(false);
	_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(false);
	_ui->actionView_scans->setEnabled(false);
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
	_ui->label_refId->clear();
	_ui->label_matchId->clear();
	_ui->graphicsView_graphView->clearAll();
	_ui->imageView_source->clear();
	_ui->imageView_loopClosure->clear();
	_ui->imageView_odometry->clear();
	_ui->imageView_source->setBackgroundColor(Qt::black);
	_ui->imageView_loopClosure->setBackgroundColor(Qt::black);
	_ui->imageView_odometry->setBackgroundColor(Qt::black);
#ifdef RTABMAP_OCTOMAP
	// re-create one if the resolution has changed
	UASSERT(_octomap != 0);
	delete _octomap;
	_octomap = new OctoMap(_preferencesDialog->getGridMapResolution());
#endif
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
	_ui->toolBar->setVisible(_state != kMonitoring && _state != kMonitoringPaused);
	_ui->toolBar_2->setVisible(true);
	_ui->statusbar->setVisible(false);
	this->setAspectRatio720p();
	_cloudViewer->resetCamera();
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

		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		_initProgressDialog->setMaximumSteps(_autoScreenCaptureCachedImages.size());
		int i=0;
		for(QMap<QString, QByteArray>::iterator iter=_autoScreenCaptureCachedImages.begin(); iter!=_autoScreenCaptureCachedImages.end(); ++iter)
		{
			QPixmap figure;
			figure.loadFromData(iter.value(), "PNG");
			figure.save(targetDir + iter.key(), "PNG");
			_initProgressDialog->appendText(tr("Saved image \"%1\" (%2/%3).").arg(targetDir + iter.key()).arg(++i).arg(_autoScreenCaptureCachedImages.size()));
			_initProgressDialog->incrementStep();
		}
		_autoScreenCaptureCachedImages.clear();
		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
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
	double gridCellSize = 0.05;
	bool ok;
	gridCellSize = QInputDialog::getDouble(this, tr("Grid cell size"), tr("Size (m):"), gridCellSize, 0.01, 1, 2, &ok);
	if(!ok)
	{
		return;
	}

	std::map<int, Transform> poses = _ui->widget_mapVisibility->getVisiblePoses();

	// create the map
	float xMin=0.0f, yMin=0.0f;
	cv::Mat pixels = util3d::create2DMapFromOccupancyLocalMaps(
				poses,
				_preferencesDialog->isGridMapFrom3DCloud()?_projectionLocalMaps:_gridLocalMaps,
				gridCellSize,
				xMin, yMin,
				0,
				_preferencesDialog->isGridMapEroded());

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

void MainWindow::exportScans()
{
	if(_exportScansDialog->isVisible())
	{
		return;
	}

	_exportScansDialog->exportScans(
			_currentPosesMap,
			_currentMapIds,
			_cachedSignatures,
			_createdScans,
			_preferencesDialog->getWorkingDirectory());
}

void MainWindow::viewScans()
{
	if(_exportScansDialog->isVisible())
	{
		return;
	}

	_exportScansDialog->viewScans(
			_ui->widget_mapVisibility->getVisiblePoses(),
			_currentMapIds,
			_cachedSignatures,
			_createdScans,
			_preferencesDialog->getWorkingDirectory());

}

void MainWindow::exportClouds()
{
	if(_exportCloudsDialog->isVisible())
	{
		return;
	}

	_exportCloudsDialog->exportClouds(
			_ui->widget_mapVisibility->getVisiblePoses(),
			_currentMapIds,
			_cachedSignatures,
			_cachedClouds,
			_preferencesDialog->getWorkingDirectory(),
			_preferencesDialog->getAllParameters());
}

void MainWindow::viewClouds()
{
	if(_exportCloudsDialog->isVisible())
	{
		return;
	}

	_exportCloudsDialog->viewClouds(
			_currentPosesMap,
			_currentMapIds,
			_cachedSignatures,
			_cachedClouds,
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
	formats.push_back("jpg");
	formats.push_back("png");
	bool ok;
	QString ext = QInputDialog::getItem(this, tr("Which RGB format?"), tr("Format:"), formats, 0, false, &ok);
	if(!ok)
	{
		return;
	}

	QString path = QFileDialog::getExistingDirectory(this, tr("Select directory where to save images..."), this->getWorkingDirectory());
	if(!path.isNull())
	{
		SensorData data;
		if(_cachedSignatures.contains(poses.rbegin()->first))
		{
			data = _cachedSignatures.value(poses.rbegin()->first).sensorData();
			data.uncompressData();
		}
		if(!data.imageRaw().empty() && !data.rightRaw().empty())
		{
			QDir dir;
			dir.mkdir(QString("%1/left").arg(path));
			dir.mkdir(QString("%1/right").arg(path));
			if(data.stereoCameraModel().isValidForProjection())
			{
				std::string cameraName = "calibration";
				StereoCameraModel model(
						cameraName,
						data.imageRaw().size(),
						data.stereoCameraModel().left().K(),
						data.stereoCameraModel().left().D(),
						data.stereoCameraModel().left().R(),
						data.stereoCameraModel().left().P(),
						data.rightRaw().size(),
						data.stereoCameraModel().right().K(),
						data.stereoCameraModel().right().D(),
						data.stereoCameraModel().right().R(),
						data.stereoCameraModel().right().P(),
						data.stereoCameraModel().R(),
						data.stereoCameraModel().T(),
						data.stereoCameraModel().E(),
						data.stereoCameraModel().F(),
						data.stereoCameraModel().left().localTransform());
				if(model.save(path.toStdString()))
				{
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
					UINFO("Saved calibration \"%s\"", (path.toStdString()+"/"+cameraName).c_str());
				}
				else
				{
					UERROR("Failed saving calibration \"%s\"", (path.toStdString()+"/"+cameraName).c_str());
				}
			}
		}
		else
		{
			QMessageBox::warning(this,
					tr("Export images..."),
					tr("Data in the cache don't seem to have images (tested node %1). Calibration file will not be saved. Try refreshing the cache (with clouds).").arg(poses.rbegin()->first));
		}
	}

	_initProgressDialog->resetProgress();
	_initProgressDialog->show();
	_initProgressDialog->setMaximumSteps(_cachedSignatures.size());

	unsigned int saved = 0;
	for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		int id = iter->first;
		SensorData data;
		if(_cachedSignatures.contains(iter->first))
		{
			data = _cachedSignatures.value(iter->first).sensorData();
			data.uncompressData();
		}
		QString info;
		bool warn = false;
		if(!data.imageRaw().empty() && !data.rightRaw().empty())
		{
			cv::imwrite(QString("%1/left/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.imageRaw());
			cv::imwrite(QString("%1/right/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.rightRaw());
			info = tr("Saved left/%1.%2 and right/%1.%2.").arg(id).arg(ext);
		}
		else if(!data.imageRaw().empty() && !data.depthRaw().empty())
		{
			cv::imwrite(QString("%1/rgb/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.imageRaw());
			cv::imwrite(QString("%1/depth/%2.png").arg(path).arg(id).toStdString(), data.depthRaw());
			info = tr("Saved rgb/%1.%2 and depth/%1.png.").arg(id).arg(ext);
		}
		else if(!data.imageRaw().empty())
		{
			cv::imwrite(QString("%1/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.imageRaw());
			info = tr("Saved %1.%2.").arg(id).arg(ext);
		}
		else
		{
			info = tr("No images saved for node %1!").arg(id);
			warn = true;
		}
		saved += warn?0:1;
		_initProgressDialog->appendText(info, !warn?Qt::black:Qt::darkYellow);
		_initProgressDialog->incrementStep();
		QApplication::processEvents();

	}
	if(saved!=poses.size())
	{
		_initProgressDialog->setAutoClose(false);
		_initProgressDialog->appendText(tr("%1 images of %2 saved to \"%3\".").arg(saved).arg(poses.size()).arg(path));
	}
	else
	{
		_initProgressDialog->appendText(tr("%1 images saved to \"%2\".").arg(saved).arg(path));
	}

	_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
}

void MainWindow::exportBundlerFormat()
{
	std::map<int, Transform> posesIn = _ui->widget_mapVisibility->getVisiblePoses();

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
			else if((_cachedSignatures[iter->first].sensorData().cameraModels().size() == 1 && _cachedSignatures[iter->first].sensorData().cameraModels().at(0).isValidForProjection()) ||
			         _cachedSignatures[iter->first].sensorData().stereoCameraModel().isValidForProjection())
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
		QString path = QFileDialog::getExistingDirectory(this, tr("Exporting cameras in Bundler format..."), _preferencesDialog->getWorkingDirectory());
		if(!path.isEmpty())
		{
			// export cameras and images
			QFile fileOut(path+QDir::separator()+"cameras.out");
			QFile fileList(path+QDir::separator()+"list.txt");
			QDir(path).mkdir("images");
			if(fileOut.open(QIODevice::WriteOnly | QIODevice::Text))
			{
				if(fileList.open(QIODevice::WriteOnly | QIODevice::Text))
				{
					QTextStream out(&fileOut);
					QTextStream list(&fileList);
					out << "# Bundle file v0.3\n";
					out << poses.size() << " 0\n";

					for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
					{
						QString p = QString("images")+QDir::separator()+tr("%1.jpg").arg(iter->first);
						list << p << "\n";
						p = path+QDir::separator()+p;
						cv::Mat image = _cachedSignatures[iter->first].sensorData().imageRaw();
						if(image.empty())
						{
							_cachedSignatures[iter->first].sensorData().uncompressDataConst(&image, 0, 0, 0);
						}

						if(cv::imwrite(p.toStdString(), image))
						{
							UINFO("saved image %s", p.toStdString().c_str());
						}
						else
						{
							UERROR("Failed to save image %s", p.toStdString().c_str());
						}

						Transform localTransform;
						if(_cachedSignatures[iter->first].sensorData().cameraModels().size())
						{
							out << _cachedSignatures[iter->first].sensorData().cameraModels().at(0).fx() << " 0 0\n";
							localTransform = _cachedSignatures[iter->first].sensorData().cameraModels().at(0).localTransform();
						}
						else
						{
							out << _cachedSignatures[iter->first].sensorData().stereoCameraModel().left().fx() << " 0 0\n";
							localTransform = _cachedSignatures[iter->first].sensorData().stereoCameraModel().left().localTransform();
						}

						Transform rotation(0,-1,0,0,
								           0,0,1,0,
								           -1,0,0,0);

						Transform R = rotation*iter->second.rotation().inverse();

						out << R.r11() << " " << R.r12() << " " << R.r13() << "\n";
						out << R.r21() << " " << R.r22() << " " << R.r23() << "\n";
						out << R.r31() << " " << R.r32() << " " << R.r33() << "\n";

						Transform t = R * iter->second.translation();
						t.x() *= -1.0f;
						t.y() *= -1.0f;
						t.z() *= -1.0f;
						out << t.x() << " " << t.y() << " " << t.z() << "\n";
					}

					QMessageBox::question(this,
							tr("Exporting cameras in Bundler format..."),
							tr("%1 cameras/images exported to directory \"%2\".").arg(poses.size()).arg(path));
					fileList.close();
				}
				fileOut.close();
			}
		}
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
		_ui->actionExport_2D_scans_ply_pcd->setEnabled(!_createdScans.empty());
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(!_gridLocalMaps.empty() || !_projectionLocalMaps.empty());
		_ui->actionView_scans->setEnabled(!_createdScans.empty());
#ifdef RTABMAP_OCTOMAP
		_ui->actionExport_octomap->setEnabled(_octomap->octree()->size());
#else
		_ui->actionExport_octomap->setEnabled(false);
#endif
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
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
		_ui->actionDelete_memory->setEnabled(true);
		_ui->actionPost_processing->setEnabled(_cachedSignatures.size() >= 2 && _currentPosesMap.size() >= 2 && _currentLinksMap.size() >= 1);
		_ui->actionExport_images_RGB_jpg_Depth_png->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
		_ui->actionGenerate_map->setEnabled(true);
		_ui->menuExport_poses->setEnabled(!_currentPosesMap.empty());
		_ui->actionSave_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionView_high_res_point_cloud->setEnabled(!_cachedSignatures.empty() || !_cachedClouds.empty());
		_ui->actionExport_2D_scans_ply_pcd->setEnabled(!_createdScans.empty());
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(!_gridLocalMaps.empty() || !_projectionLocalMaps.empty());
		_ui->actionView_scans->setEnabled(!_createdScans.empty());
#ifdef RTABMAP_OCTOMAP
		_ui->actionExport_octomap->setEnabled(_octomap->octree()->size());
#else
		_ui->actionExport_octomap->setEnabled(false);
#endif
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
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
		_ui->actionExport_2D_scans_ply_pcd->setEnabled(false);
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(false);
		_ui->actionView_scans->setEnabled(false);
		_ui->actionExport_octomap->setEnabled(false);
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(false);
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
			_ui->actionExport_2D_scans_ply_pcd->setEnabled(false);
			_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(false);
			_ui->actionView_scans->setEnabled(false);
			_ui->actionExport_octomap->setEnabled(false);
			_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(false);
			_ui->actionDownload_all_clouds->setEnabled(false);
			_ui->actionDownload_graph->setEnabled(false);
			_state = kDetecting;
			_elapsedTime->start();
			_oneSecondTimer->start();

			if(_camera)
			{
				_camera->start();
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
			_ui->actionExport_2D_scans_ply_pcd->setEnabled(!_createdScans.empty());
			_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(!_gridLocalMaps.empty() || !_projectionLocalMaps.empty());
			_ui->actionView_scans->setEnabled(!_createdScans.empty());
#ifdef RTABMAP_OCTOMAP
			_ui->actionExport_octomap->setEnabled(_octomap->octree()->size());
#else
			_ui->actionExport_octomap->setEnabled(false);
#endif
			_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
			_ui->actionDownload_all_clouds->setEnabled(true);
			_ui->actionDownload_graph->setEnabled(true);
			_state = kPaused;
			_oneSecondTimer->stop();

			// kill sensors
			if(_camera)
			{
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
		_ui->actionExport_2D_scans_ply_pcd->setEnabled(false);
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(false);
		_ui->actionView_scans->setEnabled(false);
		_ui->actionExport_octomap->setEnabled(false);
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(false);
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
		_ui->actionExport_2D_scans_ply_pcd->setEnabled(!_createdScans.empty());
		_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(!_gridLocalMaps.empty() || !_projectionLocalMaps.empty());
		_ui->actionView_scans->setEnabled(!_createdScans.empty());
#ifdef RTABMAP_OCTOMAP
		_ui->actionExport_octomap->setEnabled(_octomap->octree()->size());
#else
		_ui->actionExport_octomap->setEnabled(false);
#endif
		_ui->actionExport_cameras_in_Bundle_format_out->setEnabled(!_cachedSignatures.empty() && !_currentPosesMap.empty());
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
