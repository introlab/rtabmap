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

#include "rtabmap/gui/ImageView.h"
#include "rtabmap/gui/KeypointItem.h"
#include "rtabmap/gui/DataRecorder.h"
#include "rtabmap/gui/DatabaseViewer.h"

#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include "utilite/UPlot.h"
#include "rtabmap/gui/UCv2Qt.h"

#include "ExportCloudsDialog.h"
#include "AboutDialog.h"
#include "PdfPlot.h"
#include "StatsToolBox.h"
#include "DetailedProgressDialog.h"
#include "PostProcessingDialog.h"

#include <QtGui/QCloseEvent>
#include <QtGui/QPixmap>
#include <QtCore/QDir>
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
#include "rtabmap/core/Graph.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>

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
	_dbReader(0),
	_odomThread(0),
	_preferencesDialog(0),
	_aboutDialog(0),
	_exportDialog(0),
	_dataRecorder(0),
	_lastId(0),
	_processingStatistics(false),
	_odometryReceived(false),
	_newDatabasePath(""),
	_newDatabasePathOutput(""),
	_openedDatabasePath(""),
	_databaseUpdated(false),
	_odomImageShow(true),
	_odomImageDepthShow(false),
	_odometryCorrection(Transform::getIdentity()),
	_processingOdometry(false),
	_lastOdomInfoUpdateTime(0),
	_oneSecondTimer(0),
	_elapsedTime(0),
	_posteriorCurve(0),
	_likelihoodCurve(0),
	_rawLikelihoodCurve(0),
	_autoScreenCaptureOdomSync(false),
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
	_exportDialog = new ExportCloudsDialog(this);
	_exportDialog->setObjectName("ExportCloudsDialog");
	_postProcessingDialog = new PostProcessingDialog(this);
	_postProcessingDialog->setObjectName("PostProcessingDialog");

	_ui = new Ui_mainWindow();
	_ui->setupUi(this);

	QString title("RTAB-Map[*]");
	this->setWindowTitle(title);
	this->setWindowIconText(tr("RTAB-Map"));
	this->setObjectName("MainWindow");

	//Setup dock widgets position if it is the first time the application is started.
	//if(!QFile::exists(PreferencesDialog::getIniFilePath()))
	{
		_ui->dockWidget_posterior->setVisible(false);
		_ui->dockWidget_likelihood->setVisible(false);
		_ui->dockWidget_rawlikelihood->setVisible(false);
		_ui->dockWidget_statsV2->setVisible(false);
		_ui->dockWidget_console->setVisible(false);
		_ui->dockWidget_loopClosureViewer->setVisible(false);
		_ui->dockWidget_mapVisibility->setVisible(false);
		_ui->dockWidget_graphViewer->setVisible(false);
		//_ui->dockWidget_odometry->setVisible(false);
		//_ui->dockWidget_cloudViewer->setVisible(false);
		//_ui->dockWidget_imageView->setVisible(false);
	}

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
	_preferencesDialog->loadMainWindowState(this, _savedMaximized);
	_preferencesDialog->loadWindowGeometry(_preferencesDialog);
	_preferencesDialog->loadWindowGeometry(_exportDialog);
	_preferencesDialog->loadWindowGeometry(_postProcessingDialog);
	_preferencesDialog->loadWindowGeometry(_aboutDialog);
	setupMainLayout(_preferencesDialog->isVerticalLayoutUsed());

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

	_ui->doubleSpinBox_stats_imgRate->setValue(_preferencesDialog->getGeneralInputRate());
	_ui->doubleSpinBox_stats_detectionRate->setValue(_preferencesDialog->getDetectionRate());
	_ui->doubleSpinBox_stats_timeLimit->setValue(_preferencesDialog->getTimeLimit());

	_initProgressDialog = new DetailedProgressDialog(this);
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
	connect(_ui->actionCancel_goal, SIGNAL(triggered()), this, SLOT(cancelGoal()));
	connect(_ui->actionClear_cache, SIGNAL(triggered()), this, SLOT(clearTheCache()));
	connect(_ui->actionAbout, SIGNAL(triggered()), _aboutDialog , SLOT(exec()));
	connect(_ui->actionPrint_loop_closure_IDs_to_console, SIGNAL(triggered()), this, SLOT(printLoopClosureIds()));
	connect(_ui->actionGenerate_map, SIGNAL(triggered()), this , SLOT(generateMap()));
	connect(_ui->actionGenerate_local_map, SIGNAL(triggered()), this, SLOT(generateLocalMap()));
	connect(_ui->actionGenerate_TORO_graph_graph, SIGNAL(triggered()), this , SLOT(generateTOROMap()));
	connect(_ui->actionExport_poses_txt, SIGNAL(triggered()), this , SLOT(exportPoses()));
	connect(_ui->actionDelete_memory, SIGNAL(triggered()), this , SLOT(deleteMemory()));
	connect(_ui->actionDownload_all_clouds, SIGNAL(triggered()), this , SLOT(downloadAllClouds()));
	connect(_ui->actionDownload_graph, SIGNAL(triggered()), this , SLOT(downloadPoseGraph()));
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
	connect(_ui->actionSave_point_cloud, SIGNAL(triggered()), this, SLOT(exportClouds()));
	connect(_ui->actionExport_2D_scans_ply_pcd, SIGNAL(triggered()), this, SLOT(exportScans()));
	connect(_ui->actionExport_2D_Grid_map_bmp_png, SIGNAL(triggered()), this, SLOT(exportGridMap()));
	connect(_ui->actionView_scans, SIGNAL(triggered()), this, SLOT(viewScans()));
	connect(_ui->actionView_high_res_point_cloud, SIGNAL(triggered()), this, SLOT(viewClouds()));
	connect(_ui->actionReset_Odometry, SIGNAL(triggered()), this, SLOT(resetOdometry()));
	connect(_ui->actionTrigger_a_new_map, SIGNAL(triggered()), this, SLOT(triggerNewMap()));
	connect(_ui->actionData_recorder, SIGNAL(triggered()), this, SLOT(dataRecorder()));
	connect(_ui->actionPost_processing, SIGNAL(triggered()), this, SLOT(postProcessing()));

	_ui->actionPause->setShortcut(Qt::Key_Space);
	_ui->actionSave_GUI_config->setShortcut(QKeySequence::Save);
	_ui->actionSave_point_cloud->setEnabled(false);
	_ui->actionExport_2D_scans_ply_pcd->setEnabled(false);
	_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(false);
	_ui->actionView_scans->setEnabled(false);
	_ui->actionView_high_res_point_cloud->setEnabled(false);
	_ui->actionReset_Odometry->setEnabled(false);
	_ui->actionPost_processing->setEnabled(false);

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
	_ui->actionFreenect->setEnabled(CameraFreenect::available());
	_ui->actionOpenNI_CV->setEnabled(CameraOpenNICV::available());
	_ui->actionOpenNI_CV_ASUS->setEnabled(CameraOpenNICV::available());
	_ui->actionOpenNI2->setEnabled(CameraOpenNI2::available());
	_ui->actionOpenNI2_kinect->setEnabled(CameraOpenNI2::available());
	_ui->actionOpenNI2_sense->setEnabled(CameraOpenNI2::available());
	_ui->actionFreenect2->setEnabled(CameraFreenect2::available());
	_ui->actionStereoDC1394->setEnabled(CameraStereoDC1394::available());
	_ui->actionStereoFlyCapture2->setEnabled(CameraStereoFlyCapture2::available());
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
	connect(_ui->widget_cloudViewer, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_exportDialog, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_postProcessingDialog, SIGNAL(configChanged()), this, SLOT(configGUIModified()));
	connect(_ui->toolBar->toggleViewAction(), SIGNAL(toggled(bool)), this, SLOT(configGUIModified()));
	connect(_ui->toolBar, SIGNAL(orientationChanged(Qt::Orientation)), this, SLOT(configGUIModified()));
	QList<QDockWidget*> dockWidgets = this->findChildren<QDockWidget*>();
	for(int i=0; i<dockWidgets.size(); ++i)
	{
		connect(dockWidgets[i], SIGNAL(dockLocationChanged(Qt::DockWidgetArea)), this, SLOT(configGUIModified()));
		connect(dockWidgets[i]->toggleViewAction(), SIGNAL(toggled(bool)), this, SLOT(configGUIModified()));
	}
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
	connect(_ui->doubleSpinBox_stats_imgRate, SIGNAL(editingFinished()), this, SLOT(changeImgRateSetting()));
	connect(_ui->doubleSpinBox_stats_detectionRate, SIGNAL(editingFinished()), this, SLOT(changeDetectionRateSetting()));
	connect(_ui->doubleSpinBox_stats_timeLimit, SIGNAL(editingFinished()), this, SLOT(changeTimeLimitSetting()));
	connect(this, SIGNAL(imgRateChanged(double)), _preferencesDialog, SLOT(setInputRate(double)));
	connect(this, SIGNAL(detectionRateChanged(double)), _preferencesDialog, SLOT(setDetectionRate(double)));
	connect(this, SIGNAL(timeLimitChanged(float)), _preferencesDialog, SLOT(setTimeLimit(float)));

	// Statistics from the detector
	qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");
	connect(this, SIGNAL(statsReceived(rtabmap::Statistics)), this, SLOT(processStats(rtabmap::Statistics)));

	qRegisterMetaType<rtabmap::OdometryEvent>("rtabmap::OdometryEvent");
	connect(this, SIGNAL(odometryReceived(rtabmap::OdometryEvent)), this, SLOT(processOdometry(rtabmap::OdometryEvent)));

	connect(this, SIGNAL(noMoreImagesReceived()), this, SLOT(notifyNoMoreImages()));

	// Apply state
	this->changeState(kIdle);
	this->applyPrefSettings(PreferencesDialog::kPanelAll);

	_ui->statsToolBox->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_ui->graphicsView_graphView->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_ui->widget_cloudViewer->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_preferencesDialog->loadWidgetState(_ui->widget_cloudViewer);

	//dialog states
	_preferencesDialog->loadWidgetState(_exportDialog);
	_preferencesDialog->loadWidgetState(_postProcessingDialog);

	if(_ui->statsToolBox->findChildren<StatItem*>().size() == 0)
	{
		const std::map<std::string, float> & statistics = Statistics::defaultData();
		for(std::map<std::string, float>::const_iterator iter = statistics.begin(); iter != statistics.end(); ++iter)
		{
			_ui->statsToolBox->updateStat(QString((*iter).first.c_str()).replace('_', ' '), 0, (*iter).second);
		}
	}
	this->loadFigures();
	connect(_ui->statsToolBox, SIGNAL(figuresSetupChanged()), this, SLOT(configGUIModified()));

	// update loop closure viewer parameters
	ParametersMap parameters = _preferencesDialog->getAllParameters();
	_ui->widget_loopClosureViewer->setDecimation(atoi(parameters.at(Parameters::kLccIcp3Decimation()).c_str()));
	_ui->widget_loopClosureViewer->setMaxDepth(uStr2Float(parameters.at(Parameters::kLccIcp3MaxDepth())));
	_ui->widget_loopClosureViewer->setSamples(atoi(parameters.at(Parameters::kLccIcp3Samples()).c_str()));

	//update ui
	_ui->doubleSpinBox_stats_detectionRate->setValue(_preferencesDialog->getDetectionRate());
	_ui->doubleSpinBox_stats_timeLimit->setValue(_preferencesDialog->getTimeLimit());
	_ui->actionSLAM_mode->setChecked(_preferencesDialog->isSLAMMode());

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
		if(_dbReader)
		{
			UERROR("DBReader must be already deleted here!");
			delete _dbReader;
			_dbReader = 0;
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
		int localLoopClosureId = int(uValue(stats.data(), Statistics::kLocalLoopSpace_last_closure_id(), 0.0f));
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
		   (localLoopClosureId > 0 &&
		    _ui->actionPause_on_local_loop_detection->isChecked()))
		{
			if(_state != kPaused && _state != kMonitoringPaused)
			{
				if(_preferencesDialog->beepOnPause())
				{
					QMetaObject::invokeMethod(this, "beep");
				}
				this->pauseDetection();
			}
		}
		_processingStatistics = true;
		emit statsReceived(stats);
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
	}
	else if(anEvent->getClassName().compare("OdometryEvent") == 0)
	{
		// limit 10 Hz max
		if(UTimer::now() - _lastOdomInfoUpdateTime > 0.1)
		{
			_lastOdomInfoUpdateTime = UTimer::now();
			OdometryEvent * odomEvent = (OdometryEvent*)anEvent;
			if(!_processingOdometry && !_processingStatistics)
			{
				_processingOdometry = true; // if we receive too many odometry events!
				emit odometryReceived(*odomEvent);
			}
			else
			{
				// we receive too many odometry events! just send without data
				OdometryEvent tmp(SensorData(cv::Mat(), odomEvent->data().id()), odomEvent->pose(), odomEvent->covariance(), odomEvent->info());
				emit odometryReceived(tmp);
			}
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

void MainWindow::processOdometry(const rtabmap::OdometryEvent & odom)
{
	_processingOdometry = true;
	UTimer time;
	// Process Data
	if(!odom.data().imageRaw().empty())
	{
		Transform pose = odom.pose();
		bool lost = false;
		bool lostStateChanged = false;

		if(pose.isNull())
		{
			UDEBUG("odom lost"); // use last pose
			lostStateChanged = _ui->widget_cloudViewer->getBackgroundColor() != Qt::darkRed;
			_ui->widget_cloudViewer->setBackgroundColor(Qt::darkRed);
			_ui->imageView_odometry->setBackgroundColor(Qt::darkRed);

			pose = _lastOdomPose;
			lost = true;
		}
		else if(odom.info().inliers>0 &&
				_preferencesDialog->getOdomQualityWarnThr() &&
				odom.info().inliers < _preferencesDialog->getOdomQualityWarnThr())
		{
			UDEBUG("odom warn, quality(inliers)=%d thr=%d", odom.info().inliers, _preferencesDialog->getOdomQualityWarnThr());
			lostStateChanged = _ui->widget_cloudViewer->getBackgroundColor() == Qt::darkRed;
			_ui->widget_cloudViewer->setBackgroundColor(Qt::darkYellow);
			_ui->imageView_odometry->setBackgroundColor(Qt::darkYellow);
		}
		else
		{
			UDEBUG("odom ok");
			lostStateChanged = _ui->widget_cloudViewer->getBackgroundColor() == Qt::darkRed;
			_ui->widget_cloudViewer->setBackgroundColor(_ui->widget_cloudViewer->getDefaultBackgroundColor());
			_ui->imageView_odometry->setBackgroundColor(Qt::black);
		}

		if(!pose.isNull() && (_ui->dockWidget_cloudViewer->isVisible() || _ui->graphicsView_graphView->isVisible()))
		{
			_lastOdomPose = pose;
			_odometryReceived = true;
		}

		if(_ui->dockWidget_cloudViewer->isVisible())
		{
			if(!pose.isNull())
			{
				// 3d cloud
				if(odom.data().depthOrRightRaw().cols == odom.data().imageRaw().cols &&
				   odom.data().depthOrRightRaw().rows == odom.data().imageRaw().rows &&
				   !odom.data().depthOrRightRaw().empty() &&
				   (odom.data().cameraModels().size() || odom.data().stereoCameraModel().isValid()) &&
				   _preferencesDialog->isCloudsShown(1))
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
					cloud = util3d::cloudRGBFromSensorData(odom.data(),
							_preferencesDialog->getCloudDecimation(1),
							_preferencesDialog->getCloudMaxDepth(1),
							_preferencesDialog->getCloudVoxelSize(1));
					if(cloud->size())
					{
						cloud = util3d::transformPointCloud(cloud, pose);

						if(!_ui->widget_cloudViewer->addOrUpdateCloud("cloudOdom", cloud, _odometryCorrection))
						{
							UERROR("Adding cloudOdom to viewer failed!");
						}
						_ui->widget_cloudViewer->setCloudVisibility("cloudOdom", true);
						_ui->widget_cloudViewer->setCloudOpacity("cloudOdom", _preferencesDialog->getCloudOpacity(1));
						_ui->widget_cloudViewer->setCloudPointSize("cloudOdom", _preferencesDialog->getCloudPointSize(1));
					}
					else
					{
						UWARN("Empty cloudOdom!");
						_ui->widget_cloudViewer->setCloudVisibility("cloudOdom", false);
					}
				}

				// 2d cloud
				if(!odom.data().laserScanRaw().empty() &&
					_preferencesDialog->isScansShown(1))
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
					cloud = util3d::laserScanToPointCloud(odom.data().laserScanRaw());
					cloud = util3d::transformPointCloud(cloud, pose);
					if(!_ui->widget_cloudViewer->addOrUpdateCloud("scanOdom", cloud, _odometryCorrection))
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
						cloud = util3d::laserScanToPointCloud(odom.data().laserScanRaw());
						cloud = util3d::transformPointCloud(cloud, pose);
						if(!_ui->widget_cloudViewer->addOrUpdateCloud("scanOdom", cloud, _odometryCorrection))
						{
							UERROR("Adding scanOdom to viewer failed!");
						}
						_ui->widget_cloudViewer->setCloudVisibility("scanOdom", true);
						_ui->widget_cloudViewer->setCloudOpacity("scanOdom", _preferencesDialog->getScanOpacity(1));
						_ui->widget_cloudViewer->setCloudPointSize("scanOdom", _preferencesDialog->getScanPointSize(1));
					}
				}
			}
		}

		if(!odom.pose().isNull())
		{
			// update camera position
			_ui->widget_cloudViewer->updateCameraTargetPosition(_odometryCorrection*odom.pose());
		}
		_ui->widget_cloudViewer->update();

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

			_ui->imageView_odometry->clearLines();
			if(lost)
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
			this->captureScreen();
		}
	}

	//Process info
	if(odom.info().inliers >= 0)
	{
		_ui->statsToolBox->updateStat("Odometry/Inliers/", (float)odom.data().id(), (float)odom.info().inliers);
	}
	if(odom.info().matches >= 0)
	{
		_ui->statsToolBox->updateStat("Odometry/Matches/", (float)odom.data().id(), (float)odom.info().matches);
	}
	if(odom.info().variance >= 0)
	{
		_ui->statsToolBox->updateStat("Odometry/StdDev/", (float)odom.data().id(), sqrt((float)odom.info().variance));
	}
	if(odom.info().variance >= 0)
	{
		_ui->statsToolBox->updateStat("Odometry/Variance/", (float)odom.data().id(), (float)odom.info().variance);
	}
	if(odom.info().timeEstimation > 0)
	{
		_ui->statsToolBox->updateStat("Odometry/TimeEstimation/ms", (float)odom.data().id(), (float)odom.info().timeEstimation*1000.0f);
	}
	if(odom.info().timeParticleFiltering > 0)
	{
		_ui->statsToolBox->updateStat("Odometry/TimeFiltering/ms", (float)odom.data().id(), (float)odom.info().timeParticleFiltering*1000.0f);
	}
	if(odom.info().features >=0)
	{
		_ui->statsToolBox->updateStat("Odometry/Features/", (float)odom.data().id(), (float)odom.info().features);
	}
	if(odom.info().localMapSize >=0)
	{
		_ui->statsToolBox->updateStat("Odometry/Local_map_size/", (float)odom.data().id(), (float)odom.info().localMapSize);
	}
	_ui->statsToolBox->updateStat("Odometry/ID/", (float)odom.data().id(), (float)odom.data().id());

	float x=0.0f,y,z, roll,pitch,yaw;
	if(!odom.info().transform.isNull())
	{
		odom.info().transform.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		_ui->statsToolBox->updateStat("Odometry/Tx/m", (float)odom.data().id(), x);
		_ui->statsToolBox->updateStat("Odometry/Ty/m", (float)odom.data().id(), y);
		_ui->statsToolBox->updateStat("Odometry/Tz/m", (float)odom.data().id(), z);
		_ui->statsToolBox->updateStat("Odometry/Troll/deg", (float)odom.data().id(), roll*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/Tpitch/deg", (float)odom.data().id(), pitch*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/Tyaw/deg", (float)odom.data().id(), yaw*180.0/CV_PI);
	}

	if(!odom.info().transformFiltered.isNull())
	{
		odom.info().transformFiltered.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		_ui->statsToolBox->updateStat("Odometry/Fx/m", (float)odom.data().id(), x);
		_ui->statsToolBox->updateStat("Odometry/Fy/m", (float)odom.data().id(), y);
		_ui->statsToolBox->updateStat("Odometry/Fz/m", (float)odom.data().id(), z);
		_ui->statsToolBox->updateStat("Odometry/Froll/deg", (float)odom.data().id(), roll*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/Fpitch/deg", (float)odom.data().id(), pitch*180.0/CV_PI);
		_ui->statsToolBox->updateStat("Odometry/Fyaw/deg", (float)odom.data().id(), yaw*180.0/CV_PI);
	}

	if(odom.info().interval > 0)
	{
		_ui->statsToolBox->updateStat("Odometry/Interval/ms", (float)odom.data().id(), odom.info().interval*1000.f);
		_ui->statsToolBox->updateStat("Odometry/Speed/kph", (float)odom.data().id(), x/odom.info().interval*3.6f);
	}
	if(odom.info().distanceTravelled > 0)
	{
		_ui->statsToolBox->updateStat("Odometry/Distance/m", (float)odom.data().id(), odom.info().distanceTravelled);
	}

	_ui->statsToolBox->updateStat("/Gui refresh odom/ms", (float)odom.data().id(), time.elapsed()*1000.0);
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

	int refMapId = -1, loopMapId = -1;
	if(uContains(stat.getSignatures(), stat.refImageId()))
	{
		refMapId = stat.getSignatures().at(stat.refImageId()).mapId();
	}
	int highestHypothesisId = static_cast<float>(uValue(stat.data(), Statistics::kLoopHighest_hypothesis_id(), 0.0f));
	int loopId = stat.loopClosureId()>0?stat.loopClosureId():stat.localLoopClosureId()>0?stat.localLoopClosureId():highestHypothesisId;
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

		// update cache
		Signature signature;
		if(uContains(stat.getSignatures(), stat.refImageId()))
		{
			signature = stat.getSignatures().at(stat.refImageId());
			signature.sensorData().uncompressData(); // make sure data are uncompressed
			_cachedSignatures.insert(signature.id(), signature);
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

		int rehearsed = (int)uValue(stat.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
		int localTimeClosures = (int)uValue(stat.data(), Statistics::kLocalLoopTime_closures(), 0.0f);
		bool scanMatchingSuccess = (bool)uValue(stat.data(), Statistics::kOdomCorrectionAccepted(), 0.0f);
		_ui->label_stats_imageNumber->setText(QString("%1 [%2]").arg(stat.refImageId()).arg(refMapId));

		if(rehearsed > 0)
		{
			_ui->imageView_source->setBackgroundColor(Qt::blue);
		}
		else if(localTimeClosures > 0)
		{
			_ui->imageView_source->setBackgroundColor(Qt::darkCyan);
		}
		else if(scanMatchingSuccess)
		{
			_ui->imageView_source->setBackgroundColor(Qt::gray);
		}

		UDEBUG("time= %d ms", time.restart());

		int rejectedHyp = bool(uValue(stat.data(), Statistics::kLoopRejectedHypothesis(), 0.0f));
		float highestHypothesisValue = uValue(stat.data(), Statistics::kLoopHighest_hypothesis_value(), 0.0f);
		int matchId = 0;
		Signature loopSignature;
		int shownLoopId = 0;
		if(highestHypothesisId > 0 || stat.localLoopClosureId()>0)
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
			else if(stat.localLoopClosureId())
			{
				_ui->imageView_loopClosure->setBackgroundColor(Qt::yellow);
				_ui->label_matchId->setText(QString("Local match = %1 [%2]").arg(stat.localLoopClosureId()).arg(loopMapId));
				matchId = stat.localLoopClosureId();
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
				shownLoopId = stat.loopClosureId()>0?stat.loopClosureId():stat.localLoopClosureId()>0?stat.localLoopClosureId():highestHypothesisId;
				QMap<int, Signature>::iterator iter = _cachedSignatures.find(shownLoopId);
				if(iter != _cachedSignatures.end())
				{
					iter.value().sensorData().uncompressData();
					loopSignature = iter.value();
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

		_ui->statsToolBox->updateStat("Keypoint/Keypoints count in the last signature/", stat.refImageId(), signature.getWords().size());
		_ui->statsToolBox->updateStat("Keypoint/Keypoints count in the loop signature/", stat.refImageId(), loopSignature.getWords().size());

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
			_ui->statsToolBox->updateStat(QString((*iter).first.c_str()).replace('_', ' '), stat.refImageId(), (*iter).second);
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
			for(std::map<int, Signature>::const_iterator iter=stat.getSignatures().begin(); iter!=stat.getSignatures().end();++iter)
			{
				mapIds.insert(std::make_pair(iter->first, iter->second.mapId()));
			}
			updateMapCloud(stat.poses(),
					_odometryReceived||stat.poses().size()==0?Transform():stat.poses().rbegin()->second,
					stat.constraints(),
					mapIds);

			_odometryReceived = false;

			_odometryCorrection = stat.mapCorrection();

			UDEBUG("time= %d ms", time.restart());
			_ui->statsToolBox->updateStat("/Gui RGB-D cloud/ms", stat.refImageId(), int(timerVis.elapsed()*1000.0f));

			// loop closure view
			if((stat.loopClosureId() > 0 || stat.localLoopClosureId() > 0)  &&
			   !stat.loopClosureTransform().isNull() &&
			   !loopSignature.sensorData().imageRaw().empty())
			{
				// the last loop closure data
				Transform loopClosureTransform = stat.loopClosureTransform();
				signature.setPose(loopClosureTransform);
				_ui->widget_loopClosureViewer->setData(loopSignature, signature);
				if(_ui->dockWidget_loopClosureViewer->isVisible())
				{
					UTimer loopTimer;
					_ui->widget_loopClosureViewer->updateView();
					UINFO("Updating loop closure cloud view time=%fs", loopTimer.elapsed());
					_ui->statsToolBox->updateStat("/Gui RGB-D closure view/ms", stat.refImageId(), int(loopTimer.elapsed()*1000.0f));
				}

				UDEBUG("time= %d ms", time.restart());
			}
		}

		// update posterior on the graph view
		if(_preferencesDialog->isPosteriorGraphView() && _ui->graphicsView_graphView->isVisible() && stat.posterior().size())
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
			_ui->graphicsView_graphView->setCurrentGoalID(stat.currentGoalId());
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
	_ui->statsToolBox->updateStat("/Gui refresh stats/ms", stat.refImageId(), elapsedTime);
	if(_ui->actionAuto_screen_capture->isChecked() && !_autoScreenCaptureOdomSync)
	{
		this->captureScreen();
	}

	if(!_preferencesDialog->isImagesKept())
	{
		_cachedSignatures.clear();
	}

	_processingStatistics = false;
}

void MainWindow::updateMapCloud(
		const std::map<int, Transform> & posesIn,
		const Transform & currentPose,
		const std::multimap<int, Link> & constraints,
		const std::map<int, int> & mapIdsIn,
		bool verboseProgress)
{
	UDEBUG("posesIn=%d constraints=%d mapIdsIn=%d currentPose=%s",
			(int)posesIn.size(), (int)constraints.size(), (int)mapIdsIn.size(), currentPose.prettyPrint().c_str());
	if(posesIn.size())
	{
		_currentPosesMap = posesIn;
		_currentLinksMap = constraints;
		_currentMapIds = mapIdsIn;
		if(_currentPosesMap.size())
		{
			if(!_ui->actionSave_point_cloud->isEnabled() &&
				_cachedSignatures.size() &&
				(!(--_cachedSignatures.end())->sensorData().depthOrRightCompressed().empty() ||
				 !(--_cachedSignatures.end())->getWords3().empty()))
			{
				//enable save cloud action
				_ui->actionSave_point_cloud->setEnabled(true);
				_ui->actionView_high_res_point_cloud->setEnabled(true);
			}

			if(!_ui->actionView_scans->isEnabled() &&
				_cachedSignatures.size() &&
				!(--_cachedSignatures.end())->sensorData().laserScanCompressed().empty())
			{
				_ui->actionExport_2D_scans_ply_pcd->setEnabled(true);
				_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(true);
				_ui->actionView_scans->setEnabled(true);
			}
			else if(_preferencesDialog->isGridMapFrom3DCloud() && _projectionLocalMaps.size())
			{
				_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(true);
			}
		}
		if(_state != kMonitoring && _state != kDetecting)
		{
			_ui->actionPost_processing->setEnabled(_cachedSignatures.size() >= 2 && _currentPosesMap.size() >= 2 && _currentLinksMap.size() >= 1);
		}
	}

	// filter duplicated poses
	std::map<int, Transform> poses;
	std::map<int, int> mapIds;
	if(_preferencesDialog->isCloudFiltering() && posesIn.size())
	{
		float radius = _preferencesDialog->getCloudFilteringRadius();
		float angle = _preferencesDialog->getCloudFilteringAngle()*CV_PI/180.0; // convert to rad
		poses = rtabmap::graph::radiusPosesFiltering(posesIn, radius, angle);
		// make sure the last is here
		poses.insert(*posesIn.rbegin());
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

	// Map updated! regenerate the assembled cloud, last pose is the new one
	UDEBUG("Update map with %d locations (currentPose=%s)", poses.size(), currentPose.prettyPrint().c_str());
	QMap<std::string, Transform> viewerClouds = _ui->widget_cloudViewer->getAddedClouds();
	int i=1;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(!iter->second.isNull())
		{
			std::string cloudName = uFormat("cloud%d", iter->first);

			// 3d point cloud
			if((_ui->widget_cloudViewer->isVisible() && _preferencesDialog->isCloudsShown(0)) ||
				(_ui->graphicsView_graphView->isVisible() && _ui->graphicsView_graphView->isGridMapVisible() && _preferencesDialog->isGridMapFrom3DCloud()))
			{
				if(viewerClouds.contains(cloudName))
				{
					// Update only if the pose has changed
					Transform tCloud;
					_ui->widget_cloudViewer->getPose(cloudName, tCloud);
					if(tCloud.isNull() || iter->second != tCloud)
					{
						if(!_ui->widget_cloudViewer->updateCloudPose(cloudName, iter->second))
						{
							UERROR("Updating pose cloud %d failed!", iter->first);
						}
					}
					_ui->widget_cloudViewer->setCloudVisibility(cloudName, true);
					_ui->widget_cloudViewer->setCloudOpacity(cloudName, _preferencesDialog->getCloudOpacity(0));
					_ui->widget_cloudViewer->setCloudPointSize(cloudName, _preferencesDialog->getCloudPointSize(0));
				}
				else if(_cachedSignatures.contains(iter->first))
				{
					QMap<int, Signature>::iterator jter = _cachedSignatures.find(iter->first);
					if((!jter->sensorData().imageCompressed().empty() && !jter->sensorData().depthOrRightCompressed().empty()) || jter->getWords3().size())
					{
						this->createAndAddCloudToMap(iter->first, iter->second, uValue(mapIds, iter->first, -1));
					}
				}
			}
			else if(viewerClouds.contains(cloudName))
			{
				UDEBUG("Hide cloud %s", cloudName.c_str());
				_ui->widget_cloudViewer->setCloudVisibility(cloudName.c_str(), false);
			}

			// 2d point cloud
			std::string scanName = uFormat("scan%d", iter->first);
			if((_ui->widget_cloudViewer->isVisible() && (_preferencesDialog->isScansShown(0) || _preferencesDialog->getGridMapShown())) ||
				(_ui->graphicsView_graphView->isVisible() && _ui->graphicsView_graphView->isGridMapVisible()))
			{
				if(viewerClouds.contains(scanName))
				{
					// Update only if the pose has changed
					Transform tScan;
					_ui->widget_cloudViewer->getPose(scanName, tScan);
					if(tScan.isNull() || iter->second != tScan)
					{
						if(!_ui->widget_cloudViewer->updateCloudPose(scanName, iter->second))
						{
							UERROR("Updating pose scan %d failed!", iter->first);
						}
					}
					_ui->widget_cloudViewer->setCloudVisibility(scanName, true);
					_ui->widget_cloudViewer->setCloudOpacity(scanName, _preferencesDialog->getScanOpacity(0));
					_ui->widget_cloudViewer->setCloudPointSize(scanName, _preferencesDialog->getScanPointSize(0));
				}
				else if(_cachedSignatures.contains(iter->first))
				{
					QMap<int, Signature>::iterator jter = _cachedSignatures.find(iter->first);
					if(!jter->sensorData().laserScanCompressed().empty())
					{
						this->createAndAddScanToMap(iter->first, iter->second, uValue(mapIds, iter->first, -1));
					}
				}
				if(!_preferencesDialog->isScansShown(0))
				{
					UDEBUG("Hide scan %s", scanName.c_str());
					_ui->widget_cloudViewer->setCloudVisibility(scanName.c_str(), false);
				}
			}
			else if(viewerClouds.contains(scanName))
			{
				UDEBUG("Hide scan %s", scanName.c_str());
				_ui->widget_cloudViewer->setCloudVisibility(scanName.c_str(), false);
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

	//remove not used clouds
	for(QMap<std::string, Transform>::iterator iter = viewerClouds.begin(); iter!=viewerClouds.end(); ++iter)
	{
		std::list<std::string> splitted = uSplitNumChar(iter.key());
		if(splitted.size() == 2)
		{
			int id = std::atoi(splitted.back().c_str());
			if(poses.find(id) == poses.end())
			{
				if(_ui->widget_cloudViewer->getCloudVisibility(iter.key()))
				{
					UDEBUG("Hide %s", iter.key().c_str());
					_ui->widget_cloudViewer->setCloudVisibility(iter.key(), false);
				}
			}
		}
	}

	// update 3D graphes (show all poses)
	_ui->widget_cloudViewer->removeAllGraphs();
	_ui->widget_cloudViewer->removeCloud("graph_nodes");
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

		// add graphs
		for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter=graphs.begin(); iter!=graphs.end(); ++iter)
		{
			QColor color = Qt::gray;
			if(iter->first >= 0)
			{
				color = (Qt::GlobalColor)((iter->first+3) % 12 + 7 );
			}
			_ui->widget_cloudViewer->addOrUpdateGraph(uFormat("graph_%d", iter->first), iter->second, color);
		}
	}

	// Update occupancy grid map in 3D map view and graph view
	if(_ui->graphicsView_graphView->isVisible())
	{
		_ui->graphicsView_graphView->updateGraph(posesIn, constraints, mapIdsIn);
		if(!currentPose.isNull())
		{
			_ui->graphicsView_graphView->updateReferentialPosition(currentPose);
		}
	}
	cv::Mat map8U;
	if((_ui->graphicsView_graphView->isVisible() || _preferencesDialog->getGridMapShown()) && (_createdScans.size() || _preferencesDialog->isGridMapFrom3DCloud()))
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
				_ui->widget_cloudViewer->addOccupancyGridMap(map8U, resolution, xMin, yMin, opacity);
			}
			if(_ui->graphicsView_graphView->isVisible())
			{
				_ui->graphicsView_graphView->updateMap(map8U, resolution, xMin, yMin);
			}
		}
	}
	_ui->graphicsView_graphView->update();

	if(!_preferencesDialog->getGridMapShown())
	{
		_ui->widget_cloudViewer->removeOccupancyGridMap();
	}

	if(viewerClouds.contains("cloudOdom"))
	{
		if(!_preferencesDialog->isCloudsShown(1))
		{
			_ui->widget_cloudViewer->setCloudVisibility("cloudOdom", false);
		}
		else
		{
			_ui->widget_cloudViewer->updateCloudPose("cloudOdom", _odometryCorrection);
			_ui->widget_cloudViewer->setCloudOpacity("cloudOdom", _preferencesDialog->getCloudOpacity(1));
			_ui->widget_cloudViewer->setCloudPointSize("cloudOdom", _preferencesDialog->getCloudPointSize(1));
		}
	}
	if(viewerClouds.contains("scanOdom"))
	{
		if(!_preferencesDialog->isScansShown(1))
		{
			_ui->widget_cloudViewer->setCloudVisibility("scanOdom", false);
		}
		else
		{
			_ui->widget_cloudViewer->updateCloudPose("scanOdom", _odometryCorrection);
			_ui->widget_cloudViewer->setCloudOpacity("scanOdom", _preferencesDialog->getScanOpacity(1));
			_ui->widget_cloudViewer->setCloudPointSize("scanOdom", _preferencesDialog->getScanPointSize(1));
		}
	}

	if(!currentPose.isNull())
	{
		_ui->widget_cloudViewer->updateCameraTargetPosition(currentPose);
	}

	_ui->widget_cloudViewer->update();
}

void MainWindow::createAndAddCloudToMap(int nodeId, const Transform & pose, int mapId)
{
	UASSERT(!pose.isNull());
	std::string cloudName = uFormat("cloud%d", nodeId);
	if(_ui->widget_cloudViewer->getAddedClouds().contains(cloudName))
	{
		UERROR("Cloud %d already added to map.", nodeId);
		return;
	}

	QMap<int, Signature>::iterator iter = _cachedSignatures.find(nodeId);
	if(iter == _cachedSignatures.end())
	{
		UERROR("Node %d is not in the cache.", nodeId);
		return;
	}

	if(!iter->sensorData().imageCompressed().empty() && !iter->sensorData().depthOrRightCompressed().empty())
	{

		cv::Mat image, depth;
		SensorData data = iter->sensorData();
		data.uncompressData(&image, &depth, 0);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		UASSERT(nodeId == data.id());
		cloud = util3d::cloudRGBFromSensorData(data,
				_preferencesDialog->getCloudDecimation(0),
				_preferencesDialog->getCloudMaxDepth(0),
				_preferencesDialog->getCloudVoxelSize(0));
		_createdClouds.insert(std::make_pair(nodeId, cloud));

		if(cloud->size() && _preferencesDialog->isGridMapFrom3DCloud())
		{
			UTimer timer;
			float cellSize = _preferencesDialog->getGridMapResolution();
			float groundNormalMaxAngle = M_PI_4;
			int minClusterSize = 20;
			cv::Mat ground, obstacles;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelizedCloud = cloud;
			if(voxelizedCloud->size() && cellSize > _preferencesDialog->getCloudVoxelSize(0))
			{
				voxelizedCloud = util3d::voxelize(cloud, cellSize);
			}
			util3d::occupancy2DFromCloud3D<pcl::PointXYZRGB>(
					voxelizedCloud,
					ground, obstacles,
					cellSize,
					groundNormalMaxAngle,
					minClusterSize);
			if(!ground.empty() || !obstacles.empty())
			{
				_projectionLocalMaps.insert(std::make_pair(nodeId, std::make_pair(ground, obstacles)));
			}
			UDEBUG("time gridMapFrom2DCloud = %f s", timer.ticks());
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered = cloud;
		if(_preferencesDialog->isSubtractFiltering() &&
			_preferencesDialog->getCloudVoxelSize(0) > 0.0 &&
			cloud->size() &&
			_createdClouds.size() &&
			_currentPosesMap.size() &&
			_currentLinksMap.size())
		{
			// find link to previous neighbor
			std::map<int, Transform>::const_iterator previousIter = _currentPosesMap.find(nodeId);
			Link link;
			if(previousIter != _currentPosesMap.begin())
			{
				--previousIter;
				std::multimap<int, Link>::const_iterator linkIter = graph::findLink(_currentLinksMap, nodeId, previousIter->first);
				if(linkIter != _currentLinksMap.end())
				{
					link = linkIter->second;
					if(link.from() != nodeId)
					{
						link = link.inverse();
					}
				}
			}
			if(link.isValid())
			{
				std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator iter = _createdClouds.find(link.to());
				if(iter!=_createdClouds.end() && iter->second->size())
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr previousCloud = util3d::transformPointCloud(iter->second, link.transform());
					cloudFiltered = util3d::subtractFiltering(
							cloud,
							previousCloud,
							_preferencesDialog->getCloudVoxelSize(0),
							_preferencesDialog->getSubstractFilteringMinPts());
					UDEBUG("Filtering %d from %d -> %d", (int)previousCloud->size(), (int)cloud->size(), (int)cloudFiltered->size());

				}
			}
		}

		if(_preferencesDialog->isCloudMeshing())
		{
			pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
			if(cloudFiltered->size())
			{
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals;
				if(_preferencesDialog->getMeshSmoothing())
				{
					cloudWithNormals = util3d::computeNormalsSmoothed(cloudFiltered, (float)_preferencesDialog->getMeshSmoothingRadius());
				}
				else
				{
					cloudWithNormals = util3d::computeNormals(cloudFiltered, _preferencesDialog->getMeshNormalKSearch());
				}
				mesh = util3d::createMesh(cloudWithNormals,	_preferencesDialog->getMeshGP3Radius());
			}

			if(mesh->polygons.size())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::fromPCLPointCloud2(mesh->cloud, *tmp);
				if(!_ui->widget_cloudViewer->addCloudMesh(cloudName, tmp, mesh->polygons, pose))
				{
					UERROR("Adding mesh cloud %d to viewer failed!", nodeId);
				}
			}
		}
		else
		{
			if(_preferencesDialog->getMeshSmoothing())
			{
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals;
				cloudWithNormals = util3d::computeNormalsSmoothed(cloudFiltered, (float)_preferencesDialog->getMeshSmoothingRadius());
				cloudFiltered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::copyPointCloud(*cloudWithNormals, *cloudFiltered);
			}
			QColor color = Qt::gray;
			if(mapId >= 0)
			{
				color = (Qt::GlobalColor)(mapId+3 % 12 + 7 );
			}
			if(!_ui->widget_cloudViewer->addOrUpdateCloud(cloudName, cloudFiltered, pose, color))
			{
				UERROR("Adding cloud %d to viewer failed!", nodeId);
			}
		}
	}
	else if(iter->getWords3().size())
	{
		QColor color = Qt::gray;
		if(mapId >= 0)
		{
			color = (Qt::GlobalColor)(mapId+3 % 12 + 7 );
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud->resize(iter->getWords3().size());
		int oi=0;
		UASSERT(iter->getWords().size() == iter->getWords3().size());
		std::multimap<int, cv::KeyPoint>::const_iterator kter=iter->getWords().begin();
		for(std::multimap<int, pcl::PointXYZ>::const_iterator jter=iter->getWords3().begin();
				jter!=iter->getWords3().end(); ++jter, ++kter, ++oi)
		{
			(*cloud)[oi].x = jter->second.x;
			(*cloud)[oi].y = jter->second.y;
			(*cloud)[oi].z = jter->second.z;
			int u = kter->second.pt.x+0.5;
			int v = kter->second.pt.x+0.5;
			if(!iter->sensorData().imageRaw().empty() &&
				uIsInBounds(u, 0, iter->sensorData().imageRaw().cols-1) &&
				uIsInBounds(v, 0, iter->sensorData().imageRaw().rows-1))
			{
				if(iter->sensorData().imageRaw().channels() == 1)
				{
					(*cloud)[oi].r = (*cloud)[oi].g = (*cloud)[oi].b = iter->sensorData().imageRaw().at<unsigned char>(u, v);
				}
				else
				{
					cv::Vec3b bgr = iter->sensorData().imageRaw().at<cv::Vec3b>(u, v);
					(*cloud)[oi].r = bgr.val[0];
					(*cloud)[oi].g = bgr.val[1];
					(*cloud)[oi].b = bgr.val[2];
				}
			}
			else
			{
				(*cloud)[oi].r = (*cloud)[oi].g = (*cloud)[oi].b = 255;
			}
		}
		if(!_ui->widget_cloudViewer->addOrUpdateCloud(cloudName, cloud, pose, color))
		{
			UERROR("Adding cloud %d to viewer failed!", nodeId);
		}
		else
		{
			_createdClouds.insert(std::make_pair(nodeId, cloud));
		}
	}
	else
	{
		return;
	}

	_ui->widget_cloudViewer->setCloudOpacity(cloudName, _preferencesDialog->getCloudOpacity(0));
	_ui->widget_cloudViewer->setCloudPointSize(cloudName, _preferencesDialog->getCloudPointSize(0));
}

void MainWindow::createAndAddScanToMap(int nodeId, const Transform & pose, int mapId)
{
	std::string scanName = uFormat("scan%d", nodeId);
	if(_ui->widget_cloudViewer->getAddedClouds().contains(scanName))
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

	if(!iter->sensorData().laserScanCompressed().empty())
	{
		cv::Mat depth2D;
		iter->sensorData().uncompressData(0, 0, &depth2D);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		cloud = util3d::laserScanToPointCloud(depth2D);
		QColor color = Qt::gray;
		if(mapId >= 0)
		{
			color = (Qt::GlobalColor)(mapId+3 % 12 + 7 );
		}
		if(!_ui->widget_cloudViewer->addOrUpdateCloud(scanName, cloud, pose, color))
		{
			UERROR("Adding cloud %d to viewer failed!", nodeId);
		}
		else
		{
			_createdScans.insert(std::make_pair(nodeId, cloud));

			cv::Mat ground, obstacles;
			util3d::occupancy2DFromLaserScan(depth2D, ground, obstacles, _preferencesDialog->getGridMapResolution());
			_gridLocalMaps.insert(std::make_pair(nodeId, std::make_pair(ground, obstacles)));
		}
		_ui->widget_cloudViewer->setCloudOpacity(scanName, _preferencesDialog->getScanOpacity(0));
		_ui->widget_cloudViewer->setCloudPointSize(scanName, _preferencesDialog->getScanPointSize(0));
	}
}

void MainWindow::updateNodeVisibility(int nodeId, bool visible)
{
	if(_currentPosesMap.find(nodeId) != _currentPosesMap.end())
	{
		QMap<std::string, Transform> viewerClouds = _ui->widget_cloudViewer->getAddedClouds();
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
					_ui->widget_cloudViewer->updateCloudPose(cloudName, _currentPosesMap.find(nodeId)->second);
				}
				_ui->widget_cloudViewer->setCloudVisibility(cloudName, visible);
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
					_ui->widget_cloudViewer->updateCloudPose(scanName, _currentPosesMap.find(nodeId)->second);
				}
				_ui->widget_cloudViewer->setCloudVisibility(scanName, visible);
			}
		}
	}
	_ui->widget_cloudViewer->update();
}

void MainWindow::processRtabmapEventInit(int status, const QString & info)
{
	if((RtabmapEventInit::Status)status == RtabmapEventInit::kInitializing)
	{
		_initProgressDialog->setAutoClose(true, 1);
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		this->changeState(MainWindow::kInitializing);
	}
	else if((RtabmapEventInit::Status)status == RtabmapEventInit::kInitialized)
	{
		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
		this->changeState(MainWindow::kInitialized);
	}
	else if((RtabmapEventInit::Status)status == RtabmapEventInit::kClosing)
	{
		_initProgressDialog->setAutoClose(true, 1);
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

		UINFO("Received map!");
		_initProgressDialog->appendText(tr(" poses = %1").arg(event.getPoses().size()));
		_initProgressDialog->appendText(tr(" constraints = %1").arg(event.getConstraints().size()));

		_initProgressDialog->setMaximumSteps(int(event.getSignatures().size()+event.getPoses().size()+1));
		_initProgressDialog->appendText(QString("Inserting data in the cache (%1 signatures downloaded)...").arg(event.getSignatures().size()));
		QApplication::processEvents();

		int addedSignatures = 0;
		std::map<int, int> mapIds;
		for(std::map<int, Signature>::const_iterator iter = event.getSignatures().begin();
			iter!=event.getSignatures().end();
			++iter)
		{
			mapIds.insert(std::make_pair(iter->first, iter->second.mapId()));
			if(!_cachedSignatures.contains(iter->first))
			{
				_cachedSignatures.insert(iter->first, iter->second);
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
			this->updateMapCloud(event.getPoses(), Transform(), event.getConstraints(), mapIds, true);
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
		}
	}
	_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
}

void MainWindow::processRtabmapGlobalPathEvent(const rtabmap::RtabmapGlobalPathEvent & event)
{
	if(!event.getPoses().empty())
	{
		_ui->graphicsView_graphView->setGlobalPath(event.getPoses());
	}

	if(_preferencesDialog->notifyWhenNewGlobalPathIsReceived())
	{
		// use MessageBox
		if(event.getPoses().empty())
		{
			QMessageBox * warn = new QMessageBox(
					QMessageBox::Warning,
					tr("Setting goal failed!"),
					tr("Setting goal to location %1 failed. "
						"Some reasons: \n"
						"1) the location doesn't exist in the graph,\n"
						"2) the location is not linked to the global graph or\n"
						"3) the location is too near of the current location (goal already reached).").arg(event.getGoal()),
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
					tr("Global path computed from %1 to %2 (%3 poses, %4 m).").arg(event.getPoses().front().first).arg(event.getGoal()).arg(event.getPoses().size()).arg(graph::computePathLength(event.getPoses())),
					QMessageBox::Ok,
					this);
			info->setAttribute(Qt::WA_DeleteOnClose, true);
			info->show();
		}
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
			_camera->setImageRate(_preferencesDialog->getGeneralInputRate());

			if(_camera->camera() && dynamic_cast<CameraOpenNI2*>(_camera->camera()) != 0)
			{
				((CameraOpenNI2*)_camera->camera())->setAutoWhiteBalance(_preferencesDialog->getSourceOpenni2AutoWhiteBalance());
				((CameraOpenNI2*)_camera->camera())->setAutoExposure(_preferencesDialog->getSourceOpenni2AutoExposure());
				if(CameraOpenNI2::exposureGainAvailable())
				{
					((CameraOpenNI2*)_camera->camera())->setExposure(_preferencesDialog->getSourceOpenni2Exposure());
					((CameraOpenNI2*)_camera->camera())->setGain(_preferencesDialog->getSourceOpenni2Gain());
				}
			}
			if(_camera)
			{
				_camera->setMirroringEnabled(_preferencesDialog->isSourceMirroring());
				_camera->setColorOnly(_preferencesDialog->isSourceRGBDColorOnly());
			}
		}
		if(_dbReader)
		{
			_dbReader->setFrameRate( _preferencesDialog->getSourceDatabaseStampsUsed()?-1:_preferencesDialog->getGeneralInputRate());
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
					Transform(),
					std::multimap<int, Link>(_currentLinksMap),
					std::map<int, int>(_currentMapIds));
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

		if(_state != kIdle && parametersModified.size())
		{
			if(parametersModified.erase(Parameters::kRtabmapWorkingDirectory()))
			{
				if(_state == kMonitoring || _state == kMonitoringPaused)
				{
					QMessageBox::information(this, tr("Working memory changed"), tr("The remote working directory can't be changed while the interface is in monitoring mode."));
				}
				else
				{
					QMessageBox::information(this, tr("Working memory changed"), tr("The working directory can't be changed while the detector is running. This will be applied when the detector will stop."));
				}
			}
			if(postParamEvent)
			{
				this->post(new ParamEvent(parametersModified));
			}
		}

		if(_state != kMonitoring && _state != kMonitoringPaused &&
		   uContains(parameters, Parameters::kRtabmapWorkingDirectory()))
		{
			_ui->statsToolBox->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
			_ui->graphicsView_graphView->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
			_ui->widget_cloudViewer->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
		}

		// update loop closure viewer parameters
		if(uContains(parameters, Parameters::kLccIcp3Decimation()))
		{
			_ui->widget_loopClosureViewer->setDecimation(atoi(parameters.at(Parameters::kLccIcp3Decimation()).c_str()));
		}
		if(uContains(parameters, Parameters::kLccIcp3MaxDepth()))
		{
			_ui->widget_loopClosureViewer->setMaxDepth(uStr2Float(parameters.at(Parameters::kLccIcp3MaxDepth())));
		}
		if(uContains(parameters, Parameters::kLccIcp3Samples()))
		{
			_ui->widget_loopClosureViewer->setSamples(atoi(parameters.at(Parameters::kLccIcp3Samples()).c_str()));
		}

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
	return QWidget::eventFilter(obj, event);
}

void MainWindow::updateSelectSourceMenu()
{
	_ui->actionUsbCamera->setChecked(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcUsbDevice);

	_ui->actionMore_options->setChecked(
			_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcDatabase ||
			_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcImages ||
			_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcVideo ||
			_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcStereoImages);

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

void MainWindow::captureScreen()
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
	QString name = (QDateTime::currentDateTime().toString("yyMMddhhmmsszzz") + ".png");
	_ui->statusbar->clearMessage();
	QPixmap figure = QPixmap::grabWidget(this);
	figure.save(targetDir + name);
	QString msg = tr("Screen captured \"%1\"").arg(targetDir + name);
	_ui->statusbar->showMessage(msg, _preferencesDialog->getTimeLimit()*500);
	_ui->widget_console->appendMsg(msg);
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
	_preferencesDialog->saveWidgetState(_ui->widget_cloudViewer);
	_preferencesDialog->saveWidgetState(_ui->imageView_source);
	_preferencesDialog->saveWidgetState(_ui->imageView_loopClosure);
	_preferencesDialog->saveWidgetState(_ui->imageView_odometry);
	_preferencesDialog->saveWidgetState(_exportDialog);
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
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdInit, databasePath, 0, _preferencesDialog->getAllParameters()));
	applyPrefSettings(_preferencesDialog->getAllParameters(), false);
}

void MainWindow::openDatabase()
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
	QString path = QFileDialog::getOpenFileName(this, tr("Open database..."), _preferencesDialog->getWorkingDirectory(), tr("RTAB-Map database files (*.db)"));
	if(!path.isEmpty())
	{
		this->clearTheCache();
		_openedDatabasePath = path;
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdInit, path.toStdString(), 0, _preferencesDialog->getAllParameters()));
	}
	applyPrefSettings(_preferencesDialog->getAllParameters(), false);
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

	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdClose));
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
		DatabaseViewer * viewer = new DatabaseViewer(this);
		viewer->setWindowModality(Qt::WindowModal);
		viewer->setAttribute(Qt::WA_DeleteOnClose, true);
		viewer->showCloseButton();
		if(viewer->openDatabase(path))
		{
			if(viewer->isSavedMaximized())
			{
				viewer->showMaximized();
			}
			else
			{
				viewer->show();
			}
		}
		else
		{
			delete viewer;
		}
	}
}

void MainWindow::startDetection()
{
	ParametersMap parameters = _preferencesDialog->getAllParameters();
	// verify source with input rates
	if(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcImages ||
	   _preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcVideo ||
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

	if(!_preferencesDialog->isCloudsShown(0) || !_preferencesDialog->isScansShown(0))
	{
		QMessageBox::information(this,
				tr("Some data may not be shown!"),
				tr("Note that clouds and/or scans visibility settings are set to "
				   "OFF (see General->\"3D Rendering\" section under Map column)."));
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
	if(_dbReader != 0)
	{
		QMessageBox::warning(this,
							 tr("RTAB-Map"),
							 tr("A database reader is running, stop it first."));
		UWARN("_dbReader is not null... it must be stopped first");
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


	if(_preferencesDialog->getSourceDriver() < PreferencesDialog::kSrcDatabase)
	{
		Camera * camera = _preferencesDialog->createCamera();
		if(!camera)
		{
			emit stateChanged(kInitialized);
			return;
		}

		_camera = new CameraThread(camera);
		_camera->setMirroringEnabled(_preferencesDialog->isSourceMirroring());
		_camera->setColorOnly(_preferencesDialog->isSourceRGBDColorOnly());

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
				Odometry * odom;
				if(_preferencesDialog->getOdomStrategy() == 1)
				{
					odom = new OdometryOpticalFlow(parameters);
				}
				else if(_preferencesDialog->getOdomStrategy() == 2)
				{
					odom = new OdometryMono(parameters);
				}
				else
				{
					odom = new OdometryBOW(parameters);
				}
				_odomThread = new OdometryThread(odom, _preferencesDialog->getOdomBufferSize());

				UEventsManager::addHandler(_odomThread);
				UEventsManager::createPipe(_camera, _odomThread, "CameraEvent");
				_odomThread->start();
			}
		}
	}
	else if(_preferencesDialog->getSourceDriver() == PreferencesDialog::kSrcDatabase)
	{
		_dbReader = new DBReader(_preferencesDialog->getSourceDatabasePath().toStdString(),
								 _preferencesDialog->getSourceDatabaseStampsUsed()?-1:_preferencesDialog->getGeneralInputRate(),
								 _preferencesDialog->getSourceDatabaseOdometryIgnored(),
								 _preferencesDialog->getSourceDatabaseGoalDelayIgnored());

		//Create odometry thread if rgdb slam
		if(uStr2Bool(parameters.at(Parameters::kRGBDEnabled()).c_str()) &&
		   _preferencesDialog->getSourceDatabaseOdometryIgnored())
		{
			if(_odomThread)
			{
				UERROR("OdomThread must be already deleted here?!");
				delete _odomThread;
			}
			Odometry * odom;
			if(_preferencesDialog->getOdomStrategy() == 1)
			{
				odom = new OdometryOpticalFlow(parameters);
			}
			else if(_preferencesDialog->getOdomStrategy() == 2)
			{
				odom = new OdometryMono(parameters);
			}
			else
			{
				odom = new OdometryBOW(parameters);
			}
			_odomThread = new OdometryThread(odom);

			UEventsManager::addHandler(_odomThread);
			_odomThread->start();
		}

		if(!_dbReader->init(_preferencesDialog->getSourceDatabaseStartPos()))
		{
			ULOGGER_WARN("init DBReader failed... ");
			QMessageBox::warning(this,
								   tr("RTAB-Map"),
								   tr("Database reader initialization failed..."));
			emit stateChanged(kInitialized);
			delete _dbReader;
			_dbReader = 0;
			if(_odomThread)
			{
				delete _odomThread;
				_odomThread = 0;
			}
			return;
		}

		if(_odomThread)
		{
			UEventsManager::createPipe(_dbReader, _odomThread, "CameraEvent");
		}
	}

	if(_dataRecorder)
	{
		if(_camera)
		{
			UEventsManager::createPipe(_camera, _dataRecorder, "CameraEvent");
		}
		else if(_dbReader)
		{
			UEventsManager::createPipe(_dbReader, _dataRecorder, "CameraEvent");
		}
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

	emit stateChanged(kDetecting);
}

// Could not be in the main thread here! (see handleEvents())
void MainWindow::pauseDetection()
{
	if(_camera || _dbReader)
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
	if(!_camera && !_dbReader && !_odomThread)
	{
		return;
	}

	if(_state == kDetecting &&
			( (_camera && _camera->isRunning()) ||
			  (_dbReader && _dbReader->isRunning()) ) )
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

	if(_dbReader)
	{
		_dbReader->join(true);
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
	if(_dbReader)
	{
		delete _dbReader;
		_dbReader = 0;
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

void MainWindow::generateMap()
{
	if(_graphSavingFileName.isEmpty())
	{
		_graphSavingFileName = _preferencesDialog->getWorkingDirectory() + QDir::separator() + "Graph.dot";
	}
	QString path = QFileDialog::getSaveFileName(this, tr("Save File"), _graphSavingFileName, tr("Graphiz file (*.dot)"));
	if(!path.isEmpty())
	{
		_graphSavingFileName = path;
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGenerateDOTGraph, path.toStdString())); // The event is automatically deleted by the EventsManager...

		_ui->dockWidget_console->show();
		_ui->widget_console->appendMsg(QString("Graph saved... Tip:\nneato -Tpdf \"%1\" -o out.pdf").arg(_graphSavingFileName).arg(_graphSavingFileName));
	}
}

void MainWindow::generateLocalMap()
{
	if(_graphSavingFileName.isEmpty())
	{
		_graphSavingFileName = _preferencesDialog->getWorkingDirectory() + QDir::separator() + "Graph.dot";
	}

	bool ok = false;
	int loopId = 1;
	if(_ui->label_matchId->text().size())
	{
		std::list<std::string> values = uSplitNumChar(_ui->label_matchId->text().toStdString());
		if(values.size() > 1)
		{
			int val = QString((++values.begin())->c_str()).toInt(&ok);
			if(ok)
			{
				loopId = val;
			}
			ok = false;
		}
	}
	int id = QInputDialog::getInt(this, tr("Around which location?"), tr("Location ID"), loopId, 1, 999999, 1, &ok);
	if(ok)
	{
		int margin = QInputDialog::getInt(this, tr("Depth around the location?"), tr("Margin"), 4, 1, 100, 1, &ok);
		if(ok)
		{
			QString path = QFileDialog::getSaveFileName(this, tr("Save File"), _graphSavingFileName, tr("Graphiz file (*.dot)"));
			if(!path.isEmpty())
			{
				_graphSavingFileName = path;
				QString str = path + QString(";") + QString::number(id) + QString(";") + QString::number(margin);
				this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGenerateDOTLocalGraph, str.toStdString())); // The event is automatically deleted by the EventsManager...

				_ui->dockWidget_console->show();
				_ui->widget_console->appendMsg(QString("Graph saved... Tip:\nneato -Tpdf \"%1\" -o out.pdf").arg(_graphSavingFileName).arg(_graphSavingFileName));
			}
		}
	}
}

void MainWindow::generateTOROMap()
{
	if(_toroSavingFileName.isEmpty())
	{
		_toroSavingFileName = _preferencesDialog->getWorkingDirectory() + QDir::separator() + "toro.graph";
	}

	QStringList items;
	items.append("Local map optimized");
	items.append("Local map not optimized");
	items.append("Global map optimized");
	items.append("Global map not optimized");
	bool ok;
	QString item = QInputDialog::getItem(this, tr("Parameters"), tr("Options:"), items, 2, false, &ok);
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

		QString path = QFileDialog::getSaveFileName(this, tr("Save File"), _toroSavingFileName, tr("TORO file (*.graph)"));
		if(!path.isEmpty())
		{
			_toroSavingFileName = path;
			if(global)
			{
				this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGenerateTOROGraphGlobal, path.toStdString(), optimized?1:0));
			}
			else
			{
				this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGenerateTOROGraphLocal, path.toStdString(), optimized?1:0));
			}

			_ui->dockWidget_console->show();
			_ui->widget_console->appendMsg(QString("TORO Graph saved (global=%1, optimized=%2)... %3")
					.arg(global?"true":"false").arg(optimized?"true":"false").arg(_toroSavingFileName));
		}

	}
}

void MainWindow::exportPoses()
{
	if(_posesSavingFileName.isEmpty())
	{
		_posesSavingFileName = _preferencesDialog->getWorkingDirectory() + QDir::separator() + "poses.txt";
	}

	QStringList items;
	items.append("Local map optimized");
	items.append("Local map not optimized");
	items.append("Global map optimized");
	items.append("Global map not optimized");
	bool ok;
	QString item = QInputDialog::getItem(this, tr("Parameters"), tr("Options:"), items, 2, false, &ok);
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

		QString path = QFileDialog::getSaveFileName(this, tr("Save File"), _posesSavingFileName, tr("Text file (*.txt)"));
		if(!path.isEmpty())
		{
			_posesSavingFileName = path;
			if(global)
			{
				this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdExportPosesGlobal, path.toStdString(), optimized?1:0));
			}
			else
			{
				this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdExportPosesLocal, path.toStdString(), optimized?1:0));
			}

			_ui->dockWidget_console->show();
			_ui->widget_console->appendMsg(QString("Poses saved (global=%1, optimized=%2)... %3")
					.arg(global?"true":"false").arg(optimized?"true":"false").arg(_posesSavingFileName));
		}

	}
}

void MainWindow::postProcessing()
{
	if(_cachedSignatures.size() == 0)
	{
		UERROR("Signatures must be cached in the GUI to post processing.");
		return;
	}
	if(_postProcessingDialog->exec() != QDialog::Accepted)
	{
		return;
	}

	bool detectMoreLoopClosures = _postProcessingDialog->isDetectMoreLoopClosures();
	bool reextractFeatures = _postProcessingDialog->isReextractFeatures();
	bool refineNeighborLinks = _postProcessingDialog->isRefineNeighborLinks();
	bool refineLoopClosureLinks = _postProcessingDialog->isRefineLoopClosureLinks();
	double clusterRadius = _postProcessingDialog->clusterRadius();
	double clusterAngle = _postProcessingDialog->clusterAngle();
	int detectLoopClosureIterations = _postProcessingDialog->iterations();

	if(!detectMoreLoopClosures && !refineNeighborLinks && !refineLoopClosureLinks)
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
			if(jter->sensorData().cameraModels().size() == 0 && !jter->sensorData().stereoCameraModel().isValid())
			{
				UWARN("Calibration of %d is null.", iter->first);
				allDataAvailable = false;
			}
			if(refineNeighborLinks || refineLoopClosureLinks || reextractFeatures)
			{
				// depth data required
				if(jter->sensorData().depthOrRightCompressed().empty())
				{
					UWARN("Depth data of %d missing.", iter->first);
					allDataAvailable = false;
				}

				if(reextractFeatures)
				{
					// rgb required
					if(jter->sensorData().imageCompressed().empty())
					{
						UWARN("Rgb of %d missing.", iter->first);
						allDataAvailable = false;
					}
				}
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

	_initProgressDialog->setAutoClose(false, 1);
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
	_initProgressDialog->setMaximumSteps(totalSteps);
	_initProgressDialog->show();

	ParametersMap parameters = _preferencesDialog->getAllParameters();
	graph::Optimizer * optimizer = graph::Optimizer::create(parameters);
	bool optimizeFromGraphEnd =  Parameters::defaultRGBDOptimizeFromGraphEnd();
	Parameters::parse(parameters, Parameters::kRGBDOptimizeFromGraphEnd(), optimizeFromGraphEnd);

	int loopClosuresAdded = 0;
	if(detectMoreLoopClosures)
	{
		UDEBUG("");
		Memory memory(parameters);
		if(reextractFeatures)
		{
			ParametersMap customParameters;
			// override some parameters
			customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
			customParameters.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
			customParameters.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
			customParameters.insert(ParametersPair(Parameters::kKpNewWordsComparedTogether(), "false"));
			customParameters.insert(ParametersPair(Parameters::kKpNNStrategy(), parameters.at(Parameters::kLccReextractNNType())));
			customParameters.insert(ParametersPair(Parameters::kKpNndrRatio(), parameters.at(Parameters::kLccReextractNNDR())));
			customParameters.insert(ParametersPair(Parameters::kKpDetectorStrategy(), parameters.at(Parameters::kLccReextractFeatureType())));
			customParameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), parameters.at(Parameters::kLccReextractMaxWords())));
			customParameters.insert(ParametersPair(Parameters::kMemGenerateIds(), "false"));
			memory.parseParameters(customParameters);
		}

		UASSERT(detectLoopClosureIterations>0);
		for(int n=0; n<detectLoopClosureIterations; ++n)
		{
			_initProgressDialog->appendText(tr("Looking for more loop closures, clustering poses... (iteration=%1/%2, radius=%3 m angle=%4 degrees)")
					.arg(n+1).arg(detectLoopClosureIterations).arg(clusterRadius).arg(clusterAngle));

			std::multimap<int, int> clusters = rtabmap::graph::radiusPosesClustering(
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

				// only add new links and one per cluster per iteration
				if(addedLinks.find(from) == addedLinks.end() && addedLinks.find(to) == addedLinks.end() &&
				   rtabmap::graph::findLink(_currentLinksMap, from, to) == _currentLinksMap.end())
				{
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

						Signature & signatureFrom = _cachedSignatures[from];
						Signature & signatureTo = _cachedSignatures[to];

						Transform transform;
						std::string rejectedMsg;
						int inliers = -1;
						double variance = -1.0;
						if(reextractFeatures)
						{
							memory.init("", true); // clear previously added signatures

							// Add signatures
							SensorData dataFrom = signatureFrom.sensorData();
							SensorData dataTo = signatureTo.sensorData();

							cv::Mat image, depth;
							dataFrom.uncompressData(&image, &depth, 0);
							dataTo.uncompressData(&image, &depth, 0);

							if(dataFrom.isValid() &&
							   dataTo.isValid() &&
							   dataFrom.id() != Memory::kIdInvalid &&
							   signatureFrom.id() != Memory::kIdInvalid)
							{
								if(from > to)
								{
									memory.update(dataTo);
									memory.update(dataFrom);
								}
								else
								{
									memory.update(dataFrom);
									memory.update(dataTo);
								}

								transform = memory.computeVisualTransform(dataTo.id(), dataFrom.id(), &rejectedMsg, &inliers, &variance);
							}
							else
							{
								UERROR("not supposed to be here!");
							}
						}
						else
						{
							transform = memory.computeVisualTransform(signatureTo, signatureFrom, &rejectedMsg, &inliers, &variance);
						}
						if(!transform.isNull())
						{
							UINFO("Added new loop closure between %d and %d.", from, to);
							addedLinks.insert(from);
							addedLinks.insert(to);
							_currentLinksMap.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, transform, variance, variance)));
							++loopClosuresAdded;
							_initProgressDialog->appendText(tr("Detected loop closure %1->%2! (%3/%4)").arg(from).arg(to).arg(i+1).arg(clusters.size()));
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
		_initProgressDialog->appendText(tr("Refining links..."));

		int decimation=Parameters::defaultLccIcp3Decimation();
		float maxDepth=Parameters::defaultLccIcp3MaxDepth();
		float voxelSize=Parameters::defaultLccIcp3VoxelSize();
		int samples = Parameters::defaultLccIcp3Samples();
		float maxCorrespondenceDistance = Parameters::defaultLccIcp3MaxCorrespondenceDistance();
		float correspondenceRatio = Parameters::defaultLccIcp3CorrespondenceRatio();
		float icpIterations = Parameters::defaultLccIcp3Iterations();
		Parameters::parse(parameters, Parameters::kLccIcp3Decimation(), decimation);
		Parameters::parse(parameters, Parameters::kLccIcp3MaxDepth(), maxDepth);
		Parameters::parse(parameters, Parameters::kLccIcp3VoxelSize(), voxelSize);
		Parameters::parse(parameters, Parameters::kLccIcp3Samples(), samples);
		Parameters::parse(parameters, Parameters::kLccIcp3CorrespondenceRatio(), correspondenceRatio);
		Parameters::parse(parameters, Parameters::kLccIcp3MaxCorrespondenceDistance(), maxCorrespondenceDistance);
		Parameters::parse(parameters, Parameters::kLccIcp3Iterations(), icpIterations);
		bool pointToPlane = Parameters::defaultLccIcp3PointToPlane();
		int pointToPlaneNormalNeighbors = Parameters::defaultLccIcp3PointToPlaneNormalNeighbors();
		Parameters::parse(parameters, Parameters::kLccIcp3PointToPlane(), pointToPlane);
		Parameters::parse(parameters, Parameters::kLccIcp3PointToPlaneNormalNeighbors(), pointToPlaneNormalNeighbors);

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

					//3D
					UDEBUG("");
					cv::Mat depthA, depthB;
					if(signatureFrom.sensorData().stereoCameraModel().isValid())
					{
						cv::Mat leftA, leftB;
						signatureFrom.sensorData().uncompressData(&leftA, &depthA, 0);
						signatureTo.sensorData().uncompressData(&leftB, &depthB, 0);
					}
					else
					{
						signatureFrom.sensorData().uncompressData(0, &depthA, 0);
						signatureTo.sensorData().uncompressData(0, &depthB, 0);
					}

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA = util3d::cloudFromSensorData(
							signatureFrom.sensorData(),
							decimation,
							maxDepth,
							voxelSize,
							samples);
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB = util3d::cloudFromSensorData(
							signatureTo.sensorData(),
							decimation,
							maxDepth,
							voxelSize,
							samples);
					if(cloudA->size() && cloudB->size())
					{
						cloudB = util3d::transformPointCloud(cloudB, iter->second.transform());

						bool hasConverged = false;
						double variance = -1;
						int correspondences = 0;
						Transform transform;
						if(pointToPlane)
						{
							UDEBUG("");
							pcl::PointCloud<pcl::PointNormal>::Ptr cloudANormals = util3d::computeNormals(cloudA, pointToPlaneNormalNeighbors);
							pcl::PointCloud<pcl::PointNormal>::Ptr cloudBNormals = util3d::computeNormals(cloudB, pointToPlaneNormalNeighbors);

							cloudANormals = util3d::removeNaNNormalsFromPointCloud(cloudANormals);
							if(cloudA->size() != cloudANormals->size())
							{
								UWARN("removed nan normals...");
							}

							cloudBNormals = util3d::removeNaNNormalsFromPointCloud(cloudBNormals);
							if(cloudB->size() != cloudBNormals->size())
							{
								UWARN("removed nan normals...");
							}

							pcl::PointCloud<pcl::PointNormal>::Ptr cloudBRegistered(new pcl::PointCloud<pcl::PointNormal>);
							transform = util3d::icpPointToPlane(cloudBNormals,
									cloudANormals,
									maxCorrespondenceDistance,
									icpIterations,
									hasConverged,
									*cloudBRegistered);
							util3d::computeVarianceAndCorrespondences(
									cloudBRegistered,
									cloudANormals,
									maxCorrespondenceDistance,
									variance,
									correspondences);
						}
						else
						{
							UDEBUG("");
							pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBRegistered(new pcl::PointCloud<pcl::PointXYZ>);
							transform = util3d::icp(cloudB,
									cloudA,
									maxCorrespondenceDistance,
									icpIterations,
									hasConverged,
									*cloudBRegistered);
							util3d::computeVarianceAndCorrespondences(
									cloudBRegistered,
									cloudA,
									maxCorrespondenceDistance,
									variance,
									correspondences);
						}

						float correspondencesRatio = float(correspondences)/float(cloudB->size()>cloudA->size()?cloudB->size():cloudA->size());

						if(!transform.isNull() && hasConverged &&
						   correspondencesRatio >= correspondenceRatio)
						{
							Link newLink(from, to, iter->second.type(), transform*iter->second.transform(), variance, variance);
							iter->second = newLink;
						}
						else
						{
							QString str = tr("Cannot refine link %1->%2 (converged=%3 variance=%4 correspondencesRatio=%5 (ref=%6))").arg(from).arg(to).arg(hasConverged?"true":"false").arg(variance).arg(correspondencesRatio).arg(correspondenceRatio);
							_initProgressDialog->appendText(str, Qt::darkYellow);
							UWARN("%s", str.toStdString().c_str());
						}
					}
					else
					{
						QString str = tr("Cannot refine link %1->%2 (clouds empty!)").arg(from).arg(to);
						_initProgressDialog->appendText(str, Qt::darkYellow);
						UWARN("%s", str.toStdString().c_str());
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

	_initProgressDialog->appendText(tr("Updating map..."));
	this->updateMapCloud(optimizedPoses, Transform(), std::multimap<int, Link>(_currentLinksMap), std::map<int, int>(_currentMapIds), false);
	_initProgressDialog->appendText(tr("Updating map... done!"));

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
	int id = QInputDialog::getInt(this, tr("Send a goal"), tr("Goal location ID: "), 1, 1, 99999, 1, &ok);
	if(ok)
	{
		_ui->graphicsView_graphView->setGlobalPath(std::vector<std::pair<int, Transform> >()); // clear
		UINFO("Posting event with goal %d", id);
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdGoal, "", id));
	}
}

void MainWindow::cancelGoal()
{
	UINFO("Cancelling goal...");
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdCancelGoal));
}

void MainWindow::downloadAllClouds()
{
	QStringList items;
	items.append("Local map optimized");
	items.append("Local map not optimized");
	items.append("Global map optimized");
	items.append("Global map not optimized");

	bool ok;
	QString item = QInputDialog::getItem(this, tr("Parameters"), tr("Options:"), items, 2, false, &ok);
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
		_initProgressDialog->setAutoClose(true, 1);
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		_initProgressDialog->appendText(tr("Downloading the map (global=%1 ,optimized=%2)...")
				.arg(global?"true":"false").arg(optimized?"true":"false"));
		if(global)
		{
			this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPublish3DMapGlobal, "", optimized?1:0));
		}
		else
		{
			this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPublish3DMapLocal, "", optimized?1:0));
		}
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
	QString item = QInputDialog::getItem(this, tr("Parameters"), tr("Options:"), items, 2, false, &ok);
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
		_initProgressDialog->setAutoClose(true, 1);
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		_initProgressDialog->appendText(tr("Downloading the graph (global=%1 ,optimized=%2)...")
				.arg(global?"true":"false").arg(optimized?"true":"false"));
		if(global)
		{
			this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPublishTOROGraphGlobal, "", optimized?1:0));
		}
		else
		{
			this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPublishTOROGraphLocal, "", optimized?1:0));
		}
	}
}

void MainWindow::clearTheCache()
{
	_cachedSignatures.clear();
	_createdClouds.clear();
	_createdScans.clear();
	_gridLocalMaps.clear();
	_projectionLocalMaps.clear();
	_ui->widget_cloudViewer->removeAllClouds();
	_ui->widget_cloudViewer->removeAllGraphs();
	_ui->widget_cloudViewer->setBackgroundColor(_ui->widget_cloudViewer->getDefaultBackgroundColor());
	_ui->widget_cloudViewer->clearTrajectory();
	_ui->widget_mapVisibility->clear();
	_currentPosesMap.clear();
	_currentLinksMap.clear();
	_currentMapIds.clear();
	_odometryCorrection = Transform::getIdentity();
	_lastOdomPose.setNull();
	//disable save cloud action
	_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(false);
	_ui->actionExport_2D_scans_ply_pcd->setEnabled(false);
	_ui->actionPost_processing->setEnabled(false);
	_ui->actionSave_point_cloud->setEnabled(false);
	_ui->actionView_scans->setEnabled(false);
	_ui->actionView_high_res_point_cloud->setEnabled(false);
	_likelihoodCurve->clear();
	_rawLikelihoodCurve->clear();
	_posteriorCurve->clear();
	_lastId = 0;
	_lastIds.clear();
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
		}
		else
		{
			_ui->actionAuto_screen_capture->setChecked(false);
		}
	}
}

void MainWindow::takeScreenshot()
{
	this->captureScreen();
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
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> scans;
	if(getExportedScans(scans))
	{
		if(scans.size())
		{
			QMessageBox::StandardButton b = QMessageBox::question(this,
						tr("Binary file?"),
						tr("Do you want to save in binary mode?"),
						QMessageBox::No | QMessageBox::Yes,
						QMessageBox::Yes);

			if(b == QMessageBox::No || b == QMessageBox::Yes)
			{
				this->saveScans(scans, b == QMessageBox::Yes);
			}
		}
		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
	}
}

void MainWindow::viewScans()
{
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> scans;
	if(getExportedScans(scans))
	{
		QDialog * window = new QDialog(this, Qt::Window);
		window->setWindowFlags(Qt::Dialog);
		window->setWindowTitle(tr("Scans (%1 nodes)").arg(scans.size()));
		window->setMinimumWidth(800);
		window->setMinimumHeight(600);

		CloudViewer * viewer = new CloudViewer(window);
		viewer->setCameraLockZ(false);

		QVBoxLayout *layout = new QVBoxLayout();
		layout->addWidget(viewer);
		window->setLayout(layout);
		connect(window, SIGNAL(finished(int)), viewer, SLOT(clear()));

		window->show();

		uSleep(500);

		for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator iter = scans.begin(); iter!=scans.end(); ++iter)
		{
			_initProgressDialog->appendText(tr("Viewing the scan %1 (%2 points)...").arg(iter->first).arg(iter->second->size()));
			_initProgressDialog->incrementStep();

			QColor color = Qt::red;
			int mapId = uValue(_currentMapIds, iter->first, -1);
			if(mapId >= 0)
			{
				color = (Qt::GlobalColor)(mapId % 12 + 7 );
			}
			viewer->addCloud(uFormat("cloud%d",iter->first), iter->second, iter->first>0?_currentPosesMap.at(iter->first):Transform::getIdentity());
			_initProgressDialog->appendText(tr("Viewing the scan %1 (%2 points)... done.").arg(iter->first).arg(iter->second->size()));
		}

		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
	}
}

bool MainWindow::getExportedScans(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans)
{
	QMessageBox::StandardButton b = QMessageBox::question(this,
				tr("Assemble scans?"),
				tr("Do you want to assemble the scans in only one cloud?"),
				QMessageBox::No | QMessageBox::Yes,
				QMessageBox::Yes);

	if(b != QMessageBox::No && b != QMessageBox::Yes)
	{
		return false;
	}

	double voxel = 0.01;
	bool assemble = b == QMessageBox::Yes;

	if(assemble)
	{
		bool ok;
		voxel = QInputDialog::getDouble(this, tr("Voxel size"), tr("Voxel size (m):"), voxel, 0.00, 0.1, 2, &ok);
		if(!ok)
		{
			return false;
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr assembledScans(new pcl::PointCloud<pcl::PointXYZ>());
	std::map<int, Transform> poses = _ui->widget_mapVisibility->getVisiblePoses();

	_initProgressDialog->setAutoClose(true, 1);
	_initProgressDialog->resetProgress();
	_initProgressDialog->show();
	_initProgressDialog->setMaximumSteps(int(poses.size())*(assemble?1:2)+1);

	int count = 1;
	int i = 0;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		bool inserted = false;
		if(_createdScans.find(iter->first) != _createdScans.end())
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr scan = _createdScans.at(iter->first);
			if(scan->size())
			{
				if(assemble)
				{
					*assembledScans += *util3d::transformPointCloud(scan, iter->second);;

					if(count++ % 100 == 0)
					{
						if(assembledScans->size() && voxel)
						{
							assembledScans = util3d::voxelize(assembledScans, voxel);
						}
					}
				}
				else
				{
					scans.insert(std::make_pair(iter->first, scan));
				}
				inserted = true;
			}
		}
		if(inserted)
		{
			_initProgressDialog->appendText(tr("Generated scan %1 (%2/%3).").arg(iter->first).arg(++i).arg(poses.size()));
		}
		else
		{
			_initProgressDialog->appendText(tr("Ignored scan %1 (%2/%3).").arg(iter->first).arg(++i).arg(poses.size()));
		}
		_initProgressDialog->incrementStep();
		QApplication::processEvents();
	}

	if(assemble)
	{
		if(voxel && assembledScans->size())
		{
			assembledScans = util3d::voxelize(assembledScans, voxel);
		}
		if(assembledScans->size())
		{
			scans.insert(std::make_pair(0, assembledScans));
		}
	}
	return true;
}

void MainWindow::exportClouds()
{
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::map<int, pcl::PolygonMesh::Ptr> meshes;

	if(getExportedClouds(clouds, meshes, true))
	{
		if(meshes.size())
		{
			saveMeshes(meshes, _exportDialog->getBinaryFile());
		}
		else
		{
			saveClouds(clouds, _exportDialog->getBinaryFile());
		}
		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
	}
}

void MainWindow::viewClouds()
{
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::map<int, pcl::PolygonMesh::Ptr> meshes;

	if(getExportedClouds(clouds, meshes, false))
	{
		QDialog * window = new QDialog(this, Qt::Window);
		if(meshes.size())
		{
			window->setWindowTitle(tr("Meshes (%1 nodes)").arg(meshes.size()));
		}
		else
		{
			window->setWindowTitle(tr("Clouds (%1 nodes)").arg(clouds.size()));
		}
		window->setMinimumWidth(800);
		window->setMinimumHeight(600);

		CloudViewer * viewer = new CloudViewer(window);
		viewer->setCameraLockZ(false);

		QVBoxLayout *layout = new QVBoxLayout();
		layout->addWidget(viewer);
		window->setLayout(layout);
		connect(window, SIGNAL(finished(int)), viewer, SLOT(clear()));

		window->show();

		uSleep(500);

		if(meshes.size())
		{
			for(std::map<int, pcl::PolygonMesh::Ptr>::iterator iter = meshes.begin(); iter!=meshes.end(); ++iter)
			{
				_initProgressDialog->appendText(tr("Viewing the mesh %1 (%2 polygons)...").arg(iter->first).arg(iter->second->polygons.size()));
				_initProgressDialog->incrementStep();
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::fromPCLPointCloud2(iter->second->cloud, *cloud);
				viewer->addCloudMesh(uFormat("mesh%d",iter->first), cloud, iter->second->polygons, iter->first>0?_currentPosesMap.at(iter->first):Transform::getIdentity());
				_initProgressDialog->appendText(tr("Viewing the mesh %1 (%2 polygons)... done.").arg(iter->first).arg(iter->second->polygons.size()));
				QApplication::processEvents();
			}
		}
		else if(clouds.size())
		{
			for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator iter = clouds.begin(); iter!=clouds.end(); ++iter)
			{
				_initProgressDialog->appendText(tr("Viewing the cloud %1 (%2 points)...").arg(iter->first).arg(iter->second->size()));
				_initProgressDialog->incrementStep();

				QColor color = Qt::gray;
				int mapId = uValue(_currentMapIds, iter->first, -1);
				if(mapId >= 0)
				{
					color = (Qt::GlobalColor)(mapId % 12 + 7 );
				}
				viewer->addCloud(uFormat("cloud%d",iter->first), iter->second, iter->first>0?_currentPosesMap.at(iter->first):Transform::getIdentity());
				_initProgressDialog->appendText(tr("Viewing the cloud %1 (%2 points)... done.").arg(iter->first).arg(iter->second->size()));
			}
		}

		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
	}
}

bool MainWindow::getExportedClouds(
		std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds,
		std::map<int, pcl::PolygonMesh::Ptr> & meshes,
		bool toSave)
{
	if(_exportDialog->isVisible())
	{
		return false;
	}
	if(toSave)
	{
		_exportDialog->setSaveButton();
	}
	else
	{
		_exportDialog->setOkButton();
	}
	_exportDialog->enableRegeneration(_preferencesDialog->isImagesKept());
	if(_exportDialog->exec() == QDialog::Accepted)
	{
		std::map<int, Transform> poses = _ui->widget_mapVisibility->getVisiblePoses();

		_initProgressDialog->setAutoClose(true, 1);
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		int mul = _exportDialog->getMesh()&&!_exportDialog->getGenerate()?3:_exportDialog->getMLS()&&!_exportDialog->getGenerate()?2:1;
		_initProgressDialog->setMaximumSteps(int(poses.size())*mul+1);

		if(_exportDialog->getAssemble())
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = this->getAssembledCloud(
					poses,
					_exportDialog->getAssembleVoxel(),
					_exportDialog->getGenerate(),
					_exportDialog->getGenerateDecimation(),
					_exportDialog->getGenerateVoxel(),
					_exportDialog->getGenerateMaxDepth());

			clouds.insert(std::make_pair(0, cloud));
		}
		else
		{
			clouds = this->getClouds(
					poses,
					_exportDialog->getGenerate(),
					_exportDialog->getGenerateDecimation(),
					_exportDialog->getGenerateVoxel(),
					_exportDialog->getGenerateMaxDepth());
		}

		if(_exportDialog->getMLS() || _exportDialog->getMesh())
		{
			for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator iter=clouds.begin();
				iter!= clouds.end();
				++iter)
			{
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals;
				if(_exportDialog->getMLS())
				{
					_initProgressDialog->appendText(tr("Smoothing the surface of cloud %1 using Moving Least Squares (MLS) algorithm... "
							"[search radius=%2m]").arg(iter->first).arg(_exportDialog->getMLSRadius()));
					_initProgressDialog->incrementStep();
					QApplication::processEvents();

					cloudWithNormals = util3d::computeNormalsSmoothed(iter->second, (float)_exportDialog->getMLSRadius());

					iter->second->clear();
					pcl::copyPointCloud(*cloudWithNormals, *iter->second);
				}
				else if(_exportDialog->getMesh())
				{
					_initProgressDialog->appendText(tr("Computing surface normals of cloud %1 (without smoothing)... "
								"[K neighbors=%2]").arg(iter->first).arg(_exportDialog->getMeshNormalKSearch()));
					_initProgressDialog->incrementStep();
					QApplication::processEvents();

					cloudWithNormals = util3d::computeNormals(iter->second, _exportDialog->getMeshNormalKSearch());
				}

				if(_exportDialog->getMesh())
				{
					_initProgressDialog->appendText(tr("Greedy projection triangulation... [radius=%1m]").arg(_exportDialog->getMeshGp3Radius()));
					_initProgressDialog->incrementStep();
					QApplication::processEvents();

					pcl::PolygonMesh::Ptr mesh = util3d::createMesh(cloudWithNormals, _exportDialog->getMeshGp3Radius());
					meshes.insert(std::make_pair(iter->first, mesh));
				}
			}
		}

		return true;
	}
	return false;
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

void MainWindow::saveClouds(const std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds, bool binaryMode)
{
	if(clouds.size() == 1)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save to ..."), _preferencesDialog->getWorkingDirectory()+QDir::separator()+"cloud.ply", tr("Point cloud data (*.ply *.pcd)"));
		if(!path.isEmpty())
		{
			if(clouds.begin()->second->size())
			{
				_initProgressDialog->appendText(tr("Saving the cloud (%1 points)...").arg(clouds.begin()->second->size()));

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
					_initProgressDialog->incrementStep();
					_initProgressDialog->appendText(tr("Saving the cloud (%1 points)... done.").arg(clouds.begin()->second->size()));

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
		QString path = QFileDialog::getExistingDirectory(this, tr("Save to (*.ply *.pcd)..."), _preferencesDialog->getWorkingDirectory(), 0);
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
					for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::const_iterator iter=clouds.begin(); iter!=clouds.end(); ++iter)
					{
						if(iter->second->size())
						{
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud;
							transformedCloud = util3d::transformPointCloud(iter->second, _currentPosesMap.at(iter->first));

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
								_initProgressDialog->appendText(tr("Saved cloud %1 (%2 points) to %3.").arg(iter->first).arg(iter->second->size()).arg(pathFile));
							}
							else
							{
								_initProgressDialog->appendText(tr("Failed saving cloud %1 (%2 points) to %3.").arg(iter->first).arg(iter->second->size()).arg(pathFile));
							}
						}
						else
						{
							_initProgressDialog->appendText(tr("Cloud %1 is empty!").arg(iter->first));
						}
						_initProgressDialog->incrementStep();
						QApplication::processEvents();
					}
				}
			}
		}
	}
}

void MainWindow::saveMeshes(const std::map<int, pcl::PolygonMesh::Ptr> & meshes, bool binaryMode)
{
	if(meshes.size() == 1)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save to ..."), _preferencesDialog->getWorkingDirectory()+QDir::separator()+"mesh.ply", tr("Mesh (*.ply)"));
		if(!path.isEmpty())
		{
			if(meshes.begin()->second->polygons.size())
			{
				_initProgressDialog->appendText(tr("Saving the mesh (%1 polygons)...").arg(meshes.begin()->second->polygons.size()));

				bool success =false;
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
				else if(QFileInfo(path).suffix() == "")
				{
					//default ply
					path += ".ply";
					if(binaryMode)
					{
						success = pcl::io::savePLYFileBinary(path.toStdString(), *meshes.begin()->second) == 0;
					}
					else
					{
						success = pcl::io::savePLYFile(path.toStdString(), *meshes.begin()->second) == 0;
					}
				}
				else
				{
					UERROR("Extension not recognized! (%s) Should be (*.ply).", QFileInfo(path).suffix().toStdString().c_str());
				}
				if(success)
				{
					_initProgressDialog->incrementStep();
					_initProgressDialog->appendText(tr("Saving the mesh (%1 polygons)... done.").arg(meshes.begin()->second->polygons.size()));

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
		QString path = QFileDialog::getExistingDirectory(this, tr("Save to (*.ply)..."), _preferencesDialog->getWorkingDirectory(), 0);
		if(!path.isEmpty())
		{
			bool ok = false;
			QString prefix = QInputDialog::getText(this, tr("File prefix"), tr("Prefix:"), QLineEdit::Normal, "mesh", &ok);
			QString suffix = "ply";

			if(ok)
			{
				for(std::map<int, pcl::PolygonMesh::Ptr>::const_iterator iter=meshes.begin(); iter!=meshes.end(); ++iter)
				{
					if(iter->second->polygons.size())
					{
						pcl::PolygonMesh mesh;
						mesh.polygons = iter->second->polygons;
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
						pcl::fromPCLPointCloud2(iter->second->cloud, *tmp);
						tmp = util3d::transformPointCloud(tmp, _currentPosesMap.at(iter->first));
						pcl::toPCLPointCloud2(*tmp, mesh.cloud);

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
						else
						{
							UFATAL("Extension not recognized! (%s)", suffix.toStdString().c_str());
						}
						if(success)
						{
							_initProgressDialog->appendText(tr("Saved mesh %1 (%2 polygons) to %3.")
									.arg(iter->first).arg(iter->second->polygons.size()).arg(pathFile));
						}
						else
						{
							_initProgressDialog->appendText(tr("Failed saving mesh %1 (%2 polygons) to %3.")
									.arg(iter->first).arg(iter->second->polygons.size()).arg(pathFile));
						}
					}
					else
					{
						_initProgressDialog->appendText(tr("Mesh %1 is empty!").arg(iter->first));
					}
					_initProgressDialog->incrementStep();
					QApplication::processEvents();
				}
			}
		}
	}
}

void MainWindow::saveScans(const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> & scans, bool binaryMode)
{
	if(scans.size() == 1)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save to ..."), _preferencesDialog->getWorkingDirectory()+QDir::separator()+"scan.ply", tr("Point cloud data (*.ply *.pcd)"));
		if(!path.isEmpty())
		{
			if(scans.begin()->second->size())
			{
				_initProgressDialog->appendText(tr("Saving the scan (%1 points)...").arg(scans.begin()->second->size()));

				bool success =false;
				if(QFileInfo(path).suffix() == "pcd")
				{
					success = pcl::io::savePCDFile(path.toStdString(), *scans.begin()->second, binaryMode) == 0;
				}
				else if(QFileInfo(path).suffix() == "ply")
				{
					success = pcl::io::savePLYFile(path.toStdString(), *scans.begin()->second, binaryMode) == 0;
				}
				else if(QFileInfo(path).suffix() == "")
				{
					//use ply by default
					path += ".ply";
					success = pcl::io::savePLYFile(path.toStdString(), *scans.begin()->second, binaryMode) == 0;
				}
				else
				{
					UERROR("Extension not recognized! (%s) Should be one of (*.ply *.pcd).", QFileInfo(path).suffix().toStdString().c_str());
				}
				if(success)
				{
					_initProgressDialog->incrementStep();
					_initProgressDialog->appendText(tr("Saving the scan (%1 points)... done.").arg(scans.begin()->second->size()));

					QMessageBox::information(this, tr("Save successful!"), tr("Scan saved to \"%1\"").arg(path));
				}
				else
				{
					QMessageBox::warning(this, tr("Save failed!"), tr("Failed to save to \"%1\"").arg(path));
				}
			}
			else
			{
				QMessageBox::warning(this, tr("Save failed!"), tr("Scan is empty..."));
			}
		}
	}
	else if(scans.size())
	{
		QString path = QFileDialog::getExistingDirectory(this, tr("Save to (*.ply *.pcd)..."), _preferencesDialog->getWorkingDirectory(), 0);
		if(!path.isEmpty())
		{
			bool ok = false;
			QStringList items;
			items.push_back("ply");
			items.push_back("pcd");
			QString suffix = QInputDialog::getItem(this, tr("File format"), tr("Which format?"), items, 0, false, &ok);

			if(ok)
			{
				QString prefix = QInputDialog::getText(this, tr("File prefix"), tr("Prefix:"), QLineEdit::Normal, "scan", &ok);

				if(ok)
				{
					for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::const_iterator iter=scans.begin(); iter!=scans.end(); ++iter)
					{
						if(iter->second->size())
						{
							pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud;
							transformedCloud = util3d::transformPointCloud(iter->second, _currentPosesMap.at(iter->first));

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
								_initProgressDialog->appendText(tr("Saved scan %1 (%2 points) to %3.").arg(iter->first).arg(iter->second->size()).arg(pathFile));
							}
							else
							{
								_initProgressDialog->appendText(tr("Failed saving scan %1 (%2 points) to %3.").arg(iter->first).arg(iter->second->size()).arg(pathFile));
							}
						}
						else
						{
							_initProgressDialog->appendText(tr("Scan %1 is empty!").arg(iter->first));
						}
						_initProgressDialog->incrementStep();
						QApplication::processEvents();
					}
				}
			}
		}
	}
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MainWindow::getAssembledCloud(
		const std::map<int, Transform> & poses,
		float assembledVoxelSize,
		bool regenerateClouds,
		int regenerateDecimation,
		float regenerateVoxelSize,
		float regenerateMaxDepth) const
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	int i=0;
	int count = 0;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		bool inserted = false;
		if(!iter->second.isNull())
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			if(regenerateClouds)
			{
				if(_cachedSignatures.contains(iter->first))
				{
					const Signature & s = _cachedSignatures.find(iter->first).value();
					SensorData d = s.sensorData();
					cv::Mat image, depth;
					d.uncompressData(&image, &depth, 0);

					if(!image.empty() && !depth.empty())
					{
						UASSERT(iter->first == d.id());
						cloud = util3d::cloudRGBFromSensorData(
								d,
								regenerateDecimation,
								regenerateMaxDepth,
								regenerateVoxelSize);
						if(cloud->size())
						{
							cloud = util3d::transformPointCloud(cloud, iter->second);
						}
					}
					else if(s.getWords3().size())
					{
						cloud->resize(s.getWords3().size());
						int oi=0;
						for(std::multimap<int, pcl::PointXYZ>::const_iterator jter=s.getWords3().begin(); jter!=s.getWords3().end(); ++jter)
						{
							(*cloud)[oi].x = jter->second.x;
							(*cloud)[oi].y = jter->second.y;
							(*cloud)[oi].z = jter->second.z;
							(*cloud)[oi].r = 255;
							(*cloud)[oi].g = 255;
							(*cloud)[oi++].b = 255;
						}
					}
				}
				else
				{
					UWARN("Cloud %d not found in cache!", iter->first);
				}
			}
			else if(uContains(_createdClouds, iter->first))
			{
				cloud = util3d::transformPointCloud(_createdClouds.at(iter->first), iter->second);
			}

			if(cloud->size())
			{
				*assembledCloud += *cloud;

				inserted = true;
				++count;
			}
		}
		else
		{
			UERROR("transform is null!?");
		}

		if(inserted)
		{
			_initProgressDialog->appendText(tr("Generated cloud %1 (%2/%3).").arg(iter->first).arg(++i).arg(poses.size()));

			if(count % 100 == 0)
			{
				if(assembledCloud->size() && assembledVoxelSize)
				{
					assembledCloud = util3d::voxelize(assembledCloud, assembledVoxelSize);
				}
			}
		}
		else
		{
			_initProgressDialog->appendText(tr("Ignored cloud %1 (%2/%3).").arg(iter->first).arg(++i).arg(poses.size()));
		}
		_initProgressDialog->incrementStep();
		QApplication::processEvents();
	}

	if(assembledCloud->size() && assembledVoxelSize)
	{
		assembledCloud = util3d::voxelize(assembledCloud, assembledVoxelSize);
	}

	return assembledCloud;
}

std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > MainWindow::getClouds(
		const std::map<int, Transform> & poses,
		bool regenerateClouds,
		int regenerateDecimation,
		float regenerateVoxelSize,
		float regenerateMaxDepth) const
{
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	int i=0;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		bool inserted = false;
		if(!iter->second.isNull())
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			if(regenerateClouds)
			{
				if(_cachedSignatures.contains(iter->first))
				{
					const Signature & s = _cachedSignatures.find(iter->first).value();
					SensorData d = s.sensorData();
					cv::Mat image, depth;
					d.uncompressData(&image, &depth, 0);
					if(!image.empty() && !depth.empty())
					{
						UASSERT(iter->first == d.id());
						cloud =	 util3d::cloudRGBFromSensorData(
							d,
							regenerateDecimation,
							regenerateMaxDepth,
							regenerateVoxelSize);
					}
					else if(s.getWords3().size())
					{
						cloud->resize(s.getWords3().size());
						int oi=0;
						for(std::multimap<int, pcl::PointXYZ>::const_iterator jter=s.getWords3().begin(); jter!=s.getWords3().end(); ++jter)
						{
							(*cloud)[oi].x = jter->second.x;
							(*cloud)[oi].y = jter->second.y;
							(*cloud)[oi].z = jter->second.z;
							(*cloud)[oi].r = 255;
							(*cloud)[oi].g = 255;
							(*cloud)[oi++].b = 255;
						}
					}
				}
				else
				{
					UERROR("Cloud %d not found in cache!", iter->first);
				}
			}
			else if(uContains(_createdClouds, iter->first))
			{
				cloud = _createdClouds.at(iter->first);
			}

			if(cloud->size())
			{
				clouds.insert(std::make_pair(iter->first, cloud));
				inserted = true;
			}
		}
		else
		{
			UERROR("transform is null!?");
		}

		if(inserted)
		{
			_initProgressDialog->appendText(tr("Generated cloud %1 (%2/%3).").arg(iter->first).arg(++i).arg(poses.size()));
		}
		else
		{
			_initProgressDialog->appendText(tr("Ignored cloud %1 (%2/%3).").arg(iter->first).arg(++i).arg(poses.size()));
		}
		_initProgressDialog->incrementStep();
		QApplication::processEvents();
	}

	return clouds;
}

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
	_ui->actionNew_database->setVisible(!monitoring);
	_ui->actionOpen_database->setVisible(!monitoring);
	_ui->actionClose_database->setVisible(!monitoring);
	_ui->actionEdit_database->setVisible(!monitoring);
	_ui->actionStart->setVisible(!monitoring);
	_ui->actionStop->setVisible(!monitoring);
	_ui->actionDump_the_memory->setVisible(!monitoring);
	_ui->actionDump_the_prediction_matrix->setVisible(!monitoring);
	_ui->actionGenerate_map->setVisible(!monitoring);
	_ui->actionGenerate_local_map->setVisible(!monitoring);
	_ui->actionGenerate_TORO_graph_graph->setVisible(!monitoring);
	_ui->actionExport_poses_txt->setVisible(!monitoring);
	_ui->actionOpen_working_directory->setVisible(!monitoring);
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
	if(actions.size()>=9)
	{
		if(actions.at(2)->isSeparator())
		{
			actions.at(2)->setVisible(!monitoring);
		}
		else
		{
			UWARN("Menu File separators have not the same order.");
		}
		if(actions.at(8)->isSeparator())
		{
			actions.at(8)->setVisible(!monitoring);
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
		_ui->actionGenerate_map->setEnabled(false);
		_ui->actionGenerate_local_map->setEnabled(false);
		_ui->actionGenerate_TORO_graph_graph->setEnabled(false);
		_ui->actionExport_poses_txt->setEnabled(false);
		_ui->actionDownload_all_clouds->setEnabled(false);
		_ui->actionDownload_graph->setEnabled(false);
		_ui->menuSelect_source->setEnabled(false);
		_ui->toolBar->findChild<QAction*>("toolbar_source")->setEnabled(false);
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
		_ui->actionGenerate_map->setEnabled(true);
		_ui->actionGenerate_local_map->setEnabled(true);
		_ui->actionGenerate_TORO_graph_graph->setEnabled(true);
		_ui->actionExport_poses_txt->setEnabled(true);
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->actionDownload_graph->setEnabled(true);
		_ui->menuSelect_source->setEnabled(true);
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
		_ui->actionGenerate_map->setEnabled(false);
		_ui->actionGenerate_local_map->setEnabled(false);
		_ui->actionGenerate_TORO_graph_graph->setEnabled(false);
		_ui->actionExport_poses_txt->setEnabled(false);
		_ui->actionDownload_all_clouds->setEnabled(false);
		_ui->actionDownload_graph->setEnabled(false);
		_ui->menuSelect_source->setEnabled(false);
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

		if(_dbReader)
		{
			_dbReader->start();
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
			_ui->actionGenerate_map->setEnabled(false);
			_ui->actionGenerate_local_map->setEnabled(false);
			_ui->actionGenerate_TORO_graph_graph->setEnabled(false);
			_ui->actionExport_poses_txt->setEnabled(false);
			_ui->actionDownload_all_clouds->setEnabled(false);
			_ui->actionDownload_graph->setEnabled(false);
			_state = kDetecting;
			_elapsedTime->start();
			_oneSecondTimer->start();

			if(_camera)
			{
				_camera->start();
			}

			if(_dbReader)
			{
				_dbReader->start();
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
			_ui->actionGenerate_map->setEnabled(true);
			_ui->actionGenerate_local_map->setEnabled(true);
			_ui->actionGenerate_TORO_graph_graph->setEnabled(true);
			_ui->actionExport_poses_txt->setEnabled(true);
			_ui->actionDownload_all_clouds->setEnabled(true);
			_ui->actionDownload_graph->setEnabled(true);
			_state = kPaused;
			_oneSecondTimer->stop();

			// kill sensors
			if(_camera)
			{
				_camera->join(true);
			}

			if(_dbReader)
			{
				_dbReader->join(true);
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
		_ui->actionDelete_memory->setEnabled(true);
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->actionDownload_graph->setEnabled(true);
		_ui->actionTrigger_a_new_map->setEnabled(true);
		_ui->statusbar->showMessage(tr("Monitoring..."));
		_state = newState;
		_elapsedTime->start();
		_oneSecondTimer->start();
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPause, "", 0));
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
		_ui->actionDelete_memory->setEnabled(true);
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->actionDownload_graph->setEnabled(true);
		_ui->actionTrigger_a_new_map->setEnabled(true);
		_ui->statusbar->showMessage(tr("Monitoring paused..."));
		_state = newState;
		_oneSecondTimer->stop();
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPause, "", 1));
		break;
	default:
		break;
	}

}

}
