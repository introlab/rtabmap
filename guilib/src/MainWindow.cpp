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

#include "rtabmap/gui/MainWindow.h"

#include "ui_mainWindow.h"

#include "rtabmap/core/Camera.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/ParamEvent.h"
#include "rtabmap/core/Signature.h"

#include "rtabmap/gui/ImageView.h"
#include "rtabmap/gui/KeypointItem.h"

#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include "utilite/UPlot.h"
#include "rtabmap/gui/UCv2Qt.h"

#include "AboutDialog.h"
#include "PdfPlot.h"
#include "StatsToolBox.h"
#include "DetailedProgressDialog.h"

#include <QtGui/QCloseEvent>
#include <QtGui/QPixmap>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtGui/QMessageBox>
#include <QtGui/QFileDialog>
#include <QtGui/QGraphicsEllipseItem>
#include <QtGui/QDockWidget>
#include <QtCore/QBuffer>
#include <QtCore/QTimer>
#include <QtCore/QTime>
#include <QtGui/QActionGroup>
#include <QtCore/QThread>
#include <QtGui/QDesktopServices>
#include <QtCore/QStringList>
#include <QtCore/QProcess>
#include <QtGui/QSplashScreen>
#include <QtGui/QInputDialog>

//RGB-D stuff
#include "rtabmap/core/CameraOpenni.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/util3d.h"
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
	_cameraOpenni(0),
	_odomThread(0),
	_srcType(kSrcUndefined),
	_preferencesDialog(0),
	_aboutDialog(0),
	_lastId(0),
	_processingStatistics(false),
	_odometryReceived(false),
	_odometryCorrection(Transform::getIdentity()),
	_lastOdometryProcessed(true),
	_oneSecondTimer(0),
	_elapsedTime(0),
	_posteriorCurve(0),
	_likelihoodCurve(0),
	_rawLikelihoodCurve(0),
	_autoScreenCaptureOdomSync(false)
{
	ULOGGER_DEBUG("");

	initGuiResource();

	QPixmap pixmap(":images/RTAB-Map.png");
	QSplashScreen splash(pixmap);
	splash.show();
	splash.showMessage(tr("Loading..."));
	QApplication::processEvents();

	// Create dialogs
	_aboutDialog = new AboutDialog(this);

	_ui = new Ui_mainWindow();
	_ui->setupUi(this);

	QString title("RTAB-Map: Real-Time Appearance-Based Mapping");
	this->setWindowTitle(title);
	this->setWindowIconText(tr("RTAB-Map"));

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
	_preferencesDialog->init();

	// Restore window geometry
	_preferencesDialog->loadMainWindowState(this);
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
	_ui->imageView_source->setBackgroundBrush(QBrush(Qt::black));
	_ui->imageView_loopClosure->setBackgroundBrush(QBrush(Qt::black));

	_posteriorCurve = new PdfPlotCurve("Posterior", &_imagesMap, this);
	_ui->posteriorPlot->addCurve(_posteriorCurve, false);
	_ui->posteriorPlot->showLegend(false);
	_ui->posteriorPlot->setFixedYAxis(0,1);
	UPlotCurveThreshold * tc;
	tc = _ui->posteriorPlot->addThreshold("Loop closure thr", float(_preferencesDialog->getLoopThr()));
	connect(this, SIGNAL(loopClosureThrChanged(float)), tc, SLOT(setThreshold(float)));

	_likelihoodCurve = new PdfPlotCurve("Likelihood", &_imagesMap, this);
	_ui->likelihoodPlot->addCurve(_likelihoodCurve, false);
	_ui->likelihoodPlot->showLegend(false);

	_rawLikelihoodCurve = new PdfPlotCurve("Likelihood", &_imagesMap, this);
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
	_ui->menuShow_view->addAction(_ui->toolBar->toggleViewAction());
	_ui->toolBar->setWindowTitle(tr("Control toolbar"));
	QAction * a = _ui->menuShow_view->addAction("Progress dialog");
	a->setCheckable(false);
	connect(a, SIGNAL(triggered(bool)), _initProgressDialog, SLOT(show()));

	// connect actions with custom slots
	connect(_ui->actionStart, SIGNAL(triggered()), this, SLOT(startDetection()));
	connect(_ui->actionPause, SIGNAL(triggered()), this, SLOT(pauseDetection()));
	connect(_ui->actionStop, SIGNAL(triggered()), this, SLOT(stopDetection()));
	connect(_ui->actionDump_the_memory, SIGNAL(triggered()), this, SLOT(dumpTheMemory()));
	connect(_ui->actionDump_the_prediction_matrix, SIGNAL(triggered()), this, SLOT(dumpThePrediction()));
	connect(_ui->actionClear_cache, SIGNAL(triggered()), this, SLOT(clearTheCache()));
	connect(_ui->actionAbout, SIGNAL(triggered()), _aboutDialog , SLOT(exec()));
	connect(_ui->actionPrint_loop_closure_IDs_to_console, SIGNAL(triggered()), this, SLOT(printLoopClosureIds()));
	connect(_ui->actionGenerate_map, SIGNAL(triggered()), this , SLOT(generateMap()));
	connect(_ui->actionGenerate_local_map, SIGNAL(triggered()), this, SLOT(generateLocalMap()));
	connect(_ui->actionGenerate_TORO_graph_graph, SIGNAL(triggered()), this , SLOT(generateTOROMap()));
	connect(_ui->actionDelete_memory, SIGNAL(triggered()), this , SLOT(deleteMemory()));
	connect(_ui->actionDownload_all_clouds, SIGNAL(triggered()), this , SLOT(downloadAllClouds()));
	connect(_ui->actionDownload_graph, SIGNAL(triggered()), this , SLOT(downloadPoseGraph()));
	connect(_ui->menuEdit, SIGNAL(aboutToShow()), this, SLOT(updateEditMenu()));
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
	connect(_ui->actionSave_point_cloud, SIGNAL(triggered()), this, SLOT(savePointClouds()));
	connect(_ui->actionSave_mesh_ply_vtk_stl, SIGNAL(triggered()), this, SLOT(saveMeshes()));
	connect(_ui->actionView_high_res_point_cloud, SIGNAL(triggered()), this, SLOT(viewPointClouds()));
	connect(_ui->actionView_point_cloud_as_mesh, SIGNAL(triggered()), this, SLOT(viewMeshes()));
	connect(_ui->actionReset_Odometry, SIGNAL(triggered()), this, SLOT(resetOdometry()));
	connect(_ui->actionTrigger_a_new_map, SIGNAL(triggered()), this, SLOT(triggerNewMap()));

	_ui->actionPause->setShortcut(Qt::Key_Space);
	_ui->actionSave_point_cloud->setEnabled(false);
	_ui->actionView_high_res_point_cloud->setEnabled(false);
	_ui->actionSave_mesh_ply_vtk_stl->setEnabled(false);
	_ui->actionView_point_cloud_as_mesh->setEnabled(false);
	_ui->actionReset_Odometry->setEnabled(false);

#if defined(Q_WS_MAC) or defined(Q_WS_WIN)
	connect(_ui->actionOpen_working_directory, SIGNAL(triggered()), SLOT(openWorkingDirectory()));
#else
	_ui->menuEdit->removeAction(_ui->actionOpen_working_directory);
#endif

	//Settings menu
	QActionGroup * selectSourceImageGrp = new QActionGroup(this);
	selectSourceImageGrp->addAction(_ui->actionUsbCamera);
	selectSourceImageGrp->addAction(_ui->actionImageFiles);
	selectSourceImageGrp->addAction(_ui->actionVideo);
	selectSourceImageGrp->addAction(_ui->actionOpenni_RGBD);
	this->updateSelectSourceImageMenu(_preferencesDialog->getSourceImageType());
	connect(_ui->actionImageFiles, SIGNAL(triggered()), this, SLOT(selectImages()));
	connect(_ui->actionVideo, SIGNAL(triggered()), this, SLOT(selectVideo()));
	connect(_ui->actionUsbCamera, SIGNAL(triggered()), this, SLOT(selectStream()));
	this->updateSelectSourceDatabase(_preferencesDialog->isSourceDatabaseUsed());
	connect(_ui->actionDatabase, SIGNAL(triggered()), this, SLOT(selectDatabase()));
	this->updateSelectSourceOpenni(_preferencesDialog->isSourceOpenniUsed());
	connect(_ui->actionOpenni_RGBD, SIGNAL(triggered()), this, SLOT(selectOpenni()));

	connect(_ui->actionSave_state, SIGNAL(triggered()), this, SLOT(saveFigures()));
	connect(_ui->actionLoad_state, SIGNAL(triggered()), this, SLOT(loadFigures()));

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
	connect(_ui->actionApply_settings_to_the_detector, SIGNAL(triggered()), this, SLOT(applyAllPrefSettings()));

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

	qRegisterMetaType<rtabmap::Image>("rtabmap::Image");
	connect(this, SIGNAL(odometryReceived(rtabmap::Image)), this, SLOT(processOdometry(rtabmap::Image)));

	connect(this, SIGNAL(noMoreImagesReceived()), this, SLOT(stopDetection()));

	// Apply state
	this->changeState(kIdle);
	this->applyPrefSettings(PreferencesDialog::kPanelAll);

	_ui->statsToolBox->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_ui->graphicsView_graphView->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());

	splash.close();
}

MainWindow::~MainWindow()
{
	UDEBUG("");
	if(_state != kIdle)
	{
		this->stopDetection();
	}
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
		if(_state != kIdle)
		{
			processStopped = false;
		}
	}

	if(processStopped)
	{
		_ui->statsToolBox->closeFigures();

		//write settings before quit?
		_preferencesDialog->saveMainWindowState(this);

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
		if(_cameraOpenni)
		{
			UERROR("CameraOpenni must be already deleted here!");
			delete _cameraOpenni;
			_cameraOpenni = 0;
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
		int localLoopClosureId = int(uValue(stats.data(), Statistics::kLocalLoopSpace_closure_id(), 0.0f));
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
		UDEBUG("stat.rawLikelihood().size()=%d", stats.rawLikelihood().size());
		// Performance issue: don't process the pdf and likelihood if the last event is
		// not yet completely processed, to avoid an unresponsive GUI when events accumulate.
		if(_processingStatistics || !_ui->dockWidget_posterior->isVisible())
		{
			stats.setPosterior(std::map<int, float>());
		}
		if(_processingStatistics || !_ui->dockWidget_likelihood->isVisible())
		{
			stats.setLikelihood(std::map<int, float>());
		}
		if(_processingStatistics || !_ui->dockWidget_rawlikelihood->isVisible())
		{
			stats.setRawLikelihood(std::map<int, float>());
		}
		if(_processingStatistics || (!_ui->dockWidget_posterior->isVisible() && !_ui->dockWidget_likelihood->isVisible()))
		{
			stats.setWeights(std::map<int,int>());
		}

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
		OdometryEvent * odomEvent = (OdometryEvent*)anEvent;
		if(_ui->dockWidget_cloudViewer->isVisible() &&
		   _preferencesDialog->isCloudsShown(1) &&
		   _lastOdometryProcessed &&
		   !_processingStatistics)
		{
			_lastOdometryProcessed = false; // if we receive too many odometry events!
			emit odometryReceived(odomEvent->data());
		}
	}
	else if(anEvent->getClassName().compare("ULogEvent") == 0)
	{
		ULogEvent * logEvent = (ULogEvent*)anEvent;
		if(logEvent->getCode() >= _preferencesDialog->getGeneralLoggerPauseLevel())
		{
			QMetaObject::invokeMethod(_ui->dockWidget_console, "show");
			// The timer prevents multiple calls to pauseDetection() before the state can be changed
			if(_state != kPaused && _logEventTime->elapsed() > 1000)
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

void MainWindow::processOdometry(const rtabmap::Image & data)
{
	Transform pose = data.pose();
	if(pose.isNull())
	{
		UDEBUG("odom lost"); // use last pose
		_ui->widget_cloudViewer->setBackgroundColor(Qt::darkRed);

		pose = _lastOdomPose;
	}
	else
	{
		UDEBUG("odom ok");
		_ui->widget_cloudViewer->setBackgroundColor(Qt::black);
	}
	if(!pose.isNull())
	{
		_lastOdomPose = pose;
		_odometryReceived = true;

		// 3d cloud
		if(data.depth().cols == data.image().cols &&
		   data.depth().rows == data.image().rows &&
		   !data.depth().empty() &&
		   data.depthConstant() > 0.0f &&
		   _preferencesDialog->isCloudsShown(1))
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
			cloud = createCloud(0,
					data.image(),
					data.depth(),
					data.depthConstant(),
					data.localTransform(),
					pose,
					_preferencesDialog->getCloudVoxelSize(1),
					_preferencesDialog->getCloudDecimation(1),
					_preferencesDialog->getCloudMaxDepth(1));

			if(!_ui->widget_cloudViewer->addOrUpdateCloud("cloudOdom", cloud, _odometryCorrection))
			{
				UERROR("Adding cloudOdom to viewer failed!");
			}
			_ui->widget_cloudViewer->setCloudVisibility("cloudOdom", true);
			_ui->widget_cloudViewer->setCloudOpacity("cloudOdom", _preferencesDialog->getCloudOpacity(1));
			_ui->widget_cloudViewer->setCloudPointSize("cloudOdom", _preferencesDialog->getCloudPointSize(1));
		}

		// 2d cloud
		if(!data.depth2d().empty() &&
			_preferencesDialog->isScansShown(1))
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
			cloud = util3d::depth2DToPointCloud(data.depth2d());
			cloud = util3d::transformPointCloud(cloud, pose);
			if(!_ui->widget_cloudViewer->addOrUpdateCloud("scanOdom", cloud, _odometryCorrection))
			{
				UERROR("Adding scanOdom to viewer failed!");
			}
			_ui->widget_cloudViewer->setCloudVisibility("scanOdom", true);
			_ui->widget_cloudViewer->setCloudOpacity("scanOdom", _preferencesDialog->getScanOpacity(1));
			_ui->widget_cloudViewer->setCloudPointSize("scanOdom", _preferencesDialog->getScanPointSize(1));
		}

		if(!data.pose().isNull())
		{
			// update camera position
			_ui->widget_cloudViewer->updateCameraPosition(_odometryCorrection*data.pose());
		}
	}
	_ui->widget_cloudViewer->render();

	_lastOdometryProcessed = true;

	if(_ui->actionAuto_screen_capture->isChecked() && _autoScreenCaptureOdomSync)
	{
		this->captureScreen();
	}
}

void MainWindow::processStats(const rtabmap::Statistics & stat)
{
	_processingStatistics = true;
	ULOGGER_DEBUG("");
	QTime time, totalTime;
	time.start();
	totalTime.start();
	//Affichage des stats et images

	int refMapId = uValue(stat.getMapIds(), stat.refImageId(), -1);
	int loopMapId = uValue(stat.getMapIds(), stat.loopClosureId(), uValue(stat.getMapIds(), stat.localLoopClosureId(), -1));

	_ui->label_refId->setText(QString("New ID = %1 [%2]").arg(stat.refImageId()).arg(refMapId));
	_ui->label_matchId->clear();

	if(stat.extended())
	{
		float totalTime = static_cast<float>(uValue(stat.data(), Statistics::kTimingTotal(), 0.0f));
		if(totalTime/1000.0f > float(1.0/_preferencesDialog->getDetectionRate()))
		{
			UWARN("Processing time (%fs) is over detection rate (%fs), real-time problem!", totalTime/1000.0f, 1.0/_preferencesDialog->getDetectionRate());
		}

		UDEBUG("");
		int highestHypothesisId = static_cast<float>(uValue(stat.data(), Statistics::kLoopHighest_hypothesis_id(), 0.0f));
		bool highestHypothesisIsSaved = (bool)uValue(stat.data(), Statistics::kLoopHypothesis_reactivated(), 0.0f);

		// Loop closure info
		_ui->imageView_source->clear();
		_ui->imageView_loopClosure->clear();
		_ui->imageView_source->resetTransform();
		_ui->imageView_loopClosure->resetTransform();
		_ui->imageView_source->setBackgroundBrush(QBrush(Qt::black));
		_ui->imageView_loopClosure->setBackgroundBrush(QBrush(Qt::black));

		// update cache
		if(_preferencesDialog->isImagesKept())
		{
			// images
			for(std::map<int, std::vector<unsigned char> >::const_iterator iter = stat.getImages().begin();
				iter != stat.getImages().end();
				++iter)
			{
				if(!iter->second.empty() && !_imagesMap.contains(iter->first))
				{
					_imagesMap.insert(iter->first, iter->second);
				}
			}
			// depths
			for(std::map<int, std::vector<unsigned char> >::const_iterator iter = stat.getDepths().begin();
				iter != stat.getDepths().end();
				++iter)
			{
				if(!iter->second.empty() && !_depthsMap.contains(iter->first))
				{
					float constant = uValue(stat.getDepthConstants(), iter->first, 0.0f);
					Transform transform = uValue(stat.getLocalTransforms(), iter->first, Transform());
					if(constant != 0.0f && !transform.isNull())
					{
						_depthsMap.insert(iter->first, iter->second);
						_depthConstantsMap.insert(iter->first, constant);
						_localTransformsMap.insert(iter->first, transform);
					}
					else
					{
						UERROR("Invalid depth data for id=%d", iter->first);
					}
				}
			}
			// depths2d
			for(std::map<int, std::vector<unsigned char> >::const_iterator iter = stat.getDepth2ds().begin();
				iter != stat.getDepth2ds().end();
				++iter)
			{
				if(!iter->second.empty())
				{
					_depths2DMap.insert(std::make_pair(iter->first, iter->second));
				}
			}
		}

		int rehearsed = (int)uValue(stat.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
		int localTimeClosures = (int)uValue(stat.data(), Statistics::kLocalLoopTime_closures(), 0.0f);
		bool scanMatchingSuccess = (bool)uValue(stat.data(), Statistics::kLocalLoopOdom_corrected(), 0.0f);
		_ui->label_matchId->clear();
		_ui->label_stats_imageNumber->setText(QString("%1 [%2]").arg(stat.refImageId()).arg(refMapId));

		if(rehearsed > 0)
		{
			_ui->imageView_source->setBackgroundBrush(QBrush(Qt::blue));
		}
		else if(localTimeClosures > 0)
		{
			_ui->imageView_source->setBackgroundBrush(QBrush(Qt::darkCyan));
		}
		else if(scanMatchingSuccess)
		{
			_ui->imageView_source->setBackgroundBrush(QBrush(Qt::gray));
		}

		UDEBUG("time= %d ms", time.restart());

		std::vector<unsigned char> refImage = uValue(stat.getImages(), stat.refImageId(), std::vector<unsigned char>());
		std::vector<unsigned char> refDepth = uValue(stat.getDepths(), stat.refImageId(), std::vector<unsigned char>());
		std::vector<unsigned char> refDepth2D = uValue(stat.getDepth2ds(), stat.refImageId(), std::vector<unsigned char>());
		std::vector<unsigned char> loopImage = uValue(stat.getImages(), stat.loopClosureId()>0?stat.loopClosureId():stat.localLoopClosureId(), std::vector<unsigned char>());
		std::vector<unsigned char> loopDepth = uValue(stat.getDepths(), stat.loopClosureId()>0?stat.loopClosureId():stat.localLoopClosureId(), std::vector<unsigned char>());
		std::vector<unsigned char> loopDepth2D = uValue(stat.getDepth2ds(), stat.loopClosureId()>0?stat.loopClosureId():stat.localLoopClosureId(), std::vector<unsigned char>());

		int rejectedHyp = bool(uValue(stat.data(), Statistics::kLoopRejectedHypothesis(), 0.0f));
		float highestHypothesisValue = uValue(stat.data(), Statistics::kLoopHighest_hypothesis_value(), 0.0f);
		int matchId = 0;
		if(highestHypothesisId > 0 || stat.localLoopClosureId()>0)
		{
			bool show = true;
			if(stat.loopClosureId() > 0)
			{
				_ui->imageView_loopClosure->setBackgroundBrush(QBrush(Qt::green));
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
				_ui->imageView_loopClosure->setBackgroundBrush(QBrush(Qt::yellow));
				_ui->label_matchId->setText(QString("Local match = %1 [%2]").arg(stat.localLoopClosureId()).arg(loopMapId));
				matchId = stat.localLoopClosureId();
			}
			else if(rejectedHyp && highestHypothesisValue >= _preferencesDialog->getLoopThr())
			{
				show = _preferencesDialog->imageRejectedShown() || _preferencesDialog->imageHighestHypShown();
				if(show)
				{
					_ui->imageView_loopClosure->setBackgroundBrush(QBrush(Qt::red));
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
				if(loopImage.empty())
				{
					int id = stat.loopClosureId()>0?stat.loopClosureId():stat.localLoopClosureId()>0?stat.localLoopClosureId():highestHypothesisId;
					QMap<int, std::vector<unsigned char> >::iterator iter = _imagesMap.find(id);
					if(iter != _imagesMap.end())
					{
						loopImage = iter.value();
					}
				}
				if(loopDepth.empty())
				{
					int id = stat.loopClosureId()>0?stat.loopClosureId():stat.localLoopClosureId()>0?stat.localLoopClosureId():highestHypothesisId;
					QMap<int, std::vector<unsigned char> >::iterator iter = _depthsMap.find(id);
					if(iter != _depthsMap.end())
					{
						loopDepth = iter.value();
					}
				}
			}
		}
		_refIds.push_back(stat.refImageId());
		_loopClosureIds.push_back(matchId);

		//update image views
		{
			util3d::CompressionThread imageThread(refImage, true);
			util3d::CompressionThread imageLoopThread(loopImage, true);
			util3d::CompressionThread depthThread(refDepth, true);
			util3d::CompressionThread depthLoopThread(loopDepth, true);
			imageThread.start();
			depthThread.start();
			imageLoopThread.start();
			depthLoopThread.start();
			imageThread.join();
			depthThread.join();
			imageLoopThread.join();
			depthLoopThread.join();
			UDEBUG("time= %d ms", time.restart());

			UCvMat2QImageThread qimageThread(imageThread.getUncompressedData());
			UCvMat2QImageThread qimageLoopThread(imageLoopThread.getUncompressedData());
			UCvMat2QImageThread qdepthThread(depthThread.getUncompressedData());
			UCvMat2QImageThread qdepthLoopThread(depthLoopThread.getUncompressedData());
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
			QRectF sceneRect = img.rect();
			_ui->imageView_source->setSceneRect(sceneRect);
			_ui->imageView_loopClosure->setSceneRect(sceneRect);
		}

		UDEBUG("time= %d ms", time.restart());

		// We use the reference image to resize the 2 views
		_ui->imageView_source->resetZoom();
		_ui->imageView_loopClosure->resetZoom();
		if(refImage.empty())
		{
			_ui->imageView_source->setSceneRect(_ui->imageView_source->scene()->itemsBoundingRect());
			_ui->imageView_loopClosure->setSceneRect(_ui->imageView_source->scene()->itemsBoundingRect());
		}
		_ui->imageView_source->fitInView(_ui->imageView_source->sceneRect(), Qt::KeepAspectRatio);
		_ui->imageView_loopClosure->fitInView(_ui->imageView_source->sceneRect(), Qt::KeepAspectRatio);

		// do it after scaling
		if(_ui->imageView_loopClosure->items().size() || stat.loopClosureId()>0)
		{
			this->drawKeypoints(stat.refWords(), stat.loopWords());
		}
		else
		{
			this->drawKeypoints(stat.refWords(), std::multimap<int, cv::KeyPoint>()); //empty loop keypoints...
		}

		if(_preferencesDialog->isImageFlipped())
		{
			_ui->imageView_source->scale(-1.0, 1.0);
			_ui->imageView_loopClosure->scale(-1.0, 1.0);
		}

		UDEBUG("time= %d ms", time.restart());

		_ui->statsToolBox->updateStat("Keypoint/Keypoints count in the last signature/", stat.refImageId(), stat.refWords().size());
		_ui->statsToolBox->updateStat("Keypoint/Keypoints count in the loop signature/", stat.refImageId(), stat.loopWords().size());
		ULOGGER_DEBUG("");

		// PDF AND LIKELIHOOD
		if(!stat.posterior().empty() && _ui->dockWidget_posterior->isVisible())
		{
			UDEBUG("");
			if(stat.weights().size() != stat.posterior().size())
			{
				UWARN("%d %d", stat.weights().size(), stat.posterior().size());
			}
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
			// update pose only if a odometry is not received
			updateMapCloud(stat.poses(), _odometryReceived?Transform():stat.currentPose());

			// update some widgets
			_ui->widget_mapVisibility->setMap(stat.poses());
			if(_ui->graphicsView_graphView->isVisible())
			{
				_ui->graphicsView_graphView->updateGraph(stat.poses(), stat.constraints(), _depths2DMap);
			}

			_odometryReceived = false;

			_odometryCorrection = stat.mapCorrection();

			UDEBUG("time= %d ms", time.restart());
			_ui->statsToolBox->updateStat("/Gui RGB-D cloud/ms", stat.refImageId(), int(timerVis.elapsed()*1000.0f));

			// loop closure view
			if((stat.loopClosureId() > 0 || stat.localLoopClosureId() > 0)  &&
			   !stat.loopClosureTransform().isNull())
			{
				// the last loop closure data
				Transform loopClosureTransform = stat.loopClosureTransform();
				int loopOldId = stat.loopClosureId();
				if(!loopOldId)
				{
					loopOldId = stat.localLoopClosureId();
				}
				int loopNewId = stat.refImageId();

				// Add to loop closure viewer if all data is saved
				Signature * loopOld = new Signature(
						loopOldId,
						loopMapId,
						std::multimap<int, cv::KeyPoint>(),
						std::multimap<int, pcl::PointXYZ>(),
						Transform(),
						uValue(_depths2DMap, loopOldId, std::vector<unsigned char>()),
						_imagesMap.value(loopOldId, std::vector<unsigned char>()),
						_depthsMap.value(loopOldId, std::vector<unsigned char>()),
						_depthConstantsMap.value(loopOldId, 0.0f),
						_localTransformsMap.value(loopOldId, Transform()));

				Signature * loopNew = new Signature(
						loopNewId,
						refMapId,
						std::multimap<int, cv::KeyPoint>(),
						std::multimap<int, pcl::PointXYZ>(),
						loopClosureTransform,
						uValue(_depths2DMap, loopNewId, std::vector<unsigned char>()),
						_imagesMap.value(loopNewId, std::vector<unsigned char>()),
						_depthsMap.value(loopNewId, std::vector<unsigned char>()),
						_depthConstantsMap.value(loopNewId, 0.0f),
						_localTransformsMap.value(loopNewId, Transform()));

				_ui->widget_loopClosureViewer->setData(loopOld, loopNew);
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
		UDEBUG("");
	}
	else if(!stat.extended() && stat.loopClosureId()>0)
	{
		_ui->label_stats_loopClosuresDetected->setText(QString::number(_ui->label_stats_loopClosuresDetected->text().toInt() + 1));
		_ui->label_matchId->setText(QString("Match ID = %1 [%2]").arg(stat.loopClosureId()).arg(loopMapId));
	}
	float elapsedTime = static_cast<float>(totalTime.elapsed());
	UINFO("Updating GUI time = %fs", elapsedTime/1000.0f);
	_ui->statsToolBox->updateStat("/Gui refresh stats/ms", stat.refImageId(), elapsedTime);
	if(_ui->actionAuto_screen_capture->isChecked() && !_autoScreenCaptureOdomSync)
	{
		this->captureScreen();
	}
	_processingStatistics = false;
}

void MainWindow::updateMapCloud(const std::map<int, Transform> & posesIn, const Transform & currentPose)
{
	if(posesIn.size())
	{
		_currentPosesMap = posesIn;
		if(_currentPosesMap.size() && !_ui->actionSave_point_cloud->isEnabled())
		{
			//enable save cloud action
			_ui->actionSave_point_cloud->setEnabled(true);
			_ui->actionView_high_res_point_cloud->setEnabled(true);
			_ui->actionSave_mesh_ply_vtk_stl->setEnabled(true);
			_ui->actionView_point_cloud_as_mesh->setEnabled(true);
		}
	}

	// filter duplicated poses
	std::map<int, Transform> poses;
	if(_preferencesDialog->isCloudFiltering())
	{
		poses = radiusPosesFiltering(posesIn);
	}
	else
	{
		poses = posesIn;
	}

	// Map updated! regenerate the assembled cloud, last pose is the new one
	UDEBUG("Update map with %d locations (currentPose=%s)", poses.size(), currentPose.prettyPrint().c_str());
	QMap<std::string, Transform> viewerClouds = _ui->widget_cloudViewer->getAddedClouds();
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(!iter->second.isNull())
		{
			std::string cloudName = uFormat("cloud%d", iter->first);

			// 3d point cloud
			if(_preferencesDialog->isCloudsShown(0))
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
				else if(_imagesMap.contains(iter->first) && _depthsMap.contains(iter->first))
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
					cloud = createCloud(iter->first,
							util3d::uncompressImage(_imagesMap.value(iter->first)),
							util3d::uncompressImage(_depthsMap.value(iter->first)),
							_depthConstantsMap.value(iter->first),
							_localTransformsMap.value(iter->first),
							Transform::getIdentity(),
							_preferencesDialog->getCloudVoxelSize(0),
							_preferencesDialog->getCloudDecimation(0),
							_preferencesDialog->getCloudMaxDepth(0));

					if(_preferencesDialog->isCloudMeshing(0))
					{
						pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
						if(cloud->size())
						{
							mesh = util3d::createMesh(cloud, _preferencesDialog->getCloudVoxelSize(0)==0?0.025:_preferencesDialog->getCloudVoxelSize(0)*4);
						}

						if(mesh->polygons.size())
						{
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
							pcl::fromPCLPointCloud2(mesh->cloud, *tmp);
							if(!_ui->widget_cloudViewer->addCloudMesh(cloudName, tmp, mesh->polygons, iter->second))
							{
								UERROR("Adding mesh cloud %d to viewer failed!", iter->first);
							}

						}
					}
					else if(!_ui->widget_cloudViewer->addOrUpdateCloud(cloudName, cloud, iter->second))
					{
						UERROR("Adding cloud %d to viewer failed!", iter->first);
					}

					_ui->widget_cloudViewer->setCloudOpacity(cloudName, _preferencesDialog->getCloudOpacity(0));
					_ui->widget_cloudViewer->setCloudPointSize(cloudName, _preferencesDialog->getCloudPointSize(0));
				}
			}
			else if(viewerClouds.contains(cloudName))
			{
				UDEBUG("Hide cloud %s", cloudName.c_str());
				_ui->widget_cloudViewer->setCloudVisibility(cloudName.c_str(), false);
			}

			// 2d point cloud
			std::string scanName = uFormat("scan%d", iter->first);
			if(_preferencesDialog->isScansShown(0))
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
				else if(uContains(_depths2DMap, iter->first))
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
					cv::Mat depth2d = util3d::uncompressData(_depths2DMap.at(iter->first));
					cloud = util3d::depth2DToPointCloud(depth2d);
					if(!_ui->widget_cloudViewer->addOrUpdateCloud(scanName, cloud, iter->second))
					{
						UERROR("Adding cloud %d to viewer failed!", iter->first);
					}
					_ui->widget_cloudViewer->setCloudOpacity(scanName, _preferencesDialog->getScanOpacity(0));
					_ui->widget_cloudViewer->setCloudPointSize(scanName, _preferencesDialog->getScanPointSize(0));
				}
			}
			else if(viewerClouds.contains(scanName))
			{
				UDEBUG("Hide scan %s", scanName.c_str());
				_ui->widget_cloudViewer->setCloudVisibility(scanName.c_str(), false);
			}
		}
		else
		{
			UERROR("transform is null!?");
		}
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
		_ui->widget_cloudViewer->updateCameraPosition(currentPose);
	}

	_ui->widget_cloudViewer->render();
}

void MainWindow::updateNodeVisibility(int nodeId, bool visible)
{
	if(_currentPosesMap.find(nodeId) != _currentPosesMap.end())
	{
		if(_preferencesDialog->isCloudsShown(0))
		{
			std::string cloudName = uFormat("cloud%d", nodeId);
			_ui->widget_cloudViewer->setCloudVisibility(cloudName, visible);
		}

		if(_preferencesDialog->isScansShown(0))
		{
			std::string scanName = uFormat("scan%d", nodeId);
			_ui->widget_cloudViewer->setCloudVisibility(scanName, visible);
		}
	}
	_ui->widget_cloudViewer->render();
}

std::map<int, Transform> MainWindow::radiusPosesFiltering(const std::map<int, Transform> & poses) const
{
	float radius = _preferencesDialog->getCloudFilteringRadius();
	float angle = _preferencesDialog->getCloudFilteringAngle()*3.14159265359/180.0; // convert to rad
	if(poses.size() > 1 && radius > 0.0f && angle>0.0f)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(poses.size());
		int i=0;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			(*cloud)[i++] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
		}

		// radius filtering
		std::vector<int> names = uKeys(poses);
		std::vector<Transform> transforms = uValues(poses);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> (false));
		tree->setInputCloud(cloud);
		std::set<int> indicesChecked;
		std::set<int> indicesKept;

		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			// ignore scans
			if(indicesChecked.find(i) == indicesChecked.end())
			{
				std::vector<int> kIndices;
				std::vector<float> kDistances;
				tree->radiusSearch(cloud->at(i), radius, kIndices, kDistances);

				std::set<int> cloudIndices;
				const Transform & currentT = transforms.at(i);
				Eigen::Vector3f vA = util3d::transformToEigen3f(currentT).rotation()*Eigen::Vector3f(1,0,0);
				for(unsigned int j=0; j<kIndices.size(); ++j)
				{
					if(indicesChecked.find(kIndices[j]) == indicesChecked.end())
					{
						const Transform & checkT = transforms.at(kIndices[j]);
						// same orientation?
						Eigen::Vector3f vB = util3d::transformToEigen3f(checkT).rotation()*Eigen::Vector3f(1,0,0);
						double a = pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
						if(a <= angle)
						{
							cloudIndices.insert(kIndices[j]);
						}
					}
				}

				bool firstAdded = false;
				for(std::set<int>::reverse_iterator iter = cloudIndices.rbegin(); iter!=cloudIndices.rend(); ++iter)
				{
					if(!firstAdded)
					{
						indicesKept.insert(*iter);
						firstAdded = true;
					}
					indicesChecked.insert(*iter);
				}
			}
		}

		//pcl::IndicesPtr indicesOut(new std::vector<int>);
		//indicesOut->insert(indicesOut->end(), indicesKept.begin(), indicesKept.end());
		UINFO("Cloud filtered In = %d, Out = %d", cloud->size(), indicesKept.size());
		//pcl::io::savePCDFile("duplicateIn.pcd", *cloud);
		//pcl::io::savePCDFile("duplicateOut.pcd", *cloud, *indicesOut);

		std::map<int, Transform> keptPoses;
		for(std::set<int>::iterator iter = indicesKept.begin(); iter!=indicesKept.end(); ++iter)
		{
			keptPoses.insert(std::make_pair(names.at(*iter), transforms.at(*iter)));
		}

		return keptPoses;
	}
	else
	{
		return poses;
	}
}

void MainWindow::processRtabmapEventInit(int status, const QString & info)
{
	if((RtabmapEventInit::Status)status == RtabmapEventInit::kInitializing)
	{
		if(_state == kDetecting)
		{
			this->pauseDetection();
		}
		_initProgressDialog->setAutoClose(true, 1);
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
	}
	else if((RtabmapEventInit::Status)status == RtabmapEventInit::kInitialized)
	{
		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
	}
	else
	{
		_initProgressDialog->incrementStep();
		QString msg(info);
		if((RtabmapEventInit::Status)status == RtabmapEventInit::kError)
		{
			_initProgressDialog->setAutoClose(false);
			msg.prepend(tr("[ERROR] "));
		}
		_initProgressDialog->appendText(msg);
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
		UINFO(" images = %d", event.getImages().size());
		UINFO(" depths = %d", event.getDepths().size());
		UINFO(" depths2d = %d", event.getDepths2d().size());
		UINFO(" depthConstants = %d", event.getDepthConstants().size());
		UINFO(" localTransforms = %d", event.getLocalTransforms().size());
		UINFO(" poses = %d", event.getPoses().size());
		UINFO(" constraints = %d", event.getConstraints().size());

		_initProgressDialog->appendText("Inserting data in the cache...");

		for(std::map<int, std::vector<unsigned char> >::const_iterator iter = event.getImages().begin();
			iter!=event.getImages().end();
			++iter)
		{
			_imagesMap.insert(iter->first, iter->second);
		}
		_initProgressDialog->appendText(tr("Inserted %1 images.").arg(_imagesMap.size()));
		_initProgressDialog->incrementStep();

		for(std::map<int, std::vector<unsigned char> >::const_iterator iter = event.getDepths().begin();
			iter!=event.getDepths().end();
			++iter)
		{
			_depthsMap.insert(iter->first, iter->second);
		}
		_initProgressDialog->appendText(tr("Inserted %1 depth images.").arg(_depthsMap.size()));
		_initProgressDialog->incrementStep();

		for(std::map<int, float>::const_iterator iter = event.getDepthConstants().begin();
			iter!=event.getDepthConstants().end();
			++iter)
		{
			_depthConstantsMap.insert(iter->first, iter->second);
		}
		_initProgressDialog->appendText(tr("Inserted %1 depth constants.").arg(_depthConstantsMap.size()));
		_initProgressDialog->incrementStep();

		for(std::map<int, std::vector<unsigned char> >::const_iterator iter = event.getDepths2d().begin();
			iter!=event.getDepths2d().end();
			++iter)
		{
			_depths2DMap.insert(std::make_pair(iter->first, iter->second));
		}
		_initProgressDialog->appendText(tr("Inserted %1 laser scans.").arg(_depths2DMap.size()));
		_initProgressDialog->incrementStep();

		for(std::map<int, Transform>::const_iterator iter = event.getLocalTransforms().begin();
			iter!=event.getLocalTransforms().end();
			++iter)
		{
			_localTransformsMap.insert(iter->first, iter->second);
		}
		_initProgressDialog->appendText(tr("Inserted %1 local transforms.").arg(_localTransformsMap.size()));
		_initProgressDialog->incrementStep();

		_odometryCorrection.setIdentity();

		_initProgressDialog->appendText("Inserting data in the cache... done.");

		if(event.getPoses().size())
		{
			_initProgressDialog->appendText("Updating the 3D map cloud...");
			_initProgressDialog->incrementStep();
			QApplication::processEvents();
			this->updateMapCloud(event.getPoses(), Transform());
			_initProgressDialog->appendText("Updating the 3D map cloud... done.");

			if(_ui->graphicsView_graphView->isVisible())
			{
				_initProgressDialog->appendText("Updating the graph view...");
				_initProgressDialog->incrementStep();
				_ui->graphicsView_graphView->updateGraph(
						event.getPoses(),
						event.getConstraints(),
						event.getDepths2d().size()?event.getDepths2d():_depths2DMap);
				_initProgressDialog->appendText("Updating the graph view... done.");
			}
		}
		else
		{
			_initProgressDialog->appendText("No poses received! The map cloud cannot be updated...");
			UINFO("Map received is empty! Cannot update the map cloud...");
		}

		_initProgressDialog->appendText(tr("%1 locations are updated to/inserted in the cache.").arg(event.getPoses().size()));
	}
	_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
}

void MainWindow::applyAllPrefSettings()
{
	ULOGGER_DEBUG("");

	//This will update the statistics toolbox
	_ui->statsToolBox->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	if(_ui->statsToolBox->findChildren<StatItem*>().size() == 0)
	{
		const std::map<std::string, float> & statistics = Statistics::defaultData();
		for(std::map<std::string, float>::const_iterator iter = statistics.begin(); iter != statistics.end(); ++iter)
		{
			_ui->statsToolBox->updateStat(QString((*iter).first.c_str()).replace('_', ' '), 0, (*iter).second);
		}
	}

	_ui->graphicsView_graphView->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());

	this->applyPrefSettings(PreferencesDialog::kPanelAll);
	this->applyPrefSettings(_preferencesDialog->getAllParameters());
}

void MainWindow::applyPrefSettings(PreferencesDialog::PANEL_FLAGS flags)
{
	ULOGGER_DEBUG("");
	if(flags & PreferencesDialog::kPanelSource)
	{
		// Camera settings...
		_ui->doubleSpinBox_stats_imgRate->setValue(_preferencesDialog->getGeneralInputRate());
		this->updateSelectSourceImageMenu(_preferencesDialog->getSourceImageType());
		this->updateSelectSourceDatabase(_preferencesDialog->isSourceDatabaseUsed());
		this->updateSelectSourceOpenni(_preferencesDialog->isSourceOpenniUsed());
		QString src;
		if(_preferencesDialog->isSourceImageUsed())
		{
			src = _preferencesDialog->getSourceImageTypeStr();
		}
		else if(_preferencesDialog->isSourceDatabaseUsed())
		{
			src = "Database";
		}
		_ui->label_stats_source->setText(src);

		if(_camera)
		{
			_camera->getCamera()->setImageRate(_preferencesDialog->getGeneralInputRate());
			_camera->setAutoRestart(_preferencesDialog->getGeneralAutoRestart());
		}
		if(_dbReader)
		{
			_dbReader->setFrameRate(_preferencesDialog->getGeneralInputRate());
		}

		if(_cameraOpenni)
		{
			_cameraOpenni->setFrameRate(_preferencesDialog->getGeneralInputRate());
		}
	}

	if(flags & PreferencesDialog::kPanelGeneral)
	{
		UDEBUG("General settings changed...");
		setupMainLayout(_preferencesDialog->isVerticalLayoutUsed());
	}

	if(flags & PreferencesDialog::kPanelCloudRendering)
	{
		UDEBUG("Cloud rendering settings changed...");
		if(_currentPosesMap.size())
		{
			this->updateMapCloud(_currentPosesMap, Transform());
		}
	}

	if(flags & PreferencesDialog::kPanelLogging)
	{
		UDEBUG("Logging settings changed...");
		ULogger::setLevel((ULogger::Level)_preferencesDialog->getGeneralLoggerLevel());
		ULogger::setEventLevel((ULogger::Level)_preferencesDialog->getGeneralLoggerEventLevel());
		ULogger::setType((ULogger::Type)_preferencesDialog->getGeneralLoggerType(),
						 ((_preferencesDialog->getWorkingDirectory()+"/")+LOG_FILE_NAME).toStdString(), true);
		ULogger::setPrintTime(_preferencesDialog->getGeneralLoggerPrintTime());
	}
}

void MainWindow::applyPrefSettings(const rtabmap::ParametersMap & parameters)
{
	ULOGGER_DEBUG("");
	if(parameters.size())
	{
		for(rtabmap::ParametersMap::const_iterator iter = parameters.begin(); iter!=parameters.end(); ++iter)
		{
			UDEBUG("Parameter changed: Key=%s Value=%s", iter->first.c_str(), iter->second.c_str());
		}

		rtabmap::ParametersMap parametersModified = parameters;

		if(_state != kIdle)
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
			if(parametersModified.erase(Parameters::kRtabmapDatabasePath()))
			{
				if(_state == kMonitoring || _state == kMonitoringPaused)
				{
					QMessageBox::information(this, tr("Database path changed"), tr("The remote database path can't be changed while the interface is in monitoring mode."));
				}
				else
				{
					QMessageBox::information(this, tr("Database path changed"), tr("The database path can't be changed while the detector is running. This will be applied when the detector will stop."));
				}
			}
			this->post(new ParamEvent(parametersModified));
		}
		else if(parametersModified.size() == _preferencesDialog->getAllParameters().size())
		{
			// Send only if all parameters are sent
			this->post(new ParamEvent(parametersModified));
		}

		// update loop closure viewer parameters
		if(uContains(parameters, Parameters::kLccIcp3Decimation()))
		{
			_ui->widget_loopClosureViewer->setDecimation(atoi(parameters.at(Parameters::kLccIcp3Decimation()).c_str()));
		}
		if(uContains(parameters, Parameters::kLccIcp3MaxDepth()))
		{
			_ui->widget_loopClosureViewer->setMaxDepth(atof(parameters.at(Parameters::kLccIcp3MaxDepth()).c_str()));
		}
		if(uContains(parameters, Parameters::kLccIcp3Samples()))
		{
			_ui->widget_loopClosureViewer->setSamples(atoi(parameters.at(Parameters::kLccIcp3Samples()).c_str()));
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
	KeypointItem * item = 0;
	int alpha = _preferencesDialog->getKeypointsOpacity()*255/100;
	ULOGGER_DEBUG("refWords.size() = %d", refWords.size());
	QMap<int, KeypointItem*> addedKeypoints;
	for(std::multimap<int, cv::KeyPoint>::const_iterator i = refWords.begin(); i != refWords.end(); ++i )
	{
		const cv::KeyPoint & r = (*i).second;
		int id = (*i).first;

		QString info = QString( "WordRef = %1\n"
								"Laplacian = %2\n"
								"Dir = %3\n"
								"Hessian = %4\n"
								"X = %5\n"
								"Y = %6\n"
								"Size = %7").arg(id).arg(1).arg(r.angle).arg(r.response).arg(r.pt.x).arg(r.pt.y).arg(r.size);
		float radius = r.size*1.2/9.*2;
		if(uContains(loopWords, id))
		{
			// PINK = FOUND IN LOOP SIGNATURE
			item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 0, 255, alpha));
		}
		else if(_lastIds.contains(id))
		{
			// BLUE = FOUND IN LAST SIGNATURE
			item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(0, 0, 255, alpha));
		}
		else if(id<=_lastId)
		{
			// RED = ALREADY EXISTS
			item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 0, 0, alpha));
		}
		else if(refWords.count(id) > 1)
		{
			// YELLOW = NEW and multiple times
			item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 255, 0, alpha));
		}
		else
		{
			// GREEN = NEW
			item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(0, 255, 0, alpha));
		}
		item->setVisible(this->_ui->imageView_source->isFeaturesShown());
		this->_ui->imageView_source->scene()->addItem(item);
		item->setZValue(1);
		addedKeypoints.insert(id, item);
	}
	ULOGGER_DEBUG("source time = %f s", timer.ticks());

	timer.start();
	item = 0;
	ULOGGER_DEBUG("loopWords.size() = %d", loopWords.size());
	QList<QPair<KeypointItem*, KeypointItem*> > uniqueCorrespondences;
	for(std::multimap<int, cv::KeyPoint>::const_iterator i = loopWords.begin(); i != loopWords.end(); ++i )
	{
		const cv::KeyPoint & r = (*i).second;
		int id = (*i).first;

		QString info = QString( "WordRef = %1\n"
								"Laplacian = %2\n"
								"Dir = %3\n"
								"Hessian = %4\n"
								"X = %5\n"
								"Y = %6\n"
								"Size = %7").arg(id).arg(1).arg(r.angle).arg(r.response).arg(r.pt.x).arg(r.pt.y).arg(r.size);
		float radius = r.size*1.2/9.*2;
		if(uContains(refWords, id))
		{
			// PINK = FOUND IN LOOP SIGNATURE
			item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 0, 255, alpha));
			//To draw lines... get only unique correspondences
			if(uValues(refWords, id).size() == 1 && uValues(loopWords, id).size() == 1)
			{
				uniqueCorrespondences.push_back(QPair<KeypointItem*, KeypointItem*>(addedKeypoints.value(id), item));
			}
		}
		else if(id<=_lastId)
		{
			// RED = ALREADY EXISTS
			item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 0, 0, alpha));
		}
		else if(refWords.count(id) > 1)
		{
			// YELLOW = NEW and multiple times
			item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 255, 0, alpha));
		}
		else
		{
			// GREEN = NEW
			item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(0, 255, 0, alpha));
		}
		item->setVisible(this->_ui->imageView_loopClosure->isFeaturesShown());
		this->_ui->imageView_loopClosure->scene()->addItem(item);
		item->setZValue(1);
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
	float scaleX = _ui->imageView_source->transform().m11();
	float scaleY = _ui->imageView_source->transform().m22();
	UDEBUG("scaleX=%f scaleY=%f", scaleX, scaleY);
	int deltaX = _ui->imageView_source->width()/scaleX;
	int deltaY = 0;
	if(_preferencesDialog->isVerticalLayoutUsed())
	{
		deltaX = 0;
		deltaY = _ui->imageView_source->height()/scaleY;
		deltaY += _ui->label_matchId->height()/scaleY;
	}
	for(QList<QPair<KeypointItem*, KeypointItem*> >::iterator iter = uniqueCorrespondences.begin();
		iter!=uniqueCorrespondences.end();
		++iter)
	{
		QGraphicsLineItem * item = _ui->imageView_source->scene()->addLine(
				iter->first->rect().x()+iter->first->rect().width()/2,
				iter->first->rect().y()+iter->first->rect().height()/2,
				iter->second->rect().x()+iter->second->rect().width()/2+deltaX,
				iter->second->rect().y()+iter->second->rect().height()/2+deltaY,
				QPen(QColor(0, 255, 255, alpha)));
		item->setVisible(_ui->imageView_source->isLinesShown());
		item->setZValue(1);

		item = _ui->imageView_loopClosure->scene()->addLine(
				iter->first->rect().x()+iter->first->rect().width()/2-deltaX,
				iter->first->rect().y()+iter->first->rect().height()/2-deltaY,
				iter->second->rect().x()+iter->second->rect().width()/2,
				iter->second->rect().y()+iter->second->rect().height()/2,
				QPen(QColor(0, 255, 255, alpha)));
		item->setVisible(_ui->imageView_loopClosure->isLinesShown());
		item->setZValue(1);
	}
}

void MainWindow::resizeEvent(QResizeEvent* anEvent)
{
	_ui->imageView_source->fitInView(_ui->imageView_source->sceneRect(), Qt::KeepAspectRatio);
	_ui->imageView_loopClosure->fitInView(_ui->imageView_source->sceneRect(), Qt::KeepAspectRatio);
	_ui->imageView_source->resetZoom();
	_ui->imageView_loopClosure->resetZoom();
}

void MainWindow::updateSelectSourceImageMenu(int type)
{
	if(_preferencesDialog->isSourceImageUsed())
	{
		switch(type)
		{
		case 0:
			_ui->actionUsbCamera->setChecked(true);
			break;
		case 1:
			_ui->actionImageFiles->setChecked(true);
			break;
		case 2:
			_ui->actionVideo->setChecked(true);
			break;
		default:
			UERROR("Unknown source image type");
			break;
		}
	}
	else
	{
		// they are exclusive actions, so check/uncheck one should disable all.
		_ui->actionUsbCamera->setChecked(true);
		_ui->actionUsbCamera->setChecked(false);
	}
}

void MainWindow::updateSelectSourceDatabase(bool used)
{
	_ui->actionDatabase->setChecked(used);
}

void MainWindow::updateSelectSourceOpenni(bool used)
{
	_ui->actionOpenni_RGBD->setChecked(used);
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
	QString targetDir = _preferencesDialog->getWorkingDirectory() + "/ScreensCaptured";
	QDir dir;
	if(!dir.exists(targetDir))
	{
		dir.mkdir(targetDir);
	}
	targetDir += "/";
	targetDir += "Main_window";
	if(!dir.exists(targetDir))
	{
		dir.mkdir(targetDir);
	}
	targetDir += "/";
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

//ACTIONS
void MainWindow::startDetection()
{
	ParametersMap parameters = _preferencesDialog->getAllParameters();
	// verify source with input rates
	if((_preferencesDialog->isSourceImageUsed() && (_preferencesDialog->getSourceImageType()>0)) ||
	   _preferencesDialog->isSourceDatabaseUsed())
	{
		float inputRate = _preferencesDialog->getGeneralInputRate();
		float detectionRate = std::atof(parameters.at(Parameters::kRtabmapDetectionRate()).c_str());
		int bufferingSize = std::atof(parameters.at(Parameters::kRtabmapImageBufferSize()).c_str());
		if((detectionRate!=0.0f && detectionRate < inputRate) || (detectionRate > 0.0f && inputRate == 0.0f))
		{
			int button = QMessageBox::question(this,
					tr("Incompatible frame rates!"),
					tr("\"Source/Input rate\" (%1 Hz) is higher than \"RTAB-Map/Detection rate\" (%2 Hz). As the "
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
		emit stateChanged(kIdle);
		return;
	}
	if(_dbReader != 0)
	{
		QMessageBox::warning(this,
							 tr("RTAB-Map"),
							 tr("A database reader is running, stop it first."));
		UWARN("_dbReader is not null... it must be stopped first");
		emit stateChanged(kIdle);
		return;
	}
	if(_cameraOpenni != 0)
	{
		QMessageBox::warning(this,
							 tr("RTAB-Map"),
							 tr("An Openni camera is running, stop it first."));
		UWARN("_cameraOpenni is not null... it must be stopped first");
		emit stateChanged(kIdle);
		return;
	}

	// Adjust pre-requirements
	if( !_preferencesDialog->isSourceImageUsed() &&
		!_preferencesDialog->isSourceDatabaseUsed() &&
		!_preferencesDialog->isSourceOpenniUsed())
	{
		QMessageBox::warning(this,
				 tr("RTAB-Map"),
				 tr("No sources are selected. See Preferences->Source panel."));
		UWARN("No sources are selected. See Preferences->Source panel.");
		emit stateChanged(kIdle);
		return;
	}

	if(_preferencesDialog->isSourceOpenniUsed())
	{
		//Create odometry thread if rgbd slam
		if(uStr2Bool(parameters.at(Parameters::kRGBDEnabled()).c_str()))
		{
			if(_odomThread)
			{
				UERROR("OdomThread must be already deleted here?!");
				delete _odomThread;
			}
			Odometry * odom;
			if(_preferencesDialog->getOdometryType() == PreferencesDialog::kOdomBIN)
			{
				UINFO("Using odometry FAST/BRIEF");
				odom = new OdometryBinary(parameters);
			}
			else if(_preferencesDialog->getOdometryType() == PreferencesDialog::kOdomBOW)
			{
				UINFO("Using odometry SIFT/SURF");
				odom = new OdometryBOW(parameters);
			}
			else
			{
				UINFO("Using odometry ICP");
				odom = new OdometryICP(parameters);
			}
			_odomThread = new OdometryThread(odom);

			UEventsManager::addHandler(_odomThread);
			_odomThread->start();
		}

		// With odomtry, go as fast as we can
		_cameraOpenni = new CameraOpenni(
			_preferencesDialog->getSourceOpenniDevice().toStdString(),
			_preferencesDialog->getGeneralInputRate(),
			_preferencesDialog->getSourceOpenniLocalTransform());

		if(!_cameraOpenni->init())
		{
			ULOGGER_WARN("init CameraOpenni failed... ");
			QMessageBox::warning(this,
								   tr("RTAB-Map"),
								   tr("Openni camera initialization failed..."));
			emit stateChanged(kIdle);
			delete _cameraOpenni;
			_cameraOpenni = 0;
			if(_odomThread)
			{
				delete _odomThread;
				_odomThread = 0;
			}
			return;
		}
		if(_odomThread)
		{
			UEventsManager::createPipe(_cameraOpenni, _odomThread, "CameraEvent");
		}
	}
	else if(_preferencesDialog->isSourceDatabaseUsed())
	{
		_dbReader = new DBReader(_preferencesDialog->getSourceDatabasePath().toStdString(),
								 _preferencesDialog->getGeneralInputRate(),
								 _preferencesDialog->getSourceDatabaseOdometryIgnored());

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
			if(_preferencesDialog->getOdometryType() == PreferencesDialog::kOdomBIN)
			{
				UINFO("Using odometry FAST/BRIEF");
				odom = new OdometryBinary(parameters);
			}
			else if(_preferencesDialog->getOdometryType() == PreferencesDialog::kOdomBOW)
			{
				UINFO("Using odometry SIFT/SURF");
				odom = new OdometryBOW(parameters);
			}
			else
			{
				UINFO("Using odometry ICP");
				odom = new OdometryICP(parameters);
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
			emit stateChanged(kIdle);
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
	else
	{
		if(_preferencesDialog->isSourceImageUsed())
		{
			Camera * camera = 0;
			// Change type of the camera...
			//
			int sourceType = _preferencesDialog->getSourceImageType();
			if(sourceType == 1) //Images
			{
				camera = new CameraImages(
						_preferencesDialog->getSourceImagesPath().append(QDir::separator()).toStdString(),
						_preferencesDialog->getSourceImagesStartPos(),
						_preferencesDialog->getSourceImagesRefreshDir(),
						_preferencesDialog->getGeneralInputRate(),
						_preferencesDialog->getSourceWidth(),
						_preferencesDialog->getSourceHeight(),
						_preferencesDialog->getFramesDropped()
						);
			}
			else if(sourceType == 2)
			{
				camera = new CameraVideo(
						_preferencesDialog->getSourceVideoPath().toStdString(),
						_preferencesDialog->getGeneralInputRate(),
						_preferencesDialog->getSourceWidth(),
						_preferencesDialog->getSourceHeight(),
						_preferencesDialog->getFramesDropped());
			}
			else if(sourceType == 0)
			{
				camera = new CameraVideo(
						_preferencesDialog->getSourceUsbDeviceId(),
						_preferencesDialog->getGeneralInputRate(),
						_preferencesDialog->getSourceWidth(),
						_preferencesDialog->getSourceHeight(),
						_preferencesDialog->getFramesDropped());
			}
			else
			{
				QMessageBox::warning(this,
										   tr("RTAB-Map"),
										   tr("Source type is not supported..."));
				ULOGGER_WARN("iSource type not supported...");
				emit stateChanged(kIdle);
				return;
			}

			if(_preferencesDialog->getGeneralCameraKeypoints())
			{
				camera->setFeaturesExtracted(true);
				camera->parseParameters(_preferencesDialog->getAllParameters());
			}

			if(!camera->init())
			{
				ULOGGER_WARN("init camera failed... ");
				QMessageBox::warning(this,
									   tr("RTAB-Map"),
									   tr("Camera initialization failed..."));
				emit stateChanged(kIdle);
				delete camera;
				camera = 0;
				return;
			}

			_camera = new CameraThread(camera);
			UEventsManager::addHandler(_camera); //thread
		}
	}

	_lastOdomPose.setNull();
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdCleanDataBuffer)); // clean sensors buffer
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdTriggerNewMap)); // Trigger a new map
	this->applyAllPrefSettings();

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
	if(_camera || _dbReader || _cameraOpenni)
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
	if(_state == kIdle || (!_camera && !_dbReader && !_cameraOpenni))
	{
		return;
	}

	if(_state == kDetecting &&
			( (_camera && _camera->isRunning()) ||
			  (_dbReader && _dbReader->isRunning()) ||
			  (_cameraOpenni && _cameraOpenni->isRunning())) )
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

	if(_cameraOpenni)
	{
		_cameraOpenni->kill();
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
	if(_cameraOpenni)
	{
		delete _cameraOpenni;
		_cameraOpenni = 0;
	}
	if(_odomThread)
	{
		delete _odomThread;
		_odomThread = 0;
	}
	emit stateChanged(kIdle);
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
		_graphSavingFileName = _preferencesDialog->getWorkingDirectory() + "/Graph.dot";
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
		_graphSavingFileName = _preferencesDialog->getWorkingDirectory() + "/Graph.dot";
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
		_toroSavingFileName = _preferencesDialog->getWorkingDirectory() + "/toro.graph";
	}

	QStringList items;
	items.append("Local map optimized");
	items.append("Local map not optimized");
	items.append("Global map optimized");
	items.append("Global map not optimized");
	bool ok;
	QString item = QInputDialog::getItem(this, tr("Parameters"), tr("Options:"), items, 0, false, &ok);
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

void MainWindow::deleteMemory()
{
	QMessageBox::StandardButton button;
	QString dbPath = _preferencesDialog->getDatabasePath();
	if(_state == kMonitoring || _state == kMonitoringPaused)
	{
		button = QMessageBox::question(this,
				tr("Deleting memory..."),
				tr("The remote database file \"%1\" and log files will be deleted. Are you sure you want to continue? (This cannot be reverted)").arg(dbPath),
				QMessageBox::Yes|QMessageBox::No,
				QMessageBox::No);
	}
	else
	{
		button = QMessageBox::question(this,
				tr("Deleting memory..."),
				tr("The database file \"%1\" (%2 MB) and log files will be deleted. Are you sure you want to continue? (This cannot be reverted)").arg(dbPath).arg(UFile::length(dbPath.toStdString())/1000000),
				QMessageBox::Yes|QMessageBox::No,
				QMessageBox::No);
	}

	if(button != QMessageBox::Yes)
	{
		return;
	}

	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDeleteMemory, dbPath.toStdString()));
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
	if(_state != kMonitoring && _state != kMonitoringPaused)
	{
		QString dbPath = _preferencesDialog->getWorkingDirectory() + "/rtabmap.db";
		_ui->actionDelete_memory->setText(tr("Delete memory (%1 MB)").arg(UFile::length(dbPath.toStdString())/1000000));
	}
}

void MainWindow::selectImages()
{
	_preferencesDialog->selectSourceImage(PreferencesDialog::kSrcImages);
}

void MainWindow::selectVideo()
{
	_preferencesDialog->selectSourceImage(PreferencesDialog::kSrcVideo);
}

void MainWindow::selectStream()
{
	_preferencesDialog->selectSourceImage(PreferencesDialog::kSrcUsbDevice);
}

void MainWindow::selectDatabase()
{
	_preferencesDialog->selectSourceDatabase(true);
}

void MainWindow::selectOpenni()
{
	_preferencesDialog->selectSourceOpenni(true);
}


void MainWindow::dumpTheMemory()
{
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDumpMemory));
}

void MainWindow::dumpThePrediction()
{
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDumpPrediction));
}

void MainWindow::downloadAllClouds()
{
	bool showAll = _state != kMonitoringPaused && _state != kMonitoring;

	QStringList items;
	items.append("Local map optimized");
	if(showAll)
	{
		items.append("Local map not optimized");
	}
	items.append("Global map optimized");
	if(showAll)
	{
		items.append("Global map not optimized");
	}
	bool ok;
	QString item = QInputDialog::getItem(this, tr("Parameters"), tr("Options:"), items, showAll?2:1, false, &ok);
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
			this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPublish3DMapGlobal, optimized?1:0));
		}
		else
		{
			this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPublish3DMapLocal, optimized?1:0));
		}
	}
}

void MainWindow::downloadPoseGraph()
{
	bool showAll = _state != kMonitoringPaused && _state != kMonitoring;

	QStringList items;
	items.append("Local map optimized");
	if(showAll)
	{
		items.append("Local map not optimized");
	}
	items.append("Global map optimized");
	if(showAll)
	{
		items.append("Global map not optimized");
	}
	bool ok;
	QString item = QInputDialog::getItem(this, tr("Parameters"), tr("Options:"), items, showAll?2:1, false, &ok);
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
			this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPublishTOROGraphGlobal, optimized?1:0));
		}
		else
		{
			this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPublishTOROGraphLocal, optimized?1:0));
		}
	}
}

void MainWindow::clearTheCache()
{
	_imagesMap.clear();
	_depthsMap.clear();
	_depths2DMap.clear();
	_depthConstantsMap.clear();
	_localTransformsMap.clear();
	_ui->widget_cloudViewer->removeAllClouds();
	_ui->widget_cloudViewer->render();
	_currentPosesMap.clear();
	_odometryCorrection = Transform::getIdentity();
	_lastOdomPose.setNull();
	//disable save cloud action
	_ui->actionSave_point_cloud->setEnabled(false);
	_ui->actionView_high_res_point_cloud->setEnabled(false);
	_ui->actionSave_mesh_ply_vtk_stl->setEnabled(false);
	_ui->actionView_point_cloud_as_mesh->setEnabled(false);
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
	_ui->graphicsView_graphView->clearGraph();
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

	if(curvesPerFigure.size() == 0)
	{
		QMessageBox msgBox;
		msgBox.setText("There is no figure shown.");
		msgBox.setInformativeText("Do you want to save anyway ? (this will erase the previous saved configuration)");
		msgBox.setStandardButtons(QFlags<QMessageBox::StandardButton>(QMessageBox::Save | QMessageBox::Cancel));
		msgBox.setDefaultButton(QMessageBox::Cancel);
		int ret = msgBox.exec();
		if(ret == QMessageBox::Cancel)
		{
			return;
		}
	}

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

void MainWindow::openPreferences()
{
	_preferencesDialog->setMonitoringState(_state == kMonitoring || _state == kMonitoringPaused);
	_preferencesDialog->exec();
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

void MainWindow::savePointClouds()
{
	int button = QMessageBox::question(this,
			tr("One or multiple files?"),
			tr("Save clouds separately?"),
			QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);

	if(button == QMessageBox::Yes || button == QMessageBox::No)
	{
		_initProgressDialog->setAutoClose(true, 1);
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		_initProgressDialog->setMaximumSteps(_currentPosesMap.size()*2+1);

		std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
		if(button == QMessageBox::No)
		{
			clouds.insert(std::make_pair(0, this->createAssembledCloud()));
		}
		else
		{
			clouds = this->createPointClouds();
		}
		savePointClouds(clouds);
		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
	}
}

void MainWindow::saveMeshes()
{
	int button = QMessageBox::question(this,
			tr("One or multiple files?"),
			tr("Save meshes separately?"),
			QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);

	if(button == QMessageBox::Yes || button == QMessageBox::No)
	{
		_initProgressDialog->setAutoClose(true, 1);
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		_initProgressDialog->setMaximumSteps(_currentPosesMap.size()*2+1);

		std::map<int, pcl::PolygonMesh::Ptr> meshes;
		if(button == QMessageBox::No)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = this->createAssembledCloud();
			meshes.insert(std::make_pair(0, util3d::createMesh(cloud)));
		}
		else
		{
			meshes = this->createMeshes();
		}
		saveMeshes(meshes);
		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
	}
}



void MainWindow::viewPointClouds()
{
	int button = QMessageBox::question(this,
		tr("One or multiple clouds?"),
		tr("View clouds separately?"),
		QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);

	if(button == QMessageBox::Yes || button == QMessageBox::No)
	{
		_initProgressDialog->setAutoClose(true, 1);
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		_initProgressDialog->setMaximumSteps(_currentPosesMap.size()+1);

		std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
		if(button == QMessageBox::No)
		{
			clouds.insert(std::make_pair(0, this->createAssembledCloud()));
		}
		else
		{
			clouds = this->createPointClouds();
		}

		if(clouds.size())
		{
			QWidget * window = new QWidget(this, Qt::Window);
			window->setAttribute(Qt::WA_DeleteOnClose);
			window->setWindowFlags(Qt::Dialog);
			window->setWindowTitle(tr("Clouds (%1 nodes)").arg(clouds.size()));
			window->setMinimumWidth(800);
			window->setMinimumHeight(600);

			CloudViewer * viewer = new CloudViewer(window);

			QVBoxLayout *layout = new QVBoxLayout();
			layout->addWidget(viewer);
			window->setLayout(layout);

			window->show();

			uSleep(500);

			for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator iter = clouds.begin(); iter!=clouds.end(); ++iter)
			{
				_initProgressDialog->appendText(tr("Viewing the cloud %1 (%2 points)...").arg(iter->first).arg(iter->second->size()));
				_initProgressDialog->incrementStep();
				viewer->addCloud(uFormat("cloud%d",iter->first), iter->second, iter->first>0?_currentPosesMap.at(iter->first):Transform::getIdentity());
				_initProgressDialog->appendText(tr("Viewing the cloud %1 (%2 points)... done.").arg(iter->first).arg(iter->second->size()));
			}
		}
		else
		{
			QMessageBox::warning(this, tr("Viewing clouds failed!"), tr("Clouds are empty..."));
		}

		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
	}
}

void MainWindow::viewMeshes()
{
	int button = QMessageBox::question(this,
		tr("One or multiple meshes?"),
		tr("View meshes separately?"),
		QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);

	if(button == QMessageBox::Yes || button == QMessageBox::No)
	{
		_initProgressDialog->setAutoClose(true, 1);
		_initProgressDialog->resetProgress();
		_initProgressDialog->show();
		_initProgressDialog->setMaximumSteps(_currentPosesMap.size()+1);

		std::map<int, pcl::PolygonMesh::Ptr> meshes;
		if(button == QMessageBox::No)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = this->createAssembledCloud();
			_initProgressDialog->appendText(tr("Meshing the assembled cloud (%1 points)...").arg(cloud->size()));
			_initProgressDialog->incrementStep();
			meshes.insert(std::make_pair(0, util3d::createMesh(cloud, _preferencesDialog->getCloudVoxelSize(2)*4.0f)));
		}
		else
		{
			meshes = this->createMeshes();
		}

		if(meshes.size())
		{
			QWidget * window = new QWidget(this, Qt::Window);
			window->setAttribute(Qt::WA_DeleteOnClose);
			window->setWindowTitle(tr("Meshes (%1 nodes)").arg(meshes.size()));
			window->setMinimumWidth(800);
			window->setMinimumHeight(600);

			CloudViewer * viewer = new CloudViewer(window);

			QVBoxLayout *layout = new QVBoxLayout();
			layout->addWidget(viewer);
			window->setLayout(layout);

			window->show();

			uSleep(500);

			for(std::map<int, pcl::PolygonMesh::Ptr>::iterator iter = meshes.begin(); iter!=meshes.end(); ++iter)
			{
				_initProgressDialog->appendText(tr("Viewing the mesh %1 (%2 polygons)...").arg(iter->first).arg(iter->second->polygons.size()));
				_initProgressDialog->incrementStep();
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::fromPCLPointCloud2(iter->second->cloud, *cloud);
				viewer->addCloudMesh(uFormat("mesh%d",iter->first), cloud, iter->second->polygons, iter->first>0?_currentPosesMap.at(iter->first):Transform::getIdentity());
				_initProgressDialog->appendText(tr("Viewing the mesh %1 (%2 polygons)... done.").arg(iter->first).arg(iter->second->polygons.size()));
			}
		}
		else
		{
			QMessageBox::warning(this, tr("Viewing meshes failed!"), tr("Meshes are empty..."));
		}

		_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
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

//END ACTIONS

void MainWindow::savePointClouds(const std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds)
{
	if(clouds.size() == 1)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save to ..."), "", tr("Point cloud data (*.pcd *.ply *.vtk)"));
		if(!path.isEmpty())
		{
			if(clouds.begin()->second->size())
			{
				_initProgressDialog->appendText(tr("Saving the cloud (%1 points)...").arg(clouds.begin()->second->size()));

				bool success =false;
				if(QFileInfo(path).suffix() == "pcd")
				{
					success = pcl::io::savePCDFile(path.toStdString(), *clouds.begin()->second) == 0;
				}
				else if(QFileInfo(path).suffix() == "ply")
				{
					success = pcl::io::savePLYFile(path.toStdString(), *clouds.begin()->second) == 0;
				}
				else if(QFileInfo(path).suffix() == "vtk")
				{
					pcl::PCLPointCloud2 pt2;
					pcl::toPCLPointCloud2(*clouds.begin()->second, pt2);
					success = pcl::io::saveVTKFile(path.toStdString(), pt2) == 0;
				}
				else
				{
					UERROR("Extension not recognized! (%s)", QFileInfo(path).suffix().toStdString().c_str());
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
		QString path = QFileDialog::getExistingDirectory(this, tr("Save to (*.pcd *.ply *.vtk)..."), _preferencesDialog->getWorkingDirectory(), 0);
		if(!path.isEmpty())
		{
			bool ok = false;
			QStringList items;
			items.push_back("pcd");
			items.push_back("ply");
			items.push_back("vtk");
			QString suffix = QInputDialog::getItem(this, tr("File format"), tr("Which format?"), items, 0, false, &ok);

			if(ok)
			{
				for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::const_iterator iter=clouds.begin(); iter!=clouds.end(); ++iter)
				{
					if(iter->second->size())
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud;
						transformedCloud = util3d::transformPointCloud(iter->second, _currentPosesMap.at(iter->first));

						QString pathFile = path+QDir::separator()+QString("cloud%1.%2").arg(iter->first).arg(suffix);
						bool success =false;
						if(suffix == "pcd")
						{
							success = pcl::io::savePCDFile(pathFile.toStdString(), *transformedCloud) == 0;
						}
						else if(suffix == "ply")
						{
							success = pcl::io::savePLYFile(pathFile.toStdString(), *transformedCloud) == 0;
						}
						else if(suffix == "vtk")
						{
							pcl::PCLPointCloud2 pt2;
							pcl::toPCLPointCloud2(*transformedCloud, pt2);
							success = pcl::io::saveVTKFile(pathFile.toStdString(), pt2) == 0;
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

void MainWindow::saveMeshes(const std::map<int, pcl::PolygonMesh::Ptr> & meshes)
{
	if(meshes.size() == 1)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save to ..."), "", tr("Mesh (*.ply *.vtk)"));
		if(!path.isEmpty())
		{
			if(meshes.begin()->second->polygons.size())
			{
				_initProgressDialog->appendText(tr("Saving the mesh (%1 polygons)...").arg(meshes.begin()->second->polygons.size()));

				bool success =false;
				if(QFileInfo(path).suffix() == "ply")
				{
					success = pcl::io::savePLYFile(path.toStdString(), *meshes.begin()->second) == 0;
				}
				else if(QFileInfo(path).suffix() == "vtk")
				{
					success = pcl::io::saveVTKFile(path.toStdString(), *meshes.begin()->second) == 0;

				}
				else
				{
					UERROR("Extension not recognized! (%s)", QFileInfo(path).suffix().toStdString().c_str());
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
		QString path = QFileDialog::getExistingDirectory(this, tr("Save to (*.ply, *.vtk)..."), _preferencesDialog->getWorkingDirectory(), 0);
		if(!path.isEmpty())
		{
			bool ok = false;
			QStringList items;
			items.push_back("ply");
			items.push_back("vtk");
			QString suffix = QInputDialog::getItem(this, tr("File format"), tr("Which format?"), items, 0, false, &ok);

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

						QString pathFile = path+QDir::separator()+QString("mesh%1.%2").arg(iter->first).arg(suffix);
						bool success =false;
						if(suffix == "ply")
						{
							success = pcl::io::savePLYFile(pathFile.toStdString(), mesh) == 0;
						}
						else if(suffix == "vtk")
						{
							success = pcl::io::saveVTKFile(pathFile.toStdString(), mesh) == 0;
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MainWindow::createCloud(
		int id,
		const cv::Mat & rgb,
		const cv::Mat & depth,
		float depthConstant,
		const Transform & localTransform,
		const Transform & pose,
		float voxelSize,
		int decimation,
		float maxDepth)
{
	UTimer timer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
			rgb,
			depth,
		   float(depth.cols/2),
		   float(depth.rows/2),
		   1.0f/depthConstant,
		   1.0f/depthConstant,
		   decimation);

	if(cloud->size())
	{
		bool filtered = false;
		if(cloud->size() && maxDepth)
		{
			cloud = util3d::passThrough(cloud, "z", 0, maxDepth);
			filtered = true;
		}

		if(cloud->size() && voxelSize)
		{
			cloud = util3d::voxelize(cloud, voxelSize);
			filtered = true;
		}

		if(cloud->size() && !filtered)
		{
			cloud = util3d::removeNaNFromPointCloud (cloud);
		}

		if(cloud->size())
		{
			cloud = util3d::transformPointCloud(cloud, pose * localTransform);
		}
	}
	UDEBUG("Generated cloud %d (pts=%d) time=%fs", id, (int)cloud->size(), timer.ticks());
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MainWindow::createAssembledCloud()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	int i=0;
	int count = 0;
	for(std::map<int, Transform>::const_iterator iter = _currentPosesMap.begin(); iter!=_currentPosesMap.end(); ++iter)
	{
		bool inserted = false;
		if(!iter->second.isNull())
		{
			if(_imagesMap.contains(iter->first) && _depthsMap.contains(iter->first))
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
				cloud = createCloud(iter->first,
						util3d::uncompressImage(_imagesMap.value(iter->first)),
						util3d::uncompressImage(_depthsMap.value(iter->first)),
						_depthConstantsMap.value(iter->first),
						_localTransformsMap.value(iter->first),
						iter->second,
						_preferencesDialog->getCloudVoxelSize(2),
						_preferencesDialog->getCloudDecimation(2),
						_preferencesDialog->getCloudMaxDepth(2));

				if(cloud->size() && _preferencesDialog->getCloudVoxelSize(2))
				{
					UDEBUG("Voxelize the cloud %d (%d points, voxel %f m)", iter->first, (int)cloud->size(), _preferencesDialog->getCloudVoxelSize(2));
					cloud = util3d::voxelize(cloud, _preferencesDialog->getCloudVoxelSize(2));
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
				UERROR("Cloud %d not found?!?", iter->first);
			}
		}
		else
		{
			UERROR("transform is null!?");
		}

		if(inserted)
		{
			_initProgressDialog->appendText(tr("Generated cloud %1 (%2/%3).").arg(iter->first).arg(++i).arg(_currentPosesMap.size()));

			if(count % 100 == 0)
			{
				if(assembledCloud->size() && _preferencesDialog->getCloudVoxelSize(2))
				{
					assembledCloud = util3d::voxelize(assembledCloud, _preferencesDialog->getCloudVoxelSize(2));
				}
			}
		}
		else
		{
			_initProgressDialog->appendText(tr("Ignored cloud %1 (%2/%3).").arg(iter->first).arg(++i).arg(_currentPosesMap.size()));
		}
		_initProgressDialog->incrementStep();
		QApplication::processEvents();
	}

	if(assembledCloud->size() && _preferencesDialog->getCloudVoxelSize(2))
	{
		assembledCloud = util3d::voxelize(assembledCloud, _preferencesDialog->getCloudVoxelSize(2));
	}

	return assembledCloud;
}

std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > MainWindow::createPointClouds()
{
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	int i=0;
	for(std::map<int, Transform>::const_iterator iter = _currentPosesMap.begin(); iter!=_currentPosesMap.end(); ++iter)
	{
		bool inserted = false;
		if(!iter->second.isNull())
		{
			if(_imagesMap.contains(iter->first) && _depthsMap.contains(iter->first))
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
				cloud = createCloud(iter->first,
						util3d::uncompressImage(_imagesMap.value(iter->first)),
						util3d::uncompressImage(_depthsMap.value(iter->first)),
						_depthConstantsMap.value(iter->first),
						_localTransformsMap.value(iter->first),
						Transform::getIdentity(),
						_preferencesDialog->getCloudVoxelSize(2),
						_preferencesDialog->getCloudDecimation(2),
						_preferencesDialog->getCloudMaxDepth(2));

				if(cloud->size() && _preferencesDialog->getCloudVoxelSize(2))
				{
					UDEBUG("Voxelize the cloud of %d points (%f m)", (int)cloud->size(), _preferencesDialog->getCloudVoxelSize(2));
					cloud = util3d::voxelize(cloud, _preferencesDialog->getCloudVoxelSize(2));
				}

				if(cloud->size())
				{
					clouds.insert(std::make_pair(iter->first, cloud));
					inserted = true;
				}
			}
			else
			{
				UERROR("Cloud %d not found?!?", iter->first);
			}
		}
		else
		{
			UERROR("transform is null!?");
		}

		if(inserted)
		{
			_initProgressDialog->appendText(tr("Generated cloud %1 (%2/%3).").arg(iter->first).arg(++i).arg(_currentPosesMap.size()));
		}
		else
		{
			_initProgressDialog->appendText(tr("Ignored cloud %1 (%2/%3).").arg(iter->first).arg(++i).arg(_currentPosesMap.size()));
		}
		_initProgressDialog->incrementStep();
		QApplication::processEvents();
	}

	return clouds;
}

std::map<int, pcl::PolygonMesh::Ptr> MainWindow::createMeshes()
{
	std::map<int, pcl::PolygonMesh::Ptr> meshes;
	int i=0;
	for(std::map<int, Transform>::const_iterator iter = _currentPosesMap.begin(); iter!=_currentPosesMap.end(); ++iter)
	{
		bool inserted = false;
		if(!iter->second.isNull())
		{
			if(_imagesMap.contains(iter->first) && _depthsMap.contains(iter->first))
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
				cloud = createCloud(iter->first,
						util3d::uncompressImage(_imagesMap.value(iter->first)),
						util3d::uncompressImage(_depthsMap.value(iter->first)),
						_depthConstantsMap.value(iter->first),
						_localTransformsMap.value(iter->first),
						Transform::getIdentity(),
						_preferencesDialog->getCloudVoxelSize(2),
						_preferencesDialog->getCloudDecimation(2),
						_preferencesDialog->getCloudMaxDepth(2));

				if(cloud->size() && _preferencesDialog->getCloudVoxelSize(2))
				{
					UDEBUG("Voxelize the cloud of %d points (%f m)", (int)cloud->size(), _preferencesDialog->getCloudVoxelSize(2));
					cloud = util3d::voxelize(cloud, _preferencesDialog->getCloudVoxelSize(2));
				}

				pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
				if(cloud->size())
				{
					mesh = util3d::createMesh(cloud, _preferencesDialog->getCloudVoxelSize(2)==0?0.025:_preferencesDialog->getCloudVoxelSize(2)*4);
				}

				if(mesh->polygons.size())
				{
					meshes.insert(std::make_pair(iter->first, mesh));
					inserted = true;
				}
			}
			else
			{
				UERROR("Cloud %d not found?!?", iter->first);
			}
		}
		else
		{
			UERROR("transform is null!?");
		}

		if(inserted)
		{
			_initProgressDialog->appendText(tr("Generated mesh %1 (%2/%3).").arg(iter->first).arg(++i).arg(_currentPosesMap.size()));
		}
		else
		{
			_initProgressDialog->appendText(tr("Ignored mesh %1 (%2/%3).").arg(iter->first).arg(++i).arg(_currentPosesMap.size()));
		}
		_initProgressDialog->incrementStep();
		QApplication::processEvents();
	}

	return meshes;
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
	// TODO : To protect with mutex ?
	switch (newState)
	{
	case kIdle:
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
		_ui->actionGenerate_map->setEnabled(true);
		_ui->actionGenerate_local_map->setEnabled(true);
		_ui->actionGenerate_TORO_graph_graph->setEnabled(true);
		_ui->actionOpen_working_directory->setEnabled(true);
		_ui->actionApply_settings_to_the_detector->setEnabled(true);
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->menuSelect_source->setEnabled(true);
		_ui->doubleSpinBox_stats_imgRate->setEnabled(true);
		_ui->statusbar->clearMessage();
		_state = newState;
		_oneSecondTimer->stop();
		break;

	case kStartingDetection:
		_ui->actionStart->setEnabled(false);
		break;

	case kDetecting:
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
		_ui->actionGenerate_map->setEnabled(false);
		_ui->actionGenerate_local_map->setEnabled(false);
		_ui->actionGenerate_TORO_graph_graph->setEnabled(false);
		_ui->actionOpen_working_directory->setEnabled(true);
		_ui->actionApply_settings_to_the_detector->setEnabled(false);
		_ui->actionDownload_all_clouds->setEnabled(false);
		_ui->menuSelect_source->setEnabled(false);
		_ui->doubleSpinBox_stats_imgRate->setEnabled(true);
		_ui->statusbar->showMessage(tr("Detecting..."));
		_state = newState;
		_ui->label_elapsedTime->setText("00:00:00");
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

		if(_cameraOpenni)
		{
			_cameraOpenni->start();
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
			_ui->actionGenerate_map->setEnabled(false);
			_ui->actionGenerate_local_map->setEnabled(false);
			_ui->actionGenerate_TORO_graph_graph->setEnabled(false);
			_ui->actionDownload_all_clouds->setEnabled(false);
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

			if(_cameraOpenni)
			{
				_cameraOpenni->start();
			}
		}
		else if(_state == kDetecting)
		{
			_ui->actionPause->setToolTip(tr("Continue (shift-click for step-by-step)"));
			_ui->actionPause->setChecked(true);
			_ui->statusbar->showMessage(tr("Paused..."));
			_ui->actionDump_the_memory->setEnabled(true);
			_ui->actionDump_the_prediction_matrix->setEnabled(true);
			_ui->actionDelete_memory->setEnabled(true);
			_ui->actionGenerate_map->setEnabled(true);
			_ui->actionGenerate_local_map->setEnabled(true);
			_ui->actionGenerate_TORO_graph_graph->setEnabled(true);
			_ui->actionDownload_all_clouds->setEnabled(true);
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

			if(_cameraOpenni)
			{
				_cameraOpenni->pause();
			}
		}
		break;
	case kMonitoring:
		_ui->actionStart->setVisible(false);
		_ui->actionPause->setEnabled(true);
		_ui->actionPause->setChecked(false);
		_ui->actionPause->setToolTip(tr("Pause"));
		_ui->actionStop->setVisible(false);
		_ui->actionPause_on_match->setEnabled(true);
		_ui->actionPause_on_local_loop_detection->setEnabled(true);
		_ui->actionPause_when_a_loop_hypothesis_is_rejected->setEnabled(true);
		_ui->actionReset_Odometry->setEnabled(true);
		_ui->actionDump_the_memory->setVisible(false);
		_ui->actionDump_the_prediction_matrix->setVisible(false);
		_ui->actionDelete_memory->setEnabled(true);
		_ui->actionGenerate_map->setVisible(false);
		_ui->actionGenerate_local_map->setVisible(false);
		_ui->actionGenerate_TORO_graph_graph->setVisible(false);
		_ui->actionOpen_working_directory->setEnabled(false);
		_ui->actionApply_settings_to_the_detector->setVisible(false);
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->menuSelect_source->setVisible(false);
		_ui->doubleSpinBox_stats_imgRate->setVisible(false);
		_ui->doubleSpinBox_stats_imgRate_label->setVisible(false);
		_ui->statusbar->showMessage(tr("Monitoring..."));
		_state = newState;
		_elapsedTime->start();
		_oneSecondTimer->start();
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPause, 0));
		break;
	case kMonitoringPaused:
		_ui->actionStart->setVisible(false);
		_ui->actionPause->setToolTip(tr("Continue"));
		_ui->actionPause->setChecked(true);
		_ui->actionPause->setEnabled(true);
		_ui->actionStop->setVisible(false);
		_ui->actionPause_on_match->setEnabled(true);
		_ui->actionPause_on_local_loop_detection->setEnabled(true);
		_ui->actionPause_when_a_loop_hypothesis_is_rejected->setEnabled(true);
		_ui->actionReset_Odometry->setEnabled(true);
		_ui->actionDump_the_memory->setVisible(false);
		_ui->actionDump_the_prediction_matrix->setVisible(false);
		_ui->actionDelete_memory->setEnabled(true);
		_ui->actionGenerate_map->setVisible(false);
		_ui->actionGenerate_local_map->setVisible(false);
		_ui->actionGenerate_TORO_graph_graph->setVisible(false);
		_ui->actionOpen_working_directory->setEnabled(false);
		_ui->actionApply_settings_to_the_detector->setVisible(false);
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->menuSelect_source->setVisible(false);
		_ui->doubleSpinBox_stats_imgRate->setVisible(false);
		_ui->doubleSpinBox_stats_imgRate_label->setVisible(false);
		_ui->statusbar->showMessage(tr("Monitoring paused..."));
		_state = newState;
		_oneSecondTimer->stop();
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPause, 1));
		break;
	default:
		break;
	}

}

}
