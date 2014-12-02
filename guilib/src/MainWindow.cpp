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

#include "rtabmap/core/Camera.h"
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
#include "rtabmap/core/CameraRGBD.h"
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
	_odomThread(0),
	_srcType(kSrcUndefined),
	_preferencesDialog(0),
	_aboutDialog(0),
	_exportDialog(0),
	_lastId(0),
	_processingStatistics(false),
	_odometryReceived(false),
	_openedDatabasePath(""),
	_emptyNewDatabase(true),
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
	_exportDialog = new ExportCloudsDialog(this);
	_postProcessingDialog = new PostProcessingDialog(this);

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
		_ui->dockWidget_odometry->setVisible(false);
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
	_ui->imageView_odometry->setBackgroundBrush(QBrush(Qt::black));

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
	_ui->toolBar->setWindowTitle(tr("Control toolbar"));
	QAction * a = _ui->menuShow_view->addAction("Progress dialog");
	a->setCheckable(false);
	connect(a, SIGNAL(triggered(bool)), _initProgressDialog, SLOT(show()));

	// connect actions with custom slots
	connect(_ui->actionNew_database, SIGNAL(triggered()), this, SLOT(newDatabase()));
	connect(_ui->actionOpen_database, SIGNAL(triggered()), this, SLOT(openDatabase()));
	connect(_ui->actionClose_database, SIGNAL(triggered()), this, SLOT(closeDatabase()));
	connect(_ui->actionEdit_database, SIGNAL(triggered()), this, SLOT(editDatabase()));
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
	_ui->actionSave_point_cloud->setEnabled(false);
	_ui->actionExport_2D_scans_ply_pcd->setEnabled(false);
	_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(false);
	_ui->actionView_scans->setEnabled(false);
	_ui->actionView_high_res_point_cloud->setEnabled(false);
	_ui->actionReset_Odometry->setEnabled(false);
	_ui->actionPost_processing->setEnabled(false);

#if defined(Q_WS_MAC) || defined(Q_WS_WIN)
	connect(_ui->actionOpen_working_directory, SIGNAL(triggered()), SLOT(openWorkingDirectory()));
#else
	_ui->menuEdit->removeAction(_ui->actionOpen_working_directory);
#endif

	//Settings menu
	this->updateSelectSourceImageMenu(_preferencesDialog->getSourceImageType());
	connect(_ui->actionImageFiles, SIGNAL(triggered()), this, SLOT(selectImages()));
	connect(_ui->actionVideo, SIGNAL(triggered()), this, SLOT(selectVideo()));
	connect(_ui->actionUsbCamera, SIGNAL(triggered()), this, SLOT(selectStream()));
	this->updateSelectSourceDatabase(_preferencesDialog->isSourceDatabaseUsed());
	connect(_ui->actionDatabase, SIGNAL(triggered()), this, SLOT(selectDatabase()));
	this->updateSelectSourceRGBDMenu(_preferencesDialog->isSourceOpenniUsed(), _preferencesDialog->getSourceRGBD());
	connect(_ui->actionOpenNI_PCL, SIGNAL(triggered()), this, SLOT(selectOpenni()));
	connect(_ui->actionOpenNI_PCL_ASUS, SIGNAL(triggered()), this, SLOT(selectOpenni()));
	connect(_ui->actionFreenect, SIGNAL(triggered()), this, SLOT(selectFreenect()));
	_ui->actionFreenect->setEnabled(CameraFreenect::available());
	connect(_ui->actionOpenNI_CV, SIGNAL(triggered()), this, SLOT(selectOpenniCv()));
	connect(_ui->actionOpenNI_CV_ASUS, SIGNAL(triggered()), this, SLOT(selectOpenniCvAsus()));
	_ui->actionOpenNI_CV->setEnabled(CameraOpenNICV::available());
	_ui->actionOpenNI_CV_ASUS->setEnabled(CameraOpenNICV::available());
	connect(_ui->actionOpenNI2, SIGNAL(triggered()), this, SLOT(selectOpenni2()));
	_ui->actionOpenNI2->setEnabled(CameraOpenNI2::available());
	connect(_ui->actionOpenNI2_Sense, SIGNAL(triggered()), this, SLOT(selectOpenni2()));
	_ui->actionOpenNI2_Sense->setEnabled(CameraOpenNI2::available());

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

	qRegisterMetaType<rtabmap::SensorData>("rtabmap::SensorData");
	connect(this, SIGNAL(odometryReceived(rtabmap::SensorData, int, float, int, int)), this, SLOT(processOdometry(rtabmap::SensorData, int, float, int, int)));

	connect(this, SIGNAL(noMoreImagesReceived()), this, SLOT(stopDetection()));

	// Apply state
	this->changeState(kIdle);
	this->applyPrefSettings(PreferencesDialog::kPanelAll);

	_ui->statsToolBox->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_ui->graphicsView_graphView->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
	_ui->widget_cloudViewer->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());

	if(_ui->statsToolBox->findChildren<StatItem*>().size() == 0)
	{
		const std::map<std::string, float> & statistics = Statistics::defaultData();
		for(std::map<std::string, float>::const_iterator iter = statistics.begin(); iter != statistics.end(); ++iter)
		{
			_ui->statsToolBox->updateStat(QString((*iter).first.c_str()).replace('_', ' '), 0, (*iter).second);
		}
	}

	// update loop closure viewer parameters
	ParametersMap parameters = _preferencesDialog->getAllParameters();
	_ui->widget_loopClosureViewer->setDecimation(atoi(parameters.at(Parameters::kLccIcp3Decimation()).c_str()));
	_ui->widget_loopClosureViewer->setMaxDepth(atof(parameters.at(Parameters::kLccIcp3MaxDepth()).c_str()));
	_ui->widget_loopClosureViewer->setSamples(atoi(parameters.at(Parameters::kLccIcp3Samples()).c_str()));

	//update ui
	_ui->doubleSpinBox_stats_detectionRate->setValue(_preferencesDialog->getDetectionRate());
	_ui->doubleSpinBox_stats_timeLimit->setValue(_preferencesDialog->getTimeLimit());
	_ui->actionSLAM_mode->setChecked(_preferencesDialog->isSLAMMode());

	splash.close();

	this->setFocus();
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
			this->closeDatabase();
			this->changeState(kApplicationClosing);
		}
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
		if((_ui->dockWidget_cloudViewer->isVisible() || _ui->dockWidget_odometry->isVisible()) &&
		   _lastOdometryProcessed &&
		   !_processingStatistics)
		{
			_lastOdometryProcessed = false; // if we receive too many odometry events!
			emit odometryReceived(odomEvent->data(), odomEvent->quality(), odomEvent->time(), odomEvent->features(), odomEvent->localMapSize());
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

void MainWindow::processOdometry(const rtabmap::SensorData & data, int quality, float time, int features, int localMapSize)
{
	Transform pose = data.pose();
	bool lost = false;
	_ui->imageView_odometry->resetTransform();
	if(pose.isNull())
	{
		UDEBUG("odom lost"); // use last pose
		_ui->widget_cloudViewer->setBackgroundColor(Qt::darkRed);
		_ui->imageView_odometry->setBackgroundBrush(QBrush(Qt::darkRed));

		pose = _lastOdomPose;
		lost = true;
	}
	else if(quality>=0 &&
			_preferencesDialog->getOdomQualityWarnThr() &&
			quality < _preferencesDialog->getOdomQualityWarnThr())
	{
		UDEBUG("odom warn, quality=%d thr=%d", quality, _preferencesDialog->getOdomQualityWarnThr());
		_ui->widget_cloudViewer->setBackgroundColor(Qt::darkYellow);
		_ui->imageView_odometry->setBackgroundBrush(QBrush(Qt::darkYellow));
	}
	else
	{
		UDEBUG("odom ok");
		_ui->widget_cloudViewer->setBackgroundColor(Qt::black);
		_ui->imageView_odometry->setBackgroundBrush(QBrush(Qt::black));
	}
	if(quality >= 0)
	{
		_ui->statsToolBox->updateStat("Odometry/Inliers/", (float)data.id(), (float)quality);
	}
	if(time > 0)
	{
		_ui->statsToolBox->updateStat("Odometry/Time/ms", (float)data.id(), (float)time*1000.0f);
	}
	if(features >=0)
	{
		_ui->statsToolBox->updateStat("Odometry/Features/", (float)data.id(), (float)features);
	}
	if(localMapSize >=0)
	{
		_ui->statsToolBox->updateStat("Odometry/LocalMapSize/", (float)data.id(), (float)localMapSize);
	}

	if(_ui->dockWidget_cloudViewer->isVisible())
	{
		if(!pose.isNull())
		{
			_lastOdomPose = pose;
			_odometryReceived = true;

			// 3d cloud
			if(data.depth().cols == data.image().cols &&
			   data.depth().rows == data.image().rows &&
			   !data.depth().empty() &&
			   data.fx() > 0.0f &&
			   data.fy() > 0.0f &&
			   _preferencesDialog->isCloudsShown(1))
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
				cloud = createCloud(0,
						data.image(),
						data.depth(),
						data.fx(),
						data.fy(),
						data.cx(),
						data.cy(),
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
				cloud = util3d::transformPointCloud<pcl::PointXYZ>(cloud, pose);
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
	}

	if(_ui->dockWidget_odometry->isVisible() &&
	   !data.image().empty())
	{
		if(lost)
		{
			_ui->imageView_odometry->setImageDepth(uCvMat2QImage(data.image()));
			_ui->imageView_odometry->setImageShown(true);
			_ui->imageView_odometry->setImageDepthShown(true);
		}
		else
		{
			_ui->imageView_odometry->setImage(uCvMat2QImage(data.image()));
			_ui->imageView_odometry->setImageShown(true);
			_ui->imageView_odometry->setImageDepthShown(false);
		}

		_ui->imageView_odometry->resetZoom();
		_ui->imageView_odometry->setSceneRect(_ui->imageView_odometry->scene()->itemsBoundingRect());
		_ui->imageView_odometry->fitInView(_ui->imageView_odometry->sceneRect(), Qt::KeepAspectRatio);
		if(_preferencesDialog->isImageFlipped())
		{
			_ui->imageView_odometry->scale(-1.0, 1.0);
		}
	}

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
		Signature signature = stat.getSignature();
		signature.uncompressData(); // make sure data are uncompressed
		_cachedSignatures.insert(stat.getSignature().id(), signature);

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
				shownLoopId = stat.loopClosureId()>0?stat.loopClosureId():stat.localLoopClosureId()>0?stat.localLoopClosureId():highestHypothesisId;
				QMap<int, Signature>::iterator iter = _cachedSignatures.find(shownLoopId);
				if(iter != _cachedSignatures.end())
				{
					iter.value().uncompressData();
					loopSignature = iter.value();
				}
			}
		}
		_refIds.push_back(stat.refImageId());
		_loopClosureIds.push_back(matchId);

		//update image views
		{
			UCvMat2QImageThread qimageThread(signature.getImageRaw());
			UCvMat2QImageThread qimageLoopThread(loopSignature.getImageRaw());
			UCvMat2QImageThread qdepthThread(signature.getDepthRaw());
			UCvMat2QImageThread qdepthLoopThread(loopSignature.getDepthRaw());
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
		if(signature.getImageRaw().empty())
		{
			_ui->imageView_source->setSceneRect(_ui->imageView_source->scene()->itemsBoundingRect());
			_ui->imageView_loopClosure->setSceneRect(_ui->imageView_source->scene()->itemsBoundingRect());
		}
		_ui->imageView_source->fitInView(_ui->imageView_source->sceneRect(), Qt::KeepAspectRatio);
		_ui->imageView_loopClosure->fitInView(_ui->imageView_source->sceneRect(), Qt::KeepAspectRatio);

		// do it after scaling
		this->drawKeypoints(signature.getWords(), loopSignature.getWords());

		if(_preferencesDialog->isImageFlipped())
		{
			_ui->imageView_source->scale(-1.0, 1.0);
			_ui->imageView_loopClosure->scale(-1.0, 1.0);
		}

		UDEBUG("time= %d ms", time.restart());

		_ui->statsToolBox->updateStat("Keypoint/Keypoints count in the last signature/", stat.refImageId(), signature.getWords().size());
		_ui->statsToolBox->updateStat("Keypoint/Keypoints count in the loop signature/", stat.refImageId(), loopSignature.getWords().size());

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
			// update pose only if odometry is not received
			updateMapCloud(stat.poses(),
					_odometryReceived||stat.poses().size()==0?Transform():stat.poses().rbegin()->second,
					stat.constraints(),
					stat.getMapIds());

			_odometryReceived = false;

			_odometryCorrection = stat.mapCorrection();

			UDEBUG("time= %d ms", time.restart());
			_ui->statsToolBox->updateStat("/Gui RGB-D cloud/ms", stat.refImageId(), int(timerVis.elapsed()*1000.0f));

			// loop closure view
			if((stat.loopClosureId() > 0 || stat.localLoopClosureId() > 0)  &&
			   !stat.loopClosureTransform().isNull() &&
			   !loopSignature.getImageRaw().empty())
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

void MainWindow::updateMapCloud(
		const std::map<int, Transform> & posesIn,
		const Transform & currentPose,
		const std::multimap<int, Link> & constraints,
		const std::map<int, int> & mapIdsIn,
		bool verboseProgress)
{
	if(posesIn.size())
	{
		_currentPosesMap = posesIn;
		_currentLinksMap = constraints;
		_currentMapIds = mapIdsIn;
		if(_currentPosesMap.size())
		{
			if(!_ui->actionSave_point_cloud->isEnabled() &&
				_cachedSignatures.size() &&
				!(--_cachedSignatures.end())->getDepthCompressed().empty())
			{
				//enable save cloud action
				_ui->actionSave_point_cloud->setEnabled(true);
				_ui->actionView_high_res_point_cloud->setEnabled(true);
			}

			if(!_ui->actionView_scans->isEnabled() &&
				_cachedSignatures.size() &&
				!(--_cachedSignatures.end())->getDepth2DCompressed().empty())
			{
				_ui->actionExport_2D_scans_ply_pcd->setEnabled(true);
				_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(true);
				_ui->actionView_scans->setEnabled(true);
			}
			else if(_preferencesDialog->isGridMapFrom3DCloud() && _occupancyLocalMaps.size())
			{
				_ui->actionExport_2D_Grid_map_bmp_png->setEnabled(true);
			}
		}

		_ui->actionPost_processing->setEnabled(_currentPosesMap.size() >= 2 && _currentLinksMap.size() >= 1);
	}

	// filter duplicated poses
	std::map<int, Transform> poses;
	std::map<int, int> mapIds;
	if(_preferencesDialog->isCloudFiltering() && posesIn.size())
	{
		float radius = _preferencesDialog->getCloudFilteringRadius();
		float angle = _preferencesDialog->getCloudFilteringAngle()*CV_PI/180.0; // convert to rad
		poses = util3d::radiusPosesFiltering(posesIn, radius, angle);
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
				else if(_cachedSignatures.contains(iter->first))
				{
					QMap<int, Signature>::iterator jter = _cachedSignatures.find(iter->first);
					if(!jter->getImageCompressed().empty() && !jter->getDepthCompressed().empty())
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
			if(_preferencesDialog->isScansShown(0) || _ui->graphicsView_graphView->isVisible() || _preferencesDialog->getGridMapShown())
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
					if(!jter->getDepth2DCompressed().empty())
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
				QApplication::processEvents();
			}
		}
		else
		{
			UERROR("transform is null!?");
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
	if(_preferencesDialog->isGraphsShown() && _currentPosesMap.size())
	{
		// Find all graphs
		std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > graphs;
		for(std::map<int, Transform>::iterator iter=_currentPosesMap.begin(); iter!=_currentPosesMap.end(); ++iter)
		{
			int mapId = uValue(_currentMapIds, iter->first, -1);
			std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator kter = graphs.find(mapId);
			if(kter == graphs.end())
			{
				kter = graphs.insert(std::make_pair(mapId, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>))).first;
			}
			kter->second->push_back(pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z()));
		}

		// add graphs
		for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter=graphs.begin(); iter!=graphs.end(); ++iter)
		{
			QColor color = Qt::gray;
			if(iter->first >= 0)
			{
				color = (Qt::GlobalColor)(iter->first % 12 + 7 );
			}
			_ui->widget_cloudViewer->addOrUpdateGraph(uFormat("graph_%d", iter->first), iter->second, color);
		}
	}

	// Update occupancy grid map in 3D map view and graph view
	if(_ui->graphicsView_graphView->isVisible() && constraints.size())
	{
		_ui->graphicsView_graphView->updateGraph(posesIn, constraints);
	}
	cv::Mat map8U;
	if((_ui->graphicsView_graphView->isVisible() || _preferencesDialog->getGridMapShown()) && (_createdScans.size() || _preferencesDialog->isGridMapFrom3DCloud()))
	{
		float xMin, yMin;
		float resolution = _preferencesDialog->getGridMapResolution();
		cv::Mat map8S;
		if(_preferencesDialog->isGridMapFrom3DCloud())
		{
			int fillEmptyRadius = _preferencesDialog->getGridMapFillEmptyRadius();
			map8S = util3d::create2DMapFromOccupancyLocalMaps(poses, _occupancyLocalMaps, resolution, xMin, yMin, fillEmptyRadius);
		}
		else if(_createdScans.size())
		{
			bool fillEmptySpace = _preferencesDialog->getGridMapFillEmptySpace();
			map8S = util3d::create2DMap(poses, _createdScans, resolution, fillEmptySpace, xMin, yMin);
		}
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
		_ui->widget_cloudViewer->updateCameraPosition(currentPose);
	}

	_ui->widget_cloudViewer->render();
}

void MainWindow::createAndAddCloudToMap(int nodeId, const Transform & pose, int mapId)
{
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

	if(iter->getImageCompressed().empty() || iter->getDepthCompressed().empty())
	{
		return;
	}

	cv::Mat image, depth;
	iter->uncompressData(&image, &depth, 0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	cloud = createCloud(nodeId,
			image,
			depth,
			iter->getDepthFx(),
			iter->getDepthFy(),
			iter->getDepthCx(),
			iter->getDepthCy(),
			iter->getLocalTransform(),
			Transform::getIdentity(),
			_preferencesDialog->getCloudVoxelSize(0),
			_preferencesDialog->getCloudDecimation(0),
			_preferencesDialog->getCloudMaxDepth(0));

	if(cloud->size() && _preferencesDialog->isGridMapFrom3DCloud())
	{
		UTimer timer;
		float cellSize = _preferencesDialog->getGridMapResolution();
		float groundNormalMaxAngle = M_PI_4;
		int minClusterSize = 20;
		cv::Mat ground, obstacles;
		if(util3d::occupancy2DFromCloud3D(cloud, ground, obstacles, cellSize, groundNormalMaxAngle, minClusterSize))
		{
			_occupancyLocalMaps.insert(std::make_pair(nodeId, std::make_pair(ground, obstacles)));
		}
		UDEBUG("time gridMapFrom2DCloud = %f s", timer.ticks());
	}

	if(_preferencesDialog->isCloudMeshing())
	{
		pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
		if(cloud->size())
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals;
			if(_preferencesDialog->getMeshSmoothing())
			{
				cloudWithNormals = util3d::computeNormalsSmoothed(cloud, (float)_preferencesDialog->getMeshSmoothingRadius());
			}
			else
			{
				cloudWithNormals = util3d::computeNormals(cloud, _preferencesDialog->getMeshNormalKSearch());
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
			else
			{
				_createdClouds.insert(std::make_pair(nodeId, tmp));
			}
		}
	}
	else
	{
		if(_preferencesDialog->getMeshSmoothing())
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals;
			cloudWithNormals = util3d::computeNormalsSmoothed(cloud, (float)_preferencesDialog->getMeshSmoothingRadius());
			cloud->clear();
			pcl::copyPointCloud(*cloudWithNormals, *cloud);
		}
		QColor color = Qt::gray;
		if(mapId >= 0)
		{
			color = (Qt::GlobalColor)(mapId % 12 + 7 );
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

	if(!iter->getDepth2DCompressed().empty())
	{
		cv::Mat depth2D;
		iter->uncompressData(0, 0, &depth2D);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		cloud = util3d::depth2DToPointCloud(depth2D);
		QColor color = Qt::red;
		if(mapId >= 0)
		{
			color = (Qt::GlobalColor)(mapId % 12 + 7 );
		}
		if(!_ui->widget_cloudViewer->addOrUpdateCloud(scanName, cloud, pose))
		{
			UERROR("Adding cloud %d to viewer failed!", nodeId);
		}
		else
		{
			_createdScans.insert(std::make_pair(nodeId, cloud));
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
		if(_preferencesDialog->isCloudsShown(0) && _cachedSignatures.contains(nodeId))
		{
			std::string cloudName = uFormat("cloud%d", nodeId);
			if(visible && !viewerClouds.contains(cloudName))
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

		if(_preferencesDialog->isScansShown(0) && _cachedSignatures.contains(nodeId))
		{
			std::string scanName = uFormat("scan%d", nodeId);
			if(visible && !viewerClouds.contains(scanName))
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
	_ui->widget_cloudViewer->render();
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
		if(_openedDatabasePath.compare(_preferencesDialog->getWorkingDirectory()+QDir::separator()+Parameters::getDefaultDatabaseName().c_str()) == 0 &&
		   !_emptyNewDatabase)
		{
			// Temp database used, automatically backup with unique name (timestamp)
			QString newName = _preferencesDialog->getWorkingDirectory()+QDir::separator()+(QString("rtabmap_%1.db").arg(QDateTime::currentDateTime().toString("yyMMdd-hhmmsszzz")));
			if(QFile::rename(_openedDatabasePath, newName))
			{
				std::string msg = uFormat("Database saved to \"%s\".", newName.toStdString().c_str());
				UINFO(msg.c_str());
				QMessageBox::information(this, tr("Database saved!"), QString(msg.c_str()));
			}
			else
			{
				std::string msg = uFormat("Failed to rename temporary database from \"%s\" to \"%s\".", _openedDatabasePath.toStdString().c_str(), newName.toStdString().c_str());
				UERROR(msg.c_str());
				QMessageBox::critical(this, tr("Closing failed!"), QString(msg.c_str()));
			}
		}
		_openedDatabasePath.clear();
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
		UINFO(" signatures = %d", event.getSignatures().size());
		UINFO(" map ids = %d", event.getMapIds().size());
		UINFO(" poses = %d", event.getPoses().size());
		UINFO(" constraints = %d", event.getConstraints().size());

		_initProgressDialog->setMaximumSteps(int(event.getSignatures().size()+event.getPoses().size()+1));
		_initProgressDialog->appendText(QString("Inserting data in the cache (%1 signatures downloaded)...").arg(event.getSignatures().size()));
		QApplication::processEvents();

		int addedSignatures = 0;
		for(std::map<int, Signature>::const_iterator iter = event.getSignatures().begin();
			iter!=event.getSignatures().end();
			++iter)
		{
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
			this->updateMapCloud(event.getPoses(), Transform(), event.getConstraints(), event.getMapIds(), true);
			_initProgressDialog->appendText("Updating the 3D map cloud... done.");
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

void MainWindow::applyPrefSettings(PreferencesDialog::PANEL_FLAGS flags)
{
	ULOGGER_DEBUG("");
	if(flags & PreferencesDialog::kPanelSource)
	{
		// Camera settings...
		_ui->doubleSpinBox_stats_imgRate->setValue(_preferencesDialog->getGeneralInputRate());
		this->updateSelectSourceImageMenu(_preferencesDialog->getSourceImageType());
		this->updateSelectSourceDatabase(_preferencesDialog->isSourceDatabaseUsed());
		this->updateSelectSourceRGBDMenu(_preferencesDialog->isSourceOpenniUsed(), _preferencesDialog->getSourceRGBD());
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
			_camera->setImageRate(_preferencesDialog->getGeneralInputRate());

			if(_camera->cameraRGBD() && dynamic_cast<CameraOpenNI2*>(_camera->cameraRGBD()) != 0)
			{
				((CameraOpenNI2*)_camera->cameraRGBD())->setAutoWhiteBalance(_preferencesDialog->getSourceOpenni2AutoWhiteBalance());
				((CameraOpenNI2*)_camera->cameraRGBD())->setAutoExposure(_preferencesDialog->getSourceOpenni2AutoExposure());
				if(CameraOpenNI2::exposureGainAvailable())
				{
					((CameraOpenNI2*)_camera->cameraRGBD())->setExposure(_preferencesDialog->getSourceOpenni2Exposure());
					((CameraOpenNI2*)_camera->cameraRGBD())->setGain(_preferencesDialog->getSourceOpenni2Gain());
				}
			}
		}
		if(_dbReader)
		{
			_dbReader->setFrameRate(_preferencesDialog->getGeneralInputRate());
		}
	}//This will update the statistics toolbox

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
			this->updateMapCloud(_currentPosesMap, Transform(), _currentLinksMap, _currentMapIds);
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
			this->post(new ParamEvent(parametersModified));
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
	_ui->imageView_source->fitInView(_ui->imageView_odometry->sceneRect(), Qt::KeepAspectRatio);
	_ui->imageView_source->resetZoom();
	_ui->imageView_loopClosure->resetZoom();
	_ui->imageView_odometry->resetZoom();
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

void MainWindow::updateSelectSourceRGBDMenu(bool used, PreferencesDialog::Src src)
{
	_ui->actionOpenNI_PCL->setChecked(used && src == PreferencesDialog::kSrcOpenNI_PCL);
	_ui->actionOpenNI_PCL_ASUS->setChecked(used && src == PreferencesDialog::kSrcOpenNI_PCL);
	_ui->actionFreenect->setChecked(used && src == PreferencesDialog::kSrcFreenect);
	_ui->actionOpenNI_CV->setChecked(used && src == PreferencesDialog::kSrcOpenNI_CV);
	_ui->actionOpenNI_CV_ASUS->setChecked(used && src == PreferencesDialog::kSrcOpenNI_CV_ASUS);
	_ui->actionOpenNI2->setChecked(used && src == PreferencesDialog::kSrcOpenNI2);
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

//ACTIONS
void MainWindow::newDatabase()
{
	if(_state != MainWindow::kIdle)
	{
		UERROR("This method can be called only in IDLE state.");
		return;
	}
	_openedDatabasePath.clear();
	ULOGGER_DEBUG("");
	this->clearTheCache();
	std::string databasePath = (_preferencesDialog->getWorkingDirectory()+QDir::separator()+Parameters::getDefaultDatabaseName().c_str()).toStdString();
	if(QFile::exists(databasePath.c_str()))
	{
		if(QFile::remove(databasePath.c_str()))
		{
			UINFO("Deleted database \"%s\".", databasePath.c_str());
		}
		else
		{
			UERROR("Cannot create a new database because the temporary database \"%s\" cannot be deleted.", databasePath.c_str());
			return;
		}
	}
	_openedDatabasePath = databasePath.c_str();
	_emptyNewDatabase = true;
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdInit, databasePath, 0, _preferencesDialog->getAllParameters()));
}

void MainWindow::openDatabase()
{
	if(_state != MainWindow::kIdle)
	{
		UERROR("This method can be called only in IDLE state.");
		return;
	}
	_openedDatabasePath.clear();
	QString path = QFileDialog::getOpenFileName(this, tr("Open database..."), _preferencesDialog->getWorkingDirectory(), tr("RTAB-Map database files (*.db)"));
	if(!path.isEmpty())
	{
		this->clearTheCache();
		_openedDatabasePath = path;
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdInit, path.toStdString(), 0, _preferencesDialog->getAllParameters()));
	}
}

void MainWindow::closeDatabase()
{
	if(_state != MainWindow::kInitialized)
	{
		UERROR("This method can be called only in INITIALIZED state.");
		return;
	}
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdClose));
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
		if(viewer->openDatabase(path))
		{
			viewer->show();
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
	if( !_preferencesDialog->isSourceImageUsed() &&
		!_preferencesDialog->isSourceDatabaseUsed() &&
		!_preferencesDialog->isSourceOpenniUsed())
	{
		QMessageBox::warning(this,
				 tr("RTAB-Map"),
				 tr("No sources are selected. See Preferences->Source panel."));
		UWARN("No sources are selected. See Preferences->Source panel.");
		emit stateChanged(kInitialized);
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
			if(_preferencesDialog->getOdomStrategy() == 1)
			{
				odom = new OdometryOpticalFlow(parameters);
			}
			else
			{
				odom = new OdometryBOW(parameters);
			}
			_odomThread = new OdometryThread(odom);

			UEventsManager::addHandler(_odomThread);
			_odomThread->start();
		}

		CameraRGBD * camera = 0;
		if(_preferencesDialog->getSourceRGBD() == PreferencesDialog::kSrcOpenNI_PCL)
		{
			camera = new CameraOpenni(
					_preferencesDialog->getSourceOpenniDevice().toStdString(),
					_preferencesDialog->getGeneralInputRate(),
					_preferencesDialog->getSourceOpenniLocalTransform(),
					_preferencesDialog->getSourceOpenniFx(),
					_preferencesDialog->getSourceOpenniFy(),
					_preferencesDialog->getSourceOpenniCx(),
					_preferencesDialog->getSourceOpenniCy());
		}
		else if(_preferencesDialog->getSourceRGBD() == PreferencesDialog::kSrcOpenNI2)
		{
			camera = new CameraOpenNI2(
					_preferencesDialog->getGeneralInputRate(),
					_preferencesDialog->getSourceOpenniLocalTransform(),
					_preferencesDialog->getSourceOpenniFx(),
					_preferencesDialog->getSourceOpenniFy(),
					_preferencesDialog->getSourceOpenniCx(),
					_preferencesDialog->getSourceOpenniCy());
		}
		else if(_preferencesDialog->getSourceRGBD() == PreferencesDialog::kSrcFreenect)
		{
			camera = new CameraFreenect(
					_preferencesDialog->getSourceOpenniDevice().isEmpty()?0:atoi(_preferencesDialog->getSourceOpenniDevice().toStdString().c_str()),
					_preferencesDialog->getGeneralInputRate(),
					_preferencesDialog->getSourceOpenniLocalTransform(),
					_preferencesDialog->getSourceOpenniFx(),
					_preferencesDialog->getSourceOpenniFy(),
					_preferencesDialog->getSourceOpenniCx(),
					_preferencesDialog->getSourceOpenniCy());
		}
		else if(_preferencesDialog->getSourceRGBD() == PreferencesDialog::kSrcOpenNI_CV ||
				_preferencesDialog->getSourceRGBD() == PreferencesDialog::kSrcOpenNI_CV_ASUS)
		{
			camera = new CameraOpenNICV(
					_preferencesDialog->getSourceRGBD() == PreferencesDialog::kSrcOpenNI_CV_ASUS,
					_preferencesDialog->getGeneralInputRate(),
					_preferencesDialog->getSourceOpenniLocalTransform(),
					_preferencesDialog->getSourceOpenniFx(),
					_preferencesDialog->getSourceOpenniFy(),
					_preferencesDialog->getSourceOpenniCx(),
					_preferencesDialog->getSourceOpenniCy());
		}
		else
		{
			UFATAL("RGBD Source type undefined!");
		}

		if(!camera->init())
		{
			ULOGGER_WARN("init camera failed... ");
			QMessageBox::warning(this,
								   tr("RTAB-Map"),
								   tr("Camera initialization failed..."));
			emit stateChanged(kInitialized);
			delete camera;
			camera = 0;
			if(_odomThread)
			{
				delete _odomThread;
				_odomThread = 0;
			}
			return;
		}
		else if(_preferencesDialog->getSourceRGBD() == PreferencesDialog::kSrcOpenNI2)
		{
			((CameraOpenNI2*)camera)->setAutoWhiteBalance(_preferencesDialog->getSourceOpenni2AutoWhiteBalance());
			((CameraOpenNI2*)camera)->setAutoExposure(_preferencesDialog->getSourceOpenni2AutoExposure());
			if(CameraOpenNI2::exposureGainAvailable())
			{
				((CameraOpenNI2*)camera)->setExposure(_preferencesDialog->getSourceOpenni2Exposure());
				((CameraOpenNI2*)camera)->setGain(_preferencesDialog->getSourceOpenni2Gain());
			}
		}

		_camera = new CameraThread(camera);
		if(_odomThread)
		{
			UEventsManager::createPipe(_camera, _odomThread, "CameraEvent");
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
			if(_preferencesDialog->getOdomStrategy() == 1)
			{
				odom = new OdometryOpticalFlow(parameters);
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
						_preferencesDialog->getSourceHeight());
			}
			else if(sourceType == 2)
			{
				camera = new CameraVideo(
						_preferencesDialog->getSourceVideoPath().toStdString(),
						_preferencesDialog->getGeneralInputRate(),
						_preferencesDialog->getSourceWidth(),
						_preferencesDialog->getSourceHeight());
			}
			else //if(sourceType == 0)
			{
				camera = new CameraVideo(
						_preferencesDialog->getSourceUsbDeviceId(),
						_preferencesDialog->getGeneralInputRate(),
						_preferencesDialog->getSourceWidth(),
						_preferencesDialog->getSourceHeight());
			}

			if(!camera->init())
			{
				ULOGGER_WARN("init camera failed... ");
				QMessageBox::warning(this,
									   tr("RTAB-Map"),
									   tr("Camera initialization failed..."));
				emit stateChanged(kInitialized);
				delete camera;
				camera = 0;
				return;
			}

			_camera = new CameraThread(camera);
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

	_emptyNewDatabase = false; // if a new database is used, it won't be empty anymore...

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
	emit stateChanged(kInitialized);
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

void MainWindow::postProcessing()
{
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
			if(jter->getLocalTransform().isNull())
			{
				UWARN("Local transform of %d is null.", iter->first);
				allDataAvailable = false;
			}
			if(refineNeighborLinks || refineLoopClosureLinks || reextractFeatures)
			{
				// depth data required
				if(jter->getDepthCompressed().empty() || jter->getDepthFx() <= 0.0f || jter->getDepthFy() <= 0.0f)
				{
					UWARN("Depth data of %d missing.", iter->first);
					allDataAvailable = false;
				}

				if(reextractFeatures)
				{
					// rgb required
					if(jter->getImageCompressed().empty())
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
		totalSteps+=odomPoses.size();
	}
	if(refineLoopClosureLinks)
	{
		totalSteps+=_currentLinksMap.size() - odomPoses.size();
	}
	_initProgressDialog->setMaximumSteps(totalSteps);
	_initProgressDialog->show();

	ParametersMap parameters = _preferencesDialog->getAllParameters();
	int toroIterations = 100;
	bool toroOptimizeFromGraphEnd = false;
	Parameters::parse(parameters, Parameters::kRGBDToroIterations(), toroIterations);
	Parameters::parse(parameters, Parameters::kRGBDOptimizeFromGraphEnd(), toroOptimizeFromGraphEnd);

	int loopClosuresAdded = 0;
	if(detectMoreLoopClosures)
	{
		Memory memory(parameters);
		if(reextractFeatures)
		{
			ParametersMap customParameters;
			// override some parameters
			customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
			customParameters.insert(ParametersPair(Parameters::kMemImageKept(), "false"));
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

			std::multimap<int, int> clusters = util3d::radiusPosesClustering(
					_currentPosesMap,
					clusterRadius,
					clusterAngle*CV_PI/180.0);

			_initProgressDialog->setMaximumSteps(_initProgressDialog->maximumSteps()+clusters.size());
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
				   util3d::findLink(_currentLinksMap, from, to) == _currentLinksMap.end())
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
						int inliers;
						if(reextractFeatures)
						{
							memory.init("", true); // clear previously added signatures

							// Add signatures
							SensorData dataFrom = signatureFrom.toSensorData();
							SensorData dataTo = signatureTo.toSensorData();

							if(dataFrom.isValid() &&
							   dataFrom.isMetric() &&
							   dataTo.isValid() &&
							   dataTo.isMetric() &&
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

								transform = memory.computeVisualTransform(dataTo.id(), dataFrom.id(), &rejectedMsg, &inliers);
							}
							else
							{
								UERROR("not supposed to be here!");
							}
						}
						else
						{
							transform = memory.computeVisualTransform(signatureTo, signatureFrom, &rejectedMsg, &inliers);
						}
						if(!transform.isNull())
						{
							UINFO("Added new loop closure between %d and %d.", from, to);
							addedLinks.insert(from);
							addedLinks.insert(to);
							_currentLinksMap.insert(std::make_pair(from, Link(from, to, transform, Link::kUserClosure)));
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
				std::map<int, rtabmap::Transform> optimizedPoses;
				std::map<int, int> depthGraph = util3d::generateDepthGraph(_currentLinksMap, toroOptimizeFromGraphEnd?odomPoses.rbegin()->first:odomPoses.begin()->first);
				util3d::optimizeTOROGraph(depthGraph, odomPoses, _currentLinksMap, optimizedPoses, toroIterations);
				_currentPosesMap = optimizedPoses;
				_initProgressDialog->appendText(tr("Optimizing graph with new links... done!"));
			}
		}
		UINFO("Added %d loop closures.", loopClosuresAdded);
		_initProgressDialog->appendText(tr("Total new loop closures detected=%1").arg(loopClosuresAdded));
	}

	if(refineNeighborLinks || refineLoopClosureLinks)
	{
		if(refineLoopClosureLinks)
		{
			_initProgressDialog->setMaximumSteps(_initProgressDialog->maximumSteps()+loopClosuresAdded);
		}
		_initProgressDialog->appendText(tr("Refining links..."));

		int decimation=8;
		float maxDepth=2.0f;
		float voxelSize=0.01f;
		int samples = 0;
		float minFitness = 1.0f;
		float maxCorrespondences = 0.05f;
		float icpIterations = 30;
		Parameters::parse(parameters, Parameters::kLccIcp3Decimation(), decimation);
		Parameters::parse(parameters, Parameters::kLccIcp3MaxDepth(), maxDepth);
		Parameters::parse(parameters, Parameters::kLccIcp3VoxelSize(), voxelSize);
		Parameters::parse(parameters, Parameters::kLccIcp3Samples(), samples);
		Parameters::parse(parameters, Parameters::kLccIcp3MaxFitness(), minFitness);
		Parameters::parse(parameters, Parameters::kLccIcp3MaxCorrespondenceDistance(), maxCorrespondences);
		Parameters::parse(parameters, Parameters::kLccIcp3Iterations(), icpIterations);
		bool pointToPlane = false;
		int pointToPlaneNormalNeighbors = 20;
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
					cv::Mat depthA, depthB;
					signatureFrom.uncompressData(0, &depthA, 0);
					signatureTo.uncompressData(0, &depthB, 0);

					if(depthA.type() == CV_8UC1 || depthB.type() == CV_8UC1)
					{
						QMessageBox::critical(this, tr("ICP failed"), tr("ICP cannot be done on stereo images!"));
						UERROR("ICP 3D cannot be done on stereo images! Aborting refining links with ICP...");
						break;
					}

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA = util3d::getICPReadyCloud(depthA,
							signatureFrom.getDepthFx(), signatureFrom.getDepthFy(), signatureFrom.getDepthCx(), signatureFrom.getDepthCy(),
							decimation,
							maxDepth,
							voxelSize,
							samples,
							signatureFrom.getLocalTransform());
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB = util3d::getICPReadyCloud(depthB,
							signatureTo.getDepthFx(), signatureTo.getDepthFy(), signatureTo.getDepthCx(), signatureTo.getDepthCy(),
							decimation,
							maxDepth,
							voxelSize,
							samples,
							iter->second.transform() * signatureTo.getLocalTransform());

					bool hasConverged = false;
					double fitness = -1;
					Transform transform;
					if(pointToPlane)
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr cloudANormals = util3d::computeNormals(cloudA, pointToPlaneNormalNeighbors);
						pcl::PointCloud<pcl::PointNormal>::Ptr cloudBNormals = util3d::computeNormals(cloudB, pointToPlaneNormalNeighbors);

						cloudANormals = util3d::removeNaNNormalsFromPointCloud<pcl::PointNormal>(cloudANormals);
						if(cloudA->size() != cloudANormals->size())
						{
							UWARN("removed nan normals...");
						}

						cloudBNormals = util3d::removeNaNNormalsFromPointCloud<pcl::PointNormal>(cloudBNormals);
						if(cloudB->size() != cloudBNormals->size())
						{
							UWARN("removed nan normals...");
						}

						transform = util3d::icpPointToPlane(cloudBNormals,
								cloudANormals,
								maxCorrespondences,
								icpIterations,
								hasConverged,
								fitness);
					}
					else
					{
						transform = util3d::icp(cloudB,
								cloudA,
								maxCorrespondences,
								icpIterations,
								hasConverged,
								fitness);
					}

					if(hasConverged && !transform.isNull() && fitness>=0.0f && fitness <= minFitness)
					{
						Link newLink(from, to, transform*iter->second.transform(), iter->second.type());
						iter->second = newLink;
					}
					else
					{
						UWARN("Cannot refine link %d->%d (converged=%s fitness=%f)", from, to, hasConverged?"true":"false", fitness);
					}
				}
			}
		}
		_initProgressDialog->appendText(tr("Refining links...done!"));
	}

	_initProgressDialog->appendText(tr("Optimizing graph with updated links (%1 nodes, %2 constraints)...")
			.arg(odomPoses.size()).arg(_currentLinksMap.size()));
	std::map<int, rtabmap::Transform> optimizedPoses;
	std::map<int, int> depthGraph = util3d::generateDepthGraph(_currentLinksMap, toroOptimizeFromGraphEnd?odomPoses.rbegin()->first:odomPoses.begin()->first);
	util3d::optimizeTOROGraph(depthGraph, odomPoses, _currentLinksMap, optimizedPoses, toroIterations);
	_initProgressDialog->appendText(tr("Optimizing graph with updated links... done!"));
	_initProgressDialog->incrementStep();

	_initProgressDialog->appendText(tr("Updating map..."));
	this->updateMapCloud(optimizedPoses, Transform(), _currentLinksMap, _currentMapIds, false);
	_initProgressDialog->appendText(tr("Updating map... done!"));

	_initProgressDialog->setValue(_initProgressDialog->maximumSteps());
	_initProgressDialog->appendText("Post-processing finished!");
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
	if(_state != kMonitoring && _state != kMonitoringPaused && !_openedDatabasePath.isEmpty())
	{
		_ui->actionDelete_memory->setText(tr("Delete memory (%1 MB)").arg(UFile::length(_openedDatabasePath.toStdString())/1000000));
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
	_preferencesDialog->selectSourceRGBD(PreferencesDialog::kSrcOpenNI_PCL);
}

void MainWindow::selectFreenect()
{
	_preferencesDialog->selectSourceRGBD(PreferencesDialog::kSrcFreenect);
}

void MainWindow::selectOpenniCv()
{
	_preferencesDialog->selectSourceRGBD(PreferencesDialog::kSrcOpenNI_CV);
}

void MainWindow::selectOpenniCvAsus()
{
	_preferencesDialog->selectSourceRGBD(PreferencesDialog::kSrcOpenNI_CV_ASUS);
}

void MainWindow::selectOpenni2()
{
	_preferencesDialog->selectSourceRGBD(PreferencesDialog::kSrcOpenNI2);
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
	_occupancyLocalMaps.clear();
	_ui->widget_cloudViewer->removeAllClouds();
	_ui->widget_cloudViewer->removeAllGraphs();
	_ui->widget_cloudViewer->setBackgroundColor(Qt::black);
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
	_ui->imageView_source->resetTransform();
	_ui->imageView_loopClosure->resetTransform();
	_ui->imageView_odometry->resetTransform();
	_ui->imageView_source->setBackgroundBrush(QBrush(Qt::black));
	_ui->imageView_loopClosure->setBackgroundBrush(QBrush(Qt::black));
	_ui->imageView_odometry->setBackgroundBrush(QBrush(Qt::black));
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

void MainWindow::exportGridMap()
{
	double gridCellSize = 0.05;
	bool gridUnknownSpaceFilled = true;
	bool ok;
	gridCellSize = QInputDialog::getDouble(this, tr("Grid cell size"), tr("Size (m):"), gridCellSize, 0.01, 1, 2, &ok);
	if(!ok)
	{
		return;
	}

	QMessageBox::StandardButton b = QMessageBox::question(this,
			tr("Fill empty space?"),
			tr("Do you want to fill empty space?"),
			QMessageBox::No | QMessageBox::Yes,
			QMessageBox::Yes);

	if(b != QMessageBox::Yes && b != QMessageBox::No)
	{
		return;
	}
	gridUnknownSpaceFilled = b == QMessageBox::Yes;

	std::map<int, Transform> poses = _ui->widget_mapVisibility->getVisiblePoses();

	// create the map
	float xMin=0.0f, yMin=0.0f;
	cv::Mat pixels;
	if(_preferencesDialog->isGridMapFrom3DCloud())
	{
		pixels = util3d::create2DMapFromOccupancyLocalMaps(poses, _occupancyLocalMaps, gridCellSize, xMin, yMin, gridUnknownSpaceFilled?1:0);
	}
	else
	{
		pixels = util3d::create2DMap(poses, _createdScans, gridCellSize, gridUnknownSpaceFilled, xMin, yMin);
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
					*assembledScans += *util3d::transformPointCloud<pcl::PointXYZ>(scan, iter->second);;

					if(count++ % 100 == 0)
					{
						if(assembledScans->size() && voxel)
						{
							assembledScans = util3d::voxelize<pcl::PointXYZ>(assembledScans, voxel);
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
			assembledScans = util3d::voxelize<pcl::PointXYZ>(assembledScans, voxel);
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
	if(_camera)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save to..."), "output.db", "RTAB-Map database (*.db)");
		if(!path.isEmpty())
		{
			int r = QMessageBox::question(this, tr("Hard drive or RAM?"), tr("Save in RAM?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

			if(r == QMessageBox::No || r == QMessageBox::Yes)
			{
				bool recordInRAM = r == QMessageBox::Yes;
				QWidget * window = new QWidget(this, Qt::Popup);
				window->setAttribute(Qt::WA_DeleteOnClose);
				window->setWindowFlags(Qt::Dialog);
				window->setWindowTitle(tr("Data recorder (%1)").arg(path));

				DataRecorder * recorder = new DataRecorder(window);

				QVBoxLayout *layout = new QVBoxLayout();
				layout->addWidget(recorder);
				window->setLayout(layout);

				if(recorder->init(path, recordInRAM))
				{
					window->show();
					recorder->registerToEventsManager();
					UEventsManager::createPipe(_camera, recorder, "CameraEvent");
				}
				else
				{
					QMessageBox::warning(this, tr(""), tr("Cannot initialize the data recorder!"));
					UERROR("Cannot initialize the data recorder!");
					delete window;
				}
			}
		}
	}
	else
	{
		UERROR("Camera should be already created.");
	}
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
							transformedCloud = util3d::transformPointCloud<pcl::PointXYZRGB>(iter->second, _currentPosesMap.at(iter->first));

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
						tmp = util3d::transformPointCloud<pcl::PointXYZRGB>(tmp, _currentPosesMap.at(iter->first));
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
							transformedCloud = util3d::transformPointCloud<pcl::PointXYZ>(iter->second, _currentPosesMap.at(iter->first));

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MainWindow::createCloud(
		int id,
		const cv::Mat & rgb,
		const cv::Mat & depth,
		float fx,
		float fy,
		float cx,
		float cy,
		const Transform & localTransform,
		const Transform & pose,
		float voxelSize,
		int decimation,
		float maxDepth) const
{
	UTimer timer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	if(depth.type() == CV_8UC1)
	{
		cloud = util3d::cloudFromStereoImages(
				rgb,
				depth,
				cx, cy,
				fx, fy,
				decimation);
	}
	else
	{
		cloud = util3d::cloudFromDepthRGB(
				rgb,
				depth,
				cx, cy,
				fx, fy,
				decimation);
	}

	if(cloud->size())
	{
		bool filtered = false;
		if(cloud->size() && maxDepth)
		{
			cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, maxDepth);
			filtered = true;
		}

		if(cloud->size() && voxelSize)
		{
			cloud = util3d::voxelize<pcl::PointXYZRGB>(cloud, voxelSize);
			filtered = true;
		}

		if(cloud->size() && !filtered)
		{
			cloud = util3d::removeNaNFromPointCloud<pcl::PointXYZRGB>(cloud);
		}

		if(cloud->size())
		{
			cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, pose * localTransform);
		}
	}
	UDEBUG("Generated cloud %d (pts=%d) time=%fs", id, (int)cloud->size(), timer.ticks());
	return cloud;
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
			if(_cachedSignatures.contains(iter->first))
			{
				const Signature & s = _cachedSignatures.find(iter->first).value();
				cv::Mat image, depth;
				s.uncompressDataConst(&image, &depth, 0);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
				if(regenerateClouds)
				{
					cloud = createCloud(iter->first,
							image,
							depth,
							s.getDepthFx(),
							s.getDepthFy(),
							s.getDepthCx(),
							s.getDepthCy(),
							s.getLocalTransform(),
							iter->second,
							regenerateVoxelSize,
							regenerateDecimation,
							regenerateMaxDepth);
				}
				else if(uContains(_createdClouds, iter->first))
				{
					cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(_createdClouds.at(iter->first), iter->second);
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
			_initProgressDialog->appendText(tr("Generated cloud %1 (%2/%3).").arg(iter->first).arg(++i).arg(poses.size()));

			if(count % 100 == 0)
			{
				if(assembledCloud->size() && assembledVoxelSize)
				{
					assembledCloud = util3d::voxelize<pcl::PointXYZRGB>(assembledCloud, assembledVoxelSize);
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
		assembledCloud = util3d::voxelize<pcl::PointXYZRGB>(assembledCloud, assembledVoxelSize);
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
			if(_cachedSignatures.contains(iter->first))
			{
				const Signature & s = _cachedSignatures.find(iter->first).value();
				cv::Mat image, depth;
				s.uncompressDataConst(&image, &depth, 0);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
				if(regenerateClouds)
				{
					cloud = createCloud(iter->first,
							image,
							depth,
							s.getDepthFx(),
							s.getDepthFy(),
							s.getDepthCx(),
							s.getDepthCy(),
							s.getLocalTransform(),
							Transform::getIdentity(),
							regenerateVoxelSize,
							regenerateDecimation,
							regenerateMaxDepth);
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
				UERROR("Cloud %d not found?!?", iter->first);
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
	// TODO : To protect with mutex ?
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
		_ui->actionGenerate_map->setEnabled(false);
		_ui->actionGenerate_local_map->setEnabled(false);
		_ui->actionGenerate_TORO_graph_graph->setEnabled(false);
		_ui->actionData_recorder->setEnabled(false);
		_ui->actionOpen_working_directory->setEnabled(true);
		_ui->actionDownload_all_clouds->setEnabled(false);
		_ui->actionDownload_graph->setEnabled(false);
		_ui->menuSelect_source->setEnabled(false);
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
		_ui->actionGenerate_map->setEnabled(true);
		_ui->actionGenerate_local_map->setEnabled(true);
		_ui->actionGenerate_TORO_graph_graph->setEnabled(true);
		_ui->actionData_recorder->setEnabled(false);
		_ui->actionOpen_working_directory->setEnabled(true);
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->actionDownload_graph->setEnabled(true);
		_ui->menuSelect_source->setEnabled(true);
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
		_ui->actionGenerate_map->setEnabled(false);
		_ui->actionGenerate_local_map->setEnabled(false);
		_ui->actionGenerate_TORO_graph_graph->setEnabled(false);
		_ui->actionData_recorder->setEnabled(true);
		_ui->actionOpen_working_directory->setEnabled(true);
		_ui->actionDownload_all_clouds->setEnabled(false);
		_ui->actionDownload_graph->setEnabled(false);
		_ui->menuSelect_source->setEnabled(false);
		_ui->actionTrigger_a_new_map->setEnabled(true);
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
			_ui->actionDelete_memory->setEnabled(true);
			_ui->actionGenerate_map->setEnabled(true);
			_ui->actionGenerate_local_map->setEnabled(true);
			_ui->actionGenerate_TORO_graph_graph->setEnabled(true);
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
		_ui->actionNew_database->setVisible(false);
		_ui->actionOpen_database->setVisible(false);
		_ui->actionClose_database->setVisible(false);
		_ui->actionEdit_database->setVisible(false);
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
		_ui->actionData_recorder->setVisible(false);
		_ui->actionOpen_working_directory->setEnabled(false);
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->actionDownload_graph->setEnabled(true);
		_ui->menuSelect_source->setVisible(false);
		_ui->actionTrigger_a_new_map->setEnabled(true);
		_ui->doubleSpinBox_stats_imgRate->setVisible(false);
		_ui->doubleSpinBox_stats_imgRate_label->setVisible(false);
		_ui->statusbar->showMessage(tr("Monitoring..."));
		_state = newState;
		_elapsedTime->start();
		_oneSecondTimer->start();
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdPause, "", 0));
		break;
	case kMonitoringPaused:
		_ui->actionNew_database->setVisible(false);
		_ui->actionOpen_database->setVisible(false);
		_ui->actionClose_database->setVisible(false);
		_ui->actionEdit_database->setVisible(false);
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
		_ui->actionData_recorder->setVisible(false);
		_ui->actionOpen_working_directory->setEnabled(false);
		_ui->actionDownload_all_clouds->setEnabled(true);
		_ui->actionDownload_graph->setEnabled(true);
		_ui->menuSelect_source->setVisible(false);
		_ui->actionTrigger_a_new_map->setEnabled(true);
		_ui->doubleSpinBox_stats_imgRate->setVisible(false);
		_ui->doubleSpinBox_stats_imgRate_label->setVisible(false);
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
