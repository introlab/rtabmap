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
#include "rtabmap/core/Micro.h"
#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/SensorimotorEvent.h"
#include "CameraMicro.h"
#include "rtabmap/gui/qtipl.h"
#include "rtabmap/gui/KeypointItem.h"
#include "AboutDialog.h"
#include <utilite/UtiLite.h>
#include <utilite/UPlot.h>
#include "rtabmap/core/Parameters.h"
#include "rtabmap/gui/ImageView.h"
#include "PdfPlot.h"
#include "StatsToolBox.h"
#include "DetailedProgressDialog.h"
#include "TwistWidget.h"
#include "rtabmap/core/Sensor.h"


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
	_mic(0),
	_cameraMic(0),
	_dbReader(0),
	_srcType(kSrcUndefined),
	_preferencesDialog(0),
	_aboutDialog(0),
	_lastId(0),
	_processingStatistics(false),
	_oneSecondTimer(0),
	_elapsedTime(0),
	_posteriorCurve(0),
	_likelihoodCurve(0),
	_autoScreenCaptureFormat("png")
{
	ULOGGER_DEBUG("");

	initGuiResource();

	QPixmap pixmap(":images/RTAB-Map.png");
	QSplashScreen splash(pixmap);
	splash.show();
	splash.showMessage(tr("Loading..."));
	QApplication::processEvents();

	this->setWindowTitle(tr("Real-Time Appearance-Based Mapping"));
	this->setWindowIconText(tr("RTAB-Map"));

	// Create dialogs
	_aboutDialog = new AboutDialog(this);

	_ui = new Ui_mainWindow();
	_ui->setupUi(this);

	//Setup dock widgets position if it is the first time the application is started.
	//if(!QFile::exists(PreferencesDialog::getIniFilePath()))
	{
		_ui->dockWidget_posterior->setVisible(false);
		_ui->dockWidget_likelihood->setVisible(false);
		_ui->dockWidget_statsV2->setVisible(false);
		_ui->dockWidget_console->setVisible(false);
		_ui->dockWidget_twist->setVisible(false);
		_ui->dockWidget_audioLoopFrames->setVisible(false);
		_ui->dockWidget_audioProcessedFrames->setVisible(false);
	}

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
	_ui->posteriorPlot->addCurve(_posteriorCurve);
	_ui->posteriorPlot->showLegend(false);
	_ui->posteriorPlot->setFixedYAxis(0,1);
	UPlotCurveThreshold * tc;
	tc = _ui->posteriorPlot->addThreshold("Loop closure thr", float(_preferencesDialog->getLoopThr()));
	connect(this, SIGNAL(loopClosureThrChanged(float)), tc, SLOT(setThreshold(float)));
	tc = _ui->posteriorPlot->addThreshold("Retrieval thr", float(_preferencesDialog->getRetrievalThr()));
	connect(this, SIGNAL(retrievalThrChanged(float)), tc, SLOT(setThreshold(float)));

	_likelihoodCurve = new PdfPlotCurve("Likelihood", &_imagesMap, this);
	_ui->likelihoodPlot->addCurve(_likelihoodCurve);
	_ui->likelihoodPlot->showLegend(false);

	_ui->widget_audioProcessedFrames->setScaled(true, true);
	_ui->widget_audioProcessedFrames->setOnlyLastFramesDrawn(true);
	_ui->widget_audioLoopFrames->setScaled(true, true);
	_ui->widget_audioLoopFrames->setOnlyLastFramesDrawn(true);

	_ui->doubleSpinBox_stats_imgRate->setValue(_preferencesDialog->getGeneralInputRate());
	_ui->doubleSpinBox_stats_timeLimit->setValue(_preferencesDialog->getTimeLimit());

	_initProgressDialog = new DetailedProgressDialog(this);
	_initProgressDialog->setWindowTitle(tr("Initialization"));
	_initProgressDialog->setMinimumWidth(400);
	_initProgressDialog->setAutoClose(true, 1);

	//connect stuff
	connect(_ui->actionExit, SIGNAL(triggered()), this, SLOT(close()));
	qRegisterMetaType<MainWindow::State>("MainWindow::State");
	connect(this, SIGNAL(stateChanged(MainWindow::State)), this, SLOT(changeState(MainWindow::State)));
	connect(this, SIGNAL(rtabmapEventInitReceived(int, const QString &)), this, SLOT(processRtabmapEventInit(int, const QString &)));

	// Dock Widget view actions (Menu->Window)
	_ui->menuShow_view->addAction(_ui->dockWidget_posterior->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_likelihood->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_statsV2->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_console->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_twist->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_audioLoopFrames->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->dockWidget_audioProcessedFrames->toggleViewAction());
	_ui->menuShow_view->addAction(_ui->toolBar->toggleViewAction());
	_ui->toolBar->setWindowTitle(tr("Control toolbar"));
	QAction * a = _ui->menuShow_view->addAction("Status");
	a->setCheckable(false);
	connect(a, SIGNAL(triggered(bool)), _initProgressDialog, SLOT(show()));

	// connect actions with custom slots
	connect(_ui->actionStart, SIGNAL(triggered()), this, SLOT(startDetection()));
	connect(_ui->actionPause, SIGNAL(triggered()), this, SLOT(pauseDetection()));
	connect(_ui->actionStop, SIGNAL(triggered()), this, SLOT(stopDetection()));
	connect(_ui->actionReset_the_memory, SIGNAL(triggered()), this, SLOT(resetTheMemory()));
	connect(_ui->actionDump_the_memory, SIGNAL(triggered()), this, SLOT(dumpTheMemory()));
	connect(_ui->actionDump_the_prediction_matrix, SIGNAL(triggered()), this, SLOT(dumpThePrediction()));
	connect(_ui->actionClear_cache, SIGNAL(triggered()), this, SLOT(clearTheCache()));
	connect(_ui->actionAbout, SIGNAL(triggered()), _aboutDialog , SLOT(exec()));
	connect(_ui->actionGenerate_map, SIGNAL(triggered()), this , SLOT(generateMap()));
	connect(_ui->actionDelete_memory, SIGNAL(triggered()), this , SLOT(deleteMemory()));
	connect(_ui->menuEdit, SIGNAL(aboutToShow()), this, SLOT(updateEditMenu()));
	connect(_ui->actionAuto_screen_capture, SIGNAL(triggered(bool)), this, SLOT(selectScreenCaptureFormat(bool)));
	connect(_ui->action16_9, SIGNAL(triggered()), this, SLOT(setAspectRatio16_9()));
	connect(_ui->action16_10, SIGNAL(triggered()), this, SLOT(setAspectRatio16_10()));
	connect(_ui->action4_3, SIGNAL(triggered()), this, SLOT(setAspectRatio4_3()));
	connect(_ui->action240p, SIGNAL(triggered()), this, SLOT(setAspectRatio240p()));
	connect(_ui->action360p, SIGNAL(triggered()), this, SLOT(setAspectRatio360p()));
	connect(_ui->action480p, SIGNAL(triggered()), this, SLOT(setAspectRatio480p()));
	connect(_ui->action720p, SIGNAL(triggered()), this, SLOT(setAspectRatio720p()));
	connect(_ui->action1080p, SIGNAL(triggered()), this, SLOT(setAspectRatio1080p()));

	_ui->actionPause->setShortcut(Qt::Key_Space);

#if defined(Q_WS_MAC) or defined(Q_WS_WIN)
	connect(_ui->actionOpen_working_directory, SIGNAL(triggered()), SLOT(openWorkingDirectory()));
#else
	_ui->menuEdit->removeAction(_ui->actionOpen_working_directory);
#endif

	//Settings menu
	_selectSourceImageGrp = new QActionGroup(this);
	_selectSourceImageGrp->addAction(_ui->actionUsbCamera);
	_selectSourceImageGrp->addAction(_ui->actionImageFiles);
	_selectSourceImageGrp->addAction(_ui->actionVideo);
	this->updateSelectSourceImageMenu(_preferencesDialog->getSourceImageType());
	connect(_ui->actionImageFiles, SIGNAL(triggered()), this, SLOT(selectImages()));
	connect(_ui->actionVideo, SIGNAL(triggered()), this, SLOT(selectVideo()));
	connect(_ui->actionUsbCamera, SIGNAL(triggered()), this, SLOT(selectStream()));
	_selectSourceAudioGrp = new QActionGroup(this);
	_selectSourceAudioGrp->addAction(_ui->actionMic);
	_selectSourceAudioGrp->addAction(_ui->actionAudioFile);
	this->updateSelectSourceAudioMenu(_preferencesDialog->getSourceAudioType());
	connect(_ui->actionMic, SIGNAL(triggered()), this, SLOT(selectMic()));
	connect(_ui->actionAudioFile, SIGNAL(triggered()), this, SLOT(selectAudioFile()));
	this->updateSelectSourceDatabase(_preferencesDialog->isSourceDatabaseUsed());
	connect(_ui->actionDatabase, SIGNAL(triggered()), this, SLOT(selectDatabase()));

	connect(_ui->actionSave_state, SIGNAL(triggered()), this, SLOT(saveFigures()));
	connect(_ui->actionLoad_state, SIGNAL(triggered()), this, SLOT(loadFigures()));

	connect(_ui->actionPreferences, SIGNAL(triggered()), _preferencesDialog, SLOT(exec()));

	// Settings changed
	qRegisterMetaType<PreferencesDialog::PANEL_FLAGS>("PreferencesDialog::PANEL_FLAGS");
	connect(_preferencesDialog, SIGNAL(settingsChanged(PreferencesDialog::PANEL_FLAGS)), this, SLOT(applyPrefSettings(PreferencesDialog::PANEL_FLAGS)));
	qRegisterMetaType<rtabmap::ParametersMap>("rtabmap::ParametersMap");
	connect(_preferencesDialog, SIGNAL(settingsChanged(rtabmap::ParametersMap)), this, SLOT(applyPrefSettings(rtabmap::ParametersMap)));
	connect(_ui->actionApply_settings_to_the_detector, SIGNAL(triggered()), this, SLOT(applyAllPrefSettings()));

	// more connects...
	connect(_ui->doubleSpinBox_stats_imgRate, SIGNAL(editingFinished()), this, SLOT(changeImgRateSetting()));
	connect(_ui->doubleSpinBox_stats_timeLimit, SIGNAL(editingFinished()), this, SLOT(changeTimeLimitSetting()));
	connect(this, SIGNAL(imgRateChanged(double)), _preferencesDialog, SLOT(setInputRate(double)));
	connect(this, SIGNAL(timeLimitChanged(float)), _preferencesDialog, SLOT(setTimeLimit(float)));

	connect(this, SIGNAL(twistReceived(float, float, float, float, float, float, int, int)), _ui->twistWidget, SLOT(addTwist(float, float, float, float, float, float, int, int)));
	// Statistics from the detector
	qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");
	connect(this, SIGNAL(statsReceived(rtabmap::Statistics)), this, SLOT(processStats(rtabmap::Statistics)));

	connect(this, SIGNAL(noMoreImagesReceived()), this, SLOT(stopDetection()));

	// Apply state
	this->changeState(kIdle);
	this->applyPrefSettings(PreferencesDialog::kPanelAll);

	_ui->statsToolBox->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());

	splash.close();
}

MainWindow::~MainWindow()
{
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
		qobject_cast<QHBoxLayout *>(_ui->centralwidget->layout())->setDirection(QBoxLayout::TopToBottom);
	}
	else if(!vertical)
	{
		qobject_cast<QHBoxLayout *>(_ui->centralwidget->layout())->setDirection(QBoxLayout::LeftToRight);
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
	bool processStopped = true;
	if(_state != kIdle && _state != kMonitoring)
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

		_ui->dockWidget_likelihood->close();
		_ui->dockWidget_posterior->close();
		_ui->dockWidget_statsV2->close();
		_ui->dockWidget_console->close();

		if(_camera)
		{
			UERROR("Camera must be already deleted here!");
			delete _camera;
			_camera = 0;
		}
		if(_mic)
		{
			UERROR("Mic must be already deleted here!");
			delete _mic;
			_mic = 0;
		}
		if(_cameraMic)
		{
			UERROR("CameraMic must be already deleted here!");
			delete _mic;
			_mic = 0;
		}
		if(_dbReader)
		{
			UERROR("DBReader must be already deleted here!");
			delete _dbReader;
			_dbReader = 0;
		}
		event->accept();
	}
	else
	{
		event->ignore();
	}
}

void MainWindow::handleEvent(UEvent* anEvent)
{
	if(anEvent->getClassName().compare("RtabmapEvent") == 0)
	{
		RtabmapEvent * rtabmapEvent = (RtabmapEvent*)anEvent;
		Statistics stats = rtabmapEvent->getStats();
		int highestHypothesisId = int(uValue(stats.data(), Statistics::kLoopHighest_hypothesis_id(), 0.0f));
		bool rejectedHyp = bool(uValue(stats.data(), Statistics::kLoopRejectedHypothesis(), 0.0f));
		if((stats.loopClosureId() > 0 && _ui->actionPause_on_match->isChecked()) ||
		   (stats.loopClosureId() == 0 && highestHypothesisId > 0 && _ui->actionPause_when_a_loop_hypothesis_is_rejected->isChecked() && rejectedHyp))
		{
			if(_state != kPaused)
			{
				if(_preferencesDialog->beepOnPause())
				{
					QMetaObject::invokeMethod(this, "beep");
				}
				this->pauseDetection();
			}
		}

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
		if(_processingStatistics || (!_ui->dockWidget_posterior->isVisible() && !_ui->dockWidget_likelihood->isVisible()))
		{
			stats.setWeights(std::map<int,int>());
		}

		emit statsReceived(stats);
	}
	else if(anEvent->getClassName().compare("RtabmapEventInit") == 0)
	{
		RtabmapEventInit * rtabmapEventInit = (RtabmapEventInit*)anEvent;
		emit rtabmapEventInitReceived(rtabmapEventInit->getStatus(), rtabmapEventInit->getInfo().c_str());

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
	else if(anEvent->getClassName().compare("MicroEvent") == 0)
	{
		MicroEvent * microEvent = (MicroEvent*)anEvent;
		if(microEvent->getCode() == MicroEvent::kTypeNoMoreFrames)
		{
			if(_preferencesDialog->beepOnPause())
			{
				QMetaObject::invokeMethod(this, "beep");
			}
			emit noMoreImagesReceived();
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
	else if(anEvent->getClassName().compare("SensorimotorEvent") == 0)
	{
		SensorimotorEvent * e = (rtabmap::SensorimotorEvent*)anEvent;

		if(e->getCode() == SensorimotorEvent::kTypeNoMoreData)
		{
			if(_preferencesDialog->beepOnPause())
			{
				QMetaObject::invokeMethod(this, "beep");
			}
			emit noMoreImagesReceived();
		}
	}
}

void MainWindow::processStats(const rtabmap::Statistics & stat)
{
	_processingStatistics = true;
	ULOGGER_DEBUG("");
	QTime time;
	time.start();
	//Affichage des stats et images
	if(stat.extended())
	{
		UDEBUG("");
		_ui->label_refId->setText(QString("New ID = %1").arg(stat.refImageId()));
		int highestHypothesisId = static_cast<float>(uValue(stat.data(), Statistics::kLoopHighest_hypothesis_id(), 0.0f));
		bool refParentId = (int)uValue(stat.data(), Statistics::kParent_id(), 0.0f);
		bool highestHypothesisIsSaved = (bool)uValue(stat.data(), Statistics::kHypothesis_reactivated(), 0.0f);

		// Loop closure info
		_ui->imageView_source->scene()->clear();
		_ui->imageView_loopClosure->scene()->clear();
		_ui->imageView_source->resetTransform();
		_ui->imageView_loopClosure->resetTransform();
		_ui->imageView_loopClosure->setBackgroundBrush(QBrush(Qt::black));

		// extract some sensors
		cv::Mat refImage;
		cv::Mat refAudioFrame;
		bool twistSet = false;
		for(std::list<Sensor>::const_iterator iter = stat.refRawData().begin(); iter!=stat.refRawData().end(); ++iter)
		{
			if(refImage.empty() && iter->type() == Sensor::kTypeImage)
			{
				refImage = iter->data();
			}
			else if(refAudioFrame.empty() && iter->type() == Sensor::kTypeAudioFreqSqrdMagn)
			{
				refAudioFrame = iter->data();
			}
			else if(iter->type() == Sensor::kTypeTwist)
			{
				if(!twistSet)
				{
					const Sensor & a = *iter;
					if(a.data().total() == 6 && (a.data().type() & CV_32F))
					{
						emit twistReceived(a.data().at<float>(0),
								a.data().at<float>(1),
								a.data().at<float>(2),
								a.data().at<float>(3),
								a.data().at<float>(4),
								a.data().at<float>(5),
								0,
								0);
					}
					else
					{
						UERROR("Twist must be CV_32F and have 6 elements (%d)", a.data().total());
					}
					twistSet = true; // Show only the first
				}
			}
			else if(iter->type() != Sensor::kTypeImageFeatures2d)
			{
				UWARN("Source type %d not handled", iter->type());
			}
		}
		cv::Mat loopImage;
		cv::Mat loopAudioFrame;
		for(std::list<Sensor>::const_iterator iter = stat.loopClosureRawData().begin(); iter!=stat.loopClosureRawData().end(); ++iter)
		{
			if(loopImage.empty() && iter->type() == Sensor::kTypeImage)
			{
				loopImage = iter->data();
			}
			else if(loopAudioFrame.empty() && iter->type() == Sensor::kTypeAudioFreqSqrdMagn)
			{
				loopAudioFrame = iter->data();
			}
			else if(iter->type() == Sensor::kTypeTwist)
			{
				// do nothing
			}
			else if(iter->type() != Sensor::kTypeImageFeatures2d)
			{
				UWARN("Source type %d not handled", iter->type());
			}
		}

		_ui->label_matchId->clear();
		QPixmap refPixmap;
		_ui->label_stats_imageNumber->setText(QString::number(stat.refImageId()));
		if(!refImage.empty())
		{
			IplImage iplImg = refImage;
			QImage img = Ipl2QImage(&iplImg);
			//Only kept if it not added as a child in the core, that
			//means it will never used for a loop closure detection
			if(_preferencesDialog->isImagesKept())
			{
				QByteArray ba;
				QBuffer buffer(&ba);
				buffer.open(QIODevice::WriteOnly);
				img.save(&buffer, "JPEG"); // writes image into JPEG format
				_imagesMap.insert(stat.refImageId(), ba);
			}
			QRectF sceneRect = img.rect();
			refPixmap = QPixmap::fromImage(img);
			_ui->imageView_source->scene()->addPixmap(refPixmap)->setVisible(this->_ui->imageView_source->isImageShown());
			_ui->imageView_source->setSceneRect(sceneRect);
			_ui->imageView_loopClosure->setSceneRect(sceneRect);

			UDEBUG("Adding mask = %d", stat.refMotionMask().size());
			if(stat.refMotionMask().size() == (unsigned int)img.width()*img.height())
			{
				UDEBUG("Adding mask");
				const std::vector<unsigned char> & maskData = stat.refMotionMask();
				QImage mask(img.size(), QImage::Format_ARGB32);
				int i=0;
				for(int y=0; y<mask.height(); ++y)
				{
					for(int x=0; x<mask.width(); ++x)
					{
						mask.setPixel(x,y,qRgba(255,0,255,!maskData[i++]*_preferencesDialog->getKeypointsOpacity()));
					}
				}
				_ui->imageView_source->scene()->addPixmap(QPixmap::fromImage(mask));
			}
		}

		QImage lcImg;
		if(!loopImage.empty())
		{
			IplImage iplImg = loopImage;
			lcImg = Ipl2QImage(&iplImg);
			//Only kept if it not added as a child in the core, that
			//means it will never used for a loop closure detection
			if(_preferencesDialog->isImagesKept())
			{
				if(stat.loopClosureId()>0 || highestHypothesisId>0)
				{
					int id = stat.loopClosureId()>0?stat.loopClosureId():highestHypothesisId;
					if(!_imagesMap.contains(id))
					{
						QByteArray ba;
						QBuffer buffer(&ba);
						buffer.open(QIODevice::WriteOnly);
						lcImg.save(&buffer, "JPEG"); // writes image into JPEG format
						_imagesMap.insert(id, ba);
					}
				}
			}
		}

		int rejectedHyp = bool(uValue(stat.data(), Statistics::kLoopRejectedHypothesis(), 0.0f));
		float highestHypothesisValue = uValue(stat.data(), Statistics::kLoopHighest_hypothesis_value(), 0.0f);
		if(highestHypothesisId > 0)
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
				_ui->label_matchId->setText(QString("Match ID = %1").arg(stat.loopClosureId()));
			}
			else if(rejectedHyp && highestHypothesisValue >= _preferencesDialog->getLoopThr())
			{
				show = _preferencesDialog->imageRejectedShown() || _preferencesDialog->imageHighestHypShown();;
				if(show)
				{
					QColor color(Qt::red);
					_ui->imageView_loopClosure->setBackgroundBrush(QBrush(color));
					_ui->label_stats_loopClosuresRejected->setText(QString::number(_ui->label_stats_loopClosuresRejected->text().toInt() + 1));
					_ui->label_matchId->setText(QString("Loop hypothesis (%1) rejected!").arg(highestHypothesisId));
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
				if(lcImg.isNull())
				{
					int id = stat.loopClosureId()>0?stat.loopClosureId():highestHypothesisId;
					QMap<int, QByteArray>::iterator iter = _imagesMap.find(id);
					if(iter != _imagesMap.end())
					{
						if(!lcImg.loadFromData(iter.value(), "JPEG"))
						{
							ULOGGER_ERROR("conversion from QByteArray to QImage failed");
							lcImg = QImage();
						}
					}
				}

				if(!lcImg.isNull())
				{
					_ui->imageView_loopClosure->scene()->addPixmap(QPixmap::fromImage(lcImg))->setVisible(this->_ui->imageView_loopClosure->isImageShown());
					UDEBUG("Adding mask = %d", stat.loopMotionMask().size());
					if(stat.loopMotionMask().size() == (unsigned int)lcImg.width()*lcImg.height())
					{
						UDEBUG("Adding mask");
						const std::vector<unsigned char> & maskData = stat.loopMotionMask();
						QImage mask(lcImg.size(), QImage::Format_ARGB32);
						int i=0;
						for(int y=0; y<mask.height(); ++y)
						{
							for(int x=0; x<mask.width(); ++x)
							{
								mask.setPixel(x,y,qRgba(255,0,255,!maskData[i++]*_preferencesDialog->getKeypointsOpacity()));
							}
						}
						_ui->imageView_loopClosure->scene()->addPixmap(QPixmap::fromImage(mask));
					}
				}
			}
		}

		if(_ui->imageView_loopClosure->items().size() || stat.loopClosureId()>0)
		{
			this->drawKeypoints(stat.refWords(), stat.loopWords());
		}
		else
		{
			this->drawKeypoints(stat.refWords(), std::multimap<int, cv::KeyPoint>()); //empty loop keypoints...
		}

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

		if(_preferencesDialog->isImageFlipped())
		{
			_ui->imageView_source->scale(-1.0, 1.0);
			_ui->imageView_loopClosure->scale(-1.0, 1.0);
		}

		_ui->statsToolBox->updateStat("Keypoint/Keypoints count in the last signature/", stat.refImageId(), stat.refWords().size());
		_ui->statsToolBox->updateStat("Keypoint/Keypoints count in the loop signature/", stat.refImageId(), stat.loopWords().size());
		ULOGGER_DEBUG("");

		// PDF AND LIKELIHOOD
		if(!stat.posterior().empty() && _ui->dockWidget_posterior->isVisible())
		{
			UDEBUG("");
			if(!refParentId)
			{
				_posteriorCurve->setData(QMap<int, float>(stat.posterior()), QMap<int, int>(stat.weights()), stat.refImageId());
			}
			else
			{
				// if the new signature has a parent, don't add it
				_posteriorCurve->setData(QMap<int, float>(stat.posterior()), QMap<int, int>(stat.weights()), 0);
			}

			ULOGGER_DEBUG("");
			//Adjust thresholds
			float value;
			value = float(_preferencesDialog->getLoopThr());
			emit(loopClosureThrChanged(value));
			value = float(_preferencesDialog->getRetrievalThr());
			emit(retrievalThrChanged(value));
		}
		if(!stat.likelihood().empty() && _ui->dockWidget_likelihood->isVisible())
		{
			UDEBUG("");
			if(!refParentId)
			{
				_likelihoodCurve->setData(QMap<int, float>(stat.likelihood()), QMap<int, int>(stat.weights()), stat.refImageId());
			}
			else
			{
				// if the new signature has a parent, don't add it
				_likelihoodCurve->setData(QMap<int, float>(stat.likelihood()), QMap<int, int>(stat.weights()), 0);
			}
			ULOGGER_DEBUG("");
		}

		// Audio frame
		if(_ui->dockWidget_audioProcessedFrames->isVisible() && !refAudioFrame.empty())
		{
			if(_mic)
			{
				_ui->widget_audioProcessedFrames->setSamplingRate(_mic->fs());
			}
			std::vector<float> v((float*)refAudioFrame.data, ((float*)refAudioFrame.data) + refAudioFrame.cols);
			_ui->widget_audioProcessedFrames->push(v);
		}
		if(_ui->dockWidget_audioLoopFrames->isVisible())
		{
			if(_mic)
			{
				_ui->widget_audioLoopFrames->setSamplingRate(_mic->fs());
			}
			std::vector<float> v;
			if(!loopAudioFrame.empty())
			{
				v = std::vector<float>((float*)loopAudioFrame.data, ((float*)loopAudioFrame.data) + loopAudioFrame.cols);
			}
			else if(!refAudioFrame.empty())
			{
				v = std::vector<float>(refAudioFrame.cols, 0);
			}

			if(v.size())
			{
				_ui->widget_audioLoopFrames->push(v);
			}
		}

		// ACTIONS
		if(stat.getActuators().size())
		{
			const std::list<Actuator> & actuators = stat.getActuators();
			for(std::list<Actuator>::const_iterator iter=actuators.begin(); iter!=actuators.end(); ++iter)
			{
				const Actuator & a = *iter;
				if(a.type() == Actuator::kTypeTwist)
				{
					if(a.data().total() == 6 && (a.data().type() & CV_32F))
					{
						emit twistReceived(a.data().at<float>(0),
								a.data().at<float>(1),
								a.data().at<float>(2),
								a.data().at<float>(3),
								a.data().at<float>(4),
								a.data().at<float>(5),
								1,
								0);
					}
					else
					{
						UERROR("Twist must be CV_32F and have 6 elements (%d)", a.data().total());
					}
					break; // Show only the first
				}
			}
		}
		else
		{
			UDEBUG("Actions are empty...");
		}

		// Update statistics tool box
		const std::map<std::string, float> & statistics = stat.data();
		for(std::map<std::string, float>::const_iterator iter = statistics.begin(); iter != statistics.end(); ++iter)
		{
			//ULOGGER_DEBUG("Updating stat \"%s\"", (*iter).first.c_str());
			_ui->statsToolBox->updateStat(QString((*iter).first.c_str()).replace('_', ' '), stat.refImageId(), (*iter).second);
		}
		UDEBUG("");
	}
	else if(!stat.extended() && stat.loopClosureId()>0)
	{
		_ui->label_stats_loopClosuresDetected->setText(QString::number(_ui->label_stats_loopClosuresDetected->text().toInt() + 1));
		_ui->label_matchId->setText(QString("Match ID = %1").arg(stat.loopClosureId()));
	}
	float elapsedTime = static_cast<float>(time.elapsed());
	UINFO("Processing statistics time = %fs", elapsedTime/1000.0f);
	_ui->statsToolBox->updateStat("/Gui refresh stats/ms", stat.refImageId(), elapsedTime);
	this->captureScreen();
	_processingStatistics = false;
}

void MainWindow::processRtabmapEventInit(int status, const QString & info)
{
	if((RtabmapEventInit::Status)status == RtabmapEventInit::kInitializing)
	{
		if(_state == kDetecting)
		{
			this->pauseDetection();
		}
		_initProgressDialog->clear();
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

void MainWindow::applyAllPrefSettings()
{
	ULOGGER_DEBUG("");

	//This will update the statistics toolbox
	if(_ui->statsToolBox->findChildren<StatItem*>().size() == 0)
	{
		const std::map<std::string, float> & statistics = Statistics::defaultData();
		for(std::map<std::string, float>::const_iterator iter = statistics.begin(); iter != statistics.end(); ++iter)
		{
			_ui->statsToolBox->updateStat(QString((*iter).first.c_str()).replace('_', ' '), 0, (*iter).second);
		}
	}

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
		this->updateSelectSourceAudioMenu(_preferencesDialog->getSourceAudioType());
		this->updateSelectSourceDatabase(_preferencesDialog->isSourceDatabaseUsed());
		QString src;
		if(_preferencesDialog->isSourceImageUsed() && _preferencesDialog->isSourceAudioUsed())
		{
			src = (_preferencesDialog->getSourceImageTypeStr() + "+") + _preferencesDialog->getSourceAudioTypeStr();
		}
		else if(_preferencesDialog->isSourceImageUsed())
		{
			src = _preferencesDialog->getSourceImageTypeStr();
		}
		else if(_preferencesDialog->isSourceAudioUsed())
		{
			src = _preferencesDialog->getSourceAudioTypeStr();
		}
		else if(_preferencesDialog->isSourceDatabaseUsed())
		{
			src = "Database";
		}
		_ui->label_stats_source->setText(src);

		if(_camera)
		{
			_camera->setImageRate(_preferencesDialog->getGeneralInputRate());
			_camera->setAutoRestart(_preferencesDialog->getGeneralAutoRestart());
		}
		if(_dbReader)
		{
			_dbReader->setFrameRate(_preferencesDialog->getGeneralInputRate());
		}
	}

	if(flags & PreferencesDialog::kPanelGeneral)
	{
		ULogger::setLevel((ULogger::Level)_preferencesDialog->getGeneralLoggerLevel());
		ULogger::setEventLevel((ULogger::Level)_preferencesDialog->getGeneralLoggerEventLevel());
		ULogger::setType((ULogger::Type)_preferencesDialog->getGeneralLoggerType(), ((_preferencesDialog->getWorkingDirectory()+"/")+LOG_FILE_NAME).toStdString(), true);
		ULogger::setPrintTime(_preferencesDialog->getGeneralLoggerPrintTime());
		setupMainLayout(_preferencesDialog->isVerticalLayoutUsed());
	}
}

void MainWindow::applyPrefSettings(const rtabmap::ParametersMap & parameters)
{
	ULOGGER_DEBUG("");
	if(parameters.size())
	{
		rtabmap::ParametersMap parametersModified = parameters;

		if(_state != kIdle)
		{
			if(parametersModified.erase(Parameters::kRtabmapWorkingDirectory()))
			{
				_ui->statsToolBox->setWorkingDirectory(_preferencesDialog->getWorkingDirectory());
				if(_state == kMonitoring)
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
		else if(parametersModified.size() == _preferencesDialog->getAllParameters().size())
		{
			// Send only if all parameters are sent
			this->post(new ParamEvent(parametersModified));
		}
	}

	//update ui
	_ui->doubleSpinBox_stats_timeLimit->setValue(_preferencesDialog->getTimeLimit());

	float value;
	value = float(_preferencesDialog->getLoopThr());
	emit(loopClosureThrChanged(value));
	value = float(_preferencesDialog->getRetrievalThr());
	emit(retrievalThrChanged(value));
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
		_lastIds = QSet<int>::fromList(QList<int>::fromStdList(uKeys(refWords)));
	}

	// Draw lines between corresponding features...
	int deltaX = _ui->imageView_source->sceneRect().width();
	int deltaY = 0;
	if(_preferencesDialog->isVerticalLayoutUsed())
	{
		deltaX = 0;
		deltaY = _ui->imageView_source->sceneRect().height();
		deltaY += _ui->label_matchId->height()/_ui->imageView_source->transform().m22();
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

void MainWindow::updateSelectSourceAudioMenu(int type)
{
	if(_preferencesDialog->isSourceAudioUsed())
	{
		switch(type)
		{
		case 0:
			_ui->actionMic->setChecked(true);
			break;
		case 1:
		default:
			_ui->actionAudioFile->setChecked(true);
			break;
		}
	}
	else
	{
		// they are exclusive actions, so check/uncheck one should disable all.
		_ui->actionMic->setChecked(true);
		_ui->actionMic->setChecked(false);
	}
}

void MainWindow::updateSelectSourceDatabase(bool used)
{
	_ui->actionDatabase->setChecked(used);
}

void MainWindow::changeImgRateSetting()
{
	emit imgRateChanged(_ui->doubleSpinBox_stats_imgRate->value());
}

void MainWindow::changeTimeLimitSetting()
{
	emit timeLimitChanged((float)_ui->doubleSpinBox_stats_timeLimit->value());
}

void MainWindow::captureScreen()
{
	if(!_ui->actionAuto_screen_capture->isChecked())
	{
		return;
	}
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
	QString name = (QDateTime::currentDateTime().toString("yyMMddhhmmsszzz") + ".") + _autoScreenCaptureFormat;
	_ui->statusbar->clearMessage();
	QPixmap figure = QPixmap::grabWidget(this);
	figure.save(targetDir + name);
	_ui->statusbar->showMessage(tr("Screen captured \"%1\"").arg(targetDir + name), _preferencesDialog->getTimeLimit()*500);
}

void MainWindow::beep()
{
	QApplication::beep();
}

//ACTIONS
void MainWindow::startDetection()
{
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
	if(_mic != 0)
	{
		QMessageBox::warning(this,
							 tr("RTAB-Map"),
							 tr("A mic is running, stop it first."));
		UWARN("_mic is not null... it must be stopped first");
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

	// Adjust pre-requirements
	if( !_preferencesDialog->isSourceImageUsed() &&
		!_preferencesDialog->isSourceAudioUsed() &&
		!_preferencesDialog->isSourceDatabaseUsed())
	{
		QMessageBox::warning(this,
				 tr("RTAB-Map"),
				 tr("No sources are selected. See Preferences->Source panel."));
		UWARN("No sources are selected. See Preferences->Source panel.");
		emit stateChanged(kIdle);
		return;
	}
	if(_preferencesDialog->getMemoryType() == 0)
	{
		//KeypointMemory
		if(!_preferencesDialog->isSourceImageUsed() && !_preferencesDialog->isSourceDatabaseUsed())
		{
			QMessageBox::warning(this,
					 tr("RTAB-Map"),
					 tr("Image source is not selected. See Preferences->Source panel. "
					    "Keypoint memory only works with images."));
			UWARN("Image source is not selected. See Preferences->Source panel. "
				  "Keypoint memory only works with images.");
			emit stateChanged(kIdle);
			return;
		}
		else if(_preferencesDialog->isSourceAudioUsed())
		{
			QMessageBox::information(this,
					 tr("RTAB-Map"),
					 tr("Audio source is selected. Keypoint memory only works with images... "
					    "the audio source is now disabled."));
			UDEBUG("Audio source is selected. Keypoint memory only works with images... "
				   "the audio source is now disabled.");
			_preferencesDialog->disableSourceAudio();
		}
	}
	else
	{
		//SM memory
		if(_preferencesDialog->isSourceImageUsed() && _preferencesDialog->getGeneralCameraKeypoints())
		{
			QMessageBox::information(this,
				 tr("RTAB-Map"),
				 tr("Keypoint features extraction is selected for the camera. "
				    "Sensorimotor memory doesn't handle features (only raw images), "
				    "so it is overkill to extract them... keypoints extraction is now disabled."));
			UDEBUG("Keypoint features extraction is selected for the camera. "
				   "Sensorimotor memory doesn't handle features (only raw images), "
				   "so it is overkill to extract them... keypoints extraction is now disabled.");
			_preferencesDialog->disableGeneralCameraKeypoints();
		}
	}

	if(_preferencesDialog->isSourceDatabaseUsed())
	{
		_dbReader = new DBReader(_preferencesDialog->getSourceDatabasePath().toStdString(),
								 _preferencesDialog->getGeneralInputRate());

		if(!_dbReader->init())
		{
			ULOGGER_WARN("init DBReader failed... ");
			QMessageBox::warning(this,
								   tr("RTAB-Map"),
								   tr("Database reader initialization failed..."));
			emit stateChanged(kIdle);
			delete _dbReader;
			_dbReader = 0;
			return;
		}
	}
	else
	{
		if(_preferencesDialog->isSourceImageUsed())
		{
			// Change type of the camera...
			//
			int sourceType = _preferencesDialog->getSourceImageType();
			if(sourceType == 1) //Images
			{
				_camera = new CameraImages(
						_preferencesDialog->getSourceImagesPath().append(QDir::separator()).toStdString(),
						_preferencesDialog->getSourceImagesStartPos(),
						_preferencesDialog->getSourceImagesRefreshDir(),
						_preferencesDialog->getGeneralInputRate(),
						_preferencesDialog->getGeneralAutoRestart(),
						_preferencesDialog->getSourceWidth(),
						_preferencesDialog->getSourceHeight(),
						_preferencesDialog->getFramesDropped()
						);
			}
			else if(sourceType == 2)
			{
				_camera = new CameraVideo(
						_preferencesDialog->getSourceVideoPath().toStdString(),
						_preferencesDialog->getGeneralInputRate(),
						_preferencesDialog->getGeneralAutoRestart(),
						_preferencesDialog->getSourceWidth(),
						_preferencesDialog->getSourceHeight(),
						_preferencesDialog->getFramesDropped());
			}
			else if(sourceType == 0)
			{
				_camera = new CameraVideo(
						_preferencesDialog->getSourceUsbDeviceId(),
						_preferencesDialog->getGeneralInputRate(),
						_preferencesDialog->getGeneralAutoRestart(),
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
				_camera->setFeaturesExtracted(true);
				_camera->parseParameters(_preferencesDialog->getAllParameters());
			}

			if(!_camera->init())
			{
				ULOGGER_WARN("init camera failed... ");
				QMessageBox::warning(this,
									   tr("RTAB-Map"),
									   tr("Camera initialization failed..."));
				emit stateChanged(kIdle);
				delete _camera;
				_camera = 0;
				return;
			}

			UEventsManager::addHandler(_camera); //thread
		}
		if(_preferencesDialog->isSourceAudioUsed())
		{
			UDEBUG("");
			int sourceAudioType = _preferencesDialog->getSourceAudioType();
			if(sourceAudioType == 1) //Audio file
			{
				_mic = new Micro(MicroEvent::kTypeFrameFreqSqrdMagn,
						_preferencesDialog->getSourceAudioPath().toStdString(),
						true,
						44100.0/_preferencesDialog->getGeneralInputRate(),
						0,
						_preferencesDialog->getSourceAudioPlayWhileRecording());
			}
			else // Mic
			{
				_mic = new Micro(MicroEvent::kTypeFrameFreqSqrdMagn,
						_preferencesDialog->getSourceMicDevice(),
						_preferencesDialog->getSourceMicFs(),
						float(_preferencesDialog->getSourceMicFs())/_preferencesDialog->getGeneralInputRate(),
						1,
						_preferencesDialog->getSourceMicSampleSize());
			}

			if(!_mic->init())
			{
				QMessageBox::warning(this,
									   tr("RTAB-Map"),
									   tr("Mic initialization failed..."));
				ULOGGER_WARN("init mic failed... ");
				emit stateChanged(kIdle);
				delete _mic;
				_mic = 0;
				if(_camera)
				{
					delete _camera;
					_camera = 0;
				}
				return;
			}
		}

		if(_camera && _mic)
		{
			_cameraMic = new CameraMicro(_camera, _mic);
		}
	}

	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdCleanSensorsBuffer));
	this->applyAllPrefSettings();

	if(!_preferencesDialog->isStatisticsPublished())
	{
		QMessageBox::information(this,
				tr("Information"),
				tr("Note that publishing statistics is disabled, "
				   "progress will not be shown in the GUI."));
	}

	emit stateChanged(kDetecting);
}

void MainWindow::pauseDetection()
{
	if(_camera || _mic || _dbReader)
	{
		if(_state == kPaused && (QApplication::keyboardModifiers() & Qt::ShiftModifier))
		{
			// On Ctrl-click, start the camera and pause it automatically
			if((_camera &&_camera->isRunning()) ||
				(_mic &&_mic->isRunning()) ||
				(_cameraMic && _cameraMic->isRunning()) ||
				(_dbReader && _dbReader->isRunning()))
			{
				//Don't wait for GUI thread, kill now
				if(_cameraMic)
				{
					_cameraMic->kill();
				}
				else
				{
					if(_camera)
					{
						_camera->kill();
					}
					if(_mic)
					{
						_mic->kill();
					}
				}
				if(_dbReader)
				{
					_dbReader->kill();
				}
			}
			emit stateChanged(kPaused);
			if(_preferencesDialog->getGeneralInputRate())
			{
				QTimer::singleShot(1000/_preferencesDialog->getGeneralInputRate() + 10, this, SLOT(pauseDetection()));
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
}

void MainWindow::stopDetection()
{
	if(_state == kIdle || (!_camera && !_mic && !_dbReader))
	{
		return;
	}

	if(_state == kDetecting &&
			( (!_mic && _camera && _camera->isRunning()) ||
			  (!_camera && _mic && _mic->isRunning()) ||
			  (_cameraMic && _cameraMic->isRunning()) ||
			  (_dbReader && _dbReader->isRunning())) )
	{
		QMessageBox::StandardButton button = QMessageBox::question(this, tr("Stopping process..."), tr("Are you sure you want to stop the process?"), QMessageBox::Yes|QMessageBox::No, QMessageBox::No);

		if(button != QMessageBox::Yes)
		{
			return;
		}
	}

	ULOGGER_DEBUG("");
	// kill the processes
	if(_cameraMic)
	{
		_cameraMic->join(true);
	}
	else
	{
		if(_camera)
		{
			_camera->join(true);
		}
		if(_mic)
		{
			_mic->join(true);
		}
	}
	if(_dbReader)
	{
		_dbReader->join(true);
	}
	// delete the processes
	if(_cameraMic)
	{
		delete _cameraMic;
		_cameraMic = 0;
	}
	if(_camera)
	{
		delete _camera;
		_camera = 0;
	}
	if(_mic)
	{
		delete _mic;
		_mic = 0;
	}
	if(_dbReader)
	{
		delete _dbReader;
		_dbReader = 0;
	}
	emit stateChanged(kIdle);

	//Copy showlogs.m from appDirPath/../share/rtabmap/ShowLogs.m to working directory. (appDirPath is in bin)
	QString showLogFileSrc = (QApplication::applicationDirPath()+"/../")+SHARE_SHOW_LOG_FILE;
	QString showLogFileTarget = (_preferencesDialog->getWorkingDirectory()+"/")+UFile::getName(SHARE_SHOW_LOG_FILE).c_str();
	if(!QFile::exists(showLogFileTarget) && QFile::exists(showLogFileSrc))
	{
		QFile::copy(showLogFileSrc, showLogFileTarget);
	}

	// copy importfile.m
	QString importFileSrc = (QApplication::applicationDirPath()+"/../")+SHARE_IMPORT_FILE;
	QString importFileTarget = (_preferencesDialog->getWorkingDirectory()+"/")+UFile::getName(SHARE_IMPORT_FILE).c_str();
	if(!QFile::exists(importFileTarget) && QFile::exists(importFileSrc))
	{
		QFile::copy(importFileSrc, importFileTarget);
	}

	// copy getPrecisionRecall.m
	QString getPrFileSrc = (QApplication::applicationDirPath()+"/../")+SHARE_GET_PRECISION_RECALL_FILE;
	QString getPrFileTarget = (_preferencesDialog->getWorkingDirectory()+"/")+UFile::getName(SHARE_GET_PRECISION_RECALL_FILE).c_str();
	if(!QFile::exists(getPrFileTarget) && QFile::exists(getPrFileSrc))
	{
		QFile::copy(getPrFileSrc, getPrFileTarget);
	}
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
		RtabmapEventCmd * event = new RtabmapEventCmd(RtabmapEventCmd::kCmdGenerateGraph);
		event->setStr(path.toStdString());
		this->post(event); // The event is automatically deleted by the EventsManager...
	}
}

void MainWindow::deleteMemory()
{
	QMessageBox::StandardButton button;
	QString dbPath = _preferencesDialog->getWorkingDirectory() + QDir::separator() + "LTM.db";
	if(_state == kMonitoring)
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

	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDeleteMemory));
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
	if(_state != kMonitoring)
	{
		QString dbPath = _preferencesDialog->getWorkingDirectory() + "/LTM.db";
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

void MainWindow::selectMic()
{
	_preferencesDialog->selectSourceAudio(PreferencesDialog::kSrcAudioMicDevice);
}

void MainWindow::selectAudioFile()
{
	_preferencesDialog->selectSourceAudio(PreferencesDialog::kSrcAudioFile);
}

void MainWindow::selectDatabase()
{
	_preferencesDialog->selectSourceDatabase(true);
}

void MainWindow::resetTheMemory()
{
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdResetMemory));
	_lastId = 0;
	_lastIds.clear();
}

void MainWindow::dumpTheMemory()
{
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDumpMemory));
}

void MainWindow::dumpThePrediction()
{
	this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDumpPrediction));
}

void MainWindow::clearTheCache()
{
	_imagesMap.clear();
	_likelihoodCurve->clear();
	_posteriorCurve->clear();
	_lastId = 0;
	_lastIds.clear();
	_ui->label_stats_loopClosuresDetected->setText("0");
	_ui->label_stats_loopClosuresReactivatedDetected->setText("0");
	_ui->label_stats_loopClosuresRejected->setText("0");
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

void MainWindow::selectScreenCaptureFormat(bool checked)
{
	if(checked)
	{
		QStringList items;
		items << QString("png") << QString("jpg");
		bool ok;
		QString item = QInputDialog::getItem(this, tr("Select format"), tr("Format:"), items, 0, false, &ok);
		if(ok && !item.isEmpty())
		{
			_autoScreenCaptureFormat = item;
		}
		this->captureScreen();
	}
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

//END ACTIONS

// STATES
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
		_ui->actionPause_when_a_loop_hypothesis_is_rejected->setEnabled(true);
		_ui->actionReset_the_memory->setEnabled(true);
		_ui->actionDump_the_memory->setEnabled(true);
		_ui->actionDelete_memory->setEnabled(true);
		_ui->actionGenerate_map->setEnabled(true);
		_ui->actionOpen_working_directory->setEnabled(true);
		_ui->actionApply_settings_to_the_detector->setEnabled(true);
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
		_ui->actionPause_when_a_loop_hypothesis_is_rejected->setEnabled(true);
		_ui->actionReset_the_memory->setEnabled(false);
		_ui->actionDump_the_memory->setEnabled(false);
		_ui->actionDelete_memory->setEnabled(false);
		_ui->actionGenerate_map->setEnabled(false);
		_ui->actionOpen_working_directory->setEnabled(true);
		_ui->actionApply_settings_to_the_detector->setEnabled(false);
		_ui->menuSelect_source->setEnabled(false);
		_ui->doubleSpinBox_stats_imgRate->setEnabled(true);
		_ui->statusbar->showMessage(tr("Detecting..."));
		_state = newState;
		_ui->label_elapsedTime->setText("00:00:00");
		_elapsedTime->start();
		_oneSecondTimer->start();
		if(_cameraMic)
		{
			_cameraMic->start();
		}
		else
		{
			if(_camera)
			{
				_camera->start();
			}
			if(_mic)
			{
				_mic->start();
			}
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
			_ui->actionReset_the_memory->setEnabled(false);
			_ui->actionDump_the_memory->setEnabled(false);
			_ui->actionDelete_memory->setEnabled(false);
			_ui->actionGenerate_map->setEnabled(false);
			_state = kDetecting;
			_elapsedTime->start();
			_oneSecondTimer->start();
			if(_cameraMic)
			{
				_cameraMic->start();
			}
			else
			{
				if(_camera)
				{
					_camera->start();
				}
				if(_mic)
				{
					_mic->start();
				}
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
			_ui->actionReset_the_memory->setEnabled(true);
			_ui->actionDump_the_memory->setEnabled(true);
			_ui->actionDelete_memory->setEnabled(true);
			_ui->actionGenerate_map->setEnabled(true);
			_state = kPaused;
			_oneSecondTimer->stop();
			// kill sensors
			if(_cameraMic)
			{
				_cameraMic->join(true);
			}
			else
			{
				if(_camera)
				{
					_camera->join(true);
				}
				if(_mic)
				{
					_mic->join(true);
				}
			}
			if(_dbReader)
			{
				_dbReader->join(true);
			}
		}
		break;
	case kMonitoring:
		_ui->actionStart->setEnabled(false);
		_ui->actionPause->setEnabled(false);
		_ui->actionPause->setChecked(false);
		_ui->actionPause->setToolTip(tr("Pause"));
		_ui->actionStop->setEnabled(false);
		_ui->actionPause_on_match->setEnabled(false);
		_ui->actionPause_when_a_loop_hypothesis_is_rejected->setEnabled(false);
		_ui->actionReset_the_memory->setEnabled(true);
		_ui->actionDump_the_memory->setEnabled(true);
		_ui->actionDelete_memory->setEnabled(true);
		_ui->actionGenerate_map->setEnabled(false);
		_ui->actionOpen_working_directory->setEnabled(false);
		_ui->actionApply_settings_to_the_detector->setEnabled(false);
		_ui->menuSelect_source->setEnabled(false);
		_ui->doubleSpinBox_stats_imgRate->setEnabled(false);
		_ui->statusbar->showMessage(tr("Monitoring..."));
		_state = newState;
		_ui->label_elapsedTime->setText("00:00:00");
		_elapsedTime->start();
		_oneSecondTimer->start();
		break;
	default:
		break;
	}

}

}
