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
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/Camera.h"
#include "qtipl.h"
#include "KeypointItem.h"
#include "AboutDialog.h"
#include "utilite/UtiLite.h"
#include "rtabmap/core/Parameters.h"
#include "ImageView.h"
#include "Plot.h"
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

#define LOG_FILE_NAME "LogRtabmap.txt"
#define SHARE_SHOW_LOG_FILE "share/rtabmap/ShowLogs.m"
#define SHARE_IMPORT_FILE   "share/rtabmap/importfile.m"

using namespace rtabmap;

inline static void initGuiResource() { Q_INIT_RESOURCE(GuiLib); }


namespace rtabmap {

MainWindow::MainWindow(PreferencesDialog * prefDialog, QWidget * parent) :
	QMainWindow(parent),
	_ui(0),
	_state(kIdle),
	_camera(0),
	_srcType(kSrcUndefined),
	_preferencesDialog(0),
	_aboutDialog(0),
	_lastId(0),
	_oneSecondTimer(0),
	_elapsedTime(0),
	_posteriorCurve(0),
	_likelihoodCurve(0)
{
	ULOGGER_DEBUG("");

	initGuiResource();

	QPixmap pixmap(":images/RTAB-Map.png");
	QSplashScreen splash(pixmap);
	splash.show();
	splash.showMessage(tr("Loading..."));
	QApplication::processEvents();

	this->setWindowTitle(tr("Constant-Time Appearance-Based Mapping"));
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
	ThresholdCurve * tc;
	tc = _ui->posteriorPlot->addThreshold("Loop closure thr", float(_preferencesDialog->getLoopThr()));
	connect(this, SIGNAL(loopClosureThrChanged(float)), tc, SLOT(setThreshold(float)));
	tc = _ui->posteriorPlot->addThreshold("Retrieval thr", float(_preferencesDialog->getReacThr()));
	connect(this, SIGNAL(loopClosureThrChanged(float)), tc, SLOT(setThreshold(float)));

	_likelihoodCurve = new PdfPlotCurve("Likelihood", &_imagesMap, this);
	_ui->likelihoodPlot->addCurve(_likelihoodCurve);
	_ui->likelihoodPlot->showLegend(false);

	_ui->doubleSpinBox_stats_imgRate->setValue(_preferencesDialog->getGeneralImageRate());
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

#if defined(Q_WS_MAC) or defined(Q_WS_WIN)
	connect(_ui->actionOpen_working_directory, SIGNAL(triggered()), SLOT(openWorkingDirectory()));
#else
	_ui->menuEdit->removeAction(_ui->actionOpen_working_directory);
#endif

	//Settings menu
	_selectSourceGrp = new QActionGroup(this);
	_selectSourceGrp->addAction(_ui->actionStream);
	_selectSourceGrp->addAction(_ui->actionImages);
	_selectSourceGrp->addAction(_ui->actionVideo);
	_selectSourceGrp->addAction(_ui->actionDatabase);
	this->updateSelectSourceMenu(_preferencesDialog->getSourceType());
	connect(_ui->actionImages, SIGNAL(triggered()), this, SLOT(selectImages()));
	connect(_ui->actionVideo, SIGNAL(triggered()), this, SLOT(selectVideo()));
	connect(_ui->actionStream, SIGNAL(triggered()), this, SLOT(selectStream()));
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
	connect(this, SIGNAL(imgRateChanged(double)), _preferencesDialog, SLOT(setImgRate(double)));
	connect(this, SIGNAL(timeLimitChanged(double)), _preferencesDialog, SLOT(setTimeLimit(double)));
	connect(_ui->checkBox_showFeatures, SIGNAL(toggled(bool)), this, SLOT(updateItemsShown()));
	connect(_ui->checkBox_showFeaturesLoop, SIGNAL(toggled(bool)), this, SLOT(updateItemsShown()));
	connect(_ui->checkBox_showImage, SIGNAL(toggled(bool)), this, SLOT(updateItemsShown()));
	connect(_ui->checkBox_showImageLoop, SIGNAL(toggled(bool)), this, SLOT(updateItemsShown()));

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
		const Statistics & stats = rtabmapEvent->getStats();
		int highestHypothesisId = int(uValue(stats.data(), Statistics::kLoopHighest_hypothesis_id(), 0.0f));
		bool rejectedHyp = bool(uValue(stats.data(), Statistics::kLoopRejectedHypothesis(), 0.0f));
		if((stats.loopClosureId() > 0 && _ui->actionPause_on_match->isChecked()) ||
		   (stats.loopClosureId() == 0 && highestHypothesisId > 0 && _ui->actionPause_when_a_loop_hypothesis_is_rejected->isChecked() && rejectedHyp))
		{
			if(_state != kPaused)
			{
				if(_preferencesDialog->beepOnPause())
				{
					QApplication::beep();
				}
				this->pauseDetection();
			}
		}
		Statistics statsCp = stats;
		emit statsReceived(statsCp);
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
				QApplication::beep();
			}
			//_ui->statusbar->showMessage(tr("The source has no more images..."));
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
					QApplication::beep();
				}
				pauseDetection();
			}
		}
	}
}

void MainWindow::processStats(const rtabmap::Statistics & stat)
{
	ULOGGER_DEBUG("");
	QTime time;
	time.start();
	//Affichage des stats et images
	if(stat.extended())
	{
		ULOGGER_DEBUG("");
		_ui->label_refId->setText(QString("New ID = %1").arg(stat.refImageId()));
		int highestHypothesisId = static_cast<float>(uValue(stat.data(), Statistics::kLoopHighest_hypothesis_id(), 0.0f));
		bool refParentId = (int)uValue(stat.data(), Statistics::kParent_id(), 0.0f);
		bool highestHypothesisIsSaved = (bool)uValue(stat.data(), Statistics::kHypothesis_reactivated(), 0.0f);
		//if(!(_state == kPaused && !_ui->label_matchId->text().isEmpty() && !(stat.loopClosureId()>0 || highestHypothesisId>0)))
		{
			ULOGGER_DEBUG("");
			// Loop closure info
			_ui->imageView_source->scene()->clear();
			_ui->imageView_loopClosure->scene()->clear();
			_ui->imageView_source->resetTransform();
			_ui->imageView_loopClosure->resetTransform();

			_ui->label_matchId->clear();
			QPixmap refPixmap;
			_ui->label_stats_imageNumber->setText(QString::number(stat.refImageId()));
			if(stat.refImage())
			{
				QImage img = Ipl2QImage(stat.refImage());
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
				_ui->imageView_source->scene()->addPixmap(refPixmap)->setVisible(this->_ui->checkBox_showImage->isChecked());
				_ui->imageView_source->setSceneRect(sceneRect);
				_ui->imageView_loopClosure->setSceneRect(sceneRect);
			}
			ULOGGER_DEBUG("");
			if(stat.loopClosureImage())
			{
				QImage img = Ipl2QImage(stat.loopClosureImage());
				//Only kept if it not added as a child in the core, that
				//means it will never used for a loop closure detection
				if(_preferencesDialog->isImagesKept())
				{
					if(stat.loopClosureId()>0 || highestHypothesisId>0)
					{
						int id = stat.loopClosureId()>0?stat.loopClosureId():highestHypothesisId;
						QByteArray ba;
						QBuffer buffer(&ba);
						buffer.open(QIODevice::WriteOnly);
						img.save(&buffer, "JPEG"); // writes image into JPEG format
						_imagesMap.insert(id, ba);
					}
				}
			}
			ULOGGER_DEBUG("");
			_ui->imageView_loopClosure->setBackgroundBrush(QBrush(Qt::black));
			int rejectedHyp = bool(uValue(stat.data(), Statistics::kLoopRejectedHypothesis(), 0.0f));
			if(stat.loopClosureId()>0)
			{
				_ui->label_stats_loopClosuresDetected->setText(QString::number(_ui->label_stats_loopClosuresDetected->text().toInt() + 1));
				if(highestHypothesisIsSaved)
				{
					_ui->label_stats_loopClosuresReactivatedDetected->setText(QString::number(_ui->label_stats_loopClosuresReactivatedDetected->text().toInt() + 1));
				}
				_ui->label_matchId->setText(QString("Match ID = %1").arg(stat.loopClosureId()));

				QImage img;
				QMap<int, QByteArray>::iterator iter = _imagesMap.find(stat.loopClosureId());
				if(iter != _imagesMap.end())
				{
					if(!img.loadFromData(iter.value(), "JPEG"))
					{
						ULOGGER_ERROR("conversion from QByteArray to QImage failed");
						img = QImage();
					}
				}
				else if(stat.loopClosureImage())
				{
					img = Ipl2QImage(stat.loopClosureImage());
				}

				if(!img.isNull())
				{
					_ui->imageView_loopClosure->scene()->addPixmap(QPixmap::fromImage(img))->setVisible(this->_ui->checkBox_showImageLoop->isChecked());
				}
			}
			else if(rejectedHyp)
			{
				ULOGGER_DEBUG("");
				_ui->label_stats_loopClosuresRejected->setText(QString::number(_ui->label_stats_loopClosuresRejected->text().toInt() + 1));
				_ui->label_matchId->setText(QString("Loop hypothesis (%1) rejected!").arg(highestHypothesisId));

				if(_preferencesDialog->imageRejectedShown())
				{
					QColor color = Qt::yellow;
					QGraphicsTextItem * textItem = _ui->imageView_loopClosure->scene()->addText(tr("Rejected hypothesis"));

					textItem->setDefaultTextColor(QColor(255-color.red(), 255-color.green(), 255-color.blue())); // color inverted
					textItem->setZValue(2);

					_ui->imageView_loopClosure->setBackgroundBrush(QBrush(color));
					QImage img;
					QMap<int, QByteArray>::iterator iter = _imagesMap.find(highestHypothesisId);
					if(iter != _imagesMap.end())
					{
						if(!img.loadFromData(iter.value(), "JPEG"))
						{
							ULOGGER_ERROR("conversion from QByteArray to QImage failed");
							img = QImage();
						}
					}
					else if(stat.loopClosureImage())
					{
						img = Ipl2QImage(stat.loopClosureImage());
					}

					if(!img.isNull())
					{
						_ui->imageView_loopClosure->scene()->addPixmap(QPixmap::fromImage(img));
					}
				}
				if(highestHypothesisIsSaved)
				{
					_ui->label_matchId->setText(QString("[Retrieved!] ").append(_ui->label_matchId->text()));
				}
			}

			ULOGGER_DEBUG("");
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
			if(!stat.refImage())
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
			if(!stat.posterior().empty() && _ui->dockWidget_posterior->isVisible())
			{
				ULOGGER_DEBUG("");
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
				value = float(_preferencesDialog->getReacThr());
				emit(retrievalThrChanged(value));
			}
			if(!stat.likelihood().empty() && _ui->dockWidget_likelihood->isVisible())
			{
				ULOGGER_DEBUG("");
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

			ULOGGER_DEBUG("");
			// Update statistics tool box
			const std::map<std::string, float> & statistics = stat.data();
			for(std::map<std::string, float>::const_iterator iter = statistics.begin(); iter != statistics.end(); ++iter)
			{
				//ULOGGER_DEBUG("Updating stat \"%s\"", (*iter).first.c_str());
				_ui->statsToolBox->updateStat(QString((*iter).first.c_str()).replace('_', ' '), stat.refImageId(), (*iter).second);
			}
		}

	}
	else if(!stat.extended() && stat.loopClosureId()>0)
	{
		_ui->label_stats_loopClosuresDetected->setText(QString::number(_ui->label_stats_loopClosuresDetected->text().toInt() + 1));
		_ui->label_matchId->setText(QString("Match ID = %1").arg(stat.loopClosureId()));
	}
	float elapsedTime = static_cast<float>(time.elapsed());
	ULOGGER_INFO("Processing statistics time = %fs", elapsedTime/1000.0f);
	_ui->statsToolBox->updateStat("/Gui refresh stats/ms", stat.refImageId(), elapsedTime);

	this->captureScreen();
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
		_ui->doubleSpinBox_stats_imgRate->setValue(_preferencesDialog->getGeneralImageRate());
		this->updateSelectSourceMenu(_preferencesDialog->getSourceType());
		_ui->label_stats_source->setText(_selectSourceGrp->checkedAction()->text().replace('.', ""));

		this->post(new CameraEvent(CameraEvent::kCmdChangeParam, _preferencesDialog->getGeneralImageRate(), _preferencesDialog->getGeneralAutoRestart()));
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
		if(_state != kIdle)
		{
			rtabmap::ParametersMap parametersModified = parameters;
			if(parametersModified.erase(Parameters::kRtabmapWorkingDirectory()))
			{
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
		else if(parameters.size() == _preferencesDialog->getAllParameters().size())
		{
			// Send only if all parameters are sent
			this->post(new ParamEvent(parameters));
		}
	}

	//update ui
	_ui->doubleSpinBox_stats_timeLimit->setValue(_preferencesDialog->getTimeLimit());

	float value;
	value = float(_preferencesDialog->getLoopThr());
	emit(loopClosureThrChanged(value));
	value = float(_preferencesDialog->getReacThr());
	emit(retrievalThrChanged(value));
}

void MainWindow::drawKeypoints(const std::multimap<int, cv::KeyPoint> & refWords, const std::multimap<int, cv::KeyPoint> & loopWords)
{
	UTimer timer;

	timer.start();
	KeypointItem * item = 0;
	int alpha = _preferencesDialog->getKeypointsOpacity()*255/100;
	ULOGGER_DEBUG("refWords.size() = %d", refWords.size());
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
		item->setVisible(this->_ui->checkBox_showFeatures->isChecked());
		this->_ui->imageView_source->scene()->addItem(item);
		item->setZValue(1);
	}
	ULOGGER_DEBUG("source time = %f s", timer.ticks());

	timer.start();
	item = 0;
	ULOGGER_DEBUG("loopWords.size() = %d", loopWords.size());
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
		item->setVisible(this->_ui->checkBox_showFeaturesLoop->isChecked());
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
}

void MainWindow::updateItemsShown()
{
	QList<QGraphicsItem*> items = _ui->imageView_source->scene()->items();
	for(int i=0; i<items.size(); ++i)
	{
		if(qgraphicsitem_cast<KeypointItem*>(items.at(i)))
		{
			items.at(i)->setVisible(_ui->checkBox_showFeatures->isChecked());
		}
		else if(qgraphicsitem_cast<QGraphicsPixmapItem*>(items.at(i)))
		{
			items.at(i)->setVisible(_ui->checkBox_showImage->isChecked());
		}
	}

	items = _ui->imageView_loopClosure->scene()->items();
	for(int i=0; i<items.size(); ++i)
	{
		if(qgraphicsitem_cast<KeypointItem*>(items.at(i)))
		{
			items.at(i)->setVisible(_ui->checkBox_showFeaturesLoop->isChecked());
		}
		else if(qgraphicsitem_cast<QGraphicsPixmapItem*>(items.at(i)))
		{
			items.at(i)->setVisible(_ui->checkBox_showImageLoop->isChecked());
		}
	}
}

void MainWindow::resizeEvent(QResizeEvent* anEvent)
{
	_ui->imageView_source->fitInView(_ui->imageView_source->sceneRect(), Qt::KeepAspectRatio);
	_ui->imageView_loopClosure->fitInView(_ui->imageView_source->sceneRect(), Qt::KeepAspectRatio);
	_ui->imageView_source->resetZoom();
	_ui->imageView_loopClosure->resetZoom();
}

void MainWindow::updateSelectSourceMenu(int type)
{
	switch(type)
	{
	case 0:
		_ui->actionStream->setChecked(true);
		break;
	case 1:
		_ui->actionImages->setChecked(true);
		break;
	case 2:
		_ui->actionVideo->setChecked(true);
		break;
	case 3:
	default:
		_ui->actionDatabase->setChecked(true);
		break;
	}
}

void MainWindow::changeImgRateSetting()
{
	emit imgRateChanged(_ui->doubleSpinBox_stats_imgRate->value());
}

void MainWindow::changeTimeLimitSetting()
{
	emit timeLimitChanged(_ui->doubleSpinBox_stats_timeLimit->value());
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
	QString name = QDateTime::currentDateTime().toString("yyMMdd-hhmmsszzz") + ".png";
	_ui->statusbar->clearMessage();
	QPixmap figure = QPixmap::grabWidget(this);
	figure.save(targetDir + name);
	_ui->statusbar->showMessage(tr("Screen captured \"%1\"").arg(targetDir + name), _preferencesDialog->getTimeLimit()*500);
}


//ACTIONS
void MainWindow::startDetection()
{
	emit stateChanged(kStartingDetection);

	if(_camera != 0)
	{
		QMessageBox::warning(this,
						     tr("RTAB-Map"),
						     tr("A camera is running, stop it first."));
		ULogger::write("_camera is not null... stop it first");
		emit stateChanged(kIdle);
		return;
	}

	// Change type of the camera...
	//
	int sourceType = _preferencesDialog->getSourceType();
	if(sourceType == 1) //Images
	{
		_camera = new CameraImages(
				_preferencesDialog->getSourceImagesPath().append(QDir::separator()).toStdString(),
				_preferencesDialog->getSourceImagesStartPos(),
				_preferencesDialog->getSourceImagesRefreshDir(),
				_preferencesDialog->getGeneralImageRate(),
				_preferencesDialog->getGeneralAutoRestart(),
				_preferencesDialog->getSourceWidth(),
				_preferencesDialog->getSourceHeight()
				);
	}
	else if(sourceType == 2)
	{
		_camera = new CameraVideo(
				_preferencesDialog->getSourceVideoPath().toStdString(),
				_preferencesDialog->getGeneralImageRate(),
				_preferencesDialog->getGeneralAutoRestart(),
				_preferencesDialog->getSourceWidth(),
				_preferencesDialog->getSourceHeight());
	}
	else if(sourceType == 3)
	{
		_camera = new CameraDatabase(
				_preferencesDialog->getSourceDatabasePath().toStdString(),
				_preferencesDialog->getSourceDatabaseIgnoreChildren(),
				_preferencesDialog->getGeneralImageRate(),
				_preferencesDialog->getGeneralAutoRestart(),
				_preferencesDialog->getSourceWidth(),
				_preferencesDialog->getSourceHeight());
	}
	else if(sourceType == 0)
	{
		_camera = new CameraVideo(
				_preferencesDialog->getSourceUsbDeviceId(),
				_preferencesDialog->getGeneralImageRate(),
				_preferencesDialog->getGeneralAutoRestart(),
				_preferencesDialog->getSourceWidth(),
				_preferencesDialog->getSourceHeight());
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

	_camera->setPostThreatement(new CamKeypointTreatment(_preferencesDialog->getAllParameters()));

	if(!_camera->init())
	{
		QMessageBox::warning(this,
							   tr("RTAB-Map"),
							   tr("Camera initialization failed..."));
		ULOGGER_WARN("init camera failed... ");
		emit stateChanged(kIdle);
		delete _camera;
		_camera = 0;
		return;
	}

	UEventsManager::addHandler(_camera); //thread

	this->applyAllPrefSettings();

	_camera->start();

	emit stateChanged(kDetecting);
}

void MainWindow::pauseDetection()
{
	ULOGGER_DEBUG("");
	this->post(new CameraEvent(CameraEvent::kCmdPause));
	emit stateChanged(kPaused);
}

void MainWindow::stopDetection()
{
	if(_state == kIdle)
	{
		return;
	}
	if(_state == kDetecting && !_camera->isPaused())
	{
		QMessageBox::StandardButton button = QMessageBox::question(this, tr("Stopping process..."), tr("Are you sure you want to stop the process?"), QMessageBox::Yes|QMessageBox::No, QMessageBox::No);

		if(button != QMessageBox::Yes)
		{
			return;
		}
	}

	ULOGGER_DEBUG("");
	if(_camera)
	{
		UEventsManager::removeHandler(_camera);
		//_camera->killSafely();

		delete _camera;
		_camera = 0;
		emit stateChanged(kIdle);
	}

	//Copy ShowLogs.m from appDirPath/../share/rtabmap/ShowLogs.m to working directory. (appDirPath is in bin)
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
	QString dbPath = _preferencesDialog->getWorkingDirectory() + "/LTM.db";
	if(_state == kMonitoring)
	{
		button = QMessageBox::question(this,
				tr("Deleting memory..."),
				tr("The remote database file \"%1\" will be deleted. Are you sure you want to continue? (This cannot be reverted)").arg(dbPath),
				QMessageBox::Yes|QMessageBox::No,
				QMessageBox::No);
	}
	else
	{
		button = QMessageBox::question(this,
				tr("Deleting memory..."),
				tr("The database file \"%1\" (%2 MB) will be deleted. Are you sure you want to continue? (This cannot be reverted)").arg(dbPath).arg(UFile::length(dbPath.toStdString())/1000000),
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
	_preferencesDialog->selectSource(PreferencesDialog::kSrcImages);
}

void MainWindow::selectVideo()
{
	_preferencesDialog->selectSource(PreferencesDialog::kSrcVideo);
}

void MainWindow::selectStream()
{
	_preferencesDialog->selectSource(PreferencesDialog::kSrcUsbDevice);
}

void MainWindow::selectDatabase()
{
	_preferencesDialog->selectSource(PreferencesDialog::kSrcDatabase);
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
		msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Cancel);
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
		}
		else if(_state == kDetecting)
		{
			_ui->actionPause->setToolTip(tr("Continue"));
			_ui->actionPause->setChecked(true);
			_ui->statusbar->showMessage(tr("Paused..."));
			_ui->actionReset_the_memory->setEnabled(true);
			_ui->actionDump_the_memory->setEnabled(true);
			_ui->actionDelete_memory->setEnabled(true);
			_ui->actionGenerate_map->setEnabled(true);
			_state = kPaused;
			_oneSecondTimer->stop();
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
