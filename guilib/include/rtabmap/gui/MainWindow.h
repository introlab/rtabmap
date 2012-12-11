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

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "utilite/UEventsHandler.h"
#include <QtGui/QMainWindow>
#include <QtCore/QSet>
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/gui/PreferencesDialog.h"

namespace rtabmap {
class Camera;
class DBReader;
}

class QGraphicsScene;
class Ui_mainWindow;
class QActionGroup;

namespace rtabmap {

class LikelihoodScene;
class AboutDialog;
class Plot;
class PdfPlotCurve;
class StatsToolBox;
class DetailedProgressDialog;
class TwistGridWidget;

class RTABMAP_EXP MainWindow : public QMainWindow, public UEventsHandler
{
	Q_OBJECT

public:
	enum State {
		kIdle,
		kStartingDetection,
		kDetecting,
		kPaused,
		kMonitoring
	};

	enum SrcType {
		kSrcUndefined,
		kSrcVideo,
		kSrcImages,
		kSrcStream
	};

public:
	/**
	 * @param prefDialog If NULL, a default dialog is created. This
	 *                   dialog is automatically destroyed with the MainWindow.
	 */
	MainWindow(PreferencesDialog * prefDialog = 0, QWidget * parent = 0);
	virtual ~MainWindow();

	QString getWorkingDirectory() const;

public slots:
	void changeState(MainWindow::State state);

protected:
	virtual void closeEvent(QCloseEvent* event);
	virtual void handleEvent(UEvent* anEvent);
	virtual void resizeEvent(QResizeEvent* anEvent);

private slots:
	void beep();
	void startDetection();
	void pauseDetection();
	void stopDetection();
	void printLoopClosureIds();
	void generateMap();
	void generateLocalMap();
	void deleteMemory();
	void openWorkingDirectory();
	void updateEditMenu();
	void selectImages();
	void selectVideo();
	void selectStream();
	void selectDatabase();
	void resetTheMemory();
	void dumpTheMemory();
	void dumpThePrediction();
	void clearTheCache();
	void saveFigures();
	void loadFigures();
	void selectScreenCaptureFormat(bool checked);
	void updateElapsedTime();
	void processStats(const rtabmap::Statistics & stat);
	void applyAllPrefSettings();
	void applyPrefSettings(PreferencesDialog::PANEL_FLAGS flags);
	void applyPrefSettings(const rtabmap::ParametersMap & parameters);
	void processRtabmapEventInit(int status, const QString & info);
	void changeImgRateSetting();
	void changeTimeLimitSetting();
	void captureScreen();
	void setAspectRatio(int w, int h);
	void setAspectRatio16_9();
	void setAspectRatio16_10();
	void setAspectRatio4_3();
	void setAspectRatio240p();
	void setAspectRatio360p();
	void setAspectRatio480p();
	void setAspectRatio720p();
	void setAspectRatio1080p();

signals:
	void statsReceived(const rtabmap::Statistics &);
	void thresholdsChanged(int, int);
	void stateChanged(MainWindow::State);
	void rtabmapEventInitReceived(int status, const QString & info);
	void imgRateChanged(double);
	void timeLimitChanged(float);
	void noMoreImagesReceived();
	void loopClosureThrChanged(float);
	void twistReceived(float x, float y, float z, float roll, float pitch, float yaw, int row, int col);

private:
	void drawKeypoints(const std::multimap<int, cv::KeyPoint> & refWords, const std::multimap<int, cv::KeyPoint> & loopWords);
	void setupMainLayout(bool vertical);
	void updateSelectSourceImageMenu(int type);
	void updateSelectSourceDatabase(bool used);

private:
	Ui_mainWindow * _ui;

	State _state;
	rtabmap::Camera * _camera;
	rtabmap::DBReader * _dbReader;

	SrcType _srcType;
	QString _srcPath;

	//Dialogs
	PreferencesDialog * _preferencesDialog;
	AboutDialog * _aboutDialog;

	QSet<int> _lastIds;
	int _lastId;
	bool _processingStatistics;

	QMap<int, QByteArray> _imagesMap;

	QTimer * _oneSecondTimer;
	QTime * _elapsedTime;
	QTime * _logEventTime;

	PdfPlotCurve * _posteriorCurve;
	PdfPlotCurve * _likelihoodCurve;
	PdfPlotCurve * _rawLikelihoodCurve;

	DetailedProgressDialog * _initProgressDialog;
	QActionGroup * _selectSourceImageGrp;

	QString _graphSavingFileName;
	QString _autoScreenCaptureFormat;

	QVector<int> _refIds;
	QVector<int> _loopClosureIds;
};

}

#endif /* MainWindow_H_ */
