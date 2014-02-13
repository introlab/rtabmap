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

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include "rtabmap/utilite/UEventsHandler.h"
#include <QtGui/QMainWindow>
#include <QtCore/QSet>
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/Image.h"
#include "rtabmap/gui/PreferencesDialog.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

namespace rtabmap {
class CameraThread;
class DBReader;
class CameraOpenni;
class OdometryThread;
class CloudViewer;
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

class RTABMAPGUI_EXP MainWindow : public QMainWindow, public UEventsHandler
{
	Q_OBJECT

public:
	enum State {
		kIdle,
		kStartingDetection,
		kDetecting,
		kPaused,
		kMonitoring,
		kMonitoringPaused
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
	void setMonitoringState(bool pauseChecked = false); // in monitoring state, only some actions are enabled

public slots:
	void processStats(const rtabmap::Statistics & stat);

protected:
	virtual void closeEvent(QCloseEvent* event);
	virtual void handleEvent(UEvent* anEvent);
	virtual void resizeEvent(QResizeEvent* anEvent);

private slots:
	void changeState(MainWindow::State state);
	void beep();
	void startDetection();
	void pauseDetection();
	void stopDetection();
	void printLoopClosureIds();
	void generateMap();
	void generateLocalMap();
	void generateTOROMap();
	void deleteMemory();
	void openWorkingDirectory();
	void updateEditMenu();
	void selectImages();
	void selectVideo();
	void selectStream();
	void selectDatabase();
	void selectOpenni();
	void dumpTheMemory();
	void dumpThePrediction();
	void downloadAllClouds();
	void downloadPoseGraph();
	void clearTheCache();
	void saveFigures();
	void loadFigures();
	void openPreferences();
	void selectScreenCaptureFormat(bool checked);
	void takeScreenshot();
	void updateElapsedTime();
	void processOdometry(const rtabmap::Image & data);
	void applyAllPrefSettings();
	void applyPrefSettings(PreferencesDialog::PANEL_FLAGS flags);
	void applyPrefSettings(const rtabmap::ParametersMap & parameters);
	void processRtabmapEventInit(int status, const QString & info);
	void processRtabmapEvent3DMap(const rtabmap::RtabmapEvent3DMap & event);
	void changeImgRateSetting();
	void changeDetectionRateSetting();
	void changeTimeLimitSetting();
	void changeMappingMode();
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
	void savePointClouds();
	void saveMeshes();
	void viewPointClouds();
	void viewMeshes();
	void resetOdometry();
	void triggerNewMap();
	void updateNodeVisibility(int, bool);

signals:
	void statsReceived(const rtabmap::Statistics &);
	void odometryReceived(const rtabmap::Image &);
	void thresholdsChanged(int, int);
	void stateChanged(MainWindow::State);
	void rtabmapEventInitReceived(int status, const QString & info);
	void rtabmapEvent3DMapReceived(const rtabmap::RtabmapEvent3DMap & event);
	void imgRateChanged(double);
	void detectionRateChanged(double);
	void timeLimitChanged(float);
	void mappingModeChanged(bool);
	void noMoreImagesReceived();
	void loopClosureThrChanged(float);
	void twistReceived(float x, float y, float z, float roll, float pitch, float yaw, int row, int col);

private:
	void update3DMapVisibility(bool cloudsShown, bool scansShown);
	void updateMapCloud(const std::map<int, Transform> & poses, const Transform & pose);
	std::map<int, Transform> radiusPosesFiltering(const std::map<int, Transform> & poses) const;
	void drawKeypoints(const std::multimap<int, cv::KeyPoint> & refWords, const std::multimap<int, cv::KeyPoint> & loopWords);
	void setupMainLayout(bool vertical);
	void updateSelectSourceImageMenu(int type);
	void updateSelectSourceDatabase(bool used);
	void updateSelectSourceOpenni(bool used);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr createAssembledCloud();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr createCloud(
			int id,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			float depthConstant,
			const Transform & localTransform,
			const Transform & pose,
			float voxelSize,
			int decimation,
			float maxDepth);
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > createPointClouds();
	std::map<int, pcl::PolygonMesh::Ptr> createMeshes();

	void savePointClouds(const std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds);
	void saveMeshes(const std::map<int, pcl::PolygonMesh::Ptr> & meshes);

private:
	Ui_mainWindow * _ui;

	State _state;
	rtabmap::CameraThread * _camera;
	rtabmap::DBReader * _dbReader;
	rtabmap::CameraOpenni * _cameraOpenni;
	rtabmap::OdometryThread * _odomThread;

	SrcType _srcType;
	QString _srcPath;

	//Dialogs
	PreferencesDialog * _preferencesDialog;
	AboutDialog * _aboutDialog;

	QSet<int> _lastIds;
	int _lastId;
	bool _processingStatistics;
	bool _odometryReceived;

	QMap<int, std::vector<unsigned char> > _imagesMap;
	QMap<int, std::vector<unsigned char> > _depthsMap;
	std::map<int, std::vector<unsigned char> > _depths2DMap;
	QMap<int, float> _depthConstantsMap;
	QMap<int, Transform> _localTransformsMap;
	std::map<int, Transform> _currentPosesMap;
	Transform _odometryCorrection;
	Transform _lastOdomPose;
	bool _lastOdometryProcessed;

	QTimer * _oneSecondTimer;
	QTime * _elapsedTime;
	QTime * _logEventTime;

	PdfPlotCurve * _posteriorCurve;
	PdfPlotCurve * _likelihoodCurve;
	PdfPlotCurve * _rawLikelihoodCurve;

	DetailedProgressDialog * _initProgressDialog;

	QString _graphSavingFileName;
	QString _toroSavingFileName;
	bool _autoScreenCaptureOdomSync;

	QVector<int> _refIds;
	QVector<int> _loopClosureIds;
};

}

#endif /* MainWindow_H_ */
