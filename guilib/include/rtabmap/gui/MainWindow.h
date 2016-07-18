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

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include "rtabmap/utilite/UEventsHandler.h"
#include <QMainWindow>
#include <QtCore/QSet>
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/CameraInfo.h"
#include "rtabmap/gui/PreferencesDialog.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/pcl_base.h>
#include <pcl/TextureMesh.h>

namespace rtabmap {
class CameraThread;
class OdometryThread;
class CloudViewer;
class LoopClosureViewer;
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
class ProgressDialog;
class TwistGridWidget;
class ExportCloudsDialog;
class ExportScansDialog;
class PostProcessingDialog;
class DataRecorder;
class OctoMap;

class RTABMAPGUI_EXP MainWindow : public QMainWindow, public UEventsHandler
{
	Q_OBJECT

public:
	enum State {
		kIdle,
		kInitializing,
		kInitialized,
		kApplicationClosing,
		kClosing,
		kStartingDetection,
		kDetecting,
		kPaused,
		kMonitoring,
		kMonitoringPaused
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
	bool isSavedMaximized() const {return _savedMaximized;}

	bool isProcessingStatistics() const {return _processingStatistics;}
	bool isProcessingOdometry() const {return _processingOdometry;}

public slots:
	void processStats(const rtabmap::Statistics & stat);
	void updateCacheFromDatabase(const QString & path);
	void openDatabase(const QString & path);

protected:
	virtual void closeEvent(QCloseEvent* event);
	virtual void handleEvent(UEvent* anEvent);
	virtual void showEvent(QShowEvent* anEvent);
	virtual void moveEvent(QMoveEvent* anEvent);
	virtual void resizeEvent(QResizeEvent* anEvent);
	virtual bool eventFilter(QObject *obj, QEvent *event);

private slots:
	void changeState(MainWindow::State state);
	void beep();
	void configGUIModified();
	void saveConfigGUI();
	void newDatabase();
	void openDatabase();
	bool closeDatabase();
	void editDatabase();
	void startDetection();
	void pauseDetection();
	void stopDetection();
	void notifyNoMoreImages();
	void printLoopClosureIds();
	void generateGraphDOT();
	void exportPosesRaw();
	void exportPosesRGBDSLAM();
	void exportPosesKITTI();
	void exportPosesTORO();
	void exportPosesG2O();
	void exportImages();
	void exportOctomap();
	void postProcessing();
	void deleteMemory();
	void openWorkingDirectory();
	void updateEditMenu();
	void selectStream();
	void selectOpenni();
	void selectFreenect();
	void selectOpenniCv();
	void selectOpenniCvAsus();
	void selectOpenni2();
	void selectFreenect2();
	void selectStereoDC1394();
	void selectStereoFlyCapture2();
	void selectStereoZed();
	void selectStereoUsb();
	void dumpTheMemory();
	void dumpThePrediction();
	void sendGoal();
	void sendWaypoints();
	void postGoal(const QString & goal);
	void cancelGoal();
	void label();
	void updateCacheFromDatabase();
	void downloadAllClouds();
	void downloadPoseGraph();
	void anchorCloudsToGroundTruth();
	void clearTheCache();
	void openPreferences();
	void openPreferencesSource();
	void setDefaultViews();
	void selectScreenCaptureFormat(bool checked);
	void takeScreenshot();
	void updateElapsedTime();
	void processCameraInfo(const rtabmap::CameraInfo & info);
	void processOdometry(const rtabmap::OdometryEvent & odom, bool dataIgnored);
	void applyPrefSettings(PreferencesDialog::PANEL_FLAGS flags);
	void applyPrefSettings(const rtabmap::ParametersMap & parameters);
	void processRtabmapEventInit(int status, const QString & info);
	void processRtabmapEvent3DMap(const rtabmap::RtabmapEvent3DMap & event);
	void processRtabmapGlobalPathEvent(const rtabmap::RtabmapGlobalPathEvent & event);
	void processRtabmapLabelErrorEvent(int id, const QString & label);
	void processRtabmapGoalStatusEvent(int status);
	void changeImgRateSetting();
	void changeDetectionRateSetting();
	void changeTimeLimitSetting();
	void changeMappingMode();
	void setAspectRatio(int w, int h);
	void setAspectRatio16_9();
	void setAspectRatio16_10();
	void setAspectRatio4_3();
	void setAspectRatio240p();
	void setAspectRatio360p();
	void setAspectRatio480p();
	void setAspectRatio720p();
	void setAspectRatio1080p();
	void setAspectRatioCustom();
	void exportGridMap();
	void exportScans();
	void exportClouds();
	void exportBundlerFormat();
	void viewScans();
	void viewClouds();
	void resetOdometry();
	void triggerNewMap();
	void dataRecorder();
	void dataRecorderDestroyed();
	void updateNodeVisibility(int, bool);
	void updateGraphView();

signals:
	void statsReceived(const rtabmap::Statistics &);
	void cameraInfoReceived(const rtabmap::CameraInfo &);
	void odometryReceived(const rtabmap::OdometryEvent &, bool);
	void thresholdsChanged(int, int);
	void stateChanged(MainWindow::State);
	void rtabmapEventInitReceived(int status, const QString & info);
	void rtabmapEvent3DMapReceived(const rtabmap::RtabmapEvent3DMap & event);
	void rtabmapGlobalPathEventReceived(const rtabmap::RtabmapGlobalPathEvent & event);
	void rtabmapLabelErrorReceived(int id, const QString & label);
	void rtabmapGoalStatusEventReceived(int status);
	void imgRateChanged(double);
	void detectionRateChanged(double);
	void timeLimitChanged(float);
	void mappingModeChanged(bool);
	void noMoreImagesReceived();
	void loopClosureThrChanged(float);
	void twistReceived(float x, float y, float z, float roll, float pitch, float yaw, int row, int col);

private:
	void update3DMapVisibility(bool cloudsShown, bool scansShown);
	void updateMapCloud(
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & constraints,
			const std::map<int, int> & mapIds,
			const std::map<int, std::string> & labels,
			const std::map<int, Transform> & groundTruths,
			bool verboseProgress = false);
	std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> createAndAddCloudToMap(int nodeId,	const Transform & pose, int mapId);
	void createAndAddProjectionMap(
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
			const pcl::IndicesPtr & indices,
			int nodeId,
			const Transform & pose,
			bool updateOctomap = false);
	void createAndAddScanToMap(int nodeId, const Transform & pose, int mapId);
	void createAndAddFeaturesToMap(int nodeId, const Transform & pose, int mapId);
	Transform alignPosesToGroundTruth(std::map<int, Transform> & poses, const std::map<int, Transform> & groundTruth);
	void drawKeypoints(const std::multimap<int, cv::KeyPoint> & refWords, const std::multimap<int, cv::KeyPoint> & loopWords);
	void setupMainLayout(bool vertical);
	void updateSelectSourceMenu();
	void applyPrefSettings(const rtabmap::ParametersMap & parameters, bool postParamEvent);
	void saveFigures();
	void loadFigures();
	void exportPoses(int format);
	QString captureScreen(bool cacheInRAM = false);

private:
	Ui_mainWindow * _ui;

	State _state;
	rtabmap::CameraThread * _camera;
	rtabmap::OdometryThread * _odomThread;

	//Dialogs
	PreferencesDialog * _preferencesDialog;
	AboutDialog * _aboutDialog;
	ExportCloudsDialog * _exportCloudsDialog;
	ExportScansDialog * _exportScansDialog;
	PostProcessingDialog * _postProcessingDialog;
	DataRecorder * _dataRecorder;

	QSet<int> _lastIds;
	int _lastId;
	double _firstStamp;
	bool _processingStatistics;
	bool _processingDownloadedMap;
	bool _odometryReceived;
	QString _newDatabasePath;
	QString _newDatabasePathOutput;
	QString _openedDatabasePath;
	bool _databaseUpdated;
	bool _odomImageShow;
	bool _odomImageDepthShow;
	bool _savedMaximized;
	QStringList _waypoints;
	int _waypointsIndex;

	QMap<int, Signature> _cachedSignatures;
	long _cachedMemoryUsage;
	std::map<int, Transform> _currentPosesMap; // <nodeId, pose>
	std::map<int, Transform> _currentGTPosesMap; // <nodeId, pose>
	std::multimap<int, Link> _currentLinksMap; // <nodeFromId, link>
	std::map<int, int> _currentMapIds;   // <nodeId, mapId>
	std::map<int, std::string> _currentLabels; // <nodeId, label>
	std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> > _cachedClouds;
	long _createdCloudsMemoryUsage;
	std::pair<int, std::pair<std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>, pcl::IndicesPtr> > _previousCloud; // used for subtraction

	std::map<int, cv::Mat> _createdScans;
	std::map<int, std::pair<cv::Mat, cv::Mat> > _projectionLocalMaps; // <ground, obstacles>
	std::map<int, std::pair<cv::Mat, cv::Mat> > _gridLocalMaps; // <ground, obstacles>

	rtabmap::OctoMap * _octomap;

	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _createdFeatures;

	Transform _odometryCorrection;
	Transform _lastOdomPose;
	bool _processingOdometry;

	QTimer * _oneSecondTimer;
	QTime * _elapsedTime;
	QTime * _logEventTime;

	PdfPlotCurve * _posteriorCurve;
	PdfPlotCurve * _likelihoodCurve;
	PdfPlotCurve * _rawLikelihoodCurve;

	ProgressDialog * _initProgressDialog;

	CloudViewer * _cloudViewer;
	LoopClosureViewer * _loopClosureViewer;

	QString _graphSavingFileName;
	QMap<int, QString> _exportPosesFileName;
	bool _autoScreenCaptureOdomSync;
	bool _autoScreenCaptureRAM;
	QMap<QString, QByteArray> _autoScreenCaptureCachedImages;

	QVector<int> _refIds;
	QVector<int> _loopClosureIds;

	bool _firstCall;
};

}

#endif /* MainWindow_H_ */
