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

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include "rtabmap/utilite/UEventsHandler.h"
#include <QtGui/QMainWindow>
#include <QtCore/QSet>
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/gui/PreferencesDialog.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

namespace rtabmap {
class CameraThread;
class DBReader;
class CameraOpenni;
class CameraFreenect;
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
class ExportCloudsDialog;
class PostProcessingDialog;

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
	void newDatabase();
	void openDatabase();
	void closeDatabase();
	void editDatabase();
	void startDetection();
	void pauseDetection();
	void stopDetection();
	void printLoopClosureIds();
	void generateMap();
	void generateLocalMap();
	void generateTOROMap();
	void postProcessing();
	void deleteMemory();
	void openWorkingDirectory();
	void updateEditMenu();
	void selectImages();
	void selectVideo();
	void selectStream();
	void selectDatabase();
	void selectOpenni();
	void selectFreenect();
	void selectOpenniCv();
	void selectOpenniCvAsus();
	void selectOpenni2();
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
	void processOdometry(const rtabmap::SensorData & data, int quality, float time, int features, int localMapSize);
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
	void exportGridMap();
	void exportScans();
	void exportClouds();
	void viewScans();
	void viewClouds();
	void resetOdometry();
	void triggerNewMap();
	void dataRecorder();
	void updateNodeVisibility(int, bool);

signals:
	void statsReceived(const rtabmap::Statistics &);
	void odometryReceived(const rtabmap::SensorData &, int, float, int, int);
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
	void updateMapCloud(const std::map<int, Transform> & poses, const Transform & pose, const std::multimap<int, Link> & constraints, const std::map<int, int> & mapIds, bool verboseProgress = false);
	void createAndAddCloudToMap(int nodeId, const Transform & pose, int mapId);
	void createAndAddScanToMap(int nodeId, const Transform & pose, int mapId);
	void drawKeypoints(const std::multimap<int, cv::KeyPoint> & refWords, const std::multimap<int, cv::KeyPoint> & loopWords);
	void setupMainLayout(bool vertical);
	void updateSelectSourceImageMenu(int type);
	void updateSelectSourceDatabase(bool used);
	void updateSelectSourceRGBDMenu(bool used, PreferencesDialog::Src src);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getAssembledCloud(
			const std::map<int, Transform> & poses,
			float assembledVoxelSize,
			bool regenerateClouds,
			int regenerateDecimation,
			float regenerateVoxelSize,
			float regenerateMaxDepth) const;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr createCloud(
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
			float maxDepth) const;
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > getClouds(
			const std::map<int, Transform> & poses,
			bool regenerateClouds,
			int regenerateDecimation,
			float regenerateVoxelSize,
			float regenerateMaxDepth) const;

	bool getExportedScans(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans);
	bool getExportedClouds(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds, std::map<int, pcl::PolygonMesh::Ptr> & meshes, bool toSave);
	void saveClouds(const std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds, bool binaryMode = true);
	void saveMeshes(const std::map<int, pcl::PolygonMesh::Ptr> & meshes, bool binaryMode = true);
	void saveScans(const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> & clouds, bool binaryMode = true);

private:
	Ui_mainWindow * _ui;

	State _state;
	rtabmap::CameraThread * _camera;
	rtabmap::DBReader * _dbReader;
	rtabmap::OdometryThread * _odomThread;

	SrcType _srcType;
	QString _srcPath;

	//Dialogs
	PreferencesDialog * _preferencesDialog;
	AboutDialog * _aboutDialog;
	ExportCloudsDialog * _exportDialog;
	PostProcessingDialog * _postProcessingDialog;

	QSet<int> _lastIds;
	int _lastId;
	bool _processingStatistics;
	bool _odometryReceived;
	QString _openedDatabasePath;
	bool _emptyNewDatabase;

	QMap<int, Signature> _cachedSignatures;
	std::map<int, Transform> _currentPosesMap; // <nodeId, pose>
	std::multimap<int, Link> _currentLinksMap; // <nodeFromId, link>
	std::map<int, int> _currentMapIds;   // <nodeId, mapId>
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > _createdClouds;
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > _createdScans;
	std::map<int, std::pair<cv::Mat, cv::Mat> > _occupancyLocalMaps; // <ground, obstacles>
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
