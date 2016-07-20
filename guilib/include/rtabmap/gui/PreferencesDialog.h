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

#ifndef PREFERENCESDIALOG_H_
#define PREFERENCESDIALOG_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <QDialog>
#include <QtCore/QModelIndex>
#include <QtCore/QVector>
#include <set>

#include "rtabmap/core/Transform.h"
#include "rtabmap/core/Parameters.h"

class Ui_preferencesDialog;
class QAbstractItemModel;
class QAbstractButton;
class QStandardItemModel;
class QStandardItem;
class QFile;
class QGroupBox;
class QMainWindow;
class QLineEdit;
class QSlider;
class QProgressDialog;
class UPlotCurve;
class QStackedWidget;
class QCheckBox;
class QSpinBox;
class QDoubleSpinBox;

namespace rtabmap {

class Signature;
class LoopClosureViewer;
class Camera;
class CalibrationDialog;
class CreateSimpleCalibrationDialog;

class RTABMAPGUI_EXP PreferencesDialog : public QDialog
{
	Q_OBJECT

public:
	enum PanelFlag {
		kPanelDummy = 0,
		kPanelGeneral = 1,
		kPanelCloudRendering = 2,
		kPanelLogging = 4,
		kPanelSource = 8,
		kPanelAll = 15
	};
	// TODO, tried to change the name of PANEL_FLAGS to PanelFlags... but signals/slots errors appeared...
	Q_DECLARE_FLAGS(PANEL_FLAGS, PanelFlag);

	enum Src {
		kSrcUndef = -1,

		kSrcRGBD           = 0,
		kSrcOpenNI_PCL     = 0,
		kSrcFreenect       = 1,
		kSrcOpenNI_CV      = 2,
		kSrcOpenNI_CV_ASUS = 3,
		kSrcOpenNI2        = 4,
		kSrcFreenect2      = 5,
		kSrcRGBDImages     = 6,

		kSrcStereo         = 100,
		kSrcDC1394         = 100,
		kSrcFlyCapture2    = 101,
		kSrcStereoImages   = 102,
		kSrcStereoVideo    = 103,
		kSrcStereoZed      = 104,
		kSrcStereoUsb      = 105,

		kSrcRGB            = 200,
		kSrcUsbDevice      = 200,
		kSrcImages         = 201,
		kSrcVideo          = 202,

		kSrcDatabase       = 300
	};

public:
	PreferencesDialog(QWidget * parent = 0);
	virtual ~PreferencesDialog();

	virtual QString getIniFilePath() const;
	virtual QString getTmpIniFilePath() const;
	void init();
	void setCurrentPanelToSource();

	// save stuff
	void saveSettings();
	void saveWindowGeometry(const QWidget * window);
	void loadWindowGeometry(QWidget * window);
	void saveMainWindowState(const QMainWindow * mainWindow);
	void loadMainWindowState(QMainWindow * mainWindow, bool & maximized, bool & statusBarShown);
	void saveWidgetState(const QWidget * widget);
	void loadWidgetState(QWidget * widget);

	void saveCustomConfig(const QString & section, const QString & key, const QString & value);
	QString loadCustomConfig(const QString & section, const QString & key);

	rtabmap::ParametersMap getAllParameters() const;
	void updateParameters(const ParametersMap & parameters);

	//General panel
	int getGeneralLoggerLevel() const;
	int getGeneralLoggerEventLevel() const;
	int getGeneralLoggerPauseLevel() const;
	int getGeneralLoggerType() const;
	bool getGeneralLoggerPrintTime() const;
	bool getGeneralLoggerPrintThreadId() const;
	bool isVerticalLayoutUsed() const;
	bool imageRejectedShown() const;
	bool imageHighestHypShown() const;
	bool beepOnPause() const;
	bool isTimeUsedInFigures() const;
	bool notifyWhenNewGlobalPathIsReceived() const;
	int getOdomQualityWarnThr() const;
	bool isPosteriorGraphView() const;

	bool isGraphsShown() const;
	bool isLabelsShown() const;
	double getMapVoxel() const;
	double getMapNoiseRadius() const;
	int getMapNoiseMinNeighbors() const;
	bool isCloudsShown(int index) const;      // 0=map, 1=odom
	bool isOctomapShown() const;
	int getOctomapTreeDepth() const;
	bool isOctomapGroundAnObstacle() const;
	int getCloudDecimation(int index) const;   // 0=map, 1=odom
	double getCloudMaxDepth(int index) const;  // 0=map, 1=odom
	double getCloudMinDepth(int index) const;  // 0=map, 1=odom
	double getCloudOpacity(int index) const;   // 0=map, 1=odom
	int getCloudPointSize(int index) const;    // 0=map, 1=odom

	bool isScansShown(int index) const;       // 0=map, 1=odom
	int getDownsamplingStepScan(int index) const; // 0=map, 1=odom
	double getCloudVoxelSizeScan(int index) const; // 0=map, 1=odom
	double getScanOpacity(int index) const;    // 0=map, 1=odom
	int getScanPointSize(int index) const;     // 0=map, 1=odom

	bool isFeaturesShown(int index) const;     // 0=map, 1=odom
	int getFeaturesPointSize(int index) const; // 0=map, 1=odom

	bool isCloudFiltering() const;
	bool isSubtractFiltering() const;
	double getCloudFilteringRadius() const;
	double getCloudFilteringAngle() const;
	int getSubtractFilteringMinPts() const;
	double getSubtractFilteringRadius() const;
	double getSubtractFilteringAngle() const;
	int getNormalKSearch() const;

	bool getGridMapShown() const;
	double getGridMapResolution() const;;
	bool isGridMapEroded() const;
	bool isGridMapFrom3DCloud() const;
	bool projMapFrame() const;
	double projMaxGroundAngle() const;
	double projMaxGroundHeight() const;
	int projMinClusterSize() const;
	double projMaxObstaclesHeight() const;
	bool projFlatObstaclesDetected() const;
	double getGridMapOpacity() const;

	bool isCloudMeshing() const;
	double getCloudMeshingAngle() const;
	bool isCloudMeshingQuad() const;
	int getCloudMeshingTriangleSize();

	QString getWorkingDirectory() const;

	// source panel
	double getGeneralInputRate() const;
	bool isSourceMirroring() const;
	PreferencesDialog::Src getSourceType() const;
	PreferencesDialog::Src getSourceDriver() const;
	QString getSourceDriverStr() const;
	QString getSourceDevice() const;

	bool getSourceDatabaseStampsUsed() const;
	bool isSourceRGBDColorOnly() const;
	int getSourceImageDecimation() const;
	bool isSourceStereoDepthGenerated() const;
	bool isSourceScanFromDepth() const;
	int getSourceScanFromDepthDecimation() const;
	double getSourceScanFromDepthMaxDepth() const;
	double getSourceScanVoxelSize() const;
	int getSourceScanNormalsK() const;
	Transform getSourceLocalTransform() const;    //Openni group
	Transform getLaserLocalTransform() const; // directory images
	Camera * createCamera(bool useRawImages = false); // return camera should be deleted if not null

	int getIgnoredDCComponents() const;

	//
	bool isImagesKept() const;
	bool isCloudsKept() const;
	float getTimeLimit() const;
	float getDetectionRate() const;
	bool isSLAMMode() const;
	bool isRGBDMode() const;

	//specific
	bool isStatisticsPublished() const;
	double getLoopThr() const;
	double getVpThr() const;
	double getSimThr() const;
	int getOdomStrategy() const;
	int getOdomBufferSize() const;
	bool getRegVarianceFromInliersCount() const;
	QString getCameraInfoDir() const; // "workinfDir/camera_info"

	//
	void setMonitoringState(bool monitoringState) {_monitoringState = monitoringState;}

signals:
	void settingsChanged(PreferencesDialog::PANEL_FLAGS);
	void settingsChanged(rtabmap::ParametersMap);

public slots:
	void setInputRate(double value);
	void setDetectionRate(double value);
	void setTimeLimit(float value);
	void setSLAMMode(bool enabled);
	void selectSourceDriver(Src src);
	void calibrate();
	void calibrateSimple();

private slots:
	void closeDialog ( QAbstractButton * button );
	void resetApply ( QAbstractButton * button );
	void resetSettings(int panelNumber);
	void loadConfigFrom();
	bool saveConfigTo();
	void resetConfig();
	void makeObsoleteGeneralPanel();
	void makeObsoleteCloudRenderingPanel();
	void makeObsoleteLoggingPanel();
	void makeObsoleteSourcePanel();
	void clicked(const QModelIndex & current, const QModelIndex & previous);
	void addParameter(int value);
	void addParameter(bool value);
	void addParameter(double value);
	void addParameter(const QString & value);
	void updatePredictionPlot();
	void updateKpROI();
	void updateG2oVisibility();
	void updateStereoDisparityVisibility();
	void useOdomFeatures();
	void changeWorkingDirectory();
	void changeDictionaryPath();
	void changeOdomBowFixedLocalMapPath();
	void readSettingsEnd();
	void setupTreeView();
	void updateBasicParameter();
	void openDatabaseViewer();
	void selectSourceDatabase();
	void selectCalibrationPath();
	void selectSourceImagesStamps();
	void selectSourceRGBDImagesPathRGB();
	void selectSourceRGBDImagesPathDepth();
	void selectSourceImagesPathScans();
	void selectSourceImagesPathGt();
	void selectSourceStereoImagesPathLeft();
	void selectSourceStereoImagesPathRight();
	void selectSourceImagesPath();
	void selectSourceVideoPath();
	void selectSourceStereoVideoPath();
	void selectSourceOniPath();
	void selectSourceOni2Path();
	void selectSourceSvoPath();
	void updateSourceGrpVisibility();
	void testOdometry();
	void testCamera();

protected:
	virtual void showEvent ( QShowEvent * event );
	virtual void closeEvent(QCloseEvent *event);

	virtual QString getParamMessage();

	virtual void readGuiSettings(const QString & filePath = QString());
	virtual void readCameraSettings(const QString & filePath = QString());
	virtual bool readCoreSettings(const QString & filePath = QString());

	virtual void writeGuiSettings(const QString & filePath = QString()) const;
	virtual void writeCameraSettings(const QString & filePath = QString()) const;
	virtual void writeCoreSettings(const QString & filePath = QString()) const;

	void setParameter(const std::string & key, const std::string & value);

private:
	void readSettings(const QString & filePath = QString());
	void writeSettings(const QString & filePath = QString());
	bool validateForm();
	void setupSignals();
	void setupKpRoiPanel();
	bool parseModel(QList<QGroupBox*> & boxes, QStandardItem * parentItem, int currentLevel, int & absoluteIndex);
	void resetSettings(QGroupBox * groupBox);
	void addParameter(const QObject * object, int value);
	void addParameter(const QObject * object, bool value);
	void addParameter(const QObject * object, double value);
	void addParameter(const QObject * object, const QString & value);
	void addParameters(const QObjectList & children);
	void addParameters(const QStackedWidget * stackedWidget, int panel = -1);
	void addParameters(const QGroupBox * box);
	QList<QGroupBox*> getGroupBoxes();
	void readSettingsBegin();

protected:
	PANEL_FLAGS _obsoletePanels;

private:
	rtabmap::ParametersMap _modifiedParameters;
	rtabmap::ParametersMap _parameters;
	Ui_preferencesDialog * _ui;
	QStandardItemModel * _indexModel;
	bool _initialized;
	bool _monitoringState;

	QProgressDialog * _progressDialog;

	//calibration
	CalibrationDialog * _calibrationDialog;
	CreateSimpleCalibrationDialog * _createCalibrationDialog;

	QVector<QCheckBox*> _3dRenderingShowClouds;
	QVector<QSpinBox*> _3dRenderingDecimation;
	QVector<QDoubleSpinBox*> _3dRenderingMaxDepth;
	QVector<QDoubleSpinBox*> _3dRenderingMinDepth;
	QVector<QDoubleSpinBox*> _3dRenderingOpacity;
	QVector<QSpinBox*> _3dRenderingPtSize;
	QVector<QCheckBox*> _3dRenderingShowScans;
	QVector<QSpinBox*> _3dRenderingDownsamplingScan;
	QVector<QDoubleSpinBox*> _3dRenderingVoxelSizeScan;
	QVector<QDoubleSpinBox*> _3dRenderingOpacityScan;
	QVector<QSpinBox*> _3dRenderingPtSizeScan;
	QVector<QCheckBox*> _3dRenderingShowFeatures;
	QVector<QSpinBox*> _3dRenderingPtSizeFeatures;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(PreferencesDialog::PANEL_FLAGS)

}

#endif /* PREFERENCESDIALOG_H_ */
