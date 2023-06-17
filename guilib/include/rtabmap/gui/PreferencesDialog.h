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

#ifndef RTABMAP_PREFERENCESDIALOG_H_
#define RTABMAP_PREFERENCESDIALOG_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

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

class RTABMAP_GUI_EXPORT PreferencesDialog : public QDialog
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
	Q_DECLARE_FLAGS(PANEL_FLAGS, PanelFlag)

	enum Src {
		kSrcUndef = -1,

		kSrcRGBD           = 0,
		kSrcOpenNI_PCL     = 0,
		kSrcFreenect       = 1,
		kSrcOpenNI_CV      = 2,
		kSrcOpenNI_CV_ASUS = 3,
		kSrcOpenNI2        = 4,
		kSrcFreenect2      = 5,
		kSrcRealSense      = 6,
		kSrcRGBDImages     = 7,
		kSrcK4W2           = 8,
		kSrcRealSense2     = 9,
		kSrcK4A            = 10,

		kSrcStereo         = 100,
		kSrcDC1394         = 100,
		kSrcFlyCapture2    = 101,
		kSrcStereoImages   = 102,
		kSrcStereoVideo    = 103,
		kSrcStereoZed      = 104,
		kSrcStereoUsb      = 105,
		kSrcStereoTara 	   = 106,
		kSrcStereoRealSense2 = 107,
		kSrcStereoMyntEye  = 108,
		kSrcStereoZedOC    = 109,
		kSrcStereoDepthAI  = 110,

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
	void init(const QString & iniFilePath = "");
	void setCurrentPanelToSource();
	virtual QString getDefaultWorkingDirectory() const;

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
	std::string getParameter(const std::string & key) const;
	void updateParameters(const ParametersMap & parameters, bool setOtherParametersToDefault = false);

	//General panel
	int getGeneralLoggerLevel() const;
	int getGeneralLoggerEventLevel() const;
	int getGeneralLoggerPauseLevel() const;
	int getGeneralLoggerType() const;
	bool getGeneralLoggerPrintTime() const;
	bool getGeneralLoggerPrintThreadId() const;
	std::vector<std::string> getGeneralLoggerThreads() const;
	bool isVerticalLayoutUsed() const;
	bool imageRejectedShown() const;
	bool imageHighestHypShown() const;
	bool beepOnPause() const;
	bool isTimeUsedInFigures() const;
	bool isCacheSavedInFigures() const;
	bool notifyWhenNewGlobalPathIsReceived() const;
	int getOdomQualityWarnThr() const;
	bool isOdomOnlyInliersShown() const;
	bool isPosteriorGraphView() const;
	bool isWordsCountGraphView() const;
	bool isLocalizationsCountGraphView() const;
	bool isRelocalizationColorOdomCacheGraphView() const;
	int getOdomRegistrationApproach() const;
	double getOdomF2MGravitySigma() const;
	bool isOdomDisabled() const;
	bool isOdomSensorAsGt() const;
	bool isGroundTruthAligned() const;

	bool isGraphsShown() const;
	bool isLabelsShown() const;
	bool isFramesShown() const;
	bool isLandmarksShown() const;
	double landmarkVisSize() const;
	bool isIMUGravityShown(int index) const;
	double getIMUGravityLength(int index) const;
	bool isIMUAccShown() const;
	bool isMarkerDetection() const;
	double getMarkerLength() const;
	double getVoxel() const;
	double getNoiseRadius() const;
	int getNoiseMinNeighbors() const;
	double getCeilingFilteringHeight() const;
	double getFloorFilteringHeight() const;
	int getNormalKSearch() const;
	double getNormalRadiusSearch() const;
	double getScanCeilingFilteringHeight() const;
	double getScanFloorFilteringHeight() const;
	int getScanNormalKSearch() const;
	double getScanNormalRadiusSearch() const;
	bool isCloudsShown(int index) const;      // 0=map, 1=odom
	bool isOctomapUpdated() const;
	bool isOctomapShown() const;
	int getOctomapRenderingType() const;
	bool isOctomap2dGrid() const;
	int getOctomapTreeDepth() const;
	int getOctomapPointSize() const;
	int getCloudDecimation(int index) const;   // 0=map, 1=odom
	double getCloudMaxDepth(int index) const;  // 0=map, 1=odom
	double getCloudMinDepth(int index) const;  // 0=map, 1=odom
	std::vector<float> getCloudRoiRatios(int index) const; // 0=map, 1=odom
	int getCloudColorScheme(int index) const;   // 0=map, 1=odom
	double getCloudOpacity(int index) const;   // 0=map, 1=odom
	int getCloudPointSize(int index) const;    // 0=map, 1=odom

	bool isScansShown(int index) const;       // 0=map, 1=odom
	int getDownsamplingStepScan(int index) const; // 0=map, 1=odom
	double getScanMaxRange(int index) const; // 0=map, 1=odom
	double getScanMinRange(int index) const; // 0=map, 1=odom
	double getCloudVoxelSizeScan(int index) const; // 0=map, 1=odom
	int getScanColorScheme(int index) const;    // 0=map, 1=odom
	double getScanOpacity(int index) const;    // 0=map, 1=odom
	int getScanPointSize(int index) const;     // 0=map, 1=odom

	bool isFeaturesShown(int index) const;     // 0=map, 1=odom
	bool isFrustumsShown(int index) const;     // 0=map, 1=odom
	int getFeaturesPointSize(int index) const; // 0=map, 1=odom

	bool isCloudFiltering() const;
	bool isSubtractFiltering() const;
	double getCloudFilteringRadius() const;
	double getCloudFilteringAngle() const;
	int getSubtractFilteringMinPts() const;
	double getSubtractFilteringRadius() const;
	double getSubtractFilteringAngle() const;

	bool getGridMapShown() const;
	int getGridMapSensor() const;
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
	bool isCloudMeshingTexture() const;
	int getCloudMeshingTriangleSize();

	QString getWorkingDirectory() const;

	// source panel
	double getGeneralInputRate() const;
	bool isSourceMirroring() const;
	PreferencesDialog::Src getSourceType() const;
	PreferencesDialog::Src getSourceDriver() const;
	QString getSourceDriverStr() const;
	QString getSourceDevice() const;
	PreferencesDialog::Src getOdomSourceDriver() const;

	bool isSourceDatabaseStampsUsed() const;
	bool isSourceDatabaseStereoToDepth() const;
	bool isSourceRGBDColorOnly() const;
	int getIMUFilteringStrategy() const;
	bool getIMUFilteringBaseFrameConversion() const;
	bool isDepthFilteringAvailable() const;
	QString getSourceDistortionModel() const;
	bool isBilateralFiltering() const;
	double getBilateralSigmaS() const;
	double getBilateralSigmaR() const;
	int getSourceImageDecimation() const;
	bool isSourceStereoDepthGenerated() const;
	bool isSourceStereoExposureCompensation() const;
	bool isSourceScanFromDepth() const;
	int getSourceScanDownsampleStep() const;
	double getSourceScanRangeMin() const;
	double getSourceScanRangeMax() const;
	double getSourceScanVoxelSize() const;
	int getSourceScanNormalsK() const;
	double getSourceScanNormalsRadius() const;
	double getSourceScanForceGroundNormalsUp() const;
	Transform getSourceLocalTransform() const;    //Openni group
	Transform getLaserLocalTransform() const; // directory images
	Transform getIMULocalTransform() const; // directory images
	QString getIMUPath() const;
	int getIMURate() const;
	Camera * createCamera(bool useRawImages = false, bool useColor = true); // return camera should be deleted if not null
	Camera * createOdomSensor(Transform & extrinsics, double & timeOffset, float & scaleFactor); // return camera should be deleted if not null

	int getIgnoredDCComponents() const;

	//
	bool isImagesKept() const;
	bool isMissingCacheRepublished() const;
	bool isCloudsKept() const;
	float getTimeLimit() const;
	float getDetectionRate() const;
	bool isSLAMMode() const;
	bool isRGBDMode() const;
	int getKpMaxFeatures() const;
	bool isPriorIgnored() const;

	//specific
	bool isStatisticsPublished() const;
	double getLoopThr() const;
	double getVpThr() const;
	double getSimThr() const;
	int getOdomStrategy() const;
	int getOdomBufferSize() const;
	QString getCameraInfoDir() const; // "workinfDir/camera_info"

	//
	void setMonitoringState(bool monitoringState) {_monitoringState = monitoringState;}

Q_SIGNALS:
	void settingsChanged(PreferencesDialog::PANEL_FLAGS);
	void settingsChanged(rtabmap::ParametersMap);

public Q_SLOTS:
	void setInputRate(double value);
	void setDetectionRate(double value);
	void setTimeLimit(float value);
	void setSLAMMode(bool enabled);
	void selectSourceDriver(Src src, int variant = 0);
	void calibrate();
	void calibrateSimple();
	void calibrateOdomSensorExtrinsics();

private Q_SLOTS:
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
	void updateStereoDisparityVisibility();
	void updateFeatureMatchingVisibility();
	void updateOdometryStackedIndex(int index);
	void useOdomFeatures();
	void changeWorkingDirectory();
	void changeDictionaryPath();
	void changeOdometryORBSLAMVocabulary();
	void changeOdometryOKVISConfigPath();
	void changeOdometryVINSConfigPath();
	void changeIcpPMConfigPath();
	void changeSuperPointModelPath();
	void changePyMatcherPath();
	void changePyMatcherModel();
	void changePyDetectorPath();
	void readSettingsEnd();
	void setupTreeView();
	void updateBasicParameter();
	void openDatabaseViewer();
	void visualizeDistortionModel();
	void selectSourceDatabase();
	void selectCalibrationPath();
	void selectOdomSensorCalibrationPath();
	void selectSourceImagesStamps();
	void selectSourceRGBDImagesPathRGB();
	void selectSourceRGBDImagesPathDepth();
	void selectSourceImagesPathScans();
	void selectSourceImagesPathIMU();
	void selectSourceImagesPathOdom();
	void selectSourceImagesPathGt();
	void selectSourceStereoImagesPathLeft();
	void selectSourceStereoImagesPathRight();
	void selectSourceImagesPath();
	void selectSourceVideoPath();
	void selectSourceStereoVideoPath();
	void selectSourceStereoVideoPath2();
	void selectSourceDistortionModel();
	void selectSourceOniPath();
	void selectSourceOni2Path();
	void selectSourceMKVPath();
	void selectSourceSvoPath();
	void selectSourceRealsense2JsonPath();
	void selectSourceDepthaiBlobPath();
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
	Camera * createCamera(Src driver, const QString & device, const QString & calibrationPath, bool useRawImages, bool useColor, bool odomOnly, bool odomSensorExtrinsicsCalib); // return camera should be deleted if not null

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
	QVector<QLineEdit*> _3dRenderingRoiRatios;
	QVector<QSpinBox*> _3dRenderingColorScheme;
	QVector<QDoubleSpinBox*> _3dRenderingOpacity;
	QVector<QSpinBox*> _3dRenderingPtSize;
	QVector<QCheckBox*> _3dRenderingShowScans;
	QVector<QSpinBox*> _3dRenderingDownsamplingScan;
	QVector<QDoubleSpinBox*> _3dRenderingMaxRange;
	QVector<QDoubleSpinBox*> _3dRenderingMinRange;
	QVector<QDoubleSpinBox*> _3dRenderingVoxelSizeScan;
	QVector<QSpinBox*> _3dRenderingColorSchemeScan;
	QVector<QDoubleSpinBox*> _3dRenderingOpacityScan;
	QVector<QSpinBox*> _3dRenderingPtSizeScan;
	QVector<QCheckBox*> _3dRenderingShowFeatures;
	QVector<QCheckBox*> _3dRenderingShowFrustums;
	QVector<QSpinBox*> _3dRenderingPtSizeFeatures;
	QVector<QCheckBox*> _3dRenderingGravity;
	QVector<QDoubleSpinBox*> _3dRenderingGravityLength;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(PreferencesDialog::PANEL_FLAGS)

}

#endif /* PREFERENCESDIALOG_H_ */
