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

#ifndef PREFERENCESDIALOG_H_
#define PREFERENCESDIALOG_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <QtGui/QDialog>
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

class OdometryThread;
class CameraThread;
class Signature;
class LoopClosureViewer;

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
		kSrcUndef,
		kSrcUsbDevice,
		kSrcImages,
		kSrcVideo,
		kSrcOpenNI_PCL,
		kSrcFreenect,
		kSrcOpenNI_CV,
		kSrcOpenNI_CV_ASUS,
		kSrcOpenNI2
	};

public:
	PreferencesDialog(QWidget * parent = 0);
	virtual ~PreferencesDialog();

	virtual QString getIniFilePath() const;
	void init();

	void saveWindowGeometry(const QString & windowName, const QWidget * window);
	void loadWindowGeometry(const QString & windowName, QWidget * window);
	void saveMainWindowState(const QMainWindow * mainWindow);
	void loadMainWindowState(QMainWindow * mainWindow);

	void saveCustomConfig(const QString & section, const QString & key, const QString & value);
	QString loadCustomConfig(const QString & section, const QString & key);

	rtabmap::ParametersMap getAllParameters();

	//General panel
	int getGeneralLoggerLevel() const;
	int getGeneralLoggerEventLevel() const;
	int getGeneralLoggerPauseLevel() const;
	int getGeneralLoggerType() const;
	bool getGeneralLoggerPrintTime() const;
	bool isVerticalLayoutUsed() const;
	bool isImageFlipped() const;
	bool imageRejectedShown() const;
	bool imageHighestHypShown() const;
	bool beepOnPause() const;
	int getKeypointsOpacity() const;
	int getOdomQualityWarnThr() const;

	bool isGraphsShown() const;
	bool isCloudMeshing() const;
	bool isCloudsShown(int index) const;      // 0=map, 1=odom
	double getCloudVoxelSize(int index) const; // 0=map, 1=odom
	int getCloudDecimation(int index) const;   // 0=map, 1=odom
	double getCloudMaxDepth(int index) const;  // 0=map, 1=odom
	double getCloudOpacity(int index) const;   // 0=map, 1=odom
	int getCloudPointSize(int index) const;    // 0=map, 1=odom

	bool isScansShown(int index) const;       // 0=map, 1=odom
	double getScanOpacity(int index) const;    // 0=map, 1=odom
	int getScanPointSize(int index) const;     // 0=map, 1=odom

	int getMeshNormalKSearch() const;
	double getMeshGP3Radius() const;
	bool getMeshSmoothing() const;
	double getMeshSmoothingRadius() const;

	bool isCloudFiltering() const;
	double getCloudFilteringRadius() const;
	double getCloudFilteringAngle() const;

	bool getGridMapShown() const;
	double getGridMapResolution() const;
	bool getGridMapFillEmptySpace() const;
	bool isGridMapFrom3DCloud() const;
	int getGridMapFillEmptyRadius() const;
	double getGridMapOpacity() const;

	QString getWorkingDirectory() const;

	// source panel
	double getGeneralInputRate() const;
	bool isSourceImageUsed() const;
	bool isSourceDatabaseUsed() const;
	bool isSourceOpenniUsed() const;
	int getSourceImageType() const;
	QString getSourceImageTypeStr() const;
	int getSourceWidth() const;
	int getSourceHeight() const;
	QString getSourceImagesPath() const;	//Images group
	QString getSourceImagesSuffix() const;	//Images group
	int getSourceImagesSuffixIndex() const;	//Images group
	int getSourceImagesStartPos() const;	//Images group
	bool getSourceImagesRefreshDir() const;	//Images group
	QString getSourceVideoPath() const;	//Video group
	int getSourceUsbDeviceId() const;		//UsbDevice group
	QString getSourceDatabasePath() const; //Database group
	bool getSourceDatabaseOdometryIgnored() const; //Database group
	int getSourceDatabaseStartPos() const; //Database group
	Src getSourceRGBD() const; 			// Openni group
	bool getSourceOpenni2AutoWhiteBalance() const;  //Openni group
	bool getSourceOpenni2AutoExposure() const;  //Openni group
	int getSourceOpenni2Exposure() const;  //Openni group
	int getSourceOpenni2Gain() const;   //Openni group
	QString getSourceOpenniDevice() const;            //Openni group
	Transform getSourceOpenniLocalTransform() const;    //Openni group
	float getSourceOpenniFx() const; // Openni group
	float getSourceOpenniFy() const; // Openni group
	float getSourceOpenniCx() const; // Openni group
	float getSourceOpenniCy() const; // Openni group

	int getIgnoredDCComponents() const;

	//
	bool isImagesKept() const;
	float getTimeLimit() const;
	float getDetectionRate() const;
	bool isSLAMMode() const;

	//specific
	bool isStatisticsPublished() const;
	double getLoopThr() const;
	double getVpThr() const;
	int getOdomStrategy() const;

	//
	void setMonitoringState(bool monitoringState) {_monitoringState = monitoringState;}

signals:
	void settingsChanged(PreferencesDialog::PANEL_FLAGS);
	void settingsChanged(rtabmap::ParametersMap);

public slots:
	void setInputRate(double value);
	void setDetectionRate(double value);
	void setHardThr(int value);
	void setTimeLimit(float value);
	void setSLAMMode(bool enabled);
	void selectSourceImage(Src src = kSrcUndef);
	void selectSourceDatabase(bool user = false);
	void selectSourceRGBD(Src src = kSrcUndef);

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
	void clicked(const QModelIndex &index);
	void addParameter(int value);
	void addParameter(bool value);
	void addParameter(double value);
	void addParameter(const QString & value);
	void updatePredictionPlot();
	void updateKpROI();
	void changeWorkingDirectory();
	void changeDictionaryPath();
	void readSettingsEnd();
	void setupTreeView();
	void updateBasicParameter();
	void openDatabaseViewer();
	void showOpenNI2GroupBox(bool);
	void cleanOdometryTest();
	void testOdometry();
	void cleanRGBDCameraTest();
	void testRGBDCamera();
	void calibrate();
	void resetCalibration();

protected:
	virtual void showEvent ( QShowEvent * event );
	virtual void closeEvent(QCloseEvent *event);

	void setParameter(const std::string & key, const std::string & value);

	virtual QString getParamMessage();

	virtual void readSettings(const QString & filePath = QString());
	virtual void readGuiSettings(const QString & filePath = QString());
	virtual void readCameraSettings(const QString & filePath = QString());
	virtual bool readCoreSettings(const QString & filePath = QString());

	virtual void writeSettings(const QString & filePath = QString());
	virtual void writeGuiSettings(const QString & filePath = QString());
	virtual void writeCameraSettings(const QString & filePath = QString());
	virtual void writeCoreSettings(const QString & filePath = QString());

private:
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
	void addParameters(const QStackedWidget * stackedWidget);
	void addParameters(const QGroupBox * box);
	QList<QGroupBox*> getGroupBoxes();
	void readSettingsBegin();
	void testOdometry(int type);

protected:
	rtabmap::ParametersMap _parameters;
	PANEL_FLAGS _obsoletePanels;

private:
	Ui_preferencesDialog * _ui;
	QStandardItemModel * _indexModel;
	bool _initialized;
	bool _monitoringState;

	QProgressDialog * _progressDialog;

	//Odometry test
	CameraThread * _cameraThread;
	OdometryThread * _odomThread;

	QVector<QCheckBox*> _3dRenderingShowClouds;
	QVector<QDoubleSpinBox*> _3dRenderingVoxelSize;
	QVector<QSpinBox*> _3dRenderingDecimation;
	QVector<QDoubleSpinBox*> _3dRenderingMaxDepth;
	QVector<QDoubleSpinBox*> _3dRenderingOpacity;
	QVector<QSpinBox*> _3dRenderingPtSize;
	QVector<QCheckBox*> _3dRenderingShowScans;
	QVector<QDoubleSpinBox*> _3dRenderingOpacityScan;
	QVector<QSpinBox*> _3dRenderingPtSizeScan;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(PreferencesDialog::PANEL_FLAGS)

}

#endif /* PREFERENCESDIALOG_H_ */
