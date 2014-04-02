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

class CameraOpenni;
class OdometryThread;
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
		kSrcVideo
	};

	enum OdomType {
		kOdomBIN,
		kOdomBOW,
		kOdomICP
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

	bool isCloudMeshing(int index) const;     // 0=map
	bool isCloudsShown(int index) const;      // 0=map, 1=odom, 2=save
	double getCloudVoxelSize(int index) const; // 0=map, 1=odom, 2=save
	int getCloudDecimation(int index) const;   // 0=map, 1=odom, 2=save
	double getCloudMaxDepth(int index) const;  // 0=map, 1=odom, 2=save
	double getCloudOpacity(int index) const;   // 0=map, 1=odom, 2=save
	int getCloudPointSize(int index) const;    // 0=map, 1=odom, 2=save

	bool isScansShown(int index) const;       // 0=map, 1=odom, 2=save
	double getScanOpacity(int index) const;    // 0=map, 1=odom, 2=save
	int getScanPointSize(int index) const;     // 0=map, 1=odom, 2=save

	bool isCloudFiltering() const;
	double getCloudFilteringRadius() const;
	double getCloudFilteringAngle() const;

	QString getWorkingDirectory() const;
	QString getDatabasePath() const;

	// source panel
	double getGeneralInputRate() const;
	bool isSourceImageUsed() const;
	bool isSourceDatabaseUsed() const;
	bool isSourceOpenniUsed() const;
	OdomType getOdometryType() const;
	bool getGeneralAutoRestart() const;
	bool getGeneralCameraKeypoints() const;
	int getSourceImageType() const;
	QString getSourceImageTypeStr() const;
	int getSourceWidth() const;
	int getSourceHeight() const;
	int getFramesDropped() const;
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
	QString getSourceOpenniDevice() const;            //Openni group
	Transform getSourceOpenniLocalTransform() const;    //Openni group

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
	double getExpThr() const;

	//
	void setMonitoringState(bool monitoringState) {_monitoringState = monitoringState;}

signals:
	void settingsChanged(PreferencesDialog::PANEL_FLAGS);
	void settingsChanged(rtabmap::ParametersMap);

public slots:
	void setInputRate(double value);
	void setDetectionRate(double value);
	void setHardThr(int value);
	void setAutoRestart(bool value);
	void setTimeLimit(float value);
	void setSLAMMode(bool enabled);
	void selectSourceImage(Src src = kSrcUndef);
	void selectSourceDatabase(bool user = false);
	void selectSourceOpenni(bool user = false);

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
	void changeDatabasePath();
	void changeWorkingDirectory();
	void changeDictionaryPath();
	void readSettingsEnd();
	void setupTreeView();
	void updateBasicParameter();
	void openDatabaseViewer();
	void cleanOdometryTest();
	void testOdometry();

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
	void testOdometry(OdomType type);

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
	CameraOpenni * _odomCamera;
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
	QVector<QCheckBox*> _3dRenderingMeshing;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(PreferencesDialog::PANEL_FLAGS)

}

#endif /* PREFERENCESDIALOG_H_ */
