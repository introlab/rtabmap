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

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <QtGui/QDialog>
#include <QtCore/QModelIndex>

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

namespace rtabmap {

class PlotCurve;

class RTABMAP_EXP PreferencesDialog : public QDialog
{
	Q_OBJECT

public:
	enum PanelFlag {
		kPanelDummy = 0,
		kPanelGeneralStrategy = 1,
		kPanelGeneral = 2,
		kPanelFourier = 4,
		kPanelSurf = 8,
		kPanelSource = 16,
		kPanelAll = 31
	};
	// TODO, tried to change the name of PANEL_FLAGS to PanelFlags... but signals/slots errors appeared...
	Q_DECLARE_FLAGS(PANEL_FLAGS, PanelFlag);

	enum Src {
		kSrcUndef,
		kSrcUsbDevice,
		kSrcImages,
		kSrcVideo,
		kSrcDatabase
	};

public:
	static QString getIniFilePath();

public:
	PreferencesDialog(QWidget * parent = 0);
	virtual ~PreferencesDialog();

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
	QString getWorkingDirectory();

	// source panel
	double getGeneralImageRate() const;
	bool getGeneralAutoRestart() const;
	bool getGeneralCameraKeypoints() const;
	int getSourceType() const;
	QString getSourceTypeStr() const;
	int getSourceWidth() const;
	int getSourceHeight() const;
	QString getSourceImagesPath() const;	//Images group
	QString getSourceImagesSuffix() const;	//Images group
	int getSourceImagesSuffixIndex() const;	//Images group
	int getSourceImagesStartPos() const;	//Images group
	bool getSourceImagesRefreshDir() const;	//Images group
	QString getSourceVideoPath() const;	//Video group
	int getSourceUsbDeviceId() const;		//UsbDevice group
	QString getSourceDatabasePath() const; 			//Database group
	bool getSourceDatabaseIgnoreChildren() const;	//Database group
	bool getSourceDatabaseLoadActions() const;	//Database group

	//
	bool isImagesKept() const;
	float getTimeLimit() const;

	//specific
	double getLoopThr() const;
	double getRetrievalThr() const;
	double getVpThr() const;
	double getExpThr() const;

signals:
	void settingsChanged(PreferencesDialog::PANEL_FLAGS);
	void settingsChanged(rtabmap::ParametersMap);

public slots:
	void setHardThr(int value);
	void setRetrievalThr(int value);
	void setImgRate(double value);
	void setAutoRestart(bool value);
	void setTimeLimit(float value);
	void selectSource(Src src = kSrcUndef);

private slots:
	void closeDialog ( QAbstractButton * button );
	void resetApply ( QAbstractButton * button );
	void resetSettings(int panelNumber);
	void loadConfigFrom();
	void saveConfigTo();
	void makeObsoleteGeneralPanel();
	void makeObsoleteSourcePanel();
	void clicked(const QModelIndex &index);
	void addParameter(int value);
	void addParameter(double value);
	void addParameter(const QString & value);
	void updatePredictionPlot();
	void updateKpROI();
	void changeWorkingDirectory();
	void changeDictionaryPath();
	void readSettingsEnd();
	void setupTreeView();

protected:
	virtual void showEvent ( QShowEvent * event );

	void setParameter(const std::string & key, const std::string & value);

	virtual QString getParamMessage();

	virtual void readSettings(const QString & filePath = QString());
	virtual void readGuiSettings(const QString & filePath = QString());
	virtual void readCameraSettings(const QString & filePath = QString());
	virtual void readCoreSettings(const QString & filePath = QString());

	virtual void writeSettings(const QString & filePath = QString());
	virtual void writeGuiSettings(const QString & filePath = QString());
	virtual void writeCameraSettings(const QString & filePath = QString());
	virtual void writeCoreSettings(const QString & filePath = QString());

private:
	bool validateForm();
	void setupSignals();
	void setupKpRoiPanel();
	bool parseModel(QList<QGroupBox*> & boxes, QStandardItem * parentItem, int currentLevel, int & absoluteIndex);
	void addParameter(const QObject * object, int value);
	void addParameter(const QObject * object, double value);
	void addParameter(const QObject * object, const QString & value);
	void addParameters(const QGroupBox * box);
	QList<QGroupBox*> getGroupBoxes();
	void readSettingsBegin();

protected:
	rtabmap::ParametersMap _parameters;
	PANEL_FLAGS _obsoletePanels;

private:
	Ui_preferencesDialog * _ui;
	QStandardItemModel * _indexModel;
	bool _initialized;

	QProgressDialog * _progressDialog;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(PreferencesDialog::PANEL_FLAGS)

}

#endif /* PREFERENCESDIALOG_H_ */
