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

#include "rtabmap/gui/PreferencesDialog.h"
#include <QtCore/QSettings>
#include <QtCore/QDir>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <QtGui/QStandardItemModel>
#include <QtGui/QMainWindow>
#include <QtCore/QTimer>
#include <QtGui/QProgressDialog>
#include "utilite/ULogger.h"
#include "ui_preferencesDialog.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Parameters.h"
#include "utilite/UConversion.h"
#include "Plot.h"
#include "utilite/UStl.h"

#define DEFAULT_GUI_IMAGES_KEPT true
#define DEFAULT_LOGGER_LEVEL 2
#define DEFAULT_LOGGER_EVENT_LEVEL 3
#define DEFAULT_LOGGER_PAUSE_LEVEL 4
#define DEFAULT_LOGGER_PRINT_TIME true

using namespace rtabmap;

namespace rtabmap {

PreferencesDialog::PreferencesDialog(QWidget * parent) :
	QDialog(parent),
	_obsoletePanels(kPanelDummy),
	_ui(0),
	_indexModel(0),
	_initialized(false),
	_predictionPanelInitialized(false)
{
	ULOGGER_DEBUG("");

	_ui = new Ui_preferencesDialog();
	_ui->setupUi(this);

	_ui->predictionPlot->showLegend(false);

	// Connect
	connect(_ui->buttonBox_global, SIGNAL(clicked(QAbstractButton *)), this, SLOT(closeDialog(QAbstractButton *)));
	connect(_ui->buttonBox_local, SIGNAL(clicked(QAbstractButton *)), this, SLOT(resetApply(QAbstractButton *)));
	connect(_ui->pushButton_loadConfig, SIGNAL(clicked()), this, SLOT(loadConfigFrom()));
	connect(_ui->pushButton_saveConfig, SIGNAL(clicked()), this, SLOT(saveConfigTo()));

	// General panel
	connect(_ui->general_checkBox_imagesKept, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->comboBox_loggerLevel, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->comboBox_loggerEventLevel, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->comboBox_loggerPauseLevel, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_logger_printTime, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_verticalLayoutUsed, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_imageFlipped, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->comboBox_loggerType, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_imageRejectedShown, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_beep, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->horizontalSlider_keypointsOpacity, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));

	//Source panel
	_ui->stackedWidget_2->setCurrentIndex(_ui->source_comboBox_type->currentIndex());
	connect(_ui->source_comboBox_type, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_2, SLOT(setCurrentIndex(int)));
	connect(_ui->source_comboBox_type, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->general_doubleSpinBox_imgRate, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->general_checkBox_autoRestart, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//usbDevice group
	connect(_ui->source_usbDevice_spinBox_id, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_imgWidth, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_imgheight, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//images group
	connect(_ui->source_images_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSource()));
	connect(_ui->source_images_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_images_spinBox_startPos, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_images_refreshDir, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//video group
	connect(_ui->source_video_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSource()));
	connect(_ui->source_video_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	//database group
	connect(_ui->source_database_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSource()));
	connect(_ui->source_database_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_database_checkBox_ignoreChildren, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));


	// Map objects name with the corresponding parameter key, needed for the addParameter() slots
	//Rtabmap
	_ui->general_doubleSpinBox_retrievalThr->setObjectName(Parameters::kRtabmapRetrievalThr().c_str());
	_ui->general_checkBox_publishStats->setObjectName(Parameters::kRtabmapPublishStats().c_str());
	_ui->general_doubleSpinBox_timeThr->setObjectName(Parameters::kRtabmapTimeThr().c_str());
	_ui->general_spinBox_imagesBufferSize->setObjectName(Parameters::kRtabmapSMStateBufferSize().c_str());
	_ui->general_spinBox_minMemorySizeForLoopDetection->setObjectName(Parameters::kRtabmapMinMemorySizeForLoopDetection().c_str());
	_ui->general_spinBox_maxRetrieved->setObjectName(Parameters::kRtabmapMaxRetrieved().c_str());
	_ui->general_checkBox_actionsByTime->setObjectName(Parameters::kRtabmapActionsByTime().c_str());
	_ui->general_checkBox_actionsSentRejectHyp->setObjectName(Parameters::kRtabmapActionsSentRejectHyp().c_str());
	_ui->general_doubleSpinBox_confidenceThr->setObjectName(Parameters::kRtabmapConfidenceThr().c_str());
	_ui->lineEdit_workingDirectory->setObjectName(Parameters::kRtabmapWorkingDirectory().c_str());
	connect(_ui->toolButton_workingDirectory, SIGNAL(clicked()), this, SLOT(changeWorkingDirectory()));

	// Memory
	_ui->general_checkBox_keepRawData->setObjectName(Parameters::kMemRawDataKept().c_str());
	_ui->general_spinBox_maxStMemSize->setObjectName(Parameters::kMemMaxStMemSize().c_str());
	_ui->general_checkBox_similarityOnlyLast->setObjectName(Parameters::kMemSimilarityOnlyLast().c_str());
	_ui->general_doubleSpinBox_similarityThr->setObjectName(Parameters::kMemSimilarityThr().c_str());
	_ui->general_checkBox_incrementalMemory->setObjectName(Parameters::kMemIncrementalMemory().c_str());
	_ui->general_checkBox_commonSignatureUsed->setObjectName(Parameters::kMemCommonSignatureUsed().c_str());
	_ui->general_checkBox_databaseCleaned->setObjectName(Parameters::kMemDatabaseCleaned().c_str());
	_ui->mem_spinBox_delayRequired->setObjectName(Parameters::kMemDelayRequired().c_str());
	_ui->general_checkBox_localGraphCleaned->setObjectName(Parameters::kRtabmapLocalGraphCleaned().c_str());
	_ui->general_doubleSpinBox_recentWmRatio->setObjectName(Parameters::kMemRecentWmRatio().c_str());

	// Database
	_ui->spinBox_dbMinSignToSave->setObjectName(Parameters::kDbMinSignaturesToSave().c_str());
	_ui->spinBox_dbMinWordsToSave->setObjectName(Parameters::kDbMinWordsToSave().c_str());
	_ui->checkBox_dbInMemory->setObjectName(Parameters::kDbSqlite3InMemory().c_str());
	_ui->spinBox_dbCacheSize->setObjectName(Parameters::kDbSqlite3CacheSize().c_str());
	_ui->comboBox_dbJournalMode->setObjectName(Parameters::kDbSqlite3JournalMode().c_str());

	// Create hypotheses
	_ui->general_doubleSpinBox_hardThr->setObjectName(Parameters::kRtabmapLoopThr().c_str());
	_ui->general_doubleSpinBox_loopRatio->setObjectName(Parameters::kRtabmapLoopRatio().c_str());

	//Bayes
	_ui->general_doubleSpinBox_vp->setObjectName(Parameters::kBayesVirtualPlacePriorThr().c_str());
	_ui->lineEdit_bayes_predictionLC->setObjectName(Parameters::kBayesPredictionLC().c_str());

	//Keypoint-based
	_ui->comboBox_dictionary_strategy->setObjectName(Parameters::kKpNNStrategy().c_str());
	_ui->checkBox_dictionary_incremental->setObjectName(Parameters::kKpIncrementalDictionary().c_str());
	_ui->comboBox_detector_strategy->setObjectName(Parameters::kKpDetectorStrategy().c_str());
	_ui->comboBox_descriptor_strategy->setObjectName(Parameters::kKpDescriptorStrategy().c_str());
	_ui->checkBox_dictionary_minDistUsed->setObjectName(Parameters::kKpMinDistUsed().c_str());
	_ui->surf_doubleSpinBox_matchThr->setObjectName(Parameters::kKpMinDist().c_str());
	_ui->checkBox_dictionary_nndrUsed->setObjectName(Parameters::kKpNndrUsed().c_str());
	_ui->surf_doubleSpinBox_nndrRatio->setObjectName(Parameters::kKpNndrRatio().c_str());
	_ui->surf_spinBox_maxLeafs->setObjectName(Parameters::kKpMaxLeafs().c_str());
	_ui->surf_spinBox_wordsPerImageTarget->setObjectName(Parameters::kKpWordsPerImage().c_str());
	_ui->surf_doubleSpinBox_ratioBadSign->setObjectName(Parameters::kKpBadSignRatio().c_str());
	_ui->checkBox_kp_usingAdaptiveResponseThr->setObjectName(Parameters::kKpUsingAdaptiveResponseThr().c_str());
	_ui->general_checkBox_reactivatedWordsComparedToNewWords->setObjectName(Parameters::kKpReactivatedWordsComparedToNewWords().c_str());
	_ui->checkBox_kp_tfIdfLikelihoodUsed->setObjectName(Parameters::kKpTfIdfLikelihoodUsed().c_str());
	_ui->checkBox_kp_tfIdfNormalized->setObjectName(Parameters::kKpTfIdfNormalized().c_str());
	_ui->checkBox_kp_parallelized->setObjectName(Parameters::kKpParallelized().c_str());
	_ui->checkBox_kp_sensorStateOnly->setObjectName(Parameters::kKpSensorStateOnly().c_str());
	_ui->lineEdit_kp_roi->setObjectName(Parameters::kKpRoiRatios().c_str());
	_ui->lineEdit_dictionaryPath->setObjectName(Parameters::kKpDictionaryPath().c_str());
	connect(_ui->toolButton_dictionaryPath, SIGNAL(clicked()), this, SLOT(changeDictionaryPath()));

	//SURF detector
	_ui->surf_doubleSpinBox_hessianThr->setObjectName(Parameters::kSURFHessianThreshold().c_str());
	_ui->surf_spinBox_octaves->setObjectName(Parameters::kSURFOctaves().c_str());
	_ui->surf_spinBox_octaveLayers->setObjectName(Parameters::kSURFOctaveLayers().c_str());
	_ui->checkBox_surfExtended->setObjectName(Parameters::kSURFExtended().c_str());
	_ui->surf_checkBox_gpuVersion->setObjectName(Parameters::kSURFGpuVersion().c_str());
	_ui->surf_checkBox_upright->setObjectName(Parameters::kSURFUpright().c_str());

	//Star detector
	_ui->star_spinBox_maxSize->setObjectName(Parameters::kStarMaxSize().c_str());
	_ui->star_spinBox_responseThr->setObjectName(Parameters::kStarResponseThreshold().c_str());
	_ui->star_spinBox_lineThrProj->setObjectName(Parameters::kStarLineThresholdProjected().c_str());
	_ui->star_spinBox_lineThrBin->setObjectName(Parameters::kStarLineThresholdBinarized().c_str());
	_ui->star_spinBox_suppressNonmaxSize->setObjectName(Parameters::kStarSuppressNonmaxSize().c_str());

	//SIFT detector
	_ui->sift_doubleSpinBox_Thr->setObjectName(Parameters::kSIFTThreshold().c_str());
	_ui->sift_doubleSpinBox_edgeThr->setObjectName(Parameters::kSIFTEdgeThreshold().c_str());

	// verifyHypotheses
	_ui->comboBox_vh_strategy->setObjectName(Parameters::kRtabmapVhStrategy().c_str());
	_ui->vh_doubleSpinBox_similarity->setObjectName(Parameters::kVhSimilarity().c_str());
	_ui->surf_spinBox_matchCountMinAccepted->setObjectName(Parameters::kVhEpMatchCountMin().c_str());
	_ui->surf_doubleSpinBox_ransacParam1->setObjectName(Parameters::kVhEpRansacParam1().c_str());
	_ui->surf_doubleSpinBox_ransacParam2->setObjectName(Parameters::kVhEpRansacParam2().c_str());

	setupSignals();
	// custom signals
	connect(_ui->doubleSpinBox_kp_roi0, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));
	connect(_ui->doubleSpinBox_kp_roi1, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));
	connect(_ui->doubleSpinBox_kp_roi2, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));
	connect(_ui->doubleSpinBox_kp_roi3, SIGNAL(valueChanged(double)), this, SLOT(updateKpROI()));

	//Create a model from the stacked widgets
	// This will add all parameters to the parameters Map
	_ui->stackedWidget->setCurrentIndex(0);
	this->setupTreeView();
	connect(_ui->treeView, SIGNAL(clicked(QModelIndex)), this, SLOT(clicked(QModelIndex)));
	_ui->treeView->expandToDepth(1);

	_obsoletePanels = kPanelAll;

	_progressDialog = new QProgressDialog(this);
	_progressDialog->setWindowTitle(tr("Read parameters..."));
	_progressDialog->setMaximum(2);
}

PreferencesDialog::~PreferencesDialog() {
	this->saveWindowGeometry("PreferencesDialog", this);
	delete _ui;
}

void PreferencesDialog::init()
{
	this->readSettings();
	this->writeSettings();// This will create the ini file if not exist

	this->loadWindowGeometry("PreferencesDialog", this);

	_obsoletePanels = kPanelAll;
	_initialized = true;
}

void PreferencesDialog::setupTreeView()
{
	QFile modelFile(":/resources/PreferencesModel.txt");
	if(modelFile.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		_indexModel = new QStandardItemModel(this);
		// Parse the model
		QList<QGroupBox*> boxes = this->getGroupBoxes();
		QStandardItem * parentItem = _indexModel->invisibleRootItem();
		int index = 0;
		this->parseModel(boxes, parentItem, 0, index); // recursive method
		if(index != _ui->stackedWidget->count())
		{
			ULOGGER_ERROR("The tree model is not the same size of the stacked widgets...%d vs %d stacks", index, _ui->stackedWidget->count());
		}
		_ui->treeView->setModel(_indexModel);
	}
	else
	{
		ULOGGER_ERROR("Can't open resource file \"PreferencesModel.txt\"");
	}
}

// recursive...
bool PreferencesDialog::parseModel(QList<QGroupBox*> & boxes, QStandardItem * parentItem, int currentLevel, int & absoluteIndex)
{
	if(parentItem == 0)
	{
		ULOGGER_ERROR("Parent item is null !");
		return false;
	}

	QStandardItem * currentItem = 0;
	while(absoluteIndex < boxes.size())
	{
		QString title = boxes.at(absoluteIndex)->title();
		bool ok = false;
		int lvl = QString(title.at(0)).toInt(&ok);
		if(!ok)
		{
			ULOGGER_ERROR("Error while parsing the first number of the QGroupBox title, the first character must be the number in the hierarchy");
			return false;
		}


		if(lvl == currentLevel)
		{
			title.remove(0, 1);
			boxes.at(absoluteIndex)->setTitle(title);
			QStandardItem * item = new QStandardItem(title);
			item->setData(absoluteIndex);
			currentItem = item;
			//ULOGGER_DEBUG("PreferencesDialog::parseModel() lvl(%d) Added %s", currentLevel, title.toStdString().c_str());
			parentItem->appendRow(item);
			++absoluteIndex;
		}
		else if(lvl > currentLevel)
		{
			if(lvl>currentLevel+1)
			{
				ULOGGER_ERROR("Intermediary lvl doesn't exist, lvl %d to %d, indexes %d and %d", currentLevel, lvl, absoluteIndex-1, absoluteIndex);
				return false;
			}
			else
			{
				parseModel(boxes, currentItem, currentLevel+1, absoluteIndex); // recursive
			}
		}
		else
		{
			return false;
		}
	}
	return true;
}

void PreferencesDialog::setupSignals()
{
	const rtabmap::ParametersMap & parameters = Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		QObject * obj = _ui->stackedWidget->findChild<QObject*>((*iter).first.c_str());
		if(obj)
		{
			QSpinBox * spin = qobject_cast<QSpinBox *>(obj);
			QDoubleSpinBox * doubleSpin = qobject_cast<QDoubleSpinBox *>(obj);
			QComboBox * combo = qobject_cast<QComboBox *>(obj);
			QCheckBox * check = qobject_cast<QCheckBox *>(obj);
			QLineEdit * lineEdit = qobject_cast<QLineEdit *>(obj);
			if(spin)
			{
				connect(spin, SIGNAL(valueChanged(int)), this, SLOT(addParameter(int)));
			}
			else if(doubleSpin)
			{
				connect(doubleSpin, SIGNAL(valueChanged(double)), this, SLOT(addParameter(double)));
			}
			else if(combo)
			{
				connect(combo, SIGNAL(currentIndexChanged(int)), this, SLOT(addParameter(int)));
			}
			else if(check)
			{
				connect(check, SIGNAL(stateChanged(int)), this, SLOT(addParameter(int)));
			}
			else if(lineEdit)
			{
				connect(lineEdit, SIGNAL(textChanged(const QString &)), this, SLOT(addParameter(const QString &)));
			}
			else
			{
				ULOGGER_WARN("QObject called %s can't be cast to a supported widget", (*iter).first.c_str());
			}
		}
		else
		{
			ULOGGER_WARN("Can't find the related QObject for parameter %s", (*iter).first.c_str());
		}
	}
}

void PreferencesDialog::clicked(const QModelIndex &index)
 {
	QStandardItem * item = _indexModel->itemFromIndex(index);
	if(item)
	{
		_ui->stackedWidget->setCurrentIndex(item->data().toInt());
	}
 }

void PreferencesDialog::closeDialog ( QAbstractButton * button )
{
	QDialogButtonBox::ButtonRole role = _ui->buttonBox_global->buttonRole(button);
	switch(role)
	{
	case QDialogButtonBox::RejectRole:
		_parameters.clear();
		_obsoletePanels = kPanelDummy;
		this->reject();
		break;

	case QDialogButtonBox::AcceptRole:
		if(_obsoletePanels & kPanelAll || _parameters.size())
		{
			if(validateForm())
			{
				writeSettings();
				this->accept();
			}
		}
		else
		{
			this->accept();
		}
		break;

	default:
		break;
	}
}

void PreferencesDialog::resetApply ( QAbstractButton * button )
{
	QDialogButtonBox::ButtonRole role = _ui->buttonBox_local->buttonRole(button);
	switch(role)
	{
	case QDialogButtonBox::ApplyRole:
		if(validateForm())
		{
			writeSettings();
		}
		break;

	case QDialogButtonBox::ResetRole:
		resetSettings(_ui->stackedWidget->currentIndex());
		break;

	default:
		break;
	}
}

void PreferencesDialog::resetSettings(int panelNumber)
{
	QList<QGroupBox*> boxes = this->getGroupBoxes();
	if(panelNumber >= 0 && panelNumber < boxes.size())
	{
		if(boxes.at(panelNumber)->objectName() == "groupBox_generalSettingsGui")
		{
			_ui->general_checkBox_imagesKept->setChecked(DEFAULT_GUI_IMAGES_KEPT);
			_ui->comboBox_loggerLevel->setCurrentIndex(DEFAULT_LOGGER_LEVEL);
			_ui->comboBox_loggerEventLevel->setCurrentIndex(DEFAULT_LOGGER_EVENT_LEVEL);
			_ui->comboBox_loggerPauseLevel->setCurrentIndex(DEFAULT_LOGGER_PAUSE_LEVEL);
			_ui->checkBox_logger_printTime->setChecked(DEFAULT_LOGGER_PRINT_TIME);
		}
		else if(boxes.at(panelNumber)->objectName() == "groupBox_source")
		{
			_ui->general_doubleSpinBox_imgRate->setValue(1.0);
			_ui->source_spinBox_imgWidth->setValue(0);
			_ui->source_spinBox_imgheight->setValue(0);
			_ui->general_checkBox_autoRestart->setChecked(false);
		}
		else
		{
			QObjectList children = boxes.at(panelNumber)->children();
			rtabmap::ParametersMap defaults = Parameters::getDefaultParameters();
			std::string key;
			for(int i=0; i<children.size(); ++i)
			{
				key = children.at(i)->objectName().toStdString();
				if(uContains(defaults, key))
				{
					this->setParameter(key, defaults.at(key));
				}
			}

			if(boxes.at(panelNumber)->findChild<QLineEdit*>(_ui->lineEdit_bayes_predictionLC->objectName()))
			{
				this->setupPredictionPanel();
			}

			if(boxes.at(panelNumber)->findChild<QLineEdit*>(_ui->lineEdit_kp_roi->objectName()))
			{
				this->setupKpRoiPanel();
			}
		}
	}
	else
	{
		ULOGGER_WARN("panel number and the number of stacked widget doesn't match");
	}

}

QString PreferencesDialog::getWorkingDirectory()
{
	return _ui->lineEdit_workingDirectory->text();
}

QString PreferencesDialog::getIniFilePath()
{
	return Rtabmap::getIniFilePath().c_str();
}

void PreferencesDialog::loadConfigFrom()
{
	QString path = QFileDialog::getOpenFileName(this, tr("Load configurations..."), this->getWorkingDirectory(), "*.ini");
	if(!path.isEmpty())
	{
		this->readSettings(path);
	}
}

void PreferencesDialog::readSettings(const QString & filePath)
{
	ULOGGER_DEBUG("");
	readGuiSettings(filePath);
	readCameraSettings(filePath);
	readCoreSettings(filePath);
}

void PreferencesDialog::readGuiSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup("General");
	_ui->general_checkBox_imagesKept->setChecked(settings.value("imagesKept", _ui->general_checkBox_imagesKept->isChecked()).toBool());
	_ui->comboBox_loggerLevel->setCurrentIndex(settings.value("loggerLevel", _ui->comboBox_loggerLevel->currentIndex()).toInt());
	_ui->comboBox_loggerEventLevel->setCurrentIndex(settings.value("loggerEventLevel", _ui->comboBox_loggerEventLevel->currentIndex()).toInt());
	_ui->comboBox_loggerPauseLevel->setCurrentIndex(settings.value("loggerPauseLevel", _ui->comboBox_loggerPauseLevel->currentIndex()).toInt());
	_ui->comboBox_loggerType->setCurrentIndex(settings.value("loggerType", _ui->comboBox_loggerType->currentIndex()).toInt());
	_ui->checkBox_logger_printTime->setChecked(settings.value("loggerPrintTime", _ui->checkBox_logger_printTime->isChecked()).toBool());
	_ui->checkBox_verticalLayoutUsed->setChecked(settings.value("verticalLayoutUsed", _ui->checkBox_verticalLayoutUsed->isChecked()).toBool());
	_ui->checkBox_imageFlipped->setChecked(settings.value("imageFlipped", _ui->checkBox_imageFlipped->isChecked()).toBool());
	_ui->checkBox_imageRejectedShown->setChecked(settings.value("imageRejectedShown", _ui->checkBox_imageRejectedShown->isChecked()).toBool());
	_ui->checkBox_imageHighestHypShown->setChecked(settings.value("imageHighestHypShown", _ui->checkBox_imageHighestHypShown->isChecked()).toBool());
	_ui->checkBox_beep->setChecked(settings.value("beep", _ui->checkBox_beep->isChecked()).toBool());
	_ui->horizontalSlider_keypointsOpacity->setValue(settings.value("keypointsOpacity", _ui->horizontalSlider_keypointsOpacity->value()).toInt());

	settings.endGroup(); // General

	settings.endGroup(); // rtabmap
}

void PreferencesDialog::readCameraSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);

	settings.beginGroup("Camera");
	_ui->general_doubleSpinBox_imgRate->setValue(settings.value("imgRate", _ui->general_doubleSpinBox_imgRate->value()).toDouble());
	_ui->general_checkBox_autoRestart->setChecked(settings.value("autoRestart", _ui->general_checkBox_autoRestart->isChecked()).toBool());
	_ui->source_comboBox_type->setCurrentIndex(settings.value("type", _ui->source_comboBox_type->currentIndex()).toInt());
	_ui->source_spinBox_imgWidth->setValue(settings.value("imgWidth",_ui->source_spinBox_imgWidth->value()).toInt());
	_ui->source_spinBox_imgheight->setValue(settings.value("imgHeight",_ui->source_spinBox_imgheight->value()).toInt());
	//usbDevice group
	settings.beginGroup("usbDevice");
	_ui->source_usbDevice_spinBox_id->setValue(settings.value("id",_ui->source_usbDevice_spinBox_id->value()).toInt());
	settings.endGroup(); // usbDevice
	//images group
	settings.beginGroup("images");
	_ui->source_images_lineEdit_path->setText(settings.value("path", _ui->source_images_lineEdit_path->text()).toString());
	_ui->source_images_spinBox_startPos->setValue(settings.value("startPos",_ui->source_images_spinBox_startPos->value()).toInt());
	_ui->source_images_refreshDir->setChecked(settings.value("refreshDir",_ui->source_images_refreshDir->isChecked()).toBool());
	settings.endGroup(); // images
	//video group
	settings.beginGroup("video");
	_ui->source_video_lineEdit_path->setText(settings.value("path", _ui->source_video_lineEdit_path->text()).toString());
	settings.endGroup(); // video
	//database group
	settings.beginGroup("database");
	_ui->source_database_lineEdit_path->setText(settings.value("path",_ui->source_database_lineEdit_path->text()).toString());
	_ui->source_database_checkBox_ignoreChildren->setChecked(settings.value("ignoreChildren",_ui->source_database_checkBox_ignoreChildren->isChecked()).toBool());
	settings.endGroup(); // usbDevice
	settings.endGroup(); // Camera
}

void PreferencesDialog::readCoreSettings(const QString & filePath)
{
	UDEBUG("");
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);

	const rtabmap::ParametersMap & parameters = Parameters::getDefaultParameters();
	settings.beginGroup("Core");
	QStringList keys = settings.allKeys();
	for(rtabmap::ParametersMap::const_iterator iter = parameters.begin(); iter!=parameters.end(); ++iter)
	{
		QString key((*iter).first.c_str());
		QString value = settings.value(key, "").toString();
		if(!value.isEmpty())
		{
			this->setParameter(key.toStdString(), value.toStdString());
		}
		else
		{
			UDEBUG("key.toStdString()=%s", key.toStdString().c_str());
			// Use the default value if the key doesn't exist yet
			this->setParameter(key.toStdString(), (*iter).second);

			// Add information about the working directory if not in the config file
			if(key.toStdString().compare(Parameters::kRtabmapWorkingDirectory()) == 0)
			{
				if(!_initialized)
				{
					QMessageBox::information(this,
							tr("Working directory"),
							tr("RTAB-Map needs a working directory to put the database.\n\n"
							   "By default, the directory \"%1\" is used.\n\n"
							   "The working directory can be changed any time in the "
							   "preferences menu.").arg(
									   Parameters::defaultRtabmapWorkingDirectory().c_str()));
				}
			}
		}
	}
	settings.endGroup(); // Core
}

void PreferencesDialog::saveConfigTo()
{
	QString path = QFileDialog::getSaveFileName(this, tr("Save configurations..."), this->getWorkingDirectory()+"/config.ini", "*.ini");
	if(!path.isEmpty())
	{
		this->writeSettings(path);
	}
}

void PreferencesDialog::writeSettings(const QString & filePath)
{
	writeGuiSettings(filePath);
	writeCameraSettings(filePath);
	writeCoreSettings(filePath);

	if(_parameters.size())
	{
		emit settingsChanged(_parameters);
	}

	if(_obsoletePanels)
	{
		emit settingsChanged(_obsoletePanels);
	}

	_parameters.clear();
	_obsoletePanels = kPanelDummy;
}

void PreferencesDialog::writeGuiSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);
	settings.beginGroup("Gui");

	settings.beginGroup("General");
	settings.setValue("imagesKept", _ui->general_checkBox_imagesKept->isChecked());
	settings.setValue("loggerLevel", _ui->comboBox_loggerLevel->currentIndex());
	settings.setValue("loggerEventLevel", _ui->comboBox_loggerEventLevel->currentIndex());
	settings.setValue("loggerPauseLevel", _ui->comboBox_loggerPauseLevel->currentIndex());
	settings.setValue("loggerType", _ui->comboBox_loggerType->currentIndex());
	settings.setValue("loggerPrintTime", _ui->checkBox_logger_printTime->isChecked());
	settings.setValue("verticalLayoutUsed", _ui->checkBox_verticalLayoutUsed->isChecked());
	settings.setValue("imageFlipped", _ui->checkBox_imageFlipped->isChecked());
	settings.setValue("imageRejectedShown", _ui->checkBox_imageRejectedShown->isChecked());
	settings.setValue("imageHighestHypShown", _ui->checkBox_imageHighestHypShown->isChecked());
	settings.setValue("beep", _ui->checkBox_beep->isChecked());
	settings.setValue("keypointsOpacity", _ui->horizontalSlider_keypointsOpacity->value());
	settings.endGroup(); // General

	settings.endGroup(); // rtabmap
}

void PreferencesDialog::writeCameraSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);
	settings.beginGroup("Camera");

	settings.setValue("imgRate", 		_ui->general_doubleSpinBox_imgRate->value());
	settings.setValue("autoRestart", 	_ui->general_checkBox_autoRestart->isChecked());
	settings.setValue("type", 			_ui->source_comboBox_type->currentIndex());
	settings.setValue("imgWidth", 		_ui->source_spinBox_imgWidth->value());
	settings.setValue("imgHeight", 		_ui->source_spinBox_imgheight->value());
	//usbDevice group
	settings.beginGroup("usbDevice");
	settings.setValue("id", 			_ui->source_usbDevice_spinBox_id->value());
	settings.endGroup(); //usbDevice
	//images group
	settings.beginGroup("images");
	settings.setValue("path", 			_ui->source_images_lineEdit_path->text());
	settings.setValue("startPos", 		_ui->source_images_spinBox_startPos->value());
	settings.setValue("refreshDir", 	_ui->source_images_refreshDir->isChecked());
	settings.endGroup(); //images
	//video group
	settings.beginGroup("video");
	settings.setValue("path", 			_ui->source_video_lineEdit_path->text());
	settings.endGroup(); //video
	//database group
	settings.beginGroup("database");
	settings.setValue("path", 			_ui->source_database_lineEdit_path->text());
	settings.setValue("ignoreChildren", _ui->source_database_checkBox_ignoreChildren->isChecked());
	settings.endGroup(); //usbDevice

	settings.endGroup(); // Camera
}

void PreferencesDialog::writeCoreSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}
	QSettings settings(path, QSettings::IniFormat);
	settings.beginGroup("Core");

	const rtabmap::ParametersMap & parameters = Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		QObject * obj = _ui->stackedWidget->findChild<QObject*>((*iter).first.c_str());
		if(obj)
		{
			QSpinBox * spin = qobject_cast<QSpinBox *>(obj);
			QDoubleSpinBox * doubleSpin = qobject_cast<QDoubleSpinBox *>(obj);
			QComboBox * combo = qobject_cast<QComboBox *>(obj);
			QCheckBox * check = qobject_cast<QCheckBox *>(obj);
			QLineEdit * lineEdit = qobject_cast<QLineEdit *>(obj);
			if(spin)
			{
				settings.setValue(obj->objectName(), spin->value());
			}
			else if(doubleSpin)
			{
				settings.setValue(obj->objectName(), doubleSpin->value());
			}
			else if(combo)
			{
				settings.setValue(obj->objectName(), combo->currentIndex());
			}
			else if(check)
			{
				settings.setValue(obj->objectName(), uBool2str(check->isChecked()).c_str());
			}
			else if(lineEdit)
			{
				settings.setValue(obj->objectName(), lineEdit->text());
			}
			else
			{
				ULOGGER_WARN("QObject called %s can't be cast to a supported widget", (*iter).first.c_str());
			}
		}
		else
		{
			ULOGGER_WARN("Can't find the related QObject for parameter %s", (*iter).first.c_str());
		}
	}
	settings.endGroup(); // Core
}

bool PreferencesDialog::validateForm()
{
	//TODO...
	return true;
}

QString PreferencesDialog::getParamMessage()
{
	return tr("Reading parameters from the configuration file...");
}

void PreferencesDialog::showEvent ( QShowEvent * event )
{
	this->readSettingsBegin();
}

void PreferencesDialog::readSettingsBegin()
{
	_progressDialog->setLabelText(this->getParamMessage());
	_progressDialog->show();

	// this will let the MainThread to display the progress dialog before reading the parameters...
	QTimer::singleShot(10, this, SLOT(readSettingsEnd()));
}

void PreferencesDialog::readSettingsEnd()
{
	QApplication::processEvents();

	this->readSettings();

	_progressDialog->setValue(1);
	_progressDialog->setLabelText(tr("Reading GUI settings..."));

	this->setupPredictionPanel();
	this->setupKpRoiPanel();

	_progressDialog->setValue(2); // this will make closing...
}

void PreferencesDialog::saveWindowGeometry(const QString & windowName, const QWidget * window)
{
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup(windowName);
	settings.setValue("geometry", window->saveGeometry());
	settings.endGroup(); // "windowName"
	settings.endGroup(); // rtabmap
}

void PreferencesDialog::loadWindowGeometry(const QString & windowName, QWidget * window)
{
	QByteArray bytes;
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup(windowName);
	bytes = settings.value("geometry", QByteArray()).toByteArray();
	if(!bytes.isEmpty())
	{
		window->restoreGeometry(bytes);
	}
	settings.endGroup(); // "windowName"
	settings.endGroup(); // rtabmap
}

void PreferencesDialog::saveMainWindowState(const QMainWindow * mainWindow)
{
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup("MainWindow");
	settings.setValue("state", mainWindow->saveState());
	settings.endGroup(); // "MainWindow"
	settings.endGroup(); // rtabmap

	saveWindowGeometry("MainWindow", mainWindow);
}

void PreferencesDialog::loadMainWindowState(QMainWindow * mainWindow)
{
	QByteArray bytes;
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup("MainWindow");
	bytes = settings.value("state", QByteArray()).toByteArray();
	if(!bytes.isEmpty())
	{
		mainWindow->restoreState(bytes);
	}
	settings.endGroup(); // "MainWindow"
	settings.endGroup(); // rtabmap

	loadWindowGeometry("MainWindow", mainWindow);
}

void PreferencesDialog::saveCustomConfig(const QString & section, const QString & key, const QString & value)
{
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup(section);
	settings.setValue(key, value);
	settings.endGroup(); // "section"
	settings.endGroup(); // rtabmap
}

QString PreferencesDialog::loadCustomConfig(const QString & section, const QString & key)
{
	QString value;
	QSettings settings(getIniFilePath(), QSettings::IniFormat);
	settings.beginGroup("Gui");
	settings.beginGroup(section);
	value = settings.value(key, QString()).toString();
	settings.endGroup(); // "section"
	settings.endGroup(); // rtabmap
	return value;
}

void PreferencesDialog::selectSource(Src src)
{
	ULOGGER_DEBUG("PreferencesDialog::selectSource()");

	bool fromPrefDialog = false;
	//bool modified = false;
	if(src == kSrcUndef)
	{
		fromPrefDialog = true;
		if(_ui->source_comboBox_type->currentIndex() == 1)
		{
			src = kSrcImages;
		}
		else if(_ui->source_comboBox_type->currentIndex() == 2)
		{
			src = kSrcVideo;
		}
		else if(_ui->source_comboBox_type->currentIndex() == 3)
		{
			src = kSrcDatabase;
		}
		else
		{
			src = kSrcUsbDevice;
		}
	}

	if(src == kSrcImages)
	{
		QString path = QFileDialog::getExistingDirectory(this, QString(), _ui->source_images_lineEdit_path->text());
		QDir dir(path);
		if(!path.isEmpty() && dir.exists())
		{
			QStringList filters;
			filters << "*.jpg" << "*.ppm" << "*.bmp" << "*.png" << "*.pnm";
			dir.setNameFilters(filters);
			QFileInfoList files = dir.entryInfoList();
			if(!files.empty())
			{
				_ui->source_comboBox_type->setCurrentIndex(1);
				_ui->source_images_lineEdit_path->setText(path);
				_ui->source_images_spinBox_startPos->setValue(1);
				_ui->source_images_refreshDir->setChecked(false);
			}
			else
			{
				QMessageBox::information(this,
										   tr("RTAB-Map"),
										   tr("Images must be one of these formats: ") + filters.join(" "));
			}
		}
	}
	else if(src == kSrcVideo)
	{
		QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->source_video_lineEdit_path->text(), tr("Videos (*.avi *.mpg)"));
		QFile file(path);
		if(!path.isEmpty() && file.exists())
		{
			_ui->source_comboBox_type->setCurrentIndex(2);
			_ui->source_video_lineEdit_path->setText(path);
		}
	}
	else if(src == kSrcDatabase)
	{
		QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->source_database_lineEdit_path->text(), tr("Databases (*.db)"));
		QFile file(path);
		if(!path.isEmpty() && file.exists())
		{
			_ui->source_comboBox_type->setCurrentIndex(3);
			_ui->source_database_lineEdit_path->setText(path);
		}
	}
	else
	{
		_ui->source_comboBox_type->setCurrentIndex(0);
	}

	if(!fromPrefDialog && _obsoletePanels)
	{
		if(validateForm())
		{
			this->writeSettings();
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::setParameter(const std::string & key, const std::string & value)
{
	UDEBUG("%s=%s", key.c_str(), value.c_str());
	QObject * obj = _ui->stackedWidget->findChild<QObject*>(key.c_str());
	if(obj)
	{
		QSpinBox * spin = qobject_cast<QSpinBox *>(obj);
		QDoubleSpinBox * doubleSpin = qobject_cast<QDoubleSpinBox *>(obj);
		QComboBox * combo = qobject_cast<QComboBox *>(obj);
		QCheckBox * check = qobject_cast<QCheckBox *>(obj);
		QLineEdit * lineEdit = qobject_cast<QLineEdit *>(obj);
		if(spin)
		{
			spin->setValue(QString(value.c_str()).toInt());
		}
		else if(doubleSpin)
		{
			doubleSpin->setValue(QString(value.c_str()).toDouble());
		}
		else if(combo)
		{
			combo->setCurrentIndex(QString(value.c_str()).toInt());
		}
		else if(check)
		{
			check->setChecked(uStr2Bool(value.c_str()));
		}
		else if(lineEdit)
		{
			lineEdit->setText(value.c_str());
		}
		else
		{
			ULOGGER_WARN("QObject called %s can't be cast to a supported widget", key.c_str());
		}
	}
	else
	{
		ULOGGER_WARN("Can't find the related QObject for parameter %s", key.c_str());
	}
}

void PreferencesDialog::addParameter(int value)
{
	if(sender())
	{
		this->addParameter(sender(), value);
	}
	else
	{
		ULOGGER_ERROR("This slot must be triggered by a signal, not a direct call...");
	}
}

void PreferencesDialog::addParameter(double value)
{
	if(sender())
	{
		this->addParameter(sender(), value);
	}
	else
	{
		ULOGGER_ERROR("This slot must be triggered by a signal, not a direct call...");
	}
}

void PreferencesDialog::addParameter(const QString & value)
{
	if(sender())
	{
		this->addParameter(sender(), value);
	}
	else
	{
		ULOGGER_ERROR("This slot must be triggered by a signal, not a direct call...");
	}
}

// Booleans are interpreted like integers (see QCheckBox signals)
void PreferencesDialog::addParameter(const QObject * object, int value)
{
	if(object)
	{
		const QComboBox * comboBox = qobject_cast<const QComboBox*>(object);
		if(comboBox)
		{
			// Add related panels to parameters
			if(comboBox == _ui->comboBox_vh_strategy)
			{
				if(value == 0) // 0 none
				{
					// No panel related...
				}
				else if(value == 1) // 1 similarity
				{
					this->addParameters(_ui->groupBox_vh_similarity);
				}
				else if(value == 2) // 2 epipolar
				{
					this->addParameters(_ui->groupBox_vh_epipolar);
				}
			}
			else if(comboBox == _ui->comboBox_detector_strategy)
			{
				if(value == 0) // 0 surf
				{
					this->addParameters(_ui->groupBox_detector_surf);
				}
				else if(value == 1) // 1 star
				{
					this->addParameters(_ui->groupBox_detector_star);
				}
				else if(value == 2) // 2 sift
				{
					this->addParameters(_ui->groupBox_detector_sift);
				}
			}
			else if(comboBox == _ui->comboBox_descriptor_strategy)
			{
				this->addParameters(_ui->groupBox_surfDescriptor);
			}
		}
		// Make sure the value is inserted, check if the same key already exists
		rtabmap::ParametersMap::iterator iter = _parameters.find(object->objectName().toStdString());
		if(iter != _parameters.end())
		{
			_parameters.erase(iter);
		}
		_parameters.insert(rtabmap::ParametersPair(object->objectName().toStdString(), QString::number(value).toStdString()));
		//ULOGGER_DEBUG("PreferencesDialog::addParameter(object, int) Added [\"%s\",\"%s\"]", object->objectName().toStdString().c_str(), QString::number(value).toStdString().c_str());
	}
	else
	{
		ULOGGER_ERROR("Object is null");
	}
}

void PreferencesDialog::addParameter(const QObject * object, double value)
{
	if(object)
	{
		// Make sure the value is inserted, check if the same key already exists
		rtabmap::ParametersMap::iterator iter = _parameters.find(object->objectName().toStdString());
		if(iter != _parameters.end())
		{
			_parameters.erase(iter);
		}
		_parameters.insert(rtabmap::ParametersPair(object->objectName().toStdString(), QString::number(value).toStdString()));
		//ULOGGER_DEBUG("PreferencesDialog::addParameter(object, double) Added [\"%s\",\"%s\"]", object->objectName().toStdString().c_str(), QString::number(value).toStdString().c_str());
	}
	else
	{
		ULOGGER_ERROR("Object is null");
	}
}

void PreferencesDialog::addParameter(const QObject * object, const QString & value)
{
	if(object)
	{
		// Make sure the value is inserted, check if the same key already exists
		rtabmap::ParametersMap::iterator iter = _parameters.find(object->objectName().toStdString());
		if(iter != _parameters.end())
		{
			_parameters.erase(iter);
		}
		_parameters.insert(rtabmap::ParametersPair(object->objectName().toStdString(), value.toStdString()));
		//ULOGGER_DEBUG("PreferencesDialog::addParameter(object, QString) Added [\"%s\",\"%s\"]", object->objectName().toStdString().c_str(), QString::number(value).toStdString().c_str());
	}
	else
	{
		ULOGGER_ERROR("Object is null");
	}
}

void PreferencesDialog::addParameters(const QGroupBox * box)
{
	if(box)
	{
		QObjectList children = box->children();
		//ULOGGER_DEBUG("PreferencesDialog::addParameters(QGroupBox) box %s has %d children", box->objectName().toStdString().c_str(), children.size());
		for(int i=0; i<children.size(); ++i)
		{
			const QSpinBox * spin = qobject_cast<const QSpinBox *>(children[i]);
			const QDoubleSpinBox * doubleSpin = qobject_cast<const QDoubleSpinBox *>(children[i]);
			const QComboBox * combo = qobject_cast<const QComboBox *>(children[i]);
			const QCheckBox * check = qobject_cast<const QCheckBox *>(children[i]);
			const QLineEdit * lineEdit = qobject_cast<const QLineEdit *>(children[i]);
			if(spin)
			{
				this->addParameter(spin, spin->value());
			}
			else if(doubleSpin)
			{
				this->addParameter(doubleSpin, doubleSpin->value());
			}
			else if(combo)
			{
				this->addParameter(combo, combo->currentIndex());
			}
			else if(check)
			{
				this->addParameter(check, check->isChecked());
			}
			else if(lineEdit)
			{
				this->addParameter(lineEdit, lineEdit->text());
			}
		}
	}
}

void PreferencesDialog::makeObsoleteGeneralPanel()
{
	ULOGGER_DEBUG("");
	_obsoletePanels = _obsoletePanels | kPanelGeneral;
}

void PreferencesDialog::makeObsoleteSourcePanel()
{
	ULOGGER_DEBUG("");
	_obsoletePanels = _obsoletePanels | kPanelSource;
}

rtabmap::ParametersMap PreferencesDialog::getAllParameters()
{
	rtabmap::ParametersMap result;
	rtabmap::ParametersMap tmpParameters = _parameters;
	_parameters.clear();

	QList<QGroupBox*> boxes = this->getGroupBoxes();
	for(int i=0; i<boxes.size(); ++i)
	{
		this->addParameters(boxes.at(i));
	}

	result = _parameters;
	_parameters = tmpParameters;
	return result;
}

QList<QGroupBox*> PreferencesDialog::getGroupBoxes()
{
	QList<QGroupBox*> boxes;
	for(int i=0; i<_ui->stackedWidget->count(); ++i)
	{
		QGroupBox * gb = 0;
		const QObjectList & children = _ui->stackedWidget->widget(i)->children();
		for(int j=0; j<children.size(); ++j)
		{
			if((gb = qobject_cast<QGroupBox *>(children.at(j))))
			{
				//ULOGGER_DEBUG("PreferencesDialog::setupTreeView() index(%d) Added %s, box name=%s", i, gb->title().toStdString().c_str(), gb->objectName().toStdString().c_str());
				break;
			}
		}
		if(gb)
		{
			boxes.append(gb);
			this->addParameters(gb);
		}
		else
		{
			ULOGGER_ERROR("A QGroupBox must be included in the first level of children in stacked widget, index=%d", i);
		}
	}
	return boxes;
}

void PreferencesDialog::setupPredictionPanel()
{
	QString prediction = _ui->lineEdit_bayes_predictionLC->text();
	QStringList values = prediction.split(' ');
	if(values.size() < 2)
	{
		ULOGGER_ERROR("The prediction string is not valid (prediction=\"%s\",size=%d)", prediction.toStdString().c_str(), values.size());
		return;
	}

	//Signals
	connect(_ui->spinBox_bayes_neighbors, SIGNAL(valueChanged(int)), this, SLOT(updatePredictionLCSliders()));

	//Set values
	_ui->spinBox_bayes_neighbors->setValue(values.size()-2);
	this->updatePredictionLCSliders();
	if(_predictionLCSliders.size() != values.size())
	{
		ULOGGER_ERROR("The sliders list were not updated");
		return;
	}
	for(int i=0; i<_predictionLCSliders.size(); ++i)
	{
		bool ok;
		_predictionLCSliders.at(i)->setValue(int(QString(values.at(i)).toFloat(&ok) * 100));
		if(!ok)
		{
			ULOGGER_WARN("conversion failed to float with string \"%s\"", values.at(i).toStdString().c_str());
		}
	}
	_predictionPanelInitialized = true;
	this->updatePredictionLC();
}

void PreferencesDialog::updatePredictionLCSliders()
{
	int neighborsCount = _ui->spinBox_bayes_neighbors->value();
	if(neighborsCount < 0 || neighborsCount>9)
	{
		ULOGGER_ERROR("neighborsCount not valid (=%d)", neighborsCount);
		return;
	}

	if(_predictionLCSliders.size() == 1)
	{
		_predictionLCSliders.clear(); // must be pair...
		ULOGGER_WARN("The list of sliders is supposed to be pair");
	}

	if(_predictionLCSliders.size() == 0)
	{
		_predictionLCSliders.append(_ui->verticalSlider_prediction_vp);
		_predictionLCSliders.append(_ui->verticalSlider_prediction_lp);
		connect(_ui->verticalSlider_prediction_vp, SIGNAL(valueChanged(int)), this, SLOT(updatePredictionLC()));
		connect(_ui->verticalSlider_prediction_lp, SIGNAL(valueChanged(int)), this, SLOT(updatePredictionLC()));
	}

	// If we don't have enough sliders
	while((_predictionLCSliders.size()-2) < neighborsCount)
	{
		int neighbor = _predictionLCSliders.size();
		QVBoxLayout * vLayout = 0;
		QHBoxLayout * hLayout = 0;
		QSlider * slider = 0;
		QLabel * value = 0;
		QLabel * title = 0;

		slider = new QSlider(Qt::Vertical, this);

		title = new QLabel(QString("l%1").arg(neighbor-1), slider);

		title->setAlignment(Qt::AlignCenter);
		value = new QLabel(slider);
		value->setAlignment(Qt::AlignCenter);
		//layout
		vLayout = new QVBoxLayout();
		vLayout->addWidget(title);
		hLayout = new QHBoxLayout();
		hLayout->addWidget(slider);
		vLayout->addLayout(hLayout);
		vLayout->addWidget(value);

		_ui->horizontalLayout_prior_NP->insertLayout(-1, vLayout);

		_predictionLCSliders.push_back(slider);
		connect(slider, SIGNAL(valueChanged(int)), this, SLOT(updatePredictionLC()));
		connect(slider, SIGNAL(valueChanged(int)), value, SLOT(setNum(int)));
	}

	// If we have too much sliders
	while((_predictionLCSliders.size()-2) > neighborsCount)
	{
		// delete layouts and items
		delete _ui->horizontalLayout_prior_NP->itemAt(_ui->horizontalLayout_prior_NP->count()-1)->layout()->takeAt(0)->widget(); //label
		delete _ui->horizontalLayout_prior_NP->itemAt(_ui->horizontalLayout_prior_NP->count()-1)->layout()->itemAt(0)->layout()->takeAt(0)->widget(); //slider
		delete _ui->horizontalLayout_prior_NP->itemAt(_ui->horizontalLayout_prior_NP->count()-1)->layout()->takeAt(0)->layout(); //slider hlayout
		delete _ui->horizontalLayout_prior_NP->itemAt(_ui->horizontalLayout_prior_NP->count()-1)->layout()->takeAt(0)->widget(); //spinbox
		delete _ui->horizontalLayout_prior_NP->takeAt(_ui->horizontalLayout_prior_NP->count()-1);
		// remove the last slider
		_predictionLCSliders.removeLast();
	}

	this->updatePredictionLC();
}

void PreferencesDialog::updatePredictionLC()
{
	if(!_predictionPanelInitialized)
	{
		return;
	}
	if(_predictionLCSliders.size() < 2)
	{
		ULOGGER_ERROR("Not enough slider in the list (%d)", _predictionLCSliders.size());
		return;
	}

	QString values;
	QVector<float> dataX((_predictionLCSliders.size()-2)*2 + 1);
	QVector<float> dataY((_predictionLCSliders.size()-2)*2 + 1);
	float value;
	float sum = 0;
	int lvl = 1;
	int loopClosureIndex = (dataX.size()-1)/2;
	for(int i=0; i<_predictionLCSliders.size(); ++i)
	{
		value = (float(_predictionLCSliders.at(i)->value()))/100;
		sum+=value;
		if(i==0)
		{
			//do nothing
		}
		else if(i == 1)
		{
			dataY[loopClosureIndex] = value;
			dataX[loopClosureIndex] = 0;
		}
		else
		{
			dataY[loopClosureIndex-lvl] = value;
			dataX[loopClosureIndex-lvl] = -lvl;
			dataY[loopClosureIndex+lvl] = value;
			dataX[loopClosureIndex+lvl] = lvl;
			++lvl;
			sum+=value; // double sum
		}
		values.append(QString::number(value));
		if(i+1 < _predictionLCSliders.size())
		{
			values.append(' ');
		}
	}

	_ui->label_prediction_sum->setNum(sum);
	if(sum<=0 || sum>1.001)
	{
		_ui->label_prediction_sum->setText(QString("<font color=#FF0000>") + _ui->label_prediction_sum->text() + "</font>");
		ULOGGER_WARN("The prediction is not valid (the sum must be between >0 && <=1) (sum=%f)", sum);
	}
	else if(sum == 1)
	{
		_ui->label_prediction_sum->setText(QString("<font color=#FFa500>") + _ui->label_prediction_sum->text() + "</font>");
	}
	else
	{
		_ui->label_prediction_sum->setText(QString("<font color=#000000>") + _ui->label_prediction_sum->text() + "</font>");
	}

	_ui->predictionPlot->removeCurves();
	_ui->predictionPlot->addCurve(new PlotCurve("Prediction", dataX, dataY, _ui->predictionPlot));
	_ui->lineEdit_bayes_predictionLC->setText(values);
}

void PreferencesDialog::setupKpRoiPanel()
{
	QStringList strings = _ui->lineEdit_kp_roi->text().split(' ');
	if(strings.size()!=4)
	{
		UERROR("ROI must have 4 values (%s)", _ui->lineEdit_kp_roi->text().toStdString().c_str());
		return;
	}
	_ui->doubleSpinBox_kp_roi0->setValue(strings[0].toDouble()*100.0);
	_ui->doubleSpinBox_kp_roi1->setValue(strings[1].toDouble()*100.0);
	_ui->doubleSpinBox_kp_roi2->setValue(strings[2].toDouble()*100.0);
	_ui->doubleSpinBox_kp_roi3->setValue(strings[3].toDouble()*100.0);
}

void PreferencesDialog::updateKpROI()
{
	QStringList strings;
	strings.append(QString::number(_ui->doubleSpinBox_kp_roi0->value()/100.0));
	strings.append(QString::number(_ui->doubleSpinBox_kp_roi1->value()/100.0));
	strings.append(QString::number(_ui->doubleSpinBox_kp_roi2->value()/100.0));
	strings.append(QString::number(_ui->doubleSpinBox_kp_roi3->value()/100.0));
	_ui->lineEdit_kp_roi->setText(strings.join(" "));
}

void PreferencesDialog::changeWorkingDirectory()
{
	QString directory = QFileDialog::getExistingDirectory(this, tr("Working directory"), _ui->lineEdit_workingDirectory->text());
	if(!directory.isEmpty())
	{
		ULOGGER_DEBUG("New working directory = %s", directory.toStdString().c_str());
		_ui->lineEdit_workingDirectory->setText(directory);
	}
}

void PreferencesDialog::changeDictionaryPath()
{
	QString path;
	if(_ui->lineEdit_dictionaryPath->text().isEmpty())
	{
		path = QFileDialog::getOpenFileName(this, tr("Dictionary"), this->getWorkingDirectory());
	}
	else
	{
		path = QFileDialog::getOpenFileName(this, tr("Dictionary"), _ui->lineEdit_dictionaryPath->text());
	}
	if(!path.isEmpty())
	{
		_ui->lineEdit_dictionaryPath->setText(path);
	}
}

/*** GETTERS ***/
//General
int PreferencesDialog::getGeneralLoggerLevel() const
{
	return _ui->comboBox_loggerLevel->currentIndex();
}
int PreferencesDialog::getGeneralLoggerEventLevel() const
{
	return _ui->comboBox_loggerEventLevel->currentIndex();
}
int PreferencesDialog::getGeneralLoggerPauseLevel() const
{
	return _ui->comboBox_loggerPauseLevel->currentIndex();
}
int PreferencesDialog::getGeneralLoggerType() const
{
	return _ui->comboBox_loggerType->currentIndex();
}
bool PreferencesDialog::getGeneralLoggerPrintTime() const
{
	return _ui->checkBox_logger_printTime->isChecked();
}
bool PreferencesDialog::isVerticalLayoutUsed() const
{
	return _ui->checkBox_verticalLayoutUsed->isChecked();
}
bool PreferencesDialog::isImageFlipped() const
{
	return _ui->checkBox_imageFlipped->isChecked();
}
bool PreferencesDialog::imageRejectedShown() const
{
	return _ui->checkBox_imageRejectedShown->isChecked();
}
bool PreferencesDialog::imageHighestHypShown() const
{
	return _ui->checkBox_imageHighestHypShown->isChecked();
}
bool PreferencesDialog::beepOnPause() const
{
	return _ui->checkBox_beep->isChecked();
}
int PreferencesDialog::getKeypointsOpacity() const
{
	return _ui->horizontalSlider_keypointsOpacity->value();
}

// Source
double PreferencesDialog::getGeneralImageRate() const
{
	return _ui->general_doubleSpinBox_imgRate->value();
}
bool PreferencesDialog::getGeneralAutoRestart() const
{
	return _ui->general_checkBox_autoRestart->isChecked();
}
int PreferencesDialog::getSourceType() const
{
	return _ui->source_comboBox_type->currentIndex();
}
QString PreferencesDialog::getSourceTypeStr() const
{
	return _ui->source_comboBox_type->currentText();
}
int PreferencesDialog::getSourceWidth() const
{
	return _ui->source_spinBox_imgWidth->value();
}
int PreferencesDialog::getSourceHeight() const
{
	return _ui->source_spinBox_imgheight->value();
}
QString PreferencesDialog::getSourceImagesPath() const
{
	return _ui->source_images_lineEdit_path->text();
}
int PreferencesDialog::getSourceImagesStartPos() const
{
	return _ui->source_images_spinBox_startPos->value();
}
bool PreferencesDialog::getSourceImagesRefreshDir() const
{
	return _ui->source_images_refreshDir->isChecked();
}
QString PreferencesDialog::getSourceVideoPath() const
{
	return _ui->source_video_lineEdit_path->text();
}
int PreferencesDialog::getSourceUsbDeviceId() const
{
	return _ui->source_usbDevice_spinBox_id->value();
}
QString PreferencesDialog::getSourceDatabasePath() const
{
	return _ui->source_database_lineEdit_path->text();
}
bool PreferencesDialog::getSourceDatabaseIgnoreChildren() const
{
	return _ui->source_database_checkBox_ignoreChildren->isChecked();
}

double PreferencesDialog::getLoopThr() const
{
	return _ui->general_doubleSpinBox_hardThr->value();
}
double PreferencesDialog::getRetrievalThr() const
{
	return _ui->general_doubleSpinBox_retrievalThr->value();
}
double PreferencesDialog::getVpThr() const
{
	return _ui->general_doubleSpinBox_vp->value();
}

bool PreferencesDialog::isImagesKept() const
{
	return _ui->general_checkBox_imagesKept->isChecked();
}
double PreferencesDialog::getTimeLimit() const
{
	return _ui->general_doubleSpinBox_timeThr->value();
}

/*** SETTERS ***/
void PreferencesDialog::setHardThr(int value)
{
	double dValue = double(value)/100;
	ULOGGER_DEBUG("PreferencesDialog::setHardThr(%f)", dValue);
	if(_ui->general_doubleSpinBox_hardThr->value() != dValue)
	{
		_ui->general_doubleSpinBox_hardThr->setValue(dValue);
		if(validateForm())
		{
			this->writeSettings();
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::setRetrievalThr(int value)
{
	double dValue = double(value)/100;
	ULOGGER_DEBUG("reactivation thr=%f", dValue);
	if(_ui->general_doubleSpinBox_retrievalThr->value() != dValue)
	{
		_ui->general_doubleSpinBox_retrievalThr->setValue(dValue);
		if(validateForm())
		{
			this->writeSettings();
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::setImgRate(double value)
{
	ULOGGER_DEBUG("imgRate=%2.2f", value);
	if(_ui->general_doubleSpinBox_imgRate->value() != value)
	{
		_ui->general_doubleSpinBox_imgRate->setValue(value);
		if(validateForm())
		{
			this->writeSettings();
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::setAutoRestart(bool value)
{
	ULOGGER_DEBUG("autoRestart=%d", value);
	if(_ui->general_checkBox_autoRestart->isChecked() != value)
	{
		_ui->general_checkBox_autoRestart->setChecked(value);
		if(validateForm())
		{
			this->writeSettings();
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

void PreferencesDialog::setTimeLimit(double value)
{
	ULOGGER_DEBUG("timeLimit=%fs", value);
	if(_ui->general_doubleSpinBox_timeThr->value() != value)
	{
		_ui->general_doubleSpinBox_timeThr->setValue(value);
		if(validateForm())
		{
			this->writeSettings();
		}
		else
		{
			this->readSettingsBegin();
		}
	}
}

}
