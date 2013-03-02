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
#include "rtabmap/gui/DatabaseViewer.h"
#include <QtCore/QSettings>
#include <QtCore/QDir>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <QtGui/QStandardItemModel>
#include <QtGui/QMainWindow>
#include <QtCore/QTimer>
#include <QtGui/QProgressDialog>
#include <QtGui/QScrollBar>
#include "ui_preferencesDialog.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Parameters.h"
#include <utilite/ULogger.h>
#include <utilite/UConversion.h>
#include <utilite/UPlot.h>
#include <utilite/UStl.h>

#define DEFAULT_GUI_IMAGES_KEPT true
#define DEFAULT_LOGGER_LEVEL 2
#define DEFAULT_LOGGER_EVENT_LEVEL 3
#define DEFAULT_LOGGER_PAUSE_LEVEL 4
#define DEFAULT_LOGGER_PRINT_TIME true
#define DEFAULT_LOGGER_TYPE 1

using namespace rtabmap;

namespace rtabmap {

PreferencesDialog::PreferencesDialog(QWidget * parent) :
	QDialog(parent),
	_obsoletePanels(kPanelDummy),
	_ui(0),
	_indexModel(0),
	_initialized(false)
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
	connect(_ui->radioButton_basic, SIGNAL(toggled(bool)), this, SLOT(setupTreeView()));

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
	connect(_ui->checkBox_imageHighestHypShown, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->checkBox_beep, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));
	connect(_ui->horizontalSlider_keypointsOpacity, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteGeneralPanel()));

	//Source panel
	connect(_ui->general_doubleSpinBox_imgRate, SIGNAL(valueChanged(double)), this, SLOT(makeObsoleteSourcePanel()));
	//Image source
	connect(_ui->groupBox_sourceImage, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	_ui->stackedWidget_image->setCurrentIndex(_ui->source_comboBox_image_type->currentIndex());
	connect(_ui->source_comboBox_image_type, SIGNAL(currentIndexChanged(int)), _ui->stackedWidget_image, SLOT(setCurrentIndex(int)));
	connect(_ui->source_comboBox_image_type, SIGNAL(currentIndexChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->general_checkBox_autoRestart, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->general_checkBox_cameraKeypoints, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//usbDevice group
	connect(_ui->source_usbDevice_spinBox_id, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_imgWidth, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_imgheight, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_framesDropped, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//images group
	connect(_ui->source_images_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceImage()));
	connect(_ui->source_images_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_images_spinBox_startPos, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_images_refreshDir, SIGNAL(stateChanged(int)), this, SLOT(makeObsoleteSourcePanel()));
	//video group
	connect(_ui->source_video_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceImage()));
	connect(_ui->source_video_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	//database group
	connect(_ui->source_database_toolButton_selectSource, SIGNAL(clicked()), this, SLOT(selectSourceDatabase()));
	connect(_ui->toolButton_dbViewer, SIGNAL(clicked()), this, SLOT(openDatabaseViewer()));
	connect(_ui->groupBox_sourceDatabase, SIGNAL(toggled(bool)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_database_lineEdit_path, SIGNAL(textChanged(const QString &)), this, SLOT(makeObsoleteSourcePanel()));
	connect(_ui->source_spinBox_databaseStartPos, SIGNAL(valueChanged(int)), this, SLOT(makeObsoleteSourcePanel()));

	//Rtabmap basic
	connect(_ui->general_doubleSpinBox_timeThr, SIGNAL(valueChanged(double)), _ui->general_doubleSpinBox_timeThr_2, SLOT(setValue(double)));
	connect(_ui->general_doubleSpinBox_hardThr, SIGNAL(valueChanged(double)), _ui->general_doubleSpinBox_hardThr_2, SLOT(setValue(double)));
	connect(_ui->surf_doubleSpinBox_hessianThr, SIGNAL(valueChanged(double)), _ui->surf_doubleSpinBox_hessianThr_2, SLOT(setValue(double)));
	connect(_ui->doubleSpinBox_similarityThreshold, SIGNAL(valueChanged(double)), _ui->doubleSpinBox_similarityThreshold_2, SLOT(setValue(double)));
	connect(_ui->general_spinBox_imagesBufferSize, SIGNAL(valueChanged(int)), _ui->general_spinBox_imagesBufferSize_2, SLOT(setValue(int)));
	connect(_ui->general_spinBox_maxStMemSize, SIGNAL(valueChanged(int)), _ui->general_spinBox_maxStMemSize_2, SLOT(setValue(int)));

	connect(_ui->lineEdit_workingDirectory, SIGNAL(textChanged(const QString &)), _ui->lineEdit_workingDirectory_2, SLOT(setText(const QString &)));

	connect(_ui->general_doubleSpinBox_timeThr_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_doubleSpinBox_hardThr_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->surf_doubleSpinBox_hessianThr_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->doubleSpinBox_similarityThreshold_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_spinBox_imagesBufferSize_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_spinBox_maxStMemSize_2, SIGNAL(editingFinished()), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_publishStats, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));
	connect(_ui->general_checkBox_publishStats_2, SIGNAL(stateChanged(int)), this, SLOT(updateBasicParameter()));

	connect(_ui->lineEdit_workingDirectory_2, SIGNAL(textChanged(const QString &)), _ui->lineEdit_workingDirectory, SLOT(setText(const QString &)));
	connect(_ui->toolButton_workingDirectory_2, SIGNAL(clicked()), this, SLOT(changeWorkingDirectory()));

	// Map objects name with the corresponding parameter key, needed for the addParameter() slots
	//Rtabmap
	_ui->general_checkBox_publishStats->setObjectName(Parameters::kRtabmapPublishStats().c_str());
	_ui->general_checkBox_publishRawData->setObjectName(Parameters::kRtabmapPublishImage().c_str());
	_ui->general_checkBox_publishPdf->setObjectName(Parameters::kRtabmapPublishPdf().c_str());
	_ui->general_checkBox_publishLikelihood->setObjectName(Parameters::kRtabmapPublishLikelihood().c_str());
	_ui->general_checkBox_statisticLogsBufferedInRAM->setObjectName(Parameters::kRtabmapStatisticLogsBufferedInRAM().c_str());
	_ui->general_checkBox_statisticLogged->setObjectName(Parameters::kRtabmapStatisticLogged().c_str());
	_ui->general_doubleSpinBox_timeThr->setObjectName(Parameters::kRtabmapTimeThr().c_str());
	_ui->general_spinBox_memoryThr->setObjectName(Parameters::kRtabmapMemoryThr().c_str());
	_ui->general_spinBox_imagesBufferSize->setObjectName(Parameters::kRtabmapImageBufferSize().c_str());
	_ui->general_spinBox_maxRetrieved->setObjectName(Parameters::kRtabmapMaxRetrieved().c_str());
	_ui->general_checkBox_likelihoodNullValuesIgnored->setObjectName(Parameters::kRtabmapLikelihoodNullValuesIgnored().c_str());
	_ui->lineEdit_workingDirectory->setObjectName(Parameters::kRtabmapWorkingDirectory().c_str());
	connect(_ui->toolButton_workingDirectory, SIGNAL(clicked()), this, SLOT(changeWorkingDirectory()));

	// Memory
	_ui->general_checkBox_keepRawData->setObjectName(Parameters::kMemImageKept().c_str());
	_ui->general_checkBox_keepRehearsedNodes->setObjectName(Parameters::kMemRehearsedNodesKept().c_str());
	_ui->general_spinBox_maxStMemSize->setObjectName(Parameters::kMemSTMSize().c_str());
	_ui->general_checkBox_similarityOnlyLast->setObjectName(Parameters::kMemRehearsalOnlyWithLast().c_str());
	_ui->doubleSpinBox_similarityThreshold->setObjectName(Parameters::kMemRehearsalSimilarity().c_str());
	_ui->general_checkBox_incrementalMemory->setObjectName(Parameters::kMemIncrementalMemory().c_str());
	_ui->general_doubleSpinBox_recentWmRatio->setObjectName(Parameters::kMemRecentWmRatio().c_str());
	_ui->general_checkBox_RehearsalOldDataKept->setObjectName(Parameters::kMemRehearsalOldDataKept().c_str());
	_ui->general_checkBox_RehearsalIdUpdatedToNewOne->setObjectName(Parameters::kMemRehearsalIdUpdatedToNewOne().c_str());

	// Database
	_ui->general_checkBox_imagesCompressed->setObjectName(Parameters::kDbImagesCompressed().c_str());
	_ui->checkBox_dbInMemory->setObjectName(Parameters::kDbSqlite3InMemory().c_str());
	_ui->spinBox_dbCacheSize->setObjectName(Parameters::kDbSqlite3CacheSize().c_str());
	_ui->comboBox_dbJournalMode->setObjectName(Parameters::kDbSqlite3JournalMode().c_str());
	_ui->comboBox_dbSynchronous->setObjectName(Parameters::kDbSqlite3Synchronous().c_str());
	_ui->comboBox_dbTempStore->setObjectName(Parameters::kDbSqlite3TempStore().c_str());

	// Create hypotheses
	_ui->general_doubleSpinBox_hardThr->setObjectName(Parameters::kRtabmapLoopThr().c_str());
	_ui->general_doubleSpinBox_loopRatio->setObjectName(Parameters::kRtabmapLoopRatio().c_str());

	//Bayes
	_ui->general_doubleSpinBox_vp->setObjectName(Parameters::kBayesVirtualPlacePriorThr().c_str());
	_ui->lineEdit_bayes_predictionLC->setObjectName(Parameters::kBayesPredictionLC().c_str());
	_ui->checkBox_bayes_fullPredictionUpdate->setObjectName(Parameters::kBayesFullPredictionUpdate().c_str());
	connect(_ui->lineEdit_bayes_predictionLC, SIGNAL(textChanged(const QString &)), this, SLOT(updatePredictionPlot()));

	//Keypoint-based
	_ui->checkBox_kp_publishKeypoints->setObjectName(Parameters::kKpPublishKeypoints().c_str());
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
	_ui->general_checkBox_reactivatedWordsComparedToNewWords->setObjectName(Parameters::kKpReactivatedWordsComparedToNewWords().c_str());
	_ui->checkBox_kp_tfIdfLikelihoodUsed->setObjectName(Parameters::kKpTfIdfLikelihoodUsed().c_str());
	_ui->checkBox_kp_parallelized->setObjectName(Parameters::kKpParallelized().c_str());
	_ui->lineEdit_kp_roi->setObjectName(Parameters::kKpRoiRatios().c_str());
	_ui->lineEdit_dictionaryPath->setObjectName(Parameters::kKpDictionaryPath().c_str());
	connect(_ui->toolButton_dictionaryPath, SIGNAL(clicked()), this, SLOT(changeDictionaryPath()));

	//SURF detector
	_ui->surf_doubleSpinBox_hessianThr->setObjectName(Parameters::kSURFHessianThreshold().c_str());
	_ui->surf_spinBox_octaves->setObjectName(Parameters::kSURFOctaves().c_str());
	_ui->surf_spinBox_octaveLayers->setObjectName(Parameters::kSURFOctaveLayers().c_str());
	_ui->checkBox_surfExtended->setObjectName(Parameters::kSURFExtended().c_str());
	_ui->surf_checkBox_upright->setObjectName(Parameters::kSURFUpright().c_str());
	_ui->surf_checkBox_gpuVersion->setObjectName(Parameters::kSURFGpuVersion().c_str());

	//SIFT detector
	_ui->sift_spinBox_nFeatures->setObjectName(Parameters::kSIFTNFeatures().c_str());
	_ui->sift_spinBox_nOctaveLayers->setObjectName(Parameters::kSIFTNOctaveLayers().c_str());
	_ui->sift_doubleSpinBox_contrastThr->setObjectName(Parameters::kSIFTContrastThreshold().c_str());
	_ui->sift_doubleSpinBox_edgeThr->setObjectName(Parameters::kSIFTEdgeThreshold().c_str());
	_ui->sift_doubleSpinBox_sigma->setObjectName(Parameters::kSIFTSigma().c_str());

	// verifyHypotheses
	_ui->comboBox_vh_strategy->setObjectName(Parameters::kRtabmapVhStrategy().c_str());
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
	int currentIndex = 0;
	if(_indexModel)
	{
		currentIndex = _indexModel->itemFromIndex(_ui->treeView->currentIndex())->data().toInt();
		_ui->treeView->setModel(0);
		delete _indexModel;
	}
	_indexModel = new QStandardItemModel(this);
	// Parse the model
	QList<QGroupBox*> boxes = this->getGroupBoxes();
	if(_ui->radioButton_basic->isChecked())
	{
		boxes = boxes.mid(0,3);
	}
	else // Advanced
	{
		boxes.removeAt(2);
	}

	QStandardItem * parentItem = _indexModel->invisibleRootItem();
	int index = 0;
	this->parseModel(boxes, parentItem, 0, index); // recursive method
	if(_ui->radioButton_advanced->isChecked() && index != _ui->stackedWidget->count()-1)
	{
		ULOGGER_ERROR("The tree model is not the same size of the stacked widgets...%d vs %d advanced stacks", index, _ui->stackedWidget->count()-1);
	}
	if(_ui->radioButton_basic->isChecked())
	{
		if(currentIndex >= 2)
		{
			_ui->stackedWidget->setCurrentIndex(2);
			currentIndex = 2;
		}
	}
	else // Advanced
	{
		if(currentIndex == 2)
		{
			_ui->stackedWidget->setCurrentIndex(3);
		}
	}
	_ui->treeView->setModel(_indexModel);
	_ui->treeView->setCurrentIndex(_indexModel->index(currentIndex, 0));
	_ui->treeView->expandToDepth(0);
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
		QString objectName = boxes.at(absoluteIndex)->objectName();
		QString title = boxes.at(absoluteIndex)->title();
		bool ok = false;
		int lvl = QString(objectName.at(objectName.size()-1)).toInt(&ok);
		if(!ok)
		{
			ULOGGER_ERROR("Error while parsing the first number of the QGroupBox title (%s), the first character must be the number in the hierarchy", title.toStdString().c_str());
			return false;
		}


		if(lvl == currentLevel)
		{
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
	if(item && item->isEnabled())
	{
		int index = item->data().toInt();
		if(_ui->radioButton_advanced->isChecked() && index >= 2)
		{
			++index;
		}
		_ui->stackedWidget->setCurrentIndex(index);
		_ui->scrollArea->horizontalScrollBar()->setValue(0);
		_ui->scrollArea->verticalScrollBar()->setValue(0);
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
		updateBasicParameter();// make that changes without editing finished signal are updated.
		if((_obsoletePanels & kPanelAll) || _parameters.size())
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
		updateBasicParameter();// make that changes without editing finished signal are updated.
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
		if(boxes.at(panelNumber)->objectName() == "groupBox_generalSettingsGui0")
		{
			_ui->general_checkBox_imagesKept->setChecked(DEFAULT_GUI_IMAGES_KEPT);
			_ui->comboBox_loggerLevel->setCurrentIndex(DEFAULT_LOGGER_LEVEL);
			_ui->comboBox_loggerEventLevel->setCurrentIndex(DEFAULT_LOGGER_EVENT_LEVEL);
			_ui->comboBox_loggerPauseLevel->setCurrentIndex(DEFAULT_LOGGER_PAUSE_LEVEL);
			_ui->checkBox_logger_printTime->setChecked(DEFAULT_LOGGER_PRINT_TIME);
			_ui->comboBox_loggerType->setCurrentIndex(DEFAULT_LOGGER_TYPE);
		}
		else if(boxes.at(panelNumber)->objectName() == "groupBox_source0")
		{
			_ui->groupBox_sourceImage->setChecked(true);
			_ui->general_doubleSpinBox_imgRate->setValue(1.0);
			_ui->source_spinBox_imgWidth->setValue(0);
			_ui->source_spinBox_imgheight->setValue(0);
			_ui->source_spinBox_framesDropped->setValue(0);
			_ui->general_checkBox_autoRestart->setChecked(false);
			_ui->general_checkBox_cameraKeypoints->setChecked(false);
			_ui->source_images_spinBox_startPos->setValue(1);
			_ui->source_images_refreshDir->setChecked(false);

			_ui->groupBox_sourceDatabase->setChecked(false);
			_ui->source_spinBox_databaseStartPos->setValue(0);
		}
		else if(boxes.at(panelNumber)->objectName() == "groupBox_rtabmap_basic0")
		{
			_ui->general_doubleSpinBox_timeThr_2->setValue(Parameters::defaultRtabmapTimeThr());
			_ui->general_doubleSpinBox_hardThr_2->setValue(Parameters::defaultRtabmapLoopThr());
			_ui->surf_doubleSpinBox_hessianThr_2->setValue(Parameters::defaultSURFHessianThreshold());
			_ui->doubleSpinBox_similarityThreshold_2->setValue(Parameters::defaultMemRehearsalSimilarity());
			_ui->general_spinBox_imagesBufferSize_2->setValue(Parameters::defaultRtabmapImageBufferSize());
			_ui->general_spinBox_maxStMemSize_2->setValue(Parameters::defaultMemSTMSize());
			_ui->general_checkBox_publishStats_2->setChecked(Parameters::defaultRtabmapPublishStats());
			_ui->lineEdit_workingDirectory_2->setText(Parameters::defaultRtabmapWorkingDirectory().c_str());
			// match the advanced (spin and doubleSpin boxes)
			_ui->general_doubleSpinBox_timeThr->setValue(Parameters::defaultRtabmapTimeThr());
			_ui->general_doubleSpinBox_hardThr->setValue(Parameters::defaultRtabmapLoopThr());
			_ui->surf_doubleSpinBox_hessianThr->setValue(Parameters::defaultSURFHessianThreshold());
			_ui->doubleSpinBox_similarityThreshold->setValue(Parameters::defaultMemRehearsalSimilarity());
			_ui->general_spinBox_imagesBufferSize->setValue(Parameters::defaultRtabmapImageBufferSize());
			_ui->general_spinBox_maxStMemSize->setValue(Parameters::defaultMemSTMSize());
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
	QString privatePath = QDir::homePath() + "/.rtabmap";
	if(!QDir(privatePath).exists())
	{
		QDir::home().mkdir(".rtabmap");
	}
	return privatePath + "/rtabmap.ini";
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
	if(!readCoreSettings(filePath))
	{
		_parameters.clear();
		_obsoletePanels = kPanelDummy;

		// only keep GUI settings
		QStandardItem * parentItem = _indexModel->invisibleRootItem();
		if(parentItem)
		{
			for(int i=1; i<parentItem->rowCount(); ++i)
			{
				parentItem->child(i)->setEnabled(false);
			}
		}
		_ui->radioButton_basic->setEnabled(false);
		_ui->radioButton_advanced->setEnabled(false);
	}
	else
	{
		// enable settings
		QStandardItem * parentItem = _indexModel->invisibleRootItem();
		if(parentItem)
		{
			for(int i=0; i<parentItem->rowCount(); ++i)
			{
				parentItem->child(i)->setEnabled(true);
			}
		}
		_ui->radioButton_basic->setEnabled(true);
		_ui->radioButton_advanced->setEnabled(true);
	}
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
	_ui->groupBox_sourceImage->setChecked(settings.value("imageUsed", _ui->groupBox_sourceImage->isChecked()).toBool());
	_ui->general_doubleSpinBox_imgRate->setValue(settings.value("imgRate", _ui->general_doubleSpinBox_imgRate->value()).toDouble());
	_ui->general_checkBox_autoRestart->setChecked(settings.value("autoRestart", _ui->general_checkBox_autoRestart->isChecked()).toBool());
	_ui->general_checkBox_cameraKeypoints->setChecked(settings.value("cameraKeypoints", _ui->general_checkBox_cameraKeypoints->isChecked()).toBool());
	_ui->source_comboBox_image_type->setCurrentIndex(settings.value("type", _ui->source_comboBox_image_type->currentIndex()).toInt());
	_ui->source_spinBox_imgWidth->setValue(settings.value("imgWidth",_ui->source_spinBox_imgWidth->value()).toInt());
	_ui->source_spinBox_imgheight->setValue(settings.value("imgHeight",_ui->source_spinBox_imgheight->value()).toInt());
	_ui->source_spinBox_framesDropped->setValue(settings.value("framesDropped",_ui->source_spinBox_framesDropped->value()).toInt());
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
	settings.endGroup(); // Camera

	settings.beginGroup("Database");
	_ui->groupBox_sourceDatabase->setChecked(settings.value("databaseUsed", _ui->groupBox_sourceDatabase->isChecked()).toBool());
	_ui->source_database_lineEdit_path->setText(settings.value("path",_ui->source_database_lineEdit_path->text()).toString());
	_ui->source_spinBox_databaseStartPos->setValue(settings.value("startPos", _ui->source_spinBox_databaseStartPos->value()).toInt());
	settings.endGroup(); // Database
}

bool PreferencesDialog::readCoreSettings(const QString & filePath)
{
	UDEBUG("");
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}

	if(!QFile::exists(path))
	{
		QMessageBox::information(this, tr("INI file doesn't exist..."), tr("The configuration file \"%1\" does not exist, it will be created with default parameters.").arg(path));
	}

	QSettings settings(path, QSettings::IniFormat);


	settings.beginGroup("Core");
	QStringList keys = settings.allKeys();
	// Get profile
	const rtabmap::ParametersMap & parameters = Parameters::getDefaultParameters();
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
	return true;
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
	settings.setValue("imageUsed", 		_ui->groupBox_sourceImage->isChecked());
	settings.setValue("imgRate", 		_ui->general_doubleSpinBox_imgRate->value());
	settings.setValue("autoRestart", 	_ui->general_checkBox_autoRestart->isChecked());
	settings.setValue("cameraKeypoints", _ui->general_checkBox_cameraKeypoints->isChecked());
	settings.setValue("type", 			_ui->source_comboBox_image_type->currentIndex());
	settings.setValue("imgWidth", 		_ui->source_spinBox_imgWidth->value());
	settings.setValue("imgHeight", 		_ui->source_spinBox_imgheight->value());
	settings.setValue("framesDropped",  _ui->source_spinBox_framesDropped->value());
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

	settings.endGroup(); // Camera

	settings.beginGroup("Database");
	settings.setValue("databaseUsed", 	_ui->groupBox_sourceDatabase->isChecked());
	settings.setValue("path", 			_ui->source_database_lineEdit_path->text());
	settings.setValue("startPos",       _ui->source_spinBox_databaseStartPos->value());
	settings.endGroup();
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
				settings.setValue(obj->objectName(), uBool2Str(check->isChecked()).c_str());
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
	if(this->isVisible())
	{
		_progressDialog->setLabelText(tr("Setup dialog..."));

		this->updatePredictionPlot();
		this->setupKpRoiPanel();
	}

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

void PreferencesDialog::selectSourceImage(Src src)
{
	ULOGGER_DEBUG("");

	bool fromPrefDialog = false;
	//bool modified = false;
	if(src == kSrcUndef)
	{
		fromPrefDialog = true;
		if(_ui->source_comboBox_image_type->currentIndex() == 1)
		{
			src = kSrcImages;
		}
		else if(_ui->source_comboBox_image_type->currentIndex() == 2)
		{
			src = kSrcVideo;
		}
		else
		{
			src = kSrcUsbDevice;
		}
	}
	else
	{
		// from user
		_ui->groupBox_sourceImage->setChecked(true);
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
				_ui->source_comboBox_image_type->setCurrentIndex(1);
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
		QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->source_video_lineEdit_path->text(), tr("Videos (*.avi *.mpg *.mp4)"));
		QFile file(path);
		if(!path.isEmpty() && file.exists())
		{
			_ui->source_comboBox_image_type->setCurrentIndex(2);
			_ui->source_video_lineEdit_path->setText(path);
		}
	}
	else
	{
		_ui->source_comboBox_image_type->setCurrentIndex(0);
	}

	if(!fromPrefDialog && _obsoletePanels)
	{
		_ui->groupBox_sourceDatabase->setChecked(false);
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

void PreferencesDialog::selectSourceDatabase(bool user)
{
	ULOGGER_DEBUG("");

	if(user)
	{
		// from user
		_ui->groupBox_sourceDatabase->setChecked(true);
	}

	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), _ui->source_database_lineEdit_path->text(), tr("RTAB-Map database files (*.db)"));
	QFile file(path);
	if(!path.isEmpty() && file.exists())
	{
		_ui->source_database_lineEdit_path->setText(path);
		_ui->source_spinBox_databaseStartPos->setValue(0);
	}

	if(user && _obsoletePanels)
	{
		_ui->groupBox_sourceImage->setChecked(false);
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

void PreferencesDialog::openDatabaseViewer()
{
	DatabaseViewer * viewer = new DatabaseViewer(this);
	viewer->setAttribute(Qt::WA_DeleteOnClose, true);
	viewer->setWindowModality(Qt::WindowModal);
	if(viewer->openDatabase(_ui->source_database_lineEdit_path->text()))
	{
		viewer->show();
	}
	else
	{
		delete viewer;
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
		bool ok;
		if(spin)
		{
			spin->setValue(QString(value.c_str()).toInt(&ok));
			if(!ok)
			{
				UERROR("Conversion failed from \"%s\" for parameter %s", value.c_str(), key.c_str());
			}
		}
		else if(doubleSpin)
		{
			doubleSpin->setValue(QString(value.c_str()).toDouble(&ok));
			if(!ok)
			{
				UERROR("Conversion failed from \"%s\" for parameter %s", value.c_str(), key.c_str());
			}
		}
		else if(combo)
		{
			combo->setCurrentIndex(QString(value.c_str()).toInt(&ok));
			if(!ok)
			{
				UERROR("Conversion failed from \"%s\" for parameter %s", value.c_str(), key.c_str());
			}
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
		// Make sure the value is inserted, check if the same key already exists
		rtabmap::ParametersMap::iterator iter = _parameters.find(object->objectName().toStdString());
		if(iter != _parameters.end())
		{
			_parameters.erase(iter);
		}

		const QComboBox * comboBox = qobject_cast<const QComboBox*>(object);
		const QSpinBox * spinbox = qobject_cast<const QSpinBox*>(object);
		const QCheckBox * checkbox = qobject_cast<const QCheckBox*>(object);
		if(comboBox || spinbox)
		{
			if(comboBox)
			{
				// Add related panels to parameters
				if(comboBox == _ui->comboBox_vh_strategy)
				{
					if(value == 0) // 0 none
					{
						// No panel related...
					}
					else if(value == 1) // 2 epipolar
					{
						this->addParameters(_ui->groupBox_vh_epipolar2);
					}
				}
				else if(comboBox == _ui->comboBox_detector_strategy)
				{
					if(value == 0) // 0 surf
					{
						this->addParameters(_ui->groupBox_detector_surf2);
					}
					else if(value == 1) // 1 sift
					{
						this->addParameters(_ui->groupBox_detector_sift2);
					}
				}
				else if(comboBox == _ui->comboBox_descriptor_strategy)
				{
					if(value == 0) // 0 surf
					{
						this->addParameters(_ui->groupBox_detector_surf2);
					}
					else if(value == 1) // 1 sift
					{
						this->addParameters(_ui->groupBox_detector_sift2);
					}
				}
			}
			// Add parameter
			_parameters.insert(rtabmap::ParametersPair(object->objectName().toStdString(), QString::number(value).toStdString()));
		}
		else if(checkbox)
		{
			// Add parameter
			_parameters.insert(rtabmap::ParametersPair(object->objectName().toStdString(), uBool2Str(value)));
		}
		else
		{
			UWARN("Undefined object \"%s\"", object->objectName().toStdString().c_str());
		}

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

void PreferencesDialog::updateBasicParameter()
{
	// This method is used to update basic/advanced referred parameters, see above editingFinished()

	// basic to advanced (advanced to basic must be done by connecting signal valueChanged())
	if(sender() == _ui->general_doubleSpinBox_timeThr_2)
	{
		_ui->general_doubleSpinBox_timeThr->setValue(_ui->general_doubleSpinBox_timeThr_2->value());
	}
	else if(sender() == _ui->general_doubleSpinBox_hardThr_2)
	{
		_ui->general_doubleSpinBox_hardThr->setValue(_ui->general_doubleSpinBox_hardThr_2->value());
	}
	else if(sender() == _ui->surf_doubleSpinBox_hessianThr_2)
	{
		_ui->surf_doubleSpinBox_hessianThr->setValue(_ui->surf_doubleSpinBox_hessianThr_2->value());
	}
	else if(sender() == _ui->general_spinBox_imagesBufferSize_2)
	{
		_ui->general_spinBox_imagesBufferSize->setValue(_ui->general_spinBox_imagesBufferSize_2->value());
	}
	else if(sender() == _ui->general_spinBox_maxStMemSize_2)
	{
		_ui->general_spinBox_maxStMemSize->setValue(_ui->general_spinBox_maxStMemSize_2->value());
	}
	else if(sender() == _ui->general_checkBox_publishStats)
	{
		_ui->general_checkBox_publishStats_2->setChecked(_ui->general_checkBox_publishStats->isChecked());
	}
	else if(sender() == _ui->general_checkBox_publishStats_2)
	{
		_ui->general_checkBox_publishStats->setChecked(_ui->general_checkBox_publishStats_2->isChecked());
	}
	else if(sender() == _ui->doubleSpinBox_similarityThreshold_2)
	{
		_ui->doubleSpinBox_similarityThreshold->setValue(_ui->doubleSpinBox_similarityThreshold_2->value());
	}
	else
	{
		//update all values (only those using editingFinished signal)
		_ui->general_doubleSpinBox_timeThr->setValue(_ui->general_doubleSpinBox_timeThr_2->value());
		_ui->general_doubleSpinBox_hardThr->setValue(_ui->general_doubleSpinBox_hardThr_2->value());
		_ui->surf_doubleSpinBox_hessianThr->setValue(_ui->surf_doubleSpinBox_hessianThr_2->value());
		_ui->general_spinBox_imagesBufferSize->setValue(_ui->general_spinBox_imagesBufferSize_2->value());
		_ui->general_spinBox_maxStMemSize->setValue(_ui->general_spinBox_maxStMemSize_2->value());
		_ui->doubleSpinBox_similarityThreshold->setValue(_ui->doubleSpinBox_similarityThreshold_2->value());
	}
}

void PreferencesDialog::makeObsoleteGeneralPanel()
{
	ULOGGER_DEBUG("");
	_obsoletePanels = _obsoletePanels | kPanelGeneral;
}

void PreferencesDialog::makeObsoleteSourcePanel()
{
	if(sender() == _ui->groupBox_sourceDatabase && _ui->groupBox_sourceDatabase->isChecked())
	{
		_ui->groupBox_sourceImage->setChecked(false);
	}
	else if(sender() == _ui->groupBox_sourceImage && _ui->groupBox_sourceImage->isChecked())
	{
		_ui->groupBox_sourceDatabase->setChecked(false);
	}
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
		}
		else
		{
			ULOGGER_ERROR("A QGroupBox must be included in the first level of children in stacked widget, index=%d", i);
		}
	}
	return boxes;
}

void PreferencesDialog::updatePredictionPlot()
{
	QStringList values = _ui->lineEdit_bayes_predictionLC->text().simplified().split(' ');
	if(values.size() < 2)
	{
		UERROR("Error parsing prediction (must have at least 2 items) : %s",
				_ui->lineEdit_bayes_predictionLC->text().toStdString().c_str());
		return;
	}
	QVector<float> dataX((values.size()-2)*2 + 1);
	QVector<float> dataY((values.size()-2)*2 + 1);
	double value;
	double sum = 0;
	int lvl = 1;
	bool ok = false;
	bool error = false;
	int loopClosureIndex = (dataX.size()-1)/2;
	for(int i=0; i<values.size(); ++i)
	{
		value = values.at(i).toDouble(&ok);
		if(!ok)
		{
			UERROR("Error parsing prediction : \"%s\"", values.at(i).toStdString().c_str());
			error = true;
		}
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
		}
	}

	_ui->label_prediction_sum->setNum(sum);
	if(error)
	{
		_ui->label_prediction_sum->setText(QString("<font color=#FF0000>") + _ui->label_prediction_sum->text() + "</font>");
	}
	else if(sum == 1.0)
	{
		_ui->label_prediction_sum->setText(QString("<font color=#00FF00>") + _ui->label_prediction_sum->text() + "</font>");
	}
	else if(sum > 1.0)
	{
		_ui->label_prediction_sum->setText(QString("<font color=#FFa500>") + _ui->label_prediction_sum->text() + "</font>");
	}
	else
	{
		_ui->label_prediction_sum->setText(QString("<font color=#000000>") + _ui->label_prediction_sum->text() + "</font>");
	}

	_ui->predictionPlot->removeCurves();
	_ui->predictionPlot->addCurve(new UPlotCurve("Prediction", dataX, dataY, _ui->predictionPlot));
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
double PreferencesDialog::getGeneralInputRate() const
{
	return _ui->general_doubleSpinBox_imgRate->value();
}
bool PreferencesDialog::isSourceImageUsed() const
{
	return _ui->groupBox_sourceImage->isChecked();
}
bool PreferencesDialog::isSourceDatabaseUsed() const
{
	return _ui->groupBox_sourceDatabase->isChecked();
}
bool PreferencesDialog::getGeneralAutoRestart() const
{
	return _ui->general_checkBox_autoRestart->isChecked();
}
bool PreferencesDialog::getGeneralCameraKeypoints() const
{
	return _ui->general_checkBox_cameraKeypoints->isChecked();
}
int PreferencesDialog::getSourceImageType() const
{
	return _ui->source_comboBox_image_type->currentIndex();
}
QString PreferencesDialog::getSourceImageTypeStr() const
{
	return _ui->source_comboBox_image_type->currentText();
}
int PreferencesDialog::getSourceWidth() const
{
	return _ui->source_spinBox_imgWidth->value();
}
int PreferencesDialog::getSourceHeight() const
{
	return _ui->source_spinBox_imgheight->value();
}
int PreferencesDialog::getFramesDropped() const
{
	return _ui->source_spinBox_framesDropped->value();
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
int PreferencesDialog::getSourceDatabaseStartPos() const
{
	return _ui->source_spinBox_databaseStartPos->value();
}

bool PreferencesDialog::isStatisticsPublished() const
{
	return _ui->general_checkBox_publishStats->isChecked();
}
double PreferencesDialog::getLoopThr() const
{
	return _ui->general_doubleSpinBox_hardThr->value();
}
double PreferencesDialog::getVpThr() const
{
	return _ui->general_doubleSpinBox_vp->value();
}

bool PreferencesDialog::isImagesKept() const
{
	return _ui->general_checkBox_imagesKept->isChecked();
}
float PreferencesDialog::getTimeLimit() const
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

void PreferencesDialog::setInputRate(double value)
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

void PreferencesDialog::setTimeLimit(float value)
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

void PreferencesDialog::disableGeneralCameraKeypoints()
{
	if(_ui->general_checkBox_cameraKeypoints->isChecked())
	{
		_ui->general_checkBox_cameraKeypoints->setChecked(false);
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
