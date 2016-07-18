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

#include "rtabmap/gui/DatabaseViewer.h"
#include "rtabmap/gui/CloudViewer.h"
#include "ui_DatabaseViewer.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QGraphicsLineItem>
#include <QtGui/QCloseEvent>
#include <QGraphicsOpacityEffect>
#include <QtCore/QBuffer>
#include <QtCore/QTextStream>
#include <QtCore/QDateTime>
#include <QtCore/QSettings>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include "rtabmap/core/DBDriver.h"
#include "rtabmap/gui/KeypointItem.h"
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/utilite/UCv2Qt.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_mapping.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/Compression.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/Stereo.h"
#include "rtabmap/core/Optimizer.h"
#include "rtabmap/core/RegistrationVis.h"
#include "rtabmap/core/RegistrationIcp.h"
#include "rtabmap/gui/DataRecorder.h"
#include "rtabmap/core/SensorData.h"
#include "ExportDialog.h"
#include "rtabmap/gui/ProgressDialog.h"
#include "ParametersToolBox.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

namespace rtabmap {

DatabaseViewer::DatabaseViewer(const QString & ini, QWidget * parent) :
	QMainWindow(parent),
	dbDriver_(0),
	savedMaximized_(false),
	firstCall_(true),
	iniFilePath_(ini)
{
	pathDatabase_ = QDir::homePath()+"/Documents/RTAB-Map"; //use home directory by default

	if(!UDirectory::exists(pathDatabase_.toStdString()))
	{
		pathDatabase_ = QDir::homePath();
	}

	ui_ = new Ui_DatabaseViewer();
	ui_->setupUi(this);
	ui_->buttonBox->setVisible(false);
	connect(ui_->buttonBox->button(QDialogButtonBox::Close), SIGNAL(clicked()), this, SLOT(close()));

	ui_->comboBox_logger_level->setVisible(parent==0);
	ui_->label_logger_level->setVisible(parent==0);
	connect(ui_->comboBox_logger_level, SIGNAL(currentIndexChanged(int)), this, SLOT(updateLoggerLevel()));
	connect(ui_->checkBox_verticalLayout, SIGNAL(stateChanged(int)), this, SLOT(setupMainLayout(int)));

	QString title("RTAB-Map Database Viewer[*]");
	this->setWindowTitle(title);

	ui_->dockWidget_constraints->setVisible(false);
	ui_->dockWidget_graphView->setVisible(false);
	ui_->dockWidget_guiparameters->setVisible(false);
	ui_->dockWidget_coreparameters->setVisible(false);
	ui_->dockWidget_info->setVisible(false);
	ui_->dockWidget_stereoView->setVisible(false);
	ui_->dockWidget_view3d->setVisible(false);

	// Create cloud viewers
	constraintsViewer_ = new CloudViewer(ui_->dockWidgetContents);
	cloudViewerA_ = new CloudViewer(ui_->dockWidgetContents_3dviews);
	cloudViewerB_ = new CloudViewer(ui_->dockWidgetContents_3dviews);
	stereoViewer_ = new CloudViewer(ui_->dockWidgetContents_stereo);
	constraintsViewer_->setObjectName("constraintsViewer");
	cloudViewerA_->setObjectName("cloudViewerA");
	cloudViewerB_->setObjectName("cloudViewerB");
	stereoViewer_->setObjectName("stereoViewer");
	ui_->layout_constraintsViewer->addWidget(constraintsViewer_);
	ui_->horizontalLayout_3dviews->addWidget(cloudViewerA_, 1);
	ui_->horizontalLayout_3dviews->addWidget(cloudViewerB_, 1);
	ui_->horizontalLayout_stereo->addWidget(stereoViewer_, 1);

	constraintsViewer_->setCameraLockZ(false);
	constraintsViewer_->setCameraFree();

	ui_->graphicsView_stereo->setAlpha(255);

	QSet<QString> ignoredGroups;
	ignoredGroups.insert("Rtabmap");
	ignoredGroups.insert("Mem");
	ignoredGroups.insert("Kp");
	ignoredGroups.insert("Odom");
	ignoredGroups.insert("OdomBow");
	ignoredGroups.insert("OdomFlow");
	ignoredGroups.insert("OdomMono");
	ignoredGroups.insert("VhEp");
	ignoredGroups.insert("StereoBM");
	ignoredGroups.insert("RGBD");
	ignoredGroups.insert("DbSqlite3");
	ignoredGroups.insert("Bayes");
	ui_->parameters_toolbox->setupUi(ignoredGroups);

	this->readSettings();

	ui_->menuView->addAction(ui_->dockWidget_constraints->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_graphView->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_stereoView->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_view3d->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_guiparameters->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_coreparameters->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_info->toggleViewAction());
	connect(ui_->dockWidget_graphView->toggleViewAction(), SIGNAL(triggered()), this, SLOT(updateGraphView()));

	connect(ui_->parameters_toolbox, SIGNAL(parametersChanged(const QStringList &)), this, SLOT(notifyParametersChanged(const QStringList &)));

	connect(ui_->actionQuit, SIGNAL(triggered()), this, SLOT(close()));

	// connect actions with custom slots
	ui_->actionSave_config->setShortcut(QKeySequence::Save);
	connect(ui_->actionSave_config, SIGNAL(triggered()), this, SLOT(writeSettings()));
	connect(ui_->actionOpen_database, SIGNAL(triggered()), this, SLOT(openDatabase()));
	connect(ui_->actionExport, SIGNAL(triggered()), this, SLOT(exportDatabase()));
	connect(ui_->actionExtract_images, SIGNAL(triggered()), this, SLOT(extractImages()));
	connect(ui_->actionGenerate_graph_dot, SIGNAL(triggered()), this, SLOT(generateGraph()));
	connect(ui_->actionGenerate_local_graph_dot, SIGNAL(triggered()), this, SLOT(generateLocalGraph()));
	connect(ui_->actionGenerate_TORO_graph_graph, SIGNAL(triggered()), this, SLOT(generateTOROGraph()));
	connect(ui_->actionGenerate_g2o_graph_g2o, SIGNAL(triggered()), this, SLOT(generateG2OGraph()));
	ui_->actionGenerate_g2o_graph_g2o->setEnabled(Optimizer::isAvailable(Optimizer::kTypeG2O));
	connect(ui_->actionView_3D_map, SIGNAL(triggered()), this, SLOT(view3DMap()));
	connect(ui_->actionView_3D_laser_scans, SIGNAL(triggered()), this, SLOT(view3DLaserScans()));
	connect(ui_->actionGenerate_3D_map_pcd, SIGNAL(triggered()), this, SLOT(generate3DMap()));
	connect(ui_->actionExport_3D_laser_scans_ply_pcd, SIGNAL(triggered()), this, SLOT(generate3DLaserScans()));
	connect(ui_->actionDetect_more_loop_closures, SIGNAL(triggered()), this, SLOT(detectMoreLoopClosures()));
	connect(ui_->actionRefine_all_neighbor_links, SIGNAL(triggered()), this, SLOT(refineAllNeighborLinks()));
	connect(ui_->actionRefine_all_loop_closure_links, SIGNAL(triggered()), this, SLOT(refineAllLoopClosureLinks()));
	connect(ui_->actionVisual_Refine_all_neighbor_links, SIGNAL(triggered()), this, SLOT(refineVisuallyAllNeighborLinks()));
	connect(ui_->actionVisual_Refine_all_loop_closure_links, SIGNAL(triggered()), this, SLOT(refineVisuallyAllLoopClosureLinks()));
	connect(ui_->actionReset_all_changes, SIGNAL(triggered()), this, SLOT(resetAllChanges()));

	//ICP buttons
	connect(ui_->pushButton_refine, SIGNAL(clicked()), this, SLOT(refineConstraint()));
	connect(ui_->pushButton_refineVisually, SIGNAL(clicked()), this, SLOT(refineConstraintVisually()));
	connect(ui_->pushButton_add, SIGNAL(clicked()), this, SLOT(addConstraint()));
	connect(ui_->pushButton_reset, SIGNAL(clicked()), this, SLOT(resetConstraint()));
	connect(ui_->pushButton_reject, SIGNAL(clicked()), this, SLOT(rejectConstraint()));
	ui_->pushButton_refine->setEnabled(false);
	ui_->pushButton_refineVisually->setEnabled(false);
	ui_->pushButton_add->setEnabled(false);
	ui_->pushButton_reset->setEnabled(false);
	ui_->pushButton_reject->setEnabled(false);

	ui_->actionGenerate_TORO_graph_graph->setEnabled(false);
	ui_->actionGenerate_g2o_graph_g2o->setEnabled(false);

	ui_->horizontalSlider_A->setTracking(false);
	ui_->horizontalSlider_B->setTracking(false);
	ui_->horizontalSlider_A->setEnabled(false);
	ui_->horizontalSlider_B->setEnabled(false);
	connect(ui_->horizontalSlider_A, SIGNAL(valueChanged(int)), this, SLOT(sliderAValueChanged(int)));
	connect(ui_->horizontalSlider_B, SIGNAL(valueChanged(int)), this, SLOT(sliderBValueChanged(int)));
	connect(ui_->horizontalSlider_A, SIGNAL(sliderMoved(int)), this, SLOT(sliderAMoved(int)));
	connect(ui_->horizontalSlider_B, SIGNAL(sliderMoved(int)), this, SLOT(sliderBMoved(int)));

	connect(ui_->spinBox_mesh_angleTolerance, SIGNAL(valueChanged(int)), this, SLOT(update3dView()));
	connect(ui_->spinBox_mesh_fillDepthHoles, SIGNAL(valueChanged(int)), this, SLOT(update3dView()));
	connect(ui_->spinBox_mesh_depthError, SIGNAL(valueChanged(int)), this, SLOT(update3dView()));
	connect(ui_->checkBox_mesh_quad, SIGNAL(toggled(bool)), this, SLOT(update3dView()));
	connect(ui_->spinBox_mesh_triangleSize, SIGNAL(valueChanged(int)), this, SLOT(update3dView()));
	connect(ui_->checkBox_showMesh, SIGNAL(toggled(bool)), this, SLOT(update3dView()));

	ui_->horizontalSlider_neighbors->setTracking(false);
	ui_->horizontalSlider_loops->setTracking(false);
	ui_->horizontalSlider_neighbors->setEnabled(false);
	ui_->horizontalSlider_loops->setEnabled(false);
	connect(ui_->horizontalSlider_neighbors, SIGNAL(valueChanged(int)), this, SLOT(sliderNeighborValueChanged(int)));
	connect(ui_->horizontalSlider_loops, SIGNAL(valueChanged(int)), this, SLOT(sliderLoopValueChanged(int)));
	connect(ui_->horizontalSlider_neighbors, SIGNAL(sliderMoved(int)), this, SLOT(sliderNeighborValueChanged(int)));
	connect(ui_->horizontalSlider_loops, SIGNAL(sliderMoved(int)), this, SLOT(sliderLoopValueChanged(int)));
	connect(ui_->checkBox_showOptimized, SIGNAL(stateChanged(int)), this, SLOT(updateConstraintView()));
	connect(ui_->checkBox_show3Dclouds, SIGNAL(stateChanged(int)), this, SLOT(updateConstraintView()));
	connect(ui_->checkBox_show2DScans, SIGNAL(stateChanged(int)), this, SLOT(updateConstraintView()));
	connect(ui_->checkBox_show3DWords, SIGNAL(stateChanged(int)), this, SLOT(updateConstraintView()));
	ui_->checkBox_showOptimized->setEnabled(false);

	ui_->horizontalSlider_iterations->setTracking(false);
	ui_->horizontalSlider_iterations->setEnabled(false);
	ui_->spinBox_optimizationsFrom->setEnabled(false);
	connect(ui_->horizontalSlider_iterations, SIGNAL(valueChanged(int)), this, SLOT(sliderIterationsValueChanged(int)));
	connect(ui_->horizontalSlider_iterations, SIGNAL(sliderMoved(int)), this, SLOT(sliderIterationsValueChanged(int)));
	connect(ui_->spinBox_optimizationsFrom, SIGNAL(editingFinished()), this, SLOT(updateGraphView()));
	connect(ui_->checkBox_spanAllMaps, SIGNAL(stateChanged(int)), this, SLOT(updateGraphView()));
	connect(ui_->checkBox_ignorePoseCorrection, SIGNAL(stateChanged(int)), this, SLOT(updateGraphView()));
	connect(ui_->checkBox_ignorePoseCorrection, SIGNAL(stateChanged(int)), this, SLOT(updateConstraintView()));
	connect(ui_->checkBox_ignoreGlobalLoop, SIGNAL(stateChanged(int)), this, SLOT(updateGraphView()));
	connect(ui_->checkBox_ignoreLocalLoopSpace, SIGNAL(stateChanged(int)), this, SLOT(updateGraphView()));
	connect(ui_->checkBox_ignoreLocalLoopTime, SIGNAL(stateChanged(int)), this, SLOT(updateGraphView()));
	connect(ui_->checkBox_ignoreUserLoop, SIGNAL(stateChanged(int)), this, SLOT(updateGraphView()));
	connect(ui_->spinBox_optimizationDepth, SIGNAL(editingFinished()), this, SLOT(updateGraphView()));
	connect(ui_->checkBox_gridErode, SIGNAL(stateChanged(int)), this, SLOT(updateGrid()));
	connect(ui_->checkBox_gridFillUnkownSpace, SIGNAL(stateChanged(int)), this, SLOT(updateGrid()));
	connect(ui_->groupBox_posefiltering, SIGNAL(clicked(bool)), this, SLOT(updateGraphView()));
	connect(ui_->doubleSpinBox_posefilteringRadius, SIGNAL(editingFinished()), this, SLOT(updateGraphView()));
	connect(ui_->doubleSpinBox_posefilteringAngle, SIGNAL(editingFinished()), this, SLOT(updateGraphView()));

	connect(ui_->groupBox_gridFromProjection, SIGNAL(clicked(bool)), this, SLOT(updateGrid()));
	connect(ui_->doubleSpinBox_gridCellSize, SIGNAL(editingFinished()), this, SLOT(updateGrid()));
	connect(ui_->spinBox_projDecimation, SIGNAL(editingFinished()), this, SLOT(updateGrid()));
	connect(ui_->doubleSpinBox_projMaxDepth, SIGNAL(editingFinished()), this, SLOT(updateGrid()));
	connect(ui_->doubleSpinBox_projMinDepth, SIGNAL(editingFinished()), this, SLOT(updateGrid()));
	connect(ui_->doubleSpinBox_projMaxAngle, SIGNAL(editingFinished()), this, SLOT(updateGrid()));
	connect(ui_->spinBox_projClusterSize, SIGNAL(editingFinished()), this, SLOT(updateGrid()));

	ui_->label_stereo_inliers_name->setStyleSheet("QLabel {color : blue; }");
	ui_->label_stereo_flowOutliers_name->setStyleSheet("QLabel {color : red; }");
	ui_->label_stereo_slopeOutliers_name->setStyleSheet("QLabel {color : yellow; }");
	ui_->label_stereo_disparityOutliers_name->setStyleSheet("QLabel {color : magenta; }");


	// connect configuration changed
	connect(ui_->graphViewer, SIGNAL(configChanged()), this, SLOT(configModified()));
	//connect(ui_->graphicsView_A, SIGNAL(configChanged()), this, SLOT(configModified()));
	//connect(ui_->graphicsView_B, SIGNAL(configChanged()), this, SLOT(configModified()));
	connect(ui_->comboBox_logger_level, SIGNAL(currentIndexChanged(int)), this, SLOT(configModified()));
	connect(ui_->checkBox_verticalLayout, SIGNAL(stateChanged(int)), this, SLOT(configModified()));
	// Graph view
	connect(ui_->checkBox_spanAllMaps, SIGNAL(stateChanged(int)), this, SLOT(configModified()));
	connect(ui_->checkBox_ignorePoseCorrection, SIGNAL(stateChanged(int)), this, SLOT(configModified()));
	connect(ui_->checkBox_ignoreGlobalLoop, SIGNAL(stateChanged(int)), this, SLOT(configModified()));
	connect(ui_->checkBox_ignoreLocalLoopSpace, SIGNAL(stateChanged(int)), this, SLOT(configModified()));
	connect(ui_->checkBox_ignoreLocalLoopTime, SIGNAL(stateChanged(int)), this, SLOT(configModified()));
	connect(ui_->checkBox_ignoreUserLoop, SIGNAL(stateChanged(int)), this, SLOT(configModified()));
	connect(ui_->spinBox_optimizationDepth, SIGNAL(valueChanged(int)), this, SLOT(configModified()));
	connect(ui_->checkBox_gridErode, SIGNAL(stateChanged(int)), this, SLOT(configModified()));
	connect(ui_->checkBox_gridFillUnkownSpace, SIGNAL(stateChanged(int)), this, SLOT(configModified()));
	connect(ui_->groupBox_gridFromProjection, SIGNAL(clicked(bool)), this, SLOT(configModified()));
	connect(ui_->doubleSpinBox_gridCellSize, SIGNAL(valueChanged(double)), this, SLOT(configModified()));
	connect(ui_->spinBox_projDecimation, SIGNAL(valueChanged(int)), this, SLOT(configModified()));
	connect(ui_->doubleSpinBox_projMaxDepth, SIGNAL(valueChanged(double)), this, SLOT(configModified()));
	connect(ui_->doubleSpinBox_projMinDepth, SIGNAL(valueChanged(double)), this, SLOT(configModified()));
	connect(ui_->doubleSpinBox_projMaxAngle, SIGNAL(valueChanged(double)), this, SLOT(configModified()));
	connect(ui_->spinBox_projClusterSize, SIGNAL(valueChanged(int)), this, SLOT(configModified()));
	connect(ui_->groupBox_posefiltering, SIGNAL(clicked(bool)), this, SLOT(configModified()));
	connect(ui_->doubleSpinBox_posefilteringRadius, SIGNAL(valueChanged(double)), this, SLOT(configModified()));
	connect(ui_->doubleSpinBox_posefilteringAngle, SIGNAL(valueChanged(double)), this, SLOT(configModified()));

	connect(ui_->spinBox_icp_decimation, SIGNAL(valueChanged(int)), this, SLOT(configModified()));
	connect(ui_->doubleSpinBox_icp_maxDepth, SIGNAL(valueChanged(double)), this, SLOT(configModified()));
	connect(ui_->doubleSpinBox_icp_minDepth, SIGNAL(valueChanged(double)), this, SLOT(configModified()));
	connect(ui_->checkBox_icp_laserScan, SIGNAL(stateChanged(int)), this, SLOT(configModified()));
	
	connect(ui_->doubleSpinBox_detectMore_radius, SIGNAL(valueChanged(double)), this, SLOT(configModified()));
	connect(ui_->doubleSpinBox_detectMore_angle, SIGNAL(valueChanged(double)), this, SLOT(configModified()));
	connect(ui_->spinBox_detectMore_iterations, SIGNAL(valueChanged(int)), this, SLOT(configModified()));

	// dockwidget
	QList<QDockWidget*> dockWidgets = this->findChildren<QDockWidget*>();
	for(int i=0; i<dockWidgets.size(); ++i)
	{
		connect(dockWidgets[i], SIGNAL(dockLocationChanged(Qt::DockWidgetArea)), this, SLOT(configModified()));
		connect(dockWidgets[i]->toggleViewAction(), SIGNAL(toggled(bool)), this, SLOT(configModified()));
	}
	ui_->dockWidget_constraints->installEventFilter(this);
	ui_->dockWidget_graphView->installEventFilter(this);
	ui_->dockWidget_stereoView->installEventFilter(this);
	ui_->dockWidget_view3d->installEventFilter(this);
	ui_->dockWidget_guiparameters->installEventFilter(this);
	ui_->dockWidget_coreparameters->installEventFilter(this);
	ui_->dockWidget_info->installEventFilter(this);
}

DatabaseViewer::~DatabaseViewer()
{
	delete ui_;
	if(dbDriver_)
	{
		delete dbDriver_;
	}
}

void DatabaseViewer::setupMainLayout(int vertical)
{
	if(vertical)
	{
		qobject_cast<QHBoxLayout *>(ui_->horizontalLayout_imageViews->layout())->setDirection(QBoxLayout::TopToBottom);
		qobject_cast<QHBoxLayout *>(ui_->horizontalLayout_3dviews->layout())->setDirection(QBoxLayout::TopToBottom);
	}
	else if(!vertical)
	{
		qobject_cast<QHBoxLayout *>(ui_->horizontalLayout_imageViews->layout())->setDirection(QBoxLayout::LeftToRight);
		qobject_cast<QHBoxLayout *>(ui_->horizontalLayout_3dviews->layout())->setDirection(QBoxLayout::LeftToRight);
	}
}

void DatabaseViewer::showCloseButton(bool visible)
{
	ui_->buttonBox->setVisible(visible);
}

void DatabaseViewer::configModified()
{
	this->setWindowModified(true);
}

QString DatabaseViewer::getIniFilePath() const
{
	if(!iniFilePath_.isEmpty())
	{
		return iniFilePath_;
	}
	QString privatePath = QDir::homePath() + "/.rtabmap";
	if(!QDir(privatePath).exists())
	{
		QDir::home().mkdir(".rtabmap");
	}
	return privatePath + "/rtabmap.ini";
}

void DatabaseViewer::readSettings()
{
	QString path = getIniFilePath();
	QSettings settings(path, QSettings::IniFormat);
	settings.beginGroup("DatabaseViewer");

	//load window state / geometry
	QByteArray bytes;
	bytes = settings.value("geometry", QByteArray()).toByteArray();
	if(!bytes.isEmpty())
	{
		this->restoreGeometry(bytes);
	}
	bytes = settings.value("state", QByteArray()).toByteArray();
	if(!bytes.isEmpty())
	{
		this->restoreState(bytes);
	}
	savedMaximized_ = settings.value("maximized", false).toBool();

	ui_->comboBox_logger_level->setCurrentIndex(settings.value("loggerLevel", ui_->comboBox_logger_level->currentIndex()).toInt());
	ui_->checkBox_verticalLayout->setChecked(settings.value("verticalLayout", ui_->checkBox_verticalLayout->isChecked()).toBool());

	// GraphViewer settings
	ui_->graphViewer->loadSettings(settings, "GraphView");

	settings.beginGroup("optimization");
	ui_->checkBox_spanAllMaps->setChecked(settings.value("spanToAllMaps", ui_->checkBox_spanAllMaps->isChecked()).toBool());
	ui_->checkBox_ignorePoseCorrection->setChecked(settings.value("ignorePoseCorrection", ui_->checkBox_ignorePoseCorrection->isChecked()).toBool());
	ui_->checkBox_ignoreGlobalLoop->setChecked(settings.value("ignoreGlobalLoop", ui_->checkBox_ignoreGlobalLoop->isChecked()).toBool());
	ui_->checkBox_ignoreLocalLoopSpace->setChecked(settings.value("ignoreLocalLoopSpace", ui_->checkBox_ignoreLocalLoopSpace->isChecked()).toBool());
	ui_->checkBox_ignoreLocalLoopTime->setChecked(settings.value("ignoreLocalLoopTime", ui_->checkBox_ignoreLocalLoopTime->isChecked()).toBool());
	ui_->checkBox_ignoreUserLoop->setChecked(settings.value("ignoreUserLoop", ui_->checkBox_ignoreUserLoop->isChecked()).toBool());
	ui_->spinBox_optimizationDepth->setValue(settings.value("depth", ui_->spinBox_optimizationDepth->value()).toInt());
	ui_->checkBox_gridErode->setChecked(settings.value("erode", ui_->checkBox_gridErode->isChecked()).toBool());
	ui_->checkBox_gridFillUnkownSpace->setChecked(settings.value("unknownSpaceFilled", ui_->checkBox_gridFillUnkownSpace->isChecked()).toBool());
	settings.endGroup();

	settings.beginGroup("grid");
	ui_->groupBox_gridFromProjection->setChecked(settings.value("gridFromProj", ui_->groupBox_gridFromProjection->isChecked()).toBool());
	ui_->doubleSpinBox_gridCellSize->setValue(settings.value("gridCellSize", ui_->doubleSpinBox_gridCellSize->value()).toDouble());
	ui_->spinBox_projDecimation->setValue(settings.value("projDecimation", ui_->spinBox_projDecimation->value()).toInt());
	ui_->doubleSpinBox_projMaxDepth->setValue(settings.value("projMaxDepth", ui_->doubleSpinBox_projMaxDepth->value()).toDouble());
	ui_->doubleSpinBox_projMinDepth->setValue(settings.value("projMinDepth", ui_->doubleSpinBox_projMinDepth->value()).toDouble());
	ui_->doubleSpinBox_projMaxAngle->setValue(settings.value("projMaxAngle", ui_->doubleSpinBox_projMaxAngle->value()).toDouble());
	ui_->spinBox_projClusterSize->setValue(settings.value("projClusterSize", ui_->spinBox_projClusterSize->value()).toInt());
	ui_->groupBox_posefiltering->setChecked(settings.value("poseFiltering", ui_->groupBox_posefiltering->isChecked()).toBool());
	ui_->doubleSpinBox_posefilteringRadius->setValue(settings.value("poseFilteringRadius", ui_->doubleSpinBox_posefilteringRadius->value()).toDouble());
	ui_->doubleSpinBox_posefilteringAngle->setValue(settings.value("poseFilteringAngle", ui_->doubleSpinBox_posefilteringAngle->value()).toDouble());
	settings.endGroup();

	settings.beginGroup("mesh");
	ui_->checkBox_mesh_quad->setChecked(settings.value("quad", ui_->checkBox_mesh_quad->isChecked()).toBool());
	ui_->spinBox_mesh_angleTolerance->setValue(settings.value("angleTolerance", ui_->spinBox_mesh_angleTolerance->value()).toInt());
	ui_->spinBox_mesh_fillDepthHoles->setValue(settings.value("fillDepthHolesSize", ui_->spinBox_mesh_fillDepthHoles->value()).toInt());
	ui_->spinBox_mesh_depthError->setValue(settings.value("fillDepthHolesError", ui_->spinBox_mesh_depthError->value()).toInt());
	ui_->spinBox_mesh_triangleSize->setValue(settings.value("triangleSize", ui_->spinBox_mesh_triangleSize->value()).toInt());
	settings.endGroup();

	// ImageViews
	//ui_->graphicsView_A->loadSettings(settings, "ImageViewA");
	//ui_->graphicsView_B->loadSettings(settings, "ImageViewB");

	// ICP parameters
	settings.beginGroup("icp");
	ui_->spinBox_icp_decimation->setValue(settings.value("decimation", ui_->spinBox_icp_decimation->value()).toInt());
	ui_->doubleSpinBox_icp_maxDepth->setValue(settings.value("maxDepth", ui_->doubleSpinBox_icp_maxDepth->value()).toDouble());
	ui_->doubleSpinBox_icp_minDepth->setValue(settings.value("minDepth", ui_->doubleSpinBox_icp_minDepth->value()).toDouble());
	ui_->checkBox_icp_laserScan->setChecked(settings.value("icpLaserScan", ui_->checkBox_icp_laserScan->isChecked()).toBool());
	settings.endGroup();
	// Visual parameters
	settings.beginGroup("visual");
	ui_->doubleSpinBox_detectMore_radius->setValue(settings.value("detectMoreRadius", ui_->doubleSpinBox_detectMore_radius->value()).toDouble());
	ui_->doubleSpinBox_detectMore_angle->setValue(settings.value("detectMoreAngle", ui_->doubleSpinBox_detectMore_angle->value()).toDouble());
	ui_->spinBox_detectMore_iterations->setValue(settings.value("detectMoreIterations", ui_->spinBox_detectMore_iterations->value()).toInt());
	settings.endGroup();

	settings.endGroup(); // DatabaseViewer

	ParametersMap parameters;
	Parameters::readINI(path.toStdString(), parameters);
	for(ParametersMap::iterator iter = parameters.begin(); iter!= parameters.end(); ++iter)
	{
		ui_->parameters_toolbox->updateParameter(iter->first, iter->second);
	}
}

void DatabaseViewer::writeSettings()
{
	QString path = getIniFilePath();
	QSettings settings(path, QSettings::IniFormat);
	settings.beginGroup("DatabaseViewer");

	//save window state / geometry
	if(!this->isMaximized())
	{
		settings.setValue("geometry", this->saveGeometry());
	}
	settings.setValue("state", this->saveState());
	settings.setValue("maximized", this->isMaximized());
	savedMaximized_ = this->isMaximized();

	settings.setValue("loggerLevel", ui_->comboBox_logger_level->currentIndex());
	settings.setValue("verticalLayout", ui_->checkBox_verticalLayout->isChecked());

	// save GraphViewer settings
	ui_->graphViewer->saveSettings(settings, "GraphView");

	// save optimization settings
	settings.beginGroup("optimization");
	//settings.setValue("iterations", ui_->spinBox_iterations->value());
	settings.setValue("spanToAllMaps", ui_->checkBox_spanAllMaps->isChecked());
	//settings.setValue("robust", ui_->checkBox_robust->isChecked());
	settings.setValue("ignorePoseCorrection", ui_->checkBox_ignorePoseCorrection->isChecked());
	settings.setValue("ignoreGlobalLoop", ui_->checkBox_ignoreGlobalLoop->isChecked());
	settings.setValue("ignoreLocalLoopSpace", ui_->checkBox_ignoreLocalLoopSpace->isChecked());
	settings.setValue("ignoreLocalLoopTime", ui_->checkBox_ignoreLocalLoopTime->isChecked());
	settings.setValue("ignoreUserLoop", ui_->checkBox_ignoreUserLoop->isChecked());
	//settings.setValue("strategy", ui_->comboBox_graphOptimizer->currentIndex());
	//settings.setValue("slam2d", ui_->checkBox_2dslam->isChecked());
	settings.setValue("depth", ui_->spinBox_optimizationDepth->value());
	settings.setValue("erode", ui_->checkBox_gridErode->isChecked());
	settings.setValue("unknownSpaceFilled", ui_->checkBox_gridFillUnkownSpace->isChecked());
	settings.endGroup();

	// save Grid settings
	settings.beginGroup("grid");
	settings.setValue("gridFromProj", ui_->groupBox_gridFromProjection->isChecked());
	settings.setValue("gridCellSize", ui_->doubleSpinBox_gridCellSize->value());
	settings.setValue("projDecimation", ui_->spinBox_projDecimation->value());
	settings.setValue("projMaxDepth", ui_->doubleSpinBox_projMaxDepth->value());
	settings.setValue("projMinDepth", ui_->doubleSpinBox_projMinDepth->value());
	settings.setValue("projMaxAngle", ui_->doubleSpinBox_projMaxAngle->value());
	settings.setValue("projClusterSize", ui_->spinBox_projClusterSize->value());
	settings.setValue("poseFiltering", ui_->groupBox_posefiltering->isChecked());
	settings.setValue("poseFilteringRadius", ui_->doubleSpinBox_posefilteringRadius->value());
	settings.setValue("poseFilteringAngle", ui_->doubleSpinBox_posefilteringAngle->value());
	settings.endGroup();

	settings.beginGroup("mesh");
	settings.setValue("quad", ui_->checkBox_mesh_quad->isChecked());
	settings.setValue("angleTolerance", ui_->spinBox_mesh_angleTolerance->value());
	settings.setValue("fillDepthHolesSize", ui_->spinBox_mesh_fillDepthHoles->value());
	settings.setValue("fillDepthHolesError", ui_->spinBox_mesh_depthError->value());
	settings.setValue("triangleSize", ui_->spinBox_mesh_triangleSize->value());
	settings.endGroup();

	// ImageViews
	//ui_->graphicsView_A->saveSettings(settings, "ImageViewA");
	//ui_->graphicsView_B->saveSettings(settings, "ImageViewB");

	// save ICP parameters
	settings.beginGroup("icp");
	settings.setValue("decimation", ui_->spinBox_icp_decimation->value());
	settings.setValue("maxDepth", ui_->doubleSpinBox_icp_maxDepth->value());
	settings.setValue("minDepth", ui_->doubleSpinBox_icp_minDepth->value());
	settings.setValue("icpLaserScan", ui_->checkBox_icp_laserScan->isChecked());
	settings.endGroup();
	
	// save Visual parameters
	settings.beginGroup("visual");
	settings.setValue("detectMoreRadius", ui_->doubleSpinBox_detectMore_radius->value());
	settings.setValue("detectMoreAngle", ui_->doubleSpinBox_detectMore_angle->value());
	settings.setValue("detectMoreIterations", ui_->spinBox_detectMore_iterations->value());
	settings.endGroup();

	settings.endGroup(); // DatabaseViewer

	ParametersMap parameters = ui_->parameters_toolbox->getParameters();
	for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end();)
	{
		if(!ui_->parameters_toolbox->getParameterWidget(iter->first.c_str()))
		{
			parameters.erase(iter++);
		}
		else
		{
			++iter;
		}
	}
	Parameters::writeINI(path.toStdString(), parameters);

	this->setWindowModified(false);
}

void DatabaseViewer::openDatabase()
{
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), pathDatabase_, tr("Databases (*.db)"));
	if(!path.isEmpty())
	{
		openDatabase(path);
	}
}

bool DatabaseViewer::openDatabase(const QString & path)
{
	UDEBUG("Open database \"%s\"", path.toStdString().c_str());
	if(QFile::exists(path))
	{
		if(dbDriver_)
		{
			delete dbDriver_;
			dbDriver_ = 0;
			ids_.clear();
			idToIndex_.clear();
			neighborLinks_.clear();
			loopLinks_.clear();
			graphes_.clear();
			graphLinks_.clear();
			poses_.clear();
			groundTruthPoses_.clear();
			mapIds_.clear();
			links_.clear();
			linksAdded_.clear();
			linksRefined_.clear();
			linksRemoved_.clear();
			localMaps_.clear();
			ui_->graphViewer->clearAll();
			ui_->actionGenerate_TORO_graph_graph->setEnabled(false);
			ui_->actionGenerate_g2o_graph_g2o->setEnabled(false);
			ui_->checkBox_showOptimized->setEnabled(false);
			databaseFileName_.clear();
		}

		std::string driverType = "sqlite3";

		dbDriver_ = DBDriver::create();

		if(!dbDriver_->openConnection(path.toStdString()))
		{
			QMessageBox::warning(this, "Database error", tr("Can't open database \"%1\"").arg(path));
		}
		else
		{
			pathDatabase_ = UDirectory::getDir(path.toStdString()).c_str();
			databaseFileName_ = UFile::getName(path.toStdString());

			// look if there are saved parameters
			ParametersMap parameters = dbDriver_->getLastParameters();

			if(parameters.size())
			{
				const ParametersMap & currentParameters = ui_->parameters_toolbox->getParameters();
				ParametersMap differentParameters;
				for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
				{
					ParametersMap::const_iterator jter = currentParameters.find(iter->first);
					if(jter!=currentParameters.end() &&
					   ui_->parameters_toolbox->getParameterWidget(QString(iter->first.c_str())) != 0 &&
					   iter->second.compare(jter->second) != 0 &&
					   iter->first.compare(Parameters::kRtabmapWorkingDirectory()) != 0)
					{
						differentParameters.insert(*iter);
						QString msg = tr("Parameter \"%1\": database=\"%2\" Preferences=\"%3\"")
								.arg(iter->first.c_str())
								.arg(iter->second.c_str())
								.arg(jter->second.c_str());
						UWARN(msg.toStdString().c_str());
					}
				}

				if(differentParameters.size())
				{
					int r = QMessageBox::question(this,
							tr("Update parameters..."),
							tr("The database is using %1 different parameter(s) than "
							   "those currently set in Core parameters panel. Do you want "
							   "to use database's parameters?").arg(differentParameters.size()),
							QMessageBox::Yes | QMessageBox::No,
							QMessageBox::Yes);
					if(r == QMessageBox::Yes)
					{
						for(rtabmap::ParametersMap::const_iterator iter = differentParameters.begin(); iter!=differentParameters.end(); ++iter)
						{
							ui_->parameters_toolbox->updateParameter(iter->first, iter->second);
						}
					}
				}
			}

			updateIds();
			return true;
		}
	}
	else
	{
		QMessageBox::warning(this, "Database error", tr("Database \"%1\" does not exist.").arg(path));
	}
	return false;
}

void DatabaseViewer::closeEvent(QCloseEvent* event)
{
	//write settings before quit?
	bool save = false;
	if(this->isWindowModified())
	{
		QMessageBox::Button b=QMessageBox::question(this,
				tr("Database Viewer"),
				tr("There are unsaved changed settings. Save them?"),
				QMessageBox::Save | QMessageBox::Cancel | QMessageBox::Discard);
		if(b == QMessageBox::Save)
		{
			save = true;
		}
		else if(b != QMessageBox::Discard)
		{
			event->ignore();
			return;
		}
	}

	if(save)
	{
		writeSettings();
	}

	if(linksAdded_.size() || linksRefined_.size() || linksRemoved_.size())
	{
		QMessageBox::StandardButton button = QMessageBox::question(this,
				tr("Links modified"),
				tr("Some links are modified (%1 added, %2 refined, %3 removed), do you want to save them?")
				.arg(linksAdded_.size()).arg(linksRefined_.size()).arg(linksRemoved_.size()),
				QMessageBox::Cancel | QMessageBox::Yes | QMessageBox::No,
				QMessageBox::Cancel);

		if(button == QMessageBox::Yes)
		{
			// Added links
			for(std::multimap<int, rtabmap::Link>::iterator iter=linksAdded_.begin(); iter!=linksAdded_.end(); ++iter)
			{
				std::multimap<int, rtabmap::Link>::iterator refinedIter = rtabmap::graph::findLink(linksRefined_, iter->second.from(), iter->second.to());
				if(refinedIter != linksRefined_.end())
				{
					dbDriver_->addLink(refinedIter->second);
					dbDriver_->addLink(refinedIter->second.inverse());
				}
				else
				{
					dbDriver_->addLink(iter->second);
					dbDriver_->addLink(iter->second.inverse());
				}
			}

			//Refined links
			for(std::multimap<int, rtabmap::Link>::iterator iter=linksRefined_.begin(); iter!=linksRefined_.end(); ++iter)
			{
				if(!containsLink(linksAdded_, iter->second.from(), iter->second.to()))
				{
					dbDriver_->updateLink(iter->second);
					dbDriver_->updateLink(iter->second.inverse());
				}
			}

			// Rejected links
			for(std::multimap<int, rtabmap::Link>::iterator iter=linksRemoved_.begin(); iter!=linksRemoved_.end(); ++iter)
			{
				dbDriver_->removeLink(iter->second.to(), iter->second.from());
				dbDriver_->removeLink(iter->second.from(), iter->second.to());
			}
		}

		if(button == QMessageBox::Yes || button == QMessageBox::No)
		{
			event->accept();
		}
		else
		{
			event->ignore();
		}
	}
	else
	{
		event->accept();
	}

	if(event->isAccepted())
	{
		if(dbDriver_)
		{
			delete dbDriver_;
			dbDriver_ = 0;
		}
	}
}

void DatabaseViewer::showEvent(QShowEvent* anEvent)
{
	this->setWindowModified(false);

	if(ui_->graphViewer->isVisible() && graphes_.size() && localMaps_.size()==0)
	{
		sliderIterationsValueChanged((int)graphes_.size()-1);
	}
}

void DatabaseViewer::moveEvent(QMoveEvent* anEvent)
{
	if(this->isVisible())
	{
		// HACK, there is a move event when the window is shown the first time.
		if(!firstCall_)
		{
			this->configModified();
		}
		firstCall_ = false;
	}
}

void DatabaseViewer::resizeEvent(QResizeEvent* anEvent)
{
	if(this->isVisible())
	{
		this->configModified();
	}
}

bool DatabaseViewer::eventFilter(QObject *obj, QEvent *event)
{
	if (event->type() == QEvent::Resize && qobject_cast<QDockWidget*>(obj))
	{
		this->setWindowModified(true);
	}
	return QWidget::eventFilter(obj, event);
}


void DatabaseViewer::exportDatabase()
{
	if(!dbDriver_ || ids_.size() == 0)
	{
		return;
	}

	rtabmap::ExportDialog dialog;

	if(dialog.exec())
	{
		if(!dialog.outputPath().isEmpty())
		{
			int framesIgnored = dialog.framesIgnored();
			double frameRate = dialog.targetFramerate();
			int sessionExported = dialog.sessionExported();
			QString path = dialog.outputPath();
			rtabmap::DataRecorder recorder;
			QList<int> ids;

			double previousStamp = 0;
			std::vector<double> delays(ids_.size());
			int oi=0;
			std::map<int, Transform> poses;
			std::map<int, double> stamps;
			std::map<int, Transform> groundTruths;
			for(int i=0; i<ids_.size(); i+=1+framesIgnored)
			{
				Transform odomPose, groundTruth;
				int weight = -1;
				int mapId = -1;
				std::string label;
				double stamp = 0;
				if(dbDriver_->getNodeInfo(ids_[i], odomPose, mapId, weight, label, stamp, groundTruth))
				{
					if(frameRate == 0 ||
					   previousStamp == 0 ||
					   stamp == 0 ||
					   stamp - previousStamp >= 1.0/frameRate)
					{
						if(sessionExported < 0 || sessionExported == mapId)
						{
							ids.push_back(ids_[i]);

							if(previousStamp && stamp)
							{
								delays[oi++] = stamp - previousStamp;
							}
							previousStamp = stamp;

							poses.insert(std::make_pair(ids_[i], odomPose));
							stamps.insert(std::make_pair(ids_[i], stamp));
							groundTruths.insert(std::make_pair(ids_[i], groundTruth));
						}
					}
					if(sessionExported >= 0 && mapId > sessionExported)
					{
						break;
					}
				}
			}
			delays.resize(oi);

			if(recorder.init(path, false))
			{
				rtabmap::ProgressDialog * progressDialog = new rtabmap::ProgressDialog(this);
				progressDialog->setAttribute(Qt::WA_DeleteOnClose);
				progressDialog->setMaximumSteps(ids.size());
				progressDialog->show();
				UINFO("Decompress: rgb=%d depth=%d scan=%d userData=%d",
						dialog.isRgbExported()?1:0,
						dialog.isDepthExported()?1:0,
						dialog.isDepth2dExported()?1:0,
						dialog.isUserDataExported()?1:0);

				for(int i=0; i<ids.size(); ++i)
				{
					int id = ids.at(i);

					SensorData data;
					dbDriver_->getNodeData(id, data);
					cv::Mat depth, rgb, scan, userData;
					data.uncompressDataConst(
							!dialog.isRgbExported()?0:&rgb,
							!dialog.isDepthExported()?0:&depth,
							!dialog.isDepth2dExported()?0:&scan,
							!dialog.isUserDataExported()?0:&userData);
					cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
					if(dialog.isOdomExported())
					{
						std::map<int, Link> links;
						dbDriver_->loadLinks(id, links, Link::kNeighbor);
						if(links.size() && links.begin()->first < id)
						{
							covariance = links.begin()->second.infMatrix().inv();
						}
					}

					rtabmap::SensorData sensorData;
					if(data.cameraModels().size())
					{
						sensorData = rtabmap::SensorData(
							scan,
							dialog.isDepth2dExported()?data.laserScanMaxPts():0,
							dialog.isDepth2dExported()?data.laserScanMaxRange():0,
							rgb,
							depth,
							data.cameraModels(),
							id,
							stamps.at(id),
							userData);
					}
					else
					{
						sensorData = rtabmap::SensorData(
							scan,
							dialog.isDepth2dExported()?data.laserScanMaxPts():0,
							dialog.isDepth2dExported()?data.laserScanMaxRange():0,
							rgb,
							depth,
							data.stereoCameraModel(),
							id,
							stamps.at(id),
							userData);
					}
					sensorData.setGroundTruth(groundTruths.at(id));

					recorder.addData(sensorData, dialog.isOdomExported()?poses.at(id):Transform(), covariance);

					progressDialog->appendText(tr("Exported node %1").arg(id));
					progressDialog->incrementStep();
					QApplication::processEvents();
				}
				progressDialog->setValue(progressDialog->maximumSteps());
				if(delays.size())
				{
					progressDialog->appendText(tr("Average frame rate=%1 Hz (Min=%2, Max=%3)")
							.arg(1.0/uMean(delays)).arg(1.0/uMax(delays)).arg(1.0/uMin(delays)));
				}
				progressDialog->appendText(tr("Export finished to \"%1\"!").arg(path));
			}
			else
			{
				UERROR("DataRecorder init failed?!");
			}
		}
		else
		{
			QMessageBox::warning(this, tr("Cannot export database"), tr("An output path must be set!"));
		}
	}
}

void DatabaseViewer::extractImages()
{
	if(!dbDriver_ || ids_.size() == 0)
	{
		return;
	}

	QStringList formats;
	formats.push_back("jpg");
	formats.push_back("png");
	bool ok;
	QString ext = QInputDialog::getItem(this, tr("Which RGB format?"), tr("Format:"), formats, 0, false, &ok);
	if(!ok)
	{
		return;
	}

	QString path = QFileDialog::getExistingDirectory(this, tr("Select directory where to save images..."), QDir::homePath());
	if(!path.isNull())
	{
		if(ids_.size())
		{
			int id = ids_.at(0);
			SensorData data;
			dbDriver_->getNodeData(id, data);
			data.uncompressData();
			if(!data.imageRaw().empty() && !data.rightRaw().empty())
			{
				QDir dir;
				dir.mkdir(QString("%1/left").arg(path));
				dir.mkdir(QString("%1/right").arg(path));
				if(databaseFileName_.empty())
				{
					UERROR("Cannot save calibration file, database name is empty!");
				}
				else if(data.stereoCameraModel().isValidForProjection())
				{
					std::string cameraName = uSplit(databaseFileName_, '.').front();
					StereoCameraModel model(
							cameraName,
							data.imageRaw().size(),
							data.stereoCameraModel().left().K(),
							data.stereoCameraModel().left().D(),
							data.stereoCameraModel().left().R(),
							data.stereoCameraModel().left().P(),
							data.rightRaw().size(),
							data.stereoCameraModel().right().K(),
							data.stereoCameraModel().right().D(),
							data.stereoCameraModel().right().R(),
							data.stereoCameraModel().right().P(),
							data.stereoCameraModel().R(),
							data.stereoCameraModel().T(),
							data.stereoCameraModel().E(),
							data.stereoCameraModel().F(),
							data.stereoCameraModel().left().localTransform());
					if(model.save(path.toStdString()))
					{
						UINFO("Saved stereo calibration \"%s\"", (path.toStdString()+"/"+cameraName).c_str());
					}
					else
					{
						UERROR("Failed saving calibration \"%s\"", (path.toStdString()+"/"+cameraName).c_str());
					}
				}
			}
			else if(!data.imageRaw().empty())
			{
				if(!data.depthRaw().empty())
				{
					QDir dir;
					dir.mkdir(QString("%1/rgb").arg(path));
					dir.mkdir(QString("%1/depth").arg(path));
				}

				if(databaseFileName_.empty())
				{
					UERROR("Cannot save calibration file, database name is empty!");
				}
				else if(data.cameraModels().size() > 1)
				{
					UERROR("Only one camera calibration can be saved at this time (%d detected)", (int)data.cameraModels().size());
				}
				else if(data.cameraModels().size() == 1 && data.cameraModels().front().isValidForProjection())
				{
					std::string cameraName = uSplit(databaseFileName_, '.').front();
					CameraModel model(cameraName,
							data.imageRaw().size(),
							data.cameraModels().front().K(),
							data.cameraModels().front().D(),
							data.cameraModels().front().R(),
							data.cameraModels().front().P(),
							data.cameraModels().front().localTransform());
					if(model.save(path.toStdString()))
					{
						UINFO("Saved calibration \"%s\"", (path.toStdString()+"/"+cameraName).c_str());
					}
					else
					{
						UERROR("Failed saving calibration \"%s\"", (path.toStdString()+"/"+cameraName).c_str());
					}
				}
			}
		}

		for(int i=0; i<ids_.size(); ++i)
		{
			int id = ids_.at(i);
			SensorData data;
			dbDriver_->getNodeData(id, data);
			data.uncompressData();
			if(!data.imageRaw().empty() && !data.rightRaw().empty())
			{
				cv::imwrite(QString("%1/left/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.imageRaw());
				cv::imwrite(QString("%1/right/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.rightRaw());
				UINFO(QString("Saved left/%1.%2 and right/%1.%2").arg(id).arg(ext).toStdString().c_str());
			}
			else if(!data.imageRaw().empty() && !data.depthRaw().empty())
			{
				cv::imwrite(QString("%1/rgb/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.imageRaw());
				cv::imwrite(QString("%1/depth/%2.png").arg(path).arg(id).toStdString(), data.depthRaw());
				UINFO(QString("Saved rgb/%1.%2 and depth/%1.png").arg(id).arg(ext).toStdString().c_str());
			}
			else if(!data.imageRaw().empty())
			{
				cv::imwrite(QString("%1/%2.%3").arg(path).arg(id).arg(ext).toStdString(), data.imageRaw());
				UINFO(QString("Saved %1.%2").arg(id).arg(ext).toStdString().c_str());
			}
		}
	}
}

void DatabaseViewer::updateIds()
{
	if(!dbDriver_)
	{
		return;
	}

	UINFO("Loading all IDs...");
	std::set<int> ids;
	dbDriver_->getAllNodeIds(ids);
	ids_ = QList<int>::fromStdList(std::list<int>(ids.begin(), ids.end()));
	idToIndex_.clear();
	mapIds_.clear();
	poses_.clear();
	groundTruthPoses_.clear();
	links_.clear();
	linksAdded_.clear();
	linksRefined_.clear();
	linksRemoved_.clear();
	ui_->label_optimizeFrom->setText(tr("Optimize from"));
	std::multimap<int, Link> links;
	dbDriver_->getAllLinks(links, true);
	UDEBUG("%d total links loaded", (int)links.size());
	double totalOdom = 0.0;
	Transform previousPose;
	int sessions = ids_.size()?1:0;
	double totalTime = 0.0;
	double previousStamp = 0.0;
	std::set<int> idsWithoutBad;
	dbDriver_->getAllNodeIds(idsWithoutBad, false, true);
	int badcountInLTM = 0;
	int badCountInGraph = 0;
	for(int i=0; i<ids_.size(); ++i)
	{
		idToIndex_.insert(ids_[i], i);

		Transform p, g;
		int w;
		std::string l;
		double s;
		int mapId;
		dbDriver_->getNodeInfo(ids_[i], p, mapId, w, l, s, g);
		mapIds_.insert(std::make_pair(ids_[i], mapId));

		if(i>0)
		{
			if(mapIds_.at(ids_[i-1]) == mapId)
			{
				if(!p.isNull() && !previousPose.isNull())
				{
					totalOdom += p.getDistance(previousPose);
				}

				if(previousStamp > 0.0 && s > 0.0)
				{
					totalTime += s-previousStamp;
				}
			}
			else
			{
				++sessions;
			}
		}
		previousStamp=s;
		previousPose=p;

		//links
		bool addPose = false;
		for(std::multimap<int, Link>::iterator jter=links.find(ids_[i]); jter!=links.end() && jter->first == ids_[i]; ++jter)
		{
			std::multimap<int, Link>::iterator invertedLinkIter = graph::findLink(links, jter->second.to(), jter->second.from(), false);
			if(	jter->second.isValid() && // null transform means a rehearsed location
				ids.find(jter->second.from()) != ids.end() &&
				ids.find(jter->second.to()) != ids.end() &&
				graph::findLink(links_, jter->second.from(), jter->second.to()) == links_.end() &&
				graph::findLink(links, jter->second.from(), jter->second.to(), false) != links.end() &&
				invertedLinkIter != links.end())
			{
				// check if user_data is set in opposite direction
				if(jter->second.userDataCompressed().cols == 0 &&
				   invertedLinkIter->second.userDataCompressed().cols != 0)
				{
					links_.insert(std::make_pair(invertedLinkIter->second.from(), invertedLinkIter->second));
				}
				else
				{
					links_.insert(std::make_pair(ids_[i], jter->second));
				}
				addPose = true;
			}
			else if(graph::findLink(links_, jter->second.from(), jter->second.to()) != links_.end())
			{
				addPose = true;
			}
		}
		if(addPose)
		{
			poses_.insert(std::make_pair(ids_[i], p));
			if(!g.isNull())
			{
				groundTruthPoses_.insert(std::make_pair(ids_[i], g));
			}
		}

		if(idsWithoutBad.find(ids_[i]) == idsWithoutBad.end())
		{
			++badcountInLTM;
			if(addPose)
			{
				++badCountInGraph;
			}
		}
	}
	UINFO("Loaded %d ids, %d poses and %d links", (int)ids_.size(), (int)poses_.size(), (int)links_.size());

	UINFO("Update database info...");
	ui_->textEdit_info->clear();
	ui_->textEdit_info->append(tr("Version:\t\t%1").arg(dbDriver_->getDatabaseVersion().c_str()));
	ui_->textEdit_info->append(tr("Sessions:\t\t%1").arg(sessions));
	ui_->textEdit_info->append(tr("Total odometry length:\t%1 m").arg(totalOdom));
	ui_->textEdit_info->append(tr("Total time:\t\t%1").arg(QDateTime::fromMSecsSinceEpoch(totalTime*1000).toUTC().toString("hh:mm:ss.zzz")));
	ui_->textEdit_info->append(tr("LTM:\t\t%1 nodes and %2 words").arg(ids.size()).arg(dbDriver_->getTotalDictionarySize()));
	ui_->textEdit_info->append(tr("WM:\t\t%1 nodes and %2 words").arg(dbDriver_->getLastNodesSize()).arg(dbDriver_->getLastDictionarySize()));
	ui_->textEdit_info->append(tr("Global graph:\t%1 poses and %2 links").arg(poses_.size()).arg(links_.size()));
	ui_->textEdit_info->append(tr("Ground truth:\t%1 poses").arg(groundTruthPoses_.size()));
	ui_->textEdit_info->append("");
	ui_->textEdit_info->append(tr("Database size:\t%1 MB").arg(dbDriver_->getMemoryUsed()/1000000));
	ui_->textEdit_info->append(tr("Images size:\t%1 MB").arg(dbDriver_->getImagesMemoryUsed()/1000000));
	ui_->textEdit_info->append(tr("Depths size:\t%1 MB").arg(dbDriver_->getDepthImagesMemoryUsed()/1000000));
	ui_->textEdit_info->append(tr("Scans size:\t\t%1 MB").arg(dbDriver_->getLaserScansMemoryUsed()/1000000));
	ui_->textEdit_info->append(tr("User data size:\t%1 bytes").arg(dbDriver_->getUserDataMemoryUsed()));
	ui_->textEdit_info->append(tr("Dictionary size:\t%1 bytes").arg(dbDriver_->getWordsMemoryUsed()));
	ui_->textEdit_info->append("");
	ui_->textEdit_info->append(tr("%1 bad signatures in LTM").arg(badcountInLTM));
	ui_->textEdit_info->append(tr("%1 bad signatures in the global graph").arg(badCountInGraph));

	if(ids.size())
	{
		if(poses_.size())
		{
			bool nullPoses = poses_.begin()->second.isNull();
			for(std::map<int,Transform>::iterator iter=poses_.begin(); iter!=poses_.end(); ++iter)
			{
				if((!iter->second.isNull() && nullPoses) ||
					(iter->second.isNull() && !nullPoses))
				{
					if(iter->second.isNull())
					{
						UWARN("Pose %d is null!", iter->first);
					}
					UWARN("Mixed valid and null poses! Ignoring graph...");
					poses_.clear();
					links_.clear();
					break;
				}
			}
			if(nullPoses)
			{
				poses_.clear();
				links_.clear();
			}

			if(poses_.size())
			{
				ui_->spinBox_optimizationsFrom->setRange(poses_.begin()->first, poses_.rbegin()->first);
				ui_->spinBox_optimizationsFrom->setValue(poses_.begin()->first);
				ui_->label_optimizeFrom->setText(tr("Optimize from [%1, %2]").arg(poses_.begin()->first).arg(poses_.rbegin()->first));
			}
		}
	}

	ui_->actionGenerate_TORO_graph_graph->setEnabled(false);
	ui_->actionGenerate_g2o_graph_g2o->setEnabled(false);
	graphes_.clear();
	graphLinks_.clear();
	neighborLinks_.clear();
	loopLinks_.clear();
	for(std::multimap<int, rtabmap::Link>::iterator iter = links_.begin(); iter!=links_.end(); ++iter)
	{
		if(!iter->second.transform().isNull())
		{
			if(iter->second.type() == rtabmap::Link::kNeighbor ||
			   iter->second.type() == rtabmap::Link::kNeighborMerged)
			{
				neighborLinks_.append(iter->second);
			}
			else
			{
				loopLinks_.append(iter->second);
			}
		}
		else
		{
			UERROR("Transform null for link from %d to %d", iter->first, iter->second.to());
		}
	}

	if(ids_.size())
	{
		ui_->horizontalSlider_A->setMinimum(0);
		ui_->horizontalSlider_B->setMinimum(0);
		ui_->horizontalSlider_A->setMaximum(ids_.size()-1);
		ui_->horizontalSlider_B->setMaximum(ids_.size()-1);
		ui_->horizontalSlider_A->setEnabled(true);
		ui_->horizontalSlider_B->setEnabled(true);
		ui_->horizontalSlider_A->setSliderPosition(0);
		ui_->horizontalSlider_B->setSliderPosition(0);
		sliderAValueChanged(0);
		sliderBValueChanged(0);
	}
	else
	{
		ui_->horizontalSlider_A->setEnabled(false);
		ui_->horizontalSlider_B->setEnabled(false);
		ui_->label_idA->setText("NaN");
		ui_->label_idB->setText("NaN");
	}

	if(neighborLinks_.size())
	{
		ui_->horizontalSlider_neighbors->setMinimum(0);
		ui_->horizontalSlider_neighbors->setMaximum(neighborLinks_.size()-1);
		ui_->horizontalSlider_neighbors->setEnabled(true);
		ui_->horizontalSlider_neighbors->setSliderPosition(0);
	}
	else
	{
		ui_->horizontalSlider_neighbors->setEnabled(false);
	}

	if(ids_.size())
	{
		updateLoopClosuresSlider();
		if(ui_->graphViewer->isVisible())
		{
			updateGraphView();
		}
	}
}

void DatabaseViewer::generateGraph()
{
	if(!dbDriver_)
	{
		QMessageBox::warning(this, tr("Cannot generate a graph"), tr("A database must must loaded first...\nUse File->Open database."));
		return;
	}

	QString path = QFileDialog::getSaveFileName(this, tr("Save File"), pathDatabase_+"/Graph.dot", tr("Graphiz file (*.dot)"));
	if(!path.isEmpty())
	{
		dbDriver_->generateGraph(path.toStdString());
	}
}

void DatabaseViewer::generateLocalGraph()
{
	if(!ids_.size() || !dbDriver_)
	{
		QMessageBox::warning(this, tr("Cannot generate a graph"), tr("The database is empty..."));
		return;
	}
	bool ok = false;
	int id = QInputDialog::getInt(this, tr("Around which location?"), tr("Location ID"), ids_.first(), ids_.first(), ids_.last(), 1, &ok);

	if(ok)
	{
		int margin = QInputDialog::getInt(this, tr("Depth around the location?"), tr("Margin"), 4, 1, 100, 1, &ok);
		if(ok)
		{
			QString path = QFileDialog::getSaveFileName(this, tr("Save File"), pathDatabase_+"/Graph" + QString::number(id) + ".dot", tr("Graphiz file (*.dot)"));
			if(!path.isEmpty() && id>0)
			{
				std::map<int, int> ids;
				std::list<int> curentMarginList;
				std::set<int> currentMargin;
				std::set<int> nextMargin;
				nextMargin.insert(id);
				int m = 0;
				while((margin == 0 || m < margin) && nextMargin.size())
				{
					curentMarginList = std::list<int>(nextMargin.rbegin(), nextMargin.rend());
					nextMargin.clear();

					for(std::list<int>::iterator jter = curentMarginList.begin(); jter!=curentMarginList.end(); ++jter)
					{
						if(ids.find(*jter) == ids.end())
						{
							std::map<int, Link> links;
							ids.insert(std::pair<int, int>(*jter, m));

							UTimer timer;
							dbDriver_->loadLinks(*jter, links);

							// links
							for(std::map<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
							{
								if( !uContains(ids, iter->first))
								{
									UASSERT(iter->second.type() != Link::kUndef);
									if(iter->second.type() == Link::kNeighbor ||
									   iter->second.type() == Link::kNeighborMerged)
									{
										nextMargin.insert(iter->first);
									}
									else
									{
										// loop closures are on same margin
										if(currentMargin.insert(iter->first).second)
										{
											curentMarginList.push_back(iter->first);
										}
									}
								}
							}
						}
					}
					++m;
				}

				if(ids.size() > 0)
				{
					ids.insert(std::pair<int,int>(id, 0));
					std::set<int> idsSet;
					for(std::map<int, int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
					{
						idsSet.insert(idsSet.end(), iter->first);
						UINFO("Node %d", iter->first);
					}
					UINFO("idsSet=%d", idsSet.size());
					dbDriver_->generateGraph(path.toStdString(), idsSet);
				}
				else
				{
					QMessageBox::critical(this, tr("Error"), tr("No neighbors found for signature %1.").arg(id));
				}
			}
		}
	}
}

void DatabaseViewer::generateTOROGraph()
{
	if(graphes_.empty())
	{
		this->updateGraphView();
		if(graphes_.empty() || ui_->horizontalSlider_iterations->maximum() != (int)graphes_.size()-1)
		{
			QMessageBox::warning(this, tr("Cannot generate a graph"), tr("No graph in database?!"));
			return;
		}
	}

	if(!graphes_.size() || !graphLinks_.size())
	{
		QMessageBox::warning(this, tr("Cannot generate a TORO graph"), tr("No poses or no links..."));
		return;
	}
	bool ok = false;
	int id = QInputDialog::getInt(this, tr("Which iteration?"), tr("Iteration (0 -> %1)").arg((int)graphes_.size()-1), (int)graphes_.size()-1, 0, (int)graphes_.size()-1, 1, &ok);

	if(ok)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save File"), pathDatabase_+"/constraints" + QString::number(id) + ".graph", tr("TORO file (*.graph)"));
		if(!path.isEmpty())
		{
			bool varianceIgnored = uStr2Bool(ui_->parameters_toolbox->getParameters().at(Parameters::kOptimizerVarianceIgnored()));
			if(varianceIgnored)
			{
				std::multimap<int, rtabmap::Link> links = graphLinks_;
				for(std::multimap<int, rtabmap::Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
				{
					iter->second.setInfMatrix(cv::Mat::eye(6,6,CV_64FC1));
				}
				graph::exportPoses(path.toStdString(), 3, uValueAt(graphes_, id), links);
			}
			else
			{
				graph::exportPoses(path.toStdString(), 3, uValueAt(graphes_, id), graphLinks_);
			}
		}
	}
}

void DatabaseViewer::generateG2OGraph()
{
	if(graphes_.empty())
	{
		this->updateGraphView();
		if(graphes_.empty() || ui_->horizontalSlider_iterations->maximum() != (int)graphes_.size()-1)
		{
			QMessageBox::warning(this, tr("Cannot generate a graph"), tr("No graph in database?!"));
			return;
		}
	}

	if(!graphes_.size() || !graphLinks_.size())
	{
		QMessageBox::warning(this, tr("Cannot generate a g2o graph"), tr("No poses or no links..."));
		return;
	}
	bool ok = false;
	int id = QInputDialog::getInt(this, tr("Which iteration?"), tr("Iteration (0 -> %1)").arg((int)graphes_.size()-1), (int)graphes_.size()-1, 0, (int)graphes_.size()-1, 1, &ok);

	if(ok)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save File"), pathDatabase_+"/constraints" + QString::number(id) + ".g2o", tr("g2o file (*.g2o)"));
		if(!path.isEmpty())
		{
			bool varianceIgnored = uStr2Bool(ui_->parameters_toolbox->getParameters().at(Parameters::kOptimizerVarianceIgnored()));
			bool robust = uStr2Bool(ui_->parameters_toolbox->getParameters().at(Parameters::kOptimizerRobust()));
			if(varianceIgnored)
			{
				std::multimap<int, rtabmap::Link> links = graphLinks_;
				for(std::multimap<int, rtabmap::Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
				{
					iter->second.setInfMatrix(cv::Mat::eye(6,6,CV_64FC1));
				}
				graph::exportPoses(path.toStdString(), 4, uValueAt(graphes_, id), links, std::map<int, double>(), robust);
			}
			else
			{
				graph::exportPoses(path.toStdString(), 4, uValueAt(graphes_, id), graphLinks_, std::map<int, double>(), robust);
			}
		}
	}
}

void DatabaseViewer::view3DMap()
{
	if(!ids_.size() || !dbDriver_)
	{
		QMessageBox::warning(this, tr("Cannot view 3D map"), tr("The database is empty..."));
		return;
	}

	if(graphes_.empty())
	{
		this->updateGraphView();
		if(graphes_.empty() || ui_->horizontalSlider_iterations->maximum() != (int)graphes_.size()-1)
		{
			QMessageBox::warning(this, tr("Cannot generate a graph"), tr("No graph in database?!"));
			return;
		}
	}
	bool ok = false;
	QStringList items;
	items.append("1");
	items.append("2");
	items.append("4");
	items.append("8");
	items.append("16");
	QString item = QInputDialog::getItem(this, tr("Decimation?"), tr("Image decimation"), items, 2, false, &ok);
	if(ok)
	{
		int decimation = item.toInt();
		double maxDepth = QInputDialog::getDouble(this, tr("Camera depth?"), tr("Maximum depth (m, 0=no max):"), 4.0, 0, 100, 2, &ok);
		if(ok)
		{
			std::map<int, Transform> optimizedPoses = uValueAt(graphes_, ui_->horizontalSlider_iterations->value());
			if(ui_->groupBox_posefiltering->isChecked())
			{
				optimizedPoses = graph::radiusPosesFiltering(optimizedPoses,
						ui_->doubleSpinBox_posefilteringRadius->value(),
						ui_->doubleSpinBox_posefilteringAngle->value()*CV_PI/180.0);
			}
			if(optimizedPoses.size() > 0)
			{
				rtabmap::ProgressDialog progressDialog(this);
				progressDialog.setMaximumSteps((int)optimizedPoses.size());
				progressDialog.show();

				// create a window
				QDialog * window = new QDialog(this, Qt::Window);
				window->setModal(this->isModal());
				window->setWindowTitle(tr("3D Map"));
				window->setMinimumWidth(800);
				window->setMinimumHeight(600);

				rtabmap::CloudViewer * viewer = new rtabmap::CloudViewer(window);

				QVBoxLayout *layout = new QVBoxLayout();
				layout->addWidget(viewer);
				viewer->setCameraLockZ(false);
				window->setLayout(layout);
				connect(window, SIGNAL(finished(int)), viewer, SLOT(clear()));

				window->show();

				for(std::map<int, Transform>::const_iterator iter = optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
				{
					rtabmap::Transform pose = iter->second;
					if(!pose.isNull())
					{
						SensorData data;
						dbDriver_->getNodeData(iter->first, data);
						data.uncompressData();
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
						UASSERT(data.imageRaw().empty() || data.imageRaw().type()==CV_8UC3 || data.imageRaw().type() == CV_8UC1);
						UASSERT(data.depthOrRightRaw().empty() || data.depthOrRightRaw().type()==CV_8UC1 || data.depthOrRightRaw().type() == CV_16UC1 || data.depthOrRightRaw().type() == CV_32FC1);
						cloud = util3d::cloudRGBFromSensorData(data, decimation, maxDepth, 0, 0, ui_->parameters_toolbox->getParameters());

						if(cloud->size())
						{
							QColor color = Qt::red;
							int mapId, weight;
							Transform odomPose, groundTruth;
							std::string label;
							double stamp;
							if(dbDriver_->getNodeInfo(iter->first, odomPose, mapId, weight, label, stamp, groundTruth))
							{
								color = (Qt::GlobalColor)(mapId % 12 + 7 );
							}

							viewer->addCloud(uFormat("cloud%d", iter->first), cloud, pose, color);

							UINFO("Generated %d (%d points)", iter->first, cloud->size());
							progressDialog.appendText(QString("Generated %1 (%2 points)").arg(iter->first).arg(cloud->size()));
						}
						else
						{
							UINFO("Empty cloud %d", iter->first);
							progressDialog.appendText(QString("Empty cloud %1").arg(iter->first));
						}
						progressDialog.incrementStep();
						QApplication::processEvents();
					}
				}
				progressDialog.setValue(progressDialog.maximumSteps());
			}
			else
			{
				QMessageBox::critical(this, tr("Error"), tr("No neighbors found for node %1.").arg(ui_->spinBox_optimizationsFrom->value()));
			}
		}
	}
}

void DatabaseViewer::view3DLaserScans()
{
	if(!ids_.size() || !dbDriver_)
	{
		QMessageBox::warning(this, tr("Cannot view 3D laser scans"), tr("The database is empty..."));
		return;
	}

	if(graphes_.empty())
	{
		this->updateGraphView();
		if(graphes_.empty() || ui_->horizontalSlider_iterations->maximum() != (int)graphes_.size()-1)
		{
			QMessageBox::warning(this, tr("Cannot generate a graph"), tr("No graph in database?!"));
			return;
		}
	}
	bool ok = false;
	int downsamplingStepSize = QInputDialog::getInt(this, tr("Downsampling?"), tr("Downsample step size (1 = no filtering)"), 1, 1, 99999, 1, &ok);
	if(ok)
	{
		std::map<int, Transform> optimizedPoses = uValueAt(graphes_, ui_->horizontalSlider_iterations->value());
		if(ui_->groupBox_posefiltering->isChecked())
		{
			optimizedPoses = graph::radiusPosesFiltering(optimizedPoses,
					ui_->doubleSpinBox_posefilteringRadius->value(),
					ui_->doubleSpinBox_posefilteringAngle->value()*CV_PI/180.0);
		}
		if(optimizedPoses.size() > 0)
		{
			rtabmap::ProgressDialog progressDialog(this);
			progressDialog.setMaximumSteps((int)optimizedPoses.size());
			progressDialog.show();

			// create a window
			QDialog * window = new QDialog(this, Qt::Window);
			window->setModal(this->isModal());
			window->setWindowTitle(tr("3D Laser Scans"));
			window->setMinimumWidth(800);
			window->setMinimumHeight(600);

			rtabmap::CloudViewer * viewer = new rtabmap::CloudViewer(window);

			QVBoxLayout *layout = new QVBoxLayout();
			layout->addWidget(viewer);
			viewer->setCameraLockZ(false);
			window->setLayout(layout);
			connect(window, SIGNAL(finished(int)), viewer, SLOT(clear()));

			window->show();

			for(std::map<int, Transform>::const_iterator iter = optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
			{
				rtabmap::Transform pose = iter->second;
				if(!pose.isNull())
				{
					SensorData data;
					dbDriver_->getNodeData(iter->first, data);
					cv::Mat scan;
					data.uncompressDataConst(0, 0, &scan);
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
					UASSERT(scan.empty() || scan.type()==CV_32FC2 || scan.type() == CV_32FC3);

					if(downsamplingStepSize>1)
					{
						scan = util3d::downsample(scan, downsamplingStepSize);
					}
					cloud = util3d::laserScanToPointCloud(scan);

					if(cloud->size())
					{
						QColor color = Qt::red;
						int mapId, weight;
						Transform odomPose, groundTruth;
						std::string label;
						double stamp;
						if(dbDriver_->getNodeInfo(iter->first, odomPose, mapId, weight, label, stamp, groundTruth))
						{
							color = (Qt::GlobalColor)(mapId % 12 + 7 );
						}

						int normalK = uStr2Int(ui_->parameters_toolbox->getParameters().at(Parameters::kIcpPointToPlaneNormalNeighbors()));
						pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, normalK);
						pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);
						pcl::concatenateFields(*cloud, *normals, *cloudNormals);

						viewer->addCloud(uFormat("cloud%d", iter->first), cloudNormals, pose, color);

						UINFO("Generated %d (%d points)", iter->first, cloud->size());
						progressDialog.appendText(QString("Generated %1 (%2 points)").arg(iter->first).arg(cloud->size()));
					}
					else
					{
						UINFO("Empty cloud %d", iter->first);
						progressDialog.appendText(QString("Empty cloud %1").arg(iter->first));
					}
					progressDialog.incrementStep();
					QApplication::processEvents();
				}
			}
			progressDialog.setValue(progressDialog.maximumSteps());
		}
		else
		{
			QMessageBox::critical(this, tr("Error"), tr("No neighbors found for node %1.").arg(ui_->spinBox_optimizationsFrom->value()));
		}
	}
}

void DatabaseViewer::generate3DMap()
{
	if(!ids_.size() || !dbDriver_)
	{
		QMessageBox::warning(this, tr("Cannot generate a graph"), tr("The database is empty..."));
		return;
	}

	if(graphes_.empty())
	{
		this->updateGraphView();
		if(graphes_.empty() || ui_->horizontalSlider_iterations->maximum() != (int)graphes_.size()-1)
		{
			QMessageBox::warning(this, tr("Cannot generate a graph"), tr("No graph in database?!"));
			return;
		}
	}

	bool ok = false;
	QStringList items;
	items.append("1");
	items.append("2");
	items.append("4");
	items.append("8");
	items.append("16");
	QString item = QInputDialog::getItem(this, tr("Decimation?"), tr("Image decimation"), items, 2, false, &ok);
	if(ok)
	{
		int decimation = item.toInt();
		double maxDepth = QInputDialog::getDouble(this, tr("Camera depth?"), tr("Maximum depth (m, 0=no max):"), 4.0, 0, 100, 2, &ok);
		if(ok)
		{
			QMessageBox::StandardButton b = QMessageBox::question(
					this,
					tr("Assembling?"),
					tr("Do you want to assemble all the point clouds (creating only one file with a density of 1pt/cm)?"),
					QMessageBox::Yes|QMessageBox::No,
					QMessageBox::Yes);

			bool assemble = b == QMessageBox::Yes;
			QString path;
			if(assemble)
			{
				path = QFileDialog::getSaveFileName(this, tr("Save point cloud"),
						pathDatabase_+QDir::separator()+"cloud.ply",
						tr("Point Cloud (*.ply *.pcd)"));
			}
			else
			{
				path = QFileDialog::getExistingDirectory(this, tr("Save directory"), pathDatabase_);
			}
			if(!path.isEmpty())
			{
				std::map<int, Transform> optimizedPoses = uValueAt(graphes_, ui_->horizontalSlider_iterations->value());
				if(ui_->groupBox_posefiltering->isChecked())
				{
					optimizedPoses = graph::radiusPosesFiltering(optimizedPoses,
							ui_->doubleSpinBox_posefilteringRadius->value(),
							ui_->doubleSpinBox_posefilteringAngle->value()*CV_PI/180.0);
				}
				if(optimizedPoses.size() > 0)
				{
					rtabmap::ProgressDialog progressDialog;
					progressDialog.setMaximumSteps((int)optimizedPoses.size());
					progressDialog.show();

					pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					for(std::map<int, Transform>::const_iterator iter = optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
					{
						const rtabmap::Transform & pose = iter->second;
						if(!pose.isNull())
						{
							SensorData data;
							dbDriver_->getNodeData(iter->first, data);
							data.uncompressData();
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
							UASSERT(data.imageRaw().empty() || data.imageRaw().type()==CV_8UC3 || data.imageRaw().type() == CV_8UC1);
							UASSERT(data.depthOrRightRaw().empty() || data.depthOrRightRaw().type()==CV_8UC1 || data.depthOrRightRaw().type() == CV_16UC1 || data.depthOrRightRaw().type() == CV_32FC1);
							pcl::IndicesPtr validIndices(new std::vector<int>);
							cloud = util3d::cloudRGBFromSensorData(data, decimation, maxDepth, 0, validIndices.get(), ui_->parameters_toolbox->getParameters());

							if(assemble)
							{
								if(cloud->size())
								{
									cloud = util3d::voxelize(cloud, validIndices, 0.01);

									if(cloud->size())
									{
										cloud = rtabmap::util3d::transformPointCloud(cloud, pose);
										if(assembledCloud->size() == 0)
										{
											*assembledCloud = *cloud;
										}
										else
										{
											*assembledCloud += *cloud;
										}
									}
								}
								UINFO("Created cloud %d (%d points)", iter->first, (int)cloud->size());
								progressDialog.appendText(QString("Created cloud %1 (%2 points)").arg(iter->first).arg(cloud->size()));
							}
							else
							{
								std::string name = uFormat("%s/node%d.pcd", path.toStdString().c_str(), iter->first);
								if(cloud->size())
								{
									cloud = rtabmap::util3d::transformPointCloud(cloud, pose);
									pcl::io::savePCDFile(name, *cloud);
									UINFO("Saved %s (%d points)", name.c_str(), cloud->size());
									progressDialog.appendText(QString("Saved %1 (%2 points)").arg(name.c_str()).arg(cloud->size()));
								}
								else
								{
									UINFO("Ignored empty cloud %s", name.c_str());
									progressDialog.appendText(QString("Ignored empty cloud %1").arg(name.c_str()));
								}
							}
							progressDialog.incrementStep();
							QApplication::processEvents();
						}
					}

					if(assemble && assembledCloud->size())
					{
						//voxelize by default to 1 cm
						progressDialog.appendText(QString("Voxelize assembled cloud (%1 points)").arg(assembledCloud->size()));
						QApplication::processEvents();
						assembledCloud = util3d::voxelize(assembledCloud, 0.01);
						if(QFileInfo(path).suffix() == "ply")
						{
							pcl::io::savePLYFile(path.toStdString(), *assembledCloud);
						}
						else
						{
							pcl::io::savePCDFile(path.toStdString(), *assembledCloud);
						}
						progressDialog.appendText(QString("Saved %1 (%2 points)").arg(path).arg(assembledCloud->size()));
						QApplication::processEvents();
					}
					QMessageBox::information(this, tr("Finished"), tr("%1 clouds generated to %2.").arg(optimizedPoses.size()).arg(path));

					progressDialog.setValue(progressDialog.maximumSteps());
				}
				else
				{
					QMessageBox::critical(this, tr("Error"), tr("No neighbors found for node %1.").arg(ui_->spinBox_optimizationsFrom->value()));
				}
			}
		}
	}
}

void DatabaseViewer::generate3DLaserScans()
{
	if(!ids_.size() || !dbDriver_)
	{
		QMessageBox::warning(this, tr("Cannot generate a graph"), tr("The database is empty..."));
		return;
	}
	bool ok = false;
	int downsamplingStepSize = QInputDialog::getInt(this, tr("Downsampling?"), tr("Downsample step size (1 = no filtering)"), 1, 1, 99999, 1, &ok);
	if(ok)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save point cloud"),
					pathDatabase_+QDir::separator()+"cloud.ply",
					tr("Point Cloud (*.ply *.pcd)"));
		if(!path.isEmpty())
		{
			std::map<int, Transform> optimizedPoses = uValueAt(graphes_, ui_->horizontalSlider_iterations->value());
			if(ui_->groupBox_posefiltering->isChecked())
			{
				optimizedPoses = graph::radiusPosesFiltering(optimizedPoses,
						ui_->doubleSpinBox_posefilteringRadius->value(),
						ui_->doubleSpinBox_posefilteringAngle->value()*CV_PI/180.0);
			}
			if(optimizedPoses.size() > 0)
			{
				rtabmap::ProgressDialog progressDialog;
				progressDialog.setMaximumSteps((int)optimizedPoses.size());
				progressDialog.show();

				pcl::PointCloud<pcl::PointXYZ>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZ>);
				for(std::map<int, Transform>::const_iterator iter = optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
				{
					const rtabmap::Transform & pose = iter->second;
					if(!pose.isNull())
					{
						SensorData data;
						dbDriver_->getNodeData(iter->first, data);
						cv::Mat scan;
						data.uncompressDataConst(0, 0, &scan);
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
						UASSERT(scan.empty() || scan.type()==CV_32FC2 || scan.type() == CV_32FC3);

						if(downsamplingStepSize > 1)
						{
							scan = util3d::downsample(scan, downsamplingStepSize);
						}
						cloud = util3d::laserScanToPointCloud(scan);

						if(cloud->size())
						{
							cloud = rtabmap::util3d::transformPointCloud(cloud, pose);
							if(assembledCloud->size() == 0)
							{
								*assembledCloud = *cloud;
							}
							else
							{
								*assembledCloud += *cloud;
							}
						}
						UINFO("Created cloud %d (%d points)", iter->first, (int)cloud->size());
						progressDialog.appendText(QString("Created cloud %1 (%2 points)").arg(iter->first).arg(cloud->size()));

						progressDialog.incrementStep();
						QApplication::processEvents();
					}
				}

				if(assembledCloud->size())
				{
					//voxelize by default to 1 cm
					progressDialog.appendText(QString("Voxelize assembled cloud (%1 points)").arg(assembledCloud->size()));
					QApplication::processEvents();
					assembledCloud = util3d::voxelize(assembledCloud, 0.01);
					if(QFileInfo(path).suffix() == "ply")
					{
						pcl::io::savePLYFile(path.toStdString(), *assembledCloud);
					}
					else
					{
						pcl::io::savePCDFile(path.toStdString(), *assembledCloud);
					}
					progressDialog.appendText(QString("Saved %1 (%2 points)").arg(path).arg(assembledCloud->size()));
					QApplication::processEvents();
				}
				QMessageBox::information(this, tr("Finished"), tr("%1 clouds generated to %2.").arg(optimizedPoses.size()).arg(path));

				progressDialog.setValue(progressDialog.maximumSteps());
			}
			else
			{
				QMessageBox::critical(this, tr("Error"), tr("No neighbors found for node %1.").arg(ui_->spinBox_optimizationsFrom->value()));
			}
		}
	}
}

void DatabaseViewer::detectMoreLoopClosures()
{
	if(graphes_.empty())
	{
		this->updateGraphView();
		if(graphes_.empty() || ui_->horizontalSlider_iterations->maximum() != (int)graphes_.size()-1)
		{
			QMessageBox::warning(this, tr("Cannot generate a graph"), tr("No graph in database?!"));
			return;
		}
	}

	const std::map<int, Transform> & optimizedPoses = graphes_.back();

	int iterations = ui_->spinBox_detectMore_iterations->value();
	UASSERT(iterations > 0);
	int added = 0;
	for(int n=0; n<iterations; ++n)
	{
		UINFO("iteration %d/%d", n+1, iterations);
		std::multimap<int, int> clusters = rtabmap::graph::radiusPosesClustering(
				optimizedPoses,
				ui_->doubleSpinBox_detectMore_radius->value(),
				ui_->doubleSpinBox_detectMore_angle->value()*CV_PI/180.0);
		std::set<int> addedLinks;
		for(std::multimap<int, int>::iterator iter=clusters.begin(); iter!= clusters.end(); ++iter)
		{
			int from = iter->first;
			int to = iter->second;
			if(from < to)
			{
				from = iter->second;
				to = iter->first;
			}
			if(!findActiveLink(from, to).isValid() && !containsLink(linksRemoved_, from, to) &&
				addedLinks.find(from) == addedLinks.end() && addedLinks.find(to) == addedLinks.end())
			{
				if(addConstraint(from, to, true, false))
				{
					UINFO("Added new loop closure between %d and %d.", from, to);
					++added;
					addedLinks.insert(from);
					addedLinks.insert(to);
				}
			}
		}
		UINFO("Iteration %d/%d: added %d loop closures.", n+1, iterations, (int)addedLinks.size()/2);
		if(addedLinks.size() == 0)
		{
			break;
		}
	}
	if(added)
	{
		this->updateGraphView();
	}
	UINFO("Total added %d loop closures.", added);
}

void DatabaseViewer::refineAllNeighborLinks()
{
	if(neighborLinks_.size())
	{
		rtabmap::ProgressDialog progressDialog(this);
		progressDialog.setMaximumSteps(neighborLinks_.size());
		progressDialog.show();

		for(int i=0; i<neighborLinks_.size(); ++i)
		{
			int from = neighborLinks_[i].from();
			int to = neighborLinks_[i].to();
			this->refineConstraint(neighborLinks_[i].from(), neighborLinks_[i].to(), true, false);

			progressDialog.appendText(tr("Refined link %1->%2 (%3/%4)").arg(from).arg(to).arg(i+1).arg(neighborLinks_.size()));
			progressDialog.incrementStep();
			QApplication::processEvents();
		}
		this->updateGraphView();

		progressDialog.setValue(progressDialog.maximumSteps());
		progressDialog.appendText("Refining links finished!");
	}
}

void DatabaseViewer::refineAllLoopClosureLinks()
{
	if(loopLinks_.size())
	{
		rtabmap::ProgressDialog progressDialog(this);
		progressDialog.setMaximumSteps(loopLinks_.size());
		progressDialog.show();

		for(int i=0; i<loopLinks_.size(); ++i)
		{
			int from = loopLinks_[i].from();
			int to = loopLinks_[i].to();
			this->refineConstraint(loopLinks_[i].from(), loopLinks_[i].to(), true, false);

			progressDialog.appendText(tr("Refined link %1->%2 (%3/%4)").arg(from).arg(to).arg(i+1).arg(loopLinks_.size()));
			progressDialog.incrementStep();
			QApplication::processEvents();
		}
		this->updateGraphView();

		progressDialog.setValue(progressDialog.maximumSteps());
		progressDialog.appendText("Refining links finished!");
	}
}

void DatabaseViewer::refineVisuallyAllNeighborLinks()
{
	if(neighborLinks_.size())
	{
		rtabmap::ProgressDialog progressDialog(this);
		progressDialog.setMaximumSteps(neighborLinks_.size());
		progressDialog.show();

		for(int i=0; i<neighborLinks_.size(); ++i)
		{
			int from = neighborLinks_[i].from();
			int to = neighborLinks_[i].to();
			this->refineConstraintVisually(neighborLinks_[i].from(), neighborLinks_[i].to(), true, false);

			progressDialog.appendText(tr("Refined link %1->%2 (%3/%4)").arg(from).arg(to).arg(i+1).arg(neighborLinks_.size()));
			progressDialog.incrementStep();
			QApplication::processEvents();
		}
		this->updateGraphView();

		progressDialog.setValue(progressDialog.maximumSteps());
		progressDialog.appendText("Refining links finished!");
	}
}

void DatabaseViewer::refineVisuallyAllLoopClosureLinks()
{
	if(loopLinks_.size())
	{
		rtabmap::ProgressDialog progressDialog(this);
		progressDialog.setMaximumSteps(loopLinks_.size());
		progressDialog.show();

		for(int i=0; i<loopLinks_.size(); ++i)
		{
			int from = loopLinks_[i].from();
			int to = loopLinks_[i].to();
			this->refineConstraintVisually(loopLinks_[i].from(), loopLinks_[i].to(), true, false);

			progressDialog.appendText(tr("Refined link %1->%2 (%3/%4)").arg(from).arg(to).arg(i+1).arg(loopLinks_.size()));
			progressDialog.incrementStep();
			QApplication::processEvents();
		}
		this->updateGraphView();

		progressDialog.setValue(progressDialog.maximumSteps());
		progressDialog.appendText("Refining links finished!");
	}
}

void DatabaseViewer::resetAllChanges()
{
	linksAdded_.clear();
	linksRefined_.clear();
	linksRemoved_.clear();
	updateLoopClosuresSlider();
	this->updateGraphView();
}

void DatabaseViewer::sliderAValueChanged(int value)
{
	this->update(value,
			ui_->label_indexA,
			ui_->label_parentsA,
			ui_->label_childrenA,
			ui_->label_weightA,
			ui_->label_labelA,
			ui_->label_stampA,
			ui_->graphicsView_A,
			cloudViewerA_,
			ui_->label_idA,
			ui_->label_mapA,
			ui_->label_poseA,
			ui_->label_calibA,
			true);
}

void DatabaseViewer::sliderBValueChanged(int value)
{
	this->update(value,
			ui_->label_indexB,
			ui_->label_parentsB,
			ui_->label_childrenB,
			ui_->label_weightB,
			ui_->label_labelB,
			ui_->label_stampB,
			ui_->graphicsView_B,
			cloudViewerB_,
			ui_->label_idB,
			ui_->label_mapB,
			ui_->label_poseB,
			ui_->label_calibB,
			true);
}

void DatabaseViewer::update(int value,
						QLabel * labelIndex,
						QLabel * labelParents,
						QLabel * labelChildren,
						QLabel * weight,
						QLabel * label,
						QLabel * stamp,
						rtabmap::ImageView * view,
						rtabmap::CloudViewer * view3D,
						QLabel * labelId,
						QLabel * labelMapId,
						QLabel * labelPose,
						QLabel * labelCalib,
						bool updateConstraintView)
{
	UTimer timer;
	labelIndex->setText(QString::number(value));
	labelParents->clear();
	labelChildren->clear();
	weight->clear();
	label->clear();
	labelMapId->clear();
	labelPose->clear();
	stamp->clear();
	labelCalib->clear();
	QRectF rect;
	if(value >= 0 && value < ids_.size())
	{
		view->clear();
		int id = ids_.at(value);
		int mapId = -1;
		labelId->setText(QString::number(id));
		if(id>0)
		{
			//image
			QImage img;
			QImage imgDepth;
			if(dbDriver_)
			{
				SensorData data;
				dbDriver_->getNodeData(id, data);
				data.uncompressData();
				if(!data.imageRaw().empty())
				{
					img = uCvMat2QImage(ui_->label_indexB==labelIndex?data.imageRaw():data.imageRaw());
				}
				if(!data.depthOrRightRaw().empty())
				{
					cv::Mat depth =data.depthOrRightRaw();
					if(!data.depthRaw().empty())
					{
						if(ui_->spinBox_mesh_fillDepthHoles->value() > 0)
						{
							depth = util2d::fillDepthHoles(depth, ui_->spinBox_mesh_fillDepthHoles->value(), float(ui_->spinBox_mesh_depthError->value())/100.0f);
						}
					}
					imgDepth = uCvMat2QImage(depth);
				}

				std::list<int> ids;
				ids.push_back(id);
				std::list<Signature*> signatures;
				dbDriver_->loadSignatures(ids, signatures);

				if(signatures.size() && signatures.front()!=0 && signatures.front()->getWords().size())
				{
					view->setFeatures(signatures.front()->getWords(), data.depthOrRightRaw().type() == CV_8UC1?cv::Mat():data.depthOrRightRaw(), Qt::yellow);
					delete signatures.front();
					signatures.clear();
				}

				Transform odomPose, g;
				int w;
				std::string l;
				double s;
				dbDriver_->getNodeInfo(id, odomPose, mapId, w, l, s, g);

				weight->setNum(w);
				label->setText(l.c_str());
				float x,y,z,roll,pitch,yaw;
				odomPose.getTranslationAndEulerAngles(x,y,z,roll, pitch,yaw);
				labelPose->setText(QString("%1xyz=(%2,%3,%4)\nrpy=(%5,%6,%7)").arg(odomPose.isIdentity()?"* ":"").arg(x).arg(y).arg(z).arg(roll).arg(pitch).arg(yaw));
				if(s!=0.0)
				{
					stamp->setText(QDateTime::fromMSecsSinceEpoch(s*1000.0).toString("dd.MM.yyyy hh:mm:ss.zzz"));
				}
				if(data.cameraModels().size() || data.stereoCameraModel().isValidForProjection())
				{
					if(data.cameraModels().size())
					{
						if(!data.depthRaw().empty() && data.depthRaw().cols!=data.imageRaw().cols && data.imageRaw().cols)
						{
							labelCalib->setText(tr("%1 %2x%3 [%8x%9] fx=%4 fy=%5 cx=%6 cy=%7")
									.arg(data.cameraModels().size())
									.arg(data.cameraModels()[0].imageWidth()>0?data.cameraModels()[0].imageWidth():data.imageRaw().cols/data.cameraModels().size())
									.arg(data.cameraModels()[0].imageHeight()>0?data.cameraModels()[0].imageHeight():data.imageRaw().rows)
									.arg(data.cameraModels()[0].fx())
									.arg(data.cameraModels()[0].fy())
									.arg(data.cameraModels()[0].cx())
									.arg(data.cameraModels()[0].cy())
									.arg(data.depthRaw().cols/data.cameraModels().size())
									.arg(data.depthRaw().rows));
						}
						else
						{
							labelCalib->setText(tr("%1 %2x%3 fx=%4 fy=%5 cx=%6 cy=%7")
									.arg(data.cameraModels().size())
									.arg(data.cameraModels()[0].imageWidth()>0?data.cameraModels()[0].imageWidth():data.imageRaw().cols/data.cameraModels().size())
									.arg(data.cameraModels()[0].imageHeight()>0?data.cameraModels()[0].imageHeight():data.imageRaw().rows)
									.arg(data.cameraModels()[0].fx())
									.arg(data.cameraModels()[0].fy())
									.arg(data.cameraModels()[0].cx())
									.arg(data.cameraModels()[0].cy()));
						}
					}
					else
					{
						//stereo
						labelCalib->setText(tr("%1x%2 fx=%3 fy=%4 cx=%5 cy=%6 baseline=%7m")
									.arg(data.stereoCameraModel().left().imageWidth()>0?data.stereoCameraModel().left().imageWidth():data.imageRaw().cols)
									.arg(data.stereoCameraModel().left().imageHeight()>0?data.stereoCameraModel().left().imageHeight():data.imageRaw().rows)
									.arg(data.stereoCameraModel().left().fx())
									.arg(data.stereoCameraModel().left().fy())
									.arg(data.stereoCameraModel().left().cx())
									.arg(data.stereoCameraModel().left().cy())
									.arg(data.stereoCameraModel().baseline()));
					}

				}
				else
				{
					labelCalib->setText("NA");
				}

				//stereo
				if(!data.depthOrRightRaw().empty() && data.depthOrRightRaw().type() == CV_8UC1)
				{
					this->updateStereo(&data);
				}
				else
				{
					stereoViewer_->clear();
					ui_->graphicsView_stereo->clear();
				}

				// 3d view
				if(view3D->isVisible() &&
						(!data.depthOrRightRaw().empty() ||
						!data.laserScanRaw().empty()))
				{
					if(!data.depthOrRightRaw().empty())
					{
						if(!data.imageRaw().empty())
						{
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
							if(!data.depthRaw().empty() && data.cameraModels().size()==1)
							{
								cv::Mat depth = data.depthRaw();
								if(ui_->spinBox_mesh_fillDepthHoles->value() > 0)
								{
									depth = util2d::fillDepthHoles(depth, ui_->spinBox_mesh_fillDepthHoles->value(), float(ui_->spinBox_mesh_depthError->value())/100.0f);
								}
								cloud = util3d::cloudFromDepthRGB(
										data.imageRaw(),
										depth,
										data.cameraModels()[0]);
							}
							else
							{
								cloud = util3d::cloudRGBFromSensorData(data, 1, 0, 0, 0, ui_->parameters_toolbox->getParameters());
							}
							if(cloud->size())
							{
								if(ui_->checkBox_showMesh->isChecked() && !cloud->is_dense)
								{
									Eigen::Vector3f viewpoint(0.0f,0.0f,0.0f);
									if(data.cameraModels().size() && !data.cameraModels()[0].localTransform().isNull())
									{
										viewpoint[0] = data.cameraModels()[0].localTransform().x();
										viewpoint[1] = data.cameraModels()[0].localTransform().y();
										viewpoint[2] = data.cameraModels()[0].localTransform().z();
									}
									else if(!data.stereoCameraModel().localTransform().isNull())
									{
										viewpoint[0] = data.stereoCameraModel().localTransform().x();
										viewpoint[1] = data.stereoCameraModel().localTransform().y();
										viewpoint[2] = data.stereoCameraModel().localTransform().z();
									}
									std::vector<pcl::Vertices> polygons = util3d::organizedFastMesh(
											cloud,
											float(ui_->spinBox_mesh_angleTolerance->value())*M_PI/180.0f,
											ui_->checkBox_mesh_quad->isChecked(),
											ui_->spinBox_mesh_triangleSize->value(),
											viewpoint);
									view3D->removeCloud("0");
									view3D->addCloudMesh("0", cloud, polygons);
								}
								else
								{
									view3D->addCloud("0", cloud);
								}
							}
						}
						else
						{
							pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
							cloud = util3d::cloudFromSensorData(data, 1, 0, 0, 0, ui_->parameters_toolbox->getParameters());
							if(cloud->size())
							{
								view3D->addCloud("0", cloud);
							}
						}
					}

					//add scan
					pcl::PointCloud<pcl::PointXYZ>::Ptr scan = util3d::laserScanToPointCloud(data.laserScanRaw());
					if(scan->size())
					{
						view3D->addCloud("1", scan);
					}

					view3D->update();
				}
			}

			if(!img.isNull())
			{
				view->setImage(img);
				rect = img.rect();
			}
			else
			{
				ULOGGER_DEBUG("Image is empty");
			}

			if(!imgDepth.isNull())
			{
				view->setImageDepth(imgDepth);
				if(!img.isNull())
				{
					rect = imgDepth.rect();
				}
			}
			else
			{
				ULOGGER_DEBUG("Image depth is empty");
			}

			// loops
			std::map<int, rtabmap::Link> links;
			dbDriver_->loadLinks(id, links);
			if(links.size())
			{
				QString strParents, strChildren;
				for(std::map<int, rtabmap::Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
				{
					if(iter->second.type() != Link::kNeighbor &&
				       iter->second.type() != Link::kNeighborMerged)
					{
						if(iter->first < id)
						{
							strChildren.append(QString("%1 ").arg(iter->first));
						}
						else
						{
							strParents.append(QString("%1 ").arg(iter->first));
						}
					}
				}
				labelParents->setText(strParents);
				labelChildren->setText(strChildren);
			}
		}

		if(mapId>=0)
		{
			labelMapId->setText(QString::number(mapId));
		}
	}
	else
	{
		ULOGGER_ERROR("Slider index out of range ?");
	}

	updateConstraintButtons();
	updateWordsMatching();

	if(updateConstraintView && ui_->dockWidget_constraints->isVisible())
	{
		// update constraint view
		int from = ids_.at(ui_->horizontalSlider_A->value());
		int to = ids_.at(ui_->horizontalSlider_B->value());
		bool set = false;
		for(int i=0; i<loopLinks_.size() || i<neighborLinks_.size(); ++i)
		{
			if(i < loopLinks_.size())
			{
				if((loopLinks_[i].from() == from && loopLinks_[i].to() == to) ||
				   (loopLinks_[i].from() == to && loopLinks_[i].to() == from))
				{
					if(i != ui_->horizontalSlider_loops->value())
					{
						ui_->horizontalSlider_loops->blockSignals(true);
						ui_->horizontalSlider_loops->setValue(i);
						ui_->horizontalSlider_loops->blockSignals(false);
						this->updateConstraintView(loopLinks_[i].from() == from?loopLinks_.at(i):loopLinks_.at(i).inverse(), false);
					}
					ui_->horizontalSlider_neighbors->blockSignals(true);
					ui_->horizontalSlider_neighbors->setValue(0);
					ui_->horizontalSlider_neighbors->blockSignals(false);
					set = true;
					break;
				}
			}
			if(i < neighborLinks_.size())
			{
				if((neighborLinks_[i].from() == from && neighborLinks_[i].to() == to) ||
				   (neighborLinks_[i].from() == to && neighborLinks_[i].to() == from))
				{
					if(i != ui_->horizontalSlider_neighbors->value())
					{
						ui_->horizontalSlider_neighbors->blockSignals(true);
						ui_->horizontalSlider_neighbors->setValue(i);
						ui_->horizontalSlider_neighbors->blockSignals(false);
						this->updateConstraintView(neighborLinks_[i].from() == from?neighborLinks_.at(i):neighborLinks_.at(i).inverse(), false);
					}
					ui_->horizontalSlider_loops->blockSignals(true);
					ui_->horizontalSlider_loops->setValue(0);
					ui_->horizontalSlider_loops->blockSignals(false);
					set = true;
					break;
				}
			}
		}
		if(!set)
		{
			ui_->horizontalSlider_loops->blockSignals(true);
			ui_->horizontalSlider_neighbors->blockSignals(true);
			ui_->horizontalSlider_loops->setValue(0);
			ui_->horizontalSlider_neighbors->setValue(0);
			ui_->horizontalSlider_loops->blockSignals(false);
			ui_->horizontalSlider_neighbors->blockSignals(false);

			constraintsViewer_->removeAllClouds();

			// make a fake link using globally optimized poses
			if(graphes_.size())
			{
				std::map<int, Transform> optimizedPoses = uValueAt(graphes_, ui_->horizontalSlider_iterations->value());
				if(optimizedPoses.size() > 0)
				{
					std::map<int, Transform>::iterator fromIter = optimizedPoses.find(from);
					std::map<int, Transform>::iterator toIter = optimizedPoses.find(to);
					if(fromIter != optimizedPoses.end() &&
					   toIter != optimizedPoses.end())
					{
						Link link(from, to, Link::kUndef, fromIter->second.inverse() * toIter->second);
						this->updateConstraintView(link, false);
					}
				}
			}

			constraintsViewer_->update();

		}
	}

	if(rect.isValid())
	{
		view->setSceneRect(rect);
	}
}

void DatabaseViewer::updateLoggerLevel()
{
	if(this->parent() == 0)
	{
		ULogger::setLevel((ULogger::Level)ui_->comboBox_logger_level->currentIndex());
	}
}

void DatabaseViewer::updateStereo()
{
	if(ui_->horizontalSlider_A->maximum())
	{
		int id = ids_.at(ui_->horizontalSlider_A->value());
		SensorData data;
		dbDriver_->getNodeData(id, data);
		data.uncompressData();
		updateStereo(&data);
	}
}

void DatabaseViewer::updateStereo(const SensorData * data)
{
	if(data &&
		ui_->dockWidget_stereoView->isVisible() &&
		!data->imageRaw().empty() &&
		!data->depthOrRightRaw().empty() &&
		data->depthOrRightRaw().type() == CV_8UC1 &&
		data->stereoCameraModel().isValidForProjection())
	{
		cv::Mat leftMono;
		if(data->imageRaw().channels() == 3)
		{
			cv::cvtColor(data->imageRaw(), leftMono, CV_BGR2GRAY);
		}
		else
		{
			leftMono = data->imageRaw();
		}

		UTimer timer;
		ParametersMap parameters = ui_->parameters_toolbox->getParameters();
		bool opticalFlow = uStr2Bool(parameters.at(Parameters::kStereoOpticalFlow()));
		Stereo * stereo = 0;
		if(opticalFlow)
		{
			stereo = new StereoOpticalFlow(parameters);
		}
		else
		{
			stereo = new Stereo(parameters);
		}

		// generate kpts
		std::vector<cv::KeyPoint> kpts;
		uInsert(parameters, ParametersPair(Parameters::kKpMaxFeatures(), parameters.at(Parameters::kVisMaxFeatures())));
		uInsert(parameters, ParametersPair(Parameters::kKpMinDepth(), parameters.at(Parameters::kVisMinDepth())));
		uInsert(parameters, ParametersPair(Parameters::kKpMaxDepth(), parameters.at(Parameters::kVisMaxDepth())));
		uInsert(parameters, ParametersPair(Parameters::kKpDetectorStrategy(), parameters.at(Parameters::kVisFeatureType())));
		uInsert(parameters, ParametersPair(Parameters::kKpRoiRatios(), parameters.at(Parameters::kVisRoiRatios())));
		uInsert(parameters, ParametersPair(Parameters::kKpSubPixEps(), parameters.at(Parameters::kVisSubPixEps())));
		uInsert(parameters, ParametersPair(Parameters::kKpSubPixIterations(), parameters.at(Parameters::kVisSubPixIterations())));
		uInsert(parameters, ParametersPair(Parameters::kKpSubPixWinSize(), parameters.at(Parameters::kVisSubPixWinSize())));
		Feature2D * kptDetector = Feature2D::create(parameters);
		kpts = kptDetector->generateKeypoints(leftMono);
		delete kptDetector;

		float timeKpt = timer.ticks();

		std::vector<cv::Point2f> leftCorners;
		cv::KeyPoint::convert(kpts, leftCorners);

		// Find features in the new right image
		std::vector<unsigned char> status;
		std::vector<cv::Point2f> rightCorners;

		rightCorners = stereo->computeCorrespondences(
				leftMono,
				data->rightRaw(),
				leftCorners,
				status);
		delete stereo;

		float timeStereo = timer.ticks();

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(kpts.size());
		float bad_point = std::numeric_limits<float>::quiet_NaN ();
		UASSERT(status.size() == kpts.size());
		int oi = 0;
		int inliers = 0;
		int flowOutliers= 0;
		int slopeOutliers= 0;
		int negativeDisparityOutliers = 0;
		for(unsigned int i=0; i<status.size(); ++i)
		{
			cv::Point3f pt(bad_point, bad_point, bad_point);
			if(status[i])
			{
				float disparity = leftCorners[i].x - rightCorners[i].x;
				if(disparity > 0.0f)
				{
					cv::Point3f tmpPt = util3d::projectDisparityTo3D(
							leftCorners[i],
							disparity,
							data->stereoCameraModel());

					if(util3d::isFinite(tmpPt))
					{
						pt = util3d::transformPoint(tmpPt, data->stereoCameraModel().left().localTransform());
						status[i] = 100; //blue
						++inliers;
						cloud->at(oi++) = pcl::PointXYZ(pt.x, pt.y, pt.z);
					}
				}
				else
				{
					status[i] = 102; //magenta
					++negativeDisparityOutliers;
				}
			}
			else
			{
				++flowOutliers;
			}
		}
		cloud->resize(oi);

		UINFO("correspondences = %d/%d (%f) (time kpt=%fs stereo=%fs)",
				(int)cloud->size(), (int)leftCorners.size(), float(cloud->size())/float(leftCorners.size()), timeKpt, timeStereo);

		stereoViewer_->updateCameraTargetPosition(Transform::getIdentity());
		stereoViewer_->addCloud("stereo", cloud);
		stereoViewer_->update();

		ui_->label_stereo_inliers->setNum(inliers);
		ui_->label_stereo_flowOutliers->setNum(flowOutliers);
		ui_->label_stereo_slopeOutliers->setNum(slopeOutliers);
		ui_->label_stereo_disparityOutliers->setNum(negativeDisparityOutliers);

		std::vector<cv::KeyPoint> rightKpts;
		cv::KeyPoint::convert(rightCorners, rightKpts);
		std::vector<cv::DMatch> good_matches(kpts.size());
		for(unsigned int i=0; i<good_matches.size(); ++i)
		{
			good_matches[i].trainIdx = i;
			good_matches[i].queryIdx = i;
		}


		//
		//cv::Mat imageMatches;
		//cv::drawMatches( leftMono, kpts, data->getDepthRaw(), rightKpts,
		//			   good_matches, imageMatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
		//			   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

		//ui_->graphicsView_stereo->setImage(uCvMat2QImage(imageMatches));

		ui_->graphicsView_stereo->clear();
		ui_->graphicsView_stereo->setLinesShown(true);
		ui_->graphicsView_stereo->setFeaturesShown(false);
		ui_->graphicsView_stereo->setImageDepthShown(true);

		ui_->graphicsView_stereo->setImage(uCvMat2QImage(data->imageRaw()));
		ui_->graphicsView_stereo->setImageDepth(uCvMat2QImage(data->depthOrRightRaw()));

		// Draw lines between corresponding features...
		for(unsigned int i=0; i<kpts.size(); ++i)
		{
			if(rightKpts[i].pt.x > 0 && rightKpts[i].pt.y > 0)
			{
				QColor c = Qt::green;
				if(status[i] == 0)
				{
					c = Qt::red;
				}
				else if(status[i] == 100)
				{
					c = Qt::blue;
				}
				else if(status[i] == 101)
				{
					c = Qt::yellow;
				}
				else if(status[i] == 102)
				{
					c = Qt::magenta;
				}
				else if(status[i] == 110)
				{
					c = Qt::cyan;
				}
				ui_->graphicsView_stereo->addLine(
						kpts[i].pt.x,
						kpts[i].pt.y,
						rightKpts[i].pt.x,
						rightKpts[i].pt.y,
						c,
						QString("%1: (%2,%3) -> (%4,%5)").arg(i).arg(kpts[i].pt.x).arg(kpts[i].pt.y).arg(rightKpts[i].pt.x).arg(rightKpts[i].pt.y));
			}
		}
		ui_->graphicsView_stereo->update();
	}
}

void DatabaseViewer::updateWordsMatching()
{
	int from = ids_.at(ui_->horizontalSlider_A->value());
	int to = ids_.at(ui_->horizontalSlider_B->value());
	if(from && to)
	{
		int alpha = 70;
		ui_->graphicsView_A->clearLines();
		ui_->graphicsView_A->setFeaturesColor(QColor(255, 255, 0, alpha)); // yellow
		ui_->graphicsView_B->clearLines();
		ui_->graphicsView_B->setFeaturesColor(QColor(255, 255, 0, alpha)); // yellow

		const QMultiMap<int, KeypointItem*> & wordsA = ui_->graphicsView_A->getFeatures();
		const QMultiMap<int, KeypointItem*> & wordsB = ui_->graphicsView_B->getFeatures();
		if(wordsA.size() && wordsB.size())
		{
			QList<int> ids =  wordsA.uniqueKeys();
			for(int i=0; i<ids.size(); ++i)
			{
				if(wordsA.count(ids[i]) == 1 && wordsB.count(ids[i]) == 1)
				{
					// PINK features
					ui_->graphicsView_A->setFeatureColor(ids[i], Qt::magenta);
					ui_->graphicsView_B->setFeatureColor(ids[i], Qt::magenta);

					// Add lines
					// Draw lines between corresponding features...
					float scaleX = ui_->graphicsView_A->viewScale();
					float deltaX = 0;
					float deltaY = 0;

					if(ui_->checkBox_verticalLayout->isChecked())
					{
						deltaY = ui_->graphicsView_A->height()/scaleX;
					}
					else
					{
						deltaX = ui_->graphicsView_A->width()/scaleX;
					}

					const KeypointItem * kptA = wordsA.value(ids[i]);
					const KeypointItem * kptB = wordsB.value(ids[i]);
					ui_->graphicsView_A->addLine(
							kptA->rect().x()+kptA->rect().width()/2,
							kptA->rect().y()+kptA->rect().height()/2,
							kptB->rect().x()+kptB->rect().width()/2+deltaX,
							kptB->rect().y()+kptB->rect().height()/2+deltaY,
							Qt::cyan);

					ui_->graphicsView_B->addLine(
							kptA->rect().x()+kptA->rect().width()/2-deltaX,
							kptA->rect().y()+kptA->rect().height()/2-deltaY,
							kptB->rect().x()+kptB->rect().width()/2,
							kptB->rect().y()+kptB->rect().height()/2,
							Qt::cyan);
				}
			}
			ui_->graphicsView_A->update();
			ui_->graphicsView_B->update();
		}
	}
}

void DatabaseViewer::sliderAMoved(int value)
{
	ui_->label_indexA->setText(QString::number(value));
	if(value>=0 && value < ids_.size())
	{
		ui_->label_idA->setText(QString::number(ids_.at(value)));
	}
	else
	{
		ULOGGER_ERROR("Slider index out of range ?");
	}
}

void DatabaseViewer::sliderBMoved(int value)
{
	ui_->label_indexB->setText(QString::number(value));
	if(value>=0 && value < ids_.size())
	{
		ui_->label_idB->setText(QString::number(ids_.at(value)));
	}
	else
	{
		ULOGGER_ERROR("Slider index out of range ?");
	}
}

void DatabaseViewer::update3dView()
{
	if(ui_->dockWidget_view3d->isVisible())
	{
		sliderAValueChanged(ui_->horizontalSlider_A->value());
		sliderBValueChanged(ui_->horizontalSlider_B->value());
	}
}

void DatabaseViewer::sliderNeighborValueChanged(int value)
{
	this->updateConstraintView(neighborLinks_.at(value));
}

void DatabaseViewer::sliderLoopValueChanged(int value)
{
	this->updateConstraintView(loopLinks_.at(value));
}

// only called when ui_->checkBox_showOptimized state changed
void DatabaseViewer::updateConstraintView()
{
	if(ids_.size())
	{
		Link link = this->findActiveLink(ids_.at(ui_->horizontalSlider_A->value()), ids_.at(ui_->horizontalSlider_B->value()));
		if(link.isValid())
		{
			if(link.type() == Link::kNeighbor ||
			   link.type() == Link::kNeighborMerged)
			{
				this->updateConstraintView(neighborLinks_.at(ui_->horizontalSlider_neighbors->value()), false);
			}
			else
			{
				this->updateConstraintView(loopLinks_.at(ui_->horizontalSlider_loops->value()), false);
			}
		}
	}
}

void DatabaseViewer::updateConstraintView(
		const rtabmap::Link & linkIn,
		bool updateImageSliders,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudFrom,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudTo,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & scanFrom,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & scanTo)
{
	std::multimap<int, Link>::iterator iterLink = rtabmap::graph::findLink(linksRefined_, linkIn.from(), linkIn.to());
	rtabmap::Link link = linkIn;
	if(iterLink != linksRefined_.end())
	{
		link = iterLink->second;
	}
	else if(ui_->checkBox_ignorePoseCorrection->isChecked())
	{
		if(link.type() == Link::kNeighbor ||
		   link.type() == Link::kNeighborMerged)
		{
			Transform poseFrom = uValue(poses_, link.from(), Transform());
			Transform poseTo = uValue(poses_, link.to(), Transform());
			if(!poseFrom.isNull() && !poseTo.isNull())
			{
				link.setTransform(poseFrom.inverse() * poseTo); // recompute raw odom transformation
			}
		}
	}
	rtabmap::Transform t = link.transform();

	ui_->label_constraint->clear();
	ui_->label_constraint_opt->clear();
	ui_->checkBox_showOptimized->setEnabled(false);
	UASSERT(!t.isNull() && dbDriver_);

	ui_->label_type->setText(tr("%1 (%2)")
			.arg(link.type())
			.arg(link.type()==Link::kNeighbor?"Neighbor":
				 link.type()==Link::kNeighbor?"Merged neighbor":
				 link.type()==Link::kGlobalClosure?"Loop closure":
				 link.type()==Link::kLocalSpaceClosure?"Space proximity link":
				 link.type()==Link::kLocalTimeClosure?"Time proximity link":
				 link.type()==Link::kUserClosure?"User link":
				 link.type()==Link::kVirtualClosure?"Virtual link":"Undefined"));
	ui_->label_variance->setText(QString("%1, %2")
			.arg(sqrt(link.rotVariance()))
			.arg(sqrt(link.transVariance())));
	ui_->label_constraint->setText(QString("%1").arg(t.prettyPrint().c_str()).replace(" ", "\n"));
	if((link.type() == Link::kNeighbor || link.type() == Link::kNeighborMerged) &&
	   graphes_.size() &&
	   (int)graphes_.size()-1 == ui_->horizontalSlider_iterations->maximum())
	{
		std::map<int, rtabmap::Transform> & graph = uValueAt(graphes_, ui_->horizontalSlider_iterations->value());
		if(link.type() == Link::kNeighbor || link.type() == Link::kNeighborMerged)
		{
			std::map<int, rtabmap::Transform>::iterator iterFrom = graph.find(link.from());
			std::map<int, rtabmap::Transform>::iterator iterTo = graph.find(link.to());
			if(iterFrom != graph.end() && iterTo != graph.end())
			{
				ui_->checkBox_showOptimized->setEnabled(true);
				Transform topt = iterFrom->second.inverse()*iterTo->second;
				float diff = topt.getDistance(t);
				Transform v1 = t.rotation()*Transform(1,0,0,0,0,0);
				Transform v2 = topt.rotation()*Transform(1,0,0,0,0,0);
				float a = pcl::getAngle3D(Eigen::Vector4f(v1.x(), v1.y(), v1.z(), 0), Eigen::Vector4f(v2.x(), v2.y(), v2.z(), 0));
				a = (a *180.0f) / CV_PI;
				ui_->label_constraint_opt->setText(QString("%1\n(error=%2% a=%3)").arg(QString(topt.prettyPrint().c_str()).replace(" ", "\n")).arg((diff/t.getNorm())*100.0f).arg(a));

				if(ui_->checkBox_showOptimized->isChecked())
				{
					t = topt;
				}
			}
		}
	}

	if(updateImageSliders)
	{
		ui_->horizontalSlider_A->blockSignals(true);
		ui_->horizontalSlider_B->blockSignals(true);
		// set from on left and to on right		{
		ui_->horizontalSlider_A->setValue(idToIndex_.value(link.from()));
		ui_->horizontalSlider_B->setValue(idToIndex_.value(link.to()));
		ui_->horizontalSlider_A->blockSignals(false);
		ui_->horizontalSlider_B->blockSignals(false);
		this->update(idToIndex_.value(link.from()),
					ui_->label_indexA,
					ui_->label_parentsA,
					ui_->label_childrenA,
					ui_->label_weightA,
					ui_->label_labelA,
					ui_->label_stampA,
					ui_->graphicsView_A,
					cloudViewerA_,
					ui_->label_idA,
					ui_->label_mapA,
					ui_->label_poseA,
					ui_->label_calibA,
					false); // don't update constraints view!
		this->update(idToIndex_.value(link.to()),
					ui_->label_indexB,
					ui_->label_parentsB,
					ui_->label_childrenB,
					ui_->label_weightB,
					ui_->label_labelB,
					ui_->label_stampB,
					ui_->graphicsView_B,
					cloudViewerB_,
					ui_->label_idB,
					ui_->label_mapB,
					ui_->label_poseB,
					ui_->label_calibB,
					false); // don't update constraints view!
	}

	if(constraintsViewer_->isVisible())
	{
		SensorData dataFrom, dataTo;

		dbDriver_->getNodeData(link.from(), dataFrom);
		dataFrom.uncompressData();
		UASSERT(dataFrom.imageRaw().empty() || dataFrom.imageRaw().type()==CV_8UC3 || dataFrom.imageRaw().type() == CV_8UC1);
		UASSERT(dataFrom.depthOrRightRaw().empty() || dataFrom.depthOrRightRaw().type()==CV_8UC1 || dataFrom.depthOrRightRaw().type() == CV_16UC1 || dataFrom.depthOrRightRaw().type() == CV_32FC1);

		dbDriver_->getNodeData(link.to(), dataTo);
		dataTo.uncompressData();
		UASSERT(dataTo.imageRaw().empty() || dataTo.imageRaw().type()==CV_8UC3 || dataTo.imageRaw().type() == CV_8UC1);
		UASSERT(dataTo.depthOrRightRaw().empty() || dataTo.depthOrRightRaw().type()==CV_8UC1 || dataTo.depthOrRightRaw().type() == CV_16UC1 || dataTo.depthOrRightRaw().type() == CV_32FC1);


		if(cloudFrom->size() == 0 && cloudTo->size() == 0)
		{
			//cloud 3d
			if(ui_->checkBox_show3Dclouds->isChecked())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFrom, cloudTo;
				if(!dataFrom.imageRaw().empty() && !dataFrom.depthOrRightRaw().empty())
				{
					cloudFrom=util3d::cloudRGBFromSensorData(dataFrom, 1, 0, 0, 0, ui_->parameters_toolbox->getParameters());
				}
				if(!dataTo.imageRaw().empty() && !dataTo.depthOrRightRaw().empty())
				{
					cloudTo=util3d::cloudRGBFromSensorData(dataTo, 1, 0, 0, 0, ui_->parameters_toolbox->getParameters());
				}

				if(cloudFrom.get() && cloudFrom->size())
				{
					constraintsViewer_->addCloud("cloud0", cloudFrom, Transform::getIdentity(), Qt::red);
				}
				if(cloudTo.get() && cloudTo->size())
				{
					cloudTo = rtabmap::util3d::transformPointCloud(cloudTo, t);
					constraintsViewer_->addCloud("cloud1", cloudTo, Transform::getIdentity(), Qt::cyan);
				}
			}
			else
			{
				constraintsViewer_->removeCloud("cloud0");
				constraintsViewer_->removeCloud("cloud1");
			}
			if(ui_->checkBox_show3DWords->isChecked())
			{
				std::list<int> ids;
				ids.push_back(link.from());
				ids.push_back(link.to());
				std::list<Signature*> signatures;
				dbDriver_->loadSignatures(ids, signatures);
				if(signatures.size() == 2)
				{
					const Signature * sFrom = signatures.front();
					const Signature * sTo = signatures.back();
					UASSERT(sFrom && sTo);
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFrom(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTo(new pcl::PointCloud<pcl::PointXYZ>);
					cloudFrom->resize(sFrom->getWords3().size());
					cloudTo->resize(sTo->getWords3().size());
					int i=0;
					for(std::multimap<int, cv::Point3f>::const_iterator iter=sFrom->getWords3().begin();
						iter!=sFrom->getWords3().end();
						++iter)
					{
						cloudFrom->at(i++) = pcl::PointXYZ(iter->second.x, iter->second.y, iter->second.z);
					}
					i=0;
					for(std::multimap<int, cv::Point3f>::const_iterator iter=sTo->getWords3().begin();
						iter!=sTo->getWords3().end();
						++iter)
					{
						cloudTo->at(i++) = pcl::PointXYZ(iter->second.x, iter->second.y, iter->second.z);
					}

					if(cloudFrom->size())
					{
						cloudFrom = rtabmap::util3d::removeNaNFromPointCloud(cloudFrom);
					}
					if(cloudTo->size())
					{
						cloudTo = rtabmap::util3d::removeNaNFromPointCloud(cloudTo);
						if(cloudTo->size())
						{
							cloudTo = rtabmap::util3d::transformPointCloud(cloudTo, t);
						}
					}

					if(cloudFrom->size())
					{
						constraintsViewer_->addCloud("words0", cloudFrom, Transform::getIdentity(), Qt::red);
					}
					else
					{
						UWARN("Empty 3D words for node %d", link.from());
						constraintsViewer_->removeCloud("words0");
					}
					if(cloudTo->size())
					{
						constraintsViewer_->addCloud("words1", cloudTo, Transform::getIdentity(), Qt::cyan);
					}
					else
					{
						UWARN("Empty 3D words for node %d", link.to());
						constraintsViewer_->removeCloud("words1");
					}
				}
				else
				{
					UERROR("Not found signature %d or %d in RAM", link.from(), link.to());
					constraintsViewer_->removeCloud("words0");
					constraintsViewer_->removeCloud("words1");
				}
				//cleanup
				for(std::list<Signature*>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
				{
					delete *iter;
				}
			}
			else
			{
				constraintsViewer_->removeCloud("words0");
				constraintsViewer_->removeCloud("words1");
			}
		}
		else
		{
			if(cloudFrom->size())
			{
				constraintsViewer_->addCloud("cloud0", cloudFrom, Transform::getIdentity(), Qt::red);
			}
			if(cloudTo->size())
			{
				constraintsViewer_->addCloud("cloud1", cloudTo, Transform::getIdentity(), Qt::cyan);
			}
		}

		if(scanFrom->size() == 0 && scanTo->size() == 0)
		{
			if(ui_->checkBox_show2DScans->isChecked())
			{
				//cloud 2d

				constraintsViewer_->removeCloud("scan2");
				constraintsViewer_->removeGraph("scan2graph");
				if(link.type() == Link::kLocalSpaceClosure &&
				   !link.userDataCompressed().empty())
				{
					std::vector<int> ids;
					cv::Mat userData = link.uncompressUserDataConst();
					if(userData.type() == CV_8SC1 &&
					   userData.rows == 1 &&
					   userData.cols >= 8 && // including null str ending
					   userData.at<char>(userData.cols-1) == 0 &&
					   memcmp(userData.data, "SCANS:", 6) == 0)
					{
						std::string scansStr = (const char *)userData.data;
						UINFO("Detected \"%s\" in links's user data", scansStr.c_str());
						if(!scansStr.empty())
						{
							std::list<std::string> strs = uSplit(scansStr, ':');
							if(strs.size() == 2)
							{
								std::list<std::string> strIds = uSplit(strs.rbegin()->c_str(), ';');
								for(std::list<std::string>::iterator iter=strIds.begin(); iter!=strIds.end(); ++iter)
								{
									ids.push_back(atoi(iter->c_str()));
									if(ids.back() == link.from())
									{
										ids.pop_back();
									}
								}
							}
						}
					}
					if(ids.size())
					{
						//add other scans matching
						//optimize the path's poses locally

						std::map<int, rtabmap::Transform> poses;
						for(unsigned int i=0; i<ids.size(); ++i)
						{
							if(uContains(poses_, ids[i]))
							{
								poses.insert(*poses_.find(ids[i]));
							}
							else
							{
								UERROR("Not found %d node!", ids[i]);
							}
						}
						if(poses.size())
						{
							Optimizer * optimizer = Optimizer::create(ui_->parameters_toolbox->getParameters());

							UASSERT(uContains(poses, link.to()));
							std::map<int, rtabmap::Transform> posesOut;
							std::multimap<int, rtabmap::Link> linksOut;
							optimizer->getConnectedGraph(
									link.to(),
									poses,
									updateLinksWithModifications(links_),
									posesOut,
									linksOut);

							if(poses.size() != posesOut.size())
							{
								UWARN("Scan poses input and output are different! %d vs %d", (int)poses.size(), (int)posesOut.size());
								UWARN("Input poses: ");
								for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
								{
									UWARN(" %d", iter->first);
								}
								UWARN("Input links: ");
								std::multimap<int, Link> modifiedLinks = updateLinksWithModifications(links_);
								for(std::multimap<int, Link>::iterator iter=modifiedLinks.begin(); iter!=modifiedLinks.end(); ++iter)
								{
									UWARN(" %d->%d", iter->second.from(), iter->second.to());
								}
							}

							QTime time;
							time.start();
							std::map<int, rtabmap::Transform> finalPoses = optimizer->optimize(link.to(), posesOut, linksOut);
							delete optimizer;

							// transform local poses in loop referential
							Transform u = t * finalPoses.at(link.to()).inverse();
							pcl::PointCloud<pcl::PointXYZ>::Ptr assembledScans(new pcl::PointCloud<pcl::PointXYZ>);
							pcl::PointCloud<pcl::PointXYZ>::Ptr graph(new pcl::PointCloud<pcl::PointXYZ>);
							for(std::map<int, Transform>::iterator iter=finalPoses.begin(); iter!=finalPoses.end(); ++iter)
							{
								iter->second = u * iter->second;
								if(iter->first != link.to()) // already added to view
								{
									//create scan
									SensorData data;
									dbDriver_->getNodeData(iter->first, data);
									cv::Mat scan;
									data.uncompressDataConst(0, 0, &scan, 0);
									if(!scan.empty())
									{
										pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud = util3d::laserScanToPointCloud(scan);
										if(assembledScans->size() == 0)
										{
											assembledScans = util3d::transformPointCloud(scanCloud, iter->second);
										}
										else
										{
											*assembledScans += *util3d::transformPointCloud(scanCloud, iter->second);
										}
									}
								}
								graph->push_back(pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z()));
							}

							if(assembledScans->size())
							{
								constraintsViewer_->addCloud("scan2", assembledScans, Transform::getIdentity(), Qt::cyan);
							}
							if(graph->size())
							{
								constraintsViewer_->addOrUpdateGraph("scan2graph", graph, Qt::cyan);
							}
						}
					}
				}

				// Added loop closure scans
				pcl::PointCloud<pcl::PointXYZ>::Ptr scanA, scanB;
				scanA = rtabmap::util3d::laserScanToPointCloud(dataFrom.laserScanRaw());
				scanB = rtabmap::util3d::laserScanToPointCloud(dataTo.laserScanRaw());
				scanB = rtabmap::util3d::transformPointCloud(scanB, t);
				if(scanA->size())
				{
					constraintsViewer_->addCloud("scan0", scanA, Transform::getIdentity(), Qt::yellow);
				}
				else
				{
					constraintsViewer_->removeCloud("scan0");
				}
				if(scanB->size())
				{
					constraintsViewer_->addCloud("scan1", scanB, Transform::getIdentity(), Qt::magenta);
				}
				else
				{
					constraintsViewer_->removeCloud("scan1");
				}
			}
			else
			{
				constraintsViewer_->removeCloud("scan0");
				constraintsViewer_->removeCloud("scan1");
				constraintsViewer_->removeCloud("scan2");
			}
		}
		else
		{
			if(scanFrom->size())
			{
				constraintsViewer_->addCloud("scan0", scanFrom, Transform::getIdentity(), Qt::yellow);
			}
			else
			{
				constraintsViewer_->removeCloud("scan0");
			}
			if(scanTo->size())
			{
				constraintsViewer_->addCloud("scan1", scanTo, Transform::getIdentity(), Qt::magenta);
			}
			else
			{
				constraintsViewer_->removeCloud("scan1");
			}
			constraintsViewer_->removeCloud("scan2");
		}

		//update coordinate

		constraintsViewer_->addOrUpdateCoordinate("from_coordinate", Transform::getIdentity(), 0.2);
		constraintsViewer_->addOrUpdateCoordinate("to_coordinate", t, 0.2);
		if(uContains(groundTruthPoses_, link.from()) && uContains(groundTruthPoses_, link.to()))
		{
			constraintsViewer_->addOrUpdateCoordinate("to_coordinate_gt",
					groundTruthPoses_.at(link.from()).inverse()*groundTruthPoses_.at(link.to()), 0.1);
		}

		constraintsViewer_->clearTrajectory();

		constraintsViewer_->update();
	}

	// update buttons
	updateConstraintButtons();
}

void DatabaseViewer::updateConstraintButtons()
{
	ui_->pushButton_refine->setEnabled(false);
	ui_->pushButton_refineVisually->setEnabled(false);
	ui_->pushButton_reset->setEnabled(false);
	ui_->pushButton_add->setEnabled(false);
	ui_->pushButton_reject->setEnabled(false);

	int from = ids_.at(ui_->horizontalSlider_A->value());
	int to = ids_.at(ui_->horizontalSlider_B->value());
	if(from!=to && from && to)
	{
		if((!containsLink(links_, from ,to) && !containsLink(linksAdded_, from ,to)) ||
			containsLink(linksRemoved_, from ,to))
		{
			ui_->pushButton_add->setEnabled(true);
		}
	}

	Link currentLink = findActiveLink(from ,to);

	if(currentLink.isValid() &&
		((currentLink.from() == from && currentLink.to() == to) || (currentLink.from() == to && currentLink.to() == from)))
	{
		if(!containsLink(linksRemoved_, from ,to))
		{
			ui_->pushButton_reject->setEnabled(
					currentLink.type() != Link::kNeighbor &&
					currentLink.type() != Link::kNeighborMerged);
		}

		//check for modified link
		bool modified = false;
		std::multimap<int, Link>::iterator iter = rtabmap::graph::findLink(linksRefined_, currentLink.from(), currentLink.to());
		if(iter != linksRefined_.end())
		{
			currentLink = iter->second;
			ui_->pushButton_reset->setEnabled(true);
			modified = true;
		}
		if(!modified)
		{
			ui_->pushButton_reset->setEnabled(false);
		}
		ui_->pushButton_refine->setEnabled(true);
		ui_->pushButton_refineVisually->setEnabled(true);
	}
}

void DatabaseViewer::sliderIterationsValueChanged(int value)
{
	if(dbDriver_ && value >=0 && value < (int)graphes_.size())
	{
		std::map<int, rtabmap::Transform> & graph = uValueAt(graphes_, value);
		std::map<int, rtabmap::Transform> graphFiltered = graph;
		if(ui_->groupBox_posefiltering->isChecked())
		{
			graphFiltered = graph::radiusPosesFiltering(graph,
					ui_->doubleSpinBox_posefilteringRadius->value(),
					ui_->doubleSpinBox_posefilteringAngle->value()*CV_PI/180.0);
		}
		if(ui_->dockWidget_graphView->isVisible())
		{
			//update scans
			UINFO("Update local maps list...");
			std::vector<int> ids = uKeys(graphFiltered);
			for(unsigned int i=0; i<ids.size(); ++i)
			{
				if(localMaps_.find(ids[i]) == localMaps_.end())
				{
					UTimer time;
					bool added = false;
					if(ui_->groupBox_gridFromProjection->isChecked())
					{
						SensorData data;
						dbDriver_->getNodeData(ids.at(i), data);
						data.uncompressData();
						if(!data.depthOrRightRaw().empty())
						{
							pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
							pcl::IndicesPtr validIndices(new std::vector<int>);
							cloud = util3d::cloudFromSensorData(data,
									ui_->spinBox_projDecimation->value(),
									ui_->doubleSpinBox_projMaxDepth->value(),
									ui_->doubleSpinBox_projMinDepth->value(),
									validIndices.get(),
									ui_->parameters_toolbox->getParameters());
							UASSERT(ui_->doubleSpinBox_gridCellSize->value() > 0);
							cloud = util3d::voxelize(cloud, validIndices, ui_->doubleSpinBox_gridCellSize->value());

							// add pose rotation without yaw
							float roll, pitch, yaw;
							graphFiltered.at(ids[i]).getEulerAngles(roll, pitch, yaw);
							cloud = util3d::transformPointCloud(cloud, Transform(0,0,0, roll, pitch, 0));

							if(cloud->size())
							{
								UTimer timer;
								float cellSize = ui_->doubleSpinBox_gridCellSize->value();
								float groundNormalMaxAngle = ui_->doubleSpinBox_projMaxAngle->value();
								int minClusterSize = ui_->spinBox_projClusterSize->value();
								cv::Mat ground, obstacles;

								util3d::occupancy2DFromCloud3D<pcl::PointXYZ>(
										cloud,
										ground, obstacles,
										cellSize,
										groundNormalMaxAngle,
										minClusterSize);

								if(!ground.empty() || !obstacles.empty())
								{
									localMaps_.insert(std::make_pair(ids.at(i), std::make_pair(ground, obstacles)));
									added = true;
								}
							}
						}
					}
					else
					{
						SensorData data;
						dbDriver_->getNodeData(ids.at(i), data);
						if(!data.laserScanCompressed().empty())
						{
							pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
							cv::Mat laserScan;
							data.uncompressDataConst(0, 0, &laserScan);
							cv::Mat ground, obstacles;
							if(laserScan.type() == CV_32FC2)
							{
								util3d::occupancy2DFromLaserScan(
										laserScan,
										ground,
										obstacles,
										ui_->doubleSpinBox_gridCellSize->value(),
										ui_->checkBox_gridFillUnkownSpace->isChecked(),
										data.laserScanMaxRange());
								added = true;
							}
							localMaps_.insert(std::make_pair(ids.at(i), std::make_pair(ground, obstacles)));
						}
					}
					if(added)
					{
						UINFO("Processed grid map %d/%d (%fs)", i+1, (int)ids.size(), time.ticks());
					}
				}
			}
			//cleanup
			for(std::map<int, std::pair<cv::Mat, cv::Mat> >::iterator iter=localMaps_.begin(); iter!=localMaps_.end();)
			{
				if(graphFiltered.find(iter->first) == graphFiltered.end())
				{
					localMaps_.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
			UINFO("Update local maps list... done");
		}

		ui_->graphViewer->updateGTGraph(groundTruthPoses_);
		ui_->graphViewer->updateGraph(graph, graphLinks_, mapIds_);
		if(graph.size() && localMaps_.size() && ui_->graphViewer->isGridMapVisible())
		{
			float xMin, yMin;
			float cell = ui_->doubleSpinBox_gridCellSize->value();
			cv::Mat map;
			QTime time;
			time.start();
			map = rtabmap::util3d::create2DMapFromOccupancyLocalMaps(graphFiltered, localMaps_, cell, xMin, yMin, 0, ui_->checkBox_gridErode->isChecked());
			if(!map.empty())
			{
				ui_->graphViewer->updateMap(rtabmap::util3d::convertMap2Image8U(map), cell, xMin, yMin);
			}
			ui_->label_timeGrid->setNum(double(time.elapsed())/1000.0);
		}
		ui_->graphViewer->update();
		ui_->label_iterations->setNum(value);

		//compute total length (neighbor links)
		float length = 0.0f;
		for(std::multimap<int, rtabmap::Link>::const_iterator iter=graphLinks_.begin(); iter!=graphLinks_.end(); ++iter)
		{
			std::map<int, rtabmap::Transform>::const_iterator jterA = graph.find(iter->first);
			std::map<int, rtabmap::Transform>::const_iterator jterB = graph.find(iter->second.to());
			if(jterA != graph.end() && jterB != graph.end())
			{
				const rtabmap::Transform & poseA = jterA->second;
				const rtabmap::Transform & poseB = jterB->second;
				if(iter->second.type() == rtabmap::Link::kNeighbor ||
				   iter->second.type() == rtabmap::Link::kNeighborMerged)
				{
					Eigen::Vector3f vA, vB;
					float x,y,z;
					poseA.getTranslation(x,y,z);
					vA[0] = x; vA[1] = y; vA[2] = z;
					poseB.getTranslation(x,y,z);
					vB[0] = x; vB[1] = y; vB[2] = z;
					length += (vB - vA).norm();
				}
			}
		}
		ui_->label_pathLength->setNum(length);
	}
}
void DatabaseViewer::updateGraphView()
{
	ui_->label_loopClosures->clear();
	ui_->label_poses->clear();

	if(poses_.size())
	{
		int fromId = ui_->spinBox_optimizationsFrom->value();
		if(!uContains(poses_, fromId))
		{
			QMessageBox::warning(this, tr(""), tr("Graph optimization from id (%1) for which node is not linked to graph.\n Minimum=%2, Maximum=%3")
						.arg(fromId)
						.arg(poses_.begin()->first)
						.arg(poses_.rbegin()->first));
			return;
		}

		graphes_.clear();
		graphLinks_.clear();

		std::map<int, rtabmap::Transform> poses = poses_;

		// filter current map if not spanning to all maps
		if(!ui_->checkBox_spanAllMaps->isChecked() && uContains(mapIds_, fromId) && mapIds_.at(fromId) >= 0)
		{
			int currentMapId = mapIds_.at(fromId);
			for(std::map<int, rtabmap::Transform>::iterator iter=poses.begin(); iter!=poses.end();)
			{
				if(!uContains(mapIds_, iter->first) ||
					mapIds_.at(iter->first) != currentMapId)
				{
					poses.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
		}

		graphes_.push_back(poses);

		ui_->actionGenerate_TORO_graph_graph->setEnabled(true);
		ui_->actionGenerate_g2o_graph_g2o->setEnabled(true);
		std::multimap<int, rtabmap::Link> links = links_;

		// filter current map if not spanning to all maps
		if(!ui_->checkBox_spanAllMaps->isChecked() && uContains(mapIds_, fromId) && mapIds_.at(fromId) >= 0)
		{
			int currentMapId = mapIds_.at(fromId);
			for(std::multimap<int, rtabmap::Link>::iterator iter=links.begin(); iter!=links.end();)
			{
				if(!uContains(mapIds_, iter->second.from()) ||
					!uContains(mapIds_, iter->second.to()) ||
					mapIds_.at(iter->second.from()) != currentMapId ||
					mapIds_.at(iter->second.to()) != currentMapId)
				{
					links.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
		}

		if(ui_->checkBox_ignorePoseCorrection->isChecked())
		{
			std::multimap<int, Link> tmp = links;
			for(std::multimap<int, Link>::iterator iter=tmp.begin(); iter!=tmp.end(); ++iter)
			{
				if(iter->second.type() == Link::kNeighbor ||
				   iter->second.type() == Link::kNeighborMerged)
				{
					Transform poseFrom = uValue(poses, iter->second.from(), Transform());
					Transform poseTo = uValue(poses, iter->second.to(), Transform());
					if(!poseFrom.isNull() && !poseTo.isNull())
					{
						iter->second.setTransform(poseFrom.inverse() * poseTo); // recompute raw odom transformation
						iter->second.setVariance(1.0f, 1.0f); // reset variance
					}
				}
			}
			links = updateLinksWithModifications(tmp);
		}
		else
		{
			links = updateLinksWithModifications(links);
		}

		// filter links
		int totalNeighbor = 0;
		int totalNeighborMerged = 0;
		int totalGlobal = 0;
		int totalLocalTime = 0;
		int totalLocalSpace = 0;
		int totalUser = 0;
		for(std::multimap<int, rtabmap::Link>::iterator iter=links.begin(); iter!=links.end();)
		{
			if(iter->second.type() == Link::kNeighbor)
			{
				++totalNeighbor;
			}
			else if(iter->second.type() == Link::kNeighborMerged)
			{
				++totalNeighborMerged;
			}
			else if(iter->second.type() == Link::kGlobalClosure)
			{
				if(ui_->checkBox_ignoreGlobalLoop->isChecked())
				{
					links.erase(iter++);
					continue;
				}
				++totalGlobal;
			}
			else if(iter->second.type() == Link::kLocalSpaceClosure)
			{
				if(ui_->checkBox_ignoreLocalLoopSpace->isChecked())
				{
					links.erase(iter++);
					continue;
				}
				++totalLocalSpace;
			}
			else if(iter->second.type() == Link::kLocalTimeClosure)
			{
				if(ui_->checkBox_ignoreLocalLoopTime->isChecked())
				{
					links.erase(iter++);
					continue;
				}
				++totalLocalTime;
			}
			else if(iter->second.type() == Link::kUserClosure)
			{
				if(ui_->checkBox_ignoreUserLoop->isChecked())
				{
					links.erase(iter++);
					continue;
				}
				++totalUser;
			}
			++iter;
		}
		ui_->label_loopClosures->setText(tr("(%1, %2, %3, %4, %5, %6)")
				.arg(totalNeighbor)
				.arg(totalNeighborMerged)
				.arg(totalGlobal)
				.arg(totalLocalSpace)
				.arg(totalLocalTime)
				.arg(totalUser));

		Optimizer * optimizer = Optimizer::create(ui_->parameters_toolbox->getParameters());

		std::map<int, rtabmap::Transform> posesOut;
		std::multimap<int, rtabmap::Link> linksOut;
		UINFO("Get connected graph from %d (%d poses, %d links)", fromId, (int)poses.size(), (int)links.size());
		optimizer->getConnectedGraph(
				fromId,
				poses,
				links,
				posesOut,
				linksOut,
				ui_->spinBox_optimizationDepth->value());
		UINFO("Connected graph of %d poses and %d links", (int)posesOut.size(), (int)linksOut.size());
		QTime time;
		time.start();
		std::map<int, rtabmap::Transform> finalPoses = optimizer->optimize(fromId, posesOut, linksOut, &graphes_);
		ui_->label_timeOptimization->setNum(double(time.elapsed())/1000.0);
		graphes_.push_back(finalPoses);
		graphLinks_ = linksOut;
		ui_->label_poses->setNum((int)finalPoses.size());
		delete optimizer;
		if(posesOut.size() && finalPoses.empty())
		{
			QMessageBox::warning(this, tr("Graph optimization error!"), tr("Graph optimization has failed. See the terminal for potential errors."));
		}

		if(uContains(groundTruthPoses_, fromId) && uContains(posesOut, fromId))
		{
			// adjust the ground truth to fit the root
			Transform t = posesOut.at(fromId) * groundTruthPoses_.at(fromId).inverse();
			for(std::map<int, Transform>::iterator iter=groundTruthPoses_.begin(); iter!=groundTruthPoses_.end(); ++iter)
			{
				iter->second = t * iter->second;
			}
		}
		else if(groundTruthPoses_.size())
		{
			UWARN("Could not find ground truth for root node %d", fromId);
		}
	}
	if(graphes_.size())
	{
		ui_->horizontalSlider_iterations->setMaximum((int)graphes_.size()-1);
		ui_->horizontalSlider_iterations->setValue((int)graphes_.size()-1);
		ui_->horizontalSlider_iterations->setEnabled(true);
		ui_->spinBox_optimizationsFrom->setEnabled(true);
		sliderIterationsValueChanged((int)graphes_.size()-1);
	}
	else
	{
		ui_->horizontalSlider_iterations->setEnabled(false);
		ui_->spinBox_optimizationsFrom->setEnabled(false);
	}
}

void DatabaseViewer::updateGrid()
{
	if((sender() != ui_->spinBox_projDecimation && sender() != ui_->doubleSpinBox_projMaxDepth && sender() != ui_->doubleSpinBox_projMinDepth && sender()!=ui_->doubleSpinBox_projMaxAngle && sender()!=ui_->spinBox_projClusterSize) ||
	   (sender() == ui_->spinBox_projDecimation && ui_->groupBox_gridFromProjection->isChecked()) ||
	   (sender() == ui_->doubleSpinBox_projMaxDepth && ui_->groupBox_gridFromProjection->isChecked()) ||
	   (sender() == ui_->doubleSpinBox_projMinDepth && ui_->groupBox_gridFromProjection->isChecked()) ||
	   (sender() == ui_->doubleSpinBox_projMaxAngle && ui_->groupBox_gridFromProjection->isChecked()) ||
	   (sender() == ui_->spinBox_projClusterSize && ui_->groupBox_gridFromProjection->isChecked()))
	{
		localMaps_.clear();
		updateGraphView();
	}
}

Link DatabaseViewer::findActiveLink(int from, int to)
{
	Link link;
	std::multimap<int, Link>::iterator findIter = rtabmap::graph::findLink(linksRefined_, from ,to);
	if(findIter != linksRefined_.end())
	{
		link = findIter->second;
	}
	else
	{
		findIter = rtabmap::graph::findLink(linksAdded_, from ,to);
		if(findIter != linksAdded_.end())
		{
			link = findIter->second;
		}
		else if(!containsLink(linksRemoved_, from ,to))
		{
			findIter = rtabmap::graph::findLink(links_, from ,to);
			if(findIter != links_.end())
			{
				link = findIter->second;
			}
		}
	}
	return link;
}

bool DatabaseViewer::containsLink(std::multimap<int, Link> & links, int from, int to)
{
	return rtabmap::graph::findLink(links, from, to) != links.end();
}

void DatabaseViewer::refineConstraint()
{
	int from = ids_.at(ui_->horizontalSlider_A->value());
	int to = ids_.at(ui_->horizontalSlider_B->value());
	refineConstraint(from, to, false, true);
}

void DatabaseViewer::refineConstraint(int from, int to, bool silent, bool updateGraph)
{
	if(from == to)
	{
		UWARN("Cannot refine link to same node");
		return;
	}

	Link currentLink =  findActiveLink(from, to);
	if(!currentLink.isValid())
	{
		UERROR("Not found link! (%d->%d)", from, to);
		return;
	}
	Transform t = currentLink.transform();
	if(ui_->checkBox_showOptimized->isChecked() &&
	   (currentLink.type() == Link::kNeighbor || currentLink.type() == Link::kNeighborMerged) &&
	   graphes_.size() &&
	   (int)graphes_.size()-1 == ui_->horizontalSlider_iterations->maximum())
	{
		std::map<int, rtabmap::Transform> & graph = uValueAt(graphes_, ui_->horizontalSlider_iterations->value());
		if(currentLink.type() == Link::kNeighbor || currentLink.type() == Link::kNeighborMerged)
		{
			std::map<int, rtabmap::Transform>::iterator iterFrom = graph.find(currentLink.from());
			std::map<int, rtabmap::Transform>::iterator iterTo = graph.find(currentLink.to());
			if(iterFrom != graph.end() && iterTo != graph.end())
			{
				Transform topt = iterFrom->second.inverse()*iterTo->second;
				t = topt;
			}
		}
	}
	else if(ui_->checkBox_ignorePoseCorrection->isChecked() &&
			graph::findLink(linksRefined_, from, to) == linksRefined_.end())
	{
		if(currentLink.type() == Link::kNeighbor ||
		   currentLink.type() == Link::kNeighborMerged)
		{
			Transform poseFrom = uValue(poses_, currentLink.from(), Transform());
			Transform poseTo = uValue(poses_, currentLink.to(), Transform());
			if(!poseFrom.isNull() && !poseTo.isNull())
			{
				t  = poseFrom.inverse() * poseTo; // recompute raw odom transformation
			}
		}
	}

	Transform transform;
	RegistrationInfo info;

	SensorData dataFrom, dataTo;
	dbDriver_->getNodeData(currentLink.from(), dataFrom);
	dbDriver_->getNodeData(currentLink.to(), dataTo);

	ParametersMap parameters = ui_->parameters_toolbox->getParameters();

	UTimer timer;
	if(ui_->checkBox_icp_laserScan->isChecked())
	{
		// generate laser scans from depth image
		cv::Mat tmpA, tmpB, tmpC, tmpD;
		dataFrom.uncompressData(&tmpA, &tmpB, 0);
		dataTo.uncompressData(&tmpC, &tmpD, 0);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFrom = util3d::cloudFromSensorData(
				dataFrom,
				ui_->spinBox_icp_decimation->value(),
				ui_->doubleSpinBox_icp_maxDepth->value(),
				ui_->doubleSpinBox_icp_minDepth->value(),
				0,
				ui_->parameters_toolbox->getParameters());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTo = util3d::cloudFromSensorData(
				dataTo,
				ui_->spinBox_icp_decimation->value(),
				ui_->doubleSpinBox_icp_maxDepth->value(),
				ui_->doubleSpinBox_icp_minDepth->value(),
				0,
				ui_->parameters_toolbox->getParameters());
		int maxLaserScans = cloudFrom->size();
		dataFrom.setLaserScanRaw(util3d::laserScanFromPointCloud(*util3d::removeNaNFromPointCloud(cloudFrom), Transform()), maxLaserScans, 0);
		dataTo.setLaserScanRaw(util3d::laserScanFromPointCloud(*util3d::removeNaNFromPointCloud(cloudTo), Transform()), maxLaserScans, 0);

		if(!dataFrom.laserScanCompressed().empty() || !dataTo.laserScanCompressed().empty())
		{
			UWARN("There are laser scans in data, but generate laser scan from "
				  "depth image option is activated. Ignoring saved laser scans...");
		}
	}
	else
	{
		cv::Mat tmpA, tmpB;
		dataFrom.uncompressData(0, 0, &tmpA);
		dataTo.uncompressData(0, 0, &tmpB);
	}
	UINFO("Uncompress time: %f s", timer.ticks());

	RegistrationIcp registration(parameters);
	transform = registration.computeTransformation(dataFrom, dataTo, t, &info);
	UINFO("Icp time: %f s", timer.ticks());

	if(!transform.isNull())
	{
		Link newLink(currentLink.from(), currentLink.to(), currentLink.type(), transform, info.variance, info.variance);

		bool updated = false;
		std::multimap<int, Link>::iterator iter = linksRefined_.find(currentLink.from());
		while(iter != linksRefined_.end() && iter->first == currentLink.from())
		{
			if(iter->second.to() == currentLink.to() &&
			   iter->second.type() == currentLink.type())
			{
				iter->second = newLink;
				updated = true;
				break;
			}
			++iter;
		}
		if(!updated)
		{
			linksRefined_.insert(std::make_pair(newLink.from(), newLink));

			if(updateGraph)
			{
				this->updateGraphView();
			}
		}

		if(ui_->dockWidget_constraints->isVisible())
		{
			this->updateConstraintView(newLink, true);
		}
	}

	else if(!silent)
	{
		QMessageBox::warning(this,
				tr("Refine link"),
				tr("Cannot find a transformation between nodes %1 and %2").arg(from).arg(to));
	}
}

void DatabaseViewer::refineConstraintVisually()
{
	int from = ids_.at(ui_->horizontalSlider_A->value());
	int to = ids_.at(ui_->horizontalSlider_B->value());
	refineConstraintVisually(from, to, false, true);
}

void DatabaseViewer::refineConstraintVisually(int from, int to, bool silent, bool updateGraph)
{
	UDEBUG("");
	if(from == to)
	{
		UWARN("Cannot refine link to same node");
		return;
	}

	Link currentLink =  findActiveLink(from, to);
	if(!currentLink.isValid())
	{
		UERROR("Not found link! (%d->%d)", from, to);
		return;
	}

	ParametersMap parameters = ui_->parameters_toolbox->getParameters();

	Transform t;
	RegistrationInfo info;

	// Add sensor data to generate features
	SensorData dataFrom;
	dbDriver_->getNodeData(from, dataFrom);
	dataFrom.uncompressData();
	SensorData dataTo;
	dbDriver_->getNodeData(to, dataTo);
	dataTo.uncompressData();


	UDEBUG("");
	RegistrationVis reg(parameters);
	Signature fromS(dataFrom);
	Signature toS(dataTo);
	t = reg.computeTransformationMod(fromS, toS, currentLink.transform(), &info);
	UDEBUG("");

	if(!silent)
	{
		ui_->graphicsView_A->setFeatures(fromS.getWords(), dataFrom.depthRaw());
		ui_->graphicsView_B->setFeatures(toS.getWords(), dataTo.depthRaw());
		updateWordsMatching();
	}

	if(!t.isNull())
	{
		Link newLink(currentLink.from(), currentLink.to(), currentLink.type(), t, info.variance, info.variance);

		bool updated = false;
		std::multimap<int, Link>::iterator iter = linksRefined_.find(currentLink.from());
		while(iter != linksRefined_.end() && iter->first == currentLink.from())
		{
			if(iter->second.to() == currentLink.to() &&
			   iter->second.type() == currentLink.type())
			{
				iter->second = newLink;
				updated = true;
				break;
			}
			++iter;
		}
		if(!updated)
		{
			linksRefined_.insert(std::make_pair(newLink.from(), newLink));

			if(updateGraph)
			{
				this->updateGraphView();
			}
		}
		if(ui_->dockWidget_constraints->isVisible())
		{
			this->updateConstraintView(newLink, false);
		}
	}
	else if(!silent)
	{
		QMessageBox::warning(this,
				tr("Add link"),
				tr("Cannot find a transformation between nodes %1 and %2: %3").arg(from).arg(to).arg(info.rejectedMsg.c_str()));
	}
}

void DatabaseViewer::addConstraint()
{
	int from = ids_.at(ui_->horizontalSlider_A->value());
	int to = ids_.at(ui_->horizontalSlider_B->value());
	addConstraint(from, to, false, true);
}

bool DatabaseViewer::addConstraint(int from, int to, bool silent, bool updateGraph)
{
	if(from == to)
	{
		UWARN("Cannot add link to same node");
		return false;
	}

	bool updateSlider = false;
	if(!containsLink(linksAdded_, from, to) &&
	   !containsLink(links_, from, to))
	{
		UASSERT(!containsLink(linksRemoved_, from, to));
		UASSERT(!containsLink(linksRefined_, from, to));

		ParametersMap parameters = ui_->parameters_toolbox->getParameters();

		Transform t;
		RegistrationInfo info;

		// Add sensor data to generate features
		SensorData dataFrom;
		dbDriver_->getNodeData(from, dataFrom);
		dataFrom.uncompressData();
		SensorData dataTo;
		dbDriver_->getNodeData(to, dataTo);
		dataTo.uncompressData();


		UDEBUG("");
		RegistrationVis reg(parameters);
		Signature fromS(dataFrom);
		Signature toS(dataTo);
		t = reg.computeTransformationMod(fromS, toS, Transform::getIdentity(), &info);
		UDEBUG("");

		if(!silent)
		{
			ui_->graphicsView_A->setFeatures(fromS.getWords(), dataFrom.depthRaw());
			ui_->graphicsView_B->setFeatures(toS.getWords(), dataTo.depthRaw());
			updateWordsMatching();
		}
		
		if(!t.isNull())
		{
			// transform is valid, make a link
			if(from>to)
			{
				linksAdded_.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, t, info.variance, info.variance)));
			}
			else
			{
				linksAdded_.insert(std::make_pair(to, Link(to, from, Link::kUserClosure, t.inverse(), info.variance, info.variance)));
			}
			updateSlider = true;
		}
		else if(!silent)
		{
			QMessageBox::warning(this,
					tr("Add link"),
					tr("Cannot find a transformation between nodes %1 and %2: %3").arg(from).arg(to).arg(info.rejectedMsg.c_str()));
		}
	}
	else if(containsLink(linksRemoved_, from, to))
	{
		//simply remove from linksRemoved
		linksRemoved_.erase(rtabmap::graph::findLink(linksRemoved_, from, to));
		updateSlider = true;
	}

	if(updateSlider)
	{
		updateLoopClosuresSlider(from, to);
		if(updateGraph)
		{
			this->updateGraphView();
		}
	}
	return updateSlider;
}

void DatabaseViewer::resetConstraint()
{
	int from = ids_.at(ui_->horizontalSlider_A->value());
	int to = ids_.at(ui_->horizontalSlider_B->value());
	if(from < to)
	{
		int tmp = to;
		to = from;
		from = tmp;
	}

	if(from == to)
	{
		UWARN("Cannot reset link to same node");
		return;
	}


	std::multimap<int, Link>::iterator iter = rtabmap::graph::findLink(linksRefined_, from, to);
	if(iter != linksRefined_.end())
	{
		linksRefined_.erase(iter);
		this->updateGraphView();
	}

	iter = rtabmap::graph::findLink(links_, from, to);
	if(iter != links_.end())
	{
		this->updateConstraintView(iter->second);
	}
	iter = rtabmap::graph::findLink(linksAdded_, from, to);
	if(iter != linksAdded_.end())
	{
		this->updateConstraintView(iter->second);
	}
}

void DatabaseViewer::rejectConstraint()
{
	int from = ids_.at(ui_->horizontalSlider_A->value());
	int to = ids_.at(ui_->horizontalSlider_B->value());
	if(from < to)
	{
		int tmp = to;
		to = from;
		from = tmp;
	}

	if(from == to)
	{
		UWARN("Cannot reject link to same node");
		return;
	}

	bool removed = false;

	// find the original one
	std::multimap<int, Link>::iterator iter;
	iter = rtabmap::graph::findLink(links_, from, to);
	if(iter != links_.end())
	{
		if(iter->second.type() == Link::kNeighbor || iter->second.type() == Link::kNeighborMerged)
		{
			UWARN("Cannot reject neighbor links (%d->%d)", from, to);
			return;
		}
		linksRemoved_.insert(*iter);
		removed = true;
	}

	// remove from refined and added
	iter = rtabmap::graph::findLink(linksRefined_, from, to);
	if(iter != linksRefined_.end())
	{
		linksRefined_.erase(iter);
		removed = true;
	}
	iter = rtabmap::graph::findLink(linksAdded_, from, to);
	if(iter != linksAdded_.end())
	{
		linksAdded_.erase(iter);
		removed = true;
	}
	if(removed)
	{
		this->updateGraphView();
	}
	updateLoopClosuresSlider();
}

std::multimap<int, rtabmap::Link> DatabaseViewer::updateLinksWithModifications(
		const std::multimap<int, rtabmap::Link> & edgeConstraints)
{
	std::multimap<int, rtabmap::Link> links;
	for(std::multimap<int, rtabmap::Link>::const_iterator iter=edgeConstraints.begin();
		iter!=edgeConstraints.end();
		++iter)
	{
		std::multimap<int, rtabmap::Link>::iterator findIter;

		findIter = rtabmap::graph::findLink(linksRemoved_, iter->second.from(), iter->second.to());
		if(findIter != linksRemoved_.end())
		{
			if(!(iter->second.from() == findIter->second.from() &&
			   iter->second.to() == findIter->second.to() &&
			   iter->second.type() == findIter->second.type()))
			{
				UWARN("Links (%d->%d,%d) and (%d->%d,%d) are not equal!?",
						iter->second.from(), iter->second.to(), iter->second.type(),
						findIter->second.from(), findIter->second.to(), findIter->second.type());
			}
			else
			{
				//UINFO("Removed link (%d->%d, %d)", iter->second.from(), iter->second.to(), iter->second.type());
				continue; // don't add this link
			}
		}

		findIter = rtabmap::graph::findLink(linksRefined_, iter->second.from(), iter->second.to());
		if(findIter!=linksRefined_.end())
		{
			if(iter->second.from() == findIter->second.from() &&
			   iter->second.to() == findIter->second.to() &&
			   iter->second.type() == findIter->second.type())
			{
				links.insert(*findIter); // add the refined link
				//UINFO("Updated link (%d->%d, %d)", iter->second.from(), iter->second.to(), iter->second.type());
				continue;
			}
			else
			{
				UWARN("Links (%d->%d,%d) and (%d->%d,%d) are not equal!?",
						iter->second.from(), iter->second.to(), iter->second.type(),
						findIter->second.from(), findIter->second.to(), findIter->second.type());
			}
		}

		links.insert(*iter); // add original link
	}

	//look for added links
	for(std::multimap<int, rtabmap::Link>::const_iterator iter=linksAdded_.begin();
		iter!=linksAdded_.end();
		++iter)
	{
		//UINFO("Added link (%d->%d, %d)", iter->second.from(), iter->second.to(), iter->second.type());
		links.insert(*iter);
	}

	return links;
}

void DatabaseViewer::updateLoopClosuresSlider(int from, int to)
{
	int size = loopLinks_.size();
	loopLinks_.clear();
	std::multimap<int, Link> links = updateLinksWithModifications(links_);
	int position = ui_->horizontalSlider_loops->value();
	for(std::multimap<int, rtabmap::Link>::iterator iter = links.begin(); iter!=links.end(); ++iter)
	{
		if(!iter->second.transform().isNull())
		{
			if(iter->second.type() != rtabmap::Link::kNeighbor &&
			   iter->second.type() != rtabmap::Link::kNeighborMerged)
			{
				if((iter->second.from() == from && iter->second.to() == to) ||
				   (iter->second.to() == from && iter->second.from() == to))
				{
					position = loopLinks_.size();
				}
				loopLinks_.append(iter->second);
			}
		}
		else
		{
			UERROR("Transform null for link from %d to %d", iter->first, iter->second.to());
		}
	}

	if(loopLinks_.size())
	{
		if(loopLinks_.size() == 1)
		{
			// just to be able to move the cursor of the loop slider
			loopLinks_.push_back(loopLinks_.front());
		}
		ui_->horizontalSlider_loops->setMinimum(0);
		ui_->horizontalSlider_loops->setMaximum(loopLinks_.size()-1);
		ui_->horizontalSlider_loops->setEnabled(true);
		if(position != ui_->horizontalSlider_loops->value())
		{
			ui_->horizontalSlider_loops->setValue(position);
		}
		else if(size != loopLinks_.size())
		{
			this->updateConstraintView(loopLinks_.at(position));
		}
	}
	else
	{
		ui_->horizontalSlider_loops->setEnabled(false);
		constraintsViewer_->removeAllClouds();
		constraintsViewer_->update();
		updateConstraintButtons();
	}
}

void DatabaseViewer::notifyParametersChanged(const QStringList & parametersChanged)
{
	bool updateStereo = false;
	bool updateGraphView = false;
	for(QStringList::const_iterator iter=parametersChanged.constBegin();
	   iter!=parametersChanged.constEnd() && (!updateStereo || !updateGraphView);
	   ++iter)
	{
		QString group = iter->split('/').first();
		if(!updateStereo && group == "Stereo")
		{
			updateStereo = true;
			continue;
		}
		if(!updateGraphView && group == "Optimize")
		{
			updateGraphView = true;
			continue;
		}
	}

	if(updateStereo)
	{
		this->updateStereo();
	}
	if(updateGraphView)
	{
		this->updateGraphView();
	}
	this->configModified();
}

} // namespace rtabmap
