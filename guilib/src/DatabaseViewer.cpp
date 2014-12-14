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

#include "rtabmap/gui/DatabaseViewer.h"
#include "rtabmap/gui/CloudViewer.h"
#include "ui_DatabaseViewer.h"
#include <QtGui/QMessageBox>
#include <QtGui/QFileDialog>
#include <QtGui/QInputDialog>
#include <QtGui/QGraphicsLineItem>
#include <QtGui/QCloseEvent>
#include <QtGui/QGraphicsOpacityEffect>
#include <QtCore/QBuffer>
#include <QtCore/QTextStream>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <rtabmap/utilite/UTimer.h>
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/DBDriver.h"
#include "rtabmap/gui/KeypointItem.h"
#include "rtabmap/gui/UCv2Qt.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/gui/DataRecorder.h"
#include "rtabmap/core/SensorData.h"
#include "ExportDialog.h"
#include "DetailedProgressDialog.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

namespace rtabmap {

DatabaseViewer::DatabaseViewer(QWidget * parent) :
	QMainWindow(parent),
	memory_(0)
{
	pathDatabase_ = QDir::homePath()+"/Documents/RTAB-Map"; //use home directory by default

	if(!UDirectory::exists(pathDatabase_.toStdString()))
	{
		pathDatabase_ = QDir::homePath();
	}

	ui_ = new Ui_DatabaseViewer();
	ui_->setupUi(this);

	ui_->dockWidget_constraints->setVisible(false);
	ui_->dockWidget_graphView->setVisible(false);
	ui_->dockWidget_icp->setVisible(false);
	ui_->dockWidget_visual->setVisible(false);
	ui_->dockWidget_stereoView->setVisible(false);
	ui_->dockWidget_icp->setFloating(true);
	ui_->dockWidget_visual->setFloating(true);
	ui_->menuView->addAction(ui_->dockWidget_constraints->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_graphView->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_icp->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_visual->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_stereoView->toggleViewAction());
	connect(ui_->dockWidget_graphView->toggleViewAction(), SIGNAL(triggered()), this, SLOT(updateGraphView()));

	connect(ui_->actionQuit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui_->buttonBox, SIGNAL(rejected()), this, SLOT(close()));

	// connect actions with custom slots
	connect(ui_->actionOpen_database, SIGNAL(triggered()), this, SLOT(openDatabase()));
	connect(ui_->actionExport, SIGNAL(triggered()), this, SLOT(exportDatabase()));
	connect(ui_->actionExtract_images, SIGNAL(triggered()), this, SLOT(extractImages()));
	connect(ui_->actionGenerate_graph_dot, SIGNAL(triggered()), this, SLOT(generateGraph()));
	connect(ui_->actionGenerate_local_graph_dot, SIGNAL(triggered()), this, SLOT(generateLocalGraph()));
	connect(ui_->actionGenerate_TORO_graph_graph, SIGNAL(triggered()), this, SLOT(generateTOROGraph()));
	connect(ui_->actionView_3D_map, SIGNAL(triggered()), this, SLOT(view3DMap()));
	connect(ui_->actionGenerate_3D_map_pcd, SIGNAL(triggered()), this, SLOT(generate3DMap()));
	connect(ui_->actionDetect_more_loop_closures, SIGNAL(triggered()), this, SLOT(detectMoreLoopClosures()));
	connect(ui_->actionRefine_all_neighbor_links, SIGNAL(triggered()), this, SLOT(refineAllNeighborLinks()));
	connect(ui_->actionRefine_all_loop_closure_links, SIGNAL(triggered()), this, SLOT(refineAllLoopClosureLinks()));
	connect(ui_->actionVisual_Refine_all_neighbor_links, SIGNAL(triggered()), this, SLOT(refineVisuallyAllNeighborLinks()));
	connect(ui_->actionVisual_Refine_all_loop_closure_links, SIGNAL(triggered()), this, SLOT(refineVisuallyAllLoopClosureLinks()));

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

	ui_->horizontalSlider_A->setTracking(false);
	ui_->horizontalSlider_B->setTracking(false);
	ui_->horizontalSlider_A->setEnabled(false);
	ui_->horizontalSlider_B->setEnabled(false);
	connect(ui_->horizontalSlider_A, SIGNAL(valueChanged(int)), this, SLOT(sliderAValueChanged(int)));
	connect(ui_->horizontalSlider_B, SIGNAL(valueChanged(int)), this, SLOT(sliderBValueChanged(int)));
	connect(ui_->horizontalSlider_A, SIGNAL(sliderMoved(int)), this, SLOT(sliderAMoved(int)));
	connect(ui_->horizontalSlider_B, SIGNAL(sliderMoved(int)), this, SLOT(sliderBMoved(int)));

	ui_->horizontalSlider_neighbors->setTracking(false);
	ui_->horizontalSlider_loops->setTracking(false);
	ui_->horizontalSlider_neighbors->setEnabled(false);
	ui_->horizontalSlider_loops->setEnabled(false);
	connect(ui_->horizontalSlider_neighbors, SIGNAL(valueChanged(int)), this, SLOT(sliderNeighborValueChanged(int)));
	connect(ui_->horizontalSlider_loops, SIGNAL(valueChanged(int)), this, SLOT(sliderLoopValueChanged(int)));
	connect(ui_->horizontalSlider_neighbors, SIGNAL(sliderMoved(int)), this, SLOT(sliderNeighborValueChanged(int)));
	connect(ui_->horizontalSlider_loops, SIGNAL(sliderMoved(int)), this, SLOT(sliderLoopValueChanged(int)));
	connect(ui_->checkBox_showOptimized, SIGNAL(stateChanged(int)), this, SLOT(updateConstraintView()));
	ui_->checkBox_showOptimized->setEnabled(false);

	ui_->horizontalSlider_iterations->setTracking(false);
	ui_->dockWidget_graphView->setEnabled(false);
	connect(ui_->horizontalSlider_iterations, SIGNAL(valueChanged(int)), this, SLOT(sliderIterationsValueChanged(int)));
	connect(ui_->horizontalSlider_iterations, SIGNAL(sliderMoved(int)), this, SLOT(sliderIterationsValueChanged(int)));
	connect(ui_->spinBox_iterations, SIGNAL(editingFinished()), this, SLOT(updateGraphView()));
	connect(ui_->spinBox_optimizationsFrom, SIGNAL(editingFinished()), this, SLOT(updateGraphView()));
	connect(ui_->checkBox_initGuess, SIGNAL(stateChanged(int)), this, SLOT(updateGraphView()));
	connect(ui_->checkBox_ignoreCovariance, SIGNAL(stateChanged(int)), this, SLOT(updateGraphView()));

	ui_->constraintsViewer->setCameraLockZ(false);
	ui_->constraintsViewer->setCameraFree();
}

DatabaseViewer::~DatabaseViewer()
{
	delete ui_;
	if(memory_)
	{
		delete memory_;
	}
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
		if(memory_)
		{
			delete memory_;
			memory_ = 0;
			ids_.clear();
			idToIndex_.clear();
			neighborLinks_.clear();
			loopLinks_.clear();
			graphes_.clear();
			poses_.clear();
			links_.clear();
			linksAdded_.clear();
			linksRefined_.clear();
			linksRemoved_.clear();
			scans_.clear();
			ui_->actionGenerate_TORO_graph_graph->setEnabled(false);
			ui_->checkBox_showOptimized->setEnabled(false);
		}

		std::string driverType = "sqlite3";
		rtabmap::ParametersMap parameters;
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kDbSqlite3InMemory(), "false"));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemInitWMWithAllNodes(), "true"));
		// use BruteForce dictionary because we don't know which type of descriptors are saved in database
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNNStrategy(), "3"));

		memory_ = new rtabmap::Memory();

		if(!memory_->init(path.toStdString(), false, parameters))
		{
			QMessageBox::warning(this, "Database error", tr("Can't open database \"%1\"").arg(path));
		}
		else
		{
			pathDatabase_ = UDirectory::getDir(path.toStdString()).c_str();
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
				std::multimap<int, rtabmap::Link>::iterator refinedIter = util3d::findLink(linksRefined_, iter->second.from(), iter->second.to());
				if(refinedIter != linksRefined_.end())
				{
					memory_->addLoopClosureLink(refinedIter->second.to(), refinedIter->second.from(), refinedIter->second.transform(), refinedIter->second.type(), refinedIter->second.variance());
				}
				else
				{
					memory_->addLoopClosureLink(iter->second.to(), iter->second.from(), iter->second.transform(), iter->second.type(), iter->second.variance());
				}
			}

			//Refined links
			for(std::multimap<int, rtabmap::Link>::iterator iter=linksRefined_.begin(); iter!=linksRefined_.end(); ++iter)
			{
				if(!containsLink(linksAdded_, iter->second.from(), iter->second.to()))
				{
					memory_->rejectLoopClosure(iter->second.to(), iter->second.from());
					memory_->addLoopClosureLink(iter->second.to(), iter->second.from(), iter->second.transform(), iter->second.type(), iter->second.variance());
				}
			}

			// Rejected links
			for(std::multimap<int, rtabmap::Link>::iterator iter=linksRemoved_.begin(); iter!=linksRemoved_.end(); ++iter)
			{
				memory_->rejectLoopClosure(iter->second.to(), iter->second.from());
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
		if(memory_)
		{
			delete memory_;
			memory_ = 0;
		}
	}
}

void DatabaseViewer::showEvent(QShowEvent* anEvent)
{
	ui_->graphicsView_A->fitInView(ui_->graphicsView_A->sceneRect(), Qt::KeepAspectRatio);
	ui_->graphicsView_B->fitInView(ui_->graphicsView_B->sceneRect(), Qt::KeepAspectRatio);
	ui_->graphicsView_A->resetZoom();
	ui_->graphicsView_B->resetZoom();
}

void DatabaseViewer::resizeEvent(QResizeEvent* anEvent)
{
	ui_->graphicsView_A->fitInView(ui_->graphicsView_A->sceneRect(), Qt::KeepAspectRatio);
	ui_->graphicsView_B->fitInView(ui_->graphicsView_B->sceneRect(), Qt::KeepAspectRatio);
	ui_->graphicsView_A->resetZoom();
	ui_->graphicsView_B->resetZoom();
}


void DatabaseViewer::exportDatabase()
{
	if(!memory_ || ids_.size() == 0)
	{
		return;
	}

	rtabmap::ExportDialog dialog;

	if(dialog.exec())
	{
		if(!dialog.outputPath().isEmpty())
		{
			int framesIgnored = dialog.framesIgnored();
			QString path = dialog.outputPath();
			rtabmap::DataRecorder recorder;
			if(recorder.init(path, false))
			{
				rtabmap::DetailedProgressDialog progressDialog(this);
				progressDialog.setMaximumSteps(ids_.size() / (1+framesIgnored) + 1);
				progressDialog.show();

				for(int i=0; i<ids_.size(); i+=1+framesIgnored)
				{
					int id = ids_.at(i);

					Signature data = memory_->getSignatureData(id, true);
					rtabmap::SensorData sensorData = data.toSensorData();
					recorder.addData(sensorData);

					progressDialog.appendText(tr("Exported node %1").arg(id));
					progressDialog.incrementStep();
					QApplication::processEvents();
				}
				progressDialog.setValue(progressDialog.maximumSteps());
				progressDialog.appendText("Export finished!");
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
	if(!memory_ || ids_.size() == 0)
	{
		return;
	}

	QString path = QFileDialog::getExistingDirectory(this, tr("Select directory where to save images..."), QDir::homePath());
	if(!path.isNull())
	{
		for(int i=0; i<ids_.size(); i+=1)
		{
			int id = ids_.at(i);
			cv::Mat compressedRgb = memory_->getImageCompressed(id);
			if(!compressedRgb.empty())
			{
				cv::Mat imageMat = rtabmap::util3d::uncompressImage(compressedRgb);
				cv::imwrite(QString("%1/%2.png").arg(path).arg(id).toStdString(), imageMat);
				UINFO(QString("Saved %1/%2.png").arg(path).arg(id).toStdString().c_str());
			}
		}
	}
}

void DatabaseViewer::updateIds()
{
	if(!memory_)
	{
		return;
	}

	std::set<int> ids = memory_->getAllSignatureIds();
	ids_ = QList<int>::fromStdList(std::list<int>(ids.begin(), ids.end()));
	idToIndex_.clear();
	for(int i=0; i<ids_.size(); ++i)
	{
		idToIndex_.insert(ids_[i], i);
	}

	poses_.clear();
	links_.clear();
	linksAdded_.clear();
	linksRefined_.clear();
	linksRemoved_.clear();
	if(memory_->getLastWorkingSignature())
	{
		std::map<int, int> nids = memory_->getNeighborsId(memory_->getLastWorkingSignature()->id(), 0, -1, true);
		memory_->getMetricConstraints(uKeys(nids), poses_, links_, true);

		int first = nids.begin()->first;
		ui_->spinBox_optimizationsFrom->setRange(first, memory_->getLastWorkingSignature()->id());
		ui_->spinBox_optimizationsFrom->setValue(first);
	}

	ui_->actionGenerate_TORO_graph_graph->setEnabled(false);
	graphes_.clear();
	neighborLinks_.clear();
	loopLinks_.clear();
	for(std::multimap<int, rtabmap::Link>::iterator iter = links_.begin(); iter!=links_.end(); ++iter)
	{
		if(!iter->second.transform().isNull())
		{
			if(iter->second.type() == rtabmap::Link::kNeighbor)
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

	UINFO("Loaded %d ids", ids_.size());

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
		updateGraphView();
	}
}

void DatabaseViewer::generateGraph()
{
	if(!memory_)
	{
		QMessageBox::warning(this, tr("Cannot generate a graph"), tr("A database must must loaded first...\nUse File->Open database."));
		return;
	}

	QString path = QFileDialog::getSaveFileName(this, tr("Save File"), pathDatabase_+"/Graph.dot", tr("Graphiz file (*.dot)"));
	if(!path.isEmpty())
	{
		memory_->generateGraph(path.toStdString());
	}
}

void DatabaseViewer::generateLocalGraph()
{
	if(!ids_.size() || !memory_)
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
			if(!path.isEmpty())
			{
				std::map<int, int> ids = memory_->getNeighborsId(id, margin, -1, false);

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
					memory_->generateGraph(path.toStdString(), idsSet);
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
	std::multimap<int, Link> links = updateLinksWithModifications(links_);
	if(!graphes_.size() || !links.size())
	{
		QMessageBox::warning(this, tr("Cannot generate a TORO graph"), tr("No poses or no links..."));
		return;
	}
	bool ok = false;
	int id = QInputDialog::getInt(this, tr("Which iteration?"), tr("Iteration (0 -> %1)").arg((int)graphes_.size()-1), (int)graphes_.size()-1, 0, graphes_.size()-1, 1, &ok);

	if(ok)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save File"), pathDatabase_+"/constraints" + QString::number(id) + ".graph", tr("TORO file (*.graph)"));
		if(!path.isEmpty())
		{
			rtabmap::util3d::saveTOROGraph(path.toStdString(), uValueAt(graphes_, id), links);
		}
	}
}

void DatabaseViewer::view3DMap()
{
	if(!ids_.size() || !memory_)
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
		double maxDepth = QInputDialog::getDouble(this, tr("Camera depth?"), tr("Maximum depth (m, 0=no max):"), 4.0, 0, 10, 2, &ok);
		if(ok)
		{
			const std::map<int, Transform> & optimizedPoses = uValueAt(graphes_, ui_->horizontalSlider_iterations->value());
			if(optimizedPoses.size() > 0)
			{
				rtabmap::DetailedProgressDialog progressDialog(this);
				progressDialog.setMaximumSteps(optimizedPoses.size());
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
						Signature data = memory_->getSignatureData(iter->first, true);
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
						UASSERT(data.getImageRaw().empty() || data.getImageRaw().type()==CV_8UC3 || data.getImageRaw().type() == CV_8UC1);
						UASSERT(data.getDepthRaw().empty() || data.getDepthRaw().type()==CV_8UC1 || data.getDepthRaw().type() == CV_16UC1 || data.getDepthRaw().type() == CV_32FC1);
						if(data.getDepthRaw().type() == CV_8UC1)
						{
							cv::Mat leftImg;
							if(data.getImageRaw().channels() == 3)
							{
								cv::cvtColor(data.getImageRaw(), leftImg, CV_BGR2GRAY);
							}
							else
							{
								leftImg = data.getImageRaw();
							}
							cloud = rtabmap::util3d::cloudFromDisparityRGB(
									data.getImageRaw(),
								util3d::disparityFromStereoImages(leftImg, data.getDepthRaw()),
								data.getDepthCx(), data.getDepthCy(),
								data.getDepthFx(), data.getDepthFy(),
								decimation);
						}
						else
						{
							cloud = rtabmap::util3d::cloudFromDepthRGB(
									data.getImageRaw(),
									data.getDepthRaw(),
									data.getDepthCx(), data.getDepthCy(),
									data.getDepthFx(), data.getDepthFy(),
									decimation);
						}

						if(maxDepth)
						{
							cloud = rtabmap::util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, maxDepth);
						}

						cloud = rtabmap::util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, data.getLocalTransform());

						QColor color = Qt::red;
						int mapId = memory_->getMapId(iter->first);
						if(mapId >= 0)
						{
							color = (Qt::GlobalColor)(mapId % 12 + 7 );
						}
						viewer->addCloud(uFormat("cloud%d", iter->first), cloud, pose, color);

						UINFO("Generated %d (%d points)", iter->first, cloud->size());
						progressDialog.appendText(QString("Generated %1 (%2 points)").arg(iter->first).arg(cloud->size()));
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

void DatabaseViewer::generate3DMap()
{
	if(!ids_.size() || !memory_)
	{
		QMessageBox::warning(this, tr("Cannot generate a graph"), tr("The database is empty..."));
		return;
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
		double maxDepth = QInputDialog::getDouble(this, tr("Camera depth?"), tr("Maximum depth (m, 0=no max):"), 4.0, 0, 10, 2, &ok);
		if(ok)
		{
			QString path = QFileDialog::getExistingDirectory(this, tr("Save directory"), pathDatabase_);
			if(!path.isEmpty())
			{
				const std::map<int, Transform> & optimizedPoses = uValueAt(graphes_, ui_->horizontalSlider_iterations->value());
				if(optimizedPoses.size() > 0)
				{
					rtabmap::DetailedProgressDialog progressDialog;
					progressDialog.setMaximumSteps((int)optimizedPoses.size());
					progressDialog.show();

					for(std::map<int, Transform>::const_iterator iter = optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
					{
						const rtabmap::Transform & pose = iter->second;
						if(!pose.isNull())
						{
							Signature data = memory_->getSignatureData(iter->first, true);
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
							UASSERT(data.getImageRaw().empty() || data.getImageRaw().type()==CV_8UC3 || data.getImageRaw().type() == CV_8UC1);
							UASSERT(data.getDepthRaw().empty() || data.getDepthRaw().type()==CV_8UC1 || data.getDepthRaw().type() == CV_16UC1 || data.getDepthRaw().type() == CV_32FC1);
							if(data.getDepthRaw().type() == CV_8UC1)
							{
								cv::Mat leftImg;
								if(data.getImageRaw().channels() == 3)
								{
									cv::cvtColor(data.getImageRaw(), leftImg, CV_BGR2GRAY);
								}
								else
								{
									leftImg = data.getImageRaw();
								}
								cloud = rtabmap::util3d::cloudFromDisparityRGB(
										data.getImageRaw(),
									util3d::disparityFromStereoImages(leftImg, data.getDepthRaw()),
									data.getDepthCx(), data.getDepthCy(),
									data.getDepthFx(), data.getDepthFy(),
									decimation);
							}
							else
							{
								cloud = rtabmap::util3d::cloudFromDepthRGB(
										data.getImageRaw(),
										data.getDepthRaw(),
										data.getDepthCx(), data.getDepthCy(),
										data.getDepthFx(), data.getDepthFy(),
										decimation);
							}

							if(maxDepth)
							{
								cloud = rtabmap::util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, maxDepth);
							}

							cloud = rtabmap::util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, pose*data.getLocalTransform());
							std::string name = uFormat("%s/node%d.pcd", path.toStdString().c_str(), iter->first);
							pcl::io::savePCDFile(name, *cloud);
							UINFO("Saved %s (%d points)", name.c_str(), cloud->size());
							progressDialog.appendText(QString("Saved %1 (%2 points)").arg(name.c_str()).arg(cloud->size()));
							progressDialog.incrementStep();
							QApplication::processEvents();
						}
					}
					progressDialog.setValue(progressDialog.maximumSteps());

					QMessageBox::information(this, tr("Finished"), tr("%1 clouds generated to %2.").arg(optimizedPoses.size()).arg(path));
				}
				else
				{
					QMessageBox::critical(this, tr("Error"), tr("No neighbors found for node %1.").arg(ui_->spinBox_optimizationsFrom->value()));
				}
			}
		}
	}
}

void DatabaseViewer::detectMoreLoopClosures()
{
	const std::map<int, Transform> & optimizedPoses = graphes_.back();

	int iterations = ui_->doubleSpinBox_detectMore_iterations->value();
	UASSERT(iterations > 0);
	int added = 0;
	for(int n=0; n<iterations; ++n)
	{
		UINFO("iteration %d/%d", n+1, iterations);
		std::multimap<int, int> clusters = util3d::radiusPosesClustering(
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
		rtabmap::DetailedProgressDialog progressDialog(this);
		progressDialog.setMaximumSteps(neighborLinks_.size());
		progressDialog.show();

		for(int i=0; i<neighborLinks_.size(); ++i)
		{
			int from = neighborLinks_[i].from();
			int to = neighborLinks_[i].to();
			this->refineConstraint(neighborLinks_[i].from(), neighborLinks_[i].to(), false);

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
		rtabmap::DetailedProgressDialog progressDialog(this);
		progressDialog.setMaximumSteps(loopLinks_.size());
		progressDialog.show();

		for(int i=0; i<loopLinks_.size(); ++i)
		{
			int from = loopLinks_[i].from();
			int to = loopLinks_[i].to();
			this->refineConstraint(loopLinks_[i].from(), loopLinks_[i].to(), false);

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
		rtabmap::DetailedProgressDialog progressDialog(this);
		progressDialog.setMaximumSteps(neighborLinks_.size());
		progressDialog.show();

		for(int i=0; i<neighborLinks_.size(); ++i)
		{
			int from = neighborLinks_[i].from();
			int to = neighborLinks_[i].to();
			this->refineConstraintVisually(neighborLinks_[i].from(), neighborLinks_[i].to(), false);

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
		rtabmap::DetailedProgressDialog progressDialog(this);
		progressDialog.setMaximumSteps(loopLinks_.size());
		progressDialog.show();

		for(int i=0; i<loopLinks_.size(); ++i)
		{
			int from = loopLinks_[i].from();
			int to = loopLinks_[i].to();
			this->refineConstraintVisually(loopLinks_[i].from(), loopLinks_[i].to(), false);

			progressDialog.appendText(tr("Refined link %1->%2 (%3/%4)").arg(from).arg(to).arg(i+1).arg(loopLinks_.size()));
			progressDialog.incrementStep();
			QApplication::processEvents();
		}
		this->updateGraphView();

		progressDialog.setValue(progressDialog.maximumSteps());
		progressDialog.appendText("Refining links finished!");
	}
}

void DatabaseViewer::sliderAValueChanged(int value)
{
	this->update(value,
			ui_->label_indexA,
			ui_->label_parentsA,
			ui_->label_childrenA,
			ui_->graphicsView_A,
			ui_->label_idA);
}

void DatabaseViewer::sliderBValueChanged(int value)
{
	this->update(value,
			ui_->label_indexB,
			ui_->label_parentsB,
			ui_->label_childrenB,
			ui_->graphicsView_B,
			ui_->label_idB);
}

void DatabaseViewer::update(int value,
						QLabel * labelIndex,
						QLabel * labelParents,
						QLabel * labelChildren,
						rtabmap::ImageView * view,
						QLabel * labelId,
						bool updateConstraintView)
{
	UTimer timer;
	labelIndex->setText(QString::number(value));
	labelParents->clear();
	labelChildren->clear();
	QRectF rect;
	if(value >= 0 && value < ids_.size())
	{
		view->clear();
		view->resetTransform();
		int id = ids_.at(value);
		int mapId = -1;
		labelId->setText(QString::number(id));
		if(id>0)
		{
			//image
			QImage img;
			QImage imgDepth;
			if(memory_)
			{
				Signature data = memory_->getSignatureData(id, true);
				if(!data.getImageRaw().empty())
				{
					img = uCvMat2QImage(data.getImageRaw());
				}
				if(!data.getDepthRaw().empty())
				{
					imgDepth = uCvMat2QImage(data.getDepthRaw());
				}

				if(data.getWords().size())
				{
					view->setFeatures(data.getWords());
				}

				mapId = memory_->getMapId(id);

				//stereo
				if(!data.getDepthRaw().empty() && data.getDepthRaw().type() == CV_8UC1)
				{
					this->updateStereo(&data);
				}
			}

			if(!imgDepth.isNull())
			{
				view->setImageDepth(imgDepth);
				rect = imgDepth.rect();
			}
			else
			{
				ULOGGER_DEBUG("Image depth is empty");
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

			// loops
			std::map<int, rtabmap::Link> loopClosures;
			loopClosures = memory_->getLoopClosureLinks(id, true);
			if(loopClosures.size())
			{
				QString strParents, strChildren;
				for(std::map<int, rtabmap::Link>::iterator iter=loopClosures.begin(); iter!=loopClosures.end(); ++iter)
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
				labelParents->setText(strParents);
				labelChildren->setText(strChildren);
			}
		}

		if(mapId>=0)
		{
			labelId->setText(QString("%1 [%2]").arg(id).arg(mapId));
		}
		else
		{
			labelId->setText(QString::number(id));
		}
	}
	else
	{
		ULOGGER_ERROR("Slider index out of range ?");
	}

	updateConstraintButtons();
	updateWordsMatching();

	if(updateConstraintView)
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
						this->updateConstraintView(loopLinks_.at(i),
								pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>),
								pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>),
								false);
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
						this->updateConstraintView(neighborLinks_.at(i),
								pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>),
								pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>),
								false);
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
			ui_->constraintsViewer->removeAllClouds();
			ui_->constraintsViewer->render();
			ui_->horizontalSlider_loops->blockSignals(false);
			ui_->horizontalSlider_neighbors->blockSignals(false);
		}
	}

	if(rect.isValid())
	{
		view->setSceneRect(rect);
	}
	else
	{
		view->setSceneRect(view->scene()->itemsBoundingRect());
	}
	view->fitInView(view->sceneRect(), Qt::KeepAspectRatio);
}

void DatabaseViewer::updateStereo(const Signature * data)
{
	if(data && ui_->dockWidget_stereoView->isVisible() && !data->getImageRaw().empty() && !data->getDepthRaw().empty() && data->getDepthRaw().type() == CV_8UC1)
	{
		cv::Mat leftMono;
		if(data->getImageRaw().channels() == 3)
		{
			cv::cvtColor(data->getImageRaw(), leftMono, CV_BGR2GRAY);
		}
		else
		{
			leftMono = data->getImageRaw();
		}

		UTimer timer;

		// generate kpts
		std::vector<cv::KeyPoint> kpts;
		cv::Rect roi = Feature2D::computeRoi(leftMono, "0.03 0.03 0.04 0.04");
		ParametersMap parameters;
		parameters.insert(ParametersPair(Parameters::kGFTTMaxCorners(), "1000"));
		parameters.insert(ParametersPair(Parameters::kGFTTMinDistance(), "5"));
		Feature2D::Type type = Feature2D::kFeatureGfttBrief;
		Feature2D * kptDetector = Feature2D::create(type, parameters);
		kpts = kptDetector->generateKeypoints(leftMono, 0, roi);
		delete kptDetector;

		float timeKpt = timer.ticks();

		std::vector<cv::Point2f> leftCorners;
		cv::KeyPoint::convert(kpts, leftCorners);

		// Find features in the new left image
		std::vector<unsigned char> status;
		std::vector<float> err;
		std::vector<cv::Point2f> rightCorners;
		cv::calcOpticalFlowPyrLK(
				leftMono,
				data->getDepthRaw(),
				leftCorners,
				rightCorners,
				status,
				err,
				cv::Size(Parameters::defaultStereoWinSize(), Parameters::defaultStereoWinSize()), Parameters::defaultStereoMaxLevel(),
				cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, Parameters::defaultStereoIterations(), Parameters::defaultStereoEps()));

		float timeFlow = timer.ticks();

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(kpts.size());
		float bad_point = std::numeric_limits<float>::quiet_NaN ();
		UASSERT(status.size() == kpts.size());
		int oi = 0;
		for(unsigned int i=0; i<status.size(); ++i)
		{
			pcl::PointXYZ pt(bad_point, bad_point, bad_point);
			if(status[i])
			{
				float disparity = leftCorners[i].x - rightCorners[i].x;
				if(disparity > 0.0f)
				{
					if(fabs((leftCorners[i].y-rightCorners[i].y) / (leftCorners[i].x-rightCorners[i].x)) < Parameters::defaultStereoMaxSlope())
					{
						pcl::PointXYZ tmpPt = util3d::projectDisparityTo3D(
								leftCorners[i],
								disparity,
								data->getDepthCx(), data->getDepthCy(), data->getDepthFx(), data->getDepthFy());

						if(pcl::isFinite(tmpPt))
						{
							pt = pcl::transformPoint(tmpPt, util3d::transformToEigen3f(data->getLocalTransform()));
							if(fabs(pt.x) > 2 || fabs(pt.y) > 2 || fabs(pt.z) > 2)
							{
								status[i] = 100; //blue
							}
							cloud->at(oi++) = pt;
						}
					}
					else
					{
						status[i] = 101; //yellow
					}
				}
				else
				{
					status[i] = 102; //magenta
				}
			}
		}
		cloud->resize(oi);

		UINFO("correspondences = %d/%d (%f) (time kpt=%fs flow=%fs)",
				(int)cloud->size(), (int)leftCorners.size(), float(cloud->size())/float(leftCorners.size()), timeKpt, timeFlow);

		ui_->stereoViewer->updateCameraPosition(Transform::getIdentity());
		ui_->stereoViewer->addOrUpdateCloud("stereo", cloud);
		ui_->stereoViewer->render();

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

		ui_->graphicsView_stereo->scene()->clear();
		ui_->graphicsView_stereo->setSceneRect(0,0,(float)leftMono.cols, (float)leftMono.rows);
		ui_->graphicsView_stereo->setLinesShown(true);
		ui_->graphicsView_stereo->setFeaturesShown(false);

		QGraphicsPixmapItem * item1 = ui_->graphicsView_stereo->scene()->addPixmap(QPixmap::fromImage(uCvMat2QImage(data->getImageRaw())));
		QGraphicsPixmapItem * item2 = ui_->graphicsView_stereo->scene()->addPixmap(QPixmap::fromImage(uCvMat2QImage(data->getDepthRaw())));

		QGraphicsOpacityEffect * effect1 = new QGraphicsOpacityEffect();
		QGraphicsOpacityEffect * effect2 = new QGraphicsOpacityEffect();
		effect1->setOpacity(0.5);
		effect2->setOpacity(0.5);
		item1->setGraphicsEffect(effect1);
		item2->setGraphicsEffect(effect2);

		// Draw lines between corresponding features...
		for(unsigned int i=0; i<kpts.size(); ++i)
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
			QGraphicsLineItem * item = ui_->graphicsView_stereo->scene()->addLine(
					kpts[i].pt.x,
					kpts[i].pt.y,
					rightKpts[i].pt.x,
					rightKpts[i].pt.y,
					QPen(c));
			item->setZValue(1);
		}
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
					ui_->graphicsView_A->setFeatureColor(ids[i], QColor(255, 0, 255, alpha));
					ui_->graphicsView_B->setFeatureColor(ids[i], QColor(255, 0, 255, alpha));

					// Add lines
					// Draw lines between corresponding features...
					float deltaX = ui_->graphicsView_A->sceneRect().width();
					float deltaY = 0;

					const KeypointItem * kptA = wordsA.value(ids[i]);
					const KeypointItem * kptB = wordsB.value(ids[i]);
					QGraphicsLineItem * item = ui_->graphicsView_A->scene()->addLine(
							kptA->rect().x()+kptA->rect().width()/2,
							kptA->rect().y()+kptA->rect().height()/2,
							kptB->rect().x()+kptB->rect().width()/2+deltaX,
							kptB->rect().y()+kptB->rect().height()/2+deltaY,
							QPen(QColor(0, 255, 255, alpha)));
					item->setVisible(ui_->graphicsView_A->isLinesShown());
					item->setZValue(1);

					item = ui_->graphicsView_B->scene()->addLine(
							kptA->rect().x()+kptA->rect().width()/2-deltaX,
							kptA->rect().y()+kptA->rect().height()/2-deltaY,
							kptB->rect().x()+kptB->rect().width()/2,
							kptB->rect().y()+kptB->rect().height()/2,
							QPen(QColor(0, 255, 255, alpha)));
					item->setVisible(ui_->graphicsView_B->isLinesShown());
					item->setZValue(1);
				}
			}
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
	this->updateConstraintView(neighborLinks_.at(ui_->horizontalSlider_neighbors->value()),
			pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>),
			pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>),
			false);
}

void DatabaseViewer::updateConstraintView(const rtabmap::Link & linkIn,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudFrom,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudTo,
		bool updateImageSliders)
{
	std::multimap<int, Link>::iterator iter = util3d::findLink(linksRefined_, linkIn.from(), linkIn.to());
	rtabmap::Link link = linkIn;
	if(iter != linksRefined_.end())
	{
		link = iter->second;
	}
	rtabmap::Transform t = link.transform();

	ui_->label_constraint->clear();
	ui_->label_constraint_opt->clear();
	ui_->checkBox_showOptimized->setEnabled(false);
	UASSERT(!t.isNull() && memory_);

	ui_->label_constraint->setText(QString("%1 (%2=%3)").arg(t.prettyPrint().c_str()).arg(QChar(0xc3, 0x03)).arg(sqrt(link.variance())));
	if(link.type() == Link::kNeighbor &&
	   graphes_.size() &&
	   (int)graphes_.size()-1 == ui_->horizontalSlider_iterations->maximum())
	{
		std::map<int, rtabmap::Transform> & graph = uValueAt(graphes_, ui_->horizontalSlider_iterations->value());
		if(link.type() == Link::kNeighbor)
		{
			std::map<int, rtabmap::Transform>::iterator iterFrom = graph.find(link.from());
			std::map<int, rtabmap::Transform>::iterator iterTo = graph.find(link.to());
			if(iterFrom != graph.end() && iterTo != graph.end())
			{
				ui_->checkBox_showOptimized->setEnabled(true);
				Transform topt = iterFrom->second.inverse()*iterTo->second;
				Transform delta = t.inverse()*topt;
				Transform v1 = t.rotation()*Transform(1,0,0,0,0,0);
				Transform v2 = topt.rotation()*Transform(1,0,0,0,0,0);
				float a = pcl::getAngle3D(Eigen::Vector4f(v1.x(), v1.y(), v1.z(), 0), Eigen::Vector4f(v2.x(), v2.y(), v2.z(), 0));
				a = (a *180.0f) / CV_PI;
				ui_->label_constraint_opt->setText(QString("%1 (error=%2% a=%3)").arg(topt.prettyPrint().c_str()).arg((delta.getNorm()/t.getNorm())*100.0f).arg(a));

				if(ui_->checkBox_showOptimized->isChecked())
				{
					t = topt;
				}
			}
		}
	}

	if(updateImageSliders)
	{
		bool updateA = false;
		bool updateB = false;
		ui_->horizontalSlider_A->blockSignals(true);
		ui_->horizontalSlider_B->blockSignals(true);
		// set from on left and to on right
		if(ui_->horizontalSlider_A->value() != idToIndex_.value(link.from()))
		{
			ui_->horizontalSlider_A->setValue(idToIndex_.value(link.from()));
			updateA=true;
		}
		if(ui_->horizontalSlider_B->value() != idToIndex_.value(link.to()))
		{
			ui_->horizontalSlider_B->setValue(idToIndex_.value(link.to()));
			updateB=true;
		}
		ui_->horizontalSlider_A->blockSignals(false);
		ui_->horizontalSlider_B->blockSignals(false);
		if(updateA)
		{
			this->update(idToIndex_.value(link.from()),
						ui_->label_indexA,
						ui_->label_parentsA,
						ui_->label_childrenA,
						ui_->graphicsView_A,
						ui_->label_idA,
						false); // don't update constraints view!
		}
		if(updateB)
		{
			this->update(idToIndex_.value(link.to()),
						ui_->label_indexB,
						ui_->label_parentsB,
						ui_->label_childrenB,
						ui_->graphicsView_B,
						ui_->label_idB,
						false); // don't update constraints view!
		}
	}

	if(ui_->constraintsViewer->isVisible())
	{
		if(cloudFrom->size() == 0 && cloudTo->size() == 0)
		{
			Signature dataFrom, dataTo;

			dataFrom = memory_->getSignatureData(link.from(), true);
			UASSERT(dataFrom.getImageRaw().empty() || dataFrom.getImageRaw().type()==CV_8UC3 || dataFrom.getImageRaw().type() == CV_8UC1);
			UASSERT(dataFrom.getDepthRaw().empty() || dataFrom.getDepthRaw().type()==CV_8UC1 || dataFrom.getDepthRaw().type() == CV_16UC1 || dataFrom.getDepthRaw().type() == CV_32FC1);

			dataTo = memory_->getSignatureData(link.to(), true);
			UASSERT(dataTo.getImageRaw().empty() || dataTo.getImageRaw().type()==CV_8UC3 || dataTo.getImageRaw().type() == CV_8UC1);
			UASSERT(dataTo.getDepthRaw().empty() || dataTo.getDepthRaw().type()==CV_8UC1 || dataTo.getDepthRaw().type() == CV_16UC1 || dataTo.getDepthRaw().type() == CV_32FC1);


			//cloud 3d
			if(!ui_->checkBox_show3DWords->isChecked())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFrom;
				if(dataFrom.getDepthRaw().type() == CV_8UC1)
				{
					cloudFrom = rtabmap::util3d::cloudFromStereoImages(
							dataFrom.getImageRaw(),
							dataFrom.getDepthRaw(),
							dataFrom.getDepthCx(), dataFrom.getDepthCy(),
							dataFrom.getDepthFx(), dataFrom.getDepthFy(),
							1);
				}
				else
				{
					cloudFrom = rtabmap::util3d::cloudFromDepthRGB(
							dataFrom.getImageRaw(),
							dataFrom.getDepthRaw(),
							dataFrom.getDepthCx(), dataFrom.getDepthCy(),
							dataFrom.getDepthFx(), dataFrom.getDepthFy(),
							1);
				}

				cloudFrom = rtabmap::util3d::removeNaNFromPointCloud<pcl::PointXYZRGB>(cloudFrom);
				cloudFrom = rtabmap::util3d::transformPointCloud<pcl::PointXYZRGB>(cloudFrom, dataFrom.getLocalTransform());

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTo;
				if(dataTo.getDepthRaw().type() == CV_8UC1)
				{
					cloudTo = rtabmap::util3d::cloudFromStereoImages(
							dataTo.getImageRaw(),
							dataTo.getDepthRaw(),
							dataTo.getDepthCx(), dataTo.getDepthCy(),
							dataTo.getDepthFx(), dataTo.getDepthFy(),
							1);
				}
				else
				{
					cloudTo = rtabmap::util3d::cloudFromDepthRGB(
							dataTo.getImageRaw(),
							dataTo.getDepthRaw(),
							dataTo.getDepthCx(), dataTo.getDepthCy(),
							dataTo.getDepthFx(), dataTo.getDepthFy(),
							1);
				}

				cloudTo = rtabmap::util3d::removeNaNFromPointCloud<pcl::PointXYZRGB>(cloudTo);
				cloudTo = rtabmap::util3d::transformPointCloud<pcl::PointXYZRGB>(cloudTo, t*dataTo.getLocalTransform());

				if(cloudFrom->size())
				{
					ui_->constraintsViewer->addOrUpdateCloud("cloud0", cloudFrom);
				}
				if(cloudTo->size())
				{
					ui_->constraintsViewer->addOrUpdateCloud("cloud1", cloudTo);
				}
			}
			else
			{
				const Signature * sFrom = memory_->getSignature(link.from());
				const Signature * sTo = memory_->getSignature(link.to());
				if(sFrom && sTo)
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFrom(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTo(new pcl::PointCloud<pcl::PointXYZ>);
					cloudFrom->resize(sFrom->getWords3().size());
					cloudTo->resize(sTo->getWords3().size());
					int i=0;
					for(std::multimap<int, pcl::PointXYZ>::const_iterator iter=sFrom->getWords3().begin();
						iter!=sFrom->getWords3().end();
						++iter)
					{
						cloudFrom->at(i++) = iter->second;
					}
					i=0;
					for(std::multimap<int, pcl::PointXYZ>::const_iterator iter=sTo->getWords3().begin();
						iter!=sTo->getWords3().end();
						++iter)
					{
						cloudTo->at(i++) = iter->second;
					}

					if(cloudFrom->size())
					{
						cloudFrom = rtabmap::util3d::removeNaNFromPointCloud<pcl::PointXYZ>(cloudFrom);
					}
					if(cloudTo->size())
					{
						cloudTo = rtabmap::util3d::removeNaNFromPointCloud<pcl::PointXYZ>(cloudTo);
						cloudTo = rtabmap::util3d::transformPointCloud<pcl::PointXYZ>(cloudTo, t);
					}

					if(cloudFrom->size())
					{
						ui_->constraintsViewer->addOrUpdateCloud("cloud0", cloudFrom);
					}
					else
					{
						UWARN("Empty 3D words for node %d", link.from());
					}
					if(cloudTo->size())
					{
						ui_->constraintsViewer->addOrUpdateCloud("cloud1", cloudTo);
					}
					else
					{
						UWARN("Empty 3D words for node %d", link.to());
					}
				}
				else
				{
					UERROR("Not found signature %d or %d in RAM", link.from(), link.to());
				}
			}

			//cloud 2d
			pcl::PointCloud<pcl::PointXYZ>::Ptr scanA, scanB;
			scanA = rtabmap::util3d::laserScanToPointCloud(dataFrom.getLaserScanRaw());
			scanB = rtabmap::util3d::laserScanToPointCloud(dataTo.getLaserScanRaw());
			scanB = rtabmap::util3d::transformPointCloud<pcl::PointXYZ>(scanB, t);

			if(scanA->size())
			{
				ui_->constraintsViewer->addOrUpdateCloud("scan0", scanA);
			}
			if(scanB->size())
			{
				ui_->constraintsViewer->addOrUpdateCloud("scan1", scanB);
			}
		}
		else
		{
			if(cloudFrom->size())
			{
				ui_->constraintsViewer->addOrUpdateCloud("cloud0", cloudFrom);
			}
			if(cloudTo->size())
			{
				ui_->constraintsViewer->addOrUpdateCloud("cloud1", cloudTo);
			}
		}

		//update cordinate
		ui_->constraintsViewer->updateCameraPosition(t);
		ui_->constraintsViewer->clearTrajectory();

		ui_->constraintsViewer->render();
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
			ui_->pushButton_reject->setEnabled(currentLink.type() != Link::kNeighbor);
		}

		//check for modified link
		bool modified = false;
		std::multimap<int, Link>::iterator iter = util3d::findLink(linksRefined_, currentLink.from(), currentLink.to());
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
	if(memory_ && value >=0 && value < (int)graphes_.size())
	{
		if(ui_->dockWidget_graphView->isVisible() && scans_.size() == 0)
		{
			//update scans
			UINFO("Update scans list...");
			for(int i=0; i<ids_.size(); ++i)
			{
				Signature data = memory_->getSignatureData(ids_.at(i), false);
				if(!data.getLaserScanCompressed().empty())
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
					cv::Mat laserScan = rtabmap::util3d::uncompressData(data.getLaserScanCompressed());
					cloud = rtabmap::util3d::laserScanToPointCloud(laserScan);
					scans_.insert(std::make_pair(ids_.at(i), cloud));
				}
			}
			UINFO("Update scans list... done");
		}
		std::map<int, rtabmap::Transform> & graph = uValueAt(graphes_, value);
		std::multimap<int, Link> links = updateLinksWithModifications(links_);
		ui_->graphViewer->updateGraph(graph, links);
		if(graph.size() && scans_.size())
		{
			float xMin, yMin;
			float cell = 0.05;
			cv::Mat map = rtabmap::util3d::convertMap2Image8U(rtabmap::util3d::create2DMap(graph, scans_, cell, true, xMin, yMin));
			ui_->graphViewer->updateMap(map, cell, xMin, yMin);
		}
		ui_->label_iterations->setNum(value);

		//compute total length (neighbor links)
		float length = 0.0f;
		for(std::multimap<int, rtabmap::Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
		{
			std::map<int, rtabmap::Transform>::const_iterator jterA = graph.find(iter->first);
			std::map<int, rtabmap::Transform>::const_iterator jterB = graph.find(iter->second.to());
			if(jterA != graph.end() && jterB != graph.end())
			{
				const rtabmap::Transform & poseA = jterA->second;
				const rtabmap::Transform & poseB = jterB->second;
				if(iter->second.type() == rtabmap::Link::kNeighbor)
				{
					Eigen::Vector3f vA, vB;
					poseA.getTranslation(vA[0], vA[1], vA[2]);
					poseB.getTranslation(vB[0], vB[1], vB[2]);
					length += (vB - vA).norm();
				}
			}
		}
		ui_->label_pathLength->setNum(length);
	}
}
void DatabaseViewer::updateGraphView()
{
	if(poses_.size())
	{
		if(!uContains(poses_, ui_->spinBox_optimizationsFrom->value()))
		{
			QMessageBox::warning(this, tr(""), tr("Graph optimization from id (%1) for which node is not linked to graph.\n Minimum=%2, Maximum=%3")
						.arg(ui_->spinBox_optimizationsFrom->value())
						.arg(poses_.begin()->first)
						.arg(poses_.rbegin()->first));
			return;
		}

		graphes_.clear();

		std::map<int, rtabmap::Transform> finalPoses;
		graphes_.push_back(poses_);
		ui_->actionGenerate_TORO_graph_graph->setEnabled(true);
		std::multimap<int, rtabmap::Link> links = updateLinksWithModifications(links_);
		std::map<int, int> depthGraph = util3d::generateDepthGraph(links, ui_->spinBox_optimizationsFrom->value(), 0);
		util3d::optimizeTOROGraph(
				depthGraph,
				poses_,
				links, finalPoses,
				ui_->spinBox_iterations->value(),
				ui_->checkBox_initGuess->isChecked(),
				ui_->checkBox_ignoreCovariance->isChecked(),
				&graphes_);
		graphes_.push_back(finalPoses);
	}
	if(graphes_.size())
	{
		ui_->horizontalSlider_iterations->setMaximum(graphes_.size()-1);
		ui_->horizontalSlider_iterations->setValue(graphes_.size()-1);
		ui_->dockWidget_graphView->setEnabled(true);
		sliderIterationsValueChanged(graphes_.size()-1);
	}
	else
	{
		ui_->dockWidget_graphView->setEnabled(false);
	}
}

Link DatabaseViewer::findActiveLink(int from, int to)
{
	Link link;
	std::multimap<int, Link>::iterator findIter = util3d::findLink(linksRefined_, from ,to);
	if(findIter != linksRefined_.end())
	{
		link = findIter->second;
	}
	else
	{
		findIter = util3d::findLink(linksAdded_, from ,to);
		if(findIter != linksAdded_.end())
		{
			link = findIter->second;
		}
		else if(!containsLink(linksRemoved_, from ,to))
		{
			findIter = util3d::findLink(links_, from ,to);
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
	return util3d::findLink(links, from, to) != links.end();
}

void DatabaseViewer::refineConstraint()
{
	int from = ids_.at(ui_->horizontalSlider_A->value());
	int to = ids_.at(ui_->horizontalSlider_B->value());
	refineConstraint(from, to, true);
}

void DatabaseViewer::refineConstraint(int from, int to, bool updateGraph)
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
	   currentLink.type() == Link::kNeighbor &&
	   graphes_.size() &&
	   (int)graphes_.size()-1 == ui_->horizontalSlider_iterations->maximum())
	{
		std::map<int, rtabmap::Transform> & graph = uValueAt(graphes_, ui_->horizontalSlider_iterations->value());
		if(currentLink.type() == Link::kNeighbor)
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


	bool hasConverged = false;
	double variance = -1.0;
	int correspondences = 0;
	Transform transform;

	Signature dataFrom, dataTo;
	dataFrom = memory_->getSignatureData(currentLink.from(), false);
	dataTo = memory_->getSignatureData(currentLink.to(), false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
	if(ui_->checkBox_icp_2d->isChecked())
	{
		//2D
		cv::Mat oldLaserScan = util3d::uncompressData(dataFrom.getLaserScanCompressed());
		cv::Mat newLaserScan = util3d::uncompressData(dataTo.getLaserScanCompressed());

		if(!oldLaserScan.empty() && !newLaserScan.empty())
		{
			// 2D
			pcl::PointCloud<pcl::PointXYZ>::Ptr oldCloud = util3d::cvMat2Cloud(oldLaserScan);
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud = util3d::cvMat2Cloud(newLaserScan, t);

			//voxelize
			if(ui_->doubleSpinBox_icp_voxel->value() > 0.0f)
			{
				oldCloud = util3d::voxelize<pcl::PointXYZ>(oldCloud, ui_->doubleSpinBox_icp_voxel->value());
				newCloud = util3d::voxelize<pcl::PointXYZ>(newCloud, ui_->doubleSpinBox_icp_voxel->value());
			}

			if(newCloud->size() && oldCloud->size())
			{
				transform = util3d::icp2D(newCloud,
						oldCloud,
						ui_->doubleSpinBox_icp_maxCorrespDistance->value(),
						ui_->spinBox_icp_iteration->value(),
					   &hasConverged,
					   &variance,
					   &correspondences);
			}
		}
	}
	else
	{
		//3D
		cv::Mat depthA = rtabmap::util3d::uncompressImage(dataFrom.getDepthCompressed());
		cv::Mat depthB = rtabmap::util3d::uncompressImage(dataTo.getDepthCompressed());

		if(depthA.type() == CV_8UC1)
		{
			cv::Mat leftMono;
			cv::Mat left = rtabmap::util3d::uncompressImage(dataFrom.getImageCompressed());
			if(left.channels() > 1)
			{
				cv::cvtColor(left, leftMono, CV_BGR2GRAY);
			}
			else
			{
				leftMono = left;
			}
			cloudA = util3d::cloudFromDisparity(util3d::disparityFromStereoImages(leftMono, depthA), dataFrom.getDepthCx(), dataFrom.getDepthCy(), dataFrom.getDepthFx(), dataFrom.getDepthFy(), ui_->spinBox_icp_decimation->value());
			if(ui_->doubleSpinBox_icp_maxDepth->value() > 0)
			{
				cloudA = util3d::passThrough<pcl::PointXYZ>(cloudA, "z", 0, ui_->doubleSpinBox_icp_maxDepth->value());
			}
			if(ui_->doubleSpinBox_icp_voxel->value() > 0)
			{
				cloudA = util3d::voxelize<pcl::PointXYZ>(cloudA, ui_->doubleSpinBox_icp_voxel->value());
			}
			cloudA = util3d::transformPointCloud<pcl::PointXYZ>(cloudA, dataFrom.getLocalTransform());
		}
		else
		{
			cloudA = util3d::getICPReadyCloud(depthA,
					dataFrom.getDepthFx(), dataFrom.getDepthFy(), dataFrom.getDepthCx(), dataFrom.getDepthCy(),
					ui_->spinBox_icp_decimation->value(),
					ui_->doubleSpinBox_icp_maxDepth->value(),
					ui_->doubleSpinBox_icp_voxel->value(),
					0, // no sampling
					dataFrom.getLocalTransform());
		}
		if(depthB.type() == CV_8UC1)
		{
			cv::Mat leftMono;
			cv::Mat left = rtabmap::util3d::uncompressImage(dataTo.getImageCompressed());
			if(left.channels() > 1)
			{
				cv::cvtColor(left, leftMono, CV_BGR2GRAY);
			}
			else
			{
				leftMono = left;
			}
			cloudB = util3d::cloudFromDisparity(util3d::disparityFromStereoImages(leftMono, depthB), dataTo.getDepthCx(), dataTo.getDepthCy(), dataTo.getDepthFx(), dataTo.getDepthFy(), ui_->spinBox_icp_decimation->value());
			if(ui_->doubleSpinBox_icp_maxDepth->value() > 0)
			{
				cloudB = util3d::passThrough<pcl::PointXYZ>(cloudB, "z", 0, ui_->doubleSpinBox_icp_maxDepth->value());
			}
			if(ui_->doubleSpinBox_icp_voxel->value() > 0)
			{
				cloudB = util3d::voxelize<pcl::PointXYZ>(cloudB, ui_->doubleSpinBox_icp_voxel->value());
			}
			cloudB = util3d::transformPointCloud<pcl::PointXYZ>(cloudB, t * dataTo.getLocalTransform());
		}
		else
		{
			cloudB = util3d::getICPReadyCloud(depthB,
					dataTo.getDepthFx(), dataTo.getDepthFy(), dataTo.getDepthCx(), dataTo.getDepthCy(),
					ui_->spinBox_icp_decimation->value(),
					ui_->doubleSpinBox_icp_maxDepth->value(),
					ui_->doubleSpinBox_icp_voxel->value(),
					0, // no sampling
					t * dataTo.getLocalTransform());
		}

		if(ui_->checkBox_icp_p2plane->isChecked())
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloudANormals = util3d::computeNormals(cloudA, ui_->spinBox_icp_normalKSearch->value());
			pcl::PointCloud<pcl::PointNormal>::Ptr cloudBNormals = util3d::computeNormals(cloudB, ui_->spinBox_icp_normalKSearch->value());

			cloudANormals = util3d::removeNaNNormalsFromPointCloud<pcl::PointNormal>(cloudANormals);
			if(cloudA->size() != cloudANormals->size())
			{
				UWARN("removed nan normals...");
			}

			cloudBNormals = util3d::removeNaNNormalsFromPointCloud<pcl::PointNormal>(cloudBNormals);
			if(cloudB->size() != cloudBNormals->size())
			{
				UWARN("removed nan normals...");
			}

			transform = util3d::icpPointToPlane(cloudBNormals,
					cloudANormals,
					ui_->doubleSpinBox_icp_maxCorrespDistance->value(),
					ui_->spinBox_icp_iteration->value(),
					&hasConverged,
					&variance,
					&correspondences);
		}
		else
		{
			transform = util3d::icp(cloudB,
					cloudA,
					ui_->doubleSpinBox_icp_maxCorrespDistance->value(),
					ui_->spinBox_icp_iteration->value(),
					&hasConverged,
					&variance,
					&correspondences);
		}
	}

	if(hasConverged && !transform.isNull())
	{
		Link newLink(currentLink.from(), currentLink.to(), currentLink.type(), transform*t, variance);

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
			linksRefined_.insert(std::make_pair<int, Link>(newLink.from(), newLink));

			if(updateGraph)
			{
				this->updateGraphView();
			}
		}

		if(ui_->dockWidget_constraints->isVisible())
		{
			cloudB = util3d::transformPointCloud<pcl::PointXYZ>(cloudB, transform);
			this->updateConstraintView(newLink, cloudA, cloudB);
		}
	}
}

void DatabaseViewer::refineConstraintVisually()
{
	int from = ids_.at(ui_->horizontalSlider_A->value());
	int to = ids_.at(ui_->horizontalSlider_B->value());
	refineConstraintVisually(from, to, true);
}

void DatabaseViewer::refineConstraintVisually(int from, int to, bool updateGraph)
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

	Transform t;
	std::string rejectedMsg;
	double variance = -1.0;
	int inliers = -1;
	if(ui_->checkBox_visual_recomputeFeatures->isChecked())
	{
		// create a fake memory to regenerate features
		ParametersMap parameters;
		parameters.insert(ParametersPair(Parameters::kSURFHessianThreshold(), uNumber2Str(ui_->doubleSpinBox_visual_hessian->value())));
		parameters.insert(ParametersPair(Parameters::kLccBowInlierDistance(), uNumber2Str(ui_->doubleSpinBox_visual_maxCorrespDistance->value())));
		parameters.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(ui_->doubleSpinBox_visual_maxDepth->value())));
		parameters.insert(ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(ui_->doubleSpinBox_visual_nndr->value())));
		parameters.insert(ParametersPair(Parameters::kLccBowIterations(), uNumber2Str(ui_->spinBox_visual_iteration->value())));
		parameters.insert(ParametersPair(Parameters::kLccBowMinInliers(), uNumber2Str(ui_->spinBox_visual_minCorrespondences->value())));
		parameters.insert(ParametersPair(Parameters::kMemGenerateIds(), "false"));
		parameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0"));
		parameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), "0"));
		Memory tmpMemory(parameters);

		// Add signatures
		SensorData dataFrom = memory_->getSignatureData(from, true).toSensorData();
		SensorData dataTo = memory_->getSignatureData(to, true).toSensorData();

		if(from > to)
		{
			tmpMemory.update(dataTo);
			tmpMemory.update(dataFrom);
		}
		else
		{
			tmpMemory.update(dataFrom);
			tmpMemory.update(dataTo);
		}


		t = tmpMemory.computeVisualTransform(to, from, &rejectedMsg, &inliers, &variance);
	}
	else
	{
		ParametersMap parameters;
		parameters.insert(ParametersPair(Parameters::kLccBowInlierDistance(), uNumber2Str(ui_->doubleSpinBox_visual_maxCorrespDistance->value())));
		parameters.insert(ParametersPair(Parameters::kLccBowMaxDepth(), uNumber2Str(ui_->doubleSpinBox_visual_maxDepth->value())));
		parameters.insert(ParametersPair(Parameters::kLccBowIterations(), uNumber2Str(ui_->spinBox_visual_iteration->value())));
		parameters.insert(ParametersPair(Parameters::kLccBowMinInliers(), uNumber2Str(ui_->spinBox_visual_minCorrespondences->value())));
		memory_->parseParameters(parameters);
		t = memory_->computeVisualTransform(to, from, &rejectedMsg, &inliers, &variance);
	}

	if(!t.isNull())
	{
		Link newLink(currentLink.from(), currentLink.to(), currentLink.type(), t, variance);

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
			linksRefined_.insert(std::make_pair<int, Link>(newLink.from(), newLink));

			if(updateGraph)
			{
				this->updateGraphView();
			}
		}
		if(ui_->dockWidget_constraints->isVisible())
		{
			this->updateConstraintView(newLink);
		}
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
	if(from < to)
	{
		int tmp = to;
		to = from;
		from = tmp;
	}

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

		Transform t;
		std::string rejectedMsg;
		double variance = -1.0;
		int inliers = -1;
		if(ui_->checkBox_visual_recomputeFeatures->isChecked())
		{
			// create a fake memory to regenerate features
			ParametersMap parameters;
			parameters.insert(ParametersPair(Parameters::kSURFHessianThreshold(), uNumber2Str(ui_->doubleSpinBox_visual_hessian->value())));
			parameters.insert(ParametersPair(Parameters::kLccBowInlierDistance(), uNumber2Str(ui_->doubleSpinBox_visual_maxCorrespDistance->value())));
			parameters.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(ui_->doubleSpinBox_visual_maxDepth->value())));
			parameters.insert(ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(ui_->doubleSpinBox_visual_nndr->value())));
			parameters.insert(ParametersPair(Parameters::kLccBowIterations(), uNumber2Str(ui_->spinBox_visual_iteration->value())));
			parameters.insert(ParametersPair(Parameters::kLccBowMinInliers(), uNumber2Str(ui_->spinBox_visual_minCorrespondences->value())));
			parameters.insert(ParametersPair(Parameters::kMemGenerateIds(), "false"));
			parameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0"));
			parameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), "0"));
			Memory tmpMemory(parameters);

			// Add signatures
			SensorData dataFrom = memory_->getSignatureData(from, true).toSensorData();
			SensorData dataTo = memory_->getSignatureData(to, true).toSensorData();

			if(from > to)
			{
				tmpMemory.update(dataTo);
				tmpMemory.update(dataFrom);
			}
			else
			{
				tmpMemory.update(dataFrom);
				tmpMemory.update(dataTo);
			}


			t = tmpMemory.computeVisualTransform(to, from, &rejectedMsg, &inliers, &variance);
		}
		else
		{
			ParametersMap parameters;
			parameters.insert(ParametersPair(Parameters::kLccBowInlierDistance(), uNumber2Str(ui_->doubleSpinBox_visual_maxCorrespDistance->value())));
			parameters.insert(ParametersPair(Parameters::kLccBowMaxDepth(), uNumber2Str(ui_->doubleSpinBox_visual_maxDepth->value())));
			parameters.insert(ParametersPair(Parameters::kLccBowIterations(), uNumber2Str(ui_->spinBox_visual_iteration->value())));
			parameters.insert(ParametersPair(Parameters::kLccBowMinInliers(), uNumber2Str(ui_->spinBox_visual_minCorrespondences->value())));
			memory_->parseParameters(parameters);
			t = memory_->computeVisualTransform(to, from, &rejectedMsg, &inliers, &variance);
		}

		if(t.isNull())
		{
			if(!silent)
			{
				QMessageBox::warning(this,
						tr("Add link"),
						tr("Cannot find a transformation between nodes %1 and %2: %3").arg(from).arg(to).arg(rejectedMsg.c_str()));
			}
		}
		else
		{
			if(ui_->checkBox_visual_2d->isChecked())
			{
				// We are 2D here, make sure the guess has only YAW rotation
				float x,y,z,r,p,yaw;
				t.getTranslationAndEulerAngles(x,y,z, r,p,yaw);
				t = util3d::transformFromEigen3f(pcl::getTransformation(x,y,0, 0, 0, yaw));
			}

			// transform is valid, make a link
			linksAdded_.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, t, variance)));
			updateSlider = true;
		}
	}
	else if(containsLink(linksRemoved_, from, to))
	{
		//simply remove from linksRemoved
		linksRemoved_.erase(util3d::findLink(linksRemoved_, from, to));
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


	std::multimap<int, Link>::iterator iter = util3d::findLink(linksRefined_, from, to);
	if(iter != linksRefined_.end())
	{
		linksRefined_.erase(iter);
		this->updateGraphView();
	}

	iter = util3d::findLink(links_, from, to);
	if(iter != links_.end())
	{
		this->updateConstraintView(iter->second);
	}
	iter = util3d::findLink(linksAdded_, from, to);
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
	iter = util3d::findLink(links_, from, to);
	if(iter != links_.end())
	{
		if(iter->second.type() == Link::kNeighbor)
		{
			UWARN("Cannot reject neighbor links (%d->%d)", from, to);
			return;
		}
		linksRemoved_.insert(*iter);
		removed = true;
	}

	// remove from refined and added
	iter = util3d::findLink(linksRefined_, from, to);
	if(iter != linksRefined_.end())
	{
		linksRefined_.erase(iter);
		removed = true;
	}
	iter = util3d::findLink(linksAdded_, from, to);
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

		findIter = util3d::findLink(linksRemoved_, iter->second.from(), iter->second.to());
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

		findIter = util3d::findLink(linksRefined_, iter->second.from(), iter->second.to());
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
			if(iter->second.type() != rtabmap::Link::kNeighbor)
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
		ui_->constraintsViewer->removeAllClouds();
		ui_->constraintsViewer->render();
		updateConstraintButtons();
	}
}

} // namespace rtabmap
