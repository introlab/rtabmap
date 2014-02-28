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

#include "rtabmap/gui/DatabaseViewer.h"
#include "rtabmap/gui/CloudViewer.h"
#include "ui_DatabaseViewer.h"
#include <QtGui/QMessageBox>
#include <QtGui/QFileDialog>
#include <QtGui/QInputDialog>
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
#include "rtabmap/gui/DataRecorder.h"
#include "rtabmap/core/Image.h"
#include "ExportDialog.h"
#include "DetailedProgressDialog.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

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
	ui_->menuView->addAction(ui_->dockWidget_constraints->toggleViewAction());
	ui_->dockWidget_graphView->setVisible(false);
	ui_->menuView->addAction(ui_->dockWidget_graphView->toggleViewAction());
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

	ui_->horizontalSlider_iterations->setTracking(false);
	ui_->dockWidget_graphView->setEnabled(false);
	connect(ui_->horizontalSlider_iterations, SIGNAL(valueChanged(int)), this, SLOT(sliderIterationsValueChanged(int)));
	connect(ui_->horizontalSlider_iterations, SIGNAL(sliderMoved(int)), this, SLOT(sliderIterationsValueChanged(int)));
	connect(ui_->spinBox_iterations, SIGNAL(editingFinished()), this, SLOT(updateGraphView()));
	connect(ui_->spinBox_optimizationsFrom, SIGNAL(editingFinished()), this, SLOT(updateGraphView()));
	connect(ui_->checkBox_initGuess, SIGNAL(stateChanged(int)), this, SLOT(updateGraphView()));

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
			scans_.clear();
			ui_->actionGenerate_TORO_graph_graph->setEnabled(false);
		}

		std::string driverType = "sqlite3";
		rtabmap::ParametersMap parameters;
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kDbSqlite3InMemory(), "false"));
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "false"));

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
					std::vector<unsigned char> compressedRgb, compressedDepth, compressedDepth2d;
					float tmpDepthConstant;
					rtabmap::Transform tmpLocalTransform, pose;

					memory_->getImageDepth(id, compressedRgb, compressedDepth, compressedDepth2d, tmpDepthConstant, tmpLocalTransform);
					if(dialog.isOdomExported())
					{
						memory_->getPose(id, pose, true);
					}

					cv::Mat rgb, depth, depth2d;
					float depthConstant = 0;
					rtabmap::Transform localTransform;

					if(dialog.isRgbExported())
					{
						rgb = rtabmap::util3d::uncompressImage(compressedRgb);
					}
					if(dialog.isDepthExported())
					{
						depth = rtabmap::util3d::uncompressImage(compressedDepth);
						depthConstant = tmpDepthConstant;
						localTransform = tmpLocalTransform;
					}
					if(dialog.isDepth2dExported())
					{
						depth2d = rtabmap::util3d::uncompressData(compressedDepth2d);
					}

					rtabmap::Image data(rgb, depth, depth2d, depthConstant, pose, localTransform, id);
					recorder.addData(data);

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
			std::vector<unsigned char> compressedRgb = memory_->getImage(id);
			if(compressedRgb.size())
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
	if(memory_->getLastWorkingSignature())
	{
		std::map<int, int> nids = memory_->getNeighborsId(memory_->getLastWorkingSignature()->id(), 0, -1, true);
		memory_->getMetricConstraints(uKeys(nids), poses_, links_, true);

		ui_->spinBox_optimizationsFrom->setRange(1, memory_->getLastWorkingSignature()->id());
		ui_->spinBox_optimizationsFrom->setValue(memory_->getLastWorkingSignature()->id());
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
	if(loopLinks_.size())
	{
		ui_->horizontalSlider_loops->setMinimum(0);
		ui_->horizontalSlider_loops->setMaximum(loopLinks_.size()-1);
		ui_->horizontalSlider_loops->setEnabled(true);
		ui_->horizontalSlider_loops->setSliderPosition(0);
	}
	else
	{
		ui_->horizontalSlider_loops->setEnabled(false);
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
	if(!graphes_.size() || !links_.size())
	{
		QMessageBox::warning(this, tr("Cannot generate a TORO graph"), tr("No poses or no links..."));
		return;
	}
	bool ok = false;
	int id = QInputDialog::getInt(this, tr("Which iteration?"), tr("Iteration (0 -> %1)").arg(graphes_.size()-1), graphes_.size()-1, 0, graphes_.size()-1, 1, &ok);

	if(ok)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save File"), pathDatabase_+"/constraints" + QString::number(id) + ".graph", tr("TORO file (*.graph)"));
		if(!path.isEmpty())
		{
			rtabmap::util3d::saveTOROGraph(path.toStdString(), uValueAt(graphes_, id), links_);
		}
	}
}

void DatabaseViewer::view3DMap()
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
		int margin = QInputDialog::getInt(this, tr("Depth around the location?"), tr("Margin (0=no limit)"), 0, 0, 100, 1, &ok);
		if(ok)
		{
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
					std::map<int, int> ids = memory_->getNeighborsId(id, margin, -1, false);
					if(ids.size() > 0)
					{
						rtabmap::DetailedProgressDialog progressDialog(this);
						progressDialog.setMaximumSteps(ids.size()+2);
						progressDialog.show();

						progressDialog.appendText("Graph generation...");
						std::map<int, rtabmap::Transform> poses, optimizedPoses;
						std::multimap<int, rtabmap::Link> edgeConstraints;
						memory_->getMetricConstraints(uKeys(ids), poses, edgeConstraints, true);
						progressDialog.appendText("Graph generation... done!");
						progressDialog.incrementStep();

						progressDialog.appendText("Graph optimization...");
						rtabmap::Transform mapCorrection;
						rtabmap::util3d::optimizeTOROGraph(poses, edgeConstraints, optimizedPoses, mapCorrection, 100, true);
						progressDialog.appendText("Graph optimization... done!");
						progressDialog.incrementStep();

						// create a window
						QWidget * window = new QWidget(this, Qt::Popup);
						window->setAttribute(Qt::WA_DeleteOnClose);
						window->setWindowFlags(Qt::Dialog);
						window->setWindowTitle(tr("3D Map"));
						window->setMinimumWidth(800);
						window->setMinimumHeight(600);

						rtabmap::CloudViewer * viewer = new rtabmap::CloudViewer(window);

						QVBoxLayout *layout = new QVBoxLayout();
						layout->addWidget(viewer);
						window->setLayout(layout);

						window->showNormal();

						for(std::map<int, int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
						{
							rtabmap::Transform pose = uValue(optimizedPoses, iter->first, rtabmap::Transform());
							if(!pose.isNull())
							{
								std::vector<unsigned char> image, depth, depth2d;
								float depthConstant;
								rtabmap::Transform localTransform;
								memory_->getImageDepth(iter->first, image, depth, depth2d, depthConstant, localTransform);
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
								cv::Mat imageMat = rtabmap::util3d::uncompressImage(image);
								cv::Mat depthMat = rtabmap::util3d::uncompressImage(depth);
								cloud = rtabmap::util3d::cloudFromDepthRGB(
										imageMat,
										depthMat,
										depthMat.cols/2, depthMat.rows/2,
										1.0f/depthConstant, 1.0f/depthConstant,
										decimation);

								if(maxDepth)
								{
									cloud = rtabmap::util3d::passThrough(cloud, "z", 0, maxDepth);
								}

								cloud = rtabmap::util3d::transformPointCloud(cloud, localTransform);

								viewer->addCloud(uFormat("cloud%d", iter->first), cloud, pose);

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
						QMessageBox::critical(this, tr("Error"), tr("No neighbors found for node %1.").arg(id));
					}
				}
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
	int id = QInputDialog::getInt(this, tr("Around which location?"), tr("Location ID"), ids_.first(), ids_.first(), ids_.last(), 1, &ok);

	if(ok)
	{
		int margin = QInputDialog::getInt(this, tr("Depth around the location?"), tr("Margin (0=no limit)"), 0, 0, 100, 1, &ok);
		if(ok)
		{
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
						std::map<int, int> ids = memory_->getNeighborsId(id, margin, -1, false);
						if(ids.size() > 0)
						{
							rtabmap::DetailedProgressDialog progressDialog;
							progressDialog.setMaximumSteps(ids.size()+2);
							progressDialog.show();

							progressDialog.appendText("Graph generation...");
							std::map<int, rtabmap::Transform> poses, optimizedPoses;
							std::multimap<int, rtabmap::Link> edgeConstraints;
							memory_->getMetricConstraints(uKeys(ids), poses, edgeConstraints, true);
							progressDialog.appendText("Graph generation... done!");
							progressDialog.incrementStep();

							progressDialog.appendText("Graph optimization...");
							rtabmap::Transform mapCorrection;
							rtabmap::util3d::optimizeTOROGraph(poses, edgeConstraints, optimizedPoses, mapCorrection, 100, true);
							progressDialog.appendText("Graph optimization... done!");
							progressDialog.incrementStep();

							for(std::map<int, int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
							{
								rtabmap::Transform pose = uValue(optimizedPoses, iter->first, rtabmap::Transform());
								if(!pose.isNull())
								{
									std::vector<unsigned char> image, depth, depth2d;
									float depthConstant;
									rtabmap::Transform localTransform;
									memory_->getImageDepth(iter->first, image, depth, depth2d, depthConstant, localTransform);
									pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
									cv::Mat imageMat = rtabmap::util3d::uncompressImage(image);
									cv::Mat depthMat = rtabmap::util3d::uncompressImage(depth);
									cloud = rtabmap::util3d::cloudFromDepthRGB(
											imageMat,
											depthMat,
											depthMat.cols/2, depthMat.rows/2,
											1.0f/depthConstant, 1.0f/depthConstant,
											decimation);

									if(maxDepth)
									{
										cloud = rtabmap::util3d::passThrough(cloud, "z", 0, maxDepth);
									}

									cloud = rtabmap::util3d::transformPointCloud(cloud, pose*localTransform);
									std::string name = uFormat("%s/node%d.pcd", path.toStdString().c_str(), iter->first);
									pcl::io::savePCDFile(name, *cloud);
									UINFO("Saved %s (%d points)", name.c_str(), cloud->size());
									progressDialog.appendText(QString("Saved %1 (%2 points)").arg(name.c_str()).arg(cloud->size()));
									progressDialog.incrementStep();
									QApplication::processEvents();
								}
							}
							progressDialog.setValue(progressDialog.maximumSteps());

							QMessageBox::information(this, tr("Finished"), tr("%1 clouds generated to %2.").arg(ids.size()).arg(path));
						}
						else
						{
							QMessageBox::critical(this, tr("Error"), tr("No neighbors found for node %1.").arg(id));
						}
					}
				}
			}
		}
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
						QLabel * labelId)
{
	UTimer timer;
	labelIndex->setText(QString::number(value));
	labelParents->clear();
	labelChildren->clear();
	if(value >= 0 && value < ids_.size())
	{
		view->clear();
		int id = ids_.at(value);
		labelId->setText(QString::number(id));
		if(id>0)
		{
			//image
			QImage img;
			QImage imgDepth;
			if(memory_)
			{
				std::vector<unsigned char> image, depth, depth2d;
				float depthConstant;
				rtabmap::Transform localTransform;
				memory_->getImageDepth(id, image, depth, depth2d, depthConstant, localTransform);
				cv::Mat imageMat = rtabmap::util3d::uncompressImage(image);
				cv::Mat depthMat = rtabmap::util3d::uncompressImage(depth);
				if(!image.empty())
				{
					img = uCvMat2QImage(imageMat);
				}
				if(!depth.empty())
				{
					imgDepth = uCvMat2QImage(depthMat);
				}
			}

			if(memory_)
			{
				std::multimap<int, cv::KeyPoint> words = memory_->getWords(id);
				if(words.size())
				{
					view->setFeatures(words);
				}
			}

			if(!img.isNull())
			{
				view->setImage(img);
			}
			else
			{
				ULOGGER_DEBUG("Image is empty");
			}
			if(!imgDepth.isNull())
			{
				view->setImageDepth(imgDepth);
			}
			else
			{
				ULOGGER_DEBUG("Image depth is empty");
			}
			view->fitInView(view->sceneRect(), Qt::KeepAspectRatio);

			// loops
			std::map<int, rtabmap::Transform> parents;
			std::map<int, rtabmap::Transform> children;
			memory_->getLoopClosureIds(id, parents, children, true);
			if(parents.size())
			{
				QString str;
				for(std::map<int, rtabmap::Transform>::iterator iter=parents.begin(); iter!=parents.end(); ++iter)
				{
					str.append(QString("%1 ").arg(iter->first));
				}
				labelParents->setText(str);
			}
			if(children.size())
			{
				QString str;
				for(std::map<int, rtabmap::Transform>::iterator iter=children.begin(); iter!=children.end(); ++iter)
				{
					str.append(QString("%1 ").arg(iter->first));
				}
				labelChildren->setText(str);
			}
		}

		labelId->setText(QString::number(id));
		view->fitInView(view->scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
	}
	else
	{
		ULOGGER_ERROR("Slider index out of range ?");
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

void DatabaseViewer::updateConstraintView(const rtabmap::Link & link)
{
	rtabmap::Transform t = link.transform();
	ui_->label_constraint->clear();
	if(!t.isNull() && memory_)
	{
		ui_->label_constraint->setText(t.prettyPrint().c_str());
		ui_->horizontalSlider_A->setValue(idToIndex_.value(link.from()));
		ui_->horizontalSlider_B->setValue(idToIndex_.value(link.to()));

		float depthConstantA, depthConstantB;
		rtabmap::Transform localTransformA, localTransformB;

		std::vector<unsigned char> imageBytesA, depthBytesA, depth2dBytesA;
		memory_->getImageDepth(link.from(), imageBytesA, depthBytesA, depth2dBytesA, depthConstantA, localTransformA);
		cv::Mat imageA = rtabmap::util3d::uncompressImage(imageBytesA);
		cv::Mat depthA = rtabmap::util3d::uncompressImage(depthBytesA);
		cv::Mat depth2dA = rtabmap::util3d::uncompressData(depth2dBytesA);

		std::vector<unsigned char> imageBytesB, depthBytesB, depth2dBytesB;
		memory_->getImageDepth(link.to(), imageBytesB, depthBytesB, depth2dBytesB, depthConstantB, localTransformB);
		cv::Mat imageB = rtabmap::util3d::uncompressImage(imageBytesB);
		cv::Mat depthB = rtabmap::util3d::uncompressImage(depthBytesB);
		cv::Mat depth2dB = rtabmap::util3d::uncompressData(depth2dBytesB);

		//cloud 3d
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA;
		cloudA = rtabmap::util3d::cloudFromDepthRGB(
				imageA,
				depthA,
				depthA.cols/2,
				depthA.rows/2,
				1.0f/depthConstantA,
				1.0f/depthConstantA,
				1);

		cloudA = rtabmap::util3d::removeNaNFromPointCloud(cloudA);
		cloudA = rtabmap::util3d::transformPointCloud(cloudA, localTransformA);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB;
		cloudB = rtabmap::util3d::cloudFromDepthRGB(
				imageB,
				depthB,
				depthB.cols/2,
				depthB.rows/2,
				1.0f/depthConstantB,
				1.0f/depthConstantB,
				1);

		cloudB = rtabmap::util3d::removeNaNFromPointCloud(cloudB);
		cloudB = rtabmap::util3d::transformPointCloud(cloudB, t*localTransformB);

		//cloud 2d
		pcl::PointCloud<pcl::PointXYZ>::Ptr scanA, scanB;
		scanA = rtabmap::util3d::depth2DToPointCloud(depth2dA);
		scanB = rtabmap::util3d::depth2DToPointCloud(depth2dB);
		scanB = rtabmap::util3d::transformPointCloud(scanB, t);

		if(cloudA->size())
		{
			ui_->constraintsViewer->addOrUpdateCloud("cloud0", cloudA);
		}
		if(cloudB->size())
		{
			ui_->constraintsViewer->addOrUpdateCloud("cloud1", cloudB);
		}
		if(scanA->size())
		{
			ui_->constraintsViewer->addOrUpdateCloud("scan0", scanA);
		}
		if(scanB->size())
		{
			ui_->constraintsViewer->addOrUpdateCloud("scan1", scanB);
		}
	}
	ui_->constraintsViewer->render();
}

void DatabaseViewer::sliderIterationsValueChanged(int value)
{
	if(memory_ && value >=0 && value < (int)graphes_.size())
	{
		if(scans_.size() == 0)
		{
			//update scans
			UINFO("Update scans list...");
			for(int i=0; i<ids_.size(); ++i)
			{
				std::vector<unsigned char> imageBytesA, depthBytesA, depth2dBytesA;
				float depthConstantA;
				rtabmap::Transform localTransformA;
				memory_->getImageDepth(ids_.at(i), imageBytesA, depthBytesA, depth2dBytesA, depthConstantA, localTransformA);
				if(depth2dBytesA.size())
				{
					scans_.insert(std::make_pair(ids_.at(i), depth2dBytesA));
				}
			}
			UINFO("Update scans list... done");
		}

		ui_->graphViewer->updateGraph(uValueAt(graphes_, value), links_, scans_);
		ui_->label_iterations->setNum(value);
	}
}
void DatabaseViewer::updateGraphView()
{
	if(ui_->dockWidget_graphView->isVisible() && poses_.size())
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

		std::map<int, int> ids = memory_->getNeighborsId(ui_->spinBox_optimizationsFrom->value(), 0, -1, true);
		// Modify IDs using the margin from the current signature (TORO root will be the last signature)
		int m = 0;
		int toroId = 1;
		std::map<int, int> rtabmapToToro; // <RTAB-Map ID, TORO ID>
		std::map<int, int> toroToRtabmap; // <TORO ID, RTAB-Map ID>
		while(ids.size())
		{
			for(std::map<int, int>::iterator iter = ids.begin(); iter!=ids.end();)
			{
				if(m == iter->second)
				{
					rtabmapToToro.insert(std::make_pair(iter->first, toroId));
					toroToRtabmap.insert(std::make_pair(toroId, iter->first));
					++toroId;
					ids.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
			++m;
		}

		std::map<int, rtabmap::Transform> posesToro;
		std::multimap<int, rtabmap::Link> edgeConstraintsToro;
		for(std::map<int, rtabmap::Transform>::iterator iter = poses_.begin(); iter!=poses_.end(); ++iter)
		{
			posesToro.insert(std::make_pair(rtabmapToToro.at(iter->first), iter->second));
		}
		for(std::multimap<int, rtabmap::Link>::iterator iter = links_.begin();
			iter!=links_.end();
			++iter)
		{
			edgeConstraintsToro.insert(std::make_pair(rtabmapToToro.at(iter->first), rtabmap::Link(rtabmapToToro.at(iter->first), rtabmapToToro.at(iter->second.to()), iter->second.transform(), iter->second.type())));
		}

		std::map<int, rtabmap::Transform> optimizedPosesToro;
		rtabmap::Transform mapCorrectionToro;
		std::list<std::map<int, rtabmap::Transform> > graphesToro;

		// Optimize!
		rtabmap::util3d::optimizeTOROGraph(posesToro, edgeConstraintsToro, optimizedPosesToro, mapCorrectionToro, ui_->spinBox_iterations->value(), ui_->checkBox_initGuess->isChecked(), &graphesToro);

		for(std::list<std::map<int, rtabmap::Transform> >::iterator iter = graphesToro.begin(); iter!=graphesToro.end(); ++iter)
		{
			std::map<int, rtabmap::Transform> tmp;
			for(std::map<int, rtabmap::Transform>::iterator jter=iter->begin(); jter!=iter->end(); ++jter)
			{
				tmp.insert(std::make_pair(toroToRtabmap.at(jter->first), jter->second));
			}
			graphes_.push_back(tmp);
		}

		for(std::map<int, rtabmap::Transform>::iterator iter=optimizedPosesToro.begin(); iter!=optimizedPosesToro.end(); ++iter)
		{
			finalPoses.insert(std::make_pair(toroToRtabmap.at(iter->first), iter->second));
		}


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
