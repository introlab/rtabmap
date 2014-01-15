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
#include <rtabmap/utilite/UTimer.h>
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/DBDriver.h"
#include "rtabmap/gui/KeypointItem.h"
#include "rtabmap/gui/UCv2Qt.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/Signature.h"

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

	connect(ui_->actionQuit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui_->buttonBox, SIGNAL(rejected()), this, SLOT(close()));

	// connect actions with custom slots
	connect(ui_->actionOpen_database, SIGNAL(triggered()), this, SLOT(openDatabase()));
	connect(ui_->actionGenerate_graph_dot, SIGNAL(triggered()), this, SLOT(generateGraph()));
	connect(ui_->actionGenerate_local_graph_dot, SIGNAL(triggered()), this, SLOT(generateLocalGraph()));
	connect(ui_->actionGenerate_3D_map_pcd, SIGNAL(triggered()), this, SLOT(generate3DMap()));

	ui_->horizontalSlider_A->setTracking(false);
	ui_->horizontalSlider_B->setTracking(false);
	ui_->horizontalSlider_A->setEnabled(false);
	ui_->horizontalSlider_B->setEnabled(false);
	connect(ui_->horizontalSlider_A, SIGNAL(valueChanged(int)), this, SLOT(sliderAValueChanged(int)));
	connect(ui_->horizontalSlider_B, SIGNAL(valueChanged(int)), this, SLOT(sliderBValueChanged(int)));
	connect(ui_->horizontalSlider_A, SIGNAL(sliderMoved(int)), this, SLOT(sliderAMoved(int)));
	connect(ui_->horizontalSlider_B, SIGNAL(sliderMoved(int)), this, SLOT(sliderBMoved(int)));
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
		QStringList types;
		types << "Keypoint" << "Sensorimotor";

		if(memory_)
		{
			delete memory_;
			memory_ = 0;
			ids_.clear();
		}

		std::string driverType = "sqlite3";
		rtabmap::ParametersMap parameters;
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kDbSqlite3InMemory(), "false"));

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

void DatabaseViewer::updateIds()
{
	if(!memory_)
	{
		return;
	}

	std::set<int> ids = memory_->getAllSignatureIds();
	ids_ = QList<int>::fromStdList(std::list<int>(ids.begin(), ids.end()));

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
		int margin = QInputDialog::getInt(this, tr("Depth around the location?"), tr("Margin"), 4, 1, 100, 1, &ok);
		if(ok)
		{
			float voxelSize = QInputDialog::getDouble(this, tr("Voxel size?"), tr("Voxel Size"), 0.01, 0, 0.1, 3, &ok);
			if(ok)
			{
				QString path = QFileDialog::getSaveFileName(this, tr("Save File"), pathDatabase_+"/Map" + QString::number(id) + ".pcd", tr("PCL file (*.pcd)"));
				if(!path.isEmpty())
				{
					std::map<int, int> ids = memory_->getNeighborsId(id, margin, -1, false);
					if(ids.size() > 0)
					{
						std::map<int, rtabmap::Transform> poses, optimizedPoses;
						std::multimap<int, std::pair<int, rtabmap::Transform> > edgeConstraints;
						memory_->getMetricConstraints(uKeys(ids), memory_->getSignature(id)->mapId(), poses, edgeConstraints, true);

						UINFO("Poses=%d, constraints=%d", poses.size(), edgeConstraints.size());

						rtabmap::util3d::saveTOROGraph("toro1.graph", poses, edgeConstraints);

						rtabmap::Transform mapCorrection;
						rtabmap::util3d::optimizeTOROGraph(poses, edgeConstraints, 100, optimizedPoses, mapCorrection);

						rtabmap::util3d::saveTOROGraph("toro2.graph", optimizedPoses, edgeConstraints);

						pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
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
										1.0f/depthConstant, 1.0f/depthConstant);

								if(voxelSize > 0.0f)
								{
									cloud = rtabmap::util3d::voxelize(cloud, voxelSize);
								}

								cloud = rtabmap::util3d::transformPointCloud(cloud, pose);

								*assembledCloud += *cloud;
							}
						}

						if(voxelSize > 0.0f)
						{
							pcl::VoxelGrid<pcl::PointXYZRGB> filter;
							filter.setLeafSize(voxelSize, voxelSize, voxelSize);
							filter.setInputCloud(assembledCloud);
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
							filter.filter(*tmp);
							assembledCloud = tmp;
						}

						pcl::io::savePCDFile(path.toStdString(), *assembledCloud, true);
						QMessageBox::information(this, "Generated Map", tr("Map saved to %1!\n(%2 nodes, %3 points)").arg(path).arg(ids.size()).arg(assembledCloud->size()));
					}
					else
					{
						QMessageBox::critical(this, tr("Error"), tr("No neighbors found for signature %1.").arg(id));
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
			ui_->label_actionsA,
			ui_->label_parentsA,
			ui_->label_childrenA,
			ui_->graphicsView_A,
			ui_->label_idA);
}

void DatabaseViewer::sliderBValueChanged(int value)
{
	this->update(value,
			ui_->label_indexB,
			ui_->label_actionsB,
			ui_->label_parentsB,
			ui_->label_childrenB,
			ui_->graphicsView_B,
			ui_->label_idB);
}

void DatabaseViewer::update(int value,
						QLabel * labelIndex,
						QLabel * labelActions,
						QLabel * labelParents,
						QLabel * labelChildren,
						rtabmap::ImageView * view,
						QLabel * labelId)
{
	UTimer timer;
	labelIndex->setText(QString::number(value));
	labelActions->clear();
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
				UINFO("loaded image(%d/%d) depth(%d/%d) depthConstant(%f)",
						imageMat.cols, imageMat.rows,
						depthMat.cols, depthMat.rows,
						depthConstant);
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
	UINFO("Time = %fs", timer.ticks());
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
