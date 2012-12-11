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
#include <utilite/ULogger.h>
#include <utilite/UDirectory.h>
#include <utilite/UConversion.h>
#include <opencv2/core/core_c.h>
#include <utilite/UTimer.h>
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/DBDriver.h"
#include "rtabmap/gui/KeypointItem.h"

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

	ui_->graphicsView_A->setScene(new QGraphicsScene(this));
	ui_->graphicsView_B->setScene(new QGraphicsScene(this));

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
			imagesMap_.clear();
			ids_.clear();
		}

		std::string driverType = "sqlite3";
		rtabmap::ParametersMap parameters;
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kDbSqlite3InMemory(), "false"));

		memory_ = new rtabmap::Memory(parameters);

		if(!memory_->init(path.toStdString()))
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
	ids_.prepend(0);

	UDEBUG("Loaded %d ids", ids_.size());

	if(ids_.size())
	{
		ui_->horizontalSlider_A->setMinimum(0);
		ui_->horizontalSlider_B->setMinimum(0);
		ui_->horizontalSlider_A->setMaximum(ids_.size()-1);
		ui_->horizontalSlider_B->setMaximum(ids_.size()-1);
		ui_->horizontalSlider_A->setSliderPosition(0);
		ui_->horizontalSlider_B->setSliderPosition(0);
		ui_->horizontalSlider_A->setEnabled(true);
		ui_->horizontalSlider_B->setEnabled(true);
		ui_->label_idA->setText("0");
		ui_->label_idB->setText("0");
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

void DatabaseViewer::drawKeypoints(const std::multimap<int, cv::KeyPoint> & refWords, QGraphicsScene * scene)
{
	if(!scene)
	{
		return;
	}
	rtabmap::KeypointItem * item = 0;
	int alpha = 70;
	for(std::multimap<int, cv::KeyPoint>::const_iterator i = refWords.begin(); i != refWords.end(); ++i )
	{
		const cv::KeyPoint & r = (*i).second;
		int id = (*i).first;
		QString info = QString( "WordRef = %1\n"
								"Laplacian = %2\n"
								"Dir = %3\n"
								"Hessian = %4\n"
								"X = %5\n"
								"Y = %6\n"
								"Size = %7").arg(id).arg(1).arg(r.angle).arg(r.response).arg(r.pt.x).arg(r.pt.y).arg(r.size);
		float radius = r.size*1.2/9.*2;

		item = new rtabmap::KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 255, 0, alpha));

		scene->addItem(item);
		item->setZValue(1);
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
						QGraphicsView * view,
						QLabel * labelId)
{
	UTimer timer;
	labelIndex->setText(QString::number(value));
	labelActions->clear();
	labelParents->clear();
	labelChildren->clear();
	if(value >= 0 && value < ids_.size())
	{
		view->scene()->clear();
		int id = ids_.at(value);
		labelId->setText(QString::number(id));
		if(id>0)
		{
			//image
			QImage img;
			QMap<int, QByteArray>::iterator iter = imagesMap_.find(id);
			if(iter == imagesMap_.end())
			{
				if(memory_)
				{
					cv::Mat image = memory_->getImage(id);
					if(!image.empty())
					{
						IplImage iplImg = image;
						img = ipl2QImage(&iplImg);
						if(!img.isNull())
						{
							QByteArray ba;
							QBuffer buffer(&ba);
							buffer.open(QIODevice::WriteOnly);
							img.save(&buffer, "BMP"); // writes image into ba in BMP format
							imagesMap_.insert(id, ba);
						}
					}
				}
			}
			else
			{
				img.loadFromData(iter.value(), "BMP");
			}

			if(memory_ && dynamic_cast<rtabmap::Memory*>(memory_))
			{
				std::multimap<int, cv::KeyPoint> words = dynamic_cast<rtabmap::Memory*>(memory_)->getWords(id);
				if(words.size())
				{
					drawKeypoints(words, view->scene());
				}
			}

			if(!img.isNull())
			{
				view->scene()->addPixmap(QPixmap::fromImage(img));
			}
			else
			{
				ULOGGER_DEBUG("Image is empty");
			}

			// loops
			std::set<int> parents;
			std::set<int> children;
			memory_->getLoopClosureIds(id, parents, children, true);
			if(parents.size())
			{
				QString str;
				for(std::set<int>::iterator iter=parents.begin(); iter!=parents.end(); ++iter)
				{
					str.append(QString("%1 ").arg(*iter));
				}
				labelParents->setText(str);
			}
			if(children.size())
			{
				QString str;
				for(std::set<int>::iterator iter=children.begin(); iter!=children.end(); ++iter)
				{
					str.append(QString("%1 ").arg(*iter));
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

QImage DatabaseViewer::ipl2QImage(const IplImage *newImage)
{
	QImage qtemp;
	if (newImage && newImage->depth == IPL_DEPTH_8U && cvGetSize(newImage).width>0)
	{
		int x;
		int y;
		char* data = newImage->imageData;

		qtemp= QImage(newImage->width, newImage->height,QImage::Format_RGB32 );
		for( y = 0; y < newImage->height; y++, data +=newImage->widthStep )
		{
			for( x = 0; x < newImage->width; x++)
			{
				uint *p = (uint*)qtemp.scanLine (y) + x;
				*p = qRgb(data[x * newImage->nChannels+2], data[x * newImage->nChannels+1],data[x * newImage->nChannels]);
			}
		}
	}
	else
	{
		ULOGGER_ERROR("Wrong IplImage format");
	}
 return qtemp;
}
