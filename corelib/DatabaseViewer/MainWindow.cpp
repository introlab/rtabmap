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

#include "MainWindow.h"
#include "ui_MainWindow.h"
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
#include "KeypointMemory.h"
#include "rtabmap/core/DBDriver.h"

MainWindow::MainWindow(QWidget * parent) :
	QMainWindow(parent),
	memory_(0)
{
	pathDatabase_ = QDir::homePath()+"/Documents/RTAB-Map"; //use home directory by default

	if(!UDirectory::exists(pathDatabase_.toStdString()))
	{
		pathDatabase_ = QDir::homePath();
	}

	ui_ = new Ui_MainWindow();
	ui_->setupUi(this);

	connect(ui_->actionQuit, SIGNAL(triggered()), this, SLOT(close()));

	// connect actions with custom slots
	connect(ui_->actionOpen_database, SIGNAL(triggered()), this, SLOT(openDatabase()));
	connect(ui_->actionGenerate_graph_dot, SIGNAL(triggered()), this, SLOT(generateGraph()));
	connect(ui_->actionGenerate_local_graph_dot, SIGNAL(triggered()), this, SLOT(generateLocalGraph()));
	connect(ui_->actionClean_database, SIGNAL(triggered()), this, SLOT(cleanDatabase()));
	connect(ui_->actionClean_local_graph, SIGNAL(triggered()), this, SLOT(cleanLocalGraph()));

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

MainWindow::~MainWindow()
{
	delete ui_;
	if(memory_)
	{
		delete memory_;
	}
}

void MainWindow::openDatabase()
{
	QString path = QFileDialog::getOpenFileName(this, tr("Select file"), pathDatabase_, tr("Databases (*.db)"));
	if(!path.isEmpty())
	{
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
		memory_ = new rtabmap::KeypointMemory(parameters);
		if(!memory_)
		{
			QMessageBox::warning(this, "Database error", tr("Can't create database driver \"%1\"").arg(driverType.c_str()));
		}
		else if(!memory_->init(driverType, path.toStdString()))
		{
			QMessageBox::warning(this, "Database error", tr("Can't open database \"%1\"").arg(path));
		}
		else
		{
			pathDatabase_ = path;
			updateIds();
		}
	}


}

void MainWindow::updateIds()
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

void MainWindow::cleanDatabase()
{
	if(!memory_ || !ids_.size())
	{
		QMessageBox::warning(this, tr("Cannot clean the database"), tr("A database (not empty) must must loaded first...\nUse File->Open database."));
		return;
	}

	bool ok;
	int depth = QInputDialog::getInt(this, tr("Depth around the weighted locations?"), tr("Depth"), 10, 1, 1000, 1, &ok);
	if(ok)
	{
		UINFO("Cleaning...");
		//Remove all signatures with null weight and between two intersections
		memory_->cleanLTM(depth);

		//dbDriver_->executeNoResult(std::string("DELETE FROM Signature WHERE loopClosureId!=0;"));
		//dbDriver_->executeNoResult(std::string("DELETE FROM Neighbor WHERE NOT EXISTS (SELECT * FROM Signature WHERE Signature.id = Neighbor.sid);"));
		//dbDriver_->executeNoResult(std::string("DELETE FROM Neighbor WHERE NOT EXISTS (SELECT * FROM Signature WHERE Signature.id = Neighbor.nid);"));

		// Clean links
		//dbDriver_->executeNoResult(std::string("DELETE FROM Map_SS_VW WHERE NOT EXISTS (SELECT * FROM Signature WHERE Signature.id = signatureId);"));

		// Clean words
		//dbDriver_->deleteUnreferencedWords();

		updateIds();
		UINFO("Finished cleaning!");
	}
}

void MainWindow::cleanLocalGraph()
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
			UTimer timer;
			UINFO("Cleaning local graph for location %d", id);

			memory_->cleanLocalGraph(id, margin);

			updateIds();
			UINFO("time=%fs", timer.ticks());
		}
	}
}

void MainWindow::generateGraph()
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

void MainWindow::generateLocalGraph()
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
				std::map<int, int> ids;
				memory_->getNeighborsId(ids, id, margin-1, true);
				ids.insert(std::pair<int,int>(id, 0));
				std::set<int> idsSet;
				for(std::map<int, int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
				{
					idsSet.insert(idsSet.end(), iter->first);
				}
				memory_->generateGraph(path.toStdString(), idsSet);
			}
		}
	}
}

void MainWindow::sliderAValueChanged(int value)
{
	ui_->label_indexA->setText(QString::number(value));
	if(value >= 0 && value < ids_.size())
	{
		ui_->graphicsView_A->scene()->clear();
		int id = ids_.at(value);
		ui_->label_idA->setText(QString::number(id));
		if(id>0)
		{
			QImage img;
			QMap<int, QByteArray>::iterator iter = imagesMap_.find(id);
			if(iter == imagesMap_.end())
			{
				if(memory_)
				{
					IplImage * image = memory_->getImage(id);
					if(image)
					{
						img = ipl2QImage(image);
						cvReleaseImage(&image);
						if(!img.isNull())
						{
							QByteArray ba;
							QBuffer buffer(&ba);
							buffer.open(QIODevice::WriteOnly);
							img.save(&buffer, "JPEG"); // writes image into ba in JPEG format
							imagesMap_.insert(id, ba);
						}
					}
				}
			}
			else
			{
				img.loadFromData(iter.value(), "JPEG");
			}

			if(!img.isNull())
			{
				ui_->graphicsView_A->scene()->addPixmap(QPixmap::fromImage(img));
			}
			else
			{
				ULOGGER_DEBUG("Image is empty");
			}
		}

		ui_->label_idA->setText(QString::number(id));
		ui_->graphicsView_A->fitInView(ui_->graphicsView_A->scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
	}
	else
	{
		ULOGGER_ERROR("Slider index out of range ?");
	}
}

void MainWindow::sliderBValueChanged(int value)
{
	ui_->label_indexB->setText(QString::number(value));
	if(value >= 0 && value < ids_.size())
	{
		ui_->graphicsView_B->scene()->clear();
		int id = ids_.at(value);
		ui_->label_idB->setText(QString::number(id));
		if(id>0)
		{
			QImage img;
			QMap<int, QByteArray>::iterator iter = imagesMap_.find(id);
			if(iter == imagesMap_.end())
			{
				if(memory_)
				{
					IplImage * image = memory_->getImage(id);
					if(image)
					{
						img = ipl2QImage(image);
						cvReleaseImage(&image);

						if(!img.isNull())
						{
							QByteArray ba;
							QBuffer buffer(&ba);
							buffer.open(QIODevice::WriteOnly);
							img.save(&buffer, "JPEG"); // writes image into ba in JPEG format
							imagesMap_.insert(id, ba);
						}
					}
				}
			}
			else
			{
				img.loadFromData(iter.value(), "JPEG");
			}

			if(!img.isNull())
			{
				ui_->graphicsView_B->scene()->addPixmap(QPixmap::fromImage(img));
			}
			else
			{
				ULOGGER_DEBUG("Image is empty");
			}
		}

		ui_->label_idB->setText(QString::number(id));
		ui_->graphicsView_B->fitInView(ui_->graphicsView_B->scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
	}
	else
	{
		ULOGGER_ERROR("Slider index out of range ?");
	}
}

void MainWindow::sliderAMoved(int value)
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

void MainWindow::sliderBMoved(int value)
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

QImage MainWindow::ipl2QImage(const IplImage *newImage) //fct recuperer sur le net, converti un ldImage en QImage
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
