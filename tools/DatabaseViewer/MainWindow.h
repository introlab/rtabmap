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

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <QtGui/QMainWindow>
#include <QtCore/QByteArray>
#include <QtCore/QMap>
#include <QtCore/QSet>
#include <QtGui/QImage>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <set>

class Ui_MainWindow;
class QGraphicsScene;
class QGraphicsView;
class QLabel;

namespace rtabmap
{
class Memory;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget * parent = 0);
	virtual ~MainWindow();

private slots:
	void openDatabase();
	void cleanDatabase();
	void cleanLocalGraph();
	void generateGraph();
	void generateLocalGraph();
	void sliderAValueChanged(int);
	void sliderBValueChanged(int);
	void sliderAMoved(int);
	void sliderBMoved(int);

private:
	void updateIds();
	QImage ipl2QImage(const IplImage *newImage);
	void drawKeypoints(const std::multimap<int, cv::KeyPoint> & refWords, QGraphicsScene * scene);
	void update(int value,
				QLabel * labelIndex,
				QLabel * labelActions,
				QLabel * labelParents,
				QLabel * labelChildren,
				QGraphicsView * view,
				QLabel * labelId);

private:
	Ui_MainWindow * ui_;
	QMap<int, QByteArray> imagesMap_;
	QList<int> ids_;
	rtabmap::Memory * memory_;
	QString pathDatabase_;
};

#endif /* MAINWINDOW_H_ */
