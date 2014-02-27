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

#ifndef DATABASEVIEWER_H_
#define DATABASEVIEWER_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <QtGui/QMainWindow>
#include <QtCore/QByteArray>
#include <QtCore/QMap>
#include <QtCore/QSet>
#include <QtGui/QImage>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <set>

#include <rtabmap/core/Link.h>

class Ui_DatabaseViewer;
class QGraphicsScene;
class QGraphicsView;
class QLabel;

namespace rtabmap
{
class Memory;
class ImageView;
}

class RTABMAPGUI_EXP DatabaseViewer : public QMainWindow
{
	Q_OBJECT

public:
	DatabaseViewer(QWidget * parent = 0);
	virtual ~DatabaseViewer();
	bool openDatabase(const QString & path);

private slots:
	void openDatabase();
	void generateGraph();
	void exportDatabase();
	void extractImages();
	void generateLocalGraph();
	void generateTOROGraph();
	void view3DMap();
	void generate3DMap();
	void sliderAValueChanged(int);
	void sliderBValueChanged(int);
	void sliderAMoved(int);
	void sliderBMoved(int);
	void sliderNeighborValueChanged(int);
	void sliderLoopValueChanged(int);
	void sliderIterationsValueChanged(int);
	void updateGraphView();

private:
	void updateIds();
	void update(int value,
				QLabel * labelIndex,
				QLabel * labelParents,
				QLabel * labelChildren,
				rtabmap::ImageView * view,
				QLabel * labelId);
	void updateConstraintView(const rtabmap::Link & link);

private:
	Ui_DatabaseViewer * ui_;
	QList<int> ids_;
	QMap<int, int> idToIndex_;
	QList<rtabmap::Link> neighborLinks_;
	QList<rtabmap::Link> loopLinks_;
	rtabmap::Memory * memory_;
	QString pathDatabase_;
	std::list<std::map<int, rtabmap::Transform> > graphes_;
	std::map<int, rtabmap::Transform> poses_;
	std::multimap<int, rtabmap::Link> links_;
	std::map<int, std::vector<unsigned char> > scans_;
};

#endif /* DATABASEVIEWER_H_ */
