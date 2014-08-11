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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rtabmap/core/Link.h>

class Ui_DatabaseViewer;
class QGraphicsScene;
class QGraphicsView;
class QLabel;

namespace rtabmap
{
class Memory;
class ImageView;

class RTABMAPGUI_EXP DatabaseViewer : public QMainWindow
{
	Q_OBJECT

public:
	DatabaseViewer(QWidget * parent = 0);
	virtual ~DatabaseViewer();
	bool openDatabase(const QString & path);

protected:
	virtual void resizeEvent(QResizeEvent* anEvent);

private slots:
	void openDatabase();
	void generateGraph();
	void exportDatabase();
	void extractImages();
	void generateLocalGraph();
	void generateTOROGraph();
	void view3DMap();
	void generate3DMap();
	void detectMoreLoopClosures();
	void refineAllNeighborLinks();
	void refineAllLoopClosureLinks();
	void sliderAValueChanged(int);
	void sliderBValueChanged(int);
	void sliderAMoved(int);
	void sliderBMoved(int);
	void sliderNeighborValueChanged(int);
	void sliderLoopValueChanged(int);
	void sliderIterationsValueChanged(int);
	void updateGraphView();
	void refineConstraint();
	void addConstraint();
	void resetConstraint();
	void rejectConstraint();

private:
	void updateIds();
	void update(int value,
				QLabel * labelIndex,
				QLabel * labelParents,
				QLabel * labelChildren,
				rtabmap::ImageView * view,
				QLabel * labelId);
	void updateWordsMatching();
	void updateConstraintView(const rtabmap::Link & link);
	void updateConstraintButtons();
	std::multimap<int, Link>::iterator findLink(
				std::multimap<int, Link> & links,
				int from,
				int to);
	Link findActiveLink(int from, int to);
	bool containsLink(
			std::multimap<int, Link> & links,
			int from,
			int to);
	std::multimap<int, rtabmap::Link> updateLinksWithModifications(
			const std::multimap<int, rtabmap::Link> & edgeConstraints);
	void updateLoopClosuresSlider(int from = 0, int to = 0);
	void refineConstraint(int from, int to);
	bool addConstraint(int from, int to, bool silent);
	std::map<int, int> generateGraph(int fromNode, int margin);
	std::map<int, Transform> optimizeGraph(
			const std::map<int, int> & ids,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			std::list<std::map<int, rtabmap::Transform> > * graphes = 0);

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
	std::multimap<int, rtabmap::Link> linksRefined_;
	std::multimap<int, rtabmap::Link> linksAdded_;
	std::multimap<int, rtabmap::Link> linksRemoved_;
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > scans_;
};

}

#endif /* DATABASEVIEWER_H_ */
