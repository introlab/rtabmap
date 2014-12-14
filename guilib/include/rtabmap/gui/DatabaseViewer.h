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
class Signature;

class RTABMAPGUI_EXP DatabaseViewer : public QMainWindow
{
	Q_OBJECT

public:
	DatabaseViewer(QWidget * parent = 0);
	virtual ~DatabaseViewer();
	bool openDatabase(const QString & path);

protected:
	virtual void showEvent(QShowEvent* anEvent);
	virtual void resizeEvent(QResizeEvent* anEvent);
	virtual void closeEvent(QCloseEvent* event);

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
	void refineVisuallyAllNeighborLinks();
	void refineVisuallyAllLoopClosureLinks();
	void sliderAValueChanged(int);
	void sliderBValueChanged(int);
	void sliderAMoved(int);
	void sliderBMoved(int);
	void sliderNeighborValueChanged(int);
	void sliderLoopValueChanged(int);
	void sliderIterationsValueChanged(int);
	void updateGraphView();
	void refineConstraint();
	void refineConstraintVisually();
	void addConstraint();
	void resetConstraint();
	void rejectConstraint();
	void updateConstraintView();

private:
	void updateIds();
	void update(int value,
				QLabel * labelIndex,
				QLabel * labelParents,
				QLabel * labelChildren,
				rtabmap::ImageView * view,
				QLabel * labelId,
				bool updateConstraintView = true);
	void updateStereo(const Signature * data);
	void updateWordsMatching();
	void updateConstraintView(const rtabmap::Link & link,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudFrom = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>),
			const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudTo = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>),
			bool updateImageSliders = true);
	void updateConstraintButtons();
	Link findActiveLink(int from, int to);
	bool containsLink(
			std::multimap<int, Link> & links,
			int from,
			int to);
	std::multimap<int, rtabmap::Link> updateLinksWithModifications(
			const std::multimap<int, rtabmap::Link> & edgeConstraints);
	void updateLoopClosuresSlider(int from = 0, int to = 0);
	void refineConstraint(int from, int to, bool updateGraph);
	void refineConstraintVisually(int from, int to, bool updateGraph);
	bool addConstraint(int from, int to, bool silent, bool updateGraph);

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
