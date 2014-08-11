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

#ifndef GRAPHVIEWER_H_
#define GRAPHVIEWER_H_

#include <QtGui/QGraphicsView>
#include <QtCore/QMap>
#include <rtabmap/core/Link.h>
#include <opencv2/opencv.hpp>
#include <map>

class QGraphicsItem;
class QGraphicsPixmapItem;
class QGraphicsItemGroup;

namespace rtabmap {

class NodeItem;
class LinkItem;

class GraphViewer : public QGraphicsView {
public:
	GraphViewer(QWidget * parent = 0);
	virtual ~GraphViewer();

	void updateGraph(const std::map<int, Transform> & poses,
					 const std::multimap<int, Link> & constraints);
	void updateMap(const cv::Mat & map8U, float resolution, float xMin, float yMin);
	void clearGraph();
	void clearMap();
	void clearAll();

	void setWorkingDirectory(const QString & path) {_workingDirectory = path;}

protected:
	virtual void wheelEvent ( QWheelEvent * event );
	virtual void contextMenuEvent(QContextMenuEvent * event);

private:
	QString _workingDirectory;
	QColor _nodeColor;
	QColor _neighborColor;
	QColor _loopClosureColor;
	QGraphicsItem * _root;
	QMap<int, NodeItem*> _nodeItems;
	QMap<int, LinkItem*> _neighborLinkItems;
	QMap<int, LinkItem*> _loopLinkItems;
	float _nodeRadius;
	float _linkWidth;
	QGraphicsPixmapItem * _gridMap;
	QGraphicsItemGroup * _lastReferential;
	float _gridCellSize;
};

} /* namespace rtabmap */
#endif /* GRAPHVIEWER_H_ */
