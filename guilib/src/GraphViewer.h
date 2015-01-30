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

	Q_OBJECT;

public:
	GraphViewer(QWidget * parent = 0);
	virtual ~GraphViewer();

	void updateGraph(const std::map<int, Transform> & poses,
					 const std::multimap<int, Link> & constraints);
	void updateMap(const cv::Mat & map8U, float resolution, float xMin, float yMin);
	void updatePosterior(const std::map<int, float> & posterior);
	void clearGraph();
	void clearMap();
	void clearPosterior();
	void clearAll();

	//getters
	const QString & getWorkingDirectory() const {return _workingDirectory;}
	float getNodeRadius() const {return _nodeRadius;}
	float getLinkWidth() const {return _linkWidth;}
	const QColor & getNodeColor() const {return _nodeColor;}
	const QColor & getNeighborColor() const {return _neighborColor;}
	const QColor & getGlobalLoopClosureColor() const {return _loopClosureColor;}
	const QColor & getLocalLoopClosureColor() const {return _loopClosureLocalColor;}
	const QColor & getUserLoopClosureColor() const {return _loopClosureUserColor;}
	const QColor & getVirtualLoopClosureColor() const {return _loopClosureVirtualColor;}
	bool isGridMapVisible() const;

	// setters
	void setWorkingDirectory(const QString & path);
	void setNodeRadius(float radius);
	void setLinkWidth(float width);
	void setNodeColor(const QColor & color);
	void setNeighborColor(const QColor & color);
	void setGlobalLoopClosureColor(const QColor & color);
	void setLocalLoopClosureColor(const QColor & color);
	void setUserLoopClosureColor(const QColor & color);
	void setVirtualLoopClosureColor(const QColor & color);
	void setGridMapVisible(bool visible);

signals:
	void configChanged();

public slots:
	void restoreDefaults();

protected:
	virtual void wheelEvent ( QWheelEvent * event );
	virtual void contextMenuEvent(QContextMenuEvent * event);

private:
	QString _workingDirectory;
	QColor _nodeColor;
	QColor _neighborColor;
	QColor _loopClosureColor;
	QColor _loopClosureLocalColor;
	QColor _loopClosureUserColor;
	QColor _loopClosureVirtualColor;
	QGraphicsItem * _root;
	QMap<int, NodeItem*> _nodeItems;
	QMultiMap<int, LinkItem*> _linkItems;
	float _nodeRadius;
	float _linkWidth;
	QGraphicsPixmapItem * _gridMap;
	QGraphicsItemGroup * _lastReferential;
	float _gridCellSize;
};

} /* namespace rtabmap */
#endif /* GRAPHVIEWER_H_ */
