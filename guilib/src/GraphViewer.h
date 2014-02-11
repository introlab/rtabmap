/*
 * GraphViewer.h
 *
 *  Created on: 2014-01-24
 *      Author: mathieu
 */

#ifndef GRAPHVIEWER_H_
#define GRAPHVIEWER_H_

#include <QtGui/QGraphicsView>
#include <QtCore/QMap>
#include <rtabmap/core/Link.h>
#include <map>

class QGraphicsItem;
class QGraphicsPixmapItem;

namespace rtabmap {

class NodeItem;
class LinkItem;

class GraphViewer : public QGraphicsView {
public:
	GraphViewer(QWidget * parent = 0);
	virtual ~GraphViewer();

	void updateGraph(const std::map<int, Transform> & poses,
					 const std::multimap<int, Link> & constraints,
					 const std::map<int, std::vector<unsigned char> > & scans);
	void clearGraph();

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
};

} /* namespace rtabmap */
#endif /* GRAPHVIEWER_H_ */
