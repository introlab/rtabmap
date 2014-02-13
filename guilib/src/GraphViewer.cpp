/*
 * GraphViewer.cpp
 *
 *  Created on: 2014-01-24
 *      Author: mathieu
 */

#include "GraphViewer.h"

#include <QtGui/QGraphicsView>
#include <QtGui/QVBoxLayout>
#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsEllipseItem>
#include <QtGui/QWheelEvent>
#include <QtGui/QGraphicsSceneHoverEvent>
#include <QtGui/QMenu>
#include <QtGui/QDesktopServices>
#include <QtGui/QContextMenuEvent>
#include <QtGui/QColorDialog>
#include <QtSvg/QSvgGenerator>
#include <QtGui/QInputDialog>

#include <QtCore/QDir>
#include <QtCore/QDateTime>
#include <QtCore/QUrl>

#include <rtabmap/core/util3d.h>
#include <rtabmap/gui/UCv2Qt.h>
#include <rtabmap/utilite/UStl.h>

namespace rtabmap {

class NodeItem: public QGraphicsEllipseItem
{
public:
	// in meter
	NodeItem(int id, const Transform & pose, float radius, const cv::Mat & scan) :
		QGraphicsEllipseItem(QRectF(-radius,-radius,radius*2.0f,radius*2.0f)),
		_id(id),
		_pose(pose),
		_scan(scan)
	{
		this->setPos(-pose.y(),-pose.x());
		this->setBrush(pen().color());
		this->setAcceptHoverEvents(true);
	}

	void setColor(const QColor & color)
	{
		QPen p = this->pen();
		p.setColor(color);
		this->setPen(p);
		QBrush b = this->brush();
		b.setColor(color);
		this->setBrush(b);
	}

	void setPose(const Transform & pose) {this->setPos(-pose.y(),-pose.x()); _pose=pose;}
	void setScan(const cv::Mat & scan) {_scan = scan;}
	const cv::Mat & getScan() const {return _scan;}

protected:
	virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
	{
		this->setToolTip(QString("[%1] %2").arg(_id).arg(_pose.prettyPrint().c_str()));
		this->setScale(2);
		QGraphicsEllipseItem::hoverEnterEvent(event);
	}

	virtual void hoverLeaveEvent ( QGraphicsSceneHoverEvent * event )
	{
		qDeleteAll(this->children());
		this->setScale(1);
		QGraphicsEllipseItem::hoverEnterEvent(event);
	}

private:
	int _id;
	Transform _pose;
	cv::Mat _scan;
};

class LinkItem: public QGraphicsLineItem
{
public:
	// in meter
	LinkItem(const Transform & poseA, const Transform & poseB, bool loopClosure) :
		QGraphicsLineItem(-poseA.y(), -poseA.x(), -poseB.y(), -poseB.x()),
		_poseA(poseA),
		_poseB(poseB),
		_loopClosure(loopClosure)
	{
		this->setAcceptHoverEvents(true);
	}

	void setColor(const QColor & color)
	{
		QPen p = this->pen();
		p.setColor(color);
		this->setPen(p);
	}

	void setPoses(const Transform & poseA, const Transform & poseB)
	{
		this->setLine(-poseA.y(), -poseA.x(), -poseB.y(), -poseB.x());
		_poseA = poseA;
		_poseB = poseB;
	}

	bool isLoopClosure() const {return _loopClosure;}

protected:
	virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
	{
		QGraphicsLineItem::hoverEnterEvent(event);
	}

	virtual void hoverLeaveEvent ( QGraphicsSceneHoverEvent * event )
	{
		QGraphicsLineItem::hoverEnterEvent(event);
	}

private:
	Transform _poseA;
	Transform _poseB;
	bool _loopClosure;
};

GraphViewer::GraphViewer(QWidget * parent) :
		QGraphicsView(parent),
		_nodeColor(Qt::blue),
		_neighborColor(Qt::blue),
		_loopClosureColor(Qt::red),
		_root(0),
		_nodeRadius(0.1),
		_linkWidth(0),
		_gridMap(0)
{
	this->setScene(new QGraphicsScene(this));
	this->setDragMode(QGraphicsView::ScrollHandDrag);
	_workingDirectory = QDir::homePath();

	this->scene()->clear();
	_root = (QGraphicsItem *)this->scene()->addEllipse(QRectF(0,0,0,0));

	// add referential
	QGraphicsLineItem * item = this->scene()->addLine(0,0,0,-1, QPen(QBrush(Qt::red), _linkWidth));
	item->setZValue(100);
	item->setParentItem(_root);
	item = this->scene()->addLine(0,0,-1,0, QPen(QBrush(Qt::green), _linkWidth));
	item->setZValue(100);
	item->setParentItem(_root);

	_gridMap = this->scene()->addPixmap(QPixmap());
	_gridMap->scale(0.05, -0.05);
	_gridMap->setRotation(90);
	_gridMap->setZValue(0);
	_gridMap->setParentItem(_root);
}

GraphViewer::~GraphViewer()
{
}

void GraphViewer::updateGraph(const std::map<int, Transform> & poses,
				 const std::multimap<int, Link> & constraints,
				 const std::map<int, std::vector<unsigned char> > & scans)
{
	//Hide nodes and links
	for(QMap<int, NodeItem*>::iterator iter = _nodeItems.begin(); iter!=_nodeItems.end(); ++iter)
	{
		iter.value()->hide();
		iter.value()->setColor(_nodeColor); // reset color
	}
	for(QMap<int, LinkItem*>::iterator iter = _loopLinkItems.begin(); iter!=_loopLinkItems.end(); ++iter)
	{
		iter.value()->hide();
	}
	for(QMap<int, LinkItem*>::iterator iter = _neighborLinkItems.begin(); iter!=_neighborLinkItems.end(); ++iter)
	{
		iter.value()->hide();
	}

	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > scanClouds;
	for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		QMap<int, NodeItem*>::iterator itemIter = _nodeItems.find(iter->first);
		if(itemIter != _nodeItems.end())
		{
			itemIter.value()->setPose(iter->second);
			if(itemIter.value()->getScan().empty() && uContains(scans, iter->first))
			{
				cv::Mat depth2d = util3d::uncompressData(scans.at(iter->first));
				itemIter.value()->setScan(depth2d);
			}
			if(!itemIter.value()->getScan().empty() && _gridMap->isVisible())
			{
				scanClouds.insert(std::make_pair(iter->first, util3d::depth2DToPointCloud(itemIter.value()->getScan())));
			}
			itemIter.value()->show();
		}
		else
		{
			// create node item
			std::map<int, std::vector<unsigned char> >::const_iterator jter = scans.find(iter->first);
			cv::Mat depth2d;
			if(jter != scans.end())
			{
				depth2d = util3d::uncompressData(jter->second);
				if(_gridMap->isVisible())
				{
					scanClouds.insert(std::make_pair(iter->first, util3d::depth2DToPointCloud(depth2d)));
				}
			}

			const Transform & pose = iter->second;
			NodeItem * item = new NodeItem(iter->first, pose, _nodeRadius, depth2d);
			this->scene()->addItem(item);
			item->setZValue(2);
			item->setColor(_nodeColor);
			item->setParentItem(_root);
			_nodeItems.insert(iter->first, item);
		}
	}

	for(std::multimap<int, Link>::const_iterator iter=constraints.begin(); iter!=constraints.end(); ++iter)
	{
		std::map<int, Transform>::const_iterator jterA = poses.find(iter->first);
		std::map<int, Transform>::const_iterator jterB = poses.find(iter->second.to());
		if(jterA != poses.end() && jterB != poses.end())
		{
			const Transform & poseA = jterA->second;
			const Transform & poseB = jterB->second;
			bool loopClosure = iter->second.type() != Link::kNeighbor;

			if(loopClosure && _loopLinkItems.contains(iter->first))
			{
				QMap<int, LinkItem*>::iterator itemIter = _loopLinkItems.find(iter->first);
				itemIter.value()->setPoses(poseA, poseB);
				itemIter.value()->show();
			}
			else if(!loopClosure && _neighborLinkItems.contains(iter->first))
			{
				QMap<int, LinkItem*>::iterator itemIter = _neighborLinkItems.find(iter->first);
				itemIter.value()->setPoses(poseA, poseB);
				itemIter.value()->show();
			}
			else
			{
				//create a link item
				LinkItem * item = new LinkItem(poseA, poseB, loopClosure);
				QPen p = item->pen();
				p.setWidthF(_linkWidth);
				item->setPen(p);
				item->setColor(loopClosure?_loopClosureColor:_neighborColor);
				this->scene()->addItem(item);
				item->setZValue(1);
				item->setParentItem(_root);
				if(loopClosure)
				{
					_loopLinkItems.insert(iter->first, item);
				}
				else
				{
					_neighborLinkItems.insert(iter->first, item);
				}
			}
		}
	}

	//remove not used nodes and links
	for(QMap<int, NodeItem*>::iterator iter = _nodeItems.begin(); iter!=_nodeItems.end();)
	{
		if(!iter.value()->isVisible())
		{
			delete iter.value();
			iter = _nodeItems.erase(iter);
		}
		else
		{
			++iter;
		}
	}
	for(QMap<int, LinkItem*>::iterator iter = _loopLinkItems.begin(); iter!=_loopLinkItems.end();)
	{
		if(!iter.value()->isVisible())
		{
			delete iter.value();
			iter = _loopLinkItems.erase(iter);
		}
		else
		{
			++iter;
		}
	}
	for(QMap<int, LinkItem*>::iterator iter = _neighborLinkItems.begin(); iter!=_neighborLinkItems.end();)
	{
		if(!iter.value()->isVisible())
		{
			delete iter.value();
			iter = _neighborLinkItems.erase(iter);
		}
		else
		{
			++iter;
		}
	}

	if(scanClouds.size())
	{
		float xMin=0.0f, yMin=0.0f;
		cv::Mat map8S = util3d::create2DMap(poses, scanClouds, 0.05, xMin, yMin);
		cv::Mat map8U(map8S.rows, map8S.cols, CV_8U);
		//convert to gray scaled map
		for (int i = 0; i < map8S.rows; ++i)
		{
			for (int j = 0; j < map8S.cols; ++j)
			{
				char gray = map8S.at<char>(i, j);
				if(gray == -1)
				{
					gray = 89;
				}
				else if(gray == 0)
				{
					gray = 178;
				}
				else if(gray == 100)
				{
					gray = 0;
				}
				map8U.at<char>(i, j) = gray;
			}
		}
		QImage image = uCvMat2QImage(map8U, false);
		_gridMap->setPixmap(QPixmap::fromImage(image));
		_gridMap->setPos(-yMin, -xMin);
	}

	if(_nodeItems.size())
	{
		(--_nodeItems.end()).value()->setColor(Qt::gray);
	}

	this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents
	this->fitInView(this->scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
}

void GraphViewer::clearGraph()
{
	qDeleteAll(_nodeItems);
	_nodeItems.clear();
	qDeleteAll(_neighborLinkItems);
	_neighborLinkItems.clear();
	qDeleteAll(_loopLinkItems);
	_loopLinkItems.clear();
	_gridMap->setPixmap(QPixmap());
	this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents
}

void GraphViewer::wheelEvent ( QWheelEvent * event )
{
	if(event->delta() > 0)
	{
		this->scale(0.95, 0.95);
	}
	else
	{
		this->scale(1.05, 1.05);
	}
}

void GraphViewer::contextMenuEvent(QContextMenuEvent * event)
{
	QMenu menu;
	QAction * aScreenShotPNG = menu.addAction(tr("Take a screenshot (PNG)"));
	QAction * aScreenShotSVG = menu.addAction(tr("Take a screenshot (SVG)"));
	menu.addSeparator();
	QAction * aChangeNodeColor = menu.addAction(tr("Set node color..."));
	QAction * aChangeNeighborColor = menu.addAction(tr("Set neighbor link color..."));
	QAction * aChangeLoopColor = menu.addAction(tr("Set loop closure link color..."));
	menu.addSeparator();
	QAction * aSetNodeSize = menu.addAction(tr("Set node radius..."));
	QAction * aSetLinkSize = menu.addAction(tr("Set link width..."));
	menu.addSeparator();
	QAction * aShowHideGridMap;
	if(_gridMap->isVisible())
	{
		aShowHideGridMap = menu.addAction(tr("Hide grid map"));
	}
	else
	{
		aShowHideGridMap = menu.addAction(tr("Show grid map"));
	}

	QAction * r = menu.exec(event->globalPos());
	if(r == aScreenShotPNG || r == aScreenShotSVG)
	{
		if(_root)
		{
			QString targetDir = _workingDirectory + "/ScreensCaptured";
			QDir dir;
			if(!dir.exists(targetDir))
			{
				dir.mkdir(targetDir);
			}
			targetDir += "/";
			targetDir += "Graph_view";
			if(!dir.exists(targetDir))
			{
				dir.mkdir(targetDir);
			}
			targetDir += "/";
			bool isPNG = r == aScreenShotPNG;
			QString name = (QDateTime::currentDateTime().toString("yyMMddhhmmsszzz") + (isPNG?".png":".svg"));

			//_root->setScale(this->transform().m11()); // current view
			_root->setScale(20); // grid map precision (5cm)

			this->scene()->clearSelection();                                  // Selections would also render to the file
			this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents
			QSize sceneSize = this->scene()->sceneRect().size().toSize();

			if(isPNG)
			{
				QImage image(sceneSize, QImage::Format_ARGB32);  // Create the image with the exact size of the shrunk scene
				image.fill(Qt::transparent);                     // Start all pixels transparent
				QPainter painter(&image);

				this->scene()->render(&painter);
				image.save(targetDir + name);
			}
			else
			{
				QSvgGenerator svgGen;

				svgGen.setFileName( targetDir + name );
				svgGen.setSize(sceneSize);
				svgGen.setViewBox(QRect(0, 0, sceneSize.width(), sceneSize.height()));
				svgGen.setTitle(tr("RTAB-Map graph"));
				svgGen.setDescription(tr("RTAB-Map map and graph"));

				QPainter painter( &svgGen );

				this->scene()->render(&painter);
			}

			//reset scale
			_root->setScale(1.0f);
			this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents


			QDesktopServices::openUrl(QUrl::fromLocalFile(targetDir + name));
		}
	}
	else if(r == aChangeNodeColor ||
			r == aChangeNeighborColor ||
			r == aChangeLoopColor)
	{
		QColor color;
		if(r == aChangeNodeColor)
		{
			color = _nodeColor;
		}
		else if(r == aChangeNeighborColor)
		{
			color = _neighborColor;
		}
		else if(r == aChangeLoopColor)
		{
			color = _loopClosureColor;
		}
		color = QColorDialog::getColor(color, this);

		if(r == aChangeNodeColor)
		{
			_nodeColor = color;
		}
		else if(r == aChangeNeighborColor)
		{
			_neighborColor = color;
		}
		else if(r == aChangeLoopColor)
		{
			_loopClosureColor = color;
		}

		QList<QGraphicsItem*> items = this->scene()->items();
		for(int i=0; i<items.size(); ++i)
		{
			LinkItem * link = qgraphicsitem_cast<LinkItem *>(items[i]);
			NodeItem * node = qgraphicsitem_cast<NodeItem *>(items[i]);
			if(r == aChangeNodeColor && node)
			{
				node->setColor(color);
			}
			else if(r == aChangeNeighborColor && link && !link->isLoopClosure())
			{
				link->setColor(color);
			}
			else if(r == aChangeLoopColor && link && link->isLoopClosure())
			{
				link->setColor(color);
			}
		}
	}
	else if(r == aSetNodeSize)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Node radius"), tr("Radius (m)"), _nodeRadius, 0.01, 1, 2, &ok);
		if(ok)
		{
			_nodeRadius = value;
			QList<QGraphicsItem*> items = this->scene()->items();
			for(int i=0; i<items.size(); ++i)
			{
				NodeItem * node = qgraphicsitem_cast<NodeItem *>(items[i]);
				if(node)
				{
					node->setRect(-_nodeRadius, -_nodeRadius, _nodeRadius*2.0f, _nodeRadius*2.0f);
				}
			}
		}
	}
	else if(r == aSetLinkSize)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Link width"), tr("Width (m)"), _linkWidth, 0, 1, 2, &ok);
		if(ok)
		{
			_linkWidth = value;
			QList<QGraphicsItem*> items = this->scene()->items();
			for(int i=0; i<items.size(); ++i)
			{
				QGraphicsLineItem * line = qgraphicsitem_cast<QGraphicsLineItem *>(items[i]);
				if(line)
				{
					QPen pen = line->pen();
					pen.setWidthF(_linkWidth);
					line->setPen(pen);
				}
			}
		}
	}
	else if(r == aShowHideGridMap)
	{
		_gridMap->setVisible(!_gridMap->isVisible());
	}
}

} /* namespace rtabmap */
