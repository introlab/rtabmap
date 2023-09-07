/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/gui/GraphViewer.h"

#include <QGraphicsView>
#include <QVBoxLayout>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QtGui/QWheelEvent>
#include <QGraphicsSceneHoverEvent>
#include <QMenu>
#include <QtGui/QDesktopServices>
#include <QtGui/QContextMenuEvent>
#include <QColorDialog>
#include <QPrinter>
#include <QFileDialog>
#ifdef QT_SVG_LIB
#include <QtSvg/QSvgGenerator>
#endif
#include <QInputDialog>
#include <QMessageBox>

#include <QtCore/QDir>
#include <QtCore/QDateTime>
#include <QtCore/QUrl>

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/GeodeticCoords.h>
#include <rtabmap/utilite/UCv2Qt.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>

#include <QtGlobal>
#if QT_VERSION >= 0x050000
	#include <QStandardPaths>
#endif

#include <fstream>

namespace rtabmap {

class NodeItem: public QGraphicsEllipseItem
{
public:
	// in meter
	NodeItem(int id, int mapId, const Transform & pose, float radius, int weight, GraphViewer::ViewPlane plane, float linkWidth) :
		QGraphicsEllipseItem(QRectF(-radius*100.0f,-radius*100.0f,radius*100.0f*2.0f,radius*100.0f*2.0f)),
		_id(id),
		_mapId(mapId),
		_weight(weight),
		_pose(pose),
		_line(0)
	{
		this->setPose(pose, plane);
		this->setBrush(pen().color());
		this->setAcceptHoverEvents(true);
		float r,p,yaw;
		pose.getEulerAngles(r, p, yaw);
		radius*=100.0f;
		_line = new QGraphicsLineItem(0,0,-radius*sin(yaw),-radius*cos(yaw), this);
		QPen pen = _line->pen();
		pen.setWidth(linkWidth*100.0f);
		_line->setPen(pen);
	}
	virtual ~NodeItem() {}

	void setColor(const QColor & color)
	{
		QPen p = this->pen();
		p.setColor(color);
		this->setPen(p);
		QBrush b = this->brush();
		b.setColor(color);
		this->setBrush(b);

		QPen pen = _line->pen();
		pen.setColor(QColor(255-color.red(), 255-color.green(), 255-color.blue()));
		_line->setPen(pen);
	}

	void setRadius(float radius)
	{
		float r,p,yaw;
		_pose.getEulerAngles(r, p, yaw);
		radius*=100.0f;
		this->setRect(-radius, -radius, radius*2.0f, radius*2.0f);
		_line->setLine(0,0,-radius*sin(yaw),-radius*cos(yaw));
	}

	int id() const {return _id;};
	int mapId() const {return _mapId;}
	const Transform & pose() const {return _pose;}
	void setPose(const Transform & pose, GraphViewer::ViewPlane plane) {
		switch(plane)
		{
		case GraphViewer::XZ:
			this->setPos(pose.x()*100.0f,-pose.z()*100.0f);
			break;
		case GraphViewer::YZ:
			this->setPos(pose.y()*100.0f,-pose.z()*100.0f);
			break;
		default: // XY
			this->setPos(-pose.y()*100.0f,-pose.x()*100.0f);
			break;
		}
		_pose=pose;
		float r,p,yaw;
		_pose.getEulerAngles(r, p, yaw);
		if(_line)
		{
			float radius = this->rect().width()/2.0f;
			_line->setLine(0,0,-radius*sin(yaw),-radius*cos(yaw));
		}
	}

protected:
	virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
	{
		if(_weight>=0)
		{
			this->setToolTip(QString("%1 [map=%2, w=%3] %4").arg(_id).arg(_mapId).arg(_weight).arg(_pose.prettyPrint().c_str()));
		}
		else
		{
			this->setToolTip(QString("%1 [map=%2] %3").arg(_id).arg(_mapId).arg(_pose.prettyPrint().c_str()));
		}
		this->setScale(2);
		QGraphicsEllipseItem::hoverEnterEvent(event);
	}

	virtual void hoverLeaveEvent ( QGraphicsSceneHoverEvent * event )
	{
		this->setScale(1);
		QGraphicsEllipseItem::hoverEnterEvent(event);
	}

private:
	int _id;
	int _mapId;
	int _weight;
	Transform _pose;
	QGraphicsLineItem * _line;
};

class NodeGPSItem: public NodeItem
{
public:
	NodeGPSItem(int id, int mapId, const Transform & pose, float radius, const GPS & gps, GraphViewer::ViewPlane plane, float linkWidth) :
		NodeItem(id, mapId, pose, radius, -1, plane, linkWidth),
		_gps(gps)
	{
	}
	virtual ~NodeGPSItem() {}
protected:
	virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
	{
		this->setToolTip(QString("%1 [%2] %3\n"
				"longitude=%4 latitude=%5 altitude=%6m error=%7m bearing=%8deg")
				.arg(id()).arg(mapId()).arg(pose().prettyPrint().c_str())
				.arg(_gps.longitude()).arg(_gps.latitude()).arg(_gps.altitude()).arg(_gps.error()).arg(_gps.bearing()));
		this->setScale(2);
		QGraphicsEllipseItem::hoverEnterEvent(event);
	}
private:
	GPS _gps;
};

class LinkItem: public QGraphicsLineItem
{
public:
	// in meter
	LinkItem(int from, int to, const Transform & poseA, const Transform & poseB, const Link & link, bool interSessionClosure, GraphViewer::ViewPlane plane) :
		_from(from),
		_to(to),
		_poseA(poseA),
		_poseB(poseB),
		_link(link),
		_interSession(interSessionClosure)
	{
		this->setPoses(poseA, poseB, plane);
		this->setAcceptHoverEvents(true);
	}
	virtual ~LinkItem() {}

	void setColor(const QColor & color)
	{
		QPen p = this->pen();
		p.setColor(color);
		this->setPen(p);
	}

	void setPoses(const Transform & poseA, const Transform & poseB, GraphViewer::ViewPlane plane)
	{
		switch(plane)
		{
		case GraphViewer::XZ:
			this->setLine(poseA.x()*100.0f, -poseA.z()*100.0f, poseB.x()*100.0f, -poseB.z()*100.0f);
			break;
		case GraphViewer::YZ:
			this->setLine(poseA.y()*100.0f, -poseA.z()*100.0f, poseB.y()*100.0f, -poseB.z()*100.0f);
			break;
		default: // XY
			this->setLine(-poseA.y()*100.0f, -poseA.x()*100.0f, -poseB.y()*100.0f, -poseB.x()*100.0f);
			break;
		}
		_poseA = poseA;
		_poseB = poseB;
	}

	const Transform & getPoseA() const
	{
		return _poseA;
	}
	const Transform & getPoseB() const
	{
		return _poseB;
	}

	Link::Type linkType() const {return _link.type();}
	bool isInterSession() const {return _interSession;}
	int from() const {return _from;}
	int to() const {return _to;}

protected:
	virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
	{
		QString str = QString("%1->%2 (%3 m)").arg(_from).arg(_to).arg(_poseA.getDistance(_poseB));
		if(!_link.transform().isNull())
		{
			str.append(QString("\n%1\n%2 %3").arg(_link.transform().prettyPrint().c_str()).arg(_link.transVariance()).arg(_link.rotVariance()));
		}
		this->setToolTip(str);
		QPen pen = this->pen();
		pen.setWidthF(pen.widthF()+2);
		this->setPen(pen);
		QGraphicsLineItem::hoverEnterEvent(event);
	}

	virtual void hoverLeaveEvent ( QGraphicsSceneHoverEvent * event )
	{
		QPen pen = this->pen();
		pen.setWidthF(pen.widthF()-2);
		this->setPen(pen);
		QGraphicsLineItem::hoverEnterEvent(event);
	}

private:
	int _from;
	int _to;
	Transform _poseA;
	Transform _poseB;
	Link _link;
	bool _interSession;
};

GraphViewer::GraphViewer(QWidget * parent) :
		QGraphicsView(parent),
		_nodeColor(Qt::blue),
		_nodeOdomCacheColor(Qt::darkGreen),
		_currentGoalColor(Qt::darkMagenta),
		_neighborColor(Qt::blue),
		_loopClosureColor(Qt::red),
		_loopClosureLocalColor(Qt::yellow),
		_loopClosureUserColor(Qt::red),
		_loopClosureVirtualColor(Qt::magenta),
		_neighborMergedColor(QColor(255,170,0)),
		_landmarkColor(Qt::darkGreen),
		_loopClosureRejectedColor(Qt::black),
		_localPathColor(Qt::cyan),
		_globalPathColor(Qt::darkMagenta),
		_gtPathColor(Qt::gray),
		_gpsPathColor(Qt::darkCyan),
		_loopIntraSessionColor(Qt::red),
		_loopInterSessionColor(Qt::green),
		_intraInterSessionColors(false),
		_worldMapRotation(0.0f),
		_world(0),
		_root(0),
		_graphRoot(0),
		_globalPathRoot(0),
		_nodeVisible(true),
		_nodeRadius(0.01f),
		_linkWidth(0),
		_gridMap(0),
		_referential(0),
		_originReferential(0),
		_gridCellSize(0.0f),
		_localRadius(0),
		_loopClosureOutlierThr(0),
		_maxLinkLength(0.02f),
		_orientationENU(false),
		_viewPlane(XY),
		_ensureFrameVisible(true)
{
	this->setScene(new QGraphicsScene(this));
	this->setDragMode(QGraphicsView::ScrollHandDrag);
	_workingDirectory = QDir::homePath();

	this->scene()->clear();
	_world = (QGraphicsItem *)this->scene()->addEllipse(QRectF(-0.0001,-0.0001,0.0001,0.0001));
	_root = (QGraphicsItem *)this->scene()->addEllipse(QRectF(-0.0001,-0.0001,0.0001,0.0001));
	_root->setParentItem(_world);

	// add referential
	QGraphicsLineItem * item;
	_originReferential = new QGraphicsItemGroup();
	this->scene()->addItem(_originReferential); // ownership transfered

	// XY referential
	_originReferentialXY = new QGraphicsItemGroup();
	this->scene()->addItem(_originReferentialXY); // ownership transfered
	item = this->scene()->addLine(0,0,0,-100, QPen(QBrush(Qt::red), _linkWidth));
	item->setZValue(1);
	item->setParentItem(_root);
	_originReferentialXY->addToGroup(item);
	item = this->scene()->addLine(0,0,-100,0, QPen(QBrush(Qt::green), _linkWidth));
	item->setZValue(1);
	item->setParentItem(_root);
	_originReferentialXY->addToGroup(item);
	_originReferential->addToGroup(_originReferentialXY);

	// XZ referential
	_originReferentialXZ = new QGraphicsItemGroup();
	this->scene()->addItem(_originReferentialXZ); // ownership transfered
	item = this->scene()->addLine(0,0,100,0, QPen(QBrush(Qt::red), _linkWidth));
	item->setZValue(1);
	item->setParentItem(_root);
	_originReferentialXZ->addToGroup(item);
	item = this->scene()->addLine(0,0,0,-100, QPen(QBrush(Qt::blue), _linkWidth));
	item->setZValue(1);
	item->setParentItem(_root);
	_originReferentialXZ->addToGroup(item);
	_originReferential->addToGroup(_originReferentialXZ);
	_originReferentialXZ->setVisible(false);

	// YZ referential
	_originReferentialYZ = new QGraphicsItemGroup();
	this->scene()->addItem(_originReferentialYZ); // ownership transfered
	item = this->scene()->addLine(0,0,100,0, QPen(QBrush(Qt::green), _linkWidth));
	item->setZValue(1);
	item->setParentItem(_root);
	_originReferentialYZ->addToGroup(item);
	item = this->scene()->addLine(0,0,0,-100, QPen(QBrush(Qt::blue), _linkWidth));
	item->setZValue(1);
	item->setParentItem(_root);
	_originReferentialYZ->addToGroup(item);
	_originReferential->addToGroup(_originReferentialYZ);
	_originReferentialYZ->setVisible(false);

	_originReferential->setZValue(1);
	_originReferential->setParentItem(_root);

	// current pose
	_referential = new QGraphicsItemGroup();
	this->scene()->addItem(_referential); // ownership transfered

	// XY
	_referentialXY = new QGraphicsItemGroup();
	this->scene()->addItem(_referentialXY); // ownership transfered
	item = this->scene()->addLine(0,0,0,-50, QPen(QBrush(Qt::red), _linkWidth));
	item->setZValue(100);
	item->setParentItem(_root);
	_referentialXY->addToGroup(item);
	item = this->scene()->addLine(0,0,-50,0, QPen(QBrush(Qt::green), _linkWidth));
	item->setZValue(100);
	item->setParentItem(_root);
	_referentialXY->addToGroup(item);
	_referential->addToGroup(_referentialXY);

	// XZ
	_referentialXZ = new QGraphicsItemGroup();
	this->scene()->addItem(_referentialXZ); // ownership transfered
	item = this->scene()->addLine(0,0,50,0, QPen(QBrush(Qt::red), _linkWidth));
	item->setZValue(100);
	item->setParentItem(_root);
	_referentialXZ->addToGroup(item);
	item = this->scene()->addLine(0,0,0,-50, QPen(QBrush(Qt::blue), _linkWidth));
	item->setZValue(100);
	item->setParentItem(_root);
	_referentialXZ->addToGroup(item);
	_referential->addToGroup(_referentialXZ);
	_referentialXZ->setVisible(false);

	// XZ
	_referentialYZ = new QGraphicsItemGroup();
	this->scene()->addItem(_referentialYZ); // ownership transfered
	item = this->scene()->addLine(0,0,50,0, QPen(QBrush(Qt::green), _linkWidth));
	item->setZValue(100);
	item->setParentItem(_root);
	_referentialYZ->addToGroup(item);
	item = this->scene()->addLine(0,0,0,-50, QPen(QBrush(Qt::blue), _linkWidth));
	item->setZValue(100);
	item->setParentItem(_root);
	_referentialYZ->addToGroup(item);
	_referential->addToGroup(_referentialYZ);
	_referentialYZ->setVisible(false);

	_referential->setZValue(100);
	_referential->setParentItem(_root);

	_localRadius = this->scene()->addEllipse(-0.0001,-0.0001,0.0001,0.0001);
	_localRadius->setZValue(1);
	_localRadius->setParentItem(_root);
	_localRadius->setVisible(false);
	_localRadius->setPen(QPen(Qt::DashLine));

	_gridMap = this->scene()->addPixmap(QPixmap());
	_gridMap->setZValue(0);
	_gridMap->setParentItem(_root);

	_graphRoot = (QGraphicsItem *)this->scene()->addEllipse(QRectF(-0.0001,-0.0001,0.0001,0.0001));
	_graphRoot->setZValue(4);
	_graphRoot->setParentItem(_root);

	_globalPathRoot = (QGraphicsItem *)this->scene()->addEllipse(QRectF(-0.0001,-0.0001,0.0001,0.0001));
	_globalPathRoot->setZValue(8);
	_globalPathRoot->setParentItem(_root);

	_localPathRoot = (QGraphicsItem *)this->scene()->addEllipse(QRectF(-0.0001,-0.0001,0.0001,0.0001));
	_localPathRoot->setZValue(9);
	_localPathRoot->setParentItem(_root);

	_gtGraphRoot = (QGraphicsItem *)this->scene()->addEllipse(QRectF(-0.0001,-0.0001,0.0001,0.0001));
	_gtGraphRoot->setZValue(2);
	_gtGraphRoot->setParentItem(_root);

	_gpsGraphRoot = (QGraphicsItem *)this->scene()->addEllipse(QRectF(-0.0001,-0.0001,0.0001,0.0001));
	_gpsGraphRoot->setZValue(3);
	_gpsGraphRoot->setParentItem(_root);

	_odomCacheOverlay = this->scene()->addRect(0,0,0,0);
	_odomCacheOverlay->setZValue(21); // just under odom cache nodes and links
	_odomCacheOverlay->setParentItem(_graphRoot);
	_odomCacheOverlay->setBrush(QBrush(QColor(255, 255, 255, 150)));
	_odomCacheOverlay->setPen(QPen(Qt::NoPen));

	this->restoreDefaults();

	this->fitInView(this->sceneRect(), Qt::KeepAspectRatio);
}

GraphViewer::~GraphViewer()
{
}

void GraphViewer::setWorldMapRotation(const float & theta)
{
	_worldMapRotation = theta;
	setOrientationENU(isOrientationENU());
}

void GraphViewer::updateGraph(const std::map<int, Transform> & poses,
				 const std::multimap<int, Link> & constraints,
				 const std::map<int, int> & mapIds,
				 const std::map<int, int> & weights,
				 const std::set<int> & odomCacheIds)
{
	UTimer timer;
	bool wasVisible = _graphRoot->isVisible();
	_graphRoot->show();

	bool wasEmpty = _nodeItems.size() == 0 && _linkItems.size() == 0;
	UDEBUG("poses=%d constraints=%d", (int)poses.size(), (int)constraints.size());
	//Hide nodes and links
	for(QMap<int, NodeItem*>::iterator iter = _nodeItems.begin(); iter!=_nodeItems.end(); ++iter)
	{
		QColor color = _nodeColor;
		bool isOdomCache = odomCacheIds.find(iter.key()) != odomCacheIds.end();
		if(iter.key()<0)
		{
			color = QColor(255-color.red(), 255-color.green(), 255-color.blue());
		}
		else if(isOdomCache)
		{
			color = _nodeOdomCacheColor;
		}
		iter.value()->hide();
		iter.value()->setColor(color); // reset color
		iter.value()->setZValue(iter.key()<0?21:20);
	}
	for(QMultiMap<int, LinkItem*>::iterator iter = _linkItems.begin(); iter!=_linkItems.end(); ++iter)
	{
		iter.value()->hide();
	}

	for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		if(!iter->second.isNull())
		{
			QMap<int, NodeItem*>::iterator itemIter = _nodeItems.find(iter->first);
			if(itemIter != _nodeItems.end())
			{
				itemIter.value()->setPose(iter->second, _viewPlane);
				itemIter.value()->show();
			}
			else
			{
				// create node item
				QColor color = _nodeColor;
				bool isOdomCache = odomCacheIds.find(iter->first) != odomCacheIds.end();
				if(iter->first<0)
				{
					color = QColor(255-color.red(), 255-color.green(), 255-color.blue());
				}
				else if(isOdomCache)
				{
					color = _nodeOdomCacheColor;
				}
				const Transform & pose = iter->second;
				NodeItem * item = new NodeItem(iter->first, uContains(mapIds, iter->first)?mapIds.at(iter->first):-1, pose, _nodeRadius, uContains(weights, iter->first)?weights.at(iter->first):-1, _viewPlane, _linkWidth);
				this->scene()->addItem(item);
				item->setZValue(iter->first<0?21:20);
				item->setColor(color);
				item->setParentItem(_graphRoot);
				item->setVisible(_nodeVisible);
				_nodeItems.insert(iter->first, item);
			}
		}
	}

	for(std::multimap<int, Link>::const_iterator iter=constraints.begin(); iter!=constraints.end(); ++iter)
	{
		// make the first id the smallest one
		int idFrom = iter->first<iter->second.to()?iter->first:iter->second.to();
		int idTo = iter->first<iter->second.to()?iter->second.to():iter->first;

		std::map<int, Transform>::const_iterator jterA = poses.find(idFrom);
		std::map<int, Transform>::const_iterator jterB = poses.find(idTo);
		LinkItem * linkItem = 0;
		if(jterA != poses.end() && jterB != poses.end() &&
		   _nodeItems.contains(iter->first) && _nodeItems.contains(idTo))
		{
			const Transform & poseA = jterA->second;
			const Transform & poseB = jterB->second;

			QMultiMap<int, LinkItem*>::iterator itemIter = _linkItems.end();
			if(_linkItems.contains(idFrom))
			{
				itemIter = _linkItems.find(iter->first);
				while(itemIter.key() == idFrom && itemIter != _linkItems.end())
				{
					if(itemIter.value()->to() == idTo)
					{
						itemIter.value()->setPoses(poseA, poseB, _viewPlane);
						itemIter.value()->show();
						linkItem = itemIter.value();
						break;
					}
					++itemIter;
				}
			}

			bool interSessionClosure = false;
			if(uContains(mapIds, jterA->first) && uContains(mapIds, jterB->first))
			{
				interSessionClosure = mapIds.at(jterA->first) != mapIds.at(jterB->first);
			}

			bool isLinkedToOdomCachePoses =
					odomCacheIds.find(idFrom)!=odomCacheIds.end() ||
					odomCacheIds.find(idTo)!=odomCacheIds.end();

			if(isLinkedToOdomCachePoses)
			{
				_nodeItems.value(idFrom)->setZValue(odomCacheIds.find(idFrom)!=odomCacheIds.end()?24:23);
				_nodeItems.value(idTo)->setZValue(odomCacheIds.find(idTo)!=odomCacheIds.end()?24:23);
			}

			if(poseA.getDistance(poseB) > _maxLinkLength)
			{
				if(linkItem == 0)
				{
					//create a link item
					linkItem = new LinkItem(idFrom, idTo, poseA, poseB, iter->second, interSessionClosure, _viewPlane);
					QPen p = linkItem->pen();
					p.setWidthF(_linkWidth*100.0f);
					linkItem->setPen(p);
					linkItem->setZValue(isLinkedToOdomCachePoses?22:10);
					this->scene()->addItem(linkItem);
					linkItem->setParentItem(_graphRoot);
					_linkItems.insert(idFrom, linkItem);
				}
			}
			else if(linkItem && itemIter != _linkItems.end())
			{
				// erase small links
				_linkItems.erase(itemIter);
				delete linkItem;
				linkItem = 0;
			}

			if(linkItem)
			{
				//update color
				if(iter->second.type() == Link::kNeighbor)
				{
					linkItem->setColor(_neighborColor);
				}
				else if(iter->second.type() == Link::kVirtualClosure)
				{
					linkItem->setColor(_loopClosureVirtualColor);
				}
				else if(iter->second.type() == Link::kNeighborMerged)
				{
					linkItem->setColor(_neighborMergedColor);
				}
				else if(iter->second.type() == Link::kUserClosure)
				{
					if(_intraInterSessionColors)
					{
						linkItem->setColor(interSessionClosure?_loopInterSessionColor:_loopIntraSessionColor);
					}
					else
					{
						linkItem->setColor(_loopClosureUserColor);
					}
				}
				else if(iter->second.type() == Link::kLandmark)
				{
					linkItem->setColor(_landmarkColor);
				}
				else if(iter->second.type() == Link::kLocalSpaceClosure || iter->second.type() == Link::kLocalTimeClosure)
				{
					if(_intraInterSessionColors)
					{
						linkItem->setColor(interSessionClosure?_loopInterSessionColor:_loopIntraSessionColor);
						linkItem->setZValue(isLinkedToOdomCachePoses?22:interSessionClosure?6:7);
					}
					else
					{
						linkItem->setColor(_loopClosureLocalColor);
						linkItem->setZValue(isLinkedToOdomCachePoses?22:7);
					}
				}
				else
				{
					if(_intraInterSessionColors)
					{
						linkItem->setColor(interSessionClosure?_loopInterSessionColor:_loopIntraSessionColor);
						linkItem->setZValue(isLinkedToOdomCachePoses?22:interSessionClosure?8:9);
					}
					else
					{
						linkItem->setColor(_loopClosureColor);
						linkItem->setZValue(isLinkedToOdomCachePoses?22:9);
					}
				}

				//rejected loop closures
				if(_loopClosureOutlierThr > 0.0f)
				{
					Transform t = poseA.inverse()*poseB;
					if(iter->second.to() != idTo)
					{
						t = t.inverse();
					}
					if(iter->second.type() != Link::kNeighbor &&
					   iter->second.type() != Link::kNeighborMerged)
					{
						float linearError = fabs(iter->second.transform().getNorm() - t.getNorm());
						if(linearError > _loopClosureOutlierThr)
						{
							linkItem->setColor(_loopClosureRejectedColor);
						}
					}
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
	for(QMultiMap<int, LinkItem*>::iterator iter = _linkItems.begin(); iter!=_linkItems.end();)
	{
		if(!iter.value()->isVisible())
		{
			delete iter.value();
			iter = _linkItems.erase(iter);
		}
		else
		{
			++iter;
		}
	}

	if(_nodeItems.size())
	{
		(--_nodeItems.end()).value()->setColor(_nodeOdomCacheColor);
	}

	this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents

	if(!odomCacheIds.empty())
		_odomCacheOverlay->setRect(this->scene()->itemsBoundingRect());
	else
		_odomCacheOverlay->setRect(0, 0, 0, 0);

	if(wasEmpty)
	{
		QRectF rect = this->scene()->itemsBoundingRect();
		this->fitInView(rect.adjusted(-rect.width()/2.0f, -rect.height()/2.0f, rect.width()/2.0f, rect.height()/2.0f), Qt::KeepAspectRatio);
	}

	_graphRoot->setVisible(wasVisible);

	UDEBUG("_nodeItems=%d, _linkItems=%d, timer=%fs", _nodeItems.size(), _linkItems.size(), timer.ticks());
}

void GraphViewer::updateGTGraph(const std::map<int, Transform> & poses)
{
	UTimer timer;
	bool wasVisible = _gtGraphRoot->isVisible();
	_gtGraphRoot->show();
	bool wasEmpty = _gtNodeItems.size() == 0 && _gtLinkItems.size() == 0;
	UDEBUG("poses=%d", (int)poses.size());
	//Hide nodes and links
	for(QMap<int, NodeItem*>::iterator iter = _gtNodeItems.begin(); iter!=_gtNodeItems.end(); ++iter)
	{
		iter.value()->hide();
		iter.value()->setColor(_gtPathColor); // reset color
	}
	for(QMultiMap<int, LinkItem*>::iterator iter = _gtLinkItems.begin(); iter!=_gtLinkItems.end(); ++iter)
	{
		iter.value()->hide();
	}

	for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		if(!iter->second.isNull())
		{
			QMap<int, NodeItem*>::iterator itemIter = _gtNodeItems.find(iter->first);
			if(itemIter != _gtNodeItems.end())
			{
				itemIter.value()->setPose(iter->second, _viewPlane);
				itemIter.value()->show();
			}
			else
			{
				// create node item
				const Transform & pose = iter->second;
				NodeItem * item = new NodeItem(iter->first, -1, pose, _nodeRadius, -1, _viewPlane, _linkWidth);
				this->scene()->addItem(item);
				item->setZValue(20);
				item->setColor(_gtPathColor);
				item->setParentItem(_gtGraphRoot);
				item->setVisible(_nodeVisible);
				_gtNodeItems.insert(iter->first, item);
			}

			if(iter!=poses.begin())
			{
				std::map<int, Transform>::const_iterator iterPrevious = iter;
				--iterPrevious;
				Transform previousPose = iterPrevious->second;
				Transform currentPose = iter->second;

				LinkItem * linkItem = 0;
				QMultiMap<int, LinkItem*>::iterator linkIter = _gtLinkItems.end();
				if(_gtLinkItems.contains(iterPrevious->first))
				{
					linkIter = _gtLinkItems.find(iter->first);
					while(linkIter.key() == iterPrevious->first && linkIter != _gtLinkItems.end())
					{
						if(linkIter.value()->to() == iter->first)
						{
							linkIter.value()->setPoses(previousPose, currentPose, _viewPlane);
							linkIter.value()->show();
							linkItem = linkIter.value();
							break;
						}
						++linkIter;
					}
				}
				if(linkItem == 0)
				{
					bool linkFound = iter->first - iterPrevious->first == 1; // if consecutive, add link
					for(QMultiMap<int, LinkItem*>::iterator kter = _linkItems.find(iterPrevious->first);
							kter!=_linkItems.end() && kter.key()==iterPrevious->first && !linkFound;
							++kter)
					{
						if(kter.value()->from() == iterPrevious->first && kter.value()->to() == iter->first)
						{
							linkFound = true;
						}
					}

					if(linkFound)
					{
						//create a link item
						linkItem = new LinkItem(iterPrevious->first, iter->first, previousPose, currentPose, Link(), 1, _viewPlane);
						QPen p = linkItem->pen();
						p.setWidthF(_linkWidth*100.0f);
						linkItem->setPen(p);
						linkItem->setZValue(10);
						this->scene()->addItem(linkItem);
						linkItem->setParentItem(_gtGraphRoot);
						_gtLinkItems.insert(iterPrevious->first, linkItem);
					}
				}
				if(linkItem)
				{
					linkItem->setColor(_gtPathColor);
				}
			}
		}
	}

	//remove not used nodes and links
	for(QMap<int, NodeItem*>::iterator iter = _gtNodeItems.begin(); iter!=_gtNodeItems.end();)
	{
		if(!iter.value()->isVisible())
		{
			delete iter.value();
			iter = _gtNodeItems.erase(iter);
		}
		else
		{
			++iter;
		}
	}
	for(QMultiMap<int, LinkItem*>::iterator iter = _gtLinkItems.begin(); iter!=_gtLinkItems.end();)
	{
		if(!iter.value()->isVisible())
		{
			delete iter.value();
			iter = _gtLinkItems.erase(iter);
		}
		else
		{
			++iter;
		}
	}

	if(_gtNodeItems.size() || _gtLinkItems.size())
	{
		this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents

		if(wasEmpty)
		{
			QRectF rect = this->scene()->itemsBoundingRect();
			this->fitInView(rect.adjusted(-rect.width()/2.0f, -rect.height()/2.0f, rect.width()/2.0f, rect.height()/2.0f), Qt::KeepAspectRatio);
		}
	}

	_gtGraphRoot->setVisible(wasVisible);

	UDEBUG("_gtNodeItems=%d, _gtLinkItems=%d timer=%fs", _gtNodeItems.size(), _gtLinkItems.size(), timer.ticks());
}

void GraphViewer::updateGPSGraph(
		const std::map<int, Transform> & poses,
		const std::map<int, GPS> & gpsValues)
{
	UTimer timer;
	bool wasVisible = _gpsGraphRoot->isVisible();
	_gpsGraphRoot->show();
	bool wasEmpty = _gpsNodeItems.size() == 0 && _gpsNodeItems.size() == 0;
	UDEBUG("poses=%d", (int)poses.size());
	//Hide nodes and links
	for(QMap<int, NodeItem*>::iterator iter = _gpsNodeItems.begin(); iter!=_gpsNodeItems.end(); ++iter)
	{
		iter.value()->hide();
		iter.value()->setColor(_gpsPathColor); // reset color
	}
	for(QMultiMap<int, LinkItem*>::iterator iter = _gpsLinkItems.begin(); iter!=_gpsLinkItems.end(); ++iter)
	{
		iter.value()->hide();
	}

	for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		if(!iter->second.isNull())
		{
			QMap<int, NodeItem*>::iterator itemIter = _gpsNodeItems.find(iter->first);
			if(itemIter != _gpsNodeItems.end())
			{
				itemIter.value()->setPose(iter->second, _viewPlane);
				itemIter.value()->show();
			}
			else
			{
				// create node item
				const Transform & pose = iter->second;
				UASSERT(gpsValues.find(iter->first) != gpsValues.end());
				NodeItem * item = new NodeGPSItem(iter->first, -1, pose, _nodeRadius, gpsValues.at(iter->first), _viewPlane, _linkWidth);
				this->scene()->addItem(item);
				item->setZValue(20);
				item->setColor(_gpsPathColor);
				item->setParentItem(_gpsGraphRoot);
				item->setVisible(_nodeVisible);
				_gpsNodeItems.insert(iter->first, item);
			}

			if(iter!=poses.begin())
			{
				std::map<int, Transform>::const_iterator iterPrevious = iter;
				--iterPrevious;
				Transform previousPose = iterPrevious->second;
				Transform currentPose = iter->second;

				LinkItem * linkItem = 0;
				QMultiMap<int, LinkItem*>::iterator linkIter = _gpsLinkItems.end();
				if(_gpsLinkItems.contains(iterPrevious->first))
				{
					linkIter = _gpsLinkItems.find(iter->first);
					while(linkIter.key() == iterPrevious->first && linkIter != _gpsLinkItems.end())
					{
						if(linkIter.value()->to() == iter->first)
						{
							linkIter.value()->setPoses(previousPose, currentPose, _viewPlane);
							linkIter.value()->show();
							linkItem = linkIter.value();
							break;
						}
						++linkIter;
					}
				}
				if(linkItem == 0)
				{
					//create a link item
					linkItem = new LinkItem(iterPrevious->first, iter->first, previousPose, currentPose, Link(), 1, _viewPlane);
					QPen p = linkItem->pen();
					p.setWidthF(_linkWidth*100.0f);
					linkItem->setPen(p);
					linkItem->setZValue(10);
					this->scene()->addItem(linkItem);
					linkItem->setParentItem(_gpsGraphRoot);
					_gpsLinkItems.insert(iterPrevious->first, linkItem);
				}
				if(linkItem)
				{
					linkItem->setColor(_gpsPathColor);
				}
			}
		}
	}

	//remove not used nodes and links
	for(QMap<int, NodeItem*>::iterator iter = _gpsNodeItems.begin(); iter!=_gpsNodeItems.end();)
	{
		if(!iter.value()->isVisible())
		{
			delete iter.value();
			iter = _gpsNodeItems.erase(iter);
		}
		else
		{
			++iter;
		}
	}
	for(QMultiMap<int, LinkItem*>::iterator iter = _gpsLinkItems.begin(); iter!=_gpsLinkItems.end();)
	{
		if(!iter.value()->isVisible())
		{
			delete iter.value();
			iter = _gpsLinkItems.erase(iter);
		}
		else
		{
			++iter;
		}
	}

	if(_gpsNodeItems.size() || _gpsLinkItems.size())
	{
		this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents

		if(wasEmpty)
		{
			QRectF rect = this->scene()->itemsBoundingRect();
			this->fitInView(rect.adjusted(-rect.width()/2.0f, -rect.height()/2.0f, rect.width()/2.0f, rect.height()/2.0f), Qt::KeepAspectRatio);
		}
	}

	_gpsGraphRoot->setVisible(wasVisible);

	UDEBUG("_gpsNodeItems=%d, _gpsLinkItems=%d timer=%fs", _gpsNodeItems.size(), _gpsLinkItems.size(), timer.ticks());
}

void GraphViewer::updateReferentialPosition(const Transform & t)
{
	QTransform qt;
	qt.translate(-t.o24()*100.0f, -t.o14()*100.0f);
	if(_viewPlane == XY)
		qt.rotateRadians(-t.theta());

	_referential->setTransform(qt);
	_localRadius->setTransform(qt);

	if(_ensureFrameVisible)
	{
		this->ensureVisible(_referential);
		if(_localRadius->isVisible())
		{
			this->ensureVisible(_localRadius, 0, 0);
		}
	}
}

void GraphViewer::updateMap(const cv::Mat & map8U, float resolution, float xMin, float yMin)
{
	UASSERT(map8U.empty() || (!map8U.empty() && resolution > 0.0f));
	if(!map8U.empty())
	{
		_gridCellSize = resolution;
		QImage image = uCvMat2QImage(map8U, false);
		_gridMap->resetTransform();
		_gridMap->setTransform(QTransform::fromScale(resolution*100.0f, -resolution*100.0f), true);
		_gridMap->setRotation(90);
		_gridMap->setPixmap(QPixmap::fromImage(image));
		_gridMap->setPos(-yMin*100.0f, -xMin*100.0f);
		// Re-shrink the scene to it's bounding contents
		this->scene()->setSceneRect(this->scene()->itemsBoundingRect());
	}
	else
	{
		this->clearMap();
	}
}

void GraphViewer::updatePosterior(const std::map<int, float> & posterior, float max, int zValueOffset)
{
	//find max
	if(max <= 0.0f)
	{
		for(std::map<int, float>::const_iterator iter = posterior.begin(); iter!=posterior.end(); ++iter)
		{
			if(iter->first > 0 && iter->second>max)
			{
				max = iter->second;
			}
		}
	}
	if(max > 0.0f)
	{
		for(QMap<int, NodeItem*>::iterator iter = _nodeItems.begin(); iter!=_nodeItems.end(); ++iter)
		{
			std::map<int,float>::const_iterator jter = posterior.find(iter.key());
			if(jter != posterior.end())
			{
				float v = jter->second>max?max:jter->second;
				iter.value()->setColor(QColor::fromHsvF((1-v/max)*240.0f/360.0f, 1, 1, 1)); //0=red 240=blue
				iter.value()->setZValue(iter.value()->zValue()+zValueOffset);
			}
		}
	}
}

void GraphViewer::setGlobalPath(const std::vector<std::pair<int, Transform> > & globalPath)
{
	UDEBUG("Set global path size=%d", (int)globalPath.size());
	qDeleteAll(_globalPathLinkItems);
	_globalPathLinkItems.clear();

	if(globalPath.size() >= 2)
	{
		for(unsigned int i=0; i<globalPath.size()-1; ++i)
		{
			//create a link item
			int idFrom = globalPath[i].first;
			int idTo = globalPath[i+1].first;
			LinkItem * item = new LinkItem(idFrom, idTo, globalPath[i].second, globalPath[i+1].second, Link(), false, _viewPlane);
			QPen p = item->pen();
			p.setWidthF(_linkWidth*100.0f);
			item->setPen(p);
			item->setColor(_globalPathColor);
			this->scene()->addItem(item);
			item->setZValue(15);
			item->setParentItem(_globalPathRoot);
			_globalPathLinkItems.insert(idFrom, item);
		}
	}
}

void GraphViewer::setCurrentGoalID(int id, const Transform & pose)
{
	NodeItem * node = _nodeItems.value(id, 0);
	if(node)
	{
		node->setColor(_currentGoalColor);
	}
	else
	{
		UWARN("Current goal %d not found in the graph", id);
	}

	if(!pose.isNull() && _globalPathLinkItems.size() && _globalPathLinkItems.contains(id))
	{
		// transform the global path in the goal referential
		const LinkItem * oldPose = _globalPathLinkItems.value(id);
		Transform t = pose * oldPose->getPoseA().inverse();
		for(QMultiMap<int, LinkItem*>::iterator iter=_globalPathLinkItems.begin(); iter!=_globalPathLinkItems.end(); ++iter)
		{
			iter.value()->setPoses(t*iter.value()->getPoseA(), t*iter.value()->getPoseB(), _viewPlane);
		}
	}
}

void GraphViewer::setLocalRadius(float radius)
{
	_localRadius->setRect(-radius*100, -radius*100, radius*200, radius*200);
}

void GraphViewer::updateLocalPath(const std::vector<int> & localPath)
{
	bool wasVisible = _localPathRoot->isVisible();
	_localPathRoot->show();

	for(QMultiMap<int, LinkItem*>::iterator iter = _localPathLinkItems.begin(); iter!=_localPathLinkItems.end(); ++iter)
	{
		iter.value()->hide();
	}

	if(localPath.size() > 1)
	{
		for(unsigned int i=0; i<localPath.size()-1; ++i)
		{
			int idFrom = localPath[i]<localPath[i+1]?localPath[i]:localPath[i+1];
			int idTo = localPath[i]<localPath[i+1]?localPath[i+1]:localPath[i];
			if(_nodeItems.contains(idFrom) && _nodeItems.contains(idTo))
			{
				bool updated = false;
				if(_localPathLinkItems.contains(idFrom))
				{
					QMultiMap<int, LinkItem*>::iterator itemIter = _localPathLinkItems.find(idFrom);
					while(itemIter.key() == idFrom && itemIter != _localPathLinkItems.end())
					{
						if(itemIter.value()->to() == idTo)
						{
							itemIter.value()->setPoses(_nodeItems.value(idFrom)->pose(), _nodeItems.value(idTo)->pose(), _viewPlane);
							itemIter.value()->show();
							updated = true;
							break;
						}
						++itemIter;
					}
				}
				if(!updated)
				{
					//create a link item
					LinkItem * item = new LinkItem(idFrom, idTo, _nodeItems.value(idFrom)->pose(), _nodeItems.value(idTo)->pose(), Link(), false, _viewPlane);
					QPen p = item->pen();
					p.setWidthF(_linkWidth*100.0f);
					item->setPen(p);
					item->setColor(_localPathColor);
					this->scene()->addItem(item);
					item->setZValue(16); // just over the global path
					item->setParentItem(_localPathRoot);
					_localPathLinkItems.insert(idFrom, item);
				}
			}
		}
	}

	// remove not used links
	for(QMultiMap<int, LinkItem*>::iterator iter = _localPathLinkItems.begin(); iter!=_localPathLinkItems.end();)
	{
		if(!iter.value()->isVisible())
		{
			delete iter.value();
			iter = _localPathLinkItems.erase(iter);
		}
		else
		{
			++iter;
		}
	}
	_localPathRoot->setVisible(wasVisible);
}

void GraphViewer::clearGraph()
{
	qDeleteAll(_nodeItems);
	_nodeItems.clear();
	qDeleteAll(_linkItems);
	_linkItems.clear();
	qDeleteAll(_localPathLinkItems);
	_localPathLinkItems.clear();
	qDeleteAll(_globalPathLinkItems);
	_globalPathLinkItems.clear();
	qDeleteAll(_gtNodeItems);
	_gtNodeItems.clear();
	qDeleteAll(_gtLinkItems);
	_gtLinkItems.clear();
	qDeleteAll(_gpsNodeItems);
	_gpsNodeItems.clear();
	qDeleteAll(_gpsLinkItems);
	_gpsLinkItems.clear();

	_root->resetTransform();
	_worldMapRotation = 0.0f;
	_referential->resetTransform();
	_localRadius->resetTransform();
	this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents
}

void GraphViewer::clearMap()
{
	_gridMap->setPixmap(QPixmap());
	_gridCellSize = 0.0f;
	this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents
}

void GraphViewer::clearPosterior()
{
	for(QMap<int, NodeItem*>::iterator iter = _nodeItems.begin(); iter!=_nodeItems.end(); ++iter)
	{
		iter.value()->setColor(Qt::blue); // blue
	}
}

void GraphViewer::clearAll()
{
	clearMap();
	clearGraph();
}

void GraphViewer::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	settings.setValue("node_radius", (double)this->getNodeRadius());
	settings.setValue("link_width", (double)this->getLinkWidth());
	settings.setValue("node_color", this->getNodeColor());
	settings.setValue("node_odom_cache_color", this->getNodeOdomCacheColor());
	settings.setValue("current_goal_color", this->getCurrentGoalColor());
	settings.setValue("neighbor_color", this->getNeighborColor());
	settings.setValue("global_color", this->getGlobalLoopClosureColor());
	settings.setValue("local_color", this->getLocalLoopClosureColor());
	settings.setValue("user_color", this->getUserLoopClosureColor());
	settings.setValue("virtual_color", this->getVirtualLoopClosureColor());
	settings.setValue("neighbor_merged_color", this->getNeighborMergedColor());
	settings.setValue("rejected_color", this->getRejectedLoopClosureColor());
	settings.setValue("local_path_color", this->getLocalPathColor());
	settings.setValue("global_path_color", this->getGlobalPathColor());
	settings.setValue("gt_color", this->getGTColor());
	settings.setValue("gps_color", this->getGPSColor());
	settings.setValue("intra_session_color", this->getIntraSessionLoopColor());
	settings.setValue("inter_session_color", this->getInterSessionLoopColor());
	settings.setValue("intra_inter_session_colors_enabled", this->isIntraInterSessionColorsEnabled());
	settings.setValue("grid_visible", this->isGridMapVisible());
	settings.setValue("origin_visible", this->isOriginVisible());
	settings.setValue("referential_visible", this->isReferentialVisible());
	settings.setValue("local_radius_visible", this->isLocalRadiusVisible());
	settings.setValue("loop_closure_outlier_thr", this->getLoopClosureOutlierThr());
	settings.setValue("max_link_length", this->getMaxLinkLength());
	settings.setValue("graph_visible", this->isGraphVisible());
	settings.setValue("global_path_visible", this->isGlobalPathVisible());
	settings.setValue("local_path_visible", this->isLocalPathVisible());
	settings.setValue("gt_graph_visible", this->isGtGraphVisible());
	settings.setValue("gps_graph_visible", this->isGPSGraphVisible());
	settings.setValue("odom_cache_overlay", this->isOdomCacheOverlayVisible());
	settings.setValue("orientation_ENU", this->isOrientationENU());
	settings.setValue("view_plane", (int)this->getViewPlane());
	settings.setValue("ensure_frame_visible", (int)this->isEnsureFrameVisible());
	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void GraphViewer::loadSettings(QSettings & settings, const QString & group)
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	this->setNodeRadius(settings.value("node_radius", this->getNodeRadius()).toDouble());
	this->setLinkWidth(settings.value("link_width", this->getLinkWidth()).toDouble());
	this->setNodeColor(settings.value("node_color", this->getNodeColor()).value<QColor>());
	this->setNodeOdomCacheColor(settings.value("node_odom_cache_color", this->getNodeOdomCacheColor()).value<QColor>());
	this->setCurrentGoalColor(settings.value("current_goal_color", this->getCurrentGoalColor()).value<QColor>());
	this->setNeighborColor(settings.value("neighbor_color", this->getNeighborColor()).value<QColor>());
	this->setGlobalLoopClosureColor(settings.value("global_color", this->getGlobalLoopClosureColor()).value<QColor>());
	this->setLocalLoopClosureColor(settings.value("local_color", this->getLocalLoopClosureColor()).value<QColor>());
	this->setUserLoopClosureColor(settings.value("user_color", this->getUserLoopClosureColor()).value<QColor>());
	this->setVirtualLoopClosureColor(settings.value("virtual_color", this->getVirtualLoopClosureColor()).value<QColor>());
	this->setNeighborMergedColor(settings.value("neighbor_merged_color", this->getNeighborMergedColor()).value<QColor>());
	this->setRejectedLoopClosureColor(settings.value("rejected_color", this->getRejectedLoopClosureColor()).value<QColor>());
	this->setLocalPathColor(settings.value("local_path_color", this->getLocalPathColor()).value<QColor>());
	this->setGlobalPathColor(settings.value("global_path_color", this->getGlobalPathColor()).value<QColor>());
	this->setGTColor(settings.value("gt_color", this->getGTColor()).value<QColor>());
	this->setGPSColor(settings.value("gps_color", this->getGPSColor()).value<QColor>());
	this->setIntraSessionLoopColor(settings.value("intra_session_color", this->getIntraSessionLoopColor()).value<QColor>());
	this->setInterSessionLoopColor(settings.value("inter_session_color", this->getInterSessionLoopColor()).value<QColor>());
	this->setGridMapVisible(settings.value("grid_visible", this->isGridMapVisible()).toBool());
	this->setOriginVisible(settings.value("origin_visible", this->isOriginVisible()).toBool());
	this->setReferentialVisible(settings.value("referential_visible", this->isReferentialVisible()).toBool());
	this->setLocalRadiusVisible(settings.value("local_radius_visible", this->isLocalRadiusVisible()).toBool());
	this->setIntraInterSessionColorsEnabled(settings.value("intra_inter_session_colors_enabled", this->isIntraInterSessionColorsEnabled()).toBool());
	this->setLoopClosureOutlierThr(settings.value("loop_closure_outlier_thr", this->getLoopClosureOutlierThr()).toDouble());
	this->setMaxLinkLength(settings.value("max_link_length", this->getMaxLinkLength()).toDouble());
	this->setGraphVisible(settings.value("graph_visible", this->isGraphVisible()).toBool());
	this->setGlobalPathVisible(settings.value("global_path_visible", this->isGlobalPathVisible()).toBool());
	this->setLocalPathVisible(settings.value("local_path_visible", this->isLocalPathVisible()).toBool());
	this->setGtGraphVisible(settings.value("gt_graph_visible", this->isGtGraphVisible()).toBool());
	this->setGPSGraphVisible(settings.value("gps_graph_visible", this->isGPSGraphVisible()).toBool());
	this->setOdomCacheOverlayVisible(settings.value("odom_cache_overlay", this->isOdomCacheOverlayVisible()).toBool());
	this->setOrientationENU(settings.value("orientation_ENU", this->isOrientationENU()).toBool());
	this->setViewPlane((ViewPlane)settings.value("view_plane", (int)this->getViewPlane()).toInt());
	this->setEnsureFrameVisible(settings.value("ensure_frame_visible", this->isEnsureFrameVisible()).toBool());
	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

bool GraphViewer::isGridMapVisible() const
{
	return _gridMap->isVisible();
}
bool GraphViewer::isOriginVisible() const
{
	return _originReferential->isVisible();
}
bool GraphViewer::isReferentialVisible() const
{
	return _referential->isVisible();
}
bool GraphViewer::isLocalRadiusVisible() const
{
	return _localRadius->isVisible();
}
bool GraphViewer::isGraphVisible() const
{
	return _graphRoot->isVisible();
}
bool GraphViewer::isGlobalPathVisible() const
{
	return _globalPathRoot->isVisible();
}
bool GraphViewer::isLocalPathVisible() const
{
	return _localPathRoot->isVisible();
}
bool GraphViewer::isGtGraphVisible() const
{
	return _gtGraphRoot->isVisible();
}
bool GraphViewer::isGPSGraphVisible() const
{
	return _gpsGraphRoot->isVisible();
}
bool GraphViewer::isOdomCacheOverlayVisible() const
{
	return _odomCacheOverlay->isVisible();
}
bool GraphViewer::isOrientationENU() const
{
	return _orientationENU;
}
GraphViewer::ViewPlane GraphViewer::getViewPlane() const
{
	return _viewPlane;
}
bool GraphViewer::isEnsureFrameVisible() const
{
	return _ensureFrameVisible;
}

void GraphViewer::setWorkingDirectory(const QString & path)
{
	_workingDirectory = path;
}
void GraphViewer::setNodeVisible(bool visible)
{
	_nodeVisible = visible;
	for(QMap<int, NodeItem*>::iterator iter=_nodeItems.begin(); iter!=_nodeItems.end(); ++iter)
	{
		iter.value()->setVisible(_nodeVisible);
	}
	for(QMap<int, NodeItem*>::iterator iter=_gtNodeItems.begin(); iter!=_gtNodeItems.end(); ++iter)
	{
		iter.value()->setVisible(_nodeVisible);
	}
	for(QMap<int, NodeItem*>::iterator iter=_gpsNodeItems.begin(); iter!=_gpsNodeItems.end(); ++iter)
	{
		iter.value()->setVisible(_nodeVisible);
	}
}
void GraphViewer::setNodeRadius(float radius)
{
	_nodeRadius = radius;
	for(QMap<int, NodeItem*>::iterator iter=_nodeItems.begin(); iter!=_nodeItems.end(); ++iter)
	{
		iter.value()->setRadius(_nodeRadius);
	}
	for(QMap<int, NodeItem*>::iterator iter=_gtNodeItems.begin(); iter!=_gtNodeItems.end(); ++iter)
	{
		iter.value()->setRadius(_nodeRadius);
	}
	for(QMap<int, NodeItem*>::iterator iter=_gpsNodeItems.begin(); iter!=_gpsNodeItems.end(); ++iter)
	{
		iter.value()->setRadius(_nodeRadius);
	}
}
void GraphViewer::setLinkWidth(float width)
{
	_linkWidth = width;
	QList<QGraphicsItem*> items = this->scene()->items();
	for(int i=0; i<items.size(); ++i)
	{
		QGraphicsLineItem * line = qgraphicsitem_cast<QGraphicsLineItem *>(items[i]);
		if(line)
		{
			QPen pen = line->pen();
			pen.setWidthF(_linkWidth*100.0f);
			line->setPen(pen);
		}
	}
}
void GraphViewer::setNodeColor(const QColor & color)
{
	_nodeColor = color;
	for(QMap<int, NodeItem*>::iterator iter=_nodeItems.begin(); iter!=_nodeItems.end(); ++iter)
	{
		iter.value()->setColor(_nodeColor);
	}
}
void GraphViewer::setNodeOdomCacheColor(const QColor & color)
{
	_nodeOdomCacheColor = color;
	for(QMap<int, NodeItem*>::iterator iter=_nodeItems.begin(); iter!=_nodeItems.end(); ++iter)
	{
		if(iter.value()->zValue() == 24)
		{
			iter.value()->setColor(_nodeOdomCacheColor);
		}
	}
}
void GraphViewer::setCurrentGoalColor(const QColor & color)
{
	_currentGoalColor = color;
}
void GraphViewer::setNeighborColor(const QColor & color)
{
	_neighborColor = color;
	for(QMultiMap<int, LinkItem*>::iterator iter=_linkItems.begin(); iter!=_linkItems.end(); ++iter)
	{
		if(iter.value()->linkType() == Link::kNeighbor)
		{
			iter.value()->setColor(_neighborColor);
		}
	}
}
void GraphViewer::setGlobalLoopClosureColor(const QColor & color)
{
	_loopClosureColor = color;
	if(!_intraInterSessionColors)
	{
		for(QMultiMap<int, LinkItem*>::iterator iter=_linkItems.begin(); iter!=_linkItems.end(); ++iter)
		{
			if(iter.value()->linkType() == Link::kGlobalClosure)
			{
				iter.value()->setColor(_loopClosureColor);
				iter.value()->setZValue(10);
			}
		}
	}
}
void GraphViewer::setLocalLoopClosureColor(const QColor & color)
{
	_loopClosureLocalColor = color;
	if(!_intraInterSessionColors)
	{
		for(QMultiMap<int, LinkItem*>::iterator iter=_linkItems.begin(); iter!=_linkItems.end(); ++iter)
		{
			if(iter.value()->linkType() == Link::kLocalSpaceClosure ||
			   iter.value()->linkType() == Link::kLocalTimeClosure)
			{
				iter.value()->setColor(_loopClosureLocalColor);
				iter.value()->setZValue(10);
			}
		}
	}
}
void GraphViewer::setUserLoopClosureColor(const QColor & color)
{
	_loopClosureUserColor = color;
	for(QMultiMap<int, LinkItem*>::iterator iter=_linkItems.begin(); iter!=_linkItems.end(); ++iter)
	{
		if(iter.value()->linkType() == Link::kUserClosure)
		{
			iter.value()->setColor(_loopClosureUserColor);
		}
	}
}
void GraphViewer::setVirtualLoopClosureColor(const QColor & color)
{
	_loopClosureVirtualColor = color;
	for(QMultiMap<int, LinkItem*>::iterator iter=_linkItems.begin(); iter!=_linkItems.end(); ++iter)
	{
		if(iter.value()->linkType() == Link::kVirtualClosure)
		{
			iter.value()->setColor(_loopClosureVirtualColor);
		}
	}
}
void GraphViewer::setNeighborMergedColor(const QColor & color)
{
	_neighborMergedColor = color;
	for(QMultiMap<int, LinkItem*>::iterator iter=_linkItems.begin(); iter!=_linkItems.end(); ++iter)
	{
		if(iter.value()->linkType() == Link::kNeighborMerged)
		{
			iter.value()->setColor(_neighborMergedColor);
		}
	}
}
void GraphViewer::setLandmarkColor(const QColor & color)
{
	_landmarkColor = color;
	for(QMultiMap<int, LinkItem*>::iterator iter=_linkItems.begin(); iter!=_linkItems.end(); ++iter)
	{
		if(iter.value()->linkType() == Link::kLandmark)
		{
			iter.value()->setColor(_landmarkColor);
		}
	}
}
void GraphViewer::setRejectedLoopClosureColor(const QColor & color)
{
	_loopClosureRejectedColor = color;
}
void GraphViewer::setLocalPathColor(const QColor & color)
{
	_localPathColor = color;
}
void GraphViewer::setGlobalPathColor(const QColor & color)
{
	_globalPathColor = color;
}
void GraphViewer::setGTColor(const QColor & color)
{
	_gtPathColor = color;
	for(QMap<int, NodeItem*>::iterator iter=_gtNodeItems.begin(); iter!=_gtNodeItems.end(); ++iter)
	{
		iter.value()->setColor(_gtPathColor);
	}
	for(QMultiMap<int, LinkItem*>::iterator iter=_gtLinkItems.begin(); iter!=_gtLinkItems.end(); ++iter)
	{
		iter.value()->setColor(_gtPathColor);
	}
}
void GraphViewer::setGPSColor(const QColor & color)
{
	_gpsPathColor = color;
	for(QMap<int, NodeItem*>::iterator iter=_gpsNodeItems.begin(); iter!=_gpsNodeItems.end(); ++iter)
	{
		iter.value()->setColor(_gpsPathColor);
	}
	for(QMultiMap<int, LinkItem*>::iterator iter=_gpsLinkItems.begin(); iter!=_gpsLinkItems.end(); ++iter)
	{
		iter.value()->setColor(_gpsPathColor);
	}
}
void GraphViewer::setIntraSessionLoopColor(const QColor & color)
{
	_loopIntraSessionColor = color;
	if(_intraInterSessionColors)
	{
		for(QMultiMap<int, LinkItem*>::iterator iter=_linkItems.begin(); iter!=_linkItems.end(); ++iter)
		{
			if((iter.value()->linkType() == Link::kGlobalClosure ||
				iter.value()->linkType() == Link::kLocalSpaceClosure ||
				iter.value()->linkType() == Link::kLocalTimeClosure ||
				iter.value()->linkType() == Link::kUserClosure) &&
				!iter.value()->isInterSession())
			{
				iter.value()->setColor(_loopIntraSessionColor);
				iter.value()->setZValue(9);
			}
		}
	}
}
void GraphViewer::setInterSessionLoopColor(const QColor & color)
{
	_loopInterSessionColor = color;
	if(_intraInterSessionColors)
	{
		for(QMultiMap<int, LinkItem*>::iterator iter=_linkItems.begin(); iter!=_linkItems.end(); ++iter)
		{
			if((iter.value()->linkType() == Link::kGlobalClosure ||
				iter.value()->linkType() == Link::kLocalSpaceClosure ||
				iter.value()->linkType() == Link::kLocalTimeClosure ||
				iter.value()->linkType() == Link::kUserClosure) &&
				iter.value()->isInterSession())
			{
				iter.value()->setColor(_loopInterSessionColor);
				iter.value()->setZValue(8);
			}
		}
	}
}

void GraphViewer::setIntraInterSessionColorsEnabled(bool enabled)
{
	_intraInterSessionColors = enabled;
	if(_intraInterSessionColors)
	{
		this->setIntraSessionLoopColor(_loopIntraSessionColor);
		this->setInterSessionLoopColor(_loopInterSessionColor);
	}
	else
	{
		this->setGlobalLoopClosureColor(_loopClosureColor);
		this->setLocalLoopClosureColor(_loopClosureLocalColor);
		this->setUserLoopClosureColor(_loopClosureUserColor);
	}
}

void GraphViewer::setGridMapVisible(bool visible)
{
	if(visible && _viewPlane!=XY)
	{
		UWARN("Grid map can be shown only with view plane is XY.");
	}
	_gridMap->setVisible(_viewPlane==XY && visible);
}
void GraphViewer::setOriginVisible(bool visible)
{
	_originReferential->setVisible(visible);
}
void GraphViewer::setReferentialVisible(bool visible)
{
	_referential->setVisible(visible);
}
void GraphViewer::setLocalRadiusVisible(bool visible)
{
	_localRadius->setVisible(visible);
}
void GraphViewer::setLoopClosureOutlierThr(float value)
{
	_loopClosureOutlierThr = value;
}
void GraphViewer::setMaxLinkLength(float value)
{
	_maxLinkLength = value;
}
void GraphViewer::setGraphVisible(bool visible)
{
	_graphRoot->setVisible(visible);
}
void GraphViewer::setGlobalPathVisible(bool visible)
{
	_globalPathRoot->setVisible(visible);
}
void GraphViewer::setLocalPathVisible(bool visible)
{
	_localPathRoot->setVisible(visible);
}
void GraphViewer::setGtGraphVisible(bool visible)
{
	_gtGraphRoot->setVisible(visible);
}
void GraphViewer::setGPSGraphVisible(bool visible)
{
	_gpsGraphRoot->setVisible(visible);
}
void GraphViewer::setOdomCacheOverlayVisible(bool visible)
{
	_odomCacheOverlay->setVisible(visible);
}
void GraphViewer::setOrientationENU(bool enabled)
{
	if(enabled && _viewPlane!=XY)
	{
		UWARN("ENU orientation can be set only with view plane is XY.");
	}
	enabled = _viewPlane==XY && enabled;
	if(_orientationENU!=enabled)
	{
		_orientationENU = enabled;
		this->rotate(_orientationENU?90:270);
	}
	QTransform t;
	t.rotateRadians(_worldMapRotation);
	_root->setTransform(t);
	if(_nodeItems.size() || _linkItems.size())
	{
		this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents
	}
}

void GraphViewer::setViewPlane(ViewPlane plane)
{
	if(plane != XY)
	{
		setOrientationENU(false);
		setGridMapVisible(false);
	}
	_viewPlane = plane;

	for(QMap<int, NodeItem*>::iterator iter=_nodeItems.begin(); iter!=_nodeItems.end(); ++iter)
	{
		iter.value()->setPose(iter.value()->pose(), _viewPlane);
	}
	for(QMultiMap<int, LinkItem*>::iterator iter=_linkItems.begin(); iter!=_linkItems.end(); ++iter)
	{
		iter.value()->setPoses(iter.value()->getPoseA(), iter.value()->getPoseB(), _viewPlane);
	}

	_originReferentialXY->setVisible(plane==XY);
	_originReferentialXZ->setVisible(plane==XZ);
	_originReferentialYZ->setVisible(plane==YZ);
	_referentialXY->setVisible(plane==XY);
	_referentialXZ->setVisible(plane==XZ);
	_referentialYZ->setVisible(plane==YZ);

	if(_nodeItems.size() || _linkItems.size())
	{
		this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents
	}
}
void GraphViewer::setEnsureFrameVisible(bool visible)
{
	_ensureFrameVisible = visible;
}


void GraphViewer::restoreDefaults()
{
	setNodeRadius(0.01f);
	setLinkWidth(0.0f);
	setNodeColor(Qt::blue);
	setNeighborColor(Qt::blue);
	setGlobalLoopClosureColor(Qt::red);
	setLocalLoopClosureColor(Qt::yellow);
	setUserLoopClosureColor(Qt::red);
	setVirtualLoopClosureColor(Qt::magenta);
	setNeighborMergedColor(QColor(255,170,0));
	setLandmarkColor(Qt::darkGreen);
	setGridMapVisible(true);
	setGraphVisible(true);
	setGlobalPathVisible(true);
	setLocalPathVisible(true);
	setGtGraphVisible(true);
}

void GraphViewer::wheelEvent ( QWheelEvent * event )
{
	if(event->angleDelta().y() < 0)
	{
		this->scale(0.95, 0.95);
	}
	else
	{
		this->scale(1.05, 1.05);
	}
}

QIcon createIcon(const QColor & color)
{
	QPixmap pixmap(50, 50);
	pixmap.fill(color);
	return QIcon(pixmap);
}

void GraphViewer::contextMenuEvent(QContextMenuEvent * event)
{
	QMenu menu;
	QAction * aScreenShot = menu.addAction(tr("Take a screenshot..."));
	QAction * aExportGridMap = menu.addAction(tr("Export grid map..."));
	aExportGridMap->setEnabled(!_gridMap->pixmap().isNull());
	menu.addSeparator();

	QAction * aChangeNodeColor = menu.addAction(createIcon(_nodeColor), tr("Set node color..."));
	QAction * aChangeNodeOdomCacheColor = menu.addAction(createIcon(_nodeOdomCacheColor), tr("Set node odom cache color..."));
	QAction * aChangeCurrentGoalColor = menu.addAction(createIcon(_currentGoalColor), tr("Set current goal color..."));
	aChangeNodeColor->setIconVisibleInMenu(true);
	aChangeNodeOdomCacheColor->setIconVisibleInMenu(true);
	aChangeCurrentGoalColor->setIconVisibleInMenu(true);

	// Links
	QMenu * menuLink = menu.addMenu(tr("Set link color..."));
	QAction * aChangeNeighborColor = menuLink->addAction(tr("Neighbor"));
	QAction * aChangeGlobalLoopColor = menuLink->addAction(tr("Global loop closure"));
	QAction * aChangeLocalLoopColor = menuLink->addAction(tr("Local loop closure"));
	QAction * aChangeUserLoopColor = menuLink->addAction(tr("User loop closure"));
	QAction * aChangeVirtualLoopColor = menuLink->addAction(tr("Virtual loop closure"));
	QAction * aChangeNeighborMergedColor = menuLink->addAction(tr("Neighbor merged"));
	QAction * aChangeLandmarkColor = menuLink->addAction(tr("Landmark"));
	QAction * aChangeRejectedLoopColor = menuLink->addAction(tr("Outlier loop closure"));
	QAction * aChangeRejectedLoopThr = menuLink->addAction(tr("Set outlier threshold..."));
	QAction * aChangeLocalPathColor = menuLink->addAction(tr("Local path"));
	QAction * aChangeGlobalPathColor = menuLink->addAction(tr("Global path"));
	QAction * aChangeGTColor = menuLink->addAction(tr("Ground truth"));
	QAction * aChangeGPSColor = menuLink->addAction(tr("GPS"));
	menuLink->addSeparator();
	QAction * aSetIntraInterSessionColors = menuLink->addAction(tr("Enable intra/inter-session colors"));
	QAction * aChangeIntraSessionLoopColor = menuLink->addAction(tr("Intra-session loop closure"));
	QAction * aChangeInterSessionLoopColor = menuLink->addAction(tr("Inter-session loop closure"));
	aChangeNeighborColor->setIcon(createIcon(_neighborColor));
	aChangeGlobalLoopColor->setIcon(createIcon(_loopClosureColor));
	aChangeLocalLoopColor->setIcon(createIcon(_loopClosureLocalColor));
	aChangeUserLoopColor->setIcon(createIcon(_loopClosureUserColor));
	aChangeVirtualLoopColor->setIcon(createIcon(_loopClosureVirtualColor));
	aChangeNeighborMergedColor->setIcon(createIcon(_neighborMergedColor));
	aChangeLandmarkColor->setIcon(createIcon(_landmarkColor));
	aChangeRejectedLoopColor->setIcon(createIcon(_loopClosureRejectedColor));
	aChangeLocalPathColor->setIcon(createIcon(_localPathColor));
	aChangeGlobalPathColor->setIcon(createIcon(_globalPathColor));
	aChangeGTColor->setIcon(createIcon(_gtPathColor));
	aChangeGPSColor->setIcon(createIcon(_gpsPathColor));;
	aChangeIntraSessionLoopColor->setIcon(createIcon(_loopIntraSessionColor));
	aChangeInterSessionLoopColor->setIcon(createIcon(_loopInterSessionColor));
	aChangeNeighborColor->setIconVisibleInMenu(true);
	aChangeGlobalLoopColor->setIconVisibleInMenu(true);
	aChangeLocalLoopColor->setIconVisibleInMenu(true);
	aChangeUserLoopColor->setIconVisibleInMenu(true);
	aChangeVirtualLoopColor->setIconVisibleInMenu(true);
	aChangeNeighborMergedColor->setIconVisibleInMenu(true);
	aChangeRejectedLoopColor->setIconVisibleInMenu(true);
	aChangeLocalPathColor->setIconVisibleInMenu(true);
	aChangeGlobalPathColor->setIconVisibleInMenu(true);
	aChangeGTColor->setIconVisibleInMenu(true);
	aChangeGPSColor->setIconVisibleInMenu(true);
	aChangeIntraSessionLoopColor->setIconVisibleInMenu(true);
	aChangeInterSessionLoopColor->setIconVisibleInMenu(true);
	aSetIntraInterSessionColors->setCheckable(true);
	aSetIntraInterSessionColors->setChecked(_intraInterSessionColors);

	menu.addSeparator();
	QAction * aSetNodeSize = menu.addAction(tr("Set node radius..."));
	QAction * aSetLinkSize = menu.addAction(tr("Set link width..."));
	QAction * aChangeMaxLinkLength = menu.addAction(tr("Set maximum link length..."));
	menu.addSeparator();
	QAction * aEnsureFrameVisible;
	QAction * aShowHideGridMap;
	QAction * aShowHideGraph;
	QAction * aShowHideGraphNodes;
	QAction * aShowHideOrigin;
	QAction * aShowHideReferential;
	QAction * aShowHideLocalRadius;
	QAction * aShowHideGlobalPath;
	QAction * aShowHideLocalPath;
	QAction * aShowHideGtGraph;
	QAction * aShowHideGPSGraph;
	QAction * aShowHideOdomCacheOverlay;
	QAction * aOrientationENU;
	QAction * aViewPlaneXY;
	QAction * aViewPlaneXZ;
	QAction * aViewPlaneYZ;
	aEnsureFrameVisible = menu.addAction(tr("Ensure Frame Visible"));
	aEnsureFrameVisible->setCheckable(true);
	aEnsureFrameVisible->setChecked(_ensureFrameVisible);
	if(_gridMap->isVisible())
	{
		aShowHideGridMap = menu.addAction(tr("Hide grid map"));
	}
	else
	{
		aShowHideGridMap = menu.addAction(tr("Show grid map"));
	}
	aShowHideGridMap->setEnabled(_viewPlane == XY);
	if(_originReferential->isVisible())
	{
		aShowHideOrigin = menu.addAction(tr("Hide origin referential"));
	}
	else
	{
		aShowHideOrigin = menu.addAction(tr("Show origin referential"));
	}
	if(_referential->isVisible())
	{
		aShowHideReferential = menu.addAction(tr("Hide current referential"));
	}
	else
	{
		aShowHideReferential = menu.addAction(tr("Show current referential"));
	}
	if(_localRadius->isVisible())
	{
		aShowHideLocalRadius = menu.addAction(tr("Hide local radius"));
	}
	else
	{
		aShowHideLocalRadius = menu.addAction(tr("Show local radius"));
	}
	if(_graphRoot->isVisible())
	{
		aShowHideGraph = menu.addAction(tr("Hide graph"));
	}
	else
	{
		aShowHideGraph = menu.addAction(tr("Show graph"));
	}
	if(_nodeVisible)
	{
		aShowHideGraphNodes = menu.addAction(tr("Hide graph nodes"));
	}
	else
	{
		aShowHideGraphNodes = menu.addAction(tr("Show graph nodes"));
	}
	if(_globalPathRoot->isVisible())
	{
		aShowHideGlobalPath = menu.addAction(tr("Hide global path"));
	}
	else
	{
		aShowHideGlobalPath = menu.addAction(tr("Show global path"));
	}
	if(_localPathRoot->isVisible())
	{
		aShowHideLocalPath = menu.addAction(tr("Hide local path"));
	}
	else
	{
		aShowHideLocalPath = menu.addAction(tr("Show local path"));
	}
	if(_gtGraphRoot->isVisible())
	{
		aShowHideGtGraph = menu.addAction(tr("Hide ground truth graph"));
	}
	else
	{
		aShowHideGtGraph = menu.addAction(tr("Show ground truth graph"));
	}
	if(_gpsGraphRoot->isVisible())
	{
		aShowHideGPSGraph = menu.addAction(tr("Hide GPS graph"));
	}
	else
	{
		aShowHideGPSGraph = menu.addAction(tr("Show GPS graph"));
	}
	if(_odomCacheOverlay->isVisible())
	{
		aShowHideOdomCacheOverlay = menu.addAction(tr("Hide odom cache overlay"));
	}
	else
	{
		aShowHideOdomCacheOverlay = menu.addAction(tr("Show odom cache overlay"));
	}
	aOrientationENU = menu.addAction(tr("ENU Orientation"));
	aOrientationENU->setCheckable(true);
	aOrientationENU->setChecked(_orientationENU);
	aShowHideGraph->setEnabled(_nodeItems.size() && _viewPlane == XY);
	aShowHideGraphNodes->setEnabled(_nodeItems.size() && _graphRoot->isVisible());
	aShowHideGlobalPath->setEnabled(_globalPathLinkItems.size());
	aShowHideLocalPath->setEnabled(_localPathLinkItems.size());
	aShowHideGtGraph->setEnabled(_gtNodeItems.size());
	aShowHideGPSGraph->setEnabled(_gpsNodeItems.size());
	aShowHideOdomCacheOverlay->setEnabled(_odomCacheOverlay->rect().width()>0);

	QMenu * viewPlaneMenu = menu.addMenu("View Plane...");
	aViewPlaneXY = viewPlaneMenu->addAction("XY");
	aViewPlaneXY->setCheckable(true);
	aViewPlaneXY->setChecked(_viewPlane == XY);
	aViewPlaneXZ = viewPlaneMenu->addAction("XZ");
	aViewPlaneXZ->setCheckable(true);
	aViewPlaneXZ->setChecked(_viewPlane == XZ);
	aViewPlaneYZ = viewPlaneMenu->addAction("YZ");
	aViewPlaneYZ->setCheckable(true);
	aViewPlaneYZ->setChecked(_viewPlane == YZ);

	menu.addSeparator();
	QAction * aRestoreDefaults = menu.addAction(tr("Restore defaults"));

	QAction * r = menu.exec(event->globalPos());
	if(r == aScreenShot)
	{
		if(_root)
		{
			QString filePath;
#if QT_VERSION >= 0x050000
			filePath = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
#endif
			QDir dir;
			if(!dir.exists(filePath))
			{
				filePath = QDir::homePath();
			}
			filePath += "/graph.png";

#ifdef QT_SVG_LIB
			filePath = QFileDialog::getSaveFileName(this, tr("Save figure to ..."), filePath, "*.png *.xpm *.jpg *.pdf *.svg");
#else
			filePath = QFileDialog::getSaveFileName(this, tr("Save figure to ..."), filePath, "*.png *.xpm *.jpg *.pdf");
#endif
			if(!filePath.isEmpty())
			{
				if(QFileInfo(filePath).suffix() == "")
				{
					//use png by default
					filePath += ".png";
				}

				if(_gridCellSize)
				{
					_root->setScale(1.0f/(_gridCellSize*100.0f)); // grid map precision (for 5cm grid cell, x20 to have 1pix/5cm)
				}
				else
				{
					_root->setScale(this->transform().m11()); // current view
				}

				this->scene()->clearSelection();                                  // Selections would also render to the file
				this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents
				QSize sceneSize = this->scene()->sceneRect().size().toSize();

				if(QFileInfo(filePath).suffix().compare("pdf") == 0)
				{
					QPrinter printer(QPrinter::HighResolution);
					printer.setPageOrientation(QPageLayout::Portrait);
					printer.setOutputFileName( filePath );
					QPainter p(&printer);
					scene()->render(&p);
					p.end();
				}
				else if(QFileInfo(filePath).suffix().compare("svg") == 0)
				{
#ifdef QT_SVG_LIB
					QSvgGenerator svgGen;

					svgGen.setFileName( filePath );
					svgGen.setSize(sceneSize);
					// add 1% border to make sure values are not cropped
					int borderH = sceneSize.width()/100;
					int borderV = sceneSize.height()/100;
					svgGen.setViewBox(QRect(-borderH, -borderV, sceneSize.width()+borderH*2, sceneSize.height()+borderV*2));
					svgGen.setTitle(tr("RTAB-Map graph"));
					svgGen.setDescription(tr("RTAB-Map map and graph"));

					QPainter painter( &svgGen );

					this->scene()->render(&painter);
#else
					UERROR("RTAB-MAp is not built with Qt's SVG library, cannot save picture in svg format.");
#endif
				}
				else
				{
					QImage image(sceneSize, QImage::Format_ARGB32);  // Create the image with the exact size of the shrunk scene
					image.fill(Qt::transparent);                     // Start all pixels transparent
					QPainter painter(&image);

					this->scene()->render(&painter);
					if(!image.isNull())
					{
						image.save(filePath);
					}
					else
					{
						QMessageBox::warning(this,
								tr("Save PNG"),
								tr("Could not export in PNG (the scene may be too large %1x%2), try saving in SVG.").arg(sceneSize.width()).arg(sceneSize.height()));
					}
				}

				//reset scale
				_root->setScale(1.0f);
				this->scene()->setSceneRect(this->scene()->itemsBoundingRect());  // Re-shrink the scene to it's bounding contents


				QDesktopServices::openUrl(QUrl::fromLocalFile(filePath));
			}
		}
		return; // without emitting configChanged
	}
	else if(r == aExportGridMap)
	{
		float xMin, yMin, cellSize;

		cellSize = _gridCellSize;
		xMin = -_gridMap->y()/100.0f;
		yMin = -_gridMap->x()/100.0f;

		QString path = QFileDialog::getSaveFileName(
				this,
				tr("Save File"),
				"map.pgm",
				tr("Map (*.pgm)"));

		if(!path.isEmpty())
		{
			if(QFileInfo(path).suffix() == "")
			{
				path += ".pgm";
			}
			_gridMap->pixmap().save(path);

			QFileInfo info(path);
			QString yaml = info.absolutePath() + "/" +  info.baseName() + ".yaml";

			float occupancyThr = Parameters::defaultGridGlobalOccupancyThr();

			std::ofstream file;
			file.open (yaml.toStdString());
			file << "image: " << info.baseName().toStdString() << ".pgm" << std::endl;
			file << "resolution: " << cellSize << std::endl;
			file << "origin: [" << xMin << ", " << yMin << ", 0.0]" << std::endl;
			file << "negate: 0" << std::endl;
			file << "occupied_thresh: " << occupancyThr << std::endl;
			file << "free_thresh: 0.196" << std::endl;
			file << std::endl;
			file.close();


			QMessageBox::information(this, tr("Export 2D map"), tr("Exported %1 and %2!").arg(path).arg(yaml));
		}
	}
	else if(r == aSetIntraInterSessionColors)
	{
		setIntraInterSessionColorsEnabled(aSetIntraInterSessionColors->isChecked());
	}
	else if(r == aChangeRejectedLoopThr)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Loop closure outlier threshold"), tr("Value (m)"), _loopClosureOutlierThr, 0.0, 1000.0, 2, &ok);
		if(ok)
		{
			setLoopClosureOutlierThr(value);
		}
	}
	else if(r == aChangeMaxLinkLength)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Maximum link length to be shown"), tr("Value (m)"), _maxLinkLength, 0.0, 1000.0, 3, &ok);
		if(ok)
		{
			setMaxLinkLength(value);
		}
	}
	else if(r == aChangeNodeColor ||
			r == aChangeNodeOdomCacheColor ||
			r == aChangeCurrentGoalColor ||
			r == aChangeNeighborColor ||
			r == aChangeGlobalLoopColor ||
			r == aChangeLocalLoopColor ||
			r == aChangeUserLoopColor ||
			r == aChangeVirtualLoopColor ||
			r == aChangeNeighborMergedColor ||
			r == aChangeRejectedLoopColor ||
			r == aChangeLocalPathColor ||
			r == aChangeGlobalPathColor ||
			r == aChangeGTColor ||
			r == aChangeGPSColor ||
			r == aChangeIntraSessionLoopColor ||
			r == aChangeInterSessionLoopColor)
	{
		QColor color;
		if(r == aChangeNodeColor)
		{
			color = _nodeColor;
		}
		else if(r == aChangeNodeOdomCacheColor)
		{
			color = _nodeOdomCacheColor;
		}
		else if(r == aChangeCurrentGoalColor)
		{
			color = _currentGoalColor;
		}
		else if(r == aChangeGlobalLoopColor)
		{
			color = _loopClosureColor;
		}
		else if(r == aChangeLocalLoopColor)
		{
			color = _loopClosureLocalColor;
		}
		else if(r == aChangeUserLoopColor)
		{
			color = _loopClosureUserColor;
		}
		else if(r == aChangeVirtualLoopColor)
		{
			color = _loopClosureVirtualColor;
		}
		else if(r == aChangeNeighborMergedColor)
		{
			color = _neighborMergedColor;
		}
		else if(r == aChangeLandmarkColor)
		{
			color = _landmarkColor;
		}
		else if(r == aChangeRejectedLoopColor)
		{
			color = _loopClosureRejectedColor;
		}
		else if(r == aChangeLocalPathColor)
		{
			color = _localPathColor;
		}
		else if(r == aChangeGlobalPathColor)
		{
			color = _globalPathColor;
		}
		else if(r == aChangeGTColor)
		{
			color = _gtPathColor;
		}
		else if(r == aChangeGPSColor)
		{
			color = _gpsPathColor;
		}
		else if(r == aChangeIntraSessionLoopColor)
		{
			color = _loopIntraSessionColor;
		}
		else if(r == aChangeInterSessionLoopColor)
		{
			color = _loopInterSessionColor;
		}
		else //if(r == aChangeNeighborColor)
		{
			color = _neighborColor;
		}
		color = QColorDialog::getColor(color, this);
		if(color.isValid())
		{

			if(r == aChangeNodeColor)
			{
				this->setNodeColor(color);
			}
			else if(r == aChangeNodeOdomCacheColor)
			{
				this->setNodeOdomCacheColor(color);
			}
			else if(r == aChangeCurrentGoalColor)
			{
				this->setCurrentGoalColor(color);
			}
			else if(r == aChangeGlobalLoopColor)
			{
				this->setGlobalLoopClosureColor(color);
			}
			else if(r == aChangeLocalLoopColor)
			{
				this->setLocalLoopClosureColor(color);
			}
			else if(r == aChangeUserLoopColor)
			{
				this->setUserLoopClosureColor(color);
			}
			else if(r == aChangeVirtualLoopColor)
			{
				this->setVirtualLoopClosureColor(color);
			}
			else if(r == aChangeNeighborMergedColor)
			{
				this->setNeighborMergedColor(color);
			}
			else if(r == aChangeLandmarkColor)
			{
				this->setLandmarkColor(color);
			}
			else if(r == aChangeRejectedLoopColor)
			{
				this->setRejectedLoopClosureColor(color);
			}
			else if(r == aChangeLocalPathColor)
			{
				this->setLocalPathColor(color);
			}
			else if(r == aChangeGlobalPathColor)
			{
				this->setGlobalPathColor(color);
			}
			else if(r == aChangeGTColor)
			{
				this->setGTColor(color);
			}
			else if(r == aChangeGPSColor)
			{
				this->setGPSColor(color);
			}
			else if(r == aChangeIntraSessionLoopColor)
			{
				this->setIntraSessionLoopColor(color);
			}
			else if(r == aChangeInterSessionLoopColor)
			{
				this->setInterSessionLoopColor(color);
			}
			else //if(r == aChangeNeighborColor)
			{
				this->setNeighborColor(color);
			}
		}
		else
		{
			return; // without emitting configChanged
		}
	}
	else if(r == aSetNodeSize)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Node radius"), tr("Radius (m)"), _nodeRadius, 0.001, 100, 3, &ok);
		if(ok)
		{
			setNodeRadius(value);
		}
	}
	else if(r == aSetLinkSize)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Link width"), tr("Width (m)"), _linkWidth, 0, 100, 2, &ok);
		if(ok)
		{
			setLinkWidth(value);
		}
	}
	else if(r == aEnsureFrameVisible)
	{
		this->setEnsureFrameVisible(!this->isEnsureFrameVisible());
	}
	else if(r == aShowHideGridMap)
	{
		this->setGridMapVisible(!this->isGridMapVisible());
		if(_gridMap->isVisible())
		{
			 mapShownRequested();
		}
	}
	else if(r == aShowHideOrigin)
	{
		this->setOriginVisible(!this->isOriginVisible());
	}
	else if(r == aShowHideReferential)
	{
		this->setReferentialVisible(!this->isReferentialVisible());
	}
	else if(r == aShowHideLocalRadius)
	{
		this->setLocalRadiusVisible(!this->isLocalRadiusVisible());
	}
	else if(r == aRestoreDefaults)
	{
		this->restoreDefaults();
	}
	else if(r == aShowHideGraph)
	{
		this->setGraphVisible(!this->isGraphVisible());
	}
	else if(r == aShowHideGraphNodes)
	{
		this->setNodeVisible(!_nodeVisible);
	}
	else if(r == aShowHideGlobalPath)
	{
		this->setGlobalPathVisible(!this->isGlobalPathVisible());
	}
	else if(r == aShowHideLocalPath)
	{
		this->setLocalPathVisible(!this->isLocalPathVisible());
	}
	else if(r == aShowHideGtGraph)
	{
		this->setGtGraphVisible(!this->isGtGraphVisible());
	}
	else if(r == aShowHideGPSGraph)
	{
		this->setGPSGraphVisible(!this->isGPSGraphVisible());
	}
	else if(r == aShowHideOdomCacheOverlay)
	{
		this->setOdomCacheOverlayVisible(!this->isOdomCacheOverlayVisible());
	}
	else if(r == aOrientationENU)
	{
		this->setOrientationENU(!this->isOrientationENU());
	}
	else if(r == aViewPlaneXY)
	{
		this->setViewPlane(XY);
	}
	else if(r == aViewPlaneXZ)
	{
		this->setViewPlane(XZ);
	}
	else if(r == aViewPlaneYZ)
	{
		this->setViewPlane(YZ);
	}

	if(r)
	{
		Q_EMIT configChanged();
	}
}

} /* namespace rtabmap */
