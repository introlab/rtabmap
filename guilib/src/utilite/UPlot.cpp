/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "rtabmap/utilite/UPlot.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UMath.h"

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QGraphicsRectItem>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QtGui/QResizeEvent>
#include <QtGui/QMouseEvent>
#include <QtCore/QTime>
#include <QtCore/QTimer>
#include <QtCore/QFileInfo>
#include <QPushButton>
#include <QToolButton>
#include <QLabel>
#include <QMenu>
#include <QStyle>
#include <QInputDialog>
#include <QMessageBox>
#include <QFileDialog>
#include <QtGui/QClipboard>
#include <QApplication>
#include <QPrinter>
#include <QColorDialog>
#include <QToolTip>
#ifdef QT_SVG_LIB
#include <QtSvg/QSvgGenerator>
#endif
#include <cmath>
#include <limits>

#define PRINT_DEBUG 0

UPlotItem::UPlotItem(qreal dataX, qreal dataY, qreal width) :
	QGraphicsEllipseItem(0, 0, width, width, 0),
	_previousItem(0),
	_nextItem(0),
	_text(0),
	_textBackground(0)
{
	this->init(dataX, dataY);
}

UPlotItem::UPlotItem(const QPointF & data, qreal width) :
	QGraphicsEllipseItem(0, 0, width, width, 0),
	_previousItem(0),
	_nextItem(0),
	_text(0),
	_textBackground(0)
{
	this->init(data.x(), data.y());
}

void UPlotItem::init(qreal dataX, qreal dataY)
{
	_data.setX(dataX);
	_data.setY(dataY);
	this->setAcceptHoverEvents(true);
	this->setFlag(QGraphicsItem::ItemIsFocusable, true);
}

UPlotItem::~UPlotItem()
{
	if(_previousItem && _nextItem)
	{
		_previousItem->setNextItem(_nextItem);
		_nextItem->setPreviousItem(_previousItem);
	}
	else if(_previousItem)
	{
		_previousItem->setNextItem(0);
	}
	else if(_nextItem)
	{
		_nextItem->setPreviousItem(0);
	}
}

void UPlotItem::setData(const QPointF & data)
{
	_data = data;
}

void UPlotItem::setNextItem(UPlotItem * nextItem)
{
	if(_nextItem != nextItem)
	{
		_nextItem = nextItem;
		if(nextItem)
		{
			nextItem->setPreviousItem(this);
		}
	}
}

void UPlotItem::setPreviousItem(UPlotItem * previousItem)
{
	if(_previousItem != previousItem)
	{
		_previousItem = previousItem;
		if(previousItem)
		{
			previousItem->setNextItem(this);
		}
	}
}

void UPlotItem::showDescription(bool shown)
{
	if(!_textBackground)
	{
		_textBackground = new QGraphicsRectItem(this);
		_textBackground->setBrush(QBrush(QColor(255, 255, 255, 200)));
		_textBackground->setPen(Qt::NoPen);
		_textBackground->setZValue(this->zValue()+1);
		_textBackground->setVisible(false);

		_text = new QGraphicsTextItem(_textBackground);
	}

	if(this->parentItem() && this->parentItem() != _textBackground->parentItem())
	{
		_textBackground->setParentItem(this->parentItem());
		_textBackground->setZValue(this->zValue()+1);
	}

	if(this->scene() && shown)
	{
		_textBackground->setVisible(true);
		_text->setPlainText(QString("(%1,%2)").arg(_data.x()).arg(_data.y()));

		this->setPen(QPen(this->pen().color(), 2));

		QRectF rect = this->scene()->sceneRect();
		QPointF p = this->pos();
		QRectF br = _text->boundingRect();
		_textBackground->setRect(QRectF(0,0,br.width(), br.height()));

		// Make sure the text is always in the scene
		if(p.x() - br.width() < 0)
		{
			p.setX(0);
		}
		else if(p.x() > rect.width())
		{
			p.setX(rect.width() - br.width());
		}
		else
		{
			p.setX(p.x() - br.width());
		}

		if(p.y() - br.height() < 0)
		{
			p.setY(0);
		}
		else
		{
			p.setY(p.y() - br.height());
		}

		_textBackground->setPos(p);
	}
	else
	{
		this->setPen(QPen(this->pen().color(), 1));
		_textBackground->setVisible(false);
	}
}

void UPlotItem::hoverEnterEvent(QGraphicsSceneHoverEvent * event)
{
	this->showDescription(true);
	QGraphicsEllipseItem::hoverEnterEvent(event);
}

void UPlotItem::hoverLeaveEvent(QGraphicsSceneHoverEvent * event)
{
	if(!this->hasFocus())
	{
		this->showDescription(false);
	}
	QGraphicsEllipseItem::hoverLeaveEvent(event);
}

void UPlotItem::focusInEvent(QFocusEvent * event)
{
	this->showDescription(true);
	QGraphicsEllipseItem::focusInEvent(event);
}

void UPlotItem::focusOutEvent(QFocusEvent * event)
{
	this->showDescription(false);
	QGraphicsEllipseItem::focusOutEvent(event);
}

void UPlotItem::keyReleaseEvent(QKeyEvent * keyEvent)
{
	//Get the next/previous visible item
	if(keyEvent->key() == Qt::Key_Right)
	{
		UPlotItem * next = _nextItem;
		while(next && !next->isVisible())
		{
			next = next->nextItem();
		}
		if(next && next->isVisible())
		{
			this->clearFocus();
			next->setFocus();
		}
	}
	else if(keyEvent->key() == Qt::Key_Left)
	{
		UPlotItem * previous = _previousItem;
		while(previous && !previous->isVisible())
		{
			previous = previous->previousItem();
		}
		if(previous && previous->isVisible())
		{
			this->clearFocus();
			previous->setFocus();
		}
	}
	QGraphicsEllipseItem::keyReleaseEvent(keyEvent);
}





UPlotCurve::UPlotCurve(const QString & name, QObject * parent) :
	QObject(parent),
	_plot(0),
	_name(name),
	_xIncrement(1),
	_xStart(0),
	_visible(true),
	_valuesShown(false),
	_itemsColor(0,0,0,150)
{
	_rootItem = new QGraphicsRectItem();
}

UPlotCurve::UPlotCurve(const QString & name, QVector<UPlotItem *> data, QObject * parent) :
	QObject(parent),
	_plot(0),
	_name(name),
	_xIncrement(1),
	_xStart(0),
	_visible(true),
	_valuesShown(false),
	_itemsColor(0,0,0,150)
{
	_rootItem = new QGraphicsRectItem();
	this->setData(data);
}

UPlotCurve::UPlotCurve(const QString & name, const QVector<qreal> & x, const QVector<qreal> & y, QObject * parent) :
	QObject(parent),
	_plot(0),
	_name(name),
	_xIncrement(1),
	_xStart(0),
	_visible(true),
	_valuesShown(false),
	_itemsColor(0,0,0,150)
{
	_rootItem = new QGraphicsRectItem();
	this->setData(x, y);
}

UPlotCurve::~UPlotCurve()
{
	if(_plot)
	{
		_plot->removeCurve(this);
	}
#if PRINT_DEBUG
	ULOGGER_DEBUG("%s", this->name().toStdString().c_str());
#endif
	this->clear();
	delete _rootItem;
}

void UPlotCurve::attach(UPlot * plot)
{
	if(!plot || plot == _plot)
	{
		return;
	}
	if(_plot)
	{
		_plot->removeCurve(this);
	}
	_plot = plot;
	_plot->addItem(_rootItem);
}

void UPlotCurve::detach(UPlot * plot)
{
#if PRINT_DEBUG
	ULOGGER_DEBUG("curve=\"%s\" from plot=\"%s\"", this->objectName().toStdString().c_str(), plot?plot->objectName().toStdString().c_str():"");
#endif
	if(plot && _plot == plot)
	{
		_plot = 0;
		if(_rootItem->scene())
		{
			_rootItem->scene()->removeItem(_rootItem);
		}
	}
}

void UPlotCurve::updateMinMax()
{
	qreal x,y;
	const UPlotItem * item;
	if(!_items.size())
	{
		_minMax = QVector<qreal>();
	}
	else
	{
		_minMax = QVector<qreal>(4);
	}
	for(int i=0; i<_items.size(); ++i)
	{
		item = qgraphicsitem_cast<const UPlotItem *>(_items.at(i));
		if(item)
		{
			x = item->data().x();
			y = item->data().y();
			if(i==0)
			{
				_minMax[0] = x;
				_minMax[1] = x;
				_minMax[2] = y;
				_minMax[3] = y;
			}
			else
			{
				if(x<_minMax[0]) _minMax[0] = x;
				if(x>_minMax[1]) _minMax[1] = x;
				if(y<_minMax[2]) _minMax[2] = y;
				if(y>_minMax[3]) _minMax[3] = y;
			}
		}
	}
}

void UPlotCurve::_addValue(UPlotItem * data)
{
	// add item
	if(data)
	{
		qreal x = data->data().x();
		qreal y = data->data().y();

		if(_minMax.size() != 4)
		{
			_minMax = QVector<qreal>(4);
		}
		if(_items.size())
		{
			data->setPreviousItem((UPlotItem *)_items.last());
			QGraphicsLineItem * line = new QGraphicsLineItem(_rootItem);
			line->setPen(_pen);
			line->setVisible(false);
			_items.append(line);
			//Update min/max
			if(x<_minMax[0]) _minMax[0] = x;
			if(x>_minMax[1]) _minMax[1] = x;
			if(y<_minMax[2]) _minMax[2] = y;
			if(y>_minMax[3]) _minMax[3] = y;
		}
		else
		{
			_minMax[0] = x;
			_minMax[1] = x;
			_minMax[2] = y;
			_minMax[3] = y;
		}
		data->setParentItem(_rootItem);
		data->setZValue(1);
		_items.append(data);
		data->setVisible(false);
		QPen pen = data->pen();
		pen.setColor(_itemsColor);
		data->setPen(pen);
	}
	else
	{
		ULOGGER_ERROR("Data is null ?!?");
	}
}

void UPlotCurve::addValue(UPlotItem * data)
{
	// add item
	if(data)
	{
		this->_addValue(data);
		Q_EMIT dataChanged(this);
	}
}

void UPlotCurve::addValue(qreal x, qreal y)
{
	if(_items.size() &&
		_minMax[0] != _minMax[1] &&
		x < _minMax[1])
	{
		UWARN("New value (%f) added to curve \"%s\" is smaller "
			  "than the last added (%f). Clearing the curve.",
				x, this->name().toStdString().c_str(), ((UPlotItem*)_items.back())->data().x());
		this->clear();
	}

	qreal width = 2; // TODO warn : hard coded value!
	this->addValue(new UPlotItem(x,y,width));
}

void UPlotCurve::addValue(qreal y)
{
	qreal x = 0;
	if(_items.size())
	{
		UPlotItem * lastItem = (UPlotItem *)_items.last();
		x = lastItem->data().x() + _xIncrement;
	}
	else
	{
		x = _xStart;
	}
	this->addValue(x,y);
}

void UPlotCurve::addValue(const QString & value)
{
	bool ok;
	qreal v = value.toDouble(&ok);
	if(ok)
	{
		this->addValue(v);
	}
	else
	{
		ULOGGER_ERROR("Value not valid, must be a number, received %s", value.toStdString().c_str());
	}
}

void UPlotCurve::addValues(QVector<UPlotItem *> & data)
{
	for(int i=0; i<data.size(); ++i)
	{
		this->_addValue(data.at(i));
	}
	Q_EMIT dataChanged(this);
}

void UPlotCurve::addValues(const QVector<qreal> & xs, const QVector<qreal> & ys)
{
	qreal width = 2; // TODO warn : hard coded value!
	for(int i=0; i<xs.size() && i<ys.size(); ++i)
	{
		this->_addValue(new UPlotItem(xs.at(i),ys.at(i),width));
	}
	Q_EMIT dataChanged(this);
}

void UPlotCurve::addValues(const QVector<qreal> & ys)
{
	qreal x = 0;
	qreal width = 2; // TODO warn : hard coded value!
	for(int i=0; i<ys.size(); ++i)
	{
		if(_items.size())
		{
			UPlotItem * lastItem = (UPlotItem *)_items.last();
			x = lastItem->data().x() + _xIncrement;
		}
		else
		{
			x = _xStart;
		}
		this->_addValue(new UPlotItem(x,ys.at(i),width));
	}
	Q_EMIT dataChanged(this);
}

void UPlotCurve::addValues(const QVector<int> & ys)
{
	qreal x = 0;
	qreal width = 2; // TODO warn : hard coded value!
	for(int i=0; i<ys.size(); ++i)
	{
		if(_items.size())
		{
			UPlotItem * lastItem = (UPlotItem *)_items.last();
			x = lastItem->data().x() + _xIncrement;
		}
		else
		{
			x = _xStart;
		}
		this->_addValue(new UPlotItem(x,ys.at(i),width));
	}
	Q_EMIT dataChanged(this);
}

void UPlotCurve::addValues(const std::vector<int> & ys)
{
	qreal x = 0;
	qreal width = 2; // TODO warn : hard coded value!
	for(unsigned int i=0; i<ys.size(); ++i)
	{
		if(_items.size())
		{
			UPlotItem * lastItem = (UPlotItem *)_items.last();
			x = lastItem->data().x() + _xIncrement;
		}
		else
		{
			x = _xStart;
		}
		this->_addValue(new UPlotItem(x,ys.at(i),width));
	}
	Q_EMIT dataChanged(this);
}

void UPlotCurve::addValues(const std::vector<qreal> & ys)
{
	qreal x = 0;
	qreal width = 2; // TODO warn : hard coded value!
	for(unsigned int i=0; i<ys.size(); ++i)
	{
		if(_items.size())
		{
			UPlotItem * lastItem = (UPlotItem *)_items.last();
			x = lastItem->data().x() + _xIncrement;
		}
		else
		{
			x = _xStart;
		}
		this->_addValue(new UPlotItem(x,ys.at(i),width));
	}
	Q_EMIT dataChanged(this);
}

int UPlotCurve::removeItem(int index)
{
	if(index >= 0 && index < _items.size())
	{
		if(index!=0)
		{
			index-=1;
			delete _items.takeAt(index); // the line
		}
		else if(_items.size()>1)
		{
			delete _items.takeAt(index+1); // the line
		}
		UPlotItem * item = (UPlotItem *)_items.takeAt(index); // the plot item
		//Update min/max
		if(_minMax.size() == 4)
		{
			if(item->data().x() == _minMax[0] || item->data().x() == _minMax[1] ||
			   item->data().y() == _minMax[2] || item->data().y() == _minMax[3])
			{
				if(_items.size())
				{
					UPlotItem * tmp = (UPlotItem *)_items.at(0);
					qreal x = tmp->data().x();
					qreal y = tmp->data().y();
					_minMax[0]=x;
					_minMax[1]=x;
					_minMax[2]=y;
					_minMax[3]=y;
					for(int i = 2; i<_items.size(); i+=2)
					{
						tmp = (UPlotItem*)_items.at(i);
						x = tmp->data().x();
						y = tmp->data().y();
						if(x<_minMax[0]) _minMax[0] = x;
						if(x>_minMax[1]) _minMax[1] = x;
						if(y<_minMax[2]) _minMax[2] = y;
						if(y>_minMax[3]) _minMax[3] = y;
					}
				}
				else
				{
					_minMax = QVector<qreal>();
				}
			}
		}
		delete item;
	}

	return index;
}

void UPlotCurve::removeItem(UPlotItem * item) // ownership is transfered to the caller
{
	for(int i=0; i<_items.size(); ++i)
	{
		if(_items.at(i) == item)
		{
			if(i!=0)
			{
				i-=1;
				delete _items[i];
				_items.removeAt(i);
			}
			else if(_items.size()>1)
			{
				delete _items[i+1];
				_items.removeAt(i+1);
			}
			item->scene()->removeItem(item);
			_items.removeAt(i);
			break;
		}
	}
}

void UPlotCurve::clear()
{
#if PRINT_DEBUG
	ULOGGER_DEBUG("%s", this->name().toStdString().c_str());
#endif
	qDeleteAll(_rootItem->childItems());
	_items.clear();
}

void UPlotCurve::setPen(const QPen & pen)
{
	_pen = pen;
	for(int i=1; i<_items.size(); i+=2)
	{
		((QGraphicsLineItem*) _items.at(i))->setPen(_pen);
	}
}

void UPlotCurve::setBrush(const QBrush & brush)
{
	_brush = brush;
	ULOGGER_WARN("Not used...");
}

void UPlotCurve::setItemsColor(const QColor & color)
{
	if(color.isValid())
	{
		_itemsColor.setRgb(color.red(), color.green(), color.blue(), _itemsColor.alpha());
		for(int i=0; i<_items.size(); i+=2)
		{
			QPen pen = ((UPlotItem*) _items.at(i))->pen();
			pen.setColor(_itemsColor);
			((UPlotItem*) _items.at(i))->setPen(pen);
		}
	}
}

void UPlotCurve::update(qreal scaleX, qreal scaleY, qreal offsetX, qreal offsetY, qreal xDir, qreal yDir, int maxItemsKept)
{
	//ULOGGER_DEBUG("scaleX=%f, scaleY=%f, offsetX=%f, offsetY=%f, xDir=%d, yDir=%d, _plot->scene()->width()=%f, _plot->scene()->height=%f", scaleX, scaleY, offsetX, offsetY, xDir, yDir,_plot->scene()->width(),_plot->scene()->height());
	//make sure direction values are 1 or -1
	xDir<0?xDir=-1:xDir=1;
	yDir<0?yDir=-1:yDir=1;

	bool hide = false;
	int j=0;
	for(int i=_items.size()-1; i>=0; --i)
	{
		if(i%2 == 0)
		{
			UPlotItem * item = (UPlotItem *)_items.at(i);
			if(hide)
			{
				if(maxItemsKept == 0 || j <= maxItemsKept)
				{
					// if not visible, stop looping... all other items are normally already hidden
					if(!item->isVisible())
					{
						break;
					}
					item->setVisible(false);
				}
				else
				{
					//remove the item with his line
					i = this->removeItem(i);
				}
			}
			else
			{
				QPointF newPos(((xDir*item->data().x()+offsetX)*scaleX-item->rect().width()/2.0f),
							   ((yDir*item->data().y()+offsetY)*scaleY-item->rect().width()/2.0f));
				if(!item->isVisible())
				{
					item->setVisible(true);
				}
				item->setPos(newPos);
			}
			++j;
		}
		else
		{
			if(hide)
			{
				_items.at(i)->setVisible(false);
			}
			else
			{
				UPlotItem * from = (UPlotItem *)_items.at(i-1);
				UPlotItem * to = (UPlotItem *)_items.at(i+1);
				QGraphicsLineItem * lineItem = (QGraphicsLineItem *)_items.at(i);
				lineItem->setLine((xDir*from->data().x()+offsetX)*scaleX,
								(yDir*from->data().y()+offsetY)*scaleY,
								(xDir*to->data().x()+offsetX)*scaleX,
								(yDir*to->data().y()+offsetY)*scaleY);
				if(!lineItem->isVisible())
				{
					lineItem->setVisible(true);
				}
				//Don't update not visible items
				// (Detect also if the curve goes forward or backward)
				QLineF line = lineItem->line();
				if((line.x1() <= line.x2() && line.x2() < 0-((line.x2() - line.x1()))) ||
					(line.x1() > line.x2() && line.x2() > lineItem->scene()->sceneRect().width() + ((line.x1() - line.x2()))))
				{
					hide = true;
				}

			}
		}
	}

}

void UPlotCurve::draw(QPainter * painter, const QRect & limits)
{
	if(painter)
	{
		for(int i=_items.size()-1; i>=0 && _items.at(i)->isVisible(); i-=2)
		{
			//plotItem
			const UPlotItem * item = (const UPlotItem *)_items.at(i);
			int x = (int)item->x();
			if(x<0)
			{
				break;
			}

			// draw line in first
			if(i-1>=0)
			{
				//lineItem
				const QGraphicsLineItem * lineItem = (const QGraphicsLineItem *)_items.at(i-1);
				QLine line = lineItem->line().toLine();
				if(limits.contains(line.p1()) || limits.contains(line.p2()))
				{
					QPointF intersection;
					QLineF::IntersectType type;
					type = lineItem->line().intersect(QLineF(limits.topLeft(), limits.bottomLeft()), &intersection);
					if(type == QLineF::BoundedIntersection)
					{
						!limits.contains(line.p1())?line.setP1(intersection.toPoint()):line.setP2(intersection.toPoint());
					}
					else
					{
						type = lineItem->line().intersect(QLineF(limits.topLeft(), limits.topRight()), &intersection);
						if(type == QLineF::BoundedIntersection)
						{
							!limits.contains(line.p1())?line.setP1(intersection.toPoint()):line.setP2(intersection.toPoint());
						}
						else
						{
							type = lineItem->line().intersect(QLineF(limits.bottomLeft(), limits.bottomRight()), &intersection);
							if(type == QLineF::BoundedIntersection)
							{
								!limits.contains(line.p1())?line.setP1(intersection.toPoint()):line.setP2(intersection.toPoint());
							}
							else
							{
								type = lineItem->line().intersect(QLineF(limits.topRight(), limits.bottomRight()), &intersection);
								if(type == QLineF::BoundedIntersection)
								{
									!limits.contains(line.p1())?line.setP1(intersection.toPoint()):line.setP2(intersection.toPoint());
								}
							}
						}
					}
					painter->save();
					painter->setPen(this->pen());
					painter->setBrush(this->brush());
					painter->drawLine(line);
					painter->restore();
				}
			}

			/*if(limits.contains(item->pos().toPoint()) && limits.contains((item->pos() + QPointF(item->rect().width(), item->rect().height())).toPoint()))
			{
				painter->save();
				painter->setPen(QPen(_itemsColor));
				painter->drawEllipse(item->pos()+QPointF(item->rect().width()/2, item->rect().height()/2), (int)item->rect().width()/2, (int)item->rect().height()/2);
				painter->restore();
			}*/
		}
	}
}

int UPlotCurve::itemsSize() const
{
	return _items.size();
}

QPointF UPlotCurve::getItemData(int index)
{
	QPointF data;
	//make sure the index point to a PlotItem {PlotItem, line, PlotItem, line...}
	if(index>=0 && index < _items.size() && index % 2 == 0 )
	{
		data = ((UPlotItem*)_items.at(index))->data();
	}
	else
	{
		ULOGGER_ERROR("Wrong index, not pointing on a PlotItem");
	}
	return data;
}

void UPlotCurve::setVisible(bool visible)
{
	_visible = visible;
	for(int i=0; i<_items.size(); ++i)
	{
		_items.at(i)->setVisible(visible);
	}
}

void UPlotCurve::setXIncrement(qreal increment)
{
	_xIncrement = increment;
}

void UPlotCurve::setXStart(qreal val)
{
	_xStart = val;
}

void UPlotCurve::setData(QVector<UPlotItem*> & data)
{
	this->clear();
	for(int i = 0; i<data.size(); ++i)
	{
		this->addValue(data[i]);
	}
}

void UPlotCurve::setData(const QVector<qreal> & x, const QVector<qreal> & y)
{
	if(x.size() == y.size())
	{
		//match the size of the current data
		int margin = int((_items.size()+1)/2) - x.size();
		while(margin < 0)
		{
			UPlotItem * newItem = new UPlotItem(0, 0, 2);
			this->_addValue(newItem);
			++margin;
		}
		while(margin > 0)
		{
			this->removeItem(0);
			--margin;
		}

		// update values
		int index = 0;
		QVector<qreal>::const_iterator i=x.begin();
		QVector<qreal>::const_iterator j=y.begin();
		for(; i!=x.end() && j!=y.end(); ++i, ++j, index+=2)
		{
			((UPlotItem*)_items[index])->setData(QPointF(*i, *j));
		}

		//reset minMax, this will force the plot to update the axes
		this->updateMinMax();
		Q_EMIT dataChanged(this);
	}
	else if(y.size()>0 && x.size()==0)
	{
		this->setData(y);
	}
	else
	{
		ULOGGER_ERROR("Data vectors have not the same size.");
	}
}

void UPlotCurve::setData(const std::vector<qreal> & x, const std::vector<qreal> & y)
{
	if(x.size() == y.size())
	{
		//match the size of the current data
		int margin = int((_items.size()+1)/2) - int(x.size());
		while(margin < 0)
		{
			UPlotItem * newItem = new UPlotItem(0, 0, 2);
			this->_addValue(newItem);
			++margin;
		}
		while(margin > 0)
		{
			this->removeItem(0);
			--margin;
		}

		// update values
		int index = 0;
		std::vector<qreal>::const_iterator i=x.begin();
		std::vector<qreal>::const_iterator j=y.begin();
		for(; i!=x.end() && j!=y.end(); ++i, ++j, index+=2)
		{
			((UPlotItem*)_items[index])->setData(QPointF(*i, *j));
		}

		//reset minMax, this will force the plot to update the axes
		this->updateMinMax();
		Q_EMIT dataChanged(this);
	}
	else if(y.size()>0 && x.size()==0)
	{
		this->setData(y);
	}
	else
	{
		ULOGGER_ERROR("Data vectors have not the same size.");
	}
}

void UPlotCurve::setData(const QVector<qreal> & y)
{
	this->setData(y.toStdVector());
}

void UPlotCurve::setData(const std::vector<qreal> & y)
{
	//match the size of the current data
	int margin = int((_items.size()+1)/2) - int(y.size());
	while(margin < 0)
	{
		UPlotItem * newItem = new UPlotItem(0, 0, 2);
		this->_addValue(newItem);
		++margin;
	}
	while(margin > 0)
	{
		this->removeItem(0);
		--margin;
	}

	// update values
	int index = 0;
	qreal x = 0;
	std::vector<qreal>::const_iterator j=y.begin();
	for(; j!=y.end(); ++j, index+=2)
	{
		((UPlotItem*)_items[index])->setData(QPointF(x++, *j));
	}

	//reset minMax, this will force the plot to update the axes
	this->updateMinMax();
	Q_EMIT dataChanged(this);
}

void UPlotCurve::getData(QVector<qreal> & x, QVector<qreal> & y) const
{
	x.clear();
	y.clear();
	if(_items.size())
	{
		x.resize((_items.size()-1)/2+1);
		y.resize(x.size());
		int j=0;
		for(int i=0; i<_items.size(); i+=2)
		{
			x[j] = ((UPlotItem*)_items.at(i))->data().x();
			y[j++] = ((UPlotItem*)_items.at(i))->data().y();
		}
	}
}

void UPlotCurve::getData(QMap<qreal,qreal> & data) const
{
	data.clear();
	if(_items.size())
	{
		for(int i=0; i<_items.size(); i+=2)
		{
			data.insert(((UPlotItem*)_items.at(i))->data().x(), ((UPlotItem*)_items.at(i))->data().y());
		}
	}
}





UPlotCurveThreshold::UPlotCurveThreshold(const QString & name, qreal thesholdValue, Qt::Orientation orientation, QObject * parent) :
	UPlotCurve(name, parent),
	_orientation(orientation)
{
	if(_orientation == Qt::Horizontal)
	{
		this->addValue(0, thesholdValue);
		this->addValue(1, thesholdValue);
	}
	else
	{
		this->addValue(thesholdValue, 0);
		this->addValue(thesholdValue, 1);
	}
}

UPlotCurveThreshold::~UPlotCurveThreshold()
{

}

void UPlotCurveThreshold::setThreshold(qreal threshold)
{
#if PRINT_DEBUG
	ULOGGER_DEBUG("%f", threshold);
#endif
	if(_items.size() == 3)
	{
		UPlotItem * item = 0;
		if(_orientation == Qt::Horizontal)
		{
			item = (UPlotItem*)_items.at(0);
			item->setData(QPointF(item->data().x(), threshold));
			item = (UPlotItem*)_items.at(2);
			item->setData(QPointF(item->data().x(), threshold));
		}
		else
		{
			item = (UPlotItem*)_items.at(0);
			item->setData(QPointF(threshold, item->data().y()));
			item = (UPlotItem*)_items.at(2);
			item->setData(QPointF(threshold, item->data().y()));
		}
	}
	else
	{
		ULOGGER_ERROR("A threshold must has only 3 items (1 PlotItem + 1 QGraphicsLineItem + 1 PlotItem)");
	}
}

void UPlotCurveThreshold::setOrientation(Qt::Orientation orientation)
{
	if(_orientation != orientation)
	{
		_orientation = orientation;
		if(_items.size() == 3)
		{
			UPlotItem * item = 0;
			item = (UPlotItem*)_items.at(0);
			item->setData(QPointF(item->data().y(), item->data().x()));
			item = (UPlotItem*)_items.at(2);
			item->setData(QPointF(item->data().y(), item->data().x()));
		}
		else
		{
			ULOGGER_ERROR("A threshold must has only 3 items (1 PlotItem + 1 QGraphicsLineItem + 1 PlotItem)");
		}
	}
}

void UPlotCurveThreshold::update(qreal scaleX, qreal scaleY, qreal offsetX, qreal offsetY, qreal xDir, qreal yDir, int maxItemsKept)
{
	if(_items.size() == 3)
	{
		if(_plot)
		{
			UPlotItem * item = 0;
			if(_orientation == Qt::Horizontal)
			{
				//(xDir*item->data().x()+offsetX)*scaleX
				item = (UPlotItem*)_items.at(0);
				item->setData(QPointF(-(offsetX-item->rect().width()/scaleX)/xDir, item->data().y()));
				item = (UPlotItem*)_items.at(2);
				item->setData(QPointF( (_plot->sceneRect().width()/scaleX-(offsetX+item->rect().width()/scaleX))/xDir, item->data().y()));
			}
			else
			{
				item = (UPlotItem*)_items.at(0);
				item->setData(QPointF(item->data().x(), -(offsetY-item->rect().height()/scaleY)/yDir));
				item = (UPlotItem*)_items.at(2);
				item->setData(QPointF(item->data().x(), (_plot->sceneRect().height()/scaleY-(offsetY+item->rect().height()/scaleY))/yDir));
			}
			this->updateMinMax();
		}
	}
	else
	{
		ULOGGER_ERROR("A threshold must has only 3 items (1 PlotItem + 1 QGraphicsLineItem + 1 PlotItem)");
	}
	UPlotCurve::update(scaleX, scaleY, offsetX, offsetY, xDir, yDir, maxItemsKept);
}







UPlotAxis::UPlotAxis(Qt::Orientation orientation, qreal min, qreal max, QWidget * parent) :
	QWidget(parent),
	_orientation(orientation),
	_min(0),
	_max(0),
	_count(0),
	_step(0),
	_reversed(false),
	_gradMaxDigits(4),
	_border(0)
{
	if(_orientation == Qt::Vertical)
	{
		_reversed = true; // default bottom->up
	}
#ifdef _WIN32
	this->setMinimumSize(15, 25);
#else
	this->setMinimumSize(15, 25);
#endif
	this->setAxis(min, max); // this initialize all attributes
}

UPlotAxis::~UPlotAxis()
{
#if PRINT_DEBUG
	ULOGGER_DEBUG("");
#endif
}

// Vertical :bottom->up, horizontal :right->left
void UPlotAxis::setReversed(bool reversed)
{
	if(_reversed != reversed)
	{
		qreal min = _min;
		_min = _max;
		_max = min;
	}
	_reversed = reversed;
}

void UPlotAxis::setAxis(qreal & min, qreal & max)
{
	int borderMin = 0;
	int borderMax = 0;
	if(_orientation == Qt::Vertical)
	{
		borderMin = borderMax = this->fontMetrics().height()/2;
	}
	else
	{
		borderMin = this->fontMetrics().width(QString::number(_min,'g',_gradMaxDigits))/2;
		borderMax = this->fontMetrics().width(QString::number(_max,'g',_gradMaxDigits))/2;
	}
	int border = borderMin>borderMax?borderMin:borderMax;
	int borderDelta;
	int length;
	if(_orientation == Qt::Vertical)
	{
		length = (this->height()-border*2);
	}
	else
	{
		length = (this->width()-border*2);
	}

	if(length <= 70)
	{
		_count = 5;
	}
	else if(length <= 175)
	{
		_count = 10;
	}
	else if(length <= 350)
	{
		_count = 20;
	}
	else if(length <= 700)
	{
		_count = 40;
	}
	else if(length <= 1000)
	{
		_count = 60;
	}
	else if(length <= 1300)
	{
		_count = 80;
	}
	else
	{
		_count = 100;
	}

	// Rounding min and max
	if(min != max)
	{
		qreal mul = 1;
		qreal rangef = max - min;
		int countStep = _count/5;
		qreal val;
		for(int i=0; i<6; ++i)
		{
			val = (rangef/qreal(countStep)) * mul;
			if( val >= 1.0f && val < 10.0f)
			{
				break;
			}
			else if(val<1)
			{
				mul *= 10.0f;
			}
			else
			{
				mul /= 10.0f;
			}
		}
		//ULOGGER_DEBUG("min=%f, max=%f", min, max);
		int minR = min*mul-0.9;
		int maxR = max*mul+0.9;
		min = qreal(minR)/mul;
		max = qreal(maxR)/mul;
		//ULOGGER_DEBUG("mul=%f, minR=%d, maxR=%d,countStep=%d", mul, minR, maxR, countStep);
	}

	_min = min;
	_max = max;

	if(_reversed)
	{
		_min = _max;
		_max = min;
	}

	if(_orientation == Qt::Vertical)
	{
		_step = length/_count;
		borderDelta = length - (_step*_count);
	}
	else
	{
		_step = length/_count;
		borderDelta = length - (_step*_count);
	}

	if(borderDelta%2 != 0)
	{
		borderDelta+=1;
	}

	_border = border + borderDelta/2;

	//Resize estimation
	if(_orientation == Qt::Vertical)
	{
		int minWidth = 0;
		for (int i = 0; i <= _count; i+=5)
		{
			QString n(QString::number(_min + (i/5)*((_max-_min)/(_count/5)),'g',_gradMaxDigits));
			if(this->fontMetrics().width(n) > minWidth)
			{
				minWidth = this->fontMetrics().width(n);
			}
		}
		this->setMinimumWidth(15+minWidth);
	}
}

void UPlotAxis::paintEvent(QPaintEvent * event)
{
	QPainter painter(this);
	if(_orientation == Qt::Vertical)
	{
		painter.translate(0, _border);
		for (int i = 0; i <= _count; ++i)
		{
			if(i%5 == 0)
			{
				painter.drawLine(this->width(), 0, this->width()-10, 0);
				QLabel n(QString::number(_min + (i/5)*((_max-_min)/(_count/5)),'g',_gradMaxDigits));
				painter.drawText(this->width()-(12+n.sizeHint().width()), n.sizeHint().height()/2-2, n.text());
			}
			else
			{
				painter.drawLine(this->width(), 0, this->width()-5, 0);
			}
			painter.translate(0, _step);
		}
	}
	else
	{
		painter.translate(_border, 0);
		for (int i = 0; i <= _count; ++i)
		{
			if(i%5 == 0)
			{
				painter.drawLine(0, 0, 0, 10);
				QLabel n(QString::number(_min + (i/5)*((_max-_min)/(_count/5)),'g',_gradMaxDigits));
				painter.drawText(-(n.sizeHint().width()/2)+1, 22, n.text());
			}
			else
			{
				painter.drawLine(0, 0, 0, 5);
			}
			painter.translate(_step, 0);
		}
	}
}




UPlotLegendItem::UPlotLegendItem(UPlotCurve * curve, QWidget * parent) :
		QPushButton(parent),
		_curve(curve)
{
	QString nameSpaced = curve->name();
	nameSpaced.replace('_', ' ');
	this->setText(nameSpaced);

	this->setIcon(QIcon(this->createSymbol(curve->pen(), curve->brush())));
	this->setIconSize(QSize(25,20));

	_aChangeText = new QAction(tr("Change text..."), this);
	_aResetText = new QAction(tr("Reset text..."), this);
	_aChangeColor = new QAction(tr("Change color..."), this);
	_aCopyToClipboard = new QAction(tr("Copy curve data to clipboard"), this);
	_aShowStdDevMeanMax = new QAction(tr("Show %1, %2, max").arg(QChar(0xbc, 0x03)).arg(QChar(0xc3, 0x03)), this);
	_aShowStdDevMeanMax->setCheckable(true);
	_aMoveUp = new QAction(tr("Move up"), this);
	_aMoveDown = new QAction(tr("Move down"), this);
	_aRemoveCurve = new QAction(tr("Remove this curve"), this);
	_menu = new QMenu(tr("Curve"), this);
	_menu->addAction(_aChangeText);
	_menu->addAction(_aResetText);
	_menu->addAction(_aChangeColor);
	_menu->addAction(_aCopyToClipboard);
	_menu->addAction(_aShowStdDevMeanMax);
	_menu->addSeparator();
	_menu->addAction(_aMoveUp);
	_menu->addAction(_aMoveDown);
	_menu->addSeparator();
	_menu->addAction(_aRemoveCurve);
}

UPlotLegendItem::~UPlotLegendItem()
{

}
void UPlotLegendItem::contextMenuEvent(QContextMenuEvent * event)
{
	QAction * action = _menu->exec(event->globalPos());
	if(action == _aChangeText)
	{
		bool ok;
		QString text = QInputDialog::getText(this, _aChangeText->text(), tr("Name :"), QLineEdit::Normal, this->text(), &ok);
		if(ok && !text.isEmpty())
		{
			this->setText(text);
		}
	}
	else if(action == _aResetText)
	{
		if(_curve)
		{
			this->setText(_curve->name());
		}
	}
	else if(action == _aChangeColor)
	{
		if(_curve)
		{
			QPen pen = _curve->pen();
			QColor color = QColorDialog::getColor(pen.color(), this);
			if(color.isValid())
			{
				pen.setColor(color);
				_curve->setPen(pen);
				this->setIcon(QIcon(this->createSymbol(_curve->pen(), _curve->brush())));
			}
		}
	}
	else if (action == _aCopyToClipboard)
	{
		if(_curve)
		{
			QVector<qreal> x;
			QVector<qreal> y;
			_curve->getData(x, y);
			QString text;
			text.append("x");
			text.append('\t');
			text.append(_curve->name());
			text.append('\n');
			for(int i=0; i<x.size(); ++i)
			{
				text.append(QString::number(x[i]));
				text.append('\t');
				text.append(QString::number(y[i]));
				if(i+1<x.size())
				{
					text.append('\n');
				}
			}
			QClipboard * clipboard = QApplication::clipboard();
			clipboard->setText(text);
		}
	}
	else if(action == _aShowStdDevMeanMax)
	{
		showStdDevMeanMax(_aShowStdDevMeanMax->isChecked());
	}
	else if(action == _aRemoveCurve)
	{
		Q_EMIT legendItemRemoved(_curve);
	}
	else if(action == _aMoveUp)
	{
		Q_EMIT moveUpRequest(this);
	}
	else if(action == _aMoveDown)
	{
		Q_EMIT moveDownRequest(this);
	}
}

void UPlotLegendItem::showStdDevMeanMax(bool shown)
{
	_aShowStdDevMeanMax->setChecked(shown);
	if(shown)
	{
		connect(_curve, SIGNAL(dataChanged(const UPlotCurve *)), this, SLOT(updateStdDevMeanMax()));
		updateStdDevMeanMax();
	}
	else
	{
		disconnect(_curve, SIGNAL(dataChanged(const UPlotCurve *)), this, SLOT(updateStdDevMeanMax()));
		QString nameSpaced = _curve->name();
		nameSpaced.replace('_', ' ');
		this->setText(nameSpaced);
	}
}

QPixmap UPlotLegendItem::createSymbol(const QPen & pen, const QBrush & brush)
{
	QPixmap pixmap(50, 50);
	pixmap.fill(Qt::transparent);
	QPainter painter(&pixmap);
	QPen p = pen;
	p.setWidthF(4.0);
	painter.setPen(p);
	painter.drawLine(0.0, 25.0, 50.0, 25.0);
	return pixmap;
}

void UPlotLegendItem::updateStdDevMeanMax()
{
	QVector<qreal> x, y;
	_curve->getData(x, y);
	qreal mean = uMean(y.data(), y.size());
	qreal stdDev = std::sqrt(uVariance(y.data(), y.size(), mean));
	qreal max = uMax(y.data(), y.size());
	QString nameSpaced = _curve->name();
	nameSpaced.replace('_', ' ');
	nameSpaced += QString("\n(%1=%2, %3=%4, max=%5, n=%6)").arg(QChar(0xbc, 0x03)).arg(QString::number(mean, 'f', 3)).arg(QChar(0xc3, 0x03)).arg(QString::number(stdDev, 'f', 3)).arg(QString::number(max, 'f', 3)).arg(y.size());
	this->setText(nameSpaced);
}






UPlotLegend::UPlotLegend(QWidget * parent) :
	QWidget(parent),
	_flat(true)
{
	//menu
	_aUseFlatButtons = new QAction(tr("Use flat buttons"), this);
	_aUseFlatButtons->setCheckable(true);
	_aUseFlatButtons->setChecked(_flat);
	_aCopyAllCurvesToClipboard = new QAction(tr("Copy all curve data to clipboard"), this);
	_aShowAllStdDevMeanMax = new QAction(tr("Show all %1, %2, max").arg(QChar(0xbc, 0x03)).arg(QChar(0xc3, 0x03)), this);
	_aShowAllStdDevMeanMax->setCheckable(true);
	_aShowAllStdDevMeanMax->setChecked(false);
	_menu = new QMenu(tr("Legend"), this);
	_menu->addAction(_aUseFlatButtons);
	_menu->addAction(_aShowAllStdDevMeanMax);
	_menu->addAction(_aCopyAllCurvesToClipboard);

	_scrollArea = new QScrollArea(this);
	_scrollArea->setWidgetResizable( true );
	_scrollArea->setFrameShape(QFrame::NoFrame);

	this->setLayout(new QVBoxLayout());
	this->layout()->setContentsMargins(0,0,0,0);
	this->layout()->addWidget(_scrollArea);

	QWidget * _scrollAreaWidgetContent = new QWidget();
	_scrollArea->setWidget( _scrollAreaWidgetContent );

	_contentLayout = new QVBoxLayout();
	_scrollAreaWidgetContent->setLayout(_contentLayout);
	_contentLayout->setContentsMargins(0,0,0,0);
	((QVBoxLayout*)_contentLayout)->addStretch(0);
	_contentLayout->setSpacing(0);
}

UPlotLegend::~UPlotLegend()
{
#if PRINT_DEBUG
	ULOGGER_DEBUG("");
#endif
}

void UPlotLegend::setFlat(bool on)
{
	if(_flat != on)
	{
		_flat = on;
		QList<UPlotLegendItem*> items = this->findChildren<UPlotLegendItem*>();
		for(int i=0; i<items.size(); ++i)
		{
			items.at(i)->setFlat(_flat);
			items.at(i)->setChecked(!items.at(i)->isChecked());
		}
		_aUseFlatButtons->setChecked(_flat);
	}
}

void UPlotLegend::addItem(UPlotCurve * curve)
{
	if(curve)
	{
		UPlotLegendItem * legendItem = new UPlotLegendItem(curve, this);
		legendItem->setAutoDefault(false);
		legendItem->setFlat(_flat);
		legendItem->setCheckable(true);
		legendItem->setChecked(false);
		connect(legendItem, SIGNAL(toggled(bool)), this, SLOT(redirectToggled(bool)));
		connect(legendItem, SIGNAL(legendItemRemoved(const UPlotCurve *)), this, SLOT(removeLegendItem(const UPlotCurve *)));
		connect(legendItem, SIGNAL(moveUpRequest(UPlotLegendItem *)), this, SLOT(moveUp(UPlotLegendItem *)));
		connect(legendItem, SIGNAL(moveDownRequest(UPlotLegendItem *)), this, SLOT(moveDown(UPlotLegendItem *)));

		// layout
		QHBoxLayout * hLayout = new QHBoxLayout();
		hLayout->addWidget(legendItem);
		hLayout->addStretch(0);
		hLayout->setMargin(0);

		// add to the legend
		((QVBoxLayout*)_contentLayout)->insertLayout(_contentLayout->count()-1, hLayout);

		_scrollArea->setMinimumWidth(std::min(480, _scrollArea->widget()->sizeHint().width()+QApplication::style()->pixelMetric(QStyle::PM_ScrollBarExtent)));
	}
}

bool UPlotLegend::remove(const UPlotCurve * curve)
{
	QList<UPlotLegendItem *> items = this->findChildren<UPlotLegendItem*>();
	for(int i=0; i<items.size(); ++i)
	{
		if(items.at(i)->curve() == curve)
		{
			delete items.at(i);
			_scrollArea->setMinimumWidth(std::min(480, _scrollArea->widget()->sizeHint().width()+QApplication::style()->pixelMetric(QStyle::PM_ScrollBarExtent)));
			return true;
		}
	}
	return false;
}

void UPlotLegend::removeLegendItem(const UPlotCurve * curve)
{
	if(this->remove(curve))
	{
		Q_EMIT legendItemRemoved(curve);
	}
}

void UPlotLegend::moveUp(UPlotLegendItem * item)
{
	int index = -1;
	QLayoutItem * layoutItem = 0;
	for(int i=0; i<_contentLayout->count(); ++i)
	{
		if(_contentLayout->itemAt(i)->layout() &&
				_contentLayout->itemAt(i)->layout()->indexOf(item) != -1)
		{
			layoutItem = _contentLayout->itemAt(i);
			index = i;
			break;
		}
	}
	if(index > 0 && layoutItem)
	{
		_contentLayout->removeItem(layoutItem);
		QHBoxLayout * hLayout = new QHBoxLayout();
		hLayout->addWidget(layoutItem->layout()->itemAt(0)->widget());
		hLayout->addStretch(0);
		hLayout->setMargin(0);
		((QVBoxLayout*)_contentLayout)->insertLayout(index-1, hLayout);
		delete layoutItem;
		Q_EMIT legendItemMoved(item->curve(), index-1);
	}
}

void UPlotLegend::moveDown(UPlotLegendItem * item)
{
	int index = -1;
	QLayoutItem * layoutItem = 0;
	for(int i=0; i<_contentLayout->count(); ++i)
	{
		if(_contentLayout->itemAt(i)->layout() &&
		   _contentLayout->itemAt(i)->layout()->indexOf(item) != -1)
		{
			layoutItem = _contentLayout->itemAt(i);
			index = i;
			break;
		}
	}
	if(index < _contentLayout->count()-2 && layoutItem)
	{
		_contentLayout->removeItem(layoutItem);
		QHBoxLayout * hLayout = new QHBoxLayout();
		hLayout->addWidget(layoutItem->layout()->itemAt(0)->widget());
		hLayout->addStretch(0);
		hLayout->setMargin(0);
		((QVBoxLayout*)_contentLayout)->insertLayout(index+1, hLayout);
		delete layoutItem;
		Q_EMIT legendItemMoved(item->curve(), index+1);
	}
}

QString UPlotLegend::getAllCurveDataAsText() const
{
	QList<UPlotLegendItem *> items = this->findChildren<UPlotLegendItem*>();
	if(items.size())
	{
		// create common x-axis
		QMap<qreal, qreal> xAxisMap;
		for(int i=0; i<items.size(); ++i)
		{
			QMap<qreal, qreal> data;
			items.at(i)->curve()->getData(data);
			for(QMap<qreal, qreal>::iterator iter=data.begin(); iter!=data.end(); ++iter)
			{
				xAxisMap.insert(iter.key(), iter.value());
			}
		}
		QList<qreal> xAxis = xAxisMap.uniqueKeys();

		QVector<QVector<qreal> > axes;
		for(int i=0; i<items.size(); ++i)
		{
			QMap<qreal, qreal> data;
			items.at(i)->curve()->getData(data);

			QVector<qreal> y(xAxis.size(), std::numeric_limits<qreal>::quiet_NaN());
			// just to make sure that we have the same number of data on each curve, set NAN for unknowns
			int j=0;
			for(QList<qreal>::iterator iter=xAxis.begin(); iter!=xAxis.end(); ++iter)
			{
				if(data.contains(*iter))
				{
					y[j] = data.value(*iter);
				}
				++j;
			}
			axes.push_back(y);
		}
		if(!xAxis.empty())
		{
			axes.push_front(xAxis.toVector());
			QString text;
			text.append('x');
			text.append('\t');
			for(int i=0; i<items.size(); ++i)
			{
				text.append(items.at(i)->curve()->name());
				if(i+1<axes.size())
				{
					text.append('\t');
				}
			}
			text.append('\n');
			for(int i=0; i<axes[0].size(); ++i)
			{
				for(int j=0; j<axes.size(); ++j)
				{
					if(uIsNan(axes[j][i]))
					{
						text.append("NaN"); // NaN is interpreted by default as NaN in MatLab/Octave
					}
					else
					{
						text.append(QString::number(axes[j][i], 'f'));
					}
					if(j+1<axes.size())
					{
						text.append('\t');
					}
				}
				if(i+1<axes[0].size())
				{
					text.append("\n");
				}
			}
			return text;
		}
	}
	return "";
}

void UPlotLegend::contextMenuEvent(QContextMenuEvent * event)
{
	QAction * action = _menu->exec(event->globalPos());
	if(action == _aUseFlatButtons)
	{
		this->setFlat(_aUseFlatButtons->isChecked());
	}
	else if(action == _aCopyAllCurvesToClipboard)
	{
		QString data = getAllCurveDataAsText();
		if(!data.isEmpty())
		{
			QClipboard * clipboard = QApplication::clipboard();
			clipboard->setText(data);
		}
	}
	else if(action == _aShowAllStdDevMeanMax)
	{
		QList<UPlotLegendItem *> items = this->findChildren<UPlotLegendItem*>();
		for(int i=0; i<items.size(); ++i)
		{
			items.at(i)->showStdDevMeanMax(_aShowAllStdDevMeanMax->isChecked());
		}
	}
}

void UPlotLegend::redirectToggled(bool toggled)
{
	if(sender())
	{
		UPlotLegendItem * item = qobject_cast<UPlotLegendItem*>(sender());
		if(item)
		{
			Q_EMIT legendItemToggled(item->curve(), _flat?!toggled:toggled);
		}
	}
}







UOrientableLabel::UOrientableLabel(const QString & text, Qt::Orientation orientation, QWidget * parent) :
	QLabel(text, parent),
	_orientation(orientation)
{
}

UOrientableLabel::~UOrientableLabel()
{
}

QSize UOrientableLabel::sizeHint() const
{
	QSize size = QLabel::sizeHint();
	if (_orientation == Qt::Vertical)
		size.transpose();
	return size;

}

QSize UOrientableLabel::minimumSizeHint() const
{
	QSize size = QLabel::minimumSizeHint();
	if (_orientation == Qt::Vertical)
		size.transpose();
	return size;
}

void UOrientableLabel::setOrientation(Qt::Orientation orientation)
{
	_orientation = orientation;
	switch(orientation)
	{
	case Qt::Horizontal:
		setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
		break;

	case Qt::Vertical:
		setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Minimum);
		break;
	}
}

void UOrientableLabel::paintEvent(QPaintEvent* event)
{
	QPainter p(this);
	QRect r = rect();
	switch (_orientation)
	{
	case Qt::Horizontal:
		break;
	case Qt::Vertical:
		p.rotate(-90);
		p.translate(-height(), 0);
		QSize size = r.size();
		size.transpose();
		r.setSize(size);
		break;
	}
	p.drawText(r, this->alignment() | (this->wordWrap()?Qt::TextWordWrap:0), this->text());
}













UPlot::UPlot(QWidget *parent) :
	QWidget(parent),
	_legend(0),
	_view(0),
	_sceneRoot(0),
	_graphicsViewHolder(0),
	_verticalAxis(0),
	_horizontalAxis(0),
	_penStyleCount(0),
	_maxVisibleItems(-1),
	_title(0),
	_xLabel(0),
	_yLabel(0),
	_refreshRate(0),
	_workingDirectory(QDir::homePath()),
	_lowestRefreshRate(99),
	_autoScreenCaptureFormat("png"),
	_bgColor(Qt::white),
	_menu(0),
	_aShowLegend(0),
	_aShowGrid(0),
	_aKeepAllData(0),
	_aLimit0(0),
	_aLimit10(0),
	_aLimit50(0),
	_aLimit100(0),
	_aLimit500(0),
	_aLimit1000(0),
	_aLimitCustom(0),
	_aAddVerticalLine(0),
	_aAddHorizontalLine(0),
	_aChangeTitle(0),
	_aChangeXLabel(0),
	_aChangeYLabel(0),
	_aChangeBackgroundColor(0),
	_aYLabelVertical(0),
	_aShowRefreshRate(0),
	_aMouseTracking(0),
	_aSaveFigure(0),
	_aAutoScreenCapture(0),
	_aClearData(0),
	_aGraphicsView(0)
{
	this->setupUi();
	this->createActions();
	this->createMenus();

	// This will update actions
	this->showLegend(true);
	this->setGraphicsView(false);
	this->setMaxVisibleItems(0);
	this->showGrid(false);
	this->showRefreshRate(false);
	this->keepAllData(true);

	for(int i=0; i<4; ++i)
	{
		_axisMaximums[i] = 0;
		_axisMaximumsSet[i] = false;
		if(i<2)
		{
			_fixedAxis[i] = false;
		}
	}

	_mouseCurrentPos = _mousePressedPos; // for zooming

	_refreshIntervalTime.start();
	_refreshStartTime.start();
}

UPlot::~UPlot()
{
	_aAutoScreenCapture->setChecked(false);
#if PRINT_DEBUG
	ULOGGER_DEBUG("%s", this->title().toStdString().c_str());
#endif
	this->removeCurves();
}

void UPlot::setupUi()
{
	_legend = new UPlotLegend(this);
	_view = new QGraphicsView(this);
	_view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	_view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	_view->setScene(new QGraphicsScene(0,0,0,0,this));
	_view->setStyleSheet( "QGraphicsView { border-style: none; }" );
	_sceneRoot = _view->scene()->addText("");
	_sceneRoot->setTransform(QTransform::fromTranslate(0, 0), true);
	_graphicsViewHolder = new QWidget(this);
	_graphicsViewHolder->setMinimumSize(100,100);
	_graphicsViewHolder->setMouseTracking(true);
	_verticalAxis = new UPlotAxis(Qt::Vertical, 0, 1, this);
	_horizontalAxis = new UPlotAxis(Qt::Horizontal, 0, 1, this);
	_title = new QLabel("");
	_xLabel = new QLabel("");
	_refreshRate = new QLabel("");
	_yLabel = new UOrientableLabel("");
	_yLabel->setOrientation(Qt::Vertical);
	_title->setAlignment(Qt::AlignCenter);
	_xLabel->setAlignment(Qt::AlignCenter);
	_yLabel->setAlignment(Qt::AlignCenter);
	_refreshRate->setAlignment(Qt::AlignCenter);
	_title->setWordWrap(true);
	_xLabel->setWordWrap(true);
	_yLabel->setWordWrap(true);
	_title->setVisible(false);
	_xLabel->setVisible(false);
	_yLabel->setVisible(false);
	_refreshRate->setVisible(false);

	//layouts
	QVBoxLayout * vLayout = new QVBoxLayout(_graphicsViewHolder);
	vLayout->setContentsMargins(0,0,0,0);
	vLayout->addWidget(_view);

	QGridLayout * grid = new QGridLayout(this);
	grid->setContentsMargins(0,0,0,0);
	grid->addWidget(_title, 0, 2);
	grid->addWidget(_yLabel, 1, 0);
	grid->addWidget(_verticalAxis, 1, 1);
	grid->addWidget(_refreshRate, 2, 1);
	grid->addWidget(_graphicsViewHolder, 1, 2);
	grid->setColumnStretch(2, 1);
	grid->setRowStretch(1, 1);
	grid->addWidget(_horizontalAxis, 2, 2);
	grid->addWidget(_xLabel, 3, 2);
	grid->addWidget(_legend, 1, 3);

	connect(_legend, SIGNAL(legendItemToggled(const UPlotCurve *, bool)), this, SLOT(showCurve(const UPlotCurve *, bool)));
	connect(_legend, SIGNAL(legendItemRemoved(const UPlotCurve *)), this, SLOT(removeCurve(const UPlotCurve *)));
	connect(_legend, SIGNAL(legendItemMoved(const UPlotCurve *, int)), this, SLOT(moveCurve(const UPlotCurve *, int)));
}

void UPlot::createActions()
{
	_aShowLegend = new QAction(tr("Show legend"), this);
	_aShowLegend->setCheckable(true);
	_aShowGrid = new QAction(tr("Show grid"), this);
	_aShowGrid->setCheckable(true);
	_aShowRefreshRate = new QAction(tr("Show refresh rate"), this);
	_aShowRefreshRate->setCheckable(true);
	_aMouseTracking = new QAction(tr("Mouse tracking"), this);
	_aMouseTracking->setCheckable(true);
	_aGraphicsView = new QAction(tr("Graphics view"), this);
	_aGraphicsView->setCheckable(true);
	_aKeepAllData = new QAction(tr("Keep all data"), this);
	_aKeepAllData->setCheckable(true);
	_aLimit0 = new QAction(tr("No maximum items shown"), this);
	_aLimit10 = new QAction(tr("10"), this);
	_aLimit50 = new QAction(tr("50"), this);
	_aLimit100 = new QAction(tr("100"), this);
	_aLimit500 = new QAction(tr("500"), this);
	_aLimit1000 = new QAction(tr("1000"), this);
	_aLimitCustom = new QAction(tr(""), this);
	_aLimit0->setCheckable(true);
	_aLimit10->setCheckable(true);
	_aLimit50->setCheckable(true);
	_aLimit100->setCheckable(true);
	_aLimit500->setCheckable(true);
	_aLimit1000->setCheckable(true);
	_aLimitCustom->setCheckable(true);
	_aLimitCustom->setVisible(false);
	_aAddVerticalLine = new QAction(tr("Vertical line..."), this);
	_aAddHorizontalLine = new QAction(tr("Horizontal line..."), this);
	_aChangeTitle = new QAction(tr("Change title"), this);
	_aChangeXLabel = new QAction(tr("Change X label..."), this);
	_aChangeYLabel = new QAction(tr("Change Y label..."), this);
	_aChangeBackgroundColor = new QAction(tr("Change bg color..."), this);
	_aYLabelVertical = new QAction(tr("Vertical orientation"), this);
	_aYLabelVertical->setCheckable(true);
	_aYLabelVertical->setChecked(true);
	_aSaveFigure = new QAction(tr("Save figure..."), this);
	_aAutoScreenCapture = new QAction(tr("Auto screen capture..."), this);
	_aAutoScreenCapture->setCheckable(true);
	_aClearData = new QAction(tr("Clear data"), this);

	QActionGroup * grpLimit = new QActionGroup(this);
	grpLimit->addAction(_aLimit0);
	grpLimit->addAction(_aLimit10);
	grpLimit->addAction(_aLimit50);
	grpLimit->addAction(_aLimit100);
	grpLimit->addAction(_aLimit500);
	grpLimit->addAction(_aLimit1000);
	grpLimit->addAction(_aLimitCustom);
	_aLimit0->setChecked(true);
}

void UPlot::createMenus()
{
	_menu = new QMenu(tr("Plot"), this);
	_menu->addAction(_aShowLegend);
	_menu->addAction(_aShowGrid);
	_menu->addAction(_aShowRefreshRate);
	_menu->addAction(_aMouseTracking);
	_menu->addAction(_aGraphicsView);
	_menu->addAction(_aKeepAllData);
	_menu->addSeparator()->setStatusTip(tr("Maximum items shown"));
	_menu->addAction(_aLimit0);
	_menu->addAction(_aLimit10);
	_menu->addAction(_aLimit50);
	_menu->addAction(_aLimit100);
	_menu->addAction(_aLimit500);
	_menu->addAction(_aLimit1000);
	_menu->addAction(_aLimitCustom);
	_menu->addSeparator();
	QMenu * addLineMenu = _menu->addMenu(tr("Add line"));
	addLineMenu->addAction(_aAddHorizontalLine);
	addLineMenu->addAction(_aAddVerticalLine);
	_menu->addSeparator();
	_menu->addAction(_aChangeTitle);
	_menu->addAction(_aChangeXLabel);
	QMenu * yLabelMenu = _menu->addMenu(tr("Y label"));
	yLabelMenu->addAction(_aChangeYLabel);
	yLabelMenu->addAction(_aYLabelVertical);
	_menu->addAction(_aChangeBackgroundColor);
	_menu->addAction(_aSaveFigure);
	_menu->addAction(_aAutoScreenCapture);
	_menu->addSeparator();
	_menu->addAction(_aClearData);

}

UPlotCurve * UPlot::addCurve(const QString & curveName, const QColor & color)
{
	// add curve
	UPlotCurve * curve = new UPlotCurve(curveName, this);
	if(color.isValid())
	{
		curve->setPen(color);
	}
	else
	{
		curve->setPen(this->getRandomPenColored());
	}
	this->addCurve(curve);
	return curve;
}

bool UPlot::addCurve(UPlotCurve * curve, bool ownershipTransferred)
{
	if(curve)
	{
#if PRINT_DEBUG
		ULOGGER_DEBUG("Adding curve \"%s\" to plot \"%s\"...", curve->name().toStdString().c_str(), this->title().toStdString().c_str());
#endif
		// only last curve can trigger an update, so disable previous connections
		if(!qobject_cast<UPlotCurveThreshold*>(curve))
		{
			for(int i=_curves.size()-1; i>=0; --i)
			{
				if(!qobject_cast<UPlotCurveThreshold*>(_curves.at(i)))
				{
					disconnect(_curves.at(i), SIGNAL(dataChanged(const UPlotCurve *)), this, SLOT(updateAxis()));
					break;
				}
			}
		}

		// add curve
		_curves.append(curve);
		curve->attach(this);
		curve->setItemsColor(QColor(255-_bgColor.red(), 255-_bgColor.green(), 255-_bgColor.red(), _bgColor.alpha()));
		if(ownershipTransferred)
		{
			curve->setParent(this);
		}
		this->updateAxis(curve);
		curve->setXStart(_axisMaximums[1]);

		connect(curve, SIGNAL(dataChanged(const UPlotCurve *)), this, SLOT(updateAxis()));

		_legend->addItem(curve);

#if PRINT_DEBUG
		ULOGGER_DEBUG("Curve \"%s\" added to plot \"%s\"", curve->name().toStdString().c_str(), this->title().toStdString().c_str());
#endif

		return true;
	}
	else
	{
		ULOGGER_ERROR("The curve is null!");
	}
	return false;
}

QStringList UPlot::curveNames()
{
	QStringList names;
	for(QList<UPlotCurve*>::iterator iter = _curves.begin(); iter!=_curves.end(); ++iter)
	{
		if(*iter)
		{
			names.append((*iter)->name());
		}
	}
	return names;
}

bool UPlot::contains(const QString & curveName)
{
	for(QList<UPlotCurve*>::iterator iter = _curves.begin(); iter!=_curves.end(); ++iter)
	{
		if(*iter && (*iter)->name().compare(curveName) == 0)
		{
			return true;
		}
	}
	return false;
}

QPen UPlot::getRandomPenColored()
{
	int penStyle = 0;
	bool colorNotUsed = false;
	for(int i=0; i<12; ++i)
	{
		QColor tmp((Qt::GlobalColor)((penStyle+i) % 12 + 7 ));
		bool colorAlreadyUsed = false;
		for(QList<UPlotCurve*>::const_iterator iter = _curves.constBegin(); iter!=_curves.constEnd() && !colorAlreadyUsed; ++iter)
		{
			colorAlreadyUsed = (*iter)->pen().color() == tmp;
		}
		if(!colorAlreadyUsed)
		{
			colorNotUsed = true;
			penStyle+=i;
			break;
		}
	}
	if(colorNotUsed)
	{
		_penStyleCount = penStyle;
	}

	return QPen((Qt::GlobalColor)(_penStyleCount++ % 12 + 7 ));
}

void UPlot::replot(QPainter * painter)
{
	if(_maxVisibleItems>0)
	{
		UPlotCurve * c = 0;
		int maxItem = 0;
		// find the curve with the most items
		for(QList<UPlotCurve *>::iterator i=_curves.begin(); i!=_curves.end(); ++i)
		{
			if((*i)->isVisible() && ((UPlotCurve *)(*i))->itemsSize() > maxItem)
			{
				c = *i;
				maxItem = c->itemsSize();
			}
		}
		if(c && (maxItem-1)/2+1 > _maxVisibleItems && _axisMaximums[0] < c->getItemData((c->itemsSize()-1) -_maxVisibleItems*2).x())
		{
			_axisMaximums[0] = c->getItemData((c->itemsSize()-1) -_maxVisibleItems*2).x();
		}
	}

	qreal axis[4] = {0};
	for(int i=0; i<4; ++i)
	{
		axis[i] = _axisMaximums[i];
	}

	_verticalAxis->setAxis(axis[2], axis[3]);
	_horizontalAxis->setAxis(axis[0], axis[1]);
	if(_aGraphicsView->isChecked() && !painter)
	{
		_verticalAxis->update();
		_horizontalAxis->update();
	}

	//ULOGGER_DEBUG("x1=%f, x2=%f, y1=%f, y2=%f", _axisMaximums[0], _axisMaximums[1], _axisMaximums[2], _axisMaximums[3]);

	QRectF newRect(0,0, _graphicsViewHolder->size().width(), _graphicsViewHolder->size().height());
	_view->scene()->setSceneRect(newRect);
	qreal borderHor = (qreal)_horizontalAxis->border();
	qreal borderVer = (qreal)_verticalAxis->border();

	//grid
	qDeleteAll(hGridLines);
	hGridLines.clear();
	qDeleteAll(vGridLines);
	vGridLines.clear();
	if(_aShowGrid->isChecked())
	{
		// TODO make a PlotGrid class ?
		qreal w = newRect.width()-(borderHor*2);
		qreal h = newRect.height()-(borderVer*2);
		qreal stepH = w / qreal(_horizontalAxis->count());
		qreal stepV = h / qreal(_verticalAxis->count());
		QPen dashPen(Qt::DashLine);
		dashPen.setColor(QColor(255-_bgColor.red(), 255-_bgColor.green(), 255-_bgColor.blue(), 100));
		QPen pen(dashPen.color());
		for(qreal i=0.0f; i*stepV <= h+stepV; i+=5.0f)
		{
			//horizontal lines
			if(!_aGraphicsView->isChecked())
			{
				if(painter)
				{
					painter->save();
					painter->setPen(pen);
					painter->drawLine(0, stepV*i+borderVer+0.5f, borderHor, stepV*i+borderVer+0.5f);

					painter->setPen(dashPen);
					painter->drawLine(borderHor, stepV*i+borderVer+0.5f, w+borderHor, stepV*i+borderVer+0.5f);

					painter->setPen(pen);
					painter->drawLine(w+borderHor, stepV*i+borderVer+0.5f, w+borderHor*2, stepV*i+borderVer+0.5f);
					painter->restore();
				}
			}
			else
			{
				hGridLines.append(new QGraphicsLineItem(0, stepV*i+borderVer, borderHor, stepV*i+borderVer, _sceneRoot));
				hGridLines.last()->setPen(pen);
				hGridLines.append(new QGraphicsLineItem(borderHor, stepV*i+borderVer, w+borderHor, stepV*i+borderVer, _sceneRoot));
				hGridLines.last()->setPen(dashPen);
				hGridLines.append(new QGraphicsLineItem(w+borderHor, stepV*i+borderVer, w+borderHor*2, stepV*i+borderVer, _sceneRoot));
				hGridLines.last()->setPen(pen);
			}
		}
		for(qreal i=0; i*stepH < w+stepH; i+=5.0f)
		{
			//vertical lines
			if(!_aGraphicsView->isChecked())
			{
				if(painter)
				{
					painter->save();
					painter->setPen(pen);
					painter->drawLine(stepH*i+borderHor+0.5f, 0, stepH*i+borderHor+0.5f, borderVer);

					painter->setPen(dashPen);
					painter->drawLine(stepH*i+borderHor+0.5f, borderVer, stepH*i+borderHor+0.5f, h+borderVer);

					painter->setPen(pen);
					painter->drawLine(stepH*i+borderHor+0.5f, h+borderVer, stepH*i+borderHor+0.5f, h+borderVer*2);
					painter->restore();
				}
			}
			else
			{
				vGridLines.append(new QGraphicsLineItem(stepH*i+borderHor, 0, stepH*i+borderHor, borderVer, _sceneRoot));
				vGridLines.last()->setPen(pen);
				vGridLines.append(new QGraphicsLineItem(stepH*i+borderHor, borderVer, stepH*i+borderHor, h+borderVer, _sceneRoot));
				vGridLines.last()->setPen(dashPen);
				vGridLines.append(new QGraphicsLineItem(stepH*i+borderHor, h+borderVer, stepH*i+borderHor, h+borderVer*2, _sceneRoot));
				vGridLines.last()->setPen(pen);
			}
		}
	}

	// curves
	qreal scaleX = 1;
	qreal scaleY = 1;
	qreal den = 0;
	den = axis[1] - axis[0];
	if(den != 0)
	{
		scaleX = (newRect.width()-(borderHor*2)) / den;
	}
	den = axis[3] - axis[2];
	if(den != 0)
	{
		scaleY = (newRect.height()-(borderVer*2)) / den;
	}
	for(QList<UPlotCurve *>::iterator i=_curves.begin(); i!=_curves.end(); ++i)
	{
		if((*i)->isVisible())
		{
			qreal xDir = 1.0f;
			qreal yDir = -1.0f;
			(*i)->update(scaleX,
						scaleY,
						xDir<0?axis[1]+borderHor/scaleX:-(axis[0]-borderHor/scaleX),
						yDir<0?axis[3]+borderVer/scaleY:-(axis[2]-borderVer/scaleY),
						xDir,
						yDir,
						_aKeepAllData->isChecked()?0:_maxVisibleItems);
			if(painter)
			{
				(*i)->draw(painter, QRect(0,0,_graphicsViewHolder->rect().width(), _graphicsViewHolder->rect().height()));
			}
		}
	}

	// Update refresh rate
	if(_aShowRefreshRate->isChecked())
	{
		int refreshRate = qRound(1000.0f/qreal(_refreshIntervalTime.restart()));
		if(refreshRate > 0 && refreshRate < _lowestRefreshRate)
		{
			_lowestRefreshRate = refreshRate;
		}
		// Refresh the label only after each 1000 ms
		if(_refreshStartTime.elapsed() > 1000)
		{
			_refreshRate->setText(QString::number(_lowestRefreshRate));
			_lowestRefreshRate = 99;
			_refreshStartTime.start();
		}
	}
}

void UPlot::setFixedXAxis(qreal x1, qreal x2)
{
	_fixedAxis[0] = true;
	_axisMaximums[0] = x1;
	_axisMaximums[1] = x2;
}

void UPlot::setFixedYAxis(qreal y1, qreal y2)
{
	_fixedAxis[1] = true;
	_axisMaximums[2] = y1;
	_axisMaximums[3] = y2;
}

void UPlot::updateAxis(const UPlotCurve * curve)
{
	if(curve && curve->isVisible() && curve->itemsSize() && curve->isMinMaxValid())
	{
		const QVector<qreal> & minMax = curve->getMinMax();
		//ULOGGER_DEBUG("x1=%f, x2=%f, y1=%f, y2=%f", minMax[0], minMax[1], minMax[2], minMax[3]);
		if(minMax.size() != 4)
		{
			ULOGGER_ERROR("minMax size != 4 ?!?");
			return;
		}
		this->updateAxis(minMax[0], minMax[1], minMax[2], minMax[3]);
		_aGraphicsView->isChecked()?this->replot(0):this->update();
	}
}

bool UPlot::updateAxis(qreal x1, qreal x2, qreal y1, qreal y2)
{
	bool modified = false;
	modified = updateAxis(x1,y1);
	if(!modified)
	{
		modified = updateAxis(x2,y2);
	}
	else
	{
		updateAxis(x2,y2);
	}
	return modified;
}

bool UPlot::updateAxis(qreal x, qreal y)
{
	//ULOGGER_DEBUG("x=%f, y=%f", x,y);
	bool modified = false;
	if(!_fixedAxis[0] && (!_axisMaximumsSet[0] || x < _axisMaximums[0]))
	{
		_axisMaximums[0] = x;
		_axisMaximumsSet[0] = true;
		modified = true;
	}

	if(!_fixedAxis[0] && (!_axisMaximumsSet[1] || x > _axisMaximums[1]))
	{
		_axisMaximums[1] = x;
		_axisMaximumsSet[1] = true;
		modified = true;
	}

	if(!_fixedAxis[1] && (!_axisMaximumsSet[2] || y < _axisMaximums[2]))
	{
		_axisMaximums[2] = y;
		_axisMaximumsSet[2] = true;
		modified = true;
	}

	if(!_fixedAxis[1] && (!_axisMaximumsSet[3] || y > _axisMaximums[3]))
	{
		_axisMaximums[3] = y;
		_axisMaximumsSet[3] = true;
		modified = true;
	}

	return modified;
}

void UPlot::updateAxis()
{
	//Reset the axis
	for(int i=0; i<4; ++i)
	{
		if((!_fixedAxis[0] && i<2) || (!_fixedAxis[1] && i>=2))
		{
			_axisMaximums[i] = 0;
			_axisMaximumsSet[i] = false;
		}
	}

	for(int i=0; i<_curves.size(); ++i)
	{
		if(_curves.at(i)->isVisible() && _curves.at(i)->isMinMaxValid())
		{
			const QVector<qreal> & minMax = _curves.at(i)->getMinMax();
			this->updateAxis(minMax[0], minMax[1], minMax[2], minMax[3]);
		}
	}

	_aGraphicsView->isChecked()?this->replot(0):this->update();

	this->captureScreen();
}

void UPlot::paintEvent(QPaintEvent * event)
{
#if PRINT_DEBUG
	UDEBUG("");
#endif
	if(!_aGraphicsView->isChecked())
	{
		QPainter painter(this);
		painter.translate(_graphicsViewHolder->pos());
		painter.save();
		painter.setBrush(_bgColor);
		painter.setPen(QPen(Qt::NoPen));
		painter.drawRect(_graphicsViewHolder->rect());
		painter.restore();

		this->replot(&painter);

		if(_mouseCurrentPos != _mousePressedPos)
		{
			painter.save();
			int left, top, right, bottom;
			left = _mousePressedPos.x() < _mouseCurrentPos.x() ? _mousePressedPos.x()-_graphicsViewHolder->x():_mouseCurrentPos.x()-_graphicsViewHolder->x();
			top = _mousePressedPos.y() < _mouseCurrentPos.y() ? _mousePressedPos.y()-1-_graphicsViewHolder->y():_mouseCurrentPos.y()-1-_graphicsViewHolder->y();
			right = _mousePressedPos.x() > _mouseCurrentPos.x() ? _mousePressedPos.x()-_graphicsViewHolder->x():_mouseCurrentPos.x()-_graphicsViewHolder->x();
			bottom = _mousePressedPos.y() > _mouseCurrentPos.y() ? _mousePressedPos.y()-_graphicsViewHolder->y():_mouseCurrentPos.y()-_graphicsViewHolder->y();
			if(left <= 0)
			{
				left = 1;
			}
			if(right >= _graphicsViewHolder->width())
			{
				right = _graphicsViewHolder->width()-1;
			}
			if(top <= 0)
			{
				top = 1;
			}
			if(bottom >= _graphicsViewHolder->height())
			{
				bottom = _graphicsViewHolder->height()-1;
			}
			painter.setPen(Qt::NoPen);
			painter.setBrush(QBrush(QColor(255-_bgColor.red(),255-_bgColor.green(),255-_bgColor.blue(),100)));
			painter.drawRect(0, 0, _graphicsViewHolder->width(), top);
			painter.drawRect(0, top, left, bottom-top);
			painter.drawRect(right, top, _graphicsViewHolder->width()-right, bottom-top);
			painter.drawRect(0, bottom, _graphicsViewHolder->width(), _graphicsViewHolder->height()-bottom);
			painter.restore();
		}
	}
	else
	{
		QWidget::paintEvent(event);
	}
}

void UPlot::resizeEvent(QResizeEvent * event)
{
	if(_aGraphicsView->isChecked())
	{
		this->replot(0);
	}
	QWidget::resizeEvent(event);
}

void UPlot::mousePressEvent(QMouseEvent * event)
{
	_mousePressedPos = event->pos();
	_mouseCurrentPos = _mousePressedPos;
	QWidget::mousePressEvent(event);
}

void UPlot::mouseMoveEvent(QMouseEvent * event)
{
	if(!_aGraphicsView->isChecked())
	{
		if(!(QApplication::mouseButtons() & Qt::LeftButton))
		{
			_mousePressedPos = _mouseCurrentPos;
		}

		qreal x,y;
		if(mousePosToValue(event->pos(), x ,y))
		{
			if(QApplication::mouseButtons() & Qt::LeftButton)
			{
				_mouseCurrentPos = event->pos();
				this->update();
			}

			int xPos = event->pos().x() - _graphicsViewHolder->pos().x();
			int yPos = event->pos().y() - _graphicsViewHolder->pos().y();
			if((QApplication::mouseButtons() & Qt::LeftButton) ||
			   (_aMouseTracking->isChecked() && xPos>=0 && yPos>=0 && xPos<_graphicsViewHolder->width() && yPos<_graphicsViewHolder->height()))
			{
				QToolTip::showText(event->globalPos(), QString("%1,%2").arg(x).arg(y));
			}
			else
			{
				QToolTip::hideText();
			}
		}
		else
		{
			QToolTip::hideText();
		}
	}
	QWidget::mouseMoveEvent(event);
}

void UPlot::mouseReleaseEvent(QMouseEvent * event)
{
	if(_mousePressedPos != _mouseCurrentPos)
	{
		int left,top,bottom,right;

		left = _mousePressedPos.x() < _mouseCurrentPos.x() ? _mousePressedPos.x():_mouseCurrentPos.x();
		top = _mousePressedPos.y() < _mouseCurrentPos.y() ? _mousePressedPos.y():_mouseCurrentPos.y();
		right = _mousePressedPos.x() > _mouseCurrentPos.x() ? _mousePressedPos.x():_mouseCurrentPos.x();
		bottom = _mousePressedPos.y() > _mouseCurrentPos.y() ? _mousePressedPos.y():_mouseCurrentPos.y();

		if(right - left > 5 || bottom - top > 5)
		{
			qreal axis[4];
			if(mousePosToValue(QPoint(left, top), axis[0], axis[3]) && mousePosToValue(QPoint(right, bottom), axis[1], axis[2]))
			{
#if PRINT_DEBUG
				UDEBUG("resize! new axis = [%f, %f, %f, %f]", axis[0], axis[1], axis[2], axis[3]);
#endif
				//update axis (only if not fixed)
				for(int i=0; i<4; ++i)
				{
					if((!_fixedAxis[0] && i<2) || (!_fixedAxis[1] && i>=2))
					{
						_axisMaximums[i] = axis[i];
					}
				}
				_aGraphicsView->isChecked()?this->replot(0):this->update();
			}
		}
		_mousePressedPos = _mouseCurrentPos;
	}
	QWidget::mouseReleaseEvent(event);
}

void UPlot::mouseDoubleClickEvent(QMouseEvent * event)
{
	this->updateAxis();
	QWidget::mouseDoubleClickEvent(event);
}

bool UPlot::mousePosToValue(const QPoint & pos, qreal & x, qreal & y)
{
	int xPos = pos.x() - _graphicsViewHolder->pos().x() - _horizontalAxis->border();
	int yPos = pos.y() - _graphicsViewHolder->pos().y() - _verticalAxis->border();
	int maxX = _graphicsViewHolder->width() - _horizontalAxis->border()*2;
	int maxY = _graphicsViewHolder->height() - _verticalAxis->border()*2;
	if(maxX == 0 || maxY == 0)
	{
		return false;
	}

	if(xPos < 0)
	{
		xPos = 0;
	}
	else if(xPos > maxX)
	{
		xPos = maxX;
	}

	if(yPos < 0)
	{
		yPos = 0;
	}
	else if(yPos > maxY)
	{
		yPos = maxY;
	}

	//UDEBUG("IN");
	//UDEBUG("x1=%f, x2=%f, y1=%f, y2=%f", _axisMaximums[0], _axisMaximums[1], _axisMaximums[2], _axisMaximums[3]);
	//UDEBUG("border hor=%f ver=%f", (qreal)_horizontalAxis->border(), (qreal)_verticalAxis->border());
	//UDEBUG("rect = %d,%d %d,%d", _graphicsViewHolder->pos().x(), _graphicsViewHolder->pos().y(), _graphicsViewHolder->width(), _graphicsViewHolder->height());
	//UDEBUG("%d,%d", event->pos().x(), event->pos().y());
	//UDEBUG("x/y %d,%d", x, y);
	//UDEBUG("max %d,%d", maxX, maxY);

	//UDEBUG("map %f,%f", x, y);
	x = _axisMaximums[0] + qreal(xPos)*(_axisMaximums[1] - _axisMaximums[0]) / qreal(maxX);
	y = _axisMaximums[2] + qreal(maxY - yPos)*(_axisMaximums[3] - _axisMaximums[2]) / qreal(maxY);
	return true;
}

void UPlot::contextMenuEvent(QContextMenuEvent * event)
{
	QAction * action = _menu->exec(event->globalPos());

	if(!action)
	{
		return;
	}
	else if(action == _aShowLegend)
	{
		this->showLegend(_aShowLegend->isChecked());
	}
	else if(action == _aShowGrid)
	{
		this->showGrid(_aShowGrid->isChecked());
	}
	else if(action == _aShowRefreshRate)
	{
		this->showRefreshRate(_aShowRefreshRate->isChecked());
	}
	else if(action == _aMouseTracking)
	{
		this->trackMouse(_aMouseTracking->isChecked());
	}
	else if(action == _aGraphicsView)
	{
		this->setGraphicsView(_aGraphicsView->isChecked());
	}
	else if(action == _aKeepAllData)
	{
		this->keepAllData(_aKeepAllData->isChecked());
	}
	else if(action == _aLimit0 ||
			action == _aLimit10 ||
			action == _aLimit50 ||
			action == _aLimit100 ||
			action == _aLimit500 ||
			action == _aLimit1000 ||
			action == _aLimitCustom)
	{
		this->setMaxVisibleItems(action->text().toInt());
	}
	else if(action == _aAddVerticalLine || action == _aAddHorizontalLine)
	{
		bool ok;
		QString text = QInputDialog::getText(this, action->text(), tr("New line name :"), QLineEdit::Normal, "", &ok);
		while(ok && text.isEmpty())
		{
			QMessageBox::warning(this, action->text(), tr("The name is not valid or it is already used in this plot."));
			text = QInputDialog::getText(this, action->text(), tr("New line name :"), QLineEdit::Normal, "", &ok);
		}
		if(ok)
		{
			double min = _axisMaximums[2];
			double max = _axisMaximums[3];
			QString axis = "Y";
			if(action == _aAddVerticalLine)
			{
				min = _axisMaximums[0];
				max = _axisMaximums[1];
				axis = "X";
			}
			double value = QInputDialog::getDouble(this,
					action->text(),
					tr("%1 value (min=%2, max=%3):").arg(axis).arg(min).arg(max),
					(min+max)/2,
					-2147483647,
					2147483647,
					4,
					&ok);
			if(ok)
			{
				if(action == _aAddHorizontalLine)
				{
					this->addThreshold(text, value, Qt::Horizontal);
				}
				else
				{
					this->addThreshold(text, value, Qt::Vertical);
				}
			}
		}
	}
	else if(action == _aChangeTitle)
	{
		bool ok;
		QString text = _title->text();
		if(text.isEmpty())
		{
			text = this->objectName();
		}
		text = QInputDialog::getText(this, _aChangeTitle->text(), tr("Title :"), QLineEdit::Normal, text, &ok);
		if(ok)
		{
			this->setTitle(text);
		}
	}
	else if(action == _aChangeXLabel)
	{
		bool ok;
		QString text = QInputDialog::getText(this, _aChangeXLabel->text(), tr("X axis label :"), QLineEdit::Normal, _xLabel->text(), &ok);
		if(ok)
		{
			this->setXLabel(text);
		}
	}
	else if(action == _aChangeYLabel)
	{
		bool ok;
		QString text = QInputDialog::getText(this, _aChangeYLabel->text(), tr("Y axis label :"), QLineEdit::Normal, _yLabel->text(), &ok);
		if(ok)
		{
			this->setYLabel(text, _yLabel->orientation());
		}
	}
	else if(action == _aYLabelVertical)
	{
		this->setYLabel(_yLabel->text(), _aYLabelVertical->isChecked()?Qt::Vertical:Qt::Horizontal);
	}
	else if(action == _aChangeBackgroundColor)
	{
		QColor color = QColorDialog::getColor(_bgColor, this);
		if(color.isValid())
		{
			this->setBackgroundColor(color);
		}
	}
	else if(action == _aSaveFigure)
	{

		QString text;
#ifdef QT_SVG_LIB
		text = QFileDialog::getSaveFileName(this, tr("Save figure to ..."), (QDir::homePath() + "/") + this->title() + ".png", "*.png *.xpm *.jpg *.pdf *.svg");
#else
		text = QFileDialog::getSaveFileName(this, tr("Save figure to ..."), (QDir::homePath() + "/") + this->title() + ".png", "*.png *.xpm *.jpg *.pdf");
#endif
		if(!text.isEmpty())
		{
			bool flatModified = false;
			if(!_legend->isFlat())
			{
				_legend->setFlat(true);
				flatModified = true;
			}

			QPalette p(palette());
			// Set background color to white
			QColor c = p.color(QPalette::Background);
			p.setColor(QPalette::Background, Qt::white);
			setPalette(p);

#ifdef QT_SVG_LIB
			if(QFileInfo(text).suffix().compare("svg") == 0)
			{
				QSvgGenerator generator;
				generator.setFileName(text);
				generator.setSize(this->size());
				QPainter painter;
				painter.begin(&generator);
				this->render(&painter);
				painter.end();
			}
			else
			{
#endif
				if(QFileInfo(text).suffix().compare("pdf") == 0)
				{
					QPrinter printer;
					printer.setOutputFormat(QPrinter::PdfFormat);
					printer.setOutputFileName(text);
					this->render(&printer);
				}
				else
				{
					QPixmap figure = QPixmap::grabWidget(this);
					figure.save(text);
				}
#ifdef QT_SVG_LIB
			}
#endif
			// revert background color
			p.setColor(QPalette::Background, c);
			setPalette(p);

			if(flatModified)
			{
				_legend->setFlat(false);
			}
		}
	}
	else if(action == _aAutoScreenCapture)
	{
		if(_aAutoScreenCapture->isChecked())
		{
			this->selectScreenCaptureFormat();
		}
	}
	else if(action == _aClearData)
	{
		this->clearData();
	}
	else
	{
		ULOGGER_WARN("Unknown action");
	}
}

void UPlot::setWorkingDirectory(const QString & workingDirectory)
{
	if(QDir(_workingDirectory).exists())
	{
		_workingDirectory = workingDirectory;
	}
	else
	{
		ULOGGER_ERROR("The directory \"%s\" doesn't exist", workingDirectory.toStdString().c_str());
	}
}

void UPlot::captureScreen()
{
	if(!_aAutoScreenCapture->isChecked())
	{
		return;
	}
	QString targetDir = _workingDirectory + "/ScreensCaptured";
	QDir dir;
	if(!dir.exists(targetDir))
	{
		dir.mkdir(targetDir);
	}
	targetDir += "/";
	targetDir += this->title().replace(" ", "_");
	if(!dir.exists(targetDir))
	{
		dir.mkdir(targetDir);
	}
	targetDir += "/";
	QString name = (QDateTime::currentDateTime().toString("yyMMddhhmmsszzz") + ".") + _autoScreenCaptureFormat;
	QPixmap figure = QPixmap::grabWidget(this);
	figure.save(targetDir + name);
}

void UPlot::selectScreenCaptureFormat()
{
	QStringList items;
	items << QString("png") << QString("jpg");
	bool ok;
	QString item = QInputDialog::getItem(this, tr("Select format"), tr("Format:"), items, 0, false, &ok);
	if(ok && !item.isEmpty())
	{
		_autoScreenCaptureFormat = item;
	}
	this->captureScreen();
}

void UPlot::clearData()
{
	for(int i=0; i<_curves.size(); ++i)
	{
		// Don't clear threshold curves
		if(qobject_cast<UPlotCurveThreshold*>(_curves.at(i)) == 0)
		{
			_curves.at(i)->clear();
		}
	}
	_aGraphicsView->isChecked()?this->replot(0):this->update();
}

void UPlot::frameData(bool xAxis, bool yAxis)
{
	if(!xAxis && !yAxis)
	{
		return;
	}
	qreal minX = std::numeric_limits<qreal>::max();
	qreal minY = std::numeric_limits<qreal>::max();
	for(int i=0; i<_curves.size(); ++i)
	{
		if(qobject_cast<UPlotCurveThreshold*>(_curves.at(i)) == 0)
		{
			const QVector<qreal> & minMax = _curves.at(i)->getMinMax();
			if(minMax.size() == 4)
			{
				if(minMax[0] < minX)
				{
					minX = minMax[0];
				}
				if(minMax[2] < minY)
				{
					minY = minMax[2];
				}
			}
		}
	}
	if(minX != std::numeric_limits<qreal>::max())
	{
		for(int i=0; i<_curves.size(); ++i)
		{
			if(qobject_cast<UPlotCurveThreshold*>(_curves.at(i)) == 0)
			{
				QVector<qreal> x;
				QVector<qreal> y;
				_curves.at(i)->getData(x,y);
				for(int j=0; j<x.size(); ++j)
				{
					if(xAxis)
					{
						x[j]-=minX;
					}
					if(yAxis)
					{
						y[j]-=minY;
					}
				}
				_curves.at(i)->setData(x,y);
			}
		}
	}
	_aGraphicsView->isChecked()?this->replot(0):this->update();
}

// for convenience...
UPlotCurveThreshold * UPlot::addThreshold(const QString & name, qreal value, Qt::Orientation orientation)
{
	UPlotCurveThreshold * curve = new UPlotCurveThreshold(name, value, orientation, this);
	QPen pen = curve->pen();
	pen.setStyle((Qt::PenStyle)(_penStyleCount++ % 4 + 2));
	curve->setPen(pen);
	if(!this->addCurve(curve))
	{
		if(curve)
		{
			delete curve;
		}
	}
	else
	{
		_aGraphicsView->isChecked()?this->replot(0):this->update();
	}
	return curve;
}

void UPlot::setTitle(const QString & text)
{
	_title->setText(text);
	_title->setVisible(!text.isEmpty());
	this->update();
	if(_aGraphicsView->isChecked())
	{
		QTimer::singleShot(10, this, SLOT(updateAxis()));
	}
}

void UPlot::setXLabel(const QString & text)
{
	_xLabel->setText(text);
	_xLabel->setVisible(!text.isEmpty());
	this->update();
	if(_aGraphicsView->isChecked())
	{
		QTimer::singleShot(10, this, SLOT(updateAxis()));
	}
}

void UPlot::setYLabel(const QString & text, Qt::Orientation orientation)
{
	_yLabel->setText(text);
	_yLabel->setOrientation(orientation);
	_yLabel->setVisible(!text.isEmpty());
	_aYLabelVertical->setChecked(orientation==Qt::Vertical);
	this->update();
	if(_aGraphicsView->isChecked())
	{
		QTimer::singleShot(10, this, SLOT(updateAxis()));
	}
}

void UPlot::setBackgroundColor(const QColor & color)
{
	if(color.isValid())
	{
		_bgColor = color;
		_view->scene()->setBackgroundBrush(QBrush(_bgColor));
		for(QList<UPlotCurve*>::iterator iter=_curves.begin(); iter!=_curves.end(); ++iter)
		{
			(*iter)->setItemsColor(QColor(255-_bgColor.red(), 255-_bgColor.green(), 255-_bgColor.blue(), _bgColor.alpha()));
		}
	}
}

void UPlot::addItem(QGraphicsItem * item)
{
	item->setParentItem(_sceneRoot);
	item->setZValue(1.0f);
}

void UPlot::showLegend(bool shown)
{
	_legend->setVisible(shown);
	_aShowLegend->setChecked(shown);
	this->update();
	if(_aGraphicsView->isChecked())
	{
		QTimer::singleShot(10, this, SLOT(updateAxis()));
	}
}

void UPlot::showGrid(bool shown)
{
	_aShowGrid->setChecked(shown);
	_aGraphicsView->isChecked()?this->replot(0):this->update();
}

void UPlot::showRefreshRate(bool shown)
{
	_aShowRefreshRate->setChecked(shown);
	_refreshRate->setVisible(shown);
	this->update();
	if(_aGraphicsView->isChecked())
	{
		QTimer::singleShot(10, this, SLOT(updateAxis()));
	}
}

void UPlot::trackMouse(bool tracking)
{
	_aMouseTracking->setChecked(tracking);
	this->setMouseTracking(tracking);
}

void UPlot::setGraphicsView(bool on)
{
	_aGraphicsView->setChecked(on);
	_view->setVisible(on);
	_aGraphicsView->isChecked()?this->replot(0):this->update();
	_aMouseTracking->setEnabled(!on);
}

void UPlot::keepAllData(bool kept)
{
	_aKeepAllData->setChecked(kept);
}

void UPlot::setMaxVisibleItems(int maxVisibleItems)
{
	if(maxVisibleItems <= 0)
	{
		_aLimit0->setChecked(true);
	}
	else if(maxVisibleItems == 10)
	{
		_aLimit10->setChecked(true);
	}
	else if(maxVisibleItems == 50)
	{
		_aLimit50->setChecked(true);
	}
	else if(maxVisibleItems == 100)
	{
		_aLimit100->setChecked(true);
	}
	else if(maxVisibleItems == 500)
	{
		_aLimit500->setChecked(true);
	}
	else if(maxVisibleItems == 1000)
	{
		_aLimit1000->setChecked(true);
	}
	else
	{
		_aLimitCustom->setVisible(true);
		_aLimitCustom->setChecked(true);
		_aLimitCustom->setText(QString::number(maxVisibleItems));
	}
	_maxVisibleItems = maxVisibleItems;
	updateAxis();
}

QRectF UPlot::sceneRect() const
{
	return _view->sceneRect();
}

void UPlot::removeCurves()
{
	QList<UPlotCurve*> tmp = _curves;
	for(QList<UPlotCurve*>::iterator iter=tmp.begin(); iter!=tmp.end(); ++iter)
	{
		this->removeCurve(*iter);
	}
	_curves.clear();
}

void UPlot::removeCurve(const UPlotCurve * curve)
{
	QList<UPlotCurve *>::iterator iter = qFind(_curves.begin(), _curves.end(), curve);
#if PRINT_DEBUG
	ULOGGER_DEBUG("Plot=\"%s\" removing curve=\"%s\"", this->objectName().toStdString().c_str(), curve?curve->name().toStdString().c_str():"");
#endif
	if(iter!=_curves.end())
	{
		UPlotCurve * c = *iter;
		c->detach(this);
		_curves.erase(iter);
		_legend->remove(c);
		if(!qobject_cast<UPlotCurveThreshold*>(c))
		{
			// transfer update connection to next curve
			for(int i=_curves.size()-1; i>=0; --i)
			{
				if(!qobject_cast<UPlotCurveThreshold*>(_curves.at(i)))
				{
					connect(_curves.at(i), SIGNAL(dataChanged(const UPlotCurve *)), this, SLOT(updateAxis()));
					break;
				}
			}
		}

		if(c->parent() == this)
		{
			delete c;
		}
		// Update axis
		updateAxis();
	}
}

void UPlot::showCurve(const UPlotCurve * curve, bool shown)
{
	QList<UPlotCurve *>::iterator iter = qFind(_curves.begin(), _curves.end(), curve);
	if(iter!=_curves.end())
	{
		UPlotCurve * value = *iter;
		if(value->isVisible() != shown)
		{
			value->setVisible(shown);
			this->updateAxis();
		}
	}
}

void UPlot::moveCurve(const UPlotCurve * curve, int index)
{
	// this will change the print order
	int currentIndex = -1;
	UPlotCurve * c = 0;
	for(int i=0; i<_curves.size(); ++i)
	{
		if(_curves.at(i) == curve)
		{
			c = _curves.at(i);
			currentIndex = i;
			break;
		}
	}

	if(c && currentIndex != index)
	{
		_curves.removeAt(currentIndex);
		QList<QGraphicsItem *> children = _sceneRoot->childItems();
		_curves.insert(index, c);
		if(currentIndex > index)
		{
			children[currentIndex]->stackBefore(children[index]);
		}
		else
		{
			if(currentIndex<children.size()-2)
			{
				if(index < children.size()-1)
				{
					children[index]->stackBefore(children[currentIndex]);
				}
				else
				{
					children[currentIndex]->stackBefore(children[index]);
				}
			}
			if(currentIndex == children.size()-2 && currentIndex < index)
			{
				children[index]->stackBefore(children[currentIndex]);
			}
		}
		this->update();
	}
}

QString UPlot::getAllCurveDataAsText() const
{
	if(_legend)
	{
		return _legend->getAllCurveDataAsText();
	}
	return "";
}
