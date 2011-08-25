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

#include "Plot.h"

#ifndef PLOT_WIDGET_OUT_OF_LIB
#include "utilite/ULogger.h"
#else
#define ULOGGER_DEBUG(...)
#define ULOGGER_INFO(...)
#define ULOGGER_WARN(...)   printf(__VA_ARGS__);printf("\n")
#define ULOGGER_ERROR(...)   printf(__VA_ARGS__);printf("\n")
#define UDEBUG(...)
#define UINFO(...)
#define UWARN(...)   printf(__VA_ARGS__);printf("\n")
#define UERROR(...)   printf(__VA_ARGS__);printf("\n")
#endif

#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsView>
#include <QtGui/QGraphicsItem>
#include <QtGui/QHBoxLayout>
#include <QtGui/QFormLayout>
#include <QtGui/QResizeEvent>
#include <QtCore/QTime>
#include <QtCore/QTimer>
#include <QtCore/QFileInfo>
#include <QtGui/QPushButton>
#include <QtGui/QToolButton>
#include <QtGui/QLabel>
#include <QtGui/QMenu>
#include <QtGui/QInputDialog>
#include <QtGui/QMessageBox>
#include <QtGui/QFileDialog>
#include <QtGui/QClipboard>
#include <QtGui/QApplication>
#include <QtGui/QPrinter>
#ifdef QT_SVG_LIB
#include <QtSvg/QSvgGenerator>
#endif
#include <cmath>

#ifndef PLOT_WIDGET_OUT_OF_LIB
namespace rtabmap {
#endif

PlotItem::PlotItem(qreal dataX, qreal dataY, qreal width) :
	QGraphicsEllipseItem(0, 0, width, width, 0),
	_previousItem(0),
	_nextItem(0)
{
	_data.setX(dataX);
	_data.setY(dataY);
	this->setZValue(1);
	this->setAcceptsHoverEvents(true);
	_text = new QGraphicsTextItem(this);
	_text->setPlainText(QString("(%1,%2)").arg(_data.x()).arg(_data.y()));
	_text->setVisible(false);
	this->setFlag(QGraphicsItem::ItemIsFocusable, true);
}

PlotItem::PlotItem(const QPointF & data, qreal width) :
	QGraphicsEllipseItem(0, 0, width, width, 0),
	_data(data),
	_previousItem(0),
	_nextItem(0)
{
	this->setZValue(1);
	this->setAcceptsHoverEvents(true);
	_text = new QGraphicsTextItem(this);
	_text->setPlainText(QString("(%1,%2)").arg(_data.x()).arg(_data.y()));
	_text->setVisible(false);
	this->setFlag(QGraphicsItem::ItemIsFocusable, true);
}

PlotItem::~PlotItem()
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

void PlotItem::setData(const QPointF & data)
{
	_data = data;
	_text->setPlainText(QString("(%1,%2)").arg(_data.x()).arg(_data.y()));
}

void PlotItem::setNextItem(PlotItem * nextItem)
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

void PlotItem::setPreviousItem(PlotItem * previousItem)
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

void PlotItem::showDescription(bool shown)
{
	ULOGGER_DEBUG("");
	if(shown)
	{
		this->setPen(QPen(Qt::black, 2));
		if(this->scene())
		{
			QRectF rect = this->scene()->sceneRect();
			QPointF p = this->pos();
			QRectF br = _text->boundingRect();

			// Make sure the text is always in the scene
			if(p.x() - br.width() < 0)
			{
				p.setX(0);
			}
			else if(p.x() + br.width() > rect.width())
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

			_text->setPos(this->mapFromScene(p));
		}

		_text->setVisible(true);
	}
	else
	{
		this->setPen(QPen(Qt::black, 1));
		_text->setVisible(false);
	}
}

void PlotItem::hoverEnterEvent(QGraphicsSceneHoverEvent * event)
{
	QGraphicsScene * scene = this->scene();
	if(scene && scene->focusItem() == 0)
	{
		this->showDescription(true);
	}
	else
	{
		this->setPen(QPen(Qt::black, 2));
	}
	QGraphicsEllipseItem::hoverEnterEvent(event);
}

void PlotItem::hoverLeaveEvent(QGraphicsSceneHoverEvent * event)
{
	if(!this->hasFocus())
	{
		this->showDescription(false);
	}
	QGraphicsEllipseItem::hoverEnterEvent(event);
}

void PlotItem::focusInEvent(QFocusEvent * event)
{
	this->showDescription(true);
	QGraphicsEllipseItem::focusInEvent(event);
}

void PlotItem::focusOutEvent(QFocusEvent * event)
{
	this->showDescription(false);
	QGraphicsEllipseItem::focusOutEvent(event);
}

void PlotItem::keyReleaseEvent(QKeyEvent * keyEvent)
{
	//Get the next/previous visible item
	if(keyEvent->key() == Qt::Key_Right)
	{
		PlotItem * next = _nextItem;
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
		PlotItem * previous = _previousItem;
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





PlotCurve::PlotCurve(const QString & name, QObject * parent) :
	QObject(parent),
	_plot(0),
	_name(name),
	_defaultStepX(1),
	_startX(0),
	_visible(true),
	_valuesShown(false)
{
}

PlotCurve::PlotCurve(const QString & name, QVector<PlotItem *> data, QObject * parent) :
	QObject(parent),
	_plot(0),
	_name(name),
	_defaultStepX(1),
	_startX(0),
	_visible(true),
	_valuesShown(false)
{
	this->setData(data);
}

PlotCurve::PlotCurve(const QString & name, const QVector<float> & x, const QVector<float> & y, QObject * parent) :
	QObject(parent),
	_plot(0),
	_name(name),
	_defaultStepX(1),
	_startX(0),
	_visible(true),
	_valuesShown(false)
{
	this->setData(x, y);
}

PlotCurve::~PlotCurve()
{
	if(_plot)
	{
		_plot->removeCurve(this);
	}
	ULOGGER_DEBUG("%s", this->name().toStdString().c_str());
	this->clear();
}

void PlotCurve::attach(Plot * plot)
{
	if(!plot || plot == _plot)
	{
		return;
	}
	this->setParent(plot);
	if(_plot)
	{
		_plot->removeCurve(this);
	}
	_plot = plot;
	for(int i=0; i<_items.size(); ++i)
	{
		_plot->scene()->addItem(_items.at(i));
	}
}

void PlotCurve::detach(Plot * plot)
{
	ULOGGER_DEBUG("curve=\"%s\" from plot=\"%s\"", this->objectName().toStdString().c_str(), plot?plot->objectName().toStdString().c_str():"");
	if(plot && _plot == plot)
	{
		_plot = 0;
		for(int i=0; i<_items.size(); ++i)
		{
			if(_items.at(i)->scene())
			{
				_items.at(i)->scene()->removeItem(_items.at(i));
			}
		}
	}
}

void PlotCurve::updateMinMax()
{
	float x,y;
	const PlotItem * item;
	if(!_items.size())
	{
		_minMax = QVector<float>();
	}
	else
	{
		_minMax = QVector<float>(4);
	}
	for(int i=0; i<_items.size(); ++i)
	{
		item = qgraphicsitem_cast<const PlotItem *>(_items.at(i));
		if(item)
		{
			x = item->data().x();
			y = item->data().y();
			if(i==1)
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

void PlotCurve::_addValue(PlotItem * data)
{
	// add item
	if(data)
	{
		float x = data->data().x();
		float y = data->data().y();
		if(_minMax.size() != 4)
		{
			_minMax = QVector<float>(4);
		}
		if(_items.size())
		{
			data->setPreviousItem((PlotItem *)_items.last());

			//apply scale
			QGraphicsLineItem * line = new QGraphicsLineItem();
			line->setPen(_pen);
			line->setVisible(false);
			_items.append(line);
			if(_plot)
			{
				_plot->scene()->addItem(line);
			}

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
		_items.append(data);
		data->setVisible(false);
		//data->showDescription(_valuesShown);
		if(_plot)
		{
			_plot->scene()->addItem(_items.last());
		}
	}
	else
	{
		ULOGGER_ERROR("Data is null ?!?");
	}
}

void PlotCurve::addValue(PlotItem * data)
{
	// add item
	if(data)
	{
		this->_addValue(data);
		emit dataChanged(this);
	}
}

void PlotCurve::addValue(float x, float y)
{
	float width = 2; // TODO warn : hard coded value!
	this->addValue(new PlotItem(x,y,width));
}

void PlotCurve::addValue(float y)
{
	float x = 0;
	if(_items.size())
	{
		PlotItem * lastItem = (PlotItem *)_items.last();
		x = lastItem->data().x() + _defaultStepX;
	}
	else
	{
		x = _startX;
	}
	this->addValue(x,y);
}

void PlotCurve::addValue(const QString & value)
{
	bool ok;
	float v = value.toFloat(&ok);
	if(ok)
	{
		this->addValue(v);
	}
	else
	{
		ULOGGER_ERROR("Value not valid, must be a number, received %s", value.toStdString().c_str());
	}
}

void PlotCurve::addValues(QVector<PlotItem *> & data)
{
	for(int i=0; i<data.size(); ++i)
	{
		this->_addValue(data.at(i));
	}
	emit dataChanged(this);
}

void PlotCurve::addValues(const QVector<float> & xs, const QVector<float> & ys)
{
	float width = 2; // TODO warn : hard coded value!
	for(int i=0; i<xs.size() && i<ys.size(); ++i)
	{
		this->_addValue(new PlotItem(xs.at(i),ys.at(i),width));
	}
	emit dataChanged(this);
}

void PlotCurve::addValues(const QVector<float> & ys)
{
	float x = 0;
	float width = 2; // TODO warn : hard coded value!
	for(int i=0; i<ys.size(); ++i)
	{
		if(_items.size())
		{
			PlotItem * lastItem = (PlotItem *)_items.last();
			x = lastItem->data().x() + _defaultStepX;
		}
		else
		{
			x = _startX;
		}
		this->_addValue(new PlotItem(x,ys.at(i),width));
	}
	emit dataChanged(this);
}

int PlotCurve::removeItem(int index)
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
		PlotItem * item = (PlotItem *)_items.takeAt(index); // the plot item
		//Update min/max
		if(_minMax.size() == 4)
		{
			if(item->data().x() == _minMax[0] || item->data().x() == _minMax[1] ||
			   item->data().y() == _minMax[2] || item->data().y() == _minMax[3])
			{
				if(_items.size())
				{
					PlotItem * tmp = (PlotItem *)_items.at(0);
					float x = tmp->data().x();
					float y = tmp->data().y();
					_minMax[0]=x;
					_minMax[1]=x;
					_minMax[2]=y;
					_minMax[3]=y;
					for(int i = 2; i<_items.size(); i+=2)
					{
						tmp = (PlotItem*)_items.at(i);
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
					_minMax = QVector<float>();
				}
			}
		}
		delete item;
	}

	return index;
}

void PlotCurve::removeItem(PlotItem * item) // ownership is transfered to the caller
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

void PlotCurve::clear()
{
	ULOGGER_DEBUG("%s", this->name().toStdString().c_str());
	qDeleteAll(_items);
	_items.clear();
}

void PlotCurve::setPen(const QPen & pen)
{
	_pen = pen;
	for(int i=1; i<_items.size(); i+=2)
	{
		((QGraphicsLineItem*) _items.at(i))->setPen(_pen);
	}
}

void PlotCurve::setBrush(const QBrush & brush)
{
	_brush = brush;
	ULOGGER_WARN("Not used...");
}

void PlotCurve::update(float scaleX, float scaleY, float offsetX, float offsetY, int xDir, int yDir, bool allDataKept)
{
	//ULOGGER_DEBUG("scaleX=%f, scaleY=%f, offsetX=%f, offsetY=%f, xDir=%d, yDir=%d, _plot->scene()->width()=%f, _plot->scene()->height=%f", scaleX, scaleY, offsetX, offsetY, xDir, yDir,_plot->scene()->width(),_plot->scene()->height());
	//make sure direction values are 1 or -1
	xDir<0?xDir=-1:xDir=1;
	yDir<0?yDir=-1:yDir=1;

	bool hide = false;
	for(int i=_items.size()-1; i>=0; --i)
	{
		if(i%2 == 0)
		{
			PlotItem * item = (PlotItem *)_items.at(i);
			if(hide)
			{
				if(allDataKept)
				{
					// if not visible, stop looping... all other items are normally already hided
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
				item->setPos(((xDir*item->data().x()+offsetX)*scaleX-item->rect().width()/2),
							((yDir*item->data().y()+offsetY)*scaleY-item->rect().width()/2));
				if(!item->isVisible())
				{
					item->setVisible(true);
				}
			}

		}
		else
		{
			if(hide)
			{
				_items.at(i)->setVisible(false);
			}
			else
			{
				PlotItem * from = (PlotItem *)_items.at(i-1);
				PlotItem * to = (PlotItem *)_items.at(i+1);
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

int PlotCurve::itemsSize()
{
	return _items.size();
}

QPointF PlotCurve::getItemData(int index)
{
	QPointF data;
	//make sure the index point to a PlotItem {PlotItem, line, PlotItem, line...}
	if(index>=0 && index < _items.size() && index % 2 == 0 )
	{
		data = ((PlotItem*)_items.at(index))->data();
	}
	else
	{
		ULOGGER_ERROR("Wrong index, not pointing on a PlotItem");
	}
	return data;
}

void PlotCurve::setVisible(bool visible)
{
	_visible = visible;
	for(int i=0; i<_items.size(); ++i)
	{
		_items.at(i)->setVisible(visible);
	}
}

void PlotCurve::setData(QVector<PlotItem*> & data)
{
	this->clear();
	for(int i = 0; i<data.size(); ++i)
	{
		this->addValue(data[i]);
	}
}

void PlotCurve::setData(const QVector<float> & x, const QVector<float> & y)
{
	if(x.size() == y.size())
	{
		this->clear();
		for(int i = 0; i<x.size(); ++i)
		{
			this->addValue(x[i], y[i]);
		}
	}
	else
	{
		ULOGGER_ERROR("Data vectors have not the same size.");
	}
}

void PlotCurve::getData(QVector<float> & x, QVector<float> & y) const
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
			x[j] = ((PlotItem*)_items.at(i))->data().x();
			y[j++] = ((PlotItem*)_items.at(i))->data().y();
		}
	}
}





ThresholdCurve::ThresholdCurve(const QString & name, float thesholdValue, Qt::Orientation orientation, QObject * parent) :
	PlotCurve(name, parent),
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

ThresholdCurve::~ThresholdCurve()
{

}

void ThresholdCurve::setThreshold(float threshold)
{
	ULOGGER_DEBUG("%f", threshold);
	if(_items.size() == 3)
	{
		PlotItem * item = 0;
		if(_orientation == Qt::Horizontal)
		{
			item = (PlotItem*)_items.at(0);
			item->setData(QPointF(item->data().x(), threshold));
			item = (PlotItem*)_items.at(2);
			item->setData(QPointF(item->data().x(), threshold));
		}
		else
		{
			item = (PlotItem*)_items.at(0);
			item->setData(QPointF(threshold, item->data().y()));
			item = (PlotItem*)_items.at(2);
			item->setData(QPointF(threshold, item->data().y()));
		}
	}
	else
	{
		ULOGGER_ERROR("A threshold must has only 3 items (1 PlotItem + 1 QGraphicsLineItem + 1 PlotItem)");
	}
}

void ThresholdCurve::setOrientation(Qt::Orientation orientation)
{
	if(_orientation != orientation)
	{
		_orientation = orientation;
		if(_items.size() == 3)
		{
			PlotItem * item = 0;
			item = (PlotItem*)_items.at(0);
			item->setData(QPointF(item->data().y(), item->data().x()));
			item = (PlotItem*)_items.at(2);
			item->setData(QPointF(item->data().y(), item->data().x()));
		}
		else
		{
			ULOGGER_ERROR("A threshold must has only 3 items (1 PlotItem + 1 QGraphicsLineItem + 1 PlotItem)");
		}
	}
}

void ThresholdCurve::update(float scaleX, float scaleY, float offsetX, float offsetY, int xDir, int yDir, bool allDataKept)
{
	if(_items.size() == 3)
	{
		if(_plot)
		{
			PlotItem * item = 0;
			if(_orientation == Qt::Horizontal)
			{
				//(xDir*item->data().x()+offsetX)*scaleX
				item = (PlotItem*)_items.at(0);
				item->setData(QPointF(-offsetX/xDir, item->data().y()));
				item = (PlotItem*)_items.at(2);
				item->setData(QPointF( (_plot->scene()->width()/scaleX-offsetX)/xDir, item->data().y()));
			}
			else
			{
				item = (PlotItem*)_items.at(0);
				item->setData(QPointF(item->data().x(), -offsetY/yDir));
				item = (PlotItem*)_items.at(2);
				item->setData(QPointF(item->data().x(), (_plot->scene()->height()/scaleY-offsetY)/yDir));
			}
			this->updateMinMax();
		}
	}
	else
	{
		ULOGGER_ERROR("A threshold must has only 3 items (1 PlotItem + 1 QGraphicsLineItem + 1 PlotItem)");
	}
	PlotCurve::update(scaleX, scaleY, offsetX, offsetY, xDir, yDir, allDataKept);
}







PlotAxis::PlotAxis(Qt::Orientation orientation, float min, float max, QWidget * parent) :
	QWidget(parent),
	_orientation(orientation),
	_reversed(false),
	_gradMaxDigits(4),
	_border(0)
{
	if(_orientation == Qt::Vertical)
	{
		_reversed = true; // default bottom->up
	}
#ifdef WIN32
	this->setMinimumSize(15, 25);
#else
	this->setMinimumSize(15, 25);
#endif
	this->setAxis(min, max); // this initialize all attributes
}

PlotAxis::~PlotAxis()
{
	ULOGGER_DEBUG("");
}

// Vertical :bottom->up, horizontal :right->left
void PlotAxis::setReversed(bool reversed)
{
	if(_reversed != reversed)
	{
		float min = _min;
		_min = _max;
		_max = min;
	}
	_reversed = reversed;
}

void PlotAxis::setAxis(float & min, float & max)
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
		float mul = 1;
		float rangef = fabsf(max - min);
		int countStep = _count/5;
		float val;
		for(int i=0; i<6; ++i)
		{
			val = (rangef/countStep) * mul;
			if( val >= 1 && val < 10)
			{
				break;
			}
			else if(val<1)
			{
				mul *= 10;
			}
			else
			{
				mul /= 10;
			}
		}
		//ULOGGER_DEBUG("min=%f, max=%f", min, max);
		int minR = min*mul-0.9;
		int maxR = max*mul+0.9;
		min = float(minR)/mul;
		max = float(maxR)/mul;
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

void PlotAxis::paintEvent(QPaintEvent * event)
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




PlotLegendItem::PlotLegendItem(const PlotCurve * curve, QWidget * parent) :
		QPushButton(parent),
		_curve(curve)
{
	QString nameSpaced = curve->name();
	nameSpaced.replace('_', ' ');
	this->setText(nameSpaced);

	_aChangeText = new QAction(tr("Change text..."), this);
	_aResetText = new QAction(tr("Reset text..."), this);
	_aRemoveCurve = new QAction(tr("Remove this curve"), this);
	_aCopyToClipboard = new QAction(tr("Copy curve data to the clipboard"), this);
	_menu = new QMenu(tr("Curve"), this);
	_menu->addAction(_aChangeText);
	_menu->addAction(_aResetText);
	_menu->addAction(_aRemoveCurve);
	_menu->addAction(_aCopyToClipboard);
}

PlotLegendItem::~PlotLegendItem()
{

}
void PlotLegendItem::contextMenuEvent(QContextMenuEvent * event)
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
	else if(action == _aRemoveCurve)
	{
		emit legendItemRemoved(_curve);
	}
	else if (action == _aCopyToClipboard)
	{
		if(_curve)
		{
			QVector<float> x;
			QVector<float> y;
			_curve->getData(x, y);
			QString textX;
			QString textY;
			for(int i=0; i<x.size(); ++i)
			{
				textX.append(QString::number(x[i]));
				textY.append(QString::number(y[i]));
				if(i+1<x.size())
				{
					textX.append(' ');
					textY.append(' ');
				}
			}
			QClipboard * clipboard = QApplication::clipboard();
			clipboard->setText((textX+"\n")+textY);
		}
	}
}






PlotLegend::PlotLegend(QWidget * parent) :
	QWidget(parent),
	_flat(true)
{
	//menu
	_aUseFlatButtons = new QAction(tr("Use flat buttons"), this);
	_aUseFlatButtons->setCheckable(true);
	_aUseFlatButtons->setChecked(_flat);
	_menu = new QMenu(tr("Legend"), this);
	_menu->addAction(_aUseFlatButtons);

	QVBoxLayout * vLayout = new QVBoxLayout(this);
	vLayout->setContentsMargins(0,0,0,0);
	this->setLayout(vLayout);
	vLayout->addStretch(0);
	vLayout->setSpacing(0);
}

PlotLegend::~PlotLegend()
{
	ULOGGER_DEBUG("");
}

void PlotLegend::setFlat(bool on)
{
	if(_flat != on)
	{
		_flat = on;
		QList<PlotLegendItem*> items = this->findChildren<PlotLegendItem*>();
		for(int i=0; i<items.size(); ++i)
		{
			items.at(i)->setFlat(_flat);
			items.at(i)->setChecked(!items.at(i)->isChecked());
		}
		_aUseFlatButtons->setChecked(_flat);
	}
}

void PlotLegend::addItem(const PlotCurve * curve)
{
	if(curve)
	{
		PlotLegendItem * legendItem = new PlotLegendItem(curve, this);
		legendItem->setFlat(_flat);
		legendItem->setCheckable(true);
		legendItem->setChecked(false);
		legendItem->setIcon(QIcon(this->createSymbol(curve->pen(), curve->brush())));
		legendItem->setIconSize(QSize(25,20));
		connect(legendItem, SIGNAL(toggled(bool)), this, SLOT(redirectToggled(bool)));
		connect(legendItem, SIGNAL(legendItemRemoved(const PlotCurve *)), this, SLOT(removeLegendItem(const PlotCurve *)));
	
		// layout
		QHBoxLayout * hLayout = new QHBoxLayout();
		hLayout->addWidget(legendItem);
		hLayout->addStretch(0);
		hLayout->setMargin(0);

		// add to the legend
		((QVBoxLayout*)this->layout())->insertLayout(this->layout()->count()-1, hLayout);
	}
}

QPixmap PlotLegend::createSymbol(const QPen & pen, const QBrush & brush)
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

void PlotLegend::removeLegendItem(const PlotCurve * curve)
{
	QList<PlotLegendItem *> items = this->findChildren<PlotLegendItem*>();
	for(int i=0; i<items.size(); ++i)
	{
		if(items.at(i)->curve() == curve)
		{
			delete items.at(i);
			emit legendItemRemoved(curve);
		}
	}
}

void PlotLegend::contextMenuEvent(QContextMenuEvent * event)
{
	QAction * action = _menu->exec(event->globalPos());
	if(action == _aUseFlatButtons)
	{
		this->setFlat(_aUseFlatButtons->isChecked());
	}
}

void PlotLegend::redirectToggled(bool toggled)
{
	if(sender())
	{
		PlotLegendItem * item = qobject_cast<PlotLegendItem*>(sender());
		if(item)
		{
			emit legendItemToggled(item->curve(), _flat?!toggled:toggled);
		}
	}
}







OrientableLabel::OrientableLabel(const QString & text, Qt::Orientation orientation, QWidget * parent) :
	QLabel(text, parent),
	_orientation(orientation)
{
}

OrientableLabel::~OrientableLabel()
{
}

QSize OrientableLabel::sizeHint() const
{
	QSize size = QLabel::sizeHint();
	if (_orientation == Qt::Vertical)
		size.transpose();
	return size;

}

QSize OrientableLabel::minimumSizeHint() const
{
	QSize size = QLabel::minimumSizeHint();
	if (_orientation == Qt::Vertical)
		size.transpose();
	return size;
}

void OrientableLabel::setOrientation(Qt::Orientation orientation)
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

void OrientableLabel::paintEvent(QPaintEvent* event)
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













Plot::Plot(QWidget *parent) :
	QWidget(parent),
	_maxVisibleItems(-1),
	_autoScreenCaptureFormat("png")
{
	this->setupUi();
	this->createActions();
	this->createMenus();

	// This will update actions
	this->showLegend(true);
	this->setMaxVisibleItems(0);
	this->showGrid(false);
	this->showRefreshRate(false);
	this->keepAllData(false);

	for(int i=0; i<4; ++i)
	{
		_axisMaximums[i] = 0;
		_axisMaximumsSet[i] = false;
		if(i<2)
		{
			_fixedAxis[i] = false;
		}
	}

	_refreshIntervalTime.start();
	_lowestRefreshRate = 99;
	_refreshStartTime.start();

	_penStyleCount = rand() % 10 + 1; // rand 1->10
	_workingDirectory = QDir::homePath();
}

Plot::~Plot()
{
	ULOGGER_DEBUG("%s", this->title().toStdString().c_str());
	QList<PlotCurve*> curves = _curves.values();
	for(int i=0; i<curves.size(); ++i)
	{
		this->removeCurve(curves.at(i));
	}
}

void Plot::setupUi()
{
	_legend = new PlotLegend(this);
	_view = new QGraphicsView(this);
	_view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	_view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	_view->setScene(new QGraphicsScene(0,0,0,0,this));
	_verticalAxis = new PlotAxis(Qt::Vertical, 0, 1, this);
	_horizontalAxis = new PlotAxis(Qt::Horizontal, 0, 1, this);
	_title = new QLabel("");
	_xLabel = new QLabel("");
	_refreshRate = new QLabel("");
	_yLabel = new OrientableLabel("");
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
	QGridLayout * grid = new QGridLayout();
	grid->setContentsMargins(0,0,0,0);
	this->setLayout(grid);
	grid->addWidget(_title, 0, 2);
	grid->addWidget(_yLabel, 1, 0);
	grid->addWidget(_verticalAxis, 1, 1);
	grid->addWidget(_refreshRate, 2, 1);
	grid->addWidget(_view, 1, 2);
	grid->setColumnStretch(2, 1);
	grid->addWidget(_horizontalAxis, 2, 2);
	grid->addWidget(_xLabel, 3, 2);
	grid->addWidget(_legend, 1, 3);
	connect(_legend, SIGNAL(legendItemToggled(const PlotCurve *, bool)), this, SLOT(showCurve(const PlotCurve *, bool)));
	connect(_legend, SIGNAL(legendItemRemoved(const PlotCurve *)), this, SLOT(removeCurve(const PlotCurve *)));
}

void Plot::createActions()
{
	_aShowLegend = new QAction(tr("Show legend"), this);
	_aShowLegend->setCheckable(true);
	_aShowGrid = new QAction(tr("Show grid"), this);
	_aShowGrid->setCheckable(true);
	_aShowRefreshRate = new QAction(tr("Show refresh rate"), this);
	_aShowRefreshRate->setCheckable(true);
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

void Plot::createMenus()
{
	_menu = new QMenu(tr("Plot"), this);
	_menu->addAction(_aShowLegend);
	_menu->addAction(_aShowGrid);
	_menu->addAction(_aShowRefreshRate);
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
	_menu->addAction(_aSaveFigure);
	_menu->addAction(_aAutoScreenCapture);
	_menu->addSeparator();
	_menu->addAction(_aClearData);

}

PlotCurve * Plot::addCurve(const QString & curveName)
{
	// add curve
	PlotCurve * curve = new PlotCurve(curveName, this);
	//curve->pen() = QPen((Qt::PenStyle)(_penStyleCount++ % 4 + 2));
	curve->setPen(this->getRandomPenColored());
	this->addCurve(curve);
	return curve;
}

// Ownership is transferred only if true is returned,
// Returning false if a curve with
// the same name is already added to the plot.
bool Plot::addCurve(PlotCurve * curve)
{
	if(curve)
	{
		// add curve
		_curves.insert(curve, curve);
		curve->attach(this); // ownership is transferred
		this->updateAxis(curve);
		curve->setStartX(_axisMaximums[1]);
		connect(curve, SIGNAL(dataChanged(const PlotCurve *)), this, SLOT(updateAxis()));

		_legend->addItem(curve);

		ULOGGER_DEBUG("Curve \"%s\" added to plot \"%s\"", curve->name().toStdString().c_str(), this->title().toStdString().c_str());

		this->update();
		return true;
	}
	else
	{
		ULOGGER_ERROR("The curve is null!");
	}
	return false;
}

QStringList Plot::curveNames()
{
	QStringList names;
	for(QMap<const PlotCurve*, PlotCurve*>::iterator iter = _curves.begin(); iter!=_curves.end(); ++iter)
	{
		if(*iter)
		{
			names.append((*iter)->name());
		}
	}
	return names;
}

bool Plot::contains(const QString & curveName)
{
	for(QMap<const PlotCurve*, PlotCurve*>::iterator iter = _curves.begin(); iter!=_curves.end(); ++iter)
	{
		if(*iter && (*iter)->name().compare(curveName) == 0)
		{
			return true;
		}
	}
	return false;
}

QPen Plot::getRandomPenColored()
{
	return QPen((Qt::GlobalColor)(_penStyleCount++ % 12 + 7 ));
}

void Plot::replot()
{
	QList<PlotCurve *> curves = _curves.values();
	if(_maxVisibleItems>0)
	{
		PlotCurve * c = 0;
		int maxItem = 0;
		// find the curve with the most items
		for(QList<PlotCurve *>::iterator i=curves.begin(); i!=curves.end(); ++i)
		{
			if((*i)->isVisible() && ((PlotCurve *)(*i))->itemsSize() > maxItem)
			{
				c = *i;
				maxItem = c->itemsSize();
			}
		}
		if(c && (maxItem-1)/2+1 > _maxVisibleItems)
		{
			_axisMaximums[0] = c->getItemData((c->itemsSize()-1) -_maxVisibleItems*2).x();
		}
	}

	float axis[4] = {0};
	for(int i=0; i<4; ++i)
	{
		axis[i] = _axisMaximums[i];
	}

	_verticalAxis->setAxis(axis[2], axis[3]);
	_horizontalAxis->setAxis(axis[0], axis[1]);

	//ULOGGER_DEBUG("x1=%f, x2=%f, y1=%f, y2=%f", _axisMaximums[0], _axisMaximums[1], _axisMaximums[2], _axisMaximums[3]);

	QRectF newRect(0,0, _view->size().width(), _view->size().height());
	_view->scene()->setSceneRect(newRect);
	int borderHor = _horizontalAxis->border();
	int borderVer = _verticalAxis->border();

	float scaleX = 1;
	float scaleY = 1;
	float den = 0;
	den = axis[1] - axis[0];
	if(den != 0)
	{
		scaleX = (_view->sceneRect().width()-(borderHor*2)) / den;
	}
	den = axis[3] - axis[2];
	if(den != 0)
	{
		scaleY = (_view->sceneRect().height()-(borderVer*2)) / den;
	}

	for(QList<PlotCurve *>::iterator i=curves.begin(); i!=curves.end(); ++i)
	{
		if((*i)->isVisible())
		{
			int xDir = 1;
			int yDir = -1;
			(*i)->update(scaleX,
						scaleY,
						xDir<0?axis[1]+float(borderHor-2)/scaleX:-(axis[0]-float(borderHor-2)/scaleX),
						yDir<0?axis[3]+float(borderVer-2)/scaleY:-(axis[2]-float(borderVer-2)/scaleY),
						xDir,
						yDir,
						_aKeepAllData->isChecked());
		}
	}

	//grid
	qDeleteAll(hGridLines);
	hGridLines.clear();
	qDeleteAll(vGridLines);
	vGridLines.clear();
	if(_aShowGrid->isChecked())
	{
		borderHor-=2;
		borderVer-=2;
		// TODO make a PlotGrid class ?
		int w = _view->sceneRect().width()-(borderHor*2);
		int h = _view->sceneRect().height()-(borderVer*2);
		float stepH = w / _horizontalAxis->count();
		float stepV = h / _verticalAxis->count();
		QPen pen(Qt::DashLine);
		for(int i=0; i*stepV < h+stepV; i+=5)
		{
			//horizontal lines
			hGridLines.append(_view->scene()->addLine(0, stepV*i+borderVer, borderHor, stepV*i+borderVer));
			hGridLines.append(_view->scene()->addLine(borderHor, stepV*i+borderVer, w+borderHor, stepV*i+borderVer, pen));
			hGridLines.append(_view->scene()->addLine(w+borderHor, stepV*i+borderVer, w+borderHor*2, stepV*i+borderVer));
		}
		for(int i=0; i*stepH < w+stepH; i+=5)
		{
			//vertical lines
			vGridLines.append(_view->scene()->addLine(stepH*i+borderHor, 0, stepH*i+borderHor, borderVer));
			vGridLines.append(_view->scene()->addLine(stepH*i+borderHor, borderVer, stepH*i+borderHor, h+borderVer, pen));
			vGridLines.append(_view->scene()->addLine(stepH*i+borderHor, h+borderVer, stepH*i+borderHor, h+borderVer*2));
		}
	}

	// Update refresh rate
	if(_aShowRefreshRate->isChecked())
	{
		int refreshRate = qRound(1000.0f/float(_refreshIntervalTime.restart()));
		if(refreshRate > 0 && refreshRate < _lowestRefreshRate)
		{
			_lowestRefreshRate = refreshRate;
		}
		// Refresh the label only after each 100 ms
		if(_refreshStartTime.elapsed() > 100)
		{
			_refreshRate->setText(QString::number(_lowestRefreshRate));
			_lowestRefreshRate = 99;
			_refreshStartTime.start();
		}
	}
}

void Plot::setFixedXAxis(float x1, float x2)
{
	_fixedAxis[0] = true;
	_axisMaximums[0] = x1;
	_axisMaximums[1] = x2;
}

void Plot::setFixedYAxis(float y1, float y2)
{
	_fixedAxis[1] = true;
	_axisMaximums[2] = y1;
	_axisMaximums[3] = y2;
}

void Plot::updateAxis(const PlotCurve * curve)
{
	PlotCurve * value = _curves.value(curve, 0);
	if(value && value->isVisible() && value->itemsSize() && value->isMinMaxValid())
	{
		const QVector<float> & minMax = value->getMinMax();
		//ULOGGER_DEBUG("x1=%f, x2=%f, y1=%f, y2=%f", minMax[0], minMax[1], minMax[2], minMax[3]);
		if(minMax.size() != 4)
		{
			ULOGGER_ERROR("minMax size != 4 ?!?");
			return;
		}
		this->updateAxis(minMax[0], minMax[1], minMax[2], minMax[3]);
		this->update();
	}
}

bool Plot::updateAxis(float x1, float x2, float y1, float y2)
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

bool Plot::updateAxis(float x, float y)
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

void Plot::updateAxis()
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

	QList<PlotCurve*> curves = _curves.values();
	for(int i=0; i<curves.size(); ++i)
	{
		if(curves.at(i)->isVisible() && curves.at(i)->isMinMaxValid())
		{
			const QVector<float> & minMax = curves.at(i)->getMinMax();
			this->updateAxis(minMax[0], minMax[1], minMax[2], minMax[3]);
		}
	}

	this->update();

	this->captureScreen();
}

void Plot::paintEvent(QPaintEvent * event)
{
	this->replot();
	QWidget::paintEvent(event);
}

void Plot::contextMenuEvent(QContextMenuEvent * event)
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
		this->updateAxis();
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
		QList<PlotCurve *> curves = _curves.values();
		for(int i=0; i<curves.size(); ++i)
		{
			// Don't clear threshold curves
			if(qobject_cast<ThresholdCurve*>(curves.at(i)) == 0)
			{
				curves.at(i)->clear();
			}
		}
		this->update();
	}
	else
	{
		ULOGGER_WARN("Unknown action");
	}
}

void Plot::setWorkingDirectory(const QString & workingDirectory)
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

void Plot::captureScreen()
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

void Plot::selectScreenCaptureFormat()
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

// for convenience...
ThresholdCurve * Plot::addThreshold(const QString & name, float value, Qt::Orientation orientation)
{
	ThresholdCurve * curve = new ThresholdCurve(name, value, orientation, this);
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
		this->update();
	}
	return curve;
}

void Plot::setTitle(const QString & text)
{
	_title->setText(text);
	_title->setVisible(!text.isEmpty());
	this->update();
}

void Plot::setXLabel(const QString & text)
{
	_xLabel->setText(text);
	_xLabel->setVisible(!text.isEmpty());
	this->update();
}

void Plot::setYLabel(const QString & text, Qt::Orientation orientation)
{
	_yLabel->setText(text);
	_yLabel->setOrientation(orientation);
	_yLabel->setVisible(!text.isEmpty());
	_aYLabelVertical->setChecked(orientation==Qt::Vertical);
	this->update();
}

QGraphicsScene * Plot::scene() const
{
	return _view->scene();
}

void Plot::showLegend(bool shown)
{
	_legend->setVisible(shown);
	_aShowLegend->setChecked(shown);
	this->update();
}

void Plot::showGrid(bool shown)
{
	_aShowGrid->setChecked(shown);
	this->update();
}

void Plot::showRefreshRate(bool shown)
{
	_aShowRefreshRate->setChecked(shown);
	_refreshRate->setVisible(shown);
	this->update();
}

void Plot::keepAllData(bool kept)
{
	_aKeepAllData->setChecked(kept);
}

void Plot::setMaxVisibleItems(int maxVisibleItems)
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
}

void Plot::removeCurves()
{
	QList<PlotCurve*> tmp = _curves.values();
	for(QList<PlotCurve*>::iterator iter=tmp.begin(); iter!=tmp.end(); ++iter)
	{
		this->removeCurve(*iter);
	}
	_curves.clear();
}

void Plot::removeCurve(const PlotCurve * curve)
{
	PlotCurve * c = _curves.value(curve, 0);
	ULOGGER_DEBUG("Plot=\"%s\" removing curve=\"%s\"", this->objectName().toStdString().c_str(), curve?curve->name().toStdString().c_str():"");
	if(c)
	{
		c->detach(this);
		_curves.remove(c);
		_legend->removeLegendItem(c);
		if(c->parent() == this)
		{
			delete c;
		}
		// Update axis
		updateAxis();
	}
}

void Plot::showCurve(const PlotCurve * curve, bool shown)
{
	PlotCurve * value = _curves.value(curve, 0);
	if(value)
	{
		if(value->isVisible() != shown)
		{
			value->setVisible(shown);
			this->updateAxis();
		}
	}
}

#ifndef PLOT_WIDGET_OUT_OF_LIB
}
#endif
