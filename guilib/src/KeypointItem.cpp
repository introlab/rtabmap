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

#include "rtabmap/gui/KeypointItem.h"

#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QGraphicsScene>
#include "rtabmap/utilite/ULogger.h"

namespace rtabmap {

KeypointItem::KeypointItem(qreal x, qreal y, int r, const QString & info, const QColor & color, QGraphicsItem * parent) :
	QGraphicsEllipseItem(x, y, r, r, parent),
	_info(info),
	_placeHolder(0)
{
	this->setPen(QPen(color));
	this->setBrush(QBrush(color));
	this->setAcceptsHoverEvents(true);
	this->setFlag(QGraphicsItem::ItemIsFocusable, true);
	_width = pen().width();
}

KeypointItem::~KeypointItem()
{
	/*if(_placeHolder)
	{
		delete _placeHolder;
	}*/
}

void KeypointItem::showDescription()
{
	if(!_placeHolder)
	{
		_placeHolder = new QGraphicsRectItem (this);
		_placeHolder->setVisible(false);
		_placeHolder->setBrush(QBrush(QColor ( 0, 0, 0, 170 ))); // Black transparent background
		QGraphicsTextItem * text = new QGraphicsTextItem(_placeHolder);
		text->setDefaultTextColor(this->pen().color().rgb());
		text->setPlainText(_info);
		_placeHolder->setRect(text->boundingRect());
	}


	if(_placeHolder->parentItem())
	{
		_placeHolder->setParentItem(0); // Make it a to level item
	}
	QPen pen = this->pen();
	this->setPen(QPen(pen.color(), _width+2));
	_placeHolder->setZValue(this->zValue()+1);
	_placeHolder->setPos(this->mapFromScene(0,0));
	_placeHolder->setVisible(true);
}

void KeypointItem::hideDescription()
{
	if(_placeHolder)
	{
		_placeHolder->setVisible(false);
	}
	this->setPen(QPen(pen().color(), _width));
}

void KeypointItem::hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
{
	QGraphicsScene * scene = this->scene();
	if(scene && scene->focusItem() == 0)
	{
		this->showDescription();
	}
	else
	{
		this->setPen(QPen(pen().color(), _width+2));
	}
	QGraphicsEllipseItem::hoverEnterEvent(event);
}

void KeypointItem::hoverLeaveEvent ( QGraphicsSceneHoverEvent * event )
{
	if(!this->hasFocus())
	{
		this->hideDescription();
	}
	QGraphicsEllipseItem::hoverEnterEvent(event);
}

void KeypointItem::focusInEvent ( QFocusEvent * event )
{
	this->showDescription();
	QGraphicsEllipseItem::focusInEvent(event);
}

void KeypointItem::focusOutEvent ( QFocusEvent * event )
{
	this->hideDescription();
	QGraphicsEllipseItem::focusOutEvent(event);
}

}
