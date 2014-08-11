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
