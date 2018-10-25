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

#include "rtabmap/gui/KeypointItem.h"

#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QGraphicsScene>
#include "rtabmap/utilite/ULogger.h"

namespace rtabmap {

KeypointItem::KeypointItem(int id, const cv::KeyPoint & kpt, float depth, const QColor & color, QGraphicsItem * parent) :
	QGraphicsEllipseItem(kpt.pt.x-(kpt.size==0?3.0f:kpt.size)/2.0f, kpt.pt.y-(kpt.size==0?3.0f:kpt.size)/2.0f, kpt.size==0?3.0f:kpt.size, kpt.size==0?3.0f:kpt.size, parent),
	_id(id),
	_kpt(kpt),
	_placeHolder(0),
	_depth(depth)
{
	this->setColor(color);
	this->setAcceptHoverEvents(true);
	this->setFlag(QGraphicsItem::ItemIsFocusable, true);
	_width = pen().width();
}

KeypointItem::~KeypointItem()
{
	delete _placeHolder;
}

void KeypointItem::setColor(const QColor & color)
{
	this->setPen(QPen(color));
	this->setBrush(QBrush(color));
}

void KeypointItem::showDescription()
{
	if(!_placeHolder)
	{
		_placeHolder = new QGraphicsRectItem (this);
		_placeHolder->setVisible(false);
		if(qGray(pen().color().rgb() > 255/2))
		{
			_placeHolder->setBrush(QBrush(QColor ( 0,0,0, 170 )));
		}
		else
		{
			_placeHolder->setBrush(QBrush(QColor ( 255, 255, 255, 170 )));
		}
		QGraphicsTextItem * text = new QGraphicsTextItem(_placeHolder);
		text->setDefaultTextColor(this->pen().color().rgb());
		// Make octave compatible with SIFT packed octave (https://github.com/opencv/opencv/issues/4554)
		int octave = _kpt.octave & 255;
		octave = octave < 128 ? octave : (-128 | octave);
		float scale = octave >= 0 ? 1.f/(1 << octave) : (float)(1 << -octave);
		if(_depth <= 0)
		{
			text->setPlainText(QString( "Id = %1\n"
					"Dir = %3\n"
					"Hessian = %4\n"
					"X = %5\n"
					"Y = %6\n"
					"Size = %7\n"
					"Octave = %8\n"
					"Scale = %9").arg(_id).arg(_kpt.angle).arg(_kpt.response).arg(_kpt.pt.x).arg(_kpt.pt.y).arg(_kpt.size).arg(octave).arg(scale));
		}
		else
		{
			text->setPlainText(QString( "Id = %1\n"
					"Dir = %3\n"
					"Hessian = %4\n"
					"X = %5\n"
					"Y = %6\n"
					"Size = %7\n"
					"Octave = %8\n"
					"Scale = %9\n"
					"Depth = %10 m").arg(_id).arg(_kpt.angle).arg(_kpt.response).arg(_kpt.pt.x).arg(_kpt.pt.y).arg(_kpt.size).arg(octave).arg(scale).arg(_depth));
		}
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
