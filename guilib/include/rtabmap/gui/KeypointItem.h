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

#ifndef KEYPOINTITEM_H_
#define KEYPOINTITEM_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <QtGui/QGraphicsEllipseItem>
#include <QtGui/QGraphicsTextItem>
#include <QtGui/QPen>
#include <QtGui/QBrush>

namespace rtabmap {

class RTABMAPGUI_EXP KeypointItem : public QGraphicsEllipseItem
{
public:
	KeypointItem(qreal x, qreal y, int r, const QString & info, const QColor & color = Qt::green, QGraphicsItem * parent = 0);
	virtual ~KeypointItem();

protected:
	virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event );
	virtual void hoverLeaveEvent ( QGraphicsSceneHoverEvent * event );
	virtual void focusInEvent ( QFocusEvent * event );
	virtual void focusOutEvent ( QFocusEvent * event );

private:
	void showDescription();
	void hideDescription();

private:
	QString _info;
	QGraphicsRectItem * _placeHolder;
	int _width;
};

}

#endif /* KEYPOINTITEM_H_ */
