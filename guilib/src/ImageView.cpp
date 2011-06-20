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

#include "ImageView.h"

#include <QtGui/QWheelEvent>
#include <QtCore/qmath.h>
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>
#include <QtCore/QDir>
#include "utilite/ULogger.h"

namespace rtabmap {

ImageView::ImageView(QWidget * parent) :
		QGraphicsView(parent),
		_zoom(250),
		_minZoom(250),
		_savedFileName((QDir::homePath()+ "/") + "picture" + ".png")
{
	this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	this->setScene(new QGraphicsScene(this));
	connect(this->scene(), SIGNAL(sceneRectChanged(const QRectF &)), this, SLOT(updateZoom()));
}

ImageView::~ImageView() {

}

void ImageView::resetZoom()
{
	_zoom = _minZoom;
	this->setDragMode(QGraphicsView::NoDrag);
}

void ImageView::contextMenuEvent(QContextMenuEvent * e)
{
	QMenu menu(tr(""), this);
	QAction * saveImage = menu.addAction(tr("Save picture..."));

	if(menu.exec(e->globalPos()) == saveImage)
	{
		QString text;
#ifdef QT_SVG_LIB
		text = QFileDialog::getSaveFileName(this, tr("Save figure to ..."), _savedFileName, "*.png *.xpm *.jpg *.pdf *.svg");
#else
		text = QFileDialog::getSaveFileName(this, tr("Save figure to ..."), _savedFileName, "*.png *.xpm *.jpg *.pdf");
#endif
		if(!text.isEmpty())
		{
			_savedFileName = text;
			QImage img(this->sceneRect().width(), this->sceneRect().height(),QImage::Format_ARGB32_Premultiplied);
			QPainter p(&img);
			this->scene()->render(&p, this->sceneRect(), this->sceneRect());
			img.save(text);
		}
	}
}

void ImageView::updateZoom()
{
	qreal scaleRatio = 1;
	if(this->scene())
	{
		scaleRatio = this->geometry().width()/this->sceneRect().width();
	}
	_minZoom = log(scaleRatio)/log(2)*50+250;
}

void ImageView::wheelEvent(QWheelEvent * e)
{
	if(e->delta() > 0)
	{
		_zoom += 20;
		this->setDragMode(QGraphicsView::ScrollHandDrag);
		if(_zoom>=500)
		{
			_zoom = 500;
		}
	}
	else
	{
		_zoom -= 20;
		if(_zoom<=_minZoom)
		{
			this->setDragMode(QGraphicsView::NoDrag);
			_zoom = _minZoom;
			this->fitInView(this->sceneRect(), Qt::KeepAspectRatio);
			return;
		}
	}

	qreal scale = qPow(qreal(2), (_zoom - 250) / qreal(50));
	QMatrix matrix;
	matrix.scale(scale, scale);
	this->setMatrix(matrix);
}

}
