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

#include "rtabmap/gui/ImageView.h"

#include <QtGui/QWheelEvent>
#include <QtCore/qmath.h>
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>
#include <QtCore/QDir>
#include <QtGui/QAction>
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/gui/KeypointItem.h"

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

	_menu = new QMenu(tr(""), this);
	_showImage = _menu->addAction(tr("Show image"));
	_showImage->setCheckable(true);
	_showImage->setChecked(true);
	_showFeatures = _menu->addAction(tr("Show features"));
	_showFeatures->setCheckable(true);
	_showFeatures->setChecked(true);
	_showLines = _menu->addAction(tr("Show lines"));
	_showLines->setCheckable(true);
	_showLines->setChecked(true);
	_saveImage = _menu->addAction(tr("Save picture..."));
}

ImageView::~ImageView() {

}

void ImageView::resetZoom()
{
	_zoom = _minZoom;
	this->setDragMode(QGraphicsView::NoDrag);
}

bool ImageView::isImageShown()
{
	return _showImage->isChecked();
}

bool ImageView::isFeaturesShown()
{
	return _showFeatures->isChecked();
}

void ImageView::setFeaturesShown(bool shown)
{
	_showFeatures->setChecked(shown);
	this->updateItemsShown();
}

bool ImageView::isLinesShown()
{
	return _showLines->isChecked();
}

void ImageView::setLinesShown(bool shown)
{
	_showLines->setChecked(shown);
	this->updateItemsShown();
}

void ImageView::contextMenuEvent(QContextMenuEvent * e)
{
	QAction * action = _menu->exec(e->globalPos());
	if(action == _saveImage)
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
	else if(action == _showFeatures || action == _showImage || action == _showLines)
	{
		this->updateItemsShown();
	}
}

void ImageView::updateItemsShown()
{
	QList<QGraphicsItem*> items = this->scene()->items();
	for(int i=0; i<items.size(); ++i)
	{
		if(qgraphicsitem_cast<KeypointItem*>(items.at(i)))
		{
			items.at(i)->setVisible(_showFeatures->isChecked());
		}
		else if( qgraphicsitem_cast<QGraphicsLineItem*>(items.at(i)))
		{
			items.at(i)->setVisible(_showLines->isChecked());
		}
		else if(qgraphicsitem_cast<QGraphicsPixmapItem*>(items.at(i)))
		{
			items.at(i)->setVisible(_showImage->isChecked());
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
