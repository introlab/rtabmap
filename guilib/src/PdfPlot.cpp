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

#include "PdfPlot.h"
#include <rtabmap/utilite/ULogger.h>
#include "rtabmap/gui/UCv2Qt.h"
#include "rtabmap/core/util3d.h"

namespace rtabmap {

PdfPlotItem::PdfPlotItem(float dataX, float dataY, float width, int childCount) :
	UPlotItem(dataX, dataY, width),
	_img(0),
	_imagesRef(0),
	_text(0)
{
	setLikelihood(dataX, dataY, childCount);
}

PdfPlotItem::~PdfPlotItem()
{

}

void PdfPlotItem::setLikelihood(int id, float value, int childCount)
{
	if(_img && id != this->data().x())
	{
		delete _img;
		_img = 0;
	}
	this->setData(QPointF(id, value));
	_childCount = childCount;
}

void PdfPlotItem::showDescription(bool shown)
{
	if(!_text)
	{
		_text = new QGraphicsTextItem(this);
		_text->setVisible(false);
	}
	if(shown)
	{
		if(!_img && _imagesRef)
		{
			QImage img;
			QMap<int, std::vector<unsigned char> >::const_iterator iter = _imagesRef->find(int(this->data().x()));
			if(iter != _imagesRef->constEnd())
			{
				img = uCvMat2QImage(util3d::uncompressImage(iter.value()));
				QPixmap scaled = QPixmap::fromImage(img).scaledToWidth(128);
				_img = new QGraphicsPixmapItem(scaled, this);
				_img->setVisible(false);
			}
		}

		if(_img)
			_text->setPos(this->mapFromScene(4+150,0));
		else
			_text->setPos(this->mapFromScene(4,0));
		if(_childCount >= 0)
		{
			_text->setPlainText(QString("ID = %1\nValue = %2\nWeight = %3").arg(this->data().x()).arg(this->data().y()).arg(_childCount));
		}
		else
		{
			_text->setPlainText(QString("ID = %1\nValue = %2").arg(this->data().x()).arg(this->data().y()));
		}
		_text->setVisible(true);
		if(_img)
		{
			_img->setPos(this->mapFromScene(4,0));
			_img->setVisible(true);
		}
	}
	else
	{
		_text->setVisible(false);
		if(_img)
			_img->setVisible(false);
	}
	UPlotItem::showDescription(shown);
}





PdfPlotCurve::PdfPlotCurve(const QString & name, const QMap<int, std::vector<unsigned char> > * imagesMapRef = 0, QObject * parent) :
	UPlotCurve(name, parent),
	_imagesMapRef(imagesMapRef)
{

}

PdfPlotCurve::~PdfPlotCurve()
{

}

void PdfPlotCurve::clear()
{
	UPlotCurve::clear();
}

void PdfPlotCurve::setData(const QMap<int, float> & dataMap, const QMap<int, int> & weightsMap)
{
	ULOGGER_DEBUG("dataMap=%d, weightsMap=%d", dataMap.size(), weightsMap.size());
	if(dataMap.size() > 0)
	{
		//match the size of the current data
		int margin = int((_items.size()+1)/2) - dataMap.size();

		while(margin < 0)
		{
			PdfPlotItem * newItem = new PdfPlotItem(0, 0, 2, 0);
			newItem->setImagesRef(_imagesMapRef);
			this->_addValue(newItem);
			++margin;
		}

		while(margin > 0)
		{
			this->removeItem(0);
			--margin;
		}

		ULOGGER_DEBUG("itemsize=%d", _items.size());

		// update values
		QList<QGraphicsItem*>::iterator iter = _items.begin();
		QMap<int, int>::const_iterator j=weightsMap.begin();
		for(QMap<int, float>::const_iterator i=dataMap.begin(); i!=dataMap.end(); ++i, ++j)
		{
			((PdfPlotItem*)*iter)->setLikelihood(i.key(),  i.value(), j!=weightsMap.end()?j.value():-1);
			//2 times...
			++iter;
			++iter;
		}

		//reset minMax, this will force the plot to update the axes
		this->updateMinMax();
		emit dataChanged(this);
	}
}

}
