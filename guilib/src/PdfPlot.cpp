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

#include "rtabmap/gui/PdfPlot.h"
#include <rtabmap/utilite/ULogger.h>
#include "rtabmap/utilite/UCv2Qt.h"
#include "rtabmap/core/util3d.h"

namespace rtabmap {

PdfPlotItem::PdfPlotItem(float dataX, float dataY, float width, int childCount) :
	UPlotItem(dataX, dataY, width),
	_img(0),
	_signaturesRef(0),
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
		if(!_img && _signaturesRef)
		{
			QImage img;
			QMap<int, Signature>::const_iterator iter = _signaturesRef->find(int(this->data().x()));
			if(iter != _signaturesRef->constEnd() && !iter.value().sensorData().imageCompressed().empty())
			{
				cv::Mat image;
				iter.value().sensorData().uncompressDataConst(&image, 0, 0);
				if(!image.empty())
				{
					img = uCvMat2QImage(image);
					QPixmap scaled = QPixmap::fromImage(img).scaledToWidth(128);
					_img = new QGraphicsPixmapItem(scaled, this);
					_img->setVisible(false);
				}
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





PdfPlotCurve::PdfPlotCurve(const QString & name, const QMap<int, Signature> * signaturesMapRef = 0, QObject * parent) :
	UPlotCurve(name, parent),
	_signaturesMapRef(signaturesMapRef)
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
			newItem->setSignaturesRef(_signaturesMapRef);
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
		for(QMap<int, float>::const_iterator i=dataMap.begin(); i!=dataMap.end(); ++i)
		{
			((PdfPlotItem*)*iter)->setLikelihood(i.key(),  i.value(), weightsMap.value(i.key(), -1));
			//2 times...
			++iter;
			++iter;
		}

		//reset minMax, this will force the plot to update the axes
		this->updateMinMax();
		Q_EMIT dataChanged(this);
	}
}

}
