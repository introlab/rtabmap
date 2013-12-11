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

#ifndef PDFPLOT_H_
#define PDFPLOT_H_

#include <utilite/UPlot.h>
#include "opencv2/opencv.hpp"

namespace rtabmap {

class PdfPlotItem : public UPlotItem
{
public:
	PdfPlotItem(float dataX, float dataY, float width, int childCount = -1);
	virtual ~PdfPlotItem();

	void setLikelihood(int id, float value, int childCount);
	void setImagesRef(const QMap<int, std::vector<unsigned char> > * imagesRef) {_imagesRef = imagesRef;}

	float value() const {return this->data().y();}
	int id() const {return this->data().x();}

protected:
	virtual void showDescription(bool shown);

private:
	QGraphicsPixmapItem * _img;
	int _childCount;
	const QMap<int, std::vector<unsigned char> > * _imagesRef;
	QGraphicsTextItem * _text;

};

class PdfPlotCurve : public UPlotCurve
{
	Q_OBJECT

public:
	PdfPlotCurve(const QString & name, const QMap<int, std::vector<unsigned char> > * imagesMapRef, QObject * parent = 0);
	virtual ~PdfPlotCurve();

	virtual void clear();
	void setData(const QMap<int, float> & dataMap, const QMap<int, int> & weightsMap);

private:
	const QMap<int, std::vector<unsigned char> > * _imagesMapRef;
};

}

#endif /* PDFPLOT_H_ */
