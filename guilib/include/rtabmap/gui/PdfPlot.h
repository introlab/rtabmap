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

#ifndef RTABMAP_PDFPLOT_H_
#define RTABMAP_PDFPLOT_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <rtabmap/utilite/UPlot.h>
#include "opencv2/opencv.hpp"
#include "rtabmap/core/Signature.h"

namespace rtabmap {

class RTABMAP_GUI_EXPORT PdfPlotItem : public UPlotItem
{
public:
	PdfPlotItem(float dataX, float dataY, float width, int childCount = -1);
	virtual ~PdfPlotItem();

	void setLikelihood(int id, float value, int childCount);
	void setSignaturesRef(const QMap<int, Signature> * signaturesRef) {_signaturesRef = signaturesRef;}

	float value() const {return this->data().y();}
	int id() const {return this->data().x();}

protected:
	virtual void showDescription(bool shown);

private:
	QGraphicsPixmapItem * _img;
	int _childCount;
	const QMap<int, Signature> * _signaturesRef;
	QGraphicsTextItem * _text;

};

class RTABMAP_GUI_EXPORT PdfPlotCurve : public UPlotCurve
{
	Q_OBJECT

public:
	PdfPlotCurve(const QString & name, const QMap<int, Signature> * signaturesMapRef, QObject * parent = 0);
	virtual ~PdfPlotCurve();

	virtual void clear();
	void setData(const QMap<int, float> & dataMap, const QMap<int, int> & weightsMap);

private:
	const QMap<int, Signature> * _signaturesMapRef;
};

}

#endif /* PDFPLOT_H_ */
