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

#ifndef RTABMAP_KEYPOINTITEM_H_
#define RTABMAP_KEYPOINTITEM_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <opencv2/features2d/features2d.hpp>

namespace rtabmap {

class RTABMAP_GUI_EXPORT KeypointItem : public QGraphicsEllipseItem
{
public:
	KeypointItem(int id, const cv::KeyPoint & kpt, float depth = 0, const QColor & color = Qt::green, QGraphicsItem * parent = 0);
	virtual ~KeypointItem();

	void setColor(const QColor & color);
	const cv::KeyPoint & keypoint() const {return _kpt;}

protected:
	virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event );
	virtual void hoverLeaveEvent ( QGraphicsSceneHoverEvent * event );
	virtual void focusInEvent ( QFocusEvent * event );
	virtual void focusOutEvent ( QFocusEvent * event );

private:
	void showDescription();
	void hideDescription();

private:
	int _id;
	cv::KeyPoint _kpt;
	QGraphicsRectItem * _placeHolder;
	int _width;
	float _depth;
};

}

#endif /* KEYPOINTITEM_H_ */
