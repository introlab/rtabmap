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

#ifndef IMAGEVIEW_H_
#define IMAGEVIEW_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <QtGui/QGraphicsView>
#include <QtCore/QRectF>
#include <QtCore/QMultiMap>
#include <opencv2/features2d/features2d.hpp>
#include <map>

class QAction;
class QMenu;

namespace rtabmap {

class KeypointItem;

class RTABMAPGUI_EXP ImageView : public QGraphicsView {

	Q_OBJECT

public:
	ImageView(QWidget * parent = 0);
	virtual ~ImageView();

	void resetZoom();

	bool isImageShown();
	bool isImageDepthShown();
	bool isFeaturesShown();
	bool isLinesShown();

	void setFeaturesShown(bool shown);
	void setImageShown(bool shown);
	void setImageDepthShown(bool shown);
	void setLinesShown(bool shown);

	void setFeatures(const std::multimap<int, cv::KeyPoint> & refWords);
	void setImage(const QImage & image);
	void setImageDepth(const QImage & image);
	void setFeatureColor(int id, const QColor & color);
	void setFeaturesColor(const QColor & color);

	const QMultiMap<int, rtabmap::KeypointItem *> & getFeatures() const {return _features;}

	void clearLines();
	void clear();

protected:
	virtual void contextMenuEvent(QContextMenuEvent * e);
	virtual void wheelEvent(QWheelEvent * e);

private slots:
	void updateZoom();

private:
	void updateOpacity();

private:
	int _zoom;
	int _minZoom;
	QString _savedFileName;

	QMenu * _menu;
	QAction * _showImage;
	QAction * _showImageDepth;
	QAction * _showFeatures;
	QAction * _showLines;
	QAction * _saveImage;

	QMultiMap<int, rtabmap::KeypointItem *> _features;
	QGraphicsPixmapItem * _image;
	QGraphicsPixmapItem * _imageDepth;
};

}

#endif /* IMAGEVIEW_H_ */
