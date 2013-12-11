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

#ifndef IMAGEVIEW_H_
#define IMAGEVIEW_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <QtGui/QGraphicsView>
#include <QtCore/QRectF>
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

	QList<rtabmap::KeypointItem *> _features;
	QGraphicsPixmapItem * _image;
	QGraphicsPixmapItem * _imageDepth;
};

}

#endif /* IMAGEVIEW_H_ */
