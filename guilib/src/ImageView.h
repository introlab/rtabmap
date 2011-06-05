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

#include <QtGui/QGraphicsView>
#include <QtCore/QRectF>

namespace rtabmap {

class ImageView : public QGraphicsView {

	Q_OBJECT

public:
	ImageView(QWidget * parent = 0);
	virtual ~ImageView();

	void resetZoom();

protected:
	virtual void contextMenuEvent(QContextMenuEvent * e);
	virtual void wheelEvent(QWheelEvent * e);

private slots:
	void updateZoom();

private:
	int _zoom;
	int _minZoom;
	QString _savedFileName;
};

}

#endif /* IMAGEVIEW_H_ */
