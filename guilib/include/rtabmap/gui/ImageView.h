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

#ifndef IMAGEVIEW_H_
#define IMAGEVIEW_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <QGraphicsView>
#include <QtCore/QRectF>
#include <QtCore/QMultiMap>
#include <QtCore/QSettings>
#include <opencv2/features2d/features2d.hpp>
#include <map>

class QAction;
class QMenu;

namespace rtabmap {

class KeypointItem;

class RTABMAPGUI_EXP ImageView : public QWidget {

	Q_OBJECT

public:
	ImageView(QWidget * parent = 0);
	virtual ~ImageView();

	void saveSettings(QSettings & settings, const QString & group = "") const;
	void loadSettings(QSettings & settings, const QString & group = "");

	QRectF sceneRect() const;
	bool isImageShown() const;
	bool isImageDepthShown() const;
	bool isFeaturesShown() const;
	bool isLinesShown() const;
	int getAlpha() const {return _alpha;}
	bool isGraphicsViewMode() const;
	bool isGraphicsViewScaled() const;
	const QColor & getBackgroundColor() const;

	float viewScale() const;

	void setFeaturesShown(bool shown);
	void setImageShown(bool shown);
	void setImageDepthShown(bool shown);
	void setLinesShown(bool shown);
	void setGraphicsViewMode(bool on);
	void setGraphicsViewScaled(bool scaled);
	void setBackgroundColor(const QColor & color);

	void setFeatures(const std::multimap<int, cv::KeyPoint> & refWords, const cv::Mat & depth = cv::Mat(), const QColor & color = Qt::yellow);
	void setFeatures(const std::vector<cv::KeyPoint> & features, const cv::Mat & depth = cv::Mat(), const QColor & color = Qt::yellow);
	void addFeature(int id, const cv::KeyPoint & kpt, float depth, QColor color);
	void addLine(float x1, float y1, float x2, float y2, QColor color, const QString & text = QString());
	void setImage(const QImage & image);
	void setImageDepth(const QImage & image);
	void setFeatureColor(int id, QColor color);
	void setFeaturesColor(QColor color);
	void setAlpha(int alpha);
	void setSceneRect(const QRectF & rect);

	const QMultiMap<int, rtabmap::KeypointItem *> & getFeatures() const {return _features;}

	void clearLines();
	void clearFeatures();
	void clear();

	virtual QSize sizeHint() const;

signals:
	void configChanged();

protected:
	virtual void paintEvent(QPaintEvent *event);
	virtual void resizeEvent(QResizeEvent* event);
	virtual void contextMenuEvent(QContextMenuEvent * e);

private slots:
	void sceneRectChanged(const QRectF &rect);

private:
	void updateOpacity();
	void computeScaleOffsets(const QRect & targetRect, float & scale, float & offsetX, float & offsetY) const;

private:
	QString _savedFileName;
	int _alpha;

	QMenu * _menu;
	QAction * _showImage;
	QAction * _showImageDepth;
	QAction * _showFeatures;
	QAction * _showLines;
	QAction * _saveImage;
	QAction * _setAlpha;
	QAction * _graphicsViewMode;
	QAction * _graphicsViewScaled;

	QGraphicsView * _graphicsView;
	QMultiMap<int, rtabmap::KeypointItem *> _features;
	QList<QGraphicsLineItem*> _lines;
	QGraphicsPixmapItem * _imageItem;
	QGraphicsPixmapItem * _imageDepthItem;
	QPixmap _image;
	QPixmap _imageDepth;
};

}

#endif /* IMAGEVIEW_H_ */
