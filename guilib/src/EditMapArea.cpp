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

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QMenu>
#include <QAction>
#include <QActionGroup>
#include <QInputDialog>

#include "rtabmap/gui/EditMapArea.h"
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

EditMapArea::EditMapArea(QWidget *parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_StaticContents);
    modified_ = false;
    scribbling_ = false;
    myPenWidth_ = 3;

    menu_ = new QMenu(tr(""), this);
	setPenWidth_ = menu_->addAction(tr("Set Pen Width..."));
	addObstacle_ = menu_->addAction(tr("Add Obstacle"));
	addObstacle_->setCheckable(true);
	addObstacle_->setChecked(true);
	clearObstacle_ = menu_->addAction(tr("Clear Obstacle"));
	clearObstacle_->setCheckable(true);
	clearObstacle_->setChecked(false);
	setUnknown_ = menu_->addAction(tr("Set Unknown"));
	setUnknown_->setCheckable(true);
	setUnknown_->setChecked(false);
	QActionGroup * group = new QActionGroup(this);
	group->addAction(addObstacle_);
	group->addAction(clearObstacle_);
	group->addAction(setUnknown_);
	resetChanges_ = menu_->addAction(tr("Reset Changes"));
}

void EditMapArea::setMap(const cv::Mat &map)
{
	UASSERT(!map.empty());
	UASSERT(map.type() == CV_8UC1);
	originalMap_ = map;

	map_ = uCvMat2QImage(map, true).convertToFormat(QImage::Format_RGB32);

	modified_ = false;
	update();
}

cv::Mat EditMapArea::getModifiedMap() const
{
	cv::Mat modifiedMap = originalMap_.clone();
	if(modified_)
	{
		UASSERT(map_.width() == modifiedMap.cols &&
				map_.height() == modifiedMap.rows);
		UASSERT(modifiedMap.type() == CV_8UC1);
		for(int j=0; j<map_.height(); ++j)
		{
			for(int i=0; i<map_.width(); ++i)
			{
				modifiedMap.at<unsigned char>(j,i) = qRed(map_.pixel(i, j));
			}
		}
	}
	return modifiedMap;
}

void EditMapArea::setPenWidth(int newWidth)
{
    myPenWidth_ = newWidth;
}

void EditMapArea::resetChanges()
{
    map_ = uCvMat2QImage(originalMap_).convertToFormat(QImage::Format_RGB32);
    modified_ = false;
    update();
}

void EditMapArea::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
    	float scale, offsetX, offsetY;
		computeScaleOffsets(rect(), scale, offsetX, offsetY);
        lastPoint_.setX((event->pos().x()-offsetX)/scale);
        lastPoint_.setY((event->pos().y()-offsetY)/scale);

        scribbling_ = true;
    }
}

void EditMapArea::mouseMoveEvent(QMouseEvent *event)
{
    if ((event->buttons() & Qt::LeftButton) && scribbling_)
    {
    	float scale, offsetX, offsetY;
		computeScaleOffsets(rect(), scale, offsetX, offsetY);
		QPoint to;
		to.setX((event->pos().x()-offsetX)/scale);
		to.setY((event->pos().y()-offsetY)/scale);
        drawLineTo(to);
    }
}

void EditMapArea::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && scribbling_) {
    	float scale, offsetX, offsetY;
		computeScaleOffsets(rect(), scale, offsetX, offsetY);
		QPoint to;
		to.setX((event->pos().x()-offsetX)/scale);
		to.setY((event->pos().y()-offsetY)/scale);
		drawLineTo(to);
        scribbling_ = false;
    }
}

void EditMapArea::computeScaleOffsets(const QRect & targetRect, float & scale, float & offsetX, float & offsetY) const
{
	scale = 1.0f;
	offsetX = 0.0f;
	offsetY = 0.0f;

	if(!map_.isNull())
	{
		float w = map_.width();
		float h = map_.height();
		float widthRatio = float(targetRect.width()) / w;
		float heightRatio = float(targetRect.height()) / h;

		//printf("w=%f, h=%f, wR=%f, hR=%f, sW=%d, sH=%d\n", w, h, widthRatio, heightRatio, this->rect().width(), this->rect().height());
		if(widthRatio < heightRatio)
		{
			scale = widthRatio;
		}
		else
		{
			scale = heightRatio;
		}

		//printf("ratio=%f\n",ratio);

		w *= scale;
		h *= scale;

		if(w < targetRect.width())
		{
			offsetX = (targetRect.width() - w)/2.0f;
		}
		if(h < targetRect.height())
		{
			offsetY = (targetRect.height() - h)/2.0f;
		}
		//printf("offsetX=%f, offsetY=%f\n",offsetX, offsetY);
	}
}

void EditMapArea::paintEvent(QPaintEvent *event)
{
	//Scale
	float ratio, offsetX, offsetY;
	this->computeScaleOffsets(event->rect(), ratio, offsetX, offsetY);
	QPainter painter(this);

	painter.translate(offsetX, offsetY);
	painter.scale(ratio, ratio);
	painter.drawImage(QPoint(0,0), map_);
}

void EditMapArea::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
}

void EditMapArea::contextMenuEvent(QContextMenuEvent * e)
{
	QAction * action = menu_->exec(e->globalPos());
	if(action == setPenWidth_)
	{
		bool ok;
		int width = QInputDialog::getInt(this, tr("Set Pen Width"), tr("Width:"), penWidth(), 1, 99, 1, &ok);
		if(ok)
		{
			myPenWidth_ = width;
		}
	}
	else if(action == resetChanges_)
	{
		this->resetChanges();
	}
}

void EditMapArea::drawLineTo(const QPoint &endPoint)
{
    QPainter painter(&map_);
    QColor color;

    //base on util3d::convertMap2Image8U();
    if(addObstacle_->isChecked())
    {
    	color.setRgb(0,0,0);
    }
    else if(clearObstacle_->isChecked())
    {
    	color.setRgb(178,178,178);
    }
    else //unknown
    {
    	color.setRgb(89,89,89);
    }
    painter.setPen(QPen(color, myPenWidth_, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawLine(lastPoint_, endPoint);
    modified_ = true;

    update();
    lastPoint_ = endPoint;
}

}
