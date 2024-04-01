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

#include "rtabmap/gui/EditDepthArea.h"
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

EditDepthArea::EditDepthArea(QWidget *parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_StaticContents);
    modified_ = false;
    scribbling_ = false;
    myPenWidth_ = 10;
    clusterError_ = 0.02;

    menu_ = new QMenu(tr(""), this);
    showRGB_ = menu_->addAction(tr("Show RGB Image"));
    showRGB_->setCheckable(true);
    showRGB_->setChecked(true);
    removeCluster_ = menu_->addAction(tr("Remove Cluster"));
    clusterErrorCluster_ = menu_->addAction(tr("Set Cluster Error"));
	setPenWidth_ = menu_->addAction(tr("Set Pen Width..."));
	QMenu * colorMap = menu_->addMenu("Depth color map");
	colorMapWhiteToBlack_ = colorMap->addAction(tr("White to black"));
	colorMapWhiteToBlack_->setCheckable(true);
	colorMapWhiteToBlack_->setChecked(false);
	colorMapBlackToWhite_ = colorMap->addAction(tr("Black to white"));
	colorMapBlackToWhite_->setCheckable(true);
	colorMapBlackToWhite_->setChecked(false);
	colorMapRedToBlue_ = colorMap->addAction(tr("Red to blue"));
	colorMapRedToBlue_->setCheckable(true);
	colorMapRedToBlue_->setChecked(true);
	colorMapBlueToRed_ = colorMap->addAction(tr("Blue to red"));
	colorMapBlueToRed_->setCheckable(true);
	colorMapBlueToRed_->setChecked(false);
	QActionGroup * group = new QActionGroup(this);
	group->addAction(colorMapWhiteToBlack_);
	group->addAction(colorMapBlackToWhite_);
	group->addAction(colorMapRedToBlue_);
	group->addAction(colorMapBlueToRed_);
	resetChanges_ = menu_->addAction(tr("Reset Changes"));
}

void EditDepthArea::setImage(const cv::Mat &depth, const cv::Mat & rgb)
{
	UASSERT(!depth.empty());
	UASSERT(depth.type() == CV_32FC1 ||
			depth.type() == CV_16UC1);
	originalImage_ = depth;

	uCvQtDepthColorMap colorMap = uCvQtDepthWhiteToBlack;
	if(colorMapBlackToWhite_->isChecked())
	{
		colorMap = uCvQtDepthBlackToWhite;
	}
	else if(colorMapRedToBlue_->isChecked())
	{
		colorMap = uCvQtDepthRedToBlue;
	}
	else if(colorMapBlueToRed_->isChecked())
	{
		colorMap = uCvQtDepthBlueToRed;
	}

	image_ = uCvMat2QImage(depth, true, colorMap).convertToFormat(QImage::Format_RGB32);

	imageRGB_ = QImage();
	if(!rgb.empty())
	{
		imageRGB_ = uCvMat2QImage(rgb);
		if( depth.cols != rgb.cols ||
			depth.rows != rgb.rows)
		{
			// scale rgb to depth
			imageRGB_ = imageRGB_.scaled(image_.size());
		}
	}
	showRGB_->setEnabled(!imageRGB_.isNull());
	modified_ = false;
	update();
}

cv::Mat EditDepthArea::getModifiedImage() const
{
	cv::Mat modifiedImage = originalImage_.clone();
	if(modified_)
	{
		UASSERT(image_.width() == modifiedImage.cols &&
				image_.height() == modifiedImage.rows);
		UASSERT(modifiedImage.type() == CV_32FC1 ||
				modifiedImage.type() == CV_16UC1);
		for(int j=0; j<image_.height(); ++j)
		{
			for(int i=0; i<image_.width(); ++i)
			{
				if(qRed(image_.pixel(i, j)) == 0 &&
				   qGreen(image_.pixel(i, j)) == 0 &&
				   qBlue(image_.pixel(i, j)) == 0)
				{
					if(modifiedImage.type() == CV_32FC1)
					{
						modifiedImage.at<float>(j,i) = 0.0f;
					}
					else // CV_16UC1
					{
						modifiedImage.at<unsigned short>(j,i) = 0;
					}
				}
			}
		}
	}
	return modifiedImage;
}

void EditDepthArea::setPenWidth(int newWidth)
{
    myPenWidth_ = newWidth;
}

void EditDepthArea::resetChanges()
{
    image_ = uCvMat2QImage(originalImage_).convertToFormat(QImage::Format_RGB32);
    modified_ = false;
    update();
}

void EditDepthArea::setColorMap(uCvQtDepthColorMap type)
{
	if(type == uCvQtDepthBlackToWhite)
	{
		colorMapBlackToWhite_->setChecked(true);
	}
	else if(type == uCvQtDepthRedToBlue)
	{
		colorMapRedToBlue_->setChecked(true);
	}
	else if(type == uCvQtDepthBlueToRed)
	{
		colorMapBlueToRed_->setChecked(true);
	}
	else
	{
		colorMapWhiteToBlack_->setChecked(true);
	}

	if(!originalImage_.empty())
	{
		image_ = uCvMat2QImage(originalImage_, true, type).convertToFormat(QImage::Format_RGB32);
		update();
	}
}

void EditDepthArea::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
    	float scale, offsetX, offsetY;
		computeScaleOffsets(rect(), scale, offsetX, offsetY);
        lastPoint_.setX((event->pos().x()-offsetX)/scale);
        lastPoint_.setY((event->pos().y()-offsetY)/scale);

        scribbling_ = true;
    }
}

void EditDepthArea::mouseMoveEvent(QMouseEvent *event)
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

void EditDepthArea::mouseReleaseEvent(QMouseEvent *event)
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

void EditDepthArea::computeScaleOffsets(const QRect & targetRect, float & scale, float & offsetX, float & offsetY) const
{
	scale = 1.0f;
	offsetX = 0.0f;
	offsetY = 0.0f;

	if(!image_.isNull())
	{
		float w = image_.width();
		float h = image_.height();
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

void EditDepthArea::paintEvent(QPaintEvent *event)
{
	//Scale
	float ratio, offsetX, offsetY;
	this->computeScaleOffsets(event->rect(), ratio, offsetX, offsetY);
	QPainter painter(this);

	painter.translate(offsetX, offsetY);
	painter.scale(ratio, ratio);

	if(showRGB_->isChecked() && !imageRGB_.isNull())
	{
		painter.setOpacity(0.5);
		painter.drawImage(QPoint(0,0), imageRGB_);
	}

	painter.drawImage(QPoint(0,0), image_);
}

void EditDepthArea::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
}

void floodfill(QRgb * bits, const cv::Mat & depthImage, int x, int y, float lastDepthValue, float error, int &iterations)
{
	++iterations;
	if(x>=0 && x<depthImage.cols &&
	   y>=0 && y<depthImage.rows)
	{
		float currentValue;
		if(depthImage.type() == CV_32FC1)
		{
			currentValue = depthImage.at<float>(y, x);
		}
		else
		{
			currentValue = float(depthImage.at<unsigned short>(y, x))/1000.0f;
		}
		if(currentValue == 0.0f)
		{
			return;
		}

		QRgb & rgb = bits[x+y*depthImage.cols];
		if(qRed(rgb) == 0 && qGreen(rgb) == 0 && qBlue(rgb) == 0)
		{
			return;
		}
		if(lastDepthValue>=0.0f && fabs(lastDepthValue - currentValue) > error*lastDepthValue)
		{
			return;
		}
		rgb = 0;

		if(y+1<depthImage.rows)
		{
			QRgb & rgb = bits[x+(y+1)*depthImage.cols];
			if(qRed(rgb) != 0 || qGreen(rgb) != 0 || qBlue(rgb) != 0)
			{
				floodfill(bits, depthImage, x, y+1, currentValue, error, iterations);
			}
		}
		if(y-1>=0)
		{
			QRgb & rgb = bits[x+(y-1)*depthImage.cols];
			if(qRed(rgb) != 0 || qGreen(rgb) != 0 || qBlue(rgb) != 0)
			{
				floodfill(bits, depthImage, x, y-1, currentValue, error, iterations);
			}
		}
		if(x+1<depthImage.cols)
		{
			QRgb & rgb = bits[x+1+y*depthImage.cols];
			if(qRed(rgb) != 0 || qGreen(rgb) != 0 || qBlue(rgb) != 0)
			{
				floodfill(bits, depthImage, x+1, y, currentValue, error, iterations);
			}
		}
		if(x-1>=0)
		{
			QRgb & rgb = bits[x-1+y*depthImage.cols];
			if(qRed(rgb) != 0 || qGreen(rgb) != 0 || qBlue(rgb) != 0)
			{
				floodfill(bits, depthImage, x-1, y, currentValue, error, iterations);
			}
		}
	}
}

void EditDepthArea::contextMenuEvent(QContextMenuEvent * e)
{
	QAction * action = menu_->exec(e->globalPos());
	if(action == showRGB_)
	{
		this->update();
	}
	else if(action == removeCluster_)
	{
		float scale, offsetX, offsetY;
		computeScaleOffsets(rect(), scale, offsetX, offsetY);
		QPoint pixel;
		pixel.setX((e->pos().x()-offsetX)/scale);
		pixel.setY((e->pos().y()-offsetY)/scale);
		if(pixel.x()>=0 && pixel.x() < originalImage_.cols &&
		   pixel.y()>=0 && pixel.y() < originalImage_.rows)
		{
			int iterations=0;
			floodfill((QRgb*)image_.bits(), originalImage_, pixel.x(), pixel.y(), -1.0f, clusterError_, iterations);
		}
		modified_=true;
		this->update();
	}
	else if(action == clusterErrorCluster_)
	{
		bool ok;
		double error = QInputDialog::getDouble(this, tr("Set Cluster Error"), tr("Error:"), clusterError(), 0.001, 1, 3, &ok);
		if(ok)
		{
			clusterError_= error;
		}
		modified_=true;
	}
	else if(action == setPenWidth_)
	{
		bool ok;
		int width = QInputDialog::getInt(this, tr("Set Pen Width"), tr("Width (pixels):"), penWidth(), 1, 999, 1, &ok);
		if(ok)
		{
			myPenWidth_ = width;
		}
	}
	else if(action == colorMapBlackToWhite_ ||
			action == colorMapWhiteToBlack_ ||
			action == colorMapRedToBlue_ ||
			action == colorMapBlueToRed_)
	{
		uCvQtDepthColorMap colorMap = uCvQtDepthWhiteToBlack;
		if(colorMapBlackToWhite_->isChecked())
		{
			colorMap = uCvQtDepthBlackToWhite;
		}
		else if(colorMapRedToBlue_->isChecked())
		{
			colorMap = uCvQtDepthRedToBlue;
		}
		else if(colorMapBlueToRed_->isChecked())
		{
			colorMap = uCvQtDepthBlueToRed;
		}
		this->setColorMap(colorMap);
	}
	else if(action == resetChanges_)
	{
		this->resetChanges();
	}
}

void EditDepthArea::drawLineTo(const QPoint &endPoint)
{
    QPainter painter(&image_);
    painter.setPen(QPen(Qt::black, myPenWidth_, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawLine(lastPoint_, endPoint);
    modified_ = true;

    update();
    lastPoint_ = endPoint;
}

}
