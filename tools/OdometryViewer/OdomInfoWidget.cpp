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

#include "OdomInfoWidget.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/gui/ImageView.h>
#include <rtabmap/gui/UCv2Qt.h>
#include <QtCore/QMetaType>
#include <QtGui/QCloseEvent>
#include <QLabel>
#include <QVBoxLayout>

OdomInfoWidget::OdomInfoWidget(QWidget * parent) :
		QWidget(parent),
	imageView_(new rtabmap::ImageView(this)),
	label_(new QLabel(this)),
	processingOdomInfo_(false),
	receivingRate_(0),
	lastTime_(0),
	odomImageShow_(true),
	odomImageDepthShow_(false)
{
	qRegisterMetaType<rtabmap::SensorData>("rtabmap::SensorData");
	qRegisterMetaType<rtabmap::OdometryInfo>("rtabmap::OdometryInfo");

	imageView_->setMinimumSize(320, 240);
	QVBoxLayout * layout = new QVBoxLayout(this);
	layout->setMargin(0);
	layout->addWidget(imageView_);
	layout->addWidget(label_);
	layout->setStretch(0,1);
	this->setLayout(layout);
}


OdomInfoWidget::~OdomInfoWidget()
{
	this->unregisterFromEventsManager();
}


void OdomInfoWidget::processOdomInfo(const rtabmap::SensorData & data, const rtabmap::OdometryInfo & info)
{
	processingOdomInfo_ = true;

	const rtabmap::Transform & pose = data.pose();
	bool lost = false;
	bool lostStateChanged = false;

	if(pose.isNull())
	{
		// lost
		lostStateChanged = imageView_->getBackgroundColor() != Qt::darkRed;
		imageView_->setBackgroundColor(Qt::darkRed);
		lost = true;
	}
	else
	{
		// ok
		lostStateChanged = imageView_->getBackgroundColor() == Qt::darkRed;
		imageView_->setBackgroundColor(Qt::black);
	}

	if(!data.image().empty())
	{
		if(imageView_->isFeaturesShown())
		{
			if(info.type == 0)
			{
				imageView_->setFeatures(info.words, Qt::yellow);
			}
			else if(info.type == 1)
			{
				imageView_->setFeatures(info.refCorners, Qt::red);
			}
		}

		imageView_->clearLines();
		if(lost)
		{
			if(lostStateChanged)
			{
				// save state
				odomImageShow_ = imageView_->isImageShown();
				odomImageDepthShow_ = imageView_->isImageDepthShown();
			}
			imageView_->setImageDepth(uCvMat2QImage(data.image()));
			imageView_->setImageShown(true);
			imageView_->setImageDepthShown(true);
		}
		else
		{
			if(lostStateChanged)
			{
				// restore state
				imageView_->setImageShown(odomImageShow_);
				imageView_->setImageDepthShown(odomImageDepthShow_);
			}

			imageView_->setImage(uCvMat2QImage(data.image()));
			if(imageView_->isImageDepthShown())
			{
				imageView_->setImageDepth(uCvMat2QImage(data.depthOrRightImage()));
			}

			if(info.type == 0)
			{
				if(imageView_->isFeaturesShown())
				{
					for(unsigned int i=0; i<info.wordMatches.size(); ++i)
					{
						imageView_->setFeatureColor(info.wordMatches[i], Qt::red); // outliers
					}
					for(unsigned int i=0; i<info.wordInliers.size(); ++i)
					{
						imageView_->setFeatureColor(info.wordInliers[i], Qt::green); // inliers
					}
				}
			}
			else if(info.type == 1)
			{
				if(imageView_->isFeaturesShown() || imageView_->isLinesShown())
				{
					//draw lines
					UASSERT(info.refCorners.size() == info.newCorners.size());
					for(unsigned int i=0; i<info.cornerInliers.size(); ++i)
					{
						if(imageView_->isFeaturesShown())
						{
							imageView_->setFeatureColor(info.cornerInliers[i], Qt::green); // inliers
						}
						if(imageView_->isLinesShown())
						{
							imageView_->addLine(
									info.refCorners[info.cornerInliers[i]].pt.x,
									info.refCorners[info.cornerInliers[i]].pt.y,
									info.newCorners[info.cornerInliers[i]].pt.x,
									info.newCorners[info.cornerInliers[i]].pt.y,
									Qt::blue);
						}
					}
					imageView_->update();
				}
			}

		}
		if(!data.image().empty())
		{
			imageView_->setSceneRect(QRectF(0,0,(float)data.image().cols, (float)data.image().rows));
		}
	}
	label_->setText(tr("Rate=~%1 Hz").arg(receivingRate_));
	processingOdomInfo_ = false;
}

void OdomInfoWidget::closeEvent(QCloseEvent* event)
{
	this->unregisterFromEventsManager();
	event->accept();
}

void OdomInfoWidget::handleEvent(UEvent * event)
{
	if(event->getClassName().compare("OdometryEvent") == 0)
	{
		rtabmap::OdometryEvent * odomEvent = (rtabmap::OdometryEvent*)event;
		receivingRate_ = 1.0f/timer_.ticks();

		// update max 10 Hz
		if(UTimer::now() - lastTime_ > 0.1)
		{
			lastTime_ = UTimer::now();
			if(!processingOdomInfo_ && this->isVisible())
			{
				QMetaObject::invokeMethod(this, "processOdomInfo",
						Q_ARG(rtabmap::SensorData, odomEvent->data()),
						Q_ARG(rtabmap::OdometryInfo, odomEvent->info()));
			}
		}
	}
}
