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


#include "rtabmap/gui/MultiSessionLocSubView.h"
#include "ui_multiSessionLocSubView.h"

namespace rtabmap {

MultiSessionLocSubView::MultiSessionLocSubView(ImageView * mainView, int mapId, QWidget * parent) :
		QWidget(parent),
		mapId_(mapId)
{
	ui_ = new Ui_multiSessionLocSubView();
	ui_->setupUi(this);
	ui_->imageView->setFeaturesShown(mainView->isFeaturesShown());
	ui_->imageView->setFeaturesSize(mainView->getFeaturesSize());
	ui_->imageView->setAlpha(mainView->getAlpha());
	ui_->imageView->setDefaultMatchingFeatureColor(mainView->getDefaultMatchingFeatureColor());
}
MultiSessionLocSubView::~MultiSessionLocSubView() {}

void MultiSessionLocSubView::updateView(
		int nodeId,
		const QImage & image,
		const std::multimap<int, cv::KeyPoint> & features,
		float locRatio,
		const QColor & bgColor)
{
	if(image.isNull())
	{
		ui_->imageView->clear();
	}
	else
	{
		ui_->imageView->setImage(image);
		ui_->imageView->setFeatures(features, cv::Mat(), ui_->imageView->getDefaultMatchingFeatureColor());
		ui_->imageView->setBackgroundColor(bgColor);
	}
	ui_->label->setText(QString("%1 [%2]").arg(nodeId).arg(mapId_));
	ui_->locProgressBar->setValue(locRatio * 100);
}

void MultiSessionLocSubView::clear()
{
	ui_->imageView->clear();
	ui_->label->setText(QString("[%1]").arg(mapId_));
}

} /* namespace rtabmap */

