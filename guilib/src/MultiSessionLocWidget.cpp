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

#include "rtabmap/gui/MultiSessionLocWidget.h"
#include "rtabmap/gui/MultiSessionLocSubView.h"
#include "rtabmap/gui/ImageView.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/core/Graph.h"

#include <QHBoxLayout>
#include <QPushButton>
#include <QProgressBar>

namespace rtabmap {

MultiSessionLocWidget::MultiSessionLocWidget(
		const QMap<int, Signature> * cache,
		const std::map<int, int> * mapIds,
		QWidget * parent) :
				QWidget(parent),
				cache_(cache),
				mapIds_(mapIds),
				totalFrames_(0),
				totalLoops_(0)
{
	UASSERT(cache != 0);
	UASSERT(mapIds != 0);
	imageView_ = new ImageView(this);
	imageView_->setObjectName("multisession_imageview");
	totalLocProgressBar_ = new QProgressBar(this);
	resetbutton_ = new QPushButton(this);
	resetbutton_->setText("Reset");

	// setup layout
	this->setLayout(new QHBoxLayout());
	QVBoxLayout * vLayout = new QVBoxLayout();
	vLayout->addWidget(imageView_, 1);
	QHBoxLayout * hLayout = new QHBoxLayout();
	hLayout->addWidget(resetbutton_, 0);
	hLayout->addWidget(totalLocProgressBar_, 1);
	vLayout->addLayout(hLayout, 0);
	((QHBoxLayout*)this->layout())->addLayout(vLayout, 1);

	connect(resetbutton_, SIGNAL(clicked()), this, SLOT(clear()));
}
MultiSessionLocWidget::~MultiSessionLocWidget() {}

void MultiSessionLocWidget::updateView(
		const Signature & lastSignature,
		const Statistics & stats)
{
	++totalFrames_;
	std::multimap<int, Link> loopLinks = graph::filterLinks(lastSignature.getLinks(), Link::kGlobalClosure, true);
	std::multimap<int, Link> localLinks = graph::filterLinks(lastSignature.getLinks(), Link::kLocalSpaceClosure, true);
	loopLinks.insert(localLinks.begin(), localLinks.end());

	if(!loopLinks.empty())
	{
		++totalLoops_;
		totalLocProgressBar_->setValue(float(totalLoops_)/float(totalFrames_) * 100);
	}

	if(!lastSignature.sensorData().imageRaw().empty() ||
	   !lastSignature.sensorData().imageCompressed().empty())
	{
		cv::Mat image;
		lastSignature.sensorData().uncompressDataConst(&image, 0);
		if(!image.empty())
		{
			imageView_->setImage(uCvMat2QImage(image));
			imageView_->setFeatures(lastSignature.getWordsKpts(), cv::Mat(), imageView_->getDefaultMatchingFeatureColor());
			imageView_->setBackgroundColor(Qt::black);
		}
		else
		{
			imageView_->clear();
		}
	}
	else
	{
		imageView_->clear();
	}

	std::map<int, std::pair<int, float> > top;
	const std::map<int, float> & likelihood = stats.posterior();
	for(std::map<int, float>::const_iterator iter=likelihood.begin(); iter!=likelihood.end(); ++iter)
	{
		if(mapIds_->find(iter->first) != mapIds_->end())
		{
			int mapId = mapIds_->at(iter->first);
			if(subViews_.find(mapId)==subViews_.end())
			{
				MultiSessionLocSubView * subView = new MultiSessionLocSubView(imageView_, mapId, this);
				((QHBoxLayout*)this->layout())->addWidget(subView, 1);
				subViews_.insert(std::make_pair(mapId, std::make_pair(subView, 0)));
			}

			if(top.find(mapId) == top.end() || top.at(mapId).second < iter->second)
			{
				uInsert(top, std::make_pair(mapId, std::make_pair(iter->first, iter->second)));
			}
		}
	}

	// update highest loop closure hypotheses per session
	for(std::map<int, std::pair<MultiSessionLocSubView*, int> >::iterator iter=subViews_.begin(); iter!=subViews_.end(); ++iter)
	{

		for(std::multimap<int, Link>::iterator jter=loopLinks.begin(); jter!=loopLinks.end(); ++jter)
		{
			if(mapIds_->find(jter->first)!= mapIds_->end())
			{
				int mapId = mapIds_->at(jter->first);
				iter->second.second += mapId==iter->first?1:0;
			}
		}

		if(uContains(top, iter->first))
		{
			int nodeId = top.at(iter->first).first;
			if(cache_->contains(nodeId))
			{
				cv::Mat image;
				const Signature & s = (*cache_)[nodeId];

				Link link = loopLinks.find(nodeId) != loopLinks.end()?loopLinks.find(nodeId)->second:Link();
				std::multimap<int, cv::KeyPoint> keypoints;
				for(std::multimap<int, int>::const_iterator jter=s.getWords().begin(); jter!=s.getWords().end(); ++jter)
				{
					if(jter->first>0 && lastSignature.getWords().find(jter->first) != lastSignature.getWords().end())
					{
						keypoints.insert(std::make_pair(jter->first, s.getWordsKpts()[jter->second]));
					}
				}

				if(!keypoints.empty())
				{
					s.sensorData().uncompressDataConst(&image, 0);
					if(!image.empty())
					{
						iter->second.first->updateView(
								nodeId,
								uCvMat2QImage(image),
								keypoints,
								float(iter->second.second)/float(totalFrames_),
								link.type() == Link::kLocalSpaceClosure?Qt::yellow:
								link.type() == Link::kGlobalClosure?Qt::green:Qt::gray);
					}
					else
					{
						iter->second.first->clear();
					}
				}
				else
				{
					iter->second.first->clear();
				}
			}
			else
			{
				iter->second.first->clear();
			}
		}
		else
		{
			iter->second.first->clear();
		}
	}
}
void MultiSessionLocWidget::clear()
{
	for(std::map<int, std::pair<MultiSessionLocSubView*, int> >::iterator iter=subViews_.begin(); iter!=subViews_.end(); ++iter)
	{
		delete iter->second.first;
	}
	subViews_.clear();
	imageView_->clear();
	totalFrames_ = 0;
	totalLoops_ = 0;
	totalLocProgressBar_->setValue(0);
}

} /* namespace rtabmap */
