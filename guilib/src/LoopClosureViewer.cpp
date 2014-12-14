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

#include "rtabmap/gui/LoopClosureViewer.h"
#include "ui_loopClosureViewer.h"

#include "rtabmap/core/Memory.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UStl.h"

#include <QtCore/QTimer>

namespace rtabmap {

LoopClosureViewer::LoopClosureViewer(QWidget * parent) :
	QWidget(parent),
	decimation_(1),
	maxDepth_(0),
	samples_(0)
{
	ui_ = new Ui_loopClosureViewer();
	ui_->setupUi(this);
	ui_->cloudViewerTransform->setCameraLockZ(false);

	connect(ui_->checkBox_rawCloud, SIGNAL(clicked()), this, SLOT(updateView()));
}

LoopClosureViewer::~LoopClosureViewer() {
	delete ui_;
}

void LoopClosureViewer::setData(const Signature & sA, const Signature & sB)
{
	sA_ = sA;
	sB_ = sB;
	if(sA_.id()>0 && sB_.id()>0)
	{
		ui_->label_idA->setText(QString("[%1-%2]").arg(sA.id()).arg(sB.id()));
	}
}

void LoopClosureViewer::updateView(const Transform & transform)
{
	if(sA_.id()>0 && sB_.id()>0)
	{
		int decimation = 1;
		float maxDepth = 0;
		int samples = 0;

		if(!ui_->checkBox_rawCloud->isChecked())
		{			decimation = decimation_;
			maxDepth = maxDepth_;
			samples = samples_;
		}

		UDEBUG("decimation = %d", decimation);
		UDEBUG("maxDepth = %f", maxDepth);
		UDEBUG("samples = %d", samples);

		Transform t;
		if(!transform.isNull())
		{
			transform_ = transform;
			t = transform;
		}
		else if(!transform_.isNull())
		{
			t = transform_;
		}
		else
		{
			t = sB_.getPose();
		}

		UDEBUG("t= %s", t.prettyPrint().c_str());
		ui_->label_transform->setText(QString("(%1)").arg(t.prettyPrint().c_str()));
		if(!t.isNull())
		{
			//cloud 3d
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA;
			if(sA_.getDepthRaw().type() == CV_8UC1)
			{
				cloudA = util3d::cloudFromStereoImages(
						sA_.getImageRaw(),
						sA_.getDepthRaw(),
						sA_.getDepthCx(), sA_.getDepthCy(),
						sA_.getDepthFx(), sA_.getDepthFy(),
						decimation);
			}
			else
			{
				cloudA = util3d::cloudFromDepthRGB(
						sA_.getImageRaw(),
						sA_.getDepthRaw(),
						sA_.getDepthCx(), sA_.getDepthCy(),
						sA_.getDepthFx(), sA_.getDepthFy(),
						decimation);
			}

			cloudA = util3d::removeNaNFromPointCloud<pcl::PointXYZRGB>(cloudA);

			if(maxDepth>0.0)
			{
				cloudA = util3d::passThrough<pcl::PointXYZRGB>(cloudA, "z", 0, maxDepth);
			}
			if(samples>0 && (int)cloudA->size() > samples)
			{
				cloudA = util3d::sampling<pcl::PointXYZRGB>(cloudA, samples);
			}
			cloudA = util3d::transformPointCloud<pcl::PointXYZRGB>(cloudA, sA_.getLocalTransform());

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB;
			if(sB_.getDepthRaw().type() == CV_8UC1)
			{
				cloudB = util3d::cloudFromStereoImages(
						sB_.getImageRaw(),
						sB_.getDepthRaw(),
						sB_.getDepthCx(), sB_.getDepthCy(),
						sB_.getDepthFx(), sB_.getDepthFy(),
						decimation);
			}
			else
			{
				cloudB = util3d::cloudFromDepthRGB(
						sB_.getImageRaw(),
						sB_.getDepthRaw(),
						sB_.getDepthCx(), sB_.getDepthCy(),
						sB_.getDepthFx(), sB_.getDepthFy(),
						decimation);
			}

			cloudB = util3d::removeNaNFromPointCloud<pcl::PointXYZRGB>(cloudB);

			if(maxDepth>0.0)
			{
				cloudB = util3d::passThrough<pcl::PointXYZRGB>(cloudB, "z", 0, maxDepth);
			}
			if(samples>0 && (int)cloudB->size() > samples)
			{
				cloudB = util3d::sampling<pcl::PointXYZRGB>(cloudB, samples);
			}
			cloudB = util3d::transformPointCloud<pcl::PointXYZRGB>(cloudB, t*sB_.getLocalTransform());

			//cloud 2d
			pcl::PointCloud<pcl::PointXYZ>::Ptr scanA, scanB;
			scanA = util3d::laserScanToPointCloud(sA_.getLaserScanRaw());
			scanB = util3d::laserScanToPointCloud(sB_.getLaserScanRaw());
			scanB = util3d::transformPointCloud<pcl::PointXYZ>(scanB, t);

			ui_->label_idA->setText(QString("[%1 (%2) -> %3 (%4)]").arg(sB_.id()).arg(cloudB->size()).arg(sA_.id()).arg(cloudA->size()));

			if(cloudA->size())
			{
				ui_->cloudViewerTransform->addOrUpdateCloud("cloud0", cloudA);
			}
			if(cloudB->size())
			{
				ui_->cloudViewerTransform->addOrUpdateCloud("cloud1", cloudB);
			}
			if(scanA->size())
			{
				ui_->cloudViewerTransform->addOrUpdateCloud("scan0", scanA);
			}
			if(scanB->size())
			{
				ui_->cloudViewerTransform->addOrUpdateCloud("scan1", scanB);
			}
		}
		else
		{
			UERROR("loop transform is null !?!?");
			ui_->cloudViewerTransform->removeAllClouds();
		}
		ui_->cloudViewerTransform->render();
	}
}


void LoopClosureViewer::showEvent(QShowEvent * event)
{
	QWidget::showEvent( event );
	QTimer::singleShot(500, this, SLOT(updateView())); // make sure the QVTKWidget is shown!
}

} /* namespace rtabmap */
