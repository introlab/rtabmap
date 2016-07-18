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

#include "rtabmap/gui/LoopClosureViewer.h"
#include "ui_loopClosureViewer.h"

#include "rtabmap/core/Memory.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"
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
	minDepth_(0)
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

void LoopClosureViewer::updateView(const Transform & transform, const ParametersMap & parameters)
{
	if(sA_.id()>0 && sB_.id()>0)
	{
		int decimation = 1;
		float maxDepth = 0;
		float minDepth = 0;

		if(!ui_->checkBox_rawCloud->isChecked())
		{			decimation = decimation_;
			maxDepth = maxDepth_;
			minDepth = minDepth_;
		}

		UDEBUG("decimation = %d", decimation);
		UDEBUG("maxDepth = %f", maxDepth);
		UDEBUG("minDepth = %d", minDepth);

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
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA, cloudB;
			cloudA = util3d::cloudRGBFromSensorData(sA_.sensorData(), decimation, maxDepth, minDepth, 0, parameters);
			cloudB = util3d::cloudRGBFromSensorData(sB_.sensorData(), decimation, maxDepth, minDepth, 0, parameters);

			//cloud 2d
			pcl::PointCloud<pcl::PointXYZ>::Ptr scanA, scanB;
			scanA = util3d::laserScanToPointCloud(sA_.sensorData().laserScanRaw());
			scanB = util3d::laserScanToPointCloud(sB_.sensorData().laserScanRaw());
			scanB = util3d::transformPointCloud(scanB, t);

			ui_->label_idA->setText(QString("[%1 (%2) -> %3 (%4)]").arg(sB_.id()).arg(cloudB->size()).arg(sA_.id()).arg(cloudA->size()));

			if(cloudA->size())
			{
				ui_->cloudViewerTransform->addCloud("cloud0", cloudA);
			}
			if(cloudB->size())
			{
				cloudB = util3d::transformPointCloud(cloudB, t);
				ui_->cloudViewerTransform->addCloud("cloud1", cloudB);
			}
			if(scanA->size())
			{
				ui_->cloudViewerTransform->addCloud("scan0", scanA);
			}
			if(scanB->size())
			{
				ui_->cloudViewerTransform->addCloud("scan1", scanB);
			}
		}
		else
		{
			UERROR("loop transform is null !?!?");
			ui_->cloudViewerTransform->removeAllClouds();
		}
		ui_->cloudViewerTransform->update();
	}
}


void LoopClosureViewer::showEvent(QShowEvent * event)
{
	QWidget::showEvent( event );
	QTimer::singleShot(500, this, SLOT(updateView())); // make sure the QVTKWidget is shown!
}

} /* namespace rtabmap */
