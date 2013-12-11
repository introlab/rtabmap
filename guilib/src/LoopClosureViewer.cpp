/*
 * LoopClosureViewer.cpp
 *
 *  Created on: 2013-10-21
 *      Author: Mathieu
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
	sA_(0),
	sB_(0),
	decimation_(1),
	maxDepth_(0),
	samples_(0)
{
	ui_ = new Ui_loopClosureViewer();
	ui_->setupUi(this);

	connect(ui_->checkBox_rawCloud, SIGNAL(clicked()), this, SLOT(updateView()));
}

LoopClosureViewer::~LoopClosureViewer() {
	delete ui_;
	if(sA_)
	{
		delete sA_;
	}
	if(sB_)
	{
		delete sB_;
	}
}

void LoopClosureViewer::setData(Signature * sA, Signature * sB)
{
	if(sA_)
	{
		delete sA_;
	}
	if(sB_)
	{
		delete sB_;
	}
	sA_ = sA;
	sB_ = sB;
	if(sA_ && sB_)
	{
		ui_->label_idA->setText(QString("[%1-%2]").arg(sA->id()).arg(sB->id()));
	}
}

void LoopClosureViewer::updateView(const Transform & transform)
{
	if(sA_ && sB_)
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
		UDEBUG("maxDepth = %d", maxDepth);
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
			t = sB_->getPose();
		}

		UDEBUG("t= %s", t.prettyPrint().c_str());
		ui_->label_transform->setText(QString("(%1)").arg(t.prettyPrint().c_str()));
		if(!t.isNull())
		{
			util3d::CompressionThread ctiA(sA_->getImage(), true);
			util3d::CompressionThread ctdA(sA_->getDepth(), true);
			util3d::CompressionThread ctiB(sB_->getImage(), true);
			util3d::CompressionThread ctdB(sB_->getDepth(), true);
			util3d::CompressionThread ct2dA(sA_->getDepth2D(), false);
			util3d::CompressionThread ct2dB(sB_->getDepth2D(), false);
			ctiA.start();
			ctdA.start();
			ctiB.start();
			ctdB.start();
			ct2dA.start();
			ct2dB.start();
			ctiA.join();
			ctdA.join();
			ctiB.join();
			ctdB.join();
			ct2dA.join();
			ct2dB.join();
			cv::Mat imageA = ctiA.getUncompressedData();
			cv::Mat depthA = ctdA.getUncompressedData();
			cv::Mat imageB = ctiB.getUncompressedData();
			cv::Mat depthB = ctdB.getUncompressedData();
			cv::Mat depth2dA = ct2dA.getUncompressedData();
			cv::Mat depth2dB = ct2dB.getUncompressedData();

			//cloud 3d
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA;
			cloudA = util3d::cloudFromDepthRGB(
					imageA,
					depthA,
					depthA.cols/2,
					depthA.rows/2,
					1.0f/sA_->getDepthConstant(),
					1.0f/sA_->getDepthConstant(),
					decimation);

			cloudA = util3d::removeNaNFromPointCloud(cloudA);

			if(maxDepth>0.0)
			{
				cloudA = util3d::passThrough(cloudA, "z", 0, maxDepth);
			}
			if(samples>0 && (int)cloudA->size() > samples)
			{
				cloudA = util3d::sampling(cloudA, samples);
			}
			cloudA = util3d::transformPointCloud(cloudA, sA_->getLocalTransform());

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB;
			cloudB = util3d::cloudFromDepthRGB(
					imageB,
					depthB,
					depthB.cols/2,
					depthB.rows/2,
					1.0f/sB_->getDepthConstant(),
					1.0f/sB_->getDepthConstant(),
					decimation);

			cloudB = util3d::removeNaNFromPointCloud(cloudB);

			if(maxDepth>0.0)
			{
				cloudB = util3d::passThrough(cloudB, "z", 0, maxDepth);
			}
			if(samples>0 && (int)cloudB->size() > samples)
			{
				cloudB = util3d::sampling(cloudB, samples);
			}
			cloudB = util3d::transformPointCloud(cloudB, t*sB_->getLocalTransform());

			//cloud 2d
			pcl::PointCloud<pcl::PointXYZ>::Ptr scanA, scanB;
			scanA = util3d::depth2DToPointCloud(depth2dA);
			scanB = util3d::depth2DToPointCloud(depth2dB);
			scanB = util3d::transformPointCloud(scanB, t);

			ui_->label_idA->setText(QString("[%1 (%2) -> %3 (%4)]").arg(sB_->id()).arg(cloudB->size()).arg(sA_->id()).arg(cloudA->size()));

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
