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

#include "rtabmap/gui/OdometryViewer.h"

#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/gui/UCv2Qt.h"

#include "rtabmap/gui/ImageView.h"
#include "rtabmap/gui/CloudViewer.h"

#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QApplication>

namespace rtabmap {

OdometryViewer::OdometryViewer(int maxClouds, int decimation, float voxelSize, int qualityWarningThr, QWidget * parent) :
		QDialog(parent),
		imageView_(new ImageView(this)),
		cloudView_(new CloudViewer(this)),
		processingData_(false),
		odomImageShow_(true),
		odomImageDepthShow_(true),
		lastOdomPose_(Transform::getIdentity()),
		qualityWarningThr_(qualityWarningThr),
		id_(0),
		validDecimationValue_(1)
{

	qRegisterMetaType<rtabmap::SensorData>("rtabmap::SensorData");
	qRegisterMetaType<rtabmap::OdometryInfo>("rtabmap::OdometryInfo");

	imageView_->setImageDepthShown(false);
	imageView_->setMinimumSize(320, 240);

	cloudView_->setCameraFree();
	cloudView_->setGridShown(true);

	QLabel * maxCloudsLabel = new QLabel("Max clouds", this);
	QLabel * voxelLabel = new QLabel("Voxel", this);
	QLabel * decimationLabel = new QLabel("Decimation", this);
	maxCloudsSpin_ = new QSpinBox(this);
	maxCloudsSpin_->setMinimum(0);
	maxCloudsSpin_->setMaximum(100);
	maxCloudsSpin_->setValue(maxClouds);
	voxelSpin_ = new QDoubleSpinBox(this);
	voxelSpin_->setMinimum(0);
	voxelSpin_->setMaximum(1);
	voxelSpin_->setDecimals(3);
	voxelSpin_->setSingleStep(0.01);
	voxelSpin_->setSuffix(" m");
	voxelSpin_->setValue(voxelSize);
	decimationSpin_ = new QSpinBox(this);
	decimationSpin_->setMinimum(1);
	decimationSpin_->setMaximum(16);
	decimationSpin_->setValue(decimation);
	timeLabel_ = new QLabel(this);
	QPushButton * clearButton = new QPushButton("clear", this);
	QPushButton * closeButton = new QPushButton("close", this);
	connect(clearButton, SIGNAL(clicked()), this, SLOT(clear()));
	connect(closeButton, SIGNAL(clicked()), this, SLOT(reject()));

	//layout
	QHBoxLayout * layout = new QHBoxLayout();
	layout->setMargin(0);
	layout->setSpacing(0);
	layout->addWidget(imageView_,1);
	layout->addWidget(cloudView_,1);

	QHBoxLayout * hlayout2 = new QHBoxLayout();
	hlayout2->setMargin(0);
	hlayout2->addWidget(maxCloudsLabel);
	hlayout2->addWidget(maxCloudsSpin_);
	hlayout2->addWidget(voxelLabel);
	hlayout2->addWidget(voxelSpin_);
	hlayout2->addWidget(decimationLabel);
	hlayout2->addWidget(decimationSpin_);
	hlayout2->addWidget(timeLabel_);
	hlayout2->addStretch(1);
	hlayout2->addWidget(clearButton);
	hlayout2->addWidget(closeButton);

	QVBoxLayout * vlayout = new QVBoxLayout(this);
	vlayout->setMargin(0);
	vlayout->setSpacing(0);
	vlayout->addLayout(layout, 1);
	vlayout->addLayout(hlayout2);

	this->setLayout(vlayout);
}

OdometryViewer::~OdometryViewer()
{
	this->unregisterFromEventsManager();
	this->clear();
	UDEBUG("");
}

void OdometryViewer::clear()
{
	addedClouds_.clear();
	cloudView_->clear();
}

void OdometryViewer::processData(const rtabmap::SensorData & data, const rtabmap::OdometryInfo & info)
{
	processingData_ = true;
	int quality = info.inliers;

	bool lost = false;
	bool lostStateChanged = false;

	if(data.pose().isNull())
	{
		UDEBUG("odom lost"); // use last pose
		lostStateChanged = imageView_->getBackgroundColor() != Qt::darkRed;
		imageView_->setBackgroundColor(Qt::darkRed);
		cloudView_->setBackgroundColor(Qt::darkRed);

		lost = true;
	}
	else if(info.inliers>0 &&
			qualityWarningThr_ &&
			info.inliers < qualityWarningThr_)
	{
		UDEBUG("odom warn, quality(inliers)=%d thr=%d", info.inliers, qualityWarningThr_);
		lostStateChanged = imageView_->getBackgroundColor() == Qt::darkRed;
		imageView_->setBackgroundColor(Qt::darkYellow);
		cloudView_->setBackgroundColor(Qt::darkYellow);
	}
	else
	{
		UDEBUG("odom ok");
		lostStateChanged = imageView_->getBackgroundColor() == Qt::darkRed;
		imageView_->setBackgroundColor(cloudView_->getDefaultBackgroundColor());
		cloudView_->setBackgroundColor(Qt::black);
	}

	timeLabel_->setText(QString("%1 s").arg(info.time));

	if(!data.image().empty() && !data.depthOrRightImage().empty() && data.fx()>0.0f && data.fyOrBaseline()>0.0f)
	{
		UDEBUG("New pose = %s, quality=%d", data.pose().prettyPrint().c_str(), quality);

		if(data.image().cols % decimationSpin_->value() == 0 &&
		   data.image().rows % decimationSpin_->value() == 0)
		{
			validDecimationValue_ = decimationSpin_->value();
		}
		else
		{
			UWARN("Decimation (%d) must be a denominator of the width and height of "
					"the image (%d/%d). Using last valid decimation value (%d).",
					decimationSpin_->value(),
					data.image().cols,
					data.image().rows,
					validDecimationValue_);
		}


		// visualization: buffering the clouds
		// Create the new cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		if(!data.depth().empty())
		{
			cloud = util3d::cloudFromDepthRGB(
					data.image(),
					data.depth(),
					data.cx(), data.cy(),
					data.fx(), data.fy(),
					validDecimationValue_);
		}
		else if(!data.rightImage().empty())
		{
			cloud = util3d::cloudFromStereoImages(
					data.image(),
					data.rightImage(),
					data.cx(), data.cy(),
					data.fx(), data.baseline(),
					validDecimationValue_);
		}

		if(voxelSpin_->value() > 0.0f && cloud->size())
		{
			cloud = util3d::voxelize(cloud, voxelSpin_->value());
		}

		if(cloud->size())
		{
			cloud = util3d::transformPointCloud(cloud, data.localTransform());

			if(!data.pose().isNull())
			{
				if(cloudView_->getAddedClouds().contains("cloudtmp"))
				{
					cloudView_->removeCloud("cloudtmp");
				}

				while(maxCloudsSpin_->value()>0 && (int)addedClouds_.size() > maxCloudsSpin_->value())
				{
					UASSERT(cloudView_->removeCloud(addedClouds_.first()));
					addedClouds_.pop_front();
				}

				data.id()?id_=data.id():++id_;
				std::string cloudName = uFormat("cloud%d", id_);
				addedClouds_.push_back(cloudName);
				UASSERT(cloudView_->addCloud(cloudName, cloud, data.pose()));
			}
			else
			{
				cloudView_->addOrUpdateCloud("cloudtmp", cloud, lastOdomPose_);
			}
		}
	}

	if(!data.pose().isNull())
	{
		lastOdomPose_ = data.pose();
		cloudView_->updateCameraTargetPosition(data.pose());
	}

	if(info.localMap.size())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(info.localMap.size());
		int i=0;
		for(std::multimap<int, cv::Point3f>::const_iterator iter=info.localMap.begin(); iter!=info.localMap.end(); ++iter)
		{
			(*cloud)[i].x = iter->second.x;
			(*cloud)[i].y = iter->second.y;
			(*cloud)[i++].z = iter->second.z;
		}
		cloudView_->addOrUpdateCloud("localmap", cloud);
	}

	if(!data.image().empty())
	{
		if(info.type == 0)
		{
			imageView_->setFeatures(info.words, data.depth(), Qt::yellow);
		}
		else if(info.type == 1)
		{
			std::vector<cv::KeyPoint> kpts;
			cv::KeyPoint::convert(info.refCorners, kpts);
			imageView_->setFeatures(kpts, data.depth(), Qt::red);
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
		}
		if(info.type == 1 && info.cornerInliers.size())
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
								info.refCorners[info.cornerInliers[i]].x,
								info.refCorners[info.cornerInliers[i]].y,
								info.newCorners[info.cornerInliers[i]].x,
								info.newCorners[info.cornerInliers[i]].y,
								Qt::blue);
					}
				}
			}
		}

		if(!data.image().empty())
		{
			imageView_->setSceneRect(QRectF(0,0,(float)data.image().cols, (float)data.image().rows));
		}
	}

	imageView_->update();
	cloudView_->update();
	QApplication::processEvents();
	processingData_ = false;
}

void OdometryViewer::handleEvent(UEvent * event)
{
	if(!processingData_ && this->isVisible())
	{
		if(event->getClassName().compare("OdometryEvent") == 0)
		{
			rtabmap::OdometryEvent * odomEvent = (rtabmap::OdometryEvent*)event;
			if(odomEvent->data().isValid())
			{
				processingData_ = true;
				QMetaObject::invokeMethod(this, "processData",
						Q_ARG(rtabmap::SensorData, odomEvent->data()),
						Q_ARG(rtabmap::OdometryInfo, odomEvent->info()));
			}
		}
	}
}

} /* namespace rtabmap */
