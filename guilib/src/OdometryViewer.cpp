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

#include "rtabmap/gui/OdometryViewer.h"

#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UCv2Qt.h"

#include "rtabmap/gui/ImageView.h"
#include "rtabmap/gui/CloudViewer.h"

#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QApplication>

namespace rtabmap {

OdometryViewer::OdometryViewer(
	int maxClouds, 
	int decimation, 
	float voxelSize, 
	float maxDepth, 
	int qualityWarningThr, 
	QWidget * parent,
	const ParametersMap & parameters) :
		QDialog(parent),
		imageView_(new ImageView(this)),
		cloudView_(new CloudViewer(this)),
		processingData_(false),
		odomImageShow_(true),
		odomImageDepthShow_(true),
		lastOdomPose_(Transform::getIdentity()),
		qualityWarningThr_(qualityWarningThr),
		id_(0),
		validDecimationValue_(1),
		parameters_(parameters)
{

	qRegisterMetaType<rtabmap::OdometryEvent>("rtabmap::OdometryEvent");

	imageView_->setImageDepthShown(false);
	imageView_->setMinimumSize(320, 240);
	imageView_->setAlpha(255);

	cloudView_->setCameraTargetLocked();
	cloudView_->setGridShown(true);
	cloudView_->setFrustumShown(true);

	QLabel * maxCloudsLabel = new QLabel("Max clouds", this);
	QLabel * voxelLabel = new QLabel("Voxel", this);
	QLabel * maxDepthLabel = new QLabel("Max depth", this);
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
	maxDepthSpin_ = new QDoubleSpinBox(this);
	maxDepthSpin_->setMinimum(0);
	maxDepthSpin_->setMaximum(100);
	maxDepthSpin_->setDecimals(0);
	maxDepthSpin_->setSingleStep(1);
	maxDepthSpin_->setSuffix(" m");
	maxDepthSpin_->setValue(maxDepth);
	decimationSpin_ = new QSpinBox(this);
	decimationSpin_->setMinimum(1);
	decimationSpin_->setMaximum(16);
	decimationSpin_->setValue(decimation);
	cloudShown_ = new QCheckBox(this);
	cloudShown_->setText("Cloud");
	cloudShown_->setChecked(true);
	scanShown_ = new QCheckBox(this);
	scanShown_->setText("Scan");
	scanShown_->setChecked(true);
	featuresShown_ = new QCheckBox(this);
	featuresShown_->setText("Features");
	featuresShown_->setChecked(true);
	timeLabel_ = new QLabel(this);
	QPushButton * resetButton = new QPushButton("reset", this);
	QPushButton * clearButton = new QPushButton("clear", this);
	QPushButton * closeButton = new QPushButton("close", this);
	connect(resetButton, SIGNAL(clicked()), this, SLOT(reset()));
	connect(clearButton, SIGNAL(clicked()), this, SLOT(clear()));
	connect(closeButton, SIGNAL(clicked()), this, SLOT(reject()));

	//layout
	QHBoxLayout * layout = new QHBoxLayout();
	layout->setContentsMargins(0,0,0,0);
	layout->setSpacing(0);
	layout->addWidget(imageView_,1);
	layout->addWidget(cloudView_,1);

	QHBoxLayout * hlayout2 = new QHBoxLayout();
	hlayout2->setContentsMargins(0,0,0,0);
	hlayout2->addWidget(maxCloudsLabel);
	hlayout2->addWidget(maxCloudsSpin_);
	hlayout2->addWidget(voxelLabel);
	hlayout2->addWidget(voxelSpin_);
	hlayout2->addWidget(maxDepthLabel);
	hlayout2->addWidget(maxDepthSpin_);
	hlayout2->addWidget(decimationLabel);
	hlayout2->addWidget(decimationSpin_);
	hlayout2->addWidget(cloudShown_);
	hlayout2->addWidget(scanShown_);
	hlayout2->addWidget(featuresShown_);
	hlayout2->addWidget(timeLabel_);
	hlayout2->addStretch(1);
	hlayout2->addWidget(resetButton);
	hlayout2->addWidget(clearButton);
	hlayout2->addWidget(closeButton);

	QVBoxLayout * vlayout = new QVBoxLayout(this);
	vlayout->setContentsMargins(0,0,0,0);
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

void OdometryViewer::reset()
{
	this->post(new OdometryResetEvent());
}

void OdometryViewer::clear()
{
	addedClouds_.clear();
	cloudView_->clear();
}

void OdometryViewer::processData(const rtabmap::OdometryEvent & odom)
{
	processingData_ = true;
	int quality = odom.info().reg.inliers;

	bool lost = false;
	bool lostStateChanged = false;

	if(odom.pose().isNull())
	{
		UDEBUG("odom lost"); // use last pose
		lostStateChanged = imageView_->getBackgroundColor() != Qt::darkRed;
		imageView_->setBackgroundColor(Qt::darkRed);
		cloudView_->setBackgroundColor(Qt::darkRed);

		lost = true;
	}
	else if(odom.info().reg.inliers>0 &&
			qualityWarningThr_ &&
			odom.info().reg.inliers < qualityWarningThr_)
	{
		UDEBUG("odom warn, quality(inliers)=%d thr=%d", odom.info().reg.inliers, qualityWarningThr_);
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

	timeLabel_->setText(QString("%1 s").arg(odom.info().timeEstimation));

	if(cloudShown_->isChecked() &&
		!odom.data().imageRaw().empty() &&
		!odom.data().depthOrRightRaw().empty() &&
		(odom.data().stereoCameraModels().size() || odom.data().cameraModels().size()))
	{
		UDEBUG("New pose = %s, quality=%d", odom.pose().prettyPrint().c_str(), quality);

		if(!odom.data().depthRaw().empty())
		{
			if(odom.data().imageRaw().cols % decimationSpin_->value() == 0 &&
			   odom.data().imageRaw().rows % decimationSpin_->value() == 0)
			{
				validDecimationValue_ = decimationSpin_->value();
			}
			else
			{
				UWARN("Decimation (%d) must be a denominator of the width and height of "
						"the image (%d/%d). Using last valid decimation value (%d).",
						decimationSpin_->value(),
						odom.data().imageRaw().cols,
						odom.data().imageRaw().rows,
						validDecimationValue_);
			}
		}
		else
		{
			validDecimationValue_ = decimationSpin_->value();
		}

		// visualization: buffering the clouds
		// Create the new cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		pcl::IndicesPtr validIndices(new std::vector<int>);
		cloud = util3d::cloudRGBFromSensorData(
				odom.data(),
				validDecimationValue_,
				0,
				0,
				validIndices.get(),
				parameters_);

		if(voxelSpin_->value())
		{
			cloud = util3d::voxelize(cloud, validIndices, voxelSpin_->value());
		}

		if(cloud->size())
		{
			if(!odom.pose().isNull())
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

				odom.data().id()?id_=odom.data().id():++id_;
				std::string cloudName = uFormat("cloud%d", id_);
				addedClouds_.push_back(cloudName);
				UASSERT(cloudView_->addCloud(cloudName, cloud, odom.pose()));
			}
			else
			{
				cloudView_->addCloud("cloudtmp", cloud, lastOdomPose_);
			}
		}
	}
	else if(!cloudShown_->isChecked())
	{
		while(!addedClouds_.empty())
		{
			UASSERT(cloudView_->removeCloud(addedClouds_.first()));
			addedClouds_.pop_front();
		}
	}

	if(!odom.pose().isNull())
	{
		lastOdomPose_ = odom.pose();
		cloudView_->updateCameraTargetPosition(odom.pose());

		if(odom.data().cameraModels().size() && !odom.data().cameraModels()[0].localTransform().isNull())
		{
			cloudView_->updateCameraFrustums(odom.pose(), odom.data().cameraModels());
		}
		else if(odom.data().stereoCameraModels().size() && !odom.data().stereoCameraModels()[0].localTransform().isNull())
		{
			cloudView_->updateCameraFrustums(odom.pose(), odom.data().stereoCameraModels());
		}
	}

	if(scanShown_->isChecked())
	{
		// scan local map
		if(!odom.info().localScanMap.isEmpty())
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
			cloud = util3d::laserScanToPointCloudNormal(odom.info().localScanMap, odom.info().localScanMap.localTransform());
			if(!cloudView_->addCloud("scanMapOdom", cloud, Transform::getIdentity(), Qt::blue))
			{
				UERROR("Adding scanMapOdom to viewer failed!");
			}
			else
			{
				cloudView_->setCloudVisibility("scanMapOdom", true);
				cloudView_->setCloudOpacity("scanMapOdom", 0.5);
			}
		}
		// scan cloud
		if(!odom.data().laserScanRaw().isEmpty())
		{
			LaserScan scan = odom.data().laserScanRaw();

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
			cloud = util3d::laserScanToPointCloudNormal(scan, odom.pose() * scan.localTransform());

			if(!cloudView_->addCloud("scanOdom", cloud, Transform::getIdentity(), Qt::magenta))
			{
				UERROR("Adding scanOdom to viewer failed!");
			}
			else
			{
				cloudView_->setCloudVisibility("scanOdom", true);
				cloudView_->setCloudOpacity("scanOdom", 0.5);
			}
		}
	}
	else
	{
		cloudView_->removeCloud("scanMapOdom");
		cloudView_->removeCloud("scanOdom");
	}

	// 3d features
	if(featuresShown_->isChecked())
	{
		if(!odom.info().localMap.empty() && !odom.pose().isNull())
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			cloud->resize(odom.info().localMap.size());
			int i=0;
			for(std::map<int, cv::Point3f>::const_iterator iter=odom.info().localMap.begin(); iter!=odom.info().localMap.end(); ++iter)
			{
				// filter very far features from current location
				if(uNormSquared(iter->second.x-odom.pose().x(), iter->second.y-odom.pose().y(), iter->second.z-odom.pose().z()) < 50*50)
				{
					(*cloud)[i].x = iter->second.x;
					(*cloud)[i].y = iter->second.y;
					(*cloud)[i].z = iter->second.z;

					// green = inlier, yellow = outliers
					bool inlier = odom.info().words.find(iter->first) != odom.info().words.end();
					(*cloud)[i].r = inlier?0:255;
					(*cloud)[i].g = 255;
					(*cloud)[i++].b = 0;
				}
			}
			cloud->resize(i);

			if(!cloudView_->addCloud("featuresOdom", cloud))
			{
				UERROR("Adding featuresOdom to viewer failed!");
			}
			else
			{
				cloudView_->setCloudVisibility("featuresOdom", true);
				cloudView_->setCloudPointSize("featuresOdom", 3);
			}
		}
	}
	else
	{
		cloudView_->removeCloud("featuresOdom");
	}

	if(!odom.data().imageRaw().empty())
	{
		if(odom.info().type == (int)Odometry::kTypeF2M || odom.info().type == (int)Odometry::kTypeORBSLAM)
		{
			imageView_->setFeatures(odom.info().words, odom.data().depthRaw(), Qt::yellow);
		}
		else if(odom.info().type == (int)Odometry::kTypeF2F ||
				odom.info().type == (int)Odometry::kTypeViso2 ||
				odom.info().type == (int)Odometry::kTypeFovis ||
				odom.info().type == (int)Odometry::kTypeMSCKF)
		{
			std::vector<cv::KeyPoint> kpts;
			cv::KeyPoint::convert(odom.info().newCorners, kpts, 7);
			imageView_->setFeatures(kpts, odom.data().depthRaw(), Qt::red);
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
			imageView_->setImageDepth(odom.data().imageRaw());
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

			imageView_->setImage(uCvMat2QImage(odom.data().imageRaw()));
			if(imageView_->isImageDepthShown())
			{
				imageView_->setImageDepth(odom.data().depthOrRightRaw());
			}

			if( odom.info().type == Odometry::kTypeF2M ||
				odom.info().type == (int)Odometry::kTypeORBSLAM ||
				odom.info().type == (int)Odometry::kTypeMSCKF)
			{
				if(imageView_->isFeaturesShown())
				{
					for(unsigned int i=0; i<odom.info().reg.matchesIDs.size(); ++i)
					{
						imageView_->setFeatureColor(odom.info().reg.matchesIDs[i], Qt::red); // outliers
					}
					for(unsigned int i=0; i<odom.info().reg.inliersIDs.size(); ++i)
					{
						imageView_->setFeatureColor(odom.info().reg.inliersIDs[i], Qt::green); // inliers
					}
				}
			}
		}
		if((odom.info().type == (int)Odometry::kTypeF2F ||
			odom.info().type == (int)Odometry::kTypeViso2 ||
			odom.info().type == (int)Odometry::kTypeFovis) && odom.info().cornerInliers.size())
		{
			if(imageView_->isFeaturesShown() || imageView_->isLinesShown())
			{
				//draw lines
				UASSERT(odom.info().refCorners.size() == odom.info().newCorners.size());
				for(unsigned int i=0; i<odom.info().cornerInliers.size(); ++i)
				{
					if(imageView_->isFeaturesShown())
					{
						imageView_->setFeatureColor(odom.info().cornerInliers[i], Qt::green); // inliers
					}
					if(imageView_->isLinesShown())
					{
						imageView_->addLine(
								odom.info().newCorners[odom.info().cornerInliers[i]].x,
								odom.info().newCorners[odom.info().cornerInliers[i]].y,
								odom.info().refCorners[odom.info().cornerInliers[i]].x,
								odom.info().refCorners[odom.info().cornerInliers[i]].y,
								Qt::blue);
					}
				}
			}
		}

		if(!odom.data().imageRaw().empty())
		{
			imageView_->setSceneRect(QRectF(0,0,(float)odom.data().imageRaw().cols, (float)odom.data().imageRaw().rows));
		}
	}

	imageView_->update();
	cloudView_->update();
	QApplication::processEvents();
	processingData_ = false;
}

bool OdometryViewer::handleEvent(UEvent * event)
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
						Q_ARG(rtabmap::OdometryEvent, *odomEvent));
			}
		}
	}
	return false;
}

} /* namespace rtabmap */
