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

#include "rtabmap/core/util3d.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UConversion.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <QtGui/QInputDialog>
#include <QtGui/QAction>
#include <QtGui/QMenu>
#include <QtGui/QKeyEvent>

namespace rtabmap {


OdometryViewer::OdometryViewer(int maxClouds, int decimation, float voxelSize, int qualityWarningThr, QWidget * parent) :
		CloudViewer(parent),
		dataQuality_(-1),
		lastOdomPose_(Transform::getIdentity()),
		maxClouds_(maxClouds),
		voxelSize_(voxelSize),
		decimation_(decimation),
		qualityWarningThr_(qualityWarningThr),
		id_(0),
		_aSetVoxelSize(0),
		_aSetDecimation(0),
		_aSetCloudHistorySize(0),
		_aPause(0)
{

	//add actions to CloudViewer menu
	_aSetVoxelSize = new QAction("Set voxel size...", this);
	_aSetDecimation = new QAction("Set depth image decimation...", this);
	_aSetCloudHistorySize = new QAction("Set cloud history size...", this);
	_aPause = new QAction("Pause", this);
	_aPause->setCheckable(true);
	menu()->addAction(_aSetVoxelSize);
	menu()->addAction(_aSetDecimation);
	menu()->addAction(_aSetCloudHistorySize);
	menu()->addAction(_aPause);
}

void OdometryViewer::clear()
{
	dataMutex_.lock();
	data_.clear();
	dataMutex_.unlock();
	clouds_.clear();
	CloudViewer::clear();
}

void OdometryViewer::processData()
{
	rtabmap::SensorData data;
	int quality = -1;
	dataMutex_.lock();
	if(data_.size())
	{
		data = data_.back();
		data_.clear();
		quality = dataQuality_;
		dataQuality_ = -1;
	}
	dataMutex_.unlock();

	if(!data.image().empty() && !data.depth().empty() && data.fx()>0.0f && data.fy()>0.0f && this->isVisible())
	{
		UDEBUG("New pose = %s, quality=%d", data.pose().prettyPrint().c_str(), quality);

		// visualization: buffering the clouds
		// Create the new cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		if(data.depth().type() == CV_8UC1)
		{
			cloud = util3d::cloudFromStereoImages(
					data.image(),
					data.depth(),
					data.cx(), data.cy(),
					data.fx(), data.fy(),
					decimation_);
		}
		else
		{
			cloud = util3d::cloudFromDepthRGB(
					data.image(),
					data.depth(),
					data.cx(), data.cy(),
					data.fx(), data.fy(),
					decimation_);
		}

		if(voxelSize_ > 0.0f)
		{
			cloud = util3d::voxelize<pcl::PointXYZRGB>(cloud, voxelSize_);
		}

		cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, data.localTransform());

		if(!data.pose().isNull())
		{
			lastOdomPose_ = data.pose();
			if(this->getAddedClouds().contains("cloudtmp"))
			{
				this->removeCloud("cloudtmp");
			}

			data.id()?id_=data.id():++id_;

			clouds_.insert(std::make_pair(id_, cloud));

			while(maxClouds_>0 && (int)clouds_.size() > maxClouds_)
			{
				this->removeCloud(uFormat("cloud%d", clouds_.begin()->first));
				clouds_.erase(clouds_.begin());
			}

			if(clouds_.size())
			{
				this->addCloud(uFormat("cloud%d", clouds_.rbegin()->first), clouds_.rbegin()->second, data.pose());
			}

			this->updateCameraPosition(data.pose());

			if(qualityWarningThr_ && quality>=0 && quality < qualityWarningThr_)
			{
				this->setBackgroundColor(Qt::darkYellow);
			}
			else
			{
				this->setBackgroundColor(Qt::black);
			}
		}
		else
		{
			this->addOrUpdateCloud("cloudtmp", cloud, lastOdomPose_);
			this->setBackgroundColor(Qt::darkRed);
		}

		this->render();
	}
}

void OdometryViewer::handleEvent(UEvent * event)
{
	if(!_aPause->isChecked())
	{
		if(event->getClassName().compare("OdometryEvent") == 0)
		{
			rtabmap::OdometryEvent * odomEvent = (rtabmap::OdometryEvent*)event;

			bool empty = false;
			dataMutex_.lock();
			if(data_.empty())
			{
				data_.push_back(odomEvent->data());
				empty= true;
			}
			else
			{
				data_.back() = odomEvent->data();
			}
			dataQuality_ = odomEvent->info().inliers;
			dataMutex_.unlock();
			if(empty)
			{
				QMetaObject::invokeMethod(this, "processData");
			}
		}
	}
}

void OdometryViewer::handleAction(QAction * a)
{
	CloudViewer::handleAction(a);
	if(a == _aSetVoxelSize)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Set voxel size"), tr("Size (0=disabled)"), voxelSize_, 0.0, 0.1, 2, &ok);
		if(ok)
		{
			voxelSize_ = value;
		}
	}
	else if(a == _aSetCloudHistorySize)
	{
		bool ok;
		int value = QInputDialog::getInt(this, tr("Set cloud history size"), tr("Size (0=infinite)"), maxClouds_, 0, 100, 1, &ok);
		if(ok)
		{
			maxClouds_ = value;
		}
	}
	else if(a == _aSetDecimation)
	{
		bool ok;
		int value = QInputDialog::getInt(this, tr("Set depth image decimation"), tr("Decimation"), decimation_, 1, 8, 1, &ok);
		if(ok)
		{
			decimation_ = value;
		}
	}
}

} /* namespace rtabmap */
