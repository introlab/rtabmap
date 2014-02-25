/*
 * OdometryViewer.cpp
 *
 *  Created on: 2013-10-15
 *      Author: Mathieu
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


OdometryViewer::OdometryViewer(int maxClouds, int decimation, float voxelSize, QWidget * parent) :
		CloudViewer(parent),
		maxClouds_(maxClouds),
		voxelSize_(voxelSize),
		decimation_(decimation),
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

void OdometryViewer::processData()
{
	rtabmap::Image data;
	dataMutex_.lock();
	if(buffer_.size())
	{
		data = buffer_.back();
		buffer_.clear();
	}
	dataMutex_.unlock();

	if(!data.empty() && this->isVisible())
	{
		UINFO("New pose = %s", data.pose().prettyPrint().c_str());

		// visualization: buffering the clouds
		// Create the new cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		cloud = util3d::cloudFromDepthRGB(
				data.image(),
				data.depth(),
			   float(data.depth().cols/2),
			   float(data.depth().rows/2),
			   1.0f/data.depthConstant(),
			   1.0f/data.depthConstant(),
			   decimation_);

		if(voxelSize_ > 0.0f)
		{
			cloud = util3d::voxelize(cloud, voxelSize_);
		}

		cloud = util3d::transformPointCloud(cloud, data.localTransform());

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

		this->setBackgroundColor(Qt::black);

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

			if(odomEvent->isValid())
			{
				bool empty = false;
				dataMutex_.lock();
				if(buffer_.empty())
				{
					buffer_.push_back(odomEvent->data());
					empty= true;
				}
				else
				{
					buffer_.back() = odomEvent->data();
				}
				dataMutex_.unlock();
				if(empty)
				{
					QMetaObject::invokeMethod(this, "processData");
				}
			}
			else
			{
				//UWARN("odom=%fs, Cannot compute odometry!!!", timer_.restart());
				QMetaObject::invokeMethod(this, "setBackgroundColor", Q_ARG(QColor, Qt::darkRed));
				QMetaObject::invokeMethod(this, "render");
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
