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

#ifndef MAPBUILDER_H_
#define MAPBUILDER_H_

#include <QtGui/QVBoxLayout>
#include <QtCore/QMetaType>
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/OdometryEvent.h"

using namespace rtabmap;

// This class receives RtabmapEvent and construct/update a 3D Map
class MapBuilder : public QWidget, public UEventsHandler
{
	Q_OBJECT
public:
	MapBuilder() :
		_processingStatistics(false),
		_lastOdometryProcessed(true)
	{
		this->setWindowFlags(Qt::Dialog);
		this->setWindowTitle(tr("3D Map"));
		this->setMinimumWidth(800);
		this->setMinimumHeight(600);

		cloudViewer_ = new CloudViewer(this);

		QVBoxLayout *layout = new QVBoxLayout();
		layout->addWidget(cloudViewer_);
		this->setLayout(layout);

		qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");
		qRegisterMetaType<rtabmap::SensorData>("rtabmap::SensorData");
	}

	virtual ~MapBuilder()
	{
		this->unregisterFromEventsManager();
	}

private slots:
	void processOdometry(const rtabmap::SensorData & data)
	{
		if(!this->isVisible())
		{
			return;
		}

		Transform pose = data.pose();
		if(pose.isNull())
		{
			//Odometry lost
			cloudViewer_->setBackgroundColor(Qt::darkRed);

			pose = lastOdomPose_;
		}
		else
		{
			cloudViewer_->setBackgroundColor(Qt::black);
		}
		if(!pose.isNull())
		{
			lastOdomPose_ = pose;

			// 3d cloud
			if(data.depth().cols == data.image().cols &&
			   data.depth().rows == data.image().rows &&
			   !data.depth().empty() &&
			   data.fx() > 0.0f &&
			   data.fy() > 0.0f)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
					data.image(),
					data.depth(),
					data.cx(),
					data.cy(),
					data.fx(),
					data.fy(),
					2); // decimation // high definition
				if(cloud->size())
				{
					cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, 4.0f);
					if(cloud->size())
					{
						cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, data.localTransform());
					}
				}
				if(!cloudViewer_->addOrUpdateCloud("cloudOdom", cloud, pose))
				{
					UERROR("Adding cloudOdom to viewer failed!");
				}
			}

			if(!data.pose().isNull())
			{
				// update camera position
				cloudViewer_->updateCameraPosition(data.pose());
			}
		}
		cloudViewer_->render();

		_lastOdometryProcessed = true;
	}


	void processStatistics(const rtabmap::Statistics & stats)
	{
		_processingStatistics = true;

		const std::map<int, Transform> & poses = stats.poses();
		QMap<std::string, Transform> clouds = cloudViewer_->getAddedClouds();
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if(!iter->second.isNull())
			{
				std::string cloudName = uFormat("cloud%d", iter->first);

				// 3d point cloud
				if(clouds.contains(cloudName))
				{
					// Update only if the pose has changed
					Transform tCloud;
					cloudViewer_->getPose(cloudName, tCloud);
					if(tCloud.isNull() || iter->second != tCloud)
					{
						if(!cloudViewer_->updateCloudPose(cloudName, iter->second))
						{
							UERROR("Updating pose cloud %d failed!", iter->first);
						}
					}
					cloudViewer_->setCloudVisibility(cloudName, true);
				}
				else if(iter->first == stats.refImageId() &&
						stats.getSignature().id() == iter->first)
				{
					// Add the new cloud
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
							stats.getSignature().getImageRaw(),
							stats.getSignature().getDepthRaw(),
							stats.getSignature().getDepthCx(),
							stats.getSignature().getDepthCy(),
							stats.getSignature().getDepthFx(),
							stats.getSignature().getDepthFy(),
						   8); // decimation

					if(cloud->size())
					{
						cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, 4.0f);
						if(cloud->size())
						{
							cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, stats.getSignature().getLocalTransform());
						}
					}
					if(!cloudViewer_->addOrUpdateCloud(cloudName, cloud, iter->second))
					{
						UERROR("Adding cloud %d to viewer failed!", iter->first);
					}
				}
			}
		}

		cloudViewer_->render();

		_processingStatistics = false;
	}

protected:
	virtual void handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("RtabmapEvent") == 0)
		{
			RtabmapEvent * rtabmapEvent = (RtabmapEvent *)event;
			const Statistics & stats = rtabmapEvent->getStats();
			// Statistics must be processed in the Qt thread
			if(this->isVisible())
			{
				QMetaObject::invokeMethod(this, "processStatistics", Q_ARG(rtabmap::Statistics, stats));
			}
		}
		else if(event->getClassName().compare("OdometryEvent") == 0)
		{
			OdometryEvent * odomEvent = (OdometryEvent *)event;
			// Odometry must be processed in the Qt thread
			if(this->isVisible() &&
			   _lastOdometryProcessed &&
			   !_processingStatistics)
			{
				_lastOdometryProcessed = false; // if we receive too many odometry events!
				QMetaObject::invokeMethod(this, "processOdometry", Q_ARG(rtabmap::SensorData, odomEvent->data()));
			}
		}
	}

private:
	CloudViewer * cloudViewer_;
	Transform lastOdomPose_;
	bool _processingStatistics;
	bool _lastOdometryProcessed;
};


#endif /* MAPBUILDER_H_ */
