/*
Copyright (c) 2010-2022, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the names of its contributors may be used to endorse or promote products
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

#include <QVBoxLayout>
#include <QtCore/QMetaType>
#include <QAction>

#ifndef Q_MOC_RUN // Mac OS X issue
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/OccupancyGrid.h"
#endif
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/OdometryEvent.h"
#include <rtabmap/core/SensorCaptureThread.h>

using namespace rtabmap;

// This class receives RtabmapEvent and construct/update a 3D Map
class MapBuilder : public QWidget, public UEventsHandler
{
	Q_OBJECT
public:
	//Camera ownership is not transferred!
	MapBuilder(SensorCaptureThread * camera = 0) :
		sensorCaptureThread_(camera),
		odometryCorrection_(Transform::getIdentity()),
		processingStatistics_(false),
		lastOdometryProcessed_(true),
		visibility_(0)
	{
		this->setWindowFlags(Qt::Dialog);
		this->setWindowTitle(tr("3D Map"));
		this->setMinimumWidth(800);
		this->setMinimumHeight(600);

		cloudViewer_ = new CloudViewer(this);
		cloudViewer_->setCameraTargetLocked(true);

		QVBoxLayout *layout = new QVBoxLayout();
		layout->addWidget(cloudViewer_);
		this->setLayout(layout);

		qRegisterMetaType<rtabmap::OdometryEvent>("rtabmap::OdometryEvent");
		qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");

		QAction * pause = new QAction(this);
		this->addAction(pause);
		pause->setShortcut(Qt::Key_Space);
		connect(pause, SIGNAL(triggered()), this, SLOT(pauseDetection()));

		QAction * visibility = new QAction(this);
		this->addAction(visibility);
		visibility->setShortcut(Qt::Key_Tab);
		connect(visibility, SIGNAL(triggered()), this, SLOT(rotateVisibility()));
	}

	virtual ~MapBuilder()
	{
		this->unregisterFromEventsManager();
	}

protected Q_SLOTS:
	virtual void pauseDetection()
	{
		UWARN("");
		if(sensorCaptureThread_)
		{
			if(sensorCaptureThread_->isCapturing())
			{
				sensorCaptureThread_->join(true);
			}
			else
			{
				sensorCaptureThread_->start();
			}
		}
	}

	virtual void rotateVisibility()
	{
		visibility_ = (visibility_+1) % 3;
		if(visibility_ == 0)
		{
			UWARN("Show both Odom and Map");
		}
		else if(visibility_ == 1)
		{
			UWARN("Show only Map");
		}
		else if(visibility_ == 2)
		{
			UWARN("Show only Odom");
		}
	}

	virtual void processOdometry(const rtabmap::OdometryEvent & odom)
	{
		if(!this->isVisible())
		{
			return;
		}

		Transform pose = odom.pose();
		if(pose.isNull())
		{
			//Odometry lost
			cloudViewer_->setBackgroundColor(Qt::darkRed);

			pose = lastOdomPose_;
		}
		else
		{
			cloudViewer_->setBackgroundColor(cloudViewer_->getDefaultBackgroundColor());
		}
		if(!pose.isNull())
		{
			lastOdomPose_ = pose;

			// 3d cloud
			if(!odom.data().laserScanRaw().empty())
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::laserScanToPointCloud(odom.data().laserScanRaw(), odom.data().laserScanRaw().localTransform());
				if(cloud->size() && (visibility_ == 0 || visibility_ == 2))
				{
					if(!cloudViewer_->addCloud("cloudOdom", cloud, odometryCorrection_*pose, Qt::magenta))
					{
						UERROR("Adding cloudOdom to viewer failed!");
					}
				}
				else
				{
					cloudViewer_->setCloudVisibility("cloudOdom", false);
					if(cloud->empty())
						UWARN("Empty cloudOdom!");
				}
			}

			if(!odom.info().localScanMap.empty())
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::laserScanToPointCloud(odom.info().localScanMap, odom.info().localScanMap.localTransform());
				if(cloud->size() && (visibility_ == 0 || visibility_ == 2))
				{
					if(!cloudViewer_->addCloud("cloudOdomLocalMap", cloud, odometryCorrection_, Qt::blue))
					{
						UERROR("Adding cloudOdomLocalMap to viewer failed!");
					}
				}
				else
				{
					cloudViewer_->setCloudVisibility("cloudOdomLocalMap", false);
					if(cloud->empty())
						UWARN("Empty cloudOdomLocalMap!");
				}
			}

			if(!odom.pose().isNull())
			{
				// update camera position
				cloudViewer_->updateCameraTargetPosition(odometryCorrection_*odom.pose());
			}
		}
		cloudViewer_->update();

		lastOdometryProcessed_ = true;
	}


	virtual void processStatistics(const rtabmap::Statistics & stats)
	{
		processingStatistics_ = true;

		//============================
		// Add RGB-D clouds
		//============================
		const std::map<int, Transform> & poses = stats.poses();
		QMap<std::string, Transform> clouds = cloudViewer_->getAddedClouds();
		for(std::map<int, Transform>::const_iterator iter = poses.lower_bound(1); iter!=poses.end(); ++iter)
		{
			if(!iter->second.isNull())
			{
				std::string cloudName = uFormat("cloud_%d", iter->first);

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
				}
				else if(iter->first == stats.getLastSignatureData().id())
				{
					Signature s = stats.getLastSignatureData();
					s.sensorData().uncompressData(); // make sure data is uncompressed
					// Add the new cloud
					pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = util3d::laserScanToPointCloudI(
							s.sensorData().laserScanRaw(),
						    s.sensorData().laserScanRaw().localTransform());
					if(cloud->size())
					{
						if(!cloudViewer_->addCloud(cloudName, cloud, iter->second))
						{
							UERROR("Adding cloud %d to viewer failed!", iter->first);
						}
					}
					else
					{
						UWARN("Empty cloud %d!", iter->first);
					}
				}

				cloudViewer_->setCloudVisibility(cloudName, visibility_ == 0 || visibility_ == 1);
			}
			else
			{
				UWARN("Null pose for %d ?!?", iter->first);
			}
		}

		// cleanup
		for(QMap<std::string, Transform>::iterator iter = clouds.begin(); iter!=clouds.end(); ++iter)
		{
			if(uStrContains(iter.key(), "cloud_"))
			{
				int id = uStr2Int(uSplitNumChar(iter.key()).back());
				if(poses.find(id) == poses.end())
				{
					cloudViewer_->removeCloud(iter.key());
				}
			}
		}

		//============================
		// Add 3D graph (show all poses)
		//============================
		cloudViewer_->removeAllGraphs();
		cloudViewer_->removeCloud("graph_nodes");
		if(poses.size())
		{
			// Set graph
			pcl::PointCloud<pcl::PointXYZ>::Ptr graph(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr graphNodes(new pcl::PointCloud<pcl::PointXYZ>);
			for(std::map<int, Transform>::const_iterator iter=poses.lower_bound(1); iter!=poses.end(); ++iter)
			{
				graph->push_back(pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z()));
			}
			*graphNodes = *graph;


			// add graph
			cloudViewer_->addOrUpdateGraph("graph", graph, Qt::gray);
			cloudViewer_->addCloud("graph_nodes", graphNodes, Transform::getIdentity(), Qt::green);
			cloudViewer_->setCloudPointSize("graph_nodes", 5);
		}

		odometryCorrection_ = stats.mapCorrection();

		cloudViewer_->update();

		processingStatistics_ = false;
	}

	virtual bool handleEvent(UEvent * event)
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
			   lastOdometryProcessed_ &&
			   !processingStatistics_)
			{
				lastOdometryProcessed_ = false; // if we receive too many odometry events!
				QMetaObject::invokeMethod(this, "processOdometry", Q_ARG(rtabmap::OdometryEvent, *odomEvent));
			}
		}
		return false;
	}

protected:
	CloudViewer * cloudViewer_;
	SensorCaptureThread * sensorCaptureThread_;
	Transform lastOdomPose_;
	Transform odometryCorrection_;
	bool processingStatistics_;
	bool lastOdometryProcessed_;
	int visibility_;
};


#endif /* MAPBUILDER_H_ */
