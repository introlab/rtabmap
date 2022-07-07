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
#include "rtabmap/core/CameraThread.h"

using namespace rtabmap;

// This class receives RtabmapEvent and construct/update a 3D Map
class MapBuilder : public QWidget, public UEventsHandler
{
	Q_OBJECT
public:
	//Camera ownership is not transferred!
	MapBuilder(CameraThread * camera = 0) :
		camera_(camera),
		odometryCorrection_(Transform::getIdentity()),
		processingStatistics_(false),
		lastOdometryProcessed_(true)
	{
		this->setWindowFlags(Qt::Dialog);
		this->setWindowTitle(tr("3D Map"));
		this->setMinimumWidth(800);
		this->setMinimumHeight(600);

		cloudViewer_ = new CloudViewer(this);

		QVBoxLayout *layout = new QVBoxLayout();
		layout->addWidget(cloudViewer_);
		this->setLayout(layout);

		qRegisterMetaType<rtabmap::OdometryEvent>("rtabmap::OdometryEvent");
		qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");

		QAction * pause = new QAction(this);
		this->addAction(pause);
		pause->setShortcut(Qt::Key_Space);
		connect(pause, SIGNAL(triggered()), this, SLOT(pauseDetection()));
	}

	virtual ~MapBuilder()
	{
		this->unregisterFromEventsManager();
	}

protected Q_SLOTS:
	virtual void pauseDetection()
	{
		UWARN("");
		if(camera_)
		{
			if(camera_->isCapturing())
			{
				camera_->join(true);
			}
			else
			{
				camera_->start();
			}
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
			if(odom.data().depthOrRightRaw().cols == odom.data().imageRaw().cols &&
			   odom.data().depthOrRightRaw().rows == odom.data().imageRaw().rows &&
			   !odom.data().depthOrRightRaw().empty() &&
			   (odom.data().stereoCameraModels().size() || odom.data().cameraModels().size()))
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
					odom.data(),
					2,     // decimation
					4.0f); // max depth
				if(cloud->size())
				{
					if(!cloudViewer_->addCloud("cloudOdom", cloud, odometryCorrection_*pose))
					{
						UERROR("Adding cloudOdom to viewer failed!");
					}
				}
				else
				{
					cloudViewer_->setCloudVisibility("cloudOdom", false);
					UWARN("Empty cloudOdom!");
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
				else if(iter->first == stats.getLastSignatureData().id())
				{
					Signature s = stats.getLastSignatureData();
					s.sensorData().uncompressData(); // make sure data is uncompressed
					// Add the new cloud
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
							s.sensorData(),
						    4,     // decimation
						    4.0f); // max depth
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
			}
			else
			{
				UWARN("Null pose for %d ?!?", iter->first);
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

		//============================
		// Update/add occupancy grid (when RGBD/CreateOccupancyGrid is true)
		//============================
		if(grid_.addedNodes().find(stats.getLastSignatureData().id()) == grid_.addedNodes().end())
		{
			if(stats.getLastSignatureData().sensorData().gridCellSize() > 0.0f)
			{
				cv::Mat groundCells, obstacleCells, emptyCells;
				stats.getLastSignatureData().sensorData().uncompressDataConst(0, 0, 0, 0, &groundCells, &obstacleCells, &emptyCells);
				grid_.addToCache(stats.getLastSignatureData().id(), groundCells, obstacleCells, emptyCells);
			}
		}

		if(grid_.addedNodes().size() || grid_.cacheSize())
		{
			grid_.update(stats.poses());
		}
		if(grid_.addedNodes().size())
		{
			float xMin, yMin;
			cv::Mat map8S = grid_.getMap(xMin, yMin);
			if(!map8S.empty())
			{
				//convert to gray scaled map
				cv::Mat map8U = util3d::convertMap2Image8U(map8S);
				cloudViewer_->addOccupancyGridMap(map8U, grid_.getCellSize(), xMin, yMin, 0.75);
			}
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
	CameraThread * camera_;
	Transform lastOdomPose_;
	Transform odometryCorrection_;
	bool processingStatistics_;
	bool lastOdometryProcessed_;
	OccupancyGrid grid_;
};


#endif /* MAPBUILDER_H_ */
