/*
 * MapBuilder.h
 *
 *  Created on: 2014-02-24
 *      Author: mathieu
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
	MapBuilder()
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
		qRegisterMetaType<rtabmap::Image>("rtabmap::Image");
	}

	virtual ~MapBuilder()
	{
		this->unregisterFromEventsManager();
	}

private slots:
	void processOdometry(const rtabmap::Image & data)
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
			   data.depthConstant() > 0.0f)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
					data.image(),
					data.depth(),
					float(data.depth().cols/2),
					float(data.depth().rows/2),
					1.0f/data.depthConstant(),
					1.0f/data.depthConstant(),
					2); // decimation // high definition
				if(cloud->size())
				{
					cloud = util3d::passThrough(cloud, "z", 0, 4.0f);
					if(cloud->size())
					{
						cloud = util3d::transformPointCloud(cloud, data.localTransform());
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
	}


	void processStatistics(const rtabmap::Statistics & stats)
	{
		if(!this->isVisible())
		{
			return;
		}

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
						uContains(stats.getImages(), iter->first) &&
						uContains(stats.getDepths(), iter->first) &&
						uContains(stats.getDepthConstants(), iter->first) &&
						uContains(stats.getLocalTransforms(), iter->first))
				{
					// Add the new cloud
					cv::Mat rgb = util3d::uncompressImage(stats.getImages().at(iter->first));
					cv::Mat depth = util3d::uncompressImage(stats.getDepths().at(iter->first));
					float depthConstant = stats.getDepthConstants().at(iter->first);
					Transform localTransform = stats.getLocalTransforms().at(iter->first);
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
							rgb,
							depth,
						   float(depth.cols/2),
						   float(depth.rows/2),
						   1.0f/depthConstant,
						   1.0f/depthConstant,
						   8); // decimation

					if(cloud->size())
					{
						cloud = util3d::passThrough(cloud, "z", 0, 4.0f);
						if(cloud->size())
						{
							cloud = util3d::transformPointCloud(cloud, localTransform);
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
	}

protected:
	virtual void handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("RtabmapEvent") == 0)
		{
			RtabmapEvent * rtabmapEvent = (RtabmapEvent *)event;
			const Statistics & stats = rtabmapEvent->getStats();
			// Statistics must be processed in the Qt thread
			QMetaObject::invokeMethod(this, "processStatistics", Q_ARG(rtabmap::Statistics, stats));
		}
		else if(event->getClassName().compare("OdometryEvent") == 0)
		{
			OdometryEvent * odomEvent = (OdometryEvent *)event;
			// Odometry must be processed in the Qt thread
			QMetaObject::invokeMethod(this, "processOdometry", Q_ARG(rtabmap::Image, odomEvent->data()));
		}
	}

private:
	CloudViewer * cloudViewer_;
	Transform lastOdomPose_;
};


#endif /* MAPBUILDER_H_ */
