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

#ifndef MAPBUILDERWIFI_H_
#define MAPBUILDERWIFI_H_

#include "MapBuilder.h"
#include "rtabmap/core/UserDataEvent.h"

using namespace rtabmap;

// A percentage value that represents the signal quality
// of the network. WLAN_SIGNAL_QUALITY is of type ULONG.
// This member contains a value between 0 and 100. A value
// of 0 implies an actual RSSI signal strength of -100 dbm.
// A value of 100 implies an actual RSSI signal strength of -50 dbm.
// You can calculate the RSSI signal strength value for wlanSignalQuality
// values between 1 and 99 using linear interpolation.
inline int dBm2Quality(int dBm)
{
	// dBm to Quality:
    if(dBm <= -100)
        return 0;
    else if(dBm >= -50)
    	return 100;
    else
    	return 2 * (dBm + 100);
}

class MapBuilderWifi : public MapBuilder
{
	Q_OBJECT
public:
	// Camera ownership is not transferred!
	MapBuilderWifi(CameraThread * camera = 0) :
		MapBuilder(camera)
	{}

	virtual ~MapBuilderWifi()
	{
		this->unregisterFromEventsManager();
	}

protected Q_SLOTS:
	virtual void processStatistics(const rtabmap::Statistics & stats)
	{
		processingStatistics_ = true;

		const std::map<int, Transform> & poses = stats.poses();
		QMap<std::string, Transform> clouds = cloudViewer_->getAddedClouds();

		//============================
		// Add WIFI symbols
		//============================
		// Sort stamps by stamps->id
		nodeStamps_.insert(std::make_pair(stats.getLastSignatureData().getStamp(), stats.getLastSignatureData().id()));

		if(!stats.getLastSignatureData().sensorData().userDataRaw().empty())
		{
			UASSERT(stats.getLastSignatureData().sensorData().userDataRaw().type() == CV_64FC1 &&
					stats.getLastSignatureData().sensorData().userDataRaw().cols == 2 &&
					stats.getLastSignatureData().sensorData().userDataRaw().rows == 1);

			// format [int level, double stamp]
			int level = stats.getLastSignatureData().sensorData().userDataRaw().at<double>(0);
			double stamp = stats.getLastSignatureData().sensorData().userDataRaw().at<double>(1);
			wifiLevels_.insert(std::make_pair(stamp, level));
		}

		// for the logic below, we should keep only stamps for
		// nodes still in the graph (in case nodes are ignored when not moving)
		std::map<double, int> nodeStamps;
		for(std::map<double, int>::iterator iter=nodeStamps_.begin(); iter!=nodeStamps_.end(); ++iter)
		{
			std::map<int, Transform>::const_iterator jter = poses.find(iter->second);
			if(jter != poses.end())
			{
				nodeStamps.insert(*iter);
			}
		}

		int id = 0;
		for(std::map<double, int>::iterator iter=wifiLevels_.begin(); iter!=wifiLevels_.end(); ++iter, ++id)
		{
			// The Wifi value may be taken between two nodes, interpolate its position.
			double stampWifi = iter->first;
			std::map<double, int>::iterator previousNode = nodeStamps.lower_bound(stampWifi); // lower bound of the stamp
			if(previousNode!=nodeStamps.end() && previousNode->first > stampWifi && previousNode != nodeStamps.begin())
			{
				--previousNode;
			}
			std::map<double, int>::iterator nextNode = nodeStamps.upper_bound(stampWifi); // upper bound of the stamp

			if(previousNode != nodeStamps.end() &&
			   nextNode != nodeStamps.end() &&
			   previousNode->second != nextNode->second &&
			   uContains(poses, previousNode->second) && uContains(poses, nextNode->second))
			{
				Transform poseA = poses.at(previousNode->second);
				Transform poseB = poses.at(nextNode->second);
				double stampA = previousNode->first;
				double stampB = nextNode->first;
				UASSERT(stampWifi>=stampA && stampWifi <=stampB);

				Transform v = poseA.inverse() * poseB;
				double ratio = (stampWifi-stampA)/(stampB-stampA);

				v.x()*=ratio;
				v.y()*=ratio;
				v.z()*=ratio;

				Transform wifiPose = (poseA*v).translation(); // rip off the rotation

				std::string cloudName = uFormat("level%d", id);
				if(clouds.contains(cloudName))
				{
					if(!cloudViewer_->updateCloudPose(cloudName, wifiPose))
					{
						UERROR("Updating pose cloud %d failed!", id);
					}
				}
				else
				{
					// Make a line with points
					int quality = dBm2Quality(iter->second)/10;
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					for(int i=0; i<10; ++i)
					{
						// 2 cm between each points
						// the number of points depends on the dBm (which varies from -30 (near) to -80 (far))
						pcl::PointXYZRGB pt;
						pt.z = float(i+1)*0.02f;
						if(i<quality)
						{
							// yellow
							pt.r = 255;
							pt.g = 255;
						}
						else
						{
							// gray
							pt.r = pt.g = pt.b = 100;
						}
						cloud->push_back(pt);
					}
					pcl::PointXYZRGB anchor(255, 0, 0);
					cloud->push_back(anchor);
					//UWARN("level %d -> %d pose=%s size=%d", level, iter->second.first, wifiPose.prettyPrint().c_str(), (int)cloud->size());
					if(!cloudViewer_->addCloud(cloudName, cloud, wifiPose, Qt::yellow))
					{
						UERROR("Adding cloud %d to viewer failed!", id);
					}
					else
					{
						cloudViewer_->setCloudPointSize(cloudName, 5);
					}
				}
			}
		}

		//============================
		// Add RGB-D clouds
		//============================
		MapBuilder::processStatistics(stats);
	}

private:
	std::map<double, int> wifiLevels_;
	std::map<double, int> nodeStamps_; // <stamp, id>
};


#endif /* MAPBUILDERWIFI_H_ */
