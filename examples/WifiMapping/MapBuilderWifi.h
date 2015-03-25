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

#ifndef MAPBUILDERWIFI_H_
#define MAPBUILDERWIFI_H_

#include "../RGBDMapping/MapBuilder.h"

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

protected slots:
	virtual void processStatistics(const rtabmap::Statistics & stats)
	{
		processingStatistics_ = true;

		const std::map<int, Transform> & poses = stats.poses();
		QMap<std::string, Transform> clouds = cloudViewer_->getAddedClouds();

		//============================
		// Add WIFI symbols
		//============================
		std::map<double, int> nodeStamps; // <stamp, id>
		std::map<int, std::pair<int, double> > wifiLevels;

		UASSERT(stats.getStamps().size() == stats.getUserDatas().size());
		std::map<int, double>::const_iterator iterStamps = stats.getStamps().begin();
		std::map<int, std::vector<unsigned char> >::const_iterator iterUserDatas = stats.getUserDatas().begin();
		for(; iterStamps!=stats.getStamps().end() && iterUserDatas!=stats.getUserDatas().end(); ++iterStamps, ++iterUserDatas)
		{
			// Sort stamps by stamps
			nodeStamps.insert(std::make_pair(iterStamps->second, iterStamps->first));

			// convert userData to wifi levels
			if(iterUserDatas->second.size())
			{
				UASSERT(iterUserDatas->second.size() == sizeof(int)+sizeof(double));

				// format [int level, double stamp]
				int level;
				double stamp;
				memcpy(&level, iterUserDatas->second.data(), sizeof(int));
				memcpy(&stamp, iterUserDatas->second.data()+sizeof(int), sizeof(double));

				wifiLevels.insert(std::make_pair(iterUserDatas->first, std::make_pair(level, stamp)));
			}
		}

		for(std::map<int, std::pair<int, double> >::iterator iter=wifiLevels.begin(); iter!=wifiLevels.end(); ++iter)
		{
			// The Wifi value may be taken between two nodes, interpolate its position.
			double stampWifi = iter->second.second;
			std::map<double, int>::iterator previousNode = nodeStamps.lower_bound(stampWifi); // lower bound of the stamp
			if(previousNode!=nodeStamps.end() && previousNode->first > stampWifi && previousNode != nodeStamps.begin())
			{
				--previousNode;
			}
			std::map<double, int>::iterator nextNode = nodeStamps.upper_bound(iter->second.second); // upper bound of the stamp

			if(previousNode != nodeStamps.end() && nextNode != nodeStamps.end() &&
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

				std::string cloudName = uFormat("level%d", iter->first);
				if(clouds.contains(cloudName))
				{
					if(!cloudViewer_->updateCloudPose(cloudName, wifiPose))
					{
						UERROR("Updating pose cloud %d failed!", iter->first);
					}
				}
				else
				{
					// Make a line with points
					int quality = dBm2Quality(iter->second.first)/10;
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
					if(!cloudViewer_->addOrUpdateCloud(cloudName, cloud, wifiPose, Qt::yellow))
					{
						UERROR("Adding cloud %d to viewer failed!", iter->first);
					}
					else
					{
						cloudViewer_->setCloudPointSize(cloudName, 5);
					}
				}
			}
			else
			{
				UWARN("Bounds not found!");
			}
		}

		//============================
		// Add RGB-D clouds
		//============================
		MapBuilder::processStatistics(stats);
	}
};


#endif /* MAPBUILDERWIFI_H_ */
