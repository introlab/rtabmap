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

#include "rtabmap/core/util3d_mapping.h"

#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>

namespace rtabmap
{

namespace util3d
{
void occupancy2DFromLaserScan(
		const cv::Mat & scan,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize,
		bool unknownSpaceFilled,
		float scanMaxRange)
{
	if(scan.empty())
	{
		return;
	}

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform::getIdentity()));

	pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud = util3d::laserScanToPointCloud(scan);
	//obstaclesCloud = util3d::voxelize<pcl::PointXYZ>(obstaclesCloud, cellSize);

	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> scans;
	scans.insert(std::make_pair(1, obstaclesCloud));

	float xMin, yMin;
	cv::Mat map8S = create2DMap(poses, scans, cellSize, unknownSpaceFilled, xMin, yMin, 0.0f, scanMaxRange);

	// find ground cells
	std::list<int> groundIndices;
	for(unsigned int i=0; i< map8S.total(); ++i)
	{
		if(map8S.data[i] == 0)
		{
			groundIndices.push_back(i);
		}
	}

	// Convert to position matrices, get points to each center of the cells
	ground = cv::Mat();
	if(groundIndices.size())
	{
		ground = cv::Mat((int)groundIndices.size(), 1, CV_32FC2);
		int i=0;
		for(std::list<int>::iterator iter=groundIndices.begin();iter!=groundIndices.end(); ++iter)
		{
			int x = *iter / map8S.cols;
			int y = *iter - x*map8S.cols;
			ground.at<cv::Vec2f>(i)[0] = (float(y)+0.5)*cellSize + xMin;
			ground.at<cv::Vec2f>(i)[1] = (float(x)+0.5)*cellSize + yMin;
			++i;
		}
	}

	// copy directly obstacles precise positions
	obstacles = cv::Mat();
	if(obstaclesCloud->size())
	{
		obstacles = cv::Mat((int)obstaclesCloud->size(), 1, CV_32FC2);
		for(unsigned int i=0;i<obstaclesCloud->size(); ++i)
		{
			obstacles.at<cv::Vec2f>(i)[0] = obstaclesCloud->at(i).x;
			obstacles.at<cv::Vec2f>(i)[1] = obstaclesCloud->at(i).y;
		}
	}
}

/**
 * Create 2d Occupancy grid (CV_8S) from 2d occupancy
 * -1 = unknown
 * 0 = empty space
 * 100 = obstacle
 * @param poses
 * @param occupancy <empty, occupied>
 * @param cellSize m
 * @param xMin
 * @param yMin
 * @param minMapSize minimum width (m)
 * @param erode
 */
cv::Mat create2DMapFromOccupancyLocalMaps(
		const std::map<int, Transform> & posesIn,
		const std::map<int, std::pair<cv::Mat, cv::Mat> > & occupancy,
		float cellSize,
		float & xMin,
		float & yMin,
		float minMapSize,
		bool erode)
{
	UASSERT(minMapSize >= 0.0f);
	UDEBUG("cellSize=%f m, minMapSize=%f m, erode=%d", cellSize, minMapSize, erode?1:0);
	UTimer timer;

	std::map<int, cv::Mat> emptyLocalMaps;
	std::map<int, cv::Mat> occupiedLocalMaps;

	std::list<std::pair<int, Transform> > poses;
	// place negative poses at the end
	for(std::map<int, Transform>::const_reverse_iterator iter = posesIn.rbegin(); iter!=posesIn.rend(); ++iter)
	{
		if(iter->first>0)
		{
			poses.push_front(*iter);
		}
		else
		{
			poses.push_back(*iter);
		}
	}

	float minX=-minMapSize/2.0, minY=-minMapSize/2.0, maxX=minMapSize/2.0, maxY=minMapSize/2.0;
	bool undefinedSize = minMapSize == 0.0f;
	float x=0.0f,y=0.0f,z=0.0f,roll=0.0f,pitch=0.0f,yaw=0.0f,cosT=0.0f,sinT=0.0f;
	cv::Mat affineTransform(2,3,CV_32FC1);
	for(std::list<std::pair<int, Transform> >::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		UASSERT(!iter->second.isNull());

		iter->second.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);

		if(undefinedSize)
		{
			minX = maxX = x;
			minY = maxY = y;
			undefinedSize = false;
		}
		else
		{
			if(minX > x)
				minX = x;
			else if(maxX < x)
				maxX = x;

			if(minY > y)
				minY = y;
			else if(maxY < y)
				maxY = y;
		}

		if(uContains(occupancy, iter->first))
		{
			const std::pair<cv::Mat, cv::Mat> & pair = occupancy.at(iter->first);
			cosT = cos(yaw);
			sinT = sin(yaw);
			affineTransform.at<float>(0,0) = cosT;
			affineTransform.at<float>(0,1) = -sinT;
			affineTransform.at<float>(1,0) = sinT;
			affineTransform.at<float>(1,1) = cosT;
			affineTransform.at<float>(0,2) = x;
			affineTransform.at<float>(1,2) = y;

			//ground
			if(pair.first.rows)
			{
				UASSERT(pair.first.type() == CV_32FC2);
				cv::Mat ground(pair.first.rows, pair.first.cols, pair.first.type());
				cv::transform(pair.first, ground, affineTransform);
				for(int i=0; i<ground.rows; ++i)
				{
					if(minX > ground.at<float>(i,0))
						minX = ground.at<float>(i,0);
					else if(maxX < ground.at<float>(i,0))
						maxX = ground.at<float>(i,0);

					if(minY > ground.at<float>(i,1))
						minY = ground.at<float>(i,1);
					else if(maxY < ground.at<float>(i,1))
						maxY = ground.at<float>(i,1);
				}
				emptyLocalMaps.insert(std::make_pair(iter->first, ground));
			}

			//obstacles
			if(pair.second.rows)
			{
				UASSERT(pair.second.type() == CV_32FC2);
				cv::Mat obstacles(pair.second.rows, pair.second.cols, pair.second.type());
				cv::transform(pair.second, obstacles, affineTransform);
				for(int i=0; i<obstacles.rows; ++i)
				{
					if(minX > obstacles.at<float>(i,0))
						minX = obstacles.at<float>(i,0);
					else if(maxX < obstacles.at<float>(i,0))
						maxX = obstacles.at<float>(i,0);

					if(minY > obstacles.at<float>(i,1))
						minY = obstacles.at<float>(i,1);
					else if(maxY < obstacles.at<float>(i,1))
						maxY = obstacles.at<float>(i,1);
				}
				occupiedLocalMaps.insert(std::make_pair(iter->first, obstacles));
			}
		}
	}
	UDEBUG("timer=%fs", timer.ticks());

	cv::Mat map;
	if(minX != maxX && minY != maxY)
	{
		//Get map size
		float margin = cellSize*10.0f;
		xMin = minX-margin;
		yMin = minY-margin;
		float xMax = maxX+margin;
		float yMax = maxY+margin;
		if(fabs((yMax - yMin) / cellSize) > 99999 ||
		   fabs((xMax - xMin) / cellSize) > 99999)
		{
			UERROR("Large map size!! map min=(%f, %f) max=(%f,%f). "
					"There's maybe an error with the poses provided! The map will not be created!",
					xMin, yMin, xMax, yMax);
		}
		else
		{
			UDEBUG("map min=(%f, %f) max=(%f,%f)", xMin, yMin, xMax, yMax);


			map = cv::Mat::ones((yMax - yMin) / cellSize + 0.5f, (xMax - xMin) / cellSize + 0.5f, CV_8S)*-1;
			for(std::list<std::pair<int, Transform> >::const_iterator kter = poses.begin(); kter!=poses.end(); ++kter)
			{
				std::map<int, cv::Mat >::iterator iter = emptyLocalMaps.find(kter->first);
				std::map<int, cv::Mat >::iterator jter = occupiedLocalMaps.find(kter->first);
				if(iter!=emptyLocalMaps.end())
				{
					for(int i=0; i<iter->second.rows; ++i)
					{
						cv::Point2i pt((iter->second.at<float>(i,0)-xMin)/cellSize + 0.5f, (iter->second.at<float>(i,1)-yMin)/cellSize + 0.5f);
						map.at<char>(pt.y, pt.x) = 0; // free space
					}
				}
				if(jter!=occupiedLocalMaps.end())
				{
					for(int i=0; i<jter->second.rows; ++i)
					{
						cv::Point2i pt((jter->second.at<float>(i,0)-xMin)/cellSize + 0.5f, (jter->second.at<float>(i,1)-yMin)/cellSize + 0.5f);
						map.at<char>(pt.y, pt.x) = 100; // obstacles
					}
				}

				//UDEBUG("empty=%d occupied=%d", empty, occupied);
			}

			// fill holes and remove empty from obstacle borders
			cv::Mat updatedMap = map.clone();
			std::list<std::pair<int, int> > obstacleIndices;
			for(int i=2; i<map.rows-2; ++i)
			{
				for(int j=2; j<map.cols-2; ++j)
				{
					if(map.at<char>(i, j) == -1 &&
						map.at<char>(i+1, j) != -1 &&
						map.at<char>(i-1, j) != -1 &&
						map.at<char>(i, j+1) != -1 &&
						map.at<char>(i, j-1) != -1)
					{
						updatedMap.at<char>(i, j) = 0;
					}
					else if(map.at<char>(i, j) == 100)
					{
						// obstacle/empty/unknown -> remove empty
						// unknown/empty/obstacle -> remove empty
						if(map.at<char>(i-1, j) == 0 &&
							map.at<char>(i-2, j) == -1)
						{
							updatedMap.at<char>(i-1, j) = -1;
						}
						else if(map.at<char>(i+1, j) == 0 &&
								map.at<char>(i+2, j) == -1)
						{
							updatedMap.at<char>(i+1, j) = -1;
						}
						if(map.at<char>(i, j-1) == 0 &&
							map.at<char>(i, j-2) == -1)
						{
							updatedMap.at<char>(i, j-1) = -1;
						}
						else if(map.at<char>(i, j+1) == 0 &&
								map.at<char>(i, j+2) == -1)
						{
							updatedMap.at<char>(i, j+1) = -1;
						}

						if(erode)
						{
							obstacleIndices.push_back(std::make_pair(i, j));
						}
					}
					else if(map.at<char>(i, j) == 0)
					{
						// obstacle/empty/obstacle -> remove empty
						if(map.at<char>(i-1, j) == 100 &&
							map.at<char>(i+1, j) == 100)
						{
							updatedMap.at<char>(i, j) = -1;
						}
						else if(map.at<char>(i, j-1) == 100 &&
							map.at<char>(i, j+1) == 100)
						{
							updatedMap.at<char>(i, j) = -1;
						}
					}

				}
			}
			map = updatedMap;

			if(erode)
			{
				// remove obstacles which touch at least 3 empty cells but not unknown cells
				cv::Mat erodedMap = map.clone();
				for(std::list<std::pair<int,int> >::iterator iter = obstacleIndices.begin();
					iter!= obstacleIndices.end();
					++iter)
				{
					int i = iter->first;
					int j = iter->second;
					int touchEmpty = (map.at<char>(i+1, j) == 0?1:0) +
						(map.at<char>(i-1, j) == 0?1:0) +
						(map.at<char>(i, j+1) == 0?1:0) +
						(map.at<char>(i, j-1) == 0?1:0);
					if(touchEmpty>=3 && map.at<char>(i+1, j) != -1 &&
						map.at<char>(i-1, j) != -1 &&
						map.at<char>(i, j+1) != -1 &&
						map.at<char>(i, j-1) != -1)
					{
						erodedMap.at<char>(i, j) = 0; // empty
					}
				}
				map = erodedMap;
			}
		}
	}
	UDEBUG("timer=%fs", timer.ticks());
	return map;
}

/**
 * Create 2d Occupancy grid (CV_8S)
 * -1 = unknown
 * 0 = empty space
 * 100 = obstacle
 * @param poses
 * @param scans
 * @param cellSize m
 * @param unknownSpaceFilled if false no fill, otherwise a virtual laser sweeps the unknown space from each pose (stopping on detected obstacle)
 * @param xMin
 * @param yMin
 * @param minMapSize minimum map size in meters
 * @param scanMaxRange laser scan maximum range, would be set if unknownSpaceFilled=true
 */
cv::Mat create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans,
		float cellSize,
		bool unknownSpaceFilled,
		float & xMin,
		float & yMin,
		float minMapSize,
		float scanMaxRange)
{
	UDEBUG("poses=%d, scans = %d scanMaxRange=%f", poses.size(), scans.size(), scanMaxRange);
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > localScans;

	pcl::PointCloud<pcl::PointXYZ> minMax;
	if(minMapSize > 0.0f)
	{
		minMax.push_back(pcl::PointXYZ(-minMapSize/2.0, -minMapSize/2.0, 0));
		minMax.push_back(pcl::PointXYZ(minMapSize/2.0, minMapSize/2.0, 0));
	}
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(uContains(scans, iter->first) && scans.at(iter->first)->size())
		{
			UASSERT(!iter->second.isNull());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::transformPointCloud(scans.at(iter->first), iter->second);
			pcl::PointXYZ min, max;
			pcl::getMinMax3D(*cloud, min, max);
			minMax.push_back(min);
			minMax.push_back(max);
			minMax.push_back(pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z()));
			localScans.insert(std::make_pair(iter->first, cloud));
		}
	}

	cv::Mat map;
	if(minMax.size())
	{
		//Get map size
		pcl::PointXYZ min, max;
		pcl::getMinMax3D(minMax, min, max);

		// Added margin to make sure that all points are inside the map (when rounded to integer)
		float margin = cellSize*10.0f;
		xMin = (unknownSpaceFilled && scanMaxRange > 0 && -scanMaxRange < min.x?-scanMaxRange:min.x) - margin;
		yMin = (unknownSpaceFilled && scanMaxRange > 0 && -scanMaxRange < min.y?-scanMaxRange:min.y) - margin;
		float xMax = (unknownSpaceFilled && scanMaxRange > 0 && scanMaxRange > max.x?scanMaxRange:max.x) + margin;
		float yMax = (unknownSpaceFilled && scanMaxRange > 0 && scanMaxRange > max.y?scanMaxRange:max.y) + margin;

		//UWARN("map min=(%fm, %fm) max=(%fm,%fm) (margin=%fm, cellSize=%fm, scan range=%f, min=[%fm,%fm] max=[%fm,%fm])",
		//		xMin, yMin, xMax, yMax, margin, cellSize, scanMaxRange, min.x, min.y, max.x, max.y);

		UTimer timer;

		map = cv::Mat::ones((yMax - yMin) / cellSize + 0.5f, (xMax - xMin) / cellSize + 0.5f, CV_8S)*-1;
		int j=0;
		for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter = localScans.begin(); iter!=localScans.end(); ++iter)
		{
			const Transform & pose = poses.at(iter->first);
			cv::Point2i start((pose.x()-xMin)/cellSize + 0.5f, (pose.y()-yMin)/cellSize + 0.5f);
			for(unsigned int i=0; i<iter->second->size(); ++i)
			{
				cv::Point2i end((iter->second->points[i].x-xMin)/cellSize, (iter->second->points[i].y-yMin)/cellSize);
				if(end!=start)
				{
					rayTrace(start, end, map, true); // trace free space
					map.at<char>(end.y, end.x) = 100; // obstacle
				}
			}
			++j;
		}
		UDEBUG("Ray trace known space=%fs", timer.ticks());

		// now fill unknown spaces
		if(unknownSpaceFilled && scanMaxRange > 0)
		{
			j=0;
			float a = CV_PI/256.0f; // angle increment
			for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter = localScans.begin(); iter!=localScans.end(); ++iter)
			{
				if(iter->second->size() > 1)
				{
					if(scanMaxRange > cellSize)
					{
						const Transform & pose = poses.at(iter->first);
						cv::Point2i start((pose.x()-xMin)/cellSize + 0.5f, (pose.y()-yMin)/cellSize + 0.5f);

						//UWARN("maxLength = %f", maxLength);
						//rotate counterclockwise from the first point until we pass the last point
						// Note: assuming that first laser scan is negative y
						cv::Mat rotation = (cv::Mat_<float>(2,2) << cos(a), -sin(a),
																	 sin(a), cos(a));
						cv::Mat origin(2,1,CV_32F), endFirst(2,1,CV_32F), endLast(2,1,CV_32F);
						origin.at<float>(0) = pose.x();
						origin.at<float>(1) = pose.y();
						pcl::PointXYZ ptFirst = iter->second->points[0];
						pcl::PointXYZ ptLast = iter->second->points[iter->second->points.size()-1];
						//if(ptFirst.y > ptLast.y)
						//{
							// swap to iterate counterclockwise
						//	pcl::PointXYZ tmp = ptLast;
						//	ptLast = ptFirst;
						//	ptFirst = tmp;
						//}
						endFirst.at<float>(0) = ptFirst.x;
						endFirst.at<float>(1) = ptFirst.y;
						endLast.at<float>(0) = ptLast.x;
						endLast.at<float>(1) = ptLast.y;
						//UWARN("origin = %f %f", origin.at<float>(0), origin.at<float>(1));
						//UWARN("endFirst = %f %f", endFirst.at<float>(0), endFirst.at<float>(1));
						//UWARN("endLast = %f %f", endLast.at<float>(0), endLast.at<float>(1));
						cv::Mat tmp = (endFirst - origin);
						cv::Mat endRotated = rotation*((tmp/cv::norm(tmp))*scanMaxRange) + origin;
						cv::Mat endLastVector(3,1,CV_32F), endRotatedVector(3,1,CV_32F);
						endLastVector.at<float>(0) = endLast.at<float>(0) - origin.at<float>(0);
						endLastVector.at<float>(1) = endLast.at<float>(1) - origin.at<float>(1);
						endLastVector.at<float>(2) = 0.0f;
						endRotatedVector.at<float>(0) = endRotated.at<float>(0) - origin.at<float>(0);
						endRotatedVector.at<float>(1) = endRotated.at<float>(1) - origin.at<float>(1);
						endRotatedVector.at<float>(2) = 0.0f;
						//UWARN("endRotated = %f %f", endRotated.at<float>(0), endRotated.at<float>(1));
						float normEndRotatedVector = cv::norm(endRotatedVector);
						endLastVector = endLastVector / cv::norm(endLastVector);
						float angle = (endRotatedVector/normEndRotatedVector).dot(endLastVector);
						angle = angle<-1.0f?-1.0f:angle>1.0f?1.0f:angle;
						while(acos(angle) > M_PI_4 || endRotatedVector.cross(endLastVector).at<float>(2) > 0.0f)
						{
							cv::Point2i end((endRotated.at<float>(0)-xMin)/cellSize + 0.5f, (endRotated.at<float>(1)-yMin)/cellSize + 0.5f);
							//end must be inside the grid
							end.x = end.x < 0?0:end.x;
							end.x = end.x >= map.cols?map.cols-1:end.x;
							end.y = end.y < 0?0:end.y;
							end.y = end.y >= map.rows?map.rows-1:end.y;
							rayTrace(start, end, map, true); // trace free space
							// next point
							endRotated = rotation*(endRotated - origin) + origin;
							endRotatedVector.at<float>(0) = endRotated.at<float>(0) - origin.at<float>(0);
							endRotatedVector.at<float>(1) = endRotated.at<float>(1) - origin.at<float>(1);
							angle = (endRotatedVector/normEndRotatedVector).dot(endLastVector);
							angle = angle<-1.0f?-1.0f:angle>1.0f?1.0f:angle;

							//UWARN("endRotated = %f %f (%f %f %f)",
							//		endRotated.at<float>(0), endRotated.at<float>(1),
							//		acos(angle),
							//		angle,
							//		endRotatedVector.cross(endLastVector).at<float>(2));
						}
					}
				}
				++j;
			}
			UDEBUG("Fill empty space=%fs", timer.ticks());
			//cv::imwrite("map.png", util3d::convertMap2Image8U(map));
			//UWARN("saved map.png");
		}
	}
	return map;
}

void rayTrace(const cv::Point2i & start, const cv::Point2i & end, cv::Mat & grid, bool stopOnObstacle)
{
	UASSERT_MSG(start.x >= 0 && start.x < grid.cols, uFormat("start.x=%d grid.cols=%d", start.x, grid.cols).c_str());
	UASSERT_MSG(start.y >= 0 && start.y < grid.rows, uFormat("start.y=%d grid.rows=%d", start.y, grid.rows).c_str());
	UASSERT_MSG(end.x >= 0 && end.x < grid.cols, uFormat("end.x=%d grid.cols=%d", end.x, grid.cols).c_str());
	UASSERT_MSG(end.y >= 0 && end.y < grid.rows, uFormat("end.x=%d grid.cols=%d", end.y, grid.rows).c_str());

	cv::Point2i ptA, ptB;
	ptA = start;
	ptB = end;

	float slope = float(ptB.y - ptA.y)/float(ptB.x - ptA.x);

	bool swapped = false;
	if(slope<-1.0f || slope>1.0f)
	{
		// swap x and y
		slope = 1.0f/slope;

		int tmp = ptA.x;
		ptA.x = ptA.y;
		ptA.y = tmp;

		tmp = ptB.x;
		ptB.x = ptB.y;
		ptB.y = tmp;

		swapped = true;
	}

	float b = ptA.y - slope*ptA.x;
	for(int x=ptA.x; ptA.x<ptB.x?x<ptB.x:x>ptB.x; ptA.x<ptB.x?++x:--x)
	{
		int upperbound = float(x)*slope + b;
		int lowerbound = upperbound;
		if(x != ptA.x)
		{
			lowerbound = (ptA.x<ptB.x?x+1:x-1)*slope + b;
		}

		if(lowerbound > upperbound)
		{
			int tmp = upperbound;
			upperbound = lowerbound;
			lowerbound = tmp;
		}

		if(!swapped)
		{
			UASSERT_MSG(lowerbound >= 0 && lowerbound < grid.rows, uFormat("lowerbound=%f grid.rows=%d x=%d slope=%f b=%f x=%f", lowerbound, grid.rows, x, slope, b, x).c_str());
			UASSERT_MSG(upperbound >= 0 && upperbound < grid.rows, uFormat("upperbound=%f grid.rows=%d x+1=%d slope=%f b=%f x=%f", upperbound, grid.rows, x+1, slope, b, x).c_str());
		}
		else
		{
			UASSERT_MSG(lowerbound >= 0 && lowerbound < grid.cols, uFormat("lowerbound=%f grid.cols=%d x=%d slope=%f b=%f x=%f", lowerbound, grid.cols, x, slope, b, x).c_str());
			UASSERT_MSG(upperbound >= 0 && upperbound < grid.cols, uFormat("upperbound=%f grid.cols=%d x+1=%d slope=%f b=%f x=%f", upperbound, grid.cols, x+1, slope, b, x).c_str());
		}

		for(int y = lowerbound; y<=(int)upperbound; ++y)
		{
			char * v;
			if(swapped)
			{
				v = &grid.at<char>(x, y);
			}
			else
			{
				v = &grid.at<char>(y, x);
			}
			if(*v == 100 && stopOnObstacle)
			{
				return;
			}
			else
			{
				*v = 0; // free space
			}
		}
	}
}

//convert to gray scaled map
cv::Mat convertMap2Image8U(const cv::Mat & map8S)
{
	UASSERT(map8S.channels() == 1 && map8S.type() == CV_8S);
	cv::Mat map8U = cv::Mat(map8S.rows, map8S.cols, CV_8U);
	for (int i = 0; i < map8S.rows; ++i)
	{
		for (int j = 0; j < map8S.cols; ++j)
		{
			char v = map8S.at<char>(i, j);
			unsigned char gray;
			if(v == 0)
			{
				gray = 178;
			}
			else if(v == 100)
			{
				gray = 0;
			}
			else // -1
			{
				gray = 89;
			}
			map8U.at<unsigned char>(i, j) = gray;
		}
	}
	return map8U;
}

}

}
