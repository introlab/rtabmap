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

#include "rtabmap/core/util3d_correspondences.h"
#include "rtabmap/core/util3d.h"

#include <rtabmap/utilite/UStl.h>
#include <rtabmap/core/EpipolarGeometry.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <pcl/search/kdtree.h>

namespace rtabmap
{

namespace util3d
{


void extractXYZCorrespondences(const std::multimap<int, pcl::PointXYZ> & words1,
									  const std::multimap<int, pcl::PointXYZ> & words2,
									  pcl::PointCloud<pcl::PointXYZ> & cloud1,
									  pcl::PointCloud<pcl::PointXYZ> & cloud2)
{
	const std::list<int> & ids = uUniqueKeys(words1);
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(words1.count(*i) == 1 && words2.count(*i) == 1)
		{
			const pcl::PointXYZ & pt1 = words1.find(*i)->second;
			const pcl::PointXYZ & pt2 = words2.find(*i)->second;
			if(pcl::isFinite(pt1) && pcl::isFinite(pt2))
			{
				cloud1.push_back(pt1);
				cloud2.push_back(pt2);
			}
		}
	}
}

void extractXYZCorrespondencesRANSAC(const std::multimap<int, pcl::PointXYZ> & words1,
									  const std::multimap<int, pcl::PointXYZ> & words2,
									  pcl::PointCloud<pcl::PointXYZ> & cloud1,
									  pcl::PointCloud<pcl::PointXYZ> & cloud2)
{
	std::list<std::pair<pcl::PointXYZ, pcl::PointXYZ> > pairs;
	const std::list<int> & ids = uUniqueKeys(words1);
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(words1.count(*i) == 1 && words2.count(*i) == 1)
		{
			const pcl::PointXYZ & pt1 = words1.find(*i)->second;
			const pcl::PointXYZ & pt2 = words2.find(*i)->second;
			if(pcl::isFinite(pt1) && pcl::isFinite(pt2))
			{
				pairs.push_back(std::pair<pcl::PointXYZ, pcl::PointXYZ>(pt1, pt2));
			}
		}
	}

	if(pairs.size() > 7)
	{
		// Remove outliers using fundamental matrix RANSAC
		std::vector<uchar> status(pairs.size(), 0);
		//Convert Keypoints to a structure that OpenCV understands
		//3 dimensions (Homogeneous vectors)
		cv::Mat points1(1, (int)pairs.size(), CV_32FC2);
		cv::Mat points2(1, (int)pairs.size(), CV_32FC2);

		float * points1data = points1.ptr<float>(0);
		float * points2data = points2.ptr<float>(0);

		// Fill the points here ...
		int i=0;
		for(std::list<std::pair<pcl::PointXYZ, pcl::PointXYZ> >::const_iterator iter = pairs.begin();
			iter != pairs.end();
			++iter )
		{
			points1data[i*2] = (*iter).first.x;
			points1data[i*2+1] = (*iter).first.y;

			points2data[i*2] = (*iter).second.x;
			points2data[i*2+1] = (*iter).second.y;

			++i;
		}

		// Find the fundamental matrix
		cv::Mat fundamentalMatrix = cv::findFundamentalMat(
					points1,
					points2,
					status,
					cv::FM_RANSAC,
					3.0,
					0.99);

		if(!fundamentalMatrix.empty())
		{
			int i = 0;
			for(std::list<std::pair<pcl::PointXYZ, pcl::PointXYZ> >::iterator iter=pairs.begin(); iter!=pairs.end(); ++iter)
			{
				if(status[i])
				{
					cloud1.push_back(iter->first);
					cloud2.push_back(iter->second);
				}
				++i;
			}
		}
	}
}

void extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
									   const cv::Mat & depthImage1,
									   const cv::Mat & depthImage2,
									   float cx, float cy,
									   float fx, float fy,
									   float maxDepth,
									   pcl::PointCloud<pcl::PointXYZ> & cloud1,
									   pcl::PointCloud<pcl::PointXYZ> & cloud2)
{
	cloud1.resize(correspondences.size());
	cloud2.resize(correspondences.size());
	int oi=0;
	for(std::list<std::pair<cv::Point2f, cv::Point2f> >::const_iterator iter = correspondences.begin();
		iter!=correspondences.end();
		++iter)
	{
		pcl::PointXYZ pt1 = projectDepthTo3D(depthImage1, iter->first.x, iter->first.y, cx, cy, fx, fy, true);
		pcl::PointXYZ pt2 = projectDepthTo3D(depthImage2, iter->second.x, iter->second.y, cx, cy, fx, fy, true);
		if(pcl::isFinite(pt1) && pcl::isFinite(pt2) &&
		   (maxDepth <= 0 || (pt1.z <= maxDepth && pt2.z<=maxDepth)))
		{
			cloud1[oi] = pt1;
			cloud2[oi] = pt2;
			++oi;
		}
	}
	cloud1.resize(oi);
	cloud2.resize(oi);
}

template<typename PointT>
inline void extractXYZCorrespondencesImpl(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<PointT> & cloud1,
							   const pcl::PointCloud<PointT> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2,
							   char depthAxis)
{
	for(std::list<std::pair<cv::Point2f, cv::Point2f> >::const_iterator iter = correspondences.begin();
		iter!=correspondences.end();
		++iter)
	{
		PointT pt1 = cloud1.at(int(iter->first.y+0.5f) * cloud1.width + int(iter->first.x+0.5f));
		PointT pt2 = cloud2.at(int(iter->second.y+0.5f) * cloud2.width + int(iter->second.x+0.5f));
		if(pcl::isFinite(pt1) &&
		   pcl::isFinite(pt2))
		{
			inliers1.push_back(pcl::PointXYZ(pt1.x, pt1.y, pt1.z));
			inliers2.push_back(pcl::PointXYZ(pt2.x, pt2.y, pt2.z));
		}
	}
}

void extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<pcl::PointXYZ> & cloud1,
							   const pcl::PointCloud<pcl::PointXYZ> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2,
							   char depthAxis)
{
	extractXYZCorrespondencesImpl(correspondences, cloud1, cloud2, inliers1, inliers2, depthAxis);
}
void extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<pcl::PointXYZRGB> & cloud1,
							   const pcl::PointCloud<pcl::PointXYZRGB> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2,
							   char depthAxis)
{
	extractXYZCorrespondencesImpl(correspondences, cloud1, cloud2, inliers1, inliers2, depthAxis);
}

int countUniquePairs(const std::multimap<int, pcl::PointXYZ> & wordsA,
					 const std::multimap<int, pcl::PointXYZ> & wordsB)
{
	const std::list<int> & ids = uUniqueKeys(wordsA);
	int pairs = 0;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		std::list<pcl::PointXYZ> ptsA = uValues(wordsA, *i);
		std::list<pcl::PointXYZ> ptsB = uValues(wordsB, *i);
		if(ptsA.size() == 1 && ptsB.size() == 1)
		{
			++pairs;
		}
	}
	return pairs;
}

void filterMaxDepth(pcl::PointCloud<pcl::PointXYZ> & inliers1,
					pcl::PointCloud<pcl::PointXYZ> & inliers2,
					float maxDepth,
					char depthAxis,
					bool removeDuplicates)
{
	std::list<pcl::PointXYZ> addedPts;
	if(maxDepth > 0.0f && inliers1.size() && inliers1.size() == inliers2.size())
	{
		pcl::PointCloud<pcl::PointXYZ> tmpInliers1;
		pcl::PointCloud<pcl::PointXYZ> tmpInliers2;
		for(unsigned int i=0; i<inliers1.size(); ++i)
		{
			if((depthAxis == 'x' && inliers1.at(i).x < maxDepth && inliers2.at(i).x < maxDepth) ||
			   (depthAxis == 'y' && inliers1.at(i).y < maxDepth && inliers2.at(i).y < maxDepth) ||
			   (depthAxis == 'z' && inliers1.at(i).z < maxDepth && inliers2.at(i).z < maxDepth))
			{
				bool dup = false;
				if(removeDuplicates)
				{
					for(std::list<pcl::PointXYZ>::iterator iter = addedPts.begin(); iter!=addedPts.end(); ++iter)
					{
						if(iter->x == inliers1.at(i).x &&
						   iter->y == inliers1.at(i).y &&
						   iter->z == inliers1.at(i).z)
						{
							dup = true;
						}
					}
					if(!dup)
					{
						addedPts.push_back(inliers1.at(i));
					}
				}

				if(!dup)
				{
					tmpInliers1.push_back(inliers1.at(i));
					tmpInliers2.push_back(inliers2.at(i));
				}
			}
		}
		inliers1 = tmpInliers1;
		inliers2 = tmpInliers2;
	}
}


// a kdtree is constructed with cloud_target, then nearest neighbor
// is computed for each cloud_source points.
int getCorrespondencesCount(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_source,
							const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_target,
							float maxDistance)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdTree->setInputCloud(cloud_target);
	int count = 0;
	float sqrdMaxDistance = maxDistance * maxDistance;
	for(unsigned int i=0; i<cloud_source->size(); ++i)
	{
		std::vector<int> ind(1);
		std::vector<float> dist(1);
		if(kdTree->nearestKSearch(cloud_source->at(i), 1, ind, dist) && dist[0] < sqrdMaxDistance)
		{
			++count;
		}
	}
	return count;
}

/**
 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(2,2) (4,4)]
 * realPairsCount = 5
 */
void findCorrespondences(
		const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<cv::Point2f, cv::Point2f> > & pairs)
{
	const std::list<int> & ids = uUniqueKeys(wordsA);
	pairs.clear();
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		std::list<cv::KeyPoint> ptsA = uValues(wordsA, *i);
		std::list<cv::KeyPoint> ptsB = uValues(wordsB, *i);
		if(ptsA.size() == 1 && ptsB.size() == 1)
		{
			pairs.push_back(std::pair<cv::Point2f, cv::Point2f>(ptsA.front().pt, ptsB.front().pt));
		}
	}
}

void findCorrespondences(
		const std::multimap<int, cv::Point3f> & words1,
		const std::multimap<int, cv::Point3f> & words2,
		std::vector<cv::Point3f> & inliers1,
		std::vector<cv::Point3f> & inliers2,
		float maxDepth,
		std::vector<int> * uniqueCorrespondences)
{
	std::list<int> ids = uUniqueKeys(words1);
	// Find pairs
	inliers1.resize(ids.size());
	inliers2.resize(ids.size());
	if(uniqueCorrespondences)
	{
		uniqueCorrespondences->resize(ids.size());
	}

	int oi=0;
	for(std::list<int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
	{
		if(words1.count(*iter) == 1 && words2.count(*iter) == 1)
		{
			inliers1[oi] = words1.find(*iter)->second;
			inliers2[oi] = words2.find(*iter)->second;
			if(util3d::isFinite(inliers1[oi]) &&
			   util3d::isFinite(inliers2[oi]) &&
			   (inliers1[oi].x != 0 || inliers1[oi].y != 0 || inliers1[oi].z != 0) &&
			   (inliers2[oi].x != 0 || inliers2[oi].y != 0 || inliers2[oi].z != 0) &&
			   (maxDepth <= 0 || (inliers1[oi].x > 0 && inliers1[oi].x <= maxDepth && inliers2[oi].x>0 &&inliers2[oi].x<=maxDepth)))
			{
				if(uniqueCorrespondences)
				{
					uniqueCorrespondences->at(oi) = *iter;
				}
				++oi;
			}
		}
	}
	inliers1.resize(oi);
	inliers2.resize(oi);
	if(uniqueCorrespondences)
	{
		uniqueCorrespondences->resize(oi);
	}
}

void findCorrespondences(
		const std::map<int, cv::Point3f> & words1,
		const std::map<int, cv::Point3f> & words2,
		std::vector<cv::Point3f> & inliers1,
		std::vector<cv::Point3f> & inliers2,
		float maxDepth,
		std::vector<int> * correspondences)
{
	std::vector<int> ids = uKeys(words1);
	// Find pairs
	inliers1.resize(ids.size());
	inliers2.resize(ids.size());
	if(correspondences)
	{
		correspondences->resize(ids.size());
	}

	int oi=0;
	for(std::vector<int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
	{
		if(words2.find(*iter) != words2.end())
		{
			inliers1[oi] = words1.find(*iter)->second;
			inliers2[oi] = words2.find(*iter)->second;
			if(util3d::isFinite(inliers1[oi]) &&
			   util3d::isFinite(inliers2[oi]) &&
			   (inliers1[oi].x != 0 || inliers1[oi].y != 0 || inliers1[oi].z != 0) &&
			   (inliers2[oi].x != 0 || inliers2[oi].y != 0 || inliers2[oi].z != 0) &&
			   (maxDepth <= 0 || (inliers1[oi].x > 0 && inliers1[oi].x <= maxDepth && inliers2[oi].x>0 &&inliers2[oi].x<=maxDepth)))
			{
				if(correspondences)
				{
					correspondences->at(oi) = *iter;
				}
				++oi;
			}
		}
	}
	inliers1.resize(oi);
	inliers2.resize(oi);
	if(correspondences)
	{
		correspondences->resize(oi);
	}
}

}

}
