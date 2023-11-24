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

#ifndef UTIL3D_CORRESPONDENCES_H_
#define UTIL3D_CORRESPONDENCES_H_

#include <rtabmap/core/rtabmap_core_export.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/features2d/features2d.hpp>
#include <set>
#include <map>
#include <list>

namespace rtabmap
{

namespace util3d
{


void RTABMAP_CORE_EXPORT findCorrespondences(
		const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<cv::Point2f, cv::Point2f> > & pairs);

void RTABMAP_CORE_EXPORT findCorrespondences(
		const std::multimap<int, cv::Point3f> & words1,
		const std::multimap<int, cv::Point3f> & words2,
		std::vector<cv::Point3f> & inliers1,
		std::vector<cv::Point3f> & inliers2,
		float maxDepth,
		std::vector<int> * uniqueCorrespondences = 0);

void RTABMAP_CORE_EXPORT findCorrespondences(
		const std::map<int, cv::Point3f> & words1,
		const std::map<int, cv::Point3f> & words2,
		std::vector<cv::Point3f> & inliers1,
		std::vector<cv::Point3f> & inliers2,
		float maxDepth,
		std::vector<int> * correspondences = 0);

// remove depth by z axis
void RTABMAP_CORE_EXPORT extractXYZCorrespondences(const std::multimap<int, pcl::PointXYZ> & words1,
									  const std::multimap<int, pcl::PointXYZ> & words2,
									  pcl::PointCloud<pcl::PointXYZ> & cloud1,
									  pcl::PointCloud<pcl::PointXYZ> & cloud2);

void RTABMAP_CORE_EXPORT extractXYZCorrespondencesRANSAC(const std::multimap<int, pcl::PointXYZ> & words1,
									  const std::multimap<int, pcl::PointXYZ> & words2,
									  pcl::PointCloud<pcl::PointXYZ> & cloud1,
									  pcl::PointCloud<pcl::PointXYZ> & cloud2);

void RTABMAP_CORE_EXPORT extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
									   const cv::Mat & depthImage1,
									   const cv::Mat & depthImage2,
									   float cx, float cy,
									   float fx, float fy,
									   float maxDepth,
									   pcl::PointCloud<pcl::PointXYZ> & cloud1,
									   pcl::PointCloud<pcl::PointXYZ> & cloud2);

void RTABMAP_CORE_EXPORT extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<pcl::PointXYZ> & cloud1,
							   const pcl::PointCloud<pcl::PointXYZ> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2,
							   char depthAxis);
void RTABMAP_CORE_EXPORT extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<pcl::PointXYZRGB> & cloud1,
							   const pcl::PointCloud<pcl::PointXYZRGB> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2,
							   char depthAxis);

int RTABMAP_CORE_EXPORT countUniquePairs(const std::multimap<int, pcl::PointXYZ> & wordsA,
					 const std::multimap<int, pcl::PointXYZ> & wordsB);

void RTABMAP_CORE_EXPORT filterMaxDepth(pcl::PointCloud<pcl::PointXYZ> & inliers1,
					pcl::PointCloud<pcl::PointXYZ> & inliers2,
					float maxDepth,
					char depthAxis,
					bool removeDuplicates);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_CORRESPONDENCES_H_ */
