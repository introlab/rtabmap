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

#ifndef UTIL3D_FEATURES_H_
#define UTIL3D_FEATURES_H_

#include <rtabmap/core/RtabmapExp.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <list>
#include <map>

namespace rtabmap
{

namespace util3d
{


pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		const CameraModel & cameraModel);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP generateKeypoints3DDisparity(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & disparity,
		const StereoCameraModel & stereoCameraMode);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP generateKeypoints3DStereo(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		float fx,
		float baseline,
		float cx,
		float cy,
		Transform localTransform = Transform::getIdentity(),
		int flowWinSize = 9,
		int flowMaxLevel = 4,
		int flowIterations = 20,
		double flowEps = 0.02,
		double maxCorrespondencesSlope = 0.0);
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP generateKeypoints3DStereo(
		const std::vector<cv::Point2f> & leftCorners,
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		float fx,
		float baseline,
		float cx,
		float cy,
		Transform localTransform = Transform::getIdentity(),
		int flowWinSize = 9,
		int flowMaxLevel = 4,
		int flowIterations = 20,
		double flowEps = 0.02,
		double maxCorrespondencesSlope = 0.0);

std::multimap<int, pcl::PointXYZ> RTABMAP_EXP generateWords3DMono(
		const std::multimap<int, cv::KeyPoint> & kpts,
		const std::multimap<int, cv::KeyPoint> & previousKpts,
		const CameraModel & cameraModel,
		Transform & cameraTransform,
		int pnpIterations = 100,
		float pnpReprojError = 8.0f,
		int pnpFlags = 0, // cv::SOLVEPNP_ITERATIVE
		float ransacParam1 = 3.0f,
		float ransacParam2 = 0.99f,
		const std::multimap<int, pcl::PointXYZ> & refGuess3D = std::multimap<int, pcl::PointXYZ>(),
		double * variance = 0);

std::multimap<int, cv::KeyPoint> RTABMAP_EXP aggregate(
		const std::list<int> & wordIds,
		const std::vector<cv::KeyPoint> & keypoints);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_FEATURES_H_ */
