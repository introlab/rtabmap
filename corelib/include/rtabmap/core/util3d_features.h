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

#ifndef UTIL3D_FEATURES_H_
#define UTIL3D_FEATURES_H_

#include <rtabmap/core/rtabmap_core_export.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <list>
#include <map>

namespace rtabmap
{

namespace util3d
{


std::vector<cv::Point3f> RTABMAP_CORE_EXPORT generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		const CameraModel & cameraModel,
		float minDepth = 0,
		float maxDepth = 0);

std::vector<cv::Point3f> RTABMAP_CORE_EXPORT generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels,
		float minDepth = 0,
		float maxDepth = 0);

std::vector<cv::Point3f> RTABMAP_CORE_EXPORT generateKeypoints3DDisparity(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & disparity,
		const StereoCameraModel & stereoCameraModel,
		float minDepth = 0,
		float maxDepth = 0);

std::vector<cv::Point3f> RTABMAP_CORE_EXPORT generateKeypoints3DStereo(
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const StereoCameraModel & model,
		const std::vector<unsigned char> & mask = std::vector<unsigned char>(),
		float minDepth = 0,
		float maxDepth = 0);

std::map<int, cv::Point3f> RTABMAP_CORE_EXPORT generateWords3DMono(
		const std::map<int, cv::KeyPoint> & kpts,
		const std::map<int, cv::KeyPoint> & previousKpts,
		const CameraModel & cameraModel,
		Transform & cameraTransform,
		float ransacReprojThreshold = 3.0f,
		float ransacConfidence = 0.99f,
		const std::map<int, cv::Point3f> & refGuess3D = std::map<int, cv::Point3f>(),
		double * variance = 0,
		std::vector<int> * matchesOut = 0);

std::multimap<int, cv::KeyPoint> RTABMAP_CORE_EXPORT aggregate(
		const std::list<int> & wordIds,
		const std::vector<cv::KeyPoint> & keypoints);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_FEATURES_H_ */
