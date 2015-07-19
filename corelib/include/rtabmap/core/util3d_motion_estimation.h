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

#ifndef UTIL3D_MOTION_ESTIMATION_H_
#define UTIL3D_MOTION_ESTIMATION_H_

#include <rtabmap/core/RtabmapExp.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>

namespace rtabmap
{

namespace util3d
{

Transform estimateMotion3DTo2D(
			const std::multimap<int, pcl::PointXYZ> & words3A,
			const std::multimap<int, cv::KeyPoint> & words2B,
			const CameraModel & cameraModel,
			int minInliers = 10,
			int iterations = 100,
			double reprojError = 5.,
			int flagsPnP = 0,
			const Transform & guess = Transform::getIdentity(),
			const std::multimap<int, pcl::PointXYZ> & words3B = std::multimap<int, pcl::PointXYZ>(),
			double * varianceOut = 0,
			std::vector<int> * matchesOut = 0,
			std::vector<int> * inliersOut = 0);

Transform estimateMotion3DTo3D(
			const std::multimap<int, pcl::PointXYZ> & words3A,
			const std::multimap<int, pcl::PointXYZ> & words3B,
			int minInliers = 10,
			double inliersDistance = 0.1,
			int iterations = 100,
			int refineIterations = 5,
			double * varianceOut = 0,
			std::vector<int> * matchesOut = 0,
			std::vector<int> * inliersOut = 0);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_TRANSFORMS_H_ */
