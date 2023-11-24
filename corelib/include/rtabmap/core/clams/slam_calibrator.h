/*
Copyright (c) 2013, Alex Teichman and Stephen Miller (Stanford University)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

RTAB-Map integration: Mathieu Labbe
*/

#ifndef SLAM_CALIBRATOR_H
#define SLAM_CALIBRATOR_H

#include <rtabmap/core/clams/discrete_depth_distortion_model.h>
#include <rtabmap/core/SensorData.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace clams
{
  
DiscreteDepthDistortionModel RTABMAP_CORE_EXPORT calibrate(
		const std::map<int, rtabmap::SensorData> & sequence,
		const std::map<int, rtabmap::Transform> & trajectory,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & map,
		double coneRadius = 0.02,
		double coneStdevThresh = 0.03);

}

#endif // SLAM_CALIBRATOR_H
