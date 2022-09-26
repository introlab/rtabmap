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

#ifndef FRAME_PROJECTOR_H
#define FRAME_PROJECTOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <rtabmap/core/CameraModel.h>

#define MAX_MULT 1.3
#define MIN_MULT 0.7

namespace clams
{

  //! "Projective" point comes from the OpenNI terminology, and refers to (u, v, z), i.e.
  //! pixel id and depth value.  Here I've added color, too, so that this represents everything
  //! that is known about a pixel in an RBGD camera.
  class ProjectivePoint
  {
  public:
	  ProjectivePoint() :
		  u_(0),
		  v_(0),
		  z_(0.0f) {}

    int u_;
    int v_;
    float z_; // in meters
  };

  //! This is essentially a pinhole camera model for an RGBD sensor, with
  //! some extra functions added on for use during calibration.
  class RTABMAP_CORE_EXPORT FrameProjector
  {
  public:
    // For storing z values in meters.  This is not Euclidean distance.
    typedef std::vector< std::vector< std::vector<double> > > RangeIndex;

    FrameProjector(const rtabmap::CameraModel & model);

    RangeIndex cloudToRangeIndex(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd) const;
    //! transform is applied to the map, then projected into a depth index.
    //! The best depth estimate from the map corresponding to the measurement depth frame
    //! will be returned.
    cv::Mat estimateMapDepth(
    		const pcl::PointCloud<pcl::PointXYZ>::Ptr & map,
    		const rtabmap::Transform & transform,
			const cv::Mat & measurement,
			double coneRadius = 0.02,
			double coneStdevThresh = 0.03) const;
                          
    pcl::PointXYZ project(const ProjectivePoint& ppt) const;
    ProjectivePoint reproject(const pcl::PointXYZ& pt) const;

  protected:
    bool coneFit(const cv::Size& imageSize, const RangeIndex& rindex,
                 int uc, int vc, double radius, double measurement_depth,
                 double* mean, double* stdev) const;

  private:
    rtabmap::CameraModel model_;
  };

}  // namespace clams

#endif // FRAME_PROJECTOR_H
