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

#include "rtabmap/core/clams/frame_projector.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace Eigen;

namespace clams
{
  FrameProjector::FrameProjector(const rtabmap::CameraModel & model) :
    model_(model)
  {
	  UASSERT(model.isValidForReprojection());
  }

  // pcd is in /map frame
  FrameProjector::RangeIndex FrameProjector::cloudToRangeIndex(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd) const
  {
	  int height = model_.imageHeight();
	  int width = model_.imageWidth();
    RangeIndex ind;
    if((int)ind.size() != height)
      ind.resize(height);
    for(size_t y = 0; y < ind.size(); ++y)
      if((int)ind[y].size() != width)
        ind[y].resize(width);
    for(size_t y = 0; y < ind.size(); ++y) {
      for(size_t x = 0; x < ind[y].size(); ++x) { 
        ind[y][x].clear();
        ind[y][x].reserve(10);
      }
    }

    rtabmap::Transform t = model_.localTransform().inverse();

    ProjectivePoint ppt;
    for(size_t i = 0; i < pcd->size(); ++i) {
      if(!isFinite(pcd->at(i)))
        continue;
      ppt = reproject(rtabmap::util3d::transformPoint(pcd->at(i), t));
      if(ppt.z_ == 0 || !(ppt.u_ >= 0 && ppt.v_ >= 0 && ppt.u_ < width && ppt.v_ < height))
        continue;
      ind[ppt.v_][ppt.u_].push_back(ppt.z_);
    }
    return ind;
  }

  pcl::PointXYZ FrameProjector::project(const ProjectivePoint& ppt) const
  {
	 UASSERT(ppt.u_ >= 0 && ppt.v_ >= 0 && ppt.u_ < model_.imageWidth() && ppt.v_ < model_.imageHeight());

	 pcl::PointXYZ pt;

	 model_.project(ppt.u_, ppt.v_, ppt.z_, pt.x, pt.y, pt.z);
    return pt;
  }
  
  ProjectivePoint FrameProjector::reproject(const pcl::PointXYZ & pt) const
  {
	 UASSERT(isFinite(pt));

	 ProjectivePoint ppt;

	 if(pt.z > 0)
	 {
		 model_.reproject(pt.x, pt.y, pt.z, ppt.u_, ppt.v_);
		 ppt.z_ = pt.z;
	 }

    return ppt;
  }
  
  cv::Mat FrameProjector::estimateMapDepth(
		  const pcl::PointCloud<pcl::PointXYZ>::Ptr & map,
		  const rtabmap::Transform & transform,
          const cv::Mat& measurement,
		  double coneRadius,
		  double coneStdevThresh) const
  {
	cv::Mat estimate = cv::Mat::zeros(measurement.size(), CV_32FC1);

    // -- Get the depth index.
     pcl::PointCloud<pcl::PointXYZ>::Ptr transformed = rtabmap::util3d::transformPointCloud(map, transform);
    RangeIndex rindex = cloudToRangeIndex(transformed);

    // -- Compute the edge-of-map mask.
    cv::Mat naive_mapframe = rtabmap::util3d::projectCloudToCamera(model_.imageSize(), model_.K(), transformed, model_.localTransform());
    const cv::Mat& measurement_depth = measurement;
    const cv::Mat& naive_mapdepth = naive_mapframe;
    cv::Mat1b mask(measurement_depth.rows, measurement_depth.cols);
    mask = 0;
    for(int y = 0; y < mask.rows; ++y)
      for(int x = 0; x < mask.cols; ++x)
        if(naive_mapdepth.at<float>(y, x) != 0)
          mask(y, x) = 255;

    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 4);
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 15);

    bool isInMM = measurement_depth.type() == CV_16UC1;

    // -- Main loop: for all points in the image...
    ProjectivePoint ppt;
    for(ppt.v_ = 0; ppt.v_ < measurement_depth.rows; ++ppt.v_) {
      for(ppt.u_ = 0; ppt.u_ < measurement_depth.cols; ++ppt.u_) {

    	  float value = isInMM?float(measurement_depth.at<unsigned short>(ppt.v_, ppt.u_))*0.001f:measurement_depth.at<float>(ppt.v_, ppt.u_);

        // -- Reject points with no data.
        if(value == 0)
          continue;
        if(naive_mapdepth.at<float>(ppt.v_, ppt.u_) == 0)
          continue;
        
        // -- Reject points on the edge of the map.
        if(mask(ppt.v_, ppt.u_) == 0)
          continue;

        // -- Find nearby points in the cone to get a good estimate of the map depth.
        double mean = 0;
        double stdev = 0;
        //double stdev_thresh = numeric_limits<double>::max();
        bool valid = coneFit(
        		naive_mapdepth.size(),
        		rindex,
				ppt.u_,
				ppt.v_,
				coneRadius,
				value,
				&mean,
				&stdev);
        if(!valid)
          continue;
        if(stdev > coneStdevThresh)
          continue;

        estimate.at<float>(ppt.v_, ppt.u_) = mean;
      }
    }
    return estimate;
  }

  bool FrameProjector::coneFit(const cv::Size& imageSize, const RangeIndex& rindex,
                                int uc, int vc, double radius, double measurement_depth,
                                double* mean, double* stdev) const
  {
    pcl::PointXYZ pt_center, pt_ul, pt_lr;
    ProjectivePoint ppt, ppt_ul, ppt_lr;
    ppt.u_ = uc;
    ppt.v_ = vc;
    ppt.z_ = measurement_depth;
    pt_center = project(ppt);

    pt_ul = pt_center;
    pt_lr = pt_center;
    pt_ul.x -= radius;
    pt_ul.y -= radius;
    pt_lr.x += radius;
    pt_lr.y += radius;

    if(!pcl::isFinite(pt_ul) || !pcl::isFinite(pt_lr))
    {
    	return false;
    }

    ppt_ul = reproject(pt_ul);
    ppt_lr = reproject(pt_lr);
    if(ppt_ul.z_ == 0 || !(ppt_ul.u_ >= 0 && ppt_ul.v_ >= 0 && ppt_ul.u_ < imageSize.width && ppt_ul.v_ < imageSize.height))
      return false;
    if(ppt_lr.z_ == 0 || !(ppt_lr.u_ >= 0 && ppt_lr.v_ >= 0 && ppt_lr.u_ < imageSize.width && ppt_lr.v_ < imageSize.height))
      return false;

    int min_u = ppt_ul.u_;
    int max_u = ppt_lr.u_;
    int min_v = ppt_ul.v_;
    int max_v = ppt_lr.v_;

    *mean = 0;
    double num = 0;
    for(ppt.u_ = min_u; ppt.u_ <= max_u; ++ppt.u_) {
      for(ppt.v_ = min_v; ppt.v_ <= max_v; ++ppt.v_) {
        const vector<double>& vals = rindex[ppt.v_][ppt.u_];
        for(size_t i = 0; i < vals.size(); ++i) {
          double mult = vals[i] / measurement_depth;
          if(mult > MIN_MULT && mult < MAX_MULT) {
            *mean += vals[i];
            ++num;
          }
        }
      }
    }
    if(num == 0)
      return false;
    *mean /= num;

    double var = 0;
    for(ppt.u_ = min_u; ppt.u_ <= max_u; ++ppt.u_) {
      for(ppt.v_ = min_v; ppt.v_ <= max_v; ++ppt.v_) {
        const vector<double>& vals = rindex[ppt.v_][ppt.u_];
        for(size_t i = 0; i < vals.size(); ++i) {
          double mult = vals[i] / measurement_depth;
          if(mult > MIN_MULT && mult < MAX_MULT)
            var += (vals[i] - *mean) * (vals[i] - *mean);
        }
      }
    }
    var /= num;
    
    *stdev = sqrt(var);
    return true;
  }
  
}  // namespace clams

