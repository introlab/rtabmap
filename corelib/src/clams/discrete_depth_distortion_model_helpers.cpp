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

#include <rtabmap/core/clams/discrete_depth_distortion_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>

using namespace std;
using namespace Eigen;

namespace clams
{

  cv::Mat DiscreteDepthDistortionModel::visualize(const std::string& dir) const
  {
	  bool writeFiles = !dir.empty() && UDirectory::exists(dir);

    const DiscreteFrustum& reference_frustum = *frustums_[0][0];
    int num_layers = reference_frustum.num_bins_;

    // -- Set up for combined imagery.
    int horiz_divider = 10;
    int vert_divider = 20;
    cv::Mat3b overview(cv::Size(width_ * 2 + horiz_divider, height_ * num_layers + vert_divider * (num_layers + 2)), cv::Vec3b(0, 0, 0));
    vector<int> pub_layers;
    for(int i = 0; i < num_layers; ++i)
      pub_layers.push_back(i);
    // pub_layers.push_back(1);
    // pub_layers.push_back(2);
    // pub_layers.push_back(3);
    cv::Mat3b pub(cv::Size(width_, height_ * pub_layers.size() + vert_divider * (pub_layers.size() + 2)), cv::Vec3b(255, 255, 255));
  
    for(int i = 0; i < num_layers; ++i) {
      // -- Determine the path to save the image for this layer.
      char buffer[50];
      float mindepth = reference_frustum.bin_depth_ * i;
      float maxdepth = reference_frustum.bin_depth_ * (i + 1);
      sprintf(buffer, "%05.2f-%05.2f", mindepth, maxdepth);
      ostringstream oss;
      oss << dir << "/multipliers_" << buffer << ".png";

      // -- Compute the multipliers visualization for this layer.
      //    Multiplier of 1 is black, >1 is red, <1 is blue.  Think redshift.
      cv::Mat3b mult(cv::Size(width_, height_), cv::Vec3b(0, 0, 0));
      for(int y = 0; y < mult.rows; ++y) {
        for(int x = 0; x < mult.cols; ++x) {
          const DiscreteFrustum& frustum = *frustums_[y / bin_height_][x / bin_width_];
          float val = frustum.multipliers_(i);
          if(val > 1)
            mult(y, x)[2] = min(255., 255 * (val - 1.0) / 0.25);
          if(val < 1)
            mult(y, x)[0] = min(255., 255 * (1.0 - val) / 0.25);
        }
      }
      if(writeFiles)
	  {
		  cv::imwrite(oss.str(), mult);
		  UINFO("Written \"%s\"", oss.str().c_str());
	  }

      // -- Compute the counts visualization for this layer.
      //    0 is black, 100 is white.
      cv::Mat3b count(cv::Size(width_, height_), cv::Vec3b(0, 0, 0));
      for(int y = 0; y < count.rows; ++y) {
        for(int x = 0; x < count.cols; ++x) {
          const DiscreteFrustum& frustum = *frustums_[y / bin_height_][x / bin_width_];
          uchar val = min(255., (double)(255 * frustum.counts_(i) / 100));
          count(y, x)[0] = val;
          count(y, x)[1] = val;
          count(y, x)[2] = val;
        }
      }
      oss.str("");
      oss << dir << "/counts_" << buffer << ".png";
      if(writeFiles)
	  {
		  cv::imwrite(oss.str(), count);
		  UINFO("Written \"%s\"", oss.str().c_str());
	  }

      // -- Make images showing the two, side-by-side.
      cv::Mat3b combined(cv::Size(width_ * 2 + horiz_divider, height_), cv::Vec3b(0, 0, 0));
      for(int y = 0; y < combined.rows; ++y) {
        for(int x = 0; x < combined.cols; ++x) {
          if(x < count.cols)
            combined(y, x) = count(y, x);
          else if(x > count.cols + horiz_divider)
            combined(y, x) = mult(y, x - count.cols - horiz_divider);
        }
      }
      oss.str("");
      oss << dir << "/combined_" << buffer << ".png";
      if(writeFiles)
	  {
		  cv::imwrite(oss.str(), combined);
		  UINFO("Written \"%s\"", oss.str().c_str());
	  }

      // -- Append to the overview image.
      for(int y = 0; y < combined.rows; ++y)
        for(int x = 0; x < combined.cols; ++x)
          overview(y + i * (combined.rows + vert_divider) + vert_divider, x) = combined(y, x);

      // -- Compute the publication multipliers visualization for this layer.
      //    Multiplier of 1 is white, >1 is red, <1 is blue.  Think redshift.
      cv::Mat3b pubmult(cv::Size(width_, height_), cv::Vec3b(255, 255, 255));
      for(int y = 0; y < pubmult.rows; ++y) {
        for(int x = 0; x < pubmult.cols; ++x) {
          const DiscreteFrustum& frustum = *frustums_[y / bin_height_][x / bin_width_];
          float val = frustum.multipliers_(i);
          if(val > 1) {
            pubmult(y, x)[0] = 255 - min(255., 255 * (val - 1.0) / 0.1);
            pubmult(y, x)[1] = 255 - min(255., 255 * (val - 1.0) / 0.1);
          }
          if(val < 1) {
            pubmult(y, x)[1] = 255 - min(255., 255 * (1.0 - val) / 0.1);
            pubmult(y, x)[2] = 255 - min(255., 255 * (1.0 - val) / 0.1);
          }
        }
      }
  
      // -- Append to publication image.
      for(size_t j = 0; j < pub_layers.size(); ++j)
        if(pub_layers[j] == i)
          for(int y = 0; y < pubmult.rows; ++y)
            for(int x = 0; x < pubmult.cols; ++x)
              pub(y + j * (pubmult.rows + vert_divider) + vert_divider, x) = pubmult(y, x);
    }
  
    // -- Add a white bar at the top and bottom for reference.
    for(int y = 0; y < overview.rows; ++y)
      if(y < vert_divider || y > overview.rows - vert_divider)
        for(int x = 0; x < overview.cols; ++x)
          overview(y, x) = cv::Vec3b(255, 255, 255);
  
    // -- Save overview image.
    ostringstream oss;
    oss << dir << "/overview.png";
    if(writeFiles)
	  {
		  cv::imwrite(oss.str(), overview);
		  UINFO("Written \"%s\"", oss.str().c_str());
	  }

    // -- Save a small version for easy loading.
    cv::Mat3b overview_scaled;
    cv::resize(overview, overview_scaled, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);
    oss.str("");
    oss << dir << "/overview_scaled.png";
    if(writeFiles)
	  {
		  cv::imwrite(oss.str(), overview_scaled);
		  UINFO("Written \"%s\"", oss.str().c_str());
	  }

    // -- Save publication image.
    oss.str("");
    oss << dir << "/pub";
    // for(size_t i = 0; i < pub_layers.size(); ++i)
    //   oss << "-" << setw(2) << setfill('0') << pub_layers[i];
    oss << ".png";
    if(writeFiles)
	  {
		  cv::imwrite(oss.str(), pub);
		  UINFO("Written \"%s\"", oss.str().c_str());
	  }

    UASSERT(overview.rows == pub.rows);
    cv::Mat3b targetImage(overview.rows, overview.cols/2 + pub.cols);

	cv::Mat roiA(targetImage, cv::Rect( 0, 0, overview.cols/2, overview.rows ));
	cv::Mat(overview, cv::Rect( 0, 0, overview.cols/2, overview.rows )).copyTo(roiA);
	cv::Mat roiB( targetImage, cv::Rect( overview.cols/2, 0, pub.cols, pub.rows ) );
	pub.copyTo(roiB);

    return targetImage;
  }

}  // namespace clams
