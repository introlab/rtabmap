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

#ifndef DISCRETE_DEPTH_DISTORTION_MODEL_H
#define DISCRETE_DEPTH_DISTORTION_MODEL_H

#include <assert.h>
#include <vector>
#include <set>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <rtabmap/utilite/UMutex.h>
#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

namespace clams
{
  class RTABMAP_CORE_EXPORT DiscreteFrustum
  {
  public:
    DiscreteFrustum(int smoothing = 1, double bin_depth = 1.0, double max_dist = 10.0);
    //! z value, not distance to origin.
    //! thread-safe.
    void addExample(double ground_truth, double measurement);
    int index(double z) const;
    void undistort(double* z) const;
    void interpolatedUndistort(double* z) const;
    void serialize(std::ostream& out, bool ascii) const;
    void deserialize(std::istream& in, bool ascii);
  
  protected:
    double max_dist_;
    int num_bins_;
    double bin_depth_;
    Eigen::VectorXf counts_;
    Eigen::VectorXf total_numerators_;
    Eigen::VectorXf total_denominators_;
    Eigen::VectorXf multipliers_;

    friend class DiscreteDepthDistortionModel;
  };

  class RTABMAP_CORE_EXPORT DiscreteDepthDistortionModel
  {
  public:
    // returns all divisors of num
    static std::set<size_t> getDivisors(const size_t &num);

    // returns divisor from divisors closest to ref
    static size_t getClosestToRef(const std::set<size_t> &divisors, const double &ref);

    // sets bin_width and bin_height to appropriate values
    static void getBinSize(const size_t &width, const size_t &height, size_t &bin_width, size_t &bin_height);

  public:
    DiscreteDepthDistortionModel() :
    	width_(0),
		height_(0),
		bin_width_(0),
		bin_height_(0),
		bin_depth_(0),
		num_bins_x_(0),
		num_bins_y_(0),
		training_samples_(0)
    {}
    virtual ~DiscreteDepthDistortionModel();
    DiscreteDepthDistortionModel(int width, int height, int bin_width = 8, int bin_height = 6, double bin_depth = 2.0, int smoothing = 1, double max_depth = 10.0);
    DiscreteDepthDistortionModel(const DiscreteDepthDistortionModel& other);
    DiscreteDepthDistortionModel& operator=(const DiscreteDepthDistortionModel& other);
    void undistort(cv::Mat & depth) const;
    //! Returns the number of training examples it used from this pair.
    //! Thread-safe.
    size_t accumulate(const cv::Mat& ground_truth, const cv::Mat& measurement);
    void addExample(int v, int u, double ground_truth, double measurement);
    void save(const std::string& path) const;
	void load(const std::string& path);
	void serialize(std::ostream& out, bool ascii) const;
	void deserialize(std::istream& in, bool ascii);
    cv::Mat visualize(const std::string& path = "") const;

    int getWidth() const {return width_;}
    int getHeight() const {return height_;}
    size_t getTrainingSamples() const {return training_samples_;}
    bool isValid() const
    {
    	return !frustums_.empty();
    }

  protected:
    //! Image width.
    int width_;
    //! Image height.
    int height_;
    //! Width of each bin in pixels.
    int bin_width_;
    //! Height of each bin in pixels.
    int bin_height_;
    //! Depth of each bin in meters.
    double bin_depth_;
    int num_bins_x_;
    int num_bins_y_;
    //! frustums_[y][x]
    std::vector< std::vector<DiscreteFrustum*> > frustums_;

    size_t training_samples_;

    void deleteFrustums();
    DiscreteFrustum& frustum(int y, int x);
    const DiscreteFrustum& frustum(int y, int x) const;

    UMutex mutex_;
  };

}  // namespace clams

#endif // DISCRETE_DEPTH_DISTORTION_MODEL_H
