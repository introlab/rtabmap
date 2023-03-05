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

#ifndef CORELIB_SRC_GAINCOMPENSATOR_H_
#define CORELIB_SRC_GAINCOMPENSATOR_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <opencv2/opencv.hpp>
#include <rtabmap/core/Link.h>

namespace rtabmap {

/**
 * Works like cv::GainCompensator but with point clouds
 */
class RTABMAP_CORE_EXPORT GainCompensator {
public:
	GainCompensator(double maxCorrespondenceDistance = 0.02, double minOverlap = 0.0, double alpha = 0.01, double beta = 10);
	virtual ~GainCompensator();

	void feed(
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloudA, // should not contain NaNs
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloudB, // should not contain NaNs
			const Transform & transformB);
	void feed(
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloudA,
			const pcl::IndicesPtr & indicesA,
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloudB,
			const pcl::IndicesPtr & indicesB,
			const Transform & transformB);
	void feed(
			const std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds, // should not contain NaNs
			const std::multimap<int, Link> & links);
	void feed(
			const std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds,
			const std::map<int, pcl::IndicesPtr> & indices,
			const std::multimap<int, Link> & links);
	void feed(
			const std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> & clouds,
			const std::map<int, pcl::IndicesPtr> & indices,
			const std::multimap<int, Link> & links);
	void feed(
			const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> > & clouds,
			const std::multimap<int, Link> & links);

	void apply(
			int id,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
			bool rgb = true) const;
	void apply(
			int id,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
			const pcl::IndicesPtr & indices,
			bool rgb = true) const;
	void apply(
			int id,
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
			const pcl::IndicesPtr & indices,
			bool rgb = true) const;
	void apply(
			int id,
			cv::Mat & image,
			bool rgb = true) const;

	double getGain(int id, double * r=0, double * g=0, double * b=0) const;
	int getIndex(int id) const;

private:
	cv::Mat_<double> gains_;
	std::map<int, int> idToIndex_;
	double maxCorrespondenceDistance_;
	double minOverlap_;
	double alpha_;
	double beta_;

};

} /* namespace rtabmap */

#endif /* CORELIB_SRC_GAINCOMPENSATOR_H_ */
