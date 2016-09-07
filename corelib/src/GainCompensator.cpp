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

#include "rtabmap/core/GainCompensator.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/util3d_transforms.h>
#include <pcl/registration/correspondence_estimation.h>

namespace rtabmap {

double sqr(uchar v)
{
	return double(v)*double(v);
}

GainCompensator::GainCompensator(double maxCorrespondenceDistance, double minOverlap, double alpha, double beta) :
		maxCorrespondenceDistance_(maxCorrespondenceDistance),
		minOverlap_(minOverlap),
		alpha_(alpha),
		beta_(beta)
{
}

GainCompensator::~GainCompensator() {
}

void GainCompensator::feed(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloudA,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloudB,
		const Transform & transformB)
{
	std::multimap<int, Link> links;
	links.insert(std::make_pair(0, Link(0,1,Link::kUserClosure, transformB)));
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	clouds.insert(std::make_pair(0, cloudA));
	clouds.insert(std::make_pair(1, cloudB));
	feed(clouds, links);
}

void GainCompensator::feed(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloudA,
		const pcl::IndicesPtr & indicesA,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloudB,
		const pcl::IndicesPtr & indicesB,
		const Transform & transformB)
{
	std::multimap<int, Link> links;
	links.insert(std::make_pair(0, Link(0,1,Link::kUserClosure, transformB)));
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	clouds.insert(std::make_pair(0, cloudA));
	clouds.insert(std::make_pair(1, cloudB));
	std::map<int, pcl::IndicesPtr> indices;
	indices.insert(std::make_pair(0, indicesA));
	indices.insert(std::make_pair(1, indicesB));
	feed(clouds, indices, links);
}

void GainCompensator::feed(
		const std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds,
		const std::multimap<int, Link> & links)
{
	std::map<int, pcl::IndicesPtr> indices;
	feed(clouds, indices, links);
}

/**
 * @see https://github.com/opencv/opencv/blob/master/modules/stitching/src/exposure_compensate.cpp
 */
template<typename PointT>
void feedImpl(
		const std::map<int, typename pcl::PointCloud<PointT>::Ptr> & clouds,
		const std::map<int, pcl::IndicesPtr> & indices,
		const std::multimap<int, Link> & links,
		float maxCorrespondenceDistance,
		double minOverlap,
		double alpha,
		double beta,
		cv::Mat_<double> & gains,
		std::map<int, int> & idToIndex)
{
	UDEBUG("Exposure compensation...");

	UASSERT(maxCorrespondenceDistance > 0.0f);
	UASSERT(indices.size() == 0 || clouds.size() == indices.size());

	const int num_images = static_cast<int>(clouds.size());
	cv::Mat_<int> N(num_images, num_images); N.setTo(0);
	cv::Mat_<double> I(num_images, num_images); I.setTo(0);

	// make id to index map
	idToIndex.clear();
	int oi=0;
	for(typename std::map<int, typename pcl::PointCloud<PointT>::Ptr>::const_iterator iter=clouds.begin(); iter!=clouds.end(); ++iter)
	{
		idToIndex.insert(std::make_pair(iter->first, oi));
		UASSERT(indices.empty() || uContains(indices, iter->first));
		N(oi,oi) = iter->second->size();
		++oi;
	}

	typename pcl::registration::CorrespondenceEstimation<PointT, PointT>::Ptr est;
	est.reset(new pcl::registration::CorrespondenceEstimation<PointT, PointT>);

	for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
	{
		if(uContains(idToIndex, iter->second.from()) && uContains(idToIndex, iter->second.to()))
		{
			UDEBUG("estimate...%d %d", iter->second.from(), iter->second.to());
			const typename pcl::PointCloud<PointT>::Ptr & cloudFrom = clouds.at(iter->second.from());
			const typename pcl::PointCloud<PointT>::Ptr & cloudTo = clouds.at(iter->second.to());
			if(cloudFrom->size() && cloudTo->size())
			{
				est->setInputTarget(cloudFrom); //match
				if(iter->second.transform().isIdentity() || iter->second.transform().isNull())
				{
					est->setInputSource(cloudTo); //query
				}
				else
				{
					est->setInputSource(util3d::transformPointCloud(cloudTo, iter->second.transform())); //query
				}

				if(indices.size())
				{
					if(indices.at(iter->second.from())->size())
					{
						est->setIndicesTarget(indices.at(iter->second.from()));
					}
					if(indices.at(iter->second.to())->size())
					{
						est->setIndicesSource(indices.at(iter->second.to()));
					}
				}

				pcl::Correspondences correspondences;
				est->determineCorrespondences(correspondences, maxCorrespondenceDistance);
				UDEBUG("correspondences = %d", (int)correspondences.size());
				if((minOverlap <= 0.0 && correspondences.size()) ||
						(double(correspondences.size()) / double(clouds.at(iter->second.from())->size()) >= minOverlap &&
						 double(correspondences.size()) / double(clouds.at(iter->second.to())->size()) >= minOverlap))
				{
					int i = idToIndex.at(iter->second.from());
					int j = idToIndex.at(iter->second.to());
					N(i, j) = N(j, i) = correspondences.size();

					double Isum1 = 0, Isum2 = 0;
					for (unsigned int c = 0; c < correspondences.size(); ++c)
					{
						const PointT & pt1 = cloudFrom->at(correspondences.at(c).index_match);
						const PointT & pt2 = cloudTo->at(correspondences.at(c).index_query);

						Isum1 += std::sqrt(static_cast<double>(sqr(pt1.r) + sqr(pt1.g) + sqr(pt1.b)));
						Isum2 += std::sqrt(static_cast<double>(sqr(pt2.r) + sqr(pt2.g) + sqr(pt2.b)));
					}
					I(i, j) = Isum1 / N(i, j);
					I(j, i) = Isum2 / N(i, j);
				}
			}
		}
	}

	cv::Mat_<double> A(num_images, num_images); A.setTo(0);
	cv::Mat_<double> b(num_images, 1); b.setTo(0);
	for (int i = 0; i < num_images; ++i)
	{
		for (int j = 0; j < num_images; ++j)
		{
			b(i, 0) += beta * N(i, j);
			A(i, i) += beta * N(i, j);
			if (j == i) continue;
			A(i, i) += 2 * alpha * I(i, j) * I(i, j) * N(i, j);
			A(i, j) -= 2 * alpha * I(i, j) * I(j, i) * N(i, j);
		}
	}

	gains = cv::Mat_<double>();
	cv::solve(A, b, gains);
}

void GainCompensator::feed(
		const std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds,
		const std::map<int, pcl::IndicesPtr> & indices,
		const std::multimap<int, Link> & links)
{
	feedImpl<pcl::PointXYZRGB>(clouds, indices, links, maxCorrespondenceDistance_, minOverlap_, alpha_, beta_, gains_, idToIndex_);
}
void GainCompensator::feed(
		const std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> & clouds,
		const std::map<int, pcl::IndicesPtr> & indices,
		const std::multimap<int, Link> & links)
{
	feedImpl<pcl::PointXYZRGBNormal>(clouds, indices, links, maxCorrespondenceDistance_, minOverlap_, alpha_, beta_, gains_, idToIndex_);
}
void GainCompensator::feed(
		const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> > & cloudsIndices,
		const std::multimap<int, Link> & links)
{
	std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds;
	std::map<int, pcl::IndicesPtr> indices;
	for(std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> >::const_iterator iter=cloudsIndices.begin(); iter!=cloudsIndices.end(); ++iter)
	{
		clouds.insert(std::make_pair(iter->first, iter->second.first));
		indices.insert(std::make_pair(iter->first, iter->second.second));
	}
	feedImpl<pcl::PointXYZRGBNormal>(clouds, indices, links, maxCorrespondenceDistance_, minOverlap_, alpha_, beta_, gains_, idToIndex_);
}

template<typename PointT>
void applyImpl(
		int index,
		typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const cv::Mat_<double> & gains)
{
	double gain = gains(index, 0);
	UDEBUG("index=%d gain=%f", index, gain);
	if(indices->size())
	{
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			PointT & pt = cloud->at(indices->at(i));
			pt.r = uchar(std::max(0.0, std::min(255.0, double(pt.r) * gain)));
			pt.g = uchar(std::max(0.0, std::min(255.0, double(pt.g) * gain)));
			pt.b = uchar(std::max(0.0, std::min(255.0, double(pt.b) * gain)));
		}
	}
	else
	{
		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			PointT & pt = cloud->at(i);
			pt.r = uchar(std::max(0.0, std::min(255.0, double(pt.r) * gain)));
			pt.g = uchar(std::max(0.0, std::min(255.0, double(pt.g) * gain)));
			pt.b = uchar(std::max(0.0, std::min(255.0, double(pt.b) * gain)));
		}
	}
}

void GainCompensator::apply(
		int id,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
{
	pcl::IndicesPtr indices(new std::vector<int>);
	apply(id, cloud, indices);
}
void GainCompensator::apply(
		int id,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices)
{
	UASSERT_MSG(uContains(idToIndex_, id), uFormat("id=%d idToIndex_.size()=%d", id, (int)idToIndex_.size()).c_str());
	applyImpl<pcl::PointXYZRGB>(idToIndex_.at(id), cloud, indices, gains_);
}
void GainCompensator::apply(
		int id,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices)
{
	UASSERT_MSG(uContains(idToIndex_, id), uFormat("id=%d idToIndex_.size()=%d", id, (int)idToIndex_.size()).c_str());
	applyImpl<pcl::PointXYZRGBNormal>(idToIndex_.at(id), cloud, indices, gains_);
}

void GainCompensator::apply(
		int id,
		cv::Mat & image)
{
	UASSERT_MSG(uContains(idToIndex_, id), uFormat("id=%d idToIndex_.size()=%d", id, (int)idToIndex_.size()).c_str());
	cv::multiply(image, gains_(idToIndex_.at(id), 0), image);
}

double GainCompensator::getGain(int id) const
{
	UASSERT_MSG(uContains(idToIndex_, id), uFormat("id=%d idToIndex_.size()=%d", id, (int)idToIndex_.size()).c_str());
	return gains_(idToIndex_.at(id), 0);
}

int GainCompensator::getIndex(int id) const
{
	if(uContains(idToIndex_, id))
	{
		return idToIndex_.at(id);
	}
	return -1;
}

} /* namespace rtabmap */
