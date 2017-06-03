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
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>

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

// @see https://studiofreya.com/3d-math-and-physics/simple-aabb-vs-aabb-collision-detection/
struct AABB
{
    AABB() : c(), r() {}

    AABB(const Eigen::Vector3f & center, const Eigen::Vector3f & halfwidths)
        : c(center)
        , r(halfwidths)
    {}

    Eigen::Vector3f c;        // center point
    Eigen::Vector3f r;        // halfwidths
};

bool testAABBAABB(const AABB &a, const AABB &b)
{
    if ( fabs(a.c[0] - b.c[0]) > (a.r[0] + b.r[0]) ) return false;
    if ( fabs(a.c[1] - b.c[1]) > (a.r[1] + b.r[1]) ) return false;
    if ( fabs(a.c[2] - b.c[2]) > (a.r[2] + b.r[2]) ) return false;

    // We have an overlap
    return true;
};

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

	cv::Mat_<double> IR(num_images, num_images); IR.setTo(0);
	cv::Mat_<double> IG(num_images, num_images); IG.setTo(0);
	cv::Mat_<double> IB(num_images, num_images); IB.setTo(0);

	// make id to index map
	idToIndex.clear();
	std::vector<int> indexToId(clouds.size());
	int oi=0;
	std::map<int, std::pair<pcl::PointXYZ, pcl::PointXYZ> > boundingBoxes;
	for(typename std::map<int, typename pcl::PointCloud<PointT>::Ptr>::const_iterator iter=clouds.begin(); iter!=clouds.end(); ++iter)
	{
		idToIndex.insert(std::make_pair(iter->first, oi));
		indexToId[oi] = iter->first;
		UASSERT(indices.empty() || uContains(indices, iter->first));
		Eigen::Vector4f minPt(0,0,0,0);
		Eigen::Vector4f maxPt(0,0,0,0);
		if(indices.empty() || indices.at(iter->first)->empty())
		{
			N(oi,oi) = iter->second->size();
			pcl::getMinMax3D(*iter->second, minPt, maxPt);
		}
		else
		{
			N(oi,oi) = indices.at(iter->first)->size();
			pcl::getMinMax3D(*iter->second, *indices.at(iter->first), minPt, maxPt);
		}
		minPt[0] -= maxCorrespondenceDistance;
		minPt[1] -= maxCorrespondenceDistance;
		minPt[2] -= maxCorrespondenceDistance;
		maxPt[0] += maxCorrespondenceDistance;
		maxPt[1] += maxCorrespondenceDistance;
		maxPt[2] += maxCorrespondenceDistance;
		boundingBoxes.insert(std::make_pair(iter->first, std::make_pair(pcl::PointXYZ(minPt[0], minPt[1], minPt[2]), pcl::PointXYZ(maxPt[0], maxPt[1], maxPt[2]))));
		++oi;
	}

	typename pcl::search::KdTree<PointT> kdtree;
	int lastKdTreeId = 0;
	for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
	{
		if(uContains(idToIndex, iter->second.from()) && uContains(idToIndex, iter->second.to()))
		{
			const typename pcl::PointCloud<PointT>::Ptr & cloudFrom = clouds.at(iter->second.from());
			const typename pcl::PointCloud<PointT>::Ptr & cloudTo = clouds.at(iter->second.to());
			if(cloudFrom->size() && cloudTo->size())
			{
				//Are bounding boxes intersect?
				std::pair<pcl::PointXYZ, pcl::PointXYZ> bbMinMaxFrom = boundingBoxes.at(iter->second.from());
				std::pair<pcl::PointXYZ, pcl::PointXYZ> bbMinMaxTo = boundingBoxes.at(iter->second.to());
				Eigen::Affine3f t = Transform::getIdentity().toEigen3f();
				if(!iter->second.transform().isIdentity() && !iter->second.transform().isNull())
				{
					t = iter->second.transform().toEigen3f();
					bbMinMaxTo.first = pcl::transformPoint(bbMinMaxTo.first, t);
					bbMinMaxTo.second = pcl::transformPoint(bbMinMaxTo.second, t);
				}
				AABB bbFrom(Eigen::Vector3f((bbMinMaxFrom.second.x + bbMinMaxFrom.first.x)/2.0f, (bbMinMaxFrom.second.y + bbMinMaxFrom.first.y)/2.0f, (bbMinMaxFrom.second.z + bbMinMaxFrom.first.z)/2.0f),
						 Eigen::Vector3f((bbMinMaxFrom.second.x - bbMinMaxFrom.first.x)/2.0f, (bbMinMaxFrom.second.y - bbMinMaxFrom.first.y)/2.0f, (bbMinMaxFrom.second.z - bbMinMaxFrom.first.z)/2.0f));
				AABB bbTo(Eigen::Vector3f((bbMinMaxTo.second.x + bbMinMaxTo.first.x)/2.0f, (bbMinMaxTo.second.y + bbMinMaxTo.first.y)/2.0f, (bbMinMaxTo.second.z + bbMinMaxTo.first.z)/2.0f),
						 Eigen::Vector3f((bbMinMaxTo.second.x - bbMinMaxTo.first.x)/2.0f, (bbMinMaxTo.second.y - bbMinMaxTo.first.y)/2.0f, (bbMinMaxTo.second.z - bbMinMaxTo.first.z)/2.0f));
				//UDEBUG("%d = %f,%f,%f %f,%f,%f", iter->second.from(), bbMinMaxFrom.first[0], bbMinMaxFrom.first[1], bbMinMaxFrom.first[2], bbMinMaxFrom.second[0], bbMinMaxFrom.second[1], bbMinMaxFrom.second[2]);
				//UDEBUG("%d = %f,%f,%f %f,%f,%f", iter->second.to(), bbMinMaxTo.first[0], bbMinMaxTo.first[1], bbMinMaxTo.first[2], bbMinMaxTo.second[0], bbMinMaxTo.second[1], bbMinMaxTo.second[2]);
				if(testAABBAABB(bbFrom, bbTo))
				{
					if(lastKdTreeId <= 0 || lastKdTreeId!=iter->second.from())
					{
						//reconstruct kdtree
						if(indices.size() && indices.at(iter->second.from())->size())
						{
							kdtree.setInputCloud(cloudFrom, indices.at(iter->second.from()));
						}
						else
						{
							kdtree.setInputCloud(cloudFrom);
						}
					}

					pcl::Correspondences correspondences;
					pcl::IndicesPtr indicesTo(new std::vector<int>);
					std::set<int> addedFrom;
					if(indices.size() && indices.at(iter->second.to())->size())
					{
						const pcl::IndicesPtr & indicesTo = indices.at(iter->second.to());
						correspondences.resize(indicesTo->size());
						int oi=0;
						for(unsigned int i=0; i<indicesTo->size(); ++i)
						{
							std::vector<int> k_indices;
							std::vector<float> k_sqr_distances;
							if(kdtree.radiusSearch(pcl::transformPoint(cloudTo->at(indicesTo->at(i)), t), maxCorrespondenceDistance, k_indices, k_sqr_distances, 1))
							{
								if(addedFrom.find(k_indices[0]) == addedFrom.end())
								{
									correspondences[oi].index_match = k_indices[0];
									correspondences[oi].index_query = indicesTo->at(i);
									correspondences[oi].distance = k_sqr_distances[0];
									addedFrom.insert(k_indices[0]);
									++oi;
								}
							}
						}
						correspondences.resize(oi);
					}
					else
					{
						correspondences.resize(cloudTo->size());
						int oi=0;
						for(unsigned int i=0; i<cloudTo->size(); ++i)
						{
							std::vector<int> k_indices;
							std::vector<float> k_sqr_distances;
							if(kdtree.radiusSearch(pcl::transformPoint(cloudTo->at(i), t), maxCorrespondenceDistance, k_indices, k_sqr_distances, 1))
							{
								if(addedFrom.find(k_indices[0]) == addedFrom.end())
								{
									correspondences[oi].index_match = k_indices[0];
									correspondences[oi].index_query = i;
									correspondences[oi].distance = k_sqr_distances[0];
									addedFrom.insert(k_indices[0]);
									++oi;
								}
							}
						}
						correspondences.resize(oi);
					}

					UDEBUG("%d->%d: correspondences = %d", iter->second.from(), iter->second.to(), (int)correspondences.size());
					if(correspondences.size() && (minOverlap <= 0.0 ||
							(double(correspondences.size()) / double(clouds.at(iter->second.from())->size()) >= minOverlap &&
							 double(correspondences.size()) / double(clouds.at(iter->second.to())->size()) >= minOverlap)))
					{
						int i = idToIndex.at(iter->second.from());
						int j = idToIndex.at(iter->second.to());

						double Isum1 = 0, Isum2 = 0;
						double IRsum1 = 0, IRsum2 = 0;
						double IGsum1 = 0, IGsum2 = 0;
						double IBsum1 = 0, IBsum2 = 0;
						for (unsigned int c = 0; c < correspondences.size(); ++c)
						{
							const PointT & pt1 = cloudFrom->at(correspondences.at(c).index_match);
							const PointT & pt2 = cloudTo->at(correspondences.at(c).index_query);

							Isum1 += std::sqrt(static_cast<double>(sqr(pt1.r) + sqr(pt1.g) + sqr(pt1.b)));
							Isum2 += std::sqrt(static_cast<double>(sqr(pt2.r) + sqr(pt2.g) + sqr(pt2.b)));

							IRsum1 += static_cast<double>(pt1.r);
							IRsum2 += static_cast<double>(pt2.r);
							IGsum1 += static_cast<double>(pt1.g);
							IGsum2 += static_cast<double>(pt2.g);
							IBsum1 += static_cast<double>(pt1.b);
							IBsum2 += static_cast<double>(pt2.b);
						}
						N(i, j) = N(j, i) = correspondences.size();
						I(i, j) = Isum1 / N(i, j);
						I(j, i) = Isum2 / N(i, j);

						IR(i, j) = IRsum1 / N(i, j);
						IR(j, i) = IRsum2 / N(i, j);
						IG(i, j) = IGsum1 / N(i, j);
						IG(j, i) = IGsum2 / N(i, j);
						IB(i, j) = IBsum1 / N(i, j);
						IB(j, i) = IBsum2 / N(i, j);
					}
				}
			}
		}
	}

	cv::Mat_<double> A(num_images, num_images); A.setTo(0);
	cv::Mat_<double> b(num_images, 1); b.setTo(0);
	cv::Mat_<double> AR(num_images, num_images); AR.setTo(0);
	cv::Mat_<double> AG(num_images, num_images); AG.setTo(0);
	cv::Mat_<double> AB(num_images, num_images); AB.setTo(0);
	for (int i = 0; i < num_images; ++i)
	{
		for (int j = 0; j < num_images; ++j)
		{
			b(i, 0) += beta * N(i, j);
			A(i, i) += beta * N(i, j);
			AR(i, i) += beta * N(i, j);
			AG(i, i) += beta * N(i, j);
			AB(i, i) += beta * N(i, j);
			if (j == i) continue;
			A(i, i) += 2 * alpha * I(i, j) * I(i, j) * N(i, j);
			A(i, j) -= 2 * alpha * I(i, j) * I(j, i) * N(i, j);

			AR(i, i) += 2 * alpha * IR(i, j) * IR(i, j) * N(i, j);
			AR(i, j) -= 2 * alpha * IR(i, j) * IR(j, i) * N(i, j);

			AG(i, i) += 2 * alpha * IG(i, j) * IG(i, j) * N(i, j);
			AG(i, j) -= 2 * alpha * IG(i, j) * IG(j, i) * N(i, j);

			AB(i, i) += 2 * alpha * IB(i, j) * IB(i, j) * N(i, j);
			AB(i, j) -= 2 * alpha * IB(i, j) * IB(j, i) * N(i, j);
		}
	}

	cv::Mat_<double> gainsGray, gainsR, gainsG, gainsB;
	cv::solve(A, b, gainsGray);

	cv::solve(AR, b, gainsR);
	cv::solve(AG, b, gainsG);
	cv::solve(AB, b, gainsB);

	gains = cv::Mat_<double>(gainsGray.rows, 4);
	gainsGray.copyTo(gains.col(0));
	gainsR.copyTo(gains.col(1));
	gainsG.copyTo(gains.col(2));
	gainsB.copyTo(gains.col(3));

	if(ULogger::kInfo)
	{
		for(int i=0; i<gains.rows; ++i)
		{
			UINFO("Gain index=%d (id=%d) = %f (%f,%f,%f)", i, indexToId[i], gains(i, 0), gains(i, 1), gains(i, 2), gains(i, 3));
		}
	}
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
		const cv::Mat_<double> & gains,
		bool rgb)
{
	double gainR = gains(index, rgb?1:0);
	double gainG = gains(index, rgb?2:0);
	double gainB = gains(index, rgb?3:0);
	UDEBUG("index=%d gain=%f (%f,%f,%f)", index, gains(index, 0), gainR, gainG, gainB);
	if(indices->size())
	{
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			PointT & pt = cloud->at(indices->at(i));
			pt.r = uchar(std::max(0.0, std::min(255.0, double(pt.r) * gainR)));
			pt.g = uchar(std::max(0.0, std::min(255.0, double(pt.g) * gainG)));
			pt.b = uchar(std::max(0.0, std::min(255.0, double(pt.b) * gainB)));
		}
	}
	else
	{
		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			PointT & pt = cloud->at(i);
			pt.r = uchar(std::max(0.0, std::min(255.0, double(pt.r) * gainR)));
			pt.g = uchar(std::max(0.0, std::min(255.0, double(pt.g) * gainG)));
			pt.b = uchar(std::max(0.0, std::min(255.0, double(pt.b) * gainB)));
		}
	}
}

void GainCompensator::apply(
		int id,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		bool rgb) const
{
	pcl::IndicesPtr indices(new std::vector<int>);
	apply(id, cloud, indices, rgb);
}
void GainCompensator::apply(
		int id,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool rgb) const
{
	UASSERT_MSG(uContains(idToIndex_, id), uFormat("id=%d idToIndex_.size()=%d", id, (int)idToIndex_.size()).c_str());
	applyImpl<pcl::PointXYZRGB>(idToIndex_.at(id), cloud, indices, gains_, rgb);
}
void GainCompensator::apply(
		int id,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		bool rgb) const
{
	UASSERT_MSG(uContains(idToIndex_, id), uFormat("id=%d idToIndex_.size()=%d", id, (int)idToIndex_.size()).c_str());
	applyImpl<pcl::PointXYZRGBNormal>(idToIndex_.at(id), cloud, indices, gains_, rgb);
}

void GainCompensator::apply(
		int id,
		cv::Mat & image,
		bool rgb) const
{
	UASSERT_MSG(uContains(idToIndex_, id), uFormat("id=%d idToIndex_.size()=%d", id, (int)idToIndex_.size()).c_str());
	if(image.channels() == 1 || !rgb)
	{
		cv::multiply(image, gains_(idToIndex_.at(id), 0), image);
	}
	else if(image.channels()>=3)
	{
		std::vector<cv::Mat> channels;
		cv::split(image, channels);
		// assuming BGR
		cv::multiply(channels[0], gains_(idToIndex_.at(id), 3), channels[0]);
		cv::multiply(channels[1], gains_(idToIndex_.at(id), 2), channels[1]);
		cv::multiply(channels[2], gains_(idToIndex_.at(id), 1), channels[2]);
		cv::merge(channels, image);
	}
}

double GainCompensator::getGain(int id, double * r, double * g, double * b) const
{
	UASSERT_MSG(uContains(idToIndex_, id), uFormat("id=%d idToIndex_.size()=%d", id, (int)idToIndex_.size()).c_str());

	if(r)
	{
		*r = gains_(idToIndex_.at(id), 1);
	}
	if(g)
	{
		*g = gains_(idToIndex_.at(id), 2);
	}
	if(b)
	{
		*b = gains_(idToIndex_.at(id), 3);
	}

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
