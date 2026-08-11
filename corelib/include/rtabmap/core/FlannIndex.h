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

#ifndef CORELIB_SRC_FLANNINDEX_H_
#define CORELIB_SRC_FLANNINDEX_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines
#include <list>
#include <opencv2/opencv.hpp>

#include <nanoflann.hpp>

namespace rtabmap {

class RTABMAP_CORE_EXPORT FlannIndex
{
public:
	enum Type {
		kRtFlann,
		kNanoFlann
	};

	static FlannIndex* create(Type type = kRtFlann);

	// A forward of the internal enum, indexes should match. See src/rtflann/defines.h
	enum flann_algorithm_t
	{
		FLANN_INDEX_LINEAR 			= 0,
		FLANN_INDEX_KDTREE 			= 1,
		FLANN_INDEX_KDTREE_SINGLE   = 4,
		FLANN_INDEX_LSH 			= 6,
	};

	FlannIndex();
	virtual ~FlannIndex();

	virtual void release() = 0;
	virtual std::vector<unsigned char> serializeIndex(bool computeChecksum = true) const = 0;

	virtual size_t indexedFeatures() const = 0;

	// return Bytes
	virtual size_t memoryUsed() const = 0;

	// Note that useDistanceL1 doesn't have any effect if LSH is used
	virtual void buildIndex(
			flann_algorithm_t algorithm,
			const cv::Mat & features,
			bool useDistanceL1 = false,
			float rebalancingFactor = 2.0f) = 0;
	// Return false if the indexData doesn't correspond to expected features used and parameters.
	virtual bool loadIndex(
		const std::vector<unsigned char> & indexData,
		flann_algorithm_t algorithm,
		const cv::Mat & features,
		bool useDistanceL1 = false,
		float rebalancingFactor = 2.0f,
		std::string * errorMsg = NULL) = 0;
	virtual bool loadIndex(
		const unsigned char * indexData,
		size_t indexDataSize,
		flann_algorithm_t algorithm,
		const cv::Mat & features,
		bool useDistanceL1 = false,
		float rebalancingFactor = 2.0f,
		std::string * errorMsg = NULL) = 0;

	virtual bool isBuilt() = 0;

	virtual int featuresType() const = 0;
	virtual int featuresDim() const = 0;

	virtual std::vector<unsigned int> addPoints(const cv::Mat & features) = 0;

	virtual void removePoint(unsigned int index) = 0;

	// return squared distances (indices should be casted in size_t)
	virtual void knnSearch(
			const cv::Mat & query,
			cv::Mat & indices,
			cv::Mat & dists,
	        int knn,
			int checks = 32,
			float eps = 0.0,
			bool sorted = true) const = 0;

	// return squared distances
	virtual void radiusSearch(
			const cv::Mat & query,
			std::vector<std::vector<size_t> > & indices,
			std::vector<std::vector<float> > & dists,
			float radius,
			int maxNeighbors = 0,
			int checks = 32,
			float eps = 0.0,
			bool sorted = true) const = 0;
};

class RTABMAP_CORE_EXPORT RtFlannIndex : public FlannIndex
{
public:
	RtFlannIndex();
	virtual ~RtFlannIndex();

	virtual void release() override;
	virtual std::vector<unsigned char> serializeIndex(bool computeChecksum = true) const override;
	virtual size_t indexedFeatures() const override;
	virtual size_t memoryUsed() const override;
	virtual void buildIndex(flann_algorithm_t algorithm, const cv::Mat & features, bool useDistanceL1 = false, float rebalancingFactor = 2.0f) override;
	virtual bool loadIndex(const std::vector<unsigned char> & indexData, flann_algorithm_t algorithm, const cv::Mat & features, bool useDistanceL1 = false, float rebalancingFactor = 2.0f, std::string * errorMsg = NULL) override;
	virtual bool loadIndex(const unsigned char * indexData, size_t indexDataSize, flann_algorithm_t algorithm, const cv::Mat & features, bool useDistanceL1 = false, float rebalancingFactor = 2.0f, std::string * errorMsg = NULL) override;
	virtual bool isBuilt() override;
	virtual int featuresType() const override;
	virtual int featuresDim() const override;
	virtual std::vector<unsigned int> addPoints(const cv::Mat & features) override;
	virtual void removePoint(unsigned int index) override;
	virtual void knnSearch(const cv::Mat & query, cv::Mat & indices, cv::Mat & dists, int knn, int checks = 32, float eps = 0.0, bool sorted = true) const override;
	virtual void radiusSearch(const cv::Mat & query, std::vector<std::vector<size_t> > & indices, std::vector<std::vector<float> > & dists, float radius, int maxNeighbors = 0, int checks = 32, float eps = 0.0, bool sorted = true) const override;

private:
	void * index_;
	unsigned int nextIndex_;
	int featuresType_;
	int featuresDim_;
	bool useDistanceL1_; 
	float rebalancingFactor_;
	flann_algorithm_t algorithm_;

	std::map<int, cv::Mat> addedDescriptors_;
	std::list<int> removedIndexes_;
};

class RTABMAP_CORE_EXPORT NanoFlannIndex : public FlannIndex
{
public:
	NanoFlannIndex();
	virtual ~NanoFlannIndex();

	virtual void release() override;
	virtual std::vector<unsigned char> serializeIndex(bool computeChecksum = true) const override;
	virtual size_t indexedFeatures() const override;
	virtual size_t memoryUsed() const override;
	virtual void buildIndex(flann_algorithm_t algorithm, const cv::Mat & features, bool useDistanceL1 = false, float rebalancingFactor = 2.0f) override;
	virtual bool loadIndex(const std::vector<unsigned char> & indexData, flann_algorithm_t algorithm, const cv::Mat & features, bool useDistanceL1 = false, float rebalancingFactor = 2.0f, std::string * errorMsg = NULL) override;
	virtual bool loadIndex(const unsigned char * indexData, size_t indexDataSize, flann_algorithm_t algorithm, const cv::Mat & features, bool useDistanceL1 = false, float rebalancingFactor = 2.0f, std::string * errorMsg = NULL) override;
	virtual bool isBuilt() override;
	virtual int featuresType() const override;
	virtual int featuresDim() const override;
	virtual std::vector<unsigned int> addPoints(const cv::Mat & features) override;
	virtual void removePoint(unsigned int index) override;
	virtual void knnSearch(const cv::Mat & query, cv::Mat & indices, cv::Mat & dists, int knn, int checks = 32, float eps = 0.0, bool sorted = true) const override;
	virtual void radiusSearch(const cv::Mat & query, std::vector<std::vector<size_t> > & indices, std::vector<std::vector<float> > & dists, float radius, int maxNeighbors = 0, int checks = 32, float eps = 0.0, bool sorted = true) const override;

private:
	struct PointCloudAdapter {
        std::vector<cv::Point2f> pts; // Храним точки прямо внутри адаптера
        inline size_t kdtree_get_point_count() const { return pts.size(); }
        inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
            return (dim == 0) ? pts[idx].x : pts[idx].y;
        }
        template <class BBOX> bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
    };

    // Определение типа дерева, которое использует этот адаптер
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloudAdapter>,
        PointCloudAdapter,
        2
    > MyKDTree;

    PointCloudAdapter pc_adapter_; // Сам объект адаптера
    MyKDTree* index_;              // Указатель на дерево nanoflann
    bool isBuilt_; 
};

} /* namespace rtabmap */

#endif /* CORELIB_SRC_FLANNINDEX_H_ */
