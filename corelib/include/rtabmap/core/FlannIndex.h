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

namespace rtabmap {

class RTABMAP_CORE_EXPORT FlannIndex
{
public:
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

	void release();
	std::vector<unsigned char> serializeIndex(bool computeChecksum = true) const;

	size_t indexedFeatures() const;

	// return Bytes
	size_t memoryUsed() const;

	// Note that useDistanceL1 doesn't have any effect if LSH is used
	void buildIndex(
			flann_algorithm_t algorithm,
			const cv::Mat & features,
			bool useDistanceL1 = false,
			float rebalancingFactor = 2.0f);
	// Return false if the indexData doesn't correspond to expected features used and parameters.
	bool loadIndex(
		const std::vector<unsigned char> & indexData,
		flann_algorithm_t algorithm,
		const cv::Mat & features,
		bool useDistanceL1 = false,
		float rebalancingFactor = 2.0f,
		std::string * errorMsg = NULL);
	bool loadIndex(
		const unsigned char * indexData,
		size_t indexDataSize,
		flann_algorithm_t algorithm,
		const cv::Mat & features,
		bool useDistanceL1 = false,
		float rebalancingFactor = 2.0f,
		std::string * errorMsg = NULL);

	bool isBuilt();

	int featuresType() const {return featuresType_;}
	int featuresDim() const {return featuresDim_;}

	std::vector<unsigned int> addPoints(const cv::Mat & features);

	void removePoint(unsigned int index);

	// return squared distances (indices should be casted in size_t)
	void knnSearch(
			const cv::Mat & query,
			cv::Mat & indices,
			cv::Mat & dists,
	        int knn,
			int checks = 32,
			float eps = 0.0,
			bool sorted = true) const;

	// return squared distances
	void radiusSearch(
			const cv::Mat & query,
			std::vector<std::vector<size_t> > & indices,
			std::vector<std::vector<float> > & dists,
			float radius,
			int maxNeighbors = 0,
			int checks = 32,
			float eps = 0.0,
			bool sorted = true) const;

private:
	void * index_;
	unsigned int nextIndex_;
	int featuresType_;
	int featuresDim_;
	bool useDistanceL1_; // true=EUCLEDIAN_L2 false=MANHATTAN_L1
	float rebalancingFactor_;
	flann_algorithm_t algorithm_;

	// keep feature in memory until the tree is rebuilt
	// (in case the word is deleted when removed from the VWDictionary)
	std::map<int, cv::Mat> addedDescriptors_;
	std::list<int> removedIndexes_;
};

} /* namespace rtabmap */

#endif /* CORELIB_SRC_FLANNINDEX_H_ */
