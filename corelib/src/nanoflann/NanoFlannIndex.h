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

#ifndef CORELIB_SRC_NANOFLANN_NANOFLANNINDEX_H_
#define CORELIB_SRC_NANOFLANN_NANOFLANNINDEX_H_

#include <opencv2/opencv.hpp>

namespace rtabmap {

class NanoFlannIndexImpl;

/**
 * kd-tree backed by nanoflann, held by FlannIndex when its
 * NANOFLANN_INDEX_KDTREE_SINGLE algorithm is selected. Two trees are available,
 * buildIndex() picking one with its "incremental" argument:
 *
 * Incremental (nanoflann's KDTreeSingleIndexIncrementalAdaptor), a single
 * weight-balanced tree accepting points after it is built:
 *  - addPoints() inserts incrementally, and bulk-rebuilds when the batch is
 *    large relative to the tree.
 *  - removePoint() is lazy; a subtree is rebuilt, dropping its tombstones, once
 *    its removed fraction gets over "removedRatio".
 * Indexes returned by addPoints() stay valid for the lifetime of the index:
 * points are only appended and removals never renumber the remaining ones.
 *
 * Static (nanoflann's KDTreeSingleIndexAdaptor), built once from all the points
 * given to buildIndex(): cheaper to build and to search. Use it for a dataset
 * known to be fixed, like the image points searched during registration. Adding
 * or removing points is still possible, it rebuilds itself as the incremental
 * tree when that happens.
 *
 * Only float features are supported (nanoflann has no Hamming metric, binary
 * descriptors have to be converted first), with the L2 or L1 metric. Searches
 * are exact, there is no equivalent of rtflann's "checks" budget.
 */
class NanoFlannIndex
{
public:
	NanoFlannIndex();
	~NanoFlannIndex();

	NanoFlannIndex(const NanoFlannIndex &) = delete;
	NanoFlannIndex & operator=(const NanoFlannIndex &) = delete;

	void release();

	// features must be a CV_32FC1 matrix, one point per row. "incremental"
	// selects the tree accepting addPoints()/removePoint(), for which
	// "removedRatio" is the fraction of it that can be left removed before a
	// rebuild (1 to never rebuild). "leafMaxSize" is the number of points under
	// which the static tree stops splitting, it doesn't apply to the
	// incremental one, which holds a single point per node.
	//
	// A static tree given points to add afterwards is rebuilt as an incremental
	// one, so that building without the intention of adding points doesn't
	// prevent it (see addPoints()).
	void buildIndex(
			const cv::Mat & features,
			bool useDistanceL1,
			bool incremental,
			float removedRatio = 0.5f,
			int leafMaxSize = 10);

	// Return an empty vector if the index cannot be serialized: when it is not
	// built, or when points have been removed from it (the tree then indexes
	// holes that the matrix given back to loadIndex() cannot reproduce).
	std::vector<unsigned char> serializeIndex() const;

	// features must hold the very same points, in the same order, than those
	// that were indexed when the index was serialized.
	bool loadIndex(
			const cv::Mat & features,
			bool useDistanceL1,
			bool incremental,
			const unsigned char * indexData,
			size_t indexDataSize,
			float removedRatio = 0.5f,
			int leafMaxSize = 10,
			std::string * errorMsg = 0);

	// The indexed points as an indexedFeatures()x"dim" CV_32FC1 matrix, copied
	// out of the features they are referenced from. Empty if the index is not
	// built. Note that points removed from the tree are still part of it.
	cv::Mat indexedPoints() const;

	bool isBuilt() const {return index_ != 0;}

	// removed points excluded
	size_t indexedFeatures() const;

	// return Bytes
	size_t memoryUsed() const;

	// return the index assigned to each added point
	std::vector<unsigned int> addPoints(const cv::Mat & features);

	void removePoint(unsigned int index);

	// return squared distances, indices and distances are set to -1 for the
	// neighbors that couldn't be found.
	void knnSearch(
			const cv::Mat & query,
			cv::Mat & indices,
			cv::Mat & dists,
			int knn) const;

	// return squared distances
	void radiusSearch(
			const cv::Mat & query,
			std::vector<std::vector<size_t> > & indices,
			std::vector<std::vector<float> > & dists,
			float radius,
			int maxNeighbors,
			float eps,
			bool sorted) const;

private:
	// The metric (L2 or L1) and the compile-time dimension of the tree are only
	// known when the index is built, so the tree type is erased behind this
	// implementation, which also owns the points it indexes. Keeping nanoflann
	// out of this header is a side effect, not the reason.
	size_t appendPoints(const cv::Mat & features);
	void makeIncremental();

	NanoFlannIndexImpl * index_;
	int featuresDim_;
	// kept to rebuild the tree as an incremental one, see makeIncremental()
	bool useDistanceL1_;
	float removedRatio_;
};

} /* namespace rtabmap */

#endif /* CORELIB_SRC_NANOFLANN_NANOFLANNINDEX_H_ */
