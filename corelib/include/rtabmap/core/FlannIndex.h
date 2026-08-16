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

class NanoFlannIndex;

/**
 * @class FlannIndex
 * @brief Nearest neighbor index over a set of features
 *
 * Wraps the search structures of the vendored rtflann and nanoflann libraries
 * behind one interface, the structure being chosen with flann_algorithm_t at
 * build time. Used for the visual word dictionary (VWDictionary) and for the
 * 2D point searches of visual registration (RegistrationVis).
 *
 * The features are not copied: the index refers to the matrices it is given and
 * keeps them alive, cv::Mat data being reference counted, so they must not be
 * modified in place while it is in use. Every point it holds is
 * designated by an index, assigned in the order the points were added and
 * stable for the lifetime of the index: removePoint() leaves a hole rather
 * than renumbering the points after it.
 */
class RTABMAP_CORE_EXPORT FlannIndex
{
public:
	/**
	 * @enum flann_algorithm_t
	 * @brief The index structure built by buildIndex()
	 *
	 * The values under 8 are forwarded from rtflann's own enum and have to
	 * match it (see src/rtflann/defines.h); the nanoflann ones are
	 * rtabmap-specific and kept outside its range (0-7, 254, 255). A value is
	 * written in the serialized index header and checked back on load, so none
	 * of them may be renumbered.
	 *
	 * The nanoflann structures take float features only (nanoflann has no
	 * Hamming metric) and search exactly, ignoring "checks". That makes them
	 * the fastest ones for 2D and 3D points, and the wrong ones for
	 * descriptors: an exact search visits more and more of the tree as the
	 * dimension grows, down to being as slow as an exhaustive search. Prefer
	 * the approximate rtflann kd-trees for those.
	 */
	enum flann_algorithm_t
	{
		FLANN_INDEX_LINEAR 			= 0, ///< Exhaustive search
		FLANN_INDEX_KDTREE 			= 1, ///< 4 randomized kd-trees, searched approximately
		FLANN_INDEX_KDTREE_SINGLE   = 4, ///< Single kd-tree, searched exactly
		FLANN_INDEX_LSH 			= 6, ///< Locality-Sensitive Hashing (binary descriptors)

		/// nanoflann kd-tree. With a rebalancing factor of 1 it is built once,
		/// which is the cheapest to build and to search; over 1 it is the
		/// weight-balanced tree accepting addPoints()/removePoint(), which
		/// cannot be serialized while some of its points are removed.
		NANOFLANN_INDEX_KDTREE_SINGLE = 100,
	};

	FlannIndex();
	virtual ~FlannIndex();

	/** @brief Drop the index and everything it holds, back to the state of a new one. */
	void release();

	/**
	 * @brief Serialize the index, to be given back to loadIndex()
	 * @param computeChecksum Add a checksum of the indexed features to the
	 *        data, which loadIndex() compares against the features it is given
	 * @return The serialized index, empty when there is nothing to serialize or
	 *         when the structure in use cannot be
	 *
	 * The format depends on the architecture and on the versions of the
	 * vendored libraries: loadIndex() refuses an index it cannot read, leaving
	 * it to be rebuilt.
	 */
	std::vector<unsigned char> serializeIndex(bool computeChecksum = true) const;

	/** @return Number of indexed features, the removed ones excluded. */
	size_t indexedFeatures() const;

	/**
	 * @return Bytes used by the index, the features themselves excluded as
	 *         they are only referred to.
	 */
	size_t memoryUsed() const;

	/**
	 * @brief Build the index over the given features, releasing any previous one
	 * @param algorithm The structure to build
	 * @param features One feature per row, CV_32FC1 or, for the rtflann
	 *        structures only, CV_8UC1 for binary descriptors (Hamming distance)
	 * @param useDistanceL1 Search with the L1 distance instead of L2, ignored
	 *        by LSH and by the binary descriptors
	 * @param rebalancingFactor Fraction (factor-1)/factor of the index that can
	 *        be left removed before it is rebuilt, e.g. half of it for 2. Set
	 *        to 1 to never rebuild it.
	 */
	void buildIndex(
			flann_algorithm_t algorithm,
			const cv::Mat & features,
			bool useDistanceL1 = false,
			float rebalancingFactor = 2.0f);

	/**
	 * @brief Load an index serialized by serializeIndex(), releasing any previous one
	 * @param indexData The serialized index
	 * @param algorithm The structure it was built with
	 * @param features The very same features it was built with, in the same
	 *        order: the index refers to them by their row
	 * @param useDistanceL1 The distance it was built with
	 * @param rebalancingFactor See buildIndex(). The serialized data carries the
	 *        one the index was built with, which is deprecated and ignored:
	 *        this one is used instead.
	 * @param errorMsg Filled with what didn't match when the index is refused
	 * @return False if the data doesn't correspond to the given features and
	 *         parameters, in which case the index is left released
	 */
	bool loadIndex(
		const std::vector<unsigned char> & indexData,
		flann_algorithm_t algorithm,
		const cv::Mat & features,
		bool useDistanceL1 = false,
		float rebalancingFactor = 2.0f,
		std::string * errorMsg = NULL);
	/** @brief Load an index from a raw buffer, see the overload above. */
	bool loadIndex(
		const unsigned char * indexData,
		size_t indexDataSize,
		flann_algorithm_t algorithm,
		const cv::Mat & features,
		bool useDistanceL1 = false,
		float rebalancingFactor = 2.0f,
		std::string * errorMsg = NULL);

	/** @return Whether an index has been built or loaded. */
	bool isBuilt();

	/** @return Type of the indexed features (CV_32FC1 or CV_8UC1). */
	int featuresType() const {return featuresType_;}
	/** @return Dimension of the indexed features. */
	int featuresDim() const {return featuresDim_;}

	/**
	 * @brief Add features to the index
	 * @param features One feature per row, of the type and dimension the index
	 *        was built with
	 * @return The index assigned to each of them, empty when the structure
	 *         doesn't accept points after it is built
	 */
	std::vector<unsigned int> addPoints(const cv::Mat & features);

	/**
	 * @brief Remove an indexed feature, by the index addPoints() gave for it
	 *
	 * The feature is only marked as removed: it is skipped by the searches, but
	 * keeps taking memory until the index is rebuilt (see the rebalancing
	 * factor of buildIndex()). Not supported by every structure.
	 */
	void removePoint(unsigned int index);

	/**
	 * @brief Search the k nearest neighbors of each query
	 * @param query One feature per row, of the type and dimension the index was
	 *        built with
	 * @param indices Neighbors found, one query per row, CV_32SC1. The
	 *        neighbors that couldn't be found are set to -1.
	 * @param dists Their squared distances, CV_32FC1, or CV_32SC1 for the
	 *        Hamming distances of binary descriptors
	 * @param knn Number of neighbors to search for
	 * @param checks Number of leaves an approximate search visits, the exact
	 *        structures ignoring it
	 * @param eps Search for eps-approximate neighbors
	 * @param sorted Give the neighbors back by increasing distance
	 */
	void knnSearch(
			const cv::Mat & query,
			cv::Mat & indices,
			cv::Mat & dists,
	        int knn,
			int checks = 32,
			float eps = 0.0,
			bool sorted = true) const;

	/**
	 * @brief Search the neighbors of each query within a radius
	 * @param query One feature per row, of the type and dimension the index was
	 *        built with
	 * @param indices Neighbors found, one vector per query
	 * @param dists Their squared distances, one vector per query
	 * @param radius Search radius, squared internally: it is a distance, not a
	 *        squared one
	 * @param maxNeighbors Maximum number of neighbors per query, the nearest
	 *        ones being kept. 0 for all of them.
	 * @param checks Number of leaves an approximate search visits, the exact
	 *        structures ignoring it
	 * @param eps Search for eps-approximate neighbors
	 * @param sorted Give the neighbors back by increasing distance
	 */
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
	void * index_;               // rtflann backend
	NanoFlannIndex * nanoIndex_; // nanoflann backend, only one of the two is set
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
