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

#ifndef RTABMAP_BAYES_PREDICTIONMODEL_H_
#define RTABMAP_BAYES_PREDICTIONMODEL_H_

#include <list>
#include <map>
#include <string>
#include <vector>
#if __cplusplus >= 201103L
#include <unordered_map>
#endif

namespace rtabmap {

class Memory;

namespace bayes {

/// Where each location sits in the prediction: its id to its row and column.
#if __cplusplus >= 201103L
typedef std::unordered_map<int, int> IdToIndexMap;
#else
typedef std::map<int, int> IdToIndexMap;
#endif

/// The neighborhood of the locations it was asked for, which the incremental updates of the
/// prediction read instead of walking the graph again.
typedef std::map<int, std::map<int, int> > NeighborsCache;

/**
 * @brief The loop closure prediction model, and the column arithmetic that follows from it.
 *
 * Format `{Vp, Lc, l1, l2, ...}`: the probability of moving to a new place, of staying at the
 * same location, then of moving to a neighbor at each depth of the graph. See
 * Parameters::kBayesPredictionLC().
 *
 * A column of the prediction is the distribution over where the robot moves to from one
 * location. Both the dense and the sparse prediction fill their columns through this, so the
 * probabilities cannot end up differing between them. A column is given as the pointer to its
 * first value and the step between two of them: the matrix stores a column strided by its
 * width, while the sparse build fills one contiguous column at a time.
 */
class PredictionModel
{
public:
	/**
	 * @brief Sets the model from a space separated list of probabilities.
	 * @return False when the string does not hold at least two values in [0, 1], the previous
	 *         model being kept.
	 */
	bool set(const std::string & prediction);

	const std::vector<double> & values() const {return values_;}
	std::string str() const;

	/// How deep in the graph a column reaches: one less than the number of values.
	int depth() const {return (int)values_.size()-1;}

	/// Whether the values leave probability for normalize() to spread over every other
	/// location, which fills every zero of a column and leaves nothing sparse to keep.
	bool spreadsOverAllLocations() const {return total_ < 1;}

	float total() const {return total_;}
	bool valid() const {return values_.size() >= 2;}

	float virtualPlacePrior() const {return virtualPlacePrior_;}
	void setVirtualPlacePrior(float prior) {virtualPlacePrior_ = prior;}

	/**
	 * @brief Puts the probability of each neighbor into a column.
	 * @return The sum of what it put there, which normalize() takes.
	 */
	float addNeighborProb(float * column, size_t stride, const std::map<int, int> & neighbors,
			const IdToIndexMap & idToIndex) const;

	/**
	 * @brief Normalizes one column and applies the virtual place probability.
	 * @param index Index of the location the column is for, so of its diagonal value.
	 * @param addedProbabilitiesSum What addNeighborProb() put in it.
	 * @param virtualPlaceUsed Whether the first location is the virtual place.
	 */
	void normalize(float * column, size_t stride, int size, unsigned int index,
			float addedProbabilitiesSum, bool virtualPlaceUsed) const;

	/// Fills the column of the virtual place, the hypothesis of a location never visited.
	void fillVirtualPlaceColumn(float * column, size_t stride, int size) const;

	unsigned long memoryUsed() const {return values_.capacity() * sizeof(double);}

private:
	std::vector<double> values_;
	float total_ = 0.0f;
	float epsilon_ = 0.0f;          ///< Smallest probability of the model, under which normalize() drops a value.
	float virtualPlacePrior_ = 0.0f;
};

/**
 * @brief The neighbors of a location within the depth of the model, and the locations at
 *        margin 0 of it, meaning the same place: their columns hold this same neighborhood.
 *
 * @param cache Filled when not null, for the incremental updates to reuse.
 */
std::map<int, int> resolveNeighbors(const Memory * memory, int id, int maxDepth,
		const IdToIndexMap & idToIndexMap, std::list<int> & idsAtMargin0, NeighborsCache * cache);

/**
 * @brief The neighborhood of a location from @p cache, querying and caching it when absent.
 *
 * Caching it also adds the location to the neighborhoods of its own neighbors.
 */
const std::map<int, int> & cachedNeighbors(const Memory * memory, int id, int maxDepth,
		NeighborsCache & cache);

} // namespace bayes
} // namespace rtabmap

#endif /* RTABMAP_BAYES_PREDICTIONMODEL_H_ */
