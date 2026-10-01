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

#ifndef RTABMAP_BAYES_SPARSEPREDICTION_H_
#define RTABMAP_BAYES_SPARSEPREDICTION_H_

#include "bayes/PredictionModel.h"

#include <opencv2/core/core.hpp>
#include <utility>
#include <vector>

namespace rtabmap {

class Memory;

namespace bayes {

/**
 * @brief The prediction as its values only, one column at a time.
 *
 * A column holds the neighbors of one location within the depth of the model, so on a large
 * map the matrix DensePrediction would build is mostly zeros: holding it costs the number of
 * locations squared, against the far smaller number of values in it. Each column is built in a
 * buffer of its own and only its non-zero values are kept, so nothing of that size is ever
 * allocated.
 *
 * The values of every column live in one array, which the multiplication reads the way memory
 * likes to be read, and a column keeps the room it was given so that update() can rebuild one
 * without moving the others.
 */
class SparsePrediction
{
public:
	bool empty() const {return columns_.empty();}
	const std::vector<int> & ids() const {return ids_;}
	size_t values() const {return used_;}
	void clear();

	/**
	 * @brief Builds it for @p ids, whatever its columns come to hold.
	 *
	 * The prediction of a model that leaves probability to spread has no zero left in a column
	 * and nothing sparse to keep, which the caller answers with the matrix rather than asking
	 * for this. Nothing else falls back to one.
	 *
	 * @param cache Filled with the neighborhoods when not null, which update() needs.
	 */
	void generate(const PredictionModel & model, const Memory * memory,
			const std::vector<int> & ids, NeighborsCache * cache);

	/**
	 * @brief Carries it over to @p ids without walking the graph again.
	 *
	 * Only the columns whose contents changed are built again, from the neighborhoods of
	 * @p cache: the ones of the locations that were not there before, of their neighbors, and
	 * of the locations that shared their probability with one that is gone. Every other column
	 * is carried over, at another index when locations were removed.
	 *
	 * @return False when there is nothing to carry over: no prediction yet, or the virtual place
	 *         appearing or disappearing. The caller answers by calling generate(), which is also
	 *         what fills @p cache.
	 */
	bool update(const PredictionModel & model, const Memory * memory,
			const std::vector<int> & ids, NeighborsCache & cache);

	/// prior = prediction x posterior.
	void multiply(const std::vector<float> & posterior, std::vector<float> & prior) const;

	/**
	 * @brief The same prediction as a matrix, for the one caller that wants to look at it.
	 *
	 * The matrix costs what keeping the prediction sparse is saving, so this builds one to be
	 * read, dumped or compared against DensePrediction, and does not keep it.
	 */
	cv::Mat toMatrix() const;

	unsigned long memoryUsed() const;

private:
	/// Where a column sits in values_, and how much room it was given: a column rebuilt into
	/// more values than it has room for is moved to the end, leaving its room behind until
	/// compact() recovers it.
	struct Column
	{
		size_t offset = 0;
		size_t size = 0;
		size_t capacity = 0;
	};

	/// update() when @p ids is the ids() it was built for with more appended: every location
	/// keeps its index, so the columns are updated where they are.
	bool updateAppended(const PredictionModel & model, const Memory * memory,
			const std::vector<int> & ids, NeighborsCache & cache);

	/// update() when locations were removed, or came back in the middle of the ones already
	/// there: the index of a location moves, so the columns are laid out again.
	bool updateRemapped(const PredictionModel & model, const Memory * memory,
			const std::vector<int> & ids, NeighborsCache & cache);

	/// Builds one column, from the neighborhood of the location it is for.
	void buildColumn(const PredictionModel & model, const Memory * memory, int id, int index,
			const std::vector<int> & ids, const IdToIndexMap & idToIndex,
			std::vector<float> & buffer, NeighborsCache & cache);

	void takeColumn(std::vector<float> & column, int index, bool withRoomToGrow);
	void compact();

	std::vector<Column> columns_;
	std::vector<std::pair<int, float> > values_;
	size_t used_ = 0;               ///< How many of values_ belong to a column.
	std::vector<int> ids_;
};

} // namespace bayes
} // namespace rtabmap

#endif /* RTABMAP_BAYES_SPARSEPREDICTION_H_ */
