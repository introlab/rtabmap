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
	 * @brief Builds it for @p ids.
	 * @param cache Filled with the neighborhoods when not null, which update() needs.
	 * @return False when the prediction would not be sparse, which the caller has to answer by
	 *         building the matrix: a model that leaves probability to spread fills every zero
	 *         of a column, and a densely linked graph reaches most of the map from every one.
	 */
	bool generate(const PredictionModel & model, const Memory * memory,
			const std::vector<int> & ids, NeighborsCache * cache);

	/**
	 * @brief Carries it over to @p ids without building it again.
	 *
	 * Every location already there keeps its index when locations are only appended, so the
	 * columns already built still apply and only the ones the appended locations reach are
	 * built again.
	 *
	 * @return False when @p ids is not the ids() it was built for with more appended, which the
	 *         caller has to answer by calling generate(): a location removed shifts the index of
	 *         every one after it.
	 */
	bool update(const PredictionModel & model, const Memory * memory,
			const std::vector<int> & ids, NeighborsCache & cache);

	/// prior = prediction x posterior.
	void multiply(const std::vector<float> & posterior, std::vector<float> & prior) const;

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
