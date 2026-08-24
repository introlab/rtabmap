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

#ifndef RTABMAP_BAYES_DENSEPREDICTION_H_
#define RTABMAP_BAYES_DENSEPREDICTION_H_

#include "bayes/PredictionModel.h"

#include <opencv2/core/core.hpp>
#include <vector>

namespace rtabmap {

class Memory;

namespace bayes {

/**
 * @brief The prediction as a matrix, one column per location.
 *
 * The matrix costs the number of locations squared, whatever the graph puts in it, which on a
 * large map is most of what the Bayes filter holds and most of what an iteration reads. See
 * SparsePrediction for the form that keeps only the values.
 */
class DensePrediction
{
public:
	bool empty() const {return matrix_.empty();}
	const cv::Mat & matrix() const {return matrix_;}

	/// The locations the matrix is built for, which the incremental update carries over.
	const std::vector<int> & ids() const {return ids_;}

	void clear() {matrix_ = cv::Mat(); ids_.clear();}

	/**
	 * @brief Builds the matrix for @p ids and keeps it, along with the ids it is built for.
	 *
	 * The matrix already there is carried over when it is built for locations @p ids only
	 * appends to; otherwise every column is built again.
	 *
	 * @param fullUpdate Rebuilds every column rather than carrying the matrix over.
	 * @param cache Filled with the neighborhoods, for a later incremental update.
	 */
	const cv::Mat & generate(const PredictionModel & model, const Memory * memory,
			const std::vector<int> & ids, bool fullUpdate, NeighborsCache * cache);

	/// prior = prediction x posterior.
	void multiply(const std::vector<float> & posterior, std::vector<float> & prior) const;

	unsigned long memoryUsed() const;

private:
	cv::Mat generateFull(const PredictionModel & model, const Memory * memory,
			const std::vector<int> & ids, NeighborsCache * cache) const;
	cv::Mat update(const PredictionModel & model, const Memory * memory,
			const std::vector<int> & oldIds, const std::vector<int> & newIds,
			NeighborsCache * cache) const;

	cv::Mat matrix_;
	std::vector<int> ids_;
};

} // namespace bayes
} // namespace rtabmap

#endif /* RTABMAP_BAYES_DENSEPREDICTION_H_ */
