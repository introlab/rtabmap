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

#ifndef BAYESFILTER_H_
#define BAYESFILTER_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <opencv2/core/core.hpp>
#include <list>
#include <map>
#include <set>
#include <utility>
#include <vector>
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/core/Parameters.h"

namespace rtabmap {

class Memory;
class Signature;

namespace bayes {
class PredictionModel;
class DensePrediction;
class SparsePrediction;
}

/**
 * @class BayesFilter
 * @brief Recursive Bayesian filter for loop-closure hypothesis estimation in RTAB-Map.
 *
 * This class implements the prediction and update steps of a Bayes filter used to estimate
 * the posterior probability over candidate locations (signatures) in working memory. It is
 * typically called by Rtabmap after likelihood values have been computed from visual
 * word comparisons.
 *
 * The filter operates in two steps on each iteration:
 * - **Prediction**: builds a transition matrix from the memory graph and multiplies it
 *   with the previous posterior to obtain the prior.
 * - **Update**: multiplies the prior by the observation likelihood and normalizes the result.
 *
 * The prediction matrix is built from neighbor relationships in @ref Memory, using a
 * Gaussian-like model configured through @ref Parameters::kBayesPredictionLC(). A virtual
 * place (negative signature id, see @ref Memory::kIdVirtual) represents the hypothesis
 * that the current observation comes from a new location.
 *
 * Related parameters (see @ref Parameters):
 * - @ref Parameters::kBayesPredictionLC() — transition probabilities per graph depth level.
 * - @ref Parameters::kBayesVirtualPlacePriorThr() — prior for the virtual place.
 * - @ref Parameters::kBayesFullPredictionUpdate() — regenerate the full prediction matrix each iteration.
 * - @ref Parameters::kBayesSparsePrediction() — keep the prediction sparse and multiply it sparsely.
 *
 * @see Memory::getNeighborsId()
 * @see Rtabmap
 */
class RTABMAP_CORE_EXPORT BayesFilter
{
public:
	/**
	 * @brief Constructs a Bayes filter with default or custom parameters.
	 * @param parameters Optional parameter map (Bayes group keys). Defaults are used for missing keys.
	 */
	BayesFilter(const ParametersMap & parameters = ParametersMap());
	virtual ~BayesFilter();

	/**
	 * @brief Updates internal settings from the parameter map.
	 * @param parameters Map containing Bayes group keys.
	 */
	virtual void parseParameters(const ParametersMap & parameters);

	/**
	 * @brief Runs one Bayes filter iteration (prediction + update).
	 *
	 * Given a likelihood map over signature ids, computes and stores the normalized posterior.
	 * The prediction matrix is generated or updated from @ref Memory using the ids present
	 * in @p likelihood.
	 *
	 * Read the result with @ref getPosteriorIds() and @ref getPosteriorValues().
	 *
	 * @param memory Working memory instance (must not be null).
	 * @param likelihood Observation likelihood per signature id (must not be empty).
	 * @return False on error (null memory, empty likelihood, or invalid prediction model),
	 *         the posterior being left unchanged.
	 */
	bool computePosterior(const Memory * memory, const std::map<int, float> & likelihood);

	/**
	 * @brief Clears posterior, prediction matrix and cached neighbor indices.
	 */
	void reset();

	/**
	 * @brief Sets the loop-closure prediction model from a space-separated string.
	 *
	 * Format: `{Vp, Lc, l1, l2, l3, ...}` where:
	 * - **Vp** — virtual place probability. This is the probability to move to a new place (unvisited location).
	 * - **Lc** — loop closure (depth 0) probability. This is the probability to stay at the same location.
	 * - **l1, l2, ...** — probabilities for neighbors at increasing graph depth levels. This is the probability to move to a neighbor at the given depth level.
	 *
	 * Each value must be in [0, 1]. At least two values are required. Invalid strings are rejected
	 * and the previous model is kept.
	 *
	 * @param prediction Space-separated list of probabilities (same format as @ref Parameters::kBayesPredictionLC()).
	 */
	void setPredictionLC(const std::string & prediction);

	/**
	 * @brief The locations the posterior is over, ascending by id.
	 *
	 * The virtual place (@ref Memory::kIdVirtual) is the first of them when it is one.
	 */
	const std::vector<int> & getPosteriorIds() const {return _posteriorIds;}

	/**
	 * @brief The probability of each location of @ref getPosteriorIds(), in the same order.
	 */
	const std::vector<float> & getPosteriorValues() const {return _posteriorValues;}

	/**
	 * @brief Returns the virtual place prior threshold.
	 * @return Value in [0, 1] used when building the virtual place row of the prediction matrix.
	 */
	float getVirtualPlacePrior() const;

	/**
	 * @brief Whether the prediction is being kept in its sparse form rather than as a matrix.
	 *
	 * False when @ref Parameters::kBayesSparsePrediction() is disabled, and over a model whose
	 * values sum to less than 1, which leaves no zero in a column to keep out of the values.
	 */
	bool isPredictionSparse() const;

	/**
	 * @brief Returns the loop-closure prediction model as a vector of values.
	 * @return Vector in the format `{Vp, Lc, l1, l2, l3, ...}`.
	 */
	const std::vector<double> & getPredictionLC() const;

	/**
	 * @brief Returns the loop-closure prediction model as a space-separated string.
	 * @return String representation of @ref getPredictionLC().
	 */
	std::string getPredictionLCStr() const;

	/**
	 * @brief Builds or updates the prediction (transition) matrix for the given signature ids.
	 *
	 * Rows and columns correspond to @p ids. Neighbor links are queried from @ref Memory to fill
	 * transition probabilities according to @ref getPredictionLC(). When @p ids match the
	 * current posterior keys, the cached matrix may be returned without recomputation.
	 *
	 * When the prediction is being kept sparse, the matrix is expanded from it rather than
	 * kept: it costs the memory that keeping the prediction sparse is saving, so ask for it to
	 * read, dump or compare the prediction, not on every iteration.
	 *
	 * @param memory Working memory instance (must not be null).
	 * @param ids Ordered list of signature ids (often includes @ref Memory::kIdVirtual as first element).
	 * @return Square CV_32FC1 matrix of size ids.size() × ids.size().
	 */
	cv::Mat generatePrediction(const Memory * memory, const std::vector<int> & ids);

	/**
	 * @brief Estimates memory usage of this object and its internal containers.
	 * @return Approximate memory footprint in bytes.
	 */
	unsigned long getMemoryUsed() const;

private:
	/**
	 * @brief Realigns the posterior with the ids of the likelihood.
	 *
	 * Keeps the probability of the locations that are in both. Called only when the ids differ.
	 */
	void updatePosterior(const Memory * memory, const std::map<int, float> & likelihood);

	/**
	 * @brief Settles whether the prediction is kept sparse, from the parameter and the model.
	 *
	 * Called when either of the two changes rather than on every iteration, and releases the
	 * sparse form when the answer is no.
	 */
	void updateKeepSparse();

private:
	std::vector<int> _posteriorIds;               ///< The locations the posterior is over, ascending by id.
	std::vector<float> _posteriorValues;          ///< The probability of each of them, in the same order.
	std::vector<int> _likelihoodIds;              ///< The ids of the likelihood of an iteration, in its order.
	std::vector<float> _likelihoodValues;         ///< The likelihood of an iteration, in the same order.
	std::vector<float> _priorValues;              ///< The prior of an iteration, in the same order.

	bayes::PredictionModel * _model;              ///< The `{Vp, Lc, l1, ...}` model and the column arithmetic of it.
	bayes::DensePrediction * _dense;              ///< The prediction as a matrix, used when it is not kept sparse.
	bayes::SparsePrediction * _sparse;            ///< The prediction as its values only, one column at a time.
	std::map<int, std::map<int, int> > _neighborsIndex; ///< Cached neighbor margins per signature id, for the incremental updates.

	bool _fullPredictionUpdate;                   ///< If true, rebuild the whole prediction each time.
	bool _sparsePrediction;                       ///< Keep the prediction sparse (Bayes/SparsePrediction).
	bool _keepSparse;                             ///< Whether it is being kept sparse: the parameter, over a model that leaves nothing sparse to keep.
	bool _predictionChanged;                      ///< True when the prediction has to be built again.
};

} // namespace rtabmap

#endif /* BAYESFILTER_H_ */
