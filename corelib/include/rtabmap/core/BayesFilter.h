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
	float getVirtualPlacePrior() const {return _virtualPlacePrior;}

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
	 * @param memory Working memory instance (must not be null).
	 * @param ids Ordered list of signature ids (often includes @ref Memory::kIdVirtual as first element).
	 * @return Square CV_32FC1 matrix of size ids.size() × ids.size(), or an empty matrix when
	 *         @ref Parameters::kBayesSparsePrediction() is enabled and the prediction is being
	 *         kept in its sparse form, in which case no matrix exists to return.
	 */
	cv::Mat generatePrediction(const Memory * memory, const std::vector<int> & ids);

	/**
	 * @brief Estimates memory usage of this object and its internal containers.
	 * @return Approximate memory footprint in bytes.
	 */
	unsigned long getMemoryUsed() const;

private:
	/**
	 * @brief Whether the posterior is indexed by exactly @p ids, in that order.
	 */
	bool posteriorHasSameIds(const std::vector<int> & ids) const;

	/**
	 * @brief Incrementally updates the prediction matrix when ids are added or removed.
	 */
	cv::Mat updatePrediction(const cv::Mat & oldPrediction,
			const Memory * memory,
			const std::vector<int> & oldIds,
			const std::vector<int> & newIds);

	/**
	 * @brief Realigns the posterior with the ids of the likelihood.
	 *
	 * Rebuilds @ref _posterior and the vectors indexed the same way, keeping the probability
	 * of the locations that are in both. Called only when the ids differ.
	 */
	void updatePosterior(const Memory * memory, const std::map<int, float> & likelihood);

	/**
	 * @brief Fills the column of the virtual place (the unvisited location hypothesis).
	 *
	 * @param column First value of the column.
	 * @param stride Step between two values of the column (1 when it is contiguous, the
	 *               width of the matrix when it is one of its columns).
	 * @param size Number of values in the column.
	 */
	void fillVirtualPlaceColumn(float * column, size_t stride, int size) const;

	/**
	 * @brief Normalizes one column of the prediction and applies the virtual place probability.
	 *
	 * @param column First value of the column, see @ref fillVirtualPlaceColumn() for @p stride
	 *               and @p size.
	 * @param index Index of the location this column is for, so of its diagonal value.
	 * @param addedProbabilitiesSum Sum of the values @ref addNeighborProb() put in it.
	 * @param virtualPlaceUsed Whether the first location is the virtual place.
	 */
	void normalize(float * column, size_t stride, int size, unsigned int index, float addedProbabilitiesSum, bool virtualPlaceUsed) const;

	/**
	 * @brief Builds the prediction directly in its sparse form, without the matrix.
	 *
	 * One column at a time in a buffer of its own, so nothing of the size of the working
	 * memory squared is ever allocated. Used when the graph is fixed (localization mode),
	 * where no incremental matrix update needs the matrix to be kept.
	 *
	 * @param memory Working memory instance (must not be null).
	 * @param ids Ordered list of signature ids, as in @ref generatePrediction().
	 * @return False when the prediction would not be sparse, which the caller has to answer
	 *         by building the dense matrix. Happens when the values of
	 *         @ref Parameters::kBayesPredictionLC() sum to less than 1, as @ref normalize()
	 *         then spreads the missing probability over every zero of a column.
	 */
	bool generateSparsePrediction(const Memory * memory, const std::vector<int> & ids);

	/**
	 * @brief Carries the sparse prediction over to a longer list of ids, without rebuilding it.
	 *
	 * Every id already there keeps its index when ids are only appended, so the columns
	 * already built still apply and only the ones the appended ids reach are built again.
	 *
	 * @param memory Working memory instance (must not be null).
	 * @param oldIds The ids the prediction was built for.
	 * @param newIds The ids it should be indexed by.
	 * @return False when @p newIds is not @p oldIds with more appended, which the caller has
	 *         to answer by building the prediction again with @ref generateSparsePrediction():
	 *         an id removed shifts the index of every one after it. Also false when
	 *         @ref Parameters::kBayesFullPredictionUpdate() asks for a full rebuild.
	 */
	bool updateSparsePrediction(const Memory * memory,
			const std::vector<int> & oldIds,
			const std::vector<int> & newIds);

	/**
	 * @brief Takes the non-zero values of a built column into the prediction, and zeroes the buffer.
	 *
	 * @param column Buffer holding the column, zeroed on return.
	 * @param index Index of the column in the prediction.
	 * @param withRoomToGrow Gives the column slightly more room than its values need, so that
	 *                       rebuilding it into a few more values does not have to move it.
	 */
	void takeSparsePredictionColumn(std::vector<float> & column, int index, bool withRoomToGrow);

	/**
	 * @brief Packs the columns into the order they are multiplied in, each with the room it needs.
	 */
	void compactSparsePrediction();

	/**
	 * @brief The neighborhood of an id from @ref _neighborsIndex, querying and caching it if absent.
	 */
	const std::map<int, int> & cachedNeighbors(const Memory * memory, int id);

	/**
	 * @brief Releases the sparse prediction and the memory it holds.
	 */
	void clearSparsePrediction();

	/**
	 * @brief Approximate footprint of the sparse prediction, in bytes.
	 */
	unsigned long getSparsePredictionMemoryUsed() const;

	/**
	 * @brief Computes prior = prediction x posterior from the sparse prediction.
	 *
	 * Mathematically identical to the dense multiplication, up to the order the products of a
	 * row are summed in.
	 *
	 * @param posterior The last posterior, as many values as the prediction has columns.
	 * @param prior Output, sized by this method.
	 */
	void multiplySparsePrediction(const std::vector<float> & posterior, std::vector<float> & prior) const;

private:
	std::vector<int> _posteriorIds;               ///< The locations the posterior is over, ascending by id.
	std::vector<float> _posteriorValues;          ///< The probability of each of them, in the same order.
	std::vector<int> _likelihoodIds;              ///< The ids of the likelihood of an iteration, in its order.
	std::vector<float> _likelihoodValues;         ///< The likelihood of an iteration, in the same order.
	std::vector<float> _priorValues;              ///< The prior of an iteration, in the same order.
	cv::Mat _prediction;                          ///< Cached prediction/transition matrix.
	float _virtualPlacePrior;                     ///< Prior for virtual place transitions.
	std::vector<double> _predictionLC;            ///< Model `{Vp, Lc, l1, l2, ...}`.
	bool _fullPredictionUpdate;                   ///< If true, rebuild the full prediction matrix each time.
	float _totalPredictionLCValues;               ///< Sum of all values in _predictionLC.
	float _predictionEpsilon;                     ///< Minimum non-zero probability in the model.
	bool _sparsePrediction;                       ///< Multiply the prediction sparsely (Bayes/SparsePrediction).
	bool _predictionChanged;                      ///< True when _prediction was rebuilt, so the sparse form is stale.
	bool _sparsePredictionRejected;               ///< True when the current prediction was measured as too dense to keep sparse.
	/// Where a column of the sparse prediction sits in _sparsePredictionValues, and how much
	/// room it was given: a column rebuilt into more values than it has room for is moved to
	/// the end, leaving its room behind until compactSparsePrediction() recovers it.
	struct SparseColumn
	{
		size_t offset = 0;
		size_t size = 0;
		size_t capacity = 0;
	};
	std::vector<SparseColumn> _sparsePredictionColumns; ///< The columns of the sparse prediction, built instead of _prediction.
	std::vector<std::pair<int, float> > _sparsePredictionValues; ///< The (row, value) of every non-zero, column by column.
	size_t _sparsePredictionUsed = 0;             ///< How many of _sparsePredictionValues belong to a column.
	std::vector<int> _sparsePredictionIds;        ///< The ids _sparsePredictionColumns is indexed by.
	std::map<int, std::map<int, int> > _neighborsIndex; ///< Cached neighbor margins per signature id.
};

} // namespace rtabmap

#endif /* BAYESFILTER_H_ */
