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

#include "rtabmap/core/BayesFilter.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Parameters.h"
#include <iostream>
#include <set>
#if __cplusplus >= 201103L
#include <unordered_map>
#include <unordered_set>
#endif

#include "rtabmap/utilite/UtiLite.h"

namespace rtabmap {

#if __cplusplus >= 201103L
typedef std::unordered_map<int, int> IdToIndexMap;
#else
typedef std::map<int, int> IdToIndexMap;
#endif

BayesFilter::BayesFilter(const ParametersMap & parameters) :
	_virtualPlacePrior(Parameters::defaultBayesVirtualPlacePriorThr()),
	_fullPredictionUpdate(Parameters::defaultBayesFullPredictionUpdate()),
	_totalPredictionLCValues(0.0f),
	_predictionEpsilon(0.0f),
	_sparsePrediction(Parameters::defaultBayesSparsePrediction()),
	_predictionChanged(true)
{
	this->setPredictionLC(Parameters::defaultBayesPredictionLC());
	this->parseParameters(parameters);
}

BayesFilter::~BayesFilter() {
}

void BayesFilter::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kBayesPredictionLC())) != parameters.end())
	{
		this->setPredictionLC((*iter).second);
	}
	Parameters::parse(parameters, Parameters::kBayesVirtualPlacePriorThr(), _virtualPlacePrior);
	Parameters::parse(parameters, Parameters::kBayesFullPredictionUpdate(), _fullPredictionUpdate);
	if(Parameters::parse(parameters, Parameters::kBayesSparsePrediction(), _sparsePrediction))
	{
		// The sparse view is rebuilt on the next posterior if it was just enabled, and
		// released if it was just disabled.
		_predictionChanged = true;
		if(!_sparsePrediction)
		{
			this->clearSparsePrediction();
		}
	}

	UASSERT(_virtualPlacePrior >= 0 && _virtualPlacePrior <= 1.0f);
}

// format = {Virtual place, Loop closure, level1, level2, l3, l4...}
void BayesFilter::setPredictionLC(const std::string & prediction)
{
	std::list<std::string> strValues = uSplit(prediction, ' ');
	if(strValues.size() < 2)
	{
		UERROR("The number of values < 2 (prediction=\"%s\")", prediction.c_str());
	}
	else
	{
		std::vector<double> tmpValues(strValues.size());
		int i=0;
		bool valid = true;
		for(std::list<std::string>::iterator iter = strValues.begin(); iter!=strValues.end(); ++iter)
		{
			tmpValues[i] = uStr2Float((*iter).c_str());
			//UINFO("%d=%e", i, tmpValues[i]);
			if(tmpValues[i] < 0.0 || tmpValues[i]>1.0)
			{
				valid = false;
				break;
			}
			++i;
		}

		if(!valid)
		{
			UERROR("The prediction is not valid (values must be between >0 && <=1) prediction=\"%s\"", prediction.c_str());
		}
		else
		{
			_predictionLC = tmpValues;
		}
	}
	_totalPredictionLCValues = 0.0f;
	for(unsigned int j=0; j<_predictionLC.size(); ++j)
	{
		_totalPredictionLCValues += _predictionLC[j];
		if(j==0 || _predictionLC[j] < _predictionEpsilon)
		{
			_predictionEpsilon = _predictionLC[j];
		}
	}
	if(!_predictionLC.empty())
	{
		UDEBUG("predictionEpsilon = %f", _predictionEpsilon);
	}
	// A new model changes the values and the sparsity of the prediction matrix.
	_predictionChanged = true;
}

const std::vector<double> & BayesFilter::getPredictionLC() const
{
	// {Vp, Lc, l1, l2, l3, l4...}
	return _predictionLC;
}

std::string BayesFilter::getPredictionLCStr() const
{
	std::string values;
	for(unsigned int i=0; i<_predictionLC.size(); ++i)
	{
		values.append(uNumber2Str(_predictionLC[i]));
		if(i+1 < _predictionLC.size())
		{
			values.append(" ");
		}
	}
	return values;
}

// Whether the posterior is indexed by exactly these ids, in this order. Both are
// sorted by id, so they are compared side by side rather than collecting the keys of
// the posterior into a vector of their own to compare with.
bool BayesFilter::posteriorHasSameIds(const std::vector<int> & ids) const
{
	if(_posterior.size() != ids.size())
	{
		return false;
	}
	std::map<int, float>::const_iterator iter = _posterior.begin();
	for(size_t i=0; i<ids.size(); ++i, ++iter)
	{
		if(iter->first != ids[i])
		{
			return false;
		}
	}
	return true;
}

void BayesFilter::reset()
{
	_posterior.clear();
	_prediction = cv::Mat();
	this->clearSparsePrediction();
	_predictionChanged = true;
	_neighborsIndex.clear();
}

const std::map<int, float> & BayesFilter::computePosterior(const Memory * memory, const std::map<int, float> & likelihood)
{
	ULOGGER_DEBUG("");

	if(!memory)
	{
		ULOGGER_ERROR("Memory is Null!");
		return _posterior;
	}

	if(!likelihood.size())
	{
		ULOGGER_ERROR("likelihood is empty!");
		return _posterior;
	}

	if(_predictionLC.size() < 2)
	{
		ULOGGER_ERROR("Prediction is not valid!");
		return _posterior;
	}

	UTimer timer;
	timer.start();

	cv::Mat prior;
	cv::Mat posterior;

	float sum = 0;
	int j=0;

	// The ids of the likelihood, which both the prediction and the posterior are
	// indexed by, taken once: there are as many of them as there are locations in the
	// working memory, and walking the map to collect them is not free at that size.
	const std::vector<int> ids = uKeys(likelihood);

	// Recursive Bayes estimation...
	// STEP 1 - Prediction : Prior*lastPosterior
	//
	// The prediction is built in its sparse form only, the matrix never being
	// allocated, when the graph is fixed: in localization mode there is no incremental
	// update of the matrix to carry columns over, so nothing else needs it. While
	// mapping, the matrix is built as before and the sparse form is taken from it.
	const bool buildSparseDirectly =
			_sparsePrediction &&
			_totalPredictionLCValues >= 1 &&
			!memory->isIncremental();
	bool sparseBuilt = false;
	if(buildSparseDirectly)
	{
		if(_predictionChanged || _sparsePredictionMatrix.rows() != (int)ids.size() ||
			!this->posteriorHasSameIds(ids))
		{
			sparseBuilt = this->generateSparsePrediction(memory, ids);
		}
		else
		{
			sparseBuilt = true;
		}
		UDEBUG("STEP1-generate prior=%fs, rows=%d, cols=%d", timer.ticks(),
				(int)_sparsePredictionMatrix.rows(), (int)_sparsePredictionMatrix.cols());
	}
	if(!sparseBuilt)
	{
		_prediction = this->generatePrediction(memory, ids);
		UDEBUG("STEP1-generate prior=%fs, rows=%d, cols=%d", timer.ticks(), _prediction.rows, _prediction.cols);
		//std::cout << "Prediction=" << _prediction << std::endl;

		if(_sparsePrediction && _predictionChanged)
		{
			// Only when the matrix has changed: over a fixed graph this happens once.
			this->updateSparsePredictionFromDense();
			UDEBUG("STEP1-sparse prediction update time=%fs", timer.ticks());
		}
	}

	// Adjust the last posterior if some images were
	// reactivated or removed from the working memory
	posterior = cv::Mat(likelihood.size(), 1, CV_32FC1);
	this->updatePosterior(memory, ids);
	j=0;
	for(std::map<int, float>::const_iterator i=_posterior.begin(); i!= _posterior.end(); ++i)
	{
		((float*)posterior.data)[j++] = (*i).second;
	}
	ULOGGER_DEBUG("STEP1-update posterior=%fs, posterior rows=%d, _posterior size=%d", timer.ticks(), posterior.rows, (int)_posterior.size());
	//std::cout << "LastPosterior=" << posterior << std::endl;

	// Multiply prediction matrix with the last posterior
	// (m,m) X (m,1) = (m,1)
	// The sparse form is empty when disabled, or when the prediction was found too
	// dense for it to be worth it.
	const bool sparse = _sparsePrediction && _sparsePredictionMatrix.rows() > 0;
	if(sparse)
	{
		this->multiplySparsePrediction(posterior, prior);
	}
	else
	{
		prior = _prediction * posterior;
	}
	ULOGGER_DEBUG("STEP1-matrix mult time=%fs (sparse=%d)", timer.ticks(), sparse?1:0);
	//std::cout << "ResultingPrior=" << prior << std::endl;

	// STEP 2 - Update : Multiply with observations (likelihood)
	// The posterior holds the ids of the likelihood, updatePosterior() having just made
	// sure of it, and both are sorted by id: they are walked side by side instead of
	// looking up each id in the posterior, which at the size of the working memory is
	// as many searches through the map.
	j=0;
	std::map<int, float>::iterator p = _posterior.begin();
	for(std::map<int, float>::const_iterator i=likelihood.begin(); i!= likelihood.end(); ++i)
	{
		if(p == _posterior.end() || p->first != (*i).first)
		{
			// Should not happen. Searched for rather than assumed if it ever does.
			p = _posterior.find((*i).first);
			if(p == _posterior.end())
			{
				ULOGGER_ERROR("Problem1! can't find id=%d", (*i).first);
				continue;
			}
		}
		p->second = (*i).second * ((float*)prior.data)[j++];
		sum += p->second;
		++p;
	}
	ULOGGER_DEBUG("STEP2-likelihood time=%fs", timer.ticks());
	//std::cout << "Posterior (before normalization)=" << _posterior << std::endl;

	// Normalize
	ULOGGER_DEBUG("sum=%f", sum);
	if(sum != 0)
	{
		for(std::map<int, float>::iterator i=_posterior.begin(); i!= _posterior.end(); ++i)
		{
			(*i).second /= sum;
		}
	}
	ULOGGER_DEBUG("normalize time=%fs", timer.ticks());
	//std::cout << "Posterior=" << _posterior << std::endl;

	return _posterior;
}

// A column of the prediction matrix, given as a pointer to its first value and the
// step between two of them: the matrix stores a column strided by its width, while the
// sparse build below fills one contiguous column at a time. Both go through this and
// through BayesFilter::normalize(), so that the probabilities cannot end up differing
// between the two.
float addNeighborProb(float * column,
			size_t stride,
			const std::map<int, int> & neighbors,
			const std::vector<double> & predictionLC,
			const IdToIndexMap & idToIndex)
{
	float sum=0.0f;
	for(std::map<int, int>::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
	{
		if(iter->first>=0)
		{
			IdToIndexMap::const_iterator jter = idToIndex.find(iter->first);
			if(jter != idToIndex.end())
			{
				UASSERT((iter->second+1) < (int)predictionLC.size());
				sum += column[jter->second*stride] = predictionLC[iter->second+1];
			}
		}
	}
	return sum;
}

// The neighbors of a location within the depth of the prediction model, and the
// locations that are at margin 0 of it, meaning the same place: their columns all hold
// the probabilities of this same neighborhood. Shared by the dense and the sparse
// builds, this being the part that reads the graph.
//
// neighborsIndex is filled when not null, for updatePrediction() to reuse.
std::map<int, int> resolveNeighbors(
		const Memory * memory,
		int id,
		int maxDepth,
		const IdToIndexMap & idToIndexMap,
		std::list<int> & idsAtMargin0,
		std::map<int, std::map<int, int> > * neighborsIndex)
{
	std::map<int, int> neighbors = memory->getNeighborsId(id, maxDepth, 0, false, false, true, true);

	if(neighborsIndex)
	{
		uInsert(*neighborsIndex, std::make_pair(id, neighbors));
	}

	idsAtMargin0.clear();
	//filter neighbors in STM
	for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
	{
		if(memory->isInSTM(iter->first))
		{
			neighbors.erase(iter++);
		}
		else
		{
			if(iter->second == 0 && idToIndexMap.find(iter->first)!=idToIndexMap.end())
			{
				idsAtMargin0.push_back(iter->first);
			}
			++iter;
		}
	}

	// should at least have 1 id in idsMarginLoop
	if(idsAtMargin0.size() == 0)
	{
		UFATAL("No 0 margin neighbor for signature %d !?!?", id);
	}
	return neighbors;
}

cv::Mat BayesFilter::generatePrediction(const Memory * memory, const std::vector<int> & ids)
{
	if(this->posteriorHasSameIds(ids))
	{
		return _prediction;
	}
	std::vector<int> oldIds = uKeys(_posterior);

	// Both paths below return a newly built matrix, so the sparse view of the
	// previous one no longer applies.
	_predictionChanged = true;

	if(!_fullPredictionUpdate && !_prediction.empty())
	{
		return updatePrediction(_prediction, memory, oldIds, ids);
	}
	UDEBUG("");

	UASSERT(memory &&
		   _predictionLC.size() >= 2 &&
		   ids.size());

	UTimer timer;
	timer.start();
	UTimer timerGlobal;
	timerGlobal.start();

	IdToIndexMap idToIndexMap;
#if __cplusplus >= 201103L
	idToIndexMap.reserve(ids.size());
#endif
	for(unsigned int i=0; i<ids.size(); ++i)
	{
		if(ids[i]>0)
		{
			idToIndexMap[ids[i]] = i;
		}
	}


	//int rows = prediction.rows;
	cv::Mat prediction = cv::Mat::zeros(ids.size(), ids.size(), CV_32FC1);
	int cols = prediction.cols;

	// Each prior is a column vector
	UDEBUG("_predictionLC.size()=%d",(int)_predictionLC.size());
	std::set<int> idsDone;

	for(unsigned int i=0; i<ids.size(); ++i)
	{
		if(idsDone.find(ids[i]) == idsDone.end())
		{
			if(ids[i] > 0)
			{
				// Set high values (gaussians curves) to loop closure neighbors
				std::list<int> idsLoopMargin;
				std::map<int, int> neighbors = resolveNeighbors(
						memory, ids[i], _predictionLC.size()-1, idToIndexMap, idsLoopMargin,
						_fullPredictionUpdate?0:&_neighborsIndex);

				// same neighbor tree for loop signatures (margin = 0)
				for(std::list<int>::iterator iter = idsLoopMargin.begin(); iter!=idsLoopMargin.end(); ++iter)
				{
					if(!_fullPredictionUpdate)
					{
						uInsert(_neighborsIndex, std::make_pair(*iter, neighbors));
					}

					float sum = 0.0f; // sum values added
					int index = idToIndexMap.at(*iter);
					float * column = (float*)prediction.data + index;
					sum += addNeighborProb(column, cols, neighbors, _predictionLC, idToIndexMap);
					idsDone.insert(*iter);
					this->normalize(column, cols, cols, index, sum, ids[0]<0);
				}
			}
			else
			{
				// Set the virtual place prior
				this->fillVirtualPlaceColumn((float*)prediction.data + i, cols, cols);
			}
		}
	}

	ULOGGER_DEBUG("time = %fs", timerGlobal.ticks());

	return prediction;
}

// Appends the non zero values of a freshly built column to the triplets of the sparse
// prediction, and leaves the buffer zeroed for the next one, which saves clearing the
// whole of it every time.
void BayesFilter::appendSparseColumn(
		std::vector<float> & column,
		int index,
		std::vector<Eigen::Triplet<float> > & triplets) const
{
	for(size_t row=0; row<column.size(); ++row)
	{
		if(column[row] != 0.0f)
		{
			triplets.push_back(Eigen::Triplet<float>((int)row, index, column[row]));
			column[row] = 0.0f;
		}
	}
}

// The prediction built directly in its sparse form, the matrix never being allocated.
//
// A column of the prediction only holds the neighbors of one location within the depth
// of the prediction model, so on a large map the matrix is mostly zeros, while holding
// it costs the size of the working memory squared against the far smaller size of the
// values in it. Each column is built in a buffer of its own instead, through the same
// addNeighborProb() and normalize() as the dense build, and only its non zero values
// are kept.
//
// The columns are not built in the order of their index: a column is built for every
// location at margin 0 of the one being expanded, so several are built at once. Eigen
// takes them as triplets and orders them.
//
// Returns false when the matrix would not be sparse, which the caller has to answer by
// building the dense one. Only reachable through a model whose values sum to less than
// 1, as normalize() then spreads the missing probability over every zero of a column.
bool BayesFilter::generateSparsePrediction(const Memory * memory, const std::vector<int> & ids)
{
	UASSERT(memory && _predictionLC.size() >= 2 && ids.size());

	UTimer timer;
	this->clearSparsePrediction();
	_predictionChanged = false;

	const int size = (int)ids.size();

	IdToIndexMap idToIndexMap;
#if __cplusplus >= 201103L
	idToIndexMap.reserve(ids.size());
#endif
	for(int i=0; i<size; ++i)
	{
		if(ids[i]>0)
		{
			idToIndexMap[ids[i]] = i;
		}
	}

	// A value costs 12 bytes as a triplet and 8 in the matrix, against the 4 of the
	// dense one, so past a quarter filled the sparse form is not worth building.
	const size_t maxValues = (size_t)size*(size_t)size/4;
	std::vector<float> column(size, 0.0f);
	std::vector<Eigen::Triplet<float> > triplets;

	std::set<int> idsDone;
	for(int i=0; i<size; ++i)
	{
		if(idsDone.find(ids[i]) != idsDone.end())
		{
			continue;
		}

		if(ids[i] > 0)
		{
			std::list<int> idsLoopMargin;
			// The neighborhoods are not kept in _neighborsIndex: it is there for the
			// incremental updatePrediction(), which this build replaces, and it holds
			// one neighborhood per location, as much memory again as the values here.
			std::map<int, int> neighbors = resolveNeighbors(
					memory, ids[i], _predictionLC.size()-1, idToIndexMap, idsLoopMargin, 0);

			// same neighbor tree for loop signatures (margin = 0)
			for(std::list<int>::iterator iter=idsLoopMargin.begin(); iter!=idsLoopMargin.end(); ++iter)
			{
				const int index = idToIndexMap.at(*iter);
				float sum = addNeighborProb(&column[0], 1, neighbors, _predictionLC, idToIndexMap);
				idsDone.insert(*iter);
				this->normalize(&column[0], 1, size, index, sum, ids[0]<0);
				this->appendSparseColumn(column, index, triplets);
			}
		}
		else
		{
			this->fillVirtualPlaceColumn(&column[0], 1, size);
			this->appendSparseColumn(column, i, triplets);
		}

		if(triplets.size() > maxValues)
		{
			UWARN("The prediction has more than %ld non zero values, which is too dense "
				  "for %s to be worth it (%d of %d locations done). Building the matrix "
				  "instead. The values of %s summing to less than 1 is what fills it.",
				  (long)maxValues, Parameters::kBayesSparsePrediction().c_str(), i+1, size,
				  Parameters::kBayesPredictionLC().c_str());
			return false;
		}
	}

	_sparsePredictionMatrix.resize(size, size);
	_sparsePredictionMatrix.setFromTriplets(triplets.begin(), triplets.end());

	UDEBUG("Sparse prediction: %ld/%ld values (%.2f%%), %ld MB against the %ld MB of the "
		   "matrix, built in %fs",
			(long)_sparsePredictionMatrix.nonZeros(), (long)size*size,
			100.0*double(_sparsePredictionMatrix.nonZeros())/(double(size)*double(size)),
			(long)(this->getSparsePredictionMemoryUsed()/1048576),
			(long)((size_t)size*(size_t)size*sizeof(float)/1048576),
			timer.ticks());
	return true;
}

// The same sparse prediction, but taken from the matrix rather than built instead of
// it. Used when the matrix is there anyway, which is the case while mapping: the
// incremental updatePrediction() needs it to carry the unchanged columns over.
void BayesFilter::updateSparsePredictionFromDense()
{
	this->clearSparsePrediction();
	_predictionChanged = false;

	if(_prediction.empty())
	{
		return;
	}
	UASSERT(_prediction.type() == CV_32FC1);
	UASSERT(_prediction.isContinuous());

	// normalize() spreads the probability that the model doesn't account for over every
	// zero of a column, so with such a model there is no zero left to skip. The
	// condition is the one used there, so that both agree on the matrix content.
	if(_totalPredictionLCValues < 1)
	{
		UWARN("%s is enabled but the values of %s sum to %f < 1, so the missing "
			  "probability is spread over all the other locations and the prediction "
			  "matrix has no zeros to skip. Using the dense multiplication instead. "
			  "Make the values sum to 1 to benefit from the sparse one.",
			  Parameters::kBayesSparsePrediction().c_str(),
			  Parameters::kBayesPredictionLC().c_str(),
			  _totalPredictionLCValues);
		return;
	}

	UTimer timer;
	const int rows = _prediction.rows;
	const int cols = _prediction.cols;
	const float * data = (const float *)_prediction.data;
	const size_t maxValues = (size_t)rows*(size_t)cols/4;

	// Read in the order the matrix is stored in, a column of it being strided.
	std::vector<Eigen::Triplet<float> > triplets;
	for(int row=0; row<rows; ++row)
	{
		const float * rowPtr = data + (size_t)row*cols;
		for(int col=0; col<cols; ++col)
		{
			if(rowPtr[col] != 0.0f)
			{
				triplets.push_back(Eigen::Triplet<float>(row, col, rowPtr[col]));
			}
		}
		if(triplets.size() > maxValues)
		{
			UWARN("The prediction matrix has more than %ld non zero values, which is too "
				  "dense for %s to be worth it (%d of %d rows scanned). Using the dense "
				  "multiplication instead.",
				  (long)maxValues, Parameters::kBayesSparsePrediction().c_str(), row+1, rows);
			return;
		}
	}

	_sparsePredictionMatrix.resize(rows, cols);
	_sparsePredictionMatrix.setFromTriplets(triplets.begin(), triplets.end());

	UDEBUG("Sparse prediction: %ld/%ld values (%.2f%%), %ld MB on top of the %ld MB of "
		   "the matrix, built in %fs",
			(long)_sparsePredictionMatrix.nonZeros(), (long)rows*cols,
			100.0*double(_sparsePredictionMatrix.nonZeros())/(double(rows)*double(cols)),
			(long)(this->getSparsePredictionMemoryUsed()/1048576),
			(long)(_prediction.total()*_prediction.elemSize()/1048576),
			timer.ticks());
}

void BayesFilter::clearSparsePrediction()
{
	// Assigned rather than resized: resize(0,0) keeps the buffers it has allocated.
	_sparsePredictionMatrix = Eigen::SparseMatrix<float, Eigen::RowMajor>();
}

unsigned long BayesFilter::getSparsePredictionMemoryUsed() const
{
	typedef Eigen::SparseMatrix<float, Eigen::RowMajor>::StorageIndex StorageIndex;
	return _sparsePredictionMatrix.nonZeros() * (sizeof(float)+sizeof(StorageIndex))
			+ (_sparsePredictionMatrix.outerSize()+1) * sizeof(StorageIndex);
}

void BayesFilter::multiplySparsePrediction(const cv::Mat & posterior, cv::Mat & prior) const
{
	UASSERT(_sparsePredictionMatrix.rows() > 0);
	UASSERT(posterior.cols == 1 && posterior.type() == CV_32FC1);
	UASSERT_MSG(posterior.rows == (int)_sparsePredictionMatrix.cols(),
			uFormat("posterior=%d prediction=%d", posterior.rows,
					(int)_sparsePredictionMatrix.cols()).c_str());

	prior = cv::Mat((int)_sparsePredictionMatrix.rows(), 1, CV_32FC1);
	Eigen::Map<const Eigen::VectorXf> posteriorVector((const float *)posterior.data, posterior.rows);
	Eigen::Map<Eigen::VectorXf> priorVector((float *)prior.data, prior.rows);
	priorVector.noalias() = _sparsePredictionMatrix * posteriorVector;
}

unsigned long BayesFilter::getMemoryUsed() const
{
	long memoryUsage = sizeof(BayesFilter);
	memoryUsage += _posterior.size() * (sizeof(float)+sizeof(int)+sizeof(std::map<int, float>::iterator)) + sizeof(std::map<int, float>);
	if(!_prediction.empty())
	{
		memoryUsage += _prediction.total() * _prediction.elemSize();
	}
	memoryUsage += _predictionLC.size() * sizeof(double);
	memoryUsage += this->getSparsePredictionMemoryUsed();
	memoryUsage += _neighborsIndex.size() * (sizeof(int)+sizeof(std::map<int, int>)+sizeof(std::map<int, std::map<int, int> >::iterator)) + sizeof(std::map<int, std::map<int, int> >);
	for(std::map<int, std::map<int, int> >::const_iterator iter=_neighborsIndex.begin(); iter!=_neighborsIndex.end(); ++iter)
	{
		memoryUsage += iter->second.size() * (sizeof(int)*2+sizeof(std::map<int, int>::iterator)) + sizeof(std::map<int, int>);
	}
	return memoryUsage;
}

// The column of the virtual place, the hypothesis of being at a location that was
// never visited: the probability of moving again to a new one, then the rest split
// equally over the visited ones.
void BayesFilter::fillVirtualPlaceColumn(float * column, size_t stride, int size) const
{
	if(_virtualPlacePrior > 0)
	{
		if(size>1) // The first must be the virtual place
		{
			column[0] = _virtualPlacePrior;
			float val = (1.0-_virtualPlacePrior)/(size-1);
			for(int j=1; j<size; ++j)
			{
				column[j*stride] = val;
			}
		}
		else if(size>0)
		{
			column[0] = 1;
		}
	}
	else
	{
		// Only for some tests...
		// when _virtualPlacePrior=0, set all priors to the same value
		if(size>1)
		{
			float val = 1.0/size;
			for(int j=0; j<size; ++j)
			{
				column[j*stride] = val;
			}
		}
		else if(size>0)
		{
			column[0] = 1;
		}
	}
}

void BayesFilter::normalize(float * column, size_t stride, int size, unsigned int index, float addedProbabilitiesSum, bool virtualPlaceUsed) const
{
	UASSERT(index < (unsigned int)size);

	int cols = size;
	// ADD values of not found neighbors to loop closure
	if(addedProbabilitiesSum < _totalPredictionLCValues-_predictionLC[0])
	{
		float delta = _totalPredictionLCValues-_predictionLC[0]-addedProbabilitiesSum;
		column[index*stride] += delta;
		addedProbabilitiesSum+=delta;
	}

	float allOtherPlacesValue = 0;
	if(_totalPredictionLCValues < 1)
	{
		allOtherPlacesValue = 1.0f - _totalPredictionLCValues;
	}

	// Set all loop events to small values according to the model
	if(allOtherPlacesValue > 0 && cols>1)
	{
		float value = allOtherPlacesValue / float(cols - 1);
		for(int j=virtualPlaceUsed?1:0; j<cols; ++j)
		{
			if(column[j*stride] == 0)
			{
				column[j*stride] = value;
				addedProbabilitiesSum += column[j*stride];
			}
		}
	}

	//normalize this row
	float maxNorm = 1 - (virtualPlaceUsed?_predictionLC[0]:0); // 1 - virtual place probability
	if(addedProbabilitiesSum<maxNorm-0.0001 || addedProbabilitiesSum>maxNorm+0.0001)
	{
		for(int j=virtualPlaceUsed?1:0; j<cols; ++j)
		{
			column[j*stride] *= maxNorm / addedProbabilitiesSum;
			if(column[j*stride] < _predictionEpsilon)
			{
				column[j*stride] = 0.0f;
			}
		}
		addedProbabilitiesSum = maxNorm;
	}

	// ADD virtual place prob
	if(virtualPlaceUsed)
	{
		column[0] = _predictionLC[0];
		addedProbabilitiesSum += column[0];
	}

	//debug
	//for(int j=0; j<cols; ++j)
	//{
	//	ULOGGER_DEBUG("test col=%d = %f", i, prediction.data.fl[i + j*cols]);
	//}

	if(addedProbabilitiesSum<0.99 || addedProbabilitiesSum > 1.01)
	{
		UWARN("Prediction is not normalized sum=%f", addedProbabilitiesSum);
	}
}

cv::Mat BayesFilter::updatePrediction(const cv::Mat & oldPrediction,
		const Memory * memory,
		const std::vector<int> & oldIds,
		const std::vector<int> & newIds)
{
	UTimer timer;
	UDEBUG("");

	UASSERT(memory &&
		oldIds.size() &&
		newIds.size() &&
		oldIds.size() == (unsigned int)oldPrediction.cols &&
		oldIds.size() == (unsigned int)oldPrediction.rows);

	cv::Mat prediction = cv::Mat::zeros(newIds.size(), newIds.size(), CV_32FC1);
	UDEBUG("time creating prediction = %fs", timer.restart());

	// Create id to index maps
#if __cplusplus >= 201103L
	std::unordered_set<int> oldIdsSet(oldIds.begin(), oldIds.end());
#else
	std::set<int> oldIdsSet(oldIds.begin(), oldIds.end());
#endif
	UDEBUG("time creating old ids set = %fs", timer.restart());

	IdToIndexMap newIdToIndexMap;
#if __cplusplus >= 201103L
	newIdToIndexMap.reserve(newIds.size());
#endif
	for(unsigned int i=0; i<newIds.size(); ++i)
	{
		if(newIds[i]>0)
		{
			newIdToIndexMap[newIds[i]] = i;
		}
	}

	UDEBUG("time creating id-index vector (size=%d oldIds.back()=%d newIds.back()=%d) = %fs", (int)newIdToIndexMap.size(), oldIds.back(), newIds.back(), timer.restart());

	//Get removed ids
	std::set<int> removedIds;
	for(unsigned int i=0; i<oldIds.size(); ++i)
	{
		if(oldIds[i] > 0 && newIdToIndexMap.find(oldIds[i]) == newIdToIndexMap.end())
		{
			removedIds.insert(removedIds.end(), oldIds[i]);
			_neighborsIndex.erase(oldIds[i]);
			UDEBUG("removed id=%d at oldIndex=%d", oldIds[i], i);
		}
	}
	UDEBUG("time getting removed ids = %fs", timer.restart());

	bool oldAllCopied = false;
	if(removedIds.empty() &&
		newIds.size() > oldIds.size() &&
		memcmp(oldIds.data(), newIds.data(), oldIds.size()*sizeof(int)) == 0)
	{
		oldPrediction.copyTo(cv::Mat(prediction, cv::Range(0, oldPrediction.rows), cv::Range(0, oldPrediction.cols)));
		oldAllCopied = true;
		UDEBUG("Copied all old prediction: = %fs", timer.ticks());
	}

	int added = 0;
	// get ids to update
	std::set<int> idsToUpdate;
	for(unsigned int i=0; i<oldIds.size() || i<newIds.size(); ++i)
	{
		if(i<oldIds.size())
		{
			if(removedIds.find(oldIds[i]) != removedIds.end())
			{
				unsigned int cols = oldPrediction.cols;
				int count = 0;
				for(unsigned int j=0; j<cols; ++j)
				{
					if(j!=i && removedIds.find(oldIds[j]) == removedIds.end())
					{
						//UDEBUG("to update id=%d from id=%d removed (value=%f)", oldIds[j], oldIds[i], ((const float *)oldPrediction.data)[i + j*cols]);
						idsToUpdate.insert(oldIds[j]);
						++count;
					}
				}
				UDEBUG("From removed id %d, %d neighbors to update.", oldIds[i], count);
			}
		}
		if(i<newIds.size() && oldIdsSet.find(newIds[i]) == oldIdsSet.end())
		{
			if(_neighborsIndex.find(newIds[i]) == _neighborsIndex.end())
			{
				std::map<int, int> neighbors = memory->getNeighborsId(newIds[i], _predictionLC.size()-1, 0, false, false, true, true);

				for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
				{
					std::map<int, std::map<int, int> >::iterator jter = _neighborsIndex.find(iter->first);
					if(jter != _neighborsIndex.end())
					{
						uInsert(jter->second, std::make_pair(newIds[i], iter->second));
					}
				}
				_neighborsIndex.insert(std::make_pair(newIds[i], neighbors));
			}
			const std::map<int, int> & neighbors = _neighborsIndex.at(newIds[i]);
			//std::map<int, int> neighbors = memory->getNeighborsId(newIds[i], _predictionLC.size()-1, 0, false, false, true, true);

			float * column = (float*)prediction.data + i;
			float sum = addNeighborProb(column, prediction.cols, neighbors, _predictionLC, newIdToIndexMap);
			this->normalize(column, prediction.cols, prediction.cols, i, sum, newIds[0]<0);

			++added;
			int count = 0;
			for(std::map<int,int>::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				if(oldIdsSet.find(iter->first)!=oldIdsSet.end() &&
				   removedIds.find(iter->first) == removedIds.end())
				{
					idsToUpdate.insert(iter->first);
					++count;
				}
			}
			UDEBUG("From added id %d, %d neighbors to update.", newIds[i], count);
		}
	}
	UDEBUG("time getting %d ids to update = %fs", (int)idsToUpdate.size(), timer.restart());

	UTimer t1;
	double e0=0,e1=0, e2=0, e3=0, e4=0;
	// update modified/added ids
	int modified = 0;
	for(std::set<int>::iterator iter = idsToUpdate.begin(); iter!=idsToUpdate.end(); ++iter)
	{
		int id = *iter;
		if(id > 0)
		{
			int index = newIdToIndexMap.at(id);

			e0 = t1.ticks();
			std::map<int, std::map<int, int> >::iterator kter = _neighborsIndex.find(id);
			UASSERT_MSG(kter != _neighborsIndex.end(), uFormat("Did not find %d (current index size=%d)", id, (int)_neighborsIndex.size()).c_str());
			const std::map<int, int> & neighbors = kter->second;
			//std::map<int, int> neighbors = memory->getNeighborsId(id, _predictionLC.size()-1, 0, false, false, true, true);
			e1+=t1.ticks();

			float * column = (float*)prediction.data + index;
			float sum = addNeighborProb(column, prediction.cols, neighbors, _predictionLC, newIdToIndexMap);
			e3+=t1.ticks();

			this->normalize(column, prediction.cols, prediction.cols, index, sum, newIds[0]<0);
			++modified;
			e4+=t1.ticks();
		}
	}
	UDEBUG("time updating modified/added %d ids = %fs (e0=%f e1=%f e2=%f e3=%f e4=%f)", (int)idsToUpdate.size(), timer.restart(), e0, e1, e2, e3, e4);

	int copied = 0;
	if(!oldAllCopied)
	{
		//UDEBUG("oldIds.size()=%d, oldPrediction.cols=%d, oldPrediction.rows=%d", oldIds.size(), oldPrediction.cols, oldPrediction.rows);
		//UDEBUG("newIdToIndexMap.size()=%d, prediction.cols=%d, prediction.rows=%d", newIdToIndexMap.size(), prediction.cols, prediction.rows);
		// copy not changed probabilities
		for(unsigned int i=0; i<oldIds.size(); ++i)
		{
			if(oldIds[i]>0 && removedIds.find(oldIds[i]) == removedIds.end() && idsToUpdate.find(oldIds[i]) == idsToUpdate.end())
			{
				for(int j=0; j<oldPrediction.cols; ++j)
				{
					if(oldIds[j]>0 && removedIds.find(oldIds[j]) == removedIds.end())
					{
						//UDEBUG("i=%d, j=%d", i, j);
						//UDEBUG("oldIds[i]=%d, oldIds[j]=%d", oldIds[i], oldIds[j]);
						//UDEBUG("newIdToIndexMap.at(oldIds[i])=%d", newIdToIndexMap.at(oldIds[i]));
						//UDEBUG("newIdToIndexMap.at(oldIds[j])=%d", newIdToIndexMap.at(oldIds[j]));
						float v = ((const float *)oldPrediction.data)[i + j*oldPrediction.cols];
						int ii = newIdToIndexMap.at(oldIds[i]);
						int jj = newIdToIndexMap.at(oldIds[j]);
						((float *)prediction.data)[ii + jj*prediction.cols] = v;
						//if(ii != jj)
						//{
						//	((float *)prediction.data)[jj + ii*prediction.cols] = v;
						//}
					}
				}
				++copied;
			}
		}
		UDEBUG("time copying = %fs", timer.restart());
	}

	//update virtual place
	if(newIds[0] < 0)
	{
		if(prediction.cols>1) // The first must be the virtual place
		{
			((float*)prediction.data)[0] = _virtualPlacePrior;
			float val = (1.0-_virtualPlacePrior)/(prediction.cols-1);
			for(int j=1; j<prediction.cols; j++)
			{
				((float*)prediction.data)[j*prediction.cols] = val;
				((float*)prediction.data)[j] = _predictionLC[0];
			}
		}
		else if(prediction.cols>0)
		{
			((float*)prediction.data)[0] = 1;
		}
	}
	UDEBUG("time updating virtual place = %fs", timer.restart());

	UDEBUG("Modified=%d, Added=%d, Copied=%d", modified, added, copied);
	return prediction;
}

void BayesFilter::updatePosterior(const Memory * memory, const std::vector<int> & likelihoodIds)
{
	ULOGGER_DEBUG("");
	if(this->posteriorHasSameIds(likelihoodIds))
	{
		// Nothing was added to or removed from the working memory, which over a fixed
		// graph is every iteration: the map below would be rebuilt identical, at the
		// cost of allocating and freeing a node per location.
		return;
	}
	std::map<int, float> newPosterior;
	for(std::vector<int>::const_iterator i=likelihoodIds.begin(); i != likelihoodIds.end(); ++i)
	{
		std::map<int, float>::iterator post = _posterior.find(*i);
		if(post == _posterior.end())
		{
			if(_posterior.size() == 0)
			{
				newPosterior.insert(std::pair<int, float>(*i, 1));
			}
			else
			{
				newPosterior.insert(std::pair<int, float>(*i, 0));
			}
		}
		else
		{
			newPosterior.insert(std::pair<int, float>((*post).first, (*post).second));
		}
	}
	_posterior = newPosterior;
}

} // namespace rtabmap
