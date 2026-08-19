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
	_predictionChanged(true),
	_sparsePredictionRejected(false)
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
		_sparsePredictionRejected = false;
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
	_sparsePredictionRejected = false;
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

// Whether the posterior is indexed by exactly these ids, in this order.
bool BayesFilter::posteriorHasSameIds(const std::vector<int> & ids) const
{
	return _posteriorIds == ids;
}

void BayesFilter::reset()
{
	_posteriorIds.clear();
	_posteriorValues.clear();
	_prediction = cv::Mat();
	this->clearSparsePrediction();
	_predictionChanged = true;
	_sparsePredictionRejected = false;
	_neighborsIndex.clear();
}

bool BayesFilter::computePosterior(const Memory * memory, const std::map<int, float> & likelihood)
{
	ULOGGER_DEBUG("");

	if(!memory)
	{
		ULOGGER_ERROR("Memory is Null!");
		return false;
	}

	if(!likelihood.size())
	{
		ULOGGER_ERROR("likelihood is empty!");
		return false;
	}

	if(_predictionLC.size() < 2)
	{
		ULOGGER_ERROR("Prediction is not valid!");
		return false;
	}

	UTimer timer;
	timer.start();

	// One walk of the likelihood: its values into a vector of their own, and its ids
	// against the ones the posterior is indexed by. Everything below then works on
	// vectors, which at the size of the working memory is the difference between walking a
	// tree of tens of thousands of nodes once and walking it half a dozen times.
	_likelihoodIds.resize(likelihood.size());
	_likelihoodValues.resize(likelihood.size());
	bool sameIds = _posteriorIds.size() == likelihood.size();
	{
		size_t k = 0;
		for(std::map<int, float>::const_iterator iter=likelihood.begin(); iter!=likelihood.end(); ++iter, ++k)
		{
			_likelihoodIds[k] = iter->first;
			_likelihoodValues[k] = iter->second;
			if(sameIds && _posteriorIds[k] != iter->first)
			{
				sameIds = false;
			}
		}
	}
	const std::vector<int> & ids = _likelihoodIds;

	// Recursive Bayes estimation...
	// STEP 1 - Prediction : Prior*lastPosterior
	//
	// The prediction is kept in its sparse form only, the matrix never being allocated:
	// built once, then carried over to the locations of the next iteration. Over a fixed
	// graph nothing changes and there is nothing to do; while mapping, the appended
	// locations reach only a few of the columns and only those are built again. A location
	// leaving the working memory shifts the index of every one after it, and is answered by
	// building the prediction again, which is what the dense update does then as well.
	if(!sameIds)
	{
		// The locations changed, so whether the prediction is worth keeping sparse is a
		// question about the new one.
		_predictionChanged = true;
		_sparsePredictionRejected = false;
	}
	const bool keepSparse =
			_sparsePrediction &&
			!_sparsePredictionRejected &&
			_totalPredictionLCValues >= 1;
	bool sparseBuilt = false;
	if(keepSparse)
	{
		if(!_predictionChanged && _sparsePredictionIds == ids)
		{
			sparseBuilt = true;
		}
		else if(this->updateSparsePrediction(memory, _sparsePredictionIds, ids))
		{
			sparseBuilt = true;
		}
		else
		{
			sparseBuilt = this->generateSparsePrediction(memory, ids);
			// Measured as too dense to be worth it: the matrix is built instead, and not
			// measured again until the locations or the model change. Retrying on every
			// iteration would cost more than the multiplication it is trying to save.
			_sparsePredictionRejected = !sparseBuilt;
		}
		_predictionChanged = false;
		UDEBUG("STEP1-generate prior=%fs, columns=%d", timer.ticks(),
				(int)_sparsePredictionColumns.size());
	}
	if(!sparseBuilt)
	{
		this->clearSparsePrediction();
		_prediction = this->generatePrediction(memory, ids);
		_predictionChanged = false;
		UDEBUG("STEP1-generate prior=%fs, rows=%d, cols=%d", timer.ticks(), _prediction.rows, _prediction.cols);
		//std::cout << "Prediction=" << _prediction << std::endl;
	}

	// Adjust the last posterior if some images were
	// reactivated or removed from the working memory. After the prediction, which is built
	// against the ids the posterior still holds from the last iteration.
	if(!sameIds)
	{
		this->updatePosterior(memory, likelihood);
	}
	UASSERT(_posteriorValues.size() == likelihood.size());
	ULOGGER_DEBUG("STEP1-update posterior=%fs, posterior size=%d", timer.ticks(), (int)_posteriorValues.size());

	// Multiply prediction matrix with the last posterior
	// (m,m) X (m,1) = (m,1)
	// The sparse form is empty when disabled, or when the prediction was found too
	// dense for it to be worth it.
	const bool sparse = _sparsePrediction && !_sparsePredictionColumns.empty();
	const float * priorPtr = 0;
	cv::Mat priorMat;
	if(sparse)
	{
		this->multiplySparsePrediction(_posteriorValues, _priorValues);
		priorPtr = &_priorValues[0];
	}
	else
	{
		// A header over the values, so the matrix multiplication reads them where they are.
		const cv::Mat posteriorMat((int)_posteriorValues.size(), 1, CV_32FC1, (void*)&_posteriorValues[0]);
		priorMat = _prediction * posteriorMat;
		priorPtr = (const float *)priorMat.data;
	}
	ULOGGER_DEBUG("STEP1-matrix mult time=%fs (sparse=%d)", timer.ticks(), sparse?1:0);
	//std::cout << "ResultingPrior=" << prior << std::endl;

	// STEP 2 - Update : Multiply with observations (likelihood)
	// The likelihood, the posterior and the prior are all indexed the same way, so this is
	// three vectors walked side by side rather than a search through the posterior per id.
	float sum = 0;
	for(size_t k=0; k<_posteriorValues.size(); ++k)
	{
		_posteriorValues[k] = _likelihoodValues[k] * priorPtr[k];
		sum += _posteriorValues[k];
	}
	ULOGGER_DEBUG("STEP2-likelihood time=%fs", timer.ticks());

	// Normalize
	ULOGGER_DEBUG("sum=%f", sum);
	if(sum != 0)
	{
		for(size_t k=0; k<_posteriorValues.size(); ++k)
		{
			_posteriorValues[k] /= sum;
		}
	}

	ULOGGER_DEBUG("normalize time=%fs", timer.ticks());
	return true;
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
	if(!_sparsePredictionColumns.empty())
	{
		// The prediction is being kept sparse, so there is no matrix: building one to hand
		// over would cost the memory and the time that not building it is saving.
		return cv::Mat();
	}
	if(!_prediction.empty() && this->posteriorHasSameIds(ids))
	{
		return _prediction;
	}
	if(!_fullPredictionUpdate && !_prediction.empty())
	{
		// The ids the matrix was built for, which the posterior is still indexed by.
		return updatePrediction(_prediction, memory, _posteriorIds, ids);
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

// Takes the non zero values of a freshly built column into the prediction, and leaves the
// buffer zeroed for the next one, which saves clearing the whole of it every time.
// Takes the non zero values of a freshly built column into the prediction, and leaves the
// buffer zeroed for the next one, which saves clearing the whole of it every time.
//
// The values of every column live in one array, so that the multiplication reads them the
// way memory likes to be read. A column keeps the room it was given: rebuilt into fewer
// values it stays where it is, rebuilt into more than it has room for it is put at the end
// and the room it had is left behind, to be recovered by compactSparsePrediction(). Asking
// for a little more than is needed, when the column is one being rebuilt, buys the room for
// it to grow a few times in place.
void BayesFilter::takeSparsePredictionColumn(std::vector<float> & column, int index, bool withRoomToGrow)
{
	size_t count = 0;
	for(size_t row=0; row<column.size(); ++row)
	{
		if(column[row] != 0.0f)
		{
			++count;
		}
	}

	SparseColumn & slot = _sparsePredictionColumns[index];
	_sparsePredictionUsed -= slot.size;
	if(count > slot.capacity)
	{
		slot.offset = _sparsePredictionValues.size();
		slot.capacity = withRoomToGrow ? count + count/8 + 4 : count;
		_sparsePredictionValues.resize(slot.offset + slot.capacity);
	}
	slot.size = count;
	_sparsePredictionUsed += count;

	size_t i = slot.offset;
	for(size_t row=0; row<column.size(); ++row)
	{
		if(column[row] != 0.0f)
		{
			_sparsePredictionValues[i++] = std::make_pair((int)row, column[row]);
			column[row] = 0.0f;
		}
	}
}

// Packs the columns back into the order they are multiplied in, giving each the room it
// needs and no more. Called when the room left behind by rebuilt columns has grown to a
// quarter of what is in use, and at the end of a full build, whose columns are not built in
// the order of their index.
void BayesFilter::compactSparsePrediction()
{
	std::vector<std::pair<int, float> > packed;
	packed.reserve(_sparsePredictionUsed);
	for(size_t i=0; i<_sparsePredictionColumns.size(); ++i)
	{
		SparseColumn & slot = _sparsePredictionColumns[i];
		const size_t offset = packed.size();
		packed.insert(packed.end(),
				_sparsePredictionValues.begin()+slot.offset,
				_sparsePredictionValues.begin()+slot.offset+slot.size);
		slot.offset = offset;
		slot.capacity = slot.size;
	}
	_sparsePredictionValues.swap(packed);
}

// The neighborhood of a location, from the cache the incremental update needs, adding it
// there and to the neighborhoods of its own neighbors when it is not there yet.
const std::map<int, int> & BayesFilter::cachedNeighbors(const Memory * memory, int id)
{
	std::map<int, std::map<int, int> >::const_iterator iter = _neighborsIndex.find(id);
	if(iter == _neighborsIndex.end())
	{
		std::map<int, int> neighbors = memory->getNeighborsId(id, _predictionLC.size()-1, 0, false, false, true, true);
		for(std::map<int, int>::iterator jter=neighbors.begin(); jter!=neighbors.end(); ++jter)
		{
			std::map<int, std::map<int, int> >::iterator kter = _neighborsIndex.find(jter->first);
			if(kter != _neighborsIndex.end())
			{
				uInsert(kter->second, std::make_pair(id, jter->second));
			}
		}
		iter = _neighborsIndex.insert(std::make_pair(id, neighbors)).first;
	}
	return iter->second;
}

// The prediction built in its sparse form, the matrix never being allocated.
//
// A column of the prediction only holds the neighbors of one location within the depth of
// the prediction model, so on a large map the matrix is mostly zeros, while holding it
// costs the size of the working memory squared against the far smaller size of the values
// in it. Each column is built in a buffer of its own instead, through the same
// addNeighborProb() and normalize() as the dense build, and only its non zero values are
// kept. The columns are kept apart rather than in one array so that
// updateSparsePrediction() can replace one of them.
//
// The columns are not built in the order of their index: a column is built for every
// location at margin 0 of the one being expanded, so several are built at once.
//
// Returns false when the prediction would not be sparse, which the caller has to answer by
// building the dense matrix. Happens when the values of the model sum to less than 1, as
// normalize() then spreads the missing probability over every zero of a column.
bool BayesFilter::generateSparsePrediction(const Memory * memory, const std::vector<int> & ids)
{
	UASSERT(memory && _predictionLC.size() >= 2 && ids.size());

	UTimer timer;
	this->clearSparsePrediction();

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

	// A value costs 8 bytes kept sparse against the 4 of the matrix, so past a quarter
	// filled the sparse form is not worth building.
	const size_t maxValues = (size_t)size*(size_t)size/4;

	// The neighborhood of a few locations, to know whether the prediction is worth keeping
	// sparse before building all of it. A loop closure link costs no margin, so on a
	// densely linked graph a column reaches most of the map and there is nothing sparse to
	// keep; finding that out by building a quarter of it first would cost more than the
	// multiplications it is trying to save.
	{
		const int samples = size < 64 ? size : 64;
		size_t reached = 0;
		int sampled = 0;
		for(int s=0; s<samples; ++s)
		{
			const int i = (int)((double)s*(double)size/(double)samples);
			if(ids[i] <= 0)
			{
				continue;
			}
			std::list<int> idsLoopMargin;
			const std::map<int, int> neighbors = resolveNeighbors(
					memory, ids[i], _predictionLC.size()-1, idToIndexMap, idsLoopMargin, 0);
			for(std::map<int, int>::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				if(idToIndexMap.find(iter->first) != idToIndexMap.end())
				{
					++reached;
				}
			}
			++sampled;
		}
		if(sampled > 0)
		{
			const double perColumn = double(reached)/double(sampled);
			UDEBUG("Sparse prediction: %.0f values per column over %d locations, estimated "
				   "from %d of them", perColumn, size, sampled);
			if(perColumn*(double)size > (double)maxValues)
			{
				UWARN("A column of the prediction holds %.0f of the %d locations, estimated "
					  "from %d of them, which is too dense for %s to be worth it: a value "
					  "costs 8 bytes kept sparse against the 4 of the matrix. Building the "
					  "matrix instead. Every loop closure link widens a column, as one "
					  "costs no depth in the graph search, and so does a longer %s.",
					  perColumn, size, sampled,
					  Parameters::kBayesSparsePrediction().c_str(),
					  Parameters::kBayesPredictionLC().c_str());
				return false;
			}
		}
	}

	// The neighborhood a column was built from is kept only when locations can be added,
	// which is what updateSparsePrediction() needs it for. Over a fixed graph nothing is
	// ever appended, and one neighborhood per location is as much memory again as the
	// values of the prediction.
	std::map<int, std::map<int, int> > * cache = memory->isIncremental() ? &_neighborsIndex : 0;

	_sparsePredictionColumns.assign(size, SparseColumn());
	std::vector<float> column(size, 0.0f);

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
			std::map<int, int> neighbors = resolveNeighbors(
					memory, ids[i], _predictionLC.size()-1, idToIndexMap, idsLoopMargin, cache);

			// same neighbor tree for loop signatures (margin = 0)
			for(std::list<int>::iterator iter=idsLoopMargin.begin(); iter!=idsLoopMargin.end(); ++iter)
			{
				if(cache)
				{
					uInsert(*cache, std::make_pair(*iter, neighbors));
				}
				const int index = idToIndexMap.at(*iter);
				const float sum = addNeighborProb(&column[0], 1, neighbors, _predictionLC, idToIndexMap);
				this->normalize(&column[0], 1, size, index, sum, ids[0]<0);
				this->takeSparsePredictionColumn(column, index, false);
				idsDone.insert(*iter);
			}
		}
		else
		{
			this->fillVirtualPlaceColumn(&column[0], 1, size);
			this->takeSparsePredictionColumn(column, i, false);
		}
	}
	// The columns were not built in the order of their index, so they are packed into it.
	this->compactSparsePrediction();
	_sparsePredictionIds = ids;

	const size_t nnz = _sparsePredictionUsed;
	UDEBUG("Sparse prediction: %ld/%ld values (%.2f%%), %ld MB against the %ld MB of the "
		   "matrix, built in %fs",
			(long)nnz, (long)size*size, 100.0*double(nnz)/(double(size)*double(size)),
			(long)(this->getSparsePredictionMemoryUsed()/1048576),
			(long)((size_t)size*(size_t)size*sizeof(float)/1048576),
			timer.ticks());
	return true;
}

// The same prediction after locations were appended, without building it again.
//
// Every location that was already there keeps its index, so the columns already built
// still apply: only the ones the new locations reach have to be built again, and the
// column of the virtual place, whose values are shared out over however many locations
// there are. What a column holds does not otherwise depend on how many there are, the
// model summing to 1 leaving normalize() nothing to spread over the others.
//
// Returns false when the locations are not the previous ones with more appended, which the
// caller has to answer by building the prediction again: a location removed shifts the
// index of every one after it, and the dense update has to recompute every column then too.
bool BayesFilter::updateSparsePrediction(
		const Memory * memory,
		const std::vector<int> & oldIds,
		const std::vector<int> & newIds)
{
	if(_fullPredictionUpdate ||
	   oldIds.empty() ||
	   newIds.size() <= oldIds.size() ||
	   _sparsePredictionColumns.size() != oldIds.size() ||
	   memcmp(oldIds.data(), newIds.data(), oldIds.size()*sizeof(int)) != 0)
	{
		return false;
	}

	UTimer timer;
	const int size = (int)newIds.size();

	IdToIndexMap newIdToIndexMap;
#if __cplusplus >= 201103L
	newIdToIndexMap.reserve(newIds.size());
#endif
	for(int i=0; i<size; ++i)
	{
		if(newIds[i]>0)
		{
			newIdToIndexMap[newIds[i]] = i;
		}
	}

	_sparsePredictionColumns.resize(size);   // the appended columns start out empty
	std::vector<float> column(size, 0.0f);

	// The appended locations, and the ones that were already there whose neighborhood the
	// appended ones are now part of.
	std::set<int> idsToUpdate;
	for(size_t i=oldIds.size(); i<newIds.size(); ++i)
	{
		if(newIds[i] <= 0)
		{
			continue;
		}
		const std::map<int, int> & neighbors = this->cachedNeighbors(memory, newIds[i]);
		const float sum = addNeighborProb(&column[0], 1, neighbors, _predictionLC, newIdToIndexMap);
		this->normalize(&column[0], 1, size, (int)i, sum, newIds[0]<0);
		this->takeSparsePredictionColumn(column, (int)i, true);
		for(std::map<int, int>::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
		{
			const IdToIndexMap::const_iterator jter = newIdToIndexMap.find(iter->first);
			if(jter != newIdToIndexMap.end() && (size_t)jter->second < oldIds.size())
			{
				idsToUpdate.insert(iter->first);
			}
		}
	}

	for(std::set<int>::const_iterator iter=idsToUpdate.begin(); iter!=idsToUpdate.end(); ++iter)
	{
		const std::map<int, std::map<int, int> >::const_iterator kter = _neighborsIndex.find(*iter);
		UASSERT_MSG(kter != _neighborsIndex.end(),
				uFormat("Did not find %d (current index size=%d)", *iter, (int)_neighborsIndex.size()).c_str());
		const int index = newIdToIndexMap.at(*iter);
		const float sum = addNeighborProb(&column[0], 1, kter->second, _predictionLC, newIdToIndexMap);
		this->normalize(&column[0], 1, size, index, sum, newIds[0]<0);
		this->takeSparsePredictionColumn(column, index, true);
	}

	// The virtual place shares what is left of its probability over the visited locations,
	// so its column depends on how many of them there are.
	if(newIds[0] < 0)
	{
		this->fillVirtualPlaceColumn(&column[0], 1, size);
		this->takeSparsePredictionColumn(column, 0, true);
	}

	// The room left behind by the columns that outgrew their slot, once it is a quarter of
	// what is in use.
	const size_t waste = _sparsePredictionValues.size() - _sparsePredictionUsed;
	const bool compacted = waste > _sparsePredictionUsed/4;
	if(compacted)
	{
		this->compactSparsePrediction();
	}
	_sparsePredictionIds = newIds;

	UDEBUG("Sparse prediction: %d locations appended, %d columns rebuilt of %d, %ld values, "
		   "%ld left behind%s, updated in %fs",
			(int)(newIds.size()-oldIds.size()), (int)idsToUpdate.size(), size,
			(long)_sparsePredictionUsed, (long)waste, compacted?" (packed again)":"",
			timer.ticks());
	return true;
}

void BayesFilter::clearSparsePrediction()
{
	_sparsePredictionColumns.clear();
	_sparsePredictionValues.clear();
	_sparsePredictionIds.clear();
	_sparsePredictionUsed = 0;
}

unsigned long BayesFilter::getSparsePredictionMemoryUsed() const
{
	return _sparsePredictionValues.capacity() * sizeof(std::pair<int, float>)
			+ _sparsePredictionColumns.capacity() * sizeof(SparseColumn)
			+ _sparsePredictionIds.capacity() * sizeof(int);
}

void BayesFilter::multiplySparsePrediction(const std::vector<float> & posterior, std::vector<float> & prior) const
{
	const size_t size = _sparsePredictionColumns.size();
	UASSERT(size > 0);
	UASSERT_MSG(posterior.size() == size,
			uFormat("posterior=%d prediction=%d", (int)posterior.size(), (int)size).c_str());

	prior.assign(size, 0.0f);
	const float * posteriorPtr = &posterior[0];
	float * priorPtr = &prior[0];

	// The prior is the sum of the columns of the prediction weighted by the posterior.
	// Going by column is the order the values are stored in, and lets a location the
	// posterior has ruled out be skipped whole.
	for(size_t col=0; col<size; ++col)
	{
		const float weight = posteriorPtr[col];
		if(weight == 0.0f)
		{
			continue;
		}
		const SparseColumn & slot = _sparsePredictionColumns[col];
		for(size_t i=slot.offset; i<slot.offset+slot.size; ++i)
		{
			priorPtr[_sparsePredictionValues[i].first] += _sparsePredictionValues[i].second * weight;
		}
	}
}

unsigned long BayesFilter::getMemoryUsed() const
{
	long memoryUsage = sizeof(BayesFilter);
	if(!_prediction.empty())
	{
		memoryUsage += _prediction.total() * _prediction.elemSize();
	}
	memoryUsage += _predictionLC.size() * sizeof(double);
	// The vectors an iteration works on, indexed the same way as the posterior.
	memoryUsage += _posteriorIds.capacity() * sizeof(int);
	memoryUsage += _posteriorValues.capacity() * sizeof(float);
	memoryUsage += _likelihoodIds.capacity() * sizeof(int);
	memoryUsage += _likelihoodValues.capacity() * sizeof(float);
	memoryUsage += _priorValues.capacity() * sizeof(float);
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

// Realigns the posterior with the ids of the likelihood, keeping the probability of the
// locations that are in both. Called only when they differ, which over a fixed graph never
// happens after the first iteration.
//
// Both are sorted by id, so the ones in both are found by walking them side by side rather
// than searching for each.
void BayesFilter::updatePosterior(const Memory * memory, const std::map<int, float> & likelihood)
{
	ULOGGER_DEBUG("");
	const bool wasEmpty = _posteriorIds.empty();
	std::vector<int> ids;
	std::vector<float> values;
	ids.reserve(likelihood.size());
	values.reserve(likelihood.size());
	size_t k = 0;
	for(std::map<int, float>::const_iterator iter=likelihood.begin(); iter!=likelihood.end(); ++iter)
	{
		while(k < _posteriorIds.size() && _posteriorIds[k] < iter->first)
		{
			++k;
		}
		float value = wasEmpty ? 1.0f : 0.0f;
		if(k < _posteriorIds.size() && _posteriorIds[k] == iter->first)
		{
			value = _posteriorValues[k];
		}
		ids.push_back(iter->first);
		values.push_back(value);
	}
	_posteriorIds.swap(ids);
	_posteriorValues.swap(values);
}

} // namespace rtabmap
