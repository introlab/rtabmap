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

#include "bayes/DensePrediction.h"
#include "bayes/PredictionModel.h"
#include "bayes/SparsePrediction.h"

#include <set>

#include "rtabmap/utilite/UtiLite.h"

namespace rtabmap {

BayesFilter::BayesFilter(const ParametersMap & parameters) :
	_model(new bayes::PredictionModel()),
	_dense(new bayes::DensePrediction()),
	_sparse(new bayes::SparsePrediction()),
	_fullPredictionUpdate(Parameters::defaultBayesFullPredictionUpdate()),
	_sparsePrediction(Parameters::defaultBayesSparsePrediction()),
	_keepSparse(false),
	_predictionChanged(true)
{
	_model->setVirtualPlacePrior(Parameters::defaultBayesVirtualPlacePriorThr());
	this->setPredictionLC(Parameters::defaultBayesPredictionLC());
	this->parseParameters(parameters);
}

BayesFilter::~BayesFilter()
{
	delete _model;
	delete _dense;
	delete _sparse;
}

void BayesFilter::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kBayesPredictionLC())) != parameters.end())
	{
		this->setPredictionLC((*iter).second);
	}
	float virtualPlacePrior = _model->virtualPlacePrior();
	if(Parameters::parse(parameters, Parameters::kBayesVirtualPlacePriorThr(), virtualPlacePrior))
	{
		UASSERT(virtualPlacePrior >= 0 && virtualPlacePrior <= 1.0f);
		_model->setVirtualPlacePrior(virtualPlacePrior);
	}
	Parameters::parse(parameters, Parameters::kBayesFullPredictionUpdate(), _fullPredictionUpdate);
	if(Parameters::parse(parameters, Parameters::kBayesSparsePrediction(), _sparsePrediction))
	{
		// The sparse view is rebuilt on the next posterior if it was just enabled, and
		// released if it was just disabled.
		_predictionChanged = true;
		this->updateKeepSparse();
	}
}

// format = {Virtual place, Loop closure, level1, level2, l3, l4...}
void BayesFilter::setPredictionLC(const std::string & prediction)
{
	if(_model->set(prediction))
	{
		// A new model changes the values of the prediction, and whether any of it is worth
		// keeping sparse.
		_predictionChanged = true;
		this->updateKeepSparse();
	}
}

// Asked for by the parameter, and possible only over a model whose values sum to 1: below that,
// normalize() spreads the difference over every zero of a column and there is nothing sparse
// left to keep. Nothing else gives the sparse form up, however densely the graph is linked, so
// that what the parameter measures is the sparse form and not a fallback to the matrix.
void BayesFilter::updateKeepSparse()
{
	_keepSparse = _sparsePrediction && !_model->spreadsOverAllLocations();
	if(_sparsePrediction && !_keepSparse)
	{
		UWARN("%s is enabled but the values of %s sum to %f, less than 1: the difference is "
			  "spread over every location, which leaves no zero in a column for the sparse form "
			  "to keep out, so the prediction is held as a matrix instead.",
			  Parameters::kBayesSparsePrediction().c_str(),
			  Parameters::kBayesPredictionLC().c_str(), _model->total());
	}
	if(!_keepSparse)
	{
		_sparse->clear();
	}
}

const std::vector<double> & BayesFilter::getPredictionLC() const
{
	// {Vp, Lc, l1, l2, l3, l4...}
	return _model->values();
}

std::string BayesFilter::getPredictionLCStr() const
{
	return _model->str();
}

float BayesFilter::getVirtualPlacePrior() const
{
	return _model->virtualPlacePrior();
}

bool BayesFilter::isPredictionSparse() const
{
	return !_sparse->empty();
}

void BayesFilter::reset()
{
	_posteriorIds.clear();
	_posteriorValues.clear();
	_dense->clear();
	_sparse->clear();
	_predictionChanged = true;
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

	UASSERT(_model->valid());

	UTimer timer;
	timer.start();

	// One walk of the likelihood: its values into a vector, and its ids against the ones the
	// posterior is indexed by. Everything below then works on vectors.
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
		_predictionChanged = true;
	}
	if(_keepSparse)
	{
		// Nothing to do at all when neither the prediction nor the locations changed.
		if(_predictionChanged || _sparse->ids() != ids)
		{
			// The neighborhoods are kept only when locations can be added, which is what the
			// update needs them for: over a fixed graph one per location is as much memory
			// again as the values of the prediction.
			if(_fullPredictionUpdate || !_sparse->update(*_model, memory, ids, _neighborsIndex))
			{
				_sparse->generate(*_model, memory, ids,
						memory->isIncremental() ? &_neighborsIndex : 0);
			}
		}
		UDEBUG("STEP1-generate prior=%fs, values=%d", timer.ticks(), (int)_sparse->values());

		// The matrix is released as soon as the sparse form takes over. It is built for the
		// locations of the iteration it was built on, and the locations move on while the
		// sparse form is the one being used, so it can neither be multiplied nor carried over
		// once the sparse form gives the prediction back. A fallback to the matrix builds it
		// again.
		_dense->clear();
	}
	else
	{
		_sparse->clear();
		if(_predictionChanged || _dense->empty())
		{
			// Only when it has to be: over a fixed graph the matrix of the last iteration is
			// the one this iteration wants.
			_dense->generate(*_model, memory, ids, _fullPredictionUpdate, &_neighborsIndex);
		}
		UDEBUG("STEP1-generate prior=%fs, rows=%d, cols=%d", timer.ticks(),
				_dense->matrix().rows, _dense->matrix().cols);
		//std::cout << "Prediction=" << _dense->matrix() << std::endl;
	}
	// Cleared once, after whichever of the two built it: the sparse form, or the matrix when
	// the prediction is not kept sparse.
	_predictionChanged = false;

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
	// Held sparse, or as the matrix when updateKeepSparse() gave the sparse form up.
	const bool sparse = !_sparse->empty();
	if(sparse)
	{
		_sparse->multiply(_posteriorValues, _priorValues);
	}
	else
	{
		_dense->multiply(_posteriorValues, _priorValues);
	}
	const float * priorPtr = &_priorValues[0];
	ULOGGER_DEBUG("STEP1-matrix mult time=%fs (sparse=%d)", timer.ticks(), sparse?1:0);
	//std::cout << "ResultingPrior=" << prior << std::endl;

	// STEP 2 - Update : Multiply with observations (likelihood)
	// The likelihood, the posterior and the prior are all indexed the same way, so the three
	// are walked side by side.
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

cv::Mat BayesFilter::generatePrediction(const Memory * memory, const std::vector<int> & ids)
{
	if(!_sparse->empty() && _sparse->ids() == ids)
	{
		// Expanded from the sparse form, which holds the same prediction. The matrix costs
		// what keeping it sparse is saving, so it is built to be read and not kept.
		return _sparse->toMatrix();
	}
	if(!_dense->empty() && _dense->ids() == ids)
	{
		return _dense->matrix();
	}
	UASSERT(memory && _model->valid() && ids.size());
	return _dense->generate(*_model, memory, ids, _fullPredictionUpdate, &_neighborsIndex);
}

unsigned long BayesFilter::getMemoryUsed() const
{
	long memoryUsage = sizeof(BayesFilter);
	memoryUsage += _dense->memoryUsed();
	memoryUsage += _sparse->memoryUsed();
	memoryUsage += _model->memoryUsed();
	// The vectors an iteration works on, indexed the same way as the posterior.
	memoryUsage += _posteriorIds.capacity() * sizeof(int);
	memoryUsage += _posteriorValues.capacity() * sizeof(float);
	memoryUsage += _likelihoodIds.capacity() * sizeof(int);
	memoryUsage += _likelihoodValues.capacity() * sizeof(float);
	memoryUsage += _priorValues.capacity() * sizeof(float);
	memoryUsage += _neighborsIndex.size() * (sizeof(int)+sizeof(std::map<int, int>)+sizeof(std::map<int, std::map<int, int> >::iterator)) + sizeof(std::map<int, std::map<int, int> >);
	for(std::map<int, std::map<int, int> >::const_iterator iter=_neighborsIndex.begin(); iter!=_neighborsIndex.end(); ++iter)
	{
		memoryUsage += iter->second.size() * (sizeof(int)*2+sizeof(std::map<int, int>::iterator)) + sizeof(std::map<int, int>);
	}
	return memoryUsage;
}

void BayesFilter::updatePosterior(const Memory * memory, const std::map<int, float> & likelihood)
{
	ULOGGER_DEBUG("");
	const bool wasEmpty = _posteriorIds.empty();
	std::vector<int> ids;
	std::vector<float> values;
	ids.reserve(likelihood.size());
	values.reserve(likelihood.size());
	// Both the likelihood and the posterior are ascending by id, so the two are merged in one
	// walk, k only ever moving forward: for each location of the likelihood, advance the
	// posterior up to it. A location in both keeps its probability, a location removed from
	// the working memory is left behind, and a location that came back gets 0 (1 on the very
	// first iteration, where the posterior starts uniform).
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
