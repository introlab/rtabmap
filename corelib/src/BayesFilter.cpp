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

BayesFilter::BayesFilter(const ParametersMap & parameters) :
	_virtualPlacePrior(Parameters::defaultBayesVirtualPlacePriorThr()),
	_fullPredictionUpdate(Parameters::defaultBayesFullPredictionUpdate()),
	_totalPredictionLCValues(0.0f),
	_predictionEpsilon(0.0f)
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

void BayesFilter::reset()
{
	_posterior.clear();
	_prediction = cv::Mat();
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
	// Recursive Bayes estimation...
	// STEP 1 - Prediction : Prior*lastPosterior
	_prediction = this->generatePrediction(memory, uKeys(likelihood));

	UDEBUG("STEP1-generate prior=%fs, rows=%d, cols=%d", timer.ticks(), _prediction.rows, _prediction.cols);
	//std::cout << "Prediction=" << _prediction << std::endl;

	// Adjust the last posterior if some images were
	// reactivated or removed from the working memory
	posterior = cv::Mat(likelihood.size(), 1, CV_32FC1);
	this->updatePosterior(memory, uKeys(likelihood));
	j=0;
	for(std::map<int, float>::const_iterator i=_posterior.begin(); i!= _posterior.end(); ++i)
	{
		((float*)posterior.data)[j++] = (*i).second;
	}
	ULOGGER_DEBUG("STEP1-update posterior=%fs, posterior=%d, _posterior size=%d", posterior.rows, _posterior.size());
	//std::cout << "LastPosterior=" << posterior << std::endl;

	// Multiply prediction matrix with the last posterior
	// (m,m) X (m,1) = (m,1)
	prior = _prediction * posterior;
	ULOGGER_DEBUG("STEP1-matrix mult time=%fs", timer.ticks());
	//std::cout << "ResultingPrior=" << prior << std::endl;

	ULOGGER_DEBUG("STEP1-matrix mult time=%fs", timer.ticks());
	std::vector<float> likelihoodValues = uValues(likelihood);
	//std::cout << "Likelihood=" << cv::Mat(likelihoodValues) << std::endl;

	// STEP 2 - Update : Multiply with observations (likelihood)
	j=0;
	for(std::map<int, float>::const_iterator i=likelihood.begin(); i!= likelihood.end(); ++i)
	{
		std::map<int, float>::iterator p =_posterior.find((*i).first);
		if(p!= _posterior.end())
		{
			(*p).second = (*i).second * ((float*)prior.data)[j++];
			sum+=(*p).second;
		}
		else
		{
			ULOGGER_ERROR("Problem1! can't find id=%d", (*i).first);
		}
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

float addNeighborProb(cv::Mat & prediction,
			unsigned int col,
			const std::map<int, int> & neighbors,
			const std::vector<double> & predictionLC,
#if __cplusplus >= 201103L
			const std::unordered_map<int, int> & idToIndex
#else
			const std::map<int, int> & idToIndex
#endif
			)
{
	UASSERT(col < (unsigned int)prediction.cols &&
			col < (unsigned int)prediction.rows);

	float sum=0.0f;
	float * dataPtr = (float*)prediction.data;
	for(std::map<int, int>::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
	{
		if(iter->first>=0)
		{
#if __cplusplus >= 201103L
			std::unordered_map<int, int>::const_iterator jter = idToIndex.find(iter->first);
#else
			std::map<int, int>::const_iterator jter = idToIndex.find(iter->first);
#endif
			if(jter != idToIndex.end())
			{
				UASSERT((iter->second+1) < (int)predictionLC.size());
				sum += dataPtr[col + jter->second*prediction.cols] = predictionLC[iter->second+1];
			}
		}
	}
	return sum;
}


cv::Mat BayesFilter::generatePrediction(const Memory * memory, const std::vector<int> & ids)
{
	std::vector<int> oldIds = uKeys(_posterior);
	if(oldIds.size() == ids.size() &&
		memcmp(oldIds.data(), ids.data(), oldIds.size()*sizeof(int)) == 0)
	{
		return _prediction;
	}

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

#if __cplusplus >= 201103L
	std::unordered_map<int,int> idToIndexMap;
	idToIndexMap.reserve(ids.size());
#else
	std::map<int,int> idToIndexMap;
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
	UDEBUG("_predictionLC.size()=%d",_predictionLC.size());
	std::set<int> idsDone;

	for(unsigned int i=0; i<ids.size(); ++i)
	{
		if(idsDone.find(ids[i]) == idsDone.end())
		{
			if(ids[i] > 0)
			{
				// Set high values (gaussians curves) to loop closure neighbors

				// ADD prob for each neighbors
				std::map<int, int> neighbors = memory->getNeighborsId(ids[i], _predictionLC.size()-1, 0, false, false, true, true);

				if(!_fullPredictionUpdate)
				{
					uInsert(_neighborsIndex, std::make_pair(ids[i], neighbors));
				}

				std::list<int> idsLoopMargin;
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
							idsLoopMargin.push_back(iter->first);
						}
						++iter;
					}
				}

				// should at least have 1 id in idsMarginLoop
				if(idsLoopMargin.size() == 0)
				{
					UFATAL("No 0 margin neighbor for signature %d !?!?", ids[i]);
				}

				// same neighbor tree for loop signatures (margin = 0)
				for(std::list<int>::iterator iter = idsLoopMargin.begin(); iter!=idsLoopMargin.end(); ++iter)
				{
					if(!_fullPredictionUpdate)
					{
						uInsert(_neighborsIndex, std::make_pair(*iter, neighbors));
					}

					float sum = 0.0f; // sum values added
					int index = idToIndexMap.at(*iter);
					sum += addNeighborProb(prediction, index, neighbors, _predictionLC, idToIndexMap);
					idsDone.insert(*iter);
					this->normalize(prediction, index, sum, ids[0]<0);
				}
			}
			else
			{
				// Set the virtual place prior
				if(_virtualPlacePrior > 0)
				{
					if(cols>1) // The first must be the virtual place
					{
						((float*)prediction.data)[i] = _virtualPlacePrior;
						float val = (1.0-_virtualPlacePrior)/(cols-1);
						for(int j=1; j<cols; j++)
						{
							((float*)prediction.data)[i + j*cols] = val;
						}
					}
					else if(cols>0)
					{
						((float*)prediction.data)[i] = 1;
					}
				}
				else
				{
					// Only for some tests...
					// when _virtualPlacePrior=0, set all priors to the same value
					if(cols>1)
					{
						float val = 1.0/cols;
						for(int j=0; j<cols; j++)
						{
							((float*)prediction.data)[i + j*cols] = val;
						}
					}
					else if(cols>0)
					{
						((float*)prediction.data)[i] = 1;
					}
				}
			}
		}
	}

	ULOGGER_DEBUG("time = %fs", timerGlobal.ticks());

	return prediction;
}

void BayesFilter::normalize(cv::Mat & prediction, unsigned int index, float addedProbabilitiesSum, bool virtualPlaceUsed) const
{
	UASSERT(index < (unsigned int)prediction.rows && index < (unsigned int)prediction.cols);

	int cols = prediction.cols;
	// ADD values of not found neighbors to loop closure
	if(addedProbabilitiesSum < _totalPredictionLCValues-_predictionLC[0])
	{
		float delta = _totalPredictionLCValues-_predictionLC[0]-addedProbabilitiesSum;
		((float*)prediction.data)[index + index*cols] += delta;
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
			if(((float*)prediction.data)[index + j*cols] == 0)
			{
				((float*)prediction.data)[index + j*cols] = value;
				addedProbabilitiesSum += ((float*)prediction.data)[index + j*cols];
			}
		}
	}

	//normalize this row
	float maxNorm = 1 - (virtualPlaceUsed?_predictionLC[0]:0); // 1 - virtual place probability
	if(addedProbabilitiesSum<maxNorm-0.0001 || addedProbabilitiesSum>maxNorm+0.0001)
	{
		for(int j=virtualPlaceUsed?1:0; j<cols; ++j)
		{
			((float*)prediction.data)[index + j*cols] *= maxNorm / addedProbabilitiesSum;
			if(((float*)prediction.data)[index + j*cols] < _predictionEpsilon)
			{
				((float*)prediction.data)[index + j*cols] = 0.0f;
			}
		}
		addedProbabilitiesSum = maxNorm;
	}

	// ADD virtual place prob
	if(virtualPlaceUsed)
	{
		((float*)prediction.data)[index] = _predictionLC[0];
		addedProbabilitiesSum += ((float*)prediction.data)[index];
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

#if __cplusplus >= 201103L
	std::unordered_map<int,int> newIdToIndexMap;
	newIdToIndexMap.reserve(newIds.size());
#else
	std::map<int,int> newIdToIndexMap;
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

			float sum = addNeighborProb(prediction, i, neighbors, _predictionLC, newIdToIndexMap);
			this->normalize(prediction, i, sum, newIds[0]<0);

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
	UDEBUG("time getting %d ids to update = %fs", idsToUpdate.size(), timer.restart());

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

			float sum = addNeighborProb(prediction, index, neighbors, _predictionLC, newIdToIndexMap);
			e3+=t1.ticks();

			this->normalize(prediction, index, sum, newIds[0]<0);
			++modified;
			e4+=t1.ticks();
		}
	}
	UDEBUG("time updating modified/added %d ids = %fs (e0=%f e1=%f e2=%f e3=%f e4=%f)", idsToUpdate.size(), timer.restart(), e0, e1, e2, e3, e4);

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
