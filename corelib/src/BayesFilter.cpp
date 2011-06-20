/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "BayesFilter.h"
#include "Memory.h"
#include "Signature.h"
#include "rtabmap/core/Parameters.h"

#include "utilite/UtiLite.h"

namespace rtabmap {

BayesFilter::BayesFilter(const ParametersMap & parameters) :
	_virtualPlacePrior(Parameters::defaultBayesVirtualPlacePriorThr())
{
	this->setPredictionLC(Parameters::defaultBayesPredictionLC());
	this->parseParameters(parameters);
}

BayesFilter::~BayesFilter() {
}

void BayesFilter::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kBayesVirtualPlacePriorThr())) != parameters.end())
	{
		this->setVirtualPlacePrior(std::atof((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kBayesPredictionLC())) != parameters.end())
	{
		this->setPredictionLC((*iter).second);
	}
}

void BayesFilter::setVirtualPlacePrior(float virtualPlacePrior)
{
	if(virtualPlacePrior < 0)
	{
		ULOGGER_WARN("virtualPlacePrior=%f, must be >=0 and <=1", virtualPlacePrior);
		_virtualPlacePrior = 0;
	}
	else if(virtualPlacePrior > 1)
	{
		ULOGGER_WARN("virtualPlacePrior=%f, must be >=0 and <=1", virtualPlacePrior);
		_virtualPlacePrior = 1;
	}
	else
	{
		_virtualPlacePrior = virtualPlacePrior;
	}
}

// format = {Virtual place, Loop closure, level1, level2, l3, l4...}
void BayesFilter::setPredictionLC(const std::string & prediction)
{
	std::list<std::string> strValues = uSplit(prediction, ' ');
	if(strValues.size() < 2)
	{
		ULOGGER_ERROR("The number of values < 2 (prediction=\"%s\")", prediction.c_str());
	}
	else
	{
		std::vector<double> tmpValues(strValues.size());
		int i=0;
		bool valid = true;
		float sum = 0;;
		for(std::list<std::string>::iterator iter = strValues.begin(); iter!=strValues.end(); ++iter)
		{
			tmpValues[i] = std::atof((*iter).c_str());
			sum += tmpValues[i];
			if(i>1)
			{
				sum += tmpValues[i]; // add a second time
			}
			if(tmpValues[i] < 0 || tmpValues[i]>1)
			{
				valid = false;
				break;
			}
			++i;
		}

		if(!valid || sum <= 0 || sum > 1.001)
		{
			ULOGGER_ERROR("The prediction is not valid (the sum must be between >0 && <=1, sum=%f), negative values are not allowed (prediction=\"%s\")", sum, prediction.c_str());
		}
		else
		{
			_predictionLC = tmpValues;
		}
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
		values.append(uNumber2str(_predictionLC[i]));
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

	CvMat * prediction = 0;
	CvMat * prior = 0;
	CvMat * posterior = 0;

	float sum = 0;
	int j=0;
	// Recursive Bayes estimation...
	// STEP 1 - Prediction : Prior*lastPosterior
	prediction = cvCreateMat(likelihood.size(), likelihood.size(), CV_32FC1);
	std::map<int, int> likelihoodKeys;
	int index = 0;
	for(std::map<int, float>::const_iterator iter=likelihood.begin(); iter!=likelihood.end(); ++iter)
	{
		likelihoodKeys.insert(likelihoodKeys.end(), std::pair<int, int>(iter->first, index++));
	}

	if(this->generatePrediction(prediction, memory, likelihoodKeys))
	{
		ULOGGER_DEBUG("STEP1-generate prior=%fs, rows=%d, cols=%d", timer.ticks(), prediction->rows, prediction->cols);

		// Adjust the last posterior if some images were
		// reactivated or removed from the working memory
		posterior = cvCreateMat(likelihood.size(), 1, CV_32FC1);
		this->updatePosterior(memory, uKeys(likelihood));
		j=0;
		for(std::map<int, float>::const_iterator i=_posterior.begin(); i!= _posterior.end(); ++i)
		{
			posterior->data.fl[j++] = (*i).second;
		}
		ULOGGER_DEBUG("STEP1-update posterior=%fs, posterior=%d, _posterior size=%d", posterior->rows, _posterior.size());

		// Multiply prediction matrix with the last posterior
		// (m,m) X (m,1) = (m,1)
		prior = cvCreateMat(likelihood.size(), 1, CV_32FC1);
		cvMatMul(prediction, posterior, prior);
		ULOGGER_DEBUG("STEP1-matrix mult time=%fs", timer.ticks());

		// STEP 2 - Update : Multiply with observations (likelihood)
		j=0;
		for(std::map<int, float>::const_iterator i=likelihood.begin(); i!= likelihood.end(); ++i)
		{
			std::map<int, float>::iterator p =_posterior.find((*i).first);
			if(p!= _posterior.end())
			{
				(*p).second = (*i).second * prior->data.fl[j++];
				sum+=(*p).second;
			}
			else
			{
				ULOGGER_ERROR("Problem1! can't find id=%d", (*i).first);
			}
		}
		ULOGGER_DEBUG("STEP2-likelihood time=%fs", timer.ticks());

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
	}

	cvReleaseMat(&prediction);
	cvReleaseMat(&prior);
	cvReleaseMat(&posterior);

	return _posterior;
}

bool BayesFilter::generatePrediction(CvMat * prediction, const Memory * memory, const std::map<int, int> & likelihoodIds) const
{
	ULOGGER_DEBUG("");
	UTimer timer;
	timer.start();
	UTimer timerGlobal;
	timerGlobal.start();

	if(!likelihoodIds.size() ||
	   prediction == 0 ||
	   prediction->rows != prediction->cols ||
	   (unsigned int)prediction->rows != likelihoodIds.size()/*||
	   prediction->type != CV_32FC1*/ ||
	   _predictionLC.size() < 2 ||
	   !memory)
	{
		ULOGGER_ERROR( "fail");
		return false;
	}

	//int rows = prediction->rows;
	cvSetZero(prediction);
	int cols = prediction->cols;

	// Each priors are column vectors
	unsigned int i=0;
	ULOGGER_DEBUG("_predictionLC.size()=%d",_predictionLC.size());
	for(std::map<int, int>::const_iterator iter=likelihoodIds.begin(); iter!=likelihoodIds.end(); ++iter)
	{
		if(iter->first > 0)
		{
			// Create the sum of 2 gaussians around the loop closure

			int loopClosureId = iter->first;

			// Set high values (gaussians curves) to loop closure neighbors
			const Signature * loopSign = memory->getSignature(loopClosureId);
			if(!loopSign)
			{
				ULOGGER_ERROR("loopSign %d is not found?!?", loopClosureId);
			}

			// LoopID
			prediction->data.fl[i + i*cols] += _predictionLC[1];

			// look up for each neighbors (RECURSIVE)
			this->addNeighborProb(prediction, i, memory, likelihoodIds, loopSign, 1);

			//ULOGGER_DEBUG("neighbor prob for %d, neighbors=%d, time = %fs", loopSign->id(), loopSign->getNeighborIds().size(), timer.ticks());

			float totalModelValues = _predictionLC[0] + _predictionLC[1];
			for(unsigned int j=2; j<_predictionLC.size(); ++j)
			{
				totalModelValues += _predictionLC[j]*2;
			}

			//Add values of not found neighbors to the loop closure
			float sum = 0;
			for(int j=0; j<cols; ++j)
			{
				sum += prediction->data.fl[i + j*cols];
			}
			if(sum < (totalModelValues-_predictionLC[0]))
			{
				float gap = (totalModelValues-_predictionLC[0]) - sum;
				prediction->data.fl[i + i*cols] += gap;
				sum += gap;
			}

			// add virtual place prob
			if(likelihoodIds.begin()->first < 0)
			{
				sum += prediction->data.fl[i] = _predictionLC[0];
			}

			// Set all loop events to small values according to the model
			if(totalModelValues < 1.0f)
			{
				float value = (1.0f-totalModelValues) / float(cols);
				for(int j=0; j<cols; ++j)
				{
					if(!prediction->data.fl[i + j*cols])
					{
						sum += prediction->data.fl[i + j*cols] = value;
					}
				}
			}

			//normalize this row,
			for(int j=0; j<cols; ++j)
			{
				prediction->data.fl[i + j*cols] /= sum;
			}

			//debug
			//for(int j=0; j<cols; ++j)
			//{
			//	ULOGGER_DEBUG("test = %f", prediction->data.fl[i + j*cols]);
			//}
		}
		else
		{
			// Set the virtual place prior
			if(_virtualPlacePrior > 0)
			{
				if(cols>1) // The first must be the virtual place
				{
					prediction->data.fl[i] = _virtualPlacePrior;
					float val = (1.0-_virtualPlacePrior)/(cols-1);
					for(int j=1; j<cols; j++)
					{
						prediction->data.fl[i + j*cols] = val;
					}
				}
				else if(cols>0)
				{
					prediction->data.fl[i] = 1;
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
						prediction->data.fl[i + j*cols] = val;
					}
				}
				else if(cols>0)
				{
					prediction->data.fl[i] = 1;
				}
			}
		}
		++i;
	}

	ULOGGER_DEBUG("time = %fs", timerGlobal.ticks());

	return true;
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

//recursive...
float BayesFilter::addNeighborProb(CvMat * prediction, unsigned int row, const Memory * memory, const std::map<int, int> & likelihoodIds, const Signature * s, unsigned int level) const
{
	if(!likelihoodIds.size() ||
	   prediction == 0 ||
	   prediction->rows != prediction->cols ||
	   (unsigned int)prediction->rows != likelihoodIds.size() ||
	   _predictionLC.size() < 2 ||
	   !memory ||
	   !prediction ||
	   level<1)
	{
		ULOGGER_ERROR( "fail");
		return 0;
	}

	if(level+1 >= _predictionLC.size() || !s)
	{
		return 0;
	}

	double value = _predictionLC[level+1];
	float sum=0;

	const NeighborsMap & neighbors = s->getNeighbors();
	for(NeighborsMap::const_iterator iter=neighbors.begin(); iter!= neighbors.end(); ++iter)
	{
		int index = uValue(likelihoodIds, iter->first, -1);
		if(index >= 0)
		{
			bool alreadyAdded = false;
			// the value can be already added in the recursion
			if(value > prediction->data.fl[row + index*prediction->cols])
			{
				sum -= prediction->data.fl[row + index*prediction->cols];
				prediction->data.fl[row + index*prediction->cols] = value;
				sum += value;
			}
			else
			{
				alreadyAdded = true;
			}

			if(!alreadyAdded && level+1 < _predictionLC.size())
			{
				sum += addNeighborProb(prediction, row, memory, likelihoodIds, memory->getSignature(iter->first), level+1);
			}
		}
		else
		{
			//ULOGGER_DEBUG("BayesFilter::generatePrediction(...) F (id %d) Not found for loop %d", loopSign->getNeighborForward(), loopClosureId);
		}
	}
	return sum;
}


} // namespace rtabmap
