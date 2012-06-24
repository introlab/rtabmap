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

#include "rtabmap/core/BayesFilter.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Parameters.h"
#include <iostream>

#include "utilite/UtiLite.h"

namespace rtabmap {

BayesFilter::BayesFilter(const ParametersMap & parameters) :
	_virtualPlacePrior(Parameters::defaultBayesVirtualPlacePriorThr()),
	_predictionOnNonNullActionsOnly(Parameters::defaultBayesPredictionOnNonNullActionsOnly())
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
	if((iter=parameters.find(Parameters::kBayesPredictionOnNonNullActionsOnly())) != parameters.end())
	{
		_predictionOnNonNullActionsOnly = uStr2Bool((*iter).second.c_str());
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
		UERROR("The number of values < 2 (prediction=\"%s\")", prediction.c_str());
	}
	else
	{
		std::vector<double> tmpValues(strValues.size());
		int i=0;
		bool valid = true;
		for(std::list<std::string>::iterator iter = strValues.begin(); iter!=strValues.end(); ++iter)
		{
			tmpValues[i] = std::atof((*iter).c_str());
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

	cv::Mat prediction;
	cv::Mat prior;
	cv::Mat posterior;

	float sum = 0;
	int j=0;
	// Recursive Bayes estimation...
	// STEP 1 - Prediction : Prior*lastPosterior
	prediction = cvCreateMat(likelihood.size(), likelihood.size(), CV_32FC1);
	if(this->generatePrediction(prediction, memory, uKeys(likelihood)))
	{
		ULOGGER_DEBUG("STEP1-generate prior=%fs, rows=%d, cols=%d", timer.ticks(), prediction.rows, prediction.cols);
		//std::cout << "Prediction=" << prediction << std::endl;

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
		prior = prediction * posterior;
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

	return _posterior;
}

bool BayesFilter::generatePrediction(cv::Mat & prediction, const Memory * memory, const std::vector<int> & ids) const
{
	ULOGGER_DEBUG("");
	UTimer timer;
	timer.start();
	UTimer timerGlobal;
	timerGlobal.start();

	if(!memory ||
	   prediction.empty() ||
	   prediction.rows != prediction.cols ||
	   (unsigned int)prediction.rows != ids.size() ||
	   _predictionLC.size() < 2 ||
	   !ids.size())
	{
		ULOGGER_ERROR( "fail");
		return false;
	}

	std::map<int, int> idToIndexMap;
	for(unsigned int i=0; i<ids.size(); ++i)
	{
		if(ids[i] == 0)
		{
			UFATAL("Signature id is null ?!?");
		}
		idToIndexMap.insert(idToIndexMap.end(), std::make_pair(ids[i], i));
	}

	//int rows = prediction.rows;
	prediction = cv::Mat::zeros(prediction.rows, prediction.cols, prediction.type());
	int cols = prediction.cols;

	// Each prior is a column vector
	ULOGGER_DEBUG("_predictionLC.size()=%d",_predictionLC.size());
	for(unsigned int i=0; i<ids.size(); ++i)
	{
		int loopSignId = ids[i];
		if(loopSignId > 0)
		{
			// Set high values (gaussians curves) to loop closure neighbors

			float sum = 0.0f; // sum values added

			float totalModelValues = 0.0f;
			for(unsigned int j=0; j<_predictionLC.size(); ++j)
			{
				totalModelValues += _predictionLC[j];
			}

			// ADD prob for each neighbors
			double dbAccessTime = 0.0;
			std::map<int, int> neighbors = memory->getNeighborsId(dbAccessTime, loopSignId, _predictionLC.size()-1, 0, _predictionOnNonNullActionsOnly);
			sum += this->addNeighborProb(prediction, i, neighbors, idToIndexMap);
			// ADD values of not found neighbors to loop closure
			if(sum < totalModelValues-_predictionLC[0])
			{
				float delta = totalModelValues-_predictionLC[0]-sum;
				((float*)prediction.data)[i + i*cols] += delta;
				sum+=delta;
			}

			float allOtherPlacesValue = 0;
			if(totalModelValues < 1)
			{
				allOtherPlacesValue = 1.0f - totalModelValues;
			}

			// Set all loop events to small values according to the model
			if(allOtherPlacesValue > 0 && cols>1)
			{
				float value = allOtherPlacesValue / float(cols - 1);
				for(int j=ids[0] < 0?1:0; j<cols; ++j)
				{
					if(((float*)prediction.data)[i + j*cols] == 0)
					{
						((float*)prediction.data)[i + j*cols] = value;
						sum += ((float*)prediction.data)[i + j*cols];
					}
				}
			}

			//normalize this row
			float maxNorm = 1 - (ids[0]<0?_predictionLC[0]:0); // 1 - virtual place probability
			if(sum<maxNorm-0.0001 || sum>maxNorm+0.0001)
			{
				for(int j=ids[0] < 0?1:0; j<cols; ++j)
				{
					((float*)prediction.data)[i + j*cols] *= maxNorm / sum;
				}
				sum = maxNorm;
			}

			// ADD virtual place prob
			if(ids[0] < 0)
			{
				((float*)prediction.data)[i] = _predictionLC[0];
				sum += ((float*)prediction.data)[i];
			}

			//debug
			//for(int j=0; j<cols; ++j)
			//{
			//	ULOGGER_DEBUG("test col=%d = %f", i, prediction.data.fl[i + j*cols]);
			//}

			if(sum<0.99 || sum > 1.01)
			{
				UWARN("Prediction is not normalized sum=%f", sum);
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

float BayesFilter::addNeighborProb(cv::Mat & prediction, unsigned int col, const std::map<int, int> & neighbors, const std::map<int, int> & idToIndexMap) const
{
	if((unsigned int)prediction.cols != idToIndexMap.size() ||
	   (unsigned int)prediction.rows != idToIndexMap.size())
	{
		UFATAL("Requirements no met");
	}

	float sum=0;
	for(std::map<int, int>::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
	{
		int index = uValue(idToIndexMap, iter->first, -1);
		if(index >= 0)
		{
			sum += ((float*)prediction.data)[col + index*prediction.cols] = _predictionLC[iter->second+1];
		}
	}
	return sum;
}


} // namespace rtabmap
