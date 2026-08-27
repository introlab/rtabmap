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

#include "bayes/PredictionModel.h"

#include "rtabmap/core/Memory.h"
#include "rtabmap/utilite/UtiLite.h"

namespace rtabmap {
namespace bayes {

// format = {Virtual place, Loop closure, level1, level2, l3, l4...}
bool PredictionModel::set(const std::string & prediction)
{
	bool set = false;
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
			values_ = tmpValues;
			set = true;
		}
	}
	total_ = 0.0f;
	for(unsigned int j=0; j<values_.size(); ++j)
	{
		total_ += values_[j];
		if(j==0 || values_[j] < epsilon_)
		{
			epsilon_ = values_[j];
		}
	}
	if(!values_.empty())
	{
		UDEBUG("predictionEpsilon = %f", epsilon_);
	}
	return set;
}

std::string PredictionModel::str() const
{
	std::string values;
	for(unsigned int i=0; i<values_.size(); ++i)
	{
		values.append(uNumber2Str(values_[i]));
		if(i+1 < values_.size())
		{
			values.append(" ");
		}
	}
	return values;
}

// A column of the prediction matrix, given as a pointer to its first value and the
// step between two of them: the matrix stores a column strided by its width, while the
// sparse build below fills one contiguous column at a time. Both go through this and
// through BayesFilter::normalize(), so that the probabilities cannot end up differing
// between the two.
float PredictionModel::addNeighborProb(float * column, size_t stride,
			const std::map<int, int> & neighbors, const IdToIndexMap & idToIndex) const
{
	float sum=0.0f;
	for(std::map<int, int>::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
	{
		if(iter->first>=0)
		{
			IdToIndexMap::const_iterator jter = idToIndex.find(iter->first);
			if(jter != idToIndex.end())
			{
				UASSERT((iter->second+1) < (int)values_.size());
				sum += column[jter->second*stride] = values_[iter->second+1];
			}
		}
	}
	return sum;
}

void PredictionModel::normalize(float * column, size_t stride, int size, unsigned int index, float addedProbabilitiesSum, bool virtualPlaceUsed) const
{
	UASSERT(index < (unsigned int)size);

	int cols = size;
	// ADD values of not found neighbors to loop closure
	if(addedProbabilitiesSum < total_-values_[0])
	{
		float delta = total_-values_[0]-addedProbabilitiesSum;
		column[index*stride] += delta;
		addedProbabilitiesSum+=delta;
	}

	float allOtherPlacesValue = 0;
	if(total_ < 1)
	{
		allOtherPlacesValue = 1.0f - total_;
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
	float maxNorm = 1 - (virtualPlaceUsed?values_[0]:0); // 1 - virtual place probability
	if(addedProbabilitiesSum<maxNorm-0.0001 || addedProbabilitiesSum>maxNorm+0.0001)
	{
		for(int j=virtualPlaceUsed?1:0; j<cols; ++j)
		{
			column[j*stride] *= maxNorm / addedProbabilitiesSum;
			if(column[j*stride] < epsilon_)
			{
				column[j*stride] = 0.0f;
			}
		}
		addedProbabilitiesSum = maxNorm;
	}

	// ADD virtual place prob
	if(virtualPlaceUsed)
	{
		column[0] = values_[0];
		addedProbabilitiesSum += column[0];
	}

	//debug
	//for(int j=0; j<cols; ++j)
	//{
	//	ULOGGER_DEBUG("test col=%d = %f", i, prediction.data.fl[i + j*cols]);
	//}

	// Left out of the coverage report: no input gets here. Whatever the column held, the
	// scaling above leaves addedProbabilitiesSum at maxNorm, which is 1 without the virtual
	// place and 1 minus its probability with it -- and that probability is then added back.
	// It is kept as a canary for whoever changes the arithmetic above.
	if(addedProbabilitiesSum<0.99 || addedProbabilitiesSum > 1.01)
	{
		UWARN("Prediction is not normalized sum=%f", addedProbabilitiesSum); // LCOV_EXCL_LINE
	}
}

// The column of the virtual place, the hypothesis of being at a location that was
// never visited: the probability of moving again to a new one, then the rest split
// equally over the visited ones.
void PredictionModel::fillVirtualPlaceColumn(float * column, size_t stride, int size) const
{
	if(virtualPlacePrior_ > 0)
	{
		if(size>1) // The first must be the virtual place
		{
			column[0] = virtualPlacePrior_;
			float val = (1.0-virtualPlacePrior_)/(size-1);
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
		// when virtualPlacePrior_=0, set all priors to the same value
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

// The neighbors of a location within the depth of the prediction model, and the
// locations that are at margin 0 of it, meaning the same place: their columns all hold
// the probabilities of this same neighborhood. Shared by the dense and the sparse
// builds, this being the part that reads the graph.
//
// cache is filled when not null, for updatePrediction() to reuse.
std::map<int, int> resolveNeighbors(const Memory * memory, int id, int maxDepth,
		const IdToIndexMap & idToIndexMap, std::list<int> & idsAtMargin0, NeighborsCache * cache)
{
	std::map<int, int> neighbors = memory->getNeighborsId(id, maxDepth, 0, false, false, true, true);

	if(cache)
	{
		uInsert(*cache, std::make_pair(id, neighbors));
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

// The neighborhood of a location, from the cache the incremental update needs, adding it
// there and to the neighborhoods of its own neighbors when it is not there yet.
const std::map<int, int> & cachedNeighbors(const Memory * memory, int id, int maxDepth,
		NeighborsCache & cache)
{
	std::map<int, std::map<int, int> >::const_iterator iter = cache.find(id);
	if(iter == cache.end())
	{
		std::map<int, int> neighbors = memory->getNeighborsId(id, maxDepth, 0, false, false, true, true);
		for(std::map<int, int>::iterator jter=neighbors.begin(); jter!=neighbors.end(); ++jter)
		{
			std::map<int, std::map<int, int> >::iterator kter = cache.find(jter->first);
			if(kter != cache.end())
			{
				uInsert(kter->second, std::make_pair(id, jter->second));
			}
		}
		iter = cache.insert(std::make_pair(id, neighbors)).first;
	}
	return iter->second;
}

} // namespace bayes
} // namespace rtabmap
