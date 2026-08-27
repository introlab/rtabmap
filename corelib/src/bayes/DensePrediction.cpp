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

#include "bayes/DensePrediction.h"

#include "rtabmap/core/Memory.h"
#include "rtabmap/utilite/UtiLite.h"

#include <set>
#if __cplusplus >= 201103L
#include <unordered_set>
#endif

namespace rtabmap {
namespace bayes {

const cv::Mat & DensePrediction::generate(const PredictionModel & model, const Memory * memory,
		const std::vector<int> & ids, bool fullUpdate, NeighborsCache * cache)
{
	// The update carries the matrix already there over, so it can only be done against the
	// locations that matrix is built for. There is none to carry over when the sparse form has
	// been used since, or when the model changed, and every column is built again.
	if(!fullUpdate && !matrix_.empty() && ids_.size() == (size_t)matrix_.cols)
	{
		matrix_ = this->update(model, memory, ids_, ids, cache);
	}
	else
	{
		matrix_ = this->generateFull(model, memory, ids, fullUpdate?0:cache);
	}
	ids_ = ids;
	return matrix_;
}

void DensePrediction::multiply(const std::vector<float> & posterior, std::vector<float> & prior) const
{
	UASSERT(!matrix_.empty());
	UASSERT_MSG(matrix_.cols == (int)posterior.size(),
			uFormat("posterior=%d prediction=%d", (int)posterior.size(), matrix_.cols).c_str());

	// A header over the posterior, so the multiplication reads it where it is. The product
	// itself is left to OpenCV to allocate: asked to write into a matrix of ours it takes a
	// path orders of magnitude slower, and copying the result back is only one value per
	// location.
	const cv::Mat posteriorMat((int)posterior.size(), 1, CV_32FC1, (void*)&posterior[0]);
	const cv::Mat priorMat = matrix_ * posteriorMat;
	prior.assign((const float *)priorMat.data, (const float *)priorMat.data + priorMat.rows);
}

unsigned long DensePrediction::memoryUsed() const
{
	unsigned long memory = ids_.capacity() * sizeof(int);
	if(!matrix_.empty())
	{
		memory += (unsigned long)(matrix_.total() * matrix_.elemSize());
	}
	return memory;
}

// The matrix built column by column, every column from the neighborhood of one location.
cv::Mat DensePrediction::generateFull(const PredictionModel & model, const Memory * memory,
		const std::vector<int> & ids, NeighborsCache * cache) const
{
	UASSERT(memory &&
		   model.values().size() >= 2 &&
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
	UDEBUG("model.values().size()=%d",(int)model.values().size());
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
						memory, ids[i], model.depth(), idToIndexMap, idsLoopMargin, cache);

				// same neighbor tree for loop signatures (margin = 0)
				for(std::list<int>::iterator iter = idsLoopMargin.begin(); iter!=idsLoopMargin.end(); ++iter)
				{
					if(cache)
					{
						uInsert(*cache, std::make_pair(*iter, neighbors));
					}

					float sum = 0.0f; // sum values added
					int index = idToIndexMap.at(*iter);
					float * column = (float*)prediction.data + index;
					sum += model.addNeighborProb(column, cols, neighbors, idToIndexMap);
					idsDone.insert(*iter);
					model.normalize(column, cols, cols, index, sum, ids[0]<0);
				}
			}
			else
			{
				// Set the virtual place prior
				model.fillVirtualPlaceColumn((float*)prediction.data + i, cols, cols);
			}
		}
	}

	ULOGGER_DEBUG("time = %fs", timerGlobal.ticks());

	return prediction;
}

cv::Mat DensePrediction::update(const PredictionModel & model, const Memory * memory,
		const std::vector<int> & oldIds, const std::vector<int> & newIds,
		NeighborsCache * cache) const
{
	UTimer timer;
	UDEBUG("");

	UASSERT(memory &&
		oldIds.size() &&
		newIds.size() &&
		oldIds.size() == (unsigned int)matrix_.cols &&
		oldIds.size() == (unsigned int)matrix_.rows);

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
			(*cache).erase(oldIds[i]);
			UDEBUG("removed id=%d at oldIndex=%d", oldIds[i], i);
		}
	}
	UDEBUG("time getting removed ids = %fs", timer.restart());

	bool oldAllCopied = false;
	if(removedIds.empty() &&
		newIds.size() > oldIds.size() &&
		memcmp(oldIds.data(), newIds.data(), oldIds.size()*sizeof(int)) == 0)
	{
		matrix_.copyTo(cv::Mat(prediction, cv::Range(0, matrix_.rows), cv::Range(0, matrix_.cols)));
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
				unsigned int cols = matrix_.cols;
				int count = 0;
				for(unsigned int j=0; j<cols; ++j)
				{
					if(j!=i && removedIds.find(oldIds[j]) == removedIds.end())
					{
						//UDEBUG("to update id=%d from id=%d removed (value=%f)", oldIds[j], oldIds[i], ((const float *)matrix_.data)[i + j*cols]);
						idsToUpdate.insert(oldIds[j]);
						++count;
					}
				}
				UDEBUG("From removed id %d, %d neighbors to update.", oldIds[i], count);
			}
		}
		if(i<newIds.size() && oldIdsSet.find(newIds[i]) == oldIdsSet.end())
		{
			if((*cache).find(newIds[i]) == (*cache).end())
			{
				std::map<int, int> neighbors = memory->getNeighborsId(newIds[i], model.depth(), 0, false, false, true, true);

				for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
				{
					std::map<int, std::map<int, int> >::iterator jter = (*cache).find(iter->first);
					if(jter != (*cache).end())
					{
						uInsert(jter->second, std::make_pair(newIds[i], iter->second));
					}
				}
				(*cache).insert(std::make_pair(newIds[i], neighbors));
			}
			const std::map<int, int> & neighbors = (*cache).at(newIds[i]);
			//std::map<int, int> neighbors = memory->getNeighborsId(newIds[i], model.depth(), 0, false, false, true, true);

			float * column = (float*)prediction.data + i;
			float sum = model.addNeighborProb(column, prediction.cols, neighbors, newIdToIndexMap);
			model.normalize(column, prediction.cols, prediction.cols, i, sum, newIds[0]<0);

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
			std::map<int, std::map<int, int> >::iterator kter = (*cache).find(id);
			UASSERT_MSG(kter != (*cache).end(), uFormat("Did not find %d (current index size=%d)", id, (int)(*cache).size()).c_str());
			const std::map<int, int> & neighbors = kter->second;
			//std::map<int, int> neighbors = memory->getNeighborsId(id, model.depth(), 0, false, false, true, true);
			e1+=t1.ticks();

			float * column = (float*)prediction.data + index;
			float sum = model.addNeighborProb(column, prediction.cols, neighbors, newIdToIndexMap);
			e3+=t1.ticks();

			model.normalize(column, prediction.cols, prediction.cols, index, sum, newIds[0]<0);
			++modified;
			e4+=t1.ticks();
		}
	}
	UDEBUG("time updating modified/added %d ids = %fs (e0=%f e1=%f e2=%f e3=%f e4=%f)", (int)idsToUpdate.size(), timer.restart(), e0, e1, e2, e3, e4);

	int copied = 0;
	if(!oldAllCopied)
	{
		//UDEBUG("oldIds.size()=%d, matrix_.cols=%d, matrix_.rows=%d", oldIds.size(), matrix_.cols, matrix_.rows);
		//UDEBUG("newIdToIndexMap.size()=%d, prediction.cols=%d, prediction.rows=%d", newIdToIndexMap.size(), prediction.cols, prediction.rows);
		// copy not changed probabilities
		for(unsigned int i=0; i<oldIds.size(); ++i)
		{
			if(oldIds[i]>0 && removedIds.find(oldIds[i]) == removedIds.end() && idsToUpdate.find(oldIds[i]) == idsToUpdate.end())
			{
				for(int j=0; j<matrix_.cols; ++j)
				{
					if(oldIds[j]>0 && removedIds.find(oldIds[j]) == removedIds.end())
					{
						//UDEBUG("i=%d, j=%d", i, j);
						//UDEBUG("oldIds[i]=%d, oldIds[j]=%d", oldIds[i], oldIds[j]);
						//UDEBUG("newIdToIndexMap.at(oldIds[i])=%d", newIdToIndexMap.at(oldIds[i]));
						//UDEBUG("newIdToIndexMap.at(oldIds[j])=%d", newIdToIndexMap.at(oldIds[j]));
						float v = ((const float *)matrix_.data)[i + j*matrix_.cols];
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
			((float*)prediction.data)[0] = model.virtualPlacePrior();
			float val = (1.0-model.virtualPlacePrior())/(prediction.cols-1);
			for(int j=1; j<prediction.cols; j++)
			{
				((float*)prediction.data)[j*prediction.cols] = val;
				((float*)prediction.data)[j] = model.values()[0];
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

} // namespace bayes
} // namespace rtabmap
