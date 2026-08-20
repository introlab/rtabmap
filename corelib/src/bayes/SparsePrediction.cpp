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

#include "bayes/SparsePrediction.h"

#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/utilite/UtiLite.h"

namespace rtabmap {
namespace bayes {

void SparsePrediction::clear()
{
	columns_.clear();
	values_.clear();
	ids_.clear();
	used_ = 0;
}

unsigned long SparsePrediction::memoryUsed() const
{
	return values_.capacity() * sizeof(std::pair<int, float>)
			+ columns_.capacity() * sizeof(Column)
			+ ids_.capacity() * sizeof(int);
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
void SparsePrediction::takeColumn(std::vector<float> & column, int index, bool withRoomToGrow)
{
	size_t count = 0;
	for(size_t row=0; row<column.size(); ++row)
	{
		if(column[row] != 0.0f)
		{
			++count;
		}
	}

	Column & slot = columns_[index];
	used_ -= slot.size;
	if(count > slot.capacity)
	{
		slot.offset = values_.size();
		slot.capacity = withRoomToGrow ? count + count/8 + 4 : count;
		values_.resize(slot.offset + slot.capacity);
	}
	slot.size = count;
	used_ += count;

	size_t i = slot.offset;
	for(size_t row=0; row<column.size(); ++row)
	{
		if(column[row] != 0.0f)
		{
			values_[i++] = std::make_pair((int)row, column[row]);
			column[row] = 0.0f;
		}
	}
}

// Packs the columns back into the order they are multiplied in, giving each the room it
// needs and no more. Called when the room left behind by rebuilt columns has grown to a
// quarter of what is in use, and at the end of a full build, whose columns are not built in
// the order of their index.
void SparsePrediction::compact()
{
	std::vector<std::pair<int, float> > packed;
	packed.reserve(used_);
	for(size_t i=0; i<columns_.size(); ++i)
	{
		Column & slot = columns_[i];
		const size_t offset = packed.size();
		packed.insert(packed.end(),
				values_.begin()+slot.offset,
				values_.begin()+slot.offset+slot.size);
		slot.offset = offset;
		slot.capacity = slot.size;
	}
	values_.swap(packed);
}

// The prediction built in its sparse form, the matrix never being allocated.
//
// A column of the prediction only holds the neighbors of one location within the depth of
// the prediction model, so on a large map the matrix is mostly zeros, while holding it
// costs the size of the working memory squared against the far smaller size of the values
// in it. Each column is built in a buffer of its own instead, through the same
// addNeighborProb() and normalize() as the dense build, and only its non zero values are
// kept. Every column keeps the room it was given in values_, so that update() can rebuild
// one of them without moving the others.
//
// The columns are not built in the order of their index: a column is built for every
// location at margin 0 of the one being expanded, so several are built at once.
//
// Returns false when the prediction would not be sparse, which the caller has to answer by
// building the dense matrix. Happens when the values of the model sum to less than 1, as
// normalize() then spreads the missing probability over every zero of a column.
bool SparsePrediction::generate(const PredictionModel & model, const Memory * memory,
		const std::vector<int> & ids, NeighborsCache * cache)
{
	UASSERT(memory && model.valid() && ids.size());

	UTimer timer;
	this->clear();

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
	// keep.
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
					memory, ids[i], model.depth(), idToIndexMap, idsLoopMargin, 0);
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

	columns_.assign(size, Column());
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
					memory, ids[i], model.depth(), idToIndexMap, idsLoopMargin, cache);

			// same neighbor tree for loop signatures (margin = 0)
			for(std::list<int>::iterator iter=idsLoopMargin.begin(); iter!=idsLoopMargin.end(); ++iter)
			{
				if(cache)
				{
					uInsert(*cache, std::make_pair(*iter, neighbors));
				}
				const int index = idToIndexMap.at(*iter);
				const float sum = model.addNeighborProb(&column[0], 1, neighbors, idToIndexMap);
				model.normalize(&column[0], 1, size, index, sum, ids[0]<0);
				this->takeColumn(column, index, false);
				idsDone.insert(*iter);
			}
		}
		else
		{
			model.fillVirtualPlaceColumn(&column[0], 1, size);
			this->takeColumn(column, i, false);
		}
	}
	// The columns were not built in the order of their index, so they are packed into it.
	this->compact();
	ids_ = ids;

	const size_t nnz = used_;
	UDEBUG("Sparse prediction: %ld/%ld values (%.2f%%), %ld MB against the %ld MB of the "
		   "matrix, built in %fs",
			(long)nnz, (long)size*size, 100.0*double(nnz)/(double(size)*double(size)),
			(long)(this->memoryUsed()/1048576),
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
bool SparsePrediction::update(const PredictionModel & model, const Memory * memory,
		const std::vector<int> & newIds, NeighborsCache & cache)
{
	const std::vector<int> & oldIds = ids_;
	if(oldIds.empty() ||
	   newIds.size() <= oldIds.size() ||
	   columns_.size() != oldIds.size() ||
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

	columns_.resize(size);   // the appended columns start out empty
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
		const std::map<int, int> & neighbors = cachedNeighbors(memory, newIds[i], model.depth(), cache);
		const float sum = model.addNeighborProb(&column[0], 1, neighbors, newIdToIndexMap);
		model.normalize(&column[0], 1, size, (int)i, sum, newIds[0]<0);
		this->takeColumn(column, (int)i, true);
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
		const NeighborsCache::const_iterator kter = cache.find(*iter);
		UASSERT_MSG(kter != cache.end(),
				uFormat("Did not find %d (current index size=%d)", *iter, (int)cache.size()).c_str());
		const int index = newIdToIndexMap.at(*iter);
		const float sum = model.addNeighborProb(&column[0], 1, kter->second, newIdToIndexMap);
		model.normalize(&column[0], 1, size, index, sum, newIds[0]<0);
		this->takeColumn(column, index, true);
	}

	// The virtual place shares what is left of its probability over the visited locations,
	// so its column depends on how many of them there are.
	if(newIds[0] < 0)
	{
		model.fillVirtualPlaceColumn(&column[0], 1, size);
		this->takeColumn(column, 0, true);
	}

	// The room left behind by the columns that outgrew their slot, once it is a quarter of
	// what is in use.
	const size_t waste = values_.size() - used_;
	const bool compacted = waste > used_/4;
	if(compacted)
	{
		this->compact();
	}
	ids_ = newIds;

	UDEBUG("Sparse prediction: %d locations appended, %d columns rebuilt of %d, %ld values, "
		   "%ld left behind%s, updated in %fs",
			(int)(newIds.size()-oldIds.size()), (int)idsToUpdate.size(), size,
			(long)used_, (long)waste, compacted?" (packed again)":"",
			timer.ticks());
	return true;
}

void SparsePrediction::multiply(const std::vector<float> & posterior, std::vector<float> & prior) const
{
	const size_t size = columns_.size();
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
		const Column & slot = columns_[col];
		for(size_t i=slot.offset; i<slot.offset+slot.size; ++i)
		{
			priorPtr[values_[i].first] += values_[i].second * weight;
		}
	}
}

} // namespace bayes
} // namespace rtabmap
