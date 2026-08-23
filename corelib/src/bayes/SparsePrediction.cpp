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

cv::Mat SparsePrediction::toMatrix() const
{
	const int size = (int)columns_.size();
	cv::Mat matrix = cv::Mat::zeros(size, size, CV_32FC1);
	for(int col=0; col<size; ++col)
	{
		const Column & slot = columns_[col];
		for(size_t i=slot.offset; i<slot.offset+slot.size; ++i)
		{
			matrix.at<float>(values_[i].first, col) = values_[i].second;
		}
	}
	return matrix;
}

unsigned long SparsePrediction::memoryUsed() const
{
	return values_.capacity() * sizeof(std::pair<int, float>)
			+ columns_.capacity() * sizeof(Column)
			+ ids_.capacity() * sizeof(int);
}

// Takes the non zero values of a freshly built column into the prediction, and leaves the
// buffer zeroed for the next one, which saves clearing the whole of it every time.
//
// The values of every column live in one array, so that the multiplication reads them the
// way memory likes to be read. A column keeps the room it was given: rebuilt into fewer
// values it stays where it is, rebuilt into more than it has room for it is put at the end
// and the room it had is left behind, to be recovered by compact(). Asking
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
// Always built, whatever its columns come to hold: the caller asked for the prediction sparse
// and gets it sparse, so that what it measures is the sparse form and not a fallback. The one
// prediction with nothing sparse to keep, of a model whose values sum to less than 1,
// normalize() spreading the difference over every zero of a column, never reaches here: the
// caller answers that one with the matrix without asking.
void SparsePrediction::generate(const PredictionModel & model, const Memory * memory,
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
}

// One column, from the neighborhood of the location it is for. Read from the cache, which
// generate() filled and which the graph is only walked again for when a location came back
// from long-term memory after its neighborhood was dropped.
void SparsePrediction::buildColumn(const PredictionModel & model, const Memory * memory, int id,
		int index, const std::vector<int> & ids, const IdToIndexMap & idToIndex,
		std::vector<float> & buffer, NeighborsCache & cache)
{
	const std::map<int, int> & neighbors = cachedNeighbors(memory, id, model.depth(), cache);
	const float sum = model.addNeighborProb(&buffer[0], 1, neighbors, idToIndex);
	model.normalize(&buffer[0], 1, (int)ids.size(), index, sum, ids[0]<0);
	this->takeColumn(buffer, index, true);
}

// Carries the prediction over to the locations of an iteration, which costs the columns whose
// contents changed rather than a walk of the graph per column.
//
// Returns false when there is nothing to carry over, which the caller answers by calling
// generate().
bool SparsePrediction::update(const PredictionModel & model, const Memory * memory,
		const std::vector<int> & newIds, NeighborsCache & cache)
{
	if(ids_.empty() || newIds.empty() || columns_.size() != ids_.size())
	{
		return false;
	}

	// Appended to, or changed in any other way: the first keeps every index, the second has
	// to lay the columns out again.
	const bool appendedTo =
			newIds.size() > ids_.size() &&
			memcmp(ids_.data(), newIds.data(), ids_.size()*sizeof(int)) == 0;
	return appendedTo
			? this->updateAppended(model, memory, newIds, cache)
			: this->updateRemapped(model, memory, newIds, cache);
}

// The same prediction after locations were appended, without building it again.
//
// Every location that was already there keeps its index, so the columns already built
// still apply: only the ones the new locations reach have to be built again, and the
// column of the virtual place, whose values are shared out over however many locations
// there are. What a column holds does not otherwise depend on how many there are, the
// model summing to 1 leaving normalize() nothing to spread over the others.
bool SparsePrediction::updateAppended(const PredictionModel & model, const Memory * memory,
		const std::vector<int> & newIds, NeighborsCache & cache)
{
	UTimer timer;
	const std::vector<int> & oldIds = ids_;
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
		// Every appended location is a visited one: the virtual place is the first of them
		// and an append keeps the index of everything that was already there.
		UASSERT(newIds[i] > 0);
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
		this->buildColumn(model, memory, *iter, newIdToIndexMap.at(*iter),
				newIds, newIdToIndexMap, column, cache);
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
	const size_t appended = newIds.size()-oldIds.size();
	ids_ = newIds;

	UDEBUG("Sparse prediction: %d locations appended, %d columns rebuilt of %d, %ld values, "
		   "%ld left behind%s, updated in %fs",
			(int)appended, (int)idsToUpdate.size(), size,
			(long)used_, (long)waste, compacted?" (packed again)":"",
			timer.ticks());
	return true;
}

// The same prediction after the locations changed in any other way than being appended to:
// locations gone from the working memory as it is capped, locations back from long-term
// memory in the middle of the ones already there, or both at once.
//
// The index of a location moves, so the columns are laid out again -- but a column is only
// built again when what goes in it changed, which is when:
//   * the location was not there before, so it has no column yet;
//   * one of those is now part of its neighborhood, so it gains a value;
//   * it shared its probability with a location that is gone, which normalize() now shares
//     out over the ones that remain.
// Every other column is the same values at another row, which is a copy. That is what
// separates this from generate(): the graph is walked for the columns that changed, not for
// every one of them.
bool SparsePrediction::updateRemapped(const PredictionModel & model, const Memory * memory,
		const std::vector<int> & newIds, NeighborsCache & cache)
{
	UTimer timer;
	const std::vector<int> & oldIds = ids_;
	const int size = (int)newIds.size();

	// The virtual place appearing or disappearing changes every column, normalize() holding
	// back its probability on all of them, so there would be nothing to carry over.
	if((oldIds[0] < 0) != (newIds[0] < 0))
	{
		return false;
	}

	IdToIndexMap newIdToIndexMap;
	IdToIndexMap oldIdToIndexMap;
#if __cplusplus >= 201103L
	newIdToIndexMap.reserve(newIds.size());
	oldIdToIndexMap.reserve(oldIds.size());
#endif
	for(int i=0; i<size; ++i)
	{
		if(newIds[i]>0)
		{
			newIdToIndexMap[newIds[i]] = i;
		}
	}
	for(size_t i=0; i<oldIds.size(); ++i)
	{
		if(oldIds[i]>0)
		{
			oldIdToIndexMap[oldIds[i]] = (int)i;
		}
	}

	// Where each location went, and which ones are gone. The virtual place is the first of
	// both, so it does not move.
	std::vector<int> oldToNew(oldIds.size(), -1);
	size_t removed = 0;
	for(size_t i=0; i<oldIds.size(); ++i)
	{
		if(oldIds[i] <= 0)
		{
			oldToNew[i] = 0;
			continue;
		}
		const IdToIndexMap::const_iterator iter = newIdToIndexMap.find(oldIds[i]);
		if(iter == newIdToIndexMap.end())
		{
			// Its neighborhood is no longer ours to keep, as the dense update does too.
			cache.erase(oldIds[i]);
			++removed;
		}
		else
		{
			oldToNew[i] = iter->second;
		}
	}

	// The locations that were not there before, and the ones whose neighborhood they are
	// part of.
	std::set<int> idsToBuild;
	for(int i=0; i<size; ++i)
	{
		if(newIds[i] <= 0 || oldIdToIndexMap.find(newIds[i]) != oldIdToIndexMap.end())
		{
			continue;
		}
		idsToBuild.insert(newIds[i]);
		const std::map<int, int> & neighbors = cachedNeighbors(memory, newIds[i], model.depth(), cache);
		for(std::map<int, int>::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
		{
			if(iter->first > 0 &&
			   newIdToIndexMap.find(iter->first) != newIdToIndexMap.end() &&
			   oldIdToIndexMap.find(iter->first) != oldIdToIndexMap.end())
			{
				idsToBuild.insert(iter->first);
			}
		}
	}

	// And the ones holding a value on a row that is gone.
	if(removed)
	{
		for(size_t i=0; i<oldIds.size(); ++i)
		{
			if(oldIds[i] <= 0 || oldToNew[i] < 0 ||
			   idsToBuild.find(oldIds[i]) != idsToBuild.end())
			{
				continue;
			}
			const Column & slot = columns_[i];
			for(size_t v=slot.offset; v<slot.offset+slot.size; ++v)
			{
				if(oldToNew[values_[v].first] < 0)
				{
					idsToBuild.insert(oldIds[i]);
					break;
				}
			}
		}
	}

	// The columns that are carried over, at their new index and packed as they go: the room
	// left behind by the ones that are gone or built again is not carried with them.
	std::vector<Column> keptColumns(size);
	std::vector<std::pair<int, float> > keptValues;
	keptValues.reserve(used_);
	size_t keptUsed = 0;
	size_t carried = 0;
	for(size_t i=0; i<oldIds.size(); ++i)
	{
		const int index = oldToNew[i];
		if(index < 0 || oldIds[i] <= 0 || idsToBuild.find(oldIds[i]) != idsToBuild.end())
		{
			continue;
		}
		const Column & slot = columns_[i];
		Column & kept = keptColumns[index];
		kept.offset = keptValues.size();
		kept.size = slot.size;
		kept.capacity = slot.size;
		for(size_t v=slot.offset; v<slot.offset+slot.size; ++v)
		{
			// The rows of a column are ascending, and so are both id vectors, so a remapped
			// row stays after the one before it.
			keptValues.push_back(std::make_pair(oldToNew[values_[v].first], values_[v].second));
		}
		keptUsed += slot.size;
		++carried;
	}
	columns_.swap(keptColumns);
	values_.swap(keptValues);
	used_ = keptUsed;

	std::vector<float> column(size, 0.0f);
	for(std::set<int>::const_iterator iter=idsToBuild.begin(); iter!=idsToBuild.end(); ++iter)
	{
		this->buildColumn(model, memory, *iter, newIdToIndexMap.at(*iter),
				newIds, newIdToIndexMap, column, cache);
	}

	// The virtual place shares what is left of its probability over the visited locations,
	// so its column depends on how many of them there are.
	if(newIds[0] < 0)
	{
		model.fillVirtualPlaceColumn(&column[0], 1, size);
		this->takeColumn(column, 0, true);
	}

	const size_t waste = values_.size() - used_;
	const bool compacted = waste > used_/4;
	if(compacted)
	{
		this->compact();
	}
	ids_ = newIds;

	UDEBUG("Sparse prediction: %d locations removed, %d columns carried over and %d built "
		   "again of %d, %ld values, %ld left behind%s, updated in %fs",
			(int)removed, (int)carried, (int)idsToBuild.size(), size,
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
