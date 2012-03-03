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

#include "Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/DBDriverFactory.h"
#include "rtabmap/core/DBDriver.h"
#include "utilite/UtiLite.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/SMState.h"
#include "Node.h"

namespace rtabmap {

const int Memory::kIdStart = 1;
const int Memory::kIdVirtual = -1;
const int Memory::kIdInvalid = 0;

Memory::Memory(const ParametersMap & parameters) :
	_dbDriver(0),
	_similarityThreshold(Parameters::defaultMemSimilarityThr()),
	_similarityOnlyLast(Parameters::defaultMemSimilarityOnlyLast()),
	_rawDataKept(Parameters::defaultMemRawDataKept()),
	_incrementalMemory(Parameters::defaultMemIncrementalMemory()),
	_maxStMemSize(Parameters::defaultMemMaxStMemSize()),
	_commonSignatureUsed(Parameters::defaultMemCommonSignatureUsed()),
	_recentWmRatio(Parameters::defaultMemRecentWmRatio()),
	_dataMergedOnRehearsal(Parameters::defaultMemDataMergedOnRehearsal()),
	_idCount(kIdStart),
	_lastSignature(0),
	_lastLoopClosureId(0),
	_memoryChanged(false)
{
	this->parseParameters(parameters);
}

bool Memory::init(const std::string & dbDriverName, const std::string & dbUrl, bool dbOverwritten, const ParametersMap & parameters)
{
	ULOGGER_DEBUG("");
	this->parseParameters(parameters);

	UEventsManager::post(new RtabmapEventInit("Clearing memory..."));
	this->clear();
	UEventsManager::post(new RtabmapEventInit("Clearing memory, done!"));

	if(_dbDriver)
	{
		UEventsManager::post(new RtabmapEventInit("Closing database connection..."));
		_dbDriver->closeConnection();
		UEventsManager::post(new RtabmapEventInit("Closing database connection, done!"));

		if(_dbDriver->getDriverName().compare(dbDriverName) != 0)
		{
			delete _dbDriver;
			_dbDriver = 0;
		}
	}

	if(_dbDriver == 0)
	{
		_dbDriver = DBDriverFactory::createDBDriver(dbDriverName, parameters); // inMemory
	}

	bool success = false;
	if(_dbDriver)
	{
		UEventsManager::post(new RtabmapEventInit(std::string("Connecting to database ") + dbUrl + "..."));
		if(_dbDriver->openConnection(dbUrl, dbOverwritten))
		{
			success = true;
			UEventsManager::post(new RtabmapEventInit(std::string("Connecting to database ") + dbUrl + ", done!"));

			// Load the last working memory...
			UEventsManager::post(new RtabmapEventInit(std::string("Loading last signatures...")));
			std::list<Signature*> dbSignatures;
			_dbDriver->loadLastSignatures(dbSignatures);
			for(std::list<Signature*>::reverse_iterator iter=dbSignatures.rbegin(); iter!=dbSignatures.rend(); ++iter)
			{
				_signatures.insert(std::pair<int, Signature *>((*iter)->id(), *iter));
				if(_stMem.size() <= _maxStMemSize)
				{
					_stMem.insert((*iter)->id());
				}
				else
				{
					_workingMem.insert((*iter)->id());
				}
			}
			UEventsManager::post(new RtabmapEventInit(std::string("Loading last signatures, done! (") + uNumber2Str(int(_workingMem.size() + _stMem.size())) + " loaded)"));

			// Assign the last signature
			if(_stMem.size()>0)
			{
				_lastSignature = uValue(_signatures, *_stMem.rbegin(), (Signature*)0);
			}

			// Last id
			_dbDriver->getLastSignatureId(_idCount);
			_idCount += 1;
		}
		else
		{
			UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kError, std::string("Connecting to database ") + dbUrl + ", path is invalid!"));
		}
	}
	else
	{
		_idCount = kIdStart;
	}

	_dbDriver->start();

	ULOGGER_DEBUG("ids start with %d", _idCount);
	return success;
}

Memory::~Memory()
{
	ULOGGER_DEBUG("");
	if(!_memoryChanged)
	{
		ULOGGER_DEBUG("");
		if(_dbDriver)
		{
			_dbDriver->closeConnection();
			delete _dbDriver;
			_dbDriver = 0;
		}
		this->clear();
	}
	else
	{
		ULOGGER_DEBUG("");
		this->clear();
		if(_dbDriver)
		{
			_dbDriver->emptyTrashes();
			_dbDriver->closeConnection();
			delete _dbDriver;
			_dbDriver = 0;
		}
	}
}

void Memory::parseParameters(const ParametersMap & parameters)
{
	ULOGGER_DEBUG("");
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kMemSimilarityThr())) != parameters.end())
	{
		this->setSimilarityThreshold(std::atof((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kMemSimilarityOnlyLast())) != parameters.end())
	{
		_similarityOnlyLast = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kMemRawDataKept())) != parameters.end())
	{
		_rawDataKept = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kMemIncrementalMemory())) != parameters.end())
	{
		_incrementalMemory = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kMemMaxStMemSize())) != parameters.end())
	{
		this->setMaxStMemSize(std::atoi((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kMemCommonSignatureUsed())) != parameters.end())
	{
		this->setCommonSignatureUsed(uStr2Bool((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kMemRecentWmRatio())) != parameters.end())
	{
		this->setRecentWmRatio(std::atof((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kMemDataMergedOnRehearsal())) != parameters.end())
	{
		_dataMergedOnRehearsal = uStr2Bool((*iter).second.c_str());
	}
	if(_dbDriver)
	{
		_dbDriver->parseParameters(parameters);
	}
}

void Memory::preUpdate()
{
	_signaturesAdded = 0;
}

bool Memory::update(const SMState * smState, std::map<std::string, float> & stats)
{
	ULOGGER_DEBUG("");
	UTimer timer;
	UTimer totalTimer;
	timer.start();
	float t;

	//============================================================
	// Pre update...
	//============================================================
	ULOGGER_DEBUG("pre-updating...");
	this->preUpdate();
	t=timer.ticks()*1000;
	stats.insert(std::pair<std::string, float>(std::string("TimingMem/Pre-update/ms"), t));
	ULOGGER_DEBUG("time preUpdate=%f ms", t);

	//============================================================
	// Create a signature with the image received.
	//============================================================
	Signature * signature = this->createSignature(this->getNextId(), smState, this->isRawDataKept());
	if (signature == 0)
	{
		UFATAL("Failed to create a signature");
	}

	// It will be added to the short-term memory, no need to delete it...
	if(smState)
	{
		this->addSignatureToStm(signature, smState->getActuators());
	}
	else
	{
		this->addSignatureToStm(signature);
	}
	_lastSignature = signature;

	if(_lastLoopClosureId == 0)
	{
		// If not set use the new one added
		_lastLoopClosureId = signature->id();
	}

	t=timer.ticks()*1000;
	stats.insert(std::pair<std::string, float>(std::string("TimingMem/Signature creation/ms"), t));
	ULOGGER_DEBUG("time creating signature=%f ms", t);

	//============================================================
	// Rehearsal step...
	// Compare with the X last signatures. If different, add this
	// signature like a parent to the memory tree, otherwise add
	// it as a child to the similar signature.
	//============================================================
	this->rehearsal(signature, stats);
	t=timer.ticks()*1000;
	stats.insert(std::pair<std::string, float>(std::string("TimingMem/Rehearsal/ms"), t));
	ULOGGER_DEBUG("time rehearsal=%f ms", t);

	//============================================================
	// Update the common signature
	//============================================================
	if(_commonSignatureUsed)
	{
		Signature * s = _getSignature(kIdVirtual);
		if(s == 0)
		{
			s = this->createSignature(kIdVirtual, 0); // Create a virtual place
			_signatures.insert(std::pair<int, Signature *>(s->id(), s));
			_workingMem.insert(s->id());
		}
	}
	else
	{
		// remove virtual signature
		Signature * s = _getSignature(kIdVirtual);
		if(s)
		{
			this->moveToTrash(s);
		}
	}

	//============================================================
	// Transfer the oldest signature of the short-term memory to the working memory
	//============================================================
	while(_stMem.size() > _maxStMemSize && _stMem.size())
	{
		_workingMem.insert(_workingMem.end(), *_stMem.begin());
		_stMem.erase(*_stMem.begin());
		++_signaturesAdded;
	}

	if(!_memoryChanged)
	{
		_memoryChanged = true;
	}

	UDEBUG("totalTimer = %fs", totalTimer.ticks());

	stats.insert(std::pair<std::string, float>(std::string("Memory/Last loop closure/"), _lastLoopClosureId));

	return true;
}

void Memory::setSimilarityThreshold(float similarityThreshold)
{
	if(similarityThreshold>=0 && similarityThreshold<=1)
	{
		_similarityThreshold = similarityThreshold;
	}
	else
	{
		ULOGGER_ERROR("similarityThreshold=%f", similarityThreshold);
	}
}

void Memory::setMaxStMemSize(unsigned int maxStMemSize)
{
	if(maxStMemSize == 0)
	{
		ULOGGER_ERROR("maxStMemSize=%d (must be > 0)", maxStMemSize);
	}
	else
	{
		_maxStMemSize = maxStMemSize;
		_lastBaseIds.resize(maxStMemSize, 0);
	}
}

void Memory::setRecentWmRatio(float recentWmRatio)
{
	if(recentWmRatio < 0 || recentWmRatio >= 1)
	{
		ULOGGER_ERROR("recentWmRatio=%f (must be >= 0 and < 1)", recentWmRatio);
	}
	else
	{
		_recentWmRatio = recentWmRatio;
	}
}

void Memory::setCommonSignatureUsed(bool commonSignatureUsed)
{
	_commonSignatureUsed = commonSignatureUsed;
	if(!_commonSignatureUsed)
	{
		this->moveToTrash(this->_getSignature(kIdVirtual));
	}
}

void Memory::addSignatureToStm(Signature * signature, const std::list<std::vector<float> > & actions)
{
	UTimer timer;
	// add signature on top of the short-term memory
	if(signature)
	{
		UDEBUG("adding %d with a=%d", signature->id(), (int)actions.size());
		// Update neighbors
		if(_stMem.size())
		{
			// In terms of sensorimotor learning...
			_signatures.at(*_stMem.rbegin())->addNeighbor(NeighborLink(signature->id(), actions, _lastBaseIds));
			// actions are not backward compatible, so set null actions
			signature->addNeighbor(NeighborLink(*_stMem.rbegin()));
		}

		_signatures.insert(_signatures.end(), std::pair<int, Signature *>(signature->id(), signature));
		_stMem.insert(_stMem.end(), signature->id());

		// udpate baseIds
		if(_lastBaseIds.size())
		{
			// shift base ids (first is the last added)
			std::vector<int> tmpCpy = _lastBaseIds;
			_lastBaseIds[0] = signature->id();
			memcpy(&_lastBaseIds[1], tmpCpy.data(), (tmpCpy.size()-1) * sizeof(int));
		}
	}

	UDEBUG("time = %fs", timer.ticks());
}

void Memory::addSignatureToWm(Signature * signature)
{
	if(signature)
	{
		_workingMem.insert(signature->id());
		_signatures.insert(std::pair<int, Signature*>(signature->id(), signature));
		++_signaturesAdded;
	}
	else
	{
		UERROR("Signature is null ?!?");
	}
}

const Signature * Memory::getSignature(int id) const
{
	return _getSignature(id);
}

Signature * Memory::_getSignature(int id) const
{
	return uValue(_signatures, id, (Signature*)0);
}

Signature * Memory::getSignatureLtMem(int id)
{
	ULOGGER_DEBUG("id=%d", id);
	Signature * s = 0;
	if(id>0 && _dbDriver)
	{
		// Look up in the database
		_dbDriver->getSignature(id, &s);
		if(s)
		{
			_signatures.insert(std::pair<int, Signature *>(s->id(), s));
			_workingMem.insert(s->id());
			++_signaturesAdded;
		}
		else
		{
			UFATAL("");
		}
	}
	return s;
}

std::list<NeighborLink> Memory::getNeighborLinks(int signatureId, bool ignoreNeighborByLoopClosure, bool lookInDatabase) const
{
	std::list<NeighborLink> links;
	Signature * sTop = uValue(_signatures, signatureId, (Signature*)0);
	if(sTop)
	{
		std::list<Signature *> loops;
		loops.push_back(sTop);
		while(loops.size())
		{
			Signature * s = *loops.begin();
			loops.pop_front();
			if(s)
			{
				const NeighborsMultiMap & neighbors = s->getNeighbors();
				for(NeighborsMultiMap::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
				{
					links.push_back(iter->second);
				}
				if(!ignoreNeighborByLoopClosure)
				{
					const std::set<int> & loopIds = s->getLoopClosureIds();
					for(std::set<int>::const_iterator iter = loopIds.begin(); iter!=loopIds.end(); ++iter)
					{
						if(*iter > 0 && _stMem.find(*iter) == _stMem.end())
						{
							loops.push_back(uValue(_signatures, *iter, (Signature*)0));
						}
					}
				}
			}
		}

		if(!ignoreNeighborByLoopClosure)
		{
			// Check for child loop closure ids
			const std::set<int> & childTopIds = sTop->getChildLoopClosureIds();
			for(std::set<int>::const_iterator iter = childTopIds.begin(); iter!=childTopIds.end(); ++iter)
			{
				if(*iter > 0 && _stMem.find(*iter) == _stMem.end())
				{
					loops.push_back(uValue(_signatures, *iter, (Signature*)0));
				}
			}
			while(loops.size())
			{
				Signature * s = *loops.begin();
				loops.pop_front();
				if(s)
				{
					const NeighborsMultiMap & neighbors = s->getNeighbors();
					for(NeighborsMultiMap::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
					{
						links.push_back(iter->second);
					}
					const std::set<int> & childIds = s->getChildLoopClosureIds();
					for(std::set<int>::const_iterator iter = childIds.begin(); iter!=childIds.end(); ++iter)
					{
						if(*iter > 0 && _stMem.find(*iter) == _stMem.end())
						{
							loops.push_back(uValue(_signatures, *iter, (Signature*)0));
						}
					}
				}
			}
		}
	}
	else if(lookInDatabase && _dbDriver)
	{
		NeighborsMultiMap neighbors;
		_dbDriver->loadNeighbors(signatureId, neighbors);
		for(NeighborsMultiMap::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
		{
			links.push_back(iter->second);
		}
	}
	else
	{
		UDEBUG("Cannot found signature %d in memory", signatureId);
	}
	return links;
}

// return map<Id,Margin>, including signatureId
// maxCheckedInDatabase = -1 means no limit to check in database (default)
// maxCheckedInDatabase = 0 means don't check in database
std::map<int, int> Memory::getNeighborsId(double & dbAccessTime,
		int signatureId,
		unsigned int margin,
		int maxCheckedInDatabase, // default -1 (no limit)
		bool onlyWithActions, // default false
		bool incrementMarginOnLoop, // default false
		bool ignoreSTM, // default true
		bool ignoreLoopIds // default false
		) const
{
	//ULOGGER_DEBUG("signatureId=%d, neighborsMargin=%d", signatureId, margin);
	dbAccessTime = 0;
	std::map<int, int> ids;
	if(signatureId<=0)
	{
		return ids;
	}
	bool someLoadedFromDb = false;
	int nbLoadedFromDb = 0;
	std::list<int> currentMargin;
	std::list<int> nextMargin;
	nextMargin.push_back(signatureId);
	unsigned int m = 0;
	while(m < margin && nextMargin.size())
	{
		currentMargin = nextMargin;
		nextMargin.clear();
		someLoadedFromDb = false;
		for(std::list<int>::iterator jter = currentMargin.begin(); jter!=currentMargin.end(); ++jter)
		{
			if(ids.insert(std::pair<int, int>(*jter, m)).second)
			{
				// Look up in the short time memory if all ids are here, if not... load them from the database
				const Signature * s = this->getSignature(*jter);
				std::list<int> neighborIds;
				std::list<int> loopIds;
				std::list<int> childIds;
				if(s)
				{
					const NeighborsMultiMap & neighbors = s->getNeighbors();
					const std::set<int> & loopClosureIds = s->getLoopClosureIds();
					const std::set<int> & childLoopClosureIds = s->getChildLoopClosureIds();
					int lastId = -1;
					for(NeighborsMultiMap::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
					{
						if( (ignoreLoopIds || (loopClosureIds.find(iter->first) == loopClosureIds.end() && childLoopClosureIds.find(iter->first) == childLoopClosureIds.end())) &&
							(!onlyWithActions || iter->second.actions().size()) &&
							(!ignoreSTM || (!_stMem.size() || iter->first < *_stMem.begin()) ) &&
							lastId != iter->first)
						{
							neighborIds.push_back(iter->first);
						}
						lastId = iter->first; // just to ignore duplicates
					}
					if(!ignoreLoopIds)
					{
						for(std::set<int>::const_iterator iter=loopClosureIds.begin(); iter!=loopClosureIds.end(); ++iter)
						{
							if( *iter &&
							    (!ignoreSTM || (!_stMem.size() || *iter < *_stMem.begin()) ))
							{
								loopIds.push_back(*iter);
							}
						}
						for(std::set<int>::const_iterator iter=childLoopClosureIds.begin(); iter!=childLoopClosureIds.end(); ++iter)
						{
							if( *iter &&
								(!ignoreSTM || (!_stMem.size() || *iter < *_stMem.begin()) ))
							{
								childIds.push_back(*iter);
							}
						}
					}
				}
				else if(maxCheckedInDatabase == -1 || (maxCheckedInDatabase > 0 && _dbDriver && nbLoadedFromDb < maxCheckedInDatabase))
				{
					someLoadedFromDb = true;
					std::set<int> loopClosureIds;
					std::set<int> childLoopClosureIds;
					if(!ignoreLoopIds)
					{
						UTimer timer;
						_dbDriver->getLoopClosureIds(*jter, loopClosureIds, childLoopClosureIds);
						dbAccessTime += timer.getElapsedTime();
						for(std::set<int>::const_iterator iter = loopClosureIds.begin(); iter!=loopClosureIds.end(); ++iter)
						{
							if( *iter &&
							    (!ignoreSTM || (!_stMem.size() || *iter < *_stMem.begin()) ))
							{
								loopIds.push_back(*iter);
							}
						}
						for(std::set<int>::const_iterator iter = childLoopClosureIds.begin(); iter!=childLoopClosureIds.end(); ++iter)
						{
							if( *iter &&
							    (!ignoreSTM || (!_stMem.size() || *iter < *_stMem.begin()) ))
							{
								childIds.push_back(*iter);
							}
						}
					}
					UTimer timer;
					_dbDriver->getNeighborIds(*jter, neighborIds, onlyWithActions);
					dbAccessTime += timer.getElapsedTime();
					if(neighborIds.size() == 0)
					{
						UERROR("Signature %d doesn't have neighbor!?", *jter);
					}
					for(std::list<int>::iterator iter = neighborIds.begin(); iter!=neighborIds.end();)
					{
						if( (ignoreLoopIds || (loopClosureIds.find(*iter) == loopClosureIds.end() && childLoopClosureIds.find(*iter) == childLoopClosureIds.end())) &&
						    (!ignoreSTM || (!_stMem.size() || *iter < *_stMem.begin())))
						{
							++iter;
						}
						else
						{
							iter = neighborIds.erase(iter);
						}
					}
				}

				//Priority on neighbors
				nextMargin.insert(nextMargin.end(), neighborIds.rbegin(), neighborIds.rend());

				if(incrementMarginOnLoop)
				{
					//LoopIds
					nextMargin.insert(nextMargin.end(), loopIds.rbegin(), loopIds.rend());

					//ChildIds
					nextMargin.insert(nextMargin.end(), childIds.rbegin(), childIds.rend());
				}
				else
				{
					//LoopIds
					currentMargin.insert(currentMargin.end(), loopIds.rbegin(), loopIds.rend());

					//ChildIds
					currentMargin.insert(currentMargin.end(), childIds.rbegin(), childIds.rend());
				}
			}
		}
		if(someLoadedFromDb)
		{
			// number of margin...
			++nbLoadedFromDb;
		}
		++m;
	}
	return ids;
}

void Memory::getLoopClosureIds(int signatureId, std::set<int> & loopClosureIds, std::set<int> & childLoopClosureIds, bool lookInDatabase) const
{
	const Signature * s = this->getSignature(signatureId);
	loopClosureIds.clear();
	childLoopClosureIds.clear();
	if(s)
	{
		loopClosureIds = s->getLoopClosureIds();
		childLoopClosureIds = s->getChildLoopClosureIds();
	}
	else if(lookInDatabase && _dbDriver)
	{
		_dbDriver->getLoopClosureIds(signatureId, loopClosureIds, childLoopClosureIds);
	}
}

int Memory::getNextId()
{
	return _idCount++;
}

int Memory::getDatabaseMemoryUsed() const
{
	int memoryUsed = 0;
	if(_dbDriver)
	{
		memoryUsed = _dbDriver->getMemoryUsed()/(1024*1024); //Byte to MB
	}
	return memoryUsed;
}

double Memory::getDbSavingTime() const
{
	return _dbDriver?_dbDriver->getEmptyTrashesTime():0;
}

std::set<int> Memory::getAllSignatureIds() const
{
	std::set<int> ids;
	if(_dbDriver)
	{
		_dbDriver->getAllSignatureIds(ids);
		for(std::map<int, Signature*>::const_iterator iter = _signatures.begin(); iter!=_signatures.end(); ++iter)
		{
			ids.insert(iter->first);
		}
	}
	return ids;
}

void Memory::clear()
{
	ULOGGER_DEBUG("");

	if(_dbDriver)
	{
		_dbDriver->emptyTrashes();
		_dbDriver->join();
	}

	// Save some stats to the db, save only when the mem is not empty
	if(_dbDriver && (_stMem.size() || _workingMem.size()))
	{
		unsigned int memSize = _workingMem.size() + _stMem.size();
		if(memSize != _signatures.size())
		{
			// this is only a safe check...not supposed to occur.
			ULOGGER_ERROR("The number of signatures don't match! _workingMem=%d, _stMem=%d, _signatures=%d", _workingMem.size(), _stMem.size(), _signatures.size());
		}
		if(_workingMem.size() && *_workingMem.begin() < 0)
		{
			--memSize;
		}

		ULOGGER_DEBUG("Adding statistics after run...");
		_dbDriver->addStatisticsAfterRun(memSize, _lastSignature?_lastSignature->id():0, UProcessInfo::getMemoryUsage(), _dbDriver->getMemoryUsed());
	}
	ULOGGER_DEBUG("");

	int minId = -1;
	std::map<int, Signature*>::iterator minIter = _signatures.begin();
	while(minIter != _signatures.end())
	{
		if(minIter->first > 0)
		{
			minId = minIter->first;
			break;
		}
		++minIter;
	}

	ULOGGER_DEBUG("");

	//Get the tree root (parents)
	std::map<int, Signature*> mem = _signatures;
	for(std::map<int, Signature *>::iterator i=mem.begin(); i!=mem.end(); ++i)
	{
		if(i->second)
		{
			ULOGGER_DEBUG("deleting from the working and the short-time memory: %d", i->first);
			this->moveToTrash(i->second);
		}
	}

	if(_workingMem.size() != 0)
	{
		ULOGGER_ERROR("_workingMem must be empty here, size=%d", _workingMem.size());
	}
	_workingMem.clear();
	if(_stMem.size() != 0)
	{
		ULOGGER_ERROR("_stMem must be empty here, size=%d", _stMem.size());
	}
	_stMem.clear();

	ULOGGER_DEBUG("");
	// Wait until the db trash has finished cleaning the memory
	if(_dbDriver)
	{
		_dbDriver->emptyTrashes();
	}
	ULOGGER_DEBUG("");
	_lastSignature = 0;
	_lastLoopClosureId = 0;
	_idCount = kIdStart;
	_memoryChanged = false;
	_lastBaseIds = std::vector<int>(_lastBaseIds.size(), 0);
	_similaritiesMap.clear();
}

/**
 * Compute the likelihood of the signature with some others in the memory.
 * If an error occurs, the result is empty.
 */
std::map<int, float> Memory::computeLikelihood(const Signature * signature, const std::list<int> & ids, float & maximumScore)
{
	UTimer timer;
	timer.start();
	std::map<int, float> likelihood;
	maximumScore = 0;

	if(!signature)
	{
		ULOGGER_ERROR("The signature is null");
		return likelihood;
	}
	else if(ids.empty())
	{
		UWARN("ids list is empty");
		return likelihood;
	}

	float sumSimilarity = 0.0f;
	float maxSim = 0.0f;
	int maxId = 0;
	for(std::list<int>::const_iterator iter = ids.begin(); iter!=ids.end(); ++iter)
	{
		const Signature * sB = this->getSignature(*iter);
		if(!sB)
		{
			UFATAL("Signature %d not wfound in WM ?!?", *iter);
		}
		float sim = signature->compareTo(sB);
		likelihood.insert(likelihood.end(), std::pair<int, float>(*iter, sim));
		sumSimilarity += sim;
		UDEBUG("sim %d with %d = %f", signature->id(), *iter, sim);
		if(sim>maxSim)
		{
			maxSim = sim;
			maxId = *iter;
		}
	}
	ULOGGER_DEBUG("sumSimilarity=%f, maxSim=%f, id=%d)", sumSimilarity, maxSim, maxId);

	ULOGGER_DEBUG("_similaritiesMap.size() = %d", (int)_similaritiesMap.size());
	ULOGGER_DEBUG("compute likelihood... %f s", timer.ticks());
	maximumScore = 1.0f;
	return likelihood;
}

float Memory::compareOneToOne(const std::vector<int> & idsA, const std::vector<int> & idsB)
{
	/*std::string baseIdsDebug;
	for(unsigned int i=0; i<idsA.size(); ++i)
	{
		baseIdsDebug.append(uNumber2str(idsA[i]));
		if(i+1 < idsA.size())
		{
			baseIdsDebug.append(", ");
		}
	}
	UDEBUG("baseIdsA = [%s]", baseIdsDebug.c_str());
	baseIdsDebug.clear();
	for(unsigned int i=0; i<idsB.size(); ++i)
	{
		baseIdsDebug.append(uNumber2str(idsB[i]));
		if(i+1 < idsB.size())
		{
			baseIdsDebug.append(", ");
		}
	}
	UDEBUG("baseIdsB = [%s]", baseIdsDebug.c_str());
*/

	float sim = 0.0f;
	const Signature * sA;
	const Signature * sB;
	int idSmall;
	int idBig;
	bool added;
	int nbCompared = 0;
	for(unsigned int i=0; i<idsA.size() && i<idsB.size(); ++i)
	{
		if(idsA[i] < idsB[i])
		{
			idSmall = idsA[i];
			idBig = idsB[i];
		}
		else
		{
			idSmall = idsB[i];
			idBig = idsA[i];
		}
		added = false;
		std::map<int, std::map<int, float> >::iterator iterBig = _similaritiesMap.find(idBig);
		if(iterBig != _similaritiesMap.end())
		{
			std::map<int, float>::iterator iterSmall = iterBig->second.find(idSmall);
			if(iterSmall != iterBig->second.end())
			{
				//UDEBUG("(%d, %d, %f)", idBig, idSmall, iterSmall->second);
				sim += iterSmall->second;
				added = true;
			}
		}
		if(!added)
		{
			sA = getSignature(idsA[i]);
			sB = getSignature(idsB[i]);
			if(sA && sB)
			{
				float tmp = sA->compareTo(sB);
				sim += tmp;
				if(iterBig != _similaritiesMap.end())
				{
					iterBig->second.insert(std::make_pair(idSmall, tmp));
				}
				else
				{
					std::map<int, float> tmpMap;
					tmpMap.insert(std::make_pair(idSmall, tmp));
					_similaritiesMap.insert(_similaritiesMap.end(), std::make_pair(idBig, tmpMap));
				}
				//UDEBUG("Added (%d,%d,%f) to similaritiesMap", idBig, idSmall, tmp);
				++nbCompared;
			}
			else
			{
				if(sA && idsB[i])
				{
					UDEBUG("Not found B %d", idsB[i]);
				}
				if(sB && idsA[i])
				{
					UDEBUG("Not found A %d", idsA[i]);
				}
			}
		}
		else
		{
			++nbCompared;
		}
	}
	int total = (int)(idsA.size()>idsB.size()?idsA.size():idsB.size());
	if(total)
	{
		sim /= float(total);
	}
	UDEBUG("sim=%f, idsA=%d, idsB=%d, nbCompared=%d", sim, (int)idsA.size(), (int)idsB.size(), nbCompared);
	return sim;
}

// Weights of the signatures in the working memory <signature id, weight>
std::map<int, int> Memory::getWeights() const
{
	std::map<int, int> weights;
	for(std::set<int>::const_iterator iter=_workingMem.begin(); iter!=_workingMem.end(); ++iter)
	{
		const Signature * s = this->getSignature(*iter);
		if(!s)
		{
			UFATAL("Location %d must exist in memory", *iter);
		}
		weights.insert(weights.end(), std::make_pair(*iter, s->getWeight()));
	}
	return weights;
}

int Memory::getWeight(int id) const
{
	int weight = 0;
	const Signature * s = this->getSignature(id);
	if(s)
	{
		weight = s->getWeight();
	}
	else if(_dbDriver)
	{
		_dbDriver->getWeight(id, weight);
	}
	return weight;
}

int Memory::forget(const std::set<int> & ignoredIds)
{
	ULOGGER_DEBUG("");
	// Remove one more than total added during the iteration
	std::list<Signature *> signatures = getRemovableSignatures(_signaturesAdded+1, ignoredIds);
	for(std::list<Signature *>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
	{
		// When a signature is deleted, it notifies the memory
		// and it is removed from the memory list
		this->moveToTrash(*iter);
	}
	UDEBUG("signaturesRemoved=%d, _signaturesAdded=%d", (int)signatures.size(), _signaturesAdded);
	return (int)signatures.size();
}


int Memory::cleanup(const std::list<int> & ignoredIds)
{
	ULOGGER_DEBUG("");
	int signaturesRemoved = 0;

	// bad signature
	if(_lastSignature->isBadSignature())
	{
		moveToTrash(_lastSignature);
		++signaturesRemoved;
	}

	return signaturesRemoved;
}

void Memory::emptyTrash()
{
	if(_dbDriver)
	{
		_dbDriver->emptyTrashes(true);
	}
}

void Memory::joinTrashThread()
{
	if(_dbDriver)
	{
		_dbDriver->join();
	}
}

std::set<int> Memory::reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, double & timeDbAccess)
{
	// get the signatures, if not in the working memory, they will be loaded from the database
	std::set<int> loaded;
	UTimer timer;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(((maxLoaded && loaded.size() < maxLoaded) || !maxLoaded ) &&
		   _signatures.find(*i) == _signatures.end())
		{
			if(getSignatureLtMem(*i))
			{
				//When loaded from the long-term memory, the signature
				// is automatically added on top of the working memory
				loaded.insert(*i);
			}
			else
			{
				UFATAL("Cannot load signature %d from LTM", *i);
			}
		}
	}
	timeDbAccess = timer.getElapsedTime();
	return loaded;
}

class WeightIdKey
{
public:
	WeightIdKey(int w, int i) :
		weight(w),
		id(i) {}
	bool operator<(const WeightIdKey & k) const
	{
		if(weight < k.weight)
		{
			return true;
		}
		else if(weight == k.weight)
		{
			if(id < k.id)
			{
				return true;
			}
		}
		return false;
	}
	int weight, id;
};
std::list<Signature *> Memory::getRemovableSignatures(int count, const std::set<int> & ignoredIds)
{
	//ULOGGER_DEBUG("");
	std::list<Signature *> removableSignatures;
	std::map<WeightIdKey, Signature *> signatureMap;

	// Find the last index to check...
	const std::set<int> & wm = _workingMem;
	ULOGGER_DEBUG("mem.size()=%d, ignoredIds.size()=%d", wm.size(), ignoredIds.size());

	if(wm.size())
	{
		int recentWmMaxSize = _recentWmRatio * float(wm.size());
		bool recentWmImmunized = false;
		// look for the position of the lastLoopClosureId in WM
		int currentRecentWmSize = 0;
		if(_lastLoopClosureId > 0 && _stMem.find(_lastLoopClosureId) == _stMem.end())
		{
			// If set, it must be in WM
			std::set<int>::const_iterator iter = _workingMem.find(_lastLoopClosureId);
			while(iter != _workingMem.end())
			{
				++currentRecentWmSize;
				++iter;
			}
			if(currentRecentWmSize>1 && currentRecentWmSize < recentWmMaxSize)
			{
				recentWmImmunized = true;
			}
			else if(currentRecentWmSize == 0 && _workingMem.size() > 1)
			{
				UERROR("Last loop closure id not found in WM (%d)", _lastLoopClosureId);
			}
			UDEBUG("currentRecentWmSize=%d, recentWmMaxSize=%d, _recentWmRatio=%f, end recent wM = %d", currentRecentWmSize, recentWmMaxSize, _recentWmRatio, _lastLoopClosureId);
		}

		for(std::set<int>::const_iterator memIter = wm.begin(); memIter != wm.end(); ++memIter)
		{
			if( (recentWmImmunized && *memIter > _lastLoopClosureId) ||
				*memIter == _lastLoopClosureId)
			{
				// ignore recent memory
			}
			else if(*memIter > 0 && ignoredIds.find(*memIter) == ignoredIds.end())
			{
				Signature * s = this->_getSignature(*memIter);
				if(s)
				{
					// Its loop closures must not be in STM to be removable, rehearsal issue
					const std::set<int> & loopClosureIds = s->getLoopClosureIds();
					bool foundInSTM = false;
					for(std::set<int>::const_iterator iter = loopClosureIds.begin(); iter!=loopClosureIds.end(); ++iter)
					{
						if(_stMem.find(*iter) != _stMem.end())
						{
							foundInSTM = true;
							break;
						}
					}
					if(!foundInSTM)
					{
						// looped signature priority to be transferred
						signatureMap.insert(std::make_pair(WeightIdKey(s->getWeight(), s->id()), s));
					}
				}
				else
				{
					ULOGGER_ERROR("Not supposed to occur!!!");
				}
			}
			else
			{
				//ULOGGER_DEBUG("Ignoring id %d", memIter->first);
			}
		}

		int recentWmCount = 0;
		std::set<int> addedSignatures;
		// make the list of removable signatures
		// Criteria : Weight -> ID
		UDEBUG("signatureMap.size()=%d", (int)signatureMap.size());
		for(std::map<WeightIdKey, Signature*>::iterator iter=signatureMap.begin();
			iter!=signatureMap.end();
			++iter)
		{
			bool removable = true;
			const std::set<int> & childIds = iter->second->getChildLoopClosureIds();
			for(std::set<int>::const_iterator jter = childIds.begin(); jter != childIds.end(); ++jter)
			{
				// if the child is not in WM or if it is added to the removable list
				removable = _workingMem.find(*jter) == _workingMem.end() || addedSignatures.find(*jter) != addedSignatures.end();
				if(!removable)
				{
					break;
				}
			}
			if(removable)
			{
				if(!recentWmImmunized)
				{

					UDEBUG("weight=%d, id=%d, lcCount=%d, lcId=%d, childId=%d",
							iter->first.weight,
							iter->second->id(),
							int(iter->second->getLoopClosureIds().size()),
							iter->second->getLoopClosureIds().size()?*iter->second->getLoopClosureIds().rbegin():0,
							iter->second->getChildLoopClosureIds().size()?*iter->second->getChildLoopClosureIds().rbegin():0);
					removableSignatures.push_back(iter->second);
					addedSignatures.insert(iter->second->id());

					if(iter->second->id() > _lastLoopClosureId)
					{
						++recentWmCount;
						if(currentRecentWmSize - recentWmCount < recentWmMaxSize)
						{
							UDEBUG("switched recentWmImmunized");
							recentWmImmunized = true;
						}
					}
				}
				else if(iter->second->id() < _lastLoopClosureId)
				{
					UDEBUG("weight=%d, id=%d, lcCount=%d, lcId=%d, childId=%d",
							iter->first.weight,
							iter->second->id(),
							int(iter->second->getLoopClosureIds().size()),
							iter->second->getLoopClosureIds().size()?*iter->second->getLoopClosureIds().rbegin():0,
							iter->second->getChildLoopClosureIds().size()?*iter->second->getChildLoopClosureIds().rbegin():0);
					removableSignatures.push_back(iter->second);
					addedSignatures.insert(iter->second->id());
				}
				if(removableSignatures.size() >= (unsigned int)count)
				{
					break;
				}
			}
		}
	}
	else
	{
		ULOGGER_WARN("not enough signatures to get an old one...");
	}
	return removableSignatures;
}

void Memory::moveToTrash(Signature * s)
{
	UINFO("id=%d", s?s->id():0);
	if(s)
	{
		_workingMem.erase(s->id());
		_stMem.erase(s->id());
		_signatures.erase(s->id());
		_similaritiesMap.erase(s->id());

		if(_lastSignature == s)
		{
			_lastSignature = 0;
		}

		// In the case of loop closure detection (neighbor links have 1 empty action),
		// remove neighbor links with bad signatures.
		if(s->isBadSignature())
		{
			const NeighborsMultiMap & neighbors = s->getNeighbors();
			for(NeighborsMultiMap::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				Signature * n = this->_getSignature(iter->first);
				// neighbor to s
				if(n)
				{
					NeighborsMultiMap::const_iterator jter = n->getNeighbors().find(s->id());
					if(jter != n->getNeighbors().end())
					{
						n->removeNeighbor(s->id());
					}
				}
			}
		}

		if(_memoryChanged &&
			_dbDriver &&
			s->id()>0)
		{
			_dbDriver->asyncSave(s);
		}
		else
		{
			delete s;
		}
	}
}

const Signature * Memory::getLastSignature() const
{
	ULOGGER_DEBUG("");
	return _lastSignature;
}

bool Memory::addLoopClosureLink(int oldId, int newId)
{
	ULOGGER_INFO("old=%d, new=%d", oldId, newId);
	Signature * oldS = _getSignature(oldId);
	Signature * newS = _getSignature(newId);
	if(oldS && newS)
	{
		const std::set<int> & oldLoopclosureIds = oldS->getLoopClosureIds();
		if(oldLoopclosureIds.size() && oldLoopclosureIds.find(newS->id()) != oldLoopclosureIds.end())
		{
			// do nothing, already merged
			UDEBUG("already merged, old=%d, new=%d", oldId, newId);
			return true;
		}
		/*else if(oldLoopclosureIds.size())
		{
			for(std::set<int>::const_reverse_iterator iter = oldLoopclosureIds.rbegin(); iter!=oldLoopclosureIds.rend(); ++iter)
			{
				// Cannot be the first in STM because of the neighbor redirection issue.
				if(this->isInSTM(*iter) && *iter != *_stMem.begin())
				{
					// If there is already a loop closure with an location in STM, rehearse with it.
					return addLoopClosureLink(*iter, newId);
				}
			}
		}*/

		// Set loop closure link
		oldS->addLoopClosureId(newS->id());
		UDEBUG("Add loop closure link between %d and %d", oldS->id(), newS->id());

		if(_stMem.find(oldS->id()) == _stMem.end())
		{
			// During loop closure in WM
			newS->addChildLoopClosureId(oldS->id());
			_lastLoopClosureId = newS->id();
			newS->setWeight(newS->getWeight() + oldS->getWeight());
			oldS->setWeight(0);
		}
		else
		{
			// During rehearsal in STM...
			// Here we merge the new location with the old one,
			// redirecting all neighbor links to new location.
			if(_lastLoopClosureId == oldS->id())
			{
				_lastLoopClosureId = newS->id();
			}

			// update weight
			newS->setWeight(newS->getWeight() + 1 + oldS->getWeight());

			// redirect all baseIds from old id to new id
			for(std::set<int>::iterator iter=_stMem.begin(); iter!=_stMem.end(); ++iter)
			{
				Signature * s = _getSignature(*iter);
				if(s)
				{
					// don't modify old links (cameraDatabase needs this info
					// to reload properly the actions).
					if(s->id()!=oldS->id())
					{
						s->changeNeighborIds(oldS->id(), newS->id());
					}
				}
				else
				{
					UERROR("Location %d is not in RAM?!?", *iter);
				}
			}

			// redirect neighbor links
			const NeighborsMultiMap & neighbors = oldS->getNeighbors();
			bool allSameIdsAdded = false;
			for(NeighborsMultiMap::const_iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				NeighborLink link = iter->second;
				link.updateIds(oldS->id(), newS->id());
				if(link.id() == newS->id() && link.baseIds().size())
				{
					// Limit the number of self references to STM size (baseIds size)
					bool allSameIds = true;
					const std::vector<int> & ids = link.baseIds();
					for(unsigned int i=0; i<ids.size(); ++i)
					{
						if(ids[i]!=newS->id())
						{
							allSameIds = false;
							break;
						}
					}
					if(!allSameIds || (allSameIds && !allSameIdsAdded))
					{
						newS->addNeighbor(link);
						allSameIdsAdded = true;
					}
					else if (link.actions().size() && link.actions().front().size())
					{
						// Show warning when actions are used.
						UWARN("Ignored self reference link because base ids are all the same. (id=%d)", newS->id());
					}
				}
				else
				{
					newS->addNeighbor(link);
				}
			}

			// redirect child loop closure links
			const std::set<int> & childIds = oldS->getChildLoopClosureIds();
			for(std::set<int>::const_iterator iter = childIds.begin(); iter!=childIds.end(); ++iter)
			{
				if(*iter == newS->id())
				{
					UERROR("");
				}
				newS->addChildLoopClosureId(*iter);
				Signature * s = _getSignature(*iter);
				if(s)
				{
					UDEBUG("changed child Loop closure %d from %d to %d", *iter, oldS->id(), newS->id());
					s->removeLoopClosureId(oldS->id());
					s->addLoopClosureId(newS->id());
				}
				else
				{
					UERROR("A location (%d) in WM/STM cannot be transferred if its loop closure id is in STM", *iter);
				}
			}

			// udpate current baseIds
			for(unsigned int i=0; i<_lastBaseIds.size(); ++i)
			{
				if(_lastBaseIds[i] == oldS->id())
				{
					_lastBaseIds[i] = newS->id();
				}
			}

			if(_dataMergedOnRehearsal)
			{
				this->copyData(oldS, newS);
				// Set old image to new signature
				newS->setImage(oldS->getImage());
			}

			// remove old location
			moveToTrash(oldS);
		}
		return true;
	}
	else
	{
		if(!newS)
		{
			UERROR("newId=%d, oldId=%d, Signature %d not found in working/st memories", newId, oldId, newId);
		}
		if(!oldS)
		{
			UERROR("newId=%d, oldId=%d, Signature %d not found in working/st memories", newId, oldId, oldId);
		}
	}
	return false;
}

void Memory::dumpMemory(std::string directory) const
{
	ULOGGER_DEBUG("");
	this->dumpSignatures((directory + "DumpMemorySign.txt").c_str());
	this->dumpMemoryTree((directory + "DumpMemoryTree.txt").c_str());
}

void Memory::dumpMemoryTree(const char * fileNameTree) const
{
	FILE* foutTree = 0;
	#ifdef _MSC_VER
		fopen_s(&foutTree, fileNameTree, "w");
	#else
		foutTree = fopen(fileNameTree, "w");
	#endif

	if(foutTree)
	{
		fprintf(foutTree, "SignatureID Weight NbLoopClosureIds LoopClosureIds... NbChildLoopClosureIds ChildLoopClosureIds...\n");

		for(std::map<int, Signature *>::const_iterator i=_signatures.begin(); i!=_signatures.end(); ++i)
		{
			fprintf(foutTree, "%d %d", i->first, i->second->getWeight());

			const std::set<int> & loopIds = i->second->getLoopClosureIds();
			fprintf(foutTree, " %d", (int)loopIds.size());
			for(std::set<int>::const_iterator j=loopIds.begin(); j!=loopIds.end(); ++j)
			{
				fprintf(foutTree, " %d", *j);
			}

			const std::set<int> & childIds = i->second->getChildLoopClosureIds();
			fprintf(foutTree, " %d", (int)childIds.size());
			for(std::set<int>::const_iterator j=childIds.begin(); j!=childIds.end(); ++j)
			{
				fprintf(foutTree, " %d", *j);
			}

			fprintf(foutTree, "\n");
		}

		fclose(foutTree);
	}

}

void Memory::rehearsal(Signature * signature, std::map<std::string, float> & stats)
{
	UTimer timer;
	std::map<int, float> similarities;
	// Get parents to compare...
	std::set<int> mem = _stMem;
	mem.erase(signature->id());
	if(mem.size())
	{
		// A loop closure cannot happen on the last location of STM
		// (for neighbor links redirection issue).
		mem.erase(mem.begin());
	}

	if(_similarityOnlyLast && mem.size())
	{
		int last = *mem.rbegin();
		mem.clear();
		mem.insert(last);
	}

	if(mem.size() == 0)
	{
		return;
	}

	std::list<int> ids(mem.begin(), mem.end());
	float maximumScore = 1.0f;
	// similarities must be between 0 and 1
	similarities = Memory::computeLikelihood(signature, ids, maximumScore);

	UDEBUG("t=%fs", timer.ticks());
	if(similarities.size() == 0)
	{
		ULOGGER_WARN("similarities size == 0 ?");
		return;
	}
	else if(maximumScore == 0)
	{
		ULOGGER_WARN("maximumScore == 0 ?");
		return;
	}

	ULOGGER_DEBUG("Comparing with last signatures...");
	float value = 0;
	int id = 0;
	float maxValue = 0;
	int maxId = 0;
	for(std::map<int, float>::iterator iter=similarities.begin(); iter!=similarities.end(); ++iter)
	{
		value = iter->second/maximumScore; // scale to 0 and 1
		id = iter->first;
		if(value > maxValue)
		{
			maxId = id;
			maxValue = value;
		}
	}

	if(maxValue > _similarityThreshold)
	{
		if(_incrementalMemory)
		{
			this->addLoopClosureLink(maxId, signature->id());
		}
		else
		{
			Signature * s = _signatures.at(maxId);
			if(s)
			{
				signature->setWeight(signature->getWeight() + 1 + s->getWeight());
			}
			else
			{
				UFATAL("not supposed to happen");
			}
		}
		stats.insert(std::pair<std::string, float>(std::string("Memory/Rehearsal Closure/"), 1.0f));
	}
	else
	{
		stats.insert(std::pair<std::string, float>(std::string("Memory/Rehearsal Closure/"), 0.0f));
	}

	stats.insert(std::pair<std::string, float>(std::string("Memory/Rehearsal Max Id/"), maxId));
	stats.insert(std::pair<std::string, float>(std::string("Memory/Rehearsal Max Value/"), maxValue));

	UDEBUG("maxId=%d, maxSim=%f, t=%fs", maxId, maxValue, timer.ticks());
}

// The data returned must be released
IplImage * Memory::getImage(int id) const
{
	IplImage * img = 0;
	const Signature * s = this->getSignature(id);
	if(s && s->getImage())
	{
		img = cvCloneImage(s->getImage());
	}
	else if(_dbDriver)
	{
		_dbDriver->getImage(id, &img);
	}
	return img;
}

void Memory::generateGraph(const std::string & fileName, std::set<int> ids)
{
	if(!_dbDriver)
	{
		UERROR("A database must must loaded first...");
		return;
	}

	if(!fileName.empty())
	{
		FILE* fout = 0;
		#ifdef _MSC_VER
			fopen_s(&fout, fileName.c_str(), "w");
		#else
			fout = fopen(fileName.c_str(), "w");
		#endif

		 if (!fout)
		 {
			 UERROR("Cannot open file %s!", fileName.c_str());
			 return;
		 }

		 if(ids.size() == 0)
		 {
			 _dbDriver->getAllSignatureIds(ids);
			 for(std::map<int, Signature*>::iterator iter=_signatures.begin(); iter!=_signatures.end(); ++iter)
			 {
				 ids.insert(iter->first);
			 }
		 }

		 const char * colorG = "green";
		 const char * colorA = "red";
		 const char * colorB = "skyBlue";
		 UINFO("Generating map with %d locations", ids.size());
		 fprintf(fout, "digraph G {\n");
		 std::set<std::pair<int, int> > linksAdded;
		 for(std::set<int>::iterator i=ids.begin(); i!=ids.end(); ++i)
		 {
			 if(_signatures.find(*i) == _signatures.end())
			 {
				 int id = *i;
				 NeighborsMultiMap neighbors;
				 _dbDriver->loadNeighbors(id, neighbors);
				 int weight = 0;
				 _dbDriver->getWeight(id, weight);
				 for(NeighborsMultiMap::iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
				 {
					 if(_signatures.find(iter->first) == _signatures.end())
					 {
						 int weightNeighbor = 0;
						 _dbDriver->getWeight(iter->first, weightNeighbor);
						 linksAdded.insert(std::pair<int, int>(iter->first, id));
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"%d\", fontcolor=%s, fontsize=8];\n",
								 id,
								 weight,
								 iter->first,
								 weightNeighbor,
								 (int)iter->second.actions().size(),
								 iter->second.actions().size()>0?colorA:colorB);
					 }
				 }
				 std::set<int> loopIds;
				 std::set<int> childIds;
				 _dbDriver->getLoopClosureIds(id, loopIds, childIds);
				 for(std::set<int>::iterator iter = loopIds.begin(); iter!=loopIds.end(); ++iter)
				 {
					 int weightNeighbor = 0;
					 if(_signatures.find(*iter) == _signatures.end())
					 {
						 _dbDriver->getWeight(*iter, weightNeighbor);
					 }
					 else
					 {
						 weightNeighbor = _signatures.find(*iter)->second->getWeight();
					 }
					 linksAdded.insert(std::pair<int, int>(*iter, id));
					 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"#\", fontcolor=%s, fontsize=8];\n",
							 id,
							 weight,
							 *iter,
							 weightNeighbor,
							 colorG);
				 }
			 }
		 }
		 for(std::map<int, Signature*>::iterator i=_signatures.begin(); i!=_signatures.end(); ++i)
		 {
			 if(ids.find(i->first) != ids.end())
			 {
				 int id = i->second->id();
				 const NeighborsMultiMap & neighbors = i->second->getNeighbors();
				 int weight = i->second->getWeight();
				 for(NeighborsMultiMap::const_iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
				 {
					 //if(linksAdded.find(std::pair<int, int>(id, iter->first)) == linksAdded.end() &&
					//	linksAdded.find(std::pair<int, int>(iter->first, id)) == linksAdded.end())
					 {
						 int weightNeighbor = 0;
						 const Signature * s = this->getSignature(iter->first);
						 if(s)
						 {
							 weightNeighbor = s->getWeight();
						 }
						 else
						 {
							 _dbDriver->getWeight(iter->first, weightNeighbor);
						 }
						 linksAdded.insert(std::pair<int, int>(iter->first, id));
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"%d\", fontcolor=%s, fontsize=8];\n",
								 id,
								 weight,
								 iter->first,
								 weightNeighbor,
								 (int)iter->second.actions().size(),
								 iter->second.actions().size()>0?colorA:colorB);
					 }
				 }
				 const std::set<int> & loopIds = i->second->getLoopClosureIds();
				 for(std::set<int>::const_iterator iter = loopIds.begin(); iter!=loopIds.end(); ++iter)
				 {
					 int weightNeighbor = 0;
					 if(_signatures.find(*iter) == _signatures.end())
					 {
						 _dbDriver->getWeight(*iter, weightNeighbor);
					 }
					 else
					 {
						 weightNeighbor = _signatures.find(*iter)->second->getWeight();
					 }
					 linksAdded.insert(std::pair<int, int>(*iter, id));
					 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"%d\", fontcolor=%s, fontsize=8];\n",
							 id,
							 weight,
							 *iter,
							 weightNeighbor,
							 (int)0,
							 colorG);
				 }
			 }
		 }
		 fprintf(fout, "}\n");
		 fclose(fout);
	}
}

void Memory::cleanLocalGraph(int id, unsigned int margin)
{
	if(margin >= _stMem.size())
	{
		UERROR("The margin (%d) must be smaller than the ST size (%d)", margin, _stMem.size());
		return;
	}

	UTimer timer;
	UDEBUG("Cleaning local graph for location %d", id);
	Node root(id);
	this->createGraph(&root, margin);
	this->cleanGraph(&root);

	UDEBUG("time=%fs", timer.ticks());
}

void Memory::cleanLTM(int maxDepth)
{
	UDEBUG("");
	if(!_dbDriver || !_workingMem.size())
	{
		return;
	}

	UTimer timer;
	int wmSize = _signatures.size();
	UDEBUG("Getting %d highest weighted signatures...", wmSize);

	UDEBUG("time clear=%fs", timer.ticks());

	std::multimap<int, int> highestWeightedSignatures;
	//Add active signatures...
	for(std::map<int, Signature*>::iterator iter = _signatures.begin(); iter!=_signatures.end(); ++iter)
	{
		Signature * s = iter->second;
		highestWeightedSignatures.insert(std::pair<int, int>(s->getWeight(), s->id()));
	}
	//Look in the database
	UDEBUG("highestWeightedSignatures.size()=%d", highestWeightedSignatures.size());
	_dbDriver->getHighestWeightedSignatures(wmSize, highestWeightedSignatures);
	UDEBUG("highestWeightedSignatures.size()=%d", highestWeightedSignatures.size());

	std::set<int> weightedSignatures;
	for(std::map<int, int>::iterator iter=highestWeightedSignatures.begin(); iter!=highestWeightedSignatures.end(); ++iter)
	{
		weightedSignatures.insert(iter->second);
		UDEBUG("High signature: %d, w=%d", iter->second, iter->first);
	}

	int i=0;
	for(std::map<int, int>::iterator iter=highestWeightedSignatures.begin(); iter!=highestWeightedSignatures.end(); ++iter)
	{
		Node root(iter->second);
		UDEBUG("Creating graph around %d (w=%d) [%d/%d]", iter->second, iter->first, i++, highestWeightedSignatures.size());
		this->createGraph(&root, maxDepth, weightedSignatures);
		this->cleanGraph(&root);
	}

	UDEBUG("time cleaning LTM=%fs", timer.ticks());
}

// FIXME may not work anymore
void Memory::cleanGraph(const Node * root)
{
	if(!root)
	{
		UERROR("root is null");
		return;
	}
	UDEBUG("Cleaning graph around %d", root->id());

	UTimer timer;
	std::list<std::list<int> > paths;
	root->expand(paths);

	int i=0;
	/*for(std::list<std::list<int> >::iterator iter=paths.begin(); iter!=paths.end();++iter)
	{
		std::stringstream str;
		std::list<int> & path = *iter;
		for(std::list<int>::iterator jter = path.begin(); jter!=path.end(); ++jter)
		{
			str << *jter << " ";
		}
		UDEBUG("Paths[%d], %s", i++, str.str().c_str());
	}*/

	std::set<int> centerNodes;
	for(std::list<std::list<int> >::iterator iter=paths.begin(); iter!=paths.end(); ++iter)
	{
		std::list<int> & path = *iter;
		if(path.size() > 2)
		{
			for(std::list<int>::iterator jter = ++path.begin(); jter!=--path.end(); ++jter)
			{
				centerNodes.insert(*jter);
			}
		}
	}

	std::set<int> terminalNodes;
	for(std::list<std::list<int> >::iterator iter=paths.begin(); iter!=paths.end(); ++iter)
	{
		std::list<int> & path = *iter;
		if(centerNodes.find(path.back()) == centerNodes.end())
		{
			terminalNodes.insert(path.back());
			//UDEBUG("terminal=%d", path.back());
		}
	}

	std::set<int> toRemove;
	for(std::list<std::list<int> >::iterator iter=paths.begin(); iter!=paths.end();)
	{
		std::list<int> & path = *iter;
		if(centerNodes.find(iter->back()) != centerNodes.end())
		{
			if(path.size() > 2)
			{
				for(std::list<int>::iterator jter = ++path.begin(); jter!=--path.end(); ++jter)
				{
					toRemove.insert(*jter);
				}
			}
			iter = paths.erase(iter);
		}
		else
		{
			++iter;
		}
	}

	i=0;
	/*for(std::list<std::list<int> >::iterator iter=paths.begin(); iter!=paths.end();++iter)
	{
		std::stringstream str;
		std::list<int> & path = *iter;
		for(std::list<int>::iterator jter = path.begin(); jter!=path.end(); ++jter)
		{
			str << *jter << " ";
		}
		UDEBUG("Paths[%d], %s", i++, str.str().c_str());
	}*/

	std::set<int> immunized;
	while(paths.size())
	{
		bool allRemovable = false;
		int terminal = paths.begin()->back();
		//UDEBUG("terminal=%d", terminal);
		std::multimap<int, std::list<int> > sortedPaths;
		for(std::list<std::list<int> >::iterator iter=paths.begin(); iter!=paths.end();)
		{
			std::list<int> & path = *iter;
			if(path.back() == terminal)
			{
				if(terminalNodes.find(terminal) == terminalNodes.end())
				{
					allRemovable = true;
					if(immunized.find(terminal) == immunized.end())
					{
						toRemove.insert(terminal);
					}
				}
				int pathWeight = 0;
				if(path.size() > 2)
				{
					for(std::list<int>::iterator jter = ++path.begin(); jter!=--path.end(); ++jter)
					{
						if(immunized.find(*jter) == immunized.end())
						{
							toRemove.insert(*jter);
						}
						int w = this->getWeight(*jter);
						//UDEBUG("id(%d) w=%d", *jter, w);
						pathWeight += w;
					}
				}
				sortedPaths.insert(std::pair<int, std::list<int> >(pathWeight, *iter));
				iter = paths.erase(iter);
			}
			else
			{
				++iter;
			}
		}
		if(sortedPaths.size() && !allRemovable)
		{
			std::list<int> & path = (--sortedPaths.end())->second;
			if(sortedPaths.count((--sortedPaths.end())->first) > 1)
			{
				int highestWeight = (--sortedPaths.end())->first;
				std::map<int, std::list<int> >::iterator iter = --sortedPaths.end();
				--iter;
				while(iter->first == highestWeight)
				{
					if(iter->second.size() < path.size())
					{
						path = iter->second;
					}
					if(iter == sortedPaths.begin())
					{
						break;
					}
					--iter;
				}
			}
			for(std::list<int>::iterator jter = path.begin(); jter!=path.end(); ++jter)
			{
				toRemove.erase(*jter);
				immunized.insert(*jter);
				//UDEBUG("Immunized %d", *jter);
			}
		}
	}

	//for(std::set<int>::iterator iter=toRemove.begin(); iter!=toRemove.end(); ++iter)
	//{
	//	UDEBUG("toRemove=%d",*iter);
	//}

	std::string strToRemove;
	for(std::set<int>::iterator iter=toRemove.begin(); iter!=toRemove.end();++iter)
	{
		Signature * s = this->_getSignature(*iter);
		if(!s || (s && !s->getLoopClosureIds().size() && _stMem.find(s->id()) == _stMem.end()))
		{
			_memoryChanged = true;
			if(strToRemove.size())
			{
				strToRemove.append(" OR ");
			}
			strToRemove.append("id=");
			strToRemove.append(uNumber2Str(*iter));

			if(s)
			{
				this->moveToTrash(s);
			}
			else if(_dbDriver)
			{
				// remove reference from active neighbors
				std::list<int> neighbors;
				_dbDriver->getNeighborIds(*iter, neighbors);
				for(std::list<int>::iterator jter=neighbors.begin(); jter!=neighbors.end(); ++jter)
				{
					s = this->_getSignature(*jter);
					if(s)
					{
						//s->removeNeighbor(*iter); // FIXME
					}
				}
			}

		}
	}

	UDEBUG("Removing %s", strToRemove.c_str());
	if(_dbDriver && !strToRemove.empty())
	{
		_dbDriver->executeNoResult(std::string("UPDATE Signature SET loopClosureId=-1 WHERE ") + strToRemove);
	}

	UDEBUG("time=%fs", timer.ticks());
}

//recursive
void Memory::createGraph(Node * parent, unsigned int maxDepth, const std::set<int> & endIds)
{
	if(maxDepth == 0 || !parent)
	{
		return;
	}
	double dbAccessTime = 0.0;
	std::map<int, int> neighbors = this->getNeighborsId(dbAccessTime, parent->id(), 1, -1, false, false, false);
	for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
	{
		if(!parent->isAncestor(iter->first))
		{
			Node * n = new Node(iter->first, parent);
			if(endIds.find(iter->first) == endIds.end())
			{
				this->createGraph(n, maxDepth-1, endIds);
			}
		}
	}
}

} // namespace rtabmap
