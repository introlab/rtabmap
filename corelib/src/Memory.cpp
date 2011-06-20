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
#include "Signature.h"
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
	_idCount(kIdStart),
	_lastSignature(0),
	_lastLoopClosureId(0),
	_incrementalMemory(Parameters::defaultMemIncrementalMemory()),
	_maxStMemSize(Parameters::defaultMemMaxStMemSize()),
	_commonSignatureUsed(Parameters::defaultMemCommonSignatureUsed()),
	_databaseCleaned(Parameters::defaultMemDatabaseCleaned()),
	_delayRequired(Parameters::defaultMemDelayRequired()),
	_recentWmRatio(Parameters::defaultMemRecentWmRatio()),
	_memoryChanged(false)
{
	this->parseParameters(parameters);
}

bool Memory::init(const std::string & dbDriverName, const std::string & dbUrl, bool dbOverwritten, const ParametersMap & parameters)
{
	ULOGGER_DEBUG("");

	this->parseParameters(parameters);

	if(!_memoryChanged || dbOverwritten)
	{
		if(_dbDriver)
		{
			UEventsManager::post(new RtabmapEventInit("Closing database connection..."));
			_dbDriver->closeConnection();
			delete _dbDriver;
			_dbDriver = 0;
			UEventsManager::post(new RtabmapEventInit("Closing database connection, done!"));
		}

		UEventsManager::post(new RtabmapEventInit("Clearing memory..."));
		this->clear();
		UEventsManager::post(new RtabmapEventInit("Clearing memory, done!"));
	}
	else
	{
		UEventsManager::post(new RtabmapEventInit("Clearing memory..."));
		this->clear();
		UEventsManager::post(new RtabmapEventInit("Clearing memory, done!"));

		if(_dbDriver)
		{
			UEventsManager::post(new RtabmapEventInit("Closing database connection..."));
			_dbDriver->closeConnection();
			delete _dbDriver;
			_dbDriver = 0;
			UEventsManager::post(new RtabmapEventInit("Closing database connection, done!"));
		}
	}


	// Synchronize with the database
	_dbDriver = DBDriverFactory::createDBDriver(dbDriverName, parameters); // inMemory

	bool success = false;
	if(_dbDriver)
	{
		if(dbOverwritten)
		{
			UEventsManager::post(new RtabmapEventInit(std::string("Deleting database ") + dbUrl + "..."));
			UFile::erase(dbUrl);
			UEventsManager::post(new RtabmapEventInit(std::string("Deleting database ") + dbUrl + ", done!"));
		}
		UEventsManager::post(new RtabmapEventInit(std::string("Connecting to database ") + dbUrl + "..."));
		if(_dbDriver->openConnection(dbUrl))
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
					_workingMem.insert(std::pair<int, int>((*iter)->id(), 0.0));
				}
			}
			UEventsManager::post(new RtabmapEventInit(std::string("Loading last signatures, done! (") + uNumber2str(int(_workingMem.size() + _stMem.size())) + " loaded)"));

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
	if((iter=parameters.find(Parameters::kMemDatabaseCleaned())) != parameters.end())
	{
		_databaseCleaned = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kMemMaxStMemSize())) != parameters.end())
	{
		this->setMaxStMemSize(std::atoi((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kMemCommonSignatureUsed())) != parameters.end())
	{
		this->setCommonSignatureUsed(uStr2Bool((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kMemDelayRequired())) != parameters.end())
	{
		this->setDelayRequired(std::atoi((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kMemRecentWmRatio())) != parameters.end())
	{
		this->setRecentWmRatio(std::atof((*iter).second.c_str()));
	}
	if(_dbDriver)
	{
		_dbDriver->parseParameters(parameters);
	}
}

void Memory::preUpdate()
{
	_signaturesAdded = 0;

	for(std::map<int,int>::iterator iter=_workingMem.begin(); iter!=_workingMem.end(); ++iter)
	{
		iter->second += 1;
	}
}

bool Memory::update(const SMState * smState, std::list<std::pair<std::string, float> > & stats)
{
	ULOGGER_DEBUG("");
	UTimer timer;
	UTimer totalTimer;
	timer.start();

	//============================================================
	// Pre update...
	//============================================================
	ULOGGER_DEBUG("pre-updating...");
	this->preUpdate();
	stats.push_back(std::pair<std::string, float>(std::string("TimingMem/Pre-update/ms"), timer.ticks()*1000));
	ULOGGER_DEBUG("time preUpdate=%f ms", stats.back().second);

	//============================================================
	// Create a signature with the image received.
	//============================================================
	Signature * signature = this->createSignature(this->getNextId(), smState, this->isRawDataKept());
	if (signature == 0)
	{
		return false;
	}

	//============================================================
	// Post update... does nothing if not overridden
	// Must be before addSignature(), this will detect bad signatures
	//============================================================
	//this->postUpdate();
	//stats.insert(std::pair<std::string, float>(std::string("TimingMem/Post-update/ms"), timer.ticks()*1000));
	//ULOGGER_DEBUG("time postUpdate=%f ms", stats.at(std::string("TimingMem/Post-update/ms")));


	// It will be added to the short-time memory, no need to delete it...
	this->addSignatureToStm(signature, smState->getActuators());
	_lastSignature = signature;

	if(_lastLoopClosureId == 0)
	{
		// If not set use the new one added
		_lastLoopClosureId = signature->id();
	}

	stats.push_back(std::pair<std::string, float>(std::string("TimingMem/Signature creation/ms"), timer.ticks()*1000));
	ULOGGER_DEBUG("time creating signature=%f ms", stats.back().second);

	//============================================================
	// Comparison step...
	// Compare with the X last signatures. If different, add this
	// signature like a parent to the memory tree, otherwise add
	// it as a child to the similar signature.
	//============================================================
	int maxId = 0;
	if(_similarityThreshold<1.0f)
	{
		float maxValue=0;
		UTimer t;
		maxId = rehearsal(signature, _similarityOnlyLast, maxValue);
		UDEBUG("t=%fs", t.ticks());
		stats.push_back(std::pair<std::string, float>(std::string("Memory/Rehearsal Max Id/"), maxId));
		stats.push_back(std::pair<std::string, float>(std::string("Memory/Rehearsal Value/"), maxValue));
		if(maxId > 0 && maxValue >= _similarityThreshold)
		{
			if(_incrementalMemory)
			{
				// maxId -> signature->id() + merged
				ULOGGER_DEBUG("Linking old signature %d to id=%d", maxId, signature->id());
				this->addLoopClosureLink(maxId, signature->id(), true);
			}
			else
			{
				const Signature * s = this->getSignature(maxId);
				if(s)
				{
					signature->setWeight(signature->getWeight() + 1 + s->getWeight());
				}
				else
				{
					signature->setWeight(signature->getWeight() + 1);
				}
			}
		}
		UDEBUG("t=%fs", t.ticks());
	}
	stats.push_back(std::pair<std::string, float>(std::string("TimingMem/Rehearsal/ms"), timer.ticks()*1000));
	ULOGGER_DEBUG("time rehearsal=%f ms", stats.back().second);

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
			_workingMem.insert(std::pair<int, int>(s->id(), 0));
		}
	}
	else
	{
		// remove virtual signature
		Signature * s = _getSignature(kIdVirtual);
		if(s)
		{
			s = this->createSignature(kIdVirtual, 0); // Create a virtual place
			this->moveToTrash(s);
		}
	}

	if(!_memoryChanged)
	{
		_memoryChanged = true;
	}

	UDEBUG("totalTimer = %fs", totalTimer.ticks());

	stats.push_back(std::pair<std::string, float>(std::string("Memory/Last loop closure/"), _lastLoopClosureId));

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
	}
}

void Memory::setDelayRequired(int delayRequired)
{
	if(delayRequired < 0)
	{
		ULOGGER_ERROR("delayRequired=%d (must be >= 0)", delayRequired);
	}
	else
	{
		_delayRequired = delayRequired;
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
	// add signature on top of the short-time memory
	if(signature && !signature->isBadSignature())
	{
		UDEBUG("adding %d with a=%d", signature->id(), (int)actions.size());
		// Update neighbors
		if(_stMem.size())
		{
			_signatures.at(*_stMem.rbegin())->addNeighbor(signature->id(), actions);
			// actions are not backward compatible, so set null actions
			signature->addNeighbor(*_stMem.rbegin(), std::list<std::vector<float> >());
		}

		_signatures.insert(_signatures.end(), std::pair<int, Signature *>(signature->id(), signature));
		_stMem.insert(_stMem.end(), signature->id());
		++_signaturesAdded;
	}

	// Transfer the oldest signature of the short-time memory to the working memory
	while(_stMem.size() > _maxStMemSize)
	{
		// (_delayRequired) Can be forgotten as it enters into WM
		_workingMem.insert(_workingMem.end(), std::pair<int, int>(*_stMem.begin(), _delayRequired));
		_stMem.erase(*_stMem.begin());
	}
	UDEBUG("time = %fs", timer.ticks());
}

void Memory::addSignatureToWm(Signature * signature)
{
	if(signature)
	{
		_workingMem.insert(std::pair<int, int>(signature->id(), 0));
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
	//ULOGGER_WARN("get signature %d", id);
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
			_workingMem.insert(std::pair<int, int>(s->id(), 0));
		}
	}
	return s;
}

/*int Memory::getId(const Signature * signature) const
{
	return _workingMem.key(signature, ID_INVALID);
}*/

std::list<int> Memory::getChildrenIds(int signatureId) const
{
	std::list<int> ids;
	if(_dbDriver)
	{
		_dbDriver->getChildrenIds(signatureId, ids);
	}
	return ids;
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
		_dbDriver->kill();
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
		if(_workingMem.size() && _workingMem.begin()->first < 0)
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
		ULOGGER_DEBUG("");
		//By default unreferenced all signatures with a loop closure link
		if(_databaseCleaned)
		{
			_dbDriver->executeNoResult(std::string("DELETE FROM Signature WHERE loopClosureId!=0;"));
			_dbDriver->executeNoResult(std::string("DELETE FROM Neighbor WHERE NOT EXISTS (SELECT * FROM Signature WHERE Signature.id = Neighbor.sid);"));
			_dbDriver->executeNoResult(std::string("DELETE FROM Neighbor WHERE NOT EXISTS (SELECT * FROM Signature WHERE Signature.id = Neighbor.nid);"));
			_dbDriver->executeNoResult(std::string("DELETE FROM Map_SS_VW WHERE NOT EXISTS (SELECT * FROM Signature WHERE Signature.id = signatureId);"));
		}

		ULOGGER_DEBUG("");
	}
	ULOGGER_DEBUG("");
	_lastSignature = 0;
	_lastLoopClosureId = 0;
	_idCount = kIdStart;
	_memoryChanged = false;
}

/**
 * Compute the likelihood of the signature with some others in the memory. If
 * the signature ids list is empty, the likelihood will be calculated
 * for all signatures in the working time memory.
 * If an error occurs, the result is empty.
 */
std::map<int, float> Memory::computeLikelihood(const Signature * signature, const std::set<int> & signatureIds) const
{
	UTimer timer;
	timer.start();
	std::map<int, float> likelihood;

	if(!signature)
	{
		ULOGGER_ERROR("The signature is null");
		return likelihood;
	}

	if(signatureIds.size() == 0)
	{
		const std::map<int, int> & wm = this->getWorkingMem();
		float sumSimilarity = 0.0f;
		int nonNulls = 0;
		int nulls = 0;
		float maxSim = 0.0f;
		std::map<int, int>::const_iterator iter = wm.begin();
		for(; iter!=wm.end(); ++iter)
		{
			float sim = signature->compareTo(this->getSignature(iter->first));
			likelihood.insert(likelihood.end(), std::pair<int, float>(iter->first, sim));
			sumSimilarity += sim;
			UDEBUG("sim %d with %d = %f", signature->id(), iter->first, sim);
			if(sim>maxSim)
			{
				maxSim = sim;
			}
			if(sim)
			{
				++nonNulls;
			}
			else
			{
				++nulls;
			}
		}
		ULOGGER_DEBUG("sumSimilarity=%f, maxSim=%f, nonNulls=%d, nulls=%d)", sumSimilarity, maxSim, nonNulls, nulls);
	}
	else
	{
		for(std::set<int>::const_iterator i=signatureIds.begin(); i != signatureIds.end(); ++i)
		{
			likelihood.insert(likelihood.end(), std::pair<int, float>(*i, signature->compareTo(this->getSignature(*i))));
		}
	}

	ULOGGER_DEBUG("compute likelihood... %f s", timer.ticks());
	return likelihood;
}

// Weights of the signatures in the working memory <signature id, weight>
std::map<int, int> Memory::getWeights() const
{
	std::map<int, int> weights;
	for(std::map<int, int>::const_iterator i=_workingMem.begin(); i!=_workingMem.end(); ++i)
	{
		weights.insert(weights.end(), std::pair<int, int>(i->first, uValue(_signatures, i->first, (Signature*)0)->getWeight()));
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

int Memory::forget(const std::list<int> & ignoredIds)
{
	ULOGGER_DEBUG("");
	int signaturesRemoved = 0;
	Signature * s = 0;
	int i=0;
	// Remove one more than total added during the iteration
	while(i++<_signaturesAdded+1 && (s = getRemovableSignature(ignoredIds)))
	{
		++signaturesRemoved;
		// When a signature is deleted, it notifies the memory
		// and it is removed from the memory list
		this->moveToTrash(s);
	}
	return signaturesRemoved;
}


int Memory::cleanup(const std::list<int> & ignoredIds)
{
	ULOGGER_DEBUG("");

	//Cleanup looped signatures
	int signaturesRemoved = 0;
	Signature * s = 0;
	while((s = getRemovableSignature(ignoredIds, true)))
	{
		this->moveToTrash(s);
		++signaturesRemoved;
	}


	// Cleanup of the STM
	// We get a copy because moveTotrash() remove the signature from the _stMem
	std::set<int> mem = _stMem;
	for(std::set<int>::iterator i=mem.begin(); i!=mem.end(); ++i )
	{
		Signature * s = this->_getSignature(*i);
		if(!s)
		{
			ULOGGER_ERROR("Signature not found in STM or WM?!?");
		}
		else
		{
			if((*i > 0 && s->getLoopClosureId() != 0) ||
			   (!_incrementalMemory && _lastSignature && _lastSignature->id() == *i) ||
			   (*i > 0 && s->isBadSignature()))
			{
				this->moveToTrash(s);
				++signaturesRemoved;
			}
		}
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

int Memory::reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, unsigned int maxTouched)
{
	// get the signatures, if not in the working memory, they will be loaded from the database
	int count = 0;
	std::map<int, int>::iterator wmIter;
	unsigned int touched = 0;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(((maxLoaded && (unsigned int)count < maxLoaded) || !maxLoaded ) &&
		   _signatures.find(*i) == _signatures.end())
		{
			if(getSignatureLtMem(*i))
			{
				//When loaded from the long-term memory, the signature
				// is automatically added on top of the working memory
				++count;
			}
		}
		else if((wmIter = _workingMem.find(*i)) != _workingMem.end() && touched < maxTouched)
		{
			wmIter->second = 0;
		}
		++touched;
	}
	return count;
}

Signature * Memory::getRemovableSignature(const std::list<int> & ignoredIds, bool onlyLoopedSignatures)
{
	//ULOGGER_DEBUG("");
	Signature * removableSignature = 0;

	// Find the last index to check...
	const std::map<int, int> & wm = _workingMem;
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
			std::map<int, int>::const_iterator iter = _workingMem.find(_lastLoopClosureId);
			while(iter != _workingMem.end())
			{
				++currentRecentWmSize;
				++iter;
			}
			if(currentRecentWmSize && currentRecentWmSize < recentWmMaxSize)
			{
				recentWmImmunized = true;
			}
			else if(currentRecentWmSize == 0 && _workingMem.size() > 1)
			{
				UERROR("Last loop closure id not found in WM (%d)", _lastLoopClosureId);
			}
			UDEBUG("currentRecentWmSize=%d, recentWmMaxSize=%d, _recentWmRatio=%f, end recent wM = %d", currentRecentWmSize, recentWmMaxSize, _recentWmRatio, _lastLoopClosureId);
		}

		int timestampThr = _delayRequired; // iterations
		UDEBUG("_delayRequired = %d", _delayRequired);
		for(std::map<int, int>::const_iterator memIter = wm.begin(); memIter != wm.end(); ++memIter)
		{
			if(recentWmImmunized && memIter->first > _lastLoopClosureId)
			{
				ULOGGER_DEBUG("Reached end of recent working memory (%d)", memIter->first);
				break;
			}
			else if(memIter->first > 0 &&
			   std::find(ignoredIds.begin(), ignoredIds.end(), memIter->first) == ignoredIds.end() &&
			  memIter->first != _lastLoopClosureId)
			{
				//Not in ignored ids and not the lastLoopClosureId
				Signature * s = uValue(_signatures, memIter->first, (Signature*)0);
				if(s)
				{
					//ULOGGER_DEBUG("testing : id=%d, weight=%d, lcId=%d", s->id(), s->getWeight(), s->getLoopClosureId());
					if(onlyLoopedSignatures)
					{
						if(s->getLoopClosureId() != 0)
						{
							removableSignature = s;
							break;
						}
					}
					else if((!removableSignature) ||
					   (removableSignature->getLoopClosureId() == 0 && s->getWeight() < removableSignature->getWeight()) ||
					   (removableSignature->getLoopClosureId() == 0 && s->getLoopClosureId() != 0))
					{
						if(memIter->second < timestampThr)
						{
							ULOGGER_DEBUG("Ignoring %d because it is too recent : (%d < %d iterations)", memIter->first, memIter->second, timestampThr);
						}
						else
						{
							if(removableSignature)
							{
								ULOGGER_DEBUG("Old removable: id=%d, weight=%d, lcId=%d", removableSignature->id(), removableSignature->getWeight(), removableSignature->getLoopClosureId());
							}
							ULOGGER_DEBUG("New removable: id=%d, weight=%d, lcId=%d, age=%d", s->id(),  s->getWeight(), s->getLoopClosureId(), memIter->second);
							removableSignature = s;
							if(removableSignature->getLoopClosureId() != 0)
							{
								// stop now if a signature with a loop closure link is found
								break;
							}
						}
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
	}
	else
	{
		ULOGGER_WARN("not enough signatures to get an old one...");
	}
	ULOGGER_DEBUG("Removable signature = %d", removableSignature?removableSignature->id():0);
	return removableSignature;
}

void Memory::moveToTrash(Signature * s)
{
	ULOGGER_DEBUG("id=%d", s?s->id():0);
	if(s)
	{
		_workingMem.erase(s->id());
		_stMem.erase(s->id());
		_signatures.erase(s->id());

		if(s->getLoopClosureId() != 0)
		{
			//remove it from referenced neighbors
			const std::set<int> & neighbors = uKeysSet(s->getNeighbors());
			for(std::set<int>::const_iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				Signature * ns = this->_getSignature(*iter);
				if(ns)
				{
					ns->removeNeighbor(s->id());
				}
				else if(_dbDriver)
				{
					UDEBUG("A neighbor is not found (%d)", *iter);
					_dbDriver->removeNeighbor(*iter, s->id());
				}
				s->removeNeighbor(*iter);
			}

			if(s->getWeight() && s->getLoopClosureId() > 0)
			{
				// redirect loop closure id to children (to fix an error
				// in addLoopClosureLink where oldId doesn't exist anymore)
				// TODO is there a better way to handle that ? Like a direct bi-directional
				// loop closure link instead of iterating over all signatures?
				for(std::map<int, Signature *>::reverse_iterator iter=_signatures.rbegin(); iter!=_signatures.rend(); ++iter)
				{
					if(iter->second->getLoopClosureId() == s->id())
					{
						iter->second->setLoopClosureId(s->getLoopClosureId());
					}
				}
			}
		}

		if(_memoryChanged &&
			_dbDriver &&
			!s->isBadSignature() &&
			s->id()>0 &&
			((s->getLoopClosureId() == 0 || s->isSaved()) || !_databaseCleaned))
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

Signature * Memory::_getLastSignature()
{
	if(_stMem.size())
	{
		return uValue(_signatures, *_stMem.rbegin(), (Signature*)0);
	}
	return 0;
}

void Memory::addLoopClosureLink(int oldId, int newId, bool rehearsal)
{
	ULOGGER_INFO("old=%d, new=%d", oldId, newId);
	Signature * oldS = _getSignature(oldId);
	Signature * newS = _getSignature(newId);
	if(oldS && newS)
	{
		if(oldS->getLoopClosureId() > 0 && oldS->getLoopClosureId() == newS->id())
		{
			// do nothing, already merged
			newS->setWeight(newS->getWeight() + 1);
		}
		else if(oldS->getLoopClosureId() > 0)
		{
			this->addLoopClosureLink(oldS->getLoopClosureId(), newS->id());
			oldS->setLoopClosureId(newS->id());
		}
		else if(newS->getLoopClosureId() > 0)
		{
			ULOGGER_ERROR("Not supposed to occur!");
		}
		else
		{
			if(_stMem.find(oldS->id()) != _stMem.end() && rehearsal)
			{
				// This happens during rehearsal, use only words of the old signature
				this->merge(oldS, newS, kUseOnlyFromMerging);
				//this->merge(oldS, newS, kFullMerging);

				if(_lastLoopClosureId == oldS->id())
				{
					_lastLoopClosureId = newS->id();
				}
			}
			else
			{
				// This happens in a loop closure, use only words of the newest signature
				this->merge(oldS, newS, kUseOnlyDestMerging);
				//this->merge(oldS, newS, kFullMerging);

				_lastLoopClosureId = newS->id();
			}

			newS->setWeight(newS->getWeight() + 1 + oldS->getWeight());
			oldS->setLoopClosureId(newS->id());

			// keep actions from the old

			// Update neighbors...
			oldS->removeNeighbor(newS->id());
			newS->removeNeighbor(oldS->id());

			const NeighborsMap & neighbors = oldS->getNeighbors();
			for(NeighborsMap::const_iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
			{
				ULOGGER_DEBUG("neighbor of old = %d", i->first);
			}

			for(NeighborsMap::const_iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
			{
				if(i->first != newS->id())
				{
					ULOGGER_DEBUG("add neighbor %d to %d", i->first, newS->id());
					Signature * neighbor = _getSignature(i->first);
					bool newHasNeighbor = newS->hasNeighbor(i->first);
					if(!newHasNeighbor)
					{
						newS->addNeighbor(i->first, i->second);
					}
					if(neighbor)
					{
						bool oldHasNeighbor = neighbor->hasNeighbor(newS->id());
						if(!oldHasNeighbor)
						{
							// Use corresponding actions from the old signature
							std::list<std::vector<float> > actions = uValue(neighbor->getNeighbors(), oldS->id(), std::list<std::vector<float> >());
							neighbor->addNeighbor(newS->id(), actions);
							if(newHasNeighbor)
							{
								UERROR("%d has neighbor %d but not the inverse!?", newS->id(), neighbor->id());
							}
						}
						else if(!newHasNeighbor && oldHasNeighbor)
						{
							UERROR("%d has neighbor %d but not the inverse!?", neighbor->id(), newS->id());
						}
					}
					else if(_dbDriver)
					{
						ULOGGER_DEBUG("*i=%d not found in WM or STM, modifying it in database...", i->first);
						_dbDriver->addNeighbor(i->first, newS->id(), oldS->id());
					}
				}
			}
		}
	}
	else
	{
		if(!newS)
		{
			ULOGGER_FATAL("newId=%d, oldId=%d, Signature %d not found in working/st memories", newId, oldId, newId);
		}
		if(!oldS)
		{
			ULOGGER_FATAL("newId=%d, oldId=%d, Signature %d not found in working/st memories", newId, oldId, oldId);
		}
	}
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
		fprintf(foutTree, "SignatureID ChildsID...\n");

		for(std::map<int, Signature *>::const_iterator i=_signatures.begin(); i!=_signatures.end(); ++i)
		{
			fprintf(foutTree, "%d %d %d\n", i->first, i->second->getLoopClosureId(), i->second->getWeight());
		}

		fclose(foutTree);
	}

}

int Memory::rehearsal(const Signature * signature, bool onlyLast, float & similarity)
{
	UTimer timer;
	similarity = 0;
	std::map<int, float> likelihoodTest;
	// Get parents to compare...
	std::set<int> mem = _stMem;
	mem.erase(signature->id());
	if(onlyLast && mem.size())
	{
		int last = *(--mem.end());
		mem.clear();
		mem.insert(last);
	}
	likelihoodTest = Memory::computeLikelihood(signature, mem);

	UDEBUG("t=%fs", timer.ticks());
	if(likelihoodTest.size() == 0)
	{
		ULOGGER_WARN("likelihoodTest size == 0 ?");
		return 0;
	}

	ULOGGER_DEBUG("Comparing with last signatures...");
	float max = 0;
	int maxId = 0;
	for(std::map<int, float>::reverse_iterator i=likelihoodTest.rbegin(); i!=likelihoodTest.rend(); ++i)
	{
		//ULOGGER_DEBUG("id=%d, value=%f", (*i).first, (*i).second);
		if((*i).second > max || max == 0)
		{
			max = (*i).second;
			maxId = (*i).first;
		}
	}

	/*std::vector<float> values = uValues(likelihoodTest);
	float sum = uSum(values);
	float mean = sum/values.size();
	float stddev = uStdDev(values, mean);
	ULOGGER_DEBUG("sum=%f, Mean=%f, std dev=%f", sum, mean, stddev);*/
	UDEBUG("maxId=%d, maxSim=%f, t=%fs", maxId, max, timer.ticks());
	similarity = max;
	return maxId;
}

void Memory::touch(int signatureId)
{
	std::map<int, int>::iterator iter = _workingMem.find(signatureId);
	if(iter != _workingMem.end())
	{
		iter->second = 0;
	}
}

// recursive (return map<Id,Margin>)
void Memory::getNeighborsId(std::map<int, int> & ids, int signatureId, unsigned int margin, bool checkInDatabase, int ignoredId) const
{
	//ULOGGER_DEBUG("signatureId=%d, neighborsMargin=%d", signatureId, margin);
	if(signatureId>0 && margin > 0)
	{
		// Look up in the short time memory if all ids are here, if not... load them from the database
		const Signature * s = this->getSignature(signatureId);
		std::set<int> neighborIds;
		if(s)
		{
			neighborIds = uKeysSet(s->getNeighbors());
		}
		else if(checkInDatabase && _dbDriver)
		{
			//ULOGGER_DEBUG("DB Neighbors of %d, Margin=%d", signatureId, margin);
			_dbDriver->getNeighborIds(signatureId, neighborIds);
		}

		if(checkInDatabase && neighborIds.size() == 0)
		{
			UERROR("Signature %d doesn't have neighbor!?", signatureId);
		}

		for(std::set<int>::iterator i=neighborIds.begin(); i!=neighborIds.end();)
		{
			std::map<int, int>::iterator iter = ids.find(*i);
			if(*i != ignoredId &&
			   (iter == ids.end() || (unsigned int)iter->second < margin) )
			{
				if(iter != ids.end())
				{
					iter->second = margin;
				}
				else
				{
					ids.insert(std::pair<int,int>(*i, margin));
				}
				++i;
			}
			else
			{
				neighborIds.erase(i++);
			}
		}

		for(std::set<int>::const_iterator i=neighborIds.begin(); i!=neighborIds.end(); ++i)
		{
			this->getNeighborsId(ids, *i, margin-1, checkInDatabase, signatureId);
		}
	}
}

// The data returned must be released
IplImage * Memory::getImage(int id) const
{
	IplImage * img = 0;
	if(_dbDriver)
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
		 }

		 const char * colorA = "red";
		 const char * colorB = "skyBlue";
		 UINFO("Generating map with %d locations", ids.size()+_signatures.size());
		 fprintf(fout, "digraph G {\n");
		 std::set<std::pair<int, int> > linksAdded;
		 for(std::set<int>::iterator i=ids.begin(); i!=ids.end(); ++i)
		 {
			 if(_signatures.find(*i) == _signatures.end())
			 {
				 int id = *i;
				 int loopId = 0;
				 _dbDriver->getLoopClosureId(id, loopId);
				 if(!loopId)
				 {
					 NeighborsMap neighbors;
					 _dbDriver->loadNeighbors(id, neighbors);
					 int weight = 0;
					 _dbDriver->getWeight(id, weight);
					 for(NeighborsMap::iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
					 {
						 if(_signatures.find(iter->first) == _signatures.end() /*&& linksAdded.find(std::pair<int, int>(id, *iter)) == linksAdded.end()*/)
						 {
							 int weightNeighbor = 0;
							 _dbDriver->getWeight(iter->first, weightNeighbor);
							 linksAdded.insert(std::pair<int, int>(iter->first, id));
							 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"%d\", fontcolor=%s, fontsize=8];\n",
									 id,
									 weight,
									 iter->first,
									 weightNeighbor,
									 (int)iter->second.size(),
									 iter->second.size()>0?colorA:colorB);
						 }
					 }
				 }
			 }
		 }
		 for(std::map<int, Signature*>::iterator i=_signatures.begin(); i!=_signatures.end(); ++i)
		 {
			 int id = i->second->id();
			 const NeighborsMap & neighbors = i->second->getNeighbors();
			 int weight = i->second->getWeight();
			 for(NeighborsMap::const_iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
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
							 (int)iter->second.size(),
							 iter->second.size()>0?colorA:colorB);
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
		if(!s || (s && s->getLoopClosureId() == 0 && _stMem.find(s->id()) == _stMem.end()))
		{
			_memoryChanged = true;
			if(strToRemove.size())
			{
				strToRemove.append(" OR ");
			}
			strToRemove.append("id=");
			strToRemove.append(uNumber2str(*iter));

			if(s)
			{
				// Hack, to unreference neighbors without redirecting loop closure id to children
				s->setLoopClosureId(-1);
				this->moveToTrash(s);
			}
			else if(_dbDriver)
			{
				// remove reference from active neighbors
				std::set<int> neighbors;
				_dbDriver->getNeighborIds(*iter, neighbors);
				for(std::set<int>::iterator jter=neighbors.begin(); jter!=neighbors.end(); ++jter)
				{
					s = this->_getSignature(*jter);
					if(s)
					{
						s->removeNeighbor(*iter);
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

	std::map<int, int> neighbors;
	this->getNeighborsId(neighbors, parent->id(), 1, true);
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
