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

#include <utilite/UEventsManager.h>
#include <utilite/ULogger.h>
#include <utilite/UTimer.h>
#include <utilite/UConversion.h>
#include <utilite/UProcessInfo.h>
#include <utilite/UMath.h>

#include "rtabmap/core/Memory.h"
#include "Signature.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/VWDictionary.h"
#include "VisualWord.h"
#include "rtabmap/core/Features2d.h"
#include "DBDriverSqlite3.h"

namespace rtabmap {

const int Memory::kIdStart = 1;
const int Memory::kIdVirtual = -1;
const int Memory::kIdInvalid = 0;

Memory::Memory(const ParametersMap & parameters) :
	_dbDriver(0),
	_similarityThreshold(Parameters::defaultMemRehearsalSimilarity()),
	_rehearsalOnlyWithLast(Parameters::defaultMemRehearsalOnlyWithLast()),
	_rawDataKept(Parameters::defaultMemImageKept()),
	_incrementalMemory(Parameters::defaultMemIncrementalMemory()),
	_maxStMemSize(Parameters::defaultMemSTMSize()),
	_recentWmRatio(Parameters::defaultMemRecentWmRatio()),
	_oldDataKeptOnRehearsal(Parameters::defaultMemRehearsalOldDataKept()),
	_idUpdatedToNewOneRehearsal(Parameters::defaultMemRehearsalIdUpdatedToNewOne()),
	_idCount(kIdStart),
	_lastSignature(0),
	_lastLoopClosureId(0),
	_memoryChanged(false),
	_signaturesAdded(0),

	_keypointDetector(0),
	_keypointDescriptor(0),
	_reactivatedWordsComparedToNewWords(Parameters::defaultKpReactivatedWordsComparedToNewWords()),
	_badSignRatio(Parameters::defaultKpBadSignRatio()),
	_tfIdfLikelihoodUsed(Parameters::defaultKpTfIdfLikelihoodUsed()),
	_parallelized(Parameters::defaultKpParallelized())
{
	_vwd = new VWDictionary(parameters);
	this->parseParameters(parameters);
}

bool Memory::init(const std::string & dbUrl, bool dbOverwritten, const ParametersMap & parameters)
{
	ULOGGER_DEBUG("");
	this->parseParameters(parameters);

	UEventsManager::post(new RtabmapEventInit("Clearing memory..."));
	DBDriver * tmpDriver = 0;
	if(!_memoryChanged)
	{
		if(_dbDriver)
		{
			tmpDriver = _dbDriver;
			_dbDriver = 0; // HACK for the clear() below to think that there is no db
		}
	}
	this->clear();
	UEventsManager::post(new RtabmapEventInit("Clearing memory, done!"));

	_dbDriver = tmpDriver;
	if(_dbDriver)
	{
		UEventsManager::post(new RtabmapEventInit("Closing database connection..."));
		_dbDriver->closeConnection();
		UEventsManager::post(new RtabmapEventInit("Closing database connection, done!"));
	}

	if(_dbDriver == 0)
	{
		_dbDriver = new DBDriverSqlite3(parameters);
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
			_dbDriver->loadLastNodes(dbSignatures);
			for(std::list<Signature*>::reverse_iterator iter=dbSignatures.rbegin(); iter!=dbSignatures.rend(); ++iter)
			{
				_signatures.insert(std::pair<int, Signature *>((*iter)->id(), *iter));
				if((int)_stMem.size() <= _maxStMemSize)
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
			_dbDriver->getLastNodeId(_idCount);
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

	_workingMem.insert(kIdVirtual);

	ULOGGER_DEBUG("ids start with %d", _idCount);


	// Now load the dictionary if we have a connection
	if(_dbDriver && _dbDriver->isConnected())
	{
		UEventsManager::post(new RtabmapEventInit(std::string("Loading dictionary...")));
		_dbDriver->load(_vwd);
		UDEBUG("%d words loaded!", _vwd->getUnusedWordsSize());
		UEventsManager::post(new RtabmapEventInit(std::string("Loading dictionary, done! (") + uNumber2Str(int(_vwd->getUnusedWordsSize())) + " words)"));
	}

	UEventsManager::post(new RtabmapEventInit(std::string("Adding word references...")));
	// Enable loaded signatures
	Signature * ss;
	const std::map<int, Signature *> & signatures = this->getSignatures();
	for(std::map<int, Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
	{
		ss = this->_getSignature(i->first);
		if(ss)
		{
			const std::multimap<int, cv::KeyPoint> & words = ss->getWords();
			if(words.size())
			{
				UDEBUG("node=%d, word references=%d", ss->id(), words.size());
				for(std::multimap<int, cv::KeyPoint>::const_iterator iter = words.begin(); iter!=words.end(); ++iter)
				{
					_vwd->addWordRef(iter->first, i->first);
				}
				ss->setEnabled(true);
			}
		}
	}

	UEventsManager::post(new RtabmapEventInit(uFormat("Adding word references, done! (%d)", _vwd->getTotalActiveReferences())));

	if(_vwd->getUnusedWordsSize())
	{
		UWARN("_vwd->getUnusedWordsSize() must be empty... size=%d", _vwd->getUnusedWordsSize());
	}
	ULOGGER_DEBUG("Total word references added = %d", _vwd->getTotalActiveReferences());

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

	if(_keypointDetector)
	{
		delete _keypointDetector;
	}
	if(_keypointDescriptor)
	{
		delete _keypointDescriptor;
	}
	if(_vwd)
	{
		delete _vwd;
	}
}

void Memory::parseParameters(const ParametersMap & parameters)
{
	ULOGGER_DEBUG("");
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kMemRehearsalSimilarity())) != parameters.end())
	{
		this->setSimilarityThreshold(std::atof((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kMemRehearsalOnlyWithLast())) != parameters.end())
	{
		_rehearsalOnlyWithLast = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kMemImageKept())) != parameters.end())
	{
		_rawDataKept = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kMemIncrementalMemory())) != parameters.end())
	{
		_incrementalMemory = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kMemSTMSize())) != parameters.end())
	{
		this->setMaxStMemSize(std::atoi((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kMemRecentWmRatio())) != parameters.end())
	{
		this->setRecentWmRatio(std::atof((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kMemRehearsalOldDataKept())) != parameters.end())
	{
		_oldDataKeptOnRehearsal = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kMemRehearsalIdUpdatedToNewOne())) != parameters.end())
	{
		_idUpdatedToNewOneRehearsal = uStr2Bool((*iter).second.c_str());
	}
	if(_dbDriver)
	{
		_dbDriver->parseParameters(parameters);
	}

	// verification...
	if(!_idUpdatedToNewOneRehearsal && !_rehearsalOnlyWithLast)
	{
		UWARN("if _idUpdatedToNewOneRehearsal=false, _rehearsalOnlyWithLast must be true");
		_rehearsalOnlyWithLast = true;
	}


	// Keypoint stuff
	if(_vwd)
	{
		_vwd->parseParameters(parameters);
	}

	if((iter=parameters.find(Parameters::kKpReactivatedWordsComparedToNewWords())) != parameters.end())
	{
		_reactivatedWordsComparedToNewWords = uStr2Bool((*iter).second.c_str());
	}

	if((iter=parameters.find(Parameters::kKpTfIdfLikelihoodUsed())) != parameters.end())
	{
		_tfIdfLikelihoodUsed = uStr2Bool((*iter).second.c_str());
	}

	if((iter=parameters.find(Parameters::kKpParallelized())) != parameters.end())
	{
		_parallelized = uStr2Bool((*iter).second.c_str());
	}

	if((iter=parameters.find(Parameters::kKpBadSignRatio())) != parameters.end())
	{
		_badSignRatio = std::atof((*iter).second.c_str());
	}

	//Keypoint detector
	KeypointDetector::DetectorType detectorStrategy = KeypointDetector::kDetectorUndef;
	if((iter=parameters.find(Parameters::kKpDetectorStrategy())) != parameters.end())
	{
		detectorStrategy = (KeypointDetector::DetectorType)std::atoi((*iter).second.c_str());
	}
	if(!_keypointDetector ||  detectorStrategy!=KeypointDetector::kDetectorUndef)
	{
		ULOGGER_DEBUG("new detector strategy %d", int(detectorStrategy));
		if(_keypointDetector)
		{
			delete _keypointDetector;
			_keypointDetector = 0;
		}
		switch(detectorStrategy)
		{
		case KeypointDetector::kDetectorSift:
			_keypointDetector = new SIFTDetector(parameters);
			break;
		case KeypointDetector::kDetectorSurf:
		default:
			_keypointDetector = new SURFDetector(parameters);
			break;
		}
	}
	else if(_keypointDetector)
	{
		_keypointDetector->parseParameters(parameters);
	}

	//Keypoint descriptor
	KeypointDescriptor::DescriptorType descriptorStrategy = KeypointDescriptor::kDescriptorUndef;
	if((iter=parameters.find(Parameters::kKpDescriptorStrategy())) != parameters.end())
	{
		descriptorStrategy = (KeypointDescriptor::DescriptorType)std::atoi((*iter).second.c_str());
	}
	if(!_keypointDescriptor || descriptorStrategy!=KeypointDescriptor::kDescriptorUndef)
	{
		ULOGGER_DEBUG("new descriptor strategy %d", int(descriptorStrategy));
		if(_keypointDescriptor)
		{
			delete _keypointDescriptor;
			_keypointDescriptor = 0;
		}
		switch(descriptorStrategy)
		{
		case KeypointDescriptor::kDescriptorSift:
			_keypointDescriptor = new SIFTDescriptor(parameters);
			break;
		case KeypointDescriptor::kDescriptorSurf:
		default:
			_keypointDescriptor = new SURFDescriptor(parameters);
			break;
		}
	}
	else if(_keypointDescriptor)
	{
		_keypointDescriptor->parseParameters(parameters);
	}
}

void Memory::preUpdate()
{
	_signaturesAdded = 0;
	this->cleanUnusedWords();
	if(_vwd && !_parallelized)
	{
		//When parallelized, it is done in CreateSignature
		_vwd->update();
	}
}

bool Memory::update(const Image & image, std::map<std::string, float> & stats)
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
	Signature * signature = this->createSignature(image, this->isRawDataKept());
	if (signature == 0)
	{
		UERROR("Failed to create a signature");
		return false;
	}

	// It will be added to the short-term memory, no need to delete it...
	this->addSignatureToStm(signature);

	_lastSignature = signature;

	if(_lastLoopClosureId == 0 && !signature->isBadSignature())
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
	// Transfer the oldest signature of the short-term memory to the working memory
	//============================================================
	while(_stMem.size() && (int)_stMem.size() > _maxStMemSize)
	{
		UDEBUG("Inserting node %d from STM in WM...", *_stMem.begin());
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

void Memory::setSimilarityThreshold(float similarity)
{
	if(similarity>=0 && similarity <=1)
	{
		_similarityThreshold = similarity;
	}
	else
	{
		ULOGGER_ERROR("Error similarity threshold \"%f\" (must be between 0 and 1)", similarity);
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

void Memory::addSignatureToStm(Signature * signature)
{
	UTimer timer;
	// add signature on top of the short-term memory
	if(signature)
	{
		UDEBUG("adding %d", signature->id());
		// Update neighbors
		if(_stMem.size())
		{
			// If you want to use Bayes's parameter _predictionOnNonNullActionsOnly with no actions, dummy actions must be sent (null actions are not handled in sqlite3 driver)
			_signatures.at(*_stMem.rbegin())->addNeighbor(signature->id());
			// actions are not backward compatible, so set null actions
			signature->addNeighbor(*_stMem.rbegin());
			UDEBUG("Min STM id = %d", *_stMem.begin());
		}

		_signatures.insert(_signatures.end(), std::pair<int, Signature *>(signature->id(), signature));
		_stMem.insert(_stMem.end(), signature->id());

		if(_vwd)
		{
			ULOGGER_DEBUG("%d words ref for the signature %d", signature->getWords().size(), signature->id());
		}
		if(signature->getWords().size())
		{
			signature->setEnabled(true);
		}
	}

	UDEBUG("time = %fs", timer.ticks());
}

void Memory::addSignatureToWm(Signature * signature)
{
	if(signature)
	{
		UDEBUG("Inserting node %d in WM...", signature->id());
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

int Memory::getVWDictionarySize() const
{
	return _vwd->getVisualWords().size();
}

std::set<int> Memory::getNeighborLinks(int signatureId, bool ignoreNeighborByLoopClosure, bool lookInDatabase) const
{
	std::set<int> links;
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
				const std::set<int> & neighbors = s->getNeighbors();
				links.insert(neighbors.begin(), neighbors.end());
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
					const std::set<int> & neighbors = s->getNeighbors();
					links.insert(neighbors.begin(), neighbors.end());
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
		std::set<int> neighbors;
		_dbDriver->loadNeighbors(signatureId, neighbors);
		links.insert(neighbors.begin(), neighbors.end());
	}
	else
	{
		UWARN("Cannot find signature %d in memory", signatureId);
	}
	return links;
}

// return map<Id,Margin>, including signatureId
// maxCheckedInDatabase = -1 means no limit to check in database (default)
// maxCheckedInDatabase = 0 means don't check in database
std::map<int, int> Memory::getNeighborsId(int signatureId,
		unsigned int margin,
		int maxCheckedInDatabase, // default -1 (no limit)
		bool incrementMarginOnLoop, // default false
		bool ignoreLoopIds, // default false
		double * dbAccessTime
		) const
{
	//ULOGGER_DEBUG("signatureId=%d, neighborsMargin=%d", signatureId, margin);
	if(dbAccessTime)
	{
		*dbAccessTime = 0;
	}
	std::map<int, int> ids;
	if(signatureId<=0)
	{
		return ids;
	}
	int nbLoadedFromDb = 0;
	std::list<int> curentMarginList;
	std::set<int> currentMargin;
	std::set<int> nextMargin;
	nextMargin.insert(signatureId);
	unsigned int m = 0;
	while(m < margin && nextMargin.size())
	{
		curentMarginList = std::list<int>(nextMargin.begin(), nextMargin.end());
		nextMargin.clear();
		// first pass: count number of node in current margin in database
		for(std::list<int>::iterator jter = curentMarginList.begin(); jter!=curentMarginList.end();++jter)
		{
			if(!uContains(ids, *jter))
			{
				const Signature * s = this->getSignature(*jter);
				if(!s)
				{
					++nbLoadedFromDb;
				}
			}
		}

		for(std::list<int>::iterator jter = curentMarginList.begin(); jter!=curentMarginList.end(); ++jter)
		{
			if(ids.insert(std::pair<int, int>(*jter, m)).second)
			{
				//UDEBUG("Added %d with margin %d", *jter, m);
				// Look up in STM/WM if all ids are here, if not... load them from the database
				const Signature * s = this->getSignature(*jter);
				std::set<int> tmpNeighborIds;
				std::set<int> tmpLoopClosureIds;
				std::set<int> tmpChildLoopClosureIds;
				const std::set<int> * neighborIds = &tmpNeighborIds;
				const std::set<int> * loopClosureIds = &tmpLoopClosureIds;
				const std::set<int> * childLoopClosureIds = &tmpChildLoopClosureIds;
				if(s)
				{
					neighborIds = &s->getNeighbors();

					if(!ignoreLoopIds)
					{
						loopClosureIds = &s->getLoopClosureIds();
						childLoopClosureIds = &s->getChildLoopClosureIds();
					}
				}
				else if(maxCheckedInDatabase == -1 || (maxCheckedInDatabase > 0 && _dbDriver && nbLoadedFromDb < maxCheckedInDatabase))
				{
					UTimer timer;
					_dbDriver->getNeighborIds(*jter, tmpNeighborIds);
					if(!ignoreLoopIds)
					{
						_dbDriver->getLoopClosureIds(*jter, tmpLoopClosureIds, tmpChildLoopClosureIds);
					}
					if(dbAccessTime)
					{
						*dbAccessTime += timer.getElapsedTime();
					}
				}

				// Neighbor links
				for(std::set<int>::const_iterator iter=neighborIds->begin(); iter!=neighborIds->end(); ++iter)
				{
					if( (ignoreLoopIds || (loopClosureIds->find(*iter) == loopClosureIds->end() && childLoopClosureIds->find(*iter) == childLoopClosureIds->end())) &&
						!uContains(ids, *iter) &&
						nextMargin.find(*iter) == nextMargin.end())
					{
						nextMargin.insert(*iter);
					}
				}

				// Parent links
				for(std::set<int>::const_iterator iter=loopClosureIds->begin(); iter!=loopClosureIds->end(); ++iter)
				{
					if( *iter && !uContains(ids, *iter)/* && isInNeighborLimits(*iter, limits)*/)
					{
						if(incrementMarginOnLoop)
						{
							nextMargin.insert(*iter);
							//UDEBUG("next of %d + %d", *jter, *iter);
						}
						else
						{
							if(currentMargin.insert(*iter).second)
							{
								const Signature * s = this->getSignature(*iter);
								if(!s)
								{
									// update db count because it's on current margin
									++nbLoadedFromDb;
								}
								curentMarginList.push_back(*iter);
								//UDEBUG("current of %d + %d", *jter, *iter);
							}
						}
					}
				}

				//Child links
				for(std::set<int>::const_iterator iter=childLoopClosureIds->begin(); iter!=childLoopClosureIds->end(); ++iter)
				{
					if( *iter && !uContains(ids, *iter)/* && isInNeighborLimits(*iter, limits)*/)
					{
						if(incrementMarginOnLoop)
						{
							nextMargin.insert(*iter);
							//UDEBUG("next of %d + %d", *jter, *iter);
						}
						else
						{
							if(currentMargin.insert(*iter).second)
							{
								const Signature * s = this->getSignature(*iter);
								if(!s)
								{
									// update db count because it's on current margin
									++nbLoadedFromDb;
								}
								curentMarginList.push_back(*iter);
								//UDEBUG("current of %d + %d", *jter, *iter);
							}
						}
					}
				}
			}
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
		_dbDriver->getAllNodeIds(ids);
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

	this->cleanUnusedWords();

	if(_dbDriver)
	{
		_dbDriver->addStatisticsAfterRunSurf(_vwd->getVisualWords().size());
	}

	if(_dbDriver)
	{
		_dbDriver->emptyTrashes();
		_dbDriver->join();
	}

	// Save some stats to the db, save only when the mem is not empty
	if(_dbDriver && (_stMem.size() || _workingMem.size()))
	{
		unsigned int memSize = _workingMem.size() + _stMem.size();
		if(_workingMem.size() && *_workingMem.begin() < 0)
		{
			--memSize;
		}
		if(memSize != _signatures.size())
		{
			// this is only a safe check...not supposed to occur.
			ULOGGER_ERROR("The number of signatures don't match! _workingMem=%d, _stMem=%d, _signatures=%d", _workingMem.size(), _stMem.size(), _signatures.size());
		}

		ULOGGER_DEBUG("Adding statistics after run...");
		_dbDriver->addStatisticsAfterRun(memSize, _lastSignature?_lastSignature->id():0, UProcessInfo::getMemoryUsage(), _dbDriver->getMemoryUsed());
	}
	ULOGGER_DEBUG("");

	//Get the tree root (parents)
	std::map<int, Signature*> mem = _signatures;
	for(std::map<int, Signature *>::iterator i=mem.begin(); i!=mem.end(); ++i)
	{
		if(i->second)
		{
			ULOGGER_DEBUG("deleting from the working and the short-term memory: %d", i->first);
			this->moveToTrash(i->second);
		}
	}

	if(_workingMem.size() != 0 && !(_workingMem.size() == 1 && *_workingMem.begin() == kIdVirtual))
	{
		ULOGGER_ERROR("_workingMem must be empty here, size=%d", _workingMem.size());
	}
	_workingMem.clear();
	if(_stMem.size() != 0)
	{
		ULOGGER_ERROR("_stMem must be empty here, size=%d", _stMem.size());
	}
	_stMem.clear();
	if(_signatures.size()!=0)
	{
		ULOGGER_ERROR("_signatures must be empty here, size=%d", _signatures.size());
	}
	_signatures.clear();

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

	if(_dbDriver)
	{
		_dbDriver->join(true);
		cleanUnusedWords();
		_dbDriver->emptyTrashes();
	}
	else
	{
		cleanUnusedWords();
	}
	if(_vwd)
	{
		_vwd->clear();
	}
}

/**
 * Compute the likelihood of the signature with some others in the memory.
 * Important: Assuming that all other ids are under 'signature' id.
 * If an error occurs, the result is empty.
 */
std::map<int, float> Memory::computeLikelihood(const Signature * signature, const std::list<int> & ids)
{
	if(!_tfIdfLikelihoodUsed)
	{
		UTimer timer;
		timer.start();
		std::map<int, float> likelihood;

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

		for(std::list<int>::const_iterator iter = ids.begin(); iter!=ids.end(); ++iter)
		{
			float sim = 0.0f;
			if(*iter > 0)
			{
				const Signature * sB = this->getSignature(*iter);
				if(!sB)
				{
					UFATAL("Signature %d not found in WM ?!?", *iter);
				}
				sim = signature->compareTo(sB);
			}

			likelihood.insert(likelihood.end(), std::pair<int, float>(*iter, sim));
		}

		ULOGGER_DEBUG("compute likelihood... %f s", timer.ticks());
		return likelihood;
	}
	else
	{
		// TODO cleanup , old way...
		UTimer timer;
		timer.start();
		std::map<int, float> likelihood;
		std::map<int, float> calculatedWordsRatio;

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

		for(std::list<int>::const_iterator iter = ids.begin(); iter!=ids.end(); ++iter)
		{
			likelihood.insert(likelihood.end(), std::pair<int, float>(*iter, 0.0f));
		}

		const std::list<int> & wordIds = uUniqueKeys(signature->getWords());

		float nwi; // nwi is the number of a specific word referenced by a place
		float ni; // ni is the total of words referenced by a place
		float nw; // nw is the number of places referenced by a specific word
		float N; // N is the total number of places

		float logNnw;
		const VisualWord * vw;

		N = this->getSignatures().size();

		if(N)
		{
			ULOGGER_DEBUG("processing... ");
			// Pour chaque mot dans la signature SURF
			for(std::list<int>::const_iterator i=wordIds.begin(); i!=wordIds.end(); ++i)
			{
				// "Inverted index" - Pour chaque endroit contenu dans chaque mot
				vw = _vwd->getWord(*i);
				if(vw)
				{
					const std::map<int, int> & refs = vw->getReferences();
					nw = refs.size();
					if(nw)
					{
						logNnw = log10(N/nw);
						if(logNnw)
						{
							for(std::map<int, int>::const_iterator j=refs.begin(); j!=refs.end(); ++j)
							{
								std::map<int, float>::iterator iter = likelihood.find(j->first);
								if(iter != likelihood.end())
								{
									nwi = j->second;
									ni = this->getNi(j->first);
									if(ni != 0)
									{
										//ULOGGER_DEBUG("%d, %f %f %f %f", vw->id(), logNnw, nwi, ni, ( nwi  * logNnw ) / ni);
										iter->second += ( nwi  * logNnw ) / ni;
									}
								}
							}
						}
					}
				}
			}
		}

		ULOGGER_DEBUG("compute likelihood %f s", timer.ticks());
		return likelihood;
	}
}

// Weights of the signatures in the working memory <signature id, weight>
std::map<int, int> Memory::getWeights() const
{
	std::map<int, int> weights;
	for(std::set<int>::const_iterator iter=_workingMem.begin(); iter!=_workingMem.end(); ++iter)
	{
		if(*iter > 0)
		{
			const Signature * s = this->getSignature(*iter);
			if(!s)
			{
				UFATAL("Location %d must exist in memory", *iter);
			}
			weights.insert(weights.end(), std::make_pair(*iter, s->getWeight()));
		}
		else
		{
			weights.insert(weights.end(), std::make_pair(*iter, -1));
		}
	}
	return weights;
}

int Memory::forget(const std::set<int> & ignoredIds)
{
	ULOGGER_DEBUG("");
	int signaturesRemoved = 0;
	if(_vwd->isIncremental())
	{
		int newWords = 0;
		int wordsRemoved = 0;

		// Get how many new words added for the last run...
		newWords = _vwd->getNotIndexedWordsCount();

		// So we need to remove at least "newWords" words from the
		// dictionary to respect the limit.
		while(wordsRemoved < newWords)
		{
			std::list<Signature *> signatures = this->getRemovableSignatures(1, ignoredIds);
			if(signatures.size())
			{
				Signature *  s = dynamic_cast<Signature *>(signatures.front());
				if(s)
				{
					++signaturesRemoved;
					this->moveToTrash(s);
					wordsRemoved = _vwd->getUnusedWordsSize();
				}
				else
				{
					break;
				}
			}
			else
			{
				break;
			}
		}
		ULOGGER_DEBUG("newWords=%d, wordsRemoved=%d", newWords, wordsRemoved);
	}
	else
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
		signaturesRemoved = (int)signatures.size();
	}
	return signaturesRemoved;
}


int Memory::cleanup(const std::list<int> & ignoredIds)
{
	ULOGGER_DEBUG("");
	int signaturesRemoved = 0;

	// bad signature
	if(_lastSignature->isBadSignature() || !_incrementalMemory)
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
		UDEBUG("");
		_dbDriver->join();
		UDEBUG("");
	}
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

		// Ignore neighbor of the last location in STM (for neighbor links redirection issue during Rehearsal).
		Signature * lastInSTM = 0;
		if(_stMem.size())
		{
			lastInSTM = _signatures.at(*_stMem.begin());
		}

		for(std::set<int>::const_iterator memIter = wm.begin(); memIter != wm.end(); ++memIter)
		{
			if( (recentWmImmunized && *memIter > _lastLoopClosureId) ||
				*memIter == _lastLoopClosureId)
			{
				// ignore recent memory
			}
			else if(*memIter > 0 && ignoredIds.find(*memIter) == ignoredIds.end() && (!lastInSTM || !lastInSTM->hasNeighbor(*memIter)))
			{
				Signature * s = this->_getSignature(*memIter);
				if(s)
				{
					// Its loop closures must not be in STM to be removable, rehearsal issue
					bool foundInSTM = false;
					for(std::set<int>::const_iterator iter = s->getLoopClosureIds().begin(); iter!=s->getLoopClosureIds().end(); ++iter)
					{
						if(_stMem.find(*iter) != _stMem.end())
						{
							UDEBUG("Ignored %d because it has a parent (%d) in STM", s->id(), *iter);
							foundInSTM = true;
							break;
						}
					}
					// Its neighbors must not be in STM to be removable, rehearsal issue
					if(!foundInSTM)
					{
						for(std::set<int>::iterator iter = _stMem.begin(); iter!=_stMem.end(); ++iter)
						{
							if(s->hasNeighbor(*iter))
							{
								UDEBUG("Ignored %d because it has a neighbor (%d) in STM", s->id(), *iter);
								foundInSTM = true;
								break;
							}
						}
					}
					if(!foundInSTM)
					{
						// less weighted signature priority to be transferred
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
		this->disableWordsRef(s->id());

		_workingMem.erase(s->id());
		_stMem.erase(s->id());
		_signatures.erase(s->id());

		if(_lastSignature == s)
		{
			_lastSignature = 0;
		}

		// Remove neighbor links with bad signatures (assuming that is done in cleanup),
		// the bad signatures are always the last signature added.
		if(s->isBadSignature())
		{
			const std::set<int> & neighbors = s->getNeighbors();
			for(std::set<int>::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				Signature * n = this->_getSignature(*iter);
				// neighbor to s
				if(n)
				{
					n->removeNeighbor(s->id());
				}
			}
			s->removeNeighbors();
		}

		if(	_dbDriver &&
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

		UDEBUG("Add loop closure link between %d and %d", oldS->id(), newS->id());

		bool onRehearsal = this->isInSTM(oldS->id());
		if(!onRehearsal)
		{
			// During loop closure in WM
			oldS->addLoopClosureId(newS->id());
			newS->addChildLoopClosureId(oldS->id());
			_lastLoopClosureId = newS->id();
			newS->setWeight(newS->getWeight() + oldS->getWeight());
			oldS->setWeight(0);
			return true; // RETURN
		}

		// During rehearsal in STM
		if(_idUpdatedToNewOneRehearsal)
		{
			// update weight
			newS->setWeight(newS->getWeight() + 1 + oldS->getWeight());

			oldS->addLoopClosureId(newS->id()); // to keep track of the merged location

			if(_lastLoopClosureId == oldS->id())
			{
				_lastLoopClosureId = newS->id();
			}
		}
		else
		{
			// update weight
			oldS->setWeight(newS->getWeight() + 1 + oldS->getWeight());

			newS->addLoopClosureId(oldS->id()); // to keep track of the merged location

			if(_lastSignature == newS)
			{
				_lastSignature = oldS;
			}
		}

		if(_idUpdatedToNewOneRehearsal)
		{
			// redirect neighbor links
			std::set<int> neighbors = oldS->getNeighbors();
			for(std::set<int>::const_iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				int link = *iter;
				if(link != newS->id() && link != oldS->id())
				{
					Signature * s = this->_getSignature(link);
					if(s)
					{
						// modify neighbor "from"
						s->changeNeighborIds(oldS->id(), newS->id());

						if(!newS->hasNeighbor(link))
						{
							newS->addNeighbor(link);
						}
					}
					else
					{
						UERROR("Didn't find neighbor %d of %d in RAM...", link, oldS->id());
					}
				}
			}
			oldS->removeNeighbors();

			// redirect child loop closure links
			std::set<int> childIds = oldS->getChildLoopClosureIds();
			for(std::set<int>::iterator iter = childIds.begin(); iter!=childIds.end(); ++iter)
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
					UERROR("A location (%d, child of %d) in WM/STM cannot be transferred if its loop closure id is in STM", *iter, oldS->id());
				}
				oldS->removeChildLoopClosureId(*iter);
			}
		}

		//remove mutual links
		oldS->removeNeighbor(newId);
		newS->removeNeighbor(oldId);

		if(_oldDataKeptOnRehearsal && _idUpdatedToNewOneRehearsal)
		{
			// Set old image to new signature
			this->copyData(oldS, newS);
		}
		else if(!_oldDataKeptOnRehearsal && !_idUpdatedToNewOneRehearsal)
		{
			this->copyData(newS, oldS);
		}

		// remove location
		moveToTrash(_idUpdatedToNewOneRehearsal?oldS:newS);

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
	this->dumpDictionary((directory+"DumpMemoryWordRef.txt").c_str(), (directory+"DumpMemoryWordDesc.txt").c_str());
	this->dumpSignatures((directory + "DumpMemorySign.txt").c_str());
	this->dumpMemoryTree((directory + "DumpMemoryTree.txt").c_str());
}

void Memory::dumpDictionary(const char * fileNameRef, const char * fileNameDesc) const
{
	if(_vwd)
	{
		_vwd->exportDictionary(fileNameRef, fileNameDesc);
	}
}

void Memory::dumpSignatures(const char * fileNameSign) const
{
	FILE* foutSign = 0;
#ifdef _MSC_VER
	fopen_s(&foutSign, fileNameSign, "w");
#else
	foutSign = fopen(fileNameSign, "w");
#endif

	if(foutSign)
	{
		fprintf(foutSign, "SignatureID WordsID...\n");
		const std::map<int, Signature *> & signatures = this->getSignatures();
		for(std::map<int, Signature *>::const_iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
		{
			fprintf(foutSign, "%d ", iter->first);
			const Signature * ss = dynamic_cast<const Signature *>(iter->second);
			if(ss)
			{
				const std::multimap<int, cv::KeyPoint> & ref = ss->getWords();
				for(std::multimap<int, cv::KeyPoint>::const_iterator jter=ref.begin(); jter!=ref.end(); ++jter)
				{
					fprintf(foutSign, "%d ", (*jter).first);
				}
			}
			fprintf(foutSign, "\n");
		}
		fclose(foutSign);
	}
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

	if(_rehearsalOnlyWithLast && mem.size())
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
	// similarities must be between 0 and 1
	similarities = Memory::computeLikelihood(signature, ids);

	UDEBUG("t=%fs", timer.ticks());
	if(similarities.size() == 0)
	{
		ULOGGER_WARN("similarities size == 0 ?");
		return;
	}

	//============================================================
	// Compare
	//============================================================
	ULOGGER_DEBUG("Comparing with last signatures...");
	int id = 0;
	int nbMerged = 0;
	std::list<int> merged;
	float maxSim = 0.0f;
	for(std::map<int, float>::iterator iter=similarities.begin(); iter!=similarities.end(); ++iter)
	{
		float value = iter->second;
		id = iter->first;
		if(maxSim<=value)
		{
			maxSim = value;
		}
		if(value >= _similarityThreshold)
		{
			if(_incrementalMemory)
			{
				merged.push_back(id);
				this->addLoopClosureLink(id, signature->id());
			}
			else
			{
				Signature * s = _signatures.at(id);
				if(s)
				{
					signature->setWeight(signature->getWeight() + 1 + s->getWeight());
				}
				else
				{
					UFATAL("not supposed to happen");
				}
			}
			++nbMerged;
		}
	}

	if(merged.size()> 1)
	{
		for(std::list<int>::iterator iter=merged.begin(); iter!=merged.end(); ++iter)
		{
			UDEBUG("Multiple merge on rehearsal of %d with %d", signature->id(), *iter);
		}
	}

	stats.insert(std::pair<std::string, float>("Rehearsal/last merged/", merged.size()?*merged.rbegin():0));
	stats.insert(std::pair<std::string, float>("Rehearsal/nb merged/", nbMerged));
	stats.insert(std::pair<std::string, float>("Rehearsal/max similarity/", maxSim));

	UDEBUG("nbMerged=%d, lastMerged=%d, t=%fs", nbMerged, merged.size()?*merged.rbegin():0, timer.ticks());
}

cv::Mat Memory::getImage(int signatureId) const
{
	cv::Mat image;
	const Signature * s = this->getSignature(signatureId);
	if(s)
	{
		image = s->getImage();
	}
	if(image.empty() && this->isRawDataKept() && _dbDriver)
	{
		_dbDriver->getImage(signatureId, image);
	}
	return image;
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
			 _dbDriver->getAllNodeIds(ids);
			 UDEBUG("ids.size()=%d", ids.size());
			 for(std::map<int, Signature*>::iterator iter=_signatures.begin(); iter!=_signatures.end(); ++iter)
			 {
				 ids.insert(iter->first);
			 }
		 }

		 const char * colorG = "green";
		 const char * colorP = "pink";
;		 UINFO("Generating map with %d locations", ids.size());
		 fprintf(fout, "digraph G {\n");
		 for(std::set<int>::iterator i=ids.begin(); i!=ids.end(); ++i)
		 {
			 if(_signatures.find(*i) == _signatures.end())
			 {
				 int id = *i;
				 std::set<int> loopIds;
				 std::set<int> childIds;
				 _dbDriver->getLoopClosureIds(id, loopIds, childIds);

				 std::set<int> neighbors;
				 _dbDriver->loadNeighbors(id, neighbors);
				 int weight = 0;
				 _dbDriver->getWeight(id, weight);
				 for(std::set<int>::iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
				 {
					 if(id!=*iter)
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
						 UDEBUG("Add neighbor link from %d to %d", id, *iter);
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\"\n",
								 id,
								 weight,
								 *iter,
								 weightNeighbor);
					 }
				 }

				 // loop closure links...
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
					 UDEBUG("Add loop link from %d to %d", id, *iter);
					 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"L\", fontcolor=%s, fontsize=8];\n",
							 id,
							 weight,
							 *iter,
							 weightNeighbor,
							 colorG);
				 }
				 for(std::set<int>::iterator iter = childIds.begin(); iter!=childIds.end(); ++iter)
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
					 UDEBUG("Add child link from %d to %d", id, *iter);
					 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"C\", fontcolor=%s, fontsize=8];\n",
							 id,
							 weight,
							 *iter,
							 weightNeighbor,
							 colorP);
				 }
			 }
		 }
		 for(std::map<int, Signature*>::iterator i=_signatures.begin(); i!=_signatures.end(); ++i)
		 {
			 if(ids.find(i->first) != ids.end())
			 {
				 int id = i->second->id();
				 const std::set<int> & loopIds = i->second->getLoopClosureIds();
				 //don't show children when _loopClosuresMerged is on
				 //if(!_loopClosuresMerged || (loopIds.size() == 0))
				 {
					 const std::set<int> & neighbors = i->second->getNeighbors();
					 int weight = i->second->getWeight();
					 for(std::set<int>::const_iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
					 {
						 if(id != *iter)
						 {
							 int weightNeighbor = 0;
							 const Signature * s = this->getSignature(*iter);
							 if(s)
							 {
								 weightNeighbor = s->getWeight();
							 }
							 else
							 {
								 _dbDriver->getWeight(*iter, weightNeighbor);
							 }
							 UDEBUG("Add neighbor link from %d to %d", id, *iter);
							 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\";\n",
									 id,
									 weight,
									 *iter,
									 weightNeighbor);
						 }
					 }

					 // loop closure link
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
						 UDEBUG("Add loop link from %d to %d", id, *iter);
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"L\", fontcolor=%s, fontsize=8];\n",
								 id,
								 weight,
								 *iter,
								 weightNeighbor,
								 colorG);
					 }

					 // child loop closure link
					 const std::set<int> & childIds = i->second->getChildLoopClosureIds();
					 for(std::set<int>::const_iterator iter = childIds.begin(); iter!=childIds.end(); ++iter)
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
						 UDEBUG("Add child link from %d to %d", id, *iter);
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"C\", fontcolor=%s, fontsize=8];\n",
								 id,
								 weight,
								 *iter,
								 weightNeighbor,
								 colorP);
					 }
				 }
			 }
		 }
		 fprintf(fout, "}\n");
		 fclose(fout);
		 UINFO("Graph saved to \"%s\"", fileName.c_str());
	}
}

// Only used to generate a .dot file
class GraphNode
{
public:
	GraphNode(int id, GraphNode * parent = 0) :
		_parent(parent),
		_id(id)
	{
		if(_parent)
		{
			_parent->addChild(this);
		}
	}
	virtual ~GraphNode()
	{
		//We copy the set because when a child is destroyed, it is removed from its parent.
		std::set<GraphNode*> children = _children;
		_children.clear();
		for(std::set<GraphNode*>::iterator iter=children.begin(); iter!=children.end(); ++iter)
		{
			delete *iter;
		}
		children.clear();
		if(_parent)
		{
			_parent->removeChild(this);
		}
	}
	int id() const {return _id;}
	bool isAncestor(int id) const
	{
		if(_parent)
		{
			if(_parent->id() == id)
			{
				return true;
			}
			return _parent->isAncestor(id);
		}
		return false;
	}

	void expand(std::list<std::list<int> > & paths, std::list<int> currentPath = std::list<int>()) const
	{
		currentPath.push_back(_id);
		if(_children.size() == 0)
		{
			paths.push_back(currentPath);
			return;
		}
		for(std::set<GraphNode*>::const_iterator iter=_children.begin(); iter!=_children.end(); ++iter)
		{
			(*iter)->expand(paths, currentPath);
		}
	}

private:
	void addChild(GraphNode * child)
	{
		_children.insert(child);
	}
	void removeChild(GraphNode * child)
	{
		_children.erase(child);
	}

private:
	std::set<GraphNode*> _children;
	GraphNode * _parent;
	int _id;
};

//recursive
void Memory::createGraph(GraphNode * parent, unsigned int maxDepth, const std::set<int> & endIds)
{
	if(maxDepth == 0 || !parent)
	{
		return;
	}
	std::map<int, int> neighbors = this->getNeighborsId(parent->id(), 1, -1, false);
	for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
	{
		if(!parent->isAncestor(iter->first))
		{
			GraphNode * n = new GraphNode(iter->first, parent);
			if(endIds.find(iter->first) == endIds.end())
			{
				this->createGraph(n, maxDepth-1, endIds);
			}
		}
	}
}

// Keypoint stuff
std::multimap<int, cv::KeyPoint> Memory::getWords(int signatureId) const
{
	std::multimap<int, cv::KeyPoint> words;
	if(signatureId>0)
	{
		const Signature * s = this->getSignature(signatureId);
		if(s)
		{
			const Signature * ks = dynamic_cast<const Signature*>(s);
			if(ks)
			{
				words = ks->getWords();
			}
		}
		else if(_dbDriver)
		{
			std::list<int> ids;
			ids.push_back(signatureId);
			std::list<Signature *> signatures;
			_dbDriver->loadSignatures(ids, signatures);
			if(signatures.size())
			{
				const Signature * ks = dynamic_cast<const Signature*>(signatures.front());
				if(ks)
				{
					words = ks->getWords();
				}
			}
			for(std::list<Signature *>::iterator iter = signatures.begin(); iter!=signatures.end(); ++iter)
			{
				delete *iter;
			}
		}
	}
	return words;
}

int Memory::getNi(int signatureId) const
{
	int ni = 0;
	const Signature * s = this->getSignature(signatureId);
	if(s) // Must be a SurfSignature
	{
		ni = ((Signature *)s)->getWords().size();
	}
	else
	{
		_dbDriver->getInvertedIndexNi(signatureId, ni);
	}
	return ni;
}


void Memory::copyData(const Signature * from, Signature * to)
{
	// The signatures must be KeypointSignature
	const Signature * sFrom = dynamic_cast<const Signature *>(from);
	Signature * sTo = dynamic_cast<Signature *>(to);
	UTimer timer;
	timer.start();
	if(sFrom && sTo)
	{
		this->disableWordsRef(sTo->id());
		sTo->setWords(sFrom->getWords());

		std::list<int> id;
		id.push_back(sTo->id());
		this->enableWordsRef(id);

		if(this->isRawDataKept())
		{
			if(sFrom->getImage().empty())
			{
				// try load from database
				if(sFrom->isSaved() && _dbDriver)
				{
					cv::Mat image;
					_dbDriver->getImage(sFrom->id(), image);
					if(!image.empty())
					{
						sTo->setImage(image);
					}
					UDEBUG("Loaded image data from database");
				}
			}
			else
			{
				sTo->setImage(sFrom->getImage());
			}
		}
	}
	else
	{
		ULOGGER_ERROR("Can't merge the signatures because there are not same type.");
	}
	ULOGGER_DEBUG("Merging time = %fs", timer.ticks());
}

class PreUpdateThread : public UThreadNode
{
public:
	PreUpdateThread(VWDictionary * vwp) : _vwp(vwp) {}
	virtual ~PreUpdateThread() {}
private:
	void mainLoop() {
		if(_vwp)
		{
			_vwp->update();
		}
		this->kill();
	}
	VWDictionary * _vwp;
};

Signature * Memory::createSignature(const Image & image, bool keepRawData)
{
	PreUpdateThread preUpdateThread(_vwd);

	UTimer timer;
	timer.start();
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	int id = image.id();
	if(!id)
	{
		id = this->getNextId();
	}

	int treeSize= _workingMem.size() + _stMem.size();
	int nbCommonWords = 0;
	if(treeSize > 0)
	{
		nbCommonWords = _vwd->getTotalActiveReferences() / treeSize;
	}

	if(_parallelized)
	{
		preUpdateThread.start();
	}

	if(!image.descriptors().empty())
	{
		// DESCRIPTORS
		if(image.descriptors().rows && image.descriptors().rows >= _badSignRatio * nbCommonWords)
		{
				if(image.descriptors().type() == CV_32F)
				{
						descriptors = image.descriptors();
						keypoints = image.keypoints();
				}
				else
				{
						UERROR("Descriptors must be CV_32F.");
				}
		}
	}
	else
	{
		// IMAGE RAW
		if(_keypointDetector)
		{
			keypoints = _keypointDetector->generateKeypoints(image.image());
			ULOGGER_DEBUG("time keypoints = %fs", timer.ticks());
		}

		ULOGGER_DEBUG("ratio=%f, treeSize=%d, nbCommonWords=%d", _badSignRatio, treeSize, nbCommonWords);

		if(keypoints.size() && keypoints.size() >= _badSignRatio * nbCommonWords)
		{
			descriptors = _keypointDescriptor->generateDescriptors(image.image(), keypoints);
		}
	}

	if(_parallelized)
	{
		preUpdateThread.join(); // Wait the dictionary to be updated
	}

	std::list<int> wordIds;
	if(descriptors.rows)
	{
		if(_parallelized)
		{
			ULOGGER_DEBUG("time descriptor and memory update (%d of size=%d) = %fs", descriptors.rows, descriptors.cols, timer.ticks());
		}
		else
		{
			ULOGGER_DEBUG("time descriptor (%d of size=%d) = %fs", descriptors.rows, descriptors.cols, timer.ticks());
		}

		wordIds = _vwd->addNewWords(descriptors, id);
		ULOGGER_DEBUG("time addNewWords %fs", timer.ticks());
	}
	else if(id>0)
	{
		UDEBUG("id %d is a bad signature", id);
	}

	std::multimap<int, cv::KeyPoint> words;
	if(wordIds.size() > 0)
	{
		std::vector<cv::KeyPoint>::iterator kpIter = keypoints.begin();
		for(std::list<int>::iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
		{
			if(kpIter != keypoints.end())
			{
				words.insert(std::pair<int, cv::KeyPoint >(*iter, *kpIter));
				++kpIter;
			}
			else
			{
				if(keypoints.size())
				{
					UWARN("Words (%d) and keypoints(%d) are not the same size ?!?", (int)wordIds.size(), (int)keypoints.size());
				}
				words.insert(std::pair<int, cv::KeyPoint >(*iter, cv::KeyPoint()));
			}
		}
	}

	Signature * ks;
	if(keepRawData)
	{
		ks = new Signature(id, words, image.image().clone());
	}
	else
	{
		ks = new Signature(id, words);
	}



	ULOGGER_DEBUG("time new signature (id=%d) %fs", id, timer.ticks());
	if(words.size())
	{
		ks->setEnabled(true); // All references are already activated in the dictionary at this point (see _vwd->addNewWords())
	}
	return ks;
}

void Memory::disableWordsRef(int signatureId)
{
	ULOGGER_DEBUG("id=%d", signatureId);

	Signature * ss = dynamic_cast<Signature *>(this->_getSignature(signatureId));
	if(ss && ss->isEnabled())
	{
		const std::multimap<int, cv::KeyPoint> & words = ss->getWords();
		const std::list<int> & keys = uUniqueKeys(words);
		int count = _vwd->getTotalActiveReferences();
		// First remove all references
		for(std::list<int>::const_iterator i=keys.begin(); i!=keys.end(); ++i)
		{
			_vwd->removeAllWordRef(*i, signatureId);
		}

		count -= _vwd->getTotalActiveReferences();
		ss->setEnabled(false);
		ULOGGER_DEBUG("%d words total ref removed from signature %d... (total active ref = %d)", count, ss->id(), _vwd->getTotalActiveReferences());
	}
}

void Memory::cleanUnusedWords()
{
	UINFO("");
	if(_vwd->isIncremental())
	{
		std::vector<VisualWord*> removedWords = _vwd->getUnusedWords();

		if(removedWords.size())
		{
			// remove them from the dictionary
			_vwd->removeWords(removedWords);

			for(unsigned int i=0; i<removedWords.size(); ++i)
			{
				if(_dbDriver)
				{
					_dbDriver->asyncSave(removedWords[i]);
				}
				else
				{
					delete removedWords[i];
				}
			}
		}
		UINFO("%d words removed...", removedWords.size());
	}
}

void Memory::enableWordsRef(const std::list<int> & signatureIds)
{
	ULOGGER_DEBUG("size=%d", signatureIds.size());
	UTimer timer;
	timer.start();

	std::map<int, int> refsToChange; //<oldWordId, activeWordId>

	std::set<int> oldWordIds;
	std::list<Signature *> surfSigns;
	for(std::list<int>::const_iterator i=signatureIds.begin(); i!=signatureIds.end(); ++i)
	{
		Signature * ss = dynamic_cast<Signature *>(this->_getSignature(*i));
		if(ss && !ss->isEnabled())
		{
			surfSigns.push_back(ss);
			std::list<int> uniqueKeys = uUniqueKeys(ss->getWords());

			//Find words in the signature which they are not in the current dictionary
			for(std::list<int>::const_iterator k=uniqueKeys.begin(); k!=uniqueKeys.end(); ++k)
			{
				if(_vwd->getWord(*k) == 0 && _vwd->getUnusedWord(*k) == 0)
				{
					oldWordIds.insert(oldWordIds.end(), *k);
				}
			}
		}
	}

	ULOGGER_DEBUG("oldWordIds.size()=%d, getOldIds time=%fs", oldWordIds.size(), timer.ticks());

	// the words were deleted, so try to math it with an active word
	std::list<VisualWord *> vws;
	if(oldWordIds.size() && _dbDriver)
	{
		// get the descriptors
		_dbDriver->loadWords(oldWordIds, vws);
	}
	ULOGGER_DEBUG("loading words(%d) time=%fs", oldWordIds.size(), timer.ticks());


	if(vws.size())
	{
		//Search in the dictionary
		std::vector<int> vwActiveIds = _vwd->findNN(vws, _reactivatedWordsComparedToNewWords);
		ULOGGER_DEBUG("find active ids (number=%d) time=%fs", vws.size(), timer.ticks());
		int i=0;
		for(std::list<VisualWord *>::iterator iterVws=vws.begin(); iterVws!=vws.end(); ++iterVws)
		{
			if(vwActiveIds[i] > 0)
			{
				//ULOGGER_DEBUG("Match found %d with %d", (*iterVws)->id(), vwActiveIds[i]);
				refsToChange.insert(refsToChange.end(), std::pair<int, int>((*iterVws)->id(), vwActiveIds[i]));
				if((*iterVws)->isSaved())
				{
					delete (*iterVws);
				}
				else if(_dbDriver)
				{
					_dbDriver->asyncSave(*iterVws);
				}
			}
			else
			{
				//add to dictionary
				_vwd->addWord(*iterVws); // take ownership
			}
			++i;
		}
		ULOGGER_DEBUG("Added %d to dictionary, time=%fs", vws.size()-refsToChange.size(), timer.ticks());

		//update the global references map and update the signatures reactivated
		for(std::map<int, int>::const_iterator iter=refsToChange.begin(); iter != refsToChange.end(); ++iter)
		{
			//uInsert(_wordRefsToChange, (const std::pair<int, int>)*iter); // This will be used to change references in the database
			for(std::list<Signature *>::iterator j=surfSigns.begin(); j!=surfSigns.end(); ++j)
			{
				(*j)->changeWordsRef(iter->first, iter->second);
			}
		}
		ULOGGER_DEBUG("changing ref, total=%d, time=%fs", refsToChange.size(), timer.ticks());
	}

	int count = _vwd->getTotalActiveReferences();

	// Reactivate references and signatures
	for(std::list<Signature *>::iterator j=surfSigns.begin(); j!=surfSigns.end(); ++j)
	{
		const std::vector<int> & keys = uKeys((*j)->getWords());
		// Add all references
		for(std::vector<int>::const_iterator i=keys.begin(); i!=keys.end(); ++i)
		{
			_vwd->addWordRef(*i, (*j)->id());
		}
		if(keys.size())
		{
			(*j)->setEnabled(true);
		}
	}

	count = _vwd->getTotalActiveReferences() - count;
	ULOGGER_DEBUG("%d words total ref added from %d signatures, time=%fs...", count, surfSigns.size(), timer.ticks());
}

std::set<int> Memory::reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, double & timeDbAccess)
{
	// get the signatures, if not in the working memory, they
	// will be loaded from the database in an more efficient way
	// than how it is done in the Memory

	ULOGGER_DEBUG("");
	UTimer timer;
	std::list<int> idsToLoad;
	std::map<int, int>::iterator wmIter;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(!this->getSignature(*i) && !uContains(idsToLoad, *i))
		{
			if(!maxLoaded || idsToLoad.size() < maxLoaded)
			{
				idsToLoad.push_back(*i);
				UINFO("Loading location %d from database...", *i);
			}
		}
	}

	ULOGGER_DEBUG("idsToLoad = %d", idsToLoad.size());

	std::list<Signature *> reactivatedSigns;
	if(_dbDriver)
	{
		_dbDriver->loadSignatures(idsToLoad, reactivatedSigns);
	}
	timeDbAccess = timer.getElapsedTime();
	std::list<int> idsLoaded;
	for(std::list<Signature *>::iterator i=reactivatedSigns.begin(); i!=reactivatedSigns.end(); ++i)
	{
		idsLoaded.push_back((*i)->id());
		//append to working memory
		this->addSignatureToWm(*i);
	}
	this->enableWordsRef(idsLoaded);
	ULOGGER_DEBUG("time = %fs", timer.ticks());
	return std::set<int>(idsToLoad.begin(), idsToLoad.end());
}

} // namespace rtabmap
