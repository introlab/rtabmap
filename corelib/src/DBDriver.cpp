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

#include "rtabmap/core/DBDriver.h"

#include "Signature.h"
#include "VWDictionary.h"
#include "utilite/UConversion.h"
#include "utilite/UMath.h"
#include "utilite/ULogger.h"
#include "utilite/UTimer.h"
#include "utilite/UStl.h"

namespace rtabmap {

DBDriver::DBDriver(const ParametersMap & parameters) :
	_minSignaturesToSave(Parameters::defaultDbMinSignaturesToSave()),
	_minWordsToSave(Parameters::defaultDbMinWordsToSave()),
	_asyncWaiting(true),
	_emptyTrashesTime(0)
{
	this->parseParameters(parameters);
}

DBDriver::~DBDriver()
{
	this->kill();
	this->emptyTrashes();
}

void DBDriver::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kDbMinSignaturesToSave())) != parameters.end())
	{
		_minSignaturesToSave = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kDbMinWordsToSave())) != parameters.end())
	{
		_minWordsToSave = std::atoi((*iter).second.c_str());
	}
}

void DBDriver::closeConnection()
{
	this->kill();
	this->emptyTrashes();
	_dbSafeAccessMutex.lock();
	this->disconnectDatabaseQuery();
	_dbSafeAccessMutex.unlock();
}

bool DBDriver::openConnection(const std::string & url)
{
	_url = url;
	_dbSafeAccessMutex.lock();
	if(this->connectDatabaseQuery(url))
	{
		this->start();
		_dbSafeAccessMutex.unlock();
		return true;
	}
	_dbSafeAccessMutex.unlock();
	return false;
}

bool DBDriver::isConnected() const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = isConnectedQuery();
	_dbSafeAccessMutex.unlock();
	return r;
}

// In bytes
long DBDriver::getMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}

void DBDriver::mainLoop()
{
	UDEBUG("");
	this->emptyTrashes();
	this->kill(); // Do it only once
	UDEBUG("");
}

void DBDriver::killCleanup()
{
	UDEBUG("");
}

void DBDriver::beginTransaction() const
{
	_transactionMutex.lock();
	ULOGGER_DEBUG("");
	this->executeNoResultQuery("BEGIN TRANSACTION;");
}

void DBDriver::commit() const
{
	ULOGGER_DEBUG("");
	this->executeNoResultQuery("COMMIT;");
	_transactionMutex.unlock();
}

bool DBDriver::executeNoResult(const std::string & sql) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->executeNoResultQuery(sql);
	_dbSafeAccessMutex.unlock();
	return r;
}

void DBDriver::emptyTrashes(bool async)
{
	ULOGGER_DEBUG("");
	if(async)
	{
		ULOGGER_DEBUG("Async emptying, start the trash thread");
		this->start();
		return;
	}

	UTimer totalTime;
	totalTime.start();

	std::vector<Signature*> signatures;
	std::map<int, VisualWord*> visualWords;
	_trashesMutex.lock();
	{
		signatures = uValues(_trashSignatures);
		visualWords = _trashVisualWords;
		_trashSignatures.clear();
		_trashVisualWords.clear();

		_asyncWaiting = true;

		_dbSafeAccessMutex.lock();
	}
	_trashesMutex.unlock();

	if(signatures.size() || visualWords.size())
	{
		ULOGGER_DEBUG("trashSignatures size = %d, trashVisualWords size = %d", signatures.size(), visualWords.size());
		this->beginTransaction();
		UTimer timer;
		timer.start();
		if(signatures.size())
		{
			if(this->isConnected())
			{
				//Only one query to the database
				this->saveOrUpdate(signatures);
			}

			for(std::vector<Signature *>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
			{
				delete *iter;
			}
			signatures.clear();
		}
		ULOGGER_DEBUG("Time emptying memory signatures trash = %f...", timer.ticks());
		if(visualWords.size())
		{
			if(this->isConnected())
			{
				//Only one query to the database
				this->saveQuery(uValues(visualWords));
			}

			for(std::map<int, VisualWord *>::iterator iter=visualWords.begin(); iter!=visualWords.end(); ++iter)
			{
				delete (*iter).second;
			}
			visualWords.clear();
		}
		ULOGGER_DEBUG("Time emptying memory visualWords trash = %f...", timer.ticks());
		this->commit();
	}

	_emptyTrashesTime = totalTime.ticks();
	ULOGGER_DEBUG("Total time emptying trashes = %fs...", _emptyTrashesTime);

	_dbSafeAccessMutex.unlock();
}

void DBDriver::asyncSave(Signature * s)
{
	_trashesMutex.lock();
	{
		_trashSignatures.insert(std::pair<int, Signature*>(s->id(), s));
		if(_trashSignatures.size() > _minSignaturesToSave && this->isRunning() && _asyncWaiting)
		{
			ULOGGER_DEBUG("(Sign) Releasing addSem...");
			_asyncWaiting = false;
			this->start();
		}
	}
	_trashesMutex.unlock();
}

void DBDriver::asyncSave(VisualWord * vw)
{
	_trashesMutex.lock();
	{
		_trashVisualWords.insert(std::pair<int, VisualWord*>(vw->id(), vw));
		if(_trashVisualWords.size() > _minWordsToSave && this->isRunning() && _asyncWaiting)
		{
			ULOGGER_DEBUG("(Word) Releasing addSem...");
			_asyncWaiting = false;
			this->start();
		}
	}
	_trashesMutex.unlock();
}

bool DBDriver::getSignature(int signatureId, Signature ** s)
{
	*s = 0;
	_trashesMutex.lock();
	{
		_dbSafeAccessMutex.lock();
		_dbSafeAccessMutex.unlock();
		for(std::map<int, Signature*>::iterator i=_trashSignatures.begin(); i!=_trashSignatures.end();)
		{
			if(i->first == signatureId)
			{
				*s = i->second;
				_trashSignatures.erase(i++);
				break;
			}
			else
			{
				++i;
			}
		}
	}
	_trashesMutex.unlock();

	if(*s == 0)
	{
		bool r;
		_dbSafeAccessMutex.lock();
		r = this->loadQuery(signatureId, s);
		_dbSafeAccessMutex.unlock();
		return r;
	}

	return true;
}

bool DBDriver::getVisualWord(int wordId, VisualWord ** vw)
{
	*vw = 0;
	_trashesMutex.lock();
	{
		_dbSafeAccessMutex.lock();
		_dbSafeAccessMutex.unlock();
		for(std::map<int, VisualWord*>::iterator i=_trashVisualWords.begin(); i!=_trashVisualWords.end(); ++i)
		{
			if((*i).first == wordId)
			{
				*vw = (*i).second;
				_trashVisualWords.erase(i);
				break;
			}
		}
	}
	_trashesMutex.unlock();

	if(*vw == 0)
	{
		bool r;
		_dbSafeAccessMutex.lock();
		r = this->loadQuery(wordId, vw);
		_dbSafeAccessMutex.unlock();
		return r;
	}

	return true;
}

//Automatically begin and commit a transaction
bool DBDriver::saveOrUpdate(const std::vector<Signature *> & signatures) const
{
	ULOGGER_DEBUG("");
	std::list<KeypointSignature *> toSaveK;
	std::list<Signature *> toUpdate;
	if(this->isConnected() && signatures.size())
	{
		for(std::vector<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end();++i)
		{
			if((*i)->isSaved())
			{
				toUpdate.push_back(*i);
			}
			else if((*i)->signatureType().compare("KeypointSignature") == 0)
			{
				toSaveK.push_back((KeypointSignature *)(*i));
			}
			else
			{
				ULOGGER_ERROR("Unknown signature type ?!?");
			}
		}

		if(toUpdate.size())
		{
			this->updateQuery(toUpdate);
		}
		if(toSaveK.size())
		{
			this->saveQuery(toSaveK);
		}
	}
	return false;
}

bool DBDriver::load(VWDictionary * dictionary) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->loadQuery(dictionary);
	_dbSafeAccessMutex.unlock();
	return r;
}

bool DBDriver::loadLastSignatures(std::list<Signature *> & signatures) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->loadLastSignaturesQuery(signatures);
	_dbSafeAccessMutex.unlock();
	return r;
}

bool DBDriver::loadKeypointSignatures(const std::list<int> & signIds, std::list<Signature *> & signatures, bool onlyParents)
{
	UDEBUG("");
	// look up in the trash before the database
	std::list<int> ids = signIds;
	std::list<Signature*>::iterator sIter;
	bool valueFound = false;
	_trashesMutex.lock();
	{
		_dbSafeAccessMutex.lock();
		_dbSafeAccessMutex.unlock();
		for(std::list<int>::iterator iter = ids.begin(); iter != ids.end();)
		{
			valueFound = false;
			for(std::map<int, Signature*>::iterator sIter = _trashSignatures.begin(); sIter!=_trashSignatures.end();)
			{
				if(sIter->first == *iter)
				{
					if((onlyParents && sIter->second->getLoopClosureId() == 0) || !onlyParents)
					{
						signatures.push_back(sIter->second);
						_trashSignatures.erase(sIter++);
					}
					else
					{
						++sIter;
					}
					valueFound = true;
					break;
				}
				else
				{
					++sIter;
				}
			}
			if(valueFound)
			{
				iter = ids.erase(iter);
			}
			else
			{
				++iter;
			}
		}
	}
	_trashesMutex.unlock();
	UDEBUG("");
	if(ids.size())
	{
		bool r;
		_dbSafeAccessMutex.lock();
		r = this->loadKeypointSignaturesQuery(ids, signatures, onlyParents);
		_dbSafeAccessMutex.unlock();
		return r;
	}
	else if(signatures.size())
	{
		return true;
	}
	return false;
}

bool DBDriver::loadWords(const std::list<int> & wordIds, std::list<VisualWord *> & vws)
{
	if(!wordIds.size())
	{
		return false;
	}
	// look up in the trash before the database
	std::list<int> ids = wordIds;
	std::map<int, VisualWord*>::iterator wIter;
	_trashesMutex.lock();
	{
		_dbSafeAccessMutex.lock();
		_dbSafeAccessMutex.unlock();
		for(std::list<int>::iterator iter = ids.begin(); iter != ids.end();)
		{
			wIter = _trashVisualWords.find(*iter);
			if(wIter != _trashVisualWords.end())
			{
				//UDEBUG("put back word %d from trash", *iter);
				vws.push_back(wIter->second);
				_trashVisualWords.erase(wIter);
				iter = ids.erase(iter);
			}
			else
			{
				++iter;
			}
		}
	}
	_trashesMutex.unlock();
	if(ids.size())
	{
		bool r;
		_dbSafeAccessMutex.lock();
		r = this->loadWordsQuery(ids, vws);
		_dbSafeAccessMutex.unlock();
		return r;
	}
	else if(vws.size())
	{
		return true;
	}
	return false;
}

// <oldWordId, activeWordId>
bool DBDriver::changeWordsRef(const std::map<int, int> & refsToChange)
{
	//Change references in the trash
	KeypointSignature * s = 0;
	UTimer timer;
	_trashesMutex.lock();
	{
		_dbSafeAccessMutex.lock();
		_dbSafeAccessMutex.unlock();
		timer.start();
		for(std::map<int, Signature *>::iterator iter = _trashSignatures.begin(); iter!=_trashSignatures.end(); ++iter)
		{
			s = dynamic_cast<KeypointSignature*>(iter->second);
			if(s)
			{
				for(std::map<int, int>::const_iterator jter = refsToChange.begin(); jter!=refsToChange.end(); ++jter)
				{
					s->changeWordsRef((*jter).first, (*jter).second);
				}
			}
		}
		ULOGGER_DEBUG("Trash changing words references time=%fs", timer.ticks());
	}
	_trashesMutex.unlock();

	bool r;
	_dbSafeAccessMutex.lock();
	r = this->changeWordsRefQuery(refsToChange);
	_dbSafeAccessMutex.unlock();
	return r;
}

bool DBDriver::deleteWords(const std::vector<int> & ids)
{
	//Delete words in the trash
	std::map<int, VisualWord*>::iterator iter;
	_trashesMutex.lock();
	{
		_dbSafeAccessMutex.lock();
		_dbSafeAccessMutex.unlock();
		for(unsigned int i=0; i<ids.size(); ++i)
		{
			iter = _trashVisualWords.find(ids[i]);
			if(iter != _trashVisualWords.end())
			{
				_trashVisualWords.erase(iter);
				delete (*iter).second;
			}
		}
	}
	_trashesMutex.unlock();

	bool r;
	_dbSafeAccessMutex.lock();
	r = this->deleteWordsQuery(ids);
	_dbSafeAccessMutex.unlock();
	return r;
}

bool DBDriver::deleteAllVisualWords() const
{
	ULOGGER_DEBUG("");
	if(this->isConnected())
	{
		std::string query;
		query += "DELETE FROM VisualWord;";

		_dbSafeAccessMutex.lock();
		bool r = this->executeNoResultQuery(query);
		_dbSafeAccessMutex.unlock();
		return r;
	}
	return false;
}

bool DBDriver::deleteAllObsoleteSSVWLinks() const
{
	ULOGGER_DEBUG("");
	if(this->isConnected())
	{
		std::string query;
		query += "DELETE FROM Map_SS_VW WHERE NOT EXISTS (SELECT id FROM VisualWord WHERE id = Map_SS_VW.visualWordId);";

		_dbSafeAccessMutex.lock();
		bool r = this->executeNoResultQuery(query);
		_dbSafeAccessMutex.unlock();
		return r;
	}
	return false;
}

bool DBDriver::deleteUnreferencedWords() const
{
	ULOGGER_DEBUG("");
	if(this->isConnected())
	{
		std::string query = "DELETE FROM visualword WHERE id NOT IN (SELECT visualWordid FROM map_ss_vw);";
		_dbSafeAccessMutex.lock();
		bool r = this->executeNoResultQuery(query);
		_dbSafeAccessMutex.unlock();
		return r;
	}
	return false;
}

bool DBDriver::addNeighbor(int id, int newNeighbor, int oldNeighbor)
{
	bool r = false;
	Signature * s = 0;
	_trashesMutex.lock();
	s = uValue(_trashSignatures, id, s);
	if(s)
	{
		const NeighborsMap & neighbors = s->getNeighbors();
		std::list<std::vector<float> > actions = uValue(s->getNeighbors(), oldNeighbor, std::list<std::vector<float> >());
		s->addNeighbor(newNeighbor, actions);
		r = true;
	}
	_trashesMutex.unlock();

	if(!r)
	{
		_dbSafeAccessMutex.lock();
		r = this->addNeighborQuery(id, newNeighbor, oldNeighbor);
		_dbSafeAccessMutex.unlock();
	}

	return r;
}

bool DBDriver::removeNeighbor(int id, int neighbor)
{
	bool r = false;
	Signature * s = 0;
	_trashesMutex.lock();
	s = uValue(_trashSignatures, id, s);
	if(s)
	{
		s->removeNeighbor(neighbor);
		r = true;
	}
	_trashesMutex.unlock();

	if(!r)
	{
		r = executeNoResult("DELETE FROM Neighbor WHERE sid=" + uNumber2str(id) + " AND nid=" + uNumber2str(neighbor));
	}

	return r;
}

//TODO Check also in the trash ?
bool DBDriver::getImage(int id, IplImage ** img) const
{
	CvMat * compressed = 0;
	_dbSafeAccessMutex.lock();
	bool result = this->getImageCompressedQuery(id, &compressed);
	if(compressed)
	{
		(*img) = cvDecodeImage(compressed, CV_LOAD_IMAGE_ANYCOLOR);
		cvReleaseMat(&compressed);
	}
	_dbSafeAccessMutex.unlock();
	return result;
}

//TODO Check also in the trash ?
bool DBDriver::getNeighborIds(int signatureId, std::set<int> & neighbors) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->getNeighborIdsQuery(signatureId, neighbors);
	_dbSafeAccessMutex.unlock();
	return r;
}

//TODO Check also in the trash ?
bool DBDriver::loadNeighbors(int signatureId, std::map<int, std::list<std::vector<float> > > & neighbors) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->loadNeighborsQuery(signatureId, neighbors);
	_dbSafeAccessMutex.unlock();
	return r;
}

//TODO Check also in the trash ?
bool DBDriver::getWeight(int signatureId, int & weight) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->getWeightQuery(signatureId, weight);
	_dbSafeAccessMutex.unlock();
	return r;
}

//TODO Check also in the trash ?
bool DBDriver::getLoopClosureId(int signatureId, int & loopId) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->getLoopClosureIdQuery(signatureId, loopId);
	_dbSafeAccessMutex.unlock();
	return r;
}

//TODO Check also in the trash ?
bool DBDriver::getImageCompressed(int id, CvMat ** compressed) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->getImageCompressedQuery(id, compressed);
	_dbSafeAccessMutex.unlock();
	return r;
}

//TODO Check also in the trash ?
bool DBDriver::getAllSignatureIds(std::set<int> & ids) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->getAllSignatureIdsQuery(ids);
	_dbSafeAccessMutex.unlock();
	return r;
}

//TODO Check also in the trash ?
bool DBDriver::getLastSignatureId(int & id) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->getLastSignatureIdQuery(id);
	_dbSafeAccessMutex.unlock();
	return r;
}

//TODO Check also in the trash ?
bool DBDriver::getLastVisualWordId(int & id) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->getLastVisualWordIdQuery(id);
	_dbSafeAccessMutex.unlock();
	return r;
}

//TODO Check also in the trash ?
bool DBDriver::getSurfNi(int signatureId, int & ni) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->getSurfNiQuery(signatureId, ni);
	_dbSafeAccessMutex.unlock();
	return r;
}

//TODO Check also in the trash ?
bool DBDriver::getChildrenIds(int signatureId, std::list<int> & ids) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->getChildrenIdsQuery(signatureId, ids);
	_dbSafeAccessMutex.unlock();
	return r;
}

bool DBDriver::getHighestWeightedSignatures(unsigned int count, std::multimap<int, int> & ids) const
{
	bool r;
	_dbSafeAccessMutex.lock();
	r = this->getHighestWeightedSignaturesQuery(count, ids);
	_dbSafeAccessMutex.unlock();
	return r;
}

bool DBDriver::addStatisticsAfterRun(int stMemSize, int lastSignAdded, int processMemUsed, int databaseMemUsed) const
{
	ULOGGER_DEBUG("");
	if(this->isConnected())
	{
		std::stringstream query;
		query << "INSERT INTO StatisticsAfterRun(stMemSize,lastSignAdded,processMemUsed,databaseMemUsed) values("
			  << stMemSize << ","
		      << lastSignAdded << ","
		      << processMemUsed << ","
			  << databaseMemUsed << ");";

		bool r = this->executeNoResultQuery(query.str());
		return r;
	}
	return false;
}

bool DBDriver::addStatisticsAfterRunSurf(int dictionarySize) const
{
	ULOGGER_DEBUG("");
	if(this->isConnected())
	{
		std::stringstream query;
		query << "INSERT INTO StatisticsAfterRunSurf(dictionarySize) values(" << dictionarySize << ");";

		bool r = this->executeNoResultQuery(query.str());
		return r;
	}
	return false;
}

} // namespace rtabmap
