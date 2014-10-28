/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/DBDriver.h"

#include "rtabmap/core/Signature.h"
#include "VisualWord.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"

namespace rtabmap {

DBDriver::DBDriver(const ParametersMap & parameters) :
	_emptyTrashesTime(0)
{
	this->parseParameters(parameters);
}

DBDriver::~DBDriver()
{
	join(true);
	this->emptyTrashes();
}

void DBDriver::parseParameters(const ParametersMap & parameters)
{
}

void DBDriver::closeConnection()
{
	UDEBUG("isRunning=%d", this->isRunning());
	this->join(true);
	UDEBUG("");
	this->emptyTrashes();
	_dbSafeAccessMutex.lock();
	this->disconnectDatabaseQuery();
	_dbSafeAccessMutex.unlock();
	UDEBUG("");
}

bool DBDriver::openConnection(const std::string & url, bool overwritten)
{
	UDEBUG("");
	_url = url;
	_dbSafeAccessMutex.lock();
	if(this->connectDatabaseQuery(url, overwritten))
	{
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
	this->emptyTrashes();
	this->kill(); // Do it only once
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

void DBDriver::executeNoResult(const std::string & sql) const
{
	_dbSafeAccessMutex.lock();
	this->executeNoResultQuery(sql);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::emptyTrashes(bool async)
{
	if(async)
	{
		ULOGGER_DEBUG("Async emptying, start the trash thread");
		this->start();
		return;
	}

	UTimer totalTime;
	totalTime.start();

	std::map<int, Signature*> signatures;
	std::map<int, VisualWord*> visualWords;
	_trashesMutex.lock();
	{
		ULOGGER_DEBUG("signatures=%d, visualWords=%d", _trashSignatures.size(), _trashVisualWords.size());
		signatures = _trashSignatures;
		visualWords = _trashVisualWords;
		_trashSignatures.clear();
		_trashVisualWords.clear();

		_dbSafeAccessMutex.lock();
	}
	_trashesMutex.unlock();

	if(signatures.size() || visualWords.size())
	{
		this->beginTransaction();
		UTimer timer;
		timer.start();
		if(signatures.size())
		{
			if(this->isConnected())
			{
				//Only one query to the database
				this->saveOrUpdate(uValues(signatures));
			}

			for(std::map<int, Signature *>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
			{
				delete iter->second;
			}
			signatures.clear();
			ULOGGER_DEBUG("Time emptying memory signatures trash = %f...", timer.ticks());
		}
		if(visualWords.size())
		{
			if(this->isConnected())
			{
				//Only one query to the database
				this->saveOrUpdate(uValues(visualWords));
			}

			for(std::map<int, VisualWord *>::iterator iter=visualWords.begin(); iter!=visualWords.end(); ++iter)
			{
				delete (*iter).second;
			}
			visualWords.clear();
			ULOGGER_DEBUG("Time emptying memory visualWords trash = %f...", timer.ticks());
		}

		this->commit();
	}

	_emptyTrashesTime = totalTime.ticks();
	ULOGGER_DEBUG("Total time emptying trashes = %fs...", _emptyTrashesTime);

	_dbSafeAccessMutex.unlock();
}

void DBDriver::asyncSave(Signature * s)
{
	if(s)
	{
		UDEBUG("s=%d", s->id());
		_trashesMutex.lock();
		{
			_trashSignatures.insert(std::pair<int, Signature*>(s->id(), s));
		}
		_trashesMutex.unlock();
	}
}

void DBDriver::asyncSave(VisualWord * vw)
{
	if(vw)
	{
		_trashesMutex.lock();
		{
			_trashVisualWords.insert(std::pair<int, VisualWord*>(vw->id(), vw));
		}
		_trashesMutex.unlock();
	}
}

void DBDriver::saveOrUpdate(const std::vector<Signature *> & signatures) const
{
	ULOGGER_DEBUG("");
	std::list<Signature *> toSave;
	std::list<Signature *> toUpdate;
	if(this->isConnected() && signatures.size())
	{
		for(std::vector<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end();++i)
		{
			if((*i)->isSaved())
			{
				toUpdate.push_back(*i);
			}
			else
			{
				toSave.push_back(*i);
			}
		}

		if(toUpdate.size())
		{
			this->updateQuery(toUpdate);
		}
		if(toSave.size())
		{
			this->saveQuery(toSave);
		}
	}
}

void DBDriver::saveOrUpdate(const std::vector<VisualWord *> & words) const
{
	ULOGGER_DEBUG("");
	std::list<VisualWord *> toSave;
	std::list<VisualWord *> toUpdate;
	if(this->isConnected() && words.size())
	{
		for(std::vector<VisualWord *>::const_iterator i=words.begin(); i!=words.end();++i)
		{
			if((*i)->isSaved())
			{
				toUpdate.push_back(*i);
			}
			else
			{
				toSave.push_back(*i);
			}
		}

		if(toUpdate.size())
		{
			this->updateQuery(toUpdate);
		}
		if(toSave.size())
		{
			this->saveQuery(toSave);
		}
	}
}

void DBDriver::load(VWDictionary * dictionary) const
{
	_dbSafeAccessMutex.lock();
	this->loadQuery(dictionary);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::loadLastNodes(std::list<Signature *> & signatures) const
{
	_dbSafeAccessMutex.lock();
	this->loadLastNodesQuery(signatures);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::loadSignatures(const std::list<int> & signIds,
		std::list<Signature *> & signatures,
		std::set<int> * loadedFromTrash)
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
					signatures.push_back(sIter->second);
					_trashSignatures.erase(sIter++);

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
				if(loadedFromTrash)
				{
					loadedFromTrash->insert(*iter);
				}
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
		_dbSafeAccessMutex.lock();
		this->loadSignaturesQuery(ids, signatures);
		_dbSafeAccessMutex.unlock();
	}
}

void DBDriver::loadWords(const std::set<int> & wordIds, std::list<VisualWord *> & vws)
{
	// look up in the trash before the database
	std::set<int> ids = wordIds;
	std::map<int, VisualWord*>::iterator wIter;
	std::list<VisualWord *> puttedBack;
	_trashesMutex.lock();
	{
		if(_trashVisualWords.size())
		{
			_dbSafeAccessMutex.lock();
			_dbSafeAccessMutex.unlock();
			for(std::set<int>::iterator iter = ids.begin(); iter != ids.end();)
			{
				wIter = _trashVisualWords.find(*iter);
				if(wIter != _trashVisualWords.end())
				{
					UDEBUG("put back word %d from trash", *iter);
					puttedBack.push_back(wIter->second);
					_trashVisualWords.erase(wIter);
					ids.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
		}
	}
	_trashesMutex.unlock();
	if(ids.size())
	{
		_dbSafeAccessMutex.lock();
		this->loadWordsQuery(ids, vws);
		_dbSafeAccessMutex.unlock();
		uAppend(vws, puttedBack);
	}
	else if(puttedBack.size())
	{
		uAppend(vws, puttedBack);
	}
}

//TODO Check also in the trash ?
void DBDriver::loadNodeData(std::list<Signature *> & signatures, bool loadMetricData) const
{
	_dbSafeAccessMutex.lock();
	this->loadNodeDataQuery(signatures, loadMetricData);
	_dbSafeAccessMutex.unlock();
}

//TODO Check also in the trash ?
void DBDriver::getNodeData(
		int signatureId,
		cv::Mat & imageCompressed,
		cv::Mat & depthCompressed,
		cv::Mat & depth2dCompressed,
		float & fx,
		float & fy,
		float & cx,
		float & cy,
		Transform & localTransform) const
{
	_dbSafeAccessMutex.lock();
	this->getNodeDataQuery(signatureId, imageCompressed, depthCompressed, depth2dCompressed, fx, fy, cx, cy, localTransform);
	_dbSafeAccessMutex.unlock();
}

//TODO Check also in the trash ?
void DBDriver::getNodeData(int signatureId, cv::Mat & imageCompressed) const
{
	_dbSafeAccessMutex.lock();
	this->getNodeDataQuery(signatureId, imageCompressed);
	_dbSafeAccessMutex.unlock();
}

//TODO Check also in the trash ?
void DBDriver::getPose(int signatureId, Transform & pose, int & mapId) const
{
	_dbSafeAccessMutex.lock();
	this->getPoseQuery(signatureId, pose, mapId);
	_dbSafeAccessMutex.unlock();
}

//TODO Check also in the trash ?
void DBDriver::loadNeighbors(int signatureId, std::map<int, Transform> & neighbors) const
{
	_dbSafeAccessMutex.lock();
	this->loadNeighborsQuery(signatureId, neighbors);
	_dbSafeAccessMutex.unlock();
}

//TODO Check also in the trash ?
void DBDriver::getWeight(int signatureId, int & weight) const
{
	_dbSafeAccessMutex.lock();
	this->getWeightQuery(signatureId, weight);
	_dbSafeAccessMutex.unlock();
}

//TODO Check also in the trash ?
void DBDriver::loadLoopClosures(int signatureId, std::map<int, Transform> & loopIds, std::map<int, Transform> & childIds) const
{
	_dbSafeAccessMutex.lock();
	this->loadLoopClosuresQuery(signatureId, loopIds, childIds);
	_dbSafeAccessMutex.unlock();
}

//TODO Check also in the trash ?
void DBDriver::getAllNodeIds(std::set<int> & ids, bool ignoreChildren) const
{
	_dbSafeAccessMutex.lock();
	this->getAllNodeIdsQuery(ids, ignoreChildren);
	_dbSafeAccessMutex.unlock();
}

//TODO Check also in the trash ?
void DBDriver::getLastNodeId(int & id) const
{
	_dbSafeAccessMutex.lock();
	this->getLastIdQuery("Node", id);
	_dbSafeAccessMutex.unlock();
}

//TODO Check also in the trash ?
void DBDriver::getLastWordId(int & id) const
{
	_dbSafeAccessMutex.lock();
	this->getLastIdQuery("Word", id);
	_dbSafeAccessMutex.unlock();
}

//TODO Check also in the trash ?
void DBDriver::getInvertedIndexNi(int signatureId, int & ni) const
{
	_dbSafeAccessMutex.lock();
	this->getInvertedIndexNiQuery(signatureId, ni);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::addStatisticsAfterRun(int stMemSize, int lastSignAdded, int processMemUsed, int databaseMemUsed, int dictionarySize) const
{
	ULOGGER_DEBUG("");
	if(this->isConnected())
	{
		std::stringstream query;
		query << "INSERT INTO Statistics(STM_size,last_sign_added,process_mem_used,database_mem_used,dictionary_size) values("
			  << stMemSize << ","
		      << lastSignAdded << ","
		      << processMemUsed << ","
		      << databaseMemUsed << ","
			  << dictionarySize << ");";

		this->executeNoResultQuery(query.str());
	}
}

} // namespace rtabmap
