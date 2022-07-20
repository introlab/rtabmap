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

#include "rtabmap/core/DBDriver.h"

#include "rtabmap/core/Signature.h"
#include "rtabmap/core/VisualWord.h"
#include "rtabmap/core/DBDriverSqlite3.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"

namespace rtabmap {

DBDriver * DBDriver::create(const ParametersMap & parameters)
{
	return new DBDriverSqlite3(parameters);
}

DBDriver::DBDriver(const ParametersMap & parameters) :
	_emptyTrashesTime(0),
	_timestampUpdate(true)
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
	Parameters::parse(parameters, Parameters::kDbTargetVersion(), _targetVersion);
}

void DBDriver::closeConnection(bool save, const std::string & outputUrl)
{
	UDEBUG("isRunning=%d", this->isRunning());
	this->join(true);
	UDEBUG("");
	if(save)
	{
		this->emptyTrashes();
	}
	else
	{
		_trashesMutex.lock();
		_trashSignatures.clear();
		_trashVisualWords.clear();
		_trashesMutex.unlock();
	}
	_dbSafeAccessMutex.lock();
	this->disconnectDatabaseQuery(save, outputUrl);
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
unsigned long DBDriver::getMemoryUsed() const
{
	unsigned long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}

long DBDriver::getNodesMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getNodesMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}
long DBDriver::getLinksMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getLinksMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}
long DBDriver::getImagesMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getImagesMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}
long DBDriver::getDepthImagesMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getDepthImagesMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}
long DBDriver::getCalibrationsMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getCalibrationsMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}
long DBDriver::getGridsMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getGridsMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}
long DBDriver::getLaserScansMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getLaserScansMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}
long DBDriver::getUserDataMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getUserDataMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}
long DBDriver::getWordsMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getWordsMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}
long DBDriver::getFeaturesMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getFeaturesMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}
long DBDriver::getStatisticsMemoryUsed() const
{
	long bytes;
	_dbSafeAccessMutex.lock();
	bytes = getStatisticsMemoryUsedQuery();
	_dbSafeAccessMutex.unlock();
	return bytes;
}
int DBDriver::getLastNodesSize() const
{
	int nodes;
	_dbSafeAccessMutex.lock();
	nodes = getLastNodesSizeQuery();
	_dbSafeAccessMutex.unlock();
	return nodes;
}
int DBDriver::getLastDictionarySize() const
{
	int words;
	_dbSafeAccessMutex.lock();
	words = getLastDictionarySizeQuery();
	_dbSafeAccessMutex.unlock();
	return words;
}
int DBDriver::getTotalNodesSize() const
{
	int words;
	_dbSafeAccessMutex.lock();
	words = getTotalNodesSizeQuery();
	_dbSafeAccessMutex.unlock();
	return words;
}
int DBDriver::getTotalDictionarySize() const
{
	int words;
	_dbSafeAccessMutex.lock();
	words = getTotalDictionarySizeQuery();
	_dbSafeAccessMutex.unlock();
	return words;
}
ParametersMap DBDriver::getLastParameters() const
{
	ParametersMap parameters;
	_dbSafeAccessMutex.lock();
	parameters = getLastParametersQuery();
	_dbSafeAccessMutex.unlock();
	return parameters;
}

std::map<std::string, float> DBDriver::getStatistics(int nodeId, double & stamp, std::vector<int> * wmState) const
{
	std::map<std::string, float> statistics;
	_dbSafeAccessMutex.lock();
	statistics = getStatisticsQuery(nodeId, stamp, wmState);
	_dbSafeAccessMutex.unlock();
	return statistics;
}

std::map<int, std::pair<std::map<std::string, float>, double> > DBDriver::getAllStatistics() const
{
	std::map<int, std::pair<std::map<std::string, float>, double> > statistics;
	_dbSafeAccessMutex.lock();
	statistics = getAllStatisticsQuery();
	_dbSafeAccessMutex.unlock();
	return statistics;
}

std::map<int, std::vector<int> > DBDriver::getAllStatisticsWmStates() const
{
	std::map<int, std::vector<int> > wmStates;
	_dbSafeAccessMutex.lock();
	wmStates = getAllStatisticsWmStatesQuery();
	_dbSafeAccessMutex.unlock();
	return wmStates;
}

std::string DBDriver::getDatabaseVersion() const
{
	std::string version = "0.0.0";
	_dbSafeAccessMutex.lock();
	getDatabaseVersionQuery(version);
	_dbSafeAccessMutex.unlock();
	return version;
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

void DBDriver::saveOrUpdate(const std::vector<Signature *> & signatures)
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
			this->updateQuery(toUpdate, _timestampUpdate);
		}
		if(toSave.size())
		{
			this->saveQuery(toSave);
		}
	}
}

void DBDriver::saveOrUpdate(const std::vector<VisualWord *> & words) const
{
	ULOGGER_DEBUG("words.size=%d", (int)words.size());
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
			this->updateQuery(toUpdate, _timestampUpdate);
		}
		if(toSave.size())
		{
			this->saveQuery(toSave);
		}
	}
}

void DBDriver::addLink(const Link & link)
{
	_dbSafeAccessMutex.lock();
	this->addLinkQuery(link);
	_dbSafeAccessMutex.unlock();
}
void DBDriver::removeLink(int from, int to)
{
	this->executeNoResult(uFormat("DELETE FROM Link WHERE from_id=%d and to_id=%d", from, to).c_str());
}
void DBDriver::updateLink(const Link & link)
{
	_dbSafeAccessMutex.lock();
	this->updateLinkQuery(link);
	_dbSafeAccessMutex.unlock();
}
void DBDriver::updateOccupancyGrid(
		int nodeId,
		const cv::Mat & ground,
		const cv::Mat & obstacles,
		const cv::Mat & empty,
		float cellSize,
		const cv::Point3f & viewpoint)
{
	_dbSafeAccessMutex.lock();
	//just to make sure the occupancy grids are compressed for convenience
	SensorData data;
	data.setOccupancyGrid(ground, obstacles, empty, cellSize, viewpoint);
	this->updateOccupancyGridQuery(
			nodeId,
			data.gridGroundCellsCompressed(),
			data.gridObstacleCellsCompressed(),
			data.gridEmptyCellsCompressed(),
			cellSize,
			viewpoint);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::updateDepthImage(int nodeId, const cv::Mat & image)
{
	_dbSafeAccessMutex.lock();
	this->updateDepthImageQuery(
			nodeId,
			image);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::updateLaserScan(int nodeId, const LaserScan & scan)
{
	_dbSafeAccessMutex.lock();
	this->updateLaserScanQuery(
			nodeId,
			scan);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::load(VWDictionary * dictionary, bool lastStateOnly) const
{
	_dbSafeAccessMutex.lock();
	this->loadQuery(dictionary, lastStateOnly);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::loadLastNodes(std::list<Signature *> & signatures) const
{
	_dbSafeAccessMutex.lock();
	this->loadLastNodesQuery(signatures);
	_dbSafeAccessMutex.unlock();
}

Signature * DBDriver::loadSignature(int id, bool * loadedFromTrash)
{
	std::list<int> ids;
	ids.push_back(id);
	std::list<Signature*> signatures;
	std::set<int> loadedFromTrashSet;
	loadSignatures(ids, signatures, &loadedFromTrashSet);
	if(loadedFromTrash && loadedFromTrashSet.size())
	{
		*loadedFromTrash = true;
	}
	if(!signatures.empty())
	{
		return signatures.front();
	}
	return 0;
}
void DBDriver::loadSignatures(const std::list<int> & signIds,
		std::list<Signature *> & signatures,
		std::set<int> * loadedFromTrash)
{
	UDEBUG("");
	// look up in the trash before the database
	std::list<int> ids = signIds;
	bool valueFound = false;
	_trashesMutex.lock();
	{
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
			for(std::set<int>::iterator iter = ids.begin(); iter != ids.end();)
			{
				UASSERT(*iter>0);
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

void DBDriver::loadNodeData(Signature * signature, bool images, bool scan, bool userData, bool occupancyGrid) const
{
	std::list<Signature *> signatures;
	signatures.push_back(signature);
	this->loadNodeData(signatures, images, scan, userData, occupancyGrid);
}

void DBDriver::loadNodeData(std::list<Signature *> & signatures, bool images, bool scan, bool userData, bool occupancyGrid) const
{
	// Don't look in the trash, we assume that if we want to load
	// data of a signature, it is not in thrash! Print an error if so.
	_trashesMutex.lock();
	if(_trashSignatures.size())
	{
		for(std::list<Signature *>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
		{
			UASSERT(*iter != 0);
			UASSERT_MSG(!uContains(_trashSignatures, (*iter)->id()), uFormat("Signature %d should not be used when transferred to trash!!!!", (*iter)->id()).c_str());
		}
	}
	_trashesMutex.unlock();

	_dbSafeAccessMutex.lock();
	this->loadNodeDataQuery(signatures, images, scan, userData, occupancyGrid);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::getNodeData(
		int signatureId,
		SensorData & data,
		bool images, bool scan, bool userData, bool occupancyGrid) const
{
	bool found = false;
	// look in the trash
	_trashesMutex.lock();
	if(uContains(_trashSignatures, signatureId))
	{
		const Signature * s = _trashSignatures.at(signatureId);
		if((!s->isSaved() ||
			((!images || !s->sensorData().imageCompressed().empty()) &&
			 (!scan || !s->sensorData().laserScanCompressed().isEmpty()) &&
			 (!userData || !s->sensorData().userDataCompressed().empty()) &&
			 (!occupancyGrid || s->sensorData().gridCellSize() != 0.0f))))
		{
			data = (SensorData)s->sensorData();
			if(!images)
			{
				data.setRGBDImage(cv::Mat(), cv::Mat(), std::vector<CameraModel>());
			}
			if(!scan)
			{
				data.setLaserScan(LaserScan());
			}
			if(!userData)
			{
				data.setUserData(cv::Mat());
			}
			if(!occupancyGrid)
			{
				data.setOccupancyGrid(cv::Mat(), cv::Mat(), cv::Mat(), 0, cv::Point3f());
			}
			found = true;
		}
	}
	_trashesMutex.unlock();

	if(!found)
	{
		_dbSafeAccessMutex.lock();
		std::list<Signature *> signatures;
		Signature tmp(signatureId);
		signatures.push_back(&tmp);
		loadNodeDataQuery(signatures, images, scan, userData, occupancyGrid);
		data = signatures.front()->sensorData();
		_dbSafeAccessMutex.unlock();
	}
}

bool DBDriver::getCalibration(
		int signatureId,
		std::vector<CameraModel> & models,
		std::vector<StereoCameraModel> & stereoModels) const
{
	UDEBUG("");
	bool found = false;
	// look in the trash
	_trashesMutex.lock();
	if(uContains(_trashSignatures, signatureId))
	{
		models = _trashSignatures.at(signatureId)->sensorData().cameraModels();
		stereoModels = _trashSignatures.at(signatureId)->sensorData().stereoCameraModels();
		found = true;
	}
	_trashesMutex.unlock();

	if(!found)
	{
		_dbSafeAccessMutex.lock();
		found = this->getCalibrationQuery(signatureId, models, stereoModels);
		_dbSafeAccessMutex.unlock();
	}
	return found;
}

bool DBDriver::getLaserScanInfo(
		int signatureId,
		LaserScan & info) const
{
	UDEBUG("");
	bool found = false;
	// look in the trash
	_trashesMutex.lock();
	if(uContains(_trashSignatures, signatureId))
	{
		info = _trashSignatures.at(signatureId)->sensorData().laserScanCompressed();
		found = true;
	}
	_trashesMutex.unlock();

	if(!found)
	{
		_dbSafeAccessMutex.lock();
		found = this->getLaserScanInfoQuery(signatureId, info);
		_dbSafeAccessMutex.unlock();
	}
	return found;
}

bool DBDriver::getNodeInfo(
		int signatureId,
		Transform & pose,
		int & mapId,
		int & weight,
		std::string & label,
		double & stamp,
		Transform & groundTruthPose,
		std::vector<float> & velocity,
		GPS & gps,
		EnvSensors & sensors) const
{
	bool found = false;
	// look in the trash
	_trashesMutex.lock();
	if(uContains(_trashSignatures, signatureId))
	{
		pose = _trashSignatures.at(signatureId)->getPose().clone();
		mapId = _trashSignatures.at(signatureId)->mapId();
		weight = _trashSignatures.at(signatureId)->getWeight();
		label = std::string(_trashSignatures.at(signatureId)->getLabel());
		stamp = _trashSignatures.at(signatureId)->getStamp();
		groundTruthPose = _trashSignatures.at(signatureId)->getGroundTruthPose().clone();
		velocity = std::vector<float>(_trashSignatures.at(signatureId)->getVelocity());
		gps = GPS(_trashSignatures.at(signatureId)->sensorData().gps());
		sensors = EnvSensors(_trashSignatures.at(signatureId)->sensorData().envSensors());
		found = true;
	}
	_trashesMutex.unlock();

	if(!found)
	{
		_dbSafeAccessMutex.lock();
		found = this->getNodeInfoQuery(signatureId, pose, mapId, weight, label, stamp, groundTruthPose, velocity, gps, sensors);
		_dbSafeAccessMutex.unlock();
	}
	return found;
}

void DBDriver::loadLinks(int signatureId, std::multimap<int, Link> & links, Link::Type type) const
{
	bool found = false;
	// look in the trash
	_trashesMutex.lock();
	if(uContains(_trashSignatures, signatureId))
	{
		const Signature * s = _trashSignatures.at(signatureId);
		UASSERT(s != 0);
		for(std::map<int, Link>::const_iterator nIter = s->getLinks().begin();
				nIter!=s->getLinks().end();
				++nIter)
		{
			if(type == Link::kAllWithoutLandmarks || type == Link::kAllWithLandmarks || nIter->second.type() == type)
			{
				links.insert(*nIter);
			}
		}
		if(type == Link::kLandmark || type == Link::kAllWithLandmarks)
		{
			links.insert(s->getLandmarks().begin(), s->getLandmarks().end());
		}
		found = true;
	}
	_trashesMutex.unlock();

	if(!found)
	{
		_dbSafeAccessMutex.lock();
		this->loadLinksQuery(signatureId, links, type);
		_dbSafeAccessMutex.unlock();
	}
}

void DBDriver::getWeight(int signatureId, int & weight) const
{
	bool found = false;
	// look in the trash
	_trashesMutex.lock();
	if(uContains(_trashSignatures, signatureId))
	{
		weight = _trashSignatures.at(signatureId)->getWeight();
		found = true;
	}
	_trashesMutex.unlock();

	if(!found)
	{
		_dbSafeAccessMutex.lock();
		this->getWeightQuery(signatureId, weight);
		_dbSafeAccessMutex.unlock();
	}
}

void DBDriver::getLastNodeIds(std::set<int> & ids) const
{
	_dbSafeAccessMutex.lock();
	this->getLastNodeIdsQuery(ids);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::getAllNodeIds(std::set<int> & ids, bool ignoreChildren, bool ignoreBadSignatures, bool ignoreIntermediateNodes) const
{
	// look in the trash
	_trashesMutex.lock();
	if(_trashSignatures.size())
	{
		for(std::map<int, Signature*>::const_iterator sIter = _trashSignatures.begin(); sIter!=_trashSignatures.end(); ++sIter)
		{
			bool hasNeighbors = !ignoreChildren;
			if(ignoreChildren)
			{
				for(std::map<int, Link>::const_iterator nIter = sIter->second->getLinks().begin();
						nIter!=sIter->second->getLinks().end();
						++nIter)
				{
					if(nIter->second.type() == Link::kNeighbor ||
					   nIter->second.type() == Link::kNeighborMerged)
					{
						hasNeighbors = true;
						break;
					}
				}
			}
			if(hasNeighbors && (!ignoreIntermediateNodes || sIter->second->getWeight() != -1))
			{
				ids.insert(sIter->first);
			}
		}

		std::vector<int> keys = uKeys(_trashSignatures);

	}
	_trashesMutex.unlock();

	_dbSafeAccessMutex.lock();
	this->getAllNodeIdsQuery(ids, ignoreChildren, ignoreBadSignatures, ignoreIntermediateNodes);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::getAllLinks(std::multimap<int, Link> & links, bool ignoreNullLinks, bool withLandmarks) const
{
	_dbSafeAccessMutex.lock();
	this->getAllLinksQuery(links, ignoreNullLinks, withLandmarks);
	_dbSafeAccessMutex.unlock();

	// look in the trash
	_trashesMutex.lock();
	if(_trashSignatures.size())
	{
		for(std::map<int, Signature*>::const_iterator iter=_trashSignatures.begin(); iter!=_trashSignatures.end(); ++iter)
		{
			links.erase(iter->first);
			for(std::map<int, Link>::const_iterator jter=iter->second->getLinks().begin();
				jter!=iter->second->getLinks().end();
				++jter)
			{
				if(!ignoreNullLinks || jter->second.isValid())
				{
					links.insert(std::make_pair(iter->first, jter->second));
				}
			}
			if(withLandmarks)
			{
				for(std::map<int, Link>::const_iterator jter=iter->second->getLandmarks().begin();
					jter!=iter->second->getLandmarks().end();
					++jter)
				{
					if(!ignoreNullLinks || jter->second.isValid())
					{
						links.insert(std::make_pair(iter->first, jter->second));
					}
				}
			}
		}
	}
	_trashesMutex.unlock();
}

void DBDriver::getLastNodeId(int & id) const
{
	// look in the trash
	_trashesMutex.lock();
	if(_trashSignatures.size())
	{
		id = _trashSignatures.rbegin()->first;
	}
	_trashesMutex.unlock();

	_dbSafeAccessMutex.lock();
	this->getLastIdQuery("Node", id);
	int statisticsId = 0;
	if(uStrNumCmp(this->getDatabaseVersion(), "0.11.11") >= 0)
	{
		this->getLastIdQuery("Statistics", statisticsId);
		if(statisticsId > id)
		{
			id = statisticsId;
		}
	}
	_dbSafeAccessMutex.unlock();
}

void DBDriver::getLastMapId(int & mapId) const
{
	// look in the trash
	_trashesMutex.lock();
	if(_trashSignatures.size())
	{
		mapId = _trashSignatures.rbegin()->second->mapId();
	}
	_trashesMutex.unlock();

	_dbSafeAccessMutex.lock();
	this->getLastIdQuery("Node", mapId, "map_id");
	_dbSafeAccessMutex.unlock();
}

void DBDriver::getLastWordId(int & id) const
{
	// look in the trash
	_trashesMutex.lock();
	if(_trashVisualWords.size())
	{
		id = _trashVisualWords.rbegin()->first;
	}
	_trashesMutex.unlock();

	_dbSafeAccessMutex.lock();
	this->getLastIdQuery("Word", id);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::getInvertedIndexNi(int signatureId, int & ni) const
{
	bool found = false;
	// look in the trash
	_trashesMutex.lock();
	if(uContains(_trashSignatures, signatureId))
	{
		ni = _trashSignatures.at(signatureId)->getWords().size();
		found = true;
	}
	_trashesMutex.unlock();

	if(!found)
	{
		_dbSafeAccessMutex.lock();
		this->getInvertedIndexNiQuery(signatureId, ni);
		_dbSafeAccessMutex.unlock();
	}
}

void DBDriver::getNodesObservingLandmark(int landmarkId, std::map<int, Link> & nodes) const
{
	if(landmarkId < 0)
	{
		// look in the trash
		_trashesMutex.lock();
		for(std::map<int, Signature*>::const_iterator sIter = _trashSignatures.begin(); sIter!=_trashSignatures.end(); ++sIter)
		{
			std::map<int, Link>::const_iterator kter = sIter->second->getLandmarks().find(landmarkId);
			if(kter != sIter->second->getLandmarks().end())
			{
				nodes.insert(std::make_pair(sIter->second->id(), kter->second));
			}
		}
		_trashesMutex.unlock();

		// then look in the database
		_dbSafeAccessMutex.lock();
		this->getNodesObservingLandmarkQuery(landmarkId, nodes);
		_dbSafeAccessMutex.unlock();
	}
	else
	{
		UWARN("Can't search with an empty label!");
	}
}

void DBDriver::getNodeIdByLabel(const std::string & label, int & id) const
{
	if(!label.empty())
	{
		int idFound = 0;
		// look in the trash
		_trashesMutex.lock();
		for(std::map<int, Signature*>::const_iterator sIter = _trashSignatures.begin(); sIter!=_trashSignatures.end(); ++sIter)
		{
			if(sIter->second->getLabel().compare(label) == 0)
			{
				idFound = sIter->first;
				break;
			}
		}
		_trashesMutex.unlock();

		// then look in the database
		if(idFound == 0)
		{
			_dbSafeAccessMutex.lock();
			this->getNodeIdByLabelQuery(label, id);
			_dbSafeAccessMutex.unlock();
		}
		else
		{
			id = idFound;
		}
	}
	else
	{
		UWARN("Can't search with an empty label!");
	}
}

void DBDriver::getAllLabels(std::map<int, std::string> & labels) const
{
	// look in the trash
	_trashesMutex.lock();
	for(std::map<int, Signature*>::const_iterator sIter = _trashSignatures.begin(); sIter!=_trashSignatures.end(); ++sIter)
	{
		if(!sIter->second->getLabel().empty())
		{
			labels.insert(std::make_pair(sIter->first, sIter->second->getLabel()));
		}
	}
	_trashesMutex.unlock();

	// then look in the database
	_dbSafeAccessMutex.lock();
	this->getAllLabelsQuery(labels);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::addInfoAfterRun(
		int stMemSize,
		int lastSignAdded,
		int processMemUsed,
		int databaseMemUsed,
		int dictionarySize,
		const ParametersMap & parameters) const
{
	ULOGGER_DEBUG("");
	if(this->isConnected())
	{
		std::stringstream query;
		if(uStrNumCmp(this->getDatabaseVersion(), "0.11.8") >= 0)
		{
			std::string param = Parameters::serialize(parameters);
			if(uStrNumCmp(this->getDatabaseVersion(), "0.11.11") >= 0)
			{
				query << "INSERT INTO Info(STM_size,last_sign_added,process_mem_used,database_mem_used,dictionary_size,parameters) values("
					  << stMemSize << ","
					  << lastSignAdded << ","
					  << processMemUsed << ","
					  << databaseMemUsed << ","
					  << dictionarySize << ","
					  "\"" << param.c_str() << "\");";
			}
			else
			{
				query << "INSERT INTO Statistics(STM_size,last_sign_added,process_mem_used,database_mem_used,dictionary_size,parameters) values("
					  << stMemSize << ","
					  << lastSignAdded << ","
					  << processMemUsed << ","
					  << databaseMemUsed << ","
					  << dictionarySize << ","
					  "\"" << param.c_str() << "\");";
			}
		}
		else
		{
			query << "INSERT INTO Statistics(STM_size,last_sign_added,process_mem_used,database_mem_used,dictionary_size) values("
				  << stMemSize << ","
				  << lastSignAdded << ","
				  << processMemUsed << ","
				  << databaseMemUsed << ","
				  << dictionarySize << ");";
		}

		this->executeNoResultQuery(query.str());
	}
}

void DBDriver::addStatistics(const Statistics & statistics, bool saveWmState) const
{
	_dbSafeAccessMutex.lock();
	addStatisticsQuery(statistics, saveWmState);
	_dbSafeAccessMutex.unlock();
}

void DBDriver::savePreviewImage(const cv::Mat & image) const
{
	_dbSafeAccessMutex.lock();
	savePreviewImageQuery(image);
	_dbSafeAccessMutex.unlock();
}

cv::Mat DBDriver::loadPreviewImage() const
{
	_dbSafeAccessMutex.lock();
	cv::Mat image = loadPreviewImageQuery();
	_dbSafeAccessMutex.unlock();
	return image;
}

void DBDriver::saveOptimizedPoses(const std::map<int, Transform> & optimizedPoses, const Transform & lastlocalizationPose) const
{
	_dbSafeAccessMutex.lock();
	saveOptimizedPosesQuery(optimizedPoses, lastlocalizationPose);
	_dbSafeAccessMutex.unlock();
}
std::map<int, Transform> DBDriver::loadOptimizedPoses(Transform * lastlocalizationPose) const
{
	_dbSafeAccessMutex.lock();
	std::map<int, Transform> poses = loadOptimizedPosesQuery(lastlocalizationPose);
	_dbSafeAccessMutex.unlock();
	return poses;
}

void DBDriver::save2DMap(const cv::Mat & map, float xMin, float yMin, float cellSize) const
{
	_dbSafeAccessMutex.lock();
	save2DMapQuery(map, xMin, yMin, cellSize);
	_dbSafeAccessMutex.unlock();
}

cv::Mat DBDriver::load2DMap(float & xMin, float & yMin, float & cellSize) const
{
	_dbSafeAccessMutex.lock();
	cv::Mat map = load2DMapQuery(xMin, yMin, cellSize);
	_dbSafeAccessMutex.unlock();
	return map;
}

void DBDriver::saveOptimizedMesh(
			const cv::Mat & cloud,
			const std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > & polygons,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
			const std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > & texCoords,
#else
			const std::vector<std::vector<Eigen::Vector2f> > & texCoords,
#endif
			const cv::Mat & textures) const
{
	_dbSafeAccessMutex.lock();
	saveOptimizedMeshQuery(cloud, polygons, texCoords, textures);
	_dbSafeAccessMutex.unlock();
}

cv::Mat DBDriver::loadOptimizedMesh(
				std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > * polygons,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
				std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > * texCoords,
#else
				std::vector<std::vector<Eigen::Vector2f> > * texCoords,
#endif
				cv::Mat * textures) const
{
	_dbSafeAccessMutex.lock();
	cv::Mat cloud = loadOptimizedMeshQuery(polygons, texCoords, textures);
	_dbSafeAccessMutex.unlock();
	return cloud;
}

void DBDriver::generateGraph(
		const std::string & fileName,
		const std::set<int> & idsInput,
		const std::map<int, Signature *> & otherSignatures)
{
	if(this->isConnected())
	{
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

			 std::set<int> ids;
			 if(idsInput.size() == 0)
			 {
				 this->getAllNodeIds(ids);
				 UDEBUG("ids.size()=%d", ids.size());
				 for(std::map<int, Signature*>::const_iterator iter=otherSignatures.begin(); iter!=otherSignatures.end(); ++iter)
				 {
					 ids.insert(iter->first);
				 }
			 }
			 else
			 {
				 ids = idsInput;
			 }

			 const char * colorG = "green";
			 const char * colorP = "pink";
			 const char * colorNM = "blue";
			 UINFO("Generating map with %d locations", ids.size());
			 fprintf(fout, "digraph G {\n");
			 for(std::set<int>::iterator i=ids.begin(); i!=ids.end(); ++i)
			 {
				 if(otherSignatures.find(*i) == otherSignatures.end())
				 {
					 int id = *i;
					 std::multimap<int, Link> links;
					 this->loadLinks(id, links);
					 int weight = 0;
					 this->getWeight(id, weight);
					 for(std::multimap<int, Link>::iterator iter = links.begin(); iter!=links.end(); ++iter)
					 {
						 int weightNeighbor = 0;
						 if(otherSignatures.find(iter->first) == otherSignatures.end())
						 {
							 this->getWeight(iter->first, weightNeighbor);
						 }
						 else
						 {
							 weightNeighbor = otherSignatures.find(iter->first)->second->getWeight();
						 }
						 //UDEBUG("Add neighbor link from %d to %d", id, iter->first);
						 if(iter->second.type() == Link::kNeighbor)
						 {
							 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\"\n",
									 id,
									 weight,
									 iter->first,
									 weightNeighbor);
						 }
						 else if(iter->second.type() == Link::kNeighborMerged)
						 {
							 //merged neighbor
							 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"M\", fontcolor=%s, fontsize=8];\n",
									 id,
									 weight,
									 iter->first,
									 weightNeighbor,
									 colorNM);
						 }
						 else if(iter->first > id)
						 {
							 //loop
							 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"L\", fontcolor=%s, fontsize=8];\n",
									 id,
									 weight,
									 iter->first,
									 weightNeighbor,
									 colorG);
						 }
						 else
						 {
							 //child
							 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"C\", fontcolor=%s, fontsize=8];\n",
									 id,
									 weight,
									 iter->first,
									 weightNeighbor,
									 colorP);
						 }
					 }
				 }
			 }
			 for(std::map<int, Signature*>::const_iterator i=otherSignatures.begin(); i!=otherSignatures.end(); ++i)
			 {
				 if(ids.find(i->first) != ids.end())
				 {
					 int id = i->second->id();
					 const std::multimap<int, Link> & links = i->second->getLinks();
					 int weight = i->second->getWeight();
					 for(std::multimap<int, Link>::const_iterator iter = links.begin(); iter!=links.end(); ++iter)
					 {
						 int weightNeighbor = 0;
						 const Signature * s = uValue(otherSignatures, iter->first, (Signature*)0);
						 if(s)
						 {
							 weightNeighbor = s->getWeight();
						 }
						 else
						 {
							 this->getWeight(iter->first, weightNeighbor);
						 }
						 //UDEBUG("Add neighbor link from %d to %d", id, iter->first);
						 if(iter->second.type() == Link::kNeighbor)
						 {
							 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\"\n",
									 id,
									 weight,
									 iter->first,
									 weightNeighbor);
						 }
						 else if(iter->second.type() == Link::kNeighborMerged)
						 {
							 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"M\", fontcolor=%s, fontsize=8];\n",
									 id,
									 weight,
									 iter->first,
									 weightNeighbor,
									 colorNM);
						 }
						 else if(iter->first > id)
						 {
							 //loop
							 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"L\", fontcolor=%s, fontsize=8];\n",
									 id,
									 weight,
									 iter->first,
									 weightNeighbor,
									 colorG);
						 }
						 else if(iter->first != id)
						 {
							 //child
							 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"C\", fontcolor=%s, fontsize=8];\n",
									 id,
									 weight,
									 iter->first,
									 weightNeighbor,
									 colorP);
						 }
					 }
				 }
			 }
			 fprintf(fout, "}\n");
			 fclose(fout);
			 UINFO("Graph saved to \"%s\" (Tip: $ neato -Tpdf \"%s\" -o out.pdf)", fileName.c_str(), fileName.c_str());
		}
	}
}

} // namespace rtabmap
