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

#ifndef DBDRIVER_H_
#define DBDRIVER_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <string>
#include <list>
#include <map>
#include <set>
#include <opencv2/core/core.hpp>
#include "utilite/UMutex.h"
#include "utilite/UThreadNode.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/Signature.h"

namespace rtabmap {

class KeypointSignature;
class SMSignature;
class VWDictionary;
class VisualWord;

// Todo This class needs a refactoring, the _dbSafeAccessMutex problem when the trash is emptying (transaction)
// "Of course, it has always been the case and probably always will be
//that you cannot use the same sqlite3 connection in two or more
//threads at the same time.  You can use different sqlite3 connections
//at the same time in different threads, or you can move the same
//sqlite3 connection across threads (subject to the constraints above)
//but never, never try to use the same connection simultaneously in
//two or more threads."
//
class RTABMAP_EXP DBDriver : public UThreadNode
{
public:
	virtual ~DBDriver();

	virtual std::string getDriverName() const = 0;
	virtual void parseParameters(const ParametersMap & parameters);
	const std::string & getUrl() const {return _url;}

	void beginTransaction() const;
	void commit() const;

	void asyncSave(Signature * s);
	void asyncSave(VisualWord * s);
	void emptyTrashes(bool async = false);
	double getEmptyTrashesTime() const {return _emptyTrashesTime;}
	bool isImagesCompressed() const {return _imagesCompressed;}

public:
	bool addStatisticsAfterRun(int stMemSize, int lastSignAdded, int processMemUsed, int databaseMemUsed) const;
	bool addStatisticsAfterRunSurf(int dictionarySize) const;

	bool deleteAllVisualWords() const;
	bool deleteAllObsoleteSSVWLinks() const;
	bool deleteUnreferencedWords() const;

public:
	// Mutex-protected methods of abstract versions below
	bool getSignature(int signatureId, Signature ** s);
	bool getVisualWord(int wordId, VisualWord ** vw);

	bool openConnection(const std::string & url, bool overwritten = false);
	void closeConnection();
	bool isConnected() const;
	long getMemoryUsed() const; // In bytes

	bool executeNoResult(const std::string & sql) const;

	// Update
	bool changeWordsRef(const std::map<int, int> & refsToChange); // <oldWordId, activeWordId>
	bool deleteWords(const std::vector<int> & ids);

	// Load objects
	bool load(VWDictionary * dictionary) const;
	bool loadLastSignatures(std::list<Signature *> & signatures) const;
	bool loadKeypointSignatures(const std::list<int> & ids, std::list<Signature *> & signatures);
	bool loadSMSignatures(const std::list<int> & ids, std::list<Signature *> & signatures);
	bool loadWords(const std::list<int> & wordIds, std::list<VisualWord *> & vws);

	// Specific queries...
	bool getImage(int id, IplImage ** img) const;
	bool getNeighborIds(int signatureId, std::list<int> & neighbors, bool onlyWithActions = false) const;
	bool loadNeighbors(int signatureId, NeighborsMultiMap & neighbors) const;
	bool getWeight(int signatureId, int & weight) const;
	bool getLoopClosureIds(int signatureId, std::set<int> & loopIds, std::set<int> & childIds) const;
	bool getAllSignatureIds(std::set<int> & ids) const;
	bool getLastSignatureId(int & id) const;
	bool getLastVisualWordId(int & id) const;
	bool getSurfNi(int signatureId, int & ni) const;
	bool getHighestWeightedSignatures(unsigned int count, std::multimap<int, int> & ids) const;

protected:
	DBDriver(const ParametersMap & parameters = ParametersMap());

private:
	virtual bool connectDatabaseQuery(const std::string & url, bool overwritten = false) = 0;
	virtual void disconnectDatabaseQuery() = 0;
	virtual bool isConnectedQuery() const = 0;
	virtual long getMemoryUsedQuery() const = 0; // In bytes

	virtual bool executeNoResultQuery(const std::string & sql) const = 0;

	virtual bool changeWordsRefQuery(const std::map<int, int> & refsToChange) const = 0; // <oldWordId, activeWordId>
	virtual bool deleteWordsQuery(const std::vector<int> & ids) const = 0;
	virtual bool getNeighborIdsQuery(int signatureId, std::list<int> & neighbors, bool onlyWithActions = false) const = 0;
	virtual bool getWeightQuery(int signatureId, int & weight) const = 0;
	virtual bool getLoopClosureIdsQuery(int signatureId, std::set<int> & loopIds, std::set<int> & childIds) const = 0;

	virtual bool saveQuery(const std::vector<VisualWord *> & visualWords) const = 0;
	virtual bool updateQuery(const std::list<Signature *> & signatures) const = 0;
	virtual bool saveQuery(const std::list<Signature *> & signatures) const = 0;

	// Load objects
	virtual bool loadQuery(VWDictionary * dictionary) const = 0;
	virtual bool loadLastSignaturesQuery(std::list<Signature *> & signatures) const = 0;
	virtual bool loadQuery(int signatureId, Signature ** s) const = 0;
	virtual bool loadQuery(int wordId, VisualWord ** vw) const = 0;
	virtual bool loadQuery(int signatureId, KeypointSignature * ss) const = 0;
	virtual bool loadQuery(int signatureId, SMSignature * ss) const = 0;
	virtual bool loadKeypointSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures) const = 0;
	virtual bool loadSMSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures) const = 0;
	virtual bool loadWordsQuery(const std::list<int> & wordIds, std::list<VisualWord *> & vws) const = 0;
	virtual bool loadNeighborsQuery(int signatureId, NeighborsMultiMap & neighbors) const = 0;

	virtual bool getImageQuery(int id, IplImage ** image) const = 0;
	virtual bool getAllSignatureIdsQuery(std::set<int> & ids) const = 0;
	virtual bool getLastSignatureIdQuery(int & id) const = 0;
	virtual bool getLastVisualWordIdQuery(int & id) const = 0;
	virtual bool getSurfNiQuery(int signatureId, int & ni) const = 0;
	virtual bool getHighestWeightedSignaturesQuery(unsigned int count, std::multimap<int,int> & signatures) const = 0;

private:
	//non-abstract methods
	bool saveOrUpdate(const std::vector<Signature *> & signatures) const;

	//thread stuff
	virtual void mainLoop();
	virtual void killCleanup();

private:
	UMutex _transactionMutex;
	std::map<int, Signature *> _trashSignatures;//<id, Signature*>
	std::map<int, VisualWord *> _trashVisualWords; //<id, VisualWord*>
	UMutex _trashesMutex;
	UMutex _dbSafeAccessMutex;
	USemaphore _addSem;
	unsigned int _minSignaturesToSave;
	unsigned int _minWordsToSave;
	bool _imagesCompressed;
	bool _asyncWaiting;
	double _emptyTrashesTime;
	std::string _url;
};

}

#endif /* DBDRIVER_H_ */
