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
#include "rtabmap/utilite/UMutex.h"
#include "rtabmap/utilite/UThreadNode.h"
#include "rtabmap/core/Parameters.h"

#include <rtabmap/core/Transform.h>

namespace rtabmap {

class Signature;
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

	virtual void parseParameters(const ParametersMap & parameters);
	const std::string & getUrl() const {return _url;}

	void beginTransaction() const;
	void commit() const;

	void asyncSave(Signature * s); //ownership transferred
	void asyncSave(VisualWord * vw); //ownership transferred
	void emptyTrashes(bool async = false);
	double getEmptyTrashesTime() const {return _emptyTrashesTime;}

public:
	void addStatisticsAfterRun(int stMemSize, int lastSignAdded, int processMemUsed, int databaseMemUsed, int dictionarySize) const;

public:
	// Mutex-protected methods of abstract versions below
	bool getSignature(int signatureId, Signature ** s);
	bool getVisualWord(int wordId, VisualWord ** vw);

	bool openConnection(const std::string & url, bool overwritten = false);
	void closeConnection();
	bool isConnected() const;
	long getMemoryUsed() const; // In bytes

	void executeNoResult(const std::string & sql) const;

	// Load objects
	void load(VWDictionary * dictionary) const;
	void loadLastNodes(std::list<Signature *> & signatures) const;
	void loadSignatures(const std::list<int> & ids, std::list<Signature *> & signatures);
	void loadWords(const std::set<int> & wordIds, std::list<VisualWord *> & vws);

	// Specific queries...
	void loadNodeData(std::list<Signature *> & signatures, bool loadMetricData) const;
	void getNodeData(int signatureId, std::vector<unsigned char> & image, std::vector<unsigned char> & depth, std::vector<unsigned char> & depth2d, float & depthConstant, Transform & localTransform) const;
	void getNodeData(int signatureId, std::vector<unsigned char> & image) const;
	void getPose(int signatureId, Transform & pose, int & mapId) const;
	void loadNeighbors(int signatureId, std::map<int, Transform> & neighbors) const;
	void loadLoopClosures(int signatureId, std::map<int, Transform> & loopIds, std::map<int, Transform> & childIds) const;
	void getWeight(int signatureId, int & weight) const;
	void getAllNodeIds(std::set<int> & ids) const;
	void getLastNodeId(int & id) const;
	void getLastWordId(int & id) const;
	void getInvertedIndexNi(int signatureId, int & ni) const;

protected:
	DBDriver(const ParametersMap & parameters = ParametersMap());

private:
	virtual bool connectDatabaseQuery(const std::string & url, bool overwritten = false) = 0;
	virtual void disconnectDatabaseQuery() = 0;
	virtual bool isConnectedQuery() const = 0;
	virtual long getMemoryUsedQuery() const = 0; // In bytes

	virtual void executeNoResultQuery(const std::string & sql) const = 0;

	virtual void getWeightQuery(int signatureId, int & weight) const = 0;

	virtual void saveQuery(const std::list<Signature *> & signatures) const = 0;
	virtual void saveQuery(const std::list<VisualWord *> & words) const = 0;
	virtual void updateQuery(const std::list<Signature *> & signatures) const = 0;
	virtual void updateQuery(const std::list<VisualWord *> & words) const = 0;


	// Load objects
	virtual void loadQuery(VWDictionary * dictionary) const = 0;
	virtual void loadLastNodesQuery(std::list<Signature *> & signatures) const = 0;
	virtual void loadSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures) const = 0;
	virtual void loadWordsQuery(const std::set<int> & wordIds, std::list<VisualWord *> & vws) const = 0;
	virtual void loadNeighborsQuery(int signatureId, std::map<int, Transform> & neighbors) const = 0;
	virtual void loadLoopClosuresQuery(int signatureId, std::map<int, Transform> & loopIds, std::map<int, Transform> & childIds) const = 0;

	virtual void loadNodeDataQuery(std::list<Signature *> & signatures, bool loadMetricData) const = 0;
	virtual void getNodeDataQuery(int signatureId, std::vector<unsigned char> & image, std::vector<unsigned char> & depth, std::vector<unsigned char> & depth2d, float & depthConstant, Transform & localTransform) const = 0;
	virtual void getNodeDataQuery(int signatureId, std::vector<unsigned char> & image) const = 0;
	virtual void getPoseQuery(int signatureId, Transform & pose, int & mapId) const = 0;
	virtual void getAllNodeIdsQuery(std::set<int> & ids) const = 0;
	virtual void getLastIdQuery(const std::string & tableName, int & id) const = 0;
	virtual void getInvertedIndexNiQuery(int signatureId, int & ni) const = 0;

private:
	//non-abstract methods
	void saveOrUpdate(const std::vector<Signature *> & signatures) const;
	void saveOrUpdate(const std::vector<VisualWord *> & words) const;

	//thread stuff
	virtual void mainLoop();

private:
	UMutex _transactionMutex;
	std::map<int, Signature *> _trashSignatures;//<id, Signature*>
	std::map<int, VisualWord *> _trashVisualWords; //<id, VisualWord*>
	UMutex _trashesMutex;
	UMutex _dbSafeAccessMutex;
	USemaphore _addSem;
	double _emptyTrashesTime;
	std::string _url;
};

}

#endif /* DBDRIVER_H_ */
