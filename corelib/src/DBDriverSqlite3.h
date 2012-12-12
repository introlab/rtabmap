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

#ifndef DBDRIVERSQLITE3_H_
#define DBDRIVERSQLITE3_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines
#include "rtabmap/core/DBDriver.h"
#include <opencv2/features2d/features2d.hpp>
#include <sqlite3.h>

namespace rtabmap {

class RTABMAP_EXP DBDriverSqlite3: public DBDriver {
public:
	DBDriverSqlite3(const ParametersMap & parameters = ParametersMap());
	virtual ~DBDriverSqlite3();

	virtual void parseParameters(const ParametersMap & parameters);
	void setDbInMemory(bool dbInMemory);
	void setJournalMode(int journalMode);
	void setCacheSize(unsigned int cacheSize);
	void setSynchronous(int synchronous);
	void setTempStore(int tempStore);

private:
	virtual bool connectDatabaseQuery(const std::string & url, bool overwirtten = false);
	virtual void disconnectDatabaseQuery();
	virtual bool isConnectedQuery() const;
	virtual long getMemoryUsedQuery() const; // In bytes

	virtual void executeNoResultQuery(const std::string & sql) const;

	virtual void getNeighborIdsQuery(int signatureId, std::set<int> & neighbors, bool onlyWithActions = false) const;
	virtual void getWeightQuery(int signatureId, int & weight) const;
	virtual void getLoopClosureIdsQuery(int signatureId, std::set<int> & loopIds, std::set<int> & childIds) const;

	virtual void saveQuery(const std::list<Signature *> & signatures) const;
	virtual void saveQuery(const std::list<VisualWord *> & words) const;
	virtual void updateQuery(const std::list<Signature *> & signatures) const;
	virtual void updateQuery(const std::list<VisualWord *> & words) const;

	// Load objects
	virtual void loadQuery(VWDictionary * dictionary) const;
	virtual void loadLastNodesQuery(std::list<Signature *> & signatures) const;
	virtual void loadSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures) const;
	virtual void loadWordsQuery(const std::set<int> & wordIds, std::list<VisualWord *> & vws) const;
	virtual void loadNeighborsQuery(int signatureId, std::set<int> & neighbors) const;

	virtual void getImageQuery(int nodeId, cv::Mat & image) const;
	virtual void getAllNodeIdsQuery(std::set<int> & ids) const;
	virtual void getLastIdQuery(const std::string & tableName, int & id) const;
	virtual void getInvertedIndexNiQuery(int signatureId, int & ni) const;

private:
	std::string queryStepNode() const;
	std::string queryStepNodeToSensor() const;
	std::string queryStepImage() const;
	std::string queryStepLink() const;
	std::string queryStepWordsChanged() const;
	std::string queryStepKeypoint() const;
	void stepNode(sqlite3_stmt * ppStmt, const Signature * s) const;
	void stepNodeToSensor(sqlite3_stmt * ppStmt, int nodeId, int sensorId, int num) const;
	void stepImage(sqlite3_stmt * ppStmt, int id, const cv::Mat & image) const;
	void stepLink(sqlite3_stmt * ppStmt, int fromId, int toId, int type) const;
	void stepWordsChanged(sqlite3_stmt * ppStmt, int signatureId, int oldWordId, int newWordId) const;
	void stepKeypoint(sqlite3_stmt * ppStmt, int signatureId, int wordId, const cv::KeyPoint & kp) const;

private:
	void loadLinksQuery(std::list<Signature *> & signatures) const;
	int loadOrSaveDb(sqlite3 *pInMemory, const std::string & fileName, int isSave) const;

private:
	sqlite3 * _ppDb;
	bool _dbInMemory;
	unsigned int _cacheSize;
	int _journalMode;
	int _synchronous;
	int _tempStore;
};

}

#endif /* DBDRIVERSQLITE3_H_ */
