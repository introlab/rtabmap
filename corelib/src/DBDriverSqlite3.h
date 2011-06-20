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

private:
	virtual bool connectDatabaseQuery(const std::string & url);
	virtual void disconnectDatabaseQuery();
	virtual bool isConnectedQuery() const;
	virtual long getMemoryUsedQuery() const; // In bytes

	virtual bool executeNoResultQuery(const std::string & sql) const;

	virtual bool changeWordsRefQuery(const std::map<int, int> & refsToChange) const; // <oldWordId, activeWordId>
	virtual bool deleteWordsQuery(const std::vector<int> & ids) const;
	virtual bool getNeighborIdsQuery(int signatureId, std::set<int> & neighbors) const;
	virtual bool getWeightQuery(int signatureId, int & weight) const;
	virtual bool getLoopClosureIdQuery(int signatureId, int & loopId) const;
	virtual bool addNeighborQuery(int id, int newNeighbor, int oldNeighbor) const;

	virtual bool saveQuery(const std::vector<VisualWord *> & visualWords) const;
	virtual bool updateQuery(const std::list<Signature *> & signatures) const;
	virtual bool saveQuery(const KeypointSignature * ss) const;
	virtual bool saveQuery(const std::list<KeypointSignature *> & signatures) const;

	// Load objects
	virtual bool loadQuery(VWDictionary * dictionary) const;
	virtual bool loadLastSignaturesQuery(std::list<Signature *> & signatures) const;
	virtual bool loadQuery(int signatureId, Signature ** s) const;
	virtual bool loadQuery(int wordId, VisualWord ** vw) const;
	virtual bool loadQuery(int signatureId, KeypointSignature * ss) const;
	virtual bool loadKeypointSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures, bool onlyParents = false) const;
	virtual bool loadWordsQuery(const std::list<int> & wordIds, std::list<VisualWord *> & vws) const;
	virtual bool loadNeighborsQuery(int signatureId, std::map<int, std::list<std::vector<float> > > & neighbors) const;

	virtual bool getImageCompressedQuery(int id, CvMat ** compressed) const;
	virtual bool getAllSignatureIdsQuery(std::set<int> & ids) const;
	virtual bool getLastSignatureIdQuery(int & id) const;
	virtual bool getLastVisualWordIdQuery(int & id) const;
	virtual bool getSurfNiQuery(int signatureId, int & ni) const;
	virtual bool getChildrenIdsQuery(int signatureId, std::list<int> & ids) const;
	virtual bool getHighestWeightedSignaturesQuery(unsigned int count, std::multimap<int, int> & ids) const;

private:
	int loadOrSaveDb(sqlite3 *pInMemory, const std::string & fileName, int isSave) const;

private:
	sqlite3 * _ppDb;
	bool _dbInMemory;
	unsigned int _cacheSize;
	int _journalMode;
};

}

#endif /* DBDRIVERSQLITE3_H_ */
