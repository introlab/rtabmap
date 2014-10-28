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

#ifndef DBDRIVERSQLITE3_H_
#define DBDRIVERSQLITE3_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines
#include "rtabmap/core/DBDriver.h"
#include <opencv2/features2d/features2d.hpp>
#include "sqlite3/sqlite3.h"
#include <pcl/point_types.h>

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

	virtual void getWeightQuery(int signatureId, int & weight) const;

	virtual void saveQuery(const std::list<Signature *> & signatures) const;
	virtual void saveQuery(const std::list<VisualWord *> & words) const;
	virtual void updateQuery(const std::list<Signature *> & signatures) const;
	virtual void updateQuery(const std::list<VisualWord *> & words) const;

	// Load objects
	virtual void loadQuery(VWDictionary * dictionary) const;
	virtual void loadLastNodesQuery(std::list<Signature *> & signatures) const;
	virtual void loadSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures) const;
	virtual void loadWordsQuery(const std::set<int> & wordIds, std::list<VisualWord *> & vws) const;
	virtual void loadNeighborsQuery(int signatureId, std::map<int, Transform> & neighbors) const;
	virtual void loadLoopClosuresQuery(
			int signatureId,
			std::map<int, Transform> & loopIds,
			std::map<int, Transform> & childIds) const;

	virtual void loadNodeDataQuery(std::list<Signature *> & signatures, bool loadMetricData) const;
	virtual void getNodeDataQuery(
			int signatureId,
			cv::Mat & imageCompressed,
			cv::Mat & depthCompressed,
			cv::Mat & depth2dCompressed,
			float & fx,
			float & fy,
			float & cx,
			float & cy,
			Transform & localTransform) const;
	virtual void getNodeDataQuery(int signatureId, cv::Mat & imageCompressed) const;
	virtual void getPoseQuery(int signatureId, Transform & pose, int & mapId) const;
	virtual void getAllNodeIdsQuery(std::set<int> & ids, bool ignoreChildren) const;
	virtual void getLastIdQuery(const std::string & tableName, int & id) const;
	virtual void getInvertedIndexNiQuery(int signatureId, int & ni) const;

private:
	std::string queryStepNode() const;
	std::string queryStepImage() const;
	std::string queryStepDepth() const;
	std::string queryStepLink() const;
	std::string queryStepWordsChanged() const;
	std::string queryStepKeypoint() const;
	void stepNode(sqlite3_stmt * ppStmt, const Signature * s) const;
	void stepImage(
			sqlite3_stmt * ppStmt,
			int id,
			const cv::Mat & imageBytes) const;
	void stepDepth(
			sqlite3_stmt * ppStmt,
			int id,
			const cv::Mat & depthBytes,
			const cv::Mat & depth2dBytes,
			float fx,
			float fy,
			float cx,
			float cy,
			const Transform & localTransform) const;
	void stepLink(sqlite3_stmt * ppStmt, int fromId, int toId, int type, const Transform & transform) const;
	void stepWordsChanged(sqlite3_stmt * ppStmt, int signatureId, int oldWordId, int newWordId) const;
	void stepKeypoint(sqlite3_stmt * ppStmt, int signatureId, int wordId, const cv::KeyPoint & kp, const pcl::PointXYZ & pt) const;

private:
	void loadLinksQuery(std::list<Signature *> & signatures) const;
	int loadOrSaveDb(sqlite3 *pInMemory, const std::string & fileName, int isSave) const;
	bool getVersion(std::string &) const;

private:
	sqlite3 * _ppDb;
	std::string _version;
	bool _dbInMemory;
	unsigned int _cacheSize;
	int _journalMode;
	int _synchronous;
	int _tempStore;
};

}

#endif /* DBDRIVERSQLITE3_H_ */
