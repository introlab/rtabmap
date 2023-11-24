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

#ifndef DBDRIVERSQLITE3_H_
#define DBDRIVERSQLITE3_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines
#include "rtabmap/core/DBDriver.h"
#include <opencv2/features2d/features2d.hpp>

typedef struct sqlite3_stmt sqlite3_stmt;
typedef struct sqlite3 sqlite3;

namespace rtabmap {

class RTABMAP_CORE_EXPORT DBDriverSqlite3: public DBDriver {
public:
	DBDriverSqlite3(const ParametersMap & parameters = ParametersMap());
	virtual ~DBDriverSqlite3();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual bool isInMemory() const {return getUrl().empty() || _dbInMemory;}
	void setDbInMemory(bool dbInMemory);
	void setJournalMode(int journalMode);
	void setCacheSize(unsigned int cacheSize);
	void setSynchronous(int synchronous);
	void setTempStore(int tempStore);

protected:
	virtual bool connectDatabaseQuery(const std::string & url, bool overwritten = false);
	virtual void disconnectDatabaseQuery(bool save = true, const std::string & outputUrl = "");
	virtual bool isConnectedQuery() const;
	virtual unsigned long getMemoryUsedQuery() const; // In bytes
	virtual bool getDatabaseVersionQuery(std::string & version) const;
	virtual long getNodesMemoryUsedQuery() const;
	virtual long getLinksMemoryUsedQuery() const;
	virtual long getImagesMemoryUsedQuery() const;
	virtual long getDepthImagesMemoryUsedQuery() const;
	virtual long getCalibrationsMemoryUsedQuery() const;
	virtual long getGridsMemoryUsedQuery() const;
	virtual long getLaserScansMemoryUsedQuery() const;
	virtual long getUserDataMemoryUsedQuery() const;
	virtual long getWordsMemoryUsedQuery() const;
	virtual long getFeaturesMemoryUsedQuery() const;
	virtual long getStatisticsMemoryUsedQuery() const;
	virtual int getLastNodesSizeQuery() const;
	virtual int getLastDictionarySizeQuery() const;
	virtual int getTotalNodesSizeQuery() const;
	virtual int getTotalDictionarySizeQuery() const;
	virtual ParametersMap getLastParametersQuery() const;
	virtual std::map<std::string, float> getStatisticsQuery(int nodeId, double & stamp, std::vector<int> * wmState) const;
	virtual std::map<int, std::pair<std::map<std::string, float>, double> > getAllStatisticsQuery() const;
	virtual std::map<int, std::vector<int> > getAllStatisticsWmStatesQuery() const;

	virtual void executeNoResultQuery(const std::string & sql) const;

	virtual void getWeightQuery(int signatureId, int & weight) const;

	virtual void saveQuery(const std::list<Signature *> & signatures);
	virtual void saveQuery(const std::list<VisualWord *> & words) const;
	virtual void updateQuery(const std::list<Signature *> & signatures, bool updateTimestamp) const;
	virtual void updateQuery(const std::list<VisualWord *> & words, bool updateTimestamp) const;

	virtual void addLinkQuery(const Link & link) const;
	virtual void updateLinkQuery(const Link & link) const;

	virtual void updateOccupancyGridQuery(
			int nodeId,
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			const cv::Mat & empty,
			float cellSize,
			const cv::Point3f & viewpoint) const;

	virtual void updateDepthImageQuery(
			int nodeId,
			const cv::Mat & image) const;

	void updateLaserScanQuery(
			int nodeId,
			const LaserScan & scan) const;

	virtual void addStatisticsQuery(const Statistics & statistics, bool saveWmState) const;
	virtual void savePreviewImageQuery(const cv::Mat & image) const;
	virtual cv::Mat loadPreviewImageQuery() const;
	virtual void saveOptimizedPosesQuery(const std::map<int, Transform> & optimizedPoses, const Transform & lastlocalizationPose) const;
	virtual std::map<int, Transform> loadOptimizedPosesQuery(Transform * lastlocalizationPose = 0) const;
	virtual void save2DMapQuery(const cv::Mat & map, float xMin, float yMin, float cellSize) const;
	virtual cv::Mat load2DMapQuery(float & xMin, float & yMin, float & cellSize) const;
	virtual void saveOptimizedMeshQuery(
			const cv::Mat & cloud,
			const std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > & polygons,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
			const std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > & texCoords,
#else
			const std::vector<std::vector<Eigen::Vector2f> > & texCoords,
#endif
			const cv::Mat & textures) const;
	virtual cv::Mat loadOptimizedMeshQuery(
			std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > * polygons,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
			std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > * texCoords,
#else
			std::vector<std::vector<Eigen::Vector2f> > * texCoords,
#endif
			cv::Mat * textures) const;

	// Load objects
	virtual void loadQuery(VWDictionary * dictionary, bool lastStateOnly = true) const;
	virtual void loadLastNodesQuery(std::list<Signature *> & signatures) const;
	virtual void loadSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures) const;
	virtual void loadWordsQuery(const std::set<int> & wordIds, std::list<VisualWord *> & vws) const;
	virtual void loadLinksQuery(int signatureId, std::multimap<int, Link> & links, Link::Type type = Link::kUndef) const;

	virtual void loadNodeDataQuery(std::list<Signature *> & signatures, bool images=true, bool scan=true, bool userData=true, bool occupancyGrid=true) const;
	virtual bool getCalibrationQuery(int signatureId, std::vector<CameraModel> & models, std::vector<StereoCameraModel> & stereoModels) const;
	virtual bool getLaserScanInfoQuery(int signatureId, LaserScan & info) const;
	virtual bool getNodeInfoQuery(int signatureId, Transform & pose, int & mapId, int & weight, std::string & label, double & stamp, Transform & groundTruthPose, std::vector<float> & velocity, GPS & gps, EnvSensors & sensors) const;
	virtual void getLastNodeIdsQuery(std::set<int> & ids) const;
	virtual void getAllNodeIdsQuery(std::set<int> & ids, bool ignoreChildren, bool ignoreBadSignatures, bool ignoreIntermediateNodes) const;
	virtual void getAllLinksQuery(std::multimap<int, Link> & links, bool ignoreNullLinks, bool withLandmarks) const;
	virtual void getLastIdQuery(const std::string & tableName, int & id, const std::string & fieldName="id") const;
	virtual void getInvertedIndexNiQuery(int signatureId, int & ni) const;
	virtual void getNodesObservingLandmarkQuery(int landmarkId, std::map<int, Link> & nodes) const;
	virtual void getNodeIdByLabelQuery(const std::string & label, int & id) const;
	virtual void getAllLabelsQuery(std::map<int, std::string> & labels) const;

private:
	std::string queryStepNode() const;
	std::string queryStepImage() const;
	std::string queryStepDepth() const;
	std::string queryStepDepthUpdate() const;
	std::string queryStepScanUpdate() const;
	std::string queryStepSensorData() const;
	std::string queryStepLinkUpdate() const;
	std::string queryStepLink() const;
	std::string queryStepWordsChanged() const;
	std::string queryStepKeypoint() const;
	std::string queryStepGlobalDescriptor() const;
	std::string queryStepOccupancyGridUpdate() const;
	void stepNode(sqlite3_stmt * ppStmt, const Signature * s) const;
	void stepImage(sqlite3_stmt * ppStmt, int id, const cv::Mat & imageBytes) const;
	void stepDepth(sqlite3_stmt * ppStmt, const SensorData & sensorData) const;
	void stepDepthUpdate(sqlite3_stmt * ppStmt, int nodeId, const cv::Mat & imageCompressed) const;
	void stepScanUpdate(sqlite3_stmt * ppStmt, int nodeId, const LaserScan & image) const;
	void stepSensorData(sqlite3_stmt * ppStmt, const SensorData & sensorData) const;
	void stepLink(sqlite3_stmt * ppStmt, const Link & link) const;
	void stepWordsChanged(sqlite3_stmt * ppStmt, int signatureId, int oldWordId, int newWordId) const;
	void stepKeypoint(sqlite3_stmt * ppStmt, int nodeID, int wordId, const cv::KeyPoint & kp, const cv::Point3f & pt, const cv::Mat & descriptor) const;
	void stepGlobalDescriptor(sqlite3_stmt * ppStmt, int nodeId, const GlobalDescriptor & descriptor) const;
	void stepOccupancyGridUpdate(sqlite3_stmt * ppStmt,
			int nodeId,
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			const cv::Mat & empty,
			float cellSize,
			const cv::Point3f & viewpoint) const;

private:
	void loadLinksQuery(std::list<Signature *> & signatures) const;
	int loadOrSaveDb(sqlite3 *pInMemory, const std::string & fileName, int isSave) const;

protected:
	sqlite3 * _ppDb;
	std::string _version;

private:
	unsigned long _memoryUsedEstimate;
	bool _dbInMemory;
	unsigned int _cacheSize;
	int _journalMode;
	int _synchronous;
	int _tempStore;
};

}

#endif /* DBDRIVERSQLITE3_H_ */
