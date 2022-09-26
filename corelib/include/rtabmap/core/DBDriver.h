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

#ifndef DBDRIVER_H_
#define DBDRIVER_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <string>
#include <list>
#include <map>
#include <set>
#include <opencv2/core/core.hpp>
#include "rtabmap/utilite/UMutex.h"
#include "rtabmap/utilite/UThreadNode.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SensorData.h"
#include <rtabmap/core/Statistics.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Link.h>

namespace rtabmap {

class Signature;
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
class RTABMAP_CORE_EXPORT DBDriver : public UThreadNode
{
public:
	static DBDriver * create(const ParametersMap & parameters = ParametersMap());

public:
	virtual ~DBDriver();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual bool isInMemory() const {return _url.empty();}
	const std::string & getUrl() const {return _url;}
	const std::string & getTargetVersion() const {return _targetVersion;}

	void beginTransaction() const;
	void commit() const;

	void asyncSave(Signature * s); //ownership transferred
	void asyncSave(VisualWord * vw); //ownership transferred
	void emptyTrashes(bool async = false);
	double getEmptyTrashesTime() const {return _emptyTrashesTime;}
	void setTimestampUpdateEnabled(bool enabled) {_timestampUpdate = enabled;} // used on Update Signature and Word queries

	// Warning: the following functions don't look in the trash, direct database modifications
	void generateGraph(
			const std::string & fileName,
			const std::set<int> & ids = std::set<int>(),
			const std::map<int, Signature *> & otherSignatures = std::map<int, Signature *>());
	void addLink(const Link & link);
	void removeLink(int from, int to);
	void updateLink(const Link & link);
	void updateOccupancyGrid(
				int nodeId,
				const cv::Mat & ground,
				const cv::Mat & obstacles,
				const cv::Mat & empty,
				float cellSize,
				const cv::Point3f & viewpoint);
	void updateDepthImage(int nodeId, const cv::Mat & image);
	void updateLaserScan(int nodeId, const LaserScan & scan);

public:
	void addInfoAfterRun(int stMemSize, int lastSignAdded, int processMemUsed, int databaseMemUsed, int dictionarySize, const ParametersMap & parameters) const;
	void addStatistics(const Statistics & statistics, bool saveWmState) const;
	void savePreviewImage(const cv::Mat & image) const;
	cv::Mat loadPreviewImage() const;
	void saveOptimizedPoses(const std::map<int, Transform> & optimizedPoses, const Transform & lastlocalizationPose) const;
	std::map<int, Transform> loadOptimizedPoses(Transform * lastlocalizationPose = 0) const;
	void save2DMap(const cv::Mat & map, float xMin, float yMin, float cellSize) const;
	cv::Mat load2DMap(float & xMin, float & yMin, float & cellSize) const;
	void saveOptimizedMesh(
			const cv::Mat & cloud,
			const std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > & polygons = std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > >(),      // Textures -> polygons -> vertices
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
			const std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > & texCoords = std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > >(), // Textures -> uv coords for each vertex of the polygons
#else
			const std::vector<std::vector<Eigen::Vector2f> > & texCoords = std::vector<std::vector<Eigen::Vector2f> >(), // Textures -> uv coords for each vertex of the polygons
#endif
			const cv::Mat & textures = cv::Mat()) const; // concatenated textures (assuming square textures with all same size);
	cv::Mat loadOptimizedMesh(
			std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > * polygons = 0,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
			std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > * texCoords = 0,
#else
			std::vector<std::vector<Eigen::Vector2f> > * texCoords = 0,
#endif
			cv::Mat * textures = 0) const;

public:
	// Mutex-protected methods of abstract versions below

	bool openConnection(const std::string & url, bool overwritten = false);
	void closeConnection(bool save = true, const std::string & outputUrl = "");
	bool isConnected() const;
	unsigned long getMemoryUsed() const; // In bytes
	std::string getDatabaseVersion() const;
	long getNodesMemoryUsed() const;
	long getLinksMemoryUsed() const;
	long getImagesMemoryUsed() const;
	long getDepthImagesMemoryUsed() const;
	long getCalibrationsMemoryUsed() const;
	long getGridsMemoryUsed() const;
	long getLaserScansMemoryUsed() const;
	long getUserDataMemoryUsed() const;
	long getWordsMemoryUsed() const;
	long getFeaturesMemoryUsed() const;
	long getStatisticsMemoryUsed() const;
	int getLastNodesSize() const; // working memory
	int getLastDictionarySize() const; // working memory
	int getTotalNodesSize() const;
	int getTotalDictionarySize() const;
	ParametersMap getLastParameters() const;
	std::map<std::string, float> getStatistics(int nodeId, double & stamp, std::vector<int> * wmState=0) const;
	std::map<int, std::pair<std::map<std::string, float>, double> > getAllStatistics() const;
	std::map<int, std::vector<int> > getAllStatisticsWmStates() const;

	void executeNoResult(const std::string & sql) const;

	// Load objects
	void load(VWDictionary * dictionary, bool lastStateOnly = true) const;
	void loadLastNodes(std::list<Signature *> & signatures) const; // returned signatures must be freed after usage
	Signature * loadSignature(int id, bool * loadedFromTrash = 0); // returned signature must be freed after usage, call loadSignatures() instead if more than one signature should be loaded
	void loadSignatures(const std::list<int> & ids, std::list<Signature *> & signatures, std::set<int> * loadedFromTrash = 0); // returned signatures must be freed after usage
	void loadWords(const std::set<int> & wordIds, std::list<VisualWord *> & vws); // returned words must be freed after usage

	// Specific queries...
	void loadNodeData(Signature * signature, bool images = true, bool scan = true, bool userData = true, bool occupancyGrid = true) const;
	void loadNodeData(std::list<Signature *> & signatures, bool images = true, bool scan = true, bool userData = true, bool occupancyGrid = true) const;
	void getNodeData(int signatureId, SensorData & data, bool images = true, bool scan = true, bool userData = true, bool occupancyGrid = true) const;
	bool getCalibration(int signatureId, std::vector<CameraModel> & models, std::vector<StereoCameraModel> & stereoModels) const;
	bool getLaserScanInfo(int signatureId, LaserScan & info) const;
	bool getNodeInfo(int signatureId, Transform & pose, int & mapId, int & weight, std::string & label, double & stamp, Transform & groundTruthPose, std::vector<float> & velocity, GPS & gps, EnvSensors & sensors) const;
	void loadLinks(int signatureId, std::multimap<int, Link> & links, Link::Type type = Link::kUndef) const;
	void getWeight(int signatureId, int & weight) const;
	void getLastNodeIds(std::set<int> & ids) const;
	void getAllNodeIds(std::set<int> & ids, bool ignoreChildren = false, bool ignoreBadSignatures = false, bool ignoreIntermediateNodes = false) const;
	void getAllLinks(std::multimap<int, Link> & links, bool ignoreNullLinks = true, bool withLandmarks = false) const;
	void getLastNodeId(int & id) const;
	void getLastMapId(int & mapId) const;
	void getLastWordId(int & id) const;
	void getInvertedIndexNi(int signatureId, int & ni) const;
	void getNodesObservingLandmark(int landmarkId, std::map<int, Link> & nodes) const;
	void getNodeIdByLabel(const std::string & label, int & id) const;
	void getAllLabels(std::map<int, std::string> & labels) const;

protected:
	DBDriver(const ParametersMap & parameters = ParametersMap());

	virtual bool connectDatabaseQuery(const std::string & url, bool overwritten = false) = 0;
	virtual void disconnectDatabaseQuery(bool save = true, const std::string & outputUrl = "") = 0;
	virtual bool isConnectedQuery() const = 0;
	virtual unsigned long getMemoryUsedQuery() const = 0; // In bytes
	virtual bool getDatabaseVersionQuery(std::string & version) const = 0;
	virtual long getNodesMemoryUsedQuery() const = 0;
	virtual long getLinksMemoryUsedQuery() const = 0;
	virtual long getImagesMemoryUsedQuery() const = 0;
	virtual long getDepthImagesMemoryUsedQuery() const = 0;
	virtual long getCalibrationsMemoryUsedQuery() const = 0;
	virtual long getGridsMemoryUsedQuery() const = 0;
	virtual long getLaserScansMemoryUsedQuery() const = 0;
	virtual long getUserDataMemoryUsedQuery() const = 0;
	virtual long getWordsMemoryUsedQuery() const = 0;
	virtual long getFeaturesMemoryUsedQuery() const = 0;
	virtual long getStatisticsMemoryUsedQuery() const = 0;
	virtual int getLastNodesSizeQuery() const = 0;
	virtual int getLastDictionarySizeQuery() const = 0;
	virtual int getTotalNodesSizeQuery() const = 0;
	virtual int getTotalDictionarySizeQuery() const = 0;
	virtual ParametersMap getLastParametersQuery() const = 0;
	virtual std::map<std::string, float> getStatisticsQuery(int nodeId, double & stamp, std::vector<int> * wmState) const = 0;
	virtual std::map<int, std::pair<std::map<std::string, float>, double> > getAllStatisticsQuery() const = 0;
	virtual std::map<int, std::vector<int> > getAllStatisticsWmStatesQuery() const = 0;

	virtual void executeNoResultQuery(const std::string & sql) const = 0;

	virtual void getWeightQuery(int signatureId, int & weight) const = 0;

	virtual void saveQuery(const std::list<Signature *> & signatures) = 0;
	virtual void saveQuery(const std::list<VisualWord *> & words) const = 0;
	virtual void updateQuery(const std::list<Signature *> & signatures, bool updateTimestamp) const = 0;
	virtual void updateQuery(const std::list<VisualWord *> & words, bool updateTimestamp) const = 0;

	virtual void addLinkQuery(const Link & link) const = 0;
	virtual void updateLinkQuery(const Link & link) const = 0;

	virtual void updateOccupancyGridQuery(
			int nodeId,
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			const cv::Mat & empty,
			float cellSize,
			const cv::Point3f & viewpoint) const = 0;

	virtual void updateDepthImageQuery(
			int nodeId,
			const cv::Mat & image) const = 0;

	virtual void updateLaserScanQuery(
			int nodeId,
			const LaserScan & scan) const = 0;

	virtual void addStatisticsQuery(const Statistics & statistics, bool saveWmState) const = 0;
	virtual void savePreviewImageQuery(const cv::Mat & image) const = 0;
	virtual cv::Mat loadPreviewImageQuery() const = 0;
	virtual void saveOptimizedPosesQuery(const std::map<int, Transform> & optimizedPoses, const Transform & lastlocalizationPose) const = 0;
	virtual std::map<int, Transform> loadOptimizedPosesQuery(Transform * lastlocalizationPose = 0) const = 0;
	virtual void save2DMapQuery(const cv::Mat & map, float xMin, float yMin, float cellSize) const = 0;
	virtual cv::Mat load2DMapQuery(float & xMin, float & yMin, float & cellSize) const = 0;
	virtual void saveOptimizedMeshQuery(
				const cv::Mat & cloud,
				const std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > & polygons,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
				const std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > & texCoords,
#else
				const std::vector<std::vector<Eigen::Vector2f> > & texCoords,
#endif
				const cv::Mat & textures) const = 0;
	virtual cv::Mat loadOptimizedMeshQuery(
				std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > * polygons,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
				std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > * texCoords,
#else
				std::vector<std::vector<Eigen::Vector2f> > * texCoords,
#endif
				cv::Mat * textures) const = 0;

	// Load objects
	virtual void loadQuery(VWDictionary * dictionary, bool lastStateOnly = true) const = 0;
	virtual void loadLastNodesQuery(std::list<Signature *> & signatures) const = 0;
	virtual void loadSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures) const = 0;
	virtual void loadWordsQuery(const std::set<int> & wordIds, std::list<VisualWord *> & vws) const = 0;
	virtual void loadLinksQuery(int signatureId, std::multimap<int, Link> & links, Link::Type type = Link::kUndef) const = 0;

	virtual void loadNodeDataQuery(std::list<Signature *> & signatures, bool images=true, bool scan=true, bool userData=true, bool occupancyGrid=true) const = 0;
	virtual bool getCalibrationQuery(int signatureId, std::vector<CameraModel> & models, std::vector<StereoCameraModel> & stereoModels) const = 0;
	virtual bool getLaserScanInfoQuery(int signatureId, LaserScan & info) const = 0;
	virtual bool getNodeInfoQuery(int signatureId, Transform & pose, int & mapId, int & weight, std::string & label, double & stamp, Transform & groundTruthPose, std::vector<float> & velocity, GPS & gps, EnvSensors & sensors) const = 0;
	virtual void getLastNodeIdsQuery(std::set<int> & ids) const = 0;
	virtual void getAllNodeIdsQuery(std::set<int> & ids, bool ignoreChildren, bool ignoreBadSignatures, bool ignoreIntermediateNodes) const = 0;
	virtual void getAllLinksQuery(std::multimap<int, Link> & links, bool ignoreNullLinks, bool withLandmarks) const = 0;
	virtual void getLastIdQuery(const std::string & tableName, int & id, const std::string & fieldName="id") const = 0;
	virtual void getInvertedIndexNiQuery(int signatureId, int & ni) const = 0;
	virtual void getNodesObservingLandmarkQuery(int landmarkId, std::map<int, Link> & nodes) const = 0;
	virtual void getNodeIdByLabelQuery(const std::string & label, int & id) const = 0;
	virtual void getAllLabelsQuery(std::map<int, std::string> & labels) const = 0;

private:
	//non-abstract methods
	void saveOrUpdate(const std::vector<Signature *> & signatures);
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
	std::string _targetVersion;
	bool _timestampUpdate;
};

}

#endif /* DBDRIVER_H_ */
