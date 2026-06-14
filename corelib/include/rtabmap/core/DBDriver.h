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

/**
 * @class DBDriver
 * @brief Abstract database driver for RTAB-Map maps (signatures, links, words, statistics).
 *
 * DBDriver is the persistence layer used by @ref Memory, @ref DBReader and tools that
 * read or write \c .db files. The default implementation is @ref DBDriverSqlite3, created
 * by @ref create().
 *
 * The driver extends @ref UThreadNode: pending @ref Signature and @ref VisualWord objects
 * are queued with @ref asyncSave() and flushed to the database by @ref emptyTrashes()
 * (also called from the background thread on @ref closeConnection()).
 *
 * Public methods are thread-safe where noted (mutex-protected wrappers). Subclasses
 * implement SQL-specific logic in protected \c *Query() virtual methods.
 *
 * @see DBDriverSqlite3
 * @see Memory
 */
class RTABMAP_CORE_EXPORT DBDriver : public UThreadNode
{
public:
	/**
	 * @brief Factory: returns a SQLite database driver (@ref DBDriverSqlite3).
	 * @param parameters Optional driver parameters (e.g. @ref Parameters::kDbTargetVersion()).
	 * @return New driver instance; caller owns the pointer.
	 */
	static DBDriver * create(const ParametersMap & parameters = ParametersMap());

public:
	virtual ~DBDriver();

	/** @brief Parse driver parameters from the map (e.g. target schema version). */
	virtual void parseParameters(const ParametersMap & parameters);
	/**
	 * @brief True when no database file URL was set at open time.
	 * @note @ref DBDriverSqlite3 overrides this when @ref Parameters::kDbSqlite3InMemory() is enabled.
	 */
	virtual bool isInMemory() const {return _url.empty();}
	/** @return Database file path last passed to @ref openConnection(). */
	const std::string & getUrl() const {return _url;}
	/** @return Target schema version for new databases (from parameters). */
	const std::string & getTargetVersion() const {return _targetVersion;}

	/** @brief Queue a signature for deferred save; ownership is transferred. */
	void asyncSave(Signature * s);
	/** @brief Queue a visual word for deferred save; ownership is transferred. */
	void asyncSave(VisualWord * vw);
	/**
	 * @brief Flush queued signatures and visual words to the database.
	 * @param async If true, signal the background thread instead of flushing synchronously.
	 */
	void emptyTrashes(bool async = false);
	double getEmptyTrashesTime() const {return _emptyTrashesTime;}
	void setTimestampUpdateEnabled(bool enabled) {_timestampUpdate = enabled;} // used on Update Signature and Word queries

	/**
	 * @brief Export the pose graph to a Graphviz DOT file for visualization.
	 *
	 * Reads nodes and links from the database (and optionally from @p otherSignatures
	 * not yet persisted) and writes @p fileName. Does not modify the database.
	 * If @p ids is empty, all node ids are included.
	 */
	void generateGraph(
			const std::string & fileName,
			const std::set<int> & ids = std::set<int>(),
			const std::map<int, Signature *> & otherSignatures = std::map<int, Signature *>());

	/**
	 * @name Direct database updates
	 * @brief Write node or link data immediately (not via the async trash).
	 * @{
	 */
	/** @brief Insert or replace a link in the database. */
	void addLink(const Link & link);
	/** @brief Remove a link between two nodes. */
	void removeLink(int from, int to);
	/** @brief Update an existing link in the database. */
	void updateLink(const Link & link);
	/**
	 * @brief Update occupancy grid cells for a node.
	 * @param ground Ground cells (raw @c CV_32FC2/@c CV_32FC3, or compressed @c CV_8UC1 1×N).
	 * @param obstacles Obstacle cells (same formats as @p ground).
	 * @param empty Empty cells (same formats as @p ground; ignored on DB schema &lt; 0.16.0).
	 * Raw mats are compressed internally via @ref SensorData::setOccupancyGrid() before writing.
	 */
	void updateOccupancyGrid(
				int nodeId,
				const cv::Mat & ground,
				const cv::Mat & obstacles,
				const cv::Mat & empty,
				float cellSize,
				const cv::Point3f & viewpoint);
	/** @brief Update camera calibration stored for a node. */
	void updateCalibration(
		int nodeId,
		const std::vector<CameraModel> & models,
		const std::vector<StereoCameraModel> & stereoModels);
	/**
	 * @brief Update the depth image stored for a node.
	 * @param image Raw depth image, or pre-compressed blob (@c CV_8UC1, single row).
	 * @param format Compression format when @p image is raw (e.g. @c ".png"); ignored if already compressed.
	 * Uncompressed images are compressed with @ref Compression::compressImage2() before writing.
	 */
	void updateDepthImage(int nodeId, const cv::Mat & image, const std::string & format);
	/**
	 * @brief Update the laser scan stored for a node.
	 * @param scan Uncompressed scan, or already compressed (@ref LaserScan::isCompressed()).
	 * Uncompressed data is compressed with @ref Compression::compressData2() before writing.
	 */
	void updateLaserScan(int nodeId, const LaserScan & scan);
	/** @} */

public:
	/**
	 * @name Session exports and map artifacts
	 * @brief Statistics, preview image, optimized poses, 2D map, mesh and FLANN index.
	 * @{*/
	void addInfoAfterRun(int stMemSize, int lastSignAdded, int processMemUsed, int databaseMemUsed, int dictionarySize, const ParametersMap & parameters) const;
	/**
	 * @brief Append a @ref Statistics record for a processed node.
	 * @param statistics Metrics for @ref Statistics::refImageId() (insert skipped if id &le; 0 or data empty).
	 * @param saveWmState If true and schema &ge; 0.16.2, also store compressed working-memory state.
	 */
	void addStatistics(const Statistics & statistics, bool saveWmState) const;
	/**
	 * @brief Save the map preview thumbnail in the Admin table.
	 * @param image Raw image or JPEG-compressed blob (@c CV_8UC1, single row); empty clears it.
	 * @note Requires database schema &ge; 0.12.0.
	 */
	void savePreviewImage(const cv::Mat & image) const;
	/** @brief Load and uncompress the map preview thumbnail from the Admin table. */
	cv::Mat loadPreviewImage() const;
	/** @brief Persist graph-optimized poses and last localization pose. */
	void saveOptimizedPoses(const std::map<int, Transform> & optimizedPoses, const Transform & lastlocalizationPose) const;
	/** @brief Load optimized poses; optional last localization pose output. */
	std::map<int, Transform> loadOptimizedPoses(Transform * lastlocalizationPose = 0) const;
	/** @brief Save the assembled 2D occupancy grid and its origin metadata. */
	void save2DMap(const cv::Mat & map, float xMin, float yMin, float cellSize) const;
	/** @brief Load the 2D map and fill origin/cell size outputs. */
	cv::Mat load2DMap(float & xMin, float & yMin, float & cellSize) const;
	/**
	 * @brief Persist the global optimized 3D mesh to the database.
	 * @param cloud Point cloud (@c CV_32FC1 or @c CV_32FC3).
	 * @param polygons Optional per-texture polygon index lists (texture → polygon → vertex indices).
	 * @param texCoords Optional UV coordinates per texture (one per polygon vertex).
	 * @param textures Optional concatenated square texture images (same size per texture).
	 */
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
	/**
	 * @brief Persist the visual word dictionary FLANN index (serialized blob).
	 * @param indexData Serialized index from @ref VWDictionary::serializeIndex(); pass empty to clear.
	 * @note Requires database schema &ge; 0.23.0. Used when @ref Parameters::kKpFlannIndexSaved() is enabled.
	 */
	void saveFlannIndex(const std::vector<unsigned char> & indexData) const;
	/** @} */

public:
	/**
	 * @name Connection and database introspection
	 * @brief Mutex-protected wrappers around protected \c *Query() methods.
	 * @{*/
	/** @brief Open or create the database at @p url (empty @p url uses an in-memory database). */
	bool openConnection(const std::string & url, bool overwritten = false, bool readOnly = false);
	/** @brief Close the connection; optionally flush trashes and save in-memory DB to @p outputUrl. */
	void closeConnection(bool save = true, const std::string & outputUrl = "");
	/** @return True if a database connection is active. */
	bool isConnected() const;
	unsigned long getMemoryUsed() const; // In bytes
	/** @return Schema version string stored in the database (e.g. "0.21.0"). */
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

	/** @} */

	/**
	 * @brief Run a SQL statement that does not return result rows.
	 *
	 * Executes @p sql on the open database (e.g. @c INSERT, @c UPDATE, @c DELETE,
	 * @c CREATE, @c PRAGMA). Thread-safe wrapper around @ref executeNoResultQuery().
	 *
	 * For reads, use the dedicated @ref loadSignature(), @ref loadLinks() and related
	 * query methods instead of raw SQL.
	 *
	 * @param sql Complete SQL statement (SQLite syntax for @ref DBDriverSqlite3).
	 * @note The connection must be open (@ref isConnected()). On failure the driver
	 *       aborts with an assertion (SQLite backend).
	 * @warning @p sql is passed verbatim; sanitize any user-controlled values before calling.
	 */
	void executeNoResult(const std::string & sql) const;

	/**
	 * @name Load and query
	 * @brief Load signatures, words, links and node metadata from the database.
	 * @{*/

	// Load objects
	void load(VWDictionary & dictionary, bool lastStateOnly = true) const;
	void loadLastNodes(std::list<Signature *> & signatures, bool loadWordIdsOnly = false) const; // returned signatures must be freed after usage
	/** @brief Load one signature by id; caller must delete the returned pointer. */
	Signature * loadSignature(int id, bool * loadedFromTrash = 0); // returned signature must be freed after usage, call loadSignatures() instead if more than one signature should be loaded
	void loadSignatures(const std::list<int> & ids, std::list<Signature *> & signatures, std::set<int> * loadedFromTrash = 0, bool loadWordIdsOnly = false); // returned signatures must be freed after usage
	void loadWords(const std::set<int> & wordIds, std::list<VisualWord *> & vws); // returned words must be freed after usage

	// Specific queries...
	void loadNodeData(Signature & signature, bool images = true, bool scan = true, bool userData = true, bool occupancyGrid = true) const;
	void loadNodeData(std::list<Signature *> & signatures, bool images = true, bool scan = true, bool userData = true, bool occupancyGrid = true) const;
	void getNodeData(int signatureId, SensorData & data, bool images = true, bool scan = true, bool userData = true, bool occupancyGrid = true) const;
	bool getCalibration(int signatureId, std::vector<CameraModel> & models, std::vector<StereoCameraModel> & stereoModels) const;
	bool getLaserScanInfo(int signatureId, LaserScan & info) const;
	bool getNodeInfo(int signatureId, Transform & pose, int & mapId, int & weight, std::string & label, double & stamp, Transform & groundTruthPose, std::vector<float> & velocity, GPS & gps, EnvSensors & sensors) const;
	void getLocalFeatures(int signatureId, std::multimap<int, int> & words, std::vector<cv::KeyPoint> & keypoints, std::vector<cv::Point3f> & points, cv::Mat & descriptors) const;
	/** @brief Load outgoing links from @p signatureId, optionally filtered by @p type. */
	void loadLinks(int signatureId, std::multimap<int, Link> & links, Link::Type type = Link::kUndef) const;
	void getWeight(int signatureId, int & weight) const;
	void getLastNodeIds(std::set<int> & ids) const;
	void getAllNodeIds(std::set<int> & ids, bool ignoreChildren = false, bool ignoreBadSignatures = false, bool ignoreIntermediateNodes = false) const;
	void getAllOdomPoses(std::map<int, Transform> & poses, bool ignoreChildren = false, bool ignoreIntermediateNodes = false) const;
	void getAllLinks(std::multimap<int, Link> & links, bool ignoreNullLinks = true, bool withLandmarks = false) const;
	void getLastNodeId(int & id) const;
	void getLastMapId(int & mapId) const;
	void getLastWordId(int & id) const;
	void getInvertedIndexNi(int signatureId, int & ni) const;
	void getNodesObservingLandmark(int landmarkId, std::map<int, Link> & nodes) const;
	void getNodeIdByLabel(const std::string & label, int & id) const;
	void getAllLabels(std::map<int, std::string> & labels) const;
	/** @} */

protected:
	/**
	 * @brief Protected constructor for subclasses.
	 */
	DBDriver(const ParametersMap & parameters = ParametersMap());

	/**
	 * @name Backend implementation (subclass responsibility)
	 * @brief Pure virtual SQL/backend hooks invoked by public wrappers above.
	 * @{*/
	virtual bool connectDatabaseQuery(const std::string & url, bool overwritten = false, bool readOnly = false) = 0;
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

	virtual void updateCalibrationQuery(
			int nodeId,
			const std::vector<CameraModel> & models,
			const std::vector<StereoCameraModel> & stereoModels) const = 0;

	virtual void updateDepthImageQuery(
			int nodeId,
			const cv::Mat & image,
			const std::string & format) const = 0;

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
	virtual void saveFlannIndexQuery(const std::vector<unsigned char> & indexData) const = 0;

	// Load objects
	virtual void loadQuery(VWDictionary & dictionary, bool lastStateOnly = true) const = 0;
	virtual void loadLastNodesQuery(std::list<Signature *> & signatures, bool loadWordIdsOnly) const = 0;
	virtual void loadSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures, bool loadWordIdsOnly) const = 0;
	virtual void loadWordsQuery(const std::set<int> & wordIds, std::list<VisualWord *> & vws) const = 0;
	virtual void loadLinksQuery(int signatureId, std::multimap<int, Link> & links, Link::Type type = Link::kUndef) const = 0;

	virtual void loadNodeDataQuery(std::list<Signature *> & signatures, bool images=true, bool scan=true, bool userData=true, bool occupancyGrid=true) const = 0;
	virtual bool getCalibrationQuery(int signatureId, std::vector<CameraModel> & models, std::vector<StereoCameraModel> & stereoModels) const = 0;
	virtual bool getLaserScanInfoQuery(int signatureId, LaserScan & info) const = 0;
	virtual bool getNodeInfoQuery(int signatureId, Transform & pose, int & mapId, int & weight, std::string & label, double & stamp, Transform & groundTruthPose, std::vector<float> & velocity, GPS & gps, EnvSensors & sensors) const = 0;
	virtual void getLocalFeaturesQuery(int signatureId, std::multimap<int, int> & words, std::vector<cv::KeyPoint> & keypoints, std::vector<cv::Point3f> & points, cv::Mat & descriptors) const = 0;
	virtual void getLastNodeIdsQuery(std::set<int> & ids) const = 0;
	virtual void getAllNodeIdsQuery(std::set<int> & ids, bool ignoreChildren, bool ignoreBadSignatures, bool ignoreIntermediateNodes) const = 0;
	virtual void getAllOdomPosesQuery(std::map<int, Transform> & poses, bool ignoreChildren, bool ignoreIntermediateNodes) const = 0;
	virtual void getAllLinksQuery(std::multimap<int, Link> & links, bool ignoreNullLinks, bool withLandmarks) const = 0;
	virtual void getLastIdQuery(const std::string & tableName, int & id, const std::string & fieldName="id") const = 0;
	virtual void getInvertedIndexNiQuery(int signatureId, int & ni) const = 0;
	virtual void getNodesObservingLandmarkQuery(int landmarkId, std::map<int, Link> & nodes) const = 0;
	virtual void getNodeIdByLabelQuery(const std::string & label, int & id) const = 0;
	virtual void getAllLabelsQuery(std::map<int, std::string> & labels) const = 0;
	/** @} */

private:
	/** @brief Begin a database transaction (nested calls are serialized). */
	void beginTransaction() const;
	/** @brief Commit the current transaction. */
	void commit() const;

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
