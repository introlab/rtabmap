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

#ifndef MEMORY_H_
#define MEMORY_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Link.h"
#include "rtabmap/core/Features2d.h"
#include <typeinfo>
#include <list>
#include <map>
#include <set>
#include "rtabmap/utilite/UStl.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/pcl_config.h>

namespace rtabmap {

class Signature;
class DBDriver;
class VWDictionary;
class VisualWord;
class Feature2D;
class Statistics;
class Registration;
class RegistrationInfo;
class RegistrationIcp;
class RegistrationVis;
class Stereo;
class LocalGridMaker;
class MarkerDetector;
class GlobalDescriptorExtractor;

/**
 * @class Memory
 * @brief Three-tiered memory management (STM, WM, LTM) for RTAB-Map.
 *
 * Memory is the core map data structure used by @ref Rtabmap. It stores observations
 * as @ref Signature nodes connected by @ref Link edges and orchestrates their lifecycle
 * across three tiers:
 *
 * - **Short-Term Memory (STM)**: most recently added signatures, kept at a fixed size
 *   (see @ref Parameters::kMemSTMSize()). Used to delay recently observed places before
 *   they become candidates for loop closure.
 * - **Working Memory (WM)**: signatures available for loop-closure likelihood
 *   computation in the current iteration. Older signatures are transferred from WM
 *   to LTM by @ref forget() to bound iteration time.
 * - **Long-Term Memory (LTM)**: persisted in the database via @ref DBDriver. Signatures
 *   can be brought back to WM with @ref reactivateSignatures() when their neighbors still 
 *   in WM are good loop-closure candidates.
 *
 * The class also owns a visual word dictionary (@ref VWDictionary), feature extractor
 * (@ref Feature2D) and registration pipelines (@ref Registration, @ref RegistrationVis,
 * @ref RegistrationIcp) used to compute relative transforms between signatures.
 *
 * Typical iteration: @ref update() adds a new @ref SensorData as a @ref Signature in
 * STM; @ref computeLikelihood() scores it against WM; the @ref Rtabmap caller decides
 * on loop closures with @ref BayesFilter; @ref cleanup() drops bad signatures;
 * @ref forget() transfers oldest WM signatures to LTM and @ref reactivateSignatures()
 * pulls relevant ones back.
 *
 * @see Signature
 * @see DBDriver
 * @see VWDictionary
 * @see Rtabmap
 */
class RTABMAP_CORE_EXPORT Memory
{
public:
	/** @brief First valid signature id assigned by @ref getNextId() (positive integer). */
	static const int kIdStart;
	/** @brief Reserved id for the "virtual place" used by the Bayes filter (negative). */
	static const int kIdVirtual;
	/** @brief Sentinel value indicating an invalid signature id (zero). */
	static const int kIdInvalid;

public:
	/**
	 * @brief Constructs a Memory instance with the given parameters.
	 *
	 * The database is not opened here; call @ref init() to open or create a database
	 * and load persisted state. @p parameters may include any key from @ref Parameters
	 * (memory, keypoint, registration, etc.); missing keys fall back to defaults.
	 */
	Memory(const ParametersMap & parameters = ParametersMap());
	virtual ~Memory();

	/**
	 * @brief Re-parses parameters and propagates them to owned sub-objects.
	 *
	 * Forwards the relevant subset to @ref VWDictionary, @ref Feature2D, the registration
	 * pipelines and the database driver. Safe to call at runtime to change settings.
	 */
	virtual void parseParameters(const ParametersMap & parameters);
	/** @return The most recent parameter map applied via the constructor or @ref parseParameters(). */
	virtual const ParametersMap & getParameters() const {return parameters_;}
	/**
	 * @brief Adds a sensor observation to the map (overload without odometry pose).
	 *
	 * Equivalent to calling the full @ref update() with an identity pose and empty covariance.
	 * The new signature is added to STM; oldest STM entries are promoted to WM as needed.
	 *
	 * @param data Sensor data (images, scan, user data, odometry features) for this frame.
	 * @param stats Optional output statistics receiver for timing/diagnostic values.
	 * @return True if a signature was successfully created and added, false otherwise.
	 */
	bool update(const SensorData & data,
			Statistics * stats = 0);
	/**
	 * @brief Adds a sensor observation with odometry pose and velocity to the map.
	 *
	 * Creates a new @ref Signature, extracts visual words, links it to the previous
	 * STM signature with a neighbor link, runs rehearsal against STM and promotes
	 * the oldest STM signature to WM if STM is full.
	 *
	 * @param data Sensor data (images, scan, user data, odometry features).
	 * @param pose Odometry pose at this frame (null if odometry not used).
	 * @param covariance 6x6 odometry covariance (or empty if null odometry is provided).
	 * @param velocity Optional 6-vector (vx, vy, vz, vroll, vpitch, vyaw).
	 * @param stats Optional output statistics receiver.
	 * @return True on success, false if signature creation failed.
	 */
	bool update(const SensorData & data,
			const Transform & pose,
			const cv::Mat & covariance,
			const std::vector<float> & velocity = std::vector<float>(), // vx,vy,vz,vroll,vpitch,vyaw
			Statistics * stats = 0);
	/**
	 * @brief Opens or creates a database and loads existing state into WM.
	 *
	 * @param dbUrl Path to the database file (empty for in-memory).
	 * @param dbOverwritten If true, deletes the existing file before opening.
	 * @param parameters Optional parameter override applied before loading.
	 * @param postInitClosingEvents If true, posts @ref RtabmapEventInit events for
	 *        progress reporting (used by GUI).
	 * @return True on success, false if the database could not be opened.
	 */
	bool init(const std::string & dbUrl,
			bool dbOverwritten = false,
			const ParametersMap & parameters = ParametersMap(),
			bool postInitClosingEvents = false);
	/**
	 * @brief Flushes pending data and closes the database connection.
	 *
	 * @param databaseSaved If true, persists STM/WM signatures and statistics before closing.
	 *        If false, in-memory state is discarded.
	 * @param postInitClosingEvents If true, posts progress events while closing.
	 * @param ouputDatabasePath If non-empty, the database is copied to this path on close. If a
	 *        database on disk was initially created/loaded on a different path, it will be updated with latest
	 *        changes and renamed to the output path.
	 */
	void close(bool databaseSaved = true, bool postInitClosingEvents = false, const std::string & ouputDatabasePath = "");
	/**
	 * @brief Computes loop-closure likelihood of @p signature against a set of WM ids.
	 *
	 * Compares visual words (tf-idf if enabled) between @p signature and each id in
	 * @p ids and returns a normalized likelihood per id.
	 *
	 * @param signature Query signature (typically the last added one).
	 * @param ids Candidate signature ids in working memory.
	 * @return Map from id to likelihood score.
	 */
	std::map<int, float> computeLikelihood(const Signature * signature,
			const std::list<int> & ids);
	/**
	 * @brief Starts a new map id, breaking session continuity (e.g. after localization loss).
	 *
	 * @param reducedIds If non-null, populated with id remappings produced by graph reduction
	 *        triggered by the new map.
	 * @return The new map id (auto-incremented).
	 */
	int incrementMapId(std::map<int, int> * reducedIds = 0);
	/**
	 * @brief Refreshes the age of @p signatureId in working memory, marking it as recent.
	 *
	 * Used to keep loop-closure hypotheses active so they are not transferred to LTM
	 * during the next @ref forget() call.
	 */
	void updateAge(int signatureId);

	/**
	 * @brief Transfers oldest signatures from WM to LTM to respect memory and/or time limits.
	 *
	 * Two regimes are used depending on the visual word dictionary state:
	 * - **Word-count regime**: active only when mapping mode is on, the @ref VWDictionary
	 *   is in incremental mode, contains at least one word, and is *not* using incremental
	 *   FLANN. In this regime, signatures are transferred until the number of visual words
	 *   removed from the dictionary catches up with the number of new words indexed since
	 *   the previous iteration.
	 * - **Signature-count regime**: used in every other case (localization mode, dictionary
	 *   not incremental, dictionary still empty -- e.g. lidar-only mapping or no feature
	 *   extraction -- or incremental FLANN, where the word count is no longer the bottleneck).
	 *   In this regime, at least one more signature than the count added/retrieved in the
	 *   previous iteration is transferred, regardless of words.
	 *
	 * In both regimes, candidate selection (see @c getRemovableSignatures()) honors
	 * @p ignoredIds, skips intermediate nodes, and excludes WM nodes linked to STM (to
	 * preserve rehearsal). Intermediate (weight==-1) nodes linked to a transferred
	 * signature are dragged out with it.
	 *
	 * @param ignoredIds Signatures that must not be transferred (e.g. STM, retrieved ids, on the planned path).
	 * @return Ids of signatures moved to LTM, in transfer order.
	 */
	std::list<int> forget(const std::set<int> & ignoredIds = std::set<int>());
	/**
	 * @brief Reloads signatures from LTM into WM.
	 *
	 * @param ids Candidate ids; those already in WM/STM are ignored.
	 * @param maxLoaded Hard cap on number of ids actually loaded (0 = unlimited).
	 * @param timeDbAccess Output: time spent in the database driver (seconds).
	 * @return Ids effectively brought back to WM.
	 */
	std::set<int> reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, double & timeDbAccess);

	/**
	 * @brief Drops the last signature if flagged as bad, or any signature in localization mode.
	 * @return Id of the removed signature, or 0 if none was removed.
	 */
	int cleanup();
	/** @brief Persists @p statistics to the database; @p saveWMState records the WM id list. */
	void saveStatistics(const Statistics & statistics, bool saveWMState);
	/** @brief Stores a preview image (typically a thumbnail of the last frame) in the database. */
	void savePreviewImage(const cv::Mat & image) const;
	/** @brief Loads the preview image previously written by @ref savePreviewImage(). */
	cv::Mat loadPreviewImage() const;
	/** @brief Persists an optimized pose graph and the last localization pose for next session. */
	void saveOptimizedPoses(const std::map<int, Transform> & optimizedPoses, const Transform & lastlocalizationPose) const;
	/** @brief Loads optimized poses previously written by @ref saveOptimizedPoses(). */
	std::map<int, Transform> loadOptimizedPoses(Transform * lastlocalizationPose) const;
	/** @brief Persists a 2D occupancy grid (origin @p xMin, @p yMin and resolution @p cellSize). */
	void save2DMap(const cv::Mat & map, float xMin, float yMin, float cellSize) const;
	/** @brief Loads the 2D occupancy grid previously written by @ref save2DMap(). */
	cv::Mat load2DMap(float & xMin, float & yMin, float & cellSize) const;
	/**
	 * @brief Persists an optimized textured/colored mesh to the database.
	 * @param cloud Point cloud (XYZRGB) of vertices.
	 * @param polygons Per-texture list of polygons; each polygon is a list of vertex indices.
	 * @param texCoords Per-texture list of UV coords matching @p polygons.
	 * @param textures Concatenated texture images (square, equal-sized).
	 */
	void saveOptimizedMesh(
			const cv::Mat & cloud,
			const std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > & polygons = std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > >(),      // Textures -> polygons -> vertices
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
			const std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > & texCoords = std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > >(), // Textures -> uv coords for each vertex of the polygons
#else
			const std::vector<std::vector<Eigen::Vector2f> > & texCoords = std::vector<std::vector<Eigen::Vector2f> >(), // Textures -> uv coords for each vertex of the polygons
#endif
			const cv::Mat & textures = cv::Mat()) const; // concatenated textures (assuming square textures with all same size)
	/** @brief Loads the optimized mesh previously written by @ref saveOptimizedMesh(). */
	cv::Mat loadOptimizedMesh(
			std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > * polygons = 0,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
			std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > * texCoords = 0,
#else
			std::vector<std::vector<Eigen::Vector2f> > * texCoords = 0,
#endif
			cv::Mat * textures = 0) const;
	/** @brief Forces the database driver to flush any queued signature/word saves. */
	void emptyTrash();
	/** @brief Blocks until the asynchronous database write thread has finished pending work. */
	void joinTrashThread();
	/**
	 * @brief Adds a graph link between two signatures.
	 * @param link Link to add (type, transform, covariance).
	 * @param addInDatabase If true, the link is also added when one of the ids is only in LTM.
	 * @return True if the link was added, false on conflict or missing nodes.
	 */
	bool addLink(const Link & link, bool addInDatabase = false);
	/** @brief Replaces an existing link with @p link (same endpoints and type). */
	void updateLink(const Link & link, bool updateInDatabase = false);
	/** @brief Removes every virtual link in WM. */
	void removeAllVirtualLinks();
	/** @brief Removes virtual links attached to @p signatureId. */
	void removeVirtualLinks(int signatureId);
	/**
	 * @brief Breadth-first walk of the pose graph from @p signatureId.
	 *
	 * Visits neighbor and (optionally) loop-closure neighbors up to @p maxGraphDepth.
	 *
	 * @param signatureId Starting node.
	 * @param maxGraphDepth Maximum graph distance (0 = infinite graph depth).
	 * @param maxCheckedInDatabase Cap on LTM look-ups (-1 = unlimited, 0 = WM only).
	 * @param incrementMarginOnLoop If true, loop-closure links count toward depth.
	 * @param ignoreLoopIds If true, loop-closure neighbors are not traversed.
	 * @param ignoreIntermediateNodes If true, weight==-1 intermediate nodes are skipped.
	 * @param ignoreLocalSpaceLoopIds If true, only global loop closures are traversed.
	 * @param nodesSet If non-empty, traversal is constrained to these ids.
	 * @param dbAccessTime Output: time spent in database access (seconds).
	 * @return Map from visited node id to graph depth, including @p signatureId (with graph depth of 0)
	 */
	std::map<int, int> getNeighborsId(
			int signatureId,
			int maxGraphDepth,
			int maxCheckedInDatabase = -1,
			bool incrementMarginOnLoop = false,
			bool ignoreLoopIds = false,
			bool ignoreIntermediateNodes = false,
			bool ignoreLocalSpaceLoopIds = false,
			const std::set<int> & nodesSet = std::set<int>(),
			double * dbAccessTime = 0) const;
	/**
	 * @brief Returns neighbor ids within a Euclidean radius using optimized poses.
	 * @param signatureId Query node id.
	 * @param radius Maximum distance from @p signatureId (meters).
	 * @param optimizedPoses Pose graph after optimization (used for distances).
	 * @param maxGraphDepth Maximum graph distance (in terms of nodes) to bound the search.
	 * @return Map from node id to squared distance from @p signatureId.
	 */
	std::map<int, float> getNeighborsIdRadius(
			int signatureId,
			float radius,
			const std::map<int, Transform> & optimizedPoses,
			int maxGraphDepth) const;
	/** @brief Marks @p locationId as intermediate (weight = -1); excludes it from loop closure. */
	void convertToIntermediate(int locationId);
	/**
	 * @brief Removes @p locationId from WM/STM and the database.
	 * @param locationId Id of the signature to delete.
	 * @param deletedWords Optional output: words whose reference count dropped to zero.
	 */
	void deleteLocation(int locationId, std::list<int> * deletedWords = 0);
	/** @brief Forces @p locationId to be flushed to the database. */
	void saveLocationData(int locationId);
	/** @brief Removes any link between @p idA and @p idB (both directions). */
	void removeLink(int idA, int idB);
	/** @brief Strips raw images, scan, user data and/or occupancy grid from @p id to save memory (RAM). This doesn't clear any compressed data.*/
	void removeRawData(int id, bool image = true, bool scan = true, bool userData = true, bool occupancyGrid = true);
	/**
	 * @brief Merges @p id with a close neighbor (graph reduction).
	 * @param id Node to reduce.
	 * @param maxDistance Maximum distance to a neighbor to allow the merge (meters).
	 * @param keepLinkedInDb If true, the merged node is kept in the database (history only).
	 * @param direction Restrict merge target: 0=any, 1=previous neighbor, 2=next neighbor.
	 * @return Id of the node @p id was merged into, or 0 if no reduction was performed.
	 */
	int reduceNode(int id, float maxDistance = 0.0f, bool keepLinkedInDb = false, int direction = 0);

	/** @return Working memory as { signature id, age } (does not include STM). */
	const std::map<int, double> & getWorkingMem() const {return _workingMem;}
	/** @return Set of signature ids currently in short-term memory. */
	const std::set<int> & getStMem() const {return _stMem;}
	/** @return Configured maximum STM size (@ref Parameters::kMemSTMSize()). */
	int getMaxStMemSize() const {return _maxStMemSize;}
	/** @brief Returns neighbor (sequential) links of @p signatureId; @p lookInDatabase also checks LTM. */
	std::multimap<int, Link> getNeighborLinks(int signatureId,
			bool lookInDatabase = false) const;
	/** @brief Returns loop-closure links of @p signatureId; @p lookInDatabase also checks LTM. */
	std::multimap<int, Link> getLoopClosureLinks(int signatureId,
			bool lookInDatabase = false) const;
	/**
	 * @brief Returns all links of @p signatureId (neighbor, loop, prior, gravity, ...).
	 * @param signatureId Source node id (can also be a landmark id).
	 * @param lookInDatabase Also query LTM for links.
	 * @param withLandmarks Include landmark links in the result.
	 */
	std::multimap<int, Link> getLinks(int signatureId,
			bool lookInDatabase = false,
			bool withLandmarks = false) const;
	/** @brief Returns links of every signature; @p ignoreNullLinks drops empty placeholders. */
	std::multimap<int, Link> getAllLinks(bool lookInDatabase, bool ignoreNullLinks = true, bool withLandmarks = false) const;
	/** @return True if raw binary data (images, scans) is kept in memory after compression. */
	bool isBinDataKept() const {return _binDataKept;}
	/** @return Similarity threshold used by rehearsal (@ref Parameters::kMemRehearsalSimilarity()). */
	float getSimilarityThreshold() const {return _similarityThreshold;}
	/** @return Map from signature id to weight (rehearsal accumulation count) for WM and STM. */
	std::map<int, int> getWeights() const;
	/** @return Id of the most recently added signature, or 0 if none. */
	int getLastSignatureId() const;
	/**
	 * @brief Returns the most recent WM signature.
	 * @param ignoreIntermediateNodes If true, skips weight==-1 placeholder nodes.
	 */
	const Signature * getLastWorkingSignature(bool ignoreIntermediateNodes) const;
	/** @brief Returns all nodes observing landmark @p landmarkId, mapped to the observation link. */
	std::map<int, Link> getNodesObservingLandmark(int landmarkId, bool lookInDatabase) const;
	/** @return Signature id labeled @p label, or 0 if not found. */
	int getSignatureIdByLabel(const std::string & label, bool lookInDatabase = true) const;
	/**
	 * @brief Assigns or removes a label on @p id.
	 * @param id Signature id; pass 0 to remove an existing label by name.
	 * @param label Label text; empty to remove.
	 * @return True if the label was applied.
	 */
	bool labelSignature(int id, const std::string & label);
	/** @return Map from signature id to non-empty label (including STM+WM+LTM). */
	const std::map<int, std::string> & getAllLabels() const {return _labels;}
	/** @return Reverse landmark index: { landmark id (negative), nodes observing it }. */
	const std::map<int, std::set<int> > & getLandmarksIndex() const {return _landmarksIndex;}
	/** @return True if every persisted node (in LTM) is currently loaded in WM/STM. */
	bool allNodesInWM() const {return _allNodesInWM;}

	/**
	 * @brief Attaches user data to signature @p id, compressing it on the fly if needed.
	 *
	 * The format is detected automatically: a single-row @c CV_8UC1 matrix is treated
	 * as already-compressed data and stored as-is; anything else is considered raw and
	 * compressed before being stored.
	 *
	 * @note If you pass one-dimensional unsigned 8-bit raw data, transpose it so it has
	 *       multiple rows (not multiple columns), otherwise it will be misdetected as
	 *       already compressed.
	 *
	 * @param id Target signature id (must be in WM/STM or LTM).
	 * @param data Raw or pre-compressed user data.
	 * @return True if the data was attached, false if @p id was not found.
	 */
	bool setUserData(int id, const cv::Mat & data);
	/** @return On-disk database size in bytes. */
	int getDatabaseMemoryUsed() const;
	/** @return Schema version of the open database (e.g. "0.20.0"). */
	std::string getDatabaseVersion() const;
	/** @return File path of the open database (empty if in-memory). */
	std::string getDatabaseUrl() const;
	/** @return Last @ref emptyTrash() flush time in seconds. */
	double getDbSavingTime() const;
	/** @return Map id of signature @p id; @p lookInDatabase also queries LTM. */
	int getMapId(int id, bool lookInDatabase = false) const;
	/** @return Odometry pose stored with @p signatureId (null if unknown). */
	Transform getOdomPose(int signatureId, bool lookInDatabase = false) const;
	/** @return Ground-truth pose stored with @p signatureId (null if unknown). */
	Transform getGroundTruthPose(int signatureId, bool lookInDatabase = false) const;
	/** @return Ground-truth poses for nodes currently in WM/STM. */
	const std::map<int, Transform> & getGroundTruths() const {return _groundTruths;}
	/**
	 * @brief Returns a GPS fix for @p id, falling back to the nearest GPS-tagged neighbor.
	 *
	 * Two cases:
	 * - If @p id has a GPS fix attached, @p gps is set to that fix and @p offsetENU
	 *   is left at identity.
	 * - Otherwise, the graph is searched (via @ref getNeighborsId() with depth
	 *   @p maxGraphDepth, ignoring loop closures) for the closest neighbor that has a
	 *   GPS fix. When one is found, @p gps is set to that neighbor's fix and
	 *   @p offsetENU is the rigid transform from that neighbor's pose to @p id,
	 *   expressed in ENU coordinates (derived from the neighbor's heading/bearing).
	 *   Applying @p offsetENU on top of the GPS-derived pose of the neighbor yields
	 *   the ENU pose of @p id.
	 *
	 * If no GPS fix is found on @p id or any reachable neighbor, @p gps is returned
	 * empty (@c gps.stamp()==0) and @p offsetENU is identity.
	 *
	 * @param id Query signature id.
	 * @param gps Output GPS fix (empty if none found).
	 * @param offsetENU Output ENU-frame offset from the GPS-tagged node to @p id
	 *        (identity when @p id itself carries the GPS fix or when none is found).
	 * @param lookInDatabase If true, also fetch missing nodes from LTM during the search.
	 * @param maxGraphDepth Maximum graph depth used to look for a GPS-tagged neighbor
	 *        when @p id has none (0 = no depth limit, i.e. search the whole reachable graph).
	 */
	void getGPS(int id, GPS & gps, Transform & offsetENU, bool lookInDatabase, int maxGraphDepth = 0) const;
	/**
	 * @brief Reads metadata of @p signatureId (no images/scan/words).
	 *
	 * @param signatureId Node id.
	 * @param odomPose Output odometry pose (null if not set).
	 * @param mapId Output map id.
	 * @param weight Output rehearsal weight.
	 * @param label Output label (empty if none).
	 * @param stamp Output timestamp (seconds, epoch).
	 * @param groundTruth Output ground-truth pose (null if not set).
	 * @param velocity Output 6-vector velocity (empty if not set).
	 * @param gps Output GPS fix (invalid if not set).
	 * @param sensors Output environmental sensor readings.
	 * @param lookInDatabase Also query LTM.
	 * @return True if @p signatureId was found.
	 */
	bool getNodeInfo(int signatureId,
			Transform & odomPose,
			int & mapId,
			int & weight,
			std::string & label,
			double & stamp,
			Transform & groundTruth,
			std::vector<float> & velocity,
			GPS & gps,
			EnvSensors & sensors,
			bool lookInDatabase = false) const;
	/** @return Compressed image blob for @p signatureId (empty if not stored). */
	cv::Mat getImageCompressed(int signatureId) const;
	/**
	 * @brief Loads sensor data of @p locationId from WM or LTM.
	 * @param images Include compressed RGB/depth images.
	 * @param scan Include laser scan blob.
	 * @param userData Include user data blob.
	 * @param occupancyGrid Include occupancy grid cells.
	 */
	SensorData getNodeData(int locationId, bool images, bool scan, bool userData, bool occupancyGrid) const;
	/** @brief Loads the visual words, 3D points and global descriptors stored with @p nodeId. */
	void getNodeWordsAndGlobalDescriptors(int nodeId,
			std::multimap<int, int> & words,
			std::vector<cv::KeyPoint> & wordsKpts,
			std::vector<cv::Point3f> & words3,
			cv::Mat & wordsDescriptors,
			std::vector<GlobalDescriptor> & globalDescriptors) const;
	/** @brief Loads mono and/or stereo camera calibration stored with @p nodeId. */
	void getNodeCalibration(int nodeId,
			std::vector<CameraModel> & models,
			std::vector<StereoCameraModel> & stereoModels) const;
	/**
	 * @brief Returns all signature ids in WM, STM and LTM.
	 * @param ignoreChildren If true, nodes not linked to graph anymore are excluded
	 */
	std::set<int> getAllSignatureIds(bool ignoreChildren = true) const;
	/**
	 * @brief Reports whether the in-memory map has been modified since the database was last
	 *        loaded or reset. @ref close() uses this flag to decide whether the database
	 *        needs to be rewritten.
	 *
	 * The flag is cleared to @c false on construction and by @ref close() / @ref clear(),
	 * and is raised to @c true on any of the following events:
	 *
	 * - **@ref update() in mapping mode** (@ref Parameters::kMemIncrementalMemory() == @c true,
	 *   see @ref isIncremental()): every successful call sets the flag, since a new
	 *   @ref Signature is added to the graph and the visual word dictionary may grow.
	 * - **@ref update() in localization mode** (@ref isIncremental() == @c false): the flag
	 *   is set only when @ref Parameters::kMemLocalizationDataSaved() is enabled
	 *   (see @ref isLocalizationDataSaved()), i.e. when the new node must be persisted
	 *   back to the database. Pure localization (the default,
	 *   @ref Parameters::kMemLocalizationDataSaved() == @c false) leaves the flag at
	 *   @c false even after many @ref update() calls, because nothing needs to be saved.
	 * - **@ref reduceNode()**: merging a node into a neighbor mutates the graph and
	 *   marks the memory as changed (and also raises the link-changed flag).
	 * - **@ref init() dictionary repair**: when @ref init() rebuilds the visual word
	 *   dictionary because words are missing from the database, the flag is forced to
	 *   @c true so the regenerated dictionary is saved back on @ref close(), even if no
	 *   new data was processed.
	 *
	 * Note that link-only modifications (e.g. @ref addLink(), @ref updateLink(),
	 * @ref removeLink()) update an independent @c _linksChanged flag, not this one.
	 *
	 * @return True if the memory has changed and would need to be persisted.
	 */
	bool memoryChanged() const {return _memoryChanged;}
	/**
	 * @return True if the memory grows on @ref update() (mapping mode), false in localization mode.
	 * @see Parameters::kMemIncrementalMemory()
	 */
	bool isIncremental() const {return _incrementalMemory;}
	/**
	 * @return True in localization mode when database writes are disabled
	 *         (i.e. @ref isIncremental() is false and @ref Parameters::kMemLocalizationReadOnly() is enabled).
	 * @see Parameters::kMemIncrementalMemory()
	 * @see Parameters::kMemLocalizationReadOnly()
	 */
	bool isReadOnly() const {return !_incrementalMemory && _localizationReadOnly;}
	/**
	 * @return True if data added during localization is persisted to the database.
	 * @see Parameters::kMemLocalizationDataSaved()
	 */
	bool isLocalizationDataSaved() const {return _localizationDataSaved;}
	/** @return Signature with @p id in WM/STM, or null if not loaded. */
	const Signature * getSignature(int id) const;
	/** @return True if @p signatureId is in short-term memory. */
	bool isInSTM(int signatureId) const {return _stMem.find(signatureId) != _stMem.end();}
	/** @return True if @p signatureId is in working memory. */
	bool isInWM(int signatureId) const {return _workingMem.find(signatureId) != _workingMem.end();}
	/** @return True if @p signatureId is not in STM/WM, so when it is in LTM or non-existing. */
	bool isInLTM(int signatureId) const {return !this->isInSTM(signatureId) && !this->isInWM(signatureId);}
	/** @return True if signature ids are auto-generated, false if taken from sensor data id. */
	bool isIDsGenerated() const {return _generateIds;}
	/** @return Id of the last accepted global loop-closure node, or 0 if none. */
	int getLastGlobalLoopClosureId() const {return _lastGlobalLoopClosureId;}
	/** @return Feature extractor used to compute visual words. */
	const Feature2D * getFeature2D() const {return _feature2D;}
	/** @return True if graph reduction is enabled (@ref Parameters::kMemReduceGraph()). */
	bool isGraphReduced() const {return _reduceGraph;}
	/**
	 * @return Running per-axis maximum of the diagonal of every neighbor (odometry) link's
	 *         information matrix observed so far, as a 6-vector
	 *         (x, y, z, roll, pitch, yaw). Empty until at least one 6x6 neighbor link
	 *         information matrix has been seen.
	 *
	 * This is a runtime statistic, not a configurable parameter: it is updated on
	 * @ref init() (over all loaded neighbor links) and on every @ref update() that
	 * adds a new neighbor link.
	 *
	 * It is consumed by @ref Rtabmap::getInformation() when
	 * @ref Parameters::kRGBDLoopCovLimited() is enabled, to clip loop-closure
	 * information matrices so a loop never claims higher confidence than odometry
	 * itself ever provided.
	 *
	 * @see Parameters::kRGBDLoopCovLimited()
	 */
	const std::vector<double> & getOdomMaxInf() const {return _odomMaxInf;}
	/**
	 * @return True if the odometry pose orientation is used (instead of the IMU
	 *         orientation) as the source of each new node's gravity link.
	 *
	 * When enabled, every new node gets a self-loop @ref Link::kGravity holding the
	 * rotation of the odometry pose passed to @ref update(). This assumes odometry is
	 * already gravity-aligned (e.g. a VIO front-end). When disabled, the gravity link
	 * is built from the IMU orientation in @ref SensorData::imu() if available.
	 *
	 * Gravity links are consumed by graph optimization only when
	 * @ref Parameters::kOptimizerGravitySigma() is non-zero.
	 *
	 * @see Parameters::kMemUseOdomGravity()
	 * @see Parameters::kOptimizerGravitySigma()
	 */
	bool isOdomGravityUsed() const {return _useOdometryGravity;}

	/** @brief Writes a human-readable dump of WM/STM, links and weights to @p fileNameTree. */
	void dumpMemoryTree(const char * fileNameTree) const;
	/** @brief Dumps every internal map (signatures, words, dictionary) to text files in @p directory. */
	virtual void dumpMemory(std::string directory) const;
	/** @brief Dumps signatures' word ids (and 3D positions if @p words3D) to @p fileNameSign. */
	virtual void dumpSignatures(const char * fileNameSign, bool words3D) const;
	/** @brief Dumps the visual word dictionary: references to @p fileNameRef, descriptors to @p fileNameDesc. */
	void dumpDictionary(const char * fileNameRef, const char * fileNameDesc) const;
	/** @return Approximate RAM usage of the in-memory state, in bytes. */
	unsigned long getMemoryUsed() const; //Bytes

	/**
	 * @brief Writes a Graphviz DOT file of the pose graph.
	 * @param fileName Output path.
	 * @param ids If non-empty, restrict the graph to these node ids.
	 */
	void generateGraph(const std::string & fileName, const std::set<int> & ids = std::set<int>());
	/**
	 * @brief Removes spurious obstacle points from each node's local grid using a reference 2D map.
	 *
	 * For every node in @p poses, the node's local **obstacle** grid is loaded, each
	 * obstacle point is projected with @p poses into the reference @p map, and the
	 * point is kept only if either:
	 * - the reference @p map cell at its projection is not free space
	 *   (i.e. the cell is an obstacle or unknown, value != 0), or
	 * - the reference @p map contains an obstacle cell (value == 100) within
	 *   @p cropRadius cells of the projection.
	 *
	 * Points that fall on a free cell and have no obstacle neighbor within
	 * @p cropRadius are dropped. The filtered obstacle grid replaces the node's grid
	 * in WM/STM and (if the node is already persisted) in the database via
	 * @ref DBDriver::updateOccupancyGrid().
	 *
	 * **Ground and empty cells are not touched**: they are read and written back as-is.
	 *
	 * When @p filterScans is true, the same projection/filtering rule is also applied
	 * to each node's raw laser scan, and the rewritten scan is saved back to the
	 * database. This is useful to remove dynamic objects from the stored scans before
	 * re-meshing or re-exporting.
	 *
	 * @param poses Optimized poses used to project the local grids/scans into @p map.
	 * @param map Reference 2D occupancy grid (cell values: 0 free, 100 occupied,
	 *        anything else unknown).
	 * @param xMin Reference map origin x in world coordinates (meters).
	 * @param yMin Reference map origin y in world coordinates (meters).
	 * @param cellSize Reference map resolution (meters/cell); must match the nodes' grid cell size.
	 * @param cropRadius Search radius (in cells) around each projected point used to
	 *        accept points near an obstacle in @p map.
	 * @param filterScans If true, also filter and rewrite the raw laser scan attached
	 *        to each node, using the same rule as for obstacle cells.
	 * @return Number of (node, grid or scan) modifications performed, or -1 on error
	 *         (no database loaded, empty @p poses or empty @p map).
	 */
	int cleanupLocalGrids(
			const std::map<int, Transform> & poses,
			const cv::Mat & map,
			float xMin,
			float yMin,
			float cellSize,
			int cropRadius = 1,
			bool filterScans = false);

	/** @return Visual word dictionary used for tf-idf likelihood and feature matching. */
	const VWDictionary * getVWDictionary() const;

	/**
	 * @brief Extracts a sub-graph (poses + links) for a set of node ids.
	 *
	 * Used by graph optimization callers to retrieve constraints for a region of interest.
	 *
	 * @param ids Ids to include.
	 * @param poses Output: odometry poses for @p ids.
	 * @param links Output: links between the nodes (and to landmarks if @p landmarksAdded).
	 * @param lookInDatabase If true, fetch missing data from LTM.
	 * @param landmarksAdded If true, also include landmark constraints.
	 */
	void getMetricConstraints(
			const std::set<int> & ids,
			std::map<int, Transform> & poses,
			std::multimap<int, Link> & links,
			bool lookInDatabase = false,
			bool landmarksAdded = false);

	/**
	 * @brief Computes the relative transform from @p fromS to @p toS using the registration pipeline.
	 * @param fromS Source signature (will be modified to cache extracted data).
	 * @param toS Target signature.
	 * @param guess Initial transform estimate (null if unknown).
	 * @param info Optional output with inlier counts, variance and diagnostics.
	 * @param useKnownCorrespondencesIfPossible If true, reuses existing word-id correspondences.
	 * @return The estimated transform, or a null @ref Transform on failure.
	 */
	Transform computeTransform(Signature & fromS, Signature & toS, Transform guess, RegistrationInfo * info = 0, bool useKnownCorrespondencesIfPossible = false) const;
	/** @brief Convenience overload: loads signatures by id and forwards to the @ref Signature variant. */
	Transform computeTransform(int fromId, int toId, Transform guess, RegistrationInfo * info = 0, bool useKnownCorrespondencesIfPossible = false);
	/**
	 * @brief Refines a transform using ICP alignment of laser scans only.
	 * @return The refined transform, or a null @ref Transform on failure.
	 */
	Transform computeIcpTransform(const Signature & fromS, const Signature & toS, Transform guess, RegistrationInfo * info = 0) const;
	/**
	 * @brief ICP registration of one node against an assembled cloud from multiple neighbors.
	 * @param newId New (query) node id.
	 * @param oldId Reference node id.
	 * @param poses Neighbor poses used to assemble the reference cloud.
	 * @param info Optional output with inlier counts and diagnostics.
	 * @return The estimated transform, or a null @ref Transform on failure.
	 */
	Transform computeIcpTransformMulti(
			int newId,
			int oldId,
			const std::map<int, Transform> & poses,
			RegistrationInfo * info = 0);

private:
	void preUpdate();
	void addSignatureToStm(Signature * signature, const cv::Mat & covariance);
	void clear();
	void loadDataFromDb(bool postInitClosingEvents);
	void saveFlannIndex(bool postInitClosingEvents);
	void moveToTrash(Signature * s, bool keepLinkedToGraph = true, std::list<int> * deletedWords = 0);

	void moveSignatureToWMFromSTM(int id, int * reducedTo = 0);
	void addSignatureToWmFromLTM(Signature * signature);
	Signature * _getSignature(int id) const;
	std::list<Signature *> getRemovableSignatures(int count,
			const std::set<int> & ignoredIds = std::set<int>());
	int getNextId();
	void initCountId();
	void rehearsal(Signature * signature, Statistics * stats = 0);
	bool rehearsalMerge(int oldId, int newId);
	bool canBeReduced(const Link & link, float maxDistance, int direction);

	const std::map<int, Signature*> & getSignatures() const {return _signatures;}

	void copyData(const Signature * from, Signature * to);
	Signature * createSignature(
			const SensorData & data,
			const Transform & pose,
			Statistics * stats = 0);

	//keypoint stuff
	void disableWordsRef(int signatureId);
	void enableWordsRef(const std::list<int> & signatureIds);
	void cleanUnusedWords();
	int getNi(int signatureId) const;

protected:
	/** @brief Database driver owning the persistent storage (created by @ref init()). */
	DBDriver * _dbDriver;

private:
	// parameters
	ParametersMap parameters_;
	float _similarityThreshold;
	bool _binDataKept;
	bool _rawDescriptorsKept;
	bool _loadVisualLocalFeaturesOnInit;
	bool _saveDepth16Format;
	bool _notLinkedNodesKeptInDb;
	bool _saveIntermediateNodeData;
	std::string _rgbCompressionFormat;
	std::string _depthCompressionFormat;
	bool _incrementalMemory;
	bool _localizationReadOnly;
	bool _localizationDataSaved;
	bool _flannIndexSaved;
	bool _reduceGraph;
	int _maxStMemSize;
	float _recentWmRatio;
	bool _transferSortingByWeightId;
	bool _idUpdatedToNewOneRehearsal;
	bool _generateIds;
	bool _badSignaturesIgnored;
	bool _mapLabelsAdded;
	bool _depthAsMask;
	float _maskFloorThreshold;
	bool _stereoFromMotion;
	unsigned int _imagePreDecimation;
	unsigned int _imagePostDecimation;
	bool _compressionParallelized;
	float _laserScanDownsampleStepSize;
	float _laserScanVoxelSize;
	int _laserScanNormalK;
	float _laserScanNormalRadius;
	float _laserScanGroundNormalsUp;
	bool _reextractLoopClosureFeatures;
	bool _localBundleOnLoopClosure;
	bool _invertedReg;
	float _rehearsalMaxDistance;
	float _rehearsalMaxAngle;
	bool _rehearsalWeightIgnoredWhileMoving;
	bool _useOdometryFeatures;
	bool _useOdometryGravity;
	bool _rotateImagesUpsideUp;
	bool _createOccupancyGrid;
	int _visMaxFeatures;
	bool _visSSC;
	bool _imagesAlreadyRectified;
	bool _rectifyOnlyFeatures;
	bool _covOffDiagonalIgnored;
	bool _detectMarkers;
	float _markerLinVariance;
	float _markerAngVariance;
	bool _markerOrientationIgnored;

	int _idCount;
	int _idMapCount;
	Signature * _lastSignature;
	int _lastGlobalLoopClosureId;
	bool _memoryChanged; // False by default, become true only when Memory::update() is called.
	bool _linksChanged; // False by default, become true when links are modified.
	int _signaturesAdded;
	bool _allNodesInWM;
	bool _receivingOdometryFeatures;
	GPS _gpsOrigin;
	std::vector<CameraModel> _rectCameraModels;
	std::vector<StereoCameraModel> _rectStereoCameraModels;
	std::vector<double> _odomMaxInf;

	std::map<int, Signature *> _signatures; // TODO : check if a signature is already added? although it is not supposed to occur...
	std::set<int> _stMem; // id
	std::map<int, double> _workingMem; // id,age
	std::map<int, Transform> _groundTruths;
	std::map<int, std::string> _labels;
	std::map<int, std::set<int> > _landmarksIndex; // < -landmarkId, nodeIds >
    std::map<int, float> _landmarksSize;           // +landmarkId

	//Keypoint stuff
	VWDictionary * _vwd;
	Feature2D * _feature2D;
	float _badSignRatio;
	bool _tfIdfLikelihoodUsed;
	bool _parallelized;

	Registration * _registrationPipeline;
	RegistrationIcp * _registrationIcpMulti;
	RegistrationVis * _registrationVis;

	LocalGridMaker * _localMapMaker;

	MarkerDetector * _markerDetector;

	GlobalDescriptorExtractor * _globalDescriptorExtractor;
};

} // namespace rtabmap

#endif /* MEMORY_H_ */
