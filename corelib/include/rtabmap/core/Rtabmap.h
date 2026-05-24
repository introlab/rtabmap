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

#ifndef RTABMAP_H_
#define RTABMAP_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Statistics.h"
#include "rtabmap/core/Link.h"
#include "rtabmap/core/ProgressState.h"
#include "rtabmap/core/Graph.h"

#include <opencv2/core/core.hpp>
#include <list>
#include <stack>
#include <set>

namespace rtabmap
{

class EpipolarGeometry;
class Memory;
class BayesFilter;
class Signature;
class Optimizer;
class PythonInterface;

/**
 * @class Rtabmap
 * @brief Top-level RTAB-Map SLAM pipeline (mapping, localization and loop closure).
 *
 * Rtabmap orchestrates the full SLAM iteration. Each new sensor observation passed to
 * @ref process() goes through the steps described below.
 *
 *
 * @par 1. Memory update
 *
 * Done via @ref Memory::update(): a new @ref Signature is added to STM, the oldest
 * STM entry is promoted to WM if STM is full, and rehearsal compares the new
 * signature to the previous STM signature.
 *
 * When the robot barely moved since the previous frame (odometry displacement below
 * @ref Parameters::kRGBDLinearUpdate() and @ref Parameters::kRGBDAngularUpdate()), the
 * iteration is flagged as a "small displacement":
 *   - If a loop closure or localization was already accepted on a recent iteration,
 *     appearance-based global loop-closure detection and proximity detection by space
 *     are both skipped to avoid wasting work while the robot is stationary at an
 *     already-known location (only retrieval runs).
 *   - Otherwise (no recent loop closure / localization), both still run normally so
 *     a first-time loop closure can still be detected from a standstill.
 *
 * In either case, at the end of the iteration, if no loop closure, proximity
 * detection or landmark observation latched onto the new node, it is deleted from
 * @ref Memory so the map does not grow while the robot is idle.
 *
 * Rehearsal still runs first, so visually similar consecutive idle frames may also be
 * merged into the previous STM signature (its weight is incremented and the new
 * signature is discarded) when the similarity exceeds
 * @ref Parameters::kMemRehearsalSimilarity().
 *
 *
 * @par 2. Loop-closure hypothesis
 *
 * Scored via @ref Memory::computeLikelihood() and the recursive @ref BayesFilter
 * (prior + observation update).
 *
 *
 * @par 3. Hypothesis selection
 *
 * The highest posterior is compared against the loop-closure threshold
 * (@ref Parameters::kRtabmapLoopThr()); if accepted, the loop-closure link is added
 * and the pose graph is re-optimized by @ref Optimizer.
 *
 * In **RGB-D mode**, two extra checks must pass before the link is committed:
 *   - a valid geometric transform must be computed between the two candidate nodes by
 *     the registration pipeline (visual + optional ICP, see
 *     @ref Memory::computeTransform());
 *   - the resulting transform must not be rejected by the graph-optimization
 *     consistency check (see @ref Parameters::kRGBDOptimizeMaxError()).
 *
 * The optimization used by the consistency check depends on the operating mode:
 *   - In **mapping mode**, @ref optimizeCurrentMap() re-optimizes the local map
 *     around the current signature including the new link, then
 *     @ref graph::computeMaxGraphErrors() measures the worst per-link residual / its
 *     standard deviation. If the ratio exceeds @ref Parameters::kRGBDOptimizeMaxError(),
 *     the loop closure(s) added this iteration are removed from @ref Memory.
 *   - In **localization mode**, optimization is run on a sub-graph composed of the
 *     odometry cache (@ref Parameters::kRGBDMaxOdomCacheSize()), the newly added
 *     localization link and pose priors fixing the map nodes (weighted by
 *     @ref Parameters::kRGBDLocalizationPriorInf()). The same error-ratio check is
 *     applied; on failure the localization is rejected for this iteration but the
 *     persisted map and its links are left untouched.
 *
 * In both modes, if the same link is rejected twice in a row, @ref repairGraph() may
 * also be attempted (within @ref Parameters::kRGBDOptimizeMaxErrorRepairRadius()) to
 * drop the offending link instead of the new candidate.
 *
 * If either RGB-D check fails, the candidate is discarded and no link is added.
 *
 *
 * @par 4. Retrieval
 *
 * Once a loop-closure hypothesis is selected, neighbors of the matched node are
 * brought back from LTM into WM via @ref Memory::reactivateSignatures(), so the next
 * iteration can compare against them too. Up to @ref Parameters::kRtabmapMaxRetrieved()
 * nodes are pulled per iteration; nodes around the current path or local pose may
 * also be retrieved (capped by @ref Parameters::kRtabmapMaxLocalRetrieved()).
 *
 * Retrieval (and the related node immunization) is only active when memory management
 * is enabled, i.e. when @ref Parameters::kRtabmapTimeThr() or
 * @ref Parameters::kRtabmapMemoryThr() is non-zero.
 *
 *
 * @par 5. Proximity detection (RGB-D mode)
 *
 * Visual and scan-based local matches to nearby nodes, used in addition to the
 * appearance-based loop closure.
 *
 * Candidate proximity links go through the same two RGB-D gates as loop closures
 * above: a valid geometric transform must be computed by the registration pipeline,
 * and the transform must not be rejected by the graph-optimization consistency check.
 *
 *
 * @par 6. Transfer (WM to LTM)
 *
 * At the end of the iteration, if the iteration exceeded the configured time budget
 * (@ref Parameters::kRtabmapTimeThr()) or WM exceeded its size budget
 * (@ref Parameters::kRtabmapMemoryThr()), @ref Memory::forget() moves the oldest
 * low-frequency signatures from WM to LTM (immunized nodes -- retrieved neighbors,
 * the last localization node, etc. -- are kept in WM).
 *
 * Transfer is skipped when both thresholds are 0 (memory management disabled).
 *
 *
 * @par 7. Map / localization output
 *
 * Optimized poses, current map correction and statistics are made available to
 * callers via the getters below.
 *
 *
 * @par Operating modes
 *
 * Selected by @ref Parameters::kMemIncrementalMemory() (see @ref Memory::isIncremental()):
 *   - **Mapping**: STM and WM grow; loop closures update the optimized graph.
 *   - **Localization**: STM/WM are frozen; the current node is matched against the
 *     persisted map and only @ref getLastLocalizationPose() is updated.
 *
 *
 * @par Path planning
 *
 * Rtabmap also exposes basic graph-based path planning in RGB-D mode
 * (@ref computePath(), @ref getPath(), @ref getPathStatus()), used by the GUI to
 * navigate between mapped locations.
 *
 * When memory management is enabled (see step 4 Retrieval and step 6 Transfer), the
 * retrieval step also pulls nodes along the currently planned path back from LTM into
 * WM (capped by @ref Parameters::kRtabmapMaxLocalRetrieved()) so the robot is able to
 * re-localize against upcoming waypoints as it follows the path, even when those
 * nodes had been transferred out of WM earlier.
 *
 *
 * @see Memory
 * @see BayesFilter
 * @see Optimizer
 * @see Parameters
 */
class RTABMAP_CORE_EXPORT Rtabmap
{
public:
	/** @brief Loop-closure verification strategy. */
	enum VhStrategy {
		kVhNone,     ///< No verification: the highest hypothesis above threshold is accepted.
		kVhEpipolar, ///< Epipolar geometry verification (mostly historical, RGB-only mode).
		kVhUndef     ///< Sentinel -- undefined.
	};

public:
	Rtabmap();
	virtual ~Rtabmap();

	/**
	 * @brief Main RTAB-Map iteration: ingests one sensor frame and updates the map.
	 *
	 * Adds @p data to @ref Memory, runs the Bayes filter on the current likelihood,
	 * selects a loop-closure hypothesis if any, performs proximity detection,
	 * re-optimizes the graph as needed, and refreshes @ref getStatistics() and
	 * @ref getLastLocalizationPose().
	 *
	 * @param data Sensor data for this frame (images, scan, user data, ...).
	 * @param odomPose Odometry pose; must be non-null in RGB-D SLAM mode.
	 *        Pass a null @ref Transform to fall back to appearance-only mode.
	 * @param odomCovariance 6x6 odometry covariance (default: identity).
	 * @param odomVelocity Optional 6-vector (vx, vy, vz, vroll, vpitch, vyaw).
	 * @param externalStats Extra named statistics to record in the database for this iteration.
	 * @return True if @p data was added to the map (i.e. an @ref update() succeeded).
	 */
	bool process(
			const SensorData & data,
			Transform odomPose,
			const cv::Mat & odomCovariance = cv::Mat::eye(6,6,CV_64FC1),
			const std::vector<float> & odomVelocity = std::vector<float>(),
			const std::map<std::string, float> & externalStats = std::map<std::string, float>());
	/**
	 * @brief Convenience overload: builds a diagonal covariance from scalar variances.
	 *
	 * The 6x6 odometry covariance is constructed as
	 * @c diag(odomLinearVariance, odomLinearVariance, odomLinearVariance,
	 * odomAngularVariance, odomAngularVariance, odomAngularVariance).
	 */
	bool process(
			const SensorData & data,
			Transform odomPose,
			float odomLinearVariance,
			float odomAngularVariance,
			const std::vector<float> & odomVelocity = std::vector<float>(),
			const std::map<std::string, float> & externalStats = std::map<std::string, float>());
	/**
	 * @brief Appearance-only convenience overload (loop-closure detection without odometry).
	 *
	 * Equivalent to processing @p image alone, with no odometry pose. Useful for offline
	 * loop-closure benchmarking on image sequences.
	 *
	 * @param image RGB or grayscale frame.
	 * @param id Optional frame id (0 = auto-generated).
	 * @param externalStats Extra named statistics to record in the database for this iteration.
	 */
	bool process(
			const cv::Mat & image,
			int id=0, const std::map<std::string, float> & externalStats = std::map<std::string, float>());

	/**
	 * @brief Initializes Rtabmap with parameters and a database.
	 *
	 * @param parameters Parameters overriding default parameters and database parameters
	 *                   (see @p loadDatabaseParameters).
	 * @param databasePath Database input/output path. If empty, an in-memory database is
	 *                     used. If set and the file does not exist, it is created empty;
	 *                     if it exists, nodes and the visual word vocabulary are loaded
	 *                     into working memory.
	 * @param loadDatabaseParameters If true and an existing database is opened, the
	 *                               parameters stored inside the database are loaded and
	 *                               applied to this Rtabmap instance (then overridden by
	 *                               @p parameters).
	 */
	void init(const ParametersMap & parameters, const std::string & databasePath = "", bool loadDatabaseParameters = false);
	/**
	 * @brief Initializes Rtabmap from a configuration file and a database.
	 *
	 * @param configFile Configuration file (*.ini) overriding default parameters and
	 *                   database parameters (see @p loadDatabaseParameters).
	 * @param databasePath Database input/output path; same semantics as the other @ref init().
	 * @param loadDatabaseParameters If true and an existing database is opened, the
	 *                               parameters stored inside the database are loaded and
	 *                               applied first, then overridden by values from @p configFile.
	 */
	void init(const std::string & configFile = "", const std::string & databasePath = "", bool loadDatabaseParameters = false);

	/**
	 * @brief Closes Rtabmap and releases the underlying @ref Memory.
	 *
	 * @param databaseSaved If true, the in-memory state is flushed to the database;
	 *                      if false, in-memory changes are discarded.
	 * @param ouputDatabasePath If non-empty, the database is copied to this path on
	 *                      close. If a database on disk was initially created/loaded on
	 *                      a different path, it will be updated with the latest changes
	 *                      and renamed to the output path.
	 */
	void close(bool databaseSaved = true, const std::string & ouputDatabasePath = "");

	/** @return Working directory used for dumps, log files and temporary outputs. */
	const std::string & getWorkingDir() const {return _wDir;}
	/** @return True if RGB-D SLAM mode is enabled (@ref Parameters::kRGBDEnabled()). */
	bool isRGBDMode() const { return _rgbdSlamMode; }
	/** @return Id of the loop-closure hypothesis accepted at the last @ref process() iteration, or 0 if none. */
	int getLoopClosureId() const {return _loopClosureHypothesis.first;}
	/** @return Posterior probability of the accepted loop-closure hypothesis, or 0 if none. */
	float getLoopClosureValue() const {return _loopClosureHypothesis.second;}
	/** @return Id of the highest-posterior hypothesis at the last iteration (whether or not it was accepted). */
	int getHighestHypothesisId() const {return _highestHypothesis.first;}
	/** @return Posterior of the highest-posterior hypothesis at the last iteration. */
	float getHighestHypothesisValue() const {return _highestHypothesis.second;}
	/** @return Id of the last non-intermediate signature added to the map (0 if none). */
	int getLastLocationId() const;
	/** @return Working memory ids ordered as in @ref Memory::getWorkingMem(). */
	std::list<int> getWM() const;
	/** @return Short-term memory ids. */
	std::set<int> getSTM() const;
	/** @return Working memory size (number of WM signatures). */
	int getWMSize() const;
	/** @return Short-term memory size (number of STM signatures). */
	int getSTMSize() const;
	/** @return Per-signature weights (rehearsal counts) for WM and STM. */
	std::map<int, int> getWeights() const;
	/** @return Total number of signatures across WM, STM and LTM. */
	int getTotalMemSize() const;
	/** @return Wall-clock duration of the last @ref process() call, in seconds. */
	double getLastProcessTime() const {return _lastProcessTime;};
	/** @return True if @p locationId is currently in short-term memory. */
	bool isInSTM(int locationId) const;
	/** @return True if signature ids are auto-generated (vs. taken from @ref SensorData::id()). */
	bool isIDsGenerated() const;
	/** @return Statistics produced by the last @ref process() iteration. */
	const Statistics & getStatistics() const;
	/** @return Optimized poses of the current local map (last graph optimization result). */
	const std::map<int, Transform> & getLocalOptimizedPoses() const {return _optimizedPoses;}
	/** @return Constraints (links) of the current local map. */
	const std::multimap<int, Link> & getLocalConstraints() const {return _constraints;}
	/**
	 * @return Optimized pose of @p locationId in the current local map (identity if not present).
	 */
	Transform getPose(int locationId) const;
	/**
	 * @return Transform mapping odometry frame to the optimized map frame.
	 *
	 * This is the correction applied to incoming odometry poses so they align with the
	 * latest graph optimization output. Updated whenever a loop closure or proximity
	 * detection re-optimizes the graph.
	 *
	 * In ROS terms, this corresponds to the standard @c /map -> @c /odom TF transform
	 * published by SLAM systems: composing it with the live odometry pose
	 * (@c /odom -> @c /base_link) yields the robot pose in the map frame.
	 */
	Transform getMapCorrection() const {return _mapCorrection;}
	/** @return Owned @ref Memory (may be null before @ref init()). */
	const Memory * getMemory() const {return _memory;}
	/** @return Radius (meters) under which the current path goal is considered reached. */
	float getGoalReachedRadius() const {return _goalReachedRadius;}
	/** @return Local radius (meters) used by proximity detection and path planning queries. */
	float getLocalRadius() const {return _localRadius;}
	/**
	 * @return Last localized pose in the map frame.
	 *
	 * In **localization mode**, this is the corrected odometry pose of the last
	 * processed frame. In **mapping mode**, this is the last pose returned by
	 * @ref getLocalOptimizedPoses().
	 */
	const Transform & getLastLocalizationPose() const {return _lastLocalizationPose;}

	/**
	 * @return Maximum allowed processing time per @ref process() call, in milliseconds.
	 * @see Parameters::kRtabmapTimeThr()
	 */
	float getTimeThreshold() const {return _maxTimeAllowed;}
	/**
	 * @brief Sets the per-iteration time budget (ms).
	 *
	 * Drives how aggressively WM is transferred to LTM to keep iterations under the
	 * threshold. 0 disables the time bound.
	 *
	 * @note This setting and @ref setMemoryThreshold() are the only two switches that
	 *       enable RTAB-Map's memory management (WM-to-LTM transfer, retrieval and node
	 *       immunization). When both are 0, memory management is disabled and all
	 *       signatures stay in working memory.
	 *
	 * @see Parameters::kRtabmapTimeThr()
	 */
	void setTimeThreshold(float maxTimeAllowed);
	/**
	 * @return Maximum allowed WM size (number of signatures).
	 * @see Parameters::kRtabmapMemoryThr()
	 */
	int getMemoryThreshold() const {return _maxMemoryAllowed;}
	/**
	 * @brief Sets the maximum number of signatures kept in WM (0 = unbounded).
	 *
	 * @note This setting and @ref setTimeThreshold() are the only two switches that
	 *       enable RTAB-Map's memory management (WM-to-LTM transfer, retrieval and node
	 *       immunization). When both are 0, memory management is disabled and all
	 *       signatures stay in working memory.
	 *
	 * @see Parameters::kRtabmapMemoryThr()
	 */
	void setMemoryThreshold(int maxMemoryAllowed);

	/**
	 * @brief Sets the localization prior pose used to seed the next @ref process() call
	 *        (localization mode only).
	 *
	 * Tells RTAB-Map where the robot is currently located in the map frame, so that the
	 * very next call to @ref process() can align incoming odometry with the persisted
	 * map without waiting for a loop closure. Typical use cases: restoring localization
	 * after a session restart, applying an external pose estimate (e.g. from a GPS or
	 * a known starting point), or recovering from "kidnapped robot" situations.
	 *
	 * This call only stages state; it does **not** itself produce a non-identity
	 * @ref getMapCorrection(). The alignment between the odometry frame and the map
	 * frame is performed on the next @ref process() call, which consumes
	 * @p initialPose together with the incoming odometry pose. Two branches are taken
	 * depending on @ref Parameters::kRGBDOptimizeFromGraphEnd():
	 * - **false (default)**: @ref getMapCorrection() is set so that the live odometry
	 *   pose is shifted to land on @p initialPose in the map frame (the optimized map
	 *   is left untouched).
	 * - **true**: every optimized node pose is rigidly transformed so that the map
	 *   itself moves to align with @p initialPose (the map correction stays close to
	 *   identity).
	 *
	 * The transform applied is restricted by SLAM dimensionality: 3-DoF (x, y, yaw)
	 * for 2D SLAM, 4-DoF (x, y, z, yaw) when gravity is available
	 * (IMU orientation or @ref Memory::isOdomGravityUsed()) and
	 * @ref Parameters::kOptimizerGravitySigma() is non-zero, full 6-DoF otherwise.
	 *
	 * Side effects on the staged state:
	 * - @ref getLastLocalizationPose() is replaced by @p initialPose; the localization
	 *   covariance, the last localization node id and the odometry cache used for
	 *   loop-closure rejection are all cleared.
	 * - @ref getMapCorrection() is reset to identity and any backup is cleared.
	 * - If the current map has not been optimized yet (no entries in
	 *   @ref getLocalOptimizedPoses()) and a last working signature exists, the map
	 *   is optimized around that signature so the next @ref process() has something
	 *   to localize against.
	 *
	 * After the next @ref process() consumes the prior, the nearest optimized node to
	 * @p initialPose is recorded as the last localization node.
	 *
	 * @param initialPose Robot pose in the map frame.
	 *
	 * @note No-op (with warning) in mapping mode.
	 *
	 * @see Parameters::kMemIncrementalMemory()
	 * @see Parameters::kRGBDOptimizeFromGraphEnd()
	 */
	void setInitialPose(const Transform & initialPose);
	/**
	 * @brief Starts a new map session (next @ref process() will create a fresh map id).
	 *
	 * In **mapping mode**, this increments the map id, clears the local optimized graph
	 * and resets the Bayes filter.
	 * In **localization mode**, it resets the map correction, the localization node and
	 * the odometry cache; if @ref Parameters::kRtabmapRestartAtOrigin() is enabled, the
	 * last localization pose is reset to identity.
	 *
	 * @return The new map id (mapping mode), or -1 (localization mode).
	 */
	int triggerNewMap();
	/**
	 * @brief Assigns or clears a label on signature @p id.
	 * @return True if the label was applied.
	 */
	bool labelLocation(int id, const std::string & label);
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
	 * @return True if the data was attached.
	 */
	bool setUserData(int id, const cv::Mat & data);
	/**
	 * @brief Writes a Graphviz DOT file of the pose graph.
	 * @param path Output file path.
	 * @param id If non-zero, root the graph at @p id; otherwise use the last signature.
	 * @param margin Maximum graph depth around @p id to include.
	 */
	void generateDOTGraph(const std::string & path, int id=0, int margin=5);
	/**
	 * @brief Exports the current pose graph to a text file.
	 *
	 * Forwards to @ref graph::exportPoses() after collecting either the optimized or
	 * the raw odometry poses (and the matching constraints when needed).
	 *
	 * @param path Output file path.
	 * @param optimized If true, export optimized poses; otherwise raw odometry poses.
	 * @param global If true, include nodes from LTM as well; otherwise only WM/STM.
	 * @param format Output format code; see @ref graph::exportPoses() for the full
	 *        list of supported values (raw, RGBD-SLAM/TUM, KITTI, TORO, g2o, ...).
	 *
	 * @see graph::exportPoses()
	 */
	void exportPoses(
			const std::string & path,
			bool optimized,
			bool global,
			int format
	);
	/**
	 * @brief Clears all in-memory state and resets the database.
	 *
	 * In incremental mode, also clears the persisted map. In read-only memory mode,
	 * resets the in-memory state but leaves the database untouched.
	 */
	void resetMemory();
	/** @brief Dumps the Bayes-filter prediction matrix to a file in the working directory. */
	void dumpPrediction() const;
	/** @brief Dumps the @ref Memory state (signatures, words, dictionary) to the working directory. */
	void dumpData() const;
	/**
	 * @brief Re-parses parameters and propagates them to owned sub-objects
	 *        (@ref Memory, @ref BayesFilter, @ref Optimizer, ...).
	 */
	void parseParameters(const ParametersMap & parameters);
	/** @return Current effective parameter map. */
	const ParametersMap & getParameters() const {return _parameters;}
	/**
	 * @brief Sets the working directory used for dumps, logs and temporary files.
	 *
	 * Can also be configured through @ref Parameters::kRtabmapWorkingDirectory() in the
	 * parameter map passed to @ref init() or @ref parseParameters().
	 *
	 * @see Parameters::kRtabmapWorkingDirectory()
	 */
	void setWorkingDirectory(std::string path);
	/**
	 * @brief Removes the loop-closure link added at the last @ref process() iteration.
	 *
	 * Looks at the last non-intermediate signature in STM and erases any
	 * @ref Link::kGlobalClosure, @ref Link::kLocalSpaceClosure, @ref Link::kLocalTimeClosure
	 * or @ref Link::kUserClosure attached to it. The current optimized map is updated
	 * accordingly.
	 */
	void rejectLastLoopClosure();
	/**
	 * @brief Deletes the most recent (non-intermediate) location from the map.
	 *
	 * Used by tools to undo the very last @ref process() iteration. In mapping mode,
	 * the optimized graph is recomputed without the deleted node.
	 *
	 * @note Locations whose neighbors include intermediate nodes are not supported.
	 */
	void deleteLastLocation();
	/**
	 * @brief Replaces the current optimized poses and constraints with externally
	 *        provided ones.
	 *
	 * Useful when graph optimization is performed outside of Rtabmap.
	 *
	 * @warning No consistency check is performed against the current @ref Memory state:
	 *          @p poses and @p constraints overwrite the internal containers verbatim.
	 *          The caller is responsible for ensuring that every id in @p poses (and
	 *          every endpoint of every link in @p constraints) belongs to a signature
	 *          currently in STM or WM (see @ref Memory::isInSTM() / @ref Memory::isInWM()).
	 *          Passing poses for ids that are no longer loaded will leave dangling
	 *          entries that may confuse subsequent @ref process() calls.
	 */
	void setOptimizedPoses(const std::map<int, Transform> & poses, const std::multimap<int, Link> & constraints);
	/**
	 * @brief Returns a copy of signature @p id with optional payloads attached.
	 *
	 * Loads from WM/STM if present, otherwise from LTM. Selectively populates the
	 * returned @ref Signature with images, scan, user data, occupancy grid, visual
	 * words and global descriptors.
	 */
	Signature getSignatureCopy(int id, bool images, bool scan, bool userData, bool occupancyGrid, bool withWords, bool withGlobalDescriptors) const;
	/**
	 * @brief Deprecated: use @ref getGraph() instead with @c withImages=true,
	 *        @c withScan=true, @c withUserData=true and @c withGrid=true.
	 */
	RTABMAP_DEPRECATED
		void get3DMap(std::map<int, Signature> & signatures,
				std::map<int, Transform> & poses,
				std::multimap<int, Link> & constraints,
				bool optimized,
				bool global) const;
	/**
	 * @brief Extracts a full snapshot of the current pose graph.
	 *
	 * @param poses Output: pose for every selected node.
	 * @param constraints Output: links between selected nodes.
	 * @param optimized If true, return optimized poses; otherwise raw odometry poses.
	 * @param global If true, include nodes from LTM as well; otherwise only WM/STM.
	 * @param signatures Optional output: a copy of each node's @ref Signature (with the
	 *        payloads requested by the @p with* flags).
	 * @param withImages Attach compressed RGB/depth images to @p signatures.
	 * @param withScan Attach laser scan blob.
	 * @param withUserData Attach user data blob.
	 * @param withGrid Attach occupancy grid cells.
	 * @param withWords Attach visual words (id, keypoints, 3D points, descriptors).
	 * @param withGlobalDescriptors Attach global descriptors.
	 */
	void getGraph(std::map<int, Transform> & poses,
			std::multimap<int, Link> & constraints,
			bool optimized,
			bool global,
			std::map<int, Signature> * signatures = 0,
			bool withImages = false,
			bool withScan = false,
			bool withUserData = false,
			bool withGrid = false,
			bool withWords = true,
			bool withGlobalDescriptors = true) const;
	/**
	 * @brief Returns optimized poses within a metric radius of @p pose.
	 *
	 * @param pose Query pose in the map frame.
	 * @param radius Search radius in meters (0 falls back to @ref Parameters::kRGBDLocalRadius()).
	 * @param k If non-zero, also cap the result to the @p k nearest neighbors.
	 * @param distsSqr Optional output: per-id squared distance to @p pose.
	 * @return Nodes (and possibly landmarks) within the radius, mapped to their pose.
	 *         Landmarks have a negative id.
	 */
	std::map<int, Transform> getNodesInRadius(const Transform & pose, float radius, int k=0, std::map<int, float> * distsSqr=0);
	/**
	 * @brief Returns optimized poses within a metric radius of node @p nodeId.
	 *
	 * @param nodeId Query node id. Pass 0 to query around the latest node. A negative
	 *        id requests neighbors of the corresponding landmark.
	 * @param radius Search radius in meters (0 falls back to @ref Parameters::kRGBDLocalRadius()).
	 * @param k If non-zero, cap the result to the @p k nearest neighbors.
	 * @param distsSqr Optional output: per-id squared distance to @p nodeId.
	 * @return Nodes (and possibly landmarks) within the radius.
	 */
	std::map<int, Transform> getNodesInRadius(int nodeId, float radius, int k=0, std::map<int, float> * distsSqr=0);
	/**
	 * @brief Post-processing: searches for additional loop closures over the existing graph.
	 *
	 * Clusters nearby optimized poses and runs registration between candidates that are
	 * not yet linked. New links are added to @ref Memory and the graph is re-optimized.
	 *
	 * @note The registration approach used here is the one configured in @ref Memory via
	 *       @ref Parameters::kRegStrategy() (0=Vis, 1=Icp, 2=VisIcp), so the quality and
	 *       sensor requirements of this pass mirror the live loop-closure pipeline.
	 *
	 * @note Candidate cluster pairs whose ids differ by less than
	 *       @ref Parameters::kMemSTMSize(), or that are already reachable from each
	 *       other within that many graph hops, are filtered out. This prevents trivial
	 *       "loop closures" between temporally or topologically adjacent nodes.
	 *
	 * @param clusterRadiusMax Maximum metric distance (m) between two candidate nodes.
	 * @param clusterAngle Maximum angular distance (rad) between two candidate nodes.
	 * @param iterations Number of refinement passes.
	 * @param intraSession Include loop closures within the same map session.
	 * @param interSession Include loop closures between different map sessions.
	 * @param state Optional progress sink; cancellation requests are honored.
	 * @param clusterRadiusMin Minimum metric distance (m); pairs closer than this are
	 *        considered already linked through neighbor links.
	 * @param toFromMapId If >=0, restrict candidate pairs to nodes belonging to this map id.
	 * @return Number of loop closures added, or -1 on error
	 *         (e.g. not in RGB-D mode, no optimizer iterations).
	 */
	int detectMoreLoopClosures(
			float clusterRadiusMax = 0.5f,
			float clusterAngle = M_PI/6.0f,
			int iterations = 1,
			bool intraSession = true,
			bool interSession = true,
			const ProgressState * state = 0,
			float clusterRadiusMin = 0.0f,
			int toFromMapId = -1);
	/**
	 * @brief Runs a global bundle adjustment over the optimized graph.
	 *
	 * @param optimizerType Backend optimizer (e.g. 1=g2o); availability depends on
	 *        what RTAB-Map was built with.
	 * @param rematchFeatures If true, re-match visual features between connected nodes
	 *        before BA (otherwise reuse existing word-id correspondences).
	 * @param iterations Solver iterations (0 falls back to @ref Parameters::kOptimizerIterations()).
	 * @param pixelVariance Pixel reprojection variance used by the cost (0 falls back
	 *        to @ref Parameters::kg2oPixelVariance()).
	 * @return True if BA was run and improved poses were stored.
	 */
	bool globalBundleAdjustment(
			int optimizerType = 1 /*g2o*/,
			bool rematchFeatures = true,
			int iterations = 0,
			float pixelVariance = 0.0f);
	/**
	 * @brief Filters spurious obstacles from every node's local grid using a reference 2D map.
	 *
	 * Thin wrapper around @ref Memory::cleanupLocalGrids(); see that method for the
	 * exact filtering rule and the meaning of @p cropRadius and @p filterScans.
	 *
	 * @return Number of (node, grid or scan) modifications, or -1 on error.
	 */
	int cleanupLocalGrids(
			const std::map<int, Transform> & mapPoses,
			const cv::Mat & map,
			float xMin,
			float yMin,
			float cellSize,
			int cropRadius = 1,
			bool filterScans = false);
	/**
	 * @brief Re-runs registration on every link of the current graph and updates the
	 *        ones that converge.
	 *
	 * Useful after parameter changes to refresh stored transforms.
	 *
	 * @note The registration approach is the one configured in @ref Memory via
	 *       @ref Parameters::kRegStrategy() (0=Vis, 1=Icp, 2=VisIcp). For each link,
	 *       the link's existing relative transform (the constraint produced by the
	 *       current optimized local graph) is passed as the initial guess to
	 *       @ref Memory::computeTransform(), so links already close to convergence
	 *       are refined locally rather than re-estimated from scratch.
	 *
	 * @return Number of links refined, or -1 if not in RGB-D mode.
	 */
	int refineLinks();
	/**
	 * @brief Adds an external link to the map.
	 *
	 * The link's "from" and "to" endpoints must exist in memory (incremental mode) or
	 * in the optimized poses (localization mode). RGB-D mode only.
	 *
	 * @return True if the link was added.
	 */
	bool addLink(const Link & link);
	/**
	 * @brief Converts an odometry covariance into an information matrix, clipping by
	 *        @ref Memory::getOdomMaxInf() when @ref Parameters::kRGBDLoopCovLimited()
	 *        is enabled.
	 */
	cv::Mat getInformation(const cv::Mat & covariance) const;
	/**
	 * @brief Marks node ids whose data should be re-emitted on the next @ref process().
	 *
	 * The requested signatures are attached to the @ref Statistics object produced by
	 * the next @ref process() call (via the same mechanism as the regular "last signature
	 * data"), so consumers reading @ref getStatistics() pick them up alongside the
	 * normal output. Up to @ref Parameters::kRtabmapMaxRepublished() ids are emitted
	 * per iteration; any leftover ids stay queued for subsequent iterations until they
	 * are republished or fall out of the current graph.
	 *
	 * Pass an empty vector to clear the request set. Requires
	 * @ref Parameters::kRtabmapMaxRepublished() > 0 and
	 * @ref Parameters::kRtabmapPublishLastSignature() = true.
	 */
	void addNodesToRepublish(const std::vector<int> & ids);

	/** @return Current path status: -1 = failed, 0 = idle / executing, 1 = success. */
	int getPathStatus() const {return _pathStatus;}
	/**
	 * @brief Clears the current path and sets its terminal status.
	 * @param status -1 = failed, 0 = idle / executing, 1 = success.
	 */
	void clearPath(int status);
	/**
	 * @brief Plans a path from the current location to node @p targetNode.
	 *
	 * RGB-D mode only (requires @ref Parameters::kRGBDEnabled() = true).
	 *
	 * @param targetNode Destination node id (positive) or landmark id (negative).
	 * @param global If true, also search nodes in LTM; otherwise only the current
	 *        optimized map.
	 * @return True if a path was computed; the result is available via @ref getPath().
	 *
	 * @see Parameters::kRGBDEnabled()
	 */
	bool computePath(int targetNode, bool global);
	/**
	 * @brief Plans a path in the current optimized map toward a metric goal pose.
	 *
	 * @param targetPose Goal pose in the map frame.
	 * @param tolerance Goal-acceptance tolerance (meters). A negative value falls back
	 *        to @ref Parameters::kRGBDLocalRadius(); 0 means infinite tolerance.
	 * @return True if a path was computed.
	 */
	bool computePath(const Transform & targetPose, float tolerance = -1.0f);
	/** @return The currently planned path as a sequence of (node id, pose) waypoints. */
	const std::vector<std::pair<int, Transform> > & getPath() const {return _path;}
	/** @return Upcoming waypoints (from the current path index onward). */
	std::vector<std::pair<int, Transform> > getPathNextPoses() const;
	/** @return Upcoming node ids (from the current path index onward). */
	std::vector<int> getPathNextNodes() const;
	/** @return Id of the current intermediate path goal (the node currently being chased). */
	int getPathCurrentGoalId() const;
	/** @return Index of the current waypoint in @ref getPath(). */
	unsigned int getPathCurrentIndex() const {return _pathCurrentIndex;}
	/** @return Index of the current intermediate goal in @ref getPath(). */
	unsigned int getPathCurrentGoalIndex() const {return _pathGoalIndex;}
	/** @return Transform from the final waypoint pose to the requested goal pose. */
	const Transform & getPathTransformToGoal() const {return _pathTransformToGoal;}

	/**
	 * @brief Returns optimized poses of WM nodes located in front of @p fromId.
	 *
	 * Candidates are first gathered around @p fromId, then STM nodes are excluded, the
	 * survivors are cropped to a forward-facing box of width @p radius (1 m behind,
	 * @p radius ahead, +/-@p radius laterally), and a KdTree radius search keeps the
	 * @p maxNearestNeighbors closest poses in that box.
	 *
	 * @note Mapping vs. localization mode differs only in how the initial candidate
	 *       set is built:
	 *       - In **mapping mode** (incremental), candidates are produced by a
	 *         graph-radius walk from @p fromId: nodes reachable within @p maxDiffID
	 *         graph hops AND within @p radius meters in the optimized poses.
	 *       - In **localization mode**, the graph-hop restriction is ignored: every
	 *         optimized pose within @p radius meters of @p fromId is considered.
	 *       The forward-box crop and KdTree radius search that follow are identical
	 *       in both modes.
	 *
	 * @param fromId Reference node (must be in @ref Memory and @ref getLocalOptimizedPoses()).
	 * @param maxNearestNeighbors Cap on the number of nodes returned.
	 * @param radius Maximum metric distance from @p fromId (meters).
	 * @param maxDiffID Maximum graph depth from @p fromId in mapping mode (0 = unlimited).
	 *        Ignored in localization mode.
	 */
	std::map<int, Transform> getForwardWMPoses(int fromId, int maxNearestNeighbors, float radius, int maxDiffID) const;
	/**
	 * @brief Segments a set of optimized poses into paths connected by neighbor links.
	 *
	 * Designed to be called on the result of a radius search around @p target (a set
	 * of @p poses already constrained to be metrically close to the goal). Within that
	 * radius, the method partitions the @p poses into one or more "paths" where each
	 * path is a connected component reachable from its starting node using **only
	 * neighbor (sequential) links** -- loop-closure links, landmark links and
	 * intermediate nodes are not used to traverse between members. Paths are produced
	 * one at a time, each starting from the still-unclaimed pose nearest to @p target;
	 * a candidate is added to the current path only if it has at least one neighbor
	 * link to a node already in the path.
	 *
	 * Used internally by proximity detection by space (see
	 * @ref Parameters::kRGBDProximityBySpace()) in two independent stages, each
	 * iterating over the segmented paths:
	 * - **One-to-one** (visual registration): runs registration between the current
	 *   node and at most one node per neighbor-connected path, avoiding redundant
	 *   attempts against nearby members of the same local trajectory.
	 * - **One-to-many** (scan matching, enabled when
	 *   @ref Parameters::kRGBDProximityPathMaxNeighbors() > 0): on each path,
	 *   neighboring nodes are assembled around the nearest pose on the path (up to
	 *   the configured count, walked forward and backward) and their laser scans are
	 *   merged for an ICP registration against the current scan. The
	 *   neighbor-link-only structure of each path is what makes this assembly
	 *   geometrically consistent.
	 *
	 * @param poses Candidate nodes with their optimized poses (typically pre-filtered
	 *        to a radius around @p target).
	 * @param target Reference pose used to order paths: each path's starting node is
	 *        the still-unclaimed pose closest to @p target.
	 * @param maxGraphDepth Maximum graph depth traversed from the starting node when
	 *        gathering candidates for a path (0 = unlimited).
	 * @return Map from the starting node id of each path to its (node id -> pose) chain.
	 */
	std::map<int, std::map<int, Transform> > getPaths(const std::map<int, Transform> & poses, const Transform & target, int maxGraphDepth = 0) const;
	/**
	 * @brief Applies the standard RTAB-Map likelihood adjustment.
	 *
	 * Normalizes raw likelihoods using mean and standard deviation across non-null
	 * values. Real-place entries with @c value <= @c mean + @c stdDev are clamped to
	 * @c 1.0; only entries above that threshold are scaled. The virtual place (the
	 * first key in @p likelihood, representing the "new place" hypothesis) is then
	 * set so that its likelihood reflects how peaked the real distribution is.
	 *
	 * The exact formulas are selected by @ref Parameters::kRtabmapVirtualPlaceLikelihoodRatio()
	 * (default 0, Angeli PhD formulation):
	 *
	 * - **Ratio = 0** (mean / std-dev formulation):
	 *   - Real place above threshold: @c (value - (stdDev - epsilon)) / mean
	 *   - Virtual place: @c mean / stdDev + 1 (when @c stdDev is non-trivial and a
	 *     maximum exists; otherwise 2).
	 *   The virtual place "wins" when the real-place distribution is flat
	 *   (small @c stdDev relative to @c mean).
	 *
	 * - **Ratio != 0** (z-score formulation):
	 *   - Real place above threshold: @c (value - mean) / stdDev (i.e. the z-score).
	 *   - Virtual place: @c stdDev / (max - mean) + 1 (when @c max > @c mean;
	 *     otherwise 2). The virtual place "wins" when no real candidate stands out
	 *     far above the mean.
	 *
	 * In both formulations a low virtual-place likelihood favors a real-place loop
	 * closure on the next Bayes update; a high one favors the "new place" hypothesis.
	 *
	 * @see Parameters::kRtabmapVirtualPlaceLikelihoodRatio()
	 */
	void adjustLikelihood(std::map<int, float> & likelihood) const;

private:
	void optimizeCurrentMap(int id,
			bool lookInDatabase,
			std::map<int, Transform> & optimizedPoses,
			cv::Mat & covariance,
			std::multimap<int, Link> * constraints = 0,
			double * error = 0,
			int * iterationsDone = 0) const;
	std::map<int, Transform> optimizeGraph(
			int fromId,
			const std::set<int> & ids,
			const std::map<int, Transform> & guessPoses,
			bool lookInDatabase,
			cv::Mat & covariance,
			std::multimap<int, Link> * constraints = 0,
			double * error = 0,
			int * iterationsDone = 0) const;
	std::list<std::pair<int, int> > repairGraph(
		graph::MaxGraphErrors & maxGraphErrors,
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		double & optimizationError,
		int & optimizationIterations,
		cv::Mat & optimizationCovariance);
	void updateGoalIndex();
	bool computePath(int targetNode, std::map<int, Transform> nodes, const std::multimap<int, rtabmap::Link> & constraints);

	void createGlobalScanMap();

	void setupLogFiles(bool overwrite = false);
	void flushStatisticLogs();

private:
	// Modifiable parameters
	bool _publishStats;
	bool _publishLastSignatureData;
	bool _publishPdf;
	bool _publishLikelihood;
	bool _publishRAMUsage;
	bool _computeRMSE;
	bool _saveWMState;
	float _maxTimeAllowed;          ///< Per-iteration time budget (ms).
	unsigned int _maxMemoryAllowed; ///< Maximum number of signatures kept in WM.
	float _loopThr;
	float _loopRatio;
	float _aggressiveLoopThr;
	int _virtualPlaceLikelihoodRatio;
	float _maxLoopClosureDistance;
	bool _verifyLoopClosureHypothesis;
	unsigned int _maxRetrieved;
	unsigned int _maxLocalRetrieved;
	unsigned int _maxRepublished;
	bool _rawDataKept;
	bool _statisticLogsBufferedInRAM;
	bool _statisticLogged;
	bool _statisticLoggedHeaders;
	bool _rgbdSlamMode;
	float _rgbdLinearUpdate;
	float _rgbdAngularUpdate;
	float _rgbdLinearSpeedUpdate;
	float _rgbdAngularSpeedUpdate;
	float _newMapOdomChangeDistance;
	bool _neighborLinkRefining;
	bool _proximityByTime;
	bool _proximityBySpace;
	bool _scanMatchingIdsSavedInLinks;
	bool _loopClosureIdentityGuess;
	float _localRadius;
	float _localImmunizationRatio;
	int _proximityMaxGraphDepth;
	int _proximityMaxPaths;
	int _proximityMaxNeighbors;
	float _proximityFilteringRadius;
	bool _proximityRawPosesUsed;
	float _proximityAngle;
	bool _proximityOdomGuess;
	double _proximityMergedScanCovFactor;
	std::string _databasePath;
	bool _optimizeFromGraphEnd;
	float _optimizationMaxError;
	float _optimizationMaxErrorRepairRadius;
	bool _startNewMapOnLoopClosure;
	bool _startNewMapOnGoodSignature;
	float _goalReachedRadius;       ///< Path-goal acceptance radius (meters).
	bool _goalsSavedInUserData;
	int _pathStuckIterations;
	float _pathLinearVelocity;
	float _pathAngularVelocity;
	bool _forceOdom3doF;
	bool _restartAtOrigin;
	bool _loopCovLimited;
	bool _loopGPS;
	int _maxOdomCacheSize;
	bool _localizationSmoothing;
	double _localizationPriorInf;
	bool _localizationSecondTryWithoutProximityLinks;
	bool _createGlobalScanMap;
	float _markerPriorsLinearVariance;
	float _markerPriorsAngularVariance;

	std::pair<int, float> _loopClosureHypothesis;
	std::pair<int, float> _highestHypothesis;
	double _lastProcessTime;
	bool _someNodesHaveBeenTransferred;
	float _distanceTravelled;
	float _distanceTravelledSinceLastLocalization;
	bool _optimizeFromGraphEndChanged;

	// Abstract classes containing all loop closure
	// strategies for a type of signature or configuration.
	EpipolarGeometry * _epipolarGeometry;
	BayesFilter * _bayesFilter;
	Optimizer * _graphOptimizer;
	ParametersMap _parameters;

	Memory * _memory;

	FILE* _foutFloat;
	FILE* _foutInt;
	std::list<std::string> _bufferedLogsF;
	std::list<std::string> _bufferedLogsI;

	Statistics statistics_;

	std::string _wDir;

	std::map<int, Transform> _optimizedPoses;
	std::multimap<int, Link> _constraints;
	Transform _mapCorrection;
	Transform _mapCorrectionBackup;     ///< Used in localization mode when odometry is lost.
	Transform _lastLocalizationPose;    ///< Corrected odometry pose; in mapping mode, last pose of getLocalOptimizedPoses().
	int _lastLocalizationNodeId;        ///< Last localization node id (localization mode).
	cv::Mat _localizationCovariance;
	std::map<int, std::pair<cv::Point3d, Transform> > _gpsGeocentricCache;
	bool _currentSessionHasGPS;
	LaserScan _globalScanMap;
	std::map<int, Transform> _globalScanMapPoses;
	std::map<int, Transform> _odomCachePoses;       ///< Odometry cache used to reject loop closures (localization mode).
	std::multimap<int, Link> _odomCacheConstraints; ///< Odometry cache constraints (localization mode).
	std::map<int, Transform> _markerPriors;
	std::pair<int, int> _lastRejectedLoopClosureIds;

	std::set<int> _nodesToRepublish;

	// Planning stuff
	int _pathStatus;
	std::vector<std::pair<int,Transform> > _path;
	std::set<unsigned int> _pathUnreachableNodes;
	unsigned int _pathCurrentIndex;
	unsigned int _pathGoalIndex;
	Transform _pathTransformToGoal;
	int _pathStuckCount;
	float _pathStuckDistance;

#ifdef RTABMAP_PYTHON
	PythonInterface * _python;
#endif

};

} // namespace rtabmap
#endif /* RTABMAP_H_ */
