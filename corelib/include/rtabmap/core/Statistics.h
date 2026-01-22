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

#ifndef STATISTICS_H_
#define STATISTICS_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <list>
#include <vector>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/utilite/UStl.h>

namespace rtabmap {

/**
 * @def RTABMAP_STATS(PREFIX, NAME, UNIT)
 * @brief Macro to define a statistic with automatic name generation and default initialization
 * 
 * This macro generates:
 * - A static method `k##PREFIX##NAME()` that returns the statistic name in the format "PREFIX/NAME/UNIT"
 * - A dummy class that initializes the default data map with the statistic name and a default value of 0.0
 * 
 * @param PREFIX The group/category prefix (e.g., "Loop", "Timing", "Memory")
 * @param NAME The statistic name (e.g., "Id", "Total", "Working_memory_size")
 * @param UNIT The unit of measurement (e.g., "ms", "m", "deg", or empty string "")
 * 
 * @note The generated static method can be used to get the standardized statistic name:
 * @code
 * std::string statName = Statistics::kTimingTotal(); // Returns "Timing/Total/ms"
 * statistics.addStatistic(statName, 500.0f);
 * @endcode
 * 
 * @note The default value (0.0) is automatically added to the default data map when a Statistics
 *       object is first constructed.
 */
#define RTABMAP_STATS(PREFIX, NAME, UNIT) \
	public: \
		static std::string k##PREFIX##NAME() {return #PREFIX "/" #NAME "/" #UNIT;} \
	private: \
		class Dummy##PREFIX##NAME { \
		public: \
			Dummy##PREFIX##NAME() {if(!_defaultDataInitialized)_defaultData.insert(std::pair<std::string, float>(#PREFIX "/" #NAME "/" #UNIT, 0.0f));} \
		}; \
		Dummy##PREFIX##NAME dummy##PREFIX##NAME

/**
 * @class Statistics
 * @brief Collects and manages runtime statistics for RTAB-Map
 * 
 * The Statistics class provides a comprehensive system for collecting, storing, and managing
 * various runtime statistics about RTAB-Map's operation. Statistics are organized into groups
 * such as:
 * - **Loop**: Loop closure detection, hypothesis validation, optimization errors
 * - **Proximity**: Proximity-based detection statistics
 * - **Memory**: Working memory size, database usage, signature management
 * - **Timing**: Performance measurements for various operations
 * - **Keypoint**: Visual word dictionary and feature statistics
 * - **Gt**: Ground truth comparison statistics
 * 
 * Statistics are stored in a map with keys in the format "Group/Name/Unit" (e.g., "Timing/Total time/ms").
 * This hierarchical naming allows for easy categorization and plotting.
 * 
 * The class supports two modes:
 * - **Basic mode** (`extended() == false`): Only stores loop closure and last signature ID fields
 * - **Extended mode** (`extended() == true`): Stores all available statistics including poses,
 *   constraints, likelihoods, posteriors, and detailed timing information
 * 
 * Statistics can be serialized to/from strings for storage in databases or transmission over networks.
 * 
 * @note Statistics are typically created by RTAB-Map's core components (Rtabmap, Memory, etc.)
 *       and can be retrieved for analysis, visualization, or debugging purposes.
 * 
 */
class RTABMAP_CORE_EXPORT Statistics
{
	RTABMAP_STATS(Loop, Id,); // Combined loop or proximity detection
	RTABMAP_STATS(Loop, RejectedHypothesis,);
	RTABMAP_STATS(Loop, Accepted_hypothesis_id,);
	RTABMAP_STATS(Loop, Suppressed_hypothesis_id,);
	RTABMAP_STATS(Loop, Highest_hypothesis_id,);
	RTABMAP_STATS(Loop, Highest_hypothesis_value,);
	RTABMAP_STATS(Loop, Vp_hypothesis,);
	RTABMAP_STATS(Loop, Reactivate_id,);
	RTABMAP_STATS(Loop, Hypothesis_ratio,);
	RTABMAP_STATS(Loop, Hypothesis_reactivated,);
	RTABMAP_STATS(Loop, Map_id,);
	RTABMAP_STATS(Loop, Visual_words,);
	RTABMAP_STATS(Loop, Visual_inliers,);
	RTABMAP_STATS(Loop, Visual_inliers_ratio,);
	RTABMAP_STATS(Loop, Visual_matches,);
	RTABMAP_STATS(Loop, Visual_variance,);
	RTABMAP_STATS(Loop, Distance_since_last_loc, m);
	RTABMAP_STATS(Loop, Last_id,);
	RTABMAP_STATS(Loop, Optimization_max_error, m);
	RTABMAP_STATS(Loop, Optimization_max_error_ratio, );
	RTABMAP_STATS(Loop, Optimization_max_ang_error, deg);
	RTABMAP_STATS(Loop, Optimization_max_ang_error_ratio, );
	RTABMAP_STATS(Loop, Optimization_error, );
	RTABMAP_STATS(Loop, Optimization_iterations, );
	RTABMAP_STATS(Loop, Linear_variance,);
	RTABMAP_STATS(Loop, Angular_variance,);
	RTABMAP_STATS(Loop, Landmark_detected,);
	RTABMAP_STATS(Loop, Landmark_detected_node_ref,);
	RTABMAP_STATS(Loop, Visual_inliers_mean_dist,m);
	RTABMAP_STATS(Loop, Visual_inliers_distribution,);
	RTABMAP_STATS(Loop, Proximity_links_cleared,);
	//Odom correction
	RTABMAP_STATS(Loop, Odom_correction_norm, m);
	RTABMAP_STATS(Loop, Odom_correction_angle, deg);
	RTABMAP_STATS(Loop, Odom_correction_x, m);
	RTABMAP_STATS(Loop, Odom_correction_y, m);
	RTABMAP_STATS(Loop, Odom_correction_z, m);
	RTABMAP_STATS(Loop, Odom_correction_roll, deg);
	RTABMAP_STATS(Loop, Odom_correction_pitch, deg);
	RTABMAP_STATS(Loop, Odom_correction_yaw, deg);
	// Map to Odom
	RTABMAP_STATS(Loop, MapToOdom_norm, m);
	RTABMAP_STATS(Loop, MapToOdom_angle, deg);
	RTABMAP_STATS(Loop, MapToOdom_x, m);
	RTABMAP_STATS(Loop, MapToOdom_y, m);
	RTABMAP_STATS(Loop, MapToOdom_z, m);
	RTABMAP_STATS(Loop, MapToOdom_roll, deg);
	RTABMAP_STATS(Loop, MapToOdom_pitch, deg);
	RTABMAP_STATS(Loop, MapToOdom_yaw, deg);
	// Map to Base
	RTABMAP_STATS(Loop, MapToBase_x, m);
	RTABMAP_STATS(Loop, MapToBase_y, m);
	RTABMAP_STATS(Loop, MapToBase_z, m);
	RTABMAP_STATS(Loop, MapToBase_roll, deg);
	RTABMAP_STATS(Loop, MapToBase_pitch, deg);
	RTABMAP_STATS(Loop, MapToBase_yaw, deg);
	RTABMAP_STATS(Loop, MapToBase_lin_std, m);
	RTABMAP_STATS(Loop, MapToBase_lin_var, m2);

	RTABMAP_STATS(Proximity, Time_detections,);
	RTABMAP_STATS(Proximity, Space_last_detection_id,);
	RTABMAP_STATS(Proximity, Space_paths,);
	RTABMAP_STATS(Proximity, Space_visual_paths_checked,);
	RTABMAP_STATS(Proximity, Space_scan_paths_checked,);
	RTABMAP_STATS(Proximity, Space_detections_added_visually,);
	RTABMAP_STATS(Proximity, Space_detections_added_icp_multi,);
	RTABMAP_STATS(Proximity, Space_detections_added_icp_global,);

	RTABMAP_STATS(NeighborLinkRefining, Accepted,);
	RTABMAP_STATS(NeighborLinkRefining, Inliers,);
	RTABMAP_STATS(NeighborLinkRefining, ICP_inliers_ratio,);
	RTABMAP_STATS(NeighborLinkRefining, ICP_rotation, rad);
	RTABMAP_STATS(NeighborLinkRefining, ICP_translation, m);
	RTABMAP_STATS(NeighborLinkRefining, ICP_complexity,);
	RTABMAP_STATS(NeighborLinkRefining, Variance,);
	RTABMAP_STATS(NeighborLinkRefining, Pts,);

	RTABMAP_STATS(Memory, Working_memory_size,);
	RTABMAP_STATS(Memory, Short_time_memory_size,);
	RTABMAP_STATS(Memory, Database_memory_used, MB);
	RTABMAP_STATS(Memory, Signatures_removed,);
	RTABMAP_STATS(Memory, Immunized_globally,);
	RTABMAP_STATS(Memory, Immunized_locally,);
	RTABMAP_STATS(Memory, Immunized_locally_max,);
	RTABMAP_STATS(Memory, Signatures_retrieved,);
	RTABMAP_STATS(Memory, Images_buffered,);
	RTABMAP_STATS(Memory, Rehearsal_sim,);
	RTABMAP_STATS(Memory, Rehearsal_id,);
	RTABMAP_STATS(Memory, Rehearsal_merged,);
	RTABMAP_STATS(Memory, Local_graph_size,);
	RTABMAP_STATS(Memory, Odom_cache_poses,);
	RTABMAP_STATS(Memory, Odom_cache_links,);
	RTABMAP_STATS(Memory, Small_movement,);
	RTABMAP_STATS(Memory, Fast_movement,);
	RTABMAP_STATS(Memory, New_landmark,);
	RTABMAP_STATS(Memory, Odometry_variance_ang,);
	RTABMAP_STATS(Memory, Odometry_variance_lin,);
	RTABMAP_STATS(Memory, Distance_travelled, m);
	RTABMAP_STATS(Memory, RAM_usage, MB);
	RTABMAP_STATS(Memory, RAM_estimated, MB);
	RTABMAP_STATS(Memory, Triangulated_points, );
	RTABMAP_STATS(Memory, Closest_node_distance, m);
	RTABMAP_STATS(Memory, Closest_node_angle, rad);

	RTABMAP_STATS(Timing, Memory_update, ms);
	RTABMAP_STATS(Timing, Neighbor_link_refining, ms);
	RTABMAP_STATS(Timing, Proximity_by_time, ms);
	RTABMAP_STATS(Timing, Proximity_by_space_search, ms);
	RTABMAP_STATS(Timing, Proximity_by_space_visual, ms);
	RTABMAP_STATS(Timing, Proximity_by_space, ms);
	RTABMAP_STATS(Timing, Cleaning_neighbors, ms);
	RTABMAP_STATS(Timing, Reactivation, ms);
	RTABMAP_STATS(Timing, Add_loop_closure_link, ms);
	RTABMAP_STATS(Timing, Map_optimization, ms);
	RTABMAP_STATS(Timing, Likelihood_computation, ms);
	RTABMAP_STATS(Timing, Posterior_computation, ms);
	RTABMAP_STATS(Timing, Hypotheses_creation, ms);
	RTABMAP_STATS(Timing, Hypotheses_validation, ms);
	RTABMAP_STATS(Timing, Statistics_creation, ms);
	RTABMAP_STATS(Timing, Memory_cleanup, ms);
	RTABMAP_STATS(Timing, Total, ms);
	RTABMAP_STATS(Timing, Forgetting, ms);
	RTABMAP_STATS(Timing, Joining_trash, ms);
	RTABMAP_STATS(Timing, Emptying_trash, ms);
	RTABMAP_STATS(Timing, Finalizing_statistics, ms);
	RTABMAP_STATS(Timing, RAM_estimation, ms);

	RTABMAP_STATS(TimingMem, Pre_update, ms);
	RTABMAP_STATS(TimingMem, Signature_creation, ms);
	RTABMAP_STATS(TimingMem, Rehearsal, ms);
	RTABMAP_STATS(TimingMem, Keypoints_detection, ms);
	RTABMAP_STATS(TimingMem, Subpixel, ms);
	RTABMAP_STATS(TimingMem, Stereo_correspondences, ms);
	RTABMAP_STATS(TimingMem, Descriptors_extraction, ms);
	RTABMAP_STATS(TimingMem, Rectification, ms);
	RTABMAP_STATS(TimingMem, Keypoints_3D, ms);
	RTABMAP_STATS(TimingMem, Keypoints_3D_motion, ms);
	RTABMAP_STATS(TimingMem, Joining_dictionary_update, ms);
	RTABMAP_STATS(TimingMem, Add_new_words, ms);
	RTABMAP_STATS(TimingMem, Compressing_data, ms);
	RTABMAP_STATS(TimingMem, Post_decimation, ms);
	RTABMAP_STATS(TimingMem, Scan_filtering, ms);
	RTABMAP_STATS(TimingMem, Occupancy_grid, ms);
	RTABMAP_STATS(TimingMem, Markers_detection, ms);

	RTABMAP_STATS(Keypoint, Dictionary_size, words);
	RTABMAP_STATS(Keypoint, Current_frame, words);
	RTABMAP_STATS(Keypoint, Indexed_words, words);
	RTABMAP_STATS(Keypoint, Index_memory_usage, KB);

	RTABMAP_STATS(Gt, Translational_rmse, m);
	RTABMAP_STATS(Gt, Translational_mean, m);
	RTABMAP_STATS(Gt, Translational_median, m);
	RTABMAP_STATS(Gt, Translational_std, m);
	RTABMAP_STATS(Gt, Translational_min, m);
	RTABMAP_STATS(Gt, Translational_max, m);
	RTABMAP_STATS(Gt, Rotational_rmse, deg);
	RTABMAP_STATS(Gt, Rotational_mean, deg);
	RTABMAP_STATS(Gt, Rotational_median, deg);
	RTABMAP_STATS(Gt, Rotational_std, deg);
	RTABMAP_STATS(Gt, Rotational_min, deg);
	RTABMAP_STATS(Gt, Rotational_max, deg);
	RTABMAP_STATS(Gt, Localization_linear_error, m);
	RTABMAP_STATS(Gt, Localization_angular_error, deg);

public:
	/**
	 * @brief Returns the default statistics data map
	 * 
	 * Returns a map containing all predefined statistics with their default values (0.0).
	 * This map is initialized when the first Statistics object is constructed.
	 * 
	 * @return Const reference to the default statistics data map
	 */
	static const std::map<std::string, float> & defaultData();
	
	/**
	 * @brief Serializes a statistics data map to a string
	 * 
	 * Converts a statistics data map into a serialized string format suitable for storage
	 * or transmission. The format is: "key1:value1;key2:value2;..." where values are
	 * formatted as numbers with dots (not commas) as decimal separators (independent of the system's locale).
	 * 
	 * @param data The statistics data map to serialize
	 * @return Serialized string representation of the data
	 * 
	 * @note Empty maps result in empty strings
	 * @see deserializeData()
	 */
	static std::string serializeData(const std::map<std::string, float> & data);
	
	/**
	 * @brief Deserializes a statistics data map from a string
	 * 
	 * Parses a serialized statistics string back into a data map. The string format
	 * should be: "key1:value1;key2:value2;..." as produced by serializeData().
	 * 
	 * @param data The serialized string to parse
	 * @return Deserialized statistics data map
	 * 
	 * @note Invalid entries (missing colons, malformed values) are silently skipped
	 * @see serializeData()
	 */
	static std::map<std::string, float> deserializeData(const std::string & data);

public:
	/**
	 * @brief Default constructor
	 * 
	 * Creates a new Statistics object in basic mode (extended = false).
	 * Initializes all ID fields to 0 or -1, and timestamp to 0.0.
	 * 
	 * @note The first Statistics object constructed will initialize the default data map.
	 */
	Statistics();
	
	/**
	 * @brief Virtual destructor
	 */
	virtual ~Statistics();

	/**
	 * @brief Adds a statistic value to the data map
	 * 
	 * Adds or updates a statistic in the internal data map. The name should follow
	 * the format "Group/Name/Unit" (e.g., "Timing/Total time/ms").
	 * 
	 * @param name The statistic name in the format "Group/Name/Unit"
	 * @param value The statistic value to store
	 * 
	 * @note If a statistic with the same name already exists, it will be overwritten.
	 * @note Use the static methods generated by RTABMAP_STATS() to get standardized names:
	 * @code
	 * statistics.addStatistic(Statistics::kTimingTotal(), 500.0f);
	 * @endcode
	 */
	void addStatistic(const std::string & name, float value);

	// setters
	
	/**
	 * @brief Sets whether extended statistics mode is enabled
	 * 
	 * In basic mode (extended = false), only loop closure and last signature ID fields are filled.
	 * In extended mode (extended = true), all available statistics including poses, constraints,
	 * likelihoods, posteriors, and detailed timing information are stored.
	 * 
	 * @param extended True to enable extended mode, false for basic mode
	 */
	void setExtended(bool extended) {_extended = extended;}
	
	/**
	 * @brief Sets the reference image ID (current/last processed signature ID)
	 * @param id The signature ID
	 */
	void setRefImageId(int id) {_refImageId = id;}
	
	/**
	 * @brief Sets the reference image map ID
	 * @param id The map ID associated with the reference image
	 */
	void setRefImageMapId(int id) {_refImageMapId = id;}
	
	/**
	 * @brief Sets the loop closure detection ID
	 * @param id The signature ID where a loop closure was detected (0 if none)
	 */
	void setLoopClosureId(int id) {_loopClosureId = id;}
	
	/**
	 * @brief Sets the loop closure map ID
	 * @param id The map ID associated with the loop closure
	 */
	void setLoopClosureMapId(int id) {_loopClosureMapId = id;}
	
	/**
	 * @brief Sets the proximity detection ID
	 * @param id The signature ID where proximity was detected (0 if none)
	 */
	void setProximityDetectionId(int id) {_proximiyDetectionId = id;}
	
	/**
	 * @brief Sets the proximity detection map ID
	 * @param id The map ID associated with the proximity detection
	 */
	void setProximityDetectionMapId(int id) {_proximiyDetectionMapId = id;}
	
	/**
	 * @brief Sets the timestamp for these statistics
	 * @param stamp The timestamp (typically in seconds since epoch)
	 */
	void setStamp(double stamp) {_stamp = stamp;}

	/**
	 * @deprecated Use addSignatureData() instead
	 * @brief Sets the last signature data (deprecated)
	 * 
	 * This method is deprecated. Use addSignatureData() instead, which allows
	 * storing multiple signatures.
	 */
	RTABMAP_DEPRECATED void setLastSignatureData(const Signature & data);
	
	/**
	 * @brief Adds signature data to the statistics
	 * 
	 * Adds a signature to the internal signatures data map. Multiple signatures
	 * can be stored, indexed by their ID.
	 * 
	 * @param data The signature to add
	 * 
	 * @note If a signature with the same ID already exists, it will be overwritten.
	 */
	void addSignatureData(const Signature & data) {uInsert(_signaturesData, std::make_pair(data.id(), data));}
	
	/**
	 * @brief Sets all signature data at once
	 * @param data Map of signature IDs to Signature objects
	 */
	void setSignaturesData(const std::map<int, Signature> & data) {_signaturesData = data;}

	/**
	 * @brief Sets the pose graph (node poses)
	 * 
	 * Sets the complete pose graph, mapping node IDs to their Transform poses.
	 * Used in extended mode for visualization and analysis.
	 * 
	 * @param poses Map of node IDs to their poses
	 */
	void setPoses(const std::map<int, Transform> & poses) {_poses = poses;}
	
	/**
	 * @brief Sets the constraint graph (links between nodes)
	 * 
	 * Sets the complete constraint graph, mapping node IDs to their Link constraints.
	 * Used in extended mode for visualization and analysis.
	 * 
	 * @param constraints Multimap of node IDs to their links (a node can have multiple links)
	 */
	void setConstraints(const std::multimap<int, Link> & constraints) {_constraints = constraints;}
	
	/**
	 * @brief Sets the map correction transform
	 * 
	 * The map correction transform represents the transformation from the map fixed frame
	 * to the odometry fixed frame. This transform is typically updated after graph optimization
	 * to transform the odometry pose in map frame.
	 * 
	 * @param mapCorrection The pose of odometry frame in map coordinate system
	 */
	void setMapCorrection(const Transform & mapCorrection) {_mapCorrection = mapCorrection;}
	
	/**
	 * @brief Sets the loop closure transform
	 * 
	 * The transform between the current pose and the loop closure pose.
	 * 
	 * @param loopClosureTransform The loop closure transform
	 */
	void setLoopClosureTransform(const Transform & loopClosureTransform) {_loopClosureTransform = loopClosureTransform;}
	
	/**
	 * @brief Sets the localization covariance matrix
	 * 
	 * The covariance matrix representing the uncertainty in the current localization estimate.
	 * 
	 * @param covariance The covariance matrix (typically 6x6 for 3D pose)
	 */
	void setLocalizationCovariance(const cv::Mat & covariance) {_localizationCovariance = covariance;}
	
	/**
	 * @brief Sets node labels
	 * @param labels Map of node IDs to their string labels
	 */
	void setLabels(const std::map<int, std::string> & labels) {_labels = labels;}
	
	/**
	 * @brief Sets node weights
	 * @param weights Map of node IDs to their integer weights
	 */
	void setWeights(const std::map<int, int> & weights) {_weights = weights;}
	
	/**
	 * @brief Sets posterior probabilities for loop closure hypotheses
	 * 
	 * @param posterior Map of node IDs to their posterior probabilities
	 */
	void setPosterior(const std::map<int, float> & posterior) {_posterior = posterior;}
	
	/**
	 * @brief Sets likelihood values for loop closure hypotheses
	 * 
	 * Likelihood values represent how well each hypothesis matches the current observation.
	 * 
	 * @param likelihood Map of node IDs to their likelihood values
	 */
	void setLikelihood(const std::map<int, float> & likelihood) {_likelihood = likelihood;}
	
	/**
	 * @brief Sets raw likelihood values (before normalization)
	 * @param rawLikelihood Map of node IDs to their raw likelihood values
	 */
	void setRawLikelihood(const std::map<int, float> & rawLikelihood) {_rawLikelihood = rawLikelihood;}
	
	/**
	 * @brief Sets the local path (sequence of node IDs)
	 * 
	 * The local path represents the sequence of next nodes to visit for path planning.
	 * 
	 * @param localPath Vector of node IDs in order of next nodes to visit
	 */
	void setLocalPath(const std::vector<int> & localPath) {_localPath=localPath;}
	
	/**
	 * @brief Sets the current goal node ID
	 * @param goal The goal node ID (0 if no goal)
	 */
	void setCurrentGoalId(int goal) {_currentGoalId=goal;}
	
	/**
	 * @brief Sets the reduced IDs mapping
	 * 
	 * Maps original node IDs to reduced IDs, used for memory optimization.
	 * 
	 * @param reducedIds Map of original IDs to reduced IDs
	 */
	void setReducedIds(const std::map<int, int> & reducedIds) {_reducedIds = reducedIds;}
	
	/**
	 * @brief Sets the working memory state
	 * 
	 * The working memory state is a vector of node IDs currently in working memory.
	 * 
	 * @param state Vector of node IDs in working memory
	 */
	void setWmState(const std::vector<int> & state) {_wmState = state;}
	
	/**
	 * @brief Sets odometry cache poses
	 * 
	 * Cached odometry poses during localization mode.
	 * 
	 * @param poses Map of node IDs to their cached odometry poses
	 */
	void setOdomCachePoses(const std::map<int, Transform> & poses) {_odomCachePoses = poses;}
	
	/**
	 * @brief Sets odometry cache constraints
	 * 
	 * Cached odometry constraints (links) between nodes.
	 * 
	 * @param constraints Multimap of node IDs to their cached odometry links
	 */
	void setOdomCacheConstraints(const std::multimap<int, Link> & constraints) {_odomCacheConstraints = constraints;}

	// getters
	
	/**
	 * @brief Returns whether extended statistics mode is enabled
	 * @return True if extended mode, false if basic mode
	 */
	bool extended() const {return _extended;}
	
	/**
	 * @brief Returns the reference image ID
	 * @return The signature ID (0 if not set)
	 */
	int refImageId() const {return _refImageId;}
	
	/**
	 * @brief Returns the reference image map ID
	 * @return The map ID (-1 if not set)
	 */
	int refImageMapId() const {return _refImageMapId;}
	
	/**
	 * @brief Returns the loop closure detection ID
	 * @return The signature ID where loop closure was detected (0 if none)
	 */
	int loopClosureId() const {return _loopClosureId;}
	
	/**
	 * @brief Returns the loop closure map ID
	 * @return The map ID associated with the loop closure (-1 if not set)
	 */
	int loopClosureMapId() const {return _loopClosureMapId;}
	
	/**
	 * @brief Returns the proximity detection ID
	 * @return The signature ID where proximity was detected (0 if none)
	 */
	int proximityDetectionId() const {return _proximiyDetectionId;}
	
	/**
	 * @brief Returns the proximity detection map ID
	 * @return The map ID associated with the proximity detection (-1 if not set)
	 */
	int proximityDetectionMapId() const {return _proximiyDetectionMapId;}
	
	/**
	 * @brief Returns the timestamp
	 * @return The timestamp (0.0 if not set)
	 */
	double stamp() const {return _stamp;}

	/**
	 * @brief Returns the last signature data
	 * 
	 * Returns the most recently added signature (by ID). If no signatures are stored,
	 * returns a dummy empty signature.
	 * 
	 * @return Reference to the last signature data
	 */
	const Signature & getLastSignatureData() const {return _signaturesData.empty()?_dummyEmptyData:_signaturesData.rbegin()->second;}
	
	/**
	 * @brief Returns all signature data
	 * @return Const reference to the map of signature IDs to Signature objects
	 */
	const std::map<int, Signature> & getSignaturesData() const {return _signaturesData;}

	/**
	 * @brief Returns the pose graph
	 * @return Const reference to the map of node IDs to poses
	 */
	const std::map<int, Transform> & poses() const {return _poses;}
	
	/**
	 * @brief Returns the constraint graph
	 * @return Const reference to the multimap of node IDs to links
	 */
	const std::multimap<int, Link> & constraints() const {return _constraints;}
	
	/**
	 * @brief Returns the map correction transform
	 * 
	 * Returns the transform from the map fixed frame to the odometry fixed frame.
	 * 
	 * @return Const reference to the map correction transform
	 */
	const Transform & mapCorrection() const {return _mapCorrection;}
	
	/**
	 * @brief Returns the loop closure transform
	 * @return Const reference to the loop closure transform
	 */
	const Transform & loopClosureTransform() const {return _loopClosureTransform;}
	
	/**
	 * @brief Returns the localization covariance matrix
	 * @return Const reference to the covariance matrix (may be empty)
	 */
	const cv::Mat & localizationCovariance() const {return _localizationCovariance;}
	
	/**
	 * @brief Returns node labels
	 * @return Const reference to the map of node IDs to labels
	 */
	const std::map<int, std::string> & labels() const {return _labels;}
	
	/**
	 * @brief Returns node weights
	 * @return Const reference to the map of node IDs to weights
	 */
	const std::map<int, int> & weights() const {return _weights;}
	
	/**
	 * @brief Returns posterior probabilities
	 * @return Const reference to the map of node IDs to posterior probabilities
	 */
	const std::map<int, float> & posterior() const {return _posterior;}
	
	/**
	 * @brief Returns likelihood values
	 * @return Const reference to the map of node IDs to likelihood values
	 */
	const std::map<int, float> & likelihood() const {return _likelihood;}
	
	/**
	 * @brief Returns raw likelihood values
	 * @return Const reference to the map of node IDs to raw likelihood values
	 */
	const std::map<int, float> & rawLikelihood() const {return _rawLikelihood;}
	
	/**
	 * @brief Returns the local path
	 * @return Const reference to the vector of node IDs
	 */
	const std::vector<int> & localPath() const {return _localPath;}
	
	/**
	 * @brief Returns the current goal node ID
	 * @return The goal node ID (0 if no goal)
	 */
	int currentGoalId() const {return _currentGoalId;}
	
	/**
	 * @brief Returns the reduced IDs mapping
	 * @return Const reference to the map of original IDs to reduced IDs
	 */
	const std::map<int, int> & reducedIds() const {return _reducedIds;}
	
	/**
	 * @brief Returns the working memory state
	 * @return Const reference to the vector of node IDs in working memory
	 */
	const std::vector<int> & wmState() const {return _wmState;}
	
	/**
	 * @brief Returns odometry cache poses
	 * @return Const reference to the map of node IDs to cached odometry poses
	 */
	const std::map<int, Transform> & odomCachePoses() const {return _odomCachePoses;}
	
	/**
	 * @brief Returns odometry cache constraints
	 * @return Const reference to the multimap of node IDs to cached odometry links
	 */
	const std::multimap<int, Link> & odomCacheConstraints() const {return _odomCacheConstraints;}

	/**
	 * @brief Returns the statistics data map
	 * 
	 * Returns the map containing all plottable statistics in the format "Group/Name/Unit" -> value.
	 * This is the main data structure for storing numeric statistics.
	 * 
	 * @return Const reference to the statistics data map
	 * 
	 * @note Use addStatistic() to add values to this map
	 * @note Use serializeData() to convert this map to a string for storage
	 */
	const std::map<std::string, float> & data() const {return _data;}

private:
	bool _extended; ///< Extended mode flag: false = only loop closure and last signature ID, true = all statistics

	int _refImageId; ///< Reference image ID (current/last processed signature)
	int _refImageMapId; ///< Reference image map ID
	int _loopClosureId; ///< Loop closure detection ID (0 if none)
	int _loopClosureMapId; ///< Loop closure map ID
	int _proximiyDetectionId; ///< Proximity detection ID (0 if none)
	int _proximiyDetectionMapId; ///< Proximity detection map ID
	double _stamp; ///< Timestamp for these statistics

	std::map<int, Signature> _signaturesData; ///< Map of signature IDs to Signature objects
	Signature _dummyEmptyData; ///< Dummy empty signature returned when no signatures are stored

	std::map<int, Transform> _poses; ///< Pose graph: node IDs to poses
	std::multimap<int, Link> _constraints; ///< Constraint graph: node IDs to links (multimap allows multiple links per node)
	Transform _mapCorrection; ///< Transform from map fixed frame to odometry fixed frame (typically updated after optimization)
	Transform _loopClosureTransform; ///< Loop closure transform
	cv::Mat _localizationCovariance; ///< Localization covariance matrix

	std::map<int, std::string> _labels; ///< Node labels
	std::map<int, int> _weights; ///< Node weights
	std::map<int, float> _posterior; ///< Posterior probabilities for loop closure hypotheses
	std::map<int, float> _likelihood; ///< Likelihood values for loop closure hypotheses
	std::map<int, float> _rawLikelihood; ///< Raw likelihood values (before normalization)

	std::vector<int> _localPath; ///< Local path (sequence of node IDs)
	int _currentGoalId; ///< Current goal node ID (0 if no goal)

	std::map<int, int> _reducedIds; ///< Mapping of original IDs to reduced/compressed IDs

	std::vector<int> _wmState; ///< Working memory state (vector of node IDs in working memory)

	std::map<int, Transform> _odomCachePoses; ///< Cached odometry poses in localization mode
	std::multimap<int, Link> _odomCacheConstraints; ///< Cached odometry constraints/links

	/**
	 * @brief Statistics data map (plottable statistics)
	 * 
	 * Format: {"Group/Name/Unit", value}
	 * Example: {"Timing/Total time/ms", 500.0f}
	 * 
	 * All plottable numeric statistics are stored in this map with hierarchical names
	 * that allow for easy categorization and visualization.
	 */
	std::map<std::string, float> _data;
	
	static std::map<std::string, float> _defaultData; ///< Static default data map (initialized on first Statistics construction)
	static bool _defaultDataInitialized; ///< Flag indicating if default data has been initialized
	// end extended data
};

}// end namespace rtabmap

#endif /* STATISTICS_H_ */
