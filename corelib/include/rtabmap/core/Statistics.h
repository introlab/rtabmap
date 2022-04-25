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

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <list>
#include <vector>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Link.h>

namespace rtabmap {

#define RTABMAP_STATS(PREFIX, NAME, UNIT) \
	public: \
		static std::string k##PREFIX##NAME() {return #PREFIX "/" #NAME "/" #UNIT;} \
	private: \
		class Dummy##PREFIX##NAME { \
		public: \
			Dummy##PREFIX##NAME() {if(!_defaultDataInitialized)_defaultData.insert(std::pair<std::string, float>(#PREFIX "/" #NAME "/" #UNIT, 0.0f));} \
		}; \
		Dummy##PREFIX##NAME dummy##PREFIX##NAME

class RTABMAP_EXP Statistics
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
	RTABMAP_STATS(Loop, Distance_since_last_loc,);
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
	//Odom correction
	RTABMAP_STATS(Loop, Odom_correction_norm, m);
	RTABMAP_STATS(Loop, Odom_correction_angle, deg);
	RTABMAP_STATS(Loop, Odom_correction_x, m);
	RTABMAP_STATS(Loop, Odom_correction_y, m);
	RTABMAP_STATS(Loop, Odom_correction_z, m);
	RTABMAP_STATS(Loop, Odom_correction_roll, deg);
	RTABMAP_STATS(Loop, Odom_correction_pitch, deg);
	RTABMAP_STATS(Loop, Odom_correction_yaw, deg);
	//Odom correction
	RTABMAP_STATS(Loop, Odom_correction_acc_norm, m);
	RTABMAP_STATS(Loop, Odom_correction_acc_angle, deg);
	RTABMAP_STATS(Loop, Odom_correction_acc_x, m);
	RTABMAP_STATS(Loop, Odom_correction_acc_y, m);
	RTABMAP_STATS(Loop, Odom_correction_acc_z, m);
	RTABMAP_STATS(Loop, Odom_correction_acc_roll, deg);
	RTABMAP_STATS(Loop, Odom_correction_acc_pitch, deg);
	RTABMAP_STATS(Loop, Odom_correction_acc_yaw, deg);
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

	RTABMAP_STATS(Timing, Memory_update, ms);
	RTABMAP_STATS(Timing, Neighbor_link_refining, ms);
	RTABMAP_STATS(Timing, Proximity_by_time, ms);
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
	static const std::map<std::string, float> & defaultData();
	static std::string serializeData(const std::map<std::string, float> & data);
	static std::map<std::string, float> deserializeData(const std::string & data);

public:
	Statistics();
	virtual ~Statistics();

	// name format = "Grp/Name/unit"
	void addStatistic(const std::string & name, float value);

	// setters
	void setExtended(bool extended) {_extended = extended;}
	void setRefImageId(int id) {_refImageId = id;}
	void setRefImageMapId(int id) {_refImageMapId = id;}
	void setLoopClosureId(int id) {_loopClosureId = id;}
	void setLoopClosureMapId(int id) {_loopClosureMapId = id;}
	void setProximityDetectionId(int id) {_proximiyDetectionId = id;}
	void setProximityDetectionMapId(int id) {_proximiyDetectionMapId = id;}
	void setStamp(double stamp) {_stamp = stamp;}

	void setLastSignatureData(const Signature & data) {_lastSignatureData = data;}

	void setPoses(const std::map<int, Transform> & poses) {_poses = poses;}
	void setConstraints(const std::multimap<int, Link> & constraints) {_constraints = constraints;}
	void setMapCorrection(const Transform & mapCorrection) {_mapCorrection = mapCorrection;}
	void setLoopClosureTransform(const Transform & loopClosureTransform) {_loopClosureTransform = loopClosureTransform;}
	void setLocalizationCovariance(const cv::Mat & covariance) {_localizationCovariance = covariance;}
	void setLabels(const std::map<int, std::string> & labels) {_labels = labels;}
	void setWeights(const std::map<int, int> & weights) {_weights = weights;}
	void setPosterior(const std::map<int, float> & posterior) {_posterior = posterior;}
	void setLikelihood(const std::map<int, float> & likelihood) {_likelihood = likelihood;}
	void setRawLikelihood(const std::map<int, float> & rawLikelihood) {_rawLikelihood = rawLikelihood;}
	void setLocalPath(const std::vector<int> & localPath) {_localPath=localPath;}
	void setCurrentGoalId(int goal) {_currentGoalId=goal;}
	void setReducedIds(const std::map<int, int> & reducedIds) {_reducedIds = reducedIds;}
	void setWmState(const std::vector<int> & state) {_wmState = state;}
	void setOdomCachePoses(const std::map<int, Transform> & poses) {_odomCachePoses = poses;}
	void setOdomCacheConstraints(const std::multimap<int, Link> & constraints) {_odomCacheConstraints = constraints;}

	// getters
	bool extended() const {return _extended;}
	int refImageId() const {return _refImageId;}
	int refImageMapId() const {return _refImageMapId;}
	int loopClosureId() const {return _loopClosureId;}
	int loopClosureMapId() const {return _loopClosureMapId;}
	int proximityDetectionId() const {return _proximiyDetectionId;}
	int proximityDetectionMapId() const {return _proximiyDetectionMapId;}
	double stamp() const {return _stamp;}

	const Signature & getLastSignatureData() const {return _lastSignatureData;}

	const std::map<int, Transform> & poses() const {return _poses;}
	const std::multimap<int, Link> & constraints() const {return _constraints;}
	const Transform & mapCorrection() const {return _mapCorrection;}
	const Transform & loopClosureTransform() const {return _loopClosureTransform;}
	const cv::Mat & localizationCovariance() const {return _localizationCovariance;}
	const std::map<int, std::string> & labels() const {return _labels;}
	const std::map<int, int> & weights() const {return _weights;}
	const std::map<int, float> & posterior() const {return _posterior;}
	const std::map<int, float> & likelihood() const {return _likelihood;}
	const std::map<int, float> & rawLikelihood() const {return _rawLikelihood;}
	const std::vector<int> & localPath() const {return _localPath;}
	int currentGoalId() const {return _currentGoalId;}
	const std::map<int, int> & reducedIds() const {return _reducedIds;}
	const std::vector<int> & wmState() const {return _wmState;}
	const std::map<int, Transform> & odomCachePoses() const {return _odomCachePoses;}
	const std::multimap<int, Link> & odomCacheConstraints() const {return _odomCacheConstraints;}

	const std::map<std::string, float> & data() const {return _data;}

private:
	bool _extended; // 0 -> only loop closure and last signature ID fields are filled

	int _refImageId;
	int _refImageMapId;
	int _loopClosureId;
	int _loopClosureMapId;
	int _proximiyDetectionId;
	int _proximiyDetectionMapId;
	double _stamp;

	Signature _lastSignatureData;

	std::map<int, Transform> _poses;
	std::multimap<int, Link> _constraints;
	Transform _mapCorrection;
	Transform _loopClosureTransform;
	cv::Mat _localizationCovariance;

	std::map<int, std::string> _labels;
	std::map<int, int> _weights;
	std::map<int, float> _posterior;
	std::map<int, float> _likelihood;
	std::map<int, float> _rawLikelihood;

	std::vector<int> _localPath;
	int _currentGoalId;

	std::map<int, int> _reducedIds;

	std::vector<int> _wmState;

	std::map<int, Transform> _odomCachePoses;
	std::multimap<int, Link> _odomCacheConstraints;

	// Format for statistics (Plottable statistics must go in that map) :
	// {"Group/Name/Unit", value}
	// Example : {"Timing/Total time/ms", 500.0f}
	std::map<std::string, float> _data;
	static std::map<std::string, float> _defaultData;
	static bool _defaultDataInitialized;
	// end extended data
};

}// end namespace rtabmap

#endif /* STATISTICS_H_ */
