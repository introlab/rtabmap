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
		Dummy##PREFIX##NAME dummy##PREFIX##NAME;

class RTABMAP_EXP Statistics
{
	RTABMAP_STATS(Loop, RejectedHypothesis,);
	RTABMAP_STATS(Loop, Accepted_hypothesis_id,);
	RTABMAP_STATS(Loop, Highest_hypothesis_id,);
	RTABMAP_STATS(Loop, Highest_hypothesis_value,);
	RTABMAP_STATS(Loop, Vp_hypothesis,);
	RTABMAP_STATS(Loop, Reactivate_id,);
	RTABMAP_STATS(Loop, Hypothesis_ratio,);
	RTABMAP_STATS(Loop, Hypothesis_reactivated,);
	RTABMAP_STATS(Loop, Visual_inliers,);
	RTABMAP_STATS(Loop, Last_id,);
	RTABMAP_STATS(Loop, Optimization_max_error, m);
	RTABMAP_STATS(Loop, Optimization_error, );
	RTABMAP_STATS(Loop, Optimization_iterations, );

	RTABMAP_STATS(Proximity, Time_detections,);
	RTABMAP_STATS(Proximity, Space_last_detection_id,);
	RTABMAP_STATS(Proximity, Space_paths,);
	RTABMAP_STATS(Proximity, Space_detections_added_visually,);
	RTABMAP_STATS(Proximity, Space_detections_added_icp_only,);

	RTABMAP_STATS(NeighborLinkRefining, Accepted,);
	RTABMAP_STATS(NeighborLinkRefining, Inliers,);
	RTABMAP_STATS(NeighborLinkRefining, Inliers_ratio,);
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
	RTABMAP_STATS(Memory, Small_movement,);
	RTABMAP_STATS(Memory, Distance_travelled, m);

	RTABMAP_STATS(Timing, Memory_update, ms);
	RTABMAP_STATS(Timing, Neighbor_link_refining, ms);
	RTABMAP_STATS(Timing, Proximity_by_time, ms);
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

	RTABMAP_STATS(TimingMem, Pre_update, ms);
	RTABMAP_STATS(TimingMem, Signature_creation, ms);
	RTABMAP_STATS(TimingMem, Rehearsal, ms);
	RTABMAP_STATS(TimingMem, Keypoints_detection, ms);
	RTABMAP_STATS(TimingMem, Subpixel, ms);
	RTABMAP_STATS(TimingMem, Stereo_correspondences, ms);
	RTABMAP_STATS(TimingMem, Descriptors_extraction, ms);
	RTABMAP_STATS(TimingMem, Keypoints_3D, ms);
	RTABMAP_STATS(TimingMem, Joining_dictionary_update, ms);
	RTABMAP_STATS(TimingMem, Add_new_words, ms);
	RTABMAP_STATS(TimingMem, Compressing_data, ms);

	RTABMAP_STATS(Keypoint, Dictionary_size, words);
	RTABMAP_STATS(Keypoint, Indexed_words, words);
	RTABMAP_STATS(Keypoint, Index_memory_usage, KB);
	RTABMAP_STATS(Keypoint, Response_threshold,);

public:
	static const std::map<std::string, float> & defaultData();

public:
	Statistics();
	virtual ~Statistics();

	// name format = "Grp/Name/unit"
	void addStatistic(const std::string & name, float value);

	// setters
	void setExtended(bool extended) {_extended = extended;}
	void setRefImageId(int refImageId) {_refImageId = refImageId;}
	void setLoopClosureId(int loopClosureId) {_loopClosureId = loopClosureId;}
	void setProximityDetectionId(int id) {_proximiyDetectionId = id;}
	void setStamp(double stamp) {_stamp = stamp;}

	void setSignatures(const std::map<int, Signature> & signatures) {_signatures = signatures;}

	void setPoses(const std::map<int, Transform> & poses) {_poses = poses;}
	void setConstraints(const std::multimap<int, Link> & constraints) {_constraints = constraints;}
	void setMapCorrection(const Transform & mapCorrection) {_mapCorrection = mapCorrection;}
	void setLoopClosureTransform(const Transform & loopClosureTransform) {_loopClosureTransform = loopClosureTransform;}
	void setWeights(const std::map<int, int> & weights) {_weights = weights;}
	void setPosterior(const std::map<int, float> & posterior) {_posterior = posterior;}
	void setLikelihood(const std::map<int, float> & likelihood) {_likelihood = likelihood;}
	void setRawLikelihood(const std::map<int, float> & rawLikelihood) {_rawLikelihood = rawLikelihood;}
	void setLocalPath(const std::vector<int> & localPath) {_localPath=localPath;}
	void setCurrentGoalId(int goal) {_currentGoalId=goal;}
	void setReducedIds(const std::map<int, int> & reducedIds) {_reducedIds = reducedIds;}

	// getters
	bool extended() const {return _extended;}
	int refImageId() const {return _refImageId;}
	int loopClosureId() const {return _loopClosureId;}
	int proximityDetectionId() const {return _proximiyDetectionId;}
	double stamp() const {return _stamp;}

	const std::map<int, Signature> & getSignatures() const {return _signatures;}

	const std::map<int, Transform> & poses() const {return _poses;}
	const std::multimap<int, Link> & constraints() const {return _constraints;}
	const Transform & mapCorrection() const {return _mapCorrection;}
	const Transform & loopClosureTransform() const {return _loopClosureTransform;}
	const std::map<int, int> & weights() const {return _weights;}
	const std::map<int, float> & posterior() const {return _posterior;}
	const std::map<int, float> & likelihood() const {return _likelihood;}
	const std::map<int, float> & rawLikelihood() const {return _rawLikelihood;}
	const std::vector<int> & localPath() const {return _localPath;}
	int currentGoalId() const {return _currentGoalId;}
	const std::map<int, int> & reducedIds() const {return _reducedIds;}

	const std::map<std::string, float> & data() const {return _data;}

private:
	bool _extended; // 0 -> only loop closure and last signature ID fields are filled

	int _refImageId;
	int _loopClosureId;
	int _proximiyDetectionId;
	double _stamp;

	std::map<int, Signature> _signatures;

	std::map<int, Transform> _poses;
	std::multimap<int, Link> _constraints;
	Transform _mapCorrection;
	Transform _loopClosureTransform;

	std::map<int, int> _weights;
	std::map<int, float> _posterior;
	std::map<int, float> _likelihood;
	std::map<int, float> _rawLikelihood;

	std::vector<int> _localPath;
	int _currentGoalId;

	std::map<int, int> _reducedIds;

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
