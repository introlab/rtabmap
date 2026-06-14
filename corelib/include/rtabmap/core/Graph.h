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

#ifndef GRAPH_H_
#define GRAPH_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <map>
#include <list>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/GPS.h>
#include <rtabmap/core/CameraModel.h>

namespace rtabmap {
class Memory;

/**
 * @namespace graph
 * @brief Pose-graph I/O, trajectory metrics, link utilities, and path planning.
 *
 * Functions operate on maps of signature ids to @ref Transform poses and
 * @ref Link constraints (typically stored as `std::multimap<int, Link>` keyed by
 * the source node id).
 *
 * Main groups:
 * - **I/O:** @ref exportPoses(), @ref importPoses(), @ref exportGPS()
 * - **Evaluation:** @ref calcKittiSequenceErrors(), @ref calcRelativeErrors(),
 *   @ref calcRMSE(), @ref computeMaxGraphErrors()
 * - **Links:** @ref findLink(), @ref findLinks(), @ref filterLinks(),
 *   @ref filterDuplicateLinks()
 * - **Spatial queries:** @ref findNearestNode(), @ref findNearestNodes(),
 *   @ref frustumPosesFiltering(), @ref radiusPosesFiltering()
 * - **Planning:** @ref computePath(), @ref computePathLength(), @ref getPaths()
 */
namespace graph {

/**
 * @brief Writes poses (and optional constraints) to disk.
 * @param filePath Output path; extension may be appended from @p format.
 * @param format Export format:
 *   - `0` Raw text (`.txt`): r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
 *   - `1` RGBD-SLAM format, in motion capture frame like the ground truth of RGB-D SLAM Dataset (requires @p stamps) : stamp x y z qx qy qz qw
 *   - `10` Like `1` without coordinate-frame change (i.e., in base frame) : stamp x y z qx qy qz qw
 *   - `11` Like `10` with landmark ids after positive ids : stamp x y z qx qy qz qw id
 *   - `2` KITTI odometry format : r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
 *   - `3` TORO graph (requires @p constraints; uses @p parameters)
 *   - `4` g2o (requires @p constraints; uses @p parameters)
 * @param poses Node id → pose.
 * @param constraints Required for formats `3` and `4`.
 * @param stamps Required for formats `1`, `10`, and `11` (same size as @p poses).
 * @param parameters Optional optimizer parameters for formats `3` and `4`.
 * @return False on I/O or validation error.
 */
bool RTABMAP_CORE_EXPORT exportPoses(
		const std::string & filePath,
		int format,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & constraints = std::multimap<int, Link>(),
		const std::map<int, double> & stamps = std::map<int, double>(),
		const ParametersMap & parameters = ParametersMap());

/**
 * @brief Loads poses (and optional constraints) from disk.
 * @param filePath Input path.
 * @param format Import format:
 *   - `0` Raw text: 3×4 matrix per line (`Transform::fromString()`)
 *   - `1` RGBD-SLAM motion capture: stamp x y z qw qx qy qz (applies optical-frame conversion)
 *   - `2` KITTI odometry: 3×4 matrix per line (applies optical-frame conversion)
 *   - `3` TORO graph (fills @p constraints)
 *   - `4` g2o (not supported yet)
 *   - `5` NewCollege: stamp x y (2D; first pose is origin)
 *   - `6` Malaga Urban GPS: 25-field `*_GPS.txt` line (local X/Y/Z)
 *   - `7` St Lucia INS: 12-field log (GPS → local ENU + roll/pitch/yaw)
 *   - `8` Karlsruhe: timestamp lat lon alt x y z roll pitch yaw (first pose is origin)
 *   - `9` EuRoC MAV: stamp x y z qw qx qy qz vx vy vz vr vp vy ax ay az (17 CSV fields)
 *   - `10` RGBD-SLAM like `1` without coordinate-frame change
 *   - `11` RGBD-SLAM like `10` with node id as 9th field: stamp x y z qw qx qy qz id
 *   - `12` RGBD Bonn dynamic dataset format (stamp + pose; Bonn-specific frame conversion)
 * @param poses Output node id → pose.
 * @param constraints Optional output links (format `3` only).
 * @param stamps Optional output timestamps (formats `1`, `5`–`9`, `10`–`12` when present in file).
 * @return False on I/O or parse error.
 */
bool RTABMAP_CORE_EXPORT importPoses(
		const std::string & filePath,
		int format,
		std::map<int, Transform> & poses,
		std::multimap<int, Link> * constraints = 0,
		std::map<int, double> * stamps = 0);

/**
 * @brief Exports GPS samples to a PLY point cloud.
 * @param filePath Output `.ply` path.
 * @param gpsValues Node id → @ref GPS fix.
 * @param rgba Point color (default opaque white).
 */
bool RTABMAP_CORE_EXPORT exportGPS(
		const std::string & filePath,
		const std::map<int, GPS> & gpsValues,
		unsigned int rgba = 0xFFFFFFFF);

/**
 * @brief KITTI odometry benchmark error over fixed trajectory segments.
 *
 * For each start pose (every 10 frames) and segment length in
 * {100, 200, …, 800} m along @p poses_gt, compares the relative transform
 * GT vs estimate and accumulates normalized errors. The returned values are
 * the mean over all valid segments.
 *
 * @param poses_gt Ground-truth poses in temporal order (one per frame).
 * @param poses_result Estimated poses (same length and ordering as @p poses_gt).
 * @param t_err Output mean translation error (%): segment translation error (m)
 *              divided by segment length, averaged, then × 100.
 * @param r_err Output mean rotation error (deg/m): segment rotation error (rad)
 *              divided by segment length, averaged, then converted to deg/m.
 * @see http://www.cvlibs.net/datasets/kitti/eval_odometry.php
 */
void RTABMAP_CORE_EXPORT calcKittiSequenceErrors(
		const std::vector<Transform> &poses_gt,
		const std::vector<Transform> &poses_result,
		float & t_err,
		float & r_err);

/**
 * @brief Mean frame-to-frame relative pose error (RPE-style, one step).
 *
 * For each consecutive pair `(i, i+1)`, builds the relative motion in ground
 * truth and in the estimate, then measures how much they differ:
 * - translation: Euclidean distance between the two relative transforms (m)
 * - rotation: angle between the two relative transforms (rad → deg)
 *
 * Returns the arithmetic mean over all `N-1` pairs (`N` = trajectory length).
 * Unlike @ref calcKittiSequenceErrors(), there is no fixed segment length and
 * no path-length normalization.
 *
 * @param poses_gt Ground-truth poses in temporal order (one per frame).
 * @param poses_result Estimated poses (same length and ordering as @p poses_gt).
 * @param t_err Output mean translation error over consecutive pairs (m).
 * @param r_err Output mean rotation error over consecutive pairs (deg).
 */
void RTABMAP_CORE_EXPORT calcRelativeErrors (
		const std::vector<Transform> &poses_gt,
		const std::vector<Transform> &poses_result,
		float & t_err,
		float & r_err);

/**
 * @brief Absolute trajectory error (ATE) with Sim(3)-style alignment (TUM RGB-D tool).
 *
 * Only poses whose id exists in both @p groundTruth and @p poses are compared.
 * An alignment transform @c t is estimated so that per-pose error is measured after
 * bringing the estimate into the reference frame:
 * - If more than five poses match: @c t from SVD on position correspondences
 *   (estimate positions → ground-truth positions; z ignored when @p align2D is true).
 * - Otherwise: @c t = groundTruth[firstId] * poses[firstId]⁻¹ using the first matched id.
 *
 * For each matched pose, after `aligned = t * poses[id]`:
 * - **Translational error:** Euclidean distance between `aligned` and `groundTruth[id]` (m).
 * - **Rotational error:** Angle between the poses' +X axes (deg).
 *
 * The eight `@p translational_*` and `@p rotational_*` outputs are statistics over those
 * per-pose errors (all matched poses). They are set to `0` when no id matches.
 *
 * @param groundTruth Reference trajectory (node id → pose).
 * @param poses Estimated trajectory; ids not in @p groundTruth are skipped.
 * @param translational_rmse Root mean square of translational errors (m).
 * @param translational_mean Arithmetic mean of translational errors (m).
 * @param translational_median Middle sample in matched-pose iteration order (m).
 * @param translational_std Standard deviation of translational errors (m).
 * @param translational_min Minimum translational error (m).
 * @param translational_max Maximum translational error (m).
 * @param rotational_rmse Root mean square of rotational errors (deg).
 * @param rotational_mean Arithmetic mean of rotational errors (deg).
 * @param rotational_median Middle sample in matched-pose iteration order (deg).
 * @param rotational_std Standard deviation of rotational errors (deg).
 * @param rotational_min Minimum rotational error (deg).
 * @param rotational_max Maximum rotational error (deg).
 * @param align2D If true, alignment uses x/y only (z set to 0 for correspondence); 3D if false.
 * @return Alignment transform @c t applied as `t * poses[id]` before error computation.
 * @see https://vision.in.tum.de/data/datasets/rgbd-dataset
 */
Transform RTABMAP_CORE_EXPORT calcRMSE(
		const std::map<int, Transform> &groundTruth,
		const std::map<int, Transform> &poses,
		float & translational_rmse,
		float & translational_mean,
		float & translational_median,
		float & translational_std,
		float & translational_min,
		float & translational_max,
		float & rotational_rmse,
		float & rotational_mean,
		float & rotational_median,
		float & rotational_std,
		float & rotational_min,
		float & rotational_max,
		bool align2D = false);

/**
 * @brief Largest pose-graph constraint violations after optimization.
 *
 * For each non-self-referenced link (`from != to`), compares the relative pose implied by @p poses to the
 * link measurement and tracks the worst linear/angular error and error/std ratios.
 */
struct MaxGraphErrors
{
	float linear=-1.0f;         ///< Absolute linear error (m) of the worst link.
	float angular=-1.0f;        ///< Absolute angular error (rad) of the worst link.
	float linearRatio=-1.0f;    ///< linear / sqrt(trans variance) of the worst link.
	float angularRatio=-1.0f;   ///< angular / sqrt(rot variance) of the worst link.
	Link linearLink;            ///< Link with largest @ref linearRatio.
	Link angularLink;           ///< Link with largest @ref angularRatio.
};

/**
 * @brief Finds the worst pose-graph constraint residuals after optimization.
 *
 * Iterates over @p links and, for each non-self-referenced edge (`from != to`):
 * 1. Looks up `T_from` and `T_to` in @p poses (returns default @ref MaxGraphErrors if
 *    any endpoint pose is missing, null, or not invertible).
 * 2. Builds the relative pose implied by the optimized poses:
 *    - Normal link: `t = T_from⁻¹ · T_to`
 *    - Landmark (`from < 0`): `t = T_to⁻¹ · T_from`, link measurement inverted
 * 3. Compares `t` to the link transform:
 *    - **Linear error:** max |Δx|, |Δy|, and |Δz| (z ignored when @p for3DoF is true).
 *    - **Angular error:** full 3D angle between `t` and the link, or yaw-only if @p for3DoF;
 *      skipped for @ref Link::kLandmark when the information matrix does not constrain yaw.
 * 4. Normalizes by link uncertainty: `error / sqrt(variance)` using the link information
 *    matrix (largest diagonal variance for translation/rotation).
 *
 * The returned @ref MaxGraphErrors holds the link with the highest linear and angular
 * *ratios* (not necessarily the largest absolute error).
 *
 * @param poses Optimized node poses (must contain every `from` and `to` id used).
 * @param links Graph constraints (typically `std::multimap<int, Link>` keyed by `from`).
 * @param for3DoF If true, linear error uses x/y only and angular error compares yaw only.
 * @return @ref MaxGraphErrors; fields stay `-1` when no valid link was checked or on early abort.
 */
MaxGraphErrors RTABMAP_CORE_EXPORT computeMaxGraphErrors(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		bool for3DoF = false);

/**
 * @brief Maximum information-matrix diagonal over odometry neighbor links.
 *
 * Scans @p links of type @ref Link::kNeighbor or @ref Link::kNeighborMerged and,
 * for each dof (x, y, z, roll, pitch, yaw), keeps the largest diagonal entry of
 * the 6×6 information matrix.
 *
 * @param links Graph constraints (multimap keyed by source id).
 * @return Six maximum information values, or an empty vector if no neighbor links exist.
 */
std::vector<double> RTABMAP_CORE_EXPORT getMaxOdomInf(const std::multimap<int, Link> & links);

/**
 * @brief Finds the first link from @p from to @p to in a multimap keyed by source id.
 *
 * Iterates all entries with key @p from and matches the destination (and optionally
 * @p type). When @p checkBothWays is true, also searches key @p to for a link back
 * to @p from.
 *
 * @param links Link multimap (`key` = source node id).
 * @param from Source node id.
 * @param to Destination node id.
 * @param checkBothWays If true, also match `to → from`.
 * @param type Required link type, or @ref Link::kUndef to accept any type.
 * @return Iterator to the link, or `links.end()` if not found.
 */
std::multimap<int, Link>::iterator RTABMAP_CORE_EXPORT findLink(
		std::multimap<int, Link> & links,
		int from,
		int to,
		bool checkBothWays = true,
		Link::Type type = Link::kUndef);
/** @overload `std::multimap<int, std::pair<int, Link::Type>>`. */
std::multimap<int, std::pair<int, Link::Type> >::iterator RTABMAP_CORE_EXPORT findLink(
		std::multimap<int, std::pair<int, Link::Type> > & links,
		int from,
		int to,
		bool checkBothWays = true,
		Link::Type type = Link::kUndef);
/** @overload `std::multimap<int, int>`. */
std::multimap<int, int>::iterator RTABMAP_CORE_EXPORT findLink(
		std::multimap<int, int> & links,
		int from,
		int to,
		bool checkBothWays = true);
/** @overload const `std::multimap<int, Link>`. */
std::multimap<int, Link>::const_iterator RTABMAP_CORE_EXPORT findLink(
		const std::multimap<int, Link> & links,
		int from,
		int to,
		bool checkBothWays = true,
		Link::Type type = Link::kUndef);
/** @overload const `std::multimap<int, std::pair<int, Link::Type>>`. */
std::multimap<int, std::pair<int, Link::Type> >::const_iterator RTABMAP_CORE_EXPORT findLink(
		const std::multimap<int, std::pair<int, Link::Type> > & links,
		int from,
		int to,
		bool checkBothWays = true,
		Link::Type type = Link::kUndef);
/** @overload const `std::multimap<int, int>`. */
std::multimap<int, int>::const_iterator RTABMAP_CORE_EXPORT findLink(
		const std::multimap<int, int> & links,
		int from,
		int to,
		bool checkBothWays = true);

/**
 * @brief Lists all links incident on node @p from.
 *
 * Outgoing links (`link.from() == from`) are returned as stored; for incoming links
 * (`link.to() == from`), the inverse link is returned so the pose of @p from is always
 * the source frame.
 *
 * @param links Graph constraints.
 * @param from Node id to query.
 * @return Incident links (may be empty).
 */
std::list<Link> RTABMAP_CORE_EXPORT findLinks(
		const std::multimap<int, Link> & links,
		int from);

/**
 * @brief Removes duplicate undirected links.
 *
 * Keeps the first occurrence of each `(from, to)` or `(to, from)` pair with the same
 * @ref Link::Type (see @ref findLink() with @p checkBothWays).
 *
 * @param links Input link multimap.
 * @return Copy without duplicates.
 */
std::multimap<int, Link> RTABMAP_CORE_EXPORT filterDuplicateLinks(
		const std::multimap<int, Link> & links);

/**
 * @brief Filters links by type or self-reference.
 *
 * - If @p filteredType is @ref Link::kSelfRefLink: exclude self-references (`from == to`),
 *   or include only them when @p inverted is true.
 * - Otherwise: exclude links of @p filteredType, or keep only that type when @p inverted is true.
 *
 * @param links Input links.
 * @param filteredType Type to filter, or @ref Link::kSelfRefLink for self-reference filtering.
 * @param inverted If true, keep the filtered set instead of removing it.
 * @return Filtered link container (same structure as input).
 */
std::multimap<int, Link> RTABMAP_CORE_EXPORT filterLinks(
		const std::multimap<int, Link> & links,
		Link::Type filteredType,
		bool inverted = false);
/** @overload for `std::map<int, Link>`. */
std::map<int, Link> RTABMAP_CORE_EXPORT filterLinks(
		const std::map<int, Link> & links,
		Link::Type filteredType,
		bool inverted = false);

/**
 * @brief Keeps poses inside (or outside) a camera frustum.
 *
 * Transforms each pose position into the frustum defined by @p cameraPose using
 * @ref util3d::frustumFiltering() (this assumes the cameraPose includes the optical rotation of the camera (X right, Y down, Z forward).
 *
 * @param poses Input poses (null poses are skipped) in base frame (X forward, Y left, Z up), 
 * @param cameraPose Frustum origin and orientation including the optical rotation of the camera (X right, Y down, Z forward).
 * @param horizontalFOV Horizontal field of view (deg); see @ref CameraModel::horizontalFOV().
 * @param verticalFOV Vertical field of view (deg); see @ref CameraModel::verticalFOV().
 * @param nearClipPlaneDistance Near clipping distance (m).
 * @param farClipPlaneDistance Far clipping distance (m).
 * @param negative If false, keep poses inside the frustum; if true, keep poses outside.
 * @return Subset of @p poses passing the filter.
 */
std::map<int, Transform> RTABMAP_CORE_EXPORT frustumPosesFiltering(
		const std::map<int, Transform> & poses,
		const Transform & cameraPose,
		float horizontalFOV = 45.0f,
		float verticalFOV = 45.0f,
		float nearClipPlaneDistance = 0.1f,
		float farClipPlaneDistance = 100.0f,
		bool negative = false);

/**
 * @brief Subsamples poses that are spatially (and optionally angularly) redundant.
 *
 * For each pose not yet processed, finds all poses within @p radius (KD-tree). When
 * @p angle &gt; 0, only poses whose +X axis differs by at most @p angle (rad) are grouped.
 * From each group, keeps one pose: the latest in map order if @p keepLatest, otherwise
 * the earliest. The first and last poses of the input map are always kept.
 *
 * @param poses Input trajectory (map iteration order defines “latest/oldest”).
 * @param radius Clustering radius (m); if `≤ 0` or fewer than three poses, returns @p poses unchanged.
 * @param angle Max heading difference within a cluster (rad); `0` ignores orientation.
 * @param keepLatest If true, keep the latest pose per cluster; otherwise the earliest.
 * @return Subsampled poses.
 */
std::map<int, Transform> RTABMAP_CORE_EXPORT radiusPosesFiltering(
		const std::map<int, Transform> & poses,
		float radius,
		float angle,
		bool keepLatest = true);

/**
 * @brief Radius-neighbor clustering of poses.
 *
 * For each pose, inserts `(queryId, neighborId)` into the output for every other pose
 * within @p radius (and within @p angle of the query heading when @p angle &gt; 0).
 *
 * @param poses Input poses.
 * @param radius Search radius (m); no pairs if `≤ 0` or fewer than two poses.
 * @param angle Max heading difference (rad); `0` ignores orientation.
 * @return Multimap of pose id → neighbor id (both ids from @p poses).
 */
std::multimap<int, int> RTABMAP_CORE_EXPORT radiusPosesClustering(
		const std::map<int, Transform> & poses,
		float radius,
		float angle);

/**
 * @brief Reduces a pose graph into hyper-nodes and hyper-links.
 *
 * **Hyper-nodes:** clusters poses connected by non-neighbor loop-closure links.
 * Clustering starts from the largest id downward; each cluster is keyed by its parent
 * (hyper-node) id.
 *
 * **Hyper-links:** for each @ref Link::kNeighbor or @ref Link::kNeighborMerged link between
 * different clusters, builds one merged @ref Link along the shortest path through
 * intra-cluster closure links (Dijkstra with unit cost).
 *
 * @param poses Input optimized poses.
 * @param links Input constraints (should be unique per directed edge for closure links).
 * @param hyperNodes Output `hyperNodeId → childPoseId` membership.
 * @param hyperLinks Output links between hyper-nodes (one per hyper-edge, most recent kept).
 */
void reduceGraph(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		std::multimap<int, int> & hyperNodes,
		std::multimap<int, Link> & hyperLinks);

/**
 * @brief A* shortest path on a pose graph with Euclidean edge costs.
 *
 * Edge cost between adjacent nodes is the Euclidean distance between their poses in
 * @p poses. Uses `costSoFar + distToEnd` where `distToEnd` is the distance to the goal pose.
 *
 * @param poses Node id → pose (must contain every node reached by @p links).
 * @param links Directed edges (`from` → `to`) keyed by source id.
 * @param from Start node id.
 * @param to Goal node id.
 * @param updateNewCosts If true, use a multimap queue that can decrease keys when a shorter path is found.
 * @return Ordered path from @p from to @p to (inclusive) with poses; empty if unreachable.
 */
std::list<std::pair<int, Transform> > RTABMAP_CORE_EXPORT computePath(
			const std::map<int, rtabmap::Transform> & poses,
			const std::multimap<int, int> & links,
			int from,
			int to,
			bool updateNewCosts = false);

/**
 * @brief Dijkstra shortest path on link constraints.
 *
 * Explores outgoing links keyed by `link.from()`. Edge cost is `1` when
 * @p useSameCostForAllLinks is true, otherwise the translation norm of the link transform.
 *
 * @param links Constraints keyed by source node id.
 * @param from Start node id.
 * @param to Goal node id.
 * @param updateNewCosts If true, allow cost improvements on open nodes.
 * @param useSameCostForAllLinks If true, unit edge cost; else use `link.transform().getNorm()`.
 * @return Node ids from @p from to @p to (inclusive); empty if unreachable.
 */
std::list<int> RTABMAP_CORE_EXPORT computePath(
			const std::multimap<int, Link> & links,
			int from,
			int to,
			bool updateNewCosts = false,
			bool useSameCostForAllLinks = false);

/**
 * @brief Dijkstra path through the live @ref Memory pose graph.
 *
 * Loads links from @ref Memory (optionally from the database), chains transforms along
 * the chosen path, and returns the accumulated poses. Self-referenced links are skipped.
 *
 * By default (`linearVelocity` and `angularVelocity` ≤ 0), edge cost is translation
 * distance (m) only. When set &gt; 0, costs are expressed in seconds of motion:
 * - @p linearVelocity adds `linkTranslation / linearVelocity` (time to drive the edge at
 *   that speed). Used alone it scales every edge by the same factor, so the **shortest path
 *   is unchanged**; set it to your robot’s typical forward speed (e.g. `0.5` m/s) when you
 *   also use @p angularVelocity so translation and rotation costs are comparable.
 * - @p angularVelocity adds `headingMismatch / angularVelocity`, where heading mismatch is
 *   the angle between the displacement to the next node and that node’s forward (+X) axis.
 *   This is what changes which path is chosen: a chain followed **mostly forward** (small
 *   mismatch) can beat a shorter route through loop closures that require large reorientations
 *   (e.g. `angularVelocity = 1.0` rad/s with `linearVelocity = 0.5` m/s).
 *   With @p angularVelocity &gt; 0 and @p linearVelocity ≤ 0, translation is ignored and the
 *   path minimizes heading mismatch only (forward-following paths, regardless of distance).
 *   This can help loop-closure detection when the map was built with a forward-facing camera:
 *   the path stays aligned with how places were observed while driving forward.
 *
 * @param fromId Start signature id (`≥ 0`).
 * @param toId Goal signature id (`≠ 0`).
 * @param memory Graph memory (must not be null).
 * @param lookInDatabase If true, load links from the database when not already in RAM.
 * @param updateNewCosts If true, allow cost improvements on open nodes.
 * @param linearVelocity If &gt; 0, add `translationNorm / linearVelocity` to edge cost (m/s).
 * @param angularVelocity If &gt; 0, add rotation time from motion direction change (rad/s).
 * @param ignoreDirectLinks If true, skip the direct edge between @p fromId and @p toId.
 * @return Path as `(nodeId, pose)` pairs; first pose is identity at @p fromId. Empty if unreachable.
 */
std::list<std::pair<int, Transform> > RTABMAP_CORE_EXPORT computePath(
		int fromId,
		int toId,
		const Memory * memory,
		bool lookInDatabase = true,
		bool updateNewCosts = false,
		float linearVelocity = 0.0f,
		float angularVelocity = 0.0f,
		bool ignoreDirectLinks = false);

/**
 * @brief Id of the nearest pose to @p targetPose.
 *
 * Wrapper around @ref findNearestNodes() with `radius=0`, `k=1` (1-NN in 3D).
 *
 * @param poses Nodes to search.
 * @param targetPose Query position (x, y, z only; orientation is not used).
 * @param distance If not null, set to the squared Euclidean distance of the match.
 * @return Closest node id, or `0` if @p poses is empty.
 */
int RTABMAP_CORE_EXPORT findNearestNode(
		const std::map<int, rtabmap::Transform> & poses,
		const rtabmap::Transform & targetPose,
		float * distance = 0);

/**
 * @brief Spatial neighbors of a node (KD-tree radius or k-NN search).
 *
 * @p nodeId is removed from the search set. Requires `radius &gt; 0` or `k &gt; 0`.
 * When `radius &gt; 0`, returns all poses within @p radius (up to @p k if `k &gt; 0`).
 * When `radius == 0`, returns the @p k nearest neighbors.
 *
 * @param nodeId Query node (must exist in @p poses); excluded from results.
 * @param poses Candidate poses.
 * @param radius Search radius (m).
 * @param angle Max +X axis angle difference (rad); `0` ignores heading.
 * @param k Max neighbors (`0` = all within radius).
 * @return Neighbor id → squared Euclidean distance.
 */
std::map<int, float> RTABMAP_CORE_EXPORT findNearestNodes(
		int nodeId,
		const std::map<int, Transform> & poses,
		float radius,
		float angle = 0.0f,
		int k=0);
/**
 * @brief Spatial neighbors of a pose (KD-tree radius or k-NN search).
 * @param targetPose Query pose (position used; orientation used when @p angle &gt; 0).
 * @param poses Candidate poses (not modified).
 * @param radius Search radius (m).
 * @param angle Max +X axis angle difference (rad); `0` ignores heading.
 * @param k Max neighbors (`0` = all within radius).
 * @return Neighbor id → squared Euclidean distance.
 */
std::map<int, float> RTABMAP_CORE_EXPORT findNearestNodes(
		const Transform & targetPose,
		const std::map<int, Transform> & poses,
		float radius,
		float angle = 0.0f,
		int k=0);
/**
 * @brief Like @ref findNearestNodes(int,const std::map<int,Transform>&,float,float,int)
 * but returns full @ref Transform values.
 */
std::map<int, Transform> RTABMAP_CORE_EXPORT findNearestPoses(
		int nodeId,
		const std::map<int, Transform> & poses,
		float radius,
		float angle = 0.0f,
		int k=0);
/** @overload query by @ref Transform instead of node id. */
std::map<int, Transform> RTABMAP_CORE_EXPORT findNearestPoses(
		const Transform & targetPose,
		const std::map<int, Transform> & poses,
		float radius,
		float angle = 0.0f,
		int k=0);

/** @deprecated Use @ref findNearestNodes(const Transform&,const std::map<int,Transform>&,float,float,int) with `radius=0`, `k` set. */
RTABMAP_DEPRECATED std::map<int, float> RTABMAP_CORE_EXPORT findNearestNodes(const std::map<int, rtabmap::Transform> & nodes, const rtabmap::Transform & targetPose, int k);
/** @deprecated Use @ref findNearestNodes(int,const std::map<int,Transform>&,float,float,int). */
RTABMAP_DEPRECATED std::map<int, float> RTABMAP_CORE_EXPORT getNodesInRadius(int nodeId, const std::map<int, Transform> & nodes, float radius);
/** @deprecated Use @ref findNearestNodes(const Transform&,const std::map<int,Transform>&,float,float,int). */
RTABMAP_DEPRECATED std::map<int, float> RTABMAP_CORE_EXPORT getNodesInRadius(const Transform & targetPose, const std::map<int, Transform> & nodes, float radius);
/** @deprecated Use @ref findNearestPoses(int,const std::map<int,Transform>&,float,float,int). */
RTABMAP_DEPRECATED std::map<int, Transform> RTABMAP_CORE_EXPORT getPosesInRadius(int nodeId, const std::map<int, Transform> & nodes, float radius, float angle = 0.0f);
/** @deprecated Use @ref findNearestPoses(const Transform&,const std::map<int,Transform>&,float,float,int). */
RTABMAP_DEPRECATED std::map<int, Transform> RTABMAP_CORE_EXPORT getPosesInRadius(const Transform & targetPose, const std::map<int, Transform> & nodes, float radius, float angle = 0.0f);

/**
 * @brief Path length along an ordered list of poses.
 *
 * Sums `path[i].second.getDistance(path[i+1].second)` for consecutive entries.
 *
 * @param path Ordered `(nodeId, pose)` pairs.
 * @return Total length (m), or `0` if fewer than two poses.
 */
float RTABMAP_CORE_EXPORT computePathLength(
		const std::vector<std::pair<int, Transform> > & path);

/**
 * @brief Path length in map iteration order.
 *
 * Sums distances between consecutive poses in ascending map key order (does not verify
 * that entries form a connected path in the graph).
 *
 * @param path Poses keyed by node id (sorted by key).
 * @return Total length (m), or `0` if fewer than two poses.
 */
float RTABMAP_CORE_EXPORT computePathLength(
		const std::map<int, Transform> & path);

/**
 * @brief Splits poses into chains connected only by neighbor links.
 *
 * Repeatedly builds a path starting from the lowest remaining id: adds the next pose
 * in map order only if a @ref Link::kNeighbor or @ref Link::kNeighborMerged link exists
 * from the previous pose to it. Stops at the first gap, pushes the chain, and continues
 * until @p poses is empty.
 *
 * @param poses Input poses (cleared as segments are extracted).
 * @param links Graph constraints keyed by source id.
 * @return List of pose maps, each a contiguous neighbor chain.
 */
std::list<std::map<int, Transform> > RTABMAP_CORE_EXPORT getPaths(
		std::map<int, Transform> poses,
		const std::multimap<int, Link> & links);

/**
 * @brief Axis-aligned bounding box of pose positions.
 *
 * @param poses Input poses (no effect if empty).
 * @param min Output minimum (x, y, z) in meters.
 * @param max Output maximum (x, y, z) in meters.
 */
void RTABMAP_CORE_EXPORT computeMinMax(const std::map<int, Transform> & poses,
		cv::Vec3f & min,
		cv::Vec3f & max);

} /* namespace graph */

} /* namespace rtabmap */
#endif /* GRAPH_H_ */
