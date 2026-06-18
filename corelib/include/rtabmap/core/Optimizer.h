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

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <map>
#include <list>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>

namespace rtabmap {

/**
 * @class FeatureBA
 * @brief A single bundle adjustment feature observation: one keypoint seen in one frame.
 *
 * Used as the per-frame value in the @c wordReferences map (`<wordId, <frameId, FeatureBA>>`)
 * passed to @ref Optimizer::optimizeBA(). @ref depth is in meters when available (RGB-D / stereo
 * with disparity) or 0 if unknown (monocular). @ref descriptor and @ref cameraIndex are optional;
 * @ref cameraIndex selects which @ref CameraModel of a multi-camera rig the keypoint belongs to.
 */
class FeatureBA
{
public:
	FeatureBA(const cv::KeyPoint & kptIn, const float & depthIn = 0.0f, const cv::Mat & descriptorIn = cv::Mat(), int cameraIndexIn = 0):
		kpt(kptIn),
		depth(depthIn),
		descriptor(descriptorIn),
		cameraIndex(cameraIndexIn)
	{
		//UDEBUG("kpt=(%f,%f) depth=%f, camIndex=%d", kpt.pt.x, kpt.pt.y, depth, cameraIndex);
	}
	cv::KeyPoint kpt;     ///< 2D image keypoint.
	float depth;          ///< Depth at @ref kpt in meters, or 0 if unknown (monocular).
	cv::Mat descriptor;   ///< Optional descriptor for the keypoint (used when re-matching is enabled).
	int cameraIndex;      ///< Index into the frame's camera model list for multi-camera rigs.
};

/**
 * @class Optimizer
 * @brief Abstract base for pose-graph and bundle-adjustment optimizers.
 *
 * Optimizer is a factory + interface in front of several third-party back-ends
 * (@ref kTypeTORO "TORO", @ref kTypeG2O "g2o", @ref kTypeGTSAM "GTSAM",
 * @ref kTypeCeres "Ceres", @ref kTypeCVSBA "cvsba"). Use @ref create() to instantiate one
 * based on the @c Optimizer/Strategy parameter; use @ref isAvailable() to check whether a
 * given back-end was compiled in.
 *
 * Two families of methods are exposed:
 * - **Pose-graph optimization** (@ref optimize / @ref optimizeIncremental) — refines poses given
 *   relative-pose constraints. Subclasses override @ref optimize() with covariance output.
 * - **Bundle adjustment** (@ref optimizeBA) — jointly refines poses and 3D points using
 *   reprojection error. Subclasses override the lowest-level overload; the others are
 *   convenience wrappers that fill in models/correspondences from @ref Signature data.
 *
 * Common knobs (iterations, robust kernels, 2D-vs-3D, etc.) are configured through
 * @ref parseParameters() or per-attribute setters and apply to whichever back-end is selected.
 */
class RTABMAP_CORE_EXPORT Optimizer
{
public:
	/** @brief Graph-optimizer back-end identifier. */
	enum Type {
		kTypeUndef = -1, ///< Unspecified / invalid.
		kTypeTORO = 0,   ///< TORO (tree-based relaxation).
		kTypeG2O = 1,    ///< g2o (general graph optimization, supports BA).
		kTypeGTSAM = 2,  ///< GTSAM (factor graphs, iSAM2-style incremental).
		kTypeCeres = 3,  ///< Ceres Solver (nonlinear least squares, supports BA).
		kTypeCVSBA = 4   ///< cvsba (sparse bundle adjustment only).
	};

	/**
	 * @brief Returns whether @p type was compiled in (its third-party dependency was found).
	 *
	 * @ref kTypeUndef is treated as unavailable. @ref create() falls back through this check
	 * to pick the first available back-end when the requested one is missing.
	 */
	static bool isAvailable(Optimizer::Type type);

	/**
	 * @brief Factory: build an optimizer from a @ref ParametersMap.
	 *
	 * Reads @c Optimizer/Strategy from @p parameters; falls back to the default strategy if
	 * the chosen back-end isn't compiled in. Caller owns the returned pointer.
	 */
	static Optimizer * create(const ParametersMap & parameters);

	/** @brief Factory: build an optimizer of a specific @p type. Caller owns the result. */
	static Optimizer * create(Optimizer::Type type, const ParametersMap & parameters = ParametersMap());

	/**
	 * @brief Extracts the connected component reachable from @p fromId.
	 *
	 * Walks @p linksIn breadth-first starting at @p fromId and copies every visited pose
	 * (from @p posesIn) and every traversed link into @p posesOut / @p linksOut. Use this to
	 * isolate the subgraph that actually affects @p fromId before calling @ref optimize(),
	 * since the back-ends require a single connected component.
	 */
	void getConnectedGraph(
			int fromId,
			const std::map<int, Transform> & posesIn,
			const std::multimap<int, Link> & linksIn,
			std::map<int, Transform> & posesOut,
			std::multimap<int, Link> & linksOut) const;

public:
	virtual ~Optimizer() {}

	/** @brief Returns the concrete back-end identifier (one of @ref Type). */
	virtual Type type() const = 0;

	/// @name Getters for tunables shared across back-ends.
	/// @{
	int iterations() const {return iterations_;}             ///< Max solver iterations.
	bool isSlam2d() const {return slam2d_;}                  ///< True if optimizing in SE(2) instead of SE(3).
	bool isCovarianceIgnored() const {return covarianceIgnored_;} ///< If true, all edges share an identity information matrix.
	double epsilon() const {return epsilon_;}                ///< Convergence threshold on cost decrease.
	bool isRobust() const {return robust_;}                  ///< If true, use a robust kernel / switchable factors against bad loop closures.
	bool priorsIgnored() const {return priorsIgnored_;}      ///< If true, unary priors on poses are dropped.
	bool landmarksIgnored() const {return landmarksIgnored_;}///< If true, landmark/marker observations are dropped.
	float gravitySigma() const {return gravitySigma_;}       ///< Std-dev (rad) of the gravity prior on roll/pitch; 0 disables it.
	/// @}

	/// @name Setters mirroring the corresponding getters.
	/// @{
	void setIterations(int iterations) {iterations_ = iterations;}
	void setSlam2d(bool enabled) {slam2d_ = enabled;}
	void setCovarianceIgnored(bool enabled) {covarianceIgnored_ = enabled;}
	void setEpsilon(double epsilon) {epsilon_ = epsilon;}
	void setRobust(bool enabled) {robust_ = enabled;}
	void setPriorsIgnored(bool enabled) {priorsIgnored_ = enabled;}
	void setLandmarksIgnored(bool enabled) {landmarksIgnored_ = enabled;}
	void setGravitySigma(float value) {gravitySigma_ = value;}
	/// @}

	/**
	 * @brief Reads shared knobs from @p parameters and applies them to this instance.
	 *
	 * Subclasses override to additionally read back-end-specific keys (e.g. @c g2o/Solver,
	 * @c GTSAM/Optimizer); they should call this base implementation first.
	 */
	virtual void parseParameters(const ParametersMap & parameters);

	/**
	 * @brief Pose-graph optimization that grows the graph one node at a time.
	 *
	 * Inserts poses in @p poses iteration order, propagating odometry edges (@c Neighbor /
	 * @c NeighborMerged) directly and triggering a call to @ref optimize() whenever a loop
	 * closure is added. A final full @ref optimize() pass anchors at @p rootId. Useful for
	 * warm-starting heavily-deformed initial guesses where a single-shot @ref optimize()
	 * can diverge.
	 *
	 * @param rootId  Pose whose absolute transform is held fixed.
	 * @param poses   Initial poses keyed by id.
	 * @param constraints  Relative-pose constraints (see @ref Link::Type).
	 * @param intermediateGraphes  Optional: appended per outer iteration for debug/visualization.
	 * @param finalError  Optional: written with the solver's final cost.
	 * @param iterationsDone  Optional: written with the solver's actual iteration count.
	 * @return Refined poses (same key set as @p poses), or empty on failure.
	 */
	std::map<int, Transform> optimizeIncremental(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & constraints,
			std::list<std::map<int, Transform> > * intermediateGraphes = 0,
			double * finalError = 0,
			int * iterationsDone = 0);

	/**
	 * @brief Pose-graph optimization (single shot).
	 *
	 * Convenience overload that discards the output covariance. See the covariance-returning
	 * overload below for parameter docs.
	 */
	std::map<int, Transform> optimize(
				int rootId,
				const std::map<int, Transform> & poses,
				const std::multimap<int, Link> & constraints,
				std::list<std::map<int, Transform> > * intermediateGraphes = 0,
				double * finalError = 0,
				int * iterationsDone = 0);

	/**
	 * @brief Pose-graph optimization with marginal covariance of @p rootId.
	 *
	 * This is the primary back-end entry point — concrete subclasses override it. The base
	 * implementation just emits an error.
	 *
	 * @param rootId  Pose to hold fixed during optimization.
	 * @param poses   Initial pose estimates.
	 * @param constraints  Relative-pose constraints between pose ids (and optionally to landmarks).
	 * @param outputCovariance  Output: 6x6 covariance of the last optimized pose w.r.t. @p rootId
	 *                          (3x3 for 2D mode). Filled only if the back-end supports it.
	 * @param intermediateGraphes  Optional: appended at each iteration for debug/visualization.
	 * @param finalError  Optional: written with the solver's final cost.
	 * @param iterationsDone  Optional: written with the solver's actual iteration count.
	 * @return Refined poses, or empty on failure.
	 */
	virtual std::map<int, Transform> optimize(
				int rootId,
				const std::map<int, Transform> & poses,
				const std::multimap<int, Link> & constraints,
				cv::Mat & outputCovariance,
				std::list<std::map<int, Transform> > * intermediateGraphes = 0,
				double * finalError = 0,
				int * iterationsDone = 0);

	/**
	 * @brief Bundle adjustment: jointly refine poses and 3D points (back-end-level entry point).
	 *
	 * Concrete subclasses (g2o, Ceres, cvsba) override this; the base implementation errors out.
	 * The other @ref optimizeBA() overloads ultimately funnel here.
	 *
	 * @param rootId  Pose to hold fixed. If negative, ALL poses other than the (positive) lowest id
	 *                are held fixed.
	 * @param poses   Initial pose estimates keyed by frame id.
	 * @param links   Edges used to define the BA problem topology.
	 * @param models  Camera model(s) per frame; for stereo, @c Tx must be set on the model
	 *                (= -baseline*fx). Multi-camera rigs have multiple entries per frame.
	 * @param points3DMap  In/out: world 3D points keyed by word id; refined on return.
	 * @param wordReferences  Observations: `<wordId, <frameId, FeatureBA>>`. See @ref FeatureBA.
	 * @param outliers  Optional output: word ids dropped by the back-end's outlier rejection.
	 * @return Refined poses, or empty on failure.
	 */
	virtual std::map<int, Transform> optimizeBA(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, std::vector<CameraModel> > & models,
			std::map<int, cv::Point3f> & points3DMap,
			const std::map<int, std::map<int, FeatureBA> > & wordReferences,
			std::set<int> * outliers = 0);

	/**
	 * @brief BA wrapper that derives camera models and correspondences from signatures.
	 *
	 * Builds @c models per frame from each @ref Signature's @ref SensorData (mono or stereo,
	 * with stereo-baseline encoded in Tx), calls @ref computeBACorrespondences() to populate
	 * @p points3DMap / @p wordReferences from the signatures' words, then delegates to the
	 * back-end overload above.
	 *
	 * @param rematchFeatures  If true, re-match descriptors across linked frames before
	 *                         building correspondences (more accurate, more expensive).
	 * @param registrationParameters  Forwarded to @ref RegistrationVis when re-matching.
	 */
	std::map<int, Transform> optimizeBA(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, Signature> & signatures,
			std::map<int, cv::Point3f> & points3DMap,
			std::map<int, std::map<int, FeatureBA> > & wordReferences,
			bool rematchFeatures = false,
			const ParametersMap & registrationParameters = ParametersMap());

	/** @brief BA convenience wrapper: like the overload above but ignores the
	 *  refined 3D points and observation map. */
	std::map<int, Transform> optimizeBA(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, Signature> & signatures,
			bool rematchFeatures = false,
			const ParametersMap & registrationParameters = ParametersMap());

	/**
	 * @brief Refine a single two-frame link via BA.
	 *
	 * Sets the @c from frame at identity and the @c to frame at @c link.transform(), then runs
	 * BA over the supplied 3D points and observations. Returns the refined relative transform,
	 * or @c link.transform() unchanged on failure.
	 */
	Transform optimizeBA(
			const Link & link,
			const CameraModel & model,
			std::map<int, cv::Point3f> & points3DMap,
			const std::map<int, std::map<int, FeatureBA> > & wordReferences,
			std::set<int> * outliers = 0);

	/**
	 * @brief Build BA correspondences (3D points + per-frame observations) from signatures.
	 *
	 * For each link, matches words between the two signatures with @ref RegistrationVis,
	 * triangulates / lifts depth into world coordinates using the frames' initial poses, and
	 * populates @p points3DMap (one entry per word id) and @p wordReferences (one entry per
	 * (word, frame) observation).
	 *
	 * @param rematchFeatures  If true, descriptors are re-matched between frames instead of
	 *                         relying on pre-existing word ids — more robust to feature drift.
	 * @param useLinkTransformAsGuess  If true, the link's transform seeds the PnP guess
	 *                                 instead of estimating it from scratch.
	 * @param registrationParameters  Forwarded to @ref RegistrationVis (estimation type,
	 *                                inlier counts, NNDR, etc.).
	 */
	void computeBACorrespondences(
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, Signature> & signatures,
			std::map<int, cv::Point3f> & points3DMap,
			std::map<int, std::map<int, FeatureBA > > & wordReferences,
			bool rematchFeatures = false,
			bool useLinkTransformAsGuess = false,
			ParametersMap registrationParameters = ParametersMap());

protected:
	Optimizer(
			int iterations         = Parameters::defaultOptimizerIterations(),
			bool slam2d            = Parameters::defaultRegForce3DoF(),
			bool covarianceIgnored = Parameters::defaultOptimizerVarianceIgnored(),
			double epsilon         = Parameters::defaultOptimizerEpsilon(),
			bool robust            = Parameters::defaultOptimizerRobust(),
			bool priorsIgnored     = Parameters::defaultOptimizerPriorsIgnored(),
			bool landmarksIgnored  = Parameters::defaultOptimizerLandmarksIgnored(),
			float gravitySigma     = Parameters::defaultOptimizerGravitySigma());
	Optimizer(const ParametersMap & parameters);

private:
	int iterations_;
	bool slam2d_;
	bool covarianceIgnored_;
	double epsilon_;
	bool robust_;
	bool priorsIgnored_;
	bool landmarksIgnored_;
	float gravitySigma_;
};

} /* namespace rtabmap */
#endif /* OPTIMIZER_H_ */
