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

#ifndef UTIL3D_MOTION_ESTIMATION_H_
#define UTIL3D_MOTION_ESTIMATION_H_

#include <rtabmap/core/rtabmap_core_export.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>

namespace rtabmap
{

namespace util3d
{

/**
 * @brief Estimates a 6-DOF camera transform from 3D-2D point correspondences using PnP RANSAC.
 *
 * This function estimates the motion (transform) between two views by solving the Perspective-n-Point (PnP)
 * problem using 3D points from one frame and their corresponding 2D keypoints in another frame.
 * It optionally refines the result, computes covariance of the pose estimate, and handles degenerate cases.
 *
 * @param words3A 3D points in frame A, indexed by feature ID.
 * @param words2B 2D keypoints in frame B, indexed by feature ID (shared with `words3A`).
 * @param cameraModel Intrinsic and extrinsic parameters of the camera (must be valid).
 * @param minInliers Minimum number of inliers required to accept the estimated transform. If the value is <4, it is set internally to 4.
 * @param iterations Number of RANSAC iterations for PnP.
 * @param reprojError Maximum allowed reprojection error (in pixels) to consider a point an inlier.
 * @param flagsPnP Flags to control the `cv::solvePnPRansac` behavior (e.g., `cv::SOLVEPNP_ITERATIVE`).
 * @param refineIterations Number of iterations for non-linear optimization (set to 0 to disable refinement).
 * @param varianceMedianRatio Index divisor used to select the robust variance threshold from sorted error residuals (e.g., 4 → use the 25% percentile).
 * @param maxVariance Maximum allowed median variance (linear error). Estimates with higher variance are rejected.
 * @param guess Initial guess for the camera pose (must not be null). Typically from odometry or motion model.
 * @param words3B Optional 3D points in frame B (if available). Used to better estimate 3D errors and variances.
 * @param covariance Optional output pointer for the estimated 6x6 pose covariance matrix.
 *                   The matrix contains linear variance in the top-left 3x3 and angular variance in the bottom-right 3x3.
 * @param matchesOut Optional output vector of all matched IDs used (regardless of inlier status).
 * @param inliersOut Optional output vector of matched IDs that were determined to be inliers.
 * @param splitLinearCovarianceComponents Whether to split and compute variance for X, Y, Z components separately.
 *
 * @return The estimated transformation from frame B to frame A. If estimation fails or is rejected due to variance,
 *         a null transform is returned (i.e., `transform.isNull()` will be true).
 *
 * @note
 * - If `words3B` is provided, 3D variance is computed by comparing reprojected points to actual transformed points.
 * - If `words3B` is empty, variance is estimated using reprojection error only.
 * - The function assumes the camera model's local transform is known and factored into the pose estimation.
 *
 * @see cv::solvePnPRansac
 */
Transform RTABMAP_CORE_EXPORT estimateMotion3DTo2D(
			const std::map<int, cv::Point3f> & words3A,
			const std::map<int, cv::KeyPoint> & words2B,
			const CameraModel & cameraModel,
			int minInliers = 10,
			int iterations = 100,
			double reprojError = 5.,
			int flagsPnP = 0,
			int pnpRefineIterations = 1,
			int varianceMedianRatio = 4,
			float maxVariance = 0,
			const Transform & guess = Transform::getIdentity(),
			const std::map<int, cv::Point3f> & words3B = std::map<int, cv::Point3f>(),
			cv::Mat * covariance = 0, // mean reproj error if words3B is not set
			std::vector<int> * matchesOut = 0,
			std::vector<int> * inliersOut = 0,
			bool splitLinearCovarianceComponents = false);

/**
 * @brief Estimates the 3D-to-2D motion (pose) transformation between a set of 3D points and their corresponding 2D keypoints using the OpenGV library.
 *
 * This function uses a robust multi-camera Perspective-n-Point (PnP) algorithm to estimate the transformation from a 3D point cloud (scene A)
 * to a set of 2D keypoints (scene B) given the corresponding camera models and initial pose guess.
 * The method supports multiple camera models and uses RANSAC with OpenGV for outlier rejection.
 *
 * @param words3A                3D points in the source frame (scene A), indexed by feature ID.
 * @param words2B                2D keypoints in the destination frame (scene B), indexed by feature ID.
 * @param cameraModels           List of camera models (multi-camera rig setup) for the destination frame.
 * @param samplingPolicy         Sampling strategy (0 = auto, 1 = any, 2 = homogeneous multi-camera).
 * @param minInliers             Minimum number of inliers required to consider the estimated transform as valid.
 * @param iterations             Maximum number of RANSAC iterations.
 * @param reprojError            Reprojection error threshold used by RANSAC.
 * @param flagsPnP               PnP flags (not used internally).
 * @param refineIterations       Number of pose refinement iterations after RANSAC (not used internally).
 * @param varianceMedianRatio    Divider used to compute median-based variance from the error distribution.
 * @param maxVariance            Maximum allowed variance to accept the transform. Higher values permit more noisy estimates.
 * @param guess                  Initial guess of the transformation.
 * @param words3B                Optional 3D points in destination frame (scene B) to evaluate the covariance using 3D 
 *                               correspondences, otherwise covariance is estimated from reprojection errors.
 * @param covariance             Optional output 6x6 covariance matrix of the estimated transform.
 * @param matchesOut             Optional output: matches grouped per camera.
 * @param inliersOut             Optional output: inliers grouped per camera.
 * @param splitLinearCovarianceComponents If true, linear covariance is split into separate x/y/z components.
 *
 * @return The estimated Transform from scene A to scene B. Returns a null Transform if estimation fails or variance exceeds threshold.
 *
 * @note This function requires RTAB-Map to be built with OpenGV support.
 *
 * @warning The function assumes all camera models have the same image width and valid intrinsic parameters.
 *
 * @see https://github.com/laurentkneip/opengv
 */
Transform estimateMotion3DTo2D(
			const std::map<int, cv::Point3f> & words3A,
			const std::map<int, cv::KeyPoint> & words2B,
			const std::vector<CameraModel> & cameraModels,
			unsigned int samplingPolicy,
			int minInliers,
			int iterations,
			double reprojError,
			int flagsPnP,
			int refineIterations,
			int varianceMedianRatio,
			float maxVariance,
			const Transform & guess,
			const std::map<int, cv::Point3f> & words3B,
			cv::Mat * covariance,
			std::vector<std::vector<int> > * matchesOut,
			std::vector<std::vector<int> > * inliersOut,
			bool splitLinearCovarianceComponents);
/**
 * @brief Estimates the 3D-to-2D motion (pose) transformation between a set of 3D points and their corresponding 2D keypoints using the OpenGV library.
 * @see estimateMotion3DTo2D(), the only difference is that output matches and inliers are combined in same vector instead of per camera
 */
Transform RTABMAP_CORE_EXPORT estimateMotion3DTo2D(
			const std::map<int, cv::Point3f> & words3A,
			const std::map<int, cv::KeyPoint> & words2B,
			const std::vector<CameraModel> & cameraModels,
			unsigned int samplingPolicy = 0, // 0=AUTO, 1=ANY, 2=HOMOGENEOUS
			int minInliers = 10,
			int iterations = 100,
			double reprojError = 5.,
			int flagsPnP = 0,
			int pnpRefineIterations = 1,
			int varianceMedianRatio = 4,
			float maxVariance = 0,
			const Transform & guess = Transform::getIdentity(),
			const std::map<int, cv::Point3f> & words3B = std::map<int, cv::Point3f>(),
			cv::Mat * covariance = 0, // mean reproj error if words3B is not set
			std::vector<int> * matchesOut = 0,
			std::vector<int> * inliersOut = 0,
			bool splitLinearCovarianceComponents = false);

Transform RTABMAP_CORE_EXPORT estimateMotion3DTo3D(
			const std::map<int, cv::Point3f> & words3A,
			const std::map<int, cv::Point3f> & words3B,
			int minInliers = 10,
			double inliersDistance = 0.1,
			int iterations = 100,
			int refineIterations = 5,
			cv::Mat * covariance = 0,
			std::vector<int> * matchesOut = 0,
			std::vector<int> * inliersOut = 0);

void RTABMAP_CORE_EXPORT solvePnPRansac(
		const std::vector<cv::Point3f> & objectPoints,
		const std::vector<cv::Point2f> & imagePoints,
		const cv::Mat & cameraMatrix,
		const cv::Mat & distCoeffs,
		cv::Mat & rvec,
		cv::Mat & tvec,
		bool useExtrinsicGuess,
		int iterationsCount,
		float reprojectionError,
		int minInliersCount,
		std::vector<int> & inliers,
		int flags,
		int refineIterations = 1,
		float refineSigma = 3.0f);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_TRANSFORMS_H_ */
