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

#ifndef UTIL3D_REGISTRATION_H_
#define UTIL3D_REGISTRATION_H_

#include <rtabmap/core/rtabmap_core_export.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rtabmap/core/Transform.h>
#include <opencv2/core/core.hpp>

namespace rtabmap
{

namespace util3d
{

int RTABMAP_CORE_EXPORT getCorrespondencesCount(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_source,
							const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_target,
							float maxDistance);

/**
 * @brief Estimates the rigid 3D transformation between two point clouds using SVD.
 *
 * This function computes the transformation (rotation and translation) that best aligns
 * `cloud2` to `cloud1` using Singular Value Decomposition (SVD) based on point correspondences.
 * It assumes a one-to-one correspondence between points in the two clouds.
 *
 * Internally, it uses PCL's `TransformationEstimationSVD` to compute the 4x4 transformation matrix,
 * which is then converted to a `Transform` object.
 *
 * @param cloud1 Target point cloud (reference frame).
 * @param cloud2 Source point cloud to be aligned with `cloud1`.
 *               It must have the same number of points as `cloud1`, and the points should
 *               correspond to each other by index.
 *
 * @return A `Transform` representing the rigid-body transformation from `cloud1` to `cloud2`.
 *
 * @note This function does not perform any outlier rejection or correspondence estimation—
 *       it assumes that the input clouds are already matched appropriately.
 */
Transform RTABMAP_CORE_EXPORT transformFromXYZCorrespondencesSVD(
	const pcl::PointCloud<pcl::PointXYZ> & cloud1,
	const pcl::PointCloud<pcl::PointXYZ> & cloud2);

/**
 * @brief Estimates a rigid transformation between two point clouds using RANSAC with optional refinement.
 *
 * This function finds a 3D rigid-body transform from `cloud1` to `cloud2` using one-to-one point correspondences.
 * It applies a RANSAC-based outlier rejection and optionally refines the transformation with iterative model optimization.
 *
 * It also optionally returns the inlier indices used to compute the final model and an approximate 6x6 covariance matrix
 * of the transform.
 *
 * @param cloud1         Target point cloud (reference frame). Must contain at least 3 points and match `cloud2` in size.
 * @param cloud2         Source point cloud to align to `cloud1`. Must be the same size as `cloud1`.
 * @param inlierThreshold Maximum Euclidean distance (in meters) between corresponding points for them to be considered inliers.
 * @param iterations     Number of RANSAC iterations to perform.
 * @param refineIterations Number of refinement steps to perform after the initial RANSAC.
 *                         If set to 0, no refinement is done.
 * @param refineSigma    Multiplier for standard deviation used to adjust the inlier threshold during refinement.
 * @param inliersOut     Optional pointer to a vector that will receive the indices of the inlier correspondences.
 * @param covariance     Optional pointer to a 6x6 covariance matrix of the estimated transform (as `CV_64FC1`).
 *                       Will be identity if set and no inliers are found.
 *
 * @return A `Transform` representing the estimated pose from `cloud1` to `cloud2`.
 *         If no valid model is found, the returned transform will be identity.
 *
 * @note This function assumes a one-to-one correspondence between points in the two clouds
 *       (e.g., index `i` in `cloud1` corresponds to index `i` in `cloud2`).
 * @note If fewer than 3 points are provided or point counts do not match, the identity transform is returned.
 *
 * @warning Inlier refinement is sensitive to oscillation and may stop early if alternating inlier counts are detected.
 */
Transform RTABMAP_CORE_EXPORT transformFromXYZCorrespondences(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud1,
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud2,
		double inlierThreshold = 0.02,
		int iterations = 100,
		int refineModelIterations = 10,
		double refineModelSigma = 3.0,
		std::vector<int> * inliers = 0,
		cv::Mat * variance = 0);

/**
 * @defgroup ComputeVarianceAndCorrespondences Compute Variance and Correspondences of Two Point Clouds
 * @brief Computes the geometric variance and number of valid correspondences between two point clouds.
 *
 * This function estimates correspondences between two point clouds and computes a variance
 * measure (based on squared median error) for the inlier correspondences. Optionally filters correspondences by 
 * normal alignment angle if the point cloud has normals.
 *
 * The correspondence estimation uses PCL's reciprocal correspondence logic by default. The target and source clouds are automatically
 * chosen based on their sizes (the larger becomes the target to ensure optimal matching behavior).
 *
 * @param cloudA First input point cloud (used interchangeably with cloudB for matching).
 * @param cloudB Second input point cloud.
 * @param maxCorrespondenceDistance Maximum allowable Euclidean distance between correspondences.
 * @param maxCorrespondenceAngle Maximum angle (in radians) between normals for correspondences to be considered valid.
 *                                If ≤ 0, angle filtering is disabled.
 * @param[out] variance Output variance value (estimated using 2.1981 × median squared distance of inliers).
 * @param[out] correspondencesOut Output number of correspondences that passed all filters (distance and optional angle).
 * @param reciprocal If true, use reciprocal correspondences.
 *
 * @note This function chooses the target/source roles based on cloud size (the larger becomes the target).
 * @note The normal comparison is only applied if `maxCorrespondenceAngle > 0`.
 * @note The computed `variance` is based on a robust estimator using the median squared distance of correspondences.
 *
 * @see pcl::registration::CorrespondenceEstimation
 */
/**
 * @ingroup ComputeVarianceAndCorrespondences
 * @brief Compute with variance and correspondences of `pcl::PointNormal` point cloud type.
 */
void RTABMAP_CORE_EXPORT computeVarianceAndCorrespondences(
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloudA,
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloudB,
		double maxCorrespondenceDistance,
		double maxCorrespondenceAngle, // <=0 means that we don't care about normal angle difference
		double & variance,
		int & correspondencesOut,
		bool reciprocal);
/**
 * @ingroup ComputeVarianceAndCorrespondences
 * @brief Compute with variance and correspondences of `pcl::PointXYZINormal` point cloud type.
 */
void RTABMAP_CORE_EXPORT computeVarianceAndCorrespondences(
		const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr & cloudA,
		const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr & cloudB,
		double maxCorrespondenceDistance,
		double maxCorrespondenceAngle, // <=0 means that we don't care about normal angle difference
		double & variance,
		int & correspondencesOut,
		bool reciprocal);
/**
 * @ingroup ComputeVarianceAndCorrespondences
 * @brief Compute with variance and correspondences of `pcl::PointXYZ` point cloud type.
 */
void RTABMAP_CORE_EXPORT computeVarianceAndCorrespondences(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloudA,
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloudB,
		double maxCorrespondenceDistance,
		double & variance,
		int & correspondencesOut,
		bool reciprocal);
/**
 * @ingroup ComputeVarianceAndCorrespondences
 * @brief Compute with variance and correspondences of `pcl::PointXYZI` point cloud type.
 */
void RTABMAP_CORE_EXPORT computeVarianceAndCorrespondences(
		const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloudA,
		const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloudB,
		double maxCorrespondenceDistance,
		double & variance,
		int & correspondencesOut,
		bool reciprocal);

/**
 * @brief Performs Iterative Closest Point (ICP) alignment between two point clouds and returns the resulting transform.
 *
 * This function aligns the `cloud_source` to the `cloud_target` using PCL's ICP algorithm. It optionally supports 2D ICP,
 * which constrains the estimated transformation to the XY-plane with rotation about the Z-axis.
 *
 * The result is returned as a `Transform` representing the transformation from source to target.
 * The aligned version of the source cloud is written into `cloud_source_registered`.
 *
 * @param cloud_source The input source point cloud to align.
 * @param cloud_target The input target point cloud to align to.
 * @param maxCorrespondenceDistance Maximum distance threshold for point correspondences.
 * @param maximumIterations Maximum number of ICP iterations to perform.
 * @param[out] hasConverged Set to true if ICP converged to a solution; false otherwise.
 * @param[out] cloud_source_registered Output point cloud containing the source aligned to the target.
 * @param epsilon Convergence threshold for transformation changes between iterations (applied as squared value).
 * @param icp2D If true, enforces 2D ICP using only XY translation and Z rotation (ignores Z and X/Y rotation).
 *
 * @return Transform The estimated transformation from `cloud_source` to `cloud_target`.
 *
 * @note All input points in both clouds must be finite (i.e., no NaNs or infinite values).
 *
 * @see pcl::IterativeClosestPoint
 * @see pcl::registration::TransformationEstimation2D
 */
Transform RTABMAP_CORE_EXPORT icp(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_source,
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_target,
		double maxCorrespondenceDistance,
		int maximumIterations,
		bool & hasConverged,
		pcl::PointCloud<pcl::PointXYZ> & cloud_source_registered,
		float epsilon = 0.0f,
		bool icp2D = false);
/**
 * @brief Performs Iterative Closest Point (ICP) alignment between two point clouds and returns the resulting transform.
 * @see util3d::icp()
 */
Transform RTABMAP_CORE_EXPORT icp(
		const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_source,
		const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_target,
		double maxCorrespondenceDistance,
		int maximumIterations,
		bool & hasConverged,
		pcl::PointCloud<pcl::PointXYZI> & cloud_source_registered,
		float epsilon = 0.0f,
		bool icp2D = false);

/**
 * @brief Performs Iterative Closest Point (ICP) alignment using a point-to-plane error metric.
 *
 * This function aligns a source point cloud to a target point cloud using PCL's
 * point-to-plane ICP implementation with a linear least squares estimator. It returns
 * the estimated transformation from the source to the target.
 *
 * Optionally, if `icp2D` is true, the resulting transformation is projected to 3DoF (XY translation and rotation about Z).
 *
 * @param cloud_source Input source point cloud with normals.
 * @param cloud_target Input target point cloud with normals.
 * @param maxCorrespondenceDistance Maximum distance for considering point correspondences.
 * @param maximumIterations Maximum number of ICP iterations to perform.
 * @param[out] hasConverged Set to true if the ICP algorithm successfully converged.
 * @param[out] cloud_source_registered Output cloud representing the aligned source.
 * @param epsilon Convergence threshold for the transformation change (used as squared value).
 * @param icp2D If true, the result is projected to 2D (XY + Yaw only).
 *
 * @return The transformation from the source to the target cloud.
 *
 * @note All points and normals in both input clouds must be finite (no NaNs or infinities).
 *
 * @see pcl::IterativeClosestPoint
 * @see pcl::registration::TransformationEstimationPointToPlaneLLS
 */
Transform RTABMAP_CORE_EXPORT icpPointToPlane(
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud_source,
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud_target,
		double maxCorrespondenceDistance,
		int maximumIterations,
		bool & hasConverged,
		pcl::PointCloud<pcl::PointNormal> & cloud_source_registered,
		float epsilon = 0.0f,
		bool icp2D = false);
/**
 * @briefPerforms Iterative Closest Point (ICP) alignment using a point-to-plane error metric.
 * @see util3d::icpPointToPlane()
 */
Transform RTABMAP_CORE_EXPORT icpPointToPlane(
		const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr & cloud_source,
		const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr & cloud_target,
		double maxCorrespondenceDistance,
		int maximumIterations,
		bool & hasConverged,
		pcl::PointCloud<pcl::PointXYZINormal> & cloud_source_registered,
		float epsilon = 0.0f,
		bool icp2D = false);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_REGISTRATION_H_ */
