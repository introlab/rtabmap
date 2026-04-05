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

#ifndef UTIL3D_CORRESPONDENCES_H_
#define UTIL3D_CORRESPONDENCES_H_

#include <rtabmap/core/rtabmap_core_export.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/features2d/features2d.hpp>
#include <set>
#include <map>
#include <list>

namespace rtabmap
{

namespace util3d
{

/**
 * @brief Extracts 3D point correspondences between two sets of labeled 3D points.
 * 
 * This function identifies common point IDs (keys) between two multimap structures containing 
 * `pcl::PointXYZ` points. For each shared key that appears **exactly once** in both input maps, 
 * and where both corresponding points are finite, the matched points are added to two output point clouds.
 * 
 * The resulting `cloud1` and `cloud2` point clouds will contain points with a one-to-one correspondence, 
 * useful for geometric registration (e.g., ICP).
 * 
 * @param words1 Input multimap of point ID to 3D point for the first dataset.
 * @param words2 Input multimap of point ID to 3D point for the second dataset.
 * @param cloud1 Output point cloud (corresponding to points from `words1`).
 * @param cloud2 Output point cloud (corresponding to points from `words2`).
 * 
 * @note Only keys that appear exactly once in both `words1` and `words2`, and whose associated
 *       `pcl::PointXYZ` entries are finite, will be included in the output clouds.
 */
void RTABMAP_CORE_EXPORT extractXYZCorrespondences(const std::multimap<int, pcl::PointXYZ> & words1,
									  const std::multimap<int, pcl::PointXYZ> & words2,
									  pcl::PointCloud<pcl::PointXYZ> & cloud1,
									  pcl::PointCloud<pcl::PointXYZ> & cloud2);

/**
 * @brief Extracts reliable 3D point correspondences between two sets of labeled 3D points using RANSAC filtering.
 * 
 * This function finds correspondences between `words1` and `words2` based on shared unique keys. For each common key
 * that appears exactly once in both maps, and where the corresponding 3D points are finite, a candidate correspondence
 * is formed. If more than 7 such pairs exist, RANSAC is used via OpenCV’s `cv::findFundamentalMat` to reject outliers
 * based on the geometric consistency of the 2D projections.
 * 
 * Only the inlier correspondences determined by RANSAC are returned in the output point clouds `cloud1` and `cloud2`.
 * 
 * @param words1 Input multimap of point ID to `pcl::PointXYZ` for the first set of 3D features.
 * @param words2 Input multimap of point ID to `pcl::PointXYZ` for the second set of 3D features.
 * @param cloud1 Output point cloud containing inlier points from `words1`.
 * @param cloud2 Output point cloud containing inlier points from `words2`.
 * 
 * @note At least 8 valid point correspondences are required for RANSAC to compute a fundamental matrix.
 *       If fewer than 8 valid matches exist, the function does not modify the output clouds.
 * 
 * @warning Only 2D `(x, y)` components of the 3D points are used for RANSAC filtering.
 * 
 */
void RTABMAP_CORE_EXPORT extractXYZCorrespondencesRANSAC(const std::multimap<int, pcl::PointXYZ> & words1,
									  const std::multimap<int, pcl::PointXYZ> & words2,
									  pcl::PointCloud<pcl::PointXYZ> & cloud1,
									  pcl::PointCloud<pcl::PointXYZ> & cloud2);

/**
 * @brief Extracts 3D point correspondences from 2D pixel matches using depth images.
 * 
 * This function projects matched 2D keypoints (pixel correspondences) from two RGB-D images into 3D space 
 * using the provided camera intrinsic parameters. Only valid and finite 3D points are retained. If a `maxDepth` 
 * threshold is provided, points farther than this threshold are excluded.
 * 
 * The function returns two synchronized point clouds, `cloud1` and `cloud2`, where each point pair at the 
 * same index corresponds to a match between the two views.
 * 
 * @param correspondences List of 2D point correspondences between image 1 and image 2.
 * @param depthImage1 Depth image corresponding to the first set of points (CV_32FC1 or CV_16UC1).
 * @param depthImage2 Depth image corresponding to the second set of points (same format as depthImage1).
 * @param cx Principal point x-coordinate (camera intrinsic).
 * @param cy Principal point y-coordinate (camera intrinsic).
 * @param fx Focal length in x-direction (camera intrinsic).
 * @param fy Focal length in y-direction (camera intrinsic).
 * @param maxDepth Maximum allowed depth for a correspondence to be considered valid. If <= 0, all depths are accepted.
 * @param cloud1 Output point cloud with 3D points corresponding to the first image.
 * @param cloud2 Output point cloud with 3D points corresponding to the second image.
 * 
 * @note 
 * - Both output point clouds are resized to contain only the valid 3D matches after filtering.
 * - Invalid, non-finite, or out-of-range depth values are automatically filtered out.
 * 
 */
void RTABMAP_CORE_EXPORT extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
									   const cv::Mat & depthImage1,
									   const cv::Mat & depthImage2,
									   float cx, float cy,
									   float fx, float fy,
									   float maxDepth,
									   pcl::PointCloud<pcl::PointXYZ> & cloud1,
									   pcl::PointCloud<pcl::PointXYZ> & cloud2);

/**
 * @brief Extracts 3D correspondences from 2D feature matches using `pcl::PointXYZ` organized point clouds.
 * 
 * This function projects 2D keypoint matches into 3D using the corresponding organized
 * point clouds (`cloud1` and `cloud2`). Points that are not finite are discarded.
 *
 * @param correspondences List of 2D point correspondences between image 1 and image 2.
 * @param cloud1 Organized `pcl::PointXYZ` point cloud corresponding to the first image.
 * @param cloud2 Organized `pcl::PointXYZ` point cloud corresponding to the second image.
 * @param inliers1 Output 3D points from `cloud1` corresponding to valid 2D matches.
 * @param inliers2 Output 3D points from `cloud2` corresponding to valid 2D matches.
 * 
 * @note 
 * - Only organized point clouds are supported (i.e., width × height layout must match 
 *   image size from which 2D keypoints were taken).
 * 
 */
void RTABMAP_CORE_EXPORT extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<pcl::PointXYZ> & cloud1,
							   const pcl::PointCloud<pcl::PointXYZ> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2);
/**
 * @brief Extracts 3D correspondences from 2D feature matches using `pcl::PointXYZRGB` organized point clouds.
 * 
 * This overload behaves identically to the `pcl::PointXYZ` version, but supports input point clouds
 * that contain RGB color data. The color is not used—only the XYZ fields are extracted.
 * 
 * @param correspondences List of matched 2D keypoints between two images.
 * @param cloud1 Organized `pcl::PointXYZRGB` point cloud for the first image.
 * @param cloud2 Organized `pcl::PointXYZRGB` point cloud for the second image.
 * @param inliers1 Output 3D points from `cloud1` corresponding to valid 2D matches.
 * @param inliers2 Output 3D points from `cloud2` corresponding to valid 2D matches.
 */
void RTABMAP_CORE_EXPORT extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<pcl::PointXYZRGB> & cloud1,
							   const pcl::PointCloud<pcl::PointXYZRGB> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2);

/**
 * @brief Counts the number of unique 3D point correspondences between two sets of word-indexed features.
 *
 * This function iterates over the unique keys (word IDs) in `wordsA` and checks if the same key exists
 * in `wordsB`. A pair is considered "unique" if both `wordsA` and `wordsB` contain exactly one 3D point
 * (i.e., one `pcl::PointXYZ`) associated with the same key.
 *
 * @param wordsA A multimap of word IDs to 3D points (e.g., from frame A).
 * @param wordsB A multimap of word IDs to 3D points (e.g., from frame B).
 * @return The number of unique pairs where both `wordsA` and `wordsB` contain exactly one point for a given word ID.
 */
int RTABMAP_CORE_EXPORT countUniquePairs(const std::multimap<int, pcl::PointXYZ> & wordsA,
					 const std::multimap<int, pcl::PointXYZ> & wordsB);

/**
 * @brief Filters pairs of 3D points by maximum depth along a specified axis and optionally removes duplicates.
 * 
 * This function takes two point clouds (`inliers1` and `inliers2`) containing corresponding 3D points,
 * and filters out pairs where either point exceeds a specified maximum depth value along the given axis.
 * It can also optionally remove duplicate points in the first point cloud.
 * 
 * @param[in,out] inliers1 The first point cloud of 3D points to be filtered. Points failing the filter will be removed.
 * @param[in,out] inliers2 The second point cloud of 3D points corresponding to `inliers1`. Points failing the filter will be removed.
 *                        Must be the same size as `inliers1`.
 * @param[in] maxDepth The maximum allowed depth value along the specified axis. Points with coordinate values greater or equal to
 *                     this value on that axis will be removed. If `maxDepth` is less or equal to zero, no filtering is performed.
 * @param[in] depthAxis The axis ('x', 'y', or 'z') along which to measure depth for filtering.
 * @param[in] removeDuplicates If `true`, duplicate points in `inliers1` (exact coordinate matches) will be removed.
 *                             Duplicates are detected only in `inliers1`.
 * 
 * @warning The function modifies `inliers1` and `inliers2` in place, replacing them with filtered versions.
 */
void RTABMAP_CORE_EXPORT filterMaxDepth(pcl::PointCloud<pcl::PointXYZ> & inliers1,
					pcl::PointCloud<pcl::PointXYZ> & inliers2,
					float maxDepth,
					char depthAxis,
					bool removeDuplicates);

/**
 * @brief Finds 2D point correspondences between two sets of keypoints based on matching word IDs.
 *
 * This function compares two multimap structures containing word IDs associated with `cv::KeyPoint`s.
 * It extracts correspondences where the same word ID appears **exactly once** in each set.
 *
 * @param[in] wordsA A multimap from word ID to keypoints in set A.
 * @param[in] wordsB A multimap from word ID to keypoints in set B.
 * @param[out] pairs A list of matching 2D point correspondences (Point2f) between wordsA and wordsB.
 *
 * @note Only unique word ID matches (count == 1 in both sets) are considered valid correspondences.
 *
 * @example
 * If `wordsA = [1 2 3 4 6 6]` and `wordsB = [1 1 2 4 5 6 6]`, the output `pairs` will contain correspondences
 * for IDs `2` and `4`, because only those have exactly one match in both sets.
 */
void RTABMAP_CORE_EXPORT findCorrespondences(
		const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<cv::Point2f, cv::Point2f> > & pairs);

/**
 * @brief Finds 3D point correspondences between two sets of points based on matching word IDs.
 *
 * This function compares two multimaps of 3D points (typically from different views or frames).
 * It returns point pairs where the same word ID appears **once** in both maps, the points are finite and valid,
 * and optionally filtered by a maximum X-depth.
 *
 * @param[in] words1 A multimap of word IDs to 3D points in the first set.
 * @param[in] words2 A multimap of word IDs to 3D points in the second set.
 * @param[out] inliers1 Output vector of 3D points from `words1` with valid correspondences.
 * @param[out] inliers2 Output vector of corresponding 3D points from `words2`.
 * @param[in] maxDepth Optional filter: only points with X-values in (0, maxDepth] are kept. Use <= 0 to disable.
 * @param[out] uniqueCorrespondences (Optional) Vector of word IDs corresponding to each pair.
 *
 * @note Only pairs with exactly one occurrence in each map and non-zero, finite coordinates are kept.
 */
void RTABMAP_CORE_EXPORT findCorrespondences(
		const std::multimap<int, cv::Point3f> & words1,
		const std::multimap<int, cv::Point3f> & words2,
		std::vector<cv::Point3f> & inliers1,
		std::vector<cv::Point3f> & inliers2,
		float maxDepth,
		std::vector<int> * uniqueCorrespondences = 0);

/**
 * @brief Finds 3D point correspondences between two sets of uniquely indexed 3D points.
 *
 * This overload works with `std::map`, where each word ID appears at most once. It finds matching IDs
 * and returns valid point pairs based on similar criteria to the multimap version.
 *
 * @param[in] words1 A map of word IDs to 3D points in the first set.
 * @param[in] words2 A map of word IDs to 3D points in the second set.
 * @param[out] inliers1 Output vector of 3D points from `words1` with valid correspondences.
 * @param[out] inliers2 Output vector of corresponding 3D points from `words2`.
 * @param[in] maxDepth Optional filter: only points with X-values in (0, maxDepth] are kept. Use <= 0 to disable.
 * @param[out] correspondences (Optional) Vector of word IDs corresponding to valid matched pairs.
 *
 * @note Finite, non-zero points are required. The function ignores word IDs not found in both sets.
 */
void RTABMAP_CORE_EXPORT findCorrespondences(
		const std::map<int, cv::Point3f> & words1,
		const std::map<int, cv::Point3f> & words2,
		std::vector<cv::Point3f> & inliers1,
		std::vector<cv::Point3f> & inliers2,
		float maxDepth,
		std::vector<int> * correspondences = 0);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_CORRESPONDENCES_H_ */
