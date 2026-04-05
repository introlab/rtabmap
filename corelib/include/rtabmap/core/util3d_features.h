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

#ifndef UTIL3D_FEATURES_H_
#define UTIL3D_FEATURES_H_

#include <rtabmap/core/rtabmap_core_export.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <list>
#include <map>

namespace rtabmap
{

namespace util3d
{

/**
 * @brief Projects 2D keypoints to 3D space using the provided depth image and camera models.
 * 
 * This function takes a vector of 2D keypoints and projects them into 3D space by using 
 * depth values from a depth image and the associated camera models. It supports multi-camera setups 
 * by assuming the depth image is horizontally stacked with sub-images corresponding to each camera.
 * 
 * If a depth value at a keypoint location is invalid or outside the specified depth range 
 * (`minDepth`, `maxDepth`), the output 3D point will be set to NaN.
 * 
 * @param keypoints      A vector of 2D keypoints (in image coordinates).
 * @param depth          The depth image (must be either `CV_32FC1` or `CV_16UC1`). 
 *                       For multiple cameras, the depth images should be horizontally concatenated.
 * @param cameraModels   A vector of camera models, one per camera. Each model must provide intrinsic
 *                       parameters and optionally a local transform to apply to the resulting 3D point.
 * @param minDepth       Minimum valid depth value. If negative, no minimum is enforced.
 * @param maxDepth       Maximum valid depth value. If zero or negative, no maximum is enforced.
 * 
 * @return A vector of 3D points (`cv::Point3f`) corresponding to the input keypoints. 
 *         If the depth is invalid or outside the valid range, the point will contain NaNs.
 * 
 * @throws Assertion failure if the depth image is empty or not of the expected type,
 *         or if the camera model vector is empty, or if camera index computation fails.
 */
std::vector<cv::Point3f> RTABMAP_CORE_EXPORT generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels,
		float minDepth = 0,
		float maxDepth = 0);
/**
 * @brief Projects 2D keypoints to 3D space using the provided depth image and camera model.
 * 
 * @see util3d::generateKeypoints3DDepth()
 */
std::vector<cv::Point3f> RTABMAP_CORE_EXPORT generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		const CameraModel & cameraModel,
		float minDepth = 0,
		float maxDepth = 0);

/**
 * @brief Projects 2D keypoints into 3D space using a disparity image and a stereo camera model.
 * 
 * This function computes 3D coordinates for each input 2D keypoint by using the disparity image 
 * and the stereo camera model. Invalid or out-of-range depth values result in 3D points with `NaN` components.
 * 
 * The function applies the local transform of the left camera (from the stereo model) to each valid 3D point,
 * if the transform is not null or identity.
 * 
 * @param keypoints        A vector of 2D keypoints (image coordinates) to be projected into 3D.
 * @param disparity        The disparity image (must be of type `CV_16SC1` or `CV_32F`). 
 *                         Disparity values should correspond to the keypoints' locations.
 * @param stereoCameraModel A valid stereo camera model that provides projection parameters 
 *                          and an optional local transform.
 * @param minDepth         Minimum depth threshold. If negative, no minimum constraint is applied.
 * @param maxDepth         Maximum depth threshold. If zero or negative, no maximum constraint is applied.
 * 
 * @return A vector of 3D points (`cv::Point3f`) corresponding to the input keypoints. 
 *         Points with invalid or out-of-range depth are returned as `(NaN, NaN, NaN)`.
 * 
 * @throws Assertion failure if the disparity image is empty or of incorrect type,
 *         or if the stereo camera model is not valid for projection.
 */
std::vector<cv::Point3f> RTABMAP_CORE_EXPORT generateKeypoints3DDisparity(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & disparity,
		const StereoCameraModel & stereoCameraModel,
		float minDepth = 0,
		float maxDepth = 0);

/**
 * @brief Computes 3D keypoints from corresponding 2D points in a stereo image pair.
 * 
 * This function triangulates 3D points from pairs of corresponding 2D points 
 * (`leftCorners`, `rightCorners`) using a given stereo camera model. It optionally applies 
 * a validity mask and filters 3D points by depth range.
 * 
 * For each point pair, the disparity is computed as the x-coordinate difference between 
 * left and right corners. Only positive disparities are considered valid. If a mask is 
 * provided, only entries with a non-zero value are processed.
 * 
 * The resulting 3D points are optionally transformed using the stereo camera model's local transform,
 * if one is defined and non-identity.
 * 
 * Invalid or out-of-range points are set to `(NaN, NaN, NaN)`.
 * 
 * @param leftCorners   A vector of 2D points from the left stereo image.
 * @param rightCorners  A vector of corresponding 2D points from the right stereo image.
 * @param model         The stereo camera model containing intrinsic parameters and optional local transform.
 * @param mask          (Optional) A binary mask indicating which matches are valid (non-zero = valid).
 *                      If empty, all matches are considered valid.
 * @param minDepth      Minimum allowed depth value. If negative, no minimum is applied.
 * @param maxDepth      Maximum allowed depth value. If zero or negative, no maximum is applied.
 * 
 * @return A vector of 3D points (`cv::Point3f`) corresponding to valid stereo matches.
 *         Invalid points or those outside the depth range are returned as `(NaN, NaN, NaN)`.
 * 
 * @throws Assertion failure if the input vectors are inconsistent in size,
 *         or if the stereo camera model is invalid (e.g., non-positive focal length or baseline).
 */
std::vector<cv::Point3f> RTABMAP_CORE_EXPORT generateKeypoints3DStereo(
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const StereoCameraModel & model,
		const std::vector<unsigned char> & mask = std::vector<unsigned char>(),
		float minDepth = 0,
		float maxDepth = 0);

std::map<int, cv::Point3f> RTABMAP_CORE_EXPORT generateWords3DMono(
		const std::map<int, cv::KeyPoint> & kpts,
		const std::map<int, cv::KeyPoint> & previousKpts,
		const CameraModel & cameraModel,
		Transform & cameraTransform,
		float ransacReprojThreshold = 3.0f,
		float ransacConfidence = 0.99f,
		int varianceMedianRatio = 4,
		const std::map<int, cv::Point3f> & refGuess3D = std::map<int, cv::Point3f>(),
		double * variance = 0,
		std::vector<int> * matchesOut = 0);

/**
 * @brief Aggregates word IDs and corresponding keypoints into a multimap.
 * 
 * This function pairs each word ID from the input list with the corresponding keypoint
 * from the input vector and stores them in a `std::multimap<int, cv::KeyPoint>`.
 * 
 * It is assumed that the `wordIds` list and the `keypoints` vector are of the same length
 * and ordered such that each word ID corresponds to the keypoint at the same index.
 * 
 * @param wordIds   A list of integer word IDs (e.g., visual word identifiers).
 * @param keypoints A vector of keypoints associated with the word IDs.
 * 
 * @return A multimap where each key is a word ID and the value is the corresponding `cv::KeyPoint`.
 *         Multiple keypoints can be associated with the same word ID.
 * 
 * @throws Assertion failure if `wordIds.size() != keypoints.size()`.
 */
std::multimap<int, cv::KeyPoint> RTABMAP_CORE_EXPORT aggregate(
		const std::list<int> & wordIds,
		const std::vector<cv::KeyPoint> & keypoints);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_FEATURES_H_ */
