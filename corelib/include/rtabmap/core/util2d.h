/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef UTIL2D_H_
#define UTIL2D_H_

#include <rtabmap/core/rtabmap_core_export.h>

#include <opencv2/core/core.hpp>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/CameraModel.h>
#include <vector>

namespace rtabmap
{

/** 
 * @brief This namespace contains 2D image processing utilities.
 */
namespace util2d
{

/**
 * @brief Computes the Sum of Squared Differences (SSD) between two image patches.
 *
 * This function calculates the pixel-wise squared differences between corresponding elements 
 * in two input windows and accumulates the result into a single score. It supports grayscale 
 * 8-bit, 32-bit float, and 16-bit 2-channel short images (e.g., optical flow or stereo blocks).
 *
 * @param windowLeft Left input image patch.
 * @param windowRight Right input image patch (must be the same size and type as windowLeft).
 * @return The SSD score (a lower score indicates higher similarity).
 *
 * @throws Assertion failure if the input types or dimensions don't match.
 * 
 * Supported types:
 * - CV_8UC1 (grayscale image)
 * - CV_32FC1 (floating-point grayscale)
 * - CV_16SC2 (2-channel signed short vectors; average of both channels used)
 */
float RTABMAP_CORE_EXPORT ssd(const cv::Mat & windowLeft, const cv::Mat & windowRight);

/**
 * @brief Computes the Sum of Absolute Differences (SAD) between two image patches.
 *
 * This function calculates the absolute pixel intensity difference between two windows 
 * and accumulates the result. It supports grayscale 8-bit, 32-bit float, and 16-bit 
 * 2-channel short images.
 *
 * @param windowLeft Left input image patch.
 * @param windowRight Right input image patch (must match windowLeft in size and type).
 * @return The SAD score (a lower score indicates greater similarity).
 *
 * @throws Assertion failure if the input types or dimensions are incompatible.
 * 
 * Supported types:
 * - CV_8UC1 (grayscale image)
 * - CV_32FC1 (floating-point grayscale)
 * - CV_16SC2 (2-channel signed short vectors; average of both channels used)
 */
float RTABMAP_CORE_EXPORT sad(const cv::Mat & windowLeft, const cv::Mat & windowRight);

/**
 * @brief Computes stereo correspondences between left and right images using a pyramidal window-based matching approach.
 *
 * This function estimates the right image positions of a set of corners detected in the left image using block matching.
 * It supports both Sum of Absolute Differences (SAD) and Sum of Squared Differences (SSD) as the matching criteria,
 * and uses a coarse-to-fine strategy over an image pyramid for robustness and subpixel accuracy.
 *
 * @param leftImage          The left image (grayscale 8 bits or CV_8UC1).
 * @param rightImage         The right image (same type and size as `leftImage`).
 * @param leftCorners        The list of 2D points in the left image for which correspondences are to be found.
 * @param[out] status        Output status vector indicating the success of correspondence for each point.
 *                           (1: valid correspondence found, 0: no valid match)
 * @param winSize            The size of the search window (should be odd). Will be made odd internally if not.
 *                           Minimum size is 3.
 * @param maxLevel           The maximum level of the image pyramid to use for coarse-to-fine search.
 * @param iterations         Number of iterations for subpixel refinement (gradient descent-like search).
 * @param minDisparityF      Minimum allowed disparity (float). Defines search range.
 * @param maxDisparityF      Maximum allowed disparity (float). Defines search range.
 * @param ssdApproach        If true, uses SSD (Sum of Squared Differences) as matching cost.
 *                           If false, uses SAD (Sum of Absolute Differences).
 *
 * @return A vector of 2D points corresponding to `leftCorners` but in the right image.
 *         The size of the output matches the input `leftCorners`. Invalid or rejected points are flagged in `status`.
 *
 * @note - Both input images must be rectified (i.e., correspondences lie along epipolar lines).
 *       - This function modifies the search window to ensure it is odd-sized (required for accurate matching).
 *       - Subpixel accuracy is achieved using iterative matching and local refinement.
 *       - The disparity range is adaptive and refined at each pyramid level.
 *       - For accurate results, ensure good quality corner detection and well-rectified input images.
 *
 * @see cv::buildOpticalFlowPyramid, cv::getRectSubPix
 */
std::vector<cv::Point2f> RTABMAP_CORE_EXPORT calcStereoCorrespondences(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		std::vector<unsigned char> & status,
		cv::Size winSize = cv::Size(6,3),
		int maxLevel = 3,
		int iterations = 5,
		float minDisparity = 0.0f,
		float maxDisparity = 64.0f,
		bool ssdApproach = true);

/**
 * @brief Computes sparse optical flow using a pyramidal Lucas-Kanade method constrained to the x-axis.
 *
 * This function is a customized version of OpenCV's `cv::calcOpticalFlowPyrLK`, modified specifically
 * for stereo matching scenarios. It assumes that the `prevImg` is the left stereo image and `nextImg`
 * is the right stereo image. The optical flow is computed only along the x-axis (i.e., horizontal direction),
 * which is typically valid in rectified stereo image pairs.
 *
 * @note
 * The key modification to the original Lucas-Kanade implementation is the following:
 * Instead of computing the flow in both x and y directions, the y-displacement is **forced to zero**:
 * @code
 * // Original:
 * cv::Point2f delta((float)((A12*b2 - A22*b1) * D), (float)((A12*b1 - A11*b2) * D));
 *
 * // Modified:
 * cv::Point2f delta((float)((A12*b2 - A22*b1) * D), 0);
 * @endcode
 * This ensures that flow estimation is constrained along the epipolar lines (x-direction only).
 *
 * @param _prevImg Input image from the previous frame (or left stereo image). Supports pyramid or raw image.
 * @param _nextImg Input image from the next frame (or right stereo image). Supports pyramid or raw image.
 * @param _prevPts Vector of 2D points for which the flow needs to be found (in `prevImg`).
 * @param _nextPts Output vector of 2D points containing the calculated new positions (in `nextImg`).
 *                 If `OPTFLOW_USE_INITIAL_FLOW` is passed, it should contain initial guesses.
 * @param _status Output status vector. Each element is set to 1 if flow for the corresponding features
 *                has been found, 0 otherwise.
 * @param _err Optional output vector. Contains error or min eigenvalue values (depending on flags).
 * @param winSize Size of the search window at each pyramid level.
 * @param maxLevel 0-based maximal pyramid level number. If set to 0, pyramids are not used (single level).
 * @param criteria Termination criteria for iterative search algorithm (maxCount and/or epsilon).
 * @param flags Operation flags:
 *              - `OPTFLOW_USE_INITIAL_FLOW`: Use initial `nextPts` values.
 *              - `OPTFLOW_LK_GET_MIN_EIGENVALS`: Output minimum eigenvalues instead of error.
 * @param minEigThreshold Minimum eigenvalue threshold for rejecting unstable flow vectors.
 *
 * @see cv::calcOpticalFlowPyrLK
 * @see https://github.com/opencv/opencv/blob/4.x/modules/video/src/lkpyramid.cpp
 */
void RTABMAP_CORE_EXPORT calcOpticalFlowPyrLKStereo( cv::InputArray _prevImg, cv::InputArray _nextImg,
                           cv::InputArray _prevPts, cv::InputOutputArray _nextPts,
                           cv::OutputArray _status, cv::OutputArray _err,
                           cv::Size winSize = cv::Size(15,3), int maxLevel = 3,
						   cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01),
						   int flags = 0, double minEigThreshold = 1e-4 );

/**
 * @brief Computes the disparity map from a pair of stereo images.
 *
 * This function calculates a dense disparity map from the provided left and right stereo images.
 * It assumes that the stereo pair is rectified and of the same size. The left image can be 
 * either grayscale (CV_8UC1) or color (CV_8UC3), while the right image must be grayscale (CV_8UC1).
 *
 * If the left image is in color, it is first converted to grayscale before disparity computation.
 * The actual disparity computation is delegated to a `StereoDense` object created using the provided parameters.
 *
 * @param leftImage The left image of the stereo pair. Can be grayscale or BGR color.
 * @param rightImage The right image of the stereo pair. Must be grayscale.
 * @param parameters A map of parameters used to configure the stereo matching algorithm.
 * 
 * @return A `cv::Mat` representing the computed disparity map. Some algorithms, like 
 *         StereoBM or StereoSGBM compute 16-bit fixed-point disparity map (CV_16SC1) (where each 
 *         disparity value has 4 fractional bits), whereas other algorithms output 32-bit 
 *         floating-point (CV_32FC1) disparity map.
 * 
 * @throws Assertion failure if:
 *         - Either image is empty.
 *         - Image sizes do not match.
 *         - Image types are incompatible.
 *
 * @note The returned disparity map contains disparity values for each pixel in the left image.
 *       Pixels with no match may be set to 0 or a negative value depending on the stereo algorithm.
 * 
 * @see rtabmap::StereoDense
 */
cv::Mat RTABMAP_CORE_EXPORT disparityFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
	    const ParametersMap & parameters = ParametersMap());

/**
 * @brief Converts a disparity map to a depth map using stereo camera parameters.
 *
 * This function takes a disparity map and computes a corresponding depth map using the formula:
 * \f[ \text{depth} = \frac{\text{baseline} \times \text{focal length}}{\text{disparity}} \f]
 *
 * - Disparity values of 0 or less are ignored (depth is left as 0).
 * - For CV_16UC1 depth type, depth is scaled to millimeters (i.e., multiplied by 1000).
 * - Values that exceed the 16-bit unsigned max value (65535) are counted and ignored.
 *
 * @param disparity The input disparity map (CV_32FC1 or CV_16SC1).
 * @param fx The focal length in pixels (typically from camera intrinsic parameters).
 * @param baseline The distance between the stereo cameras in meters.
 * @param type The desired output depth image type: CV_32FC1 (meters) or CV_16UC1 (millimeters).
 * @return A depth image of the same resolution as the input disparity map.
 *
 * @warning Logs a warning if any computed depth values exceed the maximum allowed by the CV_16UC1 format (65535 mm).
 * 
 * @throws Assertion failure if the disparity map is empty, has unsupported type, or output type is invalid.
 */
cv::Mat RTABMAP_CORE_EXPORT depthFromDisparity(const cv::Mat & disparity,
		float fx, float baseline,
		int type = CV_32FC1); // CV_32FC1 or CV_16UC1

/**
 * @brief Computes a depth map from stereo image pairs using optical flow tracking.
 *
 * This function estimates the depth of features tracked between the left and right rectified stereo images
 * by computing sparse optical flow (via Lucas-Kanade) between provided feature points in the left image.
 * It uses stereo triangulation based on the tracked correspondences and known camera intrinsics.
 *
 * @param leftImage         Grayscale rectified left image (CV_8UC1).
 * @param rightImage        Grayscale rectified right image (CV_8UC1), must be same size as leftImage.
 * @param leftCorners       Feature points (e.g., corners) detected in the left image.
 * @param fx                Focal length of the camera in pixels (must be > 0).
 * @param baseline          Distance between the left and right camera centers in meters (must be > 0).
 * @param flowWinSize       Window size used for optical flow (e.g., 15 for 15x15).
 * @param flowMaxLevel      Maximum pyramid level for optical flow.
 * @param flowIterations    Maximum number of iterations for the iterative search algorithm in optical flow.
 * @param flowEps           Desired accuracy for optical flow termination criteria.
 *
 * @return A depth map (CV_32FC1) with the same size as the input images. Pixels corresponding to successfully
 *         tracked features have valid depth values, while others are zero.
 *
 * @see cv::calcOpticalFlowPyrLK, rtabmap::util2d::depthFromStereoCorrespondences
 */
cv::Mat RTABMAP_CORE_EXPORT depthFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		float fx,
		float baseline,
		int flowWinSize = 9,
		int flowMaxLevel = 4,
		int flowIterations = 20,
		double flowEps = 0.02);

/**
 * @brief Computes a disparity map from stereo correspondences between two images.
 *
 * This function calculates the disparity map based on the given stereo correspondences 
 * (the left and right corners of features) and stores the resulting disparity values in 
 * a matrix. The disparity for each point is computed as the horizontal difference 
 * between the corresponding points in the left and right images.
 * The function also accepts a mask to specify which points to include in the disparity computation.
 *
 * @param[in] disparitySize The size of the output disparity map.
 * @param[in] leftCorners The list of points in the left image where features are detected.
 * @param[in] rightCorners The list of points in the right image corresponding to the points in `leftCorners`.
 * @param[in] mask A vector of flags indicating which correspondences to use for the disparity computation. 
 *                 An empty vector means all correspondences are included.
 * 
 * @return A `cv::Mat` of type `CV_32FC1` representing the computed disparity map. Each pixel value corresponds 
 *         to the disparity (horizontal difference between left and right image points) at that location.
 * 
 * @note The disparity is computed as the horizontal difference between corresponding points in the left and right 
 *       images, specifically the difference in the x-coordinates of the points.
 * @note The disparity map is returned in floating point format, where the value represents the disparity in pixels.
 * 
 * @warning The function checks that all points are within the bounds of the disparity map.
 */
cv::Mat RTABMAP_CORE_EXPORT disparityFromStereoCorrespondences(
		const cv::Size & disparitySize,
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const std::vector<unsigned char> & mask);

/**
 * @brief Computes a sparse depth map from corresponding stereo feature points.
 *
 * This function uses known corresponding 2D feature points from rectified stereo images
 * to estimate depth via triangulation, using the disparity between matched points.
 * The computed depth values are placed into a depth map at the locations of the left image points.
 *
 * @param leftImage      The left rectified grayscale image (used for image size reference).
 * @param leftCorners    Feature points detected in the left image.
 * @param rightCorners   Corresponding feature points in the right image (same size as leftCorners).
 * @param mask           Optional binary mask indicating which correspondences are valid (1 = valid).
 *                       If empty, all correspondences are considered valid.
 * @param fx             Focal length of the camera in pixels (must be > 0).
 * @param baseline       Distance between the stereo cameras in meters (must be > 0).
 *
 * @return A sparse depth map (CV_32FC1) of the same size as the input image.
 *         Pixels corresponding to valid matches will contain depth values (in meters),
 *         while all others remain zero.
 *
 * @note It assumes stereo images are rectified and only x-axis disparity is present.
 * @see rtabmap::util2d::depthFromStereoImages
 */
cv::Mat RTABMAP_CORE_EXPORT depthFromStereoCorrespondences(
		const cv::Mat & leftImage,
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const std::vector<unsigned char> & mask,
		float fx, float baseline);

/**
 * @brief Converts a 32-bit float depth image (in meters) to a 16-bit unsigned depth image (in millimeters).
 *
 * This function converts each valid depth value from meters to millimeters 
 * (by multiplying by 1000.0) and stores it as an unsigned 16-bit integer. 
 * Depth values outside the valid range (greater than 65535 mm) are clipped 
 * to zero and counted. A warning is printed if such values are found.
 *
 * @param depth32F Input depth image of type CV_32FC1, where depth is in meters.
 *                 May be empty.
 *
 * @return A 16-bit unsigned depth image (CV_16UC1) in millimeters. Returns an
 *         empty matrix if input is empty.
 *
 * @warning If depth values exceed 65535 mm, they are ignored and a warning is issued.
 *
 * @note It's assumed that the input image is in meters (commonly used format
 *       for 32-bit depth maps). If it’s already in millimeters, do not use this function.
 * 
 * @see rtabmap::util2d::cvtDepthToFloat()
 */
cv::Mat RTABMAP_CORE_EXPORT cvtDepthFromFloat(const cv::Mat & depth32F);

/**
 * @brief Converts a 16-bit unsigned depth image (in millimeters) to a 32-bit float depth image (in meters).
 *
 * This function converts each depth value from millimeters to meters by dividing
 * by 1000.0. Useful when working with floating point depth operations or to 
 * standardize depth formats for computation or storage.
 *
 * @param depth16U Input depth image of type CV_16UC1, where depth is in millimeters.
 *                 May be empty.
 *
 * @return A 32-bit float depth image (CV_32FC1) with depth values in meters.
 *         Returns an empty matrix if input is empty.
 *
 * @note Use this function when you  want to convert from 16-bit mm format to floating point meter format.
 * 
 * @see rtabmap::util2d::cvtDepthFromFloat()
 */
cv::Mat RTABMAP_CORE_EXPORT cvtDepthToFloat(const cv::Mat & depth16U);

/**
 * @brief Retrieves a depth value from a depth image at a subpixel coordinate.
 * 
 * This function samples the depth value from a depth image (either in 16-bit unsigned integers 
 * representing millimeters or 32-bit floats representing meters) at a floating-point (x, y) 
 * coordinate. The value can be optionally smoothed using a weighted neighborhood, and fallback 
 * estimation from neighbors is possible if the depth at the target pixel is invalid or zero.
 * 
 * @param depthImage Input depth image. Must be of type CV_16UC1 (depth in mm) or CV_32FC1 (depth in meters).
 * @param x The subpixel X-coordinate in the image.
 * @param y The subpixel Y-coordinate in the image.
 * @param smoothing If true, apply a weighted 3x3 smoothing around the pixel.
 * @param depthErrorRatio Maximum acceptable depth difference ratio used during smoothing and fallback estimation.
 * @param estWithNeighborsIfNull If true, and the target pixel has an invalid depth, estimate it from valid neighboring pixels.
 * 
 * @return The depth value at the given (x, y) location (in meters), or 0 if it cannot be determined.
 * 
 * @note 
 * - The function applies bounds checking on the input coordinates.
 * - If `smoothing` is enabled, a weighted average using a 3x3 kernel is computed.
 * - If `estWithNeighborsIfNull` is enabled and the pixel has no valid depth, the value is estimated
 *   from 4-connected neighbors using consistency constraints based on `depthErrorRatio`.
 * - Pixels with zero or invalid (NaN/Inf) depth are ignored in estimation and smoothing.
 * 
 */
float RTABMAP_CORE_EXPORT getDepth(
		const cv::Mat & depthImage,
		float x, float y,
		bool smoothing,
		float depthErrorRatio = 0.02f, //ratio
		bool estWithNeighborsIfNull = false);

/**
 * @defgroup RoiComputation Region of Interest (ROI) Computation
 * Functions to compute the ROI in an image using ratio-based cropping.
 * @{
 */

/**
 * @brief Computes a region of interest (ROI) in the image using string-defined ratios.
 *
 * @param image Input image.
 * @param roiRatios A string of 4 space-separated float values representing the ROI ratios:
 *        - left, right, top, bottom.
 *        - Each ratio should be between 0 and 1.
 *        - For example, "0.1 0.1 0.2 0.2" removes 10% from the left/right and 20% from the top/bottom.
 * @return A cv::Rect defining the ROI, or an empty rectangle if input is invalid.
 */
cv::Rect RTABMAP_CORE_EXPORT computeRoi(const cv::Mat & image, const std::string & roiRatios);

/**
 * @brief Computes a region of interest (ROI) from an image size and a string of ratios.
 *
 * @param imageSize The size of the image (width x height).
 * @param roiRatios A string with 4 space-separated float values indicating cropping ratios:
 *        - left, right, top, bottom.
 * @return The computed ROI rectangle.
 */
cv::Rect RTABMAP_CORE_EXPORT computeRoi(const cv::Size & imageSize, const std::string & roiRatios);

/**
 * @brief Computes a region of interest (ROI) in the image using float vector-defined ratios.
 *
 * @param image Input image.
 * @param roiRatios A vector of 4 float values (left, right, top, bottom) representing the ROI.
 *        - Each value should be between 0 and 1.
 * @return A cv::Rect defining the cropped region.
 */
cv::Rect RTABMAP_CORE_EXPORT computeRoi(const cv::Mat & image, const std::vector<float> & roiRatios);

/**
 * @brief Computes a region of interest (ROI) using float ratios and the image size.
 *
 * @param imageSize The size of the image (width x height).
 * @param roiRatios A vector of 4 float values representing ratios for cropping:
 *        - [left, right, top, bottom].
 *        - Each must be in the range [0, 1), and valid (i.e., left + right < 1, top + bottom < 1).
 * @return The computed ROI rectangle.
 */
cv::Rect RTABMAP_CORE_EXPORT computeRoi(const cv::Size & imageSize, const std::vector<float> & roiRatios);

/** @} */  // end of RoiComputation group

/**
 * @brief Downsamples an image by a given decimation factor.
 *
 * If the image is a depth image (CV_32FC1 or CV_16UC1), it ensures that
 * decimation is done precisely without interpolation. For other types, OpenCV's
 * `resize` with `INTER_AREA` is used.
 *
 * @param image The input image to decimate.
 * @param decimation The downsampling factor (must be >= 1).
 * @return The decimated image. If the decimation factor is 1 or the image is empty, the original image is returned.
 *
 * @note For depth images, the image size must be divisible by the decimation factor.
 * @throw Assertion failure if decimation is < 1 or size mismatch for depth images.
 */
cv::Mat RTABMAP_CORE_EXPORT decimate(const cv::Mat & image, int d);

/**
 * @brief Upsamples a depth image using bilinear interpolation with depth consistency check.
 *
 * Performs a depth-aware interpolation for CV_32FC1 or CV_16UC1 types. It checks whether
 * the surrounding four corner values are consistent within a `depthErrorRatio`, and if so,
 * performs bilinear interpolation. For other image types, OpenCV's `resize` is used.
 *
 * @param image The input image to interpolate.
 * @param factor The interpolation factor (must be >= 1).
 * @param depthErrorRatio Acceptable ratio of depth difference to allow interpolation.
 * @return The interpolated image. If the factor is 1 or the image is empty, the original image is returned.
 *
 * @note This function is intended for depth images. If corners have invalid or inconsistent depth values,
 *        interpolation is skipped at that patch.
 * @throw Assertion failure if factor < 1 or invalid parameters.
 */
cv::Mat RTABMAP_CORE_EXPORT interpolate(const cv::Mat & image, int factor, float depthErrorRatio = 0.02f);

/**
 * @brief Registers a depth image to a different camera frame (typically RGB).
 *
 * This function aligns the given depth image to the coordinate frame of an RGB camera
 * using the intrinsic parameters of both cameras and the extrinsic transformation between them.
 * The output is a depth image aligned to the RGB image dimensions and field of view.
 *
 * The function assumes the depth is either in meters (`CV_32FC1`) or in millimeters (`CV_16UC1`),
 * and it returns a registered depth image in the same format.
 *
 * @param depth            The input depth image (type `CV_16UC1` in mm or `CV_32FC1` in meters).
 * @param depthK           Intrinsic matrix of the depth camera (3x3, type `CV_64FC1`).
 * @param colorSize        Size of the target RGB image (the output will match this size).
 * @param colorK           Intrinsic matrix of the RGB camera (3x3, type `CV_64FC1`).
 * @param transform        Transform from the RGB camera frame to depth camera frame.
 *
 * @return                 A depth image registered to the RGB image space, with the same type as the input depth.
 *
 * @throw Assertion failure if input validation fails (e.g., empty image, incorrect types or sizes).
 *
 * @note If multiple depth points project to the same RGB pixel, the closest one is kept.
 *       This helps with occlusion handling when registering sparse/depth data.
 */
cv::Mat RTABMAP_CORE_EXPORT registerDepth(
		const cv::Mat & depth,
		const cv::Mat & depthK,
		const cv::Size & colorSize,
		const cv::Mat & colorK,
		const rtabmap::Transform & transform);

/**
 * @brief Fills holes in the depth image using linear interpolation.
 * 
 * This function iterates through the depth image and fills in holes (missing depth values) by interpolating from 
 * surrounding valid depth values. It considers both horizontal and vertical neighbors to interpolate missing data.
 * The maximum hole size and the error ratio are used to control the filling process. The function works with 
 * both 16-bit (mm) and 32-bit (meters) depth images.
 *
 * @param depth The input depth image (CV_16UC1 or CV_32FC1).
 * @param maximumHoleSize The maximum size of a hole to be filled, in pixels.
 * @param errorRatio The ratio used to calculate the allowed depth error for interpolation.
 * 
 * @return A new depth image with holes filled.
 * 
 * @note The input depth image must be of type CV_16UC1 (depth in millimeters) or CV_32FC1 (depth in meters).
 * The filled output is of the same type as the input.
 */
cv::Mat RTABMAP_CORE_EXPORT fillDepthHoles(
		const cv::Mat & depth,
		int maximumHoleSize = 1,
		float errorRatio = 0.02f);

/**
 * @brief Fill holes in a registered depth image using linear interpolation.
 *
 * This function attempts to fill invalid (zero-valued) pixels in a registered
 * depth image by looking at neighboring pixels in vertical and/or horizontal
 * directions. Optionally, it can also fill "double holes" (gaps of two consecutive
 * pixels) if `fillDoubleHoles` is enabled.
 *
 * The interpolation is only performed if the depth difference between the
 * neighbors is within 1% of their average, to avoid introducing invalid depth values.
 *
 * @param[in,out] registeredDepth The input/output registered depth image (CV_16UC1).
 *                                Modified in-place to fill in missing depth values.
 * @param vertical If true, the function tries to fill holes in vertical direction.
 * @param horizontal If true, the function tries to fill holes in horizontal direction.
 * @param fillDoubleHoles If true, the function also attempts to fill two-pixel wide holes
 *                        by linearly interpolating between values spaced by two pixels.
 *
 * @note This function assumes that the depth image contains unsigned 16-bit values,
 *       where a value of 0 represents an invalid or missing depth value. Pixels on the 
 *       contour are not interpolated.
 * 
 * @warning The input matrix must be of type CV_16UC1, or the function will trigger
 *          an assertion failure.
 *
 * @see rtabmap::util2d::registerDepth(), rtabmap::util2d::fillDepthHoles()
 */
void RTABMAP_CORE_EXPORT fillRegisteredDepthHoles(
		cv::Mat & depthRegistered,
		bool vertical,
		bool horizontal,
		bool fillDoubleHoles = false);

/**
 * @brief Applies a 2D fast bilateral filter to a depth image.
 * 
 * This function is a 2D adaptation of the pcl::FastBilateralFiltering algorithm.
 * It processes a depth image (either CV_32FC1 or CV_16UC1) using a bilateral
 * filter with spatial and range standard deviations `sigmaS` and `sigmaR`.
 * The method includes optimizations such as early division and efficient 
 * 3D grid accumulation with smoothing.
 * 
 * @param depth Input depth image. Must be of type CV_32FC1 (meters) or CV_16UC1 (millimeters).
 * @param sigmaS Spatial standard deviation. Controls the amount of smoothing in the image plane.
 * @param sigmaR Range standard deviation. Controls the amount of smoothing in the depth (z) dimension.
 * @param earlyDivision If true, applies early normalization to improve performance.
 * 
 * @return Filtered depth image as a CV_32FC1 Mat. If the input image is empty or contains no valid depth,
 *         an empty Mat is returned.
 * 
 * @note This implementation relies on an auxiliary 3D data structure and uses trilinear interpolation 
 * for reconstructing smoothed values. Pixels with non-finite or invalid depths are ignored.
 * 
 * @warning The result is always a CV_32FC1 image, even if the input is CV_16UC1. If input depth's valid pixels have 
 *          all exact same value, the result will be retruned with all zeros (issue from the original implementation).
 */
cv::Mat RTABMAP_CORE_EXPORT fastBilateralFiltering(
		const cv::Mat & depth,
		float sigmaS = 15.0f,
		float sigmaR = 0.05f,
		bool earlyDivision = false);

/**
 * @brief Automatic brightness and contrast optimization with optional histogram clipping.
 * 
 * This function automatically adjusts the brightness and contrast of the input image based on 
 * its histogram. It optionally clips a percentage of the darkest and brightest parts of the histogram 
 * to reduce the influence of outliers (similar to "auto levels" in photo editors).
 * 
 * @param[in] src Input image. Must be of type CV_8UC1 (grayscale), CV_8UC3 (BGR), or CV_8UC4 (BGRA).
 * @param[in] mask Optional mask. Only non-zero mask pixels are considered in histogram computation.
 * @param[in] clipLowHistPercent Percentage of the lowest histogram range to clip. Use 0 to disable.
 * @param[in] clipHighHistPercent Percentage of the highest histogram range to clip. Use 0 to disable.
 * @param[out] alphaOut Optional pointer to store the computed alpha (contrast scale factor).
 * @param[out] betaOut Optional pointer to store the computed beta (brightness shift factor).
 * 
 * @return A new image with automatically adjusted brightness and contrast. 
 *         The image will have the same size and number of channels as the input.
 * 
 * @note For BGRA input images, the alpha (transparency) channel is preserved and not modified.
 * 
 * @see Original discussion: https://answers.opencv.org/question/75510/how-to-make-auto-adjustmentsbrightness-and-contrast-for-image-android-opencv-image-correction/
 */
cv::Mat RTABMAP_CORE_EXPORT brightnessAndContrastAuto(
		const cv::Mat & src,
		const cv::Mat & mask,
		float clipLowHistPercent=0,
		float clipHighHistPercent=0,
		float * alphaOut = 0,
		float * betaOut = 0);

/**
 * @brief Performs exposure fusion on a set of input images.
 * 
 * This function blends multiple images with different exposures into a single
 * well-exposed image using the Mertens exposure fusion algorithm. It leverages
 * OpenCV's `createMergeMertens()` method (available in OpenCV 3 and above).
 * 
 * @param images A vector of input images (typically CV_8UC3) to be fused. All images
 *        should have the same size and type.
 * 
 * @return A fused color image (CV_8UC3) with enhanced exposure. If OpenCV version is
 *         below 3, the function returns a clone of the first image in the input vector.
 * 
 * @note The output image is normalized to 8-bit color (0–255). Exposure fusion requires
 *       OpenCV 3.0 or later.
 * 
 * @warning If OpenCV version is lower than 3, exposure fusion is not performed and a warning is issued.
 */
cv::Mat RTABMAP_CORE_EXPORT exposureFusion(
	const std::vector<cv::Mat> & images);

/**
 * @brief Converts a color from HSV (Hue, Saturation, Value) to RGB.
 * 
 * This function takes HSV color values and converts them to their corresponding
 * RGB representation using standard sector-based color conversion.
 * 
 * @param[out] r Pointer to a float where the resulting red component (0.0–1.0) will be stored.
 * @param[out] g Pointer to a float where the resulting green component (0.0–1.0) will be stored.
 * @param[out] b Pointer to a float where the resulting blue component (0.0–1.0) will be stored.
 * @param[in]  h Hue angle in degrees (0–360). Defines the color type.
 * @param[in]  s Saturation (0.0–1.0). 0 is grayscale, 1 is full color.
 * @param[in]  v Value (brightness) (0.0–1.0). 0 is black, 1 is full brightness.
 * 
 * @note This function assumes `h` is in degrees. If `s` is 0, the resulting color is grayscale,
 *       with R=G=B=V.
 * 
 * @warning The output RGB values are in the 0.0 to 1.0 range, not 0–255.
 */
void RTABMAP_CORE_EXPORT HSVtoRGB( float *r, float *g, float *b, float h, float s, float v );

/**
 * @brief Applies Non-Maximum Suppression (NMS) to a set of keypoints.
 * 
 * This function filters a set of input keypoints by applying a grid-based non-maximum suppression
 * (NMS) algorithm. It retains only the strongest keypoints (based on response value) while 
 * ensuring that no two retained points are within a certain distance from each other.
 * 
 * @param[in]  ptsIn          Input vector of keypoints.
 * @param[in]  descriptorsIn  Corresponding descriptors for the input keypoints. Can be empty.
 * @param[out] ptsOut         Output vector of keypoints after NMS filtering.
 * @param[out] descriptorsOut Output descriptors corresponding to the filtered keypoints.
 * @param[in]  dist_thresh    Minimum allowed distance between retained keypoints (suppression radius).
 * @param[in]  img_width      Width of the image on which the keypoints are based.
 * @param[in]  img_height     Height of the image on which the keypoints are based.
 * 
 * @note Keypoints are suppressed if they are within `dist_thresh` pixels of a stronger keypoint.
 * @note If `descriptorsIn` is empty, descriptor output will remain empty.
 * @note Assumes all keypoints lie within the image dimensions provided.
 */
void RTABMAP_CORE_EXPORT NMS(
		const std::vector<cv::KeyPoint> & ptsIn,
		const cv::Mat & descriptorsIn,
		std::vector<cv::KeyPoint> & ptsOut,
		cv::Mat & descriptorsOut,
		int dist_thresh, int img_width, int img_height);

/**
 * @brief Applies the SSC (Suppression via Square Covering) algorithm to spatially select keypoints.
 *
 * This function selects a subset of keypoints that are uniformly distributed across the image
 * using a square covering method and binary search optimization to achieve a desired number of keypoints.
 *
 * @param[in] keypoints    Input vector of keypoints to select from.
 * @param[in] maxKeypoints Desired number of output keypoints. The algorithm attempts to select this many,
 *                         within a tolerance range.
 * @param[in] tolerance    Relative tolerance for the number of output keypoints (e.g., 0.1 allows ±10%).
 * @param[in] cols         Width of the image in pixels.
 * @param[in] rows         Height of the image in pixels.
 * @param[in] indx         Optional vector of indices to use instead of the original keypoints ordering.
 *                         If provided, should be the same size as `keypoints`. This allows for applying
 *                         SSC to a pre-sorted subset (e.g., top-N keypoints).
 *
 * @return A vector of indices corresponding to the selected keypoints in the input `keypoints` vector.
 *
 * @note The algorithm operates by covering the image with a grid of cells and retaining the most confident
 *       keypoint in each uncovered cell while suppressing nearby keypoints within a computed square radius.
 * @note Uses binary search to find the optimal suppression radius that yields `maxKeypoints` (± `tolerance`).
 * @note Works best when `keypoints` are pre-sorted by response strength (e.g., strongest first).
 * @note If the `indx` vector is provided, the returned indices refer to the original list, not just `indx`.
 */
std::vector<int> RTABMAP_CORE_EXPORT SSC(
	const std::vector<cv::KeyPoint> & keypoints,
	int maxKeypoints,
	float tolerance,
	int cols,
	int rows,
	const std::vector<int> & indx = {});

/**
 * @brief Rotates the input RGB and depth images to make them appear upright based on the camera's roll angle.
 *
 * This function uses the camera's extrinsic parameters to determine if the captured image is rotated
 * (e.g., sideways or upside-down), and rotates it appropriately (by 90°, 180°, or 270°) to correct orientation.
 * It also updates the associated camera model to reflect the new transformation and adjusted image size.
 *
 * @param[in,out] model The camera model associated with the images. It will be updated to reflect the new orientation.
 * @param[in,out] rgb   The RGB image to be rotated if necessary.
 * @param[in,out] depth The depth image to be rotated if necessary.
 *
 * @return True if the images were rotated, false if no rotation was needed or the pitch angle is too large for a reliable decision.
 *
 * @note The function:
 * - Ignores rotation if pitch > π/4 (too ambiguous to determine "up").
 * - Assumes roll is responsible for rotation (i.e., sideways capture).
 * - Applies necessary rotation and updates the camera intrinsics accordingly.
 * - Supports image types: RGB and depth must be valid OpenCV `cv::Mat`.
 * - Respects image transparency and depth values during rotation.
 *
 * @warning This function assumes that the camera's local transform includes the standard optical rotation.
 *
 * @see rtabmap::CameraModel, cv::transpose, cv::flip
 */
bool RTABMAP_CORE_EXPORT rotateImagesUpsideUpIfNecessary(
	CameraModel & model,
	cv::Mat & rgb,
	cv::Mat & depth);

} // namespace util2d
} // namespace rtabmap

#endif /* UTIL2D_H_ */
