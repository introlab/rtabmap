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

#ifndef STEREO_H_
#define STEREO_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <opencv2/core/core.hpp>

namespace rtabmap {

/**
 * @class Stereo
 * @brief Sparse stereo matching using block matching
 * 
 * This class implements sparse stereo matching to find corresponding feature points
 * between stereo image pairs using block matching with a search window. Unlike dense
 * stereo matching, this class works with sparse feature points rather than computing
 * disparity for every pixel.
 * 
 * The algorithm uses a pyramidal approach for efficiency, searching for correspondences
 * within a specified disparity range using either SAD (Sum of Absolute Differences) or
 * SSD (Sum of Squared Differences) as the matching cost.
 * 
 * @note Input images must be grayscale (CV_8UC1).
 * @see StereoOpticalFlow for an alternative implementation using optical flow
 * @see StereoDense for dense stereo matching
 */
class RTABMAP_CORE_EXPORT Stereo {
public:
	/**
	 * @brief Factory method to create a Stereo instance
	 * 
	 * Creates a Stereo instance based on the stereo optical flow parameter
	 * in the provided parameters map. If optical flow is enabled, creates a
	 * StereoOpticalFlow instance; otherwise, creates a standard Stereo instance.
	 * 
	 * @param parameters Parameters map containing configuration values.
	 *                   The Parameters::kStereoOpticalFlow() parameter determines
	 *                   which implementation to create.
	 * @return Pointer to the created Stereo instance (caller owns the memory).
	 *         Returns StereoOpticalFlow if optical flow is enabled, otherwise Stereo.
	 */
	static Stereo * create(const ParametersMap & parameters = ParametersMap());

public:
	/**
	 * @brief Constructor
	 * 
	 * Initializes a Stereo instance with default parameter values and then
	 * parses the provided parameters map to override defaults.
	 * 
	 * @param parameters Optional parameters map containing configuration values.
	 *                   If empty, default values are used.
	 */
	Stereo(const ParametersMap & parameters = ParametersMap());
	
	/**
	 * @brief Virtual destructor
	 */
	virtual ~Stereo() {}

	/**
	 * @brief Parse parameters from a parameters map
	 * 
	 * Updates the algorithm's configuration based on the provided parameters map.
	 * Supported parameters:
	 * - Parameters::kStereoWinWidth() - Search window width
	 * - Parameters::kStereoWinHeight() - Search window height
	 * - Parameters::kStereoIterations() - Number of iterations
	 * - Parameters::kStereoMaxLevel() - Maximum pyramid level
	 * - Parameters::kStereoMinDisparity() - Minimum disparity value
	 * - Parameters::kStereoMaxDisparity() - Maximum disparity value
	 * - Parameters::kStereoSSD() - Use SSD instead of SAD
	 * 
	 * @param parameters Parameters map containing configuration values
	 */
	virtual void parseParameters(const ParametersMap & parameters);
	
	/**
	 * @brief Compute stereo correspondences using block matching
	 * 
	 * Finds corresponding points in the right stereo image for the given
	 * points in the left stereo image using block matching with a search window.
	 * The algorithm uses a pyramidal approach for efficiency.
	 * 
	 * @param leftImage Left stereo image (must be CV_8UC1 grayscale)
	 * @param rightImage Right stereo image (must be CV_8UC1 grayscale)
	 * @param leftCorners Input vector of feature points in the left image
	 * @param status Output vector indicating which correspondences are valid (1) or invalid (0).
	 *               The size matches leftCorners.size().
	 * @return Vector of corresponding points in the right image. The size matches leftCorners.size().
	 *         Invalid correspondences may have coordinates outside the image bounds.
	 * @note Both input images must be grayscale (CV_8UC1).
	 * @note The algorithm searches for correspondences within the disparity range
	 *       [minDisparity(), maxDisparity()] and uses a search window of size winSize().
	 */
	virtual std::vector<cv::Point2f> computeCorrespondences(
			const cv::Mat & leftImage,
			const cv::Mat & rightImage,
			const std::vector<cv::Point2f> & leftCorners,
			std::vector<unsigned char> & status) const;
#ifdef HAVE_OPENCV_CUDEV
	/**
	 * @brief Compute stereo correspondences using GPU (not implemented)
	 * 
	 * GPU version of computeCorrespondences. Currently not implemented for the
	 * standard Stereo class. Use StereoOpticalFlow with GPU enabled for GPU acceleration.
	 * 
	 * @param leftImage Left stereo image on GPU (must be CV_8UC1 grayscale)
	 * @param rightImage Right stereo image on GPU (must be CV_8UC1 grayscale)
	 * @param leftCorners Input vector of feature points in the left image
	 * @param status Output vector indicating which correspondences are valid
	 * @return Empty vector (GPU support not implemented for this class)
	 * @note This method always returns an empty vector and logs an error.
	 */
	virtual std::vector<cv::Point2f> computeCorrespondences(
			const cv::cuda::GpuMat & leftImage,
			const cv::cuda::GpuMat & rightImage,
			const std::vector<cv::Point2f> & leftCorners,
			std::vector<unsigned char> & status) const;
#endif

	/**
	 * @brief Get the search window size
	 * @return Search window size as cv::Size(winWidth_, winHeight_)
	 */
	cv::Size winSize() const {return cv::Size(winWidth_, winHeight_);}
	
	/**
	 * @brief Get the number of iterations
	 * @return Number of iterations for the matching algorithm
	 */
	int iterations() const   {return iterations_;}
	
	/**
	 * @brief Get the maximum pyramid level
	 * @return Maximum pyramid level used in the pyramidal approach
	 */
	int maxLevel() const     {return maxLevel_;}
	
	/**
	 * @brief Get the minimum disparity value
	 * @return Minimum disparity value to search (in pixels)
	 */
	float minDisparity() const {return minDisparity_;}
	
	/**
	 * @brief Get the maximum disparity value
	 * @return Maximum disparity value to search (in pixels)
	 */
	float maxDisparity() const {return maxDisparity_;}
	
	/**
	 * @brief Check if SSD (Sum of Squared Differences) is used
	 * @return true if SSD is used, false if SAD (Sum of Absolute Differences) is used
	 */
	bool winSSD() const      {return winSSD_;}
	
	/**
	 * @brief Check if GPU acceleration is enabled
	 * @return Always returns false for the base Stereo class
	 */
	virtual bool isGpuEnabled() const {return false;}

private:
	int winWidth_;      ///< Search window width (default: from Parameters::defaultStereoWinWidth())
	int winHeight_;     ///< Search window height (default: from Parameters::defaultStereoWinHeight())
	int iterations_;    ///< Number of iterations for matching (default: from Parameters::defaultStereoIterations())
	int maxLevel_;      ///< Maximum pyramid level (default: from Parameters::defaultStereoMaxLevel())
	float minDisparity_; ///< Minimum disparity value to search (default: from Parameters::defaultStereoMinDisparity())
	float maxDisparity_; ///< Maximum disparity value to search (default: from Parameters::defaultStereoMaxDisparity())
	bool winSSD_;       ///< Use SSD instead of SAD for matching cost (default: from Parameters::defaultStereoSSD())
};

/**
 * @class StereoOpticalFlow
 * @brief Sparse stereo matching using optical flow
 * 
 * This class implements sparse stereo matching using optical flow (Lucas-Kanade)
 * to find corresponding feature points between stereo image pairs. It extends the
 * base Stereo class with optical flow-based matching, which can be more robust
 * than simple block matching, especially for textured regions.
 * 
 * The algorithm uses pyramidal Lucas-Kanade optical flow to track feature points
 * from the left image to the right image, then filters the results based on
 * disparity constraints.
 * 
 * @note Input images must be grayscale (CV_8UC1).
 * @note GPU acceleration is available if RTAB-Map is built with OpenCV CUDA support.
 * @see Stereo for block matching-based implementation
 */
class RTABMAP_CORE_EXPORT StereoOpticalFlow : public Stereo {
public:
	/**
	 * @brief Constructor
	 * 
	 * Initializes a StereoOpticalFlow instance with default parameter values
	 * and then parses the provided parameters map to override defaults.
	 * 
	 * @param parameters Optional parameters map containing configuration values.
	 *                   If empty, default values are used.
	 */
	StereoOpticalFlow(const ParametersMap & parameters = ParametersMap());
	
	/**
	 * @brief Virtual destructor
	 */
	virtual ~StereoOpticalFlow() {}

	/**
	 * @brief Parse parameters from a parameters map
	 * 
	 * Updates the algorithm's configuration based on the provided parameters map.
	 * First calls the base class parseParameters(), then parses optical flow-specific
	 * parameters:
	 * - Parameters::kStereoEps() - Convergence threshold for optical flow
	 * - Parameters::kStereoGpu() - Enable GPU acceleration (requires OpenCV CUDA)
	 * 
	 * @param parameters Parameters map containing configuration values
	 * @note If GPU is enabled but RTAB-Map is not built with OpenCV CUDA support,
	 *       GPU will be automatically disabled and an error message will be logged.
	 */
	virtual void parseParameters(const ParametersMap & parameters);
	
	/**
	 * @brief Compute stereo correspondences using optical flow
	 * 
	 * Finds corresponding points in the right stereo image for the given
	 * points in the left stereo image using pyramidal Lucas-Kanade optical flow.
	 * The results are filtered based on disparity constraints (minDisparity to maxDisparity).
	 * 
	 * @param leftImage Left stereo image (must be CV_8UC1 grayscale)
	 * @param rightImage Right stereo image (must be CV_8UC1 grayscale)
	 * @param leftCorners Input vector of feature points in the left image
	 * @param status Output vector indicating which correspondences are valid (1) or invalid (0).
	 *               The size matches leftCorners.size().
	 * @return Vector of corresponding points in the right image. The size matches leftCorners.size().
	 *         Invalid correspondences may have coordinates outside the image bounds.
	 * @note Both input images must be grayscale (CV_8UC1).
	 * @note If GPU is enabled, the GPU version of this method is called automatically.
	 */
	virtual std::vector<cv::Point2f> computeCorrespondences(
			const cv::Mat & leftImage,
			const cv::Mat & rightImage,
			const std::vector<cv::Point2f> & leftCorners,
			std::vector<unsigned char> & status) const;
	
#ifdef HAVE_OPENCV_CUDEV
	/**
	 * @brief Compute stereo correspondences using GPU-accelerated optical flow
	 * 
	 * GPU-accelerated version of computeCorrespondences using CUDA-optimized
	 * sparse pyramidal Lucas-Kanade optical flow. This method is automatically
	 * called when GPU is enabled.
	 * 
	 * @param leftImage Left stereo image on GPU (must be CV_8UC1 grayscale)
	 * @param rightImage Right stereo image on GPU (must be CV_8UC1 grayscale)
	 * @param leftCorners Input vector of feature points in the left image
	 * @param status Output vector indicating which correspondences are valid
	 * @return Vector of corresponding points in the right image
	 * @note Requires RTAB-Map to be built with OpenCV CUDA support (HAVE_OPENCV_CUDAOPTFLOW).
	 * @note The results are filtered based on disparity constraints after GPU computation.
	 */
	virtual std::vector<cv::Point2f> computeCorrespondences(
			const cv::cuda::GpuMat & leftImage,
			const cv::cuda::GpuMat & rightImage,
			const std::vector<cv::Point2f> & leftCorners,
			std::vector<unsigned char> & status) const;
#endif

	/**
	 * @brief Get the convergence threshold (epsilon)
	 * @return Convergence threshold for optical flow iteration termination
	 */
	float epsilon() const {return epsilon_;}
	
	/**
	 * @brief Check if GPU acceleration is enabled
	 * 
	 * Returns whether GPU acceleration is currently enabled for optical flow computation.
	 * GPU acceleration requires OpenCV CUDA support to be compiled in.
	 * 
	 * @return true if GPU is enabled and available, false otherwise
	 */
	virtual bool isGpuEnabled() const;

private:
	/**
	 * @brief Update status vector based on disparity constraints
	 * 
	 * Filters the correspondence results by checking if the computed disparity
	 * (leftCorners[i].x - rightCorners[i].x) falls within the valid range
	 * [minDisparity(), maxDisparity()]. Points outside this range are marked
	 * as invalid in the status vector.
	 * 
	 * @param leftCorners Input feature points in the left image
	 * @param rightCorners Corresponding points in the right image
	 * @param status Status vector to update (1 = valid, 0 = invalid)
	 * @note This method is called automatically after optical flow computation
	 *       to filter results based on disparity constraints.
	 */
	void updateStatus(
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		std::vector<unsigned char> & status) const;

private:
	float epsilon_; ///< Convergence threshold for optical flow (default: from Parameters::defaultStereoEps())
	bool gpu_;      ///< Enable GPU acceleration (default: from Parameters::defaultStereoGpu(), requires OpenCV CUDA)
};

} /* namespace rtabmap */

#endif /* STEREO_H_ */
