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

#ifndef STEREOBM_H_
#define STEREOBM_H_

#include <rtabmap/core/StereoDense.h>
#include <rtabmap/core/Parameters.h>
#include <opencv2/core/core.hpp>

namespace rtabmap {

/**
 * @class StereoBM
 * @brief Block Matching algorithm for dense stereo matching
 * 
 * This class implements the Block Matching (BM) algorithm for computing disparity
 * maps from stereo image pairs.
 * 
 * @note This class wraps OpenCV's cv::StereoBM implementation.
 * @see StereoSGBM for a more accurate but slower alternative
 */
class RTABMAP_CORE_EXPORT StereoBM : public StereoDense {
public:
	/**
	 * @brief Constructor with explicit block size and number of disparities
	 * 
	 * @param blockSize Size of the block window for matching (must be odd, typically 5-21)
	 * @param numDisparities Number of disparity levels to search (must be divisible by 16)
	 */
	StereoBM(int blockSize, int numDisparities);
	
	/**
	 * @brief Constructor with parameters map
	 * 
	 * Creates a StereoBM instance and initializes it from the provided parameters map.
	 * 
	 * @param parameters Parameters map containing configuration values
	 */
	StereoBM(const ParametersMap & parameters = ParametersMap());
	
	/**
	 * @brief Virtual destructor
	 */
	virtual ~StereoBM() {}

	/**
	 * @brief Parse parameters from a parameters map
	 * 
	 * Updates the algorithm's configuration based on the provided parameters map.
	 * Supported parameters:
	 * - Parameters::kStereoBMBlockSize()
	 * - Parameters::kStereoBMMinDisparity()
	 * - Parameters::kStereoBMNumDisparities()
	 * - Parameters::kStereoBMPreFilterSize()
	 * - Parameters::kStereoBMPreFilterCap()
	 * - Parameters::kStereoBMUniquenessRatio()
	 * - Parameters::kStereoBMTextureThreshold()
	 * - Parameters::kStereoBMSpeckleWindowSize()
	 * - Parameters::kStereoBMSpeckleRange()
	 * - Parameters::kStereoBMDisp12MaxDiff()
	 * 
	 * @param parameters Parameters map containing configuration values
	 */
	virtual void parseParameters(const ParametersMap & parameters);
	
	/**
	 * @brief Compute disparity map from stereo image pair
	 * 
	 * Computes a disparity map using the Block Matching algorithm. The images
	 * must have the same size and be either grayscale (CV_8UC1) or color (CV_8UC3).
	 * Color images are automatically converted to grayscale.
	 * 
	 * @param leftImage Left stereo image (CV_8UC1 or CV_8UC3)
	 * @param rightImage Right stereo image (CV_8UC1 or CV_8UC3), must have same size as leftImage
	 * @return Disparity map as a 16-bit signed integer image (CV_16SC1)
	 * @note The disparity values are stored as fixed-point numbers with 4 fractional bits.
	 *       To get the actual disparity, divide by 16.
	 */
	virtual cv::Mat computeDisparity(
			const cv::Mat & leftImage,
			const cv::Mat & rightImage) const;

private:
	int blockSize_;         ///< Size of the block window for matching (default: 15, must be odd)
	int minDisparity_;      ///< Minimum disparity value to search (default: 0)
	int numDisparities_;    ///< Number of disparity levels to search (default: 64, must be divisible by 16)
	int preFilterSize_;     ///< Size of the prefiltering window (default: 9, must be odd)
	int preFilterCap_;      ///< Prefiltering cap value (default: 31)
	int uniquenessRatio_;   ///< Uniqueness ratio for matching (default: 15)
	int textureThreshold_;  ///< Texture threshold for filtering (default: 10)
	int speckleWindowSize_; ///< Maximum size of smooth disparity regions to consider as speckles (default: 100)
	int speckleRange_;      ///< Maximum disparity variation within each connected component (default: 4)
	int disp12MaxDiff_;     ///< Maximum allowed difference in left-right disparity check (default: -1, disabled)
};

} /* namespace rtabmap */

#endif /* STEREOBM_H_ */
