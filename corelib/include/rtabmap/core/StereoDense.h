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

#ifndef STEREODENSE_H_
#define STEREODENSE_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <opencv2/core/core.hpp>

namespace rtabmap {

/**
 * @class StereoDense
 * @brief Abstract base class for dense stereo matching algorithms
 * 
 * This class provides the interface for dense stereo matching algorithms that
 * compute disparity maps from stereo image pairs. It uses a factory pattern
 * to create specific implementations (Block Matching or Semi-Global Block Matching).
 * 
 * The disparity map is a 16-bit signed integer image (CV_16SC1) where each pixel
 * value represents the horizontal displacement between corresponding pixels in
 * the left and right stereo images. Higher disparity values indicate closer objects.
 * 
 * @note Both input images must have the same size and be either grayscale (CV_8UC1)
 *       or color (CV_8UC3). Color images are automatically converted to grayscale.
 */
class RTABMAP_CORE_EXPORT StereoDense {
public:
	/**
	 * @enum Type
	 * @brief Enumeration of available stereo matching algorithm types
	 */
	enum Type {
		kTypeBM = 0,    ///< Block Matching algorithm (faster, less accurate)
		kTypeSGBM = 1  ///< Semi-Global Block Matching algorithm (slower, more accurate)
	};
	
	/**
	 * @brief Factory method to create a StereoDense instance from parameters
	 * 
	 * Creates a StereoDense instance based on the stereo dense strategy parameter
	 * in the provided parameters map. The strategy is determined by the
	 * Parameters::kStereoDenseStrategy() parameter.
	 * 
	 * @param parameters Parameters map containing configuration values
	 * @return Pointer to the created StereoDense instance (caller owns the memory)
	 * @see create(Type, const ParametersMap&)
	 */
	static StereoDense * create(const ParametersMap & parameters);
	
	/**
	 * @brief Factory method to create a StereoDense instance of a specific type
	 * 
	 * Creates a StereoDense instance of the specified type with the given parameters.
	 * 
	 * @param type The type of stereo matching algorithm to create
	 * @param parameters Optional parameters map containing configuration values
	 * @return Pointer to the created StereoDense instance (caller owns the memory)
	 * @see create(const ParametersMap&)
	 */
	static StereoDense * create(StereoDense::Type type, const ParametersMap & parameters = ParametersMap());

public:
	/**
	 * @brief Virtual destructor
	 */
	virtual ~StereoDense() {}

	/**
	 * @brief Parse parameters from a parameters map
	 * 
	 * Updates the algorithm's configuration based on the provided parameters map.
	 * Derived classes should override this method to parse their specific parameters.
	 * 
	 * @param parameters Parameters map containing configuration values
	 */
	virtual void parseParameters(const ParametersMap & parameters) {}
	
	/**
	 * @brief Compute disparity map from stereo image pair
	 * 
	 * Computes a disparity map from the given left and right stereo images.
	 * The images must have the same size and be either grayscale (CV_8UC1) or
	 * color (CV_8UC3). Color images are automatically converted to grayscale.
	 * 
	 * @param leftImage Left stereo image (CV_8UC1 or CV_8UC3)
	 * @param rightImage Right stereo image (CV_8UC1 or CV_8UC3), must have same size as leftImage
	 * @return Disparity map as a 16-bit signed integer image (CV_16SC1)
	 * @note The disparity values are stored as fixed-point numbers with 4 fractional bits.
	 *       To get the actual disparity, divide by 16.
	 * @note Invalid disparities are typically represented by negative values or zero.
	 */
	virtual cv::Mat computeDisparity(
			const cv::Mat & leftImage,
			const cv::Mat & rightImage) const = 0;

protected:
	/**
	 * @brief Protected constructor
	 * 
	 * @param parameters Optional parameters map for initialization
	 */
	StereoDense(const ParametersMap & parameters = ParametersMap()) {}
};

} /* namespace rtabmap */

#endif /* STEREODENSE_H_ */
