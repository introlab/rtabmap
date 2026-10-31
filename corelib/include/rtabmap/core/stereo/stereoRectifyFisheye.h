/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

This code is the same has cv::stereoRectify() but accepting fisheye distortion model:
All cvUndistortPoints() have been replaced by cv::fisheye::undistortPoints()
See https://github.com/opencv/opencv/blob/master/modules/calib3d/src/calibration.cpp

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

#ifndef CORELIB_SRC_OPENCV_STEREORECTIFYFISHEYE_H_
#define CORELIB_SRC_OPENCV_STEREORECTIFYFISHEYE_H_

#include <rtabmap/core/rtabmap_core_export.h>
#include <opencv2/core/core.hpp>

// stereoRectifyFisheye() relies on the OpenCV C API, which was removed in
// OpenCV 5 (use cv::fisheye::stereoRectify() instead). It also requires the
// fisheye module, added in OpenCV 2.4.10.
#if (CV_MAJOR_VERSION > 2 && CV_MAJOR_VERSION < 5) || \
    (CV_MAJOR_VERSION == 2 && (CV_MINOR_VERSION > 4 || (CV_MINOR_VERSION == 4 && CV_SUBMINOR_VERSION >= 10)))
#define RTABMAP_HAS_STEREO_RECTIFY_FISHEYE 1
#else
#define RTABMAP_HAS_STEREO_RECTIFY_FISHEYE 0
#endif

#if RTABMAP_HAS_STEREO_RECTIFY_FISHEYE

namespace rtabmap
{

/**
 * Same as cv::stereoRectify() but for the fisheye distortion model
 * (cvUndistortPoints() replaced by cv::fisheye::undistortPoints()).
 */
void RTABMAP_CORE_EXPORT stereoRectifyFisheye(
		cv::InputArray cameraMatrix1, cv::InputArray distCoeffs1,
		cv::InputArray cameraMatrix2, cv::InputArray distCoeffs2,
		cv::Size imageSize, cv::InputArray R, cv::InputArray T,
		cv::OutputArray R1, cv::OutputArray R2,
		cv::OutputArray P1, cv::OutputArray P2,
		cv::OutputArray Q, int flags,
		double alpha, cv::Size newImageSize);

}

#endif // RTABMAP_HAS_STEREO_RECTIFY_FISHEYE

#endif /* CORELIB_SRC_OPENCV_STEREORECTIFYFISHEYE_H_ */
