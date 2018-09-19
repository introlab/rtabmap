/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef RTABMAP_CORELIB_SRC_OPENCV_SOLVEPNP_H_
#define RTABMAP_CORELIB_SRC_OPENCV_SOLVEPNP_H_

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#if CV_MAJOR_VERSION >= 3
#include <opencv2/calib3d/calib3d_c.h>
#endif

namespace cv3 {




/** @brief Finds an object pose from 3D-2D point correspondences using the RANSAC scheme.

@param objectPoints Array of object points in the object coordinate space, 3xN/Nx3 1-channel or
1xN/Nx1 3-channel, where N is the number of points. vector\<Point3f\> can be also passed here.
@param imagePoints Array of corresponding image points, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel,
where N is the number of points. vector\<Point2f\> can be also passed here.
@param cameraMatrix Input camera matrix \f$A = \vecthreethree{fx}{0}{cx}{0}{fy}{cy}{0}{0}{1}\f$ .
@param distCoeffs Input vector of distortion coefficients
\f$(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\f$ of
4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are
assumed.
@param rvec Output rotation vector (see Rodrigues ) that, together with tvec , brings points from
the model coordinate system to the camera coordinate system.
@param tvec Output translation vector.
@param useExtrinsicGuess Parameter used for SOLVEPNP_ITERATIVE. If true (1), the function uses
the provided rvec and tvec values as initial approximations of the rotation and translation
vectors, respectively, and further optimizes them.
@param iterationsCount Number of iterations.
@param reprojectionError Inlier threshold value used by the RANSAC procedure. The parameter value
is the maximum allowed distance between the observed and computed point projections to consider it
an inlier.
@param confidence The probability that the algorithm produces a useful result.
@param inliers Output vector that contains indices of inliers in objectPoints and imagePoints .
@param flags Method for solving a PnP problem (see solvePnP ).

The function estimates an object pose given a set of object points, their corresponding image
projections, as well as the camera matrix and the distortion coefficients. This function finds such
a pose that minimizes reprojection error, that is, the sum of squared distances between the observed
projections imagePoints and the projected (using projectPoints ) objectPoints. The use of RANSAC
makes the function resistant to outliers.

@note
   -   An example of how to use solvePNPRansac for object detection can be found at
        opencv_source_code/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/
 */
bool solvePnPRansac( cv::InputArray objectPoints, cv::InputArray imagePoints,
					cv::InputArray cameraMatrix, cv::InputArray distCoeffs,
					cv::OutputArray rvec, cv::OutputArray tvec,
					bool useExtrinsicGuess = false, int iterationsCount = 100,
					float reprojectionError = 8.0, double confidence = 0.99,
					cv::OutputArray inliers = cv::noArray(), int flags = CV_ITERATIVE );

int RANSACUpdateNumIters( double p, double ep, int modelPoints, int maxIters );

class LMSolver : public cv::Algorithm
{
public:
    class Callback
    {
    public:
        virtual ~Callback() {}
        virtual bool compute(cv::InputArray param, cv::OutputArray err, cv::OutputArray J) const = 0;
    };

    virtual void setCallback(const cv::Ptr<LMSolver::Callback>& cb) = 0;
    virtual int run(cv::InputOutputArray _param0) const = 0;
};

cv::Ptr<LMSolver> createLMSolver(const cv::Ptr<LMSolver::Callback>& cb, int maxIters);

class PointSetRegistrator : public cv::Algorithm
{
public:
    class Callback
    {
    public:
        virtual ~Callback() {}
        virtual int runKernel(cv::InputArray m1, cv::InputArray m2, cv::OutputArray model) const = 0;
        virtual void computeError(cv::InputArray m1, cv::InputArray m2, cv::InputArray model, cv::OutputArray err) const = 0;
        virtual bool checkSubset(cv::InputArray, cv::InputArray, int) const { return true; }
    };

    virtual void setCallback(const cv::Ptr<PointSetRegistrator::Callback>& cb) = 0;
    virtual bool run(cv::InputArray m1, cv::InputArray m2, cv::OutputArray model, cv::OutputArray mask) const = 0;
};

cv::Ptr<PointSetRegistrator> createRANSACPointSetRegistrator(const cv::Ptr<PointSetRegistrator::Callback>& cb,
                                                                    int modelPoints, double threshold,
                                                                    double confidence=0.99, int maxIters=1000 );

cv::Ptr<PointSetRegistrator> createLMeDSPointSetRegistrator(const cv::Ptr<PointSetRegistrator::Callback>& cb,
                                                                   int modelPoints, double confidence=0.99, int maxIters=1000 );

template<typename T> inline int compressElems( T* ptr, const uchar* mask, int mstep, int count )
{
    int i, j;
    for( i = j = 0; i < count; i++ )
        if( mask[i*mstep] )
        {
            if( i > j )
                ptr[j] = ptr[i];
            j++;
        }
    return j;
}

}

#endif /* RTABMAP_CORELIB_SRC_OPENCV_SOLVEPNP_H_ */
