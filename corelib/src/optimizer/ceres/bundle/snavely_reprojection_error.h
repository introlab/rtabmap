/*
 * snavely_reprojection_error.h
 *
 *  Created on: Aug 16, 2019
 *      Author: mathieu
 */

#ifndef CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_SNAVELY_REPROJECTION_ERROR_H_
#define CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_SNAVELY_REPROJECTION_ERROR_H_

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <rtabmap/utilite/ULogger.h>

namespace ceres {

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 6 parameters: 3 for rotation, 3 for translation. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y, double fx, double fy)
      : observed_x(observed_x), observed_y(observed_y), fx(fx), fy(fy) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {

    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);
    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Compute final projected point position.
    T predicted_x = fx * xp;
    T predicted_y = fy * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;

    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
									 const double fx,
									 const double fy) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3>(
                new SnavelyReprojectionError(observed_x, observed_y, fx, fy)));
  }
  double observed_x;
  double observed_y;
  double fx;
  double fy;
};

}

#endif /* CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_SNAVELY_REPROJECTION_ERROR_H_ */
