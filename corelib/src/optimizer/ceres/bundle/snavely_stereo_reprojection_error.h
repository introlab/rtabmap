/*
 * snavely_stereo_reprojection_error.h
 *
 * Stereo analogue of SnavelyReprojectionError: outputs 3 residuals
 * (u, v, u-disparity) so the disparity (depth) measurement pins the
 * z component of each landmark relative to its observing camera. With
 * at least one stereo observation per point the 7-DOF mono BA gauge
 * collapses to 6 -- absolute scale becomes observable.
 */

#ifndef CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_SNAVELY_STEREO_REPROJECTION_ERROR_H_
#define CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_SNAVELY_STEREO_REPROJECTION_ERROR_H_

#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace ceres {

// Templated pinhole stereo camera model. Camera parameterization is
// the same as SnavelyReprojectionError (3 angle-axis + 3 translation).
// The stereo channel is encoded via the right-image x coordinate, i.e.
// u_right = u_left - disparity, where disparity = baseline * fx / Z.
// observed_disparity is the measured disparity (precomputed from the
// observation's depth: disparity = baseline * fx / depth).
//
// inv_sigma_uv and inv_sigma_d are 1/sigma_pixel and 1/sigma_disparity --
// they let the cost function reproduce g2o's per-axis information matrix
// (1/pixelVariance on u/v, 1/disparityVariance on disparity). Pass
// inv_sigma_uv = inv_sigma_d = 1 for isotropic behavior.
struct SnavelyStereoReprojectionError {
  SnavelyStereoReprojectionError(
      double observed_x,
      double observed_y,
      double observed_disparity,
      double fx,
      double fy,
      double baseline_fx,
      double inv_sigma_uv,
      double inv_sigma_d)
      : observed_x(observed_x),
        observed_y(observed_y),
        observed_disparity(observed_disparity),
        fx(fx),
        fy(fy),
        baseline_fx(baseline_fx),
        inv_sigma_uv(inv_sigma_uv),
        inv_sigma_d(inv_sigma_d) {}

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

    // Pinhole projection.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    T predicted_x = fx * xp;
    T predicted_y = fy * yp;
    // disparity = baseline * fx / Z
    T predicted_disparity = T(baseline_fx) / p[2];

    residuals[0] = (predicted_x - observed_x) * T(inv_sigma_uv);
    residuals[1] = (predicted_y - observed_y) * T(inv_sigma_uv);
    residuals[2] = (predicted_disparity - observed_disparity) * T(inv_sigma_d);

    return true;
  }

  static ceres::CostFunction* Create(double observed_x,
                                     double observed_y,
                                     double observed_disparity,
                                     double fx,
                                     double fy,
                                     double baseline_fx,
                                     double inv_sigma_uv,
                                     double inv_sigma_d) {
    return (new ceres::AutoDiffCostFunction<SnavelyStereoReprojectionError, 3, 6, 3>(
        new SnavelyStereoReprojectionError(
            observed_x, observed_y, observed_disparity, fx, fy, baseline_fx,
            inv_sigma_uv, inv_sigma_d)));
  }

  double observed_x;
  double observed_y;
  double observed_disparity;
  double fx;
  double fy;
  double baseline_fx;   // baseline * fx (cached)
  double inv_sigma_uv;  // 1 / sqrt(pixelVariance)
  double inv_sigma_d;   // 1 / sqrt(disparityVariance)
};

}  // namespace ceres

#endif  // CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_SNAVELY_STEREO_REPROJECTION_ERROR_H_
