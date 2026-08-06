/*
 * planar_constraint_error.h
 *
 * Unary "lock the robot to a horizontal plane" constraint for 2D BA.
 * Mirrors g2o's EdgeSBACamPrior with pinfo(2,2) = 1e9: constrains the
 * BODY-frame z coordinate of the camera vertex to its initial value, so
 * the recovered trajectory stays on the floor. Only the z translation
 * axis carries weight; everything else is free.
 *
 * Snavely camera parameterization is [aa_cw (3), t_cw (3)] (world-to-
 * camera). Given the body-to-camera (= localTransform) rotation R_bc and
 * translation t_bc, the body position in world coordinates is:
 *
 *   body_world = R_cw^T * (-R_bc^T * t_bc - t_cw)
 *
 * Only z is extracted and compared to the initial body z.
 */

#ifndef CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_PLANAR_CONSTRAINT_ERROR_H_
#define CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_PLANAR_CONSTRAINT_ERROR_H_

#include "Eigen/Core"
#include "ceres/autodiff_cost_function.h"
#include "ceres/rotation.h"

namespace ceres {

struct PlanarConstraintError {
  PlanarConstraintError(const Eigen::Matrix3d & R_bc,
                        const Eigen::Vector3d & t_bc,
                        double initial_body_z,
                        double sqrt_information)
      : neg_Rbc_T_tbc_(-R_bc.transpose() * t_bc),
        initial_body_z_(initial_body_z),
        sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T * const camera, T * residuals) const {
    // camera = [aa_cw (3), t_cw (3)] (Snavely / world-to-camera).
    // body_world = R_cw^T * (-R_bc^T * t_bc - t_cw)
    const T diff[3] = { T(neg_Rbc_T_tbc_(0)) - camera[3],
                        T(neg_Rbc_T_tbc_(1)) - camera[4],
                        T(neg_Rbc_T_tbc_(2)) - camera[5] };
    const T inv_aa[3] = { -camera[0], -camera[1], -camera[2] };
    T body_world[3];
    ceres::AngleAxisRotatePoint(inv_aa, diff, body_world);
    residuals[0] = (body_world[2] - T(initial_body_z_)) * T(sqrt_information_);
    return true;
  }

  static ceres::CostFunction * Create(const Eigen::Matrix3d & R_bc,
                                      const Eigen::Vector3d & t_bc,
                                      double initial_body_z,
                                      double sqrt_information) {
    return new ceres::AutoDiffCostFunction<PlanarConstraintError, 1, 6>(
        new PlanarConstraintError(R_bc, t_bc, initial_body_z, sqrt_information));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const Eigen::Vector3d neg_Rbc_T_tbc_;  // = -R_bc^T * t_bc (constant)
  const double initial_body_z_;
  const double sqrt_information_;
};

}  // namespace ceres

#endif  // CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_PLANAR_CONSTRAINT_ERROR_H_
