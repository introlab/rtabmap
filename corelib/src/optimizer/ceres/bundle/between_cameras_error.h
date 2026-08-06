/*
 * between_cameras_error.h
 *
 * Pose-graph constraint between two BA camera variables. Used to fold
 * relative-pose link measurements (kNeighbor edges) into the BA cost so
 * Ceres BA can use the same pose-graph chain that g2o (EdgeSBACam) and
 * GTSAM (BetweenFactor<Pose3>) include in their BA paths.
 *
 * Camera parameterization matches SnavelyReprojectionError: 6 doubles
 * per camera [angle_axis (3), translation (3)] in world-to-camera
 * convention (a point X_world is mapped to X_cam = R*X_world + t).
 */

#ifndef CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_BETWEEN_CAMERAS_ERROR_H_
#define CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_BETWEEN_CAMERAS_ERROR_H_

#include "Eigen/Core"
#include "ceres/autodiff_cost_function.h"
#include "ceres/rotation.h"

namespace ceres {

// Relative-pose constraint between two cameras, expressed in the SAME frame
// convention as g2o's EdgeSBACam: the measurement is the pose of camera B as
// seen from camera A's frame (i.e., the relative transform from cam_a to
// cam_b). Residual is 6-D [Δt; Δrot] in se(3) — Δt = predicted_t - measured_t
// and Δrot = log(R_meas^T * R_predicted) — then whitened by the square root
// of the information matrix.
struct BetweenCamerasError {
  BetweenCamerasError(const Eigen::Vector3d & translation_measured,
                      const Eigen::Vector3d & angle_axis_measured,
                      const Eigen::Matrix<double, 6, 6> & sqrt_information)
      : t_meas_(translation_measured),
        aa_meas_(angle_axis_measured),
        sqrt_info_(sqrt_information) {}

  template <typename T>
  bool operator()(const T * const cam_a,
                  const T * const cam_b,
                  T * residuals_ptr) const {
    // Camera layout: [aa(3), t(3)] -- world-to-camera.
    const T * aa_a = cam_a;
    const T * t_a  = cam_a + 3;
    const T * aa_b = cam_b;
    const T * t_b  = cam_b + 3;

    // Predicted relative transform "cam_b in cam_a's frame":
    //   R_ab = R_a * R_b^T
    //   t_ab = t_a - R_ab * t_b
    T q_a[4];
    T q_b[4];
    ceres::AngleAxisToQuaternion(aa_a, q_a);
    ceres::AngleAxisToQuaternion(aa_b, q_b);

    // q_b_inv = conjugate (unit quat assumed).
    T q_b_inv[4] = { q_b[0], -q_b[1], -q_b[2], -q_b[3] };
    T q_ab[4];
    ceres::QuaternionProduct(q_a, q_b_inv, q_ab);

    T aa_ab[3];
    ceres::QuaternionToAngleAxis(q_ab, aa_ab);

    T R_ab_tb[3];
    ceres::AngleAxisRotatePoint(aa_ab, t_b, R_ab_tb);

    T t_ab[3] = { t_a[0] - R_ab_tb[0],
                  t_a[1] - R_ab_tb[1],
                  t_a[2] - R_ab_tb[2] };

    // Translation residual: predicted - measured.
    T t_residual[3] = { t_ab[0] - T(t_meas_(0)),
                        t_ab[1] - T(t_meas_(1)),
                        t_ab[2] - T(t_meas_(2)) };

    // Rotation residual: angle-axis of (R_meas^T * R_predicted).
    T q_meas[4];
    const T aa_meas_T[3] = { T(aa_meas_(0)), T(aa_meas_(1)), T(aa_meas_(2)) };
    ceres::AngleAxisToQuaternion(aa_meas_T, q_meas);
    T q_meas_inv[4] = { q_meas[0], -q_meas[1], -q_meas[2], -q_meas[3] };
    T q_err[4];
    ceres::QuaternionProduct(q_meas_inv, q_ab, q_err);
    T aa_err[3];
    ceres::QuaternionToAngleAxis(q_err, aa_err);

    Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(residuals_ptr);
    residuals(0) = t_residual[0];
    residuals(1) = t_residual[1];
    residuals(2) = t_residual[2];
    residuals(3) = aa_err[0];
    residuals(4) = aa_err[1];
    residuals(5) = aa_err[2];
    residuals.applyOnTheLeft(sqrt_info_.template cast<T>());

    return true;
  }

  static ceres::CostFunction * Create(
      const Eigen::Vector3d & translation_measured,
      const Eigen::Vector3d & angle_axis_measured,
      const Eigen::Matrix<double, 6, 6> & sqrt_information) {
    return new ceres::AutoDiffCostFunction<BetweenCamerasError, 6, 6, 6>(
        new BetweenCamerasError(translation_measured, angle_axis_measured, sqrt_information));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const Eigen::Vector3d t_meas_;
  const Eigen::Vector3d aa_meas_;
  const Eigen::Matrix<double, 6, 6> sqrt_info_;
};

}  // namespace ceres

#endif  // CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_BETWEEN_CAMERAS_ERROR_H_
