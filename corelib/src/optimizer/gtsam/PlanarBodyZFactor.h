
/**
 * Author: Mathieu Labbe
 */

/**
 * Unary planar constraint mirroring g2o's EdgeSBACamPrior (pinfo(2,2) = 1e9):
 * locks the BODY-frame z of the camera vertex to its initial value, leaving
 * lateral motion + yaw free. Used when isSlam2d() is true in BA.
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>


namespace rtabmap {

class PlanarBodyZFactor: public gtsam::NoiseModelFactor1<gtsam::Pose3> {

  gtsam::Pose3 camera_to_body_;
  double initial_body_z_;

public:

  PlanarBodyZFactor(gtsam::Key poseKey,
                    const gtsam::Pose3 & camera_to_body,
                    double initial_body_z,
                    const gtsam::SharedNoiseModel & model):
    gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey),
    camera_to_body_(camera_to_body),
    initial_body_z_(initial_body_z) {}

  gtsam::Vector evaluateError(const gtsam::Pose3 & camPose,
#if GTSAM_VERSION_NUMERIC >= 40300
                              gtsam::OptionalMatrixType H = OptionalNone) const override
#else
                              boost::optional<gtsam::Matrix &> H = boost::none) const override
#endif
  {
    const auto error_fn = [this](const gtsam::Pose3 & p) {
      gtsam::Vector1 e;
      e(0) = p.compose(camera_to_body_).translation().z() - initial_body_z_;
      return e;
    };
    if(H)
    {
      *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(error_fn, camPose);
    }
    return error_fn(camPose);
  }

}; // PlanarBodyZFactor

} /// namespace rtabmap
