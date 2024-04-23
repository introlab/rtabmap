
/**
 * Author: Mathieu Labbe
 * This file is a copy of GPSPose2Factor.h of gtsam examples
 */

/**
 * A simple 2D 'GPS' like factor
 * The factor contains a X-Y position measurement (mx, my) for a Pose, but no rotation information
 * The error vector will be [x-mx, y-my]'
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>


namespace rtabmap {

template<class VALUE>
class XYFactor: public gtsam::NoiseModelFactor1<VALUE> {

private:
  // measurement information
  double mx_, my_;

public:

  /**
   * Constructor
   * @param poseKey    associated pose varible key
   * @param model      noise model for GPS snesor, in X-Y
   * @param m          Point2 measurement
   */
  XYFactor(gtsam::Key poseKey, const gtsam::Point2 m, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor1<VALUE>(model, poseKey), mx_(m.x()), my_(m.y()) {}

  // error function
  // @param p    the pose in Pose2
  // @param H    the optional Jacobian matrix, which use boost optional and has default null pointer
  gtsam::Vector evaluateError(const VALUE& p, boost::optional<gtsam::Matrix&> H = boost::none) const {

    // note that use boost optional like a pointer
    // only calculate jacobian matrix when non-null pointer exists
    if (H) *H = (gtsam::Matrix23() << 1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0).finished();

    // return error vector
    return (gtsam::Vector2() << p.x() - mx_, p.y() - my_).finished();
  }

};

} // namespace gtsamexamples

