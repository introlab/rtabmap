/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * Author: Mathieu Labbe
 * This file is a copy of AttitudeFactor.cpp of gtsam library but
 * with gravityError() function overridden to ignore yaw errors.
 * For the noise model, use Sigmas(Vector2(0.1, 10)) (with second sigma high!)
 */

/**
 *  @file   GravityFactor.cpp
 *  @author Frank Dellaert
 *  @brief  Implementation file for Gravity factor
 *  @date   January 28, 2014
 **/

#include "GravityFactor.h"
#include <gtsam/base/numericalDerivative.h>

using namespace std;

namespace rtabmap {

//***************************************************************************
Vector GravityFactor::gravityError(const Rot3& nRb,
    OptionalJacobian<2, 3> H) const {
  auto errorFunction = [this](const Rot3& candidate) -> Vector {
    Vector3 r = candidate.xyz();
    Unit3 nRef = Rot3::RzRyRx(r.x(), r.y(), 0) * Unit3(-bRef_.point3());
#if GTSAM_VERSION_NUMERIC >= 40300
    return nZ_.errorVector(nRef);
#else
    return nZ_.error(nRef);
#endif
  };

  if (H) {
    *H = gtsam::numericalDerivative11<Vector, Rot3>(errorFunction, nRb);
  }
  return errorFunction(nRb);
}

//***************************************************************************
void Rot3GravityFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << "Rot3GravityFactor on " << keyFormatter(this->key()) << "\n";
  nZ_.print("  measured direction in nav frame: ");
  bRef_.print("  reference direction in body frame: ");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool Rot3GravityFactor::equals(const NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol) && this->nZ_.equals(e->nZ_, tol)
      && this->bRef_.equals(e->bRef_, tol);
}

//***************************************************************************
void Pose3GravityFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << "Pose3GravityFactor on " << keyFormatter(this->key()) << "\n";
  nZ_.print("  measured direction in nav frame: ");
  bRef_.print("  reference direction in body frame: ");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool Pose3GravityFactor::equals(const NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol) && this->nZ_.equals(e->nZ_, tol)
      && this->bRef_.equals(e->bRef_, tol);
}

//***************************************************************************

}/// namespace gtsam
