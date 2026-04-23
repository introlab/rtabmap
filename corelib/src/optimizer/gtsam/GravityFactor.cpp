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

using namespace std;

namespace rtabmap {

//***************************************************************************
Vector GravityFactor::gravityError(const Rot3& nRb,
    OptionalJacobian<2, 3> H) const {
  if (H) {
    Matrix33 H_rpy_R;
    Vector3 rpy = nRb.xyz(H_rpy_R);
    Matrix23 D_rp_R = H_rpy_R.block<2, 3>(0, 0);

    Matrix23 D_nRef_R;
    Unit3 nRef = Rot3::RzRyRx(rpy.x(), rpy.y(), 0.0).rotate(Unit3(-bRef_.point3()), D_nRef_R);

    Matrix22 D_e_nRef;
    Vector e = nZ_.error(nRef, D_e_nRef);

    Matrix32 D_R_rp;
    D_R_rp << 1.0, 0.0, 0.0, cos(rpy.x()), 0.0, -sin(rpy.x());

    (*H) = D_e_nRef * D_nRef_R * D_R_rp * D_rp_R;
    return e;
  } else {
    Vector3 rpy = nRb.xyz();
    Unit3 nRef = Rot3::RzRyRx(rpy.x(), rpy.y(), 0.0) * Unit3(-bRef_.point3());
    return nZ_.error(nRef);
  }
}

//***************************************************************************
void Rot3GravityFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? "" : s + " ") << "Rot3GravityFactor on "
       << keyFormatter(this->key()) << "\n";
  nZ_.print("  measured direction in nav frame: ");
  bRef_.print("  reference direction in body frame: ");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool Rot3GravityFactor::equals(const NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && this->nZ_.equals(e->nZ_, tol)
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
  return e != nullptr && Base::equals(*e, tol) && this->nZ_.equals(e->nZ_, tol)
      && this->bRef_.equals(e->bRef_, tol);
}

//***************************************************************************

}/// namespace gtsam
