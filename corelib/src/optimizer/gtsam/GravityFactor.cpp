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
 * with attitudeError() function overridden to ignore yaw errors.
 * For the noise model, use Sigmas(Vector2(0.1, 10)) (with second sigma high!)
 */

/**
 *  @file   GravityFactor.cpp
 *  @author Frank Dellaert
 *  @brief  Implementation file for Attitude factor
 *  @date   January 28, 2014
 **/

#include "GravityFactor.h"

using namespace std;

namespace rtabmap {

//***************************************************************************
Vector GravityFactor::attitudeError(const Rot3& nRb,
    OptionalJacobian<2, 3> H) const {
  if (H) {
    Matrix23 D_nRef_R;
    Matrix22 D_e_nRef;
    Vector3 r = nRb.xyz();
    Unit3 nRef = Rot3::RzRyRx(r.x(), r.y(), 0).rotate(bRef_, D_nRef_R);
    Vector e = nZ_.error(nRef, D_e_nRef);
    (*H) = D_e_nRef * D_nRef_R;
    //printf("ref=%f %f %f grav=%f %f %f e= %f %f H=%f %f %f, %f %f %f\n",
    //		nRef.point3().x(), nRef.point3().y(), nRef.point3().z(), nZ_.point3().x(), nZ_.point3().y(), nZ_.point3().z(), e(0), e(1),
	//		(*H)(0,0), (*H)(0,1), (*H)(0,2), (*H)(1,0), (*H)(1,1), (*H)(1,2));
    return e;
  } else {
    Vector3 r = nRb.xyz();
    Unit3 nRef = Rot3::RzRyRx(r.x(), r.y(), 0) * bRef_;
    Vector e = nZ_.error(nRef);
    //printf("ref=%f %f %f grav=%f %f %f e= %f %f\n", nRef.point3().x(), nRef.point3().y(), nRef.point3().z(), nZ_.point3().x(), nZ_.point3().y(), nZ_.point3().z(), e(0), e(1));
    return e;
  }
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
