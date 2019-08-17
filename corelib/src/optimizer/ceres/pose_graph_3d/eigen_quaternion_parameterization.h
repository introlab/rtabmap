// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2016 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)

#ifndef CERES_EXAMPLES_POSE_GRAPH_3D_EIGEN_QUATERNION_PARAMETERIZATION_H_
#define CERES_EXAMPLES_POSE_GRAPH_3D_EIGEN_QUATERNION_PARAMETERIZATION_H_

#include "ceres/local_parameterization.h"

namespace ceres {

// Implements the quaternion local parameterization for Eigen's representation
// of the quaternion. Eigen uses a different internal memory layout for the
// elements of the quaternion than what is commonly used. Specifically, Eigen
// stores the elements in memory as [x, y, z, w] where the real part is last
// whereas it is typically stored first. Note, when creating an Eigen quaternion
// through the constructor the elements are accepted in w, x, y, z order. Since
// Ceres operates on parameter blocks which are raw double pointers this
// difference is important and requires a different parameterization.
//
// Plus(x, delta) = [sin(|delta|) delta / |delta|, cos(|delta|)] * x
// with * being the quaternion multiplication operator.
class EigenQuaternionParameterization : public ceres::LocalParameterization {
 public:
  virtual ~EigenQuaternionParameterization() {}
  virtual bool Plus(const double* x_ptr,
                    const double* delta,
                    double* x_plus_delta_ptr) const
  {
	  Eigen::Map<Eigen::Quaterniond> x_plus_delta(x_plus_delta_ptr);
	  Eigen::Map<const Eigen::Quaterniond> x(x_ptr);

	  const double norm_delta =
	      sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
	  if (norm_delta > 0.0) {
	    const double sin_delta_by_delta = sin(norm_delta) / norm_delta;

	    // Note, in the constructor w is first.
	    Eigen::Quaterniond delta_q(cos(norm_delta),
	                               sin_delta_by_delta * delta[0],
	                               sin_delta_by_delta * delta[1],
	                               sin_delta_by_delta * delta[2]);
	    x_plus_delta = delta_q * x;
	  } else {
	    x_plus_delta = x;
	  }

	  return true;
  }
  virtual bool ComputeJacobian(const double* x,
                               double* jacobian) const{
	jacobian[0] =  x[3]; jacobian[1]  =  x[2]; jacobian[2]  = -x[1];  // NOLINT
	jacobian[3] = -x[2]; jacobian[4]  =  x[3]; jacobian[5]  =  x[0];  // NOLINT
	jacobian[6] =  x[1]; jacobian[7]  = -x[0]; jacobian[8]  =  x[3];  // NOLINT
	jacobian[9] = -x[0]; jacobian[10] = -x[1]; jacobian[11] = -x[2];  // NOLINT
	return true;
  }
  virtual int GlobalSize() const { return 4; }
  virtual int LocalSize() const { return 3; }
};

}  // namespace ceres

#endif  // CERES_EXAMPLES_POSE_GRAPH_3D_EIGEN_QUATERNION_PARAMETERIZATION_H_
