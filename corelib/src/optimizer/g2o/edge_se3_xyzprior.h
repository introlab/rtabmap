// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_EDGE_SE3_PRIOR_XYZ_H
#define G2O_EDGE_SE3_PRIOR_XYZ_H

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"

namespace g2o {

using namespace Eigen;

/**
* \brief Prior for a 3D pose with constraints only in xyz direction
*/
class EdgeSE3XYZPrior : public BaseUnaryEdge<3, Vector3d, VertexSE3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3XYZPrior();

  virtual void setMeasurement(const Vector3d& m) {
    _measurement = m;
  }

  virtual bool setMeasurementData(const double * d) {
    Map<const Vector3d> v(d);
    _measurement = v;
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    Map<Vector3d> v(d);
    v = _measurement;
    return true;
  }

  virtual int measurementDimension() const {return 3;}

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
  virtual void computeError();
  virtual bool setMeasurementFromState();

  virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/, OptimizableGraph::Vertex* /*to*/) {return 1.;}
  virtual void initialEstimate(const OptimizableGraph::VertexSet& /*from_*/, OptimizableGraph::Vertex* /*to_*/);

  const ParameterSE3Offset* offsetParameter() { return _offsetParam; }

protected:
  virtual bool resolveCaches();
  ParameterSE3Offset* _offsetParam;
  CacheSE3Offset* _cache;
};

EdgeSE3XYZPrior::EdgeSE3XYZPrior() : BaseUnaryEdge<3, Vector3d, VertexSE3>()
{
  information().setIdentity();
  setMeasurement(Vector3d::Zero());
  _cache = 0;
  _offsetParam = 0;
  resizeParameters(1);
  installParameter(_offsetParam, 0);
}

bool EdgeSE3XYZPrior::resolveCaches(){
  assert(_offsetParam);
  ParameterVector pv(1);
  pv[0] = _offsetParam;
  resolveCache(_cache, (OptimizableGraph::Vertex*)_vertices[0], "CACHE_SE3_OFFSET", pv);
  return _cache != 0;
}

bool EdgeSE3XYZPrior::read(std::istream& is)
{
  int pid;
  is >> pid;
  if (!setParameterId(0, pid))
  return false;

  // measured keypoint
  Vector3d meas;
  for (int i = 0; i < 3; i++) is >> meas[i];
  setMeasurement(meas);

  // read covariance matrix (upper triangle)
  if (is.good()) {
    for (int i = 0; i < 3; i++) {
      for (int j = i; j < 3; j++) {
        is >> information()(i,j);
        if (i != j)
        information()(j,i) = information()(i,j);
      }
    }
  }
  return !is.fail();
}

bool EdgeSE3XYZPrior::write(std::ostream& os) const {
  os << _offsetParam->id() <<  " ";
  for (int i = 0; i < 3; i++) os << measurement()[i] << " ";
  for (int i = 0; i < 3; i++) {
    for (int j = i; j < 3; j++) {
      os << information()(i,j) << " ";
    }
  }
  return os.good();
}

void EdgeSE3XYZPrior::computeError() {
  const VertexSE3* v = static_cast<const VertexSE3*>(_vertices[0]);
  _error = v->estimate().translation() - _measurement;
}

bool EdgeSE3XYZPrior::setMeasurementFromState() {
  const VertexSE3* v = static_cast<const VertexSE3*>(_vertices[0]);
  _measurement = v->estimate().translation();
  return true;
}

void EdgeSE3XYZPrior::initialEstimate(const OptimizableGraph::VertexSet& /*from_*/, OptimizableGraph::Vertex* /*to_*/) {
  VertexSE3 *v = static_cast<VertexSE3*>(_vertices[0]);
  assert(v && "Vertex for the Prior edge is not set");

  Isometry3d newEstimate = _offsetParam->offset().inverse() * Translation3d(measurement());
  if (_information.block<3,3>(0,0).array().abs().sum() == 0){ // do not set translation, as that part of the information is all zero
    newEstimate.translation() = v->estimate().translation();
  }
  v->setEstimate(newEstimate);
}

}

#endif
