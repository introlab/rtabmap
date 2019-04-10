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

#ifndef RTAB_G2O_EDGE_SE3_PRIOR_XYZ_H
#define RTAB_G2O_EDGE_SE3_PRIOR_XYZ_H

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"

namespace rtabmap {

/**
* \brief Prior for a 3D pose with constraints only in xyz direction
*/
class EdgeSE3XYZPrior : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3XYZPrior();

  virtual void setMeasurement(const Eigen::Vector3d& m) {
    _measurement = m;
  }

  virtual bool setMeasurementData(const double * d) {
    Eigen::Map<const Eigen::Vector3d> v(d);
    _measurement = v;
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    Eigen::Map<Eigen::Vector3d> v(d);
    v = _measurement;
    return true;
  }

  virtual int measurementDimension() const {return 3;}

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
  virtual void computeError();
  virtual bool setMeasurementFromState();

  virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& /*from*/, g2o::OptimizableGraph::Vertex* /*to*/) {return 1.;}
  virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& /*from_*/, g2o::OptimizableGraph::Vertex* /*to_*/);

  const g2o::ParameterSE3Offset* offsetParameter() { return _offsetParam; }

protected:
  virtual bool resolveCaches();
  g2o::ParameterSE3Offset* _offsetParam;
  g2o::CacheSE3Offset* _cache;
};

}

#endif
