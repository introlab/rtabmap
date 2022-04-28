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


/**
 * rtabmap: To be used with older g2o version not having this file
 */

#ifndef G2O_EDGE_XYZ_PRIOR_H_
#define G2O_EDGE_XYZ_PRIOR_H_

#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

namespace g2o {

using namespace Eigen;

  /**
   * \brief prior for an XYZ vertex (VertexPointXYZ)
   *
   * Provides a prior for a 3d point vertex. The measurement is represented by a
   * Vector3d with a corresponding 3x3 upper triangle covariance matrix (upper triangle only).
   */
  class EdgeXYZPrior : public BaseUnaryEdge<3, Vector3d, VertexPointXYZ> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeXYZPrior();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError();
    
    // jacobian
    virtual void linearizeOplus();

    virtual void setMeasurement(const Vector3d& m){
      _measurement = m;
    }

    virtual bool setMeasurementData(const double* d){
        Eigen::Map<const Vector3d> v(d);
        _measurement = v;
        return true;
    }

    virtual bool getMeasurementData(double* d) const{
        Eigen::Map<Vector3d> v(d);
        v = _measurement;
        return true;
    }

    virtual int measurementDimension() const { return 3; }

    virtual bool setMeasurementFromState() ;

    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/,
    		OptimizableGraph::Vertex* /*to*/) {
      return 0;
    }
  };

    EdgeXYZPrior::EdgeXYZPrior() : BaseUnaryEdge<3, Vector3d, VertexPointXYZ>() {
      information().setIdentity();
    }

    bool EdgeXYZPrior::read(std::istream& is) {
      // read measurement
  	Vector3d meas;
      for (int i=0; i<3; i++) is >> meas[i];
      setMeasurement(meas);
      // read covariance matrix (upper triangle)
      if (is.good()) {
        for ( int i=0; i<information().rows(); i++)
          for (int j=i; j<information().cols(); j++){
            is >> information()(i,j);
            if (i!=j)
              information()(j,i)=information()(i,j);
          }
      }
      return !is.fail();
    }

    bool EdgeXYZPrior::write(std::ostream& os) const {
      for (int i = 0; i<3; i++) os << measurement()[i] << " ";
      for (int i=0; i<information().rows(); i++)
        for (int j=i; j<information().cols(); j++) {
          os << information()(i,j) << " ";
        }
      return os.good();
    }

    void EdgeXYZPrior::computeError() {
      const VertexPointXYZ* v = static_cast<const VertexPointXYZ*>(_vertices[0]);
      _error = v->estimate() - _measurement;
    }

    void EdgeXYZPrior::linearizeOplus(){
        _jacobianOplusXi = Matrix3d::Identity();
    }

    bool EdgeXYZPrior::setMeasurementFromState(){
        const VertexPointXYZ* v = static_cast<const VertexPointXYZ*>(_vertices[0]);
        _measurement = v->estimate();
        return true;
    }
}
#endif
