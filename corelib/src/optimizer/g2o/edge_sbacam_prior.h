/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * Adapted from EdgeSE3Prior
 */

#ifndef RTAB_G2O_EDGE_SBACAM_PRIOR_H_
#define RTAB_G2O_EDGE_SBACAM_PRIOR_H_

#ifdef RTABMAP_ORB_SLAM
#include "g2o/types/types_six_dof_expmap.h"
#else
#include "g2o/types/sba/types_sba.h"
#endif
#include "g2o/core/base_unary_edge.h"
namespace rtabmap {
  /**
   * \brief EdgeSBACamPrior
    * \brief g2o edge with gravity constraint
   */
class EdgeSBACamPrior : public g2o::BaseUnaryEdge<6, g2o::SE3Quat, VertexCam> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSBACamPrior() {
      setMeasurement(g2o::SE3Quat());
      information().setIdentity();
    }

    void setCameraInvLocalTransform(const g2o::SE3Quat & t)
    {
    	_cameraInvLocalTransform = t;
    }

    // return the error estimate as a 3-vector
    void computeError() {
      const VertexCam* v = static_cast<const VertexCam*>(_vertices[0]);
      g2o::SE3Quat estimate;
#ifdef RTABMAP_ORB_SLAM
      estimate = v->estimate().inverse();
#else
      estimate = v->estimate();
#endif
      g2o::SE3Quat delta = _inverseMeasurement * estimate * _cameraInvLocalTransform;
      _error[0]=delta.translation().x();
      _error[1]=delta.translation().y();
      _error[2]=delta.translation().z();
      _error[3]=delta.rotation().x();
      _error[4]=delta.rotation().y();
      _error[5]=delta.rotation().z();
    }
    
    // jacobian
    virtual void linearizeOplus() {
      _jacobianOplusXi = Eigen::Matrix<double, 6, 6>::Identity();
    }

    virtual void setMeasurement(const g2o::SE3Quat& m){
      _measurement = m;
      _inverseMeasurement = m.inverse();
    }

    virtual bool setMeasurementData(const double* d) override {
      Eigen::Map<const Eigen::Matrix<double, 7, 1, Eigen::ColMajor> > v(d);
      // SE3Quat expects [x, y, z, qx, qy, qz, qw]
      _measurement.fromVector(v);
      _inverseMeasurement = _measurement.inverse();
      return true;
    }
    
    virtual bool getMeasurementData(double* d) const override {
      Eigen::Map<Eigen::Matrix<double, 7, 1, Eigen::ColMajor> > v(d);
      // Returns [x, y, z, qx, qy, qz, qw]
      v = _measurement.toVector();
      return true;
    }
  
    virtual int measurementDimension() const {return 7;}

    virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& /*from*/, 
             g2o::OptimizableGraph::Vertex* /*to*/) { 
      return 1.;
    }

    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to) {
      VertexCam *v = static_cast<VertexCam*>(_vertices[0]);
      assert(v && "Vertex for the Prior edge is not set");

#ifdef RTABMAP_ORB_SLAM
      g2o::SE3Quat newEstimate = _cameraInvLocalTransform * _inverseMeasurement;
#else
      g2o::SE3Quat newEstimate = measurement()*_cameraInvLocalTransform.inverse();
#endif
      if (_information.block<3,3>(0,0).array().abs().sum() == 0){ // do not set translation, as that part of the information is all zero
        newEstimate.setTranslation(v->estimate().translation());
      }
      if (_information.block<3,3>(3,3).array().abs().sum() == 0){ // do not set rotation, as that part of the information is all zero
        newEstimate.setRotation(v->estimate().rotation());
      }
      v->setEstimate(newEstimate);
    }

    virtual bool read(std::istream& is) override { return true; } 
    virtual bool write(std::ostream& os) const override { return true; }
  protected:
    g2o::SE3Quat _inverseMeasurement;
    g2o::SE3Quat _cameraInvLocalTransform;
};

}
#endif
