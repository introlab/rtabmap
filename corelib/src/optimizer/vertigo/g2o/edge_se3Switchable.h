/*
 * edge_se3Switchable.h
 *
 *  Created on: 17.10.2011
 *      Author: niko
 *
 *  Updated on: 14.01.2013
 *      Author: Christian Kerl <christian.kerl@in.tum.de>
 */

#ifndef EDGE_SE3SWITCHABLE_H_
#define EDGE_SE3SWITCHABLE_H_

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/hyper_graph_action.h"

class EdgeSE3Switchable : public g2o::BaseMultiEdge<6, Eigen::Isometry3d>
{
  public:
    EdgeSE3Switchable();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError();
    void linearizeOplus();

    virtual void setMeasurement(const Eigen::Isometry3d& m){
      _measurement = m;
      _inverseMeasurement = m.inverse();
    }

  protected:
    Eigen::Isometry3d _inverseMeasurement;
};


#ifdef G2O_HAVE_OPENGL
    class EdgeSE3SwitchableDrawAction: public g2o::DrawAction{
    public:
      EdgeSE3SwitchableDrawAction();
      virtual g2o::HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element,
              g2o::HyperGraphElementAction::Parameters* params_);
    };
#endif


#endif /* EDGE_SE3SWITCHABLE_H_ */
