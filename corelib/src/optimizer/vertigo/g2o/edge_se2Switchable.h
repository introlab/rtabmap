/*
 * edge_se2Switchable.h
 *
 *  Created on: 13.07.2011
 *      Author: niko
 *
 *  Updated on: 14.01.2013
 *      Author: Christian Kerl <christian.kerl@in.tum.de>
 */

#ifndef EDGE_SE2SWITCHABLE_H_
#define EDGE_SE2SWITCHABLE_H_

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/hyper_graph_action.h"

class EdgeSE2Switchable : public g2o::BaseMultiEdge<3, g2o::SE2>
{
  public:
    EdgeSE2Switchable();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError();
    void linearizeOplus();


    virtual void setMeasurement(const g2o::SE2& m){
      _measurement = m;
      _inverseMeasurement = m.inverse();
    }

    protected:
      g2o::SE2 _inverseMeasurement;
};


#ifdef G2O_HAVE_OPENGL
    class EdgeSE2SwitchableDrawAction: public g2o::DrawAction{
    public:
      EdgeSE2SwitchableDrawAction();
      virtual g2o::HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element,
              g2o::HyperGraphElementAction::Parameters* params_);
    };
#endif



#endif /* EDGE_SE2SWITCHABLE_H_ */
