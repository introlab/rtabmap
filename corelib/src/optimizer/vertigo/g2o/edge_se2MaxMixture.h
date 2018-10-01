/*
 * edge_se2MaxMixture.h
 *
 *  Created on: 12.06.2012
 *      Author: niko
 */

#ifndef EDGE_SE2MAXMIXTURE_H_
#define EDGE_SE2MAXMIXTURE_H_

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"


class EdgeSE2MaxMixture : public g2o::EdgeSE2
{
  public:
    EdgeSE2MaxMixture();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError();
    void linearizeOplus();

    double weight;
    
    bool nullHypothesisMoreLikely;
    
    InformationType information_nullHypothesis;
    double nu_nullHypothesis;
    InformationType information_constraint;
    double nu_constraint ;
};


#ifdef G2O_HAVE_OPENGL
    class EdgeSE2MaxMixtureDrawAction: public g2o::DrawAction{
    public:
      EdgeSE2MaxMixtureDrawAction();
      virtual g2o::HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element,
              g2o::HyperGraphElementAction::Parameters* params_);
    };
#endif


#endif /* EDGE_SE2MAXMIXTURE_H_ */
