#pragma once


#include "vertex_switchLinear.h"
#include "g2o/core/base_unary_edge.h"



class EdgeSwitchPrior : public g2o::BaseUnaryEdge<1, double, VertexSwitchLinear>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSwitchPrior();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setMeasurement(const double & m);

    virtual void linearizeOplus();
    void computeError();
};
