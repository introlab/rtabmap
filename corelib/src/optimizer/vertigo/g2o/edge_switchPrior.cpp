#include "edge_switchPrior.h"
using namespace std;

    EdgeSwitchPrior::EdgeSwitchPrior()
    {
    	setMeasurement(1.0);
    }

    bool EdgeSwitchPrior::read(std::istream &is)
    {
      double new_measurement;
      is >> new_measurement;

      setMeasurement(new_measurement);

      is >> information()(0,0);
      return true;
    }

    bool EdgeSwitchPrior::write(std::ostream &os) const
    {
      os << measurement() << " " << information()(0,0);
      return true;
    }

    void EdgeSwitchPrior::setMeasurement(const double & m)
    {
    	g2o::BaseEdge<1, double>::setMeasurement(m);
    	information()(0,0) = 1.0;
    }

    void EdgeSwitchPrior::linearizeOplus()
    {
      _jacobianOplusXi[0]=-1.0;
    }

    void EdgeSwitchPrior::computeError()
    {
      const VertexSwitchLinear* s = static_cast<const VertexSwitchLinear*>(_vertices[0]);

      _error[0] = measurement() - s->x();

    }

