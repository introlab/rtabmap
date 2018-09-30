/*
 * vertex_switchLinear.h
 *
 *  Created on: 17.10.2011
 *      Author: niko
 *
 *  Updated on: 14.01.2013
 *      Author: Christian Kerl <christian.kerl@in.tum.de>
 */


#pragma once

#include "g2o/core/base_vertex.h"
#include <math.h>



    class VertexSwitchLinear : public g2o::BaseVertex<1, double>
    {

    public:
      VertexSwitchLinear();

      virtual void setToOriginImpl();

      virtual void oplusImpl(const double* update);

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;
      virtual void setEstimate(const double &et);


      double x() const { return _x; };


      //! The gradient at the current estimate is always 1;
      double gradient() const { return 1; } ;

    private:
      double _x;

    };
