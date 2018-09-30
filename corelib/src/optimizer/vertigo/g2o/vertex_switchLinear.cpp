/*
 * vertex_switchLinear.cpp
 *
 *  Created on: 17.10.2011
 *      Author: niko
 *
 *  Updated on: 14.01.2013
 *      Author: Christian Kerl <christian.kerl@in.tum.de>
 */

#include "vertex_switchLinear.h"
#include <iostream>

using namespace std;

    VertexSwitchLinear::VertexSwitchLinear() :
		_x(0)
    {
    	setToOrigin();
    	setEstimate(1.0);
    }

    bool VertexSwitchLinear:: read(std::istream& is)
    {
      is >> _x;
      _estimate=_x;

      return true;
    }

    bool  VertexSwitchLinear::write(std::ostream& os) const
    {
      os << _x;
      return os.good();
    }

    void VertexSwitchLinear::setToOriginImpl()
    {
      _x=0;
      _estimate=_x;
    }


    void VertexSwitchLinear::setEstimate(const double &et)
    {
      _x=et;
      _estimate=_x;
    }


    void VertexSwitchLinear::oplusImpl(const double* update)
    {
      _x += update[0];

      if (_x<0) _x=0;
      if (_x>1) _x=1;

      _estimate=_x;
    }
