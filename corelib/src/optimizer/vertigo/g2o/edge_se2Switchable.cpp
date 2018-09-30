/*
 * edge_se2Switchable.cpp
 *
 *  Created on: 13.07.2011
 *      Author: niko
 *
 *  Updated on: 14.01.2013
 *      Author: Christian Kerl <christian.kerl@in.tum.de>
 */

#include "edge_se2Switchable.h"
#include "vertex_switchLinear.h"

using namespace std;
using namespace Eigen;


// ================================================
EdgeSE2Switchable::EdgeSE2Switchable() : g2o::BaseMultiEdge<3, g2o::SE2>()
{
  resize(3);
  _jacobianOplus.clear();
  _jacobianOplus.push_back(JacobianType(0, 3, 3));
  _jacobianOplus.push_back(JacobianType(0, 3, 3));
  _jacobianOplus.push_back(JacobianType(0, 3, 1));

}
// ================================================
bool EdgeSE2Switchable::read(std::istream& is)
  {
    Vector3d p;
    is >> p[0] >> p[1] >> p[2];
    setMeasurement(g2o::SE2(p));
    _inverseMeasurement = measurement().inverse();

    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }
// ================================================
bool EdgeSE2Switchable::write(std::ostream& os) const
{
    Vector3d p = measurement().toVector();
    os << p.x() << " " << p.y() << " " << p.z();
    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j)
        os << " " << information()(i, j);
    return os.good();
}

// ================================================
void EdgeSE2Switchable::linearizeOplus()
{
   
    const g2o::VertexSE2* vi = static_cast<const g2o::VertexSE2*>(_vertices[0]);
    const g2o::VertexSE2* vj = static_cast<const g2o::VertexSE2*>(_vertices[1]);
    const VertexSwitchLinear* vSwitch = static_cast<const VertexSwitchLinear*>(_vertices[2]);

    double thetai = vi->estimate().rotation().angle();

    Vector2d dt = vj->estimate().translation() - vi->estimate().translation();
    double si=sin(thetai), ci=cos(thetai);

    _jacobianOplus[0](0, 0) = -ci; _jacobianOplus[0](0, 1) = -si; _jacobianOplus[0](0, 2) = -si*dt.x()+ci*dt.y();
    _jacobianOplus[0](1, 0) =  si; _jacobianOplus[0](1, 1) = -ci; _jacobianOplus[0](1, 2) = -ci*dt.x()-si*dt.y();
    _jacobianOplus[0](2, 0) =  0;  _jacobianOplus[0](2, 1) = 0;   _jacobianOplus[0](2, 2) = -1;

    _jacobianOplus[1](0, 0) = ci; _jacobianOplus[1](0, 1)= si; _jacobianOplus[1](0, 2)= 0;
    _jacobianOplus[1](1, 0) =-si; _jacobianOplus[1](1, 1)= ci; _jacobianOplus[1](1, 2)= 0;
    _jacobianOplus[1](2, 0) = 0;  _jacobianOplus[1](2, 1)= 0;  _jacobianOplus[1](2, 2)= 1;

    const g2o::SE2& rmean = _inverseMeasurement;
    Matrix3d z = Matrix3d::Zero();
    z.block<2, 2>(0, 0) = rmean.rotation().toRotationMatrix();
    z(2, 2) = 1.;
    _jacobianOplus[0] = z * _jacobianOplus[0];
    _jacobianOplus[1] = z * _jacobianOplus[1];


    _jacobianOplus[0]*=vSwitch->estimate();
    _jacobianOplus[1]*=vSwitch->estimate();


    // derivative w.r.t switch vertex
    _jacobianOplus[2].setZero();
    g2o::SE2 delta = _inverseMeasurement * (vi->estimate().inverse()*vj->estimate());
    _jacobianOplus[2] = delta.toVector() * vSwitch->gradient();  
}

// ================================================
void EdgeSE2Switchable::computeError()
{
    const g2o::VertexSE2* v1 = static_cast<const g2o::VertexSE2*>(_vertices[0]);
    const g2o::VertexSE2* v2 = static_cast<const g2o::VertexSE2*>(_vertices[1]);
    const VertexSwitchLinear* v3 = static_cast<const VertexSwitchLinear*>(_vertices[2]);
  
    g2o::SE2 delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
    _error = delta.toVector() * v3->estimate();
}

/*
#include <GL/gl.h>
#ifdef G2O_HAVE_OPENGL
  EdgeSE2SwitchableDrawAction::EdgeSE2SwitchableDrawAction(): DrawAction(typeid(EdgeSE2Switchable).name()){}

  g2o::HyperGraphElementAction* EdgeSE2SwitchableDrawAction::operator()(g2o::HyperGraph::HyperGraphElement* element,
               g2o::HyperGraphElementAction::Parameters* ){
    if (typeid(*element).name()!=_typeName)
      return 0;
    EdgeSE2Switchable* e =  static_cast<EdgeSE2Switchable*>(element);


    g2o::VertexSE2* fromEdge = static_cast<g2o::VertexSE2*>(e->vertices()[0]);
    g2o::VertexSE2* toEdge   = static_cast<g2o::VertexSE2*>(e->vertices()[1]);
    VertexSwitchLinear* s   = static_cast<VertexSwitchLinear*>(e->vertices()[2]);

    glColor3f(s->estimate()*1.0,s->estimate()*0.1,s->estimate()*0.1);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f(fromEdge->estimate().translation().x(),fromEdge->estimate().translation().y(),0.);
    glVertex3f(toEdge->estimate().translation().x(),toEdge->estimate().translation().y(),0.);
    glEnd();
    glPopAttrib();
    return this;
  }
#endif
*/
