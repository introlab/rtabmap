/*
 * edge_se3Switchable.cpp
 *
 *  Created on: 17.10.2011
 *      Author: niko
 *
 *  Updated on: 14.01.2013
 *      Author: Christian Kerl <christian.kerl@in.tum.de>
 */

#include "edge_se3Switchable.h"
#include "vertex_switchLinear.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/isometry3d_gradients.h"


using namespace std;
using namespace Eigen;


// ================================================
EdgeSE3Switchable::EdgeSE3Switchable() : g2o::BaseMultiEdge<6, Eigen::Isometry3d>()
{
  resize(3);
  _jacobianOplus.clear();
  _jacobianOplus.push_back(JacobianType(0, 6, 6));
  _jacobianOplus.push_back(JacobianType(0, 6, 6));
  _jacobianOplus.push_back(JacobianType(0, 6, 1));

}
// ================================================
bool EdgeSE3Switchable::read(std::istream& is)
  {    
 /*   g2o::Vector7d meas;
    for (int i=0; i<7; i++) 
      is >> meas[i];
    // normalize the quaternion to recover numerical precision lost by storing as human readable text
    Vector4d::MapType(meas.data()+3).normalize();
    setMeasurement(g2o::internal::fromVectorQT(meas));

    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++) {
        is >> information()(i,j);
        if (i!=j)
          information()(j,i) = information()(i,j);
      }
    return true;*/
	return false;

  }
// ================================================
bool EdgeSE3Switchable::write(std::ostream& os) const
{
   /* g2o::Vector7d meas = g2o::internal::toVectorQT(measurement());
    for (int i=0; i<7; i++) os  << meas[i] << " ";
    for (int i = 0; i < 6; ++i)
      for (int j = i; j < 6; ++j)
        os << " " << information()(i, j);
    return os.good();*/
	return false;
}

// ================================================
void EdgeSE3Switchable::linearizeOplus()
{

    g2o::VertexSE3* from = static_cast<g2o::VertexSE3*>(_vertices[0]);
    g2o::VertexSE3* to = static_cast<g2o::VertexSE3*>(_vertices[1]);
    const VertexSwitchLinear* vSwitch = static_cast<const VertexSwitchLinear*>(_vertices[2]);

    Eigen::Isometry3d E;
    const Eigen::Isometry3d& Xi=from->estimate();
    const Eigen::Isometry3d& Xj=to->estimate();
    const Eigen::Isometry3d& Z=_measurement;
    g2o::internal::computeEdgeSE3Gradient(E, _jacobianOplus[0], _jacobianOplus[1], Z, Xi, Xj);

    _jacobianOplus[0]*=vSwitch->estimate();
    _jacobianOplus[1]*=vSwitch->estimate();

    // derivative w.r.t switch vertex
    _jacobianOplus[2].setZero();
    _jacobianOplus[2] = g2o::internal::toVectorMQT(E) * vSwitch->gradient();
}


// ================================================
void EdgeSE3Switchable::computeError()
{
    const g2o::VertexSE3* v1 = dynamic_cast<const g2o::VertexSE3*>(_vertices[0]);
    const g2o::VertexSE3* v2 = dynamic_cast<const g2o::VertexSE3*>(_vertices[1]);
    const VertexSwitchLinear* v3 = static_cast<const VertexSwitchLinear*>(_vertices[2]);

    Eigen::Isometry3d delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
    _error = g2o::internal::toVectorMQT(delta) * v3->estimate();
}

/*
#include <GL/gl.h>
#ifdef G2O_HAVE_OPENGL
  EdgeSE3SwitchableDrawAction::EdgeSE3SwitchableDrawAction(): DrawAction(typeid(EdgeSE3Switchable).name()){}

  g2o::HyperGraphElementAction* EdgeSE3SwitchableDrawAction::operator()(g2o::HyperGraph::HyperGraphElement* element,
               g2o::HyperGraphElementAction::Parameters* ){
    if (typeid(*element).name()!=_typeName)
      return 0;
    EdgeSE3Switchable* e =  static_cast<EdgeSE3Switchable*>(element);


    g2o::VertexSE3* fromEdge = static_cast<g2o::VertexSE3*>(e->vertices()[0]);
    g2o::VertexSE3* toEdge   = static_cast<g2o::VertexSE3*>(e->vertices()[1]);
    VertexSwitchLinear* s   = static_cast<VertexSwitchLinear*>(e->vertices()[2]);

    glColor3f(s->estimate()*1.0,s->estimate()*0.1,s->estimate()*0.1);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f(fromEdge->estimate().translation().x(),fromEdge->estimate().translation().y(),fromEdge->estimate().translation().z());
    glVertex3f(toEdge->estimate().translation().x(),toEdge->estimate().translation().y(),toEdge->estimate().translation().z());
    glEnd();
    glPopAttrib();
    return this;
  }
#endif
*/
