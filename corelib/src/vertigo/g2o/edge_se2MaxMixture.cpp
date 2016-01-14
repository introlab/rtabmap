/*
 * edge_se2MaxMixture.cpp
 *
 *  Created on: 12.06.2012
 *      Author: niko
 */


#include "edge_se2MaxMixture.h"

using namespace std;
using namespace Eigen;

// ================================================
EdgeSE2MaxMixture::EdgeSE2MaxMixture()
{
  nullHypothesisMoreLikely = false;
}

// ================================================
bool EdgeSE2MaxMixture::read(std::istream& is)
  {
    Vector3d p;
    is >>  weight >> p[0] >> p[1] >> p[2];
    setMeasurement(g2o::SE2(p));
    _inverseMeasurement = measurement().inverse();
    //measurement().fromVector(p);
    //inverseMeasurement() = measurement().inverse();
    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }

    information_constraint = _information;
    nu_constraint = 1.0/sqrt(information_constraint.inverse().determinant());
    information_nullHypothesis = information_constraint*weight;
    nu_nullHypothesis = 1.0/sqrt(information_nullHypothesis.inverse().determinant());

    return true;
  }
// ================================================
bool EdgeSE2MaxMixture::write(std::ostream& os) const
{
    Vector3d p = measurement().toVector();
    os << p.x() << " " << p.y() << " " << p.z();
    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j)
        os << " " << information()(i, j);
    return os.good();
}

// ================================================
void EdgeSE2MaxMixture::linearizeOplus()
{
  g2o::EdgeSE2::linearizeOplus();
  if (nullHypothesisMoreLikely) {
    _jacobianOplusXi *= weight;
    _jacobianOplusXj *= weight;
  }
}

// ================================================
void EdgeSE2MaxMixture::computeError()
{
  // calculate the error for this constraint
  g2o::EdgeSE2::computeError();

  // determine the likelihood for constraint and null hypothesis
  double mahal_constraint = _error.transpose() * information_constraint * _error;
  double likelihood_constraint = nu_constraint * exp(-mahal_constraint);

  double mahal_nullHypothesis  = _error.transpose() * (information_nullHypothesis) * _error;
  double likelihood_nullHypothesis = nu_nullHypothesis * exp(-mahal_nullHypothesis);

  // if the nullHypothesis is more likely ...
  if (likelihood_nullHypothesis > likelihood_constraint) {
    _information = information_nullHypothesis;
    nullHypothesisMoreLikely = true;
  }
  else {
    _information = information_constraint;
    nullHypothesisMoreLikely = false;
  }


}

/*
#include <GL/gl.h>
// ================================================
#ifdef G2O_HAVE_OPENGL
  EdgeSE2MaxMixtureDrawAction::EdgeSE2MaxMixtureDrawAction(): DrawAction(typeid(EdgeSE2MaxMixture).name()){}

  g2o::HyperGraphElementAction* EdgeSE2MaxMixtureDrawAction::operator()(g2o::HyperGraph::HyperGraphElement* element,
               g2o::HyperGraphElementAction::Parameters* ){
    if (typeid(*element).name()!=_typeName)
      return 0;
    EdgeSE2MaxMixture* e =  static_cast<EdgeSE2MaxMixture*>(element);


    g2o::VertexSE2* fromEdge = static_cast<g2o::VertexSE2*>(e->vertices()[0]);
    g2o::VertexSE2* toEdge   = static_cast<g2o::VertexSE2*>(e->vertices()[1]);


    if (e->nullHypothesisMoreLikely) glColor3f(0.0,0.0,0.0);
    else glColor3f(1.0,0.5,0.2);

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


