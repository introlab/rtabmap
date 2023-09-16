/**********************************************************************
 *
 * This source code is part of the Tree-based Network Optimizer (TORO)
 *
 * TORO Copyright (c) 2007 Giorgio Grisetti, Cyrill Stachniss, 
 *                         Slawomir Grzonka, and Wolfram Burgard
 *
 * TORO is licences under the Common Creative License,
 * Attribution-NonCommercial-ShareAlike 3.0
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *  
 *   - Noncommercial. You may not use this work for commercial purposes.
 *  
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * TORO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 **********************************************************************/

/** \file treeoptimizer2.cpp
 *
 * \brief Defines the core optimizer class for 2D graphs which is a
 * subclass of TreePoseGraph2
 *
 **/

#include "treeoptimizer2.h"

#include <fstream>
#include <sstream>
#include <string>

typedef unsigned int uint;
using namespace std;

namespace AISNavigation {

//#define DEBUG(i) if (verboseLevel>i) cerr

/** \brief A class (struct) to compute the parameterization of the vertex v **/
struct ParameterPropagator{
  void perform(TreePoseGraph2::Vertex* v){
    if (!v->parent){
      v->parameters=TreePoseGraph2::Pose(0.,0.,0.);
      return;
    }
    v->parameters=TreePoseGraph2::Pose(v->pose.x()-v->parent->pose.x(), 
		      v->pose.y()-v->parent->pose.y(),
		      v->pose.theta()-v->parent->pose.theta());
  }
};

TreeOptimizer2::TreeOptimizer2():
	iteration(1){
  sortedEdges=0;
}

TreeOptimizer2::~TreeOptimizer2(){
}

void TreeOptimizer2::initializeTreeParameters(){
  ParameterPropagator pp;
  treeDepthVisit(pp, root);
}

void TreeOptimizer2::initializeOptimization(){
  // compute the size of the preconditioning matrix
  int sz=maxIndex()+1;
  //DEBUG(1) << "Size= " << sz << endl;
  M.resize(sz);
  //DEBUG(1) << "allocating M(" << sz << ")" << endl;
  iteration=1;

  // sorting edges
  if (sortedEdges!=0){
    delete sortedEdges;
    sortedEdges=0;
  }
  sortedEdges=sortEdges();
}

void TreeOptimizer2::initializeOnlineOptimization(){
  // compute the size of the preconditioning matrix
  int sz=maxIndex()+1;
  //DEBUG(1) << "Size= " << sz << endl;
  M.resize(sz);
  //DEBUG(1) << "allocating M(" << sz << ")" << endl;
  iteration=1;
}

void TreeOptimizer2::computePreconditioner(){
  gamma[0] = gamma[1] =  gamma[2] = numeric_limits<double>::max();

  for (uint i=0; i<M.size(); i++)
    M[i]=Pose(0.,0.,0.);

  int edgeCount=0;
  for (EdgeSet::iterator it=sortedEdges->begin(); it!=sortedEdges->end(); it++){
    edgeCount++;
    //if (! (edgeCount%10000))
    //  DEBUG(1) << "m";

    Edge* e=*it;
    Transformation t=e->transformation;
    InformationMatrix S=e->informationMatrix;
    
    InformationMatrix R;
    R.values[0][0]=t.rotationMatrix[0][0];
    R.values[0][1]=t.rotationMatrix[0][1];
    R.values[0][2]=0;
    
    R.values[1][0]=t.rotationMatrix[1][0];
    R.values[1][1]=t.rotationMatrix[1][1];
    R.values[1][2]=0;
    
    R.values[2][0]=0;
    R.values[2][1]=0;
    R.values[2][2]=1;

    InformationMatrix W =R*S*R.transpose();
    
    Vertex* top=e->top;
    for (int dir=0; dir<2; dir++){
      Vertex* n = (dir==0)? e->v1 : e->v2;
      while (n!=top){
	uint i=n->id;
	M[i].values[0]+=W.values[0][0];
	M[i].values[1]+=W.values[1][1];
	M[i].values[2]+=W.values[2][2];
	gamma[0]=gamma[0]<W.values[0][0]?gamma[0]:W.values[0][0];
	gamma[1]=gamma[1]<W.values[1][1]?gamma[1]:W.values[1][1];
	gamma[2]=gamma[2]<W.values[2][2]?gamma[2]:W.values[2][2];
	n=n->parent;
      }
    }
  }
  
  if (verboseLevel>1){
    for (uint i=0; i<M.size(); i++){
      cerr << "M[" << i << "]=" << M[i].x() << " " << M[i].y() << " " << M[i].theta() <<endl;
    }
  }
}


void TreeOptimizer2::propagateErrors(){
  iteration++;
  int edgeCount=0;
  
  for (EdgeSet::iterator it=sortedEdges->begin(); it!=sortedEdges->end(); it++){
      edgeCount++;
      //if (! (edgeCount%10000)) 	DEBUG(1) << "c";
      
      Edge* e=*it;
      Vertex* top=e->top;
      

      Vertex* v1=e->v1;
      Vertex* v2=e->v2;

      double l=e->length;
      //DEBUG(2) << "Edge: " << v1->id << " " << v2->id << ", top=" << top->id << ", length="<< l <<endl;
      
      Pose p1=getPose(v1, top);
      Pose p2=getPose(v2, top);

      //DEBUG(2) << " p1=" << p1.x() << " " << p1.y() << " " << p1.theta() << endl;
      //DEBUG(2) << " p2=" << p2.x() << " " << p2.y() << " " << p2.theta() << endl;
      
      Transformation et=e->transformation;
      Transformation t1(p1);
      Transformation t2(p2);

      Transformation t12=t1*et;
    
      Pose p12=t12.toPoseType();
      //DEBUG(2) << " pt2=" << p12.x() << " " << p12.y() << " " << p12.theta() << endl;

      Pose r(p12.x()-p2.x(), p12.y()-p2.y(), p12.theta()-p2.theta());
      double angle=r.theta();
      angle=atan2(sin(angle),cos(angle));
      r.theta()=angle;
      //DEBUG(2) << " e=" << r.x() << " " << r.y() << " " << r.theta() << endl;
    
      InformationMatrix S=e->informationMatrix;
      InformationMatrix R;
      R.values[0][0]=t1.rotationMatrix[0][0];
      R.values[0][1]=t1.rotationMatrix[0][1];
      R.values[0][2]=0;
      
      R.values[1][0]=t1.rotationMatrix[1][0];
      R.values[1][1]=t1.rotationMatrix[1][1];
      R.values[1][2]=0;
      
      R.values[2][0]=0;
      R.values[2][1]=0;
      R.values[2][2]=1;

      InformationMatrix W=R*S*R.transpose();
      Pose d=W*r*2.;

      //DEBUG(2) << " d=" << d.x() << " " << d.y() << " " << d.theta() << endl;
    
      assert(l>0);

      double alpha[3] = { 1./(gamma[0]*iteration), 1./(gamma[1]*iteration), 1./(gamma[2]*iteration) };

      double tw[3]={0.,0.,0.};
      for (int dir=0; dir<2; dir++) {
	Vertex* n = (dir==0)? v1 : v2;
	while (n!=top){
	  uint i=n->id;
	  tw[0]+=1./M[i].values[0];
	  tw[1]+=1./M[i].values[1];
	  tw[2]+=1./M[i].values[2];
	  n=n->parent;
	}
      }

      double beta[3] = {l*alpha[0]*d.values[0], l*alpha[1]*d.values[1], l*alpha[2]*d.values[2]};
      beta[0]=(fabs(beta[0])>fabs(r.values[0]))?r.values[0]:beta[0];
      beta[1]=(fabs(beta[1])>fabs(r.values[1]))?r.values[1]:beta[1];
      beta[2]=(fabs(beta[2])>fabs(r.values[2]))?r.values[2]:beta[2];

      //DEBUG(2) << " alpha=" << alpha[0] << " " << alpha[1] << " " << alpha[2] << endl;
      //DEBUG(2) << " beta=" << beta[0] << " " << beta[1] << " " << beta[2] << endl;
      
      for (int dir=0; dir<2; dir++) {
	Vertex* n = (dir==0)? v1 : v2;
	double sign=(dir==0)? -1. : 1.;
	while (n!=top){
	  uint i=n->id;
	  assert(M[i].values[0]>0);
	  assert(M[i].values[1]>0);
	  assert(M[i].values[2]>0);

	  Pose delta( beta[0]/(M[i].values[0]*tw[0]), beta[1]/(M[i].values[1]*tw[1]), beta[2]/(M[i].values[2]*tw[2]));
	  delta=delta*sign;
	  //DEBUG(2) << "   " << dir << ":"  << i <<"," << n->parent->id << ":"
	  //     << n->parameters.x() << " " << n->parameters.y() << " " << n->parameters.theta() << " -> ";

	  n->parameters.x()+=delta.x();
	  n->parameters.y()+=delta.y();
	  n->parameters.theta()+=delta.theta();
	  //DEBUG(2) << n->parameters.x() << " " << n->parameters.y() << " " << n->parameters.theta()<< endl;
	  n=n->parent;
	}
      }
      updatePoseChain(v1,top);
      updatePoseChain(v2,top);

      //Pose pf1=v1->pose;
      //Pose pf2=v2->pose;
      
      //DEBUG(2) << " pf1=" << pf1.x() << " " << pf1.y() << " " << pf1.theta() << endl;
      //DEBUG(2) << " pf2=" << pf2.x() << " " << pf2.y() << " " << pf2.theta() << endl;
      //DEBUG(2) << " en=" << p12.x()-pf2.x() << " " << p12.y()-pf2.y() << " " << p12.theta()-pf2.theta() << endl;
    }
  
}

void TreeOptimizer2::iterate(TreePoseGraph2::EdgeSet* eset){
  TreePoseGraph2::EdgeSet* temp=sortedEdges;
  if (eset){
    sortedEdges=eset;
  }
  if (iteration==1)
	  computePreconditioner();
  propagateErrors();
  sortedEdges=temp;
}

void TreeOptimizer2::updatePoseChain(Vertex* v, Vertex* top){
  if (v!=top){
    updatePoseChain(v->parent, top);
    v->pose.x()=v->parent->pose.x()+v->parameters.x();
    v->pose.y()=v->parent->pose.y()+v->parameters.y();
    v->pose.theta()=v->parent->pose.theta()+v->parameters.theta();
    return;
  } 
}

TreeOptimizer2::Pose TreeOptimizer2::getPose(Vertex*v, Vertex* top){
  Pose p(0,0,0);
  Vertex* aux=v;
  while (aux!=top){
    p.x()+=aux->parameters.x();
    p.y()+=aux->parameters.y();
    p.theta()+=aux->parameters.theta();
    aux=aux->parent;
  }
  p.x()+=aux->pose.x();
  p.y()+=aux->pose.y();
  p.theta()+=aux->pose.theta();
  return p;
}


double TreeOptimizer2::error(const Edge* e) const{
  const Vertex* v1=e->v1;
  const Vertex* v2=e->v2;
  
  Pose p1=v1->pose;
  Pose p2=v2->pose;
  
  //DEBUG(2) << " p1=" << p1.x() << " " << p1.y() << " " << p1.theta() << endl;
  //DEBUG(2) << " p2=" << p2.x() << " " << p2.y() << " " << p2.theta() << endl;
  
  Transformation et=e->transformation;
  Transformation t1(p1);
  Transformation t2(p2);
  
  Transformation t12=t1*et;
    
  Pose p12=t12.toPoseType();
  //DEBUG(2) << " pt2=" << p12.x() << " " << p12.y() << " " << p12.theta() << endl;

  Pose r(p12.x()-p2.x(), p12.y()-p2.y(), p12.theta()-p2.theta());
  double angle=r.theta();
  angle=atan2(sin(angle),cos(angle));
  r.theta()=angle;
  //DEBUG(2) << " e=" << r.x() << " " << r.y() << " " << r.theta() << endl;
    
  InformationMatrix S=e->informationMatrix;
  InformationMatrix R;
  R.values[0][0]=t1.rotationMatrix[0][0];
  R.values[0][1]=t1.rotationMatrix[0][1];
  R.values[0][2]=0;
      
  R.values[1][0]=t1.rotationMatrix[1][0];
  R.values[1][1]=t1.rotationMatrix[1][1];
  R.values[1][2]=0;
      
  R.values[2][0]=0;
  R.values[2][1]=0;
  R.values[2][2]=1;

  InformationMatrix W=R*S*R.transpose();

  Pose r1=W*r;
  return r.x()*r1.x()+r.y()*r1.y()+r.theta()*r1.theta();
}

double TreeOptimizer2::error() const{
  double globalError=0.;
  for (TreePoseGraph2::EdgeMap::const_iterator it=edges.begin(); it!=edges.end(); it++){
    globalError+=error(it->second);
  }
  return globalError;
}

}; //namespace AISNavigation
