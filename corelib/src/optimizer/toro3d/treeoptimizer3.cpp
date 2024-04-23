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

/** \file treeoptimizer3.cpp
 *
 * \brief Defines the core optimizer class for 3D graphs which is a
 * subclass of TreePoseGraph3
 *
 **/

#include "treeoptimizer3.h"

#include <fstream>
#include <sstream>
#include <string>

using namespace std;

namespace AISNavigation {

//#define DEBUG(i) if (verboseLevel>i) cerr


TreeOptimizer3::TreeOptimizer3(){
  restartOnDivergence=false;
  sortedEdges=0;
  mpl=-1;
  edgeCompareMode=EVComparator<Edge*>::CompareLevel;
}


TreeOptimizer3::~TreeOptimizer3(){
}

void TreeOptimizer3::initializeTreeParameters(){
  ParameterPropagator pp;
  treeDepthVisit(pp,root);
}


void TreeOptimizer3::iterate(TreePoseGraph3::EdgeSet* eset, bool noPreconditioner){
  TreePoseGraph3::EdgeSet* temp=sortedEdges;
  if (eset){
    sortedEdges=eset;
  }

  if (noPreconditioner)
    propagateErrors(false);
  else {
    if (iteration==1)
      computePreconditioner();  
    propagateErrors(true);
  }
  sortedEdges=temp;

  onRestartBegin();
  if (restartOnDivergence){
    double mte, ate;
    double mre, are;
    error(&mre, &mte, &are, &ate);
    maxTranslationalErrors.push_back(mte);
    maxRotationalErrors.push_back(mre);
    int interval=3;
    if ((int)maxRotationalErrors.size()>=interval){
      uint s=(uint)maxRotationalErrors.size();
      double re0 = maxRotationalErrors[s-interval];
      double re1 = maxRotationalErrors[s-1];

      if ((re1-re0)>are || sqrt(re1)>0.99*M_PI){
	double rg=rotGain;
	if (sqrt(re1)>M_PI/4){
	  cerr << "RESTART!!!!! : Angular wraparound may be occourring" << endl;
	  cerr << " err=" << re0 << " -> " << re1 << endl; 
	  cerr << "Restarting optimization and reducing the rotation factor" << endl;
	  cerr << rg << " -> ";
	  initializeOnTree();
	  initializeTreeParameters();
	  initializeOptimization();
	  error(&mre, &mte);
	  maxTranslationalErrors.push_back(mte);
	  maxRotationalErrors.push_back(mre);
	  rg*=0.1;
	  rotGain=rg;
	  cerr << rotGain << endl;
	}
	else {
	  cerr << "decreasing angular gain" << rotGain*0.1 << endl;
	  rotGain*=0.1;
	}
      }
    }
  }
  onRestartDone();
}


void TreeOptimizer3::recomputeTransformations(Vertex*v, Vertex* top){
  if (v==top)
    return;
  recomputeTransformations(v->parent, top);
  v->transformation=v->parent->transformation*v->parameters;
}


void TreeOptimizer3::recomputeParameters(Vertex*v, Vertex* top){
  while (v!=top){
    v->parameters=v->parent->transformation.inv()*v->transformation;
    v=v->parent;
  }
}


TreeOptimizer3::Transformation TreeOptimizer3::getPose(Vertex*v, Vertex* top){
  Transformation t(0.,0.,0.,0.,0.,0.);
  if (v==top)
    return v->transformation;
  while (v!=top){
    t=v->parameters*t;
    v=v->parent;
  }
  return top->transformation*t;
}

TreeOptimizer3::Rotation TreeOptimizer3::getRotation(Vertex*v, Vertex* top){
  Rotation r(0.,0.,0.);
  if (v==top)
    return v->transformation.rotation();
  while (v!=top){
    r=v->parameters.rotation()*r;
    v=v->parent;
  }
  return top->transformation.rotation()*r;
}



double TreeOptimizer3::error(const Edge* e) const{
  const Vertex* v1=e->v1;
  const Vertex* v2=e->v2;
  
  Transformation et=e->transformation;
  Transformation t1=v1->transformation;
  Transformation t2=v2->transformation;
  
  Transformation t12=(t1*et)*t2.inv();
  Pose p12=t12.toPoseType();

  Pose ps=e->informationMatrix*p12;
  double err=p12*ps;
  //DEBUG(100) << "e(" << v1->id << "," << v2->id << ")" << err << endl;
  return err;
 
}

double TreeOptimizer3::traslationalError(const Edge* e) const{
  const Vertex* v1=e->v1;
  const Vertex* v2=e->v2;
  
  Transformation et=e->transformation;
  Transformation t1=v1->transformation;
  Transformation t2=v2->transformation;
  
  Translation t12=(t2.inv()*(t1*et)).translation();
  return t12*t12;;
 
}

double TreeOptimizer3::rotationalError(const Edge* e) const{
  const Vertex* v1=e->v1;
  const Vertex* v2=e->v2;
  
  Rotation er=e->transformation.rotation();
  Rotation r1=v1->transformation.rotation();
  Rotation r2=v2->transformation.rotation();
  
  Rotation r12=r2.inverse()*(r1*er);
  double r=r12.angle();
  return r*r;
}


double TreeOptimizer3::loopError(const Edge* e) const{
  double err=0;
  const Vertex* v=e->v1;
  while (v!=e->top){
    err+=error(v->parentEdge);
    v=v->parent;
  }
  v=e->v2;
  while (v==e->top){
    err+=error(v->parentEdge);
    v=v->parent;
  }
  if (e->v2->parentEdge!=e && e->v1->parentEdge!=e)
    err+=error(e);
  return err;
}

double TreeOptimizer3::loopRotationalError(const Edge* e) const{
  double err=0;
  const Vertex* v=e->v1;
  while (v!=e->top){
    err+=rotationalError(v->parentEdge);
    v=v->parent;
  }
  v=e->v2;
  while (v!=e->top){
    err+=rotationalError(v->parentEdge);
    v=v->parent;
  }
  if (e->v2->parentEdge!=e && e->v1->parentEdge!=e)
    err+=rotationalError(e);
  return err;
}


double TreeOptimizer3::error(double* mre, double* mte, double* are, double* ate, TreePoseGraph3::EdgeSet* eset) const{
  double globalRotError=0.;
  double maxRotError=0;
  double globalTrasError=0.;
  double maxTrasError=0;

  int c=0;
  if (! eset){
    for (TreePoseGraph3::EdgeMap::const_iterator it=edges.begin(); it!=edges.end(); it++){
      double re=rotationalError(it->second);
      globalRotError+=re;
      maxRotError=maxRotError>re?maxRotError:re;
      double te=traslationalError(it->second);
      globalTrasError+=te;
      maxTrasError=maxTrasError>te?maxTrasError:te;
      c++;
    }
  } else {
    for (TreePoseGraph3::EdgeSet::const_iterator it=eset->begin(); it!=eset->end(); it++){
      const TreePoseGraph3::Edge* edge=*it;
      double re=rotationalError(edge);
      globalRotError+=re;
      maxRotError=maxRotError>re?maxRotError:re;
      double te=traslationalError(edge);
      globalTrasError+=te;
      maxTrasError=maxTrasError>te?maxTrasError:te;
      c++;
    }
  }

  if (mte)
    *mte=maxTrasError;
  if (mre)
    *mre=maxRotError;
  if (ate)
    *ate=globalTrasError/c;
  if (are)
    *are=globalRotError/c;
  return globalRotError+globalTrasError;
}



void TreeOptimizer3::initializeOptimization(EdgeCompareMode mode){
  edgeCompareMode=mode;
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
  mpl=maxPathLength();
  rotGain=1.;
  trasGain=1.;
}

void TreeOptimizer3::initializeOnlineIterations(){
  int sz=maxIndex()+1;
  //DEBUG(1) << "Size= " << sz << endl;
  M.resize(sz);
  //DEBUG(1) << "allocating M(" << sz << ")" << endl;
  iteration=1;
  maxRotationalErrors.clear();
  maxTranslationalErrors.clear();
  rotGain=1.;
  trasGain=1.;
}

void TreeOptimizer3::initializeOnlineOptimization(EdgeCompareMode mode){
  edgeCompareMode=mode;
  // compute the size of the preconditioning matrix
  clear();
  Vertex* v0=addVertex(0,Pose(0,0,0,0,0,0));
  root=v0;
  v0->parameters=Transformation(v0->pose);
  v0->parentEdge=0;
  v0->parent=0;
  v0->level=0;
  v0->transformation=Transformation(TreePoseGraph3::Pose(0,0,0,0,0,0));
}

void TreeOptimizer3::onStepStart(Edge* e){
  //DEBUG(5) << "entering edge" << e << endl;
}  
void TreeOptimizer3::onStepFinished(Edge* e){
  //DEBUG(5) << "exiting edge" << e << endl;
}

void TreeOptimizer3::onIterationStart(int iteration){
  //DEBUG(5) << "entering iteration " << iteration << endl;
}

void TreeOptimizer3::onIterationFinished(int iteration){
  //DEBUG(5) << "exiting iteration " << iteration << endl;
}

void TreeOptimizer3::onRestartBegin(){}
void TreeOptimizer3::onRestartDone(){}
bool TreeOptimizer3::isDone(){
  return false;
}


}; //namespace AISNavigation
