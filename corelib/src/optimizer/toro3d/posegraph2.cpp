/**********************************************************************
 *
 * This source code is part of the Tree-based Network Optimizer (TORO)
 *
 *   TORO Copyright (c) 2007 Giorgio Grisetti, Cyrill Stachniss, 
 *                           Slawomir Grzonka and  Wolfram Burgard
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

/** \file posegraph2.cpp
 * 
 * \brief Defines the graph of 2D poses, with specific functionalities
 * such as loading, saving, merging constraints, and etc.
 **/

#include "posegraph2.h"

#include <fstream>
#include <sstream>
#include <string>

using namespace std;

namespace AISNavigation {


typedef unsigned int uint;
#define LINESIZE 81920


//#define DEBUG(i) if (verboseLevel>i) cerr


bool TreePoseGraph2::load(const char* filename, bool overrideCovariances){
  clear();
  ifstream is(filename);
  if (!is)
    return false;

  while(is){
    char buf[LINESIZE];
    is.getline(buf,LINESIZE);
    istringstream ls(buf);
    string tag;
    ls >> tag;

    if (tag=="VERTEX" || tag=="VERTEX2"){
      int id;
      Pose p;
      ls >> id >> p.x() >> p.y() >> p.theta(); 
      addVertex(id,p);
 	//DEBUG(2) << "V " << id << endl;
      
    }

    if (tag=="EDGE" || tag=="EDGE2"){
      int id1, id2;
      Pose p;
      InformationMatrix m;
      ls >> id1 >> id2 >> p.x() >> p.y() >> p.theta();
      if (overrideCovariances){
	m.values[0][0]=1;  m.values[1][1]=1; m.values[2][2]=1;
	m.values[0][1]=0;  m.values[0][2]=0; m.values[1][2]=0;
      } else {
	ls >> m.values[0][0] >> m.values[0][1] >> m.values [1][1]
	   >> m.values[2][2] >> m.values[0][2] >> m.values [1][2];
      }
      m.values[1][0]=m.values[0][1];
      m.values[2][0]=m.values[0][2];
      m.values[2][1]=m.values[1][2];
      TreePoseGraph2::Vertex* v1=vertex(id1);
      TreePoseGraph2::Vertex* v2=vertex(id2);
      Transformation t(p);
      addEdge(v1, v2,t ,m);
	 //DEBUG(2) << "E " << id1 << " " << id2 <<  endl;
    }
  }
  return true;
}

bool TreePoseGraph2::loadEquivalences(const char* filename){
  ifstream is(filename);
  if (!is)
    return false;
  EdgeList suppressed;
  uint equivCount=0;
  while (is){
    char buf[LINESIZE];
    is.getline(buf, LINESIZE);
    istringstream ls(buf);
    string tag;
    ls >> tag;
    if (tag=="EQUIV"){
      int id1, id2;
      ls >> id1 >> id2;
      Edge* e=edge(id1,id2);
      if (!e)
	e=edge(id2,id1);
      if (e){
	suppressed.push_back(e);
	equivCount++;
      }
    }
  }
  for (EdgeList::iterator it=suppressed.begin(); it!=suppressed.end(); it++){
    Edge* e=*it;
    if (e->v1->id > e->v2->id)
      revertEdge(e);
    collapseEdge(e);
  }
  for (TreePoseGraph2::VertexMap::iterator it=vertices.begin(); it!=vertices.end(); it++){
    Vertex* v=it->second;
    v->edges.clear();
  }
  for (TreePoseGraph2::EdgeMap::iterator it=edges.begin(); it!=edges.end(); it++){
    TreePoseGraph2::Edge * e=it->second;
    e->v1->edges.push_back(e);
    e->v2->edges.push_back(e);
  }
  return true;
}

bool TreePoseGraph2::saveGnuplot(const char* filename){
  ofstream os(filename);
  if (!os)
    return false;

  for (TreePoseGraph2::EdgeMap::const_iterator it=edges.begin(); it!=edges.end(); it++){
    const TreePoseGraph2::Edge * e=it->second;
    const Vertex* v1=e->v1;
    const Vertex* v2=e->v2;
    
    os << v1->pose.x() << " " << v1->pose.y() << " " << v1->pose.theta() << endl;
    os << v2->pose.x() << " " << v2->pose.y() << " " << v2->pose.theta() << endl;
    os << endl;
  }
  return true;

}

bool TreePoseGraph2::save(const char* filename){
  ofstream os(filename);
  if (!os)
    return false;
  
  for (TreePoseGraph2::VertexMap::const_iterator it=vertices.begin(); it!=vertices.end(); it++){
    const TreePoseGraph2::Vertex* v=it->second;
    os << "VERTEX " 
       << v->id << " " 
       << v->pose.x() << " " 
       << v->pose.y() << " " 
       << v->pose.theta()<< endl; 
  }
  for (TreePoseGraph2::EdgeMap::const_iterator it=edges.begin(); it!=edges.end(); it++){
    const TreePoseGraph2::Edge * e=it->second;
    os << "EDGE " << e->v1->id << " " << e->v2->id << " ";
    Pose p=e->transformation.toPoseType();
    os << p.x() << " " << p.y() << " " << p.theta() << " ";
    os << e->informationMatrix.values[0][0] << " "
       << e->informationMatrix.values[0][1] << " "
       << e->informationMatrix.values[1][1] << " "
       << e->informationMatrix.values[2][2] << " "
       << e->informationMatrix.values[0][2] << " "
       << e->informationMatrix.values[1][2] << endl;
    }
  return true;
}

/** \brief A class (struct) used to print vertex information to a
    stream. Needed for debugging. **/
struct IdPrinter{
  IdPrinter(std::ostream& _os):os(_os){}
  std::ostream& os;
  void perform(TreePoseGraph2::Vertex* v){
    std::cout << "(" << v->id << "," << v->level << ")" << endl;
  }
};

void TreePoseGraph2::printDepth( std::ostream& os ){
  IdPrinter ip(os);
  treeDepthVisit(ip, root);
}

void TreePoseGraph2::printWidth( std::ostream& os ){
  IdPrinter ip(os);
  treeBreadthVisit(ip);
}

/** \brief A class (struct) for realizing the pose update of the
    individual nodes. Assumes the correct order of constraint updates
    (according to the tree level, see RSS07 paper)**/
struct PosePropagator{
  void perform(TreePoseGraph2::Vertex* v){
    if (!v->parent)
      return;
    TreePoseGraph2::Transformation tParent(v->parent->pose);
    TreePoseGraph2::Transformation tNode=tParent*v->parentEdge->transformation;

    //cerr << "EDGE(" << v->parentEdge->v1->id << "," << v->parentEdge->v2->id <<"): " << endl;
    //Pose pParent=v->parent->pose;
    //cerr << "     p=" << pParent.x() << "," << pParent.y() << "," << pParent.theta() <<  endl;
    //Pose pEdge=v->parentEdge->transformation.toPoseType();
    //cerr << "     m=" << pEdge.x() << "," << pEdge.y() << "," << pEdge.theta() <<  endl;
    //Pose pNode=tNode.toPoseType();
    //cerr << "     n=" << pNode.x() << "," << pNode.y() << "," << pNode.theta() <<  endl;

    assert(v->parentEdge->v1==v->parent);
    assert(v->parentEdge->v2==v);
    v->pose=tNode.toPoseType();
  }
};

void TreePoseGraph2::initializeOnTree(){
  PosePropagator pp;
  treeDepthVisit(pp, root);
}


void TreePoseGraph2::printEdgesStat(std::ostream& os){
  for (TreePoseGraph2::EdgeMap::const_iterator it=edges.begin(); it!=edges.end(); it++){
    const TreePoseGraph2::Edge * e=it->second;
    os << "EDGE " << e->v1->id << " " << e->v2->id << " ";
    Pose p=e->transformation.toPoseType();
    os << p.x() << " " << p.y() << " " << p.theta() << " ";
    os << e->informationMatrix.values[0][0] << " "
       << e->informationMatrix.values[0][1] << " "
       << e->informationMatrix.values[1][1] << " "
       << e->informationMatrix.values[2][2] << " "
       << e->informationMatrix.values[0][2] << " "
       << e->informationMatrix.values[1][2] << endl;
    os << "   top=" << e->top->id << " length=" << e->length << endl;
  }
}

void TreePoseGraph2::revertEdgeInfo(Edge* e){
  Transformation it=e->transformation.inv();
  InformationMatrix R;
  R.values[0][0]=e->transformation.rotationMatrix[0][0];
  R.values[0][1]=e->transformation.rotationMatrix[0][1];
  R.values[0][2]=0;

  R.values[1][0]=e->transformation.rotationMatrix[1][0];
  R.values[1][1]=e->transformation.rotationMatrix[1][1];
  R.values[1][2]=0;

  R.values[2][0]=0;
  R.values[2][1]=0;
  R.values[2][2]=1;

  InformationMatrix IM=R.transpose()*e->informationMatrix*R;


  //Pose np=e->transformation.toPoseType();
 
  //Pose ip=it.toPoseType();

  //Transformation tc=it*e->transformation;
  //Pose pc=tc.toPoseType();

  e->transformation=it;
  e->informationMatrix=IM;
};

void TreePoseGraph2::initializeFromParentEdge(Vertex* v){
  Transformation tp=Transformation(v->parent->pose)*v->parentEdge->transformation;
  v->transformation=tp;
  v->pose=tp.toPoseType();
  v->parameters=v->pose;
  v->parameters.x()-=v->parent->pose.x();
  v->parameters.y()-=v->parent->pose.y();
  v->parameters.theta()-=v->parent->pose.theta();
  v->parameters.theta()=atan2(sin(v->parameters.theta()), cos(v->parameters.theta()));
}

void TreePoseGraph2::collapseEdge(Edge* e){
  EdgeMap::iterator ie_it=edges.find(e);
  if (ie_it==edges.end())
    return;
  //VertexMap::iterator it1=vertices.find(e->v1->id);
  //VertexMap::iterator it2=vertices.find(e->v2->id);
  assert(vertices.find(e->v1->id)!=vertices.end());
  assert(vertices.find(e->v2->id)!=vertices.end());

  Vertex* v1=e->v1;
  Vertex* v2=e->v2;
  
  
  // all the edges of v2 become outgoing
  for (EdgeList::iterator it=v2->edges.begin(); it!=v2->edges.end(); it++){
    if ( (*it)->v1!=v2 )
      revertEdge(*it);
  }

 // all the edges of v1 become outgoing
  for (EdgeList::iterator it=v1->edges.begin(); it!=v1->edges.end(); it++){
    if ( (*it)->v1!=v1 )
      revertEdge(*it);
  }

  assert(e->v1==v1);

  InformationMatrix I12=e->informationMatrix;
  CovarianceMatrix  C12=I12.inv();
  Transformation    T12=e->transformation;
  //Pose              p12=T12.toPoseType();

  //Transformation iT12=T12.inv();

  //compute the marginal information of the nodes in the path v1-v2-v*
  for (EdgeList::iterator it2=v2->edges.begin(); it2!=v2->edges.end(); it2++){
    Edge* e2=*it2;
    if (e2->v1==v2){ //edge leaving v2
      //Transformation T2x=e2->transformation;
      //Pose           p2x=T2x.toPoseType();
      InformationMatrix I2x=e2->informationMatrix;
      CovarianceMatrix  C2x=I2x.inv();
      
      //compute the estimate of the vertex based on the path v1-v2-vx
      
      //Transformation tr=iT12*T2x;
      
      //InformationMatrix R;
      //R.values[0][0]=tr.rotationMatrix[0][0];
      //R.values[0][1]=tr.rotationMatrix[0][1];
      //R.values[0][2]=0;
      
      //R.values[1][0]=tr.rotationMatrix[1][0];
      //R.values[1][1]=tr.rotationMatrix[1][1];
      //R.values[1][2]=0;
      
      //R.values[2][0]=0;
      //R.values[2][1]=0;
      //R.values[2][2]=1;

      //CovarianceMatrix CM=R.transpose()*C2x*R;
      
      
      Transformation T1x_pred=T12*e2->transformation;
      Covariance C1x_pred=C12+C2x;
      InformationMatrix I1x_pred=C1x_pred.inv();

      e2->transformation=T1x_pred;
      e2->informationMatrix=I1x_pred;
    }
  }

  //all the edges leaving v1 and leaving v2 and leading to the same point are merged
  std::list<Transformation> tList;
  std::list<InformationMatrix> iList;
  std::list<Vertex*> vList;

  //others are transformed and added to v1
  for (EdgeList::iterator it2=v2->edges.begin(); it2!=v2->edges.end(); it2++){
    Edge* e1x=0;
    Edge* e2x=0;
    if ( ((*it2)->v1!=v1)){
      e2x=*it2;
      for (EdgeList::iterator it1=v1->edges.begin(); it1!=v1->edges.end(); it1++){
	if ((*it1)->v2==(*it2)->v2)
	  e1x=*it1;
      }

    }
    if (e1x && e2x){
      Transformation t1x=e1x->transformation;
      InformationMatrix I1x=e1x->informationMatrix;
      Pose p1x=t1x.toPoseType();

      Transformation t2x=e2x->transformation;
      InformationMatrix I2x=e2x->informationMatrix;;
      Pose p2x=t2x.toPoseType();

      InformationMatrix IM=I1x+I2x;
      CovarianceMatrix  CM=IM.inv();
      InformationMatrix scale1=CM*I1x;
      InformationMatrix scale2=CM*I2x;


      Pose p1=scale1*p1x;
      Pose p2=scale2*p2x;


      //need to recover the angles in a decent way.
      double s=scale1.values[2][2]*sin(p1x.theta())+ scale2.values[2][2]*sin(p2x.theta());
      double c=scale1.values[2][2]*cos(p1x.theta())+ scale2.values[2][2]*cos(p2x.theta());
      
      //DEBUG(2) << "p1x= " << p1x.x() << " " << p1x.y() << " " << p1x.theta() << endl;
      //DEBUG(2) << "p1x_pred= " << p2x.x() << " " << p2x.y() << " " << p2x.theta() << endl;
      
      Pose pFinal(p1.x()+p2.x(), p1.y()+p2.y(), atan2(s,c));
      //DEBUG(2) << "p1x_final= " << pFinal.x() << " " << pFinal.y() << " " << pFinal.theta() << endl;
      
      e1x->transformation=Transformation(pFinal);
      e1x->informationMatrix=IM;
    }
    if (!e1x && e2x){
      tList.push_back(e2x->transformation);
      iList.push_back(e2x->informationMatrix);
      vList.push_back(e2x->v2);
    }
  }
  removeVertex(v2->id);

  std::list<Transformation>::iterator t=tList.begin();
  std::list<InformationMatrix>::iterator i=iList.begin();
  std::list<Vertex*>::iterator v=vList.begin();
  while (i!=iList.end()){
    addEdge(v1,*v,*t,*i);
    i++;
    t++;
    v++;
  }
}

}; //namespace AISNavigation
