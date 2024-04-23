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
 * * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * TORO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 **********************************************************************/

/** \file posegraph3.cpp
 * 
 * \brief Defines the graph of 3D poses, with specific functionalities
 * such as loading, saving, merging constraints, and etc.
 **/

#include "posegraph3.h"

#include <fstream>
#include <sstream>
#include <string>

using namespace std;

namespace AISNavigation {

#define LINESIZE 81920


//#define DEBUG(i) if (verboseLevel>i) cerr


bool TreePoseGraph3::load(const char* filename, bool overrideCovariances, bool twoDimensions){
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
    
    if (twoDimensions){
      if (tag=="VERTEX"){
	int id;
	Pose p(0.,0.,0.,0.,0.,0.);
	ls >> id >> p.x() >> p.y() >> p.yaw(); 
	TreePoseGraph3::Vertex* v=addVertex(id,p);
	if (v){
	  v->transformation=Transformation(p);
	}
      }
    } else {
      if (tag=="VERTEX3"){
	int id;
	Pose p;
	ls >> id >> p.x() >> p.y() >> p.z() >> p.roll() >> p.pitch() >> p.yaw(); 
	TreePoseGraph3::Vertex* v=addVertex(id,p);
	if (v){
	  v->transformation=Transformation(p);
	}
      }
    }      
  }
  is.clear(); /* clears the end-of-file and error flags */
  is.seekg(0, ios::beg);

  //bool edgesOk=true;
  while(is){
    char buf[LINESIZE];
    is.getline(buf,LINESIZE);
    istringstream ls(buf);
    string tag;
    ls >> tag;
    
    if (twoDimensions){
      if (tag=="EDGE"){
	int id1, id2;
	Pose p(0.,0.,0.,0.,0.,0.);
	InformationMatrix m;
	ls >> id1 >> id2 >> p.x() >> p.y() >> p.yaw();
	m=DMatrix<double>::I(6);
	if (! overrideCovariances){
	  ls >> m[0][0] >> m[0][1] >> m[1][1] >> m[2][2] >> m[0][2] >> m[1][2];
	  m[2][0]=m[0][2]; m[2][1]=m[1][2]; m[1][0]=m[0][1];
	}
      
	TreePoseGraph3::Vertex* v1=vertex(id1);
	TreePoseGraph3::Vertex* v2=vertex(id2);
	Transformation t(p);
	if (!addEdge(v1, v2,t ,m)){
	  cerr << "Fatal, attempting to insert an edge between non existing nodes, skipping";
	  cerr << "edge=" << id1 <<" -> " << id2 << endl;
	  //edgesOk=false;
	} 
      }
    } else {
      if (tag=="EDGE3"){
	int id1, id2;
	Pose p;
	InformationMatrix m;
	ls >> id1 >> id2 >> p.x() >> p.y() >> p.z() >> p.roll() >> p.pitch() >> p.yaw();
	m=DMatrix<double>::I(6);
	if (! overrideCovariances){
	  for (int i=0; i<6; i++)
	    for (int j=i; j<6; j++)
	      ls >> m[i][j];
	}
	TreePoseGraph3::Vertex* v1=vertex(id1);
	TreePoseGraph3::Vertex* v2=vertex(id2);
	Transformation t(p);
	if (!addEdge(v1, v2,t ,m)){
	  cerr << "Fatal, attempting to insert an edge between non existing nodes, skipping";
	  cerr << "edge=" << id1 <<" -> " << id2 << endl;
	  //edgesOk=false;
	}
      }
    }
  }
  return true;
  //return edgesOk;
}

bool TreePoseGraph3::loadEquivalences(const char* filename){
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
  return true;
}

bool TreePoseGraph3::saveGnuplot(const char* filename){
  ofstream os(filename);
  if (!os)
    return false;
  for (TreePoseGraph3::VertexMap::iterator it=vertices.begin(); it!=vertices.end(); it++){
    TreePoseGraph3::Vertex* v=it->second;
    v->pose=v->transformation.toPoseType();
  }
  for (TreePoseGraph3::EdgeMap::const_iterator it=edges.begin(); it!=edges.end(); it++){
    const TreePoseGraph3::Edge * e=it->second;
    const Vertex* v1=e->v1;
    const Vertex* v2=e->v2;
    
    os << v1->pose.x() << " " << v1->pose.y() << " " << v1->pose.z() << " "
       << v1->pose.roll() << " " << v1->pose.pitch() << " " << v1->pose.yaw() <<endl;
    os << v2->pose.x() << " " << v2->pose.y() << " " << v2->pose.z() << " "
       << v2->pose.roll() << " " << v2->pose.pitch() << " " << v2->pose.yaw() <<endl;
    os << endl << endl;
  }
  return true;

}

bool TreePoseGraph3::save(const char* filename){
  ofstream os(filename);
  if (!os)
    return false;
  
  for (TreePoseGraph3::VertexMap::iterator it=vertices.begin(); it!=vertices.end(); it++){
    TreePoseGraph3::Vertex* v=it->second;
    v->pose=v->transformation.toPoseType();

    os << "VERTEX3 " 
       << v->id << " " 
       << v->pose.x() << " " 
       << v->pose.y() << " " 
       << v->pose.z() << " "
       << v->pose.roll() << " " 
       << v->pose.pitch() << " " 
       << v->pose.yaw() << endl; 
  }
  for (TreePoseGraph3::EdgeMap::const_iterator it=edges.begin(); it!=edges.end(); it++){
    const TreePoseGraph3::Edge * e=it->second;
    os << "EDGE3 " << e->v1->id << " " << e->v2->id << " ";
    Pose p=e->transformation.toPoseType();
    os << p.x() << " " << p.y() << " " << p.z() << " " << p.roll() << " " << p.pitch() << " " << p.yaw() << " ";
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++)
	os << e->informationMatrix[i][j] << " ";
    os << endl;
  }
  return true;
}

/** \brief A class (struct) used to print vertex information to a
    stream. Needed for debugging. **/
struct IdPrinter{
  IdPrinter(std::ostream& _os):os(_os){}
  std::ostream& os;
  void perform(TreePoseGraph3::Vertex* v){
    std::cout << "(" << v->id << "," << v->level << ")" << endl;
  }
};

void TreePoseGraph3::printDepth( std::ostream& os ){
  IdPrinter ip(os);
  treeDepthVisit(ip, root);
}

void TreePoseGraph3::printWidth( std::ostream& os ){
  IdPrinter ip(os);
  treeBreadthVisit(ip);
}

/** \brief A class (struct) for realizing the pose update of the
    individual nodes. Assumes the correct order of constraint updates
    (according to the tree level, see RSS07 paper)**/

struct PosePropagator{
  void perform(TreePoseGraph3::Vertex* v){
    if (!v->parent)
      return;
    TreePoseGraph3::Transformation tParent(v->parent->transformation);
    TreePoseGraph3::Transformation tNode=tParent*v->parentEdge->transformation;

    assert(v->parentEdge->v1==v->parent);
    assert(v->parentEdge->v2==v);
    v->transformation=tNode;
  }
};

void TreePoseGraph3::initializeOnTree(){
  PosePropagator pp;
  treeDepthVisit(pp, root);
}


void TreePoseGraph3::printEdgesStat(std::ostream& os){
  for (TreePoseGraph3::EdgeMap::const_iterator it=edges.begin(); it!=edges.end(); it++){
    const TreePoseGraph3::Edge * e=it->second;
    os << "EDGE " << e->v1->id << " " << e->v2->id << " ";
    Pose p=e->transformation.toPoseType();
    os << p.x() << " " << p.y() << " " << p.z() << " " << p.roll() << " " << p.pitch() << " " << p.yaw() << endl;
    os << "   top=" << e->top->id << " length=" << e->length << endl;
  }
}

void TreePoseGraph3::revertEdgeInfo(Edge* e){
  // here we assume uniform covariances, and we neglect the transofrmation
  // induced by the Jacobian when reverting the link
  e->transformation=e->transformation.inv();
};

void TreePoseGraph3::initializeFromParentEdge(Vertex* v){
  Transformation tp=Transformation(v->parent->pose)*v->parentEdge->transformation;
  v->transformation=tp;
  v->pose=tp.toPoseType();
  v->parameters=v->parentEdge->transformation;
}


void TreePoseGraph3::collapseEdge(Edge* e){
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
  Pose              p12=T12.toPoseType();

  //Transformation iT12=T12.inv();

  //compute the marginal information of the nodes in the path v1-v2-v*
  for (EdgeList::iterator it2=v2->edges.begin(); it2!=v2->edges.end(); it2++){
    Edge* e2=*it2;
    if (e2->v1==v2){ //edge leaving v2
      Transformation T2x=e2->transformation;
      Pose           p2x=T2x.toPoseType();
      InformationMatrix I2x=e2->informationMatrix;
      CovarianceMatrix  C2x=I2x.inv();
      
      //compute the estimate of the vertex based on the path v1-v2-vx
      
      //Transformation tr=iT12*T2x;
      
      CovarianceMatrix CM=C2x;
      
      
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
    // FIXME
    // edges leading to the same node are ignored
    // should be merged
    if (e1x && e2x){
      // here goes something for mergin the constraints, according to the information matrices.
      // in 3D it is a nightmare, so i postpone this, and i simply ignore the redundant constraints.
      // the resultng system is overconfident
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

void TreePoseGraph3::recomputeAllTransformations(){
  TransformationPropagator tp;
  treeDepthVisit(tp,root);
}

}; //namespace AISNavigation
