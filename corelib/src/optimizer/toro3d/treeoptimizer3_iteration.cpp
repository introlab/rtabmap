/**********************************************************************
 *
 * This source code is part of the Tree-based Network Optimizer (TORO)
 *
 * TORO Copyright (c) 2007 Giorgio Grisetti, Cyrill Stachniss, 
 *                         Slawomir Grzonka and Wolfram Burgard
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

#include <fstream>
#include <string>
#include "treeoptimizer3.h"

using namespace std;

namespace AISNavigation {

//#define DEBUG(i) if (verboseLevel>i) cerr

//helper functions. Should I explain :-)?
inline double max3( const double& a, const double& b, const double& c){
  double m=a>b?a:b;
  return m>c?m:c;
}
inline double min3( const double& a, const double& b, const double& c){
  double m=a<b?a:b;
  return m<c?m:c;
}


struct NodeInfo{
  TreeOptimizer3::Vertex* n;
  double translationalWeight;
  double rotationalWeight;
  int direction;
  TreeOptimizer3::Transformation transformation;
  TreeOptimizer3::Transformation parameters;
  NodeInfo(TreeOptimizer3::Vertex* v=0, double tw=0, double rw=0, int dir=0,
	   TreeOptimizer3::Transformation t=TreeOptimizer3::Transformation(0,0,0,0,0,0), 
	   TreeOptimizer3::Parameters p=TreeOptimizer3::Transformation(0,0,0,0,0,0)){
    n=v;
    translationalWeight=tw;
    rotationalWeight=rw;
    direction=dir;
    transformation=t;
    parameters=p;
  }
};

typedef std::vector<NodeInfo> NodeInfoVector;


/********************************** Preconditioned and unpreconditioned error distribution ************************************/





void TreeOptimizer3::computePreconditioner(){
  for (uint i=0; i<M.size(); i++){
    M[i][0]=0;
    M[i][1]=0;
  }
  gamma[0] = gamma[1] = numeric_limits<double>::max();

  int edgeCount=0;
  for (EdgeSet::iterator it=sortedEdges->begin(); it!=sortedEdges->end(); it++){
    edgeCount++;
    //if (! (edgeCount%1000))
    //  DEBUG(1) << "m";

    Edge* e=*it;
    //Transformation t=e->transformation;
    InformationMatrix W=e->informationMatrix;
    
    Vertex* top=e->top;
    for (int dir=0; dir<2; dir++){
      Vertex* n = (dir==0)? e->v1 : e->v2;
      while (n!=top){
	uint i=n->id;
	double rW=min3(W[0][0], W[1][1], W[2][2]);
	double tW=min3(W[3][3], W[4][4], W[5][5]);
	M[i][0]+=rW;
	M[i][1]+=tW;
	gamma[0]=gamma[0]<rW?gamma[0]:rW;
	gamma[1]=gamma[1]<tW?gamma[1]:tW;
	n=n->parent;
      }
    }

  }
  
  if (verboseLevel>1){
    for (uint i=0; i<M.size(); i++){
      cerr << "M[" << i << "]=" << M[i][0] << " " << M[i][1] << endl;
    }
  }
}


void TreeOptimizer3::propagateErrors(bool usePreconditioner){
  iteration++;
  int edgeCount=0;
  // this is the workspace for computing the paths without 
  // bothering too much the memory allocation
  static NodeInfoVector path;
  path.resize(edges.size()+1);
  static Rotation zero(0.,0.,0.);

  onIterationStart(iteration);
  for (EdgeSet::iterator it=sortedEdges->begin(); it!=sortedEdges->end(); it++){
    edgeCount++;
    //if (! (edgeCount%1000))
    //  DEBUG(1) << "c";

    if (isDone())
      return;
    
    Edge* e=*it;
    Vertex* top=e->top;
    Vertex* v1=e->v1;
    Vertex* v2=e->v2;
    int l=e->length;
    onStepStart(e);
    
    recomputeTransformations(v1,top);
    recomputeTransformations(v2,top);
    //DEBUG(2) << "Edge: " << v1->id << " " << v2->id << ", top=" << top->id << ", length="<< l <<endl;

    //BEGIN: Path and weight computation 
    int pc=0;
    Vertex* aux=v1;
    double totTW=0, totRW=0;
    while(aux!=top){
      int index=aux->id;
      double tw=1./(double)l, rw=1./(double)l;
      if (usePreconditioner){
	tw=1./M[index][0]; 
	rw=1./M[index][1];
      }
      totTW+=tw;
      totRW+=rw;
      path[pc++]=NodeInfo(aux,tw,rw,-1,aux->transformation, aux->parameters);
      aux=aux->parent;
    }
    int topIndex=pc;
    path[pc++]=NodeInfo(top,0.,0.,0, top->transformation, top->parameters);
    pc=l;
    aux=v2;
    while(aux!=top){
      int index=aux->id;
      double tw=1./l, rw=1./l;
      if (usePreconditioner){
	tw=1./M[index][0]; 
	rw=1./M[index][1];
      }
      totTW+=tw;
      totRW+=rw;
      path[pc--]=NodeInfo(aux,tw,rw,1,aux->transformation, aux->parameters);
      aux=aux->parent;
    }

    //store the transformations relative to the top node
    //Transformation topTransformation=top->transformation;
    //Transformation topParameters=top->parameters;

    //END: Path and weight computation
    


    //BEGIN: Rotational Error
    Rotation r1=getRotation(v1, top);
    Rotation r2=getRotation(v2, top);
    Rotation re=e->transformation.rotation();
    Rotation rR=r2.inverse()*(r1*re);

    double rotationFactor=(usePreconditioner)?
      sqrt(double(l))* min3(e->informationMatrix[0][0],
			    e->informationMatrix[1][1], 
			    e->informationMatrix[2][2])/
      ( gamma[0]* (double)iteration ):
      sqrt(double(l))*rotGain/(double)iteration;

//     double rotationFactor=(usePreconditioner)?
//       sqrt(double(l))*rotGain/
//       ( gamma[0]* (double)iteration * min3(e->informationMatrix[0][0],
// 					   e->informationMatrix[1][1], 
// 					   e->informationMatrix[2][2])):
//       sqrt(double(l))*rotGain/(double)iteration;

    if (rotationFactor>1)
      rotationFactor=1;

    Rotation totalRotation = path[l].transformation.rotation() * rR * path[l].transformation.rotation().inverse();

    Translation axis   = totalRotation.axis();
    double angle=totalRotation.angle();

    double cw=0;
    for (int i= 1; i<=topIndex; i++){
      cw+=path[i-1].rotationalWeight/totRW;
      Rotation R=path[i].transformation.rotation();
      Rotation B(axis, angle*cw*rotationFactor);
      R= B*R;
      path[i].transformation.setRotation(R);
    }
    for (int i= topIndex+1; i<=l; i++){
      cw+=path[i].rotationalWeight/totRW;
      Rotation R=path[i].transformation.rotation();
      Rotation B(axis, angle*cw*rotationFactor);
      R= B*R;
      path[i].transformation.setRotation(R);
    }

    //recompute the parameters based on the transformation
    for (int i=0; i<topIndex; i++){
      Vertex* n=path[i].n;
      n->parameters.setRotation(path[i+1].transformation.rotation().inverse()*path[i].transformation.rotation());
    }
    for (int i= topIndex+1; i<=l; i++){
      Vertex* n=path[i].n;
      n->parameters.setRotation(path[i-1].transformation.rotation().inverse()*path[i].transformation.rotation());
    }

    //END: Rotational Error


    //now spread the parameters
    recomputeTransformations(v1,top);
    recomputeTransformations(v2,top);

    //BEGIN: Translational Error
    //Translation topTranslation=top->transformation.translation();
   
    Transformation tr12=v1->transformation*e->transformation;
    Translation tR=tr12.translation()-v2->transformation.translation();


//     double translationFactor=(usePreconditioner)?
//       trasGain*l/( gamma[1]* (double)iteration * min3(e->informationMatrix[3][3],
// 						      e->informationMatrix[4][4], 
// 						      e->informationMatrix[5][5])):
//       trasGain*l/(double)iteration;

    double translationFactor=(usePreconditioner)?
      trasGain*l*min3(e->informationMatrix[3][3],
		      e->informationMatrix[4][4], 
		      e->informationMatrix[5][5])/( gamma[1]* (double)iteration):
      trasGain*l/(double)iteration;

    if (translationFactor>1)
      translationFactor=1;
    Translation dt=tR*translationFactor;

    //left wing
    double lcum=0;
    for (int i=topIndex-1; i>=0; i--){
      Vertex* n=path[i].n;
      lcum-=(usePreconditioner) ? path[i].translationalWeight/totTW :  1./(double)l;
      double fraction=lcum;
      Translation offset= dt*fraction;
      Translation T=n->transformation.translation()+offset;
      n->transformation.setTranslation(T);
    }
    //right wing
    
    double rcum=0;
    for (int i=topIndex+1; i<=l; i++){
      Vertex* n=path[i].n;
      rcum+=(usePreconditioner) ? path[i].translationalWeight/totTW :  1./(double)l;
      double fraction=rcum;
      Translation offset= dt*fraction;
      Translation T=n->transformation.translation()+offset;
      n->transformation.setTranslation(T);
    }
    assert(fabs(lcum+rcum)-1<1e-6);

    recomputeParameters(v1, top);
    recomputeParameters(v2, top);

    //END: Translational Error

    onStepFinished(e);

    if (verboseLevel>2){
      Rotation newRotResidual=v2->transformation.rotation().inverse()*(v1->transformation.rotation()*re);
      Translation newRotResidualAxis=newRotResidual.axis();
      double      newRotResidualAngle=newRotResidual.angle();
      Translation rotResidualAxis=rR.axis();
      double      rotResidualAngle=rR.angle();
      Translation newTransResidual=(v1->transformation*e->transformation).translation()-v2->transformation.translation();

      cerr << "RotationalFraction:  " << rotationFactor << endl;
      cerr << "Rotational residual: " 
	   << " axis " << rotResidualAxis.x()  << "\t" << rotResidualAxis.y()  << "\t" << rotResidualAxis.z() << " --> "
	   << "   ->  " << newRotResidualAxis.x()  << "\t" << newRotResidualAxis.y()  << "\t" << newRotResidualAxis.z()  << endl;
      cerr << " angle " << rotResidualAngle  << "\t" << newRotResidualAngle  << endl;
      
      cerr << "Translational Fraction:  " << translationFactor << endl;
      cerr << "Translational Residual" << endl;
      cerr << "    " << tR.x()  << "\t" << tR.y()  << "\t" << tR.z()  << endl;
      cerr << "    " << newTransResidual.x()  << "\t" << newTransResidual.y()  << "\t" << newTransResidual.z()  << endl;
    }
    
    if (verboseLevel>101){
      char filename [1000];
      sprintf(filename, "po-%02d-%03d-%03d-.dat", iteration, v1->id, v2->id);
      recomputeAllTransformations();
      saveGnuplot(filename);
    }
  }
  onIterationFinished(iteration);
}

};//namespace AISNavigation
