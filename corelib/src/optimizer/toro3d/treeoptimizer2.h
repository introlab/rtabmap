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

/** \file treeoptimizer2.hh
 *
 * \brief Defines the core optimizer class for 2D graphs which is a
 * subclass of TreePoseGraph2
 *
 **/

#ifndef _TREEOPTIMIZER2_HH_
#define _TREEOPTIMIZER2_HH_

#include "posegraph2.h"

namespace AISNavigation {

/** \brief Class that contains the core optimization algorithm **/
struct TreeOptimizer2: public TreePoseGraph2{
  typedef std::vector<Pose> PoseVector;

  /** Constructor **/
  TreeOptimizer2();

  /** Destructor **/
  virtual ~TreeOptimizer2();

  /** Initialization function **/
  void initializeTreeParameters();

  /** Initialization function **/
  void initializeOptimization();

  /** Initialization function **/
  void initializeOnlineOptimization();
  
  /** Performs one iteration of the algorithm **/
  void iterate(TreePoseGraph2::EdgeSet* eset=0);

  /** Conmputes the gloabl error of the network **/
  double error() const;

protected:
  /** The first of the two main steps of each iteration **/
  void computePreconditioner();

  /** The second of the two main steps of each iteration **/
  void propagateErrors();

  /** Recomputes the poses of all vertices from v to an arbitraty
      parent (top) of v in the tree **/
  void updatePoseChain(Vertex* v, Vertex* top);

  /** Recomputes only the pose of the node v wrt. to an arbitraty
      parent (top) of v in the tree **/
  Pose getPose(Vertex*v, Vertex* top);

  /** Conmputes the error of the constraint/edge e **/
  double error(const Edge* e) const;

  /** Iteration counter **/
  int iteration;

  /** Used to compute the learning rate lambda **/
  double gamma[3];

  /** The diaginal block elements of the preconditioning matrix (D_k
      in the paper) **/
  PoseVector M;

};

}; //namespace AISNavigation
#endif
