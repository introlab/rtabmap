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

/** \file treeoptimizer3.hh
 *
 * \brief Defines the core optimizer class for 3D graphs which is a
 * subclass of TreePoseGraph3
 *
 **/

#ifndef _TREEOPTIMIZER3_HH_
#define _TREEOPTIMIZER3_HH_

#include "posegraph3.h"

namespace AISNavigation {

/** \brief Class that contains the core optimization algorithm **/
struct TreeOptimizer3: public TreePoseGraph3{
  typedef std::vector<Pose> PoseVector;

  /** Constructor **/
  TreeOptimizer3();

  /** Destructor **/
  virtual ~TreeOptimizer3();

  /** Initialization function **/
  void initializeTreeParameters();

  /** Initialization function **/
  void initializeOptimization(EdgeCompareMode mode=EVComparator<Edge*>::CompareLevel);

  void initializeOnlineOptimization(EdgeCompareMode mode=EVComparator<Edge*>::CompareLevel);

  void initializeOnlineIterations();

  /** Performs one iteration of the algorithm **/
  void iterate(TreePoseGraph3::EdgeSet* eset=0, bool noPreconditioner=false);

  /** Conmputes the gloabl error of the network **/
  double error(double* mre=0, double* mte=0, double* are=0, double* ate=0, TreePoseGraph3::EdgeSet* eset=0) const;

  /** Conmputes the gloabl error of the network **/
  double angularError() const;

  /** Conmputes the gloabl error of the network **/
  double translationalError() const;

  bool restartOnDivergence;

  inline double getRotGain() const {return rotGain;} 

  /** Iteration counter **/
  int iteration;

  double rpFraction;

protected:

  /** Recomputes only the pose of the node v wrt. to an arbitraty
      parent (top) of v in the tree **/
  Transformation getPose(Vertex*v, Vertex* top);

  /** Recomputes only the pose of the node v wrt. to an arbitraty
      parent (top) of v in the tree **/
  Rotation getRotation(Vertex*v, Vertex* top);

  void recomputeTransformations(Vertex*v, Vertex* top);

  void recomputeParameters(Vertex*v, Vertex* top);

  void computePreconditioner();

  void propagateErrors(bool usePreconditioner=false);

  /** Computes the error of the constraint/edge e **/
  double error(const Edge* e) const;

  /** Computes the error of the constraint/edge e **/
  double loopError(const Edge* e) const;

  /** Computes the rotational error of the constraint/edge e **/
  double loopRotationalError(const Edge* e) const;
  
  /** Conmputes the error of the constraint/edge e **/
  double translationalError(const Edge* e) const;

  /** Conmputes the error of the constraint/edge e **/
  double rotationalError(const Edge* e) const;

  double traslationalError(const Edge* e) const;
  
  /** Used to compute the learning rate lambda **/
  double gamma[2];

  /** The simplified version of the preconditioning matrix **/
  struct PM_t{
    double v [2];
    inline double& operator[](int i){return v[i];}
  };
  typedef std::vector< PM_t > PMVector;
  PMVector M;

  /**cached maximum path length*/
  int mpl;

  /**history of rhe maximum rotational errors*, used when adaptiveRestart is enabled */
  std::vector<double> maxRotationalErrors;

  /**history of rhe maximum rotational errors*, used when adaptiveRestart is enabled */
  std::vector<double> maxTranslationalErrors;
  double rotGain, trasGain;

  /**callback invoked before starting the optimization of an individual constraint,
   @param e: the constraint being optimized*/
  virtual void onStepStart(Edge* e);

  /**callback invoked after finishing the optimization of an individual constraint,
   @param e: the constraint optimized*/
  virtual void onStepFinished(Edge* e);

  /**callback invoked before starting a full iteration,
   @param i: the current iteration number*/
  virtual void onIterationStart(int i);

  /**callback invoked after finishing a full iteration,
   @param i: the current iteration number*/

  virtual void onIterationFinished(int iteration);

  /**callback invoked before a restart of the optimizer
   when the angular wraparound is detected*/
  virtual void onRestartBegin();

  /**callback invoked after a restart of the optimizer*/
  virtual void onRestartDone();

  /**callback for determining a termination condition,
     it can be used by an external thread for stopping the optimizer while performing an iteration.
     @returns true when the optimizer has to stop.*/
  virtual bool isDone();
  
};

}; //namespace AISNavigation

#endif
