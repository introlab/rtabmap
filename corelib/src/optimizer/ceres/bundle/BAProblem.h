/*
 * BAProblem.h
 *
 *  Created on: Aug 16, 2019
 *      Author: mathieu
 */

#ifndef CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_BAPROBLEM_H_
#define CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_BAPROBLEM_H_

namespace ceres {

class BAProblem {
 public:
	BAProblem() :
		num_cameras_(0),
		num_points_(0),
		num_observations_(0),
		point_index_(NULL),
		camera_index_(NULL),
		observations_(NULL),
		cameras_(NULL),
		points_(NULL)
 {}
  ~BAProblem() {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] cameras_;
    delete[] points_;
  }
  int num_observations()       const { return num_observations_;}
  const double* observations() const { return observations_;}
  double* mutable_camera_for_observation(int i) {
    return cameras_ + camera_index_[i] * 6;
  }
  double* mutable_point_for_observation(int i) {
    return points_ + point_index_[i] * 3;
  }
  int num_cameras_;
  int num_points_;
  int num_observations_;
  int* point_index_;
  int* camera_index_;
  double* observations_;
  double* cameras_;
  double* points_;
};

}

#endif /* CORELIB_SRC_OPTIMIZER_CERES_BUNDLE_BAPROBLEM_H_ */
