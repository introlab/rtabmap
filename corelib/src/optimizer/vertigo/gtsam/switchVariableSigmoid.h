/*
 * switchVariableSigmoid.h
 *
 *  Created on: 08.08.2012
 *      Author: niko
 */

#ifndef SWITCHVARIABLESIGMOID_H_
#define SWITCHVARIABLESIGMOID_H_

#pragma once

// DerivedValue.h removed from gtsam repo (Dec 2018): https://github.com/borglab/gtsam/commit/e550f4f2aec423cb3f2791b81cb5858b8826ebac
#include "DerivedValue.h"
#include <gtsam/base/Lie.h>

namespace vertigo {

  /**
   * SwitchVariableSigmoid is a wrapper around double to allow it to be a Lie type
   */
  struct SwitchVariableSigmoid : public rtabmap::DerivedValue<SwitchVariableSigmoid> {

    /** default constructor */
    SwitchVariableSigmoid() : d_(10.0) {};

    /** wrap a double */
    SwitchVariableSigmoid(double d) : d_(d) {
      if (d_ < -10.0) d_=-10.0;
      else if(d_>10.0) d_=10.0;
    };

    /** access the underlying value */
    double value() const { return d_; }

    /** print @param s optional string naming the object */
    inline void print(const std::string& name="") const {
      std::cout << name << ": " << d_ << std::endl;
    }

    /** equality up to tolerance */
    inline bool equals(const SwitchVariableSigmoid& expected, double tol=1e-5) const {
      return fabs(expected.d_ - d_) <= tol;
    }

    // Manifold requirements

    /** Returns dimensionality of the tangent space */
    inline size_t dim() const { return 1; }
    inline static size_t Dim() { return 1; }

    /** Update the SwitchVariableSigmoid with a tangent space update */
    inline SwitchVariableSigmoid retract(const gtsam::Vector& v) const {
      double x = value() + v(0);

      if (x>10.0) x=10.0;
      else if (x<-10.0) x=-10.0;

      return SwitchVariableSigmoid(x);
    }

    /** @return the local coordinates of another object */
    inline gtsam::Vector localCoordinates(const SwitchVariableSigmoid& t2) const { return gtsam::Vector1(t2.value() - value()); }

    // Group requirements

    /** identity */
    inline static SwitchVariableSigmoid identity() {
      return SwitchVariableSigmoid();
    }

    /** compose with another object */
    inline SwitchVariableSigmoid compose(const SwitchVariableSigmoid& p) const {
      return SwitchVariableSigmoid(d_ + p.d_);
    }

    /** between operation */
    inline SwitchVariableSigmoid between(const SwitchVariableSigmoid& l2,
#if GTSAM_VERSION_NUMERIC >= 40300
        OptionalMatrixType H1=OptionalNone,
		OptionalMatrixType H2=OptionalNone) const {
#else
        boost::optional<gtsam::Matrix&> H1=boost::none,
        boost::optional<gtsam::Matrix&> H2=boost::none) const {
#endif
      if(H1) *H1 = -gtsam::Matrix::Identity(1, 1);
      if(H2) *H2 = gtsam::Matrix::Identity(1, 1);
      return SwitchVariableSigmoid(l2.value() - value());
    }

    /** invert the object and yield a new one */
    inline SwitchVariableSigmoid inverse() const {
      return SwitchVariableSigmoid(-1.0 * value());
    }

    // Lie functions

    /** Expmap around identity */
    static inline SwitchVariableSigmoid Expmap(const gtsam::Vector& v) { return SwitchVariableSigmoid(v(0)); }

    /** Logmap around identity - just returns with default cast back */
    static inline gtsam::Vector Logmap(const SwitchVariableSigmoid& p) { return gtsam::Vector1(p.value()); }

  private:
      double d_;
  };
}


namespace gtsam {
// Define Key to be Testable by specializing gtsam::traits
template<typename T> struct traits;
template<> struct traits<vertigo::SwitchVariableSigmoid> {
  static void Print(const vertigo::SwitchVariableSigmoid& key, const std::string& str = "") {
    key.print(str);
  }
  static bool Equals(const vertigo::SwitchVariableSigmoid& key1, const vertigo::SwitchVariableSigmoid& key2, double tol = 1e-8) {
    return key1.equals(key2, tol);
  }
  static int GetDimension(const vertigo::SwitchVariableSigmoid & key) {return key.Dim();}

  typedef OptionalJacobian<3, 3> ChartJacobian;
  typedef gtsam::Vector TangentVector;
  static TangentVector Local(const vertigo::SwitchVariableSigmoid& origin, const vertigo::SwitchVariableSigmoid& other,
#if GTSAM_VERSION_NUMERIC >= 40300
	  ChartJacobian Horigin = {}, ChartJacobian Hother = {}) {
#else
	  ChartJacobian Horigin = boost::none, ChartJacobian Hother = boost::none) {
#endif
    return origin.localCoordinates(other);
  }
  static vertigo::SwitchVariableSigmoid Retract(const vertigo::SwitchVariableSigmoid& g, const TangentVector& v,
#if GTSAM_VERSION_NUMERIC >= 40300
        ChartJacobian H1 = {}, ChartJacobian H2 = {}) {
#else
        ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
#endif
      return g.retract(v);
    }
};
}

#endif /* SWITCHVARIABLESIGMOID_H_ */
