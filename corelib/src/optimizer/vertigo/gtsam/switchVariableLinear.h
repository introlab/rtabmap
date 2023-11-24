/*
 * switchVariableLinear.h
 *
 *  Created on: 02.08.2012
 *      Author: niko
 */

#ifndef SWITCHVARIABLELINEAR_H_
#define SWITCHVARIABLELINEAR_H_

#pragma once

// DerivedValue2.h removed from gtsam repo (Dec 2018): https://github.com/borglab/gtsam/commit/e550f4f2aec423cb3f2791b81cb5858b8826ebac
#include "DerivedValue.h"
#include <gtsam/base/Lie.h>

namespace vertigo {

  /**
   * SwitchVariableLinear is a wrapper around double to allow it to be a Lie type
   */
  struct SwitchVariableLinear : public rtabmap::DerivedValue<SwitchVariableLinear> {

    /** default constructor */
    SwitchVariableLinear() : d_(0.0) {};

    /** wrap a double */
    SwitchVariableLinear(double d) : d_(d) {
     // if (d_ < 0.0) d_=0.0;
     // else if(d_>1.0) d_=1.0;
    };

    /** access the underlying value */
    double value() const { return d_; }

    /** print @param s optional string naming the object */
    inline void print(const std::string& name="") const {
      std::cout << name << ": " << d_ << std::endl;
    }

    /** equality up to tolerance */
    inline bool equals(const SwitchVariableLinear& expected, double tol=1e-5) const {
      return fabs(expected.d_ - d_) <= tol;
    }

    // Manifold requirements

    /** Returns dimensionality of the tangent space */
    inline size_t dim() const { return 1; }
    inline static size_t Dim() { return 1; }

    /** Update the SwitchVariableLinear with a tangent space update */
    inline SwitchVariableLinear retract(const gtsam::Vector& v) const {
      double x = value() + v(0);

      if (x>1.0) x=1.0;
      else if (x<0.0) x=0.0;

      return SwitchVariableLinear(x);
    }

    /** @return the local coordinates of another object */
    inline gtsam::Vector localCoordinates(const SwitchVariableLinear& t2) const { return gtsam::Vector1(t2.value() - value()); }

    // Group requirements

    /** identity */
    inline static SwitchVariableLinear identity() {
      return SwitchVariableLinear();
    }

    /** compose with another object */
    inline SwitchVariableLinear compose(const SwitchVariableLinear& p) const {
      return SwitchVariableLinear(d_ + p.d_);
    }

    /** between operation */
    inline SwitchVariableLinear between(const SwitchVariableLinear& l2,
#if GTSAM_VERSION_NUMERIC >= 40300
		OptionalMatrixType H1=OptionalNone,
		OptionalMatrixType H2=OptionalNone) const {
#else
        boost::optional<gtsam::Matrix&> H1=boost::none,
        boost::optional<gtsam::Matrix&> H2=boost::none) const {
#endif
      if(H1) *H1 = -gtsam::Matrix::Identity(1, 1);
      if(H2) *H2 = gtsam::Matrix::Identity(1, 1);
      return SwitchVariableLinear(l2.value() - value());
    }

    /** invert the object and yield a new one */
    inline SwitchVariableLinear inverse() const {
      return SwitchVariableLinear(-1.0 * value());
    }

    // Lie functions

    /** Expmap around identity */
    static inline SwitchVariableLinear Expmap(const gtsam::Vector& v) { return SwitchVariableLinear(v(0)); }

    /** Logmap around identity - just returns with default cast back */
    static inline gtsam::Vector Logmap(const SwitchVariableLinear& p) { return gtsam::Vector1(p.value()); }

  private:
      double d_;
  };
}

namespace gtsam {
// Define Key to be Testable by specializing gtsam::traits
template<typename T> struct traits;
template<> struct traits<vertigo::SwitchVariableLinear> {
  static void Print(const vertigo::SwitchVariableLinear& key, const std::string& str = "") {
    key.print(str);
  }
  static bool Equals(const vertigo::SwitchVariableLinear& key1, const vertigo::SwitchVariableLinear& key2, double tol = 1e-8) {
    return key1.equals(key2, tol);
  }
  static int GetDimension(const vertigo::SwitchVariableLinear & key) {return key.Dim();}

  typedef OptionalJacobian<3, 3> ChartJacobian;
  typedef gtsam::Vector TangentVector;
  static TangentVector Local(const vertigo::SwitchVariableLinear& origin, const vertigo::SwitchVariableLinear& other,
#if GTSAM_VERSION_NUMERIC >= 40300
      ChartJacobian Horigin = {}, ChartJacobian Hother = {}) {
#else
      ChartJacobian Horigin = boost::none, ChartJacobian Hother = boost::none) {
#endif
    return origin.localCoordinates(other);
  }
  static vertigo::SwitchVariableLinear Retract(const vertigo::SwitchVariableLinear& g, const TangentVector& v,
#if GTSAM_VERSION_NUMERIC >= 40300
      ChartJacobian H1 = {}, ChartJacobian H2 = {}) {
#else
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
#endif
      return g.retract(v);
    }
};
}



#endif /* SWITCHVARIABLELINEAR_H_ */
