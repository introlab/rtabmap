/*
 * betweenFactorSwitchable.h
 *
 *  Created on: 02.08.2012
 *      Author: niko
 */

#ifndef BETWEENFACTORSWITCHABLE_H_
#define BETWEENFACTORSWITCHABLE_H_

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
using std::cout;
using std::endl;

#include "switchVariableLinear.h"
#include "switchVariableSigmoid.h"


namespace vertigo {

  template<class VALUE>
  class BetweenFactorSwitchableLinear : public gtsam::NoiseModelFactor3<VALUE, VALUE, SwitchVariableLinear>
  {
    public:
      BetweenFactorSwitchableLinear() {};
      BetweenFactorSwitchableLinear(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, const VALUE& measured, const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor3<VALUE, VALUE, SwitchVariableLinear>(model, key1, key2, key3),
        betweenFactor(key1, key2, measured, model) {};

      gtsam::Vector evaluateError(const VALUE& p1, const VALUE& p2, const SwitchVariableLinear& s,
#if GTSAM_VERSION_NUMERIC >= 40300
		  gtsam::OptionalMatrixType H1 = OptionalNone,
		  gtsam::OptionalMatrixType H2 = OptionalNone,
		  gtsam::OptionalMatrixType H3 = OptionalNone) const
#else
          boost::optional<gtsam::Matrix&> H1 = boost::none,
          boost::optional<gtsam::Matrix&> H2 = boost::none,
          boost::optional<gtsam::Matrix&> H3 = boost::none) const
#endif
        {

          // calculate error: f(p1, p2, s) = E_raw(p1, p2) * s
          gtsam::Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);

          // Jacobian w.r.t. the switch tangent: dE/ds = E_raw (the
          // unscaled error). Must be captured BEFORE scaling `error` by
          // s.value() below. Setting H3 to the scaled error (= E_raw*s)
          // was a bug: at small s the switch gradient vanishes
          // quadratically with s, so the optimizer stalls before driving
          // the switch to 0 -- visibly worse outlier rejection than the
          // g2o equivalent (EdgeSE3Switchable, which has the correct
          // constant Jacobian-element).
          if (H3) *H3 = error;
          error *= s.value();

          // handle derivatives
          if (H1) *H1 = *H1 * s.value();
          if (H2) *H2 = *H2 * s.value();

          return error;
        };

    private:
      gtsam::BetweenFactor<VALUE> betweenFactor;

  };



  template<class VALUE>
  class BetweenFactorSwitchableSigmoid : public gtsam::NoiseModelFactor3<VALUE, VALUE, SwitchVariableSigmoid>
  {
    public:
      BetweenFactorSwitchableSigmoid() {};
      BetweenFactorSwitchableSigmoid(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, const VALUE& measured, const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor3<VALUE, VALUE, SwitchVariableSigmoid>(model, key1, key2, key3),
        betweenFactor(key1, key2, measured, model) {};

      gtsam::Vector evaluateError(const VALUE& p1, const VALUE& p2, const SwitchVariableSigmoid& s,
#if GTSAM_VERSION_NUMERIC >= 40300
		  gtsam::OptionalMatrixType H1 = OptionalNone,
		  gtsam::OptionalMatrixType H2 = OptionalNone,
		  gtsam::OptionalMatrixType H3 = OptionalNone) const
#else
          boost::optional<gtsam::Matrix&> H1 = boost::none,
          boost::optional<gtsam::Matrix&> H2 = boost::none,
          boost::optional<gtsam::Matrix&> H3 = boost::none) const
#endif
      {

        // calculate error
        gtsam::Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);


        double w = sigmoid(s.value());
        error *= w;

        // handle derivatives
        if (H1) *H1 = *H1 * w;
        if (H2) *H2 = *H2 * w;
        if (H3) *H3 = error /* (w*(1.0-w))*/;  // sig(x)*(1-sig(x)) is the derivative of sig(x) wrt. x

        return error;
      };

    private:
      gtsam::BetweenFactor<VALUE> betweenFactor;

      double sigmoid(double x) const {
        return 1.0/(1.0+exp(-x));
      }
  };

}

#endif /* BETWEENFACTORSWITCHABLE_H_ */
