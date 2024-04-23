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
		  OptionalMatrixType H1 = OptionalNone,
		  OptionalMatrixType H2 = OptionalNone,
		  OptionalMatrixType H3 = OptionalNone) const
#else
          boost::optional<gtsam::Matrix&> H1 = boost::none,
          boost::optional<gtsam::Matrix&> H2 = boost::none,
          boost::optional<gtsam::Matrix&> H3 = boost::none) const
#endif
        {

          // calculate error
          gtsam::Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);
          error *= s.value();

          // handle derivatives
          if (H1) *H1 = *H1 * s.value();
          if (H2) *H2 = *H2 * s.value();
          if (H3) *H3 = error;

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
		  OptionalMatrixType H1 = OptionalNone,
		  OptionalMatrixType H2 = OptionalNone,
		  OptionalMatrixType H3 = OptionalNone) const
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
