/*
 * betweenFactorMaxMix.h
 *
 *  Created on: 14.08.2012
 *      Author: niko
 */

#ifndef BETWEENFACTORMAXMIX_H_
#define BETWEENFACTORMAXMIX_H_

#include <gtsam/linear/NoiseModel.h>
#include <Eigen/Eigen>
#include <gtsam/config.h>

#if GTSAM_VERSION_NUMERIC >= 40100
namespace gtsam {
gtsam::Matrix inverse(const gtsam::Matrix & matrix)
{
	return matrix.inverse();
}
}
#endif

namespace vertigo {

  template<class VALUE>
  class BetweenFactorMaxMix : public gtsam::NoiseModelFactor2<VALUE, VALUE>
  {
    public:
      BetweenFactorMaxMix() : weight(0.0) {};
      BetweenFactorMaxMix(gtsam::Key key1, gtsam::Key key2, const VALUE& measured, const gtsam::SharedNoiseModel& model, const gtsam::SharedNoiseModel& model2, double w)
      : gtsam::NoiseModelFactor2<VALUE, VALUE>(model, key1, key2), weight(w), nullHypothesisModel(model2),
        betweenFactor(key1, key2, measured, model)  {   };

      gtsam::Vector evaluateError(const VALUE& p1, const VALUE& p2,
          boost::optional<gtsam::Matrix&> H1 = boost::none,
          boost::optional<gtsam::Matrix&> H2 =  boost::none) const
        {

          // calculate error
          gtsam::Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);



          // which hypothesis is more likely
          double m1 = this->noiseModel_->distance(error);
          gtsam::noiseModel::Gaussian::shared_ptr g1 = this->noiseModel_;
          gtsam::Matrix info1(g1->R().transpose()*g1->R());
          double nu1 = 1.0/sqrt(gtsam::inverse(info1).determinant());
          double l1 = nu1 * exp(-0.5*m1);

#if GTSAM_VERSION_NUMERIC >= 40100
          double m2 = nullHypothesisModel->squaredMahalanobisDistance(error);
#else
          double m2 = nullHypothesisModel->distance(error);
#endif
          gtsam::noiseModel::Gaussian::shared_ptr g2 = nullHypothesisModel;
          gtsam::Matrix info2(g2->R().transpose()*g2->R());
          double nu2 = 1.0/sqrt(gtsam::inverse(info2).determinant());
          double l2 = nu2 * exp(-0.5*m2);

          // if the null hypothesis is more likely, than proceed by applying the weight ...
          if (l2>l1) {
            if (H1) *H1 = *H1 * weight;
            if (H2) *H2 = *H2 * weight;
            error *= sqrt(weight);
          }

          return error;
        };

    private:
      gtsam::BetweenFactor<VALUE> betweenFactor;
      gtsam::SharedNoiseModel nullHypothesisModel;
      double weight;

  };
}


#endif /* BETWEENFACTORMAXMIX_H_ */
