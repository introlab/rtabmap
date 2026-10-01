// Regression for the sigmoid switch derivative, independent of the core library.
#include <gtsam/config.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include "../../src/optimizer/vertigo/gtsam/betweenFactorSwitchable.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace gtsam;

void near(const Matrix& actual, const Matrix& expected) {
    if (actual.rows() != expected.rows() || actual.cols() != expected.cols() ||
        !actual.allFinite() || (actual - expected).norm() >= 1e-6) {
        std::cerr << "Actual:\n" << actual << "\nExpected:\n" << expected << '\n';
        throw std::runtime_error("Sigmoid switch Jacobian mismatch");
    }
}

template<class Pose>
void checkSigmoid(const Pose& second, double switchValue) {
    const Pose first;
    const vertigo::SwitchVariableSigmoid s(switchValue);
    const auto model = noiseModel::Isotropic::Sigma(traits<Pose>::dimension, 1);
    const vertigo::BetweenFactorSwitchableSigmoid<Pose> factor(1, 2, 3, Pose(), model);
    Matrix h1, h2, h3;
#if GTSAM_VERSION_NUMERIC >= 40300
    const Vector error = factor.evaluateError(first, second, s, &h1, &h2, &h3);
#else
    const Vector error = factor.evaluateError(first, second, s, h1, h2, h3);
#endif
    Matrix rawH1, rawH2;
    const BetweenFactor<Pose> raw(1, 2, Pose(), model);
    const Vector rawError = raw.evaluateError(first, second, rawH1, rawH2);
    const double weight = 1.0 / (1.0 + std::exp(-switchValue));
    near(error, rawError * weight);
    near(h1, rawH1 * weight);
    near(h2, rawH2 * weight);
    near(h3, rawError * (weight * (1.0 - weight)));
    const double step = 1e-5;
    const Vector plus = factor.evaluateError(first, second,
        vertigo::SwitchVariableSigmoid(switchValue + step));
    const Vector minus = factor.evaluateError(first, second,
        vertigo::SwitchVariableSigmoid(switchValue - step));
    near(h3, (plus - minus) / (2.0 * step));
    Values values;
    values.insert(1, first); values.insert(2, second); values.insert(3, s);
    if (!factor.linearize(values)) throw std::runtime_error("Sigmoid linearization failed");
}

int main() {
    try {
        // Interior switch values cover different weights without touching clamps.
        for (double s : {-2.0, 0.0, 2.0}) {
            checkSigmoid(Pose2(1, 2, 0.2), s);
            checkSigmoid(Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1, 2, 3)), s);
        }
        std::cout << "Sigmoid derivative checks passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; return 1;
    }
}
