// Regression coverage for the Vertigo scalar manifold and gravity APIs.
#include <gtsam/config.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include "../../src/optimizer/vertigo/gtsam/betweenFactorSwitchable.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace gtsam;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void near(const Matrix& actual, const Matrix& expected) {
    require(actual.rows() == expected.rows() && actual.cols() == expected.cols(),
            "Jacobian/residual dimensions");
    if (!actual.allFinite() || (actual - expected).norm() >= 1e-6) {
        std::cerr << "Actual:\n" << actual << "\nExpected:\n" << expected << '\n';
        throw std::runtime_error("Jacobian/residual values");
    }
}

template<class Switch>
void checkSwitch(double value, double other) {
    static_assert(traits<Switch>::dimension == 1, "scalar tangent");
    const Switch x(value), y(other);
    Matrix11 h1, h2;
    near(x.localCoordinates(y, h1, h2), Vector1(other - value));
    near(h1, -Matrix11::Identity());
    near(h2, Matrix11::Identity());
    const PriorFactor<Switch> prior(1, y, noiseModel::Isotropic::Sigma(1, 1));
    Matrix h;
    near(prior.evaluateError(x, h), Vector1(value - other));
    near(h, Matrix11::Identity());
    near(h, numericalDerivative11<Vector, Switch>(
        [&prior](const Switch& s) { return prior.evaluateError(s); }, x));
    Values values;
    values.insert(1, x);
    require(bool(prior.linearize(values)), "explicit prior linearization");
#if GTSAM_VERSION_NUMERIC >= 40300
    // Default noise was added in 4.3; 4.2 requires the explicit model above.
    const PriorFactor<Switch> unitPrior(1, y);
    near(unitPrior.evaluateError(x, h), Vector1(value - other));
    near(h, Matrix11::Identity());
    require(bool(unitPrior.linearize(values)), "default prior linearization");
#endif
}

template<class Pose, class Switch, template<class> class Factor>
void checkBetween(const Pose& first, const Pose& second, double switchValue) {
    const Switch s(switchValue);
    const Factor<Pose> factor(1, 2, 3, Pose(),
        noiseModel::Isotropic::Sigma(traits<Pose>::dimension, 1));
    Matrix h1, h2, h3;
#if GTSAM_VERSION_NUMERIC >= 40300
    const Vector error = factor.evaluateError(first, second, s, &h1, &h2, &h3);
#else
    const Vector error = factor.evaluateError(first, second, s, h1, h2, h3);
#endif
    // Older GTSAM defaults to an approximate pose Local Jacobian. Compare
    // against that release's BetweenFactor; switches must scale it faithfully.
    Matrix rawH1, rawH2;
    const BetweenFactor<Pose> raw(1, 2, Pose(),
        noiseModel::Isotropic::Sigma(traits<Pose>::dimension, 1));
    const Vector rawError = raw.evaluateError(first, second, rawH1, rawH2);
    const double weight = error.norm() / rawError.norm();
    near(h1, rawH1 * weight);
    near(h2, rawH2 * weight);
#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
    near(h1, numericalDerivative11<Vector, Pose>(
        [&](const Pose& p) { return factor.evaluateError(p, second, s); }, first));
    near(h2, numericalDerivative11<Vector, Pose>(
        [&](const Pose& p) { return factor.evaluateError(first, p, s); }, second));
#endif
    near(h3, numericalDerivative11<Vector, Switch>(
        [&](const Switch& x) { return factor.evaluateError(first, second, x); }, s));
    require(error.allFinite(), "finite switchable residual");
    Values values;
    values.insert(1, first); values.insert(2, second); values.insert(3, s);
    require(bool(factor.linearize(values)), "switchable linearization");
}

void checkGravity() {
#ifdef RTABMAP_GTSAM_HAS_ATTITUDE_FACTOR_TEMPLATE
    using GravityFactor = AttitudeFactor<Pose3>;
#else
    using GravityFactor = Pose3AttitudeFactor;
#endif
    const GravityFactor gravity(1, Unit3(0,0,1), noiseModel::Isotropic::Sigma(2, 0.1));
    const Pose3 initial(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(1,2,3));
    Matrix h;
    const Vector error = gravity.evaluateError(initial, h);
    near(h, numericalDerivative11<Vector, Pose3>(
        [&](const Pose3& p) { return gravity.evaluateError(p); }, initial));
    near(gravity.evaluateError(Pose3()), Vector2::Zero());
    NonlinearFactorGraph graph;
    graph.add(gravity);
    // A loose pose prior anchors yaw and translation while gravity fixes tilt.
    graph.add(PriorFactor<Pose3>(1, Pose3(), noiseModel::Isotropic::Sigma(6, 1)));
    Values values; values.insert(1, initial);
    const Values result = LevenbergMarquardtOptimizer(graph, values).optimize();
    require(gravity.evaluateError(result.at<Pose3>(1)).norm() < error.norm()*0.01,
            "gravity optimization reduces tilt");
}

int main() {
    try {
        checkSwitch<vertigo::SwitchVariableLinear>(0.4, 0.7);
        checkSwitch<vertigo::SwitchVariableSigmoid>(-0.4, 0.7);
        // Preserve projection at boundaries and the constructor conventions.
        near(Vector1(vertigo::SwitchVariableLinear(0.4).retract(Vector1(2)).value()), Vector1(1));
        near(Vector1(vertigo::SwitchVariableLinear(0.4).retract(Vector1(-2)).value()), Vector1(0));
        near(Vector1(vertigo::SwitchVariableLinear(2).value()), Vector1(2));
        near(Vector1(vertigo::SwitchVariableSigmoid(0).retract(Vector1(20)).value()), Vector1(10));
        near(Vector1(vertigo::SwitchVariableSigmoid(0).retract(Vector1(-20)).value()), Vector1(-10));
        near(Vector1(vertigo::SwitchVariableSigmoid(20).value()), Vector1(10));
        near(Vector1(vertigo::SwitchVariableSigmoid(-20).value()), Vector1(-10));
        checkBetween<Pose2, vertigo::SwitchVariableLinear, vertigo::BetweenFactorSwitchableLinear>(Pose2(), Pose2(1,2,0.2), 0.4);
        checkBetween<Pose3, vertigo::SwitchVariableLinear, vertigo::BetweenFactorSwitchableLinear>(Pose3(), Pose3(Rot3::RzRyRx(0.1,0.2,0.3), Point3(1,2,3)), 0.4);
        checkBetween<Pose2, vertigo::SwitchVariableSigmoid, vertigo::BetweenFactorSwitchableSigmoid>(Pose2(), Pose2(1,2,0.2), -0.4);
        checkBetween<Pose3, vertigo::SwitchVariableSigmoid, vertigo::BetweenFactorSwitchableSigmoid>(Pose3(), Pose3(Rot3::RzRyRx(0.1,0.2,0.3), Point3(1,2,3)), -0.4);
        checkGravity();
        std::cout << "GTSAM compatibility checks passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; return 1;
    }
}
