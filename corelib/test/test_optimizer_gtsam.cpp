// Tests for the GTSAM pieces used by rtabmap::OptimizerGTSAM, independent of
// the optimizer itself:
//   - Vertigo switch variables (linear and sigmoid): scalar manifold traits,
//     1x1 Local Jacobians, priors, and constructor/retract clamping.
//   - Switchable between factors: residuals and Jacobians against the
//     release's BetweenFactor, the switch derivative against finite
//     differences, and linearization.
//   - Gravity (attitude) factor API selected by CMake
//     (RTABMAP_GTSAM_HAS_ATTITUDE_FACTOR_TEMPLATE).
//
// Background: Optimizer/Robust=true adds, for every loop closure, a switch
// variable s_ij with a prior, and replaces the loop closure's BetweenFactor
// by a switchable one whose residual is weighted by s_ij (linear switch) or
// sigmoid(s_ij) (sigmoid switch). The optimizer can then turn off outlier
// loop closures by driving their weight to 0. These factors live in
// corelib/src/optimizer/vertigo/gtsam and depend on GTSAM internals (traits,
// OptionalJacobian, PriorFactor), which changed across 4.0/4.2/4.3, hence the
// focused checks below. They are meant to pass on every GTSAM version
// rtabmap supports.

#include <gtest/gtest.h>
#include <gtsam/config.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include "../src/optimizer/vertigo/gtsam/betweenFactorSwitchable.h"

#include <cmath>
#include <sstream>

using namespace gtsam;

namespace {

// Compares matrices (or vectors) with the same tolerance everywhere. A size
// mismatch is reported separately: a Jacobian of the wrong size is exactly the
// kind of regression these tests target (e.g., the 3x3 Jacobians the switch
// traits used to declare for a 1-D variable, which made JacobianFactor throw
// InvalidMatrixBlock during linearization).
::testing::AssertionResult near(const Matrix& actual, const Matrix& expected)
{
	if(actual.rows() != expected.rows() || actual.cols() != expected.cols())
	{
		return ::testing::AssertionFailure() << "dimensions " << actual.rows() << "x" << actual.cols()
				<< " instead of " << expected.rows() << "x" << expected.cols();
	}
	if(!actual.allFinite() || (actual - expected).norm() >= 1e-6)
	{
		std::stringstream ss;
		ss << "Actual:\n" << actual << "\nExpected:\n" << expected;
		return ::testing::AssertionFailure() << ss.str();
	}
	return ::testing::AssertionSuccess();
}

// Checks that a switch variable behaves as a 1-D manifold for GTSAM:
//  1. traits<Switch>::dimension is 1 (required by noiseModel::Unit::Create()
//     and fixed-size code paths in GTSAM >= 4.3).
//  2. localCoordinates(y) = y - x, with analytical Jacobians -1 (wrt x) and
//     +1 (wrt y).
//  3. A PriorFactor on the switch (what OptimizerGTSAM adds for every switch)
//     gives a scalar residual x - prior and a 1x1 identity Jacobian, matching
//     a numerical derivative, and can be linearized. Depending on the GTSAM
//     version/configuration (GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR), the prior
//     takes its Jacobian from traits<Switch>::Local(), so this is where
//     incomplete traits show up.
//  4. On GTSAM >= 4.3, the same with a prior built without a noise model,
//     which goes through noiseModel::Unit::Create(value) and so needs the
//     full manifold traits (dimension, structure_category, ManifoldType).
template<class Switch>
void checkSwitch(double value, double other)
{
	static_assert(traits<Switch>::dimension == 1, "scalar tangent");
	const Switch x(value), y(other);

	// Chart and its Jacobians
	Matrix11 h1, h2;
	EXPECT_TRUE(near(x.localCoordinates(y, h1, h2), Vector1(other - value)));
	EXPECT_TRUE(near(h1, -Matrix11::Identity()));
	EXPECT_TRUE(near(h2, Matrix11::Identity()));

	// Prior with an explicit noise model, as created by OptimizerGTSAM
	const PriorFactor<Switch> prior(1, y, noiseModel::Isotropic::Sigma(1, 1));
	Matrix h;
	EXPECT_TRUE(near(prior.evaluateError(x, h), Vector1(value - other)));
	EXPECT_TRUE(near(h, Matrix11::Identity()));
	EXPECT_TRUE(near(h, numericalDerivative11<Vector, Switch>(
		[&prior](const Switch& s) { return prior.evaluateError(s); }, x)));
	Values values;
	values.insert(1, x);
	EXPECT_TRUE(bool(prior.linearize(values))) << "explicit prior linearization";

#if GTSAM_VERSION_NUMERIC >= 40300
	// Prior with the default (unit) noise model. Default noise was added in
	// 4.3; 4.2 requires the explicit model above.
	const PriorFactor<Switch> unitPrior(1, y);
	EXPECT_TRUE(near(unitPrior.evaluateError(x, h), Vector1(value - other)));
	EXPECT_TRUE(near(h, Matrix11::Identity()));
	EXPECT_TRUE(bool(unitPrior.linearize(values))) << "default prior linearization";
#endif
}

// Checks BetweenFactorSwitchableLinear, the robust loop closure factor used
// by OptimizerGTSAM: residual = s * BetweenFactor residual.
//  - The pose Jacobians must be the regular BetweenFactor Jacobians scaled by
//    the switch weight. They are compared against the BetweenFactor of the
//    installed GTSAM rather than against numerical derivatives, because older
//    GTSAM (without GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR) uses an approximate
//    Local Jacobian for poses. When the exact one is enabled, they are also
//    compared against numerical derivatives.
//  - The switch Jacobian (d residual / d s = raw residual) is always compared
//    against a numerical derivative.
//  - The factor, with the two poses and the switch in a Values, must
//    linearize (which checks that all Jacobian sizes agree with the residual).
template<class Pose>
void checkBetweenLinear(const Pose& first, const Pose& second, double switchValue)
{
	const vertigo::SwitchVariableLinear s(switchValue);
	const auto model = noiseModel::Isotropic::Sigma(traits<Pose>::dimension, 1);
	const vertigo::BetweenFactorSwitchableLinear<Pose> factor(1, 2, 3, Pose(), model);
	Matrix h1, h2, h3;
#if GTSAM_VERSION_NUMERIC >= 40300
	const Vector error = factor.evaluateError(first, second, s, &h1, &h2, &h3);
#else
	const Vector error = factor.evaluateError(first, second, s, h1, h2, h3);
#endif
	// Same measurement without switch: its residual ratio gives the weight
	// actually applied, which must also scale the pose Jacobians.
	Matrix rawH1, rawH2;
	const BetweenFactor<Pose> raw(1, 2, Pose(), model);
	const Vector rawError = raw.evaluateError(first, second, rawH1, rawH2);
	const double weight = error.norm() / rawError.norm();
	EXPECT_TRUE(near(h1, rawH1 * weight));
	EXPECT_TRUE(near(h2, rawH2 * weight));
#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
	EXPECT_TRUE(near(h1, numericalDerivative11<Vector, Pose>(
		[&](const Pose& p) { return factor.evaluateError(p, second, s); }, first)));
	EXPECT_TRUE(near(h2, numericalDerivative11<Vector, Pose>(
		[&](const Pose& p) { return factor.evaluateError(first, p, s); }, second)));
#endif
	EXPECT_TRUE(near(h3, numericalDerivative11<Vector, vertigo::SwitchVariableLinear>(
		[&](const vertigo::SwitchVariableLinear& x) { return factor.evaluateError(first, second, x); }, s)));
	EXPECT_TRUE(error.allFinite());

	Values values;
	values.insert(1, first); values.insert(2, second); values.insert(3, s);
	EXPECT_TRUE(bool(factor.linearize(values))) << "switchable linearization";
}

// Checks BetweenFactorSwitchableSigmoid: residual = w * BetweenFactor
// residual, with w = sigmoid(s) = 1/(1+exp(-s)).
//  - Residual and pose Jacobians must be the BetweenFactor ones scaled by w.
//  - The switch Jacobian must be raw residual * dw/ds = raw * w*(1-w), both
//    analytically and by central finite differences. This catches the bug
//    where it returned the weighted residual (raw * w), missing the (1-w)
//    factor.
//  - The factor must linearize.
template<class Pose>
void checkBetweenSigmoid(const Pose& second, double switchValue)
{
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
	EXPECT_TRUE(near(error, rawError * weight));
	EXPECT_TRUE(near(h1, rawH1 * weight));
	EXPECT_TRUE(near(h2, rawH2 * weight));
	// Analytical: d(sigmoid)/ds = w*(1-w)
	EXPECT_TRUE(near(h3, rawError * (weight * (1.0 - weight))));
	// Numerical: central difference over the switch value. The switch is
	// re-constructed rather than retracted, as retract() clamps it.
	const double step = 1e-5;
	const Vector plus = factor.evaluateError(first, second,
		vertigo::SwitchVariableSigmoid(switchValue + step));
	const Vector minus = factor.evaluateError(first, second,
		vertigo::SwitchVariableSigmoid(switchValue - step));
	EXPECT_TRUE(near(h3, (plus - minus) / (2.0 * step)));

	Values values;
	values.insert(1, first); values.insert(2, second); values.insert(3, s);
	EXPECT_TRUE(bool(factor.linearize(values))) << "sigmoid linearization";
}

} // namespace

// Linear switch (the one OptimizerGTSAM uses) as a 1-D GTSAM manifold.
TEST(OptimizerGTSAM, SwitchVariableLinearManifold)
{
	checkSwitch<vertigo::SwitchVariableLinear>(0.4, 0.7);
}

// Sigmoid switch as a 1-D GTSAM manifold (negative values are valid for it).
TEST(OptimizerGTSAM, SwitchVariableSigmoidManifold)
{
	checkSwitch<vertigo::SwitchVariableSigmoid>(-0.4, 0.7);
}

// Switch bounds: retract() (applied at each optimization step) projects the
// linear switch to [0,1] and the sigmoid switch to [-10,10]. The constructor
// doesn't clamp the linear switch, but does clamp the sigmoid one. Moving the
// traits to internal::Manifold must not change this behavior.
TEST(OptimizerGTSAM, SwitchVariableClamping)
{
	EXPECT_DOUBLE_EQ(vertigo::SwitchVariableLinear(0.4).retract(Vector1(2)).value(), 1.0);
	EXPECT_DOUBLE_EQ(vertigo::SwitchVariableLinear(0.4).retract(Vector1(-2)).value(), 0.0);
	EXPECT_DOUBLE_EQ(vertigo::SwitchVariableLinear(2).value(), 2.0);
	EXPECT_DOUBLE_EQ(vertigo::SwitchVariableSigmoid(0).retract(Vector1(20)).value(), 10.0);
	EXPECT_DOUBLE_EQ(vertigo::SwitchVariableSigmoid(0).retract(Vector1(-20)).value(), -10.0);
	EXPECT_DOUBLE_EQ(vertigo::SwitchVariableSigmoid(20).value(), 10.0);
	EXPECT_DOUBLE_EQ(vertigo::SwitchVariableSigmoid(-20).value(), -10.0);
}

// Robust loop closure factor with a linear switch, for 2D (Optimizer/Slam2D)
// and 3D graphs, with a partially-on switch (0.4).
TEST(OptimizerGTSAM, BetweenFactorSwitchableLinear)
{
	checkBetweenLinear(Pose2(), Pose2(1, 2, 0.2), 0.4);
	checkBetweenLinear(Pose3(), Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1, 2, 3)), 0.4);
}

// Robust loop closure factor with a sigmoid switch, for 2D and 3D graphs.
// Switch values -2, 0 and 2 give weights ~0.12, 0.5 and ~0.88, staying away
// from the [-10,10] clamps.
TEST(OptimizerGTSAM, BetweenFactorSwitchableSigmoid)
{
	for(double s : {-2.0, 0.0, 2.0})
	{
		SCOPED_TRACE(s);
		checkBetweenSigmoid(Pose2(1, 2, 0.2), s);
		checkBetweenSigmoid(Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1, 2, 3)), s);
	}
}

// Gravity constraints (Link::kGravity, Optimizer/GravitySigma > 0) are added
// as a Pose3 attitude factor. Its class changed name in GTSAM 4.3
// (Pose3AttitudeFactor -> AttitudeFactor<Pose3>), and some ROS 4.3 snapshots
// report the same version number with either API, so CMake detects which one
// compiles. This checks the detected API is usable as OptimizerGTSAM uses it:
//  - Jacobian matches a numerical derivative on a tilted pose.
//  - Residual is zero when the pose is aligned with gravity.
//  - Optimizing a tilted pose (with a loose prior to fix the yaw and the
//    translation, which gravity doesn't observe) removes the tilt.
TEST(OptimizerGTSAM, GravityFactor)
{
#ifdef RTABMAP_GTSAM_HAS_ATTITUDE_FACTOR_TEMPLATE
	using GravityFactor = AttitudeFactor<Pose3>;
#else
	using GravityFactor = Pose3AttitudeFactor;
#endif
	// Reference direction: world z axis; measured in body frame: also z (pose
	// should be level).
	const GravityFactor gravity(1, Unit3(0,0,1), noiseModel::Isotropic::Sigma(2, 0.1));
	const Pose3 initial(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(1, 2, 3));
	Matrix h;
	const Vector error = gravity.evaluateError(initial, h);
	EXPECT_TRUE(near(h, numericalDerivative11<Vector, Pose3>(
		[&](const Pose3& p) { return gravity.evaluateError(p); }, initial)));
	EXPECT_TRUE(near(gravity.evaluateError(Pose3()), Vector2::Zero()));

	NonlinearFactorGraph graph;
	graph.add(gravity);
	graph.add(PriorFactor<Pose3>(1, Pose3(), noiseModel::Isotropic::Sigma(6, 1)));
	Values values;
	values.insert(1, initial);
	const Values result = LevenbergMarquardtOptimizer(graph, values).optimize();
	EXPECT_LT(gravity.evaluateError(result.at<Pose3>(1)).norm(), error.norm()*0.01)
		<< "gravity optimization reduces tilt";
}
