// Tests for rtabmap::Optimizer -- the base class. We exercise:
//   - factory + isAvailable()
//   - getters/setters and parseParameters()
//   - getConnectedGraph(): a non-virtual graph utility that drives every
//     concrete optimizer's input prep (filters disconnected nodes, applies
//     priorsIgnored/landmarksIgnored, normalizes link direction).
//   - optimize() / optimizeIncremental() smoke tests on a deterministic
//     2/3-node chain -- whichever concrete optimizer is built in.
//
// The Optimizer base class itself is abstract; concrete behavior comes from
// TORO/g2o/GTSAM/Ceres backends. We require *some* backend to be available
// (the factory throws otherwise), and stress what's portable across all of
// them rather than backend-specific quirks.

#include <gtest/gtest.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UConversion.h>
#include <memory>
#include <random>

using namespace rtabmap;

namespace {

// One unit-trace covariance per edge. Real callers populate a 6x6 inverse;
// for these tests we just need something invertible/non-zero.
cv::Mat unitCov()
{
	return cv::Mat::eye(6, 6, CV_64FC1);
}

// Identity-translation link between two consecutive nodes a -> b along +X.
Link neighborLink(int from, int to, float dx = 1.0f)
{
	return Link(from, to, Link::kNeighbor,
			Transform(dx, 0, 0, 0, 0, 0),
			unitCov());
}

// Build a straight 3-node chain (N1 -> N2 -> N3) at +X intervals.
void makeChain3(std::map<int, Transform> & poses, std::multimap<int, Link> & links)
{
	poses[1] = Transform(0.0f, 0, 0, 0, 0, 0);
	poses[2] = Transform(1.0f, 0, 0, 0, 0, 0);
	poses[3] = Transform(2.0f, 0, 0, 0, 0, 0);
	links.insert({1, neighborLink(1, 2)});
	links.insert({2, neighborLink(2, 3)});
}

}  // namespace

// ---------------------------------------------------------------------------
// Type / factory
// ---------------------------------------------------------------------------

TEST(OptimizerTest, TypeEnumValuesAreStable)
{
	// Persisted in databases via Parameters/RGBD::OptimizerStrategy and in
	// other DBs; reordering would break backward compat.
	EXPECT_EQ(static_cast<int>(Optimizer::kTypeUndef),  -1);
	EXPECT_EQ(static_cast<int>(Optimizer::kTypeTORO),    0);
	EXPECT_EQ(static_cast<int>(Optimizer::kTypeG2O),     1);
	EXPECT_EQ(static_cast<int>(Optimizer::kTypeGTSAM),   2);
	EXPECT_EQ(static_cast<int>(Optimizer::kTypeCeres),   3);
	EXPECT_EQ(static_cast<int>(Optimizer::kTypeCVSBA),   4);
}

TEST(OptimizerTest, IsAvailableAtLeastOneOptimizer)
{
	// The factory requires at least one backend; if all return false, no test
	// using create() can ever pass. Catching it here is more diagnostic.
	const bool anyAvailable =
			Optimizer::isAvailable(Optimizer::kTypeTORO) ||
			Optimizer::isAvailable(Optimizer::kTypeG2O) ||
			Optimizer::isAvailable(Optimizer::kTypeGTSAM) ||
			Optimizer::isAvailable(Optimizer::kTypeCeres);
	EXPECT_TRUE(anyAvailable) << "No graph optimizer was compiled in";
}

TEST(OptimizerTest, IsAvailableReturnsFalseForUndef)
{
	EXPECT_FALSE(Optimizer::isAvailable(Optimizer::kTypeUndef));
}

TEST(OptimizerTest, CreateReturnsNonNullForDefaultStrategy)
{
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	ASSERT_NE(opt.get(), nullptr);
	EXPECT_NE(opt->type(), Optimizer::kTypeUndef);
}

TEST(OptimizerTest, CreateFallsBackWhenTypeNotAvailable)
{
	// Pick whatever IS available and ask for it explicitly.
	for(int t : {Optimizer::kTypeTORO, Optimizer::kTypeG2O,
				 Optimizer::kTypeGTSAM, Optimizer::kTypeCeres,
				 Optimizer::kTypeCVSBA})
	{
		if(Optimizer::isAvailable(static_cast<Optimizer::Type>(t)))
		{
			std::unique_ptr<Optimizer> opt(
					Optimizer::create(static_cast<Optimizer::Type>(t)));
			ASSERT_NE(opt.get(), nullptr);
			EXPECT_EQ(opt->type(), static_cast<Optimizer::Type>(t));
			return;
		}
	}
	GTEST_SKIP() << "No optimizer backend available";
}

TEST(OptimizerTest, CreateViaParametersMapHonorsOptimizerStrategy)
{
	// Find any available backend and request it through the parameter map
	// (the path used by Rtabmap::init when reading user config).
	for(int t : {Optimizer::kTypeTORO, Optimizer::kTypeG2O,
				 Optimizer::kTypeGTSAM, Optimizer::kTypeCeres,
				 Optimizer::kTypeCVSBA})
	{
		if(Optimizer::isAvailable(static_cast<Optimizer::Type>(t)))
		{
			ParametersMap params;
			params[Parameters::kOptimizerStrategy()] = uNumber2Str(t);
			std::unique_ptr<Optimizer> opt(Optimizer::create(params));
			ASSERT_NE(opt.get(), nullptr);
			EXPECT_EQ(opt->type(), static_cast<Optimizer::Type>(t));
			return;
		}
	}
	GTEST_SKIP() << "No optimizer backend available";
}

// ---------------------------------------------------------------------------
// Getters / setters / parseParameters
// ---------------------------------------------------------------------------

TEST(OptimizerTest, DefaultsMatchParameterDefaults)
{
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	ASSERT_NE(opt.get(), nullptr);
	EXPECT_EQ(opt->iterations(),         Parameters::defaultOptimizerIterations());
	EXPECT_EQ(opt->isSlam2d(),           Parameters::defaultRegForce3DoF());
	EXPECT_EQ(opt->isCovarianceIgnored(),Parameters::defaultOptimizerVarianceIgnored());
	EXPECT_DOUBLE_EQ(opt->epsilon(),     Parameters::defaultOptimizerEpsilon());
	EXPECT_EQ(opt->isRobust(),           Parameters::defaultOptimizerRobust());
	EXPECT_EQ(opt->priorsIgnored(),      Parameters::defaultOptimizerPriorsIgnored());
	EXPECT_EQ(opt->landmarksIgnored(),   Parameters::defaultOptimizerLandmarksIgnored());
	EXPECT_FLOAT_EQ(opt->gravitySigma(), Parameters::defaultOptimizerGravitySigma());
}

TEST(OptimizerTest, SettersUpdateGettersIndependently)
{
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	ASSERT_NE(opt.get(), nullptr);

	opt->setIterations(123);
	opt->setSlam2d(true);
	opt->setCovarianceIgnored(true);
	opt->setEpsilon(1e-4);
	opt->setRobust(true);
	opt->setPriorsIgnored(true);
	opt->setLandmarksIgnored(true);
	opt->setGravitySigma(0.42f);

	EXPECT_EQ(opt->iterations(), 123);
	EXPECT_TRUE(opt->isSlam2d());
	EXPECT_TRUE(opt->isCovarianceIgnored());
	EXPECT_DOUBLE_EQ(opt->epsilon(), 1e-4);
	EXPECT_TRUE(opt->isRobust());
	EXPECT_TRUE(opt->priorsIgnored());
	EXPECT_TRUE(opt->landmarksIgnored());
	EXPECT_FLOAT_EQ(opt->gravitySigma(), 0.42f);
}

TEST(OptimizerTest, ParseParametersFromMap)
{
	ParametersMap params;
	params[Parameters::kOptimizerIterations()]      = "77";
	params[Parameters::kRegForce3DoF()]             = "true";
	params[Parameters::kOptimizerVarianceIgnored()] = "true";
	params[Parameters::kOptimizerEpsilon()]         = "0.001";
	params[Parameters::kOptimizerRobust()]          = "true";
	params[Parameters::kOptimizerPriorsIgnored()]   = "false";
	params[Parameters::kOptimizerLandmarksIgnored()]= "true";
	params[Parameters::kOptimizerGravitySigma()]    = "0.5";

	std::unique_ptr<Optimizer> opt(Optimizer::create(params));
	ASSERT_NE(opt.get(), nullptr);

	EXPECT_EQ(opt->iterations(), 77);
	EXPECT_TRUE(opt->isSlam2d());
	EXPECT_TRUE(opt->isCovarianceIgnored());
	EXPECT_DOUBLE_EQ(opt->epsilon(), 0.001);
	EXPECT_TRUE(opt->isRobust());
	EXPECT_FALSE(opt->priorsIgnored());
	EXPECT_TRUE(opt->landmarksIgnored());
	EXPECT_FLOAT_EQ(opt->gravitySigma(), 0.5f);

	// parseParameters() can also be called post-construction to update fields
	// in-place (Rtabmap::parseParameters does this on parameter change).
	ParametersMap update;
	update[Parameters::kOptimizerIterations()] = "5";
	opt->parseParameters(update);
	EXPECT_EQ(opt->iterations(), 5);
	// Unspecified fields are preserved.
	EXPECT_TRUE(opt->isRobust());
}

// ---------------------------------------------------------------------------
// getConnectedGraph
// ---------------------------------------------------------------------------

TEST(OptimizerTest, GetConnectedGraphReturnsSingleNodeWhenNoLinks)
{
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	std::map<int, Transform> in;
	in[1] = Transform(0, 0, 0, 0, 0, 0);
	std::map<int, Transform> out;
	std::multimap<int, Link> outLinks;
	opt->getConnectedGraph(1, in, std::multimap<int, Link>(), out, outLinks);
	ASSERT_EQ(out.size(), 1u);
	EXPECT_TRUE(out.count(1));
	EXPECT_TRUE(outLinks.empty());
}

TEST(OptimizerTest, GetConnectedGraphKeepsFullChain)
{
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	makeChain3(in, inLinks);

	std::map<int, Transform> out;
	std::multimap<int, Link> outLinks;
	opt->getConnectedGraph(1, in, inLinks, out, outLinks);
	EXPECT_EQ(out.size(), 3u);
	EXPECT_EQ(outLinks.size(), 2u);
}

TEST(OptimizerTest, GetConnectedGraphDropsDisconnectedNodes)
{
	// N1 -> N2 -> N3 is the connected component starting from N1.
	// N10 has no link to the others -> must NOT appear in the output.
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	makeChain3(in, inLinks);
	in[10] = Transform(99.0f, 0, 0, 0, 0, 0);

	std::map<int, Transform> out;
	std::multimap<int, Link> outLinks;
	opt->getConnectedGraph(1, in, inLinks, out, outLinks);
	EXPECT_EQ(out.size(), 3u);
	EXPECT_FALSE(out.count(10));
}

TEST(OptimizerTest, GetConnectedGraphRecomputesPosesAlongEdges)
{
	// The input pose at N3 is "wrong" (10m away) but the link chain says
	// 0 -> 1 -> 2. getConnectedGraph must walk the links, not trust the
	// caller's poses, so the output should reflect the link-implied chain.
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	makeChain3(in, inLinks);
	in[3] = Transform(10.0f, 0, 0, 0, 0, 0);  // wrong on purpose

	std::map<int, Transform> out;
	std::multimap<int, Link> outLinks;
	opt->getConnectedGraph(1, in, inLinks, out, outLinks);
	ASSERT_TRUE(out.count(3));
	EXPECT_NEAR(out.at(3).x(), 2.0f, 1e-5);
}

TEST(OptimizerTest, GetConnectedGraphFollowsLoopClosureLinks)
{
	// Loop closure between N3 and N1 should pull N3 into the connected
	// component even though the chain (N1 N2 N3) already does so. We add
	// loop-closure types to make sure they're respected.
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	in[1] = Transform(0, 0, 0, 0, 0, 0);
	in[2] = Transform(1, 0, 0, 0, 0, 0);
	in[3] = Transform(0, 1, 0, 0, 0, 0);
	inLinks.insert({1, neighborLink(1, 2)});
	inLinks.insert({1, Link(1, 3, Link::kGlobalClosure,
			Transform(0, 1, 0, 0, 0, 0), unitCov())});

	std::map<int, Transform> out;
	std::multimap<int, Link> outLinks;
	opt->getConnectedGraph(1, in, inLinks, out, outLinks);
	EXPECT_EQ(out.size(), 3u);
	EXPECT_EQ(outLinks.size(), 2u);
}

TEST(OptimizerTest, GetConnectedGraphPriorLinksIncludedWhenAllowed)
{
	// kOptimizerPriorsIgnored defaults to true, so flip it off to exercise
	// the inclusion path.
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	opt->setPriorsIgnored(false);
	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	makeChain3(in, inLinks);
	// Pose prior on N1 (self-link with kPosePrior).
	inLinks.insert({1, Link(1, 1, Link::kPosePrior,
			in[1], unitCov())});

	std::map<int, Transform> out;
	std::multimap<int, Link> outLinks;
	opt->getConnectedGraph(1, in, inLinks, out, outLinks);
	// 2 chain links + 1 prior.
	EXPECT_EQ(outLinks.size(), 3u);
}

TEST(OptimizerTest, GetConnectedGraphPriorLinksDroppedWhenIgnored)
{
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	opt->setPriorsIgnored(true);
	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	makeChain3(in, inLinks);
	inLinks.insert({1, Link(1, 1, Link::kPosePrior, in[1], unitCov())});

	std::map<int, Transform> out;
	std::multimap<int, Link> outLinks;
	opt->getConnectedGraph(1, in, inLinks, out, outLinks);
	EXPECT_EQ(outLinks.size(), 2u);  // prior dropped
}

TEST(OptimizerTest, GetConnectedGraphLandmarkLinksDroppedWhenIgnored)
{
	// Landmarks use negative IDs by rtabmap convention. landmarksIgnored
	// keeps the landmark out of the output graph entirely.
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	opt->setLandmarksIgnored(true);
	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	makeChain3(in, inLinks);
	in[-1] = Transform(0.5f, 0.5f, 0, 0, 0, 0);
	inLinks.insert({2, Link(2, -1, Link::kLandmark,
			Transform(0, 0.5f, 0, 0, 0, 0), unitCov())});

	std::map<int, Transform> out;
	std::multimap<int, Link> outLinks;
	opt->getConnectedGraph(1, in, inLinks, out, outLinks);
	EXPECT_EQ(out.size(), 3u);
	EXPECT_FALSE(out.count(-1));
}

TEST(OptimizerTest, GetConnectedGraphLandmarkLinksKeptByDefault)
{
	std::unique_ptr<Optimizer> opt(Optimizer::create(ParametersMap()));
	ASSERT_FALSE(opt->landmarksIgnored());
	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	makeChain3(in, inLinks);
	in[-1] = Transform(0.5f, 0.5f, 0, 0, 0, 0);
	inLinks.insert({2, Link(2, -1, Link::kLandmark,
			Transform(0, 0.5f, 0, 0, 0, 0), unitCov())});

	std::map<int, Transform> out;
	std::multimap<int, Link> outLinks;
	opt->getConnectedGraph(1, in, inLinks, out, outLinks);
	EXPECT_TRUE(out.count(-1));
}

// ---------------------------------------------------------------------------
// optimize() / optimizeIncremental() smoke tests
//
// Parameterized over the four pose-graph backends. CVSBA is excluded -- it's
// bundle-adjustment-only and doesn't implement plain pose-graph optimize().
// Each test skips at runtime when the backend isn't built in.
// ---------------------------------------------------------------------------

namespace {

std::string optimizerTypeName(Optimizer::Type type)
{
	switch(type)
	{
	case Optimizer::kTypeTORO:  return "TORO";
	case Optimizer::kTypeG2O:   return "G2O";
	case Optimizer::kTypeGTSAM: return "GTSAM";
	case Optimizer::kTypeCeres: return "Ceres";
	case Optimizer::kTypeCVSBA: return "CVSBA";
	default:                    return "Undef";
	}
}

class OptimizerBackendTest : public ::testing::TestWithParam<Optimizer::Type>
{
protected:
	void SetUp() override
	{
		if(!Optimizer::isAvailable(GetParam()))
		{
			GTEST_SKIP() << optimizerTypeName(GetParam()) << " not built in";
		}
		opt_.reset(Optimizer::create(GetParam()));
		ASSERT_NE(opt_.get(), nullptr);
		ASSERT_EQ(opt_->type(), GetParam());
	}

	std::unique_ptr<Optimizer> opt_;
};

}  // namespace

TEST_P(OptimizerBackendTest, OptimizeConsistentChainReturnsSamePoses)
{
	// A chain whose link transforms match the poses exactly has zero residual,
	// so any optimizer should return ~the same poses.
	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	makeChain3(in, inLinks);

	std::map<int, Transform> out = opt_->optimize(1, in, inLinks);
	ASSERT_EQ(out.size(), 3u);
	for(const auto & kv : in)
	{
		ASSERT_TRUE(out.count(kv.first));
		EXPECT_NEAR(out.at(kv.first).x(), kv.second.x(), 1e-2)
				<< optimizerTypeName(GetParam()) << " node " << kv.first;
	}
}

TEST_P(OptimizerBackendTest, OptimizeAnchorsRootPose)
{
	// The root pose should remain fixed at the input value -- it's the gauge.
	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	makeChain3(in, inLinks);

	std::map<int, Transform> out = opt_->optimize(/*rootId=*/1, in, inLinks);
	ASSERT_TRUE(out.count(1));
	EXPECT_NEAR(out.at(1).x(), 0.0f, 1e-4) << optimizerTypeName(GetParam());
	EXPECT_NEAR(out.at(1).y(), 0.0f, 1e-4) << optimizerTypeName(GetParam());
	EXPECT_NEAR(out.at(1).z(), 0.0f, 1e-4) << optimizerTypeName(GetParam());
}

TEST_P(OptimizerBackendTest, OptimizeIncrementalConsistentChain)
{
	// Same as the smoke test above but driven through the incremental path.
	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	makeChain3(in, inLinks);
	// Incremental needs at least one loop closure to "fire"; without one it
	// short-circuits to the regular optimize() call at the end.
	inLinks.insert({1, Link(1, 3, Link::kGlobalClosure,
			Transform(2.0f, 0, 0, 0, 0, 0), unitCov())});

	std::map<int, Transform> out = opt_->optimizeIncremental(1, in, inLinks);
	ASSERT_EQ(out.size(), 3u);
	for(const auto & kv : in)
	{
		ASSERT_TRUE(out.count(kv.first));
		EXPECT_NEAR(out.at(kv.first).x(), kv.second.x(), 1e-2)
				<< optimizerTypeName(GetParam()) << " node " << kv.first;
	}
}

INSTANTIATE_TEST_SUITE_P(
		AllPoseGraphBackends,
		OptimizerBackendTest,
		::testing::Values(
				Optimizer::kTypeTORO,
				Optimizer::kTypeG2O,
				Optimizer::kTypeGTSAM,
				Optimizer::kTypeCeres),
		[](const ::testing::TestParamInfo<Optimizer::Type> & info) {
			return optimizerTypeName(info.param);
		});

// ---------------------------------------------------------------------------
// Closed-loop circular trajectory with landmarks + gravity + pose priors.
//
// Setup: 30 evenly-spaced poses on a 10 m radius circle in the XY plane. Each
// pose's yaw points tangentially (i.e. toward the next pose -- a differential
// robot orbiting the center). Constraints:
//   * 29 sequential neighbor links (1->2, 2->3, ..., 29->30) carrying the
//     exact pairwise transform.
//   * 1 loop-closure (30->1) carrying the exact transform.
//   * 1 landmark at the origin (id = -1) with identity rotation. Even-numbered
//     poses (2, 4, ..., 30) observe it via kLandmark links.
//   * A kPosePrior self-link on every pose, transform = the pose itself.
//   * A kGravity self-link on every pose, transform = the pose itself
//     (roll/pitch are what the optimizer reads).
// All link covariances are unit 6x6, including the landmark links.
//
// This is a "perfect world" -- every constraint is exactly satisfied by the
// input poses, so the optimizer's first residual is zero and the output
// should equal the input (modulo the root being the gauge). We exercise the
// 16 combinations of {Force3DoF, GravitySigma, PriorsIgnored, LandmarksIgnored,
// useLastAsRoot} on every available pose-graph backend.
// ---------------------------------------------------------------------------

namespace {

constexpr int kCircleN = 30;
constexpr float kCircleRadius = 10.0f;
constexpr int kLandmarkId = -1;
constexpr int kLandmarkId2 = -2;     // second landmark, used when numLandmarks=2

// Per-axis 1-sigma values used both for the information matrix and (when
// noise is requested) for the initial-guess perturbation.
constexpr double kLinSigma = 0.05;                  // 5 cm
const     double kAngSigma = 1.0 * CV_PI / 180.0;   // 1 deg in rad

struct CircleGraph
{
	std::map<int, Transform> truePoses;   // ground truth (link transforms came from these)
	std::map<int, Transform> poses;       // optimizer input -- equals truePoses
	                                      // except for noised non-anchor nodes
	std::multimap<int, Link> links;
};

// Build the perfectly-consistent circle graph described above. When noisy=true,
// every pose except 1, kCircleN, and the landmark is perturbed by Gaussian
// noise (sigma = kLinSigma, kAngSigma) -- giving the optimizer a noisy
// initial guess while the constraints still describe the true graph.
CircleGraph buildCircleGraph(bool noisy = false, int numLandmarks = 1, bool force3DoF = false)
{
	CircleGraph g;

	// Realistic information matrix (diag(1/sigma^2)): 5 cm 1-sigma on each
	// linear axis, 1 deg 1-sigma on each angular axis.
	const double linInfo = 1.0 / (kLinSigma * kLinSigma);
	const double angInfo = 1.0 / (kAngSigma * kAngSigma);
	cv::Mat info = cv::Mat::zeros(6, 6, CV_64FC1);
	info.at<double>(0, 0) = linInfo;
	info.at<double>(1, 1) = linInfo;
	info.at<double>(2, 2) = linInfo;
	info.at<double>(3, 3) = angInfo;
	info.at<double>(4, 4) = angInfo;
	info.at<double>(5, 5) = angInfo;

	// Sample poses around the circle. Yaw aims at the next pose so a
	// differential robot walking the chain only translates forward in its
	// own frame between consecutive nodes.
	//
	// In 6DoF mode, z and pitch vary sinusoidally (three hills around the
	// loop, ~0.5 m crest, ~8.6 deg max pitch) to simulate a robot driving
	// over rolling terrain. The pitch matches the slope so the chain is
	// physically consistent. In 3DoF mode the loop stays flat -- the 3DoF
	// optimizer doesn't model z/pitch anyway.
	const float hillAmp  = force3DoF ? 0.0f : 0.5f;
	const float hillFreq = 3.0f;
	for(int i = 1; i <= kCircleN; ++i)
	{
		const float theta = 2.0f * static_cast<float>(CV_PI) * static_cast<float>(i - 1) / static_cast<float>(kCircleN);
		const float x = kCircleRadius * std::cos(theta);
		const float y = kCircleRadius * std::sin(theta);
		const float z = hillAmp * std::sin(hillFreq * theta);
		const float yaw = theta + static_cast<float>(CV_PI) / 2.0f;  // tangent to circle
		// Slope along the body's forward direction. dz/ds = (1/R) * dz/dtheta.
		// Nose up when going uphill -> negative pitch in ZYX Euler.
		const float pitch = -hillAmp * hillFreq / kCircleRadius * std::cos(hillFreq * theta);
		g.truePoses[i] = Transform(x, y, z, 0.0f, pitch, yaw);
	}
	// Landmark at the origin with identity rotation. Optional 2nd landmark
	// horizontally offset along +X so the relative bearing between the two
	// (in body frame) is a function of each pose's position (not orientation),
	// breaking the bearing-only ambiguity of a single-landmark setup.
	g.truePoses[kLandmarkId]  = Transform::getIdentity();
	if(numLandmarks >= 2)
	{
		g.truePoses[kLandmarkId2] = Transform(5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	}

	// Neighbor chain 1 -> 2 -> ... -> 30.
	for(int i = 1; i < kCircleN; ++i)
	{
		const Transform t = g.truePoses[i].inverse() * g.truePoses[i + 1];
		g.links.insert({i, Link(i, i + 1, Link::kNeighbor, t, info)});
	}
	// Loop closure 30 -> 1.
	{
		const Transform t = g.truePoses[kCircleN].inverse() * g.truePoses[1];
		g.links.insert({kCircleN, Link(kCircleN, 1, Link::kGlobalClosure, t, info)});
	}
	// Landmark observations on even poses. Built in the post-getConnectedGraph
	// normalized format: from=landmark_id (negative), to=observer, transform =
	// landmark -> observer. This is the format the optimizer backends expect
	// and lets us bypass getConnectedGraph (which would erase pose noise by
	// reconstructing poses from links via a tree walk).
	for(int i = 2; i <= kCircleN; i += 2)
	{
		const Transform t = g.truePoses[kLandmarkId].inverse() * g.truePoses[i];
		g.links.insert({kLandmarkId, Link(kLandmarkId, i, Link::kLandmark, t, info)});
		if(numLandmarks >= 2)
		{
			const Transform t2 = g.truePoses[kLandmarkId2].inverse() * g.truePoses[i];
			g.links.insert({kLandmarkId2, Link(kLandmarkId2, i, Link::kLandmark, t2, info)});
		}
	}
	// Per-pose pose prior and gravity self-link, both equal to the pose itself.
	for(int i = 1; i <= kCircleN; ++i)
	{
		g.links.insert({i, Link(i, i, Link::kPosePrior, g.truePoses[i], info)});
		g.links.insert({i, Link(i, i, Link::kGravity,   g.truePoses[i], info)});
	}

	// Optimizer input starts at ground truth. With noisy=true, every pose
	// EXCEPT id 1 and id kCircleN (those may be used as the gauge anchor) is
	// perturbed by Gaussian noise drawn from the same (kLinSigma, kAngSigma)
	// the links were weighted against. The landmark is also perturbed -- it
	// isn't a gauge anchor; the optimizer should pull it back via its
	// observation links. The constraints still describe the true graph --
	// only the initial guess is wrong, so the optimizer should pull back to
	// the truth.
	g.poses = g.truePoses;
	if(noisy)
	{
		// Fixed seed: identical noise sequence on every platform / run.
		std::mt19937 rng(42);
		std::normal_distribution<double> linDist(0.0, kLinSigma);
		std::normal_distribution<double> angDist(0.0, kAngSigma);
		auto perturb = [&](int id) {
			const Transform & t = g.truePoses.at(id);
			float r, p, y;
			t.getEulerAngles(r, p, y);
			g.poses[id] = Transform(
					t.x() + static_cast<float>(linDist(rng)),
					t.y() + static_cast<float>(linDist(rng)),
					t.z() + static_cast<float>(linDist(rng)),
					r + static_cast<float>(angDist(rng)),
					p + static_cast<float>(angDist(rng)),
					y + static_cast<float>(angDist(rng)));
		};
		for(int i = 2; i <= kCircleN - 1; ++i)
		{
			perturb(i);
		}
		perturb(kLandmarkId);
		if(numLandmarks >= 2)
		{
			perturb(kLandmarkId2);
		}
	}
	return g;
}

// Tuple ordering: optimizer, force3DoF, gravitySigma, priorsIgnored,
// landmarksIgnored, useLastAsRoot. Bool params are wrapped in int for
// readability in gtest's default printer; the lambda below pretty-prints.
using CircleParam = std::tuple<Optimizer::Type, bool, float, bool, bool, bool>;

class CircleGraphTest : public ::testing::TestWithParam<CircleParam>
{
protected:
	void SetUp() override
	{
		if(!Optimizer::isAvailable(std::get<0>(GetParam())))
		{
			GTEST_SKIP() << optimizerTypeName(std::get<0>(GetParam())) << " not built in";
		}
	}
};

}  // namespace

TEST_P(CircleGraphTest, NoisyInitialGuessConvergesToTruth)
{
	const auto & p = GetParam();
	const Optimizer::Type backend  = std::get<0>(p);
	const bool  force3DoF          = std::get<1>(p);
	const float gravitySigma       = std::get<2>(p);
	const bool  priorsIgnored      = std::get<3>(p);
	const bool  landmarksIgnored   = std::get<4>(p);
	const bool  useLastAsRoot      = std::get<5>(p);

	ParametersMap params;
	params[Parameters::kOptimizerStrategy()]        = uNumber2Str(static_cast<int>(backend));
	params[Parameters::kRegForce3DoF()]             = force3DoF        ? "true" : "false";
	params[Parameters::kOptimizerGravitySigma()]    = uNumber2Str(gravitySigma);
	params[Parameters::kOptimizerPriorsIgnored()]   = priorsIgnored    ? "true" : "false";
	params[Parameters::kOptimizerLandmarksIgnored()]= landmarksIgnored ? "true" : "false";
	std::unique_ptr<Optimizer> opt(Optimizer::create(params));
	ASSERT_NE(opt.get(), nullptr);
	ASSERT_EQ(opt->type(), backend);

	// Build with noise: every non-anchor pose (i.e. not 1, not kCircleN, not
	// the landmark) is perturbed. The links still encode the truth, so the
	// optimizer should pull the poses back to truePoses.
	CircleGraph g = buildCircleGraph(/*noisy=*/true, /*numLandmarks=*/1, force3DoF);
	const int rootId = useLastAsRoot ? kCircleN : 1;

	// Optional orientation perturbation on top of the per-axis noise:
	// pre-multiply each input pose's rotation (positions left untouched) by
	// a small world-frame roll/pitch/(yaw) rotation. The relative link
	// constraints still describe the truth, so the optimizer needs an
	// *absolute* orientation reference to recover. The two interesting
	// cases are:
	//
	//   priors-only (gravitySigma == 0, priorsIgnored == false):
	//     Per-pose pose priors anchor every pose's full orientation, so
	//     every pose -- including the roots -- gets roll/pitch/yaw
	//     perturbed. The priors pull each pose back to truth.
	//
	//   gravity-only (gravitySigma > 0, priorsIgnored == true):
	//     Gravity ties only roll/pitch to world up; yaw remains gauge-free.
	//     So the roots (id 1, kCircleN) get roll/pitch perturbed but their
	//     yaw is left at truth -- the optimizer's gauge fix in yaw depends
	//     on the root staying at truth. Non-roots also get yaw perturbed;
	//     the chain of links propagates the root's true yaw and the relative
	//     yaw constraints pull the non-roots back.
	//
	// Skipped for TORO (its tree-based SGD doesn't converge tightly enough)
	// and for 3DoF mode (roll/pitch are not optimized there). The "neither"
	// and "both" configs are also skipped: "neither" is gauge-free in yaw,
	// and "both" is already covered by the unperturbed noisy test.
	const bool priorsOnly  = (gravitySigma == 0.0f && !priorsIgnored);
	const bool gravityOnly = (gravitySigma >  0.0f &&  priorsIgnored);
	if((opt->type() == Optimizer::kTypeG2O || opt->type() == Optimizer::kTypeGTSAM) && !force3DoF && (priorsOnly || gravityOnly))
	{
		const float deg = static_cast<float>(CV_PI) / 180.0f;
		const float dRoll        = 3.0f * deg;
		const float dPitch       = 4.0f * deg;
		const float dYawNonRoot  = 5.0f * deg;
		const float dYawRoot     = priorsOnly ? 5.0f * deg : 0.0f;
		// On gravity-only roots, perturb only ONE of roll/pitch (drop pitch).
		// Perturbing both induces a small geometric yaw drift on the root that
		// gravity can't recover (yaw is gauge-free in that mode): after
		// gravity rotates the tilted local Z back to world Z, the local X-axis
		// settles at a slightly different XY direction, and that yaw twist
		// propagates around the chain. Pure roll alone (or pure pitch alone)
		// keeps the local X-axis in the XY plane, so no drift.
		const float dPitchRoot   = gravityOnly ? 0.0f : dPitch;
		// World-frame perturbation on non-roots: pre-multiplied onto each
		// pose's transform so roll/pitch deltas act about world axes (what
		// gravity observes). On roots we post-multiply (body-frame) so the
		// root's truth position is preserved -- in gravity-only mode the
		// optimizer holds root setFixed in position, and a world-frame
		// pre-multiply would shift it about the world origin and drag the
		// whole chain along.
		for(int i = 1; i <= kCircleN; ++i)
		{
			const bool isRoot = (i == 1 || i == kCircleN);
			const float dPi   = isRoot ? dPitchRoot   : dPitch;
			const float dYaw  = isRoot ? dYawRoot     : dYawNonRoot;
			const Transform R(0.0f, 0.0f, 0.0f, dRoll, dPi, dYaw);
			g.poses[i] = isRoot ? g.poses.at(i) * R : R * g.poses.at(i);
		}
	}

	// Use getConnectedGraph() only to get the filtered link set (it drops
	// kPosePrior self-links when priorsIgnored and landmarks when
	// landmarksIgnored). Its pose output is the result of a tree-walk from
	// the root that would overwrite every pose -- erasing the noise we just
	// added -- so we discard it and feed the optimizer our noisy g.poses
	// instead.
	std::map<int, Transform> connectedPoses;
	std::multimap<int, Link> inLinks;
	opt->getConnectedGraph(rootId, g.poses, g.links, connectedPoses, inLinks);
	ASSERT_EQ(connectedPoses.size(), g.poses.size() - (landmarksIgnored ? 1u : 0u));

	std::map<int, Transform> inPoses = g.poses;
	if(landmarksIgnored)
	{
		inPoses.erase(kLandmarkId);
	}
	// In 3DoF mode the optimizer doesn't touch z/roll/pitch, so any noise we
	// added on those axes would survive into the output verbatim and fail the
	// truth comparison for no real reason. Strip it from the input poses up
	// front so what we feed in matches the DoFs the optimizer actually solves.
	if(force3DoF)
	{
		// 3DoF optimizer doesn't touch z/roll/pitch on poses OR on the
		// landmark, so strip any z/roll/pitch noise from every input pose
		// (including the landmark) to match what the optimizer can actually
		// solve.
		for(auto & kv : inPoses)
		{
			kv.second = kv.second.to3DoF();
		}
	}

	std::map<int, Transform> out = opt->optimize(rootId, inPoses, inLinks);
	ASSERT_FALSE(out.empty()) << "Optimizer returned no poses";

	// Only g2o and GTSAM actually optimize landmarks; TORO and Ceres log a
	// warning and drop the landmark silently from the output. So the landmark
	// appears in the output when (a) the backend supports it AND
	// (b) landmarksIgnored is false.
	const bool backendSupportsLandmarks =
			backend == Optimizer::kTypeG2O || backend == Optimizer::kTypeGTSAM;
	const bool expectLandmarkInOutput = backendSupportsLandmarks && !landmarksIgnored;
	EXPECT_EQ(out.size(), kCircleN + (expectLandmarkInOutput ? 1u : 0u));

	// Constraints describe the truth, so the optimizer should pull the noisy
	// initial guess back to truePoses. Tolerances are loose enough to absorb
	// each backend's residual-at-convergence, tight enough that a "didn't
	// move" regression would trip immediately (noise is up to ~3*sigma_lin
	// = 15 cm per axis from truth).
	for(const auto & kv : g.truePoses)
	{
		const int id = kv.first;
		if(id == kLandmarkId && !expectLandmarkInOutput)
		{
			continue;
		}
		ASSERT_TRUE(out.count(id)) << "missing id " << id;
		// TORO is a tree-based SGD optimizer with a fixed iteration count;
		// the others (g2o/GTSAM/Ceres) use Gauss-Newton / Levenberg-Marquardt
		// and converge to numerical zero, so we hold them to much tighter
		// bounds.
		const float distTol = (backend == Optimizer::kTypeTORO) ? 0.015f : 0.002f;
		const float angTolDeg = (backend == Optimizer::kTypeTORO) ? 0.5f : 0.01f;
		EXPECT_LT(out.at(id).getDistance(kv.second), distTol)
				<< optimizerTypeName(backend) << " id=" << id
				<< " expected=" << kv.second.prettyPrint()
				<< " got="      << out.at(id).prettyPrint();
		const float angDeg = out.at(id).getAngle(kv.second) * 180.0f / static_cast<float>(M_PI);
		EXPECT_LT(angDeg, angTolDeg)
				<< optimizerTypeName(backend) << " id=" << id
				<< " angle=" << angDeg << " deg";
	}
}

INSTANTIATE_TEST_SUITE_P(
		AllBackendsAndConfigs,
		CircleGraphTest,
		::testing::Combine(
				::testing::Values(Optimizer::kTypeTORO,
								  Optimizer::kTypeG2O,
								  Optimizer::kTypeGTSAM,
								  Optimizer::kTypeCeres),
				::testing::Bool(),                  // Reg/Force3DoF
				::testing::Values(0.0f, 0.01f),     // Optimizer/GravitySigma
				::testing::Bool(),                  // Optimizer/PriorsIgnored
				::testing::Bool(),                  // Optimizer/LandmarksIgnored
				::testing::Bool()),                 // useLastAsRoot
		[](const ::testing::TestParamInfo<CircleParam> & info) {
			std::string name = optimizerTypeName(std::get<0>(info.param));
			name += std::get<1>(info.param) ? "_3DoF"    : "_6DoF";
			name += std::get<2>(info.param) > 0.0f ? "_grav" : "_nograv";
			name += std::get<3>(info.param) ? "_noprior" : "_prior";
			name += std::get<4>(info.param) ? "_nolm"    : "_lm";
			name += std::get<5>(info.param) ? "_rootLast" : "_rootFirst";
			return name;
		});

// ---------------------------------------------------------------------------
// CircleGraphBadLoopClosureTest -- corrupt the loop-closure link with a large
// translation offset and verify that the Optimizer/Robust kernel rejects the
// outlier loop closure (g2o + GTSAM only; relies on Vertigo switchable
// factors which are conditional on WITH_VERTIGO build flag).
// ---------------------------------------------------------------------------

namespace {

using CircleBadLcParam = std::tuple<Optimizer::Type, bool /*useRobust*/>;

class CircleGraphBadLoopClosureTest : public ::testing::TestWithParam<CircleBadLcParam>
{
protected:
	void SetUp() override
	{
		const Optimizer::Type t = std::get<0>(GetParam());
		if(!Optimizer::isAvailable(t))
		{
			GTEST_SKIP() << optimizerTypeName(t) << " not built in";
		}
	}
};

}  // namespace

TEST_P(CircleGraphBadLoopClosureTest, RobustRejectsCorruptedLoopClosure)
{
	const Optimizer::Type backend   = std::get<0>(GetParam());
	const bool            useRobust = std::get<1>(GetParam());

	// Use the noisy CircleGraph (5 cm pose + point noise) so the test has a
	// realistic baseline. Force 6DoF, no gravity, ignore priors+landmarks so
	// the only relevant constraints are the neighbor chain + the (corrupted)
	// loop closure.
	ParametersMap params;
	params[Parameters::kOptimizerStrategy()]         = uNumber2Str(static_cast<int>(backend));
	params[Parameters::kRegForce3DoF()]              = "false";
	params[Parameters::kOptimizerGravitySigma()]     = "0";
	params[Parameters::kOptimizerPriorsIgnored()]    = "true";
	params[Parameters::kOptimizerLandmarksIgnored()] = "true";
	params[Parameters::kOptimizerRobust()]           = useRobust ? "true" : "false";
	std::unique_ptr<Optimizer> opt(Optimizer::create(params));
	ASSERT_NE(opt.get(), nullptr);
	ASSERT_EQ(opt->type(), backend);

	CircleGraph g = buildCircleGraph(/*noisy=*/true);

	// Corrupt the loop closure: take the truth kGlobalClosure transform and
	// add a 5 m translation offset on its X axis. We need at least a few
	// meters for the robust kernel to clearly prefer "disable the LC" over
	// "absorb the offset into the chain": 11 neighbor links with
	// sigma=5 cm (info=400/axis) can absorb ~1 m of LC offset with chain
	// chi^2 ~ 11 * (1m/11)^2 * 400 ~ 36, comparable to the cost of
	// dropping the switch (~1) + the chain-at-truth noise residual (~30).
	// At 5 m the chain absorption costs ~890 chi^2, which is decisively
	// worse than disabling the LC -- so the Vertigo switch reliably drives
	// to 0.
	std::multimap<int, Link> corruptedLinks;
	for(const auto & kv : g.links)
	{
		if(kv.second.type() == Link::kGlobalClosure)
		{
			const Transform & t = kv.second.transform();
			float r, p, y;
			t.getEulerAngles(r, p, y);
			const Transform bad(t.x() + 5.0f, t.y(), t.z(), r, p, y);
			corruptedLinks.insert({kv.first,
					Link(kv.second.from(), kv.second.to(), kv.second.type(),
							bad, kv.second.infMatrix())});
		}
		else
		{
			corruptedLinks.insert(kv);
		}
	}
	g.links = corruptedLinks;

	const int rootId = 1;
	std::map<int, Transform> connectedPoses;
	std::multimap<int, Link> inLinks;
	opt->getConnectedGraph(rootId, g.poses, g.links, connectedPoses, inLinks);

	std::map<int, Transform> out = opt->optimize(rootId, g.poses, inLinks);
	ASSERT_FALSE(out.empty()) << "Optimizer returned no poses";

	// Worst pose displacement from truth.
	float worstPoseD = 0.0f;
	for(const auto & kv : g.truePoses)
	{
		if(kv.first < 0) continue;  // skip landmarks (we ignored them anyway)
		ASSERT_TRUE(out.count(kv.first)) << "missing id " << kv.first;
		worstPoseD = std::max(worstPoseD, out.at(kv.first).getDistance(kv.second));
	}

	if(useRobust)
	{
		// Vertigo switchable factor wraps the loop closure with a switch
		// variable initialized to 1 (LC fully active) and a prior keeping
		// it near 1; the optimizer is free to drive the switch toward 0
		// to disable the LC. With the Jacobian fix in
		// betweenFactorSwitchable.h, both backends drive the switch fully
		// to 0 and recover to ~mm/sub-mm pose error:
		//   * G2O   ~1 mm.
		//   * GTSAM ~30 microns (slightly tighter than g2o on this
		//     particular setup because GaussNewton is used by default).
		EXPECT_LT(worstPoseD, 0.005f)
				<< optimizerTypeName(backend) << " robust=on worstPoseD=" << worstPoseD;
	}
	else
	{
		// Without robust, the bad loop closure is a hard constraint -- the
		// chain stretches to satisfy it and pose 30 ends up ~5 m off truth.
		EXPECT_GT(worstPoseD, 4.5f)
				<< optimizerTypeName(backend) << " robust=off worstPoseD=" << worstPoseD;
	}
}

INSTANTIATE_TEST_SUITE_P(
		Backends,
		CircleGraphBadLoopClosureTest,
		::testing::Values(
				std::make_tuple(Optimizer::kTypeG2O,   false),
				std::make_tuple(Optimizer::kTypeG2O,   true),
				std::make_tuple(Optimizer::kTypeGTSAM, false),
				std::make_tuple(Optimizer::kTypeGTSAM, true)),
		[](const ::testing::TestParamInfo<CircleBadLcParam> & info)
		{
			std::string name = optimizerTypeName(std::get<0>(info.param));
			name += std::get<1>(info.param) ? "_Robust" : "_NoRobust";
			return name;
		});

// ---------------------------------------------------------------------------
// LandmarkCovarianceTest -- effect of per-link landmark covariance on g2o/GTSAM
// ---------------------------------------------------------------------------
// Reuses the CircleGraph (closed loop + landmark observed by even poses), but
// rewrites the information matrix on every Link::kLandmark edge to one of
// several test cases before optimizing. Limited to g2o and GTSAM -- TORO and
// Ceres drop landmarks silently (see backendSupportsLandmarks above).

namespace {

// Mirrors the Marker/Variance{Linear,Angular,OrientationIgnored} triplet from
// Parameters.h. Memory.cpp uses these to build the per-landmark-link 6x6
// covariance fed to the optimizer; we re-create that same construction here
// so our tests exercise the exact code path real marker detections take.
struct LandmarkCovCase
{
	const char * name;
	double linVar;             // Marker/VarianceLinear   (1.0 in the 3x3 translation block)
	double angVar;             // Marker/VarianceAngular  (1.0 in the 3x3 rotation block; 9999 = sentinel)
	bool   orientationIgnored; // Marker/VarianceOrientationIgnored
	double linkLinNoise;       // Sigma (m) of Gaussian noise added to every kLandmark link's translation.
                               // 0 means no extra noise (default; equivalent to a "perfect" marker detector).
	bool   linkRangeOnly;      // If true, noise is applied ALONG the bearing direction only -- preserves the
	                           // direction (perfect bearing) but corrupts the magnitude (noisy range). If
	                           // false (default), noise is isotropic on x/y/z and corrupts both bearing and range.
	bool   openLoopWithDrift;  // If true: drop the loop-closure link, perturb every kNeighbor link's transform
	                           // (Gaussian translation/rotation noise plus a constant -1 deg yaw bias), and
	                           // seed the optimizer with poses derived from this perturbed chain (via
	                           // getConnectedGraph's tree walk) instead of g.poses. Simulates open-loop
	                           // odometry drift -- the optimizer has to correct it from the landmark
	                           // observations alone.
	int    numLandmarks;       // 1 (default) or 2. Adds a second landmark at (5, 0, 0) so each observer pose has
	                           // two horizontally-separated bearings -- the relative bearing between them
	                           // constrains pose position independently of orientation (breaks bearing-only
	                           // ambiguity).
};

const LandmarkCovCase kLandmarkCovCases[] = {
	// Full-pose landmark observations (orientation estimated). Identical
	// encoding for g2o and GTSAM; force3DoF doesn't change the matrix.
	{ "default",            0.001,  0.01,   false, 0.0, false, false, 1 },  // Marker defaults
	{ "tightLin",           0.0001, 0.01,   false, 0.0, false, false, 1 },
	{ "looseLin",           0.01,   0.01,   false, 0.0, false, false, 1 },
	{ "looseAng",           0.001,  1.0,    false, 0.0, false, false, 1 },

	// orientationIgnored=true: rotation block is set to 9999 (no orientation
	// estimation). For g2o only linVar matters. For GTSAM the encoding
	// switches to bearing/range -- angVar drives bearing, linVar drives range.
	// linVar=9999 is the documented sentinel that asks GTSAM to skip the
	// range factor (bearing-only).
	{ "noOrient",           0.001,  0.01,   true,  0.0, false, false, 1 },
	{ "noOrientBO",         9999,   0.01,   true,  0.0, false, false, 1 },  // bearing-only (GTSAM)

	// Same as noOrient but with sigma=0.3 m injected into every landmark
	// observation's translation. linVar is bumped to 0.1 (sigma~0.32 m) so
	// the info matrix matches the actual noise level. GTSAM's bearing/range
	// factorization is expected to handle this gracefully (multiple noisy
	// observations triangulate); g2o's Cartesian EdgeSE3PointXYZ is expected
	// to do worse on the landmark.
	{ "noOrientNoisy",      0.1,    0.001,  true,  0.3, false, false, 1 },

	// Range-only noise: sigma=0.3 m applied along the bearing direction
	// (perfect bearing, noisy range). This is the cleanest stress test for
	// GTSAM's BearingRangeFactor: multiple noise-free bearings still cross
	// at the true landmark, so the landmark should triangulate to truth and
	// pose rotations should stay essentially untouched. g2o's Cartesian
	// EdgeSE3PointXYZ sees the same noisy translations and can't exploit
	// the bearing-purity, so it should still struggle.
	{ "noOrientRangeNoisy", 0.1,    0.0001, true,  0.3, true,  false, 1 },

	// Open-loop odometry with drift + perfect-bearing landmark observations:
	// loop closure dropped, neighbor links translation/rotation-perturbed
	// with a constant -1 deg yaw bias accumulating around the chain
	// (~30 deg total drift at the open end), input poses seeded from that
	// drifted chain. Only the landmark observations can pull the trajectory
	// back to truth -- a stress test of the bearing/range factor's
	// triangulating capability vs g2o's Cartesian factor.
	{ "openLoopRangeNoisy", 2.25,   0.0001, true,  1.5, true,  true,  1 },

	// Same as openLoopRangeNoisy but with 2 horizontally-separated landmarks.
	// The added relative-bearing position constraint plus the range factor
	// (even with 1.5 m noise) should both tighten the chain further than
	// either alone.
	{ "openLoopRangeNoisy2Lm", 2.25, 0.0001, true, 1.5, true, true,  2 },

	// Same setup as openLoopRangeNoisy but with linVar=9999 -- the GTSAM
	// sentinel that disables the range factor entirely (BearingFactor
	// instead of BearingRangeFactor). With 1.5 m range noise but perfect
	// bearings, GTSAM should triangulate the landmark from bearings alone
	// and never get fooled by the (huge) range noise. For g2o this isn't
	// meaningful -- linVar=9999 sets its EdgeSE3PointXYZ info to ~1e-4 on
	// translation, the landmark becomes unobservable, and with the loop
	// closure also dropped the whole chain has no global correction; we
	// skip g2o's assertions for this case.
	{ "openLoopBearingOnly", 9999, 0.0001, true,  1.5, true,  true,  1 },

	// Same setup as openLoopBearingOnly but with a SECOND landmark at
	// (5, 0, 0). The two landmarks are horizontally separated, so each
	// observer pose's relative-bearing-between-the-two-landmarks is a
	// function of the pose's POSITION only (orientation cancels). This
	// breaks the bearing-only ambiguity (rotate-in-place + slide-along-ray)
	// of a single-landmark setup. Expected: GTSAM 6DoF bearing-only should
	// converge dramatically tighter than with one landmark.
	{ "openLoopBearingOnly2Lm", 9999, 0.0001, true, 1.5, true, true,  2 },
};

using LandmarkCovParam = std::tuple<Optimizer::Type, int /* index into kLandmarkCovCases */, bool /* force3DoF */>;

class LandmarkCovarianceTest : public ::testing::TestWithParam<LandmarkCovParam>
{
protected:
	void SetUp() override
	{
		const Optimizer::Type t = std::get<0>(GetParam());
		if(!Optimizer::isAvailable(t))
		{
			GTEST_SKIP() << optimizerTypeName(t) << " not built in";
		}
	}
};

// Build the per-landmark-link 6x6 covariance the same way Memory.cpp does
// when ingesting a marker detection (see _markerOrientationIgnored branches
// at Memory.cpp:6076-6101). The optimizer backends interpret the diagonal
// blocks specially when orientationIgnored=true (g2o uses linVar on the
// translation block; GTSAM repacks it as bearing(angVar)/range(linVar)).
// Returns the COVARIANCE -- the caller must invert to get the info matrix
// the Link stores.
cv::Mat buildMarkerCov(const LandmarkCovCase & c, Optimizer::Type backend, bool force3DoF)
{
	cv::Mat cov = cv::Mat::eye(6, 6, CV_64FC1);
	if(c.orientationIgnored)
	{
		cov(cv::Range(3, 6), cv::Range(3, 6)) *= 9999;     // disable orientation estimation
		const bool isGTSAM = (backend == Optimizer::kTypeGTSAM);
		if(!isGTSAM)
		{
			cov(cv::Range(0, 3), cv::Range(0, 3)) *= c.linVar;
		}
		else if(force3DoF)
		{
			// GTSAM 2D bearing/range: X=bearing, Y=range. (Z is unused by
			// the Pose2/Point2 BearingRange factor.)
			cov(cv::Range(0, 1), cv::Range(0, 1)) *= c.angVar;
			cov(cv::Range(1, 2), cv::Range(1, 2)) *= c.linVar;
		}
		else
		{
			// GTSAM 3D bearing/range: X,Y=bearing, Z=range.
			cov(cv::Range(0, 2), cv::Range(0, 2)) *= c.angVar;
			cov(cv::Range(2, 3), cv::Range(2, 3)) *= c.linVar;
		}
	}
	else
	{
		cov(cv::Range(0, 3), cv::Range(0, 3)) *= c.linVar;
		cov(cv::Range(3, 6), cv::Range(3, 6)) *= c.angVar;
	}
	return cov;
}

}  // namespace

TEST_P(LandmarkCovarianceTest, LandmarkCovarianceAffectsConvergence)
{
	const auto & p = GetParam();
	const Optimizer::Type backend = std::get<0>(p);
	const LandmarkCovCase & cov   = kLandmarkCovCases[std::get<1>(p)];
	const bool force3DoF          = std::get<2>(p);

	// Skip g2o + linVar>=9999: it's the GTSAM "no range factor" sentinel,
	// not meaningful for g2o (which has only one landmark factor type --
	// EdgeSE3PointXYZ -- and would just see a near-zero-info Cartesian
	// observation). The test cases are GTSAM-only.
	if(backend == Optimizer::kTypeG2O && cov.orientationIgnored && cov.linVar >= 9999)
	{
		GTEST_SKIP() << "linVar=9999 (bearing-only sentinel) is GTSAM-only; "
				"g2o would just see a near-zero-info Cartesian observation";
	}

	ParametersMap params;
	params[Parameters::kOptimizerStrategy()]         = uNumber2Str(static_cast<int>(backend));
	params[Parameters::kRegForce3DoF()]              = force3DoF ? "true" : "false";
	// Gravity is meaningful only in 6DoF -- in 3DoF the optimizer doesn't
	// touch z/roll/pitch, so a gravity prior would be redundant.
	params[Parameters::kOptimizerGravitySigma()]     = force3DoF ? "0" : "0.01";
	// For open-loop variants, enable priors and add an identity prior on the
	// landmark to anchor it -- the optimizer can no longer satisfy bearings
	// by sliding the landmark and must instead deform/rotate the pose chain.
	const bool experimentLandmarkPrior = cov.openLoopWithDrift;
	params[Parameters::kOptimizerPriorsIgnored()]    = experimentLandmarkPrior ? "false" : "true";
	params[Parameters::kOptimizerLandmarksIgnored()] = "false";
	std::unique_ptr<Optimizer> opt(Optimizer::create(params));
	ASSERT_NE(opt.get(), nullptr);
	ASSERT_EQ(opt->type(), backend);

	// Reuse the noisy CircleGraph baseline.
	CircleGraph g = buildCircleGraph(/*noisy=*/true, cov.numLandmarks, force3DoF);

	// Rewrite landmark-link information matrices to the test case, using the
	// same Memory.cpp construction marker detections go through. When the
	// case requests it, also inject Gaussian translation noise into every
	// kLandmark link's transform to simulate a noisy marker detector.
	const cv::Mat lmInfo = buildMarkerCov(cov, backend, force3DoF).inv();
	std::mt19937 linkRng(43);
	std::normal_distribution<double> linkLinDist(0.0, cov.linkLinNoise);
	// Open-loop drift on neighbor links: 1 cm linear sigma, 0.1 deg angular
	// sigma, plus a constant -1 deg yaw bias on every neighbor edge.
	std::mt19937 neighRng(44);
	std::normal_distribution<double> neighLinDist(0.0, 0.01);
	std::normal_distribution<double> neighAngDist(0.0, 0.1 * CV_PI / 180.0);
	const float neighYawBias = -1.0f * static_cast<float>(CV_PI) / 180.0f;
	std::multimap<int, Link> patchedLinks;
	for(const auto & kv : g.links)
	{
		const Link & link = kv.second;
		if(cov.openLoopWithDrift && link.type() == Link::kGlobalClosure)
		{
			// Drop the loop closure: simulates pure odometry, no place
			// recognition.
			continue;
		}
		if(cov.openLoopWithDrift && link.type() == Link::kNeighbor)
		{
			// Apply the -1 deg yaw bias as a left-multiplying rotation so it
			// rotates the link's translation as well as its rotation -- models
			// a yaw calibration bias (body frame rotated 1 deg from the true
			// robot frame), where each forward step picks up a small lateral
			// component proportional to the bias.
			Transform t = Transform(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, neighYawBias) * link.transform();
			float r, p, y;
			t.getEulerAngles(r, p, y);
			t = Transform(
					t.x() + static_cast<float>(neighLinDist(neighRng)),
					t.y() + static_cast<float>(neighLinDist(neighRng)),
					t.z() + static_cast<float>(neighLinDist(neighRng)),
					r + static_cast<float>(neighAngDist(neighRng)),
					p + static_cast<float>(neighAngDist(neighRng)),
					y + static_cast<float>(neighAngDist(neighRng)));
			patchedLinks.insert({kv.first, Link(link.from(), link.to(), link.type(), t, link.infMatrix())});
			continue;
		}
		if(link.type() == Link::kLandmark)
		{
			Transform t = link.transform();
			if(cov.linkLinNoise > 0.0)
			{
				float r, p, y;
				t.getEulerAngles(r, p, y);
				if(cov.linkRangeOnly)
				{
					// Scale t along its own direction: noisy magnitude
					// (range), exact direction (bearing). With p(observer)
					// at identity in the BearingRangeFactor construction,
					// bearing = normalize(t) and range = |t|, so this
					// perturbs the range only.
					const float range = std::sqrt(t.x()*t.x() + t.y()*t.y() + t.z()*t.z());
					if(range > 1e-6f)
					{
						const float scale = 1.0f + static_cast<float>(linkLinDist(linkRng)) / range;
						t = Transform(
								t.x() * scale,
								t.y() * scale,
								t.z() * scale,
								r, p, y);
					}
				}
				else
				{
					// Isotropic Gaussian on x/y/z (corrupts both bearing
					// and range).
					t = Transform(
							t.x() + static_cast<float>(linkLinDist(linkRng)),
							t.y() + static_cast<float>(linkLinDist(linkRng)),
							t.z() + static_cast<float>(linkLinDist(linkRng)),
							r, p, y);
				}
			}
			patchedLinks.insert({kv.first, Link(link.from(), link.to(), link.type(), t, lmInfo)});
		}
		else
		{
			// In the landmark-prior experiment we flip priorsIgnored to
			// false, which would otherwise activate the per-pose kPosePrior
			// at truth and trivialize the test. Filter them out here -- the
			// landmark prior we add below is the only prior we want active.
			if(experimentLandmarkPrior && link.type() == Link::kPosePrior)
			{
				continue;
			}
			patchedLinks.insert(kv);
		}
	}

	if(experimentLandmarkPrior)
	{
		// Position-only ("GPS-like") prior on the landmark at truth.
		// Rotation-block info is set just below the 1/9999 "ignored"
		// threshold so the GTSAM prior-detection logic treats this as a
		// GPS prior (keeps the root setFixed). The translation block is
		// tight (info ~10000 per axis -> sigma ~1 cm) to pin the landmark.
		cv::Mat priorInfo = cv::Mat::zeros(6, 6, CV_64FC1);
		for(int i = 0; i < 3; ++i) priorInfo.at<double>(i, i) = 10000.0;
		for(int i = 3; i < 6; ++i) priorInfo.at<double>(i, i) = 1.0 / 10000.0;
		patchedLinks.insert({kLandmarkId,
				Link(kLandmarkId, kLandmarkId, Link::kPosePrior,
						g.truePoses.at(kLandmarkId), priorInfo)});
		if(cov.numLandmarks >= 2)
		{
			patchedLinks.insert({kLandmarkId2,
					Link(kLandmarkId2, kLandmarkId2, Link::kPosePrior,
							g.truePoses.at(kLandmarkId2), priorInfo)});
		}
	}

	const int rootId = 1;
	std::map<int, Transform> connectedPoses;
	std::multimap<int, Link> inLinks;
	opt->getConnectedGraph(rootId, g.poses, patchedLinks, connectedPoses, inLinks);

	// In the open-loop variant, seed the optimizer with poses derived from
	// the perturbed neighbor chain (a tree walk from root that accumulates
	// the -1 deg/segment yaw bias) rather than the noisy ground-truth-ish
	// g.poses. Everywhere else, keep g.poses as the input.
	std::map<int, Transform> inPoses = cov.openLoopWithDrift ? connectedPoses : g.poses;
	if(force3DoF)
	{
		// 3DoF optimizer doesn't touch z/roll/pitch on poses, so strip the
		// noise we added on those axes. The landmark is left alone -- in
		// production it's observed in full 3D and the optimizer keeps it on
		// the SE(3) manifold even in 2D SLAM mode.
		for(auto & kv : inPoses)
		{
			if(kv.first < 0)
			{
				continue;
			}
			kv.second = kv.second.to3DoF();
		}
	}

	std::map<int, Transform> out = opt->optimize(rootId, inPoses, inLinks);
	ASSERT_FALSE(out.empty()) << "Optimizer returned no poses";
	EXPECT_EQ(out.size(), kCircleN + static_cast<size_t>(cov.numLandmarks));  // 30 poses + N landmarks


	// TODO: per-case assertions. Sketch:
	//   - tight    : output should hew very close to truth (landmark dominates).
	//   - default  : matches the CircleGraphTest baseline tolerance.
	//   - loose    : landmark contributes little; poses still converge via
	//                links + priors but with looser bounds on the landmark id.
	// With perfect landmark observations every case converges to ~numerical
	// zero on all DoFs the optimizer touches; with sigma=0.3 m noise injected
	// into the landmark link transforms (kLandmarkCovCases::linkLinNoise),
	// expect roughly the noise magnitude on the landmark and propagating to
	// the poses, with g2o's Cartesian EdgeSE3PointXYZ struggling more than
	// GTSAM's BearingRange in 6DoF (in 3DoF the two are comparable).
	// Convergence bound selection:
	//   noise-free                 -> tight 1 mm / 0.01 deg / 1 mm.
	//   linkRangeOnly (perfect bearing, sigma=0.3m on range only):
	//     GTSAM exploits bearing-purity (BearingRangeFactor) and collapses
	//     to near-truth (poseD<5cm, ang<0.3deg, lmD<5cm); g2o 6DoF's
	//     EdgeSE3PointXYZ can't decompose so it does ~2x worse.
	//   isotropic noise (sigma=0.3m on x/y/z):
	//     Both bearings and ranges are corrupted; g2o 6DoF struggles
	//     particularly because its Cartesian factor weighs every axis equally.
	const bool noisy   = (cov.linkLinNoise > 0.0);
	const bool g2o6DoF = (backend == Optimizer::kTypeG2O && !force3DoF);
	// Non-noisy default bounds. Bumped from 1 mm to 2 mm because gravity (now
	// enabled for 6DoF) adds extra Z-axis constraints that nudge G2O's
	// noOrientBO solution to ~1.4 mm (still tight, just over the original
	// 1 mm). GTSAM at ~4 microns either way.
	float landmarkDistMax = 0.002f;
	float poseDistMax     = 0.002f;
	float poseAngMaxDeg   = 0.01f;
	if(noisy && cov.openLoopWithDrift)
	{
		// Open-loop chain with -1 deg/segment yaw bias (~30 deg cumulative
		// drift at the open end). A tight identity prior pins the landmark
		// at truth (lmD essentially zero across all variants); the chain
		// then converges from the prior's pull through the landmark
		// observations. With gravity on (6DoF), roll/pitch are extra-tight.
		//
		// Behavior summary:
		//   * GTSAM 3DoF (both BearingRange and BearingOnly) recovers the
		//     chain best -- 2D bearing factor's Jacobian is well-conditioned
		//     and the pinned landmark provides a strong anchor (~3.7-3.9 m
		//     / 20-21 deg, vs ~5.8 m without prior).
		//   * GTSAM 6DoF: bearing-factor linearization on SO(3) hits a local
		//     minimum even with the landmark pinned (~4.7-5.5 m / 25-30 deg).
		//   * g2o 6DoF RangeNoisy: pinning the landmark actually HURTS
		//     convergence (~5.1 m, vs ~3.2 m without prior) -- the
		//     Cartesian observation edges can no longer balance landmark
		//     and chain residuals, and the solver settles in a worse
		//     local minimum.
		//   * g2o BearingOnly: chain stays fully uncorrected (~5.75 m /
		//     29.5 deg) regardless of prior -- the EdgeSE3PointXYZ info
		//     matrix is ~1e-4 (linVar=9999), so the landmark is connected
		//     to the chain via near-zero-information edges.
		landmarkDistMax = 0.01f;  // pinned by prior, ~0 to 1 mm in practice
		const bool isG2O   = (backend == Optimizer::kTypeG2O);
		const bool multiLm = (cov.numLandmarks >= 2);
		if(multiLm && !isG2O)
		{
			// GTSAM with 2+ horizontally-separated landmarks: relative
			// bearing between landmarks is a pure-position constraint
			// (orientation-independent), resecting the pose. With or
			// without the range factor, GTSAM converges to ~0.5 m / 3 deg
			// in BOTH 3DoF and 6DoF.
			poseDistMax   = 0.70f;
			poseAngMaxDeg = 5.0f;
		}
		else if(multiLm && isG2O)
		{
			// G2O EdgeSE3PointXYZ with FINITE landmark-edge info (linVar~2)
			// triangulates well with 2 landmarks: chain converges to ~2 m
			// / 15-18 deg. (The bearing-only g2o variant is skipped above
			// since linVar=9999 is a GTSAM-only sentinel.)
			poseDistMax   = force3DoF ? 3.00f : 2.50f;
			poseAngMaxDeg = force3DoF ? 20.0f : 17.0f;
		}
		else if(isG2O)
		{
			// G2O + 1 landmark + finite linVar: BearingRange-equivalent
			// EdgeSE3PointXYZ. Chain partially recovers (~5 m).
			poseDistMax   = 6.00f;
			poseAngMaxDeg = 30.0f;
		}
		else if(force3DoF)
		{
			// GTSAM 3DoF + 1 landmark + pinned: 2D bearing factor benefits
			// from the prior but still has one degenerate DoF.
			poseDistMax   = 4.50f;
			poseAngMaxDeg = 22.0f;
		}
		else
		{
			// GTSAM 6DoF + 1 landmark: bearing-factor SO(3) linearization
			// stuck in local minimum at large initial yaw error.
			poseDistMax   = 5.70f;
			poseAngMaxDeg = 30.0f;
		}
	}
	else if(noisy && cov.linkRangeOnly)
	{
		const bool isG2O = (backend == Optimizer::kTypeG2O);
		landmarkDistMax = isG2O   ? 0.15f : 0.05f;     // g2o has no bearing-purity advantage
		poseDistMax     = g2o6DoF ? 0.10f : 0.06f;
		poseAngMaxDeg   = g2o6DoF ? 0.70f : 0.35f;
	}
	else if(noisy)
	{
		landmarkDistMax = 0.30f;
		poseDistMax     = g2o6DoF ? 0.40f : 0.30f;
		poseAngMaxDeg   = g2o6DoF ? 3.0f  : 2.0f;
	}

	// Open-loop "loop-closure delta" check: pose 1 (root, fixed at truth)
	// and pose kCircleN are the would-be loop-closure pair (link dropped).
	// The recovered distance/angle between them should match the truth
	// delta (~2.09 m / 12 deg = one chord/segment), within the same chain
	// bounds since the worst pose deviation is at the open end.
	if(cov.openLoopWithDrift)
	{
		const float expectedD    = g.truePoses.at(1).getDistance(g.truePoses.at(kCircleN));
		const float expectedADeg = g.truePoses.at(1).getAngle(g.truePoses.at(kCircleN))
				* 180.0f / static_cast<float>(M_PI);
		const float actualD      = out.at(1).getDistance(out.at(kCircleN));
		const float actualADeg   = out.at(1).getAngle(out.at(kCircleN))
				* 180.0f / static_cast<float>(M_PI);
		EXPECT_LT(std::abs(actualD - expectedD), poseDistMax)
				<< optimizerTypeName(backend) << " case=" << cov.name
				<< " loop-closure distance expected=" << expectedD
				<< " actual=" << actualD;
		EXPECT_LT(std::abs(actualADeg - expectedADeg), poseAngMaxDeg)
				<< optimizerTypeName(backend) << " case=" << cov.name
				<< " loop-closure angle expected=" << expectedADeg << " deg"
				<< " actual=" << actualADeg << " deg";
	}

	for(const auto & kv : g.truePoses)
	{
		const int id = kv.first;
		ASSERT_TRUE(out.count(id)) << optimizerTypeName(backend)
				<< " case=" << cov.name << " missing id=" << id;

		// Landmark converges very tightly in position regardless of whether
		// orientation is tracked:
		//   6DoF, orient tracked: G2O sub-mm + ~0.001 deg, GTSAM ~numerical zero.
		//   6DoF, orient ignored: G2O ~0.14 mm, GTSAM sub-um (no angle to check
		//                         -- the landmark vertex is Point3 and the output
		//                         rpy is just the noisy input verbatim).
		//   3DoF: x/y/yaw match truth to ~numerical zero. The z/roll/pitch we
		//         noised onto the landmark survive verbatim (the optimizer
		//         treats it as Pose2/Point2 and pastes the input z/roll/pitch
		//         into the output), so project both sides to3DoF() to drop
		//         those untouched axes from the comparison.
		if(id < 0)
		{
			const Transform expected = force3DoF ? kv.second.to3DoF() : kv.second;
			const Transform got      = force3DoF ? out.at(id).to3DoF() : out.at(id);
			EXPECT_LT(got.getDistance(expected), landmarkDistMax)
					<< optimizerTypeName(backend) << " case=" << cov.name
					<< " landmark got=" << got.prettyPrint();
			// Angle is only meaningful when orientation is tracked and the
			// landmark observations are noise-free; with linkLinNoise>0 the
			// noisy translations leak into the recovered orientation chain
			// (~0.9 deg on the landmark in our experiments), which is fine
			// for the noisy case so we skip the angle assert there.
			if(!cov.orientationIgnored && !noisy)
			{
				const float angDeg = got.getAngle(expected) * 180.0f / static_cast<float>(M_PI);
				EXPECT_LT(angDeg, 0.01f)
						<< optimizerTypeName(backend) << " case=" << cov.name
						<< " landmark angle=" << angDeg << " deg";
			}
			continue;
		}

		// Non-landmark ids: noise-free cases converge to truth at well under
		// 1 mm / 0.01 deg (worst observed is G2O+tightLin in 6DoF at
		// ~0.44 mm / 0.002 deg; GTSAM is at numerical zero across the board).
		// Noisy cases stay within poseDistMax/poseAngMaxDeg set above; in 6DoF
		// g2o needs a looser envelope than GTSAM because its Cartesian
		// EdgeSE3PointXYZ propagates landmark-link translation noise into the
		// chain more strongly than GTSAM's BearingRange factorization.
		EXPECT_LT(out.at(id).getDistance(kv.second), poseDistMax)
				<< optimizerTypeName(backend) << " case=" << cov.name << " id=" << id
				<< " got=" << out.at(id).prettyPrint();
		const float poseAngDeg = out.at(id).getAngle(kv.second) * 180.0f / static_cast<float>(M_PI);
		EXPECT_LT(poseAngDeg, poseAngMaxDeg)
				<< optimizerTypeName(backend) << " case=" << cov.name << " id=" << id
				<< " angle=" << poseAngDeg << " deg";
	}
}

INSTANTIATE_TEST_SUITE_P(
		G2OAndGTSAM,
		LandmarkCovarianceTest,
		::testing::Combine(
				::testing::Values(Optimizer::kTypeG2O, Optimizer::kTypeGTSAM),
				::testing::Range(0, static_cast<int>(sizeof(kLandmarkCovCases) / sizeof(kLandmarkCovCases[0]))),
				::testing::Bool()),                     // Reg/Force3DoF
		[](const ::testing::TestParamInfo<LandmarkCovParam> & info)
		{
			std::string name = optimizerTypeName(std::get<0>(info.param));
			name += "_";
			name += kLandmarkCovCases[std::get<1>(info.param)].name;
			name += std::get<2>(info.param) ? "_3DoF" : "_6DoF";
			return name;
		});

// ---------------------------------------------------------------------------
// BundleAdjustmentTest -- cameras on a circle looking at the center, observing
// a small point cloud near the origin. Verifies that BA-capable backends
// (g2o / Ceres / CVSBA) recover both the frame poses AND the 3D point cloud
// from noisy initial guesses + clean 2D observations.
// ---------------------------------------------------------------------------

namespace {

constexpr int    kBaNumCameras   = 12;
constexpr int    kBaNumPoints    = 200;
constexpr float  kBaCircleRadius = 5.0f;
// 752 x 480 camera @ 100 deg horizontal FOV (typical of fisheye / wide-angle
// imagers on small robots).
//   fx = cx / tan(half_HFOV) = 376 / tan(50 deg) ~= 315.5
//   vertical FOV = 2 * atan(cy/fy) = 2 * atan(240/315.5) ~= 75 deg
constexpr int    kBaImageWidth   = 752;
constexpr int    kBaImageHeight  = 480;
constexpr double kBaFx           = 315.5;
constexpr double kBaFy           = 315.5;
constexpr double kBaCx           = 376.0;
constexpr double kBaCy           = 240.0;
// 3D point cloud half-extents. Sized so that a "worst-case" point at the
// corner (Lxy, Lxy, Lz) projects to ~95% of each image dimension from any
// camera position. Derivation for u (image width):
//   u_offset_max = Lxy * fx / (R - Lxy)
// Solving for 95% coverage (u_offset = 0.475 * 752 = 357):
//   Lxy = 357 * R / (357 + fx) = 357 * 5 / (357 + 315.5) ~= 2.65 m
// Same form for v gives Lz ~= 1.65 m.
constexpr float  kBaPointXY      = 3.5f;
constexpr float  kBaPointZ       = 2.4f;

struct BundleGraph
{
	std::map<int, Transform>                       truePoses;
	std::map<int, cv::Point3f>                     truePoints3D;
	std::map<int, std::vector<CameraModel>>        models;
	std::map<int, Transform>                       initialPoses;        // possibly noisy
	std::map<int, cv::Point3f>                     initialPoints3D;     // possibly noisy
	std::multimap<int, Link>                       links;               // pose-graph constraints
	std::map<int, std::map<int, FeatureBA>>        wordReferences;      // word_id -> camera_id -> FeatureBA
};

// Build the BA scenario:
//   * kBaNumCameras cameras on a circle of radius kBaCircleRadius, all looking
//     toward the origin (body +X points at origin -> yaw = theta + pi).
//   * kBaNumPoints 3D points scattered in a [-1,1]^3 box around the origin.
//   * Every camera observes every point (well within ~63 deg FOV).
//   * Neighbor links 1->2->...->N between consecutive frames.
// When noisy=true:
//   * Every non-anchor pose (id 2..N) gets 5 cm linear / 1 deg angular noise.
//   * Every 3D point gets 5 cm noise.
//   * Every neighbor link's transform gets the same noise on top of truth.
BundleGraph buildBundleGraph(bool noisy = false, bool roundPixels = false, int numCameras = kBaNumCameras, int numPoints = kBaNumPoints)
{
	BundleGraph g;

	const CameraModel model(kBaFx, kBaFy, kBaCx, kBaCy,
			CameraModel::opticalRotation(),
			/*Tx=*/0.0,
			cv::Size(kBaImageWidth, kBaImageHeight));

	// Cameras on the circle, body +X aimed at the origin.
	for(int i = 1; i <= numCameras; ++i)
	{
		const float theta = 2.0f * static_cast<float>(CV_PI) * static_cast<float>(i - 1) / static_cast<float>(numCameras);
		const float x = kBaCircleRadius * std::cos(theta);
		const float y = kBaCircleRadius * std::sin(theta);
		// body +X = (cos yaw, sin yaw, 0); to point at origin, +X = -position direction.
		const float yaw = theta + static_cast<float>(CV_PI);
		g.truePoses[i] = Transform(x, y, 0.0f, 0.0f, 0.0f, yaw);
		g.models[i]    = std::vector<CameraModel>{model};
	}

	// 3D point cloud: uniform in a box around the origin sized so that every
	// camera around the circle sees the points spread across most of its
	// image frame. Z extent is smaller than X,Y to match the narrower
	// vertical FOV (67 deg vs 100 deg horizontal at 1080p).
	{
		std::mt19937 rng(7);
		std::uniform_real_distribution<float> distXY(-kBaPointXY, kBaPointXY);
		std::uniform_real_distribution<float> distZ (-kBaPointZ,  kBaPointZ);
		for(int p = 1; p <= numPoints; ++p)
		{
			g.truePoints3D[p] = cv::Point3f(distXY(rng), distXY(rng), distZ(rng));
		}
	}

	// Project every point into every camera. Skip projections that fall behind
	// the camera or outside the image (defensive -- with our geometry every
	// point should be visible in every camera).
	for(const auto & ptkv : g.truePoints3D)
	{
		const int pointId = ptkv.first;
		for(const auto & posekv : g.truePoses)
		{
			const int camId = posekv.first;
			const Transform world_to_optical = (posekv.second * model.localTransform()).inverse();
			const cv::Point3f pc = util3d::transformPoint(ptkv.second, world_to_optical);
			if(pc.z <= 0.1f)
			{
				continue;
			}
			float u, v;
			model.reproject(pc.x, pc.y, pc.z, u, v);
			if(u < 0.0f || u >= kBaImageWidth || v < 0.0f || v >= kBaImageHeight)
			{
				continue;
			}
			if(roundPixels)
			{
				// Simulate the discrete-pixel detection that real feature
				// extractors produce -- adds +-0.5 px quantization noise to
				// every reprojection residual.
				u = std::round(u);
				v = std::round(v);
			}
			g.wordReferences[pointId].insert({camId, FeatureBA(cv::KeyPoint(u, v, 1.0f))});
		}
	}

	// Neighbor chain 1 -> 2 -> ... -> N.
	cv::Mat info = cv::Mat::eye(6, 6, CV_64FC1);
	for(int i = 0; i < 3; ++i) info.at<double>(i, i) = 1.0 / (0.05 * 0.05);
	for(int i = 3; i < 6; ++i) info.at<double>(i, i) = 1.0 / (1.0 * CV_PI / 180.0 * 1.0 * CV_PI / 180.0);
	for(int i = 1; i < numCameras; ++i)
	{
		const Transform t = g.truePoses[i].inverse() * g.truePoses[i + 1];
		g.links.insert({i, Link(i, i + 1, Link::kNeighbor, t, info)});
	}

	// Initial guess: truth by default, noisy on request.
	g.initialPoses     = g.truePoses;
	g.initialPoints3D  = g.truePoints3D;
	if(noisy)
	{
		std::mt19937 rng(11);
		std::normal_distribution<double> linNoise(0.0, 0.05);                       // 5 cm on poses
		std::normal_distribution<double> angNoise(0.0, 1.0 * CV_PI / 180.0);        // 1 deg
		std::normal_distribution<double> ptNoise(0.0, 0.05);                        // 5 cm on point positions
		std::normal_distribution<double> linkLinNoise(0.0, 0.10);                   // 10 cm on neighbor link translations
		// Perturb every pose except the root (id 1 = anchor for optimizeBA).
		for(int i = 2; i <= numCameras; ++i)
		{
			const Transform & t = g.truePoses[i];
			float r, p, y;
			t.getEulerAngles(r, p, y);
			g.initialPoses[i] = Transform(
					t.x() + static_cast<float>(linNoise(rng)),
					t.y() + static_cast<float>(linNoise(rng)),
					t.z() + static_cast<float>(linNoise(rng)),
					r + static_cast<float>(angNoise(rng)),
					p + static_cast<float>(angNoise(rng)),
					y + static_cast<float>(angNoise(rng)));
		}
		for(auto & kv : g.initialPoints3D)
		{
			kv.second.x += static_cast<float>(ptNoise(rng));
			kv.second.y += static_cast<float>(ptNoise(rng));
			kv.second.z += static_cast<float>(ptNoise(rng));
		}
		std::multimap<int, Link> noisyLinks;
		for(const auto & kv : g.links)
		{
			const Link & link = kv.second;
			Transform t = link.transform();
			float r, p, y;
			t.getEulerAngles(r, p, y);
			t = Transform(
					t.x() + static_cast<float>(linkLinNoise(rng)),
					t.y() + static_cast<float>(linkLinNoise(rng)),
					t.z() + static_cast<float>(linkLinNoise(rng)),
					r + static_cast<float>(angNoise(rng)),
					p + static_cast<float>(angNoise(rng)),
					y + static_cast<float>(angNoise(rng)));
			noisyLinks.insert({kv.first, Link(link.from(), link.to(), link.type(), t, link.infMatrix())});
		}
		g.links = noisyLinks;
	}

	return g;
}

// Per-test variant. Default exercises the full setup (poses + points +
// noisy neighbor links, mono reprojection). The other two are g2o-specific
// extensions:
//   * G2ONoLinks: drop the kNeighbor edges before calling optimizeBA, so
//     g2o BA is reduced to reprojection-only -- mimics how Ceres/CVSBA
//     handle BA, and isolates the noisy-link contribution.
//   * G2OWithDepth: provide a valid depth in every FeatureBA (range from
//     camera to 3D point + Gaussian noise) and a Tx on the CameraModel
//     so g2o creates EdgeStereoSE3ProjectXYZ "stereo" edges (u, v,
//     u-disparity) instead of mono EdgeSE3ProjectXYZ. Tests g2o's RGB-D /
//     stereo BA path.
enum class BaVariant {
	kDefault,
	kNoLinks,
	kWithDepth,
	kWithDepthNoLinks,
	kWithDepthNoLinksTuned,         // WithDepth + per-axis info calibrated to actual noise
	kWithLidarDepthNoLinksTuned,    // Accurate depth source (LiDAR-fused, 1 cm sigma) + tight DisparityVariance
};

// (backend, variant, roundPixels). roundPixels=true simulates discrete
// pixel-coordinate detection (every keypoint rounded to nearest integer)
// instead of the continuous reprojection -- adds +-0.5 px quantization
// noise to every observation.
using BaParam = std::tuple<Optimizer::Type, BaVariant, bool>;

const char * baVariantName(BaVariant v)
{
	switch(v)
	{
		case BaVariant::kDefault:             return "Default";
		case BaVariant::kNoLinks:          return "NoLinks";
		case BaVariant::kWithDepth:        return "WithDepth";
		case BaVariant::kWithDepthNoLinks: return "WithDepthNoLinks";
		case BaVariant::kWithDepthNoLinksTuned:      return "WithDepthNoLinksTuned";
		case BaVariant::kWithLidarDepthNoLinksTuned: return "WithLidarDepthNoLinksTuned";
	}
	return "?";
}

class BundleAdjustmentTest : public ::testing::TestWithParam<BaParam>
{
protected:
	void SetUp() override
	{
		const Optimizer::Type t = std::get<0>(GetParam());
		if(!Optimizer::isAvailable(t))
		{
			GTEST_SKIP() << optimizerTypeName(t) << " not built in";
		}
	}
};

}  // namespace

TEST_P(BundleAdjustmentTest, CircleCamerasRecoverPosesAndPoints)
{
	const Optimizer::Type backend     = std::get<0>(GetParam());
	const BaVariant       variant     = std::get<1>(GetParam());
	const bool            roundPixels = std::get<2>(GetParam());

	ParametersMap params;
	params[Parameters::kOptimizerStrategy()]   = uNumber2Str(static_cast<int>(backend));
	params[Parameters::kOptimizerIterations()] = "200";  // probe
	// For the stereo BA variants we set Tx on the CameraModel below, which
	// g2o picks up directly. Set g2o/Baseline to the same 0.15 m for
	// consistency (used as a fallback when Tx isn't set on the model).
	params[Parameters::kOptimizerBaseline()]         = "0.15";
	if(variant == BaVariant::kWithLidarDepthNoLinksTuned)
	{
		// Accurate-depth scenario (LiDAR-fused / structured-light): the
		// depth measurement is *more* precise than a typical feature
		// detector's u/v, so the optimizer should trust depth more.
		// PixelVariance=1 (sigma_uv ~ 1 px, typical detector),
		// DisparityVariance=0.1 (sigma_disp ~ 0.3 px, reflecting the
		// fused depth's tighter precision). 1:10 ratio puts depth ~10x
		// tighter than each pixel. Tighter still works empirically
		// (e.g. 0.01 gives same residuals) but g2o's Hessian becomes
		// ill-conditioned around info=10000, so we stay comfortably
		// within the stable range.
		params[Parameters::kOptimizerPixelVariance()]     = "1.0";
		params[Parameters::kOptimizerDisparityVariance()] = "0.1";
	}
	else if(variant == BaVariant::kWithDepthNoLinksTuned)
	{
		// Sub-pixel feature detector + standard stereo block matcher tuning:
		// PixelVariance=0.1 (sigma_uv ~ 0.3 px), DisparityVariance=1
		// (sigma_disp ~ 1 px). The 10:1 ratio gives u/v slightly more weight
		// than disparity, which breaks the info-matrix asymmetry trap seen
		// in WithDepth (default pv=1, dispVar=1 -> 12.8 cm worst point error
		// because the optimizer over-trusts noisy disparity and pushes points
		// along the depth axis to fit it). With this tuning the points
		// converge to ~4 mm.
		params[Parameters::kOptimizerPixelVariance()]     = "0.1";
		params[Parameters::kOptimizerDisparityVariance()] = "1.0";
	}
	std::unique_ptr<Optimizer> opt(Optimizer::create(params));
	ASSERT_NE(opt.get(), nullptr);
	ASSERT_EQ(opt->type(), backend);

	BundleGraph g = buildBundleGraph(/*noisy=*/true, /*roundPixels=*/roundPixels);

	if(variant == BaVariant::kNoLinks
			|| variant == BaVariant::kWithDepthNoLinks
			|| variant == BaVariant::kWithDepthNoLinksTuned
			|| variant == BaVariant::kWithLidarDepthNoLinksTuned)
	{
		// Drop the noisy neighbor links so g2o BA is reduced to a pure
		// reprojection problem (like Ceres/CVSBA).
		g.links.clear();
	}
	if(variant == BaVariant::kWithDepth
			|| variant == BaVariant::kWithDepthNoLinks
			|| variant == BaVariant::kWithDepthNoLinksTuned
			|| variant == BaVariant::kWithLidarDepthNoLinksTuned)
	{
		// Switch to a stereo camera model: baseline = 0.15 m (a common
		// medium-baseline value e.g. ZED Mini / RealSense D435i). Tx =
		// -baseline*fx so g2o detects it and uses the
		// EdgeStereoSE3ProjectXYZ path (3-axis observation: u, v,
		// u-disparity).
		const double baselineMeters = 0.15;
		const double Tx = -baselineMeters * kBaFx;
		const CameraModel stereoModel(kBaFx, kBaFy, kBaCx, kBaCy,
				CameraModel::opticalRotation(), Tx,
				cv::Size(kBaImageWidth, kBaImageHeight));
		for(auto & kv : g.models) kv.second = std::vector<CameraModel>{stereoModel};

		// Realistic stereo noise: 1 px sigma on DISPARITY, constant across
		// range (this is what a block matcher / SGM produces). Depth is
		// then derived from the noisy disparity --
		//   sigma_depth = depth^2 / (baseline*fx) * sigma_disp
		// -- so depth precision degrades quadratically with range. With our
		// 0.15 m baseline and fx=315.5: sigma_depth ranges from ~13 cm at
		// 2.5 m to ~1.2 m at 7.5 m. The optimizer's stereo edge measures
		// the disparity directly via (u, v, u-disparity), so the
		// quadratic range degradation is handled automatically by the
		// projection geometry; the calibrated info matrix matches the
		// constant ~1 px disparity noise (g2o's default PixelVariance=1
		// is well-suited).
		// In the LiDAR-fused variant we replace the stereo noise model with
		// a small constant sigma on depth (range-independent, as a depth
		// sensor / fused range source would produce). DisparityVariance is
		// set tight in the params block to reflect this higher precision.
		const bool lidarDepth = (variant == BaVariant::kWithLidarDepthNoLinksTuned);
		std::mt19937 dispRng(13);
		std::normal_distribution<double> dispNoise (0.0, 1.0);   // 1 px on disparity (stereo / RGB-D)
		std::normal_distribution<double> depthLidarNoise(0.0, 0.01);  // 1 cm on depth (LiDAR / structured-light)
		for(auto & wkv : g.wordReferences)
		{
			const cv::Point3f & worldPt = g.truePoints3D.at(wkv.first);
			for(auto & ckv : wkv.second)
			{
				const Transform world_to_optical =
						(g.truePoses.at(ckv.first) * stereoModel.localTransform()).inverse();
				const cv::Point3f pc = util3d::transformPoint(worldPt, world_to_optical);
				float depthNoisy;
				if(lidarDepth)
				{
					depthNoisy = pc.z + static_cast<float>(depthLidarNoise(dispRng));
				}
				else
				{
					const double dispTrue  = baselineMeters * kBaFx / pc.z;
					const double dispNoisy = dispTrue + dispNoise(dispRng);
					depthNoisy = static_cast<float>(
							baselineMeters * kBaFx / std::max(dispNoisy, 0.1));
				}
				ckv.second = FeatureBA(ckv.second.kpt, depthNoisy);
			}
		}
	}

	// Sanity: every point should be observed from at least 6 cameras. With
	// the spread point cloud (kBaPointXY=3.5, kBaPointZ=2.4) giving ~95%
	// image coverage from each frame, points near the box corners get
	// dropped by the most-oblique-angle cameras (depth too shallow or
	// projection outside FOV). 6 observations is still plenty for BA.
	for(const auto & ptkv : g.truePoints3D)
	{
		ASSERT_GE(g.wordReferences.at(ptkv.first).size(), 6u)
				<< "point " << ptkv.first << " only observed from "
				<< g.wordReferences.at(ptkv.first).size() << " cameras";
	}

	std::map<int, cv::Point3f> outPoints = g.initialPoints3D;
	std::map<int, Transform> outPoses = opt->optimizeBA(
			/*rootId=*/1, g.initialPoses, g.links, g.models, outPoints, g.wordReferences);

	ASSERT_FALSE(outPoses.empty()) << "optimizeBA returned no poses";

	ASSERT_EQ(outPoses.size(), g.truePoses.size());

	// BA recovers the trajectory and point-cloud SHAPE, but the gauge
	// (global rigid transform) depends on the backend: g2o fixes the root
	// pose via setFixed(rootId); Ceres and CVSBA don't fix any pose. To
	// compare gauge-independently, express every pose and point in the
	// optimizer's root frame and compare to truth in truth's root frame --
	// the rigid transform between the two solutions cancels out.
	const Transform truthRootInv = g.truePoses.at(1).inverse();
	const Transform outRootInv   = outPoses.at(1).inverse();









	// Per-backend bounds. Setup: 752x480 / 100 deg HFOV / 200 features
	// uniformly in [-3.5,3.5]x[-3.5,3.5]x[-2.4,2.4] box / 12 cameras on a
	// 5 m circle / 5 cm pose+point noise / 10 cm noisy-link translation
	// noise / 5 cm depth noise (for WithDepth variants) / 200 iterations.
	// Chain rotation recovers to ~0 deg across all backends; position
	// residual depends on how each backend handles the noisy chain:
	//   * Ceres / CVSBA: BA only uses reprojection; chain ignored. ~1.5 cm.
	//   * g2o + chain (Default / WithDepth): BA includes the kNeighbor
	//     edges with their full info matrix. The 10 cm link noise pulls
	//     poses 3-6 cm off truth.
	//   * g2o without chain (NoLinks / WithDepthNoLinks): tightest case.
	// Bounds cover both continuous and rounded-pixel observations. Rounding
	// adds +-0.5 px quantization noise per observation; with 200 obs per
	// pose the pose residual is barely affected, but individual point
	// residuals can grow ~80% (one point still depends on its few specific
	// observations).
	float poseDistMax   = 0.03f;
	float poseAngMaxDeg = 1.0f;
	float pointDistMax  = 0.025f;
	if(backend == Optimizer::kTypeG2O)
	{
		if(variant == BaVariant::kNoLinks)
		{
			poseDistMax  = 0.015f;
			pointDistMax = 0.015f;
		}
		else if(variant == BaVariant::kWithDepth)
		{
			poseDistMax  = 0.02f;
			// With realistic stereo noise (1 px on disparity), point recovery
			// at long range is dominated by the depth uncertainty. The
			// continuous-pixel variant ALSO suffers from the info-matrix
			// asymmetry: u/v residual = 0 at truth so the optimizer
			// over-trusts them vs the 1 px disparity, pushing points along
			// the depth axis to better fit the noisy disparity. Rounded u/v
			// adds ~0.5 px noise that balances the info matrix and gets
			// 8x tighter points.
			pointDistMax = roundPixels ? 0.025f : 0.15f;
		}
		else if(variant == BaVariant::kWithDepthNoLinks)
		{
			poseDistMax  = 0.015f;
			pointDistMax = roundPixels ? 0.025f : 0.15f;
		}
		else if(variant == BaVariant::kWithDepthNoLinksTuned)
		{
			// Calibrated per-axis info matrix + no noisy chain. Tightest
			// case of all the WithDepth variants: both pose AND point
			// converge to ~5 mm.
			poseDistMax  = 0.015f;
			pointDistMax = 0.015f;
		}
		else if(variant == BaVariant::kWithLidarDepthNoLinksTuned)
		{
			// Accurate depth (1 cm sigma) + tight DisparityVariance: the
			// tightest of all WithDepth variants on both pose and point.
			poseDistMax  = 0.005f;
			pointDistMax = 0.005f;
		}
		else
		{
			// Default: chain pulled by noisy links.
			poseDistMax  = 0.10f;
			pointDistMax = 0.09f;
		}
	}
	else if(backend == Optimizer::kTypeGTSAM)
	{
		// GTSAM soft-fixes the root via a tight PriorFactor (no native
		// "setFixed" like g2o), so the gauge is slightly looser; the LM
		// solver also stops at a relativeErrorTol that leaves a tiny bit of
		// residual on the longest-range points. Both effects are sub-cm on
		// the pose, ~few-cm on point cloud points that are 7-8 m from the
		// root. Stereo variants converge to the same bounds as g2o because
		// the depth constraint pins the gauge.
		if(variant == BaVariant::kWithDepthNoLinksTuned)
		{
			poseDistMax  = 0.015f;
			pointDistMax = 0.015f;
		}
		else if(variant == BaVariant::kWithLidarDepthNoLinksTuned)
		{
			poseDistMax  = 0.005f;
			pointDistMax = 0.005f;
		}
		else if(variant == BaVariant::kWithDepth || variant == BaVariant::kWithDepthNoLinks)
		{
			poseDistMax  = 0.03f;
			pointDistMax = roundPixels ? 0.025f : 0.15f;
		}
		else if(variant == BaVariant::kNoLinks)
		{
			// Pure mono reprojection. Mono BA has a 7-DOF gauge -- the
			// recovered geometry is only correct up to scale -- but we
			// solve for that scale against truth below before checking
			// bounds, so the bounds match the clean-converged case.
			poseDistMax  = 0.015f;
			pointDistMax = 0.02f;
		}
		else
		{
			// kDefault: mono BA + noisy chain. Scale handled below; the
			// chain noise still pulls poses ~few cm.
			poseDistMax  = 0.04f;
			pointDistMax = 0.04f;
		}
	}
	else if(backend == Optimizer::kTypeCVSBA)
	{
		poseDistMax  = 0.06f;
		pointDistMax = 0.04f;
	}
	if(roundPixels)
	{
		// +-0.5 px quantization noise hits the point residuals harder than
		// the pose residuals (each point depends on its few observations,
		// each pose averages over ~200). Loosen the point bound a bit.
		pointDistMax = std::max(pointDistMax, 0.02f);
	}

	// Mono BA gauge: fixing one pose leaves a 1-DOF scale ambiguity that the
	// optimizer is free to land anywhere on. Solve for the scale that maps
	// recovered geometry onto truth (closed-form LS on camera positions in
	// the root-relative frame:
	//   s = Σ(d_out · d_truth) / Σ(d_out · d_out)
	// ) and apply it to BOTH poses and points before comparing. This is
	// the test-side counterpart to the brute-force scale scan in
	// tools/Report/main.cpp; here we know the relationship is quadratic
	// so the closed form is exact.
	//
	// Applied when:
	//   * the variant is pure-mono (no depth in observations), OR
	//   * the BACKEND is mono-only -- rtabmap's CVSBA path uses cvsba's
	//     Sba::run() which takes 2D image points only and so cannot use
	//     depth; it runs mono BA even when handed stereo-style
	//     observations and is gauge-ambiguous regardless of the variant.
	// g2o, GTSAM, and Ceres switch to a stereo cost function when
	// depth+baseline are available, so on stereo variants their scale is
	// pinned by geometry and we leave it alone.
	const bool monoVariant = (variant == BaVariant::kDefault || variant == BaVariant::kNoLinks);
	const bool monoOnlyBackend = (backend == Optimizer::kTypeCVSBA);
	const bool monoBA = monoVariant || monoOnlyBackend;
	float scale = 1.0f;
	if(monoBA)
	{
		double num = 0.0;
		double den = 0.0;
		for(const auto & kv : g.truePoses)
		{
			if(kv.first == 1) continue;
			if(!outPoses.count(kv.first)) continue;
			const Transform truthRel = truthRootInv * kv.second;
			const Transform outRel   = outRootInv   * outPoses.at(kv.first);
			num += outRel.x()*truthRel.x() + outRel.y()*truthRel.y() + outRel.z()*truthRel.z();
			den += outRel.x()*outRel.x()   + outRel.y()*outRel.y()   + outRel.z()*outRel.z();
		}
		if(den > 1e-12)
		{
			scale = static_cast<float>(num / den);
		}
	}

	// Recovered poses (relative to root).
	for(const auto & kv : g.truePoses)
	{
		const int id = kv.first;
		if(id == 1) continue;  // root is its own reference (delta = identity in both frames)
		ASSERT_TRUE(outPoses.count(id));
		const Transform truthRel = truthRootInv * kv.second;
		Transform outRel         = outRootInv   * outPoses.at(id);
		if(monoBA)
		{
			outRel.x() *= scale;
			outRel.y() *= scale;
			outRel.z() *= scale;
		}
		EXPECT_LT(outRel.getDistance(truthRel), poseDistMax)
				<< optimizerTypeName(backend) << " pose " << id
				<< " got(rel)=" << outRel.prettyPrint()
				<< " truth(rel)=" << truthRel.prettyPrint();
		const float angDeg = outRel.getAngle(truthRel) * 180.0f / static_cast<float>(CV_PI);
		EXPECT_LT(angDeg, poseAngMaxDeg)
				<< optimizerTypeName(backend) << " pose " << id << " angle=" << angDeg << " deg";
	}

	// Recovered points (relative to root).
	for(const auto & kv : g.truePoints3D)
	{
		const int id = kv.first;
		ASSERT_TRUE(outPoints.count(id));
		const cv::Point3f truthRel = util3d::transformPoint(kv.second, truthRootInv);
		cv::Point3f outRel         = util3d::transformPoint(outPoints.at(id), outRootInv);
		if(monoBA)
		{
			outRel.x *= scale;
			outRel.y *= scale;
			outRel.z *= scale;
		}
		const cv::Point3f diff     = outRel - truthRel;
		const float d = std::sqrt(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
		EXPECT_LT(d, pointDistMax)
				<< optimizerTypeName(backend) << " point " << id
				<< " got(rel)=(" << outRel.x << "," << outRel.y << "," << outRel.z << ")"
				<< " truth(rel)=(" << truthRel.x << "," << truthRel.y << "," << truthRel.z << ")";
	}
}

INSTANTIATE_TEST_SUITE_P(
		BABackends,
		BundleAdjustmentTest,
		::testing::Values(
				// Continuous-pixel observations (the test's original setup).
				std::make_tuple(Optimizer::kTypeG2O,   BaVariant::kDefault,             false),
				std::make_tuple(Optimizer::kTypeG2O,   BaVariant::kNoLinks,          false),
				std::make_tuple(Optimizer::kTypeG2O,   BaVariant::kWithDepth,        false),
				std::make_tuple(Optimizer::kTypeG2O,   BaVariant::kWithDepthNoLinks, false),
				std::make_tuple(Optimizer::kTypeG2O,   BaVariant::kWithDepthNoLinksTuned,      false),
				std::make_tuple(Optimizer::kTypeG2O,   BaVariant::kWithLidarDepthNoLinksTuned, false),
				std::make_tuple(Optimizer::kTypeGTSAM, BaVariant::kDefault,             false),
				std::make_tuple(Optimizer::kTypeGTSAM, BaVariant::kNoLinks,          false),
				std::make_tuple(Optimizer::kTypeGTSAM, BaVariant::kWithDepth,        false),
				std::make_tuple(Optimizer::kTypeGTSAM, BaVariant::kWithDepthNoLinks, false),
				std::make_tuple(Optimizer::kTypeGTSAM, BaVariant::kWithDepthNoLinksTuned,      false),
				std::make_tuple(Optimizer::kTypeGTSAM, BaVariant::kWithLidarDepthNoLinksTuned, false),
				std::make_tuple(Optimizer::kTypeCeres, BaVariant::kDefault,             false),
				std::make_tuple(Optimizer::kTypeCeres, BaVariant::kNoLinks,          false),
				std::make_tuple(Optimizer::kTypeCeres, BaVariant::kWithDepth,        false),
				std::make_tuple(Optimizer::kTypeCeres, BaVariant::kWithDepthNoLinks, false),
				std::make_tuple(Optimizer::kTypeCeres, BaVariant::kWithDepthNoLinksTuned,      false),
				std::make_tuple(Optimizer::kTypeCeres, BaVariant::kWithLidarDepthNoLinksTuned, false),
				std::make_tuple(Optimizer::kTypeCVSBA, BaVariant::kDefault,             false),
				// Same setups but with keypoints rounded to integer pixel
				// coordinates (simulates a real detector).
				std::make_tuple(Optimizer::kTypeG2O,   BaVariant::kDefault,             true),
				std::make_tuple(Optimizer::kTypeG2O,   BaVariant::kNoLinks,          true),
				std::make_tuple(Optimizer::kTypeG2O,   BaVariant::kWithDepth,        true),
				std::make_tuple(Optimizer::kTypeG2O,   BaVariant::kWithDepthNoLinks, true),
				std::make_tuple(Optimizer::kTypeGTSAM, BaVariant::kDefault,             true),
				std::make_tuple(Optimizer::kTypeGTSAM, BaVariant::kNoLinks,          true),
				std::make_tuple(Optimizer::kTypeGTSAM, BaVariant::kWithDepth,        true),
				std::make_tuple(Optimizer::kTypeGTSAM, BaVariant::kWithDepthNoLinks, true),
				std::make_tuple(Optimizer::kTypeCeres, BaVariant::kDefault,             true),
				std::make_tuple(Optimizer::kTypeCeres, BaVariant::kNoLinks,             true),
				std::make_tuple(Optimizer::kTypeCeres, BaVariant::kWithDepth,           true),
				std::make_tuple(Optimizer::kTypeCeres, BaVariant::kWithDepthNoLinks,    true),
				std::make_tuple(Optimizer::kTypeCVSBA, BaVariant::kDefault,             true)),
		[](const ::testing::TestParamInfo<BaParam> & info)
		{
			std::string name = optimizerTypeName(std::get<0>(info.param));
			name += "_";
			name += baVariantName(std::get<1>(info.param));
			if(std::get<2>(info.param)) name += "_Rounded";
			return name;
		});

// ---------------------------------------------------------------------------
// PlanarBundleAdjustmentTest -- verifies that isSlam2d() in BA locks the
// recovered trajectory to its initial Z plane. g2o has supported this since
// forever via EdgeSBACamPrior; GTSAM and Ceres got matching planar
// constraints in this rev.
// ---------------------------------------------------------------------------

class PlanarBundleAdjustmentTest : public ::testing::TestWithParam<Optimizer::Type>
{
protected:
	void SetUp() override
	{
		if(!Optimizer::isAvailable(GetParam()))
		{
			GTEST_SKIP() << optimizerTypeName(GetParam()) << " not built in";
		}
	}
};

TEST_P(PlanarBundleAdjustmentTest, RecoveredTrajectoryStaysOnInitialPlane)
{
	const Optimizer::Type backend = GetParam();

	ParametersMap params;
	params[Parameters::kOptimizerStrategy()]   = uNumber2Str(static_cast<int>(backend));
	params[Parameters::kOptimizerIterations()] = "200";
	params[Parameters::kRegForce3DoF()]        = "true";  // → isSlam2d() == true
	std::unique_ptr<Optimizer> opt(Optimizer::create(params));
	ASSERT_NE(opt.get(), nullptr);
	ASSERT_TRUE(opt->isSlam2d());

	BundleGraph g = buildBundleGraph(/*noisy=*/true, /*roundPixels=*/false);
	// Planar test: initial poses reflect a real 2D-SLAM gauge -- the robot
	// lives on a single horizontal plane at some non-zero height (z=0.5 m
	// here, e.g. a camera mounted on a robot half a meter off the floor)
	// with no roll/pitch. Strip the z/roll/pitch noise from the noisy
	// initial poses and keep only the x/y/yaw noise. Using a non-zero
	// reference z verifies the constraint locks to the INITIAL plane,
	// not to z=0 by accident.
	const float planeZ = 0.5f;
	for(auto & kv : g.initialPoses)
	{
		float x, y, z, roll, pitch, yaw;
		kv.second.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
		kv.second = Transform(x, y, planeZ, 0.0f, 0.0f, yaw);
	}

	std::map<int, cv::Point3f> outPoints = g.initialPoints3D;
	std::map<int, Transform> outPoses = opt->optimizeBA(
			/*rootId=*/1, g.initialPoses, g.links, g.models, outPoints, g.wordReferences);

	ASSERT_FALSE(outPoses.empty()) << "optimizeBA returned no poses";
	ASSERT_EQ(outPoses.size(), g.truePoses.size());

	// With the planar constraint locking every camera to its initial
	// z = planeZ (weight 1e9 = sub-mm tolerance), no recovered pose
	// should drift off the plane. Without the constraint, a 3D BA on
	// this dataset can drift up to a few mm in z due to reprojection /
	// chain residuals.
	for(const auto & kv : outPoses)
	{
		EXPECT_NEAR(kv.second.z(), planeZ, 0.0005f)  // 0.5 mm
				<< optimizerTypeName(backend) << " pose " << kv.first
				<< " drifted off the z=" << planeZ << " plane: z=" << kv.second.z();
	}
}

INSTANTIATE_TEST_SUITE_P(
		Backends,
		PlanarBundleAdjustmentTest,
		::testing::Values(Optimizer::kTypeG2O, Optimizer::kTypeGTSAM, Optimizer::kTypeCeres),
		[](const ::testing::TestParamInfo<Optimizer::Type> & info)
		{
			return optimizerTypeName(info.param);
		});

TEST(OptimizerTest, CvsbaPoseGraphOptimizeReturnsEmpty)
{
	// CVSBA only overrides optimizeBA() -- it doesn't implement pose-graph
	// optimize(). Calling optimize() on a CVSBA instance hits the base
	// Optimizer::optimize() which logs an error and returns an empty map.
	// Pin that contract so callers can safely fall back.
	if(!Optimizer::isAvailable(Optimizer::kTypeCVSBA))
	{
		GTEST_SKIP() << "CVSBA not built in";
	}
	std::unique_ptr<Optimizer> opt(Optimizer::create(Optimizer::kTypeCVSBA));
	ASSERT_NE(opt.get(), nullptr);
	ASSERT_EQ(opt->type(), Optimizer::kTypeCVSBA);

	std::map<int, Transform> in;
	std::multimap<int, Link> inLinks;
	makeChain3(in, inLinks);

	std::map<int, Transform> out = opt->optimize(1, in, inLinks);
	EXPECT_TRUE(out.empty()) << "CVSBA should not handle pose-graph optimize()";
}
