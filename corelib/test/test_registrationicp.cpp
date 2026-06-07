// Tests for RegistrationIcp -- geometric (laser scan) registration via ICP.
//
// We feed the registrar a deterministic synthetic cloud of 50 tight Gaussian
// blobs scattered in [-1,1]^3, apply a known transform to one copy, and verify
// the recovered transform matches truth. The clustered structure gives each
// blob a distinctive local neighbourhood ICP's nearest-neighbour search can
// lock onto -- crucial for CCCoreLib, whose KD-tree-only correspondence
// pass loses lock on uniform random clouds past ~7 deg of misalignment.
//
// Recovery / filter / constraint tests are parameterized across every ICP
// backend built into the binary (PCL always; libpointmatcher and CCCoreLib
// when their build flags are set). Plumbing tests stay non-parameterized.

#include <gtest/gtest.h>

#include <rtabmap/core/RegistrationIcp.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UConversion.h>

#include <opencv2/core.hpp>

#include <cmath>
#include <vector>

using namespace rtabmap;

namespace {

// Iterate every IcpStrategy enum value the library exposes and keep the ones
// this build actually links in.  Lets tests run once per available backend
// without per-CI #ifdef gymnastics in the test body.
std::vector<RegistrationIcp::IcpStrategy> availableIcpStrategies()
{
	std::vector<RegistrationIcp::IcpStrategy> s;
	for(int i = RegistrationIcp::kIcpPCL; i < RegistrationIcp::kIcpEnd; ++i)
	{
		const auto strategy = static_cast<RegistrationIcp::IcpStrategy>(i);
		if(RegistrationIcp::available(strategy)) s.push_back(strategy);
	}
	return s;
}

// 50 tight Gaussian blobs scattered through [-1,1]^3, 100 points each (5000
// total).  Each blob is a distinctive local feature ICP's nearest-neighbour
// search can lock onto, so correspondences stay correct across larger initial
// misalignments than a uniform cloud allows.  Tested with all three backends
// (PCL / libpointmatcher / CCCoreLib)
cv::Mat makeRandomCloud(int numBlobs = 50, int pointsPerBlob = 100, float blobSigma = 0.04f, uint64_t seed = 0xC0FFEE)
{
	cv::RNG rng(seed);
	cv::Mat data(1, numBlobs * pointsPerBlob, CV_32FC3);
	int idx = 0;
	for(int c = 0; c < numBlobs; ++c)
	{
		const cv::Vec3f centre(
				rng.uniform(-0.9f, 0.9f),
				rng.uniform(-0.9f, 0.9f),
				rng.uniform(-0.9f, 0.9f));
		for(int i = 0; i < pointsPerBlob; ++i, ++idx)
		{
			data.at<cv::Vec3f>(0, idx) = cv::Vec3f(
					centre[0] + static_cast<float>(rng.gaussian(blobSigma)),
					centre[1] + static_cast<float>(rng.gaussian(blobSigma)),
					centre[2] + static_cast<float>(rng.gaussian(blobSigma)));
		}
	}
	return data;
}

LaserScan makeScan(const cv::Mat & scanData)
{
	return LaserScan(scanData, /*maxPoints=*/0, /*maxRange=*/0.0f, LaserScan::kXYZ);
}

// 2D laser-scan version of @ref makeRandomCloud (kXY format).  Uses smaller
// tighter blobs than the 3D variant -- losing the z dimension halves the
// effective spatial separation between cluster centroids, so we counter with
// more, smaller clusters so libpointmatcher and CCCoreLib still find
// distinctive correspondences from an identity guess.
LaserScan makeRandom2DScan(int numBlobs = 100, int pointsPerBlob = 50, float blobSigma = 0.01f, uint64_t seed = 0xC0FFEE)
{
	cv::RNG rng(seed);
	cv::Mat data(1, numBlobs * pointsPerBlob, CV_32FC2);
	int idx = 0;
	for(int c = 0; c < numBlobs; ++c)
	{
		const cv::Vec2f centre(
				rng.uniform(-0.9f, 0.9f),
				rng.uniform(-0.9f, 0.9f));
		for(int i = 0; i < pointsPerBlob; ++i, ++idx)
		{
			data.at<cv::Vec2f>(0, idx) = cv::Vec2f(
					centre[0] + static_cast<float>(rng.gaussian(blobSigma)),
					centre[1] + static_cast<float>(rng.gaussian(blobSigma)));
		}
	}
	return LaserScan(data, /*maxPoints=*/0, /*maxRange=*/0.0f, LaserScan::kXY);
}

// 3D corridor: two side walls at y=+/-halfWidth, plus floor (z=0) and
// ceiling (z=height).  All surfaces extend along the x axis from
// -length/2..+length/2.  Walls carry a small y-jitter so the per-point
// normals aren't perfectly colinear (otherwise the normal-PCA's smallest
// eigenvalue rounds to 0 and the low-complexity strategy can't pick a
// projection axis).  The corridor is intentionally degenerate along x:
// every surface maps onto itself under a pure-x translation, so ICP can't
// recover x without an external prior.
//
// With all four surfaces equally dense, structural complexity lands around
// ~0.015 -- still under Icp/PointToPlaneMinComplexity=0.02, so the
// low-complexity strategy actively fires.
cv::Mat makeCorridor3D(
		float length = 6.0f,
		float halfWidth = 0.5f,
		float height = 2.0f,
		int wallPoints = 500,
		int floorPoints = 500,
		int ceilingPoints = 500,
		float surfaceJitter = 0.005f,
		uint64_t seed = 0xC0FFEE,
		float floorZ = -0.3f)
{
	cv::RNG rng(seed);
	const int total = 2 * wallPoints + floorPoints + ceilingPoints;
	cv::Mat data(1, total, CV_32FC3);
	int idx = 0;
	const float halfL = 0.5f * length;
	const float ceilingZ = floorZ + height;
	auto wall = [&](float yBase) {
		for(int i = 0; i < wallPoints; ++i, ++idx)
		{
			const float x = rng.uniform(-halfL, halfL);
			const float y = yBase + static_cast<float>(rng.gaussian(surfaceJitter));
			data.at<cv::Vec3f>(0, idx) = cv::Vec3f(x, y, rng.uniform(floorZ, ceilingZ));
		}
	};
	wall(-halfWidth);
	wall( halfWidth);
	auto horizontalSurface = [&](float zBase) {
		for(int i = 0; i < floorPoints; ++i, ++idx)
		{
			data.at<cv::Vec3f>(0, idx) = cv::Vec3f(
					rng.uniform(-halfL, halfL),
					rng.uniform(-halfWidth, halfWidth),
					zBase + static_cast<float>(rng.gaussian(surfaceJitter)));
		}
	};
	horizontalSurface(floorZ);     // floor below origin so viewpoint is inside corridor
	horizontalSurface(ceilingZ);   // ceiling above origin
	return data;
}

// 3D room corner: three mutually perpendicular planes (floor + 2 walls),
// centered around the origin so the viewpoint at origin sits inside the
// corner volume (floor below, walls behind/left).
//   Floor:  z=-half, normal +z, x/y in [-half, half].
//   Wall x: x=-half, normal +x, y/z in [-half, half].
//   Wall y: y=-half, normal +y, x/z in [-half, half].
// Each surface carries Gaussian jitter normal to the plane so per-point
// normals are well defined. With Icp/PointToPlaneComplexityCentered=false
// (default), the uncentered second-moment metric correctly identifies all
// 3 DoF as constrained -- complexity lands well above the 0.02 threshold
// and the low-complexity fallback does NOT fire.
cv::Mat makeRoomCorner3D(
		float length = 2.0f,
		int pointsPerSurface = 500,
		float surfaceJitter = 0.005f,
		uint64_t seed = 0xC0FFEE)
{
	cv::RNG rng(seed);
	cv::Mat data(1, 3 * pointsPerSurface, CV_32FC3);
	int idx = 0;
	const float half = 0.5f * length;
	// Floor: z=-half, jitter on z.
	for(int i = 0; i < pointsPerSurface; ++i, ++idx)
	{
		data.at<cv::Vec3f>(0, idx) = cv::Vec3f(
				rng.uniform(-half, half),
				rng.uniform(-half, half),
				-half + static_cast<float>(rng.gaussian(surfaceJitter)));
	}
	// Wall at x=-half: jitter on x.
	for(int i = 0; i < pointsPerSurface; ++i, ++idx)
	{
		data.at<cv::Vec3f>(0, idx) = cv::Vec3f(
				-half + static_cast<float>(rng.gaussian(surfaceJitter)),
				rng.uniform(-half, half),
				rng.uniform(-half, half));
	}
	// Wall at y=-half: jitter on y.
	for(int i = 0; i < pointsPerSurface; ++i, ++idx)
	{
		data.at<cv::Vec3f>(0, idx) = cv::Vec3f(
				rng.uniform(-half, half),
				-half + static_cast<float>(rng.gaussian(surfaceJitter)),
				rng.uniform(-half, half));
	}
	return data;
}

// 2D corridor: two parallel lines at y=+/-halfWidth extending along x. Same
// degeneracy as the 3D version -- pure-x translation maps each line onto
// itself, so x is unobservable without a guess.
//
// Note: pointsPerLine of perfectly straight points along y=const give all
// normals (0, +/-1) exactly -- PCA on these has zero variance in any
// direction except y, so structural complexity rounds to zero and the
// low-complexity strategy can't decide a projection axis. A 5 mm y-jitter
// breaks the colinearity enough for the normal-PCA to compute meaningful
// eigenvectors while keeping the complexity at ~0.003 (well below the 0.02
// threshold, so the low-complexity strategy still fires).
LaserScan makeCorridor2D(
		float length = 6.0f,
		float halfWidth = 0.5f,
		float yJitter = 0.005f,
		int pointsPerLine = 500,
		uint64_t seed = 0xC0FFEE)
{
	cv::RNG rng(seed);
	cv::Mat data(1, pointsPerLine * 2, CV_32FC2);
	int idx = 0;
	const float halfL = 0.5f * length;
	for(int s = 0; s < 2; ++s)
	{
		const float yBase = (s == 0 ? -halfWidth : halfWidth);
		for(int i = 0; i < pointsPerLine; ++i, ++idx)
		{
			data.at<cv::Vec2f>(0, idx) = cv::Vec2f(
					rng.uniform(-halfL, halfL),
					yBase + rng.uniform(-yJitter, yJitter));
		}
	}
	return LaserScan(data, /*maxPoints=*/0, /*maxRange=*/0.0f, LaserScan::kXY);
}

// 2D room corner: two perpendicular line segments centered around the
// origin so the viewpoint at origin sits inside the L-shape volume.
//   Floor: y=-half, normal +y, x in [-half, half].
//   Wall:  x=-half, normal +x, y in [-half, half].
// With Icp/PointToPlaneComplexityCentered=false (default), the uncentered
// second-moment metric correctly identifies both DoF as constrained --
// complexity lands well above the 0.02 threshold.
LaserScan makeRoomCorner2D(
		float length = 2.0f,
		float jitter = 0.005f,
		int pointsPerLine = 500,
		uint64_t seed = 0xC0FFEE)
{
	cv::RNG rng(seed);
	cv::Mat data(1, 2 * pointsPerLine, CV_32FC2);
	int idx = 0;
	const float half = 0.5f * length;
	// Floor: y=-half, jitter on y.
	for(int i = 0; i < pointsPerLine; ++i, ++idx)
	{
		data.at<cv::Vec2f>(0, idx) = cv::Vec2f(
				rng.uniform(-half, half),
				-half + rng.uniform(-jitter, jitter));
	}
	// Wall: x=-half, jitter on x.
	for(int i = 0; i < pointsPerLine; ++i, ++idx)
	{
		data.at<cv::Vec2f>(0, idx) = cv::Vec2f(
				-half + rng.uniform(-jitter, jitter),
				rng.uniform(-half, half));
	}
	return LaserScan(data, /*maxPoints=*/0, /*maxRange=*/0.0f, LaserScan::kXY);
}

Signature makeSignature(int id, const LaserScan & scan)
{
	SensorData data;
	data.setId(id);
	data.setLaserScan(scan);
	return Signature(data);
}

ParametersMap baseIcpParams(RegistrationIcp::IcpStrategy strategy)
{
	ParametersMap p;
	p.insert(ParametersPair(Parameters::kIcpStrategy(), uNumber2Str(static_cast<int>(strategy))));
	// Epsilon=1e-3 (1 mm-equivalent) so the differential-convergence stop
	// condition can actually fire on our pristine synthetic clouds.
	// 1e-6 squared becomes 1e-12 inside libpointmatcher's
	// DifferentialTransformationChecker -- effectively never trips, every
	// run burns the full iteration budget.
	p.insert(ParametersPair(Parameters::kIcpEpsilon(), "1e-3"));
	p.insert(ParametersPair(Parameters::kIcpIterations(), "50"));
	// Leave Icp/OutlierRatio at its default (0.85). Setting it to 0.0 (to
	// "disable" the PCL RANSAC rejector) also disables libpointmatcher's
	// TrimmedDistOutlierFilter, which rejects ratios <= 0 with an exception.
	// 0.5 m max correspondence distance keeps a comfortable margin over the
	// largest motion we test (10 cm + 10 deg ~= 0.2 m max point displacement
	// over the 2 m cloud diameter).
	p.insert(ParametersPair(Parameters::kIcpMaxCorrespondenceDistance(), "0.5"));
	// CCCoreLib has its own RMS-based rejection threshold (Icp/CCMaxFinalRMS,
	// default 0.2). Bump it so synthetic clouds with 5-10 cm motion don't
	// trip the rejector before the test's own MaxTranslation/MaxRotation
	// gates get a chance.
	p.insert(ParametersPair(Parameters::kIcpCCMaxFinalRMS(), "5.0"));
	// Disable point-to-plane by default: our random blob clouds aren't
	// surfaces, so normal estimation triggers the low-complexity fallback
	// path that can zero out translation components. PointToPlane stays
	// available for the test that opts in explicitly.
	p.insert(ParametersPair(Parameters::kIcpPointToPlane(), "false"));
	// Force RegistrationIcp to its standalone path (no child).
	p.insert(ParametersPair(Parameters::kRegRepeatOnce(), "false"));
	p.insert(ParametersPair(Parameters::kRegForce3DoF(), "false"));
	return p;
}

// Default strategy for non-parameterized plumbing tests: prefer
// libpointmatcher if built, then CCCoreLib, then PCL.
RegistrationIcp::IcpStrategy defaultIcpStrategy()
{
	if(RegistrationIcp::available(RegistrationIcp::kIcpPointMatcher)) return RegistrationIcp::kIcpPointMatcher;
	if(RegistrationIcp::available(RegistrationIcp::kIcpCCCoreLib))    return RegistrationIcp::kIcpCCCoreLib;
	return RegistrationIcp::kIcpPCL;
}

Transform runIcp(
		RegistrationIcp::IcpStrategy strategy,
		const LaserScan & fromScan,
		const LaserScan & toScan,
		const ParametersMap & extra = ParametersMap(),
		const Transform & guess = Transform::getIdentity(),
		RegistrationInfo * infoOut = nullptr)
{
	ParametersMap params = baseIcpParams(strategy);
	for(const auto & kv : extra) params[kv.first] = kv.second;
	RegistrationIcp reg(params);
	Signature from = makeSignature(1, fromScan);
	Signature to   = makeSignature(2, toScan);
	RegistrationInfo info;
	const Transform t = reg.computeTransformation(from, to, guess, &info);
	if(infoOut) *infoOut = info;
	return t;
}

// Per-backend recovery tolerance.  PCL and libpointmatcher converge to
// sub-mm / sub-mrad on the clustered cloud; CCCoreLib runs ~2x looser per
// axis on the same data because its KD-tree-only correspondence pass can't
// average out as much noise as the PM matcher chain or PCL's RANSAC reject.
float recoveryTol(RegistrationIcp::IcpStrategy s)
{
	return s == RegistrationIcp::kIcpCCCoreLib ? 1e-2f : 5e-3f;
}

// Parameterized fixture: each test instance runs once per available backend.
class RegistrationIcpStrategyTest : public ::testing::TestWithParam<RegistrationIcp::IcpStrategy>
{
protected:
	RegistrationIcp::IcpStrategy strategy() const { return GetParam(); }
	float tol() const { return recoveryTol(strategy()); }
};

}  // namespace

// =====================================================================
// IcpStrategy-agnostic plumbing tests (single backend, default strategy)
// =====================================================================

TEST(RegistrationIcpTest, FlagsAdvertiseScanRequiredAndGuessSupport)
{
	RegistrationIcp reg(baseIcpParams(defaultIcpStrategy()));
	EXPECT_TRUE(reg.isScanRequired());
	EXPECT_FALSE(reg.isImageRequired());
	EXPECT_TRUE(reg.canUseGuess());
}

TEST(RegistrationIcpTest, ParseParametersOverridesDefaults)
{
	ParametersMap p = baseIcpParams(defaultIcpStrategy());
	p[Parameters::kRegForce3DoF()]      = "true";
	p[Parameters::kIcpMaxTranslation()] = "0.5";
	p[Parameters::kIcpMaxRotation()]    = "1.5";

	RegistrationIcp reg(p);
	EXPECT_TRUE(reg.force3DoF());

	// Re-parse with new values.
	ParametersMap update;
	update[Parameters::kRegForce3DoF()] = "false";
	reg.parseParameters(update);
	EXPECT_FALSE(reg.force3DoF());
}

// =====================================================================
// Corridor degeneracy: low-complexity detection + recovery strategy
//
// In a corridor (long, parallel walls), translation along the corridor axis
// is geometrically unobservable -- any pure-x translation maps the scan
// onto itself.  rtabmap detects this via @ref Icp/PointToPlaneMinComplexity
// and falls back to a strategy controlled by @ref
// Icp/PointToPlaneLowComplexityStrategy.  These tests exercise Strategy 3
// (keep PointToPlane + project onto constrained axes), which yields cleaner
// y/yaw recovery than Strategy 1 (the default, legacy: recompute with
// PointToPoint then project) on these synthetic clouds.
//
// These tests use the default ICP backend only -- the low-complexity logic
// lives in RegistrationIcp itself, so the backend underneath isn't what's
// under test.
// =====================================================================

namespace {

// Helper for the corridor tests: PointToPlane=true with normals auto-computed
// from a k-neighborhood; low-complexity strategy 3 (keep PointToPlane + project
// along the degenerate axis); 5 cm voxel downsampling on both scans before ICP.
ParametersMap corridorIcpParams(RegistrationIcp::IcpStrategy strategy)
{
	ParametersMap p = baseIcpParams(strategy);
	p[Parameters::kRegForce3DoF()] = "true";
	p[Parameters::kIcpPointToPlane()] = "true";
	// PCL's normal estimator wants either K or radius, not both. Use a
	// radius search (0.3 m -- about half the corridor width) so wall
	// normals stay locally consistent even when the cloud is sparse.
	p[Parameters::kIcpPointToPlaneK()] = "0";
	p[Parameters::kIcpPointToPlaneRadius()] = "0.3";
	// Default MinComplexity = 0.02 already detects a corridor; keep it
	// explicit so the test documents what threshold is being exercised.
	p[Parameters::kIcpPointToPlaneMinComplexity()] = "0.02";
	// Strategy 3 (keep PointToPlane + project): PointToPlane's normal-dot
	// residual gives clean y/yaw recovery on the synthetic corridor walls.
	// The default Strategy 1 (recompute with PointToPoint + project) is
	// more robust on real-world F2M drift (where libpointmatcher iteration
	// can wander on degenerate scans -- see the PR2_Scan2D_Corridor
	// integration test), but on these clean clouds PointToPoint loses
	// signal on the y axis and yaw collapses toward zero (~0.5 deg out of
	// 3 deg). So this helper opts back into Strategy 3.
	p[Parameters::kIcpPointToPlaneLowComplexityStrategy()] = "3";
	// 5 cm voxel: downsamples the dense synthetic walls while keeping enough
	// points to estimate normals + run ICP -- closer to a real-world setup.
	p[Parameters::kIcpVoxelSize()] = "0.05";
	// 0.5 m matches the integration-test PR2 corridor configuration so this
	// test exercises libpointmatcher's BoundTransformationChecker. The
	// checker measures the iteration's *delta from the initial guess*, not
	// the absolute transform, so even the with-guess tests (xGuess at 1 m
	// forward) only need ~5 cm of correction and never trip the bound.
	p[Parameters::kIcpMaxTranslation()] = "0.5";
	return p;
}

// Corridor tests are inherently about the PointToPlane + low-complexity
// strategy path; CCCoreLib doesn't expose PointToPlane in rtabmap's wrapper
// so it's excluded from this fixture.

// PointToPlane-capable backends -- everything except CCCoreLib among the
// available strategies in this build.
std::vector<RegistrationIcp::IcpStrategy> pointToPlaneCapableStrategies()
{
	std::vector<RegistrationIcp::IcpStrategy> s;
	for(auto strategy : availableIcpStrategies())
	{
		if(strategy != RegistrationIcp::kIcpCCCoreLib) s.push_back(strategy);
	}
	return s;
}

// Corridor tests model an "infinite" corridor: the truth motion is the
// actual robot motion in the world, but x is unobservable -- the corridor
// looks statistically identical 1 m forward.  We strip the x component
// from truth when transforming the `to` scan so ICP only ever sees the
// observable subset of the motion.
Transform observableMotion(const Transform & truth)
{
	float roll, pitch, yaw;
	truth.getEulerAngles(roll, pitch, yaw);
	return Transform(0.0f, truth.y(), truth.z(), roll, pitch, yaw);
}

// Parameterized fixture for the corridor degeneracy tests.
class CorridorTest : public ::testing::TestWithParam<RegistrationIcp::IcpStrategy>
{
protected:
	RegistrationIcp::IcpStrategy strategy() const { return GetParam(); }
};

}  // namespace

TEST_P(CorridorTest, NoGuess_FailsToRecoverXButGetsYAndYaw)
{
	// Corridor along x. Truth: 1 m forward, 5 cm sideways, 3 deg yaw.
	// Infinite-corridor model: `to` is an independent sample of the same
	// corridor shape, transformed by only the observable subset of truth
	// (x stripped). With identity guess, ICP can't observe the x motion;
	// strategy=1 (PCL / libpointmatcher) pins x at the guess.
	const LaserScan from = makeScan(makeCorridor3D());
	const float yaw = 3.0f * static_cast<float>(M_PI) / 180.0f;
	const Transform truth(1.0f, 0.05f, 0.0f, 0.0f, 0.0f, yaw);
	const LaserScan to = util3d::transformLaserScan(
			makeScan(makeCorridor3D(/*length=*/6.0f, /*halfWidth=*/0.5f,
					/*height=*/2.0f, /*wallPoints=*/500, /*floorPoints=*/500,
					/*ceilingPoints=*/500, /*surfaceJitter=*/0.005f,
					/*seed=*/0xBA5EBA11)),
			observableMotion(truth).inverse());

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, corridorIcpParams(strategy()),
			Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(info.icpStructuralComplexity, 0.02f)
			<< "expected corridor to land below low-complexity threshold; got "
			<< info.icpStructuralComplexity;

	float x, y, z, roll, pitch, yawOut;
	t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yawOut);
	EXPECT_LT(std::abs(x), 0.01f)
			<< "expected x to stay near guess (=0), got " << x;
	EXPECT_GT(std::abs(x - truth.x()), 0.5f)
			<< "expected x to be way off truth; got " << x;
	EXPECT_NEAR(0.05f, y, 5e-2f);
	EXPECT_NEAR(yaw, yawOut, 5e-2f);
}

TEST_P(CorridorTest, WithXGuess_RecoversFullTransform)
{
	// Same corridor + truth, but seed ICP with a guess that supplies the
	// x translation (and nothing else). Both backends recover the full
	// transform when x is anchored by the guess.
	const LaserScan from = makeScan(makeCorridor3D());
	const float yaw = 3.0f * static_cast<float>(M_PI) / 180.0f;
	const Transform truth(1.0f, 0.05f, 0.0f, 0.0f, 0.0f, yaw);
	const LaserScan to = util3d::transformLaserScan(
			makeScan(makeCorridor3D(/*length=*/6.0f, /*halfWidth=*/0.5f,
					/*height=*/2.0f, /*wallPoints=*/500, /*floorPoints=*/500,
					/*ceilingPoints=*/500, /*surfaceJitter=*/0.005f,
					/*seed=*/0xBA5EBA11)),
			observableMotion(truth).inverse());

	const Transform xGuess(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, corridorIcpParams(strategy()),
			xGuess, &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(t.getDistance(truth), 2e-2f)
			<< "got=" << t.prettyPrint() << " truth=" << truth.prettyPrint();
	EXPECT_LT(t.getAngle(truth), 1e-2f);
}

// Helper for the drone tests: same as corridorIcpParams() but full 6DoF
// (no Reg/Force3DoF).
ParametersMap corridorIcpParams6DoF(RegistrationIcp::IcpStrategy strategy)
{
	ParametersMap p = corridorIcpParams(strategy);
	p[Parameters::kRegForce3DoF()] = "false";
	return p;
}

TEST_P(CorridorTest, Drone_NoGuess_FailsToRecoverXOnly)
{
	// Simulate a drone moving in an "infinite" corridor: the truth motion
	// is 6DoF, but the corridor looks statistically identical 1 m forward
	// so the robot only sees the y/z/roll/pitch/yaw components. `from` and
	// `to` are independent samples of the same corridor (different seeds),
	// and `to` is transformed by only the observable subset of truth. ICP
	// cannot recover x from this input.
	const LaserScan from = makeScan(makeCorridor3D());
	const float deg = static_cast<float>(M_PI) / 180.0f;
	const Transform truth(1.0f, 0.05f, 0.05f, 2.0f*deg, 1.5f*deg, 3.0f*deg);
	const LaserScan to = util3d::transformLaserScan(
			makeScan(makeCorridor3D(/*length=*/6.0f, /*halfWidth=*/0.5f,
					/*height=*/2.0f, /*wallPoints=*/500, /*floorPoints=*/500,
					/*ceilingPoints=*/500, /*surfaceJitter=*/0.005f,
					/*seed=*/0xBA5EBA11)),
			observableMotion(truth).inverse());

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, corridorIcpParams6DoF(strategy()),
			Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(info.icpStructuralComplexity, 0.02f)
			<< "expected corridor to land below low-complexity threshold; got "
			<< info.icpStructuralComplexity;

	float x, y, z, roll, pitch, yawOut;
	t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yawOut);
	EXPECT_LT(std::abs(x), 0.01f)
			<< "expected x to stay near guess (=0), got " << x;
	EXPECT_GT(std::abs(x - truth.x()), 0.5f)
			<< "expected x to be way off truth; got " << x;
	// Only the *perpendicular* DoFs (z, pitch) lock reliably without
	// point-to-point correspondences -- they're constrained by direct
	// floor/ceiling-normal residuals.  y, roll, yaw are all in-plane
	// motions for the dense floor/ceiling, where PointToPlane gives zero
	// residual; without an x-anchor they drift.  The with-guess test
	// below shows the same setup recovers fully once x is provided.
	EXPECT_NEAR(0.05f, z, 1e-2f);
	EXPECT_NEAR(1.5f*deg, pitch, 5e-3f);
	// y, roll, yaw: only require they don't blow up.
	EXPECT_LT(std::abs(y),      0.1f);
	EXPECT_LT(std::abs(roll),   5.0f*deg);
	EXPECT_LT(std::abs(yawOut), 5.0f*deg);
}

TEST_P(CorridorTest, Drone_WithXGuess_RecoversFullTransform)
{
	// Same infinite-corridor setup, guess provides only the 1 m
	// x-translation. The guess equals truth.x, so all three backends
	// converge near truth on the remaining 5 DoFs (CC looser since it
	// can't run PointToPlane).
	const LaserScan from = makeScan(makeCorridor3D());
	const float deg = static_cast<float>(M_PI) / 180.0f;
	const Transform truth(1.0f, 0.05f, 0.05f, 2.0f*deg, 1.5f*deg, 3.0f*deg);
	const LaserScan to = util3d::transformLaserScan(
			makeScan(makeCorridor3D(/*length=*/6.0f, /*halfWidth=*/0.5f,
					/*height=*/2.0f, /*wallPoints=*/500, /*floorPoints=*/500,
					/*ceilingPoints=*/500, /*surfaceJitter=*/0.005f,
					/*seed=*/0xBA5EBA11)),
			observableMotion(truth).inverse());

	const Transform xGuess(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, corridorIcpParams6DoF(strategy()),
			xGuess, &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(t.getDistance(truth), 2e-2f)
			<< "got=" << t.prettyPrint() << " truth=" << truth.prettyPrint();
	EXPECT_LT(t.getAngle(truth), 1e-2f);
}

// Force4DoF variants of the drone tests. libpointmatcher only -- it's the
// only backend that exposes a 4DoF PointToPlane minimiser via its force4DOF
// parameter. PCL has no Force4DoF support (RegistrationIcp warns + falls
// back), and CCCoreLib's PointToPoint path doesn't run on PointToPlane.
//
// Models a moving drone with a perfect IMU: the drone really is rolled
// and pitched in the world (full 6DoF truth applied to the `to` scan),
// and the IMU feeds the known world roll/pitch into the guess. Force4DoF
// preserves the guess's roll/pitch (no delta updates to those axes) and
// refines only the (x, y, z, yaw) residual on top, so ICP recovers the
// full 6DoF truth.
TEST(RegistrationIcpTest, Corridor3D_Drone4DoF_ImuOnlyGuess_FailsToRecoverXOnly)
{
	if(!RegistrationIcp::available(RegistrationIcp::kIcpPointMatcher))
	{
		GTEST_SKIP() << "Force4DoF requires libpointmatcher";
	}
	const LaserScan from = makeScan(makeCorridor3D());
	const float deg = static_cast<float>(M_PI) / 180.0f;
	const Transform truth(1.0f, 0.05f, 0.05f, 2.0f*deg, 1.5f*deg, 3.0f*deg);
	const LaserScan to = util3d::transformLaserScan(
			makeScan(makeCorridor3D(/*length=*/6.0f, /*halfWidth=*/0.5f,
					/*height=*/2.0f, /*wallPoints=*/500, /*floorPoints=*/500,
					/*ceilingPoints=*/500, /*surfaceJitter=*/0.005f,
					/*seed=*/0xBA5EBA11)),
			observableMotion(truth).inverse());

	ParametersMap params = corridorIcpParams6DoF(RegistrationIcp::kIcpPointMatcher);
	params[Parameters::kIcpForce4DoF()] = "true";
	// IMU-only guess: world roll/pitch known, no odometry x.
	const Transform imuGuess(0.0f, 0.0f, 0.0f, 2.0f*deg, 1.5f*deg, 0.0f);
	RegistrationInfo info;
	const Transform t = runIcp(RegistrationIcp::kIcpPointMatcher, from, to, params,
			imuGuess, &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(info.icpStructuralComplexity, 0.02f)
			<< "expected corridor to land below low-complexity threshold; got "
			<< info.icpStructuralComplexity;

	float x, y, z, roll, pitch, yawOut;
	t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yawOut);
	EXPECT_LT(std::abs(x), 0.01f)
			<< "expected x to stay near guess (=0), got " << x;
	EXPECT_GT(std::abs(x - truth.x()), 0.5f)
			<< "expected x to be way off truth; got " << x;
	// Force4DoF preserves the guess's roll/pitch (matching the IMU-known
	// world tilt).
	EXPECT_NEAR(2.0f*deg, roll,   5e-3f);
	EXPECT_NEAR(1.5f*deg, pitch,  5e-3f);
	EXPECT_NEAR(0.05f,    y,      1e-2f);
	EXPECT_NEAR(0.05f,    z,      1e-2f);
	EXPECT_NEAR(3.0f*deg, yawOut, 5e-3f);
}

TEST(RegistrationIcpTest, Corridor3D_Drone4DoF_ImuPlusXGuess_RecoversFullTransform)
{
	if(!RegistrationIcp::available(RegistrationIcp::kIcpPointMatcher))
	{
		GTEST_SKIP() << "Force4DoF requires libpointmatcher";
	}
	const LaserScan from = makeScan(makeCorridor3D());
	const float deg = static_cast<float>(M_PI) / 180.0f;
	const Transform truth(1.0f, 0.05f, 0.05f, 2.0f*deg, 1.5f*deg, 3.0f*deg);
	const LaserScan to = util3d::transformLaserScan(
			makeScan(makeCorridor3D(/*length=*/6.0f, /*halfWidth=*/0.5f,
					/*height=*/2.0f, /*wallPoints=*/500, /*floorPoints=*/500,
					/*ceilingPoints=*/500, /*surfaceJitter=*/0.005f,
					/*seed=*/0xBA5EBA11)),
			observableMotion(truth).inverse());

	ParametersMap params = corridorIcpParams6DoF(RegistrationIcp::kIcpPointMatcher);
	params[Parameters::kIcpForce4DoF()] = "true";
	// IMU + x guess: world roll/pitch from IMU, x from odometry.
	const Transform guess(1.0f, 0.0f, 0.0f, 2.0f*deg, 1.5f*deg, 0.0f);
	RegistrationInfo info;
	const Transform t = runIcp(RegistrationIcp::kIcpPointMatcher, from, to, params,
			guess, &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;

	float x, y, z, roll, pitch, yawOut;
	t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yawOut);
	EXPECT_NEAR(1.0f,     x,      5e-3f);
	EXPECT_NEAR(2.0f*deg, roll,   5e-3f);
	EXPECT_NEAR(1.5f*deg, pitch,  5e-3f);
	EXPECT_NEAR(0.05f,    y,      1e-2f);
	EXPECT_NEAR(0.05f,    z,      1e-2f);
	EXPECT_NEAR(3.0f*deg, yawOut, 5e-3f);
}

TEST(RegistrationIcpTest, Corridor2D_NoGuess_FailsToRecoverXButGetsYAndYaw)
{
	// 2D analogue: two parallel lines along x. Same degeneracy + same
	// infinite-corridor model (independent `to` sample, x stripped from
	// the truth motion).
	const LaserScan from = makeCorridor2D();
	const float yaw = 3.0f * static_cast<float>(M_PI) / 180.0f;
	const Transform truth(1.0f, 0.05f, 0.0f, 0.0f, 0.0f, yaw);
	const LaserScan to = util3d::transformLaserScan(
			makeCorridor2D(/*length=*/6.0f, /*halfWidth=*/0.5f,
					/*yJitter=*/0.005f, /*pointsPerLine=*/500,
					/*seed=*/0xBA5EBA11),
			observableMotion(truth).inverse());

	RegistrationInfo info;
	const Transform t = runIcp(defaultIcpStrategy(), from, to, corridorIcpParams(defaultIcpStrategy()), Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(info.icpStructuralComplexity, 0.02f)
			<< "expected 2D corridor to land below low-complexity threshold; got "
			<< info.icpStructuralComplexity;

	float x, y, z, roll, pitch, yawOut;
	t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yawOut);
	EXPECT_LT(std::abs(x), 0.1f)
			<< "expected x to stay near guess (=0), got " << x;
	EXPECT_NEAR(0.05f, y, 1e-2f);
	EXPECT_NEAR(yaw, yawOut, 5e-3f);
}

TEST(RegistrationIcpTest, Corridor2D_WithXGuess_RecoversFullTransform)
{
	const LaserScan from = makeCorridor2D();
	const float yaw = 3.0f * static_cast<float>(M_PI) / 180.0f;
	const Transform truth(1.0f, 0.05f, 0.0f, 0.0f, 0.0f, yaw);
	const LaserScan to = util3d::transformLaserScan(
			makeCorridor2D(/*length=*/6.0f, /*halfWidth=*/0.5f,
					/*yJitter=*/0.005f, /*pointsPerLine=*/500,
					/*seed=*/0xBA5EBA11),
			observableMotion(truth).inverse());

	const Transform xGuess(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	RegistrationInfo info;
	const Transform t = runIcp(defaultIcpStrategy(), from, to, corridorIcpParams(defaultIcpStrategy()), xGuess, &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(t.getDistance(truth), 2e-2f)
			<< "got=" << t.prettyPrint() << " truth=" << truth.prettyPrint();
	EXPECT_LT(t.getAngle(truth), 5e-3f);
}

INSTANTIATE_TEST_SUITE_P(
		Corridor3D,
		CorridorTest,
		::testing::ValuesIn(pointToPlaneCapableStrategies()),
		[](const ::testing::TestParamInfo<RegistrationIcp::IcpStrategy> & info)
		{
			return std::string(RegistrationIcp::strategyName(info.param));
		});

// =====================================================================
// PointToPlane with perpendicular surfaces: scene fully constrains all
// DoF, so structural complexity stays well above
// Icp/PointToPlaneMinComplexity and the low-complexity fallback does NOT
// fire.  PCL has no 2D PointToPlane estimator (TransformationEstimation2D
// is point-to-point only), so the 2D L-corner case is libpointmatcher-only;
// the 3D room corner case runs on both PCL and libpointmatcher.  CCCoreLib
// has no PointToPlane support and is skipped on both.
// =====================================================================

namespace {
class CornerTest : public ::testing::TestWithParam<RegistrationIcp::IcpStrategy>
{
protected:
	RegistrationIcp::IcpStrategy strategy() const { return GetParam(); }
};
}  // namespace


TEST_P(CornerTest, RoomCorner3D_PointToPlane_RecoversFullTransform)
{
	const LaserScan from = makeScan(makeRoomCorner3D());
	const float deg = static_cast<float>(M_PI) / 180.0f;
	const Transform truth(0.05f, 0.05f, 0.05f, 2.0f*deg, 1.5f*deg, 3.0f*deg);
	// Independent sample (different seed) -- mirrors the corridor-test
	// model so the recovery tolerance reflects the per-point voxel + jitter
	// noise floor, not a degenerate identity-match.
	const LaserScan to = util3d::transformLaserScan(
			makeScan(makeRoomCorner3D(/*length=*/2.0f, /*pointsPerSurface=*/500,
					/*surfaceJitter=*/0.005f, /*seed=*/0xBA5EBA11)),
			truth.inverse());

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to,
			corridorIcpParams6DoF(strategy()), Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_GT(info.icpStructuralComplexity, 0.02f)
			<< "expected room corner to stay above low-complexity threshold; got "
			<< info.icpStructuralComplexity;
	EXPECT_LT(t.getDistance(truth), 1e-2f)
			<< "got=" << t.prettyPrint() << " truth=" << truth.prettyPrint();
	EXPECT_LT(t.getAngle(truth), 5e-3f);
}

INSTANTIATE_TEST_SUITE_P(
		RoomCorner3D,
		CornerTest,
		::testing::ValuesIn(pointToPlaneCapableStrategies()),
		[](const ::testing::TestParamInfo<RegistrationIcp::IcpStrategy> & info)
		{
			return std::string(RegistrationIcp::strategyName(info.param));
		});

TEST(RegistrationIcpTest, RoomCorner2D_PointToPlane_RecoversFullTransform)
{
	// libpointmatcher only -- PCL's TransformationEstimation2D is
	// point-to-point and crashes mid-iteration when fed normals; see
	// makeCorridor2D / util3d_registration.cpp for context.
	if(!RegistrationIcp::available(RegistrationIcp::kIcpPointMatcher))
	{
		GTEST_SKIP() << "2D PointToPlane requires libpointmatcher";
	}
	const LaserScan from = makeRoomCorner2D();
	const float yaw = 3.0f * static_cast<float>(M_PI) / 180.0f;
	const Transform truth(0.05f, 0.05f, 0.0f, 0.0f, 0.0f, yaw);
	const LaserScan to = util3d::transformLaserScan(
			makeRoomCorner2D(/*length=*/2.0f, /*jitter=*/0.005f,
					/*pointsPerLine=*/500, /*seed=*/0xBA5EBA11),
			truth.inverse());

	RegistrationInfo info;
	const Transform t = runIcp(RegistrationIcp::kIcpPointMatcher, from, to,
			corridorIcpParams(RegistrationIcp::kIcpPointMatcher),
			Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_GT(info.icpStructuralComplexity, 0.02f)
			<< "expected room corner to stay above low-complexity threshold; got "
			<< info.icpStructuralComplexity;
	EXPECT_LT(t.getDistance(truth), 1e-2f)
			<< "got=" << t.prettyPrint() << " truth=" << truth.prettyPrint();
	EXPECT_LT(t.getAngle(truth), 5e-3f);
}

// =====================================================================
// Parameterized tests -- run once per available ICP backend
// =====================================================================

TEST_P(RegistrationIcpStrategyTest, SameScanWithIdentityGuessRecoversIdentity)
{
	const LaserScan scan = makeScan(makeRandomCloud());
	RegistrationInfo info;
	const Transform t = runIcp(strategy(), scan, scan, {}, Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << "rejectedMsg=" << info.rejectedMsg;
	const Transform I = Transform::getIdentity();
	EXPECT_LT(t.getDistance(I), tol());
	EXPECT_LT(t.getAngle(I), tol());
}

TEST_P(RegistrationIcpStrategyTest, RecoversSmallTranslation)
{
	const LaserScan from = makeScan(makeRandomCloud());
	const Transform truth(0.05f, 0.03f, -0.02f, 0.0f, 0.0f, 0.0f);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, {}, Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(t.getDistance(truth), tol())
			<< "got=" << t.prettyPrint() << " truth=" << truth.prettyPrint();
	EXPECT_LT(t.getAngle(truth), tol());
}

TEST_P(RegistrationIcpStrategyTest, RecoversSmallRotation)
{
	const LaserScan from = makeScan(makeRandomCloud());
	const float yaw = 5.0f * static_cast<float>(M_PI) / 180.0f;
	const Transform truth(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, yaw);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, {}, Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(t.getDistance(truth), tol());
	EXPECT_LT(t.getAngle(truth), tol());
}

TEST_P(RegistrationIcpStrategyTest, RecoversCombinedSmallMotion)
{
	const LaserScan from = makeScan(makeRandomCloud());
	const float yaw   = 3.0f * static_cast<float>(M_PI) / 180.0f;
	const float pitch = 2.0f * static_cast<float>(M_PI) / 180.0f;
	const Transform truth(0.04f, -0.02f, 0.01f, 0.0f, pitch, yaw);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, {}, Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(t.getDistance(truth), tol());
	EXPECT_LT(t.getAngle(truth), tol());
}

TEST_P(RegistrationIcpStrategyTest, NonIdentityGuessShortensCorrection)
{
	// 8 cm + 10 deg motion. With an identity guess ICP has to make the full
	// correction; with `truth` as guess it barely moves. Comparing the
	// reported ICP correction magnitudes proves the guess is being used.
	const LaserScan from = makeScan(makeRandomCloud());
	const float yaw = 10.0f * static_cast<float>(M_PI) / 180.0f;
	const Transform truth(0.08f, 0.05f, 0.0f, 0.0f, 0.0f, yaw);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	RegistrationInfo noGuess;
	const Transform tNo = runIcp(strategy(), from, to, {}, Transform::getIdentity(), &noGuess);
	RegistrationInfo withGuess;
	const Transform tYes = runIcp(strategy(), from, to, {}, truth, &withGuess);

	ASSERT_FALSE(tNo.isNull())  << noGuess.rejectedMsg;
	ASSERT_FALSE(tYes.isNull()) << withGuess.rejectedMsg;
	EXPECT_LT(tNo.getDistance(truth),  tol());
	EXPECT_LT(tYes.getDistance(truth), tol());
	EXPECT_LT(tNo.getAngle(truth),  tol());
	EXPECT_LT(tYes.getAngle(truth), tol());

	EXPECT_LT(withGuess.icpTranslation, 0.1f * noGuess.icpTranslation)
			<< "noGuess.icpTranslation=" << noGuess.icpTranslation
			<< " withGuess.icpTranslation=" << withGuess.icpTranslation;
	EXPECT_LT(withGuess.icpRotation, 0.1f * noGuess.icpRotation)
			<< "noGuess.icpRotation=" << noGuess.icpRotation
			<< " withGuess.icpRotation=" << withGuess.icpRotation;

	// Iteration count: PCL and libpointmatcher populate it; CCCoreLib leaves
	// it at -1 (its Register() doesn't surface the count). Assert only when
	// the backend filled the field.
	if(noGuess.icpIterations > 0 && withGuess.icpIterations > 0)
	{
		EXPECT_LT(withGuess.icpIterations, noGuess.icpIterations)
				<< "noGuess.icpIterations=" << noGuess.icpIterations
				<< " withGuess.icpIterations=" << withGuess.icpIterations;
	}
}

// --- Constraint enforcement -------------------------------------------------

TEST_P(RegistrationIcpStrategyTest, Force3DoFZeroesNonPlanarComponents)
{
	// CCCoreLib's 3DoF wrapper zeroes the z coordinate of every input
	// point (see icp/cccorelib.h), collapsing our 3D blob cloud onto the
	// z=0 plane. With 50 blob centroids overlapping in z, CC's KD-tree
	// loses its lock and returns near-identity. PCL / libpointmatcher
	// constrain the *transformation* to 3DoF without touching the inputs.
	if(strategy() == RegistrationIcp::kIcpCCCoreLib)
	{
		GTEST_SKIP() << "CCCoreLib's 3DoF input-collapse defeats clustered-cloud lock";
	}
	const LaserScan from = makeScan(makeRandomCloud());
	const float yaw = 3.0f * static_cast<float>(M_PI) / 180.0f;
	const Transform truth(0.04f, -0.02f, 0.0f, 0.0f, 0.0f, yaw);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	ParametersMap extra;
	extra[Parameters::kRegForce3DoF()] = "true";

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, extra, Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	float x, y, z, roll, pitch, yawOut;
	t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yawOut);
	EXPECT_LT(std::abs(z), 1e-3f)     << "z should be pinned to 0 in 3DoF";
	EXPECT_LT(std::abs(roll), 1e-3f)  << "roll should be pinned to 0 in 3DoF";
	EXPECT_LT(std::abs(pitch), 1e-3f) << "pitch should be pinned to 0 in 3DoF";
	EXPECT_NEAR(yaw, yawOut, 5e-3f);
}

TEST_P(RegistrationIcpStrategyTest, Force3DoFOn2DScan)
{
	// Same Force3DoF semantics as above but the scan is 2D (kXY format).
	// Each backend's wrapper should detect the format and switch to a
	// 2D-aware transformation estimator: PCL uses
	// TransformationEstimation2D; libpointmatcher and CCCoreLib should
	// install equivalents.
	const LaserScan from = makeRandom2DScan();
	const float yaw = 3.0f * static_cast<float>(M_PI) / 180.0f;
	const Transform truth(0.04f, -0.02f, 0.0f, 0.0f, 0.0f, yaw);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	ParametersMap extra;
	extra[Parameters::kRegForce3DoF()] = "true";

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, extra, Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	float x, y, z, roll, pitch, yawOut;
	t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yawOut);
	EXPECT_LT(std::abs(z), 1e-3f)     << "z should be pinned to 0 in 3DoF";
	EXPECT_LT(std::abs(roll), 1e-3f)  << "roll should be pinned to 0 in 3DoF";
	EXPECT_LT(std::abs(pitch), 1e-3f) << "pitch should be pinned to 0 in 3DoF";
	// 2D scans give ICP one fewer dimension of correspondence info than 3D,
	// so the recovered yaw/x/y are a bit looser than the 3D Force3DoF test
	// (~6 mrad / 5 mm on PCL+libpointmatcher, ~7 mm on CCCoreLib).
	const float bound = 2.0f * tol();
	EXPECT_NEAR(yaw, yawOut, bound);
	EXPECT_NEAR(0.04f, x, bound);
	EXPECT_NEAR(-0.02f, y, bound);
}

TEST_P(RegistrationIcpStrategyTest, Force4DoFZeroesRollAndPitch)
{
	// Icp/Force4DoF requires a non-PCL backend (RegistrationIcp warns + falls
	// back to false on PCL).
	if(strategy() == RegistrationIcp::kIcpPCL)
	{
		GTEST_SKIP() << "Force4DoF not supported on PCL strategy";
	}
	const LaserScan from = makeScan(makeRandomCloud());
	// Motion with all 6 DoF -- after Force4DoF, only x/y/z/yaw survive.
	const float deg = static_cast<float>(M_PI) / 180.0f;
	const Transform truth(0.04f, -0.02f, 0.03f, 2.0f*deg, 1.5f*deg, 3.0f*deg);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	ParametersMap extra;
	extra[Parameters::kIcpForce4DoF()] = "true";
	// libpointmatcher only honours force4DOF inside its PointToPlane error
	// minimizer, so opt in (rtabmap's default is PointToPlane=true; we
	// override to false in baseIcpParams to keep the synthetic random-cloud
	// tests off the normal/complexity path).
	extra[Parameters::kIcpPointToPlane()] = "true";
	extra[Parameters::kIcpPointToPlaneK()] = "10";
	extra[Parameters::kIcpPointToPlaneRadius()] = "0.0";
	extra[Parameters::kIcpPointToPlaneMinComplexity()] = "0.0";

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, extra, Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	float x, y, z, roll, pitch, yawOut;
	t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yawOut);
	EXPECT_LT(std::abs(roll), 1e-3f)  << "roll should be pinned to 0 in 4DoF";
	EXPECT_LT(std::abs(pitch), 1e-3f) << "pitch should be pinned to 0 in 4DoF";
	// x, y, z, yaw remain free DoFs. Truth had roll/pitch components too,
	// so ICP solves for the best 4DoF approximation -- recovered x/y/z/yaw
	// won't match truth exactly because the rejected DoFs leak into the
	// kept ones (3 cm / 0.6 deg on libpointmatcher, ~4 cm / 2 deg on CC).
	const float transBound = strategy() == RegistrationIcp::kIcpCCCoreLib ? 0.05f : 0.03f;
	const float yawBound   = strategy() == RegistrationIcp::kIcpCCCoreLib ? 0.05f : 1e-2f;
	EXPECT_LT(std::abs(x - 0.04f), transBound);
	EXPECT_LT(std::abs(y - (-0.02f)), transBound);
	EXPECT_LT(std::abs(z - 0.03f), transBound);
	EXPECT_NEAR(3.0f * deg, yawOut, yawBound);
}

TEST_P(RegistrationIcpStrategyTest, OvershootingMaxTranslationIsRejected)
{
	// MaxTranslation default = 0.2 m. Push 0.3 m -- the correction exceeds
	// the bound, so the registrar must return a null transform and report
	// the rejection.
	const LaserScan from = makeScan(makeRandomCloud());
	const Transform truth(0.30f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	ParametersMap extra;
	extra[Parameters::kIcpMaxTranslation()] = "0.2";

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, extra, Transform::getIdentity(), &info);
	EXPECT_TRUE(t.isNull())
			<< "got=" << t.prettyPrint()
			<< " rejectedMsg=" << info.rejectedMsg;
	EXPECT_FALSE(info.rejectedMsg.empty());
}

TEST_P(RegistrationIcpStrategyTest, OvershootingMaxRotationIsRejected)
{
	// Apply 15 deg yaw and tighten MaxRotation to 0.1 rad (~5.7 deg) -- ICP
	// converges to a correction well above the bound, so the registrar must
	// reject. (60 deg with identity guess only converges partially, so the
	// correction can land below the guard; 15 deg stays inside the basin
	// and converges fully.)
	const LaserScan from = makeScan(makeRandomCloud());
	const float yaw = 15.0f * static_cast<float>(M_PI) / 180.0f;
	const Transform truth(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, yaw);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	ParametersMap extra;
	extra[Parameters::kIcpMaxRotation()] = "0.1";

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, extra, Transform::getIdentity(), &info);
	EXPECT_TRUE(t.isNull())
			<< "got=" << t.prettyPrint()
			<< " rejectedMsg=" << info.rejectedMsg;
	EXPECT_FALSE(info.rejectedMsg.empty());
}

// --- Filtering knobs --------------------------------------------------------

TEST_P(RegistrationIcpStrategyTest, VoxelDownsamplingPreservesRecovery)
{
	// 5 cm voxel collapses the per-blob neighbourhood (sigma=4 cm) to one
	// or two points per cluster, but the cluster centroids survive and
	// preserve global correspondences -- ICP still converges.
	const LaserScan from = makeScan(makeRandomCloud());
	const Transform truth(0.03f, 0.02f, 0.0f, 0.0f, 0.0f,
			2.0f * static_cast<float>(M_PI) / 180.0f);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	ParametersMap extra;
	extra[Parameters::kIcpVoxelSize()] = "0.05";

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, extra, Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(t.getDistance(truth), 1e-2f);
	EXPECT_LT(t.getAngle(truth), 1e-2f);
}

TEST_P(RegistrationIcpStrategyTest, VoxelDownsamplingPreservesRecoveryOn2DScan)
{
	// Same as above but on a 2D scan (kXY).  Voxel size = 2 cm so the
	// tighter 2D blobs (sigma=1 cm) collapse to one point per cluster --
	// enough geometry left to recover the planar motion.
	const LaserScan from = makeRandom2DScan();
	const Transform truth(0.03f, 0.02f, 0.0f, 0.0f, 0.0f,
			2.0f * static_cast<float>(M_PI) / 180.0f);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	ParametersMap extra;
	extra[Parameters::kRegForce3DoF()] = "true";
	extra[Parameters::kIcpVoxelSize()] = "0.02";

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, extra, Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	// CC settles at ~1.1 cm on the voxelized 2D blobs (vs <2 mm for PCL /
	// libpointmatcher) -- voxel-collapsed clusters lose the intra-blob
	// shape info CC otherwise leans on. Loosen to 1.5 cm for that backend.
	const float distBound = strategy() == RegistrationIcp::kIcpCCCoreLib ? 1.5e-2f : 1e-2f;
	EXPECT_LT(t.getDistance(truth), distBound);
	EXPECT_LT(t.getAngle(truth), 1e-2f);
}

TEST_P(RegistrationIcpStrategyTest, PointToPlaneVariantRecoversSmallMotion)
{
	// CCCoreLib doesn't implement point-to-plane: RegistrationIcp warns and
	// falls back to point-to-point on that backend, which is still a valid
	// run but doesn't actually exercise the point-to-plane code path. Skip.
	if(strategy() == RegistrationIcp::kIcpCCCoreLib)
	{
		GTEST_SKIP() << "CCCoreLib does not support point-to-plane ICP";
	}
	const LaserScan from = makeScan(makeRandomCloud());
	const Transform truth(0.03f, 0.02f, -0.01f, 0.0f, 0.0f,
			3.0f * static_cast<float>(M_PI) / 180.0f);
	const LaserScan to = util3d::transformLaserScan(from, truth.inverse());

	ParametersMap extra;
	extra[Parameters::kIcpPointToPlane()] = "true";
	// PointToPlane needs normals; the registrar estimates them on the fly
	// with these neighbourhood params.
	extra[Parameters::kIcpPointToPlaneK()]      = "10";
	extra[Parameters::kIcpPointToPlaneRadius()] = "0.0";
	// Pure random points have low PCA structural complexity; relax the
	// minimum so point-to-plane stays active.
	extra[Parameters::kIcpPointToPlaneMinComplexity()] = "0.0";

	RegistrationInfo info;
	const Transform t = runIcp(strategy(), from, to, extra, Transform::getIdentity(), &info);
	ASSERT_FALSE(t.isNull()) << info.rejectedMsg;
	EXPECT_LT(t.getDistance(truth), tol());
	EXPECT_LT(t.getAngle(truth), tol());
}

INSTANTIATE_TEST_SUITE_P(
		Strategies,
		RegistrationIcpStrategyTest,
		::testing::ValuesIn(availableIcpStrategies()),
		[](const ::testing::TestParamInfo<RegistrationIcp::IcpStrategy> & info)
		{
			return std::string(RegistrationIcp::strategyName(info.param));
		});
