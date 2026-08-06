#include <gtest/gtest.h>
#include <rtabmap/core/IMUFilter.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UConversion.h>
#include <cmath>
#include <memory>

using namespace rtabmap;

namespace {

static constexpr double kGravity = 9.81;

static void expectQuatNear(
		double qx,
		double qy,
		double qz,
		double qw,
		double ex,
		double ey,
		double ez,
		double ew,
		double tol = 1e-3)
{
	EXPECT_NEAR(qx, ex, tol);
	EXPECT_NEAR(qy, ey, tol);
	EXPECT_NEAR(qz, ez, tol);
	EXPECT_NEAR(qw, ew, tol);
}

static double quatNorm(double qx, double qy, double qz, double qw)
{
	return std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
}

static ParametersMap complementaryParams(
		double gainAcc = 0.2,
		bool biasEstimation = false)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kImuFilterComplementaryGainAcc(), uNumber2Str(gainAcc)));
	params.insert(ParametersPair(
			Parameters::kImuFilterComplementaryDoBiasEstimation(),
			biasEstimation ? "true" : "false"));
	params.insert(ParametersPair(Parameters::kImuFilterComplementaryDoAdpativeGain(), "false"));
	return params;
}

static void feedStaticLevel(IMUFilter & filter, double stamp, int count = 20, double dt = 0.01)
{
	for(int i = 0; i < count; ++i)
	{
		filter.update(0, 0, 0, 0, 0, kGravity, stamp + i * dt);
	}
}

static void feedYawRate(IMUFilter & filter, double gz, double stamp, int count = 50, double dt = 0.01)
{
	for(int i = 0; i < count; ++i)
	{
		filter.update(0, 0, gz, 0, 0, kGravity, stamp + i * dt);
	}
}

} // namespace

TEST(IMUFilterTest, CreateComplementaryFilter)
{
	std::unique_ptr<IMUFilter> filter(IMUFilter::create(IMUFilter::kComplementaryFilter));
	ASSERT_NE(filter.get(), nullptr);
	EXPECT_EQ(filter->type(), IMUFilter::kComplementaryFilter);
}

TEST(IMUFilterTest, CreateMadgwickOrFallback)
{
	std::unique_ptr<IMUFilter> filter(IMUFilter::create(IMUFilter::kMadgwick));
	ASSERT_NE(filter.get(), nullptr);
#ifdef RTABMAP_MADGWICK
	EXPECT_EQ(filter->type(), IMUFilter::kMadgwick);
#else
	EXPECT_EQ(filter->type(), IMUFilter::kComplementaryFilter);
#endif
}

TEST(IMUFilterTest, CreateFromParametersMap)
{
	std::unique_ptr<IMUFilter> filter(IMUFilter::create(complementaryParams()));
	ASSERT_NE(filter.get(), nullptr);
	EXPECT_EQ(filter->type(), IMUFilter::kComplementaryFilter);
}

TEST(IMUFilterTest, ResetSetsOrientation)
{
	std::unique_ptr<IMUFilter> filter(
			IMUFilter::create(IMUFilter::kComplementaryFilter, complementaryParams()));

	filter->reset();
	double qx = 0, qy = 0, qz = 0, qw = 0;
	filter->getOrientation(qx, qy, qz, qw);
	expectQuatNear(qx, qy, qz, qw, 0, 0, 0, 1, 1e-4);

	// Use a unit quaternion (reset stores its inverse internally).
	const double qxIn = 0.1, qyIn = 0.2, qzIn = 0.3;
	const double qwIn = std::sqrt(1.0 - qxIn * qxIn - qyIn * qyIn - qzIn * qzIn);
	filter->reset(qxIn, qyIn, qzIn, qwIn);
	filter->getOrientation(qx, qy, qz, qw);
	expectQuatNear(qx, qy, qz, qw, qxIn, qyIn, qzIn, qwIn, 1e-4);
	EXPECT_NEAR(quatNorm(qx, qy, qz, qw), 1.0, 1e-4);
}

TEST(IMUFilterTest, StaticAccelerometerNearIdentity)
{
	std::unique_ptr<IMUFilter> filter(
			IMUFilter::create(IMUFilter::kComplementaryFilter, complementaryParams()));
	feedStaticLevel(*filter, 0.0);

	double qx = 0, qy = 0, qz = 0, qw = 0;
	filter->getOrientation(qx, qy, qz, qw);
	expectQuatNear(qx, qy, qz, qw, 0, 0, 0, 1, 0.05);
	EXPECT_NEAR(quatNorm(qx, qy, qz, qw), 1.0, 1e-4);
}

TEST(IMUFilterTest, OrientationStaysNormalized)
{
	std::unique_ptr<IMUFilter> filter(
			IMUFilter::create(IMUFilter::kComplementaryFilter, complementaryParams()));
	feedStaticLevel(*filter, 0.0, 100);

	double qx = 0, qy = 0, qz = 0, qw = 0;
	filter->getOrientation(qx, qy, qz, qw);
	EXPECT_NEAR(quatNorm(qx, qy, qz, qw), 1.0, 1e-4);
}

TEST(IMUFilterTest, GyroIntegrationChangesYaw)
{
	std::unique_ptr<IMUFilter> filter(
			IMUFilter::create(IMUFilter::kComplementaryFilter, complementaryParams(0.01, false)));
	feedStaticLevel(*filter, 0.0, 5);
	feedYawRate(*filter, 1.0, 0.1, 80, 0.01);

	double qx = 0, qy = 0, qz = 0, qw = 0;
	filter->getOrientation(qx, qy, qz, qw);
	EXPECT_GT(std::fabs(qz), 0.05);
	EXPECT_NEAR(quatNorm(qx, qy, qz, qw), 1.0, 1e-3);
}

TEST(IMUFilterTest, ParseParametersAffectsFilter)
{
	ParametersMap params = complementaryParams(0.5, false);
	std::unique_ptr<IMUFilter> filter(IMUFilter::create(IMUFilter::kComplementaryFilter, params));
	filter->parseParameters(complementaryParams(0.01, false));
	feedStaticLevel(*filter, 0.0);

	double qx = 0, qy = 0, qz = 0, qw = 0;
	filter->getOrientation(qx, qy, qz, qw);
	EXPECT_NEAR(quatNorm(qx, qy, qz, qw), 1.0, 1e-4);
}

#ifdef RTABMAP_MADGWICK
TEST(IMUFilterTest, MadgwickStaticAccelerometerNearIdentity)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kImuFilterMadgwickGain(), "0.1"));
	params.insert(ParametersPair(Parameters::kImuFilterMadgwickZeta(), "0.0"));

	std::unique_ptr<IMUFilter> filter(IMUFilter::create(IMUFilter::kMadgwick, params));
	feedStaticLevel(*filter, 0.0);

	double qx = 0, qy = 0, qz = 0, qw = 0;
	filter->getOrientation(qx, qy, qz, qw);
	expectQuatNear(qx, qy, qz, qw, 0, 0, 0, 1, 0.1);
	EXPECT_NEAR(quatNorm(qx, qy, qz, qw), 1.0, 1e-4);
}
#endif
