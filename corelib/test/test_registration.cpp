#include <gtest/gtest.h>
#include <rtabmap/core/Registration.h>
#include <rtabmap/core/Parameters.h>
#include <opencv2/core.hpp>

using namespace rtabmap;

namespace {

class MockRegistration : public Registration
{
public:
	explicit MockRegistration(const ParametersMap & parameters = ParametersMap(), Registration * child = 0) :
		Registration(parameters, child)
	{
	}

	mutable int implCallCount = 0;
	mutable std::vector<Transform> guessHistory;
	Transform resultTransform = Transform::getIdentity();
	bool failRegistration = false;

	bool imageRequiredFlag = false;
	bool scanRequiredFlag = false;
	bool userDataRequiredFlag = false;
	bool canUseGuessFlag = false;
	int minVisualValue = 0;
	float minGeometryValue = 0.f;

protected:
	Transform computeTransformationImpl(
			Signature & /*from*/,
			Signature & /*to*/,
			Transform guess,
			RegistrationInfo & /*info*/) const override
	{
		++implCallCount;
		guessHistory.push_back(guess);
		if(failRegistration)
		{
			return Transform();
		}
		return resultTransform;
	}

	bool isImageRequiredImpl() const override { return imageRequiredFlag; }
	bool isScanRequiredImpl() const override { return scanRequiredFlag; }
	bool isUserDataRequiredImpl() const override { return userDataRequiredFlag; }
	bool canUseGuessImpl() const override { return canUseGuessFlag; }
	int getMinVisualCorrespondencesImpl() const override { return minVisualValue; }
	float getMinGeometryCorrespondencesRatioImpl() const override { return minGeometryValue; }
};

static ParametersMap registrationTestParams()
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kRegRepeatOnce(), "false"));
	params.insert(ParametersPair(Parameters::kRegForce3DoF(), "false"));
	return params;
}

} // namespace

TEST(RegistrationTest, ConstructorParsesParameters)
{
	ParametersMap params;
	params[Parameters::kRegRepeatOnce()] = "true";
	params[Parameters::kRegForce3DoF()] = "true";

	MockRegistration reg(params);
	EXPECT_TRUE(reg.repeatOnce());
	EXPECT_TRUE(reg.force3DoF());
}

TEST(RegistrationTest, ParseParametersUpdatesFlags)
{
	MockRegistration reg(registrationTestParams());
	EXPECT_FALSE(reg.repeatOnce());
	EXPECT_FALSE(reg.force3DoF());

	ParametersMap update;
	update[Parameters::kRegRepeatOnce()] = "true";
	reg.parseParameters(update);
	EXPECT_TRUE(reg.repeatOnce());
	EXPECT_FALSE(reg.force3DoF());
}

TEST(RegistrationTest, RequirementFlagsCombineWithChild)
{
	MockRegistration * child = new MockRegistration();
	child->scanRequiredFlag = true;

	MockRegistration parent(registrationTestParams(), child);
	EXPECT_FALSE(parent.isImageRequired());
	EXPECT_TRUE(parent.isScanRequired());
	EXPECT_FALSE(parent.isUserDataRequired());
}

TEST(RegistrationTest, MinCorrespondencesTakeMaximumWithChild)
{
	MockRegistration * child = new MockRegistration();
	child->minVisualValue = 10;
	child->minGeometryValue = 0.5f;

	MockRegistration parent(registrationTestParams(), child);
	parent.minVisualValue = 5;
	parent.minGeometryValue = 0.2f;

	EXPECT_EQ(parent.getMinVisualCorrespondences(), 10);
	EXPECT_FLOAT_EQ(parent.getMinGeometryCorrespondencesRatio(), 0.5f);
}

TEST(RegistrationTest, CanUseGuessCombinesWithChild)
{
	MockRegistration * child = new MockRegistration();
	child->canUseGuessFlag = true;

	MockRegistration parent(registrationTestParams(), child);
	EXPECT_FALSE(parent.canUseGuessFlag);
	EXPECT_TRUE(parent.canUseGuess());
}

TEST(RegistrationTest, SetChildRegistrationTakesOwnership)
{
	MockRegistration parent(registrationTestParams());
	MockRegistration * firstChild = new MockRegistration();
	firstChild->minVisualValue = 3;
	parent.setChildRegistration(firstChild);

	MockRegistration * secondChild = new MockRegistration();
	secondChild->minVisualValue = 8;
	parent.setChildRegistration(secondChild);

	EXPECT_EQ(parent.getMinVisualCorrespondences(), 8);
}

TEST(RegistrationTest, ComputeTransformationCallsImpl)
{
	MockRegistration reg(registrationTestParams());
	reg.resultTransform = Transform(1.f, 0.f, 0.f, 0.f, 0.f, 0.f);

	const Signature from;
	const Signature to;
	const Transform result = reg.computeTransformation(from, to);

	EXPECT_EQ(reg.implCallCount, 1);
	EXPECT_FALSE(result.isNull());
	EXPECT_NEAR(result.x(), 1.f, 1e-6f);
}

TEST(RegistrationTest, ComputeTransformationChainsChild)
{
	MockRegistration * child = new MockRegistration(registrationTestParams());
	child->resultTransform = Transform(2.f, 0.f, 0.f, 0.f, 0.f, 0.f);

	MockRegistration parent(registrationTestParams(), child);
	parent.resultTransform = Transform(1.f, 0.f, 0.f, 0.f, 0.f, 0.f);

	const Signature from;
	const Signature to;
	const Transform result = parent.computeTransformation(from, to);

	EXPECT_EQ(parent.implCallCount, 1);
	EXPECT_EQ(child->implCallCount, 1);
	EXPECT_FALSE(result.isNull());
	EXPECT_NEAR(result.x(), 2.f, 1e-6f);
	EXPECT_EQ(child->guessHistory.size(), 1u);
	EXPECT_NEAR(child->guessHistory[0].x(), 1.f, 1e-6f);
}

TEST(RegistrationTest, ComputeTransformationParentFailsUsesGuessForChild)
{
	MockRegistration * child = new MockRegistration(registrationTestParams());
	child->resultTransform = Transform(2.f, 0.f, 0.f, 0.f, 0.f, 0.f);

	MockRegistration parent(registrationTestParams(), child);
	parent.failRegistration = true;

	const Signature from;
	const Signature to;
	const Transform guess(0.5f, 0.f, 0.f, 0.f, 0.f, 0.f);
	const Transform result = parent.computeTransformation(from, to, guess);

	EXPECT_EQ(parent.implCallCount, 1);
	EXPECT_EQ(child->implCallCount, 1);
	EXPECT_FALSE(result.isNull());
	EXPECT_NEAR(result.x(), 2.f, 1e-6f);
	EXPECT_NEAR(child->guessHistory[0].x(), 0.5f, 1e-6f);
}

TEST(RegistrationTest, RepeatOnceRefinesWithFirstResultAsGuess)
{
	ParametersMap params = registrationTestParams();
	params[Parameters::kRegRepeatOnce()] = "true";

	MockRegistration reg(params);
	EXPECT_TRUE(reg.repeatOnce());
	reg.canUseGuessFlag = true;
	reg.resultTransform = Transform(1.f, 0.f, 0.f, 0.f, 0.f, 0.f);

	const Signature from;
	const Signature to;
	Transform nullGuess;
	EXPECT_TRUE(nullGuess.isNull());
	const Transform result = reg.computeTransformation(from, to, nullGuess);

	EXPECT_EQ(reg.implCallCount, 2);
	EXPECT_FALSE(result.isNull());
	ASSERT_EQ(reg.guessHistory.size(), 2u);
	EXPECT_TRUE(reg.guessHistory[0].isNull());
	EXPECT_NEAR(reg.guessHistory[1].x(), 1.f, 1e-6f);
}

TEST(RegistrationTest, Force3DoFConstrainsGuessAndResult)
{
	ParametersMap params = registrationTestParams();
	params[Parameters::kRegForce3DoF()] = "true";

	MockRegistration reg(params);
	EXPECT_TRUE(reg.force3DoF());
	reg.resultTransform = Transform(1.f, 0.f, 2.f, 0.1f, 0.2f, 0.3f);

	const Signature from;
	const Signature to;
	const Transform guess(0.f, 0.f, 5.f, 0.5f, 0.5f, 0.5f);
	const Transform result = reg.computeTransformation(from, to, guess);

	ASSERT_EQ(reg.guessHistory.size(), 1u);
	EXPECT_TRUE(reg.guessHistory[0].is3DoF());
	EXPECT_NEAR(reg.guessHistory[0].z(), 0.f, 1e-6f);
	EXPECT_TRUE(result.is3DoF());
	EXPECT_NEAR(result.z(), 0.f, 1e-6f);
}

TEST(RegistrationTest, CovarianceEpsilonAppliedWhenEmpty)
{
	MockRegistration reg(registrationTestParams());
	reg.resultTransform = Transform::getIdentity();

	const Signature from;
	const Signature to;
	RegistrationInfo info;
	const Transform result = reg.computeTransformation(from, to, Transform::getIdentity(), &info);

	EXPECT_FALSE(result.isNull());
	ASSERT_FALSE(info.covariance.empty());
	EXPECT_EQ(info.covariance.rows, 6);
	EXPECT_EQ(info.covariance.cols, 6);
	EXPECT_GE(info.covariance.at<double>(0, 0), Registration::COVARIANCE_LINEAR_EPSILON);
	EXPECT_GE(info.covariance.at<double>(3, 3), Registration::COVARIANCE_ANGULAR_EPSILON);
	EXPECT_GE(info.totalTime, 0.0);
}

TEST(RegistrationTest, ComputeTransformationFromSensorData)
{
	MockRegistration reg(registrationTestParams());
	reg.resultTransform = Transform(1.f, 0.f, 0.f, 0.f, 0.f, 0.f);

	const SensorData from;
	const SensorData to;
	const Transform result = reg.computeTransformation(from, to);

	EXPECT_EQ(reg.implCallCount, 1);
	EXPECT_FALSE(result.isNull());
	EXPECT_NEAR(result.x(), 1.f, 1e-6f);
}
