#include <gtest/gtest.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Transform.h>
#include <cmath>

using namespace rtabmap;

static cv::Mat infMatrixDiagonal(
		double x,
		double y,
		double z,
		double roll,
		double pitch,
		double yaw)
{
	cv::Mat inf = cv::Mat::zeros(6, 6, CV_64FC1);
	inf.at<double>(0, 0) = x;
	inf.at<double>(1, 1) = y;
	inf.at<double>(2, 2) = z;
	inf.at<double>(3, 3) = roll;
	inf.at<double>(4, 4) = pitch;
	inf.at<double>(5, 5) = yaw;
	return inf;
}

TEST(LinkTest, DefaultConstructorIsInvalid)
{
	Link link;
	EXPECT_FALSE(link.isValid());
	EXPECT_EQ(link.from(), 0);
	EXPECT_EQ(link.to(), 0);
	EXPECT_EQ(link.type(), Link::kUndef);
	EXPECT_TRUE(link.transform().isNull());
}

TEST(LinkTest, ValidNeighborLink)
{
	Link link(1, 2, Link::kNeighbor, Transform::getIdentity());
	EXPECT_TRUE(link.isValid());
	EXPECT_EQ(link.from(), 1);
	EXPECT_EQ(link.to(), 2);
	EXPECT_EQ(link.type(), Link::kNeighbor);
	EXPECT_EQ(link.typeName(), "Neighbor");
	EXPECT_TRUE(link.transform().isIdentity());
	EXPECT_EQ(link.infMatrix().rows, 6);
	EXPECT_EQ(link.infMatrix().cols, 6);
}

TEST(LinkTest, InvalidWhenTransformNull)
{
	Link link(1, 2, Link::kNeighbor, Transform());
	EXPECT_FALSE(link.isValid());
}

TEST(LinkTest, TypeNameStatic)
{
	for(int type = Link::kNeighbor; type < Link::kEnd; ++type)
	{
		const std::string name = Link::typeName(static_cast<Link::Type>(type));
		EXPECT_NE(name, "Undefined") << "type=" << type;
		EXPECT_FALSE(name.empty()) << "type=" << type;
	}

	EXPECT_EQ(Link::typeName(Link::kNeighbor), "Neighbor");
	EXPECT_EQ(Link::typeName(Link::kGlobalClosure), "GlobalClosure");
	EXPECT_EQ(Link::typeName(Link::kLocalSpaceClosure), "LocalSpaceClosure");
	EXPECT_EQ(Link::typeName(Link::kLocalTimeClosure), "LocalTimeClosure");
	EXPECT_EQ(Link::typeName(Link::kUserClosure), "UserClosure");
	EXPECT_EQ(Link::typeName(Link::kVirtualClosure), "VirtualClosure");
	EXPECT_EQ(Link::typeName(Link::kNeighborMerged), "NeighborMerged");
	EXPECT_EQ(Link::typeName(Link::kPosePrior), "PosePrior");
	EXPECT_EQ(Link::typeName(Link::kLandmark), "Landmark");
	EXPECT_EQ(Link::typeName(Link::kGravity), "Gravity");

	EXPECT_EQ(Link::typeName(Link::kEnd), "Undefined");
	EXPECT_EQ(Link::typeName(Link::kUndef), "Undefined");
}

TEST(LinkTest, VariancesFromInformationMatrix)
{
	const cv::Mat inf = infMatrixDiagonal(4.0, 4.0, 4.0, 9.0, 9.0, 9.0);
	Link link(1, 2, Link::kNeighbor, Transform::getIdentity(), inf);

	EXPECT_NEAR(link.transVariance(true), 0.25, 1e-6);  // max diag -> min variance
	EXPECT_NEAR(link.transVariance(false), 0.25, 1e-6);
	EXPECT_NEAR(link.rotVariance(true), 1.0 / 9.0, 1e-6);
	EXPECT_NEAR(link.rotVariance(false), 1.0 / 9.0, 1e-6);
}

TEST(LinkTest, VariancesMinimumUsesLargestDiagonal)
{
	const cv::Mat inf = infMatrixDiagonal(1.0, 4.0, 16.0, 1.0, 4.0, 16.0);
	Link link(1, 2, Link::kNeighbor, Transform::getIdentity(), inf);

	EXPECT_NEAR(link.transVariance(true), 1.0 / 16.0, 1e-6); // 1 / max(1,4,16)
	EXPECT_NEAR(link.transVariance(false), 1.0, 1e-6);          // 1 / min(1,4,16)
}

TEST(LinkTest, InverseSwapsEndpointsAndTransform)
{
	const Transform t(1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f);
	Link link(5, 10, Link::kGlobalClosure, t);
	const Link inv = link.inverse();

	EXPECT_EQ(inv.from(), 10);
	EXPECT_EQ(inv.to(), 5);
	EXPECT_EQ(inv.type(), Link::kGlobalClosure);

	const Transform expectedInv = t.inverse();
	EXPECT_NEAR(inv.transform().x(), expectedInv.x(), 1e-4f);
	EXPECT_NEAR(inv.transform().y(), expectedInv.y(), 1e-4f);
	EXPECT_NEAR(inv.transform().z(), expectedInv.z(), 1e-4f);

	const Transform roundTrip = t * inv.transform();
	EXPECT_NEAR(roundTrip.x(), 0.0f, 1e-4f);
	EXPECT_NEAR(roundTrip.y(), 0.0f, 1e-4f);
	EXPECT_NEAR(roundTrip.z(), 0.0f, 1e-4f);
}

TEST(LinkTest, MergeNeighborMergedComposesTransformAndInformation)
{
	const Link ab(1, 2, Link::kNeighbor, Transform(1.0f, 0.0f, 0.0f, 0, 0, 0));
	const Link bc(2, 3, Link::kNeighbor, Transform(0.0f, 1.0f, 0.0f, 0, 0, 0));
	const Link ac = ab.merge(bc, Link::kNeighborMerged);

	EXPECT_EQ(ac.from(), 1);
	EXPECT_EQ(ac.to(), 3);
	EXPECT_EQ(ac.type(), Link::kNeighborMerged);
	EXPECT_NEAR(ac.transform().x(), 1.0f, 1e-4f);
	EXPECT_NEAR(ac.transform().y(), 1.0f, 1e-4f);
	EXPECT_NEAR(ac.transform().z(), 0.0f, 1e-4f);

	EXPECT_NEAR(ac.transVariance(true), 2.0, 1e-4f);
}

TEST(LinkTest, MergeGlobalClosureKeepsLinkWithLowerInformationOnX)
{
	// Non-NeighborMerged merge keeps @p link's matrix unless inf_ab(0,0) < inf_bc(0,0).
	const cv::Mat infHigh = infMatrixDiagonal(10.0, 10.0, 10.0, 10.0, 10.0, 10.0);
	const cv::Mat infLow = infMatrixDiagonal(2.0, 2.0, 2.0, 2.0, 2.0, 2.0);

	const Link ab(1, 2, Link::kNeighbor, Transform::getIdentity(), infHigh);
	const Link bc(2, 3, Link::kNeighbor, Transform::getIdentity(), infLow);
	const Link ac = ab.merge(bc, Link::kGlobalClosure);

	EXPECT_EQ(ac.type(), Link::kGlobalClosure);
	EXPECT_NEAR(ac.infMatrix().at<double>(0, 0), 2.0, 1e-6);
	EXPECT_NEAR(ac.transVariance(true), 0.5, 1e-6);
}

TEST(LinkTest, UserDataCompressesAndRoundTrips)
{
	const cv::Mat userData = (cv::Mat_<float>(2, 2) << 1.f, 2.f, 3.f, 4.f);
	Link link(1, 2, Link::kNeighbor, Transform::getIdentity(), cv::Mat::eye(6, 6, CV_64FC1), userData);

	EXPECT_FALSE(link.userDataCompressed().empty());
	EXPECT_EQ(link.userDataCompressed().type(), CV_8UC1);

	const cv::Mat restored = link.uncompressUserDataConst();
	ASSERT_EQ(restored.rows, userData.rows);
	ASSERT_EQ(restored.cols, userData.cols);
	ASSERT_EQ(restored.type(), userData.type());
	for(int r = 0; r < restored.rows; ++r)
	{
		for(int c = 0; c < restored.cols; ++c)
		{
			EXPECT_NEAR(restored.at<float>(r, c), userData.at<float>(r, c), 1e-5f);
		}
	}

	link.uncompressUserData();
	EXPECT_FALSE(link.userDataRaw().empty());
}

TEST(LinkTest, UserDataAlreadyCompressedIsKeptAsIs)
{
	const cv::Mat compressed(10, 1, CV_8UC1, cv::Scalar(42));
	Link link(1, 2, Link::kNeighbor, Transform::getIdentity(), cv::Mat::eye(6, 6, CV_64FC1), compressed);

	EXPECT_EQ(link.userDataCompressed().data, compressed.data);
	EXPECT_TRUE(link.userDataRaw().empty());
}

TEST(LinkTest, SetInfMatrixUpdatesVariances)
{
	Link link(1, 2, Link::kNeighbor, Transform::getIdentity());
	link.setInfMatrix(infMatrixDiagonal(2.0, 2.0, 2.0, 2.0, 2.0, 2.0));
	EXPECT_NEAR(link.transVariance(true), 0.5, 1e-6);
}

TEST(LinkTest, LandmarkLink)
{
	Link link(5, -10, Link::kLandmark, Transform(1.0f, 0.0f, 0.0f, 0, 0, 0));
	EXPECT_TRUE(link.isValid());
	EXPECT_EQ(link.to(), -10);
	EXPECT_EQ(link.typeName(), "Landmark");
}
