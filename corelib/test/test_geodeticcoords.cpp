#include <gtest/gtest.h>
#include <rtabmap/core/GeodeticCoords.h>
#include <cmath>

using namespace rtabmap;

namespace {
constexpr double kTestLatitude = 45.37855;
constexpr double kTestLongitude = -71.94304;
} // namespace

static GeodeticCoords makeReferenceOrigin()
{
	return GeodeticCoords(kTestLatitude, kTestLongitude, 0.0);
}

static GeodeticCoords makeReferenceOffset()
{
	return GeodeticCoords(kTestLatitude + 0.0003, kTestLongitude + 0.0003, 10.0);
}

TEST(GeodeticCoordsTest, DefaultConstructorZeros)
{
	const GeodeticCoords coords;
	EXPECT_DOUBLE_EQ(coords.latitude(), 0.0);
	EXPECT_DOUBLE_EQ(coords.longitude(), 0.0);
	EXPECT_DOUBLE_EQ(coords.altitude(), 0.0);
}

TEST(GeodeticCoordsTest, ParameterizedConstructorStoresFields)
{
	const double altitude = 36.5;

	const GeodeticCoords coords(kTestLatitude, kTestLongitude, altitude);

	EXPECT_DOUBLE_EQ(coords.latitude(), kTestLatitude);
	EXPECT_DOUBLE_EQ(coords.longitude(), kTestLongitude);
	EXPECT_DOUBLE_EQ(coords.altitude(), altitude);
}

TEST(GeodeticCoordsTest, SettersUpdateFields)
{
	GeodeticCoords coords;
	coords.setLatitude(1.0);
	coords.setLongitude(2.0);
	coords.setAltitude(3.0);

	EXPECT_DOUBLE_EQ(coords.latitude(), 1.0);
	EXPECT_DOUBLE_EQ(coords.longitude(), 2.0);
	EXPECT_DOUBLE_EQ(coords.altitude(), 3.0);
}

TEST(GeodeticCoordsTest, ToGeocentricWgs84RoundTrip)
{
	const GeodeticCoords coords = makeReferenceOffset();
	const cv::Point3d geocentric = coords.toGeocentric_WGS84();

	GeodeticCoords restored;
	restored.fromGeocentric_WGS84(geocentric);

	EXPECT_NEAR(restored.latitude(), coords.latitude(), 1e-9);
	EXPECT_NEAR(restored.longitude(), coords.longitude(), 1e-9);
	EXPECT_NEAR(restored.altitude(), coords.altitude(), 1e-3);
}

TEST(GeodeticCoordsTest, ToEnuWgs84RelativeToOrigin)
{
	const GeodeticCoords origin = makeReferenceOrigin();
	const GeodeticCoords offset = makeReferenceOffset();
	const cv::Point3d enu = offset.toENU_WGS84(origin);

	EXPECT_GT(enu.x, 0.0); // east
	EXPECT_GT(enu.y, 0.0); // north
	EXPECT_NEAR(enu.z, 10.0, 0.5);
}

TEST(GeodeticCoordsTest, OriginHasZeroEnu)
{
	const GeodeticCoords origin = makeReferenceOrigin();
	const cv::Point3d enu = origin.toENU_WGS84(origin);

	EXPECT_NEAR(enu.x, 0.0, 1e-6);
	EXPECT_NEAR(enu.y, 0.0, 1e-6);
	EXPECT_NEAR(enu.z, 0.0, 1e-6);
}

TEST(GeodeticCoordsTest, FromEnuWgs84RoundTrip)
{
	const GeodeticCoords origin = makeReferenceOrigin();
	const GeodeticCoords expected = makeReferenceOffset();
	const cv::Point3d enu = expected.toENU_WGS84(origin);

	GeodeticCoords restored;
	restored.fromENU_WGS84(enu, origin);

	EXPECT_NEAR(restored.latitude(), expected.latitude(), 1e-5);
	EXPECT_NEAR(restored.longitude(), expected.longitude(), 1e-5);
	EXPECT_NEAR(restored.altitude(), expected.altitude(), 0.1);
}

TEST(GeodeticCoordsTest, StaticGeocentricToEnuMatchesMemberMethod)
{
	const GeodeticCoords origin = makeReferenceOrigin();
	const GeodeticCoords point = makeReferenceOffset();
	const cv::Point3d geocentric = point.toGeocentric_WGS84();
	const cv::Point3d originGeocentric = origin.toGeocentric_WGS84();

	const cv::Point3d enuMember = point.toENU_WGS84(origin);
	const cv::Point3d enuStatic = GeodeticCoords::Geocentric_WGS84ToENU_WGS84(
			geocentric,
			originGeocentric,
			origin);

	EXPECT_NEAR(enuStatic.x, enuMember.x, 1e-6);
	EXPECT_NEAR(enuStatic.y, enuMember.y, 1e-6);
	EXPECT_NEAR(enuStatic.z, enuMember.z, 1e-6);
}

TEST(GeodeticCoordsTest, StaticEnuToGeocentricMatchesMemberPath)
{
	const GeodeticCoords origin = makeReferenceOrigin();
	const GeodeticCoords expected = makeReferenceOffset();
	const cv::Point3d enu = expected.toENU_WGS84(origin);

	const cv::Point3d geocentricStatic = GeodeticCoords::ENU_WGS84ToGeocentric_WGS84(enu, origin);

	GeodeticCoords restored;
	restored.fromGeocentric_WGS84(geocentricStatic);

	EXPECT_NEAR(restored.latitude(), expected.latitude(), 1e-5);
	EXPECT_NEAR(restored.longitude(), expected.longitude(), 1e-5);
	EXPECT_NEAR(restored.altitude(), expected.altitude(), 0.1);
}
