#include <gtest/gtest.h>
#include <rtabmap/core/GPS.h>
#include <rtabmap/core/GeodeticCoords.h>
#include <cmath>

using namespace rtabmap;

namespace {
constexpr double kTestLatitude = 45.37855;
constexpr double kTestLongitude = -71.94304;
} // namespace

TEST(GPSTest, DefaultConstructorZeros)
{
	const GPS gps;
	EXPECT_DOUBLE_EQ(gps.stamp(), 0.0);
	EXPECT_DOUBLE_EQ(gps.longitude(), 0.0);
	EXPECT_DOUBLE_EQ(gps.latitude(), 0.0);
	EXPECT_DOUBLE_EQ(gps.altitude(), 0.0);
	EXPECT_DOUBLE_EQ(gps.error(), 0.0);
	EXPECT_DOUBLE_EQ(gps.bearing(), 0.0);
}

TEST(GPSTest, ParameterizedConstructorStoresFields)
{
	const double stamp = 12345.678;
	const double altitude = 36.5;
	const double error = 2.5;
	const double bearing = 127.25;

	const GPS gps(stamp, kTestLongitude, kTestLatitude, altitude, error, bearing);

	EXPECT_DOUBLE_EQ(gps.stamp(), stamp);
	EXPECT_DOUBLE_EQ(gps.longitude(), kTestLongitude);
	EXPECT_DOUBLE_EQ(gps.latitude(), kTestLatitude);
	EXPECT_DOUBLE_EQ(gps.altitude(), altitude);
	EXPECT_DOUBLE_EQ(gps.error(), error);
	EXPECT_DOUBLE_EQ(gps.bearing(), bearing);
}

TEST(GPSTest, ToGeodeticCoordsMapsLatLonAlt)
{
	const GPS gps(1.0, kTestLongitude, kTestLatitude, 250.0, 1.0, 90.0);
	const GeodeticCoords coords = gps.toGeodeticCoords();

	EXPECT_DOUBLE_EQ(coords.latitude(), gps.latitude());
	EXPECT_DOUBLE_EQ(coords.longitude(), gps.longitude());
	EXPECT_DOUBLE_EQ(coords.altitude(), gps.altitude());
}

TEST(GPSTest, ToGeodeticCoordsGeocentricRoundTrip)
{
	const GPS gps(0.0, kTestLongitude, kTestLatitude, 36.5, 0.0, 0.0);
	const GeodeticCoords coords = gps.toGeodeticCoords();
	const cv::Point3d geocentric = coords.toGeocentric_WGS84();

	GeodeticCoords restored;
	restored.fromGeocentric_WGS84(geocentric);

	EXPECT_NEAR(restored.latitude(), gps.latitude(), 1e-9);
	EXPECT_NEAR(restored.longitude(), gps.longitude(), 1e-9);
	EXPECT_NEAR(restored.altitude(), gps.altitude(), 1e-3);
}

TEST(GPSTest, ToGeodeticCoordsEnuRelativeToOrigin)
{
	const GPS origin(0.0, kTestLongitude, kTestLatitude, 0.0, 0.0, 0.0);
	const GPS offset(0.0, kTestLongitude + 0.0003, kTestLatitude + 0.0003, 10.0, 0.0, 0.0);

	const GeodeticCoords originCoords = origin.toGeodeticCoords();
	const GeodeticCoords offsetCoords = offset.toGeodeticCoords();
	const cv::Point3d enu = offsetCoords.toENU_WGS84(originCoords);

	EXPECT_GT(enu.x, 0.0); // east
	EXPECT_GT(enu.y, 0.0); // north
	EXPECT_NEAR(enu.z, 10.0, 0.5);
}
