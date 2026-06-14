#include <gtest/gtest.h>
#include <rtabmap/core/LocalGridMaker.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/utilite/UException.h>
#include <rtabmap/utilite/UConversion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace rtabmap;

namespace {

static ParametersMap gridParamsForLaser2D()
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kGridSensor(), "0"));
	params.insert(ParametersPair(Parameters::kGridCellSize(), "0.1"));
	params.insert(ParametersPair(Parameters::kGridScan2dUnknownSpaceFilled(), "false"));
	return params;
}

static ParametersMap gridParamsFor3DPassthrough()
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kGridSensor(), "0"));
	params.insert(ParametersPair(Parameters::kGridCellSize(), "0.1"));
	params.insert(ParametersPair(Parameters::kGrid3D(), "false"));
	params.insert(ParametersPair(Parameters::kGridNormalsSegmentation(), "false"));
	params.insert(ParametersPair(Parameters::kGridPreVoxelFiltering(), "false"));
	params.insert(ParametersPair(Parameters::kGridMinGroundHeight(), "0.0"));
	params.insert(ParametersPair(Parameters::kGridMaxGroundHeight(), "0.1"));
	params.insert(ParametersPair(Parameters::kGridMaxObstacleHeight(), "5.0"));
	params.insert(ParametersPair(Parameters::kGridRayTracing(), "false"));
	return params;
}

static ParametersMap gridParamsForPassthroughSegmentation()
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kGridNormalsSegmentation(), "false"));
	params.insert(ParametersPair(Parameters::kGridPreVoxelFiltering(), "false"));
	params.insert(ParametersPair(Parameters::kGridCellSize(), "0.1"));
	return params;
}

static LaserScan laserScan2DHits()
{
	cv::Mat data(1, 3, CV_32FC2);
	data.at<cv::Vec2f>(0, 0) = cv::Vec2f(2.f, 0.f);
	data.at<cv::Vec2f>(0, 1) = cv::Vec2f(2.f, 1.f);
	data.at<cv::Vec2f>(0, 2) = cv::Vec2f(1.5f, 0.5f);
	return LaserScan(
			data,
			LaserScan::kXY,
			0.f,
			5.f,
			-0.5f,
			0.5f,
			0.5f,
			Transform::getIdentity());
}

static LaserScan laserScan3DGroundAndObstacle()
{
	cv::Mat data(1, 4, CV_32FC3);
	data.at<cv::Vec3f>(0, 0) = cv::Vec3f(1.f, 0.f, 0.f);
	data.at<cv::Vec3f>(0, 1) = cv::Vec3f(2.f, 0.f, 0.f);
	data.at<cv::Vec3f>(0, 2) = cv::Vec3f(3.f, 0.f, 0.f);
	data.at<cv::Vec3f>(0, 3) = cv::Vec3f(2.f, 0.f, 0.5f);
	return LaserScan(data, 0, 10.f, LaserScan::kXYZ, Transform::getIdentity());
}

// Single obstacle on +x bore sight (y=0); one scan per obstacle in CreateLocalMap3DRayTracingWithRangeMax.
static LaserScan laserScan3DRayTracingObstacle(float obstacleX, float obstacleZ = 0.4f)
{
	cv::Mat data(1, 1, CV_32FC3);
	data.at<cv::Vec3f>(0, 0) = cv::Vec3f(obstacleX, 0.f, obstacleZ);
	return LaserScan(data, 0, 10.f, LaserScan::kXYZ, Transform::getIdentity());
}

static ParametersMap gridParams3DLaserRayTracing(
		bool rayTracing,
		float rangeMax = 3.f)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kGridSensor(), "0"));
	params.insert(ParametersPair(Parameters::kGridCellSize(), "0.1"));
	params.insert(ParametersPair(Parameters::kGrid3D(), "true"));
	params.insert(ParametersPair(Parameters::kGridRayTracing(), rayTracing ? "true" : "false"));
	params.insert(ParametersPair(Parameters::kGridRangeMax(), uNumber2Str(rangeMax)));
	params.insert(ParametersPair(Parameters::kGridNormalsSegmentation(), "false"));
	params.insert(ParametersPair(Parameters::kGridPreVoxelFiltering(), "false"));
	params.insert(ParametersPair(Parameters::kGridMaxGroundHeight(), "0.15"));
	params.insert(ParametersPair(Parameters::kGridMaxObstacleHeight(), "5.0"));
	return params;
}

static void createLocalMapFrom3DScan(
		const ParametersMap & params,
		const LaserScan & scan,
		cv::Mat & ground,
		cv::Mat & obstacles,
		cv::Mat & empty)
{
	LocalGridMaker maker(params);
	SensorData data;
	data.setLaserScan(scan);
	const Signature node(data);
	cv::Point3f viewPoint;
	maker.createLocalMap(node, ground, obstacles, empty, viewPoint);
}

// Count empty cells in an x interval on the bore sight (y ~= 0).
static int countEmptyCellsInXRange(
		const cv::Mat & empty,
		float xMin,
		float xMax,
		float yTolerance = 0.15f)
{
	if(empty.empty())
	{
		return 0;
	}

	int count = 0;
	if(empty.type() == CV_32FC3)
	{
		for(int i = 0; i < empty.cols; ++i)
		{
			const cv::Vec3f & p = empty.at<cv::Vec3f>(0, i);
			if(p[0] >= xMin && p[0] <= xMax && std::abs(p[1]) <= yTolerance)
			{
				++count;
			}
		}
	}
	else if(empty.type() == CV_32FC2)
	{
		for(int i = 0; i < empty.cols; ++i)
		{
			const cv::Vec2f & p = empty.at<cv::Vec2f>(0, i);
			if(p[0] >= xMin && p[0] <= xMax && std::abs(p[1]) <= yTolerance)
			{
				++count;
			}
		}
	}
	return count;
}

static float maxObstacleX(const cv::Mat & obstacles)
{
	float maxX = -std::numeric_limits<float>::infinity();
	if(obstacles.empty())
	{
		return maxX;
	}
	if(obstacles.type() == CV_32FC3)
	{
		for(int i = 0; i < obstacles.cols; ++i)
		{
			maxX = std::max(maxX, obstacles.at<cv::Vec3f>(0, i)[0]);
		}
	}
	else if(obstacles.type() == CV_32FC2)
	{
		for(int i = 0; i < obstacles.cols; ++i)
		{
			maxX = std::max(maxX, obstacles.at<cv::Vec2f>(0, i)[0]);
		}
	}
	return maxX;
}

static ParametersMap gridParamsForNormalSegmentation(
		float maxObstacleHeight,
		bool flatObstaclesDetected = false)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kGridNormalsSegmentation(), "true"));
	params.insert(ParametersPair(Parameters::kGridPreVoxelFiltering(), "false"));
	params.insert(ParametersPair(Parameters::kGridNormalK(), "10"));
	params.insert(ParametersPair(Parameters::kGridMinClusterSize(), "1"));
	params.insert(ParametersPair(Parameters::kGridClusterRadius(), "0.15"));
	params.insert(ParametersPair(Parameters::kGridMinGroundHeight(), "0.0"));
	params.insert(ParametersPair(Parameters::kGridMaxGroundHeight(), "0.0"));
	params.insert(ParametersPair(Parameters::kGridFlatObstacleDetected(), flatObstaclesDetected?"true":"false"));
	params.insert(ParametersPair(Parameters::kGridMaxObstacleHeight(), uNumber2Str(maxObstacleHeight)));
	return params;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr planeGrid(
		float z,
		int pointsPerSide,
		float step)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->reserve(pointsPerSide * pointsPerSide);
	for(int i = 0; i < pointsPerSide; ++i)
	{
		for(int j = 0; j < pointsPerSide; ++j)
		{
			cloud->push_back(pcl::PointXYZ(i * step, j * step, z));
		}
	}
	return cloud;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr groundAndCeilingCloud()
{
	const int pointsPerSide = 7;
	const float step = 0.08f;
	const float groundZ = 0.f;
	const float ceilingZ = 2.f;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = planeGrid(groundZ, pointsPerSide, step);
	const pcl::PointCloud<pcl::PointXYZ>::Ptr ceiling = planeGrid(ceilingZ, pointsPerSide, step);
	cloud->insert(cloud->end(), ceiling->begin(), ceiling->end());
	return cloud;
}

// Floor grid with a raised horizontal platform (flat obstacle) in one corner.
static pcl::PointCloud<pcl::PointXYZ>::Ptr groundWithFlatStepCloud(
		int & pointsPerSideOut,
		float & stepOut,
		size_t & flatStepPointsOut)
{
	const int pointsPerSide = 7;
	const int flatStepStart = 3;
	const float step = 0.08f;
	const float flatStepZ = 0.25f;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = planeGrid(0.f, pointsPerSide, step);
	size_t flatStepPoints = 0;
	for(int i = flatStepStart; i < pointsPerSide; ++i)
	{
		for(int j = flatStepStart; j < pointsPerSide; ++j)
		{
			cloud->push_back(pcl::PointXYZ(i * step, j * step, flatStepZ));
			++flatStepPoints;
		}
	}

	pointsPerSideOut = pointsPerSide;
	stepOut = step;
	flatStepPointsOut = flatStepPoints;
	return cloud;
}

// Floor grid plus a vertical wall (y = -0.15).
static pcl::PointCloud<pcl::PointXYZ>::Ptr floorWallCloud(
		size_t & floorPointsOut,
		size_t & wallPointsOut,
		size_t * smallObstaclePointsOut = 0)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	const int gridSide = 10;
	const float step = 0.05f;
	size_t floorPoints = 0;
	for(int i = 0; i < gridSide; ++i)
	{
		for(int j = 0; j < gridSide; ++j)
		{
			cloud->push_back(pcl::PointXYZ(step * i, step * j, 0.f));
			++floorPoints;
		}
	}
	size_t wallPoints = 0;
	for(int i = 0; i < gridSide; ++i)
	{
		for(int k = 0; k < gridSide; ++k)
		{
			cloud->push_back(pcl::PointXYZ(step * i, -0.15f, step * k));
			++wallPoints;
		}
	}
	size_t smallObstaclePoints = 0;
	if(smallObstaclePointsOut)
	{
		for(int i = 6; i < gridSide; ++i)
		{
			for(int k = 6; k < gridSide; ++k)
			{
				cloud->push_back(pcl::PointXYZ(step * i, -0.35f, step * k));
				++smallObstaclePoints;
			}
		}
		*smallObstaclePointsOut = smallObstaclePoints;
	}

	floorPointsOut = floorPoints;
	wallPointsOut = wallPoints;
	return cloud;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr floorWithNoiseOutlierCloud(size_t & floorPointsOut)
{
	size_t wallPoints = 0;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = floorWallCloud(floorPointsOut, wallPoints);
	cloud->push_back(pcl::PointXYZ(5.f, 5.f, 0.f));
	return cloud;
}

struct SegmentCloudResult
{
	size_t groundCount = 0;
	size_t obstacleCount = 0;
	size_t flatCount = 0;
	size_t returnedCloudSize = 0;
};

static SegmentCloudResult runSegmentCloud(
		const ParametersMap & params,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indicesIn,
		const Transform & pose,
		const cv::Point3f & viewPoint,
		pcl::IndicesPtr * flatObstaclesOut = 0)
{
	const LocalGridMaker maker(params);
	pcl::IndicesPtr indices = indicesIn;
	if(!indices.get())
	{
		indices.reset(new std::vector<int>);
	}
	pcl::IndicesPtr groundIndices;
	pcl::IndicesPtr obstaclesIndices;
	pcl::IndicesPtr flatObstacles;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr returned = maker.segmentCloud<pcl::PointXYZ>(
			cloud,
			indices,
			pose,
			viewPoint,
			groundIndices,
			obstaclesIndices,
			flatObstaclesOut ? &flatObstacles : 0);

	SegmentCloudResult result;
	if(groundIndices.get())
	{
		result.groundCount = groundIndices->size();
	}
	if(obstaclesIndices.get())
	{
		result.obstacleCount = obstaclesIndices->size();
	}
	if(flatObstaclesOut && flatObstacles.get())
	{
		result.flatCount = flatObstacles->size();
		*flatObstaclesOut = flatObstacles;
	}
	result.returnedCloudSize = returned? returned->size() : 0u;
	return result;
}

static void segmentCloudNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const cv::Point3f & viewPoint,
		bool flatObstaclesDetected,
		float maxObstacleHeight,
		size_t & groundCount,
		size_t & obstacleCount,
		size_t * flatObstacleCount = 0)
{
	const LocalGridMaker maker(gridParamsForNormalSegmentation(maxObstacleHeight, flatObstaclesDetected));
	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::IndicesPtr groundIndices;
	pcl::IndicesPtr obstaclesIndices;
	pcl::IndicesPtr flatObstacles;

	maker.segmentCloud<pcl::PointXYZ>(
			cloud,
			indices,
			Transform::getIdentity(),
			viewPoint,
			groundIndices,
			obstaclesIndices,
			flatObstacleCount ? &flatObstacles : 0);

	ASSERT_TRUE(groundIndices.get());
	ASSERT_TRUE(obstaclesIndices.get());
	groundCount = groundIndices->size();
	obstacleCount = obstaclesIndices->size();
	if(flatObstacleCount)
	{
		ASSERT_TRUE(flatObstacles.get());
		*flatObstacleCount = flatObstacles->size();
	}
}

static void segmentGroundAndCeiling(
		float maxObstacleHeight,
		size_t & groundCount,
		size_t & obstacleCount)
{
	const int pointsPerSide = 7;
	const float step = 0.08f;
	const cv::Point3f viewPoint(
			(pointsPerSide - 1) * step * 0.5f,
			(pointsPerSide - 1) * step * 0.5f,
			1.f);

	segmentCloudNormals(
			groundAndCeilingCloud(),
			viewPoint,
			false,
			maxObstacleHeight,
			groundCount,
			obstacleCount);
}

} // namespace

TEST(LocalGridMakerTest, DefaultParameters)
{
	const LocalGridMaker maker;
	EXPECT_FLOAT_EQ(maker.getCellSize(), Parameters::defaultGridCellSize());
	EXPECT_TRUE(maker.isGridFromDepth());
	EXPECT_FALSE(maker.isMapFrameProjection());
}

TEST(LocalGridMakerTest, ParseParametersUpdatesAccessors)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kGridCellSize(), "0.2"));
	params.insert(ParametersPair(Parameters::kGridSensor(), "0"));
	params.insert(ParametersPair(Parameters::kGridMapFrameProjection(), "true"));

	LocalGridMaker maker(params);
	EXPECT_FLOAT_EQ(maker.getCellSize(), 0.2f);
	EXPECT_FALSE(maker.isGridFromDepth());
	EXPECT_TRUE(maker.isMapFrameProjection());

	ParametersMap reset;
	reset.insert(ParametersPair(Parameters::kGridCellSize(), uNumber2Str(Parameters::defaultGridCellSize())));
	maker.parseParameters(reset);
	EXPECT_FLOAT_EQ(maker.getCellSize(), Parameters::defaultGridCellSize());
}

TEST(LocalGridMakerTest, ParseParametersRejectsInvalidCellSize)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kGridCellSize(), "0"));
	LocalGridMaker maker;
	ASSERT_THROW(maker.parseParameters(params), UException);
}

TEST(LocalGridMakerTest, CreateLocalMapFrom2DLaserScan)
{
	LocalGridMaker maker(gridParamsForLaser2D());

	SensorData data;
	data.setLaserScan(laserScan2DHits());
	const Signature node(data);

	cv::Mat ground;
	cv::Mat obstacles;
	cv::Mat empty;
	cv::Point3f viewPoint;
	maker.createLocalMap(node, ground, obstacles, empty, viewPoint);

	EXPECT_FALSE(obstacles.empty());
	EXPECT_EQ(obstacles.type(), CV_32FC2);
	EXPECT_GE(obstacles.cols, 3);
}

TEST(LocalGridMakerTest, CreateLocalMapFrom3DScanPassthrough)
{
	const LocalGridMaker maker(gridParamsFor3DPassthrough());
	const LaserScan scan = laserScan3DGroundAndObstacle();

	cv::Mat ground;
	cv::Mat obstacles;
	cv::Mat empty;
	cv::Point3f viewPoint(0.f, 0.f, 0.f);
	maker.createLocalMap(scan, Transform::getIdentity(), ground, obstacles, empty, viewPoint);

	EXPECT_FALSE(ground.empty());
	EXPECT_FALSE(obstacles.empty());
	EXPECT_EQ(ground.cols, 3);
	EXPECT_EQ(obstacles.cols, 1);
	EXPECT_EQ(ground.type(), CV_32FC2);
	EXPECT_EQ(obstacles.type(), CV_32FC2);
}

TEST(LocalGridMakerTest, SegmentCloudSplitsByHeight)
{
	const LocalGridMaker maker(gridParamsFor3DPassthrough());

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->push_back(pcl::PointXYZ(1.f, 0.f, 0.f));
	cloud->push_back(pcl::PointXYZ(2.f, 0.f, 0.f));
	cloud->push_back(pcl::PointXYZ(3.f, 0.f, 0.f));
	cloud->push_back(pcl::PointXYZ(2.f, 0.f, 0.5f));

	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::IndicesPtr groundIndices;
	pcl::IndicesPtr obstaclesIndices;
	const cv::Point3f viewPoint(0.f, 0.f, 0.f);

	maker.segmentCloud<pcl::PointXYZ>(
			cloud,
			indices,
			Transform::getIdentity(),
			viewPoint,
			groundIndices,
			obstaclesIndices);

	ASSERT_TRUE(groundIndices.get());
	ASSERT_TRUE(obstaclesIndices.get());
	EXPECT_EQ(groundIndices->size(), 3u);
	EXPECT_EQ(obstaclesIndices->size(), 1u);
}

TEST(LocalGridMakerTest, NormalSegmentationGroundAndCeiling)
{
	const size_t expectedPlanePoints = 7u * 7u;
	size_t groundCount = 0;
	size_t obstacleCount = 0;

	segmentGroundAndCeiling(0.f, groundCount, obstacleCount);
	EXPECT_EQ(groundCount, expectedPlanePoints);
	EXPECT_EQ(obstacleCount, expectedPlanePoints);
}

TEST(LocalGridMakerTest, NormalSegmentationMaxObstacleHeightFiltersCeiling)
{
	const size_t expectedPlanePoints = 7u * 7u;
	size_t groundCount = 0;
	size_t obstacleCount = 0;

	// Between ground (z=0) and ceiling (z=2): drop points above 1.5 m before segmentation.
	segmentGroundAndCeiling(1.5f, groundCount, obstacleCount);
	EXPECT_EQ(groundCount, expectedPlanePoints);
	EXPECT_EQ(obstacleCount, 0u);
}

TEST(LocalGridMakerTest, NormalSegmentationFlatObstaclesDisabled)
{
	int pointsPerSide = 0;
	float step = 0.f;
	size_t flatStepPoints = 0;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = groundWithFlatStepCloud(
			pointsPerSide,
			step,
			flatStepPoints);
	const cv::Point3f viewPoint(
			(pointsPerSide - 1) * step * 0.5f,
			(pointsPerSide - 1) * step * 0.5f,
			2.f);

	size_t groundCount = 0;
	size_t obstacleCount = 0;
	size_t flatCount = 0;
	segmentCloudNormals(cloud, viewPoint, false, 0.f, groundCount, obstacleCount, &flatCount);

	EXPECT_EQ(groundCount, size_t(pointsPerSide * pointsPerSide) + flatStepPoints);
	EXPECT_EQ(flatCount, 0u);
	EXPECT_EQ(obstacleCount, 0u);
}

TEST(LocalGridMakerTest, NormalSegmentationFlatObstaclesDetected)
{
	int pointsPerSide = 0;
	float step = 0.f;
	size_t flatStepPoints = 0;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = groundWithFlatStepCloud(
			pointsPerSide,
			step,
			flatStepPoints);
	const cv::Point3f viewPoint(
			(pointsPerSide - 1) * step * 0.5f,
			(pointsPerSide - 1) * step * 0.5f,
			2.f);

	size_t groundCount = 0;
	size_t obstacleCount = 0;
	size_t flatCount = 0;
	segmentCloudNormals(cloud, viewPoint, true, 0.f, groundCount, obstacleCount, &flatCount);

	EXPECT_EQ(groundCount, size_t(pointsPerSide * pointsPerSide));
	EXPECT_EQ(flatCount, flatStepPoints);
	EXPECT_EQ(obstacleCount, flatStepPoints);
}

TEST(LocalGridMakerTest, SegmentCloudFootprintRemovesRobotBox)
{
	ParametersMap params = gridParamsFor3DPassthrough();
	params.insert(ParametersPair(Parameters::kGridFootprintLength(), "1.0"));
	params.insert(ParametersPair(Parameters::kGridFootprintWidth(), "1.0"));
	params.insert(ParametersPair(Parameters::kGridFootprintHeight(), "2.0"));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->push_back(pcl::PointXYZ(0.f, 0.f, 0.5f));   // inside footprint
	cloud->push_back(pcl::PointXYZ(2.f, 0.f, 0.f));    // outside footprint
	cloud->push_back(pcl::PointXYZ(2.f, 0.f, 0.5f));   // outside footprint (obstacle height)

	const SegmentCloudResult result = runSegmentCloud(
			params,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.f, 0.f, 0.f));

	EXPECT_EQ(result.groundCount + result.obstacleCount, 2u);
	EXPECT_EQ(result.groundCount, 1u);
	EXPECT_EQ(result.obstacleCount, 1u);
}

TEST(LocalGridMakerTest, SegmentCloudPreVoxelFilteringReducesPoints)
{
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = planeGrid(0.f, 20, 0.05f);
	const pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized = util3d::voxelize(
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			2.0f);
	ASSERT_LT(voxelized->size(), cloud->size());

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kGridNormalsSegmentation(), "false"));
	params.insert(ParametersPair(Parameters::kGridCellSize(), "2.0"));
	params.insert(ParametersPair(Parameters::kGridMaxGroundHeight(), "10.0"));

	ParametersMap paramsOff = params;
	paramsOff.insert(ParametersPair(Parameters::kGridPreVoxelFiltering(), "false"));
	const SegmentCloudResult withoutVoxel = runSegmentCloud(
			paramsOff,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.f, 0.f, 2.f));

	ParametersMap paramsOn = params;
	paramsOn.insert(ParametersPair(Parameters::kGridPreVoxelFiltering(), "true"));

	const SegmentCloudResult withVoxel = runSegmentCloud(
			paramsOn,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.f, 0.f, 2.f));

	EXPECT_EQ(withoutVoxel.returnedCloudSize, cloud->size());
	EXPECT_EQ(withVoxel.returnedCloudSize, voxelized->size());
	EXPECT_LT(withVoxel.returnedCloudSize, cloud->size());
}

TEST(LocalGridMakerTest, SegmentCloudNoiseFilteringRemovesOutlier)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud = planeGrid(0.f, 10, 0.05f);
	const size_t floorPoints = cloud->size();
	cloud->push_back(pcl::PointXYZ(3.f, 3.f, 0.f));

	ParametersMap params = gridParamsForPassthroughSegmentation();
	params.insert(ParametersPair(Parameters::kGridMaxGroundHeight(), "10.0"));
	params.insert(ParametersPair(Parameters::kGridNoiseFilteringRadius(), "0.15"));
	params.insert(ParametersPair(Parameters::kGridNoiseFilteringMinNeighbors(), "5"));

	const SegmentCloudResult withoutNoise = runSegmentCloud(
			gridParamsForPassthroughSegmentation(),
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.25f, 0.25f, 2.f));
	const SegmentCloudResult withNoise = runSegmentCloud(
			params,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.25f, 0.25f, 2.f));

	EXPECT_EQ(withoutNoise.groundCount, floorPoints + 1u);
	EXPECT_EQ(withNoise.groundCount, floorPoints);
	EXPECT_EQ(withNoise.obstacleCount, 0u);
}

TEST(LocalGridMakerTest, SegmentCloudGroundIsObstacleUsesPassthrough)
{
	ParametersMap params = gridParamsForNormalSegmentation(0.f, false);
	params.insert(ParametersPair(Parameters::kGridGroundIsObstacle(), "true"));

	const int pointsPerSide = 7;
	const float step = 0.08f;
	const cv::Point3f viewPoint(
			(pointsPerSide - 1) * step * 0.5f,
			(pointsPerSide - 1) * step * 0.5f,
			1.f);

	const SegmentCloudResult result = runSegmentCloud(
			params,
			groundAndCeilingCloud(),
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			viewPoint);

	// Passthrough with disabled max ground height: all points classified as ground.
	EXPECT_EQ(result.groundCount, 2u * pointsPerSide * pointsPerSide);
	EXPECT_EQ(result.obstacleCount, 0u);
}

TEST(LocalGridMakerTest, SegmentCloudPartialIndices)
{
	ParametersMap params = gridParamsForNormalSegmentation(0.f, false);

	size_t floorPoints = 0;
	size_t wallPoints = 0;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = floorWallCloud(floorPoints, wallPoints);
	pcl::IndicesPtr subset(new std::vector<int>);
	for(size_t i = 0; i < floorPoints; ++i)
	{
		subset->push_back(int(i));
	}

	const SegmentCloudResult result = runSegmentCloud(
			params,
			cloud,
			subset,
			Transform::getIdentity(),
			cv::Point3f(0.25f, 0.25f, 2.f));

	EXPECT_EQ(result.groundCount, floorPoints);
	EXPECT_EQ(result.obstacleCount, 0u);
}

TEST(LocalGridMakerTest, SegmentCloudMapFrameProjectionWithTiltedPose)
{
	ParametersMap paramsOn;
	paramsOn.insert(ParametersPair(Parameters::kGridMapFrameProjection(), "true"));
	EXPECT_TRUE(LocalGridMaker(paramsOn).isMapFrameProjection());

	ParametersMap paramsOff;
	paramsOff.insert(ParametersPair(Parameters::kGridMapFrameProjection(), "false"));
	EXPECT_FALSE(LocalGridMaker(paramsOff).isMapFrameProjection());

	// pose.z is applied to the cloud transform only when map-frame projection is on.
	ParametersMap segmentOn = gridParamsForPassthroughSegmentation();
	segmentOn.insert(ParametersPair(Parameters::kGridMapFrameProjection(), "true"));
	segmentOn.insert(ParametersPair(Parameters::kGridMaxGroundHeight(), "1.5"));

	ParametersMap segmentOff = gridParamsForPassthroughSegmentation();
	segmentOff.insert(ParametersPair(Parameters::kGridMapFrameProjection(), "false"));
	segmentOff.insert(ParametersPair(Parameters::kGridMaxGroundHeight(), "1.5"));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->push_back(pcl::PointXYZ(0.f, 0.f, 0.f));
	const Transform pose(0.f, 0.f, 2.f, 0.f, float(M_PI / 6.f), 0.f);

	const SegmentCloudResult withProjection = runSegmentCloud(
			segmentOn,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			pose,
			cv::Point3f(0.f, 0.f, 0.f));
	const SegmentCloudResult withoutProjection = runSegmentCloud(
			segmentOff,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			pose,
			cv::Point3f(0.f, 0.f, 0.f));

	EXPECT_NE(withProjection.groundCount, withoutProjection.groundCount);
}

TEST(LocalGridMakerTest, SegmentCloudMaxGroundHeightMergesLowSurfaces)
{
	ParametersMap params = gridParamsForNormalSegmentation(0.f, false);
	params.insert(ParametersPair(Parameters::kGridMaxGroundHeight(), "0.1"));

	int pointsPerSide = 0;
	float step = 0.f;
	size_t flatStepPoints = 0;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = groundWithFlatStepCloud(
			pointsPerSide,
			step,
			flatStepPoints);
	const cv::Point3f viewPoint(
			(pointsPerSide - 1) * step * 0.5f,
			(pointsPerSide - 1) * step * 0.5f,
			2.f);

	const SegmentCloudResult result = runSegmentCloud(
			params,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			viewPoint);

	EXPECT_EQ(result.groundCount, size_t(pointsPerSide * pointsPerSide) + flatStepPoints);
	EXPECT_EQ(result.obstacleCount, 0u);
}

TEST(LocalGridMakerTest, SegmentCloudMinGroundHeightFiltersLowPoints)
{
	ParametersMap params = gridParamsForPassthroughSegmentation();
	params.insert(ParametersPair(Parameters::kGridMinGroundHeight(), "0.05"));
	params.insert(ParametersPair(Parameters::kGridMaxObstacleHeight(), "0.2"));
	params.insert(ParametersPair(Parameters::kGridMaxGroundHeight(), "0.2"));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->push_back(pcl::PointXYZ(1.f, 0.f, 0.f));
	cloud->push_back(pcl::PointXYZ(2.f, 0.f, 0.f));
	cloud->push_back(pcl::PointXYZ(2.f, 0.f, 0.1f));

	const SegmentCloudResult result = runSegmentCloud(
			params,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.f, 0.f, 0.f));

	EXPECT_EQ(result.groundCount, 1u);
	EXPECT_EQ(result.obstacleCount, 0u);
}

TEST(LocalGridMakerTest, SegmentCloudVerticalWallIsObstacle)
{
	ParametersMap params = gridParamsForNormalSegmentation(0.f, false);

	size_t floorPoints = 0;
	size_t wallPoints = 0;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = floorWallCloud(floorPoints, wallPoints);

	const SegmentCloudResult result = runSegmentCloud(
			params,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.25f, 0.25f, 2.f));

	EXPECT_EQ(result.groundCount, floorPoints);
	EXPECT_EQ(result.obstacleCount, wallPoints);
}

TEST(LocalGridMakerTest, SegmentCloudMinClusterSizeFiltersSmallClusters)
{
	// Same layout as Util3dMappingTest.SegmentObstaclesFromGround (minClusterSize=17).
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(int i = 0; i < 10; ++i)
	{
		for(int j = 0; j < 10; ++j)
		{
			cloud->push_back(pcl::PointXYZ(0.05f * i, 0.05f * j, 0.01f * i));
		}
	}
	for(int i = 0; i < 10; ++i)
	{
		for(int k = 0; k < 10; ++k)
		{
			if(i > 5 && k > 5)
			{
				cloud->push_back(pcl::PointXYZ(0.05f * i, -0.35f, 0.05f * k));
			}
			cloud->push_back(pcl::PointXYZ(0.05f * i, -0.15f, 0.05f * k));
		}
	}

	pcl::IndicesPtr ground;
	pcl::IndicesPtr obstacles;
	const float angleMax = 20.f * float(M_PI) / 180.f;
	util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
			cloud,
			ground,
			obstacles,
			5,
			angleMax,
			0.1f,
			1,
			false,
			0.f,
			0,
			Eigen::Vector4f(0, 0, 2, 0),
			0.8f);
	const size_t obstaclesMinCluster1 = obstacles->size();

	util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
			cloud,
			ground,
			obstacles,
			5,
			angleMax,
			0.1f,
			17,
			false,
			0.f,
			0,
			Eigen::Vector4f(0, 0, 2, 0),
			0.8f);
	const size_t obstaclesMinCluster17 = obstacles->size();
	ASSERT_GT(obstaclesMinCluster1, obstaclesMinCluster17);

	ParametersMap baseParams = gridParamsForNormalSegmentation(0.f, false);
	baseParams.insert(ParametersPair(Parameters::kGridNormalK(), "5"));
	baseParams.insert(ParametersPair(Parameters::kGridMaxGroundAngle(), "20"));
	baseParams.insert(ParametersPair(Parameters::kIcpPointToPlaneGroundNormalsUp(), "0.8"));
	baseParams.insert(ParametersPair(Parameters::kGridClusterRadius(), "0.1"));
	baseParams.insert(ParametersPair(Parameters::kGridMinClusterSize(), "1"));

	ParametersMap params = baseParams;
	params[Parameters::kGridMinClusterSize()] = "17";

	const SegmentCloudResult withDefault = runSegmentCloud(
			baseParams,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.f, 0.f, 2.f));
	const SegmentCloudResult withLargeMinCluster = runSegmentCloud(
			params,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.f, 0.f, 2.f));

	EXPECT_EQ(withDefault.obstacleCount, obstaclesMinCluster1);
	EXPECT_EQ(withLargeMinCluster.obstacleCount, obstaclesMinCluster17);
}

TEST(LocalGridMakerTest, SegmentCloudEmptyInput)
{
	const SegmentCloudResult result = runSegmentCloud(
			gridParamsForPassthroughSegmentation(),
			pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>),
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.f, 0.f, 0.f));

	EXPECT_EQ(result.returnedCloudSize, 0u);
	EXPECT_EQ(result.groundCount, 0u);
	EXPECT_EQ(result.obstacleCount, 0u);
}

TEST(LocalGridMakerTest, SegmentCloudNoPointsAfterFootprintCrop)
{
	ParametersMap params = gridParamsForPassthroughSegmentation();
	params.insert(ParametersPair(Parameters::kGridFootprintLength(), "2.0"));
	params.insert(ParametersPair(Parameters::kGridFootprintWidth(), "2.0"));
	params.insert(ParametersPair(Parameters::kGridFootprintHeight(), "2.0"));
	params.insert(ParametersPair(Parameters::kGridMaxGroundHeight(), "10.0"));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->push_back(pcl::PointXYZ(0.f, 0.f, 0.f));
	cloud->push_back(pcl::PointXYZ(0.5f, 0.f, 0.f));

	const SegmentCloudResult result = runSegmentCloud(
			params,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.f, 0.f, 0.f));

	EXPECT_EQ(result.groundCount, 0u);
	EXPECT_EQ(result.obstacleCount, 0u);
}

TEST(LocalGridMakerTest, SegmentCloudRangeMaxPreservesFarPointsDuringNoiseFilter)
{
	ParametersMap params = gridParamsForNormalSegmentation(0.f, false);
	params.insert(ParametersPair(Parameters::kGridNoiseFilteringRadius(), "0.12"));
	params.insert(ParametersPair(Parameters::kGridNoiseFilteringMinNeighbors(), "5"));
	params.insert(ParametersPair(Parameters::kGridRangeMax(), "3.0"));

	size_t floorPoints = 0;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = floorWithNoiseOutlierCloud(floorPoints);

	const SegmentCloudResult result = runSegmentCloud(
			params,
			cloud,
			pcl::IndicesPtr(new std::vector<int>),
			Transform::getIdentity(),
			cv::Point3f(0.25f, 0.25f, 2.f));

	// Far outlier kept because it is beyond rangeMax (not noise-filtered).
	EXPECT_EQ(result.groundCount, floorPoints + 1u);
	EXPECT_GT(result.obstacleCount, 0u);
}

// Grid/3D + Grid/RayTracing + OctoMap (2D ray tracing: Util3dMappingTest).
// One obstacle per scan on +x (y=0, z=0.4): near x=2 m, far x=4.5 m (separate point clouds).
//
// Side view (x right, z up). o = sensor, # = obstacle, ~ = empty cells, | = RangeMax.
//         0   2 |3|  4.5
//
// Expected empty-cell counts (Grid/CellSize=0.1, bore sight y~=0):
//   | Scan | RangeMax | x interval   | Empty cells |
//   |------|----------|--------------|-------------|
//   | Near | 3        | [0.1, 1.9]   | 21          |
//   | Far  | 3        | [0.1, 2.9]   | 30          |
//   | Far  | 3        | [3.15, 4.4]  | 0           |
//   | Far  | 5.5      | [3.15, 4.4]  | 13          |
//   | Far  | 5.5      | past x=3.2   | 13          |
TEST(LocalGridMakerTest, CreateLocalMap3DRayTracingWithRangeMax)
{
#ifndef RTABMAP_OCTOMAP
	GTEST_SKIP() << "OctoMap support required for 3D ray tracing.";
#endif

	const float rangeMax = 3.f;
	const float nearObstacleX = 2.f;
	const float farObstacleX = 4.5f;
	const float gapXMin = rangeMax + 0.15f;
	const float gapXMax = farObstacleX - 0.1f;
	const float yTolerance = 0.15f;
	const int nearRayEmpty = 21;
	const int farRayEmptyBeforeClip = 30;
	const int gapEmptyShort = 0;
	const int gapEmptyLong = 13;
	const int beyondRangeEmptyLong = 13;

	const LaserScan nearScan = laserScan3DRayTracingObstacle(nearObstacleX);
	const LaserScan farScan = laserScan3DRayTracingObstacle(farObstacleX);

	cv::Mat ground;
	cv::Mat obstacles;
	cv::Mat empty;

	// RayTracing=false: no empty layer.
	createLocalMapFrom3DScan(
			gridParams3DLaserRayTracing(false, rangeMax),
			nearScan,
			ground,
			obstacles,
			empty);
	EXPECT_TRUE(empty.empty());

	// Near obstacle: free cells along the ray before the hit.
	cv::Mat emptyNear;
	createLocalMapFrom3DScan(
			gridParams3DLaserRayTracing(true, rangeMax),
			nearScan,
			ground,
			obstacles,
			emptyNear);
	ASSERT_FALSE(emptyNear.empty());
	EXPECT_EQ(emptyNear.type(), CV_32FC3);
	EXPECT_EQ(nearRayEmpty,
			countEmptyCellsInXRange(emptyNear, 0.1f, nearObstacleX - 0.1f, yTolerance));

	// Far obstacle, RangeMax=3: ray stops at |; no ~ in gap; hit still in obstacles.
	cv::Mat emptyFarShort;
	createLocalMapFrom3DScan(
			gridParams3DLaserRayTracing(true, rangeMax),
			farScan,
			ground,
			obstacles,
			emptyFarShort);
	ASSERT_FALSE(emptyFarShort.empty());
	EXPECT_EQ(farRayEmptyBeforeClip,
			countEmptyCellsInXRange(emptyFarShort, 0.1f, rangeMax - 0.1f, yTolerance));
	EXPECT_EQ(gapEmptyShort,
			countEmptyCellsInXRange(emptyFarShort, gapXMin, gapXMax, yTolerance));
	EXPECT_EQ(gapEmptyShort,
			countEmptyCellsInXRange(emptyFarShort, rangeMax + 0.2f, 10.f, yTolerance));

	// Far obstacle, RangeMax>4.5: ~ along the ray in the gap and past x=3.2.
	cv::Mat emptyFarLong;
	createLocalMapFrom3DScan(
			gridParams3DLaserRayTracing(true, farObstacleX + 1.f),
			farScan,
			ground,
			obstacles,
			emptyFarLong);
	ASSERT_FALSE(emptyFarLong.empty());
	EXPECT_EQ(gapEmptyLong,
			countEmptyCellsInXRange(emptyFarLong, gapXMin, gapXMax, yTolerance));
	EXPECT_EQ(beyondRangeEmptyLong,
			countEmptyCellsInXRange(emptyFarLong, rangeMax + 0.2f, 10.f, yTolerance));
	// Ray reaches the hit: obstacle cell at x=4.5 (scan not range-filtered when ray tracing is on).
	EXPECT_FLOAT_EQ(farObstacleX, maxObstacleX(obstacles));
}

TEST(LocalGridMakerTest, SegmentCloudPointNormalType)
{
	ParametersMap params = gridParamsFor3DPassthrough();

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointNormal p;
	p.x = 1.f; p.y = 0.f; p.z = 0.f;
	p.normal_x = 0.f; p.normal_y = 0.f; p.normal_z = 1.f;
	cloud->push_back(p);
	p.x = 2.f; p.z = 0.5f;
	cloud->push_back(p);
	p.x = 2.f; p.z = 0.f;
	cloud->push_back(p);

	const LocalGridMaker maker(params);
	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::IndicesPtr groundIndices;
	pcl::IndicesPtr obstaclesIndices;
	const pcl::PointCloud<pcl::PointNormal>::Ptr returned = maker.segmentCloud<pcl::PointNormal>(
			cloud,
			indices,
			Transform::getIdentity(),
			cv::Point3f(0.f, 0.f, 0.f),
			groundIndices,
			obstaclesIndices);

	ASSERT_TRUE(returned.get());
	ASSERT_TRUE(groundIndices.get());
	ASSERT_TRUE(obstaclesIndices.get());
	EXPECT_EQ(returned->size(), 3u);
	EXPECT_EQ(groundIndices->size(), 2u);
	EXPECT_EQ(obstaclesIndices->size(), 1u);
}
