/*
 * Util3D.h
 *      Author: mathieu
 */

#ifndef UTIL3D_H_
#define UTIL3D_H_

#include "rtabmap/core/RtabmapExp.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include <string>

#include <rtabmap/core/Link.h>
#include <rtabmap/utilite/UThread.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

namespace rtabmap
{

class Signature;

namespace util3d
{

/**
 * Compress image or data
 *
 * Example compression:
 *   cv::Mat image;// an image
 *   CompressionThread ct(image);
 *   ct.start();
 *   ct.join();
 *   std::vector<unsigned char> bytes = ct.getCompressedData();
 *
 * Example uncompression
 *   std::vector<unsigned char> bytes;// a compressed image
 *   CompressionThread ct(bytes);
 *   ct.start();
 *   ct.join();
 *   cv::Mat image = ct.getUncompressedData();
 */
class RTABMAP_EXP CompressionThread : public UThread
{
public:
	// format : ".png" ".jpg" "" (empty is general)
	CompressionThread(const cv::Mat & mat, const std::string & format = "");
	CompressionThread(const std::vector<unsigned char> & bytes, bool isImage);
	const std::vector<unsigned char> & getCompressedData() const {return compressedData_;}
	cv::Mat & getUncompressedData() {return uncompressedData_;}
protected:
	virtual void mainLoop();
private:
	std::vector<unsigned char> compressedData_;
	cv::Mat uncompressedData_;
	std::string format_;
	bool image_;
	bool compressMode_;
};

cv::Mat RTABMAP_EXP rgbFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA> & cloud, bool bgrOrder = true);
cv::Mat RTABMAP_EXP depthFromCloud(
		const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		float & fx,
		float & fy,
		bool depth16U = true);
void RTABMAP_EXP rgbdFromCloud(
		const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		cv::Mat & rgb,
		cv::Mat & depth,
		float & fx,
		float & fy,
		bool bgrOrder = true,
		bool depth16U = true);

cv::Mat RTABMAP_EXP cvtDepthFromFloat(const cv::Mat & depth32F);
cv::Mat RTABMAP_EXP cvtDepthToFloat(const cv::Mat & depth16U);

std::multimap<int, pcl::PointXYZ> RTABMAP_EXP generateWords3(
		const std::multimap<int, cv::KeyPoint> & words,
		const cv::Mat & depth,
		float depthConstant,
		const Transform & transform);

std::multimap<int, cv::KeyPoint> RTABMAP_EXP aggregate(
		const std::list<int> & wordIds,
		const std::vector<cv::KeyPoint> & keypoints);

pcl::PointXYZ RTABMAP_EXP getDepth(
		const cv::Mat & depthImage,
		int x, int y,
		float depthConstant);

pcl::PointXYZ RTABMAP_EXP getDepth(const cv::Mat & depthImage,
					   int x, int y,
					   float cx, float cy,
					   float fx, float fy);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		float voxelSize);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP voxelize(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float voxelSize);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP sampling(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int samples);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP sampling(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int samples);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP passThrough(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);

pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & transform);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & transform);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromDepth(
		const cv::Mat & imageDepth,
		float depthConstant,
		int decimation);
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromDepth(
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation = 1);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		float depthConstant,
		int decimation = 1);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation = 1);

cv::Mat RTABMAP_EXP depth2DFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP depth2DToPointCloud(const cv::Mat & depth2D);

std::vector<unsigned char> RTABMAP_EXP compressImage(const cv::Mat & image, const std::string & format = ".png");
cv::Mat RTABMAP_EXP uncompressImage(const std::vector<unsigned char> & bytes);

std::vector<unsigned char> RTABMAP_EXP compressData(const cv::Mat & data);
cv::Mat RTABMAP_EXP uncompressData(const std::vector<unsigned char> & bytes);

// remove depth by z axis
void RTABMAP_EXP extractXYZCorrespondences(const std::multimap<int, pcl::PointXYZ> & words1,
									  const std::multimap<int, pcl::PointXYZ> & words2,
									  pcl::PointCloud<pcl::PointXYZ> & cloud1,
									  pcl::PointCloud<pcl::PointXYZ> & cloud2);

void RTABMAP_EXP extractXYZCorrespondencesRANSAC(const std::multimap<int, pcl::PointXYZ> & words1,
									  const std::multimap<int, pcl::PointXYZ> & words2,
									  pcl::PointCloud<pcl::PointXYZ> & cloud1,
									  pcl::PointCloud<pcl::PointXYZ> & cloud2);

void RTABMAP_EXP extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
									   const cv::Mat & depthImage1,
									   const cv::Mat & depthImage2,
									   float cx, float cy,
									   float fx, float fy,
									   float maxDepth,
									   pcl::PointCloud<pcl::PointXYZ> & cloud1,
									   pcl::PointCloud<pcl::PointXYZ> & cloud2);

void RTABMAP_EXP extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<pcl::PointXYZ> & cloud1,
							   const pcl::PointCloud<pcl::PointXYZ> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2,
							   char depthAxis);
void RTABMAP_EXP extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<pcl::PointXYZRGB> & cloud1,
							   const pcl::PointCloud<pcl::PointXYZRGB> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2,
							   char depthAxis);

int RTABMAP_EXP countUniquePairs(const std::multimap<int, pcl::PointXYZ> & wordsA,
					 const std::multimap<int, pcl::PointXYZ> & wordsB);

void RTABMAP_EXP filterMaxDepth(pcl::PointCloud<pcl::PointXYZ> & inliers1,
					pcl::PointCloud<pcl::PointXYZ> & inliers2,
					float maxDepth,
					char depthAxis,
					bool removeDuplicates);

Transform RTABMAP_EXP transformFromXYZCorrespondences(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud1,
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud2,
		double inlierThreshold = 0.02,
		int iterations = 100,
		int * inliers = 0);

Transform RTABMAP_EXP icp(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_source,
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_target,
		double maxCorrespondenceDistance,
		int maximumIterations,
		bool & hasConverged,
		double & fitnessScore);

Transform RTABMAP_EXP icpPointToPlane(
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud_source,
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud_target,
		double maxCorrespondenceDistance,
		int maximumIterations,
		bool & hasConverged,
		double & fitnessScore);

Transform RTABMAP_EXP icp2D(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_source,
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_target,
		double maxCorrespondenceDistance,
		int maximumIterations,
		bool & hasConverged,
		double & fitnessScore);

pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

int RTABMAP_EXP getCorrespondencesCount(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_source,
							const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_target,
							float maxDistance);

void RTABMAP_EXP findCorrespondences(
		const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<cv::Point2f, cv::Point2f> > & pairs);

void RTABMAP_EXP findCorrespondences(
		const std::multimap<int, pcl::PointXYZ> & words1,
		const std::multimap<int, pcl::PointXYZ> & words2,
		pcl::PointCloud<pcl::PointXYZ> & inliers1,
		pcl::PointCloud<pcl::PointXYZ> & inliers2,
		float maxDepth);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cvMat2Cloud(
		const cv::Mat & matrix,
		const Transform & tranform = Transform::getIdentity());

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP getICPReadyCloud(
		const cv::Mat & depth,
		float depthConstant,
		int decimation,
		double maxDepth,
		float voxel,
		int samples,
		const Transform & transform = Transform::getIdentity());

inline Eigen::Matrix4f transformToEigen4f(const Transform & transform)
{
	Eigen::Matrix4f m;
	m << transform[0], transform[1], transform[2], transform[3],
		 transform[4], transform[5], transform[6], transform[7],
		 transform[8], transform[9], transform[10], transform[11],
		 0,0,0,1;
	return m;
}
inline Eigen::Matrix4d transformToEigen4d(const Transform & transform)
{
	Eigen::Matrix4d m;
	m << transform[0], transform[1], transform[2], transform[3],
		 transform[4], transform[5], transform[6], transform[7],
		 transform[8], transform[9], transform[10], transform[11],
		 0,0,0,1;
	return m;
}

inline Eigen::Affine3f transformToEigen3f(const Transform & transform)
{
	return Eigen::Affine3f(transformToEigen4f(transform));
}

inline Eigen::Affine3d transformToEigen3d(const Transform & transform)
{
	return Eigen::Affine3d(transformToEigen4d(transform));
}

inline Transform transformFromEigen4f(const Eigen::Matrix4f & matrix)
{
	return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
					 matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
					 matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}
inline Transform transformFromEigen4d(const Eigen::Matrix4d & matrix)
{
	return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
					 matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
					 matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}

inline Transform transformFromEigen3f(const Eigen::Affine3f & matrix)
{
	return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
					 matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
					 matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}
inline Transform transformFromEigen3d(const Eigen::Affine3d & matrix)
{
	return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
					 matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
					 matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP concatenateClouds(const std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clouds);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP concatenateClouds(const std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds);
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP get3DFASTKpts(
		const cv::Mat & image,
		const cv::Mat & imageDepth,
		float constant,
		int fastThreshold=50,
		bool fastNonmaxSuppression=true,
		float maxDepth = 5.0f);

pcl::PolygonMesh::Ptr RTABMAP_EXP createMesh(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, float maxEdgeLength = 0.025, bool smoothing = true);

void RTABMAP_EXP optimizeTOROGraph(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		std::map<int, Transform> & optimizedPoses,
		Transform & mapCorrection,
		int toroIterations = 100,
		bool toroInitialGuess = true,
		std::list<std::map<int, Transform> > * intermediateGraphes = 0);

bool RTABMAP_EXP saveTOROGraph(
		const std::string & fileName,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints);

bool RTABMAP_EXP loadTOROGraph(const std::string & fileName,
		std::map<int, Transform> & poses,
		std::multimap<int, std::pair<int, Transform> > & edgeConstraints);

cv::Mat RTABMAP_EXP create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans,
		float delta,
		float & xMin,
		float & yMin);

void RTABMAP_EXP rayTrace(const cv::Point2i & start,
		const cv::Point2i & end,
		cv::Mat & grid);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL3D_H_ */
