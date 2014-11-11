/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef UTIL3D_H_
#define UTIL3D_H_

#include "rtabmap/core/RtabmapExp.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include <string>
#include <set>

#include <rtabmap/core/Link.h>
#include <rtabmap/utilite/UThread.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/pcl_base.h>

namespace rtabmap
{

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
	CompressionThread(const cv::Mat & bytes, bool isImage);
	const cv::Mat & getCompressedData() const {return compressedData_;}
	cv::Mat & getUncompressedData() {return uncompressedData_;}
protected:
	virtual void mainLoop();
private:
	cv::Mat compressedData_;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		float fx,
		float fy,
		float cx,
		float cy,
		const Transform & transform);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP generateKeypoints3DDisparity(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & disparity,
		float fx,
		float baseline,
		float cx,
		float cy,
		const Transform & transform);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP generateKeypoints3DStereo(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		float fx,
		float baseline,
		float cx,
		float cy,
		const Transform & transform = Transform::getIdentity(),
		int flowWinSize = 9,
		int flowMaxLevel = 4,
		int flowIterations = 20,
		double flowEps = 0.02);

std::multimap<int, cv::KeyPoint> RTABMAP_EXP aggregate(
		const std::list<int> & wordIds,
		const std::vector<cv::KeyPoint> & keypoints);

pcl::PointXYZ RTABMAP_EXP projectDepthTo3D(
		const cv::Mat & depthImage,
		float x, float y,
		float cx, float cy,
		float fx, float fy,
		bool smoothing,
		float maxZError = 0.03f);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromDepth(
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation = 1);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation = 1);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromDisparity(
		const cv::Mat & imageDisparity,
		float cx, float cy,
		float fx, float baseline,
		int decimation);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromDisparityRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDisparity,
		float cx, float cy,
		float fx, float baseline,
		int decimation);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudFromStereoImages(
		const cv::Mat & imageLeft,
		const cv::Mat & imageRight,
		float cx, float cy,
		float fx, float baseline,
		int decimation);

cv::Mat RTABMAP_EXP disparityFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage);

cv::Mat RTABMAP_EXP disparityFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		int flowWinSize = 9,
		int flowMaxLevel = 4,
		int flowIterations = 20,
		double flowEps = 0.02,
		float maxCorrespondencesSlope = 0.1f);

cv::Mat RTABMAP_EXP depthFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		float fx,
		float baseline,
		int flowWinSize = 9,
		int flowMaxLevel = 4,
		int flowIterations = 20,
		double flowEps = 0.02);

cv::Mat RTABMAP_EXP disparityFromStereoCorrespondences(
		const cv::Mat & leftImage,
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const std::vector<unsigned char> & mask,
		float maxSlope = 0.1f);

cv::Mat RTABMAP_EXP depthFromStereoCorrespondences(
		const cv::Mat & leftImage,
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const std::vector<unsigned char> & mask,
		float fx, float baseline);

pcl::PointXYZ RTABMAP_EXP projectDisparityTo3D(
		const cv::Point2f & pt,
		float disparity,
		float cx, float cy, float fx, float baseline);

pcl::PointXYZ RTABMAP_EXP projectDisparityTo3D(
		const cv::Point2f & pt,
		const cv::Mat & disparity,
		float cx, float cy, float fx, float baseline);

cv::Mat RTABMAP_EXP depthFromDisparity(const cv::Mat & disparity,
		float fx, float baseline,
		int type = CV_32FC1);

cv::Mat RTABMAP_EXP depth2DFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP depth2DToPointCloud(const cv::Mat & depth2D);

std::vector<unsigned char> RTABMAP_EXP compressImage(const cv::Mat & image, const std::string & format = ".png");
cv::Mat RTABMAP_EXP compressImage2(const cv::Mat & image, const std::string & format = ".png");

cv::Mat RTABMAP_EXP uncompressImage(const cv::Mat & bytes);
cv::Mat RTABMAP_EXP uncompressImage(const std::vector<unsigned char> & bytes);

std::vector<unsigned char> RTABMAP_EXP compressData(const cv::Mat & data);
cv::Mat RTABMAP_EXP compressData2(const cv::Mat & data);

cv::Mat RTABMAP_EXP uncompressData(const cv::Mat & bytes);
cv::Mat RTABMAP_EXP uncompressData(const std::vector<unsigned char> & bytes);
cv::Mat RTABMAP_EXP uncompressData(const unsigned char * bytes, unsigned long size);

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
		bool refineModel = false,
		double refineModelSigma = 3.0,
		int refineModelIterations = 10,
		std::vector<int> * inliers = 0);

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
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int normalKSearch = 20);

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP computeNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int normalKSearch = 20);

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP computeNormalsSmoothed(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float smoothingSearchRadius = 0.025,
		bool smoothingPolynomialFit = true);

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
		float maxDepth,
		std::set<int> * uniqueCorrespondences = 0);

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cvMat2Cloud(
		const cv::Mat & matrix,
		const Transform & tranform = Transform::getIdentity());

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP getICPReadyCloud(
		const cv::Mat & depth,
		float fx,
		float fy,
		float cx,
		float cy,
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

pcl::PolygonMesh::Ptr RTABMAP_EXP createMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloudWithNormals,
		float gp3SearchRadius = 0.025,
		float gp3Mu = 2.5,
		int gp3MaximumNearestNeighbors = 100,
		float gp3MaximumSurfaceAngle = M_PI/4,
		float gp3MinimumAngle = M_PI/18,
		float gp3MaximumAngle = 2*M_PI/3,
		bool gp3NormalConsistency = false);

std::multimap<int, Link>::iterator RTABMAP_EXP findLink(
		std::multimap<int, Link> & links,
		int from,
		int to);

// <int, depth> depth=0 means infinite depth
std::map<int, int> RTABMAP_EXP generateDepthGraph(
		const std::multimap<int, Link> & links,
		int fromId,
		int depth = 0);

void RTABMAP_EXP optimizeTOROGraph(
		const std::map<int, int> & depthGraph,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		std::map<int, Transform> & optimizedPoses,
		int toroIterations = 100,
		bool toroInitialGuess = true,
		std::list<std::map<int, Transform> > * intermediateGraphes = 0);

void RTABMAP_EXP optimizeTOROGraph(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		std::map<int, Transform> & optimizedPoses,
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

std::map<int, Transform> RTABMAP_EXP radiusPosesFiltering(
		const std::map<int, Transform> & poses,
		float radius,
		float angle,
		bool keepLatest = true);

std::multimap<int, int> RTABMAP_EXP radiusPosesClustering(
		const std::map<int, Transform> & poses,
		float radius,
		float angle);

bool RTABMAP_EXP occupancy2DFromCloud3D(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize = 0.05f,
		float groundNormalAngle = M_PI_4,
		int minClusterSize = 20);

cv::Mat RTABMAP_EXP create2DMapFromOccupancyLocalMaps(
		const std::map<int, Transform> & poses,
		const std::map<int, std::pair<cv::Mat, cv::Mat> > & occupancy,
		float cellSize,
		float & xMin,
		float & yMin,
		int fillEmptyRadius = 0,
		float minMapSize = 0.0f);

cv::Mat RTABMAP_EXP create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans,
		float cellSize,
		bool unknownSpaceFilled,
		float & xMin,
		float & yMin,
		float minMapSize = 0.0f);

void RTABMAP_EXP rayTrace(const cv::Point2i & start,
		const cv::Point2i & end,
		cv::Mat & grid,
		bool stopOnObstacle);

cv::Mat RTABMAP_EXP convertMap2Image8U(const cv::Mat & map8S);

/**
 * @brief Concatenate a vector of indices to a single vector.
 *
 * @param indices the vector of indices to concatenate.
 * @note This methods doesn't check if indices exist in the two set and doesn't
 * sort the output indices. If we are not sure if the the
 * two set of indices set are disjoint and/or you need sorted indices, the use of mergeIndices().
 * @return the indices concatenated.
 */
pcl::IndicesPtr RTABMAP_EXP concatenate(
		const std::vector<pcl::IndicesPtr> & indices);

/**
 * @brief Concatenate two vector of indices to a single vector.
 *
 * @param indicesA the first vector of indices to concatenate.
 * @param indicesB the second vector of indices to concatenate.
 * @note This methods doesn't check if indices exist in the two set and doesn't
 * sort the output indices. If we are not sure if the the
 * two set of indices set are disjoint and/or you need sorted indices, the use of mergeIndices().
 * @return the indices concatenated.
 */
pcl::IndicesPtr RTABMAP_EXP concatenate(
		const pcl::IndicesPtr & indicesA,
		const pcl::IndicesPtr & indicesB);

///////////////////
// Templated PCL methods
///////////////////

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr voxelize(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		float voxelSize);

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr sampling(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		int samples);

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr passThrough(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max);

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr removeNaNFromPointCloud(
		const typename pcl::PointCloud<PointT>::Ptr & cloud);

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr removeNaNNormalsFromPointCloud(
		const typename pcl::PointCloud<PointT>::Ptr & cloud);

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr transformPointCloud(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const Transform & transform);

template<typename PointT>
PointT transformPoint(
		const PointT & pt,
		const Transform & transform);

template<typename PointT>
void segmentObstaclesFromGround(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		pcl::IndicesPtr & ground,
		pcl::IndicesPtr & obstacles,
		float normalRadiusSearch,
		float groundNormalAngle,
		int minClusterSize,
		bool segmentFlatObstacles = false);

template<typename PointT>
void projectCloudOnXYPlane(
		typename pcl::PointCloud<PointT>::Ptr & cloud);

/**
 * For convenience.
 */
template<typename PointT>
pcl::IndicesPtr radiusFiltering(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		float radiusSearch,
		int minNeighborsInRadius);

/**
 * @brief Wrapper of the pcl::RadiusOutlierRemoval class.
 *
 * Points in the cloud which have less than a minimum of neighbors in the
 * specified radius are filtered.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to check, if empty, all points in the cloud are checked.
 * @param radiusSearch the radius in meter.
 * @param minNeighborsInRadius the minimum of neighbors to keep the point.
 * @return the indices of the points satisfying the parameters.
 */
template<typename PointT>
pcl::IndicesPtr radiusFiltering(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius);

/**
 * For convenience.
 */
template<typename PointT>
pcl::IndicesPtr normalFiltering(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		float angleMax,
		const Eigen::Vector4f & normal,
		float radiusSearch,
		const Eigen::Vector4f & viewpoint);

/**
 * @brief Given a normal and a maximum angle error, keep all points of the cloud
 * respecting this normal.
 *
 * The normals are computed using the radius search parameter (pcl::NormalEstimation class is used for this), then
 * for each normal, the corresponding point is filtered if the
 * angle (using pcl::getAngle3D()) with the normal specified by the user is larger than the maximum
 * angle specified by the user.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to process, if empty, all points in the cloud are processed.
 * @param angleMax the maximum angle.
 * @param normal the normal to which each point's normal is compared.
 * @param radiusSearch radius parameter used for normal estimation (see pcl::NormalEstimation).
 * @param viewpoint from which viewpoint the normals should be estimated (see pcl::NormalEstimation).
 * @return the indices of the points which respect the normal constraint.
 */
template<typename PointT>
pcl::IndicesPtr normalFiltering(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		float radiusSearch,
		const Eigen::Vector4f & viewpoint);

/**
 * For convenience.
 */
template<typename PointT>
std::vector<pcl::IndicesPtr> extractClusters(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);

/**
 * @brief Wrapper of the pcl::EuclideanClusterExtraction class.
 *
 * Extract all clusters from a point cloud given a maximum cluster distance tolerance.
 * @param cloud the input cloud.
 * @param indices the input indices of the cloud to process, if empty, all points in the cloud are processed.
 * @param clusterTolerance the cluster distance tolerance (see pcl::EuclideanClusterExtraction).
 * @param minClusterSize minimum size of the clusters to return (see pcl::EuclideanClusterExtraction).
 * @param maxClusterSize maximum size of the clusters to return (see pcl::EuclideanClusterExtraction).
 * @param biggestClusterIndex the index of the biggest cluster, if the clusters are empty, a negative index is set.
 * @return the indices of each cluster found.
 */
template<typename PointT>
std::vector<pcl::IndicesPtr> extractClusters(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float clusterTolerance,
		int minClusterSize,
		int maxClusterSize = std::numeric_limits<int>::max(),
		int * biggestClusterIndex = 0);

template<typename PointT>
pcl::IndicesPtr extractNegativeIndices(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices);

} // namespace util3d
} // namespace rtabmap

#include "rtabmap/core/impl/util3d.hpp"

#endif /* UTIL3D_H_ */
