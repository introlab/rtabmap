/*
 * util.cpp
 *
 *  Created on: 2013-02-27
 *      Author: mathieu
 */

#include <rtabmap/core/EpipolarGeometry.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>

#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <rtabmap/core/VWDictionary.h>
#include <cmath>
#include <stdio.h>

#include <zlib.h>

#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/Signature.h"
#include "toro3d/treeoptimizer3.hh"

#include <pcl/filters/random_sample.h>

namespace rtabmap
{

namespace util3d
{

// format : ".png" ".jpg" "" (empty is general)
CompressionThread::CompressionThread(const cv::Mat & mat, const std::string & format) :
	uncompressedData_(mat),
	format_(format),
	image_(!format.empty()),
	compressMode_(true)
{
	UASSERT(format.empty() || format.compare(".png") == 0 || format.compare(".jpg") == 0);
}
// assume image
CompressionThread::CompressionThread(const std::vector<unsigned char> & bytes, bool isImage) :
	compressedData_(bytes),
	image_(isImage),
	compressMode_(false)
{}
void CompressionThread::mainLoop()
{
	if(compressMode_)
	{
		if(!uncompressedData_.empty())
		{
			if(image_)
			{
				compressedData_ = compressImage(uncompressedData_, format_);
			}
			else
			{
				compressedData_ = compressData(uncompressedData_);
			}
		}
	}
	else // uncompress
	{
		if(!compressedData_.empty())
		{
			if(image_)
			{
				uncompressedData_ = uncompressImage(compressedData_);
			}
			else
			{
				uncompressedData_ = uncompressData(compressedData_);
			}
		}
	}
	this->kill();
}

cv::Mat bgrFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA> & cloud, bool bgrOrder)
{
	cv::Mat frameBGR = cv::Mat(cloud.height,cloud.width,CV_8UC3);

	for(unsigned int h = 0; h < cloud.height; h++)
	{
		for(unsigned int w = 0; w < cloud.width; w++)
		{
			if(bgrOrder)
			{
				frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).b;
				frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
				frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).r;
			}
			else
			{
				frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).r;
				frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
				frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).b;
			}
		}
	}
	return frameBGR;
}

// return float image in meter
cv::Mat depthFromCloud(
		const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		float & fx,
		float & fy,
		bool depth16U)
{
	cv::Mat frameDepth = cv::Mat(cloud.height,cloud.width,depth16U?CV_16UC1:CV_32FC1);
	fx = 0.0f; // needed to reconstruct the cloud
	fy = 0.0f; // needed to reconstruct the cloud
	for(unsigned int h = 0; h < cloud.height; h++)
	{
		for(unsigned int w = 0; w < cloud.width; w++)
		{
			float depth = cloud.at(h*cloud.width + w).z;
			if(depth16U)
			{
				depth *= 1000.0f;
				unsigned short depthMM = 0;
				if(depth <= (float)USHRT_MAX)
				{
					depthMM = (unsigned short)depth;
				}
				frameDepth.at<unsigned short>(h,w) = depthMM;
			}
			else
			{
				frameDepth.at<float>(h,w) = depth;
			}

			// update constants
			if(fx == 0.0f &&
			   uIsFinite(cloud.at(h*cloud.width + w).x) &&
			   uIsFinite(depth) &&
			   w != cloud.width/2 &&
			   depth > 0)
			{
				fx = cloud.at(h*cloud.width + w).x / ((float(w) - float(cloud.width)/2.0f) * depth);
				if(depth16U)
				{
					fx*=1000.0f;
				}
			}
			if(fy == 0.0f &&
			   uIsFinite(cloud.at(h*cloud.width + w).y) &&
			   uIsFinite(depth) &&
			   h != cloud.height/2 &&
			   depth > 0)
			{
				fy = cloud.at(h*cloud.width + w).y / ((float(h) - float(cloud.height)/2.0f) * depth);
				if(depth16U)
				{
					fy*=1000.0f;
				}
			}
		}
	}
	return frameDepth;
}

// return (unsigned short 16bits image in mm) (float 32bits image in m)
void rgbdFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		cv::Mat & frameBGR,
		cv::Mat & frameDepth,
		float & fx,
		float & fy,
		bool bgrOrder,
		bool depth16U)
{
	frameDepth = cv::Mat(cloud.height,cloud.width,depth16U?CV_16UC1:CV_32FC1);
	frameBGR = cv::Mat(cloud.height,cloud.width,CV_8UC3);

	fx = 0.0f; // needed to reconstruct the cloud
	fy = 0.0f; // needed to reconstruct the cloud
	for(unsigned int h = 0; h < cloud.height; h++)
	{
		for(unsigned int w = 0; w < cloud.width; w++)
		{
			//rgb
			if(bgrOrder)
			{
				frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).b;
				frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
				frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).r;
			}
			else
			{
				frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).r;
				frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
				frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).b;
			}

			//depth
			float depth = cloud.at(h*cloud.width + w).z;
			if(depth16U)
			{
				depth *= 1000.0f;
				unsigned short depthMM = 0;
				if(depth <= (float)USHRT_MAX)
				{
					depthMM = (unsigned short)depth;
				}
				frameDepth.at<unsigned short>(h,w) = depthMM;
			}
			else
			{
				frameDepth.at<float>(h,w) = depth;
			}

			// update constants
			if(fx == 0.0f &&
			   uIsFinite(cloud.at(h*cloud.width + w).x) &&
			   uIsFinite(depth) &&
			   w != cloud.width/2 &&
			   depth > 0)
			{
				fx = 1.0f/(cloud.at(h*cloud.width + w).x / ((float(w) - float(cloud.width)/2.0f) * depth));
				if(depth16U)
				{
					fx/=1000.0f;
				}
			}
			if(fy == 0.0f &&
			   uIsFinite(cloud.at(h*cloud.width + w).y) &&
			   uIsFinite(depth) &&
			   h != cloud.height/2 &&
			   depth > 0)
			{
				fy = 1.0f/(cloud.at(h*cloud.width + w).y / ((float(h) - float(cloud.height)/2.0f) * depth));
				if(depth16U)
				{
					fy/=1000.0f;
				}
			}
		}
	}
}

cv::Mat cvtDepthFromFloat(const cv::Mat & depth32F)
{
	UASSERT(depth32F.empty() || depth32F.type() == CV_32FC1);
	cv::Mat depth16U;
	if(!depth32F.empty())
	{
		depth16U = cv::Mat(depth32F.rows, depth32F.cols, CV_16UC1);
		for(int i=0; i<depth32F.rows; ++i)
		{
			for(int j=0; j<depth32F.cols; ++j)
			{
				float depth = (depth32F.at<float>(i,j)*1000.0f);
				unsigned short depthMM = 0;
				if(depth <= (float)USHRT_MAX)
				{
					depthMM = (unsigned short)depth;
				}
				depth16U.at<unsigned short>(i, j) = depthMM;
			}
		}
	}
	return depth16U;
}

cv::Mat cvtDepthToFloat(const cv::Mat & depth16U)
{
	UASSERT(depth16U.empty() || depth16U.type() == CV_16UC1);
	cv::Mat depth32F;
	if(!depth16U.empty())
	{
		depth32F = cv::Mat(depth16U.rows, depth16U.cols, CV_32FC1);
		for(int i=0; i<depth16U.rows; ++i)
		{
			for(int j=0; j<depth16U.cols; ++j)
			{
				float depth = float(depth16U.at<unsigned short>(i,j))/1000.0f;
				depth32F.at<float>(i, j) = depth;
			}
		}
	}
	return depth32F;
}

std::multimap<int, pcl::PointXYZ> generateWords3(
		const std::multimap<int, cv::KeyPoint> & words,
		const cv::Mat & depth,
		float depthConstant,
		const Transform & transform)
{
	std::multimap<int, pcl::PointXYZ> words3;
	for(std::multimap<int, cv::KeyPoint>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
	{
		pcl::PointXYZ pt = util3d::getDepth(
				depth,
				iter->second.pt.x+0.5f,
				iter->second.pt.y+0.5f,
				depth.cols/2,
				depth.rows/2,
				1.0f/depthConstant,
				1.0f/depthConstant);

		if(!transform.isNull() && !transform.isIdentity())
		{
			pt = pcl::transformPoint(pt, util3d::transformToEigen3f(transform));
		}
		words3.insert(std::make_pair(iter->first, pt));
	}
	return words3;
}

std::multimap<int, cv::KeyPoint> aggregate(
		const std::list<int> & wordIds,
		const std::vector<cv::KeyPoint> & keypoints)
{
	std::multimap<int, cv::KeyPoint> words;
	std::vector<cv::KeyPoint>::const_iterator kpIter = keypoints.begin();
	for(std::list<int>::const_iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
	{
		words.insert(std::pair<int, cv::KeyPoint >(*iter, *kpIter));
		++kpIter;
	}
	return words;
}

std::list<std::pair<cv::Point2f, cv::Point2f> > findCorrespondences(
		const std::multimap<int, cv::KeyPoint> & words1,
		const std::multimap<int, cv::KeyPoint> & words2)
{
	std::list<std::pair<cv::Point2f, cv::Point2f> > correspondences;

	// Find pairs
	std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
	rtabmap::EpipolarGeometry::findPairsUnique(words1, words2, pairs);

	if(pairs.size() > 7) // 8 min?
	{
		// Find fundamental matrix
		std::vector<uchar> status;
		cv::Mat fundamentalMatrix = rtabmap::EpipolarGeometry::findFFromWords(pairs, status);
		//ROS_INFO("inliers = %d/%d", uSum(status), pairs.size());
		if(!fundamentalMatrix.empty())
		{
			int i = 0;
			//int goodCount = 0;
			for(std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > >::iterator iter=pairs.begin(); iter!=pairs.end(); ++iter)
			{
				if(status[i])
				{
					correspondences.push_back(std::pair<cv::Point2f, cv::Point2f>(iter->second.first.pt, iter->second.second.pt));
					//ROS_INFO("inliers kpts %f %f vs %f %f", iter->second.first.pt.x, iter->second.first.pt.y, iter->second.second.pt.x, iter->second.second.pt.y);
				}
				++i;
			}
		}
	}
	return correspondences;
}

void findCorrespondences(
		const std::multimap<int, pcl::PointXYZ> & words1,
		const std::multimap<int, pcl::PointXYZ> & words2,
		pcl::PointCloud<pcl::PointXYZ> & inliers1,
		pcl::PointCloud<pcl::PointXYZ> & inliers2,
		float maxDepth)
{
	std::list<int> ids = uUniqueKeys(words1);
	// Find pairs
	inliers1.resize(ids.size());
	inliers2.resize(ids.size());

	int oi=0;
	for(std::list<int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
	{
		if(words1.count(*iter) == 1 && words2.count(*iter) == 1)
		{
			inliers1[oi] = words1.find(*iter)->second;
			inliers2[oi] = words2.find(*iter)->second;
			if(pcl::isFinite(inliers1[oi]) &&
			   pcl::isFinite(inliers2[oi]) &&
			   (inliers1[oi].x != 0 || inliers1[oi].y != 0 || inliers1[oi].z != 0) &&
			   (inliers2[oi].x != 0 || inliers2[oi].y != 0 || inliers2[oi].z != 0) &&
			   (maxDepth <= 0 || (inliers1[oi].x <= maxDepth && inliers2[oi].x<=maxDepth)))
			{
				++oi;
			}
		}
	}
	inliers1.resize(oi);
	inliers2.resize(oi);
}

pcl::PointXYZ getDepth(
		const cv::Mat & depthImage,
		int x, int y,
		float depthConstant)
{
	return getDepth(depthImage, x, y,
			(float)depthImage.cols/2,
			(float)depthImage.rows/2,
			1.0f/depthConstant,
			1.0f/depthConstant);
}

pcl::PointXYZ getDepth(const cv::Mat & depthImage,
					   int x, int y,
			       float cx, float cy,
			       float fx, float fy)
{
	UASSERT(x >=0 && x<depthImage.cols && y >=0 && y<depthImage.rows);

	pcl::PointXYZ pt;

	// Use correct principal point from calibration
	float center_x = cx; //cameraInfo.K.at(2)
	float center_y = cy; //cameraInfo.K.at(5)

	bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

	// Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
	float unit_scaling = isInMM?0.001f:1.0f;
	float constant_x = unit_scaling / fx; //cameraInfo.K.at(0)
	float constant_y = unit_scaling / fy; //cameraInfo.K.at(4)
	float bad_point = std::numeric_limits<float>::quiet_NaN ();

	float depth;
	bool isValid;
	if(isInMM)
	{
		depth = (float)depthImage.at<uint16_t>(y,x);
		isValid = depth != 0.0f;
	}
	else
	{
		depth = depthImage.at<float>(y,x);
		isValid = uIsFinite(depth);
	}

	// Check for invalid measurements
	if (!isValid)
	{
		pt.x = pt.y = pt.z = bad_point;
	}
	else
	{
		// Fill in XYZ
		pt.x = (float(x) - center_x) * depth * constant_x;
		pt.y = (float(y) - center_y) * depth * constant_y;
		pt.z = depth*unit_scaling;
	}
	return pt;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float voxelSize)
{
	UASSERT(voxelSize > 0.0f);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setLeafSize(voxelSize, voxelSize, voxelSize);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, float voxelSize)
{
	UASSERT(voxelSize > 0.0f);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> filter;
	filter.setLeafSize(voxelSize, voxelSize, voxelSize);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr sampling(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, int samples)
{
	UASSERT(samples > 0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::RandomSample<pcl::PointXYZ> filter;
	filter.setSample(samples);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampling(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, int samples)
{
	UASSERT(samples > 0);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::RandomSample<pcl::PointXYZRGB> filter;
	filter.setSample(samples);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max)
{
	UASSERT(max > min);
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThrough(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::string & axis,
		float min,
		float max)
{
	UASSERT(max > min);
	UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setFilterFieldName(axis);
	filter.setFilterLimits(min, max);
	filter.setInputCloud(cloud);
	filter.filter(*output);
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud (*cloud, *output, indices);
	return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud (*cloud, *output, indices);
	return output;
}

pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_EXP removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);
	std::vector<int> indices;
	pcl::removeNaNNormalsFromPointCloud (*cloud, *output, indices);
	return output;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_EXP removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::vector<int> indices;
	pcl::removeNaNNormalsFromPointCloud(*cloud, *output, indices);
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud, *output, transformToEigen4f(transform));
	return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*cloud, *output, transformToEigen4f(transform));
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDepth(
		const cv::Mat & imageDepth,
		float depthConstant,
		int decimation)
{
	return cloudFromDepth(
			imageDepth,
			float(imageDepth.cols/2),
			float(imageDepth.rows/2),
			1.0f/depthConstant,
			1.0f/depthConstant,
			decimation);
}
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDepth(
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if(decimation < 1)
	{
		return cloud;
	}

	//cloud.header = cameraInfo.header;
	cloud->height = imageDepth.rows/decimation;
	cloud->width  = imageDepth.cols/decimation;
	cloud->is_dense = false;

	cloud->resize(cloud->height * cloud->width);

	int count = 0 ;

	for(int h = 0; h < imageDepth.rows; h+=decimation)
	{
		for(int w = 0; w < imageDepth.cols; w+=decimation)
		{
			pcl::PointXYZ & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

			pcl::PointXYZ ptXYZ = getDepth(imageDepth, w, h, cx, cy, fx, fy);
			pt.x = ptXYZ.x;
			pt.y = ptXYZ.y;
			pt.z = ptXYZ.z;
			++count;
		}
	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		float depthConstant,
		int decimation)
{
	return cloudFromDepthRGB(
			imageRgb,
			imageDepth,
			float(imageDepth.cols/2),
			float(imageDepth.rows/2),
			1.0f/depthConstant,
			1.0f/depthConstant,
			decimation);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(decimation < 1)
	{
		return cloud;
	}

	bool mono;
	if(imageRgb.channels() == 3) // BGR
	{
		mono = false;
	}
	else if(imageRgb.channels() == 1) // Mono
	{
		mono = true;
	}
	else
	{
		return cloud;
	}

	//cloud.header = cameraInfo.header;
	cloud->height = imageDepth.rows/decimation;
	cloud->width  = imageDepth.cols/decimation;
	cloud->is_dense = false;
	cloud->resize(cloud->height * cloud->width);

	for(int h = 0; h < imageDepth.rows && h/decimation < (int)cloud->height; h+=decimation)
	{
		for(int w = 0; w < imageDepth.cols && w/decimation < (int)cloud->width; w+=decimation)
		{
			pcl::PointXYZRGB & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));
			if(!mono)
			{
				pt.b = imageRgb.at<cv::Vec3b>(h,w)[0];
				pt.g = imageRgb.at<cv::Vec3b>(h,w)[1];
				pt.r = imageRgb.at<cv::Vec3b>(h,w)[2];
			}
			else
			{
				unsigned char v = imageRgb.at<unsigned char>(h,w);
				pt.b = v;
				pt.g = v;
				pt.r = v;
			}

			pcl::PointXYZ ptXYZ = getDepth(imageDepth, w, h, cx, cy, fx, fy);
			pt.x = ptXYZ.x;
			pt.y = ptXYZ.y;
			pt.z = ptXYZ.z;
		}
	}
	return cloud;
}

cv::Mat depth2DFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
	cv::Mat depth2d(1, cloud.size(), CV_32FC2);
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		depth2d.at<cv::Vec2f>(i)[0] = cloud.at(i).x;
		depth2d.at<cv::Vec2f>(i)[1] = cloud.at(i).y;
	}
	return depth2d;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr depth2DToPointCloud(const cv::Mat & depth2D)
{
	UASSERT(depth2D.empty() || depth2D.type() == CV_32FC2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	output->resize(depth2D.cols);
	for(int i=0; i<depth2D.cols; ++i)
	{
		output->at(i).x = depth2D.at<cv::Vec2f>(i)[0];
		output->at(i).y = depth2D.at<cv::Vec2f>(i)[1];
	}
	return output;
}

// ".png" or ".jpg"
std::vector<unsigned char> compressImage(const cv::Mat & image, const std::string & format)
{
	std::vector<unsigned char> bytes;
	if(!image.empty())
	{
		cv::imencode(format, image, bytes);
	}
	return bytes;
}

cv::Mat uncompressImage(const std::vector<unsigned char> & bytes)
{
	 cv::Mat image;
	if(bytes.size())
	{
#if CV_MAJOR_VERSION >=2 and CV_MINOR_VERSION >=4
		image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
#else
		image = cv::imdecode(bytes, -1);
#endif
	}
	return image;
}

std::vector<unsigned char> compressData(const cv::Mat & data)
{
	std::vector<unsigned char> bytes;
	if(!data.empty())
	{
		uLong sourceLen = data.total()*data.elemSize();
		uLong destLen = compressBound(sourceLen);
		bytes.resize(destLen);
		int errCode = compress(
						(Bytef *)bytes.data(),
						&destLen,
						(const Bytef *)data.data,
						sourceLen);

		bytes.resize(destLen+3*sizeof(int));
		*((int*)&bytes[destLen]) = data.rows;
		*((int*)&bytes[destLen+sizeof(int)]) = data.cols;
		*((int*)&bytes[destLen+2*sizeof(int)]) = data.type();

		if(errCode == Z_MEM_ERROR)
		{
			UERROR("Z_MEM_ERROR : Insufficient memory.");
		}
		else if(errCode == Z_BUF_ERROR)
		{
			UERROR("Z_BUF_ERROR : The buffer dest was not large enough to hold the uncompressed data.");
		}
	}
	return bytes;
}

cv::Mat uncompressData(const std::vector<unsigned char> & bytes)
{
	cv::Mat data;
	if(bytes.size()>=3*sizeof(int))
	{
		//last 3 int elements are matrix size and type
		int height = *((int*)&bytes[bytes.size()-3*sizeof(int)]);
		int width = *((int*)&bytes[bytes.size()-2*sizeof(int)]);
		int type = *((int*)&bytes[bytes.size()-1*sizeof(int)]);

		// If the size is higher, it may be a wrong data format.
		UASSERT_MSG(height>=0 && height<10000 &&
				    width>=0 && width<10000,
				    uFormat("size=%d, height=%d width=%d type=%d", bytes.size(), height, width, type).c_str());

		data = cv::Mat(height, width, type);
		uLongf totalUncompressed = data.total()*data.elemSize();

		int errCode = uncompress(
						(Bytef*)data.data,
						&totalUncompressed,
						(const Bytef*)bytes.data(),
						bytes.size());

		if(errCode == Z_MEM_ERROR)
		{
			UERROR("Z_MEM_ERROR : Insufficient memory.");
		}
		else if(errCode == Z_BUF_ERROR)
		{
			UERROR("Z_BUF_ERROR : The buffer dest was not large enough to hold the uncompressed data.");
		}
		else if(errCode == Z_DATA_ERROR)
		{
			UERROR("Z_DATA_ERROR : The compressed data (referenced by source) was corrupted.");
		}
	}
	return data;
}

void extractXYZCorrespondences(const std::multimap<int, pcl::PointXYZ> & words1,
									  const std::multimap<int, pcl::PointXYZ> & words2,
									  pcl::PointCloud<pcl::PointXYZ> & cloud1,
									  pcl::PointCloud<pcl::PointXYZ> & cloud2)
{
	const std::list<int> & ids = uUniqueKeys(words1);
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(words1.count(*i) == 1 && words2.count(*i) == 1)
		{
			const pcl::PointXYZ & pt1 = words1.find(*i)->second;
			const pcl::PointXYZ & pt2 = words2.find(*i)->second;
			if(pcl::isFinite(pt1) && pcl::isFinite(pt2))
			{
				cloud1.push_back(pt1);
				cloud2.push_back(pt2);
			}
		}
	}
}

void extractXYZCorrespondencesRANSAC(const std::multimap<int, pcl::PointXYZ> & words1,
									  const std::multimap<int, pcl::PointXYZ> & words2,
									  pcl::PointCloud<pcl::PointXYZ> & cloud1,
									  pcl::PointCloud<pcl::PointXYZ> & cloud2)
{
	std::list<std::pair<pcl::PointXYZ, pcl::PointXYZ> > pairs;
	const std::list<int> & ids = uUniqueKeys(words1);
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(words1.count(*i) == 1 && words2.count(*i) == 1)
		{
			const pcl::PointXYZ & pt1 = words1.find(*i)->second;
			const pcl::PointXYZ & pt2 = words2.find(*i)->second;
			if(pcl::isFinite(pt1) && pcl::isFinite(pt2))
			{
				pairs.push_back(std::pair<pcl::PointXYZ, pcl::PointXYZ>(pt1, pt2));
			}
		}
	}

	if(pairs.size() > 7)
	{
		// Remove outliers using fundamental matrix RANSAC
		std::vector<uchar> status(pairs.size(), 0);
		//Convert Keypoints to a structure that OpenCV understands
		//3 dimensions (Homogeneous vectors)
		cv::Mat points1(1, pairs.size(), CV_32FC2);
		cv::Mat points2(1, pairs.size(), CV_32FC2);

		float * points1data = points1.ptr<float>(0);
		float * points2data = points2.ptr<float>(0);

		// Fill the points here ...
		int i=0;
		for(std::list<std::pair<pcl::PointXYZ, pcl::PointXYZ> >::const_iterator iter = pairs.begin();
			iter != pairs.end();
			++iter )
		{
			points1data[i*2] = (*iter).first.x;
			points1data[i*2+1] = (*iter).first.y;

			points2data[i*2] = (*iter).second.x;
			points2data[i*2+1] = (*iter).second.y;

			++i;
		}

		// Find the fundamental matrix
		cv::Mat fundamentalMatrix = cv::findFundamentalMat(
					points1,
					points2,
					status,
					cv::FM_RANSAC,
					3.0,
					0.99);

		if(!fundamentalMatrix.empty())
		{
			int i = 0;
			for(std::list<std::pair<pcl::PointXYZ, pcl::PointXYZ> >::iterator iter=pairs.begin(); iter!=pairs.end(); ++iter)
			{
				if(status[i])
				{
					cloud1.push_back(iter->first);
					cloud2.push_back(iter->second);
				}
				++i;
			}
		}
	}
}

void extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
									   const cv::Mat & depthImage1,
									   const cv::Mat & depthImage2,
									   float cx, float cy,
									   float fx, float fy,
									   float maxDepth,
									   pcl::PointCloud<pcl::PointXYZ> & cloud1,
									   pcl::PointCloud<pcl::PointXYZ> & cloud2)
{
	cloud1.resize(correspondences.size());
	cloud2.resize(correspondences.size());
	int oi=0;
	for(std::list<std::pair<cv::Point2f, cv::Point2f> >::const_iterator iter = correspondences.begin();
		iter!=correspondences.end();
		++iter)
	{
		pcl::PointXYZ pt1 = getDepth(depthImage1, int(iter->first.x+0.5f), int(iter->first.y+0.5f), cx, cy, fx, fy);
		pcl::PointXYZ pt2 = getDepth(depthImage2, int(iter->second.x+0.5f), int(iter->second.y+0.5f), cx, cy, fx, fy);
		if(pcl::isFinite(pt1) && pcl::isFinite(pt2) &&
		   (maxDepth <= 0 || (pt1.z <= maxDepth && pt2.z<=maxDepth)))
		{
			cloud1[oi] = pt1;
			cloud2[oi] = pt2;
			++oi;
		}
	}
	cloud1.resize(oi);
	cloud2.resize(oi);
}

template<typename PointT>
inline void extractXYZCorrespondencesImpl(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<PointT> & cloud1,
							   const pcl::PointCloud<PointT> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2,
							   char depthAxis)
{
	for(std::list<std::pair<cv::Point2f, cv::Point2f> >::const_iterator iter = correspondences.begin();
		iter!=correspondences.end();
		++iter)
	{
		PointT pt1 = cloud1.at(int(iter->first.y+0.5f) * cloud1.width + int(iter->first.x+0.5f));
		PointT pt2 = cloud2.at(int(iter->second.y+0.5f) * cloud2.width + int(iter->second.x+0.5f));
		if(pcl::isFinite(pt1) &&
		   pcl::isFinite(pt2))
		{
			inliers1.push_back(pcl::PointXYZ(pt1.x, pt1.y, pt1.z));
			inliers2.push_back(pcl::PointXYZ(pt2.x, pt2.y, pt2.z));
		}
	}
}

void extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<pcl::PointXYZ> & cloud1,
							   const pcl::PointCloud<pcl::PointXYZ> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2,
							   char depthAxis)
{
	extractXYZCorrespondencesImpl(correspondences, cloud1, cloud2, inliers1, inliers2, depthAxis);
}
void extractXYZCorrespondences(const std::list<std::pair<cv::Point2f, cv::Point2f> > & correspondences,
							   const pcl::PointCloud<pcl::PointXYZRGB> & cloud1,
							   const pcl::PointCloud<pcl::PointXYZRGB> & cloud2,
							   pcl::PointCloud<pcl::PointXYZ> & inliers1,
							   pcl::PointCloud<pcl::PointXYZ> & inliers2,
							   char depthAxis)
{
	extractXYZCorrespondencesImpl(correspondences, cloud1, cloud2, inliers1, inliers2, depthAxis);
}

int countUniquePairs(const std::multimap<int, pcl::PointXYZ> & wordsA,
					 const std::multimap<int, pcl::PointXYZ> & wordsB)
{
	const std::list<int> & ids = uUniqueKeys(wordsA);
	int pairs = 0;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		std::list<pcl::PointXYZ> ptsA = uValues(wordsA, *i);
		std::list<pcl::PointXYZ> ptsB = uValues(wordsB, *i);
		if(ptsA.size() == 1 && ptsB.size() == 1)
		{
			++pairs;
		}
	}
	return pairs;
}

void filterMaxDepth(pcl::PointCloud<pcl::PointXYZ> & inliers1,
					pcl::PointCloud<pcl::PointXYZ> & inliers2,
					float maxDepth,
					char depthAxis,
					bool removeDuplicates)
{
	std::list<pcl::PointXYZ> addedPts;
	if(maxDepth > 0.0f && inliers1.size() && inliers1.size() == inliers2.size())
	{
		pcl::PointCloud<pcl::PointXYZ> tmpInliers1;
		pcl::PointCloud<pcl::PointXYZ> tmpInliers2;
		for(unsigned int i=0; i<inliers1.size(); ++i)
		{
			if((depthAxis == 'x' && inliers1.at(i).x < maxDepth && inliers2.at(i).x < maxDepth) ||
			   (depthAxis == 'y' && inliers1.at(i).y < maxDepth && inliers2.at(i).y < maxDepth) ||
			   (depthAxis == 'z' && inliers1.at(i).z < maxDepth && inliers2.at(i).z < maxDepth))
			{
				bool dup = false;
				if(removeDuplicates)
				{
					for(std::list<pcl::PointXYZ>::iterator iter = addedPts.begin(); iter!=addedPts.end(); ++iter)
					{
						if(iter->x == inliers1.at(i).x &&
						   iter->y == inliers1.at(i).y &&
						   iter->z == inliers1.at(i).z)
						{
							dup = true;
						}
					}
					if(!dup)
					{
						addedPts.push_back(inliers1.at(i));
					}
				}

				if(!dup)
				{
					tmpInliers1.push_back(inliers1.at(i));
					tmpInliers2.push_back(inliers2.at(i));
				}
			}
		}
		inliers1 = tmpInliers1;
		inliers2 = tmpInliers2;
	}
}

// Get transform from cloud2 to cloud1
Transform transformFromXYZCorrespondences(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud1,
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud2,
		double inlierThreshold,
		int iterations,
		int * inliers)
{
	Transform transform;
	if(cloud1->size() && cloud1->size() == cloud2->size())
	{
		// Not robust to outliers...
		//Eigen::Matrix4f transformMatrix;
		//pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
		//trans_est.estimateRigidTransformation (*cloud2, *cloud1, transformMatrix);

		// Robust to outliers RANSAC
		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
		for(unsigned int i = 0; i<cloud1->size(); ++i)
		{
			correspondences->push_back(pcl::Correspondence(i, i, pcl::euclideanDistance(cloud2->at(i), cloud1->at(i))));
		}

		pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> crsc;
		crsc.setInputCorrespondences(correspondences);
		crsc.setInputSource(cloud2);
		crsc.setInputTarget(cloud1);
		crsc.setMaximumIterations(iterations);
		crsc.setInlierThreshold(inlierThreshold);
		pcl::Correspondences correspondencesInliers;
		crsc.getCorrespondences(correspondencesInliers);

		UDEBUG("RANSAC inliers=%d outliers=%d", (int)correspondencesInliers.size(), (int)correspondences->size()-(int)correspondencesInliers.size());
		transform = util3d::transformFromEigen4f(crsc.getBestTransformation());

		if(correspondencesInliers.size() == correspondences->size() && transform.isIdentity())
		{
			//Wrong transform
			UDEBUG("Wrong transform: identity");
			transform.setNull();
			if(inliers)
			{
				*inliers = 0;
			}
		}
		else if(inliers)
		{
			*inliers = correspondencesInliers.size();
		}

		//std::cout << "transformMatrix: " << transformMatrix << std::endl;

		//std::cout << "quality: " << float(correspondencesRej.size()) / float(correspondences->size());
	}
	else
	{
		UDEBUG("not enough points to compute the transform");
	}
	return transform.inverse(); // inverse to get actual pose transform (not correspondences transform)
}

// return transform from source to target (All points must be finite!!!)
Transform icp(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_source,
			  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_target,
			  double maxCorrespondenceDistance,
			  int maximumIterations,
			  bool & hasConverged,
			  double & fitnessScore)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	// Set the input source and target
	icp.setInputTarget (cloud_target);
	icp.setInputSource (cloud_source);

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (maximumIterations);
	// Set the transformation epsilon (criterion 2)
	//icp.setTransformationEpsilon (transformationEpsilon);
	// Set the euclidean distance difference epsilon (criterion 3)
	//icp.setEuclideanFitnessEpsilon (1);
	//icp.setRANSACOutlierRejectionThreshold(maxCorrespondenceDistance);

	// Perform the alignment
	pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
	icp.align (cloud_source_registered);
	fitnessScore = icp.getFitnessScore();
	hasConverged = icp.hasConverged();

	return transformFromEigen4f(icp.getFinalTransformation());
}

// return transform from source to target (All points/normals must be finite!!!)
Transform icpPointToPlane(
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud_source,
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud_target,
		double maxCorrespondenceDistance,
		int maximumIterations,
		bool & hasConverged,
		double & fitnessScore)
{
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	// Set the input source and target
	icp.setInputTarget (cloud_target);
	icp.setInputSource (cloud_source);

	pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>::Ptr est;
	est.reset(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>);
	icp.setTransformationEstimation(est);

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (maximumIterations);
	// Set the transformation epsilon (criterion 2)
	//icp.setTransformationEpsilon (transformationEpsilon);
	// Set the euclidean distance difference epsilon (criterion 3)
	//icp.setEuclideanFitnessEpsilon (1);
	//icp.setRANSACOutlierRejectionThreshold(maxCorrespondenceDistance);

	// Perform the alignment
	pcl::PointCloud<pcl::PointNormal> cloud_source_registered;
	icp.align (cloud_source_registered);
	fitnessScore = icp.getFitnessScore();
	hasConverged = icp.hasConverged();

	return transformFromEigen4f(icp.getFinalTransformation());
}

// return transform from source to target (All points must be finite!!!)
Transform icp2D(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_source,
			  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_target,
			  double maxCorrespondenceDistance,
			  int maximumIterations,
			  bool & hasConverged,
			  double & fitnessScore)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	// Set the input source and target
	icp.setInputTarget (cloud_target);
	icp.setInputSource (cloud_source);

	pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr est;
	est.reset(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
	icp.setTransformationEstimation(est);

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (maximumIterations);
	// Set the transformation epsilon (criterion 2)
	//icp.setTransformationEpsilon (transformationEpsilon);
	// Set the euclidean distance difference epsilon (criterion 3)
	//icp.setEuclideanFitnessEpsilon (1);
	//icp.setRANSACOutlierRejectionThreshold(maxCorrespondenceDistance);

	// Perform the alignment
	pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
	icp.align (cloud_source_registered);
	fitnessScore = icp.getFitnessScore();
	hasConverged = icp.hasConverged();

	return transformFromEigen4f(icp.getFinalTransformation());
}

pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	// Normal estimation*
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals*/

	return cloud_with_normals;
}

// a kdtree is constructed with cloud_target, then nearest neighbor
// is computed for each cloud_source points.
int getCorrespondencesCount(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_source,
							const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_target,
							float maxDistance)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdTree->setInputCloud(cloud_target);
	int count = 0;
	float sqrdMaxDistance = maxDistance * maxDistance;
	for(unsigned int i=0; i<cloud_source->size(); ++i)
	{
		std::vector<int> ind(1);
		std::vector<float> dist(1);
		if(kdTree->nearestKSearch(cloud_source->at(i), 1, ind, dist) && dist[0] < sqrdMaxDistance)
		{
			++count;
		}
	}
	return count;
}

/**
 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(2,2) (4,4)]
 * realPairsCount = 5
 */
void findCorrespondences(
		const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<cv::Point2f, cv::Point2f> > & pairs)
{
	const std::list<int> & ids = uUniqueKeys(wordsA);
	pairs.clear();
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		std::list<cv::KeyPoint> ptsA = uValues(wordsA, *i);
		std::list<cv::KeyPoint> ptsB = uValues(wordsB, *i);
		if(ptsA.size() == 1 && ptsB.size() == 1)
		{
			pairs.push_back(std::pair<cv::Point2f, cv::Point2f>(ptsA.front().pt, ptsB.front().pt));
		}
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cvMat2Cloud(
		const cv::Mat & matrix,
		const Transform & tranform)
{
	UASSERT(matrix.type() == CV_32FC2 || matrix.type() == CV_32FC3);
	UASSERT(matrix.rows == 1);

	Eigen::Affine3f t = transformToEigen3f(tranform);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(matrix.cols);
	if(matrix.channels() == 2)
	{
		for(int i=0; i<matrix.cols; ++i)
		{
			cloud->at(i).x = matrix.at<cv::Vec2f>(0,i)[0];
			cloud->at(i).y = matrix.at<cv::Vec2f>(0,i)[1];
			cloud->at(i).z = 0.0f;
			cloud->at(i) = pcl::transformPoint(cloud->at(i), t);
		}
	}
	else // channels=3
	{
		for(int i=0; i<matrix.cols; ++i)
		{
			cloud->at(i).x = matrix.at<cv::Vec3f>(0,i)[0];
			cloud->at(i).y = matrix.at<cv::Vec3f>(0,i)[1];
			cloud->at(i).z = matrix.at<cv::Vec3f>(0,i)[2];
			cloud->at(i) = pcl::transformPoint(cloud->at(i), t);
		}
	}
	return cloud;
}

// If "voxel" > 0, "samples" is ignored
pcl::PointCloud<pcl::PointXYZ>::Ptr getICPReadyCloud(
		const cv::Mat & depth,
		float depthConstant,
		int decimation,
		double maxDepth,
		float voxel,
		int samples,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	cloud = cloudFromDepth(
			depth,
			depth.cols/2,
			depth.rows/2,
			1.0f/depthConstant,
			1.0f/depthConstant,
			decimation);

	if(cloud->size())
	{
		if(maxDepth>0.0)
		{
			cloud = passThrough(cloud, "z", 0, maxDepth);
		}

		if(cloud->size())
		{
			if(voxel>0)
			{
				cloud = voxelize(cloud, voxel);
			}
			else if(samples>0 && (int)cloud->size() > samples)
			{
				cloud = sampling(cloud, samples);
			}

			if(cloud->size())
			{
				if(!transform.isNull() && !transform.isIdentity())
				{
					cloud = transformPointCloud(cloud, transform);
				}
			}
		}
	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr concatenateClouds(const std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clouds)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr>::const_iterator iter = clouds.begin(); iter!=clouds.end(); ++iter)
	{
		*cloud += *(*iter);
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenateClouds(const std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::const_iterator iter = clouds.begin(); iter!=clouds.end(); ++iter)
	{
		*cloud+=*(*iter);
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get3DFASTKpts(
		const cv::Mat & image,
		const cv::Mat & imageDepth,
		float constant,
		int fastThreshold,
		bool fastNonmaxSuppression,
		float maxDepth)
{
	// Extract words
	cv::FastFeatureDetector detector(fastThreshold, fastNonmaxSuppression);
	std::vector<cv::KeyPoint> kpts;
	detector.detect(image, kpts);

	pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
	for(unsigned int i=0; i<kpts.size(); ++i)
	{
		pcl::PointXYZ pt = getDepth(imageDepth, int(kpts[i].pt.x+0.5f), int(kpts[i].pt.y+0.5f), (float)image.cols/2, (float)image.rows/2, 1.0f/constant, 1.0f/constant);
		if(uIsFinite(pt.z) && (maxDepth <= 0 || pt.z <= maxDepth))
		{
			points->push_back(pt);
		}
	}
	UDEBUG("points %d -> %d", (int)kpts.size(), (int)points->size());
	return points;
}

pcl::PolygonMesh::Ptr createMesh(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, float maxEdgeLength, bool smoothing)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud);

	if(smoothing)
	{
		// Init object (second point type is for the normals, even if unused)
		pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;

		mls.setComputeNormals (true);

		// Set parameters
		mls.setInputCloud (cloud);
		mls.setPolynomialFit (true);
		mls.setSearchMethod (tree);
		mls.setSearchRadius (maxEdgeLength);

		// Reconstruct
		mls.process (*cloud_with_normals);
	}
	else
	{
		// Normal estimation*
		pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		n.setInputCloud (cloud);
		n.setSearchMethod (tree);
		n.setKSearch (20);
		n.compute (*normals);
		//* normals should not contain the point normals + surface curvatures

		// Concatenate the XYZ and normal fields*
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals*/
	}

	cloud_with_normals = removeNaNNormalsFromPointCloud(cloud_with_normals);

	// Create search tree*
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (maxEdgeLength);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (*mesh);

	return mesh;
}

//On success, optimizedPoses is cleared and new poses are inserted in
void optimizeTOROGraph(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		std::map<int, Transform> & optimizedPoses,
		Transform & mapCorrection,
		int toroIterations,
		bool toroInitialGuess,
		std::list<std::map<int, Transform> > * intermediateGraphes) // contains poses after tree init to last one before the end
{
	if(edgeConstraints.size()>=1 && poses.size()>=2)
	{
		// Apply TORO optimization
		AISNavigation::TreeOptimizer3 pg;
		pg.verboseLevel = 0;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			float x,y,z, roll,pitch,yaw;
			pcl::getTranslationAndEulerAngles(transformToEigen3f(iter->second), x,y,z, roll,pitch,yaw);
			AISNavigation::TreePoseGraph3::Pose p(x, y, z, roll, pitch, yaw);
			AISNavigation::TreePoseGraph<AISNavigation::Operations3D<double> >::Vertex* v = pg.addVertex(iter->first, p);
			if (v)
			{
				v->transformation=AISNavigation::TreePoseGraph3::Transformation(p);
			}
			else
			{
				UERROR("cannot insert vertex %d!?", iter->first);
			}
		}

		for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
		{
			int id1 = iter->first;
			int id2 = iter->second.to();
			float x,y,z, roll,pitch,yaw;
			pcl::getTranslationAndEulerAngles(transformToEigen3f(iter->second.transform()), x,y,z, roll,pitch,yaw);
			AISNavigation::TreePoseGraph3::Pose p(x, y, z, roll, pitch, yaw);
			AISNavigation::TreePoseGraph3::InformationMatrix m;
			m=DMatrix<double>::I(6);

			AISNavigation::TreePoseGraph<AISNavigation::Operations3D<double> >::Vertex* v1=pg.vertex(id1);
			AISNavigation::TreePoseGraph<AISNavigation::Operations3D<double> >::Vertex* v2=pg.vertex(id2);
			AISNavigation::TreePoseGraph3::Transformation t(p);
			if (!pg.addEdge(v1, v2,t ,m))
			{
				UERROR("Map: Edge already exits between nodes %d and %d, skipping", id1, id2);
				return;
			}
		}
		pg.buildMST(pg.vertices.begin()->first); // pg.buildSimpleTree();

		UDEBUG("Initial guess...");
		if(toroInitialGuess)
		{
			pg.initializeOnTree(); // optional
		}

		pg.initializeTreeParameters();
		UDEBUG("Building TORO tree... (if a crash happens just after this msg, "
			   "TORO is not able to find the root of the graph!)");
		pg.initializeOptimization();

		UDEBUG("TORO iterate begin");
		for (int i=0; i<toroIterations; i++)
		{
			if(intermediateGraphes && (toroInitialGuess || i>0))
			{
				std::map<int, Transform> tmpPoses;
				for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
				{
					AISNavigation::TreePoseGraph<AISNavigation::Operations3D<double> >::Vertex* v=pg.vertex(iter->first);
					v->pose=v->transformation.toPoseType();
					Transform newPose = transformFromEigen3f(pcl::getTransformation(v->pose.x(), v->pose.y(), v->pose.z(), v->pose.roll(), v->pose.pitch(), v->pose.yaw()));

					tmpPoses.insert(std::pair<int, Transform>(iter->first, newPose));
				}
				intermediateGraphes->push_back(tmpPoses);
			}

			pg.iterate();
		}
		UDEBUG("TORO iterate end");

		optimizedPoses.clear();
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			AISNavigation::TreePoseGraph<AISNavigation::Operations3D<double> >::Vertex* v=pg.vertex(iter->first);
			v->pose=v->transformation.toPoseType();
			Transform newPose = transformFromEigen3f(pcl::getTransformation(v->pose.x(), v->pose.y(), v->pose.z(), v->pose.roll(), v->pose.pitch(), v->pose.yaw()));

			optimizedPoses.insert(std::pair<int, Transform>(iter->first, newPose));
		}

		Eigen::Matrix4f newPose = transformToEigen4f(optimizedPoses.at(poses.rbegin()->first));
		Eigen::Matrix4f oldPose = transformToEigen4f(poses.rbegin()->second);
		Eigen::Matrix4f poseCorrection = oldPose.inverse() * newPose; // transform from odom to correct odom
		Eigen::Matrix4f result = oldPose*poseCorrection*oldPose.inverse();
		mapCorrection = transformFromEigen4f(result);
	}
	else
	{
		UWARN("This method should be called at least with 2 poses and one link!");
	}
}

bool saveTOROGraph(
		const std::string & fileName,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints)
{
	FILE * file = fopen(fileName.c_str(), "w");

	if(file)
	{
		// VERTEX3 id x y z phi theta psi
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			float x,y,z, yaw,pitch,roll;
			pcl::getTranslationAndEulerAngles(transformToEigen3f(iter->second), x,y,z, roll, pitch, yaw);
			fprintf(file, "VERTEX3 %d %f %f %f %f %f %f\n",
					iter->first,
					x,
					y,
					z,
					roll,
					pitch,
					yaw);
		}

		//EDGE3 observed_vertex_id observing_vertex_id x y z roll pitch yaw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
		for(std::multimap<int, Link>::const_iterator iter = edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
		{
			float x,y,z, yaw,pitch,roll;
			pcl::getTranslationAndEulerAngles(transformToEigen3f(iter->second.transform()), x,y,z, roll, pitch, yaw);
			fprintf(file, "EDGE3 %d %d %f %f %f %f %f %f 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1\n",
					iter->first,
					iter->second.to(),
					x,
					y,
					z,
					roll,
					pitch,
					yaw);
		}
		UINFO("Graph saved to %s", fileName.c_str());
		fclose(file);
	}
	else
	{
		UERROR("Cannot save to file %s", fileName.c_str());
		return false;
	}
	return true;
}

bool loadTOROGraph(const std::string & fileName,
		std::map<int, Transform> & poses,
		std::multimap<int, std::pair<int, Transform> > & edgeConstraints)
{
	FILE * file = fopen(fileName.c_str(), "r");

	if(file)
	{
		char line[200];
		while ( fgets (line , 200 , file) != NULL )
		{
			std::vector<std::string> strList = uListToVector(uSplit(line, ' '));
			if(strList.size() == 8)
			{
				//VERTEX3
				int id = atoi(strList[1].c_str());
				float x = atof(strList[2].c_str());
				float y = atof(strList[3].c_str());
				float z = atof(strList[4].c_str());
				float roll = atof(strList[5].c_str());
				float pitch = atof(strList[6].c_str());
				float yaw = atof(strList[7].c_str());
				Transform pose = transformFromEigen3f(pcl::getTransformation(x, y, z, roll, pitch, yaw));
				std::map<int, Transform>::iterator iter = poses.find(id);
				if(iter != poses.end())
				{
					iter->second = pose;
				}
				else
				{
					UFATAL("");
				}
			}
			else if(strList.size() == 30)
			{
				//EDGE3
				int idFrom = atoi(strList[1].c_str());
				int idTo = atoi(strList[2].c_str());
				float x = atof(strList[3].c_str());
				float y = atof(strList[4].c_str());
				float z = atof(strList[5].c_str());
				float roll = atof(strList[6].c_str());
				float pitch = atof(strList[7].c_str());
				float yaw = atof(strList[8].c_str());
				Transform transform = transformFromEigen3f(pcl::getTransformation(x, y, z, roll, pitch, yaw));
				if(poses.find(idFrom) != poses.end() && poses.find(idTo) != poses.end())
				{
					std::pair<int, Transform> edge(idTo, transform);
					edgeConstraints.insert(std::pair<int, std::pair<int, Transform> >(idFrom, edge));
				}
				else
				{
					UFATAL("");
				}
			}
			else
			{
				UFATAL("Error parsing map file %s", fileName.c_str());
			}
		}

		UINFO("Graph loaded from %s", fileName.c_str());
		fclose(file);
	}
	else
	{
		UERROR("Cannot open file %s", fileName.c_str());
		return false;
	}
	return true;
}

cv::Mat create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans,
		float delta,
		float & xMin,
		float & yMin)
{
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > localScans;

	pcl::PointCloud<pcl::PointXYZ> minMax;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(uContains(scans, iter->first))
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::transformPointCloud(scans.at(iter->first), iter->second);
			pcl::PointXYZ min, max;
			pcl::getMinMax3D(*cloud, min, max);
			minMax.push_back(min);
			minMax.push_back(max);
			minMax.push_back(pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z()));
			localScans.insert(std::make_pair(iter->first, cloud));
		}
	}

	cv::Mat map;
	if(minMax.size())
	{
		//Get map size
		pcl::PointXYZ min, max;
		pcl::getMinMax3D(minMax, min, max);

		xMin = min.x-1.0f;
		yMin = min.y-1.0f;
		float xMax = max.x+1.0f;
		float yMax = max.y+1.0f;

		map = cv::Mat::ones((yMax - yMin) / delta, (xMax - xMin) / delta, CV_8S)*-1;
		for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter = localScans.begin(); iter!=localScans.end(); ++iter)
		{
			for(unsigned int i=0; i<iter->second->size(); ++i)
			{
				const Transform & pose = poses.at(iter->first);
				cv::Point2i start((pose.x()-xMin)/delta + 0.5f, (pose.y()-yMin)/delta + 0.5f);
				cv::Point2i end((iter->second->points[i].x-xMin)/delta + 0.5f, (iter->second->points[i].y-yMin)/delta + 0.5f);

				rayTrace(start, end, map); // trace free space

				map.at<char>(end.y, end.x) = 100; // obstacle
			}
		}
	}
	return map;
}

void rayTrace(const cv::Point2i & start, const cv::Point2i & end, cv::Mat & grid)
{
	UASSERT_MSG(start.x >= 0 && start.x < grid.cols, uFormat("start.x=%d grid.cols=%d", start.x, grid.cols).c_str());
	UASSERT_MSG(start.y >= 0 && start.y < grid.rows, uFormat("start.y=%d grid.rows=%d", start.y, grid.rows).c_str());
	UASSERT_MSG(end.x >= 0 && end.x < grid.cols, uFormat("end.x=%d grid.cols=%d", end.x, grid.cols).c_str());
	UASSERT_MSG(end.y >= 0 && end.y < grid.rows, uFormat("end.x=%d grid.cols=%d", end.y, grid.rows).c_str());

	cv::Point2i ptA, ptB;
	if(start.x > end.x)
	{
		ptA = end;
		ptB = start;
	}
	else
	{
		ptA = start;
		ptB = end;
	}

	float slope = float(ptB.y - ptA.y)/float(ptB.x - ptA.x);
	float b = ptA.y - slope*ptA.x;


	//ROS_WARN("start=%d,%d end=%d,%d", ptA.x, ptA.y, ptB.x, ptB.y);

	//ROS_WARN("y = %f*x + %f", slope, b);

	for(int x=ptA.x; x<ptB.x; ++x)
	{
		float lowerbound = float(x)*slope + b;
		float upperbound = float(x+1)*slope + b;

		if(lowerbound > upperbound)
		{
			float tmp = lowerbound;
			lowerbound = upperbound;
			upperbound = tmp;
		}

		//ROS_WARN("lowerbound=%f upperbound=%f", lowerbound, upperbound);
		UASSERT_MSG(lowerbound >= 0 && lowerbound < grid.rows, uFormat("lowerbound=%f grid.cols=%d x=%d slope=%f b=%f", lowerbound, grid.cols, x, slope, b).c_str());
		UASSERT_MSG(upperbound >= 0 && upperbound < grid.rows, uFormat("upperbound=%f grid.cols=%d x+1=%d slope=%f b=%f", upperbound, grid.cols, x+1, slope, b).c_str());
		for(int y = lowerbound; y<=(int)upperbound; ++y)
		{
			//if(grid.at<char>(y, x) == -1)
			{
				grid.at<char>(y, x) = 0; // free space
			}
		}
	}
}

}

}
