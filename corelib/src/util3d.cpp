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

#include <rtabmap/core/EpipolarGeometry.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#ifdef WITH_NONFREE
#include <opencv2/nonfree/features2d.hpp>
#endif
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <rtabmap/core/VWDictionary.h>
#include <cmath>
#include <stdio.h>

#include <zlib.h>

#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UTimer.h"
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
CompressionThread::CompressionThread(const cv::Mat & bytes, bool isImage) :
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
				compressedData_ = compressImage2(uncompressedData_, format_);
			}
			else
			{
				compressedData_ = compressData2(uncompressedData_);
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

pcl::PointCloud<pcl::PointXYZ>::Ptr generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		float fx,
		float fy,
		float cx,
		float cy,
		const Transform & transform)
{
	UASSERT(!depth.empty() && (depth.type() == CV_32FC1 || depth.type() == CV_16UC1));
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3d(new pcl::PointCloud<pcl::PointXYZ>);
	if(!depth.empty())
	{
		keypoints3d->resize(keypoints.size());
		for(unsigned int i=0; i!=keypoints.size(); ++i)
		{
			pcl::PointXYZ pt = util3d::projectDepthTo3D(
					depth,
					keypoints[i].pt.x,
					keypoints[i].pt.y,
					cx,
					cy,
					fx,
					fy,
					true);

			if(!transform.isNull() && !transform.isIdentity())
			{
				pt = pcl::transformPoint(pt, util3d::transformToEigen3f(transform));
			}
			keypoints3d->at(i) = pt;
		}
	}
	return keypoints3d;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generateKeypoints3DDisparity(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & disparity,
		float fx,
		float baseline,
		float cx,
		float cy,
		const Transform & transform)
{
	UASSERT(!disparity.empty() && (disparity.type() == CV_16SC1 || disparity.type() == CV_32F));
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3d(new pcl::PointCloud<pcl::PointXYZ>);
	keypoints3d->resize(keypoints.size());
	for(unsigned int i=0; i!=keypoints.size(); ++i)
	{
		pcl::PointXYZ pt = util3d::projectDisparityTo3D(
				keypoints[i].pt,
				disparity,
				cx,
				cy,
				fx,
				baseline);

		if(pcl::isFinite(pt) && !transform.isNull() && !transform.isIdentity())
		{
			pt = pcl::transformPoint(pt, util3d::transformToEigen3f(transform));
		}
		keypoints3d->at(i) = pt;
	}
	return keypoints3d;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generateKeypoints3DStereo(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		float fx,
		float baseline,
		float cx,
		float cy,
		const Transform & transform,
		int flowWinSize,
		int flowMaxLevel,
		int flowIterations,
		double flowEps)
{
	UASSERT(!leftImage.empty() && !rightImage.empty() &&
			leftImage.type() == CV_8UC1 && rightImage.type() == CV_8UC1 &&
			leftImage.rows == rightImage.rows && leftImage.cols == rightImage.cols);

	std::vector<cv::Point2f> leftCorners;
	cv::KeyPoint::convert(keypoints, leftCorners);

	// Find features in the new left image
	std::vector<unsigned char> status;
	std::vector<float> err;
	std::vector<cv::Point2f> rightCorners;
	UDEBUG("cv::calcOpticalFlowPyrLK() begin");
	cv::calcOpticalFlowPyrLK(
			leftImage,
			rightImage,
			leftCorners,
			rightCorners,
			status,
			err,
			cv::Size(flowWinSize, flowWinSize), flowMaxLevel,
			cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations, flowEps),
			cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
	UDEBUG("cv::calcOpticalFlowPyrLK() end");

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3d(new pcl::PointCloud<pcl::PointXYZ>);
	keypoints3d->resize(keypoints.size());
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	UASSERT(status.size() == keypoints.size());
	for(unsigned int i=0; i<status.size(); ++i)
	{
		pcl::PointXYZ pt(bad_point, bad_point, bad_point);
		if(status[i])
		{
			float disparity = leftCorners[i].x - rightCorners[i].x;
			if(disparity > 0.0f)
			{
				pcl::PointXYZ tmpPt = util3d::projectDisparityTo3D(
						leftCorners[i],
						disparity,
						cx, cy, fx, baseline);

				if(pcl::isFinite(tmpPt))
				{
					pt = tmpPt;
					if(!transform.isNull() && !transform.isIdentity())
					{
						pt = pcl::transformPoint(pt, util3d::transformToEigen3f(transform));
					}
				}
			}
		}

		keypoints3d->at(i) = pt;
	}
	return keypoints3d;
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
		float maxDepth,
		std::set<int> * uniqueCorrespondences)
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
				if(uniqueCorrespondences)
				{
					uniqueCorrespondences->insert(*iter);
				}
			}
		}
	}
	inliers1.resize(oi);
	inliers2.resize(oi);
}

pcl::PointXYZ projectDepthTo3D(
		const cv::Mat & depthImage,
		float x, float y,
		float cx, float cy,
		float fx, float fy,
		bool smoothing,
		float maxZError)
{
	UASSERT(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1);

	pcl::PointXYZ pt;
	float bad_point = std::numeric_limits<float>::quiet_NaN ();

	int u = int(x+0.5f);
	int v = int(y+0.5f);

	if(!(u >=0 && u<depthImage.cols && v >=0 && v<depthImage.rows))
	{
		UERROR("!(x >=0 && x<depthImage.cols && y >=0 && y<depthImage.rows) cond failed! returning bad point. (x=%f (u=%d), y=%f (v=%d), cols=%d, rows=%d)",
				x,u,y,v,depthImage.cols, depthImage.rows);
		pt.x = pt.y = pt.z = bad_point;
		return pt;
	}

	bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

	// Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
	// https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
	// Window weights:
	//  | 1 | 2 | 1 |
	//  | 2 | 4 | 2 |
	//  | 1 | 2 | 1 |
	int u_start = std::max(u-1, 0);
	int v_start = std::max(v-1, 0);
	int u_end = std::min(u+1, depthImage.cols-1);
	int v_end = std::min(v+1, depthImage.rows-1);

	float depth = isInMM?(float)depthImage.at<uint16_t>(v,u)*0.001f:depthImage.at<float>(v,u);
	if(depth!=0.0f && uIsFinite(depth))
	{
		if(smoothing)
		{
			float sumWeights = 0.0f;
			float sumDepths = 0.0f;
			for(int uu = u_start; uu <= u_end; ++uu)
			{
				for(int vv = v_start; vv <= v_end; ++vv)
				{
					if(!(uu == u && vv == v))
					{
						float d = isInMM?(float)depthImage.at<uint16_t>(vv,uu)*0.001f:depthImage.at<float>(vv,uu);
						// ignore if not valid or depth difference is too high
						if(d != 0.0f && uIsFinite(d) && fabs(d - depth) < maxZError)
						{
							if(uu == u || vv == v)
							{
								sumWeights+=2.0f;
								d*=2.0f;
							}
							else
							{
								sumWeights+=1.0f;
							}
							sumDepths += d;
						}
					}
				}
			}
			// set window weight to center point
			depth *= 4.0f;
			sumWeights += 4.0f;

			// mean
			depth = (depth+sumDepths)/sumWeights;
		}

		// Use correct principal point from calibration
		cx = cx > 0.0f ? cx : float(depthImage.cols/2) - 0.5f; //cameraInfo.K.at(2)
		cy = cy > 0.0f ? cy : float(depthImage.rows/2) - 0.5f; //cameraInfo.K.at(5)

		// Fill in XYZ
		pt.x = (x - cx) * depth / fx;
		pt.y = (y - cy) * depth / fy;
		pt.z = depth;
	}
	else
	{
		pt.x = pt.y = pt.z = bad_point;
	}
	return pt;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDepth(
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation)
{
	UASSERT(!imageDepth.empty() && (imageDepth.type() == CV_16UC1 || imageDepth.type() == CV_32FC1));
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

			pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w, h, cx, cy, fx, fy, false);
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
		float cx, float cy,
		float fx, float fy,
		int decimation)
{
	UASSERT(imageRgb.rows == imageDepth.rows && imageRgb.cols == imageDepth.cols);
	UASSERT(!imageDepth.empty() && (imageDepth.type() == CV_16UC1 || imageDepth.type() == CV_32FC1));
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

			pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w, h, cx, cy, fx, fy, false);
			pt.x = ptXYZ.x;
			pt.y = ptXYZ.y;
			pt.z = ptXYZ.z;
		}
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDisparity(
		const cv::Mat & imageDisparity,
		float cx, float cy,
		float fx, float baseline,
		int decimation)
{
	UASSERT(imageDisparity.type() == CV_32FC1 || imageDisparity.type()==CV_16SC1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if(decimation < 1)
	{
		return cloud;
	}

	//cloud.header = cameraInfo.header;
	cloud->height = imageDisparity.rows/decimation;
	cloud->width  = imageDisparity.cols/decimation;
	cloud->is_dense = false;
	cloud->resize(cloud->height * cloud->width);

	if(imageDisparity.type()==CV_16SC1)
	{
		for(int h = 0; h < imageDisparity.rows && h/decimation < (int)cloud->height; h+=decimation)
		{
			for(int w = 0; w < imageDisparity.cols && w/decimation < (int)cloud->width; w+=decimation)
			{
				float disp = float(imageDisparity.at<short>(h,w))/16.0f;
				cloud->at((h/decimation)*cloud->width + (w/decimation)) = projectDisparityTo3D(cv::Point2f(w, h), disp, cx, cy, fx, baseline);
			}
		}
	}
	else
	{
		for(int h = 0; h < imageDisparity.rows && h/decimation < (int)cloud->height; h+=decimation)
		{
			for(int w = 0; w < imageDisparity.cols && w/decimation < (int)cloud->width; w+=decimation)
			{
				float disp = imageDisparity.at<float>(h,w);
				cloud->at((h/decimation)*cloud->width + (w/decimation)) = projectDisparityTo3D(cv::Point2f(w, h), disp, cx, cy, fx, baseline);
			}
		}
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDisparityRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDisparity,
		float cx, float cy,
		float fx, float baseline,
		int decimation)
{
	UASSERT(imageRgb.rows == imageDisparity.rows &&
			imageRgb.cols == imageDisparity.cols &&
			(imageDisparity.type() == CV_32FC1 || imageDisparity.type()==CV_16SC1));
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
	cloud->height = imageRgb.rows/decimation;
	cloud->width  = imageRgb.cols/decimation;
	cloud->is_dense = false;
	cloud->resize(cloud->height * cloud->width);

	for(int h = 0; h < imageRgb.rows && h/decimation < (int)cloud->height; h+=decimation)
	{
		for(int w = 0; w < imageRgb.cols && w/decimation < (int)cloud->width; w+=decimation)
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

			float disp = imageDisparity.type()==CV_16SC1?float(imageDisparity.at<short>(h,w))/16.0f:imageDisparity.at<float>(h,w);
			pcl::PointXYZ ptXYZ = projectDisparityTo3D(cv::Point2f(w, h), disp, cx, cy, fx, baseline);
			pt.x = ptXYZ.x;
			pt.y = ptXYZ.y;
			pt.z = ptXYZ.z;
		}
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromStereoImages(
		const cv::Mat & imageLeft,
		const cv::Mat & imageRight,
		float cx, float cy,
		float fx, float baseline,
		int decimation)
{
	UASSERT(imageRight.type() == CV_8UC1);

	cv::Mat leftMono;
	if(imageLeft.channels() == 3)
	{
		cv::cvtColor(imageLeft, leftMono, CV_BGR2GRAY);
	}
	else
	{
		leftMono = imageLeft;
	}
	return rtabmap::util3d::cloudFromDisparityRGB(
			imageLeft,
			util3d::disparityFromStereoImages(leftMono, imageRight),
			cx, cy,
			fx, baseline,
			decimation);
}

cv::Mat disparityFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage)
{
	UASSERT(!leftImage.empty() && !rightImage.empty() &&
			leftImage.type() == CV_8UC1 && rightImage.type() == CV_8UC1 &&
			leftImage.cols == rightImage.cols &&
			leftImage.rows == rightImage.rows);
	cv::StereoBM stereo(cv::StereoBM::BASIC_PRESET, 160, 15);
	cv::Mat disparity;
	stereo(leftImage, rightImage, disparity, CV_16SC1);
	cv::filterSpeckles(disparity, 0, 1000, 16);
	return disparity;
}

cv::Mat disparityFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		int flowWinSize,
		int flowMaxLevel,
		int flowIterations,
		double flowEps,
		float maxCorrespondencesSlope)
{
	UASSERT(!leftImage.empty() && !rightImage.empty() &&
			leftImage.type() == CV_8UC1 && rightImage.type() == CV_8UC1 &&
			leftImage.cols == rightImage.cols &&
			leftImage.rows == rightImage.rows);

	// Find features in the new left image
	std::vector<unsigned char> status;
	std::vector<float> err;
	std::vector<cv::Point2f> rightCorners;
	UDEBUG("cv::calcOpticalFlowPyrLK() begin");
	cv::calcOpticalFlowPyrLK(
			leftImage,
			rightImage,
			leftCorners,
			rightCorners,
			status,
			err,
			cv::Size(flowWinSize, flowWinSize), flowMaxLevel,
			cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations, flowEps),
			cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
	UDEBUG("cv::calcOpticalFlowPyrLK() end");

	return disparityFromStereoCorrespondences(leftImage, leftCorners, rightCorners, status, maxCorrespondencesSlope);
}

cv::Mat depthFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		float fx,
		float baseline,
		int flowWinSize,
		int flowMaxLevel,
		int flowIterations,
		double flowEps)
{
	UASSERT(!leftImage.empty() && !rightImage.empty() &&
			leftImage.type() == CV_8UC1 && rightImage.type() == CV_8UC1 &&
			leftImage.cols == rightImage.cols &&
			leftImage.rows == rightImage.rows);
	UASSERT(fx > 0.0f && baseline > 0.0f);

	// Find features in the new left image
	std::vector<unsigned char> status;
	std::vector<float> err;
	std::vector<cv::Point2f> rightCorners;
	UDEBUG("cv::calcOpticalFlowPyrLK() begin");
	cv::calcOpticalFlowPyrLK(
			leftImage,
			rightImage,
			leftCorners,
			rightCorners,
			status,
			err,
			cv::Size(flowWinSize, flowWinSize), flowMaxLevel,
			cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations, flowEps),
			cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
	UDEBUG("cv::calcOpticalFlowPyrLK() end");

	return depthFromStereoCorrespondences(leftImage, leftCorners, rightCorners, status, fx, baseline);
}

cv::Mat disparityFromStereoCorrespondences(
		const cv::Mat & leftImage,
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const std::vector<unsigned char> & mask,
		float maxSlope)
{
	UASSERT(!leftImage.empty() && leftCorners.size() == rightCorners.size());
	UASSERT(mask.size() == 0 || mask.size() == leftCorners.size());
	cv::Mat disparity = cv::Mat::zeros(leftImage.rows, leftImage.cols, CV_32FC1);
	for(unsigned int i=0; i<leftCorners.size(); ++i)
	{
		if(mask.size() == 0 || mask[i])
		{
			float d = leftCorners[i].x - rightCorners[i].x;
			float slope = fabs((leftCorners[i].y - rightCorners[i].y) / (leftCorners[i].x - rightCorners[i].x));
			if(d > 0.0f && slope < maxSlope)
			{
				disparity.at<float>(int(leftCorners[i].y+0.5f), int(leftCorners[i].x+0.5f)) = d;
			}
		}
	}
	return disparity;
}

cv::Mat depthFromStereoCorrespondences(
		const cv::Mat & leftImage,
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const std::vector<unsigned char> & mask,
		float fx, float baseline)
{
	UASSERT(!leftImage.empty() && leftCorners.size() == rightCorners.size());
	UASSERT(mask.size() == 0 || mask.size() == leftCorners.size());
	cv::Mat depth = cv::Mat::zeros(leftImage.rows, leftImage.cols, CV_32FC1);
	for(unsigned int i=0; i<leftCorners.size(); ++i)
	{
		if(mask.size() == 0 || mask[i])
		{
			float disparity = leftCorners[i].x - rightCorners[i].x;
			if(disparity > 0.0f)
			{
				float d = baseline * fx / disparity;
				depth.at<float>(int(leftCorners[i].y+0.5f), int(leftCorners[i].x+0.5f)) = d;
			}
		}
	}
	return depth;
}

// inspired from ROS image_geometry/src/stereo_camera_model.cpp
pcl::PointXYZ projectDisparityTo3D(
		const cv::Point2f & pt,
		float disparity,
		float cx, float cy, float fx, float baseline)
{
	if(disparity > 0.0f && baseline > 0.0f && fx > 0.0f)
	{
		float W = disparity/baseline;// + (right_.cx() - left_.cx()) / Tx;
		return pcl::PointXYZ((pt.x - cx)/W, (pt.y - cy)/W, fx/W);
	}
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	return pcl::PointXYZ(bad_point, bad_point, bad_point);
}

pcl::PointXYZ projectDisparityTo3D(
		const cv::Point2f & pt,
		const cv::Mat & disparity,
		float cx, float cy, float fx, float baseline)
{
	UASSERT(!disparity.empty() && (disparity.type() == CV_32FC1 || disparity.type() == CV_16SC1));
	int u = int(pt.x+0.5f);
	int v = int(pt.y+0.5f);
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	if(uIsInBounds(u, 0, disparity.cols-1) &&
	   uIsInBounds(v, 0, disparity.rows-1))
	{
		float d = disparity.type() == CV_16SC1?float(disparity.at<short>(v,u))/16.0f:disparity.at<float>(v,u);
		return projectDisparityTo3D(pt, d, cx, cy, fx, baseline);
	}
	return pcl::PointXYZ(bad_point, bad_point, bad_point);
}

cv::Mat depthFromDisparity(const cv::Mat & disparity,
		float fx, float baseline,
		int type)
{
	UASSERT(!disparity.empty() && (disparity.type() == CV_32FC1 || disparity.type() == CV_16SC1));
	UASSERT(type == CV_32FC1 || type == CV_16U);
	cv::Mat depth = cv::Mat::zeros(disparity.rows, disparity.cols, type);
	for (int i = 0; i < disparity.rows; i++)
	{
		for (int j = 0; j < disparity.cols; j++)
		{
			float disparity_value = disparity.type() == CV_16SC1?float(disparity.at<short>(i,j))/16.0f:disparity.at<float>(i,j);
			if (disparity_value > 0.0f)
			{
				// baseline * focal / disparity
				float d = baseline * fx / disparity_value;
				if(depth.type() == CV_32FC1)
				{
					depth.at<float>(i,j) = d;
				}
				else
				{
					depth.at<unsigned short>(i,j) = (unsigned short)(d*1000.0f);
				}
			}
		}
	}
	return depth;
}

cv::Mat depth2DFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
	cv::Mat depth2d(1, (int)cloud.size(), CV_32FC2);
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

// ".png" or ".jpg"
cv::Mat compressImage2(const cv::Mat & image, const std::string & format)
{
	std::vector<unsigned char> bytes = compressImage(image, format);
	if(bytes.size())
	{
		return cv::Mat(1, bytes.size(), CV_8UC1, bytes.data()).clone();
	}
	return cv::Mat();
}

cv::Mat uncompressImage(const cv::Mat & bytes)
{
	 cv::Mat image;
	if(!bytes.empty())
	{
#if CV_MAJOR_VERSION>2 || (CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >=4)
		image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
#else
		image = cv::imdecode(bytes, -1);
#endif
	}
	return image;
}

cv::Mat uncompressImage(const std::vector<unsigned char> & bytes)
{
	 cv::Mat image;
	if(bytes.size())
	{
#if CV_MAJOR_VERSION>2 || (CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >=4)
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
		uLong sourceLen = uLong(data.total())*uLong(data.elemSize());
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

cv::Mat compressData2(const cv::Mat & data)
{
	cv::Mat bytes;
	if(!data.empty())
	{
		uLong sourceLen = uLong(data.total())*uLong(data.elemSize());
		uLong destLen = compressBound(sourceLen);
		bytes = cv::Mat(1, destLen+3*sizeof(int), CV_8UC1);
		int errCode = compress(
						(Bytef *)bytes.data,
						&destLen,
						(const Bytef *)data.data,
						sourceLen);
		bytes = cv::Mat(bytes, cv::Rect(0,0, destLen+3*sizeof(int), 1));
		*((int*)&bytes.data[destLen]) = data.rows;
		*((int*)&bytes.data[destLen+sizeof(int)]) = data.cols;
		*((int*)&bytes.data[destLen+2*sizeof(int)]) = data.type();

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

cv::Mat uncompressData(const cv::Mat & bytes)
{
	UASSERT(bytes.empty() || bytes.type() == CV_8UC1);
	return uncompressData(bytes.data, bytes.cols*bytes.rows);
}

cv::Mat uncompressData(const std::vector<unsigned char> & bytes)
{
	return uncompressData(bytes.data(), bytes.size());
}

cv::Mat uncompressData(const unsigned char * bytes, unsigned long size)
{
	cv::Mat data;
	if(bytes && size>=3*sizeof(int))
	{
		//last 3 int elements are matrix size and type
		int height = *((int*)&bytes[size-3*sizeof(int)]);
		int width = *((int*)&bytes[size-2*sizeof(int)]);
		int type = *((int*)&bytes[size-1*sizeof(int)]);

		// If the size is higher, it may be a wrong data format.
		UASSERT_MSG(height>=0 && height<10000 &&
				    width>=0 && width<10000,
				    uFormat("size=%d, height=%d width=%d type=%d", size, height, width, type).c_str());

		data = cv::Mat(height, width, type);
		uLongf totalUncompressed = uLongf(data.total())*uLongf(data.elemSize());

		int errCode = uncompress(
						(Bytef*)data.data,
						&totalUncompressed,
						(const Bytef*)bytes,
						uLong(size));

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
		cv::Mat points1(1, (int)pairs.size(), CV_32FC2);
		cv::Mat points2(1, (int)pairs.size(), CV_32FC2);

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
		pcl::PointXYZ pt1 = projectDepthTo3D(depthImage1, iter->first.x, iter->first.y, cx, cy, fx, fy, true);
		pcl::PointXYZ pt2 = projectDepthTo3D(depthImage2, iter->second.x, iter->second.y, cx, cy, fx, fy, true);
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
		bool refineModel,
		double refineModelSigma,
		int refineModelIterations,
		std::vector<int> * inliersOut)
{
	//NOTE: this method is a mix of two methods:
	//  - getRemainingCorrespondences() in pcl/registration/impl/correspondence_rejection_sample_consensus.hpp
	//  - refineModel() in pcl/sample_consensus/sac.h

	Transform transform;
	if(cloud1->size() >=3 && cloud1->size() == cloud2->size())
	{
		// RANSAC
		UDEBUG("iterations=%d inlierThreshold=%f", iterations, inlierThreshold);
		std::vector<int> source_indices (cloud2->size());
		std::vector<int> target_indices (cloud1->size());

		// Copy the query-match indices
		for (int i = 0; i < (int)cloud1->size(); ++i)
		{
			source_indices[i] = i;
			target_indices[i] = i;
		}

		// From the set of correspondences found, attempt to remove outliers
		// Create the registration model
		pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model;
		model.reset(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(cloud2, source_indices));
		// Pass the target_indices
		model->setInputTarget (cloud1, target_indices);
		// Create a RANSAC model
		pcl::RandomSampleConsensus<pcl::PointXYZ> sac (model, inlierThreshold);
		sac.setMaxIterations(iterations);

		// Compute the set of inliers
		if(sac.computeModel())
		{
			std::vector<int> inliers;
			Eigen::VectorXf model_coefficients;

			sac.getInliers(inliers);
			sac.getModelCoefficients (model_coefficients);

			if (refineModel)
			{
				double inlier_distance_threshold_sqr = inlierThreshold * inlierThreshold;
				double error_threshold = inlierThreshold;
				double sigma_sqr = refineModelSigma * refineModelSigma;
				int refine_iterations = 0;
				bool inlier_changed = false, oscillating = false;
				std::vector<int> new_inliers, prev_inliers = inliers;
				std::vector<size_t> inliers_sizes;
				Eigen::VectorXf new_model_coefficients = model_coefficients;
				do
				{
					// Optimize the model coefficients
					model->optimizeModelCoefficients (prev_inliers, new_model_coefficients, new_model_coefficients);
					inliers_sizes.push_back (prev_inliers.size ());

					// Select the new inliers based on the optimized coefficients and new threshold
					model->selectWithinDistance (new_model_coefficients, error_threshold, new_inliers);
					UDEBUG("RANSAC refineModel: Number of inliers found (before/after): %d/%d, with an error threshold of %f.",
							(int)prev_inliers.size (), (int)new_inliers.size (), error_threshold);

					if (new_inliers.empty ())
					{
						++refine_iterations;
						if (refine_iterations >= refineModelIterations)
						{
							break;
						}
						continue;
					}

					// Estimate the variance and the new threshold
					double variance = model->computeVariance ();
					error_threshold = sqrt (std::min (inlier_distance_threshold_sqr, sigma_sqr * variance));

					UDEBUG ("RANSAC refineModel: New estimated error threshold: %f (variance=%f) on iteration %d out of %d.",
						  error_threshold, variance, refine_iterations, refineModelIterations);
					inlier_changed = false;
					std::swap (prev_inliers, new_inliers);

					// If the number of inliers changed, then we are still optimizing
					if (new_inliers.size () != prev_inliers.size ())
					{
						// Check if the number of inliers is oscillating in between two values
						if (inliers_sizes.size () >= 4)
						{
							if (inliers_sizes[inliers_sizes.size () - 1] == inliers_sizes[inliers_sizes.size () - 3] &&
							inliers_sizes[inliers_sizes.size () - 2] == inliers_sizes[inliers_sizes.size () - 4])
							{
								oscillating = true;
								break;
							}
						}
						inlier_changed = true;
						continue;
					}

					// Check the values of the inlier set
					for (size_t i = 0; i < prev_inliers.size (); ++i)
					{
						// If the value of the inliers changed, then we are still optimizing
						if (prev_inliers[i] != new_inliers[i])
						{
							inlier_changed = true;
							break;
						}
					}
				}
				while (inlier_changed && ++refine_iterations < refineModelIterations);

				// If the new set of inliers is empty, we didn't do a good job refining
				if (new_inliers.empty ())
				{
					UWARN ("RANSAC refineModel: Refinement failed: got an empty set of inliers!");
				}

				if (oscillating)
				{
					UDEBUG("RANSAC refineModel: Detected oscillations in the model refinement.");
				}

				std::swap (inliers, new_inliers);
				model_coefficients = new_model_coefficients;
			}

			if (inliers.size() >= 3)
			{
				if(inliersOut)
				{
					*inliersOut = inliers;
				}

				// get best transformation
				Eigen::Matrix4f bestTransformation;
				bestTransformation.row (0) = model_coefficients.segment<4>(0);
				bestTransformation.row (1) = model_coefficients.segment<4>(4);
				bestTransformation.row (2) = model_coefficients.segment<4>(8);
				bestTransformation.row (3) = model_coefficients.segment<4>(12);

				transform = util3d::transformFromEigen4f(bestTransformation);
				UDEBUG("RANSAC inliers=%d/%d tf=%s", (int)inliers.size(), (int)cloud1->size(), transform.prettyPrint().c_str());

				return transform.inverse(); // inverse to get actual pose transform (not correspondences transform)
			}
			else
			{
				UDEBUG("RANSAC: Model with inliers < 3");
			}
		}
		else
		{
			UDEBUG("RANSAC: Failed to find model");
		}
	}
	else
	{
		UDEBUG("Not enough points to compute the transform");
	}
	return Transform();
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
	icp.setRANSACOutlierRejectionThreshold(maxCorrespondenceDistance);

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
	icp.setRANSACOutlierRejectionThreshold(maxCorrespondenceDistance);

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
	icp.setRANSACOutlierRejectionThreshold(maxCorrespondenceDistance);

	// Perform the alignment
	pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
	icp.align (cloud_source_registered);
	fitnessScore = icp.getFitnessScore();
	hasConverged = icp.hasConverged();

	return transformFromEigen4f(icp.getFinalTransformation());
}

pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int normalKSearch)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	// Normal estimation*
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (normalKSearch);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals*/

	return cloud_with_normals;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int normalKSearch)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud);

	// Normal estimation*
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (normalKSearch);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals*/

	return cloud_with_normals;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr computeNormalsSmoothed(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float smoothingSearchRadius,
		bool smoothingPolynomialFit)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud);

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;

	mls.setComputeNormals (true);

	// Set parameters
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (smoothingPolynomialFit);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (smoothingSearchRadius);

	// Reconstruct
	mls.process (*cloud_with_normals);

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
		float fx,
		float fy,
		float cx,
		float cy,
		int decimation,
		double maxDepth,
		float voxel,
		int samples,
		const Transform & transform)
{
	UASSERT(!depth.empty() && (depth.type() == CV_16UC1 || depth.type() == CV_32FC1));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	cloud = cloudFromDepth(
			depth,
			cx,
			cy,
			fx,
			fy,
			decimation);

	if(cloud->size())
	{
		if(maxDepth>0.0)
		{
			cloud = passThrough<pcl::PointXYZ>(cloud, "z", 0, maxDepth);
		}

		if(cloud->size())
		{
			if(voxel>0)
			{
				cloud = voxelize<pcl::PointXYZ>(cloud, voxel);
			}
			else if(samples>0 && (int)cloud->size() > samples)
			{
				cloud = sampling<pcl::PointXYZ>(cloud, samples);
			}

			if(cloud->size())
			{
				if(!transform.isNull() && !transform.isIdentity())
				{
					cloud = transformPointCloud<pcl::PointXYZ>(cloud, transform);
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
		pcl::PointXYZ pt = projectDepthTo3D(imageDepth, kpts[i].pt.x, kpts[i].pt.y, 0, 0, 1.0f/constant, 1.0f/constant, true);
		if(uIsFinite(pt.z) && (maxDepth <= 0 || pt.z <= maxDepth))
		{
			points->push_back(pt);
		}
	}
	UDEBUG("points %d -> %d", (int)kpts.size(), (int)points->size());
	return points;
}

pcl::PolygonMesh::Ptr createMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloudWithNormals,
		float gp3SearchRadius,
		float gp3Mu,
		int gp3MaximumNearestNeighbors,
		float gp3MaximumSurfaceAngle,
		float gp3MinimumAngle,
		float gp3MaximumAngle,
		bool gp3NormalConsistency)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormalsNoNaN = removeNaNNormalsFromPointCloud<pcl::PointXYZRGBNormal>(cloudWithNormals);

	// Create search tree*
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud (cloudWithNormalsNoNaN);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (gp3SearchRadius);

	// Set typical values for the parameters
	gp3.setMu (gp3Mu);
	gp3.setMaximumNearestNeighbors (gp3MaximumNearestNeighbors);
	gp3.setMaximumSurfaceAngle(gp3MaximumSurfaceAngle); // 45 degrees
	gp3.setMinimumAngle(gp3MinimumAngle); // 10 degrees
	gp3.setMaximumAngle(gp3MaximumAngle); // 120 degrees
	gp3.setNormalConsistency(gp3NormalConsistency);

	// Get result
	gp3.setInputCloud (cloudWithNormalsNoNaN);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (*mesh);

	return mesh;
}

std::multimap<int, Link>::iterator findLink(
		std::multimap<int, Link> & links,
		int from,
		int to)
{
	std::multimap<int, Link>::iterator iter = links.find(from);
	while(iter != links.end() && iter->first == from)
	{
		if(iter->second.to() == to)
		{
			return iter;
		}
		++iter;
	}

	// let's try to -> from
	iter = links.find(to);
	while(iter != links.end() && iter->first == to)
	{
		if(iter->second.to() == from)
		{
			return iter;
		}
		++iter;
	}
	return links.end();
}


// <int, depth> margin=0 means infinite margin
std::map<int, int> generateDepthGraph(
		const std::multimap<int, Link> & links,
		int fromId,
		int depth)
{
	UASSERT(depth >= 0);
	//UDEBUG("signatureId=%d, neighborsMargin=%d", signatureId, margin);
	std::map<int, int> ids;
	if(fromId<=0)
	{
		return ids;
	}

	std::list<int> curentDepthList;
	std::set<int> nextDepth;
	nextDepth.insert(fromId);
	int d = 0;
	while((depth == 0 || d < depth) && nextDepth.size())
	{
		curentDepthList = std::list<int>(nextDepth.begin(), nextDepth.end());
		nextDepth.clear();

		for(std::list<int>::iterator jter = curentDepthList.begin(); jter!=curentDepthList.end(); ++jter)
		{
			if(ids.find(*jter) == ids.end())
			{
				std::set<int> marginIds;

				ids.insert(std::pair<int, int>(*jter, d));

				for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
				{
					if(iter->second.from() == *jter)
					{
						marginIds.insert(iter->second.to());
					}
					else if(iter->second.to() == *jter)
					{
						marginIds.insert(iter->second.from());
					}
				}

				// Margin links
				for(std::set<int>::const_iterator iter=marginIds.begin(); iter!=marginIds.end(); ++iter)
				{
					if( !uContains(ids, *iter) && nextDepth.find(*iter) == nextDepth.end())
					{
						nextDepth.insert(*iter);
					}
				}
			}
		}
		++d;
	}
	return ids;
}

void optimizeTOROGraph(
		const std::map<int, int> & depthGraph,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		std::map<int, Transform> & optimizedPoses,
		int toroIterations,
		bool toroInitialGuess,
		std::list<std::map<int, Transform> > * intermediateGraphes)
{
	optimizedPoses.clear();
	if(depthGraph.size() && poses.size()>=2 && links.size()>=1)
	{
		// Modify IDs using the margin from the current signature (TORO root will be the last signature)
		int m = 0;
		int toroId = 1;
		std::map<int, int> rtabmapToToro; // <RTAB-Map ID, TORO ID>
		std::map<int, int> toroToRtabmap; // <TORO ID, RTAB-Map ID>
		std::map<int, int> idsTmp = depthGraph;
		while(idsTmp.size())
		{
			for(std::map<int, int>::iterator iter = idsTmp.begin(); iter!=idsTmp.end();)
			{
				if(m == iter->second)
				{
					rtabmapToToro.insert(std::make_pair(iter->first, toroId));
					toroToRtabmap.insert(std::make_pair(toroId, iter->first));
					++toroId;
					idsTmp.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
			++m;
		}

		std::map<int, rtabmap::Transform> posesToro;
		std::multimap<int, rtabmap::Link> edgeConstraintsToro;
		for(std::map<int, rtabmap::Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if(uContains(depthGraph, iter->first))
			{
				posesToro.insert(std::make_pair(rtabmapToToro.at(iter->first), iter->second));
			}
		}
		for(std::multimap<int, rtabmap::Link>::const_iterator iter = links.begin();
			iter!=links.end();
			++iter)
		{
			if(uContains(depthGraph, iter->second.from()) && uContains(depthGraph, iter->second.to()))
			{
				edgeConstraintsToro.insert(std::make_pair(rtabmapToToro.at(iter->first), rtabmap::Link(rtabmapToToro.at(iter->first), rtabmapToToro.at(iter->second.to()), iter->second.transform(), iter->second.type())));
			}
		}

		std::map<int, rtabmap::Transform> optimizedPosesToro;

		// Optimize!
		if(posesToro.size() && edgeConstraintsToro.size())
		{
			std::list<std::map<int, rtabmap::Transform> > graphesToro;
			rtabmap::util3d::optimizeTOROGraph(posesToro, edgeConstraintsToro, optimizedPosesToro, toroIterations, toroInitialGuess, &graphesToro);
			for(std::map<int, rtabmap::Transform>::iterator iter=optimizedPosesToro.begin(); iter!=optimizedPosesToro.end(); ++iter)
			{
				optimizedPoses.insert(std::make_pair(toroToRtabmap.at(iter->first), iter->second));
			}

			if(intermediateGraphes)
			{
				for(std::list<std::map<int, rtabmap::Transform> >::iterator iter = graphesToro.begin(); iter!=graphesToro.end(); ++iter)
				{
					std::map<int, rtabmap::Transform> tmp;
					for(std::map<int, rtabmap::Transform>::iterator jter=iter->begin(); jter!=iter->end(); ++jter)
					{
						tmp.insert(std::make_pair(toroToRtabmap.at(jter->first), jter->second));
					}
					intermediateGraphes->push_back(tmp);
				}
			}
		}
		else
		{
			UERROR("No TORO poses and constraints!?");
		}
	}
	else if(links.size() == 0 && poses.size() == 1)
	{
		optimizedPoses = poses;
	}
	else
	{
		UERROR("Wrong inputs! depthGraph=%d poses=%d links=%d",
				(int)depthGraph.size(), (int)poses.size(), (int)links.size());
	}
}

//On success, optimizedPoses is cleared and new poses are inserted in
void optimizeTOROGraph(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		std::map<int, Transform> & optimizedPoses,
		int toroIterations,
		bool toroInitialGuess,
		std::list<std::map<int, Transform> > * intermediateGraphes) // contains poses after tree init to last one before the end
{
	UASSERT(toroIterations>0);
	optimizedPoses.clear();
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

		UDEBUG("TORO iterate begin (iterations=%d)", toroIterations);
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

		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			AISNavigation::TreePoseGraph<AISNavigation::Operations3D<double> >::Vertex* v=pg.vertex(iter->first);
			v->pose=v->transformation.toPoseType();
			Transform newPose = transformFromEigen3f(pcl::getTransformation(v->pose.x(), v->pose.y(), v->pose.z(), v->pose.roll(), v->pose.pitch(), v->pose.yaw()));

			optimizedPoses.insert(std::pair<int, Transform>(iter->first, newPose));
		}

		//Eigen::Matrix4f newPose = transformToEigen4f(optimizedPoses.at(poses.rbegin()->first));
		//Eigen::Matrix4f oldPose = transformToEigen4f(poses.rbegin()->second);
		//Eigen::Matrix4f poseCorrection = oldPose.inverse() * newPose; // transform from odom to correct odom
		//Eigen::Matrix4f result = oldPose*poseCorrection*oldPose.inverse();
		//mapCorrection = transformFromEigen4f(result);
	}
	else if(edgeConstraints.size() == 0 && poses.size() == 1)
	{
		optimizedPoses = poses;
	}
	else
	{
		UWARN("This method should be called at least with 1 pose!");
	}
}

bool saveTOROGraph(
		const std::string & fileName,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints)
{
	FILE * file = 0;

#ifdef _MSC_VER
	fopen_s(&file, fileName.c_str(), "w");
#else
	file = fopen(fileName.c_str(), "w");
#endif

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
	FILE * file = 0;
#ifdef _MSC_VER
	fopen_s(&file, fileName.c_str(), "r");
#else
	file = fopen(fileName.c_str(), "r");
#endif

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


std::map<int, Transform> radiusPosesFiltering(const std::map<int, Transform> & poses, float radius, float angle, bool keepLatest)
{
	if(poses.size() > 1 && radius > 0.0f && angle>0.0f)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(poses.size());
		int i=0;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			(*cloud)[i++] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
		}

		// radius filtering
		std::vector<int> names = uKeys(poses);
		std::vector<Transform> transforms = uValues(poses);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> (false));
		tree->setInputCloud(cloud);
		std::set<int> indicesChecked;
		std::set<int> indicesKept;

		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			// ignore scans
			if(indicesChecked.find(i) == indicesChecked.end())
			{
				std::vector<int> kIndices;
				std::vector<float> kDistances;
				tree->radiusSearch(cloud->at(i), radius, kIndices, kDistances);

				std::set<int> cloudIndices;
				const Transform & currentT = transforms.at(i);
				Eigen::Vector3f vA = util3d::transformToEigen3f(currentT).rotation()*Eigen::Vector3f(1,0,0);
				for(unsigned int j=0; j<kIndices.size(); ++j)
				{
					if(indicesChecked.find(kIndices[j]) == indicesChecked.end())
					{
						const Transform & checkT = transforms.at(kIndices[j]);
						// same orientation?
						Eigen::Vector3f vB = util3d::transformToEigen3f(checkT).rotation()*Eigen::Vector3f(1,0,0);
						double a = pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
						if(a <= angle)
						{
							cloudIndices.insert(kIndices[j]);
						}
					}
				}

				if(keepLatest)
				{
					bool lastAdded = false;
					for(std::set<int>::reverse_iterator iter = cloudIndices.rbegin(); iter!=cloudIndices.rend(); ++iter)
					{
						if(!lastAdded)
						{
							indicesKept.insert(*iter);
							lastAdded = true;
						}
						indicesChecked.insert(*iter);
					}
				}
				else
				{
					bool firstAdded = false;
					for(std::set<int>::iterator iter = cloudIndices.begin(); iter!=cloudIndices.end(); ++iter)
					{
						if(!firstAdded)
						{
							indicesKept.insert(*iter);
							firstAdded = true;
						}
						indicesChecked.insert(*iter);
					}
				}
			}
		}

		//pcl::IndicesPtr indicesOut(new std::vector<int>);
		//indicesOut->insert(indicesOut->end(), indicesKept.begin(), indicesKept.end());
		UINFO("Cloud filtered In = %d, Out = %d", cloud->size(), indicesKept.size());
		//pcl::io::savePCDFile("duplicateIn.pcd", *cloud);
		//pcl::io::savePCDFile("duplicateOut.pcd", *cloud, *indicesOut);

		std::map<int, Transform> keptPoses;
		for(std::set<int>::iterator iter = indicesKept.begin(); iter!=indicesKept.end(); ++iter)
		{
			keptPoses.insert(std::make_pair(names.at(*iter), transforms.at(*iter)));
		}

		return keptPoses;
	}
	else
	{
		return poses;
	}
}

std::multimap<int, int> radiusPosesClustering(const std::map<int, Transform> & poses, float radius, float angle)
{
	std::multimap<int, int> clusters;
	if(poses.size() > 1 && radius > 0.0f && angle>0.0f)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(poses.size());
		int i=0;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			(*cloud)[i++] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
		}

		// radius clustering (nearest neighbors)
		std::vector<int> ids = uKeys(poses);
		std::vector<Transform> transforms = uValues(poses);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> (false));
		tree->setInputCloud(cloud);

		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			std::vector<int> kIndices;
			std::vector<float> kDistances;
			tree->radiusSearch(cloud->at(i), radius, kIndices, kDistances);

			std::set<int> cloudIndices;
			const Transform & currentT = transforms.at(i);
			Eigen::Vector3f vA = util3d::transformToEigen3f(currentT).rotation()*Eigen::Vector3f(1,0,0);
			for(unsigned int j=0; j<kIndices.size(); ++j)
			{
				if((int)i != kIndices[j])
				{
					const Transform & checkT = transforms.at(kIndices[j]);
					// same orientation?
					Eigen::Vector3f vB = util3d::transformToEigen3f(checkT).rotation()*Eigen::Vector3f(1,0,0);
					double a = pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
					if(a <= angle)
					{
						clusters.insert(std::make_pair(ids[i], ids[kIndices[j]]));
					}
				}
			}
		}
	}
	return clusters;
}

bool occupancy2DFromCloud3D(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		cv::Mat & ground,
		cv::Mat & obstacles,
		float cellSize,
		float groundNormalAngle,
		int minClusterSize)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZ>);

	//voxelize
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelizedCloud = util3d::voxelize<pcl::PointXYZRGB>(cloud, cellSize);

	pcl::IndicesPtr groundIndices, obstaclesIndices;

	segmentObstaclesFromGround<pcl::PointXYZRGB>(cloud,
			groundIndices,
			obstaclesIndices,
			cellSize,
			groundNormalAngle,
			minClusterSize);

	if(groundIndices->size())
	{
		pcl::copyPointCloud(*cloud, *groundIndices, *groundCloud);
		//project on XY plane
		util3d::projectCloudOnXYPlane<pcl::PointXYZ>(groundCloud);
		//voxelize to grid cell size
		groundCloud = util3d::voxelize<pcl::PointXYZ>(groundCloud, cellSize);
	}

	if(obstaclesIndices->size())
	{
		pcl::copyPointCloud(*cloud, *obstaclesIndices, *obstaclesCloud);
		//project on XY plane
		util3d::projectCloudOnXYPlane<pcl::PointXYZ>(obstaclesCloud);
		//voxelize to grid cell size
		obstaclesCloud = util3d::voxelize<pcl::PointXYZ>(obstaclesCloud, cellSize);
	}

	ground = cv::Mat();
	if(groundCloud->size())
	{
		ground = cv::Mat(groundCloud->size(), 1, CV_32FC2);
		for(unsigned int i=0;i<groundCloud->size(); ++i)
		{
			ground.at<cv::Vec2f>(i)[0] = groundCloud->at(i).x;
			ground.at<cv::Vec2f>(i)[1] = groundCloud->at(i).y;
		}
	}

	obstacles = cv::Mat();
	if(obstaclesCloud->size())
	{
		obstacles = cv::Mat(obstaclesCloud->size(), 1, CV_32FC2);
		for(unsigned int i=0;i<obstaclesCloud->size(); ++i)
		{
			obstacles.at<cv::Vec2f>(i)[0] = obstaclesCloud->at(i).x;
			obstacles.at<cv::Vec2f>(i)[1] = obstaclesCloud->at(i).y;
		}
	}
	/*
	if(cloud->size())
	{
		UWARN("saving cloud");
		pcl::io::savePCDFile("cloud.pcd", *cloud);
		pcl::io::savePCDFile("cloudXYZ.pcd", *cloudXYZ);
	}
	if(groundCloud->size())
	{
		UWARN("saving ground");
		pcl::io::savePCDFile("ground.pcd", *groundCloud);
		pcl::io::savePCDFile("ground_indices.pcd", *cloudXYZ, *groundIndices);
	}
	if(obstaclesCloud->size())
	{
		UWARN("saving obstacles");
		pcl::io::savePCDFile("obstacles.pcd", *obstaclesCloud);
		pcl::io::savePCDFile("obstacles_indices.pcd", *cloudXYZ, *obstaclesIndices);
	}
	*/
	return !ground.empty();
}

/**
 * Create 2d Occupancy grid (CV_8S) from 2d occupancy
 * -1 = unknown
 * 0 = empty space
 * 100 = obstacle
 * @param poses
 * @param occupancy <empty, occupied>
 * @param cellSize m
 * @param xMin
 * @param yMin
 * @param fillEmptyRadius fill neighbors of empty space if there're no obstacles.
 */
cv::Mat create2DMapFromOccupancyLocalMaps(
		const std::map<int, Transform> & poses,
		const std::map<int, std::pair<cv::Mat, cv::Mat> > & occupancy,
		float cellSize,
		float & xMin,
		float & yMin,
		int fillEmptyRadius,
		float minMapSize)
{
	UASSERT(fillEmptyRadius >= 0);
	UASSERT(minMapSize >= 0.0f);
	UDEBUG("");
	UTimer timer;

	std::map<int, cv::Mat> emptyLocalMaps;
	std::map<int, cv::Mat> occupiedLocalMaps;

	float minX=-minMapSize/2.0, minY=-minMapSize/2.0, maxX=minMapSize/2.0, maxY=minMapSize/2.0;
	bool undefinedSize = minMapSize == 0.0f;
	float x,y,z,toll,pitch,yaw,cosT,sinT;
	cv::Mat affineTransform(2,3,CV_32FC1);
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(uContains(occupancy, iter->first))
		{
			const std::pair<cv::Mat, cv::Mat> & pair = occupancy.at(iter->first);

			iter->second.getTranslationAndEulerAngles(x,y,z,toll,pitch,yaw);
			cosT = cos(yaw);
			sinT = sin(yaw);
			affineTransform.at<float>(0,0) = cosT;
			affineTransform.at<float>(0,1) = -sinT;
			affineTransform.at<float>(1,0) = sinT;
			affineTransform.at<float>(1,1) = cosT;
			affineTransform.at<float>(0,2) = x;
			affineTransform.at<float>(1,2) = y;

			if(undefinedSize)
			{
				minX = maxX = x;
				minY = maxY = y;
				undefinedSize = false;
			}
			else
			{
				if(minX > x)
					minX = x;
				else if(maxX < x)
					maxX = x;

				if(minY > y)
					minY = y;
				else if(maxY < y)
					maxY = y;
			}

			//ground
			if(pair.first.rows)
			{
				UASSERT(pair.first.type() == CV_32FC2);
				cv::Mat ground(pair.first.rows, pair.first.cols, pair.first.type());
				cv::transform(pair.first, ground, affineTransform);
				for(int i=0; i<ground.rows; ++i)
				{
					if(minX > ground.at<float>(i,0))
						minX = ground.at<float>(i,0);
					else if(maxX < ground.at<float>(i,0))
						maxX = ground.at<float>(i,0);

					if(minY > ground.at<float>(i,1))
						minY = ground.at<float>(i,1);
					else if(maxY < ground.at<float>(i,1))
						maxY = ground.at<float>(i,1);
				}
				emptyLocalMaps.insert(std::make_pair(iter->first, ground));
			}

			//obstacles
			if(pair.second.rows)
			{
				UASSERT(pair.second.type() == CV_32FC2);
				cv::Mat obstacles(pair.second.rows, pair.second.cols, pair.second.type());
				cv::transform(pair.second, obstacles, affineTransform);
				for(int i=0; i<obstacles.rows; ++i)
				{
					if(minX > obstacles.at<float>(i,0))
						minX = obstacles.at<float>(i,0);
					else if(maxX < obstacles.at<float>(i,0))
						maxX = obstacles.at<float>(i,0);

					if(minY > obstacles.at<float>(i,1))
						minY = obstacles.at<float>(i,1);
					else if(maxY < obstacles.at<float>(i,1))
						maxY = obstacles.at<float>(i,1);
				}
				occupiedLocalMaps.insert(std::make_pair(iter->first, obstacles));
			}
		}
	}
	UDEBUG("timer=%fs", timer.ticks());

	cv::Mat map;
	if(minX != maxX && minY != maxY)
	{
		//Get map size
		float margin = (fillEmptyRadius + 1)*cellSize;
		xMin = minX-margin;
		yMin = minY-margin;
		float xMax = maxX+margin;
		float yMax = maxY+margin;
		UDEBUG("map min=(%f, %f) max=(%f,%f)", xMin, yMin, xMax, yMax);

		map = cv::Mat::ones((yMax - yMin) / cellSize + 0.5f, (xMax - xMin) / cellSize + 0.5f, CV_8S)*-1;
		for(std::map<int, Transform>::const_iterator kter = poses.begin(); kter!=poses.end(); ++kter)
		{
			std::map<int, cv::Mat >::iterator iter = emptyLocalMaps.find(kter->first);
			std::map<int, cv::Mat >::iterator jter = occupiedLocalMaps.find(kter->first);
			if(iter!=emptyLocalMaps.end())
			{
				for(int i=0; i<iter->second.rows; ++i)
				{
					cv::Point2i pt((iter->second.at<float>(i,0)-xMin)/cellSize + 0.5f, (iter->second.at<float>(i,1)-yMin)/cellSize + 0.5f);
					map.at<char>(pt.y, pt.x) = 0; // free space
					if(fillEmptyRadius>0)
					{
						for(int j=pt.y-fillEmptyRadius; j<=pt.y+fillEmptyRadius; ++j)
						{
							for(int k=pt.x-fillEmptyRadius; k<=pt.x+fillEmptyRadius; ++k)
							{
								if(map.at<char>(j, k) == -1)
								{
									map.at<char>(j, k) = 0;
								}
							}
						}
					}
				}
			}
			if(jter!=occupiedLocalMaps.end())
			{
				for(int i=0; i<jter->second.rows; ++i)
				{
					cv::Point2i pt((jter->second.at<float>(i,0)-xMin)/cellSize + 0.5f, (jter->second.at<float>(i,1)-yMin)/cellSize + 0.5f);
					map.at<char>(pt.y, pt.x) = 100; // obstacles
				}
			}
			//UDEBUG("empty=%d occupied=%d", empty, occupied);
		}
	}
	UDEBUG("timer=%fs", timer.ticks());
	return map;
}

/**
 * Create 2d Occupancy grid (CV_8S)
 * -1 = unknown
 * 0 = empty space
 * 100 = obstacle
 * @param poses
 * @param scans
 * @param cellSize m
 * @param unknownSpaceFilled if false no fill, otherwise a virtual laser sweeps the unknown space from each pose (stopping on detected obstacle)
 * @param xMin
 * @param yMin
 */
cv::Mat create2DMap(const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > & scans,
		float cellSize,
		bool unknownSpaceFilled,
		float & xMin,
		float & yMin,
		float minMapSize)
{
	UDEBUG("poses=%d, scans = %d", poses.size(), scans.size());
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > localScans;

	pcl::PointCloud<pcl::PointXYZ> minMax;
	if(minMapSize > 0.0f)
	{
		minMax.push_back(pcl::PointXYZ(-minMapSize/2.0, -minMapSize/2.0, 0));
		minMax.push_back(pcl::PointXYZ(minMapSize/2.0, minMapSize/2.0, 0));
	}
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(uContains(scans, iter->first) && scans.at(iter->first)->size())
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = transformPointCloud<pcl::PointXYZ>(scans.at(iter->first), iter->second);
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

		xMin = min.x-cellSize;
		yMin = min.y-cellSize;
		float xMax = max.x+cellSize;
		float yMax = max.y+cellSize;

		UDEBUG("map min=(%f, %f) max=(%f,%f)", xMin, yMin, xMax, yMax);

		//UTimer timer;

		map = cv::Mat::ones((yMax - yMin) / cellSize + 0.5f, (xMax - xMin) / cellSize + 0.5f, CV_8S)*-1;
		std::vector<float> maxSquaredLength(localScans.size(), 0.0f);
		int j=0;
		for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter = localScans.begin(); iter!=localScans.end(); ++iter)
		{
			const Transform & pose = poses.at(iter->first);
			cv::Point2i start((pose.x()-xMin)/cellSize + 0.5f, (pose.y()-yMin)/cellSize + 0.5f);
			for(unsigned int i=0; i<iter->second->size(); ++i)
			{
				cv::Point2i end((iter->second->points[i].x-xMin)/cellSize + 0.5f, (iter->second->points[i].y-yMin)/cellSize + 0.5f);
				map.at<char>(end.y, end.x) = 100; // obstacle
				rayTrace(start, end, map, true); // trace free space

				if(unknownSpaceFilled)
				{
					float dx = iter->second->points[i].x - pose.x();
					float dy = iter->second->points[i].y - pose.y();
					float l = dx*dx + dy*dy;
					if(l > maxSquaredLength[j])
					{
						maxSquaredLength[j] = l;
					}
				}
			}
			++j;
		}
		//UWARN("timer=%fs", timer.ticks());

		// now fill unknown spaces
		if(unknownSpaceFilled)
		{
			j=0;
			for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter = localScans.begin(); iter!=localScans.end(); ++iter)
			{
				if(iter->second->size() > 1 && maxSquaredLength[j] > 0.0f)
				{
					float maxLength = sqrt(maxSquaredLength[j]);
					if(maxLength > cellSize)
					{
						// compute angle
						float a = (CV_PI/2.0f) /  (maxLength / cellSize);
						//UWARN("a=%f PI/256=%f", a, CV_PI/256.0f);
						UASSERT_MSG(a >= 0 && a < 5.0f*CV_PI/8.0f, uFormat("a=%f length=%f cell=%f", a, maxLength, cellSize).c_str());

						const Transform & pose = poses.at(iter->first);
						cv::Point2i start((pose.x()-xMin)/cellSize + 0.5f, (pose.y()-yMin)/cellSize + 0.5f);

						//UWARN("maxLength = %f", maxLength);
						//rotate counterclockwise from the first point until we pass the last point
						cv::Mat rotation = (cv::Mat_<float>(2,2) << cos(a), -sin(a),
																	 sin(a), cos(a));
						cv::Mat origin(2,1,CV_32F), endFirst(2,1,CV_32F), endLast(2,1,CV_32F);
						origin.at<float>(0) = pose.x();
						origin.at<float>(1) = pose.y();
						endFirst.at<float>(0) = iter->second->points[0].x;
						endFirst.at<float>(1) = iter->second->points[0].y;
						endLast.at<float>(0) = iter->second->points[iter->second->points.size()-1].x;
						endLast.at<float>(1) = iter->second->points[iter->second->points.size()-1].y;
						//UWARN("origin = %f %f", origin.at<float>(0), origin.at<float>(1));
						//UWARN("endFirst = %f %f", endFirst.at<float>(0), endFirst.at<float>(1));
						//UWARN("endLast = %f %f", endLast.at<float>(0), endLast.at<float>(1));
						cv::Mat tmp = (endFirst - origin);
						cv::Mat endRotated = rotation*((tmp/cv::norm(tmp))*maxLength) + origin;
						cv::Mat endLastVector(3,1,CV_32F), endRotatedVector(3,1,CV_32F);
						endLastVector.at<float>(0) = endLast.at<float>(0) - origin.at<float>(0);
						endLastVector.at<float>(1) = endLast.at<float>(1) - origin.at<float>(1);
						endLastVector.at<float>(2) = 0.0f;
						endRotatedVector.at<float>(0) = endRotated.at<float>(0) - origin.at<float>(0);
						endRotatedVector.at<float>(1) = endRotated.at<float>(1) - origin.at<float>(1);
						endRotatedVector.at<float>(2) = 0.0f;
						//UWARN("endRotated = %f %f", endRotated.at<float>(0), endRotated.at<float>(1));
						while(endRotatedVector.cross(endLastVector).at<float>(2) > 0.0f)
						{
							cv::Point2i end((endRotated.at<float>(0)-xMin)/cellSize + 0.5f, (endRotated.at<float>(1)-yMin)/cellSize + 0.5f);
							//end must be inside the grid
							end.x = end.x < 0?0:end.x;
							end.x = end.x >= map.cols?map.cols-1:end.x;
							end.y = end.y < 0?0:end.y;
							end.y = end.y >= map.rows?map.rows-1:end.y;
							rayTrace(start, end, map, true); // trace free space

							// next point
							endRotated = rotation*(endRotated - origin) + origin;
							endRotatedVector.at<float>(0) = endRotated.at<float>(0) - origin.at<float>(0);
							endRotatedVector.at<float>(1) = endRotated.at<float>(1) - origin.at<float>(1);
							//UWARN("endRotated = %f %f", endRotated.at<float>(0), endRotated.at<float>(1));
						}
					}
				}
				++j;
				//UWARN("timer=%fs", timer.ticks());
			}
		}
	}
	return map;
}

void rayTrace(const cv::Point2i & start, const cv::Point2i & end, cv::Mat & grid, bool stopOnObstacle)
{
	UASSERT_MSG(start.x >= 0 && start.x < grid.cols, uFormat("start.x=%d grid.cols=%d", start.x, grid.cols).c_str());
	UASSERT_MSG(start.y >= 0 && start.y < grid.rows, uFormat("start.y=%d grid.rows=%d", start.y, grid.rows).c_str());
	UASSERT_MSG(end.x >= 0 && end.x < grid.cols, uFormat("end.x=%d grid.cols=%d", end.x, grid.cols).c_str());
	UASSERT_MSG(end.y >= 0 && end.y < grid.rows, uFormat("end.x=%d grid.cols=%d", end.y, grid.rows).c_str());

	cv::Point2i ptA, ptB;
	ptA = start;
	ptB = end;

	float slope = float(ptB.y - ptA.y)/float(ptB.x - ptA.x);
	float b = ptA.y - slope*ptA.x;

	//UWARN("start=%d,%d end=%d,%d", ptA.x, ptA.y, ptB.x, ptB.y);

	//ROS_WARN("y = %f*x + %f", slope, b);
	for(int x=ptA.x; ptA.x<ptB.x?x<ptB.x:x>ptB.x; ptA.x<ptB.x?++x:--x)
	{
		int lowerbound = float(x)*slope + b;
		int upperbound = float(ptA.x<ptB.x?x+1:x-1)*slope + b;

		if(lowerbound > upperbound)
		{
			int tmp = lowerbound;
			lowerbound = upperbound;
			upperbound = tmp;
		}

		//ROS_WARN("lowerbound=%f upperbound=%f", lowerbound, upperbound);
		UASSERT_MSG(lowerbound >= 0 && lowerbound < grid.rows, uFormat("lowerbound=%f grid.rows=%d x=%d slope=%f b=%f x=%f", lowerbound, grid.rows, x, slope, b, x).c_str());
		UASSERT_MSG(upperbound >= 0 && upperbound < grid.rows, uFormat("upperbound=%f grid.rows=%d x+1=%d slope=%f b=%f x=%f", upperbound, grid.rows, x+1, slope, b, x).c_str());
		// verify if there is no obstacle
		bool stopped = false;
		if(stopOnObstacle)
		{
			for(int y = lowerbound; y<=(int)upperbound; ++y)
			{
				if(grid.at<char>(y, x) == 100)
				{
					stopped = true;
					break;
				}
			}
		}
		if(stopped)
		{
			break;
		}
		for(int y = lowerbound; y<=(int)upperbound; ++y)
		{
			grid.at<char>(y, x) = 0; // free space
		}
	}
}

//convert to gray scaled map
cv::Mat convertMap2Image8U(const cv::Mat & map8S)
{
	UASSERT(map8S.channels() == 1 && map8S.type() == CV_8S);
	cv::Mat map8U = cv::Mat(map8S.rows, map8S.cols, CV_8U);
	for (int i = 0; i < map8S.rows; ++i)
	{
		for (int j = 0; j < map8S.cols; ++j)
		{
			char v = map8S.at<char>(i, j);
			unsigned char gray;
			if(v == 0)
			{
				gray = 178;
			}
			else if(v == 100)
			{
				gray = 0;
			}
			else // -1
			{
				gray = 89;
			}
			map8U.at<unsigned char>(i, j) = gray;
		}
	}
	return map8U;
}

pcl::IndicesPtr concatenate(const std::vector<pcl::IndicesPtr> & indices)
{
	//compute total size
	unsigned int totalSize = 0;
	for(unsigned int i=0; i<indices.size(); ++i)
	{
		totalSize += indices[i]->size();
	}
	pcl::IndicesPtr ind(new std::vector<int>(totalSize));
	unsigned int io = 0;
	for(unsigned int i=0; i<indices.size(); ++i)
	{
		for(unsigned int j=0; j<indices[i]->size(); ++j)
		{
			ind->at(io++) = indices[i]->at(j);
		}
	}
	return ind;
}

pcl::IndicesPtr concatenate(const pcl::IndicesPtr & indicesA, const pcl::IndicesPtr & indicesB)
{
	pcl::IndicesPtr ind(new std::vector<int>(*indicesA));
	ind->resize(ind->size()+indicesB->size());
	unsigned int oi = indicesA->size();
	for(unsigned int i=0; i<indicesB->size(); ++i)
	{
		ind->at(oi++) = indicesB->at(i);
	}
	return ind;
}

}

}
