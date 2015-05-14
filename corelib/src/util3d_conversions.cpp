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

#include "rtabmap/core/util3d_conversions.h"

#include "rtabmap/utilite/ULogger.h"
#include <pcl/common/transforms.h>

namespace rtabmap
{

namespace util3d
{

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

cv::Mat laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC2);
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		laserScan.at<cv::Vec2f>(i)[0] = cloud.at(i).x;
		laserScan.at<cv::Vec2f>(i)[1] = cloud.at(i).y;
	}
	return laserScan;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanToPointCloud(const cv::Mat & laserScan)
{
	UASSERT(laserScan.empty() || laserScan.type() == CV_32FC2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	output->resize(laserScan.cols);
	for(int i=0; i<laserScan.cols; ++i)
	{
		output->at(i).x = laserScan.at<cv::Vec2f>(i)[0];
		output->at(i).y = laserScan.at<cv::Vec2f>(i)[1];
	}
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cvMat2Cloud(
		const cv::Mat & matrix,
		const Transform & tranform)
{
	UASSERT(matrix.type() == CV_32FC2 || matrix.type() == CV_32FC3);
	UASSERT(matrix.rows == 1);

	Eigen::Affine3f t = tranform.toEigen3f();
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

}

}
