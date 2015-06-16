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

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace rtabmap
{

namespace util3d
{

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

	float depth = util2d::getDepth(depthImage, x, y, smoothing, maxZError);
	if(depth)
	{
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
	UASSERT(imageDepth.rows % decimation == 0);
	UASSERT(imageDepth.cols % decimation == 0);

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
	UASSERT(imageDepth.rows % decimation == 0);
	UASSERT(imageDepth.cols % decimation == 0);

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
	UASSERT(imageDisparity.rows % decimation == 0);
	UASSERT(imageDisparity.cols % decimation == 0);
	UASSERT(decimation >= 1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

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
	UASSERT(!imageRgb.empty() && !imageDisparity.empty());
	UASSERT(imageRgb.rows == imageDisparity.rows &&
			imageRgb.cols == imageDisparity.cols &&
			(imageDisparity.type() == CV_32FC1 || imageDisparity.type()==CV_16SC1));
	UASSERT(imageRgb.channels() == 3 || imageRgb.channels() == 1);
	UASSERT(decimation >= 1);
	UASSERT(imageDisparity.rows % decimation == 0 && imageDisparity.cols % decimation == 0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	bool mono;
	if(imageRgb.channels() == 3) // BGR
	{
		mono = false;
	}
	else // Mono
	{
		mono = true;
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
	UASSERT(!imageLeft.empty() && !imageRight.empty());
	UASSERT(imageRight.type() == CV_8UC1);
	UASSERT(imageLeft.channels() == 3 || imageLeft.channels() == 1);
	UASSERT(imageLeft.rows == imageRight.rows &&
			imageLeft.cols == imageRight.cols);
	UASSERT(decimation >= 1);

	cv::Mat leftColor = imageLeft;
	cv::Mat rightMono = imageRight;

	if(leftColor.rows % decimation != 0 ||
	   leftColor.cols % decimation != 0)
	{
		leftColor = util2d::decimate(leftColor, decimation);
		rightMono = util2d::decimate(rightMono, decimation);
		fx /= float(decimation);
		cx /= float(decimation);
		cy /= float(decimation);
		decimation = 1;
	}

	cv::Mat leftMono;
	if(leftColor.channels() == 3)
	{
		cv::cvtColor(leftColor, leftMono, CV_BGR2GRAY);
	}
	else
	{
		leftMono = leftColor;
	}

	return cloudFromDisparityRGB(
			leftColor,
			util2d::disparityFromStereoImages(leftMono, rightMono),
			cx, cy,
			fx, baseline,
			decimation);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromSensorData(
		const SensorData & sensorData,
		int decimation,
		float maxDepth,
		float voxelSize,
		int samples)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	if(!sensorData.depthRaw().empty() && sensorData.cameraModels().size())
	{
		//depth
		UASSERT(int((sensorData.depthRaw().cols/sensorData.cameraModels().size())*sensorData.cameraModels().size()) == sensorData.depthRaw().cols);
		int subImageWidth = sensorData.depthRaw().cols/sensorData.cameraModels().size();
		cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
		for(unsigned int i=0; i<sensorData.cameraModels().size(); ++i)
		{
			if(sensorData.cameraModels()[i].isValid())
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = util3d::cloudFromDepth(
						cv::Mat(sensorData.depthRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, sensorData.depthRaw().rows)),
						sensorData.cameraModels()[i].cx(),
						sensorData.cameraModels()[i].cy(),
						sensorData.cameraModels()[i].fx(),
						sensorData.cameraModels()[i].fy(),
						decimation);

				if(tmp->size())
				{
					bool filtered = false;
					if(tmp->size() && maxDepth)
					{
						tmp = util3d::passThrough(tmp, "z", 0, maxDepth);
						filtered = true;
					}

					if(tmp->size() && voxelSize)
					{
						tmp = util3d::voxelize(tmp, voxelSize);
						filtered = true;
					}

					if(tmp->size() && samples)
					{
						tmp = util3d::sampling(tmp, samples);
						filtered = true;
					}

					if(tmp->size() && !filtered)
					{
						tmp = util3d::removeNaNFromPointCloud(tmp);
					}

					if(tmp->size())
					{
						tmp = util3d::transformPointCloud(tmp, sensorData.cameraModels()[i].localTransform());
					}

					*cloud += *tmp;
				}
			}
			else
			{
				UERROR("Camera model %d is invalid", i);
			}
		}

		if(cloud->size() && voxelSize)
		{
			cloud = util3d::voxelize(cloud, voxelSize);
		}
	}
	else if(!sensorData.imageRaw().empty() && !sensorData.rightRaw().empty() && sensorData.stereoCameraModel().isValid())
	{
		//stereo
		UASSERT(sensorData.rightRaw().type() == CV_8UC1);

		cv::Mat leftMono;
		if(sensorData.imageRaw().channels() == 3)
		{
			cv::cvtColor(sensorData.imageRaw(), leftMono, CV_BGR2GRAY);
		}
		else
		{
			leftMono = sensorData.imageRaw();
		}
		cloud = cloudFromDisparity(
				util2d::disparityFromStereoImages(leftMono, sensorData.rightRaw()),
				sensorData.stereoCameraModel().left().cx(),
				sensorData.stereoCameraModel().left().cy(),
				sensorData.stereoCameraModel().left().fx(),
				sensorData.stereoCameraModel().baseline(),
				decimation);

		if(cloud->size())
		{
			bool filtered = false;
			if(cloud->size() && maxDepth)
			{
				cloud = util3d::passThrough(cloud, "z", 0, maxDepth);
				filtered = true;
			}

			if(cloud->size() && voxelSize)
			{
				cloud = util3d::voxelize(cloud, voxelSize);
				filtered = true;
			}

			if(cloud->size() && !filtered)
			{
				cloud = util3d::removeNaNFromPointCloud(cloud);
			}

			if(cloud->size())
			{
				cloud = util3d::transformPointCloud(cloud, sensorData.stereoCameraModel().left().localTransform());
			}
		}
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudRGBFromSensorData(
		const SensorData & sensorData,
		int decimation,
		float maxDepth,
		float voxelSize,
		int samples)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	if(!sensorData.imageRaw().empty())
	{
		if(!sensorData.depthRaw().empty() && sensorData.cameraModels().size())
		{
			//depth
			UASSERT(int((sensorData.imageRaw().cols/sensorData.cameraModels().size())*sensorData.cameraModels().size()) == sensorData.imageRaw().cols);
			UASSERT(sensorData.depthRaw().size() == sensorData.imageRaw().size());
			int subImageWidth = sensorData.imageRaw().cols/sensorData.cameraModels().size();
			cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
			for(unsigned int i=0; i<sensorData.cameraModels().size(); ++i)
			{
				if(sensorData.cameraModels()[i].isValid())
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = util3d::cloudFromDepthRGB(
							cv::Mat(sensorData.imageRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, sensorData.imageRaw().rows)),
							cv::Mat(sensorData.depthRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, sensorData.depthRaw().rows)),
							sensorData.cameraModels()[i].cx(),
							sensorData.cameraModels()[i].cy(),
							sensorData.cameraModels()[i].fx(),
							sensorData.cameraModels()[i].fy(),
							decimation);

					if(tmp->size())
					{
						bool filtered = false;
						if(tmp->size() && maxDepth)
						{
							tmp = util3d::passThrough(tmp, "z", 0, maxDepth);
							filtered = true;
						}

						if(tmp->size() && voxelSize)
						{
							tmp = util3d::voxelize(tmp, voxelSize);
							filtered = true;
						}

						if(tmp->size() && samples)
						{
							tmp = util3d::sampling(tmp, samples);
							filtered = true;
						}

						if(tmp->size() && !filtered)
						{
							tmp = util3d::removeNaNFromPointCloud(tmp);
						}

						if(tmp->size())
						{
							tmp = util3d::transformPointCloud(tmp, sensorData.cameraModels()[i].localTransform());
						}

						*cloud += *tmp;
					}
				}
				else
				{
					UERROR("Camera model %d is invalid", i);
				}
			}

			if(cloud->size() && voxelSize)
			{
				cloud = util3d::voxelize(cloud, voxelSize);
			}
		}
		else if(!sensorData.rightRaw().empty() && sensorData.stereoCameraModel().isValid())
		{
			//stereo
			cloud = cloudFromStereoImages(sensorData.imageRaw(),
					sensorData.rightRaw(),
					sensorData.stereoCameraModel().left().cx(),
					sensorData.stereoCameraModel().left().cy(),
					sensorData.stereoCameraModel().left().fx(),
					sensorData.stereoCameraModel().baseline(),
					decimation);

			if(cloud->size())
			{
				bool filtered = false;
				if(cloud->size() && maxDepth)
				{
					cloud = util3d::passThrough(cloud, "z", 0, maxDepth);
					filtered = true;
				}

				if(cloud->size() && voxelSize)
				{
					cloud = util3d::voxelize(cloud, voxelSize);
					filtered = true;
				}

				if(cloud->size() && !filtered)
				{
					cloud = util3d::removeNaNFromPointCloud(cloud);
				}

				if(cloud->size())
				{
					cloud = util3d::transformPointCloud(cloud, sensorData.stereoCameraModel().left().localTransform());
				}
			}
		}
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ> laserScanFromDepthImage(
					const cv::Mat & depthImage,
					float fx,
					float fy,
					float cx,
					float cy,
					float maxDepth,
					const Transform & localTransform)
{
	UASSERT(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1);
	UASSERT(!localTransform.isNull());

	pcl::PointCloud<pcl::PointXYZ> scan;
	int middle = depthImage.rows/2;
	if(middle)
	{
		scan.resize(depthImage.cols);
		int oi = 0;
		for(int i=0; i<depthImage.cols; ++i)
		{
			pcl::PointXYZ pt = util3d::projectDepthTo3D(depthImage, i, middle, cx, cy, fx, fy, false);
			if(pcl::isFinite(pt) && (maxDepth == 0 || pt.z < maxDepth))
			{
				if(!localTransform.isIdentity())
				{
					pt = util3d::transformPoint(pt, localTransform);
				}
				scan[oi++] = pt;
			}
		}
		scan.resize(oi);
	}
	return scan;
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
	if(uIsInBounds(u, 0, disparity.cols) &&
	   uIsInBounds(v, 0, disparity.rows))
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

pcl::IndicesPtr concatenate(const std::vector<pcl::IndicesPtr> & indices)
{
	//compute total size
	unsigned int totalSize = 0;
	for(unsigned int i=0; i<indices.size(); ++i)
	{
		totalSize += (unsigned int)indices[i]->size();
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
	unsigned int oi = (unsigned int)indicesA->size();
	for(unsigned int i=0; i<indicesB->size(); ++i)
	{
		ind->at(oi++) = indicesB->at(i);
	}
	return ind;
}

void savePCDWords(
		const std::string & fileName,
		const std::multimap<int, pcl::PointXYZ> & words,
		const Transform & transform)
{
	if(words.size())
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.resize(words.size());
		int i=0;
		for(std::multimap<int, pcl::PointXYZ>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
		{
			cloud[i++] = transformPoint(iter->second, transform);
		}
		pcl::io::savePCDFile(fileName, cloud);
	}
}

}

}
